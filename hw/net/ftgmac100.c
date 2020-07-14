/*
 * Faraday FTGMAC100 Gigabit Ethernet
 *
 * Copyright (C) 2016-2017, IBM Corporation.
 *
 * Based on Coldfire Fast Ethernet Controller emulation.
 *
 * Copyright (c) 2007 CodeSourcery.
 *
 * This code is licensed under the GPL version 2 or later. See the
 * COPYING file in the top-level directory.
 */

#include "qemu/osdep.h"
#include "hw/irq.h"
#include "hw/net/ftgmac100.h"
#include "sysemu/dma.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "net/checksum.h"
#include "net/eth.h"
#include "hw/net/mii.h"
#include "hw/qdev-properties.h"
#include "migration/vmstate.h"

/* For crc32 */
#include <zlib.h>
#define BIT(nr) (1UL << (nr))
/* Register definitions derived from Linux source */
#define SYS_REV_CTRL			0x00

#define SYS_PORT_CTRL			0x04
#define PORT_MODE_EXT_GPHY		3

#define GENET_SYS_OFF			0x0000
#define SYS_RBUF_FLUSH_CTRL		(GENET_SYS_OFF  + 0x08)
#define SYS_TBUF_FLUSH_CTRL		(GENET_SYS_OFF  + 0x0c)

#define GENET_EXT_OFF			0x0080
#define EXT_RGMII_OOB_CTRL		(GENET_EXT_OFF + 0x0c)
#define RGMII_LINK			BIT(4)
#define OOB_DISABLE			BIT(5)
#define RGMII_MODE_EN			BIT(6)
#define ID_MODE_DIS			BIT(16)

#define GENET_RBUF_OFF			0x0300
#define RBUF_TBUF_SIZE_CTRL		(GENET_RBUF_OFF + 0xb4)
#define RBUF_CTRL			(GENET_RBUF_OFF + 0x00)
#define RBUF_ALIGN_2B			BIT(1)

#define GENET_UMAC_OFF			0x0800
#define UMAC_MIB_CTRL			(GENET_UMAC_OFF + 0x580)
#define UMAC_MAX_FRAME_LEN		(GENET_UMAC_OFF + 0x014)
#define UMAC_MAC0			(GENET_UMAC_OFF + 0x00c)
#define UMAC_MAC1			(GENET_UMAC_OFF + 0x010)
#define UMAC_CMD			(GENET_UMAC_OFF + 0x008)
#define MDIO_CMD			(GENET_UMAC_OFF + 0x614)
#define UMAC_TX_FLUSH			(GENET_UMAC_OFF + 0x334)
#define MDIO_START_BUSY			BIT(29)
#define MDIO_READ_FAIL			BIT(28)
#define MDIO_RD				BIT(27)
#define MDIO_WR				BIT(26)
#define MDIO_PMD_SHIFT			21
#define MDIO_PMD_MASK			0x1f
#define MDIO_REG_SHIFT			16
#define MDIO_REG_MASK			0x1f

#define CMD_TX_EN			BIT(0)
#define CMD_RX_EN			BIT(1)
#define UMAC_SPEED_10			0
#define UMAC_SPEED_100			1
#define UMAC_SPEED_1000			2
#define UMAC_SPEED_2500			3
#define CMD_SPEED_SHIFT			2
#define CMD_SPEED_MASK			3
#define CMD_SW_RESET			BIT(13)
#define CMD_LCL_LOOP_EN			BIT(15)

#define MIB_RESET_RX			BIT(0)
#define MIB_RESET_RUNT			BIT(1)
#define MIB_RESET_TX			BIT(2)

/* total number of Buffer Descriptors, same for Rx/Tx */
#define TOTAL_DESCS			256
#define RX_DESCS			TOTAL_DESCS
#define TX_DESCS			TOTAL_DESCS

#define DEFAULT_Q			0x10

/* Body(1500) + EH_SIZE(14) + VLANTAG(4) + BRCMTAG(6) + FCS(4) = 1528.
 * 1536 is multiple of 256 bytes
 */
#define ENET_BRCM_TAG_LEN		6
#define ENET_PAD			8
#define ENET_MAX_MTU_SIZE		(ETH_DATA_LEN + ETH_HLEN +	 \
					 VLAN_HLEN + ENET_BRCM_TAG_LEN + \
					 ETH_FCS_LEN + ENET_PAD)

/* Tx/Rx Dma Descriptor common bits */
#define DMA_EN				BIT(0)
#define DMA_RING_BUF_EN_SHIFT		0x01
#define DMA_RING_BUF_EN_MASK		0xffff
#define DMA_BUFLENGTH_MASK		0x0fff
#define DMA_BUFLENGTH_SHIFT		16
#define DMA_RING_SIZE_SHIFT		16
#define DMA_OWN				0x8000
#define DMA_EOP				0x4000
#define DMA_SOP				0x2000
#define DMA_WRAP			0x1000
#define DMA_MAX_BURST_LENGTH		0x8
/* Tx specific DMA descriptor bits */
#define DMA_TX_UNDERRUN			0x0200
#define DMA_TX_APPEND_CRC		0x0040
#define DMA_TX_OW_CRC			0x0020
#define DMA_TX_DO_CSUM			0x0010
#define DMA_TX_QTAG_SHIFT		7

/* DMA rings size */
#define DMA_RING_SIZE			0x40
#define DMA_RINGS_SIZE			(DMA_RING_SIZE * (DEFAULT_Q + 1))

/* DMA descriptor */
#define DMA_DESC_LENGTH_STATUS		0x00
#define DMA_DESC_ADDRESS_LO		0x04
#define DMA_DESC_ADDRESS_HI		0x08
#define DMA_DESC_SIZE			12

#define GENET_RX_OFF			0x2000
#define GENET_RDMA_REG_OFF					\
	(GENET_RX_OFF + TOTAL_DESCS * DMA_DESC_SIZE)
#define GENET_TX_OFF			0x4000
#define GENET_TDMA_REG_OFF					\
	(GENET_TX_OFF + TOTAL_DESCS * DMA_DESC_SIZE)

#define DMA_FC_THRESH_HI		(RX_DESCS >> 4)
#define DMA_FC_THRESH_LO		5
#define DMA_FC_THRESH_VALUE		((DMA_FC_THRESH_LO << 16) |	\
					  DMA_FC_THRESH_HI)

#define DMA_XOFF_THRESHOLD_SHIFT	16

#define TDMA_RING_REG_BASE					\
	(GENET_TDMA_REG_OFF + DEFAULT_Q * DMA_RING_SIZE)
#define TDMA_READ_PTR			(TDMA_RING_REG_BASE + 0x00)
#define TDMA_CONS_INDEX			(TDMA_RING_REG_BASE + 0x08)
#define TDMA_PROD_INDEX			(TDMA_RING_REG_BASE + 0x0c)
#define DMA_P_INDEX_MASK		0xFFFF
#define DMA_C_INDEX_MASK		0xFFFF
#define DMA_RING_BUF_SIZE		0x10
#define DMA_START_ADDR			0x14
#define DMA_END_ADDR			0x1c
#define DMA_MBUF_DONE_THRESH		0x24
#define TDMA_FLOW_PERIOD		(TDMA_RING_REG_BASE + 0x28)
#define TDMA_WRITE_PTR			(TDMA_RING_REG_BASE + 0x2c)

#define RDMA_RING_REG_BASE					\
	(GENET_RDMA_REG_OFF + DEFAULT_Q * DMA_RING_SIZE)
#define RDMA_WRITE_PTR			(RDMA_RING_REG_BASE + 0x00)
#define RDMA_PROD_INDEX			(RDMA_RING_REG_BASE + 0x08)
#define RDMA_CONS_INDEX			(RDMA_RING_REG_BASE + 0x0c)
#define RDMA_XON_XOFF_THRESH		(RDMA_RING_REG_BASE + 0x28)
#define RDMA_READ_PTR			(RDMA_RING_REG_BASE + 0x2c)

#define TDMA_REG_BASE			(GENET_TDMA_REG_OFF + DMA_RINGS_SIZE)
#define RDMA_REG_BASE			(GENET_RDMA_REG_OFF + DMA_RINGS_SIZE)
#define DMA_RING_CFG			0x00
#define DMA_CTRL			0x04
#define DMA_SCB_BURST_SIZE		0x0c

#define RX_BUF_LENGTH			2048
#define RX_TOTAL_BUFSIZE		(RX_BUF_LENGTH * RX_DESCS)
#define RX_BUF_OFFSET			2


#define FTGMAC100_MATH0           0x10
#define FTGMAC100_MATH1           0x14
#define FTGMAC100_NPTXPD          0x18
#define FTGMAC100_RXPD            0x1C
#define FTGMAC100_NPTXR_BADR      0x20
#define FTGMAC100_RXR_BADR        0x24
#define FTGMAC100_HPTXPD          0x28
#define FTGMAC100_HPTXR_BADR      0x2c
#define FTGMAC100_ITC             0x30
#define FTGMAC100_APTC            0x34
#define FTGMAC100_DBLAC           0x38
#define FTGMAC100_REVR            0x40
#define FTGMAC100_FEAR1           0x44
#define FTGMAC100_RBSR            0x4c
#define FTGMAC100_TPAFCR          0x48

#define FTGMAC100_MACCR           0x50
#define FTGMAC100_MACSR           0x54
#define FTGMAC100_PHYCR           0x60
#define FTGMAC100_PHYDATA         0x64
#define FTGMAC100_FCR             0x68

/*
 * Interrupt status register & interrupt enable register
 */
#define FTGMAC100_INT_RPKT_BUF    (1 << 0)
#define FTGMAC100_INT_RPKT_FIFO   (1 << 1)
#define FTGMAC100_INT_NO_RXBUF    (1 << 2)
#define FTGMAC100_INT_RPKT_LOST   (1 << 3)
#define FTGMAC100_INT_XPKT_ETH    (1 << 4)
#define FTGMAC100_INT_XPKT_FIFO   (1 << 5)
#define FTGMAC100_INT_NO_NPTXBUF  (1 << 6)
#define FTGMAC100_INT_XPKT_LOST   (1 << 7)
#define FTGMAC100_INT_AHB_ERR     (1 << 8)
#define FTGMAC100_INT_PHYSTS_CHG  (1 << 9)
#define FTGMAC100_INT_NO_HPTXBUF  (1 << 10)

/*
 * Automatic polling timer control register
 */
#define FTGMAC100_APTC_RXPOLL_CNT(x)        ((x) & 0xf)
#define FTGMAC100_APTC_RXPOLL_TIME_SEL      (1 << 4)
#define FTGMAC100_APTC_TXPOLL_CNT(x)        (((x) >> 8) & 0xf)
#define FTGMAC100_APTC_TXPOLL_TIME_SEL      (1 << 12)

/*
 * PHY control register
 */
#define FTGMAC100_PHYCR_MIIRD               (1 << 27)
#define FTGMAC100_PHYCR_MIIWR               (1 << 26)

#define FTGMAC100_PHYCR_DEV(x)              (((x) >> 21) & 0x1f)
#define FTGMAC100_PHYCR_REG(x)              (((x) >> 16) & 0x1f)

/*
 * PHY data register
 */
#define FTGMAC100_PHYDATA_MIIWDATA(x)       ((x) & 0xffff)
#define FTGMAC100_PHYDATA_MIIRDATA(x)       (((x) >> 16) & 0xffff)

/*
 * PHY control register - New MDC/MDIO interface
 */
#define FTGMAC100_PHYCR_NEW_DATA(x)     (((x) >> 16) & 0xffff)
#define FTGMAC100_PHYCR_NEW_FIRE        (1 << 15)
#define FTGMAC100_PHYCR_NEW_ST_22       (1 << 12)
#define FTGMAC100_PHYCR_NEW_OP(x)       (((x) >> 10) & 3)
#define   FTGMAC100_PHYCR_NEW_OP_WRITE    0x1
#define   FTGMAC100_PHYCR_NEW_OP_READ     0x2
#define FTGMAC100_PHYCR_NEW_DEV(x)      (((x) >> 5) & 0x1f)
#define FTGMAC100_PHYCR_NEW_REG(x)      ((x) & 0x1f)

/*
 * Feature Register
 */
#define FTGMAC100_REVR_NEW_MDIO_INTERFACE   (1 << 31)

/*
 * MAC control register
 */
#define FTGMAC100_MACCR_TXDMA_EN         (1 << 0)
#define FTGMAC100_MACCR_RXDMA_EN         (1 << 1)
#define FTGMAC100_MACCR_TXMAC_EN         (1 << 2)
#define FTGMAC100_MACCR_RXMAC_EN         (1 << 3)
#define FTGMAC100_MACCR_RM_VLAN          (1 << 4)
#define FTGMAC100_MACCR_HPTXR_EN         (1 << 5)
#define FTGMAC100_MACCR_LOOP_EN          (1 << 6)
#define FTGMAC100_MACCR_ENRX_IN_HALFTX   (1 << 7)
#define FTGMAC100_MACCR_FULLDUP          (1 << 8)
#define FTGMAC100_MACCR_GIGA_MODE        (1 << 9)
#define FTGMAC100_MACCR_CRC_APD          (1 << 10) /* not needed */
#define FTGMAC100_MACCR_RX_RUNT          (1 << 12)
#define FTGMAC100_MACCR_JUMBO_LF         (1 << 13)
#define FTGMAC100_MACCR_RX_ALL           (1 << 14)
#define FTGMAC100_MACCR_HT_MULTI_EN      (1 << 15)
#define FTGMAC100_MACCR_RX_MULTIPKT      (1 << 16)
#define FTGMAC100_MACCR_RX_BROADPKT      (1 << 17)
#define FTGMAC100_MACCR_DISCARD_CRCERR   (1 << 18)
#define FTGMAC100_MACCR_FAST_MODE        (1 << 19)
#define FTGMAC100_MACCR_SW_RST           (1 << 31)

/*
 * Transmit descriptor
 */
#define FTGMAC100_TXDES0_TXBUF_SIZE(x)   ((x) & 0x3fff)
#define FTGMAC100_TXDES0_EDOTR           (1 << 15)
#define FTGMAC100_TXDES0_CRC_ERR         (1 << 19)
#define FTGMAC100_TXDES0_LTS             (1 << 28)
#define FTGMAC100_TXDES0_FTS             (1 << 29)
#define FTGMAC100_TXDES0_EDOTR_ASPEED    (1 << 30)
#define FTGMAC100_TXDES0_TXDMA_OWN       (1 << 31)

#define FTGMAC100_TXDES1_VLANTAG_CI(x)   ((x) & 0xffff)
#define FTGMAC100_TXDES1_INS_VLANTAG     (1 << 16)
#define FTGMAC100_TXDES1_TCP_CHKSUM      (1 << 17)
#define FTGMAC100_TXDES1_UDP_CHKSUM      (1 << 18)
#define FTGMAC100_TXDES1_IP_CHKSUM       (1 << 19)
#define FTGMAC100_TXDES1_LLC             (1 << 22)
#define FTGMAC100_TXDES1_TX2FIC          (1 << 30)
#define FTGMAC100_TXDES1_TXIC            (1 << 31)

/*
 * Receive descriptor
 */
#define FTGMAC100_RXDES0_VDBC            0x3fff
#define FTGMAC100_RXDES0_EDORR           (1 << 15)
#define FTGMAC100_RXDES0_MULTICAST       (1 << 16)
#define FTGMAC100_RXDES0_BROADCAST       (1 << 17)
#define FTGMAC100_RXDES0_RX_ERR          (1 << 18)
#define FTGMAC100_RXDES0_CRC_ERR         (1 << 19)
#define FTGMAC100_RXDES0_FTL             (1 << 20)
#define FTGMAC100_RXDES0_RUNT            (1 << 21)
#define FTGMAC100_RXDES0_RX_ODD_NB       (1 << 22)
#define FTGMAC100_RXDES0_FIFO_FULL       (1 << 23)
#define FTGMAC100_RXDES0_PAUSE_OPCODE    (1 << 24)
#define FTGMAC100_RXDES0_PAUSE_FRAME     (1 << 25)
#define FTGMAC100_RXDES0_LRS             (1 << 28)
#define FTGMAC100_RXDES0_FRS             (1 << 29)
#define FTGMAC100_RXDES0_EDORR_ASPEED    (1 << 30)
#define FTGMAC100_RXDES0_RXPKT_RDY       (1 << 31)

#define FTGMAC100_RXDES1_VLANTAG_CI      0xffff
#define FTGMAC100_RXDES1_PROT_MASK       (0x3 << 20)
#define FTGMAC100_RXDES1_PROT_NONIP      (0x0 << 20)
#define FTGMAC100_RXDES1_PROT_IP         (0x1 << 20)
#define FTGMAC100_RXDES1_PROT_TCPIP      (0x2 << 20)
#define FTGMAC100_RXDES1_PROT_UDPIP      (0x3 << 20)
#define FTGMAC100_RXDES1_LLC             (1 << 22)
#define FTGMAC100_RXDES1_DF              (1 << 23)
#define FTGMAC100_RXDES1_VLANTAG_AVAIL   (1 << 24)
#define FTGMAC100_RXDES1_TCP_CHKSUM_ERR  (1 << 25)
#define FTGMAC100_RXDES1_UDP_CHKSUM_ERR  (1 << 26)
#define FTGMAC100_RXDES1_IP_CHKSUM_ERR   (1 << 27)

/*
 * Receive and transmit Buffer Descriptor
 */
typedef struct {
    uint64_t        des0;
    uint64_t        des1;
    uint64_t        des2;        /* not used by HW */
    uint64_t        des3;
} FTGMAC100Desc;

#define FTGMAC100_DESC_ALIGNMENT 16

/*
 * Specific RTL8211E MII Registers
 */
#define RTL8211E_MII_PHYCR        16 /* PHY Specific Control */
#define RTL8211E_MII_PHYSR        17 /* PHY Specific Status */
#define RTL8211E_MII_INER         18 /* Interrupt Enable */
#define RTL8211E_MII_INSR         19 /* Interrupt Status */
#define RTL8211E_MII_RXERC        24 /* Receive Error Counter */
#define RTL8211E_MII_LDPSR        27 /* Link Down Power Saving */
#define RTL8211E_MII_EPAGSR       30 /* Extension Page Select */
#define RTL8211E_MII_PAGSEL       31 /* Page Select */
/*
 * RTL8211E Interrupt Status
 */
#define PHY_INT_AUTONEG_ERROR       (1 << 15)
#define PHY_INT_PAGE_RECV           (1 << 12)
#define PHY_INT_AUTONEG_COMPLETE    (1 << 11)
#define PHY_INT_LINK_STATUS         (1 << 10)
#define PHY_INT_ERROR               (1 << 9)
#define PHY_INT_DOWN                (1 << 8)
#define PHY_INT_JABBER              (1 << 0)


/* Broadcom BCM54xx -- taken from linux sungem_phy */

#define MIIM_BCM54xx_AUXCNTL			0x18
#define MIIM_BCM54xx_AUXCNTL_ENCODE(val) (((val & 0x7) >> 12)|(val & 0x7))
#define MIIM_BCM54xx_AUXSTATUS		0x19
#define MIIM_BCM54xx_AUXSTATUS_LINKMODE_MASK	0x0700
#define MIIM_BCM54xx_AUXSTATUS_LINKMODE_SHIFT	8

#define MIIM_BCM54XX_SHD			0x1c
#define MIIM_BCM54XX_SHD_WRITE		0x8000
#define MIIM_BCM54XX_SHD_REG(x)		(((x) >> 10) & 0x1f)
#define MIIM_BCM54XX_SHD_DATA(x)		((x & 0x3ff) >> 0)
#define MIIM_BCM54XX_SHD_WR_DECODE(val, data)	\
	(MIIM_BCM54XX_SHD_WRITE | MIIM_BCM54XX_SHD_VAL(val) | \
	 MIIM_BCM54XX_SHD_DATA(data))

#define MIIM_BCM54XX_EXP_DATA		0x15	/* Expansion register data */
#define MIIM_BCM54XX_EXP_SEL		0x17	/* Expansion register select */
#define MIIM_BCM54XX_EXP_SEL_SSD	0x0e00	/* Secondary SerDes select */
#define MIIM_BCM54XX_EXP_SEL_ER		0x0f00	/* Expansion register select */

#define MIIM_BCM_AUXCNTL_SHDWSEL_MISC	0x0007
#define MIIM_BCM_AUXCNTL_ACTL_SMDSP_EN	0x0800

#define MIIM_BCM_CHANNEL_WIDTH    0x2000
#define ASPEED_MII_PHYCR_DATA(x)     (x & 0xffff)
#define ASPEED_MII_PHYCR_PHY(x)      (((x) >> 21) & 0x1f)
#define ASPEED_MII_PHYCR_REG(x)      (((x) >> 16) & 0x1f)
/*
 * Max frame size for the receiving buffer
 */
#define FTGMAC100_MAX_FRAME_SIZE    9220

/* Limits depending on the type of the frame
 *
 *   9216 for Jumbo frames (+ 4 for VLAN)
 *   1518 for other frames (+ 4 for VLAN)
 */
static int ftgmac100_max_frame_size(FTGMAC100State *s, uint16_t proto)
{
    int max = (s->maccr & FTGMAC100_MACCR_JUMBO_LF ? 9216 : 1518);

    return max + (proto == ETH_P_VLAN ? 4 : 0);
}

static void ftgmac100_update_irq(FTGMAC100State *s)
{
    qemu_set_irq(s->irq, s->isr & s->ier);
}

/*
 * The MII phy could raise a GPIO to the processor which in turn
 * could be handled as an interrpt by the OS.
 * For now we don't handle any GPIO/interrupt line, so the OS will
 * have to poll for the PHY status.
 */
static void phy_update_irq(FTGMAC100State *s)
{
    ftgmac100_update_irq(s);
}

static void phy_update_link(FTGMAC100State *s)
{
    /* Autonegotiation status mirrors link status.  */
    if (qemu_get_queue(s->nic)->link_down) {
        s->phy_status &= ~(MII_BMSR_LINK_ST | MII_BMSR_AN_COMP);
        s->phy_int |= PHY_INT_DOWN;
    } else {
        s->phy_status |= (MII_BMSR_LINK_ST | MII_BMSR_AN_COMP);
        s->phy_int |= PHY_INT_AUTONEG_COMPLETE;
    }
    phy_update_irq(s);
}

static void ftgmac100_set_link(NetClientState *nc)
{
    phy_update_link(FTGMAC100(qemu_get_nic_opaque(nc)));
}

static void phy_reset(FTGMAC100State *s)
{
    s->phy_status = (MII_BMSR_100TX_FD | MII_BMSR_100TX_HD | MII_BMSR_10T_FD |
                     MII_BMSR_10T_HD | MII_BMSR_EXTSTAT | MII_BMSR_MFPS |
                     MII_BMSR_AN_COMP | MII_BMSR_AUTONEG | MII_BMSR_LINK_ST |
                     MII_BMSR_EXTCAP);
    s->phy_control = (MII_BMCR_AUTOEN | MII_BMCR_FD | MII_BMCR_SPEED1000);
    s->phy_advertise = (/*MII_ANAR_PAUSE_ASYM | MII_ANAR_PAUSE |*/ MII_ANAR_TXFD |
                        MII_ANAR_TX | MII_ANAR_10FD | MII_ANAR_10 |
                        MII_ANAR_CSMACD);
    s->phy_ctrl1000 = (MII_CTRL1000_HALF | MII_CTRL1000_FULL);                        
    s->phy_int_mask = 0;
    s->phy_int = 0;
}

static uint16_t do_phy_read(FTGMAC100State *s, uint8_t reg, uint8_t phy_addr)
{
    uint16_t val;

    if (phy_addr != s->phy_addr) {
        return 0xffff;
    }
    switch (reg) {
    case MII_BMCR: /* Basic Control */
        val = s->phy_control;
        break;
    case MII_BMSR: /* Basic Status */
        val = s->phy_status;
        break;
    case MII_PHYID1: /* ID1 */
        val = 0x600d;
        break;
    case MII_PHYID2: /* ID2 */
        val = 0x84a2;
        break;
    case MII_ANAR: /* Auto-neg advertisement */
        val = s->phy_advertise;
        break;
    case MII_ANLPAR: /* Auto-neg Link Partner Ability */
        val = (MII_ANLPAR_ACK | MII_ANLPAR_PAUSE | MII_ANLPAR_TXFD |
               MII_ANLPAR_TX | MII_ANLPAR_10FD | MII_ANLPAR_10 |
               MII_ANLPAR_CSMACD);
        break;
    case MII_ANER: /* Auto-neg Expansion */
        val = MII_ANER_NWAY;
        break;
    case MII_CTRL1000: /* 1000BASE-T control  */
        val = s->phy_ctrl1000;
        break;
    case MII_STAT1000: /* 1000BASE-T status  */
        val = MII_STAT1000_FULL;
        break;
    case MII_EXTSTAT:
        val = 0x3000;
        break;
    case RTL8211E_MII_INSR:  /* Interrupt status.  */
        val = s->phy_int;
        s->phy_int = 0;
        phy_update_irq(s);
        break;
    case MII_NSR:  /* Interrupt enable */
        val = s->phy_int_mask;
        break;
    case MII_NCONFIG: 
        val = s->phy_nconfig;
        break; 
         
 //   case MII_SNRDR:  
    case MII_TEST:
         val = 0x871c;
         break;
              
    
    case MII_RERRCOUNTER:    
         val = 0x5d86; //??
         break;
	    
    case MII_RESV1:
        val = s->phy_resv1; //??
        break;	
    case MII_NWAYTEST:	        
    case MII_SREVISION:        
    case MII_LBREMR:        
    case MII_REC:              
    case MII_LBRERROR:	     
//      case MII_PHYADDR:	     
    case MII_RESV2:	   
    case MII_TPISTATUS:
        qemu_log_mask(LOG_UNIMP, "%s: reg %d not implemented\n",
                      __func__, reg);
        val = 0;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: Bad address at offset %d\n",
                      __func__, reg);
        val = 0;
        break;
    }

    return val;
}

#define MII_BMCR_MASK (MII_BMCR_LOOPBACK | MII_BMCR_SPEED100 |          \
                       MII_BMCR_SPEED | MII_BMCR_AUTOEN | MII_BMCR_PDOWN | \
                       MII_BMCR_FD | MII_BMCR_CTST)
#define MII_ANAR_MASK 0x2d7f

static void do_phy_write(FTGMAC100State *s, uint8_t reg, uint16_t val)
{ printf("reg 0x%x val 0x%x\n",reg,val);

    switch (reg) {
    case MII_BMCR:     /* Basic Control */
        if (val & MII_BMCR_RESET) {
            phy_reset(s);
        } else {
            s->phy_control = val & MII_BMCR_MASK;
            /* Complete autonegotiation immediately.  */
            if (val & MII_BMCR_AUTOEN) {
                s->phy_status |= MII_BMSR_AN_COMP;
            }
        }
        break;
    case MII_ANAR:     /* Auto-neg advertisement */
        s->phy_advertise = (val & MII_ANAR_MASK) | MII_ANAR_TX;
        break;
    case MII_CTRL1000:
        s->phy_ctrl1000 = val; //(MII_CTRL1000_HALF | MII_CTRL1000_FULL);
        break;
    case MII_NSR: /* Interrupt enable */
        s->phy_int_mask = val & 0xffff;
        phy_update_irq(s);
        break;
    case MII_NCONFIG: 
        printf("*******MMD_CTL_DEVAD %x\n", MIIM_BCM54XX_SHD_REG(val));

        switch (MIIM_BCM54XX_SHD_REG(val)) {
             case 0x01:
             case 0x02:
             case 0x03:
             case 0x04:
             case 0x06:
             case 0x07:
             case 0x08: 
             case 0x0b:
             case 0x0c:
             case 0x0d:
             case 0x0e:
             case 0x0f:                         
                 qemu_log_mask(LOG_UNIMP, "%s: reg %d not implemented\n",
                      __func__, reg);
                 break;
             case 0x05:
                  val |= 0x1f;
                 break;
             case 0x09:
             
                 break;
             case 0x0a:
                  val |= 0x01;
                 break;

        }
        s->phy_nconfig = (val & ~(1 << 15));  
        break;
    case MII_RESV1:	
        s->phy_resv1 = val; //??
        break;         
    case MII_LBREMR:        
    case MII_REC:           
    case MII_SNRDR:         
    case MII_NWAYTEST:	    
    case MII_RERRCOUNTER:    
    case MII_SREVISION:	    
    
    case MII_LBRERROR:	     
//      case MII_PHYADDR:	     
    case MII_RESV2:	   
    case MII_TPISTATUS:
        qemu_log_mask(LOG_UNIMP, "%s: reg %d not implemented\n",
                      __func__, reg);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: Bad address at offset %d\n",
                      __func__, reg);
        break;
    }
}
#if 0
static void do_phy_new_ctl(FTGMAC100State *s)
{
    uint8_t reg;
    uint16_t data;

 //   if (!(s->phycr & FTGMAC100_PHYCR_NEW_ST_22)) {
  //      qemu_log_mask(LOG_UNIMP, "%s: unsupported ST code\n", __func__);
  //      return;
  //  }

    /* Nothing to do */
    if (!(s->phycr & FTGMAC100_PHYCR_NEW_FIRE)) {
        return;
    }

    reg = FTGMAC100_PHYCR_NEW_REG(s->phycr);
    data = FTGMAC100_PHYCR_NEW_DATA(s->phycr);

    switch (FTGMAC100_PHYCR_NEW_OP(s->phycr)) {
    case FTGMAC100_PHYCR_NEW_OP_WRITE:
        do_phy_write(s, reg, data);
        break;
    case FTGMAC100_PHYCR_NEW_OP_READ:
        s->phydata = do_phy_read(s, reg) & 0xffff;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: invalid OP code %08x\n",
                      __func__, s->phycr);
    }

    s->phycr &= ~FTGMAC100_PHYCR_NEW_FIRE;
}

static void do_phy_ctl(FTGMAC100State *s)
{
    uint8_t reg = FTGMAC100_PHYCR_REG(s->phycr);
printf("phycr 0x%x\n",s->phycr);
    if (s->phycr & FTGMAC100_PHYCR_MIIWR) {
        do_phy_write(s, reg, s->phydata & 0xffff);
        s->phycr &= ~FTGMAC100_PHYCR_MIIWR;
    } else if (s->phycr & FTGMAC100_PHYCR_MIIRD) {
        s->phydata = do_phy_read(s, reg) << 16;
        s->phycr &= ~FTGMAC100_PHYCR_MIIRD;
    } else {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: no OP code %08x\n",
                      __func__, s->phycr);
    }
}
#endif
static int ftgmac100_read_bd(FTGMAC100Desc *bd, dma_addr_t addr)
{
    if (dma_memory_read(&address_space_memory, addr, bd, sizeof(*bd))) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: failed to read descriptor @ 0x%"
                      HWADDR_PRIx "\n", __func__, addr);
        return -1;
    }
    bd->des0 = le32_to_cpu(bd->des0);
    bd->des1 = le32_to_cpu(bd->des1);
    bd->des2 = le32_to_cpu(bd->des2);
    bd->des3 = le32_to_cpu(bd->des3);
    return 0;
}

static int ftgmac100_write_bd(FTGMAC100Desc *bd, dma_addr_t addr)
{
    FTGMAC100Desc lebd;

    lebd.des0 = cpu_to_le32(bd->des0);
    lebd.des1 = cpu_to_le32(bd->des1);
    lebd.des2 = cpu_to_le32(bd->des2);
    lebd.des3 = cpu_to_le32(bd->des3);
    if (dma_memory_write(&address_space_memory, addr, &lebd, sizeof(lebd))) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: failed to write descriptor @ 0x%"
                      HWADDR_PRIx "\n", __func__, addr);
        return -1;
    }
    return 0;
}

static void ftgmac100_do_tx(FTGMAC100State *s, uint32_t tx_ring,
                            uint32_t tx_descriptor)
{
    int frame_size = 0;
    uint8_t *ptr = s->frame;
    dma_addr_t addr;
    uint32_t txdma_cfg, txumac_cfg;  
    int prod_idx, cons_idx;  
    uint32_t flags = 0;
    uint32_t dma_len_stat;

    /* Check that both TX MAC and TX DMA are enabled. We don't
     * handle DMA-less direct FIFO operations (we don't emulate
     * the FIFO at all).
     *
     * A write to TXDMA_KICK while DMA isn't enabled can happen
     * when the driver is resetting the pointer.
     */
    txdma_cfg = s->tdma_ctl;
    txumac_cfg = s->maccr;
    if (!(txdma_cfg & DMA_EN) ||
        !(txumac_cfg & CMD_TX_EN)) {
 //       trace_sungem_tx_disabled();
        return;
    }
   
    prod_idx = s->tdma_prod_index & DMA_P_INDEX_MASK;
    cons_idx = s->tdma_cons_index & DMA_C_INDEX_MASK;
    addr = s->tdma_daddhi[prod_idx-1];
    addr = (addr << 32) | s->tdma_daddlo[prod_idx-1];      
    dma_len_stat = s->tdma_dls[prod_idx-1];
//   for (frame_size = 0; frame_size <256; frame_size++) printf("daddlo 0x%x\n",s->tdma_daddlo[frame_size]);
    while (cons_idx != prod_idx) {
//    while (prod_idx < cons_idx) {
        FTGMAC100Desc bd;
        int len;
printf("********in loop addr 0x%lx len_stat 0x%x prod_idx 0x%x\n",addr,dma_len_stat, prod_idx);
        if (ftgmac100_read_bd(&bd, addr) ||
            ((dma_len_stat & DMA_OWN) == 0)) {  //??use DMA_EOP??
            /* Run out of descriptors to transmit.  */
            s->isr |= FTGMAC100_INT_NO_NPTXBUF;
       //     break;
        }
printf("*********bddes0 0x%lx\n",bd.des0);
printf("*********bddes1 0x%lx\n",bd.des1);
printf("*********bddes2 0x%lx\n",bd.des2);
printf("*********bddes3 0x%lx\n",bd.des3);
        /* record transmit flags as they are valid only on the first
         * segment */
        if (bd.des0 & FTGMAC100_TXDES0_FTS) {
            flags = bd.des1;
        }
    /* If it's a start of frame, discard anything we had in the
     * buffer and start again. This should be an error condition
     * if we had something ... for now we ignore it
     */
//    if (dma_len_stat & DMA_SOP) {
  //      if (s->tx_first_ctl) {
 ///           trace_sungem_tx_unfinished();
   //     }
   //     s->frame_size = 0;
     //   s->tx_first_ctl = dma_len_stat;
  //  }
        len = (dma_len_stat >> DMA_BUFLENGTH_SHIFT) & DMA_BUFLENGTH_MASK;;
        if (frame_size + len > sizeof(s->frame)) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: frame too big : %d bytes\n",
                          __func__, len);
            s->isr |= FTGMAC100_INT_XPKT_LOST;
            len =  sizeof(s->frame) - frame_size;
        }

        if (dma_memory_read(&address_space_memory, bd.des2, ptr, len)) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: failed to read packet @ 0x%lx\n",
                          __func__, bd.des2);
            s->isr |= FTGMAC100_INT_NO_NPTXBUF;
            break;
        }

        /* Check for VLAN */
        if (bd.des0 & FTGMAC100_TXDES0_FTS &&
            bd.des1 & FTGMAC100_TXDES1_INS_VLANTAG &&
            be16_to_cpu(PKT_GET_ETH_HDR(ptr)->h_proto) != ETH_P_VLAN) {
            if (frame_size + len + 4 > sizeof(s->frame)) {
                qemu_log_mask(LOG_GUEST_ERROR, "%s: frame too big : %d bytes\n",
                              __func__, len);
                s->isr |= FTGMAC100_INT_XPKT_LOST;
                len =  sizeof(s->frame) - frame_size - 4;
            }
            memmove(ptr + 16, ptr + 12, len - 12);
            stw_be_p(ptr + 12, ETH_P_VLAN);
            stw_be_p(ptr + 14, bd.des1);
            len += 4;
        }

        ptr += len;
        frame_size += len;
        if (dma_len_stat & DMA_EOP) {
            if (dma_len_stat & DMA_TX_APPEND_CRC) {
                net_checksum_calculate(s->frame, frame_size);
            }
            /* Last buffer in frame.  */
            qemu_send_packet(qemu_get_queue(s->nic), s->frame, frame_size);
            ptr = s->frame;
            frame_size = 0;
            if (flags & FTGMAC100_TXDES1_TXIC) {
                s->isr |= FTGMAC100_INT_XPKT_ETH;
            }
        }

        if (flags & FTGMAC100_TXDES1_TX2FIC) {
            s->isr |= FTGMAC100_INT_XPKT_FIFO;
        }
        dma_len_stat &= ~DMA_OWN;

        /* Write back the modified descriptor.  */
        ftgmac100_write_bd(&bd, addr);
        /* Advance to the next descriptor.  */
        /* Next ! */
        cons_idx = (cons_idx + 1) & DMA_C_INDEX_MASK;
        s->tdma_cons_index = cons_idx;        
 ////       if (bd.des0 & s->txdes0_edotr) {
 //           addr = tx_ring;
  //      } else {
    //        addr += sizeof(FTGMAC100Desc);
    //    }
    }

    s->tx_descriptor = addr;

    ftgmac100_update_irq(s);
}

static int ftgmac100_can_receive(NetClientState *nc)
{
    FTGMAC100State *s = FTGMAC100(qemu_get_nic_opaque(nc));
    FTGMAC100Desc bd;

    if (!(s->maccr & CMD_RX_EN) && !(s->rdma_ctl & DMA_EN)){
       printf("************can not receive*********\n");
        return 0;
    }

    if (ftgmac100_read_bd(&bd, s->rx_descriptor)) {
        return 0;
    }
    return !(bd.des0 & FTGMAC100_RXDES0_RXPKT_RDY);
}

/*
 * This is purely informative. The HW can poll the RW (and RX) ring
 * buffers for available descriptors but we don't need to trigger a
 * timer for that in qemu.
 */
static uint32_t ftgmac100_rxpoll(FTGMAC100State *s)
{
    /* Polling times :
     *
     * Speed      TIME_SEL=0    TIME_SEL=1
     *
     *    10         51.2 ms      819.2 ms
     *   100         5.12 ms      81.92 ms
     *  1000        1.024 ms     16.384 ms
     */
    static const int div[] = { 20, 200, 1000 };

    uint32_t cnt = 1024 * FTGMAC100_APTC_RXPOLL_CNT(s->aptcr);
    uint32_t speed = (s->maccr & FTGMAC100_MACCR_FAST_MODE) ? 1 : 0;

    if (s->aptcr & FTGMAC100_APTC_RXPOLL_TIME_SEL) {
        cnt <<= 4;
    }

    if (s->maccr & FTGMAC100_MACCR_GIGA_MODE) {
        speed = 2;
    }

    return cnt / div[speed];
}

static void ftgmac100_reset(DeviceState *d)
{
    FTGMAC100State *s = FTGMAC100(d);
printf("******************************RESET******************************");
    /* Reset the FTGMAC100 */
    s->isr = 0;
    s->ier = 0;
    s->rx_enabled = 0;
    s->rx_ring = 0;
    s->rbsr = 0x640;
    s->rx_descriptor = 0;
    s->tx_ring = 0;
    s->tx_descriptor = 0;
    s->math[0] = 0;
    s->math[1] = 0;
    s->itc = 0;
    s->aptcr = 1;
    s->dblac = 0x00022f00;
    s->revr = 0;
    s->fear1 = 0;
    s->tpafcr = 0xf1;

    s->maccr = 0;
    s->phycr = 0x3000000;
    s->phydata = 0;
    s->fcr = 0x400;

    /* and the PHY */
    phy_reset(s);
}

static uint64_t ftgmac100_read(void *opaque, hwaddr addr, unsigned size)
{
    FTGMAC100State *s = FTGMAC100(opaque);
//printf("*********addr 0x%lx\n", addr );
    int index;
    uint32_t ret;
    switch (addr & 0xffff) {
    case SYS_REV_CTRL: 
        return 0x6000000;
    case SYS_PORT_CTRL:
        return s->phymode;
    case SYS_RBUF_FLUSH_CTRL: 
        return s->rbsr;
    case SYS_TBUF_FLUSH_CTRL: 
        return s->tbsr;
    case EXT_RGMII_OOB_CTRL:
        return s->rgmii_oob_ctl;      
    case RBUF_CTRL:
        return s->rbuf_ctl; 
    case RBUF_TBUF_SIZE_CTRL:
        return s->rbtb_sz_ctl;                     
    case MDIO_CMD:
    printf("*********mdio cmd addr 0x%lx\n", addr );
        return s->phydata; 
    case UMAC_CMD: /* MAC Device control */
        return s->maccr;
    case UMAC_TX_FLUSH:
        return s->tx_flush;                 
    case UMAC_MAC0:
        return ((uint32_t) s->conf.macaddr.a[5] << 24) |
            (s->conf.macaddr.a[4] << 16) | (s->conf.macaddr.a[3] << 8) |
            s->conf.macaddr.a[2];    
        
    case UMAC_MAC1:
        return (s->conf.macaddr.a[0] << 8)  | s->conf.macaddr.a[1];
    case UMAC_MAX_FRAME_LEN:
        return s->max_frm_len;
    case UMAC_MIB_CTRL:
        return s->mib_ctl; 
        
    case TDMA_REG_BASE + DMA_CTRL:        // 0x5044
        return s->tdma_ctl;
        
        
    case RDMA_REG_BASE + DMA_CTRL:        //0x3044
         return s->rdma_ctl;   
        
    case RDMA_REG_BASE + DMA_SCB_BURST_SIZE:
         return s->rdma_scb_burst_size;
     
    case RDMA_RING_REG_BASE + DMA_START_ADDR:
         return s->rdma_ring_dma_start_addr;
    case RDMA_READ_PTR:
         return s->rdma_rd_ptr;
    case RDMA_WRITE_PTR:
         return s->rdma_wr_ptr;
    case RDMA_RING_REG_BASE + DMA_END_ADDR:
         return s->rdma_ring_dma_end_addr;
    case RDMA_PROD_INDEX:
         return s->rdma_prod_index;
    case RDMA_CONS_INDEX:
         return s->rdma_cons_index = 0x000000ff;
    case RDMA_RING_REG_BASE + DMA_RING_BUF_SIZE:
         return s->rdma_ring_dma_ring_buf_size;
    case RDMA_XON_XOFF_THRESH:
         return s->rdma_xon_xoff_thresh;
    case RDMA_REG_BASE + DMA_RING_CFG:   
         return s->rdma_dma_ring_cfg;
    case TDMA_REG_BASE + DMA_SCB_BURST_SIZE:
         return s->tdma_dma_scb_burst_size;
    case TDMA_RING_REG_BASE + DMA_START_ADDR:
         return s->tdma_ring_dma_start_addr;
    case TDMA_READ_PTR:
         return s->tdma_rd_ptr;
    case TDMA_WRITE_PTR:
         return s->tdma_wr_ptr;
    case TDMA_RING_REG_BASE + DMA_END_ADDR:
         return s->tdma_ring_dma_end_addr;
    case TDMA_PROD_INDEX:
         return s->tdma_prod_index;
    case TDMA_CONS_INDEX:
         return s->tdma_cons_index;
    case TDMA_RING_REG_BASE + DMA_MBUF_DONE_THRESH:
         return s->tdma_ring_dma_mbuf_done_thresh;
    case TDMA_FLOW_PERIOD:
         return s->tdma_flow_period;             
    case TDMA_RING_REG_BASE + DMA_RING_BUF_SIZE:
         return s->tdma_ring_dma_ring_buf_size;
//    case TDMA_XON_XOFF_THRESH:
    case TDMA_REG_BASE + DMA_RING_CFG:   
         return s->tdma_dma_ring_cfg; 
    
        
    case GENET_RX_OFF ... GENET_RX_OFF+(RX_DESCS*DMA_DESC_SIZE):
         switch ((addr/4 & 0xffff) % 3) {
             case 0:
                  index = ((addr&0xfff) - DMA_DESC_ADDRESS_LO)/DMA_DESC_SIZE;

                  ret = s->rdma_daddlo[index];// DMA_DESC_ADDRESS_LO
                  break;
             case 1:
                  index = ((addr&0xfff) - DMA_DESC_ADDRESS_HI)/DMA_DESC_SIZE;
                  ret = s->rdma_daddhi[index];// DMA_DESC_ADDRESS_HI        
                  break; 
             case 2:
                  index = ((addr&0xfff) - DMA_DESC_LENGTH_STATUS)/DMA_DESC_SIZE;
               //    printf("status addr 0x%lx index %d\n",addr,index);
                 //   printf("status addr 0x%lx index %lx\n",addr,s->dma_dls[index]);
                  ret = s->rdma_dls[index];// DMA_DESC_LENGTH_STATUS
                  break;
            // default:

         }    
 
                  if ((addr & 0x3) == 0) ret >>= 16;
         else if ((addr & 0x3) == 1)  ret >>= 24;
         else if ((addr & 0x3) == 2)  ret >>= 0;
         else if ((addr & 0x3) == 3)  ret >>= 8;
        
         return ret;  
         
        
    case GENET_TX_OFF ... GENET_TX_OFF+(TX_DESCS*DMA_DESC_SIZE):
         switch ((addr/4 & 0xffff) % 3) {
             case 2:
                  index = ((addr&0xfff) - DMA_DESC_ADDRESS_LO)/DMA_DESC_SIZE;

                  ret = s->tdma_daddlo[index];// DMA_DESC_ADDRESS_LO
                  break;
             case 0:
                  index = ((addr&0xfff) - DMA_DESC_ADDRESS_HI)/DMA_DESC_SIZE;
                  ret = s->tdma_daddhi[index];// DMA_DESC_ADDRESS_HI        
                  break; 
             case 1:
                  index = ((addr&0xfff) - DMA_DESC_LENGTH_STATUS)/DMA_DESC_SIZE;
               //    printf("status addr 0x%lx index %d\n",addr,index);
                 //   printf("status addr 0x%lx index %lx\n",addr,s->dma_dls[index]);
                  ret = s->tdma_dls[index];// DMA_DESC_LENGTH_STATUS
                  break;
            // default:

         }    
 
                  if ((addr & 0x3) == 0) ret >>= 16;
         else if ((addr & 0x3) == 1)  ret >>= 24;
         else if ((addr & 0x3) == 2)  ret >>= 0;
         else if ((addr & 0x3) == 3)  ret >>= 8;
        
         return ret;                  
    case FTGMAC100_DBLAC:
        return s->dblac;
    case FTGMAC100_REVR:
        return s->revr;
    case FTGMAC100_FEAR1:
        return s->fear1;
    case FTGMAC100_TPAFCR:
        return s->tpafcr;
    case FTGMAC100_FCR:
        return s->fcr;



        /* We might want to support these one day */
    case FTGMAC100_HPTXPD: /* High Priority Transmit Poll Demand */
    case FTGMAC100_HPTXR_BADR: /* High Priority Transmit Ring Base Address */
    case FTGMAC100_MACSR: /* MAC Status Register (MACSR) */
        qemu_log_mask(LOG_UNIMP, "%s: read to unimplemented register 0x%"
                      HWADDR_PRIx "\n", __func__, addr);
        return 0;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: Bad address at offset 0x%"
                      HWADDR_PRIx "\n", __func__, addr);
        return 0;
    }
}

static void ftgmac100_write(void *opaque, hwaddr addr,
                          uint64_t value, unsigned size)
{
    FTGMAC100State *s = FTGMAC100(opaque);
printf("addr 0x%lx value 0x%lx\n",addr,value);
    int index;
    switch (addr & 0xffff) {

    case SYS_PORT_CTRL: /* Sys Port Control */
        s->phymode = value;
//        if (s->revr & FTGMAC100_REVR_NEW_MDIO_INTERFACE) {
  //          do_phy_new_ctl(s);
    //    } else {
      //      do_phy_ctl(s);
        //}
        break;
    case EXT_RGMII_OOB_CTRL:
        s->rgmii_oob_ctl = value;
        break;      
    case UMAC_MAC0: /* MAC */
        s->conf.macaddr.a[5] = value >> 24;
        s->conf.macaddr.a[4] = value >> 16;
        s->conf.macaddr.a[3] = value >> 8;
        s->conf.macaddr.a[2] = value;    

        break;
    case UMAC_MAC1: 
        s->conf.macaddr.a[0] = value >> 8;
        s->conf.macaddr.a[1] = value;
        break;
    case FTGMAC100_ITC: /* TODO: Interrupt Timer Control */
        s->itc = value;
        break;
    case FTGMAC100_RXR_BADR: /* Ring buffer address */
        if (!QEMU_IS_ALIGNED(value, FTGMAC100_DESC_ALIGNMENT)) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: Bad RX buffer alignment 0x%"
                          HWADDR_PRIx "\n", __func__, value);
            return;
        }

        s->rx_ring = value;
        s->rx_descriptor = s->rx_ring;
        break;

    case SYS_RBUF_FLUSH_CTRL: 
        s->rbsr = value;
        break;
    case SYS_TBUF_FLUSH_CTRL: 
        s->tbsr = value;
        break;
    case RBUF_CTRL:
        s->rbuf_ctl = value;
        break;
    case RBUF_TBUF_SIZE_CTRL:
        s->rbtb_sz_ctl = value;
        break;     
    case FTGMAC100_NPTXR_BADR: /* Transmit buffer address */
        if (!QEMU_IS_ALIGNED(value, FTGMAC100_DESC_ALIGNMENT)) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: Bad TX buffer alignment 0x%"
                          HWADDR_PRIx "\n", __func__, value);
            return;
        }
        s->tx_ring = value;
        s->tx_descriptor = s->tx_ring;
        break;

    case FTGMAC100_NPTXPD: /* Trigger transmit */
        if ((s->maccr & (FTGMAC100_MACCR_TXDMA_EN | FTGMAC100_MACCR_TXMAC_EN))
            == (FTGMAC100_MACCR_TXDMA_EN | FTGMAC100_MACCR_TXMAC_EN)) {
            /* TODO: high priority tx ring */
            ftgmac100_do_tx(s, s->tx_ring, s->tx_descriptor);
        }
        if (ftgmac100_can_receive(qemu_get_queue(s->nic))) {
            qemu_flush_queued_packets(qemu_get_queue(s->nic));
        }
        break;

    case FTGMAC100_RXPD: /* Receive Poll Demand Register */
        if (ftgmac100_can_receive(qemu_get_queue(s->nic))) {
            qemu_flush_queued_packets(qemu_get_queue(s->nic));
        }
        break;

    case FTGMAC100_APTC: /* Automatic polling */
        s->aptcr = value;

        if (FTGMAC100_APTC_RXPOLL_CNT(s->aptcr)) {
            ftgmac100_rxpoll(s);
        }

        if (FTGMAC100_APTC_TXPOLL_CNT(s->aptcr)) {
            qemu_log_mask(LOG_UNIMP, "%s: no transmit polling\n", __func__);
        }
        break;

    case UMAC_CMD: /* MAC Device control */
        s->maccr = value;
        if (value & (CMD_SW_RESET | CMD_LCL_LOOP_EN)) {
            ftgmac100_reset(DEVICE(s));
        }

        if (ftgmac100_can_receive(qemu_get_queue(s->nic))) {
            qemu_flush_queued_packets(qemu_get_queue(s->nic));
        }
        break;
    case MDIO_CMD:
        s->mdio_cmd = value;
        break;
        
    case UMAC_TX_FLUSH:
        s->tx_flush = value;
        break;
    case UMAC_MIB_CTRL:
        s->mib_ctl = value;
        break;
    case UMAC_MAX_FRAME_LEN:
        s->max_frm_len = value; 
        break;
        
    case TDMA_REG_BASE + DMA_CTRL:        
        s->tdma_ctl = value;
        break;
        
    case RDMA_REG_BASE + DMA_CTRL:         
        s->rdma_ctl = value;
        break;  
    case RDMA_REG_BASE + DMA_SCB_BURST_SIZE:
        s->rdma_scb_burst_size = value;
        break;            
    case RDMA_RING_REG_BASE + DMA_START_ADDR:
        s->rdma_ring_dma_start_addr = value;
	break;
    case RDMA_READ_PTR:
        s->rdma_rd_ptr = value;
	break;
    case RDMA_WRITE_PTR:
        s->rdma_wr_ptr = value;
	break;
    case RDMA_RING_REG_BASE + DMA_END_ADDR:
        s->rdma_ring_dma_end_addr = value;
	break;
    case RDMA_PROD_INDEX:

    
        s->rdma_prod_index = value;
	break;
    case RDMA_CONS_INDEX:
        s->rdma_cons_index = value;
	break;
    case RDMA_RING_REG_BASE + DMA_RING_BUF_SIZE:
        s->rdma_ring_dma_ring_buf_size = value;
	break;
    case RDMA_XON_XOFF_THRESH:
        s->rdma_xon_xoff_thresh = value;
	break;
    case RDMA_REG_BASE + DMA_RING_CFG:   
        s->rdma_dma_ring_cfg = value;
	break;
    case TDMA_REG_BASE + DMA_SCB_BURST_SIZE:
        s->tdma_dma_scb_burst_size = value;
	break;
    case TDMA_RING_REG_BASE + DMA_START_ADDR:
        s->tdma_ring_dma_start_addr = value;
	break;
    case TDMA_READ_PTR:
        s->tdma_rd_ptr = value;
	break;
    case TDMA_WRITE_PTR:
        s->tdma_wr_ptr = value;
	break;
    case TDMA_RING_REG_BASE + DMA_END_ADDR:
        s->tdma_ring_dma_end_addr = value;
	break;
    case TDMA_PROD_INDEX:
           s->tdma_prod_index = value; 
        if ((s->maccr & CMD_TX_EN) && (s->tdma_ctl & DMA_EN)){
            /* TODO: high priority tx ring */  printf("**********transmit********8\n");
            ftgmac100_do_tx(s, s->tx_ring, s->tx_descriptor);
        }
        if (ftgmac100_can_receive(qemu_get_queue(s->nic))) {
            qemu_flush_queued_packets(qemu_get_queue(s->nic));
        }


	break;
    case TDMA_CONS_INDEX:
        s->tdma_cons_index = value;
	break;
    case TDMA_RING_REG_BASE + DMA_MBUF_DONE_THRESH:
        s->tdma_ring_dma_mbuf_done_thresh = value;
        break;
    case TDMA_FLOW_PERIOD:
        s->tdma_flow_period = value;  	
        break;
    case TDMA_RING_REG_BASE + DMA_RING_BUF_SIZE:
        s->tdma_ring_dma_ring_buf_size = value;
	break;
//    case TDMA_XON_XOFF_THRESH:
    case TDMA_REG_BASE + DMA_RING_CFG:   
        s->tdma_dma_ring_cfg = value;
	break;        
        
     case GENET_RX_OFF ... GENET_RX_OFF+(RX_DESCS*DMA_DESC_SIZE):
        if (!QEMU_IS_ALIGNED(value, ARCH_DMA_MINALIGN)) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: Bad RX buffer alignment 0x%"
                          HWADDR_PRIx "\n", __func__, value);
            return;
        }     
         switch ((addr/4 & 0xffff) % 3) {
             case 0:

                  index = ((addr&0xfff) - DMA_DESC_ADDRESS_LO)/DMA_DESC_SIZE;
//  printf("loaddr 0x%lx index %d\n",addr,index);
                  s->rdma_daddlo[index] = value;// DMA_DESC_ADDRESS_LO
                  break;
             case 1:
                  index = ((addr&0xfff) - DMA_DESC_ADDRESS_HI)/DMA_DESC_SIZE;
//  printf("hiaddr 0x%lx index %d\n",addr,index);
                  s->rdma_daddhi[index] = value;// DMA_DESC_ADDRESS_HI     
                  break;      
             case 2:
                  index = ((addr&0xfff) - DMA_DESC_LENGTH_STATUS)/DMA_DESC_SIZE;
 // printf("status addr 0x%lx index %d\n",addr,index);
                  s->rdma_dls[index] = value;// DMA_DESC_LENGTH_STATUS
//                   printf("status addr 0x%lx dma_dls 0x%x\n",addr,s->rdma_dls[index]);
                  break;
         }        
      break;  
        
     case GENET_TX_OFF ... GENET_TX_OFF+(TX_DESCS*DMA_DESC_SIZE):
        if (!QEMU_IS_ALIGNED(value, ARCH_DMA_MINALIGN)) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: Bad TX buffer alignment 0x%"
                          HWADDR_PRIx "\n", __func__, value);
            return;
        }        
         switch ((addr/4 & 0xffff) % 3) {
             case 2:

                  index = ((addr&0xfff) - DMA_DESC_ADDRESS_LO)/DMA_DESC_SIZE;

                  s->tdma_daddlo[index] = value;// DMA_DESC_ADDRESS_LO
printf("loaddr 0x%lx index %d daddlo 0x%x\n",addr,index,s->tdma_daddlo[index]);
                  break;
             case 0:
                  index = ((addr&0xfff) - DMA_DESC_ADDRESS_HI)/DMA_DESC_SIZE;

                  s->tdma_daddhi[index] = value;// DMA_DESC_ADDRESS_HI 
printf("hiaddr 0x%lx index %d daddhi 0x%x\n",addr,index,s->tdma_daddhi[index]);
                      
                  break;      
             case 1:
                  index = ((addr&0xfff) - DMA_DESC_LENGTH_STATUS)/DMA_DESC_SIZE;

                  s->tdma_dls[index] = value;// DMA_DESC_LENGTH_STATUS
 printf("status addr 0x%lx index %d status 0x%x\n",addr,index,s->tdma_dls[index]);
                  break;
         }        
      break;          
        
        
               
    case FTGMAC100_DBLAC: /* DMA Burst Length and Arbitration Control */
        s->dblac = value;
        break;
    case FTGMAC100_REVR:  /* Feature Register */
        s->revr = value;
        break;
    case FTGMAC100_FEAR1: /* Feature Register 1 */
        s->fear1 = value;
        break;
    case FTGMAC100_TPAFCR: /* Transmit Priority Arbitration and FIFO Control */
        s->tpafcr = value;
        break;
    case FTGMAC100_FCR: /* Flow Control  */
        s->fcr  = value;
        break;

    case FTGMAC100_HPTXPD: /* High Priority Transmit Poll Demand */
    case FTGMAC100_HPTXR_BADR: /* High Priority Transmit Ring Base Address */
    case FTGMAC100_MACSR: /* MAC Status Register (MACSR) */
        qemu_log_mask(LOG_UNIMP, "%s: write to unimplemented register 0x%"
                      HWADDR_PRIx "\n", __func__, addr);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: Bad address at offset 0x%"
                      HWADDR_PRIx "\n", __func__, addr);
        break;
    }

    ftgmac100_update_irq(s);
}

static int ftgmac100_filter(FTGMAC100State *s, const uint8_t *buf, size_t len)
{
    unsigned mcast_idx;

    if (s->maccr & FTGMAC100_MACCR_RX_ALL) {
        return 1;
    }

    switch (get_eth_packet_type(PKT_GET_ETH_HDR(buf))) {
    case ETH_PKT_BCAST:
        if (!(s->maccr & FTGMAC100_MACCR_RX_BROADPKT)) {
            return 0;
        }
        break;
    case ETH_PKT_MCAST:
        if (!(s->maccr & FTGMAC100_MACCR_RX_MULTIPKT)) {
            if (!(s->maccr & FTGMAC100_MACCR_HT_MULTI_EN)) {
                return 0;
            }

            mcast_idx = net_crc32_le(buf, ETH_ALEN);
            mcast_idx = (~(mcast_idx >> 2)) & 0x3f;
            if (!(s->math[mcast_idx / 32] & (1 << (mcast_idx % 32)))) {
                return 0;
            }
        }
        break;
    case ETH_PKT_UCAST:
        if (memcmp(s->conf.macaddr.a, buf, 6)) {
            return 0;
        }
        break;
    }

    return 1;
}

static ssize_t ftgmac100_receive(NetClientState *nc, const uint8_t *buf,
                                 size_t len)
{
    FTGMAC100State *s = FTGMAC100(qemu_get_nic_opaque(nc));
    FTGMAC100Desc bd;
    uint32_t flags = 0;
//    uint32_t addr;
    uint32_t crc;
    uint32_t buf_addr;
    uint8_t *crc_ptr;
    uint32_t buf_len;
    size_t size = len;
    uint32_t first = FTGMAC100_RXDES0_FRS;
    uint16_t proto = be16_to_cpu(PKT_GET_ETH_HDR(buf)->h_proto);
    int max_frame_size = ftgmac100_max_frame_size(s, proto);
    dma_addr_t addr;
//    uint32_t rxdma_cfg, rxumac_cfg;  
    int prod_idx, cons_idx;  
    uint32_t dma_len_stat;    

 //   if ((s->maccr & CMD_RX_EN) && (s->rdma_ctl & DMA_EN)){
   //     return -1;
   // }
printf("*******receive size 0x%lx\n",size);
    /* TODO : Pad to minimum Ethernet frame length */
    /* handle small packets.  */
    if (size < 10) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: dropped frame of %zd bytes\n",
                      __func__, size);
        return size;
    }

    if (!ftgmac100_filter(s, buf, size)) {
       // return size;
    }

    /* 4 bytes for the CRC.  */
    size += 4;
    crc = cpu_to_be32(crc32(~0, buf, size));
    crc_ptr = (uint8_t *) &crc;

    /* Huge frames are truncated.  */
    if (size > max_frame_size) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: frame too big : %zd bytes\n",
                      __func__, size);
        size = max_frame_size;
        flags |= FTGMAC100_RXDES0_FTL;
    }

    switch (get_eth_packet_type(PKT_GET_ETH_HDR(buf))) {
    case ETH_PKT_BCAST:
        flags |= FTGMAC100_RXDES0_BROADCAST;
        break;
    case ETH_PKT_MCAST:
        flags |= FTGMAC100_RXDES0_MULTICAST;
        break;
    case ETH_PKT_UCAST:
        break;
    }

    prod_idx = s->rdma_prod_index & DMA_P_INDEX_MASK;
    cons_idx = s->rdma_cons_index & DMA_C_INDEX_MASK;
    
   /* Ring full ? Can't receive */

 //   if (prod_idx == ((cons_idx + 1) & DMA_C_INDEX_MASK )) {
        //trace_sungem_rx_ringfull();
   //     return 0;
  //  }    
    addr = s->rdma_daddhi[prod_idx];
    addr = (addr << 32) | s->rdma_daddlo[prod_idx];      
    dma_len_stat = s->rdma_dls[prod_idx];

    while (size > 0) {
   //     if (!ftgmac100_can_receive(nc)) {
   //         qemu_log_mask(LOG_GUEST_ERROR, "%s: Unexpected packet\n", __func__);
     //       return -1;
     //   }
printf("***************receive prod_idx 0x%x cons_idx 0x%x addr 0x%lx\n",prod_idx,cons_idx,addr);
 //       if (ftgmac100_read_bd(&bd, addr) ||
  //          (bd.des0 & FTGMAC100_RXDES0_RXPKT_RDY)) {
            /* No descriptors available.  Bail out.  */
    //        qemu_log_mask(LOG_GUEST_ERROR, "%s: Lost end of frame\n",
   //                       __func__);
      //      s->isr |= FTGMAC100_INT_NO_RXBUF;
      //      break;
      //  }
      ftgmac100_read_bd(&bd, addr);
   printf("*********bddes0 0x%lx\n",bd.des0);
printf("*********bddes1 0x%lx\n",bd.des1);
printf("*********bddes2 0x%lx\n",bd.des2);
printf("*********bddes3 0x%lx\n",bd.des3);   
        buf_len = (size <= s->rbsr) ? size : s->rbsr;
        dma_len_stat |= (buf_len >> DMA_BUFLENGTH_SHIFT) & DMA_BUFLENGTH_MASK;
        size -= buf_len;

        /* The last 4 bytes are the CRC.  */
        if (size < 4) {
            buf_len += size - 4;
        }
        buf_addr = bd.des3;
        if (first && proto == ETH_P_VLAN && buf_len >= 18) {
            bd.des1 = lduw_be_p(buf + 14) | FTGMAC100_RXDES1_VLANTAG_AVAIL;

            if (s->maccr & FTGMAC100_MACCR_RM_VLAN) {
                dma_memory_write(&address_space_memory, buf_addr, buf, 12);
                dma_memory_write(&address_space_memory, buf_addr + 12, buf + 16,
                                 buf_len - 16);
            } else {
                dma_memory_write(&address_space_memory, buf_addr, buf, buf_len);
            }
        } else {
            bd.des1 = 0;
            dma_memory_write(&address_space_memory, buf_addr, buf, buf_len);
        }
        buf += buf_len;
        if (size < 4) {
            dma_memory_write(&address_space_memory, buf_addr + buf_len,
                             crc_ptr, 4 - size);
            crc_ptr += 4 - size;
        }

        dma_len_stat |= first | FTGMAC100_RXDES0_RXPKT_RDY;
        first = 0;
        if (size == 0) {
            /* Last buffer in frame.  */
            dma_len_stat |= flags | FTGMAC100_RXDES0_LRS;
            s->isr |= FTGMAC100_INT_RPKT_BUF;
        } else {
            s->isr |= FTGMAC100_INT_RPKT_FIFO;
        }
        
        ftgmac100_write_bd(&bd, addr);
        prod_idx = (prod_idx + 1) & DMA_P_INDEX_MASK;
        s->rdma_prod_index = prod_idx;    


//        if (bd.des0 & s->rxdes0_edorr) {
  //          addr = s->rx_ring;
    //    } else {
    //        addr += sizeof(FTGMAC100Desc);
    //    }
    }
//    s->rx_descriptor = addr;

    ftgmac100_update_irq(s);
    return len;
}

static const MemoryRegionOps ftgmac100_ops = {
    .read = ftgmac100_read,
    .write = ftgmac100_write,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void ftgmac100_cleanup(NetClientState *nc)
{
    FTGMAC100State *s = FTGMAC100(qemu_get_nic_opaque(nc));

    s->nic = NULL;
}

static NetClientInfo net_ftgmac100_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NICState),
    .can_receive = ftgmac100_can_receive,
    .receive = ftgmac100_receive,
    .cleanup = ftgmac100_cleanup,
    .link_status_changed = ftgmac100_set_link,
};

static void ftgmac100_realize(DeviceState *dev, Error **errp)
{
    FTGMAC100State *s = FTGMAC100(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    if (s->aspeed) {
        s->txdes0_edotr = FTGMAC100_TXDES0_EDOTR_ASPEED;
        s->rxdes0_edorr = FTGMAC100_RXDES0_EDORR_ASPEED;
    } else {
        s->txdes0_edotr = FTGMAC100_TXDES0_EDOTR;
        s->rxdes0_edorr = FTGMAC100_RXDES0_EDORR;
    }

    memory_region_init_io(&s->iomem, OBJECT(dev), &ftgmac100_ops, s,
                          TYPE_FTGMAC100, 0x10000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    qemu_macaddr_default_if_unset(&s->conf.macaddr);

    s->nic = qemu_new_nic(&net_ftgmac100_info, &s->conf,
                          object_get_typename(OBJECT(dev)), DEVICE(dev)->id,
                          s);
    qemu_format_nic_info_str(qemu_get_queue(s->nic), s->conf.macaddr.a);
}

static const VMStateDescription vmstate_ftgmac100 = {
    .name = TYPE_FTGMAC100,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(irq_state, FTGMAC100State),
        VMSTATE_UINT32(isr, FTGMAC100State),
        VMSTATE_UINT32(ier, FTGMAC100State),
        VMSTATE_UINT32(rx_enabled, FTGMAC100State),
        VMSTATE_UINT32(rx_ring, FTGMAC100State),
        VMSTATE_UINT32(rbsr, FTGMAC100State),
        VMSTATE_UINT32(tx_ring, FTGMAC100State),
        VMSTATE_UINT32(rx_descriptor, FTGMAC100State),
        VMSTATE_UINT32(tx_descriptor, FTGMAC100State),
        VMSTATE_UINT32_ARRAY(math, FTGMAC100State, 2),
        VMSTATE_UINT32(itc, FTGMAC100State),
        VMSTATE_UINT32(aptcr, FTGMAC100State),
        VMSTATE_UINT32(dblac, FTGMAC100State),
        VMSTATE_UINT32(revr, FTGMAC100State),
        VMSTATE_UINT32(fear1, FTGMAC100State),
        VMSTATE_UINT32(tpafcr, FTGMAC100State),
        VMSTATE_UINT32(maccr, FTGMAC100State),
        VMSTATE_UINT32(phycr, FTGMAC100State),
        VMSTATE_UINT32(phydata, FTGMAC100State),
        VMSTATE_UINT32(fcr, FTGMAC100State),
        VMSTATE_UINT32(phy_status, FTGMAC100State),
        VMSTATE_UINT32(phy_control, FTGMAC100State),
        VMSTATE_UINT32(phy_advertise, FTGMAC100State),
        VMSTATE_UINT32(phy_int, FTGMAC100State),
        VMSTATE_UINT32(phy_int_mask, FTGMAC100State),
        VMSTATE_UINT32(txdes0_edotr, FTGMAC100State),
        VMSTATE_UINT32(rxdes0_edorr, FTGMAC100State),
        VMSTATE_END_OF_LIST()
    }
};

static Property ftgmac100_properties[] = {
    DEFINE_PROP_BOOL("aspeed", FTGMAC100State, aspeed, false),
    DEFINE_NIC_PROPERTIES(FTGMAC100State, conf),
    /* Phy address should be 1 for most raspberry pi machines.
     * Can be set by a machine override in peripherals.
     */
    DEFINE_PROP_UINT32("phy_addr", FTGMAC100State, phy_addr, 0),      
    DEFINE_PROP_END_OF_LIST(),
};

static void ftgmac100_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->vmsd = &vmstate_ftgmac100;
    dc->reset = ftgmac100_reset;
    device_class_set_props(dc, ftgmac100_properties);
    set_bit(DEVICE_CATEGORY_NETWORK, dc->categories);
    dc->realize = ftgmac100_realize;
    dc->desc = "Faraday FTGMAC100 Gigabit Ethernet emulation";
}

static const TypeInfo ftgmac100_info = {
    .name = TYPE_FTGMAC100,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(FTGMAC100State),
    .class_init = ftgmac100_class_init,
};

/*
 * AST2600 MII controller
 */
#define MDIO_START_BUSY	       BIT(29)
#define ASPEED_MII_PHYCR_ST_22       BIT(28)
#define ASPEED_MII_PHYCR_OP(x)       ((x) & (MDIO_WR | \
                                             MDIO_RD))
//#define MDIO_WR	    BIT(26)
//#define MDIO_RD    BIT(27)
#define ASPEED_MII_PHYCR_DATA(x)     (x & 0xffff)
#define ASPEED_MII_PHYCR_PHY(x)      (((x) >> 21) & 0x1f)
#define ASPEED_MII_PHYCR_REG(x)      (((x) >> 16) & 0x1f)

#define ASPEED_MII_PHYDATA_IDLE      BIT(16)
#if 1
static void aspeed_mii_transition(AspeedMiiState *s, bool fire)
{
    if (fire) {
        s->phycr |= MDIO_START_BUSY;
 //       s->phydata &= ~ASPEED_MII_PHYDATA_IDLE;
    } else {
        s->phycr &= ~MDIO_START_BUSY;
 //       s->phydata |= ASPEED_MII_PHYDATA_IDLE;
    }
}
#endif
static void aspeed_mii_do_phy_ctl(AspeedMiiState *s)
{
    uint8_t reg;
    uint16_t data;
    uint8_t phy_addr;
 //   if (!(s->phycr & ASPEED_MII_PHYCR_ST_22)) {
 //       aspeed_mii_transition(s, !MDIO_START_BUSY);
  //      qemu_log_mask(LOG_UNIMP, "%s: unsupported ST code\n", __func__);
  //      return;
  //  }

    /* Nothing to do */
    if (!(s->phycr & MDIO_START_BUSY)) {
        return;
    }
    phy_addr = ASPEED_MII_PHYCR_PHY(s->phycr);
    reg = ASPEED_MII_PHYCR_REG(s->phycr);
    data = ASPEED_MII_PHYCR_DATA(s->phycr);
//printf("reg 0x%x data 0x%x phycr 0x%x\n",reg,data,s->phycr);
    switch (ASPEED_MII_PHYCR_OP(s->phycr)) {
    case MDIO_WR:
    printf("******write\n");
        do_phy_write(s->nic, reg, data);
        break;
    case MDIO_RD:
//        printf("******read\n");
//        s->phydata = (s->phydata & ~0xffff) | do_phy_read(s->nic, reg);
        s->phydata = do_phy_read(s->nic, reg, phy_addr) & 0xffff;
        if (s->phydata == 0xffff) s->phydata |= MDIO_READ_FAIL;
        s->phycr |= s->phydata;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: invalid OP code %08x\n",
                      __func__, s->phycr);
    }


}

static uint64_t aspeed_mii_read(void *opaque, hwaddr addr, unsigned size)
{
    AspeedMiiState *s = ASPEED_MII(opaque);
//printf("mii read addr 0x%lx phycr 0x%x\n",addr,s->phycr);
    aspeed_mii_transition(s, !MDIO_START_BUSY);
    switch (addr) {
    case 0x0:
        return s->phycr;
    case 0x4:
        return s->mdio_clk;
    default:         
        g_assert_not_reached();

    }
  //  aspeed_mii_transition(s, !MDIO_START_BUSY);
}

static void aspeed_mii_write(void *opaque, hwaddr addr,
                             uint64_t value, unsigned size)
{
    AspeedMiiState *s = ASPEED_MII(opaque);
//printf("mii write addr 0x%lx value 0x%lx\n",addr,value);
    switch (addr) {
    case 0x0:
        s->phycr = value;// & ~(s->phycr & MDIO_START_BUSY);
        break;
    case 0x4:
        s->mdio_clk = (value | 0x3f0) >> 4;
        break;
    default:
        g_assert_not_reached();
    }

 //   aspeed_mii_transition(s, !!(s->phycr & MDIO_START_BUSY));
    aspeed_mii_do_phy_ctl(s);     
}

static const MemoryRegionOps aspeed_mii_ops = {
    .read = aspeed_mii_read,
    .write = aspeed_mii_write,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void aspeed_mii_reset(DeviceState *dev)
{
    AspeedMiiState *s = ASPEED_MII(dev);

    s->phycr = 0;
    s->phydata = 0;

   // aspeed_mii_transition(s, !!(s->phycr & MDIO_START_BUSY));
};

static void aspeed_mii_realize(DeviceState *dev, Error **errp)
{
    AspeedMiiState *s = ASPEED_MII(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    assert(s->nic);

    memory_region_init_io(&s->iomem, OBJECT(dev), &aspeed_mii_ops, s,
                          TYPE_ASPEED_MII, 0x9);
    sysbus_init_mmio(sbd, &s->iomem);
}

static const VMStateDescription vmstate_aspeed_mii = {
    .name = TYPE_ASPEED_MII,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(phycr, FTGMAC100State),
        VMSTATE_UINT32(phydata, FTGMAC100State),
        VMSTATE_END_OF_LIST()
    }
};

static Property aspeed_mii_properties[] = {
    DEFINE_PROP_LINK("nic", AspeedMiiState, nic, TYPE_FTGMAC100,
                     FTGMAC100State *),
                   
    DEFINE_PROP_END_OF_LIST(),
};


static void aspeed_mii_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->vmsd = &vmstate_aspeed_mii;
    dc->reset = aspeed_mii_reset;
    dc->realize = aspeed_mii_realize;
    dc->desc = "Unimac MII controller";
    device_class_set_props(dc, aspeed_mii_properties);
}

static const TypeInfo aspeed_mii_info = {
    .name = TYPE_ASPEED_MII,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AspeedMiiState),
    .class_init = aspeed_mii_class_init,
};

static void ftgmac100_register_types(void)
{
    type_register_static(&ftgmac100_info);
    type_register_static(&aspeed_mii_info);
}

type_init(ftgmac100_register_types)
