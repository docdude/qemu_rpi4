#ifndef BCMGENET_H
#define BCMGENET_H
//#include "hw/net/unimac_mdio.h"

#include "hw/sysbus.h"
#include "net/net.h"

#define TYPE_BCMGENET "bcmgenet"

#define BCMGENET(obj) OBJECT_CHECK(BCMGENETState, (obj), TYPE_BCMGENET)

#define GENET_MMIO_SIZE        0x100000

#define BCMGENET_MMIO_SYS_SIZE 0x80
#define BCMGENET_MMIO_EXT_SIZE 0x180
#define BCMGENET_MMIO_INTRL2_0_SIZE 0x40
#define BCMGENET_MMIO_INTRL2_1_SIZE 0xC0
#define BCMGENET_MMIO_RBUF_SIZE 0x300
#define BCMGENET_MMIO_TBUF_SIZE 0x200
#define BCMGENET_MMIO_UMAC_SIZE 0x1800
#define BCMGENET_MMIO_RDMA_SIZE 0x2000
#define BCMGENET_MMIO_TDMA_SIZE 0x8000
#define BCMGENET_MMIO_HFB_SIZE 0x7d00
/*
 * Max frame size for the receiving buffer
 */
#define BCMGENET_MAX_FRAME_SIZE    1536
#define ARCH_DMA_MINALIGN 64



/* Register definitions derived from Linux source */


#define GENET_SYS_OFF			0x0000UL
#define SYS_REV_CTRL			0x0000UL
#define SYS_PORT_CTRL			0x0004UL
#define  PORT_MODE_INT_EPHY		0
#define  PORT_MODE_INT_GPHY		1
#define  PORT_MODE_EXT_EPHY		2
#define  PORT_MODE_EXT_GPHY		3
#define  PORT_MODE_EXT_RVMII_25	(4 | BIT(4))
#define  PORT_MODE_EXT_RVMII_50	4
#define  LED_ACT_SOURCE_MAC		BIT(9)
#define SYS_RBUF_FLUSH_CTRL		0x0008UL
#define SYS_TBUF_FLUSH_CTRL		0x000cUL

#define GENET_EXT_OFF			0x0080UL
/* Ext block register offsets and definitions */
#define EXT_EXT_PWR_MGMT		0x0000UL
#define  EXT_PWR_DOWN_BIAS		BIT(0)
#define  EXT_PWR_DOWN_DLL		BIT(1)
#define  EXT_PWR_DOWN_PHY		BIT(2)
#define  EXT_PWR_DN_EN_LD		BIT(3)
#define  EXT_ENERGY_DET		BIT(4)
#define  EXT_IDDQ_FROM_PHY		BIT(5)
#define  EXT_IDDQ_GLBL_PWR		BIT(7)
#define  EXT_PHY_RESET			BIT(8)
#define  EXT_ENERGY_DET_MASK		BIT(12)
#define  EXT_PWR_DOWN_PHY_TX		BIT(16)
#define  EXT_PWR_DOWN_PHY_RX		BIT(17)
#define  EXT_PWR_DOWN_PHY_SD		BIT(18)
#define  EXT_PWR_DOWN_PHY_RD		BIT(19)
#define  EXT_PWR_DOWN_PHY_EN		BIT(20)
#define EXT_RGMII_OOB_CTRL		0x000cUL
#define RGMII_LINK			BIT(4)
#define OOB_DISABLE			BIT(5)
#define RGMII_MODE_EN			BIT(6)
#define ID_MODE_DIS			BIT(16)
#define EXT_GPHY_CTRL			0x001CUL
#define  EXT_CFG_IDDQ_BIAS		BIT(0)
#define  EXT_CFG_PWR_DOWN		BIT(1)
#define  EXT_CK25_DIS			BIT(4)
#define  EXT_GPHY_RESET		BIT(5)

#define GENET_INTRL2_0_OFF		0x0200UL
#define INTRL2_0_INST                  0x0000UL
#define UMAC_IRQ_SCB			BIT(0)
#define UMAC_IRQ_EPHY			BIT(1)
#define UMAC_IRQ_PHY_DET_R		BIT(2)
#define UMAC_IRQ_PHY_DET_F		BIT(3)
#define UMAC_IRQ_LINK_UP		BIT(4)
#define UMAC_IRQ_LINK_DOWN		BIT(5)
#define UMAC_IRQ_LINK_EVENT		(UMAC_IRQ_LINK_UP | UMAC_IRQ_LINK_DOWN)
#define UMAC_IRQ_UMAC			BIT(6)
#define UMAC_IRQ_UMAC_TSV		BIT(7)
#define UMAC_IRQ_TBUF_UNDERRUN	BIT(8)
#define UMAC_IRQ_RBUF_OVERFLOW	BIT(9)
#define UMAC_IRQ_HFB_SM		BIT(10)
#define UMAC_IRQ_HFB_MM		BIT(11)
#define UMAC_IRQ_MPD_R			BIT(12)
#define UMAC_IRQ_RXDMA_MBDONE		BIT(13)
#define UMAC_IRQ_RXDMA_PDONE		BIT(14)
#define UMAC_IRQ_RXDMA_BDONE		BIT(15)
#define UMAC_IRQ_RXDMA_DONE		UMAC_IRQ_RXDMA_MBDONE
#define UMAC_IRQ_TXDMA_MBDONE		BIT(16)
#define UMAC_IRQ_TXDMA_PDONE		BIT(17)
#define UMAC_IRQ_TXDMA_BDONE		BIT(18)
#define UMAC_IRQ_TXDMA_DONE		UMAC_IRQ_TXDMA_MBDONE


#define GENET_INTRL2_1_OFF		0x0240UL
#define INTRL2_CPU_STAT		0x0000UL
#define INTRL2_CPU_SET			0x0004UL
#define INTRL2_CPU_CLEAR		0x0008UL
#define INTRL2_CPU_MASK_STATUS	0x000CUL
#define INTRL2_CPU_MASK_SET		0x0010UL
#define INTRL2_CPU_MASK_CLEAR		0x0014UL

/* Only valid for GENETv3+ */
#define UMAC_IRQ_MDIO_DONE		(1 << 23)
#define UMAC_IRQ_MDIO_ERROR		(1 << 24)
/* INTRL2 instance 1 definitions */
#define UMAC_IRQ1_TX_INTR_MASK		0xFFFF
#define UMAC_IRQ1_RX_INTR_MASK		0xFFFF
#define UMAC_IRQ1_RX_INTR_SHIFT		16


#define GENET_RBUF_OFF			0x0300UL

#define RBUF_CTRL			0x00
#define  RBUF_64B_EN			(1 << 0)
#define  RBUF_ALIGN_2B			(1 << 1)
#define  RBUF_BAD_DIS			(1 << 2)
#define RBUF_STATUS			0x0C
#define  RBUF_STATUS_WOL		(1 << 0)
#define  RBUF_STATUS_MPD_INTR_ACTIVE	(1 << 1)
#define  RBUF_STATUS_ACPI_INTR_ACTIVE	(1 << 2)
#define RBUF_CHK_CTRL			0x14
#define  RBUF_RXCHK_EN			(1 << 0)
#define  RBUF_SKIP_FCS			(1 << 4)
#define RBUF_ENERGY_CTRL		0x9c
#define  RBUF_EEE_EN			(1 << 0)
#define  RBUF_PM_EN			(1 << 1)
#define RBUF_TBUF_SIZE_CTRL		0xb4

#define GENET_TBUF_OFF			0x0600UL
#define TBUF_CTRL			0x0000UL
#define TBUF_BP_MC			0x000CUL
#define TBUF_ENERGY_CTRL		0x0014UL
#define TBUF_EEE_EN			BIT(0)
#define TBUF_PM_EN			BIT(1)
#define TBUF_CTRL_V1			0x0080UL
#define TBUF_BP_MC_V1			0x00A0UL

#define GENET_UMAC_OFF			0x0800UL

#define UMAC_CMD			0x0008UL
#define UMAC_MAC0			0x000cUL
#define UMAC_MAC1			0x0010UL
#define UMAC_MAX_FRAME_LEN		0x0014UL
#define UMACEEECTRL			0x0064UL
#define UMACEEELPITIMER 		0x0068UL
#define UMACEEEWAKETIMER		0x006cUL
#define UMACEEEREFCOUNT		0x0070UL
#define UMAC_TX_FLUSH			0x0334UL
#define UMAC_MIB_CTRL			0x0580UL
#define UMAC_MDIO_CMD			0x0614UL		
#define UMACMPDCTRL			0x0620UL 
#define UMACMDFCTRL			0x0650UL
#define UMACMDFADDR0			0x0654UL
#if 0
#define MDIO_START_BUSY		BIT(29)
#define MDIO_READ_FAIL			BIT(28)
#define MDIO_RD			BIT(27)
#define MDIO_WR			BIT(26)
#define MDIO_PMD_SHIFT			21
#define MDIO_PMD_MASK			0x1f
#define MDIO_REG_SHIFT			16
#define MDIO_REG_MASK			0x1f
#endif

#define UMAC_SPEED_10			0
#define UMAC_SPEED_100			1
#define UMAC_SPEED_1000		2
#define UMAC_SPEED_2500		3
#define CMD_SPEED_SHIFT		2
#define CMD_SPEED_MASK			3
#define CMD_TX_EN			BIT(0)
#define CMD_RX_EN			BIT(1)
#define CMD_PROMISC			BIT(4)
#define CMD_PAD_EN			BIT(5)
#define CMD_CRC_FWD			BIT(6)
#define CMD_PAUSE_FWD			BIT(7)
#define CMD_RX_PAUSE_IGNORE		BIT(8)
#define CMD_TX_ADDR_INS		BIT(9)
#define CMD_HD_EN			BIT(10)
#define CMD_SW_RESET			BIT(13)
#define CMD_LCL_LOOP_EN		BIT(15)
#define CMD_AUTO_CONFIG		BIT(22)
#define CMD_CNTL_FRM_EN		BIT(23)
#define CMD_NO_LEN_CHK			BIT(24)
#define CMD_RMT_LOOP_EN		BIT(25)
#define CMD_PRBL_EN			BIT(27)
#define CMD_TX_PAUSE_IGNORE		BIT(28)
#define CMD_TX_RX_EN			BIT(29)
#define CMD_RUNT_FILTER_DIS		BIT(30)

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
#define DMA_OWN			BIT(15) //0x8000
#define DMA_EOP			BIT(14) //0x4000
#define DMA_SOP			BIT(13) //0x2000
#define DMA_WRAP			BIT(12) //0x1000
#define DMA_MAX_BURST_LENGTH		BIT(3)  //0x8
/* Tx specific DMA descriptor bits */
#define DMA_TX_UNDERRUN		BIT(9) //0x0200
#define DMA_TX_APPEND_CRC		BIT(6) //0x0040
#define DMA_TX_OW_CRC			BIT(5) //0x0020
#define DMA_TX_DO_CSUM			BIT(4) //0x0010
#define DMA_TX_QTAG_SHIFT		7

#define DMA_RX_CHK_V3PLUS		BiT(15)
#define DMA_RX_CHK_V12			BIT(12)
#define DMA_RX_BRDCAST			BIT(6)
#define DMA_RX_MULT			BIT(5)
#define DMA_RX_LG			BIT(4)
#define DMA_RX_NO			BiT(3)
#define DMA_RX_RXER			BIT(2)
#define DMA_RX_CRC_ERROR		BIT(1)
#define DMA_RX_OV			BIT(0)

#define DMA_RX_FI_MASK			0x001F
#define DMA_RX_FI_SHIFT		0x0007
#define DMA_DESC_ALLOC_MASK		0x00FF
#define DMA_ARBITER_RR			0x00
#define DMA_ARBITER_WRR		0x01
#define DMA_ARBITER_SP			0x02

/* DMA rings size */
#define DMA_RING_SIZE			0x40
#define DMA_RINGS_SIZE			(DMA_RING_SIZE * (DEFAULT_Q + 1))

/* DMA descriptor */
#define DMA_DESC_LENGTH_STATUS	0x00
#define DMA_DESC_ADDRESS_LO		0x04
#define DMA_DESC_ADDRESS_HI		0x08
#define DMA_DESC_SIZE			12




#define DMA_FC_THRESH_HI		(RX_DESCS >> 4)
#define DMA_FC_THRESH_LO		5
#define DMA_FC_THRESH_VALUE		((DMA_FC_THRESH_LO << 16) |	\
					  DMA_FC_THRESH_HI)

#define DMA_XOFF_THRESHOLD_SHIFT	16

#define GENET_RX_OFF			0x2000UL
#define GENET_RDMA_REG_OFF					\
	(TOTAL_DESCS * DMA_DESC_SIZE)
#define RDMA_RING_REG_BASE(x)					\
	(GENET_RDMA_REG_OFF + (x * DMA_RING_SIZE))

#define RDMA_WRITE_PTR_Q0		(RDMA_RING_REG_BASE(0) + 0x00)
#define RDMA_PROD_INDEX_Q0		(RDMA_RING_REG_BASE(0) + 0x08)
#define RDMA_CONS_INDEX_Q0		(RDMA_RING_REG_BASE(0) + 0x0c)
#define RDMA_RING_BUF_SIZE_Q0		(RDMA_RING_REG_BASE(0) + 0x10)
#define RDMA_START_ADDR_Q0		(RDMA_RING_REG_BASE(0) + 0x14)
#define RDMA_END_ADDR_Q0		(RDMA_RING_REG_BASE(0) + 0x1c)
#define RDMA_MBUF_DONE_THRESH_Q0	(RDMA_RING_REG_BASE(0) + 0x24)
#define RDMA_XON_XOFF_THRESH_Q0	(RDMA_RING_REG_BASE(0) + 0x28)
#define RDMA_READ_PTR_Q0		(RDMA_RING_REG_BASE(0) + 0x2c)

#define RDMA_WRITE_PTR_Q1		(RDMA_RING_REG_BASE(1) + 0x00)
#define RDMA_PROD_INDEX_Q1		(RDMA_RING_REG_BASE(1) + 0x08)
#define RDMA_CONS_INDEX_Q1		(RDMA_RING_REG_BASE(1) + 0x0c)
#define RDMA_RING_BUF_SIZE_Q1		(RDMA_RING_REG_BASE(1) + 0x10)
#define RDMA_START_ADDR_Q1		(RDMA_RING_REG_BASE(1) + 0x14)
#define RDMA_END_ADDR_Q1		(RDMA_RING_REG_BASE(1) + 0x1c)
#define RDMA_MBUF_DONE_THRESH_Q1	(RDMA_RING_REG_BASE(1) + 0x24)
#define RDMA_XON_XOFF_THRESH_Q1	(RDMA_RING_REG_BASE(1) + 0x28)
#define RDMA_READ_PTR_Q1		(RDMA_RING_REG_BASE(1) + 0x2c)

#define RDMA_WRITE_PTR_Q2		(RDMA_RING_REG_BASE(2) + 0x00)
#define RDMA_PROD_INDEX_Q2		(RDMA_RING_REG_BASE(2) + 0x08)
#define RDMA_CONS_INDEX_Q2		(RDMA_RING_REG_BASE(2) + 0x0c)
#define RDMA_RING_BUF_SIZE_Q2		(RDMA_RING_REG_BASE(2) + 0x10)
#define RDMA_START_ADDR_Q2		(RDMA_RING_REG_BASE(2) + 0x14)
#define RDMA_END_ADDR_Q2		(RDMA_RING_REG_BASE(2) + 0x1c)
#define RDMA_MBUF_DONE_THRESH_Q2	(RDMA_RING_REG_BASE(2) + 0x24)
#define RDMA_XON_XOFF_THRESH_Q2	(RDMA_RING_REG_BASE(2) + 0x28)
#define RDMA_READ_PTR_Q2		(RDMA_RING_REG_BASE(2) + 0x2c)

#define RDMA_WRITE_PTR_Q3		(RDMA_RING_REG_BASE(3) + 0x00)
#define RDMA_PROD_INDEX_Q3		(RDMA_RING_REG_BASE(3) + 0x08)
#define RDMA_CONS_INDEX_Q3		(RDMA_RING_REG_BASE(3) + 0x0c)
#define RDMA_RING_BUF_SIZE_Q3		(RDMA_RING_REG_BASE(3) + 0x10)
#define RDMA_START_ADDR_Q3		(RDMA_RING_REG_BASE(3) + 0x14)
#define RDMA_END_ADDR_Q3		(RDMA_RING_REG_BASE(3) + 0x1c)
#define RDMA_MBUF_DONE_THRESH_Q3	(RDMA_RING_REG_BASE(3) + 0x24)
#define RDMA_XON_XOFF_THRESH_Q3	(RDMA_RING_REG_BASE(3) + 0x28)
#define RDMA_READ_PTR_Q3		(RDMA_RING_REG_BASE(3) + 0x2c)

#define RDMA_WRITE_PTR_Q16		(RDMA_RING_REG_BASE(16) + 0x00)
#define RDMA_PROD_INDEX_Q16		(RDMA_RING_REG_BASE(16) + 0x08)
#define RDMA_CONS_INDEX_Q16		(RDMA_RING_REG_BASE(16) + 0x0c)
#define RDMA_RING_BUF_SIZE_Q16	(RDMA_RING_REG_BASE(16) + 0x10)
#define RDMA_START_ADDR_Q16		(RDMA_RING_REG_BASE(16) + 0x14)
#define RDMA_END_ADDR_Q16		(RDMA_RING_REG_BASE(16) + 0x1c)
#define RDMA_MBUF_DONE_THRESH_Q16	(RDMA_RING_REG_BASE(16) + 0x24)
#define RDMA_XON_XOFF_THRESH_Q16	(RDMA_RING_REG_BASE(16) + 0x28)
#define RDMA_READ_PTR_Q16		(RDMA_RING_REG_BASE(16) + 0x2c)

#define RDMA_REG_BASE			(GENET_RDMA_REG_OFF + DMA_RINGS_SIZE)
#define RDMA_RING_CFG			(RDMA_REG_BASE + 0x00)
#define RDMA_CTRL			(RDMA_REG_BASE + 0x04)
#define RDMASTATUS	                (RDMA_REG_BASE + 0X08)
#define RDMA_SCB_BURST_SIZE		(RDMA_REG_BASE + 0x0c)
#define RDMAPRIORITY0			(RDMA_REG_BASE +0X30)
#define RDMAPRIORITY1			(RDMA_REG_BASE +0X34)
#define RDMAPRIORITY2			(RDMA_REG_BASE +0X38)

#define RDMA_RING0_TIMEOUT		(RDMA_REG_BASE + 0x2C)
#define RDMA_RING1_TIMEOUT		(RDMA_REG_BASE + 0x30)
#define RDMA_RING2_TIMEOUT		(RDMA_REG_BASE + 0x34)
#define RDMA_RING3_TIMEOUT		(RDMA_REG_BASE + 0x38)
#define RDMA_RING4_TIMEOUT		(RDMA_REG_BASE + 0x3c)
#define RDMA_RING5_TIMEOUT		(RDMA_REG_BASE + 0x40)
#define RDMA_RING6_TIMEOUT		(RDMA_REG_BASE + 0x44)
#define RDMA_RING7_TIMEOUT		(RDMA_REG_BASE + 0x48)
#define RDMA_RING8_TIMEOUT		(RDMA_REG_BASE + 0x4c)
#define RDMA_RING9_TIMEOUT		(RDMA_REG_BASE + 0x50)
#define RDMA_RING10_TIMEOUT		(RDMA_REG_BASE + 0x54)
#define RDMA_RING11_TIMEOUT		(RDMA_REG_BASE + 0x58)
#define RDMA_RING12_TIMEOUT		(RDMA_REG_BASE + 0x5c)
#define RDMA_RING13_TIMEOUT		(RDMA_REG_BASE + 0x60)
#define RDMA_RING14_TIMEOUT		(RDMA_REG_BASE + 0x64)
#define RDMA_RING15_TIMEOUT		(RDMA_REG_BASE + 0x68)
#define RDMA_RING16_TIMEOUT		(RDMA_REG_BASE + 0x6C)
#define RDMA_INDEX2RING_0		(RDMA_REG_BASE + 0x70)
#define RDMA_INDEX2RING_1		(RDMA_REG_BASE + 0x74)
#define RDMA_INDEX2RING_2		(RDMA_REG_BASE + 0x78)
#define RDMA_INDEX2RING_3		(RDMA_REG_BASE + 0x7C)
#define RDMA_INDEX2RING_4		(RDMA_REG_BASE + 0x80)
#define RDMA_INDEX2RING_5		(RDMA_REG_BASE + 0x84)
#define RDMA_INDEX2RING_6		(RDMA_REG_BASE + 0x88)
#define RDMA_INDEX2RING_7		(RDMA_REG_BASE + 0x8C)

#define GENET_TX_OFF			0x4000UL
#define GENET_TDMA_REG_OFF					\
	(TOTAL_DESCS * DMA_DESC_SIZE)
#define TDMA_RING_REG_BASE(x)					\
	( GENET_TDMA_REG_OFF + (x * DMA_RING_SIZE))
#if 1

#define TDMA_READ_PTR_Q0		(TDMA_RING_REG_BASE(0) + 0x00)
#define TDMA_CONS_INDEX_Q0		(TDMA_RING_REG_BASE(0) + 0x08)
#define TDMA_PROD_INDEX_Q0		(TDMA_RING_REG_BASE(0) + 0x0c)
#define TDMA_RING_BUF_SIZE_Q0		(TDMA_RING_REG_BASE(0) + 0x10)
#define TDMA_START_ADDR_Q0		(TDMA_RING_REG_BASE(0) + 0x14)
#define TDMA_END_ADDR_Q0		(TDMA_RING_REG_BASE(0) + 0x1c)
#define TDMA_MBUF_DONE_THRESH_Q0	(TDMA_RING_REG_BASE(0) + 0x24)
#define TDMA_FLOW_PERIOD_Q0		(TDMA_RING_REG_BASE(0) + 0x28)
#define TDMA_WRITE_PTR_Q0		(TDMA_RING_REG_BASE(0) + 0x2c)

#define TDMA_READ_PTR_Q1		(TDMA_RING_REG_BASE(1) + 0x00)
#define TDMA_CONS_INDEX_Q1		(TDMA_RING_REG_BASE(1) + 0x08)
#define TDMA_PROD_INDEX_Q1		(TDMA_RING_REG_BASE(1) + 0x0c)
#define TDMA_RING_BUF_SIZE_Q1		(TDMA_RING_REG_BASE(1) + 0x10)
#define TDMA_START_ADDR_Q1		(TDMA_RING_REG_BASE(1) + 0x14)
#define TDMA_END_ADDR_Q1		(TDMA_RING_REG_BASE(1) + 0x1c)
#define TDMA_MBUF_DONE_THRESH_Q1	(TDMA_RING_REG_BASE(1) + 0x24)
#define TDMA_FLOW_PERIOD_Q1		(TDMA_RING_REG_BASE(1) + 0x28)
#define TDMA_WRITE_PTR_Q1		(TDMA_RING_REG_BASE(1) + 0x2c)

#define TDMA_READ_PTR_Q2		(TDMA_RING_REG_BASE(2) + 0x00)
#define TDMA_CONS_INDEX_Q2		(TDMA_RING_REG_BASE(2) + 0x08)
#define TDMA_PROD_INDEX_Q2		(TDMA_RING_REG_BASE(2) + 0x0c)
#define TDMA_RING_BUF_SIZE_Q2		(TDMA_RING_REG_BASE(2) + 0x10)
#define TDMA_START_ADDR_Q2		(TDMA_RING_REG_BASE(2) + 0x14)
#define TDMA_END_ADDR_Q2		(TDMA_RING_REG_BASE(2) + 0x1c)
#define TDMA_MBUF_DONE_THRESH_Q2	(TDMA_RING_REG_BASE(2) + 0x24)
#define TDMA_FLOW_PERIOD_Q2		(TDMA_RING_REG_BASE(2) + 0x28)
#define TDMA_WRITE_PTR_Q2		(TDMA_RING_REG_BASE(2) + 0x2c)

#define TDMA_READ_PTR_Q3		(TDMA_RING_REG_BASE(3) + 0x00)
#define TDMA_CONS_INDEX_Q3		(TDMA_RING_REG_BASE(3) + 0x08)
#define TDMA_PROD_INDEX_Q3		(TDMA_RING_REG_BASE(3) + 0x0c)
#define TDMA_RING_BUF_SIZE_Q3		(TDMA_RING_REG_BASE(3) + 0x10)
#define TDMA_START_ADDR_Q3		(TDMA_RING_REG_BASE(3) + 0x14)
#define TDMA_END_ADDR_Q3		(TDMA_RING_REG_BASE(3) + 0x1c)
#define TDMA_MBUF_DONE_THRESH_Q3	(TDMA_RING_REG_BASE(3) + 0x24)
#define TDMA_FLOW_PERIOD_Q3		(TDMA_RING_REG_BASE(3) + 0x28)
#define TDMA_WRITE_PTR_Q3		(TDMA_RING_REG_BASE(3) + 0x2c)

#define TDMA_READ_PTR_Q16		(TDMA_RING_REG_BASE(16) + 0x00)
#define TDMA_CONS_INDEX_Q16		(TDMA_RING_REG_BASE(16) + 0x08)
#define TDMA_PROD_INDEX_Q16		(TDMA_RING_REG_BASE(16) + 0x0c)
#define TDMA_RING_BUF_SIZE_Q16	(TDMA_RING_REG_BASE(16) + 0x10)
#define TDMA_START_ADDR_Q16		(TDMA_RING_REG_BASE(16) + 0x14)
#define TDMA_END_ADDR_Q16		(TDMA_RING_REG_BASE(16) + 0x1c)
#define TDMA_MBUF_DONE_THRESH_Q16	(TDMA_RING_REG_BASE(16) + 0x24)
#define TDMA_FLOW_PERIOD_Q16		(TDMA_RING_REG_BASE(16) + 0x28)
#define TDMA_WRITE_PTR_Q16		(TDMA_RING_REG_BASE(16) + 0x2c)


#endif


#define TDMA_REG_BASE			(GENET_TDMA_REG_OFF + DMA_RINGS_SIZE)
#define TDMA_RING_CFG			(TDMA_REG_BASE + 0x00)
#define TDMA_CTRL			(TDMA_REG_BASE + 0x04)
#define TDMASTATUS	                (TDMA_REG_BASE + 0X08)
#define TDMA_SCB_BURST_SIZE		(TDMA_REG_BASE + 0x0c)
#define TDMAARBCTRL	 		(TDMA_REG_BASE +0X2C)
#define TDMAPRIORITY0			(TDMA_REG_BASE +0X30)
#define TDMAPRIORITY1			(TDMA_REG_BASE +0X34)
#define TDMAPRIORITY2			(TDMA_REG_BASE +0X38)

#define TDMA_RING0_TIMEOUT		(TDMA_REG_BASE + 0x2C)
#define TDMA_RING1_TIMEOUT		(TDMA_REG_BASE + 0x30)
#define TDMA_RING2_TIMEOUT		(TDMA_REG_BASE + 0x34)
#define TDMA_RING3_TIMEOUT		(TDMA_REG_BASE + 0x38)
#define TDMA_RING4_TIMEOUT		(TDMA_REG_BASE + 0x3c)
#define TDMA_RING5_TIMEOUT		(TDMA_REG_BASE + 0x40)
#define TDMA_RING6_TIMEOUT		(TDMA_REG_BASE + 0x44)
#define TDMA_RING7_TIMEOUT		(TDMA_REG_BASE + 0x48)
#define TDMA_RING8_TIMEOUT		(TDMA_REG_BASE + 0x4c)
#define TDMA_RING9_TIMEOUT		(TDMA_REG_BASE + 0x50)
#define TDMA_RING10_TIMEOUT		(TDMA_REG_BASE + 0x54)
#define TDMA_RING11_TIMEOUT		(TDMA_REG_BASE + 0x58)
#define TDMA_RING12_TIMEOUT		(TDMA_REG_BASE + 0x5c)
#define TDMA_RING13_TIMEOUT		(TDMA_REG_BASE + 0x60)
#define TDMA_RING14_TIMEOUT		(TDMA_REG_BASE + 0x64)
#define TDMA_RING15_TIMEOUT		(TDMA_REG_BASE + 0x68)
#define TDMA_RING16_TIMEOUT		(TDMA_REG_BASE + 0x6C)
#define TDMA_INDEX2RING_0		(TDMA_REG_BASE + 0x70)
#define TDMA_INDEX2RING_1		(TDMA_REG_BASE + 0x74)
#define TDMA_INDEX2RING_2		(TDMA_REG_BASE + 0x78)
#define TDMA_INDEX2RING_3		(TDMA_REG_BASE + 0x7C)
#define TDMA_INDEX2RING_4		(TDMA_REG_BASE + 0x80)
#define TDMA_INDEX2RING_5		(TDMA_REG_BASE + 0x84)
#define TDMA_INDEX2RING_6		(TDMA_REG_BASE + 0x88)
#define TDMA_INDEX2RING_7		(TDMA_REG_BASE + 0x8C)

#define DMA_P_INDEX_MASK		0xFFFF
#define DMA_C_INDEX_MASK		0xFFFF
#define RX_BUF_LENGTH			2048
#define RX_TOTAL_BUFSIZE		(RX_BUF_LENGTH * RX_DESCS)
#define RX_BUF_OFFSET			2
#define GENET_Q16_TX_BD_CNT  		TOTAL_DESCS - (4 * 32)
#define GENET_Q16_RX_BD_CNT  		TOTAL_DESCS
#define DEBUG 0

#define GENET_HFB_OFF			0x8000
#define HFBCTLR	0xFC00
#define HFBFLTENABLE	0xFC04
#define HFBFLTLEN	0xFC1C

typedef struct BCMGENETState {
    /*< private >*/
    SysBusDevice parent_obj;

    NICState *nic;
    NICConf conf;
    qemu_irq irq, intrl2_0, intrl2_1; 
    MemoryRegion bcmgenet;   
    struct {  
	MemoryRegion sys;
    	MemoryRegion ext;
    	MemoryRegion intrl2_0;  
    	MemoryRegion intrl2_1;            
    	MemoryRegion tdma;
    	MemoryRegion rdma;
    	MemoryRegion umac;
    	MemoryRegion rbuf;
    	MemoryRegion tbuf;
    	MemoryRegion hfb;    
    } iomem; 
    uint32_t isr;
    uint32_t ier;   
    uint32_t phy_addr;

    uint32_t sysregs[BCMGENET_MMIO_SYS_SIZE >> 2];
    uint32_t extregs[BCMGENET_MMIO_EXT_SIZE >> 2];
    uint32_t intrl2_0regs[BCMGENET_MMIO_INTRL2_0_SIZE >> 2];
    uint32_t intrl2_1regs[BCMGENET_MMIO_INTRL2_1_SIZE >> 2];            
    uint32_t tdmaregs[BCMGENET_MMIO_TDMA_SIZE >> 2];
    uint32_t rdmaregs[BCMGENET_MMIO_RDMA_SIZE >> 2];
    uint32_t umacregs[BCMGENET_MMIO_UMAC_SIZE >> 2];
    uint32_t rbufregs[BCMGENET_MMIO_RBUF_SIZE >> 2];
    uint32_t tbufregs[BCMGENET_MMIO_TBUF_SIZE >> 2];
    uint32_t hfbregs[BCMGENET_MMIO_HFB_SIZE >> 2];    

    /* Cache some useful things */
    uint32_t rx_mask;
    uint32_t last_desc_base;
    uint32_t ring_idx;

    /* Current tx packet */
//    uint8_t tx_data[MAX_PACKET_SIZE];
    uint8_t frame[BCMGENET_MAX_FRAME_SIZE];
    uint32_t tx_size;
    uint64_t tx_first_ctl;

    bool unimac_mdio;
    uint32_t phy_status;
    uint32_t phy_control;
    uint32_t phy_annp;
    uint32_t phy_anlprnp;
    uint32_t phy_ctrl1000;  
    uint32_t phy_extstat;  
    uint32_t phy_advertise;
    uint32_t phy_int;
    uint32_t phy_ecr;
    uint32_t phy_esr;    
    uint32_t phy_shdw;
    uint32_t phy_imr;
    uint32_t phy_isr;       
    uint32_t phy_aux_ctl_shdwsel[8];
    uint32_t phy_aux_ctl;
    uint32_t phy_exp_sel;
    uint32_t phy_exp_selregs;
    uint32_t phy_exp_data[20];

    uint32_t phy_shdwregs[16];     
} BCMGENETState; 





#define TYPE_UNIMAC_MDIO "unimac-mdio"
#define UNIMAC_MDIO(obj) OBJECT_CHECK(UNIMACState, (obj), TYPE_UNIMAC_MDIO)

typedef struct UNIMACState {
    /*< private >*/
    SysBusDevice parent_obj;

    BCMGENETState *nic;
    qemu_irq irq;
    MemoryRegion iomem;
    uint32_t phycr;
    uint32_t phydata;

    
    uint32_t mdio_clk;  
    uint32_t phy_addr;
    

} UNIMACState;

void phy_reset(BCMGENETState *);
void bcmgenet_update_status(BCMGENETState *s, uint32_t bits, bool val, int reg);

#endif
