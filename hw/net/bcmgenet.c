/*
 * QEMU model of GENET(BCM54123PE) ethernet controller
 *
 * 
 *
 * 
 * Copyright 2020 Juan Loya, MD
 */

#include "qemu/osdep.h"
#include "hw/qdev-properties.h"
#include "migration/vmstate.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "net/net.h"
#include "net/eth.h"
#include "hw/irq.h"
#include "net/checksum.h"
#include "hw/net/mii.h"
#include "sysemu/sysemu.h"
#include "sysemu/dma.h"
#include "trace.h"
#include "hw/net/bcmgenet.h"
//#include "hw/net/unimac_mdio.h"



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


#define GENET_RBUF_OFF			0x0300UL
#define RBUF_TBUF_SIZE_CTRL		0xb4
#define RBUF_CTRL			0x00
#define RBUF_ALIGN_2B			BIT(1)

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
#define MDIO_CMD			0x0614UL		
#define UMACMPDCTRL			0x0620UL 
#define UMACMDFCTRL			0x0650UL
#define UMACMDFADDR0			0x0654UL

#define MDIO_START_BUSY		BIT(29)
#define MDIO_READ_FAIL			BIT(28)
#define MDIO_RD			BIT(27)
#define MDIO_WR			BIT(26)
#define MDIO_PMD_SHIFT			21
#define MDIO_PMD_MASK			0x1f
#define MDIO_REG_SHIFT			16
#define MDIO_REG_MASK			0x1f


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
#define RDMA_RING_REG_BASE					\
	(GENET_RDMA_REG_OFF + DEFAULT_Q * DMA_RING_SIZE)
#define RDMA_WRITE_PTR			(RDMA_RING_REG_BASE + 0x00)
#define RDMA_PROD_INDEX		(RDMA_RING_REG_BASE + 0x08)
#define RDMA_CONS_INDEX		(RDMA_RING_REG_BASE + 0x0c)
#define RDMA_RING_BUF_SIZE		(RDMA_RING_REG_BASE + 0x10)
#define RDMA_START_ADDR		(RDMA_RING_REG_BASE + 0x14)
#define RDMA_END_ADDR			(RDMA_RING_REG_BASE + 0x1c)
#define RDMA_MBUF_DONE_THRESH		(RDMA_RING_REG_BASE + 0x24)
#define RDMA_XON_XOFF_THRESH		(RDMA_RING_REG_BASE + 0x28)
#define RDMA_READ_PTR			(RDMA_RING_REG_BASE + 0x2c)

#define RDMA_REG_BASE			(GENET_RDMA_REG_OFF + DMA_RINGS_SIZE)
#define RDMA_RING_CFG			(RDMA_REG_BASE + 0x00)
#define RDMA_CTRL			(RDMA_REG_BASE + 0x04)
#define RDMASTATUS	                (RDMA_REG_BASE + 0X08)
#define RDMA_SCB_BURST_SIZE		(RDMA_REG_BASE + 0x0c)
#define RDMATIMEOUT0			(RDMA_REG_BASE + 0X2C)
#define RDMAINDEX2RING0	   	(RDMA_REG_BASE + 0X70)

#define GENET_TX_OFF			0x4000UL
#define GENET_TDMA_REG_OFF					\
	(TOTAL_DESCS * DMA_DESC_SIZE)
#define TDMA_RING_REG_BASE					\
	(GENET_TDMA_REG_OFF + DEFAULT_Q * DMA_RING_SIZE)
#define TDMA_READ_PTR			(TDMA_RING_REG_BASE + 0x00)
#define TDMA_CONS_INDEX		(TDMA_RING_REG_BASE + 0x08)
#define TDMA_PROD_INDEX		(TDMA_RING_REG_BASE + 0x0c)
#define TDMA_RING_BUF_SIZE		(TDMA_RING_REG_BASE + 0x10)
#define TDMA_START_ADDR		(TDMA_RING_REG_BASE + 0x14)
#define TDMA_END_ADDR			(TDMA_RING_REG_BASE + 0x1c)
#define TDMA_MBUF_DONE_THRESH		(TDMA_RING_REG_BASE + 0x24)
#define TDMA_FLOW_PERIOD		(TDMA_RING_REG_BASE + 0x28)
#define TDMA_WRITE_PTR			(TDMA_RING_REG_BASE + 0x2c)

#define TDMA_REG_BASE			(GENET_TDMA_REG_OFF + DMA_RINGS_SIZE)
#define TDMA_RING_CFG			(TDMA_REG_BASE + 0x00)
#define TDMA_CTRL			(TDMA_REG_BASE + 0x04)
#define TDMASTATUS	                (TDMA_REG_BASE + 0X08)
#define TDMA_SCB_BURST_SIZE		(TDMA_REG_BASE + 0x0c)
#define TDMAARBCTRL	 		(TDMA_REG_BASE +0X2C)
#define TDMAPRIORITY0			(TDMA_REG_BASE +0X30)
#define TDMAPRIORITY1			(TDMA_REG_BASE +0X34)
#define TDMAPRIORITY2			(TDMA_REG_BASE +0X38)

#define DMA_P_INDEX_MASK		0xFFFF
#define DMA_C_INDEX_MASK		0xFFFF
#define RX_BUF_LENGTH			2048
#define RX_TOTAL_BUFSIZE		(RX_BUF_LENGTH * RX_DESCS)
#define RX_BUF_OFFSET			2

#define DEBUG 0

#define GENET_HFB_OFF			0x8000
#define HFBCTLR	0xFC00
#define HFBFLTENABLE	0xFC04
#define HFBFLTLEN	0xFC1C



/* ?????????????????????????
 * Max frame size for the receiving buffer
 */
//#define BCMGENET_MAX_FRAME_SIZE    9220




static void bcmgenet_update_irq(BCMGENETState *s)
{
    qemu_set_irq(s->irq157, 1);
    qemu_set_irq(s->irq158, 1);
}
#if 0
static void phy_update_link(BCMGENETState *s)
{
    /* Autonegotiation status mirrors link status.  */
    if (qemu_get_queue(s->nic)->link_down) {
        s->mdio.phy_status &= ~(MII_BMSR_LINK_ST | MII_BMSR_AN_COMP);
  //      s->mdio.phy_int |= PHY_INT_DOWN;
    } else {
        s->mdio.phy_status |= (MII_BMSR_LINK_ST | MII_BMSR_AN_COMP);
 //       s->mdio.phy_int |= PHY_INT_AUTONEG_COMPLETE;
    }
    phy_update_irq(s);
}

static void bmcgenet_set_link(NetClientState *nc)
{
    phy_update_link(BCMGENETState(qemu_get_nic_opaque(nc)));
}

#endif

#if 1
/*
 * Receive and transmit Buffer Descriptor
 */
typedef struct {
    uint64_t        des0;
    uint64_t        des1;
    uint64_t        des2;        /* not used by HW */
    uint64_t        des3;
    uint64_t        des4;
    uint64_t        des5;        
} BCMGENETDesc;
 
static int bcmgenet_read_bd(BCMGENETDesc *bd, dma_addr_t addr)
{
    if (dma_memory_read(&address_space_memory, addr, bd, sizeof(*bd))) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: failed to read descriptor @ 0x%"
                      HWADDR_PRIx "\n", __func__, addr);
        return -1;
    }
    bd->des0 = le64_to_cpu(bd->des0);
    bd->des1 = le64_to_cpu(bd->des1);
    bd->des2 = le64_to_cpu(bd->des2);
    bd->des3 = le64_to_cpu(bd->des3);
    bd->des4 = le64_to_cpu(bd->des4);
    bd->des5 = le64_to_cpu(bd->des5);    
    return 0;
}
#if 0
static int bcmgenet_write_bd(BCMGENETDesc *bd, dma_addr_t addr)
{
    BCMGENETDesc lebd;

    lebd.des0 = cpu_to_le64(bd->des0);
    lebd.des1 = cpu_to_le64(bd->des1);
    lebd.des2 = cpu_to_le64(bd->des2);
    lebd.des3 = cpu_to_le64(bd->des3);
    lebd.des4 = cpu_to_le64(bd->des4);
    lebd.des5 = cpu_to_le64(bd->des5);    
    if (dma_memory_write(&address_space_memory, addr, &lebd, sizeof(lebd))) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: failed to write descriptor @ 0x%"
                      HWADDR_PRIx "\n", __func__, addr);
        return -1;
    }
    return 0;
}
#endif
static void bcmgenet_do_tx(BCMGENETState *s)
{
    dma_addr_t addr;
    int frame_size = 0;
    uint8_t *ptr = s->frame;
    uint32_t tdma_cfg, umac_cfg;  
    uint32_t prod_idx, cons_idx;  
//    uint32_t flags = 0;
    uint32_t dma_len_stat;
    uint32_t desc_base;

   trace_bcmgenet_tx_do();

    /* Check that both TX MAC and TX DMA are enabled. We don't
     * handle DMA-less direct FIFO operations (we don't emulate
     * the FIFO at all).
     *
     * A write to TXDMA_KICK while DMA isn't enabled can happen
     * when the driver is resetting the pointer.
     */

    tdma_cfg = s->tdmaregs[TDMA_CTRL >> 2];

    umac_cfg = s->umacregs[UMAC_CMD >> 2];
printf("*****umaccfg 0x%x tdmacfg 0x%x\n",umac_cfg,tdma_cfg);
    if (!(tdma_cfg & DMA_EN) ||
        !(umac_cfg & CMD_TX_EN)) {
        trace_bcmgenet_tx_disabled();
        return;
    }
    prod_idx = s->tdmaregs[TDMA_PROD_INDEX >> 2] & DMA_P_INDEX_MASK;
    cons_idx = s->tdmaregs[TDMA_CONS_INDEX >> 2] & DMA_C_INDEX_MASK;
    desc_base = (prod_idx - 1)  * DMA_DESC_SIZE;

    addr = s->tdmaregs[(desc_base + DMA_DESC_ADDRESS_HI) >> 2];
    addr = (addr << 32) | s->tdmaregs[(desc_base + DMA_DESC_ADDRESS_LO) >> 2];      
    dma_len_stat = s->tdmaregs[(desc_base + DMA_DESC_LENGTH_STATUS) >> 2];
#if DEBUG    
printf("***********descbase 0x%x addrlo 0x%lx addrhi 0x%lx dma_len_stat 0x%x\n",desc_base,addr>>32, addr,dma_len_stat);
#endif
    trace_bcmgenet_tx_process(cons_idx, prod_idx);

    /* This is rather primitive for now, we just send everything we
     * can in one go, like e1000. Ideally we should do the sending
     * from some kind of background task
     */
  //  while (cons_idx!= prod_idx) {
        BCMGENETDesc bd;
        int len;
#if DEBUG        
printf("********in loop addr 0x%lx len_stat 0x%x prod_idx 0x%x\n",addr,dma_len_stat, prod_idx);
#endif
        if (bcmgenet_read_bd(&bd, addr) ||
            ((dma_len_stat & DMA_OWN) == 0)) {  //??use DMA_EOP??
            /* Run out of descriptors to transmit.  */
      //      s->isr |= FTGMAC100_INT_NO_NPTXBUF;
       //     break;
        }
//printf("*********bddes0 0x%lx\n",bd.des0);
//printf("*********bddes1 0x%lx\n",bd.des1);
//printf("*********bddes2 0x%lx\n",bd.des2);
//printf("*********bddes3 0x%lx\n",bd.des3);
//printf("*********bddes4 0x%lx\n",bd.des4);
//printf("*********bddes5 0x%lx\n",bd.des5);
    /* If it's a start of frame, discard anything we had in the
     * buffer and start again. This should be an error condition
     * if we had something ... for now we ignore it
     */
//    if (dma_len_stat & DMA_SOP) {
  //      if (s->tx_first_ctl) {
 ///           trace_bcmgenet_tx_unfinished();
   //     }
   //     s->frame_size = 0;
     //   s->tx_first_ctl = dma_len_stat;
  //  }

        len = (dma_len_stat >> DMA_BUFLENGTH_SHIFT) & DMA_BUFLENGTH_MASK;;
        if (frame_size + len > sizeof(s->frame)) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: frame too big : %d bytes\n",
                          __func__, len);
            trace_bcmgenet_tx_overflow();
            len =  sizeof(s->frame) - frame_size;
        }

        if (dma_memory_read(&address_space_memory, addr, ptr, len)) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: failed to read packet @ 0x%lx\n",
                          __func__, addr);
 //           s->isr |= FTGMAC100_INT_NO_NPTXBUF;
            return;
        }

        ptr += len;
        frame_size += len;
        if (dma_len_stat & DMA_EOP) {
            trace_bcmgenet_tx_finished(frame_size);        
            if (dma_len_stat & DMA_TX_APPEND_CRC) {
                net_checksum_calculate(s->frame, frame_size);
            }
#if 0   
    int i=0;

    printf("************crcbuf 0x");
        for (i = 0; i < frame_size; i++) {
printf("%02x ",s->frame[i]);
}
printf("\n");
#endif            
            /* Last buffer in frame.  */
            qemu_send_packet(qemu_get_queue(s->nic), s->frame, frame_size);
            ptr = s->frame;
            frame_size = 0;
        }
//    addr = s->rdmaregs[(desc_base + DMA_DESC_ADDRESS_HI) >> 2];
 //   addr = (addr << 32) | s->rdmaregs[(desc_base + DMA_DESC_ADDRESS_LO) >> 2];      
  //  dma_len_stat = s->rdmaregs[(desc_base + DMA_DESC_LENGTH_STATUS) >> 2];
        dma_len_stat &= ~DMA_OWN;
        /* Write back the modified descriptor.  */
//        bcmgenet_write_bd(&bd, addr);
//*ptr = bswap64((uint64_t)&s->frame);

dma_memory_write(&address_space_memory, addr, ptr, sizeof(ptr));
        /* Next ! */
        /* Advance to the next descriptor.  */
        /* Next ! */
   //     cons_idx = (cons_idx + 1) & DMA_C_INDEX_MASK;
    //    s->tdmaregs[TDMA_CONS_INDEX >> 2] = cons_idx;
              //  s->rdmaregs[RDMA_PROD_INDEX >> 2] = prod_idx;     
 //   }

    /* We sent everything, set status/irq bit */
 //   sungem_update_status(s, GREG_STAT_TXALL, true);
}
#endif

static bool bcmgenet_rx_full(BCMGENETState *s, uint32_t prod_idx, uint32_t cons_idx)
{
    return prod_idx == cons_idx + 1;// & s->rx_mask);
}

static int bcmgenet_can_receive(NetClientState *nc)
{
    BCMGENETState *s = qemu_get_nic_opaque(nc);
    uint32_t rdma_cfg, umac_cfg;
    uint32_t prod_idx, cons_idx;      
    bool full;

    rdma_cfg = s->rdmaregs[RDMA_CTRL >> 2];

    umac_cfg = s->umacregs[UMAC_CMD >> 2];
#if DEBUG    
printf("*****CAN RECEIVE umaccfg 0x%x cfg 0x%x\n",umac_cfg,rdma_cfg);
#endif
    /* If MAC disabled, can't receive */
    if ((umac_cfg & CMD_RX_EN) == 0) {
        trace_bcmgenet_rx_mac_disabled();
        return 0;
    }
    if ((rdma_cfg & DMA_EN) == 0) {
        trace_bcmgenet_rx_txdma_disabled();
        return 0;
    }

    /* Check RX availability */
    prod_idx = s->rdmaregs[RDMA_PROD_INDEX >> 2] & DMA_P_INDEX_MASK;
    cons_idx = s->rdmaregs[RDMA_CONS_INDEX >> 2] & DMA_C_INDEX_MASK;
    full = bcmgenet_rx_full(s, prod_idx, cons_idx);

    trace_bcmgenet_rx_check(!full, prod_idx, cons_idx);

    return !full;
}

enum {
        rx_no_match,
        rx_match_promisc,
        rx_match_bcast,
        rx_match_allmcast,
        rx_match_mcast,
        rx_match_mac,
        rx_match_altmac,
};
#if 1
static int bcmgenet_check_rx_mac(BCMGENETState *s, const uint8_t *mac, uint32_t crc)
{
    uint32_t     umac_cfg = s->umacregs[UMAC_CMD >> 2];
    uint32_t mac0, mac1, mac2;

    /* Promisc enabled ? */
    if (umac_cfg & CMD_PROMISC) {
        return rx_match_promisc;
    }

    /* Format MAC address into dwords */
    mac0 = (mac[4] << 8) | mac[5];
    mac1 = (mac[2] << 8) | mac[3];
    mac2 = (mac[0] << 8) | mac[1];

    trace_bcmgenet_rx_mac_check(mac0, mac1, mac2);

    /* Is this a broadcast frame ? */
    if (mac0 == 0xffff && mac1 == 0xffff && mac2 == 0xffff) {
        return rx_match_bcast;
    }

    /* TODO: Implement address filter registers (or we don't care ?) */

    /* Is this a multicast frame ? */
    if (mac[0] & 1) {
        trace_bcmgenet_rx_mac_multicast();

        /* Promisc group enabled ? */
 //       if (rxcfg & MAC_RXCFG_PGRP) {
   //         return rx_match_allmcast;
   //     }

        /* TODO: Check MAC control frames (or we don't care) ? */

        /* Check hash filter (somebody check that's correct ?) */
  //     if (rxcfg & MAC_RXCFG_HFE) {
     //       uint32_t hash, idx;
//
     //       crc >>= 24;
     //       idx = (crc >> 2) & 0x3c;
     //       hash = s->umacregs[(MAC_HASH0 + idx) >> 2];
     //       if (hash & (1 << (15 - (crc & 0xf)))) {
      //          return rx_match_mcast;
       //     }
      //  }
      //  return rx_no_match;
    }

    /* Main MAC check */
    trace_bcmgenet_rx_mac_compare(s->umacregs[UMAC_MAC1 >> 2],
                                s->umacregs[UMAC_MAC0 >> 2] & 0xffff  ,
                                s->umacregs[UMAC_MAC0 >> 2] >> 16);
    if (mac0 == s->umacregs[UMAC_MAC1 >> 2] &&
        mac1 == (s->umacregs[UMAC_MAC0 >> 2] & 0xffff) &&
        mac2 == s->umacregs[UMAC_MAC0 >> 2] >> 16) {
        return rx_match_mac;
    }

    /* Alt MAC check */ //FIXMEEEEEE
    if (mac0 == s->umacregs[UMAC_MAC0 >> 2] >> 8 &&
        mac1 == s->umacregs[UMAC_MAC0 >> 2] >> 16 &&
        mac2 == s->umacregs[UMAC_MAC0 >> 2] >> 24) {
        return rx_match_altmac;
    }

    return rx_no_match;
}
#endif
static ssize_t bcmgenet_receive(NetClientState *nc, const uint8_t *buf,
                              size_t size)
{
    BCMGENETState *s = qemu_get_nic_opaque(nc);

    uint32_t mac_crc, cons_idx, prod_idx, max_fsize;
    uint32_t fcs_size, /*ints,*/ rdma_cfg, umac_cfg, /*csum, coff,*/ buf_len, dma_len_stat;
    uint8_t smallbuf[60];
    BCMGENETDesc bd;
  
    unsigned int rx_cond;
    dma_addr_t addr;
    uint32_t desc_base;//, buf_addr;
    
    trace_bcmgenet_rx_packet(size);

    max_fsize = s->umacregs[UMAC_MAX_FRAME_LEN >> 2] & 0x7fff;

    rdma_cfg = s->rdmaregs[RDMA_CTRL >> 2];

    umac_cfg = s->umacregs[UMAC_CMD >> 2];
#if DEBUG    
printf("*****umaccfg 0x%x rdmacfg 0x%x\n",umac_cfg,rdma_cfg);
#endif
    /* If MAC or DMA disabled, can't receive */    
    if (!(rdma_cfg & DMA_EN) ||
        !(umac_cfg & CMD_RX_EN)) {
        trace_bcmgenet_rx_disabled();
        return 0;
    }
#if DEBUG    
printf("*******receive size 0x%lx\n",size);
#endif

    /* Size adjustment for FCS */
//    if (rmac_cfg & MAC_RXCFG_SFCS) {
  //      fcs_size = 0;
  //  } else {
  //      fcs_size = 4;
  //  }
//
    /* Discard frame smaller than a MAC or larger than max frame size
     * (when accounting for FCS)
     */
    if (size < 6 || (size + 4) > max_fsize) {
        trace_bcmgenet_rx_bad_frame_size(size);
        /* XXX Increment error statistics ? */
        return size;
    }

    /* We don't drop too small frames since we get them in qemu, we pad
     * them instead. We should probably use the min frame size register
     * but I don't want to use a variable size staging buffer and I
     * know both MacOS and Linux use the default 64 anyway. We use 60
     * here to account for the non-existent FCS.
     */
    if (size < 60) {
        memcpy(smallbuf, buf, size);
        memset(&smallbuf[size], 0, 60 - size);
        buf = smallbuf;
        size = 60;
    }
#if DEBUG    
    int i=0;

    printf("************buf 0x");
        for (i = 0; i < size; i++) {
printf("%02x",buf[i]);
}
printf("\n");
#endif
#if 1
    /* Get MAC crc */
    mac_crc = net_crc32_le(buf, ETH_ALEN);

    /* Packet isn't for me ? */
    rx_cond = bcmgenet_check_rx_mac(s, buf, mac_crc);
    if (rx_cond == rx_no_match) {
        /* Just drop it */
        trace_bcmgenet_rx_unmatched();
        return size;
    }
#endif
    /* Get ring pointers */
    prod_idx = s->rdmaregs[RDMA_PROD_INDEX >> 2] & DMA_P_INDEX_MASK;
    cons_idx = s->rdmaregs[RDMA_CONS_INDEX >> 2] & DMA_C_INDEX_MASK;
    trace_bcmgenet_rx_process(cons_idx, prod_idx);
        cons_idx = (cons_idx + 1) & DMA_C_INDEX_MASK;
        s->rdmaregs[RDMA_CONS_INDEX >> 2] = cons_idx; 
                prod_idx = (prod_idx + 1) & DMA_P_INDEX_MASK;
        s->rdmaregs[RDMA_PROD_INDEX >> 2] = prod_idx;  
    /* Ring full ? Can't receive */
    if (bcmgenet_rx_full(s, prod_idx, cons_idx)) {
        trace_bcmgenet_rx_ringfull();
        return 0;
    }

    /* Note: The real GEM will fetch descriptors in blocks of 4,
     * for now we handle them one at a time, I think the driver will
     * cope
     */
 
    desc_base = prod_idx * DMA_DESC_SIZE;
#if DEBUG    
printf("RX descbase 0x%x prod_idx 0x%x\n",desc_base,prod_idx);
#endif
    addr = s->rdmaregs[(desc_base + DMA_DESC_ADDRESS_HI) >> 2];
    addr = (addr << 32) | s->rdmaregs[(desc_base + DMA_DESC_ADDRESS_LO) >> 2];      
    dma_len_stat = s->rdmaregs[(desc_base + DMA_DESC_LENGTH_STATUS) >> 2];
    /* Read the next descriptor */
//    pci_dma_read(d, dbase + done * sizeof(desc), &desc, sizeof(desc));
#if DEBUG      
printf("***********RX descbase 0x%x addrlo 0x%lx addrhi 0x%lx dma_len_stat 0x%x\n",desc_base,addr>>32, addr,dma_len_stat);
    
#endif
    /* Effective buffer address */
//    baddr = le64_to_cpu(desc.buffer) & ~7ull;
 //   baddr |= (rxdma_cfg & RXDMA_CFG_FBOFF) >> 10;


 


    if (fcs_size) {
        /* Should we add an FCS ? Linux doesn't ask us to strip it,
         * however I believe nothing checks it... For now we just
         * do nothing. It's faster this way.
         */
    }
 //buf_len = 0x40;
        buf_len = (dma_len_stat >> DMA_BUFLENGTH_SHIFT) & DMA_BUFLENGTH_MASK;
        size -= buf_len;

        /* The last 4 bytes are the CRC.  */
        if (size < 4) {
   //         buf_len += size - 4;
        }
 //       buf_addr = bd.des3;
    /* Calculate the checksum */
 //   coff = (rdma_cfg & RXDMA_CFG_CSUMOFF) >> 13;
 //   csum = net_raw_checksum((uint8_t *)buf + coff, size - coff);
 
    /* Build the updated descriptor */
//    desc.status_word = (size + fcs_size) << 16;
 //   desc.status_word |= ((uint64_t)(mac_crc >> 16)) << 44;
 //   desc.status_word |= csum;
//    if (rx_cond == rx_match_mcast) {
 //       desc.status_word |= RXDCTRL_HPASS;
 //   }
  //  if (rx_cond == rx_match_altmac) {
  //      desc.status_word |= RXDCTRL_ALTMAC;
  //  }
//    desc.status_word = cpu_to_le64(desc.status_word);

//    pci_dma_write(d, dbase + done * sizeof(desc), &desc, sizeof(desc));
   dma_memory_write(&address_space_memory, addr+2, buf, buf_len);
      bcmgenet_read_bd(&bd, addr);
#if DEBUG      
printf("*********bddes0 0x%lx\n",bd.des0);
printf("*********bddes1 0x%lx\n",bd.des1);
printf("*********bddes2 0x%lx\n",bd.des2);
printf("*********bddes3 0x%lx\n",bd.des3); 
#endif
    trace_bcmgenet_rx_desc(le32_to_cpu(dma_len_stat),
                         le32_to_cpu(addr));
    /* Write buffer out */
//        bcmgenet_write_bd(&bd, addr);


    /* XXX Unconditionally set RX interrupt for now. The interrupt
     * mitigation timer might well end up adding more overhead than
     * helping here...
     */
//    ints = GREG_STAT_RXDONE;
  //  if (sungem_rx_full(s, kick, done)) {
  //      ints |= GREG_STAT_RXNOBUF;
  //  }
  //  sungem_update_status(s, ints, true);

    return size;
}

static void bcmgenet_reset(DeviceState *d)
{
    BCMGENETState *s = BCMGENET(d);
printf("******************************RESET******************************");
    /* Reset the BCMGENET */
#if 0     
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
#endif
    s->intrl2_0regs[INTRL2_CPU_STAT >> 2] = 0x802000;
    s->intrl2_0regs[INTRL2_CPU_MASK_STATUS >> 2] = 0x7feffff;

    /* and the PHY */
    phy_reset(s);
}
















static uint64_t bcmgenet_mmio_sys_read(void *opaque, hwaddr addr, unsigned size)
{
    BCMGENETState *s = opaque;
    uint32_t val;

    if (!(addr < 0x20) && !(addr >= 0x1000 && addr <= 0x1010)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "Read from unknown SYS register 0x%"HWADDR_PRIx"\n",
                      addr);
        return 0;
    }

    val = s->sysregs[addr >> 2];

    trace_bcmgenet_mmio_sys_read(GENET_SYS_OFF + addr, val);

    switch (addr) {
    case SYS_REV_CTRL: 
        val = 0x6000000;
        break;
    }
    return val;
}

static void bcmgenet_mmio_sys_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    BCMGENETState *s = opaque;

    if (!(addr < 0x20) && !(addr >= 0x1000 && addr <= 0x1010)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "Write to unknown SYS register 0x%"HWADDR_PRIx"\n",
                      addr);
        return;
    }

    trace_bcmgenet_mmio_sys_write(GENET_SYS_OFF + addr, val);

    /* Pre-write filter */
    switch (addr) {
    /* Read only registers */
    case SYS_REV_CTRL:            
        return; /* No actual write */
    }

    s->sysregs[addr  >> 2] = val;

    /* Post write action */
    switch (addr) {
    case SYS_PORT_CTRL:					
    case SYS_RBUF_FLUSH_CTRL:		
    case SYS_TBUF_FLUSH_CTRL:
        return;
    }
}



static const MemoryRegionOps bcmgenet_mmio_sys_ops = {
    .read = bcmgenet_mmio_sys_read,
    .write = bcmgenet_mmio_sys_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static uint64_t bcmgenet_mmio_ext_read(void *opaque, hwaddr addr, unsigned size)
{
    BCMGENETState *s = opaque;
    uint32_t val;

    if (!(addr < 0x20) && !(addr >= 0x1000 && addr <= 0x1010)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "Read from unknown EXT register 0x%"HWADDR_PRIx"\n",
                      addr);
        return 0;
    }

    val = s->extregs[addr >> 2];

    trace_bcmgenet_mmio_ext_read(GENET_EXT_OFF + addr, val);

    return val;
}


 
static void bcmgenet_mmio_ext_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    BCMGENETState *s = opaque;

    if (!(addr < 0x20) && !(addr >= 0x1000 && addr <= 0x1010)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "Write to unknown EXT register 0x%"HWADDR_PRIx"\n",
                      addr);
        return;
    }

    trace_bcmgenet_mmio_ext_write(GENET_EXT_OFF + addr, val);

    /* Pre-write filter */
//    switch (addr) {
    /* Read only registers */
 //      return;        

 //   }

    s->extregs[addr  >> 2] = val;

    /* Post write action */
    switch (addr) {
    case EXT_RGMII_OOB_CTRL:
    case EXT_EXT_PWR_MGMT:
    case EXT_GPHY_CTRL:
        break;
    }
}

static const MemoryRegionOps bcmgenet_mmio_ext_ops = {
    .read = bcmgenet_mmio_ext_read,
    .write = bcmgenet_mmio_ext_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static uint64_t bcmgenet_mmio_intrl2_0_read(void *opaque, hwaddr addr, unsigned size)
{
    BCMGENETState *s = opaque;
    uint32_t val;

    if (!(addr <= 0x20)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "Read from unknown INTRL2_0 register 0x%"HWADDR_PRIx"\n",
                      addr);
        return 0;
    }

    val = s->intrl2_0regs[addr >> 2];

    trace_bcmgenet_mmio_intrl2_0_read(GENET_INTRL2_0_OFF + addr, val);

    return val;
}

static void bcmgenet_mmio_intrl2_0_write(void *opaque, hwaddr addr, uint64_t val,
                                  unsigned size)
{
    BCMGENETState *s = opaque;

    if (!(addr <= 0x20)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "Write to unknown INTRL2_0 register 0x%"HWADDR_PRIx"\n",
                      addr);
        return;
    }

    trace_bcmgenet_mmio_intrl2_0_write(GENET_INTRL2_0_OFF + addr, val);

    /* Pre-write filter */
  //  switch (addr) {
    /* Read only registers */

  //      return;
  //  }

    s->intrl2_0regs[addr >> 2] = val;

    /* Post write action */
    switch (addr) {
    case INTRL2_CPU_STAT:
    break;			
    case INTRL2_CPU_SET:
    break;			
    case INTRL2_CPU_CLEAR:
        qemu_set_irq(s->irq157, 0);
    qemu_set_irq(s->irq158, 0);
    break;		
    case INTRL2_CPU_MASK_STATUS:
    break;		
    case INTRL2_CPU_MASK_SET:
        break;		
    case INTRL2_CPU_MASK_CLEAR:
        bcmgenet_update_irq(s);		
        break;

    }
}



static const MemoryRegionOps bcmgenet_mmio_intrl2_0_ops = {
    .read = bcmgenet_mmio_intrl2_0_read,
    .write = bcmgenet_mmio_intrl2_0_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

#if 1
static uint64_t bcmgenet_mmio_intrl2_1_read(void *opaque, hwaddr addr, unsigned size)
{
    BCMGENETState *s = opaque;
    uint32_t val;

    if (!(addr <= 0x1c)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "Read from unknown INTRL2_1 register 0x%"HWADDR_PRIx"\n",
                      addr);
        return 0;
    }

    val = s->intrl2_1regs[addr >> 2];

    trace_bcmgenet_mmio_intrl2_1_read(GENET_INTRL2_1_OFF + addr, val);

    return val;
}

static void bcmgenet_mmio_intrl2_1_write(void *opaque, hwaddr addr, uint64_t val,
                                  unsigned size)
{
    BCMGENETState *s = opaque;

    if (!(addr <= 0x1c)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "Write to unknown INTRL2_1 register 0x%"HWADDR_PRIx"\n",
                      addr);
        return;
    }

    trace_bcmgenet_mmio_intrl2_1_write(GENET_INTRL2_1_OFF + addr, val);

    /* Pre-write filter */
    switch (addr) {
    /* Read only registers */

    }

    s->intrl2_1regs[addr >> 2] = val;

    /* Post write action */
    switch (addr) {
    case INTRL2_CPU_STAT:			
    case INTRL2_CPU_SET:			
    case INTRL2_CPU_CLEAR:		
    case INTRL2_CPU_MASK_STATUS:		
    case INTRL2_CPU_MASK_SET:
        break;		
    case INTRL2_CPU_MASK_CLEAR:
        bcmgenet_update_irq(s);		
        break;
    }
}

static const MemoryRegionOps bcmgenet_mmio_intrl2_1_ops = {
    .read = bcmgenet_mmio_intrl2_1_read,
    .write = bcmgenet_mmio_intrl2_1_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static uint64_t bcmgenet_mmio_rbuf_read(void *opaque, hwaddr addr, unsigned size)
{
    BCMGENETState *s = opaque;
    uint32_t val;

    if (!(addr < 0x5ff)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "Read from unknown RBUF register 0x%"HWADDR_PRIx"\n",
                      addr);
        return 0;
    }

    val = s->rbufregs[addr >> 2];

    trace_bcmgenet_mmio_rbuf_read(GENET_RBUF_OFF + addr, val);

    return val;
}


 
static void bcmgenet_mmio_rbuf_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    BCMGENETState *s = opaque;

    if (!(addr < 0x5ff)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "Write to unknown RBUF register 0x%"HWADDR_PRIx"\n",
                      addr);
        return;
    }

    trace_bcmgenet_mmio_rbuf_write(GENET_RBUF_OFF + addr, val);

    /* Pre-write filter */
//    switch (addr) {
    /* Read only registers */
 //      return;        

 //   }

    s->rbufregs[addr  >> 2] = val;

    /* Post write action */
    switch (addr) {
    case EXT_RGMII_OOB_CTRL:
    case EXT_EXT_PWR_MGMT:
    case EXT_GPHY_CTRL:
        break;
    }
}

static const MemoryRegionOps bcmgenet_mmio_rbuf_ops = {
    .read = bcmgenet_mmio_rbuf_read,
    .write = bcmgenet_mmio_rbuf_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static uint64_t bcmgenet_mmio_tbuf_read(void *opaque, hwaddr addr, unsigned size)
{
    BCMGENETState *s = opaque;
    uint32_t val;

    if (!(addr < 0x7ff)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "Read from unknown TBUF register 0x%"HWADDR_PRIx"\n",
                      addr);
        return 0;
    }

    val = s->tbufregs[addr >> 2];

    trace_bcmgenet_mmio_tbuf_read(GENET_TBUF_OFF + addr, val);

    return val;
}


 
static void bcmgenet_mmio_tbuf_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    BCMGENETState *s = opaque;

    if (!(addr < 0x7ff)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "Write to unknown TBUF register 0x%"HWADDR_PRIx"\n",
                      addr);
        return;
    }

    trace_bcmgenet_mmio_tbuf_write(GENET_TBUF_OFF + addr, val);

    /* Pre-write filter */
//    switch (addr) {
    /* Read only registers */
 //      return;        

 //   }

    s->tbufregs[addr  >> 2] = val;

    /* Post write action */
    switch (addr) {
    case EXT_RGMII_OOB_CTRL:
    case EXT_EXT_PWR_MGMT:
    case EXT_GPHY_CTRL:
        break;
    }
}

static const MemoryRegionOps bcmgenet_mmio_tbuf_ops = {
    .read = bcmgenet_mmio_tbuf_read,
    .write = bcmgenet_mmio_tbuf_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};
static uint64_t bcmgenet_mmio_tdma_read(void *opaque, hwaddr addr, unsigned size)
{
    BCMGENETState *s = opaque;
    uint32_t val;

//    if (!(addr < 0xC0) && !(addr >= 0x00 && addr <= 0x3fff) {
    if (!(addr < 0x3fff)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "Read from unknown TDMA register 0x%"HWADDR_PRIx"\n",
                      addr);
        return 0;
    }

    val = s->tdmaregs[addr >> 2];

    trace_bcmgenet_mmio_tdma_read(GENET_TX_OFF + addr, val);

    return val;
}

static void bcmgenet_mmio_tdma_write(void *opaque, hwaddr addr, uint64_t val,
                                    unsigned size)
{
    BCMGENETState *s = opaque;

//    if (!(addr < 0xc0) && !(addr >= 0x00 && addr <= 0x3fff)) {
    if (!(addr < 0x3fff)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "Write to unknown TDMA register 0x%"HWADDR_PRIx"\n",
                      addr);
        return;
    }

    trace_bcmgenet_mmio_tdma_write(GENET_TX_OFF + addr, val);

    /* Pre-write filter */
    switch (addr) {
    /* Read only registers */

  //      return; /* No actual write */
    }

    s->tdmaregs[addr >> 2] = val;

    /* Post write action */
    switch (addr) { 
	
    case TDMA_PROD_INDEX:
         /* TODO: high priority tx ring */  
printf("**********transmit********8 0x%x\n",TDMA_PROD_INDEX);
        
         bcmgenet_do_tx(s);
            //     cons_idx = (cons_idx + 1) & DMA_C_INDEX_MASK;
        s->tdmaregs[TDMA_CONS_INDEX >> 2] += 1;
        if (bcmgenet_can_receive(qemu_get_queue(s->nic))) {
            qemu_flush_queued_packets(qemu_get_queue(s->nic));
        }
         break;
    case TDMA_READ_PTR:
    case TDMA_CONS_INDEX:	         
    case TDMA_RING_BUF_SIZE:		
    case TDMA_START_ADDR:		
    case TDMA_END_ADDR:			
    case TDMA_MBUF_DONE_THRESH:		
    case TDMA_FLOW_PERIOD:		
    case TDMA_WRITE_PTR:			

   case TDMA_RING_CFG:			
   case TDMA_CTRL:			
   case TDMASTATUS:	             
   case TDMA_SCB_BURST_SIZE:		
   case TDMAARBCTRL:	 		
   case TDMAPRIORITY0:			
   case TDMAPRIORITY1:			
   case TDMAPRIORITY2:			
        break;
              default:

        break;
    }

}


static const MemoryRegionOps bcmgenet_mmio_tdma_ops = {
    .read = bcmgenet_mmio_tdma_read,
    .write = bcmgenet_mmio_tdma_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static uint64_t bcmgenet_mmio_rdma_read(void *opaque, hwaddr addr, unsigned size)
{
    BCMGENETState *s = opaque;
    uint32_t val;

//    if (!(addr <= 0xc0) && !(addr >= 0x00 && addr <= 0x2fff)) {
    if (!(addr < 0x2fff)) {    
        qemu_log_mask(LOG_GUEST_ERROR,
                      "Read from unknown RXDMA register 0x%"HWADDR_PRIx"\n",
                      addr);
        return 0;
    }

    val = s->rdmaregs[addr >> 2];

    trace_bcmgenet_mmio_rdma_read(GENET_RX_OFF + addr, val);

    return val;
}

static void bcmgenet_mmio_rdma_write(void *opaque, hwaddr addr, uint64_t val,
                                    unsigned size)
{
    BCMGENETState *s = opaque;

//    if (!(addr <= 0xc0) && !(addr >= 0x00 && addr <= 0x2fff)) {
    if (!(addr < 0x2fff)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "Write to unknown RDMA register 0x%"HWADDR_PRIx"\n",
                      addr);
        return;
    }

    trace_bcmgenet_mmio_rdma_write(GENET_RX_OFF + addr, val);

    /* Pre-write filter */
    switch (addr) {
    /* Read only registers */

 //       return; /* No actual write */
    }

    s->rdmaregs[addr >> 2] = val;

    /* Post write action */
    switch (addr) {
    case RDMA_READ_PTR:
    case RDMA_CONS_INDEX:		
    case RDMA_PROD_INDEX:		
    case RDMA_RING_BUF_SIZE:		
    case RDMA_START_ADDR:		
    case RDMA_END_ADDR:			
    case RDMA_MBUF_DONE_THRESH:	
    case RDMA_XON_XOFF_THRESH:			
	
    case RDMA_WRITE_PTR:			

    case RDMA_RING_CFG:			
    case RDMA_CTRL:			
    case RDMASTATUS:	             
    case RDMA_SCB_BURST_SIZE:	
    case RDMATIMEOUT0:
    case RDMAINDEX2RING0:    

        break;
    }
}



static const MemoryRegionOps bcmgenet_mmio_rdma_ops = {
    .read = bcmgenet_mmio_rdma_read,
    .write = bcmgenet_mmio_rdma_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static uint64_t bcmgenet_mmio_umac_read(void *opaque, hwaddr addr, unsigned size)
{
    BCMGENETState *s = opaque;
    uint32_t val;

    if (!(addr <= 0x1000)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "Read from unknown UMAC register 0x%"HWADDR_PRIx"\n",
                      addr);
        return 0;
    }

    val = s->umacregs[addr >> 2];
    
    switch (addr) {
    case UMAC_MAC0:
        s->umacregs[addr >> 2] = ((uint32_t) s->conf.macaddr.a[5] << 24) |
            (s->conf.macaddr.a[4] << 16) | (s->conf.macaddr.a[3] << 8) |
            s->conf.macaddr.a[2];    
        break;
    case UMAC_MAC1:
        s->umacregs[addr >> 2] = (s->conf.macaddr.a[0] << 8)  | s->conf.macaddr.a[1];
        break;
    }   
    trace_bcmgenet_mmio_umac_read(GENET_UMAC_OFF + addr, val);

    return val;
}

static void bcmgenet_mmio_umac_write(void *opaque, hwaddr addr, uint64_t val,
                                  unsigned size)
{
    BCMGENETState *s = opaque;

    if (!(addr <= 0x1000)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "Write to unknown UMAC register 0x%"HWADDR_PRIx"\n",
                      addr);
    //    return;
    }

    trace_bcmgenet_mmio_umac_write(GENET_UMAC_OFF + addr, val);

    /* Pre-write filter */
    switch (addr) {
    /* Read only registers */

  //      return; /* No actual write */
//    case MAC_MINFSZ:
        /* 10-bits implemented */
//        val &= 0x3ff;
   //     break;
    }

    s->umacregs[addr >> 2] = val;

    /* Post write action */
    switch (addr) {


    case UMAC_CMD:
        if (val & (CMD_SW_RESET | CMD_LCL_LOOP_EN)) {
            bcmgenet_reset(DEVICE(s));
         printf("***********RESET detected********\n");
        }

    //    if (bcmgenet_can_receive(qemu_get_queue(s->nic))) {
      //      qemu_flush_queued_packets(qemu_get_queue(s->nic));
       // }			
    case UMAC_MAC0: /* MAC */
        s->conf.macaddr.a[5] = val >> 24;
        s->conf.macaddr.a[4] = val >> 16;
        s->conf.macaddr.a[3] = val >> 8;
        s->conf.macaddr.a[2] = val;    

        break;
    case UMAC_MAC1: 
        s->conf.macaddr.a[0] = val >> 8;
        s->conf.macaddr.a[1] = val;
        break;		
    case UMAC_MAX_FRAME_LEN:		
    case UMACEEECTRL:		
    case UMACEEELPITIMER:
    case UMACEEEWAKETIMER:
    case UMACEEEREFCOUNT:
    case UMAC_TX_FLUSH:
    case UMAC_MIB_CTRL:			
    case MDIO_CMD:				
    case UMACMPDCTRL:	
    case UMACMDFCTRL:	
    case UMACMDFADDR0:	
        break;
    }
}



static const MemoryRegionOps bcmgenet_mmio_umac_ops = {
    .read = bcmgenet_mmio_umac_read,
    .write = bcmgenet_mmio_umac_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
}; 
 

static uint64_t bcmgenet_mmio_hfb_read(void *opaque, hwaddr addr, unsigned size)
{
    BCMGENETState *s = opaque;
    uint32_t val;

    if (!(addr < 0x7c20)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "Read from unknown HFB register 0x%"HWADDR_PRIx"\n",
                      addr);
        return 0;
    }

    val = s->hfbregs[addr >> 2];

    trace_bcmgenet_mmio_hfb_read(GENET_HFB_OFF + addr, val);

     return val;
}

static void bcmgenet_mmio_hfb_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    BCMGENETState *s = opaque;

    if (!(addr < 0x7c20)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "Write to unknown HFB register 0x%"HWADDR_PRIx"\n",
                      addr);
        return;
    }

    trace_bcmgenet_mmio_hfb_write(GENET_HFB_OFF + addr, val);

    /* Pre-write filter */
    switch (addr) {
    /* Read only registers */

    }

    s->hfbregs[addr  >> 2] = val;

    /* Post write action */
    switch (addr) {
    case HFBCTLR:	
    case HFBFLTENABLE:	
    case HFBFLTLEN:
        return;
    }
}



static const MemoryRegionOps bcmgenet_mmio_hfb_ops = {
    .read = bcmgenet_mmio_hfb_read,
    .write = bcmgenet_mmio_hfb_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};



 
#endif
#if 0
static void bcmgenet_init(Object *obj)
{ 
    UNIMACState *s = UNIMAC_MDIO(obj); 
                             
    sysbus_init_child_obj(obj, "unimac_mdio", &s->iomem, sizeof(s->iomem),
                             TYPE_UNIMAC_MDIO);   
 
}
#endif
static void bcmgenet_cleanup(NetClientState *nc)
{
    BCMGENETState *s = BCMGENET(qemu_get_nic_opaque(nc));

    s->nic = NULL;
}

static NetClientInfo net_bcmgenet_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NICState),
    .can_receive = bcmgenet_can_receive,
    .receive = bcmgenet_receive,
    .cleanup = bcmgenet_cleanup,
//    .link_status_changed = bcmgenet_set_link,
};

static void bcmgenet_realize(DeviceState *dev, Error **errp)
{
    BCMGENETState *s = BCMGENET(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

//    bcmgenet_reset_all(s, true);
    memory_region_init(&s->bcmgenet, OBJECT(s), "bcmgenet", GENET_MMIO_SIZE);

    memory_region_init_io(&s->sys, OBJECT(s), &bcmgenet_mmio_sys_ops, s,
                          "bmcgenet.sys", BCMGENET_MMIO_SYS_SIZE);
    memory_region_add_subregion(&s->bcmgenet, GENET_SYS_OFF, &s->sys);
//    sysbus_init_mmio(sbd, &s->sys);

    memory_region_init_io(&s->ext, OBJECT(s), &bcmgenet_mmio_ext_ops, s,
                          "bcmgenet.ext", BCMGENET_MMIO_EXT_SIZE);
    memory_region_add_subregion(&s->bcmgenet, GENET_EXT_OFF, &s->ext);
   
    memory_region_init_io(&s->intrl2_0, OBJECT(s), &bcmgenet_mmio_intrl2_0_ops, s,
                          "bcmgenet.intrl2_0", BCMGENET_MMIO_INTRL2_0_SIZE);
    memory_region_add_subregion(&s->bcmgenet, GENET_INTRL2_0_OFF, &s->intrl2_0);

    memory_region_init_io(&s->intrl2_1, OBJECT(s), &bcmgenet_mmio_intrl2_1_ops, s,
                          "bcmgenet.intrl2_1", BCMGENET_MMIO_INTRL2_1_SIZE);
    memory_region_add_subregion(&s->bcmgenet, GENET_INTRL2_1_OFF, &s->intrl2_1);  
   
    memory_region_init_io(&s->tdma, OBJECT(s), &bcmgenet_mmio_tdma_ops, s,
                          "bcmgenet.tdma", BCMGENET_MMIO_TDMA_SIZE);
    memory_region_add_subregion(&s->bcmgenet, GENET_TX_OFF, &s->tdma);

    memory_region_init_io(&s->rdma, OBJECT(s), &bcmgenet_mmio_rdma_ops, s,
                          "bcmgenet.rdma", BCMGENET_MMIO_RDMA_SIZE);
    memory_region_add_subregion(&s->bcmgenet, GENET_RX_OFF, &s->rdma);

    memory_region_init_io(&s->umac, OBJECT(s), &bcmgenet_mmio_umac_ops, s,
                          "bcmgenet.umac", BCMGENET_MMIO_UMAC_SIZE);
    memory_region_add_subregion(&s->bcmgenet, GENET_UMAC_OFF, &s->umac);
#if 1
    memory_region_init_io(&s->rbuf, OBJECT(s), &bcmgenet_mmio_rbuf_ops, s,
                          "bcmgenet.rbuf", BCMGENET_MMIO_RBUF_SIZE);
    memory_region_add_subregion(&s->bcmgenet, GENET_RBUF_OFF, &s->rbuf);

    memory_region_init_io(&s->tbuf, OBJECT(s), &bcmgenet_mmio_tbuf_ops, s,
                          "bcmgenet.tbuf", BCMGENET_MMIO_TBUF_SIZE);
    memory_region_add_subregion(&s->bcmgenet, GENET_TBUF_OFF, &s->tbuf);
    memory_region_init_io(&s->hfb, OBJECT(s), &bcmgenet_mmio_hfb_ops, s,
                          "bcmgenet.hfb", BCMGENET_MMIO_HFB_SIZE);
    memory_region_add_subregion(&s->bcmgenet, GENET_HFB_OFF, &s->hfb);    
#endif
#if 0
        object_property_set_link(OBJECT(&s->unimac_mdio), OBJECT(&s->bcmgenet),
                                 "nic", &error_abort);
        object_property_set_bool(OBJECT(&s->unimac_mdio), true, "realized",
                                 &err);
        if (err) {
            error_propagate(errp, err);
            return;
        }
memory_region_add_subregion(&s->bcmgenet, GENET_UMAC_OFF,
                sysbus_mmio_get_region(SYS_BUS_DEVICE(&mdio->iomem), 0)); 
#endif                
    sysbus_init_irq(sbd, &s->irq157);
    sysbus_init_irq(sbd, &s->irq158);
    qemu_macaddr_default_if_unset(&s->conf.macaddr);
    s->nic = qemu_new_nic(&net_bcmgenet_info, &s->conf,
                          object_get_typename(OBJECT(dev)),
                          DEVICE(dev)->id, s);
    qemu_format_nic_info_str(qemu_get_queue(s->nic),
                             s->conf.macaddr.a);
                             
}

static const VMStateDescription vmstate_bcmgenet = {
    .name = TYPE_BCMGENET,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(isr, BCMGENETState),
        VMSTATE_UINT32(ier, BCMGENETState),
#if 0 
        VMSTATE_UINT32(rx_enabled, BCMGENETState),
        VMSTATE_UINT32(rx_ring, BCMGENETState),
        VMSTATE_UINT32(rbsr, BCMGENETState),
        VMSTATE_UINT32(tx_ring, BCMGENETState),
        VMSTATE_UINT32(rx_descriptor, BCMGENETState),
        VMSTATE_UINT32(tx_descriptor, BCMGENETState),
        VMSTATE_UINT32_ARRAY(math, BCMGENETState, 2),
        VMSTATE_UINT32(itc, BCMGENETState),
        VMSTATE_UINT32(aptcr, BCMGENETState),
        VMSTATE_UINT32(dblac, BCMGENETState),
        VMSTATE_UINT32(revr, BCMGENETState),
        VMSTATE_UINT32(fear1, BCMGENETState),
        VMSTATE_UINT32(tpafcr, BCMGENETState),
        VMSTATE_UINT32(maccr, BCMGENETState),
        VMSTATE_UINT32(phycr, BCMGENETState),
        VMSTATE_UINT32(phydata, BCMGENETState),
        VMSTATE_UINT32(fcr, BCMGENETState),
        VMSTATE_UINT32(phy_status, BCMGENETState),
        VMSTATE_UINT32(phy_control, BCMGENETState),
        VMSTATE_UINT32(phy_advertise, BCMGENETState),
        VMSTATE_UINT32(phy_int, BCMGENETState),
        VMSTATE_UINT32(phy_int_mask, BCMGENETState),
        VMSTATE_UINT32(txdes0_edotr, BCMGENETState),
        VMSTATE_UINT32(rxdes0_edorr, BCMGENETState),
#endif       
        VMSTATE_END_OF_LIST()
    }
};

static Property bcmgenet_properties[] = {
    DEFINE_PROP_BOOL("unimac_mdio", BCMGENETState, unimac_mdio, false),
    DEFINE_NIC_PROPERTIES(BCMGENETState, conf),
    DEFINE_PROP_UINT32("phy_addr", BCMGENETState, phy_addr, 0),       
    DEFINE_PROP_END_OF_LIST(),
};

static void bcmgenet_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->vmsd = &vmstate_bcmgenet;
    dc->reset = bcmgenet_reset;
    device_class_set_props(dc, bcmgenet_properties);
    set_bit(DEVICE_CATEGORY_NETWORK, dc->categories);
    dc->realize = bcmgenet_realize;
    dc->desc = "BCMGENET Gigabit Ethernet emulation";
}

static const TypeInfo bcmgenet_info = {
    .name = TYPE_BCMGENET,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(BCMGENETState),
//    .instance_init = bcmgenet_init,    
    .class_init = bcmgenet_class_init,
};

static void bcmgenet_register_types(void)
{
    type_register_static(&bcmgenet_info);
//    type_register_static(&bcmgenet_mii_info);
}

type_init(bcmgenet_register_types)
