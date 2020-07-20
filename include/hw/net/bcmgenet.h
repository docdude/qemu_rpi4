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
    uint32_t tx_mask;

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

#endif
