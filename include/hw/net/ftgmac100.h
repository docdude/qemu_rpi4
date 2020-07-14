/*
 * Faraday FTGMAC100 Gigabit Ethernet
 *
 * Copyright (C) 2016-2017, IBM Corporation.
 *
 * This code is licensed under the GPL version 2 or later. See the
 * COPYING file in the top-level directory.
 */

#ifndef FTGMAC100_H
#define FTGMAC100_H

#define TYPE_FTGMAC100 "bcm2711-genet-v5"
#define FTGMAC100(obj) OBJECT_CHECK(FTGMAC100State, (obj), TYPE_FTGMAC100)

#include "hw/sysbus.h"
#include "net/net.h"

/*
 * Max frame size for the receiving buffer
 */
#define FTGMAC100_MAX_FRAME_SIZE    9220
#define ARCH_DMA_MINALIGN 64
typedef struct FTGMAC100State {
    /*< private >*/
    SysBusDevice parent_obj;

    /*< public >*/
    NICState *nic;
    NICConf conf;
    qemu_irq irq;
    MemoryRegion iomem;

    uint8_t frame[FTGMAC100_MAX_FRAME_SIZE];

    uint32_t irq_state;
    uint32_t isr;
    uint32_t ier;
    uint32_t rx_enabled;
    uint32_t rx_ring;
    uint32_t rx_descriptor;
    uint32_t tx_ring;
    uint32_t tx_descriptor;
    uint32_t math[2];
    uint32_t rbsr, tbsr;
    uint32_t itc;
    uint32_t aptcr;
    uint32_t dblac;
    uint32_t revr;
    uint32_t fear1;
    uint32_t tpafcr;
    uint32_t maccr;
    uint32_t phycr;
    uint32_t phydata;
    uint32_t phymode;
    uint32_t fcr;
    uint32_t mdio_cmd;
    uint32_t mib_ctl;
    uint32_t rgmii_oob_ctl;
    uint32_t max_frm_len;
    uint32_t rbuf_ctl;
    uint32_t rbtb_sz_ctl;
    uint32_t tx_flush;
    uint32_t tdma_ctl;
    uint32_t rdma_ctl;
    uint32_t rdma_scb_burst_size;

    uint32_t rdma_ring_dma_start_addr;
    uint32_t rdma_rd_ptr;
    uint32_t rdma_wr_ptr;
    uint32_t rdma_ring_dma_end_addr;
    uint32_t rdma_prod_index;
    uint32_t rdma_cons_index;
    uint32_t rdma_ring_dma_ring_buf_size;
    uint32_t rdma_xon_xoff_thresh;
    uint32_t rdma_dma_ring_cfg;   
        
    uint32_t tdma_dma_scb_burst_size;

    uint32_t tdma_ring_dma_start_addr;
    uint32_t tdma_rd_ptr;
    uint32_t tdma_wr_ptr;
    uint32_t tdma_ring_dma_end_addr;
    uint32_t tdma_prod_index;
    uint32_t tdma_cons_index;
    uint32_t tdma_ring_dma_mbuf_done_thresh;
    uint32_t tdma_flow_period;
    uint32_t tdma_ring_dma_ring_buf_size;

    uint32_t tdma_dma_ring_cfg;        
    uint32_t tdma_daddlo[256];
    uint32_t tdma_daddhi[256];
    uint32_t tdma_dls[256];
    uint32_t rdma_daddlo[256];
    uint32_t rdma_daddhi[256];
    uint32_t rdma_dls[256];    
    uint32_t phy_addr;    
    uint32_t phy_status;
    uint32_t phy_control;
    uint32_t phy_advertise;
    uint32_t phy_ctrl1000;
    uint32_t phy_int;
    uint32_t phy_int_mask;
    uint32_t phy_nconfig;  
    uint32_t phy_resv1;  

    bool aspeed;
    uint32_t txdes0_edotr;
    uint32_t rxdes0_edorr;
} FTGMAC100State;

#define TYPE_ASPEED_MII "unimac_mdio"
#define ASPEED_MII(obj) OBJECT_CHECK(AspeedMiiState, (obj), TYPE_ASPEED_MII)

/*
 * AST2600 MII controller
 */
typedef struct AspeedMiiState {
    /*< private >*/
    SysBusDevice parent_obj;

    FTGMAC100State *nic;

    MemoryRegion iomem;

    uint32_t phycr;
    uint32_t phydata;
    uint32_t mdio_clk;    
} AspeedMiiState;

#endif
