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
#include "qemu-common.h"
#include "hw/net/bcmgenet.h"




static void bcmgenet_set_irq(BCMGENETState *s, int reg)
{
    if (reg) {
      qemu_set_irq(s->intrl2_1, 1);
    } else {
      qemu_set_irq(s->intrl2_0, 1);
    }
}

static void bcmgenet_clear_irq(BCMGENETState *s, int reg)
{
    if (reg) {
      qemu_set_irq(s->intrl2_1, 0);
    } else {
      qemu_set_irq(s->intrl2_0, 0);
    }
}

static void bcmgenet_eval_irq(BCMGENETState *s, int reg)
{
    uint32_t stat, mask;

    if (reg) {
      mask = s->intrl2_1regs[INTRL2_CPU_MASK_STATUS >> 2];
      stat = s->intrl2_1regs[INTRL2_CPU_STAT >> 2]; 
      if (stat & ~mask) {
        bcmgenet_set_irq(s, 1);
      } else {
        bcmgenet_clear_irq(s, 1);
      }
    } else {
      mask = s->intrl2_0regs[INTRL2_CPU_MASK_STATUS >> 2];
      stat = s->intrl2_0regs[INTRL2_CPU_STAT >> 2]; 
//      printf("stat 0x%x\n",stat);
//      printf("mask 0x%x\n",mask);
      if (stat & ~mask) {
        bcmgenet_set_irq(s,0);
      } else {
        bcmgenet_clear_irq(s,0);
      }
    }
    
}


static void phy_update_link(BCMGENETState *s)
{ printf("***********************LINK**********\n");
    trace_bcmgenet_link_status_changed(qemu_get_queue(s->nic)->link_down ? false : true);
    /* Autonegotiation status mirrors link status.  */
    if (qemu_get_queue(s->nic)->link_down) {
        s->phy_status &= ~(MII_BMSR_LINK_ST | MII_BMSR_AN_COMP);
//        s->phy_int |= PHY_INT_DOWN;
   	bcmgenet_update_status(s, UMAC_IRQ_LINK_DOWN, true, 0);
    } else {
        s->phy_status |= (MII_BMSR_LINK_ST | MII_BMSR_AN_COMP);
//        s->phy_int |= PHY_INT_AUTONEG_COMPLETE;
   	bcmgenet_update_status(s, UMAC_IRQ_LINK_UP, true, 0);
    }
//    phy_update_irq(s);
}

static void bcmgenet_set_link_status(NetClientState *nc)
{   /* OS driver does not use phy interrupt */
    phy_update_link(BCMGENET(qemu_get_nic_opaque(nc)));
}

void bcmgenet_update_status(BCMGENETState *s, uint32_t bits, bool val, int reg)
{
    uint32_t stat;

    if (reg) {
      stat = s->intrl2_1regs[INTRL2_CPU_STAT >> 2];
      if (val) {
        stat |= bits;
      } else {
          if (bits == 0xffffffff) {
//stat = (stat || bits) && !(stat && bits);
	    stat = ~(stat | bits);
          } else {
            stat &= ~bits;
          }     
      }

      s->intrl2_1regs[INTRL2_CPU_STAT >> 2] = stat;
      bcmgenet_eval_irq(s, 1);
    } else {
      stat = s->intrl2_0regs[INTRL2_CPU_STAT >> 2];
      if (val) {
        stat |= bits;
        s->intrl2_0regs[(INTRL2_CPU_CLEAR >> 2) + 0x04] |= stat;
      } else {

          if (bits == 0xffffffff) {
//        
//stat = (stat || bits) && !(stat && bits);
	     stat = ~(stat | bits);
          } else {
            stat &= ~bits;
          }
      }    
      s->intrl2_0regs[INTRL2_CPU_STAT >> 2] = stat;

      bcmgenet_eval_irq(s, 0);
    }
}

static void bcmgenet_update_mask_status(BCMGENETState *s, uint32_t bits, bool val, int reg)
{
    uint32_t stat;

    if (reg) {
      stat = s->intrl2_1regs[INTRL2_CPU_MASK_STATUS >> 2];
      if (val) {
          if (bits == 0xffffffff) { 
          stat = ~(stat ^ bits);
//stat = (stat || bits) && !(stat && bits);

          } else {
            stat |= bits;
          } 
      } else {
        stat &= ~bits;
      }

      s->intrl2_1regs[INTRL2_CPU_MASK_STATUS >> 2] = stat;
      bcmgenet_eval_irq(s, 1);
    } else {     
      stat = s->intrl2_0regs[INTRL2_CPU_MASK_STATUS >> 2];
      if (val) {
          if (bits == 0xffffffff) { 
          stat = ~(stat ^ bits);
//stat = (stat || bits) && !(stat && bits);

          } else {
            stat |= bits;
          }  
      } else {
        stat &= ~bits;
      }

      s->intrl2_0regs[INTRL2_CPU_MASK_STATUS >> 2] = stat;
      bcmgenet_eval_irq(s, 0);
    }
}






 
static void bcmgenet_do_tx(BCMGENETState *s, int ring)
{
    dma_addr_t addr;
    int frame_size = 0;
    uint8_t *ptr = s->frame;
    uint32_t tdma_cfg, umac_cfg, ints;  
    uint32_t prod_idx, cons_idx, write_ptr;  
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
    s->ring_idx = ring;
            switch (ring) {
           case 0:
               prod_idx = s->tdmaregs[TDMA_PROD_INDEX_Q0 >> 2] & DMA_P_INDEX_MASK;
               cons_idx = s->tdmaregs[TDMA_CONS_INDEX_Q0 >> 2] & DMA_C_INDEX_MASK;
               write_ptr = s->tdmaregs[TDMA_WRITE_PTR_Q0 >> 2];
           break;
           case 1:
               prod_idx = s->tdmaregs[TDMA_PROD_INDEX_Q1 >> 2] & DMA_P_INDEX_MASK;
 	       cons_idx = s->tdmaregs[TDMA_CONS_INDEX_Q1 >> 2] & DMA_C_INDEX_MASK;
               write_ptr = s->tdmaregs[TDMA_WRITE_PTR_Q1 >> 2]; 	       
           break;
           case 2:
               prod_idx = s->tdmaregs[TDMA_PROD_INDEX_Q2 >> 2] & DMA_P_INDEX_MASK;
    	       cons_idx = s->tdmaregs[TDMA_CONS_INDEX_Q2 >> 2] & DMA_C_INDEX_MASK;
    	       write_ptr = s->tdmaregs[TDMA_WRITE_PTR_Q2 >> 2];
           break;
           case 3:
               prod_idx = s->tdmaregs[TDMA_PROD_INDEX_Q3 >> 2] & DMA_P_INDEX_MASK;
  	       cons_idx = s->tdmaregs[TDMA_CONS_INDEX_Q3 >> 2] & DMA_C_INDEX_MASK;
  	       write_ptr = s->tdmaregs[TDMA_WRITE_PTR_Q3 >> 2];
           break;
           case 16:
               prod_idx = s->tdmaregs[TDMA_PROD_INDEX_Q16 >> 2] & DMA_P_INDEX_MASK;
               cons_idx = s->tdmaregs[TDMA_CONS_INDEX_Q16 >> 2] & DMA_C_INDEX_MASK;
               write_ptr = s->tdmaregs[TDMA_WRITE_PTR_Q16 >> 2];
           break;
        }    
//    prod_idx = s->tdmaregs[TDMA_PROD_INDEX >> 2] & DMA_P_INDEX_MASK;
//    cons_idx = s->tdmaregs[TDMA_CONS_INDEX >> 2] & DMA_C_INDEX_MASK;
//    desc_base = (prod_idx - 1)  * DMA_DESC_SIZE;
//    desc_base = prod_idx  * DMA_DESC_SIZE;
//    desc_base = ((ring * (write_ptr/(3*ring))) + (prod_idx-1)) * DMA_DESC_SIZE;
    desc_base = ((write_ptr/3) + (prod_idx-1)) * DMA_DESC_SIZE;
    addr = s->tdmaregs[(desc_base + DMA_DESC_ADDRESS_HI) >> 2];
    addr = (addr << 32) | s->tdmaregs[(desc_base + DMA_DESC_ADDRESS_LO) >> 2];      
    dma_len_stat = s->tdmaregs[(desc_base + DMA_DESC_LENGTH_STATUS) >> 2];
//#if DEBUG    
printf("***********prod_idx 0x%x writeptr 0x%x descbase 0x%x addrlo 0x%lx addrhi 0x%lx dma_len_stat 0x%x\n",prod_idx, write_ptr, desc_base,addr>>32, addr,dma_len_stat);
//#endif
    trace_bcmgenet_tx_process(cons_idx, prod_idx);

    /* This is rather primitive for now, we just send everything we
     * can in one go, like e1000. Ideally we should do the sending
     * from some kind of background task
     */
  //  while (cons_idx != prod_idx) {
if (cons_idx != prod_idx) {
        int len;
#if DEBUG        
printf("********in loop addr 0x%lx len_stat 0x%x prod_idx 0x%x\n",addr,dma_len_stat, prod_idx);
#endif
        if ((dma_len_stat & DMA_OWN) == 0) {  //??use DMA_EOP??
            /* Run out of descriptors to transmit.  */
      //      s->isr |= FTGMAC100_INT_NO_NPTXBUF;
       //     break;
        }

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
        trace_bcmgenet_tx_desc(cons_idx, le32_to_cpu(dma_len_stat), le32_to_cpu(addr));
        
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

            return;
        }
        qemu_hexdump((void *)ptr, stderr, "", len);
        ptr += len;
        frame_size += len;
        if (dma_len_stat & DMA_EOP) {
            trace_bcmgenet_tx_finished(frame_size);        
            if (dma_len_stat & DMA_TX_APPEND_CRC) {
                net_checksum_calculate(s->frame, frame_size);
            }
#if DEBUG   
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

       dma_len_stat &= ~DMA_OWN;

       /* Write back the modified descriptor.  */
       dma_memory_write(&address_space_memory, addr, ptr, sizeof(ptr));



       /* Interrupt */
       if (ring < 16) {
         ints = UMAC_IRQ_TXDMA_MBDONE | UMAC_IRQ_TXDMA_PDONE | UMAC_IRQ_TXDMA_BDONE;
         bcmgenet_update_status(s, ints, true, 0);

         ints = 1 << ring;
         bcmgenet_update_status(s, ints, true, 1);
       } else {
                ints = UMAC_IRQ_TXDMA_MBDONE | UMAC_IRQ_TXDMA_PDONE | UMAC_IRQ_TXDMA_BDONE;
         bcmgenet_update_status(s, UMAC_IRQ_TXDMA_DONE, true, 0);
       }  
        
       /* Advance to the next descriptor.  */   
       /* Update consumer index */
       cons_idx = (cons_idx + 1) & DMA_C_INDEX_MASK;
       switch (ring) {
          case 0:
          s->tdmaregs[TDMA_CONS_INDEX_Q0 >> 2] = cons_idx;
          break;
          case 1:
          s->tdmaregs[TDMA_CONS_INDEX_Q1 >> 2] = cons_idx;
          break;
          case 2:
          s->tdmaregs[TDMA_CONS_INDEX_Q2 >> 2] = cons_idx;
          break;
          case 3:
          s->tdmaregs[TDMA_CONS_INDEX_Q3 >> 2] = cons_idx;
          break;
          case 16:
          s->tdmaregs[TDMA_CONS_INDEX_Q16 >> 2] = cons_idx;
          break;
       }         

    }
    /* We sent everything, set status/irq bit */
    /* Interrupt TX Done */
                    ints = UMAC_IRQ_TXDMA_MBDONE | UMAC_IRQ_TXDMA_PDONE | UMAC_IRQ_TXDMA_BDONE;
    bcmgenet_update_status(s, ints, true, 0);
}


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
  // 	bcmgenet_update_status(s, UMAC_IRQ_LINK_UP, true, 0);
    /* Check RX availability */
    prod_idx = s->rdmaregs[RDMA_PROD_INDEX_Q16 >> 2] & DMA_P_INDEX_MASK;
    cons_idx = s->rdmaregs[RDMA_CONS_INDEX_Q16 >> 2] & DMA_C_INDEX_MASK;
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
    uint32_t fcs_size, ints, rdma_cfg, umac_cfg; /*csum, coff,*/ 
    uint8_t smallbuf[60];
  
    unsigned int rx_cond;
    dma_addr_t addr;
    uint32_t desc_base, buf_len, dma_len_stat, dma_flags;

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

    if (s->umacregs[UMAC_CMD >> 2] & CMD_CRC_FWD) {
	size += 4;//ETH_FCS_LEN;
    }
    /* Get ring pointers */
    prod_idx = s->rdmaregs[RDMA_PROD_INDEX_Q16 >> 2] & DMA_P_INDEX_MASK;
    cons_idx = s->rdmaregs[RDMA_CONS_INDEX_Q16 >> 2] & DMA_C_INDEX_MASK;
    trace_bcmgenet_rx_process(cons_idx, prod_idx);
 
        desc_base = (prod_idx) * DMA_DESC_SIZE;
#if DEBUG    
printf("RX descbase 0x%x prod_idx 0x%x\n",desc_base,prod_idx);
#endif
//size +=4;
    addr = s->rdmaregs[(desc_base + DMA_DESC_ADDRESS_HI) >> 2];
    addr = (addr << 32) | s->rdmaregs[(desc_base + DMA_DESC_ADDRESS_LO) >> 2];      
    dma_len_stat = s->rdmaregs[(desc_base + DMA_DESC_LENGTH_STATUS) >> 2];
    /* Read the next descriptor */
#if 1    
    dma_len_stat = size << DMA_BUFLENGTH_SHIFT;
    dma_len_stat |= 0x3F << DMA_TX_QTAG_SHIFT;
    dma_len_stat |= DMA_TX_APPEND_CRC | DMA_SOP | DMA_EOP;
    s->rdmaregs[(desc_base + DMA_DESC_LENGTH_STATUS) >> 2] = dma_len_stat;
#endif    

    qemu_hexdump((void *)buf, stderr, "", size);
    /* Update Consumer Index */
    cons_idx = (cons_idx + 1) & DMA_C_INDEX_MASK;
    s->rdmaregs[RDMA_CONS_INDEX_Q16 >> 2] = cons_idx;
    /* Update Producer Index */ 
    prod_idx = (prod_idx + 1) & DMA_P_INDEX_MASK;
    s->rdmaregs[RDMA_PROD_INDEX_Q16 >> 2] = prod_idx; 

    if (rx_cond == rx_match_mac) {
        
        ints = UMAC_IRQ_RXDMA_DONE | UMAC_IRQ_RXDMA_BDONE | UMAC_IRQ_RXDMA_PDONE;
//    if (sungem_rx_full(s, kick, done)) {
 //       ints |= GREG_STAT_RXNOBUF;
 //   }
//        bcmgenet_update_status(s, ints, true, 0);
        bcmgenet_update_status(s, UMAC_IRQ_RXDMA_DONE, true, 0);
        s->rdmaregs[(desc_base + DMA_DESC_LENGTH_STATUS) >> 2] = dma_len_stat;

    }     
    /* Ring full ? Can't receive */
    if (bcmgenet_rx_full(s, prod_idx, cons_idx)) {
        trace_bcmgenet_rx_ringfull();
        return 0;
    }

    /* Note: The real GEM will fetch descriptors in blocks of 4,
     * for now we handle them one at a time, I think the driver will
     * cope
     */
 

    /* Effective buffer address */
//    baddr = le64_to_cpu(desc.buffer) & ~7ull;
 //   baddr |= (rxdma_cfg & RXDMA_CFG_FBOFF) >> 10;


     dma_flags = (dma_len_stat >> DMA_RING_BUF_EN_SHIFT) & DMA_RING_BUF_EN_MASK;
    buf_len = (dma_len_stat >> DMA_BUFLENGTH_SHIFT) & DMA_BUFLENGTH_MASK;

    trace_bcmgenet_rx_desc(le32_to_cpu(dma_len_stat), dma_flags, le32_to_cpu(addr));

    if (fcs_size) {
        /* Should we add an FCS ? Linux doesn't ask us to strip it,
         * however I believe nothing checks it... For now we just
         * do nothing. It's faster this way.
         */
    }
    


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


    /* Write buffer out */
//        bcmgenet_write_bd(&bd, addr);
    //            prod_idx = (prod_idx + 1) & DMA_P_INDEX_MASK;
    //    s->rdmaregs[RDMA_PROD_INDEX >> 2] = prod_idx; 

    /* XXX Unconditionally set RX interrupt for now. The interrupt
     * mitigation timer might well end up adding more overhead than
     * helping here...
     */
   ints = UMAC_IRQ_RXDMA_MBDONE | UMAC_IRQ_RXDMA_PDONE | UMAC_IRQ_RXDMA_BDONE;
    bcmgenet_update_status(s, ints, true, 0);
    return size;
}

static void bcmgenet_reset(DeviceState *d)
{
    BCMGENETState *s = BCMGENET(d);
printf("******************************RESET******************************\n");
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

#endif
    if (qemu_get_queue(s->nic)->link_down) {
        s->phy_status &= ~(MII_BMSR_LINK_ST | MII_BMSR_AN_COMP);
//        s->phy_int |= PHY_INT_DOWN;
   	bcmgenet_update_status(s, UMAC_IRQ_LINK_DOWN, true, 0);
    }
    s->umacregs[UMAC_CMD >> 2] = 0x10000d8;
    s->rbufregs[RBUF_CTRL>> 2] = 0xc040;
    s->extregs[EXT_RGMII_OOB_CTRL >> 2] = 0xf00000;
    s->intrl2_0regs[INTRL2_CPU_STAT >> 2] = /*0x800000; */0x24;  //0
    s->intrl2_0regs[INTRL2_CPU_MASK_STATUS >> 2] = /*0x67fffff; */0x7ffffff;  //3
        s->intrl2_0regs[INTRL2_CPU_CLEAR >> 2] = 0x0; //2
                s->intrl2_0regs[INTRL2_CPU_MASK_CLEAR >> 2] = 0x00; //2
    s->intrl2_0regs[(INTRL2_CPU_CLEAR >> 2) + 0x04] = 0x24; //2
    s->intrl2_0regs[(INTRL2_CPU_SET >> 2) + 0x08] = 0x7ffffff;    //1
        s->intrl2_1regs[INTRL2_CPU_MASK_STATUS >> 2] = 0xffffffff;
    s->intrl2_1regs[(INTRL2_CPU_SET >> 2) + 0x08] = 0xffffffff;    //1        
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
        if (val & RGMII_LINK)
           bcmgenet_update_status(s, UMAC_IRQ_LINK_UP, true, 0);
        break;     	
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

    if (!(addr <= 0x40)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "Read from unknown INTRL2_0 register 0x%"HWADDR_PRIx"\n",
                      addr);
        return 0;
    }

    val = s->intrl2_0regs[addr >> 2];
    switch (addr) {
    case INTRL2_CPU_STAT:
  //       bcmgenet_eval_irq(s, 0);
 //        val |= 0x800000;//val & ~UMAC_IRQ_TXDMA_DONE;
    break;
    case INTRL2_CPU_MASK_STATUS:
   //     bcmgenet_eval_irq(s,0);
    break;	    
    }
    trace_bcmgenet_mmio_intrl2_0_read(GENET_INTRL2_0_OFF + addr, val);

    return val;
}

static void bcmgenet_mmio_intrl2_0_write(void *opaque, hwaddr addr, uint64_t val,
                                  unsigned size)
{
    BCMGENETState *s = opaque;
//    uint32_t bits;
    if (!(addr <= 0x40)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "Write to unknown INTRL2_0 register 0x%"HWADDR_PRIx"\n",
                      addr);
        return;
    }

    trace_bcmgenet_mmio_intrl2_0_write(GENET_INTRL2_0_OFF + addr, val);

    /* Pre-write filter */
    switch (addr) {
    /* Read only registers */
    case INTRL2_CPU_CLEAR:
//    printf("*****val 0x%lx\n",val);
            bcmgenet_update_status(s, val, false, 0);

  //  break;	
 
        return;
    case INTRL2_CPU_MASK_SET:
//        printf("*****val 0x%lx\n",val);
                bcmgenet_update_mask_status(s, val, true, 0);
        return;
	
    case INTRL2_CPU_MASK_CLEAR:
                bcmgenet_update_mask_status(s, val, false, 0);
        return;
        	        
    }

    s->intrl2_0regs[addr >> 2] = val;

    /* Post write action */
    switch (addr) {
    case INTRL2_CPU_STAT:
    break;			
    case INTRL2_CPU_SET:
    break;			
	
    case INTRL2_CPU_MASK_STATUS:
        bcmgenet_eval_irq(s,0);
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

    if (!(addr <= 0x100)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "Read from unknown INTRL2_1 register 0x%"HWADDR_PRIx"\n",
                      addr);
        return 0;
    }

    val = s->intrl2_1regs[addr >> 2];
    switch (addr) {
    case INTRL2_CPU_STAT:
   //      bcmgenet_eval_irq(s, 1);
     //    val = val & ~UMAC_IRQ_TXDMA_DONE;
    break;
    case INTRL2_CPU_MASK_STATUS:
    //    bcmgenet_eval_irq(s,1);
    break;	    
    }    

    trace_bcmgenet_mmio_intrl2_1_read(GENET_INTRL2_1_OFF + addr, val);

    return val;
}

static void bcmgenet_mmio_intrl2_1_write(void *opaque, hwaddr addr, uint64_t val,
                                  unsigned size)
{
    BCMGENETState *s = opaque;

    if (!(addr <= 0x100)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "Write to unknown INTRL2_1 register 0x%"HWADDR_PRIx"\n",
                      addr);
        return;
    }

    trace_bcmgenet_mmio_intrl2_1_write(GENET_INTRL2_1_OFF + addr, val);

    /* Pre-write filter */
    switch (addr) {
    /* Read only registers */
    case INTRL2_CPU_CLEAR:
            bcmgenet_update_status(s, val, false, 1);


        return;
    case INTRL2_CPU_MASK_SET:
                bcmgenet_update_mask_status(s, val, true, 1);
        return;
	
    case INTRL2_CPU_MASK_CLEAR:
                bcmgenet_update_mask_status(s, val, false, 1);
        return;        	        	
    }

    s->intrl2_1regs[addr >> 2] = val;

    /* Post write action */
    switch (addr) {
    case INTRL2_CPU_STAT:	
        break;		
    case INTRL2_CPU_SET:	
        break;		
	
    case INTRL2_CPU_MASK_STATUS:
        bcmgenet_eval_irq(s,1);	
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
    int ring_idx;
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
    case TDMA_CONS_INDEX_Q0:
    case TDMA_CONS_INDEX_Q1:
    case TDMA_CONS_INDEX_Q2:
    case TDMA_CONS_INDEX_Q3:
    case TDMA_CONS_INDEX_Q16:                
         if (s->tdmaregs[TDMA_CONS_INDEX_Q0 >> 2] > 0 ||  
             s->tdmaregs[TDMA_CONS_INDEX_Q1 >> 2] > 0 || 
             s->tdmaregs[TDMA_CONS_INDEX_Q2 >> 2] > 0 || 
             s->tdmaregs[TDMA_CONS_INDEX_Q3 >> 2] > 0 || 
             s->tdmaregs[TDMA_CONS_INDEX_Q16 >> 2] > 0)     
           return; /* No actual write */
    }

    s->tdmaregs[addr >> 2] = val;

    /* Post write action */
    switch (addr) { 
    case TDMA_CONS_INDEX_Q0:
    case TDMA_CONS_INDEX_Q1:
    case TDMA_CONS_INDEX_Q2:
    case TDMA_CONS_INDEX_Q3:
    case TDMA_CONS_INDEX_Q16:  
        break;
    case TDMA_PROD_INDEX_Q0:
    case TDMA_PROD_INDEX_Q1:
    case TDMA_PROD_INDEX_Q2:
    case TDMA_PROD_INDEX_Q3:
    case TDMA_PROD_INDEX_Q16:
         /* TODO: high priority tx ring */  

         ring_idx = (addr - GENET_TDMA_REG_OFF) >> 6;
         printf("**********transmit******** ring_idx %d 0x%lx\n",ring_idx, val);
         bcmgenet_do_tx(s,ring_idx);
            //     cons_idx = (cons_idx + 1) & DMA_C_INDEX_MASK;
     //   s->tdmaregs[TDMA_CONS_INDEX >> 2] += 1;
        if (bcmgenet_can_receive(qemu_get_queue(s->nic))) {
            qemu_flush_queued_packets(qemu_get_queue(s->nic));
        }
         break;
	

    case TDMA_READ_PTR_Q0:
    case TDMA_READ_PTR_Q1:
    case TDMA_READ_PTR_Q2:
    case TDMA_READ_PTR_Q3:
    case TDMA_READ_PTR_Q16:
	break;         
    case TDMA_RING_BUF_SIZE_Q0:
    case TDMA_RING_BUF_SIZE_Q1:
    case TDMA_RING_BUF_SIZE_Q2:
    case TDMA_RING_BUF_SIZE_Q3:
    case TDMA_RING_BUF_SIZE_Q16:
        break;	
    case TDMA_START_ADDR_Q0:
    case TDMA_START_ADDR_Q1:
    case TDMA_START_ADDR_Q2:
    case TDMA_START_ADDR_Q3:
    case TDMA_START_ADDR_Q16:	
        break;	
    case TDMA_END_ADDR_Q0:
    case TDMA_END_ADDR_Q1:
    case TDMA_END_ADDR_Q2:
    case TDMA_END_ADDR_Q3:
    case TDMA_END_ADDR_Q16:	
        break;		
    case TDMA_MBUF_DONE_THRESH_Q0:
    case TDMA_MBUF_DONE_THRESH_Q1:
    case TDMA_MBUF_DONE_THRESH_Q2:
    case TDMA_MBUF_DONE_THRESH_Q3:
    case TDMA_MBUF_DONE_THRESH_Q16:	
        break;	
    case TDMA_FLOW_PERIOD_Q0:
    case TDMA_FLOW_PERIOD_Q1:
    case TDMA_FLOW_PERIOD_Q2:
    case TDMA_FLOW_PERIOD_Q3:
    case TDMA_FLOW_PERIOD_Q16:
        break;		
    case TDMA_WRITE_PTR_Q0:
    case TDMA_WRITE_PTR_Q1:
    case TDMA_WRITE_PTR_Q2:
    case TDMA_WRITE_PTR_Q3:
    case TDMA_WRITE_PTR_Q16:			
        break;
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
        qemu_log_mask(LOG_GUEST_ERROR,
                      "Write to unknown TDMA register 0x%"HWADDR_PRIx"\n",
                      addr);
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
    case RDMA_CONS_INDEX_Q0:
    case RDMA_CONS_INDEX_Q1:
    case RDMA_CONS_INDEX_Q2:
    case RDMA_CONS_INDEX_Q3:
    case RDMA_CONS_INDEX_Q16:  
        break;
    case RDMA_PROD_INDEX_Q0:
    case RDMA_PROD_INDEX_Q1:
    case RDMA_PROD_INDEX_Q2:
    case RDMA_PROD_INDEX_Q3:
    case RDMA_PROD_INDEX_Q16:
         break;
	

    case RDMA_READ_PTR_Q0:
    case RDMA_READ_PTR_Q1:
    case RDMA_READ_PTR_Q2:
    case RDMA_READ_PTR_Q3:
    case RDMA_READ_PTR_Q16:
	break;         
    case RDMA_RING_BUF_SIZE_Q0:
    case RDMA_RING_BUF_SIZE_Q1:
    case RDMA_RING_BUF_SIZE_Q2:
    case RDMA_RING_BUF_SIZE_Q3:
    case RDMA_RING_BUF_SIZE_Q16:
        break;	
    case RDMA_START_ADDR_Q0:
    case RDMA_START_ADDR_Q1:
    case RDMA_START_ADDR_Q2:
    case RDMA_START_ADDR_Q3:
    case RDMA_START_ADDR_Q16:	
        break;	
    case RDMA_END_ADDR_Q0:
    case RDMA_END_ADDR_Q1:
    case RDMA_END_ADDR_Q2:
    case RDMA_END_ADDR_Q3:
    case RDMA_END_ADDR_Q16:	
        break;		
    case RDMA_MBUF_DONE_THRESH_Q0:
    case RDMA_MBUF_DONE_THRESH_Q1:
    case RDMA_MBUF_DONE_THRESH_Q2:
    case RDMA_MBUF_DONE_THRESH_Q3:
    case RDMA_MBUF_DONE_THRESH_Q16:	
        break;	
    case RDMA_XON_XOFF_THRESH_Q0:
    case RDMA_XON_XOFF_THRESH_Q1:
    case RDMA_XON_XOFF_THRESH_Q2:
    case RDMA_XON_XOFF_THRESH_Q3:
    case RDMA_XON_XOFF_THRESH_Q16:
        break;		
    case RDMA_WRITE_PTR_Q0:
    case RDMA_WRITE_PTR_Q1:
    case RDMA_WRITE_PTR_Q2:
    case RDMA_WRITE_PTR_Q3:
    case RDMA_WRITE_PTR_Q16:			
        break;			

    case RDMA_RING_CFG:			
      
        		
    case RDMASTATUS:	             
    case RDMA_SCB_BURST_SIZE:	
    case RDMA_RING0_TIMEOUT:
    case RDMA_INDEX2RING_0:    

        break;
    case RDMA_CTRL:
        if ((s->rdmaregs[RDMA_CTRL >> 2] & DMA_EN) != 0 &&
            (s->umacregs[UMAC_CMD >> 2] & CMD_RX_EN) != 0) {	
            qemu_flush_queued_packets(qemu_get_queue(s->nic));
        }
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
    case UMAC_CMD:
////////////////////////
//  bcmgenet_update_status(s,UMAC_IRQ_UMAC,false,0);
        break;
    case UMAC_MDIO_CMD:				



        break;        
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
   //     if ((s->rdmaregs[RDMA_CTRL >> 2] & DMA_EN) != 0 &&
     //       (s->umacregs[UMAC_CMD >> 2] & CMD_RX_EN) != 0) {	
       //    qemu_flush_queued_packets(qemu_get_queue(s->nic));
       // }
        if (bcmgenet_can_receive(qemu_get_queue(s->nic))) {
            qemu_flush_queued_packets(qemu_get_queue(s->nic));
        }	

        break;		
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
    case UMAC_MDIO_CMD:				
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
        bcmgenet_set_irq(s,0);
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
    BCMGENETState *s = BCMGENET(obj); 
                             
  //  sysbus_init_child_obj(obj, "unimac_mdio", &s->iomem, sizeof(s->iomem),
    //                         TYPE_UNIMAC_MDIO);   
        bcmgenet_reset(DEVICE(s)); 
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
    .link_status_changed = bcmgenet_set_link_status,
};

static void bcmgenet_realize(DeviceState *dev, Error **errp)
{
    BCMGENETState *s = BCMGENET(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

 //   bcmgenet_reset(DEVICE(s));
    memory_region_init(&s->bcmgenet, OBJECT(s), "bcmgenet", GENET_MMIO_SIZE);

    memory_region_init_io(&s->iomem.sys, OBJECT(s), &bcmgenet_mmio_sys_ops, s,
                          "bmcgenet.sys", BCMGENET_MMIO_SYS_SIZE);
    memory_region_add_subregion(&s->bcmgenet, GENET_SYS_OFF, &s->iomem.sys);
//    sysbus_init_mmio(sbd, &s->sys);

    memory_region_init_io(&s->iomem.ext, OBJECT(s), &bcmgenet_mmio_ext_ops, s,
                          "bcmgenet.ext", BCMGENET_MMIO_EXT_SIZE);
    memory_region_add_subregion(&s->bcmgenet, GENET_EXT_OFF, &s->iomem.ext);
   
    memory_region_init_io(&s->iomem.intrl2_0, OBJECT(s), &bcmgenet_mmio_intrl2_0_ops, s,
                          "bcmgenet.intrl2_0", BCMGENET_MMIO_INTRL2_0_SIZE);
    memory_region_add_subregion(&s->bcmgenet, GENET_INTRL2_0_OFF, &s->iomem.intrl2_0);

    memory_region_init_io(&s->iomem.intrl2_1, OBJECT(s), &bcmgenet_mmio_intrl2_1_ops, s,
                          "bcmgenet.intrl2_1", BCMGENET_MMIO_INTRL2_1_SIZE);
    memory_region_add_subregion(&s->bcmgenet, GENET_INTRL2_1_OFF, &s->iomem.intrl2_1);  
   
    memory_region_init_io(&s->iomem.tdma, OBJECT(s), &bcmgenet_mmio_tdma_ops, s,
                          "bcmgenet.tdma", BCMGENET_MMIO_TDMA_SIZE);
    memory_region_add_subregion(&s->bcmgenet, GENET_TX_OFF, &s->iomem.tdma);

    memory_region_init_io(&s->iomem.rdma, OBJECT(s), &bcmgenet_mmio_rdma_ops, s,
                          "bcmgenet.rdma", BCMGENET_MMIO_RDMA_SIZE);
    memory_region_add_subregion(&s->bcmgenet, GENET_RX_OFF, &s->iomem.rdma);

    memory_region_init_io(&s->iomem.umac, OBJECT(s), &bcmgenet_mmio_umac_ops, s,
                          "bcmgenet.umac", BCMGENET_MMIO_UMAC_SIZE);
    memory_region_add_subregion(&s->bcmgenet, GENET_UMAC_OFF, &s->iomem.umac);
#if 1
    memory_region_init_io(&s->iomem.rbuf, OBJECT(s), &bcmgenet_mmio_rbuf_ops, s,
                          "bcmgenet.rbuf", BCMGENET_MMIO_RBUF_SIZE);
    memory_region_add_subregion(&s->bcmgenet, GENET_RBUF_OFF, &s->iomem.rbuf);

    memory_region_init_io(&s->iomem.tbuf, OBJECT(s), &bcmgenet_mmio_tbuf_ops, s,
                          "bcmgenet.tbuf", BCMGENET_MMIO_TBUF_SIZE);
    memory_region_add_subregion(&s->bcmgenet, GENET_TBUF_OFF, &s->iomem.tbuf);
    
    memory_region_init_io(&s->iomem.hfb, OBJECT(s), &bcmgenet_mmio_hfb_ops, s,
                          "bcmgenet.hfb", BCMGENET_MMIO_HFB_SIZE);
    memory_region_add_subregion(&s->bcmgenet, GENET_HFB_OFF, &s->iomem.hfb);    
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
    sysbus_init_irq(sbd, &s->intrl2_0);
    sysbus_init_irq(sbd, &s->intrl2_1);
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
