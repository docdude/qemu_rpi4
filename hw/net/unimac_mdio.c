/*
 * QEMU model of Broadcom UniMAC MDIO bus controller controller
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
//#include "net/net.h"
#include "net/eth.h"
#include "hw/irq.h"
#include "net/checksum.h"
#include "hw/net/mii.h"
#include "sysemu/sysemu.h"
//#include "hw/sysbus.h"
#include "trace.h"
//#include "hw/net/unimac_mdio.h"
#include "hw/net/bcmgenet.h"

#define TYPE_UNIMAC_MDIO "unimac-mdio"
#define UNIMAC_MDIO(obj) OBJECT_CHECK(UNIMACState, (obj), TYPE_UNIMAC_MDIO)

#define BIT(nr) (1UL << (nr))
#define DEBUG 0
/*
 * Broadcom UniMAC MDIO bus controller
 */


#define MDIO_CMD		0x00
#define  MDIO_START_BUSY	BIT(29)
#define  MDIO_READ_FAIL	BIT(28)
#define  MDIO_RD		BIT(27)
#define  MDIO_WR		BIT(26)
#define  MDIO_PMD_SHIFT		21
#define  MDIO_PMD_MASK		0x1F
#define  MDIO_REG_SHIFT		16
#define  MDIO_REG_MASK		0x1F
#define MDIO_CFG		0x04
#define  MDIO_C22		(1 << 0)
#define  MDIO_C45		0
#define  MDIO_CLK_DIV_SHIFT	4
#define  MDIO_CLK_DIV_MASK	0x3F
#define  MDIO_SUPP_PREAMBLE	(1 << 12)

#define UNIMAC_MDIO_PHYCR_OP(x)       ((x) & (MDIO_WR | \
                                             MDIO_RD))
#define UNIMAC_MDIO_PHYCR_DATA(x)     (x & 0xffff)
#define UNIMAC_MDIO_PHYCR_PHY(x)      (((x) >> MDIO_PMD_SHIFT) & MDIO_PMD_MASK)
#define UNIMAC_MDIO_PHYCR_REG(x)      (((x) >> MDIO_REG_SHIFT) & MDIO_REG_MASK)


/* Broadcom BCM54XX register definitions, common to most Broadcom PHYs */
#define MII_BCM54XX_ECR		0x10	/* BCM54xx extended control register */
#define MII_BCM54XX_ECR_IM	0x1000	/* Interrupt mask */
#define MII_BCM54XX_ECR_IF	0x0800	/* Interrupt force */
#define MII_BCM54XX_ESR		0x11	/* BCM54xx extended status register */
#define MII_BCM54XX_ESR_IS	0x1000	/* Interrupt status */
#define MII_BCM54XX_EXP_DATA	0x15	/* Expansion register data */
#define MII_BCM54XX_EXP_SEL	0x17	/* Expansion register select */
#define MII_BCM54XX_EXP_SEL_SSD	0x0e00	/* Secondary SerDes select */
#define MII_BCM54XX_EXP_SEL_ER	0x0f00	/* Expansion register select */
#define MII_BCM54XX_EXP_SEL_ETC	0x0d00	/* Expansion register spare + 2k mem */
/*
 * AUXILIARY CONTROL SHADOW ACCESS REGISTERS.  (PHY REG 0x18)
 */
#define MII_BCM54XX_AUX_CTL	0x18	/* Auxiliary control register */
#define MII_BCM54XX_AUX_CTL_ENCODE(val) (((val & 0x7) << 12)|(val & 0x7))
#define MII_BCM54XX_AUX_CTL_DECODE(val) ((val & 0x7) >> 12)//|(val & 0x7))
#define MII_BCM54XX_AUXCTL_SHDWSEL_AUXCTL	0x00
#define MII_BCM54XX_AUXCTL_ACTL_TX_6DB		0x0400
#define MII_BCM54XX_AUXCTL_ACTL_SMDSP_ENA	0x0800
#define MII_BCM54XX_AUXCTL_SHDWSEL_MISC			0x07
#define MII_BCM54XX_AUXCTL_SHDWSEL_MISC_WIRESPEED_EN	0x0010
#define MII_BCM54XX_AUXCTL_SHDWSEL_MISC_RGMII_SKEW_EN	0x0100
#define MII_BCM54XX_AUXCTL_MISC_FORCE_AMDIX		0x0200
#define MII_BCM54XX_AUXCTL_MISC_WREN			0x8000
#define MII_BCM54XX_AUXCTL_SHDWSEL_READ_SHIFT	12
#define MII_BCM54XX_AUXCTL_SHDWSEL_MASK	0x0007


#define MII_BCM54XX_AUX_STATUS  0x19    /* Auxiliary status */
#define MII_BCM54XX_AUX_STATUS_LINKMODE_MASK 0x0700
#define MII_BCM54XX_AUX_STATUS_LINKMODE_SHIFT 8
#define MII_BCM54XX_ISR		0x1a	/* BCM54xx interrupt status register */
#define MII_BCM54XX_IMR		0x1b	/* BCM54xx interrupt mask register */
#define MII_BCM54XX_INT_CRCERR	0x0001	/* CRC error */
#define MII_BCM54XX_INT_LINK	0x0002	/* Link status changed */
#define MII_BCM54XX_INT_SPEED	0x0004	/* Link speed change */
#define MII_BCM54XX_INT_DUPLEX	0x0008	/* Duplex mode changed */
#define MII_BCM54XX_INT_LRS	0x0010	/* Local receiver status changed */
#define MII_BCM54XX_INT_RRS	0x0020	/* Remote receiver status changed */
#define MII_BCM54XX_INT_SSERR	0x0040	/* Scrambler synchronization error */
#define MII_BCM54XX_INT_UHCD	0x0080	/* Unsupported HCD negotiated */
#define MII_BCM54XX_INT_NHCD	0x0100	/* No HCD */
#define MII_BCM54XX_INT_NHCDL	0x0200	/* No HCD link */
#define MII_BCM54XX_INT_ANPR	0x0400	/* Auto-negotiation page received */
#define MII_BCM54XX_INT_LC	0x0800	/* All counters below 128 */
#define MII_BCM54XX_INT_HC	0x1000	/* Counter above 32768 */
#define MII_BCM54XX_INT_MDIX	0x2000	/* MDIX status change */
#define MII_BCM54XX_INT_PSERR	0x4000	/* Pair swap error */
#define MII_BCM54XX_SHD	0x1c	/* 0x1c shadow registers */
#define MII_BCM54XX_SHD_WRITE	0x8000
//#define MII_BCM54XX_SHD_VAL(x)	((x & 0x1f) << 10)
#define MII_BCM54XX_SHD_DATA(x)	((x & 0x3ff) << 0)
#define MIIM_BCM54XX_SHD_REG(x)	(((x) >> 10) & 0x1f)
#define MII_BCM54213PE_PHYID1 0x600d  
#define MII_BCM54213PE_PHYID2 0x84a2
#define MII_BCM54XX_RDB_ADDR	0x1e
#define MII_BCM54XX_RDB_DATA	0x1f

/*
 * BCM5482: Shadow registers
 * Shadow values go into bits [14:10] of register 0x1c to select a shadow
 * register to access.
 */
#define BCM54810_SHD_CLK_CTL			0x3
#define BCM54810_SHD_CLK_CTL_GTXCLK_EN	BIT(9) 
/* 00100: Reserved control register 2 */
#define BCM54XX_SHD_SCR2		0x04
#define  BCM54XX_SHD_SCR2_WSPD_RTRY_DIS	0x100
#define  BCM54XX_SHD_SCR2_WSPD_RTRY_LMT_SHIFT	2
#define  BCM54XX_SHD_SCR2_WSPD_RTRY_LMT_OFFSET	2
#define  BCM54XX_SHD_SCR2_WSPD_RTRY_LMT_MASK	0x7

/* 00101: Spare Control Register 3 */
#define BCM54XX_SHD_SCR3		0x05
#define  BCM54XX_SHD_SCR3_DEF_CLK125	0x0001
#define  BCM54XX_SHD_SCR3_DLLAPD_DIS	0x0002
#define  BCM54XX_SHD_SCR3_TRDDAPD	0x0004

/* 01010: Auto Power-Down */
#define BCM54XX_SHD_APD		0x0a
#define  BCM_APD_CLR_MASK		0xFE9F /* clear bits 5, 6 & 8 */
#define  BCM54XX_SHD_APD_EN		0x0020
#define  BCM_NO_ANEG_APD_EN		0x0060 /* bits 5 & 6 */
#define  BCM_APD_SINGLELP_EN		0x0100 /* Bit 8 */
#define BCM54XX_SHD_RGMII_MODE	0x0b	/* 01011: RGMII Mode Selector */
#define BCM5482_SHD_LEDS1		0x0d	/* 01101: LED Selector 1 */
					/* LED3 / ~LINKSPD[2] selector */
#define BCM5482_SHD_LEDS1_LED3(src)	((src & 0xf) << 4)
					/* LED1 / ~LINKSPD[1] selector */
#define BCM5482_SHD_LEDS1_LED1(src)	((src & 0xf) << 0)
#define BCM5482_SHD_LEDS4             0x0e

#define BCM54XX_SHD_RGMII_MODE	0x0b	/* 01011: RGMII Mode Selector */

#define BCM5482_SHD_SSD		0x14	/* 10100: Secondary SerDes control */
#define BCM5482_SHD_SSD_LEDM		0x0008	/* SSD LED Mode enable */
#define BCM5482_SHD_SSD_EN		0x0001	/* SSD enable */
 // 0x18 Autodetect SGMII/Media Converter Register (Address 1Ch, Shadow Value 11000) 
 // 0x1a 1000BASE-X Auto-Negotiation Debug Register (Address 1Ch, Shadow Value 11010)
 // 0x1b  Auxiliary 1000BASE-X Control Register (Address 1Ch, Shadow Value 11011)
#define BCM5482_SHD_MODE		0x1f	/* 11111: Mode Control Register */
#define BCM5482_SHD_MODE_1000BX	0x0001	/* Enable 1000BASE-X registers */
static void phy_update_irq(BCMGENETState *s)
{
    qemu_set_irq(s->intrl2_0, s->isr & s->ier);
        qemu_set_irq(s->intrl2_1, s->isr & s->ier);
}

void phy_reset(BCMGENETState *s)
{
    s->phy_status = (MII_BMSR_100TX_FD | MII_BMSR_100TX_HD | MII_BMSR_10T_FD |
                     MII_BMSR_10T_HD | MII_BMSR_EXTSTAT | MII_BMSR_MFPS |
                     MII_BMSR_AN_COMP | MII_BMSR_AUTONEG | MII_BMSR_LINK_ST |
                     MII_BMSR_EXTCAP);
    s->phy_control = (MII_BMCR_AUTOEN | MII_BMCR_FD | MII_BMCR_SPEED1000);
    s->phy_advertise = (MII_ANAR_PAUSE_ASYM | MII_ANAR_PAUSE | MII_ANAR_TXFD |
                        MII_ANAR_TX | MII_ANAR_10FD | MII_ANAR_10 |
                        MII_ANAR_CSMACD);
    s->phy_annp = 0x2001;
    s->phy_anlprnp = MII_ANLPAR_ACK;
    s->phy_ctrl1000 = (MII_CTRL1000_HALF | MII_CTRL1000_FULL);
    s->phy_extstat = 0x3000; //1000BASE-T full-duplex capable DUPLEX CAPABLE L  | 1000BASE-T half-duplex capable DUPLEX CAPABLE
    s->phy_ecr = 0x02;
    s->phy_int = 0;
    s->phy_shdw = 0x00;
    s->phy_shdwregs[BCM54XX_SHD_APD] = 0x0001;   /* 0001 */
    s->phy_shdwregs[BCM54XX_SHD_SCR2] = 0x000c;  /* 01100 */
    s->phy_shdwregs[BCM54XX_SHD_SCR3] = 0x001f;  /* 11111 */
    s->phy_shdwregs[0x08] = 0x0000;   /* 1000 */
    s->phy_shdwregs[0x09] = 0x0008;   /* 1000 */
    s->phy_shdwregs[BCM5482_SHD_LEDS1] = 0x0010; /* 10000 */
    s->phy_aux_ctl_shdwsel[0x00] = 0x00400;
    s->phy_aux_ctl_shdwsel[0x01] = 0x00000;
    s->phy_aux_ctl_shdwsel[0x02] = 0x00500;
    s->phy_aux_ctl_shdwsel[0x04] = 0x00000;
    s->phy_aux_ctl_shdwsel[0x05] = 0x00000;    
    s->phy_aux_ctl_shdwsel[0x07] = 0x00060;
    s->phy_exp_data[0x42] = 0x4006;
} 

static uint16_t do_phy_shdw_write(BCMGENETState *s, uint16_t val)
{
    uint32_t data;
    trace_unimac_phy_shdw_write( val);
    printf("*******MMD_CTL_DEVAD %x\n", MIIM_BCM54XX_SHD_REG(val));
//    s->phy_shdwregs[MIIM_BCM54XX_SHD_REG(val)] = val;
    switch (MIIM_BCM54XX_SHD_REG(val)) {
         case 0x08:  //  R/O   move   LED Status Register (Address 1Ch, Shadow Value 01000)
         break; // return val;
    }    
    switch (MIIM_BCM54XX_SHD_REG(val)) {

         case BCM54810_SHD_CLK_CTL_GTXCLK_EN:
         case BCM54XX_SHD_SCR2:
         case 0x09:   //  LED Control Register (Address 1Ch, Shadow Value 01001) 
         case BCM54XX_SHD_RGMII_MODE: //0x0b
         case 0x0c: //  LED GPIO Control/Status Register (Address 1Ch, Shadow Value 01111) 
         case BCM5482_SHD_LEDS1: //0x0d LED Selector 1 Register (Address 1Ch, Shadow Value 01101)
         case BCM5482_SHD_LEDS4: //0x0e LED Selector 2 Register (Address 1Ch, Shadow Value 01110)
         case BCM54XX_SHD_SCR3:
         case BCM54XX_SHD_APD:
              break;
         case 0x01:
         case 0x02:              
         case 0x06:
         case 0x07:
         case 0x0f:  
         case BCM5482_SHD_SSD:  //0x14
         case BCM5482_SHD_MODE:     // 0x1f             

             qemu_log_mask(LOG_UNIMP, "%s: reg %d not implemented\n",
                   __func__, val);
             break;
        default:
            qemu_log_mask(LOG_GUEST_ERROR, "%s: Bad address at offset %d\n",
                          __func__, val);
            break;             
     }
     
     data = val;
     if (val >> 15) {
       s->phy_shdwregs[MIIM_BCM54XX_SHD_REG(val)] |= data;
     } else {
       val = s->phy_shdwregs[MIIM_BCM54XX_SHD_REG(val)];
     }
         
    return (val & ~(1 << 15));  
}          

static uint16_t do_phy_read(BCMGENETState *s, uint8_t phy_addr,
                                  uint8_t reg)
{

    uint16_t val=0;
    
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
        val = MII_BCM54213PE_PHYID1;
        break;
    case MII_PHYID2: /* ID2 */
        val = MII_BCM54213PE_PHYID2;
        break;
    case MII_ANAR: /* Auto-neg advertisement */
        val = s->phy_advertise;
        break;
    case MII_ANLPAR: /* Auto-neg Link Partner Ability */
        val = (MII_ANLPAR_NEXT_PAGE | MII_ANLPAR_ACK | MII_ANLPAR_PAUSE | MII_ANLPAR_TXFD |
               MII_ANLPAR_TX | MII_ANLPAR_10FD | MII_ANLPAR_10 |
               MII_ANLPAR_CSMACD);
        break;
    case MII_ANER: /* Auto-neg Expansion */
        val = MII_ANER_NWAY;
        break;
    case MII_ANNP:
        val = s->phy_annp;
        break;
    case MII_ANLPRNP:
        val = s->phy_anlprnp;
        break;    
    case MII_CTRL1000: /* 1000BASE-T control  */
        val = s->phy_ctrl1000;//(MII_CTRL1000_HALF | MII_CTRL1000_FULL);
        break;
    case MII_STAT1000: /* 1000BASE-T status  */
        val = MII_STAT1000_FULL;
        break;
    case MII_EXTSTAT:
        val = s->phy_extstat;
        break;    
    case MII_BCM54XX_ESR:  /* PHY Extended Status Register (PHY_Addr = 0x1, Reg_Addr = 11h)  */
        val = s->phy_esr;
       // s->phy_int = 0;

        break;
    case MII_BCM54XX_ECR:  /* PHY Extended Control Register (PHY_Addr = 0x1, Reg_Addr = 10h) */
        val = s->phy_ecr;
        break;
    case MII_BCM54XX_SHD:
    
        val = s->phy_shdw;
        break;
    case MII_BCM54XX_EXP_DATA: 
        val = s->phy_exp_data[s->phy_exp_selregs]; //?? 
        break;
    case MII_BCM54XX_AUX_STATUS:
        val = 0xff1f;
        break;
    

    case MII_BCM54XX_ISR:
         
         if ((s->phy_isr & 0xffff) && (s->phy_ecr >> 12))  phy_update_irq(s);
         val = s->phy_isr;
         break;
    case MII_BCM54XX_IMR:
         val = s->phy_imr;
         break;
    case MII_BCM54XX_EXP_SEL:
         val = s->phy_exp_sel;
         break;
    case MII_BCM54XX_AUX_CTL:
         val = s->phy_aux_ctl;
         break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: Bad address at offset 0x%x val 0x%x\n",
                      __func__, reg, val);
        val = 0;
        break;
    }
    trace_unimac_phy_read(phy_addr, reg, val);   
    return val;
}



#define MII_BMCR_MASK (MII_BMCR_LOOPBACK | MII_BMCR_SPEED100 | MII_BMCR_SPEED1000  |       \
                       MII_BMCR_SPEED | MII_BMCR_AUTOEN | MII_BMCR_PDOWN | \
                       MII_BMCR_FD | MII_BMCR_CTST)
#define MII_ANAR_MASK 0x2d7f

static void do_phy_write(BCMGENETState *s, uint8_t phy_addr, uint8_t reg, uint16_t val)
{

    if (phy_addr != s->phy_addr) {
        return;
    }
    trace_unimac_phy_write(phy_addr, reg, val);
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
    case MII_ANNP:
        s->phy_annp = val;
        break;
    case MII_ANLPRNP:
        s->phy_anlprnp = val;
        break;                  
    case MII_CTRL1000: /* 1000BASE-T control  */
        s->phy_ctrl1000 = val; //(MII_CTRL1000_HALF | MII_CTRL1000_FULL);
        break; 
            
    case MII_BCM54XX_ECR: /* PHY Extended Control Register (PHY_Addr = 0x1, Reg_Addr = 10h) */
        s->phy_ecr = val & 0xffff;

        break;
        
    case MII_BCM54XX_SHD:
        s->phy_shdw = do_phy_shdw_write(s, val);    
        break;    

    case MII_BCM54XX_ISR:  // Read-only
//        s->phy_isr |= val;

        break;
    case MII_BCM54XX_IMR:
        s->phy_imr |= val;
        s->phy_isr |= s->phy_imr;
        break;
    case MII_BCM54XX_EXP_SEL:
        switch (val & 0xf00) {
        case 0x00: /* 00h “Expansion Register 00h: Receive/Transmit Packet Counter” */
	case 0x01: /* 01h “Expansion Register 01h: Expansion Interrupt Status” */
	case 0x03: /* 03h “Expansion Register 03h: SerDes Control” */
	case 0x04: /* 04h “Expansion Register 04h: Multicolor LED Selector” */
	case 0x05: /* 05h “Expansion Register 05h: Multicolor LED Flash Rate Controls” */
	case 0x06: /* 06h “Expansion Register 06h: Multicolor LED Programmable Blink Controls” */
	case 0x10: /* 10h “Expansion Register 10h: Cable Diagnostic Controls” */
	case 0x11: /* 11h “Expansion Register 11h: Cable Diagnostic Results” */
	case 0x12: /* 12h “Expansion Register 12h: Cable Diagnostic Lengths Channels 1/2” */
	case 0x13: /*13h “Expansion Register 13h: Cable Diagnostic Lengths Channels 3/4” */
	case MII_BCM54XX_EXP_SEL_ETC: /* 0x0d00 Expansion register spare + 2k mem */
	case MII_BCM54XX_EXP_SEL_SSD: /* 0x0e00 Secondary SerDes select */

	case MII_BCM54XX_EXP_SEL_ER: /* 0x0f00 Expansion register select */
        	    s->phy_exp_selregs = val & 0xff; 
        break;
        } 
        break;
    case MII_BCM54XX_EXP_DATA: 
         s->phy_exp_data[s->phy_exp_selregs] = val;//val = 0x5d86; //?? 
        break;         
    case MII_BCM54XX_AUX_CTL:
        if (val >> 15) {  //WRITE ENABLE
           s->phy_aux_ctl_shdwsel[val & 0x07] |= val & 0xfff;
           s->phy_aux_ctl = s->phy_aux_ctl_shdwsel[val & 0x07];
        } else	if ((val & 0x07) == 0x07) {
      	   switch (MII_BCM54XX_AUX_CTL_DECODE(val)) {  // Select Shadow Read Register Selector
           case 0x00: /* 000 = Normal Operation */ 
 	   case 0x01: /* 001 = 10 BASE-T Register */ 
	   case 0x02: /* 010 = Power Control Register */ 
 	   case 0x03: /* 011 = Reserved. */
	   case 0x04: /* 100 = Misc Test Register 1 */
	   case 0x05: /* 101 = Misc Test Register 2. */
	   case 0x06: /* 110 = Reserved. */
	   case 0x07: /* 111 = Misc Control Register */
	       s->phy_aux_ctl = s->phy_aux_ctl_shdwsel[MII_BCM54XX_AUX_CTL_DECODE(val)];
       
         
           break;
           default:
               qemu_log_mask(LOG_GUEST_ERROR, "%s: Bad shadow register at offset 0x%x val 0x%x\n",
                      __func__, reg, MII_BCM54XX_AUX_CTL_DECODE(val));
           break;
           }
           
        }
        trace_unimac_phy_aux_ctl_write(phy_addr, reg, val);
           
        break;       
//        qemu_log_mask(LOG_UNIMP, "%s: reg 0x%x not implemented val 0x%x\n",
//                      __func__, reg, val);
//        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: Bad address at offset 0x%x val 0x%x\n",
                      __func__, reg, val);
        break;
    }
}
static void unimac_mdio_transition(UNIMACState *s, bool fire)
{
    if (fire) {
        s->phycr |= MDIO_START_BUSY;
    } else {
        s->phycr &= ~MDIO_START_BUSY;
    }
}

static void unimac_mdio_do_phy_ctl(UNIMACState *s)
{
    uint8_t phy_addr, reg;
    uint16_t data;

 //   if (!(s->phycr & MDIO_C22)) {
  //      unimac_mdio_transition(s, !MDIO_START_BUSY);
   //     qemu_log_mask(LOG_UNIMP, "%s: unsupported ST code\n", __func__);
   //     return;
   // }

    /* Nothing to do */
    if (!(s->phycr & MDIO_START_BUSY)) {
        return;
    }

    phy_addr = UNIMAC_MDIO_PHYCR_PHY(s->phycr);
    reg = UNIMAC_MDIO_PHYCR_REG(s->phycr);
    data = UNIMAC_MDIO_PHYCR_DATA(s->phycr);
#if DEBUG    
    printf("reg 0x%x data 0x%x phyaddr 0x%x\n",reg,data,phy_addr);
#endif
    switch (UNIMAC_MDIO_PHYCR_OP(s->phycr)) {
    case MDIO_WR:
//        printf("******write\n");
        do_phy_write(s->nic, phy_addr, reg, data);
        break;
    case MDIO_RD:
//                  printf("******read phydata %x\n",s->phydata);
 //       s->phydata = (s->phydata & ~0xffff) | do_phy_read(s->nic, phy_addr, reg);
 //                 printf("******read phydata %x\n",s->phydata);
         s->phydata = do_phy_read(s->nic, phy_addr, reg) & 0xffff;
        if (s->phydata == 0xffff) s->phydata |= MDIO_READ_FAIL;
        s->phycr |= s->phydata;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: invalid OP code %08x\n",
                      __func__, s->phycr);
        trace_unimac_mdio_invalid_op(UNIMAC_MDIO_PHYCR_OP(s->phycr));            
    }

 //   unimac_mdio_transition(s, !MDIO_START_BUSY);
}

static uint64_t unimac_mdio_read(void *opaque, hwaddr addr, unsigned size)
{
    UNIMACState *s = UNIMAC_MDIO(opaque);
    unimac_mdio_transition(s, !MDIO_START_BUSY);
//printf("mii read addr 0x%lx phycr 0x%x\n",addr,s->phycr);    
    switch (addr) {
    case 0x0:
        return s->phycr;
    case 0x4:
        return s->mdio_clk;
    default:
        g_assert_not_reached();
    }
 
}

static void unimac_mdio_write(void *opaque, hwaddr addr,
                             uint64_t value, unsigned size)
{
    UNIMACState *s = UNIMAC_MDIO(opaque);

#if DEBUG
printf("mii write addr 0x%lx value 0x%lx\n",addr,value);
#endif
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

 //   unimac_mdio_transition(s, !!(s->phycr & MDIO_START_BUSY));
    unimac_mdio_do_phy_ctl(s);  
}

static const MemoryRegionOps unimac_mdio_ops = {
    .read = unimac_mdio_read,
    .write = unimac_mdio_write,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void unimac_mdio_reset(DeviceState *dev)
{
    UNIMACState *s = UNIMAC_MDIO(dev);

    s->phycr = 0;
    s->phydata = 0;

//    unimac_mdio_transition(s, !!(s->phycr & ASPEED_MII_PHYCR_FIRE));
};

static void unimac_mdio_realize(DeviceState *dev, Error **errp)
{
    UNIMACState *s = UNIMAC_MDIO(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    assert(s->nic);

    memory_region_init_io(&s->iomem, OBJECT(dev), &unimac_mdio_ops, s,
                          TYPE_UNIMAC_MDIO, 0x9);
    sysbus_init_mmio(sbd, &s->iomem);
}

static const VMStateDescription vmstate_unimac_mdio = {
    .name = TYPE_UNIMAC_MDIO,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(phycr, UNIMACState),
        VMSTATE_UINT32(phydata, UNIMACState),
        VMSTATE_END_OF_LIST()
    }
};

static Property unimac_mdio_properties[] = {
    DEFINE_PROP_LINK("nic", UNIMACState, nic, TYPE_BCMGENET,
                     BCMGENETState *),
                  
    DEFINE_PROP_END_OF_LIST(),
};

static void unimac_mdio_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->vmsd = &vmstate_unimac_mdio;
    dc->reset = unimac_mdio_reset;
    dc->realize = unimac_mdio_realize;
    dc->desc = "UNIMAC MDIO controller";
    device_class_set_props(dc, unimac_mdio_properties);
}

static const TypeInfo unimac_mdio_info = {
    .name = TYPE_UNIMAC_MDIO,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(UNIMACState),
    .class_init = unimac_mdio_class_init,
};
static void unimac_mdio_register_types(void)
{
    type_register_static(&unimac_mdio_info);

}

type_init(unimac_mdio_register_types)
