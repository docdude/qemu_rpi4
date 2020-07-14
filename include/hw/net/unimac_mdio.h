#ifndef UNIMAC_H
#define UNIMAC_H
#include "hw/net/bcmgenet.h"
typedef struct UNIMACState {
    /*< private >*/
    SysBusDevice parent_obj;

    BCMGENETState *nic;

    MemoryRegion iomem;
    uint32_t phycr;
    uint32_t phydata;
    uint32_t phy_status;
    uint32_t phy_control;
    uint32_t phy_advertise;
    uint32_t phy_int;
    uint32_t phy_int_mask;
    BCMGENETState genet;
} UNIMACState;

#endif
