/*
 * Copyright (c) 2021-2021, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __ARMCHINA_REGISTER_H__
#define __ARMCHINA_REGISTER_H__

#include <linux/module.h>
#include <linux/init.h>
#include <linux/io.h>
#include <asm/io.h>
#include <linux/ioctl.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/i2c-dev.h>

/****************ISP_REG_SRAM****************/
#define ISP_REG_BASE                    0x83c00000
#define ISP_REG_SRAM_END                0x83c0fb00
#define ISP_GDC_REG_BASE                0x83d00000
#define ISP_REG_SIZE                    (ISP_REG_SRAM_END - ISP_REG_BASE + 1)

/****************ISP_DDR********************/
//AE STATS
#define AE0_STATS_ADDR_0                0x48567000 //0x1000
#define AE0_STATS_ADDR_1                0x48568000

#define AE1_STATS_ADDR_0                0x48569000
#define AE1_STATS_ADDR_1                0x4856A000

#define AE2_STATS_ADDR_0                0x4856B000
#define AE2_STATS_ADDR_1                0x4856C000
#define AE2_STATS_ADDR_2                0x48575000 //NEW
#define AE2_STATS_ADDR_3                0x48576000 //NEW

#define AE3_STATS_ADDR_0                0x4856D000
#define AE3_STATS_ADDR_1                0x4856E000

#define AE4_STATS_ADDR_0                0x4856F000 //0x400
#define AE4_STATS_ADDR_1                0x4856F400


//AF STATS
#define AF_STATS_ADDR_0                 0x48574000 //0x288
#define AF_STATS_ADDR_1                 0x48574400
#define AF_STATS_ADDR_2                 0x48574800
#define AF_STATS_ADDR_3                 0x48574C00

//VOUT
#define VOUT0_BUFFER_ADDR_0             0x50000000
#define VOUT1_BUFFER_ADDR_0             0x54000000
#define VOUT2_BUFFER_ADDR_0             0x58000000
#define VOUT3_BUFFER_ADDR_0             0x5C000000
#define VOUT0_BUFFER_ADDR_1             0x70000000
#define VOUT1_BUFFER_ADDR_1             0x74000000
#define VOUT2_BUFFER_ADDR_1             0x78000000
#define VOUT3_BUFFER_ADDR_1             0x7C000000


#define CACHELINE_SIZE                  32  // A9 or not

#ifdef ARMCB_CAM_AHB
/* AHB DPHY Register base, size 64KB. */
#define AHB_DPHY0_REG_BASE               0x142A0000
#define AHB_DPHY0_REG_SIZE               0x10000
#define AHB_DPHY0_REG_END                (AHB_DPHY0_REG_BASE+AHB_DPHY0_REG_SIZE-1)

#define AHB_DPHY1_REG_BASE               0x14300000
#define AHB_DPHY1_REG_SIZE               0x10000
#define AHB_DPHY1_REG_END                (AHB_DPHY1_REG_BASE+AHB_DPHY1_REG_SIZE-1)


/* AHB CSI Register base, size 64KB. */
#define AHB_CSI0_REG_BASE                0x14280000
#define AHB_CSI0_REG_SIZE                0x10000
#define AHB_CSI0_REG_END                 (AHB_CSI0_REG_BASE+AHB_CSI0_REG_SIZE-1)

#define AHB_CSI1_REG_BASE                0x14290000
#define AHB_CSI1_REG_SIZE                0x10000
#define AHB_CSI1_REG_END                 (AHB_CSI1_REG_BASE+AHB_CSI1_REG_SIZE-1)

#define AHB_CSI2_REG_BASE                0x142E0000
#define AHB_CSI2_REG_SIZE                0x10000
#define AHB_CSI2_REG_END                 (AHB_CSI2_REG_BASE+AHB_CSI2_REG_SIZE-1)

#define AHB_CSI3_REG_BASE                0x142F0000
#define AHB_CSI3_REG_SIZE                0x10000
#define AHB_CSI3_REG_END                 (AHB_CSI3_REG_BASE+AHB_CSI3_REG_SIZE-1)

/* AHB CSIDMA Register base, size 64KB. */
#define AHB_CSIDMA0_REG_BASE                0x142B0000
#define AHB_CSIDMA0_REG_SIZE                0x10000
#define AHB_CSIDMA0_REG_END                 (AHB_CSIDMA0_REG_BASE+AHB_CSI0_REG_SIZE-1)

#define AHB_CSIDMA1_REG_BASE                0x142C0000
#define AHB_CSIDMA1_REG_SIZE                0x10000
#define AHB_CSIDMA1_REG_END                 (AHB_CSIDMA1_REG_BASE+AHB_CSI1_REG_SIZE-1)

#define AHB_CSIDMA2_REG_BASE                0x14310000
#define AHB_CSIDMA2_REG_SIZE                0x10000
#define AHB_CSIDMA2_REG_END                 (AHB_CSIDMA2_REG_BASE+AHB_CSI2_REG_SIZE-1)

#define AHB_CSIDMA3_REG_BASE                0x14320000
#define AHB_CSIDMA3_REG_SIZE                0x10000
#define AHB_CSIDMA3_REG_END                 (AHB_CSIDMA3_REG_BASE+AHB_CSI3_REG_SIZE-1)

/* AHB CSIDMA SEL Register base, size 64KB. */
#define AHB_CSIRCSU0_REG_BASE                0x14270000
#define AHB_CSIRCSU0_REG_SIZE                0x10000
#define AHB_CSIRCSU0_REG_END                 (AHB_CSIRCSU0_REG_BASE+AHB_CSIRCSU0_REG_SIZE-1)

#define AHB_CSIRCSU1_REG_BASE                0x142D0000
#define AHB_CSIRCSU1_REG_SIZE                0x10000
#define AHB_CSIRCSU1_REG_END                 (AHB_CSIRCSU1_REG_BASE+AHB_CSIRCSU1_REG_SIZE-1)

#define AHB_PMCTRL_RES_REG_BASE                0x16000404
#define AHB_PMCTRL_RES_REG_SIZE                0x1
#define AHB_PMCTRL_RES_REG_END                 (AHB_PMCTRL_RES_REG_BASE+AHB_PMCTRL_RES_REG_SIZE-1)

/***0x17000000-0x17002000(DPHY_POWER_BASE-CSIDMA_POWER_REG_END) is not an actual physical**
 **address,but an address used for transmitting CSI, CSIDMA, and PHY information.**********/
#define DPHY_POWER_BASE                        0x17000000
#define DPHY_POWER_SIZE                        0x10000
#define DPHY_POWER_END                         (DPHY_POWER_BASE + DPHY_POWER_SIZE - 1)

#define CSI_POWER_BASE                         0x17010000
#define CSI_POWER_SIZE                         0x10000
#define CSI_POWER_END                          (CSI_POWER_BASE + CSI_POWER_SIZE - 1)

#define CSIDMA_POWER_BASE                      0x17020000
#define CSIDMA_POWER_SIZE                      0x10000
#define CSIDMA_POWER_END                       (CSIDMA_POWER_BASE + CSIDMA_POWER_SIZE - 1)

#endif

//-- APB2 Register base, size 64KB.
#define APB2_REG_BASE                   0x83c40000
#define APB2_REG_SIZE                   0x70
#define APB2_REG_END                    (APB2_REG_BASE+APB2_REG_SIZE-1)

//-- VDMA Register base, size 1KB
#define VDMA_REG_BASE                   0x83c60000
#define VDMA_REG_SIZE                   0x400
#define VDMA_REG_END                    (VDMA_REG_BASE+VDMA_REG_SIZE-1)


/* Definitions for peripheral AXI_CDMA_0 */
#define AXI_REG_CDMA_BASE                0x8E200000
#define AXI_REG_CDMA_SIZE                0x10000
#define AXI_REG_CDMA_END                 0x8E20FFFF

// XC7Z020 DDR3 memory range.
#define MEM_START_7020                  0x00000000
#define MEM_END_7020                    0x3FFFFFFF  // 1GB

#define I5_3A_START_ADDR                0x1430
#define I5_VOUT0_START_ADDR             0x1634
#define I5_VOUT1_START_ADDR             0x1664
#define I5_VOUT2_START_ADDR             0x1864
#define I5_VOUT3_START_ADDR             0x1994

#define I7_VIN_LONG_START_ADDR_DAW0     0x380      //DAW0
#define I7_VIN_MID_START_ADDR_DAW1      0x3B0      //DAW1
#define I7_VIN_SHORT_START_ADDR_DAW2    0x3E0      //DAW2
#define I7_VIN_VSHORT_START_ADDR_DAW3   0x420      //DAW3

#define I7_VIN_LONG_START_ADDR_DAR0     0x800      //DAR0
#define I7_VIN_MID_START_ADDR_DAR1      0x830      //DAR1
#define I7_VIN_SHORT_START_ADDR_DAR2    0x860      //DAR2
#define I7_VIN_VSHORT_START_ADDR_DAR3   0x890      //DAR3

#define I7_3A_START_ADDR                0x5d0
#define I7_VOUT0_START_ADDR             0x600
#define I7_VOUT1_START_ADDR             0x630
#define I7_VOUT2_START_ADDR             0x660
#define I7_VOUT3_START_ADDR             0x690
#define I7_VOUT4_START_ADDR             0x6c0
#define I7_VOUT5_START_ADDR             0x6f0
#define I7_VOUT6_START_ADDR             0x720
#define I7_VOUT7_START_ADDR             0x750
#define I7_VOUT8_START_ADDR             0x780

u32 armcb_register_get_int32(u32 phy_addr);
void armcb_register_set_int32(u32 phy_addr, u32 value);

#endif
