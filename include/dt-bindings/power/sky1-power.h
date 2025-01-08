/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright 2024 Cix Technology Group Co., Ltd.
 * All Rights Reserved.
 *
 * The following programs are the sole property of Copyright 2024 Cix Technology Group Co., Ltd.,
 * and contain its proprietary and confidential information.
 */

#ifndef __DT_BINDINGS_SKY1_POWER_H__
#define __DT_BINDINGS_SKY1_POWER_H__

/* The Rich OS need flow the macro */
#define SKY1_PD_AUDIO	0
#define SKY1_PD_PCIE_CTRL0	1
#define SKY1_PD_PCIE_DUMMY	2
#define SKY1_PD_PCIEHUB	3
#define SKY1_PD_MMHUB	4
#define SKY1_PD_MMHUB_SMMU	5
#define SKY1_PD_DPU0	6
#define SKY1_PD_DPU1	7
#define	SKY1_PD_DPU2	8
#define SKY1_PD_DPU3	9
#define SKY1_PD_DPU4	10
#define SKY1_PD_VPU_TOP	11
#define SKY1_PD_VPU_CORE0	12
#define SKY1_PD_VPU_CORE1	13
#define SKY1_PD_VPU_CORE2	14
#define SKY1_PD_VPU_CORE3	15
#define SKY1_PD_NPU_CORE0	16
#define SKY1_PD_NPU_CORE1	17
#define SKY1_PD_NPU_CORE2	18
#define SKY1_PD_NPU_TOP	19
#define SKY1_PD_ISP0	20
#define SKY1_PD_GPU	21
#define SKY1_PD_MAX	32

/* devfreq domain id */
#define GPU_CORE_DVFS_DOMAIN_ID	0
#define GPU_TOP_DVFS_DOMAIN_ID	1
#define CPU_L_DVFS_DOMAIN_ID	2
#define CPU_B0_DVFS_DOMAIN_ID	3
#define CPU_B1_DVFS_DOMAIN_ID	4
#define CPU_M0_DVFS_DOMAIN_ID	5
#define CPU_M1_DVFS_DOMAIN_ID	6
#define DSU_DVFS_DOMAIN_ID	7
#define NPU_DFS_DOMAIN_ID	8
#define VPU_DFS_DOMAIN_ID	9
#define CI700_DFS_DOMAIN_ID	10
#define NI700_DFS_DOMAIN_ID	11

#endif
