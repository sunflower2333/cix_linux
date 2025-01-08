/* SPDX-License-Identifier: GPL-2.0 */
/*
 * pci-sky1 - PCIe controller driver for CIX's sky1 SoCs
 *
 * Author: Hans Zhang <Hans.Zhang@cixtech.com>
 */

#ifndef _PCIE_SKY1_H
#define _PCIE_SKY1_H

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/pci.h>
#include <linux/pci-ecam.h>
#include <linux/pm_runtime.h>
#include <linux/delay.h>

#include "../../pci.h"
#include "pcie-cadence.h"

#define MAX_RC_NUM (5)

#define APP_OFFSET_STRAP_REG 0x300
#define APP_OFFSET_STATUS_REG 0x400

#define STRAP_REG0 0x00 ////
#define STRAP_REG1 0x04
#define STRAP_REG2 0x08
#define STRAP_REG3 0x0c
#define STRAP_REG4 0x10
#define STRAP_REG5 0x14
#define STRAP_REG6 0x18
#define STRAP_REG8 0x20 //
#define STRAP_REG9 0x24
#define STRAP_REG10 0x28
#define STRAP_REG14 0x38
#define STRAP_REG16 0x40 ////
#define STRAP_REG17 0x44
#define STRAP_REG18 0x48
#define STRAP_REG19 0x4c
#define STRAP_REG20 0x50
#define STRAP_REG21 0x54
#define STRAP_REG22 0x58
#define STRAP_REG24 0x60 //
#define STRAP_REG25 0x64
#define STRAP_REG26 0x68
#define STRAP_REG27 0x6c
#define STRAP_REG28 0x70
#define STRAP_REG29 0x74
#define STRAP_REG30 0x78
#define STRAP_REG32 0x80 //
#define STRAP_REG33 0x84
#define STRAP_REG34 0x88
#define STRAP_REG35 0x8c
#define STRAP_REG36 0x90
#define STRAP_REG37 0x94
#define STRAP_REG38 0x98
#define STRAP_REG40 0xa0 //
#define STRAP_REG41 0xa4
#define STRAP_REG42 0xa8
#define STRAP_REG46 0xb8

#define STATUS_REG0 0x00 //
#define STATUS_REG1 0x04
#define STATUS_REG2 0x08
#define STATUS_REG3 0x0c
#define STATUS_REG4 0x10
#define STATUS_REG5 0x14
#define STATUS_REG6 0x18
#define STATUS_REG7 0x1c
#define STATUS_REG8 0x20 //
#define STATUS_REG9 0x24
#define STATUS_REG10 0x28
#define STATUS_REG16 0x40 //
#define STATUS_REG24 0x60 //
#define STATUS_REG32 0x80 //
#define STATUS_REG40 0xa0 //
#define STATUS_REG41 0xa4
#define STATUS_REG42 0xa8

#define PCIE_LTSSM_STATUS_SHIFT 10
#define LTSSTTRAN_MSG_LEN 8

enum sky1_pcie_mode {
	PCI_MODE_RC,
	PCI_MODE_EP,
};

enum sky1_pcie_plat {
	PCIE_PLAT_FPGA,
	PCIE_PLAT_EMU,
	PCIE_PLAT_EVK,
	PCIE_PLAT_NIO,
};

enum sky1_pcie_id {
	PCIE_ID_x8,
	PCIE_ID_x4,
	PCIE_ID_x2,
	PCIE_ID_x1_1,
	PCIE_ID_x1_0,
};
#define ID_VALID(n) ((n >= PCIE_ID_x8) && (n <= PCIE_ID_x1_0))
#define ID_INVALID(n) ((n < PCIE_ID_x8) || (n > PCIE_ID_x1_0))

enum sky1_local_err {
	PCIE_LOCAL_ERR0,
	PCIE_LOCAL_ERR1,
};

struct sky1_pcie_ctrl_desc {
	u32 id;
	char name[15]; /* sky1-pcie-x8 */
	bool dualmode;
	u32 link_speed;
	u32 max_lanes;
	u32 sbsa;
};

struct sky1_pcie_data {
	enum sky1_pcie_mode mode;
	const struct sky1_pcie_ctrl_desc *desc;
};

struct sky1_phy {
	void __iomem *phy_reg_base;
	struct resource *phy_reg_res;
	struct clk *pcie_phy_apb_clk;
	struct clk *pcie_refclk_phy;
};

struct sky1_aer_stats {
	u64 rp_total_cor_errs;
	u64 rp_total_fatal_errs;
	u64 rp_total_nonfatal_errs;
	u64 ep_total_cor_errs;
	u64 ep_total_fatal_errs;
	u64 ep_total_nonfatal_errs;
};

struct sky1_pcie {
	const struct sky1_pcie_data *data;
	const struct sky1_pcie_ctrl_desc *desc;
	struct reset_control *rst;
	struct device *dev;
	struct cdns_pcie *cdns_pcie;
	struct cdns_pcie_rc *cdns_pcie_rc;
	struct cdns_pcie_ep *cdns_pcie_ep;
	struct resource *cfg_res;
	struct resource *msg_res;
	struct pci_config_window *cfg;
	void __iomem *rcsu_base;
	void __iomem *strap_base;
	void __iomem *status_base;
	void __iomem *reg_base;
	void __iomem *cfg_base;
	void __iomem *msg_base;
	void __iomem *mem_base;
	void __iomem *mem_high_base;
	unsigned int ecam_support_flag : 1;

	u32 id;
	u32 link_speed;
	u32 num_lanes;
	u32 mode;

	struct clk *pcie_axi_clk;
	struct clk *pcie_apb_clk;
	struct clk *pcie_pm_clk;
	struct clk *pcie_refclk_b;
	struct gpio_desc *reset;
	struct gpio_desc *wake;
	struct regulator *vsupply;
	struct regulator *epsupply;

	u32 plat;

#ifdef CONFIG_DEBUG_FS
	struct dentry *debugfs;
	struct pci_dev *rc_pdev;
	struct pci_dev *ep_pdev;
	bool aspm_not_support;
	u32 aspm_support:7;
	u32 aspm_capable:7;

	bool L0s_support;
	bool L1ss_support;
#endif

	u8 linkctrl_offset;
	u32 local_irq;
	spinlock_t pme_lock;
	struct work_struct pme_work;
	bool is_aer_uncor_panic;
	u32 aer_c_irq;
	u32 aer_f_irq;
	u32 aer_nf_irq;
	struct mutex aer_mutex;
	struct sky1_aer_stats aer_stats;
	bool aer_uncor_dump;

	struct phy *pcie_phy;
	struct workqueue_struct *wk;
	struct delayed_work wk_handler;
	bool is_probe;
};

enum esky1_lpwr {
	eLPWR_L0S,
	eLPWR_ASPM_L10,
	eLPWR_ASPM_L11,
	eLPWR_ASPM_L12,
	eLPWR_PCIPM_L10,
	eLPWR_PCIPM_L11,
	eLPWR_PCIPM_L12,
	eLPWR_MAX,
};

union ustrap_pin0 {
	struct {
		u32 strap_supported_preset : 11;
		u32 strap_core_clk_frequency_multiplier : 1;
		u32 strap_pcie_rate_max : 3;
		u32 strap_lane_count_in : 3;
		u32 strap_dc_max_eval_iteration : 7;
		u32 strap_bypass_remote_tx_eq : 1;
		u32 strap_bypass_phase23 : 1;
		u32 strap_ipreg_link_access_enable : 1;
		u32 strap_ari_enable : 1;
		u32 strap_debug_data_mux : 2;
		u32 reserved : 1;
	};
	u32 reg;
};

u32 sky1_pcie_get_bits32(void __iomem *addr, u32 bits_msk);
bool sky1_pcie_link_up(struct cdns_pcie *cdns_pcie);

#endif // _PCIE_SKY1_H
