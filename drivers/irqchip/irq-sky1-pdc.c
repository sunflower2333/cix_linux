// SPDX-License-Identifier: GPL-2.0-only

#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/slab.h>
#include <linux/irqchip.h>
#include <linux/syscore_ops.h>

#define SKY1_SIP_PDC                     0xC2000009
#define SKY1_SIP_CONFIG_PDC_SET_WAKE     0x02

#define PDC_MAX_IRQS            1000

struct irq_domain *cix_domain;

struct pdcv1_irqchip_data {
	struct raw_spinlock	rlock;
	void __iomem		*pdc_base;
};

static int pdcv1_wakeup_source_save(void)
{
	return 0;
}

static void pdcv1_wakeup_source_restore(void)
{
	return;
}

static struct syscore_ops sky1_pdc_syscore_ops = {
	.suspend	= pdcv1_wakeup_source_save,
	.resume		= pdcv1_wakeup_source_restore,
};

static int sky1_pdc_irq_set_wake(struct irq_data *d, unsigned int on)
{
	struct pdcv1_irqchip_data *cd = d->chip_data;
	struct arm_smccc_res res;
	unsigned long flags;

	raw_spin_lock_irqsave(&cd->rlock, flags);
	arm_smccc_smc(SKY1_SIP_PDC, SKY1_SIP_CONFIG_PDC_SET_WAKE,
		d->hwirq, on, 0, 0, 0, 0, &res);
	raw_spin_unlock_irqrestore(&cd->rlock, flags);

	return 0;
}

static struct irq_chip pdcv1_irqchip_data_chip = {
	.name			= "PDCv1",
	.irq_eoi		= irq_chip_eoi_parent,
	.irq_mask		= irq_chip_mask_parent,
	.irq_unmask		= irq_chip_unmask_parent,
	.irq_set_wake		= sky1_pdc_irq_set_wake,
	.irq_retrigger		= irq_chip_retrigger_hierarchy,
	.irq_set_type		= irq_chip_set_type_parent,
#ifdef CONFIG_SMP
	.irq_set_affinity	= irq_chip_set_affinity_parent,
#endif
};

static int sky1_pdc_domain_translate(struct irq_domain *d,
				      struct irq_fwspec *fwspec,
				      unsigned long *hwirq,
				      unsigned int *type)
{
	if (is_of_node(fwspec->fwnode)) {
		if (fwspec->param_count != 3)
			return -EINVAL;

		/* No PPI should point to this domain */
		if (fwspec->param[0] != 0)
			return -EINVAL;

		*hwirq = fwspec->param[1];
		*type = fwspec->param[2];
		return 0;
	} else if (is_acpi_device_node(fwspec->fwnode)) {
		if(fwspec->param_count != 2)
			return -EINVAL;

		if (fwspec->param[0] < 32) {
			pr_err(FW_BUG "Illegal GSI%d translation request\n",
			       fwspec->param[0]);
			return -EINVAL;
		}
		/* In ACPI asl file, using GSI to configure interrupt resource. */
		*hwirq = fwspec->param[0] - 32;
		*type = fwspec->param[1];

		WARN_ON(*type == IRQ_TYPE_NONE);
		return 0;
	}

	return -EINVAL;
}

static int sky1_pdc_domain_alloc(struct irq_domain *domain,
				  unsigned int irq, unsigned int nr_irqs,
				  void *data)
{
	struct irq_fwspec *fwspec = data;
	struct irq_fwspec parent_fwspec;
	irq_hw_number_t hwirq;
	unsigned int type;
	int err;
	int i;

	err = sky1_pdc_domain_translate(domain, fwspec, &hwirq, &type);
	if (err)
		return err;

	if (hwirq >= PDC_MAX_IRQS)
		return -EINVAL;

	for (i = 0; i < nr_irqs; i++) {
		irq_domain_set_hwirq_and_chip(domain, irq + i, hwirq + i,
				&pdcv1_irqchip_data_chip, domain->host_data);
	}

	parent_fwspec = *fwspec;
	parent_fwspec.fwnode = domain->parent->fwnode;

	return irq_domain_alloc_irqs_parent(domain, irq, nr_irqs,
					    &parent_fwspec);
}

static const struct irq_domain_ops pdcv1_irqchip_data_domain_ops = {
	.translate	= sky1_pdc_domain_translate,
	.alloc		= sky1_pdc_domain_alloc,
	.free		= irq_domain_free_irqs_common,
};

static const struct of_device_id pdcv1_of_match[] = {
	{ .compatible = "cix,sky1-pdc",  .data = (const void *) 2 },
	{ /* END */ }
};

static int __init sky1_pdc_irqchip_init(struct device_node *node,
			       struct device_node *parent)
{
	struct irq_domain *parent_domain, *domain;
	struct pdcv1_irqchip_data *cd;
	const struct of_device_id *id;

	if (!parent) {
		pr_err("%pOF: no parent, giving up\n", node);
		return -ENODEV;
	}

	id = of_match_node(pdcv1_of_match, node);
	if (!id) {
		pr_err("%pOF: unknown compatibility string\n", node);
		return -ENODEV;
	}

	parent_domain = irq_find_host(parent);
	if (!parent_domain) {
		pr_err("%pOF: unable to get parent domain\n", node);
		return -ENXIO;
	}

	cd = kzalloc(sizeof(struct pdcv1_irqchip_data), GFP_KERNEL);
	if (!cd) {
		pr_err("%pOF: kzalloc failed!\n", node);
		return -ENOMEM;
	}

	raw_spin_lock_init(&cd->rlock);

	cd->pdc_base = of_iomap(node, 0);
	if (!cd->pdc_base) {
		pr_err("%pOF: unable to map pdc registers\n", node);
		kfree(cd);
		return -ENOMEM;
	}

	domain = irq_domain_add_hierarchy(parent_domain, 0, PDC_MAX_IRQS,
				node, &pdcv1_irqchip_data_domain_ops, cd);
	if (!domain) {
		iounmap(cd->pdc_base);
		kfree(cd);
		return -ENOMEM;
	}
	irq_set_default_host(domain);

	cix_domain = domain;

	register_syscore_ops(&sky1_pdc_syscore_ops);

	/*
	 * Clear the OF_POPULATED flag set in of_irq_init so that
	 * later the pdc power domain driver will not be skipped.
	 */
	of_node_clear_flag(node, OF_POPULATED);

	return 0;
}

#ifdef CONFIG_ACPI
static int __init sky1_acpi_pdc_irqchip_init(struct platform_device *pdev)
{
	struct irq_domain *domain;
	struct pdcv1_irqchip_data *cd;
	struct resource *res_dp = NULL;
	cd = kzalloc(sizeof(struct pdcv1_irqchip_data), GFP_KERNEL);
	if (!cd) {
		pr_err("%pACPI: kzalloc failed!\n", pdev);
		return -ENOMEM;
	}

	raw_spin_lock_init(&cd->rlock);
	res_dp = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	cd->pdc_base = devm_ioremap_resource(&pdev->dev, res_dp);
	if (!cd->pdc_base) {
		pr_err("%pACPI: unable to map pdc registers\n", pdev);
		kfree(cd);
		return -ENOMEM;
	}

	domain = acpi_irq_create_hierarchy(0, PDC_MAX_IRQS, pdev->dev.fwnode,
					   &pdcv1_irqchip_data_domain_ops, cd);

	if (!domain) {
		iounmap(cd->pdc_base);
		kfree(cd);
		return -ENOMEM;
	}

	irq_set_default_host(domain);
	cix_domain = domain;

	register_syscore_ops(&sky1_pdc_syscore_ops);

	return 0;
}

static const struct acpi_device_id pdcv1_acpi_match[] = {
	{ .id = "CIXHA019", .driver_data = 0 },
	{ /* END */ },
};

static int pdc_probe(struct platform_device *p_dev)
{
	return sky1_acpi_pdc_irqchip_init(p_dev);
}

static struct platform_driver pdc_platform_driver = {
	.probe = pdc_probe,
	.driver = { .name = "sky1-pdc",
		    .owner = THIS_MODULE,
		    .acpi_match_table = ACPI_PTR(pdcv1_acpi_match) },
};

static int __init sky1_pdc_init(void) {
	return platform_driver_register(&pdc_platform_driver);
}

core_initcall(sky1_pdc_init);
#endif

IRQCHIP_DECLARE(sky1_pdc, "cix,sky1-pdc", sky1_pdc_irqchip_init);

MODULE_AUTHOR("Copyright 2024 Cix Technology Group Co., Ltd.");
MODULE_DESCRIPTION("Cix Sky1 irq pdc driver");
MODULE_LICENSE("GPL v2");
