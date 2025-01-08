// SPDX-License-Identifier: GPL-2.0+

#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/clockchips.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/sched_clock.h>
#include <linux/screen_info.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/acpi.h>
#include <linux/reset.h>
#include <linux/pm.h>

#define TIMER_NAME "sky1_timer"
/*timer registers */
#define LDINIT_RW  0x0000 /*Contains 32bit initial value to be loaded to Timer*/
#define CAPT_CNT_0 0x0004 /*Contains 32bit to store the count value from Timer when captured */
#define CAPT_CNT_1 0x0008 /*Contains 32bit to store the count value from Timer when captured */
#define CAPT_CNT_2 0x000C /*Contains 32bit to store the count value from Timer when captured */
#define CAPT_CNT_3 0x0010 /*Contains 32bit to store the count value from Timer when captured */
#define CAPT_CNT_4 0x0014 /*Contains 32bit to store the count value from Timer when captured */
#define PULSE_CNT  0x0018 /*Reserved*/
#define FREE_CNT   0x001C /*Contains 32bit to store real-time Free-running count value*/
#define COMP       0x0020 /*Contains 32bit to store the compare value in Free-running mode*/
#define TCTL       0x0024 /*Control register for Timer*/
#define PWMH       0x0028 /*Contains PWM_HIGH_TIME value to be loaded to Timer*/
#define PWMPRD     0x002C /*Contains PWM_PERIOD value to be loaded to timer*/
#define INTSTAT    0x0030 /*Interrupt status register*/
#define INTCLR     0x0034 /*Clear interrupt status*/

/*timer function mode */
#define GENER_MODE    0x0 /*Generate mode (default)*/
#define CAP_MODE      0x1 /*Capture mode*/
#define FREERUN_MODE  0x2 /*Free-running mode*/
#define PWM_MODE      0x3 /*PWM mode*/
#define PULSE_MODE    0x4 /*Pulse monitor mode*/

/*TCTL register */
#define ENABLE        (1 << 17) /*Enable Timer (This bit should be set finally)*/
#define FREEENABLE    (1 << 15) /*Free-running compare enabled*/
#define TWID	      (1 << 12) /*Timer WIDTH 0:32bit 1:64bit */
#define UPENABLE      (0 << 7)  /*Count up mode */
#define INTENABLE     (1 << 6)  /*Interrupt enabled*/
#define LOADENABLE    (1 << 5)  /*load LDINIT value to Timer*/
#define RELOADENABLE  (1 << 4)  /*Reload generate value*/

/*timer width register*/
#define TWIDENABLE	   (1 << 12) /*64-bit timer enable*/

/*INTCLR register*/
#define FREE_INTCLEAR      (1 << 5) /*clear free-running mode interrupt status*/

/*INTSTATUS register*/
#define FREE_INTSTATUS	   (1 << 5) /*free-running mode interrupt status*/

/* OFFSET between odd timer and even timer*/
#define TIMER_OFFSET  0x1000

struct sky1_timer {
	void __iomem *base;
	int irq;
	struct clock_event_device ced;
	struct clk *pclk, *tclk;
	struct reset_control *func_reset;
};

static inline struct sky1_timer *to_sky1_timer(struct clock_event_device *ced)
{
	return container_of(ced, struct sky1_timer, ced);
}

static void sky1_gpt_irq_disable(struct sky1_timer *sky1tm)
{
	u32 tmp;

	tmp = readl_relaxed(sky1tm->base + TIMER_OFFSET + TCTL);
	writel_relaxed(tmp & ~INTENABLE, sky1tm->base + TIMER_OFFSET + TCTL);
}

static void sky1_gpt_irq_enable(struct sky1_timer *sky1tm)
{
	u32 tmp;

	tmp = readl_relaxed(sky1tm->base + TIMER_OFFSET + TCTL);
	writel_relaxed(tmp | INTENABLE, sky1tm->base + TIMER_OFFSET + TCTL);
}

static void sky1_gpt_irq_acknowledge(struct sky1_timer *sky1tm)
{
	writel_relaxed(FREE_INTCLEAR, sky1tm->base + TIMER_OFFSET + INTCLR);
}

static void sky1_gpt_setup_tctl(struct sky1_timer *sky1tm)
{
	u32 tctl_val;

	tctl_val = FREEENABLE | FREERUN_MODE | INTENABLE;
	writel_relaxed(tctl_val, sky1tm->base + TCTL);

	tctl_val = FREEENABLE | TWID | FREERUN_MODE | INTENABLE;
	writel_relaxed(tctl_val, sky1tm->base + TIMER_OFFSET + TCTL);

	/*
	 * Enable bit should be set no earlier than
	 * any other bits or registers
	 */
	tctl_val = ENABLE | FREEENABLE | FREERUN_MODE | INTENABLE;
	writel_relaxed(tctl_val, sky1tm->base + TCTL);

	tctl_val = ENABLE | FREEENABLE | TWID | FREERUN_MODE | INTENABLE;
	writel_relaxed(tctl_val, sky1tm->base + TIMER_OFFSET + TCTL);
}


static void __iomem *sched_clock_reg;

/*
 * To get the value from the Global Timer Counter register proceed as follows:
 * 1. Read the upper 32-bit timer counter register
 * 2. Read the lower 32-bit timer counter register
 * 3. Read the upper 32-bit timer counter register again. If the value is
 *  different to the 32-bit upper value read previously, go back to step 2.
 *  Otherwise the 64-bit timer counter value is correct.
 */
static u64 notrace _sky1_gt_counter_read(void)
{
	u64 counter;
	u32 lower;
	u32 upper, old_upper;

	upper = readl_relaxed(sched_clock_reg + TIMER_OFFSET);
	do {
		old_upper = upper;
		lower = readl_relaxed(sched_clock_reg);
		upper = readl_relaxed(sched_clock_reg + TIMER_OFFSET);
	} while (upper != old_upper);

	counter = upper;
	counter <<= 32;
	counter |= lower;
	return counter;
}

static u64 notrace sky1_read_sched_clock(void)
{
	return sched_clock_reg ? _sky1_gt_counter_read() : 0;
}

static u64 sky1_gt_clocksource_read(struct clocksource *cs)
{
	return _sky1_gt_counter_read();
}

static struct clocksource clocksource_gpt = {
	.name	= "sky1_counter",
	.rating	= 200,
	.read	= sky1_gt_clocksource_read,
	.mask	= CLOCKSOURCE_MASK(64),
	.flags	= CLOCK_SOURCE_IS_CONTINUOUS,
};

static int __init sky1_clocksource_init(struct sky1_timer *sky1tm)
{
	unsigned int c = clk_get_rate(sky1tm->tclk);
	void __iomem *reg = sky1tm->base + FREE_CNT;

	sched_clock_reg = reg;

	sched_clock_register(sky1_read_sched_clock, 64, c);

	return clocksource_register_hz(&clocksource_gpt, c);
}

/*
 * clock event
 * To ensure that updates to comparator value register do not set the
 * Interrupt Status Register proceed as follows:
 * 1. Clear the Comp Enable bit in the Timer Control Register.
 * 2. Write the lower 32-bit Comparator Value Register.
 * 3. Write the upper 32-bit Comparator Value Register.
 * 4. Set the Comp Enable bit and, if necessary, the IRQ enable bit.
 */
static int sky1_set_next_event(unsigned long evt, struct clock_event_device *ced)
{
	struct sky1_timer *sky1tm = to_sky1_timer(ced);
	u64 tcn;

	tcn = _sky1_gt_counter_read();
	tcn = tcn + evt;

	/* Set event time into far-far future */
	writel_relaxed(tcn >> 32, sky1tm->base + COMP + TIMER_OFFSET);
	writel_relaxed((u32)tcn, sky1tm->base + COMP);

	return tcn < _sky1_gt_counter_read()  ? -ETIME : 0;
}

static int sky1_shutdown(struct clock_event_device *ced)
{
	struct sky1_timer *sky1tm = to_sky1_timer(ced);
	u64 tcn;

	/* Disable interrupt */
	sky1_gpt_irq_disable(sky1tm);

	tcn = _sky1_gt_counter_read();

	/* Set event time into far-far future */
	writel_relaxed((tcn - 3) >> 32, sky1tm->base + COMP + TIMER_OFFSET);
	writel_relaxed((u32)(tcn - 3), sky1tm->base + COMP);

	/* Clear pending interrupt */
	sky1_gpt_irq_acknowledge(sky1tm);

	return 0;
}

static int sky1_tick_resume(struct clock_event_device *ced)
{
	struct sky1_timer *sky1tm = to_sky1_timer(ced);

	/* reset timer */
	if (!screen_info.lfb_linelength)
		reset_control_reset(sky1tm->func_reset);

	sky1_gpt_setup_tctl(sky1tm);

	sky1_shutdown(ced);

	return 0;
}

static int sky1_set_oneshot(struct clock_event_device *ced)
{
	struct sky1_timer *sky1tm = to_sky1_timer(ced);
	u64 tcn;

	/* Disable interrupt */
	sky1_gpt_irq_disable(sky1tm);

	if (!clockevent_state_oneshot(ced)) {
		tcn = _sky1_gt_counter_read();
		/* Set event time into far-far future */
		writel_relaxed((tcn - 3) >> 32, sky1tm->base + COMP + TIMER_OFFSET);
		writel_relaxed((u32)(tcn - 3), sky1tm->base + COMP);

		/* Clear pending interrupt */
		sky1_gpt_irq_acknowledge(sky1tm);
	}

#ifdef DEBUG
	printk(KERN_INFO "%s: changing mode\n", __func__);
#endif

	/*
	 * Do not put overhead of interrupt enable/disable into
	 * sky1_set_next_event(), the core has about 4 minutes
	 * to call sky1_set_next_event() or shutdown clock after
	 * mode switching
	 */
	sky1_gpt_irq_enable(sky1tm);
	return 0;
}

/*
 * IRQ handler for the timer
 */
static irqreturn_t sky1_timer_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *ced = dev_id;
	struct sky1_timer *sky1tm = to_sky1_timer(ced);

	sky1_gpt_irq_acknowledge(sky1tm);

	ced->event_handler(ced);

	return IRQ_HANDLED;
}

static int __init sky1_clockevent_init(struct sky1_timer *sky1tm)
{
	struct clock_event_device *ced = &sky1tm->ced;

	ced->name = TIMER_NAME;
	ced->features = CLOCK_EVT_FEAT_ONESHOT | CLOCK_EVT_FEAT_DYNIRQ;
	ced->set_next_event = sky1_set_next_event;
	ced->set_state_shutdown = sky1_shutdown;
	ced->set_state_oneshot = sky1_set_oneshot;
	ced->set_state_oneshot_stopped = sky1_shutdown;
	ced->tick_resume = sky1_tick_resume;
	ced->rating = 200;
	ced->cpumask = cpu_possible_mask;
	ced->irq = sky1tm->irq;
	clockevents_config_and_register(ced, clk_get_rate(sky1tm->tclk), 1, UINT_MAX);

	return request_irq(sky1tm->irq, sky1_timer_interrupt,
			   IRQF_TIMER, TIMER_NAME, ced);
}

static int sky1_timer_probe(struct platform_device *pdev)
{
	struct sky1_timer *sky1tm;
	static int initialized;
	int ret = 0;

	/* Support one instance only */
	if (initialized)
		return 0;

	sky1tm = devm_kzalloc(&pdev->dev, sizeof(*sky1tm), GFP_KERNEL);
	if (!sky1tm)
		return -ENOMEM;

	/* Acpi resource parse */
	sky1tm->base = devm_platform_ioremap_resource(pdev, 0);
	if (!sky1tm->base)
		return -ENXIO;

	sky1tm->irq = platform_get_irq(pdev, 0);
	if (sky1tm->irq <= 0)
		return -EINVAL;

	sky1tm->pclk = devm_clk_get(&pdev->dev, "fch_timer_apb_clk");
	if (IS_ERR(sky1tm->pclk))
		return -EINVAL;

	sky1tm->tclk = devm_clk_get(&pdev->dev, "fch_timer_func_clk");
	if (IS_ERR(sky1tm->tclk))
		return -EINVAL;

	clk_prepare_enable(sky1tm->pclk);
	clk_prepare_enable(sky1tm->tclk);

	sky1tm->func_reset = devm_reset_control_get_optional_shared(&pdev->dev, "func_reset");
	if (IS_ERR(sky1tm->func_reset)) {
		dev_err(&pdev->dev, "[%s:%d]get reset error\n", __func__, __LINE__);
		ret = PTR_ERR(sky1tm->func_reset);
		goto err_clk_dis;
	}

	/* reset timer */
	if (!screen_info.lfb_linelength) /* already init in uefi */
		reset_control_reset(sky1tm->func_reset);

	sky1_gpt_setup_tctl(sky1tm);

	platform_set_drvdata(pdev, sky1tm);

	/* init and register the timer to the framework */
	ret = sky1_clocksource_init(sky1tm);
	if (ret)
		return ret;

	sky1_clockevent_init(sky1tm);

	initialized = 1;

	dev_info(&pdev->dev, "sky1 gpt timer init done\n");

	return 0;

err_clk_dis:
	clk_disable_unprepare(sky1tm->pclk);
	clk_disable_unprepare(sky1tm->tclk);
	return ret;
}

static const struct of_device_id sky1_timer_match_table[] = {
	{ .compatible = "cix,sky1-gpt" },
	{ /* sentinel */ },
};

static const struct acpi_device_id sky1_timer_acpi_match[] = {
	{ "CIXH1007", },
	{ },
};
MODULE_DEVICE_TABLE(acpi, sky1_timer_acpi_match);

static struct platform_driver sky1_timer_driver = {
	.driver = {
		.name = TIMER_NAME,
		.of_match_table = sky1_timer_match_table,
		.acpi_match_table = ACPI_PTR(sky1_timer_acpi_match),
	},
	.probe = sky1_timer_probe,
};
module_platform_driver(sky1_timer_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Jerry Zhu <jerry.zhu@cixtech.com>");
