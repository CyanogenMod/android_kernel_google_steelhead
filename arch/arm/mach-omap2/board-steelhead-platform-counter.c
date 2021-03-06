/*
 * Copyright (C) 2011 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/err.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/aah_localtime.h>
#include <asm-generic/uaccess.h>
#include <plat/dmtimer.h>

#include "board-steelhead.h"
#include "mux.h"

#define DM_TIMER_ID 8
#define DM_TIMER_CLK OMAP_TIMER_SRC_SYS_CLK
#define DM_TIMER_CAPTURE_PIN_NAME "dpm_emu16.dmtimer8_pwm_evt"

struct counter_state {
	u32 upper;
	u32 lower_last;
	spinlock_t lock;
};

static struct omap_dm_timer	*counter_timer;
static u32			counter_freq = 1;
static unsigned long		rollover_check_timer_period;
static struct timer_list	rollover_check_timer;

#define VCXO_PWM_TIMER_ID 10
#define VCXO_PWM_PIN_NAME "dpm_emu18.dmtimer10_pwm_evt"
#define VCXO_PWM_SAFE_MODE_PIN_NAME "dpm_emu18.safe_mode"
#define VCXO_PWM_CLK OMAP_TIMER_SRC_SYS_CLK

static struct omap_dm_timer	*vcxo_pwm_timer;
static spinlock_t		vcxo_lock;
static s16			vcxo_last_rate;

static struct counter_state counter_state = {
	.upper = 0,
	.lower_last = 0,
	.lock = __SPIN_LOCK_UNLOCKED(counter_state.lock),
};

#ifdef CONFIG_AAH_TIMESYNC_DEBUG
static DEFINE_SPINLOCK(timesync_event_handler_lock);
static void (*timesync_event_handler)(void *d, u64);
static void                    *timesync_event_handler_data;

static struct counter_state tsdebug_counter_state = {
	.upper = 0,
	.lower_last = 0,
	.lock = __SPIN_LOCK_UNLOCKED(counter_state.lock),
};
#endif

static s64 get_counter_internal_l(struct counter_state *state, u32 lower)
{
	s64 ret;

	/* Check for rollover.  We have rolled over if bit 31 of the lower
	 * was set last time, but is clear this time.
	 */
	if (((lower ^ 0x80000000) & state->lower_last) & 0x80000000)
		++state->upper;

	state->lower_last = lower;

	ret = ((s64)state->upper << 32) | lower;

	return ret;
}

static u32 get_counter_lower(void)
{
	if (IS_ERR_OR_NULL(counter_timer))
		return 0;

	return (u32)omap_dm_timer_read_counter(counter_timer);
}

#ifdef CONFIG_AAH_TIMESYNC_DEBUG
static irqreturn_t timer_capture_irq(int irq, void *dev_id)
{
	struct omap_dm_timer *gpt = (struct omap_dm_timer *)dev_id;
	unsigned long irq_state;
	s64 event_time;
	u32 lower;

	/* Read the captured counter and ack the interrupt. */
	lower = omap_dm_timers_read_capture1(gpt);
	omap_dm_timer_write_status(gpt, OMAP_TIMER_INT_CAPTURE);

	spin_lock_irqsave(&counter_state.lock, irq_state);
	event_time = get_counter_internal_l(&tsdebug_counter_state, lower);
	spin_unlock_irqrestore(&counter_state.lock, irq_state);

	if (NULL != timesync_event_handler)
		timesync_event_handler(timesync_event_handler_data, event_time);

	return IRQ_HANDLED;
}
#endif

static void counter_rollover_check(unsigned long arg)
{
	/* Force a rollover check. */
	steelhead_get_raw_counter();

	rollover_check_timer.expires += rollover_check_timer_period;
	add_timer(&rollover_check_timer);
}

#define VCXO_CYCLE_TICKS 1024
#define VCXO_TIMER_START (0xFFFFFFFF - VCXO_CYCLE_TICKS)

/* Note: we do all of our VCXO timer setup using direct register reads and
 * writes.  We used to use the omap_dm_timer routines, but the code underneath
 * kept changing in ways which were breaking our use case.  For now, we go
 * directly to the registers. If/when the omap_dm_timer routines get fixed, we
 * can go back to using them. */

#define TIMER_ID_OFFSET				0x00
#define TIMER_OCP_CFG_OFFSET			0x10
#define TIMER_SYS_STAT_OFFSET			0x14
#define TIMER_STAT_OFFSET			0x18
#define TIMER_INT_EN_OFFSET			0x1c
#define TIMER_WAKEUP_EN_OFFSET			0x20
#define TIMER_CTRL_OFFSET			0x24
#define		TIMER_CTRL_GPOCFG		(1 << 14)
#define		TIMER_CTRL_CAPTMODE		(1 << 13)
#define		TIMER_CTRL_PT			(1 << 12)
#define		TIMER_CTRL_TRIG_NONE		(0x0 << 10)
#define		TIMER_CTRL_TRIG_OVFL		(0x1 << 10)
#define		TIMER_CTRL_TRIG_OVFL_MATCH	(0x2 << 10)
#define		TIMER_CTRL_TRIG_MASK		(0x3 << 10)
#define		TIMER_CTRL_TCM_LOWTOHIGH	(0x1 << 8)
#define		TIMER_CTRL_TCM_HIGHTOLOW	(0x2 << 8)
#define		TIMER_CTRL_TCM_BOTHEDGES	(0x3 << 8)
#define		TIMER_CTRL_SCPWM		(1 << 7)
#define		TIMER_CTRL_CE			(1 << 6) /* compare enable */
#define		TIMER_CTRL_PRE			(1 << 5) /* prescaler enable */
#define		TIMER_CTRL_PTV_SHIFT		2 /* prescaler value shift */
#define		TIMER_CTRL_POSTED		(1 << 2)
#define		TIMER_CTRL_AR			(1 << 1) /* auto-reload
							    enable */
#define		TIMER_CTRL_ST			(1 << 0) /* start timer */
#define TIMER_COUNTER_OFFSET			0x28
#define TIMER_LOAD_OFFSET			0x2c
#define TIMER_TRIGGER_OFFSET			0x30
#define TIMER_WRITE_PEND_OFFSET			0x34
#define TIMER_MATCH_OFFSET			0x38
#define TIMER_CAPTURE_OFFSET			0x3c
#define TIMER_IF_CTRL_OFFSET			0x40

static inline void timer_wait_no_write_pending(struct omap_dm_timer *t)
{
	if (t->posted) {
		int i;
		void *tgt = (void *)((u32)t->io_base + t->func_offset
				+ TIMER_WRITE_PEND_OFFSET);

		for (i = 0; (i < 100000) && (readl(tgt) & 0xff); ++i)
			;

		if (WARN_ON_ONCE(i == 100000))
			dev_err(&t->pdev->dev, "wp timeout.\n");
	}
}

static inline u32 timer_read_reg(struct omap_dm_timer *t, u32 reg)
{
	timer_wait_no_write_pending(t);

	if (reg >= TIMER_WAKEUP_EN_OFFSET)
		reg += t->func_offset;
	else if (reg >= TIMER_STAT_OFFSET)
		reg += t->intr_offset;

	return readl(t->io_base + reg);
}

static inline void timer_write_reg(struct omap_dm_timer *t, u32 reg, u32 val)
{
	timer_wait_no_write_pending(t);

	if (reg >= TIMER_WAKEUP_EN_OFFSET)
		reg += t->func_offset;
	else if (reg >= TIMER_STAT_OFFSET)
		reg += t->intr_offset;

	writel(val, t->io_base + reg);
}

static void steelhead_set_vcxo_rate(s16 rate)
{
	u32 match_pos, ctrl;
	unsigned long irq_state;

	/* If we never had a timer, then there is nothing to do. */
	if (!vcxo_pwm_timer)
		return;

	/* Start by computing where on our cycle range the match position should
	 * set.  Linearly map MIN_INT16 to always off and MAX_INT16 to always
	 * on. */
	match_pos   = (u32)(rate + 0x8000);
	match_pos  *= VCXO_CYCLE_TICKS;
	match_pos >>= 16;

	spin_lock_irqsave(&vcxo_lock, irq_state);

	/* Make sure the timer is stopped.  It is not safe to change the PWM's
	 * duty cycle while the timer is running. Remember to wait at least 3.5
	 * timer fClk cycles after stopping before touching anything else in the
	 * timer.  At 38.4 MHz, 3.5 cycles is ~91nSec, so we just wait a
	 * microsecond and call it good. */
	ctrl = timer_read_reg(vcxo_pwm_timer, TIMER_CTRL_OFFSET);
	ctrl &= ~TIMER_CTRL_ST;
	timer_write_reg(vcxo_pwm_timer, TIMER_CTRL_OFFSET, ctrl);
	udelay(1);

	if (!match_pos) {
		/* always off case.  Just set the config to indicate an output
		 * with a default value of 0 and get out without bothering to
		 * start the timer. ctrl holds the current value of CTRL, so we
		 * can just modify ctrl and write. */
		ctrl &= ~(TIMER_CTRL_GPOCFG | TIMER_CTRL_SCPWM);
		timer_write_reg(vcxo_pwm_timer, TIMER_CTRL_OFFSET, ctrl);
		goto finished;
	}

	match_pos += VCXO_TIMER_START;
	if (match_pos >= 0xFFFFFFFE) {
		/* always on case.  See above, s/0/1/ */
		ctrl &= ~TIMER_CTRL_GPOCFG;
		ctrl |= TIMER_CTRL_SCPWM;
		timer_write_reg(vcxo_pwm_timer, TIMER_CTRL_OFFSET, ctrl);
		goto finished;
	}

	/* Set the reload and match values for the timer. */
	timer_write_reg(vcxo_pwm_timer, TIMER_LOAD_OFFSET, VCXO_TIMER_START);
	timer_write_reg(vcxo_pwm_timer, TIMER_MATCH_OFFSET, match_pos);

	/* Position the counter just before overflow and then start the timer up
	 * in PWM mode with an initial output value of 0 and configured to
	 * toggle the output on both match and overflow.  This should avoid the
	 * missed match event (see section 22.2.4.10 of the OMAP44xx TRM) and
	 * cause the PWM to output high until the match event, and then low
	 * until the overflow event. */
	timer_write_reg(vcxo_pwm_timer, TIMER_COUNTER_OFFSET, 0xFFFFFFFD);

	ctrl &= ~(TIMER_CTRL_GPOCFG			/* pwm output */
			| TIMER_CTRL_SCPWM		/* initial out == 0 */
			| TIMER_CTRL_TRIG_MASK);	/* clear trig bits */
	ctrl |=  (TIMER_CTRL_CE				/* compare enabled */
			| TIMER_CTRL_AR			/* autoreload == 1 */
			| TIMER_CTRL_ST			/* start == 1 */
			| TIMER_CTRL_PT			/* pwm toggle mode */
			| TIMER_CTRL_TRIG_OVFL_MATCH);	/* toggle on both */
	timer_write_reg(vcxo_pwm_timer, TIMER_CTRL_OFFSET, ctrl);

finished:
	/* update our last rate, release our lock and get out */
	vcxo_last_rate = rate;
	spin_unlock_irqrestore(&vcxo_lock, irq_state);
}

static int __init steelhead_setup_vcxo_control(void)
{
	/* Set up our lock */
	spin_lock_init(&vcxo_lock);

	/* Attempt to grab our timer */
	vcxo_pwm_timer = omap_dm_timer_request_specific(VCXO_PWM_TIMER_ID);
	if (IS_ERR_OR_NULL(vcxo_pwm_timer)) {
		pr_err("Steelhead: failed to request DMTIMER%d, VCXO"
				"control will be unavailable.\n",
				VCXO_PWM_TIMER_ID);
		goto failed_to_fetch_timer;
	}

	/* set up the timer to source from the sysclk and make sure it is
	 * enabled so that power management does not shut it off. */
	omap_dm_timer_set_source(vcxo_pwm_timer, VCXO_PWM_CLK);
	omap_dm_timer_enable(vcxo_pwm_timer);

	/* set the timer up in PWM mode with a 50/50 duty cycle to begin with */
	steelhead_set_vcxo_rate(0);

	/* turn the pin on and we are done */
	omap_mux_init_signal(VCXO_PWM_PIN_NAME, OMAP_PIN_OUTPUT);

	return 0;

failed_to_fetch_timer:
	vcxo_pwm_timer = NULL;
	return -1;
}

/******************************************************************************
 *                                                                            *
 *                          API implementation                                *
 *                                                                            *
 ******************************************************************************/
s64 steelhead_get_raw_counter(void)
{
	unsigned long irq_state;
	s64 ret;

	spin_lock_irqsave(&counter_state.lock, irq_state);

	ret = get_counter_internal_l(&counter_state, get_counter_lower());

	spin_unlock_irqrestore(&counter_state.lock, irq_state);
	return ret;
}

u32 steelhead_get_raw_counter_nominal_freq(void)
{
	return counter_freq;
}

#ifdef CONFIG_AAH_TIMESYNC_DEBUG
void steelhead_register_timesync_event_handler(void *user_data,
					       void (*handler)(void *d, u64))
{
	unsigned long irq_state;

	spin_lock_irqsave(&timesync_event_handler_lock, irq_state);
	timesync_event_handler = handler;
	timesync_event_handler_data = user_data;
	spin_unlock_irqrestore(&timesync_event_handler_lock, irq_state);
}
#endif

static struct aah_localtime_platform_data localtime_pdata = {
	.get_raw_counter = steelhead_get_raw_counter,
	.get_raw_counter_nominal_freq = steelhead_get_raw_counter_nominal_freq,
	.set_counter_slew_rate = steelhead_set_vcxo_rate,
#ifdef CONFIG_AAH_TIMESYNC_DEBUG
	.register_timesync_event_handler =
			steelhead_register_timesync_event_handler,
#endif
};

static struct platform_device aah_localtime_device = {
	.name	= "aah_localtime",
	.id	= -1,
	.dev	= {
		.platform_data	= &localtime_pdata,
	},
};

#ifdef CONFIG_DEBUG_FS
/******************************************************************************
 *                                                                            *
 *              DebugFS support for VCXO Factory Diagnostics                  *
 *                                                                            *
 ******************************************************************************/
static struct dentry *vcxo_debugfs_dir;
static struct dentry *vcxo_debugfs_test_node;
static struct dentry *vcxo_debugfs_vcxo_value_node;

static const char *kVCXODebugDirName = "steelhead-vcxo";
static const char *kVCXOTestNodeName = "factory_test";
static const char *kVCXOValueNodeName = "value";

static DEFINE_SPINLOCK(vcxo_factory_test_spinlock);
static u32 vcxo_factory_test_32k_start_time;
static s64 vcxo_factory_test_38400k_start_time;
static int vcxo_factory_test_in_progress;

#define SYS32K_COUNTER_REG 0x4A304010
static u32 vcxo_debugfs_read_32k(void)
{
	/* Note: This register is being read with a 32 bit access even though the
	 * TRM (section 22.4.4.1, dated Jan 2011) says that the counter must be
	 * accessed 16 bits at a time.  I checked with TI and they verified via
	 * simulation that the documentation was out of date and that 32 bit
	 * access should be no problem.  Expect later versions of the TRM to
	 * have this detail corrected.
	 */
	return omap_readl(SYS32K_COUNTER_REG);
}

static int vcxo_debugfs_test_open(struct inode *node, struct file *f)
{
	f->private_data = NULL;
	return 0;
}

static ssize_t vcxo_debugfs_test_read(struct file *f,
			       char __user *user_buf,
			       size_t count,
			       loff_t *pos)
{
	ssize_t ret_val = 0;
	size_t amt;
	char buf[512];
	unsigned long irq_state;
	u32 delta_32k;
	s64 delta_38400k;
	int was_in_progress;

	if (NULL != f->private_data)
		goto finished;

	spin_lock_irqsave(&vcxo_factory_test_spinlock, irq_state);
	was_in_progress = vcxo_factory_test_in_progress;
	vcxo_factory_test_in_progress = 0;
	if (was_in_progress) {
		delta_32k = vcxo_debugfs_read_32k() -
			vcxo_factory_test_32k_start_time;
		delta_38400k = steelhead_get_raw_counter() -
			vcxo_factory_test_38400k_start_time;
	}
	spin_unlock_irqrestore(&vcxo_factory_test_spinlock, irq_state);

	if (was_in_progress) {
		s64 usec_delta_32k, usec_delta_38400k, ppm;

		/* usec_delta_32k = delta_32k * 1000000 / 32768
		 *                = delta_32k * 15625 / 512
		 */
		usec_delta_32k = ((s64)delta_32k * 15625ll);
		do_div(usec_delta_32k, 512);

		/* usec_delta_38400k = delta_38400k * 1000000 / 38400000
		 *                   = delta_38400k * 10 / 384
		 */
		usec_delta_38400k = ((s64)delta_38400k * 10ll);
		do_div(usec_delta_38400k, 384);

		ppm = usec_delta_38400k - usec_delta_32k;
		ppm *= 1000000;
		if (ppm < 0) {
			ppm = -ppm;
			do_div(ppm, (uint32_t)usec_delta_32k);
			ppm = -ppm;
		} else {
			do_div(ppm, (uint32_t)usec_delta_32k);
		}
		snprintf(buf, sizeof(buf),
				"VCXO Factory Test\n"
				"uSec on 32 KHz Clock   = %lld\n"
				"uSec on 38.4 MHz Clock = %lld\n"
				"ppm difference         = %lld\n",
				usec_delta_32k, usec_delta_38400k, ppm);
	} else {
		snprintf(buf, sizeof(buf),
				"VCXO Factory Test was not started.\n");
	}

	buf[sizeof(buf) - 1] = 0;

	amt = strlen(buf);
	if (amt > count)
		amt = count;

	if (copy_to_user(user_buf, buf, amt)) {
		ret_val = -EFAULT;
		goto finished;
	}

	ret_val = amt;

finished:
	/* Mark the fact that we have been through here before */
	f->private_data = (void *)(0x01);
	return ret_val;
}

static ssize_t vcxo_debugfs_test_write(struct file *f,
				const char __user *user_buf,
				size_t count,
				loff_t *pos)
{
	unsigned long irq_state;
	spin_lock_irqsave(&vcxo_factory_test_spinlock, irq_state);

	vcxo_factory_test_32k_start_time = vcxo_debugfs_read_32k();
	vcxo_factory_test_38400k_start_time = steelhead_get_raw_counter();
	vcxo_factory_test_in_progress = 1;

	spin_unlock_irqrestore(&vcxo_factory_test_spinlock, irq_state);
	return count;
}

static const struct file_operations vcxo_debugfs_test_fops = {
	.owner	= THIS_MODULE,
	.open	= vcxo_debugfs_test_open,
	.read	= vcxo_debugfs_test_read,
	.write	= vcxo_debugfs_test_write
};

static int vcxo_debugfs_get_vcxo_value(void *ctx, u64 *out)
{
	unsigned long irq_state;

	spin_lock_irqsave(&vcxo_lock, irq_state);
	*out = (u64)vcxo_last_rate;
	spin_unlock_irqrestore(&vcxo_lock, irq_state);

	return 0;
}

static int vcxo_debugfs_set_vcxo_value(void *ctx, u64 value)
{
	s16 rate = (s16)value;
	steelhead_set_vcxo_rate(rate);
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(vcxo_debugfs_vcxo_value_fops,
		vcxo_debugfs_get_vcxo_value,
		vcxo_debugfs_set_vcxo_value,
		"%lld");

static void vcxo_debugfs_cleanup(void)
{
	if (IS_ERR_OR_NULL(vcxo_debugfs_dir))
		debugfs_remove_recursive(vcxo_debugfs_dir);

	vcxo_debugfs_dir = NULL;
	vcxo_debugfs_test_node = NULL;
	vcxo_debugfs_vcxo_value_node = NULL;
}

static void vcxo_debugfs_init(void)
{
	vcxo_debugfs_dir = debugfs_create_dir(kVCXODebugDirName, NULL);

	if (IS_ERR_OR_NULL(vcxo_debugfs_dir))
		goto err;

	vcxo_debugfs_test_node = debugfs_create_file(
			kVCXOTestNodeName,
			0644, vcxo_debugfs_dir,
			NULL, &vcxo_debugfs_test_fops);

	if (IS_ERR_OR_NULL(vcxo_debugfs_test_node))
		goto err;

	vcxo_debugfs_vcxo_value_node = debugfs_create_file(
			kVCXOValueNodeName,
			0644, vcxo_debugfs_dir,
			NULL, &vcxo_debugfs_vcxo_value_fops);

	if (IS_ERR_OR_NULL(vcxo_debugfs_vcxo_value_node))
		goto err;

	return;
err:
	vcxo_debugfs_cleanup();
}
#else
static void vcxo_debugfs_init(void) { }
#endif

void __init steelhead_platform_init_counter(void)
{
	u64 tmp;

	/* Get a hold of the timer we plan to use as the basis for local time
	 * and set it up to count using the sysclock auto resetting every time
	 * it rolls over.
	 */
	struct clk *steelhead_clock;
	BUG_ON(NULL != counter_timer);
	counter_timer = omap_dm_timer_request_specific(DM_TIMER_ID);
	BUG_ON(IS_ERR_OR_NULL(counter_timer));

	omap_dm_timer_set_source(counter_timer, DM_TIMER_CLK);

	steelhead_clock = omap_dm_timer_get_fclk(counter_timer);
	BUG_ON(IS_ERR_OR_NULL(steelhead_clock));
	counter_freq = clk_get_rate(steelhead_clock);

	/* Initialize the state we use for extending the counter to 64 bits and
	 * register the rollover check timer to make certain the counter is
	 * called frequently enough to detect rollover.
	 */
	counter_state.upper = 0;
	counter_state.lower_last = 0;
#ifdef CONFIG_AAH_TIMESYNC_DEBUG
	tsdebug_counter_state.upper = 0;
	tsdebug_counter_state.lower_last = 0;
#endif

	/* Just to be safe, check for rollover every 1/4 of a 32 bit counter
	 * period.
	 */
	tmp = 0x40000000ull * HZ;
	do_div(tmp, steelhead_get_raw_counter_nominal_freq());
	rollover_check_timer_period = (u32)tmp;

	init_timer(&rollover_check_timer);
	rollover_check_timer.function = counter_rollover_check;
	rollover_check_timer.expires  = jiffies + rollover_check_timer_period;
	add_timer(&rollover_check_timer);

	/* Our GP timer should be fully set up now.  Go ahead and start it up.
	 */
	omap_dm_timer_set_load_start(counter_timer, 1, 0);

#ifdef CONFIG_AAH_TIMESYNC_DEBUG
	/* If we are set up to monitor the local clock rate for debugging
	 * purposes from an externally synchronized event, then set up capture
	 * mode on the OMAP GP timer we have chosen to produce the timesync
	 * event.
	 */
	{
		int status;
		omap_dm_timers_set_capture_mode(counter_timer,
				OMAP_TIMER_CAPTURE_MODE_SINGLE,
				OMAP_TIMER_CAPTURE_EDGE_BOTH);
		status = request_irq(
				omap_dm_timer_get_irq(counter_timer),
				timer_capture_irq,
				IRQF_TIMER | IRQF_IRQPOLL,
				"steelhead tsdebug",
				counter_timer);
		if (status) {
			pr_err("Steelhead: Failed to setup timesync debug "
					"interrupt (status = %d)\n", status);
			BUG_ON(1);
		}

		/* NOTE: do not use omap_dm_timer_set_int_enable.  TI's timer
		 * libraries are super broken right now (see comments up near
		 * the VCXO control code) and will disable the timer
		 * unconditionally after enabling interrupts which causes all
		 * sorts of other trouble.
		 */
		timer_write_reg(counter_timer, TIMER_INT_EN_OFFSET,
				OMAP_TIMER_INT_CAPTURE);
		timer_write_reg(counter_timer, TIMER_WAKEUP_EN_OFFSET,
				OMAP_TIMER_INT_CAPTURE);

		/* enable the capture interrupt pin as an input */
		omap_mux_init_signal(DM_TIMER_CAPTURE_PIN_NAME,
				OMAP_PIN_INPUT_PULLDOWN);
	}
#endif

	/* Now setup the PWM we use to control the VCXO used to slew the main
	 * system oscillator. */
	steelhead_setup_vcxo_control();

	/* Register the platform driver which will give user-land access to the
	 * local time clock */
	platform_device_register(&aah_localtime_device);

	/* Finally, register the debugFS entries used in manufacturing
	 * diagnostics to verify VCXO performance. */
	vcxo_debugfs_init();
}
