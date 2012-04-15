/*
 *  drivers/cpufreq/cpufreq_hyper.c
 *
 *  Copyright (C)  2001 Russell King
 *            (C)  2003 Venkatesh Pallipadi <venkatesh.pallipadi@intel.com>.
 *                      Jun Nakajima <jun.nakajima@intel.com>
 *
 *                 2012 Minor Edits by Sar Castillo <sar.castillo@gmail.com>
 *                 2012 MAR heavy addons by DORIMANX <yuri@bynet.co.il>
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/cpufreq.h>
#include <linux/cpu.h>
#include <linux/jiffies.h>
#include <linux/kernel_stat.h>
#include <linux/mutex.h>
#include <linux/hrtimer.h>
#include <linux/tick.h>
#include <linux/ktime.h>
#include <linux/sched.h>
#include <linux/notifier.h>
#include <linux/earlysuspend.h>

#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/slab.h>

#include <../arch/x86/include/asm/idle.h>

#define CPUMON 0
#define _LIMIT_LCD_OFF_CPU_MAX_FREQ_

/*
 * dbs is used in this file as a shortform for demandbased switching
 * It helps to keep variable names smaller, simpler
 */

/* Tuned for MAX performance and MID battery save */
#define DEF_FREQUENCY_DOWN_DIFFERENTIAL		(10)
#define DEF_FREQUENCY_UP_THRESHOLD		(70)
#define DEF_SAMPLING_DOWN_FACTOR		(10)
#define MAX_SAMPLING_DOWN_FACTOR		(10000)
#define MICRO_FREQUENCY_DOWN_DIFFERENTIAL	(2)
#define MICRO_FREQUENCY_UP_THRESHOLD		(70)
#define MICRO_FREQUENCY_MIN_SAMPLE_RATE		(10000)
#define MIN_FREQUENCY_UP_THRESHOLD		(10)
#define MAX_FREQUENCY_UP_THRESHOLD		(100)
#define MIN_FREQUENCY_DOWN_DIFFERENTIAL		(1)
#define DEF_SUSPEND_FREQ			(245000)

/*
 * The polling frequency of this governor depends on the capability of
 * the processor. Default polling frequency is 1000 times the transition
 * latency of the processor. The governor will work on any processor with
 * transition latency <= 10mS, using appropriate sampling
 * rate.
 * For CPUs with transition latency > 10mS (mostly drivers with CPUFREQ_ETERNAL)
 * this governor will not work.
 * All times here are in uS.
 */
#define MIN_SAMPLING_RATE_RATIO			(1)

static unsigned int min_sampling_rate;

static DEFINE_PER_CPU(unsigned int, idle_start);
static DEFINE_PER_CPU(unsigned int, idle_end);

static int idle_event_handler(struct notifier_block *n,
	unsigned long cmd, void *p)
{
	switch (cmd) {
	case IDLE_START:
		percpu_write(idle_start, jiffies);
		break;
	case IDLE_END:
		percpu_write(idle_end, jiffies);
		break;
	}

	return NOTIFY_DONE;
}

static struct notifier_block idle_notifier_block = {
	.notifier_call  = idle_event_handler,
};

#define MIN_LATENCY_MULTIPLIER			(100)
#if defined(CONFIG_ARCH_MSM_SCORPION)
#define TRANSITION_LATENCY_LIMIT (8000000)
#else
#define TRANSITION_LATENCY_LIMIT (10000000)
#endif

#define POWERSAVE_BIAS_MAXLEVEL			(1000)
#define POWERSAVE_BIAS_MINLEVEL			(-1000)
static void do_dbs_timer(struct work_struct *work);
static int cpufreq_governor_dbs(struct cpufreq_policy *policy,
				unsigned int event);

#ifdef _LIMIT_LCD_OFF_CPU_MAX_FREQ_
#ifdef CONFIG_HAS_EARLYSUSPEND
static struct early_suspend cpufreq_gov_early_suspend;
static unsigned int cpufreq_gov_lcd_status;
#endif
#endif

#ifndef CONFIG_CPU_FREQ_DEFAULT_GOV_HYPER
static
#endif
struct cpufreq_governor cpufreq_gov_hyper = {
       .name                   = "hyper",
       .governor               = cpufreq_governor_dbs,
       .max_transition_latency = TRANSITION_LATENCY_LIMIT,
       .owner                  = THIS_MODULE,
};

/* Sampling types */
enum {DBS_NORMAL_SAMPLE, DBS_SUB_SAMPLE};

struct cpu_dbs_info_s {
	cputime64_t prev_cpu_idle;
	cputime64_t prev_cpu_iowait;
	cputime64_t prev_cpu_wall;
	cputime64_t prev_cpu_nice;
	struct cpufreq_policy *cur_policy;
	struct delayed_work work;
	struct cpufreq_frequency_table *freq_table;
	unsigned int freq_lo;
	unsigned int freq_lo_jiffies;
	unsigned int freq_hi_jiffies;
	unsigned int rate_mult;
	int cpu;
	unsigned int sample_type:1;
	/*
	 * percpu mutex that serializes governor limit change with
	 * do_dbs_timer invocation. We do not want do_dbs_timer to run
	 * when user is changing the governor or limits.
	 */
	struct mutex timer_mutex;
};
static DEFINE_PER_CPU(struct cpu_dbs_info_s, od_cpu_dbs_info);

static inline void dbs_timer_init(struct cpu_dbs_info_s *dbs_info);
static inline void dbs_timer_exit(struct cpu_dbs_info_s *dbs_info);
static unsigned int dbs_enable;	/* number of CPUs using this policy */

/*
 * dbs_mutex protects dbs_enable in governor start/stop.
 */
static DEFINE_MUTEX(dbs_mutex);

static struct workqueue_struct *input_wq;

static DEFINE_PER_CPU(struct work_struct, dbs_refresh_work);
static struct dbs_tuners {
	unsigned int sampling_rate;
	unsigned int up_threshold;
	unsigned int down_differential;
	unsigned int ignore_nice;
	unsigned int sampling_down_factor;
	int          powersave_bias;
	unsigned int io_is_busy;
	unsigned int fast_start;
	unsigned int deep_sleep;
	unsigned int suspend_freq;
} dbs_tuners_ins = {
	.up_threshold = DEF_FREQUENCY_UP_THRESHOLD,
	.sampling_down_factor = DEF_SAMPLING_DOWN_FACTOR,
	.down_differential = DEF_FREQUENCY_DOWN_DIFFERENTIAL,
	.ignore_nice = 0,
	.powersave_bias = 0,
	.deep_sleep = 1,
	.fast_start = 1,
	.suspend_freq = DEF_SUSPEND_FREQ,
};

static unsigned int dbs_enable=0;	/* number of CPUs using this policy */

// hyper suspend mods (Thanks to Imoseyon)
static unsigned int suspended = 0;
static void hyper_suspend(int suspend)
{
        struct cpu_dbs_info_s *dbs_info = &per_cpu(od_cpu_dbs_info, smp_processor_id());
        if (dbs_enable==0) return;
        if (!suspend) { // resume at max speed:
                suspended = 0;
                __cpufreq_driver_target(dbs_info->cur_policy, dbs_info->cur_policy->max,
			CPUFREQ_RELATION_L);
                pr_info("[hyper] hyper awake at %d\n", dbs_info->cur_policy->cur);
        } else {
                suspended = 1;
		// let's give it a little breathing room
                __cpufreq_driver_target(dbs_info->cur_policy, dbs_tuners_ins.suspend_freq, CPUFREQ_RELATION_H);
                pr_info("[hyper] hyper suspended at %d\n", dbs_info->cur_policy->cur);
        }
}

static void hyper_early_suspend(struct early_suspend *handler) {
       hyper_suspend(1);
}

static void hyper_late_resume(struct early_suspend *handler) {
       hyper_suspend(0);
}

static struct early_suspend hyper_power_suspend = {
        .suspend = hyper_early_suspend,
        .resume = hyper_late_resume,
        .level = EARLY_SUSPEND_LEVEL_DISABLE_FB + 1,
};

//intellidemand mod
static unsigned int cpufreq_gov_lcd_status=1;
static void cpufreq_gov_suspend(struct early_suspend *h)
{
	cpufreq_gov_lcd_status = 0;
}

static void cpufreq_gov_resume(struct early_suspend *h)
{
	cpufreq_gov_lcd_status = 1;
}

static struct early_suspend cpufreq_gov_early_suspend = {
         .suspend = cpufreq_gov_suspend,
         .resume = cpufreq_gov_resume,
         .level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1,
};

static inline cputime64_t get_cpu_idle_time_jiffy(unsigned int cpu,
							cputime64_t *wall)
{
	cputime64_t idle_time;
	cputime64_t cur_wall_time;
	cputime64_t busy_time;

	cur_wall_time = jiffies64_to_cputime64(get_jiffies_64());
	busy_time = cputime64_add(kstat_cpu(cpu).cpustat.user,
			kstat_cpu(cpu).cpustat.system);

	busy_time = cputime64_add(busy_time, kstat_cpu(cpu).cpustat.irq);
	busy_time = cputime64_add(busy_time, kstat_cpu(cpu).cpustat.softirq);
	busy_time = cputime64_add(busy_time, kstat_cpu(cpu).cpustat.steal);
	busy_time = cputime64_add(busy_time, kstat_cpu(cpu).cpustat.nice);

	idle_time = cputime64_sub(cur_wall_time, busy_time);
	if (wall)
		*wall = (cputime64_t)jiffies_to_usecs(cur_wall_time);

	return (cputime64_t)jiffies_to_usecs(idle_time);
}

static inline cputime64_t get_cpu_idle_time(unsigned int cpu, cputime64_t *wall)
{
	u64 idle_time = get_cpu_idle_time_us(cpu, wall);

	if (idle_time == -1ULL)
		return get_cpu_idle_time_jiffy(cpu, wall);

	return idle_time;
}

static inline cputime64_t get_cpu_iowait_time(unsigned int cpu, cputime64_t *wall)
{
	u64 iowait_time = get_cpu_iowait_time_us(cpu, wall);

	if (iowait_time == -1ULL)
		return 0;

	return iowait_time;
}

/*
 * Find right freq to be set now with powersave_bias on.
 * Returns the freq_hi to be used right now and will set freq_hi_jiffies,
 * freq_lo, and freq_lo_jiffies in percpu area for averaging freqs.
 */
static unsigned int powersave_bias_target(struct cpufreq_policy *policy,
					  unsigned int freq_next,
					  unsigned int relation)
{
	unsigned int freq_req, freq_avg;
	unsigned int freq_hi, freq_lo;
	unsigned int index = 0;
	unsigned int jiffies_total, jiffies_hi, jiffies_lo;
	int freq_reduc;
	struct cpu_dbs_info_s *dbs_info = &per_cpu(od_cpu_dbs_info,
						   policy->cpu);

	if (!dbs_info->freq_table) {
		dbs_info->freq_lo = 0;
		dbs_info->freq_lo_jiffies = 0;
		return freq_next;
	}

	cpufreq_frequency_table_target(policy, dbs_info->freq_table, freq_next,
			relation, &index);
	freq_req = dbs_info->freq_table[index].frequency;
	freq_reduc = freq_req * dbs_tuners_ins.powersave_bias / 1000;
	freq_avg = freq_req - freq_reduc;

	/* Find freq bounds for freq_avg in freq_table */
	index = 0;
	cpufreq_frequency_table_target(policy, dbs_info->freq_table, freq_avg,
			CPUFREQ_RELATION_H, &index);
	freq_lo = dbs_info->freq_table[index].frequency;
	index = 0;
	cpufreq_frequency_table_target(policy, dbs_info->freq_table, freq_avg,
			CPUFREQ_RELATION_L, &index);
	freq_hi = dbs_info->freq_table[index].frequency;

	/* Find out how long we have to be in hi and lo freqs */
	if (freq_hi == freq_lo) {
		dbs_info->freq_lo = 0;
		dbs_info->freq_lo_jiffies = 0;
		return freq_lo;
	}
	jiffies_total = usecs_to_jiffies(dbs_tuners_ins.sampling_rate);
	jiffies_hi = (freq_avg - freq_lo) * jiffies_total;
	jiffies_hi += ((freq_hi - freq_lo) / 2);
	jiffies_hi /= (freq_hi - freq_lo);
	jiffies_lo = jiffies_total - jiffies_hi;
	dbs_info->freq_lo = freq_lo;
	dbs_info->freq_lo_jiffies = jiffies_lo;
	dbs_info->freq_hi_jiffies = jiffies_hi;
	return freq_hi;
}

static int hyper_powersave_bias_setspeed(struct cpufreq_policy *policy,
					    struct cpufreq_policy *altpolicy,
					    int level)
{
	if (level == POWERSAVE_BIAS_MAXLEVEL) {
		/* maximum powersave; set to lowest frequency */
		__cpufreq_driver_target(policy,
			(altpolicy) ? altpolicy->min : policy->min,
			CPUFREQ_RELATION_L);
		return 1;
	} else if (level == POWERSAVE_BIAS_MINLEVEL) {
		/* minimum powersave; set to highest frequency */
		__cpufreq_driver_target(policy,
			(altpolicy) ? altpolicy->max : policy->max,
			CPUFREQ_RELATION_H);
		return 1;
	}
	return 0;
}

static void hyper_powersave_bias_init_cpu(int cpu)
{
	struct cpu_dbs_info_s *dbs_info = &per_cpu(od_cpu_dbs_info, cpu);
	dbs_info->freq_table = cpufreq_frequency_get_table(cpu);
	dbs_info->freq_lo = 0;
}

static void hyper_powersave_bias_init(void)
{
	int i;
	for_each_online_cpu(i) {
		hyper_powersave_bias_init_cpu(i);
	}
}

/************************** sysfs interface ************************/

static ssize_t show_sampling_rate_min(struct kobject *kobj,
				      struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", min_sampling_rate);
}

define_one_global_ro(sampling_rate_min);

/* cpufreq_hyper Governor Tunables */
#define show_one(file_name, object)					\
static ssize_t show_##file_name						\
(struct kobject *kobj, struct attribute *attr, char *buf)              \
{									\
	return sprintf(buf, "%u\n", dbs_tuners_ins.object);		\
}
show_one(sampling_rate, sampling_rate);
show_one(io_is_busy, io_is_busy);
show_one(deep_sleep, deep_sleep);
show_one(up_threshold, up_threshold);
show_one(down_differential, down_differential);
show_one(sampling_down_factor, sampling_down_factor);
show_one(ignore_nice_load, ignore_nice);
show_one(suspend_freq, suspend_freq);
show_one(fast_start, fast_start);

#ifdef CONFIG_SEC_LIMIT_MAX_FREQ // limit max freq
void set_lmf_browser_state(bool onOff);
void set_lmf_temp_state(bool onOff);
void set_lmf_active_load(unsigned long freq);
void set_lmf_inactive_load(unsigned long freq);
bool get_lmf_browser_state(void);
bool get_lmf_temp_state(void);
unsigned long get_lmf_active_load(void);
unsigned long get_lmf_inactive_load(void);

static ssize_t show_lmf_temp(struct kobject *kobj,
				      struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", get_lmf_temp_state());
}

static ssize_t show_lmf_browser(struct kobject *kobj,
				      struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", get_lmf_browser_state());
}

static ssize_t show_lmf_active_load(struct kobject *kobj,
				      struct attribute *attr, char *buf)
{
	return sprintf(buf, "%ld\n", get_lmf_active_load());
}

static ssize_t show_lmf_inactive_load(struct kobject *kobj,
				      struct attribute *attr, char *buf)
{
	return sprintf(buf, "%ld\n", get_lmf_inactive_load());
}
#endif

static ssize_t show_powersave_bias
(struct kobject *kobj, struct attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", dbs_tuners_ins.powersave_bias);
}

static ssize_t store_sampling_rate(struct kobject *a, struct attribute *b,
				   const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;

	dbs_tuners_ins.sampling_rate = max(input, min_sampling_rate);
	return count;
}

static ssize_t store_io_is_busy(struct kobject *a, struct attribute *b,
				   const char *buf, size_t count)
{
	unsigned int input;
	int ret;

	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;

	dbs_tuners_ins.io_is_busy = !!input;
	return count;
}

static ssize_t store_fast_start(struct kobject *a, struct attribute *b,
				 const char *buf, size_t count)
{
	unsigned int input;
	int ret;

	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;

	dbs_tuners_ins.fast_start = !!input;

	return count;
}

static ssize_t store_deep_sleep(struct kobject *a, struct attribute *b,
				  const char *buf, size_t count)
{
	unsigned int input;
	int ret;

	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;

	dbs_tuners_ins.deep_sleep = !!input;

	return count;
}

static ssize_t store_up_threshold(struct kobject *a, struct attribute *b,
				  const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1 || input > MAX_FREQUENCY_UP_THRESHOLD ||
			input < MIN_FREQUENCY_UP_THRESHOLD) {
		return -EINVAL;
	}

	dbs_tuners_ins.up_threshold = input;
	return count;
}

static ssize_t store_down_differential(struct kobject *a, struct attribute *b,
		const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1 || input >= dbs_tuners_ins.up_threshold ||
			input < MIN_FREQUENCY_DOWN_DIFFERENTIAL) {
		return -EINVAL;
	}

	dbs_tuners_ins.down_differential = input;

	return count;
}

static ssize_t store_sampling_down_factor(struct kobject *a,
			struct attribute *b, const char *buf, size_t count)
{
	unsigned int input, j;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1 || input > MAX_SAMPLING_DOWN_FACTOR || input < 1)
		return -EINVAL;
	dbs_tuners_ins.sampling_down_factor = input;

	/* Reset down sampling multiplier in case it was active */
	for_each_online_cpu(j) {
		struct cpu_dbs_info_s *dbs_info;
		dbs_info = &per_cpu(od_cpu_dbs_info, j);
		dbs_info->rate_mult = 1;
	}
	return count;
}

static ssize_t store_ignore_nice_load(struct kobject *a, struct attribute *b,
				      const char *buf, size_t count)
{
	unsigned int input;
	int ret;

	unsigned int j;

	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;

	if (input > 1)
		input = 1;

	if (input == dbs_tuners_ins.ignore_nice) { /* nothing to do */
		return count;
	}
	dbs_tuners_ins.ignore_nice = input;

	/* we need to re-evaluate prev_cpu_idle */
	for_each_online_cpu(j) {
		struct cpu_dbs_info_s *dbs_info;
		dbs_info = &per_cpu(od_cpu_dbs_info, j);
		dbs_info->prev_cpu_idle = get_cpu_idle_time(j,
						&dbs_info->prev_cpu_wall);
		if (dbs_tuners_ins.ignore_nice)
			dbs_info->prev_cpu_nice = kstat_cpu(j).cpustat.nice;

	}
	return count;
}

static ssize_t store_powersave_bias(struct kobject *a, struct attribute *b,
				    const char *buf, size_t count)
{
	int input  = 0;
	int bypass = 0;
	int ret, cpu, reenable_timer;
	struct cpu_dbs_info_s *dbs_info;

	ret = sscanf(buf, "%d", &input);

	if (ret != 1)
		return -EINVAL;

	if (input >= POWERSAVE_BIAS_MAXLEVEL) {
		input  = POWERSAVE_BIAS_MAXLEVEL;
		bypass = 1;
	} else if (input <= POWERSAVE_BIAS_MINLEVEL) {
		input  = POWERSAVE_BIAS_MINLEVEL;
		bypass = 1;
	}

	if (input == dbs_tuners_ins.powersave_bias) {
		/* no change */
		return count;
	}

	reenable_timer = ((dbs_tuners_ins.powersave_bias ==
				POWERSAVE_BIAS_MAXLEVEL) ||
				(dbs_tuners_ins.powersave_bias ==
				POWERSAVE_BIAS_MINLEVEL));

	dbs_tuners_ins.powersave_bias = input;
	if (!bypass) {
		if (reenable_timer) {
			/* reinstate dbs timer */
			for_each_online_cpu(cpu) {
				if (lock_policy_rwsem_write(cpu) < 0)
					continue;

				dbs_info = &per_cpu(od_cpu_dbs_info, cpu);
				if (dbs_info->cur_policy) {
					/* restart dbs timer */
					dbs_timer_init(dbs_info);
				}
				unlock_policy_rwsem_write(cpu);
			}
		}
		hyper_powersave_bias_init();
	} else {
		/* running at maximum or minimum frequencies; cancel
		   dbs timer as periodic load sampling is not necessary */
		for_each_online_cpu(cpu) {
			if (lock_policy_rwsem_write(cpu) < 0)
				continue;

			dbs_info = &per_cpu(od_cpu_dbs_info, cpu);
			if (dbs_info->cur_policy) {
				/* cpu using ondemand, cancel dbs timer */
				mutex_lock(&dbs_info->timer_mutex);
				dbs_timer_exit(dbs_info);

				hyper_powersave_bias_setspeed(
					dbs_info->cur_policy,
					NULL,
					input);

				mutex_unlock(&dbs_info->timer_mutex);
			}
			unlock_policy_rwsem_write(cpu);
		}
	}

	return count;
}

static ssize_t store_suspend_freq(struct kobject *a, struct attribute *b,
				    const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1)
		return -EINVAL;

	if (input > 1200000)
		input = 1200000;

	if (input < 100000)
		input = 100000;

	dbs_tuners_ins.suspend_freq = input;

	return count;
}

#ifdef CONFIG_SEC_LIMIT_MAX_FREQ // limit max freq
static ssize_t store_lmf_temp(struct kobject *a, struct attribute *b,
				   const char *buf, size_t count)
{
	unsigned int input;
	int ret;

	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;

	set_lmf_temp_state(input);

	return count;
}

static ssize_t store_lmf_browser(struct kobject *a, struct attribute *b,
				   const char *buf, size_t count)
{
	unsigned int input;
	int ret;

	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;

	set_lmf_browser_state(input);

	return count;
}

static ssize_t store_lmf_active_load(struct kobject *a, struct attribute *b,
				   const char *buf, size_t count)
{
	unsigned long input;
	int ret;

	ret = sscanf(buf, "%ld", &input);
	if (ret != 1)
		return -EINVAL;

	set_lmf_active_load(input);

	return count;
}

static ssize_t store_lmf_inactive_load(struct kobject *a, struct attribute *b,
				   const char *buf, size_t count)
{
	unsigned long input;
	int ret;

	ret = sscanf(buf, "%ld", &input);
	if (ret != 1)
		return -EINVAL;

	set_lmf_inactive_load(input);

	return count;
}
#endif

define_one_global_rw(sampling_rate);
define_one_global_rw(io_is_busy);
define_one_global_rw(deep_sleep);
define_one_global_rw(fast_start);
define_one_global_rw(up_threshold);
define_one_global_rw(down_differential);
define_one_global_rw(sampling_down_factor);
define_one_global_rw(ignore_nice_load);
define_one_global_rw(powersave_bias);
define_one_global_rw(suspend_freq);
#ifdef CONFIG_SEC_LIMIT_MAX_FREQ // limit max freq
define_one_global_rw(lmf_temp);
define_one_global_rw(lmf_browser);
define_one_global_rw(lmf_active_load);
define_one_global_rw(lmf_inactive_load);
#endif

static struct attribute *dbs_attributes[] = {
	&sampling_rate_min.attr,
	&sampling_rate.attr,
	&up_threshold.attr,
	&down_differential.attr,
	&sampling_down_factor.attr,
	&ignore_nice_load.attr,
	&powersave_bias.attr,
	&io_is_busy.attr,
	&deep_sleep.attr,
	&fast_start.attr,
	&suspend_freq.attr,
#ifdef CONFIG_SEC_LIMIT_MAX_FREQ // limit max freq
	&lmf_temp.attr,
	&lmf_browser.attr,
	&lmf_active_load.attr,
	&lmf_inactive_load.attr,
#endif
	NULL
};

static struct attribute_group dbs_attr_group = {
	.attrs = dbs_attributes,
	.name = "hyper",
};

/************************** sysfs end ************************/

static void dbs_freq_increase(struct cpufreq_policy *p, unsigned int freq)
{
	if (dbs_tuners_ins.powersave_bias)
		freq = powersave_bias_target(p, freq, CPUFREQ_RELATION_H);
	else if (p->cur == p->max)
		return;

	if (suspended && freq > dbs_tuners_ins.suspend_freq) {
		freq = dbs_tuners_ins.suspend_freq;
		__cpufreq_driver_target(p, freq, CPUFREQ_RELATION_H);
	} else
	__cpufreq_driver_target(p, freq, dbs_tuners_ins.powersave_bias ?
			CPUFREQ_RELATION_L : CPUFREQ_RELATION_H);
}

static void dbs_check_cpu(struct cpu_dbs_info_s *this_dbs_info)
{
	unsigned int max_load_freq;

	struct cpufreq_policy *policy;
	unsigned int j;

	this_dbs_info->freq_lo = 0;
	policy = this_dbs_info->cur_policy;

	/*
	 * Every sampling_rate, we check, if current idle time is less
	 * than 20% (default), then we try to increase frequency
	 * Every sampling_rate, we look for a the lowest
	 * frequency which can sustain the load while keeping idle time over
	 * 30%. If such a frequency exist, we try to decrease to this frequency.
	 *
	 * Any frequency increase takes it to the maximum frequency.
	 * Frequency reduction happens at minimum steps of
	 * 5% (default) of current frequency
	 */

	/* Get Absolute Load - in terms of freq */
	max_load_freq = 0;

#if CPUMON
	int load_each[2] = {0, 0};
#endif

	for_each_cpu(j, policy->cpus) {
		struct cpu_dbs_info_s *j_dbs_info;
		cputime64_t cur_wall_time, cur_idle_time, cur_iowait_time;
		unsigned int idle_time, wall_time, iowait_time;
		unsigned int load, load_freq;
		int freq_avg;
		unsigned long start, end, delta, sampling_delta;

		j_dbs_info = &per_cpu(od_cpu_dbs_info, j);

		start = per_cpu(idle_start, j);
		end = per_cpu(idle_end, j);
		if (time_after(end, start))
			delta = (long)end - (long)start;
		else
			delta = 0;
		sampling_delta = usecs_to_jiffies(dbs_tuners_ins.sampling_rate);

		/*
		 * Deep sleep detection.
		 *
		 * If the previous low-power idle interval was 4x the sampling
		 * rate or more, we're probably exiting or entering from a deep
		 * sleep state.
		 *
		 * In both cases ignore the cpu activity, because probably
		 * there's a big chance to get back to the deep sleep again.
		 */
		if (dbs_tuners_ins.deep_sleep) {
			if (delta > 4 * sampling_delta)
				continue;
		}

		/*
		 * Fast start detection.
		 *
		 * If in the last 4 samples the cpu did not enter in low-power
		 * idle state, probably the cpu is busy; try to jump to the
		 * maximum speed.
		 */
		if (dbs_tuners_ins.fast_start) {
			if (delta && time_is_before_jiffies(end +
							4 * sampling_delta)) {
				max_load_freq = UINT_MAX;
				break;
			}
		}

		cur_idle_time = get_cpu_idle_time(j, &cur_wall_time);
		cur_iowait_time = get_cpu_iowait_time(j, &cur_wall_time);

		wall_time = (unsigned int) cputime64_sub(cur_wall_time,
				j_dbs_info->prev_cpu_wall);
		j_dbs_info->prev_cpu_wall = cur_wall_time;

		idle_time = (unsigned int) cputime64_sub(cur_idle_time,
				j_dbs_info->prev_cpu_idle);
		j_dbs_info->prev_cpu_idle = cur_idle_time;

		iowait_time = (unsigned int) cputime64_sub(cur_iowait_time,
				j_dbs_info->prev_cpu_iowait);
		j_dbs_info->prev_cpu_iowait = cur_iowait_time;

		if (dbs_tuners_ins.ignore_nice) {
			cputime64_t cur_nice;
			unsigned long cur_nice_jiffies;

			cur_nice = cputime64_sub(kstat_cpu(j).cpustat.nice,
					 j_dbs_info->prev_cpu_nice);
			/*
			 * Assumption: nice time between sampling periods will
			 * be less than 2^32 jiffies for 32 bit sys
			 */
			cur_nice_jiffies = (unsigned long)
					cputime64_to_jiffies64(cur_nice);

			j_dbs_info->prev_cpu_nice = kstat_cpu(j).cpustat.nice;
			idle_time += jiffies_to_usecs(cur_nice_jiffies);
		}

		/*
		 * For the purpose of hyper, waiting for disk IO is an
		 * indication that you're performance critical, and not that
		 * the system is actually idle. So subtract the iowait time
		 * from the cpu idle time.
		 */

		if (dbs_tuners_ins.io_is_busy && idle_time >= iowait_time)
			idle_time -= iowait_time;

		if (unlikely(!wall_time || wall_time < idle_time))
			continue;

		load = 100 * (wall_time - idle_time) / wall_time;
#if CPUMON
		load_each[j & 0x01] = load;
#endif

		freq_avg = __cpufreq_driver_getavg(policy, j);
		if (freq_avg <= 0)
			freq_avg = policy->cur;

		load_freq = load * freq_avg;
		if (load_freq > max_load_freq)
			max_load_freq = load_freq;
	}

#if CPUMON
	printk(KERN_ERR "CPUMON L %d %d\n", load_each[0], load_each[1]);
#endif

	/* Check for frequency increase */
	if (max_load_freq > dbs_tuners_ins.up_threshold * policy->cur) {
#ifdef _LIMIT_LCD_OFF_CPU_MAX_FREQ_
		if(!cpufreq_gov_lcd_status) {
			if (policy->cur < policy->max) {
				if (policy->cur < 245000) dbs_freq_increase(policy, 998400);
				else if (policy->cur < 499200) dbs_freq_increase(policy, 998400);
				else {
					this_dbs_info->rate_mult = dbs_tuners_ins.sampling_down_factor;
					dbs_freq_increase(policy, policy->max);
				}
			}
			return;
		} else
#endif

		/* if we are already at full speed then break out early */
		if (!dbs_tuners_ins.powersave_bias) {
			this_dbs_info->rate_mult =
				dbs_tuners_ins.sampling_down_factor;

			__cpufreq_driver_target(policy, policy->max,
				CPUFREQ_RELATION_H);
		} else {
			int freq = powersave_bias_target(policy, policy->max,
					CPUFREQ_RELATION_H);
			__cpufreq_driver_target(policy, freq,
				CPUFREQ_RELATION_L);
		}
		return;
	}

	/* Check for frequency decrease */
	/* if we cannot reduce the frequency anymore, break out early */
        if(!cpufreq_gov_lcd_status) {
	if (policy->cur == policy->min)
		return;
        } else
	if (policy->cur == policy->min) {
		__cpufreq_driver_target(policy, policy->min,
				CPUFREQ_RELATION_L);
		return;
	}

	/*
	 * The optimal frequency is the frequency that is the lowest that
	 * can support the current CPU usage without triggering the up
	 * policy. To be safe, we focus 10 points under the threshold.
	 */
	if (max_load_freq <
	    (dbs_tuners_ins.up_threshold - dbs_tuners_ins.down_differential) *
	     policy->cur) {
		unsigned int freq_next;
		freq_next = max_load_freq /
				(dbs_tuners_ins.up_threshold -
				 dbs_tuners_ins.down_differential);

		/* No longer fully busy, reset rate_mult */
		this_dbs_info->rate_mult = 1;

		if (freq_next < policy->min)
			freq_next = policy->min;

		if (!dbs_tuners_ins.powersave_bias) {
			__cpufreq_driver_target(policy, freq_next,
					CPUFREQ_RELATION_L);
		} else {
			int freq = powersave_bias_target(policy, freq_next,
					CPUFREQ_RELATION_L);
			__cpufreq_driver_target(policy, freq,
				CPUFREQ_RELATION_L);
		}
	}
}

#ifdef CONFIG_DETECT_BROWSER_STATE
extern bool yamato_busy;
#endif

#ifdef CONFIG_SEC_LIMIT_MAX_FREQ // limit max freq

#include "../../kernel/power/power.h"

enum {	
	SET_MIN = 0,	
	SET_MAX
};

enum {	
	BOOT_CPU = 0,	
	NON_BOOT_CPU
};

#define SAMPLE_DURATION_MSEC	(10*1000) // 10 secs >= 10000 msec
#define ACTIVE_DURATION_MSEC	(10*60*1000) // 10 mins
#define INACTIVE_DURATION_MSEC	(2*60*1000) // 2 mins
#define MAX_ACTIVE_FREQ_LIMIT	65 // %
#define MAX_INACTIVE_FREQ_LIMIT	45 // %
#ifdef CONFIG_BOOST_L2_BANDWIDTH
#define ACTIVE_MAX_FREQ			998000 // 1.00GHz
#else
#define ACTIVE_MAX_FREQ			998000 // 1.00GHz
#endif
#define INACTIVE_MAX_FREQ		998000	// 1.0GHZ

#define NUM_ACTIVE_LOAD_ARRAY	(ACTIVE_DURATION_MSEC/SAMPLE_DURATION_MSEC)
#define NUM_INACTIVE_LOAD_ARRAY	(INACTIVE_DURATION_MSEC/SAMPLE_DURATION_MSEC)

static bool lmf_browser_state = false;
static bool lmf_temp_state = true; // temp is not used now

static unsigned long lmf_active_load_limit = MAX_ACTIVE_FREQ_LIMIT;
static unsigned long lmf_inactive_load_limit = MAX_INACTIVE_FREQ_LIMIT;

static unsigned long jiffies_old = 0;
static unsigned long time_int = 0;
static unsigned long time_int1 = 0;
static unsigned long load_state_total0  = 0;
static unsigned long load_state_total1  = 0;
static unsigned long load_limit_index = 0;	
static unsigned long load_limit_total[NUM_ACTIVE_LOAD_ARRAY];
static unsigned long msecs_limit_total = 0;
static bool active_state = true;
static bool lmf_old_state = false;

extern int cpufreq_set_limits(int cpu, unsigned int limit, unsigned int value);
extern int cpufreq_set_limits_off(int cpu, unsigned int limit, unsigned int value);
extern suspend_state_t get_suspend_state(void);

void set_lmf_browser_state(bool onOff)
{
	if (onOff)
		lmf_browser_state = true;
	else
		lmf_browser_state = false;
}

void set_lmf_temp_state(bool onOff)
{
	if (onOff)
		lmf_temp_state = true;
	else
		lmf_temp_state = false;
}

void set_lmf_active_load(unsigned long freq)
{
	lmf_active_load_limit = freq;
}

void set_lmf_inactive_load(unsigned long freq)
{
	lmf_inactive_load_limit = freq;
}

bool get_lmf_browser_state(void)
{
#ifdef CONFIG_DETECT_BROWSER_STATE
	return yamato_busy;
#else
	return lmf_browser_state;
#endif
}

bool get_lmf_temp_state(void)
{
	return lmf_temp_state;
}

unsigned long get_lmf_active_load(void)
{
	return lmf_active_load_limit;
}

unsigned long get_lmf_inactive_load(void)
{
	return lmf_inactive_load_limit;
}
#endif
static void do_dbs_timer(struct work_struct *work)
{
	struct cpu_dbs_info_s *dbs_info =
		container_of(work, struct cpu_dbs_info_s, work.work);
	unsigned int cpu = dbs_info->cpu;
	int sample_type = dbs_info->sample_type;

#ifdef CONFIG_SEC_LIMIT_MAX_FREQ // limit max freq

#ifdef CONFIG_HAS_EARLYSUSPEND
#ifdef _LIMIT_LCD_OFF_CPU_MAX_FREQ_
#ifdef CONFIG_DETECT_BROWSER_STATE
	if (!yamato_busy || !lmf_temp_state || !cpufreq_gov_lcd_status)
#else
	if (!lmf_browser_state || !lmf_temp_state || !cpufreq_gov_lcd_status)
#endif
#else
	if (!lmf_browser_state || !lmf_temp_state || !(get_suspend_state()==PM_SUSPEND_ON))
#endif
#else
	if (!lmf_browser_state || !lmf_temp_state)
#endif
	{
		if (cpu == BOOT_CPU)
		{
			if (lmf_old_state == true)
			{
				printk("LMF: disabled\n");
				lmf_old_state = false;
			}

			if (!active_state)
			{
				/* set freq to 1.0GHz */
				printk("LMF: CPU0 set max freq to 1.0GHz\n");
				cpufreq_set_limits(BOOT_CPU, SET_MAX, ACTIVE_MAX_FREQ);
				
				printk("LMF: CPU1 set max freq to 1.0GHz\n");
				if (cpu_online(NON_BOOT_CPU))
					cpufreq_set_limits(NON_BOOT_CPU, SET_MAX, ACTIVE_MAX_FREQ);
				else
					cpufreq_set_limits_off(NON_BOOT_CPU, SET_MAX, ACTIVE_MAX_FREQ);
			}
			
			jiffies_old = 0;
			time_int = 0;
			time_int1 = 0;
			load_state_total0 = 0;
			load_state_total1 = 0;
			msecs_limit_total = 0;
			load_limit_index = 0;
			active_state = true;
		}
	}
	else // lmf_browser_state && lmf_temp_state -> TRUE
	{
		struct cpufreq_policy *policy;
		unsigned long load_state_cpu = 0;
		unsigned int delay_msec = 0;
		unsigned long load_total  = 0;
		unsigned long jiffies_cur = jiffies;
		
		if (cpu == NON_BOOT_CPU)
		{
			delay_msec = (dbs_tuners_ins.sampling_rate * dbs_info->rate_mult) / 1000;
			policy = dbs_info->cur_policy;
			load_state_cpu = ((policy->cur) * delay_msec)/10000;

			time_int1 += delay_msec;
			load_state_total1 += load_state_cpu;
		}
		else
		{
			if (lmf_old_state == false)
			{
				printk("LMF: enabled\n");
				lmf_old_state = true;
			}

			if (jiffies_old == 0) 
			{
				jiffies_old = jiffies_cur;
			}
			else
			{
				delay_msec = jiffies_to_msecs(jiffies_cur - jiffies_old);
				jiffies_old = jiffies_cur;
				policy = dbs_info->cur_policy;
				load_state_cpu = ((policy->cur) * delay_msec)/10000;
				
				time_int += delay_msec;
				load_state_total0 += load_state_cpu;			
				
				/* average */
				if (time_int >= SAMPLE_DURATION_MSEC)
				{
					int i = 0;
					unsigned long ave_max = 0;
					unsigned long average = 0;
					unsigned long average_dec = 0;
					unsigned long total_load = 0;

					load_total = load_state_total0 + load_state_total1;
					ave_max = (time_int / 10) * ((ACTIVE_MAX_FREQ/1000) * 2);
					average = (load_total * 100) / ave_max;
					average_dec = (load_total  * 100) % ave_max;

					msecs_limit_total += time_int;
					load_limit_total[load_limit_index++] = average;

					//printk("LMF: average = %ld.%ld, (%ld:%ld) (%ld:%ld) (%ld:%ld)\n", 
					//	average, average_dec, time_int, time_int1, load_state_total0, load_state_total1, load_limit_index-1, msecs_limit_total);

					time_int = 0;
					time_int1 = 0;
					load_state_total0 = 0;
					load_state_total1 = 0;

					/* active */
					if (active_state)
					{
						if (load_limit_index >= NUM_ACTIVE_LOAD_ARRAY)
						{
							load_limit_index = 0;
						}
						
						if (msecs_limit_total > ACTIVE_DURATION_MSEC)
						{
							for (i=0; i<NUM_ACTIVE_LOAD_ARRAY; i++)
							{
								total_load += load_limit_total[i];
							}

							average = total_load / NUM_ACTIVE_LOAD_ARRAY;
							average_dec = total_load % NUM_ACTIVE_LOAD_ARRAY;
							//printk("LMF:ACTIVE: total_avg = %ld.%ld\n", average, average_dec);

							if (average > lmf_active_load_limit)
							{
								msecs_limit_total = 0;
								load_limit_index = 0;
								active_state = false;

								/* set freq to 1.0GHz */
								printk("LMF: CPU0 set max freq to 1.0GHz\n");
								cpufreq_set_limits(BOOT_CPU, SET_MAX, INACTIVE_MAX_FREQ);
								
								printk("LMF: CPU1 set max freq to 1.0GHz\n");
								if (cpu_online(NON_BOOT_CPU))
									cpufreq_set_limits(NON_BOOT_CPU, SET_MAX, INACTIVE_MAX_FREQ);
								else
									cpufreq_set_limits_off(NON_BOOT_CPU, SET_MAX, INACTIVE_MAX_FREQ);
							}
							else
							{
								msecs_limit_total = ACTIVE_DURATION_MSEC; // to prevent overflow
							}
						}
					}
					else /* inactive */
					{
						if (load_limit_index >= NUM_INACTIVE_LOAD_ARRAY)
						{
							load_limit_index = 0;
						}
						
						if (msecs_limit_total > INACTIVE_DURATION_MSEC)
						{
							for (i=0; i<NUM_INACTIVE_LOAD_ARRAY; i++)
							{
								total_load += load_limit_total[i];
							}

							average = total_load / NUM_INACTIVE_LOAD_ARRAY;
							average_dec = total_load % NUM_INACTIVE_LOAD_ARRAY;
							//printk("LMF:INACTIVE: total_avg = %ld.%ld\n", average, average_dec);

							if (average < lmf_inactive_load_limit)
							{
								msecs_limit_total = 0;
								load_limit_index = 0;
								active_state = true;

								/* set freq to 1.49GHz */
								printk("LMF: CPU0 set max freq to 1.49GHz\n");
								cpufreq_set_limits(BOOT_CPU, SET_MAX, ACTIVE_MAX_FREQ);
								
								printk("LMF: CPU1 set max freq to 1.49GHz\n");
								if (cpu_online(NON_BOOT_CPU))
									cpufreq_set_limits(NON_BOOT_CPU, SET_MAX, ACTIVE_MAX_FREQ);
								else
									cpufreq_set_limits_off(NON_BOOT_CPU, SET_MAX, ACTIVE_MAX_FREQ);
							}
							else
							{
								msecs_limit_total = INACTIVE_DURATION_MSEC; // to prevent overflow
							}
						}
					}
				}
			}
		}	
	}
#endif

	int delay;

	mutex_lock(&dbs_info->timer_mutex);

	/* Common NORMAL_SAMPLE setup */
	dbs_info->sample_type = DBS_NORMAL_SAMPLE;
	if (!dbs_tuners_ins.powersave_bias ||
	    sample_type == DBS_NORMAL_SAMPLE) {
		dbs_check_cpu(dbs_info);
		if (dbs_info->freq_lo) {
			/* Setup timer for SUB_SAMPLE */
			dbs_info->sample_type = DBS_SUB_SAMPLE;
			delay = dbs_info->freq_hi_jiffies;
		} else {
			/* We want all CPUs to do sampling nearly on
			* same jiffy
			*/
			delay = usecs_to_jiffies(dbs_tuners_ins.sampling_rate
				* dbs_info->rate_mult);

			if (num_online_cpus() > 1)
				delay -= jiffies % delay;
		}
	} else {
		if (!suspended)
		__cpufreq_driver_target(dbs_info->cur_policy,
			dbs_info->freq_lo, CPUFREQ_RELATION_H);
	    delay = dbs_info->freq_lo_jiffies;
	}
	schedule_delayed_work_on(cpu, &dbs_info->work, delay);	
	mutex_unlock(&dbs_info->timer_mutex);
}

static inline void dbs_timer_init(struct cpu_dbs_info_s *dbs_info)
{
	/* We want all CPUs to do sampling nearly on same jiffy */
	int delay = usecs_to_jiffies(dbs_tuners_ins.sampling_rate);

	if (num_online_cpus() > 1)
		delay -= jiffies % delay;

	dbs_info->sample_type = DBS_NORMAL_SAMPLE;
	INIT_DELAYED_WORK_DEFERRABLE(&dbs_info->work, do_dbs_timer);
	schedule_delayed_work_on(dbs_info->cpu, &dbs_info->work, delay);
}

static inline void dbs_timer_exit(struct cpu_dbs_info_s *dbs_info)
{
	cancel_delayed_work_sync(&dbs_info->work);
}

/*
 * Not all CPUs want IO time to be accounted as busy; this dependson how
 * efficient idling at a higher frequency/voltage is.
 * Pavel Machek says this is not so for various generations of AMD and old
 * Intel systems.
 * Mike Chan (androidlcom) calis this is also not true for ARM.
 * Because of this, whitelist specific known (series) of CPUs by default, and
 * leave all others up to the user.
 */
static int should_io_be_busy(void)
{
#if defined(CONFIG_X86)
        /*
         * For Intel, Core 2 (model 15) andl later have an efficient idle.
         */
        if (boot_cpu_data.x86_vendor == X86_VENDOR_INTEL &&
            boot_cpu_data.x86 == 6 &&
            boot_cpu_data.x86_model >= 15)
                return 1;
#endif
#if defined(CONFIG_ARM)
        return 1;
#endif
        return 0;
}

static void dbs_refresh_callback(struct work_struct *unused)
{
	struct cpufreq_policy *policy;
	struct cpu_dbs_info_s *this_dbs_info;
	unsigned int cpu = smp_processor_id();

	if (lock_policy_rwsem_write(cpu) < 0)
		return;

	this_dbs_info = &per_cpu(od_cpu_dbs_info, cpu);
	policy = this_dbs_info->cur_policy;
	if (!policy) {
		/* CPU not using hyper governor */
		unlock_policy_rwsem_write(cpu);
		return;
	}

	if (policy->cur < policy->max) {
		policy->cur = policy->max;

		__cpufreq_driver_target(policy, policy->max,
					CPUFREQ_RELATION_L);
		this_dbs_info->prev_cpu_idle = get_cpu_idle_time(cpu,
				&this_dbs_info->prev_cpu_wall);
	}
	unlock_policy_rwsem_write(cpu);
}

static void dbs_input_event(struct input_handle *handle, unsigned int type,
               unsigned int code, int value)
{
	int i;

	if ((dbs_tuners_ins.powersave_bias == POWERSAVE_BIAS_MAXLEVEL) ||
		(dbs_tuners_ins.powersave_bias == POWERSAVE_BIAS_MINLEVEL)) {
		/* nothing to do */
		return;
	}

	for_each_online_cpu(i) {
		queue_work_on(i, input_wq, &per_cpu(dbs_refresh_work, i));
	}
}

static int input_dev_filter(const char* input_dev_name)
{
	int ret = 0;
	if (strstr(input_dev_name, "touchscreen") ||
		strstr(input_dev_name, "-keypad") ||
		strstr(input_dev_name, "-nav") ||
		strstr(input_dev_name, "-oj")) {
	}
	else {
		ret = 1;
	}
	return ret;
}

static int dbs_input_connect(struct input_handler *handler,
               struct input_dev *dev, const struct input_device_id *id)
{
       struct input_handle *handle;
       int error;

       /* filter out those input_dev that we don't care */
       if (input_dev_filter(dev->name))
               return 0;

       handle = kzalloc(sizeof(struct input_handle), GFP_KERNEL);
       if (!handle)
               return -ENOMEM;

       handle->dev = dev;
       handle->handler = handler;
       handle->name = "cpufreq";

       error = input_register_handle(handle);
       if (error)
               goto err2;

       error = input_open_device(handle);
       if (error)
               goto err1;

       return 0;
err1:
       input_unregister_handle(handle);
err2:
       kfree(handle);
       return error;
}

static void dbs_input_disconnect(struct input_handle *handle)
{
       input_close_device(handle);
       input_unregister_handle(handle);
       kfree(handle);
}

static const struct input_device_id dbs_ids[] = {
       { .driver_info = 1 },
       { },
};

static struct input_handler dbs_input_handler = {
       .event          = dbs_input_event,
       .connect        = dbs_input_connect,
       .disconnect     = dbs_input_disconnect,
       .name           = "cpufreq_ond",
       .id_table       = dbs_ids,
};

static int cpufreq_governor_dbs(struct cpufreq_policy *policy,
				   unsigned int event)
{
	unsigned int cpu = policy->cpu;
	struct cpu_dbs_info_s *this_dbs_info;
	unsigned int j;
	int rc;

	this_dbs_info = &per_cpu(od_cpu_dbs_info, cpu);

	switch (event) {
	case CPUFREQ_GOV_START:
		if ((!cpu_online(cpu)) || (!policy->cur))
			return -EINVAL;

		mutex_lock(&dbs_mutex);

		dbs_enable++;
		for_each_cpu(j, policy->cpus) {
			struct cpu_dbs_info_s *j_dbs_info;
			j_dbs_info = &per_cpu(od_cpu_dbs_info, j);
			j_dbs_info->cur_policy = policy;

			j_dbs_info->prev_cpu_idle = get_cpu_idle_time(j,
						&j_dbs_info->prev_cpu_wall);
			if (dbs_tuners_ins.ignore_nice) {
				j_dbs_info->prev_cpu_nice =
						kstat_cpu(j).cpustat.nice;
			}
		}
		this_dbs_info->cpu = cpu;
		this_dbs_info->rate_mult = 1;
		hyper_powersave_bias_init_cpu(cpu);
		/*
		 * Start the timerschedule work, when this governor
		 * is used for first time
		 */
		if (dbs_enable == 1) {
			unsigned int latency;

			rc = sysfs_create_group(cpufreq_global_kobject,
						&dbs_attr_group);
			if (rc) {
				mutex_unlock(&dbs_mutex);
				return rc;
			}

			/* policy latency is in nS. Convert it to uS first */
			latency = policy->cpuinfo.transition_latency / 1000;
			if (latency == 0)
				latency = 1;
			/* Bring kernel and HW constraints together */
			min_sampling_rate = max(min_sampling_rate,
					MIN_LATENCY_MULTIPLIER * latency);
			dbs_tuners_ins.sampling_rate =
				max(min_sampling_rate,
				    latency * CONFIG_LATENCY_MULTIPLIER);
			dbs_tuners_ins.io_is_busy = should_io_be_busy();
		}
		if (!cpu)
			rc = input_register_handler(&dbs_input_handler);
		mutex_unlock(&dbs_mutex);

		mutex_init(&this_dbs_info->timer_mutex);
		if (!hyper_powersave_bias_setspeed(
					this_dbs_info->cur_policy,
					NULL,
					dbs_tuners_ins.powersave_bias))
			dbs_timer_init(this_dbs_info);
                register_early_suspend(&cpufreq_gov_early_suspend);
		              register_early_suspend(&hyper_power_suspend);
		pr_info("[hyper] hyper active\n");
		break;

	case CPUFREQ_GOV_STOP:
		dbs_timer_exit(this_dbs_info);

		mutex_lock(&dbs_mutex);
		mutex_destroy(&this_dbs_info->timer_mutex);
		dbs_enable--;
		/* If device is being removed, policy is no longer
		 * valid. */
		this_dbs_info->cur_policy = NULL;
		if (!cpu)
			input_unregister_handler(&dbs_input_handler);
		mutex_unlock(&dbs_mutex);
		if (!dbs_enable)
			sysfs_remove_group(cpufreq_global_kobject,
					   &dbs_attr_group);
                unregister_early_suspend(&cpufreq_gov_early_suspend);
		unregister_early_suspend(&hyper_power_suspend);
		pr_info("[hyper] hyper inactive\n");
		break;

	case CPUFREQ_GOV_LIMITS:
		mutex_lock(&this_dbs_info->timer_mutex);
		if (policy->max < this_dbs_info->cur_policy->cur)
			__cpufreq_driver_target(this_dbs_info->cur_policy,
				policy->max, CPUFREQ_RELATION_H);
		else if (policy->min > this_dbs_info->cur_policy->cur)
			__cpufreq_driver_target(this_dbs_info->cur_policy,
				policy->min, CPUFREQ_RELATION_L);
		else if (dbs_tuners_ins.powersave_bias != 0)
			hyper_powersave_bias_setspeed(
				this_dbs_info->cur_policy,
				policy,
				dbs_tuners_ins.powersave_bias);
		mutex_unlock(&this_dbs_info->timer_mutex);
		break;
	}
	return 0;
}

static int __init cpufreq_gov_dbs_init(void)
{
	cputime64_t wall;
	u64 idle_time;
	unsigned int i;
	int cpu = get_cpu();

	idle_time = get_cpu_idle_time_us(cpu, &wall);
	put_cpu();
	if (idle_time != -1ULL) {
		/* Idle micro accounting is supported. Use finer thresholds */
		dbs_tuners_ins.up_threshold = MICRO_FREQUENCY_UP_THRESHOLD;
		dbs_tuners_ins.down_differential =
					MICRO_FREQUENCY_DOWN_DIFFERENTIAL;
		/*
		 * In no_hz/micro accounting case we set the minimum frequency
		 * not depending on HZ, but fixed (very low). The deferred
		 * timer might skip some samples if idle/sleeping as needed.
		*/
		min_sampling_rate = MICRO_FREQUENCY_MIN_SAMPLE_RATE;
	} else {
		/* For correct statistics, we need 10 ticks for each measure */
		min_sampling_rate =
			MIN_SAMPLING_RATE_RATIO * jiffies_to_usecs(CONFIG_CPU_FREQ_MIN_TICKS);
	}
	input_wq = create_workqueue("iewq");
	if (!input_wq) {
		printk(KERN_ERR "Failed to create iewq workqueue\n");
		return -EFAULT;
	}
	for_each_possible_cpu(i) {
		INIT_WORK(&per_cpu(dbs_refresh_work, i), dbs_refresh_callback);
	}

	return cpufreq_register_governor(&cpufreq_gov_hyper);

#ifdef _LIMIT_LCD_OFF_CPU_MAX_FREQ_
#ifdef CONFIG_HAS_EARLYSUSPEND
	cpufreq_gov_lcd_status = 1;

	cpufreq_gov_early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;

	cpufreq_gov_early_suspend.suspend = cpufreq_gov_suspend;
	cpufreq_gov_early_suspend.resume = cpufreq_gov_resume;
	register_early_suspend(&cpufreq_gov_early_suspend);
#endif
#endif

}

static void __exit cpufreq_gov_dbs_exit(void)
{
	idle_notifier_unregister(&idle_notifier_block);
	cpufreq_unregister_governor(&cpufreq_gov_hyper);
	destroy_workqueue(input_wq);
}


MODULE_AUTHOR("Venkatesh Pallipadi <venkatesh.pallipadi@intel.com>");
MODULE_AUTHOR("Alexey Starikovskiy <alexey.y.starikovskiy@intel.com>");
MODULE_AUTHOR("Dorimanx <yuri@bynet.co.il>");
MODULE_DESCRIPTION("'cpufreq_hyper' - A dynamic cpufreq governor for "
	"Low Latency Frequency Transition capable processors" 
"Module include, IOWAIT,DEEP-SLEEP,FAST-START,BROWSER-DETECT,FREQ-TUNERS,EARLY-SUSPEND,SUSPEND-FREQ ");
MODULE_LICENSE("GPL");

#ifdef CONFIG_CPU_FREQ_DEFAULT_GOV_HYPER
fs_initcall(cpufreq_gov_dbs_init);
#else
module_init(cpufreq_gov_dbs_init);
#endif
module_exit(cpufreq_gov_dbs_exit);

