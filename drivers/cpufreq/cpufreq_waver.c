#include <linux/cpu.h>
#include <linux/cpumask.h>
#include <linux/cpufreq.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/rwsem.h>
#include <linux/sched.h>
#include <linux/sched/rt.h>
#include <linux/tick.h>
#include <linux/time.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/kthread.h>
#include <linux/slab.h>
#include <linux/kernel_stat.h>
#include <asm/cputime.h>

#include "cpufreq_waver.h"

static int lcluster_cores;
static int bcluster_cores;
static int lcluster_start;
static int bcluster_start;

static int waver_mask = 0;
module_param_named(debug_mask, waver_mask, int, S_IRUSR | S_IWUSR);

struct frqinfo info = {
	.sm = FREEZE_STATE,
	.sw = FLOOR_WAVE,
	.floor_freq = 0,
	.ceili_freq = 0,
	.start_in_time = 0,
	.last_policy = NOTHING,
#ifdef DEBUG
	.en = 1,
	.enforce = 0,
#endif
};

struct cpu_load_info cpu_load;

struct frequency_band band_freq[]= {
	{
		 /* little */
		.turbo        = 1248000,
		.nominal      = 600000,
		.semi_nominal = 787200,
		.svs          = 384000,
		.effi         = 600000,
		.perf         = 1440000,
		.freeze       = 600000
	},
	{
		/* big */
		.turbo        = 864000,
		.nominal      = 600000,
		.svs          = 480000,
		.effi         = 768000,
		.perf         = 1824000,
		.freeze       = 480000
	},
	{
		/* gpu */
		.turbo        = 600000000,
		.nominal      = 450000000,
		.effi         = 300000000,
		.svs          = 180000000,
	}
};

static int start_policy(struct frqinfo *info);
static DEFINE_PER_CPU(struct cpu_load_info, cpuload);
extern int gpu_power_level;

#ifdef DEBUG
static void create_debug_fs(void);

static int param_lockin = false;

static void param_lock(void)
{
	param_lockin = true;
	info.sm = FREEZE_STATE;
}

static void param_unlock(void)
{
	info.sm = IDLE_STATE;
	param_lockin = false;
}
#endif

static void decide_post_pause_resume(struct frqinfo *info)
{
	int req_freq = (info->cur_policy == SEMI_TURBO) ?
			info->last_l_freq : info->last_b_freq;

	if (info->cur_policy == SEMI_TURBO) {
		if (req_freq < band_freq[CLUSTER_LITTLE].freeze) {
			t_debug(WAVER_PAUSE_RESUME, ".... LITTLE pause state : %d - %d \n",
					info->last_l_freq, req_freq);
			info->sm = PAUSE_STATE;
			return;
		}

		if (req_freq >= band_freq[CLUSTER_LITTLE].effi &&
			info->sm == PAUSE_STATE) {
			t_debug(WAVER_PAUSE_RESUME, ".... LITTLE resume state : %d - %d \n",
					info->last_l_freq, req_freq);
			info->sm = RESUME_STATE;
			return;
		}
	} else if (info->cur_policy == SUPER_TURBO) {
		if (req_freq < band_freq[CLUSTER_BIG].freeze) {
			t_debug(WAVER_PAUSE_RESUME, ".... BIG pause state : %d - %d \n",
					info->last_b_freq, req_freq);
			info->sm = PAUSE_STATE;
			return;
		}

		if (req_freq >= band_freq[CLUSTER_BIG].effi &&
			info->sm == PAUSE_STATE) {
			t_debug(WAVER_PAUSE_RESUME, ".... BIG resume STATE : %d - %d \n",
					info->last_l_freq, req_freq);
			info->sm = RESUME_STATE;
			return;
		}
	} else {
		/* relase pause IMPORTANT!
		 * pause state only has semi_turbo and super_turbo
		 * else (nominal etc) it has to be release pause mode */
		if(info->sm == PAUSE_STATE)
			info->sm = RESUME_STATE;

	}
	return;
}

static int gov_online = 0;
#ifdef DEBUG
static int cur_cpu = 0;
#endif
static int decide_dst_cpu(enum  state_policy  cur_policy)
{
#define INVALID (-1)
	struct cpu_load_info *pcpu;
	int cpu;
	int dst_cpu = INVALID;

	/* SUPER TURBO only to BIG cores */
	if(cur_policy == SUPER_TURBO) {
		for_each_online_cpu(cpu) {
			pcpu = &per_cpu(cpuload, cpu);
			if(cpu >= bcluster_start && pcpu->online) {
				dst_cpu = cpu;
			}
		}
	} else {
		/* dst_cpu is always 0 at little policy */
		dst_cpu = lcluster_start;
	}
	/* dst_cpu == -1
	  * whole big cores are in hotplug
	  * policy should be freeze
	  */
#ifdef DEBUG
	cur_cpu = dst_cpu;
#endif
	return dst_cpu;
}

static int cpu_set_freq(int freq, int cpu)
{
	int relation = CPUFREQ_RELATION_L;
	struct cpu_load_info *pcpu;

	if(cpu < 0)
		return -EINVAL;

	pcpu = &per_cpu(cpuload, cpu);

	if(!pcpu->online)
		return -EINVAL;

	relation =  (freq > pcpu->policy->cur) ? CPUFREQ_RELATION_H:
									CPUFREQ_RELATION_L;
	__cpufreq_driver_target(pcpu->policy, freq, relation);
	return 0;
}
static void target_frequency_work(struct work_struct *work)
{
	struct frqinfo *info = container_of(work,
				struct frqinfo,
				delayed_freq_worker.work);
	int cpu;
	/* make cpufreq wave btw determined floor to ceiling freq */
	if (info->sm == WAVE_STATE) {

		cpu = decide_dst_cpu(info->cur_policy);

		if (info->sw == PERF_WAVE) {
			if (!cpu_set_freq(info->floor_freq, cpu))
				info->sw = FLOOR_WAVE;
			else	goto out;

		} else if (info->sw == FLOOR_WAVE) {
			if (!cpu_set_freq(info->ceili_freq, cpu))
				info->sw = PERF_WAVE;
			else	goto out;
		}
		queue_delayed_work_on(0, info->frequency_work,
					&info->delayed_freq_worker,
					usecs_to_jiffies(20 * USEC_PER_MSEC));

	} else if (info->sm == FREEZE_STATE) {

		t_debug(WAVER_PAUSE_RESUME, ".... FREEZE POLICY ....\n");
		info->last_policy = info->cur_policy = NOTHING;

	} else {
		/* nothing to do */
	}
	return;
out:
	info->sm = FREEZE_STATE;
	return;
}
static int update_frequencies(struct frqinfo *info)
{
	int ret = 0;

#ifdef DEBUG
	if (param_lockin) {
		pr_err("setting new param\n");
		return ret;
	}
#endif
	switch (info->cur_policy) {
	case SUPER_TURBO:
		info->ceili_freq = info->last_b_freq;
		info->floor_freq = (info->last_b_freq > band_freq[CLUSTER_BIG].effi) ?
					band_freq[CLUSTER_BIG].effi : band_freq[CLUSTER_BIG].svs;
		break;
	case SEMI_TURBO:
		info->ceili_freq = (info->last_l_freq > band_freq[CLUSTER_LITTLE].effi) ?
					info->last_l_freq : band_freq[CLUSTER_LITTLE].effi;
		info->floor_freq = (info->last_l_freq >= band_freq[CLUSTER_LITTLE].effi) ?
			band_freq[CLUSTER_LITTLE].effi : band_freq[CLUSTER_LITTLE].svs;
		break;
	case SEMI_NOMINAL:
		info->ceili_freq = band_freq[CLUSTER_LITTLE].svs;
		info->floor_freq = band_freq[CLUSTER_LITTLE].svs;
		break;
	case NOMINAL:
		info->ceili_freq = (info->last_l_freq >= band_freq[CLUSTER_LITTLE].effi) ?
					info->last_l_freq : band_freq[CLUSTER_LITTLE].svs;
		/* for performance */
		info->floor_freq = (info->last_l_freq >= band_freq[CLUSTER_LITTLE].effi) ?
			band_freq[CLUSTER_LITTLE].effi : band_freq[CLUSTER_LITTLE].svs;
		/* for power */
		//to->floor_freq = band_freq[CLUSTER_LITTLE].svs;
		break;
	default:
		info->sm = FREEZE_STATE;
		ret = 1;
		break;
	}
	return ret;
}
static void update_state_machine(struct frqinfo *info, bool b)
{
	if(info->sm != WAVE_STATE)
		return;
	update_frequencies(info);
#ifdef DEBUG
	if(!b)
		t_debug(WAVER_TRACE, " [UPDATE] PERF:%d FLOOR:%d POL:%d dst_cpu:%d src_cpu: %d\n",
				info->ceili_freq,
				info->floor_freq,
				info->cur_policy,
				cur_cpu,
				smp_processor_id());
#endif
	return;
}


static void get_max_cluster_load(struct cpu_load_info *info,

	int *little_load, int *big_load)
{
	int i;
	int tlload = 0, tbload = 0;
	struct cpu_load_info *p;

	for_each_online_cpu(i) {
		p = &per_cpu(cpuload, i);
		if(i < lcluster_cores) {
			/* little */
			p->cpu_t_load = p->cpu_t_load / DIVIDE_ZERO(p->cpu_t_load_count);
			if(p->cpu_t_load > tlload)
				tlload = p->cpu_t_load;
		} else {
			/* big */
			p->cpu_t_load = p->cpu_t_load /DIVIDE_ZERO(p->cpu_t_load_count);
			if(p->cpu_t_load > tbload)
				tbload = p->cpu_t_load;
		}
	}
	*little_load = tlload;
	*big_load = tbload;
}
static void clear_all_cpu_load(struct cpu_load_info *info)
{
	int i;
	struct cpu_load_info *p;
	for_each_online_cpu(i) {
		p = &per_cpu(cpuload, i);
		p->cpu_t_load = 0;
		p->cpu_t_load_count = 0;
	}
}

static int start_policy(struct frqinfo *info)
{
	if (info->sm == PAUSE_STATE) {
		t_debug(WAVER_PAUSE_RESUME, "... it's pause state. ignor...!... : \n");
		goto out;
	}

	info->sw = PERF_WAVE;

	if(update_frequencies(info)) {
		goto out;
	}

	if (info->last_policy == NOTHING ||
		info->sm == FREEZE_STATE ||
		info->sm == RESUME_STATE) {
		/* in order to trigger once */
		info->sm = WAVE_STATE;
		queue_delayed_work_on(0, info->frequency_work,
				&info->delayed_freq_worker,
				usecs_to_jiffies(1000));
	}

out:
	info->last_policy = info->cur_policy;
	return 0;
}

#define NO_LOAD       (0)
#define HIGH_LOAD     (100)
#define MAX_PORTION   (HIGH_LOAD * 100)

static int decide_post_policy(struct frqinfo *info,
	int bscore, int lscore)
{
	/* below condition is pretty indistinct */
	if(bscore <= 20 && info->cur_policy == SUPER_TURBO) {
		/* the load of big core goes to low band...
		    but little core in highband
		    */
		if(info->turbo_portion[CLUSTER_LITTLE] == 10000) {
			goto skip;
		}
	}
	/* stop condition commonly */
	/* turbo stop condition */
	else if (bscore == NO_LOAD &&info->cur_policy == SUPER_TURBO)
		goto out;
	/* nominal stop condition */
	else if ((lscore < 50 && gpu_power_level <= band_freq[BIT_GPU].effi) &&
		info->cur_policy == NOMINAL)
		goto out;
	else
		return 0;
out:
	info->cur_policy = NOTHING;
	info->sm = FREEZE_STATE;
skip:
	info->cur_policy = SEMI_TURBO;
	return 1;
}

static void decide_policy(struct frqinfo *info)
{
	int lscore, bscore;
	int lload, bload;
	get_max_cluster_load(&cpu_load, &lload, &bload);

	lscore = info->turbo_portion[CLUSTER_LITTLE] / lload;
	bscore = info->turbo_portion[CLUSTER_BIG] / bload;

	/* pre decide policy */
	if ((info->turbo_portion[CLUSTER_LITTLE] == MAX_PORTION
				&& (lload <= 8))
		|| (info->turbo_portion[CLUSTER_BIG] == MAX_PORTION
				&& (bload <= 4))) {
		info->cur_policy = NOTHING;
		clear_all_cpu_load(&cpu_load);
		t_debug(WAVER_POLICY, "... Not Allowed Policy ...\n");
		return;
	}

	if (bscore == 0 && lload <= 50 && info->turbo_portion[CLUSTER_LITTLE] <= 3000) {
		info->cur_policy = NOTHING;
		clear_all_cpu_load(&cpu_load);
		t_debug(WAVER_POLICY, "... low bypass ...\n");
		return;
	}

	/* determine cur_policy [ORDER DEPENDENCY]
	 *
	 * Policy set      big               little            gpu             pre_policy
	 *
	 * SEMI_TURBO      0                 150 >= x >= 100   undr eff(300)   D/C
	 * SUPER_TURBO     150 >= x  >70     D/C               over nor(450)   D/C
	 * NOMINAL         0                 150 >  x >= 50    over eff(300)   D/C
	 *
	 */

	if (bscore == NO_LOAD &&
		((lscore >= HIGH_LOAD && lscore <= 150) ||
		info->turbo_portion[CLUSTER_LITTLE] == 10000) &&
		gpu_power_level < band_freq[BIT_GPU].effi) {
		info->cur_policy = SEMI_TURBO;
	} else if ((bscore > 70  && bscore <= 150) &&
		gpu_power_level >= band_freq[BIT_GPU].nominal) {
		info->cur_policy = SUPER_TURBO;
	} else if (bscore == NO_LOAD && (lscore >= 50 &&
		lscore < 150) &&
		gpu_power_level > band_freq[BIT_GPU].effi) {
		info->cur_policy = NOMINAL;
	}
#ifdef DEBUG
	if(info->enforce > 0) {
		info->cur_policy = info->enforce;
	}
#endif
	/* post decide policy */
	decide_post_policy(info, bscore, lscore);
	decide_post_pause_resume(info);

	info->last_beffi = bscore;
	info->last_leffi = lscore;
	clear_all_cpu_load(&cpu_load);
#ifdef DEBUG
	if(info->en)
#endif

	t_debug(WAVER_DUMP_LOG, ".... bt-p:%d lt-p:%d n-p:%d s-p:%d l-l:%d b-l:%d\n",
	info->turbo_portion[CLUSTER_BIG],
	info->turbo_portion[CLUSTER_LITTLE],
	info->nom_portion[CLUSTER_LITTLE],
	info->svs_portion[CLUSTER_LITTLE],
	lload, bload);
	t_debug(WAVER_DUMP_LOG, ".... b-s:%d  l-s:%d gpu:%d l-f:%d b-f:%d sm:%d\n",
	bscore, lscore,
	gpu_power_level,
	info->last_l_freq,
	info->last_b_freq,
	info->sm);
	 start_policy(info);
	return;
}
static void update_params(struct frqinfo *info, u32 curr_time)
{
	int temp = curr_time - info->start_in_time;
	int total_portion ;
	int i ;
#define MSEC_TO_SEC 1000
#define SECOND_DIGIT 10000
	for (i = 0; i < MAX_CLUSTER; i++) {
		info->turbo_portion[i] = (info->turbo_time[i] * MSEC_TO_SEC)
			/ DIVIDE_ZERO(temp);
		info->nom_portion[i] = (info->nom_time[i] * MSEC_TO_SEC)
			/ DIVIDE_ZERO(temp);
		info->svs_portion[i] = (info->svs_time[i] * MSEC_TO_SEC)
			/ DIVIDE_ZERO(temp);

		total_portion =  info->turbo_portion[i]
			+ info->nom_portion[i] + info->svs_portion[i];

		info->turbo_portion[i] = (SECOND_DIGIT * info->turbo_portion[i])
			/ DIVIDE_ZERO(total_portion);
		info->nom_portion[i] = (SECOND_DIGIT * info->nom_portion[i])
			/ DIVIDE_ZERO(total_portion);
		info->svs_portion[i] = (SECOND_DIGIT * info->svs_portion[i])
			/ DIVIDE_ZERO(total_portion);
	}
}

static void clear_time_param(struct frqinfo *info, u32 curr)
{
	int i = 0;
		info->start_in_time = curr;
		for (i = 0; i < MAX_CLUSTER; i++) {
			info->turbo_time[i] = 0;
			info->nom_time[i] = 0;
			info->svs_time[i] = 0;
		}
}

#ifdef DEBUG
static void dump_log(struct frqinfo *info)
{
	t_debug(WAVER_DUMP_LOG, "\n (WAVING LITTLE) Turbo[%dms %d%%]  Nom[%dms %d%%]  Svs[%dms %d%%] \n",
					info->turbo_time[CLUSTER_LITTLE], info->turbo_portion[CLUSTER_LITTLE],
					info->nom_time[CLUSTER_LITTLE],info->nom_portion[CLUSTER_LITTLE],
					info->svs_time[CLUSTER_LITTLE],info->svs_portion[CLUSTER_LITTLE]);

	t_debug(WAVER_DUMP_LOG, "\n (WAVING BIG   ) Turbo[%dms %d%%]  Nom[%dms %d%%]  Svs[%dms %d%%] \n",
					info->turbo_time[CLUSTER_BIG], info->turbo_portion[CLUSTER_BIG],
					info->nom_time[CLUSTER_BIG],info->nom_portion[CLUSTER_BIG],
					info->svs_time[CLUSTER_BIG],info->svs_portion[CLUSTER_BIG]);
}
#endif
static void update_cpu_online(bool gov)
{
	int i, ret;
	struct cpufreq_policy policy;
	if(!gov) {
		get_online_cpus();
		for_each_online_cpu(i) {
			ret = cpufreq_get_policy(&policy, i);
			if (ret)
				continue;
			ret = cpufreq_update_policy(i);
		}
		put_online_cpus();
	}
}
void _update_online_state(int online, struct cpufreq_policy *policy)
{
	unsigned int i;
	struct cpu_load_info *pcpu;
	unsigned long flags;
	if(online)
		gov_online = online;
	spin_lock_irqsave(&info.policy_lock, flags);
	for_each_cpu(i, policy->cpus) {
		pcpu = &per_cpu(cpuload, i);
		pcpu->policy = policy;
		pcpu->online = online;
	}
	info.sm = FREEZE_STATE;
	spin_unlock_irqrestore(&info.policy_lock, flags);
	cancel_delayed_work_sync(&info.delayed_freq_worker);
}
void _update_cpu_load(int cpu, int freq, int load)
{
	struct cpu_load_info *p;
	p = &per_cpu(cpuload , cpu);
	if(cpu < lcluster_cores) {
		if(freq >= band_freq[CLUSTER_LITTLE].turbo) {
			p->cpu_t_load += load;
			++(p->cpu_t_load_count);
		}
	} else {
		if(freq >= band_freq[CLUSTER_BIG].nominal) {
			p->cpu_t_load += load;
			++(p->cpu_t_load_count);
		}
	}
}

static void update_frequency_portion(struct frqinfo *info, bool gov)
{
	u32 curr_time = (u32)ktime_to_ms(ktime_get());
	u32 delta ;
	int elapsed;
	int i, *c ;

	/*
	  * gov is false
	  * Invoked by workQ
	  * both of little and big data must be updated
	  */
	if(info->start_in_time == 0)
		clear_time_param(info, curr_time);

	delta = curr_time - info->last_in_time;

	for (i = 0; i < MAX_CLUSTER; i++) {
		c = (i == 0) ? &info->last_l_freq : &info->last_b_freq;
		if (*c >= band_freq[i].turbo)
			info->turbo_time[i] += delta;
		else if (*c >= band_freq[i].nominal &&
				*c < band_freq[i].turbo) {
			info->nom_time[i] += delta;
		}
		else {
			info->svs_time[i] += delta;
		}
	}

	info->last_in_time = curr_time;
	elapsed = curr_time - info->start_in_time;
	/* every 2 seconds */
	if(elapsed > info->trigger_intv ) {
		update_params(info, curr_time);
#ifdef DEBUG
		dump_log(info);
#endif
		decide_policy(info);
		info->start_in_time = 0;
	}
	return;
}

static void stack_normalize(struct work_struct *work)
{

	struct frqinfo *info = container_of(work,
			struct frqinfo, delayed_freq_normalizer.work);
	update_cpu_online(false);
	update_frequency_portion(info, false);
	update_state_machine(info, false);
#ifdef DEBUG
	if(info->en)
#endif
	queue_delayed_work_on(0, info->freq_mon,
			&info->delayed_freq_normalizer,
			usecs_to_jiffies(500 * 1000));

	return;
}

static int check_cluster(int cpu)
{
	if (lcluster_cores != 0 && bcluster_cores != 0) {
		if (cpu < lcluster_cores)
			return CLUSTER_LITTLE;
		else if (cpu < (lcluster_cores + bcluster_cores))
			return CLUSTER_BIG;
		else
			pr_err("%s : cannot detect cpu %d's cluster\n",
					__func__, cpu);
	}

	pr_err("%s : lcluster, bcluster is not set\n", __func__);
	return -EINVAL;
}

static int stack(int cpu, int freq)
{
	info.cluster = check_cluster(cpu);

	if (!info.cluster)
		info.last_l_freq = freq;
	else
		info.last_b_freq = freq;
	update_frequency_portion(&info, true);
	return 0;
}


static int cpufreq_notifier(
	struct notifier_block *nb, unsigned long val, void *data)
{
	struct cpufreq_freqs *freq = data;

	if (val == CPUFREQ_POSTCHANGE) {
		if (!info.gov_init) {
			if(gov_online) {
				info.gov_init = !!true;
				queue_delayed_work_on(0, info.freq_mon,
						&info.delayed_freq_normalizer,
						usecs_to_jiffies(2000 * 1000));
			}
		} else {
			stack(freq->cpu, freq->new);
		}
	}
	return 0;
}

/*
 * governor policy was change by boost somethings
 * minimum frequency has been changed
 */
static int policy_notifier(struct notifier_block *nb,
		unsigned long val, void *data)
{
	switch (val) {
	case CPUFREQ_ADJUST:
		break;
	}
	return 0;
}

static struct notifier_block cpufreq_notifier_block = {
	.notifier_call = cpufreq_notifier,
};

static struct notifier_block policy_notifier_block = {
	.notifier_call = policy_notifier,
};

static int __init ptp_init(void)
{
	int i, ret = 0;
	struct cpu_load_info *pcpu;
	int csiblings[MAX_CPUS_DEFLT] = {-1,};

	/* get cpu core count */
	pr_err("MAX_CPUs : %d\n", MAX_CPUS);

	for (i = 0; i < MAX_CPUS; i++) {
		csiblings[i]= topology_physical_package_id(i);
	}

	for (i = 0; i < MAX_CPUS; i++) {
		pr_err("csiblings[%d] = 0x%x\n", i, csiblings[i]);
		if (csiblings[i] == CLUSTER_LITTLE)
			lcluster_cores++;
		else if (csiblings[i] == CLUSTER_BIG)
			bcluster_cores++;
		else
			pr_err("csiblings not defined\n");
	}

	lcluster_start = 0;
	bcluster_start = lcluster_cores;

	pr_err("lcluster_cores : %d, bcluster_cores : %d\n",
			lcluster_cores, bcluster_cores);

	info.frequency_work = alloc_workqueue("little_wq", WQ_HIGHPRI, 0);
	info.freq_mon = alloc_workqueue("freq_mon", WQ_HIGHPRI, 0);
	info.gov_init = false;

	/* tringgering every 2seconds */
	info.trigger_intv = 2000;
	INIT_DELAYED_WORK(&info.delayed_freq_worker, target_frequency_work);
	INIT_DELAYED_WORK(&info.delayed_freq_normalizer, stack_normalize);

	mutex_init(&info.freq_lock);
	spin_lock_init(&info.policy_lock);
	for_each_online_cpu(i) {
		pcpu = &per_cpu(cpuload, i);
		pcpu->cpu_t_load = 0;
		pcpu->cpu_t_load_count = 0;
		init_rwsem(&pcpu->sem);
	}
	ret = cpufreq_register_notifier(&cpufreq_notifier_block,
			CPUFREQ_TRANSITION_NOTIFIER);
	if (ret) {

	}

	ret = cpufreq_register_notifier(&policy_notifier_block,
			CPUFREQ_POLICY_NOTIFIER);
	if (ret) {

	}
#ifdef DEBUG
        create_debug_fs();
#endif

	return 0;
}

static void __exit ptp_exit(void)
{
	destroy_workqueue(info.frequency_work);
	destroy_workqueue(info.freq_mon);

	cpufreq_unregister_notifier(&cpufreq_notifier_block,
			CPUFREQ_TRANSITION_NOTIFIER);
	cpufreq_unregister_notifier(&policy_notifier_block,
			CPUFREQ_POLICY_NOTIFIER);
}

/*=====================================================================
 * debug fs
 =======================================================================*/
#ifdef DEBUG
#define show_one(file_name, object)                                     \
	static ssize_t show_##file_name                                 \
	(struct kobject *kobj, struct attribute *attr, char *buf)       \
	{                                                               \
		return sprintf(buf, "%u\n", info.object);               \
	}
static ssize_t store_cur_policy(struct kobject *a, struct attribute *b,
				const char *buf, size_t count)
{
	return count;
}

static ssize_t store_en(struct kobject *a, struct attribute *b,
				const char *buf, size_t count)
{
	int input;
	int ret;

	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;

	mutex_lock(&info.freq_lock);
	info.en = input;
	info.last_policy = info.cur_policy = NOTHING;
	info.sm = FREEZE_STATE;
	mutex_unlock(&info.freq_lock);

	if (!info.en) {
		cancel_delayed_work(&info.delayed_freq_normalizer);
		cancel_delayed_work(&info.delayed_freq_worker);
	} else {
		queue_delayed_work_on(0, info.freq_mon,
				&info.delayed_freq_normalizer,
				usecs_to_jiffies(2000 * 1000));
	}
	return count;
}

static ssize_t store_enforce(struct kobject *a, struct attribute *b,
				const char *buf, size_t count)
{
	int input;
	int ret;

	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;

	info.enforce = input;
	return count;
}

static ssize_t store_trigger_intv(struct kobject *a, struct attribute *b,
				const char *buf, size_t count)
{
	int input;
	int ret;
	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;
	info.trigger_intv = input;
	return count;
}

show_one(cur_policy, cur_policy);
show_one(en, en);
show_one(enforce, enforce);
show_one(trigger_intv, trigger_intv);
define_one_global_rw(cur_policy);
define_one_global_rw(en);
define_one_global_rw(enforce);
define_one_global_rw(trigger_intv);

static ssize_t show_band_cluster(struct kobject *kobj,
		struct attribute *attr, char *buf)
{
	int i  = 0;
	pr_err("=========================================================\n");
	for ( i = 0; i < MAX_CLUSTER; i++) {
		pr_err("band_freq[%d].turbo         = %d\n", i, band_freq[i].turbo);
		pr_err("band_freq[%d].nominal       = %d\n", i, band_freq[i].nominal);
		pr_err("band_freq[%d].semi_nominal  = %d\n", i, band_freq[i].semi_nominal);
		pr_err("band_freq[%d].svs           = %d\n", i, band_freq[i].svs);
		pr_err("band_freq[%d].effi          = %d\n", i, band_freq[i].effi);
		pr_err("band_freq[%d].perf          = %d\n", i, band_freq[i].perf);
		pr_err("band_freq[%d].freeze        = %d\n", i, band_freq[i].freeze);
		pr_err("=========================================================\n");
	}
	pr_err("gpu.turbo                  = %d\n", band_freq[BIT_GPU].turbo);
	pr_err("gpu.nominal                = %d\n", band_freq[BIT_GPU].nominal);
	pr_err("gpu.effi                   = %d\n", band_freq[BIT_GPU].effi);
	pr_err("gpu.svs                    = %d\n", band_freq[BIT_GPU].svs);
	pr_err("=========================================================\n");
	return 1;
}

static ssize_t show_band_gpu(struct kobject *kobj,
		struct attribute *attr, char *buf)
{
	int i = 0;
	pr_err("=========================================================\n");
	for ( i = 0; i < MAX_CLUSTER; i++) {
		pr_err("band_freq[%d].turbo         = %d\n", i, band_freq[i].turbo);
		pr_err("band_freq[%d].nominal       = %d\n", i, band_freq[i].nominal);
		pr_err("band_freq[%d].semi_nominal  = %d\n", i, band_freq[i].semi_nominal);
		pr_err("band_freq[%d].svs           = %d\n", i, band_freq[i].svs);
		pr_err("band_freq[%d].effi          = %d\n", i, band_freq[i].effi);
		pr_err("band_freq[%d].perf          = %d\n", i, band_freq[i].perf);
		pr_err("band_freq[%d].freeze        = %d\n", i, band_freq[i].freeze);
		pr_err("=========================================================\n");
	}
	pr_err("gpu.turbo                  = %d\n", band_freq[BIT_GPU].turbo);
	pr_err("gpu.nominal                = %d\n", band_freq[BIT_GPU].nominal);
	pr_err("gpu.effi                   = %d\n", band_freq[BIT_GPU].effi);
	pr_err("gpu.svs                    = %d\n", band_freq[BIT_GPU].svs);
	pr_err("=========================================================\n");
	return 1;
}

static ssize_t store_band_cluster(struct kobject *a, struct attribute *b,
				const char *buf, size_t count)
{

	struct frequency_band set;
	int ret, idx = 0;

	param_lock();

	ret = sscanf(buf, "%d %d %d %d %d %d %d %d",
			&idx,
			&set.turbo, &set.nominal, &set.semi_nominal,
			&set.svs, &set.effi, &set.perf, &set.freeze);

	if (idx < CLUSTER_LITTLE || idx > CLUSTER_BIG) {
		pr_err("cannot set band_freq CPU, over range %d\n", idx);
		param_unlock();
		return count;
	}

	band_freq[idx].turbo         = set.turbo;
	band_freq[idx].nominal       = set.nominal;
	band_freq[idx].semi_nominal  = set.semi_nominal;
	band_freq[idx].svs           = set.svs;
	band_freq[idx].effi          = set.effi;
	band_freq[idx].perf          = set.perf;
	band_freq[idx].freeze        = set.freeze;

	param_unlock();
	return count;
}

static ssize_t store_band_gpu(struct kobject *a, struct attribute *b,
				const char *buf, size_t count)
{

	struct frequency_band set;
	int ret, idx = 0;

	param_lock();

	ret = sscanf(buf, "%d %d %d %d %d",
			&idx, &set.turbo, &set.nominal, &set.effi, &set.svs);

	if (idx != BIT_GPU) {
		pr_err("cannot set band_freq GPU, over range %d\n", idx);
		param_unlock();
		return count;
	}

	band_freq[BIT_GPU].turbo         = set.turbo;
	band_freq[BIT_GPU].nominal       = set.nominal;
	band_freq[BIT_GPU].effi          = set.effi;
	band_freq[BIT_GPU].svs           = set.svs;

	param_unlock();
	return count;
}

static struct global_attr band_cluster =
	__ATTR(band_cluster, 0666, show_band_cluster, store_band_cluster);

static struct global_attr band_gpu =
	__ATTR(band_gpu, 0666, show_band_gpu, store_band_gpu);

static struct attribute *waver_attributes[] = {
	&cur_policy.attr,
	&en.attr,
	&enforce.attr,
	&trigger_intv.attr,
	&band_cluster.attr,
	&band_gpu.attr,
	NULL
};

static struct attribute_group attr_group = {
	.attrs = waver_attributes,
};

static struct kobject *intel_pstate_kobject;

static void create_debug_fs(void)
{
	int rc;

	intel_pstate_kobject = kobject_create_and_add("waver",
			&cpu_subsys.dev_root->kobj);
	BUG_ON(!intel_pstate_kobject);
	rc = sysfs_create_group(intel_pstate_kobject,
		&attr_group);
	BUG_ON(rc);
}
#endif

fs_initcall(ptp_init);
module_exit(ptp_exit);
