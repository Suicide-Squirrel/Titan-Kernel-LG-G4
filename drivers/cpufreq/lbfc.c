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

#include "lbfc.h"

static int lcluster_cores;
static int bcluster_cores;
static int lcluster_start;
static int bcluster_start;
/* defulat debug mask */
static int _mask = 1<<1;
#define DIVIDE(x,y) (x/(( (y == 0) ? 1 : y )))
module_param_named(debug_mask, _mask, int, S_IRUSR | S_IWUSR);

struct frqinfo info = {
	.sm = FREEZE_STATE,
	.sw = FLOOR,
	.floor = 0,
	.ceili = 0,
	.sitime = 0,
	.last_policy = NOTHING,
	.cm = 0,
#ifdef DEBUG
	.enforce = 0,
#endif
};

static struct cpu_load_info cpu_load;

static struct params *param;

static void show_log	(struct frqinfo *info, int lload, int bload);
static int start_policy(struct frqinfo *info);
static void stop_policy(struct frqinfo *info);
static int update_frequencies(struct frqinfo *info);
static void get_max_cluster_load(struct cpu_load_info *info,int *little_load, int *big_load);
static DEFINE_PER_CPU(struct cpu_load_info, cpuload);
extern int get_cpuload_info(int little, int thresh);

extern int gpu_power_level;

static void create_debug_fs(void);
static int check_cluster(int cpu);
static int param_lockin = false;
static struct kobject *pstate_kobject;

static void param_lock(void)
{
	param_lockin = true;
	info.sm = FREEZE_STATE;
}

static void param_unlock(void)
{
	info.sm = FREEZE_STATE;
	param_lockin = false;
}

static void decide_post_pause_resume(struct frqinfo *info)
{
	int req_freq = (info->cur_policy == SEMI_TURBO) ?
			info->llast : info->blast;

	if (info->cur_policy == SEMI_TURBO) {
		if (req_freq < param[CLUSTER_LITTLE].freeze) {
			info->sm = PAUSE_STATE;
			return;
		}

		if (req_freq >= param[CLUSTER_LITTLE].effi &&
			info->sm == PAUSE_STATE) {
			info->sm = RESUME_STATE;
			return;
		}
	} else if (info->cur_policy == SUPER_TURBO) {
		if (req_freq < param[CLUSTER_BIG].freeze) {
			info->sm = PAUSE_STATE;
			return;
		}

		if (req_freq >= param[CLUSTER_BIG].effi &&
			info->sm == PAUSE_STATE) {
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
extern int cpufreq_interactive_gov_stat(int cpu);
int cpufreq_interactive_get_load(int cpu, int *freq);
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
	/* check target cpu */
	if(cpufreq_interactive_gov_stat(cpu)) {
		 __cpufreq_driver_target(pcpu->policy, freq, relation);
	}
	return 0;
}
int get_lbfc_state(int cpu)
{
	int cluster = check_cluster(cpu);
	int ret = 1;
	if(info.sm == RUNNING_STATE) {
		if(info.cur_policy == SUPER_TURBO) {
			ret = (cluster) ? 0 : 1;
		} else {
			ret = (cluster) ? 1 : 0;
		}
	}
	return ret;
}
static int get_target_frequency
	(struct frqinfo *info, int target_freq)
{
	int freq = 0;
	int num_of_feed = 0;

	if(info->cur_policy == SEMI_TURBO) {
		num_of_feed = get_cpuload_info(bcluster_start,
					info->tunable->tuanable_feeding_load);
		if(num_of_feed >=
			info->tunable->tunable_feeding_num_t1) {
			freq = info->llast;
		} else if(num_of_feed ==
			info->tunable->tunable_feeding_num_t2){
			freq = param[CLUSTER_LITTLE].effi;
		} else if(num_of_feed ==
			info->tunable->tunable_feeding_num_t3){
			freq = info->tunable->tunable_feeding_num_t3_freq;
		} else {
		/* nothing to do */
		}
		return freq;
	}
	return target_freq;
}
static void target_check_resotre_frequency(struct frqinfo* info)
{
	int *check_freq;
	int cpu = 0;
	cpu = decide_dst_cpu(info->cur_policy);
	check_freq = (cpu < bcluster_start) ? &info->llast : &info->blast;
	if(info->floor != *check_freq) {
		t_debug(TRACE, "lastfreq is not match, restore to last freq %d->%d\n",
			info->floor, *check_freq);
		cpu_set_freq(*check_freq, cpu);
	}
}
static void target_frequency_work(struct work_struct *work)
{
	struct frqinfo *info = container_of(work,
				struct frqinfo,
				delayed_freq_worker.work);
	int cpu;
	int freq = 0;
	/* make cpufreq wave btw determined floor to ceiling freq */
	if (info->sm == RUNNING_STATE) {
		cpu = decide_dst_cpu(info->cur_policy);

		if (info->sw == PERF) {
			freq = get_target_frequency(info, info->floor);
			if (!cpu_set_freq(freq, cpu))
				info->sw = FLOOR;
			else	goto out;

		} else if (info->sw == FLOOR) {
			freq = get_target_frequency(info, info->ceili);
			if (!cpu_set_freq(freq, cpu))
				info->sw = PERF;
			else	goto out;
		}
		queue_delayed_work_on(0, info->frequency_work,
					&info->delayed_freq_worker,
					usecs_to_jiffies(20 * USEC_PER_MSEC));

	} else if (info->sm == FREEZE_STATE) {
		goto out;

	} else {
		/* nothing to do */
	}
	return;
out:
	stop_policy(info);
	return;
}
static int update_frequencies(struct frqinfo *info)
{
	int ret = 0;
	if (param_lockin) {
		pr_err("setting new param\n");
		return ret;
	}
	switch (info->cur_policy) {
	case SUPER_TURBO:
		info->ceili = info->blast;
		info->floor = (info->blast >= param[CLUSTER_BIG].effi) ?
					param[CLUSTER_BIG].effi : info->blast;
		break;
	case SEMI_TURBO:
		info->ceili = info->llast;
		info->floor = param[CLUSTER_LITTLE].effi;
		break;
	case NOMINAL:
		info->ceili = (info->llast >= param[CLUSTER_LITTLE].effi) ?
					info->llast : param[CLUSTER_LITTLE].effi;
		info->floor = param[CLUSTER_LITTLE].effi;
		break;
	default:
		info->cur_policy = NOTHING;
		info->sm = FREEZE_STATE;
		ret = 1;
		break;
	}
	return ret;
}
static void update_state_machine(struct frqinfo *info, bool b)
{
	/* nothing to do */
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
			p->load = DIVIDE(p->load, p->count);
			if(p->load > tlload)
				tlload = p->load;
		} else {
			p->load = DIVIDE(p->load, p->count);
			if(p->load > tbload)
				tbload = p->load;
		}
	}
	*little_load = tlload;
	*big_load = tbload;
}
static void clear_all_cpu_res(struct frqinfo *info)
{
	int i;
	for(i = 0 ; i < MAX_CLUSTER; i++) {
		info->tres[i] = 0;
		info->nres[i] = 0;
		info->sres[i] = 0;
	}
}
static void clear_all_cpu_load(struct cpu_load_info *info)
{
	int i;
	struct cpu_load_info *p;
	for_each_online_cpu(i) {
		p = &per_cpu(cpuload, i);
		p->load = 0;
		p->count = 0;
	}
}
static void stop_policy(struct frqinfo *info)
{
	if(info->cur_policy == NOTHING ||
		info->sm == FREEZE_STATE)
		return;
	target_check_resotre_frequency(info);
#ifdef DEBUG
	if(!info->enforce) {
#endif
	info->cur_policy = NOTHING;
	info->sm = FREEZE_STATE;
	sysfs_notify(pstate_kobject, NULL, "cur_policy");
	t_debug(TRACE, "[P] stop pol :%d-> %d\n", info->last_policy, info->cur_policy);
#ifdef DEBUG
	}
#endif
}

static int start_policy(struct frqinfo *info)
{
	if (info->sm == PAUSE_STATE) {
		goto out;
	}

	if(update_frequencies(info)) {
		goto out;
	}
	if(info->last_policy != info->cur_policy) {
		t_debug(TRACE, "[P] start pol :%d->%d\n", info->last_policy,
			info->cur_policy);
		sysfs_notify(pstate_kobject, NULL, "cur_policy");
	}
	if (info->sm == FREEZE_STATE ||
		info->sm == RESUME_STATE) {
		/* in order to trigger once */
		info->sw = FLOOR;
		info->sm = RUNNING_STATE;

		queue_delayed_work_on(0, info->frequency_work,
				&info->delayed_freq_worker,
				usecs_to_jiffies(0));
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
	if(info->cur_policy == SUPER_TURBO) {
		/* the load of big core goes to low band...
		    but little core in highband
		    */
		int svs_rates = DIVIDE(info->sres[CLUSTER_BIG],
					info->tres[CLUSTER_BIG]);
		int nom_rates = DIVIDE(info->nres[CLUSTER_BIG],
					info->tres[CLUSTER_BIG]);
		if(svs_rates > 0 || nom_rates > 0) {
			if(info->tres[CLUSTER_LITTLE] == MAX_PORTION) {
				goto next;
			}
			goto stop;
		}
	}
	/* stop condition commonly */
	/* turbo stop condition */
	else if (bscore == NO_LOAD &&info->cur_policy == SUPER_TURBO)
		goto stop;
	/* nominal stop condition */
	else if (gpu_power_level <= param[BIT_GPU].effi &&
		info->cur_policy == NOMINAL)
		goto stop;
	else if(info->cur_policy == SEMI_TURBO) {
		int svs_rates = DIVIDE(info->sres[CLUSTER_LITTLE],
					info->tres[CLUSTER_LITTLE]);
		int nom_rates =DIVIDE(info->nres[CLUSTER_LITTLE],
					info->tres[CLUSTER_LITTLE]);
		if(svs_rates > 0 ||nom_rates > 0) {
			goto stop;
		}
	}
	else {
	/* nothing to do */
	}
	return 0;
stop:
	stop_policy(info);
	return 0;
next:
	if(gpu_power_level < param[BIT_GPU].effi)
		info->cur_policy = SEMI_TURBO;
	else{
		/* nothing to do */
	}
	return 0;
}
static void show_log
	(struct frqinfo *info, int lload, int bload)
{
		t_debug(DUMP_LOG, "(L)g:%d t:%d%% n:%d%% s:%d%% l:%d pol:%d\n",
						gpu_power_level,
						info->tres[CLUSTER_LITTLE],
						info->nres[CLUSTER_LITTLE],
						info->sres[CLUSTER_LITTLE],
						lload,
						info->cur_policy);
		t_debug(DUMP_LOG, "(B)t:%d%% n:%d%% s:%d%% l:%d\n",
						info->tres[CLUSTER_BIG],
						info->nres[CLUSTER_BIG],
						info->sres[CLUSTER_BIG],
						bload);
		t_debug(DUMP_LOG, "last : %d  cur : %d big: %d little : %d"
			"  cm : %d\n", info->last_policy,
						info->cur_policy, info->blast,
						info->llast, info->cm);
}
static void decide_policy(struct frqinfo *info)
{
	int lscore, bscore;
	int lload, bload;
	get_max_cluster_load(&cpu_load, &lload, &bload);

	lscore = info->tres[CLUSTER_LITTLE] / lload;
	bscore = info->tres[CLUSTER_BIG] / bload;

	/* pre decide policy */
	if ((info->tres[CLUSTER_LITTLE] == MAX_PORTION
				&& (lload <= (info->tunable->tunable_ign_thres_1*2)))
		|| (info->tres[CLUSTER_BIG] == MAX_PORTION
				&& (bload <= info->tunable->tunable_ign_thres_1))) {
		t_debug(TRACE, "skipped pol\n");
		goto out;
	}

	if (bscore == NO_LOAD && lload <= info->tunable->tunable_ign_thres_2 &&
		info->tres[CLUSTER_LITTLE] <= info->tunable->tunable_ign_res) {
		t_debug(TRACE, "halted pol\n");
		goto out;
	}
	if (bscore == NO_LOAD &&
		((lscore >= HIGH_LOAD &&
		lscore <= info->tunable->tunable_stu_high_thres) ||
		info->tres[CLUSTER_LITTLE] == info->tunable->tunable_stu_high_res) &&
		gpu_power_level < param[BIT_GPU].effi) {

			info->cur_policy = SEMI_TURBO;

	} else if (((bscore > info->tunable->tunable_sstu_low_thres &&
			bscore <= info->tunable->tunable_sstu_high_thres) ||
			info->tres[CLUSTER_BIG] == info->tunable->tunable_stu_high_res)) {
		if(info->cm) {
			info->cur_policy = SUPER_TURBO;
		} else {
			if(gpu_power_level >= param[BIT_GPU].nominal)
				info->cur_policy = SUPER_TURBO;
		}

	} else if (bscore == NO_LOAD &&
		(lscore >= info->tunable->tunable_nom_low_thres &&
		lscore < info->tunable->tunable_nom_high_thres) &&
		gpu_power_level > param[BIT_GPU].effi) {
			info->cur_policy = NOMINAL;
	}
	/* post decide policy */
	decide_post_policy(info, bscore, lscore);
	decide_post_pause_resume(info);
	show_log(info,lload, bload);
	clear_all_cpu_load(&cpu_load);
	clear_all_cpu_res(info);
#ifdef DEBUG
	if(info->enforce > 0) {
		info->cur_policy = info->enforce;
	}
#endif
	start_policy(info);
	return;
out:
	info->cur_policy = NOTHING;
	clear_all_cpu_load(&cpu_load);
	clear_all_cpu_res(info);
	return;
}
static void update_params(struct frqinfo *info, u32 curr_time)
{
	int temp = curr_time - info->sitime;
	int total_portion ;
	int i ;
#define MSEC_TO_SEC 1000
#define SECOND_DIGIT 10000
	for (i = 0; i < MAX_CLUSTER; i++) {
		info->tres[i] = DIVIDE(info->ttime[i] * MSEC_TO_SEC,
			temp);
		info->nres[i] = DIVIDE(info->ntime[i] * MSEC_TO_SEC,
			temp);
		info->sres[i] = DIVIDE(info->stime[i] * MSEC_TO_SEC,
			temp);

		total_portion =  info->tres[i]
			+ info->nres[i] + info->sres[i];

		info->tres[i] = DIVIDE(SECOND_DIGIT * info->tres[i],
			total_portion);
		info->nres[i] = DIVIDE(SECOND_DIGIT * info->nres[i],
			total_portion);
		info->sres[i] = DIVIDE(SECOND_DIGIT * info->sres[i],
			total_portion);
	}
}

static void clear_time_param(struct frqinfo *info, u32 curr)
{
	int i = 0;
		info->sitime = curr;
		for (i = 0; i < MAX_CLUSTER; i++) {
			info->ttime[i] = 0;
			info->ntime[i] = 0;
			info->stime[i] = 0;
		}
}

void _update_online_state(int online, struct cpufreq_policy *policy)
{
	unsigned int i;
	struct cpu_load_info *pcpu;
	unsigned long flags;
	spin_lock_irqsave(&info.policy_lock, flags);
	for_each_cpu(i, policy->cpus) {
		pcpu = &per_cpu(cpuload, i);
		pcpu->policy = policy;
		pcpu->online = online;
	}
	spin_unlock_irqrestore(&info.policy_lock, flags);
	stop_policy(&info);
	cancel_delayed_work_sync(&info.delayed_freq_worker);
}
void _update_cpu_load(int cpu, int freq, int load)
{
	struct cpu_load_info *p;
	p = &per_cpu(cpuload , cpu);
	if(cpu < lcluster_cores) {
		if(freq >= param[CLUSTER_LITTLE].turbo) {
			p->load += load;
			++(p->count);
		}
	} else {
		if(freq >= param[CLUSTER_BIG].nominal) {
			p->load += load;
			++(p->count);
		}
	}
}

static void update_frequency_portion(struct frqinfo *info, bool gov)
{
	u32 curr_time = ktime_to_ms(ktime_get());
	u32 delta ;
	int elapsed;
	int i, *c ;

	if(!info->en) {
		clear_time_param(info, 0);
		clear_all_cpu_load(&cpu_load);
		clear_all_cpu_res(info);
		info->cur_policy = NOTHING;
		info->sm = FREEZE_STATE;
		return;
	}
	/*
	  * gov is false
	  * Invoked by workQ
	  * both of little and big data must be updated
	  */
	if(info->sitime == 0)
		clear_time_param(info, curr_time);

	delta = curr_time - info->litime;

	for (i = 0; i < MAX_CLUSTER; i++) {
		c = (i == 0) ? &info->llast : &info->blast;
		if (*c >= param[i].turbo)
			info->ttime[i] += delta;
		else if (*c >= param[i].nominal &&
				*c < param[i].turbo) {
			info->ntime[i] += delta;
		}
		else {
			info->stime[i] += delta;
		}
	}

	info->litime = curr_time;
	elapsed = curr_time - info->sitime;
	if(elapsed > info->intv ) {
		update_params(info, curr_time);
		decide_policy(info);
		info->sitime = 0;
	}
	return;
}

static void stack_normalize(struct work_struct *work)
{

	struct frqinfo *info = container_of(work,
			struct frqinfo, delayed_freq_normalizer.work);
	update_frequency_portion(info, false);
	update_state_machine(info, false);

	if(info->en)
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

int stack(int cpu, int freq)
{
	int *target;
	info.cluster = check_cluster(cpu);
	target = (!info.cluster) ? &info.llast : &info.blast;
	*target = freq;

	update_frequencies(&info);
	update_frequency_portion(&info, true);
	return 0;
}


static int cpufreq_notifier(
	struct notifier_block *nb, unsigned long val, void *data)
{
	return 0;
}

/*
 * governor policy was change by boost somethings
 * minimum frequency has been changed
 */
static int policy_notifier(struct notifier_block *nb,
		unsigned long val, void *data)
{
	return 0;
}

static struct notifier_block cpufreq_notifier_block = {
	.notifier_call = cpufreq_notifier,
};

static struct notifier_block policy_notifier_block = {
	.notifier_call = policy_notifier,
};

static int __init lbfc_init(void)
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

	info.frequency_work = alloc_workqueue("lbfc:frqwq", WQ_HIGHPRI, 0);
	info.freq_mon = alloc_workqueue("lbfc:monwq", WQ_HIGHPRI, 0);

	INIT_DELAYED_WORK(&info.delayed_freq_worker, target_frequency_work);
	INIT_DELAYED_WORK(&info.delayed_freq_normalizer, stack_normalize);

	mutex_init(&info.freq_lock);
	spin_lock_init(&info.policy_lock);
	for_each_online_cpu(i) {
		pcpu = &per_cpu(cpuload, i);
		pcpu->load = 0;
		pcpu->count = 0;
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
	info.tunable = kmalloc(sizeof(struct tunables),
					GFP_KERNEL);
	if(!info.tunable) {
		return -ENOMEM;
	}
	param = kmalloc(sizeof(struct params) * 3,
					GFP_KERNEL);
	if(!param) {
		return -ENOMEM;
	}
        create_debug_fs();
	return 0;
}

static void lbfc_uninit(void)
{
	destroy_workqueue(info.frequency_work);
	destroy_workqueue(info.freq_mon);

	cpufreq_unregister_notifier(&cpufreq_notifier_block,
			CPUFREQ_TRANSITION_NOTIFIER);
	cpufreq_unregister_notifier(&policy_notifier_block,
			CPUFREQ_POLICY_NOTIFIER);
	if(param) {
		kfree(param);
		param = NULL;
	}
	if(info.tunable) {
		kfree(info.tunable);
		info.tunable = NULL;
	}
}

static void __exit lbfc_exit(void)
{
	lbfc_uninit();
}

/*=====================================================================
 * debug fs
 =======================================================================*/
#define show_one(file_name, object)                                     \
	static ssize_t show_##file_name                                 \
	(struct kobject *kobj, struct attribute *attr, char *buf)       \
	{                                                               \
		return sprintf(buf, "%u\n", info.object);               \
	}
static ssize_t show_en(struct kobject *kobj,
		struct attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", info.en);
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
static ssize_t show_cur_policy(struct kobject *kobj,
		struct attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", info.cur_policy);
}

static ssize_t store_cur_policy(struct kobject *a, struct attribute *b,
				const char *buf, size_t count)
{
	int input;
	int ret;

	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;
	if(input == 0) {
		/* stop policy by hint*/
		cancel_delayed_work(&info.delayed_freq_normalizer);
		cancel_delayed_work(&info.delayed_freq_worker);
		stop_policy(&info);
	}
	return count;
}
#ifdef DEBUG
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
show_one(enforce, enforce);
define_one_global_rw(enforce);
#endif
static ssize_t store_intv(struct kobject *a, struct attribute *b,
				const char *buf, size_t count)
{
	int input;
	int ret;
	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;
	info.intv = input;
	return count;
}
show_one(intv, intv);
define_one_global_rw(intv);

static ssize_t show_sys_tunables(struct kobject *kobj,
		struct attribute *attr, char *buf)
{
	return 0;
}
static ssize_t show_band_cluster(struct kobject *kobj,
		struct attribute *attr, char *buf)
{
	return 0;
}

static ssize_t store_sys_tunables(struct kobject *a, struct attribute *b,
				const char *buf, size_t count)
{
	struct tunables tunable;
	int ret = 0;
	param_lock();
	ret = sscanf(buf, "%d %d %d %d %d %d %d %d %d %d %d %d %d %d",
				&tunable.tunable_sstu_low_thres,
				&tunable.tunable_sstu_high_thres,
				&tunable.tunable_stu_high_thres,
				&tunable.tunable_stu_high_res,
				&tunable.tunable_nom_high_thres,
				&tunable.tunable_nom_low_thres,
				&tunable.tunable_ign_thres_1,
				&tunable.tunable_ign_thres_2,
				&tunable.tunable_ign_res,
				&tunable.tuanable_feeding_load,
				&tunable.tunable_feeding_num_t1,
				&tunable.tunable_feeding_num_t2,
				&tunable.tunable_feeding_num_t3,
				&tunable.tunable_feeding_num_t3_freq);
	if(!info.tunable) {
		pr_err("critical, check tunables\n");
		param_unlock();
		return -EINVAL;
	}
	memcpy(info.tunable, &tunable, sizeof(struct tunables));
	param_unlock();
	return count;
}

static ssize_t store_band_cluster(struct kobject *a, struct attribute *b,
				const char *buf, size_t count)
{

	struct params set;
	int ret, idx = 0;

	param_lock();
	ret = sscanf(buf, "%d %d %d %d %d %d %d",
			&idx,
			&set.turbo, &set.nominal,
			&set.svs, &set.effi, &set.perf, &set.freeze);

	if (idx < CLUSTER_LITTLE || idx > CLUSTER_BIG) {
		pr_err("cannot set param CPU, over range %d\n", idx);
		param_unlock();
		return count;
	}

	param[idx].turbo         = set.turbo;
	param[idx].nominal       = set.nominal;
	param[idx].svs           = set.svs;
	param[idx].effi          = set.effi;
	param[idx].perf          = set.perf;
	param[idx].freeze        = set.freeze;

	param_unlock();
	return count;
}

static ssize_t store_band_gpu(struct kobject *a, struct attribute *b,
				const char *buf, size_t count)
{

	struct params set;
	int ret, idx = 0;

	param_lock();

	ret = sscanf(buf, "%d %d %d %d %d",
			&idx, &set.turbo, &set.nominal, &set.effi, &set.svs);

	if (idx != BIT_GPU) {
		pr_err("cannot set param GPU, over range %d\n", idx);
		param_unlock();
		return count;
	}

	param[BIT_GPU].turbo         = set.turbo;
	param[BIT_GPU].nominal       = set.nominal;
	param[BIT_GPU].effi          = set.effi;
	param[BIT_GPU].svs           = set.svs;

	param_unlock();
	return count;
}
static ssize_t store_cm(struct kobject *a, struct attribute *b,
				const char *buf, size_t count)
{
	int input;
	int ret;

	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;
	info.cm = input;
	return count;
}

static struct global_attr cm =
	__ATTR(cm, 0666, NULL, store_cm);
static struct global_attr sys_tunables =
	__ATTR(sys_tunables, 0666, show_sys_tunables, store_sys_tunables);
static struct global_attr band_cluster =
	__ATTR(band_cluster, 0666, show_band_cluster, store_band_cluster);

static struct global_attr band_gpu =
	__ATTR(band_gpu, 0666, NULL, store_band_gpu);

static struct global_attr cur_policy =
	__ATTR(cur_policy, 0666, show_cur_policy, store_cur_policy);

static struct global_attr en =
	__ATTR(en, 0666, show_en, store_en);
static struct attribute *_attributes[] = {
	&cm.attr,
	&en.attr,
	&cur_policy.attr,
#ifdef DEBUG
	&enforce.attr,
#endif
	&intv.attr,
	&band_cluster.attr,
	&band_gpu.attr,
	&sys_tunables.attr,
	NULL
};

static struct attribute_group attr_group = {
	.attrs = _attributes,
};

static void create_debug_fs(void)
{
	int rc;

	pstate_kobject = kobject_create_and_add("lbfc",
			&cpu_subsys.dev_root->kobj);
	BUG_ON(!pstate_kobject);
	rc = sysfs_create_group(pstate_kobject,
		&attr_group);
	BUG_ON(rc);
}

fs_initcall(lbfc_init);
module_exit(lbfc_exit);
