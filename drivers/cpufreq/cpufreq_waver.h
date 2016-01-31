enum state_machine{
	IDLE_STATE,
	WAVE_STATE,
	FREEZE_STATE,
	PAUSE_STATE,
	RESUME_STATE
};
enum state_wave{
	FLOOR_WAVE,
	PERF_WAVE,
};
enum state_policy {
	NOTHING,
	SUPER_TURBO,
	SEMI_TURBO,
/* between SEMI_TURBO and NOMINAL*/
/* light games, middle of lscore but high gpu */
	TURBO_NOMINAL,
	SEMI_NOMINAL,
	NOMINAL, /* real nominal */
};

enum {
	WAVER_TRACE              = BIT(0), /* report tracing     */
	WAVER_DUMP_LOG           = BIT(1), /* show DUMP_LOG      */
	WAVER_PAUSE_RESUME       = BIT(2), /* show PAUSE, RESUME */
	WAVER_POLICY             = BIT(3), /* Show POLICY        */
};

#define DIVIDE_ZERO(x) ((x == 0 ) ? 1 : x)
#define MAX_CLUSTER       (2)
#define MAX_CPUS_DEFLT    (8)
#define MAX_CPUS          (nr_cpu_ids)

/* define waver section */
#define BIT_GPU           BIT(1)             /* 2 */
#define CLUSTER_BIG       BIT(0)             /* 1 */
#define CLUSTER_LITTLE    (CLUSTER_BIG >> 1) /* 0 */

/* frequency range for policy */
#define NOM_PERF_FREQ (460800)
#define NOM_FLOOR_FREQ (384000)
/* end of tunables */

#define DEBUG
#define CPU_LOAD_CHECK
#ifdef DEBUG
#define t_debug(mask, args...)\
do {\
	if (mask & waver_mask)\
		pr_info(args);\
} while(0)

#else
#define t_debug(fmt, args...)	do {} while(0)
#endif

struct frequency_band {
	int turbo;
	int nominal;
	int semi_nominal;
	int svs;
	int effi;
	int perf;
	int freeze;
};

struct frqinfo{
	enum state_machine              sm;
	enum state_wave                 sw;
	enum state_policy               cur_policy;
	enum state_policy               last_policy;
	/* waving work q */
	struct workqueue_struct         *frequency_work;
	struct workqueue_struct         *freq_mon;
	struct delayed_work             delayed_freq_worker;
	/* cpu frequency normalizer */
	struct delayed_work 		delayed_freq_normalizer;
	spinlock_t      policy_lock;
	struct mutex    freq_lock;
	int             floor_freq;
	int             ceili_freq;
	int             last_l_freq;
	int             last_b_freq;
	int             cluster;
	u32             turbo_time[MAX_CLUSTER];
	u32             nom_time[MAX_CLUSTER];
	u32             svs_time[MAX_CLUSTER];
	int             turbo_portion[MAX_CLUSTER];
	int             nom_portion[MAX_CLUSTER];
	int             svs_portion[MAX_CLUSTER];
	bool            gov_init;
	u32             start_in_time;
	u32             last_in_time;
	int             last_leffi;
	int             last_beffi;
	int 		trigger_intv;
#ifdef DEBUG
	bool            en;
	int             enforce;
#endif
};
struct cpu_load_info
{
	int cpu_t_load;
	int cpu_t_load_count;
	int online;
	struct cpufreq_policy *policy;
	struct rw_semaphore sem;
};


extern void set_current_cpu_load(int cpu, int load, int freq);
