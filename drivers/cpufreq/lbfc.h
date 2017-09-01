enum state_machine{
	IDLE_STATE,
	RUNNING_STATE,
	FREEZE_STATE,
	PAUSE_STATE,
	RESUME_STATE
};
enum state_wave{
	FLOOR,
	PERF,
};
enum state_policy {
	NOTHING,
	SUPER_TURBO,
	SEMI_TURBO,
	TURBO_NOMINAL,
	SEMI_NOMINAL,
	NOMINAL
};

enum {
	TRACE              = BIT(0), /* report tracing     */
	DUMP_LOG           = BIT(1), /* show DUMP_LOG      */
	PAUSE_RESUME       = BIT(2), /* show PAUSE, RESUME */
	POLICY             = BIT(3), /* Show POLICY        */
};

#define DIVIDE_ZERO(x) ((x == 0 ) ? 1 : x)
#define MAX_CLUSTER       (2)
#define MAX_CPUS_DEFLT    (8)
#define MAX_CPUS          (nr_cpu_ids)

/* define waver section */
#define BIT_GPU           BIT(1)             /* 2 */
#define CLUSTER_BIG       BIT(0)             /* 1 */
#define CLUSTER_LITTLE    (CLUSTER_BIG >> 1) /* 0 */

//#define DEBUG
#define t_debug(mask, args...)\
do {\
	if (mask & _mask)\
		pr_info("lbfc: " args);\
} while(0)
struct params {
	int turbo;
	int nominal;
	int svs;
	int effi;
	int perf;
	int freeze;
};
struct tunables {
	int tunable_sstu_low_thres;
	int tunable_sstu_high_thres;
	int tunable_stu_high_thres;
	int tunable_stu_high_res;
	int tunable_nom_high_thres;
	int tunable_nom_low_thres;
	int tunable_ign_thres_1;
	int tunable_ign_thres_2;
	int tunable_ign_res;
	int tuanable_feeding_load;
	int tunable_feeding_num_t1;
	int tunable_feeding_num_t2;
	int tunable_feeding_num_t3;
	int tunable_feeding_num_t3_freq;
};
struct frqinfo{
	enum state_machine              sm;
	enum state_wave                  sw;
	enum state_policy                 cur_policy;
	enum state_policy                 last_policy;
	struct workqueue_struct         *frequency_work;
	struct workqueue_struct         *freq_mon;
	struct delayed_work             delayed_freq_worker;
	struct delayed_work 		delayed_freq_normalizer;
	spinlock_t      policy_lock;
	struct mutex    freq_lock;
	int             floor;
	int             ceili;
	int             llast;
	int             blast;
	int             cluster;
	u32             ttime[MAX_CLUSTER];
	u32             ntime[MAX_CLUSTER];
	u32             stime[MAX_CLUSTER];
	int             tres[MAX_CLUSTER];
	int             nres[MAX_CLUSTER];
	int             sres[MAX_CLUSTER];
	u32             sitime;
	u32             litime;
	int 		intv;
	bool            en;
#ifdef DEBUG
	int             enforce;
#endif
	bool		cm;
	struct tunables *tunable;
};
struct cpu_load_info
{
	int load;
	int count;
	int online;
	struct cpufreq_policy *policy;
	struct rw_semaphore sem;
};
int get_lbfc_state(int cpu);
int stack(int cpu, int freq);
void cpufreq_update(int cpu);
void cpufreq_interactive_sync(int cpu, u64 now);
void _update_cpu_load(int cpu, int freq, int load);
void _update_online_state(int online, struct cpufreq_policy *policy);

