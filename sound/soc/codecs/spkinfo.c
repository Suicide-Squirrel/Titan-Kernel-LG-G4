#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/uaccess.h>
#include <linux/types.h>
#include <linux/kdev_t.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/sysfs.h>
#include <sound/q6afe-v2.h>
#include <linux/kthread.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/syscalls.h>
#include <linux/types.h>
#include <linux/fcntl.h>
#include <linux/file.h>
#include <linux/sched.h>

#define TEMP_OFFSET 2	//passing R ch
#define EXCUR_OFFSET 3	//passing R ch and rx process end point
#define MAX_BUF_TEMP 2048
#define MAX_BUF_EXCUR 2048
#define IDENTIFIER_TEMP (MAX_BUF_TEMP - 1)
#define IDENTIFIER_EXCUR (MAX_BUF_EXCUR - 1)

struct spk_info {
	int temp_idx;
	int excur_idx;
	int ebuf_temp;
	int ebuf_excur;
	unsigned int spk_temp[MAX_BUF_TEMP];
	unsigned int spk_excur[MAX_BUF_EXCUR];
	dev_t dev_num;
	struct cdev c_dev;
	struct task_struct *get_temp_task;
	struct class *info_cl;
	struct mutex mtx;
	int (*fp_get_temp)(unsigned int *_spk_temp);
	int (*fp_get_excur)(unsigned int *_spk_excur);
} si;

int get_spk_temp(unsigned int *_spk_temp);
int get_spk_excursion(unsigned int *_spk_excur);

void workqueue_excur(struct work_struct *dummy);
DECLARE_WAIT_QUEUE_HEAD(myevent_wq);
EXPORT_SYMBOL(myevent_wq);
DECLARE_WAIT_QUEUE_HEAD(eventb);
DEFINE_RWLOCK(myevent_lock);
EXPORT_SYMBOL(myevent_lock);
DECLARE_WORK(work_queue, workqueue_excur);

extern unsigned int over_temp_data[2];
extern int32_t opalum_excursion_model[3];

int logs_enable;

#define LOGSENABLE
#ifdef LOGSENABLE
#define logs(fmt, ...) \
	printk(KERN_ERR pr_fmt(fmt), ##__VA_ARGS__)
#else
#define logs(fmt, ...)
#endif



const static struct file_operations pugs_fops = {
	.owner = THIS_MODULE,
};

static ssize_t sysfs_show_temp(struct class *cls,
		struct class_attribute *attr, char *buf)
{
	int i,n = 0;
	logs("%s() sprint : %d \n"
			,__func__,sprintf(buf, "tmp : 0x%x, 0x%x \n",si.spk_temp[0], si.spk_temp[1]));

	mutex_lock(&si.mtx);
	if (si.ebuf_temp) {
		for(i=0; i < MAX_BUF_TEMP; i++) {
		   	n +=  sprintf(buf+n, "[temp] %d \n",si.spk_temp[i]);
			i++;
	   		if (i == IDENTIFIER_TEMP)  n +=  sprintf(buf+n, "[temp] %d buf end!(%d) \n",si.spk_temp[i],MAX_BUF_TEMP);
		}
	}else
	{
		for(i=0; i < si.temp_idx; i++) {
			n +=  sprintf(buf+n, "[temp] %d	\n",si.spk_temp[i]);
			i++;
		}
	}
	mutex_unlock(&si.mtx);
	return n;
}
static ssize_t sysfs_store_temp(struct class *cls,
		struct class_attribute *attr, const char *buf, size_t count)
{
	int value;
	sscanf(buf, "%d", &value);
	logs("%s() value = %d \n",__func__, value);
	return count;
}

static ssize_t sysfs_show_excur(struct class *cls, struct class_attribute *attr, char *buf) {
	int i, n = 0;
	mutex_lock(&si.mtx);
	if (si.ebuf_excur) {
		for(i=0; i < MAX_BUF_EXCUR; i++) {
		   	n +=  sprintf(buf+n, "[excur] 0x%x \n",si.spk_excur[i]);
			i += 2;
	   		if (i == IDENTIFIER_EXCUR)  n +=
				sprintf(buf+n, "[excur] 0x%x buf end!(%d) \n",si.spk_excur[i],MAX_BUF_EXCUR);
		}
	}else
	{
		for(i=0; i < si.excur_idx; i++) {
			n +=  sprintf(buf+n, "[excur] 0x%x \n",si.spk_excur[i]);
			i += 2;
		}
	}
	mutex_unlock(&si.mtx);
	return n;

}
static ssize_t sysfs_store_excur(struct class *cls, struct class_attribute *attr, const char *buf, size_t count) {
	int value;
	sscanf(buf, "%d", &value);
	logs_enable = value;
	pr_err("logs_enable:%d \n",logs_enable);
	logs("%s() value = %d \n",__func__, value);
	return count;
}
static CLASS_ATTR(spktemp_sysfs, S_IWUSR | S_IRUGO, sysfs_show_temp, sysfs_store_temp);
static CLASS_ATTR(spkexcur_sysfs, S_IWUSR | S_IRUGO, sysfs_show_excur, sysfs_store_excur);

void workqueue_excur(struct work_struct *dummy)
{
	do {
		opalum_afe_get_param(EXCURSION_DATA);
		get_spk_excursion(opalum_excursion_model);
		msleep(5000);
	} while (!si.spk_excur[si.excur_idx-1]);
	logs("*****WorkQueue_Task End!!\n");
}
int get_spk_excursion(unsigned int *_spk_excur) {
        int ret = -1;
	si.spk_excur[si.excur_idx]    = *(_spk_excur);  //L excur
	si.spk_excur[si.excur_idx+1]  = *(_spk_excur+1);//R excur
	si.spk_excur[si.excur_idx+2]  = *(_spk_excur+2);//rx process end status
	
	logs("\n %s() 0x%x, rx_proc_end:%d: ...idx:%d\n"
			,__func__,si.spk_excur[si.excur_idx], si.spk_excur[si.excur_idx+2], si.excur_idx);
	si.excur_idx += EXCUR_OFFSET;
	if (si.excur_idx >= IDENTIFIER_EXCUR) {
		logs("over flow \n");
		si.spk_excur[IDENTIFIER_EXCUR] = 0xFFFFFFFF; //seperate symbol for gathering next queue
		si.ebuf_excur++;
		si.excur_idx = 0;
	}
	else {
		return ret = -1;
	}
	return ret = 0;
}
int get_spk_temp(unsigned int *_spk_temp) {
 	int ret = -1;

	si.spk_temp[si.temp_idx]   = *(_spk_temp);   //L channel
	si.spk_temp[si.temp_idx+1] = *(_spk_temp+1); //R channel(reserved)
	logs("%s() %d, 0x%x .....idx:%d\n",__func__,si.spk_temp[si.temp_idx], si.spk_temp[si.temp_idx+1], si.temp_idx);
	si.temp_idx += TEMP_OFFSET;

	if (si.temp_idx >= IDENTIFIER_TEMP) {
		logs("over flow \n");
		si.spk_temp[IDENTIFIER_TEMP] = 0xFFFFFFFF; //seperate symbol for gathering next queue
		si.ebuf_temp++;
		si.temp_idx = 0;
	}
	else {
		return ret = -1;
	}
	return ret = 0;
}

int getTempThread(void *unused)
{
	static volatile int start_excur_queue = 1;
	while(!kthread_should_stop()) {
		interruptible_sleep_on(&myevent_wq);

		read_lock(&myevent_lock);
		si.fp_get_temp(over_temp_data);
        	read_unlock(&myevent_lock);

		if (start_excur_queue) {
			logs("run excursion queue\n");
			start_excur_queue = 0;
			schedule_work(&work_queue);
		}
		read_lock(&myevent_lock);
		if (opalum_excursion_model[2]) {
			logs("reset work_queue \n");
			start_excur_queue = 1;
		}
		read_unlock(&myevent_lock);
        }

    si.get_temp_task = NULL;
    return 0;
}
static int spk_info_init(void)
{
	int ret = -EINVAL;

	logs("%s()\n",__func__);
	si.fp_get_temp = get_spk_temp;
	si.fp_get_excur = get_spk_excursion;

	if (alloc_chrdev_region(&si.dev_num, 0, 1, "spk_info_drv") < 0) {
		goto rets;
	}

	if ((si.info_cl = class_create(THIS_MODULE, "spk_info")) == NULL) {
		unregister_chrdev_region(si.dev_num, 1);
		goto rets;
	}

	if (device_create(si.info_cl, NULL, si.dev_num, NULL, "spk_dev") == NULL) {
		class_destroy(si.info_cl);
		unregister_chrdev_region(si.dev_num, 1);
		goto rets;
	}

	cdev_init(&si.c_dev, &pugs_fops);
	if (cdev_add(&si.c_dev, si.dev_num, 1) == -1) {
		device_destroy(si.info_cl, si.dev_num);
		class_destroy(si.info_cl);
		unregister_chrdev_region(si.dev_num, 1);
		goto rets;
	}

	ret = class_create_file(si.info_cl, &class_attr_spktemp_sysfs);

	if (ret) {
		logs("Err class_create spktemp_sysfs: %d\n",ret);
		goto rets;
	}

	ret = class_create_file(si.info_cl, &class_attr_spkexcur_sysfs);
	if (ret) {
		logs("Err class_create spkexcur_sysfs: %d\n",ret);
		goto rets;
	}

	si.get_temp_task = kthread_create(getTempThread, NULL, "%s", "getTempThread");
	if (IS_ERR(si.get_temp_task)) {
		logs(" Err Wakeup thread create\n");
	        si.get_temp_task = NULL;
		goto rets;
        }
	else wake_up_process(si.get_temp_task);
	logs("Kernel Thread Create Done(get_temp_task:%p)!!\n", si.get_temp_task);

	rwlock_init(&myevent_lock);
    	mutex_init(&si.mtx);

    return ret = 0;
rets:
    return ret = -EINVAL;
}

static void spk_info_deinit(void)
{
	if(si.get_temp_task) {
		logs("get_temp_task thread stopped(%p)!!\n", si.get_temp_task);
		kthread_stop(si.get_temp_task);
	}

	cdev_del(&si.c_dev);
	device_destroy(si.info_cl, si.dev_num);
	class_destroy(si.info_cl);
	unregister_chrdev_region(si.dev_num, 1);
	logs("WorkQueueTest Module is Unloaded ....\n");
}
module_init(spk_info_init);
module_exit(spk_info_deinit);
MODULE_LICENSE("GPL");
