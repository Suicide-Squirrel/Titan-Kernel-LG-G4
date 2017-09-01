/*
 * Copyright (C) 2015 The Android Open Source Project
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#define DEBUG

#include <linux/cdev.h>
#include <linux/compat.h>
#include <linux/completion.h>
#include <linux/cred.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/ioctl.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/spi/spi-contexthub.h>
#include <linux/spi/spi.h>
#include <linux/jiffies.h>
#include <asm/poll.h>
#include <asm/uaccess.h>

#include "spi_common.h"

static struct spi_driver spich_spi_driver;

/* -------------------------------------------------------------------- */
/* function prototypes							*/
/* -------------------------------------------------------------------- */
#if defined(CONFIG_SPI_MH1)
static int mh1_param_init(struct device *dev, struct spich_data *spich);
#endif
//static int Set_bl_mode(struct spich_data *spich);

#if defined(STM_RESET_DEBUG)
static int dummy_arg;
mh1_data_t *mh1_debug;
static int mh1_spi_write_debug(const char *val, struct kernel_param *kp)
{
	return 0;
}
module_param_call(mh1_spi_write_debug, mh1_spi_write_debug, param_get_bool,
		&dummy_arg, S_IWUSR | S_IRUGO);
#endif


#if defined(CONFIG_SPI_MH1)
static int mh1_param_init(struct device *dev, struct spich_data *spich)
{
	struct device_node *node = dev->of_node;
	int rc = 0;
	u32 spi_freq_mhz = 0;

	printk( "mh1_param_init called\n");

	if (node == NULL) {
		printk(KERN_ERR "%s: Could not find OF device node\n", __func__);
		goto of_err;
	}

	rc = of_property_read_u32(node, "spi-max-frequency", &spi_freq_mhz);
	if (rc) {
		dev_err(dev,"%s: Missing OF property 'spi-max-frequency' \n", __func__);
		goto of_err;
	}

	spich->spi_freq_mhz = spi_freq_mhz;

	/*firmware */
	of_property_read_string(node, "fw_image",&mh1_inbuilt_fw_name_list);

	printk("MH1 fw_image : %s\n", mh1_inbuilt_fw_name_list);

	spich->gpio_cs0 = of_get_named_gpio(node, "mh1,mh1_gpio_cs", 0);
	if (spich->gpio_cs0  < 0) {
		printk("Looking up property in node gpio-cs0 failed. rc =  %d\n",  spich->gpio_cs0);
		goto of_err;
	}

	spich->gpio_cs2 = of_get_named_gpio(node, "mh1,contexthub_gpio_cs", 0);
	if (spich->gpio_cs0  < 0) {
		printk("Looking up property in node gpio-cs2 failed. rc =  %d\n", spich->gpio_cs2);
		goto of_err;
	}

	printk(KERN_ERR "%s Test Log :  spi_freq_mhz : %d, gpio-cs0= %d, gpio-cs2= %d !!!!\n", __func__, spi_freq_mhz, spich->gpio_cs0, spich->gpio_cs2);

	return 0;

of_err:
	return -ENODEV;
}
#endif
#define READ_SYNC_TIMEOUT 100
#define IPC_DEBUG
#if defined(IPC_DEBUG)
#define MAX_DATA 2
enum IPC_VAL {
    IPC_INT = 0x00,
    IPC_WRITE_START,
    IPC_NSS_START,
    IPC_NSS_END,
    IPC_DATA_WRITE_START,
    IPC_DATA_WRITE_END,
    IPC_USER_START,
    IPC_USER_END,
    IPC_INT_DELTA,
    IPC_MAX
};
u64 int_before=0;
u32 ipc_count[IPC_MAX]={0,};
u64 IPC_time[MAX_DATA][IPC_MAX];
void clearTick(void)
{
    u16 i;
    for(i=0;i<IPC_MAX;i++) ipc_count[i] = 0;
}

void getTick(enum IPC_VAL val)
    {
    IPC_time[0][val] = local_clock()/1000000;
    ipc_count[val]++;
    if(val == IPC_INT){
        IPC_time[0][IPC_INT_DELTA] =IPC_time[0][IPC_INT]-int_before ;
        int_before = IPC_time[0][IPC_INT];
    }
}
#endif

static int spich_suspend(struct spi_device *spi, pm_message_t state)
{
#if !defined(STM_NOT_SLEEP)
if(spich_p->pre_reset_delay ==0)
    StmResetHub(STM_SHUTDOWN);
#endif    
	dev_dbg(&spi->dev, "STM spich_suspend\n");
	return 0;
}

static int spich_resume(struct spi_device *spi)
{
#if !defined(STM_NOT_SLEEP)
if(spich_p->pre_reset_delay ==0)
   StmResetHub(STM_RESET);
#endif    
	dev_dbg(&spi->dev, "STM spich_resume\n");
	return 0;
}

static irqreturn_t sh2ap_isr(int irq, void *data)
{
	struct spich_data *spich = data;
#if defined(IPC_DEBUG)
    getTick(IPC_INT);
#endif
	complete(&spich->sh2ap_completion);

	return IRQ_HANDLED;
}

static int spich_init_instance(struct spich_data *spich)
{
	int status = 0, i=0;
    u32 val;

	printk( "STM %s(%d)\n",__func__,__LINE__);

	/*gyro_vdd */
	status = of_property_read_u32(spich->spi->dev.of_node, "contexthub,pre_reset_delay", &val);
	if (status < 0) {
		printk("STM pre_reset_delay not found\n");
        spich->pre_reset_delay=0;
	}
    spich->pre_reset_delay= val;
    printk("STM post delay=%d \n",spich->pre_reset_delay);
	/*firmware */
	of_property_read_string(spich->spi->dev.of_node, "contexthub,fw_image",&stm_inbuilt_fw_name_list);
	spich->gpio_array[GPIO_IDX_AP2SH].flags = GPIOF_OUT_INIT_HIGH;
	spich->gpio_array[GPIO_IDX_AP2SH].label = "contexthub,ap2sh";
	spich->gpio_array[GPIO_IDX_SH2AP].flags = GPIOF_DIR_IN;
	spich->gpio_array[GPIO_IDX_SH2AP].label = "contexthub,sh2ap";
	spich->gpio_array[GPIO_IDX_BOOT0].flags = GPIOF_OUT_INIT_LOW;
	spich->gpio_array[GPIO_IDX_BOOT0].label = "contexthub,boot0";
#if defined(DEBUG_STM_JTAG)
	spich->gpio_array[GPIO_IDX_NRST].flags = GPIOF_OPEN_DRAIN | GPIOF_OUT_INIT_HIGH;
#else
	spich->gpio_array[GPIO_IDX_NRST].flags = GPIOF_OUT_INIT_HIGH;
#endif
	spich->gpio_array[GPIO_IDX_NRST].label = "contexthub,nrst";
	spich->gpio_array[GPIO_IDX_WAKEUP].flags = GPIOF_DIR_IN;
	spich->gpio_array[GPIO_IDX_WAKEUP].label = "contexthub,wakeup";
	spich->gpio_array[GPIO_IDX_CS2].flags = GPIOF_OUT_INIT_HIGH;
	spich->gpio_array[GPIO_IDX_CS2].label = "contexthub,cs2";
	spich->gpio_array[GPIO_IDX_CS0].flags = GPIOF_OUT_INIT_LOW;
	spich->gpio_array[GPIO_IDX_CS0].label = "contexthub,cs0";
#if defined (CONFIG_MACH_MSM8992_PPLUS_KR)
	spich->gpio_array[GPIO_IDX_CS3].flags = GPIOF_OUT_INIT_HIGH;
	spich->gpio_array[GPIO_IDX_CS3].label = "contexthub,cs3";
#endif
	spich->gpio_array[GPIO_IDX_LDOEN].flags = GPIOF_OUT_INIT_LOW;
	spich->gpio_array[GPIO_IDX_LDOEN].label = "contexthub,ldoen";
	for (i = 0; i < ARRAY_SIZE(spich->gpio_array); i++) {
		int gpio=0;

		gpio = of_get_named_gpio(spich->spi->dev.of_node,
					 spich->gpio_array[i].label, 0);
		if (gpio < 0) {
			dev_err(&spich->spi->dev,
				"STM failed to find %s property in DT\n",
				spich->gpio_array[i].label);
			return -1;
		}
		spich->gpio_array[i].gpio = gpio;
		dev_info(&spich->spi->dev, "%s: %s=%u\n",
			 __func__, spich->gpio_array[i].label,
			 spich->gpio_array[i].gpio);
	}

	spich->buffer = kmalloc(SPICH_BUFFER_SIZE, GFP_KERNEL);
	if (!spich->buffer) {
		dev_err(&spich->spi->dev, "STM open/ENOMEM\n");
		status = -ENOMEM;
		goto bail;
	}

	spich->bufferrx = kmalloc(SPICH_BUFFER_SIZE, GFP_KERNEL);
	if (!spich->bufferrx) {
		dev_err(&spich->spi->dev, "STM open/ENOMEM\n");
		status = -ENOMEM;
		goto bail2;
	}
	status = gpio_request_array(spich->gpio_array,GPIO_IDX_FIXED_ARRAY);/* change ARRAY_SIZE(spich->gpio_array)); to GPIO_IDX_FIXED_ARRAY. because CS3 is registered in TDMB already. we can't register that gpio */

	if (status != 0) {
		dev_err(&spich->spi->dev, "STM open/gpio_request\n");
		goto bail3;
	}
	spich->sh2ap_irq = gpio_to_irq(spich->gpio_array[GPIO_IDX_SH2AP].gpio);

	init_completion(&spich->sh2ap_completion);

	status = devm_request_irq(&spich->spi->dev,
				  spich->sh2ap_irq,
				  sh2ap_isr,
				  IRQF_TRIGGER_FALLING,
				  dev_name(&spich->spi->dev), spich);


	if (status != 0) {
		dev_err(&spich->spi->dev, "STM open/devm_request_irq\n");
		goto bail4;
	}

	return 0;

bail4:
	gpio_free_array(spich->gpio_array, ARRAY_SIZE(spich->gpio_array));
bail3:
	kfree(spich->bufferrx);
	spich->bufferrx = NULL;
bail2:
	kfree(spich->buffer);
	spich->buffer = NULL;

bail:
	return status;
}

static void spich_destroy_instance(struct spich_data *spich)
{
	devm_free_irq(&spich->spi->dev, spich->sh2ap_irq, spich);

	printk( "STM %s(%d)\n",__func__,__LINE__);
	gpio_free_array(spich->gpio_array, ARRAY_SIZE(spich->gpio_array));

	kfree(spich->buffer);
	spich->buffer = NULL;

	kfree(spich->bufferrx);
	spich->bufferrx = NULL;
}

static int spich_open(struct inode *inode, struct file *filp)
{
	struct spich_data *spich;
	unsigned users;
	unsigned uid = current_uid();
	printk( "STM %s(%d)\n",__func__,__LINE__);
#if defined(IPC_DEBUG)
    clearTick();
#endif
	spich = container_of(inode->i_cdev, struct spich_data, cdev);

	spin_lock_irq(&spich->spi_lock);
	users = spich->users++;

	/* Processes running as root (uid 0) can always open the device;
	 * otherwise, non-root processes must wait until all other processes
	 * close the device */
	if ((uid != 0) && (users != 0)) {
		--spich->users;
		spin_unlock_irq(&spich->spi_lock);
	    printk( "STM %s(%d) spich->users : %d \n",__func__,__LINE__,spich->users);
		return -EBUSY;
	}
	spin_unlock_irq(&spich->spi_lock);

	filp->private_data = spich;
	nonseekable_open(inode, filp);
    //MATTHEW_TEST
	init_completion(&spich->sh2ap_completion);
	return 0;
}

static int spich_release(struct inode *inode, struct file *filp)
{
	struct spich_data *spich;
	unsigned users;

	printk( "STM %s(%d)\n",__func__,__LINE__);

	spich = filp->private_data;
	filp->private_data = NULL;

	spin_lock_irq(&spich->spi_lock);
	users = --spich->users;
	spin_unlock_irq(&spich->spi_lock);

#if defined(STM_NOT_SLEEP)
    StmResetHub(STM_SHUTDOWN);
#endif    
	if (users == 0) {
		if (spich->spi == NULL) {
			spich_destroy_instance(spich);
			mutex_destroy(&spich->buf_lock);
			kfree(spich);
		}
	}
	return 0;
}

static void spich_complete(void *arg)
{
	complete(arg);
}

static ssize_t spich_sync(struct spich_data *spich, struct spi_message *message)
{
	DECLARE_COMPLETION_ONSTACK(done);
	int status;

	message->complete = spich_complete;
	message->context = &done;

	gpio_set_value(spich->gpio_array[GPIO_IDX_AP2SH].gpio, 0);

	spin_lock_irq(&spich->spi_lock);
	if (spich->spi == NULL) {
		status = -ESHUTDOWN;
	} else {
#if defined(IPC_DEBUG)
            getTick(IPC_DATA_WRITE_START);
#endif
		status = spi_async(spich->spi, message);
#if defined(IPC_DEBUG)
            getTick(IPC_DATA_WRITE_START);
#endif
	}
	spin_unlock_irq(&spich->spi_lock);

	if (status == 0) {
		wait_for_completion(&done);
		status = message->status;
		if (status == 0) {
			status = message->actual_length;
		}
	}

	gpio_set_value(spich->gpio_array[GPIO_IDX_AP2SH].gpio, 1);

	return status;
}

static void SET_U64(void *_data, uint64_t x)
{
	uint8_t *data = (uint8_t *) _data;
	data[0] = (x >> 56) & 0xff;
	data[1] = (x >> 48) & 0xff;
	data[2] = (x >> 40) & 0xff;
	data[3] = (x >> 32) & 0xff;
	data[4] = (x >> 24) & 0xff;
	data[5] = (x >> 16) & 0xff;
	data[6] = (x >> 8) & 0xff;
	data[7] = x & 0xff;
}

static int spich_message(struct spich_data *spich,
			 struct spi_ioc_transfer *u_xfers, unsigned n_xfers)
{
	struct spi_message msg;
	struct spi_transfer *k_xfers;
	struct spi_transfer *k_tmp;
	struct spi_ioc_transfer *u_tmp;
	u8 *buf;
	u8 *bufrx;
	int status = -EFAULT;
	unsigned n;
	unsigned total;
	struct timespec t;
	uint64_t now_us;

	spi_message_init(&msg);

	k_xfers = kcalloc(n_xfers, sizeof(struct spi_transfer), GFP_KERNEL);
	if (k_xfers == NULL) {
		return -ENOMEM;
	}

	if (spich->flags & SPICH_FLAG_TIMESTAMPS_ENABLED) {
		do_posix_clock_monotonic_gettime(&t);
		now_us = t.tv_sec * 1000000ull + (t.tv_nsec + 500ull) / 1000ull;
	}

	buf = spich->buffer;
	bufrx = spich->bufferrx;
	total = 0;
	for (n = n_xfers, k_tmp = k_xfers, u_tmp = u_xfers; n;
	     --n, ++k_tmp, ++u_tmp) {
		k_tmp->len = u_tmp->len;

		/* to prevent malicious attacks with extremely large lengths,
		 * check to make sure each transfer is less than the buffer
		 * size. This prevents overflow of 'total'.
		 */
		if (k_tmp->len > SPICH_BUFFER_SIZE) {
			status = -EMSGSIZE;
			goto done;
		}

		total += k_tmp->len;
		if (total > SPICH_BUFFER_SIZE) {
			status = -EMSGSIZE;
			goto done;
		}

		if (u_tmp->rx_buf) {
			k_tmp->rx_buf = bufrx;
			if (!access_ok(VERIFY_WRITE,
				       (u8 __user *) (uintptr_t) u_tmp->rx_buf,
				       u_tmp->len)) {
				goto done;
			}
		}

		if (u_tmp->tx_buf) {
			k_tmp->tx_buf = buf;
			if (copy_from_user(buf, (const u8 __user *)(uintptr_t)
					   u_tmp->tx_buf, u_tmp->len)) {
				goto done;
			}

			if (u_tmp->len >= 9
			    && (spich->flags & SPICH_FLAG_TIMESTAMPS_ENABLED)) {
				SET_U64(&buf[1], now_us);
			}
		}

		buf += k_tmp->len;
		bufrx += k_tmp->len;

		k_tmp->cs_change = ! !u_tmp->cs_change;
		k_tmp->bits_per_word = u_tmp->bits_per_word;
		k_tmp->delay_usecs = u_tmp->delay_usecs;
		k_tmp->speed_hz = u_tmp->speed_hz;

		spi_message_add_tail(k_tmp, &msg);
	}
	status = spich_sync(spich, &msg);
	if (status < 0) {
		goto done;
	}

	buf = spich->bufferrx;
	for (n = n_xfers, u_tmp = u_xfers; n; --n, ++u_tmp) {
		if (u_tmp->rx_buf) {
			if (__copy_to_user
			    ((u8 __user *) (uintptr_t) u_tmp->rx_buf, buf,
			     u_tmp->len)) {
				status = -EFAULT;
				goto done;
			}
		}

		buf += u_tmp->len;
	}

	status = total;

done:
	kfree(k_xfers);
	return status;
}

#define SPI_MODE_MASK                           \
    (SPI_CPHA | SPI_CPOL | SPI_CS_HIGH          \
    | SPI_LSB_FIRST | SPI_3WIRE | SPI_LOOP      \
    | SPI_NO_CS | SPI_READY)

static long spich_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int err = 0;
	struct spich_data *spich;
	struct spi_device *spi;

	if (_IOC_TYPE(cmd) != SPI_IOC_MAGIC) {
		return -ENOTTY;
	}

	if (_IOC_DIR(cmd) & _IOC_READ) {
		err =
		    !access_ok(VERIFY_WRITE, (void __user *)arg,
			       _IOC_SIZE(cmd));
	}

	if (err == 0 && (_IOC_DIR(cmd) & _IOC_WRITE)) {
		err =
		    !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	}

	if (err) {
		return -EFAULT;
	}

	spich = filp->private_data;
	spin_lock_irq(&spich->spi_lock);
	spi = spi_dev_get(spich->spi);
	spin_unlock_irq(&spich->spi_lock);

	if (spi == NULL) {
		return -ESHUTDOWN;
	}

	mutex_lock(&spich->buf_lock);

	switch (cmd) {
	case SPI_IOC_RESET_HUB:
		{
			u32 tmp;
			err = __get_user(tmp, (u8 __user *) arg);

			if (err != 0) {
				break;
			}

            if(tmp==1){
                StmResetHub(STM_SYSTEM);
            }
            else if(tmp==0){
                StmResetHub(STM_RESET);
            }
	        init_completion(&spich->sh2ap_completion);
        break;
		}

	case SPI_IOC_ENABLE_TIMESTAMPS:
		{
			u32 tmp;
			err = __get_user(tmp, (u8 __user *) arg);
			if (err != 0) {
				break;
			}

			if (tmp) {
				spich->flags |= SPICH_FLAG_TIMESTAMPS_ENABLED;
			} else {
				spich->flags &= ~SPICH_FLAG_TIMESTAMPS_ENABLED;
			}
			break;
		}

	case SPI_IOC_WR_MODE:
		{
			u32 tmp;
			u8 save;

			err = __get_user(tmp, (u8 __user *) arg);
			if (err != 0) {
				break;
			}
			save = spi->mode;
			if (tmp & ~SPI_MODE_MASK) {
				err = -EINVAL;
				break;
			}
			tmp |= spi->mode & ~SPI_MODE_MASK;
			spi->mode = (u8) tmp;
			err = spi_setup(spi);
			if (err < 0) {
				spi->mode = save;
			} else {
				dev_dbg(&spi->dev, "STM spi mode %02x\n", tmp);
			}
			break;
		}

	case SPI_IOC_WR_LSB_FIRST:
		{
			u32 tmp;
			u8 save;
			err = __get_user(tmp, (u8 __user *) arg);
			if (err != 0) {
				break;
			}
			save = spi->mode;
			if (tmp) {
				spi->mode |= SPI_LSB_FIRST;
			} else {
				spi->mode &= ~SPI_LSB_FIRST;
			}
			err = spi_setup(spi);
			if (err < 0) {
				spi->mode = save;
			} else {
				dev_dbg(&spi->dev, "STM %csb first\n",
					tmp ? 'l' : 'm');
			}
			break;
		}

	case SPI_IOC_WR_BITS_PER_WORD:
		{
			u32 tmp;
			u8 save;

			err = __get_user(tmp, (u8 __user *) arg);
			if (err != 0) {
				break;
			}
			save = spi->bits_per_word;
			spi->bits_per_word = tmp;
			err = spi_setup(spi);
			if (err < 0) {
				spi->bits_per_word = save;
			} else {
				dev_dbg(&spi->dev, "STM %d bits per word\n", tmp);
			}
			break;
		}

	case SPI_IOC_WR_MAX_SPEED_HZ:
		{
			u32 tmp;
			u32 save;

			err = __get_user(tmp, (__u32 __user *) arg);
			if (err != 0) {
				break;
			}

			save = spi->max_speed_hz;
			spi->max_speed_hz = tmp;
			err = spi_setup(spi);

			if (err < 0) {
				spi->max_speed_hz = save;
			} else {
				dev_dbg(&spi->dev, "STM %d Hz (max)\n", tmp);
			}
			break;
		}
    /* MATTHEW ADD */
	case SPI_IOC_DOWNLOAD_FIRMWARE:
		{
			u32 tmp;
            int error=0;

            err = __get_user(tmp, (u8 __user *) arg);
			if (err != 0) {
                printk("STM DOWNLOAD FIRMWARE err = %d \n",err);
				break;
			}
            error = try_download_firmware(spich);/* download the stm firmware image */
            if(error!=0)
                printk("STM DOWNLOAD FIRMWARE error = %d \n",error);
			break;
		}
	case SPI_IOC_TXRX_STM:
		{
			struct spi_ioc_transfer t;
			if (__copy_from_user(&t, (void __user *)arg, sizeof(t))) {
				err = -EFAULT;
			}
            gpio_set_value(spich->gpio_array[GPIO_IDX_AP2SH].gpio,1);
			if (gpio_get_value
			    (spich->gpio_array[GPIO_IDX_SH2AP].gpio) == 1) {
				wait_for_completion_interruptible
				    (&spich->sh2ap_completion);
			}

			err = spich_message(spich, &t, 1);
			break;
		}
	case SPI_IOC_READ_SYNC:
		{
			u32 tmp;
            int rc=0;
            err = __get_user(tmp, (u8 __user *) arg);
			if (err != 0) {
                printk("STM READ SYNC err = %d \n",err);
				break;
			}
				rc = wait_for_completion_interruptible_timeout
				    (&spich->sh2ap_completion,(msecs_to_jiffies(READ_SYNC_TIMEOUT)) );
                if(rc==0){ //timeout
                    err = -EBUSY;
                    printk("STM READ SYNC %d second timeout error completion cnt = %d \n",READ_SYNC_TIMEOUT,spich->sh2ap_completion.done);
                }
			break;
		}
	case SPI_IOC_WRITE_SYNC:
		{
#if 0
            u32 tmp;
			err = __get_user(tmp, (u8 __user *) arg);
			if (err != 0) {
                printk("STM DOWNLOAD FIRMWARE err = %d \n",err);
				break;
			}
            gpio_set_value(spich->gpio_array[GPIO_IDX_AP2SH].gpio,0);
#endif
			break;
		}
	case SPI_IOC_WRITE_SYNC_END:
		{
#if 0
			u32 tmp;
			err = __get_user(tmp, (u8 __user *) arg);
			if (err != 0) {
                printk("STM DOWNLOAD FIRMWARE err = %d \n",err);
				break;
			}
            gpio_set_value(spich->gpio_array[GPIO_IDX_AP2SH].gpio,1);
#endif
			break;
		}
	case SPI_IOC_TXRX_START:
		{
#if 0
            u32 tmp;
			err = __get_user(tmp, (u8 __user *) arg);

			if (err != 0) {
				break;
			}
            gpio_set_value(spich->gpio_array[GPIO_IDX_CS2].gpio,0);
			break;
#endif
            break;
		}
	case SPI_IOC_TXRX_END:
		{
#if 0
            u32 tmp;
			err = __get_user(tmp, (u8 __user *) arg);
			if (err != 0) {
				break;
			}
            gpio_set_value(spich->gpio_array[GPIO_IDX_CS2].gpio,1);
			break;
#endif
            break;
		}
    /* MATTHEW END */
	case SPI_IOC_TXRX:
		{
			struct spi_ioc_transfer t;

			if (__copy_from_user(&t, (void __user *)arg, sizeof(t))) {
				err = -EFAULT;
				break;
			}

			if (gpio_get_value
			    (spich->gpio_array[GPIO_IDX_SH2AP].gpio) == 1) {
				wait_for_completion_interruptible
				    (&spich->sh2ap_completion);
			}

			err = spich_message(spich, &t, 1);
			break;
		}

	default:
		{
			u32 tmp;
			unsigned n_ioc;
			struct spi_ioc_transfer *ioc;

			if (_IOC_NR(cmd) != _IOC_NR(SPI_IOC_MESSAGE(0))
			    || _IOC_DIR(cmd) != _IOC_WRITE) {
				err = -ENOTTY;
				break;
			}

			tmp = _IOC_SIZE(cmd);
			if ((tmp % sizeof(struct spi_ioc_transfer)) != 0) {
				err = -EINVAL;
				break;
			}

			n_ioc = tmp / sizeof(struct spi_ioc_transfer);
			if (n_ioc == 0) {
				break;
			}

			ioc = kmalloc(tmp, GFP_KERNEL);
			if (!ioc) {
				err = -ENOMEM;
				break;
			}

			if (__copy_from_user(ioc, (void __user *)arg, tmp)) {
				kfree(ioc);
				err = -EFAULT;
				break;
			} else {

#if defined(IPC_DEBUG)
             getTick(IPC_NSS_START);
#endif
                Spi_Cs_Configuration(2);
#if defined(IPC_DEBUG)
             getTick(IPC_USER_START);
#endif
				err = spich_message(spich, ioc, n_ioc);
#if defined(IPC_DEBUG)
             getTick(IPC_USER_END);
#endif
			}

            Spi_Cs_Configuration(4);
#if defined(IPC_DEBUG)
             getTick(IPC_NSS_END);
#endif
            if(((IPC_time[0][IPC_NSS_END]-IPC_time[0][IPC_INT])> 10)|| \
                (IPC_time[0][IPC_INT_DELTA]> 35)|| \
                ((ipc_count[IPC_INT]%30)==0))
                printk( "STM(ms)cnt:%d,INT(D),%llu,INT,%llu,NSS,%llu,USER,%llu,TOT,%llu \n",ipc_count[IPC_INT],IPC_time[0][IPC_INT_DELTA],IPC_time[0][IPC_INT], \
                                                               (IPC_time[0][IPC_NSS_END]-IPC_time[0][IPC_NSS_START]), \
                                                               (IPC_time[0][IPC_USER_END]-IPC_time[0][IPC_USER_START]), \
                                                               (IPC_time[0][IPC_NSS_END]-IPC_time[0][IPC_INT]));
			kfree(ioc);
			break;
		}
	}

	mutex_unlock(&spich->buf_lock);
	spi_dev_put(spi);

	return err;
}

#ifdef CONFIG_COMPAT
static long spich_compat_ioctl(struct file *filp, unsigned int cmd,
			       unsigned long arg)
{
	return spich_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
}
#else
#define spich_compat_ioctl      NULL
#endif

static ssize_t spich_sync_read(struct spich_data *spich, size_t len)
{
	struct spi_transfer t = {
		.tx_buf = spich->buffer,
		.rx_buf = spich->bufferrx,
		.len = len,
	};
	struct spi_message m;
	struct timespec ts;
	uint64_t now_us;
	ssize_t status;
	printk( "STM %s(%d)\n",__func__,__LINE__);

	memset(spich->buffer, 0, len);
	spich->buffer[0] = 0x55;

	do_posix_clock_monotonic_gettime(&ts);
	now_us = ts.tv_sec * 1000000ull + (ts.tv_nsec + 500ull) / 1000ull;
	SET_U64(&spich->buffer[1], now_us);

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);

	status = spich_sync(spich, &m);

	if (status != 0) {
		return status;
	}

	status = len;
	return status;
}

static ssize_t spich_read(struct file *filp, char __user * buf, size_t count,
			  loff_t * f_pos)
{
	struct spich_data *spich;
	ssize_t status;
	printk("STM %s(%d)\n",__func__,__LINE__);

	if (count > SPICH_BUFFER_SIZE) {
		return -EMSGSIZE;
	}

	spich = filp->private_data;
	if (gpio_get_value(spich->gpio_array[GPIO_IDX_SH2AP].gpio) == 1) {
		wait_for_completion_interruptible(&spich->sh2ap_completion);
	}

	mutex_lock(&spich->buf_lock);

	status = spich_sync_read(spich, count);

	if (status > 0) {
		unsigned long missing =
		    copy_to_user(buf, spich->bufferrx, status);

		if (missing == status) {
			status = -EFAULT;
		} else {
			status -= missing;
		}
	}

	mutex_unlock(&spich->buf_lock);

	return status;
}

static unsigned int spich_poll(struct file *filp,
			       struct poll_table_struct *wait)
{
	struct spich_data *spich;
	unsigned int mask = 0;
	unsigned users;
	unsigned uid = current_uid();
	printk("STM %s(%d)\n",__func__,__LINE__);

	spich = filp->private_data;

	if (gpio_get_value(spich->gpio_array[GPIO_IDX_SH2AP].gpio) == 1) {
		poll_wait(filp, &spich->sh2ap_completion.wait, wait);
	}

	/* If this a non-root process and there is another (root) user,
	 * then only the root user gets access to SPI traffic */
	spin_lock_irq(&spich->spi_lock);
	users = spich->users;
	if ((uid != 0) && (users > 1)) {
		spin_unlock_irq(&spich->spi_lock);
		return 0;
	}
	spin_unlock_irq(&spich->spi_lock);

	if (gpio_get_value(spich->gpio_array[GPIO_IDX_SH2AP].gpio) == 0) {
		mask |= POLLIN | POLLRDNORM;
	}

	return mask;
}

/* sysfs START==========================================
 * /sys/class/spi_master/spi200/spi200.2/stm_cal
 *                                       stm_id
 *                                       stm_fw_ver
 */
int cal_flag=1;
static ssize_t stm_cal_store(struct device *dev,struct device_attribute *attr, const char *buf,size_t count)
{
    int result;
    printk("STM %s %d %s \n",__func__,__LINE__,buf);
    result =  stm_calibration();
    if(result == 0){
        cal_flag=0;
    }
    else{
        cal_flag=1;
    }
    return count;
}   
static ssize_t stm_cal_show(struct device *dev,struct device_attribute *attr, char *buf)
{
    printk("STM %s cal_value = %d \n",__func__,cal_flag);
	return snprintf(buf,PAGE_SIZE,"%d\n",cal_flag);
}
static DEVICE_ATTR(stm_cal,S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP, stm_cal_show,stm_cal_store);
static struct attribute *attributes[] = {
	&dev_attr_stm_cal.attr,
	NULL
};
static const struct attribute_group attribute_group = {
	.attrs = attributes,
};

/*sysfs END================================================
 *
 */
static const struct file_operations spich_fops = {
	.owner = THIS_MODULE,

	.unlocked_ioctl = spich_ioctl,
	.compat_ioctl = spich_compat_ioctl,
	.open = spich_open,
	.release = spich_release,
	.llseek = no_llseek,
	.read = spich_read,
	.poll = spich_poll,
};

static int spich_probe(struct spi_device *spi)
{
	struct spich_data *spich;
	const char *st = NULL;
	int error = 0;

	printk( "STM %s(%d)\n",__func__,__LINE__);
	spich = kzalloc(sizeof(struct spich_data), GFP_KERNEL);

	if (!spich) {
		dev_err(&spi->dev, "STM memory allocation error!.\n");
		return -ENOMEM;
	}

	spich->spi = spi;
	spin_lock_init(&spich->spi_lock);
	mutex_init(&spich->buf_lock);

	error = of_property_read_string(spich->spi->dev.of_node,
					"lge.dev_name", &st);
	if (error) {
		pr_err(":error reading name. rc=%d\n", error);
		goto err_class_create;
	}

	printk("spich_probe name = %s\n", st);

	if(!strcmp(st, "contexthub"))
	{
		spich->class = class_create(THIS_MODULE, DRV_CLASS_NAME);
		if (IS_ERR(spich->class)) {
			dev_err(&spich->spi->dev, "STM failed to create class.\n");
			error = PTR_ERR(spich->class);
			goto err_class_create;
		}

		error = alloc_chrdev_region(&spich->devno, 0, 1, DRV_NAME);
		if (error < 0) {
			dev_err(&spich->spi->dev, "STM alloc_chrdev_region failed.\n");
			goto err_alloc_chrdev;
		}

		spich->device = device_create(spich->class, NULL, spich->devno,
					      NULL, "%s", DRV_NAME);

		if (IS_ERR(spich->device)) {
			dev_err(&spich->spi->dev, "STM device_create failed.\n");
			error = PTR_ERR(spich->device);
			goto err_device_create;
		}
	    error = sysfs_create_group(&spich->spi->dev.kobj, &attribute_group);
    	if (error) {
	    	printk("STM could not create sysfs\n");
    	}
		cdev_init(&spich->cdev, &spich_fops);
		spich->cdev.owner = THIS_MODULE;

		error = cdev_add(&spich->cdev, spich->devno, 1);
		if (error) {
			dev_err(&spich->spi->dev, "STM cdev_add failed.\n");
			goto err_device_create;
		}
        error = spich_init_instance(spich);
		if (error) {
			dev_err(&spich->spi->dev, "STM spich_init_instance failed.\n");
			goto err_spich_init_instance;
		}
        spich_p = spich;
        try_download_firmware(spich);/* download the stm firmware image */
		goto contexthub_complete_spi;
   }
#if defined(CONFIG_SPI_MH1)
		if(!strcmp(st, "mh1"))
		{
			error = mh1_param_init(&spi->dev, spich);
			if (error) {
				dev_err(&spich->spi->dev, "mh1_param_init failed.\n");
				goto err_class_create;
			}
		mh1_spich = spich;
//		mh1_fetch_image(spich);
		goto mh1_complete_spi;
		}
#endif


#if defined(CONFIG_SPI_MH1)
mh1_complete_spi:
	spi_set_drvdata(spi, spich);
	return 0;
#endif
contexthub_complete_spi:
	spi_set_drvdata(spi, spich);
	return 0;
err_spich_init_instance:
	cdev_del(&spich->cdev);

err_device_create:
	device_destroy(spich->class, spich->devno);

err_alloc_chrdev:
	class_destroy(spich->class);

err_class_create:
	kfree(spich);

	return error;
}

static int spich_remove(struct spi_device *spi)
{
	struct spich_data *spich = spi_get_drvdata(spi);
	unsigned users;

	cdev_del(&spich->cdev);
	device_destroy(spich->class, spich->devno);
	class_destroy(spich->class);

	spin_lock_irq(&spich->spi_lock);
	spich->spi = NULL;
	users = spich->users;
	spi_set_drvdata(spi, NULL);
	spin_unlock_irq(&spich->spi_lock);

	if (users == 0) {
		spich_destroy_instance(spich);
		mutex_destroy(&spich->buf_lock);
		kfree(spich);
	}

	return 0;
}

static const struct of_device_id spich_dt_ids[] = {
	{.compatible = "contexthub" DRV_NAME },
	{.compatible = "mh1" DRV_NAME },
	{},
};

MODULE_DEVICE_TABLE(of, spich_dt_ids);

static struct spi_driver spich_spi_driver = {
	.driver = {
		   .name = DRV_NAME,
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(spich_dt_ids),
		   },
	.probe = spich_probe,
	.remove = spich_remove,
	.suspend = spich_suspend,
	.resume = spich_resume,
};

static int __init spich_init(void)
{
	pr_info(DRV_NAME ": %s\n", __func__);
	if (spi_register_driver(&spich_spi_driver)) {
		pr_err(DRV_NAME ": failed to spi_register_driver\n");
		return -EINVAL;
	}
    return 0;
}

static void __exit spich_exit(void)
{
	pr_info(DRV_NAME ": %s\n", __func__);

	spi_unregister_driver(&spich_spi_driver);
}

late_initcall(spich_init);
module_exit(spich_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Google");
MODULE_DESCRIPTION("User mode SPI context hub interface");
