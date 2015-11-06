/* FPC1021a Area sensor driver
 *
 * Copyright (c) 2013 Fingerprint Cards AB <tech@fingerprints.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License Version 2
 * as published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>

#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>

#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/kthread.h>
#include <linux/poll.h>
#include <linux/ioctl.h>
#include <linux/types.h>
#ifdef CONFIG_OF
#include <linux/of_gpio.h>
#include <linux/fs.h>
#include <linux/of.h>
#endif

#include <linux/input.h>
#include <linux/clk.h>
#include <linux/clk/msm-clk.h>
#include <linux/irqchip/msm-gpio-irq.h>
#include <linux/irqchip/msm-mpm-irq.h>
#include <linux/err.h>

#include "fpc_btp.h"
#include "fpc_btp_regs.h"

#define SUPPORT_TRUSTZONE
/* #define FEATURE_FPC_USE_XO */
/* #define FEATURE_FPC_USE_PINCTRL */

#ifdef FEATURE_FPC_USE_PINCTRL
#include <linux/pinctrl/consumer.h>
#endif

#if defined(SUPPORT_TRUSTZONE)
#include <soc/qcom/scm.h>
#endif

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Fingerprint Cards AB <tech@fingerprints.com>");
MODULE_DESCRIPTION("FPC_BTP area sensor driver.");


#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
	const bool target_little_endian = true;
#else
	#warning BE target not tested!
	const bool target_little_endian = false;
#endif
/* -------------------------------------------------------------------- */
/* fpc_btp sensor commands                                              */
/* -------------------------------------------------------------------- */


/* -------------------------------------------------------------------- */
/* global variables                                                     */
/* -------------------------------------------------------------------- */
static int fpc_btp_device_count;

/* -------------------------------------------------------------------- */
/* fpc_btp data types                                                   */
/* -------------------------------------------------------------------- */
struct fpc_btp_data_t {
	struct spi_device      *spi;
	struct class           *class;
	struct device          *device;
	struct cdev            cdev;
	dev_t                  devno;
	u32                    cs_gpio;
	u32                    reset_gpio;
	u32                    irq_gpio;
	int                    irq;
	int                    int_done;
	int                    select_checkerboard;

#if defined(SUPPORT_TRUSTZONE)
	u32 qup_id;

	/* picntrl info */
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_active;
	struct pinctrl_state *pins_sleep;

	/* clk info */
	struct clk *core_clk;
	struct clk *iface_clk;
#endif
};

struct fpc_btp_reg_access_t {
	fpc_btp_reg_t reg;
	bool          write;
	u16           reg_size;
	u8            *dataptr;
};



/* -------------------------------------------------------------------- */
/* fpc_btp driver constants                                             */
/* -------------------------------------------------------------------- */
#define FPC_BTP_HWID_A           0x022a
#define FPC_BTP_HWID_B           0x0111

#define FPC_BTP_DEV_NAME            "btp"
#define FPC_BTP_CLASS_NAME          "fpsensor"
#define FPC_BTP_MAJOR               229
#define FPC_BTP_RESET_RETRIES       2
#define FPC_BTP_IOCTL_MAGIC_NO      0xFC
#define FPC_BTP_IOCTL_RESET         _IO(FPC_BTP_IOCTL_MAGIC_NO, 0)
#define FPC_BTP_IOCTL_INTR_STATUS   _IOR(FPC_BTP_IOCTL_MAGIC_NO, 1, int)
#define FPC_BTP_SPI_CLOCK_SPEED     (5 * 1000000U)



/* -------------------------------------------------------------------- */
/* function prototypes                                                  */
/* -------------------------------------------------------------------- */

static int __init fpc_btp_init(void);
static void __exit fpc_btp_exit(void);
static int fpc_btp_probe(struct spi_device *spi);
static int fpc_btp_remove(struct spi_device *spi);
static int fpc_btp_suspend(struct device *dev);
static int fpc_btp_resume(struct device *dev);
static int fpc_btp_open(struct inode *inode, struct file *file);
static ssize_t fpc_btp_write(struct file *file,
					const char *buff,
					size_t count, loff_t *ppos);
static ssize_t fpc_btp_read(struct file *file,
					char *buff,
					size_t count, loff_t *ppos);
static int fpc_btp_release(struct inode *inode, struct file *file);
static long fpc_btp_ioctl(struct file *filp,
					unsigned int cmd,
					unsigned long arg);
static unsigned int fpc_btp_poll(struct file *file, poll_table *wait);
static int fpc_btp_cleanup(struct fpc_btp_data_t *fpc_btp,
					struct spi_device *spidev);
static int fpc_btp_reset_init(struct fpc_btp_data_t *fpc_btp,
					struct fpc_btp_platform_data *pdata);
static int fpc_btp_irq_init(struct fpc_btp_data_t *fpc_btp,
					struct fpc_btp_platform_data *pdata);
static int fpc_btp_get_of_pdata(struct device *dev,
					struct fpc_btp_platform_data *pdata);
static int fpc_btp_gpio_reset(struct fpc_btp_data_t *fpc_btp);
static int fpc_btp_sleep(struct fpc_btp_data_t *fpc_btp, bool deep_sleep);
static int fpc_btp_create_class(struct fpc_btp_data_t *fpc_btp);
static int fpc_btp_create_device(struct fpc_btp_data_t *fpc_btp);
static irqreturn_t fpc_btp_interrupt(int irq, void *_fpc_btp);
static int fpc_btp_verify_hw_id(struct fpc_btp_data_t *fpc_btp);
static int fpc_btp_reg_access(struct fpc_btp_data_t *fpc_btp,
			      struct fpc_btp_reg_access_t *reg_data);



#define FPC_BTP_MK_REG_READ_BYTES(__dst, __reg, __count, __ptr) {	\
	(__dst).reg      = (__reg);					\
	(__dst).reg_size = (__count);					\
	(__dst).write    = false;					\
	(__dst).dataptr  = (__ptr); }

#define FPC_BTP_MK_REG_READ(__dst, __reg, __ptr) {	\
	(__dst).reg      = (__reg);			\
	(__dst).reg_size = FPC_BTP_REG_SIZE((__reg));	\
	(__dst).write    = false;			\
	(__dst).dataptr  = (u8 *)(__ptr); }

#define FPC_BTP_MK_REG_WRITE_BYTES(__dst, __reg, __count, __ptr) {	\
	(__dst).reg      = (__reg);					\
	(__dst).reg_size = (__count);					\
	(__dst).write    = true;					\
	(__dst).dataptr  = (__ptr); }

#define FPC_BTP_MK_REG_WRITE(__dst, __reg, __ptr) {	\
	(__dst).reg      = (__reg);			\
	(__dst).reg_size = FPC_BTP_REG_SIZE((__reg));	\
	(__dst).write    = true;			\
	(__dst).dataptr  = (u8 *)(__ptr); }

/* -------------------------------------------------------------------- */
/* External interface                                                   */
/* -------------------------------------------------------------------- */
module_init(fpc_btp_init);
module_exit(fpc_btp_exit);

static const struct dev_pm_ops fpc_btp_pm = {
	.suspend = fpc_btp_suspend,
	.resume = fpc_btp_resume
};

#ifdef CONFIG_OF
static struct of_device_id fpc_btp_of_match[] = {
	{ .compatible = "fpc,btp", },
	{}
};

/* MODULE_DEVICE_TABLE(of, fpc_btp_of_match); */
#endif

static struct spi_driver fpc_btp_driver = {
	.driver = {
		.name	= FPC_BTP_DEV_NAME,
		.bus	= &spi_bus_type,
		.owner	= THIS_MODULE,
		.pm     = &fpc_btp_pm,
#ifdef CONFIG_OF
		.of_match_table = fpc_btp_of_match,
#endif
	},
	.probe	= fpc_btp_probe,
	.remove	= fpc_btp_remove,
};

static const struct file_operations fpc_btp_fops = {
	.owner          = THIS_MODULE,
	.open           = fpc_btp_open,
	.write          = fpc_btp_write,
	.read           = fpc_btp_read,
	.release        = fpc_btp_release,
	.poll           = fpc_btp_poll,
	.unlocked_ioctl = fpc_btp_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = fpc_btp_ioctl,
#endif  /* CONFIG_COMPAT */
};

/* -------------------------------------------------------------------- */
/* function definitions                                                 */
/* -------------------------------------------------------------------- */
static int __init fpc_btp_init(void)
{
	if (spi_register_driver(&fpc_btp_driver))
		return -EINVAL;

	return 0;
}

/* -------------------------------------------------------------------- */
static void __exit fpc_btp_exit(void)
{
	pr_info("%s\n", __func__);

	spi_unregister_driver(&fpc_btp_driver);
}

#if defined(SUPPORT_TRUSTZONE)
/*
static int fpc_scm_call(struct scm_hdcp_req *req, u32 *resp)
{
	int ret = 0;

	if (!is_scm_armv8()) {
		ret = scm_call(SCM_SVC_HDCP, SCM_CMD_HDCP, (void *) req,
			     SCM_HDCP_MAX_REG * sizeof(struct scm_hdcp_req),
			     &resp, sizeof(*resp));
	} else {
		struct scm_desc desc;

		desc.args[0] = req[0].addr;
		desc.args[1] = req[0].val;
		//desc.args[2] = req[1].addr;
		//desc.args[3] = req[1].val;
		//desc.args[4] = req[2].addr;
		//desc.args[5] = req[2].val;
		//desc.args[6] = req[3].addr;
		//desc.args[7] = req[3].val;
		//desc.args[8] = req[4].addr;
		//desc.args[9] = req[4].val;
		desc.arginfo = SCM_ARGS(2);

		ret = scm_call2(SCM_SIP_FNID(SCM_SVC_HDCP, SCM_CMD_HDCP),
				&desc);
		*resp = desc.ret[0];
		if (ret)
			return ret;
	}

	return ret;
}
*/

/* Standard SPI device probe function  : Example of a slave probe function*/
static int spi_test_probe(struct fpc_btp_data_t *fpc_btp)
{
	int ret = 0;

	/* Get the pinctrl node */
	fpc_btp->pinctrl = devm_pinctrl_get(&fpc_btp->spi->dev);


	if (IS_ERR_OR_NULL(fpc_btp->pinctrl)) {
		dev_err(&fpc_btp->spi->dev,
				"%s: Failed to get pinctrl\n", __func__);
	  return PTR_ERR(fpc_btp->pinctrl);
	}

	/* Get the active setting */
	fpc_btp->pins_active = pinctrl_lookup_state(fpc_btp->pinctrl,
						   PINCTRL_STATE_DEFAULT);
	if (IS_ERR_OR_NULL(fpc_btp->pins_active)) {
		dev_err(&fpc_btp->spi->dev,
				"%s: Failed to get pinctrl state active\n",
				__func__);
		return PTR_ERR(fpc_btp->pins_active);
	}

	/* Get sleep settings */
	fpc_btp->pins_sleep = pinctrl_lookup_state(fpc_btp->pinctrl,
						  PINCTRL_STATE_SLEEP);
	if (IS_ERR_OR_NULL(fpc_btp->pins_sleep)) {
		dev_err(&fpc_btp->spi->dev,
				"%s: Failed to get pinctrl state sleep\n",
				__func__);
		return PTR_ERR(fpc_btp->pins_sleep);
	}

	/* Get iface_clk info */
	fpc_btp->iface_clk = clk_get(&fpc_btp->spi->dev, "iface_clk");
	if (IS_ERR(fpc_btp->iface_clk)) {
		dev_err(&fpc_btp->spi->dev,
				"%s: Failed to get iface_clk %ld\n",
				__func__,
				PTR_ERR(fpc_btp->iface_clk));

		return PTR_ERR(fpc_btp->iface_clk);
	}

	/* Get core_clk info */
	fpc_btp->core_clk = clk_get(&fpc_btp->spi->dev, "core_clk");
	if (IS_ERR(fpc_btp->core_clk)) {
		dev_err(&fpc_btp->spi->dev,
				"%s: Failed to get core_clk %p\n",
				__func__,
				fpc_btp->core_clk);
		return PTR_ERR(fpc_btp->core_clk);
	}

	/* Get the QUP ID (#1-12) */
	ret = of_property_read_u32(fpc_btp->spi->dev.of_node,
							   "qcom,qup-id",
							   &fpc_btp->qup_id);

	if (ret) {
		dev_err(&fpc_btp->spi->dev, "Error getting qup_id\n");
		return ret;
	}

	/* Grab SPI master lock for exclusive access */
	/* call spi_bus_unlock to unlock the lock. */
	spi_bus_lock(fpc_btp->spi->master);
	return 0;
}

/* -------------------------------------------------------------------- */
/* Example of how to change pinctrl settings */
static int spi_set_pinctrl(struct fpc_btp_data_t *fpc_btp, bool active)
{
	int ret;

	fpc_btp->pinctrl = devm_pinctrl_get(&fpc_btp->spi->dev);

	if (IS_ERR_OR_NULL(fpc_btp->pinctrl)) {
		pr_err("%s: Getting pinctrl handle failed\n", __func__);
		return -EINVAL;
	}

	if (active) { /* Change to active settings */
		fpc_btp->pins_active
		= pinctrl_lookup_state(fpc_btp->pinctrl, "default");

		if (IS_ERR_OR_NULL(fpc_btp->pins_active)) {
			pr_err("%s: Failed to get the suspend state	pinctrl handle\n",
				   __func__);
			return -EINVAL;
		}

		ret = pinctrl_select_state(fpc_btp->pinctrl,
			  fpc_btp->pins_active);
	} else {

		fpc_btp->pins_sleep = pinctrl_lookup_state(fpc_btp->pinctrl,
												   "sleep");

		if (IS_ERR_OR_NULL(fpc_btp->pins_sleep)) {
			pr_err("%s: Failed to get the suspend state pinctrl handle\n",
				   __func__);
				   return -EINVAL;
		}

		ret = pinctrl_select_state(fpc_btp->pinctrl,
			  fpc_btp->pins_sleep);
	}

	dev_err(&fpc_btp->spi->dev,
			"%s: pinctrl_select_state ret:%d Setting:%d\n",
			 __func__,
			 ret,
			 active);
	return ret;
}


/* Example of how to request fabric resources */
static int spi_set_fabric(struct fpc_btp_data_t *fpc_btp, bool active)
{
	int ret;
	struct spi_master *master = fpc_btp->spi->master;

	if (active)
		ret = master->prepare_transfer_hardware(master);
	else
		ret = master->unprepare_transfer_hardware(master);

	return ret;
}

/* Example of how to set up SPI clocks */
static int spi_set_clks(struct fpc_btp_data_t *fpc_btp, bool enable)
{
	int ret = 0;

	if (enable) {
		/* Enable the spi clocks */

		ret = clk_set_rate(fpc_btp->core_clk,
						   fpc_btp->spi->max_speed_hz);
		if (ret) {
			dev_err(&fpc_btp->spi->dev,
					"%s: Error setting clk_rate:%d\n",
					__func__,
					fpc_btp->spi->max_speed_hz);
		}

		ret = clk_prepare_enable(fpc_btp->core_clk);
		if (ret) {
			dev_err(&fpc_btp->spi->dev,
					"%s: Error enabling core clk\n",
					__func__);
		}

		ret = clk_prepare_enable(fpc_btp->iface_clk);

		if (ret) {
			dev_err(&fpc_btp->spi->dev,
			"%s: Error enabling iface clk\n",
			__func__);
		}

	} else {
		/* Disable the clocks */
		clk_disable_unprepare(fpc_btp->iface_clk);
		clk_disable_unprepare(fpc_btp->core_clk);
	}

	return ret;
}

/* Example of a change in BAM Pipe ownership */
static int spi_change_pipe_owner(struct fpc_btp_data_t *fpc_btp, bool to_tz)
{
	/* CMD ID collected from tzbsp_blsp.c to change Ownership */
	const u32 TZ_BLSP_MODIFY_OWNERSHIP_ID = 3;
	const u32 TZBSP_APSS_ID = 1;
	const u32 TZBSP_TZ_ID = 3;
	struct scm_desc desc; /* scm call descriptor */
	int ret;

	/* CMD ID to change BAM PIPE Owner*/
	desc.arginfo = SCM_ARGS(2);
	desc.args[0] = fpc_btp->qup_id;
	desc.args[1] = (to_tz) ? TZBSP_TZ_ID : TZBSP_APSS_ID;

	/* scm_call failed: func id 0x2000403, arginfo: 0x2, args:0:10.439 */
	ret = scm_call2(SCM_SIP_FNID(SCM_SVC_TZ,
					TZ_BLSP_MODIFY_OWNERSHIP_ID), &desc);

	return (ret || desc.ret[0]) ? -1 : 0;
}
#endif

/* -------------------------------------------------------------------- */
static int fpc_btp_reg_access(struct fpc_btp_data_t *fpc_btp,
			      struct fpc_btp_reg_access_t *reg_data)
{
	int error = 0;
	int cmd_size = 1;
	u8 tx[reg_data->reg_size+cmd_size];
	struct spi_message msg;

	struct spi_transfer data = {
		.cs_change = 0,
		.delay_usecs = 0,
		.speed_hz = FPC_BTP_SPI_CLOCK_SPEED,
		.tx_buf = tx,
		.rx_buf = (!reg_data->write)? tx : NULL,
		.len    = reg_data->reg_size+cmd_size,
		.tx_dma = 0,
		.rx_dma = 0,
		.bits_per_word = 0,
	};

	tx[0] = reg_data->reg;
	if (reg_data->write) {
		if (target_little_endian) {
			int src = 0;
			int dst = reg_data->reg_size - 1 + cmd_size;
			while (src < reg_data->reg_size) {
				tx[dst] = reg_data->dataptr[src];
				src++;
				dst--;
			}
		} else {
			memcpy(&tx[cmd_size], reg_data->dataptr,
				reg_data->reg_size);
		}
	}

	spi_message_init(&msg);
	spi_message_add_tail(&data, &msg);

	error = spi_sync(fpc_btp->spi, &msg);
	if (error) {
		dev_err(&fpc_btp->spi->dev, "spi_sync failed.\n");
	}

	if (!reg_data->write) {
		if (target_little_endian) {
			int src = reg_data->reg_size - 1+cmd_size;
			int dst = 0;

			while (dst < reg_data->reg_size) {
				reg_data->dataptr[dst] =
				tx[src];
				src--;
				dst++;
			}
		} else {
			memcpy(reg_data->dataptr, &tx[cmd_size],
				reg_data->reg_size);
		}
	}

	return error;
}


/* -------------------------------------------------------------------- */
static int fpc_btp_verify_hw_id(struct fpc_btp_data_t *fpc_btp)
{
	int error = 0;
	u16 version;
	struct fpc_btp_reg_access_t reg;

	dev_dbg(&fpc_btp->spi->dev, "%s\n", __func__);

	FPC_BTP_MK_REG_READ(reg, FPC_BTP_REG_HWID, &version);
	error = fpc_btp_reg_access(fpc_btp, &reg);

	if (error) {
		dev_err(&fpc_btp->spi->dev, "fpc_btp_reg_access failed.\n");
		return error;
	}

	if ((version != FPC_BTP_HWID_A) && (version != FPC_BTP_HWID_B)) {
		dev_err(&fpc_btp->spi->dev,
			"HWID mismatch: got 0x%x, expected 0x%x or 0x%x\n",
			version,
			FPC_BTP_HWID_A, FPC_BTP_HWID_B);

		return -EIO;
	}
	dev_info(&fpc_btp->spi->dev, "HWID: 0x%x\n", version);

	return error;
}

static int fpc_btp_spi_setup(struct fpc_btp_data_t *fpc_btp,
					struct fpc_btp_platform_data *pdata)
{
	int error = 0;

	printk(KERN_INFO "%s\n", __func__);

	fpc_btp->spi->mode = SPI_MODE_0;
	fpc_btp->spi->bits_per_word = 8;
	fpc_btp->spi->chip_select = 0;

	error = spi_setup(fpc_btp->spi);

	if (error) {
		dev_err(&fpc_btp->spi->dev, "spi_setup failed\n");
		goto out_err;
	}

out_err:
	return error;
}



/* -------------------------------------------------------------------- */
static int fpc_btp_probe(struct spi_device *spi)
{
	struct fpc_btp_platform_data *fpc_btp_pdata;
	struct fpc_btp_platform_data pdata_of;
	struct device *dev = &spi->dev;
	int error = 0;
	struct fpc_btp_data_t *fpc_btp = NULL;

	fpc_btp = kzalloc(sizeof(*fpc_btp), GFP_KERNEL);
	if (!fpc_btp) {
		dev_err(&spi->dev,
		"failed to allocate memory for struct fpc_btp_data\n");
		return -ENOMEM;
	}

	pr_info("%s\n", __func__);

	spi_set_drvdata(spi, fpc_btp);
	fpc_btp->spi = spi;
	fpc_btp->reset_gpio = -EINVAL;
	fpc_btp->irq_gpio   = -EINVAL;
	fpc_btp->cs_gpio    = -EINVAL;
	fpc_btp->irq        = -EINVAL;

	fpc_btp_pdata = spi->dev.platform_data;
	if (!fpc_btp_pdata) {
		error = fpc_btp_get_of_pdata(dev, &pdata_of);
		if (error)
			goto err;

		fpc_btp_pdata = &pdata_of;
		if (!fpc_btp_pdata) {
			dev_err(&fpc_btp->spi->dev,
					"spi->dev.platform_data is NULL.\n");
			error = -EINVAL;
			goto err;
		}
	}

	error = fpc_btp_reset_init(fpc_btp, fpc_btp_pdata);
	if(error)
		goto err;

	error = fpc_btp_irq_init(fpc_btp, fpc_btp_pdata);
	if(error)
		goto err;

	error = fpc_btp_spi_setup(fpc_btp, fpc_btp_pdata);
	if(error)
		goto err;

	error = fpc_btp_gpio_reset(fpc_btp);
	if(error)
		goto err;

	error = fpc_btp_verify_hw_id(fpc_btp);
	if(error)
		goto err;


#if defined(SUPPORT_TRUSTZONE)
	error = spi_test_probe(fpc_btp);
	if (error)
		goto err;
#endif


#if defined(SUPPORT_TRUSTZONE)
	/* 1. Change pinctrl to Active. */
	error = spi_set_pinctrl(fpc_btp, true);
	if (error)
		goto err;

	/* 2. Vote for the fabric resources. */
	error = spi_set_fabric(fpc_btp, true);
	if (error)
		goto err;

	/* 3. Enable the SPI clocks. */
	error = spi_set_clks(fpc_btp, true);
	if (error)
		goto err;

	/* 4. Change BAM Pipe ownership to TrustZone. */
	error = spi_change_pipe_owner(fpc_btp, true);
	if (error)
		goto err;
#endif

	error = fpc_btp_create_class(fpc_btp);
	if (error)
		goto err;

	error = fpc_btp_create_device(fpc_btp);
	if (error)
		goto err;

	cdev_init(&fpc_btp->cdev, &fpc_btp_fops);
	fpc_btp->cdev.owner = THIS_MODULE;

	error = cdev_add(&fpc_btp->cdev, fpc_btp->devno, fpc_btp_device_count);
	if (error) {
		dev_err(&fpc_btp->spi->dev, "cdev_add failed.\n");
		goto err_chrdev;
	}

#if !defined(SUPPORT_TRUSTZONE)
	error = fpc_btp_sleep(fpc_btp, true);
	if (error)
		goto err_cdev;
#endif
	pr_info("%s done!!\n", __func__);
	return 0;

#if !defined(SUPPORT_TRUSTZONE)
err_cdev:
	cdev_del(&fpc_btp->cdev);
#endif
err_chrdev:
	unregister_chrdev_region(fpc_btp->devno, 1);

err:
	fpc_btp_cleanup(fpc_btp, spi);
	pr_err("%s : error (%d) \n", __func__, error);
	return error;
}

/* -------------------------------------------------------------------- */
static int fpc_btp_remove(struct spi_device *spi)
{
	struct fpc_btp_data_t *fpc_btp = spi_get_drvdata(spi);

	pr_info("%s\n", __func__);

#if defined(SUPPORT_TRUSTZONE)
	spi_change_pipe_owner(fpc_btp, false);
	spi_set_clks(fpc_btp, false);
	spi_set_fabric(fpc_btp, false);
	spi_set_pinctrl(fpc_btp, false);
#endif

	fpc_btp_sleep(fpc_btp, true);

	cdev_del(&fpc_btp->cdev);

	unregister_chrdev_region(fpc_btp->devno, 1);

	fpc_btp_cleanup(fpc_btp, spi);

	return 0;
}

/* -------------------------------------------------------------------- */
static int fpc_btp_suspend(struct device *dev)
{

	struct fpc_btp_data_t *fpc_btp = NULL;
	int ret = 0;

	pr_info("%s\n", __func__);

	fpc_btp = dev_get_drvdata(dev);

	ret = spi_change_pipe_owner(fpc_btp, false);

	if (ret)
		dev_err(dev, "change pipe owner failed !!\n");

	spi_set_clks(fpc_btp, false);
	spi_set_fabric(fpc_btp, false);
	spi_set_pinctrl(fpc_btp, false);

	return 0;
}

/* -------------------------------------------------------------------- */
static int fpc_btp_resume(struct device *dev)
{

	struct fpc_btp_data_t *fpc_btp = NULL;
	int ret = 0;

	pr_info("%s\n", __func__);

	fpc_btp = dev_get_drvdata(dev);

	spi_set_pinctrl(fpc_btp, true);
	spi_set_fabric(fpc_btp, true);
	spi_set_clks(fpc_btp, true);

	ret = spi_change_pipe_owner(fpc_btp, true);

	if (ret)
		dev_err(dev, "change pipe owner failed !!\n");

	return 0;
}

/* -------------------------------------------------------------------- */
static int fpc_btp_open(struct inode *inode, struct file *file)
{
	struct fpc_btp_data_t *fpc_btp;
	int error = 0;

	pr_err("%s is called !!!\n", __func__);

	fpc_btp = container_of(inode->i_cdev, struct fpc_btp_data_t, cdev);
	file->private_data = fpc_btp;

	return error;
}

/* -------------------------------------------------------------------- */
static ssize_t fpc_btp_write(struct file *file, const char *buff,
					size_t count, loff_t *ppos)
{
	pr_info("%s\n", __func__);

	return -ENOTTY;
}

/* -------------------------------------------------------------------- */
static ssize_t fpc_btp_read(struct file *file, char *buff,
				size_t count, loff_t *ppos)
{
	int error = 0;
	/* u32	read_cnt; */
	/* u32	remain_cnt = 0; */

	return error;
}

/* -------------------------------------------------------------------- */
static int fpc_btp_release(struct inode *inode, struct file *file)
{
	struct fpc_btp_data_t *fpc_btp = file->private_data;
	int status = 0;
	dev_dbg(&fpc_btp->spi->dev, "%s\n", __func__);
	pr_err("%s is called !!!\n", __func__);

	return status;
}

/* -------------------------------------------------------------------- */
static long
fpc_btp_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int error = 0;
	struct fpc_btp_data_t *fpc_btp = filp->private_data;

	switch (cmd) {
	case FPC_BTP_IOCTL_RESET:
		error = fpc_btp_gpio_reset(fpc_btp);
		break;
	case FPC_BTP_IOCTL_INTR_STATUS:
		if (copy_to_user((int *)arg, &fpc_btp->int_done, sizeof(int)))
			return -EFAULT;

		break;
	default:
		error = -ENOTTY;
		break;
	}

	return error;
}

/* -------------------------------------------------------------------- */
static unsigned int fpc_btp_poll(struct file *file, poll_table *wait)
{
	struct fpc_btp_data_t *fpc_btp = file->private_data;
	unsigned int ret = 0;
	int i = 0;

	pr_info("%s\n", __func__);

	pr_info("%s interrupt_done = %d\n", __func__, fpc_btp->int_done);

	if (fpc_btp->int_done) {
		for (i = 0; i < 50000; i++)
			;
		pr_err("interrupt_done : ture");
		ret |= POLLHUP;
	} else {
		for (i = 0; i < 50000; i++)
			;
		pr_err("interrupt_done : false");
		ret |= (POLLIN | POLLRDNORM);
	}

	return ret;
}

/* -------------------------------------------------------------------- */
static int
fpc_btp_cleanup(struct fpc_btp_data_t *fpc_btp, struct spi_device *spidev)
{
	dev_err(&fpc_btp->spi->dev, "%s\n", __func__);

	if (!IS_ERR_OR_NULL(fpc_btp->device))
		device_destroy(fpc_btp->class, fpc_btp->devno);

	class_destroy(fpc_btp->class);

	if (fpc_btp->irq >= 0)
		free_irq(fpc_btp->irq, fpc_btp);

	if (gpio_is_valid(fpc_btp->irq_gpio))
		gpio_free(fpc_btp->irq_gpio);

	if (gpio_is_valid(fpc_btp->reset_gpio))
		gpio_free(fpc_btp->reset_gpio);

	kfree(fpc_btp);

	spi_set_drvdata(spidev, NULL);

	return 0;
}

/* -------------------------------------------------------------------- */
static int fpc_btp_reset_init(struct fpc_btp_data_t *fpc_btp,
					struct fpc_btp_platform_data *pdata)
{
	int error = 0;

	pr_info("%s\n", __func__);

	if (gpio_is_valid(pdata->reset_gpio)) {

		dev_info(&fpc_btp->spi->dev,
				 "Assign HW reset -> GPIO%d\n",
				 pdata->reset_gpio);

		error = gpio_request(pdata->reset_gpio, "fpc_btp_reset");

		if (error) {
			dev_err(&fpc_btp->spi->dev, "gpio_request (reset) failed.\n");
			return error;
		}

		fpc_btp->reset_gpio = pdata->reset_gpio;

		error = gpio_direction_output(fpc_btp->reset_gpio, 1);

		if (error) {
			dev_err(&fpc_btp->spi->dev, "gpio_direction_output(reset) failed.\n");
			return error;
		}
	} else {
		dev_err(&fpc_btp->spi->dev, "%s failed\n", __func__);
	}

	return error;
}

/* -------------------------------------------------------------------- */
static int
fpc_btp_irq_init(struct fpc_btp_data_t *fpc_btp,
				 struct fpc_btp_platform_data *pdata)
{
	int error = 0;

	pr_info("%s\n", __func__);

	if (gpio_is_valid(pdata->irq_gpio)) {

		dev_info(&fpc_btp->spi->dev,
				"Assign IRQ -> GPIO%d\n",
				pdata->irq_gpio);

		error = gpio_request(pdata->irq_gpio, "fpc_btp_irq");

		if (error) {
			dev_err(&fpc_btp->spi->dev,
					"gpio_request (irq) failed.\n");

			return error;
		}
		fpc_btp->irq_gpio = pdata->irq_gpio;

		error = gpio_direction_input(fpc_btp->irq_gpio);

		if (error) {
			dev_err(&fpc_btp->spi->dev,
					"gpio_direction_input (irq) failed.\n");
			return error;
		}
	} else {
		return -EINVAL;
	}

	fpc_btp->irq = gpio_to_irq(fpc_btp->irq_gpio);

	if (fpc_btp->irq < 0) {
		dev_err(&fpc_btp->spi->dev, "gpio_to_irq failed.\n");
		error = fpc_btp->irq;
		return error;
	}

	error = request_irq(fpc_btp->irq, fpc_btp_interrupt,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			"fpc_btp", fpc_btp);

	if (error) {
		dev_err(&fpc_btp->spi->dev,
				"request_irq %i failed.\n",
				fpc_btp->irq);

		fpc_btp->irq = -EINVAL;

		return error;
	}

	return error;
}

/* -------------------------------------------------------------------- */
#ifdef FEATURE_FPC_USE_XO
static int fpc_btp_spi_clk(struct fpc_btp_data_t *fpc_btp)
{
	int error = 0;
	struct clk *btp_clk;

	pr_info("%s\n", __func__);

	btp_clk = clk_get(&fpc_btp->spi->dev, "fpc_xo");
	if (IS_ERR(btp_clk)) {
		error = PTR_ERR(btp_clk);
		dev_err(&fpc_btp->spi->dev, "could not get clock\n");
		goto out_err;
	}

	/* We enable/disable the clock only to assure it works */
	error = clk_prepare_enable(btp_clk);
	if (error) {
		dev_err(&fpc_btp->spi->dev, "could not enable clock\n");
		goto out_err;
	}
	/* clk_disable_unprepare(btp_clk); */

out_err:
	return error;
}
#endif

/* -------------------------------------------------------------------- */
#ifdef CONFIG_OF
static int fpc_btp_get_of_pdata(struct device *dev,
					struct fpc_btp_platform_data *pdata)
{
	struct device_node *node = dev->of_node;

	u32 irq_prop = of_get_named_gpio(node, "fpc,gpio_irq",   0);
	u32 rst_prop = of_get_named_gpio(node, "fpc,gpio_reset", 0);
	u32 cs_prop  = of_get_named_gpio(node, "fpc,gpio_cs",    0);


	pr_info("%s\n", __func__);

	if (node == NULL) {
		dev_err(dev, "%s: Could not find OF device node\n", __func__);
		goto of_err;
	}

	if (!irq_prop || !rst_prop || !cs_prop) {
		dev_err(dev, "%s: Missing OF property\n", __func__);
		goto of_err;
	}

	pdata->irq_gpio   = irq_prop;
	pdata->reset_gpio = rst_prop;
	pdata->cs_gpio    = cs_prop;

	return 0;

of_err:
	pdata->reset_gpio = -EINVAL;
	pdata->irq_gpio   = -EINVAL;
	pdata->cs_gpio    = -EINVAL;

	return -ENODEV;
}

#else
static int fpc_btp_get_of_pdata(struct device *dev,
					struct fpc_btp_platform_data *pdata)
{
	int error = 0;

	pdata->irq_gpio = of_get_named_gpio(dev->of_node,
						"fpc,gpio_irq", 0);
	pdata->reset_gpio = of_get_named_gpio(dev->of_node,
						"fpc,gpio_reset", 0);
	pdata->cs_gpio = of_get_named_gpio(dev->of_node,
						"fpc,gpio_cs", 0);

	error = gpio_request(pdata->cs_gpio, "fpc_btp_cs");
	if (error < 0)
		pr_err("%s:Failed GPIO fpc_btp_cs request!!!\n",
			   __func__);

	pr_err("%s: irq(%d), reset(%d), cs(%d)\n", __func__,
				pdata->irq_gpio,
				pdata->reset_gpio,
				pdata->cs_gpio);

	return 0; /* -ENODEV; */
}
#endif

#ifdef FEATURE_FPC_USE_PINCTRL
static int fpc_btp_pinctrl_init(struct fpc_btp_data_t *fpc_btp)
{
	struct pinctrl *fpc_pinctrl;
	struct pinctrl_state *gpio_state_suspend;

	fpc_pinctrl = devm_pinctrl_get(&(fpc_btp->spi->dev));

	if (IS_ERR_OR_NULL(fpc_pinctrl)) {
		pr_err("%s: Getting pinctrl handle failed\n", __func__);
		return -EINVAL;
	}
	gpio_state_suspend
		= pinctrl_lookup_state(fpc_pinctrl, "gpio_fpc_suspend");

	if (IS_ERR_OR_NULL(gpio_state_suspend)) {
		pr_err("%s: Failed to get the suspend state pinctrl handle\n",
				__func__);
		return -EINVAL;
	}

	if (pinctrl_select_state(fpc_pinctrl, gpio_state_suspend)) {
		pr_err("%s: error on pinctrl_select_state\n", __func__);
		return -EINVAL;
	} else {
		pr_err("%s: success to set pinctrl_select_state\n",
			   __func__);
	}

	return 0;
}
#endif


/* -------------------------------------------------------------------- */
static int fpc_btp_gpio_reset(struct fpc_btp_data_t *fpc_btp)
{
	int error = 0;
	int counter = FPC_BTP_RESET_RETRIES;

	while (counter) {
		counter--;

		/* gpio_set_value(fpc_btp->reset_gpio, 0); */
		gpio_direction_output(fpc_btp->reset_gpio, 0);
		udelay(1000);
		/* mdelay(2); */

		/* gpio_set_value(fpc_btp->reset_gpio, 1); */
		gpio_direction_output(fpc_btp->reset_gpio, 1);
		udelay(1250);
		/* mdelay(3); */

		error = gpio_get_value(fpc_btp->irq_gpio) ? 0 : -EIO;
		if (!error) {
			counter = 0;
		} else {
			pr_info("%s timed out,retrying ...\n", __func__);

			udelay(1250);
		}
	}

	disable_irq(fpc_btp->irq);
	fpc_btp->int_done = 0;
	enable_irq(fpc_btp->irq);

	error = gpio_get_value(fpc_btp->irq_gpio) ? 0 : -EIO;
	if (error)
		dev_err(&fpc_btp->spi->dev, "reset failed.\n");
	else
		dev_dbg(&fpc_btp->spi->dev, "reset succeed.\n");

	return error;
}

/* -------------------------------------------------------------------- */
static int fpc_btp_sleep(struct fpc_btp_data_t *fpc_btp, bool deep_sleep)
{
	if (deep_sleep &&  gpio_is_valid(fpc_btp->reset_gpio)) {
		/* hyojin.an */
		/* Kenel panic because gpio_set_value(fpc_btp->cs_gpio, 0); */
		/* gpio_set_value(fpc_btp->reset_gpio, 0); */
		gpio_direction_output(fpc_btp->reset_gpio, 0);
		dev_dbg(&fpc_btp->spi->dev, "%s : reset_gpio -> 0\n", __func__);
	}

	if (deep_sleep && gpio_is_valid(fpc_btp->cs_gpio)) {
		/* gpio_set_value(fpc_btp->cs_gpio, 0); */
		gpio_direction_output(fpc_btp->cs_gpio, 0);
		dev_dbg(&fpc_btp->spi->dev, "%s : cs_gpio -> 0\n", __func__);
	}

	dev_dbg(&fpc_btp->spi->dev, "%s sleep OK\n", __func__);

	return 0;
}

/* -------------------------------------------------------------------- */
static int fpc_btp_create_class(struct fpc_btp_data_t *fpc_btp)
{
	int error = 0;

	dev_dbg(&fpc_btp->spi->dev, "%s\n", __func__);

	fpc_btp->class = class_create(THIS_MODULE, FPC_BTP_CLASS_NAME);

	if (IS_ERR(fpc_btp->class)) {
		dev_err(&fpc_btp->spi->dev, "failed to create class.\n");
		error = PTR_ERR(fpc_btp->class);
	}

	return error;
}

/* -------------------------------------------------------------------- */
static int fpc_btp_create_device(struct fpc_btp_data_t *fpc_btp)
{
	int error = 0;

	dev_dbg(&fpc_btp->spi->dev, "%s\n", __func__);

	alloc_chrdev_region(&fpc_btp->devno, 0, 1, FPC_BTP_DEV_NAME);
	fpc_btp_device_count++;

	fpc_btp->device = device_create(fpc_btp->class, NULL,
			fpc_btp->devno,
			NULL, "%s", FPC_BTP_DEV_NAME);

	if (IS_ERR(fpc_btp->device)) {
		dev_err(&fpc_btp->spi->dev, "device_create failed.\n");
		error = PTR_ERR(fpc_btp->device);
	}

	return error;
}

/* -------------------------------------------------------------------- */
static irqreturn_t fpc_btp_interrupt(int irq, void *_fpc_btp)
{
	struct fpc_btp_data_t *fpc_btp = _fpc_btp;

	if (gpio_get_value(fpc_btp->irq_gpio)) {
		fpc_btp->int_done = 1;
		return IRQ_HANDLED;
	} else {
		fpc_btp->int_done = 0;
		return IRQ_HANDLED;
	}
}
/* -------------------------------------------------------------------- */
