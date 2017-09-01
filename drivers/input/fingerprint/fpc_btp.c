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
#include <linux/input.h>
#include <linux/mutex.h>
#include <linux/regulator/consumer.h>
#include <linux/wakelock.h>
#include <linux/jiffies.h>
#include "qseecom_kernel.h"


#include "fpc_btp.h"
#include "fpc_btp_regs.h"
#include "fpc_log.h"

//#define SUPPORT_TZ_CMD_WAKELOCK
#define SUPPORT_TRUSTZONE
#define INTERRUPT_INPUT_REPORT

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

/* -------------------------------------------------------------------- */
/* fpc_btp data types                                                   */
/* -------------------------------------------------------------------- */
struct fpc_test_result {
	int  pass;
	int  value;
};

struct fpc_btp_data_t {
	struct spi_device      *spi;
	struct input_dev       *input;
	u32                    cs_gpio;
	u32                    reset_gpio;
	u32                    irq_gpio;
	int                    irq;
	struct mutex           p_mutex;
	bool                   power_on;
	struct fpc_btp_platform_data *platform_pdata;
#ifdef SUPPORT_TZ_CMD_WAKELOCK
	struct wake_lock       cmd_wake_lock;
#endif
#if defined(SUPPORT_TRUSTZONE)
	u32 qup_id;
	bool pipe_owner;

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

#define FPC_BTP_DEV_NAME         "btp"
#define FPC_BTP_RESET_RETRIES    2
#define FPC_BTP_SUPPLY_1V8       1800000UL
#define FPC_BTP_VOLTAGE_MIN      FPC_BTP_SUPPLY_1V8
#define FPC_BTP_VOLTAGE_MAX      FPC_BTP_SUPPLY_1V8
#define FPC_BTP_LOAD_UA          7000

/* CMD ID collected from tzbsp_blsp.c to change Ownership */
static const u32 TZ_BLSP_MODIFY_OWNERSHIP_ID = 3;
static const u32 TZBSP_APSS_ID = 1;
static const u32 TZBSP_TZ_ID = 3;

#ifdef INTERRUPT_INPUT_REPORT
#define FPC_BTP_INTERRUPT                        REL_MISC
#endif

/* -------------------------------------------------------------------- */
/* function prototypes                                                  */
/* -------------------------------------------------------------------- */

static int __init fpc_btp_init(void);
static void __exit fpc_btp_exit(void);
static int fpc_btp_probe(struct spi_device *spi);
static int fpc_btp_remove(struct spi_device *spi);
static int fpc_btp_suspend(struct device *dev);
static int fpc_btp_resume(struct device *dev);

static int fpc_btp_cleanup(struct fpc_btp_data_t *fpc_btp,
					struct spi_device *spidev);
static int fpc_btp_reset_init(struct fpc_btp_data_t *fpc_btp,
					struct fpc_btp_platform_data *pdata);
static int fpc_btp_cs_init(struct fpc_btp_data_t *fpc_btp,
					struct fpc_btp_platform_data *pdata);
static int fpc_btp_irq_init(struct fpc_btp_data_t *fpc_btp,
					struct fpc_btp_platform_data *pdata);
static int fpc_btp_get_of_pdata(struct device *dev,
					struct fpc_btp_platform_data *pdata);
static int fpc_btp_gpio_reset(struct fpc_btp_data_t *fpc_btp);
static int fpc_btp_sleep(struct fpc_btp_data_t *fpc_btp, bool deep_sleep);
static irqreturn_t fpc_btp_interrupt(int irq, void *_fpc_btp);
static int fpc_btp_verify_hw_id(struct fpc_btp_data_t *fpc_btp);
static int fpc_btp_reg_access(struct fpc_btp_data_t *fpc_btp,
			      struct fpc_btp_reg_access_t *reg_data);
static int fpc_btp_regulator_init(struct fpc_btp_data_t *fpc_btp,
					struct fpc_btp_platform_data *pdata);
static int fpc_btp_regulator_set(struct fpc_btp_data_t *fpc_btp, bool enable);



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
	PINFO("enter");

	spi_unregister_driver(&fpc_btp_driver);
}

#if defined(SUPPORT_TRUSTZONE)
static int spi_test_probe(struct fpc_btp_data_t *fpc_btp)
{
	int ret = 0;

	/* Get the pinctrl node */
	fpc_btp->pinctrl = devm_pinctrl_get(&fpc_btp->spi->dev);


	if (IS_ERR_OR_NULL(fpc_btp->pinctrl)) {
		PERR("Failed to get pinctrl");
	  return PTR_ERR(fpc_btp->pinctrl);
	}

	/* Get the active setting */
	fpc_btp->pins_active = pinctrl_lookup_state(fpc_btp->pinctrl,
						   PINCTRL_STATE_DEFAULT);
	if (IS_ERR_OR_NULL(fpc_btp->pins_active)) {
		PERR("Failed to get pinctrl state active");
		return PTR_ERR(fpc_btp->pins_active);
	}

	/* Get sleep settings */
	fpc_btp->pins_sleep = pinctrl_lookup_state(fpc_btp->pinctrl,
						  PINCTRL_STATE_SLEEP);
	if (IS_ERR_OR_NULL(fpc_btp->pins_sleep)) {
		PERR("Failed to get pinctrl state sleep");
		return PTR_ERR(fpc_btp->pins_sleep);
	}

	/* Get iface_clk info */
	fpc_btp->iface_clk = clk_get(&fpc_btp->spi->dev, "iface_clk");
	if (IS_ERR(fpc_btp->iface_clk)) {
		PERR("Failed to get iface_clk %ld",
			PTR_ERR(fpc_btp->iface_clk));

		return PTR_ERR(fpc_btp->iface_clk);
	}

	/* Get core_clk info */
	fpc_btp->core_clk = clk_get(&fpc_btp->spi->dev, "core_clk");
	if (IS_ERR(fpc_btp->core_clk)) {
		PERR("Failed to get core_clk %p", fpc_btp->core_clk);
		return PTR_ERR(fpc_btp->core_clk);
	}

	/* Get the QUP ID (#1-12) */
	ret = of_property_read_u32(fpc_btp->spi->dev.of_node,
							   "qcom,qup-id",
							   &fpc_btp->qup_id);

	if (ret) {
		PERR("Error getting qup_id");
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
		PERR("Getting pinctrl handle failed");
		return -EINVAL;
	}

	if (active) { /* Change to active settings */
		fpc_btp->pins_active
			= pinctrl_lookup_state(fpc_btp->pinctrl, "default");

		if (IS_ERR_OR_NULL(fpc_btp->pins_active)) {
			PERR("Failed to get the suspend state pinctrl handle");
			return -EINVAL;
		}

		ret = pinctrl_select_state(fpc_btp->pinctrl,
			  fpc_btp->pins_active);
	} else {

		fpc_btp->pins_sleep = pinctrl_lookup_state(fpc_btp->pinctrl,
			"sleep");

		if (IS_ERR_OR_NULL(fpc_btp->pins_sleep)) {
			PERR("Failed to get the suspend state pinctrl handle");
			return -EINVAL;
		}

		ret = pinctrl_select_state(fpc_btp->pinctrl,
			  fpc_btp->pins_sleep);
	}

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
			PERR("Error setting clk_rate:%d",
				fpc_btp->spi->max_speed_hz);
		}

		ret = clk_prepare_enable(fpc_btp->core_clk);
		if (ret) {
			PERR("Error enabling core clk");
		}

		ret = clk_prepare_enable(fpc_btp->iface_clk);

		if (ret) {
			PERR("Error enabling iface clk");
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
	struct scm_desc desc; /* scm call descriptor */
	int ret;

	/* CMD ID to change BAM PIPE Owner*/
	desc.arginfo = SCM_ARGS(2);
	desc.args[0] = fpc_btp->qup_id;
	desc.args[1] = (to_tz) ? TZBSP_TZ_ID : TZBSP_APSS_ID;

	/* scm_call failed: func id 0x2000403, arginfo: 0x2, args:0:10.439 */
	ret = scm_call2(SCM_SIP_FNID(SCM_SVC_TZ,
					TZ_BLSP_MODIFY_OWNERSHIP_ID), &desc);

	if(ret || desc.ret[0]) {
		PERR("ownership change failed!!");
		return -1;
	}

	/* set spi ownership flag */
	fpc_btp->pipe_owner = to_tz;

	return 0;
}

static int spi_set_prepare(struct fpc_btp_data_t *fpc_btp, bool enable)
{
	int ret = 0;

	mutex_lock(&fpc_btp->p_mutex);

	if(enable && !fpc_btp->pipe_owner)  {
#ifdef SUPPORT_TZ_CMD_WAKELOCK
		wake_lock(&fpc_btp->cmd_wake_lock);
#endif
		ret = spi_set_pinctrl(fpc_btp, true);
		if(ret) {
			PERR("spi_set_pinctrl failed : %d", ret);
			goto prepare_err;
		}

		ret = spi_set_fabric(fpc_btp, true);
		if(ret) {
			PERR("spi_set_fabric failed : %d", ret);
			goto prepare_err;
		}

		ret = spi_set_clks(fpc_btp, true);
		if(ret) {
			PERR("spi_set_clks failed : %d", ret);
			goto prepare_err;
		}

		ret = spi_change_pipe_owner(fpc_btp, true);
		if(ret) {
			PERR("change pipe owner failed : %d", ret);
			goto prepare_err;
		}

	} else if(!enable && fpc_btp->pipe_owner) {
		ret = spi_change_pipe_owner(fpc_btp, false);
		if(ret) {
			PERR("change pipe owner failed !!");
			goto prepare_err;
		}
		ret = spi_set_clks(fpc_btp, false);
		if(ret) {
			PERR("spi_set_clks failed : %d", ret);
			goto prepare_err;
		}

		ret = spi_set_fabric(fpc_btp, false);
		if(ret) {
			PERR("spi_set_fabric failed : %d", ret);
			goto prepare_err;
		}

		ret = spi_set_pinctrl(fpc_btp, false);
		if(ret) {
			PERR("spi_set_pinctrl failed : %d", ret);
			goto prepare_err;
		}
#ifdef SUPPORT_TZ_CMD_WAKELOCK
		wake_unlock(&fpc_btp->cmd_wake_lock);
#endif
	}

prepare_err:
	mutex_unlock(&fpc_btp->p_mutex);
	return ret;
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
		.speed_hz = fpc_btp->spi->max_speed_hz,
		.tx_buf = tx,
		.rx_buf = (!reg_data->write)? tx : NULL,
		.len    = reg_data->reg_size+cmd_size,
		.tx_dma = 0,
		.rx_dma = 0,
		.bits_per_word = 0,
	};

	tx[0] = reg_data->reg;
	if (reg_data->write) {
		int src = 0;
		int dst = reg_data->reg_size - 1 + cmd_size;
		while (src < reg_data->reg_size) {
			tx[dst] = reg_data->dataptr[src];
			src++;
			dst--;
		}

	}

	spi_message_init(&msg);
	spi_message_add_tail(&data, &msg);

	error = spi_sync(fpc_btp->spi, &msg);
	if (error) {
		PERR("spi_sync failed.");
	}

	if (!reg_data->write) {
		int src = reg_data->reg_size - 1+cmd_size;
		int dst = 0;

		while (dst < reg_data->reg_size) {
			reg_data->dataptr[dst] =
			tx[src];
			src--;
			dst++;
		}
	}

	return error;
}

/* -------------------------------------------------------------------- */
static ssize_t fpc_btp_get_hwid(struct fpc_btp_data_t *fpc_btp,
					u16 *version)
{
	int error = 0;
	struct fpc_btp_reg_access_t reg;

	PINFO("enter");

	FPC_BTP_MK_REG_READ(reg, FPC_BTP_REG_HWID, version);
	error = fpc_btp_reg_access(fpc_btp, &reg);

	return error;
}

/* -------------------------------------------------------------------- */
static int fpc_btp_verify_hw_id(struct fpc_btp_data_t *fpc_btp)
{
	int error = 0;
	u16 version = 0;

	PINFO("enter");

	error = fpc_btp_get_hwid(fpc_btp, &version);
	if (error) {
		PERR("fpc_btp_reg_access failed.");
		return error;
	}

	if ((version != FPC_BTP_HWID_A) && (version != FPC_BTP_HWID_B)) {
		PERR("HWID mismatch: got 0x%x, expected 0x%x or 0x%x\n",
			version,
			FPC_BTP_HWID_A, FPC_BTP_HWID_B);

		return -EIO;
	}
	PINFO("HWID: 0x%x", version);

	return error;
}

/* -------------------------------------------------------------------- */
static int fpc_btp_spi_setup(struct fpc_btp_data_t *fpc_btp,
					struct fpc_btp_platform_data *pdata)
{
	int error = 0;

	PDEBUG("enter");

	fpc_btp->spi->mode = SPI_MODE_0;
	fpc_btp->spi->bits_per_word = 8;
	fpc_btp->spi->chip_select = 0;

	error = spi_setup(fpc_btp->spi);

	if (error) {
		PERR("spi_setup failed");
		goto out_err;
	}

out_err:
	return error;
}


static ssize_t fpc_btp_show_qup_id(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct fpc_btp_data_t *fpc_btp = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", fpc_btp->qup_id);
}

static DEVICE_ATTR(qup_id, S_IRUGO,
		   fpc_btp_show_qup_id, NULL);


/* -------------------------------------------------------------------- */
static ssize_t fpc_btp_show_intstatus(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "not support\n");
}

static DEVICE_ATTR(intstatus, S_IRUGO,
		   fpc_btp_show_intstatus, NULL);

/* -------------------------------------------------------------------- */
static ssize_t fpc_btp_store_pinctrl_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct fpc_btp_data_t *fpc_btp = dev_get_drvdata(dev);
	int res = spi_set_pinctrl(fpc_btp, *buf == '1');
	return res ? res : count;
}

static DEVICE_ATTR(pinctrl, S_IWUSR, NULL, fpc_btp_store_pinctrl_set);

/* -------------------------------------------------------------------- */
static ssize_t fpc_btp_store_fabric_vote_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct fpc_btp_data_t *fpc_btp = dev_get_drvdata(dev);
	int res = spi_set_fabric(fpc_btp, *buf == '1');
	return res ? res : count;
}

static DEVICE_ATTR(fabric_vote, S_IWUSR, NULL, fpc_btp_store_fabric_vote_set);

/* -------------------------------------------------------------------- */
static ssize_t fpc_btp_store_clk_enable_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct fpc_btp_data_t *fpc_btp = dev_get_drvdata(dev);
	int res = spi_set_clks(fpc_btp, *buf == '1');
	return res ? res : count;
}

static DEVICE_ATTR(clk_enable, S_IWUSR, NULL, fpc_btp_store_clk_enable_set);

/* -------------------------------------------------------------------- */
static ssize_t fpc_btp_store_spi_owner_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct fpc_btp_data_t *fpc_btp = dev_get_drvdata(dev);
	int res = spi_change_pipe_owner(fpc_btp, *buf == '1');
	return res ? res : count;
}

static DEVICE_ATTR(spi_owner, S_IWUSR, NULL, fpc_btp_store_spi_owner_set);

/* -------------------------------------------------------------------- */
static ssize_t fpc_btp_store_spi_prepare_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct fpc_btp_data_t *fpc_btp = dev_get_drvdata(dev);
	int res;
	bool to_tz;

	if(*buf == '1')
	  to_tz = true;
	else if(*buf == '0')
	  to_tz = false;
	else
	  return -EINVAL;

	res = spi_set_prepare(fpc_btp, to_tz);

	return res ? res : count;
}

static ssize_t fpc_btp_show_spi_prepare(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct fpc_btp_data_t *fpc_btp = dev_get_drvdata(dev);

	if(fpc_btp->pipe_owner)
	  return sprintf(buf, "%d \n", TZBSP_TZ_ID);
	else
	  return sprintf(buf, "%d \n", TZBSP_APSS_ID);
}

static DEVICE_ATTR(spi_prepare, S_IRUGO | S_IWUSR,
		fpc_btp_show_spi_prepare, fpc_btp_store_spi_prepare_set);

/* -------------------------------------------------------------------- */
static ssize_t fpc_btp_store_regulator(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct fpc_btp_data_t *fpc_btp = dev_get_drvdata(dev);
	bool enable;

	if(*buf == '1')
		enable = true;
	else
		enable = false;

	if(fpc_btp_regulator_set(fpc_btp,  enable) < 0)
		PERR("regulator (%d) fail", enable);

	return count;
}

static ssize_t fpc_btp_show_regulator(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct fpc_btp_data_t *fpc_btp = dev_get_drvdata(dev);

	return sprintf(buf, "%d \n", fpc_btp->power_on);
}

static DEVICE_ATTR(regulator, S_IRUGO | S_IWUSR,
		fpc_btp_show_regulator, fpc_btp_store_regulator);

/* -------------------------------------------------------------------- */
static ssize_t fpc_btp_store_gpio(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct fpc_btp_data_t *fpc_btp = dev_get_drvdata(dev);
	int error;

	if(*buf == '1' && !fpc_btp->power_on) {
		error = regulator_enable(fpc_btp->platform_pdata->vreg);
		if(error < 0) {
			PERR("regulator enable fail");
			return count;
		}

		if((error = fpc_btp_gpio_reset(fpc_btp)))
			PERR("reset gpio init fail");

		enable_irq(fpc_btp->irq);
		fpc_btp->power_on = true;
	}
	else if(*buf == '0' && fpc_btp->power_on) {
		disable_irq(fpc_btp->irq);
		gpio_direction_output(fpc_btp->reset_gpio, 0);
		error = regulator_disable(fpc_btp->platform_pdata->vreg);
		if(error < 0) {
			PERR("regulator disable fail");
			return count;
		}
		fpc_btp->power_on = false;
	}

	return count;
}

static ssize_t fpc_btp_show_gpio(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct fpc_btp_data_t *fpc_btp = dev_get_drvdata(dev);

	return sprintf(buf, "%d \n", fpc_btp->power_on);
}

static DEVICE_ATTR(gpio, S_IRUGO | S_IWUSR,
		fpc_btp_show_gpio, fpc_btp_store_gpio);

/* -------------------------------------------------------------------- */

static struct attribute *fpc_btp_attributes[] = {
	&dev_attr_intstatus.attr,
	&dev_attr_qup_id.attr,
	&dev_attr_pinctrl.attr,
	&dev_attr_fabric_vote.attr,
	&dev_attr_clk_enable.attr,
	&dev_attr_spi_owner.attr,
	&dev_attr_spi_prepare.attr,
	&dev_attr_regulator.attr,
	&dev_attr_gpio.attr,
	NULL
};


static const struct attribute_group fpc_btp_attr_group = {
	.attrs = fpc_btp_attributes,
};


/* -------------------------------------------------------------------- */
static int fpc_btp_probe(struct spi_device *spi)
{
	struct fpc_btp_platform_data *fpc_btp_pdata;
	struct device *dev = &spi->dev;
	int error = 0;
	struct fpc_btp_data_t *fpc_btp = NULL;

	PDEBUG("enter");

	fpc_btp = devm_kzalloc(dev, sizeof(*fpc_btp), GFP_KERNEL);
	if (!fpc_btp) {
		PERR("failed to allocate memory for struct fpc_btp_data");
		return -ENOMEM;
	}

	spi_set_drvdata(spi, fpc_btp);
	fpc_btp->spi = spi;
	fpc_btp->reset_gpio = -EINVAL;
	fpc_btp->irq_gpio   = -EINVAL;
	fpc_btp->cs_gpio    = -EINVAL;
	fpc_btp->irq        = -EINVAL;
	fpc_btp->pipe_owner = false;
	mutex_init(&fpc_btp->p_mutex);
#ifdef SUPPORT_TZ_CMD_WAKELOCK
	wake_lock_init(&fpc_btp->cmd_wake_lock, WAKE_LOCK_SUSPEND, "csfp_wakelock");
#endif
	fpc_btp->platform_pdata = NULL;

	if(spi->dev.of_node) {
		fpc_btp_pdata = devm_kzalloc(dev, sizeof(*fpc_btp_pdata), GFP_KERNEL);
		if (!fpc_btp_pdata) {
			PERR("Failed to allocate memory");
			return -ENOMEM;
		}

		spi->dev.platform_data = fpc_btp_pdata;
		fpc_btp->platform_pdata = fpc_btp_pdata;
		error = fpc_btp_get_of_pdata(dev, fpc_btp_pdata);
		if (error)
			goto err;

	} else {
		fpc_btp_pdata = spi->dev.platform_data;
	}

	if((error = fpc_btp_regulator_init(fpc_btp, fpc_btp_pdata)) < 0)
		goto err;

	if((error = fpc_btp_regulator_set(fpc_btp, true)))
			goto err;

	if((error = fpc_btp_reset_init(fpc_btp, fpc_btp_pdata)))
		goto err;

	if((error = fpc_btp_cs_init(fpc_btp, fpc_btp_pdata)))
		goto err;

	if((error = fpc_btp_irq_init(fpc_btp, fpc_btp_pdata)))
		goto err;

	if((error = fpc_btp_spi_setup(fpc_btp, fpc_btp_pdata)))
		goto err;

	if((error = fpc_btp_gpio_reset(fpc_btp)))
		goto err;

	if((error = fpc_btp_verify_hw_id(fpc_btp)))
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

	/* 5. Change ownership to Apss and disable clk, fabric, pinctrl */
	error = spi_set_prepare(fpc_btp, false);
	if(error)
		goto err;

#endif

	/* register input device */
	fpc_btp->input = input_allocate_device();
	if(!fpc_btp->input) {
		PERR("input_allocate_deivce failed.");
		error = -ENOMEM;
		goto err;
	}

	fpc_btp->input->name = "fingerprint";
	fpc_btp->input->dev.init_name = "lge_fingerprint";

#ifdef INTERRUPT_INPUT_REPORT
	input_set_capability(fpc_btp->input, EV_REL, FPC_BTP_INTERRUPT);

	error = devm_request_threaded_irq(dev, fpc_btp->irq, NULL,
			fpc_btp_interrupt,
			IRQF_TRIGGER_RISING | IRQF_ONESHOT,
			"fpc_btp", fpc_btp);

	if (error) {
		PERR("request_irq %i failed.", fpc_btp->irq);
		fpc_btp->irq = -EINVAL;
		goto err;
	}
	disable_irq(fpc_btp->irq);
	enable_irq(fpc_btp->irq);
#endif
	input_set_drvdata(fpc_btp->input, fpc_btp);
	error = input_register_device(fpc_btp->input);
	if(error) {
		PERR("input_register_device failed.");
		input_free_device(fpc_btp->input);
		goto err;
	}

	if(sysfs_create_group(&fpc_btp->input->dev.kobj, &fpc_btp_attr_group)) {
		PERR("sysfs_create_group failed.");
		goto err_sysfs;
	}

	PINFO("done!!");

	return 0;

err_sysfs:
	input_free_device(fpc_btp->input);
	input_unregister_device(fpc_btp->input);

err:
	mutex_destroy(&fpc_btp->p_mutex);
	fpc_btp_cleanup(fpc_btp, spi);
	return error;
}

/* -------------------------------------------------------------------- */
static int fpc_btp_remove(struct spi_device *spi)
{
	struct fpc_btp_data_t *fpc_btp = spi_get_drvdata(spi);

	PINFO("enter");

#if defined(SUPPORT_TRUSTZONE)
	spi_set_prepare(fpc_btp, false);
#endif
	fpc_btp_sleep(fpc_btp, true);

	sysfs_remove_group(&fpc_btp->input->dev.kobj, &fpc_btp_attr_group);
	input_free_device(fpc_btp->input);
	input_unregister_device(fpc_btp->input);

	fpc_btp_cleanup(fpc_btp, spi);

	return 0;
}


/* -------------------------------------------------------------------- */
static int fpc_btp_suspend(struct device *dev)
{

	struct fpc_btp_data_t *fpc_btp = dev_get_drvdata(dev);
	int ret = 0;

	PINFO("enter");

	if(fpc_btp->pipe_owner) {
		PERR("spi owner is TZ!!");
	}

	ret = spi_set_prepare(fpc_btp, false);
	if(ret)
		PERR("spi_set_prepare failed.");

	disable_irq(fpc_btp->irq);
	gpio_direction_output(fpc_btp->reset_gpio, 0);
	if(fpc_btp_regulator_set(fpc_btp, false) < 0)
		PERR("reguator off fail");

	return 0;
}

/* -------------------------------------------------------------------- */
static int fpc_btp_resume(struct device *dev)
{
	struct fpc_btp_data_t *fpc_btp = dev_get_drvdata(dev);

	PINFO("enter");

	if(fpc_btp_regulator_set(fpc_btp, true) < 0)
		PERR("reguator on fail");

	if(fpc_btp_gpio_reset(fpc_btp))
		PINFO("reset gpio init fail");

	enable_irq(fpc_btp->irq);

	return 0;
}

/* -------------------------------------------------------------------- */
static int
fpc_btp_cleanup(struct fpc_btp_data_t *fpc_btp, struct spi_device *spidev)
{
	PINFO("enter");
#ifdef SUPPORT_TZ_CMD_WAKELOCK
	wake_lock_destroy(&fpc_btp->cmd_wake_lock);
#endif
	if (gpio_is_valid(fpc_btp->irq_gpio))
		gpio_free(fpc_btp->irq_gpio);

	if (gpio_is_valid(fpc_btp->reset_gpio))
		gpio_free(fpc_btp->reset_gpio);

	spi_set_drvdata(spidev, NULL);

	return 0;
}

/* -------------------------------------------------------------------- */
static int fpc_btp_reset_init(struct fpc_btp_data_t *fpc_btp,
					struct fpc_btp_platform_data *pdata)
{
	int error = 0;

	PINFO("enter");

	if (gpio_is_valid(pdata->reset_gpio)) {

		PINFO("Assign HW reset -> GPIO%d",
				 pdata->reset_gpio);

		error = gpio_request(pdata->reset_gpio, "fpc_btp_reset");

		if (error) {
			PERR("gpio_request (reset) failed.");
			return error;
		}

		fpc_btp->reset_gpio = pdata->reset_gpio;

		error = gpio_direction_output(fpc_btp->reset_gpio, 1);

		if (error) {
			PERR("gpio_direction_output(reset) failed.");
			return error;
		}
	} else {
		PERR("failed");
	}

	return error;
}

/* -------------------------------------------------------------------- */
static int fpc_btp_cs_init(struct fpc_btp_data_t *fpc_btp,
					struct fpc_btp_platform_data *pdata)
{
	int error = 0;

	PINFO("enter");

	if (gpio_is_valid(pdata->cs_gpio)) {

		PINFO("Assign CS -> GPIO%d", pdata->cs_gpio);

		error = gpio_request(pdata->cs_gpio, "fpc_btp_cs");

		if (error) {
			PERR("gpio_request (cs) failed.");
			return error;
		}

		fpc_btp->cs_gpio = pdata->cs_gpio;

		error = gpio_direction_output(fpc_btp->cs_gpio, 0);

		if (error) {
			PERR("gpio_direction_output(cs) failed.");
			return error;
		}

	} else {
		PERR("failed");
	}

	return error;
}

/* -------------------------------------------------------------------- */
static int
fpc_btp_irq_init(struct fpc_btp_data_t *fpc_btp,
				 struct fpc_btp_platform_data *pdata)
{
	int error = 0;

	PINFO("enter");

	if (gpio_is_valid(pdata->irq_gpio)) {

		PINFO("Assign IRQ -> GPIO%d", pdata->irq_gpio);

		error = gpio_request(pdata->irq_gpio, "fpc_btp_irq");
		if (error) {
			PERR("gpio_request (irq) failed.");

			return error;
		}
		fpc_btp->irq_gpio = pdata->irq_gpio;

		error = gpio_direction_input(fpc_btp->irq_gpio);

		if (error) {
			PERR("gpio_direction_input (irq) failed.");
			return error;
		}
	} else {
		return -EINVAL;
	}

	fpc_btp->irq = gpio_to_irq(fpc_btp->irq_gpio);

	if (fpc_btp->irq < 0) {
		PERR("gpio_to_irq failed.");
		error = fpc_btp->irq;
		return error;
	}

#ifndef INTERRUPT_INPUT_REPORT
	error = request_irq(fpc_btp->irq, fpc_btp_interrupt,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			"fpc_btp", fpc_btp);

	if (error) {
		PERR("request_irq %i failed.", fpc_btp->irq);
		fpc_btp->irq = -EINVAL;
		return error;
	}
#endif
	return error;
}

/* -------------------------------------------------------------------- */
#ifdef FEATURE_FPC_USE_XO
static int fpc_btp_spi_clk(struct fpc_btp_data_t *fpc_btp)
{
	int error = 0;
	struct clk *btp_clk;

	PINFO("enter");

	btp_clk = clk_get(&fpc_btp->spi->dev, "fpc_xo");
	if (IS_ERR(btp_clk)) {
		error = PTR_ERR(btp_clk);
		PERR("could not get clock");
		goto out_err;
	}

	/* We enable/disable the clock only to assure it works */
	error = clk_prepare_enable(btp_clk);
	if (error) {
		PERR("could not enable clock");
		goto out_err;
	}
	/* clk_disable_unprepare(btp_clk); */

out_err:
	return error;
}
#endif

/* -------------------------------------------------------------------- */
static int fpc_btp_get_of_pdata(struct device *dev,
					struct fpc_btp_platform_data *pdata)
{
	struct device_node *node = dev->of_node;

	u32 irq_prop = of_get_named_gpio(node, "fpc,gpio_irq",   0);
	u32 rst_prop = of_get_named_gpio(node, "fpc,gpio_reset", 0);
	u32 cs_prop  = of_get_named_gpio(node, "fpc,gpio_cs",    0);

	PINFO("enter");

	if (node == NULL) {
		PERR("Could not find OF device node");
		goto of_err;
	}

	if (!irq_prop || !rst_prop || !cs_prop) {
		PINFO("Missing OF property");
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

/* -------------------------------------------------------------------- */
static int fpc_btp_regulator_init(struct fpc_btp_data_t *fpc_btp,
					struct fpc_btp_platform_data *pdata)
{
	int error = 0;
	struct regulator *vreg;

	PINFO("enter!!");

	fpc_btp->power_on = false;
	pdata->vreg = NULL;
	vreg = regulator_get(&fpc_btp->spi->dev, "fpc,vddio");
	if (IS_ERR(vreg)) {
		error = PTR_ERR(vreg);
		PERR("Regulator get failed, error=%d", error);
		return error;
	}

	if (regulator_count_voltages(vreg) > 0) {
		error = regulator_set_voltage(vreg,
			FPC_BTP_VOLTAGE_MIN, FPC_BTP_VOLTAGE_MAX);
		if (error) {
			PERR("regulator set_vtg failed error=%d", error);
			goto err;
		}
	}

	if(regulator_count_voltages(vreg) > 0) {
		error = regulator_set_optimum_mode(vreg, FPC_BTP_LOAD_UA);
		if(error < 0) {
			PERR("unable to set current");
			goto err;
		}
	}
	pdata->vreg = vreg;
	return error;
err:
	regulator_put(vreg);
	return error;
}


/* -------------------------------------------------------------------- */
static int fpc_btp_regulator_set(struct fpc_btp_data_t *fpc_btp, bool enable)
{
	int error = 0;
	struct fpc_btp_platform_data *pdata = fpc_btp->platform_pdata;

	PINFO("power %s!!", (enable) ? "on" : "off");

	if(enable) {
		if(!fpc_btp->power_on)
			error = regulator_enable(pdata->vreg);
	} else {
		if(fpc_btp->power_on)
			error = regulator_disable(pdata->vreg);
	}

	if(error < 0)
		PERR("can't set(%d) regulator, error(%d)", enable, error);
	else
		fpc_btp->power_on = enable;

	return error;
}


#ifdef FEATURE_FPC_USE_PINCTRL
static int fpc_btp_pinctrl_init(struct fpc_btp_data_t *fpc_btp)
{
	struct pinctrl *fpc_pinctrl;
	struct pinctrl_state *gpio_state_suspend;

	fpc_pinctrl = devm_pinctrl_get(&(fpc_btp->spi->dev));

	if (IS_ERR_OR_NULL(fpc_pinctrl)) {
		PERR("Getting pinctrl handle failed");
		return -EINVAL;
	}
	gpio_state_suspend
		= pinctrl_lookup_state(fpc_pinctrl, "gpio_fpc_suspend");

	if (IS_ERR_OR_NULL(gpio_state_suspend)) {
		PERR("Failed to get the suspend state pinctrl handle");
		return -EINVAL;
	}

	if (pinctrl_select_state(fpc_pinctrl, gpio_state_suspend)) {
		PERR("error on pinctrl_select_state");
		return -EINVAL;
	} else {
		PERR("success to set pinctrl_select_state");
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
			PINFO("timed out,retrying ...");

			udelay(1250);
		}
	}

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
		PDEBUG("reset_gpio -> 0");
	}

	if (deep_sleep && gpio_is_valid(fpc_btp->cs_gpio)) {
		/* gpio_set_value(fpc_btp->cs_gpio, 0); */
		gpio_direction_output(fpc_btp->cs_gpio, 0);
		PDEBUG("cs_gpio -> 0");
	}

	PDEBUG("sleep OK");

	return 0;
}

/* -------------------------------------------------------------------- */
static irqreturn_t fpc_btp_interrupt(int irq, void *_fpc_btp)
{
	struct fpc_btp_data_t *fpc_btp = _fpc_btp;
	int gpio_value;

	gpio_value = gpio_get_value(fpc_btp->irq_gpio);

#ifdef INTERRUPT_INPUT_REPORT
	if(gpio_value) {
		input_report_rel(fpc_btp->input, FPC_BTP_INTERRUPT, 1);
		input_sync(fpc_btp->input);
	}
#endif
	return IRQ_HANDLED;
}
/* -------------------------------------------------------------------- */

