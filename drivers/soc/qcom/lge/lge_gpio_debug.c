/*
 * arch/arm/mach-msm/lge/lge_gpio_debug.c
 *
 * Copyright (C) 2013-2014 LGE, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/ctype.h>
#include <linux/debugfs.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/bitops.h>
#include <linux/of.h>
#include <linux/spmi.h>

#include <soc/qcom/lge/board_lge.h>

/* MSM GPIOs */
#define GPIO_CONFIG(tlmm, gpio)        (tlmm->base + 0x1000 + (0x10 * (gpio)))
#define GPIO_IN_OUT(tlmm, gpio)        (tlmm->base + 0x1004 + (0x10 * (gpio)))
#define GPIO_INTR_CFG(tlmm, gpio)      (tlmm->base + 0x1008 + (0x10 * (gpio)))
#define GPIO_INTR_STATUS(tlmm, gpio)   (tlmm->base + 0x100c + (0x10 * (gpio)))
#define GPIO_DIR_CONN_INTR(tlmm, intr) (tlmm->base + 0x2800 + (0x04 * (intr)))

static char *pull[] = {"NO_PULL", "PULL_DOWN", "KEEPER", "PULL_UP"};

#define PMIC_SPMI_SID_PRIMARY    0
#define PMIC_SPMI_SID_SECONDARY  2
/* PM GPIOs */
#define PMIC_GPIO_REG            0xC004
#define PMIC_GPIO_MODE(gpio)     (PMIC_GPIO_REG + 0x3C + (0x100 * (gpio-1)))
#define PMIC_GPIO_INT(gpio)      (PMIC_GPIO_REG + 0x11 + (0x100 * (gpio-1)))

static char *gpio_mode[] = {"IN", "OUT", "IN/OUT", "Reserved"};
static char *gpio_pull[] = {"PULL_UP_30uA", "PULL_UP_1.5uA",
		"PULL_UP_31.5uA", "PULL_UP_1.5uA+30uA", "PULL_DOWN_10uA",
		"NO_PULL", "Reserved", "Reserved"};
static char *gpio_out[] = {"CMOS", "NMOS", "PMOS", "N/A"};
static char *gpio_drv[] = {"Reserved", "Low", "Medium", "High"};

/* PM MPPs */
#define PMIC_MPP_REG             0xA004
#define PMIC_MPP_MODE(gpio)      (PMIC_MPP_REG + 0x3C + (0x100 * (gpio-1)))
#define PMIC_MPP_INT(gpio)       (PMIC_MPP_REG + 0x11 + (0x100 * (gpio-1)))

static char *mpp_mode[] = {"D_IN", "D_OUT", "D_IN/OUT",
	"Bidirection", "A_IN", "A_OUT", "Current Sink", "Reserved"};
static char *mpp_pull[] = {"0.6kohm", "10 kohm", "30 kohm", "Open"};

static DEFINE_SPINLOCK(gpios_lock);
static struct dentry *debugfs_base;
static struct dentry *tlmm_dentry;
static struct spmi_controller *ctrl;
static u32 debug_suspend;

struct lge_gpio_debug_data {
	unsigned int n_msm_gpio;
	unsigned int n_pm_gpio;
	unsigned int n_pm_mpp;
	unsigned int n_pmi_gpio;
	unsigned int n_pmi_mpp;
	void __iomem *base;
	struct device *dev;
};
static struct lge_gpio_debug_data *dbgdata;

void gpio_debug_print(void)
{
	int i;
	unsigned cfg;
	unsigned out;
	unsigned intr;
	unsigned char d[6];
	unsigned long flags;

	if (likely(!debug_suspend))
		return;

	spin_lock_irqsave(&gpios_lock, flags);

	pr_cont("MSM GPIOs:\n");
	for (i = 0; i < dbgdata->n_msm_gpio; i++) {
		cfg = __raw_readl(GPIO_CONFIG(dbgdata, i));
		out = __raw_readl(GPIO_IN_OUT(dbgdata, i));
		intr = __raw_readl(GPIO_INTR_STATUS(dbgdata, i));

		pr_cont("GPIO[%d]: [FS]0x%x, [DIR]%s, [PULL]%s, [DRV]%dmA",
					i, (cfg&0x3C)>>2,
					((cfg&0x200)>>9) ? "OUT" : "IN",
					pull[(cfg&0x3)],
					(((cfg&0x1C0)>>6)<<1)+2);

		if ((cfg&0x200)>>9)
			pr_cont(", [VAL]%s", ((out>>1)&0x1) ? "HIGH" : "LOW");

		if (intr&0x1)
			pr_cont(", [INT]HIGH");

		pr_cont("\n");
	}

	pr_cont("PRIMARY PMIC GPIOs:\n");
	for (i = 1; i < dbgdata->n_pm_gpio+1; i++) {
		spmi_ext_register_readl(ctrl, PMIC_SPMI_SID_PRIMARY,
						PMIC_GPIO_MODE(i), d, 6);

		pr_cont("GPIO[%d]: [DIR]%s, [PULL]%s, [OUT]%s, [DRV]%s",
					i, gpio_mode[(d[0]&0x70)>>4],
					gpio_pull[d[2]&0x7],
					gpio_out[d[5]&0x30>>4],
					gpio_drv[d[5]&0x3]);

		spmi_ext_register_readl(ctrl, PMIC_SPMI_SID_PRIMARY,
						PMIC_GPIO_INT(i), d, 1);

		if (d[0])
			pr_cont(", [INT]Enable");

		pr_cont("\n");
	}

	pr_cont("PRIMARY PMIC MPPs:\n");
	for (i = 1; i < dbgdata->n_pm_mpp+1; i++) {
		spmi_ext_register_readl(ctrl, PMIC_SPMI_SID_PRIMARY,
					PMIC_MPP_MODE(i), d, 3);

		pr_cont("MPP[%d]: [DIR]%s, [PULL]%s",
					i, mpp_mode[(d[0]&0x70)>>4],
					mpp_pull[d[2]&0x7]);

		spmi_ext_register_readl(ctrl, PMIC_SPMI_SID_PRIMARY,
					PMIC_MPP_INT(i), d, 1);

		if (d[0])
			pr_cont(", [INT]Enable");

		pr_cont("\n");
	}

	pr_cont("SECONDARY PMIC GPIOs:\n");
	for (i = 1; i < dbgdata->n_pmi_gpio+1; i++) {
		spmi_ext_register_readl(ctrl, PMIC_SPMI_SID_SECONDARY,
					PMIC_GPIO_MODE(i), d, 6);

		pr_cont("GPIO[%d]: [DIR]%s, [PULL]%s, [OUT]%s, [DRV]%s",
					i, gpio_mode[(d[0]&0x70)>>4],
					gpio_pull[d[2]&0x7],
					gpio_out[d[5]&0x30>>4],
					gpio_drv[d[5]&0x3]);

		spmi_ext_register_readl(ctrl, PMIC_SPMI_SID_SECONDARY,
					PMIC_GPIO_INT(i), d, 1);

		if (d[0])
			pr_cont(", [INT]Enable");

		pr_cont("\n");
	}

	pr_cont("SECONDARY PMIC MPPs:\n");
	for (i = 1; i < dbgdata->n_pmi_mpp+1; i++) {
		spmi_ext_register_readl(ctrl, PMIC_SPMI_SID_SECONDARY,
					PMIC_MPP_MODE(i), d, 3);

		pr_cont("MPP[%d]: [DIR]%s, [PULL]%s",
					i, mpp_mode[(d[0]&0x70)>>4],
					mpp_pull[d[2]&0x7]);

		spmi_ext_register_readl(ctrl, PMIC_SPMI_SID_SECONDARY,
					PMIC_MPP_INT(i), d, 1);

		if (d[0])
			pr_cont(", [INT]Enable");

		pr_cont("\n");
	}

	spin_unlock_irqrestore(&gpios_lock, flags);

	return;
}

static int status_show(struct seq_file *m, void *unused)
{
	int i;
	unsigned cfg;
	unsigned out;
	unsigned intr;
	unsigned char d[6];
	unsigned long flags;

	spin_lock_irqsave(&gpios_lock, flags);

	seq_puts(m, "MSM GPIOs:\n");
	for (i = 0; i < dbgdata->n_msm_gpio; i++) {
		cfg = __raw_readl(GPIO_CONFIG(dbgdata, i));
		out = __raw_readl(GPIO_IN_OUT(dbgdata, i));
		intr = __raw_readl(GPIO_INTR_STATUS(dbgdata, i));

		seq_printf(m, "GPIO[%d]: [FS]0x%x, [DIR]%s, [PULL]%s, [DRV]%dmA",
					i, (cfg&0x3C)>>2,
					((cfg&0x200)>>9) ? "OUT" : "IN",
					pull[(cfg&0x3)],
					(((cfg&0x1C0)>>6)<<1)+2);

		if ((cfg&0x200)>>9)
			seq_printf(m, ", [VAL]%s",
					((out>>1)&0x1) ? "HIGH" : "LOW");

		if (intr&0x1)
			seq_puts(m, ", [INT]HIGH");

		seq_puts(m, "\n");
	}

	seq_puts(m, "PRIMARY PMIC GPIOs:\n");
	for (i = 1; i < dbgdata->n_pm_gpio+1; i++) {
		spmi_ext_register_readl(ctrl, PMIC_SPMI_SID_PRIMARY,
					PMIC_GPIO_MODE(i), d, 6);

		seq_printf(m, "GPIO[%d]: [DIR]%s, [PULL]%s, [OUT]%s, [DRV]%s",
					i, gpio_mode[(d[0]&0x70)>>4],
					gpio_pull[d[2]&0x7],
					gpio_out[d[5]&0x30>>4],
					gpio_drv[d[5]&0x3]);

		spmi_ext_register_readl(ctrl, PMIC_SPMI_SID_PRIMARY,
					PMIC_GPIO_INT(i), d, 1);

		if (d[0])
			seq_puts(m, ", [INT]Enable");

		seq_puts(m, "\n");
	}

	seq_puts(m, "PRIMARY PMIC MPPs:\n");
	for (i = 1; i < dbgdata->n_pm_mpp+1; i++) {
		spmi_ext_register_readl(ctrl, PMIC_SPMI_SID_PRIMARY,
					PMIC_MPP_MODE(i), d, 3);

		seq_printf(m, "MPP[%d]: [DIR]%s, [PULL]%s",
					i, mpp_mode[(d[0]&0x70)>>4],
					mpp_pull[d[2]&0x7]);

		spmi_ext_register_readl(ctrl, PMIC_SPMI_SID_PRIMARY,
					PMIC_MPP_INT(i), d, 1);

		if (d[0])
			seq_puts(m, ", [INT]Enable");

		seq_puts(m, "\n");
	}

	seq_puts(m, "SECONDARY PMIC GPIOs:\n");
	for (i = 1; i < dbgdata->n_pmi_gpio+1; i++) {
		spmi_ext_register_readl(ctrl, PMIC_SPMI_SID_SECONDARY,
					PMIC_GPIO_MODE(i), d, 6);

		seq_printf(m, "GPIO[%d]: [DIR]%s, [PULL]%s, [OUT]%s, [DRV]%s",
					i, gpio_mode[(d[0]&0x70)>>4],
					gpio_pull[d[2]&0x7],
					gpio_out[d[5]&0x30>>4],
					gpio_drv[d[5]&0x3]);

		spmi_ext_register_readl(ctrl, PMIC_SPMI_SID_SECONDARY,
					PMIC_GPIO_INT(i), d, 1);

		if (d[0])
			seq_puts(m, ", [INT]Enable");

		seq_puts(m, "\n");
	}

	seq_puts(m, "SECONDARY PMIC MPPs:\n");
	for (i = 1; i < dbgdata->n_pmi_mpp+1; i++) {
		spmi_ext_register_readl(ctrl, PMIC_SPMI_SID_SECONDARY,
					PMIC_MPP_MODE(i), d, 3);

		seq_printf(m, "MPP[%d]: [DIR]%s, [PULL]%s",
					i, mpp_mode[(d[0]&0x70)>>4],
					mpp_pull[d[2]&0x7]);

		spmi_ext_register_readl(ctrl, PMIC_SPMI_SID_SECONDARY,
					PMIC_MPP_INT(i), d, 1);

		if (d[0])
			seq_puts(m, ", [INT]Enable");

		seq_puts(m, "\n");
	}

	spin_unlock_irqrestore(&gpios_lock, flags);

	return 0;
}

static int status_open(struct inode *inode, struct file *file)
{
	return single_open(file, status_show, NULL);
}

static const struct file_operations status_fops = {
	.owner = THIS_MODULE,
	.open = status_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int gpio_debug_probe(struct platform_device *pdev)
{
	int ret;
	struct resource *res;
	struct device *dev = &pdev->dev;
	const struct device_node *node = pdev->dev.of_node;

	if (!pdev->dev.of_node)
		return -ENODEV;
	dbgdata = devm_kzalloc(dev, sizeof(struct lge_gpio_debug_data),
							GFP_KERNEL);
	if (!dbgdata)
		return -EIO;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "tlmm-base");
	if (!res)
		goto err;

	dbgdata->base = devm_ioremap(dev, res->start, resource_size(res));
	if (!dbgdata->base)
		goto err;

	ret = of_property_read_u32(node, "lge,n-msm-gpio",
					&dbgdata->n_msm_gpio);
	if (ret)
		dbgdata->n_msm_gpio = 0;

	ret = of_property_read_u32(node, "lge,n-pm-gpio",
					&dbgdata->n_pm_gpio);
	if (ret)
		dbgdata->n_pm_gpio = 0;

	ret = of_property_read_u32(node, "lge,n-pm-mpp",
					&dbgdata->n_pm_mpp);
	if (ret)
		dbgdata->n_pm_mpp = 0;

	ret = of_property_read_u32(node, "lge,n-pmi-gpio",
					&dbgdata->n_pmi_gpio);
	if (ret)
		dbgdata->n_pmi_gpio = 0;

	ret = of_property_read_u32(node, "lge,n-pmi-mpp",
					&dbgdata->n_pmi_mpp);
	if (ret)
		dbgdata->n_pmi_mpp = 0;

	platform_set_drvdata(pdev, dbgdata);

	debugfs_base = debugfs_create_dir("gpios", NULL);

	if (!debugfs_base)
		goto err;

	if (!debugfs_create_u32("debug_suspend", S_IRUGO | S_IWUSR,
				debugfs_base, &debug_suspend)) {
		debugfs_remove_recursive(debugfs_base);
		goto err;
	}

	tlmm_dentry = debugfs_create_file("status", S_IRUGO,
				debugfs_base, NULL, &status_fops);

	if (!tlmm_dentry)
		goto err;

	ctrl = spmi_busnum_to_ctrl(0);

	return 0;
err:
	return -ENODEV;
}

static int gpio_debug_remove(struct platform_device *pdev)
{
	return 0;
}

static struct of_device_id gpio_debug_match[] = {
	{ .compatible = "lge,gpio-debug"},
	{}
};

static struct platform_driver gpio_debug_driver = {
	.probe          = gpio_debug_probe,
	.remove         = gpio_debug_remove,
	.driver         = {
		.name   = "lge-gpio-debug",
		.owner  = THIS_MODULE,
		.of_match_table = gpio_debug_match,
	},
};

static int __init gpio_debug_init(void)
{
	return platform_driver_register(&gpio_debug_driver);
}
module_init(gpio_debug_init);

static void __exit gpio_debug_exit(void)
{
	platform_driver_unregister(&gpio_debug_driver);
}
module_exit(gpio_debug_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("GPIOs dynamic debugger driver");
