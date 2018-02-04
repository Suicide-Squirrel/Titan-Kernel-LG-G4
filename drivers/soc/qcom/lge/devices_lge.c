#define DEBUG // yjcho

#include <linux/kernel.h>
#include <linux/string.h>

#include <soc/qcom/lge/board_lge.h>
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/platform_device.h>

#ifdef CONFIG_LGE_PM_USB_ID
#include <linux/err.h>
#include <linux/qpnp/qpnp-adc.h>
#include <linux/power_supply.h>
#endif

#ifdef CONFIG_LGE_USB_G_ANDROID
#include <linux/platform_data/lge_android_usb.h>
#endif

#ifdef CONFIG_LGE_EARJACK_DEBUGGER
#include <soc/qcom/lge/board_lge.h>
#endif
#ifdef CONFIG_LGE_PM_USB_ID
struct chg_cable_info_table {
	int threshhold;
	enum acc_cable_type type;
	unsigned ta_ma;
	unsigned usb_ma;
};

#define ADC_NO_INIT_CABLE   0
#define C_NO_INIT_TA_MA     0
#define C_NO_INIT_USB_MA    0
#define ADC_CABLE_NONE      1900000
#define C_NONE_TA_MA        700
#define C_NONE_USB_MA       500

#define MAX_CABLE_NUM		15
static bool cable_type_defined;
static struct chg_cable_info_table lge_acc_cable_type_data[MAX_CABLE_NUM];
#endif

#if defined(CONFIG_LGE_MIPI_P1_INCELL_QHD_CMD_PANEL)
static char dsv_vendor[3];
int display_panel_type;
int lk_panel_init_fail = 0;
int rsp_nvm_rw;
#if defined(CONFIG_MACH_MSM8992_P1_CN) || defined(CONFIG_MACH_MSM8992_P1_GLOBAL_COM) || defined(CONFIG_MACH_MSM8992_SJR_GLOBAL_COM)
int lge_sim_type;
#endif
#endif

#if IS_ENABLED(CONFIG_LGE_MIPI_PP_INCELL_QHD_CMD_PANEL)
int display_dic_rev;
#endif
static enum hw_rev_type lge_bd_rev = HW_REV_MAX;

/* CAUTION: These strings are come from LK. */
char *rev_str[] = {"evb1", "evb2", "evb3", "rev_0", "rev_a", "rev_b", "rev_c",
	"rev_d", "rev_e", "rev_f", "rev_g", "rev_10", "rev_11", "rev_12",
	"reserved"};
extern unsigned int system_rev;

static int __init board_revno_setup(char *rev_info)
{
	int i;

	for (i = 0; i < HW_REV_MAX; i++) {
		if (!strncmp(rev_info, rev_str[i], 6)) {
			lge_bd_rev = i;
			system_rev = lge_bd_rev;
			break;
		}
	}

	pr_info("BOARD : LGE %s\n", rev_str[lge_bd_rev]);
	return 1;
}
__setup("lge.rev=", board_revno_setup);

enum hw_rev_type lge_get_board_revno(void)
{
	return lge_bd_rev;
}

#ifdef CONFIG_LGE_PM_USB_ID
void get_cable_data_from_dt(void *of_node)
{
	int i;
	u32 cable_value[3];
	struct device_node *node_temp = (struct device_node *)of_node;
	const char *propname[MAX_CABLE_NUM] = {
		"lge,no-init-cable",
		"lge,cable-mhl-1k",
		"lge,cable-u-28p7k",
		"lge,cable-28p7k",
		"lge,cable-56k",
		"lge,cable-100k",
		"lge,cable-130k",
		"lge,cable-180k",
		"lge,cable-200k",
		"lge,cable-220k",
		"lge,cable-270k",
		"lge,cable-330k",
		"lge,cable-620k",
		"lge,cable-910k",
		"lge,cable-none"
	};
	if (cable_type_defined) {
		pr_info("Cable type is already defined\n");
		return;
	}

	for (i = 0; i < MAX_CABLE_NUM; i++) {
		of_property_read_u32_array(node_temp, propname[i],
				cable_value, 3);
		lge_acc_cable_type_data[i].threshhold = cable_value[0];
		lge_acc_cable_type_data[i].type = i;
		lge_acc_cable_type_data[i].ta_ma = cable_value[1];
		lge_acc_cable_type_data[i].usb_ma = cable_value[2];
	}
	cable_type_defined = 1;
}

int lge_pm_get_cable_info(struct qpnp_vadc_chip *vadc,
		struct chg_cable_info *cable_info)
{
	char *type_str[] = {
		"NOT INIT", "MHL 1K", "U_28P7K", "28P7K", "56K",
		"100K", "130K", "180K", "200K", "220K",
		"270K", "330K", "620K", "910K", "OPEN"
	};

	struct qpnp_vadc_result result;
	struct chg_cable_info *info = cable_info;
	struct chg_cable_info_table *table;
	int table_size = ARRAY_SIZE(lge_acc_cable_type_data);
	int acc_read_value = 0;
	int i, rc;
	int count = 1;

	if (!info) {
		pr_err("%s : invalid info parameters\n", __func__);
		return -EINVAL;
	}

	if (!vadc) {
		pr_err("%s : invalid vadc parameters\n", __func__);
		return -EINVAL;
	}

	if (!cable_type_defined) {
		pr_err("%s : cable type is not defined yet.\n", __func__);
		return -EINVAL;
	}

	for (i = 0; i < count; i++) {
		rc = qpnp_vadc_read(vadc, LR_MUX10_USB_ID_LV, &result);

		if (rc < 0) {
			if (rc == -ETIMEDOUT) {
				/* reason: adc read timeout,
				 * assume it is open cable
				 */
				info->cable_type = CABLE_NONE;
				info->ta_ma = C_NONE_TA_MA;
				info->usb_ma = C_NONE_USB_MA;
			}
			pr_err("%s : adc read error - %d\n", __func__, rc);
			return rc;
		}

		acc_read_value = (int)result.physical;
		pr_info("%s : adc_read-%d\n", __func__, (int)result.physical);
		/* mdelay(10); */
	}

	info->cable_type = NO_INIT_CABLE;
	info->ta_ma = C_NO_INIT_TA_MA;
	info->usb_ma = C_NO_INIT_USB_MA;

	/* assume: adc value must be existed in ascending order */
	for (i = 0; i < table_size; i++) {
		table = &lge_acc_cable_type_data[i];

		if (acc_read_value <= table->threshhold) {
			info->cable_type = table->type;
			info->ta_ma = table->ta_ma;
			info->usb_ma = table->usb_ma;
			break;
		}
	}

	pr_err("\n\n[PM]Cable detected: %d(%s)(%d, %d)\n\n",
			acc_read_value, type_str[info->cable_type],
			info->ta_ma, info->usb_ma);

	return 0;
}

/* Belows are for using in interrupt context */
static struct chg_cable_info lge_cable_info;

enum acc_cable_type lge_pm_get_cable_type(void)
{
	return lge_cable_info.cable_type;
}

unsigned lge_pm_get_ta_current(void)
{
	return lge_cable_info.ta_ma;
}

unsigned lge_pm_get_usb_current(void)
{
	return lge_cable_info.usb_ma;
}

/* This must be invoked in process context */
void lge_pm_read_cable_info(struct qpnp_vadc_chip *vadc)
{
	lge_cable_info.cable_type = NO_INIT_CABLE;
	lge_cable_info.ta_ma = C_NO_INIT_TA_MA;
	lge_cable_info.usb_ma = C_NO_INIT_USB_MA;

	lge_pm_get_cable_info(vadc, &lge_cable_info);
}

void lge_pm_read_cable_info_and_type(struct device *dev,
				struct qpnp_vadc_chip *vadc)
{
	struct device_node *node = NULL;
	const char *cable_type = "lge,cable-type";

	if (!cable_type_defined) {
		if (dev && dev->of_node) {
		    node = of_parse_phandle(dev->of_node, cable_type, 0);
			if (node)
				get_cable_data_from_dt(node);
		}
	}

	lge_pm_read_cable_info(vadc);
}
#endif

#ifdef CONFIG_LGE_EARJACK_DEBUGGER
/* s_uart_console_status bits format
 * ------higher than bit4 are not used
 * bit5...: not used
 * ------bit4 indicates whenter uart console was ready(probed)
 * bit4: [UART_CONSOLE_READY]
 * ------current uart console status -----------------
 * bit3: [UART_CONSOLE_ENABLED]
 * ------configuration bit field -----------------
 * bit2: [UART_CONSOLE_ENABLE_ON_DEFAULT]
 * bit1; [UART_CONSOLE_ENABLE_ON_EARJACK_DEBUGGER]
 * bit0: [UART_CONSOLE_ENABLE_ON_EARJACK]
 */
static unsigned int s_uart_console_status = 0;	/* disabling uart console */

unsigned int lge_uart_console_get_config(void)
{
	return (s_uart_console_status & UART_CONSOLE_MASK_CONFIG);
}

void lge_uart_console_set_config(unsigned int config)
{
	config &= UART_CONSOLE_MASK_CONFIG;
	s_uart_console_status |= config;
}

unsigned int lge_uart_console_get_enabled(void)
{
	return s_uart_console_status & UART_CONSOLE_MASK_ENABLED;
}

void lge_uart_console_set_enabled(int enabled)
{
	s_uart_console_status &= ~UART_CONSOLE_MASK_ENABLED;
	/* for caller conding convenience, regard no-zero as enabled also */
	s_uart_console_status |= (enabled ? UART_CONSOLE_ENABLED : 0);
}

unsigned int lge_uart_console_get_ready(void)
{
	return s_uart_console_status & UART_CONSOLE_MASK_READY;
}

void lge_uart_console_set_ready(unsigned int ready)
{
	s_uart_console_status &= ~UART_CONSOLE_MASK_READY;
	/* for caller side coding convenience, regard no-zero as ready also */
	s_uart_console_status |= (ready ? UART_CONSOLE_READY : 0);
}

#endif /* CONFIG_LGE_EARJACK_DEBUGGER */

#if defined(CONFIG_LGE_MIPI_P1_INCELL_QHD_CMD_PANEL)
static int __init display_dsv_setup(char *dsv_cmd)
{
	sscanf(dsv_cmd, "%s", dsv_vendor);
	pr_info("dsv vendor id is %s\n", dsv_vendor);

	return 1;
}
__setup("lge.dsv_id=", display_dsv_setup);

char* lge_get_dsv_vendor(void)
{
     return dsv_vendor;
}

void lge_set_panel(int panel_type)
{
	pr_info("panel_type is %d\n",panel_type);

	display_panel_type = panel_type;

}

int lge_get_panel(void)
{
	return display_panel_type;
}

static int __init lge_rsp_nvm_setup(char *rsp_nvm)
{
	if (strncmp(rsp_nvm, "0", 1) == 0) {
		rsp_nvm_rw = 0;
	} else if (strncmp(rsp_nvm, "1", 1) == 0) {
		rsp_nvm_rw = 1;
	} else {
		pr_err("%s : fail to read rsp_nvm \n", __func__);
	}
	pr_debug("rsp_nvm %d,\n", rsp_nvm_rw);

	return 1;
}
__setup("lge.rsp_nvm=", lge_rsp_nvm_setup);

int lge_get_rsp_nvm(void)
{
	return rsp_nvm_rw;
}

#if defined(CONFIG_MACH_MSM8992_P1_CN) || defined(CONFIG_MACH_MSM8992_P1_GLOBAL_COM) || defined(CONFIG_MACH_MSM8992_SJR_GLOBAL_COM)
static int __init lge_sim_setup(char *sim_num)
{
	if (strncmp(sim_num, "1", 1) == 0) {
		lge_sim_type = 1;
	} else if (strncmp(sim_num, "2", 1) == 0) {
		lge_sim_type = 2;
	} else {
		lge_sim_type = 0;
		pr_err("%s : fail to read sim type\n", __func__);
	}
	pr_debug("lge_sim_type is %d, sim_num set %s= \n", lge_sim_type,sim_num);

	return 1;
}
__setup("lge.sim_num=", lge_sim_setup);

int lge_get_sim_type(void)
{
	return lge_sim_type;
}
#endif

static int __init lk_panel_init_status(char *panel_init_cmd)
{
	if (strncmp(panel_init_cmd, "1", 1) == 0) {
		lk_panel_init_fail = 1;
		pr_info("lk panel init fail[%d]\n", lk_panel_init_fail);
	} else {
		lk_panel_init_fail = 0;
	}

	return 1;
}
__setup("lge.pinit_fail=", lk_panel_init_status);


int lge_get_lk_panel_status(void)
{
     return lk_panel_init_fail;
}
#endif

#if IS_ENABLED(CONFIG_LGE_MIPI_PP_INCELL_QHD_CMD_PANEL)
static int __init lge_lgd_sic4945_rev_setup(char *dic_cmd)
{
        sscanf(dic_cmd, "%d", &display_dic_rev);
            pr_info("lge_lgd_sic4945_rev is %d\n", display_dic_rev);

                return 1;
}
__setup("lge.dic_rev=", lge_lgd_sic4945_rev_setup);

int lge_get_lgd_sic4945_rev(void)
{
        return display_dic_rev;
}
#endif

/*
   for download complete using LAF image
   return value : 1 --> right after laf complete & reset
 */

int android_dlcomplete = 0;

int __init lge_android_dlcomplete(char *s)
{
	if (strncmp(s, "1", 1) == 0)
		android_dlcomplete = 1;
	else
		android_dlcomplete = 0;
	pr_info("androidboot.dlcomplete = %d\n", android_dlcomplete);

	return 1;
}
__setup("androidboot.dlcomplete=", lge_android_dlcomplete);

int lge_get_android_dlcomplete(void)
{
	return android_dlcomplete;
}

#ifdef CONFIG_LGE_PM_FACTORY_PSEUDO_BATTERY
struct pseudo_batt_info_type pseudo_batt_info = {
	.mode = 0,
	.id = 1,
	.therm = 100,
	.temp = 400,
	.volt = 4100,
	.capacity = 80,
	.charging = 1,
};

int safety_timer = 1;

void pseudo_batt_set(struct pseudo_batt_info_type *info)
{
	struct power_supply *batt_psy, *usb_psy;
	union power_supply_propval ret = {0,};

	batt_psy = power_supply_get_by_name("battery");

	if (!batt_psy) {
		pr_err("called before init\n");
		return;
	}

	usb_psy = power_supply_get_by_name("usb");
	if (!usb_psy) {
		pr_err("called before init\n");
		return;
	}

	pseudo_batt_info.mode = info->mode;
	pseudo_batt_info.id = info->id;
	pseudo_batt_info.therm = info->therm;
	pseudo_batt_info.temp = info->temp;
	pseudo_batt_info.volt = info->volt;
	pseudo_batt_info.capacity = info->capacity;
	pseudo_batt_info.charging = info->charging;

	pr_err("pseudo batt set success\n");
	ret.intval = !pseudo_batt_info.mode;
	batt_psy->set_property(batt_psy, POWER_SUPPLY_PROP_SAFETY_TIMER_ENABLE,
			&ret);
	power_supply_changed(batt_psy);
	power_supply_changed(usb_psy);
}
#endif

/* get boot mode information from cmdline.
 * If any boot mode is not specified,
 * boot mode is normal type.
 */
static enum lge_boot_mode_type lge_boot_mode = LGE_BOOT_MODE_NORMAL;
int __init lge_boot_mode_init(char *s)
{
	if (!strcmp(s, "charger"))
		lge_boot_mode = LGE_BOOT_MODE_CHARGERLOGO;
	else if (!strcmp(s, "chargerlogo"))
		lge_boot_mode = LGE_BOOT_MODE_CHARGERLOGO;
	else if (!strcmp(s, "qem_56k"))
		lge_boot_mode = LGE_BOOT_MODE_QEM_56K;
	else if (!strcmp(s, "qem_130k"))
		lge_boot_mode = LGE_BOOT_MODE_QEM_130K;
	else if (!strcmp(s, "qem_910k"))
		lge_boot_mode = LGE_BOOT_MODE_QEM_910K;
	else if (!strcmp(s, "pif_56k"))
		lge_boot_mode = LGE_BOOT_MODE_PIF_56K;
	else if (!strcmp(s, "pif_130k"))
		lge_boot_mode = LGE_BOOT_MODE_PIF_130K;
	else if (!strcmp(s, "pif_910k"))
		lge_boot_mode = LGE_BOOT_MODE_PIF_910K;
	/* LGE_UPDATE_S for MINIOS2.0 */
	else if (!strcmp(s, "miniOS"))
		lge_boot_mode = LGE_BOOT_MODE_MINIOS;
	pr_info("ANDROID BOOT MODE : %d %s\n", lge_boot_mode, s);
	/* LGE_UPDATE_E for MINIOS2.0 */

	return 1;
}
__setup("androidboot.mode=", lge_boot_mode_init);

enum lge_boot_mode_type lge_get_boot_mode(void)
{
	return lge_boot_mode;
}

int lge_get_factory_boot(void)
{
	int res;

	/*   if boot mode is factory,
	 *   cable must be factory cable.
	 */
	switch (lge_boot_mode) {
	case LGE_BOOT_MODE_QEM_56K:
	case LGE_BOOT_MODE_QEM_130K:
	case LGE_BOOT_MODE_QEM_910K:
	case LGE_BOOT_MODE_PIF_56K:
	case LGE_BOOT_MODE_PIF_130K:
	case LGE_BOOT_MODE_PIF_910K:
	case LGE_BOOT_MODE_MINIOS:
		res = 1;
		break;
	default:
		res = 0;
		break;
	}
	return res;
}

static enum lge_laf_mode_type lge_laf_mode = LGE_LAF_MODE_NORMAL;

int __init lge_laf_mode_init(char *s)
{
	if (strcmp(s, "") && strcmp(s, "MID"))
		lge_laf_mode = LGE_LAF_MODE_LAF;

	return 1;
}
__setup("androidboot.laf=", lge_laf_mode_init);

enum lge_laf_mode_type lge_get_laf_mode(void)
{
	return lge_laf_mode;
}

#ifdef CONFIG_LGE_USB_G_ANDROID
int get_factory_cable(void)
{
	int res = 0;

	/* if boot mode is factory, cable must be factory cable. */
	switch (lge_boot_mode) {
	case LGE_BOOT_MODE_QEM_56K:
	case LGE_BOOT_MODE_PIF_56K:
		res = LGEUSB_FACTORY_56K;
		break;

	case LGE_BOOT_MODE_QEM_130K:
	case LGE_BOOT_MODE_PIF_130K:
		res = LGEUSB_FACTORY_130K;
		break;

	case LGE_BOOT_MODE_QEM_910K:
	case LGE_BOOT_MODE_PIF_910K:
		res = LGEUSB_FACTORY_910K;
		break;

	default:
		res = 0;
		break;
	}

	return res;
}

struct lge_android_usb_platform_data lge_android_usb_pdata = {
	.vendor_id = 0x1004,
	.factory_pid = 0x6000,
	.iSerialNumber = 0,
	.product_name = "LGE Android Phone",
	.manufacturer_name = "LG Electronics Inc.",
	.factory_composition = "acm,diag",
	.get_factory_cable = get_factory_cable,
};

static struct platform_device lge_android_usb_device = {
	.name = "lge_android_usb",
	.id = -1,
	.dev = {
		.platform_data = &lge_android_usb_pdata,
	},
};

static int __init lge_android_usb_devices_init(void)
{
	return platform_device_register(&lge_android_usb_device);
}
arch_initcall(lge_android_usb_devices_init);
#endif

#ifdef CONFIG_LGE_USB_DIAG_LOCK
static struct platform_device lg_diag_cmd_device = {
	.name = "lg_diag_cmd",
	.id = -1,
	.dev    = {
		.platform_data = 0, /* &lg_diag_cmd_pdata */
	},
};

static int __init lge_diag_devices_init(void)
{
	return platform_device_register(&lg_diag_cmd_device);
}
arch_initcall(lge_diag_devices_init);
#endif

#ifdef CONFIG_LGE_QFPROM_INTERFACE
static struct platform_device qfprom_device = {
	.name = "lge-qfprom",
	.id = -1,
};

static int __init lge_add_qfprom_devices(void)
{
	return platform_device_register(&qfprom_device);
}

arch_initcall(lge_add_qfprom_devices);
#endif

static int lge_boot_reason = -1; /* undefined for error checking */
static int __init lge_check_bootreason(char *reason)
{
	int ret = 0;

	/* handle corner case of kstrtoint */
	if (!strcmp(reason, "0xffffffff")) {
		lge_boot_reason = 0xffffffff;
		return 1;
	}

	ret = kstrtoint(reason, 16, &lge_boot_reason);
	if (!ret)
		printk(KERN_INFO "LGE REBOOT REASON: %x\n", lge_boot_reason);
	else
		printk(KERN_INFO "LGE REBOOT REASON: Couldn't get bootreason - %d\n",
				ret);

	return 1;
}
__setup("lge.bootreasoncode=", lge_check_bootreason);

int lge_get_bootreason(void)
{
	return lge_boot_reason;
}

int on_hidden_reset;

static int __init lge_check_hidden_reset(char *reset_mode)
{
	if (!strncmp(reset_mode, "on", 2))
		on_hidden_reset = 1;

	return 1;
}
__setup("lge.hreset=", lge_check_hidden_reset);

#ifdef CONFIG_LGE_LCD_OFF_DIMMING
int lge_get_bootreason_with_lcd_dimming(void)
{
	int ret = 0;

	if (lge_get_bootreason() == 0x77665560)
		ret = 1;
	else if (lge_get_bootreason() == 0x77665561)
		ret = 2;
	else if (lge_get_bootreason() == 0x77665562)
		ret = 3;

	return ret;
}
#endif
