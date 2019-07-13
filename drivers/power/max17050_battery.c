/*
 * Fuel gauge driver for Maxim 17050 / 8966 / 8997
 *  Note that Maxim 8966 and 8997 are mfd and this is its subdevice.
 *
 * Copyright (C) 2012 LG Electronics
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * This driver is based on max17040_battery.c
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mod_devicetable.h>
#include <linux/power/max17050_battery.h>
#include <linux/delay.h>

/*#include <mach/board_lge.h>*/
#include <linux/module.h>
#include <linux/moduleparam.h>
/*#include "../../lge/include/lg_backup_items.h"*/

#ifdef CONFIG_LGE_PM
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/power_supply.h>
#include <linux/wakelock.h>
#include <soc/qcom/smem.h>
#include <soc/qcom/lge/board_lge.h>
#include <linux/moduleparam.h>
#include "charger-controller.h"
#endif

#ifdef CONFIG_LGE_PM_BATTERY_ID_CHECKER
#include <linux/power/lge_battery_id.h>
#endif

#ifdef CONFIG_LGE_PM_MAX17050_POLLING
#define MAX17050_POLLING_PERIOD_20 20000
#define MAX17050_POLLING_PERIOD_10 10000
#define MAX17050_POLLING_PERIOD_5 5000
#endif

/* Factory cable type */
#define LT_CABLE_56K		6
#define LT_CABLE_130K		7
#define LT_CABLE_910K		11

/* Status register bits */
#define STATUS_POR_BIT      (1 << 1)
#define STATUS_BI_BIT       (1 << 11)
#define STATUS_BR_BIT       (1 << 15)

/* Interrupt config/status bits */
#define CFG_ALRT_BIT_ENBL	(1 << 2)
#define CFG_EXT_TEMP_BIT	(1 << 8)
#define STATUS_INTR_SOCMIN_BIT	(1 << 10)
#define STATUS_INTR_SOCMAX_BIT	(1 << 14)

#define FAKE_OCV_LGC 3180
#define FAKE_OCV_TCD 3280

extern void write_shutdown_soc(char *filename, int write_val, int pos);
extern int read_shutdown_soc(char *filename, int pos);
void max17050_battery_compare_soc(int current_soc, int before_soc,
		int before_vfocv, int before_cell);
bool max17050_quick_start(void);
static int compare_flag;
static int por_state;
static int recal_volt;

enum print_reason {
	PR_DEBUG		= BIT(0),
	PR_INFO		= BIT(1),
	PR_ERR		= BIT(2),
};

static int fake_batt_flag = 0;
static int fg_debug_mask = PR_INFO|PR_ERR;
module_param_named(
	debug_mask, fg_debug_mask, int, S_IRUSR | S_IWUSR
);

#define pr_max17050(reason, fmt, ...)                                \
	do {                                                             \
	if (fg_debug_mask & (reason))                                    \
		pr_info("[MAX17050] " fmt, ##__VA_ARGS__);                         \
	else                                                             \
		pr_debug("[MAX17050] " fmt, ##__VA_ARGS__);                        \
	} while (0)

static struct i2c_client *max17050_i2c_client;

u16 pre_soc = 100;
u16 real_soc = 100;

struct max17050_chip {
	struct i2c_client *client;
	/*struct power_supply battery;*/
	struct power_supply		*batt_psy;
	struct power_supply		*ac_psy;
	struct power_supply		battery;
	struct power_supply		*dc_psy;
	struct max17050_platform_data *pdata;
	struct work_struct work;
	struct mutex mutex;
	struct delayed_work	max17050_model_data_write_work;
	struct delayed_work	max17050_monitor_work;
	struct delayed_work	max17050_dump_work;
#ifdef CONFIG_LGE_PM_MAX17050_RECHARGING
	struct wake_lock 	recharging_lock;
#endif
	bool suspended;
	bool init_done;
	bool use_ext_temp;

	int soc_rep;
	int soc_vf;
	int prev_soc;
	int last_soc;
	int soc_calc_cut_off_flag;
	int fake_soc;

	int soc_rep_raw;
	int soc_vf_raw;
	int before_soc_rep_raw;
	int before_soc_vf_raw;
	int avg_ibatt;

};

static struct max17050_chip *ref;

/* 130411 junnyoung.jang@lge.com Implement Power test SOC quickstart */
static unsigned int cable_smem_size;
int lge_power_test_flag_max17050 = 1;
int lge_power_init_flag_max17050 = 0;
/* 130411 junnyoung.jang@lge.com Implement Power test SOC quickstart */

/*static int max17050_access_control_of_flash(void);
bool max17050_count_control(u8 select, u8 count_up);
int max17050_save_bat_info_check(void);*/
#ifdef CONFIG_LGE_PM_BATTERY_ID_CHECKER
/* using to cal rcomp */
int cell_info = 0;
int is_factory_cable_56K = 0;
int is_factory_cable_130K = 0;
/* Default Rescale soc & factor */
int rescale_soc = 9400;
#endif
static int max17050_write_reg(struct i2c_client *client, u8 reg, u16 value)
{
	int ret = i2c_smbus_write_word_data(client, reg, value);

	if (ret < 0)
		dev_err(&client->dev, "err %d\n", ret);

	return ret;
}

static int max17050_read_reg(struct i2c_client *client, u8 reg)
{
	int ret = i2c_smbus_read_word_data(client, reg);

	if (ret < 0)
		dev_err(&client->dev, "err %d\n", ret);

	return ret;
}

static int max17050_multi_write_data(struct i2c_client *client,
			int reg, const u8 *values, int length)
{
	int ret;

	ret = i2c_smbus_write_i2c_block_data(client, reg, length, values);

	if (ret < 0)
		dev_err(&client->dev, "err %d\n", ret);

	return ret;
}

static int max17050_multi_read_data(struct i2c_client *client,
			int reg, u8 *values, int length)
{
	int ret;

	ret = i2c_smbus_read_i2c_block_data(client, reg, length, values);

	if (ret < 0)
		dev_err(&client->dev, "err %d\n", ret);

	return ret;
}


int max17050_get_mvolts(void)
{
	u16 read_reg;
	int vbatt_mv;

	/*if (max17050_nobattery)
		return 3950;*/
	if (max17050_i2c_client == NULL) {
		pr_max17050(PR_ERR, "%s : i2c NULL vbatt = 800 mV\n", __func__);
		return 800;
	}
	read_reg = max17050_read_reg(max17050_i2c_client, MAX17050_V_CELL);
	if (read_reg < 0)
		return 800;

	vbatt_mv = (read_reg >> 3);
	vbatt_mv = (vbatt_mv * 625) / 1000;

	pr_max17050(PR_DEBUG, "%s : vbatt = %d mV\n", __func__, vbatt_mv);

	return vbatt_mv;
}

int max17050_get_ocv_mvolts(void)
{
	u16 read_reg;
	int ocv_mv;

	/*if (max17050_nobattery)
		return 3950;*/
	if (max17050_i2c_client == NULL) {
		pr_max17050(PR_ERR, "%s : i2c NULL vbatt = 800 mV\n", __func__);
		return 800;
	}
	read_reg = max17050_read_reg(max17050_i2c_client, MAX17050_V_FOCV);
	if (read_reg < 0)
		return 800;

	ocv_mv = (read_reg >> 4);
	ocv_mv = (ocv_mv * 125) / 100;

	pr_max17050(PR_DEBUG,"ocv = %d mV\n", ocv_mv);

	return ocv_mv;

}

int max17050_suspend_get_mvolts(void)
{
	u16 read_reg;
	int vbatt_mv;

	/*if (max17050_nobattery)
		return 3950;*/
	if (max17050_i2c_client == NULL) {
		pr_max17050(PR_ERR, "%s : i2c NULL vbatt = 3950 mV\n", __func__);
		return 800;
	}

	read_reg = max17050_read_reg(max17050_i2c_client, MAX17050_V_CELL);
	if (read_reg < 0)
		return 800;

	vbatt_mv = (read_reg >> 3);
	vbatt_mv = (vbatt_mv * 625) / 1000;

	pr_max17050(PR_DEBUG, "%s : vbatt = %d mV\n", __func__, vbatt_mv);

	return vbatt_mv;
}

int max17050_get_capacity_percent(void)
{
	int battery_soc_rep = 0;
	int battery_soc_vf = 0;
	int read_reg_rep = 0;
	int read_reg_vf = 0;

	u8 upper_reg_rep;
	u8 lower_reg_rep;
	u8 upper_reg_vf;
	u8 lower_reg_vf;

	if (max17050_i2c_client == NULL) {
		return 80;
	} else {
		/* change voltage base(SOC_VF) to current base(SOC_REP) */
		read_reg_rep = max17050_read_reg(max17050_i2c_client,
			MAX17050_SOC_REP);

		read_reg_vf = max17050_read_reg(max17050_i2c_client,
			MAX17050_SOC_VF);

		if (read_reg_rep < 0) {
			pr_max17050(PR_ERR, "%s : i2c Read Fail battery SOC = %d\n",
					__func__, pre_soc);
			return pre_soc;
		}
		if (read_reg_vf < 0) {
			pr_max17050(PR_ERR, "%s : i2c Read Fail battery SOC = %d\n",
					__func__, pre_soc);
			return pre_soc;
		}

		upper_reg_rep = (read_reg_rep & 0xFF00) >> 8 ;
		lower_reg_rep = (read_reg_rep & 0xFF);

		pr_max17050(PR_DEBUG, "%s : SOC_REP : read_reg_rep = %X"
				"upper_reg_rep = %X lower_reg_rep = %X\n",
				__func__, read_reg_rep, upper_reg_rep, lower_reg_rep);

		upper_reg_vf = (read_reg_vf & 0xFF00) >> 8;
		lower_reg_vf = (read_reg_vf & 0xFF);

		pr_max17050(PR_DEBUG, "%s : SOC_VF : read_reg_vf = %X"
				"upper_reg_vf = %X lower_reg_vf = %X\n",
				__func__, read_reg_vf, upper_reg_vf, lower_reg_vf);

		pr_max17050(PR_DEBUG, "%s : SOC_REP : read_reg_rep = %X"
				": SOC_VF : read_reg_vf = %X \n",
				__func__, read_reg_rep, read_reg_vf);

		/* SOC scaling for stable max SOC and changed Cut-off */
		/* Adj SOC = (FG SOC - Emply) / (Full - Empty) * 100 */
		/* cut off vol 3.3V : (soc - 1.132%) * 100 / (94.28% - 1.132%) */
		/* full capacity SOC 106.5% , real battery SOC 100.7% */
		battery_soc_rep = ((upper_reg_rep * 256) + lower_reg_rep) * 10 / 256;
		battery_soc_vf = ((upper_reg_vf * 256) + lower_reg_vf) * 10 / 256;

		ref->before_soc_rep_raw = battery_soc_rep;
		ref->before_soc_vf_raw = battery_soc_vf;

		pr_max17050(PR_DEBUG, "%s : Before_rescailing raw"
				"battery_soc_rep = %d : battery_soc_vf = %d\n",
				__func__, battery_soc_rep, battery_soc_vf);

		battery_soc_rep = (battery_soc_rep * 100) * 100;
		battery_soc_rep = (battery_soc_rep /
				(ref->pdata->rescale_soc)) - (ref->pdata->rescale_factor);
		/* 100 -> 105.8% scailing */

		if (cell_info == LGC_LLL) { /*LGC Battery*/
			if (8 <= battery_soc_rep && battery_soc_rep <= 20) {
				battery_soc_rep = 20;
				pr_max17050(PR_DEBUG, "%s : cut off for LGC 2 per\n", __func__);
			}
			if (5 <= battery_soc_rep && battery_soc_rep < 8) {
				battery_soc_rep = 10;
				pr_max17050(PR_DEBUG, "%s : cut off for LGC 1 per\n", __func__);
			}
		} else { /*Tocad battery*/
			if (3 <= battery_soc_rep && battery_soc_rep <= 20) {
				battery_soc_rep = 20;
				pr_max17050(PR_DEBUG, "%s : cut off for tocad 2 per\n", __func__);
			}
			if (1 <= battery_soc_rep && battery_soc_rep < 3) {
				battery_soc_rep = 10;
				pr_max17050(PR_DEBUG, "%s : cut off for tocad 1 per\n", __func__);
			}
		}
		battery_soc_vf = (battery_soc_vf * 100) * 100;
		battery_soc_vf = (battery_soc_vf /
				(ref->pdata->rescale_soc)) - (ref->pdata->rescale_factor);
		/* 106.8% scailing */

		ref->soc_rep_raw = battery_soc_rep;
		ref->soc_vf_raw = battery_soc_vf;

		pr_max17050(PR_DEBUG, "%s rescale_soc %d,"
				"rescale_factor %d\n",__func__,
				ref->pdata->rescale_soc,ref->pdata->rescale_factor);

		pr_max17050(PR_DEBUG, "%s : After_rescailing raw battery_soc_rep = %d "
			"(upper_reg_rep = %d lower_reg_rep = %d)\n",
			__func__, battery_soc_rep, upper_reg_rep, lower_reg_rep);

		pr_max17050(PR_DEBUG, "%s : After_rescailing raw battery_soc_vf = %d "
			"(upper_reg_vf = %d lower_reg_vf = %d)\n",
			__func__, battery_soc_vf, upper_reg_vf, lower_reg_vf);

		battery_soc_rep /= 10;
		battery_soc_vf /= 10;

		ref->soc_rep = battery_soc_rep;
		ref->soc_vf = battery_soc_vf;

		pr_max17050(PR_DEBUG, "%s : After_rescailing SOC_REP  = %d : SOC_VF  = %d\n",
			__func__, battery_soc_rep, battery_soc_vf);

#ifdef CONFIG_LGE_PM_MAX17050_SOC_REP
		real_soc = battery_soc_rep;

		if (battery_soc_rep >= 100)
			battery_soc_rep = 100;

		if (battery_soc_rep < 0)
			battery_soc_rep = 0;

	}
	pre_soc = battery_soc_rep;

	return battery_soc_rep;
#endif
#ifdef CONFIG_LGE_PM_MAX17050_SOC_VF
		real_soc = battery_soc_vf;

		if (battery_soc_vf >= 100)
			battery_soc_vf = 100;

		if (battery_soc_vf < 0)
			battery_soc_vf = 0;
	}
	pre_soc = battery_soc_vf;

	return battery_soc_vf;
#endif
}

int max17050_get_current(void)
{
	u16 read_reg;
	int ibatt_ma;
	int avg_ibatt_ma;
	u16 sign_bit;

	if (max17050_i2c_client == NULL) {
		pr_max17050(PR_ERR, "%s : i2c NULL", __func__);
		return 999;
	}
	read_reg = max17050_read_reg(max17050_i2c_client, MAX17050_CURRENT);
	if (read_reg < 0)
		return 999; /*Error Value return.*/

	sign_bit = (read_reg & 0x8000)>>15;

	if (sign_bit == 1)
		ibatt_ma = (15625 * (read_reg  - 65536))/100000;
	else
		ibatt_ma = (15625 * read_reg) / 100000;

	/* reverse (charging is negative by convention) */
	ibatt_ma *= -1;

	read_reg = max17050_read_reg(max17050_i2c_client,
		MAX17050_AVERAGE_CURRENT);
	if (read_reg < 0)
		return 999;/*Error Value return.*/

	sign_bit = (read_reg & 0x8000)>>15;

	if (sign_bit == 1)
		avg_ibatt_ma = (15625 * (read_reg  - 65536)) / 100000;
	else
		avg_ibatt_ma = (15625 * read_reg) / 100000;

	/* reverse (charging is negative by convention) */
	avg_ibatt_ma *= -1;

	ref->avg_ibatt = avg_ibatt_ma;

	pr_max17050(PR_DEBUG, "%s : I_batt = %d mA avg_I_batt = %d mA\n",
		__func__, ibatt_ma, avg_ibatt_ma);

	return ibatt_ma;
}

#define DEFAULT_TEMP	25
int max17050_write_temp(void)
{
	int battery_temp;
	int batt_temp_raw;
	u16 read_reg;
	u16 write_temp;

	union power_supply_propval val = {0,};

	if (ref->use_ext_temp) {
		if (!ref->batt_psy) {
			ref->batt_psy = power_supply_get_by_name("battery");

			if (!ref->batt_psy)
				return false;
		}

		ref->batt_psy->get_property(ref->batt_psy,
				POWER_SUPPLY_PROP_TEMP, &val);

		batt_temp_raw = val.intval;

		pr_max17050(PR_DEBUG, "%s : battery_temp from power_supply %d\n",
				__func__, batt_temp_raw);
		if(batt_temp_raw < 0)
			write_temp = 0xFF00 & (u16)(batt_temp_raw * 256 / 10);
		else
			write_temp = (u16)(batt_temp_raw * 256 / 10);
		max17050_write_reg(max17050_i2c_client,
				MAX17050_TEMPERATURE, write_temp);

		/*At least 3mS of delay added between Write and Read functions*/
		msleep(4);

		read_reg = max17050_read_reg(max17050_i2c_client,
				MAX17050_TEMPERATURE);
		pr_max17050(PR_DEBUG, "%s : battery_temp %X\n",
				__func__, read_reg);
		battery_temp = (read_reg * 10 / 256) / 10;

		if(battery_temp < 0)
			return battery_temp = batt_temp_raw;

	} else {
		pr_max17050(PR_INFO, "%s : Not use batt temp"
				": defalult temp 25C\n", __func__);
		battery_temp = DEFAULT_TEMP;
	}

	pr_max17050(PR_DEBUG, "%s : battery_temp %d\n",
			__func__, battery_temp);

	return battery_temp;
}

int max17050_read_battery_age(void)
{
	u16 read_reg;
	int battery_age;

	if (max17050_i2c_client == NULL) {
		pr_max17050(PR_ERR, "%s : i2c NULL battery age: 800\n", __func__);
		return 800;
	}
	read_reg = max17050_read_reg(max17050_i2c_client, MAX17050_AGE);
	if (read_reg < 0)
		return 999; /*Error Value return.*/

	battery_age = (read_reg >> 8);

	pr_max17050(PR_DEBUG, "%s : battery_age = %d\n", __func__, battery_age);

	return battery_age;
}

int max17050_get_condition(void)
{
	int batt_age = max17050_get_battery_age();
	int batt_condition = 0;

	if (batt_age == 999)
		/* Error or Uncalculated Battery age. */
		batt_condition = 0;
	else if (batt_age >= 80)
		/* Very Good Condition */
		batt_condition = 1;
	else if (batt_age >= 50)
		/* Good Condition */
		batt_condition = 2;
	else if (batt_age >= 0)
		/* Bad Condition */
		batt_condition = 3;
	else
		/* Uncalculated Battery age. */
		batt_condition = 0;

	return batt_condition;
}
#ifdef CONFIG_LGE_PM_MAX17050_RECHARGING
#define RECAHGING_VFLOAT_VOLTAGE 4440
bool max17050_recharging(void)
{
	union power_supply_propval val = {0,};
	int rc;
	if (ref == NULL)
		return false;

	if (!ref->batt_psy) {
		ref->batt_psy = power_supply_get_by_name("battery");

		if (!ref->batt_psy)
			return false;
	}

	rc = ref->batt_psy->get_property(ref->batt_psy,
			POWER_SUPPLY_PROP_VOLTAGE_MAX, &val);

	if (rc < 0) {
	pr_max17050(PR_ERR, "%s : fail vol max property , rc =%d\n",
			__func__, rc);
	}
	pr_max17050(PR_DEBUG, "%s : vfloat voltage = %d, soc_raw = %d \n",
			__func__, val.intval, ref->before_soc_rep_raw);

	if (val.intval == RECAHGING_VFLOAT_VOLTAGE)
		return true;
	else
		return false;
}

#endif

#define REM_CAP_DECREASE_VAL 20
void max17050_battery_capacity_compensate(int battery_full_cap,
		int battery_rem_cap, int before_soc_rep_raw,
		int before_soc_vf_raw, int battery_soc, int battery_current)
{
	u16 read_reg;
	
	/* Full charging case */
	if (battery_rem_cap > battery_full_cap) {
		pr_max17050(PR_ERR, "%s : [CMP - full chg] full_cap and rem_cap before rewrite"
				": full_cap = %d, rem_cap = %d\n",
				__func__, battery_full_cap, battery_rem_cap);
		/* Full&Rem cap errors are occurred*/
		/* so default full capacity value should be rewrited */
		if (battery_full_cap > 3000) {
			pr_max17050(PR_ERR, "[CMP - full chg] CASE1 full cap error,"
					"rem cap error\n");
			max17050_i2c_write_and_verify(MAX17050_FULL_CAP,
					ref->pdata->capacity);
			max17050_write_reg(max17050_i2c_client, MAX17050_DESIGN_CAP,
					ref->pdata->vf_fullcap);
			max17050_i2c_write_and_verify(MAX17050_FULL_CAP_NOM,
					ref->pdata->vf_fullcap);

			max17050_i2c_write_and_verify(MAX17050_REM_CAP_REP,
					ref->pdata->capacity);

			read_reg = max17050_read_reg(max17050_i2c_client, MAX17050_FULL_CAP);
			battery_full_cap = (5 * read_reg) / 10;
			read_reg = max17050_read_reg(max17050_i2c_client, MAX17050_REM_CAP_REP);
			battery_rem_cap = (5 * read_reg) / 10;
		}
		/* Only rem cap error is occurred, so rem cap should be changed to full cap */
		else {
			pr_max17050(PR_DEBUG, "[CMP - full chg] CASE2 Only rem cap error\n");
			read_reg = max17050_read_reg(max17050_i2c_client, MAX17050_FULL_CAP);

			max17050_i2c_write_and_verify(MAX17050_REM_CAP_REP,
					read_reg);
			read_reg = max17050_read_reg(max17050_i2c_client, MAX17050_REM_CAP_REP);
			battery_rem_cap = (5 * read_reg) / 10;
		}
		pr_max17050(PR_ERR, "%s : [CMP - full chg] full_cap and rem_cap rewrite"
				": full_cap = %d, rem_cap = %d\n",
				__func__, battery_full_cap, battery_rem_cap);
	}
	/* Not full charging */
	else {
		pr_max17050(PR_DEBUG, "%s : [CMP - nor chg] full_cap and rem_cap before rewrite"
				": full_cap = %d, rem_cap = %d\n",
				__func__, battery_full_cap, battery_rem_cap);
		/* Without changing SOC, full cap and rem cap are changed */
		if (battery_full_cap > 3000) {
			pr_max17050(PR_ERR, "[CMP - nor] CASE3 Only full cap error\n");
			max17050_i2c_write_and_verify(MAX17050_FULL_CAP,
					ref->pdata->capacity);
			max17050_write_reg(max17050_i2c_client, MAX17050_DESIGN_CAP,
					ref->pdata->vf_fullcap);
			max17050_i2c_write_and_verify(MAX17050_FULL_CAP_NOM,
					ref->pdata->vf_fullcap);
			/*new Full cap*/
			read_reg = max17050_read_reg(max17050_i2c_client, MAX17050_FULL_CAP);
			battery_full_cap = (5 * read_reg) / 10;
			/*new Rem cap*/
			battery_rem_cap = battery_full_cap * before_soc_rep_raw / 1000;

			read_reg = (battery_rem_cap * 10) / 5;
			max17050_i2c_write_and_verify(MAX17050_REM_CAP_REP,
					read_reg);
		}
	}
	pr_max17050(PR_DEBUG, "%s : [CMP - nor] full_cap and rem_cap rewrite"
				": full_cap = %d, rem_cap = %d\n",__func__, battery_full_cap,
				battery_rem_cap);

	return;
}

void max17050_battery_dump_print(void)
{
#ifndef CONFGIG_LGE_PM_MAX17050_DUMP
	u16 read_reg;
	int i;
	printk("Register Dump:");
	max17050_write_reg(max17050_i2c_client, MAX17050_MODEL_LOCK1, 0x59);
	max17050_write_reg(max17050_i2c_client, MAX17050_MODEL_LOCK2, 0xc4);
	for (i = 0; i <= 0xFF; i++) {
		if(i == 0x50)
			i = 0xE0;
		read_reg = max17050_read_reg(max17050_i2c_client, i);
		printk("%04xh,", read_reg);
	}
	max17050_write_reg(max17050_i2c_client, MAX17050_MODEL_LOCK1, 0x0000);
	max17050_write_reg(max17050_i2c_client, MAX17050_MODEL_LOCK2, 0x0000);
	printk("\n");
#endif
}
bool max17050_battery_full_info_print(void)
{
	u16 read_reg;
	u16 learn_cfg;
	int battery_age;
	int battery_remain_capacity;
	int battery_time_to_empty_sec;
	int battery_soc;
	int battery_voltage;
	int battery_temp;
	int battery_current;
	int battery_full_cap;
	int battery_voltage_ocv;
	int vfocv_1;
	int vfocv_2;
	int before_vfocv;
	int before_soc;
	int before_cell;

	battery_age = max17050_read_battery_age();

	read_reg = max17050_read_reg(max17050_i2c_client, MAX17050_REM_CAP_REP);

	battery_remain_capacity = (5 * read_reg) / 10;

	read_reg = max17050_read_reg(max17050_i2c_client, MAX17050_FULL_CAP);

	battery_full_cap = (5 * read_reg) / 10;

	read_reg = max17050_read_reg(max17050_i2c_client, MAX17050_TTE);

	battery_time_to_empty_sec = (5625 * read_reg) / 1000;

	battery_soc = max17050_get_capacity_percent();

	ref->last_soc = battery_soc;

	battery_voltage = max17050_get_mvolts();

	battery_voltage_ocv = max17050_get_ocv_mvolts();

	battery_temp = max17050_write_temp();

	learn_cfg = read_reg = max17050_read_reg(max17050_i2c_client, MAX17050_LEARN_CFG);

	if (battery_temp > 127)
		battery_temp = battery_temp - 256;
	else
		battery_temp = battery_temp;

	battery_current = max17050_get_current();
	read_reg = max17050_read_reg(max17050_i2c_client, MAX17050_RCOMP_0);

	pr_max17050(PR_INFO, "[MAX17050_PRINT_INFO]=F_cap=%d,recap=%d,Ibatt=%d,Iavg=%d,"
			"Empty=%d,Vbatt=%d,OCV=%d,SoC=%d,AfterSoCRep=%d,Raw=%d,AfterSocVF=%d,"
			"Raw=%d,Rcomp0=0x%X,temp=%d,learn=0x%X\n",
			battery_full_cap,battery_remain_capacity,battery_current,ref->avg_ibatt,
			battery_time_to_empty_sec/60,battery_voltage,battery_voltage_ocv,
			battery_soc,ref->soc_rep_raw, ref->before_soc_rep_raw,ref->soc_vf_raw,
			ref->before_soc_vf_raw,read_reg,battery_temp,learn_cfg);

	if (compare_flag == 1) {
		before_soc = read_shutdown_soc("/persist/last_soc",0);
		vfocv_1 = read_shutdown_soc("/persist/last_soc",3);
		vfocv_2 = read_shutdown_soc("/persist/last_soc",6);
		before_vfocv = vfocv_1 + (vfocv_2 << 8);
		before_cell = read_shutdown_soc("/persist/last_soc",9);
		fake_batt_flag = read_shutdown_soc("/persist/last_soc",12);
		max17050_battery_compare_soc(battery_soc, before_soc,
				before_vfocv, before_cell);
	}
	if (compare_flag == 2) {
		write_shutdown_soc("/persist/last_soc",battery_soc,0);
		vfocv_1 = battery_voltage_ocv & 0xFF;
		write_shutdown_soc("/persist/last_soc",vfocv_1,3);
		vfocv_2 = (battery_voltage_ocv >> 8) & 0xFF;
		write_shutdown_soc("/persist/last_soc",vfocv_2,6);
		write_shutdown_soc("/persist/last_soc",cell_info,9);
	}


	max17050_battery_capacity_compensate(battery_full_cap, battery_remain_capacity,
		ref->before_soc_rep_raw, ref->before_soc_vf_raw, battery_soc, battery_current);

	return 0;
}

#define DUMP_PRINT_TIME 40000
static void max17050_dump_work(struct work_struct *work)
{
	struct max17050_chip *chip = container_of(work,
				struct max17050_chip,
				max17050_monitor_work.work);

	max17050_battery_dump_print();

	queue_delayed_work(system_power_efficient_wq,
			&chip->max17050_monitor_work,
			msecs_to_jiffies(DUMP_PRINT_TIME));

}
#define RECAHGING_RAW_SOC 960
static void max17050_monitor_work(struct work_struct *work)
{
	int battery_voltage;
	int battery_temp;
	int battery_current;
	int battery_voltage_ocv;
	int capacity;
	static int cut_off_count = 0;
	static int shut_down_count = 0;
	static int fk_off = 0;

	struct max17050_chip *chip = container_of(work,
				struct max17050_chip,
				max17050_monitor_work.work);

	battery_voltage = max17050_get_mvolts();
	battery_current = max17050_get_current();
	battery_voltage_ocv = max17050_get_ocv_mvolts();

	max17050_battery_full_info_print();

	battery_temp = max17050_write_temp();

	if (battery_temp > 127)
		battery_temp = battery_temp - 256;
	else
		battery_temp = battery_temp;

	pr_max17050(PR_DEBUG, "%s : POLLING_FAKE_SOC_CHECK :"
			"temp = %d,currnet = %d, voltage = %d\n",
			__func__, battery_temp,battery_current,battery_voltage);
#ifdef CONFIG_LGE_PM_MAX17050_POLLING
	if (compare_flag == 0) {
		pr_max17050(PR_INFO, "%s : POLLING_FOR_COMPARE\n", __func__);
		queue_delayed_work(system_power_efficient_wq,
				&chip->max17050_monitor_work,
				msecs_to_jiffies(MAX17050_POLLING_PERIOD_10));
		compare_flag = 1;
	} else if((battery_voltage <= 3270 && battery_current > 0 && battery_temp >= 10)
			|| (battery_voltage <= 3270 && battery_current > 0 && battery_temp <= 10)
			|| fk_off == 1) {
		cut_off_count += 1;
		pr_max17050(PR_DEBUG, "%s : POLLING_FAKE_SOC : count = %d\n",
				__func__, cut_off_count);
		if (cut_off_count <= 1) {
			pr_max17050(PR_ERR, "%s : POLLING_PERIOD_5_FAKE_SOC_PRE\n", __func__);
			queue_delayed_work(system_power_efficient_wq,
			&chip->max17050_monitor_work,
			msecs_to_jiffies(MAX17050_POLLING_PERIOD_5));
		} else {
			pr_max17050(PR_DEBUG, "%s : POLLING_PERIOD_5_FAKE_SOC\n", __func__);
			queue_delayed_work(system_power_efficient_wq,
			&chip->max17050_monitor_work,
			msecs_to_jiffies(MAX17050_POLLING_PERIOD_5));
				if (ref->soc_calc_cut_off_flag < 1) {
					ref->fake_soc = max17050_get_capacity_percent();
					pr_max17050(PR_ERR, "%s : POLLING_FAKE_SOC :"
							"fist read real soc = %d\n",
							__func__, ref->fake_soc);

					ref->soc_calc_cut_off_flag += 1;
					pr_max17050(PR_DEBUG, "%s : POLLING_FAKE_SOC :"
							"resd soc flag = %d\n",
							__func__, ref->soc_calc_cut_off_flag);
				}
			ref->fake_soc -= 1;
			battery_voltage = max17050_get_mvolts();
			fake_batt_flag = 1;
			write_shutdown_soc("/persist/last_soc",fake_batt_flag,12);
			fake_batt_flag = read_shutdown_soc("/persist/last_soc",12);
				if (ref->fake_soc <= 0) {
					ref->fake_soc = 0;
				} else if (battery_voltage <= 3000) {
					if (shut_down_count >= 0) {
						fk_off = 1;
						ref->fake_soc = 0;
						pr_max17050(PR_ERR, "%s : Force shutdown "
								"at low voltage", __func__);
					}
					shut_down_count += 1;
				} else {
					shut_down_count = 0;
				}
			pr_max17050(PR_ERR, "%s : POLLING_FAKE_SOC_FINAL"
					"= %d FK_FLAG = %d\n",
					__func__, ref->fake_soc, fake_batt_flag);
			pr_max17050(PR_ERR, "%s : POLLING_FAKE_SOC_UPDATE BATT_PSY\n", __func__);
			power_supply_changed(chip->batt_psy);
			if (ref->fake_soc <= 2)
				fk_off = 1;
		}
	} else {
		if (fake_batt_flag != 0 && compare_flag == 2) {
			fake_batt_flag = 0;
			write_shutdown_soc("/persist/last_soc",fake_batt_flag,12);
		}
		pr_max17050(PR_DEBUG, "%s : POLLING_NORMAL_WORK\n", __func__);
		capacity = max17050_get_capacity_percent();
		if (ref->fake_soc < capacity && ref->soc_calc_cut_off_flag == 1) {
			if(battery_current < 0)
				ref->fake_soc += 1;
			pr_max17050(PR_ERR, "%s : POLLING_FAKE_SOC_CHG FSOC = %d RSOC = %d\n", __func__, ref->fake_soc, capacity);
			if(ref->fake_soc == capacity)
				ref->soc_calc_cut_off_flag = 0;

			power_supply_changed(chip->batt_psy);

			queue_delayed_work(system_power_efficient_wq,
			&chip->max17050_monitor_work,
			msecs_to_jiffies(MAX17050_POLLING_PERIOD_5));

		}
		cut_off_count = 0;
		if (chip->prev_soc != chip->last_soc) {
			pr_max17050(PR_DEBUG, "%s : Update batt_psy\n", __func__);
			power_supply_changed(chip->batt_psy);
		}
		pr_max17050(PR_DEBUG, "%s : prev_soc:%d, last_soc:%d\n",
				__func__, chip->prev_soc, chip->last_soc);

		chip->prev_soc = chip->last_soc;
#ifdef CONFIG_LGE_PM_MAX17050_TEMP_FOR_DUMP
		if (chip->last_soc <= 3) {
			/* 0%~3% 5sec polling */
			pr_max17050(PR_DEBUG, "%s : POLLING_PERIOD_5\n", __func__);
			queue_delayed_work(system_power_efficient_wq,
					&chip->max17050_monitor_work,
					msecs_to_jiffies(MAX17050_POLLING_PERIOD_5));
		} else if (4 <= chip->last_soc && chip->last_soc <= 7) {
			/* 4%~7% 10sec polling */
			pr_max17050(PR_DEBUG, "%s : POLLING_PERIOD_10\n", __func__);
			queue_delayed_work(system_power_efficient_wq,
					&chip->max17050_monitor_work,
					msecs_to_jiffies(MAX17050_POLLING_PERIOD_10));
		} else {
			/* 8%~100% 20sec polling */
			pr_max17050(PR_DEBUG, "%s : POLLING_PERIOD_20\n", __func__);
			queue_delayed_work(system_power_efficient_wq,
			&chip->max17050_monitor_work,
					msecs_to_jiffies(MAX17050_POLLING_PERIOD_20));
		}
#endif
		if (chip->last_soc <= 15) {
			/* 0%~15% 5sec polling */
			pr_max17050(PR_DEBUG, "%s : POLLING_PERIOD_5\n", __func__);
			queue_delayed_work(system_power_efficient_wq,
					&chip->max17050_monitor_work,
					msecs_to_jiffies(MAX17050_POLLING_PERIOD_5));
		} else {
			/* 16~100% 10sec polling */
			pr_max17050(PR_DEBUG, "%s : POLLING_PERIOD_20\n", __func__);
			queue_delayed_work(system_power_efficient_wq,
					&chip->max17050_monitor_work,
					msecs_to_jiffies(MAX17050_POLLING_PERIOD_20));
		}
	}
#else
		pr_max17050(PR_ERR, "%s : POLLING_PERIOD_5_EX\n", __func__);
		queue_delayed_work(system_power_efficient_wq,
				&chip->max17050_monitor_work,
				msecs_to_jiffies(MAX17050_POLLING_PERIOD_5));
#endif
	if(max17050_recharging() &&
		ref->before_soc_rep_raw <= RECAHGING_RAW_SOC) {
		pr_max17050(PR_INFO,"%s : request recharging.\n", __func__);
		wake_lock(&chip->recharging_lock);
		restart_charging_check_cc(ref->before_soc_rep_raw);
		wake_unlock(&chip->recharging_lock);
	}
}

bool max17050_i2c_write_and_verify(u8 addr, u16 value)
{
	u16 read_reg;

	max17050_write_reg(max17050_i2c_client, addr, value);
	/*Delay at least 3mS*/
	msleep(4);
	read_reg = max17050_read_reg(max17050_i2c_client, addr);

	if (read_reg == value) {
			pr_max17050(PR_DEBUG, "%s() Addr = 0x%X,", __func__, addr);
			pr_max17050(PR_DEBUG, "%s() Value = 0x%X Success\n", __func__, value);
			return 1;
	} else {
		pr_max17050(PR_ERR, "%s : () Addr = 0x%X,", __func__, addr);
		pr_max17050(PR_ERR, "%s :  Value = 0x%X Fail to write.", __func__, value);
		pr_max17050(PR_ERR, "%s :  Write once more.\n", __func__);
			max17050_write_reg(max17050_i2c_client, addr, value);
			return 0;
	}

	return 1;
}

/*charger controller*/
#ifdef CONFIG_LGE_PM
static void external_smb349_enable_charging(bool enable)
{
	union power_supply_propval val = {0,};
	val.intval = enable;
	ref->batt_psy->set_property(ref->batt_psy, POWER_SUPPLY_PROP_CHARGING_ENABLED, &val);
}
#endif

void max17050_recalculate_soc(int before_vfocv)
{
	u16 read_reg;
	u16 write_reg;
	u16 vfsoc;
	u16 rem_cap;
	u16 rep_cap;
	u16 dQ_acc;
	u16 qh_register;

	before_vfocv = (before_vfocv * 100) / 125;
	before_vfocv = before_vfocv << 4;

	/* Set MiscCFG.VEX */
	read_reg = max17050_read_reg(max17050_i2c_client, MAX17050_MISC_CFG);
	write_reg = read_reg | 0x0004;
	max17050_write_reg(max17050_i2c_client, MAX17050_MISC_CFG, write_reg);
	msleep(175);
	/* Write VCell to previously saved VFOCV */
	max17050_write_reg(max17050_i2c_client, MAX17050_V_CELL, before_vfocv);
	/* Quick Start */
	read_reg = max17050_read_reg(max17050_i2c_client, MAX17050_MISC_CFG);
	write_reg = read_reg | 0x0400;
	max17050_write_reg(max17050_i2c_client, MAX17050_MISC_CFG, write_reg);
	msleep(350);
	/* Clear MiscCFG */
	read_reg = max17050_read_reg(max17050_i2c_client, MAX17050_MISC_CFG);
	write_reg = read_reg & 0xFBFB;
	max17050_write_reg(max17050_i2c_client, MAX17050_MISC_CFG, write_reg);


	read_reg = max17050_read_reg(max17050_i2c_client, MAX17050_MISC_CFG);
	pr_max17050(PR_ERR, "%s : Clear MiscCFG = 0x%X\n", __func__, read_reg);

	/*13. Delay at least 350mS*/
	msleep(350);

	/*14. Write VFSOC value to VFSOC 0 and QH0*/
	vfsoc = max17050_read_reg(max17050_i2c_client, MAX17050_SOC_VF);
	pr_max17050(PR_ERR, "%s : ()  vfsoc = 0x%X\n", __func__, vfsoc);
	max17050_write_reg(max17050_i2c_client, MAX17050_VFSOC0_LOCK, 0x0080);
	max17050_i2c_write_and_verify(MAX17050_VFSOC0, vfsoc);
	qh_register = max17050_read_reg(max17050_i2c_client, MAX17050_QH);
	max17050_write_reg(max17050_i2c_client, MAX17050_QH0, qh_register);
	max17050_write_reg(max17050_i2c_client, MAX17050_VFSOC0_LOCK, 0);

	/*15. Advance to Coulomb-Counter Mode */
	max17050_i2c_write_and_verify(MAX17050_CYCLES, 0x0060);

	/*15. Write temperature (default 20 deg C)*/
	/*max17050_i2c_write_and_verify(MAX17050_TEMPERATURE, 0x1400);*/

	/*16. Load New Capacity Parameters*/
	rem_cap = (vfsoc * ref->pdata->vf_fullcap) / 25600;
	pr_max17050(PR_ERR, "%s : ()  vf_full_cap = %d  = 0x%X\n",
			__func__, ref->pdata->vf_fullcap, ref->pdata->vf_fullcap);
	pr_max17050(PR_ERR, "%s : ()  rem_cap = %d  = 0x%X\n",
			__func__, rem_cap, rem_cap);
	max17050_i2c_write_and_verify(MAX17050_REM_CAP_MIX, rem_cap);
	rep_cap = rem_cap;
	max17050_i2c_write_and_verify(MAX17050_REM_CAP_REP, rep_cap);
	dQ_acc = (ref->pdata->capacity / 16);
	max17050_i2c_write_and_verify(MAX17050_D_PACC, 0x0C80);
	max17050_i2c_write_and_verify(MAX17050_D_QACC, dQ_acc);
	max17050_i2c_write_and_verify(MAX17050_FULL_CAP,
			ref->pdata->capacity);
	max17050_write_reg(max17050_i2c_client, MAX17050_DESIGN_CAP,
			ref->pdata->vf_fullcap);
	max17050_i2c_write_and_verify(MAX17050_FULL_CAP_NOM,
			ref->pdata->vf_fullcap);
	max17050_write_reg(max17050_i2c_client, MAX17050_SOC_REP, vfsoc);

	/*17. Initialization Complete*/
	read_reg = max17050_read_reg(max17050_i2c_client, MAX17050_STATUS);
	max17050_i2c_write_and_verify(MAX17050_STATUS, (read_reg & 0xFFFD));

	pr_max17050(PR_INFO, "%s : [CMP] End of the recalculate_soc.\n", __func__);

}

void max17050_battery_compare_soc(int current_soc, int before_soc,
		int before_vfocv, int before_cell)
{
	u8 read_recal_flag;
	int current_vfocv;
	int vfocv_1;
	int vfocv_2;
	int tolerance_hi;
	int tolerance_low;
	int fake_ocv;

	vfocv_1 = max17050_read_reg(max17050_i2c_client,0x14);
	vfocv_2 = max17050_read_reg(max17050_i2c_client,0x15);
	tolerance_hi = max17050_read_reg(max17050_i2c_client,0x20);
	tolerance_low = tolerance_hi * (-1);
	current_vfocv = vfocv_1 + (vfocv_2 << 8);

	read_recal_flag = max17050_read_reg(max17050_i2c_client, 0x33);

	pr_max17050(PR_INFO,"[CMP] c_soc=%d b_soc=%d "
		   "c_vfocv=%d b_vfocv=%d tol=%d\n"
			,current_soc, before_soc, current_vfocv,
			before_vfocv, tolerance_hi);

	if(cell_info == LGC_LLL)
		fake_ocv = FAKE_OCV_LGC;
	else
		fake_ocv = FAKE_OCV_TCD;

	if (fake_batt_flag == 1 && por_state == 1) {
		max17050_recalculate_soc(fake_ocv);
		pr_max17050(PR_ERR, "%s : [CMP] comp fk_power_off(non-por)\n", __func__);
	} else if (read_recal_flag == 1 && recal_volt != 0) {
		max17050_recalculate_soc(recal_volt);
		pr_max17050(PR_ERR, "%s : [CMP] recal flag enablen, recal volt = %d\n",
				__func__,recal_volt);
	} else if (por_state == 0 && before_cell == cell_info && before_soc > 1 &&
			(current_soc - before_soc >= 3 ||
			 current_soc - before_soc <= -3)) {
		if (current_vfocv - before_vfocv <= tolerance_hi &&
				current_vfocv - before_vfocv >= tolerance_low) {
			if (fake_batt_flag == 1) {
				max17050_recalculate_soc(fake_ocv);
				pr_max17050(PR_ERR, "%s : [CMP] comp fk_power_off(por).\n",
						__func__);
			} else {
				max17050_recalculate_soc(before_vfocv);
				pr_max17050(PR_ERR, "%s : [CMP] removed/insert same battery.\n",
						__func__);
			}
		} else {
			pr_max17050(PR_ERR, "%s : [CMP] insert new battery.\n",
					__func__);
		}
	}

	if (fake_batt_flag == 1){
		fake_batt_flag = 0;
		write_shutdown_soc("/persist/last_soc",fake_batt_flag,12);
		pr_max17050(PR_ERR, "%s : [CMP] fk_power_off clear.\n", __func__);
	}
	compare_flag = 2;
}

static int max17050_new_custom_model_write(void)
{
	/*u16 ret;*/
	u16 read_reg;
	u16 misc_flag;
	/*u16 write_reg;*/
	u16 vfsoc;
	u16 rem_cap;
	u16 rep_cap;
	u16 dQ_acc;
	u16 qh_register;
	u16 i;

	u8 read_custom_model_80[MODEL_SIZE];
	u8 read_custom_model_90[MODEL_SIZE];
	u8 read_custom_model_A0[MODEL_SIZE];
	u8 read_recal_flag;
	u8 read_reset_flag;

	pr_max17050(PR_INFO, "%s : Model_data Start\n", __func__);

	read_recal_flag = max17050_read_reg(max17050_i2c_client, 0x33);
	read_reset_flag = max17050_read_reg(max17050_i2c_client, 0x34);
	misc_flag = max17050_read_reg(max17050_i2c_client, MAX17050_MISC_CFG);
	pr_max17050(PR_INFO, "%s :[CMP] recal_flag = %d, creset_flag = %d," 
			"MISC_CFG = 0x%X\n", __func__, read_recal_flag, 
			read_reset_flag, misc_flag);

	/*0. Check for POR or Battery Insertion*/
	read_reg = max17050_read_reg(max17050_i2c_client, MAX17050_STATUS);
	pr_max17050(PR_INFO, "%s : STATUS = 0x%X\n", __func__, read_reg);
	if (((read_reg & (STATUS_POR_BIT | STATUS_BI_BIT)) == 0 && read_recal_flag == 0
#ifdef CONFIG_MACH_MSM8992_PPLUS_KR
	&& misc_flag == 0x810
#endif
	)
	|| (read_reset_flag == 0 && read_recal_flag == 0
#ifdef CONFIG_MACH_MSM8992_PPLUS_KR
			&& misc_flag == 0x810
#endif
	)) {
		pr_max17050(PR_INFO, "%s : IC Non-Power-On-Reset state.", __func__);
		por_state = 1;
		return 2; /*Go to save the custom model.*/
	} else {
		if (((read_reg & (STATUS_POR_BIT | STATUS_BI_BIT)) == 0 && read_recal_flag == 1)
				|| (read_reg == 0x8800 && read_recal_flag == 1)
				|| (read_reg == 0x800A && read_recal_flag == 1)) {
			por_state = 1;
			pr_max17050(PR_INFO, "%s :[CMP]IC Non-Power-On-Reset state."
					" But recal trigger, Start Custom Model Write.\n", __func__);
		} else
			pr_max17050(PR_INFO, "%s : IC Power-On-Reset state."
					" Start Custom Model Write.\n", __func__);

		/*1. Delay 500mS*/
		msleep(500);

		/*1.1 Version Check*/
		read_reg = max17050_read_reg(max17050_i2c_client, MAX17050_VERSION);
		pr_max17050(PR_INFO, "%s : MAX17050_VERSION = 0x%X\n", __func__, read_reg);
		if (read_reg != 0xAC) {
			pr_max17050(PR_ERR, "%s : Version Check Error.", __func__);
			pr_max17050(PR_ERR, "%s :  Version Check = 0x%x\n", __func__, read_reg);
			return 1; /*Version Check Error*/
		}

		/*2. Initialize Configuration*/
		/* External temp and enable alert function */
		max17050_write_reg(max17050_i2c_client, MAX17050_CONFIG,
							ref->pdata->config);
		max17050_write_reg(max17050_i2c_client, MAX17050_FILTER_CFG,
							ref->pdata->filtercfg);
		max17050_write_reg(max17050_i2c_client, MAX17050_RELAX_CFG,
							ref->pdata->relaxcfg);
		max17050_write_reg(max17050_i2c_client, MAX17050_LEARN_CFG,
							ref->pdata->learncfg);
		max17050_write_reg(max17050_i2c_client, MAX17050_MISC_CFG,
							ref->pdata->misccfg);
		max17050_write_reg(max17050_i2c_client, MAX17050_FULL_SOC_THR,
							ref->pdata->fullsocthr);
		max17050_write_reg(max17050_i2c_client, MAX17050_I_AVG_EMPTY,
							ref->pdata->iavg_empty);

		/*4. Unlock Model Access*/
		max17050_write_reg(max17050_i2c_client, MAX17050_MODEL_LOCK1, 0x59);
		max17050_write_reg(max17050_i2c_client, MAX17050_MODEL_LOCK2, 0xc4);

		/*5. Write/Read/Verify the Custom Model*/
		max17050_multi_write_data(max17050_i2c_client,
					MAX17050_MODEL_TABLE_80, ref->pdata->model_80, MODEL_SIZE);
		max17050_multi_write_data(max17050_i2c_client,
					MAX17050_MODEL_TABLE_90, ref->pdata->model_90, MODEL_SIZE);
		max17050_multi_write_data(max17050_i2c_client,
					MAX17050_MODEL_TABLE_A0, ref->pdata->model_A0, MODEL_SIZE);

		/*For test only. Read back written-custom model data.*/
		max17050_multi_read_data(max17050_i2c_client,
					MAX17050_MODEL_TABLE_80, read_custom_model_80, MODEL_SIZE);
		max17050_multi_read_data(max17050_i2c_client,
					MAX17050_MODEL_TABLE_90, read_custom_model_90, MODEL_SIZE);
		max17050_multi_read_data(max17050_i2c_client,
					MAX17050_MODEL_TABLE_A0, read_custom_model_A0, MODEL_SIZE);

		/*Print read_custom_model print */
		for (i = 0; i < MODEL_SIZE; i++)
			pr_max17050(PR_DEBUG, "%s : Model_data_80 %d = 0x%x\n",
					__func__, i, read_custom_model_80[i]);

		for (i = 0; i < MODEL_SIZE; i++)
			pr_max17050(PR_DEBUG, "%s : Model_data_90 %d = 0x%x\n",
					__func__, i, read_custom_model_90[i]);

		for (i = 0; i < MODEL_SIZE; i++)
			pr_max17050(PR_DEBUG, "%s : Model_data_A0 %d = 0x%x\n",
					__func__, i, read_custom_model_A0[i]);

		/*Compare with original one.*/
		for (i = 0 ; i < MODEL_SIZE ; i++) {
			if (read_custom_model_80[i] != ref->pdata->model_80[i]) {
				pr_max17050(PR_INFO, "%s : [MAX17050] Custom Model",
						__func__);
				pr_max17050(PR_INFO, "%s :  1[%d]	Write Error\n",
						__func__, i);
			}
		}

		for (i = 0 ; i < MODEL_SIZE ; i++) {
			if (read_custom_model_90[i] != ref->pdata->model_90[i]) {
				pr_max17050(PR_INFO, "%s : [MAX17050] Custom Model", __func__);
				pr_max17050(PR_INFO, "%s :  2[%d] Write Error\n", __func__, i);
			}
		}

		for (i = 0 ; i < MODEL_SIZE ; i++) {
			if (read_custom_model_A0[i] != ref->pdata->model_A0[i]) {
				pr_max17050(PR_INFO, "%s : [MAX17050] Custom Model", __func__);
				pr_max17050(PR_INFO, "%s :  3[%d] Write Error\n", __func__, i);
			}
		}
		/*For Test only end.*/

		/*8. Lock Model Access*/
		max17050_write_reg(max17050_i2c_client, MAX17050_MODEL_LOCK1, 0x0000);
		max17050_write_reg(max17050_i2c_client, MAX17050_MODEL_LOCK2, 0x0000);

		/*9. Verify the Model Access is locked.*/
		/*Skip.*/

		/*10. Write Custom Parameters*/
		max17050_i2c_write_and_verify(MAX17050_RCOMP_0,
							ref->pdata->rcomp0);
		max17050_i2c_write_and_verify(MAX17050_TEMP_CO,
							ref->pdata->tempco);
		max17050_i2c_write_and_verify(MAX17050_TEMP_NOM,
							ref->pdata->tempnom);
		max17050_i2c_write_and_verify(MAX17050_I_CHG_TERM,
							ref->pdata->ichgterm);
		max17050_i2c_write_and_verify(MAX17050_T_GAIN,
							ref->pdata->tgain);
		max17050_i2c_write_and_verify(MAX17050_T_OFF,
							ref->pdata->toff);
		max17050_i2c_write_and_verify(MAX17050_V_EMPTY,
							ref->pdata->vempty);
		max17050_i2c_write_and_verify(MAX17050_Q_RESIDUAL_00,
							ref->pdata->qrtable00);
		max17050_i2c_write_and_verify(MAX17050_Q_RESIDUAL_10,
							ref->pdata->qrtable10);
		max17050_i2c_write_and_verify(MAX17050_Q_RESIDUAL_20,
							ref->pdata->qrtable20);
		max17050_i2c_write_and_verify(MAX17050_Q_RESIDUAL_30,
							ref->pdata->qrtable30);

		/*max17050_write_reg(max17050_i2c_client, MAX17050_T_GAIN, 0xE932);*/
		/*max17050_write_reg(max17050_i2c_client, MAX17050_T_OFF, 0x2381);*/

		/*11. Update Full Capacity Parameters*/
		max17050_i2c_write_and_verify(MAX17050_REM_CAP_REP, 0x0);
		max17050_i2c_write_and_verify(MAX17050_FULL_CAP,
							ref->pdata->capacity);
		max17050_write_reg(max17050_i2c_client, MAX17050_DESIGN_CAP,
							ref->pdata->vf_fullcap);
		max17050_i2c_write_and_verify(MAX17050_FULL_CAP_NOM,
							ref->pdata->vf_fullcap);

		/*13. Delay at least 350mS*/
		msleep(350);

		/*14. Write VFSOC value to VFSOC 0 and QH0*/
		vfsoc = max17050_read_reg(max17050_i2c_client, MAX17050_SOC_VF);
		pr_max17050(PR_ERR, "%s : ()  vfsoc = 0x%X\n", __func__, vfsoc);
		max17050_write_reg(max17050_i2c_client, MAX17050_VFSOC0_LOCK, 0x0080);
		max17050_i2c_write_and_verify(MAX17050_VFSOC0, vfsoc);
		qh_register = max17050_read_reg(max17050_i2c_client, MAX17050_QH);
		max17050_write_reg(max17050_i2c_client, MAX17050_QH0, qh_register);
		max17050_write_reg(max17050_i2c_client, MAX17050_VFSOC0_LOCK, 0);

		/*15. Advance to Coulomb-Counter Mode */
		max17050_i2c_write_and_verify(MAX17050_CYCLES, 0x0060);

		/*15. Write temperature (default 20 deg C)*/
		/*max17050_i2c_write_and_verify(MAX17050_TEMPERATURE, 0x1400);*/

		/*16. Load New Capacity Parameters*/
		rem_cap = (vfsoc * ref->pdata->vf_fullcap) / 25600;
		pr_max17050(PR_ERR, "%s : ()  vf_full_cap = %d  = 0x%X\n",
			__func__, ref->pdata->vf_fullcap, ref->pdata->vf_fullcap);
		pr_max17050(PR_ERR, "%s : ()  rem_cap = %d  = 0x%X\n",
			__func__, rem_cap, rem_cap);
		max17050_i2c_write_and_verify(MAX17050_REM_CAP_MIX, rem_cap);
		rep_cap = rem_cap;
		max17050_i2c_write_and_verify(MAX17050_REM_CAP_REP, rep_cap);
		dQ_acc = (ref->pdata->capacity / 16);
		max17050_i2c_write_and_verify(MAX17050_D_PACC, 0x0C80);
		max17050_i2c_write_and_verify(MAX17050_D_QACC, dQ_acc);
		max17050_i2c_write_and_verify(MAX17050_FULL_CAP,
							ref->pdata->capacity);
		max17050_write_reg(max17050_i2c_client, MAX17050_DESIGN_CAP,
							ref->pdata->vf_fullcap);
		max17050_i2c_write_and_verify(MAX17050_FULL_CAP_NOM,
							ref->pdata->vf_fullcap);
		max17050_write_reg(max17050_i2c_client, MAX17050_SOC_REP, vfsoc);

		/*17. Initialization Complete*/
		read_reg = max17050_read_reg(max17050_i2c_client, MAX17050_STATUS);
		max17050_i2c_write_and_verify(MAX17050_STATUS, (read_reg & 0xFFFD));

		/*End of the Custom Model step 1.*/
		pr_max17050(PR_INFO, "%s : End of the max17050_new_custom_model_write.\n",
			__func__);

		if(read_recal_flag == 1) {
			pr_max17050(PR_INFO, "%s :[CMP]Recal trigger, Quick start.\n",
					__func__);
			external_smb349_enable_charging(0);
			msleep(100);
			max17050_quick_start();
			recal_volt = max17050_get_mvolts();
			external_smb349_enable_charging(1);
		}
	}
	pr_max17050(PR_INFO, "%s : Model_data End\n", __func__);
	return 0; /*Success to write.*/
}

static void max17050_model_data_write_work(struct work_struct *work)
{
	int ret = 0;

	/*Call max17050_new_custom_model_write*/
	ret = max17050_new_custom_model_write();

	if (ret == 2) {
		pr_max17050(PR_INFO, "%s : NON-POWER_ON reset. Proceed Booting.\n", __func__);
	}
	/*Error occurred. Write once more.*/
	else if (ret == 1) {
		max17050_new_custom_model_write();
		pr_max17050(PR_INFO, "%s : Error occurred. Write once more.\n", __func__);
	}
	/*New Custom model write End.
	Go to max17050_restore_bat_info_from_flash.*/
	else if (ret == 0) {
		pr_max17050(PR_INFO, "%s : POWER_ON reset. Custom Model Success.\n", __func__);
	}
	/*Check to enable external battery temperature from CONFIG*/
	ref->use_ext_temp = (ref->pdata->config & CFG_EXT_TEMP_BIT);
	pr_max17050(PR_INFO, "%s : use_ext_temp = %d\n",
			__func__, ref->use_ext_temp);

#ifdef CONFIG_MACH_MSM8992_PPLUS_KR
	/*dQ_acc and dP_acc rewrite for drift current*/
	max17050_i2c_write_and_verify(MAX17050_D_QACC, 0x05AA);
	max17050_i2c_write_and_verify(MAX17050_D_PACC, 0x3200);
	pr_max17050(PR_INFO, "%s : D_QACC and D_PACC reset\n", __func__);

	/*Apply MAXIM new Implementation - MISC_CFG value = 0x0810 */
	max17050_i2c_write_and_verify(MAX17050_MISC_CFG, 0x0810);
	pr_max17050(PR_INFO, "%s : MISC_CFG initial value changed\n", __func__);

	/*Apply MAXIM new Implementation - Filter_CFG value = 0xCDA4 */
	max17050_i2c_write_and_verify(MAX17050_FILTER_CFG, 0xCDA4);
	pr_max17050(PR_INFO, "%s : FILTER_CFG initial value changed\n", __func__);

	/*Apply MAXIM new Implementation - Relax_CFG value = 0x000F */
	max17050_i2c_write_and_verify(MAX17050_RELAX_CFG, 0x000F);
	pr_max17050(PR_INFO, "%s : RELAX_CFG initial value changed\n", __func__);
#endif
}

#define MAX17050_QUICKSTART_VERIFY_CNT 2
bool max17050_quick_start(void)
{
	u16 read_reg;
	u16 write_reg;
	u16 check_reg;

	int retry_cnt = 0;

	pr_max17050(PR_INFO, "%s : quick_start\n", __func__);
QUICK_STEP1:
	/*1. Set the QuickStart and Verify bits*/
	read_reg = max17050_read_reg(max17050_i2c_client, MAX17050_MISC_CFG);
	write_reg = read_reg | 0x1400;
	max17050_write_reg(max17050_i2c_client, MAX17050_MISC_CFG, 0x810);

	read_reg = max17050_read_reg(max17050_i2c_client, MAX17050_MISC_CFG);
	pr_max17050(PR_ERR, "%s : VFRemCap MiscCFG step 1= 0x%X\n", __func__, read_reg);

	/*2. Verify no memory leaks during Quickstart writing*/
	read_reg = max17050_read_reg(max17050_i2c_client, MAX17050_MISC_CFG);
	pr_max17050(PR_ERR, "%s : VFRemCap MiscCFG step 2 = 0x%X\n", __func__, read_reg);
	check_reg = read_reg & 0x0800;
	if (check_reg != 0x0800) {
		if (retry_cnt <= MAX17050_QUICKSTART_VERIFY_CNT) {
			pr_max17050(PR_ERR, "%s : quick_start error STEP2 retry:%d\n",
					__func__, ++retry_cnt);
			goto QUICK_STEP1;
		} else {
		pr_max17050(PR_ERR, "%s : quick_start error !!!!\n",
					__func__);
		return 1;
		}
	}
	retry_cnt = 0;

QUICK_STEP3:
	/*3. Clean the Verify bit*/
	pr_max17050(PR_ERR, "%s : VFRemCap MiscCFG step 3 = 0x%X\n", __func__, read_reg);
	read_reg = max17050_read_reg(max17050_i2c_client, MAX17050_MISC_CFG);
	write_reg = read_reg & 0xefff;
	max17050_write_reg(max17050_i2c_client, MAX17050_MISC_CFG, write_reg);

	/*4. Verify no memory leaks during Verify bit clearing*/
	read_reg = max17050_read_reg(max17050_i2c_client, MAX17050_MISC_CFG);
	check_reg = read_reg & 0x0800;
	if (check_reg != 0x0800) {
		if (retry_cnt <= MAX17050_QUICKSTART_VERIFY_CNT) {
			pr_max17050(PR_ERR, "%s : quick_start error STEP4 retry:%d\n",
					__func__, ++retry_cnt);
			goto QUICK_STEP3;
		} else {
		pr_max17050(PR_ERR, "%s :  [MAX17050] quick_start error !!!!\n",
					__func__);
		return 1;
		}
	}
	/*5. Delay 500ms*/
	msleep(500);

	/*6. Writing and Verify FullCAP Register Value*/
	max17050_i2c_write_and_verify(MAX17050_FULL_CAP,
						ref->pdata->capacity);

	/*7. Delay 500ms*/
	msleep(500);

	return 0;
}

int max17050_get_soc_for_charging_complete_at_cmd(void)
{

	int guage_level = 0;
	int batt_voltage =0;

/*	pm8921_charger_enable(0);
	pm8921_disable_source_current(1);*/


	/* Reduce charger source */
	external_smb349_enable_charging(0);


	pr_max17050(PR_ERR, "%s : [AT_CMD][at_fuel_guage_level_show]"
			" max17050_quick_start\n", __func__);
	max17050_quick_start();
	guage_level = max17050_get_capacity_percent();
	batt_voltage = max17050_get_mvolts();

	guage_level = real_soc;

	if (guage_level > 100)
		guage_level = 100;
	else if (guage_level < 0)
		guage_level = 0;

/*	pm8921_disable_source_current(0);
	pm8921_charger_enable(1);*/


	/* Restore charger source */
	external_smb349_enable_charging(1);


	pr_max17050(PR_ERR, "%s :  [AT_CMD][at_fuel_guage_soc_for_charging_complete]", __func__);
	pr_max17050(PR_ERR, "%s :  BATT guage_level = %d ,", __func__, guage_level);
	pr_max17050(PR_ERR, "%s :  real_soc = %d\n", __func__, real_soc);
	pr_max17050(PR_ERR, "%s :  real batt voltage = %d\n", __func__, batt_voltage);

	return guage_level;
}
EXPORT_SYMBOL(max17050_get_soc_for_charging_complete_at_cmd);

int max17050_get_battery_mvolts(void)
{
	return max17050_get_mvolts();
}
EXPORT_SYMBOL(max17050_get_battery_mvolts);

u8 at_cmd_buf[2] = {0xff, 0xff};
int max17050_get_battery_capacity_percent(void)
{
	if (at_cmd_buf[0] == 1)
		return at_cmd_buf[1];

	if (lge_power_init_flag_max17050 == 1) {
		max17050_quick_start();
		lge_power_init_flag_max17050 = 0;
	}
	return max17050_get_capacity_percent();
}
EXPORT_SYMBOL(max17050_get_battery_capacity_percent);

int max17050_get_battery_current(void)
{
	return max17050_get_current();
}
EXPORT_SYMBOL(max17050_get_battery_current);

int max17050_write_battery_temp(void)
{
	return max17050_write_temp();
}
EXPORT_SYMBOL(max17050_write_battery_temp);

int max17050_get_battery_age(void)
{
	return max17050_read_battery_age();
}
EXPORT_SYMBOL(max17050_get_battery_age);

int max17050_get_full_design(void)
{
	if (ref == NULL)
		return 2000;

	return ref->pdata->full_design;
	pr_max17050(PR_INFO, "%s : [MAX17050] fuel gauge is not initialized\n", __func__);
}
EXPORT_SYMBOL(max17050_get_full_design);

int max17050_get_battery_condition(void)
{
	return max17050_get_condition();
}
EXPORT_SYMBOL(max17050_get_battery_condition);


/* For max17050 AT cmd */
bool max17050_set_battery_atcmd(int flag, int value)
{
	bool ret;

	u16 soc_read;
	u16 vbat_mv;

	if (flag == 0) {
		/*Call max17050 Quick Start function.*/
		ret = max17050_quick_start();

		if (ret == 0) {
			/*Read and Verify Outputs*/
			soc_read = max17050_get_capacity_percent();
			vbat_mv = max17050_suspend_get_mvolts();
			pr_max17050(PR_ERR, "%s : max17050_quick_start end", __func__);
			pr_max17050(PR_ERR, "%s : Reset_SOC = %d %% vbat = %d mV\n",
				__func__, soc_read, vbat_mv);

			if ((vbat_mv >= 4100) && (soc_read < 70)) {
				at_cmd_buf[0] = 1;
				at_cmd_buf[1] = 100;
				pr_max17050(PR_ERR, "%s : quick_start error.\n",
						__func__);
				return 1;
			} else
				at_cmd_buf[0] = 0;
		} else {
				at_cmd_buf[0] = 1;
				at_cmd_buf[1] = 100;
				pr_max17050(PR_ERR, "%s : quick_start error.\n",
						__func__);
				return 1;
		}
	} else if (flag == 1) {
		at_cmd_buf[0] = 0;
	}

	return 0;
}

static ssize_t at_fuel_guage_reset_show
	(struct device *dev, struct device_attribute *attr, char *buf)
{
	int r = 0;
	bool ret = 0;

	pr_max17050(PR_INFO, "%s : [AT_CMD][at_fuel_guage_reset_show] start\n", __func__);

	/* Reduce charger source */
	external_smb349_enable_charging(0);

	ret = max17050_set_battery_atcmd(0, 100);  /* Reset the fuel guage IC*/
	if (ret == 1)
		pr_max17050(PR_ERR, "%s : at_fuel_guage_reset_show error.\n", __func__);

	r = snprintf(buf, PAGE_SIZE, "%d\n", true);
	/*at_cmd_force_control = TRUE;*/

	msleep(100);

	/* Restore charger source */
	external_smb349_enable_charging(1);

	pr_max17050(PR_INFO, "%s :  [AT_CMD][at_fuel_guage_reset_show] end\n", __func__);

	return r;
}

static ssize_t at_fuel_guage_level_show
	(struct device *dev, struct device_attribute *attr, char *buf)
{
	int r = 0;
	int guage_level = 0;
	int batt_voltage = 0;

	pr_max17050(PR_INFO, "%s :  [AT_CMD][at_fuel_guage_level_show] start\n", __func__);

	/* 121128 doosan.baek@lge.com Implement Power test SOC quickstart */
	if (lge_power_test_flag_max17050 == 1) {
		/*pm8921_charger_enable(0);
		pm8921_disable_source_current(1);*/

		pr_max17050(PR_INFO, "%s :  [AT_CMD][at_fuel_guage_level_show]", __func__);
		pr_max17050(PR_INFO, "%s :  max17050_quick_start\n", __func__);

		/* Reduce charger source */
		external_smb349_enable_charging(0);

		max17050_quick_start();
		guage_level = max17050_get_capacity_percent();
		batt_voltage = max17050_get_mvolts();

		guage_level = real_soc;

		if (guage_level > 100)
			guage_level = 100;
		else if (guage_level < 0)
			guage_level = 0;

		pr_max17050(PR_ERR, "%s :  real batt voltage = %d\n", __func__, batt_voltage);

		if((is_factory_cable_56K || is_factory_cable_130K) && (batt_voltage > 4200)) {
			guage_level = 100;
			pr_max17050(PR_ERR, "%s :  force gauge level 100(56K "
					"and 130K)\n", __func__);
		}

		pr_max17050(PR_ERR, "%s :  [AT_CMD][at_fuel_guage_level_show]\n", __func__);
		pr_max17050(PR_ERR, "%s :  BATT guage_level = %d \n", __func__, guage_level);
		pr_max17050(PR_ERR, "%s :  real_soc = %d\n", __func__, real_soc);

		/*pm8921_disable_source_current(0);
		pm8921_charger_enable(1);*/

		/* Restore charger source */
		external_smb349_enable_charging(1);

		return snprintf(buf, PAGE_SIZE, "%d\n", guage_level);
	}
	/* 121128 doosan.baek@lge.com Implement Power test SOC quickstart */
	guage_level = max17050_get_capacity_percent();
	pr_max17050(PR_INFO, "%s :  [AT_CMD][at_fuel_guage_level_show]",
			__func__);
	pr_max17050(PR_INFO, "%s :  not quick start BATT guage_level = %d\n",
			__func__, guage_level);
	r = snprintf(buf, PAGE_SIZE, "%d\n", guage_level);

	return r;
}

static ssize_t at_batt_level_show
	(struct device *dev, struct device_attribute *attr, char *buf)
{
	int r = 0;
	int battery_level = 0;

	pr_max17050(PR_INFO, "%s : [AT_CMD][at_batt_level_show] start\n",
			__func__);

	/* 121128 doosan.baek@lge.com Implement Power test SOC quickstart */
	if (lge_power_test_flag_max17050 == 1) {
		/*pm8921_charger_enable(0);
		pm8921_disable_source_current(1);*/

		/* Reduce charger source */
		external_smb349_enable_charging(0);


		pr_max17050(PR_INFO, "%s : [AT_CMD][at_batt_level_show]"
				" max17050_quick_start\n", __func__);

		max17050_quick_start();
		battery_level =  max17050_get_battery_mvolts();

		/*pm8921_disable_source_current(0);
		pm8921_charger_enable(1);*/

		/* Restore charger source */
		external_smb349_enable_charging(1);

		pr_max17050(PR_INFO, "%s : [AT_CMD][at_batt_level_show]"
				" flag end\n", __func__);
		pr_max17050(PR_INFO, "%s : [AT_CMD][at_batt_level_show]"
				" BATT LVL = %d\n",
				__func__, battery_level);

		return snprintf(buf, PAGE_SIZE, "%d\n", battery_level);
	}
	/* 121128 doosan.baek@lge.com Implement Power test SOC quickstart */

	battery_level =  max17050_get_battery_mvolts();
	pr_max17050(PR_INFO, "%s : [AT_CMD][at_batt_level_show] end\n",
			__func__);
	pr_max17050(PR_INFO, "%s : not quick start BATT LVL = %d\n",
			__func__, battery_level);

	r = snprintf(buf, PAGE_SIZE, "%d\n", battery_level);

	return r;
}

DEVICE_ATTR(at_fuelrst, 0444, at_fuel_guage_reset_show, NULL);
DEVICE_ATTR(at_fuelval, 0444, at_fuel_guage_level_show, NULL);
DEVICE_ATTR(at_batl, 0444, at_batt_level_show, NULL);
static int  max17050_quickstart_check_chglogo_boot
	(u16 v_cell, u16 v_focv)
{
	u16 result;
	int ret = 0;

	result = v_focv - v_cell;
	pr_max17050(PR_INFO, "%s : chglogoboot-result(%d),"
			"v_focv(%d),vcell(%d)\n", __func__,
			result, v_focv, v_cell);

	if(result >= 25 && result <= 100)
	{
		pr_max17050(PR_INFO, "%s :  skip quickstart\n", __func__);
	}
	else
	{
		pr_max17050(PR_INFO, "%s :  quick_start !!\n", __func__);
		ret = 1;
	}

	return ret;

}
static int  max17050_quickstart_check_normalboot
	(u16 v_cell, u16 v_focv)
{
	u16 result;
	int ret = 0;
#if defined(CONFIG_MACH_MSM8974_G2_DCM)
	u16 result2;
	bool power_ok;

	result2 = v_cell - v_focv;
#endif
	result = v_focv - v_cell;
	pr_max17050(PR_ERR, "%s : normalboot -result (%d),", __func__, result);
	pr_max17050(PR_ERR, "%s : v_focv (%d),  vcell(%d)\n", __func__, v_focv, v_cell);

#if defined(CONFIG_MACH_MSM8974_G2_DCM)
	power_ok = external_smb349_is_charger_present();
#endif

#if defined(CONFIG_MACH_MSM8974_G2_DCM)
	/* For A1 DCM with BT-L7(3000mAh) battery*/
	if (!power_ok) {
		if (v_cell >= 3800) {
			if (result <= 100 || result2 <= 100) {
				pr_max17050(PR_ERR, "%s : skip quickstart more than 3.8V = %d\n",
						__func__, result);
				pr_max17050(PR_ERR, "%s : skip quickstart more than 3.8V = %d\n",
						__func__, result2);
			} else {
				ret = 1;
				pr_max17050(PR_ERR, "%s : quickstart more than 3.8V = %d\n",
						__func__, result);
				pr_max17050(PR_ERR, "%s : quickstart more than 3.8V = %d\n",
						__func__, result2);
			}
		} else {
			if (result <= 120 || result2 <= 120) {
				pr_max17050(PR_ERR, "%s : skip quickstart under 3.8V = %d\n",
						__func__, result);
				pr_max17050(PR_ERR, "%s : skip quickstart under 3.8V = %d\n",
						__func__, result2);
			} else {
				ret = 1;
				pr_max17050(PR_ERR, "%s : quickstart under 3.8V = %d\n",
						__func__, result);
				pr_max17050(PR_ERR, "%s : quickstart under 3.8V = %d\n",
						__func__, result2);
			}
		}
	} else if (power_ok) {
		if (result <= 170 || result2 <= 170) {
			pr_max17050(PR_ERR, "%s : skip quickstart = %d\n",
					__func__, result);
			pr_max17050(PR_ERR, "%s : skip quickstart = %d\n",
					__func__, result2);
		}
		else {
			pr_max17050(PR_ERR, "%s : quickstart result2 = %d\n",
					__func__, result);
			pr_max17050(PR_ERR, "%s : quickstart result2 = %d\n",
					__func__, result2);
			ret = 1;
		}
	}
#else
	/* Brought from Vu, GV model. */
	if (v_cell >= 3800) {
		if (result >= 67 && result <= 100) {
			pr_max17050(PR_ERR, "%s : [MAX17047] - skip quickstart\n", __func__);
		} else {
			pr_max17050(PR_ERR, "%s : [MAX17047] - quick_start !!\n", __func__);
			ret = 1;
		}
	} else {
		if (result >= 72 && result <= 100) {
			pr_max17050(PR_ERR, "%s : [MAX17047]  - skip quickstart\n", __func__);
		} else {
			pr_max17050(PR_ERR, "%s : [MAX17047]  - quick_start !!\n", __func__);
			ret = 1;
		}
	}
#endif

	return ret;
}

void max17050_initial_quickstart_check(void)
{

	u16 read_reg;
	/*u16 write_reg;*/
	u16 vfocv;
	u16 vcell;

	unsigned int *power_on_status = 0;

	read_reg = max17050_read_reg(max17050_i2c_client, MAX17050_V_CELL);
	vcell = (read_reg >> 3);
	vcell = (vcell * 625) / 1000;

	read_reg = max17050_read_reg(max17050_i2c_client, MAX17050_V_FOCV);
	vfocv = (read_reg >> 4);
	vfocv = (vfocv * 125) / 100;


	power_on_status = smem_alloc(SMEM_POWER_ON_STATUS_INFO,
						sizeof(power_on_status), 0, SMEM_ANY_HOST_FLAG);
	if (!power_on_status) {
		pr_max17050(PR_ERR, "%s : max17050 subsystem failure reason:", __func__);
		pr_max17050(PR_ERR, "%s :  (unknown, smem_get_entry failed).\n", __func__);
		return;
	}
#if defined(CONFIG_MACH_MSM8974_G2_DCM)
	/* for A1 DCM PM8941 */
	if((0xff & *power_on_status) ==0x10)
#else
	/* for G, GV PM8941 */
	if((0xff & *power_on_status) ==0x20)
#endif
	{
		if(max17050_quickstart_check_chglogo_boot(vcell, vfocv))
			pr_max17050(PR_ERR, "%s :  chglogo quicksart!!\n", __func__);
			/*goto quick_start;*/
	} else {
		if (max17050_quickstart_check_normalboot(vcell, vfocv))
			pr_max17050(PR_ERR, "%s :  normalboot quicksart!!\n", __func__);
			/*goto quick_start;*/
	}
	return;

/*quick_start:
	max17050_quick_start();
	return;*/
}
EXPORT_SYMBOL(max17050_initial_quickstart_check);

static enum power_supply_property max17050_battery_props[] = {
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_BATTERY_CONDITION,
	POWER_SUPPLY_PROP_BATTERY_AGE,
};

static int max17050_get_property(struct power_supply *psy,
			    enum power_supply_property psp,
			    union power_supply_propval *val)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = max17050_get_battery_mvolts()*1000;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		if(ref->soc_calc_cut_off_flag > 0)
			val->intval = ref->fake_soc;
		else
			val->intval = max17050_get_battery_capacity_percent();
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = max17050_get_battery_current();
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = max17050_get_full_design();
		break;
	case POWER_SUPPLY_PROP_BATTERY_CONDITION:
		val->intval = max17050_get_battery_condition();
		break;
	case POWER_SUPPLY_PROP_BATTERY_AGE:
		val->intval = max17050_get_battery_age();
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static char *pm_power_supplied_to[] = {
	"battery",
};

static struct power_supply max17050_ps = {
	.name = "fuelgauge",
	.type = POWER_SUPPLY_TYPE_FUELGAUGE,
	.supplied_to = pm_power_supplied_to,
	.num_supplicants = ARRAY_SIZE(pm_power_supplied_to),
	.properties = max17050_battery_props,
	.num_properties = ARRAY_SIZE(max17050_battery_props),
	.get_property = max17050_get_property,
};

static void max17050_set_soc_thresholds(struct max17050_chip *chip,
								s16 threshold)
{
	s16 soc_now;
	s16 soc_max;
	s16 soc_min;


	soc_now = max17050_read_reg(chip->client, MAX17050_SOC_REP) >> 8;

	pr_max17050(PR_INFO, "%s : soc_now %d\n", __func__, soc_now);

	soc_max = soc_now + threshold;
	if (soc_max > 100)
		soc_max = 100;
	soc_min = soc_now - threshold;
	if (soc_min < 0)
		soc_min = 0;

	max17050_write_reg(chip->client, MAX17050_SOC_ALRT_THRESHOLD,
		(u16)soc_min | ((u16)soc_max << 8));
}

static irqreturn_t max17050_interrupt(int id, void *dev)
{
	struct max17050_chip *chip = dev;
	struct i2c_client *client = chip->client;
	u16 val;

	pr_max17050(PR_INFO, "%s : Interrupt occured, ID = %d\n", __func__, id);

	val = max17050_read_reg(client, MAX17050_STATUS);

	/* Signal userspace when the capacity exceeds the limits */
	if ((val & STATUS_INTR_SOCMIN_BIT) || (val & STATUS_INTR_SOCMAX_BIT)) {
		/* Clear interrupt status bits */
		max17050_write_reg(client, MAX17050_STATUS, val &
			~(STATUS_INTR_SOCMIN_BIT | STATUS_INTR_SOCMAX_BIT));

		/* Reset capacity thresholds */
		max17050_set_soc_thresholds(chip, 5);

		power_supply_changed(chip->batt_psy);
	}

	return IRQ_HANDLED;
}

static void max17050_complete_init(struct max17050_chip *chip)
{

	struct i2c_client *client = chip->client;
	int val;

	if (client->irq) {
		/* Set capacity thresholds to +/- 5% of current capacity */
		max17050_set_soc_thresholds(chip, 5);

		/* Enable capacity interrupts */
		val = max17050_read_reg(client, MAX17050_CONFIG);

		max17050_write_reg(client, MAX17050_CONFIG,
						val | CFG_ALRT_BIT_ENBL);
		pr_max17050(PR_INFO, "%s : MAX17050_CONFIG_val after write = 0x%X\n",
				__func__, val);
	}

	chip->init_done = true;
}


static void max17050_init_worker(struct work_struct *work)
{
	struct max17050_chip *chip = container_of(work,
				struct max17050_chip, work);

	/*max17050_init_chip(chip);*/
	max17050_complete_init(chip);
}

static int max17050_parse_dt(struct device *dev,
		struct max17050_platform_data *pdata)
{
	struct device_node *dev_node = dev->of_node;
	int i;
	int rc = 0;

#ifdef CONFIG_LGE_PM_BATTERY_ID_CHECKER
	/*Battery ID */
	pr_max17050(PR_INFO, "%s : <Battery ID> cell_info n = %d\n",
			__func__, cell_info);
#endif

	rc = of_property_read_u32(dev_node, "max17050,rsens-microohm",
			&pdata->r_sns);

	if (!pdata->r_sns)
		pdata->enable_current_sense = true;
	else
		pdata->r_sns = MAX17050_DEFAULT_SNS_RESISTOR;

	/* Load Battery cell*/
	if (cell_info == LGC_LLL) { /*LGC Battery*/
		rc = of_property_read_u8_array(dev_node, "max17050,model_80_l",
				pdata->model_80, MODEL_SIZE);

		rc = of_property_read_u8_array(dev_node, "max17050,model_90_l",
				pdata->model_90, MODEL_SIZE);

		rc = of_property_read_u8_array(dev_node, "max17050,model_A0_l",
				pdata->model_A0, MODEL_SIZE);

		rc = of_property_read_u32(dev_node, "max17050,rcomp0_l",
				&pdata->rcomp0);

		rc = of_property_read_u32(dev_node, "max17050,tempco_l",
				&pdata->tempco);

		rc = of_property_read_u32(dev_node, "max17050,ichgterm_l",
				&pdata->ichgterm);

		rc = of_property_read_u32(dev_node, "max17050,vempty_l",
				&pdata->vempty);

		rc = of_property_read_u32(dev_node, "max17050,qrtable00_l",
				&pdata->qrtable00);

		rc = of_property_read_u32(dev_node, "max17050,qrtable10_l",
				&pdata->qrtable10);

		rc = of_property_read_u32(dev_node, "max17050,qrtable20_l",
				&pdata->qrtable20);

		rc = of_property_read_u32(dev_node, "max17050,qrtable30_l",
				&pdata->qrtable30);

		rc = of_property_read_u32(dev_node, "max17050,capacity_l",
				&pdata->capacity);

		rc = of_property_read_u32(dev_node, "max17050,vf_fullcap_l",
				&pdata->vf_fullcap);

		rc = of_property_read_u32(dev_node, "max17050,iavg_empty_l",
			&pdata->iavg_empty);

		rc = of_property_read_u32(dev_node, "max17050,rescale_factor_l",
			&pdata->rescale_factor);
		rc = of_property_read_u32(dev_node, "max17050,rescale_soc_l",
			&pdata->rescale_soc);

	} else { /*Tocad battery*/
		rc = of_property_read_u8_array(dev_node, "max17050,model_80_t",
				pdata->model_80, MODEL_SIZE);

		rc = of_property_read_u8_array(dev_node, "max17050,model_90_t",
				pdata->model_90, MODEL_SIZE);

		rc = of_property_read_u8_array(dev_node, "max17050,model_A0_t",
				pdata->model_A0, MODEL_SIZE);

		rc = of_property_read_u32(dev_node, "max17050,rcomp0_t",
				&pdata->rcomp0);

		rc = of_property_read_u32(dev_node, "max17050,tempco_t",
				&pdata->tempco);

		rc = of_property_read_u32(dev_node, "max17050,ichgterm_t",
				&pdata->ichgterm);

		rc = of_property_read_u32(dev_node, "max17050,vempty_t",
				&pdata->vempty);

		rc = of_property_read_u32(dev_node, "max17050,qrtable00_t",
				&pdata->qrtable00);

		rc = of_property_read_u32(dev_node, "max17050,qrtable10_t",
				&pdata->qrtable10);

		rc = of_property_read_u32(dev_node, "max17050,qrtable20_t",
				&pdata->qrtable20);

		rc = of_property_read_u32(dev_node, "max17050,qrtable30_t",
				&pdata->qrtable30);

		rc = of_property_read_u32(dev_node, "max17050,capacity_t",
				&pdata->capacity);

		rc = of_property_read_u32(dev_node, "max17050,vf_fullcap_t",
				&pdata->vf_fullcap);

		rc = of_property_read_u32(dev_node, "max17050,iavg_empty_t",
			&pdata->iavg_empty);

		rc = of_property_read_u32(dev_node, "max17050,rescale_factor_t",
			&pdata->rescale_factor);

		rc = of_property_read_u32(dev_node, "max17050,rescale_soc_t",
			&pdata->rescale_soc);
	}
	rc = of_property_read_u32(dev_node, "max17050,config",
			&pdata->config);

	rc = of_property_read_u32(dev_node, "max17050,relaxcfg",
			&pdata->relaxcfg);

	rc = of_property_read_u32(dev_node, "max17050,filtercfg",
			&pdata->filtercfg);

	rc = of_property_read_u32(dev_node, "max17050,learncfg",
			&pdata->learncfg);

	rc = of_property_read_u32(dev_node, "max17050,misccfg",
			&pdata->misccfg);

	rc = of_property_read_u32(dev_node, "max17050,fullsocthr",
			&pdata->fullsocthr);

	rc = of_property_read_u32(dev_node, "max17050,tempnom",
				&pdata->tempnom);

	rc = of_property_read_u32(dev_node, "max17050,tgain",
			&pdata->tgain);

	rc = of_property_read_u32(dev_node, "max17050,toff",
			&pdata->toff);

	rc = of_property_read_u32(dev_node, "max17050,param-version",
			&pdata->param_version);

	rc = of_property_read_u32(dev_node, "max17050,full_design",
			&pdata->full_design);


	/* Debug log model data by dtsi parsing */
	for (i = 0; i < MODEL_SIZE; i++)
		pr_max17050(PR_DEBUG, "%s : Model_data_80 %d = 0x%x\n",
				__func__, i, pdata->model_80[i]);

	for (i = 0; i < MODEL_SIZE; i++)
		pr_max17050(PR_DEBUG, "%s : Model_data_90 %d = 0x%x\n",
				__func__, i, pdata->model_90[i]);

	for (i = 0; i < MODEL_SIZE; i++)
		pr_max17050(PR_DEBUG, "%s : Model_data_A0 %d = 0x%x\n",
				__func__, i, pdata->model_A0[i]);

	pr_max17050(PR_INFO, "%s : Platform data : "\
			"rcomp = 0x%x, "\
			"tempco = 0x%x, "\
			"ichgterm = 0x%x, "\
			"vempty = 0x%x, "\
			"full_design = 0x%x "\
			"rescale_soc =%d, "\
			"rescale_factor =%d\n",
			__func__,
			pdata->rcomp0,
			pdata->tempco,
			pdata->ichgterm,
			pdata->vempty,
			pdata->vf_fullcap,
			pdata->rescale_soc,
			pdata->rescale_factor);

	return rc;
}

static int max17050_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct max17050_chip *chip = NULL;
	int ret = 0;
	int rc = 0;
	int cable_type = 0;
#ifdef CONFIG_LGE_PM_FACTORY_TESTMODE
	unsigned int *p_cable_type = (unsigned int *)
		(smem_get_entry(SMEM_ID_VENDOR1, &cable_smem_size, 0, 0));
#endif

#ifdef CONFIG_LGE_PM_BATTERY_ID_CHECKER
	unsigned int smem_size = 0;
#if defined(CONFIG_LGE_LOW_BATT_LIMIT)
	uint	_batt_id_ = 0;
#endif
	unsigned int *batt_id = (unsigned int *)
		(smem_get_entry(SMEM_BATT_INFO, &smem_size, 0, 0));

	pr_max17050(PR_INFO, "%s : ----- Start-----\n", __func__);

#if defined(CONFIG_LGE_LOW_BATT_LIMIT)
	if (smem_size != 0 && batt_id) {
		_batt_id_ = (*batt_id >> 8) & 0x00ff;
		if (_batt_id_ == BATT_NOT_PRESENT) {
			pr_max17050(PR_INFO, "%s :  probe : skip for no model data\n", __func__);
			ref = NULL;
			ret = -EINVAL;
			goto error;
		}
#ifdef CONFIG_LGE_PM_BATTERY_ID_CHECKER
		 else if (_batt_id_ == BATT_ID_DS2704_L
			|| _batt_id_ == BATT_ID_ISL6296_C
			|| _batt_id_ == BATT_ID_RA4301_VC1
			|| _batt_id_ == BATT_ID_SW3800_VC0) {
			cell_info = LGC_LLL; /* LGC Battery */
			pr_max17050(PR_INFO, "%s : LGC profile\n", __func__);
		} else if (_batt_id_ == BATT_ID_DS2704_C
			|| _batt_id_ == BATT_ID_ISL6296_L
			|| _batt_id_ == BATT_ID_RA4301_VC0
			|| _batt_id_ == BATT_ID_SW3800_VC1) {
			cell_info = LGC_LLL;
			pr_max17050(PR_INFO, "%s : TCD cell, but using LGC profile by force\n",
					__func__);
		} else {
			pr_max17050(PR_INFO, "%s : Unknown cell, Using LGC profile\n",
					__func__);
			cell_info = LGC_LLL;
		}
#endif
	}

#else
	if (smem_size != 0 && batt_id) {
		if (*batt_id == BATT_NOT_PRESENT) {
			pr_max17050(PR_INFO, "%s : [MAX17050] probe : skip for no model data\n",
					__func__);
			ref = NULL;
			ret = -EINVAL;
			goto error;
		}
#ifdef CONFIG_LGE_PM_BATTERY_ID_CHECKER
		 else if (*batt_id == BATT_ID_DS2704_L
			|| *batt_id == BATT_ID_ISL6296_C
			||  *batt_id == BATT_ID_RA4301_VC1
			||  *batt_id == BATT_ID_SW3800_VC0) {
			cell_info = LGC_LLL; /* LGC Battery */
			pr_max17050(PR_INFO, "%s : LGC profile\n", __func__);
		} else if (*batt_id == BATT_ID_DS2704_C
			|| *batt_id == BATT_ID_ISL6296_L
			|| *batt_id == BATT_ID_RA4301_VC0
			|| *batt_id == BATT_ID_SW3800_VC1) {
			cell_info = TCD_AAC; /* Tocad, Hitachi Battery */
			pr_max17050(PR_INFO, "%s : TDC profile\n", __func__);
		} else {
			pr_max17050(PR_INFO, "%s : Unknown cell, Using LGC profile\n", __func__);
			cell_info = LGC_LLL;
		}
#endif
	}
#endif
#endif


#ifdef CONFIG_LGE_PM_FACTORY_TESTMODE
	if (p_cable_type)
		cable_type = *p_cable_type;
	else
		cable_type = 0;

	if (cable_type == LT_CABLE_56K || cable_type == LT_CABLE_130K ||
		cable_type == LT_CABLE_910K) {
		lge_power_init_flag_max17050 = 1;
		pr_max17050(PR_INFO, "%s : cable_type is = %d"
			" factory_mode quick start \n", __func__, cable_type);
	}
	if (cable_type == LT_CABLE_56K)
		is_factory_cable_56K = 1;
	if (cable_type == LT_CABLE_130K)
		is_factory_cable_130K = 1;
#endif


	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_WORD_DATA)) {
		ret = -EIO;
		goto error;
	}

	chip = kzalloc(sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		ret = -ENOMEM;
		goto error;
	}

	chip->client = client;

	chip->pdata = devm_kzalloc(&client->dev,
					 sizeof(struct max17050_platform_data),
					 GFP_KERNEL);
	if (!chip->pdata) {
		pr_max17050(PR_ERR, "%s : missing platform data\n", __func__);
		ret = -ENODEV;
		goto error;
	}


	chip->dc_psy = power_supply_get_by_name("dc");
	if (!chip->dc_psy) {
		pr_max17050(PR_ERR, "%s : DC supply not found, deferring probe\n", __func__);
		ret = -EPROBE_DEFER;
		goto error;
	}

	chip->batt_psy = power_supply_get_by_name("battery");
	if (!chip->dc_psy) {
		pr_max17050(PR_ERR, "%s : batt_psy supply not found, deferring probe\n", __func__);
		ret = -EPROBE_DEFER;
		goto error;
	}

#ifdef CONFIG_OF
	if (!(&client->dev.of_node)) {
		pr_max17050(PR_ERR, "%s : max17050_probe of_node err.\n", __func__);
		goto error;
	}

	ret = max17050_parse_dt(&client->dev, chip->pdata);

	if (ret != 0) {
		pr_max17050(PR_ERR, "%s : device tree parsing error\n", __func__);
		goto error;
	}
#else
	chip->pdata = client->dev.platform_data;
#endif

	i2c_set_clientdata(client, chip);

	max17050_i2c_client = client;

	ref = chip;

	INIT_DELAYED_WORK(&chip->max17050_model_data_write_work, max17050_model_data_write_work);
	queue_delayed_work(system_power_efficient_wq,
	&chip->max17050_model_data_write_work, 0);

	if (client->irq) {
		ret = request_threaded_irq(client->irq, NULL,
					max17050_interrupt,
					IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					chip->battery.name, chip);
		if (ret) {
			dev_err(&client->dev, "cannot enable irq");
			goto error;
		} else {
			enable_irq_wake(client->irq);
		}
	}

	INIT_WORK(&chip->work, max17050_init_worker);
	mutex_init(&chip->mutex);
#ifdef CONFIG_LGE_PM_FACTORY_TESTMODE
	/* sysfs path : /sys/bus/i2c/drivers/max17050/1-0036/at_fuelrst */
	ret = device_create_file(&client->dev, &dev_attr_at_fuelrst);
	if (ret < 0) {
		pr_max17050(PR_ERR, "%s : File device creation failed: %d\n", __func__, ret);
		ret = -ENODEV;
		goto err_create_file_fuelrst_failed;
	}
	/* sysfs path : /sys/bus/i2c/drivers/max17050/1-0036/at_fuelval */
	ret = device_create_file(&client->dev, &dev_attr_at_fuelval);
	if (ret < 0) {
		pr_max17050(PR_ERR, "%s : File device creation failed: %d\n", __func__, ret);
		ret = -ENODEV;
		goto err_create_file_fuelval_failed;
	}
	/* sysfs path : /sys/bus/i2c/drivers/max17050/1-0036/at_batl */
	ret = device_create_file(&client->dev, &dev_attr_at_batl);
	if (ret < 0) {
		pr_max17050(PR_ERR, "%s : File device creation failed: %d\n", __func__, ret);
		ret = -ENODEV;
		goto err_create_file_batl_failed;
	}
#endif
	rc =  power_supply_register(&chip->client->dev, &max17050_ps);
	if (rc < 0) {
		pr_max17050(PR_ERR, "%s : [MAX17050]failed to register a power_supply rc = %d\n"
			, __func__, ret);
	}
	ref = chip;

	INIT_DELAYED_WORK(&chip->max17050_monitor_work, max17050_monitor_work);
	queue_delayed_work(system_power_efficient_wq,
	&chip->max17050_monitor_work, 0);
	INIT_DELAYED_WORK(&chip->max17050_dump_work, max17050_dump_work);
	queue_delayed_work(system_power_efficient_wq,
	&chip->max17050_dump_work, 0);
#ifdef CONFIG_LGE_PM_MAX17050_RECHARGING
	wake_lock_init(&chip->recharging_lock, WAKE_LOCK_SUSPEND, "max17050_recharging");
#endif
	pr_max17050(PR_INFO, "%s : () ----- End -----\n", __func__);

	return 0;

err_create_file_fuelrst_failed:
	device_remove_file(&client->dev, &dev_attr_at_fuelrst);
	pr_max17050(PR_ERR, "%s : Probe err_create_file_fuelrst_failed\n ", __func__);
err_create_file_fuelval_failed:
	device_remove_file(&client->dev, &dev_attr_at_fuelval);
	pr_max17050(PR_ERR, "%s : Probe err_create_file_fuelval_failed!!!\n ", __func__);
err_create_file_batl_failed:
	device_remove_file(&client->dev, &dev_attr_at_batl);
	pr_max17050(PR_ERR, "%s : Probe err_create_file_batl_failed!!!\n ", __func__);
error:
	pr_max17050(PR_ERR, "%s : [MAX17050] Probe fail!!!\n", __func__);
	pr_max17050(PR_ERR, "%s : Probe fail!!!\n ", __func__);

	if (chip != NULL)
		kfree(chip);

	return ret;
}

static int max17050_remove(struct i2c_client *client)
{
	struct max17050_chip *chip = i2c_get_clientdata(client);

	device_remove_file(&client->dev, &dev_attr_at_fuelrst);
	device_remove_file(&client->dev, &dev_attr_at_fuelval);
	device_remove_file(&client->dev, &dev_attr_at_batl);

#ifdef CONFIG_LGE_PM_MAX17050_RECHARGING
	wake_lock_destroy(&chip->recharging_lock);
#endif

	/*power_supply_unregister(&chip->battery);*/
	i2c_set_clientdata(client, NULL);
	power_supply_unregister(&chip->battery);
	kfree(chip);
	return 0;
}
static int max17050_pm_prepare(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct max17050_chip *chip = i2c_get_clientdata(client);

	/* Cancel any pending work, if necessary */
	cancel_delayed_work_sync(&ref->max17050_monitor_work);
	cancel_delayed_work_sync(&ref->max17050_dump_work);

	chip->suspended = true;
	if (chip->client->irq) {
		disable_irq(chip->client->irq);
		enable_irq_wake(chip->client->irq);
	}
	return 0;
}

static void max17050_pm_complete(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct max17050_chip *chip = i2c_get_clientdata(client);

	chip->suspended = false;
	if (chip->client->irq) {
		disable_irq_wake(chip->client->irq);
		enable_irq(chip->client->irq);
	}

	/* Schedule update, if needed */
	queue_delayed_work(system_power_efficient_wq,
	&ref->max17050_monitor_work, msecs_to_jiffies(HZ));
	queue_delayed_work(system_power_efficient_wq,
	&ref->max17050_dump_work, msecs_to_jiffies(HZ));
}

static const struct dev_pm_ops max17050_pm_ops = {
	.prepare = max17050_pm_prepare,
	.complete = max17050_pm_complete,
};

static struct of_device_id max17050_match_table[] = {
	{ .compatible = "maxim,max17050", },
	{ },
};

static const struct i2c_device_id max17050_id[] = {
	{ "max17050", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, max17050_id);

static struct i2c_driver max17050_i2c_driver = {
	.driver	= {
		.name	= "max17050",
		.owner	= THIS_MODULE,
		.of_match_table = max17050_match_table,
		.pm = &max17050_pm_ops,

	},
	.probe		= max17050_probe,
	.remove		= max17050_remove,
	.id_table	= max17050_id,
};

static int __init max17050_init(void)
{
	return i2c_add_driver(&max17050_i2c_driver);
}
module_init(max17050_init);

static void __exit max17050_exit(void)
{
	i2c_del_driver(&max17050_i2c_driver);
}
module_exit(max17050_exit);

MODULE_AUTHOR("LG Power <lge.com>");
MODULE_DESCRIPTION("MAX17050 Fuel Gauge");
MODULE_LICENSE("GPL");
