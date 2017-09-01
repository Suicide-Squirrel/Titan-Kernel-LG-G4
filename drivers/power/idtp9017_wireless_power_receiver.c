/*
 * IDPT9025A Wireless Power Receiver driver
 *
 * Copyright (C) 2014 LG Electronics, Inc
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 */

#define DEBUG

#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/regulator/machine.h>
#include <linux/string.h>
#include <linux/power/idtp9017_wireless_charger.h>
#include <linux/pm.h>

#ifdef CONFIG_LGE_PM_CHARGING_USING_CHGSTS_WLC
#include <linux/power_supply.h>
#endif

struct idtp9017_chip {
	struct i2c_client *client;
	int wlc_active_n_gpio;
	int wlc_full_chg_gpio;
	int wlc_off_gpio;
	int set_out_voltage;
	int set_limit_current_ma;
	int x_axis;
	int y_axis;
	int fod1_gain;
	int fod2_gain;
	int die_shdn_off;
	int die_shdn_hys;
	int die_temp_off;
	int die_temp_hys;
	int mode_depth;
	bool suspend;
	bool wlc_chg_en;
#ifdef CONFIG_LGE_PM_CHARGING_USING_CHGSTS_WLC
	int previous_soc;
	struct power_supply 	*fuelgauge;
	struct delayed_work 	update_soc_worker;
#endif
	struct delayed_work 	wlc_status_work;
	struct delayed_work 	set_env_work;
};

struct idtp9017_chip *the_chip;

static int idtp9017_read_reg(struct i2c_client *client, int reg, u8 *val)
{
	s32 ret;

	ret = i2c_smbus_read_byte_data(client, reg);
	if (ret < 0) {
		dev_err(&client->dev, "i2c read fail: can't read from %02x\n",
			reg);
		return ret;
	} else {
		*val = ret;
	}
	return 0;
}

static int idtp9017_write_reg(struct i2c_client *client, int reg, u8 val)
{
	s32 ret;

	ret = i2c_smbus_write_byte_data(client, reg, val);
	if (ret < 0) {
		dev_err(&client->dev, "i2c write fail: can't write %02x to %02x\n",
				val, reg);
		return ret;
	}
	return 0;
}

static int idtp9017_masked_write(struct i2c_client *client, u8 reg,
		u8 mask, u8 val) {
	s32 ret;
	u8 temp = 0x00;

	ret = idtp9017_read_reg(client, reg, &temp);
	if (ret < 0) {
		pr_err("Failed to read reg(%02x)\n", temp);
		return ret;
	}

	temp &= ~mask;
	temp |= val & mask;

	ret = idtp9017_write_reg(client, reg, temp);
	if (ret < 0) {
		pr_err("Failed to write reg(%02x)\n", reg);
		return ret;
	}

	return ret;
}

static int idtp9017_wlc_status(struct idtp9017_chip *chip) {
	u8 reg_val_H = 0x00;
	u8 reg_val_L = 0x00;
	int ret = 0;
	int enable = 0;

	ret = idtp9017_read_reg(chip->client, RDST_6A_H, &reg_val_H);
	if (ret < 0) {
		pr_err("Fail to read RDST_6A_H\n");
		return ret;
	}

	pr_debug("Read reg_status_H : 0x%02x\n", reg_val_H);
	reg_val_H &= (ABNM_RAW_15 | ABNM_RAW_14 | ABNM_RAW_10);

	if (reg_val_H && ABNM_RAW_15) {
		pr_info("[IDTP9017] TX detection\n");
		enable = 0;
	} else if (reg_val_H && ABNM_RAW_14) {
		pr_err("[IDTP9017] Too low freq\n");
		enable = 0;
	} else if (reg_val_H && ABNM_RAW_10) {
		pr_err("[IDTP9017] Vrect over 8.5V\n");
		enable = 0;
	} else {
		enable = 1;
	}

	msleep(200);

	ret = idtp9017_read_reg(chip->client, RDST_6A_L, &reg_val_L);
	if (ret < 0) {
		pr_err("Fail to read RDST_6A_L\n");
		return ret;
	}

	pr_debug("Read reg_status_L : 0x%02x\n", reg_val_L);
	reg_val_L &= (ABNM_RAW_2 | ABNM_RAW_0);
	if (reg_val_L && ABNM_RAW_2) {
		pr_info("[IDTP9017] Charging complete\n");
		enable = 2;
	} else if (reg_val_L && ABNM_RAW_0) {
		pr_err("[IDTP9017] Disable wlc charger\n");
		enable = 0;
	} else {
		enable = 1;
	}

	return enable;
}

#ifdef CONFIG_LGE_PM_CHARGING_USING_CHGSTS_WLC
static int idtp9017_get_soc(struct idtp9017_chip *chip) {
	int soc = 0;
	union power_supply_propval ret = {0, };

	if (chip->fuelgauge == NULL)
		chip->fuelgauge = power_supply_get_by_name("fuelgauge");

	if (chip->fuelgauge != NULL) {
		chip->fuelgauge->get_property(chip->fuelgauge,
				POWER_SUPPLY_PROP_CAPACITY, &ret);
		soc = ret.intval;
	} else {
		soc = chip->previous_soc;
		pr_err("Failed to read soc\n");
	}

	return soc;
}

static int idtp9017_set_chg_status(struct idtp9017_chip *chip) {
	int ret = 0;
	u8 reg_val = 0x00;

	reg_val = (u8)idtp9017_get_soc(chip);

	if (reg_val >= 0x64)  /* 100(DEX) -> 64(HEX) */
		reg_val = 0x64;

	ret = idtp9017_masked_write(chip->client, REG_CHGSTS,
			CHGSTS_FRC, reg_val);
	if (ret < 0) {
		pr_err("Failed to write chgsts_reg\n");
		return ret;
	}

	return 0;
}

static void update_wlc_worker(struct work_struct *work) {
	struct delayed_work *dwork = to_delayed_work(work);
	struct idtp9017_chip *chip = container_of(dwork, struct idtp9017_chip,
			update_soc_worker);
	int ret = 0;
	int cur_soc = 0;

	if (chip->wlc_chg_en) {
		cur_soc = idtp9017_get_soc(chip);

		if (cur_soc != chip->previous_soc) {
			ret = idtp9017_set_chg_status(chip);
			if (ret < 0) {
				pr_err("Failed to write chg_status at worker ret : %d\n",
						ret);
				return;
			}
		}
		chip->previous_soc = cur_soc;
	}
	schedule_delayed_work(&chip->update_soc_worker, round_jiffies_relative
			(msecs_to_jiffies(200)));
}
#endif

static int idtp9017_get_chg_mode(struct idtp9017_chip *chip) {
	u8 reg_val = 0x00;
	int ret = 0;
	bool wpc_mode = false;

	ret = idtp9017_read_reg(chip->client, REG_CHG_MODE, &reg_val);
	if (ret < 0) {
		pr_err("Failed to read REG_CHG_MODE\n");
		return ret;
	}

	pr_debug("Charging mode reg_val : 0x%02x\n", reg_val);

	if (reg_val >= 0x01)
		wpc_mode = true;
	else
		wpc_mode = false;

	return wpc_mode;
}

static int idtp9017_get_out_voltage(struct idtp9017_chip *chip) {
	u8 reg_val_1 = 0x00;
	u8 reg_val_2 = 0x00;
	u16 reg_sum = 0x00;
	int ret = 0;
	int read_voltage = 0;

	ret = idtp9017_read_reg(chip->client, RDST_32_H, &reg_val_1);
	if (ret < 0) {
		pr_err("Fail to Vout rdst_32_h reg\n");
		return ret;
	}

	reg_val_1 &= ADC_4BIT;
	reg_sum = reg_val_1 << SHIFT_FOR_ADC;

	ret = idtp9017_read_reg(chip->client, RDST_32_L, &reg_val_2);
	if (ret < 0) {
		pr_err("Fail to Vout rdst_32_L reg\n");
		return ret;
	}

	reg_sum |= reg_val_2;
	read_voltage = (reg_sum * 25) / 10;
	pr_debug("Read Vout : %d\n", read_voltage);

	return read_voltage;
}

static int idtp9017_get_out_current(struct idtp9017_chip *chip) {
	u8 reg_val_1 = 0x00;
	u8 reg_val_2 = 0x00;
	u16 reg_sum = 0x0000;
	int ret = 0;
	int read_current = 0;

	ret = idtp9017_read_reg(chip->client, RDST_31_H, &reg_val_1);
	if (ret < 0) {
		pr_err("Fail to Vout rdst_31_h reg\n");
		return ret;
	}
	reg_val_1 &= ADC_4BIT;
	reg_sum = reg_val_1 << SHIFT_FOR_ADC;
	ret = idtp9017_read_reg(chip->client, RDST_31_L, &reg_val_2);
	if (ret < 0) {
		pr_err("Fail to Vout rdst_31_L reg\n");
		return ret;
	}

	reg_sum |= reg_val_2;
	read_current = ((int)reg_sum * 5) / 10;
	pr_debug("Read Iout : %d\n", read_current);

	return read_current;
}

static int idtp9017_get_voltage_rect(struct idtp9017_chip *chip) {
	u8 reg_val_1 = 0x00;
	u8 reg_val_2 = 0x00;
	u16 reg_sum = 0x0000;
	int ret = 0;
	int read_voltage = 0;

	ret = idtp9017_read_reg(chip->client, RDST_30_H, &reg_val_1);
	if (ret < 0) {
		pr_err("Fail to Vout rdst_30_h reg\n");
		return ret;
	}
	reg_val_1 &= ADC_4BIT;
	reg_sum = reg_val_1 << SHIFT_FOR_ADC;
	ret = idtp9017_read_reg(chip->client, RDST_30_L, &reg_val_2);
	if (ret < 0) {
		pr_err("Fail to Vout rdst_30_L reg\n");
		return ret;
	}

	reg_sum |= reg_val_2;
	read_voltage = (reg_sum * 25) / 10;
	pr_debug("Read Vrect : %d\n", read_voltage);

	return read_voltage;
}

static int idtp9017_get_die_temperature(struct idtp9017_chip *chip) {
	u8 reg_val = 0x00;
	int ret = 0;
	int read_temperature = 0;

	ret = idtp9017_read_reg(chip->client, RDST_33_L, &reg_val);
	if (ret < 0) {
		pr_err("Fail to Vout rdst_33_L reg\n");
		return ret;
	}

	read_temperature = ((reg_val * 100) / 107) - 55;
	pr_debug("Read die_temp : %d reg_val : 0x%02x\n",
			read_temperature, reg_val);

	return read_temperature;
}

static int idtp9017_get_align_axis(struct idtp9017_chip *chip,
		int sign, char axis) {
	u8 reg_val = 0x00;
	u8 sign_reg = 0x00;
	int ret = 0;
	unsigned int axis_value = 0;

	if (axis == 'x') {
		ret = idtp9017_read_reg(chip->client, RDST_36_L, &reg_val);
		if (ret < 0) {
			pr_err("Fail to Vout rdst_36_h reg\n");
			return ret;
		}
	} else if (axis == 'y') {
		ret = idtp9017_read_reg(chip->client, RDST_37_L, &reg_val);
		if (ret < 0) {
			pr_err("Fail to Vout rdst_37_h reg\n");
			return ret;
		}
	}

	sign_reg &= SIGN_BIT;
	if (sign_reg)
		sign = 1;	/* '1' minus */
	else
		sign = 0; 	/* '0' plus */

	reg_val &= ADC_7BIT;
	axis_value = (int)reg_val;

	pr_debug("%c axis, %s %d\n", axis, sign ? "-" : "+", axis_value);

	return axis_value;
}

/* Select select_fod_reg "1" or "2" */
static int idtp9017_get_fod_gain(struct idtp9017_chip * chip,
		int select_fod_reg) {
	u8 reg_val = 0x00;
	int ret = 0;
	long int fod_gain;

	if (select_fod_reg == 1) {
		ret = idtp9017_read_reg(chip->client, RDST_38_L, &reg_val);
		if (ret < 0) {
			pr_err("Fail to Vout rdst_33_h reg\n");
			return ret;
		}

		reg_val &= ADC_4BIT;
		fod_gain = (int)((reg_val * 78) + 8828);
	} else if (select_fod_reg == 2) {
		ret = idtp9017_read_reg(chip->client, RDST_37_L, &reg_val);
		if (ret < 0) {
			pr_err("Fail to Vout rdst_33_h reg\n");
			return ret;
		}

		reg_val &= ADC_5BIT;
		fod_gain = (int)((reg_val * 3904) - 58560);
	}

	return fod_gain;
}

static int idtp9017_set_fod_gain(struct idtp9017_chip * chip,
		int select_fod_reg, long int gain) {
	u8 reg_val = 0x00;
	u8 set_en = 0x00;
	int ret = 0;

	set_en = 1 << SHIFT_EN;

	pr_err("set enable : 0x%02x, reg_val : 0x%02x\n", set_en, reg_val);

	if (select_fod_reg == 1) {
		reg_val = (u8)((gain - 8828)/78);
		ret = idtp9017_masked_write(chip->client, REG_18_H,
				FOD1_EN, set_en);
		if (ret < 0) {
			pr_err("Fail to Vout rdst_33_h reg\n");
			return ret;
		}

		ret = idtp9017_masked_write(chip->client, REG_18_H,
				FOD1_VALUE, reg_val);
		if (ret < 0) {
			pr_err("Fail to Vout rdst_33_h reg\n");
			return ret;
		}
	} else if (select_fod_reg == 2) {
		reg_val = (u8)((gain + 58560) / 3904);
		ret = idtp9017_masked_write(chip->client, REG_18_L,
				FOD2_EN, set_en);
		if (ret < 0) {
			pr_err("Fail to Vout rdst_33_h reg\n");
			return ret;
		}

		ret = idtp9017_masked_write(chip->client, REG_18_L,
				FOD2_VALUE, reg_val);
		if (ret < 0) {
			pr_err("Fail to Vout rdst_33_h reg\n");
			return ret;
		}
	} else {
		pr_info("Not set FOD1 and FOD2 Gain\n");
	}

	return 0;
}

#define DEFAULT_CURRENT 1600
static int idtp9017_get_i_limit(struct idtp9017_chip *chip) {
	u8 reg_val = 0x00;
	int ret = 0;
	int read_i_limit = 0;
	int i;

	if (chip->set_limit_current_ma != 0) {
		ret = idtp9017_read_reg(chip->client, RDST_3A_L, &reg_val);
		if (ret < 0) {
			pr_err("Fail to Vout rdst_3a_l reg\n");
			return ret;
		}
		reg_val &= ADC_5BIT;

		for(i = ARRAY_SIZE(icl_ma_table) - 2; i >= 0; i--) {
			if (icl_ma_table[i].value == reg_val) {
				read_i_limit = icl_ma_table[i].icl_ma;
				pr_debug("I : %d, read_i : %d\n", i, read_i_limit);
				break;
			} else if (icl_ma_table[i].value < reg_val) {
				read_i_limit = icl_ma_table[i+1].icl_ma;
				pr_debug("I : %d, read_i : %d\n", i, read_i_limit);
				break;
			}
		}
	} else {
		read_i_limit = DEFAULT_CURRENT;
		pr_info("Not set default current(%d mA)\n", read_i_limit);
	}

	return read_i_limit;
}

static int idtp9017_get_target_voltage(struct idtp9017_chip *chip) {
	u8 reg_val = 0x00;
	int ret = 0;
	int read_voltage = 0;

	ret = idtp9017_read_reg(chip->client, RDST_3B_L, &reg_val);
	if (ret < 0) {
		pr_err("Fail to Vout rdst_3b_l reg\n");
		return ret;
	}

	reg_val &= ADC_5BIT;
	read_voltage = ((int)(reg_val & VSET_VALUE) * 100) + 4100;

	pr_debug("Target_voltage Read reg_val (0x%02x) and voltage(%d mV)\n",
			reg_val, read_voltage);

	return read_voltage;
}

static int idtp9017_enable_i_limit(struct idtp9017_chip *chip,
		bool enable) {
	u8 reg_val = 0x00;
	int ret = 0;

	reg_val = enable << 7;

	ret = idtp9017_masked_write(chip->client, REG_19_H,
			ILIM_EN, reg_val);
	if (ret < 0) {
		pr_err("Failed to set i_limit\n");
		return ret;
	}

	return 0;
}

static int idtp9017_set_i_limit(struct idtp9017_chip *chip,
		int set_current) {
	u8 reg_val = 0x00;
	int ret = 0;
	int i;

	set_current = 1000;
	for (i = ARRAY_SIZE(icl_ma_table) - 5; i >= 0; i--) {
		if (icl_ma_table[i].icl_ma == set_current) {
			reg_val = icl_ma_table[i].value;
			pr_debug("i : %d, table_ma : %d, table_value : 0x%02x\n",
					i, icl_ma_table[i].icl_ma, icl_ma_table[i].value);
			break;
		}
	}

	ret = idtp9017_masked_write(chip->client, REG_19_H,
			ILIM_VALUE, reg_val);
	if (ret < 0) {
		pr_err("Failed to set i_limit\n");
		return ret;
	}

	pr_debug("reg_val : 0x%02x, set current : %d\n",
			reg_val, set_current);

	return 0;
}

static int idtp9017_enable_out_voltage(struct idtp9017_chip *chip,
		bool enable) {
	u8 reg_val = 0x00;
	int ret = 0;

	reg_val = enable << 7;

	ret = idtp9017_masked_write(chip->client, REG_19_L,
			VSET_EN, reg_val);
	if (ret < 0) {
		pr_err("Failed to set i_limit\n");
		return ret;
	}
	pr_err("Set out_voltage rev_val : 0x%02x, set enable %d\n",
			reg_val, enable);

	return 0;
}

static int idtp9017_set_out_voltage(struct idtp9017_chip *chip,
		int voltage) {
	u8 reg_val = 0x00;
	int ret = 0;

	reg_val = (voltage - 4100) / 100;
	ret = idtp9017_masked_write(chip->client, REG_19_L,
			VSET_VALUE, reg_val);
	if (ret < 0) {
		pr_err("Failed to set out_voltage\n");
		return ret;
	}
	pr_err("Set out_voltage rev_val : 0x%02x, set voltage %d\n",
			reg_val, voltage);

	return 0;
}

static int die_shdn_control(struct idtp9017_chip *chip,
		int shdn_value, int hys_value) {
	u8 reg_val = 0x00;
	int ret = 0;

	reg_val = (u8)shdn_value << SHIFT_THRESHOLD_OFF;
	if (shdn_value != 0) {
		reg_val = (u8)hys_value << SHIFT_THRESHOLD_HYS;

		ret = idtp9017_masked_write(chip->client, REG_04_H,
				(TDIE_SHDN_OFF | TDIE_SHDN_HYS), reg_val);
		if (ret < 0) {
			pr_err("Failed to write shdn_off\n");
			return ret;
		}
	} else {
		ret = idtp9017_masked_write(chip->client, REG_04_H,
				TDIE_SHDN_OFF, reg_val);
		if (ret < 0) {
			pr_err("Failed to write shdn_off\n");
			return ret;
		}
	}

	return 0;
}

static int die_temp_control(struct idtp9017_chip *chip,
		int temp_value, int hys_value) {
	u8 reg_val = 0x00;
	int ret = 0;

	reg_val = (u8)temp_value << SHIFT_THRESHOLD_OFF;
	if (temp_value != 0) {
		reg_val = (u8)hys_value << SHIFT_THRESHOLD_HYS;

		ret = idtp9017_masked_write(chip->client, REG_04_L,
				(TDIE_THMR_OFF | TDIE_THMR_HYS), reg_val);
		if (ret < 0) {
			pr_err("Failed to write shdn_off\n");
			return ret;
		}
	} else {
		ret = idtp9017_masked_write(chip->client, REG_04_L,
				TDIE_THMR_OFF, reg_val);
		if (ret < 0) {
			pr_err("Failed to write shdn_off\n");
			return ret;
		}
	}

	return 0;
}

static int idtp9017_get_operate_freq(struct idtp9017_chip *chip) {
	u8 reg_val = 0x00;
	u16 reg_sum = 0x000;
	int ret = 0;
	int get_freq = 0;

	ret = idtp9017_read_reg(chip->client, REG_3F_H, &reg_val);
	if (ret < 0) {
		pr_err("Failed get freq\n");
		return ret;
	}

	reg_sum = (reg_val & ADC_4BIT) << SHIFT_FOR_ADC;

	ret = idtp9017_read_reg(chip->client, REG_3F_L, &reg_val);
	if (ret < 0) {
		pr_err("Failed get freq\n");
		return ret;
	}
	reg_sum |= reg_val;
	get_freq = (1 / (reg_sum * 3125)) * 10^6;

	pr_debug("reg_val : 0x%03x, get_freq : %d MHz\n",
			reg_sum, get_freq);

	return get_freq;
}

#define MAX_SET_CUR 16000
#define SET_CUR_STEP 100
#define MAX_SET_VOL 7000
#define SET_VOL_STEP 100
void idtp9017_align_start(void) {
	int ret = 0;
	int out_current = idtp9017_get_out_current(the_chip);
	int set_current = 0;
	int out_voltage = idtp9017_get_out_voltage(the_chip);
	int set_voltage = 0;

	if (out_current != the_chip->set_limit_current_ma) {
		ret = idtp9017_enable_i_limit(the_chip, true);
		if (ret < 0) {
			pr_err("Failed enable limit_currnet ret : %d\n", ret);
			return;
		}

		set_current = out_current + SET_CUR_STEP;
		if (set_current > MAX_SET_CUR)
			set_current = MAX_SET_CUR;

		ret = idtp9017_set_i_limit(the_chip, set_current);
		if (ret < 0) {
			pr_err("Failed set limit_currnet ret : %d\n", ret);
			return;
		}
	} else {
		pr_info("Complete the compansation of current\n");
	}

	if (out_voltage != the_chip->set_out_voltage) {
		ret = idtp9017_enable_out_voltage(the_chip, true);
		if (ret < 0) {
			pr_err("Failed enable out_voltage ret : %d\n", ret);
			return;
		}

		set_voltage = out_voltage + SET_VOL_STEP;
		if (set_voltage > MAX_SET_VOL)
			set_voltage = MAX_SET_VOL;

		ret = idtp9017_set_out_voltage(the_chip, set_voltage);
		if (ret < 0) {
			pr_err("Failed set out_voltage ret : %d\n", ret);
			return;
		}
	} else {
		pr_info("Complete the compansation of voltage\n");
	}
}

static void idtp9017_set_enviroment(struct work_struct *work) {
	struct delayed_work *dwork = to_delayed_work(work);
	struct idtp9017_chip *chip = container_of(dwork, struct idtp9017_chip,
			set_env_work);
	int ret = 0;
	static int delayed_counter = 0;

	if (chip->wlc_chg_en)
		delayed_counter++;

	if (chip->wlc_chg_en && delayed_counter >= 2) {
		if (chip->set_limit_current_ma != 0) {
			ret = idtp9017_enable_i_limit(chip, true);
			if (ret < 0) {
				pr_err("Failed enable limit_currnet\n");
				goto FAIL;
			}

			ret = idtp9017_set_i_limit(chip, chip->set_limit_current_ma);
			if (ret < 0) {
				pr_err("Failed set limit_currnet\n");
				goto FAIL;
			}
		} else {
			pr_err("Not set limit_current\n");
			ret = idtp9017_enable_i_limit(chip, chip->set_limit_current_ma);
			if (ret < 0) {
				pr_err("Failed enable limit_currnet\n");
				goto FAIL;
			}
		}

		if (chip->set_out_voltage != 0) {
			ret = idtp9017_enable_out_voltage(chip, true);
			if (ret < 0) {
				pr_err("Failed enable out_voltage\n");
				goto FAIL;
			}

			ret = idtp9017_set_out_voltage(chip, chip->set_out_voltage);
			if (ret < 0) {
				pr_err("Failed set out_voltage\n");
				goto FAIL;
			}
		} else {
			pr_err("Not set out_voltage\n");
			ret = idtp9017_enable_out_voltage(chip, false);
			if (ret < 0) {
				pr_err("Failed enable out_voltage\n");
				goto FAIL;
			}
		}

		if (chip->fod1_gain != 0) {
			ret = idtp9017_set_fod_gain(chip, 1, chip->fod1_gain);
			if (ret < 0) {
				pr_err("Failed set fod1\n");
				goto FAIL;
			}
		} else {
			pr_info ("Not set fod1_gain\n");
		}

		if (chip->fod2_gain != 0) {
			ret = idtp9017_set_fod_gain(chip, 2, chip->fod2_gain);
			if (ret < 0) {
				pr_err("Failed set fod1\n");
				goto FAIL;
			}
		} else {
			pr_info ("Not set fod2_gain\n");
		}

		if (chip->die_shdn_off != 0) {
			ret = die_shdn_control(chip, chip->die_shdn_off,
					chip->die_shdn_hys);
			if (ret < 0) {
				pr_err("Failed set die_shdn_off\n");
				goto FAIL;
			}
		}

		if (chip->die_temp_off) {
			ret = die_temp_control(chip, chip->die_temp_off,
					chip->die_temp_hys);
			if (ret < 0) {
				pr_err("Failed set die_temp_off\n");
				goto FAIL;
			}
		}

		pr_err("[IDTP9017] Complete to set enviroment\n");
	} else {
		if (chip->wlc_chg_en)
			pr_err("[IDTP9017] Waiting, not yet set enviroment\n");
		else
			delayed_counter = 0;

		schedule_delayed_work(&chip->set_env_work, round_jiffies_relative
				(msecs_to_jiffies(100*100)));
	}

	return;

FAIL:
	pr_err("Fail to set enviroment\n");

	schedule_delayed_work(&chip->set_env_work, round_jiffies_relative
			(msecs_to_jiffies(500)));

	return;
}

static void wlc_info_worker(struct work_struct *work) {
//	struct delayed_work *dwork = to_delayed_work(work);
	struct idtp9017_chip *chip = container_of(work, struct idtp9017_chip,
			wlc_status_work.work);
	int limit_cur_ma = 0; int out_cur_ma = 0;
	int out_vol_mv = 0;  int target_vol = 0;
	int rect_vol_mv = 0;
	int wpc_mode = 0;
	int die_temp = 0;
	int delay = 500 * 100;
	static int temp_counter;
	int fod1_gain = 0; int fod2_gain = 0;
	int wlc_status = 0;
	int temp_sign_x = 0;
	int temp_sign_y = 0;
	static int check_counter = 0;
	int op_freq = 0;
	int chg_done = 0;

	chip->wlc_chg_en = !(gpio_get_value(chip->wlc_active_n_gpio));

	if (chip->wlc_chg_en)
		check_counter++;
	else
		check_counter = 0;

	if (chip->wlc_chg_en && check_counter >= 2) {
		limit_cur_ma = idtp9017_get_i_limit(chip);
		out_cur_ma = idtp9017_get_out_current(chip);
		out_vol_mv = idtp9017_get_out_voltage(chip);
		target_vol = idtp9017_get_target_voltage(chip);
		die_temp = idtp9017_get_die_temperature(chip);
		rect_vol_mv = idtp9017_get_voltage_rect(chip);
		wpc_mode = idtp9017_get_chg_mode(chip);
		fod1_gain = idtp9017_get_fod_gain(chip, 1);
		fod2_gain = idtp9017_get_fod_gain(chip, 2);
		chip->x_axis = idtp9017_get_align_axis(chip, temp_sign_x, 'x');
		chip->y_axis = idtp9017_get_align_axis(chip, temp_sign_y, 'y');
		wlc_status = idtp9017_wlc_status(chip);
		op_freq = idtp9017_get_operate_freq(chip);
		chg_done = idtp9017_wlc_status(chip);
		if (chg_done == 2)
			chg_done = 1;
		else
			chg_done = 0;

		pr_info("[idtp9017] chg_en : %s, chg_mode : %s, chg_done : %s\n",
				wpc_mode ? "wpc_mode" : "pma_mode",
				wlc_status ? "enable" : "disable",
				chg_done ? "Done" : "Not yet");
		pr_info("[idtp9017] Op_freq : %d MHz, Limit_cur : %d mA, Out_cur %dmA\n",
				op_freq, limit_cur_ma, out_cur_ma);
		pr_info("[idtp9017] target_vol : %dmV, Out_vol : %dmV, Vrect : %dmV\n",
				target_vol, out_vol_mv, rect_vol_mv);
		pr_info("[idtp9017] Die_temperature : %d, Axis(%s%d, %s%d)\n", die_temp,
				temp_sign_x ? "-" : "+", chip->x_axis,
				temp_sign_y ? "-" : "+", chip->y_axis);
		pr_info("[idtp9017] Fod1_gain : %d.%d%%, Fod2_gain : %d.%dmW\n",
				fod1_gain/100, fod1_gain%100, fod2_gain/100, fod2_gain%100);
		temp_counter = 0;
		check_counter = 3;
		if (chg_done)
			delay = delay * 20;
		else
			delay = delay * 10;
	} else {
		delay /= 5;
		if (temp_counter <= 2) {
			pr_info("[WLC_STATUS] Not Connected WLC\n");
			temp_counter = 3;
		}
		temp_counter++;
	}

	schedule_delayed_work(&chip->wlc_status_work, round_jiffies_relative
			(msecs_to_jiffies(delay)));
}

static int idtp9017_parse_dt(struct device_node *dev_node,
		struct idtp9017_chip *chip) {
	int ret = -1;

	chip->wlc_active_n_gpio = of_get_named_gpio(dev_node,
			"idt,wlc_active_n_gpio", 0);
	if (chip->wlc_active_n_gpio < 0) {
		pr_err("Fail to get wlc_active_n_gpio\n");
		goto out;
	} else {
		pr_info("Get wlc_active_n_gpio : %d\n",
				chip->wlc_active_n_gpio);
	}

	chip->wlc_full_chg_gpio = of_get_named_gpio(dev_node,
			"idt,wlc_full_chg_gpio", 0);
	if (chip->wlc_full_chg_gpio < 0) {
		pr_err("Fail to get wlc_full_chg_gpio\n");
		goto out;
	} else {
		pr_info("Get wlc_full_chg_gpio : %d\n",
				chip->wlc_full_chg_gpio);
	}

	chip->wlc_off_gpio = of_get_named_gpio(dev_node,
			"idt,wlc_off_gpio", 0);
	if (chip->wlc_off_gpio < 0) {
		pr_err("Fail to get wlc_off_gpio\n");
		goto out;
	} else {
		pr_info("Get wlc_off_gpio : %d\n",
				chip->wlc_off_gpio);
	}

	ret = of_property_read_u32(dev_node,
			"idt,mode_depth", &chip->mode_depth);
	if (ret) {
		pr_err("Not exist die_shdn_off paramaeter\n");
		chip->mode_depth = 0;
	} else {
		pr_info("Get mode_depth : %d\n",
				chip->mode_depth);
	}

	ret = of_property_read_u32(dev_node,
			"idt,fod1-gain", &chip->fod1_gain);
	if (ret) {
		pr_err("Not exist die_shdn_off paramaeter\n");
		chip->fod1_gain = 0;
	} else {
		pr_info("Get fod1_gain : %d\n",
				chip->fod1_gain);
	}

	ret = of_property_read_u32(dev_node,
			"idt,fod2-gain", &chip->fod2_gain);
	if (ret) {
		pr_err("Not exist die_shdn_off paramaeter\n");
		chip->fod2_gain = 0;
	} else {
		pr_info("Get fod2_gain : %d\n",
				chip->fod2_gain);
	}

	ret = of_property_read_u32(dev_node,
			"idt,die-shdn-off", &chip->die_shdn_off);
	if (ret) {
		pr_err("Not exist die_shdn_off paramaeter\n");
		chip->die_shdn_off = 0;
	} else {
		pr_info("Get die_shdn_off : %d\n",
				chip->die_shdn_off);
	}

	ret = of_property_read_u32(dev_node,
			"idt,die-shdn-hys", &chip->die_shdn_hys);
	if (ret) {
		pr_err("Not exist die_shdn_hys paramaeter\n");
		chip->die_shdn_hys = 0;
	} else {
		pr_info("Get shdn_hys : %d\n",
				chip->die_shdn_hys);
	}

	ret = of_property_read_u32(dev_node,
			"idt,die-temp-off", &chip->die_temp_off);
	if (ret) {
		pr_err("Not exist tmep_off paramaeter\n");
		chip->die_temp_off = 0;
	} else {
		pr_info("Get temp_off : %d\n",
				chip->die_temp_off);
	}

	ret = of_property_read_u32(dev_node,
			"idt,die-temp-hys", &chip->die_temp_hys);
	if (ret) {
		pr_err("Not exist temp_hys paramaeter\n");
		chip->die_temp_hys = 0;
	} else {
		pr_info("Get temp_hys : %d\n",
				chip->die_temp_hys);
	}

	ret = of_property_read_u32(dev_node,
			"idt,limit-current", &chip->set_limit_current_ma);
	if (ret) {
		pr_err("Not exist temp_hys paramaeter\n");
		chip->set_limit_current_ma = 0;
	} else {
		pr_info("Get limit_current : %d\n",
				chip->set_limit_current_ma);
	}

	ret = of_property_read_u32(dev_node,
			"idt,out-voltage", &chip->set_out_voltage);
	if (ret) {
		pr_err("Not exist temp_hys paramaeter\n");
		chip->set_out_voltage = 0;
	} else {
		pr_info("Get out_voltage : %d\n",
				chip->set_out_voltage);
	}

	return 0;

out:
	return ret;
}

static int idtp9017_set_gpio(struct idtp9017_chip *chip) {
	int ret = 0;
	int check_gpio = 0;

	check_gpio = check_gpio_status(chip->wlc_active_n_gpio);
	if (check_gpio < 0) {
		ret = gpio_request_one(chip->wlc_active_n_gpio, GPIOF_DIR_IN,
				"wlc_active_n_gpio");
		if (ret < 0) {
			pr_err("Fail to request wlc_active_n_gpio\n");
			return ret;
		}
	} else {
		pr_info("Already set wlc_active_n_gpio\n");
	}

	check_gpio = check_gpio_status(chip->wlc_full_chg_gpio);
	if (check_gpio < 0) {
		ret = gpio_request_one(chip->wlc_full_chg_gpio, GPIOF_DIR_OUT,
				"wlc_full_chg_gpio");
		if (ret < 0) {
			pr_err("Fail to request wlc_full_chg_gpio\n");
			return ret;
		}
	} else {
		pr_info("Already set wlc_full_chg_gpio\n");
	}

	check_gpio = check_gpio_status(chip->wlc_off_gpio);
	if (check_gpio < 0) {
		ret = gpio_request_one(chip->wlc_off_gpio, GPIOF_DIR_OUT,
				"wlc_off_gpio");
		if (ret < 0) {
			pr_err("Fail to request wlc_off_gpio\n");
			return ret;
		}
	} else {
		pr_info("Already set wlc_off_gpio\n");
	}

	return 0;
}

static int idtp9017_probe(struct i2c_client *client,
						 const struct i2c_device_id *id)
{
	struct idtp9017_chip *chip;
	struct device_node *dev_node = client->dev.of_node;
	int ret;

	pr_info("[IDTP9017] Start\n");

	if (!i2c_check_functionality(client->adapter,
			I2C_FUNC_SMBUS_BYTE_DATA)) {
		pr_err("i2c func fail.\n");
		return -EIO;
	}

	chip = kzalloc(sizeof(struct idtp9017_chip), GFP_KERNEL);
	if (!chip) {
		pr_err("Failed to alloc memory\n");
		goto error;
	}

	chip->client = client;

	/* need dts parser */
	if (dev_node) {
		ret = idtp9017_parse_dt(dev_node, chip);
		if (ret < 0) {
			pr_err("Fail to read parse_dt\n");
			goto err_hw_init;
		}
	}

	ret = idtp9017_set_gpio(chip);
	if (ret < 0) {
		pr_err("Fail to request gpio at probe\n");
		goto err_hw_init;
	}

	i2c_set_clientdata(client, NULL);

	the_chip = chip;

	INIT_DELAYED_WORK(&chip->wlc_status_work, wlc_info_worker);
	schedule_delayed_work(&chip->wlc_status_work, round_jiffies_relative
			(msecs_to_jiffies(500)));
	INIT_DELAYED_WORK(&chip->set_env_work, idtp9017_set_enviroment);
	schedule_delayed_work(&chip->set_env_work, round_jiffies_relative
			(msecs_to_jiffies(500)));
#ifdef CONFIG_LGE_PM_CHARGING_USING_CHGSTS_WLC
	INIT_DELAYED_WORK(&chip->update_soc_worker, update_wlc_worker);
	schedule_delayed_work(&chip->update_soc_worker, round_jiffies_relative
			(msecs_to_jiffies(200)));
#endif

	pr_info("[IDTP9017] Complete IDTP9017 probe\n");

	return 0;

err_hw_init:
	if (chip->wlc_active_n_gpio)
		gpio_free(chip->wlc_active_n_gpio);
	if (chip->wlc_full_chg_gpio)
		gpio_free(chip->wlc_full_chg_gpio);
	if (chip->wlc_off_gpio)
		gpio_free(chip->wlc_off_gpio);
error:
	kfree(chip);
	pr_err("[IDTP9017] Failed to probe\n");

	return ret;
}

static void  idtp9017_resume(struct device *dev) {
//	struct i2c_client *client = to_i2c_client(dev);
//	struct idtp9017_chip *chip = i2c_get_clientdata(client);
	pr_info("[WLC] wlc_resume\n");

	the_chip->suspend = false;

	schedule_delayed_work(&the_chip->wlc_status_work, round_jiffies_relative(
				msecs_to_jiffies(500)));
#ifdef CONFIG_LGE_PM_CHARGING_USING_CHGSTS_WLC
	schedule_delayed_work(&chip->update_soc_worker, round_jiffies_relative
			(msecs_to_jiffies(500)));
#endif
}

static int idtp9017_suspend(struct device *dev) {
//	struct i2c_client *client = to_i2c_client(dev);
//	struct idtp9017_chip *chip = i2c_get_clientdata(client);
	pr_info("[WLC] wlc_suspend\n");

	the_chip->suspend = true;

	cancel_delayed_work_sync(&the_chip->wlc_status_work);

#ifdef CONFIG_LGE_PM_CHARGING_USING_CHGSTS_WLC
	cancel_delayed_work_sync(&chip->update_soc_worker);
#endif
	return 0;
}

static const struct dev_pm_ops idtp9017_pm_ops = {
	.prepare 	= idtp9017_suspend,
	.complete 	= idtp9017_resume,

};

static int idtp9017_remove(struct i2c_client *client) {
	struct idtp9017_chip *chip = i2c_get_clientdata(client);

	if (chip->wlc_active_n_gpio)
		gpio_free(chip->wlc_active_n_gpio);
	if (chip->wlc_full_chg_gpio)
		gpio_free(chip->wlc_full_chg_gpio);
	if (chip->wlc_off_gpio)
		gpio_free(chip->wlc_off_gpio);

	cancel_delayed_work_sync(&chip->wlc_status_work);

#ifdef CONFIG_LGE_PM_CHARGING_USING_CHGSTS_WLC
	cancel_delayed_work_sync(&chip->update_soc_worker);
#endif

	kfree(chip);
	chip = NULL;
	the_chip = NULL;

	return 0;
}

static struct of_device_id idt_idtp9017_table[] = {
		{ .compatible = "idt,idtp9017", },
		{},
};

static const struct i2c_device_id idtp9017_id[] = {
	{IDTP9017_NAME, 0},
	{}
};

static struct i2c_driver idtp9017_driver = {
	.driver = {
		.name  	= IDTP9017_NAME,
		.owner 	= THIS_MODULE,
		.of_match_table = idt_idtp9017_table,
		.pm 	= &idtp9017_pm_ops,
	},
	.probe  	= idtp9017_probe,
	.id_table  	= idtp9017_id,
//	.resume  	= idtp9017_resume,
//	.suspend  	= idtp9017_suspend,
	.remove  	= idtp9017_remove,
};

static int __init idtp9017_init(void) {
	return i2c_add_driver(&idtp9017_driver);
}
module_init(idtp9017_init);

static void __exit idtp9017_exit(void) {
	return i2c_del_driver(&idtp9017_driver);
}
module_exit(idtp9017_exit);

MODULE_DESCRIPTION("idtp9017");
MODULE_LICENSE("GPL V2");
