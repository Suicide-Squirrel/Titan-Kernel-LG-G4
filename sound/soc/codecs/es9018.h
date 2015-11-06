#ifndef __ES9018_H__
#define __ES9018_H__

#define ESS9018_SYSTEM_SET		0
#define ESS9018_IN_CONFIG		1
#define ESS9018_02				2
#define ESS9018_03				3
#define ESS9018_SVOLCNTL1		4
#define ESS9018_SVOLCNTL2		5
#define ESS9018_SVOLCNTL3		6
#define ESS9018_SETTING		7
#define ESS9018_GPIOCFG		8
#define ESS9018_09				9
#define ESS9018_MASTERMODE	10
#define ESS9018_CHANNELMAP	11
#define ESS9018_DPLLASRC		12
#define ESS9018_THD_COMP		13
#define ESS9018_SOFT_START		14
#define ESS9018_VOL1			15
#define ESS9018_VOL2			16
#define ESS9018_17				17
#define ESS9018_18				18
#define ESS9018_19				19
#define ESS9018_MASTERTRIM		20
#define ESS9018_GPIO_INSEL		21
#define ESS9018_2ND_HCOMP1	22
#define ESS9018_2ND_HCOMP2	23
#define ESS9018_3RD_HCOMP1	24
#define ESS9018_3RD_HCOMP2	25
#define ESS9018_FILTER_ADDR	26
#define ESS9018_FILTER_COEF	29
#define ESS9018_FILTER_CONT	30
#define ESS9018_CHIPSTATUS		64
#define ESS9018_65				65
#define ESS9018_DPLLRATIO1		66
#define ESS9018_DPLLRATIO2		67
#define ESS9018_DPLLRATIO3		68
#define ESS9018_DPLLRATIO4		69

#define ES9018_I2C_LEN_MASK	0xc0
#define ES9018_SOFT_START_MASK 0x80

#define INPUT_CONFIG_SOURCE 1
#define I2S_BIT_FORMAT_MASK (0x03 << 6)
#define MASTER_MODE_CONTROL 10
#define I2S_CLK_DIVID_MASK (0x03 << 5)
#define RIGHT_CHANNEL_VOLUME_15 15
#define LEFT_CHANNEL_VOLUME_16 16
#define MASTER_TRIM_VOLUME_17 17
#define MASTER_TRIM_VOLUME_18 18
#define MASTER_TRIM_VOLUME_19 19
#define MASTER_TRIM_VOLUME_20 20
#define HEADPHONE_AMPLIFIER_CONTROL 42

enum {
	ESS_PS_CLOSE,
	ESS_PS_OPEN,
	ESS_PS_BYPASS,
	ESS_PS_HIFI,
	ESS_PS_IDLE,
	ESS_PS_ACTIVE,
};

struct es9018_priv {
	struct snd_soc_codec *codec;
	struct i2c_client *i2c_client;
	struct es9018_data *es9018_data;
	struct delayed_work sleep_work;
	struct mutex power_lock;
} es9018_priv;


struct es9018_data {
	int reset_gpio;
	int power_gpio;
	int hph_switch;
#ifdef DEDICATED_I2C
	int i2c_scl_gpio;
	int i2c_sda_gpio;
	int i2c_addr;
#endif
};

#endif //__ES9018_H__

