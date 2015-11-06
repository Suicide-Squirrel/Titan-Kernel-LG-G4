/*
 * i2c-im401.c  --  IM401 Smart DMIC bridge driver
 *
 * Copyright 2014 Fortemeida Inc.
 * Author: Henry Zhang <henryhzhang@fortemedia.com.cn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

/* #define DEBUG */
/* #define VERBOSE_DEBUG */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/firmware.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/debugfs.h>
#include <linux/slab.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/uaccess.h>
//for fw download time
#include <linux/timer.h>
#include <linux/timex.h>
#include <linux/rtc.h>
//end for fw download time
#include <linux/err.h>
#include <linux/wakelock.h>
#include <linux/spinlock.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <linux/of_gpio.h>
#include <linux/async.h>

/* for Device Tree */
#include <linux/io.h>
#include <linux/of.h>

#include <linux/delay.h>
#include <linux/input.h>
#include <linux/platform_data/i2c-im401.h>
#include <linux/mfd/pm8xxx/pm8xxx-adc.h>
#include <linux/qpnp/qpnp-adc.h>

#include <linux/clk.h>

#define FEATURE_LG_G3_TEST_ONLY // Korea_20150306_add for LG G3 test
#define LG_DEBUG

#define DRIVER_DESC    "IM401 Smart DMIC bridge driver"
#define DRIVER_NAME    "i2c-im401"
#define DRIVER_VERSION "0.0.2"
#define DRIVER_AUTHOR  "Fortemedia"

//#define IM401_IRQ
#define MAX14688_JACK_IN_VALUE           1
#define MAX14688_BUTTON_DOWN_VALUE       1
#define MAX14688_BUTTON_UP_VALUE         0

#define JACK_IN_VALUE      MAX14688_JACK_IN_VALUE
#define BUTTON_DOWN_VALUE  MAX14688_BUTTON_DOWN_VALUE
#define BUTTON_UP_VALUE    MAX14688_BUTTON_UP_VALUE

/* woohyun.seok, enable for RT clk */
//#define IM401_RT_CLK_ENABLE
#ifdef IM401_RT_CLK_ENABLE
struct clk *fm_clk;
#endif

static struct i2c_client *im401_i2c;
static struct IM401_gpio *im401_gpio_g;

static const struct firmware *im401_fw = NULL;
static const struct firmware *im401_grammar_fw = NULL;
static struct delayed_work reload_work;
static struct delayed_work load_grammar_work;
static struct delayed_work dsp_start_bypass;
static struct delayed_work interrupt_work;

//Henry Jan.23,2015 for interrupt handler.
struct work_struct 	im401_irq_work;

static bool im401_start_rest = false;
int im401_private_fw_exist = 0;
#ifdef SHOW_DL_TIME
struct timex  txc;
struct rtc_time tm;
#endif

u32 im401_irq;
static int im401_host_irqstatus = 0;    // To notify the HAL about the incoming irq.

struct wake_lock im401_wake_lock;

static int im401_write(struct i2c_client *i2c, unsigned int reg,
    unsigned int value)
{
    u8 data[3];
    int ret;

    data[0] = reg & 0xff;
    data[1] = (0xff00 & value) >> 8;
    data[2] = value & 0xff;

    ret = i2c_master_send(i2c, data, 3);
    if (ret == 3)
        return 0;
    if (ret < 0){
		printk("Error: im401_write: ret = %x\n", ret);
        return ret;
    }
    else {
		printk("Error: im401_write: ret = %x\n", ret);
        return -EIO;
    }
}

static int im401_read(struct i2c_client *i2c, unsigned int r)
{
    struct i2c_msg xfer[2];
    u8 reg[1];
    u8 data[2];
    int value = 0x0;
    int ret;

    /* Write register */
    reg[0] = r & 0xff;
    xfer[0].addr = i2c->addr;
    xfer[0].flags = 0;
    xfer[0].len = 1;
    xfer[0].buf = &reg[0];

    /* Read data */
    xfer[1].addr = i2c->addr;
    xfer[1].flags = I2C_M_RD;
    xfer[1].len = 2;
    xfer[1].buf = data;

    ret = i2c_transfer(i2c->adapter, xfer, 2);
    if (ret != 2) {
		printk("Error: im401_read: ret = %x\n", ret);
        return -EIO;
    }

    value = (data[0] << 8) | data[1];

    return value;
}

static int im401_dsp_read(struct i2c_client *i2c, unsigned int addr)
{
	unsigned int ret_value;
	unsigned short l_value;
	unsigned short h_value;

	im401_write(i2c, IM401_DSP_MEM_CTRL1, addr & 0xffff);
	im401_write(i2c, IM401_DSP_MEM_CTRL2, (addr & 0xffff0000) >> 16);
	im401_write(i2c, IM401_DSP_MEM_CTRL5, 0x2);

	l_value = im401_read(i2c, IM401_DSP_CTRL3);
	printk("im401_dsp_read: l_value = 0x%x\n", l_value);

	h_value = im401_read(i2c, IM401_DSP_CTRL4);
	printk("im401_dsp_read: h_value = 0x%x\n", h_value);

	ret_value = h_value;
	ret_value = (ret_value << 16) | l_value;

	return ret_value;
}

static int im401_dsp_burst_write(struct i2c_client *i2c,
    unsigned int addr, const u8 *buf, size_t size)
{
    u8 data[5];
    unsigned int pos = 0;
    int ret;

	im401_write(i2c, IM401_DSP_MEM_CTRL1, addr & 0xffff);
    im401_write(i2c, IM401_DSP_MEM_CTRL2, (addr & 0xffff0000) >> 16);

    while (size > pos) {
        data[0] = IM401_DSP_MEM_CTRL7;
        data[1] = buf[pos + 1];
        data[2] = buf[pos + 0];
        data[3] = buf[pos + 3];
        data[4] = buf[pos + 2];

        ret = i2c_master_send(i2c, data, ARRAY_SIZE(data));
        if (ret < 0) {
			printk("Error: im401_dsp_burst_write: ret = %x\n", ret);
            return ret;
        }

        pos += 4;
    }

    return 0;
}

void im401_dsp_one_shot(void) {
	printk("im401: im401_dsp_one_shot......................\n");
	/* M1/N1/P1/Q1 Option */
	im401_write(im401_i2c, IM401_PLL_CLOCK_CTRL3, 0x0704);
	im401_write(im401_i2c, IM401_UPFILTER_CTRL1, 0x6aa0);
	im401_write(im401_i2c, IM401_UPFILTER_CTRL1, 0x6ea0);
	im401_write(im401_i2c, IM401_PWR_DIG, 0x0003);
	im401_write(im401_i2c, IM401_DMIC_DATA_CTRL, 0x0091);
}
EXPORT_SYMBOL_GPL(im401_dsp_one_shot);

void im401_enable_dsp_clock(void) {
    printk("im401: im401_enable_dsp_clock......................\n");
    im401_write(im401_i2c, IM401_AUTO_MODE_CTRL, 0x00c0);
    im401_write(im401_i2c, IM401_PWR_ANLG1, 0x8ba1);
    im401_write(im401_i2c, IM401_ANA_CIRCUIT_CTRL_LDO2, 0xa342);
    im401_write(im401_i2c, IM401_PWR_DSP, 0x0005);
    im401_write(im401_i2c, IM401_DIG_PAD_CTRL2, 0x0400);
    im401_write(im401_i2c, IM401_DUMMY_RTK3, 0x6395);
}

void im401_set_clock_cal_pll(int iOption) {
	// Henry Zhang choose P1 Option to slow down PLL_CLK_EXT Apr.17, 2015
	printk("im401: im401_set_clock_cal_pll......................\n");
	im401_write(im401_i2c, IM401_PLL_CAL_CTRL8, 0x0022);
	im401_write(im401_i2c, IM401_PLL_CLK_EXT_CTRL1, 0x0040);
	/* M1 Option */
	im401_write(im401_i2c, IM401_PLL_CLK_EXT_CTRL2, 0x2202);
	im401_write(im401_i2c, IM401_PLL_CLOCK_CTRL1, 0x0e06);
	im401_write(im401_i2c, IM401_PLL_CLOCK_CTRL2, 0x00d3);
	im401_write(im401_i2c, IM401_PLL_CLOCK_CTRL3, 0x0704);
	im401_write(im401_i2c, IM401_BUF_MODE_CTRL_PLL_CAL1, 0x8040);
	im401_write(im401_i2c, IM401_BUF_MODE_CTRL_PLL_CAL2, 0x0200);
	im401_write(im401_i2c, IM401_PLL_CAL_CTRL1, 0x0000);
	im401_write(im401_i2c, IM401_PLL_CAL_CTRL2, 0x01f4);
	im401_write(im401_i2c, IM401_PLL_CAL_CTRL8, 0x8022);
	im401_write(im401_i2c, IM401_PLL_CAL_CTRL8, 0xc022);
}

void im401_dsp_start_first(void) {
	im401_write(im401_i2c, IM401_PWR_ANLG1, 0x8ba1);
	im401_write(im401_i2c, IM401_AUTO_MODE_CTRL, 0x00c0);
	im401_write(im401_i2c, IM401_DSP_CTRL1, 0x0000);
	im401_write(im401_i2c, IM401_GPIO_CTRL1, 0xa800);
	im401_write(im401_i2c, IM401_PWR_DSP, 0x0005);
	im401_write(im401_i2c, IM401_PWR_DSP, 0x0007);
	im401_write(im401_i2c, IM401_PWR_DSP, 0x0005);
	im401_write(im401_i2c, IM401_PWR_DSP, 0x0004);
	im401_write(im401_i2c, IM401_BUF_SRAM_CTRL7, 0x8000);
	im401_write(im401_i2c, IM401_AD_DIG_FILTER_CTRL2, 0x0022);
	im401_write(im401_i2c, IM401_PWR_DSP, 0x0001);
	im401_write(im401_i2c, IM401_PWR_DSP, 0x0019);
	im401_write(im401_i2c, IM401_PWR_DSP, 0x0018);
	im401_write(im401_i2c, IM401_PWR_DIG, 0x0001);
	im401_write(im401_i2c, IM401_BUF_SRAM_CTRL6, 0x003a);
	im401_write(im401_i2c, IM401_BUF_SRAM_CTRL7, 0x0000);
	im401_write(im401_i2c, IM401_VAD_CTRL1, 0x11c0);
	im401_write(im401_i2c, IM401_VAD_STATUS1, 0x8000);
	im401_write(im401_i2c, IM401_AD_DIG_FILTER_CTRL1, 0x8a2f);	//gain = 24dB
	im401_write(im401_i2c, IM401_VAD_CTRL2, 0x0039);		//VAD sensitivity
	im401_write(im401_i2c, IM401_VAD_CTRL3, 0xff01);		//VAD sensitivity
	im401_write(im401_i2c, IM401_VAD_CTRL4, 0x7c0a);
	im401_write(im401_i2c, IM401_VAD_STATUS1, 0x0000);
	im401_write(im401_i2c, IM401_DMIC_DATA_CTRL, 0x0090);
	im401_write(im401_i2c, IM401_PLL_CAL_CTRL8, 0x8022);
	im401_write(im401_i2c, IM401_PLL_CAL_CTRL8, 0xc022);
}

void im401_dsp_start_rest(int iOption) {
	printk("im401: im401_dsp_start_reset iOption(%d)\n",iOption);
	im401_write(im401_i2c, IM401_PWR_ANLG1, 0x8B81);
	im401_write(im401_i2c, IM401_AUTO_MODE_CTRL, 0x00c0);
	im401_write(im401_i2c, IM401_DSP_CTRL1, 0x0000);
	im401_write(im401_i2c, IM401_PWR_DSP, 0x0005);
	im401_write(im401_i2c, IM401_PWR_DSP, 0x0004);
	im401_write(im401_i2c, IM401_BUF_SRAM_CTRL7, 0x8000);
	im401_write(im401_i2c, IM401_PLL_CLOCK_CTRL3, 0x0704);
	/* M1 Option */
	im401_write(im401_i2c, IM401_PLL_CLK_EXT_CTRL2, 0x2202);
	im401_write(im401_i2c, IM401_PWR_DSP, 0x0001);
	im401_write(im401_i2c, IM401_PWR_DSP, 0x0019);
	im401_write(im401_i2c, IM401_PWR_DSP, 0x0018);
	im401_write(im401_i2c, IM401_PWR_DIG, 0x0001);
	im401_write(im401_i2c, IM401_BUF_SRAM_CTRL6, 0x003a);
	im401_write(im401_i2c, IM401_BUF_SRAM_CTRL7, 0x0000);
	im401_write(im401_i2c, IM401_VAD_CTRL1, 0x11c0);
	im401_write(im401_i2c, IM401_VAD_STATUS1, 0x8000);
	im401_write(im401_i2c, IM401_VAD_STATUS1, 0x0000);
	im401_write(im401_i2c, IM401_DMIC_DATA_CTRL, 0x0090);
	im401_write(im401_i2c, IM401_AD_DIG_FILTER_CTRL1, 0x8a2f);	//gain = 24dB
	im401_write(im401_i2c, IM401_PLL_CAL_CTRL8, 0x8022);
	im401_write(im401_i2c, IM401_PLL_CAL_CTRL8, 0xc022);
}

void im401_dsp_start(void) {
	unsigned int ret_value;
	unsigned int addr;

	//printk("IM401: im401_dsp_start\n");
	im401_set_clock_cal_pll(0);

	if (im401_start_rest) {
		im401_dsp_start_rest(0);
	} else {
		im401_dsp_start_first();
		im401_start_rest = true;
	}

	addr = 0x0ffe37d0;
	ret_value = im401_dsp_read(im401_i2c, addr);
	printk("im401: TDHeader runtime addr=0x%x, value=0x%x\n", addr, ret_value);
	ret_value = im401_dsp_read(im401_i2c, addr+4);
	printk("im401: TDHeader runtime addr=0x%x, value=0x%x\n", addr+4, ret_value);
	ret_value = im401_dsp_read(im401_i2c, addr+8);
	printk("im401: TDHeader runtime addr=0x%x, value=0x%x\n", addr+8, ret_value);
	ret_value = im401_dsp_read(im401_i2c, addr+12);
	printk("im401: TDHeader runtime addr=0x%x, value=0x%x\n", addr+12, ret_value);
	ret_value = im401_dsp_read(im401_i2c, addr+16);
	printk("im401: TDHeader runtime addr=0x%x, value=0x%x\n", addr+16, ret_value);
	ret_value = im401_dsp_read(im401_i2c, addr+20);
	printk("im401: TDHeader runtime addr=0x%x, value=0x%x\n", addr+20, ret_value);
}
EXPORT_SYMBOL_GPL(im401_dsp_start);

void im401_dsp_stop(void) {
	im401_write(im401_i2c, IM401_VAD_STATUS1, 0x8000);
	im401_write(im401_i2c, IM401_PWR_DSP, 0x0019);
	im401_write(im401_i2c, IM401_AUTO_MODE_CTRL, 0x03c1);
	im401_write(im401_i2c, IM401_PLL_CLK_EXT_CTRL2, 0x2008);
	im401_write(im401_i2c, IM401_DMIC_DATA_CTRL, 0x0092);
	im401_write(im401_i2c, IM401_PWR_DIG, 0x0000);
	im401_write(im401_i2c, IM401_VAD_CTRL1, 0x1040);
	im401_write(im401_i2c, IM401_BUF_SRAM_CTRL6, 0x002a);
	im401_write(im401_i2c, IM401_PWR_ANLG1, 0x88a0);
}
EXPORT_SYMBOL_GPL(im401_dsp_stop);

static size_t im401_read_file(char *file_path, const u8 **buf)
{
    loff_t pos = 0;
    unsigned int file_size = 0;
    struct file *fp;

    printk("im401: im401_read_file\n");

    fp = filp_open(file_path, O_RDONLY, 0);
    if (!IS_ERR(fp)) {
        file_size = vfs_llseek(fp, pos, SEEK_END);

        *buf = kzalloc(file_size, GFP_KERNEL);
        if (*buf == NULL) {
            filp_close(fp, 0);
            return 0;
        }

        kernel_read(fp, pos, (char *)*buf, file_size);
        filp_close(fp, 0);

        return file_size;
    }

    return 0;
}

unsigned int im401_4byte_le_to_uint(const u8 *data)
{
    return data[0] | data[1] << 8 | data[2] << 16 | data[3] << 24;
}

void im401_parse_header(const u8 *buf)
{
    SMicFWHeader sMicFWHeader;
    SMicFWSubHeader sMicFWSubHeader;

    int i, offset = 0;
    const u8 *data;
    char file_path[32];
    size_t size;

    sMicFWHeader.Sync = im401_4byte_le_to_uint(buf);
    printk("im401: sMicFWHeader.Sync = %08x\n", sMicFWHeader.Sync);

    offset += 4;
    sMicFWHeader.Version =  im401_4byte_le_to_uint(buf + offset);
    printk("im401: sMicFWHeader.Version = %08x\n", sMicFWHeader.Version);

    offset += 4;
    sMicFWHeader.NumBin = im401_4byte_le_to_uint(buf + offset);
    printk("im401: sMicFWHeader.NumBin = %08x\n", sMicFWHeader.NumBin);

    sMicFWHeader.BinArray = kzalloc(sizeof(SMicBinInfo) * sMicFWHeader.NumBin, GFP_KERNEL);

    for (i = 0 ; i < sMicFWHeader.NumBin; i++) {
        offset += 4;
        sMicFWHeader.BinArray[i].Offset = im401_4byte_le_to_uint(buf + offset);
        printk("im401: sMicFWHeader.BinArray[%d].Offset = %08x\n", i, sMicFWHeader.BinArray[i].Offset);

        offset += 4;
        sMicFWHeader.BinArray[i].Size = im401_4byte_le_to_uint(buf + offset);
        printk("im401: sMicFWHeader.BinArray[%d].Size = %08x\n", i, sMicFWHeader.BinArray[i].Size);

        offset += 4;
        sMicFWHeader.BinArray[i].Addr = im401_4byte_le_to_uint(buf + offset);
        printk("im401: sMicFWHeader.BinArray[%d].Addr = %08x\n", i, sMicFWHeader.BinArray[i].Addr);

        im401_dsp_burst_write(im401_i2c, sMicFWHeader.BinArray[i].Addr,
            buf + sMicFWHeader.BinArray[i].Offset, sMicFWHeader.BinArray[i].Size);
    }

    offset += 4;
    sMicFWSubHeader.NumTD = im401_4byte_le_to_uint(buf + offset);
    //printk("im401: sMicFWSubHeader.NumTD = %08x\n", sMicFWSubHeader.NumTD);

    sMicFWSubHeader.TDArray = 
        kzalloc(sizeof(SMicTDInfo) * sMicFWSubHeader.NumTD, GFP_KERNEL);

    for (i = 0 ; i < sMicFWSubHeader.NumTD; i++) {
        offset += 4;
        sMicFWSubHeader.TDArray[i].ID = im401_4byte_le_to_uint(buf + offset);
        printk("im401: sMicFWSubHeader.TDArray[%d].ID = %08x\n", i, sMicFWSubHeader.TDArray[i].ID);

        offset += 4;
        sMicFWSubHeader.TDArray[i].Addr = im401_4byte_le_to_uint(buf + offset);
        printk("im401: sMicFWSubHeader.TDArray[%d].Addr = %08x\n", i, sMicFWSubHeader.TDArray[i].Addr);

        sprintf(file_path, IM401_CUSTOM_FIRMWARE "SMicTD%u.dat",
            sMicFWSubHeader.TDArray[i].ID);
		printk("im401: parse_header SMicTD%u.dat path=%s\n", i, file_path);

        size = im401_read_file(file_path, &data);
		printk("im401: parse_header CUSTOM_FIRMWARE file size=%d\n", (int)size);

        if (size) {
			printk("im401: parse_header CUSTOM_FIRMWARE addr=0x%x, size=%d\n", sMicFWSubHeader.TDArray[i].Addr, (int)size);
            im401_dsp_burst_write(im401_i2c,
                sMicFWSubHeader.TDArray[i].Addr, data, size);
            kfree(data);
            continue;
        }

        sprintf(file_path, IM401_FIRMWARE "SMicTD%u.dat",
            sMicFWSubHeader.TDArray[i].ID);
		printk("im401: parse_header SMicTD%u.dat path=%s\n", i, file_path);

        size = im401_read_file(file_path, &data);
		printk("im401: parse_header FIRMWARE file size=%d\n",(int)size);

        if (size) {
			printk("im401: parse_header IM401_FIRMWARE addr=0x%x, size=%d\n", sMicFWSubHeader.TDArray[i].Addr,(int)size);
            im401_dsp_burst_write(im401_i2c,
                sMicFWSubHeader.TDArray[i].Addr, data, size);
            kfree(data);
        }
    }
	kfree(sMicFWSubHeader.TDArray);
	kfree(sMicFWHeader.BinArray);
}

static void im401_reset(struct i2c_client *im401_i2c)
{
    im401_write(im401_i2c, IM401_RESET, 0x10ec);
}

static void im401_dsp_load_fw(void) {
	const u8 *data;
	char file_path[32];
	size_t size = 0;
	int count = 0;
	unsigned int fw_addr;
	
	printk("function: im401_dsp_load_fw.\n");
	if (im401_private_fw_exist == 1) {
		if (im401_fw == NULL) {
			printk("im401_dsp_load_fw: request firmware 0x0fff0000.dat.\n");
			request_firmware(&im401_fw, "0x0fff0000.dat", &im401_i2c->dev);
		}
		im401_reset(im401_i2c);
		im401_enable_dsp_clock();

		sprintf(file_path, IM401_FIRMWARE "0x0fff0000.dat");
		printk("im401_dsp_load_fw---------firmware file path = %s\n", file_path);

        while (true) {
            size = im401_read_file(file_path, &data);
            printk("im401: firmware path = %s (%u)\n", file_path, (int)size);

			if (size || count >= 5)			
				break;
			msleep(1000);
			count++;
		}	
		if (size) {
			fw_addr = 0x0fff0000;
#ifdef SHOW_DL_TIME
			do_gettimeofday(&(txc.time));
			rtc_time_to_tm(txc.time.tv_sec,&tm);
			printk("im401-i2c fw download UTC time :%d-%d-%d %d:%d:%d /n",tm.tm_year+1900,tm.tm_mon, tm.tm_mday,tm.tm_hour,tm.tm_min,tm.tm_sec); 			
#endif
			im401_dsp_burst_write(im401_i2c, fw_addr, data, size);
#ifdef SHOW_DL_TIME
			do_gettimeofday(&(txc.time));
			rtc_time_to_tm(txc.time.tv_sec,&tm);
			printk("im401-i2c fw download UTC time :%d-%d-%d %d:%d:%d /n",tm.tm_year+1900,tm.tm_mon, tm.tm_mday,tm.tm_hour,tm.tm_min,tm.tm_sec); 			
#endif
			kfree(data);

			//sprintf(file_path, IM401_PRIVATE_DATA2);
			sprintf(file_path, IM401_FIRMWARE "0x0ffe0000.dat");
			while (true) {
				size = im401_read_file(file_path, &data);
				printk("im401: firmware path = %s (%u)\n", file_path, (int)size);
				if (size || count >= 5)			
					break;
				msleep(1000);
				count++;
			}
			if (size) {
				fw_addr = 0x0ffe0000;
#ifdef SHOW_DL_TIME
				do_gettimeofday(&(txc.time));
				rtc_time_to_tm(txc.time.tv_sec,&tm);
				printk("im401-i2c fw download UTC time :%d-%d-%d %d:%d:%d /n",tm.tm_year+1900,tm.tm_mon, tm.tm_mday,tm.tm_hour,tm.tm_min,tm.tm_sec); 			
#endif
				printk("im401_dsp_load_fw: writting 0x0ffe0000.dat.\n");
				im401_dsp_burst_write(im401_i2c, fw_addr, data, size);
#ifdef SHOW_DL_TIME
				do_gettimeofday(&(txc.time));
				rtc_time_to_tm(txc.time.tv_sec,&tm);
				printk("im401-i2c fw download UTC time :%d-%d-%d %d:%d:%d /n",tm.tm_year+1900,tm.tm_mon, tm.tm_mday,tm.tm_hour,tm.tm_min,tm.tm_sec); 			
#endif
				kfree(data);
			}
		}
	} else {
		if (im401_fw == NULL) {
			printk("im401_dap_load_fw: request firmware SMicBin.dat\n");
			request_firmware(&im401_fw, "SMicBin.dat", &im401_i2c->dev);
		}
		im401_reset(im401_i2c);
		im401_enable_dsp_clock();

		sprintf(file_path, IM401_FIRMWARE "SMicBin.dat");
        printk("im401_dsp_load_fw---------firmware file path = %s\n", file_path);

        while (true) {
            size = im401_read_file(file_path, &data);
			printk("im401_dsp_load_fw: firmware path = %s (%u)\n", file_path, (int)size);

            if (size || count >= 5)
                break;

            msleep(1000);
            count++;
        }

        if (size) {
            printk("im401: firmware file read success, size = %d\n", (int)size);
			im401_parse_header(data);
			kfree(data);
		} else {
			if (im401_fw) {
				printk("Error IM401: default firmware file read failure.\n");
				im401_parse_header(im401_fw->data);
			}
		}
	}
	//im401_set_clock_cal_pll();
	im401_dsp_start();
	im401_dsp_stop();

    release_firmware(im401_fw);
    im401_fw = NULL;
	printk("im401: im401_dsp_load_fw() success\n");
}

static void im401_fw_loaded(const struct firmware *fw, void *context)
{
    if (fw) {
        im401_fw = fw;
        im401_dsp_load_fw();
    }

}

static void im401_dsp_load_grammar_fw(void) {
	const u8 *data;
	char file_path[32];
	size_t size = 0;
	int count = 0;
	unsigned int fw_addr;
	
	if (im401_grammar_fw == NULL) {
		printk("im401_dsp_load_grammar_fw: reload...\n");
		request_firmware(&im401_grammar_fw, "grammar.dat", &im401_i2c->dev);
	}
	im401_reset(im401_i2c);
	im401_enable_dsp_clock();

	sprintf(file_path, IM401_PRIVATE_GRAMMAR);
	printk("im401_dsp_load_grammar_fw---------firmware file path = %s\n", file_path);

	while (true) {
		size = im401_read_file(file_path, &data);
		printk("im401: firmware path = %s (%u)\n", file_path, (int)size);

		if (size || count >= 5)			
			break;
		msleep(1000);
		count++;
	}	
	if (size) {
		fw_addr = 0x0fffffff;
#ifdef SHOW_DL_TIME
		do_gettimeofday(&(txc.time));
		rtc_time_to_tm(txc.time.tv_sec,&tm);
		printk("im401-i2c fw download UTC time :%d-%d-%d %d:%d:%d /n",tm.tm_year+1900,tm.tm_mon, tm.tm_mday,tm.tm_hour,tm.tm_min,tm.tm_sec); 			
#endif
		im401_dsp_burst_write(im401_i2c, fw_addr, data, size);
#ifdef SHOW_DL_TIME
		do_gettimeofday(&(txc.time));
		rtc_time_to_tm(txc.time.tv_sec,&tm);
		printk("im401-i2c fw download UTC time :%d-%d-%d %d:%d:%d /n",tm.tm_year+1900,tm.tm_mon, tm.tm_mday,tm.tm_hour,tm.tm_min,tm.tm_sec);
#endif
		kfree(data);
	}
	//im401_set_clock_cal_pll();
	im401_dsp_start();
	im401_dsp_stop();

	release_firmware(im401_grammar_fw);
	im401_grammar_fw = NULL;

	dev_dbg(&im401_i2c->dev, "grammar firmware loading end\n");
}

static int im401_readable_register(unsigned int reg)
{
    switch (reg) {
    case IM401_RESET:
    case IM401_ANA_CIRCUIT_CTRL_LDO1:
    case IM401_ANA_CIRCUIT_CTRL_LDO2:
    case IM401_ANA_CIRCUIT_CTRL_LDO3:
    case IM401_ANA_CIRCUIT_CTRL_ADC1_1:
    case IM401_ANA_CIRCUIT_CTRL_ADC1_2:
    case IM401_ANA_CIRCUIT_CTRL_ADC2_1:
    case IM401_ANA_CIRCUIT_CTRL_ADC2_2:
    case IM401_ANA_CIRCUIT_CTRL_ADC2_3:
    case IM401_ANA_CIRCUIT_CTRL_MICBST:
    case IM401_ANA_CIRCUIT_CTRL_ADCFED:
    case IM401_ANA_CIRCUIT_CTRL_INPUTBUF:
    case IM401_ANA_CIRCUIT_CTRL_VREF:
    case IM401_ANA_CIRCUIT_CTRL_MBIAS:
    case IM401_AD_DIG_FILTER_CTRL1:
    case IM401_AD_DIG_FILTER_CTRL2:
    case IM401_DFT_BIST_SCAN:
    case IM401_UPFILTER_CTRL1:
    case IM401_UPFILTER_CTRL2:
    case IM401_GPIO_CTRL1:
    case IM401_GPIO_CTRL2:
    case IM401_GPIO_CTRL3:
    case IM401_GPIO_STATUS:
    case IM401_DIG_PAD_CTRL1:
    case IM401_DIG_PAD_CTRL2:
    case IM401_DMIC_DATA_CTRL:
    case IM401_TEST_MODE_CTRL1:
    case IM401_TEST_MODE_CTRL2:
    case IM401_TEST_MODE_CTRL3:
    case IM401_VAD_CTRL1:
    case IM401_VAD_CTRL2:
    case IM401_VAD_CTRL3:
    case IM401_VAD_CTRL4:
    case IM401_VAD_STATUS1:
    case IM401_VAD_STATUS2:
    case IM401_BUF_SRAM_CTRL1:
    case IM401_BUF_SRAM_CTRL2:
    case IM401_BUF_SRAM_CTRL3:
    case IM401_BUF_SRAM_CTRL4:
    case IM401_BUF_SRAM_CTRL5:
    case IM401_BUF_SRAM_CTRL6:
    case IM401_BUF_SRAM_CTRL7:
    case IM401_AUTO_MODE_CTRL:
    case IM401_PWR_ANLG1:
    case IM401_PWR_ANLG2:
    case IM401_PWR_DIG:
    case IM401_PWR_DSP:
    case IM401_PRIV_INDEX:
    case IM401_PRIV_DATA:
    case IM401_BUF_MODE_CTRL_PLL_CAL1:
    case IM401_BUF_MODE_CTRL_PLL_CAL2:
    case IM401_BUF_MODE_CTRL_PLL_CAL3:
    case IM401_BUF_MODE_CTRL_PLL_CAL4:
    case IM401_BUF_MODE_CTRL_PLL_CAL5:
    case IM401_BUF_MODE_CTRL_PLL_CAL6:
    case IM401_KEY_FHRASE_CTRL_AVD:
    case IM401_AUTO_CLK_SEL_STATUS1:
    case IM401_AUTO_CLK_SEL_STATUS2:
    case IM401_AUTO_CLK_SEL_STATUS3:
    case IM401_AUTO_CLK_SEL_STATUS4:
    case IM401_PLL_CLOCK_CTRL1:
    case IM401_PLL_CLOCK_CTRL2:
    case IM401_PLL_CLOCK_CTRL3:
    case IM401_PLL_CAL_CTRL1:
    case IM401_PLL_CAL_CTRL2:
    case IM401_PLL_CAL_CTRL3:
    case IM401_PLL_CAL_CTRL4:
    case IM401_PLL_CAL_CTRL5:
    case IM401_PLL_CAL_CTRL6:
    case IM401_PLL_CAL_CTRL7:
    case IM401_PLL_CAL_CTRL8:
    case IM401_PLL_CAL_CTRL9:
    case IM401_PLL_CAL_STATUS1:
    case IM401_PLL_CAL_STATUS2:
    case IM401_PLL_CAL_STATUS3:
    case IM401_DSP_CTRL1:
    case IM401_DSP_CTRL2:
    case IM401_DSP_CTRL3:
    case IM401_DSP_CTRL4:
    case IM401_DSP_CTRL5:
    case IM401_DSP_CTRL6:
    case IM401_DSP_CTRL7:
    case IM401_DSP_CTRL8:
    case IM401_DSP_CTRL9:
    case IM401_DSP_CTRL10:
    case IM401_DSP_CTRL11:
    case IM401_DSP_CTRL12:
    case IM401_DSP_CTRL13:
    case IM401_DSP_CTRL14:
    case IM401_DSP_CTRL15:
    case IM401_PLL_CLK_EXT_CTRL1:
    case IM401_PLL_CLK_EXT_CTRL2:
    case IM401_ADC_EXT_CTRL1:
    case IM401_DUMMY_RTK1:
    case IM401_DUMMY_RTK2:
    case IM401_DUMMY_RTK3:
    case IM401_DUMMY_RTK4:
    case IM401_DUMMY_RTK5:
    case IM401_DUMMY_RTK6:
    case IM401_DUMMY_RTK7:
    case IM401_DUMMY_RTK8:
    case IM401_DUMMY_RTK9:
    case IM401_DUMMY_RTK10:
    case IM401_DUMMY_RTK11:
    case IM401_DUMMY_RTK12:
    case IM401_DUMMY_RTK13:
    case IM401_DUMMY_RTK14:
    case IM401_DUMMY_RTK15:
    case IM401_DUMMY_RTK16:
    case IM401_DUMMY_CUSTOMER1:
    case IM401_DUMMY_CUSTOMER2:
    case IM401_DUMMY_CUSTOMER3:
    case IM401_DUMMY_CUSTOMER4:
    case IM401_DUMMY_CUSTOMER5:
    case IM401_DUMMY_CUSTOMER6:
    case IM401_DUMMY_CUSTOMER7:
    case IM401_DUMMY_CUSTOMER8:
    case IM401_DUMMY_CUSTOMER9:
    case IM401_DUMMY_CUSTOMER10:
    case IM401_DUMMY_CUSTOMER11:
    case IM401_DUMMY_CUSTOMER12:
    case IM401_DUMMY_CUSTOMER13:
    case IM401_DUMMY_CUSTOMER14:
    case IM401_DUMMY_CUSTOMER15:
    case IM401_DUMMY_CUSTOMER16:
    case IM401_DSP_MEM_CTRL1:
    case IM401_DSP_MEM_CTRL2:
    case IM401_DSP_MEM_CTRL3:
    case IM401_DSP_MEM_CTRL4:
    case IM401_DSP_MEM_CTRL5:
    case IM401_DSP_MEM_CTRL6:
    case IM401_DSP_MEM_CTRL7:
    case IM401_DUMMY1:
    case IM401_DUMMY2:
    case IM401_DUMMY3:
    case IM401_VENDOR_ID:
    case IM401_VENDOR_ID1:
    case IM401_VENDOR_ID2:
        return true;
    default:
        return false;
    }
}

static ssize_t im401_reg_show(struct device *dev, 
    struct device_attribute *attr, char *buf)
{
    int count = 0;
    int i;
    int value;

    for (i = IM401_RESET; i <= IM401_VENDOR_ID2; i++) {
        if (im401_readable_register(i)) {
            value = im401_read(im401_i2c, i);
            if (value < 0)
                count += sprintf(buf + count, "%02x: XXXX\n",
                    i);
            else
                count += sprintf(buf + count, "%02x: %04x\n", i,
                    value);

            if (count >= PAGE_SIZE - 1)
                break;
        }
    }

    return count;
}

static ssize_t im401_reg_store(struct device *dev,
    struct device_attribute *attr, const char *buf, size_t count)
{
    unsigned int val = 0, addr = 0;
    int i;

    for (i = 0; i < count; i++) {
        if (*(buf + i) <= '9' && *(buf + i) >= '0')
            addr = (addr << 4) | (*(buf + i)-'0');
        else if (*(buf + i) <= 'f' && *(buf + i) >= 'a')
            addr = (addr << 4) | ((*(buf + i) - 'a') + 0xa);
        else if (*(buf + i) <= 'F' && *(buf + i) >= 'A')
            addr = (addr << 4) | ((*(buf + i)-'A') + 0xa);
        else
            break;
    }

    for (i = i + 1 ; i < count; i++) {
        if (*(buf + i) <= '9' && *(buf + i) >= '0')
            val = (val << 4) | (*(buf + i) - '0');
        else if (*(buf + i) <= 'f' && *(buf + i) >= 'a')
            val = (val << 4) | ((*(buf + i) - 'a') + 0xa);
        else if (*(buf + i) <= 'F' && *(buf + i) >= 'A')
            val = (val << 4) | ((*(buf + i) - 'A') + 0xa);
        else
            break;
    }

    if (addr > IM401_VENDOR_ID2 || val > 0xffff || val < 0)
        return count;

    if (i == count)
        pr_info("0x%02x = 0x%04x\n", addr,
            im401_read(im401_i2c, addr));
    else
        im401_write(im401_i2c, addr, val);

    return count;
}
static DEVICE_ATTR(im401_reg, 0644, im401_reg_show, im401_reg_store);

static ssize_t device_read(struct file *file, char __user * buffer,
	size_t length, loff_t * offset)
{
	char fc[5];
	size_t ret = 0;
	int mode, i;
	int reg_val;
	long ret_val = 0;

	char *local_buffer;

	local_buffer = 	(char *)kmalloc(length * sizeof(char), GFP_KERNEL);
	if (!local_buffer) {
		printk("IM401 device_read: local_buffer allocation failure.\n");
		goto out;
	}
	ret_val = copy_from_user(local_buffer, buffer, length);

	for (i = 0; i < length; i++)
		printk("IM401 device_read: local_buffer[%d]=>%d\n", i, local_buffer[i]);

	mode = (local_buffer[1] << 8) | local_buffer[0];
	//printk("IM401 device_read: buffer=>%s, length=>%d, offset=>%d\n", *buffer, length, *offset);

	//if (copy_from_user(&mode, buffer, 1))
	//	return -EFAULT;
	//printk("IM401 device_read: mode = %d\n", mode);

	switch (mode) {
	case IM401_DSP_IRQQUERY:
		reg_val = im401_read(im401_i2c, IM401_DSP_CTRL2);
		printk("IM401 device_read IRQ: reg_val = 0x%02x\n", reg_val);
		//ret = sprintf(fc, "%d", reg_val);
		if (copy_to_user(buffer, &reg_val, 2))
			return -EFAULT;
		printk("IM401 device_read IRQ status: buffer = 0x%02x\n", (unsigned int)*buffer);
		break;
	case IM401_QUERY_DUMMY_CUSTOMER14:
		reg_val = im401_read(im401_i2c, IM401_DUMMY_CUSTOMER14);
		printk("IM401 device_read Command: reg_val = 0x%02x\n", reg_val);
		//ret = sprintf(fc, "%d", reg_val);
		if (copy_to_user(buffer, &reg_val, 2))
			return -EFAULT;
		printk("IM401 device_read Command: buffer = 0x%02x\n", (unsigned int)*buffer);
		break;
	case IM401_DSP_QUERYFWSTATE:
		reg_val = im401_read(im401_i2c, IM401_VAD_STATUS1);
		printk("IM401 device_read FW: reg_val = 0x%02x\n", reg_val); 
		//ret = sprintf(fc, "%d", reg_val);
		if (copy_to_user(buffer, &reg_val, 2))
				return -EFAULT;
		printk("IM401 device_read FW: buffer => 0x%02x\n", (unsigned int)*buffer);
		break;
	case IM401_HOST_IRQQUERY:
		if (im401_host_irqstatus == 1) {
			ret_val = copy_to_user(buffer, &im401_host_irqstatus, sizeof(unsigned short));
			im401_host_irqstatus = 0;
		} else {
			ret_val = copy_to_user(buffer, &im401_host_irqstatus, sizeof(unsigned short));
		}
		printk("IM401 device_read FW: buffer=>%02x\n", (unsigned int)*buffer);
		break;
	case IM401_REG_DUMP:
		printk("IM401 device_read: reg_dump\n");
		for (i = 0; i < im401_reg_num; i++)
		{
			reg_val =  im401_read(im401_i2c, im401_regs[i]);
			local_buffer[2 * (i + 1)] = reg_val & 0xff;
			local_buffer[2 * (i + 1) + 1] = (reg_val >> 8) & 0xff;
			printk("IM401 device_read: addr = 0x%x, reg_val=>0x%4x\n", im401_regs[i], reg_val);
		}
		if (copy_to_user(buffer, local_buffer, length)) {
			if (local_buffer) kfree(local_buffer);
			return -EFAULT;
		}

		printk("IM401 device_read Command: buffer=>%2x\n", (unsigned int)*buffer);
		break;
	default:
		ret = sprintf(fc, "%d", im401_read(im401_i2c, IM401_DUMMY3));

		if (copy_to_user(buffer, fc, ret))
			return -EFAULT;

		*offset += ret;
	}
	if (local_buffer) kfree(local_buffer);
out:
	return ret;
}

static ssize_t device_write(struct file *file, const char __user * buffer,
    size_t length, loff_t * offset)
{
	//char mode;
	int reg_val;
	int mode;
	int i;
	char *local_buffer;
	long ret = 0;

	local_buffer = 	(char *)kmalloc(length * sizeof(char), GFP_KERNEL);
	if (!local_buffer) {
		printk("IM401 device_write: local_buffer allocation failure.\n");
		goto out;
	}
	ret = copy_from_user(local_buffer, buffer, length);

	for (i = 0; i < length; i++)
		printk("IM401 device_write: local_buffer[%d]=>%d\n", i, local_buffer[i]);
	//if (copy_from_user(&mode, buffer, 1))
	//	goto out;

	mode = (local_buffer[1] << 8) | local_buffer[0];

	printk("IM401 device_write: mode=>%d\n", mode);
	switch(mode) {
	case IM401_DSP:
		printk("IM401 device_write: dsp_start\n");
		im401_dsp_start();
		break;
	case IM401_DSP_ONESHOT:
		printk("IM401 device_write: dsp_oneshot\n");
		im401_dsp_one_shot();
		break;
	case IM401_DSP_RELOAD:
		printk("IM401 device_write: dsp_reload\n");
		schedule_delayed_work(&reload_work, msecs_to_jiffies(50));
		break;
	case IM401_DSP_LOADGRAMMAR:
		printk("IM401 device_write: dsp_loadgrammar\n");
		schedule_delayed_work(&load_grammar_work, msecs_to_jiffies(50));
		break;
	case IM401_DSP_IRQRESET:
		reg_val = im401_read(im401_i2c, IM401_DSP_CTRL2);
		if ((reg_val & 0x1) == 0x1) {
			im401_write(im401_i2c, IM401_DSP_CTRL2, 0x0000);
		}
		break;
	case IM401_DSP_STARTONESHOT_FLAG:
		im401_write(im401_i2c, IM401_DUMMY2, 0x5);
		reg_val = im401_read(im401_i2c, IM401_DUMMY2);
		printk("IM401 device_write: 0x%2x = 0x%4x.\n", IM401_DUMMY2, reg_val);
		break;
	case IM401_DSP_STOPONESHOT_FLAG:
		im401_write(im401_i2c, IM401_DUMMY2, 0xa);
		reg_val = im401_read(im401_i2c, IM401_DUMMY2);
		printk("IM401 device_write: 0x%2x = 0x%4x.\n", IM401_DUMMY2, reg_val);
		break;
	case IM401_DSP_BOOST_RESET:
		printk("IM401 device_write: DMIC boost reset.\n");
		im401_write(im401_i2c, IM401_UPFILTER_CTRL1, 0x6aa0);
		im401_write(im401_i2c, IM401_UPFILTER_CTRL1, 0x6ea0);
		break;
	case IM401_DSP_FACTORY_ENTER:
		reg_val = im401_read(im401_i2c, IM401_DUMMY_CUSTOMER10);
		printk("IM401 device_write: 0x%2x = 0x%4x.\n", IM401_DUMMY_CUSTOMER10, reg_val);
		im401_write(im401_i2c, IM401_DUMMY_CUSTOMER10, 0x708); // For factory mode, higher value is easy to be detected.
		break;
	case IM401_DSP_FACTORY_QUIT:
		reg_val = im401_read(im401_i2c, IM401_DUMMY_CUSTOMER10);
		printk("IM401 device_write: 0x%2x = 0x%4x.\n", IM401_DUMMY_CUSTOMER10, reg_val);
		im401_write(im401_i2c, IM401_DUMMY_CUSTOMER10, 0x64);
		break;
	case IM401_DSP_SVTHD:
		reg_val = (local_buffer[3] << 8) | local_buffer[2];
		printk("IM401 device_write SVTHD: 0x%2x = 0x%4x.\n", IM401_DUMMY_CUSTOMER9, reg_val);
		im401_write(im401_i2c, IM401_DUMMY_CUSTOMER9, reg_val);
		break;
#ifndef LG_DEBUG
	case 'a':
		reg_val = im401_read(im401_i2c, IM401_DUMMY2);
		printk("im401 device_write: 0xFB value(%d)\n",reg_val);
		reg_val = im401_write(im401_i2c, IM401_DUMMY2,0x00);
		reg_val = im401_read(im401_i2c, IM401_DUMMY2);
		printk("im401 device_write: 0xFB value(%d)\n",reg_val);
		break;
	case 'b':
		printk("im401 device_write: 0xFB value(0x0a)\n");
		reg_val = im401_write(im401_i2c, IM401_DUMMY2,0x0a);
		break;
	case 'c':
		printk("im401 device_write: 0xFB value(0x05)\n");
		reg_val = im401_write(im401_i2c, IM401_DUMMY2,0x05);
		break;
	case 'd':
		reg_val = im401_read(im401_i2c, IM401_DUMMY2);
		printk("im401 device_write: 0xFB value(%d)\n",reg_val);
		break;
#endif
    case IM401_NORMAL:
    default:
		printk("IM401 device_write: dsp_stop\n");
        im401_dsp_stop();
    }
out:
    return length;
}

struct file_operations iM401_fops = {
    .owner = THIS_MODULE,
    .read = device_read,
    .write = device_write,
};

static struct miscdevice im401_dev = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "i2c-im401",
    .fops = &iM401_fops
};

static void im401_reload_work(struct work_struct *work)
{
	printk("im401_reload_work.\n");
	im401_start_rest = false;
	im401_dsp_load_fw();
}

static void im401_load_grammar_work(struct work_struct *work)
{
	printk("im401_load_grammar_work.\n");
	im401_start_rest = false;
	im401_dsp_load_grammar_fw();
}

static void dsp_start_bypass_work(struct work_struct *work)
{
	printk("dsp_start_bypass_work.\n");
	im401_dsp_one_shot();
}

#ifdef IM401_IRQ
static u32 im401_irq_handler(void *para)
{
    unsigned long status;
    int    upio_index;
    
    printk("im401 irq handler entering...\n");

    im401_host_irqstatus = 1;
    
    return IRQ_HANDLED;
}
#endif

void im401_parse_dt(struct device *dev, struct IM401_gpio *im401_gpio)
{
    struct device_node *np = dev->of_node;
	
    /* irq gpio info */
    im401_gpio->irq_gpio = of_get_named_gpio_flags(np, "fmedia_im401,gpio_irq", 0, NULL);
}

static void im401_report_interrupt_event (struct IM401_gpio *im401_gpio, int value)
{
    printk("im401_report_interrupt_event() trying to wake up\n");
	input_event(im401_gpio->input_dev,EV_KEY,KEY_MEDIA, value);
	input_sync(im401_gpio->input_dev); 
}

static void handle_interrupt_work(struct work_struct *work)
{
	printk("handle_interrupt_work.\n");
	im401_report_interrupt_event(im401_gpio_g,MAX14688_BUTTON_DOWN_VALUE);
	im401_report_interrupt_event(im401_gpio_g,MAX14688_BUTTON_UP_VALUE);
}

static irqreturn_t im401_dev_irq_handler(int irq, void *dev_id)
{
    struct IM401_gpio *im401_gpio = dev_id;
    unsigned int irq_gpio_val;
    unsigned long flags;
    long wake_timeout = 2;

    irq_gpio_val = gpio_get_value(im401_gpio->irq_gpio);
    if (irq_gpio_val == 0) {
        printk("%s: False Interrupt!\n", __func__);
        return IRQ_HANDLED;
    }
    printk("im401_dev_irq_handler interrupt occurred!");

    spin_lock_irqsave(&im401_gpio->irq_enabled_lock, flags);

    wake_lock_timeout(&im401_wake_lock,wake_timeout*HZ);
    schedule_delayed_work(&interrupt_work, msecs_to_jiffies(50));

    spin_unlock_irqrestore(&im401_gpio->irq_enabled_lock, flags);

    return IRQ_HANDLED;
}

/*
static void im401_disable_irq(struct IM401_gpio *im401_gpio)
{
    unsigned long flags;

    spin_lock_irqsave(&im401_gpio->irq_enabled_lock, flags);
    if (im401_gpio->irq_enabled) {
        disable_irq_nosync(im401_gpio->irq_gpio);
        disable_irq_wake(im401_gpio->irq_gpio);
        im401_gpio->irq_enabled = false;
    }
    spin_unlock_irqrestore(&im401_gpio->irq_enabled_lock, flags);
}
*/

static int im401_i2c_suspend(struct i2c_client *i2c,pm_message_t mesg)
{
    printk("im401 Driver suspend\n");
    return 0;
}

static int im401_i2c_resume(struct i2c_client *i2c)
{
    printk("im401 Driver resume\n");
    return 0;
}

static int im401_i2c_probe (struct i2c_client *client,
    const struct i2c_device_id *id)
{
    int ret;
#ifndef FEATURE_LG_G3_TEST_ONLY
    struct file *fp;
#endif
    char file_path[32];
    struct IM401_gpio *im401_gpio = NULL;
    unsigned int irq_num=0;

    im401_gpio = kzalloc(sizeof(*im401_gpio), GFP_KERNEL);
    if (im401_gpio == NULL) {
        printk("Error: im401_probe malloc failed\n");
        ret = -ENOMEM;
        goto err_exit;
    }
    else {
        im401_gpio_g = im401_gpio;
    }
    im401_parse_dt(&client->dev, im401_gpio);

    printk("im401 Driver Init\n");

    pr_info("im401 Driver Version %s\n", DRIVER_VERSION);

    ret = device_create_file(&client->dev, &dev_attr_im401_reg);
    if (ret < 0)
        printk("Error: im401_probe failed to add im401_reg sysfs files\n");

    im401_reset(client);
    
    im401_i2c = client;

    sprintf(file_path, IM401_PRIVATE_DATA1);
    printk("im401-i2c probe: file_path = %s\n", file_path);
#ifdef FEATURE_LG_G3_TEST_ONLY // to load private fw(0x0fff0000.dat)
#if 0
    im401_private_fw_exist = 1;
    request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG, "0x0fff0000.dat", &client->dev, GFP_KERNEL, client, im401_fw_loaded);
    printk("im401-i2c probe: request_firmware_nowait 0x0fff0000.dat\n");
#else
	request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG, "SMicBin.dat", &client->dev, GFP_KERNEL, client, im401_fw_loaded);
	printk("im401-i2c probe: request_firmware_nowait SMicBin.dat\n");
#endif
#else
    fp = filp_open(file_path, O_RDONLY, 0);
    if (!IS_ERR(fp)) {
        filp_close(fp, 0);
        im401_private_fw_exist = 1;
        request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG, "0x0fff0000.dat", &client->dev, GFP_KERNEL, client, im401_fw_loaded);
        printk("im401-i2c probe: request_firmware_nowait 0x0fff0000.dat\n");
    } else {
        request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG, "SMicBin.dat", &client->dev, GFP_KERNEL, client, im401_fw_loaded);
        printk("im401-i2c probe: request_firmware_nowait SMicBin.dat\n");
    }
#endif

    ret = misc_register(&im401_dev);
    if (ret)
        printk("Error: im401_probe IM401_probe misc_register failed\n");

    INIT_DELAYED_WORK(&reload_work, im401_reload_work);
	INIT_DELAYED_WORK(&load_grammar_work, im401_load_grammar_work);
	INIT_DELAYED_WORK(&dsp_start_bypass, dsp_start_bypass_work);
	INIT_DELAYED_WORK(&interrupt_work, handle_interrupt_work);
	
    ret = gpio_request(im401_gpio->irq_gpio,"im401_int");
    if (ret) {
        printk("Error: im401_probe IM401_probe gpio_request failed\n");
        goto err_int;
    }
    ret = gpio_direction_input(im401_gpio->irq_gpio);


    irq_num = gpio_to_irq(im401_gpio->irq_gpio);
    /* init mutex and queues */
    mutex_init(&im401_gpio->read_mutex);
    spin_lock_init(&im401_gpio->irq_enabled_lock);

    wake_lock_init(&im401_wake_lock, WAKE_LOCK_SUSPEND, "IM401WAKE");

    /* request irq.  the irq is set whenever the chip has data available
     * for reading.  it is cleared when all data has been read.
     */
    printk("im401_probe irq_num(%d) gpio_request client->irq (%d) im401_gpio->irq_gpio(%d)\n",irq_num,client->irq,im401_gpio->irq_gpio);
    im401_gpio->irq_enabled = true;
    ret = request_irq(im401_i2c->irq, im401_dev_irq_handler,
              IRQF_TRIGGER_RISING, client->name, im401_gpio);
    if (ret) {
        printk("Error: im401_probe request_irq failed\n");
        goto err_request_irq_failed;
    }

    ret = enable_irq_wake(im401_i2c->irq);
    if (ret) {
        printk("Error: im401_probe enable_irq_wake failed ret(%d)\n",ret);
        goto err_request_irq_failed;
    }
    //im401_disable_irq(im401_gpio);
    i2c_set_clientdata(client, im401_gpio);

    im401_gpio->dev = &client->dev;
    im401_gpio->kobj = &client->dev.kobj;
    
    im401_gpio->input_dev = input_allocate_device();
    if (unlikely(!im401_gpio->input_dev)) {
	  printk ("Error: im401_probe failed to allocatoin  memory for new input device");
	  ret = -ENOMEM;
	  goto err_exit;
    }

    im401_gpio->input_dev->name = DRIVER_NAME;
    im401_gpio->input_dev->phys = DRIVER_NAME"/input0";
    im401_gpio->input_dev->dev.parent = im401_gpio->dev;

    input_set_capability(im401_gpio->input_dev,EV_SW,SW_HEADPHONE_INSERT);
    input_set_capability(im401_gpio->input_dev,EV_KEY,KEY_HOME);
    input_set_capability(im401_gpio->input_dev,EV_KEY,KEY_MEDIA);
    

    ret = input_register_device (im401_gpio->input_dev);
    if (unlikely(ret)) {
    	printk ("Error: im401_probe failed to input_register_device for new input device");
    	input_free_device(im401_gpio->input_dev);
    	im401_gpio->input_dev=NULL;
    	ret = -ENXIO;
    	goto err_exit;
    }

#ifdef IM401_RT_CLK_ENABLE
	fm_clk = clk_get(im401_gpio->dev, "fmedia_xo");
	if (IS_ERR(fm_clk)) {
		ret = PTR_ERR(fm_clk);
		pr_err("%s :: woohyun.seok :: could not get clock\n", __func__);
		return ret;
	}

	ret = clk_prepare_enable(fm_clk);
	if (ret) {
		pr_err("%s :: woohyun.seok :: could not enable clock\n", __func__);
		return ret;
	}
#endif

    return 0;

    err_request_irq_failed:
	misc_deregister(&im401_dev);

    err_int:
	kfree(im401_gpio);

    err_exit:
      printk("Error: im401_probe err_exit failed\n");
      
    return ret;
}

static int im401_i2c_remove (struct i2c_client *client)
{
    printk("im401 Driver remove\n");
    
    im401_dsp_stop();
    im401_i2c = NULL;

    if (im401_dev.minor)
        misc_deregister(&im401_dev);
    wake_lock_destroy(&im401_wake_lock);
    return 0;
}


static const struct i2c_device_id im401_i2c_ids[] = {
    { DRIVER_NAME, 0 },
    { /* end of array */ }
};
MODULE_DEVICE_TABLE(i2c, im401_i2c_ids);

#ifdef CONFIG_OF
static const struct of_device_id im401_device_ids[] = {
    { .compatible = "fmedia,im401" },
    { }
};

MODULE_DEVICE_TABLE(of, im401_device_ids);
#endif /* CONFIG_OF */

static struct i2c_driver im401_i2c_driver = {
    .driver.name           = DRIVER_NAME,
    .driver.owner          = THIS_MODULE,
#ifdef CONFIG_OF
    .driver.of_match_table = of_match_ptr(im401_device_ids),
#endif /* CONFIG_OF */
    .probe                 = im401_i2c_probe,
    .remove                = im401_i2c_remove,
    .id_table              = im401_i2c_ids,
    .suspend               = im401_i2c_suspend,
    .resume                = im401_i2c_resume,

};

module_i2c_driver(im401_i2c_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_VERSION(DRIVER_VERSION);

