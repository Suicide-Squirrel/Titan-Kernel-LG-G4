/* Copyright (c) 2011-2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt) "%s:%d " fmt, __func__, __LINE__

#include <linux/module.h>
#include "msm_sd.h"
#include "msm_mh1.h"
#include "msm_cci.h"
#include <linux/syscalls.h>
#include "../stm/spi_common.h"

#include <linux/io.h>

#include "../msm_sensor.h"
#include <linux/interrupt.h>

DEFINE_MSM_MUTEX(msm_mh1_mutex);

#define BIN_SIZE 0x80000
#define I2C_WRITE_UNIT_SIZE 0x1000

#undef CDBG
#define CDBG(fmt, args...) pr_debug(fmt, ##args)

#undef DUMP
//#define DUMP
#define ISR_USE
//#define I2C_DOWNLOAD
//#define TRF_SHD_TBL
//#define LOG_SAVE
#define FW_PUSH

#ifdef ISR_USE
	uint32_t mh1_irq_gpio;
#endif

static struct v4l2_file_operations msm_mh1_v4l2_subdev_fops;
static struct i2c_driver msm_mh1_driver;

static struct msm_camera_i2c_fn_t msm_sensor_qup_func_tbl = {
	.i2c_read = msm_camera_qup_i2c_read,
	.i2c_read_seq = msm_camera_qup_i2c_read_seq,
	.i2c_write = msm_camera_qup_i2c_write,
	.i2c_write_seq = msm_camera_qup_i2c_write_seq,
	.i2c_write_table = msm_camera_qup_i2c_write_table,
	.i2c_write_seq_table = msm_camera_qup_i2c_write_seq_table,
	.i2c_write_table_w_microdelay =
	msm_camera_qup_i2c_write_table_w_microdelay,
};

static struct msm_mh1_ctrl_t g_mctrl;
static uint8_t mh1_settings[16];
static uint8_t current_resolution;

static const struct i2c_device_id msm_mh1_id[] = {
	{"mh1", (kernel_ulong_t)&g_mctrl},
	{ },
};

static int32_t msm_mh1_get_subdev_id(struct msm_mh1_ctrl_t *mh1_ctrl,
									   void *arg)
{
	uint32_t *subdev_id = (uint32_t *)arg;
	CDBG("Enter\n");
	if (!subdev_id) {
		pr_err("failed\n");
		return -EINVAL;
	}
	if (mh1_ctrl->mh1_device_type == MSM_CAMERA_PLATFORM_DEVICE)
		*subdev_id = mh1_ctrl->pdev->id;
	else
		*subdev_id = mh1_ctrl->subdev_id;

	CDBG("subdev_id %d\n", *subdev_id);
	CDBG("Exit\n");
	return 0;
}

#ifdef DUMP
static int mh1_mem_dump(struct msm_camera_i2c_client *client, unsigned short len, unsigned int addr, unsigned char *val)
{
	struct i2c_msg msg;
	unsigned char data[8];
	unsigned char recv_data[len + 3];
	int i, err = 0;

	if (!client->client->adapter)
		return -ENODEV;

	if (len <= 0)
		return -EINVAL;

	msg.addr = client->client->addr >> 1;
	msg.flags = 0;
	msg.len = sizeof(data);
	msg.buf = data;

	/* high byte goes out first */
	data[0] = 0x00;
	data[1] = 0x03;
	data[2] = (addr >> 24) & 0xFF; //0x18;
	data[3] = (addr >> 16) & 0xFF;
	data[4] = (addr >> 8) & 0xFF;
	data[5] = addr & 0xFF;
	data[6] = (len >> 8) & 0xFF;
	data[7] = len & 0xFF;

	for (i = 3; i; i--) {
		err = i2c_transfer(client->client->adapter, &msg, 1);
		if (err == 1)
			break;
		else
			pr_err("i2c write error\n");

		msleep(20);
	}

	if (err != 1)
		return err;

	msg.flags = I2C_M_RD;
	msg.len = sizeof(recv_data);
	msg.buf = recv_data;
	for (i = 3; i; i--) {
		err = i2c_transfer(client->client->adapter, &msg, 1);
		if (err == 1)
			break;
		else
			pr_err("i2c read error\n");

		msleep(20);
	}

	if (err != 1)
		return err;

	if (len != (recv_data[1] << 8 | recv_data[2]))
		pr_err("expected length %d, but return length %d\n",
			len, recv_data[1] << 8 | recv_data[2]);

	memcpy(val, recv_data + 3, len);

	/*pr_err("address %#x, length %d\n", addr, len);*/	/*for debug*/
	return err;
}
#endif

#ifdef I2C_DOWNLOAD
static int mh1_mem_write(struct msm_camera_i2c_client *client, unsigned char cmd,
		unsigned short len, unsigned int addr, unsigned char *val)
{
	struct i2c_msg msg;
	unsigned char data[len + 8];
	int i, err = 0;

	if (!client->client->adapter)
		return -ENODEV;

	msg.addr = client->client->addr >> 1;
	msg.flags = 0;
	msg.len = sizeof(data);
	msg.buf = data;

	/* high byte goes out first */
	data[0] = 0x00;
	data[1] = cmd;
	data[2] = ((addr >> 24) & 0xFF);
	data[3] = ((addr >> 16) & 0xFF);
	data[4] = ((addr >> 8) & 0xFF);
	data[5] = (addr & 0xFF);
	data[6] = ((len >> 8) & 0xFF);
	data[7] = (len & 0xFF);
	memcpy(data + 2 + sizeof(addr) + sizeof(len), val, len);

	pr_err("address %#x, length %d\n", addr, len);
	for (i = 3; i; i--) {
		err = i2c_transfer(client->client->adapter, &msg, 1);
		if (err == 1)
			break;
		msleep(20);
	}

	return err;
}
#endif

#ifdef LOG_SAVE
static int mh1_log_read(struct msm_camera_i2c_client *client, uint16_t len, uint32_t addr, unsigned char *val)
{
	struct i2c_msg msg;
	unsigned char data[8];
	unsigned char recv_data[len + 3];
	int i, err = 0;

	if (!client->client->adapter)
		return -ENODEV;

	if (len <= 0)
		return -EINVAL;

	msg.addr = client->client->addr >> 1;
	msg.flags = 0;
	msg.len = sizeof(data);
	msg.buf = data;

	/* high byte goes out first */
	data[0] = 0x00;
	data[1] = 0x03;
	data[2] = (addr >> 24) & 0xFF;
	data[3] = (addr >> 16) & 0xFF;
	data[4] = (addr >> 8) & 0xFF;
	data[5] = addr & 0xFF;
	data[6] = (len >> 8) & 0xFF;
	data[7] = len & 0xFF;

	for (i = 3; i; i--) {
		err = i2c_transfer(client->client->adapter, &msg, 1);
		if (err == 1)
			break;
		msleep(20);
	}

	if (err != 1)
		return err;

	msg.flags = I2C_M_RD;
	msg.len = sizeof(recv_data);
	msg.buf = recv_data;
	for (i = 3; i; i--) {
		err = i2c_transfer(client->client->adapter, &msg, 1);
		if (err == 1)
			break;
		msleep(20);
	}

	if (err != 1)
		return err;

	if (len != (recv_data[1] << 8 | recv_data[2]))
		pr_err("expected length %d, but return length %d\n",
			len, recv_data[1] << 8 | recv_data[2]);

	memcpy(val, recv_data + 3, len);

	CDBG("address %#x, length %d\n", addr, len);
	return err;
}
#endif

static int mh1_read(struct msm_camera_i2c_client *client, unsigned char len,
	unsigned char category, unsigned char byte, int *val, bool log)
{
	struct i2c_msg msg;
	unsigned char data[5];
	unsigned char recv_data[len + 1];
	int i, err = 0;

	if (!client->client->adapter)
		return -ENODEV;

	if (len != 0x01 && len != 0x02 && len != 0x04)
		return -EINVAL;

	msg.addr = client->client->addr >> 1;
	msg.flags = 0;
	msg.len = sizeof(data);
	msg.buf = data;

	/* high byte goes out first */
	data[0] = msg.len;
	data[1] = 0x01;			/* Read category parameters */
	data[2] = category;
	data[3] = byte;
	data[4] = len;

	for (i = 3; i; i--) {
		err = i2c_transfer(client->client->adapter, &msg, 1);
		if (err == 1)
			break;
		msleep(20);
	}

	if (err != 1) {
		pr_err("category %#x, byte %#x, err %d\n",
			category, byte, err);
		return err;
	}

	msg.flags = I2C_M_RD;
	msg.len = sizeof(recv_data);
	msg.buf = recv_data;
	for (i = 3 ; i; i--) {
		err = i2c_transfer(client->client->adapter, &msg, 1);
		if (err == 1)
			break;
		msleep(20);
	}

	if (err != 1) {
		pr_err("RD category %#x, byte %#x, err %d\n",
			category, byte, err);
		return err;
	}

	if (recv_data[0] != sizeof(recv_data))
		pr_err("expected length %d, but return length %d\n", (int)sizeof(recv_data), (int)recv_data[0]);

	if (len == 0x01)
		*val = recv_data[1];
	else if (len == 0x02)
		*val = recv_data[1] << 8 | recv_data[2];
	else
		*val = recv_data[1] << 24 | recv_data[2] << 16 |
				recv_data[3] << 8 | recv_data[4];

	if (log)
		pr_err("[ %4d ] Read %s %#02x, byte %#x, value %#x\n",
			__LINE__, (len == 4 ? "L" : (len == 2 ? "W" : "B")),
			category, byte, *val);

	return err;
}

static int mh1_write(struct msm_camera_i2c_client *client, unsigned char len,
	unsigned char category, unsigned char byte, int val, bool log)
{
	struct i2c_msg msg;
	unsigned char data[len + 4];
	int i, err;

	if (!client->client->adapter)
		return -ENODEV;

	if (len != 0x01 && len != 0x02 && len != 0x04)
		return -EINVAL;

	msg.addr = client->client->addr >> 1;
	msg.flags = 0;
	msg.len = sizeof(data);
	msg.buf = data;

	data[0] = msg.len;
	data[1] = 0x02;			/* Write category parameters */
	data[2] = category;
	data[3] = byte;
	if (len == 0x01) {
		data[4] = (val & 0xFF);
	} else if (len == 0x02) {
		data[4] = ((val >> 8) & 0xFF);
		data[5] = (val & 0xFF);
	} else {
		data[4] = ((val >> 24) & 0xFF);
		data[5] = ((val >> 16) & 0xFF);
		data[6] = ((val >> 8) & 0xFF);
		data[7] = (val & 0xFF);
	}

	if (log)
		pr_err("[ %4d ] Write %s %#x, byte %#x, value %#x\n",
			__LINE__, (len == 4 ? "L" : (len == 2 ? "W" : "B")),
			category, byte, val);

	for (i = 3; i; i--) {
		err = i2c_transfer(client->client->adapter, &msg, 1);
		if (err == 1)
			break;
		msleep(20);
	}

	if (err != 1) {
		pr_err("category %#x, byte %#x, err %d\n",
			category, byte, err);
		return err;
	}

	return err;
}

static int mh1_burst_write(struct msm_camera_i2c_client *client, unsigned char len,
	unsigned char category, unsigned char byte, unsigned char * ae_data)
{
	struct i2c_msg msg;
	unsigned char data[len + 4];
	int i, err;

	if (!client->client->adapter)
		return -ENODEV;

	if (len != 10)
		return -EINVAL;

	msg.addr = client->client->addr >> 1;
	msg.flags = 0;
	msg.len = sizeof(data);
	msg.buf = data;

	data[0] = msg.len;
	data[1] = 0x02;			/* Write category parameters */
	data[2] = category;
	data[3] = byte;

	data[4] = *ae_data;
	data[5] = *(ae_data+1);
	data[6] = *(ae_data+2);
	data[7] = *(ae_data+3);
	data[8] = *(ae_data+4);
	data[9] = *(ae_data+5);
	data[10] = *(ae_data+6);
	data[11] = *(ae_data+7);
	data[12] = *(ae_data+8);
	data[13] = *(ae_data+9);

	for (i = 3; i; i--) {
		err = i2c_transfer(client->client->adapter, &msg, 1);
		if (err == 1)
			break;
		msleep(20);
	}

	if (err != 1) {
		pr_err("category %#x, byte %#x, err %d\n",
			category, byte, err);
		return err;
	}

	return err;
}

#ifdef LOG_SAVE
static uint8_t log_cnt;
void mh1_save_log(struct msm_mh1_ctrl_t *m_ctrl)
{
	struct file *fp = NULL;
	mm_segment_t old_fs;
	char filepath[256];
	int ptr = 0, rc;
	int addr = 0;
	uint8_t  len = 0xFF;
	uint32_t i, intram_unit = 4096;

	u8 buf[4096];

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	sprintf(filepath, "/data/mh1_log_%d.txt", log_cnt);

	fp = filp_open(filepath,
		O_WRONLY|O_CREAT|O_TRUNC, S_IRUGO|S_IWUGO|S_IXUSR);
	if (IS_ERR(fp)) {
		pr_err("failed to open %s, err %ld\n",
			filepath, PTR_ERR(fp));
		rc = -ENOENT;
		goto file_out;
	}

	//1. read LOG_STR_ADD
	rc = mh1_read(m_ctrl->i2c_client, 4, 0x0D, 0x08, &addr, true);  //value size, category, register, address, log
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		goto out;
	}
	
	rc = mh1_write(g_mctrl.i2c_client, 1, 0x0D, 0x0E, 0x02, true);	//value size, category, register, value, log
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		goto out;
	}

	rc = mh1_write(g_mctrl.i2c_client, 1, 0x0D, 0x0C, ptr, true);	//value size, category, register, value, log
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		goto out;
	}

	rc = mh1_write(g_mctrl.i2c_client, 1, 0x0D, 0x0E, 0x03, true);	//value size, category, register, value, log
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		goto out;
	}

	while (ptr < 10000) {


		while (len == 0xff){
			rc = mh1_read(m_ctrl->i2c_client, 1, 0x0D, 0x07, (int)&len, true);  //value size, category, register, address, log
			if (rc < 0) {
				pr_err("%s failed %d\n", __func__, __LINE__);
				goto out;
			}
		}
		if (len == 0x00)
			break;

		i = 0;
		while(i <= len) {
			/*pr_err("### i %d, len %d, (len - i = %d)", i, len, len - i);*/
			if ((len - i) <= intram_unit) {
				rc = mh1_log_read(m_ctrl->i2c_client, len - i, addr + i, buf);
				if (err < 0) {
					pr_err("i2c falied, err %d\n", err);
					goto out;
				}
				vfs_write(fp, buf, len - i, &fp->f_pos);
				break;
			}
			rc = mh1_log_read(m_ctrl->i2c_client, intram_unit, addr + i, buf);
			if (rc < 0) {
				pr_err("i2c falied, err %d\n", err);
				goto out;
			}
			vfs_write(fp, buf, intram_unit,  &fp->f_pos);
			i += intram_unit;
		}
		len = 0xff;
		ptr++;

	}
	
	rc = mh1_write(m_ctrl->i2c_client, 1, 0x0D, 0x0E, 0x01, true);  //value size, category, register, address, log
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		goto out;
	}

out:
	if (!IS_ERR(fp))
		filp_close(fp, current->files);

file_out:
	set_fs(old_fs);
}

#endif

#ifdef I2C_DOWNLOAD
static int mh1_program_fw(struct msm_mh1_ctrl_t *m_ctrl, unsigned char *buf,
	unsigned int count)
{


	int i, rc = 0;
	int test_count = 0;
	unsigned int addr = 0x01000000;

	for (i = 0; i < I2C_WRITE_UNIT_SIZE*count; i += I2C_WRITE_UNIT_SIZE) {
		rc = mh1_mem_write(m_ctrl->i2c_client, 0x04, I2C_WRITE_UNIT_SIZE,
				addr, buf + i);

		pr_err("fw Send = %x count = %d\n", i, test_count++);
		addr = addr + I2C_WRITE_UNIT_SIZE;

	}
	CDBG(" %s: END!\n", __func__);
	return rc;
}
#endif


#ifdef DUMP
static int mh1_dump_fw(struct msm_mh1_ctrl_t *m_ctrl)
{
	struct file *fp;
	uint8_t fs_name_buf1[256];

	mm_segment_t old_fs;
	unsigned char *buf/*, val*/;
	unsigned int addr, unit, count, intram_unit = 0x1000;
	int i, /*j,*/ err;

	CDBG("start\n");

	old_fs = get_fs();
	set_fs(KERNEL_DS);
	sprintf(fs_name_buf1, "/data/%s", "MH-1_dump_24.bin");

	fp = filp_open(fs_name_buf1, O_WRONLY|O_CREAT|O_EXCL, S_IRUGO|S_IWUGO|S_IXUSR); //O_EXCL(one time) or O_TRUNC(always)
	if (IS_ERR(fp)) {
		printk("%s: failed to open \n",__func__);
		set_fs(old_fs);
		return -1;
	}

	buf = kmalloc(intram_unit, GFP_KERNEL);
	if (!buf) {
		pr_err("failed to allocate memory\n");
		err = -ENOMEM;
		goto out;
	}

#if 1
	addr = 0x01000000;
	unit = 32;
	//	count = 1024*10;	// 1M
	count = 8153;		// 1/2M
	for (i = 0; i < count; i++) {
			err = mh1_mem_dump(m_ctrl->i2c_client, unit, addr + (i * unit), buf);
			//pr_err("dump ~~ count : %d\n", i);
			if (err < 0) {
				pr_err("i2c falied, err %d\n", err);
				goto out;
			}
			//pr_err("[mh1 %4d ] ############Read value %#x ###########3\n", __LINE__, *buf);
			vfs_write(fp, buf, unit, &fp->f_pos);
	}
#else
	for (i = 0; i < count; i++) {
			rc = mh1_read(m_ctrl->i2c_client, 1, 0x0F, 0x10, &val, true);

#endif

	CDBG("end\n");
out:
	if (buf)
		kfree(buf);

	if (!IS_ERR(fp))
		filp_close(fp, current->files);

	set_fs(old_fs);;

	return err;
}
#endif

int32_t msm_mh1_sensor_mode(struct msm_mh1_ctrl_t *m_ctrl, uint8_t sensor_mode, bool writing_register){
    int32_t rc = 0;

    if (msm_sensor_get_i2c_path() == false){
		pr_err("%s skip!! %d\n", __func__, __LINE__);
		return rc;
	}

    CDBG("MH1 - SENSOR mode : %d\n", sensor_mode);

	if(writing_register){
		//if need, write sensor register
		rc = mh1_write(m_ctrl->i2c_client, 1, 0x02, 0x02, 0x01, true);
	    if (rc < 0 ) {
			pr_err("%s failed %d\n", __func__, __LINE__);
	    }
	}
    rc = mh1_write(m_ctrl->i2c_client, 1, 0x02, 0x00, (int)sensor_mode, true);
    if (rc < 0 ) {
		pr_err("%s failed %d\n", __func__, __LINE__);
    }
    return rc;
}

int msm_mh1_start_stream(bool is_first){
	int rc = 0;
	int32_t err = 0;

	pr_err("%s: Enter\n", __func__);
	if (is_first){
	//set current resolution
		err = msm_mh1_sensor_mode(&g_mctrl, current_resolution, 0);
		if (err < 0) {
			pr_err("%s failed %d\n", __func__, __LINE__);
			return err;
		}
	}
	msleep(1);
	rc = mh1_write(g_mctrl.i2c_client, 1, 0x01, 0x00, 0x01, true);	//value size, category, register, value, log
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		return rc;
	}

	return rc;	
}

int msm_mh1_stop_stream(void){
	int rc = 0;
	pr_err("%s: Enter\n", __func__);

	msleep(1);
	rc = mh1_write(g_mctrl.i2c_client, 1, 0x01, 0x00, 0x00, true);	//value size, category, register, value, log
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		return rc;
	}

	return rc;	
}

EXPORT_SYMBOL(msm_mh1_proxy_i2c_e2p_read);
int msm_mh1_proxy_i2c_e2p_read(int32_t eeprom_addr, int16_t *value)
{
	int rc = 0;
	unsigned int val = 0;
	unsigned int loop_count = 0;
  
	pr_err("%s: Enter. \n", __func__);

	rc = mh1_write(g_mctrl.i2c_client, 2, 0x0D, 0x3E, eeprom_addr, false);	//value size, category, register, value, log
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		return rc;
	}

    rc = mh1_write(g_mctrl.i2c_client, 1, 0x0D, 0x3D, 0x01, false);	//value size, category, register, value, log
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		return rc;
	}

	while (loop_count < 10) {
		rc = mh1_read(g_mctrl.i2c_client, 1, 0x0D, 0x3d, &val, true);  //value size, category, register, address, log
		if (rc < 0) {
			pr_err("%s failed %d\n", __func__, __LINE__);
			return rc;
		}
		if (val == 3){
			pr_err("%s: eeprom read completed!!\n", __func__);
			break;
		}
		msleep(10);
		loop_count++;
	}

    rc = mh1_read(g_mctrl.i2c_client, 2, 0x0D, 0x40, (int *)value, true);  //value size, category, register, address, log
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		return rc;
	}
	pr_err("%s: exit!!\n", __func__);

	return rc;	
}

EXPORT_SYMBOL(msm_mh1_proxy_i2c_e2p_write);
int msm_mh1_proxy_i2c_e2p_write(int32_t eeprom_addr, int16_t value)
{
	int rc = 0;
	unsigned int val = 0;
	unsigned int loop_count = 0;
  
	pr_err("%s: Enter. value : 0x%X\n", __func__, value);

	rc = mh1_write(g_mctrl.i2c_client, 2, 0x0D, 0x3E, eeprom_addr, false);	//value size, category, register, value, log
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		return rc;
	}

	rc = mh1_write(g_mctrl.i2c_client, 2, 0x0D, 0x40, value, true);	//value size, category, register, value, log
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		return rc;
	}

    rc = mh1_write(g_mctrl.i2c_client, 1, 0x0D, 0x3D, 0x02, false);	//value size, category, register, value, log
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		return rc;
	}

	while (loop_count < 10) {
		rc = mh1_read(g_mctrl.i2c_client, 1, 0x0D, 0x3d, &val, true);  //value size, category, register, address, log
		if (rc < 0) {
			pr_err("%s failed %d\n", __func__, __LINE__);
			return rc;
		}
		if (val == 3){
			pr_err("%s: eeprom write completed!!\n", __func__);
			break;
		}
		msleep(10);
		loop_count++;
	}
	pr_err("%s: exit!!\n", __func__);

	return rc;	
}

EXPORT_SYMBOL(msm_mh1_ois_enable);
int msm_mh1_ois_enable(void)
{
	int rc = 0;
  
	pr_err("%s: Enter.\n", __func__);
  
	rc = mh1_write(g_mctrl.i2c_client, 1, 0x0B, 0x00, 0x07, false);	//value size, category, register, value, log
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		return rc;
	}
	return rc;	
}

EXPORT_SYMBOL(msm_mh1_af_vcm_code);
int msm_mh1_af_vcm_code(int16_t UsVcmCod)
{
	int rc = 0;
    uint8_t high_vcm_code, low_vcm_code = 0;
  
	pr_err("%s: Enter. UsVcmCod : 0x%X\n", __func__, UsVcmCod);

    high_vcm_code = (UsVcmCod >> 8) & 0xFF;
    low_vcm_code = (UsVcmCod) & 0xFF;
  
	rc = mh1_write(g_mctrl.i2c_client, 1, 0x0B, 0x04, high_vcm_code, false);	//value size, category, register, value, log
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		return rc;
	}

    rc = mh1_write(g_mctrl.i2c_client, 1, 0x0B, 0x05, low_vcm_code, false);	//value size, category, register, value, log
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		return rc;
	}  

    rc = mh1_write(g_mctrl.i2c_client, 1, 0x0B, 0x00, 3, false);	//value size, category, register, value, log
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		return rc;
	} 
	return rc;	
}

EXPORT_SYMBOL(msm_mh1_ois_mode);
int msm_mh1_ois_mode(uint8_t set_mode)
{
	int rc = 0;
	pr_err("%s: Enter\n", __func__);

	switch (set_mode) {
    /* still */
	case 0:
		CDBG("%s:OIS_MODE_STILL %d\n", __func__, set_mode);
		set_mode = 1;
		break;
   /* video */
	case 1:
		CDBG("%s:OIS_MODE_VIDEO %d\n", __func__, set_mode);
		set_mode = 2;
		break;
	}
    
	rc = mh1_write(g_mctrl.i2c_client, 1, 0x0B, 0x00, set_mode, true);	//value size, category, register, value, log
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		return rc;
	}
	return rc;	
}

#ifdef ISR_USE
static irqreturn_t mh1_spi_isr(int irq, void *handle)
{
    pr_err("%s: spi isr triggering\n", __func__);

	g_mctrl.issued = 1;
	wake_up_interruptible(&g_mctrl.wait);

    return IRQ_HANDLED;
}
#endif

int32_t msm_init_mh1(struct msm_mh1_ctrl_t *m_ctrl){
	int rc = 0;
	unsigned int val;

#ifdef I2C_DOWNLOAD
	int fd1;
	uint8_t fs_name_buf1[256];
	uint32_t cur_fd_pos;

	mm_segment_t old_fs = get_fs();

	uint8_t *buf_mh1 = NULL;
	uint32_t fsize = BIN_SIZE;
	unsigned int count = 0;
#endif	
	CDBG("%s called\n", __func__);

	//3. create ISR and wait
#ifdef ISR_USE
	if(!g_mctrl.issued){ //gpio_get_value(mh1_irq_gpio)){
		CDBG("%s wait interrupt START!! (line%d)\n", __func__, __LINE__);
		if (wait_event_interruptible_timeout(g_mctrl.wait,
			g_mctrl.issued == 1,
			msecs_to_jiffies(1000)) == 0) {
			pr_err("%s:wait interrupt timeout!!!\n", __func__);
			return 0;
		}else{
			g_mctrl.issued = 0;
			CDBG("%s wait interrupt END!! (line%d)\n", __func__, __LINE__);
		}
	}else{
		g_mctrl.issued = 0;
		CDBG("%s mh1_irq_gpio is already enabled!!! (line%d)\n", __func__, __LINE__);
	}
#else
	msleep(600);   //600ms
#endif
	//4. clear interrupt
	rc = mh1_read(m_ctrl->i2c_client, 1, 0x0F, 0x10, &val, true);  //value size, category, register, address, log
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		return rc;
	}
	
	//5. Set PLL Value
	rc = mh1_write(m_ctrl->i2c_client, 4, 0x0F, 0x1C, 0x012f0257, true);	//value size, category, register, value, log
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		return rc;
	}
#ifndef I2C_DOWNLOAD
	//6. set SIO latch timing --> if i2c, skip
	rc = mh1_write(m_ctrl->i2c_client, 1, 0x0F, 0x4B, 0x4c, true);	//value size, category, register, value, log
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		return rc;
	}
#endif

	//7. prepare SIO transfer
	rc = mh1_write(m_ctrl->i2c_client, 1, 0x0F, 0x4A, 0x03, true);	//value size, category, register, value, log
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		return rc;
	}
	//8.wait
#ifdef ISR_USE
	if(!g_mctrl.issued){
		CDBG("%s wait interrupt START!! (line%d)\n", __func__, __LINE__);
		if (wait_event_interruptible_timeout(g_mctrl.wait,
			g_mctrl.issued == 1,
			msecs_to_jiffies(1000)) == 0) {
			pr_err("%s:wait interrupt timeout!!!\n", __func__);
			return 0;
		}else{
			g_mctrl.issued = 0;
			CDBG("%s wait interrupt END!! (line%d)\n", __func__, __LINE__);
		}
	}else{
		g_mctrl.issued = 0;
		CDBG("%s mh1_irq_gpio is already enabled!!! (line%d)\n", __func__, __LINE__);
	}
#else
	msleep(10);   //600ms
#endif
	//9.clear interrupt
	rc = mh1_read(m_ctrl->i2c_client, 1, 0x0F, 0x10, &val, true);  //value size, category, register, address, log
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		return rc;
	}

	//10. download bin
#ifdef I2C_DOWNLOAD
	set_fs(KERNEL_DS);

	sprintf(fs_name_buf1, "/system/media/%s", "RS_MH1.bin");

	fd1 = sys_open(fs_name_buf1, O_RDONLY, 0);
	if (fd1 < 0) {
		printk("%s: File not exist \n",__func__);
		set_fs(old_fs);
		return -1;
	}

	if ((unsigned)sys_lseek(fd1, (off_t)0, 2) != fsize) {
		printk("%s: File size error \n",__func__);
		sys_close(fd1);
		set_fs(old_fs);
		return -1;
	}

	cur_fd_pos = (unsigned)sys_lseek(fd1, (off_t)0, 0);

	buf_mh1 = kmalloc(fsize, GFP_KERNEL);

	memset(buf_mh1, 0x00, fsize);

	if ((unsigned)sys_read(fd1, (char __user *)buf_mh1, fsize) != fsize) {
		printk("%s: File read error \n",__func__);
		sys_close(fd1);
		set_fs(old_fs);
		return -1;
	}
	sys_close(fd1);
	set_fs(old_fs);

	count = fsize / I2C_WRITE_UNIT_SIZE;
	
	rc = mh1_program_fw(m_ctrl, buf_mh1, count);
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		sys_close(fd1);
		set_fs(old_fs);
		return rc;
	}

	msleep(600);

#else //spi download
	pr_err("%s calling mh1_fetch-image %d\n", __func__, __LINE__);

#ifdef FW_PUSH
	rc = mh1_fetch_image_from_sd(mh1_spich);
#else
	rc = mh1_fetch_image(mh1_spich);
#endif
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		return rc;
	}
#endif
#ifdef DUMP
//check dump
    mh1_dump_fw(m_ctrl);
#endif

#ifdef TRF_SHD_TBL

	//11. Set SIO transfer address
	rc = mh1_write(m_ctrl->i2c_client, 4, 0x0F, 0x14, 0x4004A000, true);	//value size, category, register, value, log
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		return rc;
	}

	//12. Update Address
	rc = mh1_write(m_ctrl->i2c_client, 1, 0x0F, 0x4A, 0x03, true);	//value size, category, register, value, log
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		return rc;
	}

	//13. transfer shd table
	rc = msm_mh1_set_shading_tbl(m_ctrl, shading_table, sizeof(uint8_t *)*6*1040*2);
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		return rc;
	}
	//14. wait
#ifdef ISR_USE
	if(!g_mctrl.issued){
		CDBG("%s wait interrupt START!! (line%d)\n", __func__, __LINE__);
		if (wait_event_interruptible_timeout(g_mctrl.wait,
			g_mctrl.issued == 1,
			msecs_to_jiffies(1000)) == 0) {
			pr_err("%s:wait interrupt timeout!!!\n", __func__);
			return 0;
		}else{
			g_mctrl.issued = 0;
			CDBG("%s wait interrupt END!! (line%d)\n", __func__, __LINE__);
		}
		//free irq
		//free_irq(g_mctrl.irq, NULL);
	}else{
		g_mctrl.issued = 0;
		CDBG("%s mh1_irq_gpio is already enabled!!! (line%d)\n", __func__, __LINE__);
	}
#else
	msleep(600);
#endif
	//15. clear interrupt
	rc = mh1_read(m_ctrl->i2c_client, 1, 0x0F, 0x10, &val, true);  //value size, category, register, address, log
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		return rc;
	}
#endif
    //16. Jump to start address of RS_MH1.bin
	rc = mh1_write(m_ctrl->i2c_client, 1, 0x0F, 0x12, 0x02, true);	//value size, category, register, value, log
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		return rc;
	}

	//17. wait
#ifdef ISR_USE
	if(!g_mctrl.issued){
		CDBG("%s wait interrupt START!! (line%d)\n", __func__, __LINE__);
		if (wait_event_interruptible_timeout(g_mctrl.wait,
			g_mctrl.issued == 1,
			msecs_to_jiffies(1000)) == 0) {
			pr_err("%s:wait interrupt timeout!!!\n", __func__);
			
			return 0;
		}else{
			g_mctrl.issued = 0;
			CDBG("%s wait interrupt END!! (line%d)\n", __func__, __LINE__);
		}
		//free irq
		//free_irq(g_mctrl.irq, NULL);
	}else{
		g_mctrl.issued = 0;
		CDBG("%s mh1_irq_gpio is already enabled!!! (line%d)\n", __func__, __LINE__);
	}
#else
	msleep(600);
#endif
	//13. clear interrupt
	rc = mh1_read(m_ctrl->i2c_client, 1, 0x00, 0x1C, &val, true);  //value size, category, register, address, log
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		return rc;
	}

	//13. test
	rc = mh1_read(m_ctrl->i2c_client, 1, 0x03, 0x02, &val, true);  //value size, category, register, address, log
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		return rc;
	}
		rc = mh1_read(m_ctrl->i2c_client, 1, 0x03, 0x03, &val, true);  //value size, category, register, address, log
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		return rc;
	}
	
	 //14. meta on/off
#if 0
	rc = mh1_write(m_ctrl->i2c_client, 1, 0x01, 0x11, 0x1, true);	//value size, category, register, value, log
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		return rc;
	}
#endif
   
#ifndef TRF_SHD_TBL
//shading off
        rc = mh1_write(m_ctrl->i2c_client, 1, 0x07, 0x00, 0x00, true);
        if (rc < 0 ) {
            pr_err("%s failed %d\n", __func__, __LINE__);
        }
#endif

        // test. HDR on 1/4
#if 0
        rc = mh1_write(m_ctrl->i2c_client, 1, 0x04, 0x35, 0x02, true);
        if (rc < 0 ) {
        	pr_err("%s failed %d\n", __func__, __LINE__);
        }
        
        rc = mh1_write(m_ctrl->i2c_client, 1, 0x04, 0x28, 0x13, true);
        if (rc < 0 ) {
        	pr_err("%s failed %d\n", __func__, __LINE__);
        }
#endif

#ifdef LOG_SAVE
	if (log_cnt > 20)
		log_cnt = 0;
	mh1_save_log(m_ctrl);
	log_cnt++;	
#endif
	CDBG("Exit\n");

	return rc;
}

int32_t msm_mh1_set_current_resolution(struct msm_mh1_ctrl_t *m_ctrl, uint8_t cur_res){

    CDBG("%s: MH1 - current_resolution : %d\n", __func__, cur_res);

    current_resolution = cur_res;
	return 0;
}

int32_t msm_mh1_hdr_mode(struct msm_mh1_ctrl_t *m_ctrl, uint8_t hdr_mode){
    int32_t rc = 0;

    if (msm_sensor_get_i2c_path() == false){
		pr_err("%s skip!! %d\n", __func__, __LINE__);
		return rc;
	}

    pr_err("MH1 - HDR mode : %d\n", hdr_mode);

    switch (hdr_mode) {   // 0 : off,  1,2,4,8,16 : on (1:1, 1:2, 1:4, 1:8, 1:16)
        case 1:
            rc = mh1_write(m_ctrl->i2c_client, 1, 0x04, 0x35, 0x10, false);
            if (rc < 0 ) {
            	pr_err("%s failed %d\n", __func__, __LINE__);
            }
            
            rc = mh1_write(m_ctrl->i2c_client, 1, 0x04, 0x28, 0x11, false);
            if (rc < 0 ) {
            	pr_err("%s failed %d\n", __func__, __LINE__);
            }
            break;
        case 2:
            rc = mh1_write(m_ctrl->i2c_client, 1, 0x04, 0x35, 0x03, false);
            if (rc < 0 ) {
            	pr_err("%s failed %d\n", __func__, __LINE__);
            }
            
            rc = mh1_write(m_ctrl->i2c_client, 1, 0x04, 0x28, 0x12, false);
            if (rc < 0 ) {
            	pr_err("%s failed %d\n", __func__, __LINE__);
            }
            break;
        case 4:
            rc = mh1_write(m_ctrl->i2c_client, 1, 0x04, 0x35, 0x02, false);
            if (rc < 0 ) {
            	pr_err("%s failed %d\n", __func__, __LINE__);
            }
            
            rc = mh1_write(m_ctrl->i2c_client, 1, 0x04, 0x28, 0x13, false);
            if (rc < 0 ) {
            	pr_err("%s failed %d\n", __func__, __LINE__);
            }
            break;
        case 8:
            rc = mh1_write(m_ctrl->i2c_client, 1, 0x04, 0x35, 0x01, false);
            if (rc < 0 ) {
            	pr_err("%s failed %d\n", __func__, __LINE__);
            }
            
            rc = mh1_write(m_ctrl->i2c_client, 1, 0x04, 0x28, 0x14, false);
            if (rc < 0 ) {
            	pr_err("%s failed %d\n", __func__, __LINE__);
            }
            break;
        case 16:
            rc = mh1_write(m_ctrl->i2c_client, 1, 0x04, 0x35, 0x00, false);
            if (rc < 0 ) {
            	pr_err("%s failed %d\n", __func__, __LINE__);
            }
            
            rc = mh1_write(m_ctrl->i2c_client, 1, 0x04, 0x28, 0x15, false);
            if (rc < 0 ) {
            	pr_err("%s failed %d\n", __func__, __LINE__);
            }
            break;
        case 0:
        default:   
            rc = mh1_write(m_ctrl->i2c_client, 1, 0x04, 0x28, 0x00, false);
            if (rc < 0 ) {
            	pr_err("%s failed %d\n", __func__, __LINE__);
            }
            break;
    }
 
    return rc;
}

#if 0
int32_t msm_mh1_aec_update(struct msm_mh1_ctrl_t *m_ctrl, void *data) {
    int32_t rc = 0;

    uint8_t *ae_data = (uint8_t *)data;
#if 1
	if (msm_sensor_get_i2c_path() == false){
		pr_err("%s skip!! %d\n", __func__, __LINE__);
		return rc;
	}
#endif
    pr_err("debug - Entet msm_mh1_aec_update : gain %x %x\n", *(ae_data+2), *(ae_data+3));
	
    rc = mh1_write(m_ctrl->i2c_client, 1, 0x0C, 0x00, 0x1, true);	//value size, category, register, value, log
    if (rc < 0) {
        pr_err("%s failed %d\n", __func__, __LINE__);
        return rc;
    }
    rc = mh1_write(m_ctrl->i2c_client, 1, 0x0C, 0x02, (int)*ae_data, true);	//value size, category, register, value, log
    if (rc < 0) {
        pr_err("%s failed %d\n", __func__, __LINE__);
        return rc;
    }
    rc = mh1_write(m_ctrl->i2c_client, 1, 0x0C, 0x03, (int)*(ae_data+1), true);	//value size, category, register, value, log
    if (rc < 0) {
        pr_err("%s failed %d\n", __func__, __LINE__);
        return rc;
    }
    rc = mh1_write(m_ctrl->i2c_client, 1, 0x0C, 0x04, (int)*(ae_data+2), true);	//value size, category, register, value, log
    if (rc < 0) {
        pr_err("%s failed %d\n", __func__, __LINE__);
        return rc;
    }
    rc = mh1_write(m_ctrl->i2c_client, 1, 0x0C, 0x05, (int)*(ae_data+3), true);	//value size, category, register, value, log
    if (rc < 0) {
        pr_err("%s failed %d\n", __func__, __LINE__);
        return rc;
    }
    rc = mh1_write(m_ctrl->i2c_client, 1, 0x0C, 0x06, (int)*(ae_data+4), true);	//value size, category, register, value, log
    if (rc < 0) {
        pr_err("%s failed %d\n", __func__, __LINE__);
        return rc;
    }
    rc = mh1_write(m_ctrl->i2c_client, 1, 0x0C, 0x07, (int)*(ae_data+5), true);	//value size, category, register, value, log
    if (rc < 0) {
        pr_err("%s failed %d\n", __func__, __LINE__);
        return rc;
    }
    rc = mh1_write(m_ctrl->i2c_client, 1, 0x0C, 0x08, (int)*(ae_data+6), true);	//value size, category, register, value, log
    if (rc < 0) {
        pr_err("%s failed %d\n", __func__, __LINE__);
        return rc;
    }
    rc = mh1_write(m_ctrl->i2c_client, 1, 0x0C, 0x09, (int)*(ae_data+7), true);	//value size, category, register, value, log
    if (rc < 0) {
        pr_err("%s failed %d\n", __func__, __LINE__);
        return rc;
    }
    rc = mh1_write(m_ctrl->i2c_client, 1, 0x0C, 0x0A, (int)*(ae_data+8), true);	//value size, category, register, value, log
    if (rc < 0) {
        pr_err("%s failed %d\n", __func__, __LINE__);
        return rc;
    }
    rc = mh1_write(m_ctrl->i2c_client, 1, 0x0C, 0x0B, (int)*(ae_data+9), true);	//value size, category, register, value, log
    if (rc < 0) {
        pr_err("%s failed %d\n", __func__, __LINE__);
        return rc;
    }
    rc = mh1_write(m_ctrl->i2c_client, 1, 0x0C, 0x00, 0x0, true);	//value size, category, register, value, log
    if (rc < 0) {
        pr_err("%s failed %d\n", __func__, __LINE__);
        return rc;
    }
    return 0;
}
#else
int32_t msm_mh1_aec_update(struct msm_mh1_ctrl_t *m_ctrl, void *data) {
    int32_t rc = 0;

	uint8_t *ae_data = (uint8_t *)data;
#if 1
	if (msm_sensor_get_i2c_path() == false){
		pr_err("%s skip!! %d\n", __func__, __LINE__);
		return rc;
	}
#endif
    pr_err("debug - Entet msm_mh1_aec_update : gain %x %x\n", *(ae_data+2), *(ae_data+3));
	
    rc = mh1_write(m_ctrl->i2c_client, 1, 0x0C, 0x00, 0x1, false);	//value size, category, register, value, log
    if (rc < 0) {
        pr_err("%s failed %d\n", __func__, __LINE__);
        return rc;
    }
    rc = mh1_burst_write(m_ctrl->i2c_client, 10, 0x0C, 0x02, ae_data);	//value size, category, register, value, log
    if (rc < 0) {
        pr_err("%s failed %d\n", __func__, __LINE__);
        return rc;
    }
   
    rc = mh1_write(m_ctrl->i2c_client, 1, 0x0C, 0x00, 0x0, false);	//value size, category, register, value, log
    if (rc < 0) {
        pr_err("%s failed %d\n", __func__, __LINE__);
        return rc;
    }
    return rc;
}

#endif

int32_t msm_mh1_awb_update(struct msm_mh1_ctrl_t *m_ctrl, void *data) {
    int32_t rc = 0;
    struct msm_mh1_awb_info_t *awb_data = (struct msm_mh1_awb_info_t *)data;

	if (msm_sensor_get_i2c_path() == false){
		pr_err("%s skip!! %d\n", __func__, __LINE__);
		return rc;
	}
    pr_err("%s r: %u, b: %u, cct:%u\n", __func__,awb_data->awb_gain_r, awb_data->awb_gain_b, awb_data->awb_cct);

    rc = mh1_write(m_ctrl->i2c_client, 2, 0x04, 0x2E, (int)(awb_data->awb_gain_r), false);
    if (rc < 0) {
        pr_err("%s failed %d\n", __func__, __LINE__);
        return rc;
    }
    rc = mh1_write(m_ctrl->i2c_client, 2, 0x04, 0x32, (int)(awb_data->awb_gain_b), false);
    if (rc < 0) {
        pr_err("%s failed %d\n", __func__, __LINE__);
        return rc;
    }
    rc = mh1_write(m_ctrl->i2c_client, 2, 0x07, 0x06, (int)(awb_data->awb_cct), false);
    if (rc < 0) {
        pr_err("%s failed %d\n", __func__, __LINE__);
        return rc;
    }

    return rc;
}

int32_t msm_mh1_set_exp_gain(struct msm_mh1_ctrl_t *m_ctrl, void *data) {

	int32_t rc = 0;
	struct msm_mh1_exp_info_t* exp_info = (struct msm_mh1_exp_info_t *)data;
	pr_err("%s exp time %u gain %u lux %u flash %u\n", __func__, exp_info->exp_time , exp_info->gain, exp_info->lux_index,exp_info->flash_rate);

	rc = mh1_write(m_ctrl->i2c_client, 4, 0x02, 0x08, (int)(exp_info->exp_time), false);
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		return rc;
	}

	rc = mh1_write(m_ctrl->i2c_client, 4, 0x02, 0xC, (int)(exp_info->gain), false);
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		return rc;
	}

	rc = mh1_write(m_ctrl->i2c_client, 2, 0x07, 0x04, (int)(exp_info->lux_index), false);
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		return rc;
	}

	rc = mh1_write(m_ctrl->i2c_client, 2, 0x07, 0x8, (int)(exp_info->flash_rate), false);
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		return rc;
	}
    return rc;
}

#ifdef TRF_SHD_TBL
int msm_mh1_set_shading_tbl(struct msm_mh1_ctrl_t *m_ctrl, uint8_t *shading_tbl, int size ){
  int rc = 0;
  
  rc = mh1_fetch_shading_tbl(mh1_spich, shading_tbl, size);
  if (rc < 0) {
	  pr_err("%s failed %d\n", __func__, __LINE__);
	  return rc;
  }

 CDBG("Exit\n");
 return rc;
}
#endif
static int32_t msm_mh1_config(struct msm_mh1_ctrl_t *mh1_ctrl,
	void __user *argp)
{
	struct msm_mh1_cfg_data *cdata =
		(struct msm_mh1_cfg_data *)argp;
	int32_t rc = 0;
	mutex_lock(mh1_ctrl->mh1_mutex);

	CDBG("%s type %d\n", __func__, cdata->cfgtype);
	switch (cdata->cfgtype) {
        case CFG_MH1_INIT:
            rc = msm_init_mh1(mh1_ctrl);
			rc = 0; //always return 0 because fail of first download 
        	break;
		case CFG_MH1_SET_CURRENT_RES:
			rc = msm_mh1_set_current_resolution(mh1_ctrl, cdata->cfg.cur_res);
			break;
        case CFG_MH1_SENSOR_MODE:
            rc = msm_mh1_sensor_mode(mh1_ctrl, cdata->cfg.sensor_mode, 1);
			break;
        case CFG_MH1_HDR_MODE:
			rc = msm_mh1_hdr_mode(mh1_ctrl, cdata->cfg.hdr_mode);
			break;
#ifdef TRF_SHD_TBL
		case CFG_MH1_SET_SHADING_TBL:
			shading_table = (uint8_t *)kmalloc(sizeof(uint8_t)*6*1040*2,GFP_KERNEL);
			if (shading_table == NULL) {
				pr_err("Error allocating memory\n");
				return -EFAULT;
			}
			if (copy_from_user(shading_table,(uint8_t *)cdata->cfg.shading_tbl, sizeof(shading_table))) {
				kfree(shading_table);
				pr_err("Error copying\n");
				return -EFAULT;
			}
			break;
#endif
        case CFG_MH1_AEC_UPDATE :
            rc = msm_mh1_aec_update(mh1_ctrl, cdata->cfg.set_info.setting);
            break;
        case CFG_MH1_AWB_UPDATE:
            rc = msm_mh1_awb_update(mh1_ctrl, cdata->cfg.set_info.setting);
            break;
        case CFG_MH1_SENSOR_EXP_GAIN:
            rc = msm_mh1_set_exp_gain(mh1_ctrl, cdata->cfg.set_info.setting);
            break;

	default:
		break;
	}
	mutex_unlock(mh1_ctrl->mh1_mutex);
	CDBG("Exit\n");
	return rc;
}

static long msm_mh1_subdev_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct msm_mh1_ctrl_t *m_ctrl = v4l2_get_subdevdata(sd);
	void __user *argp = (void __user *)arg;
	CDBG("%s:%d m_ctrl %p argp %p\n", __func__, __LINE__, m_ctrl, argp);
	switch (cmd) {
	case VIDIOC_MSM_SENSOR_GET_SUBDEV_ID:
		return msm_mh1_get_subdev_id(m_ctrl, argp);
  case VIDIOC_MSM_MH1_CFG:
		return msm_mh1_config(m_ctrl, argp);
    break;
	default:
		return -ENOIOCTLCMD;
	}
}

#ifdef CONFIG_COMPAT
static long msm_mh1_subdev_do_ioctl(
	struct file *file, unsigned int cmd, void *arg)
{
	struct video_device *vdev = video_devdata(file);
	struct v4l2_subdev *sd = vdev_to_v4l2_subdev(vdev);
	struct msm_mh1_cfg_data32 *u32 =
		(struct msm_mh1_cfg_data32 *)arg;
	struct msm_mh1_cfg_data mh1_data;
	void __user *parg = (void __user *)arg;

	CDBG("Enter\n");
	switch (cmd) {
		case VIDIOC_MSM_MH1_CFG32:
			cmd = VIDIOC_MSM_MH1_CFG;
			switch (u32->cfgtype) {
		      case CFG_MH1_INIT:
				break;
			  case CFG_MH1_SET_CURRENT_RES:
				mh1_data.cfgtype = u32->cfgtype;
				mh1_data.cfg.cur_res = u32->cfg.cur_res;
				parg = &mh1_data;
				break;
			  case CFG_MH1_SENSOR_MODE:
				mh1_data.cfgtype = u32->cfgtype;
				mh1_data.cfg.sensor_mode = u32->cfg.sensor_mode;
				parg = &mh1_data;
				break;
			  case CFG_MH1_HDR_MODE:
				mh1_data.cfgtype = u32->cfgtype;
				mh1_data.cfg.hdr_mode = u32->cfg.hdr_mode;
				parg = &mh1_data;
				break;
			  case CFG_MH1_AEC_UPDATE:
				mh1_data.cfgtype = u32->cfgtype;
				if (copy_from_user(mh1_settings,compat_ptr(u32->cfg.set_info.setting), sizeof(uint8_t)*10)) {
					pr_err("Error copying\n");
					break;
				}
				mh1_data.cfg.set_info.setting = (void*)mh1_settings;
				parg = &mh1_data;
				break;

			case CFG_MH1_AWB_UPDATE:
				mh1_data.cfgtype = u32->cfgtype;
				if (copy_from_user(mh1_settings,compat_ptr(u32->cfg.set_info.setting), sizeof(uint8_t)*6)) {
					pr_err("Error copying\n");
					break;
				}
				mh1_data.cfg.set_info.setting = (void*)mh1_settings;
				parg = &mh1_data;
				break;
			case CFG_MH1_SENSOR_EXP_GAIN:
				mh1_data.cfgtype = u32->cfgtype;
				if (copy_from_user(mh1_settings,compat_ptr(u32->cfg.set_info.setting), sizeof(uint8_t)*12)) {
					pr_err("Error copying\n");
					break;
				}
				mh1_data.cfg.set_info.setting = (void*)mh1_settings;
				parg = &mh1_data;
				break;
#ifdef TRF_SHD_TBL
			case CFG_MH1_SET_SHADING_TBL:
				mh1_data.cfgtype = u32->cfgtype;
				mh1_data.cfg.shading_tbl = compat_ptr(u32->cfg.shading_tbl);
				parg = &mh1_data;
				break;
#endif
		      default:
		        mh1_data.cfgtype = u32->cfgtype;
		        parg = &mh1_data;
		        break;
			}
	}
	return msm_mh1_subdev_ioctl(sd, cmd, parg);
}

static long msm_mh1_subdev_fops_ioctl(struct file *file, unsigned int cmd,
	unsigned long arg)
{
	return video_usercopy(file, cmd, arg, msm_mh1_subdev_do_ioctl);
}
#endif

static struct v4l2_subdev_core_ops msm_mh1_subdev_core_ops = {
	.ioctl = msm_mh1_subdev_ioctl,
};

static struct v4l2_subdev_ops msm_mh1_subdev_ops = {
	.core = &msm_mh1_subdev_core_ops,
};

static const struct v4l2_subdev_internal_ops msm_mh1_internal_ops;

static struct msm_camera_i2c_client mh1_client = {
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
};

static int32_t msm_mh1_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int rc = 0;
	uint32_t temp;
	struct msm_mh1_ctrl_t *mh1_ctrl_t = NULL;

	CDBG("%s: Enter\n", __func__);
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("i2c_check_functionality failed\n");
		kfree(mh1_ctrl_t);
		return -EINVAL;
	}
	
	mh1_ctrl_t = (struct msm_mh1_ctrl_t *)(id->driver_data);
	
	if (mh1_ctrl_t->i2c_client){
		mh1_ctrl_t->i2c_client->client = client;
	}else{
		pr_err("%s:%d mh1 i2c client NULL\n", __func__,
			__LINE__);
		kfree(mh1_ctrl_t);
		return -EINVAL;
	}

	mh1_ctrl_t->mh1_device_type = MSM_CAMERA_I2C_DEVICE;

	rc = of_property_read_u32(client->dev.of_node, "cell-index",&mh1_ctrl_t->subdev_id);
		
	CDBG("cell-index %d, rc %d\n", mh1_ctrl_t->subdev_id, rc);
	if (rc < 0) {
		pr_err("failed rc %d\n", rc);
		return rc;
	}
	
	if (mh1_ctrl_t->i2c_client != NULL) {
		mh1_ctrl_t->i2c_client->client = client;
		rc = of_property_read_u32(client->dev.of_node, "qcom,slave-addr", &temp);
		if (rc < 0) {
			pr_err("%s failed rc %d\n", __func__, rc);
			kfree(mh1_ctrl_t);
			return rc;
		}
		mh1_ctrl_t->i2c_client->client->addr = temp;
	} else {
		pr_err("%s %s sensor_i2c_client NULL\n",
			__func__, client->name);
		rc = -EFAULT;
		kfree(mh1_ctrl_t);
		return rc;
	}

	if (!mh1_ctrl_t->i2c_client->i2c_func_tbl)
		mh1_ctrl_t->i2c_client->i2c_func_tbl = &msm_sensor_qup_func_tbl;

#ifdef ISR_USE
	//make isr
	init_waitqueue_head(&g_mctrl.wait);

	//mh1_irq_gpio = of_get_named_gpio_flags(client->dev.of_node, "mh1_irq", 0, NULL);
	mh1_irq_gpio = of_get_named_gpio(client->dev.of_node, "mh1_irq", 0);
	 CDBG("%s: mh1_irq_gpio : %d\n", __func__, mh1_irq_gpio);

     rc = gpio_request(mh1_irq_gpio, "mh1 irq");
	 if (rc < 0){
	 	pr_err("%s gpio_request failed. rc: %d!!\n", __func__, rc);
  		return -ENODEV;
  	 }

	 rc = gpio_direction_input(mh1_irq_gpio);
	 if (rc < 0) {
		 pr_err("%s failed rc %d\n", __func__, rc);
	 }
	 
	 g_mctrl.irq = gpio_to_irq(mh1_irq_gpio);

	 rc = request_irq(g_mctrl.irq, mh1_spi_isr, IRQF_TRIGGER_RISING, "mh1_irq", NULL);
	 if (rc < 0) {
		 pr_err("%s failed rc %d, line: %d.\n", __func__, rc, __LINE__);
		 return rc;
	 }
	 g_mctrl.issued = 0;
	 CDBG("%s make ISR OK %d\n", __func__, __LINE__);
#endif
	
	/* Initialize sub device */
	v4l2_i2c_subdev_init(&mh1_ctrl_t->msm_sd.sd, mh1_ctrl_t->i2c_client->client, &msm_mh1_subdev_ops);
	mh1_ctrl_t->mh1_mutex = &msm_mh1_mutex;

	v4l2_set_subdevdata(&mh1_ctrl_t->msm_sd.sd, mh1_ctrl_t);

	mh1_ctrl_t->msm_sd.sd.internal_ops = &msm_mh1_internal_ops;
	mh1_ctrl_t->msm_sd.sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;

	snprintf(mh1_ctrl_t->msm_sd.sd.name, ARRAY_SIZE(mh1_ctrl_t->msm_sd.sd.name),
			"mh1");

	media_entity_init(&mh1_ctrl_t->msm_sd.sd.entity, 0, NULL, 0);
	mh1_ctrl_t->msm_sd.sd.entity.type = MEDIA_ENT_T_V4L2_SUBDEV;
	mh1_ctrl_t->msm_sd.sd.entity.group_id = MSM_CAMERA_SUBDEV_MH1;
	mh1_ctrl_t->msm_sd.close_seq = MSM_SD_CLOSE_2ND_CATEGORY | 0x2;

	rc = msm_sd_register(&mh1_ctrl_t->msm_sd);
	if (rc < 0) {
		pr_err("%s failed rc %d\n", __func__, rc);
	}

	msm_mh1_v4l2_subdev_fops = v4l2_subdev_fops;
#ifdef CONFIG_COMPAT
	msm_mh1_v4l2_subdev_fops.compat_ioctl32 =
		msm_mh1_subdev_fops_ioctl;
#endif
	mh1_ctrl_t->msm_sd.sd.devnode->fops =
		&msm_mh1_v4l2_subdev_fops;

	CDBG("%s, succeeded\n", __func__);

	return rc;
}

static struct msm_mh1_ctrl_t g_mctrl = {
	.i2c_client = &mh1_client,
};

MODULE_DEVICE_TABLE(of, msm_mh1_dt_match);

static struct i2c_driver msm_mh1_driver = {
	.id_table = msm_mh1_id,
	.probe  = msm_mh1_probe,
	.driver = {
		.name = "mh1",
	},
};

static int __init msm_mh1_init_module(void)
{
	CDBG("%s: Enter\n", __func__);
	return i2c_add_driver(&msm_mh1_driver);
}

static void __exit msm_mh1_exit_module(void)
{
	i2c_del_driver(&msm_mh1_driver);
	return;
}

module_init(msm_mh1_init_module);
module_exit(msm_mh1_exit_module);
MODULE_DESCRIPTION("MSM MH1");
MODULE_LICENSE("GPL v2");
