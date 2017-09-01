/*
 * spi_common.c - spi common  driver
 *
 * Copyright (c) 2011,2012, LG Electronics.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#define DEBUG

#include <linux/gpio.h>
#include <linux/interrupt.h>

#include <linux/clk.h>
#include <linux/clk/msm-clk.h>
#include <linux/firmware.h>
#include <linux/syscalls.h>
#include <linux/io.h>

#include "spi_common.h"
#include "stm_driver.h"

#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
	const bool target_little_endian = true;
#else
	#warning BE target not tested!
	const bool target_little_endian = false;
#endif


#if defined(CONFIG_SPI_MH1)
/* -------------------------------------------------------------------- */
/* mh1 data types							*/
/* -------------------------------------------------------------------- */
const char *mh1_inbuilt_fw_name_list;
struct spich_data *mh1_spich;
/* -------------------------------------------------------------------- */
/* mh1 driver constants						*/
/* -------------------------------------------------------------------- */



/* -------------------------------------------------------------------- */
/* function prototypes							*/
/* -------------------------------------------------------------------- */

/* -------------------------------------------------------------------- */
/* function definitions							*/
/* -------------------------------------------------------------------- */
int mh1_spi_read_set(struct spich_data *spich)
{
	int error = 0;

	//Disable STM
	gpio_set_value(spich->gpio_cs2,1);

	spich->spi->mode = SPI_MODE_1 | SPI_CS_HIGH ;
	spich->spi->bits_per_word = 8;
	spich->spi->chip_select = 0;

	error = spi_setup(spich->spi);

	return error;
}


int mh1_spi_write_set(struct spich_data *spich)
{
	int error = 0;

	//Disable STM
	gpio_set_value(spich->gpio_cs2,1);

	spich->spi->mode = SPI_MODE_0 | SPI_CS_HIGH ;
	spich->spi->bits_per_word = 8;
	spich->spi->chip_select = 0;

	error = spi_setup(spich->spi);

	return error;
}

int mh1_fetch_shading_tbl(struct spich_data *spich, uint8_t *table, int size)
{
	int error = 0;
	struct spi_message msg;

	printk("%s : Enter!! \n", __func__);

	if (!error) {

		struct spi_transfer cmd2 = {	//send frimware
			.cs_change = 1,
			.delay_usecs = 0,
			.speed_hz = (u32)spich->spi_freq_mhz,
			.tx_buf = table,
			.rx_buf = NULL,
			.len	= (int)size,
			.tx_dma = 0,
			.rx_dma = 0,
			.bits_per_word = 0,
		};

		mutex_lock(&spich->buf_lock);

		mh1_spi_write_set(spich);

		spi_message_init(&msg);
		spi_message_add_tail(&cmd2,  &msg);
		error = spi_sync(spich->spi, &msg);
		if (error)
			dev_err(&spich->spi->dev, "spi_sync failed.\n");

		mutex_unlock(&spich->buf_lock);

	}

	printk("%s done\n", __func__);

	return error;

}

int mh1_fetch_image_from_sd(struct spich_data *spich)
{
	int error = 0;
	struct spi_message msg;
	struct file *fp = NULL;

	mm_segment_t old_fs = get_fs();

	long fsize, nread;
	uint8_t *buf_mh1 = NULL;

	pr_err("%s : Enter!! \n", __func__);

	set_fs(KERNEL_DS);

	fp = filp_open("/system/media/RS_MH1.BIN", O_RDONLY, 0);
	if (IS_ERR(fp)) {
		pr_err("failed to open %s, err %ld. load from kernel/firmware\n",
			"/system/media/RS_MH1.BIN", PTR_ERR(fp));

		error = mh1_fetch_image(mh1_spich);
		if (error){
			pr_err("%s : load failed!! \n", __func__);
			goto out;
		}else{
			pr_err("%s : load success from kernel/firmware!! \n", __func__);
			return error;
		}
	}

	fsize = fp->f_path.dentry->d_inode->i_size;

	buf_mh1 = kmalloc(fsize, GFP_KERNEL);
	if (!buf_mh1) {
		pr_err("failed to allocate memory\n");
		error = -ENOMEM;
		goto out;
	}

	nread = vfs_read(fp, (char __user *)buf_mh1, fsize, &fp->f_pos);
	if (nread != fsize) {
		pr_err("failed to read firmware file, %ld Bytes\n", nread);
		error = -EIO;
		goto out;
	}

	filp_close(fp, current->files);

	if (!error) {

		struct spi_transfer cmd2 = {	//send frimware
			.cs_change = 1,
			.delay_usecs = 0,
			.speed_hz = (u32)spich->spi_freq_mhz,
			.tx_buf = buf_mh1,
			.rx_buf = NULL,
			.len	= fsize,
			.tx_dma = 0,
			.rx_dma = 0,
			.bits_per_word = 0,
		};

		mutex_lock(&spich->buf_lock);

		mh1_spi_write_set(spich);

		spi_message_init(&msg);
		spi_message_add_tail(&cmd2,  &msg);
		error = spi_sync(spich->spi, &msg);
		if (error)
			dev_err(&spich->spi->dev, "spi_sync failed.\n");

		mutex_unlock(&spich->buf_lock);

	}

	pr_err("%s:mh1_fetch_image done\n", __func__);

out:
	if (buf_mh1)
		kfree(buf_mh1);

	if (!IS_ERR(fp))
		filp_close(fp, current->files);

	set_fs(old_fs);

	pr_err("X");

	return error;

}

int mh1_fetch_image(struct spich_data *spich)
{
	int error = 0;
	struct spi_message msg;
	const struct firmware *fw_entry=NULL;

//	char tx_buffer[8] = {0x4, 0x7, 0x74, 0xe0, 0x1, 0x0, 0x0, 0x0}; // send frimware
//	char rx_buffer[1] = {0};

	if(strlen(mh1_inbuilt_fw_name_list) > 0)
	{
		error = request_firmware(&fw_entry,
				mh1_inbuilt_fw_name_list,
				&spich->spi->dev);
		if (error != 0)
		{
			printk( "%s: Firmware image %s not available\n", __func__, mh1_inbuilt_fw_name_list);
			return 1;
		}
	}
	if( fw_entry != NULL)
		printk("MH1 Firmware image size = %zu\n", fw_entry->size);

	if (!error) {
/*
		struct spi_transfer cmd1 = {	//send frimware
			.cs_change = 1,
			.delay_usecs = 0,
			.speed_hz = (u32)spich->spi_freq_mhz,
			.tx_buf = tx_buffer,
			.rx_buf = NULL,
			.len    = 8,
			.tx_dma = 0,
			.rx_dma = 0,
			.bits_per_word = 0,
		};
*/
		struct spi_transfer cmd2 = {	//send frimware
			.cs_change = 1,
			.delay_usecs = 0,
			.speed_hz = (u32)spich->spi_freq_mhz,
			.tx_buf =(u8*) fw_entry->data,
			.rx_buf = NULL,
			.len	= (int)fw_entry->size,
			.tx_dma = 0,
			.rx_dma = 0,
			.bits_per_word = 0,
		};

/*
		struct spi_transfer data = {
			.cs_change = 1,
			.delay_usecs = 0,
			.speed_hz = (u32)spich->spi_freq_mhz,
			.tx_buf = NULL,
			.rx_buf = rx_buffer,
			.len    = 1,
			.tx_dma = 0,
			.rx_dma = 0,
			.bits_per_word = 0,
		};
*/
		mutex_lock(&spich->buf_lock);

//Send Firmware
/*
		mh1_spi_write_set(spich);

		spi_message_init(&msg);
		spi_message_add_tail(&cmd1,  &msg);

		error = spi_sync(spich->spi, &msg);
		if (error)
			dev_err(&spich->spi->dev, "spi_sync failed.\n");

		mh1_spi_read_set(spich);

		spi_message_init(&msg);
		spi_message_add_tail(&data,  &msg);
		error = spi_sync(spich->spi, &msg);
		if (error)
			dev_err(&spich->spi->dev, "spi_sync failed.\n");

		printk("MH1 rx_buffer = %d\n", rx_buffer[0]);
*/
// Send Firmware
		mh1_spi_write_set(spich);

		spi_message_init(&msg);
		spi_message_add_tail(&cmd2,  &msg);
		error = spi_sync(spich->spi, &msg);
		if (error)
			dev_err(&spich->spi->dev, "spi_sync failed.\n");

		mutex_unlock(&spich->buf_lock);

	}

	pr_err("%s:mh1_fetch_image done\n", __func__);

	if (fw_entry)
		release_firmware(fw_entry);

	return error;
}
#endif

void Spi_Cs_Configuration(int number)
{
	if(number==1){ //cs3, TDMB
		gpio_set_value(spich_p->gpio_array[GPIO_IDX_CS0].gpio,0);
	#if defined (CONFIG_MACH_MSM8992_PPLUS_KR)
		gpio_set_value(spich_p->gpio_array[GPIO_IDX_CS3].gpio,0);
	#endif
		gpio_set_value(spich_p->gpio_array[GPIO_IDX_CS2].gpio,1);
	}
	else if(number==2){ //cs2, STM
		gpio_set_value(spich_p->gpio_array[GPIO_IDX_CS0].gpio,0);
		gpio_set_value(spich_p->gpio_array[GPIO_IDX_CS2].gpio,0);
	#if defined (CONFIG_MACH_MSM8992_PPLUS_KR)
		gpio_set_value(spich_p->gpio_array[GPIO_IDX_CS3].gpio,1);
	#endif
	}
	else if(number==3){ //cs0, MH1
		gpio_set_value(spich_p->gpio_array[GPIO_IDX_CS0].gpio,1);
		gpio_set_value(spich_p->gpio_array[GPIO_IDX_CS2].gpio,1);
	#if defined (CONFIG_MACH_MSM8992_PPLUS_KR)
		gpio_set_value(spich_p->gpio_array[GPIO_IDX_CS3].gpio,1);
	#endif
	}
	else if(number==4){ //default state
		gpio_set_value(spich_p->gpio_array[GPIO_IDX_CS0].gpio,0);
		gpio_set_value(spich_p->gpio_array[GPIO_IDX_CS2].gpio,1);
	#if defined (CONFIG_MACH_MSM8992_PPLUS_KR)
		gpio_set_value(spich_p->gpio_array[GPIO_IDX_CS3].gpio,1);
	#endif
	}
}

EXPORT_SYMBOL(Spi_Cs_Configuration);




struct spich_data *spich_p;
const char *stm_inbuilt_fw_name_list;

void spi_transfer(u8 *tx_buf,u8 *rx_buf,int size)
{

	int error = 0;
	struct spi_message msg;
    struct spi_transfer cmd2 = {	//send frimware
		.cs_change = 1,
		.delay_usecs = 10,
		.speed_hz = 5*1024*1024,
		.tx_buf =(u8*)tx_buf,
		.rx_buf = (u8*)rx_buf,
		.len	= (int)size,
		.tx_dma = 0,
		.rx_dma = 0,
		.bits_per_word = 8,
	};
//	mutex_lock(&spich_p->buf_lock);
	spi_message_init(&msg);
	spi_message_add_tail(&cmd2,  &msg);
	error = spi_sync(spich_p->spi, &msg);
	if (error)
		dev_err(&spich_p->spi->dev, "spi_sync failed.\n");

//	mutex_unlock(&spich_p->buf_lock);

}


void  StmResetHub( uint8_t tmp) {
	int gpio_state[5] = {0,};
    printk("STM %s START status : %d \n",__func__,tmp);
	if(gpio_state[0] == 0){
	    gpio_set_value(spich_p->gpio_array[GPIO_IDX_LDOEN].gpio,1);
	    mdelay(spich_p->pre_reset_delay);
	}
	if(tmp==STM_SYSTEM) {
		gpio_set_value(spich_p->gpio_array[GPIO_IDX_BOOT0].gpio,1);
	}
	else if(tmp == STM_RESET) {
		gpio_set_value(spich_p->gpio_array[GPIO_IDX_BOOT0].gpio,0);
	}
	else if(tmp == STM_SHUTDOWN) {
        if(spich_p->pre_reset_delay==0){
		    gpio_set_value(spich_p->gpio_array[GPIO_IDX_LDOEN].gpio,0);
        }
		return;
	}
	mdelay(spich_p->pre_reset_delay*50);// if under rev.A reset_delay is 20ms
	gpio_set_value(spich_p->gpio_array[GPIO_IDX_NRST].gpio, 0);
	mdelay(spich_p->pre_reset_delay+3);
	gpio_set_value(spich_p->gpio_array[GPIO_IDX_NRST].gpio, 1);

    printk("STM %s END status : %d \n",__func__,tmp);
	return;
}

/* function name : stm_calibration
 *
 *
 */

#define ENABLE 0x01
#define DISABLE 0x00
#define HUB_ID 0x12
#define HUB_ID_TRY_CNT 10

enum SPI_APIF_REG_e {
	SPI_APIF_REG_HUBID = 0x01,
	SPI_APIF_REG_TSENABLE,
	SPI_APIF_REG_FACTORYCAL,
	SPI_APIF_REG_TSBINS,
	SPI_APIF_REG_INTSTAT,
	SPI_APIF_REG_FRCNT,
	SPI_APIF_REG_TSDATA,
	SPI_APIF_REG_APSTAT,
	SPI_APIF_REG_HUBSTAT,
	SPI_APIF_REG_GYROCALVALUE,
	SPI_APIF_REG_ACCELCALVALUE,
	SPI_APIF_REG_IMUCFG,
	SPI_APIF_REG_MAX
};
enum SPI_APIF_HUBSTATUS_e {
	SPI_APIF_HUB_READY,
	SPI_APIF_HUB_PWR,
	SPI_APIF_HUB_PWRRES,
	SPI_APIF_HUB_TSEN,
	SPI_APIF_HUB_VDATA,
	SPI_APIF_HUB_SDATA,
	SPI_APIF_HUB_GYRCAL,
	SPI_APIF_HUB_ACCCAL,
};

void spi_read_write(uint8_t reg,uint8_t *tx_buffer, uint8_t *rx_buffer, uint32_t size) {
	*tx_buffer = reg;
	gpio_set_value(spich_p->gpio_array[GPIO_IDX_AP2SH].gpio, 0);
	spi_transfer(tx_buffer, rx_buffer, size);
	gpio_set_value(spich_p->gpio_array[GPIO_IDX_AP2SH].gpio, 1);
}

int stm_calibration_cmd(void)
{
    uint16_t hub_status;
    int id_chk_cnt = 0;
    int cal_try_cnt = 0;
    uint16_t gyrocal_bit = 1<<SPI_APIF_HUB_GYRCAL;
    int16_t gyro_offset[3];
	int8_t accel_offset[3];

	uint8_t tx_buffer[10]={0,};
	uint8_t rx_buffer[10]={0,};
	printk("STM %s(%d)\n",__func__,__LINE__);
	usleep(120000);
	do{
		spi_read_write(SPI_APIF_REG_HUBID|0x80,tx_buffer, rx_buffer, 3);
	} while (rx_buffer[2] != HUB_ID && ++id_chk_cnt < HUB_ID_TRY_CNT);
	printk("STM Device ID : %x, cnt : %d \n", rx_buffer[2], id_chk_cnt);

	spi_read_write(SPI_APIF_REG_FACTORYCAL,tx_buffer, rx_buffer, 2);
	usleep(900000);

    do {
        spi_read_write(SPI_APIF_REG_HUBSTAT|0x80,tx_buffer, rx_buffer, 4);
        memcpy(&hub_status, rx_buffer+2, 2);
		usleep(10000);
	} while (++cal_try_cnt < HUB_ID_TRY_CNT && ((hub_status & gyrocal_bit) == 0));

	if ( hub_status & gyrocal_bit) {
        spi_read_write(SPI_APIF_REG_GYROCALVALUE|0x80,tx_buffer, rx_buffer, 8);
		memcpy(gyro_offset, rx_buffer+2, 6);

        spi_read_write(SPI_APIF_REG_ACCELCALVALUE|0x80,tx_buffer, rx_buffer, 8);
		memcpy(accel_offset, rx_buffer+2, 3);
		printk("STM Gryoscope calibrated at %dth attempt, gyro offset : (%d,%d,%d), accel offset : (%d,%d,%d)\n", cal_try_cnt, gyro_offset[0], gyro_offset[1], gyro_offset[2], accel_offset[0], accel_offset[1], accel_offset[2]);
	} else {
		printk("STM Gryoscope calibration failed \n");
        return STM_NOK;
	}

	printk( "STM calibration cmd send.\n");
    return STM_OK;
}

int stm_calibration(void)
{
    uint8_t result=0;
    int error=0;

	spich_p->spi->mode = SPI_MODE_0;
	spich_p->spi->bits_per_word = 8;
	error = spi_setup(spich_p->spi);
	if(error < 0) {
		printk("STM spi setup fail  spi error = %d \n",error);
		return STM_SPI_SETUP_FAIL;
	}

    StmResetHub(STM_RESET);
    Spi_Cs_Configuration(2);
    /* we need implement the result code in here
     * I guess we can use wait_for_completion
     */
    printk("STM CALIBRATION START %s(%d) \n",__func__,__LINE__);

    result = stm_calibration_cmd();

    printk("STM CALIBRATION END %s(%d) \n",__func__,__LINE__);
    Spi_Cs_Configuration(4);
	StmResetHub(STM_SHUTDOWN);
    return result;
}
/*firmware update procedure
 * 1. read the kernel firmrware header info
 * 2. read STM header in specific address "STM_HEADER_ADDRESS"
 * 3. compare header info of both
 * 4. if not? download the new firmware
 */

int try_download_firmware(struct spich_data *spich)
{
	const struct firmware *fw_entry=NULL;

	uint8_t fw_header[STM_HEADER_SIZE+1]={0,};
	uint8_t stm_header[STM_HEADER_SIZE+1]={0,};
    int error=0;
    uint8_t FwHeaderExist=0x00;
    uint32_t header_offset=0;
    uint32_t FirmwareWriteOffset=0;
    uint8_t retry_cnt=0;

	spich_p = spich;
	spich->spi->mode = SPI_MODE_0;
	spich->spi->bits_per_word = 8;
//	spi->chip_select = 2;
	error = spi_setup(spich->spi);
    memset(fw_header,0x00,STM_HEADER_SIZE+1);
    memset(stm_header,0x00,STM_HEADER_SIZE+1);
	if(error < 0) {
		printk("STM spi setup fail  spi error = %d \n",error);
		return STM_SPI_SETUP_FAIL;
	}

	if(strlen(stm_inbuilt_fw_name_list) <= 0){
		return STM_FIRMWARE_NOT_EXIST;
    }
	error = request_firmware(&fw_entry,	stm_inbuilt_fw_name_list,&spich->spi->dev);
	if (error != 0) {
	    printk( "STM %s: Firmware image %s not available error=%d\n", __func__, stm_inbuilt_fw_name_list,error);
		return STM_FIRMWARE_NOT_EXIST;
	}
	if( fw_entry != NULL)
		printk( "STM Firmware image name = %s size = %zu\n",stm_inbuilt_fw_name_list, fw_entry->size);

    StmResetHub(STM_RESET);
    mdelay(spich->pre_reset_delay);
    StmResetHub(STM_SYSTEM);
    mdelay(spich->pre_reset_delay);

    Spi_Cs_Configuration(2);
    for(retry_cnt=0;retry_cnt < 12;retry_cnt++){
        if(communication_check()== STM_OK){
            break;
        }
        else{
            printk("AP-STM communication fail try = %dth\n",retry_cnt);
            StmResetHub(STM_RESET);
            mdelay(spich->pre_reset_delay);
            StmResetHub(STM_SYSTEM);
            mdelay(spich->pre_reset_delay);
        }
    }
    if(retry_cnt >= 12){
#if defined(STM_NOT_SLEEP)
	    StmResetHub(STM_SHUTDOWN);
#endif
        return STM_COMMUNICATION_FAIL;
    }
    FirmwareWriteOffset=0;
    header_offset = (fw_entry->size-STM_HEADER_SIZE);
    memcpy(fw_header,(fw_entry->data+header_offset),STM_HEADER_SIZE);
    printk("STM  fw header : %s\n",fw_header);


    if(firmware_header_check(fw_header) == STM_HEADER_EXIST){
        FwHeaderExist|=0x01;
        printk("STM firmware header exist !!!! \n");
    }

    StmFlashRead((STM_CODE_START_ADDRESS+header_offset),stm_header,STM_HEADER_SIZE);
    printk("STM stm header : %s\n",stm_header);
    if(firmware_header_check(stm_header) == STM_HEADER_EXIST){
        FwHeaderExist|=0x02;
        printk("STM stm header exist !!!! \n");
    }
#if defined(DONOT_DOWNLOAD_FIRMWARE)
#if defined(STM_NOT_SLEEP)
    StmResetHub(STM_SHUTDOWN);
#endif
	return STM_OK;
#endif
    switch(FwHeaderExist){
        case 0x00: /* both not exists */

        case 0x01: /* firmware header exist but stm NOT exist.  download the image and header*/
        case 0x02: /* firmware header NOT exist STM exist. download the image and erase header */
            printk("STM %d header NOT exist!!!\n",FwHeaderExist);
            StmErase(fw_entry->size+STM_HEADER_SIZE);
            StmFlashWrite(STM_CODE_START_ADDRESS,((uint8_t *)fw_entry->data+FirmwareWriteOffset),(uint32_t)fw_entry->size); //main code download
            break;
        case 0x03: /* Both Exists */
        {
            printk("STM both header exist!!!\n");
            if(strncmp(fw_header,stm_header, STM_HEADER_SIZE)!=0){ /*not same .download the image*/
                printk("STM you need to update STM firmware. download start.... erase start ");
                StmErase(fw_entry->size+STM_HEADER_SIZE);
                StmFlashWrite(STM_CODE_START_ADDRESS,((uint8_t *)fw_entry->data+FirmwareWriteOffset),(uint32_t)fw_entry->size);
            }
            else{
                printk("STM firmware is already latest version \n");
            }
            break;
        }
        default:
            printk("STM error %s(%d)\n",__func__,__LINE__);
            break;
        break;
    };

    Spi_Cs_Configuration(4);
#if !defined(STM_NOT_SLEEP)
	StmResetHub(STM_RESET);
#else
	StmResetHub(STM_SHUTDOWN);
#endif
    printk("STM download firmware done\n");

	if (fw_entry)
		release_firmware(fw_entry);

	return STM_OK;
}
