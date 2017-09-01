
/*
 * stm_driver.c - stm driver
 *
 * Copyright (c) 2015 LG Electronics.  All rights reserved.
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


#include <linux/delay.h>
#include <linux/io.h>
#include "stm_driver.h"


static uint8_t StmGetACK(void) {
  uint32_t count=0;
  uint8_t tx_buffer[1]={0,};
  uint8_t rx_buffer[1]={0,};
  tx_buffer[0] = 0x00;
  spi_transfer(tx_buffer, rx_buffer, 1);

  for (count=0;count<DEF_ACK_RESET_RETRIES;count++) {
      tx_buffer[0] = 0x00;
      spi_transfer(tx_buffer, rx_buffer, 1);
      tx_buffer[0] = 0x00;
      if (rx_buffer[0] == STM_SPI_ACK || rx_buffer[0] == STM_SPI_NACK) {
          tx_buffer[0] = STM_SPI_ACK;
      }
      spi_transfer(tx_buffer, rx_buffer, 1);

      if (tx_buffer[0] == STM_SPI_ACK) {
		return STM_OK;
      }
  }
  printk("STM STmGetACK TIMEOUT error !!!!! count=%d\n",count);
  return STM_ACK_TIMEOUT_FAIL;
}


static uint8_t StmSendCommand(uint8_t cmd,uint8_t xor_cmd) {
    uint8_t tx_buffer[3]={0,};
    uint8_t rx_buffer[1]={0,};
    tx_buffer[0] = 0x5a;
    tx_buffer[1] = cmd;
    tx_buffer[2] = xor_cmd;
    spi_transfer(tx_buffer, rx_buffer, 3);
    return StmGetACK();
}

static int StmSendAddr(uint32_t fw_addr) {
    uint8_t tx_buffer[5]={0,};
    uint8_t rx_buffer[1]={0,};
    tx_buffer[0] = (uint8_t)((fw_addr >> 24) & 0xFF);
    tx_buffer[1] = (uint8_t)((fw_addr >> 16) & 0xFF);
    tx_buffer[2] = (uint8_t)((fw_addr >> 8) & 0xFF);
    tx_buffer[3] = (uint8_t)(fw_addr & 0xFF);
    tx_buffer[4] = tx_buffer[0] ^ tx_buffer[1] ^ tx_buffer[2] ^ tx_buffer[3];
    spi_transfer(tx_buffer, rx_buffer, 5);
    return StmGetACK();
}

int StmErasePagesCmd(uint16_t nr_sectors) {
    uint16_t count = 0; //nr_sectors - 1;
    uint8_t tx_buffer[3]={0,};
    uint8_t rx_buffer[1]={0,};
    uint8_t result = STM_NOK;
    printk("STM %s(%d) sector : %d \n",__func__,__LINE__,nr_sectors);
    tx_buffer[0] = (uint8_t)(count >> 8);
    tx_buffer[1] = (uint8_t)(count & 0xFF);
    tx_buffer[2] = tx_buffer[0] ^ tx_buffer[1];

    spi_transfer(tx_buffer, rx_buffer, 3);
    result= StmGetACK();
    if(result != STM_OK)
        return result;
    tx_buffer[0] = (uint8_t)((nr_sectors >> 8) & 0xFF);
    tx_buffer[1] = (uint8_t)(nr_sectors & 0xFF);
    tx_buffer[2] = tx_buffer[0] ^ tx_buffer[1];
    spi_transfer(tx_buffer, rx_buffer, 3);
    return StmGetACK();


}

int StmEraseGlobalCmd(void) {
    uint16_t kNumPages=0xffff;
    uint8_t tx_buffer[3]={0,};
    uint8_t rx_buffer[1]={0,};
    tx_buffer[0] = kNumPages >> 8;
    tx_buffer[1] = kNumPages & 0xff;
    tx_buffer[2] = tx_buffer[0] ^ tx_buffer[1];
    spi_transfer(tx_buffer, rx_buffer, 3);
    return StmGetACK();
}


int StmSendByteCount(int bytes)
{
    uint8_t tx_buffer[2]={0,};
    uint8_t rx_buffer[1]={0,};
    if (bytes > 256) {
        printk("STM Cannot send or rcv more than 256 bytes per transfer\n");
        return STM_INVAL;
    }
    tx_buffer[0] = bytes - 1;
    tx_buffer[1] = ~tx_buffer[0];
    spi_transfer(tx_buffer, rx_buffer, 2);
    return StmGetACK();

}


/*  function name :StmErase
 *  Input value : firmware_size
 *  0xffff : chip erase,
 *  ex) if firmware_size is 17K, it erase sector 0,1
 */
int StmErase(uint32_t  firmware_size) {
    //    uint8_t gotACK;
    uint8_t cnt=0;
    int32_t remainSize=0;
    uint8_t result = STM_NOK;
    uint32_t stm_flash_sector[8] = {
        16*1024,
        16*1024,
        16*1024,
        16*1024,
        64*1024,
        128*1024,
        128*1024,
        128*1024};

    remainSize=firmware_size;
    printk("STM %s(%d) size:%d \n",__func__,__LINE__,firmware_size);


    if(firmware_size == 0xffff){ //erase all
        result = StmSendCommand(EXT_ER_COMMAND,XOR_EXT_ER_COMMAND);
        if(result != STM_OK)
            return result;
        result = StmEraseGlobalCmd();
        if(result != STM_OK)
            return result;
    }
    else { //erase sector
        while(remainSize >= 0){
            result = StmSendCommand(EXT_ER_COMMAND,XOR_EXT_ER_COMMAND);
            if(result != STM_OK)
                return result;
            result = StmErasePagesCmd(cnt);
            if(result != STM_OK)
                return result;
            remainSize -= stm_flash_sector[cnt];
            cnt++;
        }
    }
    return 0;
}

static uint16_t StmReadMoreData(uint8_t *src_file, uint8_t *dest_file, int request_size, int16_t src_file_size) {
    int n=0;
    int cnt=0;
    if(src_file_size <= 0) return 0;

    if(src_file_size > request_size)
        n=request_size;//remain date is above request_size
    else
        n = src_file_size;//remain size is under request_size
    for(cnt=0;cnt<n;cnt++) dest_file[cnt] = src_file[cnt];
    if (n & 1) {
        dest_file[cnt] = 0xff;
    }

    return n;
}
/*  function name :StmFlashWrite
 *  Input value : dest_addr : stm address to write the firmware
 *                     file : firmware to write
 *                     size : firmware size
 */
int StmFlashWrite(uint32_t dest_addr, uint8_t *file,uint16_t size) {
    uint16_t offset = 0;
    uint8_t data[MAX_SPI_PAYLOAD_SIZE+5]={0,};
    uint16_t n=0;
    uint8_t checksum ;
    uint16_t i;
    uint8_t c;
    uint32_t dstAddr=0;
    uint8_t tx_buffer[MAX_SPI_PAYLOAD_SIZE+5]={0,};
    uint8_t rx_buffer[1]={0,};
    uint8_t result = STM_NOK;
    printk("STM %s(%d) dest_addr : 0x%x size : %d \n",__func__,__LINE__,dest_addr,size);


    if(dest_addr < 0x8000000){
        printk( "STM flash write address range error dest = 0x%x \n",dest_addr);
        return STM_NOK ;
    }
    tx_buffer[0] = 0x5a;
    spi_transfer(tx_buffer, rx_buffer, 1);

    if(StmGetACK() != STM_OK)
        return result;

    while ((n = StmReadMoreData(file+offset, data, MAX_SPI_PAYLOAD_SIZE,size-offset)) > 0) {
        result = StmSendCommand( WMEM_COMMAND,XOR_WMEM_COMMAND);
        if(result != STM_OK)
            return result;
        dstAddr = dest_addr + offset;
        result = StmSendAddr(dstAddr);
        if(result != STM_OK)
            return result;
        tx_buffer[0] = MAX_SPI_PAYLOAD_SIZE-1;
        checksum = tx_buffer[0];
        for ( i = 0; i < n; i++) {
            c = data[i];
            tx_buffer[1 + i] = c;
            checksum ^= c;
        }
        tx_buffer[1 + n] = checksum;

        spi_transfer(tx_buffer, rx_buffer, n + 2);
        if(StmGetACK() != STM_OK)
            return result;
        offset += n;
    }
    printk("STM writing flash done.\n");
    return STM_OK;

}

/* function name : StmFlashRead
 *  Input value : dest_addr : stm address to write the firmware
 *                     file : firmware to write
 *                     size : firmware size
 */

int StmFlashRead(uint32_t start_addr,uint8_t *rx_data, int size)
{
    uint8_t tx_buffer[1]={0,};
    uint8_t rx_buffer[MAX_SPI_PAYLOAD_SIZE+5]={0,};
    uint8_t result = STM_NOK;
    printk("STM %s(%d) addr = %x size=%d \n",__func__,__LINE__,start_addr,size);
    tx_buffer[0] = 0x5a;
    spi_transfer(tx_buffer, rx_buffer, 1);

    if(StmGetACK() != STM_OK)
        return result;

    result = StmSendCommand(RMEM_COMMAND,XOR_RMEM_COMMAND);
    if(result != STM_OK)
        return result;
    result = StmSendAddr(start_addr);
    if(result != STM_OK)
        return result;
    result =StmSendByteCount(size);
    if(result != STM_OK)
        return result;
    tx_buffer[0] = 0x00;
    spi_transfer(tx_buffer, rx_buffer,1);
    tx_buffer[0] = 0x00;
    spi_transfer(tx_buffer, rx_buffer,size);
    memcpy(rx_data,rx_buffer,size);
    return STM_OK;
}

int communication_check(void)
{
    uint8_t tx_buffer[1]={0,};
    uint8_t rx_buffer[MAX_SPI_PAYLOAD_SIZE+5]={0,};
    printk("STM %s(%d)\n",__func__,__LINE__);
    msleep(60);
    tx_buffer[0] = STM_SPI_SOF;
    spi_transfer(tx_buffer, rx_buffer, 1);
    printk( "STM finish sending sync.\n");
    return StmGetACK();
}

int firmware_header_check(uint8_t *header)
{
    int ret=0;
    int cnt=0;
    for(cnt=0;cnt<STM_HEADER_SIZE;cnt++){
        //        printk("STM cnt=%d value=0x%x \n",cnt,header[cnt]);
        if(header[cnt]=='S'){
            header +=cnt;
            ret = strncmp((char *)header,"STM",3);
            if(ret == 0) //same
                return STM_HEADER_EXIST;
            else
                return STM_HEADER_NOT_EXIST;
        }
    }
    return STM_HEADER_NOT_EXIST;
}



