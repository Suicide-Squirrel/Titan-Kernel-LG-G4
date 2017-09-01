#include <linux/io.h>

#define STM_SPI_SOF 	0x5A
#define STM_SPI_ACK     	0x79
#define STM_SPI_NACK    	0x1F
#define STM_SPI_IDLE    	0xA5
// Commnad List
#define GET_CMD_COMMAND        0x00  /* Get CMD command               */
#define GET_VER_COMMAND        0x01  /* Get Version command           */
#define GET_ID_COMMAND         0x02  /* Get ID command                */
#define RMEM_COMMAND           0x11  /* Read Memory command           */
#define GO_COMMAND             0x21  /* GO command                    */
#define WMEM_COMMAND           0x31  /* Write Memory command          */
#define EXT_ER_COMMAND         0x44  /* Erase Memory command          */
#define WP_COMMAND             0x63  /* Write Protect command         */
#define WU_COMMAND             0x73  /* Write Unprotect command       */
#define RP_COMMAND             0x82  /* Readout Protect command       */
#define RU_COMMAND             0x92  /* Readout Unprotect command     */


#define XOR_GET_CMD_COMMAND    0xFF  /* Get CMD command               */
#define XOR_GET_VER_COMMAND    0xFE  /* Get Version command           */
#define XOR_GET_ID_COMMAND     0xFD  /* Get ID command                */
#define XOR_RMEM_COMMAND       0xEE  /* Read Memory command           */
#define XOR_GO_COMMAND         0xDE  /* GO command                    */
#define XOR_WMEM_COMMAND       0xCE  /* Write Memory command          */
#define XOR_EXT_ER_COMMAND     0xBB  /* Erase Memory command          */
#define XOR_WP_COMMAND         0x9C  /* Write Protect command         */
#define XOR_WU_COMMAND         0x8C  /* Write Unprotect command       */
#define XOR_RP_COMMAND         0x7D  /* Readout Protect command       */
#define XOR_RU_COMMAND         0x6D  /* Readout Unprotect command     */

#define DEF_ACK_RESET_RETRIES 100000
#define MAX_BUFFER_SIZE 256
#define MAX_SPI_PAYLOAD_SIZE 256
#define STM_HEADER_SIZE 32
#define STM_CODE_START_ADDRESS 0x8000000 /* code address */

typedef struct stm_headerInfo_s
{
    uint8_t head[8];
    uint8_t code_size[8];
    uint8_t date[8];
    uint8_t version[8];
} stm_headerInfo_t;

/*STM sector Info
 * Sector 0 0x800 0000 - 0x8003FFF : 16KB
 * Sector 1 0x800 4000 - 0x8007FFF : 16KB
 * Sector 2 0x800 8000 - 0x800BFFF : 16KB
 * Sector 3 0x800 C000 - 0x800FFFF : 16KB
 * Sector 4 0x801 0000 - 0x800FFFF : 64KB
 * Sector 5 0x802 0000 - 0x800FFFF : 128KB
 * Sector 6 0x804 0000 - 0x800FFFF : 128KB
 * Sector 7 0x806 0000 - 0x800FFFF : 128KB
 *        
 *
 */
#define STM_HEADER_ADDRESS 0x0800C000 /* last sector 7 */

enum {
    STM_OK = 0,
    STM_NOK,
    STM_COMMUNICATION_FAIL,
    STM_HEADER_EXIST,
    STM_HEADER_NOT_EXIST,
    STM_FIRMWARE_NOT_EXIST,
    STM_SPI_SETUP_FAIL,
    STM_ACK_TIMEOUT_FAIL,
    STM_INVAL,
};

extern void  StmResetHub( uint8_t tmp);
extern  int StmErase(uint32_t  firmware_size);
extern int StmFlashWrite(uint32_t dest_addr, uint8_t *file,uint16_t size);
extern int StmFlashRead(uint32_t start_addr,uint8_t *rx_data, int size);
extern int communication_check(void);
extern void spi_transfer(u8 *tx_buf,u8 *rx_buf,int size);


