

#ifndef __BSP_I3C_H
#define	__BSP_I3C_H

#include "stm32h5xx_hal.h"
#include "stm32h5xx_nucleo.h"

/* Broadcast CCC Command */
#define Broadcast_ENEC              0x00
#define Broadcast_DISEC             0x01
#define Broadcast_RSTDAA            0x06
#define Broadcast_ENTDAA            0x07
/* Direct CCC Command  */
#define Direct_ENEC                 0x80
#define Direct_DISEC                0x81
#define Direct_ENTAS0               0x82
#define Direct_ENTAS1               0x83
#define Direct_ENTAS2               0x84
#define Direct_ENTAS3               0x85
#define Direct_RSTDASA              0x86
#define Direct_SETDASA              0x87
#define Direct_SETNEWDA             0x88
#define Direct_SETMWL               0x89
#define Direct_SETMRL               0x8A
#define Direct_GETMWL               0x8B
#define Direct_GETMRL               0x8C
#define Direct_GETPID               0x8D
#define Direct_GETBCR               0x8E
#define Direct_GETDCR               0x8F
#define Direct_GETSTATUS            0x90
#define Direct_GETACCMST            0x91
#define Direct_GETMXDS              0x94
#define Direct_GETCAPS              0x95
#define Direct_SETXTIME             0x98
#define Direct_GETXTIME             0x99
#define Direct_RSTACT               0x9A
#define Direct_SETGRPA              0x9B
#define Direct_RSTGRPA              0x9C

typedef void (*int_callback)(void);

/* Target descriptor */
typedef struct {
  char *        TARGET_NAME;          /*!< Marketing Target reference */
  uint32_t      TARGET_ID;            /*!< Target Identifier on the Bus */
  uint64_t      TARGET_BCR_DCR_PID;   /*!< Concatenation value of PID, BCR and DCR of the target */
  uint8_t       STATIC_ADDR;          /*!< Static Address of the target, value found in the datasheet of the device */
  uint8_t       DYNAMIC_ADDR;         /*!< Dynamic Address of the target preset by software/application */
} TargetDesc_TypeDef;

/* BCR descriptor */
typedef struct {
  uint8_t      DCR;     /*!< Device Caracteristics Register */
  uint8_t      BCR;     /*!< Bus Caracteristics Register */
  uint64_t     PID;     /*!< Provisioned Identifier */
} PAYLOAD_DESC_t;
/* USER CODE END ET */


#define I3C_MAX_DEV		2
typedef struct
{
	TargetDesc_TypeDef		desc[I3C_MAX_DEV];
	//uint8_t					index;
	uint8_t					total;

	uint8_t					daa_done;
	uint8_t					daa_req_done;
	uint32_t				ibi_req;
	int_callback			func;
} i3c_content_t;

#define I3C_CTL_BUF_LEN		256		// 512
#define I3C_TX_LEN			256		//512
#define I3C_RX_LEN			256		//512

typedef struct
{
	uint8_t					dynaddr;
	HAL_StatusTypeDef		status;
	I3C_PrivateTypeDef		private;
	I3C_CCCTypeDef			ccc;
	I3C_XferTypeDef			xfer;
} i3c_wr_t;

typedef struct
{
	uint32_t				ctlBuf[I3C_CTL_BUF_LEN];
	uint8_t 				txBuf[I3C_TX_LEN];
	uint8_t 				rxBuf[I3C_RX_LEN];
} i3c_buf_t;

extern void I3C_Wait_Ready(void);
extern void MX_I3C1_Init(unsigned char p_clk_l, unsigned char p_clk_h);
extern int qst_evb_enry_i3c(void);
HAL_StatusTypeDef I3C_Send_Direct_CCC(uint8_t       ccc);
extern void qst_evb_i3c_enable_ibi(int ibi_en);
extern HAL_StatusTypeDef I3C_ReadReg(uint8_t reg, uint8_t* buf, uint32_t len);
extern HAL_StatusTypeDef I3C_WriteReg(uint8_t reg, uint8_t* buf, uint32_t len);
extern int bsp_i3c_read(unsigned char addr, unsigned char *buf, unsigned short len);
extern int bsp_i3c_write(unsigned char addr, unsigned char value);

extern void qst_i3c_ibi_handle(void);
extern void qst_evb_reg_i3c_ibi_hdlr(int_callback func);

#endif
