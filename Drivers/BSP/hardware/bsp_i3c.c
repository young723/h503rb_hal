
#include "stdio.h"
#include "string.h"
#include "bsp_i3c.h"


#define TARGET1_DYN_ADDR        0x32	//0x32
#define TARGET2_DYN_ADDR        0x36
I3C_HandleTypeDef hi3c1;

static int i3c_wait_count = 0;
#define HAL_WAIT_I3C_READY	\
	i3c_wait_count = 0;	\
	while(HAL_I3C_GetState(&hi3c1) != HAL_I3C_STATE_READY)	\
	{	\
		qst_delay_us(200);	\
		if(i3c_wait_count++ > 20000)	\
		{	\
			printf("error: wait i3c ready timeout\r\n");	\
			i3c_wait_count = 0;	\
			break;	\
		}	\
	} \
	i3c_wait_count = 0;

/*
#define HAL_I3C_ERROR_HANDLE	\
	__disable_irq();	\
	while(1)	\
	{	\
		if(i3c_wait_count++ > 200000)	\
		{	\
			printf("line-%d i3c error!\r\n", __LINE__);	\
			i3c_wait_count = 0;	\
		}	\
	}	\
*/

#define HAL_I3C_ERROR_HANDLE		\
	do	\
	{	\
		printf("line-%d i3c error!\r\n", __LINE__);	\
		while(1)	\
		{\
			if(i3c_wait_count++ > 200000)	\
			{\
				break;	\
			}\
		}\
	}while(0);

/* Variable to catch IBI event */
//__IO uint32_t uwIBIRequested = 0;

/* Descriptor that contains the bus devices configurations */
static I3C_DeviceConfTypeDef DeviceConf[4] = {0};
/* CCC information updated after CCC event */
static I3C_CCCInfoTypeDef CCCInfo;

static uint8_t aSETDASA_data[1] = {(TARGET1_DYN_ADDR << 1)};
static uint8_t aENEC_data[1] = {0x01};
static uint8_t aDISEC_data[1] = {0x01};
static uint8_t aSETMWL_data[2] = {0x0, 0x10};
static uint8_t aSETMRL_data[2] = {0x0, 0x10};

/* Descriptor for direct CCC */
static const I3C_CCCTypeDef RstDASA_CCC[] =
{
/*   Target Addr            CCC Value    			CCC data         						Direction        */	
	{TARGET1_DYN_ADDR,		Direct_RSTDASA, 		{NULL, 0},								LL_I3C_DIRECTION_WRITE},
};

static const I3C_CCCTypeDef SetDASA_CCC[] =
{
/*   Target Addr            CCC Value    			CCC data         						Direction        */	
	{0x0c,         			Direct_SETDASA,         {aSETDASA_data, 1},              		LL_I3C_DIRECTION_WRITE},
};

/* Descriptor for direct CCC */
static const I3C_CCCTypeDef Dir_CCC[] =
{
/*   Target Addr            CCC Value    			CCC data         						Direction        */
	{TARGET1_DYN_ADDR,		Direct_GETPID,          {NULL,                  6},             LL_I3C_DIRECTION_READ },
	{TARGET1_DYN_ADDR,		Direct_GETBCR,          {NULL,                  1},             LL_I3C_DIRECTION_READ },
	{TARGET1_DYN_ADDR,		Direct_GETDCR,          {NULL,                  1},             LL_I3C_DIRECTION_READ },
	{TARGET1_DYN_ADDR,		Direct_GETSTATUS,		{NULL,					2}, 			LL_I3C_DIRECTION_READ },

	{TARGET1_DYN_ADDR,		Direct_SETMWL,          {aSETMWL_data,          2},             LL_I3C_DIRECTION_WRITE},
	{TARGET1_DYN_ADDR,		Direct_SETMRL,          {aSETMRL_data,          2},             LL_I3C_DIRECTION_WRITE},
	{TARGET1_DYN_ADDR,		Direct_GETMWL,          {NULL,                  2},             LL_I3C_DIRECTION_READ },
	{TARGET1_DYN_ADDR,		Direct_GETMRL,          {NULL,                  2},             LL_I3C_DIRECTION_READ },
	
	{TARGET1_DYN_ADDR,		Direct_DISEC,          	{aDISEC_data,			1},             LL_I3C_DIRECTION_WRITE },
	{TARGET1_DYN_ADDR,		Direct_ENEC,          	{aENEC_data,			1},             LL_I3C_DIRECTION_WRITE }
};

static i3c_content_t		i3cCnt;
static i3c_wr_t				i3cWR;
static i3c_buf_t			i3cBuf;

/**
  * @brief I3C1 Initialization Function
  * @param None
  * @retval None
  */
void MX_I3C1_Init(unsigned char p_clk_l, unsigned char p_clk_h)
{

  /* USER CODE BEGIN I3C1_Init 0 */

  /* USER CODE END I3C1_Init 0 */

  I3C_FifoConfTypeDef sFifoConfig = {0};
  I3C_CtrlConfTypeDef sCtrlConfig = {0};

  /* USER CODE BEGIN I3C1_Init 1 */

  /* USER CODE END I3C1_Init 1 */
  hi3c1.Instance = I3C1;
  hi3c1.Mode = HAL_I3C_MODE_CONTROLLER;
  hi3c1.Init.CtrlBusCharacteristic.SDAHoldTime = HAL_I3C_SDA_HOLD_TIME_1_5;
  hi3c1.Init.CtrlBusCharacteristic.WaitTime = HAL_I3C_OWN_ACTIVITY_STATE_0;
#if 0
#if defined(I3C_1M)
  hi3c1.Init.CtrlBusCharacteristic.SCLPPLowDuration = 0x7c;
  hi3c1.Init.CtrlBusCharacteristic.SCLI3CHighDuration = 0x7c;
#elif defined(I3C_4M)
  hi3c1.Init.CtrlBusCharacteristic.SCLPPLowDuration = 0x1e;
  hi3c1.Init.CtrlBusCharacteristic.SCLI3CHighDuration = 0x1e;
#elif defined(I3C_6_25M)
  hi3c1.Init.CtrlBusCharacteristic.SCLPPLowDuration = 0x13;
  hi3c1.Init.CtrlBusCharacteristic.SCLI3CHighDuration = 0x13;
#elif defined(I3C_10M)
  hi3c1.Init.CtrlBusCharacteristic.SCLPPLowDuration = 0x0b;
  hi3c1.Init.CtrlBusCharacteristic.SCLI3CHighDuration = 0x0b;
#else
  hi3c1.Init.CtrlBusCharacteristic.SCLPPLowDuration = 0x09;
  hi3c1.Init.CtrlBusCharacteristic.SCLI3CHighDuration = 0x09;
#endif
#endif

  hi3c1.Init.CtrlBusCharacteristic.SCLPPLowDuration = p_clk_l;
  hi3c1.Init.CtrlBusCharacteristic.SCLI3CHighDuration = p_clk_h;

  hi3c1.Init.CtrlBusCharacteristic.SCLODLowDuration = 0xff;		//0xff;	//0x59;
  hi3c1.Init.CtrlBusCharacteristic.SCLI2CHighDuration = 0x00;

  hi3c1.Init.CtrlBusCharacteristic.BusFreeDuration = 0x32;
  hi3c1.Init.CtrlBusCharacteristic.BusIdleDuration = 0xf8;

#if defined(I3C_I2C_MIX_MODE)
  hi3c1.Init.CtrlBusCharacteristic.SCLPPLowDuration = 0x09;
  hi3c1.Init.CtrlBusCharacteristic.SCLI3CHighDuration = 0x09;
  hi3c1.Init.CtrlBusCharacteristic.SCLODLowDuration = 0xae;
  hi3c1.Init.CtrlBusCharacteristic.SCLI2CHighDuration = 0x4a;
  hi3c1.Init.CtrlBusCharacteristic.BusFreeDuration = 0x83;
  hi3c1.Init.CtrlBusCharacteristic.BusIdleDuration = 0xf8;
#endif
  if (HAL_I3C_Init(&hi3c1) != HAL_OK)
  {
    HAL_I3C_ERROR_HANDLE;
  }

  /** Configure FIFO
  */
  sFifoConfig.RxFifoThreshold = HAL_I3C_RXFIFO_THRESHOLD_1_4;
  sFifoConfig.TxFifoThreshold = HAL_I3C_TXFIFO_THRESHOLD_1_4;
  sFifoConfig.ControlFifo = HAL_I3C_CONTROLFIFO_DISABLE;
  sFifoConfig.StatusFifo = HAL_I3C_STATUSFIFO_DISABLE;
  if (HAL_I3C_SetConfigFifo(&hi3c1, &sFifoConfig) != HAL_OK)
  {
    HAL_I3C_ERROR_HANDLE;
  }

  /** Configure controller
  */
  sCtrlConfig.DynamicAddr = 0;
  sCtrlConfig.StallTime = 0x00;
  sCtrlConfig.HotJoinAllowed = DISABLE;
  sCtrlConfig.ACKStallState = DISABLE;
  sCtrlConfig.CCCStallState = DISABLE;
  sCtrlConfig.TxStallState = DISABLE;
  sCtrlConfig.RxStallState = DISABLE;
  sCtrlConfig.HighKeeperSDA = DISABLE;
  if (HAL_I3C_Ctrl_Config(&hi3c1, &sCtrlConfig) != HAL_OK)
  {
    HAL_I3C_ERROR_HANDLE;
  }
  /* USER CODE BEGIN I3C1_Init 2 */

  /* USER CODE END I3C1_Init 2 */

}


#if 0
HAL_StatusTypeDef I3C_Send_Direct_CCC(void)
{
	i3cWR.xfer.CtrlBuf.pBuffer = i3cBuf.ctlBuf;
	i3cWR.xfer.CtrlBuf.Size    = COUNTOF(i3cBuf.ctlBuf);
	i3cWR.xfer.TxBuf.pBuffer   = i3cBuf.txBuf;
	i3cWR.xfer.TxBuf.Size      = 0;	//COUNTOF(i3cBuf.txBuf);
	i3cWR.xfer.RxBuf.pBuffer   = i3cBuf.rxBuf;
	i3cWR.xfer.RxBuf.Size      = 0;	//COUNTOF(i3cBuf.rxBuf);

	for(int32_t i=0; i<COUNTOF(Dir_CCC); i++)
	{
		if(Dir_CCC[i].Direction == LL_I3C_DIRECTION_WRITE)
		{
			i3cWR.xfer.TxBuf.Size += Dir_CCC[i].CCCBuf.Size;
		}
		else if(Dir_CCC[i].Direction == LL_I3C_DIRECTION_READ)
		{
			i3cWR.xfer.RxBuf.Size += Dir_CCC[i].CCCBuf.Size;
		}
		Dir_CCC[i].TargetAddr = i3cCnt.desc[0].DYNAMIC_ADDR;	//aTargetDesc[0]->DYNAMIC_ADDR;
	}
	if(HAL_I3C_AddDescToFrame(&hi3c1, Dir_CCC, NULL, &i3cWR.xfer, COUNTOF(Dir_CCC), I3C_DIRECT_WITHOUT_DEFBYTE_RESTART|I3C_DIRECT_WITHOUT_DEFBYTE_STOP) != HAL_OK)
	{
		printf("I3C_Send_Direct_CCC err1\r\n");
		HAL_I3C_ERROR_HANDLE;
	}
	HAL_WAIT_I3C_READY
	if(HAL_I3C_Ctrl_MultipleTransfer_IT(&hi3c1, &i3cWR.xfer) != HAL_OK)
	{
		printf("I3C_Send_Direct_CCC err2\r\n");
		HAL_I3C_ERROR_HANDLE;
	}

	HAL_WAIT_I3C_READY

	return HAL_OK;
}
#endif

HAL_StatusTypeDef I3C_Send_Direct_CCC(uint8_t       ccc)
{
	I3C_CCCTypeDef selCCC;

	i3cWR.xfer.CtrlBuf.pBuffer = i3cBuf.ctlBuf;
	i3cWR.xfer.CtrlBuf.Size    = COUNTOF(i3cBuf.ctlBuf);
	i3cWR.xfer.TxBuf.pBuffer   = i3cBuf.txBuf;
	i3cWR.xfer.TxBuf.Size      = 0;//COUNTOF(i3cBuf.txBuf);
	i3cWR.xfer.RxBuf.pBuffer   = i3cBuf.rxBuf;
	i3cWR.xfer.RxBuf.Size      = 0;///COUNTOF(i3cBuf.rxBuf);


	for(int32_t i=0; i<COUNTOF(Dir_CCC); i++)
	{
		if(Dir_CCC[i].CCC == ccc)
		{
			if(Dir_CCC[i].Direction == LL_I3C_DIRECTION_WRITE)
			{
				i3cWR.xfer.TxBuf.Size += Dir_CCC[i].CCCBuf.Size;
			}
			else if(Dir_CCC[i].Direction == LL_I3C_DIRECTION_READ)
			{
				i3cWR.xfer.RxBuf.Size += Dir_CCC[i].CCCBuf.Size;
			}
			memcpy(&selCCC, &Dir_CCC[i], sizeof(I3C_CCCTypeDef));

			selCCC.TargetAddr = i3cCnt.desc[0].DYNAMIC_ADDR;

			HAL_WAIT_I3C_READY
			if(HAL_I3C_AddDescToFrame(&hi3c1, &selCCC, NULL, &i3cWR.xfer, 1, I3C_DIRECT_WITHOUT_DEFBYTE_RESTART|I3C_DIRECT_WITHOUT_DEFBYTE_STOP) != HAL_OK)
			{
				printf("I3C_send_direct_ccc_ext %d err1\r\n", ccc);
				HAL_I3C_ERROR_HANDLE;
			}
			if(HAL_I3C_Ctrl_MultipleTransfer_IT(&hi3c1, &i3cWR.xfer) != HAL_OK)
			{
				printf("I3C_send_direct_ccc_ext %d err2\r\n", ccc);
				HAL_I3C_ERROR_HANDLE;
			}
			break;
		}
	}

	return HAL_OK;
}

HAL_StatusTypeDef I3C_Send_CCC_EC(int en)
{
	i3cWR.xfer.CtrlBuf.pBuffer = i3cBuf.ctlBuf;
	i3cWR.xfer.CtrlBuf.Size    = COUNTOF(i3cBuf.ctlBuf);
	i3cWR.xfer.TxBuf.pBuffer   = i3cBuf.txBuf;
	i3cWR.xfer.TxBuf.Size      = COUNTOF(i3cBuf.txBuf);	//1;
	i3cWR.xfer.RxBuf.pBuffer   = NULL;
	i3cWR.xfer.RxBuf.Size      = 0;

	HAL_WAIT_I3C_READY
	if(en)
	{
		I3C_CCCTypeDef en_ec[] =
		{
			{TARGET1_DYN_ADDR, 		Broadcast_ENEC, 		{aENEC_data,		1}, 	LL_I3C_DIRECTION_WRITE},
		};

		if(HAL_I3C_AddDescToFrame(&hi3c1, en_ec, NULL, &i3cWR.xfer, COUNTOF(en_ec), I3C_BROADCAST_WITHOUT_DEFBYTE_RESTART) != HAL_OK)
		{
			printf("I3C_send_ccc ENEC err1\r\n");
			HAL_I3C_ERROR_HANDLE;
		}
	}
	else
	{
		I3C_CCCTypeDef dis_ec[] =
		{
			{TARGET1_DYN_ADDR, 		Broadcast_DISEC,		{aDISEC_data,		1}, 	LL_I3C_DIRECTION_WRITE},
		};
		if(HAL_I3C_AddDescToFrame(&hi3c1, dis_ec, NULL, &i3cWR.xfer, COUNTOF(dis_ec), I3C_BROADCAST_WITHOUT_DEFBYTE_RESTART) != HAL_OK)
		{
			printf("I3C_send_ccc DISEC err1\r\n");
			HAL_I3C_ERROR_HANDLE;
		}
	}

	if(HAL_I3C_Ctrl_TransmitCCC_IT(&hi3c1, &i3cWR.xfer) != HAL_OK)
	//if(HAL_I3C_Ctrl_TransmitCCC(&hi3c1, &i3cWR.xfer, 1000) != HAL_OK)
	{
		printf("I3C_Send_Broadcast_CCC err2\r\n");
		HAL_I3C_ERROR_HANDLE;
	}

	return HAL_OK;

}

HAL_StatusTypeDef I3C_Send_Direct_RSTDASA(void)
{
	i3cWR.xfer.CtrlBuf.pBuffer = i3cBuf.ctlBuf;
	i3cWR.xfer.CtrlBuf.Size    = COUNTOF(i3cBuf.ctlBuf);
	i3cWR.xfer.TxBuf.pBuffer   = i3cBuf.txBuf;
	i3cWR.xfer.TxBuf.Size      = 0;	//COUNTOF(i3cBuf.txBuf);
	i3cWR.xfer.RxBuf.pBuffer   = i3cBuf.rxBuf;
	i3cWR.xfer.RxBuf.Size      = 0;	//COUNTOF(i3cBuf.rxBuf);

	for(int32_t i=0; i<COUNTOF(RstDASA_CCC); i++)
	{
		if(RstDASA_CCC[i].Direction == LL_I3C_DIRECTION_WRITE)
		{
			i3cWR.xfer.TxBuf.Size += RstDASA_CCC[i].CCCBuf.Size;
		}
		else if(RstDASA_CCC[i].Direction == LL_I3C_DIRECTION_READ)
		{
			i3cWR.xfer.RxBuf.Size += RstDASA_CCC[i].CCCBuf.Size;
		}

	}
	HAL_WAIT_I3C_READY
	if(HAL_I3C_AddDescToFrame(&hi3c1, RstDASA_CCC, NULL, &i3cWR.xfer, COUNTOF(RstDASA_CCC), I3C_DIRECT_WITHOUT_DEFBYTE_RESTART) != HAL_OK)
	{
		printf("I3C_Send_Direct_RSTDASA err1\r\n");
		HAL_I3C_ERROR_HANDLE;
	}
	if(HAL_I3C_Ctrl_MultipleTransfer_IT(&hi3c1, &i3cWR.xfer) != HAL_OK)
	{
		printf("I3C_Send_Direct_RSTDASA err2\r\n");
		HAL_I3C_ERROR_HANDLE;
	}

	return HAL_OK;
}

HAL_StatusTypeDef I3C_Send_Direct_DASA(void)
{
	i3cWR.xfer.CtrlBuf.pBuffer = i3cBuf.ctlBuf;
	i3cWR.xfer.CtrlBuf.Size    = COUNTOF(i3cBuf.ctlBuf);
	i3cWR.xfer.TxBuf.pBuffer   = i3cBuf.txBuf;
	i3cWR.xfer.TxBuf.Size      = 0;	//COUNTOF(i3cBuf.txBuf);
	i3cWR.xfer.RxBuf.pBuffer   = i3cBuf.rxBuf;
	i3cWR.xfer.RxBuf.Size      = 0;	//COUNTOF(i3cBuf.rxBuf);

	for(int32_t i=0; i<COUNTOF(SetDASA_CCC); i++)
	{
		if(SetDASA_CCC[i].Direction == LL_I3C_DIRECTION_WRITE)
		{
			i3cWR.xfer.TxBuf.Size += SetDASA_CCC[i].CCCBuf.Size;
		}
		else if(SetDASA_CCC[i].Direction == LL_I3C_DIRECTION_READ)
		{
			i3cWR.xfer.RxBuf.Size += SetDASA_CCC[i].CCCBuf.Size;
		}

		i3cCnt.desc[i].STATIC_ADDR = SetDASA_CCC[i].TargetAddr;
		i3cCnt.desc[i].DYNAMIC_ADDR = SetDASA_CCC[i].CCCBuf.pBuffer[0]>>1;
	}
	HAL_WAIT_I3C_READY
	if(HAL_I3C_AddDescToFrame(&hi3c1, SetDASA_CCC, NULL, &i3cWR.xfer, COUNTOF(SetDASA_CCC), I3C_DIRECT_WITHOUT_DEFBYTE_RESTART) != HAL_OK)
	{
		printf("I3C_Send_Direct_DASA err1\r\n");
		HAL_I3C_ERROR_HANDLE;
	}
	if(HAL_I3C_Ctrl_MultipleTransfer_IT(&hi3c1, &i3cWR.xfer) != HAL_OK)
	{
		printf("I3C_Send_Direct_DASA err2\r\n");
		HAL_I3C_ERROR_HANDLE;
	}

	return HAL_OK;
}


#define I3C_PRIVATE_OPTION			I3C_PRIVATE_WITH_ARB_STOP
//#define I3C_PRIVATE_OPTION			I3C_PRIVATE_WITH_ARB_STOP|I3C_PRIVATE_WITH_ARB_RESTART
//#define I3C_PRIVATE_OPTION			(I3C_PRIVATE_WITHOUT_ARB_RESTART|I3C_PRIVATE_WITHOUT_ARB_STOP)

HAL_StatusTypeDef I3C_ReadReg(uint8_t reg, uint8_t* buf, uint32_t len)
{
	i3cBuf.txBuf[0] = reg;

	i3cWR.xfer.CtrlBuf.pBuffer = i3cBuf.ctlBuf;
	i3cWR.xfer.CtrlBuf.Size    = 1;
	i3cWR.xfer.TxBuf.pBuffer   = i3cBuf.txBuf;
	i3cWR.xfer.TxBuf.Size      = 1;
	i3cWR.xfer.RxBuf.pBuffer   = i3cBuf.rxBuf;
	i3cWR.xfer.RxBuf.Size      = len;

	i3cWR.private.TargetAddr = TARGET1_DYN_ADDR;
	i3cWR.private.Direction = HAL_I3C_DIRECTION_WRITE;
	i3cWR.private.TxBuf.pBuffer = i3cBuf.txBuf;
	i3cWR.private.TxBuf.Size = 1;
	i3cWR.private.RxBuf.pBuffer = i3cBuf.rxBuf;
	i3cWR.private.RxBuf.Size = len;


	/*##- Add context buffer transmit in Frame context #####################*/
	HAL_WAIT_I3C_READY
	if(HAL_I3C_AddDescToFrame(&hi3c1, NULL, &i3cWR.private, &i3cWR.xfer, 1, I3C_PRIVATE_OPTION) != HAL_OK)		// I3C_PRIVATE_WITH_ARB_STOP
	{
		/* HAL_I3C_ERROR_HANDLE function is called when error occurs. */
		HAL_I3C_ERROR_HANDLE;
	}
	/*##- Start the transmission process ###################################*/
	/* Transmit private data processus */
	//HAL_WAIT_I3C_READY
	if(HAL_I3C_Ctrl_Transmit(&hi3c1, &i3cWR.xfer, 500) != HAL_OK)
	{
		/* HAL_I3C_ERROR_HANDLE function is called when error occurs. */
		HAL_I3C_ERROR_HANDLE;
	}

	HAL_WAIT_I3C_READY
	i3cWR.private.Direction = HAL_I3C_DIRECTION_READ;
	if(HAL_I3C_AddDescToFrame(&hi3c1, NULL, &i3cWR.private, &i3cWR.xfer, 1, I3C_PRIVATE_OPTION) != HAL_OK)
	{
		/* HAL_I3C_ERROR_HANDLE function is called when error occurs. */
		HAL_I3C_ERROR_HANDLE;
	}
	//HAL_WAIT_I3C_READY
	if (HAL_I3C_Ctrl_Receive(&hi3c1, &i3cWR.xfer, 500) != HAL_OK)
	{
		/* HAL_I3C_ERROR_HANDLE function is called when error occurs. */
		HAL_I3C_ERROR_HANDLE;
	}
	//HAL_WAIT_I3C_READY

	for(int i=0; i<len; i++)
	{
		buf[i] = i3cBuf.rxBuf[i];
	}
	
	return HAL_OK;
}

HAL_StatusTypeDef I3C_WriteReg(uint8_t reg, uint8_t* buf, uint32_t len)
{
	I3C_PrivateTypeDef des;
	I3C_XferTypeDef xfer;

	i3cBuf.txBuf[0] = reg;
	for(int i=0; i<len; i++)
	{
		i3cBuf.txBuf[i+1] = buf[i];
	}

	i3cWR.xfer.CtrlBuf.pBuffer = i3cBuf.ctlBuf;
	i3cWR.xfer.CtrlBuf.Size    = 1;
	i3cWR.xfer.TxBuf.pBuffer   = i3cBuf.txBuf;
	i3cWR.xfer.TxBuf.Size      = len+1;
//	i3cWR.xfer.RxBuf.pBuffer   = i3cBuf.rxBuf;
//	i3cWR.xfer.RxBuf.Size      = len;

	i3cWR.private.TargetAddr = TARGET1_DYN_ADDR;
	i3cWR.private.Direction = HAL_I3C_DIRECTION_WRITE;
	i3cWR.private.TxBuf.pBuffer = i3cBuf.txBuf;
	i3cWR.private.TxBuf.Size = len+1;
//	i3cWR.private.RxBuf.pBuffer = i3cBuf.rxBuf;
//	i3cWR.private.RxBuf.Size = len;

	HAL_WAIT_I3C_READY
	if (HAL_I3C_AddDescToFrame(&hi3c1, NULL, &i3cWR.private, &i3cWR.xfer, 1, I3C_PRIVATE_OPTION) != HAL_OK)
	{
		/* HAL_I3C_ERROR_HANDLE function is called when error occurs. */
		HAL_I3C_ERROR_HANDLE;
	}

	if (HAL_I3C_Ctrl_Transmit(&hi3c1, &i3cWR.xfer, 500) != HAL_OK)
	{
		/* HAL_I3C_ERROR_HANDLE function is called when error occurs. */
		HAL_I3C_ERROR_HANDLE;
	}

	return HAL_OK;
}

int bsp_i3c_read(unsigned char addr, unsigned char *buf, unsigned short len)
{
	if(I3C_ReadReg(addr, buf, len) == HAL_OK)
		return 1;
	else
		return 0;
}

int bsp_i3c_write(unsigned char addr, unsigned char value)
{
	if(I3C_WriteReg(addr, &value, 1) == HAL_OK)
		return 1;
	else
		return 0;
}


void qst_evb_i3c_info(void)
{
	uint8_t	buf[8];
	uint16_t factory_id;
	uint8_t	bcr;
	uint8_t dcr;

	for(uint8_t i=0; i<i3cCnt.total; i++)
	{
		memcpy(buf, &i3cCnt.desc[i].TARGET_BCR_DCR_PID , sizeof(uint64_t));
		factory_id = (buf[0]<<8)|buf[1];
		bcr = buf[6];
		dcr = buf[7];

		if((factory_id == 0x0437)||(factory_id == 0x0000))
		{
			i3cCnt.desc[i].STATIC_ADDR = 0x0c;
			i3cCnt.desc[i].TARGET_NAME = "qmc6309h";
		}
		else if(factory_id == 0x086e)
		{
			i3cCnt.desc[i].STATIC_ADDR = 0x6a;
			i3cCnt.desc[i].TARGET_NAME = "qmi8658";
		}

		printf("static-add[0x%02x] dyn-add[0x%02x]\r\n", i3cCnt.desc[i].STATIC_ADDR, i3cCnt.desc[i].DYNAMIC_ADDR);
		printf("buf: 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\r\n", buf[0],buf[1],buf[2],buf[3],buf[4],buf[5],buf[6],buf[7]);
		printf("Pid: 0x%02x%02x%02x%02x%02x%02x\r\n", buf[0],buf[1],buf[2],buf[3],buf[4],buf[5]);
		printf("Manufacture id: 0x%04x\r\n", factory_id);
		printf("VerId:0x%02x\r\n", buf[2]);
		printf("AsicVer:0x%02x\r\n",(buf[3]>>5)&0x07);
		printf("WaferId:0x%02x\r\n",(buf[3])&0x1f);
		printf("DieId:0x%02x 0x%02x\r\n",buf[4],buf[5]);
		printf("BCR: 0x%02x\r\n", bcr);
		printf("DCR: 0x%02x\r\n", dcr);

	}
}


int qst_evb_enry_i3c(void)
{
	int retry = 0;
	memset(&i3cCnt, 0, sizeof(i3cCnt));

//ENTDAA:
	HAL_WAIT_I3C_READY;
#if 0
	printf("I3C_DIRECT_SET_DASA \r\n");
	I3C_Send_Direct_RSTDASA();
	I3C_Send_Direct_DASA();
#else
	printf("I3C_RSTDAA_THEN_ENTDAA \r\n");
	if(HAL_I3C_Ctrl_DynAddrAssign_IT(&hi3c1, I3C_RSTDAA_THEN_ENTDAA) != HAL_OK)	// HAL_I3C_Ctrl_DynAddrAssign I3C_RSTDAA_THEN_ENTDAA		I3C_ONLY_ENTDAA
	{
		HAL_I3C_ERROR_HANDLE;
		return 0;
	}
	else
	{
		//qst_delay_ms(500);
	}

	retry = 0;
	while((i3cCnt.daa_req_done==0)&&(i3cCnt.daa_done==0))
	{
		qst_delay_ms(200);	//qst_delay_us(500000);
		printf("wait for entdaa flag(%d %d) count(%d)\r\n", i3cCnt.daa_req_done, i3cCnt.daa_done, i3cCnt.total);
		if(retry++ > 50)
		{
			return 0;
			//goto ENTDAA;
		}
	}

	while(i3cCnt.total == 0)
	{
		printf("i3cCnt.total = 0\r\n");
		//qst_delay_ms(100);
		return 0;
	}
#endif

	qst_evb_i3c_info();
	//qst_delay_ms(100);
	I3C_Send_CCC_EC(1);	
	//qst_delay_ms(100);
	//I3C_Send_Direct_CCC();
	//qst_delay_ms(100);

	
	i3cWR.xfer.CtrlBuf.pBuffer = i3cBuf.ctlBuf;
	i3cWR.xfer.CtrlBuf.Size    = 0;
	i3cWR.xfer.TxBuf.pBuffer   = i3cBuf.txBuf;
	i3cWR.xfer.TxBuf.Size	   = 0;
	i3cWR.xfer.RxBuf.pBuffer   = i3cBuf.rxBuf;
	i3cWR.xfer.RxBuf.Size	   = 0;

	i3cWR.private.TargetAddr = TARGET1_DYN_ADDR;
	i3cWR.private.Direction = HAL_I3C_DIRECTION_WRITE;
	i3cWR.private.TxBuf.pBuffer = i3cBuf.txBuf;
	i3cWR.private.TxBuf.Size = 0;
	i3cWR.private.RxBuf.pBuffer = i3cBuf.rxBuf;
	i3cWR.private.RxBuf.Size = 0;

	return 1;
}


void qst_evb_i3c_enable_ibi(int ibi_en)
{
	static char mcu_ibi_init = 0;
	
	if((ibi_en)&&(mcu_ibi_init==0))
	{
		for(int i = 0; i < i3cCnt.total; i++)
		{
			DeviceConf[i].DeviceIndex        = (i+1);
			DeviceConf[i].TargetDynamicAddr  = i3cCnt.desc[i].DYNAMIC_ADDR;
			DeviceConf[i].IBIAck             = __HAL_I3C_GET_IBI_CAPABLE(__HAL_I3C_GET_BCR(i3cCnt.desc[i].TARGET_BCR_DCR_PID));
			DeviceConf[i].IBIPayload         = __HAL_I3C_GET_IBI_PAYLOAD(__HAL_I3C_GET_BCR(i3cCnt.desc[i].TARGET_BCR_DCR_PID));
			DeviceConf[i].CtrlRoleReqAck     = __HAL_I3C_GET_CR_CAPABLE(__HAL_I3C_GET_BCR(i3cCnt.desc[i].TARGET_BCR_DCR_PID));
			DeviceConf[i].CtrlStopTransfer   = DISABLE;

			if (HAL_I3C_Ctrl_ConfigBusDevices(&hi3c1, &DeviceConf[i], 1U) != HAL_OK)
			{
				HAL_I3C_ERROR_HANDLE;
			}
		}

		/*##- Start the listen mode process ####################################*/
		/* Activate notifications for specially for this example
		 - In Band Interrupt requested by a Target. */
		if( HAL_I3C_ActivateNotification(&hi3c1, NULL, HAL_I3C_IT_IBIIE) != HAL_OK)
		{
			HAL_I3C_ERROR_HANDLE;
		}
		mcu_ibi_init = 1;
	}
}


void qst_i3c_ibi_handle(void)
{
	if(i3cCnt.ibi_req)
	{
		i3cCnt.ibi_req = 0;
		/* Getting the information from the last IBI request */
		//if (HAL_I3C_GetCCCInfo(&hi3c1, EVENT_ID_IBI, &CCCInfo) != HAL_OK)
		//{
		//	HAL_I3C_ERROR_HANDLE;
		//}
		if(HAL_I3C_DeactivateNotification(&hi3c1, HAL_I3C_IT_IBIIE) != HAL_OK)
		{
			HAL_I3C_ERROR_HANDLE;
		}
//		printf("IBI ");
		if(i3cCnt.func)
		{
			i3cCnt.func();
		}

		if(HAL_I3C_ActivateNotification(&hi3c1, NULL, HAL_I3C_IT_IBIIE) != HAL_OK)
		{
			HAL_I3C_ERROR_HANDLE;
		}
	}
}

void qst_evb_reg_i3c_ibi_hdlr(int_callback func)
{
	i3cCnt.func = func;
}


/**
  * @brief I3C target request a dynamic address callback.
  *        The main objective of this user function is to check if a target request a dynamic address.
  *        if the case we should assign a dynamic address to the target.
  * @par Called functions
  * - HAL_I3C_TgtReqDynamicAddrCallback()
  * - HAL_I3C_Ctrl_SetDynamicAddress()
  * @retval None
  */
void HAL_I3C_TgtReqDynamicAddrCallback(I3C_HandleTypeDef *hi3c, uint64_t targetPayload)
{
	//printf("HAL_I3C_TgtReqDynamicAddrCallback\r\n");
	/* Update Payload on aTargetDesc */
#if 0
	aTargetDesc[uwTargetCount]->TARGET_BCR_DCR_PID = targetPayload;
	/* Send associated dynamic address */
	HAL_I3C_Ctrl_SetDynAddr(hi3c, aTargetDesc[uwTargetCount++]->DYNAMIC_ADDR);
	daa_req_done = 1;
#else
	i3cCnt.desc[i3cCnt.total].TARGET_BCR_DCR_PID = targetPayload;
	i3cCnt.desc[i3cCnt.total].DYNAMIC_ADDR = TARGET1_DYN_ADDR+i3cCnt.total;
	HAL_I3C_Ctrl_SetDynAddr(hi3c, i3cCnt.desc[i3cCnt.total].DYNAMIC_ADDR);
	i3cCnt.total++;
	i3cCnt.daa_req_done = 1;
#endif
}

/**
  * @brief  Controller dynamic address assignment Complete callback.
  * @param  hi3c : [IN] Pointer to an I3C_HandleTypeDef structure that contains the configuration information
  *                     for the specified I3C.
  * @retval None
  */
void HAL_I3C_CtrlDAACpltCallback(I3C_HandleTypeDef *hi3c)
{
	/* No specific action, on this example */
//	printf("HAL_I3C_CtrlDAACpltCallback\r\n");
	//daa_done = 1;
	i3cCnt.daa_done = 1;
}

/**
  * @brief  Controller Transmit Complete callback.
  * @param  hi3c : [IN] Pointer to an I3C_HandleTypeDef structure that contains the configuration information
  *                     for the specified I3C.
  * @retval None
  */
void HAL_I3C_CtrlTxCpltCallback(I3C_HandleTypeDef *hi3c)
{
	
}

/**
  * @brief  Controller Reception Complete callback.
  * @param  hi3c : [IN] Pointer to an I3C_HandleTypeDef structure that contains the configuration information
  *                     for the specified I3C.
  * @retval None
  */
void HAL_I3C_CtrlRxCpltCallback(I3C_HandleTypeDef *hi3c)
{

}

void HAL_I3C_CtrlMultipleXferCpltCallback(I3C_HandleTypeDef *hi3c)
{
	//printf("HAL_I3C_CtrlMultipleXferCpltCallback\r\n");
}


/**
  * @brief I3C notify callback after receiving a notification.
  *        The main objective of this user function is to check on the notification ID and assign 1 to the global
  *        variable used to indicate that the event is well finished.
  * @par Called functions
  * - HAL_I3C_NotifyCallback()
  * @retval None
  */
void HAL_I3C_NotifyCallback(I3C_HandleTypeDef *hi3c, uint32_t eventId)
{
  if ((eventId & EVENT_ID_IBI) == EVENT_ID_IBI)
  {
	i3cCnt.ibi_req = 1;
  }
  else
  {
    /* HAL_I3C_ERROR_HANDLE function is called when error occurs. */
    HAL_I3C_ERROR_HANDLE;
  }
}

