#include "servo.h"

sSERVO servo[SERVO_NUM];

void DMA1_Stream4_IRQHandler(void)  //·¢ËÍDMAÖÐ¶Ï
{
	sSERVO *ele = servo;
	__HAL_DMA_CLEAR_FLAG(ele->huart->hdmatx,DMA_FLAG_FEIF0_4|DMA_FLAG_DMEIF0_4|DMA_FLAG_TEIF0_4|DMA_FLAG_HTIF0_4|DMA_FLAG_TCIF0_4);
	__HAL_DMA_DISABLE(ele->huart->hdmatx);
	ele->TxFlag = false;
}

bool UART3_Transmit_DMA(u8 *pData, u16 Size)  //DMA·¢ËÍ
{
	sSERVO *ele = servo;
	if(ele->TxFlag == true) return false;
	__HAL_DMA_DISABLE(ele->huart->hdmatx);
	ele->huart->hdmatx->Instance->NDTR = Size;
	ele->huart->hdmatx->Instance->M0AR = (u32)pData;
	__HAL_DMA_CLEAR_FLAG(ele->huart->hdmatx,DMA_FLAG_FEIF0_4|DMA_FLAG_DMEIF0_4|DMA_FLAG_TEIF0_4|DMA_FLAG_HTIF0_4|DMA_FLAG_TCIF0_4);
	__HAL_DMA_ENABLE(ele->huart->hdmatx);
	ele->TxFlag = true;
	return true;
}

void Servo_Init_Para(sSERVO *ele,sSERVO_CFG *elecfg)
{
	ele->Update = false;
	ele->Sta = STA_INI;
	ele->Err = ERR_NONE;
	
	ele->huart = elecfg->huart;
	ele->name = elecfg->name;
	
	ele->Chk.CNT = 0;
	ele->Chk.CCR = elecfg->CheckNum;
	ele->Time = 0;
	
}

bool Servo_Init(sSERVO *ele)
{
	//??DMA     DMA1_Stream4
	__HAL_DMA_DISABLE(ele->huart->hdmatx);
	ele->huart->hdmatx->Instance->PAR = (u32)&ele->huart->Instance->TDR;
	ele->huart->hdmatx->Instance->NDTR = SERVO_TX_LEN;
	ele->huart->hdmatx->Instance->M0AR = (u32)ele->TxDat;
	__HAL_DMA_CLEAR_FLAG(ele->huart->hdmatx,DMA_FLAG_FEIF0_4|DMA_FLAG_DMEIF0_4|DMA_FLAG_TEIF0_4|DMA_FLAG_HTIF0_4|DMA_FLAG_TCIF0_4);
	__HAL_DMA_ENABLE_IT(ele->huart->hdmatx,DMA_IT_TC);
	SET_BIT(ele->huart->Instance->CR3, USART_CR3_DMAT);
	
	Dprintf("\r\n%s Init...\r\n",ele->name);
	if(ele->Sta != STA_INI)
	{
		Dprintf("--Par [NO]\r\n");
        return false;
	}
	ele->Sta = STA_RUN;
	Dprintf("%s Init.[OK]\r\n",ele->name);
	return true;
}

bool Servo_Calc(sSERVO *ele)
{
	if(ele->TxFlag == true) return false;
	u16 temp;

	servo->TxDat[0]='$';
	servo->TxDat[1]=11;
	temp = mot->PWM[6];
	servo->TxDat[2]=BYTE1(temp);
	servo->TxDat[3]=BYTE0(temp);
	temp = mot->PWM[7];
	servo->TxDat[4]=BYTE1(temp);
	servo->TxDat[5]=BYTE0(temp);
//	temp = mot->PWM[8];
//	servo->TxDat[6]=BYTE1(temp);
//	servo->TxDat[7]=BYTE0(temp);
//	temp = mot->PWM[9];
//	servo->TxDat[8]=BYTE1(temp);
//	servo->TxDat[9]=BYTE0(temp);
//	
	u8 sum = 0;
	for(u8 i=1;i<10;i++)sum += servo->TxDat[i];
	servo->TxDat[10]=sum;
	return UART3_Transmit_DMA ((u8 *)servo->TxDat, 11);
	
	
}


