#include "system.h"

/***************************************************\
功能：
  系统 多线程布置
说明：
  1、初始化需要一定时间，若有看门狗，需要在初始化之后再开启
  2、Sys_Get线程进行输入设备数据获取与计算，如传感器、控制指令
  3、Sys_Spr线程进行备份输入设备数据获取与计算
  4、Sys_Ctr线程进行输出设备输出计算与控制，如执行器、回传信息
  5、Sys_Chk线程进行系统状态检测与决策，LED或蜂鸣器等显示状态
  6、Sys_1ms线程进行1ms运行，执行快速动作或状态机切换等
\***************************************************/

sTIM Sys_Time[5];
sTIM Deta_Time[5];
u32 Time[5];

u16 number = 0;


///******************驱动程序****************/
void TIM7_IRQHandler(void)
{
  if(__HAL_TIM_GET_FLAG(&htim7, TIM_FLAG_UPDATE) != RESET)
  {
    __HAL_TIM_CLEAR_IT(&htim7, TIM_IT_UPDATE);
	  HAL_IncTick();
  }
}

/******************功能函数****************/
void Sys_Init(void)
{
	HAL_Delay(500);
	Dprintf("\r\n**********Ductet Vehicle**********\r\n");
	Para_Init(para);
	Bell_Init(bell);
//	Imu_Init(imu);//..mpu6050
	Gps_Init(gps);
	Ms5611_Init(&ms5611[0]);
	Ms5611_Init(&ms5611[1]);
	Icm20602_Init(&ICM20602[0]);
	Ak8975_Init(&ak8975[0]);
	Lsm_Init(&lsm[0]);
	L3g_Init(&l3g[0]);
	Rc_Init(rc);
	
	Ahrs_Init(&ahrs[0]);
	
	//EKF_Init_MCH(&ahrs[1]);
	ESKF_Init_MCH(&ahrs[1]);
//	EKF_Init_Para(&EKF);
	
	Exit_Init(exitx);
	Tran_Init(tran);
	
	Mot_Init(mot);
//	Servo_Init(servo);
//	Eso_Init(eso);
	HAL_TIM_Base_Start_IT(&htim5);
	Bell_Start(bell);
	Dprintf("\r\n**********INITIAL SUCCESS**********\r\n");
}

//1ms周期，特殊时序控制，在freeRTOS.c修改
void Sys_1ms(void)
{
	Tim_Calc(&Deta_Time[3]);
	Tim_Calc(&Sys_Time[3]);
	Bell_Loop_1ms(bell);
	Tran_Loop_1ms(tran);
	Tim_Calc(&Sys_Time[3]);
	Time[3] = Sys_Time[3].OUT;
}

//5ms周期，传感器获取，在freeRTOS.c修改
void Sys_Get(void)
{
	Tim_Calc(&Deta_Time[0]);
	Tim_Calc(&Sys_Time[0]);
	Ms5611_Calc(&ms5611[0]);
//	Ms5611_Calc(&ms5611[1]);
	Icm20602_Calc(&ICM20602[0]);
//	Ak8975_Calc(&ak8975[0]);
//	Lsm_Calc(&lsm[0]);
//	L3g_Calc(&l3g[0]);
	Gps_Calc(gps);
	Exit_Calc(exitx);
	Rc_Calc(rc);
	Bell_Calc(bell);
	Ahrs_Calc(&ahrs[0]);
	//EKF_Attitude_Calc_MCH(&ahrs[1]);
	ESKF_Attitude_Calc_MCH(&ahrs[1]);//0.7ms
	
	Tran_Calc(tran);
	Tim_Calc(&Sys_Time[0]);
	Time[0] = Sys_Time[0].OUT;
//	gps->GPSUpdate=false;
	Tim_Calc(&Sys_Time[0]);
	Time[0] = Sys_Time[0].OUT;
	
}

//10ms周期，执行器控制，在freeRTOS.c修改
void Sys_Run(void)
{
	Tim_Calc(&Deta_Time[2]);
	Tim_Calc(&Sys_Time[2]);
//	EKF_PV_Calc(gps);//1.8ms
//	Eso_Calc(eso);
	
	GPS_INS_EKF(gps);//9阶EKF 位置、速度、加速度偏置
	LLH2NED(gps);//LLH坐标系转NED坐标系
	gps->GPSUpdate=false;
	
	Control_Step(CtrlIO);

	Ms5611_Ctr(ms5611);//气压计温度控制
	exitx->Update[1] = false;
	exitx->Update[2] = false;
	Mot_Ctr(mot);
  //Fprintf("%d$",CtrlIO->control_mode,"%d\r\n",Time[2]);
	Tim_Calc(&Sys_Time[2]);
	Time[2] = Sys_Time[2].OUT;	}


//..改成10ms//50ms周期１，备用传感器获取，在freeRTOS.c修改
void Sys_Spr(void)
{
	Tim_Calc(&Deta_Time[1]);
	Tim_Calc(&Sys_Time[1]);
//	EKF_Run(&EKF);//22阶EKF滤波器	//0.01ms
//	DMAprintf("rps:%.2f\r\n",exitx->DatRel[0]);
//	DMAprintf("%d,%d,%d,%d,%d\r\n",Deta_Time[0].OUT,Deta_Time[1].OUT,Deta_Time[2].OUT,Deta_Time[3].OUT,Deta_Time[4].OUT);
	Tim_Calc(&Sys_Time[1]);
	Time[1] = Sys_Time[1].OUT;
}


//100ms周期，系统检测，在freeRTOS.c修改
void Sys_Chk(void)
{ 
	Tim_Calc(&Deta_Time[4]);
	Tim_Calc(&Sys_Time[4]);
	HAL_ADC_Start_DMA(bell->hadc,(u32 *)bell->AdcRaw,1);	//adc采集电池电压
	Led_Tog(1);
	if(ICM20602->Err)Led_Set(2);
	else Led_Clr(2);
	if(ms5611->Err)Led_Set(3);
	else Led_Clr(3);
	if(ak8975->Err)Led_Set(4);
	else Led_Clr(4);
	u8 ErrFlag=0;
	if(ICM20602->Err)ErrFlag+=0x01;
//	if(imu->Err)ErrFlag+=0x02;
//	if(uwb->Err)ErrFlag+=0x04;
	if(ms5611[0].Err)ErrFlag+=0x08;
//	if(ms5611[1].Err)ErrFlag+=0x10;
//	if(gps->MagErr)ErrFlag+=0x20;
//	if(exitx->Err)ErrFlag+=0x40;
	if(bell->Err)ErrFlag+=80;//5
	if(ErrFlag)
	{
		Bell_Error(bell);
//		DMAprintf("%2X\r\n",ErrFlag);
	}
	ICM20602->Err=ERR_NONE;
	ms5611[0].Err=ERR_NONE;
	ms5611[1].Err=ERR_NONE;
	rc->Err=ERR_NONE;
	gps->MagErr=ERR_NONE;
	exitx->Err[0]=ERR_NONE;
	exitx->Err[1]=ERR_NONE;
	exitx->Err[2]=ERR_NONE;
	bell->Err=ERR_NONE;
	Tim_Calc(&Sys_Time[4]);
	Time[4] = Sys_Time[4].OUT;
}
