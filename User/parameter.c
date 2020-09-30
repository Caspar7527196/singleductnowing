#include "parameter.h"

/***************************************************\
功能：
  参数表
说明：
  1、修改几乎所有模块的配置
\***************************************************/

sPARA para[1];

void Para_Init(sPARA *ele)
{
	ele->icm20602[0].hspi = &hspi4;
	ele->icm20602[0].name = (char *)"ICM20602";
	ele->icm20602[0].FiltNum = 50;
	ele->icm20602[0].Dir = (char *)"-Y+X+Z";
//	ele->icm20602[0].AccOff[0] = -68;
//	ele->icm20602[0].AccOff[1] = -19;
//	ele->icm20602[0].AccOff[2] = 34;
//	ele->icm20602[0].GyrOff[0] = -0.0181f;
//	ele->icm20602[0].GyrOff[1] = -0.0271f;
//	ele->icm20602[0].GyrOff[2] = 0.0099f;
	ICM20602_Init_Para(ICM20602,ele->icm20602);
	
	ele->ak8975[0].hspi = &hspi4;
	ele->ak8975[0].name = (char *)"AK8975";
	ele->ak8975[0].FiltNum = 2;
//	ele->ak8975[0].Dir = (char *)"+X+Y+Z";
	ele->ak8975[0].Dir = (char *)"+X-Y-Z";
	ele->ak8975[0].MagOff[0] = 0;	//17
	ele->ak8975[0].MagOff[1] = 0;	//-47
	ele->ak8975[0].MagOff[2] = 0;	//-47
//	ele->ak8975[0].MagOff[0] = -86;
//	ele->ak8975[0].MagOff[1] = 26;
//	ele->ak8975[0].MagOff[2] = -1;
	Ak8975_Init_Para(ak8975,ele->ak8975);
	
	ele->l3gd20cfg[0].hspi = &hspi4;
	ele->l3gd20cfg[0].name = (char *)"L3GD20";
	ele->l3gd20cfg[0].Dir  = (char *)"+X+Y+Z"; //右手坐标系
	ele->l3gd20cfg[0].FiltNum = 5;
	ele->l3gd20cfg[0].GyrOffO[0] = 0;
	ele->l3gd20cfg[0].GyrOffO[1] = 0;
	ele->l3gd20cfg[0].GyrOffO[2] = 0;
	ele->l3gd20cfg[0].GyrOffT[0] = 0;
	ele->l3gd20cfg[0].GyrOffT[1] = 0;
	ele->l3gd20cfg[0].GyrOffT[2] = 0;
	L3g_Init_Para(l3g, ele->l3gd20cfg);
	
	ele->lsm303dcfg[0].hspi = &hspi4;
	ele->lsm303dcfg[0].name = (char *)"LSM303D";
	ele->lsm303dcfg[0].Dir  = (char *)"-X+Y-Z"; //右手坐标系
	ele->lsm303dcfg[0].FiltNum = 5;
	ele->lsm303dcfg[0].OneG = 9.875f; //9.71f
	ele->lsm303dcfg[0].AccOff[0] = 0;		//【待修改】
	ele->lsm303dcfg[0].AccOff[1] = 0;
	ele->lsm303dcfg[0].AccOff[2] = 0;
	ele->lsm303dcfg[0].MagOff[0] = 0;	//【1号机】1102，MagData1b （朝北时有?°左右的偏差）
	ele->lsm303dcfg[0].MagOff[1] = 0;
	ele->lsm303dcfg[0].MagOff[2] = 0;
	float Mat[9] = {1.7587645,   0.2458849,   0.1823861,
					0.2458849,   1.8875928,  -0.0562108,
					0.1823861,  -0.0562108,   1.9296080}; //[1号机]
//	float Mat[9] = {1.7450434,   0.2368298,   0.1819922,
//					0.2368298,   1.9451324,  -0.0547358,
//					0.1819922,  -0.0547358,   1.9609632}; //[2号机]
	ele->lsm303dcfg[0].MagMat = Mat;
	Lsm_Init_Para(lsm, ele->lsm303dcfg);

	ele->gpscfg[0].huart = &huart6;
	ele->gpscfg[0].name = (char *)"GPS";
	ele->gpscfg[0].CheckNum = 40;       //200ms
	ele->gpscfg[0].Dir = (char *)"+X-Y-Z";  //?????
//	ele->gpscfg[0].MagOff[0] = 555.0164f;
//	ele->gpscfg[0].MagOff[1] = 600.5846f;
//	ele->gpscfg[0].MagOff[2] = -921.0773f;//..2019.10.16 旧RTK
//	ele->gpscfg[0].MagOff[0] = 2490.5f;
//	ele->gpscfg[0].MagOff[1] = 2643.7f;
//	ele->gpscfg[0].MagOff[2] = -458.1f;	//RTK3		
//	ele->gpscfg[0].MagOff[0] = -958.2933f;
//	ele->gpscfg[0].MagOff[1] = -240.5982f;
//	ele->gpscfg[0].MagOff[2] = -436.0148f;	//RTK1
					
//	ele->gpscfg[0].MagOff[0] = 3627.6f;
//	ele->gpscfg[0].MagOff[1] = 2898.6f;
//	ele->gpscfg[0].MagOff[2] = -1588.2f;	//RTK4	

	ele->gpscfg[0].MagOff[0] = -657.9f;
	ele->gpscfg[0].MagOff[1] = 2493.9f;
	ele->gpscfg[0].MagOff[2] = -15.1f;	//RTK4	
	
	Gps_Init_Para(gps,ele->gpscfg);
	
	ele->ms5611cfg[0].hspi = &hspi2;
	ele->ms5611cfg[0].name = (char *)"MS5611[0]";
	ele->ms5611cfg[0].FiltNum = 3;       
	ele->ms5611cfg[0].Delay = 0;     
	Ms5611_Init_Para(&ms5611[0],&ele->ms5611cfg[0]);
	
//	ele->ms5611cfg[1].hspi = &hspi4;
//	ele->ms5611cfg[1].name = (char *)"MS5611[1]";
//	ele->ms5611cfg[1].FiltNum = 3;   
//	ele->ms5611cfg[1].Delay = 10;      
//	Ms5611_Init_Para(&ms5611[1],&ele->ms5611cfg[1]);
//	
	ele->ahrscfg[0].name = (char *)"AHRS[0]";
	ele->ahrscfg[0].AngOff[0] = 0.0f;   //往左飘增大该系数
	ele->ahrscfg[0].AngOff[1] = 0.0f;   //往前飘增大该系数
	ele->ahrscfg[0].AngOff[2] = 0.0f;
	ele->ahrscfg[0].halfDt = 0.0025f;
	ele->ahrscfg[0].mode = FLY_HEL;     //固定翼模式
	Ahrs_Init_Para(&ahrs[0],&ele->ahrscfg[0]);
	
	ele->ahrscfg[1].name = (char *)"AHRS[1]";
	ele->ahrscfg[1].AngOff[0] = 0.0f;   //往左飘增大该系数
	ele->ahrscfg[1].AngOff[1] = 0.0f;   //往前飘增大该系数
	ele->ahrscfg[1].AngOff[2] = 0.0f;
	ele->ahrscfg[1].halfDt = 0.0025f;
	ele->ahrscfg[1].mode = FLY_HEL;     //直升机模式
	Ahrs_Init_Para(&ahrs[1],&ele->ahrscfg[1]);
	
	ele->pv_ekfcfg[1].name = (char *)"PV_EKF[0]";
	ele->pv_ekfcfg[1].dt = 0.01f;
	EKF_PV_Init_Para(&pv_ekf[0], &ele->pv_ekfcfg[0]);
//	

//////	ele->motcfg[0].htim = &htim3;
//////	ele->motcfg[0].name = (char *)"MOTOR";
////////	ele->motcfg[0].PwmOff[0] = 0;
////////	ele->motcfg[0].PwmOff[1] = -1;
////////	ele->motcfg[0].PwmOff[2] = 5;
////////	ele->motcfg[0].PwmOff[3] = 10;
//////	Mot_Init_Para(mot,ele->motcfg);
	ele->motcfg[0].htimA = &htim4;  //左4 中1 右5
	ele->motcfg[0].htimB = &htim3;
	ele->motcfg[0].name = (char *)"MOTOR";
	ele->motcfg[0].PwmOff[0] = 0;
	ele->motcfg[0].PwmOff[1] = 0;
	ele->motcfg[0].PwmOff[2] = 0;
	ele->motcfg[0].PwmOff[3] = 0;
	ele->motcfg[0].PwmOff[4] = 0;
	ele->motcfg[0].PwmOff[5] = 0;
	ele->motcfg[0].PwmOff[6] = 0;
	ele->motcfg[0].PwmOff[7] = 0;
	Mot_Init_Para(mot,ele->motcfg);
	
//	
//	ele->cmdcfg[0].huart = &huart7;
//	ele->cmdcfg[0].name = (char *)"CMD";
//	ele->cmdcfg[0].CheckNum = 200;           //1s
//	Cmd_Init_Para(cmd,ele->cmdcfg);
//	
	ele->rccfg[0].huart = &huart4;
	ele->rccfg[0].name = (char *)"RC";
	ele->rccfg[0].CheckNum = 6;           //30ms
	ele->rccfg[0].ModeNum = 60;           //1.5s，连续多少个遥控器数据包
	ele->rccfg[0].LOW_THR = 1088;
	ele->rccfg[0].MID_THR = 1420;
	ele->rccfg[0].HIG_THR = 1912;
	Rc_Init_Para(rc,ele->rccfg);

	ele->exitcfg[0].name = (char *)"EXIT";
	ele->exitcfg[0].CheckNum[0] = 10;         //50ms
	ele->exitcfg[0].CheckNum[1] = 4;         //20ms
	ele->exitcfg[0].CheckNum[2] = 4;         //20ms
	ele->exitcfg[0].FiltNum[0] = 5;
	ele->exitcfg[0].FiltNum[1] = 5;
	ele->exitcfg[0].FiltNum[2] = 15;
	Exit_Init_Para(exitx,ele->exitcfg);
	
	ele->trancfg[0].huart = &huart3;
	ele->trancfg[0].name = (char *)"TRAN";
	Tran_Init_Para(tran,ele->trancfg);
	
	ele->bellcfg[0].hadc = &hadc1;
	ele->bellcfg[0].htim = &htim1;
	ele->bellcfg[0].name = (char *)"BELL";
	ele->bellcfg[0].chn = TIM_CHANNEL_4;
	ele->bellcfg[0].FiltNum = 20;
	Bell_Init_Para(bell,ele->bellcfg);
//	
//	//optiflow
//	ele->flowcfg[0].huart = &huart8;
//	ele->flowcfg[0].name = (char *)"OPTIFLOW";
//	ele->flowcfg[0].Dir = (char *)"+X-Y-Z";
//	ele->flowcfg[0].CheckNum = 6;
//	ele->flowcfg[0].dt = 0.005;
//	Flow_Init_Para(flow,ele->flowcfg);

	ele->esocfg[0].name = (char *)"ESO";
	ele->esocfg[0].Dt = 0.01f;//步长
	ele->esocfg[0].e = 0.05f;//增益
	ele->esocfg[0].ai = 0.8f;//幂函数参数
	ele->esocfg[0].kcs = 2.5f;//输入系数矩阵增益
	ele->esocfg[0].A11 = 0.7357f;//舵机近似模型为阻尼=0.9999，时间常数T=0.01的二阶系统，零阶保持离散化
	ele->esocfg[0].A12 = 0.003679f;
	ele->esocfg[0].A21 = -36.79f;
	ele->esocfg[0].A22 = 1.226e-05f;
	
	ele->esocfg[0].B11 =  0.2643f;
	ele->esocfg[0].B21 =  36.79f;
	ele->esocfg[0].L_1 =  0.17078793f;
	ele->esocfg[0].L_2 =  0.06647954f;
	ele->esocfg[0].beta[0] = 3;//beta应使得H阵是hurwitz的(特征值为负实部)
	ele->esocfg[0].beta[1] = 3; 
	ele->esocfg[0].beta[2] = 1; 
	Eso_Init_Para(eso,ele->esocfg);

	
	//地面站PID1
	ele->pidcfg[0].Kp = 0.4f;
	ele->pidcfg[0].Ki = 0.32f;
	ele->pidcfg[0].Kd = 0.0f;
	ele->pidcfg[0].Kb = 0.0f;
	ele->pidcfg[0].eLimit = 4*PI;
	ele->pidcfg[0].iLimit = 20*D2R;
	ele->pidcfg[0].dLimit = PI;
	Pid_Init_Para(pidRolRate,&ele->pidcfg[0]);
	//2
	ele->pidcfg[1].Kp = 0.4f;
	ele->pidcfg[1].Ki = 0.32f;
	ele->pidcfg[1].Kd = 0.0f;
	ele->pidcfg[1].Kb = 0.0f;
	ele->pidcfg[1].eLimit = 4*PI;
	ele->pidcfg[1].iLimit = 20*D2R;
	ele->pidcfg[1].dLimit = PI;
	Pid_Init_Para(pidPitRate,&ele->pidcfg[1]);
	//3
	ele->pidcfg[2].Kp = 0.25f;
	ele->pidcfg[2].Ki = 0.5f;
	ele->pidcfg[2].Kd = 0.0f;
	ele->pidcfg[2].Kb = 0.0f;
	ele->pidcfg[2].eLimit = 4*PI;
	ele->pidcfg[2].iLimit = 20*D2R;
	ele->pidcfg[2].dLimit = PI;
	Pid_Init_Para(pidYawRate,&ele->pidcfg[2]);
	//4
	ele->pidcfg[3].Kp = 7.0f;//9
	ele->pidcfg[3].Ki = 0.0f;
	ele->pidcfg[3].Kd = 0.0f;
	ele->pidcfg[3].Kb = 0.0f;
	ele->pidcfg[3].eLimit=PI;
	ele->pidcfg[3].iLimit = PI;
	Pid_Init_Para(pidRol,&ele->pidcfg[3]);
	//5
	ele->pidcfg[4].Kp = 7.0f;//9
	ele->pidcfg[4].Ki = 0.0f;
	ele->pidcfg[4].Kd = 0.0f;
	ele->pidcfg[4].Kb = 0.0f;
	ele->pidcfg[4].eLimit = PI;
	ele->pidcfg[4].iLimit = PI;
	Pid_Init_Para(pidPit,&ele->pidcfg[4]);
	//6
	ele->pidcfg[5].Kp = 5.0f;//6
	ele->pidcfg[5].Ki = 0.2f;
	ele->pidcfg[5].Kd = 0.0f;
	ele->pidcfg[5].Kb = 0.0f;
	ele->pidcfg[5].eLimit = PI;
	ele->pidcfg[5].iLimit = PI;
	Pid_Init_Para(pidYaw,&ele->pidcfg[5]);
	
	
	//7，x速度
	ele->pidcfg[6].Kp = 1.0;//0.8
	ele->pidcfg[6].Ki = 0.1;//0.5
	ele->pidcfg[6].Kd = 0.0f;
	ele->pidcfg[6].Kb = 0.0f;
	ele->pidcfg[6].eLimit = 20;
	ele->pidcfg[6].iLimit = 20;
	ele->pidcfg[6].dLimit = 20;
	Pid_Init_Para(pidXRate,&ele->pidcfg[6]);
	//8，y速度
	ele->pidcfg[7].Kp = 1.0;//0.8
	ele->pidcfg[7].Ki = 0.1;//0.5
	ele->pidcfg[7].Kd = 0.0f;
	ele->pidcfg[7].Kb = 0.0f;
	ele->pidcfg[7].eLimit = 20;
	ele->pidcfg[7].iLimit = 20;
	ele->pidcfg[7].dLimit = 20;
	Pid_Init_Para(pidYRate,&ele->pidcfg[7]);
	//9，z速度，没有用
	ele->pidcfg[8].Kp = 1.5f;
	ele->pidcfg[8].Ki = 0.1f;
	ele->pidcfg[8].Kd = 0.0f;
	ele->pidcfg[8].Kb = 0.0f;
	ele->pidcfg[8].eLimit = 20;
	ele->pidcfg[8].iLimit = 200;
	ele->pidcfg[8].dLimit = 20;
	Pid_Init_Para(pidZRate,&ele->pidcfg[8]);
	//10，x位置
	ele->pidcfg[9].Kp = 0.3f;
	ele->pidcfg[9].Ki = 0.0f;
	ele->pidcfg[9].Kd = 0.0f;
	ele->pidcfg[9].Kb = 0.0f;
	ele->pidcfg[9].eLimit = 200.0f;
	ele->pidcfg[9].iLimit = 200.0f;
	ele->pidcfg[9].dLimit = 200.0f;
	Pid_Init_Para(pidX,&ele->pidcfg[9]);
	//11，y位置
	ele->pidcfg[10].Kp = 0.3f;
	ele->pidcfg[10].Ki = 0.0f;
	ele->pidcfg[10].Kd = 0.0f;
	ele->pidcfg[10].Kb = 0.0f;
	ele->pidcfg[10].eLimit = 200.0f;
	ele->pidcfg[10].iLimit = 200.0f;
	ele->pidcfg[10].dLimit = 200.0f;
	Pid_Init_Para(pidY,&ele->pidcfg[10]);
	//12，z位置
	ele->pidcfg[11].Kp = 0.4f;//激光2.50f;
	ele->pidcfg[11].Ki = 0.01f;//0.20f;
	ele->pidcfg[11].Kd = 0.01f;//2.50f;
	ele->pidcfg[11].Kb = 0.0f;
	ele->pidcfg[11].eLimit = 20.0f;
	ele->pidcfg[11].iLimit = 200.0f;
	ele->pidcfg[11].dLimit = 20.0f;
	Pid_Init_Para(pidZ,&ele->pidcfg[11]);
	
	
	//温度
	ele->pidcfg[12].Kp = 300.0f;
	ele->pidcfg[12].Ki = 0.07f;
	ele->pidcfg[12].Kd = 20.02f;
	ele->pidcfg[12].filter_para = 0.557f;
	ele->pidcfg[12].setpoint=40.0f;
	Pid_Init_Para(pidTmp,&ele->pidcfg[12]);
	
	ele->pidcfg[13].Kp = 0.0f;
	ele->pidcfg[13].Ki = 0.02f;
	ele->pidcfg[13].Kd = 0.0f;
	ele->pidcfg[13].Kb = 0.0f;
	ele->pidcfg[13].eLimit = 200.0f;
	ele->pidcfg[13].iLimit = 200.0f;
	ele->pidcfg[13].dLimit = 200.0f;
	Pid_Init_Para(pidThrust,&ele->pidcfg[13]);
	
	//发送给SERVO的PWM
//	ele->servocfg[0].huart = &huart4;
//	ele->servocfg[0].name = (char *)"SERVO";
//	ele->servocfg[0].CheckNum  = 6;
//	Servo_Init_Para(servo,ele->servocfg);
	
	Control_Init();
}

/******************************END OF FILE************************************/

