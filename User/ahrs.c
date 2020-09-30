#include "ahrs.h"

/***************************************************\
功能：
  1.惯导系统，欧拉角姿态解算，四元数解析方式
//----------------------MCH-2018-10-10------------------------------------
	2.EKF姿态估计，参考秦永元卡尔曼滤波与组合导航系统第3版 
	3.ESKF姿态估计，参考Quaternion kinematics for the error-state KF
	
说明：
  1、采用欧拉角表示，四元数计算方法
  2、PI互补滤波
//----------------------MCH-2018-10-10------------------------------------
	3.乘性四元数，即估计误差模型的状态运用四元数乘法得到估计的状态
	4.矩阵求逆，采用LU分解
\***************************************************/

sAHRS ahrs[AHRS_NUM];
//--------------------------EKF 姿态 变量----------------------------
float sinR,cosR,sinP,cosP,cosY,sinY;
/*********EKF 姿态 变量************/

/******************功能函数****************/
void Ahrs_Init_Para(sAHRS *ele,sAHRS_CFG *elecfg)
{
	ele->name = elecfg->name;
	ele->Update = false;
	ele->halfDt = elecfg->halfDt;
	for(u8 i=0;i<3;i++)
	{
		ele->pqr[i]=0.0f;
		ele->Ang[i]=0.0f;
		ele->Acc[i] = 0.0f;
		ele->Uvw[i] = 0.0f;
		ele->Xyz[i] = 0.0f;
		ele->AngOff[i]=elecfg->AngOff[i];
	}
	memset(ele->LLS,0,9);   //数组置零
	if(elecfg->mode == FLY_HEL)
	{
		ele->LLS[0][0]=1.0f;
		ele->LLS[1][1]=1.0f;
		ele->LLS[2][2]=1.0f;
	}
	else
	{
		ele->LLS[0][2]=-1.0f;
		ele->LLS[1][1]=1.0f;
		ele->LLS[2][0]=1.0f;
	}
	arm_mat_init_f32(&ele->mat33,3,3,ele->LLS[0]);   //LLS[0]表示LLS（首地址）  
	ele->w = 1;
	ele->x = 0;
	ele->y = 0;
	ele->z = 0;
	ele->exInt = 0.0f;
	ele->eyInt = 0.0f;
	ele->ezInt = 0.0f;
	ele->Time = 0;
	ele->Cnt=0;
	
	ele->PosOffsetBody[0] = 0.0f;
	ele->PosOffsetBody[1] = 0.0f;
	ele->PosOffsetBody[2] = -0.15f;	//..GPS相对IMU的位置（机体系）
	
	ele->PosOffsetNed[0] = 0.0f;
	ele->PosOffsetNed[1] = 0.0f;
	ele->PosOffsetNed[2] = 0.0f;
	
	ele->VelOffsetNed[0] = 0.0f;
	ele->VelOffsetNed[1] = 0.0f;
	ele->VelOffsetNed[2] = 0.0f;	
	
	ele->PosNedCor[0] = 0.0f;
	ele->PosNedCor[1] = 0.0f;
	ele->PosNedCor[2] = 0.0f;	
	
	ele->VelNedCor[0] = 0.0f;
	ele->VelNedCor[1] = 0.0f;
	ele->VelNedCor[2] = 0.0f;	
	
	ele->gyr_cnt = 0;
	
	ele->acc_buffer_cnt = 0;
	ele->Acc_Fil_cnt = 0;
	for(u8 i=0;i<200;i++)
	{
		for(u8 j=0;j<3;j++)
		{
			ele->acc_buffer[j][i] = 0.0f;
			ele->Acc_Fil_buffer[j][i] = 0.0f;
		}
	}
	
	ele->circle_loop = 0;
}

bool Ahrs_Init(sAHRS *ele)
{
	Dprintf("\r\n%s Init...\r\n",ele->name);
	HAL_Delay(50);    //等地信号来临
	Icm20602_Calc(&ICM20602[0]);
	Gps_Calc(gps);
	if(ICM20602->Update==false || gps->MagUpdate==false)return false;
	//滤波后的信号xxxFil经过方向变换，得到Datxxx
	arm_mat_init_f32(&ele->matA,3,1,ICM20602->AccFil_3nd);
	arm_mat_init_f32(&ele->matB,3,1,ele->DatAcc);
	arm_mat_mult_f32(&ele->mat33,&ele->matA,&ele->matB);
	arm_mat_init_f32(&ele->matA,3,1,ICM20602->GyrFil_2nd);
	arm_mat_init_f32(&ele->matB,3,1,ele->DatGyr);
	arm_mat_mult_f32(&ele->mat33,&ele->matA,&ele->matB);
	arm_mat_init_f32(&ele->matA,3,1,gps->MagFil);
	arm_mat_init_f32(&ele->matB,3,1,ele->DatMag);
	arm_mat_mult_f32(&ele->mat33,&ele->matA,&ele->matB);
	//-----------------------	AccFil_3nd校准---------------------
//	ele->DatAcc[0] = 0.999909f*(ele->DatAcc[0] +0.128785f);
//	ele->DatAcc[1] = 1.000408f*(ele->DatAcc[1] +0.482182f);
//	ele->DatAcc[2] = 1.000698f*(ele->DatAcc[2] -0.315462f);
//-----------------------	AccFil校准---------------------
//	ele->DatAcc[0] = 0.997810f*(ele->DatAcc[0] +0.094557f);
//	ele->DatAcc[1] = 1.002028f*(ele->DatAcc[1] +0.451319f);
//	ele->DatAcc[2] = 1.001173f*(ele->DatAcc[2] -0.313092f);
		//----------------------实验桶加速度计校正AccFil-------------------/
//  ele->DatAcc[0] = 1.010117f*(ele->DatAcc[0] -0.411136f);
//	ele->DatAcc[1] = 1.004138f*(ele->DatAcc[1] -0.089136f);
//	ele->DatAcc[2] = 1.005333f*(ele->DatAcc[2] -0.016428f);
	//欧拉角计算，
//	float sinR,cosR,sinP,cosP,cosY,sinY;
	ele->Ang[0] = atan2f(ele->DatAcc[1],ele->DatAcc[2]);
	ele->Ang[1] = -asinf(ele->DatAcc[0]/ICM20602->OneG);
	arm_sin_cos_f32(ele->Ang[0]*R2D,&sinR,&cosR);
	arm_sin_cos_f32(ele->Ang[1]*R2D,&sinP,&cosP);
	ele->Ang[2] = atan2f(-ele->DatMag[1]*cosR + ele->DatMag[2]*sinR,
	ele->DatMag[0]*cosP + ele->DatMag[1]*sinP*sinR + ele->DatMag[2]*sinP*cosR);
	//欧拉角转四元数
	arm_sin_cos_f32(ele->Ang[0]/2.0f*R2D,&sinR,&cosR);
	arm_sin_cos_f32(ele->Ang[1]/2.0f*R2D,&sinP,&cosP);
	arm_sin_cos_f32(ele->Ang[2]/2.0f*R2D,&sinY,&cosY);
	ele->w=cosY*cosP*cosR+sinY*sinP*sinR;
	ele->x=cosY*cosP*sinR-sinY*sinP*cosR;
	ele->y=cosY*sinP*cosR+sinY*cosP*sinR;
	ele->z=sinY*cosP*cosR-cosY*sinP*sinR;
	ele->Cb2n[0][0] = 1.0f-2.0f*(ele->y*ele->y + ele->z*ele->z);
	ele->Cb2n[0][1] =      2.0f*(ele->x*ele->y - ele->w*ele->z);
	ele->Cb2n[0][2] =      2.0f*(ele->x*ele->z + ele->w*ele->y);
	ele->Cb2n[1][0] =      2.0f*(ele->x*ele->y + ele->w*ele->z);
	ele->Cb2n[1][1] = 1.0f-2.0f*(ele->x*ele->x + ele->z*ele->z);
	ele->Cb2n[1][2] =      2.0f*(ele->y*ele->z - ele->w*ele->x);
	ele->Cb2n[2][0] =      2.0f*(ele->x*ele->z - ele->w*ele->y);
	ele->Cb2n[2][1] =      2.0f*(ele->y*ele->z + ele->w*ele->x);
	ele->Cb2n[2][2] = 1.0f-2.0f*(ele->x*ele->x + ele->y*ele->y);
	if(Ahrs_Calc(ele)==false)
	{
		Dprintf("%s Init.[NO]\r\n",ele->name);
		return false;
	}
	Dprintf("%s Init.[OK]\r\n",ele->name);
	return true;
}

bool Ahrs_Calc(sAHRS *ele)
{
	Tim_Calc(&ele->Tim);   //计时
	//传感器值获取
	if(ICM20602->Update==true)
	{
		arm_mat_init_f32(&ele->matA,3,1,ICM20602->AccFil_3nd);   //AccFil和matA挂钩
		arm_mat_init_f32(&ele->matB,3,1,ele->DatAcc);   //DatAcc和matB挂钩（同步变化）
		arm_mat_mult_f32(&ele->mat33,&ele->matA,&ele->matB);    //matB=mat33*matA，也即DatAcc=mat33*AccFil。将AccFil进行坐标变换得到DatAcc
		arm_mat_init_f32(&ele->matA,3,1,ICM20602->GyrFil_2nd);
		arm_mat_init_f32(&ele->matB,3,1,ele->DatGyr);
		arm_mat_mult_f32(&ele->mat33,&ele->matA,&ele->matB);
//		adis->Update = false;
	}
	if(gps->MagUpdate==true)
	{
		arm_mat_init_f32(&ele->matA,3,1,gps->MagFil);
		arm_mat_init_f32(&ele->matB,3,1,ele->DatMag);
		arm_mat_mult_f32(&ele->mat33,&ele->matA,&ele->matB);
//		gps->MagUpdate =false;
	}
	//-----------------------	AccFil_3nd校准---------------------
//	ele->DatAcc[0] = 0.999909f*(ele->DatAcc[0] +0.128785f);
//	ele->DatAcc[1] = 1.000408f*(ele->DatAcc[1] +0.482182f);
//	ele->DatAcc[2] = 1.000698f*(ele->DatAcc[2] -0.315462f);
	//-----------------------	AccFil校准---------------------
//	ele->DatAcc[0] = 0.997810f*(ele->DatAcc[0] +0.094557f);
//	ele->DatAcc[1] = 1.002028f*(ele->DatAcc[1] +0.451319f);
//	ele->DatAcc[2] = 1.001173f*(ele->DatAcc[2] -0.313092f);
	//----------------------实验桶加速度计校正AccFil-------------------/
//  ele->DatAcc[0] = 1.010117f*(ele->DatAcc[0] -0.411136f);
//	ele->DatAcc[1] = 1.004138f*(ele->DatAcc[1] -0.089136f);
//	ele->DatAcc[2] = 1.005333f*(ele->DatAcc[2] -0.016428f);
	/***************角速度获取*****************/
	ele->pqr[0] = ele->DatGyr[0];
	ele->pqr[1] = ele->DatGyr[1];
	ele->pqr[2] = ele->DatGyr[2];
	/****************角度获取******************/
	// 重力实测的重力向量
	float ax = ele->DatAcc[0], ay = ele->DatAcc[1], az = ele->DatAcc[2];
	float recipNorm_a = invSqrt(ax*ax + ay*ay + az*az);
	ax *= recipNorm_a;
	ay *= recipNorm_a;
	az *= recipNorm_a;
	// 上一个四元数解算出来的重力向量，伪常量。Cb2n*gb=gn=[0;0;g]推出
	//gb=Cn2b*gn=Cn2b最后一列乘以g即Cb2n最后一行乘以g
	float vx = ele->Cb2n[2][0];
	float vy = ele->Cb2n[2][1];
	float vz = ele->Cb2n[2][2];
	// 规范化测量的磁场强度（实测）
	float mx = ele->DatMag[0], my = ele->DatMag[1], mz = ele->DatMag[2];
	float recipNorm_h = invSqrt(mx*mx + my*my + mz*mz);
	mx *= recipNorm_h;
	my *= recipNorm_h;
	mz *= recipNorm_h;


	// 计算参考地磁方向，伪常量（ 上一个四元数解算出来）
	float hx = mx*ele->Cb2n[0][0] + my*ele->Cb2n[0][1] + mz*ele->Cb2n[0][2];
	float hy = mx*ele->Cb2n[1][0] + my*ele->Cb2n[1][1] + mz*ele->Cb2n[1][2];
	float hz = mx*ele->Cb2n[2][0] + my*ele->Cb2n[2][1] + mz*ele->Cb2n[2][2];
	float bx = sqrtf((hx*hx) + (hy*hy));
	float bz = hz;
	// 计算常量在机体坐标下的表示。m到h到b到r，参考https://blog.csdn.net/hongbin_xu/article/details/59110226
	float rx = bx*ele->Cb2n[0][0] + bz*ele->Cb2n[2][0];
	float ry = bx*ele->Cb2n[0][1] + bz*ele->Cb2n[2][1];
	float rz = bx*ele->Cb2n[0][2] + bz*ele->Cb2n[2][2];	
	// pi参数控制
	const static float Kp = 0.2f;   // 0.2f
	const static float Ki = 0.00005f;  //0.00005f
//	float Kp = pidX->signal;
//	float Ki = pidY->signal;
	// 构造增量旋转。 
	float gx = ele->DatGyr[0], gy = ele->DatGyr[1], gz = ele->DatGyr[2]; //单位：rad/s
//	float dx, dy, dz;
	float compass = 1.0f;
	float acc = 1.0f;
	if(gps->MagUpdate == false)compass = 0.0f;
	float ex = (ay*vz - az*vy)*acc;         //	float ex = (ay*vz - az*vy)*acc + (my*rz - mz*ry)*compass;
	float ey = (az*vx - ax*vz)*acc;         //	float ey = (az*vx - ax*vz)*acc + (mz*rx - mx*rz)*compass;
	float ez = (mx*ry - my*rx)*compass;     //	float ez = (ax*vy - ay*vx)*acc + (mx*ry - my*rx)*compass;
	ele->exInt += ex;
	ele->eyInt += ey;
	ele->ezInt += ez;
	//---------------------[2018-9-25]修改-mch	---------------------------
//	dx = (gx + ex*Kp + ele->exInt*Ki)*ele->halfDt;
//	dy = (gy + ey*Kp + ele->eyInt*Ki)*ele->halfDt;
//	dz = (gz + ez*Kp + ele->ezInt*Ki)*ele->halfDt;
		gx = (gx + ex*Kp + ele->exInt*Ki);
		gy = (gy + ey*Kp + ele->eyInt*Ki);
		gz = (gz + ez*Kp + ele->ezInt*Ki);
//---------------------------------------------------------------------------
	// 融合，四元数乘法。[注][2016-1-18]实质上也是四元素微分方程的毕卡求解法之一阶近似算法
	//(q(k+1)-q(k))/Dt=1/2*[0 -wx -wy -wz;wx 0 wz -wy;wz wy -wx 0]*q(k) 
	//q(k+1)= q(k) + [0 -wx -wy -wz;wx 0 wz -wy;wz wy -wx 0]*q(k)*halfDt ,其中w即补偿过的陀螺仪数据g , halfDt=Dt/2
	float q0 = ele->w;
	float q1 = ele->x;
	float q2 = ele->y;
	float q3 = ele->z;
//---------------------[2018-9-25]修改-mch----------------------------------------
//	ele->w = q0    - q1*dx - q2*dy - q3*dz; 
//	ele->x = q0*dx + q1    + q2*dz - q3*dy;
//	ele->y = q0*dy - q1*dz + q2    + q3*dx;
//	ele->z = q0*dz + q1*dy - q2*dx + q3;
		ele->w = q0 + (-q1*gx - q2*gy - q3*gz)*ele->halfDt; 
		ele->x = q1 + ( q0*gx + q2*gz - q3*gy)*ele->halfDt;
		ele->y = q2 + ( q0*gy - q1*gz + q3*gx)*ele->halfDt;
		ele->z = q3 + ( q0*gz + q1*gy - q2*gx)*ele->halfDt;//之前ele->z = q3 + ( q0*gz + q1*gy - q2*dx)*ele->halfDt有问题
//---------------------------------------------------------------------------------
	// 规范化四元数，防止发散。
	float recipNorm_q = invSqrt(ele->w*ele->w + ele->x*ele->x + ele->y*ele->y + ele->z*ele->z);
	ele->w *= recipNorm_q;
	ele->x *= recipNorm_q;
	ele->y *= recipNorm_q;
	ele->z *= recipNorm_q;
	//坐标系转换（更新旋转矩阵）
	ele->Cb2n[0][0] = 1.0f-2.0f*(ele->y*ele->y + ele->z*ele->z);
	ele->Cb2n[0][1] =      2.0f*(ele->x*ele->y - ele->w*ele->z);
	ele->Cb2n[0][2] =      2.0f*(ele->x*ele->z + ele->w*ele->y);
	ele->Cb2n[1][0] =      2.0f*(ele->x*ele->y + ele->w*ele->z);
	ele->Cb2n[1][1] = 1.0f-2.0f*(ele->x*ele->x + ele->z*ele->z);
	ele->Cb2n[1][2] =      2.0f*(ele->y*ele->z - ele->w*ele->x);
	ele->Cb2n[2][0] =      2.0f*(ele->x*ele->z - ele->w*ele->y);
	ele->Cb2n[2][1] =      2.0f*(ele->y*ele->z + ele->w*ele->x);
	ele->Cb2n[2][2] = 1.0f-2.0f*(ele->x*ele->x + ele->y*ele->y);
	//四元数转欧拉角
	ele->Ang[0] =  atan2f(2.0f * ele->y * ele->z + 2.0f * ele->w * ele->x, -2.0f * ele->x * ele->x - 2.0f * ele->y * ele->y + 1.0f) - ele->AngOff[0];
	ele->Ang[1] =  asinf(fConstrain(-2.0f * ele->x * ele->z + 2.0f * ele->w * ele->y,-1.0f,1.0f)) - ele->AngOff[1];
	ele->Ang[2] =  atan2f(2.0f * ele->x * ele->y + 2.0f * ele->w * ele->z, -2.0f * ele->y * ele->y - 2.0f * ele->z * ele->z + 1.0f) - ele->AngOff[2];

	arm_sin_cos_f32(ele->Ang[0]*R2D,&sinR,&cosR);
	arm_sin_cos_f32(ele->Ang[1]*R2D,&sinP,&cosP);
	ele->yaw_mag = atan2f(-ele->DatMag[1]*cosR + ele->DatMag[2]*sinR,
	ele->DatMag[0]*cosP + ele->DatMag[1]*sinP*sinR + ele->DatMag[2]*sinP*cosR);
//重力夹角
	float sinY,cosY;
	arm_sin_cos_f32(ahrs->Ang[2]*R2D,&sinY,&cosY);
	float Dt = 2.0f*ele->halfDt;
	ele->Fac = fConstrain(1.0f/ele->Cb2n[2][2] - 1.0f,0.0f,0.7f); //相当于1/(cos(r)cos(p))-1
	/*************NED加速度计算**************/
//	ele->AccB[0]=-ICM20602->AccFil_3nd[0];ele->AccB[1]=-ICM20602->AccFil_3nd[1];ele->AccB[2]=-ICM20602->AccFil_3nd[2];//by czh
	ele->AccB[0]=-ICM20602->AccFil[0];ele->AccB[1]=-ICM20602->AccFil[1];ele->AccB[2]=-ICM20602->AccFil[2];//by czh
//  ele->AccB[0]=-ICM20602->AccRel[0];ele->AccB[1]=-ICM20602->AccRel[1];ele->AccB[2]=-ICM20602->AccRel[2];
	ele->Acc[0] = ele->AccB[0]*ahrs[1].Cb2n[0][0] + ele->AccB[1]*ahrs[1].Cb2n[0][1] + ele->AccB[2]*ahrs[1].Cb2n[0][2];
	ele->Acc[1] = ele->AccB[0]*ahrs[1].Cb2n[1][0] + ele->AccB[1]*ahrs[1].Cb2n[1][1] + ele->AccB[2]*ahrs[1].Cb2n[1][2];
	ele->Acc[2] = ele->AccB[0]*ahrs[1].Cb2n[2][0] + ele->AccB[1]*ahrs[1].Cb2n[2][1] + ele->AccB[2]*ahrs[1].Cb2n[2][2]+ICM20602->OneG;

//if(GpsFlag)
//{
//		/*********杠杆效应GPS位置速度补偿******/
//	ele->PosOffsetNed[0] = ele->PosOffsetBody[0]*ahrs[1].Cb2n[0][0] + ele->PosOffsetBody[1]*ahrs[1].Cb2n[0][1] + ele->PosOffsetBody[2]*ahrs[1].Cb2n[0][2];
//	ele->PosOffsetNed[1] = ele->PosOffsetBody[0]*ahrs[1].Cb2n[1][0] + ele->PosOffsetBody[1]*ahrs[1].Cb2n[1][1] + ele->PosOffsetBody[2]*ahrs[1].Cb2n[1][2];
//	ele->PosOffsetNed[2] = ele->PosOffsetBody[0]*ahrs[1].Cb2n[2][0] + ele->PosOffsetBody[1]*ahrs[1].Cb2n[2][1] + ele->PosOffsetBody[2]*ahrs[1].Cb2n[2][2];

//for(int i=0;i<3;i++)
//{
//  ele->gyr_buffer[i][ele->gyr_cnt] = ele->DatGyr[i] * 2;
//}
//	ele->gyr_cnt++;
//	if(ele->gyr_cnt == 60)
//	{
//		ele->gyr_cnt = 0;
//	}

//	ele->VelOffsetBody[0] =  0.0f * ele->PosOffsetBody[0]                             - ele->gyr_buffer[2][ele->gyr_cnt] * ele->PosOffsetBody[1] + ele->gyr_buffer[1][ele->gyr_cnt] * ele->PosOffsetBody[2];
//	ele->VelOffsetBody[1] =  ele->gyr_buffer[2][ele->gyr_cnt] * ele->PosOffsetBody[0] + 0.0f * ele->PosOffsetBody[1]                             - ele->gyr_buffer[0][ele->gyr_cnt] * ele->PosOffsetBody[2];
//	ele->VelOffsetBody[2] = -ele->gyr_buffer[1][ele->gyr_cnt] * ele->PosOffsetBody[0] + ele->gyr_buffer[1][ele->gyr_cnt] * ele->PosOffsetBody[1] + 0.0f * ele->PosOffsetBody[2];
//	
//	ele->VelOffsetNed[0] = ele->VelOffsetBody[0]*ahrs[1].Cb2n[0][0] + ele->VelOffsetBody[1]*ahrs[1].Cb2n[0][1] + ele->VelOffsetBody[2]*ahrs[1].Cb2n[0][2];
//	ele->VelOffsetNed[1] = ele->VelOffsetBody[0]*ahrs[1].Cb2n[1][0] + ele->VelOffsetBody[1]*ahrs[1].Cb2n[1][1] + ele->VelOffsetBody[2]*ahrs[1].Cb2n[1][2];
//	ele->VelOffsetNed[2] = ele->VelOffsetBody[0]*ahrs[1].Cb2n[2][0] + ele->VelOffsetBody[1]*ahrs[1].Cb2n[2][1] + ele->VelOffsetBody[2]*ahrs[1].Cb2n[2][2];	
//	for(int j=0;j<3;j++)
//	{
//		ele->PosNedCor[j] = gps->NED[j] - ele->PosOffsetNed[j];
//		ele->VelNedCor[j] = gps->NED_spd[j] - ele->VelOffsetNed[j];
//	}
//}
/*************NED速度计算**************/
	float Kv[3] = {0.04f,0.04f,0.04f};
//	if(gps->ECEF_Init_Flag == false){Kv[0]=0.0f;Kv[1]=0.0f;}
//for(u8 i=0;i<3;i++)
//	{
//		ele->acc_buffer[i][ele->acc_buffer_cnt] = ele->Acc[i];
//    ele->Acc_Fil_buffer[i][ele->acc_buffer_cnt] = -ICM20602->AccFil_3nd2[i];		
//	}
//		ele->acc_buffer_cnt++;
//	if(ele->acc_buffer_cnt == 100)
//	{
//		ele->acc_buffer_cnt = 0;
//	}

	ele->Uvw[0] = (1.0f-Kv[0])*(ele->Uvw[0] + ele->Acc[0]*Dt) + Kv[0]*gps->NED_spd[0];
	ele->Uvw[1] = (1.0f-Kv[1])*(ele->Uvw[1] + ele->Acc[1]*Dt) + Kv[1]*gps->NED_spd[1];
	ele->Uvw[2] = (1.0f-Kv[2])*(ele->Uvw[2] + ele->Acc[2]*Dt) + Kv[2]*gps->NED_spd[2];
//	ele->Uvw[0] = (1.0f-Kv[0])*(ele->Uvw[0] + ele->acc_buffer[0][ele->acc_buffer_cnt]*Dt) + Kv[0]*gps->NED_spd[0];
//	ele->Uvw[1] = (1.0f-Kv[1])*(ele->Uvw[1] + ele->acc_buffer[1][ele->acc_buffer_cnt]*Dt) + Kv[1]*gps->NED_spd[1];
//	ele->Uvw[2] = (1.0f-Kv[2])*(ele->Uvw[2] + ele->acc_buffer[2][ele->acc_buffer_cnt]*Dt) + Kv[2]*gps->NED_spd[2];
//	ele->Uvw[0] = (ele->Uvw[0] + ele->acc_buffer[0][ele->acc_buffer_cnt]*Dt);
//	ele->Uvw[1] = (ele->Uvw[1] + ele->acc_buffer[1][ele->acc_buffer_cnt]*Dt);
//	ele->Uvw[2] = (ele->Uvw[2] + ele->acc_buffer[2][ele->acc_buffer_cnt]*Dt);
//	float TmpW = 0.0f, FACv=0.0f;
//	if(ms5611[0].Err == ERR_NONE && ms5611[0].Sta == STA_RUN){TmpW+=-ms5611[0].AltSlope;FACv += 1.0f;} 
//	if(ms5611[1].Err == ERR_NONE && ms5611[1].Sta == STA_RUN){TmpW+=-ms5611[1].AltSlope;FACv += 1.0f;} 
//	if(exitx->Update[2] == true && -ele->Xyz[2]<3.0f) {TmpW+=-exitx->DatSlope[1];FACv += 1.0f;}
//  if(exitx->Update[2] == true ) {TmpW+=-(exitx->DatSlopeFil[2]*ele->Cb2n[2][2]);FACv += 1.0f;}
//	if(FACv>0.0f) TmpW /= FACv;
//	else Kv[2]=0.0f;
//	ele->Uvw[2] = (1.0f-Kv[2])*(ele->Uvw[2] + ele->Acc[2]*Dt) + Kv[2]*TmpW;
	/*************NED位置计算**************/
	float Ks[3] = {0.03f,0.03f,0.03f};
//	if(gps->ECEF_Init_Flag == false){Ks[0]=0.0f;Ks[1]=0.0f;}
	ele->Xyz[0] = (1.0f-Ks[0])*(ele->Xyz[0] + ele->Uvw[0]*Dt) + Ks[0]*gps->NED[0];
	ele->Xyz[1] = (1.0f-Ks[1])*(ele->Xyz[1] + ele->Uvw[1]*Dt) + Ks[1]*gps->NED[1];
	ele->Xyz[2] = (1.0f-Ks[2])*(ele->Xyz[2] + ele->Uvw[2]*Dt) + Ks[2]*gps->NED[2];
//	float TmpZ = 0.0f, FACs=0.0f;
//	if(ms5611[0].Err == ERR_NONE && ms5611[0].Sta == STA_RUN){TmpZ+=-ms5611[0].AltSlope;FACs += 1.0f;} 
//	if(ms5611[1].Err == ERR_NONE && ms5611[1].Sta == STA_RUN){TmpZ+=-ms5611[1].AltSlope;FACs += 1.0f;} 
//	if(exitx->Update[2] == true ) {TmpZ+=-(exitx->DatSlope[2]*ele->Cb2n[2][2]);FACv += 1.0f;}
//	if(FACs>0.0f) TmpZ /= FACs;
//	else Ks[2]=0.0f;
//	FACs = (-ele->Xyz[2]-3.0f)*0.5f;
//	FACs = fConstrain(FACs,0.0f,1.0f);
	//ele->Xyz[2] = (1.0f-Ks[2])*(ele->Xyz[2] + ele->Uvw[2]*Dt) + Ks[2]*(FACs*TmpZ + (1.0f-FACs)*-exitx->DatFil[2]*ele->Cb2n[2][2]);
//	ele->Xyz[2] = (1.0f-Ks[2])*(ele->Xyz[2] + ele->Uvw[2]*Dt) + Ks[2]*(-exitx->DatFil[2]*ele->Cb2n[2][2]);//(1.0f-Ks[2])*积分 + Ks[2]*测量
  /*************机头坐标系计算**************/
	ele->AccH[0] = ele->Acc[0]*cosY + ele->Acc[1]*sinY;
	ele->AccH[1] = -ele->Acc[0]*sinY + ele->Acc[1]*cosY;
	ele->AccH[2] = ele->Acc[2];
	ele->UvwH[0] = ele->Uvw[0]*cosY + ele->Uvw[1]*sinY;
	ele->UvwH[1] = -ele->Uvw[0]*sinY + ele->Uvw[1]*cosY;
	ele->UvwH[2] = ele->Uvw[2];
	ele->XyzH[0] = ele->Xyz[0]*cosY + ele->Xyz[1]*sinY;
	ele->XyzH[1] = -ele->Xyz[0]*sinY + ele->Xyz[1]*cosY;
	ele->XyzH[2] = ele->Xyz[2];
	/****************计算结束****************/

	ele->Update = true;
	Tim_Calc(&ele->Tim);   //计时
	ele->Time = ele->Tim.OUT;
	return ele->Update;
}




//----------------------ESKF 姿态部分 2018.10 MCH----------------------------
bool ESKF_Init_MCH(sAHRS *ele)
{
	Dprintf("\r\n%s Init...\r\n",ele->name);
	HAL_Delay(50);    //等地信号来临
	Icm20602_Calc(&ICM20602[0]);
	Gps_Calc(gps);
	if(ICM20602->Update==false || gps->MagUpdate==false)return false;
	//滤波后的信号xxxFil经过方向变换，得到Datxxx
	arm_mat_init_f32(&ele->matA,3,1,ICM20602->AccRel);
	arm_mat_init_f32(&ele->matB,3,1,ele->DatAcc);
	arm_mat_mult_f32(&ele->mat33,&ele->matA,&ele->matB);
	arm_mat_init_f32(&ele->matA,3,1,ICM20602->GyrRel);
	arm_mat_init_f32(&ele->matB,3,1,ele->DatGyr);
	arm_mat_mult_f32(&ele->mat33,&ele->matA,&ele->matB);
	arm_mat_init_f32(&ele->matA,3,1,gps->MagFil);
	arm_mat_init_f32(&ele->matB,3,1,ele->DatMag);
	arm_mat_mult_f32(&ele->mat33,&ele->matA,&ele->matB);
	/***************角速度获取*****************/
	ele->pqr[0] = ele->DatGyr[0];
	ele->pqr[1] = ele->DatGyr[1];
	ele->pqr[2] = ele->DatGyr[2];
	//float w_rad[3];
	//w_rad[0] = ele->pqr[0];
	//w_rad[1] = ele->pqr[1];
	//w_rad[2] = ele->pqr[2];
	// 重力实测的重力向量
	float ax = ele->DatAcc[0], ay = ele->DatAcc[1], az = ele->DatAcc[2];
	float recipNorm_a = invSqrt(ax*ax + ay*ay + az*az);
	ax *= recipNorm_a;
	ay *= recipNorm_a;
	az *= recipNorm_a;
//	float a_norm[3];
//	a_norm[0] = ax;
//	a_norm[1] = ay;
//	a_norm[2] = az;
	// 规范化测量的磁场强度
	float mx = ele->DatMag[0], my = ele->DatMag[1], mz = ele->DatMag[2];
	float recipNorm_h = invSqrt(mx*mx + my*my + mz*mz);
	mx *= recipNorm_h;
	my *= recipNorm_h;
	mz *= recipNorm_h;
	float m_norm[3];
	m_norm[0] = mx;
	m_norm[1] = my;
	m_norm[2] = mz;
	float mag_calib[2];
	float sinR,cosR,sinP,cosP;
	ele->Ang_Init[0] = 0;
	ele->Ang_Init[1] = 0;
	arm_sin_cos_f32(ele->Ang_Init[0]*R2D,&sinR,&cosR);
	arm_sin_cos_f32(ele->Ang_Init[1]*R2D,&sinP,&cosP);
	
	mag_calib[0]=-m_norm[1]*cosR + m_norm[2]*sinR;//分子
	mag_calib[1]=m_norm[0]*cosP + m_norm[1]*sinP*sinR + m_norm[2]*sinP*cosR;//分母
	
		if (mag_calib[1] != 0)
	{
		ele->Ang_Init[2] = atan2f(mag_calib[0], mag_calib[1]);
	}
	else
	{
		if (mag_calib[0] < 0)
			ele->Ang_Init[2] = -PI/2;
		else
			ele->Ang_Init[2] = PI/2;
	}
	euler2quaternion(ele->Ang_Init,ele->q_init);	
	//基本矩阵对应的数组初始化
	memset(ele->Q_element,0,36U*sizeof(double)); 
	memset(ele->R_element,0,16U*sizeof(double)); 
	memset(ele->I_element,0,36U*sizeof(double));
	memset(ele->P_p_element,0,36U*sizeof(double)); 
	memset(ele->P_n_element,0,36U*sizeof(double)); 
	memset(ele->K_k_element,0,28U*sizeof(double)); 
	memset(ele->P_k_element,0,36U*sizeof(double)); 
		//--------------------------------
	//中间变量对应的数组初始化
	memset(ele->HT_middle_element,0,24U*sizeof(double)); 
	memset(ele->PHT_middle_element,0,24U*sizeof(double)); 
	//----------提高精度，用double-------------------------
	memset(ele->HPHT_middle_element,0,16U*sizeof(double)); 
	memset(ele->HPHT_add_R_element,0,16U*sizeof(double));
  //-----------HPHT_add_R_inv_element根据情况调整类型-----------
	memset(ele->HPHT_add_R_inv_element,0,16U*sizeof(double));
	//------------------------------------------------------------
	memset(ele->Z_sub_h_element,0,4U*sizeof(double));
	memset(ele->X_k_error_element,0,6U*sizeof(double));	
	memset(ele->G_element,0,36U*sizeof(double));
	memset(ele->GP_element,0,36U*sizeof(double));
	memset(ele->GT_element,0,36U*sizeof(double));
	memset(ele->KH_element,0,36U*sizeof(double));
	memset(ele->I_sub_KH_element,0,36U*sizeof(double));
	memset(ele->FT_element,0,36U*sizeof(double));
	memset(ele->PFT_element,0,36U*sizeof(double));
	memset(ele->FPFT_element,0,36U*sizeof(double));
	memset(ele->P_n1T_element,0,36U*sizeof(double));
	memset(ele->P_n1_element,0,36U*sizeof(double));
	memset(ele->P_n2_element,0,36U*sizeof(double));
		//卡尔曼迭代计算用矩阵数组置零
	memset(ele->H1_element,0,28U*sizeof(double)); 
	memset(ele->H2_element,0,42U*sizeof(double)); 
	memset(ele->H_element,0,24U*sizeof(double)); 
	memset(ele->h_X_p_element,0,4U*sizeof(double));
	memset(ele->Z_k_element,0,4U*sizeof(double));
	//计算转移矩阵相关的矩阵数组置零
	memset(ele->F_element,0,36U*sizeof(double));

	for (int i=0;i<6;i++)
	{
		ele->G_element[i][i]=1.0F;
		ele->P_p_element[i][i]=0.0001F;
		ele->I_element[i][i]=1.0F;	
	}
//==================================================

	//---------------Q?--------------------------------
//	ele->Q1=1.5e-08F;
//	ele->Q2=1.5e-08F;
//	ele->R1=0.06F;
//	ele->R2=0.06F;
	ele->Q1=5.5e-08F;
	ele->Q2=5.5e-08F;
	ele->R1=0.1F;
	ele->R2=0.1F;
	for (int i=0;i<3;i++)
	{
		ele->Q_element[i][i]=ele->Q1;//0.00000001F
  }
	ele->Q_element[2][2]=5.5e-04F;
	for (int i=3;i<6;i++)													
	{
		ele->Q_element[i][i]=ele->Q2;//=0.0000001F
	}
	ele->Q_element[5][5]=5.5e-04F;
//--------------------------------------------------		
//==================================================			
//==================================================
//-------------R?----------------------------------
	for (int i=0;i<3;i++)
	{
		ele->R_element[i][i]=ele->R1;//0.008F
	}
	  ele->R_element[3][3]=ele->R2;//0.0081F;0.001F
	
	
//--------------------------------------------------
//==================================================
	
	ele->X_p_q[0] = ele->q_init[0];
	ele->X_p_q[1] = ele->q_init[1];
	ele->X_p_q[2] = ele->q_init[2];
	ele->X_p_q[3] = ele->q_init[3];		
	if(ESKF_Attitude_Calc_MCH(ele)==false)
	{
		Dprintf("%s Init.[NO]\r\n",ele->name);
		return false;
	}
	Dprintf("%s Init.[OK]\r\n",ele->name);
	return true;
}

void ESKF_Attitude_MCH(double X_p_q[4],double X_p_w[3],float a_norm[3],float m_norm[3],float w_rad[3], 
												double P_p_element[][6], float halfDt,double X_k_q[4],double X_k_w[3],
													double X_n_q[4],double X_n_w[3], double P_n_element[][6])
{
	float mag_calib[2],Ang[3];
	float sinR,cosR,sinP,cosP;
	Ang[0] = atan2f(a_norm[1],a_norm[2]);
	Ang[1] = -asinf(a_norm[0]);
	arm_sin_cos_f32(Ang[0]*R2D,&sinR,&cosR);
	arm_sin_cos_f32(Ang[1]*R2D,&sinP,&cosP);
	mag_calib[0]=-m_norm[1]*cosR + m_norm[2]*sinR;//分子
	mag_calib[1]=m_norm[0]*cosP + m_norm[1]*sinP*sinR + m_norm[2]*sinP*cosR;//分母
		if (mag_calib[1] != 0)
	{
		Ang[2] = atan2f(mag_calib[0], mag_calib[1]);
	}
	else
	{
		if (mag_calib[0] < 0)
		{
			Ang[2] = -PI/2;
		}
		else
		{
			Ang[2] = PI/2;
		}
	}
	double yaw_molecule=2*(X_p_q[0]*X_p_q[3]+X_p_q[1]*X_p_q[2]);
	double yaw_Denominator=1-2*(X_p_q[2]*X_p_q[2]+X_p_q[3]*X_p_q[3]);	
	ahrs[1].H1_element[0][0]=-2*X_p_q[2];
	ahrs[1].H1_element[0][1]=2*X_p_q[3];
	ahrs[1].H1_element[0][2]=-2*X_p_q[0];
	ahrs[1].H1_element[0][3]=2*X_p_q[1];
	ahrs[1].H1_element[1][0]=2*X_p_q[1];
	ahrs[1].H1_element[1][1]=2*X_p_q[0];
	ahrs[1].H1_element[1][2]=2*X_p_q[3];
	ahrs[1].H1_element[1][3]=2*X_p_q[2];
	ahrs[1].H1_element[2][1]=-4*X_p_q[1];
	ahrs[1].H1_element[2][2]=-4*X_p_q[2];
	ahrs[1].H1_element[3][0]=2*X_p_q[3]*yaw_Denominator/(yaw_molecule*yaw_molecule+yaw_Denominator*yaw_Denominator);
	ahrs[1].H1_element[3][1]=2*X_p_q[2]*yaw_Denominator/(yaw_molecule*yaw_molecule+yaw_Denominator*yaw_Denominator);
	ahrs[1].H1_element[3][2]=(2*X_p_q[1]*yaw_Denominator+4*X_p_q[2]*yaw_molecule)/(yaw_molecule*yaw_molecule+yaw_Denominator*yaw_Denominator);
	ahrs[1].H1_element[3][3]=(2*X_p_q[0]*yaw_Denominator+4*X_p_q[3]*yaw_molecule)/(yaw_molecule*yaw_molecule+yaw_Denominator*yaw_Denominator);
	
	ahrs[1].H2_element[0][0]=-0.5f*X_p_q[1];
	ahrs[1].H2_element[0][1]=-0.5f*X_p_q[2];
	ahrs[1].H2_element[0][2]=-0.5f*X_p_q[3];
	ahrs[1].H2_element[1][0]= 0.5f*X_p_q[0];
	ahrs[1].H2_element[1][1]=-0.5f*X_p_q[3];
	ahrs[1].H2_element[1][2]= 0.5f*X_p_q[2];
	ahrs[1].H2_element[2][0]= 0.5f*X_p_q[3];
	ahrs[1].H2_element[2][1]= 0.5f*X_p_q[0];
	ahrs[1].H2_element[2][2]=-0.5f*X_p_q[1];
	ahrs[1].H2_element[3][0]=-0.5f*X_p_q[2];
	ahrs[1].H2_element[3][1]= 0.5f*X_p_q[1];
	ahrs[1].H2_element[3][2]= 0.5f*X_p_q[0];
	ahrs[1].H2_element[4][3]= 1;
	ahrs[1].H2_element[5][4]= 1;
	ahrs[1].H2_element[6][5]= 1;
	for(int i=0;i<4;i++)
	{
		for(int j=0;j<6;j++)
		{
			double  temp = 0.0f;
			for(int k = 0 ; k < 7 ; k++)
			{
					temp += ahrs[1].H1_element[i][k] * ahrs[1].H2_element[k][j];
			}
			ahrs[1].H_element[i][j] = temp;
		}
	}
//	arm_mat_trans_f32(&ahrs[1].H,&ahrs[1].HT_middle);
	for(int i=0;i<6;i++)
	{
		for(int j=0;j<4;j++)
		{	
			ahrs[1].HT_middle_element[i][j]= ahrs[1].H_element[j][i];
		}
	}
//--------------------------------------------------	
//	arm_mat_mult_f32(P_p,&ahrs[1].HT_middle,&ahrs[1].PHT_middle);
	for(int i=0;i<6;i++)
	{
		for(int j=0;j<4;j++)
		{
			double  temp = 0.0f;
			for(int k = 0 ; k < 6 ; k++)
			{
					temp += ahrs[1].P_p_element[i][k] * ahrs[1].HT_middle_element[k][j];
			}
			ahrs[1].PHT_middle_element[i][j] = temp;
		}
	}
	
	//arm_mat_mult_f32(&ahrs[1].H,&ahrs[1].PHT_middle,&ahrs[1].HPHT_middle);
	for(int i=0;i<4;i++)
	{
		for(int j=0;j<4;j++)
		{
			double  temp = 0.0f;
			for(int k = 0 ; k < 6 ; k++)
			{
					temp += ahrs[1].H_element[i][k] * ahrs[1].PHT_middle_element[k][j];
			}
			ahrs[1].HPHT_middle_element[i][j] = temp;
		}
	}
//	arm_mat_add_f32(&ahrs[1].HPHT_middle,&ahrs[1].R,&ahrs[1].HPHT_add_R);
	for(int i=0;i<4;i++)
	{
		for(int j=0;j<4;j++)
		{
				ahrs[1].HPHT_add_R_element[i][j] = ahrs[1].HPHT_middle_element[i][j] + ahrs[1].R_element[i][j];
		}
	}
	//=====================LU分解求逆=======================================================
	//---------若HPHT_add_R_inv_element为float，函数matrix_inverse_LU里需要做强制转换-------
	matrix_inverse_LU(4, ahrs[1].HPHT_add_R_element,ahrs[1].HPHT_add_R_inv_element);//
	//======================================================================================
	for(int i=0;i<6;i++)
	{
		for(int j=0;j<4;j++)
		{
			double  temp = 0.0f;
			for(int k = 0 ; k < 4 ; k++)
			{
					temp += ahrs[1].PHT_middle_element[i][k] * ahrs[1].HPHT_add_R_inv_element[k][j];
			}
			ahrs[1].K_k_element[i][j] = temp;
		}
	}
	//h_X_p=h(X(k|k-1))元素,由上一时刻的一步预测X(k|k-1)得到的 理论观测值
	ahrs[1].h_X_p_element[0]=2*(X_p_q[1]*X_p_q[3]-X_p_q[0]*X_p_q[2]);
	ahrs[1].h_X_p_element[1]=2*(X_p_q[2]*X_p_q[3]+X_p_q[0]*X_p_q[1]);
	ahrs[1].h_X_p_element[2]=1-2*(X_p_q[1]*X_p_q[1]+X_p_q[2]*X_p_q[2]);
	ahrs[1].h_X_p_element[3]=atan2f(yaw_molecule,yaw_Denominator);
	//实际观测值,即观测向量=重力加速度向量和航向角
	ahrs[1].Z_k_element[0]=a_norm[0];
	ahrs[1].Z_k_element[1]=a_norm[1];
	ahrs[1].Z_k_element[2]=a_norm[2];
	ahrs[1].Z_k_element[3]=Ang[2];
//	arm_mat_sub_f32(&ahrs[1].Z_k,&ahrs[1].h_X_p,&ahrs[1].Z_sub_h);
	for(int i=0;i<4;i++)
	{
		ahrs[1].Z_sub_h_element[i]=ahrs[1].Z_k_element[i]-ahrs[1].h_X_p_element[i];	
	}
	if (ahrs[1].Z_sub_h_element[3]>PI)
	{
		ahrs[1].Z_sub_h_element[3]=ahrs[1].Z_sub_h_element[3]-2*PI;
	}
	if (ahrs[1].Z_sub_h_element[3]<-PI)
	{
		ahrs[1].Z_sub_h_element[3]=ahrs[1].Z_sub_h_element[3]+2*PI;
	}
//	arm_mat_mult_f32(&ahrs[1].K_k,&ahrs[1].Z_sub_h,&ahrs[1].X_k_error);
	for(int i=0;i<6;i++)
	{
		double  temp = 0.0f;
		for(int k = 0 ; k < 4 ; k++)
		{
			temp += ahrs[1].K_k_element[i][k] * ahrs[1].Z_sub_h_element[k];
		}
		ahrs[1].X_k_error_element[i] = temp;
	}
	for(int i=0;i<3;i++)
	{
			ahrs[1].d_theta[i]=ahrs[1].X_k_error_element[i];
	}
	angle_axis_of_rotation(ahrs[1].d_theta, &ahrs[1].d_phi, ahrs[1].d_u);
	creat_quaternion(&ahrs[1].d_phi, ahrs[1].d_u, ahrs[1].d_q_element);
	quaternion_mul(X_p_q,  ahrs[1].d_q_element, X_k_q);//void quaternion_mul(double q1[4], double q2[4], double q[4])

	for(int i=0;i<3;i++)
	{
		X_k_w[i]=X_p_w[i]+ahrs[1].X_k_error_element[i+3];
	}
	
	for(int i=0;i<3;i++)
	{
			ahrs[1].wm_sub_wb[i]=(w_rad[i]-X_k_w[i])*0.005f;
	}
	angle_axis_of_rotation(ahrs[1].wm_sub_wb, &ahrs[1].dw_phi, ahrs[1].dw_u);
	
	creat_rotation_matrix(&ahrs[1].dw_phi, ahrs[1].dw_u, ahrs[1].Rot_element);
	
	
	creat_quaternion(&ahrs[1].dw_phi, ahrs[1].dw_u, ahrs[1].q_w);
	quaternion_mul(X_k_q, ahrs[1].q_w, X_n_q);
	X_n_w[0]= X_k_w[0];
	X_n_w[1]= X_k_w[1];
	X_n_w[2]= X_k_w[2];	
	
	ahrs[1].F_element[0][0]= ahrs[1].Rot_element[0][0];
	ahrs[1].F_element[0][1]= ahrs[1].Rot_element[1][0];
	ahrs[1].F_element[0][2]= ahrs[1].Rot_element[2][0];
	ahrs[1].F_element[0][3]= -0.005f;
	ahrs[1].F_element[1][0]= ahrs[1].Rot_element[0][1];
	ahrs[1].F_element[1][1]= ahrs[1].Rot_element[1][1];
	ahrs[1].F_element[1][2]= ahrs[1].Rot_element[2][1];
	ahrs[1].F_element[1][4]= -0.005f;
	ahrs[1].F_element[2][0]= ahrs[1].Rot_element[0][2];
	ahrs[1].F_element[2][1]= ahrs[1].Rot_element[1][2];
	ahrs[1].F_element[2][2]= ahrs[1].Rot_element[2][2];
	ahrs[1].F_element[2][5]= -0.005f;
	ahrs[1].F_element[3][3]= 1.0f;
	ahrs[1].F_element[4][4]= 1.0f;
	ahrs[1].F_element[5][5]= 1.0f;
	//arm_mat_mult_f32(&ahrs[1].K_k,&ahrs[1].H,&ahrs[1].KH);
	for(int i=0;i<6;i++)
	{
		for(int j=0;j<6;j++)
		{
			double  temp = 0.0f;
			for(int k = 0 ; k < 4 ; k++)
			{
					temp += ahrs[1].K_k_element[i][k] * ahrs[1].H_element[k][j];
			}
			ahrs[1].KH_element[i][j] = temp;
		}
	}
//	arm_mat_sub_f32(&ahrs[1].I,&ahrs[1].KH,&ahrs[1].I_sub_KH);
	for(int i=0;i<6;i++)
	{
		for(int j=0;j<6;j++)
		{
				ahrs[1].I_sub_KH_element[i][j] = ahrs[1].I_element[i][j] - ahrs[1].KH_element[i][j];
		}
	}
//	arm_mat_mult_f32(&ahrs[1].I_sub_KH,P_p,&ahrs[1].P_k);
	for(int i=0;i<6;i++)
	{
		for(int j=0;j<6;j++)
		{
			double  temp = 0.0f;
			for(int k = 0 ; k < 6 ; k++)
			{
					temp += ahrs[1].I_sub_KH_element[i][k] * ahrs[1].P_p_element[k][j];
			}
			ahrs[1].P_k_element[i][j] = temp;
		}
	}
//	//P_k=G P_k GT   加上这段会产生波动
//	double half_d_theta[3], half_d_theta_x_element[3][3];
//	for(int i=0;i<3;i++)
//	{
//		half_d_theta[i]=ahrs[1].d_theta[i]/2.0f;
//	}
//	cross_product_matrix(half_d_theta,half_d_theta_x_element);
//	for(int i=0;i<3;i++)
//	{
//		for(int j=0;j<3;j++)
//		{
//			ahrs[1].G_element[i][j]=ahrs[1].G_element[i][j]-half_d_theta_x_element[i][j];
//		}
//	}
//	for(int i=0;i<6;i++)
//	{
//		for(int j=0;j<6;j++)
//		{	
//			ahrs[1].GT_element[i][j]= ahrs[1].G_element[j][i];
//		}
//	}
//	for(int i=0;i<6;i++)
//	{
//		for(int j=0;j<6;j++)
//		{
//			double  temp = 0.0f;
//			for(int k = 0 ; k < 6 ; k++)
//			{
//					temp += ahrs[1].G_element[i][k] * ahrs[1].P_k_element[k][j];
//			}
//			ahrs[1].GP_element[i][j] = temp;
//		}
//	}
//	for(int i=0;i<6;i++)
//	{
//		for(int j=0;j<6;j++)
//		{
//			double  temp = 0.0f;
//			for(int k = 0 ; k < 6 ; k++)
//			{
//					temp += ahrs[1].GP_element[i][k] * ahrs[1].GT_element[k][j];
//			}
//			ahrs[1].P_k_element[i][j] = temp;
//		}
//	}
	
	
	//	arm_mat_trans_f32(&ahrs[1].A,&ahrs[1].AT);
	for(int i=0;i<6;i++)
	{
		for(int j=0;j<6;j++)
		{	
			ahrs[1].FT_element[i][j]= ahrs[1].F_element[j][i];
		}
	}
//	arm_mat_mult_f32(&ahrs[1].P_k,&ahrs[1].FT,&ahrs[1].PFT);
	for(int i=0;i<6;i++)
	{
		for(int j=0;j<6;j++)
		{
			double  temp = 0.0f;
			for(int k = 0 ; k < 6 ; k++)
			{
					temp += ahrs[1].P_k_element[i][k] * ahrs[1].FT_element[k][j];
			}
			ahrs[1].PFT_element[i][j] = temp;
		}
	}
//	arm_mat_mult_f32(&ahrs[1].F,&ahrs[1].PFT,&ahrs[1].FPFT);
	for(int i=0;i<6;i++)
	{
		for(int j=0;j<6;j++)
		{
			double  temp = 0.0f;
			for(int k = 0 ; k < 6 ; k++)
			{
					temp += ahrs[1].F_element[i][k] * ahrs[1].PFT_element[k][j];
			}
			ahrs[1].FPFT_element[i][j] = temp;
		}
	}
	//求平均值使P_n保持对称
	//arm_mat_scale_f32(&ahrs[1].P_n2,0.5F,P_n);
//	arm_mat_add_f32(&ahrs[1].FPFT,&ahrs[1].Q,&ahrs[1].P_n1);
	for(int i=0;i<6;i++)
	{
		for(int j=0;j<6;j++)
		{	
			ahrs[1].P_n1_element[i][j] = ahrs[1].FPFT_element[i][j] + ahrs[1].Q_element[i][j];
		}
	}
//	arm_mat_trans_f32(&ahrs[1].P_n1,&ahrs[1].P_n1T);
	for(int i=0;i<6;i++)
	{
		for(int j=0;j<6;j++)
		{	
			ahrs[1].P_n1T_element[i][j]= ahrs[1].P_n1_element[j][i];
		}
	}
	//arm_mat_add_f32(&ahrs[1].P_n1,&ahrs[1].P_n1T,&ahrs[1].P_n2);
	for(int i=0;i<6;i++)
	{
		for(int j=0;j<6;j++)
		{	
			ahrs[1].P_n_element[i][j] =( ahrs[1].P_n1_element[i][j] + ahrs[1].P_n1T_element[i][j])*0.5f;
		}
	}
  memset(ahrs[1].X_k_error_element,0,6U*sizeof(double));
}

bool ESKF_Attitude_Calc_MCH(sAHRS *ele)
{
	Tim_Calc(&ele->Tim);   //计时
	//传感器值获取
	if(ICM20602->Update==true)
	{
		arm_mat_init_f32(&ele->matA,3,1,ICM20602->AccRel);
		arm_mat_init_f32(&ele->matB,3,1,ele->DatAcc);
		arm_mat_mult_f32(&ele->mat33,&ele->matA,&ele->matB);
		arm_mat_init_f32(&ele->matA,3,1,ICM20602->GyrRel);
		arm_mat_init_f32(&ele->matB,3,1,ele->DatGyr);
		arm_mat_mult_f32(&ele->mat33,&ele->matA,&ele->matB);
	}
	if(gps->MagUpdate==true)
	{
		arm_mat_init_f32(&ele->matA,3,1,gps->MagFil);
		arm_mat_init_f32(&ele->matB,3,1,ele->DatMag);
		arm_mat_mult_f32(&ele->mat33,&ele->matA,&ele->matB);
	}
	
	ele->ahrs_update_time = ele->Tim.CNT;
	ele->gps_ins_update_flag = true;
	
	//----------------------角速度获取-----------------------/
	ele->pqr[0] = ele->DatGyr[0];
	ele->pqr[1] = ele->DatGyr[1];
	ele->pqr[2] = ele->DatGyr[2];
	float w_rad[3];
	w_rad[0] = ele->pqr[0];
	w_rad[1] = ele->pqr[1];
	w_rad[2] = ele->pqr[2];
	/****************角度获取******************/
	// 重力实测的重力向量
	float ax = ele->DatAcc[0], ay = ele->DatAcc[1], az = ele->DatAcc[2];
	float recipNorm_a = invSqrt(ax*ax + ay*ay + az*az);
	ax *= recipNorm_a;
	ay *= recipNorm_a;
	az *= recipNorm_a;
	float a_norm[3];
	a_norm[0] = ax;
	a_norm[1] = ay;
	a_norm[2] = az;
	// 规范化测量的磁场强度
	float mx = ele->DatMag[0], my = ele->DatMag[1], mz = ele->DatMag[2];
	float recipNorm_h = invSqrt(mx*mx + my*my + mz*mz);
	mx *= recipNorm_h;
	my *= recipNorm_h;
	mz *= recipNorm_h;
	float m_norm[3];
	m_norm[0] = mx;
	m_norm[1] = my;
	m_norm[2] = mz;
	
	ele->acc_sum = ele->DatAcc[0]*ele->DatAcc[0] + ele->DatAcc[1]*ele->DatAcc[1] + ele->DatAcc[2]*ele->DatAcc[2];
	ele->acc_R_x = Abs(ele->acc_sum - 9.788f*9.788f);
	ele->acc_R_a = 4.0f;
	ele->acc_R_b = 0.1f;
	ele->R = ele->acc_R_b * exp(ele->acc_R_a*ele->acc_R_x);
	
	//-----------------------------------------------------------------------------------------
//	for (int i=0;i<3;i++)
//	{
//		ele->Q_element[i][i]=ele->Q1;//
//  }
//	for (int i=3;i<6;i++)													
//	{
//		ele->Q_element[i][i]=ele->Q2;//
//	}
	for (int i=0;i<3;i++)
	{
		ele->R_element[i][i]=ele->R;//
	}
	  ele->R_element[3][3]=ele->R;//
	
//------------------------------------------------------------------------------------------
	
//------------------------------------------------------------------------------------------
	ESKF_Attitude_MCH(ele->X_p_q, ele->X_p_w, a_norm, m_norm, w_rad, ele->P_p_element, ele->halfDt, ele->X_k_q, ele->X_k_w, ele->X_n_q, ele->X_n_w, ele->P_n_element);
//	arm_mat_mult_f32(&ele->I, &ele->X_n,&ele->X_p);
	for(int j=0;j<4;j++)
	{
			ele->X_p_q[j] = ele->X_n_q[j];
	}
	for(int i=0;i<3;i++)
	{
			ele->X_p_w[i] = ele->X_n_w[i];
	}
	for(int i=0;i<6;i++)
	{
		for(int j=0;j<6;j++)
		{	
			ele->P_p_element[i][j] = ele->P_n_element[i][j] ;
		}
	}
	//EKF得到的四元数信息
	ele->w =  ele->X_k_q[0];
	ele->x =  ele->X_k_q[1];
	ele->y =  ele->X_k_q[2];
	ele->z =  ele->X_k_q[3];
	//坐标系转换
	ele->Cb2n[0][0] = 1.0f-2.0f*(ele->y*ele->y + ele->z*ele->z);
	ele->Cb2n[0][1] =      2.0f*(ele->x*ele->y - ele->w*ele->z);
	ele->Cb2n[0][2] =      2.0f*(ele->x*ele->z + ele->w*ele->y);
	ele->Cb2n[1][0] =      2.0f*(ele->x*ele->y + ele->w*ele->z);
	ele->Cb2n[1][1] = 1.0f-2.0f*(ele->x*ele->x + ele->z*ele->z);
	ele->Cb2n[1][2] =      2.0f*(ele->y*ele->z - ele->w*ele->x);
	ele->Cb2n[2][0] =      2.0f*(ele->x*ele->z - ele->w*ele->y);
	ele->Cb2n[2][1] =      2.0f*(ele->y*ele->z + ele->w*ele->x);
	ele->Cb2n[2][2] = 1.0f-2.0f*(ele->x*ele->x + ele->y*ele->y);
	//四元数转欧拉角
	ele->Ang[0] =  atan2f(2.0f * ele->y * ele->z + 2.0f * ele->w * ele->x, -2.0f * ele->x * ele->x - 2.0f * ele->y * ele->y + 1.0f) - ele->AngOff[0];
	ele->Ang[1] =  asinf(fConstrain(-2.0f * ele->x * ele->z + 2.0f * ele->w * ele->y,-1.0f,1.0f)) - ele->AngOff[1];
	ele->Ang[2] =  atan2f(2.0f * ele->x * ele->y + 2.0f * ele->w * ele->z, -2.0f * ele->y * ele->y - 2.0f * ele->z * ele->z + 1.0f) - ele->AngOff[2];
	
	//机体坐标系下加速度，比力，adis测量的是力在机体系下的坐标
	ele->AccB[0]= -ele->DatAcc[0];
	ele->AccB[1]= -ele->DatAcc[1];
	ele->AccB[2]= -ele->DatAcc[2];
//	ele->Accm[0]= -ICM20602->AccFil_3nd[0];ele->Accm[1]=-ICM20602->AccFil_3nd[1];ele->Accm[2]=-ICM20602->AccFil_3nd[2];
	//?????????
//	ele->AccB[0]= -ele->DatAcc[0];ele->AccB[1]=-ele->DatAcc[1];ele->AccB[2]=-ele->DatAcc[2];
//	ele->Acc[0] = ele->AccB[0]*ele->Cb2n[0][0] + ele->AccB[1]*ele->Cb2n[0][1] + ele->AccB[2]*ele->Cb2n[0][2];
//	ele->Acc[1] = ele->AccB[0]*ele->Cb2n[1][0] + ele->AccB[1]*ele->Cb2n[1][1] + ele->AccB[2]*ele->Cb2n[1][2];
//	ele->Acc[2] = ele->AccB[0]*ele->Cb2n[2][0] + ele->AccB[1]*ele->Cb2n[2][1] + ele->AccB[2]*ele->Cb2n[2][2]+adis->OneG;
	/****************计算结束****************/
	
	ele->circle_loop ++;
	if(ele->circle_loop>1000)
		ele->circle_loop = 1000;	
	
	ele->Update = true;
	Tim_Calc(&ele->Tim);   //计时
	ele->Time = ele->Tim.OUT;	
	return ele->Update;
}




