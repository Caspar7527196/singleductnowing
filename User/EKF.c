#include "EKF.h"
sEKF EKF;
STORE_IMU_BUFFER storeIMU;
STORE_MAG_BUFFER storeMAG;
STORE_BARO_BUFFER storeBARO;
STORE_LASER_BUFFER  storeLASER;
STORE_GPS_BUFFER storeGPS;
STORE_OUTPUT_BUFFER storeOUTPUT;

float nextP[22][22];
float P[22][22];
float KH[22][22];
float KHP[22][22];
float euler_EKF[3];
float euler_EKF1[3];
float vel_EKF[3];
float vel_EKF1[3];
float pos_EKF[3];
float pos_EKF1[3];
float deltaAngErr[3];
float innovVelPos[6];
float varInnovVelPos[6];
float innov_Mag[3];
float varInnovMag[3];
float posOffsetBody[3] = {0.0f, 0.0f, -0.15f};//GPS相对IMU位置偏移量，视大小飞机而定

float GYRO_BIAS_PROCESS_NOISE = 0.001f;
float ACC_BIAS_PROCESS_NOISE = 0.5f;
float GYRO_NOISE = 0.001f;				
float ACC_NOISE = 1.0f;
float MAG_EARTH_PROCESS_NOISE = 0.001f;
float MAG_BODY_PROCESS_NOISE = 0.0001f;

u32 time_max = 0;
void 	EKF_Init_Para(sEKF *ele){
	storeIMU.oldest = storeIMU.youngest = 0;
	storeIMU.is_filled = false;
	
	storeMAG.oldest = storeMAG.youngest = 0;
	storeMAG.new_data_flag = false;
	
	storeBARO.oldest = storeBARO.youngest = 0;
	storeBARO.new_data_flag = false;
	
	storeGPS.oldest = storeGPS.youngest = 0;
	storeGPS.new_data_flag = false;
	
	ele->imuQuatDownSampleNew[0] = 1.0f;
	ele->imuQuatDownSampleNew[1] = 0.0f;
	ele->imuQuatDownSampleNew[2] = 0.0f;
	ele->imuQuatDownSampleNew[3] = 0.0f;
	
	for(u8 i = 0;i<3;i++){
		ele->imuDataNew.delAng[i] = 0.0f;
		ele->imuDataNew.delVel[i] = 0.0f;
		ele->imuDataDownSampledNew.delAng[i] = 0.0f;
		ele->imuDataDownSampledNew.delVel[i] = 0.0f;
		ele->imuDataDelayed.delAng[i] = 0.0f;
		ele->imuDataDelayed.delVel[i] = 0.0f;
	}
	ele->imuDataNew.delAngDT = 0.0f;
	ele->imuDataNew.delVelDT = 0.0f;
	ele->imuDataNew.time_ms = 0;
	ele->imuDataDownSampledNew.delAngDT = 0.0f;
	ele->imuDataDownSampledNew.delVelDT = 0.0f;
	ele->imuDataDownSampledNew.time_ms = 0;
	ele->imuDataDelayed.delAngDT = 0.0f;
	ele->imuDataDelayed.delVelDT = 0.0f;
	ele->imuDataDelayed.time_ms = 0;
	ele->imuSampleTime_ms = 0;
	ele->imuPreSampleTime_ms = 0;
	
	ele->lastMagUpdate_ms = 0;
	
	ele->state[0] = 1.0f;
	ele->state[1] = 0.0f;
	ele->state[2] = 0.0f;
	ele->state[3] = 0.0f;
	for(u8 i = 4;i < 22;i++){
		ele->state[i] = 0.0f;
	}
	
	for(u8 i = 0;i < 22;i++){
		for(u8 j = 0;j < 22;j++){
			P[i][j] = 0.0f;
			nextP[i][j] = 0.0f;
		}
	}
	
	for(u8 i = 0;i < 3;i++){
		for(u8 j = 0;j < 3;j++){
			ele->prevTnb[i][j] = 0.0f;
		}
	}
	
	for(u8 i = 0;i < 3;i++){
		for(u8 j = 0;j < 3;j++){
			ele->prevTbn[i][j] = 0.0f;
		}
	}
	
	ele->ekf_init_flag = false;
	ele->ekf_start_flag = true;
	ele->run_update_filter = false;
	ele->magFusePerformed = false;
	ele->fuseVelData = false;
	ele->fusePosData = false;
	ele->fuseHgtData = false;
	ele->on_Ground = true;
	ele->in_flight = false;
	ele->finalInflightYawInit = false;
	ele->finalInflightMagInit = false;
	ele->magFieldLearned = false;
	ele->inhibitMagStates = true;
	
	for(int i = 0 ; i <3 ;i++)
	{
		ele->magTestRatio[i] = 0;
	}
	ele->HgtTestRatio = 0;
	ele->posTestRatio = 0;
	ele->velTestRatio = 0;
	
	
	ele->magFieldGate = 3.0f;
	ele->gpsposGate = 5.0f;
	ele->gpsvelGate = 5.0f;
	ele->baroHgtGate = 5.0f;
	ele->yawInnovGate = 5.0f;
	ele->PV_AidingMode = ABSOLUTE;
	ele->PV_AidingModePrev = NONE;
	ele->posTimeout = true;
	ele->velTimeout = true;
	
	ele->tiltDriftTimeMax_ms = 15000;  //
	
}

void Quaternion_Init(sEKF *ele)
{
	float acc_magnitude = sqrtf(ICM20602[0].AccFil_3nd[0]*ICM20602[0].AccFil_3nd[0] + ICM20602[0].AccFil_3nd[1]*ICM20602[0].AccFil_3nd[1] + ICM20602[0].AccFil_3nd[2]*ICM20602[0].AccFil_3nd[2]);
	float mag_magnitude = sqrtf(gps->MagFil[0] * gps->MagFil[0] + gps->MagFil[1] * gps->MagFil[1] + gps->MagFil[2] * gps->MagFil[2]);
	float acc_norm[3];
	float mag_norm[3];
	for(u8 i = 0;i<3;i++){
		acc_norm[i] = ICM20602[0].AccFil_3nd[i]/acc_magnitude;
		mag_norm[i] = gps->MagFil[i]/mag_magnitude;
	}
	float roll,pitch,yaw;
  roll = atan2f(acc_norm[1],acc_norm[2]);
	pitch = asinf(-acc_norm[0]);
	float mag_calib[2];
	mag_calib[0] =  mag_norm[0] * cos(pitch) + mag_norm[1] * sin(pitch) * sin(roll) + mag_norm[2] * sin(pitch) * cos(roll);
	mag_calib[1] = -mag_norm[1] * cos(roll) + mag_norm[2] * sin(roll);
	if (mag_calib[0] != 0)
	{
		yaw= atan2f(mag_calib[1], mag_calib[0]);
	}
	else
	{
		if (mag_calib[1] < 0)
			yaw = -PI/2;
		else
			yaw = PI/2;
	}
	yaw += (gps->yaw_declination) * D2R;
	float euler[3] = {roll,pitch,yaw};
	float q_init[4];
	euler2quaternion(euler,q_init);
	rotation_matrix(q_init,ele->prevTbn);
	for(u8 i = 0;i < 4;i++){
		ele->state[i] = q_init[i];
	}
	QuaternionReverse(q_init);
	rotation_matrix(q_init,ele->prevTnb);
}

void CovarianceInit(sEKF *ele)
{
	float rot_vec_var[3];
	rot_vec_var[0] = rot_vec_var[1] = rot_vec_var[2] = 0.1f *0.1f;
	initialiseQuatCovariances(ele,rot_vec_var);
	P[4][4] = ele->gpsDataNew.vel_accurancy * ele->gpsDataNew.vel_accurancy;
	P[5][5] = P[4][4];
	P[6][6] = P[4][4];
	
	P[7][7] = ele->gpsDataNew.pos_accurancy * ele->gpsDataNew.pos_accurancy;
	P[8][8] = P[7][7];
	P[9][9] = ele->gpsDataNew.hei_accuracy * ele->gpsDataNew.hei_accuracy;
	//ysj 0.05？
	P[10][10] = (GYROBIASUNCERTAINTY * D2R * EKF_TARGET_DT) * (GYROBIASUNCERTAINTY * D2R * EKF_TARGET_DT);
	P[11][11] = P[10][10];
	P[12][12] = P[10][10];
	
	P[13][13] = (ACCBIAS_VAR * EKF_TARGET_DT) * (ACCBIAS_VAR * EKF_TARGET_DT);
	P[14][14] = P[13][13];
	P[15][15] = P[13][13];
	
	P[16][16] = MAG_NED_R*MAG_NED_R;
	P[17][17] = P[16][16];
	P[18][18] = P[16][16];
	
	P[19][19] = MAG_NED_R*MAG_NED_R;
	P[20][20] = P[19][19];
	P[21][21] = P[19][19];
}

void EKF_Run(sEKF *ele)
{  
//	if(ms5611->AltOff!=0)
//	gps->ECEF_Init_Flag = true;
	if(gps->ECEF_Init_Flag)
	{
		if(!EKF.ekf_init_flag){
			EKF_Initialise(ele);
		}
		else{
			EKF_Update(ele);
		}
	}
}

void calcEarthRateNED(sEKF *ele)//..计算地球角速度
{
	#define EARTHRATE 0.000072921f
	float lat_rad = gps->lat * D2R;
	ele->earthRateNED[0] = EARTHRATE * cosf(lat_rad);
	ele->earthRateNED[1] = 0;
	ele->earthRateNED[2] = -EARTHRATE * sinf(lat_rad);
}

void EKF_Initialise(sEKF *ele)
{
	readIMUData(ele);
	readMAGData(ele);
	readGPSData(ele);
	readBAROData(ele);
	readLASERData(ele);
	if(!storeIMU.is_filled) return;
	
	//四元数初始化
	Quaternion_Init(ele);
	
	//NED速度初始化
	for(u8 i = 0;i < 3;i++){
		ele->state[i+4] = gps->NED_spd[i];
	}
	//NED水平位置初始化
//	for(u8 i = 0;i < 2;i++){
//		ele->state[i+7] = gps->NED[i];
//	}
	float pos_correct[3];
	MatrixMultiVector(ele->prevTbn,posOffsetBody,pos_correct);//得到NED坐标系下的位置补偿
	for(u8 i = 0;i < 2;i++){
		ele->state[i+7] = gps->NED[i] - pos_correct[i];
	}
	//NED高度位置初始化
	ele->state[9] = 0.0f;
	
//	//NED磁力计初始化
//	float mag_NED_init[3];
//	float q_init[4];
//	for(u8 i = 0;i < 4;i++){
//		q_init[i] = ele->state[i];
//	}
//	float rot_matrix[3][3];
//	rotation_matrix(q_init,rot_matrix);//四元数转旋转矩阵
//	MatrixMultiVector(rot_matrix,gps->MagFil,mag_NED_init);//计算旋转过后的速度增量
//	float magLengthNE = sqrtf(mag_NED_init[0] * mag_NED_init[0] + mag_NED_init[1] * mag_NED_init[1]);
//	mag_NED_init[0] = magLengthNE * cosf(gps->yaw_declination * D2R);
//	mag_NED_init[1] = magLengthNE * sinf(gps->yaw_declination * D2R);
//	for(u8 i = 0;i < 3;i++){
//		ele->state[i+16] = 0.0001f * mag_NED_init[i];
//	}
	
	//计算地球角速度
	calcEarthRateNED(ele);
	
	//误差矩阵初始化
	CovarianceInit(ele);
	
	//输出数组初始化
	StoreOutputReset(ele);
	
	ele->ekf_init_flag = true;
}

void updateStateIndexLim(sEKF *ele)
{
	if(ele->inhibitMagStates)
		ele->stateIndexLim = 15;
	else
		ele->stateIndexLim = 21;
}

void detectFlight(sEKF *ele){
	if(((rc->PPM[2]>=1200)&&(rc->PPM[2]<=1400))||ele->state[9] < -0.5f){
	//if(ele->state[9]<-0.2f){
		ele->on_Ground = false;
	}else{
		ele->in_flight = false;
		ele->on_Ground = true;
	}
	if(!ele->on_Ground){
		if(ele->state[9] - ele->posDownAtTakeoff < -0.5f){
			ele->in_flight = true;
		}
	}
	if(ele->on_Ground){
		ele->posDownAtTakeoff = ele->state[9];
	}
}

void setMagStateLearningMode(sEKF *ele)
{
	bool magCalRequested = ele->finalInflightYawInit && ele->finalInflightMagInit;//航向角初始化、磁力计初始化完毕才进行22阶
	
	bool setMagInhibit = !magCalRequested || ele->on_Ground;//在地面则不进行校准
	
	if(!ele->inhibitMagStates && setMagInhibit) {
    ele->inhibitMagStates = true;
    updateStateIndexLim(ele);
  }else if (ele->inhibitMagStates && !setMagInhibit) {
    ele->inhibitMagStates = false;
    updateStateIndexLim(ele);
    if(ele->magFieldLearned) {
      P[16][16] = ele->earthMagFieldVar[0];
			P[17][17] = ele->earthMagFieldVar[1];
			P[18][18] = ele->earthMagFieldVar[2];
			P[19][19] = ele->bodyMagFieldVar[0];
			P[20][20] = ele->bodyMagFieldVar[1];
			P[21][21] = ele->bodyMagFieldVar[2];
    }else{
      for(u8 index=18; index<=21; index++) {
        P[index][index] = sq(MAG_NED_R);
      }
      alignMagStateDeclination(ele);
    }
  }
	
	if(ele->on_Ground){
     ele->finalInflightYawInit = false;
     ele->finalInflightMagInit = false;
  }
	updateStateIndexLim(ele);
}

void controlFilterModes(sEKF *ele)
{
	detectFlight(ele);
	
	setMagStateLearningMode(ele);
	
	setAidingMode(ele);
}

void EKF_Update(sEKF *ele)
{
	Tim_Calc(&ele->Tim);
	controlFilterModes(ele);
	readIMUData(ele);//读取IMU数据
	if(ele->run_update_filter){
		UpdateStrapdownEquationNED(ele);//捷联式惯性导航方程更新	
		CovariancePrediction(ele);//误差矩阵预测更新
		SelectMagFusion(ele);//融合磁力计信息
		SelectVelPosFusion(ele);//融合位置速度信息
	}
	calcOutputStates(ele);
	float quat_temp1[4];
	for(u8 i = 0;i<=3;i++){
		quat_temp1[i] = ele->state[i];
		//quat_temp[i] = ele->state[i];
	}
	Quaternion2Euler(quat_temp1,euler_EKF1);
	for(u8 i = 0;i<3;i++){
		vel_EKF1[i] = ele->state[i+4];
		pos_EKF1[i] = ele->state[i+7];
	}
	float quat_temp[4];
	for(u8 i = 0;i<=3;i++){
		quat_temp[i] = ele->outputDataNew.quat[i];
		//quat_temp[i] = ele->state[i];
	}
	Quaternion2Euler(quat_temp,euler_EKF);
	for(u8 i = 0;i<=2;i++){
		vel_EKF[i] = ele->outputDataNew.vel[i];
		pos_EKF[i] = ele->outputDataNew.pos[i];		
	}	
	Tim_Calc(&ele->Tim);
}
void readIMUData(sEKF *ele)
{
	//从IMU获得数据
	ele->imuDataNew.time_ms = ICM20602[0].imuSampleTime_ms;//记录当前IMU采集时间
	for(u8 i=0;i<3;i++){
		ele->imuDataNew.delAng[i] = ICM20602[0].delAng_acc[i];
		ele->imuDataNew.delVel[i] = ICM20602[0].delVel_acc[i];
		ICM20602[0].delAng_acc[i] = 0.0f;
		ICM20602[0].delVel_acc[i] = 0.0f;
	}
	ele->imuDataNew.delAngDT = ICM20602[0].delAng_acc_dt;
	ele->imuDataNew.delVelDT = ICM20602[0].delVel_acc_dt;
	ICM20602[0].delAng_acc_dt = 0.0f;//imu产生数据的时间大于read的时间，实际上需要用求和近似积分
	ICM20602[0].delVel_acc_dt = 0.0f;
	
	
	
	/****速度、时间  下采样叠加****///下采样就是对 由连续信号进行高采样率得到的离散信号 采样（采样频率更低）
	ele->imuDataDownSampledNew.delAngDT += ele->imuDataNew.delAngDT;//角速度增量时间累积
	ele->imuDataDownSampledNew.delVelDT += ele->imuDataNew.delVelDT;//速度增量时间累积
	float quat_temp[4];
	from_axis_angle(ele->imuDataNew.delAng,quat_temp);//将角增量转换到四元数
	QuaternionMulti(ele->imuQuatDownSampleNew,quat_temp);//将角增量得到的四元数乘以原有累积的四元数
	QuaternionNormalise(ele->imuQuatDownSampleNew);//归一化四元数
	float rot_matrix[3][3];
	rotation_matrix(ele->imuQuatDownSampleNew,rot_matrix);//四元数转旋转矩阵
	float delVel_temp[3];
	MatrixMultiVector(rot_matrix,ele->imuDataNew.delVel,delVel_temp);//计算旋转过后的速度增量
	for(u8 i = 0;i < 3;i++){
		ele->imuDataDownSampledNew.delVel[i]+=delVel_temp[i];//叠加速度增量
	}
	/****角速度  下采样叠加****/
	//delAngDT>15ms才run_update_filter = true
	if((ele->imuDataDownSampledNew.delAngDT >= EKF_TARGET_DT - 0.5f*IMU_AVR_DT)&&ele->ekf_start_flag){
		float del_Ang_temp[3];
		to_axis_angle(ele->imuQuatDownSampleNew,del_Ang_temp);
		for(u8 i = 0;i < 3;i++){
			ele->imuDataDownSampledNew.delAng[i] = del_Ang_temp[i];
		}
		ele->imuDataDownSampledNew.time_ms = ele->imuDataNew.time_ms;
		
		
		
		//将收集到的IMU数据PUSH到FIFO里
		Imu_Data_Push(ele->imuDataDownSampledNew);
		//清零
		for(u8 i = 0;i < 3;i++){
			ele->imuDataDownSampledNew.delAng[i] = 0.0f;
			ele->imuDataDownSampledNew.delVel[i] = 0.0f;
		}
		ele->imuDataDownSampledNew.delAngDT = 0.0f;
		ele->imuDataDownSampledNew.delVelDT = 0.0f;
		ele->imuQuatDownSampleNew[0] = 1.0f;
		ele->imuQuatDownSampleNew[1] = 0.0f;
		ele->imuQuatDownSampleNew[2] = 0.0f;
		ele->imuQuatDownSampleNew[3] = 0.0f;
		//弹出延时的IMU数据
		Imu_Data_Pop(&ele->imuDataDelayed);
		for(u8 i = 0;i < 3; i++){
			ele->imuDataDelayed.delAng[i] -= ele->state[i+10]*ele->imuDataDelayed.delAngDT/EKF_TARGET_DT;
			ele->imuDataDelayed.delVel[i] -= ele->state[i+13]*ele->imuDataDelayed.delVelDT/EKF_TARGET_DT;
		}
		ele->run_update_filter = true;
	}
	else{
		ele->run_update_filter = false;
	}
}


void Imu_Data_Push(imu_elements data)
{
	storeIMU.youngest = (storeIMU.youngest + 1)%IMU_BUFFER_SIZE;
	storeIMU.imu_data[storeIMU.youngest] = data;
	storeIMU.oldest = (storeIMU.youngest + 1)%IMU_BUFFER_SIZE;
	if(storeIMU.oldest ==0) 
	storeIMU.is_filled = true;
}

void Imu_Data_Pop(imu_elements* data)
{
	*data = storeIMU.imu_data[storeIMU.oldest];//数组imu_data的标号为 oldest 的数据赋给data指向的内容
}

bool Recall_Mag_data(STORE_MAG_BUFFER* data,u32 sampleing_time,sEKF *ele)
{
	u8 oldest = data->oldest;
	u8 bestindex;
	bool success = false;
	if(!storeMAG.new_data_flag)
		return false;
	if(data->youngest == oldest)
	{
		if((data->mag_data[oldest].time_ms!=0) && (data->mag_data[oldest].time_ms <= sampleing_time)){
			if(sampleing_time - data->mag_data[oldest].time_ms < 100)
			{
				bestindex = oldest;
				success = true;
				storeMAG.new_data_flag = false;
			}
		}
	}
	else{
		while(data->youngest != oldest)
		{
			if((data->mag_data[oldest].time_ms!=0) && (data->mag_data[oldest].time_ms <= sampleing_time)){
				if(sampleing_time - data->mag_data[oldest].time_ms < 100)
				{
					bestindex = oldest;
					success = true;
					//storeMAG.new_data_flag = false;
				}
			}
			else if (data->mag_data[oldest].time_ms > sampleing_time)
			{
				break;
			}
			oldest = (oldest+1)%OBS_BUFFER_SIZE;
		}
	}
	
	if(success)
	{
		ele->magDataDelayed = data->mag_data[bestindex];
		data->oldest = (bestindex+1)%OBS_BUFFER_SIZE;
		data->mag_data[bestindex].time_ms = 0;
		return true;
	}
	else
	{
		return false;
	}
}

void Mag_Data_Push(mag_elements data)
{
	storeMAG.youngest = (storeMAG.youngest + 1)%OBS_BUFFER_SIZE;
	storeMAG.mag_data[storeMAG.youngest] = data;
	storeMAG.new_data_flag = true;
}

void readMAGData(sEKF *ele)
{
	u32 curMagUpdate_ms = 0.001f * gps->mag_update_time;//当前磁力计更新时间
	if(curMagUpdate_ms - ele->lastMagUpdate_ms > MIN_INTERVAL_SENSOR_MS){//判断间隔是否满足要求
		ele->magDataNew.time_ms = curMagUpdate_ms - MAG_DELAY_MS;//磁力计真正更新时间
		ele->lastMagUpdate_ms = curMagUpdate_ms;
		for(u8 i = 0;i < 3;i++){
			ele->magDataNew.mag[i] = gps->MagFil[i]*0.0001f;//缩小10000倍 防止矩阵奇异
		}
		Mag_Data_Push(ele->magDataNew);
	}
}

bool Recall_Baro_data(STORE_BARO_BUFFER* data,u32 sampleing_time,sEKF *ele)
{
	u8 oldest = data->oldest;
	u8 bestindex;
	bool success = false;
	if(!storeBARO.new_data_flag)
		return false;
	if(data->youngest == oldest)
	{
		if((data->baro_data[oldest].time_ms!=0) && (data->baro_data[oldest].time_ms <= sampleing_time)){
			if(sampleing_time - data->baro_data[oldest].time_ms < 100)
			{
				bestindex = oldest;
				success = true;
				storeBARO.new_data_flag = false;
			}
		}
	}
	else{
		while(data->youngest != oldest)
		{
			if((data->baro_data[oldest].time_ms!=0) && (data->baro_data[oldest].time_ms <= sampleing_time)){
				if(sampleing_time - data->baro_data[oldest].time_ms < 100)
				{
					bestindex = oldest;
					success = true;
					//storeBARO.new_data_flag = false;
				}
			}
			else if (data->baro_data[oldest].time_ms > sampleing_time)
			{
				break;
			}
			oldest = (oldest+1)%OBS_BUFFER_SIZE;
		}
	}
	
	if(success)
	{
		ele->baroDataDelayed = data->baro_data[bestindex];
		data->oldest = (bestindex+1)%OBS_BUFFER_SIZE;
		data->baro_data[bestindex].time_ms = 0;
		return true;
	}
	else
	{
		return false;
	}
}

bool Recall_Laser_data(STORE_LASER_BUFFER* data,u32 sampleing_time,sEKF *ele)
{
	
	u8 oldest = data->oldest;
	u8 bestindex;
	bool success = false;
	if(!storeBARO.new_data_flag)
		return false;
	if(data->youngest == oldest)
	{
		if((data->laser_data[oldest].time_ms!=0) && (data->laser_data[oldest].time_ms <= sampleing_time)){
			if(sampleing_time - data->laser_data[oldest].time_ms < 100)
			{
				bestindex = oldest;
				success = true;
				storeLASER.new_data_flag = false;
			}
		}
	}
	else{
		while(data->youngest != oldest)
		{
			if((data->laser_data[oldest].time_ms!=0) && (data->laser_data[oldest].time_ms <= sampleing_time)){
				if(sampleing_time - data->laser_data[oldest].time_ms < 100)
				{
					bestindex = oldest;
					success = true;
					//storeBARO.new_data_flag = false;
				}
			}
			else if (data->laser_data[oldest].time_ms > sampleing_time)
			{
				break;
			}
			oldest = (oldest+1)%OBS_BUFFER_SIZE;
		}
	}
	
	if(success)
	{
		ele->laserDataDelayed = data->laser_data[bestindex];
		data->oldest = (bestindex+1)%LASER_BUFFER_SIZE;
		data->laser_data[bestindex].time_ms = 0;
		return true;
	}
	else
	{
		return false;
	}

}
void Laser_Data_Push(baro_elements data)
{
	storeLASER.youngest = (storeLASER.youngest + 1)%LASER_BUFFER_SIZE;
	storeLASER.laser_data[storeLASER.youngest] = data;
	storeLASER.new_data_flag = true;
}

void  Baro_Data_Push(laser_elements data)
{
	storeBARO.youngest = (storeBARO.youngest + 1)%OBS_BUFFER_SIZE;
	storeBARO.baro_data[storeBARO.youngest] = data;
	storeBARO.new_data_flag = true;
}
void readBAROData(sEKF *ele)
{
	u32 curBaroUpdate_ms = 0.001f * ms5611->height_update_time;//当前气压计采集的时刻
	if(curBaroUpdate_ms - ele->lastBaroUpdate_ms > MIN_INTERVAL_SENSOR_MS){//判断是否间隔时间满足要求
		ele->baroDataNew.time_ms = curBaroUpdate_ms - BARO_DELAY_MS;//去除延迟时间
		ele->lastBaroUpdate_ms = curBaroUpdate_ms;//更新上一时刻采集时刻
		ele->baroDataNew.hgt = -ms5611->AltFil;//高度赋值
		Baro_Data_Push(ele->baroDataNew);
//				 static int  tests[100];
//		static  int  test_count;
//			sTIM  test;
//		Tim_Calc(&test);
//		tests[test_count] = test.CNT;
//		test_count++;
	}
}

void readLASERData(sEKF *ele)
{
		u32 curLaserUpdate_ms = 0.001f *exitx->height_update_time;//当前激光采集时刻
	if(curLaserUpdate_ms - ele->lastLaserUpdate_ms > MIN_INTERVAL_SENSOR_MS){//判断是否间隔时间满足要求
		ele->baroDataNew.time_ms = curLaserUpdate_ms - BARO_DELAY_MS;//去除延迟时间
		ele->lastBaroUpdate_ms = curLaserUpdate_ms;//更新上一时刻采集时刻
		ele->baroDataNew.hgt =  -exitx->DatRel[2];//高度赋值
		Laser_Data_Push(ele->laserDataNew);
	  ele->rngValidMeaTime_ms = ele->imuSampleTime_ms;
	   }
}
	
bool Recall_Gps_data(STORE_GPS_BUFFER* data,u32 sampleing_time,sEKF *ele)
{
	u8 oldest = data->oldest;
	u8 bestindex;
	bool success = false;
	if(!storeGPS.new_data_flag)
		return false;
	if(data->youngest == oldest)
	{
		if((data->gps_data[oldest].time_ms!=0) && (data->gps_data[oldest].time_ms <= sampleing_time)){
			if(sampleing_time - data->gps_data[oldest].time_ms < 100)
			{
				bestindex = oldest;
				success = true;
				storeGPS.new_data_flag = false;
			}
		}
	}
	else{
		while(data->youngest != oldest)
		{
			if((data->gps_data[oldest].time_ms!=0) && (data->gps_data[oldest].time_ms <= sampleing_time)){
				if(sampleing_time - data->gps_data[oldest].time_ms < 100)
				{
					bestindex = oldest;
					success = true;
					storeGPS.new_data_flag = false;
				}
			}
			else if (data->gps_data[oldest].time_ms > sampleing_time)
			{
				break;
			}
			oldest = (oldest+1)%OBS_BUFFER_SIZE;
		}
	}
	
	if(success)
	{
		ele->gpsDataDelayed = data->gps_data[bestindex];
		data->oldest = (bestindex+1)%OBS_BUFFER_SIZE;
		data->gps_data[bestindex].time_ms = 0;
		return true;
	}
	else
	{
		return false;
	}
}

void Gps_Data_Push(gps_elements data)
{
	storeGPS.youngest = (storeGPS.youngest + 1)%OBS_BUFFER_SIZE;
	storeGPS.gps_data[storeGPS.youngest] = data;
	storeGPS.new_data_flag = true;
}

void readGPSData(sEKF *ele)
{    
	u32 curGpsUpdate_ms = 0.001f * gps->gps_update_time;
	if(curGpsUpdate_ms - ele->lastGpsUpdate_ms >MIN_INTERVAL_SENSOR_MS){
		ele->gpsDataNew.time_ms = curGpsUpdate_ms - GPS_DELAY_MS;
		ele->lastGpsUpdate_ms = curGpsUpdate_ms;
		for(u8 i = 0;i<3;i++){
			ele->gpsDataNew.pos[i] = gps->NED[i];
			ele->gpsDataNew.vel[i] = gps->NED_spd[i];
		}
		ele->gpsDataNew.pos_accurancy = gps->gpsPosAccuracy;
		ele->gpsDataNew.hei_accuracy  = gps->gpsHeiAccuracy;
		ele->gpsDataNew.vel_accurancy = gps->gpsSpdAccuracy;
		Gps_Data_Push(ele->gpsDataNew);

		  
	   //监控GPS是否可用于EKF
			if(ele->PV_AidingMode == NONE)
			{
				ele->gpsGoodToAlign = calcGpsGoodToAlign(ele);
			}
			else ele->gpsGoodToAlign = false;
	}
}

void StoreOutputReset(sEKF *ele)//..输出数组初始化
{
	for(u8 i = 0;i <4;i++){
		ele->outputDataNew.quat[i] = ele->state[i];
	}
	for(u8 i = 0;i < 3;i++){
		ele->outputDataNew.vel[i] = ele->state[i+4];
	}
	for(u8 i = 0;i < 3;i++){
		ele->outputDataNew.pos[i] = ele->state[i+7];
	}
	for(u8 i = 0;i < IMU_BUFFER_SIZE;i++){
		storeOUTPUT.output_data[i] = ele->outputDataNew;
	}
	ele->outputDataDelayed = ele->outputDataNew;
}

void UpdateStrapdownEquationNED(sEKF *ele)
{
	float earthRateBody[3];
	MatrixMultiVector(ele->prevTnb,ele->earthRateNED,earthRateBody);//计算机体坐标系下的地球转速
	float delAng_total[3];
	for(u8 i = 0;i < 3;i++){
		delAng_total[i] = ele->imuDataDelayed.delAng[i] - earthRateBody[i] * ele->imuDataDelayed.delAngDT;//修正后的角增量
	}
	float del_quat_temp[4];
	from_axis_angle(delAng_total,del_quat_temp);//将角增量转换成四元数
	float quat_from_state[4];
	for(u8 i = 0;i < 4;i++){
		quat_from_state[i] = ele->state[i];//之前的四元数
	}
	QuaternionMulti(quat_from_state,del_quat_temp);//四元数一步预测
	QuaternionNormalise(quat_from_state);//四元数归一化
	for(u8 i = 0;i < 4;i++){
		ele->state[i] = quat_from_state[i];
	}
	
	float delVelNav[3];
	MatrixMultiVector(ele->prevTbn,ele->imuDataDelayed.delVel,delVelNav);//将速度增量转换到北东地坐标系
	delVelNav[2]+= GRAVITY_MSS * ele->imuDataDelayed.delVelDT;//速度增量叠加重力分量
	
	rotation_matrix(quat_from_state,ele->prevTbn);//计算上一时刻机体到北东地
	QuaternionReverse(quat_from_state);
	rotation_matrix(quat_from_state,ele->prevTnb);//计算上一时刻北东地到机体
	
	static float lastVel[3] = {0.0f,0.0f,0.0f};
	for(u8 i = 0;i < 3;i++){
		lastVel[i] = ele->state[i+4];//上一时刻的速度
	}
	
	for(u8 i = 0;i < 3;i++){
		ele->state[i+4] += delVelNav[i];//速度预测
	}
	
	for(u8 i = 0;i < 3;i++){
		ele->state[i+7] += (lastVel[i] + ele->state[i+4]) * 0.5f * ele->imuDataDelayed.delVelDT;
	}
}

void CovariancePrediction(sEKF *ele)
{
	float daxVar;       // X axis delta angle noise variance rad^2
	float dayVar;       // Y axis delta angle noise variance rad^2
	float dazVar;       // Z axis delta angle noise variance rad^2
	float dvxVar;       // X axis delta velocity variance noise (m/s)^2
	float dvyVar;       // Y axis delta velocity variance noise (m/s)^2
	float dvzVar;       // Z axis delta velocity variance noise (m/s)^2
	float dvx;          // X axis delta velocity (m/s)
	float dvy;          // Y axis delta velocity (m/s)
	float dvz;          // Z axis delta velocity (m/s)
	float dax;          // X axis delta angle (rad)
	float day;          // Y axis delta angle (rad)
	float daz;          // Z axis delta angle (rad)
	float q0;           // attitude quaternion
	float q1;           // attitude quaternion
	float q2;           // attitude quaternion
	float q3;           // attitude quaternion
	float dax_b;        // X axis delta angle measurement bias (rad)
	float day_b;        // Y axis delta angle measurement bias (rad)
	float daz_b;        // Z axis delta angle measurement bias (rad)
	float dvx_b;        // X axis delta velocity measurement bias (rad)
	float dvy_b;        // Y axis delta velocity measurement bias (rad)
	float dvz_b;        // Z axis delta velocity measurement bias (rad)
	
	float dt = 0.5f * (ele->imuDataDelayed.delAngDT + ele->imuDataDelayed.delVelDT);
	float processNoiseVariance[12];
		
	float dAngBiasVar = (dt * dt * GYRO_BIAS_PROCESS_NOISE) * (dt * dt * GYRO_BIAS_PROCESS_NOISE);
	for (u8 i=0; i<=2; i++) processNoiseVariance[i] = dAngBiasVar;

	float dVelBiasVar = (dt * dt * ACC_BIAS_PROCESS_NOISE) * (dt * dt * ACC_BIAS_PROCESS_NOISE);
	for (u8 i=3; i<=5; i++) {
		processNoiseVariance[i] = dVelBiasVar;
	}
	
	float magEarthVar = (dt * MAG_EARTH_PROCESS_NOISE) * (dt * MAG_EARTH_PROCESS_NOISE);
	float magBodyVar  = (dt * MAG_BODY_PROCESS_NOISE) * (dt * MAG_BODY_PROCESS_NOISE);
	for (u8 i=6; i<=8; i++) processNoiseVariance[i] = magEarthVar;
	for (u8 i=9; i<=11; i++) processNoiseVariance[i] = magBodyVar;
	
	// set variables used to calculate covariance growth
	dvx = ele->imuDataDelayed.delVel[0];
	dvy = ele->imuDataDelayed.delVel[1];
	dvz = ele->imuDataDelayed.delVel[2];
	dax = ele->imuDataDelayed.delAng[0];
	day = ele->imuDataDelayed.delAng[1];
	daz = ele->imuDataDelayed.delAng[2];
	q0 = ele->state[0];
	q1 = ele->state[1];
	q2 = ele->state[2];
	q3 = ele->state[3];
	dax_b = ele->state[10];
	day_b = ele->state[11];
	daz_b = ele->state[12];
	dvx_b = ele->state[13];
	dvy_b = ele->state[14];
	dvz_b = ele->state[15];
	
	float _gyrNoise = GYRO_NOISE;
	daxVar = dayVar = dazVar = (dt*_gyrNoise) * (dt*_gyrNoise);
	float _accNoise = ACC_NOISE;
	dvxVar = dvyVar = dvzVar = (dt*_accNoise) * (dt*_accNoise);
	    
	float SF[21];
  SF[0] = dvz - dvz_b;
	SF[1] = dvy - dvy_b;
	SF[2] = dvx - dvx_b;
	SF[3] = 2*q1*SF[2] + 2*q2*SF[1] + 2*q3*SF[0];
	SF[4] = 2*q0*SF[1] - 2*q1*SF[0] + 2*q3*SF[2];
	SF[5] = 2*q0*SF[2] + 2*q2*SF[0] - 2*q3*SF[1];
	SF[6] = day/2 - day_b/2;
	SF[7] = daz/2 - daz_b/2;
	SF[8] = dax/2 - dax_b/2;
	SF[9] = dax_b/2 - dax/2;
	SF[10] = daz_b/2 - daz/2;
	SF[11] = day_b/2 - day/2;
	SF[12] = 2*q1*SF[1];
	SF[13] = 2*q0*SF[0];
	SF[14] = q1/2;
	SF[15] = q2/2;
	SF[16] = q3/2;
	SF[17] = (q3)*(q3);
	SF[18] = (q2)*(q2);
	SF[19] = (q1)*(q1);
	SF[20] = (q0)*(q0);

  float SG[8];
	SG[0] = q0/2;
	SG[1] = (q3)*(q3);
	SG[2] = (q2)*(q2);
	SG[3] = (q1)*(q1);
	SG[4] = (q0)*(q0);
	SG[5] = 2*q2*q3;
	SG[6] = 2*q1*q3;
	SG[7] = 2*q1*q2;

  float SQ[11];
	SQ[0] = dvzVar*(SG[5] - 2*q0*q1)*(SG[1] - SG[2] - SG[3] + SG[4]) - dvyVar*(SG[5] + 2*q0*q1)*(SG[1] - SG[2] + SG[3] - SG[4]) + dvxVar*(SG[6] - 2*q0*q2)*(SG[7] + 2*q0*q3);
	SQ[1] = dvzVar*(SG[6] + 2*q0*q2)*(SG[1] - SG[2] - SG[3] + SG[4]) - dvxVar*(SG[6] - 2*q0*q2)*(SG[1] + SG[2] - SG[3] - SG[4]) + dvyVar*(SG[5] + 2*q0*q1)*(SG[7] - 2*q0*q3);
	SQ[2] = dvzVar*(SG[5] - 2*q0*q1)*(SG[6] + 2*q0*q2) - dvyVar*(SG[7] - 2*q0*q3)*(SG[1] - SG[2] + SG[3] - SG[4]) - dvxVar*(SG[7] + 2*q0*q3)*(SG[1] + SG[2] - SG[3] - SG[4]);
	SQ[3] = (dayVar*q1*SG[0])/2 - (dazVar*q1*SG[0])/2 - (daxVar*q2*q3)/4;
	SQ[4] = (dazVar*q2*SG[0])/2 - (daxVar*q2*SG[0])/2 - (dayVar*q1*q3)/4;
	SQ[5] = (daxVar*q3*SG[0])/2 - (dayVar*q3*SG[0])/2 - (dazVar*q1*q2)/4;
	SQ[6] = (daxVar*q1*q2)/4 - (dazVar*q3*SG[0])/2 - (dayVar*q1*q2)/4;
	SQ[7] = (dazVar*q1*q3)/4 - (daxVar*q1*q3)/4 - (dayVar*q2*SG[0])/2;
	SQ[8] = (dayVar*q2*q3)/4 - (daxVar*q1*SG[0])/2 - (dazVar*q2*q3)/4;
	SQ[9] = (SG[0]) * (SG[0]);
	SQ[10] = (q1) * (q1);

	float SPP[11];
	SPP[0] = SF[12] + SF[13] - 2*q2*SF[2];
	SPP[1] = SF[17] - SF[18] - SF[19] + SF[20];
	SPP[2] = SF[17] - SF[18] + SF[19] - SF[20];
	SPP[3] = SF[17] + SF[18] - SF[19] - SF[20];
	SPP[4] = 2*q0*q2 - 2*q1*q3;
	SPP[5] = 2*q0*q1 - 2*q2*q3;
	SPP[6] = 2*q0*q3 - 2*q1*q2;
	SPP[7] = 2*q0*q1 + 2*q2*q3;
	SPP[8] = 2*q0*q3 + 2*q1*q2;
	SPP[9] = 2*q0*q2 + 2*q1*q3;
	SPP[10] = SF[16];

	nextP[0][0] = P[0][0] + P[1][0]*SF[9] + P[2][0]*SF[11] + P[3][0]*SF[10] + P[10][0]*SF[14] + P[11][0]*SF[15] + P[12][0]*SPP[10] + (daxVar*SQ[10])/4 + SF[9]*(P[0][1] + P[1][1]*SF[9] + P[2][1]*SF[11] + P[3][1]*SF[10] + P[10][1]*SF[14] + P[11][1]*SF[15] + P[12][1]*SPP[10]) + SF[11]*(P[0][2] + P[1][2]*SF[9] + P[2][2]*SF[11] + P[3][2]*SF[10] + P[10][2]*SF[14] + P[11][2]*SF[15] + P[12][2]*SPP[10]) + SF[10]*(P[0][3] + P[1][3]*SF[9] + P[2][3]*SF[11] + P[3][3]*SF[10] + P[10][3]*SF[14] + P[11][3]*SF[15] + P[12][3]*SPP[10]) + SF[14]*(P[0][10] + P[1][10]*SF[9] + P[2][10]*SF[11] + P[3][10]*SF[10] + P[10][10]*SF[14] + P[11][10]*SF[15] + P[12][10]*SPP[10]) + SF[15]*(P[0][11] + P[1][11]*SF[9] + P[2][11]*SF[11] + P[3][11]*SF[10] + P[10][11]*SF[14] + P[11][11]*SF[15] + P[12][11]*SPP[10]) + SPP[10]*(P[0][12] + P[1][12]*SF[9] + P[2][12]*SF[11] + P[3][12]*SF[10] + P[10][12]*SF[14] + P[11][12]*SF[15] + P[12][12]*SPP[10]) + (dayVar*(q2)*(q2))/4 + (dazVar*(q3)*(q3))/4;
	nextP[0][1] = P[0][1] + SQ[8] + P[1][1]*SF[9] + P[2][1]*SF[11] + P[3][1]*SF[10] + P[10][1]*SF[14] + P[11][1]*SF[15] + P[12][1]*SPP[10] + SF[8]*(P[0][0] + P[1][0]*SF[9] + P[2][0]*SF[11] + P[3][0]*SF[10] + P[10][0]*SF[14] + P[11][0]*SF[15] + P[12][0]*SPP[10]) + SF[7]*(P[0][2] + P[1][2]*SF[9] + P[2][2]*SF[11] + P[3][2]*SF[10] + P[10][2]*SF[14] + P[11][2]*SF[15] + P[12][2]*SPP[10]) + SF[11]*(P[0][3] + P[1][3]*SF[9] + P[2][3]*SF[11] + P[3][3]*SF[10] + P[10][3]*SF[14] + P[11][3]*SF[15] + P[12][3]*SPP[10]) - SF[15]*(P[0][12] + P[1][12]*SF[9] + P[2][12]*SF[11] + P[3][12]*SF[10] + P[10][12]*SF[14] + P[11][12]*SF[15] + P[12][12]*SPP[10]) + SPP[10]*(P[0][11] + P[1][11]*SF[9] + P[2][11]*SF[11] + P[3][11]*SF[10] + P[10][11]*SF[14] + P[11][11]*SF[15] + P[12][11]*SPP[10]) - (q0*(P[0][10] + P[1][10]*SF[9] + P[2][10]*SF[11] + P[3][10]*SF[10] + P[10][10]*SF[14] + P[11][10]*SF[15] + P[12][10]*SPP[10]))/2;
	nextP[1][1] = P[1][1] + P[0][1]*SF[8] + P[2][1]*SF[7] + P[3][1]*SF[11] - P[12][1]*SF[15] + P[11][1]*SPP[10] + daxVar*SQ[9] - (P[10][1]*q0)/2 + SF[8]*(P[1][0] + P[0][0]*SF[8] + P[2][0]*SF[7] + P[3][0]*SF[11] - P[12][0]*SF[15] + P[11][0]*SPP[10] - (P[10][0]*q0)/2) + SF[7]*(P[1][2] + P[0][2]*SF[8] + P[2][2]*SF[7] + P[3][2]*SF[11] - P[12][2]*SF[15] + P[11][2]*SPP[10] - (P[10][2]*q0)/2) + SF[11]*(P[1][3] + P[0][3]*SF[8] + P[2][3]*SF[7] + P[3][3]*SF[11] - P[12][3]*SF[15] + P[11][3]*SPP[10] - (P[10][3]*q0)/2) - SF[15]*(P[1][12] + P[0][12]*SF[8] + P[2][12]*SF[7] + P[3][12]*SF[11] - P[12][12]*SF[15] + P[11][12]*SPP[10] - (P[10][12]*q0)/2) + SPP[10]*(P[1][11] + P[0][11]*SF[8] + P[2][11]*SF[7] + P[3][11]*SF[11] - P[12][11]*SF[15] + P[11][11]*SPP[10] - (P[10][11]*q0)/2) + (dayVar*(q3)*(q3))/4 + (dazVar*(q2)*(q2))/4 - (q0*(P[1][10] + P[0][10]*SF[8] + P[2][10]*SF[7] + P[3][10]*SF[11] - P[12][10]*SF[15] + P[11][10]*SPP[10] - (P[10][10]*q0)/2))/2;
	nextP[0][2] = P[0][2] + SQ[7] + P[1][2]*SF[9] + P[2][2]*SF[11] + P[3][2]*SF[10] + P[10][2]*SF[14] + P[11][2]*SF[15] + P[12][2]*SPP[10] + SF[6]*(P[0][0] + P[1][0]*SF[9] + P[2][0]*SF[11] + P[3][0]*SF[10] + P[10][0]*SF[14] + P[11][0]*SF[15] + P[12][0]*SPP[10]) + SF[10]*(P[0][1] + P[1][1]*SF[9] + P[2][1]*SF[11] + P[3][1]*SF[10] + P[10][1]*SF[14] + P[11][1]*SF[15] + P[12][1]*SPP[10]) + SF[8]*(P[0][3] + P[1][3]*SF[9] + P[2][3]*SF[11] + P[3][3]*SF[10] + P[10][3]*SF[14] + P[11][3]*SF[15] + P[12][3]*SPP[10]) + SF[14]*(P[0][12] + P[1][12]*SF[9] + P[2][12]*SF[11] + P[3][12]*SF[10] + P[10][12]*SF[14] + P[11][12]*SF[15] + P[12][12]*SPP[10]) - SPP[10]*(P[0][10] + P[1][10]*SF[9] + P[2][10]*SF[11] + P[3][10]*SF[10] + P[10][10]*SF[14] + P[11][10]*SF[15] + P[12][10]*SPP[10]) - (q0*(P[0][11] + P[1][11]*SF[9] + P[2][11]*SF[11] + P[3][11]*SF[10] + P[10][11]*SF[14] + P[11][11]*SF[15] + P[12][11]*SPP[10]))/2;
	nextP[1][2] = P[1][2] + SQ[5] + P[0][2]*SF[8] + P[2][2]*SF[7] + P[3][2]*SF[11] - P[12][2]*SF[15] + P[11][2]*SPP[10] - (P[10][2]*q0)/2 + SF[6]*(P[1][0] + P[0][0]*SF[8] + P[2][0]*SF[7] + P[3][0]*SF[11] - P[12][0]*SF[15] + P[11][0]*SPP[10] - (P[10][0]*q0)/2) + SF[10]*(P[1][1] + P[0][1]*SF[8] + P[2][1]*SF[7] + P[3][1]*SF[11] - P[12][1]*SF[15] + P[11][1]*SPP[10] - (P[10][1]*q0)/2) + SF[8]*(P[1][3] + P[0][3]*SF[8] + P[2][3]*SF[7] + P[3][3]*SF[11] - P[12][3]*SF[15] + P[11][3]*SPP[10] - (P[10][3]*q0)/2) + SF[14]*(P[1][12] + P[0][12]*SF[8] + P[2][12]*SF[7] + P[3][12]*SF[11] - P[12][12]*SF[15] + P[11][12]*SPP[10] - (P[10][12]*q0)/2) - SPP[10]*(P[1][10] + P[0][10]*SF[8] + P[2][10]*SF[7] + P[3][10]*SF[11] - P[12][10]*SF[15] + P[11][10]*SPP[10] - (P[10][10]*q0)/2) - (q0*(P[1][11] + P[0][11]*SF[8] + P[2][11]*SF[7] + P[3][11]*SF[11] - P[12][11]*SF[15] + P[11][11]*SPP[10] - (P[10][11]*q0)/2))/2;
	nextP[2][2] = P[2][2] + P[0][2]*SF[6] + P[1][2]*SF[10] + P[3][2]*SF[8] + P[12][2]*SF[14] - P[10][2]*SPP[10] + dayVar*SQ[9] + (dazVar*SQ[10])/4 - (P[11][2]*q0)/2 + SF[6]*(P[2][0] + P[0][0]*SF[6] + P[1][0]*SF[10] + P[3][0]*SF[8] + P[12][0]*SF[14] - P[10][0]*SPP[10] - (P[11][0]*q0)/2) + SF[10]*(P[2][1] + P[0][1]*SF[6] + P[1][1]*SF[10] + P[3][1]*SF[8] + P[12][1]*SF[14] - P[10][1]*SPP[10] - (P[11][1]*q0)/2) + SF[8]*(P[2][3] + P[0][3]*SF[6] + P[1][3]*SF[10] + P[3][3]*SF[8] + P[12][3]*SF[14] - P[10][3]*SPP[10] - (P[11][3]*q0)/2) + SF[14]*(P[2][12] + P[0][12]*SF[6] + P[1][12]*SF[10] + P[3][12]*SF[8] + P[12][12]*SF[14] - P[10][12]*SPP[10] - (P[11][12]*q0)/2) - SPP[10]*(P[2][10] + P[0][10]*SF[6] + P[1][10]*SF[10] + P[3][10]*SF[8] + P[12][10]*SF[14] - P[10][10]*SPP[10] - (P[11][10]*q0)/2) + (daxVar*(q3)*(q3))/4 - (q0*(P[2][11] + P[0][11]*SF[6] + P[1][11]*SF[10] + P[3][11]*SF[8] + P[12][11]*SF[14] - P[10][11]*SPP[10] - (P[11][11]*q0)/2))/2;
	nextP[0][3] = P[0][3] + SQ[6] + P[1][3]*SF[9] + P[2][3]*SF[11] + P[3][3]*SF[10] + P[10][3]*SF[14] + P[11][3]*SF[15] + P[12][3]*SPP[10] + SF[7]*(P[0][0] + P[1][0]*SF[9] + P[2][0]*SF[11] + P[3][0]*SF[10] + P[10][0]*SF[14] + P[11][0]*SF[15] + P[12][0]*SPP[10]) + SF[6]*(P[0][1] + P[1][1]*SF[9] + P[2][1]*SF[11] + P[3][1]*SF[10] + P[10][1]*SF[14] + P[11][1]*SF[15] + P[12][1]*SPP[10]) + SF[9]*(P[0][2] + P[1][2]*SF[9] + P[2][2]*SF[11] + P[3][2]*SF[10] + P[10][2]*SF[14] + P[11][2]*SF[15] + P[12][2]*SPP[10]) + SF[15]*(P[0][10] + P[1][10]*SF[9] + P[2][10]*SF[11] + P[3][10]*SF[10] + P[10][10]*SF[14] + P[11][10]*SF[15] + P[12][10]*SPP[10]) - SF[14]*(P[0][11] + P[1][11]*SF[9] + P[2][11]*SF[11] + P[3][11]*SF[10] + P[10][11]*SF[14] + P[11][11]*SF[15] + P[12][11]*SPP[10]) - (q0*(P[0][12] + P[1][12]*SF[9] + P[2][12]*SF[11] + P[3][12]*SF[10] + P[10][12]*SF[14] + P[11][12]*SF[15] + P[12][12]*SPP[10]))/2;
	nextP[1][3] = P[1][3] + SQ[4] + P[0][3]*SF[8] + P[2][3]*SF[7] + P[3][3]*SF[11] - P[12][3]*SF[15] + P[11][3]*SPP[10] - (P[10][3]*q0)/2 + SF[7]*(P[1][0] + P[0][0]*SF[8] + P[2][0]*SF[7] + P[3][0]*SF[11] - P[12][0]*SF[15] + P[11][0]*SPP[10] - (P[10][0]*q0)/2) + SF[6]*(P[1][1] + P[0][1]*SF[8] + P[2][1]*SF[7] + P[3][1]*SF[11] - P[12][1]*SF[15] + P[11][1]*SPP[10] - (P[10][1]*q0)/2) + SF[9]*(P[1][2] + P[0][2]*SF[8] + P[2][2]*SF[7] + P[3][2]*SF[11] - P[12][2]*SF[15] + P[11][2]*SPP[10] - (P[10][2]*q0)/2) + SF[15]*(P[1][10] + P[0][10]*SF[8] + P[2][10]*SF[7] + P[3][10]*SF[11] - P[12][10]*SF[15] + P[11][10]*SPP[10] - (P[10][10]*q0)/2) - SF[14]*(P[1][11] + P[0][11]*SF[8] + P[2][11]*SF[7] + P[3][11]*SF[11] - P[12][11]*SF[15] + P[11][11]*SPP[10] - (P[10][11]*q0)/2) - (q0*(P[1][12] + P[0][12]*SF[8] + P[2][12]*SF[7] + P[3][12]*SF[11] - P[12][12]*SF[15] + P[11][12]*SPP[10] - (P[10][12]*q0)/2))/2;
	nextP[2][3] = P[2][3] + SQ[3] + P[0][3]*SF[6] + P[1][3]*SF[10] + P[3][3]*SF[8] + P[12][3]*SF[14] - P[10][3]*SPP[10] - (P[11][3]*q0)/2 + SF[7]*(P[2][0] + P[0][0]*SF[6] + P[1][0]*SF[10] + P[3][0]*SF[8] + P[12][0]*SF[14] - P[10][0]*SPP[10] - (P[11][0]*q0)/2) + SF[6]*(P[2][1] + P[0][1]*SF[6] + P[1][1]*SF[10] + P[3][1]*SF[8] + P[12][1]*SF[14] - P[10][1]*SPP[10] - (P[11][1]*q0)/2) + SF[9]*(P[2][2] + P[0][2]*SF[6] + P[1][2]*SF[10] + P[3][2]*SF[8] + P[12][2]*SF[14] - P[10][2]*SPP[10] - (P[11][2]*q0)/2) + SF[15]*(P[2][10] + P[0][10]*SF[6] + P[1][10]*SF[10] + P[3][10]*SF[8] + P[12][10]*SF[14] - P[10][10]*SPP[10] - (P[11][10]*q0)/2) - SF[14]*(P[2][11] + P[0][11]*SF[6] + P[1][11]*SF[10] + P[3][11]*SF[8] + P[12][11]*SF[14] - P[10][11]*SPP[10] - (P[11][11]*q0)/2) - (q0*(P[2][12] + P[0][12]*SF[6] + P[1][12]*SF[10] + P[3][12]*SF[8] + P[12][12]*SF[14] - P[10][12]*SPP[10] - (P[11][12]*q0)/2))/2;
	nextP[3][3] = P[3][3] + P[0][3]*SF[7] + P[1][3]*SF[6] + P[2][3]*SF[9] + P[10][3]*SF[15] - P[11][3]*SF[14] + (dayVar*SQ[10])/4 + dazVar*SQ[9] - (P[12][3]*q0)/2 + SF[7]*(P[3][0] + P[0][0]*SF[7] + P[1][0]*SF[6] + P[2][0]*SF[9] + P[10][0]*SF[15] - P[11][0]*SF[14] - (P[12][0]*q0)/2) + SF[6]*(P[3][1] + P[0][1]*SF[7] + P[1][1]*SF[6] + P[2][1]*SF[9] + P[10][1]*SF[15] - P[11][1]*SF[14] - (P[12][1]*q0)/2) + SF[9]*(P[3][2] + P[0][2]*SF[7] + P[1][2]*SF[6] + P[2][2]*SF[9] + P[10][2]*SF[15] - P[11][2]*SF[14] - (P[12][2]*q0)/2) + SF[15]*(P[3][10] + P[0][10]*SF[7] + P[1][10]*SF[6] + P[2][10]*SF[9] + P[10][10]*SF[15] - P[11][10]*SF[14] - (P[12][10]*q0)/2) - SF[14]*(P[3][11] + P[0][11]*SF[7] + P[1][11]*SF[6] + P[2][11]*SF[9] + P[10][11]*SF[15] - P[11][11]*SF[14] - (P[12][11]*q0)/2) + (daxVar*(q2)*(q2))/4 - (q0*(P[3][12] + P[0][12]*SF[7] + P[1][12]*SF[6] + P[2][12]*SF[9] + P[10][12]*SF[15] - P[11][12]*SF[14] - (P[12][12]*q0)/2))/2;
	nextP[0][4] = P[0][4] + P[1][4]*SF[9] + P[2][4]*SF[11] + P[3][4]*SF[10] + P[10][4]*SF[14] + P[11][4]*SF[15] + P[12][4]*SPP[10] + SF[5]*(P[0][0] + P[1][0]*SF[9] + P[2][0]*SF[11] + P[3][0]*SF[10] + P[10][0]*SF[14] + P[11][0]*SF[15] + P[12][0]*SPP[10]) + SF[3]*(P[0][1] + P[1][1]*SF[9] + P[2][1]*SF[11] + P[3][1]*SF[10] + P[10][1]*SF[14] + P[11][1]*SF[15] + P[12][1]*SPP[10]) - SF[4]*(P[0][3] + P[1][3]*SF[9] + P[2][3]*SF[11] + P[3][3]*SF[10] + P[10][3]*SF[14] + P[11][3]*SF[15] + P[12][3]*SPP[10]) + SPP[0]*(P[0][2] + P[1][2]*SF[9] + P[2][2]*SF[11] + P[3][2]*SF[10] + P[10][2]*SF[14] + P[11][2]*SF[15] + P[12][2]*SPP[10]) + SPP[3]*(P[0][13] + P[1][13]*SF[9] + P[2][13]*SF[11] + P[3][13]*SF[10] + P[10][13]*SF[14] + P[11][13]*SF[15] + P[12][13]*SPP[10]) + SPP[6]*(P[0][14] + P[1][14]*SF[9] + P[2][14]*SF[11] + P[3][14]*SF[10] + P[10][14]*SF[14] + P[11][14]*SF[15] + P[12][14]*SPP[10]) - SPP[9]*(P[0][15] + P[1][15]*SF[9] + P[2][15]*SF[11] + P[3][15]*SF[10] + P[10][15]*SF[14] + P[11][15]*SF[15] + P[12][15]*SPP[10]);
	nextP[1][4] = P[1][4] + P[0][4]*SF[8] + P[2][4]*SF[7] + P[3][4]*SF[11] - P[12][4]*SF[15] + P[11][4]*SPP[10] - (P[10][4]*q0)/2 + SF[5]*(P[1][0] + P[0][0]*SF[8] + P[2][0]*SF[7] + P[3][0]*SF[11] - P[12][0]*SF[15] + P[11][0]*SPP[10] - (P[10][0]*q0)/2) + SF[3]*(P[1][1] + P[0][1]*SF[8] + P[2][1]*SF[7] + P[3][1]*SF[11] - P[12][1]*SF[15] + P[11][1]*SPP[10] - (P[10][1]*q0)/2) - SF[4]*(P[1][3] + P[0][3]*SF[8] + P[2][3]*SF[7] + P[3][3]*SF[11] - P[12][3]*SF[15] + P[11][3]*SPP[10] - (P[10][3]*q0)/2) + SPP[0]*(P[1][2] + P[0][2]*SF[8] + P[2][2]*SF[7] + P[3][2]*SF[11] - P[12][2]*SF[15] + P[11][2]*SPP[10] - (P[10][2]*q0)/2) + SPP[3]*(P[1][13] + P[0][13]*SF[8] + P[2][13]*SF[7] + P[3][13]*SF[11] - P[12][13]*SF[15] + P[11][13]*SPP[10] - (P[10][13]*q0)/2) + SPP[6]*(P[1][14] + P[0][14]*SF[8] + P[2][14]*SF[7] + P[3][14]*SF[11] - P[12][14]*SF[15] + P[11][14]*SPP[10] - (P[10][14]*q0)/2) - SPP[9]*(P[1][15] + P[0][15]*SF[8] + P[2][15]*SF[7] + P[3][15]*SF[11] - P[12][15]*SF[15] + P[11][15]*SPP[10] - (P[10][15]*q0)/2);
	nextP[2][4] = P[2][4] + P[0][4]*SF[6] + P[1][4]*SF[10] + P[3][4]*SF[8] + P[12][4]*SF[14] - P[10][4]*SPP[10] - (P[11][4]*q0)/2 + SF[5]*(P[2][0] + P[0][0]*SF[6] + P[1][0]*SF[10] + P[3][0]*SF[8] + P[12][0]*SF[14] - P[10][0]*SPP[10] - (P[11][0]*q0)/2) + SF[3]*(P[2][1] + P[0][1]*SF[6] + P[1][1]*SF[10] + P[3][1]*SF[8] + P[12][1]*SF[14] - P[10][1]*SPP[10] - (P[11][1]*q0)/2) - SF[4]*(P[2][3] + P[0][3]*SF[6] + P[1][3]*SF[10] + P[3][3]*SF[8] + P[12][3]*SF[14] - P[10][3]*SPP[10] - (P[11][3]*q0)/2) + SPP[0]*(P[2][2] + P[0][2]*SF[6] + P[1][2]*SF[10] + P[3][2]*SF[8] + P[12][2]*SF[14] - P[10][2]*SPP[10] - (P[11][2]*q0)/2) + SPP[3]*(P[2][13] + P[0][13]*SF[6] + P[1][13]*SF[10] + P[3][13]*SF[8] + P[12][13]*SF[14] - P[10][13]*SPP[10] - (P[11][13]*q0)/2) + SPP[6]*(P[2][14] + P[0][14]*SF[6] + P[1][14]*SF[10] + P[3][14]*SF[8] + P[12][14]*SF[14] - P[10][14]*SPP[10] - (P[11][14]*q0)/2) - SPP[9]*(P[2][15] + P[0][15]*SF[6] + P[1][15]*SF[10] + P[3][15]*SF[8] + P[12][15]*SF[14] - P[10][15]*SPP[10] - (P[11][15]*q0)/2);
	nextP[3][4] = P[3][4] + P[0][4]*SF[7] + P[1][4]*SF[6] + P[2][4]*SF[9] + P[10][4]*SF[15] - P[11][4]*SF[14] - (P[12][4]*q0)/2 + SF[5]*(P[3][0] + P[0][0]*SF[7] + P[1][0]*SF[6] + P[2][0]*SF[9] + P[10][0]*SF[15] - P[11][0]*SF[14] - (P[12][0]*q0)/2) + SF[3]*(P[3][1] + P[0][1]*SF[7] + P[1][1]*SF[6] + P[2][1]*SF[9] + P[10][1]*SF[15] - P[11][1]*SF[14] - (P[12][1]*q0)/2) - SF[4]*(P[3][3] + P[0][3]*SF[7] + P[1][3]*SF[6] + P[2][3]*SF[9] + P[10][3]*SF[15] - P[11][3]*SF[14] - (P[12][3]*q0)/2) + SPP[0]*(P[3][2] + P[0][2]*SF[7] + P[1][2]*SF[6] + P[2][2]*SF[9] + P[10][2]*SF[15] - P[11][2]*SF[14] - (P[12][2]*q0)/2) + SPP[3]*(P[3][13] + P[0][13]*SF[7] + P[1][13]*SF[6] + P[2][13]*SF[9] + P[10][13]*SF[15] - P[11][13]*SF[14] - (P[12][13]*q0)/2) + SPP[6]*(P[3][14] + P[0][14]*SF[7] + P[1][14]*SF[6] + P[2][14]*SF[9] + P[10][14]*SF[15] - P[11][14]*SF[14] - (P[12][14]*q0)/2) - SPP[9]*(P[3][15] + P[0][15]*SF[7] + P[1][15]*SF[6] + P[2][15]*SF[9] + P[10][15]*SF[15] - P[11][15]*SF[14] - (P[12][15]*q0)/2);
	nextP[4][4] = P[4][4] + P[0][4]*SF[5] + P[1][4]*SF[3] - P[3][4]*SF[4] + P[2][4]*SPP[0] + P[13][4]*SPP[3] + P[14][4]*SPP[6] - P[15][4]*SPP[9] + dvyVar*(SG[7] - 2*q0*q3)*(SG[7] - 2*q0*q3) + dvzVar*(SG[6] + 2*q0*q2)*(SG[6] + 2*q0*q2) + SF[5]*(P[4][0] + P[0][0]*SF[5] + P[1][0]*SF[3] - P[3][0]*SF[4] + P[2][0]*SPP[0] + P[13][0]*SPP[3] + P[14][0]*SPP[6] - P[15][0]*SPP[9]) + SF[3]*(P[4][1] + P[0][1]*SF[5] + P[1][1]*SF[3] - P[3][1]*SF[4] + P[2][1]*SPP[0] + P[13][1]*SPP[3] + P[14][1]*SPP[6] - P[15][1]*SPP[9]) - SF[4]*(P[4][3] + P[0][3]*SF[5] + P[1][3]*SF[3] - P[3][3]*SF[4] + P[2][3]*SPP[0] + P[13][3]*SPP[3] + P[14][3]*SPP[6] - P[15][3]*SPP[9]) + SPP[0]*(P[4][2] + P[0][2]*SF[5] + P[1][2]*SF[3] - P[3][2]*SF[4] + P[2][2]*SPP[0] + P[13][2]*SPP[3] + P[14][2]*SPP[6] - P[15][2]*SPP[9]) + SPP[3]*(P[4][13] + P[0][13]*SF[5] + P[1][13]*SF[3] - P[3][13]*SF[4] + P[2][13]*SPP[0] + P[13][13]*SPP[3] + P[14][13]*SPP[6] - P[15][13]*SPP[9]) + SPP[6]*(P[4][14] + P[0][14]*SF[5] + P[1][14]*SF[3] - P[3][14]*SF[4] + P[2][14]*SPP[0] + P[13][14]*SPP[3] + P[14][14]*SPP[6] - P[15][14]*SPP[9]) - SPP[9]*(P[4][15] + P[0][15]*SF[5] + P[1][15]*SF[3] - P[3][15]*SF[4] + P[2][15]*SPP[0] + P[13][15]*SPP[3] + P[14][15]*SPP[6] - P[15][15]*SPP[9]) + dvxVar*(SG[1] + SG[2] - SG[3] - SG[4])*(SG[1] + SG[2] - SG[3] - SG[4]);
	nextP[0][5] = P[0][5] + P[1][5]*SF[9] + P[2][5]*SF[11] + P[3][5]*SF[10] + P[10][5]*SF[14] + P[11][5]*SF[15] + P[12][5]*SPP[10] + SF[4]*(P[0][0] + P[1][0]*SF[9] + P[2][0]*SF[11] + P[3][0]*SF[10] + P[10][0]*SF[14] + P[11][0]*SF[15] + P[12][0]*SPP[10]) + SF[3]*(P[0][2] + P[1][2]*SF[9] + P[2][2]*SF[11] + P[3][2]*SF[10] + P[10][2]*SF[14] + P[11][2]*SF[15] + P[12][2]*SPP[10]) + SF[5]*(P[0][3] + P[1][3]*SF[9] + P[2][3]*SF[11] + P[3][3]*SF[10] + P[10][3]*SF[14] + P[11][3]*SF[15] + P[12][3]*SPP[10]) - SPP[0]*(P[0][1] + P[1][1]*SF[9] + P[2][1]*SF[11] + P[3][1]*SF[10] + P[10][1]*SF[14] + P[11][1]*SF[15] + P[12][1]*SPP[10]) - SPP[8]*(P[0][13] + P[1][13]*SF[9] + P[2][13]*SF[11] + P[3][13]*SF[10] + P[10][13]*SF[14] + P[11][13]*SF[15] + P[12][13]*SPP[10]) + SPP[2]*(P[0][14] + P[1][14]*SF[9] + P[2][14]*SF[11] + P[3][14]*SF[10] + P[10][14]*SF[14] + P[11][14]*SF[15] + P[12][14]*SPP[10]) + SPP[5]*(P[0][15] + P[1][15]*SF[9] + P[2][15]*SF[11] + P[3][15]*SF[10] + P[10][15]*SF[14] + P[11][15]*SF[15] + P[12][15]*SPP[10]);
	nextP[1][5] = P[1][5] + P[0][5]*SF[8] + P[2][5]*SF[7] + P[3][5]*SF[11] - P[12][5]*SF[15] + P[11][5]*SPP[10] - (P[10][5]*q0)/2 + SF[4]*(P[1][0] + P[0][0]*SF[8] + P[2][0]*SF[7] + P[3][0]*SF[11] - P[12][0]*SF[15] + P[11][0]*SPP[10] - (P[10][0]*q0)/2) + SF[3]*(P[1][2] + P[0][2]*SF[8] + P[2][2]*SF[7] + P[3][2]*SF[11] - P[12][2]*SF[15] + P[11][2]*SPP[10] - (P[10][2]*q0)/2) + SF[5]*(P[1][3] + P[0][3]*SF[8] + P[2][3]*SF[7] + P[3][3]*SF[11] - P[12][3]*SF[15] + P[11][3]*SPP[10] - (P[10][3]*q0)/2) - SPP[0]*(P[1][1] + P[0][1]*SF[8] + P[2][1]*SF[7] + P[3][1]*SF[11] - P[12][1]*SF[15] + P[11][1]*SPP[10] - (P[10][1]*q0)/2) - SPP[8]*(P[1][13] + P[0][13]*SF[8] + P[2][13]*SF[7] + P[3][13]*SF[11] - P[12][13]*SF[15] + P[11][13]*SPP[10] - (P[10][13]*q0)/2) + SPP[2]*(P[1][14] + P[0][14]*SF[8] + P[2][14]*SF[7] + P[3][14]*SF[11] - P[12][14]*SF[15] + P[11][14]*SPP[10] - (P[10][14]*q0)/2) + SPP[5]*(P[1][15] + P[0][15]*SF[8] + P[2][15]*SF[7] + P[3][15]*SF[11] - P[12][15]*SF[15] + P[11][15]*SPP[10] - (P[10][15]*q0)/2);
	nextP[2][5] = P[2][5] + P[0][5]*SF[6] + P[1][5]*SF[10] + P[3][5]*SF[8] + P[12][5]*SF[14] - P[10][5]*SPP[10] - (P[11][5]*q0)/2 + SF[4]*(P[2][0] + P[0][0]*SF[6] + P[1][0]*SF[10] + P[3][0]*SF[8] + P[12][0]*SF[14] - P[10][0]*SPP[10] - (P[11][0]*q0)/2) + SF[3]*(P[2][2] + P[0][2]*SF[6] + P[1][2]*SF[10] + P[3][2]*SF[8] + P[12][2]*SF[14] - P[10][2]*SPP[10] - (P[11][2]*q0)/2) + SF[5]*(P[2][3] + P[0][3]*SF[6] + P[1][3]*SF[10] + P[3][3]*SF[8] + P[12][3]*SF[14] - P[10][3]*SPP[10] - (P[11][3]*q0)/2) - SPP[0]*(P[2][1] + P[0][1]*SF[6] + P[1][1]*SF[10] + P[3][1]*SF[8] + P[12][1]*SF[14] - P[10][1]*SPP[10] - (P[11][1]*q0)/2) - SPP[8]*(P[2][13] + P[0][13]*SF[6] + P[1][13]*SF[10] + P[3][13]*SF[8] + P[12][13]*SF[14] - P[10][13]*SPP[10] - (P[11][13]*q0)/2) + SPP[2]*(P[2][14] + P[0][14]*SF[6] + P[1][14]*SF[10] + P[3][14]*SF[8] + P[12][14]*SF[14] - P[10][14]*SPP[10] - (P[11][14]*q0)/2) + SPP[5]*(P[2][15] + P[0][15]*SF[6] + P[1][15]*SF[10] + P[3][15]*SF[8] + P[12][15]*SF[14] - P[10][15]*SPP[10] - (P[11][15]*q0)/2);
	nextP[3][5] = P[3][5] + P[0][5]*SF[7] + P[1][5]*SF[6] + P[2][5]*SF[9] + P[10][5]*SF[15] - P[11][5]*SF[14] - (P[12][5]*q0)/2 + SF[4]*(P[3][0] + P[0][0]*SF[7] + P[1][0]*SF[6] + P[2][0]*SF[9] + P[10][0]*SF[15] - P[11][0]*SF[14] - (P[12][0]*q0)/2) + SF[3]*(P[3][2] + P[0][2]*SF[7] + P[1][2]*SF[6] + P[2][2]*SF[9] + P[10][2]*SF[15] - P[11][2]*SF[14] - (P[12][2]*q0)/2) + SF[5]*(P[3][3] + P[0][3]*SF[7] + P[1][3]*SF[6] + P[2][3]*SF[9] + P[10][3]*SF[15] - P[11][3]*SF[14] - (P[12][3]*q0)/2) - SPP[0]*(P[3][1] + P[0][1]*SF[7] + P[1][1]*SF[6] + P[2][1]*SF[9] + P[10][1]*SF[15] - P[11][1]*SF[14] - (P[12][1]*q0)/2) - SPP[8]*(P[3][13] + P[0][13]*SF[7] + P[1][13]*SF[6] + P[2][13]*SF[9] + P[10][13]*SF[15] - P[11][13]*SF[14] - (P[12][13]*q0)/2) + SPP[2]*(P[3][14] + P[0][14]*SF[7] + P[1][14]*SF[6] + P[2][14]*SF[9] + P[10][14]*SF[15] - P[11][14]*SF[14] - (P[12][14]*q0)/2) + SPP[5]*(P[3][15] + P[0][15]*SF[7] + P[1][15]*SF[6] + P[2][15]*SF[9] + P[10][15]*SF[15] - P[11][15]*SF[14] - (P[12][15]*q0)/2);
	nextP[4][5] = P[4][5] + SQ[2] + P[0][5]*SF[5] + P[1][5]*SF[3] - P[3][5]*SF[4] + P[2][5]*SPP[0] + P[13][5]*SPP[3] + P[14][5]*SPP[6] - P[15][5]*SPP[9] + SF[4]*(P[4][0] + P[0][0]*SF[5] + P[1][0]*SF[3] - P[3][0]*SF[4] + P[2][0]*SPP[0] + P[13][0]*SPP[3] + P[14][0]*SPP[6] - P[15][0]*SPP[9]) + SF[3]*(P[4][2] + P[0][2]*SF[5] + P[1][2]*SF[3] - P[3][2]*SF[4] + P[2][2]*SPP[0] + P[13][2]*SPP[3] + P[14][2]*SPP[6] - P[15][2]*SPP[9]) + SF[5]*(P[4][3] + P[0][3]*SF[5] + P[1][3]*SF[3] - P[3][3]*SF[4] + P[2][3]*SPP[0] + P[13][3]*SPP[3] + P[14][3]*SPP[6] - P[15][3]*SPP[9]) - SPP[0]*(P[4][1] + P[0][1]*SF[5] + P[1][1]*SF[3] - P[3][1]*SF[4] + P[2][1]*SPP[0] + P[13][1]*SPP[3] + P[14][1]*SPP[6] - P[15][1]*SPP[9]) - SPP[8]*(P[4][13] + P[0][13]*SF[5] + P[1][13]*SF[3] - P[3][13]*SF[4] + P[2][13]*SPP[0] + P[13][13]*SPP[3] + P[14][13]*SPP[6] - P[15][13]*SPP[9]) + SPP[2]*(P[4][14] + P[0][14]*SF[5] + P[1][14]*SF[3] - P[3][14]*SF[4] + P[2][14]*SPP[0] + P[13][14]*SPP[3] + P[14][14]*SPP[6] - P[15][14]*SPP[9]) + SPP[5]*(P[4][15] + P[0][15]*SF[5] + P[1][15]*SF[3] - P[3][15]*SF[4] + P[2][15]*SPP[0] + P[13][15]*SPP[3] + P[14][15]*SPP[6] - P[15][15]*SPP[9]);
	nextP[5][5] = P[5][5] + P[0][5]*SF[4] + P[2][5]*SF[3] + P[3][5]*SF[5] - P[1][5]*SPP[0] - P[13][5]*SPP[8] + P[14][5]*SPP[2] + P[15][5]*SPP[5] + dvxVar*(SG[7] + 2*q0*q3)*(SG[7] + 2*q0*q3) + dvzVar*(SG[5] - 2*q0*q1)*(SG[5] - 2*q0*q1) + SF[4]*(P[5][0] + P[0][0]*SF[4] + P[2][0]*SF[3] + P[3][0]*SF[5] - P[1][0]*SPP[0] - P[13][0]*SPP[8] + P[14][0]*SPP[2] + P[15][0]*SPP[5]) + SF[3]*(P[5][2] + P[0][2]*SF[4] + P[2][2]*SF[3] + P[3][2]*SF[5] - P[1][2]*SPP[0] - P[13][2]*SPP[8] + P[14][2]*SPP[2] + P[15][2]*SPP[5]) + SF[5]*(P[5][3] + P[0][3]*SF[4] + P[2][3]*SF[3] + P[3][3]*SF[5] - P[1][3]*SPP[0] - P[13][3]*SPP[8] + P[14][3]*SPP[2] + P[15][3]*SPP[5]) - SPP[0]*(P[5][1] + P[0][1]*SF[4] + P[2][1]*SF[3] + P[3][1]*SF[5] - P[1][1]*SPP[0] - P[13][1]*SPP[8] + P[14][1]*SPP[2] + P[15][1]*SPP[5]) - SPP[8]*(P[5][13] + P[0][13]*SF[4] + P[2][13]*SF[3] + P[3][13]*SF[5] - P[1][13]*SPP[0] - P[13][13]*SPP[8] + P[14][13]*SPP[2] + P[15][13]*SPP[5]) + SPP[2]*(P[5][14] + P[0][14]*SF[4] + P[2][14]*SF[3] + P[3][14]*SF[5] - P[1][14]*SPP[0] - P[13][14]*SPP[8] + P[14][14]*SPP[2] + P[15][14]*SPP[5]) + SPP[5]*(P[5][15] + P[0][15]*SF[4] + P[2][15]*SF[3] + P[3][15]*SF[5] - P[1][15]*SPP[0] - P[13][15]*SPP[8] + P[14][15]*SPP[2] + P[15][15]*SPP[5]) + dvyVar*(SG[1] - SG[2] + SG[3] - SG[4])*(SG[1] - SG[2] + SG[3] - SG[4]);
	nextP[0][6] = P[0][6] + P[1][6]*SF[9] + P[2][6]*SF[11] + P[3][6]*SF[10] + P[10][6]*SF[14] + P[11][6]*SF[15] + P[12][6]*SPP[10] + SF[4]*(P[0][1] + P[1][1]*SF[9] + P[2][1]*SF[11] + P[3][1]*SF[10] + P[10][1]*SF[14] + P[11][1]*SF[15] + P[12][1]*SPP[10]) - SF[5]*(P[0][2] + P[1][2]*SF[9] + P[2][2]*SF[11] + P[3][2]*SF[10] + P[10][2]*SF[14] + P[11][2]*SF[15] + P[12][2]*SPP[10]) + SF[3]*(P[0][3] + P[1][3]*SF[9] + P[2][3]*SF[11] + P[3][3]*SF[10] + P[10][3]*SF[14] + P[11][3]*SF[15] + P[12][3]*SPP[10]) + SPP[0]*(P[0][0] + P[1][0]*SF[9] + P[2][0]*SF[11] + P[3][0]*SF[10] + P[10][0]*SF[14] + P[11][0]*SF[15] + P[12][0]*SPP[10]) + SPP[4]*(P[0][13] + P[1][13]*SF[9] + P[2][13]*SF[11] + P[3][13]*SF[10] + P[10][13]*SF[14] + P[11][13]*SF[15] + P[12][13]*SPP[10]) - SPP[7]*(P[0][14] + P[1][14]*SF[9] + P[2][14]*SF[11] + P[3][14]*SF[10] + P[10][14]*SF[14] + P[11][14]*SF[15] + P[12][14]*SPP[10]) - SPP[1]*(P[0][15] + P[1][15]*SF[9] + P[2][15]*SF[11] + P[3][15]*SF[10] + P[10][15]*SF[14] + P[11][15]*SF[15] + P[12][15]*SPP[10]);
	nextP[1][6] = P[1][6] + P[0][6]*SF[8] + P[2][6]*SF[7] + P[3][6]*SF[11] - P[12][6]*SF[15] + P[11][6]*SPP[10] - (P[10][6]*q0)/2 + SF[4]*(P[1][1] + P[0][1]*SF[8] + P[2][1]*SF[7] + P[3][1]*SF[11] - P[12][1]*SF[15] + P[11][1]*SPP[10] - (P[10][1]*q0)/2) - SF[5]*(P[1][2] + P[0][2]*SF[8] + P[2][2]*SF[7] + P[3][2]*SF[11] - P[12][2]*SF[15] + P[11][2]*SPP[10] - (P[10][2]*q0)/2) + SF[3]*(P[1][3] + P[0][3]*SF[8] + P[2][3]*SF[7] + P[3][3]*SF[11] - P[12][3]*SF[15] + P[11][3]*SPP[10] - (P[10][3]*q0)/2) + SPP[0]*(P[1][0] + P[0][0]*SF[8] + P[2][0]*SF[7] + P[3][0]*SF[11] - P[12][0]*SF[15] + P[11][0]*SPP[10] - (P[10][0]*q0)/2) + SPP[4]*(P[1][13] + P[0][13]*SF[8] + P[2][13]*SF[7] + P[3][13]*SF[11] - P[12][13]*SF[15] + P[11][13]*SPP[10] - (P[10][13]*q0)/2) - SPP[7]*(P[1][14] + P[0][14]*SF[8] + P[2][14]*SF[7] + P[3][14]*SF[11] - P[12][14]*SF[15] + P[11][14]*SPP[10] - (P[10][14]*q0)/2) - SPP[1]*(P[1][15] + P[0][15]*SF[8] + P[2][15]*SF[7] + P[3][15]*SF[11] - P[12][15]*SF[15] + P[11][15]*SPP[10] - (P[10][15]*q0)/2);
	nextP[2][6] = P[2][6] + P[0][6]*SF[6] + P[1][6]*SF[10] + P[3][6]*SF[8] + P[12][6]*SF[14] - P[10][6]*SPP[10] - (P[11][6]*q0)/2 + SF[4]*(P[2][1] + P[0][1]*SF[6] + P[1][1]*SF[10] + P[3][1]*SF[8] + P[12][1]*SF[14] - P[10][1]*SPP[10] - (P[11][1]*q0)/2) - SF[5]*(P[2][2] + P[0][2]*SF[6] + P[1][2]*SF[10] + P[3][2]*SF[8] + P[12][2]*SF[14] - P[10][2]*SPP[10] - (P[11][2]*q0)/2) + SF[3]*(P[2][3] + P[0][3]*SF[6] + P[1][3]*SF[10] + P[3][3]*SF[8] + P[12][3]*SF[14] - P[10][3]*SPP[10] - (P[11][3]*q0)/2) + SPP[0]*(P[2][0] + P[0][0]*SF[6] + P[1][0]*SF[10] + P[3][0]*SF[8] + P[12][0]*SF[14] - P[10][0]*SPP[10] - (P[11][0]*q0)/2) + SPP[4]*(P[2][13] + P[0][13]*SF[6] + P[1][13]*SF[10] + P[3][13]*SF[8] + P[12][13]*SF[14] - P[10][13]*SPP[10] - (P[11][13]*q0)/2) - SPP[7]*(P[2][14] + P[0][14]*SF[6] + P[1][14]*SF[10] + P[3][14]*SF[8] + P[12][14]*SF[14] - P[10][14]*SPP[10] - (P[11][14]*q0)/2) - SPP[1]*(P[2][15] + P[0][15]*SF[6] + P[1][15]*SF[10] + P[3][15]*SF[8] + P[12][15]*SF[14] - P[10][15]*SPP[10] - (P[11][15]*q0)/2);
	nextP[3][6] = P[3][6] + P[0][6]*SF[7] + P[1][6]*SF[6] + P[2][6]*SF[9] + P[10][6]*SF[15] - P[11][6]*SF[14] - (P[12][6]*q0)/2 + SF[4]*(P[3][1] + P[0][1]*SF[7] + P[1][1]*SF[6] + P[2][1]*SF[9] + P[10][1]*SF[15] - P[11][1]*SF[14] - (P[12][1]*q0)/2) - SF[5]*(P[3][2] + P[0][2]*SF[7] + P[1][2]*SF[6] + P[2][2]*SF[9] + P[10][2]*SF[15] - P[11][2]*SF[14] - (P[12][2]*q0)/2) + SF[3]*(P[3][3] + P[0][3]*SF[7] + P[1][3]*SF[6] + P[2][3]*SF[9] + P[10][3]*SF[15] - P[11][3]*SF[14] - (P[12][3]*q0)/2) + SPP[0]*(P[3][0] + P[0][0]*SF[7] + P[1][0]*SF[6] + P[2][0]*SF[9] + P[10][0]*SF[15] - P[11][0]*SF[14] - (P[12][0]*q0)/2) + SPP[4]*(P[3][13] + P[0][13]*SF[7] + P[1][13]*SF[6] + P[2][13]*SF[9] + P[10][13]*SF[15] - P[11][13]*SF[14] - (P[12][13]*q0)/2) - SPP[7]*(P[3][14] + P[0][14]*SF[7] + P[1][14]*SF[6] + P[2][14]*SF[9] + P[10][14]*SF[15] - P[11][14]*SF[14] - (P[12][14]*q0)/2) - SPP[1]*(P[3][15] + P[0][15]*SF[7] + P[1][15]*SF[6] + P[2][15]*SF[9] + P[10][15]*SF[15] - P[11][15]*SF[14] - (P[12][15]*q0)/2);
	nextP[4][6] = P[4][6] + SQ[1] + P[0][6]*SF[5] + P[1][6]*SF[3] - P[3][6]*SF[4] + P[2][6]*SPP[0] + P[13][6]*SPP[3] + P[14][6]*SPP[6] - P[15][6]*SPP[9] + SF[4]*(P[4][1] + P[0][1]*SF[5] + P[1][1]*SF[3] - P[3][1]*SF[4] + P[2][1]*SPP[0] + P[13][1]*SPP[3] + P[14][1]*SPP[6] - P[15][1]*SPP[9]) - SF[5]*(P[4][2] + P[0][2]*SF[5] + P[1][2]*SF[3] - P[3][2]*SF[4] + P[2][2]*SPP[0] + P[13][2]*SPP[3] + P[14][2]*SPP[6] - P[15][2]*SPP[9]) + SF[3]*(P[4][3] + P[0][3]*SF[5] + P[1][3]*SF[3] - P[3][3]*SF[4] + P[2][3]*SPP[0] + P[13][3]*SPP[3] + P[14][3]*SPP[6] - P[15][3]*SPP[9]) + SPP[0]*(P[4][0] + P[0][0]*SF[5] + P[1][0]*SF[3] - P[3][0]*SF[4] + P[2][0]*SPP[0] + P[13][0]*SPP[3] + P[14][0]*SPP[6] - P[15][0]*SPP[9]) + SPP[4]*(P[4][13] + P[0][13]*SF[5] + P[1][13]*SF[3] - P[3][13]*SF[4] + P[2][13]*SPP[0] + P[13][13]*SPP[3] + P[14][13]*SPP[6] - P[15][13]*SPP[9]) - SPP[7]*(P[4][14] + P[0][14]*SF[5] + P[1][14]*SF[3] - P[3][14]*SF[4] + P[2][14]*SPP[0] + P[13][14]*SPP[3] + P[14][14]*SPP[6] - P[15][14]*SPP[9]) - SPP[1]*(P[4][15] + P[0][15]*SF[5] + P[1][15]*SF[3] - P[3][15]*SF[4] + P[2][15]*SPP[0] + P[13][15]*SPP[3] + P[14][15]*SPP[6] - P[15][15]*SPP[9]);
	nextP[5][6] = P[5][6] + SQ[0] + P[0][6]*SF[4] + P[2][6]*SF[3] + P[3][6]*SF[5] - P[1][6]*SPP[0] - P[13][6]*SPP[8] + P[14][6]*SPP[2] + P[15][6]*SPP[5] + SF[4]*(P[5][1] + P[0][1]*SF[4] + P[2][1]*SF[3] + P[3][1]*SF[5] - P[1][1]*SPP[0] - P[13][1]*SPP[8] + P[14][1]*SPP[2] + P[15][1]*SPP[5]) - SF[5]*(P[5][2] + P[0][2]*SF[4] + P[2][2]*SF[3] + P[3][2]*SF[5] - P[1][2]*SPP[0] - P[13][2]*SPP[8] + P[14][2]*SPP[2] + P[15][2]*SPP[5]) + SF[3]*(P[5][3] + P[0][3]*SF[4] + P[2][3]*SF[3] + P[3][3]*SF[5] - P[1][3]*SPP[0] - P[13][3]*SPP[8] + P[14][3]*SPP[2] + P[15][3]*SPP[5]) + SPP[0]*(P[5][0] + P[0][0]*SF[4] + P[2][0]*SF[3] + P[3][0]*SF[5] - P[1][0]*SPP[0] - P[13][0]*SPP[8] + P[14][0]*SPP[2] + P[15][0]*SPP[5]) + SPP[4]*(P[5][13] + P[0][13]*SF[4] + P[2][13]*SF[3] + P[3][13]*SF[5] - P[1][13]*SPP[0] - P[13][13]*SPP[8] + P[14][13]*SPP[2] + P[15][13]*SPP[5]) - SPP[7]*(P[5][14] + P[0][14]*SF[4] + P[2][14]*SF[3] + P[3][14]*SF[5] - P[1][14]*SPP[0] - P[13][14]*SPP[8] + P[14][14]*SPP[2] + P[15][14]*SPP[5]) - SPP[1]*(P[5][15] + P[0][15]*SF[4] + P[2][15]*SF[3] + P[3][15]*SF[5] - P[1][15]*SPP[0] - P[13][15]*SPP[8] + P[14][15]*SPP[2] + P[15][15]*SPP[5]);
	nextP[6][6] = P[6][6] + P[1][6]*SF[4] - P[2][6]*SF[5] + P[3][6]*SF[3] + P[0][6]*SPP[0] + P[13][6]*SPP[4] - P[14][6]*SPP[7] - P[15][6]*SPP[1] + dvxVar*(SG[6] - 2*q0*q2)*(SG[6] - 2*q0*q2) + dvyVar*(SG[5] + 2*q0*q1)*(SG[5] + 2*q0*q1) + SF[4]*(P[6][1] + P[1][1]*SF[4] - P[2][1]*SF[5] + P[3][1]*SF[3] + P[0][1]*SPP[0] + P[13][1]*SPP[4] - P[14][1]*SPP[7] - P[15][1]*SPP[1]) - SF[5]*(P[6][2] + P[1][2]*SF[4] - P[2][2]*SF[5] + P[3][2]*SF[3] + P[0][2]*SPP[0] + P[13][2]*SPP[4] - P[14][2]*SPP[7] - P[15][2]*SPP[1]) + SF[3]*(P[6][3] + P[1][3]*SF[4] - P[2][3]*SF[5] + P[3][3]*SF[3] + P[0][3]*SPP[0] + P[13][3]*SPP[4] - P[14][3]*SPP[7] - P[15][3]*SPP[1]) + SPP[0]*(P[6][0] + P[1][0]*SF[4] - P[2][0]*SF[5] + P[3][0]*SF[3] + P[0][0]*SPP[0] + P[13][0]*SPP[4] - P[14][0]*SPP[7] - P[15][0]*SPP[1]) + SPP[4]*(P[6][13] + P[1][13]*SF[4] - P[2][13]*SF[5] + P[3][13]*SF[3] + P[0][13]*SPP[0] + P[13][13]*SPP[4] - P[14][13]*SPP[7] - P[15][13]*SPP[1]) - SPP[7]*(P[6][14] + P[1][14]*SF[4] - P[2][14]*SF[5] + P[3][14]*SF[3] + P[0][14]*SPP[0] + P[13][14]*SPP[4] - P[14][14]*SPP[7] - P[15][14]*SPP[1]) - SPP[1]*(P[6][15] + P[1][15]*SF[4] - P[2][15]*SF[5] + P[3][15]*SF[3] + P[0][15]*SPP[0] + P[13][15]*SPP[4] - P[14][15]*SPP[7] - P[15][15]*SPP[1]) + dvzVar*(SG[1] - SG[2] - SG[3] + SG[4])*(SG[1] - SG[2] - SG[3] + SG[4]);
	nextP[0][7] = P[0][7] + P[1][7]*SF[9] + P[2][7]*SF[11] + P[3][7]*SF[10] + P[10][7]*SF[14] + P[11][7]*SF[15] + P[12][7]*SPP[10] + dt*(P[0][4] + P[1][4]*SF[9] + P[2][4]*SF[11] + P[3][4]*SF[10] + P[10][4]*SF[14] + P[11][4]*SF[15] + P[12][4]*SPP[10]);
	nextP[1][7] = P[1][7] + P[0][7]*SF[8] + P[2][7]*SF[7] + P[3][7]*SF[11] - P[12][7]*SF[15] + P[11][7]*SPP[10] - (P[10][7]*q0)/2 + dt*(P[1][4] + P[0][4]*SF[8] + P[2][4]*SF[7] + P[3][4]*SF[11] - P[12][4]*SF[15] + P[11][4]*SPP[10] - (P[10][4]*q0)/2);
	nextP[2][7] = P[2][7] + P[0][7]*SF[6] + P[1][7]*SF[10] + P[3][7]*SF[8] + P[12][7]*SF[14] - P[10][7]*SPP[10] - (P[11][7]*q0)/2 + dt*(P[2][4] + P[0][4]*SF[6] + P[1][4]*SF[10] + P[3][4]*SF[8] + P[12][4]*SF[14] - P[10][4]*SPP[10] - (P[11][4]*q0)/2);
	nextP[3][7] = P[3][7] + P[0][7]*SF[7] + P[1][7]*SF[6] + P[2][7]*SF[9] + P[10][7]*SF[15] - P[11][7]*SF[14] - (P[12][7]*q0)/2 + dt*(P[3][4] + P[0][4]*SF[7] + P[1][4]*SF[6] + P[2][4]*SF[9] + P[10][4]*SF[15] - P[11][4]*SF[14] - (P[12][4]*q0)/2);
	nextP[4][7] = P[4][7] + P[0][7]*SF[5] + P[1][7]*SF[3] - P[3][7]*SF[4] + P[2][7]*SPP[0] + P[13][7]*SPP[3] + P[14][7]*SPP[6] - P[15][7]*SPP[9] + dt*(P[4][4] + P[0][4]*SF[5] + P[1][4]*SF[3] - P[3][4]*SF[4] + P[2][4]*SPP[0] + P[13][4]*SPP[3] + P[14][4]*SPP[6] - P[15][4]*SPP[9]);
	nextP[5][7] = P[5][7] + P[0][7]*SF[4] + P[2][7]*SF[3] + P[3][7]*SF[5] - P[1][7]*SPP[0] - P[13][7]*SPP[8] + P[14][7]*SPP[2] + P[15][7]*SPP[5] + dt*(P[5][4] + P[0][4]*SF[4] + P[2][4]*SF[3] + P[3][4]*SF[5] - P[1][4]*SPP[0] - P[13][4]*SPP[8] + P[14][4]*SPP[2] + P[15][4]*SPP[5]);
	nextP[6][7] = P[6][7] + P[1][7]*SF[4] - P[2][7]*SF[5] + P[3][7]*SF[3] + P[0][7]*SPP[0] + P[13][7]*SPP[4] - P[14][7]*SPP[7] - P[15][7]*SPP[1] + dt*(P[6][4] + P[1][4]*SF[4] - P[2][4]*SF[5] + P[3][4]*SF[3] + P[0][4]*SPP[0] + P[13][4]*SPP[4] - P[14][4]*SPP[7] - P[15][4]*SPP[1]);
	nextP[7][7] = P[7][7] + P[4][7]*dt + dt*(P[7][4] + P[4][4]*dt);
	nextP[0][8] = P[0][8] + P[1][8]*SF[9] + P[2][8]*SF[11] + P[3][8]*SF[10] + P[10][8]*SF[14] + P[11][8]*SF[15] + P[12][8]*SPP[10] + dt*(P[0][5] + P[1][5]*SF[9] + P[2][5]*SF[11] + P[3][5]*SF[10] + P[10][5]*SF[14] + P[11][5]*SF[15] + P[12][5]*SPP[10]);
	nextP[1][8] = P[1][8] + P[0][8]*SF[8] + P[2][8]*SF[7] + P[3][8]*SF[11] - P[12][8]*SF[15] + P[11][8]*SPP[10] - (P[10][8]*q0)/2 + dt*(P[1][5] + P[0][5]*SF[8] + P[2][5]*SF[7] + P[3][5]*SF[11] - P[12][5]*SF[15] + P[11][5]*SPP[10] - (P[10][5]*q0)/2);
	nextP[2][8] = P[2][8] + P[0][8]*SF[6] + P[1][8]*SF[10] + P[3][8]*SF[8] + P[12][8]*SF[14] - P[10][8]*SPP[10] - (P[11][8]*q0)/2 + dt*(P[2][5] + P[0][5]*SF[6] + P[1][5]*SF[10] + P[3][5]*SF[8] + P[12][5]*SF[14] - P[10][5]*SPP[10] - (P[11][5]*q0)/2);
	nextP[3][8] = P[3][8] + P[0][8]*SF[7] + P[1][8]*SF[6] + P[2][8]*SF[9] + P[10][8]*SF[15] - P[11][8]*SF[14] - (P[12][8]*q0)/2 + dt*(P[3][5] + P[0][5]*SF[7] + P[1][5]*SF[6] + P[2][5]*SF[9] + P[10][5]*SF[15] - P[11][5]*SF[14] - (P[12][5]*q0)/2);
	nextP[4][8] = P[4][8] + P[0][8]*SF[5] + P[1][8]*SF[3] - P[3][8]*SF[4] + P[2][8]*SPP[0] + P[13][8]*SPP[3] + P[14][8]*SPP[6] - P[15][8]*SPP[9] + dt*(P[4][5] + P[0][5]*SF[5] + P[1][5]*SF[3] - P[3][5]*SF[4] + P[2][5]*SPP[0] + P[13][5]*SPP[3] + P[14][5]*SPP[6] - P[15][5]*SPP[9]);
	nextP[5][8] = P[5][8] + P[0][8]*SF[4] + P[2][8]*SF[3] + P[3][8]*SF[5] - P[1][8]*SPP[0] - P[13][8]*SPP[8] + P[14][8]*SPP[2] + P[15][8]*SPP[5] + dt*(P[5][5] + P[0][5]*SF[4] + P[2][5]*SF[3] + P[3][5]*SF[5] - P[1][5]*SPP[0] - P[13][5]*SPP[8] + P[14][5]*SPP[2] + P[15][5]*SPP[5]);
	nextP[6][8] = P[6][8] + P[1][8]*SF[4] - P[2][8]*SF[5] + P[3][8]*SF[3] + P[0][8]*SPP[0] + P[13][8]*SPP[4] - P[14][8]*SPP[7] - P[15][8]*SPP[1] + dt*(P[6][5] + P[1][5]*SF[4] - P[2][5]*SF[5] + P[3][5]*SF[3] + P[0][5]*SPP[0] + P[13][5]*SPP[4] - P[14][5]*SPP[7] - P[15][5]*SPP[1]);
	nextP[7][8] = P[7][8] + P[4][8]*dt + dt*(P[7][5] + P[4][5]*dt);
	nextP[8][8] = P[8][8] + P[5][8]*dt + dt*(P[8][5] + P[5][5]*dt);
	nextP[0][9] = P[0][9] + P[1][9]*SF[9] + P[2][9]*SF[11] + P[3][9]*SF[10] + P[10][9]*SF[14] + P[11][9]*SF[15] + P[12][9]*SPP[10] + dt*(P[0][6] + P[1][6]*SF[9] + P[2][6]*SF[11] + P[3][6]*SF[10] + P[10][6]*SF[14] + P[11][6]*SF[15] + P[12][6]*SPP[10]);
	nextP[1][9] = P[1][9] + P[0][9]*SF[8] + P[2][9]*SF[7] + P[3][9]*SF[11] - P[12][9]*SF[15] + P[11][9]*SPP[10] - (P[10][9]*q0)/2 + dt*(P[1][6] + P[0][6]*SF[8] + P[2][6]*SF[7] + P[3][6]*SF[11] - P[12][6]*SF[15] + P[11][6]*SPP[10] - (P[10][6]*q0)/2);
	nextP[2][9] = P[2][9] + P[0][9]*SF[6] + P[1][9]*SF[10] + P[3][9]*SF[8] + P[12][9]*SF[14] - P[10][9]*SPP[10] - (P[11][9]*q0)/2 + dt*(P[2][6] + P[0][6]*SF[6] + P[1][6]*SF[10] + P[3][6]*SF[8] + P[12][6]*SF[14] - P[10][6]*SPP[10] - (P[11][6]*q0)/2);
	nextP[3][9] = P[3][9] + P[0][9]*SF[7] + P[1][9]*SF[6] + P[2][9]*SF[9] + P[10][9]*SF[15] - P[11][9]*SF[14] - (P[12][9]*q0)/2 + dt*(P[3][6] + P[0][6]*SF[7] + P[1][6]*SF[6] + P[2][6]*SF[9] + P[10][6]*SF[15] - P[11][6]*SF[14] - (P[12][6]*q0)/2);
	nextP[4][9] = P[4][9] + P[0][9]*SF[5] + P[1][9]*SF[3] - P[3][9]*SF[4] + P[2][9]*SPP[0] + P[13][9]*SPP[3] + P[14][9]*SPP[6] - P[15][9]*SPP[9] + dt*(P[4][6] + P[0][6]*SF[5] + P[1][6]*SF[3] - P[3][6]*SF[4] + P[2][6]*SPP[0] + P[13][6]*SPP[3] + P[14][6]*SPP[6] - P[15][6]*SPP[9]);
	nextP[5][9] = P[5][9] + P[0][9]*SF[4] + P[2][9]*SF[3] + P[3][9]*SF[5] - P[1][9]*SPP[0] - P[13][9]*SPP[8] + P[14][9]*SPP[2] + P[15][9]*SPP[5] + dt*(P[5][6] + P[0][6]*SF[4] + P[2][6]*SF[3] + P[3][6]*SF[5] - P[1][6]*SPP[0] - P[13][6]*SPP[8] + P[14][6]*SPP[2] + P[15][6]*SPP[5]);
	nextP[6][9] = P[6][9] + P[1][9]*SF[4] - P[2][9]*SF[5] + P[3][9]*SF[3] + P[0][9]*SPP[0] + P[13][9]*SPP[4] - P[14][9]*SPP[7] - P[15][9]*SPP[1] + dt*(P[6][6] + P[1][6]*SF[4] - P[2][6]*SF[5] + P[3][6]*SF[3] + P[0][6]*SPP[0] + P[13][6]*SPP[4] - P[14][6]*SPP[7] - P[15][6]*SPP[1]);
	nextP[7][9] = P[7][9] + P[4][9]*dt + dt*(P[7][6] + P[4][6]*dt);
	nextP[8][9] = P[8][9] + P[5][9]*dt + dt*(P[8][6] + P[5][6]*dt);
	nextP[9][9] = P[9][9] + P[6][9]*dt + dt*(P[9][6] + P[6][6]*dt);

	nextP[0][10] = P[0][10] + P[1][10]*SF[9] + P[2][10]*SF[11] + P[3][10]*SF[10] + P[10][10]*SF[14] + P[11][10]*SF[15] + P[12][10]*SPP[10];
	nextP[1][10] = P[1][10] + P[0][10]*SF[8] + P[2][10]*SF[7] + P[3][10]*SF[11] - P[12][10]*SF[15] + P[11][10]*SPP[10] - (P[10][10]*q0)/2;
	nextP[2][10] = P[2][10] + P[0][10]*SF[6] + P[1][10]*SF[10] + P[3][10]*SF[8] + P[12][10]*SF[14] - P[10][10]*SPP[10] - (P[11][10]*q0)/2;
	nextP[3][10] = P[3][10] + P[0][10]*SF[7] + P[1][10]*SF[6] + P[2][10]*SF[9] + P[10][10]*SF[15] - P[11][10]*SF[14] - (P[12][10]*q0)/2;
	nextP[4][10] = P[4][10] + P[0][10]*SF[5] + P[1][10]*SF[3] - P[3][10]*SF[4] + P[2][10]*SPP[0] + P[13][10]*SPP[3] + P[14][10]*SPP[6] - P[15][10]*SPP[9];
	nextP[5][10] = P[5][10] + P[0][10]*SF[4] + P[2][10]*SF[3] + P[3][10]*SF[5] - P[1][10]*SPP[0] - P[13][10]*SPP[8] + P[14][10]*SPP[2] + P[15][10]*SPP[5];
	nextP[6][10] = P[6][10] + P[1][10]*SF[4] - P[2][10]*SF[5] + P[3][10]*SF[3] + P[0][10]*SPP[0] + P[13][10]*SPP[4] - P[14][10]*SPP[7] - P[15][10]*SPP[1];
	nextP[7][10] = P[7][10] + P[4][10]*dt;
	nextP[8][10] = P[8][10] + P[5][10]*dt;
	nextP[9][10] = P[9][10] + P[6][10]*dt;
	nextP[10][10] = P[10][10];
	nextP[0][11] = P[0][11] + P[1][11]*SF[9] + P[2][11]*SF[11] + P[3][11]*SF[10] + P[10][11]*SF[14] + P[11][11]*SF[15] + P[12][11]*SPP[10];
	nextP[1][11] = P[1][11] + P[0][11]*SF[8] + P[2][11]*SF[7] + P[3][11]*SF[11] - P[12][11]*SF[15] + P[11][11]*SPP[10] - (P[10][11]*q0)/2;
	nextP[2][11] = P[2][11] + P[0][11]*SF[6] + P[1][11]*SF[10] + P[3][11]*SF[8] + P[12][11]*SF[14] - P[10][11]*SPP[10] - (P[11][11]*q0)/2;
	nextP[3][11] = P[3][11] + P[0][11]*SF[7] + P[1][11]*SF[6] + P[2][11]*SF[9] + P[10][11]*SF[15] - P[11][11]*SF[14] - (P[12][11]*q0)/2;
	nextP[4][11] = P[4][11] + P[0][11]*SF[5] + P[1][11]*SF[3] - P[3][11]*SF[4] + P[2][11]*SPP[0] + P[13][11]*SPP[3] + P[14][11]*SPP[6] - P[15][11]*SPP[9];
	nextP[5][11] = P[5][11] + P[0][11]*SF[4] + P[2][11]*SF[3] + P[3][11]*SF[5] - P[1][11]*SPP[0] - P[13][11]*SPP[8] + P[14][11]*SPP[2] + P[15][11]*SPP[5];
	nextP[6][11] = P[6][11] + P[1][11]*SF[4] - P[2][11]*SF[5] + P[3][11]*SF[3] + P[0][11]*SPP[0] + P[13][11]*SPP[4] - P[14][11]*SPP[7] - P[15][11]*SPP[1];
	nextP[7][11] = P[7][11] + P[4][11]*dt;
	nextP[8][11] = P[8][11] + P[5][11]*dt;
	nextP[9][11] = P[9][11] + P[6][11]*dt;
	nextP[10][11] = P[10][11];
	nextP[11][11] = P[11][11];
	nextP[0][12] = P[0][12] + P[1][12]*SF[9] + P[2][12]*SF[11] + P[3][12]*SF[10] + P[10][12]*SF[14] + P[11][12]*SF[15] + P[12][12]*SPP[10];
	nextP[1][12] = P[1][12] + P[0][12]*SF[8] + P[2][12]*SF[7] + P[3][12]*SF[11] - P[12][12]*SF[15] + P[11][12]*SPP[10] - (P[10][12]*q0)/2;
	nextP[2][12] = P[2][12] + P[0][12]*SF[6] + P[1][12]*SF[10] + P[3][12]*SF[8] + P[12][12]*SF[14] - P[10][12]*SPP[10] - (P[11][12]*q0)/2;
	nextP[3][12] = P[3][12] + P[0][12]*SF[7] + P[1][12]*SF[6] + P[2][12]*SF[9] + P[10][12]*SF[15] - P[11][12]*SF[14] - (P[12][12]*q0)/2;
	nextP[4][12] = P[4][12] + P[0][12]*SF[5] + P[1][12]*SF[3] - P[3][12]*SF[4] + P[2][12]*SPP[0] + P[13][12]*SPP[3] + P[14][12]*SPP[6] - P[15][12]*SPP[9];
	nextP[5][12] = P[5][12] + P[0][12]*SF[4] + P[2][12]*SF[3] + P[3][12]*SF[5] - P[1][12]*SPP[0] - P[13][12]*SPP[8] + P[14][12]*SPP[2] + P[15][12]*SPP[5];
	nextP[6][12] = P[6][12] + P[1][12]*SF[4] - P[2][12]*SF[5] + P[3][12]*SF[3] + P[0][12]*SPP[0] + P[13][12]*SPP[4] - P[14][12]*SPP[7] - P[15][12]*SPP[1];
	nextP[7][12] = P[7][12] + P[4][12]*dt;
	nextP[8][12] = P[8][12] + P[5][12]*dt;
	nextP[9][12] = P[9][12] + P[6][12]*dt;
	nextP[10][12] = P[10][12];
	nextP[11][12] = P[11][12];
	nextP[12][12] = P[12][12];

	nextP[0][13] = P[0][13] + P[1][13]*SF[9] + P[2][13]*SF[11] + P[3][13]*SF[10] + P[10][13]*SF[14] + P[11][13]*SF[15] + P[12][13]*SPP[10];
	nextP[1][13] = P[1][13] + P[0][13]*SF[8] + P[2][13]*SF[7] + P[3][13]*SF[11] - P[12][13]*SF[15] + P[11][13]*SPP[10] - (P[10][13]*q0)/2;
	nextP[2][13] = P[2][13] + P[0][13]*SF[6] + P[1][13]*SF[10] + P[3][13]*SF[8] + P[12][13]*SF[14] - P[10][13]*SPP[10] - (P[11][13]*q0)/2;
	nextP[3][13] = P[3][13] + P[0][13]*SF[7] + P[1][13]*SF[6] + P[2][13]*SF[9] + P[10][13]*SF[15] - P[11][13]*SF[14] - (P[12][13]*q0)/2;
	nextP[4][13] = P[4][13] + P[0][13]*SF[5] + P[1][13]*SF[3] - P[3][13]*SF[4] + P[2][13]*SPP[0] + P[13][13]*SPP[3] + P[14][13]*SPP[6] - P[15][13]*SPP[9];
	nextP[5][13] = P[5][13] + P[0][13]*SF[4] + P[2][13]*SF[3] + P[3][13]*SF[5] - P[1][13]*SPP[0] - P[13][13]*SPP[8] + P[14][13]*SPP[2] + P[15][13]*SPP[5];
	nextP[6][13] = P[6][13] + P[1][13]*SF[4] - P[2][13]*SF[5] + P[3][13]*SF[3] + P[0][13]*SPP[0] + P[13][13]*SPP[4] - P[14][13]*SPP[7] - P[15][13]*SPP[1];
	nextP[7][13] = P[7][13] + P[4][13]*dt;
	nextP[8][13] = P[8][13] + P[5][13]*dt;
	nextP[9][13] = P[9][13] + P[6][13]*dt;
	nextP[10][13] = P[10][13];
	nextP[11][13] = P[11][13];
	nextP[12][13] = P[12][13];
	nextP[13][13] = P[13][13];
	nextP[0][14] = P[0][14] + P[1][14]*SF[9] + P[2][14]*SF[11] + P[3][14]*SF[10] + P[10][14]*SF[14] + P[11][14]*SF[15] + P[12][14]*SPP[10];
	nextP[1][14] = P[1][14] + P[0][14]*SF[8] + P[2][14]*SF[7] + P[3][14]*SF[11] - P[12][14]*SF[15] + P[11][14]*SPP[10] - (P[10][14]*q0)/2;
	nextP[2][14] = P[2][14] + P[0][14]*SF[6] + P[1][14]*SF[10] + P[3][14]*SF[8] + P[12][14]*SF[14] - P[10][14]*SPP[10] - (P[11][14]*q0)/2;
	nextP[3][14] = P[3][14] + P[0][14]*SF[7] + P[1][14]*SF[6] + P[2][14]*SF[9] + P[10][14]*SF[15] - P[11][14]*SF[14] - (P[12][14]*q0)/2;
	nextP[4][14] = P[4][14] + P[0][14]*SF[5] + P[1][14]*SF[3] - P[3][14]*SF[4] + P[2][14]*SPP[0] + P[13][14]*SPP[3] + P[14][14]*SPP[6] - P[15][14]*SPP[9];
	nextP[5][14] = P[5][14] + P[0][14]*SF[4] + P[2][14]*SF[3] + P[3][14]*SF[5] - P[1][14]*SPP[0] - P[13][14]*SPP[8] + P[14][14]*SPP[2] + P[15][14]*SPP[5];
	nextP[6][14] = P[6][14] + P[1][14]*SF[4] - P[2][14]*SF[5] + P[3][14]*SF[3] + P[0][14]*SPP[0] + P[13][14]*SPP[4] - P[14][14]*SPP[7] - P[15][14]*SPP[1];
	nextP[7][14] = P[7][14] + P[4][14]*dt;
	nextP[8][14] = P[8][14] + P[5][14]*dt;
	nextP[9][14] = P[9][14] + P[6][14]*dt;
	nextP[10][14] = P[10][14];
	nextP[11][14] = P[11][14];
	nextP[12][14] = P[12][14];
	nextP[13][14] = P[13][14];
	nextP[14][14] = P[14][14];
	nextP[0][15] = P[0][15] + P[1][15]*SF[9] + P[2][15]*SF[11] + P[3][15]*SF[10] + P[10][15]*SF[14] + P[11][15]*SF[15] + P[12][15]*SPP[10];
	nextP[1][15] = P[1][15] + P[0][15]*SF[8] + P[2][15]*SF[7] + P[3][15]*SF[11] - P[12][15]*SF[15] + P[11][15]*SPP[10] - (P[10][15]*q0)/2;
	nextP[2][15] = P[2][15] + P[0][15]*SF[6] + P[1][15]*SF[10] + P[3][15]*SF[8] + P[12][15]*SF[14] - P[10][15]*SPP[10] - (P[11][15]*q0)/2;
	nextP[3][15] = P[3][15] + P[0][15]*SF[7] + P[1][15]*SF[6] + P[2][15]*SF[9] + P[10][15]*SF[15] - P[11][15]*SF[14] - (P[12][15]*q0)/2;
	nextP[4][15] = P[4][15] + P[0][15]*SF[5] + P[1][15]*SF[3] - P[3][15]*SF[4] + P[2][15]*SPP[0] + P[13][15]*SPP[3] + P[14][15]*SPP[6] - P[15][15]*SPP[9];
	nextP[5][15] = P[5][15] + P[0][15]*SF[4] + P[2][15]*SF[3] + P[3][15]*SF[5] - P[1][15]*SPP[0] - P[13][15]*SPP[8] + P[14][15]*SPP[2] + P[15][15]*SPP[5];
	nextP[6][15] = P[6][15] + P[1][15]*SF[4] - P[2][15]*SF[5] + P[3][15]*SF[3] + P[0][15]*SPP[0] + P[13][15]*SPP[4] - P[14][15]*SPP[7] - P[15][15]*SPP[1];
	nextP[7][15] = P[7][15] + P[4][15]*dt;
	nextP[8][15] = P[8][15] + P[5][15]*dt;
	nextP[9][15] = P[9][15] + P[6][15]*dt;
	nextP[10][15] = P[10][15];
	nextP[11][15] = P[11][15];
	nextP[12][15] = P[12][15];
	nextP[13][15] = P[13][15];
	nextP[14][15] = P[14][15];
	nextP[15][15] = P[15][15];
	if(ele->stateIndexLim>15){
		nextP[0][16] = P[0][16] + P[1][16]*SF[9] + P[2][16]*SF[11] + P[3][16]*SF[10] + P[10][16]*SF[14] + P[11][16]*SF[15] + P[12][16]*SPP[10];
		nextP[1][16] = P[1][16] + P[0][16]*SF[8] + P[2][16]*SF[7] + P[3][16]*SF[11] - P[12][16]*SF[15] + P[11][16]*SPP[10] - (P[10][16]*q0)/2;
		nextP[2][16] = P[2][16] + P[0][16]*SF[6] + P[1][16]*SF[10] + P[3][16]*SF[8] + P[12][16]*SF[14] - P[10][16]*SPP[10] - (P[11][16]*q0)/2;
		nextP[3][16] = P[3][16] + P[0][16]*SF[7] + P[1][16]*SF[6] + P[2][16]*SF[9] + P[10][16]*SF[15] - P[11][16]*SF[14] - (P[12][16]*q0)/2;
		nextP[4][16] = P[4][16] + P[0][16]*SF[5] + P[1][16]*SF[3] - P[3][16]*SF[4] + P[2][16]*SPP[0] + P[13][16]*SPP[3] + P[14][16]*SPP[6] - P[15][16]*SPP[9];
		nextP[5][16] = P[5][16] + P[0][16]*SF[4] + P[2][16]*SF[3] + P[3][16]*SF[5] - P[1][16]*SPP[0] - P[13][16]*SPP[8] + P[14][16]*SPP[2] + P[15][16]*SPP[5];
		nextP[6][16] = P[6][16] + P[1][16]*SF[4] - P[2][16]*SF[5] + P[3][16]*SF[3] + P[0][16]*SPP[0] + P[13][16]*SPP[4] - P[14][16]*SPP[7] - P[15][16]*SPP[1];
		nextP[7][16] = P[7][16] + P[4][16]*dt;
		nextP[8][16] = P[8][16] + P[5][16]*dt;
		nextP[9][16] = P[9][16] + P[6][16]*dt;
		nextP[10][16] = P[10][16];
		nextP[11][16] = P[11][16];
		nextP[12][16] = P[12][16];
		nextP[13][16] = P[13][16];
		nextP[14][16] = P[14][16];
		nextP[15][16] = P[15][16];
		nextP[16][16] = P[16][16];
		nextP[0][17] = P[0][17] + P[1][17]*SF[9] + P[2][17]*SF[11] + P[3][17]*SF[10] + P[10][17]*SF[14] + P[11][17]*SF[15] + P[12][17]*SPP[10];
		nextP[1][17] = P[1][17] + P[0][17]*SF[8] + P[2][17]*SF[7] + P[3][17]*SF[11] - P[12][17]*SF[15] + P[11][17]*SPP[10] - (P[10][17]*q0)/2;
		nextP[2][17] = P[2][17] + P[0][17]*SF[6] + P[1][17]*SF[10] + P[3][17]*SF[8] + P[12][17]*SF[14] - P[10][17]*SPP[10] - (P[11][17]*q0)/2;
		nextP[3][17] = P[3][17] + P[0][17]*SF[7] + P[1][17]*SF[6] + P[2][17]*SF[9] + P[10][17]*SF[15] - P[11][17]*SF[14] - (P[12][17]*q0)/2;
		nextP[4][17] = P[4][17] + P[0][17]*SF[5] + P[1][17]*SF[3] - P[3][17]*SF[4] + P[2][17]*SPP[0] + P[13][17]*SPP[3] + P[14][17]*SPP[6] - P[15][17]*SPP[9];
		nextP[5][17] = P[5][17] + P[0][17]*SF[4] + P[2][17]*SF[3] + P[3][17]*SF[5] - P[1][17]*SPP[0] - P[13][17]*SPP[8] + P[14][17]*SPP[2] + P[15][17]*SPP[5];
		nextP[6][17] = P[6][17] + P[1][17]*SF[4] - P[2][17]*SF[5] + P[3][17]*SF[3] + P[0][17]*SPP[0] + P[13][17]*SPP[4] - P[14][17]*SPP[7] - P[15][17]*SPP[1];
		nextP[7][17] = P[7][17] + P[4][17]*dt;
		nextP[8][17] = P[8][17] + P[5][17]*dt;
		nextP[9][17] = P[9][17] + P[6][17]*dt;
		nextP[10][17] = P[10][17];
		nextP[11][17] = P[11][17];
		nextP[12][17] = P[12][17];
		nextP[13][17] = P[13][17];
		nextP[14][17] = P[14][17];
		nextP[15][17] = P[15][17];
		nextP[16][17] = P[16][17];
		nextP[17][17] = P[17][17];
		nextP[0][18] = P[0][18] + P[1][18]*SF[9] + P[2][18]*SF[11] + P[3][18]*SF[10] + P[10][18]*SF[14] + P[11][18]*SF[15] + P[12][18]*SPP[10];
		nextP[1][18] = P[1][18] + P[0][18]*SF[8] + P[2][18]*SF[7] + P[3][18]*SF[11] - P[12][18]*SF[15] + P[11][18]*SPP[10] - (P[10][18]*q0)/2;
		nextP[2][18] = P[2][18] + P[0][18]*SF[6] + P[1][18]*SF[10] + P[3][18]*SF[8] + P[12][18]*SF[14] - P[10][18]*SPP[10] - (P[11][18]*q0)/2;
		nextP[3][18] = P[3][18] + P[0][18]*SF[7] + P[1][18]*SF[6] + P[2][18]*SF[9] + P[10][18]*SF[15] - P[11][18]*SF[14] - (P[12][18]*q0)/2;
		nextP[4][18] = P[4][18] + P[0][18]*SF[5] + P[1][18]*SF[3] - P[3][18]*SF[4] + P[2][18]*SPP[0] + P[13][18]*SPP[3] + P[14][18]*SPP[6] - P[15][18]*SPP[9];
		nextP[5][18] = P[5][18] + P[0][18]*SF[4] + P[2][18]*SF[3] + P[3][18]*SF[5] - P[1][18]*SPP[0] - P[13][18]*SPP[8] + P[14][18]*SPP[2] + P[15][18]*SPP[5];
		nextP[6][18] = P[6][18] + P[1][18]*SF[4] - P[2][18]*SF[5] + P[3][18]*SF[3] + P[0][18]*SPP[0] + P[13][18]*SPP[4] - P[14][18]*SPP[7] - P[15][18]*SPP[1];
		nextP[7][18] = P[7][18] + P[4][18]*dt;
		nextP[8][18] = P[8][18] + P[5][18]*dt;
		nextP[9][18] = P[9][18] + P[6][18]*dt;
		nextP[10][18] = P[10][18];
		nextP[11][18] = P[11][18];
		nextP[12][18] = P[12][18];
		nextP[13][18] = P[13][18];
		nextP[14][18] = P[14][18];
		nextP[15][18] = P[15][18];
		nextP[16][18] = P[16][18];
		nextP[17][18] = P[17][18];
		nextP[18][18] = P[18][18];
		nextP[0][19] = P[0][19] + P[1][19]*SF[9] + P[2][19]*SF[11] + P[3][19]*SF[10] + P[10][19]*SF[14] + P[11][19]*SF[15] + P[12][19]*SPP[10];
		nextP[1][19] = P[1][19] + P[0][19]*SF[8] + P[2][19]*SF[7] + P[3][19]*SF[11] - P[12][19]*SF[15] + P[11][19]*SPP[10] - (P[10][19]*q0)/2;
		nextP[2][19] = P[2][19] + P[0][19]*SF[6] + P[1][19]*SF[10] + P[3][19]*SF[8] + P[12][19]*SF[14] - P[10][19]*SPP[10] - (P[11][19]*q0)/2;
		nextP[3][19] = P[3][19] + P[0][19]*SF[7] + P[1][19]*SF[6] + P[2][19]*SF[9] + P[10][19]*SF[15] - P[11][19]*SF[14] - (P[12][19]*q0)/2;
		nextP[4][19] = P[4][19] + P[0][19]*SF[5] + P[1][19]*SF[3] - P[3][19]*SF[4] + P[2][19]*SPP[0] + P[13][19]*SPP[3] + P[14][19]*SPP[6] - P[15][19]*SPP[9];
		nextP[5][19] = P[5][19] + P[0][19]*SF[4] + P[2][19]*SF[3] + P[3][19]*SF[5] - P[1][19]*SPP[0] - P[13][19]*SPP[8] + P[14][19]*SPP[2] + P[15][19]*SPP[5];
		nextP[6][19] = P[6][19] + P[1][19]*SF[4] - P[2][19]*SF[5] + P[3][19]*SF[3] + P[0][19]*SPP[0] + P[13][19]*SPP[4] - P[14][19]*SPP[7] - P[15][19]*SPP[1];
		nextP[7][19] = P[7][19] + P[4][19]*dt;
		nextP[8][19] = P[8][19] + P[5][19]*dt;
		nextP[9][19] = P[9][19] + P[6][19]*dt;
		nextP[10][19] = P[10][19];
		nextP[11][19] = P[11][19];
		nextP[12][19] = P[12][19];
		nextP[13][19] = P[13][19];
		nextP[14][19] = P[14][19];
		nextP[15][19] = P[15][19];
		nextP[16][19] = P[16][19];
		nextP[17][19] = P[17][19];
		nextP[18][19] = P[18][19];
		nextP[19][19] = P[19][19];
		nextP[0][20] = P[0][20] + P[1][20]*SF[9] + P[2][20]*SF[11] + P[3][20]*SF[10] + P[10][20]*SF[14] + P[11][20]*SF[15] + P[12][20]*SPP[10];
		nextP[1][20] = P[1][20] + P[0][20]*SF[8] + P[2][20]*SF[7] + P[3][20]*SF[11] - P[12][20]*SF[15] + P[11][20]*SPP[10] - (P[10][20]*q0)/2;
		nextP[2][20] = P[2][20] + P[0][20]*SF[6] + P[1][20]*SF[10] + P[3][20]*SF[8] + P[12][20]*SF[14] - P[10][20]*SPP[10] - (P[11][20]*q0)/2;
		nextP[3][20] = P[3][20] + P[0][20]*SF[7] + P[1][20]*SF[6] + P[2][20]*SF[9] + P[10][20]*SF[15] - P[11][20]*SF[14] - (P[12][20]*q0)/2;
		nextP[4][20] = P[4][20] + P[0][20]*SF[5] + P[1][20]*SF[3] - P[3][20]*SF[4] + P[2][20]*SPP[0] + P[13][20]*SPP[3] + P[14][20]*SPP[6] - P[15][20]*SPP[9];
		nextP[5][20] = P[5][20] + P[0][20]*SF[4] + P[2][20]*SF[3] + P[3][20]*SF[5] - P[1][20]*SPP[0] - P[13][20]*SPP[8] + P[14][20]*SPP[2] + P[15][20]*SPP[5];
		nextP[6][20] = P[6][20] + P[1][20]*SF[4] - P[2][20]*SF[5] + P[3][20]*SF[3] + P[0][20]*SPP[0] + P[13][20]*SPP[4] - P[14][20]*SPP[7] - P[15][20]*SPP[1];
		nextP[7][20] = P[7][20] + P[4][20]*dt;
		nextP[8][20] = P[8][20] + P[5][20]*dt;
		nextP[9][20] = P[9][20] + P[6][20]*dt;
		nextP[10][20] = P[10][20];
		nextP[11][20] = P[11][20];
		nextP[12][20] = P[12][20];
		nextP[13][20] = P[13][20];
		nextP[14][20] = P[14][20];
		nextP[15][20] = P[15][20];
		nextP[16][20] = P[16][20];
		nextP[17][20] = P[17][20];
		nextP[18][20] = P[18][20];
		nextP[19][20] = P[19][20];
		nextP[20][20] = P[20][20];
		nextP[0][21] = P[0][21] + P[1][21]*SF[9] + P[2][21]*SF[11] + P[3][21]*SF[10] + P[10][21]*SF[14] + P[11][21]*SF[15] + P[12][21]*SPP[10];
		nextP[1][21] = P[1][21] + P[0][21]*SF[8] + P[2][21]*SF[7] + P[3][21]*SF[11] - P[12][21]*SF[15] + P[11][21]*SPP[10] - (P[10][21]*q0)/2;
		nextP[2][21] = P[2][21] + P[0][21]*SF[6] + P[1][21]*SF[10] + P[3][21]*SF[8] + P[12][21]*SF[14] - P[10][21]*SPP[10] - (P[11][21]*q0)/2;
		nextP[3][21] = P[3][21] + P[0][21]*SF[7] + P[1][21]*SF[6] + P[2][21]*SF[9] + P[10][21]*SF[15] - P[11][21]*SF[14] - (P[12][21]*q0)/2;
		nextP[4][21] = P[4][21] + P[0][21]*SF[5] + P[1][21]*SF[3] - P[3][21]*SF[4] + P[2][21]*SPP[0] + P[13][21]*SPP[3] + P[14][21]*SPP[6] - P[15][21]*SPP[9];
		nextP[5][21] = P[5][21] + P[0][21]*SF[4] + P[2][21]*SF[3] + P[3][21]*SF[5] - P[1][21]*SPP[0] - P[13][21]*SPP[8] + P[14][21]*SPP[2] + P[15][21]*SPP[5];
		nextP[6][21] = P[6][21] + P[1][21]*SF[4] - P[2][21]*SF[5] + P[3][21]*SF[3] + P[0][21]*SPP[0] + P[13][21]*SPP[4] - P[14][21]*SPP[7] - P[15][21]*SPP[1];
		nextP[7][21] = P[7][21] + P[4][21]*dt;
		nextP[8][21] = P[8][21] + P[5][21]*dt;
		nextP[9][21] = P[9][21] + P[6][21]*dt;
		nextP[10][21] = P[10][21];
		nextP[11][21] = P[11][21];
		nextP[12][21] = P[12][21];
		nextP[13][21] = P[13][21];
		nextP[14][21] = P[14][21];
		nextP[15][21] = P[15][21];
		nextP[16][21] = P[16][21];
		nextP[17][21] = P[17][21];
		nextP[18][21] = P[18][21];
		nextP[19][21] = P[19][21];
		nextP[20][21] = P[20][21];
		nextP[21][21] = P[21][21];
	}
	for (u8 i=10; i<=ele->stateIndexLim; i++) {
		nextP[i][i] = nextP[i][i] + processNoiseVariance[i-10];
	}
	
	for (u8 row = 0; row <= ele->stateIndexLim; row++) {

		P[row][row] = nextP[row][row];

		for (u8 column = 0 ; column < row; column++) {
				P[row][column] = P[column][row] = nextP[column][row];
		}
	}
}

void zeroCols(float covMat[22][22], u8 first, u8 last)
{
	u8 row;
  for (row=0; row<=21; row++)
  {
    memset(&covMat[row][first], 0, sizeof(covMat[0][0])*(1+last-first));
  }
}

void zeroRows(float covMat[22][22], u8 first, u8 last)
{
  u8 row;
  for (row=first; row<=last; row++)
  {
    memset(&covMat[row][0], 0, sizeof(covMat[0][0])*22);
  }
}

void zeroAttCovOnly()
{
  float varTemp[4];
  for (u8 index=0; index<=3; index++) {
    varTemp[index] = P[index][index];
  }
  zeroCols(P,0,3);
  zeroRows(P,0,3);
  for (u8 index=0; index<=3; index++) {
    P[index][index] = varTemp[index];
  }
}

void alignMagStateDeclination(sEKF *ele)
{
	if(ele->magFieldLearned) {
    return;
  }
	
//	float initMagNED[3];
//	for(u8 i=0;i<3;i++){
//		initMagNED[i] = ele->state[i+16];
//	}
//	float magLengthNE = sqrt(sq(initMagNED[0])+sq(initMagNED[1]));
//	ele->state[16] = magLengthNE * cosf(gps->yaw_declination);
//	ele->state[17] = magLengthNE * sinf(gps->yaw_declination);
	if (!ele->inhibitMagStates) {
		
		float var_16 = P[16][16];
		float var_17 = P[17][17];
		zeroRows(P,16,17);
		zeroCols(P,16,17);
		P[16][16] = var_16;
		P[17][17] = var_17;
		FuseDeclination(ele);
	}
}

void calcQuatAndFieldStates(float roll,float pitch,float init_Quat[4],sEKF *ele)
{
	float yaw;//航向角
	float Tbn_temp[3][3];//旋转矩阵
  float initMagNED[3];//NED系下三轴磁场
	
  rot_from_euler(roll,pitch,0.0f,Tbn_temp);
	
	readMAGData(ele);
	
	MatrixMultiVector(Tbn_temp,ele->magDataDelayed.mag,initMagNED);
	
	float magHeading = atan2f(initMagNED[1], initMagNED[0]);
	yaw = gps->yaw_declination*D2R - magHeading;
	if(yaw>PI) yaw-=2*PI;
	else if(yaw<-PI) yaw+=2*PI;
	
	float init_euler[3];
	init_euler[0] = roll;
	init_euler[1] = pitch;
	init_euler[2] = yaw;
	
	euler2quaternion(init_euler,init_Quat);//得到新的四元数
	zeroAttCovOnly();//P阵姿态部分协方差置0，方差不置零
	if(!ele->magFieldLearned){
		rotation_matrix(init_Quat,Tbn_temp);
		float mag_earth[3];
		MatrixMultiVector(Tbn_temp,ele->magDataDelayed.mag,mag_earth);
		for(u8 i = 0;i<3;i++){
			ele->state[i+16] = mag_earth[i];
		}
		alignMagStateDeclination(ele);
		zeroRows(P,18,21);
		zeroCols(P,18,21);
		P[18][18] = sq(MAG_NED_R);
		P[19][19] = P[18][18];
		P[20][20] = P[18][18];
		P[21][21] = P[18][18];
	}	
	if (ele->in_flight) {
    ele->finalInflightMagInit = true;
  }
}

void calcRotVecVariances(sEKF *ele,float rotVarVec[3])
{
	float q0 = ele->state[0];
	float q1 = ele->state[1];
	float q2 = ele->state[2];
	float q3 = ele->state[3];
	if (q0 < 0) {
			q0 = -q0;
			q1 = -q1;
			q2 = -q2;
			q3 = -q3;
	}
	float t2 = q0*q0;
	float t3 = acosf(q0);
	float t4 = -t2+1.0f;
	float t5 = t2-1.0f;
	if ((t4 > 1e-9f) && (t5 < -1e-9f)) {
			float t6 = 1.0f/t5;
			float t7 = q1*t6*2.0f;
			float t8 = 1.0f/powf(t4,1.5f);
			float t9 = q0*q1*t3*t8*2.0f;
			float t10 = t7+t9;
			float t11 = 1.0f/sqrtf(t4);
			float t12 = q2*t6*2.0f;
			float t13 = q0*q2*t3*t8*2.0f;
			float t14 = t12+t13;
			float t15 = q3*t6*2.0f;
			float t16 = q0*q3*t3*t8*2.0f;
			float t17 = t15+t16;
			rotVarVec[0] = t10*(P[0][0]*t10+P[1][0]*t3*t11*2.0f)+t3*t11*(P[0][1]*t10+P[1][1]*t3*t11*2.0f)*2.0f;
			rotVarVec[1]= t14*(P[0][0]*t14+P[2][0]*t3*t11*2.0f)+t3*t11*(P[0][2]*t14+P[2][2]*t3*t11*2.0f)*2.0f;
			rotVarVec[2] = t17*(P[0][0]*t17+P[3][0]*t3*t11*2.0f)+t3*t11*(P[0][3]*t17+P[3][3]*t3*t11*2.0f)*2.0f;
	} else {
			rotVarVec[0] = 4.0f * P[1][1];
			rotVarVec[1] = 4.0f * P[2][2];
			rotVarVec[2] = 4.0f * P[3][3];
	}
}
void initialiseQuatCovariances(sEKF *ele,float rotVarVec[3])
{
	float q0 = ele->state[0];
	float q1 = ele->state[1];
	float q2 = ele->state[2];
	float q3 = ele->state[3];
	if (q0 < 0) {
			q0 = -q0;
			q1 = -q1;
			q2 = -q2;
			q3 = -q3;
	}
	float delta = 2.0f*acosf(q0);
	float scaler;
	if (fabsf(delta) > 1e-6f) {
			scaler = (delta/sinf(delta*0.5f));
	} else {
			scaler = 2.0f;
	}
	float rotX = scaler*q1;
	float rotY = scaler*q2;
	float rotZ = scaler*q3;

	// autocode generated using matlab symbolic toolbox
	float t2 = rotX*rotX;
	float t4 = rotY*rotY;
	float t5 = rotZ*rotZ;
	float t6 = t2+t4+t5;
	if (t6 > 1e-9f) {
			float t7 = sqrtf(t6);
			float t8 = t7*0.5f;
			float t3 = sinf(t8);
			float t9 = t3*t3;
			float t10 = 1.0f/t6;
			float t11 = 1.0f/sqrtf(t6);
			float t12 = cosf(t8);
			float t13 = 1.0f/powf(t6,1.5f);
			float t14 = t3*t11;
			float t15 = rotX*rotY*t3*t13;
			float t16 = rotX*rotZ*t3*t13;
			float t17 = rotY*rotZ*t3*t13;
			float t18 = t2*t10*t12*0.5f;
			float t27 = t2*t3*t13;
			float t19 = t14+t18-t27;
			float t23 = rotX*rotY*t10*t12*0.5f;
			float t28 = t15-t23;
			float t20 = rotY*rotVarVec[1]*t3*t11*t28*0.5f;
			float t25 = rotX*rotZ*t10*t12*0.5f;
			float t31 = t16-t25;
			float t21 = rotZ*rotVarVec[2]*t3*t11*t31*0.5f;
			float t22 = t20+t21-rotX*rotVarVec[0]*t3*t11*t19*0.5f;
			float t24 = t15-t23;
			float t26 = t16-t25;
			float t29 = t4*t10*t12*0.5f;
			float t34 = t3*t4*t13;
			float t30 = t14+t29-t34;
			float t32 = t5*t10*t12*0.5f;
			float t40 = t3*t5*t13;
			float t33 = t14+t32-t40;
			float t36 = rotY*rotZ*t10*t12*0.5f;
			float t39 = t17-t36;
			float t35 = rotZ*rotVarVec[2]*t3*t11*t39*0.5f;
			float t37 = t15-t23;
			float t38 = t17-t36;
			float t41 = rotVarVec[0]*(t15-t23)*(t16-t25);
			float t42 = t41-rotVarVec[1]*t30*t39-rotVarVec[2]*t33*t39;
			float t43 = t16-t25;
			float t44 = t17-t36;

			// zero all the quaternion covariances
			zeroRows(P,0,3);
			zeroCols(P,0,3);

			// Update the quaternion internal covariances using auto-code generated using matlab symbolic toolbox
			P[0][0] = rotVarVec[0]*t2*t9*t10*0.25f+rotVarVec[1]*t4*t9*t10*0.25f+rotVarVec[2]*t5*t9*t10*0.25f;
			P[0][1] = t22;
			P[0][2] = t35+rotX*rotVarVec[0]*t3*t11*(t15-rotX*rotY*t10*t12*0.5f)*0.5f-rotY*rotVarVec[1]*t3*t11*t30*0.5f;
			P[0][3] = rotX*rotVarVec[0]*t3*t11*(t16-rotX*rotZ*t10*t12*0.5f)*0.5f+rotY*rotVarVec[1]*t3*t11*(t17-rotY*rotZ*t10*t12*0.5f)*0.5f-rotZ*rotVarVec[2]*t3*t11*t33*0.5f;
			P[1][0] = t22;
			P[1][1] = rotVarVec[0]*(t19*t19)+rotVarVec[1]*(t24*t24)+rotVarVec[2]*(t26*t26);
			P[1][2] = rotVarVec[2]*(t16-t25)*(t17-rotY*rotZ*t10*t12*0.5f)-rotVarVec[0]*t19*t28-rotVarVec[1]*t28*t30;
			P[1][3] = rotVarVec[1]*(t15-t23)*(t17-rotY*rotZ*t10*t12*0.5f)-rotVarVec[0]*t19*t31-rotVarVec[2]*t31*t33;
			P[2][0] = t35-rotY*rotVarVec[1]*t3*t11*t30*0.5f+rotX*rotVarVec[0]*t3*t11*(t15-t23)*0.5f;
			P[2][1] = rotVarVec[2]*(t16-t25)*(t17-t36)-rotVarVec[0]*t19*t28-rotVarVec[1]*t28*t30;
			P[2][2] = rotVarVec[1]*(t30*t30)+rotVarVec[0]*(t37*t37)+rotVarVec[2]*(t38*t38);
			P[2][3] = t42;
			P[3][0] = rotZ*rotVarVec[2]*t3*t11*t33*(-0.5f)+rotX*rotVarVec[0]*t3*t11*(t16-t25)*0.5f+rotY*rotVarVec[1]*t3*t11*(t17-t36)*0.5f;
			P[3][1] = rotVarVec[1]*(t15-t23)*(t17-t36)-rotVarVec[0]*t19*t31-rotVarVec[2]*t31*t33;
			P[3][2] = t42;
			P[3][3] = rotVarVec[2]*(t33*t33)+rotVarVec[0]*(t43*t43)+rotVarVec[1]*(t44*t44);

	} else {
			// the equations are badly conditioned so use a small angle approximation
			P[0][0] = 0.0f;
			P[0][1] = 0.0f;
			P[0][2] = 0.0f;
			P[0][3] = 0.0f;
			P[1][0] = 0.0f;
			P[1][1] = 0.25f*rotVarVec[0];
			P[1][2] = 0.0f;
			P[1][3] = 0.0f;
			P[2][0] = 0.0f;
			P[2][1] = 0.0f;
			P[2][2] = 0.25f*rotVarVec[1];
			P[2][3] = 0.0f;
			P[3][0] = 0.0f;
			P[3][1] = 0.0f;
			P[3][2] = 0.0f;
			P[3][3] = 0.25f*rotVarVec[2];
	}
}

void StoreQuatRotate(sEKF *ele,float deltaQuat[4])
{
    QuaternionMulti(ele->outputDataNew.quat,deltaQuat);
	
    for (uint8_t i=0; i<IMU_BUFFER_SIZE; i++) {
      QuaternionMulti(storeOUTPUT.output_data[i].quat,deltaQuat);
    }
		
    QuaternionMulti(ele->outputDataDelayed.quat,deltaQuat);
}

void controlMagYawReset(sEKF *ele)
{
	float deltaRotVecTemp[3];
	
	bool flightResetAllowed = false;//进行空中磁力计校准标志位
	
	if(!ele->finalInflightYawInit){//如果空中航向角未对准
		float deltaQuatTemp[4];//计算当前姿态四元数变化
		float cur_Quat[4];//当前姿态四元数
		for(u8 i = 0;i<4;i++){
			cur_Quat[i] = ele->state[i];
		}
		QuaternionDivision(cur_Quat,ele->prevQuatMagReset,deltaQuatTemp);
		for(u8 i = 0;i<4;i++){
			ele->prevQuatMagReset[i] = cur_Quat[i];
		}
		to_axis_angle(deltaQuatTemp,deltaRotVecTemp);//误差四元数转换成轴角
		bool angRateOK = sqrt(sq(deltaRotVecTemp[0])+sq(deltaRotVecTemp[1])+sq(deltaRotVecTemp[1]))< 0.1745f;//如果变化小于5°且在高空中
		flightResetAllowed = angRateOK && !ele->on_Ground;//则允许空中对准
	}
	
	bool finalResetRequest = false;
	if(flightResetAllowed){
		finalResetRequest = (ele->state[9]  - ele->posDownAtTakeoff) < -EKF_MAG_FINAL_RESET_ALT;//起飞高度达到要求，则进行空中磁力计对准
	}		
	
	if(finalResetRequest){
		float eulerAngles[3];
		float temp_Quat[4];
		for(u8 i = 0;i<4;i++){
			temp_Quat[i] = ele->state[i];
		}
    Quaternion2Euler(temp_Quat,eulerAngles);//计算当前欧拉角
		float newQuat[4];
		calcQuatAndFieldStates(eulerAngles[0], eulerAngles[1],newQuat,ele);//重新对准欧拉角和磁场变量
		
		float angleErrVarVec[3];//欧拉角误差
		calcRotVecVariances(ele,angleErrVarVec);//根据当前P阵计算欧拉角误差
		for(u8 i=0;i<4;i++){
			ele->state[i] = newQuat[i];
		}
		angleErrVarVec[2] = R_YAW_HEADING;//航向角误差
		initialiseQuatCovariances(ele,angleErrVarVec);//初始化四元数误差
		float quat_err[4];
		QuaternionDivision(newQuat,temp_Quat,quat_err);//计算新对准的四元数与原来四元数之差
		StoreQuatRotate(ele,quat_err);//补偿
		if (ele->in_flight) {
      ele->finalInflightYawInit = true;//航向角对准成功
    }
	}
}

void FuseMagnetometer(sEKF *ele){
	float q0       = ele->state[0];
	float q1       = ele->state[1];
	float q2       = ele->state[2];
	float q3       = ele->state[3];
	float magN     = ele->state[16];
	float magE     = ele->state[17];
	float magD     = ele->state[18];
	float magXbias = ele->state[19];
	float magYbias = ele->state[20];
	float magZbias = ele->state[21];
	
	float H_MAG[22];
	float SK_MX[5];
	float SK_MY[5];
	float SK_MZ[5];
	
	float MagPred[3];
	float DCM[3][3];
	
	DCM[0][0] = q0*q0 + q1*q1 - q2*q2 - q3*q3;
	DCM[0][1] = 2.0f*(q1*q2 + q0*q3);
	DCM[0][2] = 2.0f*(q1*q3-q0*q2);
	DCM[1][0] = 2.0f*(q1*q2 - q0*q3);
	DCM[1][1] = q0*q0 - q1*q1 + q2*q2 - q3*q3;
	DCM[1][2] = 2.0f*(q2*q3 + q0*q1);
	DCM[2][0] = 2.0f*(q1*q3 + q0*q2);
	DCM[2][1] = 2.0f*(q2*q3 - q0*q1);
	DCM[2][2] = q0*q0 - q1*q1 - q2*q2 + q3*q3;
	MagPred[0] = DCM[0][0]*magN + DCM[0][1]*magE  + DCM[0][2]*magD + magXbias;
	MagPred[1] = DCM[1][0]*magN + DCM[1][1]*magE  + DCM[1][2]*magD + magYbias;
	MagPred[2] = DCM[2][0]*magN + DCM[2][1]*magE  + DCM[2][2]*magD + magZbias;
	
	for(u8 i = 0;i<3;i++){
		innov_Mag[i] = MagPred[i] - ele->magDataDelayed.mag[i];//磁力计新息
	}
	
	float R_MAG = MAG_NED_R * MAG_NED_R;//磁力计的方差
	
	float SH_MAG[9];
	SH_MAG[0] = 2.0f*magD*q3 + 2.0f*magE*q2 + 2.0f*magN*q1;
	SH_MAG[1] = 2.0f*magD*q0 - 2.0f*magE*q1 + 2.0f*magN*q2;
	SH_MAG[2] = 2.0f*magD*q1 + 2.0f*magE*q0 - 2.0f*magN*q3;
	SH_MAG[3] = (q3)*(q3);
	SH_MAG[4] = (q2)*(q2);
	SH_MAG[5] = (q1)*(q1);
	SH_MAG[6] = (q0)*(q0);
	SH_MAG[7] = 2.0f*magN*q0;
	SH_MAG[8] = 2.0f*magE*q3;
	
	varInnovMag[0] = (P[19][19] + R_MAG + P[1][19]*SH_MAG[0] - P[2][19]*SH_MAG[1] + P[3][19]*SH_MAG[2] - P[16][19]*(SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6]) + (2.0f*q0*q3 + 2.0f*q1*q2)*(P[19][17] + P[1][17]*SH_MAG[0] - P[2][17]*SH_MAG[1] + P[3][17]*SH_MAG[2] - P[16][17]*(SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6]) + P[17][17]*(2.0f*q0*q3 + 2.0f*q1*q2) - P[18][17]*(2.0f*q0*q2 - 2.0f*q1*q3) + P[0][17]*(SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2)) - (2.0f*q0*q2 - 2.0f*q1*q3)*(P[19][18] + P[1][18]*SH_MAG[0] - P[2][18]*SH_MAG[1] + P[3][18]*SH_MAG[2] - P[16][18]*(SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6]) + P[17][18]*(2.0f*q0*q3 + 2.0f*q1*q2) - P[18][18]*(2.0f*q0*q2 - 2.0f*q1*q3) + P[0][18]*(SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2)) + (SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2)*(P[19][0] + P[1][0]*SH_MAG[0] - P[2][0]*SH_MAG[1] + P[3][0]*SH_MAG[2] - P[16][0]*(SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6]) + P[17][0]*(2.0f*q0*q3 + 2.0f*q1*q2) - P[18][0]*(2.0f*q0*q2 - 2.0f*q1*q3) + P[0][0]*(SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2)) + P[17][19]*(2.0f*q0*q3 + 2.0f*q1*q2) - P[18][19]*(2.0f*q0*q2 - 2.0f*q1*q3) + SH_MAG[0]*(P[19][1] + P[1][1]*SH_MAG[0] - P[2][1]*SH_MAG[1] + P[3][1]*SH_MAG[2] - P[16][1]*(SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6]) + P[17][1]*(2.0f*q0*q3 + 2.0f*q1*q2) - P[18][1]*(2.0f*q0*q2 - 2.0f*q1*q3) + P[0][1]*(SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2)) - SH_MAG[1]*(P[19][2] + P[1][2]*SH_MAG[0] - P[2][2]*SH_MAG[1] + P[3][2]*SH_MAG[2] - P[16][2]*(SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6]) + P[17][2]*(2.0f*q0*q3 + 2.0f*q1*q2) - P[18][2]*(2.0f*q0*q2 - 2.0f*q1*q3) + P[0][2]*(SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2)) + SH_MAG[2]*(P[19][3] + P[1][3]*SH_MAG[0] - P[2][3]*SH_MAG[1] + P[3][3]*SH_MAG[2] - P[16][3]*(SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6]) + P[17][3]*(2.0f*q0*q3 + 2.0f*q1*q2) - P[18][3]*(2.0f*q0*q2 - 2.0f*q1*q3) + P[0][3]*(SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2)) - (SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6])*(P[19][16] + P[1][16]*SH_MAG[0] - P[2][16]*SH_MAG[1] + P[3][16]*SH_MAG[2] - P[16][16]*(SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6]) + P[17][16]*(2.0f*q0*q3 + 2.0f*q1*q2) - P[18][16]*(2.0f*q0*q2 - 2.0f*q1*q3) + P[0][16]*(SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2)) + P[0][19]*(SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2));
  varInnovMag[1] = (P[20][20] + R_MAG + P[0][20]*SH_MAG[2] + P[1][20]*SH_MAG[1] + P[2][20]*SH_MAG[0] - P[17][20]*(SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6]) - (2.0f*q0*q3 - 2.0f*q1*q2)*(P[20][16] + P[0][16]*SH_MAG[2] + P[1][16]*SH_MAG[1] + P[2][16]*SH_MAG[0] - P[17][16]*(SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6]) - P[16][16]*(2.0f*q0*q3 - 2.0f*q1*q2) + P[18][16]*(2.0f*q0*q1 + 2.0f*q2*q3) - P[3][16]*(SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2)) + (2.0f*q0*q1 + 2.0f*q2*q3)*(P[20][18] + P[0][18]*SH_MAG[2] + P[1][18]*SH_MAG[1] + P[2][18]*SH_MAG[0] - P[17][18]*(SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6]) - P[16][18]*(2.0f*q0*q3 - 2.0f*q1*q2) + P[18][18]*(2.0f*q0*q1 + 2.0f*q2*q3) - P[3][18]*(SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2)) - (SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2)*(P[20][3] + P[0][3]*SH_MAG[2] + P[1][3]*SH_MAG[1] + P[2][3]*SH_MAG[0] - P[17][3]*(SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6]) - P[16][3]*(2.0f*q0*q3 - 2.0f*q1*q2) + P[18][3]*(2.0f*q0*q1 + 2.0f*q2*q3) - P[3][3]*(SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2)) - P[16][20]*(2.0f*q0*q3 - 2.0f*q1*q2) + P[18][20]*(2.0f*q0*q1 + 2.0f*q2*q3) + SH_MAG[2]*(P[20][0] + P[0][0]*SH_MAG[2] + P[1][0]*SH_MAG[1] + P[2][0]*SH_MAG[0] - P[17][0]*(SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6]) - P[16][0]*(2.0f*q0*q3 - 2.0f*q1*q2) + P[18][0]*(2.0f*q0*q1 + 2.0f*q2*q3) - P[3][0]*(SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2)) + SH_MAG[1]*(P[20][1] + P[0][1]*SH_MAG[2] + P[1][1]*SH_MAG[1] + P[2][1]*SH_MAG[0] - P[17][1]*(SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6]) - P[16][1]*(2.0f*q0*q3 - 2.0f*q1*q2) + P[18][1]*(2.0f*q0*q1 + 2.0f*q2*q3) - P[3][1]*(SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2)) + SH_MAG[0]*(P[20][2] + P[0][2]*SH_MAG[2] + P[1][2]*SH_MAG[1] + P[2][2]*SH_MAG[0] - P[17][2]*(SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6]) - P[16][2]*(2.0f*q0*q3 - 2.0f*q1*q2) + P[18][2]*(2.0f*q0*q1 + 2.0f*q2*q3) - P[3][2]*(SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2)) - (SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6])*(P[20][17] + P[0][17]*SH_MAG[2] + P[1][17]*SH_MAG[1] + P[2][17]*SH_MAG[0] - P[17][17]*(SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6]) - P[16][17]*(2.0f*q0*q3 - 2.0f*q1*q2) + P[18][17]*(2.0f*q0*q1 + 2.0f*q2*q3) - P[3][17]*(SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2)) - P[3][20]*(SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2));
  varInnovMag[2] = (P[21][21] + R_MAG + P[0][21]*SH_MAG[1] - P[1][21]*SH_MAG[2] + P[3][21]*SH_MAG[0] + P[18][21]*(SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6]) + (2.0f*q0*q2 + 2.0f*q1*q3)*(P[21][16] + P[0][16]*SH_MAG[1] - P[1][16]*SH_MAG[2] + P[3][16]*SH_MAG[0] + P[18][16]*(SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6]) + P[16][16]*(2.0f*q0*q2 + 2.0f*q1*q3) - P[17][16]*(2.0f*q0*q1 - 2.0f*q2*q3) + P[2][16]*(SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2)) - (2.0f*q0*q1 - 2.0f*q2*q3)*(P[21][17] + P[0][17]*SH_MAG[1] - P[1][17]*SH_MAG[2] + P[3][17]*SH_MAG[0] + P[18][17]*(SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6]) + P[16][17]*(2.0f*q0*q2 + 2.0f*q1*q3) - P[17][17]*(2.0f*q0*q1 - 2.0f*q2*q3) + P[2][17]*(SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2)) + (SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2)*(P[21][2] + P[0][2]*SH_MAG[1] - P[1][2]*SH_MAG[2] + P[3][2]*SH_MAG[0] + P[18][2]*(SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6]) + P[16][2]*(2.0f*q0*q2 + 2.0f*q1*q3) - P[17][2]*(2.0f*q0*q1 - 2.0f*q2*q3) + P[2][2]*(SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2)) + P[16][21]*(2.0f*q0*q2 + 2.0f*q1*q3) - P[17][21]*(2.0f*q0*q1 - 2.0f*q2*q3) + SH_MAG[1]*(P[21][0] + P[0][0]*SH_MAG[1] - P[1][0]*SH_MAG[2] + P[3][0]*SH_MAG[0] + P[18][0]*(SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6]) + P[16][0]*(2.0f*q0*q2 + 2.0f*q1*q3) - P[17][0]*(2.0f*q0*q1 - 2.0f*q2*q3) + P[2][0]*(SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2)) - SH_MAG[2]*(P[21][1] + P[0][1]*SH_MAG[1] - P[1][1]*SH_MAG[2] + P[3][1]*SH_MAG[0] + P[18][1]*(SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6]) + P[16][1]*(2.0f*q0*q2 + 2.0f*q1*q3) - P[17][1]*(2.0f*q0*q1 - 2.0f*q2*q3) + P[2][1]*(SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2)) + SH_MAG[0]*(P[21][3] + P[0][3]*SH_MAG[1] - P[1][3]*SH_MAG[2] + P[3][3]*SH_MAG[0] + P[18][3]*(SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6]) + P[16][3]*(2.0f*q0*q2 + 2.0f*q1*q3) - P[17][3]*(2.0f*q0*q1 - 2.0f*q2*q3) + P[2][3]*(SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2)) + (SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6])*(P[21][18] + P[0][18]*SH_MAG[1] - P[1][18]*SH_MAG[2] + P[3][18]*SH_MAG[0] + P[18][18]*(SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6]) + P[16][18]*(2.0f*q0*q2 + 2.0f*q1*q3) - P[17][18]*(2.0f*q0*q1 - 2.0f*q2*q3) + P[2][18]*(SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2)) + P[2][21]*(SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2));

 for(u8 i=0 ; i<=2 ; i++)
 {
	 float innov_temp = powf(innov_Mag[i],2);
	 float varInnov_temp = powf(ele->magFieldGate,2)*varInnovMag[i];
	 ele->magTestRatio[i] = innov_temp/varInnov_temp;
	// if (ele->magTestRatio[i]>1.0f) 
		// return ;
 }

	float Kfusion[22];
	for(u8 obsIndex=0;obsIndex< 3;obsIndex++){
		if(obsIndex == 0){
			for (u8 j = 0; j<=ele->stateIndexLim; j++) H_MAG[j] = 0.0f;
			H_MAG[0] = SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2;
			H_MAG[1] = SH_MAG[0];
			H_MAG[2] = -SH_MAG[1];
			H_MAG[3] = SH_MAG[2];
			H_MAG[16] = SH_MAG[5] - SH_MAG[4] - SH_MAG[3] + SH_MAG[6];
			H_MAG[17] = 2.0f*q0*q3 + 2.0f*q1*q2;
			H_MAG[18] = 2.0f*q1*q3 - 2.0f*q0*q2;
			H_MAG[19] = 1.0f;
			
			SK_MX[0] = 1.0f / varInnovMag[0];
			SK_MX[1] = SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6];
			SK_MX[2] = SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2;
			SK_MX[3] = 2.0f*q0*q2 - 2.0f*q1*q3;
			SK_MX[4] = 2.0f*q0*q3 + 2.0f*q1*q2;
			
			Kfusion[0] = SK_MX[0]*(P[0][19] + P[0][1]*SH_MAG[0] - P[0][2]*SH_MAG[1] + P[0][3]*SH_MAG[2] + P[0][0]*SK_MX[2] - P[0][16]*SK_MX[1] + P[0][17]*SK_MX[4] - P[0][18]*SK_MX[3]);
			Kfusion[1] = SK_MX[0]*(P[1][19] + P[1][1]*SH_MAG[0] - P[1][2]*SH_MAG[1] + P[1][3]*SH_MAG[2] + P[1][0]*SK_MX[2] - P[1][16]*SK_MX[1] + P[1][17]*SK_MX[4] - P[1][18]*SK_MX[3]);
			Kfusion[2] = SK_MX[0]*(P[2][19] + P[2][1]*SH_MAG[0] - P[2][2]*SH_MAG[1] + P[2][3]*SH_MAG[2] + P[2][0]*SK_MX[2] - P[2][16]*SK_MX[1] + P[2][17]*SK_MX[4] - P[2][18]*SK_MX[3]);
			Kfusion[3] = SK_MX[0]*(P[3][19] + P[3][1]*SH_MAG[0] - P[3][2]*SH_MAG[1] + P[3][3]*SH_MAG[2] + P[3][0]*SK_MX[2] - P[3][16]*SK_MX[1] + P[3][17]*SK_MX[4] - P[3][18]*SK_MX[3]);
			Kfusion[4] = SK_MX[0]*(P[4][19] + P[4][1]*SH_MAG[0] - P[4][2]*SH_MAG[1] + P[4][3]*SH_MAG[2] + P[4][0]*SK_MX[2] - P[4][16]*SK_MX[1] + P[4][17]*SK_MX[4] - P[4][18]*SK_MX[3]);
			Kfusion[5] = SK_MX[0]*(P[5][19] + P[5][1]*SH_MAG[0] - P[5][2]*SH_MAG[1] + P[5][3]*SH_MAG[2] + P[5][0]*SK_MX[2] - P[5][16]*SK_MX[1] + P[5][17]*SK_MX[4] - P[5][18]*SK_MX[3]);
			Kfusion[6] = SK_MX[0]*(P[6][19] + P[6][1]*SH_MAG[0] - P[6][2]*SH_MAG[1] + P[6][3]*SH_MAG[2] + P[6][0]*SK_MX[2] - P[6][16]*SK_MX[1] + P[6][17]*SK_MX[4] - P[6][18]*SK_MX[3]);
			Kfusion[7] = SK_MX[0]*(P[7][19] + P[7][1]*SH_MAG[0] - P[7][2]*SH_MAG[1] + P[7][3]*SH_MAG[2] + P[7][0]*SK_MX[2] - P[7][16]*SK_MX[1] + P[7][17]*SK_MX[4] - P[7][18]*SK_MX[3]);
			Kfusion[8] = SK_MX[0]*(P[8][19] + P[8][1]*SH_MAG[0] - P[8][2]*SH_MAG[1] + P[8][3]*SH_MAG[2] + P[8][0]*SK_MX[2] - P[8][16]*SK_MX[1] + P[8][17]*SK_MX[4] - P[8][18]*SK_MX[3]);
			Kfusion[9] = SK_MX[0]*(P[9][19] + P[9][1]*SH_MAG[0] - P[9][2]*SH_MAG[1] + P[9][3]*SH_MAG[2] + P[9][0]*SK_MX[2] - P[9][16]*SK_MX[1] + P[9][17]*SK_MX[4] - P[9][18]*SK_MX[3]);
			Kfusion[10] = SK_MX[0]*(P[10][19] + P[10][1]*SH_MAG[0] - P[10][2]*SH_MAG[1] + P[10][3]*SH_MAG[2] + P[10][0]*SK_MX[2] - P[10][16]*SK_MX[1] + P[10][17]*SK_MX[4] - P[10][18]*SK_MX[3]);
			Kfusion[11] = SK_MX[0]*(P[11][19] + P[11][1]*SH_MAG[0] - P[11][2]*SH_MAG[1] + P[11][3]*SH_MAG[2] + P[11][0]*SK_MX[2] - P[11][16]*SK_MX[1] + P[11][17]*SK_MX[4] - P[11][18]*SK_MX[3]);
			Kfusion[12] = SK_MX[0]*(P[12][19] + P[12][1]*SH_MAG[0] - P[12][2]*SH_MAG[1] + P[12][3]*SH_MAG[2] + P[12][0]*SK_MX[2] - P[12][16]*SK_MX[1] + P[12][17]*SK_MX[4] - P[12][18]*SK_MX[3]);
			Kfusion[13] = SK_MX[0]*(P[13][19] + P[13][1]*SH_MAG[0] - P[13][2]*SH_MAG[1] + P[13][3]*SH_MAG[2] + P[13][0]*SK_MX[2] - P[13][16]*SK_MX[1] + P[13][17]*SK_MX[4] - P[13][18]*SK_MX[3]);
			Kfusion[14] = SK_MX[0]*(P[14][19] + P[14][1]*SH_MAG[0] - P[14][2]*SH_MAG[1] + P[14][3]*SH_MAG[2] + P[14][0]*SK_MX[2] - P[14][16]*SK_MX[1] + P[14][17]*SK_MX[4] - P[14][18]*SK_MX[3]);
			Kfusion[15] = SK_MX[0]*(P[15][19] + P[15][1]*SH_MAG[0] - P[15][2]*SH_MAG[1] + P[15][3]*SH_MAG[2] + P[15][0]*SK_MX[2] - P[15][16]*SK_MX[1] + P[15][17]*SK_MX[4] - P[15][18]*SK_MX[3]);
			Kfusion[16] = SK_MX[0]*(P[16][19] + P[16][1]*SH_MAG[0] - P[16][2]*SH_MAG[1] + P[16][3]*SH_MAG[2] + P[16][0]*SK_MX[2] - P[16][16]*SK_MX[1] + P[16][17]*SK_MX[4] - P[16][18]*SK_MX[3]);
			Kfusion[17] = SK_MX[0]*(P[17][19] + P[17][1]*SH_MAG[0] - P[17][2]*SH_MAG[1] + P[17][3]*SH_MAG[2] + P[17][0]*SK_MX[2] - P[17][16]*SK_MX[1] + P[17][17]*SK_MX[4] - P[17][18]*SK_MX[3]);
			Kfusion[18] = SK_MX[0]*(P[18][19] + P[18][1]*SH_MAG[0] - P[18][2]*SH_MAG[1] + P[18][3]*SH_MAG[2] + P[18][0]*SK_MX[2] - P[18][16]*SK_MX[1] + P[18][17]*SK_MX[4] - P[18][18]*SK_MX[3]);
			Kfusion[19] = SK_MX[0]*(P[19][19] + P[19][1]*SH_MAG[0] - P[19][2]*SH_MAG[1] + P[19][3]*SH_MAG[2] + P[19][0]*SK_MX[2] - P[19][16]*SK_MX[1] + P[19][17]*SK_MX[4] - P[19][18]*SK_MX[3]);
			Kfusion[20] = SK_MX[0]*(P[20][19] + P[20][1]*SH_MAG[0] - P[20][2]*SH_MAG[1] + P[20][3]*SH_MAG[2] + P[20][0]*SK_MX[2] - P[20][16]*SK_MX[1] + P[20][17]*SK_MX[4] - P[20][18]*SK_MX[3]);
			Kfusion[21] = SK_MX[0]*(P[21][19] + P[21][1]*SH_MAG[0] - P[21][2]*SH_MAG[1] + P[21][3]*SH_MAG[2] + P[21][0]*SK_MX[2] - P[21][16]*SK_MX[1] + P[21][17]*SK_MX[4] - P[21][18]*SK_MX[3]);
		}else if(obsIndex == 1){
			// calculate observation jacobians
			for (u8 j = 0; j<=ele->stateIndexLim; j++) H_MAG[j] = 0.0f;
			H_MAG[0] = SH_MAG[2];
			H_MAG[1] = SH_MAG[1];
			H_MAG[2] = SH_MAG[0];
			H_MAG[3] = 2.0f*magD*q2 - SH_MAG[8] - SH_MAG[7];
			H_MAG[16] = 2.0f*q1*q2 - 2.0f*q0*q3;
			H_MAG[17] = SH_MAG[4] - SH_MAG[3] - SH_MAG[5] + SH_MAG[6];
			H_MAG[18] = 2.0f*q0*q1 + 2.0f*q2*q3;
			H_MAG[20] = 1.0f;

			// calculate Kalman gain
			SK_MY[0] = 1.0f / varInnovMag[1];
			SK_MY[1] = SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6];
			SK_MY[2] = SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2;
			SK_MY[3] = 2.0f*q0*q3 - 2.0f*q1*q2;
			SK_MY[4] = 2.0f*q0*q1 + 2.0f*q2*q3;

			Kfusion[0] = SK_MY[0]*(P[0][20] + P[0][0]*SH_MAG[2] + P[0][1]*SH_MAG[1] + P[0][2]*SH_MAG[0] - P[0][3]*SK_MY[2] - P[0][17]*SK_MY[1] - P[0][16]*SK_MY[3] + P[0][18]*SK_MY[4]);
			Kfusion[1] = SK_MY[0]*(P[1][20] + P[1][0]*SH_MAG[2] + P[1][1]*SH_MAG[1] + P[1][2]*SH_MAG[0] - P[1][3]*SK_MY[2] - P[1][17]*SK_MY[1] - P[1][16]*SK_MY[3] + P[1][18]*SK_MY[4]);
			Kfusion[2] = SK_MY[0]*(P[2][20] + P[2][0]*SH_MAG[2] + P[2][1]*SH_MAG[1] + P[2][2]*SH_MAG[0] - P[2][3]*SK_MY[2] - P[2][17]*SK_MY[1] - P[2][16]*SK_MY[3] + P[2][18]*SK_MY[4]);
			Kfusion[3] = SK_MY[0]*(P[3][20] + P[3][0]*SH_MAG[2] + P[3][1]*SH_MAG[1] + P[3][2]*SH_MAG[0] - P[3][3]*SK_MY[2] - P[3][17]*SK_MY[1] - P[3][16]*SK_MY[3] + P[3][18]*SK_MY[4]);
			Kfusion[4] = SK_MY[0]*(P[4][20] + P[4][0]*SH_MAG[2] + P[4][1]*SH_MAG[1] + P[4][2]*SH_MAG[0] - P[4][3]*SK_MY[2] - P[4][17]*SK_MY[1] - P[4][16]*SK_MY[3] + P[4][18]*SK_MY[4]);
			Kfusion[5] = SK_MY[0]*(P[5][20] + P[5][0]*SH_MAG[2] + P[5][1]*SH_MAG[1] + P[5][2]*SH_MAG[0] - P[5][3]*SK_MY[2] - P[5][17]*SK_MY[1] - P[5][16]*SK_MY[3] + P[5][18]*SK_MY[4]);
			Kfusion[6] = SK_MY[0]*(P[6][20] + P[6][0]*SH_MAG[2] + P[6][1]*SH_MAG[1] + P[6][2]*SH_MAG[0] - P[6][3]*SK_MY[2] - P[6][17]*SK_MY[1] - P[6][16]*SK_MY[3] + P[6][18]*SK_MY[4]);
			Kfusion[7] = SK_MY[0]*(P[7][20] + P[7][0]*SH_MAG[2] + P[7][1]*SH_MAG[1] + P[7][2]*SH_MAG[0] - P[7][3]*SK_MY[2] - P[7][17]*SK_MY[1] - P[7][16]*SK_MY[3] + P[7][18]*SK_MY[4]);
			Kfusion[8] = SK_MY[0]*(P[8][20] + P[8][0]*SH_MAG[2] + P[8][1]*SH_MAG[1] + P[8][2]*SH_MAG[0] - P[8][3]*SK_MY[2] - P[8][17]*SK_MY[1] - P[8][16]*SK_MY[3] + P[8][18]*SK_MY[4]);
			Kfusion[9] = SK_MY[0]*(P[9][20] + P[9][0]*SH_MAG[2] + P[9][1]*SH_MAG[1] + P[9][2]*SH_MAG[0] - P[9][3]*SK_MY[2] - P[9][17]*SK_MY[1] - P[9][16]*SK_MY[3] + P[9][18]*SK_MY[4]);
			Kfusion[10] = SK_MY[0]*(P[10][20] + P[10][0]*SH_MAG[2] + P[10][1]*SH_MAG[1] + P[10][2]*SH_MAG[0] - P[10][3]*SK_MY[2] - P[10][17]*SK_MY[1] - P[10][16]*SK_MY[3] + P[10][18]*SK_MY[4]);
			Kfusion[11] = SK_MY[0]*(P[11][20] + P[11][0]*SH_MAG[2] + P[11][1]*SH_MAG[1] + P[11][2]*SH_MAG[0] - P[11][3]*SK_MY[2] - P[11][17]*SK_MY[1] - P[11][16]*SK_MY[3] + P[11][18]*SK_MY[4]);
			Kfusion[12] = SK_MY[0]*(P[12][20] + P[12][0]*SH_MAG[2] + P[12][1]*SH_MAG[1] + P[12][2]*SH_MAG[0] - P[12][3]*SK_MY[2] - P[12][17]*SK_MY[1] - P[12][16]*SK_MY[3] + P[12][18]*SK_MY[4]);
			Kfusion[13] = SK_MY[0]*(P[13][20] + P[13][0]*SH_MAG[2] + P[13][1]*SH_MAG[1] + P[13][2]*SH_MAG[0] - P[13][3]*SK_MY[2] - P[13][17]*SK_MY[1] - P[13][16]*SK_MY[3] + P[13][18]*SK_MY[4]);
			Kfusion[14] = SK_MY[0]*(P[14][20] + P[14][0]*SH_MAG[2] + P[14][1]*SH_MAG[1] + P[14][2]*SH_MAG[0] - P[14][3]*SK_MY[2] - P[14][17]*SK_MY[1] - P[14][16]*SK_MY[3] + P[14][18]*SK_MY[4]);
			Kfusion[15] = SK_MY[0]*(P[15][20] + P[15][0]*SH_MAG[2] + P[15][1]*SH_MAG[1] + P[15][2]*SH_MAG[0] - P[15][3]*SK_MY[2] - P[15][17]*SK_MY[1] - P[15][16]*SK_MY[3] + P[15][18]*SK_MY[4]);
			Kfusion[16] = SK_MY[0]*(P[16][20] + P[16][0]*SH_MAG[2] + P[16][1]*SH_MAG[1] + P[16][2]*SH_MAG[0] - P[16][3]*SK_MY[2] - P[16][17]*SK_MY[1] - P[16][16]*SK_MY[3] + P[16][18]*SK_MY[4]);
			Kfusion[17] = SK_MY[0]*(P[17][20] + P[17][0]*SH_MAG[2] + P[17][1]*SH_MAG[1] + P[17][2]*SH_MAG[0] - P[17][3]*SK_MY[2] - P[17][17]*SK_MY[1] - P[17][16]*SK_MY[3] + P[17][18]*SK_MY[4]);
			Kfusion[18] = SK_MY[0]*(P[18][20] + P[18][0]*SH_MAG[2] + P[18][1]*SH_MAG[1] + P[18][2]*SH_MAG[0] - P[18][3]*SK_MY[2] - P[18][17]*SK_MY[1] - P[18][16]*SK_MY[3] + P[18][18]*SK_MY[4]);
			Kfusion[19] = SK_MY[0]*(P[19][20] + P[19][0]*SH_MAG[2] + P[19][1]*SH_MAG[1] + P[19][2]*SH_MAG[0] - P[19][3]*SK_MY[2] - P[19][17]*SK_MY[1] - P[19][16]*SK_MY[3] + P[19][18]*SK_MY[4]);
			Kfusion[20] = SK_MY[0]*(P[20][20] + P[20][0]*SH_MAG[2] + P[20][1]*SH_MAG[1] + P[20][2]*SH_MAG[0] - P[20][3]*SK_MY[2] - P[20][17]*SK_MY[1] - P[20][16]*SK_MY[3] + P[20][18]*SK_MY[4]);
			Kfusion[21] = SK_MY[0]*(P[21][20] + P[21][0]*SH_MAG[2] + P[21][1]*SH_MAG[1] + P[21][2]*SH_MAG[0] - P[21][3]*SK_MY[2] - P[21][17]*SK_MY[1] - P[21][16]*SK_MY[3] + P[21][18]*SK_MY[4]);
		}else if(obsIndex == 2){
			for (u8 j = 0; j<=ele->stateIndexLim; j++) H_MAG[j] = 0.0f;
			H_MAG[0] = SH_MAG[1];
			H_MAG[1] = -SH_MAG[2];
			H_MAG[2] = SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2;
			H_MAG[3] = SH_MAG[0];
			H_MAG[16] = 2.0f*q0*q2 + 2.0f*q1*q3;
			H_MAG[17] = 2.0f*q2*q3 - 2.0f*q0*q1;
			H_MAG[18] = SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6];
			H_MAG[21] = 1.0f;

			SK_MZ[0] = 1.0f / varInnovMag[2];
			SK_MZ[1] = SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6];
			SK_MZ[2] = SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2;
			SK_MZ[3] = 2.0f*q0*q1 - 2.0f*q2*q3;
			SK_MZ[4] = 2.0f*q0*q2 + 2.0f*q1*q3;
			
			Kfusion[0] = SK_MZ[0]*(P[0][21] + P[0][0]*SH_MAG[1] - P[0][1]*SH_MAG[2] + P[0][3]*SH_MAG[0] + P[0][2]*SK_MZ[2] + P[0][18]*SK_MZ[1] + P[0][16]*SK_MZ[4] - P[0][17]*SK_MZ[3]);
			Kfusion[1] = SK_MZ[0]*(P[1][21] + P[1][0]*SH_MAG[1] - P[1][1]*SH_MAG[2] + P[1][3]*SH_MAG[0] + P[1][2]*SK_MZ[2] + P[1][18]*SK_MZ[1] + P[1][16]*SK_MZ[4] - P[1][17]*SK_MZ[3]);
			Kfusion[2] = SK_MZ[0]*(P[2][21] + P[2][0]*SH_MAG[1] - P[2][1]*SH_MAG[2] + P[2][3]*SH_MAG[0] + P[2][2]*SK_MZ[2] + P[2][18]*SK_MZ[1] + P[2][16]*SK_MZ[4] - P[2][17]*SK_MZ[3]);
			Kfusion[3] = SK_MZ[0]*(P[3][21] + P[3][0]*SH_MAG[1] - P[3][1]*SH_MAG[2] + P[3][3]*SH_MAG[0] + P[3][2]*SK_MZ[2] + P[3][18]*SK_MZ[1] + P[3][16]*SK_MZ[4] - P[3][17]*SK_MZ[3]);
			Kfusion[4] = SK_MZ[0]*(P[4][21] + P[4][0]*SH_MAG[1] - P[4][1]*SH_MAG[2] + P[4][3]*SH_MAG[0] + P[4][2]*SK_MZ[2] + P[4][18]*SK_MZ[1] + P[4][16]*SK_MZ[4] - P[4][17]*SK_MZ[3]);
			Kfusion[5] = SK_MZ[0]*(P[5][21] + P[5][0]*SH_MAG[1] - P[5][1]*SH_MAG[2] + P[5][3]*SH_MAG[0] + P[5][2]*SK_MZ[2] + P[5][18]*SK_MZ[1] + P[5][16]*SK_MZ[4] - P[5][17]*SK_MZ[3]);
			Kfusion[6] = SK_MZ[0]*(P[6][21] + P[6][0]*SH_MAG[1] - P[6][1]*SH_MAG[2] + P[6][3]*SH_MAG[0] + P[6][2]*SK_MZ[2] + P[6][18]*SK_MZ[1] + P[6][16]*SK_MZ[4] - P[6][17]*SK_MZ[3]);
			Kfusion[7] = SK_MZ[0]*(P[7][21] + P[7][0]*SH_MAG[1] - P[7][1]*SH_MAG[2] + P[7][3]*SH_MAG[0] + P[7][2]*SK_MZ[2] + P[7][18]*SK_MZ[1] + P[7][16]*SK_MZ[4] - P[7][17]*SK_MZ[3]);
			Kfusion[8] = SK_MZ[0]*(P[8][21] + P[8][0]*SH_MAG[1] - P[8][1]*SH_MAG[2] + P[8][3]*SH_MAG[0] + P[8][2]*SK_MZ[2] + P[8][18]*SK_MZ[1] + P[8][16]*SK_MZ[4] - P[8][17]*SK_MZ[3]);
			Kfusion[9] = SK_MZ[0]*(P[9][21] + P[9][0]*SH_MAG[1] - P[9][1]*SH_MAG[2] + P[9][3]*SH_MAG[0] + P[9][2]*SK_MZ[2] + P[9][18]*SK_MZ[1] + P[9][16]*SK_MZ[4] - P[9][17]*SK_MZ[3]);
			Kfusion[10] = SK_MZ[0]*(P[10][21] + P[10][0]*SH_MAG[1] - P[10][1]*SH_MAG[2] + P[10][3]*SH_MAG[0] + P[10][2]*SK_MZ[2] + P[10][18]*SK_MZ[1] + P[10][16]*SK_MZ[4] - P[10][17]*SK_MZ[3]);
			Kfusion[11] = SK_MZ[0]*(P[11][21] + P[11][0]*SH_MAG[1] - P[11][1]*SH_MAG[2] + P[11][3]*SH_MAG[0] + P[11][2]*SK_MZ[2] + P[11][18]*SK_MZ[1] + P[11][16]*SK_MZ[4] - P[11][17]*SK_MZ[3]);
			Kfusion[12] = SK_MZ[0]*(P[12][21] + P[12][0]*SH_MAG[1] - P[12][1]*SH_MAG[2] + P[12][3]*SH_MAG[0] + P[12][2]*SK_MZ[2] + P[12][18]*SK_MZ[1] + P[12][16]*SK_MZ[4] - P[12][17]*SK_MZ[3]);
			Kfusion[13] = SK_MZ[0]*(P[13][21] + P[13][0]*SH_MAG[1] - P[13][1]*SH_MAG[2] + P[13][3]*SH_MAG[0] + P[13][2]*SK_MZ[2] + P[13][18]*SK_MZ[1] + P[13][16]*SK_MZ[4] - P[13][17]*SK_MZ[3]);
			Kfusion[14] = SK_MZ[0]*(P[14][21] + P[14][0]*SH_MAG[1] - P[14][1]*SH_MAG[2] + P[14][3]*SH_MAG[0] + P[14][2]*SK_MZ[2] + P[14][18]*SK_MZ[1] + P[14][16]*SK_MZ[4] - P[14][17]*SK_MZ[3]);
			Kfusion[15] = SK_MZ[0]*(P[15][21] + P[15][0]*SH_MAG[1] - P[15][1]*SH_MAG[2] + P[15][3]*SH_MAG[0] + P[15][2]*SK_MZ[2] + P[15][18]*SK_MZ[1] + P[15][16]*SK_MZ[4] - P[15][17]*SK_MZ[3]);
			Kfusion[16] = SK_MZ[0]*(P[16][21] + P[16][0]*SH_MAG[1] - P[16][1]*SH_MAG[2] + P[16][3]*SH_MAG[0] + P[16][2]*SK_MZ[2] + P[16][18]*SK_MZ[1] + P[16][16]*SK_MZ[4] - P[16][17]*SK_MZ[3]);
			Kfusion[17] = SK_MZ[0]*(P[17][21] + P[17][0]*SH_MAG[1] - P[17][1]*SH_MAG[2] + P[17][3]*SH_MAG[0] + P[17][2]*SK_MZ[2] + P[17][18]*SK_MZ[1] + P[17][16]*SK_MZ[4] - P[17][17]*SK_MZ[3]);
			Kfusion[18] = SK_MZ[0]*(P[18][21] + P[18][0]*SH_MAG[1] - P[18][1]*SH_MAG[2] + P[18][3]*SH_MAG[0] + P[18][2]*SK_MZ[2] + P[18][18]*SK_MZ[1] + P[18][16]*SK_MZ[4] - P[18][17]*SK_MZ[3]);
			Kfusion[19] = SK_MZ[0]*(P[19][21] + P[19][0]*SH_MAG[1] - P[19][1]*SH_MAG[2] + P[19][3]*SH_MAG[0] + P[19][2]*SK_MZ[2] + P[19][18]*SK_MZ[1] + P[19][16]*SK_MZ[4] - P[19][17]*SK_MZ[3]);
			Kfusion[20] = SK_MZ[0]*(P[20][21] + P[20][0]*SH_MAG[1] - P[20][1]*SH_MAG[2] + P[20][3]*SH_MAG[0] + P[20][2]*SK_MZ[2] + P[20][18]*SK_MZ[1] + P[20][16]*SK_MZ[4] - P[20][17]*SK_MZ[3]);
			Kfusion[21] = SK_MZ[0]*(P[21][21] + P[21][0]*SH_MAG[1] - P[21][1]*SH_MAG[2] + P[21][3]*SH_MAG[0] + P[21][2]*SK_MZ[2] + P[21][18]*SK_MZ[1] + P[21][16]*SK_MZ[4] - P[21][17]*SK_MZ[3]);
		}
		//磁力计融合标志位置1
		ele->magFusePerformed = true;
		//更新KH矩阵
		for (u8 i = 0; i<=ele->stateIndexLim; i++) {
			for (u8 j = 0; j<=3; j++) {
				KH[i][j] = Kfusion[i] * H_MAG[j];
			}
			for (u8 j = 4; j<=15; j++) {
				KH[i][j] = 0.0f;
			}
			for (u8 j = 16; j<=21; j++) {
				KH[i][j] = Kfusion[i] * H_MAG[j];
			}
		}
		//更新KHP矩阵
		for (u8 j = 0; j<=ele->stateIndexLim; j++) {
			for (u8 i = 0; i<=ele->stateIndexLim; i++) {
				float res = 0.0f;
				res += KH[i][0] * P[0][j];
				res += KH[i][1] * P[1][j];
				res += KH[i][2] * P[2][j];
				res += KH[i][3] * P[3][j];
				res += KH[i][16] * P[16][j];
				res += KH[i][17] * P[17][j];
				res += KH[i][18] * P[18][j];
				res += KH[i][19] * P[19][j];
				res += KH[i][20] * P[20][j];
				res += KH[i][21] * P[21][j];
				KHP[i][j] = res;
			}
		}
		//更新P矩阵
		for (u8 i= 0; i<=ele->stateIndexLim; i++) {
			for (u8 j= 0; j<=ele->stateIndexLim; j++) {
				P[i][j] = P[i][j] - KHP[i][j];
			}
		}
		//强制对称
		ForceSymmetry();
		//状态修正
		for (u8 j= 0; j<=ele->stateIndexLim; j++) {
			ele->state[j] = ele->state[j] - Kfusion[j] * innov_Mag[obsIndex];
		}
		float quat_temp[4];
		for(u8 i = 0;i<4;i++){
			quat_temp[i] = ele->state[i];
		}
		QuaternionNormalise(quat_temp);
		for(u8 i = 0;i<4;i++){
			ele->state[i] = quat_temp[i];
		}
	}
}

void ForceSymmetry(void)
{
	for (u8 i=1; i<=21; i++)
	{
		for (u8 j=0; j<=i-1; j++)
		{
			float temp = 0.5f*(P[i][j] + P[j][i]);
			P[i][j] = temp;
			P[j][i] = temp;
		}
	}
}

void FuseDeclination(sEKF *ele)
{
	float magN = ele->state[16];
	float magE = ele->state[17];
	if (magN < 1e-3f) {
		return;
	}
	float t2 = magE*magE;
	float t3 = magN*magN;

	float t4 = t2+t3;
	if (t4 < 1e-4f) {
		return;
	}
	float t5 = P[16][16]*t2;
	float t6 = P[17][17]*t3;
	float t7 = t2*t2;
	float t8 = R_DECL*t7;
	float t9 = t3*t3;
	float t10 = R_DECL*t9;
	float t11 = R_DECL*t2*t3*2.0f;
	float t14 = P[16][17]*magE*magN;
	float t15 = P[17][16]*magE*magN;
	float t12 = t5+t6+t8+t10+t11-t14-t15;
	float t13;
	if (fabsf(t12) > 1e-6f) {
		t13 = 1.0f / t12;
	} else {
		return;
	}
	float t18 = magE*magE;
	float t19 = magN*magN;
	float t20 = t18+t19;
	float t21;
	if (fabsf(t20) > 1e-6f) {
		t21 = 1.0f/t20;
	} else {
		return;
	}

	float H_DECL[24];
	H_DECL[16] = -magE*t21;
	H_DECL[17] = magN*t21;

	float Kfusion[22];
	Kfusion[0] = -t4*t13*(P[0][16]*magE-P[0][17]*magN);
	Kfusion[1] = -t4*t13*(P[1][16]*magE-P[1][17]*magN);
	Kfusion[2] = -t4*t13*(P[2][16]*magE-P[2][17]*magN);
	Kfusion[3] = -t4*t13*(P[3][16]*magE-P[3][17]*magN);
	Kfusion[4] = -t4*t13*(P[4][16]*magE-P[4][17]*magN);
	Kfusion[5] = -t4*t13*(P[5][16]*magE-P[5][17]*magN);
	Kfusion[6] = -t4*t13*(P[6][16]*magE-P[6][17]*magN);
	Kfusion[7] = -t4*t13*(P[7][16]*magE-P[7][17]*magN);
	Kfusion[8] = -t4*t13*(P[8][16]*magE-P[8][17]*magN);
  Kfusion[9] = -t4*t13*(P[9][16]*magE-P[9][17]*magN);
	Kfusion[10] = -t4*t13*(P[10][16]*magE-P[10][17]*magN);
	Kfusion[11] = -t4*t13*(P[11][16]*magE-P[11][17]*magN);
	Kfusion[12] = -t4*t13*(P[12][16]*magE-P[12][17]*magN);
	Kfusion[13] = -t4*t13*(P[13][16]*magE-P[13][17]*magN);
	Kfusion[14] = -t4*t13*(P[14][16]*magE-P[14][17]*magN);
	Kfusion[15] = -t4*t13*(P[15][16]*magE-P[15][17]*magN);
	Kfusion[16] = -t4*t13*(P[16][16]*magE-P[16][17]*magN);
	Kfusion[17] = -t4*t13*(P[17][16]*magE-P[17][17]*magN);
	Kfusion[18] = -t4*t13*(P[18][16]*magE-P[18][17]*magN);
	Kfusion[19] = -t4*t13*(P[19][16]*magE-P[19][17]*magN);
	Kfusion[20] = -t4*t13*(P[20][16]*magE-P[20][17]*magN);
	Kfusion[21] = -t4*t13*(P[21][16]*magE-P[21][17]*magN);
	
	float magDecAng = -2.8f * D2R;
	float innovation = atan2f(magE , magN) - magDecAng;

	for (u8 i = 0; i<=ele->stateIndexLim; i++) {
		for (u8 j = 0; j<=15; j++) {
			KH[i][j] = 0.0f;
		}
		KH[i][16] = Kfusion[i] * H_DECL[16];
		KH[i][17] = Kfusion[i] * H_DECL[17];
		for (u8 j = 18; j<=21; j++) {
			KH[i][j] = 0.0f;
		}
	}
	for (u8 j = 0; j<=ele->stateIndexLim; j++) {
		for (u8 i = 0; i<=ele->stateIndexLim; i++) {
			KHP[i][j] = KH[i][16] * P[16][j] + KH[i][17] * P[17][j];
		}
	}
	for (uint8_t i= 0; i<=ele->stateIndexLim; i++) {
		for (uint8_t j= 0; j<=ele->stateIndexLim; j++) {
			P[i][j] = P[i][j] - KHP[i][j];
		}
	}
	//强制对称
	ForceSymmetry();
	//状态修正
	for (u8 j= 0; j<=ele->stateIndexLim; j++) {
		ele->state[j] = ele->state[j] - Kfusion[j] * innovation;
	}
	float quat_temp[4];
	for(u8 i = 0;i<4;i++){
		quat_temp[i] = ele->state[i];
	}
	QuaternionNormalise(quat_temp);
	for(u8 i = 0;i<4;i++){
		ele->state[i] = quat_temp[i];
	}
}

void fuseEulerYaw(sEKF *ele)
{
	float q0 = ele->state[0];
	float q1 = ele->state[1];
	float q2 = ele->state[2];
	float q3 = ele->state[3];

	float predicted_yaw;
	float H_YAW[4];
	float Tbn_zeroYaw[3][3];

	if (fabsf(ele->prevTnb[0][2]) < fabsf(ele->prevTnb[1][2])) {
			// calculate observation jacobian when we are observing the first rotation in a 321 sequence
		float t9 = q0*q3;
		float t10 = q1*q2;
		float t2 = t9+t10;
		float t3 = q0*q0;
		float t4 = q1*q1;
		float t5 = q2*q2;
		float t6 = q3*q3;
		float t7 = t3+t4-t5-t6;
		float t8 = t7*t7;
		if (t8 > 1e-6f) {
				t8 = 1.0f/t8;
		} else {
				return;
		}
		float t11 = t2*t2;
		float t12 = t8*t11*4.0f;
		float t13 = t12+1.0f;
		float t14;
		if (fabsf(t13) > 1e-6f) {
				t14 = 1.0f/t13;
		} else {
				return;
		}

		H_YAW[0] = t8*t14*(q3*t3-q3*t4+q3*t5+q3*t6+q0*q1*q2*2.0f)*-2.0f;
		H_YAW[1] = t8*t14*(-q2*t3+q2*t4+q2*t5+q2*t6+q0*q1*q3*2.0f)*-2.0f;
		H_YAW[2] = t8*t14*(q1*t3+q1*t4+q1*t5-q1*t6+q0*q2*q3*2.0f)*2.0f;
		H_YAW[3] = t8*t14*(q0*t3+q0*t4-q0*t5+q0*t6+q1*q2*q3*2.0f)*2.0f;

		// Get the 321 euler angles
		float euler321[3];
		float quat_temp[4];
		for(u8 i=0;i<4;i++){
			quat_temp[i] = ele->state[i];
		}
		Quaternion2Euler(quat_temp,euler321);
		predicted_yaw = euler321[2];

		// set the yaw to zero and calculate the zero yaw rotation from body to earth frame
		rot_from_euler(euler321[0], euler321[1], 0.0f,Tbn_zeroYaw);

	} else {
			// calculate observation jacobian when we are observing a rotation in a 312 sequence
		float t9 = q0*q3;
		float t10 = q1*q2;
		float t2 = t9-t10;
		float t3 = q0*q0;
		float t4 = q1*q1;
		float t5 = q2*q2;
		float t6 = q3*q3;
		float t7 = t3-t4+t5-t6;
		float t8 = t7*t7;
		if (t8 > 1e-6f) {
				t8 = 1.0f/t8;
		} else {
				return;
		}
		float t11 = t2*t2;
		float t12 = t8*t11*4.0f;
		float t13 = t12+1.0f;
		float t14;
		if (fabsf(t13) > 1e-6f) {
				t14 = 1.0f/t13;
		} else {
				return;
		}

		H_YAW[0] = t8*t14*(q3*t3+q3*t4-q3*t5+q3*t6-q0*q1*q2*2.0f)*-2.0f;
		H_YAW[1] = t8*t14*(q2*t3+q2*t4+q2*t5-q2*t6-q0*q1*q3*2.0f)*-2.0f;
		H_YAW[2] = t8*t14*(-q1*t3+q1*t4+q1*t5+q1*t6-q0*q2*q3*2.0f)*2.0f;
		H_YAW[3] = t8*t14*(q0*t3-q0*t4+q0*t5+q0*t6-q1*q2*q3*2.0f)*2.0f;

		// Get the 321 euler angles
		float euler312[3];
		float quat_temp[4];
		for(u8 i=0;i<4;i++){
			quat_temp[i] = ele->state[i];
		}
		float mat_temp[3][3];
		rotation_matrix(quat_temp,mat_temp);
		to_euler312(mat_temp,euler312);
		predicted_yaw = euler312[2];

		// set the yaw to zero and calculate the zero yaw rotation from body to earth frame
		from_euler312(euler312[0], euler312[1], 0.0f,Tbn_zeroYaw);
	}
	float magMeasNED[3];
	MatrixMultiVector(Tbn_zeroYaw,ele->magDataDelayed.mag,magMeasNED);
	float measured_yaw;
	measured_yaw = (-atan2f(magMeasNED[1], magMeasNED[0])) + gps->yaw_declination*D2R;
	if(measured_yaw>PI) measured_yaw-=2*PI;
	else if(measured_yaw<-PI) measured_yaw+=2*PI;
	
	float innovation = predicted_yaw - measured_yaw;
	if(innovation>PI) innovation-=2*PI;
	else if(innovation<-PI) innovation+=2*PI;
	
	float PH[4];
	float varInnov = (R_YAW_HEADING)*(R_YAW_HEADING);
	for (u8 rowIndex=0; rowIndex<=3; rowIndex++) {
			PH[rowIndex] = 0.0f;
			for (u8 colIndex=0; colIndex<=3; colIndex++) {
					PH[rowIndex] += P[rowIndex][colIndex]*H_YAW[colIndex];
			}
			varInnov += H_YAW[rowIndex]*PH[rowIndex];
	}
	float varInnovInv;
	varInnovInv = 1.0f / varInnov;
	
	float Kfusion[16];
	for (u8 rowIndex=0; rowIndex<=ele->stateIndexLim; rowIndex++) {
		Kfusion[rowIndex] = 0.0f;
		for (u8 colIndex=0; colIndex<=3; colIndex++) {
			Kfusion[rowIndex] += P[rowIndex][colIndex]*H_YAW[colIndex];
		}
		Kfusion[rowIndex] *= varInnovInv;
	}
	ele->yawTestRatio = sq(innovation) / (sq(ele->yawInnovGate) * varInnov);
	if(ele->yawTestRatio>1.0f)
		return;
	
	for (uint8_t row = 0; row <= ele->stateIndexLim; row++) {
		for (uint8_t column = 0; column <= 3; column++) {
				KH[row][column] = Kfusion[row] * H_YAW[column];
		}
	}
	for (uint8_t row = 0; row <= ele->stateIndexLim; row++) {
		for (uint8_t column = 0; column <= ele->stateIndexLim; column++) {
			 float tmp = KH[row][0] * P[0][column];
			 tmp += KH[row][1] * P[1][column];
			 tmp += KH[row][2] * P[2][column];
			 tmp += KH[row][3] * P[3][column];
			 KHP[row][column] = tmp;
			}
	}
	
	for (uint8_t i= 0; i<=ele->stateIndexLim; i++) {
		for (uint8_t j= 0; j<=ele->stateIndexLim; j++) {
			P[i][j] = P[i][j] - KHP[i][j];
		}
	}
	//强制对称
	ForceSymmetry();
	//状态修正
	for (u8 j= 0; j<=ele->stateIndexLim; j++) {
		ele->state[j] = ele->state[j] - Kfusion[j] * innovation;
	}
	float quat_temp[4];
	for(u8 i = 0;i<4;i++){
		quat_temp[i] = ele->state[i];
	}
	QuaternionNormalise(quat_temp);
	for(u8 i = 0;i<4;i++){
		ele->state[i] = quat_temp[i];
	}
}

void SelectMagFusion(sEKF *ele)
{
	ele->magFusePerformed = false;//当前是否融合磁力计标志位至0
	readMAGData(ele);//读取磁力计数据
	bool magDataToFuse = Recall_Mag_data(&storeMAG,ele->imuDataDelayed.time_ms,ele);
	
	if(magDataToFuse){
//		uint32_t mch1[100];
//		static int count1;
//		if(count1<100)
//		{
//			mch1[count1]=ele->imuDataDelayed.time_ms;
//			count1++;
//		}
		controlMagYawReset(ele);
		if(ele->inhibitMagStates){
			fuseEulerYaw(ele);
		}
		else{
			FuseMagnetometer(ele);
			//FuseDeclination(ele);
		}
	}
	
	if(!ele->magFieldLearned && ele->finalInflightMagInit) {
    ele->magFieldLearned = (P[16][16] < sq(0.01f)) && (P[17][17] < sq(0.01f)) && (P[18][18] < sq(0.01f));
  }
	if(ele->magFieldLearned && !ele->inhibitMagStates) {
    ele->earthMagFieldVar[0] = P[16][16];
		ele->earthMagFieldVar[1] = P[17][17];
		ele->earthMagFieldVar[2] = P[18][18];
		ele->bodyMagFieldVar[0] = P[19][19];
		ele->bodyMagFieldVar[1] = P[20][20];
		ele->bodyMagFieldVar[2] = P[21][21];
  }
}
void FuseVelPosNED(sEKF *ele)
{
	bool fuseData[6] = {false,false,false,false,false,false};
	if(ele->fuseVelData ||ele->fusePosData || ele->fuseHgtData){
		float observation[6];
		observation[0] = ele->gpsDataDelayed.vel[0];
		observation[1] = ele->gpsDataDelayed.vel[1];
		observation[2] = ele->gpsDataDelayed.vel[2];
		observation[3] = ele->gpsDataDelayed.pos[0];
		observation[4] = ele->gpsDataDelayed.pos[1];
		observation[5] = ele->gpsDataDelayed.pos[2];
		
		float R_OBS[6];
		R_OBS[0] = ele->gpsDataDelayed.vel_accurancy * ele->gpsDataDelayed.vel_accurancy;
		R_OBS[1] = R_OBS[0];
		R_OBS[2] = R_OBS[0];
		R_OBS[3] = ele->gpsDataDelayed.pos_accurancy * ele->gpsDataDelayed.pos_accurancy;
		R_OBS[4] = R_OBS[3];
		R_OBS[5] = ele->gpsDataDelayed.hei_accuracy  * ele->gpsDataDelayed.hei_accuracy;
		
		if (ele->fusePosData) {
			innovVelPos[3] = ele->state[7] - observation[3];		
			innovVelPos[4] = ele->state[8] - observation[4];			
			varInnovVelPos[3] = P[7][7] + R_OBS[3];
      varInnovVelPos[4] = P[8][8] + R_OBS[4];
			ele->posTestRatio = ((innovVelPos[3]*innovVelPos[3])+(innovVelPos[4]*innovVelPos[4]))/
			               (ele->gpsposGate * ele->gpsposGate * (varInnovVelPos[3]+varInnovVelPos[4]));
			if (ele->PV_AidingMode == NONE)
			{
				fuseData[3] = true;
				fuseData[4] = true;
				ele->lastPosPassTime = ele->imuSampleTime_ms;
				
			}
			//卡方检测
			else if(ele->posTestRatio < 1.0f)
			{
				fuseData[3] = true;
				fuseData[4] = true;
				ele->lastPosPassTime = ele->imuSampleTime_ms;

			}
		  
		}
		if(ele->fuseVelData){
			innovVelPos[0] = ele->state[4] - observation[0];			
			innovVelPos[1] = ele->state[5] - observation[1];
			innovVelPos[2] = ele->state[6] - observation[2];
			
			varInnovVelPos[0] = P[4][4] + R_OBS[0];
      varInnovVelPos[1] = P[5][5] + R_OBS[1];
      varInnovVelPos[2] = P[6][6] + R_OBS[2];
			ele->velTestRatio = (innovVelPos[0]*innovVelPos[0]+innovVelPos[1]*innovVelPos[1]+innovVelPos[2]*innovVelPos[2])/
			               (ele->gpsvelGate * ele->gpsvelGate * (varInnovVelPos[0]+varInnovVelPos[1]+varInnovVelPos[2]));
			//卡方检测
			if(ele->velTestRatio < 1.0f)
			{ fuseData[0] = true;	
				fuseData[1] = true;	
				fuseData[2] = true;
				ele->lastVelPassTime = ele->imuSampleTime_ms;
				
			}				
		}
		if(ele->fuseHgtData){
			innovVelPos[5] = ele->state[9] - observation[5];
			
			varInnovVelPos[5] = P[9][9] + R_OBS[5];
			ele->HgtTestRatio = (innovVelPos[5]*innovVelPos[5])/
			               (ele->gpsheiGate * ele->gpsheiGate * varInnovVelPos[5]);
			//卡方检测
			if(ele->HgtTestRatio < 1.0f)
			{
				fuseData[5] = true;
 				ele->lastHgtPassTime = ele->imuSampleTime_ms;
			}
		}
		
		
		float SK;
		float Kfusion[22];
		for (u8 obsIndex=0; obsIndex<=5; obsIndex++) {
			if (fuseData[obsIndex]) {
				u8 stateIndex = 4 + obsIndex;
				varInnovVelPos[obsIndex] = P[stateIndex][stateIndex] + R_OBS[obsIndex];
				SK = 1.0f/varInnovVelPos[obsIndex];
				
				for(u8 i = 0;i <= 21;i++){
					Kfusion[i] = P[i][stateIndex]*SK;
				}
				
				for (uint8_t i= 0; i<=21; i++) {
					for (uint8_t j= 0; j<=21; j++)
					{
						KHP[i][j] = Kfusion[i] * P[stateIndex][j];
					}
				}
				
				for (u8 i= 0; i<=21; i++) {
					for (u8 j= 0; j<=21; j++) {
						P[i][j] = P[i][j] - KHP[i][j];
					}
				}
				
				ForceSymmetry();
				
				for (u8 i = 0; i<=ele->stateIndexLim; i++) {
					ele->state[i] = ele->state[i] - Kfusion[i] * innovVelPos[obsIndex];
				}
				float quat_temp[4];
				for(u8 i = 0;i<4;i++){
					quat_temp[i] = ele->state[i];
				}
				QuaternionNormalise(quat_temp);
				for(u8 i = 0;i<4;i++){
					ele->state[i] = quat_temp[i];
				}
			}
		}
	}
}

void SelectVelPosFusion(sEKF *ele)
{
	if(!ele->posVelFusionDelayed && ele->magFusePerformed){
		ele->posVelFusionDelayed = true;//如果当前循环已经融合了磁力计，则GPS融合则推迟到下一时刻融合，避免超时
		return;
	}else{
		ele->posVelFusionDelayed = false;
	}
	readGPSData(ele);
	
	ele->gpsDataToFuse = Recall_Gps_data(&storeGPS,ele->imuDataDelayed.time_ms,ele);
/* 11.8 */
	if(ele->gpsDataToFuse && ele->PV_AidingMode == ABSOLUTE)
	//判断是否融合速度和位置
	{
		//杠杆效应
		//		float posOffsetBody[3] = {-0.4,0,-0.15};//GPS相对IMU位置偏移量，视大小飞机而定
		float angRate[3];//角速度
		for(u8 i = 0;i<3;i++){
			angRate[i] = ele->imuDataDelayed.delAng[i]/ele->imuDataDelayed.delAngDT;
		}
		crossMulti3X3(angRate,posOffsetBody);
		float vel_correct[3];
//		uint32_t mch[100];
//		static int count1;
//		if(count1<100)
//		{
//			mch[count1]=ele->imuDataDelayed.time_ms;
//			count1++;
//		}
//		else
//		{
//			count1=0;
//		}
		MatrixMultiVector(ele->prevTbn,angRate,vel_correct);//得到NED坐标系下的速度补偿
		for(u8 i = 0; i<3;i++){
			ele->gpsDataDelayed.vel[i] -= vel_correct[i];
		}
		float pos_correct[3];
		MatrixMultiVector(ele->prevTbn,posOffsetBody,pos_correct);//得到NED坐标系下的位置补偿
		for(u8 i = 0;i<2;i++){
			ele->gpsDataDelayed.pos[i] -= pos_correct[i];
		}
		ele->fuseVelData = true;//位置融合标志位置1
		ele->fusePosData = true;//速度融合标志位置1
	}
	else
	{
		ele->fusePosData = false;
		ele->fuseVelData = false;
	}
	//选择高度值
	selectHeightForFusion(ele);



	if(ele->fuseHgtData && ele->PV_AidingMode == NONE)
	{
	  ele->gpsDataDelayed.vel[0] = 0.0f;
		ele->gpsDataDelayed.vel[1] = 0.0f;
		ele->gpsDataDelayed.vel[2] = 0.0f;
		ele->gpsDataDelayed.pos[0] = ele->lastKnowPosition[0];
    ele->gpsDataDelayed.pos[1] = ele->lastKnowPosition[1];
		ele->fuseVelData = false;
		ele->fusePosData = true;
		
	}
	if (ele->fuseVelData || ele->fusePosData || ele->fuseHgtData) {
		FuseVelPosNED(ele);//融合位置速度
		// clear the flags to prevent repeated fusion of the same data
		ele->fuseVelData = false;
		ele->fuseHgtData = false;
		ele->fusePosData = false;
	}
}

void calcOutputStates(sEKF *ele)
{
	
	for(u8 i = 0;i < 3;i++){
		ele->imuDataNew.delAng[i] -= ele->state[i+10]*ele->imuDataNew.delAngDT/EKF_TARGET_DT;//偏置做单位转换再修正IMU数据
		ele->imuDataNew.delVel[i] -= ele->state[i+13]*ele->imuDataNew.delVelDT/EKF_TARGET_DT;		
	}
	
	for(u8 i = 0;i < 3;i++){
		ele->imuDataNew.delAng[i]+= ele->delAngCorrection[i];
	}
	float Tbn_temp[3][3];
	rotation_matrix(ele->outputDataNew.quat,Tbn_temp);
	float delQuat[4];
	from_axis_angle(ele->imuDataNew.delAng,delQuat);
	QuaternionMulti(ele->outputDataNew.quat,delQuat);
	QuaternionNormalise(ele->outputDataNew.quat);
	
	float delVelNav[3];
	MatrixMultiVector(Tbn_temp,ele->imuDataNew.delVel,delVelNav);
	float lastVel[3];	
	for(u8 i=0; i < 3;i++){
		lastVel[i] = ele->outputDataNew.vel[i];
	}
	for(u8 i = 0;i<3;i++){
		ele->outputDataNew.vel[i] += delVelNav[i];
	}
	ele->outputDataNew.vel[2]+= GRAVITY_MSS * ele->imuDataNew.delVelDT;
	
	for(u8 i = 0;i<3;i++){
		ele->outputDataNew.pos[i] += (lastVel[i] + ele->outputDataNew.vel[i]) * 0.5f * ele->imuDataNew.delVelDT;
	}
	if(ele->run_update_filter){
		storeOUTPUT.output_data[storeIMU.youngest] = ele->outputDataNew;

		ele->outputDataDelayed = storeOUTPUT.output_data[storeIMU.oldest];
		
		float quatErr[4];
		float quat_EKF[4];
		for(u8 i = 0;i<4;i++){
			quat_EKF[i] = ele->state[i];
		}
		QuaternionDivision(quat_EKF,ele->outputDataDelayed.quat,quatErr);
		QuaternionNormalise(quatErr);

		float scaler;
		if (quatErr[0] >= 0.0f) {
			scaler = 2.0f;
		} else {
			scaler = -2.0f;
		}
		deltaAngErr[0] = scaler * quatErr[1];
		deltaAngErr[1] = scaler * quatErr[2];
		deltaAngErr[2] = scaler * quatErr[3];
		
		float timeDelay = 1e-3f * (float)(ele->imuDataNew.time_ms - ele->imuDataDelayed.time_ms);
    float errorGain = 0.5f / timeDelay;
		for(u8 i=0;i<3;i++){
			ele->delAngCorrection[i] = errorGain * 0.01f * deltaAngErr[i];
		}
		
		float velErr[3];
		static float velErr_Inter[3] = {0.0f,0.0f,0.0f};
		static float posErr_Inter[3] = {0.0f,0.0f,0.0f};
		for(u8 i = 0;i<3;i++){
			velErr[i] = ele->state[4+i] - ele->outputDataDelayed.vel[i];//误差
			velErr_Inter[i] += velErr[i];//误差积分
		}
		float posErr[3];
		for(u8 i = 0;i<3;i++){
			posErr[i] = ele->state[7+i] - ele->outputDataDelayed.pos[i];
			posErr_Inter[i] += posErr[i];
		}
		
		float tauPosVel = 0.25f;//原0.25f;
		float velPosGain = 0.1f / tauPosVel;//..原为0.01
		float velCorrection[3];
		for(u8 i = 0;i<3;i++){
			velCorrection[i] = velErr[i] * velPosGain + velErr_Inter[i] * (velPosGain) * (velPosGain) * 0.1f;
		}
		float posCorrection[3];
		for(u8 i = 0;i<3;i++){
			posCorrection[i] = posErr[i] * velPosGain + posErr_Inter[i] * (velPosGain) * (velPosGain) * 0.1f;
		}
		output_elements outputStates;
		for (u8 index=0; index < IMU_BUFFER_SIZE; index++) {
			outputStates = storeOUTPUT.output_data[index];

			// a constant  velocity correction is applied
			for(u8 i=0;i<3;i++){
				outputStates.vel[i]  += velCorrection[i];
			}

			// a constant position correction is applied
			for(u8 i=0;i<3;i++){
				outputStates.pos[i] += posCorrection[i];
			}

			// push the updated data to the buffer
			storeOUTPUT.output_data[index] = outputStates;
		}
		ele->outputDataNew = storeOUTPUT.output_data[storeIMU.youngest];
	}
}

void setAidingMode(sEKF *ele)
{
	ele->PV_AidingModePrev = ele->PV_AidingMode;
	
	//判断是否切换援助模式
	switch(ele->PV_AidingMode)
	{
		case NONE:
		{
		    if(readyToUseGPS(ele))
					ele->PV_AidingMode = ABSOLUTE;
				break;
		}
		case ABSOLUTE:
		{   
			  int minTesttime_ms = 7000 ;
			  bool gpsPosUsed = (ele->imuSampleTime_ms - ele->lastPosPassTime <=  minTesttime_ms);
			  bool gpsVelUsed = (ele->imuSampleTime_ms - ele->lastVelPassTime <= minTesttime_ms);
			  bool attAiding = gpsPosUsed || gpsVelUsed ;
			  bool attAidlossCritical = false;
			if(!attAiding)
			{
				attAidlossCritical = (ele->imuSampleTime_ms - ele->lastPosPassTime > ele->tiltDriftTimeMax_ms)
				                   &&(ele->imuSampleTime_ms - ele->lastVelPassTime > ele->tiltDriftTimeMax_ms);
								
			}
			if(attAidlossCritical)
			{
				ele->PV_AidingMode = NONE;
				ele->posTimeout = true;
				ele->velTimeout = true;
				ele->gpsNOtAvailable = true;
				
			}
		   break;
		}
	}
	if(ele->PV_AidingMode != ele->PV_AidingModePrev)
	{
		switch(ele->PV_AidingMode)
		{
			case NONE:
			{
				ele->posTimeout = true ;
				ele->velTimeout = true;
				ele->velTestRatio = 0.0f;
				ele->posTestRatio = 0.0f;
				ele->lastKnowPosition[0] = ele->state[7];
				ele->lastKnowPosition[1] = ele->state[8];
				
				break;
			}
			case ABSOLUTE:
				
					ele->posTimeout = false;
					ele->velTimeout = false;
			    ele->lastPosPassTime = ele->imuSampleTime_ms;
			    ele->lastVelPassTime = ele->imuSampleTime_ms ;
			break;
					
				 
				
		}
		//重新更新速度位置状态与方差
	ResetVelocity(ele);
	ResetPosition(ele);
	}
}

void ResetVelocity(sEKF *ele)
{
	for(u8 row = 4 ;row <=5; row++)
	{
		for(u8 col = 0 ;col <=22 ; col++)
		    P[row][col] = 0 ;
	}
	for(u8 col = 4 ;col <=5; col++)
	{
		for(u8 row = 0 ;row <=22 ; row++)
		    P[row][col] = 0 ;
	}
	//若gps不能用，速度置零
	if(ele->PV_AidingMode == NONE)
	{
		ele->state[4] = 0;
		ele->state[5] = 0;
		P[4][4] = P[5][5] = gps->gpsSpdAccuracy * gps->gpsSpdAccuracy;
				
	}
	else //gps可用，则用新的gps数据初始化状态值
	{
		ele->state[4] = ele->gpsDataNew.vel[0];
		ele->state[5] = ele->gpsDataNew.vel[1];
		P[4][4] = P[5][5] = gps->gpsSpdAccuracy * gps->gpsSpdAccuracy;
    ele->velTimeout = false;
		ele->lastVelPassTime = ele->imuSampleTime_ms;
		
	}
	for(u8 i=0 ; i< IMU_BUFFER_SIZE;i++)
	{
		storeOUTPUT.output_data[i].vel[0] = ele->state[4];
	  storeOUTPUT.output_data[i].vel[1] = ele->state[5];
	}
	ele->outputDataNew.vel[0] = ele->state[4];
	ele->outputDataNew.vel[1] = ele->state[5];
	ele->outputDataDelayed.vel[0] = ele->state[4];
	ele->outputDataDelayed.vel[1] = ele->state[5];
	
	ele->lastVelReset_ms = ele->imuSampleTime_ms;
	
}
void ResetPosition(sEKF *ele)
{
	for(u8 row = 7 ;row <=8; row++)
	{
		for(u8 col = 0 ;col < 22 ; col++)
		    P[row][col] = 0 ;
	}
	for(u8 col = 7 ;col <=8; col++)
	{
		for(u8 row = 0 ;row < 22 ; row++)
		    P[row][col] = 0 ;
	}
	if(ele->PV_AidingMode == NONE)
	{
		ele->state[7] = ele->lastKnowPosition[0];
		ele->state[8] = ele->lastKnowPosition[1];
	  P[7][7] = P[8][8] = gps->gpsPosAccuracy * gps->gpsPosAccuracy;
	}
	else  //useGPS
	{
		ele->state[7] = ele->gpsDataNew.pos[0];
		ele->state[8] = ele->gpsDataNew.pos[1];
		P[7][7] = P[8][8] = gps->gpsPosAccuracy * gps->gpsPosAccuracy;
    ele->posTimeout = false;
		ele->lastPosPassTime = ele->imuSampleTime_ms;
	}
		for(u8 i=0 ; i< IMU_BUFFER_SIZE;i++)
	{
		storeOUTPUT.output_data[i].pos[0] = ele->state[7];
	  storeOUTPUT.output_data[i].pos[1] = ele->state[8];
	}
	ele->outputDataNew.pos[0] = ele->state[7];
	ele->outputDataNew.pos[1] = ele->state[8];
	ele->outputDataDelayed.pos[0] = ele->state[7];
	ele->outputDataDelayed.pos[1] = ele->state[8];
	
	ele->lastPosReset_ms = ele->imuSampleTime_ms;
	
}


bool readyToUseGPS(sEKF *ele) 
{
    return gps->ECEF_Init_Flag && ele->gpsGoodToAlign && ele->gpsDataToFuse ;
}

bool calcGpsGoodToAlign(sEKF *ele)
{
	 bool gpsSpdAccFail = (gps->gpsSpdAccuracy > 2.0f) ;
	 bool hdopFail = false ;
	 bool numSatsFail = (gps->star < 6);  //
	if(ele->lastGpsVelFail_ms == 0)
	{
		ele->lastGpsVelFail_ms = ele->imuSampleTime_ms;
	}
	if(gpsSpdAccFail || numSatsFail || hdopFail)
     ele->lastGpsVelFail_ms = ele->imuSampleTime_ms;
	if(ele->imuSampleTime_ms  - ele->lastGpsVelFail_ms > 5000)
		return true ;
	return false;
}

void selectHeightForFusion(sEKF  *ele)
{
	readLASERData(ele);
	ele->laserDataToFuse = Recall_Laser_data(&storeLASER,ele->imuDataDelayed.time_ms,ele);
	readBAROData(ele);//读取气压计数据
	ele->baroDataToFuse = Recall_Baro_data(&storeBARO,ele->imuDataDelayed.time_ms,ele);
	
	//选择高度传感器
	if (altSource == 0)
		ele->activeHgtSource = HGT_SOURCE_BARO;     //气压计
	else ele->activeHgtSource =  HGT_SOURCE_LASER;   //激光
	
	//若激光出问题用气压计替代
	
	bool  lostLaser = ((ele->activeHgtSource == HGT_SOURCE_LASER)&&((ele->imuSampleTime_ms - ele->rngValidMeaTime_ms) > 500));
	if(lostLaser) 		ele->activeHgtSource = HGT_SOURCE_BARO;     //气压计
  if(ele->laserDataToFuse&&ele->activeHgtSource == HGT_SOURCE_LASER)
		
	{
		ele->fuseHgtData = true;
		ele->hgtMea = ele->laserDataDelayed.hgt ;
		ele->posDownObsNoise = LASER_R*LASER_R;
	}
	if(ele->baroDataToFuse&&ele->activeHgtSource == HGT_SOURCE_BARO)
		
	{
		ele->fuseHgtData = true;
		ele->hgtMea = ele->baroDataDelayed.hgt ;
		ele->posDownObsNoise = BARO_R*BARO_R;
	}
	
	

}
