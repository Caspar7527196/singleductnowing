#ifndef __EKF_H_
#define __EKF_H_
#include "userlib.h"
#include "icm20602.h"
#include "ak8975.h"
#include "ahrs.h"
#include "exit.h"

//高度使用传感器
#define altSource  0   //0  baro   1laser  2gps
#define HGT_SOURCE_BARO  0
#define HGT_SOURCE_LASER  1

#define EKF_MAG_FINAL_RESET_ALT 3.5f
#define MIN_INTERVAL_SENSOR_MS 50 
#define MAG_DELAY_MS 5
#define BARO_DELAY_MS 60
#define GPS_DELAY_MS 140
#define EKF_TARGET_DT 0.02f
#define IMU_AVR_DT 0.01f
#define IMU_BUFFER_SIZE 13
#define OBS_BUFFER_SIZE 7
#define LASER_BUFFER_SIZE 7

#define GRAVITY_MSS 9.788f

//初始P矩阵协方差
#define GPS_VEL_VAR 0.2f
#define GPS_POSH_VAR 2.0f
#define GPS_BRAO_VAR 0.4f
#define GPS_LASER_VAR 0.4f
#define GYROBIASUNCERTAINTY 0.005f
#define ACCBIAS_VAR 0.02f 
#define MAG_NED_FIELD_VAR 0.005f
#define MAG_XYZ_BIAS_VAR  0.001f
//过程噪声
//#define GYRO_BIAS_PROCESS_NOISE 0.001f
//#define ACC_BIAS_PROCESS_NOISE 1.0f
//#define GYRO_NOISE 0.01f				
//#define ACC_NOISE 1.5f	
//#define MAG_EARTH_PROCESS_NOISE 0.001f
//#define MAG_BODY_PROCESS_NOISE 0.0001f
////修改
//#define GYRO_BIAS_PROCESS_NOISE 0.006f
//#define ACC_BIAS_PROCESS_NOISE 0.08f
//#define GYRO_NOISE 0.2f
//#define ACC_NOISE 0.4f
//#define MAG_EARTH_PROCESS_NOISE 0.001f
//#define MAG_BODY_PROCESS_NOISE 0.0001f	

//观测噪声
#define MAG_NED_R 0.05f
#define BARO_R 0.25f  //激光0.05   气压计 0.3
#define LASER_R  0.09f
#define R_DECL 0.01f
#define R_YAW_HEADING 0.05f
typedef struct  {
  float    delAng[3];         // 0..2
  float    delVel[3];         // 3..5
  float    delAngDT;       // 6
  float    delVelDT;       // 7
  uint32_t    time_ms;        // 8
}imu_elements;

typedef struct
{
	u8  youngest;
	u8  oldest;
	bool is_filled;
  imu_elements imu_data[IMU_BUFFER_SIZE];
}STORE_IMU_BUFFER;

typedef struct
{
	float mag[3];
	uint32_t time_ms;
}mag_elements;

typedef struct
{
	u8  youngest;
	u8  oldest;
	bool new_data_flag;
	u32 last_update_time;
  mag_elements mag_data[OBS_BUFFER_SIZE];
}STORE_MAG_BUFFER;

typedef struct
{
	float hgt;
	uint32_t time_ms;
}baro_elements,laser_elements;

typedef struct
{
	u8  youngest;
	u8  oldest;
	bool new_data_flag;
	u32 last_update_time;
  baro_elements baro_data[OBS_BUFFER_SIZE];
}STORE_BARO_BUFFER;

typedef struct
{
	u8  youngest;
	u8  oldest;
	bool new_data_flag;
	u32 last_update_time;
  laser_elements laser_data[LASER_BUFFER_SIZE];
}STORE_LASER_BUFFER;

typedef struct  {
  float    pos[3];         // 0..2
  float    vel[3];         // 3..5
  float    pos_accurancy;       // 6
  float    vel_accurancy;       // 7
	float    hei_accuracy;
  uint32_t    time_ms;        // 8
}gps_elements;

typedef struct
{
	u8  youngest;
	u8  oldest;
	bool new_data_flag;
	u32 last_update_time;
  gps_elements gps_data[OBS_BUFFER_SIZE];
}STORE_GPS_BUFFER;

typedef struct
{
	float quat[4];
	float vel[3];
	float pos[3];
}output_elements;

typedef struct
{
	u8  youngest;
	u8  oldest;
  output_elements output_data[IMU_BUFFER_SIZE];
}STORE_OUTPUT_BUFFER;

typedef enum
{
	NONE = 1,
  ABSOLUTE = 2
}Aid_mode;


typedef struct{
	sTIM Tim;        //计时器
	//EKF标志位
	bool ekf_init_flag;
	bool ekf_start_flag;
	bool run_update_filter;
	bool magFusePerformed;
	bool posVelFusionDelayed;
	bool fuseVelData;
	bool fusePosData;
	bool fuseHgtData;
	bool on_Ground;
	bool in_flight;
	bool finalInflightYawInit;
	bool finalInflightMagInit;
	bool magFieldLearned;
	bool inhibitMagStates;
	//高度R阵
	float posDownObsNoise;
	//更新过程变量
	float posTestRatio;
	float velTestRatio;
	float HgtTestRatio;
	float yawTestRatio;
	float magTestRatio[3];
	
	float magFieldGate;
	float gpsposGate;
	float gpsheiGate;	
	float gpsvelGate;
	float baroHgtGate;
	float yawInnovGate;
	
	//援助模式
	Aid_mode PV_AidingModePrev;
	Aid_mode PV_AidingMode;
	float lastKnowPosition[2];
	

	//EKF输入量
	imu_elements imuDataDelayed;
	imu_elements imuDataNew;
	imu_elements imuDataDownSampledNew;
	u32 imuSampleTime_ms;
	u32 imuPreSampleTime_ms;
	float imuQuatDownSampleNew[4];
	float earthRateNED[3];
	
	//EKF状态量
	float state[22];
	float prevTnb[3][3];//ned2body
	float prevTbn[3][3];//body2ned
	u8 stateIndexLim;
	//EKF观测值
	mag_elements magDataNew;
	mag_elements magDataDelayed;
	u32 lastMagUpdate_ms;
	
	baro_elements baroDataNew;
	baro_elements baroDataDelayed;
  u32 lastBaroUpdate_ms;
	
	laser_elements laserDataNew;
	laser_elements laserDataDelayed;
  u32 lastLaserUpdate_ms;
	
	gps_elements gpsDataNew;
	gps_elements gpsDataDelayed;
	u32 lastGpsUpdate_ms;
//EKF输出量
	float delAngCorrection[3];
	output_elements outputDataNew;
	output_elements outputDataDelayed;
//融合标志位
  bool baroDataToFuse;
	bool laserDataToFuse;
	bool gpsDataToFuse;
	bool gpsGoodToAlign; //判断GPS是否可用于EKF
	//传感器超时标志位
	bool posTimeout;
	bool velTimeout;
	bool gpsNOtAvailable;
	
	//高度值
	u8 activeHgtSource ;
	float hgtMea;
	//gps相关时间
  u32 lastPosPassTime;
	u32 lastVelPassTime;
	u32 lastHgtPassTime;
	
	u32 lastVelReset_ms;
	u32 lastPosReset_ms;
	u32 lastGpsVelFail_ms;
	
	//激光相关时间
	u32 rngValidMeaTime_ms;
	
	
	int tiltDriftTimeMax_ms;
	//磁力计校准变量
	float prevQuatMagReset[4];
	float posDownAtTakeoff;
	float posDownAtLastMagReset;
	float earthMagFieldVar[3];
	float bodyMagFieldVar[3];
}sEKF;

extern sEKF EKF;
extern float P[22][22];
extern float euler_EKF[3];
extern float euler_EKF1[3];
extern float vel_EKF[3];
extern float vel_EKF1[3];
extern float pos_EKF1[3];
extern float pos_EKF[3];
extern float innovVelPos[6];
extern float varInnovVelPos[6];
extern float innov_Mag[3];
extern float varInnovMag[3];
void EKF_Run(sEKF *ele);
void EKF_Init_Para(sEKF *ele);
void EKF_Initialise(sEKF *ele);
void EKF_Update(sEKF *ele);
void StoreOutputReset(sEKF *ele);
void readIMUData(sEKF *ele);
void readMAGData(sEKF *ele);
void readBAROData(sEKF *ele);
void readLASERData(sEKF *ele);
void readGPSData(sEKF *ele);
void UpdateStrapdownEquationNED(sEKF *ele);
void CovariancePrediction(sEKF *ele);
void SelectMagFusion(sEKF *ele);
void SelectVelPosFusion(sEKF *ele);
void Imu_Data_Push(imu_elements data);
void Imu_Data_Pop(imu_elements* data);
bool Recall_Mag_data(STORE_MAG_BUFFER* data,u32 sampleing_time,sEKF *ele);
bool Recall_Baro_data(STORE_BARO_BUFFER* data,u32 sampleing_time,sEKF *ele);
bool Recall_Gps_data(STORE_GPS_BUFFER* data,u32 sampleing_time,sEKF *ele);
void ForceSymmetry(void);
void setAidingMode(sEKF *ele);
void calcOutputStates(sEKF *ele);
void ResetVelocity(sEKF *ele);
void ResetPosition(sEKF *ele);
bool readyToUseGPS(sEKF *ele) ;
bool calcGpsGoodToAlign(sEKF *ele); //判断gps能否用于EKF；
void detectFlight(sEKF *ele);
void FuseDeclination(sEKF *ele);
void alignMagStateDeclination(sEKF *ele);
void selectHeightForFusion(sEKF  *ele);
void initialiseQuatCovariances(sEKF *ele,float rotVarVec[3]);
#endif
