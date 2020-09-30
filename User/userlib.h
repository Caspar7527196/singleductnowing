#ifndef __USERLIB_H
#define __USERLIB_H

#include "stm32f7xx_hal.h"
#include "stm32f7xx.h"
#include "arm_math.h"
#include "stdio.h"
#include "stdbool.h"
#include "stdlib.h"
#include "stdarg.h"
#include "string.h"
#include "main.h"

#ifdef __cplusplus
 extern "C" {
#endif 

//datatype define
typedef int64_t s64;
typedef int32_t s32;
typedef int16_t s16;
typedef int8_t  s8;
typedef const int64_t sc64;  /*!< Read Only */
typedef const int32_t sc32;  /*!< Read Only */
typedef const int16_t sc16;  /*!< Read Only */
typedef const int8_t sc8;   /*!< Read Only */
typedef __IO int64_t  vs64;
typedef __IO int32_t  vs32;
typedef __IO int16_t  vs16;
typedef __IO int8_t   vs8;
typedef __I int64_t vsc64;  /*!< Read Only */
typedef __I int32_t vsc32;  /*!< Read Only */
typedef __I int16_t vsc16;  /*!< Read Only */
typedef __I int8_t vsc8;   /*!< Read Only */
typedef uint64_t u64;
typedef uint32_t u32;
typedef uint16_t u16;
typedef uint8_t  u8;
typedef const uint64_t uc64;  /*!< Read Only */
typedef const uint32_t uc32;  /*!< Read Only */
typedef const uint16_t uc16;  /*!< Read Only */
typedef const uint8_t uc8;   /*!< Read Only */
typedef __IO uint64_t vu64;
typedef __IO uint32_t vu32;
typedef __IO uint16_t vu16;
typedef __IO uint8_t  vu8;
typedef __I uint64_t vuc64;  /*!< Read Only */
typedef __I uint32_t vuc32;  /*!< Read Only */
typedef __I uint16_t vuc16;  /*!< Read Only */
typedef __I uint8_t vuc8;   /*!< Read Only */

//常用定义
#define D2R  (PI / 180.0f)
#define D2R_D 0.017453292519943295
#define R2D  (180.0f / PI)
#define R2D_D 57.295779513082323
#define SQR(x)  ((x)*(x))
#define absf(x) (x>0.0f?x:-x)
#define MAX(x,y) (x>y?x:y)
#define MIN(x,y) (x<y?x:y)

//数据拆分宏定义
#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)    ) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )

typedef union 		//16位联合体
{
	u8   uc[2];
	s8   sc[2];
	u16  us;
	s16  ss;
}BIT16;

typedef union 		//32位联合体
{
	u8   uc[4];
	s8   sc[4];
	u16  us[2];
	s16  ss[2];
	u32  ui;
	s32  si;
	float fp;
}BIT32;

typedef struct      //计时器，计算运行时间
{
	u32 CNT;        //计数值
	u32 OUT;        //输出值
}sTIM;

typedef struct      //计数器，滤波等使用
{
	s16 CNT;        //当前计数值
	s16 CCR;        //比较值
}sCNT;

typedef enum {NED=0, SWD=1, NWU=2, SEU=3}eLLS; 
typedef enum {XAXIS=0,YAXIS=1,ZAXIS=2}eAXIS;
typedef enum {ROL=0,PIT=1,YAW=2}eAXLE;

typedef enum            //错误类型
{
	ERR_NONE=0x00U,       //正常
	ERR_SOFT=0x01U,       //软件错误
	ERR_HARD=0x02U,       //硬件错误
	ERR_LOST=0x03U,       //对象不存在
	ERR_UPDT=0x04U,       //周期内未更新或者数据无效
}eERR;

typedef enum            //传感器或控制器状态
{
	STA_INI=0x01U,        //初始化参数，配置等
	STA_CHK=0x02U,        //自检状态，检测自身存在
	STA_CAL=0x03U,        //校准状态，校准参数
	STA_RUN=0x04U,        //运行状态，读取数据或执行控制
	STA_TST=0x05U,        //测试状态，测试
}eSTA;

/*********************修改部分**********************/
#define UART_DEBUG_EN 1        //调试信息

#define TIM_COUNT htim2
#define UART_DEBUG huart2
#define UART_SEND huart8
#define UART_CACHE_NUM 300
#define I2C_TIMEOUT 2
#define UART_TIMEOUT 20

#define ON  0x00U
#define OFF 0x01U

#define Led_Tog(n) HAL_GPIO_TogglePin(GPIOE,0x02<<n)
#define Led_Set(n) HAL_GPIO_WritePin(GPIOE,0x02<<n,GPIO_PIN_RESET)
#define Led_Clr(n) HAL_GPIO_WritePin(GPIOE,0x02<<n,GPIO_PIN_SET)

extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;



extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
extern SPI_HandleTypeDef hspi4;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim7;

extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart8;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_uart4_rx;
extern DMA_HandleTypeDef hdma_uart4_tx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_uart8_rx;
extern DMA_HandleTypeDef hdma_uart8_tx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;
/*********************修改部分**********************/

void DMA_CLEAR_FLAG_ALL(DMA_HandleTypeDef *dmax);
void Sys_Rst(void);
void Tim_Calc(sTIM *timx);
void User_Delay(u32 nus);
void Dprintf(const char * _format,...) __attribute__((format(printf,1,2)));
void Sprintf(const char * _format,...) __attribute__((format(printf,1,2)));
bool Dir_Trans(s8 Dir[6],const char DirChr[6]);
float Sign(float value);
float invSqrt(float x);
float Norm3(float *data);
float fConstrain(float Input, float minValue, float maxValue);
s16 iConstrain(s16 Input, s16 minValue, s16 maxValue);
float LoopConstrain(float Input, float minValue, float maxValue);
float Rad_Format(float Ang);
float Theta_Format(float Ang);
void SlideFilt(float *Dat,float *DatRel,u8 num,sCNT *Filt,u8 Cmd);
u8 StrSeg(const char *str,char chr,char *para[],u8 num);
bool DataCheck(bool same,sCNT *Check);
void LineFit(float* Y,u16 N,float *ab);
void circleFit(float d[][2],u16 N,float circleOrigin[],float *circleRadius);
void sphereFit(float d[][3],u16 N,u16 MaxIterations,float Err,u16 Population[][3],float SphereOrigin[],float *SphereRadius);
double norm(const double x[3]);
//add by wqz
//---------------mch----------------------
float sq(float val);
void crossMulti3X3(float a[3],float b[3]);
void from_axis_angle(float v[3],float q[4]);
void to_axis_angle(float q[4],float v[3]);
void QuaternionMulti(float q[4],float v[4]);
void QuaternionNormalise(float v[4]);
void rotation_matrix(float q[4],float matric[3][3]);
void MatrixMultiVector(float m[3][3],float v[3],float result[3]);

void QuaternionReverse(float q[4]);
void Quaternion2Euler(float q[4],float euler[3]);
void QuaternionDivision(float quat[4],float v[4],float result[4]);
void rot_from_euler(float roll, float pitch, float yaw,float rot_mat[3][3]);
void to_euler312(float mat[3][3],float euler[3]);
void from_euler312(float roll, float pitch, float yaw,float rot_mat[3][3]);
void PIto2PI(float *theta);
void euler2quaternion(float euler[3],float q[4]);
void angle_axis_of_rotation(double theta[3],double *phi, double u[3]);
void creat_quaternion(double *phi, double u[3],double q[4]);
void creat_rotation_matrix(double *phi, double u[3],double Rot_element[][3]);
void quaternion_mul(double q1[4], double q2[4], double q[4]);
void matrix_inverse_LU(int N, double A[][N],double A_inverse[][N]);
void cross_product_matrix(double u[3],double u_x_element[][3]);
float Abs(float input);

#ifdef __cplusplus
}
#endif

#endif
