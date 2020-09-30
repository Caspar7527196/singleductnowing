#include "userlib.h"

/***************************************************\
功能：
  延时函数，打印函数，常用函数
说明：
  1、编译器加入预编译ARM_MATH_CM7,__FPU_PRESENT=1
  2、CMSIS里添加DSP库。
  3、根据硬件配置修改h文件标注需要修改的部分。 
\***************************************************/

/******************驱动程序****************/
void DMA_CLEAR_FLAG_ALL(DMA_HandleTypeDef *dmax)
{		
	u32 ele=(u32)dmax->Instance;
  (ele == (u32)DMA1_Stream0)? (DMA1->LIFCR=0x0000003D) :\
	(ele == (u32)DMA1_Stream1)? (DMA1->LIFCR=0x00000F40) :\
	(ele == (u32)DMA1_Stream2)? (DMA1->LIFCR=0x003D0000) :\
	(ele == (u32)DMA1_Stream3)? (DMA1->LIFCR=0x0F400000) :\
	(ele == (u32)DMA1_Stream4)? (DMA1->HIFCR=0x0000003D) :\
	(ele == (u32)DMA1_Stream5)? (DMA1->HIFCR=0x00000F40) :\
	(ele == (u32)DMA1_Stream6)? (DMA1->HIFCR=0x003D0000) :\
	(ele == (u32)DMA1_Stream7)? (DMA1->HIFCR=0x0F400000) :\
	(ele == (u32)DMA2_Stream0)? (DMA2->LIFCR=0x0000003D) :\
	(ele == (u32)DMA2_Stream1)? (DMA2->LIFCR=0x00000F40) :\
	(ele == (u32)DMA2_Stream2)? (DMA2->LIFCR=0x003D0000) :\
	(ele == (u32)DMA2_Stream3)? (DMA2->LIFCR=0x0F400000) :\
	(ele == (u32)DMA2_Stream4)? (DMA2->HIFCR=0x0000003D) :\
	(ele == (u32)DMA2_Stream5)? (DMA2->HIFCR=0x00000F40) :\
	(ele == (u32)DMA2_Stream6)? (DMA2->HIFCR=0x003D0000) :\
	(DMA2->HIFCR=0x0F400000);
}

/******************功能函数****************/
//系统软复位
void Sys_Rst(void)
{   
	SCB->AIRCR =0X05FA0000|(u32)0x04;	  
} 

static bool TIM_FIRST_USE=true;
//计时，调用两次的时间差放在OUT里边，CNT存放上一次数据。
void Tim_Calc(sTIM *timx)
{
	if(TIM_FIRST_USE){HAL_TIM_Base_Start(&TIM_COUNT);TIM_FIRST_USE=false;}    //计时器
	timx->OUT=TIM_COUNT.Instance->CNT - timx->CNT;
	timx->CNT=TIM_COUNT.Instance->CNT;
}

//用户延时函数，1us计时，最大延时为4294s
void User_Delay(u32 nus)
{
	if(TIM_FIRST_USE){HAL_TIM_Base_Start(&TIM_COUNT);TIM_FIRST_USE=false;}    //计时器
	u32 StrTim = TIM_COUNT.Instance->CNT;
	while(TIM_COUNT.Instance->CNT-StrTim<=nus);
}

//调试串口打印函数
char cache[UART_CACHE_NUM];
void Dprintf(const char * _format,...)
{
	#if UART_DEBUG_EN==1
	va_list ap;
	va_start(ap,_format);
	vsnprintf(cache,UART_CACHE_NUM,_format,ap);
	va_end(ap);
	HAL_UART_Transmit(&UART_DEBUG, (u8 *)cache, strlen(cache), UART_TIMEOUT);
	#endif
}

//发送串口打印函数
void Sprintf(const char * _format,...)
{
	va_list ap;
	va_start(ap,_format);
	vsnprintf(cache,UART_CACHE_NUM,_format,ap);
	va_end(ap);
	HAL_UART_Transmit(&UART_SEND, (u8 *)cache, strlen(cache), UART_TIMEOUT);
}

//字符串转矩阵，将"+X+Y+Z"转成 +1 0 +1 1 +1 2
bool Dir_Trans(s8 Dir[6],const char DirChr[6])
{
	for(int i=0;i<6;)
	{
		switch(DirChr[i])
		{
			case '+':Dir[i]= 1;break;
			case '-':Dir[i]=-1;break;
			default:return false;
		}
		i++;
		switch(DirChr[i])
		{
			case 'X':Dir[i]=0;break;
			case 'Y':Dir[i]=1;break;
			case 'Z':Dir[i]=2;break;
			default:return false;
		}
		i++;
	}
	return true;
}

//判断符号位
float Sign(float value)
{
	if(value>=0.0f)return 1.0f;
	else return -1.0f;
}

float invSqrt(float x)
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

float Norm3(float *data)
{
	return sqrtf(SQR(data[0])+SQR(data[1])+SQR(data[2]));
}

//限幅函数
float fConstrain(float Input, float minValue, float maxValue)
{
	if (Input < minValue) return minValue;
	else if (Input > maxValue) return maxValue;
	else return Input;
}

//限幅函数
s16 iConstrain(s16 Input, s16 minValue, s16 maxValue)
{
	if (Input < minValue) return minValue;
	else if (Input > maxValue) return maxValue;
	else return Input;
}

//循环限幅函数
float LoopConstrain(float Input, float minValue, float maxValue)
{
	if(Input>=maxValue)return LoopConstrain(Input-(maxValue-minValue),minValue,maxValue);
	if(Input<minValue)return LoopConstrain(Input+(maxValue-minValue),minValue,maxValue);
	return Input;
}

//弧度格式化为-PI~PI
float Rad_Format(float Ang)
{
	return LoopConstrain(Ang,-PI,PI);
}

//角度格式化为-180~180
float Theta_Format(float Ang)
{
	return LoopConstrain(Ang,-180.0f,180.0f);
}

//滑窗均值滤波器，CMD:0重新初始化，1正常滤波，2使用同个FIL滤波。
void SlideFilt(float *Dat,float *DatRel,u8 num,sCNT *Filt,u8 Cmd)
{
	switch(Cmd)
	{
		case 0:Filt->CNT=1;break;
		case 1:if(Filt->CNT < Filt->CCR)Filt->CNT++;break;
		case 2:break;
	}
	if(Filt->CNT==1)
	{
		for(int i=0;i<num;i++)
		{
			Dat[i]=DatRel[i];
		}
	}
	for(int i=0;i<num;i++)
	{
		Dat[i]=(Dat[i]*(Filt->CNT-1)+DatRel[i])/Filt->CNT;
	}
}

//字符分割
u8 StrSeg(const char *str,char chr,char *para[],u8 num)
{
	u8 i;
	char *pStr = (char *)strchr(str,chr);       
	if(pStr==NULL)return 0;         //找不到','
	for(i=0;i<num;i++)
	{
		pStr = strchr(pStr,chr);
		if(pStr==NULL)break;      
		para[i]=++pStr;
	}
	return i;  
}

//数据伪更新滤除器，same:当前数据是否相同，返回值：true：正常更新 false：伪更新
bool DataCheck(bool same,sCNT *Check)
{
	if(same==false)
	{
		Check->CNT=0;
		return true;
	}
	if(++Check->CNT>=Check->CCR)return false;
	return true;
}

void LineFit(float* Y,u16 N,float *ab) //y=ax+b
{
	float SumXX=0.0f,SumX=0.0f,SumXY=0.0f,SumY=0.0f;
	for(u16 i=0;i<N;i++)
	{
		SumXX += i*i;
		SumX  += i;
		SumXY += Y[i]*i;
		SumY  += Y[i];
	}
	ab[0] = (SumXY*N-SumX*SumY)/(SumXX*N-SumX*SumX);
	ab[1] = (SumXX*SumY-SumX*SumXY)/(SumXX*N-SumX*SumX);
}

//圆形拟合，d：源数据 N:个数 圆圆心与半径
void circleFit(float d[][2],u16 N,float circleOrigin[],float *circleRadius)
{
	int i;
	float x2,y2;
	float sum_x = 0.0f, sum_y = 0.0f;
	float sum_x2 = 0.0f, sum_y2 = 0.0f;
	float sum_x3 = 0.0f, sum_y3 = 0.0f;
	float sum_xy = 0.0f, sum_xy2 = 0.0f, sum_x2y = 0.0f;
	float C, D, E, G, H;
	float a, b, c;
	circleOrigin[0] = 0.0f;
	circleOrigin[1] = 0.0f;
	*circleRadius = 0.0f;
	for(i=0;i<N;i++)
	{
		x2 = d[i][0]*d[i][0];
		y2 = d[i][1]*d[i][1];
		sum_x += d[i][0];
		sum_y += d[i][1];
		sum_x2 += x2;
		sum_y2 += y2;
		sum_x3 += x2*d[i][0];
		sum_y3 += y2*d[i][1];
		sum_xy += d[i][0]*d[i][1];
		sum_xy2 += d[i][0]*y2;
		sum_x2y += d[i][1]*x2;
	}
	C = N * sum_x2 - sum_x * sum_x;
	D = N * sum_xy - sum_x * sum_y;
	E = N * sum_x3 + N * sum_xy2 - (sum_x2 + sum_y2) * sum_x;
	G = N * sum_y2 - sum_y * sum_y;
	H = N * sum_x2y + N * sum_y3 - (sum_x2 + sum_y2) * sum_y;
	a = (H * D - E * G) / (C * G - D * D);
	b = (H * C - E * D) / (D * D - G * C);
	c = -(a * sum_x + b * sum_y + sum_x2 + sum_y2) / N;
	circleOrigin[0] = a / (-2);
	circleOrigin[1] = b / (-2);
	*circleRadius = sqrt(a * a + b * b - 4 * c) / 2;
}

//球形拟合  d：源数据 N:个数 球圆心与半径
void sphereFit(float d[][3],u16 N,u16 MaxIterations,float Err,u16 Population[][3],float SphereOrigin[],float *SphereRadius)
{
  u8  c;
	u16 i, Iterations;
	float    s[3], s2[3], s3[3], sum[3], sum2[3], sum3[3];
	float    x2sum[3], y2sum[3], z2sum[3];
	float    xy_sum, xz_sum, yz_sum;
	float    XY, XZ, YZ, X2Z, Y2X, Y2Z, Z2X, X2Y, Z2Y;
	float    QS, QB, Q0, Q1, Q2;
	float    R2, C[3], C2[3], Delta[3], Denom[3];
	float    F0, F1, F2, F3, F4;
	float    di2[3];
	float    SizeR;
	for (c = 0; c <= 2; c++)
	{
		s[c] = s2[c] = s3[c] = sum[c] = x2sum[c] = y2sum[c] = z2sum[c] = 0.0f;
		Population[0][c] = Population[1][c] = 0;
	}
	xy_sum = xz_sum = yz_sum = 0.0f;
	for (i = 0; i < N; i++)
	{
		for (c = 0; c <= 2; c++)
		{
				di2[c] = SQR(d[i][c]);
				s[c]  += d[i][c];
				s2[c] += di2[c];
				s3[c] += di2[c] * d[i][c];
				Population[d[i][c] > 0.0f][c]++;
		}
		xy_sum += d[i][0] * d[i][1];
		xz_sum += d[i][0] * d[i][2];
		yz_sum += d[i][1] * d[i][2];
		x2sum[1] += di2[0] * d[i][1];
		x2sum[2] += di2[0] * d[i][2];
		y2sum[0] += di2[1] * d[i][0];
		y2sum[2] += di2[1] * d[i][2];
		z2sum[0] += di2[2] * d[i][0];
		z2sum[1] += di2[2] * d[i][1];
	}
	SizeR = 1.0f / (float) N;
	for (c = 0; c <= 2; c++)
	{
		sum[c]  = s[c]  * SizeR; //sum( X[n]   )
		sum2[c] = s2[c] * SizeR; //sum( X[n]^2 )
		sum3[c] = s3[c] * SizeR; //sum( X[n]^3 )
	}
	XY = xy_sum * SizeR;         //sum( X[n] * Y[n] )
	XZ = xz_sum * SizeR;         //sum( X[n] * Z[n] )
	YZ = yz_sum * SizeR;         //sum( Y[n] * Z[n] )
	X2Y = x2sum[1] * SizeR;  //sum( X[n]^2 * Y[n] )
	X2Z = x2sum[2] * SizeR;  //sum( X[n]^2 * Z[n] )
	Y2X = y2sum[0] * SizeR;  //sum( Y[n]^2 * X[n] )
	Y2Z = y2sum[2] * SizeR;  //sum( Y[n]^2 * Z[n] )
	Z2X = z2sum[0] * SizeR;  //sum( Z[n]^2 * X[n] )
	Z2Y = z2sum[1] * SizeR;  //sum( Z[n]^2 * Y[n] )
	//Reduction of multiplications
	F0 = sum2[0] + sum2[1] + sum2[2];
	F1 = 0.5f * F0;
	F2 = -8.0f * (sum3[0] + Y2X + Z2X);
	F3 = -8.0f * (X2Y + sum3[1] + Z2Y);
	F4 = -8.0f * (X2Z + Y2Z + sum3[2]);
	for (c = 0; c <= 2; c++)
	{
		C[c]  = sum[c];
		C2[c] = SQR(C[c]);
	}
	QS = C2[0] + C2[1] + C2[2];
	QB = -2.0f * (SQR(C[0]) + SQR(C[1]) + SQR(C[2]));
	R2 = F0 + QB + QS;
	Q0 = 0.5f * (QS - R2);
	Q1 = F1 + Q0;
	Q2 = 8.0f * (QS - R2 + QB + F0);
	Iterations = 0;
	do
	{
		for (c = 0; c <= 2; c++)
		{
			Denom[c] = Q2 + 16.0f * (C2[c] - 2.0f * C[c] * sum[c] + sum2[c]);
			if (Denom[c] == 0.0f)
				Denom[c] = 1.0f;
		}
		Delta[0] = -((F2 + 16.0f * (C[1] * XY + C[2] * XZ + sum[0] * (-C2[0] - Q0)
																		+ C[0] * (sum2[0] + Q1 - C[2] * sum[2] - C[1] * sum[1]))) / Denom[0]);
		Delta[1] = -((F3 + 16.0f * (C[0] * XY + C[2] * YZ + sum[1] * (-C2[1] - Q0)
																		+ C[1] * (sum2[1] + Q1 - C[0] * sum[0] - C[2] * sum[2]))) / Denom[1]);
		Delta[2] = -((F4 + 16.0f * (C[0] * XZ + C[1] * YZ + sum[2] * (-C2[2] - Q0)
																		+ C[2] * (sum2[2] + Q1 - C[0] * sum[0] - C[1] * sum[1]))) / Denom[2]);
		for (c = 0; c <= 2; c++)
		{
			C[c] += Delta[c];
			C2[c] = SQR(C[c]);
		}
		QS = C2[0] + C2[1] + C2[2];
		QB = -2.0f * (C[0] * sum[0] + C[1] * sum[1] + C[2] * sum[2]);
		R2 = F0 + QB + QS;
		Q0 = 0.5f * (QS - R2);
		Q1 = F1 + Q0;
		Q2 = 8.0f * (QS - R2 + QB + F0);
		Iterations++;
	}
	while ((Iterations < 50) || ((Iterations < MaxIterations) && ((SQR(Delta[0]) + SQR(Delta[1]) + SQR(Delta[2])) > Err)));
	for (c = 0; c <= 2; c++)  SphereOrigin[c] = C[c];
	*SphereRadius = sqrt(R2);
}

//add by wqz
//绝对值函数
float Abs(float input){
	if(input < 0) return -input;
	else return input;
}
float sq(float val)
{
  return powf(val, 2);
}



void crossMulti3X3(float a[3],float b[3])
{
	float temp[3];
	temp[0] = a[1]*b[2] - a[2]*b[1];
	temp[1] = a[2]*b[0] - a[0]*b[2];
	temp[2] = a[0]*b[1] - a[1]*b[0];
	a[0] = temp[0];
	a[1] = temp[1];
	a[2] = temp[2];
}

void from_axis_angle_3(float axis[3], float theta,float q[4])
{
    // axis must be a unit vector as there is no check for length
    if (theta == 0.0f) {
        q[0] = 1.0f;
        q[1]=q[2]=q[3]=0.0f;
        return;
    }
    float st2 = sinf(theta/2.0f);
    q[0]= cosf(theta/2.0f);
    q[1]= axis[0] * st2;
    q[2]= axis[1] * st2;
    q[3]= axis[2] * st2;
}

void from_axis_angle(float s[3],float q[4])
{
		float v[3];
	  for(u8 i = 0;i<3;i++){
			v[i] = s[i];
		}
    float theta = sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
    if (theta == 0.0f) {
        q[0] = 1.0f;
        q[1]=q[2]=q[3]=0.0f;
        return;
    }
		for(u8 i =0;i<3;i++){
			v[i]/=theta;
		}
    from_axis_angle_3(v,theta,q);
}

void to_axis_angle(float q[4],float v[3])
{
	float l = sqrtf((q[1]*q[1])+(q[2]*q[2])+(q[3]*q[3]));
	v[0] = q[1];
	v[1] = q[2];
	v[2] = q[3];
	if (l!= 0.0f) {
		v[0] /= l;
		v[1] /= l;
		v[2] /= l;			
		float theta = 2.0f*atan2f(l,q[0]);
		if(theta >PI) theta-=2*PI;
		else if(theta<-PI) theta+=2*PI;
		v[0] *= theta;
		v[1] *= theta;
		v[2] *= theta;		
	}
}

double norm(const double x[3])
{
	return sqrt(x[0]*x[0]+x[1]*x[1]+x[2]*x[2]);
}
void QuaternionMulti(float q[4],float v[4])
{
    float temp[4];
		float w1 = q[0];
    float x1 = q[1];
    float y1 = q[2];
    float z1 = q[3];

    float w2 = v[0];
    float x2 = v[1];
    float y2 = v[2];
    float z2 = v[3];

    temp[0] = w1*w2 - x1*x2 - y1*y2 - z1*z2;
    temp[1] = w1*x2 + x1*w2 + y1*z2 - z1*y2;
    temp[2] = w1*y2 - x1*z2 + y1*w2 + z1*x2;
    temp[3] = w1*z2 + x1*y2 - y1*x2 + z1*w2;
		
		q[0] = temp[0];
		q[1] = temp[1];
		q[2] = temp[2];
		q[3] = temp[3];
}

void QuaternionNormalise(float v[4]){
    float quatMag = sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2] + v[3]*v[3]);
    if ((quatMag != 0.0f)) {
        float quatMagInv = 1.0f/quatMag;
        v[0] *= quatMagInv;
        v[1] *= quatMagInv;
        v[2] *= quatMagInv;
        v[3] *= quatMagInv;
    }
}



void QuaternionDivision(float quat[4],float v[4],float result[4])
{
    float quat0 = quat[0];
    float quat1 = quat[1];
    float quat2 = quat[2];
    float quat3 = quat[3];

    float rquat0 = v[0];
    float rquat1 = v[1];
    float rquat2 = v[2];
    float rquat3 = v[3];

    result[0] = (rquat0*quat0 + rquat1*quat1 + rquat2*quat2 + rquat3*quat3);
    result[1] = (rquat0*quat1 - rquat1*quat0 - rquat2*quat3 + rquat3*quat2);
    result[2] = (rquat0*quat2 + rquat1*quat3 - rquat2*quat0 - rquat3*quat1);
    result[3] = (rquat0*quat3 - rquat1*quat2 + rquat2*quat1 - rquat3*quat0);
}

void rotation_matrix(float q[4],float matrix[3][3])
{
	float q1 = q[0];
	float q2 = q[1];
	float q3 = q[2];
	float q4 = q[3];
	float q3q3 = q3 * q3;
	float q3q4 = q3 * q4;
	float q2q2 = q2 * q2;
	float q2q3 = q2 * q3;
	float q2q4 = q2 * q4;
	float q1q2 = q1 * q2;
	float q1q3 = q1 * q3;
	float q1q4 = q1 * q4;
  float q4q4 = q4 * q4;

  matrix[0][0] = 1.0f-2.0f*(q3q3 + q4q4);
  matrix[0][1] =   2.0f*(q2q3 - q1q4);
  matrix[0][2] =   2.0f*(q2q4 + q1q3);
  matrix[1][0] =   2.0f*(q2q3 + q1q4);
  matrix[1][1] = 1.0f-2.0f*(q2q2 + q4q4);
  matrix[1][2] =   2.0f*(q3q4 - q1q2);
  matrix[2][0] =   2.0f*(q2q4 - q1q3);
  matrix[2][1] =   2.0f*(q3q4 + q1q2);
  matrix[2][2] = 1.0f-2.0f*(q2q2 + q3q3);
}

void MatrixMultiVector(float m[3][3],float v[3],float result[3])
{
	float temp[3];
	temp[0] = m[0][0]*v[0] + m[0][1] * v[1] + m[0][2] * v[2];
	temp[1] = m[1][0]*v[0] + m[1][1] * v[1] + m[1][2] * v[2];
	temp[2] = m[2][0]*v[0] + m[2][1] * v[1] + m[2][2] * v[2];
	
	result[0] = temp[0];
	result[1] = temp[1];
	result[2] = temp[2];
}

void QuaternionReverse(float q[4])
{
	q[0] = q[0];
	q[1] = -q[1];
	q[2] = -q[2];
	q[3] = -q[3];
}

void Quaternion2Euler(float q[4],float euler[3])
{
	float q1 = q[0];
	float q2 = q[1];
	float q3 = q[2];
	float q4 = q[3];
	
	euler[0] = (atan2f(2.0f*(q1*q2 + q3*q4), 1.0f - 2.0f*(q2*q2 + q3*q3)));;
	euler[1] = asinf(2.0f*(q1*q3 - q4*q2));;
	euler[2] = atan2f(2.0f*(q1*q4 + q2*q3), 1.0f - 2.0f*(q3*q3 + q4*q4));;
}

void rot_from_euler(float roll, float pitch, float yaw,float rot_mat[3][3])
{
	float cp = cos(pitch);
	float sp = sin(pitch);
	float sr = sin(roll);
	float cr = cos(roll);
	float sy = sin(yaw);
	float cy = cos(yaw);

	rot_mat[0][0] = cp * cy;
	rot_mat[0][1] = (sr * sp * cy) - (cr * sy);
	rot_mat[0][2] = (cr * sp * cy) + (sr * sy);
	rot_mat[1][0] = cp * sy;
	rot_mat[1][1] = (sr * sp * sy) + (cr * cy);
	rot_mat[1][2] = (cr * sp * sy) - (sr * cy);
	rot_mat[2][0] = -sp;
	rot_mat[2][1] = sr * cp;
	rot_mat[2][2] = cr * cp;
}

void from_euler312(float roll, float pitch, float yaw,float rot_mat[3][3])
{
    float c3 = cosf(pitch);
    float s3 = sinf(pitch);
    float s2 = sinf(roll);
    float c2 = cosf(roll);
    float s1 = sinf(yaw);
    float c1 = cosf(yaw);

    rot_mat[0][0] = c1 * c3 - s1 * s2 * s3;
    rot_mat[1][1] = c1 * c2;
    rot_mat[2][2] = c3 * c2;
    rot_mat[0][1] = -c2*s1;
    rot_mat[0][2] = s3*c1 + c3*s2*s1;
    rot_mat[1][0] = c3*s1 + s3*s2*c1;
    rot_mat[1][2] = s1*s3 - s2*c1*c3;
    rot_mat[2][0] = -s3*c2;
    rot_mat[2][1] = s2;
}

void to_euler312(float mat[3][3],float euler[3])
{
	euler[0] = asinf(mat[2][1]);
	euler[1] = atan2f(-mat[2][0],mat[2][2]);
	euler[2] = atan2f(-mat[0][1],mat[1][1]);
}

//注释 -180-180 转到0-360
void PIto2PI(float *theta){
	if(*theta<0.0f){
		while(*theta<0.0f){
			*theta+=2*PI;
		}
	}
	else if(*theta>=2*PI){
		while(*theta>=2*PI){
			*theta-=2*PI;
		}
	}
	else{
		return;
	}
}
void euler2quaternion(float euler[3],float q[4])
{
	float cphi = cos(0.5f * euler[0]);
  float sphi = sin(0.5f * euler[0]);
	float cthe = cos(0.5f * euler[1]);
	float sthe = sin(0.5f * euler[1]);
	float cpsi = cos(0.5f * euler[2]);
	float spsi = sin(0.5f * euler[2]);
	q[0] = cphi * cthe * cpsi + sphi * sthe * spsi;
	q[1] = sphi * cthe * cpsi - cphi * sthe * spsi;
	q[2] = cphi * sthe * cpsi + sphi * cthe * spsi;
	q[3] = cphi * cthe * spsi - sphi * sthe * cpsi;
}


//LU分解求逆
void matrix_inverse_LU(int N, double A[][N],double A_inverse[][N])
{
    //对矩阵A，由高斯消元法化为上三角U，即有A=LU,其中L就是一系列初等变换的乘积，是一个对角线为1的下三角矩阵，U是上三角矩阵
		//上下三角矩阵求逆比较简单。预设处逆阵，由L*L_inverse=E解方程求出
	  //预设处L和U的元素可以由方程解出，顺序为U[i][i],U[i][i+1],------,U[i][N],再求L[i+1][i],L[i+2][i],------,L[N][i]
		double  L[N][N], U[N][N];
    double  L_inverse[N][N], U_inverse[N][N];
    int i, j, k;
    double  s;
	//short占 2 字节,int ，float，long 都占 4 字节,double 占8 字节
	//void *memset(void *s,int c,size_t n):将已开辟内存空间s的首n个字节的值设为c
    //sizeof用来计算一个变量，或者数据类型的长度，以字节为单位
    memset(L, 0, N*N*sizeof(double));
    memset(U, 0, N*N*sizeof(double));
    memset(L_inverse, 0, N*N*sizeof(double));
    memset(U_inverse, 0, N*N*sizeof(double));  
	//----------------根据情况调整类型---------------
	  memset(A_inverse, 0, N*N*sizeof(double));
	//-----------------------------------------------
    for (i = 0; i < N;i++)       //计算L矩阵的对角线
    {
        L[i][i] = 1;
    }
    for (i = 0;i < N;i++)
    {
        for (j = i;j < N;j++)
        {
            s = 0;
            for (k = 0;k < i;k++)
            {
                s += L[i][k] * U[k][j];
            }
            U[i][j] = A[i][j] - s;      //按行计?U[i][j]        
        }
        for (j = i + 1;j < N;j++)
        {
            s = 0;
            for (k = 0; k < i; k++)
            {
                s += L[j][k] * U[k][i];
            }
            L[j][i] = (A[j][i] - s) / U[i][i];      //按列计算L[j][i]
        }
    }
    for (i = 0;i < N;i++)        //按行序，行内从高到低，计算L的逆矩阵
    {
        L_inverse[i][i] = 1;
    }
    for (i= 1;i < N;i++)
    {
        for (j = 0;j < i;j++)
        {
            s = 0;
            for (k = 0;k < i;k++)
            {
                s += L[i][k] * L_inverse[k][j];
            }
            L_inverse[i][j] = -s;
        }
    } 
    for (i = 0;i < N;i++)         //按列序，列内按照从下到上，计算U的逆矩阵
    {
        U_inverse[i][i] = 1 / U[i][i];
    }
    for (i = 1;i < N;i++)
    {
        for (j = i - 1;j >=0;j--)//从下到上
        {
            s = 0;
            for (k = j + 1;k <= i;k++)
            {
                s += U[j][k] * U_inverse[k][i];
            }
            U_inverse[j][i] = -s / U[j][j];
        }
    } 
    for (i = 0;i < N;i++)            //计算A的逆矩阵，矩阵乘法，3for循环
    {
        for (j = 0;j < N;j++)
        {
            for (k = 0;k < N;k++)
            {
                A_inverse[i][j] += U_inverse[i][k] * L_inverse[k][j];//根据所需的输出类型做强制转换//(float)
            }
        }
    }
}
void cross_product_matrix(double u[3],double u_x_element[][3])
{
	u_x_element[0][0]=0;
	u_x_element[0][1]=-u[2];
	u_x_element[0][2]= u[1];
	u_x_element[1][0]= u[2];
	u_x_element[1][1]=0;
	u_x_element[1][2]=-u[0];
	u_x_element[2][0]=-u[1];
	u_x_element[2][1]= u[0];
	u_x_element[2][2]=0;
}

void angle_axis_of_rotation(double theta[3],double *phi, double u[3])
{
	double a=(theta[0]*theta[0]+theta[1]*theta[1]+theta[2]*theta[2]);
	*phi=sqrt(a);
	for(int i=0;i<3;i++)
	{
		u[i]=theta[i]* invSqrt(a);
	}
}	
void creat_quaternion(double *phi, double u[3],double q[4])
{
	double sinphi, cosphi;
	cosphi=cos((*phi)/2.0f);
	sinphi=sin((*phi)/2.0f);
	q[0]=cosphi;
	q[1]=u[0]*sinphi;
	q[2]=u[1]*sinphi;
	q[3]=u[2]*sinphi;
}
void creat_rotation_matrix(double *phi, double u[3],double Rot_element[][3])
{
	double sinphi, cosphi;
	cosphi=cos(*phi);
	sinphi=sin(*phi);
	double u_x_element[3][3], uuT_element[3][3];
	cross_product_matrix(u, u_x_element); 
	for(int i=0;i<3;i++)
	{
		for(int j=0;j<3;j++)
		{
			uuT_element[i][j]=u[i]*u[j];
		}
	}	
	for(int i=0;i<3;i++)
	{
		for(int j=0;j<3;j++)
		{
			Rot_element[i][j]=sinphi*u_x_element[i][j]+(1-cosphi)*uuT_element[i][j];
		}
	}
	Rot_element[0][0]=Rot_element[0][0]+cosphi;
	Rot_element[1][1]=Rot_element[1][1]+cosphi;
	Rot_element[2][2]=Rot_element[2][2]+cosphi;
}
void quaternion_mul(double q1[4], double q2[4], double q[4])
{
	double q1_L_element[4][4];
	q1_L_element[0][0]= q1[0];
	q1_L_element[0][1]=-q1[1];
	q1_L_element[0][2]=-q1[2];
	q1_L_element[0][3]=-q1[3];
	q1_L_element[1][0]= q1[1];
	q1_L_element[1][1]= q1[0];
	q1_L_element[1][2]=-q1[3];
	q1_L_element[1][3]= q1[2];
	q1_L_element[2][0]= q1[2];
	q1_L_element[2][1]= q1[3];
	q1_L_element[2][2]= q1[0];
	q1_L_element[2][3]=-q1[1];
	q1_L_element[3][0]= q1[3];
	q1_L_element[3][1]=-q1[2];
	q1_L_element[3][2]= q1[1];
	q1_L_element[3][3]= q1[0];
	for(int i=0;i<4;i++)
	{
		double  temp = 0.0f;
		for(int k=0 ; k < 4 ; k++)
		{
			temp += q1_L_element[i][k] * q2[k];
		}
		q[i] = temp;
	}
}



