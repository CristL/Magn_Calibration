#ifndef __9150_TEST_H
#define __9150_TEST_H

#define	SMPLRT_DIV		0x19	//陀螺仪采样率，典型值：0x07(125Hz) 采样率 = 陀螺仪的输出率 / (1 + SMPLRT_DIV)
								//当数字低通滤波器没有使能的时候，陀螺仪的输出平路等于8KHZ，反之等于1KHZ。

#define	CONFIG			0x1A	//低通滤波频率，典型值：0x06(5Hz)
#define	GYRO_CONFIG		0x1B	//陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
#define	ACCEL_CONFIG	0x1C	//加速计自检、测量范围及高通滤波频率，典型值：0x01(不自检，2G，5Hz)
#define	ACCEL_XOUT_H	0x3B 	//加速度计
#define	ACCEL_XOUT_L	0x3C
#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E 
#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40
#define	TEMP_OUT_H		0x41  	//温度
#define	TEMP_OUT_L		0x42

#define	GYRO_XOUT_H		0x43  	//陀螺仪
#define	GYRO_XOUT_L		0x44	
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48

#define	MAGN_XOUT_L		0x03
#define	MAGN_XOUT_H		0x04
#define	MAGN_YOUT_L		0x05
#define	MAGN_YOUT_H		0x06
#define	MAGN_ZOUT_L		0x07
#define	MAGN_ZOUT_H		0x08
#define	MAGN_CNTL		0x0A	//data=0x0F	init
#define	MAGN_I2CDIS		0x0F	//default enable ; 00011011 to disable ; 0 to prohibit
#define	MAGN_ASAX		0x10	//Sensitivity adjustment values
/**********************
Hadj = H * ((ASA-128)*0.5/128 + 1)
**********************/
#define	MAGN_ASAY		0x11
#define	MAGN_ASAZ		0x12



#define	PWR_MGMT_1		0x6B	//电源管理，典型值：0x00(正常启用)
#define	WHO_AM_I		0x75	//IIC地址寄存器(默认数值0x68，只读)

#define	MPU6050_Addr	0xD0	//定义器件在IIC总线中的从地址,根据ALT  ADDRESS地址引脚不同修改

//****************************

#define MAGN_OUT_X_L	0x28
#define MAGN_OUT_X_H 	0x29
#define MAGN_OUT_Y_L	0x2A
#define MAGN_OUT_Y_H	0x2B
#define MAGN_OUT_Z_L 	0x2C
#define MAGN_OUT_Z_H 	0x2D
#define MAGN_STATUS_REG	0x27

#define PROPORTION	0.9

/*****************************/
#define pi 3.1415926
#define ot 1e-8
#define min(a,b) ((a<b)?(a):(b))
#define max(a,b) ((a>b)?(a):(b))

void Get_AccelGyro_Data(int fd);
double SimpKalfilter_Pitch(double z_measured);
double SimpKalfilter_Roll(double z_measured);
void Accel_Correct(int);
void Calculate_PitchRoll(void);
void Coordinate_Transformation(void);
void Calculate_Heading(int magn_fd,FILE* to_fd);
void Magnetic_Vector_Projection(void);
void Get_Magn_Data(int magn_fd);
void Get_9150Magn_Data(int fd);
void Printf_AccelGyroMagn_Output(void);
double Heading_Calculate(double ture_Hx,double ture_Hy);

void Gauss(double **pCoff, double *pConst,int n);
double MAX(double *a,unsigned int m);
double MIN(double *a,unsigned int m);
double SimpKalfilter(double z_measured);

typedef struct accelgyro{
	double accel[3];
	double gyro[3];
}ACCELGYRO;

typedef struct magnetometer{
	short mx;
	short my;
	short mz;
}MAGNETOMETER_DATA;

typedef struct magn_horizontal{
	double Bx;
	double By;
	double Bz;
}MAGN_CHANGE_TO_DOUBLE;

typedef struct attitude{
	double pitch;
	double roll;
	double yaw;
}ATTITUDE_;

typedef struct config_temp_value{
	double **V;
	double *bb;
}CONFIG_TEMP_VALUE;

typedef struct  ellipse_parameter{
	double A0;
	double B0;
	double C0;
	double D0;
	double E0;
	double F;
	double a_radius;
	double b_radius;
	double X0;
	double Y0;
	double orientation_rad;
}ELLIPSE_PARAMETER;

CONFIG_TEMP_VALUE equation_temp_value(double *ax,double *bx,CONFIG_TEMP_VALUE g_equation_temp_value,unsigned int m);
ELLIPSE_PARAMETER ellipse_parameter_calculate(CONFIG_TEMP_VALUE g_equation_temp_value,ELLIPSE_PARAMETER g_ellipse_parameter);

#endif