#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <math.h>
#include "9150_test.h"

//#define RAW_TO_VIEW
#define SIMPKALFILTER
#define COORDINATE_TRANSFORMATION
//#define RADIAN_TO_ANGLE
//#define ATTITUDE
#define HEADING
#define Q 0.001
#define R 0.45
#define pi 3.1415926
void accelgyro_data_collection(int);
double SimpKalfilter_Pitch(double z_measured);
double SimpKalfilter_Roll(double z_measured);
void Acc_Correct(int);
void Printf_AccelGyro_Data(void);
void Calculate_PitchRoll(void);
void Coordinate_Transformation(void);
void Get_Magn_Data(int magn_fd);
void Magnetic_Vector_Projection(void);

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
}MAGN_HORIZONTAL;

typedef struct attitude{
	double pitch;
	double roll;
	double yaw;
}ATTITUDE_;
ACCELGYRO accelgyro_data;
ACCELGYRO accelgyro_offset;
ATTITUDE_ attitude_static;
MAGNETOMETER_DATA magn_measure_val;
MAGN_HORIZONTAL magnetic_vector;

short accelgyro_raw[6];
double ture_Hx,ture_Hy;
double heading;

int main(int argc, char **argv)
{
	int fd;
	int magn_fd;
	int flag = 0;
	fd = open("/dev/mpu9150",O_RDWR);
	if (fd < 0)
	{
		printf("can't open /dev/mpu9150\n");
		return -1;
	}else{
		printf("Now,you are in /dev/mpu9150\n");
	}
	/* 	读寄存器判断芯片的选择，同时判断写入寄存器的配置是否正确 */
	/*
	buf[0] = WHO_AM_I;
		read(fd,buf,1);
		printf("WHO_AM_I = %d\n",buf[0]);
	buf[1] = CTRL_REG3;
		read(fd,&buf[1],1);
		printf("CTRL_REG3 = 0x%x\n",buf[1]);
	buf[2] = mpu9150_STATUS_REG;
		read(fd,&buf[2],1);
		printf("mpu9150_STATUS_REG = 0x%x\n",buf[2]);
	buf[3] = CTRL_REG4;
		read(fd,&buf[3],1);
		printf("CTRL_REG4 = 0x%x\n",buf[3]);
	*/
	//sleep(3);/* 跳过前面的读数，因为前期的值是不准确的 */
	
	magn_fd = open("/dev/magn",O_RDWR);
	if (magn_fd < 0)
	{
		printf("can't open /dev/magn\n");
		return -1;
	}else{
		printf("Now,you are in /dev/magn\n");
	}
	
	Acc_Correct(fd);
	if(!flag)
	while(1){
		Get_AccelGyro_Data(fd);//通过flag来判断收集两轴数据，保存在指定文件中;
#ifdef COORDINATE_TRANSFORMATION
		Coordinate_Transformation();//将9150坐标系变换到磁力计lis3mdl坐标系；
#endif
		Calculate_PitchRoll();//计算俯仰角和侧倾角
		
		Get_Magn_Data(magn_fd);
		Printf_AccelGyroMagn_Output();
		usleep(50*1000);
	}
	
	close(fd);
	close(magn_fd);
	
	return 0;
}


void Get_AccelGyro_Data(int fd){
	//FILE *file_fd;
	int i = 0;
	unsigned char buf[12];
	/*
	file_fd = fopen("/data_file.txt","w+");
	if(!file_fd)
	{
		fprintf(stderr,"Open 'data_file' is %s",strerror(1));
		exit(1);
	}
	*/
	
	buf[0] = ACCEL_XOUT_L;
	read(fd,buf,1);
	buf[1] = ACCEL_XOUT_H;
	read(fd,&buf[1],1);
	buf[2] = ACCEL_YOUT_L;
	read(fd,&buf[2],1);
	buf[3] = ACCEL_YOUT_H;
	read(fd,&buf[3],1);
	buf[4] = ACCEL_ZOUT_L;
	read(fd,&buf[4],1);
	buf[5] = ACCEL_ZOUT_H;
	read(fd,&buf[5],1);
	
	buf[6] = GYRO_XOUT_L;
	read(fd,&buf[6],1);
	buf[7] = GYRO_XOUT_H;
	read(fd,&buf[7],1);
	buf[8] = GYRO_YOUT_L;
	read(fd,&buf[8],1);
	buf[9] = GYRO_YOUT_H;
	read(fd,&buf[9],1);
	buf[10] = GYRO_ZOUT_L;
	read(fd,&buf[10],1);
	buf[11] = GYRO_ZOUT_H;
	read(fd,&buf[11],1);
	
	//data[0] = (((s16) ((buf[1] << 8) | buf[0])) >> 4);
	accelgyro_raw[0] = ((short) ((buf[1] << 8) | buf[0]));
	accelgyro_raw[1] = ((short) ((buf[3] << 8) | buf[2]));
	accelgyro_raw[2] = ((short) ((buf[5] << 8) | buf[4]));
	
	accelgyro_raw[3] = ((short) ((buf[7] << 8) | buf[6]));
	accelgyro_raw[4] = ((short) ((buf[9] << 8) | buf[8]));
	accelgyro_raw[5] = ((short) ((buf[11] << 8) | buf[10]));

	
	while(i<6)
	{
		if(i>2)
		{
			/* 一阶滤波 */
			//double_data[i] = SimpKalfilter((double)data[i]) / 16.384;
			accelgyro_data.gyro[i-3] = accelgyro_raw[i] / 16.384;//陀螺仪的量程为2000，这里得到的值为（度/s）
		}else{
			accelgyro_data.accel[i] = accelgyro_raw[i] / 163.84;//加速度计的量程为2g，这里表示的为重力加速度g的100分之一
		}			
		i++;
	}
	i = 0;
	//fclose(file_fd);
}

#ifdef SIMPKALFILTER
double SimpKalfilter_Pitch(double z_measured)
{ 
//printf("You are here 1.1\n");
	static double x_est_last = 0; 
	static double P_last = 0; 
	static double K; 
	static double P; 

	//the noise in the system 
	double P_temp; 
	double x_temp_est; 
	double x_est; 

	//do a prediction ，状态与协方差预测方程
	x_temp_est = x_est_last;     								//方程1
	P_temp = P_last + Q;        				 				//方程2
	//calculate the Kalman gain，增益估计方程
	K = P_temp * (1.0/(P_temp + R));     						//方程3
	//correct ，滤波估计方程，得出下一个数值，与滤波协方差方程
	x_est = x_temp_est + K * (z_measured - x_temp_est);  		//方程4
	P = (1- K) * P_temp;    									//方程5
	//we have our new system    

	//update our last's 
	P_last = P; 
	x_est_last = x_est; 

	return x_est; 
}

double SimpKalfilter_Roll(double z_measured)
{ 
//printf("You are here 1.1\n");
	static double x_est_last = 0; 
	static double P_last = 0; 
	static double K; 
	static double P; 

	//the noise in the system 
	double P_temp; 
	double x_temp_est; 
	double x_est; 

	//do a prediction ，状态与协方差预测方程
	x_temp_est = x_est_last;     								//方程1
	P_temp = P_last + Q;        				 				//方程2
	//calculate the Kalman gain，增益估计方程
	K = P_temp * (1.0/(P_temp + R));     						//方程3
	//correct ，滤波估计方程，得出下一个数值，与滤波协方差方程
	x_est = x_temp_est + K * (z_measured - x_temp_est);  		//方程4
	P = (1- K) * P_temp;    									//方程5
	//we have our new system    

	//update our last's 
	P_last = P; 
	x_est_last = x_est; 

	return x_est; 
}
#endif

void Acc_Correct(int fd)
{
	unsigned char i=0;
	unsigned char numAcc=200;

	double Angleaccx=0;
	double Angleaccy=0;
	double Angleaccz=0;							  //加速度计校正中间变量

	for(i=0;i<numAcc;i++)
	{		
		Get_AccelGyro_Data(fd);
		Angleaccx += accelgyro_data.accel[0];
		Angleaccy += accelgyro_data.accel[1];
		//Angleaccz += accelgyro_data.accel[2];
	}	
	accelgyro_offset.accel[0] += Angleaccx/numAcc;//得到加速度计基准
	accelgyro_offset.accel[1] += Angleaccy/numAcc;
	//accelgyro_offset.accel[2] += Angleaccz/numAcc; 	
}

void Printf_AccelGyroMagn_Output(void)
{
#ifdef RAW_TO_VIEW		
	printf("%.2lf,%.2lf,%.2lf,%.2lf,%.2lf,%.2lf,%2lf,%2lf,%2lf\n" \
		,accelgyro_data.accel[0],accelgyro_data.accel[1],accelgyro_data.accel[2],accelgyro_data.gyro[0],accelgyro_data.gyro[1],accelgyro_data.gyro[2]);
#elif defined(ATTITUDE)
	printf("%.2lf,%.2lf,%.2lf\n",attitude_static.pitch,attitude_static.roll,attitude_static.yaw);
#elif defined(HEADING)
	printf("%.2lf\n",heading);
	//printf("%.2lf,%.2lf\n",magnetic_vector.Bx,magnetic_vector.By);
#else
	printf("%d,%d,%d,%d,%d,%d,%d,%d,%d\n",accelgyro_raw[0],accelgyro_raw[1],accelgyro_raw[2],accelgyro_raw[3],accelgyro_raw[4],accelgyro_raw[5], \
	magn_measure_val.mx,magn_measure_val.my,magn_measure_val.mz);
#endif
}

void Calculate_PitchRoll(void)
{
#ifndef COORDINATE_TRANSFORMATION
	int i = 0;
	while(i<2)//暂时不考虑Z轴
	{
		accelgyro_data.accel[i] -= accelgyro_offset.accel[i];
		i++;
	}
	i = 0;
#endif
	/*
	attitude_static.pitch = atan2(accelgyro_data.accel[0],accelgyro_data.accel[2]) * 57.3;
	attitude_static.roll = atan2(accelgyro_data.accel[1],accelgyro_data.accel[2]) * 57.3;
	*/
	attitude_static.roll = atan(accelgyro_data.accel[1]/accelgyro_data.accel[2]);//侧倾角弧度值
	//printf("%lf\n",attitude_static.roll);
	attitude_static.pitch = atan(accelgyro_data.accel[0]/(accelgyro_data.accel[1]*sin(attitude_static.roll) + accelgyro_data.accel[2]* cos(attitude_static.roll)));//俯仰角弧度值
#ifdef RADIAN_TO_ANGLE
	attitude_static.roll = attitude_static.roll * 57.3;
	attitude_static.pitch = attitude_static.pitch * 57.3;
#endif
	//printf("%lf\n",attitude_static.roll);
	
	//attitude_static.yaw = atan2(accelgyro_data.accel[1],accelgyro_data.accel[0]) * 57.3;
#ifdef SIMPKALFILTER
	attitude_static.pitch = SimpKalfilter_Pitch(attitude_static.pitch);
	attitude_static.roll = SimpKalfilter_Roll(attitude_static.roll);
#endif
}

/* 坐标变换，将9150的坐标系变换到lis3mdl磁力计的坐标系下 */
void Coordinate_Transformation(void)
{
	int i = 0;
	double temp;
	while(i<2)//暂时不考虑Z轴
	{
		accelgyro_data.accel[i] -= accelgyro_offset.accel[i];
		i++;
	}
	temp = accelgyro_data.accel[1];
	accelgyro_data.accel[1] = -accelgyro_data.accel[0];
	accelgyro_data.accel[0] = temp;	
}
/* 磁矢量投影，将倾斜的目标投影到水平面，得到磁矢量的水平投影值 */
void Magnetic_Vector_Projection(void)
{
	magnetic_vector.Bx = magn_measure_val.mx*cos(attitude_static.pitch) + magn_measure_val.my*sin(attitude_static.pitch)*sin(attitude_static.roll) \
	+ magn_measure_val.mz*sin(attitude_static.pitch)*cos(attitude_static.roll);
	magnetic_vector.By = magn_measure_val.my*cos(attitude_static.roll) - magn_measure_val.mz*sin(attitude_static.roll);
}

void Get_Magn_Data(int magn_fd)
{
	int j = 0;
	unsigned char buf[6];

	buf[0] = MAGN_OUT_X_L;
	read(magn_fd,buf,1);
	buf[1] = MAGN_OUT_X_H;
	read(magn_fd,&buf[1],1);
	buf[2] = MAGN_OUT_Y_L;
	read(magn_fd,&buf[2],1);
	buf[3] = MAGN_OUT_Y_H;
	read(magn_fd,&buf[3],1);
	buf[4] = MAGN_OUT_Z_L;
	read(magn_fd,&buf[4],1);
	buf[5] = MAGN_OUT_Z_H;
	read(magn_fd,&buf[5],1);
	
	//data[0] = (((s16) ((buf[1] << 8) | buf[0])) >> 4);
	magn_measure_val.mx = ((short) ((buf[1] << 8) | buf[0]));
	magn_measure_val.my = ((short) ((buf[3] << 8) | buf[2]));
	magn_measure_val.mz = ((short) ((buf[5] << 8) | buf[4]));
	
	Magnetic_Vector_Projection();
	
	ture_Hx = magnetic_vector.By;
	ture_Hy = magnetic_vector.Bx;
	if(ture_Hx > 0 && ture_Hy <= 0)
			heading = (pi + atan(-ture_Hx / ture_Hy)) * 57.3;
		else if(ture_Hx == 0 && ture_Hy < 0)
			heading = pi * 57.3;
		else if(ture_Hx < 0 && ture_Hy < 0)
			heading = (pi + atan(-ture_Hx / ture_Hy))*57.3;
		else if(ture_Hx < 0 && ture_Hy > 0)
			heading = atan(-ture_Hx / ture_Hy)*57.3;
		else if(ture_Hx == 0 && ture_Hy > 0)
			heading = 0;
		else if(ture_Hx > 0 && ture_Hy > 0)
			heading = (2.0 * pi + atan(-ture_Hx / ture_Hy))*57.3;	
}