#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <wait.h>
#include <math.h>
#include "magn_data_collection.h"

void Get_AccelGyro_Data(int mpu9150_fd);
double Calculate_Angle(double ture_Hx,double ture_Hy);
void Calculate_PitchRoll(void);
void Coordinate_Transformation(void);
void Magnetic_Vector_Projection(void);
void Magn_Horizontal_Projection(int magn_fd,FILE* to_mpu9150_fd);
void Get_Magn_Data(int magn_fd);

#define ACCELGYRO_RAW_FILTER				//输出加速度计和陀螺仪原始值进行简单滤波值

#define TILT_ANGLE_MAPPING					//椭圆校正采集值是否进行倾斜角转换
//#define MAGNETOMETER_MAPPING_COMPARISON	//对比磁力计倾斜角映射前后输出值
//#define RADIAN_TO_ANGLE					//将倾斜角的弧度值转化为角度值输出

#ifdef SIMPKALFILTER
double Q = 0.001; 
double R = 0.45; //0.45//R值表示的是测量方程协方差值，这里的0.7448 是在未滤波的输出条件下对采集值进行协方差求解得到的值；
#endif

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

short accelgyro_raw[6];
ACCELGYRO accelgyro_data;
ATTITUDE_ attitude_static_rad;
ATTITUDE_ attitude_static_angle;
MAGNETOMETER_DATA magn_measure_val;
MAGN_CHANGE_TO_DOUBLE magnetic_vector;
MAGN_CHANGE_TO_DOUBLE filter_magn_measure_val;

int main(int argc, char **argv)
{
	int magn_fd,mpu9150_fd;
	double *ax,*ay;
	
	magn_fd = open("/dev/magn",O_RDWR);				//lis3mdltr module
	if (magn_fd < 0){
		printf("can't open /dev/magn\n");
		return -1;
	}else{
		printf("Open '/dev/magn' success!\n");
	}
	
	mpu9150_fd = open("/dev/mpu9150",O_RDWR);		//mpu9150 module
	if (mpu9150_fd < 0){
		printf("can't open /dev/mpu9150\n");
		return -1;
	}else{
		printf("Open '/dev/mpu9150' success!\n");
	}
	
	FILE *to_magn_file;
	to_magn_file = fopen("/data_file.txt","w+");	//File to save lis3mdltr (xy) data
	if(!to_magn_file){
		fprintf(stderr,"Open 'data_file' is %s",strerror(1));
		exit(1);
	}
	
	/* 	读寄存器判断芯片的选择，同时判断写入寄存器的配置是否正确 */
	/*
	buf[0] = WHO_AM_I;
		read(magn_fd,buf,1);
		printf("WHO_AM_I = %d\n",buf[0]);
	buf[1] = CTRL_REG3;
		read(magn_fd,&buf[1],1);
		printf("CTRL_REG3 = 0x%x\n",buf[1]);
	buf[2] = MAGN_STATUS_REG;
		read(magn_fd,&buf[2],1);
		printf("MAGN_STATUS_REG = 0x%x\n",buf[2]);
	buf[3] = CTRL_REG4;
		read(magn_fd,&buf[3],1);
		printf("CTRL_REG4 = 0x%x\n",buf[3]);
	*/
	
	/* Ignore the previous data,it's wrong data */
	sleep(2);								

	while(1)
	{
		/* 使用姿态传感器测量模块倾斜角，使用倾斜角让采集到的磁力计值回归到旋转水平面 */
		Get_AccelGyro_Data(mpu9150_fd);
		Coordinate_Transformation();
		Calculate_PitchRoll();
		Magn_Horizontal_Projection(magn_fd,to_magn_file);		

#ifdef RADIAN_TO_ANGLE								//输出俯仰和侧倾角
		printf("%.2lf %.2lf\n",attitude_static_angle.pitch,attitude_static_angle.roll);
#elif defined (MAGNETOMETER_MAPPING_COMPARISON)		//对比磁力计倾斜角映射前后输出值	
		printf("%.2lf %.2lf %.2lf %.2lf %.2lf %.2lf\n",filter_magn_measure_val.Bx,\
		filter_magn_measure_val.By,filter_magn_measure_val.Bz,magnetic_vector.Bx,magnetic_vector.By,magnetic_vector.Bz);
		fprintf(to_magn_file,"%.2lf %.2lf %.2lf %.2lf %.2lf %.2lf\n",filter_magn_measure_val.Bx,filter_magn_measure_val.By,filter_magn_measure_val.Bz,magnetic_vector.Bx,\
		magnetic_vector.By,magnetic_vector.Bz);
#else
		printf("%.2lf %.2lf\n",magnetic_vector.Bx,magnetic_vector.By);
		fprintf(to_magn_file,"%.2lf %.2lf\n",magnetic_vector.Bx,magnetic_vector.By);	//save the horizontal magnetic vector component
#endif
		usleep(20*1000);
	}
	fclose(to_magn_file);
	close(magn_fd);
	close(mpu9150_fd);
	
	return 0;
}

/* Get the accel and gyro data from 9150 */
void Get_AccelGyro_Data(int mpu9150_fd){
	int i = 0;
	unsigned char buf[12];
	
#ifdef ACCELGYRO_RAW_FILTER
	ACCELGYRO old_accelgyro_data;

	for(int j=0;j<3;j++)
	{
		old_accelgyro_data.accel[j] = accelgyro_raw[j];
	}
#endif
	
	buf[0] = ACCEL_XOUT_L;
	read(mpu9150_fd,buf,1);
	buf[1] = ACCEL_XOUT_H;
	read(mpu9150_fd,&buf[1],1);
	buf[2] = ACCEL_YOUT_L;
	read(mpu9150_fd,&buf[2],1);
	buf[3] = ACCEL_YOUT_H;
	read(mpu9150_fd,&buf[3],1);
	buf[4] = ACCEL_ZOUT_L;
	read(mpu9150_fd,&buf[4],1);
	buf[5] = ACCEL_ZOUT_H;
	read(mpu9150_fd,&buf[5],1);
	
	buf[6] = GYRO_XOUT_L;
	read(mpu9150_fd,&buf[6],1);
	buf[7] = GYRO_XOUT_H;
	read(mpu9150_fd,&buf[7],1);
	buf[8] = GYRO_YOUT_L;
	read(mpu9150_fd,&buf[8],1);
	buf[9] = GYRO_YOUT_H;
	read(mpu9150_fd,&buf[9],1);
	buf[10] = GYRO_ZOUT_L;
	read(mpu9150_fd,&buf[10],1);
	buf[11] = GYRO_ZOUT_H;
	read(mpu9150_fd,&buf[11],1);
	
	//data[0] = (((s16) ((buf[1] << 8) | buf[0])) >> 4);
	accelgyro_raw[0] = ((short) ((buf[1] << 8) | buf[0]));
	accelgyro_raw[1] = ((short) ((buf[3] << 8) | buf[2]));
	accelgyro_raw[2] = ((short) ((buf[5] << 8) | buf[4]));
	
	accelgyro_raw[3] = ((short) ((buf[7] << 8) | buf[6]));
	accelgyro_raw[4] = ((short) ((buf[9] << 8) | buf[8]));
	accelgyro_raw[5] = ((short) ((buf[11] << 8) | buf[10]));

	while(i<6)
	{
		if(i>2){
			accelgyro_data.gyro[i-3] = accelgyro_raw[i] / 16.384;	//陀螺仪的量程为2000，这里得到的值为（度/s）
		}else{			
			accelgyro_data.accel[i] = accelgyro_raw[i] / 163.84;	//加速度计的量程为2g，这里表示的为重力加速度g的100分之一

#ifdef ACCELGYRO_RAW_FILTER
			/* 一阶滤波 */
			accelgyro_data.accel[i] = old_accelgyro_data.accel[i] * PROPORTION + accelgyro_data.accel[i] * (1 - PROPORTION);
#endif
		}			
		i++;
	}
}

void Calculate_PitchRoll(void)
{
	attitude_static_rad.roll = atan(accelgyro_data.accel[1] / accelgyro_data.accel[2]);//侧倾角弧度值
	attitude_static_rad.pitch = atan(-accelgyro_data.accel[0]/(accelgyro_data.accel[1]*sin(attitude_static_rad.roll)\
	+ accelgyro_data.accel[2]* cos(attitude_static_rad.roll)));//俯仰角弧度值
	
	/*
	attitude_static_rad.roll = Calculate_Angle(accelgyro_data.accel[1],accelgyro_data.accel[2]);		//侧倾角弧度值
	attitude_static_rad.pitch = Calculate_Angle(accelgyro_data.accel[0],(accelgyro_data.accel[1]*sin(attitude_static_rad.roll)\
	+ accelgyro_data.accel[2]*cos(attitude_static_rad.roll)));											//俯仰角弧度值
	*/
	
#ifdef RADIAN_TO_ANGLE
	attitude_static_angle.roll = attitude_static_rad.roll * 57.3;
	attitude_static_angle.pitch = attitude_static_rad.pitch * 57.3;
#endif
	
#ifdef SIMPKALFILTER
	attitude_static_rad.pitch = SimpKalfilter_Pitch(attitude_static_rad.pitch);
	attitude_static_rad.roll = SimpKalfilter_Roll(attitude_static_rad.roll);
#endif
}

double Calculate_Angle(double ture_Hx,double ture_Hy)
{
	double angle;
	if(ture_Hx > 0 && ture_Hy <= 0)
			angle = (pi + atan(-ture_Hx / ture_Hy));
		else if(ture_Hx == 0 && ture_Hy < 0)
			angle = pi;
		else if(ture_Hx < 0 && ture_Hy < 0)
			angle = (pi + atan(-ture_Hx / ture_Hy));
		else if(ture_Hx < 0 && ture_Hy > 0)
			angle = atan(-ture_Hx / ture_Hy);
		else if(ture_Hx == 0 && ture_Hy > 0)
			angle = 0;
		else if(ture_Hx > 0 && ture_Hy > 0)
			angle = (2.0 * pi + atan(-ture_Hx / ture_Hy));
	return angle;
}

/* 坐标变换，将9150的坐标系变换到lis3mdl磁力计的坐标系下 */
void Coordinate_Transformation(void)
{
	int i = 0;
	double temp;
	
	temp = accelgyro_data.accel[1];
	accelgyro_data.accel[1] = -accelgyro_data.accel[0];
	accelgyro_data.accel[0] = temp;
}

void Magnetic_Vector_Projection(void)
{
	/* bmsf 
	magnetic_vector.Bx = filter_magn_measure_val.Bx*cos(attitude_static_rad.pitch) + filter_magn_measure_val.Bz*sin(attitude_static_rad.pitch);
	magnetic_vector.By = -filter_magn_measure_val.Bx*sin(attitude_static_rad.pitch)*sin(attitude_static_rad.roll) + filter_magn_measure_val.By*cos(attitude_static_rad.roll) \
	+ filter_magn_measure_val.Bz*cos(attitude_static_rad.pitch)*sin(attitude_static_rad.roll);
	
	magnetic_vector.Bz = -filter_magn_measure_val.Bx*sin(attitude_static_rad.pitch)*cos(attitude_static_rad.roll) - \
	filter_magn_measure_val.By*sin(attitude_static_rad.roll) + filter_magn_measure_val.Bz*cos(attitude_static_rad.pitch)*cos(attitude_static_rad.roll);
	*/
#ifdef TILT_ANGLE_MAPPING
	/*reference patent*/
	magnetic_vector.Bx = filter_magn_measure_val.Bx*cos(attitude_static_rad.pitch) + filter_magn_measure_val.By*sin(attitude_static_rad.pitch)*sin(attitude_static_rad.roll) \
	+ filter_magn_measure_val.Bz*sin(attitude_static_rad.pitch)*cos(attitude_static_rad.roll);
	magnetic_vector.By = filter_magn_measure_val.By*cos(attitude_static_rad.roll) + filter_magn_measure_val.Bz*sin(attitude_static_rad.roll);
	
	magnetic_vector.Bz = filter_magn_measure_val.Bx*sin(attitude_static_rad.pitch) - filter_magn_measure_val.By*sin(attitude_static_rad.roll)*cos(attitude_static_rad.pitch) + \
	filter_magn_measure_val.Bz*cos(attitude_static_rad.pitch)*cos(attitude_static_rad.roll);
#else
	magnetic_vector.Bx = filter_magn_measure_val.Bx;
	magnetic_vector.By = filter_magn_measure_val.By;
#endif
}

void Magn_Horizontal_Projection(int magn_fd,FILE* to_magn_file)
{
	MAGNETOMETER_DATA old_magn_measure_val;
	old_magn_measure_val.mx = magn_measure_val.mx;
	old_magn_measure_val.my = magn_measure_val.my;
	old_magn_measure_val.mz = magn_measure_val.mz;
	
	Get_Magn_Data(magn_fd);
	
	/* 将采集到的原始数据进行简单滤波 */
	filter_magn_measure_val.Bx = old_magn_measure_val.mx * PROPORTION + magn_measure_val.mx * (1 - PROPORTION);
	filter_magn_measure_val.By = old_magn_measure_val.my * PROPORTION + magn_measure_val.mx * (1 - PROPORTION);
	filter_magn_measure_val.Bz = old_magn_measure_val.mz * PROPORTION + magn_measure_val.mx * (1 - PROPORTION);
	
	Magnetic_Vector_Projection();		//通过倾斜角将平面映射到水平面
}
	
void Get_Magn_Data(int magn_fd)
{
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