#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <wait.h>
#include <math.h>
#include <time.h>
#include "9150_test.h"

//#define RAW_TO_VIEW
#define SIMPKALFILTER
#define COORDINATE_TRANSFORMATION
//#define RADIAN_TO_ANGLE
//#define ATTITUDE
#define HEADING

#define pi 3.1415926
#define min(a,b) ((a<b)?(a):(b))
#define max(a,b) ((a>b)?(a):(b))

#define ot 1e-8

#ifdef SIMPKALFILTER
double Q = 0.001; 
double R = 0.7448; //0.45//R值表示的是测量方程协方差值，这里的0.7448 是在未滤波的输出条件下对采集值进行协方差求解得到的值；
#endif
void accelgyro_data_collection(int);
double SimpKalfilter_Pitch(double z_measured);
double SimpKalfilter_Roll(double z_measured);
void Acc_Correct(int);
void Printf_AccelGyro_Data(void);
void Calculate_PitchRoll(void);
void Coordinate_Transformation(void);
void Get_Magn_Data(int magn_fd,FILE* to_fd);
void Magnetic_Vector_Projection(void);

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
}MAGN_HORIZONTAL;

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

ACCELGYRO accelgyro_data;
ACCELGYRO accelgyro_offset;
ATTITUDE_ attitude_static;
MAGNETOMETER_DATA magn_measure_val;
MAGN_HORIZONTAL magnetic_vector;
CONFIG_TEMP_VALUE g_equation_temp_value;//临时变量
ELLIPSE_PARAMETER g_ellipse_parameter;//椭圆方程相关参数


CONFIG_TEMP_VALUE equation_temp_value(double *ax,double *bx,CONFIG_TEMP_VALUE g_equation_temp_value,unsigned int m);
ELLIPSE_PARAMETER ellipse_parameter_calculate(CONFIG_TEMP_VALUE g_equation_temp_value,ELLIPSE_PARAMETER g_ellipse_parameter);

short accelgyro_raw[6];
double heading;
double cos_phi;
double sin_phi;

int main(int argc, char **argv)
{
	int mpu9150_fd;
	int m = 1150;
	FILE *from_fd,*to_fd;
	int magn_fd;
	int flag = 0;
	double *ax,*ay;
	double count_B2;//水平磁场分量

	mpu9150_fd = open("/dev/mpu9150",O_RDWR);
	if (mpu9150_fd < 0)
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


	ax = (double *)malloc(sizeof (double)* m);
	if(NULL==ax)
	{
		printf("error to malloc ax\n");
		exit(1);
	}
	ay = (double *)malloc(sizeof (double)* m);
	if(NULL==ay)
	{
		printf("error to malloc ay\n");
		exit(1);
	}

	from_fd = fopen("/data_file.txt","r");
	if(from_fd == NULL)
	{
		printf("open '/data_file.txt' failed!");
		return -1;
	}

	for(int i=0;i<m;i++)
	{
		fscanf(from_fd,"%lf%*c%lf",&ax[i],&ay[i]);//中间跳过空格
	}
	fclose(from_fd);


	count_B2 = ((MAX(ax,m) - MIN(ax,m))/2.0)*((MAX(ax,m) - MIN(ax,m))/2.0);
	printf("The value of count_B2 is %lf\n",count_B2);
	g_ellipse_parameter.F = - count_B2;

	
	g_equation_temp_value.bb = (double *)malloc(sizeof (double)* 5);
	for(int i=0;i<5;i++)
	{
		g_equation_temp_value.bb[i] = count_B2;//初始化线性常量
	}
		
	g_equation_temp_value = equation_temp_value(ax,ay,g_equation_temp_value,m);

	free(ay);
	ay = NULL;
	free(ax);
	ax = NULL;


	Gauss(g_equation_temp_value.V,g_equation_temp_value.bb,5);//计算得到拟合椭圆参数，存储在bb[0]~bb[4]中

	free(g_equation_temp_value.V);
	g_ellipse_parameter = ellipse_parameter_calculate(g_equation_temp_value,g_ellipse_parameter);

	free(g_equation_temp_value.bb);
/*
	from_fd = fopen("/data_file.txt","r");
	if(from_fd)
	{
		printf("open '/data_file.txt' failed!");
		return -1;
	}
	for(int i=0;i<m;i++)
	{
		fscanf(from_fd,"%lf%*c%lf",&ax[i],&ay[i]);//中间跳过空格

	}	
	fclose(from_fd);
*/
	cos_phi=cos(g_ellipse_parameter.orientation_rad);
	sin_phi=sin(g_ellipse_parameter.orientation_rad);
	
	to_fd = fopen("/magn_data_output.txt","w+");
	if(to_fd == NULL)
	{
		printf("open '/magn_data_output.txt' failed!");
		return -1;
	}
	
	Acc_Correct(mpu9150_fd);
	if(!flag)
	while(1){
		Get_AccelGyro_Data(mpu9150_fd);//通过flag来判断收集两轴数据，保存在指定文件中;
#ifdef COORDINATE_TRANSFORMATION
		Coordinate_Transformation();//将9150坐标系变换到磁力计lis3mdl坐标系；
#endif
		Calculate_PitchRoll();//计算俯仰角和侧倾角
		
		Get_Magn_Data(magn_fd,to_fd);
		///Printf_AccelGyroMagn_Output();
		usleep(50*1000);
	}
	
	close(mpu9150_fd);
	close(magn_fd);
	fclose(to_fd);
	
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

double SimpKalfilter(double z_measured)
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
	P_temp = P_last + Q;        				 					//方程2
	//calculate the Kalman gain，增益估计方程
	K = P_temp * (1.0/(P_temp + R));     						//方程3
	//correct ，滤波估计方程，得出下一个数值，与滤波协方差方程
	x_est = x_temp_est + K * (z_measured - x_temp_est);  		//方程4
	P = (1- K) * P_temp;    										//方程5
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
	magnetic_vector.Bx = magn_measure_val.mx*cos(attitude_static.pitch) - magn_measure_val.my*sin(attitude_static.pitch);
	magnetic_vector.By = -magn_measure_val.mx*sin(attitude_static.pitch)*cos(attitude_static.roll) + magn_measure_val.my*cos(attitude_static.roll) \
	- magn_measure_val.mz*sin(attitude_static.roll)*cos(attitude_static.pitch);
	magnetic_vector.Bz = magn_measure_val.mx*(sin(attitude_static.pitch)*cos(attitude_static.roll) + sin(attitude_static.roll)*cos(attitude_static.pitch)) - \
	magn_measure_val.my*sin(attitude_static.pitch)*sin(attitude_static.roll) + magn_measure_val.mz*cos(attitude_static.pitch)*cos(attitude_static.roll);
	
	/*reference patent
	magnetic_vector.Bx = magn_measure_val.mx*cos(attitude_static.pitch) + magn_measure_val.my*sin(attitude_static.pitch)*sin(attitude_static.roll) \
	+ magn_measure_val.mz*sin(attitude_static.pitch)*cos(attitude_static.roll);
	magnetic_vector.By = magn_measure_val.my*cos(attitude_static.roll) - magn_measure_val.mz*sin(attitude_static.roll);
	*/
}

void Get_Magn_Data(int magn_fd,FILE* to_fd)
{
	int j = 0;
	unsigned char buf[6];
	double ture_Hx0;
	double ture_Hy0;
	double ture_Hx;//实际磁场值
	double ture_Hy;

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
	
	fprintf(to_fd,"%d %d %d ",magn_measure_val.mx,magn_measure_val.my,magn_measure_val.mz);
	
	Magnetic_Vector_Projection();
	
	fprintf(to_fd,"%.2lf %.2lf %.2lf\n",magnetic_vector.Bx,magnetic_vector.By,magnetic_vector.Bz);
	
	ture_Hx = magnetic_vector.Bx;
	ture_Hy = magnetic_vector.By;
	
	ture_Hx0 = ((ture_Hx - g_ellipse_parameter.X0)*cos_phi + (ture_Hy - g_ellipse_parameter.Y0)*sin_phi) / g_ellipse_parameter.a_radius;
	ture_Hy0 = (-(ture_Hx - g_ellipse_parameter.X0)*sin_phi + (ture_Hy - g_ellipse_parameter.Y0)*cos_phi) / g_ellipse_parameter.b_radius;
	ture_Hx = ture_Hx0*cos_phi - ture_Hy0*sin_phi;
	ture_Hy = ture_Hy0*cos_phi + ture_Hx0*sin_phi;
	
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
	heading = SimpKalfilter(heading);
}

CONFIG_TEMP_VALUE equation_temp_value(double *ax,double *ay,CONFIG_TEMP_VALUE g_equation_temp_value,unsigned int m)
{
	double D[m][5],S[5][m],sum[5];
	
	for(int i=0;i<m;i++)
		for(int j=0;j<5;j++)
			D[i][j]=0.0;

	g_equation_temp_value.V = (double **)malloc(sizeof (double*)* 5);
	for (int i=0;i<5;i++)
	{
		g_equation_temp_value.V[i] = (double *)malloc(sizeof (double)* 5);
	}

	for(int i=0;i<m;i++)
	{
		D[i][0]=ax[i]*ax[i];
		D[i][1]=ax[i]*ay[i];
		D[i][2]=ay[i]*ay[i];
		D[i][3]=ax[i];
		D[i][4]=ay[i];
	}

	for(int i=0;i<m;i++)
		for(int j=0;j<5;j++)
			S[j][i]=D[i][j];//矩阵转置

	for(int i=0;i<5;i++)
		for(int j=0;j<5;j++)	
			{
				g_equation_temp_value.V[i][j]=0.0;
				for(int k=0;k<m;k++)
				{
					g_equation_temp_value.V[i][j]+=S[i][k]*D[k][j];
				}
			}

	for(int i=0;i<5;i++)
	{
		sum[i]=0.0;
		for(int j=0;j<m;j++)
			sum[i]+=D[j][i];
		g_equation_temp_value.bb[i] *= sum[i];
	}//每一列的和
	return g_equation_temp_value;
}


ELLIPSE_PARAMETER ellipse_parameter_calculate(CONFIG_TEMP_VALUE g_equation_temp_value,ELLIPSE_PARAMETER g_ellipse_parameter){
	double cos_phi,sin_phi,pm_test,long_axis,short_axis;
	double A0,B0,C0,D0,E0,X0,Y0;
	g_ellipse_parameter.A0 = g_equation_temp_value.bb[0];
	g_ellipse_parameter.B0 = g_equation_temp_value.bb[1];
	g_ellipse_parameter.C0 = g_equation_temp_value.bb[2];
	g_ellipse_parameter.D0 = g_equation_temp_value.bb[3];
	g_ellipse_parameter.E0 = g_equation_temp_value.bb[4];
	A0 = g_equation_temp_value.bb[0];
	B0 = g_equation_temp_value.bb[1];
	C0 = g_equation_temp_value.bb[2];
	D0 = g_equation_temp_value.bb[3];
	E0 = g_equation_temp_value.bb[4];
	//printf("A0=%lf,B0=%lf,C0=%lf,D0=%lf,E0=%lf\n",A0,B0,C0,D0,E0);

	if(min(abs(B0/A0),abs(B0/C0))>ot)//判断是否为椭圆
	{
		g_ellipse_parameter.orientation_rad=0.5*atan(B0/(A0-C0));//calculate the tilt value
		cos_phi=cos(g_ellipse_parameter.orientation_rad);
		sin_phi=sin(g_ellipse_parameter.orientation_rad);
	}
	else
	{
		g_ellipse_parameter.orientation_rad=0;
		cos_phi=cos(g_ellipse_parameter.orientation_rad);
		sin_phi=sin(g_ellipse_parameter.orientation_rad);
	}
	//if we found an ellipse return it's data
	pm_test=A0*C0;
	if(pm_test>0)
	{
		//make sure coefficients are positive as required
		if(A0<0)
		{
			A0=-A0;
			C0=-C0;
			D0=-D0;
			E0=-E0;

		}
		
		g_ellipse_parameter.X0 = (B0*E0 - 2*C0*D0)/(4*A0*C0 - B0*B0);
		g_ellipse_parameter.Y0 = (B0*D0 - 2*A0*E0)/(4*A0*C0 - B0*B0);
		X0 = g_ellipse_parameter.X0;
		Y0 = g_ellipse_parameter.Y0;
	
		/*a_radius = 2*sqrt(abs(-2.0*F/(A0 + C0 - sqrt(B0*B0 + ((A0 - C0)/F)*((A0 - C0)/F)))));
		b_radius = 2*sqrt(abs(-2.0*F/(A0 + C0 + sqrt(B0*B0 + ((A0 - C0)/F)*((A0 - C0)/F)))));*/
		g_ellipse_parameter.a_radius = sqrt(abs(-(g_ellipse_parameter.F + A0*X0*X0 + B0*X0*Y0 +C0*Y0*Y0 +D0*X0 + E0*Y0)/(A0*cos_phi*cos_phi + B0*cos_phi*sin_phi +C0*sin_phi*sin_phi)));
		g_ellipse_parameter.b_radius = sqrt(abs(-(g_ellipse_parameter.F + A0*X0*X0 + B0*X0*Y0 +C0*Y0*Y0 +D0*X0 + E0*Y0)/(A0*sin_phi*sin_phi - B0*cos_phi*sin_phi +C0*cos_phi*cos_phi)));
		long_axis=2*max(g_ellipse_parameter.a_radius,g_ellipse_parameter.b_radius);
		short_axis=2*min(g_ellipse_parameter.a_radius,g_ellipse_parameter.b_radius);
	}
	

	//printf("a=%lf,b=%lf,phi=%lf,X0=%lf,Y0=%lf,long_axis=%lf,short_axis=%lf\n",a_radius,b_radius,orientation_rad*57.3,X0,Y0,long_axis,short_axis);
	return g_ellipse_parameter;
}
	
void Gauss(double **pCoff,double *pConst,int n)
{
	double t,d;
	int i,j,k,row,flag,*pCol;

	// 临时缓冲区，存放列数
	pCol = (int *)malloc(sizeof (int)*n);
	if(pCol != NULL)
	{
		//消元
		flag = 1;

		for (k=0;k<n-1;k++)
		{
			d = 0.0;
			for (i=k;i<n;i++)
			{
				for(j=k;j<n;j++)
				{
					t = fabs(pCoff[i][j]);
					if (t>d)
					{
						d = t;
						pCol[k] = j;
						row = i;
					}
				}
			}

			if (d==0.0)
				flag = 0;

			//列交换
			else 
			{
				if (pCol[k]!=k)
				{
					for (i=0;i<n;i++)
					{
						t = pCoff[i][k];
						pCoff[i][k] = pCoff[i][pCol[k]];
						pCoff[i][pCol[k]] = t;
					}
				}
				//行交换
				if (row != k)
				{
					for (j=k;j<n;j++)
					{
						t = pCoff[k][j];
						pCoff[k][j] = pCoff[row][j];
						pCoff[row][j] = t;
					}
					
				   t=pConst[k]; 
				   pConst[k]=pConst[row]; 
				   pConst[row]=t;
				}
			}
		

			//没有解
			if(flag == 0)
			{
				free(pCol);
				pCol = NULL;
				//delete [] pCol;
				//cout << "没有解！?" <<endl;
				printf("The question is no solution");
			}

			d = pCoff[k][k];
			for (j=k;j<n;j++)
			{
				pCoff[k][j] = pCoff[k][j]/d;
			}
			
			pConst[k] = pConst[k]/d;

			for (i=k+1;i<n;i++)
			{
				for(j=k+1;j<n;j++)
				{
					pCoff[i][j] = pCoff[i][j]-pCoff[i][k]*pCoff[k][j];
				}
				pConst[i] = pConst[i]-pCoff[i][k]*pConst[k];
			}
		}

			//求解失败
			d = pCoff[n-1][n-1];
			if(d==0)
			{
				//delete [] pCol;
				//cout << "没有解！" <<endl;
				free(pCol);
				pCol = NULL;
				printf("The question is no solution");
			}

			//代入求解
			pConst[n-1] = pConst[n-1]/d;
			for (i=n-2;i>=0;i--)
			{
				t = 0.0;
				for (j=i+1;j<n;j++)
				{
					t = t+pCoff[i][j]*pConst[j];
				}
				pConst[i] =  pConst[i]-t;
			}

			//调整解的位置
			pCol[n-1]=n-1;
			for (k=n-1;k>=0;k--)
			{
				if (pCol[k]!=k)
				{ 
					t=pConst[k]; 
					pConst[k]=pConst[pCol[k]]; 
					pConst[pCol[k]]=t;
				}
			}

			//回收内存
			free(pCol);
			pCol = NULL;					
	}
	else
 
	{	 
		printf("mallocerror!\n");				 
		exit(-1);	 
	}

}

double MAX(double *a,unsigned int m)
{
	double max_val;
	max_val = a[0];
	for(int i=1;i<m;i++)
	{
		if(a[i] > max_val)
			max_val = a[i];
	}
	return max_val;
}

double MIN(double *a,unsigned int m)
{
	double min_val;
	min_val = a[0];
	for(int i=1;i<m;i++)
	{
		if(a[i] < min_val)
			min_val = a[i];
	}
	return min_val;
}