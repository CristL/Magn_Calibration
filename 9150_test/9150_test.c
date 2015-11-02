#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <wait.h>
#include <math.h>
#include "9150_test.h"

//#define RAW_TO_VIEW
#define SIMPKALFILTER
//#define RADIAN_TO_ANGLE
//#define ATTITUDE
//#define HEADING
#define HEADING_CONTRAST

#ifdef SIMPKALFILTER
double Q = 0.001; 
double R = 0.45; //0.45//R值表示的是测量方程协方差值，这里的0.7448 是在未滤波的输出条件下对采集值进行协方差求解得到的值;
#endif


ACCELGYRO accelgyro_data;
ACCELGYRO accelgyro_offset;
ATTITUDE_ attitude_static_rad;
ATTITUDE_ attitude_static_angle;
MAGNETOMETER_DATA magn_measure_val;
MAGN_CHANGE_TO_DOUBLE filter_magn_measure_val;
MAGN_CHANGE_TO_DOUBLE magn_offset;
MAGN_CHANGE_TO_DOUBLE magn_raw_filter;		//磁力计原始数据滤波后值
MAGN_CHANGE_TO_DOUBLE magnetic_vector;
CONFIG_TEMP_VALUE g_equation_temp_value;	//临时变量
ELLIPSE_PARAMETER g_ellipse_parameter;		//椭圆方程相关参数


short accelgyro_raw[6];
short magn9150_raw[3];
double heading_raw;
double heading;
double cos_phi;
double sin_phi;
char accfilter_flag = 0;

int main(int argc, char **argv)
{
	int mpu9150_fd;
	unsigned int m = 1903;
	FILE *from_fd,*to_fd;
	int magn_fd;
	double *ax,*ay;
	double count_B2;//水平磁场分量

	mpu9150_fd = open("/dev/mpu9150",O_RDWR);
	if (mpu9150_fd < 0){
		printf("can't open /dev/mpu9150\n");
		return -1;
	}else{
		printf("Now,you are in /dev/mpu9150\n");
	}
	
	
	magn_fd = open("/dev/magn",O_RDWR);
	if (magn_fd < 0){
		printf("can't open /dev/magn\n");
		return -1;
	}else{
		printf("Now,you are in /dev/magn\n");
	}

	ax = (double *)malloc(sizeof (double)* m);
	if(NULL==ax){
		printf("error to malloc ax\n");
		exit(1);
	}
	ay = (double *)malloc(sizeof (double)* m);
	if(NULL==ay){
		printf("error to malloc ay\n");
		exit(1);
	}

	from_fd = fopen("/data_file.txt","r");
	if(from_fd == NULL){
		printf("open '/data_file.txt' failed!");
		return -1;
	}

	for(int i=0;i<m;i++){
		fscanf(from_fd,"%lf%*c%lf",&ax[i],&ay[i]);									//中间跳过空格,水平分量值读入数组中
	}
	fclose(from_fd);

	count_B2 = ((MAX(ax,m) - MIN(ax,m))/2.0)*((MAX(ax,m) - MIN(ax,m))/2.0);
	g_ellipse_parameter.F = -count_B2;
	
	g_equation_temp_value.bb = (double *)malloc(sizeof (double)* 5);
	for(int i=0;i<5;i++){
		g_equation_temp_value.bb[i] = count_B2;										//初始化线性常量
	}
		
	g_equation_temp_value = equation_temp_value(ax,ay,g_equation_temp_value,m);		//相关运算，得到设定方程的临时参数

	free(ay);
	ay = NULL;
	free(ax);
	ax = NULL;

	Gauss(g_equation_temp_value.V,g_equation_temp_value.bb,5);						//计算得到拟合椭圆参数，存储在bb[0]~bb[4]中

	free(g_equation_temp_value.V);
	g_ellipse_parameter = ellipse_parameter_calculate(g_equation_temp_value,g_ellipse_parameter);//计算得到椭圆倾斜角等量

	free(g_equation_temp_value.bb);

	cos_phi=cos(g_ellipse_parameter.orientation_rad);
	sin_phi=sin(g_ellipse_parameter.orientation_rad);
	
	to_fd = fopen("/magn_data_output.txt","w+");
	if(to_fd == NULL){
		printf("open '/magn_data_output.txt' failed!");
		return -1;
	}
	
	sleep(2);
	Accel_Correct(mpu9150_fd);				//消除固有误差（放置不水平等问题）
	while(1){
		Get_AccelGyro_Data(mpu9150_fd);
		//Get_9150Magn_Data(mpu9150_fd);	//未驱动使用；

		Coordinate_Transformation();		//将9150坐标系变换到磁力计lis3mdl坐标系;

		Calculate_PitchRoll();				//计算俯仰角和侧倾角
		
		Calculate_Heading(magn_fd,to_fd);
		Printf_AccelGyroMagn_Output();
		usleep(20*1000);
	}
	
	close(mpu9150_fd);
	close(magn_fd);
	fclose(to_fd);
	
	return 0;
}

void Get_9150Magn_Data(int fd){
	unsigned char buf[6];
	
	buf[0] = MAGN_XOUT_L;
	read(fd,buf,1);
	buf[1] = MAGN_XOUT_H;
	read(fd,&buf[1],1);
	buf[2] = MAGN_YOUT_L;
	read(fd,&buf[2],1);
	buf[3] = MAGN_YOUT_H;
	read(fd,&buf[3],1);
	buf[4] = MAGN_ZOUT_L;
	read(fd,&buf[4],1);
	buf[5] = MAGN_ZOUT_H;
	read(fd,&buf[5],1);

	magn9150_raw[0] = ((short) ((buf[1] << 8) | buf[0]));
	magn9150_raw[1] = ((short) ((buf[3] << 8) | buf[2]));
	magn9150_raw[2] = ((short) ((buf[5] << 8) | buf[4]));
	
	}


void Get_AccelGyro_Data(int fd){
	int i = 0;
	unsigned char buf[12];
	
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
			accelgyro_data.accel[i] = accelgyro_raw[i] / 163.84;//加速度计的量程为+-2g，这里表示的为重力加速度g的100分之一
		}			
		i++;
	}
}

void Accel_Correct(int fd)
{
	unsigned char i=0;
	unsigned char numAcc=200;

	double Sumaccx=0;
	double Sumaccy=0;
	double Sumaccz=0;							  //加速度计校正中间变量

	for(i=0;i<numAcc;i++)
	{		
		Get_AccelGyro_Data(fd);
		Sumaccx += accelgyro_data.accel[0];
		Sumaccy += accelgyro_data.accel[1];
		//Sumaccz += accelgyro_data.accel[2];
		Sumaccz += sqrt(accelgyro_data.accel[0]*accelgyro_data.accel[0] + accelgyro_data.accel[1]*accelgyro_data.accel[1] + accelgyro_data.accel[2]*accelgyro_data.accel[2]);
	}	
	accelgyro_offset.accel[0] += Sumaccx/numAcc;//得到加速度计基准
	accelgyro_offset.accel[1] += Sumaccy/numAcc;
	accelgyro_offset.accel[2] += Sumaccz/numAcc;//重力加速度g值	
}

void Magn_Z_Correct(int magn_fd)
{
	unsigned char i=0;
	unsigned char numMagn=200;

	double sumMagn_z=0.0;					  //磁力计总和校正中间变量

	for(i=0;i<numMagn;i++)
	{
		Get_Magn_Data(magn_fd);
		sumMagn_z += magn_measure_val.mz;
	}	
	magn_offset.Bz += sumMagn_z/numMagn;	//得到磁力计Z轴基准
}
	

void Printf_AccelGyroMagn_Output(void)
{
#ifdef RAW_TO_VIEW
	printf("%.2lf,%.2lf,%.2lf,%.2lf,%.2lf,%.2lf\n" \
		,accelgyro_data.accel[0],accelgyro_data.accel[1],accelgyro_data.accel[2],accelgyro_data.gyro[0],accelgyro_data.gyro[1],accelgyro_data.gyro[2]);
#elif defined(ATTITUDE)
	printf("%.2lf,%.2lf,%.2lf\n",attitude_static_angle.pitch,attitude_static_angle.roll,attitude_static_angle.yaw);
#elif defined(HEADING_CONTRAST)
	printf("%.2lf,%.2lf\n",heading_raw,heading);
#elif defined(HEADING)
	printf("%.2lf\n",heading);
#else
	printf("%d,%d,%d,%d,%d,%d,%d,%d,%d\n",accelgyro_raw[0],accelgyro_raw[1],accelgyro_raw[2],accelgyro_raw[3],accelgyro_raw[4],accelgyro_raw[5], \
	magn_measure_val.mx,magn_measure_val.my,magn_measure_val.mz);
#endif
}

void Calculate_PitchRoll(void)
{
	attitude_static_rad.roll = atan(accelgyro_data.accel[1]/accelgyro_data.accel[2]);//侧倾角弧度值
	attitude_static_rad.pitch = atan(-accelgyro_data.accel[0]/(accelgyro_data.accel[1]*sin(attitude_static_rad.roll) + accelgyro_data.accel[2]* cos(attitude_static_rad.roll)));//俯仰角弧度值
#ifdef RADIAN_TO_ANGLE
	attitude_static_angle.roll = attitude_static_rad.roll * 57.3;
	attitude_static_angle.pitch = attitude_static_rad.pitch * 57.3;
#endif

#ifdef SIMPKALFILTER
	attitude_static_rad.pitch = SimpKalfilter_Pitch(attitude_static_rad.pitch);
	attitude_static_rad.roll = SimpKalfilter_Roll(attitude_static_rad.roll);
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
	accelgyro_data.accel[2] = sqrt(accelgyro_offset.accel[2]*accelgyro_offset.accel[2] - accelgyro_data.accel[1]*accelgyro_data.accel[1] - accelgyro_data.accel[0]*accelgyro_data.accel[0]);
	temp = accelgyro_data.accel[1];
	accelgyro_data.accel[1] = -accelgyro_data.accel[0];
	accelgyro_data.accel[0] = temp;
}

/* 磁矢量投影，将倾斜的目标投影到水平面，得到磁矢量的水平投影值 */
void Magnetic_Vector_Projection(void)
{
	/* write by myself 
	magnetic_vector.Bx = filter_magn_measure_val.Bx*cos(attitude_static_rad.pitch) + filter_magn_measure_val.Bz*sin(attitude_static_rad.pitch);
	magnetic_vector.By = -filter_magn_measure_val.Bx*sin(attitude_static_rad.pitch)*sin(attitude_static_rad.roll) + filter_magn_measure_val.By*cos(attitude_static_rad.roll) \
	+ filter_magn_measure_val.Bz*cos(attitude_static_rad.pitch)*sin(attitude_static_rad.roll);
	
	magnetic_vector.Bz = -filter_magn_measure_val.Bx*sin(attitude_static_rad.pitch)*cos(attitude_static_rad.roll) - \
	filter_magn_measure_val.By*sin(attitude_static_rad.roll) + filter_magn_measure_val.Bz*cos(attitude_static_rad.pitch)*cos(attitude_static_rad.roll);
	*/
	
	/*reference patent*/
	magnetic_vector.Bx = filter_magn_measure_val.Bx*cos(attitude_static_rad.pitch) + filter_magn_measure_val.By*sin(attitude_static_rad.pitch)*sin(attitude_static_rad.roll) \
	+ filter_magn_measure_val.Bz*sin(attitude_static_rad.pitch)*cos(attitude_static_rad.roll);
	magnetic_vector.By = filter_magn_measure_val.By*cos(attitude_static_rad.roll) + filter_magn_measure_val.Bz*sin(attitude_static_rad.roll);
	
	magnetic_vector.Bz = filter_magn_measure_val.Bx*sin(attitude_static_rad.pitch) - filter_magn_measure_val.By*sin(attitude_static_rad.roll)*cos(attitude_static_rad.pitch) + \
	filter_magn_measure_val.Bz*cos(attitude_static_rad.pitch)*cos(attitude_static_rad.roll);
	
}

void Get_Magn_Data(int magn_fd)
{
	unsigned char buf[6];
	MAGNETOMETER_DATA old_magn_measure_val;
	old_magn_measure_val.mx = magn_measure_val.mx;
	old_magn_measure_val.my = magn_measure_val.my;
	old_magn_measure_val.mz = magn_measure_val.mz;

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
	
	filter_magn_measure_val.Bx = old_magn_measure_val.mx * PROPORTION + magn_measure_val.mx * (1 - PROPORTION);
	filter_magn_measure_val.By = old_magn_measure_val.my * PROPORTION + magn_measure_val.mx * (1 - PROPORTION);
	filter_magn_measure_val.Bz = old_magn_measure_val.mz * PROPORTION + magn_measure_val.mx * (1 - PROPORTION);

}

void Calculate_Heading(int magn_fd,FILE* to_fd)
{
	double ture_Hx0;
	double ture_Hy0;
	double ture_Hx;//实际磁场值
	double ture_Hy;
	
	Get_Magn_Data(magn_fd);
	
#ifdef HEADING_CONTRAST
	//magn_raw_filter.Bx = filter_magn_measure_val.Bx;
	magn_raw_filter.Bx = magn_measure_val.mx;
	magn_raw_filter.By = magn_measure_val.my;
	//magn_raw_filter.By = filter_magn_measure_val.By;
	heading_raw = Heading_Calculate(magn_raw_filter.Bx,magn_raw_filter.By);
#endif
		
	//fprintf(to_fd,"%d %d %d ",magn_measure_val.mx,magn_measure_val.my,magn_measure_val.mz);
	
	Magnetic_Vector_Projection();
	
	//fprintf(to_fd,"%.2lf %.2lf %.2lf\n",magnetic_vector.Bx,magnetic_vector.By,magnetic_vector.Bz);
	
	ture_Hx = magnetic_vector.Bx;
	ture_Hy = magnetic_vector.By;
	
	ture_Hx0 = ((ture_Hx - g_ellipse_parameter.X0)*cos_phi + (ture_Hy - g_ellipse_parameter.Y0)*sin_phi) / g_ellipse_parameter.a_radius;
	ture_Hy0 = (-(ture_Hx - g_ellipse_parameter.X0)*sin_phi + (ture_Hy - g_ellipse_parameter.Y0)*cos_phi) / g_ellipse_parameter.b_radius;
	ture_Hx = ture_Hx0*cos_phi - ture_Hy0*sin_phi;
	ture_Hy = ture_Hy0*cos_phi + ture_Hx0*sin_phi;
	
	heading = Heading_Calculate(ture_Hx,ture_Hy);
	heading = SimpKalfilter(heading);
}

double Heading_Calculate(double ture_Hx,double ture_Hy)
{
	double heading;
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
	return heading;
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