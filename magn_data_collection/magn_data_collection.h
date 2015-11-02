#ifndef __MAGN_DATA_COLLECTION
#define __MAGN_DATA_COLLECTION

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

#define	PWR_MGMT_1		0x6B	//电源管理，典型值：0x00(正常启用)
#define	WHO_AM_I		0x75	//IIC地址寄存器(默认数值0x68，只读)

#define	MPU6050_Addr   0xD0	 	 //定义器件在IIC总线中的从地址,根据ALT  ADDRESS地址引脚不同修改

//****************************

#define MAGN_OUT_X_L 0x28
#define MAGN_OUT_X_H 0x29
#define MAGN_OUT_Y_L 0x2A
#define MAGN_OUT_Y_H 0x2B
#define MAGN_OUT_Z_L 0x2C
#define MAGN_OUT_Z_H 0x2D
#define MAGN_STATUS_REG	0x27

#define PROPORTION 0.9
#define pi 3.1415926

#endif