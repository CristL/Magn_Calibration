#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include "mpu9150_drv.h"

//#define printk printk
static int major;
static struct class *class;
static struct i2c_client *mpu9150_client;

/*输入buf[0] : addr
  *输出buf[0] : data
*/
static ssize_t mpu9150_read(struct file * file, char __user *buf, size_t count, loff_t *off)
{
	unsigned char addr, data;
	
	copy_from_user(&addr, buf, 1);
	data = i2c_smbus_read_byte_data(mpu9150_client, addr);
	copy_to_user(buf, &data, 1);
	return 1;
}
/*buf[0] : addr
  *buf[1] : data
*/
static ssize_t mpu9150_write(struct file *file, const char __user *buf, size_t count, loff_t *off)
{
	unsigned char ker_buf[2];
	unsigned char addr, data;

	copy_from_user(ker_buf, buf, 2);
	addr = ker_buf[0];
	data = ker_buf[1];

	printk("addr = 0x%02x, data = 0x%02x\n", addr, data);

	if (!i2c_smbus_write_byte_data(mpu9150_client, addr, data))
		return 2;
	else
		return -EIO;
}

/*
static int lis3_i2c_init(struct lis3lv02d *lis3)
{
	u8 reg;
	int ret;

	lis3->read(lis3, WHO_AM_I, &reg);
	if (reg != lis3->whoami)
		printk(KERN_ERR "lis3: power on failure\n");

	// power up the device 
	ret = lis3->read(lis3, CTRL_REG1, &reg);
	if (ret < 0)
		return ret;

	if (lis3->whoami == WAI_3DLH)
		reg |= CTRL1_PM0 | CTRL1_Xen | CTRL1_Yen | CTRL1_Zen;
	else
		reg |= CTRL1_PD0 | CTRL1_Xen | CTRL1_Yen | CTRL1_Zen;
	return lis3->write(lis3, CTRL_REG1, reg);
}
*/

static int mpu9150_i2c_init(void)

{
	printk("\n The function i2c_smbus_write_byte_data is start!\n");
	i2c_smbus_write_byte_data(mpu9150_client, PWR_MGMT_1, 0x00);//解除休眠状态
	i2c_smbus_write_byte_data(mpu9150_client, SMPLRT_DIV, 0x01);//取样率2分频，1k/2=500hz,采样率是根据gyro的输出频率来定的，
																//Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
	i2c_smbus_write_byte_data(mpu9150_client, CONFIG, 0x02);//低通滤波?94hz	accel带宽94Hz,延时3ms;gyro带宽98Hz，延时2.8ms,Fs 1kHz
	i2c_smbus_write_byte_data(mpu9150_client, GYRO_CONFIG, 0x18);//0x18  2000dps 65536/4000 = 16.384 LSB /（度每秒）
	i2c_smbus_write_byte_data(mpu9150_client, ACCEL_CONFIG, 0x01);//2g    0x11 +-8g,	5Hz高通滤波?
	printk("\n The function i2c_smbus_write_byte_data is over!\n");
	return 0;
	//i2c_smbus_write_byte_data(mpu9150_client, CTRL_REG5, 0x0);
	//i2c_smbus_write_byte_data(mpu9150_client, INT_CFG, 0xE5);
	//i2c_smbus_write_byte_data(mpu9150_client, INT1_THS_L, 0x0);
	//i2c_smbus_write_byte_data(mpu9150_client, INT1_THS_H, 0x0);
	
	/*
	unsigned char reg;

		reg |= CTRL1_TEMP_EN | CTRL1_OM | CTRL1_DO | CTRL1_ST;		
		
	printk("\n you are in the function lis3_i2c_init!\n");
	
	if(!i2c_smbus_write_byte_data(mpu9150_client, CTRL_REG1, reg)){
		printk("\n The function i2c_smbus_write_byte_data is over!\n");
		i2c_smbus_write_byte_data(mpu9150_client, CTRL_REG2, 0x0);
		i2c_smbus_write_byte_data(mpu9150_client, CTRL_REG3, 0x03);
		i2c_smbus_write_byte_data(mpu9150_client, CTRL_REG4, 0x0);
		i2c_smbus_write_byte_data(mpu9150_client, CTRL_REG5, 0x0);
		//i2c_smbus_write_byte_data(mpu9150_client, INT_CFG, 0xE5);
		i2c_smbus_write_byte_data(mpu9150_client, INT1_THS_L, 0x0);
		i2c_smbus_write_byte_data(mpu9150_client, INT1_THS_H, 0x0);
		return 0;
	}
	else
		return -EIO;	
	*/
}

static struct file_operations mpu9150_fops = {
	
	.owner = THIS_MODULE,
	.read  = mpu9150_read,
	.write = mpu9150_write,

};

static int __devinit mpu9150_probe(struct i2c_client *client,
				  const struct i2c_device_id *id)
{
	printk("%s %s %d\n", __FILE__, __FUNCTION__, __LINE__);
	
	mpu9150_client = client;

	major = register_chrdev(0,"mpu9150",&mpu9150_fops);
	class = class_create(THIS_MODULE,"mpu9150");
	device_create(class,NULL,MKDEV(major,0),NULL,"mpu9150");
	
	printk("\n Now you have create a device!\n");
	
	mpu9150_i2c_init();
	
	return 0;
}

static int __devexit mpu9150_remove(struct i2c_client *client)
{
	printk("%s %s %d\n", __FILE__, __FUNCTION__, __LINE__);
	device_destroy(class,MKDEV(major,0));
	class_destroy(class);
	unregister_chrdev(major,"mpu9150");
	
	return 0;
}

static const struct i2c_device_id mpu9150_id_table[] = {
	{ "mpu9150", 0 },
};


/* 1. 分配/设置i2c_driver */
static struct i2c_driver mpu9150_driver = {
	.driver	= {
		.name	= "mpu9150",
		.owner	= THIS_MODULE,
	},
	.probe		= mpu9150_probe,
	.remove		= __devexit_p(mpu9150_remove),
	.id_table	= mpu9150_id_table,
};

static int mpu9150_drv_init(void)
{
	/* 2. 注册i2c_driver */
	i2c_add_driver(&mpu9150_driver);
	
	return 0;
}

static void mpu9150_drv_exit(void)
{
	i2c_del_driver(&mpu9150_driver);
}


module_init(mpu9150_drv_init);
module_exit(mpu9150_drv_exit);
MODULE_LICENSE("GPL");