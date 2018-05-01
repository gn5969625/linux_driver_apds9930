#include <linux/module.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/timer.h>
#include <linux/init.h>
#include <linux/serial_core.h>
#include <linux/platform_device.h>
#include <asm/mach/map.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include "apds9930.h"
#include <linux/sysfs.h>

#define ATIME 0xff // 2.7 ms – minimum ALS integration time
#define WTIME 0xff // 2.7 ms – minimum Wait time
#define PTIME 0xff // 2.7 ms – minimum Prox integration time
#define PPULSE 0x0e // Minimum prox pulse count
#define PDRIVE 0 //100mA of LED Power
#define PDIODE 0x20 // CH1 Diode
#define PGAIN 0 //1x Prox gain
#define AGAIN 0 //1x ALS gain
#define WEN 8 // Enable Wait
#define PEN 4 // Enable Prox
#define AEN 2 // Enable ALS
#define PON 1 // Enable Power On

#define DRV_VERSION "V1.0"
static struct i2c_client *apds9930_client;

static int i2c_apds9930_read_len(struct i2c_client *client,unsigned char reg_addr,unsigned char len,unsigned char *buf)
{
	int ret;
        unsigned char txbuf = reg_addr;
        struct i2c_msg msg[] = {
                {client->addr,0,1,&txbuf},
                {client->addr,1,len,buf}
        };
        ret = i2c_transfer(client->adapter,msg,2);
        if(ret < 0) {
                printk("i2c_transfer read len error\n");
                return -1;
        }
        return 0;
}
static int i2c_apds9930_read_byte(struct i2c_client *client,unsigned char reg_addr)
{
	int ret;
	unsigned char txbuf = reg_addr;
	unsigned char rxbuf;
	struct i2c_msg msg[] = {
		{client->addr,0,1,&txbuf},
		{client->addr,I2C_M_RD,1,&rxbuf}
	};
	ret = i2c_transfer(client->adapter,msg,2);
	if(ret < 0) {
		printk("i2c_transfer read error\n");
                return -1;
	}
	return rxbuf;
}
static int i2c_apds9930_write_byte(struct i2c_client *client,unsigned char reg_addr,unsigned char data_buf)
{
	int ret;
	unsigned char txbuf[] = {reg_addr,data_buf};
	struct i2c_msg msg[] = {client->addr,0,2,txbuf};
	ret = i2c_transfer(client->adapter,msg,1);
	if(ret < 0) {
		printk("i2c_transfer write error\n");
		return -1;
	}
	return 0;
}

#define COMMAND_AUTO_INCREMENT_CODE 0xA0
static ssize_t apds9930_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	int CH0_data, CH1_data, Prox_data;
	CH0_data = i2c_smbus_read_word_data(apds9930_client,COMMAND_AUTO_INCREMENT_CODE | 0x14);
        CH1_data = i2c_smbus_read_word_data(apds9930_client,COMMAND_AUTO_INCREMENT_CODE | 0x16);
	Prox_data = i2c_smbus_read_word_data(apds9930_client,COMMAND_AUTO_INCREMENT_CODE | 0x18);
	return sprintf(buf,"CH0_data = %d, CH1_data = %d, Prox_data = %d \n",CH0_data, CH1_data, Prox_data);

	return 0;
}
static ssize_t apds9930_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	return 0;
}
static DEVICE_ATTR(data, S_IRUGO | S_IWUSR, apds9930_show, apds9930_store);
static struct attribute *apds9930_attrs[] = {
    &dev_attr_data.attr,
    NULL
};
static struct attribute_group mydrv_attr_group = {
    .name = "apds9930_drv",
    .attrs = apds9930_attrs,
};
#define COMMAND_CODE 0x80
static int apds9930_dev_init(void)
{
	int res;
	printk("%s called\n", __func__);
	res = i2c_smbus_write_byte_data(apds9930_client, COMMAND_CODE | 0x00, 0);
	res = i2c_smbus_write_byte_data(apds9930_client, COMMAND_CODE | 0x01, ATIME);
	res = i2c_smbus_write_byte_data(apds9930_client, COMMAND_CODE | 0x02, PTIME);
	res = i2c_smbus_write_byte_data(apds9930_client, COMMAND_CODE | 0x03, WTIME);
	res = i2c_smbus_write_byte_data(apds9930_client, COMMAND_CODE | 0x0e, PPULSE);
	res = i2c_smbus_write_byte_data(apds9930_client, COMMAND_CODE | 0x0f, PDRIVE | PDIODE | PGAIN | AGAIN);
	res = i2c_smbus_write_byte_data(apds9930_client, COMMAND_CODE | 0x00, WEN | PEN | AEN | PON);
	mdelay(12);
	//read device id
	res = i2c_smbus_read_byte_data(apds9930_client, COMMAND_CODE | 0x12);
	if(res == 0x39) {
		printk("the device id:%x \n", res);
	}
	return 0;
}
static int apds9930_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
	//struct proc_dir_entry *file;
	int ret;
	dev_dbg(&i2c->dev, "%s\n", __func__);
	if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL))
                return -ENODEV;

	dev_info(&i2c->dev, "chip found, driver version " DRV_VERSION "\n");
	apds9930_client = i2c;
	apds9930_dev_init();
	printk("apds9930 device component found!~\n");
	ret = sysfs_create_group(&i2c->dev.kobj, &mydrv_attr_group);
	return 0;
}
static int apds9930_remove(struct i2c_client *i2c)
{
	sysfs_remove_group(&i2c->dev.kobj, &mydrv_attr_group);
	return 0;
}

static const struct i2c_device_id apds9930_id[] = {  
    { "apds9930", 0},
    {}
};
MODULE_DEVICE_TABLE(i2c, apds9930_id);

static struct of_device_id apds9930_of_match[] = {
        { .compatible = "avago,apds9930" },
        { },
};
//MODULE_DEVICE_TABLE(of, apds9930_of_match);
struct i2c_driver apds9930_driver = {
    .driver = {
        .name           = "apds9930",
        .owner          = THIS_MODULE,
        .of_match_table = of_match_ptr(apds9930_of_match),
//	.of_match_table = apds9930_of_match,
    },
    .probe      = apds9930_probe,
    .remove     = apds9930_remove,
    .id_table   = apds9930_id,
};

static int apds9930_init(void)
{
	return i2c_add_driver(&apds9930_driver);
}

static void apds9930_exit(void)
{
	printk("exit apds9930 driver module");
	i2c_del_driver(&apds9930_driver);
}

module_init(apds9930_init);
module_exit(apds9930_exit);

//module_i2c_driver(apds9930_driver);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Kevin.Shen");
MODULE_DESCRIPTION("A i2c-apds9930 driver for testing module ");
MODULE_VERSION("V1.0");
