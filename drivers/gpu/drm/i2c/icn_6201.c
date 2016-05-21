/* 
 * Copyright (c) 2016  http://www.lemaker.org/
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/ft5x06_ts.h>

#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/interrupt.h>

#include <linux/kernel.h>
#include <linux/semaphore.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/syscalls.h>
#include <linux/unistd.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/timer.h>


#define DEBUG    1
#include <linux/device.h>	


struct ts_event {
	u16 au16_x[CFG_MAX_TOUCH_POINTS];	/*x coordinate */
	u16 au16_y[CFG_MAX_TOUCH_POINTS];	/*y coordinate */
	u8 au8_touch_event[CFG_MAX_TOUCH_POINTS];	/*touch event:
					0 -- down; 1-- contact; 2 -- contact */
	u8 au8_finger_id[CFG_MAX_TOUCH_POINTS];	/*touch ID */
	u16 pressure;
	u8 touch_point;
};

struct ft5x0x_ts_data {
	unsigned int irq;
	unsigned int x_max;
	unsigned int y_max;
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct ts_event event;
	struct ft5x0x_platform_data *pdata;
#ifdef CONFIG_PM
	struct early_suspend *early_suspend;
#endif
};

#define FTS_POINT_UP		0x01
#define FTS_POINT_DOWN		0x00
#define FTS_POINT_CONTACT	0x02

#define ICN6201_NAME	"icn6201"



struct ICM6201_setting_table 
{    
	unsigned cmd;    
	unsigned char count;    
	unsigned char para_list[64];
};

/*
{0x20,2,{0x20, 0x00}},

{0x21,2,{0x21, 0x58}},
{0x22,2,{0x22, 0x24}},

{0x23,2,{0x23, 0x82}},
{0x24,2,{0x24, 0x1E}},
{0x25,2,{0x25, 0x82}},
{0x26,2,{0x26, 0x00}},
{0x27,2,{0x27, 0x1E}},

{0x28,2,{0x28, 0x0A}},
{0x29,2,{0x29, 0x23}},
{0x34,2,{0x34, 0x80}},	//buffer
{0x36,2,{0x36, 0x82}},	//buffer

{0xB5,2,{0xB5, 0xA0}}, 
{0x5C,2,{0x5C, 0xFF}},	//delay

{0x13,2,{0x13, 0x10}},	//8bit, 6bit del 10(8) 00(6)

{0x56,2,{0x56, 0x93}}, //0x90 extern clk , 0x92 inter clk

{0x6B,2,{0x6B, 0x21}}, // lvds clk

{0x69,2,{0x69, 0x1D}}, //lvds clk

{0xB6,2,{0xB6, 0x20}},

{0x51,2,{0x51, 0x20}}, //pll 

{0x09,2,{0x09, 0x10}}  //disply on 


*/

/*
{0x20,2,{0x20, 0x00}},

{0x21,2,{0x21, 0x58}},
{0x22,2,{0x22, 0x24}},

{0x23,2,{0x23, 0xE6}},
{0x24,2,{0x24, 0x3C}},
{0x25,2,{0x25, 0x02}},
{0x26,2,{0x26, 0x00}},
{0x27,2,{0x27, 0x14}},

{0x28,2,{0x28, 0x0A}},
{0x29,2,{0x29, 0x14}},
{0x34,2,{0x34, 0x80}},	//buffer
{0x36,2,{0x36, 0xE6}},	//buffer

{0xB5,2,{0xB5, 0xA0}}, 
{0x5C,2,{0x5C, 0xFF}},	//delay

{0x13,2,{0x13, 0x10}},	//8bit, 6bit del 10(8) 00(6)

{0x56,2,{0x56, 0x90}}, //0x90 extern clk , 0x92 inter clk

{0x6B,2,{0x6B, 0x21}}, // lvds clk

{0x69,2,{0x69, 0x1D}}, //lvds clk

{0xB6,2,{0xB6, 0x20}},

{0x51,2,{0x51, 0x20}}, //pll 

{0x09,2,{0x09, 0x10}}  //disply on 


*/




static struct ICM6201_setting_table icm6201_initialization_setting[] = 
{
#if 1
	{0x20,2,{0x20, 0x00}},
	
	{0x21,2,{0x21, 0x58}},
	{0x22,2,{0x22, 0x24}},
	
	{0x23,2,{0x23, 0xE6}},
	{0x24,2,{0x24, 0x3C}},
	{0x25,2,{0x25, 0x02}},
	{0x26,2,{0x26, 0x00}},
	{0x27,2,{0x27, 0x14}},
	
	{0x28,2,{0x28, 0x0A}},
	{0x29,2,{0x29, 0x14}},
	{0x34,2,{0x34, 0x80}},	//buffer
	{0x36,2,{0x36, 0xE6}},	//buffer
	
	{0xB5,2,{0xB5, 0xA0}}, 
	{0x5C,2,{0x5C, 0xFF}},	//delay
	
	{0x13,2,{0x13, 0x10}},	//8bit, 6bit del 10(8) 00(6)
	
	{0x56,2,{0x56, 0x90}}, //0x90 extern clk , 0x92 inter clk
	
	{0x6B,2,{0x6B, 0x21}}, // lvds clk
	
	{0x69,2,{0x69, 0x1D}}, //lvds clk
	
	{0xB6,2,{0xB6, 0x20}},
	
	{0x51,2,{0x51, 0x20}}, //pll 
	
	{0x09,2,{0x09, 0x10}}  //disply on 

#else
	{0x20,2,{0x20, 0x00}},

	{0x21,2,{0x21, 0x58}},
	{0x22,2,{0x22, 0x24}},

	{0x23,2,{0x23, 0x50}},
	{0x24,2,{0x24, 0x40}},
	{0x25,2,{0x25, 0x50}},
	{0x26,2,{0x26, 0x04}},
	{0x27,2,{0x27, 0x14}},

	{0x28,2,{0x28, 0x23}},
	{0x29,2,{0x29, 0x14}},
	{0x34,2,{0x34, 0x80}},  //buffer
	{0x36,2,{0x36, 0x50}},  //buffer

	{0xB5,2,{0xB5, 0xA0}}, 
	{0x5C,2,{0x5C, 0xFF}},  //delay


	{0x56,2,{0x56, 0x93}}, //0x90 extern clk , 0x92 inter clk

	{0x6B,2,{0x6B, 0x21}}, // lvds clk

	{0x69,2,{0x69, 0x1C}}, //lvds clk

	{0x10,2,{0x10, 0x47}},	
	{0x2A,2,{0x2A, 0x41}}, 

    {0xB6,2,{0xB6, 0x20}},

	{0x51,2,{0x51, 0x20}}, //pll 

	{0x09,2,{0x09, 0x10}}  //disply on 

#endif
	
};


/*
*icn6201_i2c_Read-read data and write data by i2c
*@client: handle of i2c
*@writebuf: Data that will be written to the slave
*@writelen: How many bytes to write
*@readbuf: Where to store data read from slave
*@readlen: How many bytes to read
*
*Returns negative errno, else the number of messages executed
*
*
*/
int icn6201_i2c_Read(struct i2c_client *client, char *writebuf,
		    int writelen, char *readbuf, int readlen)
{
	int ret;

	if (writelen > 0) {
		struct i2c_msg msgs[] = {
			{
			 .addr = client->addr,
			 .flags = 0,
			 .len = writelen,
			 .buf = writebuf,
			 },
			{
			 .addr = client->addr,
			 .flags = I2C_M_RD,
			 .len = readlen,
			 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret < 0)
			dev_err(&client->dev, "f%s: i2c read error.\n",
				__func__);
	} else {
		struct i2c_msg msgs[] = {
			{
			 .addr = client->addr,
			 .flags = I2C_M_RD,
			 .len = readlen,
			 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(client->adapter, msgs, 1);
		if (ret < 0)
			dev_err(&client->dev, "%s:i2c read error.\n", __func__);
	}
	return ret;
}
/*write data by i2c*/
int ic6201_i2c_Write(struct i2c_client *client, char *writebuf, int writelen)
{
	int ret;

	struct i2c_msg msg[] = {
		{
		 .addr = client->addr,
		 .flags = 0,
		 .len = writelen,
		 .buf = writebuf,
		 },
	};

	ret = i2c_transfer(client->adapter, msg, 1);
	if (ret < 0)
		dev_err(&client->dev, "%s i2c write error.\n", __func__);
	printk("..ic6201_i2c_Write.........ret = %d\n",ret);
	return ret;
}


static void push_table(struct i2c_client *client, struct ICM6201_setting_table *table, unsigned int count)
{
	unsigned int i;
   for(i = 0; i < count; i++) 
	{	
        unsigned cmd;
        cmd = table[i].cmd;
		ic6201_i2c_Write(client, table[i].para_list, table[i].count);
    }

	/*unsigned char buf[2] = {0};
	buf[0] = 0x20;
	buf[1] = 0x00;


	ic6201_i2c_Write(client, buf, sizeof(buf));*/
}

static void icm6201_init_timeout(unsigned long data)
{
	//struct i2c_client *client = (struct i2c_client *)data;
	
	//push_table(client, icm6201_initialization_setting, sizeof(icm6201_initialization_setting) / sizeof(struct ICM6201_setting_table));
	//printk(".................icm6201_init_timeout .................\n");
}


static int icn6201_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	int err = 0;
	unsigned char uc_reg_value;
	unsigned char uc_reg_addr; 
 	struct device *dev = &client->dev;
 	struct gpio_desc *gpio_pwr_en;
	struct gpio_desc *gpio_bl_en;
	struct gpio_desc *gpio_pwm;
	struct ft5x0x_ts_data *ft5x0x_ts;
	unsigned int irq_no;
	struct timer_list mytimer;


#if 0

	gpio_pwr_en = devm_gpiod_get_optional(dev, "pwr-en", GPIOD_OUT_HIGH);
	if (IS_ERR(gpio_pwr_en))
		return PTR_ERR(gpio_pwr_en);

	if (gpio_pwr_en) {
		mdelay(5);	
		gpiod_set_value_cansleep(gpio_pwr_en, 1);
		printk(".................icn6201_probe ..........0.......\n");
	}

	gpio_bl_en = devm_gpiod_get_optional(dev, "bl-en", GPIOD_OUT_HIGH);
	if (IS_ERR(gpio_bl_en))
		return PTR_ERR(gpio_bl_en);

	if (gpio_bl_en) {
		mdelay(5);	
		gpiod_set_value_cansleep(gpio_bl_en, 1);
	}

	gpio_pwm = devm_gpiod_get_optional(dev, "pwm", GPIOD_OUT_HIGH);
	if (IS_ERR(gpio_pwm))
		return PTR_ERR(gpio_pwm);

	if (gpio_pwm) {
		mdelay(5);	
		gpiod_set_value_cansleep(gpio_pwm, 1);
	}
	
#endif


	
	
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}

	ft5x0x_ts = devm_kzalloc(dev, sizeof(*ft5x0x_ts), GFP_KERNEL);
	
	if (!ft5x0x_ts) {
		err = -ENOMEM;
		goto exit_alloc_data_failed;
	}
	
	i2c_set_clientdata(client, ft5x0x_ts);
	
	//ft5x0x_ts->irq = client->irq;
        ft5x0x_ts->irq = irq_no;	
	ft5x0x_ts->client = client;
	
	//ft5x0x_ts->pdata = pdata;
	ft5x0x_ts->x_max = 800;//pdata->x_max - 1;
	ft5x0x_ts->y_max = 1280;//pdata->y_max - 1;

#if 0
	init_timer(&mytimer);  
	mytimer.expires = jiffies + 2*HZ;

  	mytimer.data = (unsigned long) client;

  	mytimer.function = icm6201_init_timeout;
	add_timer(&mytimer);
#endif	
	mdelay(500);	
	push_table(client, icm6201_initialization_setting, sizeof(icm6201_initialization_setting) / sizeof(struct ICM6201_setting_table));
    

	/*make sure CTP already finish startup process */
	msleep(150);
	


	/*get some register information */
	uc_reg_addr = 0x20;
	icn6201_i2c_Read(client, &uc_reg_addr, 1, &uc_reg_value, 1);
	dev_dbg(&client->dev, "[FTS] Firmware version = 0x%x\n", uc_reg_value);

	err = i2c_smbus_read_i2c_block_data(client,0x20,1,&uc_reg_value);
	printk("...................icn6201_probe .   err = %d, client->addr = 0x%x\n",err,client->addr);
#if 0

	uc_reg_addr = FT5x0x_REG_POINT_RATE;
	icn6201_i2c_Read(client, &uc_reg_addr, 1, &uc_reg_value, 1);
	dev_dbg(&client->dev, "[FTS] report rate is %dHz.\n",
		uc_reg_value * 10);

	uc_reg_addr = FT5X0X_REG_THGROUP;
	icn6201_i2c_Read(client, &uc_reg_addr, 1, &uc_reg_value, 1);
	dev_dbg(&client->dev, "[FTS] touch threshold is %d.\n",
		uc_reg_value * 4);
#endif
	
	printk("icn6201_probe ..................... succssss\n");
	return 0;


exit_irq_request_failed:
	i2c_set_clientdata(client, NULL);
	//kfree(ft5x0x_ts);

exit_alloc_data_failed:
exit_check_functionality_failed:
	return err;
}


static int icn6201_remove(struct i2c_client *client)
{
	struct ft5x0x_ts_data *ft5x0x_ts;
	ft5x0x_ts = i2c_get_clientdata(client);
	input_unregister_device(ft5x0x_ts->input_dev);
	#ifdef CONFIG_PM
	gpio_free(ft5x0x_ts->pdata->reset);
	#endif

	free_irq(client->irq, ft5x0x_ts);
	kfree(ft5x0x_ts);
	i2c_set_clientdata(client, NULL);
	return 0;
}

static const struct i2c_device_id icn6201_id[] = {
	{ICN6201_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, icn6201_id);

static const struct of_device_id inc6201_of_match[] = {
	{ .compatible = "ChipOne,icn6201", },
	{ }
};
MODULE_DEVICE_TABLE(of, inc6201_of_match);


static struct i2c_driver icn6201_driver = {
	.probe = icn6201_probe,
	.remove = icn6201_remove,
	.id_table = icn6201_id,
	.driver = {
		   .name = ICN6201_NAME,
		   .owner = THIS_MODULE,
		   .of_match_table = inc6201_of_match,
		   },
};

static int __init icn6201_init(void)
{
	int ret;
	ret = i2c_add_driver(&icn6201_driver);
	if (ret) {
		printk(KERN_WARNING "Adding icn6201 driver failed "
		       "(errno = %d)\n", ret);
	} else {
		pr_info("Successfully added driver %s\n",
			icn6201_driver.driver.name);
	}
	printk("...................icn6201_init.............................\n");
	return ret;
}

static void __exit icn6201_exit(void)
{
	i2c_del_driver(&icn6201_driver);
}


module_init(icn6201_init);
module_exit(icn6201_exit);

MODULE_AUTHOR("support@lemaker.org");
MODULE_DESCRIPTION("ICN6201 driver");
MODULE_LICENSE("GPL");
