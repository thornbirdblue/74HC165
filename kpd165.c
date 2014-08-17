/*******************************************************************************
 *
 *	CorpyRights (C)	disc,2024			All Rights Reserved!
 *
 *	Module:	74HC165 kpd driver
 *
 *	File:	kpd165.c
 *
 *	Author:	thornbird
 *
 *	Date:	2014-08-07
 *
 *	E-mail:	thornbird.blue@gmail.com
 *
 ***********************************************************************************/

/************************************************************************************
 *	History:
 *
 *	Name		Date		Version 	Act
 *---------------------------------------------------------------------------------------
 *	thornbird	2014-08-07	V00		Create
 *
 ***********************************************************************************/

#ifndef __KPD_165_C__
#define __KPD_163_C__

/* headers */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/delay.h>

#include <linux/kthread.h>
#include <linux/wait.h>
#include <linux/timer.h>

#include <linux/input.h>

#include <linux/errno.h>

#include <mach/mt_spi.h>

#include <mach/mt_reg_base.h>
#include <mach/mt_gpio.h>
#include <mach/mt_gpio_core.h>

// Mac
#define KPD165_NAME		"kpd165"
#define KPD165_INPUT_NAME	"kpd165_input"

#define SPI_CS			(GPIO80|0x80000000)			// AC8
#define SPI_CK			(GPIO81|0x80000000)			// AD7
#define SPI_MI			(GPIO82|0x80000000)			// AB9

#define KPD165_NUM		4					// 4 74HC165 CHIPS

#define KEY_SCAN_TIMER		10					//HZ/10 = 100MS
typedef struct kpd165_data{
	int state;
	unsigned int key_val;
	unsigned int key_save_val;
	unsigned char key_scan_val[KPD165_NUM];
	
	int thread_run;
	int wake_up;
}kpd165_data_t;

static kpd165_data_t kpd_data;

static wait_queue_head_t kpd_queue_head;
static struct task_struct *kpd_kthread;
static struct timer_list kpd_timer;

static struct input_dev *kpd165_input_dev;
/********************************************** spi ctrl intf ***************************************************/
static int mt_spi_pins_config(void)
{
	if (mt_set_gpio_mode(SPI_CS,GPIO_MODE_00) != RSUCCESS)
	{
		pr_err("%s(ERROR):set spi cs pin mode error!!!",__func__);
		return -1;
	}

	if (mt_set_gpio_dir(SPI_CS, GPIO_DIR_OUT) != RSUCCESS)
	{
		pr_err("%s(ERROR):set spi cs pin out error!!!",__func__);
		return -1;
	}

	if (mt_set_gpio_pull_enable(SPI_CS, GPIO_PULL_DISABLE) != RSUCCESS)
	{
		pr_err("%s(ERROR):set spi cs pin pull error!!!",__func__);
		return -1;
	}

	if (mt_set_gpio_mode(SPI_CK,GPIO_MODE_00) != RSUCCESS)
	{
		pr_err("%s(ERROR):set spi clk pin mode error!!!",__func__);
		return -1;
	}
	
	if (mt_set_gpio_dir(SPI_CK, GPIO_DIR_OUT) != RSUCCESS)
	{
		pr_err("%s(ERROR):set spi clk pin out error!!!",__func__);
		return -1;
	}
	
	if (mt_set_gpio_pull_enable(SPI_CK, GPIO_PULL_DISABLE) != RSUCCESS)
	{
		pr_err("%s(ERROR):set spi clk pin pull error!!!",__func__);
		return -1;
	}

	if (mt_set_gpio_mode(SPI_MI,GPIO_MODE_00) != RSUCCESS)
	{
		pr_err("%s(ERROR):set spi mi pin mode error!!!",__func__);
		return -1;
	}
	
	if (mt_set_gpio_dir(SPI_MI, GPIO_DIR_IN) != RSUCCESS)
	{
		pr_err("%s(ERROR):set spi mi pin in error!!!",__func__);
		return -1;
	}

	if (mt_set_gpio_pull_enable(SPI_MI, GPIO_PULL_DISABLE) != RSUCCESS)
	{
		pr_err("%s(ERROR):set spi mi pin pull error!!!",__func__);
		return -1;
	}

	return 0;
}

static int spi_cs_high(void)
{
	if (mt_set_gpio_out(SPI_CS, GPIO_OUT_ONE) != RSUCCESS)
	{
		pr_err("%s(ERROR):set spi cs pin high error!!!",__func__);
		return -1;
	}
	return 0;
}
static int spi_cs_low(void)
{
	if (mt_set_gpio_out(SPI_CS, GPIO_OUT_ZERO) != RSUCCESS)
	{
		pr_err("%s(ERROR):set spi cs pin low error!!!",__func__);
		return -1;
	}
	return 0;
}

static int spi_ck_high(void)
{
	if (mt_set_gpio_out(SPI_CK, GPIO_OUT_ONE) != RSUCCESS)
	{
		pr_err("%s(ERROR):set spi cs pin high error!!!",__func__);
		return -1;
	}
	return 0;
}
static int spi_ck_low(void)
{
	if (mt_set_gpio_out(SPI_CK, GPIO_OUT_ZERO) != RSUCCESS)
	{
		pr_err("%s(ERROR):set spi cs pin low error!!!",__func__);
		return -1;
	}
	return 0;
}

static int spi_mi(void)
{
	return mt_get_gpio_in(SPI_MI);
}

static void delay_time()
{
	udelay(1);	
}

static unsigned int spi_read()
{
	int i,j;
	unsigned int val;
	spi_cs_high();
	delay_time();
	spi_cs_low();
	delay_time();
	spi_cs_high();
	delay_time();

	for(i=0;i<KPD165_NUM;i++)
	{
		kpd_data.key_scan_val[i] = 0;
		for(j=0;j<8;j++)
		{
			spi_ck_low();
			delay_time();
			if(spi_mi())
				kpd_data.key_scan_val[i] |= (0x01<<j);
			else
				kpd_data.key_scan_val[i] &= ~(0x01<<j);
			spi_ck_high();
			delay_time();
		}
	}

	spi_cs_low();
	val = kpd_data.key_scan_val[3];
	val <<= 8;
	val += kpd_data.key_scan_val[2];
	val <<= 8;
	val += kpd_data.key_scan_val[1];
	val <<= 8;
	val += kpd_data.key_scan_val[0];

	pr_info("%s:read val is 0x%x!\n",__func__,val);
	return val;
}

static int spi_init()
{
	int rc;

	rc = mt_spi_pins_config();	
	if(rc < 0)
	{
		pr_err("%s(ERROR): init failed!!!",__func__);
		return -1;
	}

	return 0;
}
/**********************************************  kpd process ******************************************************/
int kpd_kthread_run(void *data)
{
	kpd165_data_t *kpd_dat = data;
	pr_debug("%s: will run!\n",__func__);
	do{
		wait_event(kpd_queue_head,1 == kpd_dat->wake_up);
		kpd_dat->key_val = spi_read();
		if(kpd_dat->key_val != 0xffffffff)
			kpd_dat->key_save_val = kpd_dat->key_val;
		
		kpd_dat->wake_up = 0;

	}while(1 == kpd_data.thread_run);
	return 0;
}

void kpd_timer_run(unsigned long data)
{
	kpd_data.wake_up = 1;
	wake_up(&kpd_queue_head);

	kpd_timer.expires = jiffies + HZ/KEY_SCAN_TIMER;		// KEY scan timer
	add_timer(&kpd_timer);
}

/********************************************* kpd165 input device ************************************************/
static int kpd165_open(struct input_dev *dev)
{
	pr_info("%s: open E!",__func__);
	return 0;
}

static int kpd165_input_device_init(void)
{
	int rc = 0;

	kpd165_input_dev = input_allocate_device();
	if(!kpd165_input_dev)
	{
		pr_err("%s(ERROR): allocate failed!!!",__func__);
		return -1;
	}

	kpd165_input_dev->name = KPD165_INPUT_NAME;
	kpd165_input_dev->open = kpd165_open;

	__set_bit(EV_KEY,kpd165_input_dev->evbit);
	
	rc = input_register_device(kpd165_input_dev);
	if(rc)
	{
		pr_err("%s(ERROR):input device register is failed!!!",__func__);
		return -1;
	}

	return 0;
}

/********************************************** kpd sys intf ******************************************************/
static ssize_t kpd165_dump_scan_key(struct device *dev,struct device_attribute *attr,char *buf)
{
	pr_debug("%s:scan value is 0x%x!",__func__,kpd_data.key_save_val);
	return sprintf(buf,"scan value is 0x%x!\n\n",kpd_data.key_save_val);
}

static DEVICE_ATTR(kpd165_value,0600,kpd165_dump_scan_key,NULL);

static ssize_t kpd165_dump_spi_value(struct device *dev,struct device_attribute *attr,char *buf)
{
	unsigned int val;
	val = spi_read();
	pr_debug("%s:scan value is 0x%x!",__func__,val);
	return sprintf(buf,"scan value is 0x%x!\n\n",val);
}

static DEVICE_ATTR(kpd165_spi_value,0600,kpd165_dump_spi_value,NULL);


/********************************************* kpd driver *********************************************************/
static struct platform_device kpd165_device = {
	.name =	KPD165_NAME,
	.id = -1,
};

static int kpd165_probe(struct platform_device *dev)
{
	int rc = 0;
	pr_info("%s: probe E",__func__);
	
	memset(&kpd_data,0,sizeof(kpd_data));

	rc = device_create_file(&dev->dev,&dev_attr_kpd165_value);
	if(rc < 0)
	{
		pr_err("%s(ERROR):device_create_file is error!!!",__func__);
		return -1;
	}
	rc = device_create_file(&dev->dev,&dev_attr_kpd165_spi_value);
	if(rc < 0)
	{
		pr_err("%s(ERROR):device_create_file spi value is error!!!",__func__);
		return -1;
	}

	kpd_data.thread_run = 1;

	kpd_kthread = kthread_run(kpd_kthread_run,&kpd_data,"kpd165_thread");
	if(IS_ERR(kpd_kthread))
	{
		pr_err("%s(ERROR):create kpd kthread error!!!",__func__);
		return -1;
	}

	// init wait queue header
	init_waitqueue_head(&kpd_queue_head);

	// init timer_list
	init_timer(&kpd_timer);
	kpd_timer.function = kpd_timer_run;
	kpd_timer.expires = jiffies + HZ/KEY_SCAN_TIMER;		// KEY scan timer

	add_timer(&kpd_timer);

	rc = spi_init();
	if(rc < 0)
	{
		pr_err("%s(ERROR):spi init error!!!",__func__);
		return -1;
	}

	rc = kpd165_input_device_init();
	if(rc < 0)
	{
		pr_err("%s(ERROR):kpd165 input device init error!!!",__func__);
		return -1;
	}

	return 0;	
}

static struct platform_driver kpd165_driver ={
	.driver = {
		.name = KPD165_NAME,
		.owner = THIS_MODULE,
	},	
	.probe = kpd165_probe,
};

static int __init kpd165_init(void)
{
	int rc;
	pr_info("%s: init",__func__);
	rc = platform_device_register(&kpd165_device);
	if(rc < 0)
	{
		pr_err("%s(ERROR): platform_device_register is error!!!",__func__);
		return -1;
	}	


	rc = platform_driver_register(&kpd165_driver);
	if(rc < 0)
	{
		pr_err("%s(ERROR): platform_driver_register is error!!!",__func__);
		return -1;
	}	

	return 0;
}

static void __exit kpd165_exit(void)
{
	pr_info("%s: exit!",__func__);
}

module_init(kpd165_init);
module_exit(kpd165_exit);
MODULE_LICENSE("GPL");
#endif	

