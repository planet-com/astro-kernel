/*
 * Copyright (C) 2010 MediaTek, Inc.
 *
 * Author: Terry Chang <terry.chang@mediatek.com>
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

#include "function_key.h"
//#include <linux/wakelock.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/clk.h>
//#include <linux/switch.h>
#include <linux/workqueue.h>
//#include <linux/irqchip/mt-eic.h>
#include <linux/of_gpio.h>//of_get_named_gpio
#include <mt-plat/mtk_boot.h>

#define FUNCTION_KEY_NAME	"mtk-function_key"
#define MTK_KP_WAKESOURCE	/* this is for auto set wake up source */

#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#define FUNCTION_KEY_PROC_NAME        	"AEON_FUNCTION_KEY"
static struct proc_dir_entry *function_key_proc_entry;

static unsigned int function_key_irqnr;
unsigned int function_keygpiopin, function_keydebounce;
unsigned int function_key_eint_type;
static bool function_key_suspend;
static char call_status;
//struct wake_lock function_key_suspend_lock;	/* For suspend usage */

static int function_key_pdrv_probe(struct platform_device *pdev);
static int function_key_pdrv_remove(struct platform_device *pdev);
#ifndef USE_EARLY_SUSPEND
static int function_key_pdrv_suspend(struct platform_device *pdev, pm_message_t state);
static int function_key_pdrv_resume(struct platform_device *pdev);
#endif
extern void mt_irq_set_polarity(unsigned int irq, unsigned int polarity);

//static irqreturn_t function_key_fcover_eint_handler(void);

#define FCOVER_OPEN        (1)
#define FCOVER_CLOSE       (0)
#if 0
static struct switch_dev fcover_data;
#endif
static struct work_struct fcover_work;
static struct workqueue_struct *fcover_workqueue = NULL;
static DEFINE_SPINLOCK(fcover_lock);
int new_function_key_fcover = FCOVER_CLOSE;
static int fcover_close_flag = FCOVER_CLOSE;
extern struct input_dev *accdet_input_dev;
static int function_key_stat = 0;
#if 0
extern struct input_dev *kpd_input_dev;
#endif
struct pinctrl * function_key_pinctrl;
//struct pinctrl_state * pins_eint_int1;
#if	0
extern void battery_uisoc_update(void);
#endif
extern int g_boot_mode; 
static int is_function_key_state(void)
{
	return gpio_get_value(function_keygpiopin);
}
EXPORT_SYMBOL(is_function_key_state);

int get_function_key_state(void)
{
        return fcover_close_flag;
}
EXPORT_SYMBOL(get_function_key_state);

static void function_keyfcover_key_handler(struct work_struct *work)
{
	new_function_key_fcover = gpio_get_value(function_keygpiopin);
	printk("function_key==>function_keyfcover_key_handler new_function_key_fcover=%d , fcover_close_flag=%d\n ",new_function_key_fcover, fcover_close_flag);

	printk("function_keyfcover_key_handler g_boot_mode== %d  \n",g_boot_mode);
	if(g_boot_mode !=0)
		return;
	
	printk("function_key=1>function_keyfcover_key_handler new_function_key_fcover=%d , fcover_close_flag=%d\n",new_function_key_fcover, fcover_close_flag);
	
	if(fcover_close_flag != new_function_key_fcover)
	{
		spin_lock(&fcover_lock);
		fcover_close_flag = new_function_key_fcover;
		spin_unlock(&fcover_lock);
		
		if(fcover_close_flag == FCOVER_CLOSE)
		{			  
            input_report_key(accdet_input_dev, KEY_F10, 1);
			input_sync(accdet_input_dev); 
		}
		else  
		{			
			input_report_key(accdet_input_dev, KEY_F10, 0);
			input_sync(accdet_input_dev);	
		}
	#if 0
		switch_set_state((struct switch_dev *)&fcover_data, fcover_close_flag);
	#endif
	}
	if(new_function_key_fcover)
		irq_set_irq_type(function_key_irqnr, IRQ_TYPE_LEVEL_LOW);
	else
		irq_set_irq_type(function_key_irqnr, IRQ_TYPE_LEVEL_HIGH);
		
	gpio_set_debounce(function_keygpiopin, function_keydebounce);	
	enable_irq(function_key_irqnr);
}

static irqreturn_t function_key_fcover_eint_handler(int irq, void *dev_id)
{
	printk("function_key_fcover_eint_handler ..\n");
	disable_irq_nosync(function_key_irqnr);
	queue_work(fcover_workqueue, &fcover_work);	
	return IRQ_HANDLED;
}


static const struct of_device_id function_key_of_match[] = {
	{.compatible = "mediatek, function_key-eint"},
	{},
};

static struct platform_driver function_key_pdrv = {
	.probe = function_key_pdrv_probe,
	.remove = function_key_pdrv_remove,
#ifndef USE_EARLY_SUSPEND
	.suspend = function_key_pdrv_suspend,
	.resume = function_key_pdrv_resume,
#endif
	.driver = {
		   .name = FUNCTION_KEY_NAME,
		   .owner = THIS_MODULE,
		   .of_match_table = function_key_of_match,
		   },
};
static ssize_t  function_key_write(struct file *file, const char *buffer, size_t count,loff_t *data)
{
	return 0;
}

static ssize_t function_key_read(struct file *filp, char __user *buffer, size_t size, loff_t *ppos)
{
    char *page = NULL;
    char *ptr = NULL;
    int err = -1;
    size_t len = 0;

    page = kmalloc(128, GFP_KERNEL);   
    if (!page) 
    {       
        kfree(page);        
        return -ENOMEM; 
    } 
    ptr = page; 
	 function_key_stat = is_function_key_state();
	if(function_key_stat)
		ptr += sprintf(ptr, "1\n");
	else
		ptr += sprintf(ptr, "0\n");
	
    len = ptr - page;               
    if(*ppos >= len)
    {     
        kfree(page);      
        return 0;     
    } 
    err = copy_to_user(buffer,(char *)page,len);          
    *ppos += len;     
    if(err) 
    {     
        kfree(page);        
        return err;   
    } 
    kfree(page);  
    return len;   
}

static const struct file_operations function_key_proc_fops = {
	.write = function_key_write,
	.read = function_key_read,
};
static int function_key_pdrv_probe(struct platform_device *pdev)
{
	//int err = 0;
	int ret = 0;
	u32 ints[2] = { 0, 0 };
	u32 ints1[2] = { 0, 0 };
	struct device_node *node = NULL;

	printk("function_key probe start!!!\n");
	
	if (g_boot_mode == KERNEL_POWER_OFF_CHARGING_BOOT || g_boot_mode == LOW_POWER_OFF_CHARGING_BOOT){
		return -1;
	}
	
	__set_bit(EV_KEY, accdet_input_dev->evbit);
	__set_bit(KEY_F10, accdet_input_dev->keybit);
#if 0
	__set_bit(EV_KEY, kpd_input_dev->evbit);
	__set_bit(KEY_POWER, kpd_input_dev->keybit);
	//__set_bit(KEY_SLEEP, kpd_input_dev->keybit);
	
#endif
	//wake_lock_init(&function_key_suspend_lock, WAKE_LOCK_SUSPEND, "function_key wakelock");
	fcover_workqueue = create_singlethread_workqueue("fcover");
	INIT_WORK(&fcover_work, function_keyfcover_key_handler);

	fcover_close_flag = gpio_get_value(function_keygpiopin);
	
	node = of_find_matching_node(node, function_key_of_match);
	if (node) {
		of_property_read_u32_array(node, "debounce", ints, ARRAY_SIZE(ints));
		of_property_read_u32_array(node, "interrupts", ints1, ARRAY_SIZE(ints1));
		//function_keygpiopin = ints[0];
		//printk("function_keygpiopin 1st : %d\n",function_keygpiopin);
		function_keygpiopin = of_get_named_gpio(node, "deb-gpios", 0); /* you need modified DWS files */
		printk("function_keygpiopin 1st : %d\n",function_keygpiopin);
		function_keydebounce = ints[0];
		function_key_eint_type = ints1[1];
        printk("%s(): function_keydebounce=%d, function_key_eint_type = %d\n",__func__,
                       function_keydebounce,function_key_eint_type);
		gpio_set_debounce(function_keygpiopin, function_keydebounce);
		function_key_irqnr = irq_of_parse_and_map(node, 0);
		ret = request_irq(function_key_irqnr, (irq_handler_t)function_key_fcover_eint_handler, IRQF_TRIGGER_NONE, "function_key-eint", NULL);
		if (ret != 0) {
			printk("[function_key]EINT IRQ LINE NOT AVAILABLE\n");
		} else {
			printk("[function_key]function_key set EINT finished, function_key_irqnr=%d, function_keygpiopin=%d, function_keydebounce=%d, function_key_eint_type=%d\n",
				     function_key_irqnr, function_keygpiopin, function_keydebounce, function_key_eint_type);
		}
	} else {
		printk("[function_key]%s can't find compatible node\n", __func__);
	}
	
//	fcover_close_flag = gpio_get_value(function_keygpiopin);
	printk("function_key_fcover_eint_handler done..\n");

//	wake_lock_init(&function_key_suspend_lock, WAKE_LOCK_SUSPEND, "function_key wakelock");
	
	function_key_proc_entry = proc_create(FUNCTION_KEY_PROC_NAME, 0777, NULL, &function_key_proc_fops);
	if (NULL == function_key_proc_entry)
	{
		printk("proc_create %s failed\n", FUNCTION_KEY_PROC_NAME);
	}	
#if 0
	fcover_data.name = "function_key";
	fcover_data.index = 0;
	fcover_data.state = fcover_close_flag;
	
	err = switch_dev_register(&fcover_data);
	if(err)
	{
		printk("[Accdet]switch_dev_register returned:%d!\n", err);
	}
	switch_set_state((struct switch_dev *)&fcover_data, fcover_close_flag);
#endif
	
    enable_irq_wake(function_key_irqnr);
	enable_irq(function_key_irqnr);
	
	printk("====%s success=====.\n" , __func__);
	return 0;
}

/* should never be called */
static int function_key_pdrv_remove(struct platform_device *pdev)
{
	return 0;
}

#ifndef USE_EARLY_SUSPEND
static int function_key_pdrv_suspend(struct platform_device *pdev, pm_message_t state)
{
	function_key_suspend = true;
#ifdef MTK_KP_WAKESOURCE
	if (call_status == 2) {
		printk("function_key_early_suspend wake up source enable!! (%d)\n", function_key_suspend);
	} else {
		kpd_wakeup_src_setting(0);
		printk("function_key_early_suspend wake up source disable!! (%d)\n", function_key_suspend);
	}
#endif
	printk("suspend!! (%d)\n", function_key_suspend);
	return 0;
}

static int function_key_pdrv_resume(struct platform_device *pdev)
{
	function_key_suspend = false;
#ifdef MTK_KP_WAKESOURCE
	if (call_status == 2) {
		printk("function_key_early_suspend wake up source enable!! (%d)\n", function_key_suspend);
	} else {
		printk("function_key_early_suspend wake up source resume!! (%d)\n", function_key_suspend);
		kpd_wakeup_src_setting(1);
	}
#endif
	printk("resume!! (%d)\n", function_key_suspend);
	return 0;
}
#else
#define function_key_pdrv_suspend	NULL
#define function_key_pdrv_resume		NULL
#endif

#ifdef USE_EARLY_SUSPEND
static void function_key_early_suspend(struct early_suspend *h)
{
	function_key_suspend = true;
#ifdef MTK_KP_WAKESOURCE
	if (call_status == 2) {
		printk("function_key_early_suspend wake up source enable!! (%d)\n", function_key_suspend);
	} else {
		/* function_key_wakeup_src_setting(0); */
		printk("function_key_early_suspend wake up source disable!! (%d)\n", function_key_suspend);
	}
#endif
	printk("early suspend!! (%d)\n", function_key_suspend);
}

static void function_key_early_resume(struct early_suspend *h)
{
	function_key_suspend = false;
#ifdef MTK_KP_WAKESOURCE
	if (call_status == 2) {
		printk("function_key_early_resume wake up source resume!! (%d)\n", function_key_suspend);
	} else {
		printk("function_key_early_resume wake up source enable!! (%d)\n", function_key_suspend);
		/* function_key_wakeup_src_setting(1); */
	}
#endif
	printk("early resume!! (%d)\n", function_key_suspend);
}

static struct early_suspend function_key_early_suspend_desc = {
	//.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1,
	.suspend = function_key_early_suspend,
	.resume = function_key_early_resume,
};
#endif

#ifdef CONFIG_MTK_SMARTBOOK_SUPPORT
#ifdef CONFIG_HAS_SBSUSPEND
static struct sb_handler function_key_sb_handler_desc = {
	//.level = SB_LEVEL_DISABLE_KEYPAD,
	.plug_in = sb_function_key_enable,
	.plug_out = sb_function_key_disable,
};
#endif
#endif

static int __init function_key_mod_init(void)
{
	int r;

	r = platform_driver_register(&function_key_pdrv);
	if (r) {
		printk("register driver failed (%d)\n", r);
		return r;
	}
#ifdef USE_EARLY_SUSPEND
	register_early_suspend(&function_key_early_suspend_desc);
#endif

#ifdef CONFIG_MTK_SMARTBOOK_SUPPORT
#ifdef CONFIG_HAS_SBSUSPEND
	register_sb_handler(&function_key_sb_handler_desc);
#endif
#endif

	return 0;
}

/* should never be called */
static void __exit function_key_mod_exit(void)
{
}

module_init(function_key_mod_init);
module_exit(function_key_mod_exit);
MODULE_AUTHOR("yucong.xiong <yucong.xiong@mediatek.com>");
MODULE_DESCRIPTION("MTK Keypad (function_key) Driver v0.4");
MODULE_LICENSE("GPL");
