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

#include "uswitch.h"
//#include <linux/wakelock.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/clk.h>
//#include <linux/switch.h>
#include <linux/workqueue.h>
//#include <linux/irqchip/mt-eic.h>
#include <linux/of_gpio.h>//of_get_named_gpio
#include <linux/delay.h>
#include <mt-plat/mtk_boot.h>
#include <linux/spinlock.h>

#define USWITCH_NAME	"mtk-uswitch"
#define MTK_KP_WAKESOURCE	/* this is for auto set wake up source */

#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#define USWITCH_PROC_NAME        	"AEON_USWITCH"
static struct proc_dir_entry *uswitch_proc_entry;

static unsigned int uswitch_irqnr;
unsigned int uswitchgpiopin, uswitchdebounce;
unsigned int uswitch_eint_type;
static bool uswitch_suspend;
static char call_status;
//struct wake_lock uswitch_suspend_lock;	/* For suspend usage */

static int uswitch_pdrv_probe(struct platform_device *pdev);
static int uswitch_pdrv_remove(struct platform_device *pdev);
#ifndef USE_EARLY_SUSPEND
static int uswitch_pdrv_suspend(struct platform_device *pdev, pm_message_t state);
static int uswitch_pdrv_resume(struct platform_device *pdev);
#endif
extern void mt_irq_set_polarity(unsigned int irq, unsigned int polarity);

extern unsigned int aeon_gpio_get(const char *name);
extern int aeon_gpio_set(const char *name);

//static irqreturn_t uswitch_fcover_eint_handler(void);

#define FCOVER_OPEN        (1)
#define FCOVER_CLOSE       (0)

static struct work_struct fcover_work;
static struct workqueue_struct *fcover_workqueue = NULL;
static DEFINE_SPINLOCK(fcover_lock);
int new_fcover_uswitch = FCOVER_CLOSE;
static int fcover_close_flag = FCOVER_OPEN;
extern struct input_dev *accdet_input_dev;
static int uswitch_stat = 0;

extern unsigned char aeon_usbl_state;
extern unsigned char aeon_usbr_state;
extern void mt6360_enable_uug_on(bool en);
extern int check_usbl_type(void);
extern int tcpc_otg_enable(void);
extern int tcpc_otg_disable(void);
extern void mt_usb_disconnect(void);

static struct mutex read_mutex;
extern int g_boot_mode; 
static int is_uswitch_state(void)
{
	return gpio_get_value(uswitchgpiopin);
}
EXPORT_SYMBOL(is_uswitch_state);

int get_uswitch_state(void)
{
        return fcover_close_flag;
}
EXPORT_SYMBOL(get_uswitch_state);

void usb_gpio_init(void)
{
	if ((g_boot_mode == KERNEL_POWER_OFF_CHARGING_BOOT || g_boot_mode == LOW_POWER_OFF_CHARGING_BOOT) 
			&& aeon_gpio_get("hdmi_hpd_state") == 0 && aeon_gpio_get("usbl_charger_state") == 0){
		aeon_gpio_set("usbl_chg_en_low");//GPIO34
	} else {
		aeon_gpio_set("usbl_chg_en_high");//GPIO34
	}
	aeon_gpio_set("sil9022_hdmi_pwren0");//GPIO126
	aeon_gpio_set("hdmi_usb_sw_low");//GPIO127
	aeon_gpio_set("usb_sw_low");//GPIO13
	aeon_gpio_set("wireless_chg_en_low");//GPIO99
}

static void uswitchfcover_key_handler(struct work_struct *work)
{
	int usblType = 0;
	
	new_fcover_uswitch = gpio_get_value(uswitchgpiopin); 
	printk("uswitch==>uswitchfcover_key_handler new_fcover_uswitch=%d , fcover_close_flag=%d\n ",new_fcover_uswitch, fcover_close_flag);

	printk("uswitchfcover_key_handler g_boot_mode== %d  \n",g_boot_mode);
	if(g_boot_mode !=0)
		return;
	
	if (aeon_gpio_get("hdmi_hpd_state") == 1){
		return;
	}
	
	printk("uswitch=1>uswitchfcover_key_handler new_fcover_uswitch=%d , fcover_close_flag=%d\n",new_fcover_uswitch, fcover_close_flag);
	
	if(fcover_close_flag != new_fcover_uswitch)
	{
		//spin_lock(&fcover_lock);
		mutex_lock(&read_mutex);
		fcover_close_flag = new_fcover_uswitch;
		//spin_unlock(&fcover_lock);
		
		if(fcover_close_flag == FCOVER_CLOSE) //GPIO17 low , usbl plugin
		{	
			mdelay(1000);
			if (aeon_gpio_get("hdmi_hpd_state") == 0)	//GPIO98 low, usbl is plugin
			{
				usblType = check_usbl_type();
				if (usblType == 2){
					printk("usb1 is otg or hdmi mode\n");
				} else {
					//usbl charging plugin
					printk("aeon usbl is charger\n");
					//aeon_gpio_set("usbl_chg_en_low");//GPIO34
					if (aeon_usbr_state == 1){
						aeon_gpio_set("usbl_chg_en_high");//GPIO34
						//aeon_gpio_set("wireless_chg_en_high"); //GPIO99
					} else if (aeon_usbr_state == 2){
						printk("usbr otg, aeon usbl is charger plugin\n");
						tcpc_otg_disable();
						aeon_gpio_set("usbl_chg_en_low");//GPIO34
						msleep(100);
						//mt_usb_disconnect();
						tcpc_otg_enable();
						mt6360_enable_uug_on(0);
					} else if (aeon_usbr_state == 0){
						aeon_gpio_set("usbl_chg_en_low");//GPIO34
						printk("usbl charger, usbr null\n");
					}
					aeon_usbl_state = 1;
				}
			}
			else
			{
				//usbl hdmi plugin
				printk("aeon usbl is hdmi\n");
				aeon_gpio_set("usbl_chg_en_high");//GPIO34
				aeon_usbl_state = 3;
			}
		}
		else  
		{		
			printk("aeon usbl is plugout\n");
			if (aeon_usbl_state == 1){ //usbl charger plugout
				if (aeon_usbr_state == 0){
					//aeon_gpio_set("usbl_chg_en_high");//GPIO34
					usb_gpio_init();
				} else if (aeon_usbr_state == 1){
					aeon_gpio_set("usbl_chg_en_high");//GPIO34
				} else if (aeon_usbr_state == 2){
					aeon_gpio_set("usbl_chg_en_high");//GPIO34
					mt6360_enable_uug_on(1);
				}
			}
			aeon_usbl_state = 0;
		}
		mutex_unlock(&read_mutex);
	}
	if(new_fcover_uswitch)
		irq_set_irq_type(uswitch_irqnr, IRQ_TYPE_LEVEL_LOW);
	else
		irq_set_irq_type(uswitch_irqnr, IRQ_TYPE_LEVEL_HIGH);
		
	gpio_set_debounce(uswitchgpiopin, uswitchdebounce);	
	enable_irq(uswitch_irqnr);
}

static irqreturn_t uswitch_fcover_eint_handler(int irq, void *dev_id)
{
	printk("uswitch_fcover_eint_handler ..\n");
	
	if (aeon_gpio_get("hdmi_hpd_state") == 1){
		return IRQ_NONE;
	}
	
	disable_irq_nosync(uswitch_irqnr);
	queue_work(fcover_workqueue, &fcover_work);	
	return IRQ_HANDLED;
}


static const struct of_device_id uswitch_of_match[] = {
	{.compatible = "mediatek, uswitch-eint"},
	{},
};

static struct platform_driver uswitch_pdrv = {
	.probe = uswitch_pdrv_probe,
	.remove = uswitch_pdrv_remove,
#ifndef USE_EARLY_SUSPEND
	.suspend = uswitch_pdrv_suspend,
	.resume = uswitch_pdrv_resume,
#endif
	.driver = {
		   .name = USWITCH_NAME,
		   .owner = THIS_MODULE,
		   .of_match_table = uswitch_of_match,
		   },
};
static ssize_t  uswitch_write(struct file *file, const char *buffer, size_t count,loff_t *data)
{
	return 0;
}

static ssize_t uswitch_read(struct file *filp, char __user *buffer, size_t size, loff_t *ppos)
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
	 uswitch_stat = is_uswitch_state();
	if(uswitch_stat)
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

static const struct file_operations uswitch_proc_fops = {
	.write = uswitch_write,
	.read = uswitch_read,
};
static int uswitch_pdrv_probe(struct platform_device *pdev)
{
	//int err = 0;
	int ret = 0;
	u32 ints[2] = { 0, 0 };
	u32 ints1[2] = { 0, 0 };
	struct device_node *node = NULL;

	printk("uswitch probe start!!!\n");
	
	aeon_gpio_set("wireless_chg_en_low");//GPIO99
	
	if (g_boot_mode == KERNEL_POWER_OFF_CHARGING_BOOT || g_boot_mode == LOW_POWER_OFF_CHARGING_BOOT){
		return -1;
	}
	
	//__set_bit(EV_KEY, accdet_input_dev->evbit);
	//__set_bit(KEY_F11, accdet_input_dev->keybit);
	//__set_bit(KEY_F12, accdet_input_dev->keybit);	

	//wake_lock_init(&uswitch_suspend_lock, WAKE_LOCK_SUSPEND, "uswitch wakelock");
	

	//fcover_close_flag = gpio_get_value(uswitchgpiopin);
	fcover_workqueue = create_singlethread_workqueue("fcover");
	
	node = of_find_matching_node(node, uswitch_of_match);
	if (node) {
		of_property_read_u32_array(node, "debounce", ints, ARRAY_SIZE(ints));
		of_property_read_u32_array(node, "interrupts", ints1, ARRAY_SIZE(ints1));
		//uswitchgpiopin = ints[0];
		//printk("uswitchgpiopin 1st : %d\n",uswitchgpiopin);
		uswitchgpiopin = of_get_named_gpio(node, "deb-gpios", 0); /* you need modified DWS files */
		printk("uswitchgpiopin 1st : %d\n",uswitchgpiopin);
		uswitchdebounce = ints[0];
		uswitch_eint_type = ints1[1];
        printk("%s(): uswitchdebounce=%d, uswitch_eint_type = %d\n",__func__,
                       uswitchdebounce,uswitch_eint_type);
		gpio_set_debounce(uswitchgpiopin, uswitchdebounce);
		uswitch_irqnr = irq_of_parse_and_map(node, 0);
		ret = request_irq(uswitch_irqnr, (irq_handler_t)uswitch_fcover_eint_handler, IRQF_TRIGGER_NONE, "uswitch-eint", NULL);
		if (ret != 0) {
			printk("[uswitch]EINT IRQ LINE NOT AVAILABLE\n");
		} else {
			printk("[uswitch]uswitch set EINT finished, uswitch_irqnr=%d, uswitchgpiopin=%d, uswitchdebounce=%d, uswitch_eint_type=%d\n",
				     uswitch_irqnr, uswitchgpiopin, uswitchdebounce, uswitch_eint_type);
		}
	} else {
		printk("[uswitch]%s can't find compatible node\n", __func__);
	}
	
	fcover_close_flag = gpio_get_value(uswitchgpiopin);
	
	mutex_init(&read_mutex);
	
	INIT_WORK(&fcover_work, uswitchfcover_key_handler);
	printk("uswitch_fcover_eint_handler done..\n");

//	wake_lock_init(&uswitch_suspend_lock, WAKE_LOCK_SUSPEND, "uswitch wakelock");
	
	uswitch_proc_entry = proc_create(USWITCH_PROC_NAME, 0777, NULL, &uswitch_proc_fops);
	if (NULL == uswitch_proc_entry)
	{
		printk("proc_create %s failed\n", USWITCH_PROC_NAME);
	}
	
    enable_irq_wake(uswitch_irqnr);
	enable_irq(uswitch_irqnr);
	
	printk("====%s success=====.\n" , __func__);
	return 0;
}

/* should never be called */
static int uswitch_pdrv_remove(struct platform_device *pdev)
{
	mutex_destroy(&read_mutex);
	return 0;
}

#ifndef USE_EARLY_SUSPEND
static int uswitch_pdrv_suspend(struct platform_device *pdev, pm_message_t state)
{
	uswitch_suspend = true;
#ifdef MTK_KP_WAKESOURCE
	if (call_status == 2) {
		printk("uswitch_early_suspend wake up source enable!! (%d)\n", uswitch_suspend);
	} else {
		kpd_wakeup_src_setting(0);
		printk("uswitch_early_suspend wake up source disable!! (%d)\n", uswitch_suspend);
	}
#endif
	printk("suspend!! (%d)\n", uswitch_suspend);
	return 0;
}

static int uswitch_pdrv_resume(struct platform_device *pdev)
{
	uswitch_suspend = false;
#ifdef MTK_KP_WAKESOURCE
	if (call_status == 2) {
		printk("uswitch_early_suspend wake up source enable!! (%d)\n", uswitch_suspend);
	} else {
		printk("uswitch_early_suspend wake up source resume!! (%d)\n", uswitch_suspend);
		kpd_wakeup_src_setting(1);
	}
#endif
	printk("resume!! (%d)\n", uswitch_suspend);
	return 0;
}
#else
#define uswitch_pdrv_suspend	NULL
#define uswitch_pdrv_resume		NULL
#endif

#ifdef USE_EARLY_SUSPEND
static void uswitch_early_suspend(struct early_suspend *h)
{
	uswitch_suspend = true;
#ifdef MTK_KP_WAKESOURCE
	if (call_status == 2) {
		printk("uswitch_early_suspend wake up source enable!! (%d)\n", uswitch_suspend);
	} else {
		/* uswitch_wakeup_src_setting(0); */
		printk("uswitch_early_suspend wake up source disable!! (%d)\n", uswitch_suspend);
	}
#endif
	printk("early suspend!! (%d)\n", uswitch_suspend);
}

static void uswitch_early_resume(struct early_suspend *h)
{
	uswitch_suspend = false;
#ifdef MTK_KP_WAKESOURCE
	if (call_status == 2) {
		printk("uswitch_early_resume wake up source resume!! (%d)\n", uswitch_suspend);
	} else {
		printk("uswitch_early_resume wake up source enable!! (%d)\n", uswitch_suspend);
		/* uswitch_wakeup_src_setting(1); */
	}
#endif
	printk("early resume!! (%d)\n", uswitch_suspend);
}

static struct early_suspend uswitch_early_suspend_desc = {
	//.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1,
	.suspend = uswitch_early_suspend,
	.resume = uswitch_early_resume,
};
#endif

#ifdef CONFIG_MTK_SMARTBOOK_SUPPORT
#ifdef CONFIG_HAS_SBSUSPEND
static struct sb_handler uswitch_sb_handler_desc = {
	//.level = SB_LEVEL_DISABLE_KEYPAD,
	.plug_in = sb_uswitch_enable,
	.plug_out = sb_uswitch_disable,
};
#endif
#endif

static int __init uswitch_mod_init(void)
{
	int r;

	usb_gpio_init();
	
	r = platform_driver_register(&uswitch_pdrv);
	if (r) {
		printk("register driver failed (%d)\n", r);
		return r;
	}
#ifdef USE_EARLY_SUSPEND
	register_early_suspend(&uswitch_early_suspend_desc);
#endif

#ifdef CONFIG_MTK_SMARTBOOK_SUPPORT
#ifdef CONFIG_HAS_SBSUSPEND
	register_sb_handler(&uswitch_sb_handler_desc);
#endif
#endif

	return 0;
}

/* should never be called */
static void __exit uswitch_mod_exit(void)
{
}

module_init(uswitch_mod_init);
module_exit(uswitch_mod_exit);
MODULE_AUTHOR("yucong.xiong <yucong.xiong@mediatek.com>");
MODULE_DESCRIPTION("MTK Keypad (uswitch) Driver v0.4");
MODULE_LICENSE("GPL");
