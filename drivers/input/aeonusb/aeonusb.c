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

#include "aeonusb.h"
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
#include <linux/spinlock.h>
#include <linux/reboot.h>

#define AEONUSB_NAME	"mtk-aeonusb"
#define MTK_KP_WAKESOURCE	/* this is for auto set wake up source */

#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#define AEONUSB_PROC_NAME        	"AEON_AEONUSB"
static struct proc_dir_entry *aeonusb_proc_entry;

static unsigned int aeonusb_irqnr;
unsigned int aeonusbgpiopin, aeonusbdebounce;
unsigned int aeonusb_eint_type;
static bool aeonusb_suspend;
static char call_status;
//struct wake_lock aeonusb_suspend_lock;	/* For suspend usage */

static int aeonusb_pdrv_probe(struct platform_device *pdev);
static int aeonusb_pdrv_remove(struct platform_device *pdev);
#ifndef USE_EARLY_SUSPEND
static int aeonusb_pdrv_suspend(struct platform_device *pdev, pm_message_t state);
static int aeonusb_pdrv_resume(struct platform_device *pdev);
#endif
extern void mt_irq_set_polarity(unsigned int irq, unsigned int polarity);

extern unsigned int aeon_gpio_get(const char *name);
extern int aeon_gpio_set(const char *name);

//static irqreturn_t aeonusb_fcover_eint_handler(void);

#define FCOVER_OPEN        (1)
#define FCOVER_CLOSE       (0)

static struct work_struct fcover_work;
static struct workqueue_struct *fcover_workqueue = NULL;
//static DEFINE_SPINLOCK(fcover_lock);
int new_fcover_aeonusb = FCOVER_CLOSE;
static int fcover_close_flag = FCOVER_OPEN;
extern struct input_dev *accdet_input_dev;
static int aeonusb_stat = 0;

unsigned char aeon_usbl_state = 0; //0:plugout; 1:charger plugin; 2:otg plugin; 3:hdmi plugin
unsigned char aeon_usbr_state = 0; //0:plugout; 1:charger plugin; 2:otg plugin;
extern void mt6360_enable_uug_on(bool en);
extern int tcpc_otg_enable(void);
extern int tcpc_otg_disable(void);
//extern void aeon_plug_in_out_handler(bool en, bool ignore);
extern void mt_usb_disconnect(void);
extern void mt_usb_connect(void);
//extern void mt_usbhost_connect(void);
extern void usb_gpio_init(void);

extern unsigned char IsAeonHdmi;

static struct mutex read_mutex;

extern int g_boot_mode; 
static int is_aeonusb_state(void)
{
	return gpio_get_value(aeonusbgpiopin);
}
EXPORT_SYMBOL(is_aeonusb_state);

int get_aeonusb_state(void)
{
        return fcover_close_flag;
}
EXPORT_SYMBOL(get_aeonusb_state);

static void aeonusbfcover_key_handler(struct work_struct *work)
{
	new_fcover_aeonusb = gpio_get_value(aeonusbgpiopin); 
	printk("aeonusb==>aeonusbfcover_key_handler new_fcover_aeonusb=%d , fcover_close_flag=%d\n ",new_fcover_aeonusb, fcover_close_flag);

	printk("aeonusbfcover_key_handler g_boot_mode== %d  \n",g_boot_mode);
	if(g_boot_mode !=0)
		return;
	
	if (aeon_gpio_get("hdmi_hpd_state") == 1){
		return;
	}
	
	printk("aeonusb=1>aeonusbfcover_key_handler new_fcover_aeonusb=%d , fcover_close_flag=%d\n",new_fcover_aeonusb, fcover_close_flag);
	
	if(fcover_close_flag != new_fcover_aeonusb)
	{
		//spin_lock(&fcover_lock);
		mutex_lock(&read_mutex);
		fcover_close_flag = new_fcover_aeonusb;
		//spin_unlock(&fcover_lock);
		
		if(fcover_close_flag == FCOVER_CLOSE) //GPIO94 low , usbl plugin
		{
			aeon_gpio_set("wireless_chg_en_high");//GPIO99
			aeon_gpio_set("sil9022_hdmi_pwren1");//GPIO126
			aeon_gpio_set("lt8912_pwren_high");//GPIO18
			mdelay(1000);
			
			if (aeon_gpio_get("hdmi_hpd_state") == 0)	//GPIO98 low, usbl is otg
			{
				mdelay(1000);
				if (aeon_gpio_get("hdmi_hpd_state") == 1) {
					printk("aeon usbl is hdmi\n");
					aeon_usbl_state = 3;
				} else {
				//usbl otg plugin
				printk("aeon usbl is otg\n");
				if (aeon_usbr_state == 0){
					if (IsAeonHdmi == 1) {
						
					} else {
						printk("usbl otg, usbr null\n");
						aeon_gpio_set("usb_sw_high");//GPIO13
						aeon_gpio_set("hdmi_usb_sw_high");//GPIO127
						tcpc_otg_enable();
						mt6360_enable_uug_on(0);
					}
				} else if (aeon_usbr_state == 1){
					printk("usbl otg, usbr charging\n");
					aeon_gpio_set("usb_sw_high");//GPIO13
					aeon_gpio_set("hdmi_usb_sw_high");//GPIO127
					mt_usb_disconnect();
					
					tcpc_otg_enable();
					mt6360_enable_uug_on(0);		
				} else if (aeon_usbr_state == 2){
					printk("usbl otg, usbr otg\n");
				}
				//aeon_gpio_set("hdmi_usb_sw_high");//GPIO127
				//tcpc_otg_enable();
				aeon_usbl_state = 2;
				}
			}
			else
			{
				//usbl hdmi plugin
				printk("aeon usbl is hdmi\n");
				aeon_usbl_state = 3;
			}
		}
		else  
		{			
		    printk("aeonusb usbl is plugout\n");
			aeon_gpio_set("wireless_chg_en_low");//GPIO99
			aeon_gpio_set("sil9022_hdmi_pwren0");//GPIO126
			aeon_gpio_set("lt8912_pwren_low");
			tcpc_otg_disable();
			if (aeon_usbr_state == 0){
				//aeon_gpio_set("usb_sw_low");//GPIO13
				//aeon_gpio_set("hdmi_usb_sw_low");//GPIO127
				mt6360_enable_uug_on(1);
				usb_gpio_init();
			} else if (aeon_usbr_state == 1){
				aeon_gpio_set("usb_sw_low");//GPIO13
				aeon_gpio_set("hdmi_usb_sw_low");//GPIO127
				mt6360_enable_uug_on(1);
				mt_usb_connect();
			} else if (aeon_usbr_state == 2){
				
			}
			
			aeon_usbl_state = 0;
			
			printk("%s:IsAeonHdmi=%d\n", __func__, IsAeonHdmi);
			if (IsAeonHdmi == 1) {
				kernel_restart(NULL);
			}
		}
		
		mutex_unlock(&read_mutex);
	}
	if(new_fcover_aeonusb)
		irq_set_irq_type(aeonusb_irqnr, IRQ_TYPE_LEVEL_LOW);
	else
		irq_set_irq_type(aeonusb_irqnr, IRQ_TYPE_LEVEL_HIGH);
		
	gpio_set_debounce(aeonusbgpiopin, aeonusbdebounce);	
	enable_irq(aeonusb_irqnr);
}

static irqreturn_t aeonusb_fcover_eint_handler(int irq, void *dev_id)
{
	printk("aeonusb_fcover_eint_handler ..\n");
	
	if (aeon_gpio_get("hdmi_hpd_state") == 1){
		return IRQ_NONE;
	}
	
	
	disable_irq_nosync(aeonusb_irqnr);
	queue_work(fcover_workqueue, &fcover_work);	
	return IRQ_HANDLED;
}


static const struct of_device_id aeonusb_of_match[] = {
	{.compatible = "mediatek, usbl_det-eint"},
	{},
};

static struct platform_driver aeonusb_pdrv = {
	.probe = aeonusb_pdrv_probe,
	.remove = aeonusb_pdrv_remove,
#ifndef USE_EARLY_SUSPEND
	.suspend = aeonusb_pdrv_suspend,
	.resume = aeonusb_pdrv_resume,
#endif
	.driver = {
		   .name = AEONUSB_NAME,
		   .owner = THIS_MODULE,
		   .of_match_table = aeonusb_of_match,
		   },
};
static ssize_t  aeonusb_write(struct file *file, const char *buffer, size_t count,loff_t *data)
{
	return 0;
}

static ssize_t aeonusb_read(struct file *filp, char __user *buffer, size_t size, loff_t *ppos)
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
	 aeonusb_stat = is_aeonusb_state();
	if(aeonusb_stat)
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

static const struct file_operations aeonusb_proc_fops = {
	.write = aeonusb_write,
	.read = aeonusb_read,
};

static int aeonusb_pdrv_probe(struct platform_device *pdev)
{
	//int err = 0;
	int ret = 0;
	u32 ints[2] = { 0, 0 };
	u32 ints1[2] = { 0, 0 };
	struct device_node *node = NULL;

	printk("aeonusb probe start!!!\n");
	
	if (g_boot_mode == KERNEL_POWER_OFF_CHARGING_BOOT || g_boot_mode == LOW_POWER_OFF_CHARGING_BOOT){
		return -1;
	}
	
	//__set_bit(EV_KEY, accdet_input_dev->evbit);
	//__set_bit(KEY_F11, accdet_input_dev->keybit);
	//__set_bit(KEY_F12, accdet_input_dev->keybit);	

	//wake_lock_init(&aeonusb_suspend_lock, WAKE_LOCK_SUSPEND, "aeonusb wakelock");
	fcover_workqueue = create_singlethread_workqueue("fcover");

	//fcover_close_flag = gpio_get_value(aeonusbgpiopin);
	
	node = of_find_matching_node(node, aeonusb_of_match);
	if (node) {
		of_property_read_u32_array(node, "debounce", ints, ARRAY_SIZE(ints));
		of_property_read_u32_array(node, "interrupts", ints1, ARRAY_SIZE(ints1));
		//aeonusbgpiopin = ints[0];
		//printk("aeonusbgpiopin 1st : %d\n",aeonusbgpiopin);
		aeonusbgpiopin = of_get_named_gpio(node, "deb-gpios", 0); /* you need modified DWS files */
		printk("aeonusbgpiopin 1st : %d\n",aeonusbgpiopin);
		aeonusbdebounce = ints[0];
		aeonusb_eint_type = ints1[1];
        printk("%s(): aeonusbdebounce=%d, aeonusb_eint_type = %d\n",__func__,
                       aeonusbdebounce,aeonusb_eint_type);
		gpio_set_debounce(aeonusbgpiopin, aeonusbdebounce);
		
		fcover_close_flag = gpio_get_value(aeonusbgpiopin);
		printk("%s:aeonusb_fcover_eint_handler done.., fcover_close_flag=%d\n", __func__, fcover_close_flag);
		if (aeon_gpio_get("hdmi_hpd_state") == 1)
		{
			fcover_close_flag = 0;
			aeon_usbl_state = 3;
			printk("%s:fcover_close_flag=%d\n", __func__, fcover_close_flag);
		}
		
		aeonusb_irqnr = irq_of_parse_and_map(node, 0);
		ret = request_irq(aeonusb_irqnr, (irq_handler_t)aeonusb_fcover_eint_handler, IRQF_TRIGGER_NONE, "usbl_det-eint", NULL);
		if (ret != 0) {
			printk("[aeonusb]EINT IRQ LINE NOT AVAILABLE\n");
		} else {
			printk("[aeonusb]aeonusb set EINT finished, aeonusb_irqnr=%d, aeonusbgpiopin=%d, aeonusbdebounce=%d, aeonusb_eint_type=%d\n",
				     aeonusb_irqnr, aeonusbgpiopin, aeonusbdebounce, aeonusb_eint_type);
		}
	} else {
		printk("[aeonusb]%s can't find compatible node\n", __func__);
	}
	
	mutex_init(&read_mutex);
	
	INIT_WORK(&fcover_work, aeonusbfcover_key_handler);

//	wake_lock_init(&aeonusb_suspend_lock, WAKE_LOCK_SUSPEND, "aeonusb wakelock");
	
	aeonusb_proc_entry = proc_create(AEONUSB_PROC_NAME, 0777, NULL, &aeonusb_proc_fops);
	if (NULL == aeonusb_proc_entry)
	{
		printk("proc_create %s failed\n", AEONUSB_PROC_NAME);
	}
	
    enable_irq_wake(aeonusb_irqnr);
	enable_irq(aeonusb_irqnr);
	
	printk("====%s success=====.\n" , __func__);
	return 0;
}

/* should never be called */
static int aeonusb_pdrv_remove(struct platform_device *pdev)
{
	mutex_destroy(&read_mutex);
	return 0;
}

#ifndef USE_EARLY_SUSPEND
static int aeonusb_pdrv_suspend(struct platform_device *pdev, pm_message_t state)
{
	aeonusb_suspend = true;
#ifdef MTK_KP_WAKESOURCE
	if (call_status == 2) {
		printk("aeonusb_early_suspend wake up source enable!! (%d)\n", aeonusb_suspend);
	} else {
		kpd_wakeup_src_setting(0);
		printk("aeonusb_early_suspend wake up source disable!! (%d)\n", aeonusb_suspend);
	}
#endif
	printk("suspend!! (%d)\n", aeonusb_suspend);
	return 0;
}

static int aeonusb_pdrv_resume(struct platform_device *pdev)
{
	aeonusb_suspend = false;
#ifdef MTK_KP_WAKESOURCE
	if (call_status == 2) {
		printk("aeonusb_early_suspend wake up source enable!! (%d)\n", aeonusb_suspend);
	} else {
		printk("aeonusb_early_suspend wake up source resume!! (%d)\n", aeonusb_suspend);
		kpd_wakeup_src_setting(1);
	}
#endif
	printk("resume!! (%d)\n", aeonusb_suspend);
	return 0;
}
#else
#define aeonusb_pdrv_suspend	NULL
#define aeonusb_pdrv_resume		NULL
#endif

#ifdef USE_EARLY_SUSPEND
static void aeonusb_early_suspend(struct early_suspend *h)
{
	aeonusb_suspend = true;
#ifdef MTK_KP_WAKESOURCE
	if (call_status == 2) {
		printk("aeonusb_early_suspend wake up source enable!! (%d)\n", aeonusb_suspend);
	} else {
		/* aeonusb_wakeup_src_setting(0); */
		printk("aeonusb_early_suspend wake up source disable!! (%d)\n", aeonusb_suspend);
	}
#endif
	printk("early suspend!! (%d)\n", aeonusb_suspend);
}

static void aeonusb_early_resume(struct early_suspend *h)
{
	aeonusb_suspend = false;
#ifdef MTK_KP_WAKESOURCE
	if (call_status == 2) {
		printk("aeonusb_early_resume wake up source resume!! (%d)\n", aeonusb_suspend);
	} else {
		printk("aeonusb_early_resume wake up source enable!! (%d)\n", aeonusb_suspend);
		/* aeonusb_wakeup_src_setting(1); */
	}
#endif
	printk("early resume!! (%d)\n", aeonusb_suspend);
}

static struct early_suspend aeonusb_early_suspend_desc = {
	//.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1,
	.suspend = aeonusb_early_suspend,
	.resume = aeonusb_early_resume,
};
#endif

#ifdef CONFIG_MTK_SMARTBOOK_SUPPORT
#ifdef CONFIG_HAS_SBSUSPEND
static struct sb_handler aeonusb_sb_handler_desc = {
	//.level = SB_LEVEL_DISABLE_KEYPAD,
	.plug_in = sb_aeonusb_enable,
	.plug_out = sb_aeonusb_disable,
};
#endif
#endif

static int __init aeonusb_mod_init(void)
{
	int r;

	r = platform_driver_register(&aeonusb_pdrv);
	if (r) {
		printk("register driver failed (%d)\n", r);
		return r;
	}
#ifdef USE_EARLY_SUSPEND
	register_early_suspend(&aeonusb_early_suspend_desc);
#endif

#ifdef CONFIG_MTK_SMARTBOOK_SUPPORT
#ifdef CONFIG_HAS_SBSUSPEND
	register_sb_handler(&aeonusb_sb_handler_desc);
#endif
#endif

	return 0;
}

/* should never be called */
static void __exit aeonusb_mod_exit(void)
{
}

module_init(aeonusb_mod_init);
module_exit(aeonusb_mod_exit);
MODULE_AUTHOR("yucong.xiong <yucong.xiong@mediatek.com>");
MODULE_DESCRIPTION("MTK Keypad (aeonusb) Driver v0.4");
MODULE_LICENSE("GPL");
