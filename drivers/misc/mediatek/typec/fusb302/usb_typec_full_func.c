/*
* Copyright (C) 2016 MediaTek Inc.
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
* See http://www.gnu.org/licenses/gpl-2.0.html for more details.
*/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <typec.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>

#include <linux/gpio.h>


#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/firmware.h>
#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/pci.h>
#include <linux/dma-mapping.h>
#include <linux/gameport.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
//#include <linux/wakelock.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/input.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/miscdevice.h>

#include <linux/proc_fs.h>
#include<linux/slab.h>
#include <linux/uaccess.h>

#include "fusb302.h"

#ifndef USB_TYPE_C
#define USB_TYPE_C

#define K_EMERG	(1<<7)
#define K_QMU	(1<<7)
#define K_ALET		(1<<6)
#define K_CRIT		(1<<5)
#define K_ERR		(1<<4)
#define K_WARNIN	(1<<3)
#define K_NOTICE	(1<<2)
#define K_INFO		(1<<1)
#define K_DEBUG	(1<<0)

#define fusb_printk(level, fmt, args...) do { \
			pr_err("[FUSB302]" fmt, ## args); \
	} while (0)

#define SKIP_TIMER

unsigned int hdmi_plug_in_flag = 0;
//unsigned int keyboardlight_flag = 0;
unsigned int is_rusb_onotg = 0;
//extern void Ext_Speaker_Amp_Change(bool enable);
//extern int AudDrv_GPIO_EXTAMP_Select(int bEnable, int mode);
//extern int AudDrv_GPIO_EXTAMP2_Select(int bEnable, int mode);


//static u32 debug_level = (255 - K_DEBUG);
static struct usbtypc *g_exttypec;
static struct i2c_client *typec_client;
static unsigned int usbid_irqnum;
static unsigned int debounce=0, gpiopin=0;

//extern unsigned int aeon_gpio_get(const char *name);
extern int aeon_gpio_set(const char *name);
//extern void sil9024_test(void);

unsigned int hdmi_det_gpio;
void hdmi_plug(void);

/* /////////////////////////////////////////////////////////////////////////// */
/* Variables accessible outside of the FUSB300 state machine */
/* /////////////////////////////////////////////////////////////////////////// */
static FUSB300reg_t Registers;	/* Variable holding the current status of the FUSB300 registers */
void InitializeFUSB300(void)
{
	fusb_printk(K_DEBUG, "%s\n", __func__);
	FUSB300Read(regDeviceID, 1, &Registers.DeviceID.byte);	/* Read the device ID */
	fusb_printk(K_DEBUG, "%s DeviceID: 0x%x\n", __func__, Registers.DeviceID.byte);
	
	Registers.Mode.byte = 0x10;
	FUSB300Write(regMode, 1, &Registers.Mode.byte);
	
//	FUSB300Read(regSlice, 1, &Registers.Slice.byte);	/* Read the slice */
//	Registers.Mask.byte = 0x00;	/* Do not mask any interrupts */

	/* Clear all interrupt masks (we want to do something with them) */
//	FUSB300Write(regMask, 1, &Registers.Mask.byte);
}

/* /////////////////////////////////////////////////////////////////////////// */
/* FUSB300 I2C Routines */
/* /////////////////////////////////////////////////////////////////////////// */
/* BOOL FUSB300Write(struct usbtypc *typec, unsigned char regAddr, unsigned char length, unsigned char* data) */
int FUSB300Write(unsigned char regAddr, unsigned char length, unsigned char *data)
{
	int i;

	for (i = 0; i < length; i++)
		fusb300_i2c_w_reg8(typec_client, regAddr + i, data[i]);

	return 1;
}

/* BOOL FUSB300Read(struct usbtypc *typec, unsigned char regAddr, unsigned char length, unsigned char* data) */
int FUSB300Read(unsigned char regAddr, unsigned char length, unsigned char *data)
{
	int i;

	for (i = 0; i < length; i++)
		data[i] = fusb300_i2c_r_reg(typec_client, regAddr + i);

	return 1;
}

void hdmi_plug(void)
{
	int ret = 0;
	int hdmi_plug_test = 0;
	struct device_node *node_hdmi_plug;
	
	node_hdmi_plug = of_find_compatible_node(NULL, NULL, "mediatek,hdmi_plug_dts");
	if (!node_hdmi_plug){
		printk("wgx>>>>>>  %s[%d] get node_hdmi_plug fail!\n",__func__,__LINE__);
	} 

///////////  GPIO98
	hdmi_det_gpio = of_get_named_gpio(node_hdmi_plug, "hdmi_det-gpio", 0);
	if (hdmi_det_gpio < 0) {
		printk("wgx>>>>>>  %s[%d] get hdmi_det_gpio fail!\n",__func__,__LINE__);
	}

	ret = gpio_request(hdmi_det_gpio, "hdmi_plug-gpio");
	if (ret)
		printk("wgx>>>>>>  %s[%d] gpio_request hdmi_det_gpio fail, ret = %d \n",__func__,__LINE__,ret);

	gpio_direction_input(hdmi_det_gpio); 
	hdmi_plug_test = __gpio_get_value(hdmi_det_gpio);	
	printk("wgx>>>>>>  %s[%d]fusb300 hdmi_plug_test = %d \n", __func__,__LINE__,hdmi_plug_test);
	
}
static void fusb300_gpio_init(void)
{
	printk("%s\n", __func__);
	aeon_gpio_set("fusb301a_sw_en_high");//GPIO177 high HDMI输出信号切换使能控制信号
	aeon_gpio_set("fusb301a_sw_sel_low");//GPIO176 low  HDMI输出信号1和2切换控制信号
	//aeon_gpio_set("sw7226_en_low");//GPIO72 low
	//aeon_gpio_set("sil9022_hdmi_pwren0");//GPIO160
	//printk("qzshdmi fusb300_gpio_init 1 keyboardlight_flag=%d\n",keyboardlight_flag);
	//if(keyboardlight_flag == 0)
	//aeon_gpio_set("sil9022_hdmi_hplg0");//GPIO178
}

//extern void right_otg_in_report_key(void);
//extern void right_otg_out_report_key(void);
//extern void mt_usb_disconnect(void);
//extern void mt_usb_dev_off(void);
//extern void mt_usb_connect(void);
//extern void tcpc_otg_enable(bool enable);
//extern void mt6370_enable_UUG_ON(bool en);

 void force_to_otg(bool enable)
 {
	#if 0
	 if (enable)
	 {
		 aeon_gpio_set("aeon_usb_lr_sw_high");//usb D+ D- switch to right usb port
		 //mt_usb_disconnect();
		 //mt_usb_dev_off();
		 //aeon_gpio_set("aeon_wake_stm32_low");			 
		 //mdelay(1);				 
		 //aeon_gpio_set("aeon_wake_stm32_high");				 
		 //mdelay(1);				 
		 //aeon_gpio_set("aeon_wake_stm32_low");	 
		 //tcpc_otg_enable(true);
 
	 }
	 else
	 {
		 aeon_gpio_set("aeon_usb_lr_sw_low");//usb D+ D- switch to left usb port
		 //tcpc_vbus_enable(enable);
		 //tcpc_otg_enable(false);
		 //mt_usb_connect();
	 }
	#endif
 }
 #define AEON_USB_CONTROL_SUPPORT
#ifdef AEON_USB_CONTROL_SUPPORT
 #define USB_CONTROL_PROC_NAME        "AEON_USB_CONTROL"
int cmd_contrl=0;
static struct proc_dir_entry *usb_control_entry;
static ssize_t USB_CONTROL_read(struct file *filp, char __user *buffer, size_t size, loff_t *ppos)
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
       
   if(cmd_contrl == 0){
	  ptr += sprintf(ptr, "0\n");
   }else if(cmd_contrl==1){
	  ptr += sprintf(ptr, "1\n");
   }else if(cmd_contrl==2){
	  ptr += sprintf(ptr, "2\n");
   }else if(cmd_contrl==3){
	  ptr += sprintf(ptr, "3\n");
   }else if(cmd_contrl==4){
	  ptr += sprintf(ptr, "4\n");
   }else if(cmd_contrl==5){
	  ptr += sprintf(ptr, "5\n");
   }  
		   
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

static ssize_t  USB_CONTROL_write(struct file *file, const char *buffer, size_t count,loff_t *data)
 {
         char Buf[4];
         int stat,ret=0;
         printk("USB_CONTROL_write\n");
         if (copy_from_user(Buf, buffer, 4)){
                  return -EFAULT;
         }
         ret = sscanf(Buf,"%d",&stat);
         printk("USB_CONTROL_write stat=%d\n",stat);
		 cmd_contrl = stat;
         if(stat == 0){
		 	force_to_otg(FALSE);
         }
         else if(stat == 1){
			force_to_otg(TRUE);
         }else if(stat == 2){
	//		aeon_gpio_set("sil9022_hdmi_hplg0");//GPIO178
         } else if(stat == 3){
	//		aeon_gpio_set("sil9022_hdmi_hplg1");//GPIO178
         } else if(stat == 4){
			//mt6370_enable_UUG_ON(0);
         } else if(stat == 5){
			//mt6370_enable_UUG_ON(1);
         }
		 
         return count;
 }
static const struct file_operations usb_control_proc_fops = {
		.read  = USB_CONTROL_read,
		.write = USB_CONTROL_write
};
#endif



static DEFINE_MUTEX(typec_lock);

//extern int aeon_otg_enable; 
void fusb300_eint_work(struct work_struct *data)
{
//	struct usbtypc *typec = container_of(to_delayed_work(data), struct usbtypc, fsm_work);
	unsigned int usb1_id_state = 0;
	//unsigned int HDMI_id_state = 0;

	unsigned char CCXstate = 0;

	mutex_lock(&typec_lock);
	
	printk("====%s:zhaolong debug x600 USB1=====2019040222117\n", __func__);
	
	usb1_id_state = gpio_get_value(gpiopin);//HDMI插入方向检测信号
	
	printk("====%s:zhaolong debug x600 USB1=gpiopin=%d=usb1_id_state=%d===\n", __func__,gpiopin,usb1_id_state);

	//usb1_id_state = gpio_get_value(hdmi_det_gpio);
	printk("wgx>>>>>>  %s[%d]:zhaolong gpio166 = %d \n", __func__,__LINE__,usb1_id_state);
	if (!usb1_id_state){
		printk("===%s USB1 is plug in===\n",__func__);
		FUSB300Read(regStatus, 1, &CCXstate);	/* Read CC1 CC2 state*/
		CCXstate &= 0x30;
		//printk("===%s hdmi plug in===0x11 CCXstate=0x%x\n",__func__,CCXstate);
		if ((CCXstate == 0x10) || (CCXstate == 0x20)){
			
			//aeon_gpio_set("sil9022_hdmi_pwren1");//GPIO160
			aeon_gpio_set("sil9022_hdmi_pwren1");//GPIO126
			mdelay(400);
			//HDMI_id_state = gpio_get_value(hdmi_det_gpio);
			//printk("====%s:zhaolong debug x600 USB1=HDMI_id_state=%d===\n", __func__,HDMI_id_state);
			
			if (gpio_get_value(hdmi_det_gpio)){
				printk("===%s hdmi plug in===\n",__func__);
				if(CCXstate == 0x10){
					printk("===zhaolong====CC1=======\n",__func__);
					aeon_gpio_set("fusb301a_sw_en_low");//GPIO70 low
					aeon_gpio_set("fusb301a_sw_sel_low");//GPIO71 low
					//
					//aeon_gpio_set("sw7226_en_low");//GPIO72 low
					hdmi_plug_in_flag = 1;
					//AudDrv_GPIO_EXTAMP_Select(false, 3);
					//AudDrv_GPIO_EXTAMP2_Select(false,3);
				}else if(CCXstate == 0x20){
					printk("%s==zhaolong=====CC2=======\n",__func__);
					aeon_gpio_set("fusb301a_sw_en_low");//GPIO70 low
					aeon_gpio_set("fusb301a_sw_sel_high");//GPIO71 high
					//
					//aeon_gpio_set("sw7226_en_low");//GPIO72 low
					hdmi_plug_in_flag = 1;
					//AudDrv_GPIO_EXTAMP_Select(false, 3);
					//AudDrv_GPIO_EXTAMP2_Select(false,3);
				}else{
					printk("%s==zhaolong=====CCX detect error=======\n",__func__);
					fusb300_gpio_init();
					hdmi_plug_in_flag = 0;
					//AudDrv_GPIO_EXTAMP_Select(true, 3);
					//AudDrv_GPIO_EXTAMP2_Select(true,3);
				}
			}else{
				printk("%s=zhaolong==usb1 OTG mode===\n",__func__);	
				//right_otg_in_report_key();
				//aeon_otg_enable = 3;
				//force_to_otg(TRUE);
				is_rusb_onotg = 1;
			}
		}else{
			printk("%s==zhaolong=====CCX detect error=======\n",__func__);
			fusb300_gpio_init();
			hdmi_plug_in_flag = 0;
			//AudDrv_GPIO_EXTAMP_Select(true, 3);
			//AudDrv_GPIO_EXTAMP2_Select(true,3);
		}

	}else{
		printk("%s==zhaolong=====USB1 plug out=======\n",__func__);
		/*if(aeon_otg_enable != 2){
			aeon_otg_enable = 0;
		}*/
		if(hdmi_plug_in_flag==0){
			//right_otg_out_report_key();
		}
		fusb300_gpio_init();
		hdmi_plug_in_flag = 0;
		//AudDrv_GPIO_EXTAMP_Select(true, 3);
		//AudDrv_GPIO_EXTAMP2_Select(true,3);
	}
	printk("wgx >>>>>>>> usb1_id_state = %d,CCXstate = 0x%x , usbid_irqnum = %d, gpiopin = %d, debounce = %d",usb1_id_state,CCXstate,usbid_irqnum,gpiopin,debounce);
	if (usb1_id_state && is_rusb_onotg == 1)
	{
		is_rusb_onotg = 0;
		//force_to_otg(FALSE);
	}
	if(usb1_id_state)
		irq_set_irq_type(usbid_irqnum, IRQ_TYPE_LEVEL_LOW);
	else
		irq_set_irq_type(usbid_irqnum, IRQ_TYPE_LEVEL_HIGH);
	
	gpio_set_debounce(gpiopin, debounce);	
	
	enable_irq(usbid_irqnum);
	printk("wgx>>>>>>  %s[%d] \n",__func__,__LINE__);
	//sil9024_test();
	printk("wgx>>>>>>  %s[%d] \n",__func__,__LINE__);
	mutex_unlock(&typec_lock);
}
EXPORT_SYMBOL(hdmi_plug_in_flag);

#if 0
void test_right_usb(void){
	unsigned char CCXstate = 0;
	FUSB300Read(regType, 1, &CCXstate); /* Read CC1 CC2 state*/
	CCXstate &= 0x18;
	if(CCXstate == 0x8){
		aeon_gpio_set("aeon_irq_stm32_high");
		aeon_gpio_set("sil9022_hdmi_hplg0");//GPIO178
		mdelay(1);
		printk("===%s hdmi plug in===0x12 WAKE_STM32 YES\n",__func__);
	}
	else{
		aeon_gpio_set("sil9022_hdmi_hplg1");//GPIO178
		mdelay(10);
		aeon_gpio_set("aeon_irq_stm32_low");
		mdelay(1); 
		printk("===%s hdmi plug in===0x12 WAKE_STM32 NO\n",__func__);
	}
	printk("===%s hdmi plug in===0x12 CCXstate=0x%x\n",__func__,CCXstate);
}
EXPORT_SYMBOL(test_right_usb);
#endif

static irqreturn_t fusb300_eint_isr(int irqnum, void *data)
{
	int ret;
	struct usbtypc *typec = data;

	//if (typec->en_irq) {
		fusb_printk(K_DEBUG, "Disable IRQ\n");
		disable_irq_nosync(usbid_irqnum);
		//typec->en_irq = 0;
	//}

	ret = schedule_delayed_work_on(WORK_CPU_UNBOUND, &typec->fsm_work, 0);

	return IRQ_HANDLED;
}

int fusb300_eint_init(struct usbtypc *typec)
{
	int retval = 0;
	u32 ints[2] = { 0, 0 };
	struct device_node *node;

	node = of_find_compatible_node(NULL, NULL, "mediatek,fusb301a-pin");
	if (node) {
		of_property_read_u32_array(node, "debounce", ints, ARRAY_SIZE(ints));
		debounce = ints[1];
		gpiopin = of_get_named_gpio(node, "gpio-irq-std", 0);//ints[0];

		//gpio_set_debounce(gpiopin, debounce);
	} else {
		fusb_printk(K_INFO, "fusb300_eint_init request_irq node = NULL\n");
	}

	//typec->irqnum = irq_of_parse_and_map(node, 0);
	usbid_irqnum = irq_of_parse_and_map(node, 0);
	typec->en_irq = 1;
	
	fusb_printk(K_INFO, "fusb300_eint_init request_irq debounce=%d, gpiopin=%d\n", debounce, gpiopin);
	
	fusb_printk(K_INFO, "fusb300_eint_init request_irq irqnum=0x%d\n", usbid_irqnum);

	retval =
	    request_irq(usbid_irqnum, fusb300_eint_isr, IRQF_TRIGGER_NONE, "fusb300-eint", typec);
	if (retval != 0) {
		fusb_printk(K_ERR, "fusb300_eint_init request_irq fail, ret %d, irqnum %d!!!\n", retval,
			    usbid_irqnum);
	}
	printk("fusb300_eint_init OK!!!\n");
	return retval;
}

void fusb300_i2c_w_reg8(struct i2c_client *client, u8 addr, u8 var)
{
	char buffer[2];

	buffer[0] = addr;
	buffer[1] = var;
	i2c_master_send(client, buffer, 2);
}

u8 fusb300_i2c_r_reg(struct i2c_client *client, u8 addr)
{
	u8 var;

	i2c_master_send(client, &addr, 1);
	i2c_master_recv(client, &var, 1);
	return var;
}

static int fusb300_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct usbtypc *typec;
//	unsigned char port_type;
	unsigned char CCXstate = 0;

	fusb_printk(K_INFO, "%s 0x%x\n", __func__, client->addr);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		fusb_printk(K_ERR, "fusb300 i2c functionality check fail.\n");
		return -ENODEV;
	}

	fusb_printk(K_DEBUG, "%s %s\n", __func__, client->dev.driver->name);

	if (!g_exttypec)
		g_exttypec = kzalloc(sizeof(struct usbtypc), GFP_KERNEL);

	typec = g_exttypec;

	typec->i2c_hd = client;
	typec_client = client;

	spin_lock_init(&typec->fsm_lock);
	mutex_init(&typec_lock);

	INIT_DELAYED_WORK(&typec->fsm_work, fusb300_eint_work);

//	fusb300_init_debugfs(typec);
	
	InitializeFUSB300();
#ifdef AEON_USB_CONTROL_SUPPORT  
	usb_control_entry = proc_create(USB_CONTROL_PROC_NAME, 0777, NULL, &usb_control_proc_fops);  
if (NULL == usb_control_entry)  
{          
	printk("proc_create %s failed\n", USB_CONTROL_PROC_NAME);  
}
#endif 

//	usb_redriver_init(typec);
	fusb300_eint_init(typec);
	fusb_printk(K_INFO, "%s %x\n", __func__, fusb300_i2c_r_reg(client, 0x1));

	//FUSB300Write(0x05, 1, 0x01);
	//Registers.Reset.byte = 0x01;
	//FUSB300Write(0x05, 1, &Registers.Reset.byte);
	//mdelay(10);
	
	FUSB300Read(regStatus, 1, &CCXstate);
	fusb_printk(K_INFO, "%s:CCXstate = 0x%x\n", __func__, CCXstate);
	
	/*precheck status */
	/* StateMachineFUSB300(typec); */
	hdmi_plug();
	return 0;
}

#if 0
/* /////////////////////////////////////////////////////////////////////////////// */
int register_typec_switch_callback(struct typec_switch_data *new_driver)
{
	//fusb_printk(K_INFO, "Register driver %s %d\n", new_driver->name, new_driver->type);
#if 0
	if (new_driver->type == DEVICE_TYPE) {
		g_exttypec->device_driver = new_driver;
		g_exttypec->device_driver->on = 0;
		return 0;
	}

	if (new_driver->type == HOST_TYPE) {
		g_exttypec->host_driver = new_driver;
		g_exttypec->host_driver->on = 0;
//		if (ConnState == AttachedSource)
//			trigger_driver(g_exttypec, HOST_TYPE, ENABLE, DONT_CARE);
		return 0;
	}

	return -1;
#endif
        return 0;
}
EXPORT_SYMBOL_GPL(register_typec_switch_callback);

int unregister_typec_switch_callback(struct typec_switch_data *new_driver)
{
	fusb_printk(K_INFO, "Unregister driver %s %d\n", new_driver->name, new_driver->type);
#if 0
	if ((new_driver->type == DEVICE_TYPE) && (g_exttypec->device_driver == new_driver))
		g_exttypec->device_driver = NULL;

	if ((new_driver->type == HOST_TYPE) && (g_exttypec->host_driver == new_driver))
		g_exttypec->host_driver = NULL;
#endif
	return 0;
}
EXPORT_SYMBOL_GPL(unregister_typec_switch_callback);
#endif



#define FUSB302_NAME "FUSB301_1"

static const struct i2c_device_id usb_i2c_id[] = {
		{FUSB302_NAME, 0},
		{}
	};

#ifdef CONFIG_OF
static const struct of_device_id fusb301_of_match[] = {
		{.compatible = "mediatek,fusb301a"},
		{},
	};
#endif

struct i2c_driver usb_i2c_driver = {
	.probe = fusb300_i2c_probe,
	.driver = {
		.owner = THIS_MODULE,
		.name = FUSB302_NAME,
#ifdef CONFIG_OF
		.of_match_table = fusb301_of_match,
#endif
	},
	.id_table = usb_i2c_id,
};

static int __init fusb300_init(void)
{
	int ret = 0;
	
	fusb300_gpio_init();
	if (i2c_add_driver(&usb_i2c_driver) != 0) {
		fusb_printk(K_ERR, "fusb300_init initialization failed!!\n");
		ret = -1;
	} else {
		fusb_printk(K_DEBUG, "fusb300_init initialization succeed!!\n");
	}
	return ret;
}

static void __exit fusb300_exit(void)
{

}
fs_initcall(fusb300_init);
/* module_exit(fusb300_exit); */

#endif				/*USB_TYPE_C */
