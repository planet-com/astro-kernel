/**
 * AW6302 Fingerprint sensor device driver
 *
 * This driver will control the platform resources that the AW6302 fingerprint
 * sensor needs to operate. The major things are probing the sensor to check
 * that it is actually connected and let the Kernel know this and with that also
 * enabling and disabling of regulators, enabling and disabling of platform
 * clocks, controlling GPIOs such as SPI chip select, sensor reset line, sensor
 * IRQ line, MISO and MOSI lines.
 *
 * The driver will expose most of its available functionality in sysfs which
 * enables dynamic control of these features from eg. a user space process.
 *
 
 
 
 * The sensor's IRQ events will be pushed to Kernel's event handling system and
 * are exposed in the drivers event node. This makes it possible for a user
 * space process to poll the input node and receive IRQ events easily. Usually
 * this node is available under /dev/input/eventX where 'X' is a number given by
 * the event system. A user space process will need to traverse all the event
 * nodes and ask for its parent's name (through EVIOCGNAME) which should match
 * the value in device tree named input-device-name.
 *
 * This driver will NOT send any SPI commands to the sensor it only controls the
 * electrical parts.
 *
 *
 * Copyright (C) 2016 AW6302 Corporation. <http://www.AW6302.com>
 * Copyright (C) 2016 XXX <mailto:xxx@AW6302.com>
 *
 * This program is free software; you can redistribute it and/or modify it 
 * under the terms of the GNU General Public License as published by the Free 
 * Software Foundation; either version 2 of the License, or (at your option) 
 * any later version.
 *
 * This program is distributed in the hope that it will be useful, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General 
 * Public License for more details.
 **/

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/of.h>
#include <linux/err.h>
#include <linux/of_platform.h>
#include <linux/pinctrl/consumer.h>
#include <linux/gpio.h>
#include <linux/of_irq.h>
#include <linux/input.h>
#include <linux/proc_fs.h>
//#include <soc/qcom/scm.h>
#if defined(CONFIG_FB) //system-defined Macro!!
#include <linux/notifier.h>
#include <linux/fb.h>
#endif
extern int aeon_gpio_set(const char *name);
#if defined(CONFIG_HAS_EARLYSUSPEND)  //system-defined Macro!!
#include <linux/earlysuspend.h>
/* Early-suspend level */
#define FT_SUSPEND_LEVEL 1
#endif

#include "cf_ctl.h"

//SPI CLK
#if defined(MTK_PLATFORM)
	#if (defined(TEE) && defined(MTK6739))//kernel version >kernel-4.4 
		#include <mtk_spi.h>
		#include "mtk_spi_hal.h"
extern void mt_spi_enable_master_clk(struct spi_device *spi);
extern void mt_spi_disable_master_clk(struct spi_device *spi);
	#else	
		#include "mt_spi.h"
		#include "mt_spi_hal.h"
		extern void mt_spi_enable_master_clk(struct spi_device *spi);
		extern void mt_spi_disable_master_clk(struct spi_device *spi);
		#if defined(MTK6580)
			#include "mach/mt_clkmgr.h"
		#endif
	#endif

#endif

//spi api
#if defined(TEE)
	#if defined(IS_ISEE)
		#include <fp_vendor.h>//doujia
		#include <teei_fp.h>
		#include <tee_client_api.h>
	#elif defined(IS_TRUSTKERNEL)
		#include <tee_fp.h>
	#endif
#endif


#define MODULE_NAME "aw_ctl"
#ifndef CONFIG_OF
# error "error: this driver 'MODULE_NAME' only support dts."
#endif


#define ANDROID_WAKELOCK 1
#if ANDROID_WAKELOCK
	#if defined(KERNEL49)
		#include <linux/pm_wakeup.h>
	#else
		#include <linux/wakelock.h>
	#endif

#endif

#define CF_RESET_LOW_US      1000
#define CF_RESET_HIGH1_US    100
#define PWR_ON_STEP_SLEEP    100
#define PWR_ON_STEP_RANGE1   100
#define PWR_ON_STEP_RANGE2   900
#define CF_TTW_HOLD_TIME     1000

#if defined(MTK_PLATFORM)
typedef enum {
	CF_PIN_STATE_RST_HIGH,
	CF_PIN_STATE_RST_LOW,
	CF_PIN_STATE_INT,
	CF_PIN_STATE_CLK,
	CF_PIN_STATE_CS,
	CF_PIN_STATE_MI,
	CF_PIN_STATE_MO,
	#if defined(FP_POWER)
	CF_PIN_STATE_POWER_ON,
	CF_PIN_STATE_POWER_OFF,
	#endif
	/* Array size */
	CF_PIN_STATE_MAX
} aw_pin_state_t;

static const char * const pctl_names[] = {
	"cs_finger_reset_en1",
	"cs_finger_reset_en0",
	"cs_finger_int_as_int",
	"cs_finger_spi0_clk_as_spi0_clk",
	"cs_finger_spi0_cs_as_spi0_cs",
	"cs_finger_spi0_mi_as_spi0_mi",
	"cs_finger_spi0_mo_as_spi0_mo"
	#if defined(FP_POWER)
	"cs_finger_power_on",
	"cs_finger_power_off",
	#endif
	
};
#endif
#if defined(IS_ISEE)
struct TEEC_UUID vendor_uuid = {0x8aaaf200, 0x2450, 0x11e4,
	{ 0xab, 0xe2, 0x00, 0x02, 0xa5, 0xd5, 0xc5, 0x1a }};
#endif

struct vreg_config {
	char *name;
	unsigned long vmin;
	unsigned long vmax;
	int ua_load;
};

static const struct vreg_config const vreg_conf[] = {
	{ "VDD", 2800000UL, 2800000UL, 6000, },
};

#if defined(REE)
const static unsigned int bufsiz = 10240*10;
#endif

/**
 * Define the driver version string.
 * There is NO need to modify 'rXXXX_yyyymmdd', it should be updated automatically
 * by the building script (see the 'Driver-revision' section in 'build.sh').
 */
#define CF_DRV_VERSION "Driver_v3.1.1-20181128"

struct aw_device {
	struct device *dev;
	struct spi_device *spi;
	struct cdev     cdev;
	struct class*    class;
	struct device*   device;
	dev_t             devno;
	struct pinctrl *fingerprint_pinctrl;
	struct pinctrl_state *pinctrl_state[CF_PIN_STATE_MAX];
	struct clk *iface_clk;
	struct clk *core_clk;
	struct regulator *vreg[ARRAY_SIZE(vreg_conf)];
	struct input_dev *input;
	struct fasync_struct *async;
	//struct work_struct work_queue;
	struct platform_device* pf_dev;

#if defined(MTK_PLATFORM) 
	#if !defined(KERNEL49)
		struct mtk_chip_config spi_mcc;
	#else
		struct mt_chip_conf spi_mcc;
		struct mt_spi_t *mt_spi;
	#endif
	
#endif

#if defined(CONFIG_FB)
	struct notifier_block fb_notify;
#endif

#if defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_suspend;
#endif

	#if defined(KERNEL49)
		struct wakeup_source ttw_wl;
	#else
		struct wake_lock ttw_wl;
	#endif
	int irq;
	int irq_gpio;
	int rst_gpio;
	int pwr_gpio;
	int qup_id;
	struct mutex lock;
	spinlock_t spin_lock;
	bool prepared;
	atomic_t wakeup_enabled;
	bool irq_enabled;
	bool clocks_enabled;
	bool clocks_suspended;
    bool isPowerOn;
	u8 *buf;
	bool blankChanged;
	int display_blank_flag;
};

/**************************debug******************************/
#define ERR_LOG  (0)
#define INFO_LOG (1)
#define DEBUG_LOG (2)

/* debug log setting */
u8 aw_debug_level = DEBUG_LOG;

#define aw_debug(level, fmt, args...) do { \
	if (aw_debug_level >= level) {\
		printk("[aw6302]%s line:%d  "fmt, __func__, __LINE__, ##args);\
	} \
} while (0)

#define FUNC_ENTRY()  aw_debug(DEBUG_LOG, "entry\n")
#define FUNC_EXIT()  aw_debug(DEBUG_LOG, "exit\n")

/*************************************************************/
extern int aw_sfr_read(struct spi_device *spi, unsigned short addr, unsigned char *recv_buf, unsigned short buflen);

#if 1
static int vreg_setup(struct aw_device *aw_dev, const char *name, bool enable)
{
	size_t i;
	int rc;
	struct regulator *vreg;
	struct device *dev = aw_dev->dev;

	for (i = 0; i < ARRAY_SIZE(aw_dev->vreg); i++) 
	{
		const char *n = vreg_conf[i].name;
		if (!strncmp(n, name, strlen(n)))
			goto found;
	}
	aw_debug(ERR_LOG, "Regulator %s not found\n", name);
	return -EINVAL;

found:
	vreg = aw_dev->vreg[i];
	if (enable) 
	{
		if (!vreg) 
		{
			vreg = regulator_get(dev, name);
			if (IS_ERR(vreg)) 
			{
				aw_debug(ERR_LOG, "Unable to get %s\n", name);
				return PTR_ERR(vreg);
			}
		}
		if (regulator_count_voltages(vreg) > 0) 
		{
			rc = regulator_set_voltage(vreg, vreg_conf[i].vmin,
					vreg_conf[i].vmax);
			if (rc)
				aw_debug(ERR_LOG, "Unable to set voltage on %s, %d\n", name, rc);
		}
		#if 0
		rc = regulator_set_optimum_mode(vreg, vreg_conf[i].ua_load);
		if (rc < 0)
			aw_debug(ERR_LOG, "Unable to set current on %s, %d\n", name, rc);
		#endif
		rc = regulator_enable(vreg);
		if (rc) 
		{
			aw_debug(ERR_LOG, "error enabling %s: %d\n", name, rc);
			regulator_put(vreg);
			vreg = NULL;
		}
		aw_dev->vreg[i] = vreg;
	} 
	else 
	{
		if (vreg) 
		{
			if (regulator_is_enabled(vreg)) 
			{
				regulator_disable(vreg);
				aw_debug(ERR_LOG, "disabled %s\n", name);
			}
			regulator_put(vreg);
			aw_dev->vreg[i] = NULL;
		}
		rc = 0;
	}
	return rc;
}
#endif

#if 0
static DEVICE_ATTR(irq, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH);

static struct attribute *attributes[] = {
	&dev_attr_irq.attr,
	NULL
};

static const struct attribute_group attribute_group = {
	.attrs = attributes,
};
#endif

#if (defined(IS_ISEE) || defined(IS_TRUSTKERNEL) || defined(IS_RSEE) || defined(IS_TRUSTONIC))
static void aw_spi_disable_clk(struct aw_device *aw_dev)
{
#if defined(MTK6580)
	#if 0
		aw_dev->mt_spi = spi_master_get_devdata(aw_dev->spi->master);
		if (aw_dev->mt_spi)
			mt_spi_disable_clk(aw_dev->mt_spi);
	#else
		disable_clock(MT_CG_SPI_SW_CG, "spi");
		clkmux_sel(MT_CLKMUX_SPI_GFMUX_SEL, MT_CG_SYS_26M, "spi");
	#endif
#else
	mt_spi_disable_master_clk(aw_dev->spi);
#endif
}

static void aw_spi_enable_clk(struct aw_device *aw_dev)
{
#if defined(MTK6580)
	#if 0
		aw_dev->mt_spi = spi_master_get_devdata(aw_dev->spi->master);
		if (aw_dev->mt_spi)
			mt_spi_enable_clk(aw_dev->mt_spi);	
	#else
		clkmux_sel(MT_CLKMUX_SPI_GFMUX_SEL, MT_CG_UPLL_D12, "spi");
		enable_clock(MT_CG_SPI_SW_CG, "spi");	
	#endif 
#else
	aw_debug(ERR_LOG, "aw_spi_enable_clk \n");
	mt_spi_enable_master_clk(aw_dev->spi);
	
#endif
}

#endif




static void aw_fb_notify(struct aw_device *aw_dev)
{
	FUNC_ENTRY();

	/* Make sure 'wakeup_enabled' is updated before using it
	 ** since this is interrupt context (other thread...) */
	smp_rmb();

	if (atomic_read(&aw_dev->wakeup_enabled)) {
		#if defined(KERNEL49)
			__pm_wakeup_event(&aw_dev->ttw_wl, msecs_to_jiffies(CF_TTW_HOLD_TIME));
		#else
			wake_lock_timeout(&aw_dev->ttw_wl, msecs_to_jiffies(CF_TTW_HOLD_TIME));
		#endif
	}
	
	//schedule_work(&aw_dev->work_queue);
}

static int aw_open(struct inode* inode, struct file* file)
{
	struct aw_device *aw_dev;
	FUNC_ENTRY();
	aw_dev = container_of(inode->i_cdev, struct aw_device, cdev);
	file->private_data = aw_dev;
	#if defined(REE)
	if (NULL == aw_dev->buf){
		aw_dev->buf = kmalloc(bufsiz, GFP_KERNEL);
		if (NULL == aw_dev->buf) {
			aw_debug(ERR_LOG,"kmalloc ENOMEM\n");
			return -ENOMEM;
		}
	}
	#endif
	return 0;	
}


static int aw_release(struct inode* inode, struct file* file)
{
	#if defined(REE)
	struct aw_device *aw_dev;
	FUNC_ENTRY();
	aw_dev = container_of(inode->i_cdev, struct aw_device, cdev);
	kfree(aw_dev->buf);
	aw_dev->buf = NULL;
	#endif
	return 0;
}

static ssize_t aw_write(struct file *file, const char __user *buf, size_t count, loff_t *offset)
{

	
	struct aw_device *aw_dev = file->private_data;
	
	#if defined(REE)
	//FUNC_ENTRY();
	u8 rxbuf[2] = {0};
	aw_sfr_read(aw_dev->spi, 0x3e, rxbuf, 2);
	aw_debug(INFO_LOG, "rxbuf[0] = 0x%x, rxbuf[1] = 0x%x\n",rxbuf[0], rxbuf[1]);
	#endif
	//aw_debug(INFO_LOG, "irq_gpio = %d, goio_to_irq = %d, irq = %d\n",aw_dev->irq_gpio, gpio_to_irq(aw_dev->irq_gpio), aw_dev->irq);
	//aw_debug(INFO_LOG, "rst_gpio_value = %d, irq_gpio_value = %d\n", gpio_get_value(aw_dev->rst_gpio), gpio_get_value(aw_dev->irq_gpio));
	//FUNC_EXIT();
	return count;
}

static long aw_ioctl(struct file* file, unsigned int cmd, unsigned long arg)
{
	struct aw_device *aw_dev = NULL;
	int err = 0;
	aw_dev = file->private_data;
	FUNC_ENTRY();
	mutex_unlock(&aw_dev->lock);
	return err;	
}

static int aw_fasync(int fd, struct file *fp, int mode)
{
	struct aw_device *aw_dev;
	FUNC_ENTRY();
	aw_dev = fp->private_data;
	return fasync_helper(fd, fp, mode, &aw_dev->async);
}

static const struct file_operations aw_fops =
{
	.owner			= THIS_MODULE,
	.open			= aw_open,
	.release		= aw_release,
	.unlocked_ioctl	= aw_ioctl,
	.fasync         = aw_fasync,
	.write          = aw_write,
};

#if defined(MTK_PLATFORM)
	#if (defined(REE) || defined(IS_TRUSTKERNEL))
		//#include <linux/platform_data/spi-mt65xx.h>
		#if !defined(KERNEL49)
		static struct mtk_chip_config aw_spi_conf =
		{
			.rx_mlsb = 1,
			.tx_mlsb = 1,
			.cs_pol = 0,
			.sample_sel = 0,
		};
		#else
		static struct mt_chip_conf aw_spi_conf =
		{
			.setuptime = 7,//20,
			.holdtime = 7,//20,
			.high_time = 50,//50,
			.low_time = 50,//50,
			.cs_idletime = 3,// 5,
			.rx_mlsb = 1,
			.tx_mlsb = 1,
			.tx_endian = 0,
			.rx_endian = 0,
			.cpol = 0,
			.cpha = 0,
			//.com_mod = FIFO_TRANSFER,
			.com_mod = DMA_TRANSFER,
			.pause = 0,//1,
			.finish_intr = 1,
			.deassert = 0,
			.ulthigh = 0,
			.tckdly = 0,
		};
		#endif
	#endif
	#if 0//(!defined(MTK6739) || !defined(KERNEL49))
	static struct spi_board_info spi_fp_board_info[] __initdata =
	{
		[0] = {
			.modalias = "aw6302",
			.bus_num = 0,
			.chip_select = 0,
			.mode = SPI_MODE_0,
			.controller_data = &aw_spi_conf, //&spi_conf
		},
	};
	
	#endif
#endif

#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block* self, unsigned long event, void* data)
{
	struct fb_event *evdata = data;
	int* blank;
	struct aw_device *aw_dev = container_of(self, struct aw_device, fb_notify);
	if (evdata && evdata->data && aw_dev) 
	{
		if (event == FB_EVENT_BLANK) 
		{
			blank = evdata->data;
			if (*blank == FB_BLANK_UNBLANK) 
			{
				//TODO
				aw_debug(INFO_LOG, "LCD on\n");
				aw_dev->blankChanged = true;
				aw_dev->display_blank_flag = 0;
				aw_fb_notify(aw_dev);
			}
			else if (*blank == FB_BLANK_POWERDOWN) 
			{
				//TODO
				aw_debug(INFO_LOG, "LCD off\n");
				aw_dev->blankChanged = true;
				aw_dev->display_blank_flag = 1;
				aw_fb_notify(aw_dev);
			}
		}
	}

	return 0;
}
#endif

#if defined(CONFIG_HAS_EARLYSUSPEND)

static void aw_early_suspend(struct early_suspend *handler)
{
	struct aw_device *aw_dev = container_of(handler, struct aw_device, early_suspend);

	//TODO
	aw_debug(INFO_LOG, "LCD off-3");
}

static void aw_late_resume(struct early_suspend *handler)
{
	struct aw_device *aw_dev = container_of(handler, struct aw_device, early_suspend);

	//TODO
	aw_debug(INFO_LOG, "LCD on-3");
}
#endif




/* qzs add for FP_Vendor_ID*/
#define CS_CHIP_ID_PROC_FILE   "AEON_AW6302"
//static ssize_t cs_chip_id_read_proc(struct file *, char __user *, size_t, loff_t *);
static struct proc_dir_entry *cs_chip_id_proc = NULL;

/* qzs  end*/
int aw_sim_flag=0;
struct spi_device *aw_spi =NULL;
/* qzs add for chip id*/
void test_for_aw6302(int n)
{
	unsigned char send_buf[1];
	
	
	if(n==0){
		printk("qzs>>> 0000000 %s()[%d]card1\n",__func__,__LINE__);	
		//check ID fail restrn -1;
		//card1	
		msleep(10);
		aeon_gpio_set("sim_rst_high");
		msleep(10);
		aeon_gpio_set("sim_rst_low");
		msleep(10);
		aeon_gpio_set("sim_rst_high");
		msleep(10);
		send_buf[0]=0x48;
		aw_sfr_write(aw_spi,0,send_buf,1);
		send_buf[0]=0x60;
		aw_sfr_write(aw_spi,0,send_buf,1);
		msleep(1);		
		send_buf[0]=0x64;
		aw_sfr_write(aw_spi,0,send_buf,1);
		send_buf[0]=0;//0x1;
		aw_sfr_write(aw_spi,0,send_buf,1);
		send_buf[0]=0x29;
		aw_sfr_write(aw_spi,0,send_buf,1);
		send_buf[0]=0x49;
		aw_sfr_write(aw_spi,0,send_buf,1);
		//D0+D2
		aw_sim_flag = n;
	}else if(n==1){
		printk("qzs>>> 0000000 %s()[%d]card2\n",__func__,__LINE__);	
		//card2
		msleep(10);
		aeon_gpio_set("sim_rst_high");
		msleep(10);
		aeon_gpio_set("sim_rst_low");
		msleep(10);
		aeon_gpio_set("sim_rst_high");
		msleep(10);
		send_buf[0]=0x44;
		aw_sfr_write(aw_spi,0,send_buf,1);			
		send_buf[0]=0x60;
		aw_sfr_write(aw_spi,0,send_buf,1);
		msleep(1);			
		send_buf[0]=0x68;
		aw_sfr_write(aw_spi,0,send_buf,1);	
		send_buf[0]=0;//0x2;
		aw_sfr_write(aw_spi,0,send_buf,1);
		send_buf[0]=0x26;
		aw_sfr_write(aw_spi,0,send_buf,1);
		send_buf[0]=0x46;
		aw_sfr_write(aw_spi,0,send_buf,1);
		//D1+D3	
		aw_sim_flag = n;
	}else if(n==2){
		//send_buf[0]=0x44;
		//aw_sfr_write(aw_spi,0,send_buf,1);			
	}else if(n==3){
		printk("qzs>>> 0000000 %s()[%d]card power off\n",__func__,__LINE__);	
		send_buf[0]=0x60;
		aw_sfr_write(aw_spi,0,send_buf,1);
		aw_sim_flag = n;		
	}
}
EXPORT_SYMBOL(test_for_aw6302);

static ssize_t cs_chip_id_read_proc(struct file *filp, char __user *buffer, size_t size, loff_t *ppos)
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

	if(aw_sim_flag ==0)
    ptr += sprintf(ptr, "0\n");
	else if(aw_sim_flag==1)
	ptr += sprintf(ptr, "1\n");
	else if(aw_sim_flag==3)
	ptr += sprintf(ptr, "3\n");

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

static ssize_t cs_chip_id_write_proc(struct file *filp, const char __user *buff, size_t count, loff_t *ppos)
{
#if 1
    char acBuf[3];
    char *str_buf = acBuf;
         
    if (count >= sizeof(acBuf)){
		printk("%s() len outsize!! %ld\n",__func__, count);
        return -EFAULT;  
    }
         
    if(copy_from_user(str_buf, buff, count)){ 
		pr_warn("copy_from_user---error\n");
		return -EFAULT; 
	}
#else	
    //char str_buf[1] = {0};
	u8 *str_buf = NULL;
	if (copy_from_user(str_buf, buff, count)){
        pr_warn("copy_from_user---error\n");
        return -EFAULT;
    } 
#endif

	msleep(10);
		if (str_buf[0]== '0') {
			//beijia_stat = 0;
			test_for_aw6302(0);
			printk("qzs>>> 0000000 %s()[%d]\n",__func__,__LINE__);
		}else if(str_buf[0]== '1'){
			//beijia_stat = 1;
			test_for_aw6302(1);
			printk("qzs>>> 1111111 %s()[%d]\n",__func__,__LINE__);
		}else if(str_buf[0]== '3'){
			test_for_aw6302(3);
			printk("qzs>>> 3333333 %s()[%d]\n",__func__,__LINE__);			
		}

    return count;
}
static const struct file_operations cs_chip_id_proc_ops = {
	.owner = THIS_MODULE,
	.read = cs_chip_id_read_proc,
	.write = cs_chip_id_write_proc,
};

/* qzs end*/
void aw_dev_data_init(struct spi_device *spi)
{
	unsigned char send_buf[1];	
	aw_spi =spi ;
	printk("qzs>>> 0000000 %s()[%d]\n",__func__,__LINE__);
	msleep(10);
	aeon_gpio_set("sim_rst_high");
	msleep(10);
	aeon_gpio_set("sim_rst_low");
	msleep(10);
	aeon_gpio_set("sim_rst_high");
	msleep(10);
#if 1
	send_buf[0]=0x64;
	aw_sfr_write(aw_spi,0,send_buf,1);	
	send_buf[0]=0x0;
	aw_sfr_write(aw_spi,0,send_buf,1);
	send_buf[0]=0x29;
	aw_sfr_write(aw_spi,0,send_buf,1);
	send_buf[0]=0x49;
	aw_sfr_write(aw_spi,0,send_buf,1);
#else
	send_buf[0]=0x68;
	aw_sfr_write(aw_spi,0,send_buf,1);	
	send_buf[0]=0x0;
	aw_sfr_write(aw_spi,0,send_buf,1);
	send_buf[0]=0x22;
	aw_sfr_write(aw_spi,0,send_buf,1);
	send_buf[0]=0x42;
	aw_sfr_write(aw_spi,0,send_buf,1);
#endif
}
static int aw_probe(struct spi_device *spi)
{
	int rc = 0;

	#if (defined(REE) || defined(TEE) || defined(IS_TRUSTKERNEL) || defined(IS_ISEE))
	//int i = 0;
	#endif
	struct device *dev;
	//struct device_node *np;
	struct aw_device *aw_dev;
	//struct platform_device *pdev;
	//u32 val;
	aw_debug(ERR_LOG, "aw6302 aw_probe start!!!\n");

	dev = &spi->dev;
	aw_dev = devm_kzalloc(dev, sizeof(*aw_dev), GFP_KERNEL);
	if (!aw_dev)
	{
		aw_debug(ERR_LOG,	"failed to allocate memory for struct aw_device");
		rc = -ENOMEM;
		goto exit;
	}

	aw_dev->dev = dev;
	dev_set_drvdata(dev, aw_dev);
	aw_dev->spi = spi;
	
	//setup SPI
	aw_dev->spi->mode            = SPI_MODE_0;
	aw_dev->spi->bits_per_word   = 8;
	aw_dev->spi->max_speed_hz    = 2 * 1000 * 1000;
#if defined(MTK_PLATFORM)
	#if !defined(MTK6739)
	aw_dev->spi->controller_data = (void *)&aw_spi_conf;
	#endif
#endif	
	spi_setup(aw_dev->spi);
	aw_dev_data_init(aw_dev->spi);

   /* qzs add for chip id*/
   cs_chip_id_proc = proc_create(CS_CHIP_ID_PROC_FILE, 0666, NULL, &cs_chip_id_proc_ops);
   if (cs_chip_id_proc == NULL) {
       printk("create AW6302 proc entry %s error.", CS_CHIP_ID_PROC_FILE);
   }
   else{
       printk("create AW6302 proc entry %s success.", CS_CHIP_ID_PROC_FILE);
   }
    /* qzs end for chip id*/
	aw_dev->class = class_create(THIS_MODULE, FP_CLASS_NAME);
	rc = alloc_chrdev_region(&aw_dev->devno, 0, 1, FP_DEV_NAME);
	if (rc) 
	{
		aw_debug(ERR_LOG, "alloc_chrdev_region failed, error = %d\n", rc);
		goto exit;
	}
	aw_dev->device = device_create(aw_dev->class, NULL, aw_dev->devno, NULL, "%s", FP_DEV_NAME);
	cdev_init(&aw_dev->cdev, &aw_fops);
	aw_dev->cdev.owner = THIS_MODULE;

	rc = cdev_add(&aw_dev->cdev, aw_dev->devno, 1);
	if (rc) 
	{
		aw_debug(ERR_LOG, "cdev_add failed, error = %d\n", rc);
		goto exit;
	}	

	aw_dev->clocks_enabled = false;
	aw_dev->clocks_suspended = false;

	mutex_init(&aw_dev->lock);
	spin_lock_init(&aw_dev->spin_lock);
	#if defined(KERNEL49)
		wakeup_source_init(&aw_dev->ttw_wl, "aw_ttw_wl");
	#else
		wake_lock_init(&aw_dev->ttw_wl, WAKE_LOCK_SUSPEND, "aw6302_ttw_wl");
	#endif
	

	atomic_set(&aw_dev->wakeup_enabled, 1);

    aw_dev->pf_dev = platform_device_alloc(FP_DEV_NAME, -1);
    if (!aw_dev->pf_dev)
    {
        rc = -ENOMEM;
        aw_debug(ERR_LOG,"platform_device_alloc failed\n");
        goto exit;
    }
    rc = platform_device_add(aw_dev->pf_dev);

    if (rc)
    {
        aw_debug(ERR_LOG,"platform_device_add failed\n");
        platform_device_del(aw_dev->pf_dev);
        goto exit;
    }
    else
    {
        dev_set_drvdata(&aw_dev->pf_dev->dev, aw_dev);
		#if 0
        rc = sysfs_create_group(&aw_dev->pf_dev->dev.kobj, &attribute_group);

        if (rc)
        {
            aw_debug(ERR_LOG,"sysfs_create_group failed\n");
            goto exit;
        }
		#endif
    }
    

#if defined(CONFIG_FB)
	aw_dev->fb_notify.notifier_call = fb_notifier_callback;
	fb_register_client(&aw_dev->fb_notify);
#endif

#if defined(CONFIG_HAS_EARLYSUSPEND)
	aw_dev->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN +
		FT_SUSPEND_LEVEL;
	aw_dev->early_suspend.suspend = aw_early_suspend;
	aw_dev->early_suspend.resume = aw_late_resume;
	register_early_suspend(&aw_dev->early_suspend);
#endif	

	test_for_aw6302(0); //add for sim2 init 2021.4.28
	
	FUNC_EXIT();
exit:
	return rc;
}

static int aw_remove(struct spi_device *spi)
{
	struct aw_device *aw_dev = dev_get_drvdata(&spi->dev);

#if defined(CONFIG_HAS_EARLYSUSPENDCONFIG_HAS_EARLYSUSPEND)
	unregister_early_suspend(&aw_dev->early_suspend);
#endif

#if defined(CONFIG_FB)
	if (aw_dev->fb_notify.notifier_call) 
	{
		aw_dev->fb_notify.notifier_call = NULL;
		fb_unregister_client(&aw_dev->fb_notify);
	}
#endif

	//sysfs_remove_group(&aw_dev->pf_dev->dev.kobj, &attribute_group);
	class_destroy(aw_dev->class);
	#if defined(KERNEL49)
		__pm_relax(&aw_dev->ttw_wl);
	#else
		mutex_destroy(&aw_dev->lock);
	#endif
	
	(void)vreg_setup(aw_dev, "VDD", false);
	#if defined(FP_POWER)
		aw_power_off(aw_dev);
	#endif
	FUNC_EXIT();
	return 0;
}

static int aw_suspend(struct device *dev)
{
	return 0;

}

static int aw_resume(struct device *dev)
{

	return 0;
}

static const struct dev_pm_ops aw_pm_ops = {
	.suspend = aw_suspend,
	.resume = aw_resume,
};


static struct of_device_id aw_of_match[] = {
	{ .compatible = "mediatek,aw6302", },
	{}
};
MODULE_DEVICE_TABLE(of, aw_of_match);

static struct spi_driver aw_driver = {
	.driver = {
		.name	= "aw6302",
		.owner	= THIS_MODULE,
		.of_match_table = aw_of_match,
		.pm = &aw_pm_ops,
	},
	.probe		= aw_probe,
	.remove		= aw_remove,
};


static int __init aw_driver_init(void)
{
	int rc;
	FUNC_ENTRY();

#if defined(MTK_PLATFORM)
	#if 0//(!defined(MTK6739) ||  !defined(KERNEL49))
	printk("AW6302 cs_finger spi_register_board_info start!!!\n");
	spi_register_board_info(spi_fp_board_info, ARRAY_SIZE(spi_fp_board_info));
	#endif
#endif

	rc = spi_register_driver(&aw_driver);
	if (!rc)
		aw_debug(ERR_LOG, "spi_register_driver(..) pass\n");
	else
		aw_debug(ERR_LOG, "spi_register_driver(..) fail, error = %d\n", rc);

	return rc;
}

static void __exit aw_driver_exit(void)
{
	FUNC_ENTRY();
	spi_unregister_driver(&aw_driver);
}

module_init(aw_driver_init);
module_exit(aw_driver_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("QZS");
MODULE_DESCRIPTION("AW6302 Fingerprint sensor device driver.");
