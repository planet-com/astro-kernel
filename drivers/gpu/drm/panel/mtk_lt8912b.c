#include <linux/string.h>
//#include "mt-plat/upmu_common.h"
#include <linux/kernel.h>
#include <linux/module.h>  
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
//#include <linux/jiffies.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
//#include <mt-plat/mtk_pwm.h>
#include <linux/device.h>
#include <linux/sysfs.h>

#if 0
#define FRAME_WIDTH  										(854)
#define FRAME_HEIGHT 										(480)

#define HSYNC_PULSE_WIDTH           96 
#define HSYNC_BACK_PORCH            48
#define HSYNC_FRONT_PORCH           16
#define VSYNC_PULSE_WIDTH           2
#define VSYNC_BACK_PORCH            33
#define VSYNC_FRONT_PORCH           10

#define MIPI_CLK 99

#else
#define FRAME_WIDTH  										(1280)
#define FRAME_HEIGHT 										(720)

#define HSYNC_PULSE_WIDTH           40 
#define HSYNC_BACK_PORCH            220
#define HSYNC_FRONT_PORCH           110
#define VSYNC_PULSE_WIDTH           5
#define VSYNC_BACK_PORCH            20
#define VSYNC_FRONT_PORCH           5

#define MIPI_CLK 223

#endif

enum
{
	H_act = 0,
	V_act,
	H_tol,
	V_tol,
	H_bp,
	H_sync,
	V_sync,
	V_bp
};

#define MIPI_Lane 4     // 4 Lane MIPI input

static int MIPI_Timing[] =
//  H_act	V_act	 H_total V_total H_BP	 H_sync  V_sync		V_BP
   // {1280,  	800,	 1450,	 832,	 80,	 10,	 2,  		12};// 1080P  Vesa Timing
	
	//{1280,  	720,	 1650,	 750,	 220,	 40,	 5,  		20};// 1080P  Vesa Timing
	{FRAME_WIDTH,  	FRAME_HEIGHT,	 (FRAME_WIDTH+HSYNC_PULSE_WIDTH+HSYNC_BACK_PORCH+HSYNC_FRONT_PORCH),	 (FRAME_HEIGHT+VSYNC_PULSE_WIDTH+VSYNC_BACK_PORCH+VSYNC_FRONT_PORCH),	 HSYNC_BACK_PORCH,	 HSYNC_PULSE_WIDTH,	 VSYNC_PULSE_WIDTH, VSYNC_BACK_PORCH};// 1080P  Vesa Timing
	
	//  H_act	V_act	 H_total V_total H_BP	 H_sync  V_sync		V_BP                     
 //   {1920,  	1080,	 2200,	 1125,	 148,	 44,	 5,  		36};// 1080P  Vesa Timing


//  {1366,  768,	 1500,	 800,	 64, 56, 3,  28};// 1366x768 VESA Timing
//  { 1024, 768, 1344, 806, 160, 136, 6, 29 };  // 1024x768 Timing

extern int aeon_gpio_set(const char *name);

#define I2C_ID_NAME "lt8912"
static struct i2c_client *lt8912_i2c_client = NULL;

static const struct of_device_id lt8912_of_match[] = {
		{.compatible = "mediatek,lt8912"},
		{},
};

/***************************************************************************** 
 * Function Prototype
 *****************************************************************************/ 
 
static int lt8912_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int lt8912_remove(struct i2c_client *client);
/***************************************************************************** 
 * Data Structure
 *****************************************************************************/

struct lt8912_dev	{	
    struct i2c_client	*client;
};

static const struct i2c_device_id lt8912_id[] = {
     {I2C_ID_NAME, 0},
};

static struct i2c_driver lt8912_iic_driver = {
    .id_table	= lt8912_id,
    .probe		= lt8912_probe,
    .remove		= lt8912_remove,
    .driver		= {
        .owner	= THIS_MODULE,
        .name	= "lt8912",
		.of_match_table = lt8912_of_match,
    },

};

static int match_id(const struct i2c_device_id *id, const struct i2c_client *client)
{
	if (strcmp(client->name, id->name) == 0)
		return 1;

	return 0;
}


static int lt8912_probe(struct i2c_client *client, const struct i2c_device_id *id)
{  
	if (match_id(&lt8912_id[0], client)) {	
		lt8912_i2c_client = client;
		printk("attached lt8912 : %s, lt8912_i2c_client=%p\n "
			"into i2c adapter successfully\n", id->name, lt8912_i2c_client);
	} else {		
		printk("lt8912 invalid i2c adapter: can not found dev_id matched\n");
		return -EIO;
	}
	
    return 0;      
}

static int lt8912_remove(struct i2c_client *client)
{  	
    printk( "lt8912_remove\n");
    lt8912_i2c_client = NULL;
    i2c_unregister_device(client);
    return 0;
}

#define LT8912B_SLAVE_ADDR_WRITE  0x90
#define LT8912B_SLAVE_ADDR92_WRITE  0x92
#define LT8912B_SLAVE_ADDR94_WRITE  0x94

#if 1
static int hdmi8912b_write_byte(unsigned char i2c_addr, unsigned char addr, unsigned char value)
{
#if 0
    int ret = 0;
    struct i2c_client *client = lt8912_i2c_client;
    char write_data[2]={0};	

    client->addr = (i2c_addr >> 1);
    write_data[0] = addr;
    write_data[1] = value;

    ret=i2c_master_send(client, write_data, 2);
    if(ret<0)
        printk("lt8912 write data fail !!\n");	
	
	printk("hdmi8912b_write_byte-------OK-------\n");
    return ret ;
#else
    char write_data[2]={0};	
	struct i2c_msg msgs[] = {
		{
			.addr = (i2c_addr >> 1),
			.flags = 0,
			.len = 2,
			.buf = write_data,
		},
	};
    write_data[0] = addr;
    write_data[1] = value;

	if (i2c_transfer(lt8912_i2c_client->adapter, msgs, 1) < 0) {
		printk("hdmi8912b_write_byte: transfer error\n");
		return -EIO;
	} else {
        printk("---hdmi8912b_write_byte I2C OK, addr = 0x%x, value = 0x%x---\n", addr, value);
		return 0;
    }
#endif

}
#endif

#if 1
int hdmi8912b_read_bytes(u8 *a_pSendData , u16 a_sizeSendData, u8 *a_pRecvData, u16 a_sizeRecvData, u16 i2cId)
{

	struct i2c_msg msgs[] = {
		{
			.addr = (i2cId >> 1),
			.flags = 0,
			.len = a_sizeSendData,
			.buf = a_pSendData,
		},
		{
			.addr = (i2cId >> 1),
			.flags = I2C_M_RD,
			.len = a_sizeRecvData,
			.buf = a_pRecvData,
		},
	};

	if (i2c_transfer(lt8912_i2c_client->adapter, msgs, 2) < 0) {
		pr_err("hdmi8912b_read_bytes: transfer error\n");
		return EIO;
	} else{
        printk("%s:I2C read OK\n", __func__);
		return 0;
    }
}

void hdmi8912b_read_byte(unsigned char addr,u16 i2cId)
{
	char get_byte=0;
	char puSendCmd = { (char)(addr & 0xFF) };
	
	hdmi8912b_read_bytes(&puSendCmd , 1, (u8*)&get_byte, 1, i2cId);
	printk("%s addr = 0x%x get_byte = 0x%x  \n",__func__,addr ,get_byte);
}
#endif

#if 1
void lt8912b_initial(void)
{
    printk("=====lt8912b_initial=====\n");
	// i2c address = 0x90
	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR_WRITE, 0x08, 0xff);// Register address : 0x08; 	Value : 0xff
	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR_WRITE, 0x09, 0x81);
	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR_WRITE, 0x0a, 0xff);
	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR_WRITE, 0x0b, 0x64);//
	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR_WRITE, 0x0c, 0xff);

	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR_WRITE, 0x44, 0x31);// Close LVDS ouput
	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR_WRITE, 0x51, 0x1f);

	// i2c address = 0x90
	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR_WRITE, 0x31, 0xa1);
	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR_WRITE, 0x32, 0xa1);//0xa1
	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR_WRITE, 0x33, 0x03);//0x03 //0x03 Open HDMI Tx锟斤拷 0x00 Close HDMI Tx
	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR_WRITE, 0x37, 0x00);
	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR_WRITE, 0x38, 0x22);
	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR_WRITE, 0x60, 0x82);

	// i2c address  = 0x90
	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR_WRITE, 0x39, 0x45);
	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR_WRITE, 0x3b, 0x00);

	// i2c address  = 0x90
	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR_WRITE, 0x44, 0x31);
	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR_WRITE, 0x55, 0x44);
	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR_WRITE, 0x57, 0x01);
	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR_WRITE, 0x5a, 0x02);

	// i2c address  = 0x92
	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR92_WRITE, 0x10, 0x01); // 0x05 
	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR92_WRITE, 0x11, 0x01); // 0x12 
	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR92_WRITE, 0x12, 0x04);  
	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR92_WRITE, 0x13, MIPI_Lane%0x04);  // 00 4 lane  // 01 lane // 02 2 lane //03 3 lane
	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR92_WRITE, 0x14, 0x00);  
	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR92_WRITE, 0x15, 0x00);
	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR92_WRITE, 0x1a, 0x03);  
	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR92_WRITE, 0x1b, 0x03);  

	// i2c address  = 0x92
	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR92_WRITE, 0x18, (u8)(MIPI_Timing[H_sync]%256)); // hwidth
	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR92_WRITE, 0x19, (u8)(MIPI_Timing[V_sync]%256)); // vwidth
	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR92_WRITE, 0x1c, (u8)(MIPI_Timing[H_act]%256)); // H_active[7:0]
	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR92_WRITE, 0x1d, (u8)(MIPI_Timing[H_act]/256)); // H_active[15:8]

	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR92_WRITE, 0x1e, 0x67); // hs/vs/de pol hdmi sel pll sel
	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR92_WRITE, 0x2f, 0x0c); // fifo_buff_length 12

	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR92_WRITE, 0x34, (u8)(MIPI_Timing[H_tol]%256)); // H_total[7:0]
	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR92_WRITE, 0x35, (u8)(MIPI_Timing[H_tol]/256)); // H_total[15:8]
	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR92_WRITE, 0x36, (u8)(MIPI_Timing[V_tol]%256)); // V_total[7:0]
	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR92_WRITE, 0x37, (u8)(MIPI_Timing[V_tol]/256)); // V_total[15:8]
	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR92_WRITE, 0x38, (u8)(MIPI_Timing[V_bp]%256)); // VBP[7:0]
	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR92_WRITE, 0x39, (u8)(MIPI_Timing[V_bp]/256)); // VBP[15:8]
	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR92_WRITE, 0x3a, (u8)((MIPI_Timing[V_tol]-MIPI_Timing[V_act]-MIPI_Timing[V_bp]-MIPI_Timing[V_sync])%256)); // VFP[7:0]
	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR92_WRITE, 0x3b, (u8)((MIPI_Timing[V_tol]-MIPI_Timing[V_act]-MIPI_Timing[V_bp]-MIPI_Timing[V_sync])/256)); // VFP[15:8]
	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR92_WRITE, 0x3c, (u8)(MIPI_Timing[H_bp]%256)); // HBP[7:0]
	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR92_WRITE, 0x3d, (u8)(MIPI_Timing[H_bp]/256)); // HBP[15:8]
	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR92_WRITE, 0x3e, (u8)((MIPI_Timing[H_tol]-MIPI_Timing[H_act]-MIPI_Timing[H_bp]-MIPI_Timing[H_sync])%256)); // HFP[7:0]
	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR92_WRITE, 0x3f, (u8)((MIPI_Timing[H_tol]-MIPI_Timing[H_act]-MIPI_Timing[H_bp]-MIPI_Timing[H_sync])/256)); // HFP[15:8]

	// i2c address  = 0x92
	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR92_WRITE, 0x4e, 0x52);
	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR92_WRITE, 0x4f, 0xde);
	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR92_WRITE, 0x50, 0xc0);
	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR92_WRITE, 0x51, 0x80);
	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR92_WRITE, 0x51, 0x00);

	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR92_WRITE, 0x1f, 0x5e);
	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR92_WRITE, 0x20, 0x01);
	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR92_WRITE, 0x21, 0x2c);
	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR92_WRITE, 0x22, 0x01);
	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR92_WRITE, 0x23, 0xfa);
	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR92_WRITE, 0x24, 0x00);
	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR92_WRITE, 0x25, 0xc8);
	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR92_WRITE, 0x26, 0x00);
	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR92_WRITE, 0x27, 0x5e);
	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR92_WRITE, 0x28, 0x01);
	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR92_WRITE, 0x29, 0x2c);
	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR92_WRITE, 0x2a, 0x01);
	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR92_WRITE, 0x2b, 0xfa);
	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR92_WRITE, 0x2c, 0x00);
	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR92_WRITE, 0x2d, 0xc8);
	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR92_WRITE, 0x2e, 0x00);

	// i2c address  = 0x90
	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR_WRITE, 0x03, 0x7f);
	mdelay(10);
	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR_WRITE, 0x03, 0xff);

	// i2c address  = 0x92
//	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR92_WRITE, 0x51, 0x00);
	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR92_WRITE, 0x42, 0x64);
	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR92_WRITE, 0x43, 0x00);
	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR92_WRITE, 0x44, 0x04);
	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR92_WRITE, 0x45, 0x00);
	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR92_WRITE, 0x46, 0x59);
	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR92_WRITE, 0x47, 0x00);
	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR92_WRITE, 0x48, 0xf2);
	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR92_WRITE, 0x49, 0x06);
	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR92_WRITE, 0x4a, 0x00);
	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR92_WRITE, 0x4b, 0x72);
	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR92_WRITE, 0x4c, 0x45);
	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR92_WRITE, 0x4d, 0x00);
	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR92_WRITE, 0x52, 0x08);
	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR92_WRITE, 0x53, 0x00);
	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR92_WRITE, 0x54, 0xb2);
	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR92_WRITE, 0x55, 0x00);
	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR92_WRITE, 0x56, 0xe4);
	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR92_WRITE, 0x57, 0x0d);
	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR92_WRITE, 0x58, 0x00);
	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR92_WRITE, 0x59, 0xe4);
	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR92_WRITE, 0x5a, 0x8a);
	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR92_WRITE, 0x5b, 0x00);
	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR92_WRITE, 0x5c, 0x34);
	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR92_WRITE, 0x1e, 0x4f);
	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR92_WRITE, 0x51, 0x00);

	// i2c address  = 0x90
	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR_WRITE, 0xB2, 0x01);
	// i2c address  = 0x94
	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR94_WRITE, 0x06, 0x08);
	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR94_WRITE, 0x07, 0xF0);
	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR94_WRITE, 0x34, 0xD2);

	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR94_WRITE, 0x3c, 0x41);
	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR94_WRITE, 0x3e, 0x0a);
	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR94_WRITE, 0x43, 0x46);
	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR94_WRITE, 0x44, 0x10);
	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR94_WRITE, 0x45, 0x2A); //0x19:4:3 ; 0x2A : 16:9
	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR94_WRITE, 0x47, 0x00);
	// i2c address = 0x90
	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR_WRITE, 0x03, 0x7f);
	mdelay(10);
	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR_WRITE, 0x03, 0xff);
	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR92_WRITE, 0x51, 0x80);
	mdelay(10);
	hdmi8912b_write_byte(LT8912B_SLAVE_ADDR92_WRITE, 0x51, 0x00);
}
#endif

void lt8912_suspend(void)
{
	printk("++++++++lt8912_suspend++++++++++++\n");
    aeon_gpio_set("lt8912_rst_low");
	aeon_gpio_set("lt8912_pwren_low");
}

void lt8912_resume(void)
{
	printk("++++++++lt8912_resume++++++++++++\n");
#if 1
	aeon_gpio_set("lt8912_pwren_high");
	aeon_gpio_set("lt8912_rst_high");
	mdelay(20);
	aeon_gpio_set("lt8912_rst_low");
	mdelay(20);
	aeon_gpio_set("lt8912_rst_high");
    mdelay(40);
	
	lt8912b_initial();	
	mdelay(30);
#endif	
#if 1	
	printk("%s:+++++++++++LT8912B_read begin++++++++++++\n", __func__);
	hdmi8912b_read_byte(0x00, LT8912B_SLAVE_ADDR_WRITE);
	hdmi8912b_read_byte(0x01, LT8912B_SLAVE_ADDR_WRITE);
	hdmi8912b_read_byte(0x9C, LT8912B_SLAVE_ADDR_WRITE);
	hdmi8912b_read_byte(0x9D, LT8912B_SLAVE_ADDR_WRITE);
	hdmi8912b_read_byte(0x9E, LT8912B_SLAVE_ADDR_WRITE);
	hdmi8912b_read_byte(0x9F, LT8912B_SLAVE_ADDR_WRITE);
    
    hdmi8912b_read_byte(0x03, LT8912B_SLAVE_ADDR_WRITE);
    hdmi8912b_read_byte(0x45, LT8912B_SLAVE_ADDR_WRITE);
	printk("%s:++++++++LT8912B_read finish++++++++++++\n", __func__);
#endif
}

static int __init lt8912_iic_init(void)
{
    i2c_add_driver(&lt8912_iic_driver);
    printk( "lt8912_iic_init success\n");	
    return 0;
}

static void __exit lt8912_iic_exit(void)
{
    i2c_del_driver(&lt8912_iic_driver);  
}

module_init(lt8912_iic_init);
module_exit(lt8912_iic_exit);

MODULE_AUTHOR("Xiaokuan Shi");
MODULE_DESCRIPTION("LT8912 I2C Driver");
MODULE_LICENSE("GPL");

