
#include <linux/irq.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <media/soc_camera.h>
#include <linux/videodev2.h>
#include <media/videobuf-core.h>
#include <media/v4l2-device.h>
#include "mt9v034_dev.h"


#define MT9V034_I2C_ADDR				(0x90>>1)
#define MT9V034_MODULE_NAME				"mt9v034"
#define MT9V034_I2C_BUS_NUM	3



static unsigned long mt9v034_camera_query_bus_param(struct soc_camera_link *link)
{
	return 0;

}

static int mt9v034_camera_set_bus_param(struct soc_camera_link *link, unsigned long flags)
{
	return 0;

}

static void mt9v034_camera_free_bus(struct soc_camera_link *link)
{


}


// static void mt9v034_set_clock(struct v4l2_subdev *subdev, unsigned int rate)
// {

// }




// static const s64 mt9v034_link_freqs[] = {
// 	13000000,
// 	26600000,
// 	27000000,
// 	0,
// };


// static struct mt9v034_platform_data mt9v034_platform_data = {
// 	.clk_pol = 0,
// 	.set_clock = mt9v034_set_clock,
// 	.link_freqs = mt9v034_link_freqs,
// 	.link_def_freq = 26600000,
// 	.icd = &mt9v034_soc_camera_device;
// };




static struct i2c_board_info mt9v034_i2c_device_info;


static struct soc_camera_link iclink[] = {
	{
		.bus_id				= 0, /* Must match with the camera ID */
		.board_info			= &mt9v034_i2c_device_info,
		.i2c_adapter_id		= 0,
		.query_bus_param	= mt9v034_camera_query_bus_param,
		.set_bus_param		= mt9v034_camera_set_bus_param,
		.free_bus			= mt9v034_camera_free_bus,
		.module_name		= MT9V034_MODULE_NAME,
	}, 
};


struct soc_camera_device mt9v034_soc_camera_device = {
	.dev = {
			.platform_data = &iclink,
	},
};



/* Board I2C devices. */
static struct i2c_board_info mt9v034_i2c_device_info = {
	I2C_BOARD_INFO("mt9v034", MT9V034_I2C_ADDR),
	.platform_data = &mt9v034_soc_camera_device,
};



static struct i2c_client *mt9v034_client;

/*
 * 1.驱动入口
 * */
static int mt9v034_dev_init(void)
{
	//创建i2c设备，i2c设配器
	struct i2c_adapter *adapter = i2c_get_adapter(2);	//从SoC的注册信息以及硬件上得知
	int ret = 0;
	mt9v034_client = i2c_new_device(adapter,&mt9v034_i2c_device_info);
	if(mt9v034_client == NULL)
	{
		printk("mt9v034_client is null\n");
		return -1;
	}

	printk("in dev i also want to check client->adapter->id is %d\r\n",mt9v034_client->adapter->id);
	i2c_put_adapter(adapter);


	ret = i2c_smbus_read_byte_data(mt9v034_client,0x00);
	printk("addr:0x00  val=0x%x\n",ret);

	// ret = i2c_smbus_read_byte_data(mt9v034_client,0x1B);
	// printk("addr:0x00  val=%d\n",ret);


	printk("mt9v034_dev_init\n");
	return 0;
}

/*
 * 2.驱动出口
 * */
static void mt9v034_dev_exit(void)
{
	printk("mt9v034_dev_exit\n");
	i2c_unregister_device(mt9v034_client);
}


module_init(mt9v034_dev_init);
module_exit(mt9v034_dev_exit);
MODULE_DESCRIPTION("Aptina MT9V034 Camera driver device");
MODULE_AUTHOR("CareDrive");
MODULE_LICENSE("GPL");

