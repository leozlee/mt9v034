#ifndef __MT9V034_DEV_H__
#define __MT9V034_DEV_H__



struct v4l2_subdev;
struct soc_camera_device;


struct mt9v034_platform_data {
	unsigned int clk_pol:1;

	void (*set_clock)(struct v4l2_subdev *subdev, unsigned int rate);

	const s64 *link_freqs;
	s64 link_def_freq;

	//struct soc_camera_device *icd;
};







#endif /* ifndef OV2656_H */
