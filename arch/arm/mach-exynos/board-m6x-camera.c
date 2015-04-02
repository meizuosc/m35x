/*
 * m6x_camera.c - camera driver helper for m06x board
 *
 * Copyright (C) 2012 Meizu Technology Co.Ltd, Zhuhai, China
 * Author:  QuDao   <qudao@meizu.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
#include <linux/module.h>
#include <media/exynos_flite.h>
#include <media/exynos_gscaler.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <plat/gpio-cfg.h>
#include <mach/gpio-meizu.h>
#include <plat/devs.h>
#include <linux/delay.h>
#include <plat/mipi_csis.h>
#include <plat/iic.h>
#include <linux/spi/spi.h>
#include <plat/s3c64xx-spi.h>
#include <mach/spi-clocks.h>
#ifdef CONFIG_VIDEO_MS2R
#include <media/ms2r.h>
#endif
#include <asm/mach-types.h>

#define ISP_NAME "ms2r"

static void ms2r_configure_gpio(bool enable)
{
	int gpio;

	if (enable) {
		pr_debug("%s(), enable mclk && i2c && spi pin\n", __func__);
		s3c_gpio_cfgpin(MEIZU_ISP_XCLK_IN, S3C_GPIO_SFN(0x2));
		s3c_gpio_setpull(MEIZU_ISP_XCLK_IN, S3C_GPIO_PULL_NONE);

		s3c_gpio_cfgall_range(MEIZU_ISP_I2CSDA, 2,
				      S3C_GPIO_SFN(2), S3C_GPIO_PULL_UP);

		s3c_gpio_cfgpin(MEIZU_ISP_SPI_CLK, S3C_GPIO_SFN(2));
		s3c_gpio_setpull(MEIZU_ISP_SPI_CLK, S3C_GPIO_PULL_UP);
		s3c_gpio_cfgpin(MEIZU_ISP_SPI_CS, S3C_GPIO_OUTPUT);
		s3c_gpio_setpull(MEIZU_ISP_SPI_CS, S3C_GPIO_PULL_UP);
		s3c_gpio_cfgpin(MEIZU_ISP_SPI_MOSI, S3C_GPIO_SFN(2));
		s3c_gpio_setpull(MEIZU_ISP_SPI_MOSI, S3C_GPIO_PULL_UP);

		for (gpio = MEIZU_ISP_SPI_CLK;
				gpio <= MEIZU_ISP_SPI_MOSI; gpio++)
			s5p_gpio_set_drvstr(gpio, S5P_GPIO_DRVSTR_LV3);

	} else {
		pr_debug("%s(), disable mclk && i2c && spi pin\n", __func__);
		s3c_gpio_cfgpin(MEIZU_ISP_XCLK_IN, S3C_GPIO_INPUT);
		s3c_gpio_setpull(MEIZU_ISP_XCLK_IN, S3C_GPIO_PULL_DOWN);

		s3c_gpio_cfgall_range(MEIZU_ISP_I2CSDA, 2,
				      S3C_GPIO_INPUT, S3C_GPIO_PULL_NONE);

		s3c_gpio_cfgall_range(MEIZU_ISP_SPI_CLK, 4,
				      S3C_GPIO_INPUT, S3C_GPIO_PULL_NONE);
	}
}


#ifdef CONFIG_VIDEO_MS2R
static int ms2r_init_gpio(struct device *dev)
{
	int ret = 0;

	/*ms2r reset pin shoud be initialized to low*/
	ret = gpio_request_one(MEIZU_ISP_RST, GPIOF_OUT_INIT_LOW, "MS2R_RESET");
	if (ret) {
		pr_err("%s():gpio_request failed\n", __func__);
		return -EINVAL;
	}

	ms2r_configure_gpio(false);
	
	return ret;
}

/*
* This configure the 24M clock for external ISP
* See exynos5_fimc_is_cfg_clk()[setup-fimc-is.c]
*/
static int ms2r_init_clock(struct device *dev)
{
	struct clk *mout_ipll = NULL;
	struct clk *sclk_mout_isp_sensor = NULL;
	struct clk *sclk_isp_sensor0 = NULL;
	int ret = 0;

	mout_ipll = clk_get(NULL, "mout_ipll");
	if (IS_ERR(mout_ipll)) {
		pr_err("%s : clk_get(mout_ipll) failed\n", __func__);
		return -EINVAL;
	}

	sclk_mout_isp_sensor = clk_get(NULL, "sclk_mout_isp_sensor");
	if (IS_ERR(sclk_mout_isp_sensor)) {
		pr_err("%s : clk_get(sclk_mout_isp_sensor) failed\n", __func__);
		ret = -EINVAL;
		goto err1;
	}

	sclk_isp_sensor0 = clk_get(NULL, "sclk_isp_sensor0");
	if (IS_ERR(sclk_isp_sensor0)) {
		pr_err("%s : clk_get(sclk_isp_sensor0) failed\n", __func__);
		ret = -EINVAL;
		goto err2;
	}

	if (clk_set_parent(sclk_mout_isp_sensor, mout_ipll)) {
		ret = -EINVAL;
		pr_err("%s(): unable to set parent!\n", __func__);
		goto err3;
	}

	clk_set_rate(sclk_isp_sensor0, 24000000);
	
err3:
	clk_put(sclk_isp_sensor0);
err2:
	clk_put(sclk_mout_isp_sensor);
err1:
	clk_put(mout_ipll);

	return ret;
}

static int ms2r_set_isp_power(bool enable)
{
	struct regulator_bulk_data supplies[2];
	int num_consumers = 0, ret;

	pr_info("%s():enable = %d\n", __func__, enable);

	supplies[num_consumers++].supply = "vdd18_ispa";
	supplies[num_consumers++].supply = "vdd12_ispa";
	
	ret = regulator_bulk_get(NULL, num_consumers, supplies);
	if (ret) {
		pr_err("%s():isp regulator_bulk_get failed\n", __func__);
		return ret;
	}

	if (enable) {
		ret = regulator_bulk_enable(num_consumers, supplies);
		if (ret) {
			pr_info("%s(), Power ON ISP failed!ret: %d\n",
				__func__, ret);
			goto exit_regulator;
		}
		msleep(2);
		gpio_set_value(MEIZU_ISP_RST, 1);
		if (machine_is_m69()) {
			pr_info("%s(), m69: isp1.2v read is %d\n", __func__,
				regulator_get_voltage(supplies[1].consumer));
		}
	} else { 
		gpio_set_value(MEIZU_ISP_RST, 0);
		
		ret = regulator_bulk_disable(num_consumers, supplies);
		if (ret) {
			pr_info("%s(), Power OFF ISP failed!ret: %d\n",
				__func__, ret);
			goto exit_regulator;
		}
	}

exit_regulator:
	regulator_bulk_free(num_consumers, supplies);

	return ret;
}

static int ms2r_set_sensor_power(int cam_id, bool enable)
{
	struct regulator_bulk_data supplies[7];
	int num_consumers = 0, ret;

	pr_info("%s():camera id = %d, enable = %d\n", __func__, cam_id, enable);
#if 1
	if (enable) {
		if (cam_id == 0) {  /* Rear IMX179. Why greater than M040 */
			supplies[num_consumers++].supply = "vcc28_8msen";
			supplies[num_consumers++].supply = "vdd18_8msen";
			supplies[num_consumers++].supply = "vcc28_af";//AF
			supplies[num_consumers++].supply = "vdd12_8msen";
		} else if (cam_id == 1) {  /* Front IMX132 */
			supplies[num_consumers++].supply = "vdd28_fcamera"; // old
			supplies[num_consumers++].supply = "vdd12_fcamera";
			supplies[num_consumers++].supply = "vdd18_fcamera";
		} else {
			pr_err("%s:wrong camera index\n", __func__);
			return -EINVAL;
		}
	} else {
		if (cam_id == 0) {  /* Rear IMX179. Why greater than M040 */
			supplies[num_consumers++].supply = "vdd12_8msen";
			supplies[num_consumers++].supply = "vcc28_8msen";
			supplies[num_consumers++].supply = "vcc28_af";//AF
			supplies[num_consumers++].supply = "vdd18_8msen";
		} else if (cam_id == 1) {  /* Front IMX132 */
			supplies[num_consumers++].supply = "vdd18_fcamera";
			supplies[num_consumers++].supply = "vdd12_fcamera";
			supplies[num_consumers++].supply = "vdd28_fcamera";
		} else {
			pr_err("%s:wrong camera index\n", __func__);
			return -EINVAL;
		}
	}
#else
	pr_info("%s(), For test, manipulate FRONT && REAR together\n", __func__);
	supplies[num_consumers++].supply = "vdd12_8msen";
	supplies[num_consumers++].supply = "vcc28_8msen";
	supplies[num_consumers++].supply = "vcc28_af";//AF
	supplies[num_consumers++].supply = "vdd18_8msen";

	supplies[num_consumers++].supply = "vdd28_fcamera";
	supplies[num_consumers++].supply = "vdd18_fcamera";
	supplies[num_consumers++].supply = "vdd12_fcamera";
#endif
	ret = regulator_bulk_get(NULL, num_consumers, supplies);
	if (ret) {
		pr_err("%s():isp regulator_bulk_get failed\n", __func__);
		return ret;
	}

	if (enable) {
		ms2r_configure_gpio(enable);
		ret = regulator_bulk_enable(num_consumers, supplies);
		if (ret) {
			pr_info("%s(), Power ON Camera %d failed! ret: %d\n",
				__func__, cam_id, ret);
			goto exit_regulator;
		}
	} else {
		ms2r_configure_gpio(enable);
		ret = regulator_bulk_disable(num_consumers, supplies);
		if (ret) {
			pr_info("%s(), Power OFF Camera %d failed! ret: %d\n",
				__func__, cam_id, ret);
			goto exit_regulator;
		}
	}

	pr_info("%s(), sleep for 2ms\n", __func__);
	msleep(2);
 exit_regulator:
	regulator_bulk_free(num_consumers, supplies);

	return ret;
}

static void ms2r_reset(void)
{
	gpio_set_value(MEIZU_ISP_RST, 0);
	msleep(5);
	gpio_set_value(MEIZU_ISP_RST, 1);
	msleep(5);
}

/*
* This enables fimc module's operating clock and clocks for isp.
*/
static int ms2r_clock_enable(struct device *dev, bool enable)
{
	struct clk *fimc_clk, *clk;
	int ret = 0;

	/* be able to handle clock on/off only with this clock
	* This is for FIMC-LITE module.
	*/
	fimc_clk = clk_get(NULL, "gscl_flite0");
	if (IS_ERR(fimc_clk)) {
		dev_err(dev, "failed to get interface clock\n");
		return -EINVAL;
	}

	/* mclk */
	clk = clk_get(NULL, "sclk_isp_sensor0");
	if (IS_ERR(clk)) {
		dev_err(dev, "failed to get mclk source\n");
		ret = -EINVAL;
		goto exit_clkget_cam;
	}

	if (enable) {
		clk_enable(fimc_clk);
		clk_enable(clk);
	} else {
		clk_disable(clk);
		clk_disable(fimc_clk);
	}

	pr_debug("%s(), gscl_flite0 rate is %lu\n", __func__,
		clk_get_rate(fimc_clk));
	pr_debug("%s(), sclk_isp_sensor0 rate is %lu\n", __func__,
		clk_get_rate(clk));

	clk_put(clk);

exit_clkget_cam:
	clk_put(fimc_clk);

	pr_info("%s(), after enable: %d, sleep for 2ms\n", __func__, enable);
	msleep(2);
	
	return ret;
}

static struct ms2r_platform_data ms2r_plat = {
	.default_width = 640,
	.default_height = 480,
	.pixelformat = V4L2_PIX_FMT_UYVY,
	.freq = 24000000,
	.is_mipi = 1,

	.init_gpio = ms2r_init_gpio,
	.init_clock = ms2r_init_clock,
	.set_isp_power = ms2r_set_isp_power,
	.set_sensor_power = ms2r_set_sensor_power,
	.reset = ms2r_reset,
	.clock_enable = ms2r_clock_enable,
};

static struct i2c_board_info i2c_devs0[] = {
	{
		I2C_BOARD_INFO(ISP_NAME, 0x1f),
		.platform_data = &ms2r_plat,
		//.irq = MEIZU_ISP_IRQ,/*M6X, GPK1_0, Should be configured as INT somewhere*/
	},
};
#else
static struct i2c_board_info i2c_devs0[] = {
};
#endif

#ifdef CONFIG_S3C64XX_DEV_SPI0
int ms2r_spi_cfg_cs(int gpio, int ch_num, int level)
{
	char cs_name[16];

	snprintf(cs_name, sizeof(cs_name), "SPI_CS%d", ch_num);

	if (gpio_request(gpio, cs_name))
		return -EIO;

	gpio_direction_output(gpio, level);
	s3c_gpio_cfgpin(gpio, S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(gpio, S3C_GPIO_PULL_UP);
	gpio_free(gpio);

	return 0;
}

static struct s3c64xx_spi_csinfo spi0_csi[] = {
	[0] = {
		//.line		= MEIZU_ISP_SPI_CS,
		.set_level	= gpio_set_value,
		.fb_delay	= 0x2,
	},
};

static struct spi_board_info spi0_board_info[] __initdata = {
	{
		.modalias		= "ms2r_spi",
		.platform_data		= NULL,
		.max_speed_hz		= 25 * 1000 * 1000,
		.bus_num		= 0,
		.chip_select		= 0,
		.mode			= SPI_MODE_1 | SPI_CS_HIGH,
		.controller_data	= &spi0_csi[0],
	}
};
#endif

#if defined(CONFIG_VIDEO_EXYNOS_GSCALER) && defined(CONFIG_VIDEO_EXYNOS_FIMC_LITE)
#if defined(CONFIG_VIDEO_MS2R)
static struct exynos_isp_info ms2r_isp = {
	.board_info	= i2c_devs0,
	/*
	* This is set to 24M Hz.
	*/
	.cam_srclk_name	= "sclk_isp_sensor",
	.clk_frequency  = 24000000UL,
	.bus_type	= CAM_TYPE_MIPI,
	.cam_clk_src_name = "dout_aclk_333_432_gscl",
	.cam_clk_name	= "aclk_333_432_gscl",
	/* Use MIPI CSI0 */
	.camif_clk_name	= "gscl_flite0",
	.i2c_bus_num	= 0,
	.cam_port	= CAM_PORT_A, /* A-Port : 0 */

	.flags		= CAM_CLK_INV_VSYNC,
	.csi_data_align = 32,
};
/* This is for platdata of fimc-lite */
static struct s3c_platform_camera flite_ms2r = {
	.type		= CAM_TYPE_MIPI,
	.use_isp	= true,
	/*
	* Copied from M6MO
	*/
	.inv_pclk	= 0,
	.inv_vsync = 1,
	.inv_href	= 0,
	.inv_hsync = 0,
};
#endif

static struct platform_device *m6x_camera_platformdevices[] __initdata = {
	&s3c64xx_device_spi0, /*For ISP*/
};

/*
* Init camera subsystem gpios.
* MCLK, I2C
*/
static void __init m6x_camera_init_gpio(void)
{
	/*
	* 24MHz pin config
	* "sclk_isp_sensor0"
	*/
	gpio_request(MEIZU_ISP_XCLK_IN, "ISPXCLK");
	s3c_gpio_cfgpin(MEIZU_ISP_XCLK_IN, S3C_GPIO_SFN(0x2));
	s3c_gpio_setpull(MEIZU_ISP_XCLK_IN, S3C_GPIO_PULL_NONE);
	gpio_free(MEIZU_ISP_XCLK_IN);

	/*
	* I2C
	*/
	gpio_request(MEIZU_ISP_I2CSDA, "ISPI2CSDA");
	s3c_gpio_cfgpin(MEIZU_ISP_I2CSDA, S3C_GPIO_SFN(0x2));
	s3c_gpio_setpull(MEIZU_ISP_I2CSDA, S3C_GPIO_PULL_NONE);
	gpio_free(MEIZU_ISP_I2CSDA);

	gpio_request(MEIZU_ISP_I2CSCL, "ISPI2CSCL");
	s3c_gpio_cfgpin(MEIZU_ISP_I2CSCL, S3C_GPIO_SFN(0x2));
	s3c_gpio_setpull(MEIZU_ISP_I2CSCL, S3C_GPIO_PULL_NONE);
	gpio_free(MEIZU_ISP_I2CSCL);

	/*
	* ISP INT
	*/
	s3c_gpio_cfgpin(MEIZU_ISP_IRQ, S3C_GPIO_SFN(0xF));
	s3c_gpio_setpull(MEIZU_ISP_IRQ, S3C_GPIO_PULL_NONE);
	s5p_register_gpio_interrupt(MEIZU_ISP_IRQ);
	
}

/*
* Set @camcording to 1 if we wanna utilize the gsc to record,
* @previe is the same way.
*/
static void __set_gsc_camera_config(struct exynos_platform_gscaler *data,
					u32 active_index, u32 preview,
					u32 camcording, u32 max_cam)
{
	data->active_cam_index = active_index;
	data->cam_preview = preview;
	data->cam_camcording = camcording;
	data->num_clients = max_cam;
}

static void __set_flite_camera_config(struct exynos_platform_flite *data,
					u32 active_index, u32 max_cam)
{
	data->active_cam_index = active_index;
	data->num_clients = max_cam;
}


static void __init m6x_set_camera_platdata(void)
{
	int gsc_cam_index = 0;
	int flite0_cam_index = 0;
	int flite1_cam_index = 0;
	int flite2_cam_index = 0;
#if defined(CONFIG_VIDEO_MS2R)
	i2c_devs0[0].irq = MEIZU_ISP_IRQ;
	exynos_gsc1_default_data.isp_info[gsc_cam_index++] = &ms2r_isp;

	exynos_flite0_default_data.cam[flite0_cam_index] = &flite_ms2r;
	exynos_flite0_default_data.isp_info[flite0_cam_index] = &ms2r_isp;
	flite0_cam_index++;

	/*
	* EXYNOS5410 MIPI CSIS0/1 has 4 lanes.
	* M6X utilize MIPI CSIS0.
	*/
	s5p_mipi_csis0_default_data.lanes = 4;
	s5p_mipi_csis0_default_data.alignment = ms2r_isp.csi_data_align;
	s5p_mipi_csis0_default_data.hs_settle = 8;

#endif

	/*
	* exynos_flite0_default_data is going
	* to be "struct platform_device exynos_device_flite0" 's platdata.
	*/
	__set_flite_camera_config(&exynos_flite0_default_data, 0, flite0_cam_index);
	__set_flite_camera_config(&exynos_flite1_default_data, 0, flite1_cam_index);
	__set_flite_camera_config(&exynos_flite2_default_data, 0, flite2_cam_index);

	/* gscaler platdata register */
	/* GSC-0 */
	__set_gsc_camera_config(&exynos_gsc1_default_data, 0, 1, 0, gsc_cam_index);

	/* GSC-1 */
	/* GSC-2 */
	/* GSC-3 */
}

#ifdef CONFIG_S3C64XX_DEV_SPI0
static void __init spi_rt_init_res(void) {
	spi0_csi[0].line = MEIZU_ISP_SPI_CS;
}
#endif

static void __init m6x_add_camera_platdata(void)
{
#ifdef CONFIG_VIDEO_EXYNOS_MIPI_CSIS
	s3c_set_platdata(&s5p_mipi_csis0_default_data,
			sizeof(s5p_mipi_csis0_default_data), &s5p_device_mipi_csis0);
	s3c_set_platdata(&s5p_mipi_csis1_default_data,
			sizeof(s5p_mipi_csis1_default_data), &s5p_device_mipi_csis1);
	s3c_set_platdata(&s5p_mipi_csis2_default_data,
			sizeof(s5p_mipi_csis2_default_data), &s5p_device_mipi_csis2);
#endif
#ifdef CONFIG_VIDEO_EXYNOS_FIMC_LITE
	s3c_set_platdata(&exynos_flite0_default_data,
			sizeof(exynos_flite0_default_data), &exynos_device_flite0);
	s3c_set_platdata(&exynos_flite1_default_data,
			sizeof(exynos_flite1_default_data), &exynos_device_flite1);
	s3c_set_platdata(&exynos_flite2_default_data,
			sizeof(exynos_flite2_default_data), &exynos_device_flite2);
#endif
#ifdef CONFIG_VIDEO_EXYNOS_GSCALER
	exynos5_gsc_set_ip_ver(IP_VER_GSC_5A);

	s3c_set_platdata(&exynos_gsc0_default_data, sizeof(exynos_gsc0_default_data),
			&exynos5_device_gsc0);
	s3c_set_platdata(&exynos_gsc1_default_data, sizeof(exynos_gsc1_default_data),
			&exynos5_device_gsc1);
	s3c_set_platdata(&exynos_gsc2_default_data, sizeof(exynos_gsc2_default_data),
			&exynos5_device_gsc2);
	s3c_set_platdata(&exynos_gsc3_default_data, sizeof(exynos_gsc3_default_data),
			&exynos5_device_gsc3);
#endif

#ifdef CONFIG_S3C64XX_DEV_SPI0
	exynos_spi_clock_setup(&s3c64xx_device_spi0.dev, 0);
	spi_rt_init_res();
	if (!ms2r_spi_cfg_cs(spi0_csi[0].line, 0, 0)) {
		s3c64xx_spi0_set_platdata(&s3c64xx_spi0_pdata,
			EXYNOS_SPI_SRCCLK_SCLK, ARRAY_SIZE(spi0_csi));

		spi_register_board_info(spi0_board_info,
			ARRAY_SIZE(spi0_board_info));
	} else {
		pr_err("%s: Error requesting gpio for SPI-CH0 CS\n", __func__);
	}
#endif


}

static void __init m6x_register_platformdevice(void)
{
	platform_add_devices(m6x_camera_platformdevices,
			ARRAY_SIZE(m6x_camera_platformdevices));
}
/*
* Configure FIMC-LITE and CSI's platdata.
* Make sure I2C0 that utilized by external ISP is properly configured.
*/
static int __init m6x_init_camera(void)
{
	/*
	* Utilize I2C0 @400KHz to communicate with MS2R
	*/
	pr_info("%s(): ++++\n", __func__);
	m6x_camera_init_gpio();
	s3c_i2c0_set_platdata(NULL);
	//i2c_register_board_info(0, i2c_devs0, ARRAY_SIZE(i2c_devs0));

	m6x_set_camera_platdata();
	m6x_add_camera_platdata();
	m6x_register_platformdevice();
	pr_info("%s(): ----\n", __func__);
	return 0;

}

arch_initcall(m6x_init_camera);

#endif /* CONFIG_VIDEO_EXYNOS_GSCALER */

MODULE_DESCRIPTION("m6x fimc-lite and camera driver helper");
MODULE_AUTHOR("QuDao <qudaoman@126.com>");
MODULE_LICENSE("GPLV2");


