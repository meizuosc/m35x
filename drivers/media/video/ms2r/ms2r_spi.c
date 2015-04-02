/*
 * Driver for MS2R SPI
 *
 * Copyright (c) 2013, Meizu. All rights reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/gpio.h>
#include <plat/gpio-cfg.h>

static struct spi_device *g_spi;

int ms2r_spi_write(const void *buf, size_t size)
{
	int ret = 0;
	if (!g_spi) {
		pr_err("%s(), ms2r spi device is NOT initialized!\n", __func__);
		return -ENODEV;
	}
	pr_info("%s(), write 0x%x bytes\n", __func__, size);
	ret = spi_write(g_spi, buf, size);
	if (ret < 0) {
		pr_err("%s(), spi_write() err: %d\n", __func__, ret);
	}
	return ret;
}

static int __devinit ms2r_spi_probe(struct spi_device *spi)
{
	int ret = 0;

	dev_info(&spi->dev, "%s(), utilize SPI%d at frequence %dHz\n",
		__func__, spi->master->bus_num, spi->max_speed_hz);

	/* spi->bits_per_word = 16; */
	if (spi_setup(spi)) {
		dev_err(&spi->dev, "failed to setup spi for ms2r\n");
		ret = -EINVAL;
		goto exit;
	}

	g_spi = spi;

exit:
	return ret;

}

static int __devexit ms2r_spi_remove(struct spi_device *spi)
{
	g_spi = NULL;
	return 0;
}

static struct spi_driver ms2r_spi_driver = {
	.driver = {
		.name = "ms2r_spi",
		.bus = &spi_bus_type,
		.owner = THIS_MODULE,
	},
	.probe = ms2r_spi_probe,
	.remove = __devexit_p(ms2r_spi_remove),
};

static int __init ms2r_is_spi_init(void)
{
	int ret;

	ret = spi_register_driver(&ms2r_spi_driver);

	if (ret)
		pr_err("failed to register imc_is_spi- %x\n", ret);

	return ret;
}

static void __exit ms2r_is_spi_exit(void)
{
	spi_unregister_driver(&ms2r_spi_driver);
}

module_init(ms2r_is_spi_init);
module_exit(ms2r_is_spi_exit);

MODULE_AUTHOR("QuDao <qudaoman@126.com>");
MODULE_DESCRIPTION("MS2R SPI driver");
MODULE_LICENSE("GPL");

