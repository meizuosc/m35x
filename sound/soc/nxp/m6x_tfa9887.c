/*
 * tfa9887.c - handle nxp tfa9887
 *
 * Copyright (C) 2013 Wang Xiaowu
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/export.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/sysfs.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/mod_devicetable.h>
#include <linux/log2.h>
#include <linux/i2c.h>
#include <linux/i2c/tfa9887.h>
#include <asm/uaccess.h>


struct tfa9887_pdata *tfa9887 = NULL;

#define PA_ERR(fmt, args...) \
	printk(KERN_ERR" [TFA9887] %s: " fmt, __func__ , ## args)
#define PA_INFO(fmt, args...) \
	printk(KERN_INFO" [TFA9887] %s: " fmt, __func__ , ## args)

static int tfa9887_i2c_read(struct i2c_client *client,
				  int bytes, void *dest)
{
	struct i2c_msg xfer;
	int ret;

	xfer.addr = client->addr;
	xfer.flags = I2C_M_RD;
	xfer.len = bytes;
	xfer.buf = (char *)dest;

	ret = i2c_transfer(client->adapter, &xfer, 1);
	if (ret < 0)
		goto err_read;
	if (ret != 1) {
		ret = -EIO;
		goto err_read;
	}

	{
			int i = 0;
			for (i = 1; i < bytes; i++) {
				printk("rx[%d] = %2x\n", i, *((char *)dest + i - 1));
				if (!(i % 4))
					printk("\n");
			}
	}

	return 0;

err_read:
	printk("i2c read error\n");
	return ret;
}

static int tfa9887_i2c_write(struct i2c_client *client, int bytes, const void *src)
{
	struct i2c_msg xfer;
	int ret, i;

	xfer.addr = client->addr;
	xfer.flags = 0;
	xfer.len = bytes;
	xfer.buf = (char *)src;

	{
		int i = 0;
		for (i = 1; i < bytes; i++) {
			printk("rx[%d] = %2x\n", i, *((char *)src + i - 1));
			if (!(i % 4))
				printk("\n");
		}
	}

	ret = i2c_transfer(client->adapter, &xfer, 1);
	if (ret < 0)
		goto err_write;
	if (ret != 1) {
		ret = -EIO;
		goto err_write;
	}

	return 0;

err_write:
	return ret;
}

static int tfa9887_id(struct i2c_client *client)
{
	int ret, revision;
	char rbuf[2];
	char wbuf = TFA98XX_REVISION;

	ret = tfa9887_i2c_write(client, sizeof(wbuf), &wbuf);
	if (ret < 0) {
		dev_err(&client->dev, "failed to read tfa9887 rom id\n");
		return -EIO;
	}
	ret = tfa9887_i2c_read(client, 2, rbuf);
	if (ret < 0) {
		dev_err(&client->dev, "failed to read tfa9887 rom id\n");
		return -EINVAL;
	}

	revision = (rbuf[0] << 8) | (rbuf[1]);
	PA_INFO("NXP tfa9887 revision: %d\n", revision);

	return ret;
}


static int tfa9887_open(struct inode *inode, struct file *file)
{
	PA_INFO();
	return 0;
}

static int tfa9887_release(struct inode *inode, struct file *file)
{
	PA_INFO();
	return 0;
}

static long tfa9887_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	PA_INFO();
	void __user *argp = (void __user *)arg;
	struct tfa9887_write_read_img wrimg;
	uint8_t *write_data = NULL;
	uint8_t *read_data = NULL;
	int ret = 0;
	int i;

	switch (cmd) {
	case TFA9887_I2C_WRITE_READ:
		if (copy_from_user(&wrimg, argp, sizeof(wrimg))) {
			PA_ERR("%s: copy from user failed.\n", __func__);
			return -EFAULT;
		}
		if (NULL == wrimg.write) {
			PA_ERR("%s: out of memory\n", __func__);
			return -EINVAL;
		}
		write_data = (uint8_t *)kmalloc(wrimg.write_size * sizeof(uint8_t), GFP_KERNEL);
		if (!write_data) {
			PA_ERR("%s: out of memory\n", __func__);
			return -ENOMEM;
		}
		if (copy_from_user(write_data, wrimg.write, wrimg.write_size)) {
			PA_ERR("%s: copy data from user failed.\n", __func__);
			kfree(write_data);
			write_data = NULL;
			return -EFAULT;
		}

		tfa9887_i2c_write(tfa9887->client, wrimg.write_size, write_data);

		if (wrimg.read_size <= 0)
			break;

		read_data = (uint8_t *)kmalloc(wrimg.read_size * sizeof(uint8_t), GFP_KERNEL);
		if (!read_data) {
			PA_ERR("%s: out of memory\n", __func__);
			return -ENOMEM;
		}
		tfa9887_i2c_write(tfa9887->client, 1, write_data);
		tfa9887_i2c_read(tfa9887->client, wrimg.read_size, read_data);
		if (copy_to_user(wrimg.read, read_data, sizeof(read_data)))
			return -EFAULT;

		kfree(write_data);
		kfree(read_data);
		write_data = NULL;
		read_data = NULL;

		break;
	case TFA9887_ID:

		break;
	default:
		PA_ERR("%s: invalid command\n", __func__);
		ret = -EINVAL;
		break;
	}

	return ret;
}

static const struct file_operations tfa9887_fileops = {
	.owner = THIS_MODULE,
	.open = tfa9887_open,
	.unlocked_ioctl = tfa9887_ioctl,
	.release = tfa9887_release,
};

static struct miscdevice tfa9887_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "nxp_tfa9887",
	.fops = &tfa9887_fileops,
};

static int tfa9887_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret;

	tfa9887 = kzalloc(sizeof(struct tfa9887_pdata), GFP_KERNEL);
	if (!tfa9887) {
		PA_ERR("tfa9887 is NULL\n");
		ret = -ENOMEM;
		goto err_malloc;
	}

	tfa9887->dev = &client->dev;
	tfa9887->client = client;
	i2c_set_clientdata(client, tfa9887);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		PA_ERR("i2c check functionality error\n");
		ret = -ENODEV;
		goto err_i2c;
	}

	tfa9887_id(tfa9887->client);

	mutex_init(&tfa9887->tfa9887_mutex);
	ret = misc_register(&tfa9887_device);
	if (ret) {
		PA_ERR("%s: tfa9887_device register failed\n", __func__);
		goto err_register_device;
	}

	return ret;
err_i2c:
err_register_device:
	kfree(tfa9887);
err_malloc:
	return ret;
}

static int __devexit tfa9887_remove(struct i2c_client *client)
{
	struct tfa9887_pdata *tfa9887 = i2c_get_clientdata(client);

	PA_INFO();

	mutex_destroy(&tfa9887->tfa9887_mutex);
	// es305b_destroy_atts(tfa9887->dev);
	i2c_set_clientdata(client, NULL);
	kfree(tfa9887);
}

/*-------------------------------------------------------------------------*/

static const struct i2c_device_id tfa9887_ids[] = {
	{ "tfa9887", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tfa9887_ids);
static struct i2c_driver tfa9887_driver = {
	.driver = {
		.name = "tfa9887",
		.owner = THIS_MODULE,
	},
	.probe = tfa9887_probe,
	.remove = __devexit_p(tfa9887_remove),
	.id_table = tfa9887_ids,
};

static int __init tfa9887_init(void)
{
	return i2c_add_driver(&tfa9887_driver);
}
module_init(tfa9887_init);

static void __exit tfa9887_exit(void)
{
	i2c_del_driver(&tfa9887_driver);
}
module_exit(tfa9887_exit);

MODULE_DESCRIPTION("Driver for Tfa9887");
MODULE_AUTHOR("Wang Xiaowu");
MODULE_LICENSE("GPL");
