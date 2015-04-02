/*
 * Copyright (c) 2012-2013 MEIZU Incorporated
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
 */

#define FUNCTION_DATA rmi_fn_51_data
#define FNUM 51


#include <linux/module.h>
#include <linux/slab.h>
#include <linux/leds.h>
#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/input/synaptics_dsx.h>

#include "synaptics_dsx_i2c.h"

#define F51_LED_USED
#ifdef	CONFIG_LEDS_MXHUB
#undef	F51_LED_USED
#endif

union f51_query {
	struct {
		/* Query 0 */
		u8 query_data;
	} __attribute__((__packed__));
	struct {
		u8 data[1];
		u16 address;
	} __attribute__((__packed__));
};

union f51_control_0 {	
	struct {
		/*Control 0.* */
		u8 en_single_tap_int:1;
		u8 en_double_tap_int:1;
		u8 en_0d_touched_int:1;
		u8 en_0d_released_int:1;
		u8 rev:1;
		u8 anti_err_active:1;
		u8 wake_up_active:1;
		u8 partitioned_mode:1;
	} __attribute__((__packed__));	
	struct {
		u8 data[1];
		u16 address;
	} __attribute__((__packed__));
};

union f51_control_1 {	
	struct {
		/*Control 1.* */
		u8 key_ttap;
	} __attribute__((__packed__));	
	struct {
		u8 data[1];
		u16 address;
	} __attribute__((__packed__));
};

union f51_control_2_3 {
	struct {
		/* control 2 3*/
		u16 wake_up_left;
	} __attribute__((__packed__));
	struct {
		u8 data[2];
		u16 address;
	} __attribute__((__packed__));
};

union f51_control_4_5 {
	struct {
		/* control 4 5*/
		u16 wake_up_top;
	} __attribute__((__packed__));
	struct {
		u8 data[2];
		u16 address;
	} __attribute__((__packed__));
};

union f51_control_6_7{
	struct {
		/* control 6 7*/
		u16 wake_up_right;
	} __attribute__((__packed__));
	struct {
		u8 data[2];
		u16 address;
	} __attribute__((__packed__));
};


union f51_control_8_9 {
	struct {
		/* control 18 19*/
		u16 wake_up_bottom;
	} __attribute__((__packed__));
	struct {
		u8 data[2];
		u16 address;
	} __attribute__((__packed__));
};

union f51_control_10_11 {
	struct {
		/* control 10 11*/
		u16 anti_err_left;
	} __attribute__((__packed__));
	struct {
		u8 data[2];
		u16 address;
	} __attribute__((__packed__));
};

union f51_control_12_13 {
	struct {
		/* control 12 13*/
		u16 anti_err_top;
	} __attribute__((__packed__));
	struct {
		u8 data[2];
		u16 address;
	} __attribute__((__packed__));
};

union f51_control_14_15 {
	struct {
		/* control 14 15*/
		u16 anti_err_right;
	} __attribute__((__packed__));
	struct {
		u8 data[2];
		u16 address;
	} __attribute__((__packed__));
};

union f51_control_16_17 {
	struct {
		/* control 16 17*/
		u16 anti_err_bottom;
	} __attribute__((__packed__));
	struct {
		u8 data[2];
		u16 address;
	} __attribute__((__packed__));
};


#define RMI_F51_NUM_CTRL_REGS 28
struct f51_control {
	/* Control 0 */
	union f51_control_0 *reg_0;

	/* Control 1 */
	union f51_control_1 *reg_1;

	/* Control 2 3*/
	union f51_control_2_3 *reg_2_3;

	/* Control 4 5*/
	union f51_control_4_5*reg_4_5;

	/* Control 6 7*/
	union f51_control_6_7*reg_6_7;

	/* Control 8 9*/
	union f51_control_8_9*reg_8_9;

	/* Control 10 11*/
	union f51_control_10_11*reg_10_11;

	/* Control 12 13*/
	union f51_control_12_13*reg_12_13;

	/* Control 14 15*/
	union f51_control_14_15*reg_14_15;

	/* Control 16 17*/
	union f51_control_16_17*reg_16_17;
};


union f51_data_2 {
	struct {
		/* Data 2 */
		u8 tap_detected:1;
		u8 double_tap_detected:1;
		u8 d0_touched:1;
		u8 d0_released:1;
		u8 rev0:1;
		u8 anti_err_touched:1;
		u8 rev1:2;
	} __attribute__((__packed__));
	struct {
		u8 data[1];
		u16 address;
	} __attribute__((__packed__));
};

struct f51_data {	
	/* Data 0 */
	union f51_data_2 *reg_2;
};


#ifdef	F51_LED_USED
struct rmi_led{
	struct led_classdev  led_cdev;
	int			 brightness;
	int			 slope;
	int			 pwm;
};
#endif


/* data specific to fn $51 that needs to be kept around */
struct rmi_fn_51_data {
	struct device *dev;
	union f51_query query;
	struct f51_control control;
	struct f51_data data;
#ifdef	F51_LED_USED
	struct rmi_led led;
#endif
	
	unsigned short query_base_addr;
	unsigned short control_base_addr;
	unsigned short data_base_addr;
	unsigned short command_base_addr;
	
	struct kobject *attr_dir;
	
	struct mutex control_mutex;
	struct mutex data_mutex;
	struct mutex cmd_mutex;
	struct synaptics_rmi4_exp_fn_ptr *fn_ptr;
	struct synaptics_rmi4_data *rmi4_data;
};


static struct rmi_fn_51_data *f51= NULL;

static struct completion remove_complete;



static int rmi_f51_alloc_memory(struct synaptics_rmi4_data *rmi4_data);

static void rmi_f51_free_memory(struct synaptics_rmi4_data *rmi4_data);

static int rmi_f51_initialize(struct synaptics_rmi4_data *rmi4_data);


#ifdef	F51_LED_USED
static int rmi_f51_initled(struct synaptics_rmi4_data *rmi4_data);
#endif

static int rmi_f51_create_sysfs(struct synaptics_rmi4_data *rmi4_data);

/* Sysfs files */

/* Query sysfs files */


show_prototype(query_data)

static struct attribute *attrs[] = {
	attrify(query_data),
	NULL
};
static struct attribute_group attrs_query = GROUP(attrs);
/* Control sysfs files */
show_store_prototype(en_single_tap_int)
show_store_prototype(en_double_tap_int)
show_store_prototype(en_0d_touched_int)
show_store_prototype(en_0d_released_int)
show_store_prototype(anti_err_active)
show_store_prototype(wake_up_active)
show_store_prototype(partitioned_mode)
show_store_prototype(key_ttap)

show_store_prototype(wake_up_left)
show_store_prototype(wake_up_top)
show_store_prototype(wake_up_right)
show_store_prototype(wake_up_bottom)
show_store_prototype(anti_err_left)
show_store_prototype(anti_err_top)
show_store_prototype(anti_err_right)
show_store_prototype(anti_err_bottom)



static struct attribute *attrs2[] = {
	attrify(en_single_tap_int),
	attrify(en_double_tap_int),
	attrify(en_0d_touched_int),
	attrify(en_0d_released_int),
	attrify(anti_err_active),
	attrify(wake_up_active),
	attrify(partitioned_mode),
	attrify(key_ttap),
	attrify(wake_up_left),
	attrify(wake_up_top),
	attrify(wake_up_right),
	attrify(wake_up_bottom),
	attrify(anti_err_left),
	attrify(anti_err_top),
	attrify(anti_err_right),
	attrify(anti_err_bottom),
	NULL
};

static struct attribute_group attrs_control = GROUP(attrs2);

/* Data sysfs files */
show_store_prototype(tap_detected)
show_store_prototype(double_tap_detected)
show_store_prototype(d0_touched)
show_store_prototype(d0_released)
show_store_prototype(anti_err_touched)

static struct attribute *attrs3[] = {
	attrify(tap_detected),
	attrify(double_tap_detected),
	attrify(d0_touched),
	attrify(d0_released),
	attrify(anti_err_touched),
	NULL
};
static struct attribute_group attrs_data = GROUP(attrs3);


static int rmi_f51_init(struct synaptics_rmi4_data *rmi4_data)
{
	int retval = 0;
	
	dev_info(&rmi4_data->i2c_client->dev, "Intializing F51.");

	retval = rmi_f51_alloc_memory(rmi4_data);
	if (retval < 0)
		goto error_exit;

	retval = rmi_f51_initialize(rmi4_data);
	if (retval < 0)
		goto error_exit;
	
#ifdef	F51_LED_USED
	retval = rmi_f51_initled(rmi4_data);
	if (retval < 0)
		goto error_exit;
#endif // #ifndef	F51_LED_USED	

	retval = rmi_f51_create_sysfs(rmi4_data);
	if (retval < 0)
		goto error_exit;

	return retval;

error_exit:
	rmi_f51_free_memory(rmi4_data);

	return retval;
}

static int rmi_f51_alloc_memory(struct synaptics_rmi4_data *rmi4_data)
{
	f51 = kzalloc(sizeof(struct rmi_fn_51_data), GFP_KERNEL);
	if (!f51) {
		dev_err(f51->dev, "Failed to allocate rmi_fn_51_data.\n");
		return -ENOMEM;
	}
	
	f51->fn_ptr = kzalloc(sizeof(*(f51->fn_ptr)), GFP_KERNEL);
	if (!f51->fn_ptr) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to alloc mem for fn_ptr\n",
				__func__);
		return -ENOMEM;
	}

	return 0;
}


static void rmi_f51_free_memory(struct synaptics_rmi4_data *rmi4_data)
{
	if (f51) {
		if(f51->attr_dir)
		{
			sysfs_remove_group(f51->attr_dir, &attrs_query);
			sysfs_remove_group(f51->attr_dir, &attrs_control);
			sysfs_remove_group(f51->attr_dir, &attrs_data);
		}
		kfree(f51->control.reg_0);
		kfree(f51->control.reg_1);
		kfree(f51->control.reg_2_3);
		kfree(f51->control.reg_4_5);
		kfree(f51->control.reg_6_7);
		kfree(f51->control.reg_8_9);
		kfree(f51->control.reg_10_11);
		kfree(f51->control.reg_12_13);
		kfree(f51->control.reg_14_15);
		kfree(f51->control.reg_16_17);
		kfree(f51->data.reg_2);
		kfree(f51->fn_ptr);
		kfree(f51);
		f51 = NULL;
	}
}


static int rmi_f51_initialize(struct synaptics_rmi4_data *rmi4_data)
{
	int retval = 0;
	bool hasF51 = false;
	u16 next_loc;
	unsigned short ii;
	unsigned char page;
	struct synaptics_rmi4_fn_desc rmi_fd;

	f51->rmi4_data = rmi4_data;
	f51->fn_ptr->read = rmi4_data->i2c_read;
	f51->fn_ptr->write = rmi4_data->i2c_write;
	f51->fn_ptr->enable = rmi4_data->irq_enable;
	f51->dev = &rmi4_data->i2c_client->dev;

	for (page = 0; page < PAGES_TO_SERVICE; page++) {
		for (ii = PDT_START; ii > PDT_END; ii -= PDT_ENTRY_SIZE) {
			ii |= (page << 8);

			retval = f51->fn_ptr->read(rmi4_data,
					ii,
					(unsigned char *)&rmi_fd,
					sizeof(rmi_fd));
			if (retval < 0)
				return -ENOMEM;

			if (!rmi_fd.fn_number)
				break;

			if (rmi_fd.fn_number == SYNAPTICS_RMI4_F51) {
				hasF51 = true;
				f51->query_base_addr =
					rmi_fd.query_base_addr | (page << 8);
				f51->control_base_addr =
					rmi_fd.ctrl_base_addr | (page << 8);
				f51->data_base_addr =
					rmi_fd.data_base_addr | (page << 8);
				f51->command_base_addr =
					rmi_fd.cmd_base_addr | (page << 8);
			} 
		}
	}
	
	if (!hasF51) {
		dev_err(f51->dev,
				"%s: F$54 is not available\n",
				__func__);
		return -ENOMEM;
	}	

	rmi4_data->f51_query_base_addr= f51->query_base_addr;
	rmi4_data->f51_ctrl_base_addr= f51->control_base_addr;
	rmi4_data->f51_data_base_addr= f51->data_base_addr ;
	rmi4_data->f51_cmd_base_addr = f51->command_base_addr ;

	/* Read F51 Query Data */
	f51->query.address = f51->query_base_addr;
	retval = f51->fn_ptr->read(rmi4_data, f51->query.address,
		(u8 *)&f51->query, sizeof(f51->query.data));
	if (retval < 0) {
		dev_err(f51->dev, "Could not read query registers from 0x%04x\n", f51->query.address);
		return retval;
	}

	/* Initialize Control Data */
	next_loc = f51->control_base_addr;

	f51->control.reg_0 =
			kzalloc(sizeof(union f51_control_0), GFP_KERNEL);
	if (!f51->control.reg_0) {
		dev_err(f51->dev, "Failed to allocate control registers.");
		return -ENOMEM;
	}
	f51->control.reg_0->address = next_loc;
	next_loc += sizeof(f51->control.reg_0->data);
	
	f51->control.reg_1=
			kzalloc(sizeof(union f51_control_1), GFP_KERNEL);
	if (!f51->control.reg_1) {
		dev_err(f51->dev, "Failed to allocate control registers.");
		return -ENOMEM;
	}
	f51->control.reg_1->address = next_loc;
	next_loc += sizeof(f51->control.reg_1->data);
	
	f51->control.reg_2_3=
			kzalloc(sizeof(union f51_control_2_3), GFP_KERNEL);
	if (!f51->control.reg_2_3) {
		dev_err(f51->dev, "Failed to allocate control registers.");
		return -ENOMEM;
	}
	f51->control.reg_2_3->address = next_loc;
	next_loc += sizeof(f51->control.reg_2_3->data);
	
	f51->control.reg_4_5=
			kzalloc(sizeof(union f51_control_4_5), GFP_KERNEL);
	if (!f51->control.reg_4_5) {
		dev_err(f51->dev, "Failed to allocate control registers.");
		return -ENOMEM;
	}
	f51->control.reg_4_5->address = next_loc;
	next_loc += sizeof(f51->control.reg_4_5->data);
	
	f51->control.reg_6_7=
			kzalloc(sizeof(union f51_control_6_7), GFP_KERNEL);
	if (!f51->control.reg_6_7) {
		dev_err(f51->dev, "Failed to allocate control registers.");
		return -ENOMEM;
	}
	f51->control.reg_6_7->address = next_loc;
	next_loc += sizeof(f51->control.reg_6_7->data);
	
	f51->control.reg_8_9=
			kzalloc(sizeof(union f51_control_8_9), GFP_KERNEL);
	if (!f51->control.reg_8_9) {
		dev_err(f51->dev, "Failed to allocate control registers.");
		return -ENOMEM;
	}
	f51->control.reg_8_9->address = next_loc;
	next_loc += sizeof(f51->control.reg_8_9->data);
	
	f51->control.reg_10_11=
			kzalloc(sizeof(union f51_control_10_11), GFP_KERNEL);
	if (!f51->control.reg_10_11) {
		dev_err(f51->dev, "Failed to allocate control registers.");
		return -ENOMEM;
	}
	f51->control.reg_10_11->address = next_loc;
	next_loc += sizeof(f51->control.reg_10_11->data);
	
	f51->control.reg_12_13=
			kzalloc(sizeof(union f51_control_12_13), GFP_KERNEL);
	if (!f51->control.reg_12_13) {
		dev_err(f51->dev, "Failed to allocate control registers.");
		return -ENOMEM;
	}
	f51->control.reg_12_13->address = next_loc;
	next_loc += sizeof(f51->control.reg_12_13->data);
	
	f51->control.reg_14_15=
			kzalloc(sizeof(union f51_control_14_15), GFP_KERNEL);
	if (!f51->control.reg_14_15) {
		dev_err(f51->dev, "Failed to allocate control registers.");
		return -ENOMEM;
	}
	f51->control.reg_14_15->address = next_loc;
	next_loc += sizeof(f51->control.reg_14_15->data);
	
	f51->control.reg_16_17=
			kzalloc(sizeof(union f51_control_16_17), GFP_KERNEL);
	if (!f51->control.reg_16_17) {
		dev_err(f51->dev, "Failed to allocate control registers.");
		return -ENOMEM;
	}
	f51->control.reg_16_17->address = next_loc;
	next_loc += sizeof(f51->control.reg_16_17->data);
	
	mutex_init(&f51->control_mutex);

	
	/* initialize data registers */
	next_loc = f51->data_base_addr;
	f51->data.reg_2 =
			kzalloc(sizeof(union f51_data_2), GFP_KERNEL);
	if (!f51->data.reg_2) {
		dev_err(f51->dev, "Failed to allocate data registers.");
		return -ENOMEM;
	}
	f51->data.reg_2->address = next_loc;
	next_loc += sizeof(f51->data.reg_2->data);
	
	mutex_init(&f51->data_mutex);
	
	return 0;
}


#ifdef	F51_LED_USED

#define PWM_SHIFT	6
static int rmi_led_brightness_setcurrent(struct led_classdev *led_cdev,
		 enum led_brightness value)
{
	int ret = 0;
	struct rmi_fn_51_data *f51 =
		 container_of(led_cdev, struct rmi_fn_51_data, led.led_cdev);

	if( !f51 )
		return -ENOMEM;
		
	mutex_lock(&f51->control_mutex);	
	
	
	mutex_unlock(&f51->control_mutex);	
	
	return ret;
}

static int rmi_led_brightness_setslope(struct led_classdev *led_cdev,
		 enum led_brightness value)
{
	int ret = 0;
	struct rmi_fn_51_data *f51 =
		 container_of(led_cdev, struct rmi_fn_51_data, led.led_cdev);
	
	if( !f51 )
		return -ENOMEM;
	
	return ret;
}

static int rmi_led_brightness_pwm(struct led_classdev *led_cdev,
		 enum led_brightness value)
{
	int ret = 0;
	//struct rmi_fn_51_data *f51 =
	//	 container_of(led_cdev, struct rmi_fn_51_data, led.led_cdev);

	ret = rmi_led_brightness_setcurrent(led_cdev,led_cdev->brightness);
		
	return ret;
}

#ifdef CONFIG_MFD_MX_HUB
extern int mx_hub_setbrightness(u8 data);
extern int mx_hub_setslope(u8 data);
extern int check_mxhub(void );
#endif

#define	GET_MODE(x)	((x>>8)&0x0F)
static void rmi_led_brightness_set(struct led_classdev *led_cdev,
		 enum led_brightness value)
{
	int ret = 0;
	int mode;
	int data;

	dev_info(led_cdev->dev, "value = 0x%.3X \n",value);

	mode = GET_MODE(value);
	data = value & 0xFF; 

	switch( mode )
	{
	case 0:
		ret = rmi_led_brightness_setcurrent(led_cdev,data);
#ifdef CONFIG_MFD_MX_HUB
		if( check_mxhub() )
			ret = mx_hub_setbrightness(data);
#endif
		break;

	case 1:
		ret = rmi_led_brightness_setslope(led_cdev,data);
#ifdef CONFIG_MFD_MX_HUB
		if( check_mxhub() )
			ret = mx_hub_setslope(data);
#endif
		break;
		
	case 2:
		ret = rmi_led_brightness_pwm(led_cdev,data);
		break;		
		
	default:		
		dev_err(led_cdev->dev, "mode  %d is valite \n",mode);
		ret = -EINVAL;			
		break;
	}

	if(ret < 0)
		dev_err(led_cdev->dev, "brightness set failed ret = %d \n",ret);
}
 

static int rmi_f51_initled(struct synaptics_rmi4_data *rmi4_data)
{
	struct led_classdev	* cdev;
	int ret;

	cdev = &f51->led.led_cdev;

	cdev->name = "mx-led";
	cdev->brightness = 127;
	cdev->max_brightness= 0xFFF;

	cdev->brightness_set = rmi_led_brightness_set;

#ifdef CONFIG_LEDS_CLASS
	ret = led_classdev_register(f51->dev, cdev);
	if (ret) {
		dev_err(f51->dev, "failed to register LED\n");
	}
#endif

	mutex_lock(&f51->control_mutex);	

	
	mutex_unlock(&f51->control_mutex);	

	f51->led.pwm = (1<<PWM_SHIFT);
	f51->led.slope= 0;

	rmi_led_brightness_set(cdev,cdev->brightness);
	
	return ret;
}
#endif //F51_LED_USED

static int rmi_f51_create_sysfs(struct synaptics_rmi4_data *rmi4_data0)
{
	struct synaptics_rmi4_data *rmi4_data = f51->rmi4_data;
	
	dev_dbg(f51->dev, "Creating sysfs files.");
	
	f51->attr_dir = kobject_create_and_add("f51",
			&rmi4_data->input_dev->dev.kobj);
	if (!f51->attr_dir) {
		dev_err(f51->dev,
			"%s: Failed to create sysfs directory\n",
			__func__);
		return -ENODEV;
	}
	
	/* Set up sysfs device attributes. */
	if (sysfs_create_group(f51->attr_dir, &attrs_query) < 0) {
		dev_err(f51->dev, "Failed to create query sysfs files.");
		return -ENODEV;
	}
	if (sysfs_create_group(f51->attr_dir, &attrs_control) < 0) {
		dev_err(f51->dev, "Failed to create control sysfs files.");
		return -ENODEV;
	}
	if (sysfs_create_group(f51->attr_dir, &attrs_data) < 0) {
		dev_err(f51->dev, "Failed to create data sysfs files.");
		return -ENODEV;
	}
	return 0;
}


static void rmi_f51_remove(struct synaptics_rmi4_data *rmi4_data)
{
#ifndef	CONFIG_LEDS_MXHUB
#ifdef CONFIG_LEDS_CLASS
	led_classdev_unregister(&f51->led.led_cdev);
#endif
#endif //	CONFIG_LEDS_MXHUB
	
	dev_info(f51->dev, "Removing F51.");
	rmi_f51_free_memory(rmi4_data);
}

/* sysfs functions */
/* Query */
simple_show_func_unsigned(query, query_data)

/* Control */
show_store_func_unsigned(control, reg_0,en_single_tap_int)
show_store_func_unsigned(control, reg_0,en_double_tap_int)
show_store_func_unsigned(control, reg_0,en_0d_touched_int)
show_store_func_unsigned(control, reg_0,en_0d_released_int)
show_store_func_unsigned(control, reg_0,anti_err_active)
show_store_func_unsigned(control, reg_0,wake_up_active)
show_store_func_unsigned(control, reg_0,partitioned_mode)
show_store_func_unsigned(control, reg_1,key_ttap)

show_store_func_unsigned(control, reg_2_3,wake_up_left)
show_store_func_unsigned(control, reg_4_5,wake_up_top)
show_store_func_unsigned(control, reg_6_7,wake_up_right)
show_store_func_unsigned(control, reg_8_9,wake_up_bottom)
show_store_func_unsigned(control, reg_10_11,anti_err_left)
show_store_func_unsigned(control, reg_12_13,anti_err_top)
show_store_func_unsigned(control, reg_14_15,anti_err_right)
show_store_func_unsigned(control, reg_16_17,anti_err_bottom)

/* Data */
show_store_func_unsigned(data, reg_2, tap_detected)
show_store_func_unsigned(data, reg_2, double_tap_detected)
show_store_func_unsigned(data, reg_2, d0_touched)
show_store_func_unsigned(data, reg_2, d0_released)
show_store_func_unsigned(data, reg_2, anti_err_touched)

#ifdef CONFIG_PM
static int rmi_f51_resume(struct device *dev)
{
	//struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);
	int retval = 0;

	dev_dbg(f51->dev, "Resume...\n");
	
	return retval;
}

static int rmi_f51_suspund(struct device *dev)
{
	//struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);
	int retval = 0;

	dev_dbg(f51->dev, "Suspend...\n");
	
	return retval;
}

static const struct dev_pm_ops rmi_f51_pm_ops = {
	.suspend = rmi_f51_suspund,
	.resume = rmi_f51_resume,
};
#endif 


static int __init rmi4_f51_module_init(void)
{
	synaptics_rmi4_new_function(RMI_F51, true,
			rmi_f51_init,
			rmi_f51_remove,
			NULL);

	return 0;
}

static void __exit rmi4_f51_module_exit(void)
{
	init_completion(&remove_complete);
	synaptics_rmi4_new_function(RMI_F51, false,
			rmi_f51_init,
			rmi_f51_remove,
			NULL);

	wait_for_completion(&remove_complete);
	return;
}

module_init(rmi4_f51_module_init);
module_exit(rmi4_f51_module_exit);

MODULE_AUTHOR("Chris Chen <Chwei@meizu.com>");
MODULE_DESCRIPTION("RMI F51 module");
MODULE_LICENSE("GPL");
MODULE_VERSION(RMI_DRIVER_VERSION);

