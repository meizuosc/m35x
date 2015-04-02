/*
 * Copyright (C) 2012 Meizu Technology Co.Ltd, Zhuhai, China
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
 * Modified for Crespo on August, 2010 By Samsung Electronics Co.
 * This is modified operate according to each status.
 *
 */

#include <linux/module.h>
#include <linux/random.h>
#include <linux/rsa.h>
#include <linux/rsa_pubkey.h>
#include <linux/slab.h>
#include <linux/bootinfo.h>
#include <asm/uaccess.h>

extern int deal_private_block(int write, unsigned offset , long len, void *buffer, int area);
extern struct bootinfo basic_bootinfo;

#define BASE_PRIVATE_ENTRY_OFFSET  (basic_bootinfo.part[PART_PRIVATE].start_sec*512)
#define MAX_PRIVATE_ENTRY (1024)

#define PRIVATE_ENTRY_BLOCK_SIZE (1024)
#define PRIVATE_ENTRY_SIG_SIZE (256)
#define CAMERA_MODULE_SLOT (4)

static char private_entry_buf[PRIVATE_ENTRY_BLOCK_SIZE];

static inline int slot_to_offset(int slot)
{
//	pr_info("%s(slot = %d) called\n", __func__,slot);

	if(slot >= MAX_PRIVATE_ENTRY || slot < 0) {
		pr_info("out of private entry range\n");
		return -1;
	}
	return BASE_PRIVATE_ENTRY_OFFSET + slot * PRIVATE_ENTRY_BLOCK_SIZE;
}

static char device_sn[256];
static char device_mac[6];
static char err_mac[6] = {0x0,0x1,0x2,0x3,0x4,0x5};

extern int meizu_set_sn(char *sn, int size);

static void init_device_sn(void)
{
	int offset = slot_to_offset(0);//sn slot 0
	uint16_t len;
	char* data = private_entry_buf + PRIVATE_ENTRY_SIG_SIZE + sizeof(len);

//	pr_info("%s called read slot 0\n", __func__);
	deal_private_block(0, offset, PRIVATE_ENTRY_BLOCK_SIZE, private_entry_buf,PART_PRIVATE);
	len = *(uint16_t *)(private_entry_buf + PRIVATE_ENTRY_SIG_SIZE);

	if(!rsa_with_sha1_verify(data, len, &factory_rsa_pk, private_entry_buf)) {

		memcpy(device_sn, data, len);

		meizu_set_sn(device_sn, sizeof(device_sn));
		pr_info("@@@@ SN %s\n", device_sn);
	} else {
		pr_info("@@@@ SN rsa with sha1 verify failed!!!!\n");
	}
}

static void init_device_mac(void)
{
	int offset = slot_to_offset(1);//mac slot 1
	uint16_t len;
	char* data = private_entry_buf + PRIVATE_ENTRY_SIG_SIZE + sizeof(len);

//	pr_info("%s called read slot 1\n", __func__);
	memcpy(device_mac, err_mac, sizeof(device_mac));

	deal_private_block(0, offset, PRIVATE_ENTRY_BLOCK_SIZE, private_entry_buf,PART_PRIVATE);
	len = 6;

	if(!rsa_with_sha1_verify(data, len, &factory_rsa_pk, private_entry_buf)) {

		memcpy(device_mac, data, len);

		pr_info("@@@@ MAC %02X:%02X:%02X:%02X:%02X:%02X\n",
				device_mac[0], device_mac[1], device_mac[2],
				device_mac[3], device_mac[4], device_mac[5]);
	} else {
		pr_info("@@@@ MAC rsa with sha1 verify failed!!!!\n");
	}
}

/*
	@len the length of write buffer, less than 1024 bytes
	@buf the write buffer
	@return 0 OK, else fail
*/
int write_battery_model(int len, void *buf)
{
	int err = 0;
	int offset = basic_bootinfo.part[PART_BAT_MODEL].start_sec*512;

	if(len > 1024){
		pr_info("write battery model fail, buffer length should be less than 1024 bytes\n");
		err = -1;
		goto out;
	}

	err = deal_private_block(1, offset, len, buf,PART_BAT_MODEL);
	if (err)
		goto out;

out:
	pr_info("%s rtn code %d\n", __func__, err);
	return err;
}
/*
	@return 0 OK, else fail
*/
int read_battery_model(int len, void *buf)
{
	void* slot_buf=NULL;
	int err = 0;
	int offset = basic_bootinfo.part[PART_BAT_MODEL].start_sec*512;

	if(len > 1024){
		pr_info("read battery model fail, buffer length should be less than 1024 bytes\n");
		err = -1;
		goto out;
	}

	slot_buf = kmalloc(PRIVATE_ENTRY_BLOCK_SIZE, GFP_KERNEL);
	if(slot_buf == NULL){
		pr_err("read battery model fail, failed to malloc");
		err = -2;
		goto out;
	}

	err = deal_private_block(0, offset, PRIVATE_ENTRY_BLOCK_SIZE, slot_buf,PART_BAT_MODEL);
	if (err)
		goto out;

	memcpy(buf,slot_buf,len);
	kfree(slot_buf);

out:
	pr_info("%s rtn code %d\n", __func__, err);
	return err;
}

int manipulate_camera_module(int len, void *buf, bool write)
{
	int ret = 0;
	int offset = slot_to_offset(CAMERA_MODULE_SLOT);
	int direction = write ? 1 : 0;

	ret = deal_private_block(direction, offset, len, buf, PART_PRIVATE);
	return ret;
}

int get_mac_form_device(unsigned char *buf)
{

	if(memcmp(device_mac, err_mac, sizeof(err_mac))) {
		memcpy(buf, device_mac, sizeof(device_mac));
		buf[0] = 0x38;
		buf[1] = 0xBC;
		buf[2] = 0x1A;
		return 0;
	}

	pr_info("%s failed\n", __func__);
	return -EINVAL;
}

int meizu_device_info_init(void)
{
	//char *src_buf = "123456789";
	//char   dest_buf[10];
	pr_info("%s called\n", __func__);
	init_device_sn();
	init_device_mac();
	//write_battery_model(10,src_buf);
	//read_battery_model(10,dest_buf);
	//pr_info("%s %s\n", __func__, dest_buf);
	return 0;
}
