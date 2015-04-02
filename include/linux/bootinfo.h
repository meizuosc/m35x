#ifndef __MEIZU_HARDWARE_BOOTINFO_H__
#define __MEIZU_HARDWARE_BOOTINFO_H__

#include <linux/types.h>

u32 get_bootinfo_version(void);
char * get_bootinfo_uboot_version(void);
char *get_boot_info_build_variant(void);
u8 get_bootinfo_board_version(void);

enum {
	PART_KERNEL = 0,
	PART_RAMDISK,
	PART_RECOVERY,
	PART_LOGO,
	PART_PARAM,
	PART_PRIVATE,
	PART_BAT_MODEL,
	/* recovery private data partition */
	PART_REC_PRIV,
	PART_RESERVED1,
	PART_RESERVED2,
	PART_MAX,
};

#define RECOVERY_PRIV_MAGIC_LEN		(8)
#define RECOVERY_PRIV_MAGIC_NUM		"OnTheGo!"
struct part_info {
	u32 start_sec;
	u32 sec_count;
};

struct bootinfo {
	u32		info_version;			// 1/2/3/4/..
	struct		part_info part[32];		// partition layout, null terminal
	char		uboot_version[64];		// uboot string version
	char		build_variant[16];		// eng/user/oversea/..
	u8		not_signed_check;		// signed check, 0 means check the signature
	u8		board_version;			// defined by ID1-3 GPIO pin
};

struct bootinfo *get_bootinfo(void);

#endif
