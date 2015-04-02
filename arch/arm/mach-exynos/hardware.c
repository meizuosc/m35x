/* arch/arm/mach-exynos/hardware.c
 *
 * Meizu Mobilephone Hardware information support
 *
 * Author : Li Tao <litao@meizu.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/


#include <linux/fs.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/bootinfo.h>

#include <asm/mach-types.h>

#include <mach/hardware.h>


unsigned int meizu_board_mdm_type(void)
{
	unsigned int mdm_type = MEIZU_MDM_XMM6260;

	if (machine_is_m69())
		mdm_type = MEIZU_MDM_SC8803G;

	return mdm_type;
}

/*  ID   PCB   NFC
 *  000  V1.0  T
 *  001  V1.0  F
 *  010  V2.0  T
 *  011  V2.0  F
 */
bool meizu_board_have_nfc(void)
{
	bool have_nfc = false;
	u8 board_version = m6x_get_board_version();

	/*bit0: [0] NFC-> true, [1] NFC-> false*/
	if (!(board_version & 0x1))
		have_nfc = true;

	return have_nfc;
}

static int c_show(struct seq_file *m, void *v)
{
	u8 board_version = m6x_get_board_version();

	seq_printf(m, "SOC\t\t: %s\n", "EXYNOS5410");
	seq_printf(m, "Version\t\t: %d\n", board_version);
	seq_printf(m, "NFC\t\t: %s\n", meizu_board_have_nfc()? "true":"false");
	seq_printf(m, "Modem\t\t: %s\n", (meizu_board_mdm_type() == MEIZU_MDM_XMM6260)? "WCDMA":"TD-SCDMA");

	return 0;
}

/*proc functions*/
static void *c_start(struct seq_file *m, loff_t *pos)
{
	return *pos < 1 ? (void *)1 : NULL;
}

static void *c_next(struct seq_file *m, void *v, loff_t *pos)
{
	++*pos;
	return NULL;
}

static void c_stop(struct seq_file *m, void *v)
{
}

static const struct seq_operations hwinfo_op = {
	.start	= c_start,
	.next	= c_next,
	.stop	= c_stop,
	.show	= c_show
};

static int hwinfo_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &hwinfo_op);
}

static const struct file_operations proc_hwinfo_operations = {
	.open		= hwinfo_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= seq_release,
};

static int __init proc_hwinfo_init(void)
{
	proc_create("hwinfo", 0, NULL, &proc_hwinfo_operations);
	return 0;
}

module_init(proc_hwinfo_init);

MODULE_DESCRIPTION("Meizu Mobilephone Hardware Information");
MODULE_AUTHOR("Li Tao <litao@meizu.com>");
MODULE_LICENSE("GPL");
