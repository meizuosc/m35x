#include <linux/fs.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/bootinfo.h>

extern struct bootinfo basic_bootinfo;

static int uboot_version_proc_show(struct seq_file *seq, void *v)
{
	seq_printf(seq, "%s\n", basic_bootinfo.build_variant);
	return 0;
}

static int uboot_version_proc_open(struct inode *inode, struct file *fp)
{
	return single_open(fp, uboot_version_proc_show, NULL);
}

static const struct file_operations uboot_version_proc_fops = {
	.open = uboot_version_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int __init proc_uboot_version_init(void)
{
	/* Info about Eng\User\Oversea... */
	proc_create("uboot_version", 0, NULL, &uboot_version_proc_fops);
	return 0;
}

module_init(proc_uboot_version_init);
