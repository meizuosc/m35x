/*  
 * include/linux/meizu-sys.h
 * 
 * meizu technology particular classes
 *
 * Author : Li Tao <litao@meizu.com>
 */

#ifndef __H_MEIZU_PARTICULAR_CLASS_H__
#define __H_MEIZU_PARTICULAR_CLASS_H__

#include <linux/types.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/err.h>

extern struct class *meizu_class;
extern struct class *sensors_class;

#if 0
extern int sensors_register(struct device *dev, void *drvdata,
        struct device_attribute *attributes[], char *name);
extern void sensors_unregister(struct device *dev,
        struct device_attribute *attributes[]);
#endif

#endif /*__H_MEIZU_PARTICULAR_CLASS_H__*/
