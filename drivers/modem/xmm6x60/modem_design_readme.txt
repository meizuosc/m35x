Modem driver说明：

1. modem在arch/arm/mach-exynos/目录下的board-*-modems.c定义gpio口和device，
m03x和m040分别对应board-m03x-modems.c和board-m040-modems.c两个文件

2. modem驱动目录为driver/modem目录:
modem driver从meizu_modem.c开始初始化,
xmm6260的控制接口在meizu_modemctl_xmm6260.c中定义
hsic的操作在meizu_modem_hsic.c文件中定义
提供给上层tty接口在meizu_modem_io_device.c中定义

modem driver结构：
board-*-modems.c ==> meizu_modem.c ==> meizu_modemctl_xmm6260.c ==>
meizu_modem_hsic.c ==> meizu_modem_io_device.c

TODO:
1.稳定性测试；2.优化几个重要数据结构的关系;3.优化memory pool, rx_work,rx_queue etc.

