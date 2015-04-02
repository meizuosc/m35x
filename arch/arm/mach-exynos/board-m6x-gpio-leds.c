#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/leds.h>
#include <linux/err.h>

#include <mach/map.h>
#include <mach/gpio-meizu.h>

#ifdef CONFIG_LEDS_GPIO
static struct gpio_led __initdata gpio_leds[] = {
	[0] = {
		.name = "gpio-led0",
		.default_trigger = "cpuidle_aftr",
		.gpio = MEIZU_LED_ID1,
		.active_low = true,
		.retain_state_suspended = false,
		.default_state = LEDS_GPIO_DEFSTATE_OFF,
	},
	[1] = {
		.name = "gpio-led1",
		.default_trigger = "exynos5410-idle1",
		.gpio = MEIZU_LED_ID2,
		.active_low = true,
		.retain_state_suspended = true,
		.default_state = LEDS_GPIO_DEFSTATE_OFF,
	},
	[2] = {
		.name = "gpio-led2",
		.default_trigger = "exynos5410-idle2",
		.gpio = MEIZU_LED_ID3,
		.active_low = true,
		.retain_state_suspended = false,
		.default_state = LEDS_GPIO_DEFSTATE_OFF,
	},
	[3] = {
		.name = "gpio-led3",
		.default_trigger = "cpu0-idle",
		.gpio = MEIZU_LED_ID4,
		.active_low = true,
		.retain_state_suspended = false,
		.default_state = LEDS_GPIO_DEFSTATE_OFF,
	},
	[4] = {
		.name = "gpio-led4",
		.default_trigger = "exynos5410-idle3",
		.gpio = MEIZU_LED_ID5,
		.active_low = true,
		.retain_state_suspended = false,
		.default_state = LEDS_GPIO_DEFSTATE_OFF,
	},
	[5] = {
		.name = "gpio-led5",
		.default_trigger = "cpuidle_lpa",
		.gpio = MEIZU_LED_ID6,
		.active_low = true,
		.retain_state_suspended = false,
		.default_state = LEDS_GPIO_DEFSTATE_OFF,
	},
};

static struct gpio_led_platform_data __initdata gpio_led = {
	.num_leds = ARRAY_SIZE(gpio_leds),
	.leds = gpio_leds,
};


static int  __init gpio_init_led(void)
{
	struct platform_device *ret = NULL;

	ret = gpio_led_register_device(-1, &gpio_led);

	return PTR_ERR(ret);
}

arch_initcall(gpio_init_led);
#endif
