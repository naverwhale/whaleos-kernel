// SPDX-License-Identifier: GPL-2.0-only
/*
 * LED Disk Activity Trigger
 *
 * Copyright 2006 Openedhand Ltd.
 *
 * Author: Richard Purdie <rpurdie@openedhand.com>
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/leds.h>
#ifdef CONFIG_LEDS_TRIGGER_SDHCI
#include <linux/io.h>
#include <linux/dmi.h>
#endif

#define BLINK_DELAY 30

DEFINE_LED_TRIGGER(ledtrig_disk);
DEFINE_LED_TRIGGER(ledtrig_disk_read);
DEFINE_LED_TRIGGER(ledtrig_disk_write);
DEFINE_LED_TRIGGER(ledtrig_ide);
#ifdef CONFIG_LEDS_TRIGGER_SDHCI
DEFINE_LED_TRIGGER(ledtrig_sdhci);
#endif

void ledtrig_disk_activity(bool write)
{
	unsigned long blink_delay = BLINK_DELAY;

	led_trigger_blink_oneshot(ledtrig_disk,
				  &blink_delay, &blink_delay, 0);
	led_trigger_blink_oneshot(ledtrig_ide,
				  &blink_delay, &blink_delay, 0);
	if (write)
		led_trigger_blink_oneshot(ledtrig_disk_write,
					  &blink_delay, &blink_delay, 0);
	else
		led_trigger_blink_oneshot(ledtrig_disk_read,
					  &blink_delay, &blink_delay, 0);
}
EXPORT_SYMBOL(ledtrig_disk_activity);

#ifdef CONFIG_LEDS_TRIGGER_SDHCI
bool ledtrig_bypass;
unsigned long reg1, reg2;
void __iomem *ledtrig_phy_addr;

void ledtrig_sdhci_activity(bool onoff)
{
	if (ledtrig_phy_addr == 0) {
		const char *dmi_info;
		dmi_info = dmi_get_system_info(DMI_PRODUCT_NAME);
		do {
			if (strstr(dmi_info, "14T")) {
				if (strstr(dmi_info, "30Q")) {
					ledtrig_phy_addr = ioremap(0xFD6E0B20, PAGE_SIZE);
					reg1 = 0x01800062;
					reg2 = 0x84000200;
					break;
				}
				if (strstr(dmi_info, "30S")) {
					ledtrig_phy_addr = ioremap(0xFD6D0A50, PAGE_SIZE);
					reg1 = 0x41;
					reg2 = 0x84000200;
					break;
				}
			}
			ledtrig_phy_addr = (void __iomem*)1;
			ledtrig_bypass = true;
		} while(false);
	}
	if (!ledtrig_bypass) {
		iowrite32(reg1, ledtrig_phy_addr + 4);
		if (onoff)
			iowrite32(reg2, ledtrig_phy_addr);
		else
			iowrite32(reg2 | 1, ledtrig_phy_addr);
	}

	led_trigger_direct(ledtrig_sdhci,
		onoff == true ? 1 : LED_OFF);
}
EXPORT_SYMBOL(ledtrig_sdhci_activity);
#endif

static int __init ledtrig_disk_init(void)
{
	led_trigger_register_simple("disk-activity", &ledtrig_disk);
	led_trigger_register_simple("disk-read", &ledtrig_disk_read);
	led_trigger_register_simple("disk-write", &ledtrig_disk_write);
	led_trigger_register_simple("ide-disk", &ledtrig_ide);
#ifdef CONFIG_LEDS_TRIGGER_SDHCI
	led_trigger_register_simple("sdhci-activity", &ledtrig_sdhci);
#endif
	return 0;
}
device_initcall(ledtrig_disk_init);
