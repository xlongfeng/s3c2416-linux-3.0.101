/* linux/arch/arm/mach-s3c2416/mach-hanlin_v3c.c
 *
 * Copyright (c) 2009 Yauhen Kharuzhy <jekhor@gmail.com>,
 *	as part of OpenInkpot project
 * Copyright (c) 2009 Promwad Innovation Company
 *	Yauhen Kharuzhy <yauhen.kharuzhy@promwad.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
*/

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/timer.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/serial_core.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/mtd/partitions.h>
#include <linux/gpio.h>
#include <linux/gpio_keys.h>
#include <linux/fb.h>
#include <linux/delay.h>
#include <linux/smsc911x.h>
#include <linux/pwm_backlight.h>
#include <linux/pwm.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>

#include <mach/hardware.h>
#include <asm/irq.h>
#include <asm/mach-types.h>

#include <plat/regs-serial.h>
#include <mach/regs-gpio.h>
#include <mach/regs-lcd.h>
#include <mach/regs-s3c2443-clock.h>

#include <mach/idle.h>
#include <mach/leds-gpio.h>
#include <plat/iic.h>

#include <plat/s3c2416.h>
#include <plat/gpio-cfg.h>
#include <plat/clock.h>
#include <plat/devs.h>
#include <plat/cpu.h>
#include <plat/nand.h>
#include <plat/sdhci.h>
#include <plat/udc.h>

#include <plat/regs-fb-v4.h>
#include <plat/fb.h>

#include <plat/common-smdk.h>

static struct map_desc smdk2416_iodesc[] __initdata = {
	/* ISA IO Space map (memory space selected by A24) */

	{
		.virtual	= (u32)S3C24XX_VA_ISA_WORD,
		.pfn		= __phys_to_pfn(S3C2410_CS2),
		.length		= 0x10000,
		.type		= MT_DEVICE,
	}, {
		.virtual	= (u32)S3C24XX_VA_ISA_WORD + 0x10000,
		.pfn		= __phys_to_pfn(S3C2410_CS2 + (1<<24)),
		.length		= SZ_4M,
		.type		= MT_DEVICE,
	}, {
		.virtual	= (u32)S3C24XX_VA_ISA_BYTE,
		.pfn		= __phys_to_pfn(S3C2410_CS2),
		.length		= 0x10000,
		.type		= MT_DEVICE,
	}, {
		.virtual	= (u32)S3C24XX_VA_ISA_BYTE + 0x10000,
		.pfn		= __phys_to_pfn(S3C2410_CS2 + (1<<24)),
		.length		= SZ_4M,
		.type		= MT_DEVICE,
	}
};

#define UCON (S3C2410_UCON_DEFAULT	| \
		S3C2440_UCON_PCLK	| \
		S3C2443_UCON_RXERR_IRQEN)

#define ULCON (S3C2410_LCON_CS8 | S3C2410_LCON_PNONE)

#define UFCON (S3C2410_UFCON_RXTRIG8	| \
		S3C2410_UFCON_FIFOMODE	| \
		S3C2440_UFCON_TXTRIG16)

static struct s3c2410_uartcfg smdk2416_uartcfgs[] __initdata = {
	[0] = {
		.hwport	     = 0,
		.flags	     = 0,
		.ucon	     = UCON,
		.ulcon	     = ULCON,
		.ufcon	     = UFCON,
	},
	[1] = {
		.hwport	     = 1,
		.flags	     = 0,
		.ucon	     = UCON,
		.ulcon	     = ULCON,
		.ufcon	     = UFCON,
	},
	/* IR port */
	[2] = {
		.hwport	     = 2,
		.flags	     = 0,
		.ucon	     = UCON,
		.ulcon	     = ULCON | 0x50,
		.ufcon	     = UFCON,
	},
	[3] = {
		.hwport	     = 3,
		.flags	     = 0,
		.ucon	     = UCON,
		.ulcon	     = ULCON,
		.ufcon	     = UFCON,
	}
};

void smdk2416_hsudc_gpio_init(void)
{
	s3c2410_modify_misccr(S3C2416_MISCCR_SEL_SUSPND, 0);
}

void smdk2416_hsudc_gpio_uninit(void)
{
	s3c2410_modify_misccr(S3C2416_MISCCR_SEL_SUSPND, 1);
}

struct s3c24xx_hsudc_platdata smdk2416_hsudc_platdata = {
	.epnum = 9,
	.gpio_init = smdk2416_hsudc_gpio_init,
	.gpio_uninit = smdk2416_hsudc_gpio_uninit,
};

struct s3c_fb_pd_win smdk2416_fb_win[] = {
	[0] = {
		/* think this is the same as the smdk6410 */
		.win_mode	= {
			.pixclock	= 79190,
			.left_margin	= 8,
			.right_margin	= 13,
			.upper_margin	= 7,
			.lower_margin	= 5,
			.hsync_len	= 3,
			.vsync_len	= 1,
			.xres           = 800,
			.yres           = 600,
			.refresh	= 25,
		},
		.default_bpp	= 16,
		.max_bpp	= 32,
	},
};

static void s3c2416_fb_gpio_setup_24bpp(void)
{
	unsigned int gpio;

	for (gpio = S3C2410_GPC(1); gpio <= S3C2410_GPC(4); gpio++) {
		s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(2));
		s3c_gpio_setpull(gpio, S3C_GPIO_PULL_NONE);
	}

	for (gpio = S3C2410_GPC(8); gpio <= S3C2410_GPC(15); gpio++) {
		s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(2));
		s3c_gpio_setpull(gpio, S3C_GPIO_PULL_NONE);
	}

	for (gpio = S3C2410_GPD(0); gpio <= S3C2410_GPD(15); gpio++) {
		s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(2));
		s3c_gpio_setpull(gpio, S3C_GPIO_PULL_NONE);
	}
}

static struct s3c_fb_platdata smdk2416_fb_platdata = {
	.win[0]		= &smdk2416_fb_win[0],
	.setup_gpio	= s3c2416_fb_gpio_setup_24bpp,
	.vidcon0	= VIDCON0_VIDOUT_RGB | VIDCON0_PNRMODE_RGB,
};

static void s3c_bl_power(int enable)
{
	static int enabled;
	if (enabled == enable)
		return;
	if (!enable) {
			gpio_direction_output(S3C2410_GPA(12), 0);
			gpio_direction_output(S3C2410_GPB(0), 0);
	} else {
			/* LED driver need a "push" to power on */
			gpio_direction_output(S3C2410_GPB(0), 0);
			/* Warm up backlight for one period of PWM.
			 * Without this trick its almost impossible to
			 * enable backlight with low brightness value
			 */
			ndelay(48000);
			s3c_gpio_cfgpin(S3C2410_GPB(0), S3C2410_GPB0_TOUT0);
			gpio_direction_output(S3C2410_GPA(12), 1);
	}
	enabled = enable;
}

static int s3c_backlight_init(struct device *dev)
{
	WARN_ON(gpio_request(S3C2410_GPA(12), "Backlight"));
	WARN_ON(gpio_request(S3C2410_GPB(0), "Backlight"));

	s3c_bl_power(1);

	return 0;
}

static void s3c_backlight_exit(struct device *dev)
{
	s3c_bl_power(0);

	gpio_free(S3C2410_GPB(0));
	gpio_free(S3C2410_GPA(12));
}

static int s3c_backlight_notify(struct device *dev, int brightness)
{
	if (!brightness) {
		s3c_bl_power(0);
	} else {
		s3c_bl_power(1);
	}
	return brightness;
}

static struct platform_pwm_backlight_data s3c_backlight_data = {
	.pwm_id = 0,
	.max_brightness = 24,
	.dft_brightness = 4,
	.pwm_period_ns = 48000,
	.init = s3c_backlight_init,
	.notify = s3c_backlight_notify,
	.exit = s3c_backlight_exit,
};

static struct platform_device s3c_device_backlight = {
	.name = "pwm-backlight",
	.dev = {
		.parent = &s3c_device_timer[0].dev,
		.platform_data = &s3c_backlight_data,
	},
};

static struct gpio_keys_button s3c_gpio_keys_table[] = {
	{
		.code		= KEY_POWER,
		.gpio		= S3C2410_GPF(3),
		.active_low	= 1,
		.desc		= "Power button",
		.wakeup		= 1,
	},
	{
		.code		= KEY_F1,
		.gpio		= S3C2410_GPG(2),
		.active_low	= 1,
		.desc		= "Alarm Mute button",
	},
	{
		.code		= KEY_F2,
		.gpio		= S3C2410_GPG(3),
		.active_low	= 1,
		.desc		= "Alarm Pause button",
	},
	{
		.code		= KEY_F3,
		.gpio		= S3C2410_GPG(4),
		.active_low	= 1,
		.desc		= "Freeze button",
	},
	{
		.code		= KEY_F4,
		.gpio		= S3C2410_GPG(6),
		.active_low	= 1,
		.desc		= "NIBP button",
	},
	{
		.code		= KEY_F5,
		.gpio		= S3C2410_GPG(7),
		.active_low	= 1,
		.desc		= "Print button",
	},
	{
		.code		= KEY_F6,
		.gpio		= S3C2410_GPG(0),
		.active_low	= 1,
		.desc		= "Menu button",
	},
	{
		.code		= KEY_UP,
		.gpio		= S3C2410_GPF(6),
		.inverse_gpio	= S3C2410_GPF(5),
		.active_low	= 1,
		.desc		= "Up button",
	},
	{
		.code		= KEY_DOWN,
		.gpio		= S3C2410_GPF(5),
		.inverse_gpio	= S3C2410_GPF(6),
		.active_low	= 1,
		.desc		= "Down button",
	},
	{
		.code		= KEY_ENTER,
		.gpio		= S3C2410_GPF(1),
		.active_low	= 1,
		.desc		= "Ok button"
	},
};

static int s3c_gpio_keys_enable(struct device *dev)
{
	s3c_gpio_setpull(S3C2410_GPF(1), S3C_GPIO_PULL_UP);
	s3c_gpio_setpull(S3C2410_GPF(3), S3C_GPIO_PULL_UP);
	s3c_gpio_setpull(S3C2410_GPF(5), S3C_GPIO_PULL_UP);
	s3c_gpio_setpull(S3C2410_GPF(6), S3C_GPIO_PULL_UP);
	s3c_gpio_setpull(S3C2410_GPG(0), S3C_GPIO_PULL_UP);
	s3c_gpio_setpull(S3C2410_GPG(2), S3C_GPIO_PULL_UP);
	s3c_gpio_setpull(S3C2410_GPG(3), S3C_GPIO_PULL_UP);
	s3c_gpio_setpull(S3C2410_GPG(4), S3C_GPIO_PULL_UP);
	s3c_gpio_setpull(S3C2410_GPG(6), S3C_GPIO_PULL_UP);
	s3c_gpio_setpull(S3C2410_GPG(7), S3C_GPIO_PULL_UP);

	return 0;
}

static void s3c_gpio_keys_disable(struct device *dev)
{

}

static struct gpio_keys_platform_data s3c_gpio_keys_data = {
	.buttons = s3c_gpio_keys_table,
	.nbuttons = ARRAY_SIZE(s3c_gpio_keys_table),
	.enable = s3c_gpio_keys_enable,
	.disable = s3c_gpio_keys_disable,
};

static struct platform_device s3c_device_gpiokeys = {
	.name = "gpio-keys",
	.dev.platform_data = &s3c_gpio_keys_data,
};

static struct s3c_sdhci_platdata smdk2416_hsmmc0_pdata __initdata = {
	.max_width		= 4,
	.cd_type		= S3C_SDHCI_CD_PERMANENT,
};

static struct s3c_sdhci_platdata smdk2416_hsmmc1_pdata __initdata = {
	.max_width		= 4,
	.cd_type		= S3C_SDHCI_CD_NONE,
};

static struct resource s3c_smsc911x_resources[] = {
      [0] = {
		.start  = S3C2410_CS3,
		.end    = S3C2410_CS3 + SZ_64K - 1,
		.flags  = IORESOURCE_MEM,
      },
      [1] = {
		.start = IRQ_EINT4,
		.end   = IRQ_EINT4,
		.flags = IORESOURCE_IRQ | IRQ_TYPE_LEVEL_LOW,
      },
};

static struct smsc911x_platform_config s3c_smsc911x_pdata = {
	.irq_polarity  = SMSC911X_IRQ_POLARITY_ACTIVE_LOW,
	.irq_type      = SMSC911X_IRQ_TYPE_OPEN_DRAIN,
	.flags         = SMSC911X_USE_16BIT | SMSC911X_FORCE_INTERNAL_PHY,
	.phy_interface = PHY_INTERFACE_MODE_MII,
};

struct platform_device s3c_device_smsc911x = {
	.name           = "smsc911x",
	.id             =  -1,
	.num_resources  = ARRAY_SIZE(s3c_smsc911x_resources),
	.resource       = s3c_smsc911x_resources,
	.dev = {
		.platform_data = &s3c_smsc911x_pdata,
	},
};

static struct platform_device *smdk2416_devices[] __initdata = {
	&s3c_device_fb,
	&s3c_device_wdt,
	&s3c_device_ohci,
	&s3c_device_i2c0,
	&s3c_device_hsmmc0,
	&s3c_device_usb_hsudc,
	&s3c_device_rtc,
	&s3c_device_timer[0],
	&s3c_device_backlight,
	&s3c_device_gpiokeys,
	&s3c_device_smsc911x,
};

static void __init smdk2416_map_io(void)
{
	s3c24xx_init_io(smdk2416_iodesc, ARRAY_SIZE(smdk2416_iodesc));
	s3c24xx_init_clocks(12000000);
	s3c24xx_init_uarts(smdk2416_uartcfgs, ARRAY_SIZE(smdk2416_uartcfgs));
}

static void __init smdk2416_machine_init(void)
{
	s3c_i2c0_set_platdata(NULL);
	s3c_fb_set_platdata(&smdk2416_fb_platdata);

	s3c_sdhci0_set_platdata(&smdk2416_hsmmc0_pdata);
	s3c_sdhci1_set_platdata(&smdk2416_hsmmc1_pdata);

	s3c24xx_hsudc_set_platdata(&smdk2416_hsudc_platdata);

	platform_add_devices(smdk2416_devices, ARRAY_SIZE(smdk2416_devices));
	smdk_machine_init();
}

MACHINE_START(SMDK2416, "SMDK2416")
	/* Maintainer: Yauhen Kharuzhy <jekhor@gmail.com> */
	.boot_params	= S3C2410_SDRAM_PA + 0x100,

	.init_irq	= s3c24xx_init_irq,
	.map_io		= smdk2416_map_io,
	.init_machine	= smdk2416_machine_init,
	.timer		= &s3c24xx_timer,
MACHINE_END
