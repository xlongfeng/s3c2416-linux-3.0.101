/* linux/arch/arm/plat-s3c2416/dev-spi.c
 *
 * Copyright (C) 2009 Samsung Electronics Ltd.
 *	Jaswinder Singh <jassi.brar@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/gpio.h>

#include <mach/dma.h>
#include <mach/map.h>
#include <mach/irqs.h>

#include <plat/s3c2416-spi.h>
#include <plat/gpio-cfg.h>
#include <plat/devs.h>

static char *spi_src_clks[] = {
	"pclk",
	"usb-bus",
	"epll",
};

/* SPI Controller platform_devices */

/* Since we emulate multi-cs capability, we do not touch the GPC-3,7.
 * The emulated CS is toggled by board specific mechanism, as it can
 * be either some immediate GPIO or some signal out of some other
 * chip in between ... or some yet another way.
 * We simply do not assume anything about CS.
 */
static int s3c2416_spi_cfg_gpio(struct platform_device *pdev)
{
	s3c_gpio_cfgall_range(S3C2410_GPE(11), 3,
			      S3C_GPIO_SFN(2), S3C_GPIO_PULL_UP);

	return 0;
}

static struct resource s3c2416_spi_resource[] = {
	[0] = {
		.start = S3C2416_PA_SPI,
		.end   = S3C2416_PA_SPI + 0x100 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_SPI0,
		.end   = IRQ_SPI0,
		.flags = IORESOURCE_IRQ,
	},
};

static struct s3c2416_spi_info s3c2416_spi_pdata = {
	.cfg_gpio = s3c2416_spi_cfg_gpio,
	.fifo_lvl_mask = 0x7f,
	.rx_lvl_offset = 13,
	.tx_st_done = 21,
};

struct platform_device s3c2416_device_spi = {
	.name		  = "s3c2416-spi",
	.id		  = 0,
	.num_resources	  = ARRAY_SIZE(s3c2416_spi_resource),
	.resource	  = s3c2416_spi_resource,
	.dev = {
		.platform_data = &s3c2416_spi_pdata,
	},
};
EXPORT_SYMBOL(s3c2416_device_spi);

void __init s3c2416_spi_set_info(int src_clk_nr, int num_cs)
{
	struct s3c2416_spi_info *pd;

	/* Reject invalid configuration */
	if (!num_cs || src_clk_nr < 0 || src_clk_nr > 3) {
		printk(KERN_ERR "%s: Invalid SPI configuration\n", __func__);
		return;
	}

	pd = &s3c2416_spi_pdata;
	pd->num_cs = num_cs;
	pd->src_clk_nr = src_clk_nr;
	pd->src_clk_name = spi_src_clks[src_clk_nr];
}
