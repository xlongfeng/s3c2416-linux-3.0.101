/*
 * si4432 spi driver
 *
 * Copyright (c) 2015, longfeng.xiao <xlongfeng@126.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */


#ifndef __SI4432_H__
#define __SI4432_H__

#include <linux/types.h>
#include <linux/memory.h>

struct si4432_platform_data {
	int gpio_irq;
	int gpio_sdn;
};

struct si4432_ioc_transfer {
	__u64		tx_buf;
	__u64		rx_buf;
	__u32		len;
};

/* IOCTL commands */

#define SI4432_IOC_MAGIC			's'

/* not all platforms use <asm-generic/ioctl.h> or _IOC_TYPECHECK() ... */
#define SI4432_MSGSIZE(N) \
	((((N)*(sizeof (struct si4432_ioc_transfer))) < (1 << _IOC_SIZEBITS)) \
		? ((N)*(sizeof (struct si4432_ioc_transfer))) : 0)
#define SI4432_IOC_MESSAGE(N) _IOW(SI4432_IOC_MAGIC, 0, char[SI4432_MSGSIZE(N)])

#define SI4432_IOC_RESET			_IOR(SI4432_IOC_MAGIC, 1, __u8)

#endif /* __SI4432_H__ */
