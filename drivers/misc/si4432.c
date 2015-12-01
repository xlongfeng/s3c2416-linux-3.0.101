/*
 * si4432 spi driver
 *
 * Copyright (c) 2015, longfeng.xiao <xlongfeng@126.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your osiion) any later version.
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

#include <linux/types.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/completion.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/spi/spi.h>
#include <linux/spi/si4432.h>
#include <linux/pwm.h>
#include <linux/device.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <asm/uaccess.h>

enum AntennaMode {
        RXMode = 0x04, TXMode = 0x08, Ready = 0x01, TuneMode = 0x02
};

enum Registers {
	REG_DEV_TYPE = 0x00,
	REG_DEV_VERSION = 0x01,
	REG_DEV_STATUS = 0x02,

	REG_INT_STATUS1 = 0x03,
	REG_INT_STATUS2 = 0x04,
	REG_INT_ENABLE1 = 0x05,
	REG_INT_ENABLE2 = 0x06,
	REG_STATE = 0x07,
	REG_OPERATION_CONTROL = 0x08,

	REG_GPIO0_CONF = 0x0B,
	REG_GPIO1_CONF = 0x0C,
	REG_GPIO2_CONF = 0x0D,
	REG_IOPORT_CONF = 0x0E,

	REG_IF_FILTER_BW = 0x1C,
	REG_AFC_LOOP_GEARSHIFT_OVERRIDE = 0x1D,
	REG_AFC_TIMING_CONTROL = 0x1E,
	REG_CLOCK_RECOVERY_GEARSHIFT = 0x1F,
	REG_CLOCK_RECOVERY_OVERSAMPLING = 0x20,
	REG_CLOCK_RECOVERY_OFFSET2 = 0x21,
	REG_CLOCK_RECOVERY_OFFSET1 = 0x22,
	REG_CLOCK_RECOVERY_OFFSET0 = 0x23,
	REG_CLOCK_RECOVERY_TIMING_GAIN1 = 0x24,
	REG_CLOCK_RECOVERY_TIMING_GAIN0 = 0x25,
	REG_RSSI = 0x26,
	REG_RSSI_THRESHOLD = 0x27,

	REG_AFC_LIMITER = 0x2A,
	REG_AFC_CORRECTION_READ = 0x2B,

	REG_DATAACCESS_CONTROL = 0x30,
	REG_EZMAC_STATUS = 0x31,
	REG_HEADER_CONTROL1 = 0x32,
	REG_HEADER_CONTROL2 = 0x33,
	REG_PREAMBLE_LENGTH = 0x34,
	REG_PREAMBLE_DETECTION = 0x35,
	REG_SYNC_WORD3 = 0x36,
	REG_SYNC_WORD2 = 0x37,
	REG_SYNC_WORD1 = 0x38,
	REG_SYNC_WORD0 = 0x39,
	REG_TRANSMIT_HEADER3 = 0x3A,
	REG_TRANSMIT_HEADER2 = 0x3B,
	REG_TRANSMIT_HEADER1 = 0x3C,
	REG_TRANSMIT_HEADER0 = 0x3D,

	REG_PKG_LEN = 0x3E,

	REG_CHECK_HEADER3 = 0x3F,
	REG_CHECK_HEADER2 = 0x40,
	REG_CHECK_HEADER1 = 0x41,
	REG_CHECK_HEADER0 = 0x42,

	REG_RECEIVED_HEADER3 = 0x47,
	REG_RECEIVED_HEADER2 = 0x48,
	REG_RECEIVED_HEADER1 = 0x49,
	REG_RECEIVED_HEADER0 = 0x4A,

	REG_RECEIVED_LENGTH = 0x4B,

	REG_CHARGEPUMP_OVERRIDE = 0x58,
	REG_DIVIDER_CURRENT_TRIM = 0x59,
	REG_VCO_CURRENT_TRIM = 0x5A,

	REG_AGC_OVERRIDE = 0x69,

	REG_TX_POWER = 0x6D,
	REG_TX_DATARATE1 = 0x6E,
	REG_TX_DATARATE0 = 0x6F,

	REG_MODULATION_MODE1 = 0x70,
	REG_MODULATION_MODE2 = 0x71,

	REG_FREQ_DEVIATION = 0x72,
	REG_FREQ_OFFSET1 = 0x73,
	REG_FREQ_OFFSET2 = 0x74,
	REG_FREQBAND = 0x75,
	REG_FREQCARRIER_H = 0x76,
	REG_FREQCARRIER_L = 0x77,

	REG_FREQCHANNEL = 0x79,
	REG_CHANNEL_STEPSIZE = 0x7A,

	REG_FIFO = 0x7F,
};

static const u16 IFFilterTable[][2] = {
	{ 322, 0x26 },
	{ 3355, 0x88 },
	{ 3618, 0x89 },
	{ 4202, 0x8A },
	{ 4684, 0x8B },
	{ 5188, 0x8C },
	{ 5770, 0x8D },
	{ 6207, 0x8E }
};

static u64 _freqCarrier = 443;
static u8 _freqChannel = 0;
static u16 _kbps = 30;
static u16 _packageSign = 0xDEAD;

struct si4432 {
	struct spi_device	*spi;
	dev_t			devt;

	struct completion	interrupt;
	int			irq;
	int			gpio_irq;
	int			gpio_sdn;
	u8			buffer[64];

	struct mutex		lock;
	bool			stopped;	/* P: lock */
	bool			disabled;	/* P: lock */
	bool			suspended;	/* P: lock */


	struct spi_message	msg;
	struct spi_transfer	xfer;
	u8			tx_buf[64];
	u8			rx_buf[64];
	u16			rx_length;
};

static struct si4432 *si4432dev;

static void write_burst(struct si4432 *si, enum Registers startReg, const u8 *value, u8 length)
{
	int err;

	si->buffer[0] = (u8) startReg | 0x80; // set MSB
	memcpy(si->buffer + 1, value, length);

	spi_message_init(&si->msg);

	si->xfer.tx_buf = si->buffer;
	si->xfer.rx_buf = 0;
	si->xfer.len = length + 1;

	spi_message_add_tail(&si->xfer, &si->msg);
	err = spi_sync(si->spi, &si->msg);
	if (err < 0) {
		dev_err(&si->spi->dev, "spi_sync failed with %d\n", err);
	}
}
#define burstWrite(startReg, value, length) write_burst(si, startReg, value, length)

static void read_burst(struct si4432 *si, enum Registers startReg, u8 *value, u8 length)
{
	int err;

	memset(si->buffer, 0xff, length + 1);

	si->buffer[0] = ((u8) startReg & 0x7F); // clr MSB

	spi_message_init(&si->msg);

	si->xfer.tx_buf = si->buffer;
	si->xfer.rx_buf = si->buffer;
	si->xfer.len = length + 1;

	spi_message_add_tail(&si->xfer, &si->msg);
	err = spi_sync(si->spi, &si->msg);
	if (err < 0) {
		dev_err(&si->spi->dev, "spi_sync failed with %d\n", err);
	}
	memcpy(value, si->buffer + 1, length);
}
#define burstRead(startReg, value, length) read_burst(si, startReg, value, length)

static void write_register(struct si4432 *si, enum Registers reg, u8 value)
{
	burstWrite(reg, &value, 1);
}
#define changeRegister(reg, value) write_register(si, reg, value)

static u8 read_register(struct si4432 *si, enum Registers reg)
{
	u8 val = 0xFF;
	burstRead(reg, &val, 1);
	return val;
}
#define readRegister(reg) read_register(si, reg)

static void turnOn(struct si4432 *si)
{
	gpio_set_value(si->gpio_sdn, 0);
	msleep(20);
}

static void turnOff(struct si4432 *si)
{
	gpio_set_value(si->gpio_sdn, 1);
	msleep(1);
}

static void setFrequency(struct si4432 *si, unsigned long baseFrequencyMhz)
{
	u8 freqband = 0x13;
	u16 freqcarrier = 0x4aff;
	u8 highBand = (baseFrequencyMhz >= 480);
	u8 vals[3] = {
		(u8)(0x40 | (highBand << 5) | (freqband & 0x3F)),
		(u8)(freqcarrier >> 8),
		(u8)(freqcarrier & 0xFF)
	};

	burstWrite(REG_FREQBAND, vals, 3);
}

static void setBaudRate(struct si4432 *si, u16 kbps)
{
	u8 modulationVals[] = { 0x0c, 0x23, 0xf0 };
	u8 datarateVals[] = { 0x07, 0xae};
	u8 IFValue = 0x88;
	u8 timingVals[] = { 0x09, 0x20, 0x51, 0xec, 0x00, 0x22 };

	burstWrite(REG_MODULATION_MODE1, modulationVals, 3);
	burstWrite(REG_TX_DATARATE1, datarateVals, 2);
	changeRegister(REG_IF_FILTER_BW, IFValue);
	burstWrite(REG_CLOCK_RECOVERY_OVERSAMPLING, timingVals, 6);
}

static void setChannel(struct si4432 *si, u8 channel)
{
	changeRegister(REG_FREQCHANNEL, channel);
}

static void setCommsSignature(struct si4432 *si, u16 signature)
{
	_packageSign = signature;

	changeRegister(REG_TRANSMIT_HEADER3, _packageSign >> 8); // header (signature) u8 3 val
	changeRegister(REG_TRANSMIT_HEADER2, (_packageSign & 0xFF)); // header (signature) u8 2 val

	changeRegister(REG_CHECK_HEADER3, _packageSign >> 8); // header (signature) u8 3 val for receive checks
	changeRegister(REG_CHECK_HEADER2, (_packageSign & 0xFF)); // header (signature) u8 2 val for receive checks
#ifdef DEBUG
	printk("Package signature is set!\n");
#endif
}

static void switchMode(struct si4432 *si, u8 mode)
{
	if (mode & TXMode)
		udelay(1000);
	changeRegister(REG_STATE, mode); // receive mode
#ifdef DEBUG
	{
		u8 val = readRegister(REG_DEV_STATUS);
		if (val == 0 || val == 0xFF) {
			printk("%x -- WHAT THE HELL!!\n", val);
		}
	}
#endif
}

static void boot(struct si4432 *si)
{
	/*
	u8 currentFix[] = { 0x80, 0x40, 0x7F };
	burstWrite(REG_CHARGEPUMP_OVERRIDE, currentFix, 3); // refer to AN440 for reasons

	changeRegister(REG_GPIO0_CONF, 0x0F); // tx/rx data clk pin
	changeRegister(REG_GPIO1_CONF, 0x00); // POR inverted pin
	changeRegister(REG_GPIO2_CONF, 0x1C); // clear channel pin
	*/
	changeRegister(REG_AFC_TIMING_CONTROL, 0x02); // refer to AN440 for reasons
	changeRegister(REG_AFC_LIMITER, 0xFF); // write max value - excel file did that.
	changeRegister(REG_AGC_OVERRIDE, 0x60); // max gain control
	changeRegister(REG_AFC_LOOP_GEARSHIFT_OVERRIDE, 0x3C); // turn off AFC
	changeRegister(REG_DATAACCESS_CONTROL, 0xAD); // enable rx packet handling, enable tx packet handling, enable CRC, use CRC-IBM
	changeRegister(REG_HEADER_CONTROL1, 0x0C); // no broadcast address control, enable check headers for u8s 3 & 2
	changeRegister(REG_HEADER_CONTROL2, 0x22);  // enable headers u8 3 & 2, no fixed package length, sync word 3 & 2
	changeRegister(REG_PREAMBLE_LENGTH, 0x08); // 8 * 4 bits = 32 bits (4 u8s) preamble length
	changeRegister(REG_PREAMBLE_DETECTION, 0x3A); // validate 7 * 4 bits of preamble  in a package
	changeRegister(REG_SYNC_WORD3, 0x2D); // sync u8 3 val
	changeRegister(REG_SYNC_WORD2, 0xD4); // sync u8 2 val

	changeRegister(REG_TX_POWER, 0x1F); // max power

	changeRegister(REG_CHANNEL_STEPSIZE, 0x64); // each channel is of 1 Mhz interval

	setFrequency(si, _freqCarrier); // default freq
	setBaudRate(si, _kbps); // default baud rate is 100kpbs
	setChannel(si, _freqChannel); // default channel is 0
	setCommsSignature(si, _packageSign); // default signature

	switchMode(si, Ready);
}

static void hardReset(struct si4432 *si)
{
	unsigned long timeout;
	u8 reg;

	turnOff(si);
	turnOn(si);

	timeout = jiffies +  msecs_to_jiffies(100);

	do {
		reg = readRegister(REG_INT_STATUS2);
		if ((reg & 0x02) == 0x02)
			break;
#ifdef DEBUG
		printk("POR: %x\n", reg);
#endif
		if (time_after(jiffies, timeout)) {
			return;
		}
	} while (1);

	dev_info(&si->spi->dev, "POR: %x\n", reg);

	boot(si);
}

static void softReset(struct si4432 *si)
{
	u8 reg;

	changeRegister(REG_STATE, 0x80);

	reg = readRegister(REG_INT_STATUS2);
	while ((reg & 0x02) != 0x02) {
		msleep(1);
		reg = readRegister(REG_INT_STATUS2);
	}

	boot(si);
}

static void clearTxFIFO(struct si4432 *si)
{
	changeRegister(REG_OPERATION_CONTROL, 0x01);
	changeRegister(REG_OPERATION_CONTROL, 0x00);
}

static void clearRxFIFO(struct si4432 *si)
{
	changeRegister(REG_OPERATION_CONTROL, 0x02);
	changeRegister(REG_OPERATION_CONTROL, 0x00);
}

static void clearFIFO(struct si4432 *si)
{
	changeRegister(REG_OPERATION_CONTROL, 0x03);
	changeRegister(REG_OPERATION_CONTROL, 0x00);
}

static void startListening(struct si4432 *si)
{
	clearRxFIFO(si); // clear first, so it doesn't overflow if packet is big

	changeRegister(REG_INT_ENABLE1, 0x03); // set interrupts on for package received and CRC error

#ifdef DEBUG
	changeRegister(REG_INT_ENABLE2, 0xC0);
#else
	changeRegister(REG_INT_ENABLE2, 0x00); // set other interrupts off
#endif
	//read interrupt registers to clean them
	readRegister(REG_INT_STATUS1);
	readRegister(REG_INT_STATUS2);

	switchMode(si, RXMode | Ready);
}

static void getPacketReceived(struct si4432 *si)
{
	si->rx_length = readRegister(REG_RECEIVED_LENGTH);

	burstRead(REG_FIFO, si->rx_buf, si->rx_length);

	clearRxFIFO(si); // which will also clear the interrupts
}

static bool isPacketReceived(struct si4432 *si)
{
	// check for package received status interrupt register
	u8 intStat = readRegister(REG_INT_STATUS1);

#ifdef DEBUG
	u8 intStat2 = readRegister(REG_INT_STATUS2);

	if (intStat2 & 0x40) { //interrupt occured, check it && read the Interrupt Status1 register for 'preamble '
		printk("HEY!! HEY!! Valid Preamble detected -- %x\n", intStat2);
	}

	if (intStat2 & 0x80) { //interrupt occured, check it && read the Interrupt Status1 register for 'preamble '
		printk("HEY!! HEY!! SYNC WORD detected -- %x\n", intStat2);
	}
#else
	readRegister(REG_INT_STATUS2);
#endif

	if (intStat & 0x02) { //interrupt occured, check it && read the Interrupt Status1 register for 'valid packet'
		switchMode(si, Ready | TuneMode); // if packet came, get out of Rx mode till the packet is read out. Keep PLL on for fast reaction
#ifdef DEBUG
		printk("Packet detected -- %x\n", intStat);
#endif
		return true;
	} else if (intStat & 0x01) { // packet crc error
		switchMode(si, Ready); // get out of Rx mode till buffers are cleared
		dev_err(&si->spi->dev, "CRC Error in Packet detected!-- %x\n", intStat);
		clearRxFIFO(si);
		switchMode(si, RXMode | Ready); // get back to work
		return false;
	}

	// no relevant interrupt? no packet!
	return false;
}

static bool waitForPacket(struct si4432 *si, int waitMs)
{
	unsigned long tmo;

	startListening(si);

	do {
		si->interrupt.done = 0;
		tmo = wait_for_completion_timeout(&si->interrupt, msecs_to_jiffies(waitMs));
		if (tmo > 0 && isPacketReceived(si)) {
			return true;
		}
	} while (tmo > 0);

	switchMode(si, Ready);
	clearRxFIFO(si);
	return false;
}

static bool sendPacket(struct si4432 *si, u8 length, const u8* data)
{
	unsigned long tmo;

	clearTxFIFO(si);
	changeRegister(REG_PKG_LEN, length);

	burstWrite(REG_FIFO, data, length);

	changeRegister(REG_INT_ENABLE1, 0x04); // set interrupts on for package sent
	changeRegister(REG_INT_ENABLE2, 0x00); // set interrupts off for anything else
	//read interrupt registers to clean them
	readRegister(REG_INT_STATUS1);
	readRegister(REG_INT_STATUS2);

	switchMode(si, TXMode | Ready);

	si->interrupt.done = 0;
	tmo = wait_for_completion_timeout(&si->interrupt, msecs_to_jiffies(100));

	if (tmo > 0) {
		u8 intStatus = readRegister(REG_INT_STATUS1);
		readRegister(REG_INT_STATUS2);
		if (intStatus & 0x04) {
			switchMode(si, Ready | TuneMode);
#ifdef DEBUG
			printk("Package sent! -- %x\n", intStatus);
#endif
			return true;
		}
	} else {
		// timeout occured.
		dev_err(&si->spi->dev, "packet transmit timeout\n");
		switchMode(si, Ready);
		if (readRegister(REG_DEV_STATUS) & 0x80) {
			clearFIFO(si);
		}
        }

	return false;
}

static irqreturn_t si4432_interrupt(int irq, void *dev_id)
{
	struct si4432 *si = dev_id;
	complete(&si->interrupt);
	return IRQ_HANDLED;
}

static int si4432_message(struct si4432 *si,
		struct si4432_ioc_transfer *u_xfers, unsigned n_xfers)
{
	int			status = -EFAULT;
	unsigned		len;

	len = u_xfers->len;

	if (u_xfers->rx_buf) {
		if (!access_ok(VERIFY_WRITE, (u8 __user *)
					(uintptr_t) u_xfers->rx_buf,
					u_xfers->len))
			goto done;
	}
	if (u_xfers->tx_buf) {
		if (copy_from_user(si->tx_buf, (const u8 __user *)
					(uintptr_t) u_xfers->tx_buf,
					len))
			goto done;
	}

	if (sendPacket(si, len, si->tx_buf)) {
		status = len;
		if (u_xfers->rx_buf) {
			// package sent. now, return true if not to wait ack,
			// or wait ack (wait for packet only for 'remaining' amount of time)
			if (waitForPacket(si, 100)) {
				getPacketReceived(si);
				if (__copy_to_user((u8 __user *)
						(uintptr_t) u_xfers->rx_buf, si->rx_buf,
						si->rx_length)) {
					status = -EFAULT;
					goto done;
				}
				status = si->rx_length;
			} else {
				status = -EFAULT;
			}
		}
	}

done:
	return status;
}

static long
spidev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int			err = 0;
	int			retval = 0;
	struct si4432		*si;
	u32			tmp;
	unsigned		n_ioc;
	struct si4432_ioc_transfer	*ioc;

	/* Check type and command number */
	if (_IOC_TYPE(cmd) != SI4432_IOC_MAGIC)
		return -ENOTTY;

	/* Check access direction once here; don't repeat below.
	 * IOC_DIR is from the user perspective, while access_ok is
	 * from the kernel perspective; so they look reversed.
	 */
	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE,
				(void __user *)arg, _IOC_SIZE(cmd));
	if (err == 0 && _IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ,
				(void __user *)arg, _IOC_SIZE(cmd));
	if (err)
		return -EFAULT;

	/* guard against device removal before, or while,
	 * we issue this ioctl.
	 */
	si = si4432dev;
	if (si == NULL)
		return -ESHUTDOWN;

	mutex_lock(&si->lock);

	switch (cmd) {
	case SI4432_IOC_RESET:
		dev_info(&si->spi->dev, "hard reset");
		hardReset(si);
		break;
	default:
		/* segmented and/or full-duplex I/O request */
		if (_IOC_NR(cmd) != _IOC_NR(SI4432_IOC_MESSAGE(0))
				|| _IOC_DIR(cmd) != _IOC_WRITE) {
			retval = -ENOTTY;
			break;
		}

		tmp = _IOC_SIZE(cmd);
		if ((tmp % sizeof(struct si4432_ioc_transfer)) != 0) {
			retval = -EINVAL;
			break;
		}
		n_ioc = tmp / sizeof(struct si4432_ioc_transfer);
		if (n_ioc != 1)
			break;

		/* copy into scratch area */
		ioc = kmalloc(tmp, GFP_KERNEL);
		if (!ioc) {
			retval = -ENOMEM;
		}
		if (__copy_from_user(ioc, (void __user *)arg, tmp)) {
			kfree(ioc);
			retval = -EFAULT;
			break;
		}

		/* translate to si4432_message, execute */
		retval = si4432_message(si, ioc, 1 /* n_ioc */);
		kfree(ioc);
		break;
	}

	mutex_unlock(&si->lock);
	return retval;
}

#ifdef CONFIG_COMPAT
static long
spidev_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	return spidev_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
}
#else
#define spidev_compat_ioctl NULL
#endif /* CONFIG_COMPAT */

static const struct file_operations spidev_fops = {
	.owner =	THIS_MODULE,
	.unlocked_ioctl = spidev_ioctl,
	.compat_ioctl = spidev_compat_ioctl,
	.llseek =	no_llseek,
};

/* The main reason to have this class is to make mdev/udev create the
 * /dev/si4432 character device nodes exposing our userspace API.
 * It also simplifies memory management.
 */

static struct class *si4432_class;
static int major;		/* major number we get from the kernel */

#ifdef CONFIG_PM_SLEEP
static int si4432_suspend(struct device *dev)
{
	struct si4432 *si = dev_get_drvdata(dev);

	mutex_lock(&si->lock);

	if (!si->suspended) {

		/* if (!si->disabled) */
			/* __si4432_disable(si); */

		if (device_may_wakeup(&si->spi->dev))
			enable_irq_wake(si->spi->irq);

		si->suspended = true;
	}

	mutex_unlock(&si->lock);

	return 0;
}

static int si4432_resume(struct device *dev)
{
	struct si4432 *si = dev_get_drvdata(dev);

	mutex_lock(&si->lock);

	if (si->suspended) {

		si->suspended = false;

		if (device_may_wakeup(&si->spi->dev))
			disable_irq_wake(si->spi->irq);

		/* if (!si->disabled) */
			/* __si4432_enable(si); */
	}

	mutex_unlock(&si->lock);

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(si4432_pm, si4432_suspend, si4432_resume);

static int __devinit si4432_probe(struct spi_device *spi)
{
	struct si4432 *si;
	struct device *dev;
	struct si4432_platform_data *pdata = spi->dev.platform_data;
	int err;

	if (si4432dev) {
		return -EBUSY;
	}

	if (!pdata) {
		dev_dbg(&spi->dev, "no platform data?\n");
		return -ENODEV;
	}

	/* We'd set TX word size 8 bisi and RX word size to 13 bisi ... excesi
	 * that even if the hardware can do that, the SPI controller driver
	 * may not.  So we stick to very-portable 8 bit words, both RX and TX.
	 */
	spi->bits_per_word = 8;
	spi->mode = SPI_MODE_0;
	err = spi_setup(spi);
	if (err < 0)
		return err;

	si = kzalloc(sizeof(struct si4432), GFP_KERNEL);
	if (!si) {
		return -ENOMEM;
	}

	dev_set_drvdata(&spi->dev, si);

	si->spi = spi;

	init_completion(&si->interrupt);
	si->gpio_irq = pdata->gpio_irq;
	si->gpio_sdn = pdata->gpio_sdn;

	WARN_ON(gpio_request(si->gpio_irq, "si4432"));
	WARN_ON(gpio_request(si->gpio_sdn, "si4432"));
	gpio_direction_output(si->gpio_sdn, 1);

	mutex_init(&si->lock);

	si->irq = gpio_to_irq(si->gpio_irq);
	err = request_irq(si->irq, si4432_interrupt,
			IRQF_TRIGGER_FALLING | IRQF_DISABLED, "si4432", si);
	if (err) {
		dev_err(&si->spi->dev, "Unable to claim irq %d, error %d\n",
				si->irq,
				err);
		goto err_free_gpio;
	}

	si->devt = MKDEV(major, 0);
	dev = device_create(si4432_class, &si->spi->dev, si->devt,
			    si, "si4432");
	if (IS_ERR(dev)) {
		err = PTR_ERR(dev);
		goto err_free_irq;
	}

	dev_info(&spi->dev, "si4432 dirver\n");

	device_init_wakeup(&spi->dev, true);

	hardReset(si);

	si4432dev = si;

	return 0;

 err_free_irq:
	free_irq(si->irq, si);
 err_free_gpio:
	gpio_free(si->gpio_sdn);
	gpio_free(si->gpio_irq);
 err_free_mem:
	kfree(si);
	return err;
}

static int __devexit si4432_remove(struct spi_device *spi)
{
	struct si4432 *si = dev_get_drvdata(&spi->dev);

	si4432dev = NULL;

	turnOff(si);

	device_init_wakeup(&spi->dev, false);

	device_destroy(si4432_class, si->devt);

	free_irq(si->irq, si);

	gpio_free(si->gpio_sdn);
	gpio_free(si->gpio_irq);

	kfree(si);

	dev_dbg(&spi->dev, "unregistered si4432\n");

	return 0;
}

static struct spi_driver si4432_driver = {
	.driver = {
		.name	= "si4432",
		.bus	= &spi_bus_type,
		.owner	= THIS_MODULE,
		.pm	= &si4432_pm,
	},
	.probe		= si4432_probe,
	.remove		= __devexit_p(si4432_remove),
};

static int __init si4432_init(void)
{
	int ret;

	major = register_chrdev(0, "si4432", &spidev_fops);
	if (major < 0)
		return major;

	si4432_class = class_create(THIS_MODULE, "si4432");
	if (IS_ERR(si4432_class)) {
		unregister_chrdev(major, si4432_driver.driver.name);
		return PTR_ERR(si4432_class);
	}

	ret = spi_register_driver(&si4432_driver);
	if (ret < 0) {
		class_destroy(si4432_class);
		unregister_chrdev(major, si4432_driver.driver.name);
	}
	return ret;
}
module_init(si4432_init);

static void __exit si4432_exit(void)
{
	spi_unregister_driver(&si4432_driver);
	class_destroy(si4432_class);
	unregister_chrdev(major, si4432_driver.driver.name);
}
module_exit(si4432_exit);

MODULE_DESCRIPTION("SI4432 Driver");
MODULE_AUTHOR("longfeng.xiao <xlongfeng@126.com>");
MODULE_LICENSE("GPL");
