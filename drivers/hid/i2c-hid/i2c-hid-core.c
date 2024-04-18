/*
 * HID over I2C protocol implementation
 *
 * Copyright (c) 2012 Benjamin Tissoires <benjamin.tissoires@gmail.com>
 * Copyright (c) 2012 Ecole Nationale de l'Aviation Civile, France
 * Copyright (c) 2012 Red Hat, Inc
 *
 * This code is partly based on "USB HID support for Linux":
 *
 *  Copyright (c) 1999 Andreas Gal
 *  Copyright (c) 2000-2005 Vojtech Pavlik <vojtech@suse.cz>
 *  Copyright (c) 2005 Michael Haboustak <mike-@cinci.rr.com> for Concept2, Inc
 *  Copyright (c) 2007-2008 Oliver Neukum
 *  Copyright (c) 2006-2010 Jiri Kosina
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive for
 * more details.
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/pm.h>
#include <linux/pm_wakeirq.h>
#include <linux/device.h>
#include <linux/wait.h>
#include <linux/err.h>
#include <linux/string.h>
#include <linux/list.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/hid.h>
#include <linux/mutex.h>

#include <drm/drm_panel.h>

#include "../hid-ids.h"
#include "i2c-hid.h"

/* quirks to control the device */
#define I2C_HID_QUIRK_SET_PWR_WAKEUP_DEV	BIT(0)
#define I2C_HID_QUIRK_NO_IRQ_AFTER_RESET	BIT(1)
#define I2C_HID_QUIRK_BOGUS_IRQ			BIT(4)
#define I2C_HID_QUIRK_RESET_ON_RESUME		BIT(5)
#define I2C_HID_QUIRK_BAD_INPUT_SIZE		BIT(6)
#define I2C_HID_QUIRK_NO_WAKEUP_AFTER_RESET	BIT(7)


/* flags */
#define I2C_HID_STARTED		0
#define I2C_HID_RESET_PENDING	1
#define I2C_HID_READ_PENDING	2

#define I2C_HID_PWR_ON		0x00
#define I2C_HID_PWR_SLEEP	0x01

#define EVE_TP_I2C_ADDR	0x49
#define EVE_TP_RETRIES	10
#define EVE_TP_DELAY_MS 1

/* debug option */
static bool debug;
module_param(debug, bool, 0444);
MODULE_PARM_DESC(debug, "print a lot of debug information");

#define i2c_hid_dbg(ihid, fmt, arg...)					  \
do {									  \
	if (debug)							  \
		dev_printk(KERN_DEBUG, &(ihid)->client->dev, fmt, ##arg); \
} while (0)

struct i2c_hid_desc {
	__le16 wHIDDescLength;
	__le16 bcdVersion;
	__le16 wReportDescLength;
	__le16 wReportDescRegister;
	__le16 wInputRegister;
	__le16 wMaxInputLength;
	__le16 wOutputRegister;
	__le16 wMaxOutputLength;
	__le16 wCommandRegister;
	__le16 wDataRegister;
	__le16 wVendorID;
	__le16 wProductID;
	__le16 wVersionID;
	__le32 reserved;
} __packed;

struct i2c_hid_cmd {
	unsigned int registerIndex;
	__u8 opcode;
	unsigned int length;
	bool wait;
};

union command {
	u8 data[0];
	struct cmd {
		__le16 reg;
		__u8 reportTypeID;
		__u8 opcode;
		__u8 reportID;
	} __packed c;
};

#define I2C_HID_CMD(opcode_) \
	.opcode = opcode_, .length = 4, \
	.registerIndex = offsetof(struct i2c_hid_desc, wCommandRegister)

/* fetch HID descriptor */
static const struct i2c_hid_cmd hid_descr_cmd = { .length = 2 };
/* fetch report descriptors */
static const struct i2c_hid_cmd hid_report_descr_cmd = {
		.registerIndex = offsetof(struct i2c_hid_desc,
			wReportDescRegister),
		.opcode = 0x00,
		.length = 2 };
/* commands */
static const struct i2c_hid_cmd hid_reset_cmd =		{ I2C_HID_CMD(0x01),
							  .wait = true };
static const struct i2c_hid_cmd hid_get_report_cmd =	{ I2C_HID_CMD(0x02) };
static const struct i2c_hid_cmd hid_set_report_cmd =	{ I2C_HID_CMD(0x03) };
static const struct i2c_hid_cmd hid_set_power_cmd =	{ I2C_HID_CMD(0x08) };
static const struct i2c_hid_cmd hid_no_cmd =		{ .length = 0 };

/*
 * These definitions are not used here, but are defined by the spec.
 * Keeping them here for documentation purposes.
 *
 * static const struct i2c_hid_cmd hid_get_idle_cmd = { I2C_HID_CMD(0x04) };
 * static const struct i2c_hid_cmd hid_set_idle_cmd = { I2C_HID_CMD(0x05) };
 * static const struct i2c_hid_cmd hid_get_protocol_cmd = { I2C_HID_CMD(0x06) };
 * static const struct i2c_hid_cmd hid_set_protocol_cmd = { I2C_HID_CMD(0x07) };
 */

/* The main device structure */
struct i2c_hid {
	struct i2c_client	*client;	/* i2c client */
	struct hid_device	*hid;	/* pointer to corresponding HID dev */
	union {
		__u8 hdesc_buffer[sizeof(struct i2c_hid_desc)];
		struct i2c_hid_desc hdesc;	/* the HID Descriptor */
	};
	__le16			wHIDDescRegister; /* location of the i2c
						   * register of the HID
						   * descriptor. */
	unsigned int		bufsize;	/* i2c buffer size */
	u8			*inbuf;		/* Input buffer */
	u8			*rawbuf;	/* Raw Input buffer */
	u8			*cmdbuf;	/* Command buffer */
	u8			*argsbuf;	/* Command arguments buffer */

	unsigned long		flags;		/* device flags */
	unsigned long		quirks;		/* Various quirks */

	wait_queue_head_t	wait;		/* For waiting the interrupt */

	struct mutex		reset_lock;

	struct i2chid_ops	*ops;
	struct drm_panel_follower panel_follower;
	struct work_struct	panel_follower_prepare_work;
	bool			is_panel_follower;
	bool			prepare_work_finished;
};

static const struct i2c_hid_quirks {
	__u16 idVendor;
	__u16 idProduct;
	__u32 quirks;
} i2c_hid_quirks[] = {
	{ USB_VENDOR_ID_WEIDA, HID_ANY_ID,
		I2C_HID_QUIRK_SET_PWR_WAKEUP_DEV },
	{ I2C_VENDOR_ID_HANTICK, I2C_PRODUCT_ID_HANTICK_5288,
		I2C_HID_QUIRK_NO_IRQ_AFTER_RESET },
	{ I2C_VENDOR_ID_ITE, I2C_DEVICE_ID_ITE_VOYO_WINPAD_A15,
		I2C_HID_QUIRK_NO_IRQ_AFTER_RESET },
	{ I2C_VENDOR_ID_RAYDIUM, I2C_PRODUCT_ID_RAYDIUM_3118,
		I2C_HID_QUIRK_NO_IRQ_AFTER_RESET },
	{ USB_VENDOR_ID_ALPS_JP, HID_ANY_ID,
		 I2C_HID_QUIRK_RESET_ON_RESUME },
	{ I2C_VENDOR_ID_SYNAPTICS, I2C_PRODUCT_ID_SYNAPTICS_SYNA2393,
		 I2C_HID_QUIRK_RESET_ON_RESUME },
	{ USB_VENDOR_ID_ITE, I2C_DEVICE_ID_ITE_LENOVO_LEGION_Y720,
		I2C_HID_QUIRK_BAD_INPUT_SIZE },
	/*
	 * Sending the wakeup after reset actually break ELAN touchscreen controller
	 */
	{ USB_VENDOR_ID_ELAN, HID_ANY_ID,
		 I2C_HID_QUIRK_NO_WAKEUP_AFTER_RESET |
		 I2C_HID_QUIRK_BOGUS_IRQ },
	{ 0, 0 }
};

/*
 * i2c_hid_lookup_quirk: return any quirks associated with a I2C HID device
 * @idVendor: the 16-bit vendor ID
 * @idProduct: the 16-bit product ID
 *
 * Returns: a u32 quirks value.
 */
static u32 i2c_hid_lookup_quirk(const u16 idVendor, const u16 idProduct)
{
	u32 quirks = 0;
	int n;

	for (n = 0; i2c_hid_quirks[n].idVendor; n++)
		if (i2c_hid_quirks[n].idVendor == idVendor &&
		    (i2c_hid_quirks[n].idProduct == (__u16)HID_ANY_ID ||
		     i2c_hid_quirks[n].idProduct == idProduct))
			quirks = i2c_hid_quirks[n].quirks;

	return quirks;
}

static int __i2c_hid_command(struct i2c_hid *ihid,
		const struct i2c_hid_cmd *command, u8 reportID,
		u8 reportType, u8 *args, int args_len,
		unsigned char *buf_recv, int data_len)
{
	struct i2c_client *client = ihid->client;
	union command *cmd = (union command *)ihid->cmdbuf;
	int ret;
	struct i2c_msg msg[2];
	int msg_num = 1;

	int length = command->length;
	bool wait = command->wait;
	unsigned int registerIndex = command->registerIndex;

	/* special case for hid_descr_cmd */
	if (command == &hid_descr_cmd) {
		cmd->c.reg = ihid->wHIDDescRegister;
	} else {
		cmd->data[0] = ihid->hdesc_buffer[registerIndex];
		cmd->data[1] = ihid->hdesc_buffer[registerIndex + 1];
	}

	if (length > 2) {
		cmd->c.opcode = command->opcode;
		if (reportID < 0x0F) {
			cmd->c.reportTypeID = reportType << 4 | reportID;
		} else {
			cmd->c.reportTypeID = reportType << 4 | 0x0F;
			cmd->c.reportID = reportID;
			length++;
		}
	}

	memcpy(cmd->data + length, args, args_len);
	length += args_len;

	i2c_hid_dbg(ihid, "%s: cmd=%*ph\n", __func__, length, cmd->data);

	msg[0].addr = client->addr;
	msg[0].flags = client->flags & I2C_M_TEN;
	msg[0].len = length;
	msg[0].buf = cmd->data;
	if (data_len > 0) {
		msg[1].addr = client->addr;
		msg[1].flags = client->flags & I2C_M_TEN;
		msg[1].flags |= I2C_M_RD;
		msg[1].len = data_len;
		msg[1].buf = buf_recv;
		msg_num = 2;
		set_bit(I2C_HID_READ_PENDING, &ihid->flags);
	}

	if (wait)
		set_bit(I2C_HID_RESET_PENDING, &ihid->flags);

	ret = i2c_transfer(client->adapter, msg, msg_num);

	if (data_len > 0)
		clear_bit(I2C_HID_READ_PENDING, &ihid->flags);

	if (ret != msg_num)
		return ret < 0 ? ret : -EIO;

	ret = 0;

	if (wait && (ihid->quirks & I2C_HID_QUIRK_NO_IRQ_AFTER_RESET)) {
		msleep(100);
	} else if (wait) {
		i2c_hid_dbg(ihid, "%s: waiting...\n", __func__);
		if (!wait_event_timeout(ihid->wait,
				!test_bit(I2C_HID_RESET_PENDING, &ihid->flags),
				msecs_to_jiffies(5000)))
			ret = -ENODATA;
		i2c_hid_dbg(ihid, "%s: finished.\n", __func__);
	}

	return ret;
}

static int i2c_hid_command(struct i2c_hid *ihid,
		const struct i2c_hid_cmd *command,
		unsigned char *buf_recv, int data_len)
{
	return __i2c_hid_command(ihid, command, 0, 0, NULL, 0,
				buf_recv, data_len);
}

static int i2c_hid_get_report(struct i2c_hid *ihid, u8 reportType,
		u8 reportID, unsigned char *buf_recv, int data_len)
{
	u8 args[2];
	int ret;
	int args_len = 0;
	u16 readRegister = le16_to_cpu(ihid->hdesc.wDataRegister);

	i2c_hid_dbg(ihid, "%s\n", __func__);

	args[args_len++] = readRegister & 0xFF;
	args[args_len++] = readRegister >> 8;

	ret = __i2c_hid_command(ihid, &hid_get_report_cmd, reportID,
		reportType, args, args_len, buf_recv, data_len);
	if (ret) {
		dev_err(&ihid->client->dev,
			"failed to retrieve report from device.\n");
		return ret;
	}

	return 0;
}

/**
 * i2c_hid_set_or_send_report: forward an incoming report to the device
 * @ihid: the i2c hid device
 * @reportType: 0x03 for HID_FEATURE_REPORT ; 0x02 for HID_OUTPUT_REPORT
 * @reportID: the report ID
 * @buf: the actual data to transfer, without the report ID
 * @data_len: size of buf
 * @use_data: true: use SET_REPORT HID command, false: send plain OUTPUT report
 */
static int i2c_hid_set_or_send_report(struct i2c_hid *ihid, u8 reportType,
		u8 reportID, unsigned char *buf, size_t data_len, bool use_data)
{
	u8 *args = ihid->argsbuf;
	const struct i2c_hid_cmd *hidcmd;
	int ret;
	u16 dataRegister = le16_to_cpu(ihid->hdesc.wDataRegister);
	u16 outputRegister = le16_to_cpu(ihid->hdesc.wOutputRegister);
	u16 maxOutputLength = le16_to_cpu(ihid->hdesc.wMaxOutputLength);
	u16 size;
	int args_len;
	int index = 0;

	i2c_hid_dbg(ihid, "%s\n", __func__);

	if (data_len > ihid->bufsize)
		return -EINVAL;

	size =		2			/* size */ +
			(reportID ? 1 : 0)	/* reportID */ +
			data_len		/* buf */;
	args_len =	2			/* dataRegister */ +
			size			/* args */;

	if (!use_data && maxOutputLength == 0)
		return -ENOSYS;

	/*
	 * use the data register for feature reports or if the device does not
	 * support the output register
	 */
	if (use_data) {
		args[index++] = dataRegister & 0xFF;
		args[index++] = dataRegister >> 8;
		hidcmd = &hid_set_report_cmd;
	} else {
		args[index++] = outputRegister & 0xFF;
		args[index++] = outputRegister >> 8;
		hidcmd = &hid_no_cmd;
	}

	args[index++] = size & 0xFF;
	args[index++] = size >> 8;

	if (reportID)
		args[index++] = reportID;

	memcpy(&args[index], buf, data_len);

	ret = __i2c_hid_command(ihid, hidcmd, reportID,
		reportType, args, args_len, NULL, 0);
	if (ret) {
		dev_err(&ihid->client->dev,
			"failed to set a report to device.\n");
		return ret;
	}

	return data_len;
}

static int i2c_hid_set_power(struct i2c_hid *ihid, int power_state)
{
	int ret;

	i2c_hid_dbg(ihid, "%s\n", __func__);

	/*
	 * Some devices require to send a command to wakeup before power on.
	 * The call will get a return value (EREMOTEIO) but device will be
	 * triggered and activated. After that, it goes like a normal device.
	 */
	if (power_state == I2C_HID_PWR_ON &&
	    ihid->quirks & I2C_HID_QUIRK_SET_PWR_WAKEUP_DEV) {
		ret = i2c_hid_command(ihid, &hid_set_power_cmd, NULL, 0);

		/* Device was already activated */
		if (!ret)
			goto set_pwr_exit;
	}

	ret = __i2c_hid_command(ihid, &hid_set_power_cmd, power_state,
		0, NULL, 0, NULL, 0);
	if (ret)
		dev_err(&ihid->client->dev,
			"failed to change power setting.\n");

set_pwr_exit:

	/*
	 * The HID over I2C specification states that if a DEVICE needs time
	 * after the PWR_ON request, it should utilise CLOCK stretching.
	 * However, it has been observered that the Windows driver provides a
	 * 1ms sleep between the PWR_ON and RESET requests.
	 * According to Goodix Windows even waits 60 ms after (other?)
	 * PWR_ON requests. Testing has confirmed that several devices
	 * will not work properly without a delay after a PWR_ON request.
	 */
	if (!ret && power_state == I2C_HID_PWR_ON)
		msleep(60);

	return ret;
}

static int i2c_hid_hwreset(struct i2c_hid *ihid)
{
	int ret;

	i2c_hid_dbg(ihid, "%s\n", __func__);

	/*
	 * This prevents sending feature reports while the device is
	 * being reset. Otherwise we may lose the reset complete
	 * interrupt.
	 */
	mutex_lock(&ihid->reset_lock);

	ret = i2c_hid_set_power(ihid, I2C_HID_PWR_ON);
	if (ret)
		goto out_unlock;

	i2c_hid_dbg(ihid, "resetting...\n");

	ret = i2c_hid_command(ihid, &hid_reset_cmd, NULL, 0);
	if (ret) {
		dev_err(&ihid->client->dev, "failed to reset device.\n");
		i2c_hid_set_power(ihid, I2C_HID_PWR_SLEEP);
		goto out_unlock;
	}

	/* At least some SIS devices need this after reset */
	if (!(ihid->quirks & I2C_HID_QUIRK_NO_WAKEUP_AFTER_RESET))
		ret = i2c_hid_set_power(ihid, I2C_HID_PWR_ON);

out_unlock:
	mutex_unlock(&ihid->reset_lock);
	return ret;
}

static void i2c_hid_get_input(struct i2c_hid *ihid)
{
	int ret;
	u32 ret_size;
	int size = le16_to_cpu(ihid->hdesc.wMaxInputLength);

	if (size > ihid->bufsize)
		size = ihid->bufsize;

	ret = i2c_master_recv(ihid->client, ihid->inbuf, size);
	if (ret != size) {
		if (ret < 0)
			return;

		dev_err(&ihid->client->dev, "%s: got %d data instead of %d\n",
			__func__, ret, size);
		return;
	}

	ret_size = ihid->inbuf[0] | ihid->inbuf[1] << 8;

	if (!ret_size) {
		/* host or device initiated RESET completed */
		if (test_and_clear_bit(I2C_HID_RESET_PENDING, &ihid->flags))
			wake_up(&ihid->wait);
		if (ihid->hid && ihid->hid->driver && ihid->hid->driver->reset)
			ihid->hid->driver->reset(ihid->hid);
		return;
	}

	if (ihid->quirks & I2C_HID_QUIRK_BOGUS_IRQ && ret_size == 0xffff) {
		dev_warn_once(&ihid->client->dev, "%s: IRQ triggered but "
			      "there's no data\n", __func__);
		return;
	}

	if ((ret_size > size) || (ret_size < 2)) {
		if (ihid->quirks & I2C_HID_QUIRK_BAD_INPUT_SIZE) {
			ihid->inbuf[0] = size & 0xff;
			ihid->inbuf[1] = size >> 8;
			ret_size = size;
		} else {
			dev_err(&ihid->client->dev, "%s: incomplete report (%d/%d)\n",
				__func__, size, ret_size);
			return;
		}
	}

	i2c_hid_dbg(ihid, "input: %*ph\n", ret_size, ihid->inbuf);

	if (test_bit(I2C_HID_STARTED, &ihid->flags)) {
		if (ihid->hid->group != HID_GROUP_RMI)
			pm_wakeup_event(&ihid->client->dev, 0);

		hid_input_report(ihid->hid, HID_INPUT_REPORT, ihid->inbuf + 2,
				ret_size - 2, 1);
	}

	return;
}

static irqreturn_t i2c_hid_irq(int irq, void *dev_id)
{
	struct i2c_hid *ihid = dev_id;

	if (test_bit(I2C_HID_READ_PENDING, &ihid->flags))
		return IRQ_HANDLED;

	i2c_hid_get_input(ihid);

	return IRQ_HANDLED;
}

static int i2c_hid_get_report_length(struct hid_report *report)
{
	return ((report->size - 1) >> 3) + 1 +
		report->device->report_enum[report->type].numbered + 2;
}

/*
 * Traverse the supplied list of reports and find the longest
 */
static void i2c_hid_find_max_report(struct hid_device *hid, unsigned int type,
		unsigned int *max)
{
	struct hid_report *report;
	unsigned int size;

	/* We should not rely on wMaxInputLength, as some devices may set it to
	 * a wrong length. */
	list_for_each_entry(report, &hid->report_enum[type].report_list, list) {
		size = i2c_hid_get_report_length(report);
		if (*max < size)
			*max = size;
	}
}

static void i2c_hid_free_buffers(struct i2c_hid *ihid)
{
	kfree(ihid->inbuf);
	kfree(ihid->rawbuf);
	kfree(ihid->argsbuf);
	kfree(ihid->cmdbuf);
	ihid->inbuf = NULL;
	ihid->rawbuf = NULL;
	ihid->cmdbuf = NULL;
	ihid->argsbuf = NULL;
	ihid->bufsize = 0;
}

static int i2c_hid_alloc_buffers(struct i2c_hid *ihid, size_t report_size)
{
	/* the worst case is computed from the set_report command with a
	 * reportID > 15 and the maximum report length */
	int args_len = sizeof(__u8) + /* ReportID */
		       sizeof(__u8) + /* optional ReportID byte */
		       sizeof(__u16) + /* data register */
		       sizeof(__u16) + /* size of the report */
		       report_size; /* report */

	ihid->inbuf = kzalloc(report_size, GFP_KERNEL);
	ihid->rawbuf = kzalloc(report_size, GFP_KERNEL);
	ihid->argsbuf = kzalloc(args_len, GFP_KERNEL);
	ihid->cmdbuf = kzalloc(sizeof(union command) + args_len, GFP_KERNEL);

	if (!ihid->inbuf || !ihid->rawbuf || !ihid->argsbuf || !ihid->cmdbuf) {
		i2c_hid_free_buffers(ihid);
		return -ENOMEM;
	}

	ihid->bufsize = report_size;

	return 0;
}

static int i2c_hid_get_raw_report(struct hid_device *hid,
		unsigned char report_number, __u8 *buf, size_t count,
		unsigned char report_type)
{
	struct i2c_client *client = hid->driver_data;
	struct i2c_hid *ihid = i2c_get_clientdata(client);
	size_t ret_count, ask_count;
	int ret;

	if (report_type == HID_OUTPUT_REPORT)
		return -EINVAL;

	/*
	 * In case of unnumbered reports the response from the device will
	 * not have the report ID that the upper layers expect, so we need
	 * to stash it the buffer ourselves and adjust the data size.
	 */
	if (!report_number) {
		buf[0] = 0;
		buf++;
		count--;
	}

	/* +2 bytes to include the size of the reply in the query buffer */
	ask_count = min(count + 2, (size_t)ihid->bufsize);

	ret = i2c_hid_get_report(ihid,
			report_type == HID_FEATURE_REPORT ? 0x03 : 0x01,
			report_number, ihid->rawbuf, ask_count);

	if (ret < 0)
		return ret;

	ret_count = ihid->rawbuf[0] | (ihid->rawbuf[1] << 8);

	if (ret_count <= 2)
		return 0;

	ret_count = min(ret_count, ask_count);

	/* The query buffer contains the size, dropping it in the reply */
	count = min(count, ret_count - 2);
	memcpy(buf, ihid->rawbuf + 2, count);

	if (!report_number)
		count++;

	return count;
}

static int i2c_hid_output_raw_report(struct hid_device *hid, __u8 *buf,
		size_t count, unsigned char report_type, bool use_data)
{
	struct i2c_client *client = hid->driver_data;
	struct i2c_hid *ihid = i2c_get_clientdata(client);
	int report_id = buf[0];
	int ret;

	if (report_type == HID_INPUT_REPORT)
		return -EINVAL;

	mutex_lock(&ihid->reset_lock);

	/*
	 * Note that both numbered and unnumbered reports passed here
	 * are supposed to have report ID stored in the 1st byte of the
	 * buffer, so we strip it off unconditionally before passing payload
	 * to i2c_hid_set_or_send_report which takes care of encoding
	 * everything properly.
	 */
	ret = i2c_hid_set_or_send_report(ihid,
				report_type == HID_FEATURE_REPORT ? 0x03 : 0x02,
				report_id, buf + 1, count - 1, use_data);

	if (ret >= 0)
		ret++; /* add report_id to the number of transferred bytes */

	mutex_unlock(&ihid->reset_lock);

	return ret;
}

static int i2c_hid_output_report(struct hid_device *hid, __u8 *buf,
		size_t count)
{
	return i2c_hid_output_raw_report(hid, buf, count, HID_OUTPUT_REPORT,
			false);
}

static int i2c_hid_raw_request(struct hid_device *hid, unsigned char reportnum,
			       __u8 *buf, size_t len, unsigned char rtype,
			       int reqtype)
{
	switch (reqtype) {
	case HID_REQ_GET_REPORT:
		return i2c_hid_get_raw_report(hid, reportnum, buf, len, rtype);
	case HID_REQ_SET_REPORT:
		if (buf[0] != reportnum)
			return -EINVAL;
		return i2c_hid_output_raw_report(hid, buf, len, rtype, true);
	default:
		return -EIO;
	}
}

static int i2c_hid_parse(struct hid_device *hid)
{
	struct i2c_client *client = hid->driver_data;
	struct i2c_hid *ihid = i2c_get_clientdata(client);
	struct i2c_hid_desc *hdesc = &ihid->hdesc;
	unsigned int rsize;
	char *rdesc;
	int ret;
	int tries = 3;
	char *use_override;

	i2c_hid_dbg(ihid, "entering %s\n", __func__);

	rsize = le16_to_cpu(hdesc->wReportDescLength);
	if (!rsize || rsize > HID_MAX_DESCRIPTOR_SIZE) {
		dbg_hid("weird size of report descriptor (%u)\n", rsize);
		return -EINVAL;
	}

	do {
		ret = i2c_hid_hwreset(ihid);
		if (ret)
			msleep(1000);
	} while (tries-- > 0 && ret);

	if (ret)
		return ret;

	use_override = i2c_hid_get_dmi_hid_report_desc_override(client->name,
								&rsize);

	if (use_override) {
		rdesc = use_override;
		i2c_hid_dbg(ihid, "Using a HID report descriptor override\n");
	} else {
		rdesc = kzalloc(rsize, GFP_KERNEL);

		if (!rdesc) {
			dbg_hid("couldn't allocate rdesc memory\n");
			return -ENOMEM;
		}

		i2c_hid_dbg(ihid, "asking HID report descriptor\n");

		ret = i2c_hid_command(ihid, &hid_report_descr_cmd,
				      rdesc, rsize);
		if (ret) {
			hid_err(hid, "reading report descriptor failed\n");
			kfree(rdesc);
			return -EIO;
		}
	}

	i2c_hid_dbg(ihid, "Report Descriptor: %*ph\n", rsize, rdesc);

	ret = hid_parse_report(hid, rdesc, rsize);
	if (!use_override)
		kfree(rdesc);

	if (ret) {
		dbg_hid("parsing report descriptor failed\n");
		return ret;
	}

	return 0;
}

static int i2c_hid_start(struct hid_device *hid)
{
	struct i2c_client *client = hid->driver_data;
	struct i2c_hid *ihid = i2c_get_clientdata(client);
	int ret;
	unsigned int bufsize = HID_MIN_BUFFER_SIZE;

	i2c_hid_find_max_report(hid, HID_INPUT_REPORT, &bufsize);
	i2c_hid_find_max_report(hid, HID_OUTPUT_REPORT, &bufsize);
	i2c_hid_find_max_report(hid, HID_FEATURE_REPORT, &bufsize);

	if (bufsize > ihid->bufsize) {
		disable_irq(client->irq);
		i2c_hid_free_buffers(ihid);

		ret = i2c_hid_alloc_buffers(ihid, bufsize);
		enable_irq(client->irq);

		if (ret)
			return ret;
	}

	return 0;
}

static void i2c_hid_stop(struct hid_device *hid)
{
	hid->claimed = 0;
}

static int i2c_hid_open(struct hid_device *hid)
{
	struct i2c_client *client = hid->driver_data;
	struct i2c_hid *ihid = i2c_get_clientdata(client);

	set_bit(I2C_HID_STARTED, &ihid->flags);
	return 0;
}

static void i2c_hid_close(struct hid_device *hid)
{
	struct i2c_client *client = hid->driver_data;
	struct i2c_hid *ihid = i2c_get_clientdata(client);

	clear_bit(I2C_HID_STARTED, &ihid->flags);
}

struct hid_ll_driver i2c_hid_ll_driver = {
	.parse = i2c_hid_parse,
	.start = i2c_hid_start,
	.stop = i2c_hid_stop,
	.open = i2c_hid_open,
	.close = i2c_hid_close,
	.output_report = i2c_hid_output_report,
	.raw_request = i2c_hid_raw_request,
};
EXPORT_SYMBOL_GPL(i2c_hid_ll_driver);

static int i2c_hid_init_irq(struct i2c_client *client)
{
	struct i2c_hid *ihid = i2c_get_clientdata(client);
	unsigned long irqflags = 0;
	int ret;

	i2c_hid_dbg(ihid, "Requesting IRQ: %d\n", client->irq);

	if (!irq_get_trigger_type(client->irq))
		irqflags = IRQF_TRIGGER_LOW;

	ret = request_threaded_irq(client->irq, NULL, i2c_hid_irq,
				   irqflags | IRQF_ONESHOT | IRQF_NO_AUTOEN,
				   client->name, ihid);
	if (ret < 0) {
		dev_warn(&client->dev,
			"Could not register for %s interrupt, irq = %d,"
			" ret = %d\n",
			client->name, client->irq, ret);

		return ret;
	}

	return 0;
}

static int i2c_hid_fetch_hid_descriptor(struct i2c_hid *ihid)
{
	struct i2c_client *client = ihid->client;
	struct i2c_hid_desc *hdesc = &ihid->hdesc;
	unsigned int dsize;
	int ret;

	/* i2c hid fetch using a fixed descriptor size (30 bytes) */
	if (i2c_hid_get_dmi_i2c_hid_desc_override(client->name)) {
		i2c_hid_dbg(ihid, "Using a HID descriptor override\n");
		ihid->hdesc =
			*i2c_hid_get_dmi_i2c_hid_desc_override(client->name);
	} else {
		i2c_hid_dbg(ihid, "Fetching the HID descriptor\n");
		ret = i2c_hid_command(ihid, &hid_descr_cmd,
				      ihid->hdesc_buffer,
				      sizeof(struct i2c_hid_desc));
		if (ret) {
			dev_err(&ihid->client->dev, "hid_descr_cmd failed\n");
			return -ENODEV;
		}
	}

	/* Validate the length of HID descriptor, the 4 first bytes:
	 * bytes 0-1 -> length
	 * bytes 2-3 -> bcdVersion (has to be 1.00) */
	/* check bcdVersion == 1.0 */
	if (le16_to_cpu(hdesc->bcdVersion) != 0x0100) {
		dev_err(&ihid->client->dev,
			"unexpected HID descriptor bcdVersion (0x%04hx)\n",
			le16_to_cpu(hdesc->bcdVersion));
		return -ENODEV;
	}

	/* Descriptor length should be 30 bytes as per the specification */
	dsize = le16_to_cpu(hdesc->wHIDDescLength);
	if (dsize != sizeof(struct i2c_hid_desc)) {
		dev_err(&ihid->client->dev,
			"weird size of HID descriptor (%u)\n", dsize);
		return -ENODEV;
	}
	i2c_hid_dbg(ihid, "HID Descriptor: %*ph\n", dsize, ihid->hdesc_buffer);
	return 0;
}

static int i2c_hid_core_power_up(struct i2c_hid *ihid)
{
	if (!ihid->ops->power_up)
		return 0;

	return ihid->ops->power_up(ihid->ops);
}

static void i2c_hid_core_power_down(struct i2c_hid *ihid)
{
	if (!ihid->ops->power_down)
		return;

	ihid->ops->power_down(ihid->ops);
}

static void i2c_hid_core_shutdown_tail(struct i2c_hid *ihid)
{
	if (!ihid->ops->shutdown_tail)
		return;

	ihid->ops->shutdown_tail(ihid->ops);
}

static int i2c_hid_core_suspend(struct i2c_hid *ihid, bool force_poweroff)
{
	struct i2c_client *client = ihid->client;
	struct hid_device *hid = ihid->hid;
	int ret;

	ret = hid_driver_suspend(hid, PMSG_SUSPEND);
	if (ret < 0)
		return ret;

	/* Save some power */
	i2c_hid_set_power(ihid, I2C_HID_PWR_SLEEP);

	disable_irq(client->irq);

	if (force_poweroff || !device_may_wakeup(&client->dev))
		i2c_hid_core_power_down(ihid);

	return 0;
}

static int i2c_hid_core_resume(struct i2c_hid *ihid)
{
	struct i2c_client *client = ihid->client;
	struct hid_device *hid = ihid->hid;
	int ret;

	if (!device_may_wakeup(&client->dev))
		i2c_hid_core_power_up(ihid);

	enable_irq(client->irq);

	/* Instead of resetting device, simply powers the device on. This
	 * solves "incomplete reports" on Raydium devices 2386:3118 and
	 * 2386:4B33 and fixes various SIS touchscreens no longer sending
	 * data after a suspend/resume.
	 *
	 * However some ALPS touchpads generate IRQ storm without reset, so
	 * let's still reset them here.
	 */
	if (ihid->quirks & I2C_HID_QUIRK_RESET_ON_RESUME)
		ret = i2c_hid_hwreset(ihid);
	else
		ret = i2c_hid_set_power(ihid, I2C_HID_PWR_ON);

	if (ret)
		return ret;

	return hid_driver_reset_resume(hid);
}

/*
 * Check that the device exists and parse the HID descriptor.
 */
static int __i2c_hid_core_probe(struct i2c_hid *ihid)
{
	struct i2c_client *client = ihid->client;
	struct hid_device *hid = ihid->hid;
	int ret, retries = 0;

	/*
	 * Eve touchpad does not consistently ACK the slave address.
	 * Retry the SMBUS sequence as a mitigation until proper fix
	 * is prepared.
	 * This is a temporary workaround for b/156232671.
	 */
	if (client->addr == EVE_TP_I2C_ADDR)
		retries = EVE_TP_RETRIES;

	/* Make sure there is something at this address */
	ret = i2c_smbus_read_byte(client);
	while ((ret < 0) && (retries--)) {
		dev_err(&client->dev, "no response, retry left %d", retries);
		msleep(EVE_TP_DELAY_MS);
		ret = i2c_smbus_read_byte(client);
	}
	if (ret < 0) {
		i2c_hid_dbg(ihid, "nothing at this address: %d\n", ret);
		return -ENXIO;
	}

	ret = i2c_hid_fetch_hid_descriptor(ihid);
	if (ret < 0) {
		dev_err(&client->dev,
			"Failed to fetch the HID Descriptor\n");
		return ret;
	}

	hid->version = le16_to_cpu(ihid->hdesc.bcdVersion);
	hid->vendor = le16_to_cpu(ihid->hdesc.wVendorID);
	hid->product = le16_to_cpu(ihid->hdesc.wProductID);

	hid->initial_quirks |= i2c_hid_get_dmi_quirks(hid->vendor,
						      hid->product);

	snprintf(hid->name, sizeof(hid->name), "%s %04X:%04X",
		 client->name, (u16)hid->vendor, (u16)hid->product);
	strscpy(hid->phys, dev_name(&client->dev), sizeof(hid->phys));

	ihid->quirks = i2c_hid_lookup_quirk(hid->vendor, hid->product);

	return 0;
}

static int i2c_hid_core_register_hid(struct i2c_hid *ihid)
{
	struct i2c_client *client = ihid->client;
	struct hid_device *hid = ihid->hid;
	int ret;

	enable_irq(client->irq);

	ret = hid_add_device(hid);
	if (ret) {
		if (ret != -ENODEV)
			hid_err(client, "can't add hid device: %d\n", ret);
		disable_irq(client->irq);
		return ret;
	}

	return 0;
}

static int i2c_hid_core_probe_panel_follower(struct i2c_hid *ihid)
{
	int ret;

	ret = i2c_hid_core_power_up(ihid);
	if (ret)
		return ret;

	ret = __i2c_hid_core_probe(ihid);
	if (ret)
		goto err_power_down;

	ret = i2c_hid_core_register_hid(ihid);
	if (ret)
		goto err_power_down;

	return 0;

err_power_down:
	i2c_hid_core_power_down(ihid);

	return ret;
}

static void ihid_core_panel_prepare_work(struct work_struct *work)
{
	struct i2c_hid *ihid = container_of(work, struct i2c_hid,
					    panel_follower_prepare_work);
	struct hid_device *hid = ihid->hid;
	int ret;

	/*
	 * hid->version is set on the first power up. If it's still zero then
	 * this is the first power on so we should perform initial power up
	 * steps.
	 */
	if (!hid->version)
		ret = i2c_hid_core_probe_panel_follower(ihid);
	else
		ret = i2c_hid_core_resume(ihid);

	if (ret)
		dev_warn(&ihid->client->dev, "Power on failed: %d\n", ret);
	else
		WRITE_ONCE(ihid->prepare_work_finished, true);

	/*
	 * The work APIs provide a number of memory ordering guarantees
	 * including one that says that memory writes before schedule_work()
	 * are always visible to the work function, but they don't appear to
	 * guarantee that a write that happened in the work is visible after
	 * cancel_work_sync(). We'll add a write memory barrier here to match
	 * with i2c_hid_core_panel_unpreparing() to ensure that our write to
	 * prepare_work_finished is visible there.
	 */
	smp_wmb();
}

static int i2c_hid_core_panel_prepared(struct drm_panel_follower *follower)
{
	struct i2c_hid *ihid = container_of(follower, struct i2c_hid, panel_follower);

	/*
	 * Powering on a touchscreen can be a slow process. Queue the work to
	 * the system workqueue so we don't block the panel's power up.
	 */
	WRITE_ONCE(ihid->prepare_work_finished, false);
	schedule_work(&ihid->panel_follower_prepare_work);

	return 0;
}

static int i2c_hid_core_panel_unpreparing(struct drm_panel_follower *follower)
{
	struct i2c_hid *ihid = container_of(follower, struct i2c_hid, panel_follower);

	cancel_work_sync(&ihid->panel_follower_prepare_work);

	/* Match with ihid_core_panel_prepare_work() */
	smp_rmb();
	if (!READ_ONCE(ihid->prepare_work_finished))
		return 0;

	return i2c_hid_core_suspend(ihid, true);
}

static const struct drm_panel_follower_funcs i2c_hid_core_panel_follower_funcs = {
	.panel_prepared = i2c_hid_core_panel_prepared,
	.panel_unpreparing = i2c_hid_core_panel_unpreparing,
};

static int i2c_hid_core_register_panel_follower(struct i2c_hid *ihid)
{
	struct device *dev = &ihid->client->dev;
	int ret;

	ihid->panel_follower.funcs = &i2c_hid_core_panel_follower_funcs;

	/*
	 * If we're not in control of our own power up/power down then we can't
	 * do the logic to manage wakeups. Give a warning if a user thought
	 * that was possible then force the capability off.
	 */
	if (device_can_wakeup(dev)) {
		dev_warn(dev, "Can't wakeup if following panel\n");
		device_set_wakeup_capable(dev, false);
	}

	ret = drm_panel_add_follower(dev, &ihid->panel_follower);
	if (ret)
		return ret;

	return 0;
}

int i2c_hid_core_probe(struct i2c_client *client, struct i2chid_ops *ops,
		       u16 hid_descriptor_address, u32 quirks)
{
	int ret;
	struct i2c_hid *ihid;
	struct hid_device *hid;

	dbg_hid("HID probe called for i2c 0x%02x\n", client->addr);

	if (!client->irq) {
		dev_err(&client->dev,
			"HID over i2c has not been provided an Int IRQ\n");
		return -EINVAL;
	}

	if (client->irq < 0) {
		if (client->irq != -EPROBE_DEFER)
			dev_err(&client->dev,
				"HID over i2c doesn't have a valid IRQ\n");
		return client->irq;
	}

	ihid = devm_kzalloc(&client->dev, sizeof(*ihid), GFP_KERNEL);
	if (!ihid)
		return -ENOMEM;

	i2c_set_clientdata(client, ihid);

	ihid->ops = ops;
	ihid->client = client;
	ihid->wHIDDescRegister = cpu_to_le16(hid_descriptor_address);
	ihid->is_panel_follower = drm_is_panel_follower(&client->dev);

	init_waitqueue_head(&ihid->wait);
	mutex_init(&ihid->reset_lock);
	INIT_WORK(&ihid->panel_follower_prepare_work, ihid_core_panel_prepare_work);

	/* we need to allocate the command buffer without knowing the maximum
	 * size of the reports. Let's use HID_MIN_BUFFER_SIZE, then we do the
	 * real computation later. */
	ret = i2c_hid_alloc_buffers(ihid, HID_MIN_BUFFER_SIZE);
	if (ret < 0)
		return ret;
	device_enable_async_suspend(&client->dev);

	hid = hid_allocate_device();
	if (IS_ERR(hid)) {
		ret = PTR_ERR(hid);
		goto err_free_buffers;
	}

	ihid->hid = hid;

	hid->driver_data = client;
	hid->ll_driver = &i2c_hid_ll_driver;
	hid->dev.parent = &client->dev;
	hid->bus = BUS_I2C;
	hid->initial_quirks = quirks;

	/* Power on and probe unless device is a panel follower. */
	if (!ihid->is_panel_follower) {
		ret = i2c_hid_core_power_up(ihid);
		if (ret < 0)
			goto err_destroy_device;

		ret = __i2c_hid_core_probe(ihid);
		if (ret < 0)
			goto err_power_down;
	}

	ret = i2c_hid_init_irq(client);
	if (ret < 0)
		goto err_power_down;

	/*
	 * If we're a panel follower, we'll register when the panel turns on;
	 * otherwise we do it right away.
	 */
	if (ihid->is_panel_follower)
		ret = i2c_hid_core_register_panel_follower(ihid);
	else
		ret = i2c_hid_core_register_hid(ihid);
	if (ret)
		goto err_free_irq;

	return 0;

err_free_irq:
	free_irq(client->irq, ihid);
err_power_down:
	if (!ihid->is_panel_follower)
		i2c_hid_core_power_down(ihid);
err_destroy_device:
	hid_destroy_device(hid);
err_free_buffers:
	i2c_hid_free_buffers(ihid);

	return ret;
}
EXPORT_SYMBOL_GPL(i2c_hid_core_probe);

int i2c_hid_core_remove(struct i2c_client *client)
{
	struct i2c_hid *ihid = i2c_get_clientdata(client);
	struct hid_device *hid;

	/*
	 * If we're a follower, the act of unfollowing will cause us to be
	 * powered down. Otherwise we need to manually do it.
	 */
	if (ihid->is_panel_follower)
		drm_panel_remove_follower(&ihid->panel_follower);
	else
		i2c_hid_core_suspend(ihid, true);

	hid = ihid->hid;
	hid_destroy_device(hid);

	free_irq(client->irq, ihid);

	if (ihid->bufsize)
		i2c_hid_free_buffers(ihid);

	return 0;
}
EXPORT_SYMBOL_GPL(i2c_hid_core_remove);

void i2c_hid_core_shutdown(struct i2c_client *client)
{
	struct i2c_hid *ihid = i2c_get_clientdata(client);

	i2c_hid_set_power(ihid, I2C_HID_PWR_SLEEP);
	free_irq(client->irq, ihid);

	i2c_hid_core_shutdown_tail(ihid);
}
EXPORT_SYMBOL_GPL(i2c_hid_core_shutdown);

static int i2c_hid_core_pm_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct i2c_hid *ihid = i2c_get_clientdata(client);

	if (ihid->is_panel_follower)
		return 0;

	return i2c_hid_core_suspend(ihid, false);
}

static int i2c_hid_core_pm_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct i2c_hid *ihid = i2c_get_clientdata(client);

	if (ihid->is_panel_follower)
		return 0;

	return i2c_hid_core_resume(ihid);
}

const struct dev_pm_ops i2c_hid_core_pm = {
	SYSTEM_SLEEP_PM_OPS(i2c_hid_core_pm_suspend, i2c_hid_core_pm_resume)
};
EXPORT_SYMBOL_GPL(i2c_hid_core_pm);

MODULE_DESCRIPTION("HID over I2C core driver");
MODULE_AUTHOR("Benjamin Tissoires <benjamin.tissoires@gmail.com>");
MODULE_LICENSE("GPL");
