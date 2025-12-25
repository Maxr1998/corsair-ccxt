// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * corsair-ccxt.c - Linux driver for Corsair Commander Pro
 * Copyright (C) 2020 Marius Zachmann <mail@mariuszachmann.de>
 * Copyright (C) 2025 Max Rumpf <kernel@maxr1998.de>
 *
 * This driver uses hid reports to communicate with the device to allow hidraw userspace drivers
 * still being used. The device does not use report ids. When using hidraw and this driver
 * simultaneously, reports could be switched.
 */

#include <linux/bitops.h>
#include <linux/completion.h>
#include <linux/debugfs.h>
#include <linux/hid.h>
#include <linux/hwmon.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/types.h>

// ReSharper disable CppJoinDeclarationAndAssignment CppLocalVariableMayBeConst CppParameterMayBeConst CppParameterMayBeConstPtrOrRef

#define USB_VENDOR_ID_CORSAIR 0x1b1c
#define USB_PRODUCT_ID_CORSAIR_COMMANDER_CORE_XT 0x0c2a

/*
TODO:
 - add function to query current mode (hardware or software control) and expose it somehow.
 - check whether we can set fan target (RPM) as an alternative
 - automatically switch to software mode when writing pwm or rpm values (on errors only)?
*/

/**
 * The maximum number of fans and temperature sensors supported by the driver.
 */
#define NUM_FANS 6
#define NUM_TEMP_SENSORS 2

#define REQ_TIMEOUT 300
#define OUT_BUFFER_SIZE 385
#define IN_BUFFER_SIZE 384
#define LABEL_LENGTH 11

#define CMD_HEADER_SIZE 2
#define WRITE_DATA_HEADER_SIZE 4

#define FAN_CNT_INDEX 5
#define FAN_DATA_OFFSET 6

#define TEMP_CNT_INDEX 5
#define TEMP_DATA_OFFSET 6

#define FAN_STATE_OK 0x07

#define prepare_cmd_safe(ccxt, cmd) \
	({ prepare_cmd(ccxt, cmd, ARRAY_SIZE(cmd)); })

#define prepare_endpoint_cmd_safe(ccxt, cmd, endpoint) \
	({ prepare_endpoint_cmd(ccxt, cmd, ARRAY_SIZE(cmd), endpoint); })

/**
 * Returns the firmware version as four bytes (patch version uses two bytes)
 */
static const u8 cmd_get_firmware[] = { 0x02, 0x13 };
static const u8 cmd_hardware_mode[] = { 0x01, 0x03, 0x00, 0x01 };
static const u8 cmd_software_mode[] = { 0x01, 0x03, 0x00, 0x02 };
static const u8 cmd_open_endpoint[] = { 0x0d, 0x01 };
static const u8 cmd_close_endpoint[] = { 0x05, 0x01, 0x01 };
static const u8 cmd_write[] = { 0x06, 0x01 };
static const u8 cmd_read[] = { 0x08, 0x01 };

/**
 * Endpoint to query the fan speed of all connected fans.
 */
static const u8 endpoint_fan_state = 0x17;

/**
 * Endpoint to set the fan PWM of one or multiple fans by id.
 */
static const u8 endpoint_fan_pwm = 0x18;

/**
 * Endpoint to query the number of total supported fans and the connection state for each.
 */
static const u8 endpoint_get_fans = 0x1a;

/**
 * Endpoint to query the number of total supported temperature sensors
 * and the temperature reported by each connected sensor.
 */
static const u8 endpoint_get_temperatures = 0x21;

static const u8 data_type_set_speed[] = { 0x07, 0x00 };

struct firmware_version {
	u8 major;
	u8 minor;
	u16 patch;
};

struct ccxt_device {
	struct hid_device *hdev;
	struct device *hwmon_dev;
	struct dentry *debugfs;
	/* For reinitializing the completion below */
	spinlock_t wait_input_report_lock;
	struct completion wait_input_report;
	struct mutex mutex;
	/* whenever buffer is used, lock before send_usb_cmd */
	u8 *cmd_buffer;
	u8 *buffer;
	/* required to store result of multistep get_data */
	u8 *data_buffer;
	int buffer_recv_size; /* number of received bytes in buffer */
	int data_buffer_recv_size; /* number of received bytes in data_buffer */
	int target[NUM_FANS];
	DECLARE_BITMAP(temp_cnct, NUM_TEMP_SENSORS);
	DECLARE_BITMAP(fan_cnct, NUM_FANS);
	char fan_label[NUM_FANS][LABEL_LENGTH];
	struct firmware_version firmware_ver;
	u8 bootloader_ver[2];
};

/* converts response error in buffer to errno */
static int ccxt_get_errno(struct ccxt_device *ccxt)
{
	switch (ccxt->buffer[0]) {
	case 0x00: /* success */
		return 0;
	case 0x01: /* called invalid command */
		return -EOPNOTSUPP;
	case 0x10: /* called GET_VOLT / GET_TMP with invalid arguments */
		return -EINVAL;
	case 0x11: /* requested temps of disconnected sensors */
	case 0x12: /* requested pwm of not pwm-controlled channels */
		return -ENODATA;
	default:
		hid_dbg(ccxt->hdev, "unknown device response error: %d",
			ccxt->buffer[0]);
		return -EIO;
	}
}

/* prepare ccxt->cmd_buffer with command and return used size */
static int prepare_cmd(struct ccxt_device *ccxt, const u8 *command,
			size_t command_len)
{
	memset(ccxt->cmd_buffer, 0x00, OUT_BUFFER_SIZE);
	ccxt->cmd_buffer[0] = 0x00;
	ccxt->cmd_buffer[1] = 0x08;
	memcpy(ccxt->cmd_buffer + CMD_HEADER_SIZE, command, command_len);

	return CMD_HEADER_SIZE + command_len;
}

/* prepare ccxt->cmd_buffer with command and single-valued endpoint and return used size */
static int prepare_endpoint_cmd(struct ccxt_device *ccxt, const u8 *command,
				size_t command_len, u8 endpoint)
{
	int ret = prepare_cmd(ccxt, command, command_len);
	ccxt->cmd_buffer[ret] = endpoint;
	return ret + 1;
}

/* send current ccxt->cmd_buffer, check for error in response, response in ccxt->buffer */
static int send_usb(struct ccxt_device *ccxt)
{
	unsigned long t;
	int ret;

	/*
	 * Disable raw event parsing for a moment to safely reinitialize the
	 * completion. Reinit is done because hidraw could have triggered
	 * the raw event parsing and marked the ccxt->wait_input_report
	 * completion as done.
	 */
	spin_lock_bh(&ccxt->wait_input_report_lock);
	reinit_completion(&ccxt->wait_input_report);
	spin_unlock_bh(&ccxt->wait_input_report_lock);

	ret = hid_hw_output_report(ccxt->hdev, ccxt->cmd_buffer,
				OUT_BUFFER_SIZE);
	if (ret < 0)
		return ret;

	t = wait_for_completion_timeout(&ccxt->wait_input_report,
					msecs_to_jiffies(REQ_TIMEOUT));
	if (!t)
		return -ETIMEDOUT;

	if (ccxt->buffer_recv_size != IN_BUFFER_SIZE)
		return -EPROTO;

	return ccxt_get_errno(ccxt);
}

static int ccxt_raw_event(struct hid_device *hdev, struct hid_report *report,
			u8 *data, int size)
{
	struct ccxt_device *ccxt = hid_get_drvdata(hdev);

	/* only copy buffer when requested */
	spin_lock(&ccxt->wait_input_report_lock);
	if (!completion_done(&ccxt->wait_input_report)) {
		memcpy(ccxt->buffer, data, min(IN_BUFFER_SIZE, size));
		ccxt->buffer_recv_size = size;
		complete_all(&ccxt->wait_input_report);
	}
	spin_unlock(&ccxt->wait_input_report_lock);

	return 0;
}

static int set_hardware_mode(struct ccxt_device *ccxt)
{
	int ret;

	mutex_lock(&ccxt->mutex);

	prepare_cmd_safe(ccxt, cmd_hardware_mode);
	ret = send_usb(ccxt);

	mutex_unlock(&ccxt->mutex);
	return ret;
}

static int set_software_mode(struct ccxt_device *ccxt)
{
	int ret;

	mutex_lock(&ccxt->mutex);

	prepare_cmd_safe(ccxt, cmd_software_mode);
	ret = send_usb(ccxt);

	mutex_unlock(&ccxt->mutex);
	return ret;
}

/* read firmware version */
static int get_fw_version(struct ccxt_device *ccxt)
{
	int ret;

	mutex_lock(&ccxt->mutex);

	prepare_cmd_safe(ccxt, cmd_get_firmware);
	ret = send_usb(ccxt);

	if (ret) {
		hid_notice(ccxt->hdev, "failed to read firmware version.\n");
		goto out_unlock;
	}
	ccxt->firmware_ver.major = ccxt->buffer[3];
	ccxt->firmware_ver.minor = ccxt->buffer[4];
	ccxt->firmware_ver.patch = (u16)ccxt->buffer[5] | (u16)ccxt->buffer[6]
				<< 8;

out_unlock:
	mutex_unlock(&ccxt->mutex);
	return ret;
}

/* read bootloader version */
static int get_bl_version(struct ccxt_device *ccxt)
{
	int ret;

	return -1;

	// TODO: implement bootloader version readout
	/*ret = send_usb(ccxt);
	if (ret) {
		hid_notice(ccxt->hdev, "Failed to read bootloader version.\n");
		return ret;
	}
	ccxt->bootloader_ver[0] = ccxt->buffer[1];
	ccxt->bootloader_ver[1] = ccxt->buffer[2];

	return 0;*/
}

/* reads the data from the given endpoint and stores it in data_buffer */
static int read_data(struct ccxt_device *ccxt, u8 endpoint)
{
	int ret, i;

	mutex_lock(&ccxt->mutex);

	prepare_endpoint_cmd_safe(ccxt, cmd_close_endpoint, endpoint);
	ret = send_usb(ccxt);
	if (ret)
		goto out_unlock;

	prepare_endpoint_cmd_safe(ccxt, cmd_open_endpoint, endpoint);
	ret = send_usb(ccxt);
	if (ret)
		goto out_unlock;

	prepare_endpoint_cmd_safe(ccxt, cmd_read, endpoint);
	ret = send_usb(ccxt);
	if (ret)
		goto out_unlock;

	/* copy result to data buffer */
	memcpy(ccxt->data_buffer, ccxt->buffer, IN_BUFFER_SIZE);
	ccxt->data_buffer_recv_size = ccxt->buffer_recv_size;

	prepare_endpoint_cmd_safe(ccxt, cmd_close_endpoint, endpoint);
	ret = send_usb(ccxt);

out_unlock:
	mutex_unlock(&ccxt->mutex);
	return ret;
}

static int write_data(struct ccxt_device *ccxt, u8 endpoint,
			const u8 *data_type, size_t data_type_size,
			const u8 *data, size_t data_size)
{
	int ret;
	u8 *writer_header_dst, *data_type_dst, *data_dst;

	mutex_lock(&ccxt->mutex);

	prepare_endpoint_cmd_safe(ccxt, cmd_close_endpoint, endpoint);
	ret = send_usb(ccxt);
	if (ret)
		goto out_unlock;

	prepare_endpoint_cmd_safe(ccxt, cmd_open_endpoint, endpoint);
	ret = send_usb(ccxt);
	if (ret)
		goto out_unlock;

	ret = prepare_cmd_safe(ccxt, cmd_write);

	/* compute header offsets */
	writer_header_dst = ccxt->cmd_buffer + ret;
	data_type_dst = writer_header_dst + WRITE_DATA_HEADER_SIZE;
	data_dst = data_type_dst + data_type_size;

	writer_header_dst[0] = data_type_size + data_size;
	memcpy(data_type_dst, data_type, data_type_size);
	memcpy(data_dst, data, data_size);

	ret = send_usb(ccxt);
	if (ret)
		goto out_unlock;

	/* copy result to data buffer */
	memcpy(ccxt->data_buffer, ccxt->buffer, IN_BUFFER_SIZE);

	prepare_endpoint_cmd_safe(ccxt, cmd_close_endpoint, endpoint);
	ret = send_usb(ccxt);

out_unlock:
	mutex_unlock(&ccxt->mutex);
	return ret;
}

/* read fan connection status and set labels */
static int get_fan_cnct(struct ccxt_device *ccxt)
{
	int ret, num_fans, channel, state;

	ret = read_data(ccxt, endpoint_get_fans);
	if (ret)
		return ret;

	/* The theoretical number of fans this controller supports */
	num_fans = ccxt->data_buffer[FAN_CNT_INDEX];

	for (channel = 0; channel < min(num_fans, NUM_FANS); channel++) {
		state = ccxt->data_buffer[FAN_DATA_OFFSET + channel];
		if (state != FAN_STATE_OK)
			continue;

		set_bit(channel, ccxt->fan_cnct);
		ccxt->target[channel] = -ENODATA;

		scnprintf(ccxt->fan_label[channel], LABEL_LENGTH, "fan%d",
			channel + 1);
	}

	return 0;
}

/* read temp sensor connection status */
static int get_temp_cnct(struct ccxt_device *ccxt)
{
	int ret, num_sensors, channel, state;

	ret = read_data(ccxt, endpoint_get_temperatures);
	if (ret)
		return ret;

	/* The theoretical number of temperature sensors this controller supports */
	num_sensors = ccxt->data_buffer[TEMP_CNT_INDEX];

	for (
		channel = 0;
		channel < min(num_sensors, NUM_TEMP_SENSORS);
		channel++
	) {
		state = ccxt->data_buffer[TEMP_DATA_OFFSET + channel];
		/*mode = ccxt->buffer[channel + 1];
		if (mode == 0)
			continue;

		set_bit(channel, ccxt->temp_cnct);*/
	}

	return -1;
}

static int get_fan_rpm(struct ccxt_device *ccxt, int channel, long *val)
{
	int ret, num_fans, data_index;

	ret = read_data(ccxt, endpoint_fan_state);
	if (ret)
		return ret;

	num_fans = ccxt->data_buffer[FAN_CNT_INDEX];

	if (channel >= min(num_fans, NUM_FANS)) {
		hid_notice(ccxt->hdev, "invalid fan channel %d\n", channel);
		return -EINVAL;
	}

	/* two bytes per value */
	data_index = FAN_DATA_OFFSET + channel * 2;

	*val = (s16)((u16)ccxt->data_buffer[data_index] |
			(u16)ccxt->data_buffer[data_index + 1] << 8);

	hid_notice(ccxt->hdev, "fan%d rpm changed to %ld\n", channel, *val);

	return 0;
}

static int get_fan_pwm(struct ccxt_device *ccxt, int channel, long *val)
{
	int ret, num_fans, data_index, id, pwm;

	ret = read_data(ccxt, endpoint_fan_pwm);
	if (ret)
		return ret;

	num_fans = ccxt->data_buffer[FAN_CNT_INDEX];

	if (channel >= min(num_fans, NUM_FANS)) {
		hid_notice(ccxt->hdev, "invalid fan channel %d\n", channel);
		return -EINVAL;
	}

	data_index = FAN_DATA_OFFSET + channel * 4;

	/* validate channel id from response */
	id = ccxt->data_buffer[data_index];
	if (id != channel) {
		hid_notice(ccxt->hdev,
			"invalid fan id %d in response for channel %d\n", id,
			channel);
		return -EIO;
	}

	pwm = ccxt->data_buffer[data_index + 2];

	*val = DIV_ROUND_CLOSEST(pwm * 255, 100);

	hid_notice(ccxt->hdev, "fan%d pwm changed to %ld\n", channel, *val);

	return 0;
}

static int set_pwm(struct ccxt_device *ccxt, int channel, long val)
{
	int ret;

	if (val < 0 || val > 255)
		return -EINVAL;

	/* Corsair uses values from 0-100 */
	val = DIV_ROUND_CLOSEST(val * 100, 255);

	/* {count, id, mode, val, 0x00} */
	const u8 speed_cmd[5] = { 1, channel, 0, val, 0x00 };
	ret = write_data(ccxt, endpoint_fan_pwm, data_type_set_speed,
			sizeof(data_type_set_speed), speed_cmd,
			sizeof(speed_cmd));
	if (!ret)
		ccxt->target[channel] = -ENODATA;

	hid_notice(ccxt->hdev, "fan%d pwm set to %ld\n", channel, val);

	return ret;
}

static int set_target(struct ccxt_device *ccxt, int channel, long val)
{
	int ret;

	val = clamp_val(val, 0, 0xFFFF);
	ccxt->target[channel] = val;

	mutex_lock(&ccxt->mutex);
	ret = -1;
	// send_usb_cmd(ccxt, CTL_SET_FAN_TARGET, channel, val >> 8, val);

	mutex_unlock(&ccxt->mutex);
	return ret;
}

static int ccxt_read_string(struct device *dev, enum hwmon_sensor_types type,
			u32 attr, int channel, const char **str)
{
	struct ccxt_device *ccxt = dev_get_drvdata(dev);

	switch (type) {
	case hwmon_fan:
		switch (attr) {
		case hwmon_fan_label:
			*str = ccxt->fan_label[channel];
			return 0;
		default:
			break;
		}
		break;
	default:
		break;
	}

	return -EOPNOTSUPP;
}

static int ccxt_read(struct device *dev, enum hwmon_sensor_types type, u32 attr,
			int channel, long *val)
{
	struct ccxt_device *ccxt = dev_get_drvdata(dev);
	int ret;

	switch (type) {
	case hwmon_temp:
		switch (attr) {
		case hwmon_temp_input:
			ret = 0; // get_data(ccxt, CTL_GET_TMP, channel, true);
			if (ret < 0)
				return ret;
			*val = (s16)ret * 10;
			return 0;
		default:
			break;
		}
		break;
	case hwmon_fan:
		switch (attr) {
		case hwmon_fan_input:
			ret = get_fan_rpm(ccxt, channel, val);
			if (ret < 0)
				return ret;
			return 0;
		case hwmon_fan_target:
			/* how to read target values from the device is unknown */
			/* driver returns last set value or 0			*/
			if (ccxt->target[channel] < 0)
				return -ENODATA;
			*val = ccxt->target[channel];
			return 0;
		default:
			break;
		}
		break;
	case hwmon_pwm:
		switch (attr) {
		case hwmon_pwm_input:
			ret = get_fan_pwm(ccxt, channel, val);
			if (ret < 0)
				return ret;
			return 0;
		default:
			break;
		}
		break;
	case hwmon_in:
		switch (attr) {
		case hwmon_in_input:
			ret = 0; // get_data(ccxt, CTL_GET_VOLT, channel, true);
			if (ret < 0)
				return ret;
			*val = ret;
			return 0;
		default:
			break;
		}
		break;
	default:
		break;
	}

	return -EOPNOTSUPP;
};

static int ccxt_write(struct device *dev, enum hwmon_sensor_types type,
			u32 attr, int channel, long val)
{
	struct ccxt_device *ccxt = dev_get_drvdata(dev);

	switch (type) {
	case hwmon_pwm:
		switch (attr) {
		case hwmon_pwm_input:
			return set_pwm(ccxt, channel, val);
		default:
			break;
		}
		break;
	case hwmon_fan:
		switch (attr) {
		case hwmon_fan_target:
			return set_target(ccxt, channel, val);
		default:
			break;
		}
		break;
	default:
		break;
	}

	return -EOPNOTSUPP;
};

static umode_t ccxt_is_visible(const void *data, enum hwmon_sensor_types type,
				u32 attr, int channel)
{
	const struct ccxt_device *ccxt = data;

	switch (type) {
	case hwmon_temp:
		if (!test_bit(channel, ccxt->temp_cnct))
			break;

		switch (attr) {
		case hwmon_temp_input:
			return 0444;
		case hwmon_temp_label:
			return 0444;
		default:
			break;
		}
		break;
	case hwmon_fan:
		if (!test_bit(channel, ccxt->fan_cnct))
			break;

		switch (attr) {
		case hwmon_fan_input:
			return 0444;
		case hwmon_fan_label:
			return 0444;
		case hwmon_fan_target:
			return 0644;
		default:
			break;
		}
		break;
	case hwmon_pwm:
		if (!test_bit(channel, ccxt->fan_cnct))
			break;

		switch (attr) {
		case hwmon_pwm_input:
			return 0644;
		default:
			break;
		}
		break;
	case hwmon_in:
		switch (attr) {
		case hwmon_in_input:
			return 0444;
		default:
			break;
		}
		break;
	default:
		break;
	}

	return 0;
};

static const struct hwmon_ops ccxt_hwmon_ops = {
	.is_visible = ccxt_is_visible,
	.read = ccxt_read,
	.read_string = ccxt_read_string,
	.write = ccxt_write,
};

static const struct hwmon_channel_info *const ccxt_info[] = {
	HWMON_CHANNEL_INFO(chip, HWMON_C_REGISTER_TZ),
	HWMON_CHANNEL_INFO(temp, HWMON_T_INPUT, HWMON_T_INPUT, HWMON_T_INPUT,
			HWMON_T_INPUT),
	HWMON_CHANNEL_INFO(fan, HWMON_F_INPUT | HWMON_F_LABEL | HWMON_F_TARGET,
			HWMON_F_INPUT | HWMON_F_LABEL | HWMON_F_TARGET,
			HWMON_F_INPUT | HWMON_F_LABEL | HWMON_F_TARGET,
			HWMON_F_INPUT | HWMON_F_LABEL | HWMON_F_TARGET,
			HWMON_F_INPUT | HWMON_F_LABEL | HWMON_F_TARGET,
			HWMON_F_INPUT | HWMON_F_LABEL | HWMON_F_TARGET),
	HWMON_CHANNEL_INFO(pwm, HWMON_PWM_INPUT, HWMON_PWM_INPUT,
			HWMON_PWM_INPUT, HWMON_PWM_INPUT, HWMON_PWM_INPUT,
			HWMON_PWM_INPUT),
	HWMON_CHANNEL_INFO(in, HWMON_I_INPUT, HWMON_I_INPUT, HWMON_I_INPUT),
	NULL
};

static const struct hwmon_chip_info ccxt_chip_info = {
	.ops = &ccxt_hwmon_ops,
	.info = ccxt_info,
};

static int firmware_show(struct seq_file *seqf, void *unused)
{
	const struct ccxt_device *ccxt = seqf->private;

	seq_printf(seqf, "%d.%d.%d\n", ccxt->firmware_ver.major,
		ccxt->firmware_ver.minor, ccxt->firmware_ver.patch);

	return 0;
}

DEFINE_SHOW_ATTRIBUTE(firmware);

static int bootloader_show(struct seq_file *seqf, void *unused)
{
	struct ccxt_device *ccxt = seqf->private;

	seq_printf(seqf, "%d.%d\n", ccxt->bootloader_ver[0],
		ccxt->bootloader_ver[1]);

	return 0;
}

DEFINE_SHOW_ATTRIBUTE(bootloader);

static void ccxt_debugfs_init(struct ccxt_device *ccxt)
{
	char name[32];
	int ret;

	scnprintf(name, sizeof(name), "corsairccxt-%s",
		dev_name(&ccxt->hdev->dev));
	ccxt->debugfs = debugfs_create_dir(name, NULL);

	ret = get_fw_version(ccxt);
	if (!ret)
		debugfs_create_file("firmware_version", 0444, ccxt->debugfs,
			ccxt, &firmware_fops);

	ret = get_bl_version(ccxt);
	if (!ret)
		debugfs_create_file("bootloader_version", 0444, ccxt->debugfs,
			ccxt, &bootloader_fops);
}

static int ccxt_probe(struct hid_device *hdev, const struct hid_device_id *id)
{
	struct ccxt_device *ccxt;
	int ret;

	ccxt = devm_kzalloc(&hdev->dev, sizeof(*ccxt), GFP_KERNEL);
	if (!ccxt)
		return -ENOMEM;

	ccxt->cmd_buffer =
		devm_kmalloc(&hdev->dev, OUT_BUFFER_SIZE, GFP_KERNEL);
	if (!ccxt->cmd_buffer)
		return -ENOMEM;

	ccxt->buffer = devm_kmalloc(&hdev->dev, IN_BUFFER_SIZE, GFP_KERNEL);
	if (!ccxt->buffer)
		return -ENOMEM;

	ccxt->data_buffer =
		devm_kmalloc(&hdev->dev, IN_BUFFER_SIZE, GFP_KERNEL);
	if (!ccxt->data_buffer)
		return -ENOMEM;

	ret = hid_parse(hdev);
	if (ret)
		return ret;

	ret = hid_hw_start(hdev, HID_CONNECT_HIDRAW);
	if (ret)
		return ret;

	ret = hid_hw_open(hdev);
	if (ret)
		goto out_hw_stop;

	ccxt->hdev = hdev;
	hid_set_drvdata(hdev, ccxt);

	mutex_init(&ccxt->mutex);
	spin_lock_init(&ccxt->wait_input_report_lock);
	init_completion(&ccxt->wait_input_report);

	hid_device_io_start(hdev);

	/* required to be able to speak to the controller */
	ret = set_software_mode(ccxt);
	if (ret)
		goto out_hw_close;

	/* fan and temp connection status only updates when the device is powered on */
	ret = get_fan_cnct(ccxt);
	if (ret)
		goto out_hw_close;

	ret = get_temp_cnct(ccxt);
	/*if (ret)
		goto out_hw_close;*/

	ccxt_debugfs_init(ccxt);

	ccxt->hwmon_dev = hwmon_device_register_with_info(
		&hdev->dev, "corsairccxt", ccxt, &ccxt_chip_info, NULL);
	if (IS_ERR(ccxt->hwmon_dev)) {
		ret = PTR_ERR(ccxt->hwmon_dev);
		goto out_hw_close;
	}

	return 0;

out_hw_close:
	hid_hw_close(hdev);
out_hw_stop:
	hid_hw_stop(hdev);
	return ret;
}

static void ccxt_remove(struct hid_device *hdev)
{
	struct ccxt_device *ccxt = hid_get_drvdata(hdev);

	debugfs_remove_recursive(ccxt->debugfs);
	hwmon_device_unregister(ccxt->hwmon_dev);
	set_hardware_mode(ccxt);
	hid_hw_close(hdev);
	hid_hw_stop(hdev);
}

static const struct hid_device_id ccxt_devices[] = {
	{ HID_USB_DEVICE(USB_VENDOR_ID_CORSAIR,
			USB_PRODUCT_ID_CORSAIR_COMMANDER_CORE_XT) },
	{}
};

static struct hid_driver ccxt_driver = {
	.name = "corsair-ccxt",
	.id_table = ccxt_devices,
	.probe = ccxt_probe,
	.remove = ccxt_remove,
	.raw_event = ccxt_raw_event,
};

MODULE_DEVICE_TABLE(hid, ccxt_devices);
MODULE_DESCRIPTION("Corsair Commander Core XT controller driver");
MODULE_LICENSE("GPL");

static int __init ccxt_init(void)
{
	return hid_register_driver(&ccxt_driver);
}

static void __exit ccxt_exit(void)
{
	hid_unregister_driver(&ccxt_driver);
}

/*
 * When compiling this driver as built-in, hwmon initcalls will get called before the
 * hid driver and this driver would fail to register. late_initcall solves this.
 */
late_initcall(ccxt_init);
module_exit(ccxt_exit);