// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * corsair-cpro-setconf.c - Small tool for setting fan-config on Corsair Commander Pro
 * Copyright (C) 2022 Marius Zachmann <mail@mariuszachmann.de>
 */
#include <getopt.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <hidapi/hidapi.h>
#include "polyfills.h"

/* Polyfills */
typedef unsigned char u8;
typedef unsigned short u16;
typedef short s16;
#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))

#define USB_VENDOR_ID_CORSAIR 0x1b1c
#define USB_PRODUCT_ID_CORSAIR_COMMANDER_CORE_XT 0x0c2a

/**
 * The maximum number of fans and temperature sensors supported by the driver.
 */
#define NUM_FANS 6
#define NUM_TEMP_SENSORS 4

#define REQ_TIMEOUT 300
#define OUT_BUFFER_SIZE 385
#define IN_BUFFER_SIZE 384
#define LABEL_LENGTH 11

#define CMD_HEADER_SIZE 2
#define WRITE_DATA_HEADER_SIZE 4

#define FAN_CNT_INDEX 5
#define FAN_DATA_OFFSET 6

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
	hid_device *hdev;
	u8 *cmd_buffer;
	u8 *buffer;
	u8 *data_buffer;
	int buffer_recv_size; /* number of received bytes in buffer */
	int data_buffer_recv_size; /* number of received bytes in data_buffer */
	struct firmware_version firmware_ver;
	u8 bootloader_ver[2];
	int target[NUM_FANS];
	DECLARE_BITMAP(temp_cnct, NUM_TEMP_SENSORS);
	DECLARE_BITMAP(fan_cnct, NUM_FANS);
	char fan_label[NUM_FANS][LABEL_LENGTH];
	void *mutex;
};

void clear_buffer(u8 *buffer, const size_t buffer_len)
{
	for (int i = 0; i < buffer_len; i++)
		buffer[i] = 0x00;
}

void print_buffer(const u8 *buffer, const int length)
{
	int x;

	for (x = 0; x < length; x++) {
		fprintf(stderr, "%02x ", buffer[x]);
		if ((x + 1) % 16 == 0 && (x + 1) != length)
			printf("\n");
	}
	fprintf(stderr, "\n-DEZ:-\n");
	for (x = 0; x < length; x++) {
		fprintf(stderr, "%d ", buffer[x]);
		if ((x + 1) % 16 == 0 && (x + 1) != length)
			printf("\n");
	}
	printf("\n--------------------\n");
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

static int send_usb(struct ccxt_device *ccp)
{
	int res;

	res = hid_write(ccp->hdev, ccp->cmd_buffer, OUT_BUFFER_SIZE);
	if (res == -1) {
		fprintf(stderr, "Could not write to device\n");
		return 1;
	}

	res = hid_read(ccp->hdev, ccp->buffer, IN_BUFFER_SIZE);
	if (res == -1) {
		fprintf(stderr, "Could not read from device\n");
		return -1;
	}
	ccp->buffer_recv_size = res;

	return 0;
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
	int ret, num_sensors, channel;

	ret = read_data(ccxt, endpoint_get_temperatures);
	if (ret)
		return ret;

	/* The theoretical number of temperature sensors this controller supports */
	num_sensors = ccxt->data_buffer[FAN_CNT_INDEX];

	for (
		channel = 0;
		channel < min(num_sensors, NUM_TEMP_SENSORS);
		channel++
	) {
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

int main(void)
{
	u8 cmd_buffer[OUT_BUFFER_SIZE];
	u8 buffer[IN_BUFFER_SIZE];
	u8 data_buffer[IN_BUFFER_SIZE];

	struct ccxt_device ccxt = {
		.cmd_buffer = cmd_buffer,
		.buffer = buffer,
		.data_buffer = data_buffer,
	};

	int res = hid_init();
	if (res) {
		fprintf(stderr, "Could not initialize hid_api\n");
		return 1;
	}

	ccxt.hdev = hid_open(USB_VENDOR_ID_CORSAIR,
				USB_PRODUCT_ID_CORSAIR_COMMANDER_CORE_XT, NULL);
	if (ccxt.hdev == NULL) {
		fprintf(stderr, "Could not find device\n");
		return 1;
	}

	res = get_fw_version(&ccxt);
	if (res)
		goto close;

	fprintf(stdout, "FW: %u.%u.%u\n", ccxt.firmware_ver.major,
		ccxt.firmware_ver.minor, ccxt.firmware_ver.patch);

	res = set_software_mode(&ccxt);
	if (res)
		goto close;

	res = get_temp_cnct(&ccxt);
	print_buffer(ccxt.data_buffer, ccxt.data_buffer_recv_size);

	/* res = set_hardware_mode(&ccxt);
	if (res)
		goto close;*/

close:
	hid_close(ccxt.hdev);

	return 0;
}