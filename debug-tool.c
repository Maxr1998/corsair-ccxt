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
#include <hidapi/hidapi.h>

#define USB_VENDOR_ID_CORSAIR			0x1b1c
#define USB_PRODUCT_ID_CORSAIR_COMMANDER_CORE_XT	0x0c2a

#define OUT_BUFFER_SIZE		385
#define IN_BUFFER_SIZE		384

typedef unsigned char u8;
typedef unsigned short u16;

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
static const u8 endpoint_get_rpm = 0x17;

/**
 * Endpoint to set the fan PWM of one or multiple fans by id.
 */
static const u8 endpoint_fan_pwm = 0x18;

/**
 * Endpoint to query the number of total supported fans and the connection state for each.
 */
static const u8 endpoint_get_fans = 0x1a;

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
	struct firmware_version firmware_ver;
	u8 bootloader_ver[2];
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

static int send_usb_buffer_cmd(struct ccxt_device *ccp,
                               const u8 *command, const size_t command_len,
                               const size_t buffer_len)
{
	int res;

	clear_buffer(ccp->cmd_buffer, OUT_BUFFER_SIZE);
	ccp->cmd_buffer[0] = 0x00;
	ccp->cmd_buffer[1] = 0x08;
	memcpy(ccp->cmd_buffer + 2, command, command_len);
	memcpy(ccp->cmd_buffer + 2 + command_len, ccp->data_buffer, buffer_len);

	printf("transfer: ");
	for (int i = 0; i < 2 + command_len + buffer_len; i++)
		printf("%02x ", ccp->cmd_buffer[i]);
	printf("\n");

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

static int send_usb_endpoint_cmd(struct ccxt_device *ccp,
                                 const u8 *command, const size_t command_len,
                                 const u8 endpoint)
{
	ccp->data_buffer[0] = endpoint;
	return send_usb_buffer_cmd(ccp, command, command_len, 1);
}

/* requests and returns single data values depending on channel */
static int get_data(struct ccxt_device *ccp, const u8 endpoint)
{
	int ret;

	ret = send_usb_endpoint_cmd(ccp, cmd_close_endpoint,
	                            sizeof(cmd_close_endpoint),
	                            endpoint);
	if (ret)
		goto out_unlock;

	ret = send_usb_endpoint_cmd(ccp, cmd_open_endpoint,
	                            sizeof(cmd_open_endpoint),
	                            endpoint);
	if (ret)
		goto out_unlock;

	ret = send_usb_endpoint_cmd(ccp, cmd_read, sizeof(cmd_read),
	                            endpoint);
	if (ret)
		goto out_unlock;

	memcpy(ccp->data_buffer, ccp->buffer, IN_BUFFER_SIZE);

	ret = send_usb_endpoint_cmd(ccp, cmd_close_endpoint,
	                            sizeof(cmd_close_endpoint),
	                            endpoint);
out_unlock:
	return ret;
}

static int write_data(struct ccxt_device *ccp, u8 endpoint,
                      const u8 *data, size_t size)
{
	int ret;

	ret = send_usb_endpoint_cmd(ccp, cmd_close_endpoint,
	                            sizeof(cmd_close_endpoint),
	                            endpoint);
	if (ret)
		goto out_unlock;

	ret = send_usb_endpoint_cmd(ccp, cmd_open_endpoint,
	                            sizeof(cmd_open_endpoint),
	                            endpoint);
	if (ret)
		goto out_unlock;

	memcpy(ccp->data_buffer, data, size);

	ret = send_usb_buffer_cmd(ccp, cmd_write, sizeof(cmd_write),
	                          size);
	print_buffer(ccp->cmd_buffer, 16);
	if (ret)
		goto out_unlock;

	memcpy(ccp->data_buffer, ccp->buffer, IN_BUFFER_SIZE);

	ret = send_usb_endpoint_cmd(ccp, cmd_close_endpoint,
	                            sizeof(cmd_close_endpoint),
	                            endpoint);

out_unlock:
	return ret;
}

/* read firmware version */
static int get_fw_version(struct ccxt_device *ccp)
{
	int ret;

	ret = send_usb_endpoint_cmd(ccp, cmd_get_firmware,
	                            sizeof(cmd_get_firmware), 0);
	if (ret) {
		fprintf(stderr,
		        "Failed to read firmware version, error code %d.\n",
		        ret);
		return ret;
	}
	ccp->firmware_ver.major = ccp->buffer[3];
	ccp->firmware_ver.minor = ccp->buffer[4];
	ccp->firmware_ver.patch = (u16)ccp->buffer[5] |
	                          (u16)ccp->buffer[6] << 8;

	return 0;
}

static int set_hardware_mode(struct ccxt_device *ccp)
{
	int ret;

	ret = send_usb_endpoint_cmd(ccp, cmd_hardware_mode,
	                            sizeof(cmd_hardware_mode), 0);

	return ret;
}

static int set_software_mode(struct ccxt_device *ccp)
{
	int ret;

	ret = send_usb_endpoint_cmd(ccp, cmd_software_mode,
	                            sizeof(cmd_software_mode), 0);

	return ret;
}

int main(void)
{
	u8 cmd_buffer[OUT_BUFFER_SIZE];
	u8 buffer[IN_BUFFER_SIZE];
	u8 data_buffer[IN_BUFFER_SIZE];

	struct ccxt_device ccp = {
		.cmd_buffer = cmd_buffer,
		.buffer = buffer,
		.data_buffer = data_buffer,
	};

	int res = hid_init();
	if (res) {
		fprintf(stderr, "Could not initialize hid_api\n");
		return 1;
	}

	ccp.hdev = hid_open(USB_VENDOR_ID_CORSAIR,
	                    USB_PRODUCT_ID_CORSAIR_COMMANDER_CORE_XT,
	                    NULL);
	if (ccp.hdev == NULL) {
		fprintf(stderr, "Could not find device\n");
		return 1;
	}

	res = get_fw_version(&ccp);
	if (res)
		goto close;

	fprintf(stdout, "FW: %u.%u.%u\n",
	        ccp.firmware_ver.major,
	        ccp.firmware_ver.minor,
	        ccp.firmware_ver.patch);

	res = set_software_mode(&ccp);
	if (res)
		goto close;

	fprintf(stderr, "=== WRITING FAN SPEEDS ===\n");

	const u8 speed_cmd[] = { 0x07, 0x00, /* packet size (data + type) */
	                         0x00, 0x00, /* extra header space */
	                         0x07, 0x00, /* buffer type*/
	                         1, 0x04, 0, 50, 0x00 /*fan data */ };
	res = write_data(&ccp, endpoint_fan_pwm, speed_cmd, sizeof(speed_cmd));
	if (res)
		fprintf(stderr, "Failed to write data to endpoint %02x\n", res);
	else
		print_buffer(ccp.data_buffer, 16);

	fprintf(stderr, "=== WROTE FAN SPEEDS ===\n");

	// Read PWM data
	get_data(&ccp, endpoint_fan_pwm);
	int num_fans = ccp.data_buffer[5];
	for (int i = 0, o = 6; i < num_fans; i++, o += 4) {
		int fan_id = ccp.data_buffer[o];
		int mode = ccp.data_buffer[o + 1];
		int pwm = ccp.data_buffer[o + 2];
		fprintf(stderr, "Fan %d: mode %d, pwm %d\n", fan_id, mode, pwm);
	}

	// set_hardware_mode(&ccp);

close:
	hid_close(ccp.hdev);

	return 0;
}