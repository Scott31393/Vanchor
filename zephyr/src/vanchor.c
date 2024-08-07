/*
 * Copyright (c) 2019 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Sample echo app for CDC ACM class
 *
 * Sample app for USB CDC ACM class driver. The received data is echoed back
 * to the serial port.
 */

#include <sample_usbd.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/stepper.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/ring_buffer.h>

#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/logging/log.h>

#include "vanchor.h"

LOG_MODULE_REGISTER(motors_ctrl, LOG_LEVEL_INF);

/* DC/Stepper motors */
static vanchor_motors_handler_t mot_hdlr;

#if !DT_NODE_HAS_STATUS(DT_ALIAS(stepper), okay)
#error "Unsupported board: pa8_pwm1 devicetree alias is not defined"
#endif
const struct device *const stepper = DEVICE_DT_GET(DT_ALIAS(stepper));

/* Servo motor pwm */
#if !DT_NODE_HAS_STATUS(DT_ALIAS(pa8_pwm1), okay)
#error "Unsupported board: pa8_pwm1 devicetree alias is not defined"
#endif
const struct pwm_dt_spec pa8_pwm1 = PWM_DT_SPEC_GET(DT_ALIAS(pa8_pwm1));
__maybe_unused static const uint32_t pa8_min_pulse = DT_PROP(DT_ALIAS(pa8_pwm1), min_pulse);
__maybe_unused static const uint32_t pa8_max_pulse = DT_PROP(DT_ALIAS(pa8_pwm1), max_pulse);

#if !DT_NODE_HAS_STATUS(DT_ALIAS(pa0_pwm2), okay)
#error "Unsupported board: pa0_pwm2 devicetree alias is not defined"
#endif
const struct pwm_dt_spec pa0_pwm2 = PWM_DT_SPEC_GET(DT_ALIAS(pa0_pwm2));
__maybe_unused static const uint32_t pa0_min_pulse = DT_PROP(DT_ALIAS(pa0_pwm2), min_pulse);
__maybe_unused static const uint32_t pa0_max_pulse = DT_PROP(DT_ALIAS(pa0_pwm2), max_pulse);

#define STEP PWM_USEC(100)

enum direction {
	DOWN,
	UP,
};

/* USB CDC */
#define RING_BUF_SIZE 1024

const struct device *const uart_dev = DEVICE_DT_GET_ONE(zephyr_cdc_acm_uart);

uint8_t ring_buffer[RING_BUF_SIZE];
struct ring_buf ringbuf;

static bool rx_throttled;

/* vanchor cmd type: update, calibration, ping */
vanchor_msg_t msg;

char str_upd[] = "UPD";
char str_cal[] = "CAL";
char str_ping[] = "PING";

static int vanchor_get_dis2go(vanchor_motors_handler_t *mot_hdlr)
{

	int ret;
	int32_t cur;

	ret = stepper_get_actual_position(stepper, &cur);
	if (ret)
		return ret;

	mot_hdlr->step_dis2go = mot_hdlr->des_step_pos - cur;

	LOG_DBG("Current position (raw): %d", cur);
	LOG_DBG("Desired position (raw): %d", mot_hdlr->des_step_pos);
	LOG_DBG("Distance to go (steps): %d", mot_hdlr->step_dis2go);

	return ret;
}

static void vanchor_log_motors_status(vanchor_motors_handler_t *mot_hdlr,
				      char *buffer, size_t buffer_size)
{
	int ret;

	ret = vanchor_get_dis2go(mot_hdlr);
	if (ret)
		return;

	snprintf(buffer, buffer_size,
		 "STATUS SSP:%d SDTG:%d MS:%d CB:%d CE:%d\r\n",
		 mot_hdlr->des_step_pos,
		 mot_hdlr->step_dis2go,
		 mot_hdlr->des_dc_speed,
		 mot_hdlr->calib_begin,
		 mot_hdlr->calib_end);

	LOG_INF("STATUS SSP:%d SDTG:%d MS:%d CB:%d CE:%d\r\n",
		 mot_hdlr->des_step_pos,
		 mot_hdlr->step_dis2go,
		 mot_hdlr->des_dc_speed,
		 mot_hdlr->calib_begin,
		 mot_hdlr->calib_end);
}

static int vanchor_ping_cmd(vanchor_msg_t *cmd)
{
	LOG_INF("Pong\n");
	return 0;
}

static int vanchor_calibration_cmd(vanchor_msg_t *cmd)
{
	stepper_set_actual_position(stepper, cmd->param1);
	return 0;
}

static int vanchor_update_cmd(vanchor_msg_t *cmd,
			      vanchor_motors_handler_t *mot_hdlr)
{
	mot_hdlr->cur_step_pos = cmd->param1;
	mot_hdlr->cur_step_speed = cmd->param2;
	mot_hdlr->cur_step_acc = cmd->param3;
	mot_hdlr->cur_dc_speed = cmd->param4;
	mot_hdlr->cur_dc_rev = cmd->param5;

	LOG_INF("UPD CSP:%d CSS:%d CSA:%d CDS:%d CDR:%d\r\n",
		 mot_hdlr->cur_step_pos,
		 mot_hdlr->cur_step_speed,
		 mot_hdlr->cur_step_acc,
		 mot_hdlr->cur_dc_speed,
		 mot_hdlr->cur_dc_rev);

	if (mot_hdlr->des_step_pos != mot_hdlr->cur_step_pos) {
		stepper_move(stepper, mot_hdlr->cur_step_pos);
		mot_hdlr->des_step_pos = mot_hdlr->cur_step_pos;
	}

	if (mot_hdlr->des_step_speed != mot_hdlr->cur_step_speed) {
		stepper_set_max_velocity(stepper, mot_hdlr->cur_step_speed);
		mot_hdlr->des_step_speed = mot_hdlr->cur_step_speed;
	}

	if (mot_hdlr->des_step_acc != mot_hdlr->cur_step_acc) {
		/*
		 * set stepper acceleration:
		 * The desired acceleration in steps per second per second.
		 * stepper_set_accelleration(stepper, mot_hdlr->cur_step_acc);
		 */
		mot_hdlr->des_step_acc = mot_hdlr->cur_step_acc;
	}

	if (mot_hdlr->des_dc_speed != mot_hdlr->cur_dc_speed) {
		/* set_dc_motor_speed here */
		mot_hdlr->des_dc_speed = mot_hdlr->cur_dc_speed;
		mot_hdlr->des_dc_rev = mot_hdlr->cur_dc_rev;
	}

	return 0;
}

static bool vanchor_parse_msg(const char *msg_in, vanchor_msg_t *cmd)
{
	int ret;
	int msg_type = VANCHOR_NONE;

	ret = sscanf(msg_in, "%3s %d %d %d %d %d",
		     cmd->type, &cmd->param1, &cmd->param2,
		     &cmd->param3, &cmd->param4, &cmd->param5) == 6;

	/* check cmd type: update, calibration, ping*/
	if (strcmp(cmd->type, str_upd) == 0)
		msg_type = VANCHOR_UPD;
	else if (strcmp(cmd->type, str_cal) == 0)
		msg_type = VANCHOR_CAL;
	else if (strcmp(cmd->type, str_cal) == 0)
		msg_type = VANCHOR_PING;

	switch (msg_type) {
	case VANCHOR_UPD:
		vanchor_update_cmd(cmd, &mot_hdlr);
		break;
	case VANCHOR_CAL:
		vanchor_calibration_cmd(cmd);
		break;
	case VANCHOR_PING:
		vanchor_ping_cmd(cmd);
		break;
	default:
		break;
	}

	return ret;
}

/* usb cdc irq handler*/
static void irq_handler(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);
	char log_buffer[128];

	while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
		if (!rx_throttled && uart_irq_rx_ready(dev)) {
			int recv_len, rb_len;
			uint8_t buffer[64];
			size_t len = MIN(ring_buf_space_get(&ringbuf),
					 sizeof(buffer));

			if (len == 0) {
				/* Throttle because ring buffer is full */
				uart_irq_rx_disable(dev);
				rx_throttled = true;
				continue;
			}

			recv_len = uart_fifo_read(dev, buffer, len);
			if (recv_len < 0) {
				LOG_ERR("Failed to read UART FIFO");
				recv_len = 0;
			};

			if (vanchor_parse_msg((char *)buffer, &msg)) {
				/* msg parsed */
			} else {
				LOG_ERR("Failed to parse motor command");
			}

			/* Log the motor status */
			vanchor_log_motors_status(&mot_hdlr, log_buffer, sizeof(log_buffer));

			/* Put the log buffer into the ring buffer for transmission */
			rb_len = ring_buf_put(&ringbuf, (uint8_t *)log_buffer, strlen(log_buffer));
			if (rb_len < strlen(log_buffer)) {
			LOG_ERR("Drop %u bytes", strlen(log_buffer) - rb_len);
			}

			LOG_DBG("tty fifo -> ringbuf %d bytes", rb_len);
			if (rb_len) {
				uart_irq_tx_enable(dev);
			}
		}

		if (uart_irq_tx_ready(dev)) {
			uint8_t buffer[64];
			int rb_len, send_len;

			rb_len = ring_buf_get(&ringbuf, buffer, sizeof(buffer));
			if (!rb_len) {
				LOG_DBG("Ring buffer empty, disable TX IRQ");
				uart_irq_tx_disable(dev);
				continue;
			}

			if (rx_throttled) {
				uart_irq_rx_enable(dev);
				rx_throttled = false;
			}

			send_len = uart_fifo_fill(dev, buffer, rb_len);
			if (send_len < rb_len) {
				LOG_ERR("Drop %d bytes", rb_len - send_len);
			}

			LOG_DBG("ringbuf -> tty fifo %d bytes", send_len);
		}
	}
}

static int vanchor_motors_hdlr_init(void)
{
	mot_hdlr.step_need_init = true;
	mot_hdlr.step_dis2go = 0;
	mot_hdlr.des_step_pos = 0;
	mot_hdlr.des_step_speed = 0;
	mot_hdlr.des_step_acc = 0;
	mot_hdlr.des_dc_speed = 0;
	mot_hdlr.des_dc_rev = 0;
	mot_hdlr.cur_step_pos = 0;
	mot_hdlr.cur_step_speed = 0;
	mot_hdlr.cur_step_acc = 0;
	mot_hdlr.cur_dc_speed = 0;
	mot_hdlr.cur_dc_rev = 0;
	mot_hdlr.calib_begin = 0;
	mot_hdlr.calib_end = 0;

	return 0;
}

static int vanchor_usb_irq_init(void)
{
	int ret;

	ring_buf_init(&ringbuf, sizeof(ring_buffer), ring_buffer);
	LOG_INF("Wait for DTR");

	while (true) {
		uint32_t dtr = 0U;

		uart_line_ctrl_get(uart_dev, UART_LINE_CTRL_DTR, &dtr);
		if (dtr) {
			break;
		} else {
			/* Give CPU resources to low priority threads. */
			k_sleep(K_MSEC(100));
		}
	}
	LOG_INF("DTR set");

	/* They are optional, we use them to test the interrupt endpoint */
	ret = uart_line_ctrl_set(uart_dev, UART_LINE_CTRL_DCD, 1);
	if (ret) {
		LOG_WRN("Failed to set DCD, ret code %d", ret);
	}

	ret = uart_line_ctrl_set(uart_dev, UART_LINE_CTRL_DSR, 1);
	if (ret) {
		LOG_WRN("Failed to set DSR, ret code %d", ret);
	}

	/* Wait 100ms for the host to do all settings */
	k_msleep(100);

	uart_irq_callback_set(uart_dev, irq_handler);

	/* Enable rx interrupts */
	uart_irq_rx_enable(uart_dev);

	return ret;
}

static int vanchor_pwm_init(void)
{
	uint32_t pa8_pulse_w = pa8_min_pulse;
	uint32_t pa0_pulse_w = pa0_min_pulse;
	int ret;

	ret = pwm_set_pulse_dt(&pa8_pwm1, pa8_pulse_w);
	if (ret < 0) {
		printk("Error %d: failed to set pulse width\n", ret);
		return ret;
	}

	ret = pwm_set_pulse_dt(&pa0_pwm2, pa0_pulse_w);
	if (ret < 0) {
		printk("Error %d: failed to set pulse width\n", ret);
		return ret;
	}

	return 0;
}

static int vanchor_dt_check(void)
{
	if (!device_is_ready(uart_dev)) {
		LOG_ERR("CDC ACM device not ready");
		return -ENODEV;;
	}

	if (!device_is_ready(stepper)) {
		LOG_ERR("stepper device not ready");
		return -ENODEV;;
	}

	if (!pwm_is_ready_dt(&pa8_pwm1)) {
		LOG_ERR("Error: pa8_pwm1 device %s is not ready\n",
			pa8_pwm1.dev->name);
		return -ENODEV;;
	}

	if (!pwm_is_ready_dt(&pa0_pwm2)) {
		LOG_ERR("Error: pa0_pwm2 device %s is not ready\n",
			pa0_pwm2.dev->name);
		return -ENODEV;;
	}

	return 0;
}

int main(void)
{
	int ret;

	ret = vanchor_dt_check();
	if (ret) {
		LOG_ERR("Error %d: vanchor_dt_check\n", ret);
		return ret;
	}

	ret = vanchor_pwm_init();
	if (ret) {
		LOG_ERR("Error %d: vanchor_pwm_init\n", ret);
		return ret;
	}

	ret = usb_enable(NULL);
	if (ret != 0) {
		LOG_ERR("Failed to enable USB");
		return 0;
	}

	ret = vanchor_usb_irq_init();
	if (ret) {
		LOG_ERR("Error %d: vanchor_usb_irq_init\n", ret);
		return ret;
	}

	ret = vanchor_motors_hdlr_init();
	if (ret) {
		LOG_ERR("Error %d: vanchor_motors_hdlr_init\n", ret);
		return ret;
	}

	return 0;
}
