/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2026
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#include "brake.h"

#ifdef CONFIG_BRAKE

#include <stdbool.h>
#include <stdint.h>

#include "board.h"
#include "can.h"
#include "can_common.h"
#include "gs_usb.h"
#include "hal_include.h"
#include "led.h"

/* --- E-STOP input --- */

/* Consecutive samples required before a button state change is accepted. */
#ifndef BRAKE_DEBOUNCE_SAMPLES
#define BRAKE_DEBOUNCE_SAMPLES 3
#endif

/*
 * PB5 level that means the E-STOP is pressed. Fail-safe normally-closed
 * (B-contact): released = LOW (contact ties BRAKE to GND), pressed / broken
 * wire / unplugged = HIGH (R11 pull-up). Override in config.h to invert.
 */
#ifndef BRAKE_PRESSED_STATE
#define BRAKE_PRESSED_STATE GPIO_PIN_SET
#endif

/* --- motor MIT-mode damping command (RS02 / RobStride) ------------------------
 *
 * RS02 MIT protocol (manual ch.6.5 "MIT Dynamic Parameters"): STANDARD 11-bit
 * frame, CAN id = motor id (mode-type 0). The 8-byte payload packs
 * pos(16) speed(12) Kp(12) Kd(12) torque(12); the values below are
 * pos=0, speed=0, Kp=0, Kd=5.0(max), torque=0 -> pure max-damping brake
 * (tau = -Kd*v). Range-independent (0 -> mid-scale, Kd -> full-scale).
 */
#define BRAKE_MOTOR_CMD  { 0x7F, 0xFF, 0x7F, 0xF0, 0x00, 0xFF, 0xF7, 0xFF }
#define BRAKE_CAN_ID(id) ((uint32_t)(id))

/*
 * Total motors (CAN ids 1..N), split evenly over the CAN channels:
 *   1 channel  -> ch0 brakes ids 1..N
 *   2 channels -> ch0 brakes 1..N/2, ch1 brakes N/2+1..N
 */
#ifndef BRAKE_NUM_MOTORS
#define BRAKE_NUM_MOTORS 12
#endif

/* Bounded busy-retry per frame while the TX FIFO is full (never hangs). */
#ifndef BRAKE_SEND_TRIES
#define BRAKE_SEND_TRIES 0xFFFFu
#endif

/*
 * Re-send the damping command every N ms while pressed. MIT-mode motors expect
 * a continuous command, so a one-shot could time out and release the brake.
 * Set to 0 for a single burst on the press edge only.
 */
#ifndef BRAKE_RESEND_MS
#define BRAKE_RESEND_MS 5
#endif

static const uint8_t brake_motor_cmd[8] = BRAKE_MOTOR_CMD;

static bool brake_engaged;
static uint8_t brake_debounce;
static uint32_t brake_next_send;

static bool brake_input_pressed(void)
{
	const struct brake_config *cfg = &config.brake;

	/* wired E-STOP (BRAKE) */
	if (HAL_GPIO_ReadPin(cfg->button_port, cfg->button_pin) == BRAKE_PRESSED_STATE) {
		return true;
	}

	/* wireless E-STOP (BRAKE_W), if the board wires one (e.g. Mujina PB4) */
	if (cfg->wl_button_port != NULL &&
		HAL_GPIO_ReadPin(cfg->wl_button_port, cfg->wl_button_pin) == BRAKE_PRESSED_STATE) {
		return true;
	}

	return false;
}

/* --- indicators --- */

static void brake_set_can_leds(bool on)
{
	for (unsigned int i = 0; i < NUM_CAN_CHANNEL; i++) {
		const struct led_config *leds = config.channel[i].leds;

		for (unsigned int j = 0; j < LED_MAX; j++) {
			HAL_GPIO_WritePin(leds[j].port, leds[j].pin,
							  (on == leds[j].active_high) ?
							  GPIO_PIN_SET : GPIO_PIN_RESET);
		}
	}
}

static void brake_set_led_brake(bool on)
{
	const struct brake_config *cfg = &config.brake;

	HAL_GPIO_WritePin(cfg->led_port, cfg->led_pin,
					  on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void brake_apply_leds(void)
{
	/*
	 * released/idle -> CAN Tx/Rx LEDs ON,  LED_BRAKE OFF
	 * pressed       -> CAN Tx/Rx LEDs OFF, LED_BRAKE ON
	 * Forced in both states (independent of the CAN interface being up), so on
	 * a brake board the CAN Tx/Rx LEDs act purely as the E-STOP indicator.
	 */
	brake_set_led_brake(brake_engaged);
	brake_set_can_leds(!brake_engaged);
}

/* --- damping command --- */

static void brake_send_range(can_data_t *channel, uint8_t id_first, uint8_t id_last)
{
	/* nothing to do if the bus is not up (can_send would just fail) */
	if (!can_is_enabled(channel)) {
		return;
	}

	/*
	 * gs_host_frame's payload is a flexible array member, so build the frame in
	 * a correctly sized gs_host_frame_object on the stack (never a bare
	 * gs_host_frame, never a borrowed frame-pool buffer). can_send() copies into
	 * the HW TX FIFO synchronously, so the stack object is fine.
	 */
	struct gs_host_frame_object tx = { 0 };
	struct gs_host_frame *frame = &tx.frame;

	frame->echo_id = 0xFFFFFFFF; /* not an echo of a host frame */
	frame->can_dlc = 8;
	frame->channel = can_channel_get_nr(channel);
	frame->flags = 0;
	for (uint8_t i = 0; i < 8; i++) {
		frame->classic_can->data[i] = brake_motor_cmd[i];
	}

	for (uint8_t id = id_first; id <= id_last; id++) {
		frame->can_id = BRAKE_CAN_ID(id);

		uint32_t tries = BRAKE_SEND_TRIES;
		while (!can_send(channel, frame) && --tries) {
			/* TX FIFO full: spin until a slot frees or we give up */
		}
	}
}

static void brake_send_all(USBD_GS_CAN_HandleTypeDef *hcan)
{
	const uint8_t per = BRAKE_NUM_MOTORS / NUM_CAN_CHANNEL;

	for (uint8_t ch = 0; ch < NUM_CAN_CHANNEL; ch++) {
		uint8_t id_first = (uint8_t)(ch * per + 1);
		uint8_t id_last = (ch == NUM_CAN_CHANNEL - 1) ?
						  BRAKE_NUM_MOTORS : (uint8_t)((ch + 1) * per);

		brake_send_range(&hcan->channels[ch], id_first, id_last);
	}
}

/* --- API --- */

void brake_init(void)
{
	brake_debounce = 0;
	brake_engaged = brake_input_pressed();
	brake_apply_leds();
}

bool brake_is_engaged(void)
{
	return brake_engaged;
}

void brake_task(USBD_GS_CAN_HandleTypeDef *hcan)
{
	bool pressed = brake_input_pressed();

	/* non-blocking debounce: require a few consecutive disagreeing samples */
	if (pressed != brake_engaged) {
		if (++brake_debounce >= BRAKE_DEBOUNCE_SAMPLES) {
			brake_debounce = 0;
			brake_engaged = pressed;

			if (brake_engaged) {
				/* engage edge: brake immediately, then hold */
				brake_send_all(hcan);
				brake_next_send = HAL_GetTick() + BRAKE_RESEND_MS;
			}
		}
	} else {
		brake_debounce = 0;
	}

	brake_apply_leds();

	/* hold the damping command alive while pressed (MIT watchdog safety) */
	if (brake_engaged && BRAKE_RESEND_MS) {
		uint32_t now = HAL_GetTick();

		if ((int32_t)(now - brake_next_send) >= 0) {
			brake_send_all(hcan);
			brake_next_send = now + BRAKE_RESEND_MS;
		}
	}
}

#endif /* CONFIG_BRAKE */
