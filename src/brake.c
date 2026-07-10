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

/*
 * Engagement is immediate. Release must remain stable for this long before the
 * damping command is stopped, so contact bounce cannot briefly release it.
 */
#ifndef BRAKE_RELEASE_DEBOUNCE_MS
#define BRAKE_RELEASE_DEBOUNCE_MS 50u
#endif

/*
 * PB5 level that means the E-STOP is pressed. Fail-safe normally-closed
 * (B-contact): released = LOW (contact ties BRAKE to GND), pressed / broken
 * wire / unplugged = HIGH (R11 pull-up). Override in config.h to invert.
 */
#ifndef BRAKE_PRESSED_STATE
#define BRAKE_PRESSED_STATE GPIO_PIN_SET
#endif

/* --- motor damping command (RS02 / RobStride, PRIVATE operation-control) ------
 *
 * The Mujina RS02 motors run RobStride's PRIVATE CAN protocol, NOT the manual's
 * ch.6.5 "MIT" protocol. Normal operation AND the host's own emergency stop use
 * "MotionControl" (Communication_Type = 1) -- byte-for-byte the frame emitted by
 * quadruped_control's RobStrideController._send_robstride_move_control():
 *
 *   extended id = CAN_EFF_FLAG | (1 << 24) | (torque_u16 << 8) | motor_id
 *   payload (big-endian uint16 each):
 *     [0:2] angle  in [-12.5, 12.5] rad
 *     [2:4] speed  in [-44,   44  ] rad/s
 *     [4:6] Kp     in [0,     500 ]
 *     [6:8] Kd     in [0,     5.0 ]
 *   torque (in [-17,17]) rides in the id's opt field, NOT the payload.
 *
 * Damping brake = angle=0, speed=0, Kp=0, Kd=2.0, torque=0
 *   -> tau = Kp*(0-p) + Kd*(0-v) + 0 = -Kd*v  (pure velocity damping).
 * uint = (x-min)*65535/(max-min): symmetric field 0 -> 0x7FFF, Kp=0 -> 0x0000,
 * torque=0 -> opt 0x7FFF. Kd=2.0 of the 0..5.0 range is encoded as
 * floor(3.5/5.0*65535) = 0xB332. tau=-Kd*v reaches the 17 N.m limit above
 * ~4.9 rad/s.
 * Damping regenerates energy into the DC bus; the supply and
 * bulk capacitance must be able to absorb it without overvoltage before Kd is
 * increased. The motor must already be enabled. The frame is re-sent every
 * BRAKE_RESEND_MS to maintain the command and feed a configured CAN watchdog.
 */
#define BRAKE_MOTOR_CMD  { 0x7F, 0xFF, 0x7F, 0xFF, 0x00, 0x00, 0x66, 0x66 }
#define BRAKE_MOTIONCTRL 0x01u   /* Communication_Type_MotionControl */
#define BRAKE_TORQUE_U16 0x7FFFu /* torque = 0 (mid of the +/-17 N.m range) */
#define BRAKE_CAN_ID(id) \
	(CAN_EFF_FLAG | ((uint32_t)BRAKE_MOTIONCTRL << 24) | \
	 ((uint32_t)BRAKE_TORQUE_U16 << 8) | (uint32_t)(id))

/*
 * Total motors (CAN ids 1..N), split evenly over the CAN channels:
 *   1 channel  -> ch0 brakes ids 1..N
 *   2 channels -> ch0 brakes 1..N/2, ch1 brakes N/2+1..N
 */
#ifndef BRAKE_NUM_MOTORS
#define BRAKE_NUM_MOTORS 12
#endif

/*
 * Re-send the damping command to every motor every N ms while pressed. Set to 0
 * to queue one complete-motor cycle on the press edge only.
 */
#ifndef BRAKE_RESEND_MS
#define BRAKE_RESEND_MS 10u
#endif

#if BRAKE_NUM_MOTORS < 1 || BRAKE_NUM_MOTORS > 32
#error "BRAKE_NUM_MOTORS must be in the range 1..32"
#endif

#if NUM_CAN_CHANNEL < 1 || NUM_CAN_CHANNEL > BRAKE_NUM_MOTORS
#error "NUM_CAN_CHANNEL must be in the range 1..BRAKE_NUM_MOTORS"
#endif

static const uint8_t brake_motor_cmd[8] = BRAKE_MOTOR_CMD;

static bool brake_engaged;
static bool brake_release_pending;
static uint32_t brake_release_since;
static uint32_t brake_next_send[NUM_CAN_CHANNEL];
static uint32_t brake_pending_mask[NUM_CAN_CHANNEL];

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

static void brake_send_pending_range(can_data_t *channel, uint32_t *pending_mask,
									 uint8_t id_first, uint8_t id_last)
{
	/* Keep the bits pending until the host has brought this CAN channel up. */
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
		const uint32_t motor_bit = 1u << (id - 1u);
		if (!(*pending_mask & motor_bit)) {
			continue;
		}

		frame->can_id = BRAKE_CAN_ID(id);

		/*
		 * Never wait for the hardware FIFO in the E-STOP path. Leaving this and
		 * later motor bits set retries them on the next main-loop iteration,
		 * while button sampling, USB, CAN RX and error handling keep running.
		 */
		if (!can_send(channel, frame)) {
			break;
		}
		*pending_mask &= ~motor_bit;
	}
}

static uint32_t brake_channel_motor_mask(uint8_t ch)
{
	const uint8_t per = BRAKE_NUM_MOTORS / NUM_CAN_CHANNEL;
	const uint8_t id_first = (uint8_t)(ch * per + 1);
	const uint8_t id_last = (ch == NUM_CAN_CHANNEL - 1) ?
							BRAKE_NUM_MOTORS : (uint8_t)((ch + 1) * per);
	uint32_t mask = 0;

	for (uint8_t id = id_first; id <= id_last; id++) {
		mask |= 1u << (id - 1u);
	}
	return mask;
}

static void brake_schedule_all_channels(uint32_t now)
{
	for (uint8_t ch = 0; ch < NUM_CAN_CHANNEL; ch++) {
		brake_pending_mask[ch] = brake_channel_motor_mask(ch);
		brake_next_send[ch] = now + BRAKE_RESEND_MS;
	}
}

static void brake_send_pending(USBD_GS_CAN_HandleTypeDef *hcan)
{
	const uint8_t per = BRAKE_NUM_MOTORS / NUM_CAN_CHANNEL;

	for (uint8_t ch = 0; ch < NUM_CAN_CHANNEL; ch++) {
		uint8_t id_first = (uint8_t)(ch * per + 1);
		uint8_t id_last = (ch == NUM_CAN_CHANNEL - 1) ?
						  BRAKE_NUM_MOTORS : (uint8_t)((ch + 1) * per);

		brake_send_pending_range(&hcan->channels[ch],
								 &brake_pending_mask[ch],
								 id_first, id_last);
	}
}

static void brake_discard_host_frames(USBD_GS_CAN_HandleTypeDef *hcan)
{
	for (uint8_t ch = 0; ch < NUM_CAN_CHANNEL; ch++) {
		CAN_DiscardPendingTxFrames(hcan, &hcan->channels[ch]);
	}
}

/* --- API --- */

void brake_init(void)
{
	const uint32_t now = HAL_GetTick();

	brake_engaged = brake_input_pressed();
	brake_release_pending = false;
	brake_release_since = 0;
	for (uint8_t ch = 0; ch < NUM_CAN_CHANNEL; ch++) {
		brake_pending_mask[ch] = 0;
		brake_next_send[ch] = now + BRAKE_RESEND_MS;
	}
	if (brake_engaged) {
		brake_schedule_all_channels(now);
	}
	brake_apply_leds();
}

bool brake_is_engaged(void)
{
	return brake_engaged;
}

void brake_task(USBD_GS_CAN_HandleTypeDef *hcan)
{
	const uint32_t now = HAL_GetTick();
	const bool pressed = brake_input_pressed();
	const bool was_engaged = brake_engaged;

	if (pressed) {
		/* Fail-safe engage: one pressed sample latches the damping state. */
		brake_release_pending = false;
		if (!brake_engaged) {
			brake_engaged = true;
			brake_schedule_all_channels(now);
		}
	} else if (brake_engaged) {
		if (!brake_release_pending) {
			brake_release_pending = true;
			brake_release_since = now;
		} else if ((uint32_t)(now - brake_release_since) >=
				   BRAKE_RELEASE_DEBOUNCE_MS) {
			brake_engaged = false;
			brake_release_pending = false;
			for (uint8_t ch = 0; ch < NUM_CAN_CHANNEL; ch++) {
				brake_pending_mask[ch] = 0;
			}
		}
	}

	if (brake_engaged) {
		/*
		 * Host commands must neither fight the damping command nor survive the
		 * stop and run later. The discard helper still returns gs_usb echoes.
		 */
		brake_discard_host_frames(hcan);

		/* Each CAN channel advances independently if another bus is down/full. */
		for (uint8_t ch = 0; ch < NUM_CAN_CHANNEL; ch++) {
			if (BRAKE_RESEND_MS && brake_pending_mask[ch] == 0 &&
				(int32_t)(now - brake_next_send[ch]) >= 0) {
				brake_pending_mask[ch] = brake_channel_motor_mask(ch);
				/* Do not burst catch-up cycles after a delayed main loop. */
				brake_next_send[ch] = now + BRAKE_RESEND_MS;
			}
		}

		brake_send_pending(hcan);
	} else if (was_engaged) {
		/* Flush the final stop-period frames before host forwarding resumes. */
		brake_discard_host_frames(hcan);
	}

	brake_apply_leds();
}

#endif /* CONFIG_BRAKE */
