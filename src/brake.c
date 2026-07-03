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
#include "hal_include.h"
#include "led.h"

/* Consecutive samples required before a button state change is accepted. */
#ifndef BRAKE_DEBOUNCE_SAMPLES
#define BRAKE_DEBOUNCE_SAMPLES 3
#endif

/*
 * PB5 level that means the E-STOP is pressed/engaged. This is a fail-safe
 * normally-closed (B-contact) input: the board pulls BRAKE up via R11 and the
 * closed contact ties it to GND while released, so
 *   released (normal) = LOW,  pressed / broken wire / unplugged = HIGH.
 * Override in config.h if a board wires the E-STOP the other way round.
 */
#ifndef BRAKE_PRESSED_STATE
#define BRAKE_PRESSED_STATE GPIO_PIN_SET
#endif

static bool brake_engaged;
static uint8_t brake_debounce;

static bool brake_input_pressed(void)
{
	const struct brake_config *cfg = &config.brake;

	return HAL_GPIO_ReadPin(cfg->button_port, cfg->button_pin) == BRAKE_PRESSED_STATE;
}

/*
 * Drive every CAN Tx/Rx LED. Iterates the configured channels, so it covers
 * 2 LEDs on a 1-channel build and 4 LEDs on a 2-channel build automatically
 * (keyed on NUM_CAN_CHANNEL).
 */
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

/* LED_BRAKE indicator (active high). */
static void brake_set_led_brake(bool on)
{
	const struct brake_config *cfg = &config.brake;

	HAL_GPIO_WritePin(cfg->led_port, cfg->led_pin,
					  on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* Apply the indicators for the current engaged/idle state. */
static void brake_apply(void)
{
	/* Both the CAN Tx/Rx LEDs and LED_BRAKE light while the E-STOP is pressed. */
	brake_set_led_brake(brake_engaged);

	/*
	 * CAN Tx/Rx LEDs: forced on while pressed. While released they revert to
	 * the normal led_update() path (the main loop runs it only while released).
	 */
	if (brake_engaged) {
		brake_set_can_leds(true);
	}
}

void brake_init(void)
{
	brake_debounce = 0;
	brake_engaged = brake_input_pressed();
	brake_apply();
}

bool brake_is_engaged(void)
{
	return brake_engaged;
}

void brake_task(void)
{
	bool pressed = brake_input_pressed();

	/* non-blocking debounce: require a few consecutive disagreeing samples */
	if (pressed != brake_engaged) {
		if (++brake_debounce >= BRAKE_DEBOUNCE_SAMPLES) {
			brake_debounce = 0;
			brake_engaged = pressed;
		}
	} else {
		brake_debounce = 0;
	}

	/* apply every iteration (idempotent) so the idle "CAN LEDs on" override is
	 * robust against the start-up LED pattern and needs no edge bookkeeping */
	brake_apply();
}

#endif /* CONFIG_BRAKE */
