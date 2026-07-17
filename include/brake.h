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

#pragma once

#include "config.h"

#ifdef CONFIG_BRAKE

#include <stdbool.h>

#include "usbd_gs_can.h"

/* Reset state (call once during start-up). */
void brake_init(void);

/*
 * Poll the E-STOP / BRAKE input (call once per main-loop iteration). While the
 * E-STOP is pressed this:
 *   - lights all CAN Tx/Rx LEDs and LED_BRAKE,
 *   - sends the RS02 PRIVATE MotionControl damping command (Kp=0, Kd=5.0) to the motors,
 *     distributed over the CAN channels (1 ch: ids 1..12; 2 ch: 1..6 and 7..12),
 *     and non-blockingly retries/repeats it every BRAKE_RESEND_MS.
 * The brake is a one-shot latch: once pressed it stays engaged after the
 * button is released; only a reset/power-cycle clears it.
 * Host-frame discarding is disabled: host TX requests queue while engaged
 * (the main loop skips forwarding) until the shared frame pool runs dry,
 * after which the host sees ENOBUFS and CAN->host forwarding also starves.
 */
void brake_task(USBD_GS_CAN_HandleTypeDef *hcan);

/*
 * True while the E-STOP is pressed. While engaged the main loop must skip
 * host->CAN forwarding and led_update() (the firmware drives the bus and LEDs).
 */
bool brake_is_engaged(void);

#endif /* CONFIG_BRAKE */
