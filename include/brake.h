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

/* Reset state (call once during start-up). */
void brake_init(void);

/*
 * Poll the E-STOP / BRAKE input (call once per main-loop iteration) and drive
 * the indicators:
 *   - pressed (E-STOP engaged): CAN Tx/Rx LEDs on,  LED_BRAKE on
 *   - released (idle):          CAN Tx/Rx LEDs off, LED_BRAKE off
 * (CAN Tx/Rx: 2 LEDs on a 1-channel build, 4 on a 2-channel build. "off" while
 * released means the normal led_update() state, i.e. off when the CAN
 * interface is down, normal activity indication when it is up.)
 */
void brake_task(void);

/*
 * True while the E-STOP is pressed. The main loop runs led_update() only while
 * released; while pressed it is skipped so brake_task()'s "CAN LEDs on"
 * override is kept.
 */
bool brake_is_engaged(void);

#endif /* CONFIG_BRAKE */
