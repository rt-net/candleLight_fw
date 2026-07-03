/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2023 Pengutronix,
 *               Marc Kleine-Budde <kernel@pengutronix.de>
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

#include "board.h"
#include "config.h"
#include "device.h"
#include "gpio.h"
#include "usbd_gs_can.h"

/*
 * LED1_RX  PD2   (channel 0 / FDCAN1)
 * LED1_TX  PD3   (channel 0 / FDCAN1)
 * LED2_RX  PA3   (channel 1 / FDCAN2)
 * LED2_TX  PA4   (channel 1 / FDCAN2)
 * UART_TX  PA9
 * UART_RX  PA10
 * USB_P    PA12
 * USB_N    PA11
 * CAN1_RX  PD0
 * CAN1_TX  PD1
 * CAN1_S   PA15
 * CAN2_RX  PB0
 * CAN2_TX  PB1
 * CAN2_S   PB2
 */

/* channel 0 (FDCAN1) LEDs */
#define LED1RX_GPIO_Port  GPIOD
#define LED1RX_Pin		  GPIO_PIN_2
#define LED1TX_GPIO_Port  GPIOD
#define LED1TX_Pin		  GPIO_PIN_3

/* channel 1 (FDCAN2) LEDs */
#define LED2RX_GPIO_Port  GPIOA
#define LED2RX_Pin		  GPIO_PIN_3
#define LED2TX_GPIO_Port  GPIOA
#define LED2TX_Pin		  GPIO_PIN_4

#define LED_Mode		  GPIO_MODE_OUTPUT_PP
#define LED_Active_High	  1

#ifdef CONFIG_BRAKE
/* E-STOP / BRAKE input (PB5) and LED_BRAKE indicator (PB6) */
#define BRAKE_GPIO_Port	  GPIOB
#define BRAKE_Pin		  GPIO_PIN_5
#define LEDBRAKE_GPIO_Port GPIOB
#define LEDBRAKE_Pin	  GPIO_PIN_6
#endif

static void candlelightfd_setup(USBD_GS_CAN_HandleTypeDef *hcan)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	UNUSED(hcan);

	__HAL_RCC_GPIOA_CLK_ENABLE();
#if NUM_CAN_CHANNEL == 2 || defined(CONFIG_BRAKE)
	__HAL_RCC_GPIOB_CLK_ENABLE();
#endif
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/* channel 0 LEDs (LED1: RX=PD2, TX=PD3) */
	HAL_GPIO_WritePin(LED1RX_GPIO_Port, LED1RX_Pin, GPIO_INIT_STATE(LED_Active_High));
	HAL_GPIO_WritePin(LED1TX_GPIO_Port, LED1TX_Pin, GPIO_INIT_STATE(LED_Active_High));
	GPIO_InitStruct.Pin = LED1RX_Pin | LED1TX_Pin;
	GPIO_InitStruct.Mode = LED_Mode;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LED1RX_GPIO_Port, &GPIO_InitStruct);

#if NUM_CAN_CHANNEL == 2
	/* channel 1 LEDs (LED2: RX=PA3, TX=PA4) */
	HAL_GPIO_WritePin(LED2RX_GPIO_Port, LED2RX_Pin, GPIO_INIT_STATE(LED_Active_High));
	HAL_GPIO_WritePin(LED2TX_GPIO_Port, LED2TX_Pin, GPIO_INIT_STATE(LED_Active_High));
	GPIO_InitStruct.Pin = LED2RX_Pin | LED2TX_Pin;
	GPIO_InitStruct.Mode = LED_Mode;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LED2RX_GPIO_Port, &GPIO_InitStruct);
#endif

	/* Setup transceiver silent pin */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
	GPIO_InitStruct.Pin = GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

#ifdef CONFIG_BRAKE
	/* BRAKE / E-STOP input PB5 (fail-safe N.C.: released=LOW, pressed=HIGH via R11) */
	GPIO_InitStruct.Pin = BRAKE_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(BRAKE_GPIO_Port, &GPIO_InitStruct);

	/* LED_BRAKE indicator PB6 (active high), off at start */
	HAL_GPIO_WritePin(LEDBRAKE_GPIO_Port, LEDBRAKE_Pin, GPIO_PIN_RESET);
	GPIO_InitStruct.Pin = LEDBRAKE_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LEDBRAKE_GPIO_Port, &GPIO_InitStruct);
#endif

#if NUM_CAN_CHANNEL == 2
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
	GPIO_InitStruct.Pin = GPIO_PIN_2;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
#endif

	/* FDCAN */

	RCC_PeriphCLKInitTypeDef PeriphClkInit = {
		.PeriphClockSelection = RCC_PERIPHCLK_FDCAN,
		.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL,
	};

	HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);
	__HAL_RCC_FDCAN_CLK_ENABLE();

	/* FDCAN1_RX, FDCAN1_TX */
	GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF3_FDCAN1;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

#if NUM_CAN_CHANNEL == 2
	/* FDCAN2_RX, FDCAN2_TX */
	GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF3_FDCAN2;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
#endif
}

const struct board_config config = {
	.setup = candlelightfd_setup,
	.channel[0] = {
		.interface = FDCAN1,
		.leds = {
			[LED_RX] = {
				.port = LED1RX_GPIO_Port,
				.pin = LED1RX_Pin,
				.active_high = LED_Active_High,
			},
			[LED_TX] = {
				.port = LED1TX_GPIO_Port,
				.pin = LED1TX_Pin,
				.active_high = LED_Active_High,
			},
		},
	},
#if NUM_CAN_CHANNEL == 2
	.channel[1] = {
		.interface = FDCAN2,
		.leds = {
			[LED_RX] = {
				.port = LED2RX_GPIO_Port,
				.pin = LED2RX_Pin,
				.active_high = LED_Active_High,
			},
			[LED_TX] = {
				.port = LED2TX_GPIO_Port,
				.pin = LED2TX_Pin,
				.active_high = LED_Active_High,
			},
		},
	},
#endif
#ifdef CONFIG_BRAKE
	.brake = {
		.button_port = BRAKE_GPIO_Port,
		.button_pin = BRAKE_Pin,
		.led_port = LEDBRAKE_GPIO_Port,
		.led_pin = LEDBRAKE_Pin,
	},
#endif
};
