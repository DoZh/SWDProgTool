/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2011  Black Sphere Technologies Ltd.
 * Written by Gareth McMullin <gareth@blacksphere.co.nz>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/* This file implements the platform specific functions for the STM32
 * implementation.
 */
#ifndef __PLATFORM_H
#define __PLATFORM_H

#include "gpio.h"
//#include "timing.h"
//#include "timing_stm32.h"
//#include "version.h"

#include <setjmp.h>

#define PLATFORM_HAS_TRACESWO
#define BOARD_IDENT "Black Magic Probe (F4Discovery), (Firmware " FIRMWARE_VERSION ")"
#define DFU_IDENT   "Black Magic Firmware Upgrade (F4Discovery)"

/* Important pin mappings for STM32 implementation:
 *
 * LED0 = 	PD12	(Green  LED : Running)
 * LED1 = 	PD13	(Orange LED : Idle)
 * LED2 = 	PD12	(Red LED    : Error)
 * LED3 = 	PD15	(Blue LED   : Bootloader active)
 *
 * nTRST = 	PC1
 * SRST_OUT =   PC8
 * TDI = 	PC2
 * TMS = 	PC4 (input for SWDP)
 * TCK = 	PC5/SWCLK
 * TDO = 	PC6 (input for TRACESWO
 * nSRST =
 *
 * Force DFU mode button: PA0
 */

/* Hardware definitions... */
#define JTAG_PORT 	Target_SWDCLK_GPIO_Port
#define TDI_PORT	Target_TDI_GPIO_Port
#define TMS_PORT	Target_SWDIO_GPIO_Port
#define TCK_PORT	Target_SWDCLK_GPIO_Port
#define TDO_PORT	Target_TDO_GPIO_Port
#define TDI_PIN		Target_TDI_Pin
#define TMS_PIN		Target_SWDIO_Pin
#define TCK_PIN		Target_SWDCLK_Pin
#define TDO_PIN		Target_TDO_Pin

#define SWDIO_PORT 	TMS_PORT
#define SWCLK_PORT 	TCK_PORT
#define SWDIO_PIN	TMS_PIN
#define SWCLK_PIN	TCK_PIN

#define SWDIO_DIR_PORT Target_DIR_GPIO_Port
#define SWDIO_DIR_PIN Target_DIR_Pin

#define TRST_PORT	GPIOC
#define TRST_PIN	GPIO_PIN_1
#define SRST_PORT	GPIOC
#define SRST_PIN	GPIO_PIN_8

#define LED_PORT	LED1_GPIO_Port
#define LED_PORT_UART	LED0_GPIO_Port
#define LED_UART	LED0_Pin
#define LED_IDLE_RUN	LED1_Pin
#define LED_ERROR	LED2_Pin
#define LED_BOOTLOADER	LED3_Pin
#define BOOTMAGIC0 0xb007da7a
#define BOOTMAGIC1 0xbaadfeed

#define TMS_SET_MODE() \
	gpio_mode_setup(TMS_PORT, GPIO_MODE_OUTPUT_PP, \
	                GPIO_PULLDOWN, TMS_PIN);
#define SWDIO_MODE_FLOAT() do{ \
	gpio_mode_setup(SWDIO_PORT, GPIO_MODE_INPUT, \
	                GPIO_PULLUP, SWDIO_PIN); \
	gpio_clear(SWDIO_DIR_PORT, SWDIO_DIR_PIN); \
} while(0)

#define SWDIO_MODE_DRIVE() do{ \
	gpio_set(SWDIO_DIR_PORT, SWDIO_DIR_PIN); \
	gpio_mode_setup(SWDIO_PORT, GPIO_MODE_OUTPUT_PP, \
	                GPIO_PULLUP, SWDIO_PIN); \
} while(0)


#define USB_DRIVER      stm32f107_usb_driver
#define USB_IRQ         NVIC_OTG_FS_IRQ
#define USB_ISR         otg_fs_isr
/* Interrupt priorities.  Low numbers are high priority.
 * For now USART1 preempts USB which may spin while buffer is drained.
 * TIM3 is used for traceswo capture and must be highest priority.
 */
#define IRQ_PRI_USB		(2 << 4)
#define IRQ_PRI_USBUSART	(1 << 4)
#define IRQ_PRI_USBUSART_TIM	(3 << 4)
#define IRQ_PRI_TRACE		(0 << 4)

#define USBUSART USART3
#define USBUSART_CR1 USART3_CR1
#define USBUSART_IRQ NVIC_USART3_IRQ
#define USBUSART_CLK RCC_USART3
#define USBUSART_TX_PORT GPIOD
#define USBUSART_TX_PIN  GPIO8
#define USBUSART_RX_PORT GPIOD
#define USBUSART_RX_PIN  GPIO9
#define USBUSART_ISR usart3_isr
#define USBUSART_TIM TIM4
#define USBUSART_TIM_CLK_EN() rcc_periph_clock_enable(RCC_TIM4)
#define USBUSART_TIM_IRQ NVIC_TIM4_IRQ
#define USBUSART_TIM_ISR tim4_isr

#define UART_PIN_SETUP() do { \
	gpio_mode_setup(USBUSART_TX_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, \
	                USBUSART_TX_PIN); \
	gpio_mode_setup(USBUSART_RX_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, \
	                USBUSART_RX_PIN); \
	gpio_set_af(USBUSART_TX_PORT, GPIO_AF7, USBUSART_TX_PIN); \
	gpio_set_af(USBUSART_RX_PORT, GPIO_AF7, USBUSART_RX_PIN); \
    } while(0)

#define TRACE_TIM TIM3
#define TRACE_TIM_CLK_EN() rcc_periph_clock_enable(RCC_TIM3)
#define TRACE_IRQ   NVIC_TIM3_IRQ
#define TRACE_ISR   tim3_isr

#define DEBUG(...)

#define gpio_set_val(port, pin, val) do {	\
	if(val)					\
		gpio_set((port), (pin));	\
	else					\
		gpio_clear((port), (pin));	\
} while(0)

#define SET_RUN_STATE(state)	{running_status = (state);}
#define SET_IDLE_STATE(state)	{gpio_set_val(LED_PORT, LED_IDLE_RUN, state);}
#define SET_ERROR_STATE(state)	{gpio_set_val(LED_PORT, LED_ERROR, state);}

static inline int platform_hwversion(void)
{
	return 0;
}

/* Use newlib provided integer only stdio functions */
//#define sscanf siscanf
//#define sprintf siprintf
//#define vasprintf vasiprintf
//#define snprintf sniprintf

#endif
