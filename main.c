/*
 * Copyright 2016-2019 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
 
/**
 * @file    Leds_Project.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK64F12.h"
#include "fsl_debug_console.h"
/* TODO: insert other include files here. */
#include "bits.h"
#include "gpio.h"
/* TODO: insert other definitions and declarations here. */
//#define gpio
#define rtos_gpio
/*
 * @brief   Application entry point.
 */
int main(void) {

  	/* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
  	/* Init FSL debug console. */
    BOARD_InitDebugConsole();

    PRINTF("Hello World\n");

    /*gpio made by us*/
#ifdef gpio
    GPIO_clock_gating(GPIO_A);
    GPIO_clock_gating(GPIO_B);
    GPIO_clock_gating(GPIO_E);

    gpio_pin_control_register_t config_led_config = GPIO_MUX1 | GPIO_PS | GPIO_PE | INTR_FALLING_EDGE;
    gpio_pin_control_register_t output_led_config = GPIO_MUX1;

	/**Pin control configuration of GPIOB pin22 as GPIO*/
	GPIO_pin_control_register(GPIO_B,bit_22,&output_led_config);
	/**Pin control configuration of GPIOB pin21 as GPIO*/
	GPIO_pin_control_register(GPIO_B,bit_21,&output_led_config);
	/**Pin control configuration of GPIOE pin26 as GPIO*/
	GPIO_pin_control_register(GPIO_E,bit_26,&output_led_config);
#endif

#ifdef rtos_gpio
    /*gpio of fsl_port.h*/
	port_pin_config_t rtos_pin_led_config =
	{
			kPORT_PullDisable,
			kPORT_SlowSlewRate,
			kPORT_PassiveFilterDisable,
			kPORT_OpenDrainDisable,
			kPORT_LowDriveStrength,
			kPORT_MuxAsGpio,
			kPORT_UnlockRegister
	};
	port_pin_config_t rtos__pin_switch_config =
	{
			kPORT_PullDisable,
			kPORT_SlowSlewRate,
			kPORT_PassiveFilterDisable,
			kPORT_OpenDrainDisable,
			kPORT_LowDriveStrength,
			kPORT_MuxAsGpio,
			kPORT_UnlockRegister
	};

	gpio_pin_config_t rtos_gpio_led_config =
	{
			kGPIO_DigitalOutput,
			1
	};
	gpio_pin_config_t rtos_gpio_switch_config =
	{
			kGPIO_DigitalInput,
			1
	};

	NVIC_EnableIRQ(PORTA_IRQn);

	PORT_SetPinConfig(PORTB, 21, &rtos_pin_led_config);
	PORT_SetPinConfig(PORTA, 4, &rtos__pin_switch_config);

	PORT_SetPinInterruptConfig(PORTA, 4, kPORT_InterruptFallingEdge);

	GPIO_PinInit(GPIOB, 21, &rtos_gpio_led_config);
	GPIO_PinInit(GPIOA, 4, &rtos_gpio_switch_config);

	GPIO_PinWrite(GPIOB,bit_21,0);
#endif


    /* Enter an infinite loop, just incrementing a counter. */
    for(;;)
    {

    }
    return 0 ;
}
