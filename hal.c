/*
 * BSD 3-Clause License
 * 
 * Copyright (c) 2020, Erik Larson
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * hal.c
 *
 *  Created on: May 7, 2020
 *      Author: Erik Larson
 */
#include "msp430.h"

#include "driverlib.h"

#include "hal.h"

void hal_init(uint32_t mclkFreq)
{
#ifdef LAUNCHPAD
    //Ignore unused pins, don't care about low power state for Launchpad

//    //CC1352R RESET_N P2.0 - Default as input, set output low for reset operation
//    //CC1352R BOOT_N  P2.2 - Default as input, set output low for boot operation
//    GPIO_setAsInputPin(    GPIO_PORT_P2, GPIO_PIN0|GPIO_PIN2);
//    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0|GPIO_PIN2);

    //Need pull-up on BOOT pin to prevent inadvertent BSL entry, not needed on target board
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P2, GPIO_PIN0|GPIO_PIN2);

    //CC1352R UART
#if defined (BRIDGE_UART0)
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P3, GPIO_PIN3 | GPIO_PIN4);
#else
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4, GPIO_PIN4 | GPIO_PIN5);
#endif

    //CC1352R UART CTRL - default disconnected, HIGH to connect (LED Emulated)
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN5);
    GPIO_setAsOutputPin(   GPIO_PORT_P2, GPIO_PIN5);

    //DEBUG LED (RED)
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
    GPIO_setAsOutputPin(   GPIO_PORT_P1, GPIO_PIN0);

    //DEBUG LED (GREEN)
    //GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN7);
    GPIO_setAsOutputPin(   GPIO_PORT_P4, GPIO_PIN7);

#else
    //Unused pins, low power state
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3);
    GPIO_setAsOutputPin(   GPIO_PORT_P1, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3);
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3);
    GPIO_setAsOutputPin(   GPIO_PORT_P4, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3);
    GPIO_setOutputLowOnPin(GPIO_PORT_PJ, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3);
    GPIO_setAsOutputPin(   GPIO_PORT_PJ, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3);
    GPIO_setOutputLowOnPin(GPIO_PORT_P6, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3);
    GPIO_setAsOutputPin(   GPIO_PORT_P6, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3);

    //SPI, Not used by MSP430
    GPIO_setAsInputPin(GPIO_PORT_P1, GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);

    //CC1352R RESET_N - Default as input, set output low for reset operation
    GPIO_setAsInputPin(    GPIO_PORT_P2, GPIO_PIN0);
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0);

    //CC1352R UART
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4, GPIO_PIN4 | GPIO_PIN5);

    //CC1352R BOOT_N - Default as input, set output low for boot operation
    GPIO_setAsInputPin(    GPIO_PORT_P4, GPIO_PIN7);

    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN6|GPIO_PIN7);

    //CC1352R UART CTRL - default disconnected, HIGH to connect
    GPIO_setAsOutputPin(   GPIO_PORT_P4, GPIO_PIN6);


#endif //LAUNCHPAD
    UCS_initClockSignal(
       UCS_FLLREF,
       UCS_REFOCLK_SELECT,
       UCS_CLOCK_DIVIDER_1);

    UCS_initClockSignal(
       UCS_ACLK,
       UCS_REFOCLK_SELECT,
       UCS_CLOCK_DIVIDER_1);

    UCS_initFLLSettle(
        mclkFreq/1000,
        mclkFreq/32768);
}

#ifdef LAUNCHPAD
void hal_ext_boot(uint8_t active)
{
    if(active) {
        GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN2);
        GPIO_setAsOutputPin(   GPIO_PORT_P2, GPIO_PIN2);
    } else {
        GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P2, GPIO_PIN2);
    }
}

uint8_t hal_ext_boot_read(void)
{
    return GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN2);
}

void hal_ext_reset(uint8_t active)
{
    if(active) {
        GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0);
        GPIO_setAsOutputPin(   GPIO_PORT_P2, GPIO_PIN0);

    } else {
        GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P2, GPIO_PIN0);
    }
}

void hal_ext_uart(uint8_t active)
{
    if(active) {
        GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN5);
        GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);

        USCI_A_UART_enableInterrupt(UART_BRIDGE, USCI_A_UART_RECEIVE_INTERRUPT);
    } else {
        GPIO_setOutputLowOnPin( GPIO_PORT_P2, GPIO_PIN5);
        GPIO_setOutputLowOnPin( GPIO_PORT_P1, GPIO_PIN0);

        USCI_A_UART_disableInterrupt(UART_BRIDGE, USCI_A_UART_RECEIVE_INTERRUPT);
    }
}
#else
void hal_ext_boot(uint8_t active)
{
    if(active) {
        GPIO_setAsOutputPin(   GPIO_PORT_P4, GPIO_PIN7);
    } else {
        GPIO_setAsInputPin(    GPIO_PORT_P4, GPIO_PIN7);
    }
}

uint8_t hal_ext_boot_read(void)
{
    return GPIO_getInputPinValue(GPIO_PORT_P4, GPIO_PIN7);
}

void hal_ext_reset(uint8_t active)
{
    if(active) {
        GPIO_setAsOutputPin(   GPIO_PORT_P2, GPIO_PIN0);
    } else {
        GPIO_setAsInputPin(    GPIO_PORT_P2, GPIO_PIN0);
    }
}

void hal_ext_uart(uint8_t active)
{
    if(active) {
        GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN6);
        USCI_A_UART_enableInterrupt(UART_BRIDGE, USCI_A_UART_RECEIVE_INTERRUPT);
    } else {
        GPIO_setOutputLowOnPin( GPIO_PORT_P4, GPIO_PIN6);
        USCI_A_UART_disableInterrupt(UART_BRIDGE, USCI_A_UART_RECEIVE_INTERRUPT);
    }
}
#endif //LAUNCHPAD
