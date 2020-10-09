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
 * main.c
 *
 *  Created on: May 7, 2020
 *      Author: Erik Larson
 *
 */

#include <msp430.h> 
#include <string.h>

#include "driverlib.h"
#include "hal.h"

#include "USB_config/descriptors.h"
#include "USB_API/USB_Common/device.h"
#include "USB_API/USB_Common/usb.h"
#include "USB_API/USB_CDC_API/UsbCdc.h"
#include "USB_app/usbConstructs.h"

#include "ringbuf.h"

#define MCLK_FREQUENCY 20000000

// Global flags set by events
volatile uint8_t bCDCDataReceived_event = FALSE;  // Flag set by event handler to
                                               // indicate data has been
                                               // received into USB buffer
volatile uint8_t bCDCBreak_event = 0;             // Flag set by event handler to
                                               // indicate break has been
                                               // received by USB
                                               // 0x01 for SET, 0x02 for CLEAR

//#define BUFFER_SIZE 1024
#define BUFFER_SIZE 512
static uint8_t usbRxBuffer[BUFFER_SIZE];
static uint8_t usbTxBuffer[BUFFER_SIZE];
static uint8_t uartRingBuffer[BUFFER_SIZE];
static ringbuf_t uartRing;

static volatile uint8_t usbError = 0;
static volatile uint8_t uartError = 0;
static volatile uint16_t uartRxOverflow = 0;

int main(void)
{
	WDTCTL = WDTPW | WDTHOLD;	// stop watchdog timer
	
    // Minimum Vcore setting required for the USB API is PMM_CORE_LEVEL_2 .
    PMM_setVCore(PMM_CORE_LEVEL_2);
    hal_init(MCLK_FREQUENCY); //MCLK=SMCLK=FLL=MCLK_FREQUENCY; ACLK=REFO=32kHz
    USB_setup(TRUE, TRUE); // Init USB & events; if a host is present, connect

    //Baudrate = 115200, clock freq = 8.000MHz
    //UCBRx = 69, UCBRFx = 0, UCBRSx = 4, UCOS16 = 0
    //Baudrate = 115200, clock freq = 16.000MHz
    //UCBRx = 138, UCBRFx = 0, UCBRSx = 7, UCOS16 = 0
    //Baudrate = 115200, clock freq = 20.000MHz
    //UCBRx = 173, UCBRFx = 0, UCBRSx = 5, UCOS16 = 0
    USCI_A_UART_initParam param = {0};
    param.selectClockSource = USCI_A_UART_CLOCKSOURCE_SMCLK;
    param.clockPrescalar = 173;
    param.firstModReg = 0;
    param.secondModReg = 5;
    param.parity = USCI_A_UART_NO_PARITY;
    param.msborLsbFirst = USCI_A_UART_LSB_FIRST;
    param.numberofStopBits = USCI_A_UART_ONE_STOP_BIT;
    param.uartMode = USCI_A_UART_MODE;
    param.overSampling = USCI_A_UART_LOW_FREQUENCY_BAUDRATE_GENERATION;

    if (STATUS_FAIL == USCI_A_UART_init(UART_BRIDGE, &param)){
        return 1;
    }

    RINGBUF_init(&uartRing, (uint8_t*)&uartRingBuffer, BUFFER_SIZE);

    //Enable UART module for operation
    USCI_A_UART_enable(UART_BRIDGE);

    //Enable Receive Interrupt
    USCI_A_UART_clearInterrupt(UART_BRIDGE, USCI_A_UART_RECEIVE_INTERRUPT);

    __enable_interrupt();  // Enable interrupts globally

    while (1)
    {
        uint8_t sendError;
        uint16_t count;
        uint16_t bytesSent, bytesReceived;


        if(bCDCBreak_event == 3) {
            //Break Set/Cleared, jump to BSL
//            __disable_interrupt(); // disable interrupts
//            ((void ( * )())0x1000)(); // jump to BSL

            //Reset CC1352R into BSL mode
            bCDCBreak_event = 0;
            hal_ext_reset(TRUE);
            hal_ext_boot(TRUE);
            __delay_cycles(2000);
            hal_ext_reset(FALSE);
            __delay_cycles(60000);
            hal_ext_boot(FALSE);
        }

        switch (USB_getConnectionState())
        {
            case ST_ENUM_ACTIVE:
                hal_ext_uart(TRUE);
//                USCI_A_UART_enableInterrupt(UART_BRIDGE, USCI_A_UART_RECEIVE_INTERRUPT);


                if (bCDCDataReceived_event){
                    bCDCDataReceived_event = FALSE;

                    count = USBCDC_receiveDataInBuffer((uint8_t*)usbRxBuffer, BUFFER_SIZE, CDC0_INTFNUM);

                    uint8_t *txBuffer = usbRxBuffer;
                    for(int i=count; i; i--) {
                        USCI_A_UART_transmitData(UART_BRIDGE, *txBuffer++);
                    }
                }

                if(!RINGBUF_empty(&uartRing) && !(USBCDC_getInterfaceStatus(CDC0_INTFNUM,&bytesSent,&bytesReceived) & USBCDC_WAITING_FOR_SEND)) {
                    count = RINGBUF_receiveDataInBuffer(&uartRing, usbTxBuffer, BUFFER_SIZE);
                    sendError = USBCDC_sendData(usbTxBuffer,count,CDC0_INTFNUM);
                    if(sendError != USBCDC_SEND_STARTED) {
                        usbError = sendError;
                        RINGBUF_flush(&uartRing);
                    }
                }
                break;

            case ST_PHYS_DISCONNECTED:
            case ST_ENUM_SUSPENDED:
            case ST_PHYS_CONNECTED_NOENUM_SUSP:
                //Deep sleep until active
//                USCI_A_UART_disableInterrupt(UART_BRIDGE, USCI_A_UART_RECEIVE_INTERRUPT);
                hal_ext_uart(FALSE);
//                hal_ext_boot(FALSE);
//                hal_ext_reset(FALSE);
                RINGBUF_flush(&uartRing);
                __bis_SR_register(LPM3_bits + GIE);
                _NOP();
                break;

            case ST_ENUM_IN_PROGRESS:
            default:
//                hal_ext_uart(FALSE);
//                hal_ext_boot(FALSE);
//                hal_ext_reset(FALSE);
                break;
        }
    }
}

/*
 * ======== UNMI_ISR ========
 */
#if defined(__TI_COMPILER_VERSION__)
#pragma vector = UNMI_VECTOR
__interrupt void UNMI_ISR (void)
#elif defined(__GNUC__)
void __attribute__((interrupt(UNMI_VECTOR))) UNMI_ISR (void)
#else
#error Compiler not supported
#endif
{
    switch (__even_in_range(SYSUNIV, SYSUNIV_BUSIFG ))
    {
        case SYSUNIV_NONE:
            __no_operation();
            break;
        case SYSUNIV_NMIIFG:
            __no_operation();
            break;
        case SYSUNIV_OFIFG:
            UCS_clearFaultFlag(UCS_XT2OFFG);
            UCS_clearFaultFlag(UCS_DCOFFG);
            SFR_clearInterrupt(SFR_OSCILLATOR_FAULT_INTERRUPT);
            break;
        case SYSUNIV_ACCVIFG:
            __no_operation();
            break;
        case SYSUNIV_BUSIFG:
            // If the CPU accesses USB memory while the USB module is
            // suspended, a "bus error" can occur.  This generates an NMI.  If
            // USB is automatically disconnecting in your software, set a
            // breakpoint here and see if execution hits it.  See the
            // Programmer's Guide for more information.
            SYSBERRIV = 0; // clear bus error flag
            USB_disable(); // Disable
    }
}

#if defined(__TI_COMPILER_VERSION__)
#if defined (BRIDGE_UART0)
#pragma vector=USCI_A0_VECTOR
#else
#pragma vector=USCI_A1_VECTOR
#endif
__interrupt void 
#elif defined(__GNUC__)
  #if defined (BRIDGE_UART0)
void __attribute__((interrupt(USCI_A0_VECTOR)))
  #else
void __attribute__((interrupt(USCI_A1_VECTOR))) 
  #endif
#else
#error Compiler not supported
#endif
USCI_BRIDGE_ISR (void)
{
    uint8_t rx = 0;
    uint8_t flags = 0;
    switch (__even_in_range(UCA_BRIDGE_IV,4)){
        //Vector 2 - RXIFG
        case 2:
            flags = USCI_A_UART_queryStatusFlags(UART_BRIDGE, USCI_A_UART_FRAMING_ERROR|USCI_A_UART_OVERRUN_ERROR|USCI_A_UART_PARITY_ERROR|USCI_A_UART_RECEIVE_ERROR);
            rx = USCI_A_UART_receiveData(UART_BRIDGE);

            if(USCI_A_UART_queryStatusFlags(UART_BRIDGE, USCI_A_UART_OVERRUN_ERROR) | flags) {
                uartError = flags;
            }
            if(!RINGBUF_push(&uartRing, rx)) {
                uartRxOverflow = 1;
            }
            //Wake up, data to process
            __bic_SR_register_on_exit(LPM3_bits);
            __no_operation();
            break;
        default: break;
    }
}
