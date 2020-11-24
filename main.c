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
#include "USB_API/USB_WPAN_API/UsbWpan.h"
#include "USB_app/usbConstructs.h"

#include "ringbuf.h"

#define MCLK_FREQUENCY 24000000

//UART config for Baudrate 115200
#if   MCLK_FREQUENCY == 20000000
#define UART_PRESCALAR 173
#define UART_FIRST_MOD   0
#define UART_SECOND_MOD  5
#elif MCLK_FREQUENCY == 24000000
#define UART_PRESCALAR 208
#define UART_FIRST_MOD   0
#define UART_SECOND_MOD  3
#else
#error INVALID MCLK_FREQUENCY 
#endif

#define CRC_DEBUG
//#define SKIP_CRC

// Global flags set by events
volatile uint8_t bCDCDataReceived_event = FALSE;  // Flag set by event handler to
                                               // indicate data has been
                                               // received into USB buffer
volatile uint8_t bCDCBreak_event = 0;             // Flag set by event handler to
                                               // indicate break has been
                                               // received by USB
                                               // 0x01 for SET, 0x02 for CLEAR
volatile uint8_t bWPANDataReceived_event = FALSE;
volatile uint8_t bUSBIE = 0;

#define BUFFER_SIZE 768

static uint8_t rxBuffer[BUFFER_SIZE];
static uint8_t txBuffer[BUFFER_SIZE];

static uint8_t uartRingBuffer[BUFFER_SIZE];
static ringbuf_t uartRing;

static uint8_t userBootState = 1;

static volatile uint8_t usbError = 0;
static volatile uint8_t uartError = 0;
static volatile uint8_t uartRxOverflow = 0;

#define ADDRESS_WPAN 0x03
#define ADDRESS_CDC  0x05
#define ADDRESS_HW   0x41

static uint16_t bytesSent, bytesReceived;

static enum _rx_state {
    STATE_LOOK_FOR_HDLC,
    STATE_HDLC,
    STATE_PASSTHROUGH
} rxState = STATE_LOOK_FOR_HDLC;

void poll_hdlc(int reset)
{
    static uint8_t inEsc = FALSE;
    static uint8_t currentAddress = 0xFF;
    static uint16_t currentOffset = 0;
#ifdef CRC_DEBUG
    uint8_t CRC_DEBUG_ERROR = 0;
#endif
    uint16_t crc_check = 0;

    if (reset) {
        inEsc = FALSE;
        currentAddress = 0xFF;
        currentOffset = 0;
        RINGBUF_flush(&uartRing);
        return;
    }

    while(!bWPANDataReceived_event && !RINGBUF_empty(&uartRing)) {
        if(currentAddress == ADDRESS_WPAN) {
            if(USBWPAN_getInterfaceStatus(WPAN0_INTFNUM) & USBWPAN_WAITING_FOR_SEND) {
                return;
            }
        } else if(currentAddress == ADDRESS_CDC) {
            if(USBCDC_getInterfaceStatus(CDC0_INTFNUM,&bytesSent,&bytesReceived) & USBCDC_WAITING_FOR_SEND) {
                return;
            }
        }

        uint8_t c = RINGBUF_pop_unsafe(&uartRing);
        
        if(c == HDLC_FRAME) {
            if(currentAddress != 0xFF) {
#ifdef SKIP_CRC
                crc_check = 0xf0b8;
#else
                CRC_setSeed(CRC_BASE, 0xffff);
                CRC_set8BitData(CRC_BASE, currentAddress);
                for(int i=0; i<currentOffset; i++) {
                    CRC_set8BitData(CRC_BASE, rxBuffer[i]);
                }
                crc_check = CRC_getResultBitsReversed(CRC_BASE);
#endif
                if(crc_check != 0xf0b8) {
#ifdef CRC_DEBUG
                    CRC_DEBUG_ERROR = 1;
#endif
                } else {
                    rxState = STATE_HDLC;

                    if((rxBuffer[0] & 1) == 0) {
                        //I-Frame, send S-Frame ACK
                        USBWPAN_sendAck(currentAddress, (rxBuffer[0] >> 1) & 0x7);
                    }

                    if(currentAddress == ADDRESS_WPAN) {
                        USBWPAN_sendData(rxBuffer+1, currentOffset-3, WPAN0_INTFNUM);
                    }
                    else if(currentAddress == ADDRESS_CDC) {
                        USBCDC_sendData(rxBuffer+1, currentOffset - 3, CDC0_INTFNUM);
                    }
                }
            }

#ifdef CRC_DEBUG
            if (CRC_DEBUG_ERROR) {
                USCI_A_UART_transmitData(UART_BRIDGE, HDLC_FRAME);
                USCI_A_UART_transmitData(UART_BRIDGE, 0xEE);
                USCI_A_UART_transmitData(UART_BRIDGE, CRC_DEBUG_ERROR);
                USCI_A_UART_transmitData(UART_BRIDGE, currentAddress);
                USCI_A_UART_transmitData(UART_BRIDGE, currentOffset);
                USCI_A_UART_transmitData(UART_BRIDGE, uartError);
                USCI_A_UART_transmitData(UART_BRIDGE, uartRxOverflow);
                USCI_A_UART_transmitData(UART_BRIDGE, crc_check >> 8);
                USCI_A_UART_transmitData(UART_BRIDGE, crc_check & 0xFF);
                USCI_A_UART_transmitData(UART_BRIDGE, HDLC_FRAME);
                uartError = 0;
                uartRxOverflow = 0;
            }
#endif
            currentOffset = 0;
            currentAddress = 0xFF;
        } else if(c == HDLC_ESC) {
            inEsc = TRUE;
        } else {
            if(inEsc) {
                c ^= 0x20;
                inEsc = FALSE;
            }

            if(currentAddress == 0xFF) {
                currentAddress = c;
                if(currentAddress == ADDRESS_WPAN || 
                   currentAddress == ADDRESS_CDC ||
                   currentAddress == ADDRESS_HW) {
                } else {
                    currentAddress = 0xFF;
                }
                currentOffset = 0;
            } else {
                if(rxState == STATE_LOOK_FOR_HDLC && currentOffset > 10) {
                    rxState = STATE_PASSTHROUGH;
                    USBCDC_sendData(txBuffer, currentOffset, CDC0_INTFNUM);
                    return;
                }
                else if(currentOffset < BUFFER_SIZE) {
                    rxBuffer[currentOffset] = c;
                    currentOffset++;
                } else {
                    //buffer overflow
                    currentAddress = 0xFF;
                    currentOffset = 0;
                }
            }
        }
    }
}

void poll_passthrough(void)
{
    uint8_t sendError = 0;
    uint16_t count = 0;

    if (bCDCDataReceived_event){
        bCDCDataReceived_event = FALSE;

        count = USBCDC_receiveDataInBuffer((uint8_t*)rxBuffer, BUFFER_SIZE, CDC0_INTFNUM);

        uint8_t *pBuffer = rxBuffer;
        for(int i=count; i; i--) {
            USCI_A_UART_transmitData(UART_BRIDGE, *pBuffer++);
        }
    }

    if(!RINGBUF_empty(&uartRing) &&
       !(USBCDC_getInterfaceStatus(CDC0_INTFNUM, &bytesSent, &bytesReceived) & USBCDC_WAITING_FOR_SEND)) {
        count = RINGBUF_receiveDataInBuffer(&uartRing, txBuffer, BUFFER_SIZE);
        sendError = USBCDC_sendData(txBuffer, count, CDC0_INTFNUM);
        if(sendError != USBCDC_SEND_STARTED) {
            usbError = sendError;
            RINGBUF_flush(&uartRing);
        }
    }
}

int main(void)
{
	WDTCTL = WDTPW | WDTHOLD;	// stop watchdog timer
	
    // Use Level 3 for high speed system clock
    PMM_setVCore(PMM_CORE_LEVEL_3);
    hal_init(MCLK_FREQUENCY); //MCLK=SMCLK=FLL=MCLK_FREQUENCY; ACLK=REFO=32kHz
    USB_setup(TRUE, TRUE); // Init USB & events; if a host is present, connect

    USCI_A_UART_initParam param = {0};
    param.selectClockSource = USCI_A_UART_CLOCKSOURCE_SMCLK;
    param.clockPrescalar    = UART_PRESCALAR;
    param.firstModReg       = UART_FIRST_MOD;
    param.secondModReg      = UART_SECOND_MOD;
    param.parity            = USCI_A_UART_NO_PARITY;
    param.msborLsbFirst     = USCI_A_UART_LSB_FIRST;
    param.numberofStopBits  = USCI_A_UART_ONE_STOP_BIT;
    param.uartMode          = USCI_A_UART_MODE;
    param.overSampling      = USCI_A_UART_LOW_FREQUENCY_BAUDRATE_GENERATION;

    if (STATUS_FAIL == USCI_A_UART_init(UART_BRIDGE, &param)){
        return 1;
    }

    RINGBUF_init(&uartRing, (uint8_t*)&uartRingBuffer, BUFFER_SIZE);

    //Enable UART module for operation
    USCI_A_UART_enable(UART_BRIDGE);

    //Enable Receive Interrupt
    USCI_A_UART_clearInterrupt(UART_BRIDGE, USCI_A_UART_RECEIVE_INTERRUPT);

    __enable_interrupt();  // Enable interrupts globally

    USCI_A_UART_enableInterrupt(UART_BRIDGE, USCI_A_UART_RECEIVE_INTERRUPT);

    rxState = STATE_LOOK_FOR_HDLC;

    uint8_t userBootNow = 1;

    while (1)
    {
        if(bCDCBreak_event == 3) {
            //Break Set/Cleared, jump to BSL
//            __disable_interrupt(); // disable interrupts
//            ((void ( * )())0x1000)(); // jump to BSL

            //Reset CC1352R into BSL mode
            bCDCBreak_event = 0;
            hal_ext_reset(TRUE);
            hal_ext_boot(TRUE);
            __delay_cycles(2000);

            rxState = STATE_PASSTHROUGH;
            userBootState = 1;
            userBootNow = 1;
            if(bWPANDataReceived_event) {
                USBWPAN_reset(); //auto-clear flag
            }
            poll_hdlc(TRUE);

            hal_ext_reset(FALSE);
            __delay_cycles(60000);
            hal_ext_boot(FALSE);
        }

        switch (USB_getConnectionState())
        {
            case ST_ENUM_ACTIVE:
                hal_ext_uart(TRUE);

                userBootNow = hal_ext_boot_read();

                if(userBootState == 0 && userBootNow == 1) {
                    //0 -> 1 transition, button released, passthrough
                    rxState = STATE_PASSTHROUGH;
                    if(bWPANDataReceived_event) {
                        USBWPAN_reset(); //auto-clear flag
                    }
                } else if(userBootState == 1 && userBootNow == 0) {
                    //1 -> 0 transition, button pressed, look for HDLC
                    rxState = STATE_LOOK_FOR_HDLC;
                }
                userBootState = userBootNow;

                if (rxState == STATE_PASSTHROUGH) {
                    poll_passthrough();
                } else {
                    // EP0 packet received, send out over UART
                    if(bWPANDataReceived_event) {
                        USBWPAN_sendPacket(); //auto-clear flag
                    }
                    poll_hdlc(FALSE);
                }
                break;

            case ST_PHYS_DISCONNECTED:
            case ST_ENUM_SUSPENDED:
            case ST_PHYS_CONNECTED_NOENUM_SUSP:
                hal_ext_uart(FALSE);
                poll_hdlc(TRUE);
                __bis_SR_register(LPM3_bits + GIE);
                _NOP();
                break;

            case ST_ENUM_IN_PROGRESS:
            default:
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

    switch (__even_in_range(UCA_BRIDGE_IV,4)){
        //Vector 2 - RXIFG
        case 2:
            uartError = USCI_A_UART_queryStatusFlags(UART_BRIDGE, USCI_A_UART_FRAMING_ERROR|USCI_A_UART_OVERRUN_ERROR|USCI_A_UART_PARITY_ERROR|USCI_A_UART_RECEIVE_ERROR);
            rx = USCI_A_UART_receiveData(UART_BRIDGE);

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
