/* --COPYRIGHT--,BSD
 * Copyright (c) 2020, Erik Larson
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/

/*
 * ======== UsbWpan.c ========
 */
#include <descriptors.h>

#include <stddef.h>

#ifdef _WPAN_

#include "../USB_Common/device.h"
#include "../USB_Common/defMSP430USB.h"
#include "../USB_Common/usb.h"                  //USB-specific Data Structures
#include "../USB_WPAN_API/UsbWpan.h"

#include <string.h>

#include "driverlib.h"
#include "hal.h"

#define INTFNUM_OFFSET(X)   (X - WPAN0_INTFNUM)  //Get the CDC offset

#define EP_MAX_PACKET_SIZE_WPAN      64
#define MAX_CONTROL_DATA_SIZE_WPAN  256

extern uint16_t wUsbEventMask;

extern void *(*USB_TX_memcpy)(void * dest, const void * source, size_t count);
int16_t WpanToHostFromBuffer(uint8_t intfNum);

typedef struct {
    uint16_t nBytesToSend;                       //holds counter of bytes to be sent
    uint16_t nBytesToSendLeft;                   //holds counter how many bytes is still to be sent
    const uint8_t* pUsbBufferToSend;             //holds the buffer with data to be sent
    uint8_t bCurrentBufferXY;                    //is 0 if current buffer to write data is X, or 1 if current buffer is Y
    uint8_t bZeroPacketSent;                     //= FALSE;
    uint8_t last_ByteSend;
} WpanWrite;

WpanWrite WpanWriteCtrl[WPAN_NUM_INTERFACES];

uint8_t usbRequestIncomingData[MAX_CONTROL_DATA_SIZE_WPAN];
uint16_t usbRequestIncomingLength = 0;

extern __no_init tEDB __data16 tInputEndPointDescriptorBlock[];

void WpanResetData ()
{
    memset(&WpanWriteCtrl, 0, sizeof(WpanWriteCtrl));
}

void sendFrameByte(void)
{
    USCI_A_UART_transmitData(UART_BRIDGE, HDLC_FRAME);
}

void sendInCrc(uint8_t value)
{
    CRC_set8BitData(CRC_BASE, value);
    USCI_A_UART_transmitData(UART_BRIDGE, value);
}

void sendInCrcEscaped(uint8_t value)
{
    USCI_A_UART_transmitData(UART_BRIDGE, HDLC_ESC);
    CRC_set8BitData(CRC_BASE, value);
    USCI_A_UART_transmitData(UART_BRIDGE, value^HDLC_XOR);
}

void sendEscaped(const void *buf, size_t len) {
    uint8_t *pTx = (uint8_t*)buf;
    for(int i=0; i<len; i++) {
        if(*pTx == HDLC_FRAME || *pTx == HDLC_ESC) {
            sendInCrcEscaped(*pTx);
        } else {
            sendInCrc(*pTx);
        }
        pTx++;
    }
}

void sendCrc(void)
{
    uint16_t crc_value = CRC_getResultBitsReversed(CRC_BASE);
    crc_value ^= 0xFFFF;
    uint8_t crc_buf[2] = {crc_value & 0xFF, (crc_value >> 8) & 0xFF};
    sendEscaped(crc_buf, 2);
}

//Send control packet to CC1352
void USBWPAN_sendPacket(void)
{
    CRC_setSeed(CRC_BASE, 0xffff);
    sendFrameByte();
    sendInCrc(0x01); //address EP0
    sendInCrc(0x03); //packet type
    sendEscaped(usbRequestIncomingData, usbRequestIncomingLength);
    sendCrc();
    sendFrameByte();
    usbRequestIncomingLength = 0;
    USBWPAN_handleDataConsumed();
}

void USBWPAN_sendAck(uint8_t address, uint8_t send_seq)
{
    uint8_t seq = (send_seq + 1) & 0x07;
    CRC_setSeed(CRC_BASE, 0xffff);
    sendFrameByte();
    sendInCrc(address);
    sendInCrc((seq << 5) & 0x1); //S-Frame ACK
    sendCrc();
    sendFrameByte();
}

void USBWPAN_reset(void)
{
    USBWPAN_handleDataConsumed();
}

//process a received EP0 control header in tSetupPacket
uint8_t wpanOutputRequest(void)
{
    uint8_t bWakeUp = FALSE;

    memcpy(usbRequestIncomingData, &tSetupPacket, sizeof(tDEVICE_REQUEST));
    usbRequestIncomingLength = sizeof(tDEVICE_REQUEST);

    if(tSetupPacket.wLength) {
        usbReceiveDataPacketOnEP0(usbRequestIncomingData+usbRequestIncomingLength);
        usbRequestIncomingLength += tSetupPacket.wLength;
    } else {
        bWakeUp |= USBWPAN_handleDataReceived();
        usbSendZeroLengthPacketOnIEP0();
    }
    return bWakeUp;
}

//Handle EP0 data interrupt, send to CC1352
uint8_t wpanOutputHandler(void)
{
    return USBWPAN_handleDataReceived();
}

int16_t WpanToHostFromBuffer (uint8_t intfNum)
{
    uint8_t byte_count, nTmp2;
    uint8_t * pEP1;
    uint8_t * pEP2;
    uint8_t * pCT1;
    uint8_t * pCT2;
    uint8_t bWakeUp = FALSE;                                                   //TRUE for wake up after interrupt
    uint8_t edbIndex;

    edbIndex = stUsbHandle[intfNum].edb_Index;

    if (WpanWriteCtrl[INTFNUM_OFFSET(intfNum)].nBytesToSendLeft == 0){    //do we have somtething to send?
        if (!WpanWriteCtrl[INTFNUM_OFFSET(intfNum)].bZeroPacketSent){        //zero packet was not yet sent
            WpanWriteCtrl[INTFNUM_OFFSET(intfNum)].bZeroPacketSent = TRUE;

            if (WpanWriteCtrl[INTFNUM_OFFSET(intfNum)].last_ByteSend ==
                EP_MAX_PACKET_SIZE_WPAN){
                if (WpanWriteCtrl[INTFNUM_OFFSET(intfNum)].bCurrentBufferXY ==
                    X_BUFFER){
                    if (tInputEndPointDescriptorBlock[edbIndex].bEPBCTX &
                        EPBCNT_NAK){
                        tInputEndPointDescriptorBlock[edbIndex].bEPBCTX = 0;
                        WpanWriteCtrl[INTFNUM_OFFSET(intfNum)].bCurrentBufferXY
                            = Y_BUFFER;                                     //switch buffer
                    }
                } else {
                    if (tInputEndPointDescriptorBlock[edbIndex].bEPBCTY &
                        EPBCNT_NAK){
                        tInputEndPointDescriptorBlock[edbIndex].bEPBCTY = 0;
                        WpanWriteCtrl[INTFNUM_OFFSET(intfNum)].bCurrentBufferXY
                            = X_BUFFER;                                     //switch buffer
                    }
                }
            }

            WpanWriteCtrl[INTFNUM_OFFSET(intfNum)].nBytesToSend = 0;      //nothing to send

            //call event callback function
            if (wUsbEventMask & USB_SEND_COMPLETED_EVENT){
                bWakeUp = TRUE;
            }
        } //if (!bSentZeroPacket)

        return (bWakeUp);
    }

    WpanWriteCtrl[INTFNUM_OFFSET(intfNum)].bZeroPacketSent = FALSE;          //zero packet will be not sent: we have data

    if (WpanWriteCtrl[INTFNUM_OFFSET(intfNum)].bCurrentBufferXY == X_BUFFER){
        //this is the active EP buffer
        pEP1 = (uint8_t *)((uintptr_t)stUsbHandle[intfNum].iep_X_Buffer);
        pCT1 = &tInputEndPointDescriptorBlock[edbIndex].bEPBCTX;

        //second EP buffer
        pEP2 = (uint8_t *)((uintptr_t)stUsbHandle[intfNum].iep_Y_Buffer);
        pCT2 = &tInputEndPointDescriptorBlock[edbIndex].bEPBCTY;
    } else {
        //this is the active EP buffer
        pEP1 = (uint8_t *)((uintptr_t)stUsbHandle[intfNum].iep_Y_Buffer);
        pCT1 = &tInputEndPointDescriptorBlock[edbIndex].bEPBCTY;

        //second EP buffer
        pEP2 = (uint8_t *)((uintptr_t)stUsbHandle[intfNum].iep_X_Buffer);
        pCT2 = &tInputEndPointDescriptorBlock[edbIndex].bEPBCTX;
    }

    //how many byte we can send over one endpoint buffer
    byte_count =
        (WpanWriteCtrl[INTFNUM_OFFSET(intfNum)].nBytesToSendLeft >
         EP_MAX_PACKET_SIZE_WPAN) ? EP_MAX_PACKET_SIZE_WPAN : WpanWriteCtrl[
            INTFNUM_OFFSET(intfNum)].nBytesToSendLeft;
    nTmp2 = *pCT1;

    if (nTmp2 & EPBCNT_NAK){
        USB_TX_memcpy(pEP1, WpanWriteCtrl[INTFNUM_OFFSET(
                                             intfNum)].pUsbBufferToSend,
            byte_count);                                                            //copy data into IEP3 X or Y buffer
        *pCT1 = byte_count;                                                         //Set counter for usb In-Transaction
        WpanWriteCtrl[INTFNUM_OFFSET(intfNum)].bCurrentBufferXY =
            (WpanWriteCtrl[INTFNUM_OFFSET(intfNum)].bCurrentBufferXY + 1) & 0x01;    //switch buffer
        WpanWriteCtrl[INTFNUM_OFFSET(intfNum)].nBytesToSendLeft -= byte_count;
        WpanWriteCtrl[INTFNUM_OFFSET(intfNum)].pUsbBufferToSend += byte_count;       //move buffer pointer
        WpanWriteCtrl[INTFNUM_OFFSET(intfNum)].last_ByteSend = byte_count;

        //try to send data over second buffer
        nTmp2 = *pCT2;
        if ((WpanWriteCtrl[INTFNUM_OFFSET(intfNum)].nBytesToSendLeft > 0) &&      //do we have more data to send?
            (nTmp2 & EPBCNT_NAK)){                                                  //if the second buffer is free?
            //how many byte we can send over one endpoint buffer
            byte_count =
                (WpanWriteCtrl[INTFNUM_OFFSET(intfNum)].nBytesToSendLeft >
                 EP_MAX_PACKET_SIZE_WPAN) ? EP_MAX_PACKET_SIZE_WPAN :
                WpanWriteCtrl[
                    INTFNUM_OFFSET(intfNum)].nBytesToSendLeft;

            USB_TX_memcpy(pEP2, WpanWriteCtrl[INTFNUM_OFFSET(
                                                 intfNum)].pUsbBufferToSend,
                byte_count);                                                        //copy data into IEP3 X or Y buffer
            *pCT2 = byte_count;                                                     //Set counter for usb In-Transaction
            WpanWriteCtrl[INTFNUM_OFFSET(intfNum)].bCurrentBufferXY =
                (WpanWriteCtrl[INTFNUM_OFFSET(intfNum)].bCurrentBufferXY +
                 1) & 0x01;                                                         //switch buffer
            WpanWriteCtrl[INTFNUM_OFFSET(intfNum)].nBytesToSendLeft -=
                byte_count;
            WpanWriteCtrl[INTFNUM_OFFSET(intfNum)].pUsbBufferToSend +=
                byte_count;                                                         //move buffer pointer
            WpanWriteCtrl[INTFNUM_OFFSET(intfNum)].last_ByteSend = byte_count;
        }
    }
    return (bWakeUp);
}

uint8_t USBWPAN_sendData (const uint8_t* data, uint16_t size, uint8_t intfNum)
{
    uint8_t edbIndex;
    uint16_t state;

    edbIndex = stUsbHandle[intfNum].edb_Index;

    if (size == 0){
        return (USBWPAN_GENERAL_ERROR);
    }

    state = usbDisableInEndpointInterrupt(edbIndex);

    //do not access USB memory if suspended (PLL uce BUS_ERROR
    if ((bFunctionSuspended) ||
        (bEnumerationStatus != ENUMERATION_COMPLETE)){
        //data can not be read because of USB suspended
        usbRestoreInEndpointInterrupt(state);                                            //restore interrupt status
        return (USBWPAN_BUS_NOT_AVAILABLE);
    }

    if (WpanWriteCtrl[INTFNUM_OFFSET(intfNum)].nBytesToSendLeft != 0){
        //the USB still sends previous data, we have to wait
        usbRestoreInEndpointInterrupt(state);                                           //restore interrupt status
        return (USBWPAN_INTERFACE_BUSY_ERROR);
    }

    //This function generate the USB interrupt. The data will be sent out from interrupt

    WpanWriteCtrl[INTFNUM_OFFSET(intfNum)].nBytesToSend = size;
    WpanWriteCtrl[INTFNUM_OFFSET(intfNum)].nBytesToSendLeft = size;
    WpanWriteCtrl[INTFNUM_OFFSET(intfNum)].pUsbBufferToSend = data;

    //trigger Endpoint Interrupt - to start send operation
    USBIEPIFG |= 1 << (edbIndex + 1);                                       //IEPIFGx;

    usbRestoreInEndpointInterrupt(state);

    return (USBWPAN_SEND_STARTED);
}

uint8_t USBWPAN_getInterfaceStatus (uint8_t intfNum)
{
    uint8_t ret = 0;
    uint16_t stateIn;
    uint8_t edbIndex;

    edbIndex = stUsbHandle[intfNum].edb_Index;

    stateIn = usbDisableInEndpointInterrupt(edbIndex);

    //Is send operation underway?
    if (WpanWriteCtrl[INTFNUM_OFFSET(intfNum)].nBytesToSendLeft != 0){
        ret |= USBWPAN_WAITING_FOR_SEND;
    }

    if ((bFunctionSuspended) ||
        (bEnumerationStatus != ENUMERATION_COMPLETE)){
        //if suspended or not enumerated - report no other tasks pending
        ret |= USBWPAN_BUS_NOT_AVAILABLE;
    }

    //restore interrupt status
    usbRestoreInEndpointInterrupt(stateIn);

    __no_operation();
    return (ret);
}

#endif
