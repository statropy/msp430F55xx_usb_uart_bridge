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
 * ======== UsbWpan.h ========
 */

#ifndef _UsbWpan_H_
#define _UsbWpan_H_

#ifdef __cplusplus
extern "C"
{
#endif

#ifdef _VENDOR_

#ifdef _WPAN_

#define usbvendorOutputRequest wpanOutputRequest

#define USBWPAN_RESET                   0x00
#define USBWPAN_TX                      0x01
#define USBWPAN_XMIT_ASYNC              0x02
#define USBWPAN_ED                      0x03
#define USBWPAN_SET_CHANNEL             0x04
#define USBWPAN_START                   0x05
#define USBWPAN_STOP                    0x06
#define USBWPAN_SET_SHORT_ADDR          0x07
#define USBWPAN_SET_PAN_ID              0x08
#define USBWPAN_SET_IEEE_ADDR           0x09
#define USBWPAN_SET_TXPOWER             0x0A
#define USBWPAN_SET_CCA_MODE            0x0B
#define USBWPAN_SET_CCA_ED_LEVEL        0x0C
#define USBWPAN_SET_CSMA_PARAMS         0x0D
#define USBWPAN_SET_LBT                 0x0E
#define USBWPAN_SET_FRAME_RETRIES       0x0F
#define USBWPAN_SET_PROMISCUOUS_MODE    0x10
#define USBWPAN_SET_EXTENDED_ADDR       0x11
#define USBWAPN_GET_CAPABILITIES        0x12

/*
 * WPANUSB output requests to forward to device
 */
uint8_t wpanOutputRequest(void);

#endif //_VENDOR_

#endif //_WPAN_

#ifdef __cplusplus
}
#endif
#endif  //_UsbWpan_H_
