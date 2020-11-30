# BSD 3-Clause License
# 
# Copyright (c) 2020, Erik Larson
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
# 
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
# 
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# 
# wpan_test.py
#
#  Created on: Sept 7, 2020
#      Author: Erik Larson
import serial
import argparse
import struct
import unittest

###############################################
# Assumed setup:
# UART with USB/Serial converter for port
###############################################


# STATE_IDLE      = 0
# STATE_HEADER    = 1
# STATE_DATA      = 2
# STATE_ETX       = 3

# state = STATE_IDLE
# header = []
# data = []

# header_buf = []

# SLIP_END        = 0xC0
# SLIP_ESC        = 0xDB
# SLIP_ESC_END    = 0xDC
# SLIP_ESC_ESC    = 0xDD

# with serial.Serial(args.port, 115200) as ser:

#     while True:
#         if state == STATE_IDLE:
#             stx = ser.read(1)
#             if stx[0] == SLIP_ESC:
#                 state = STATE_HEADER
#         elif state == STATE_HEADER:
#             header = struct.unpack('<BBHHH', ser.read(8))
#             if header[-1] > 0: #length field
#                 state = STATE_DATA
#             else:
#                 state = STATE_ETX
#         elif state == STATE_DATA:
#             data = ser.read(header[-1])
#             state = STATE_ETX
#         elif state == STATE_ETX:
#             etx = ser.read(1)
#             if etx == b']':
#                 if header[1] == 1: #TX, send ACK
#                     ser.write(b'[')
#                     ser.write(bytearray([header[3]]))
#                     ser.write(b']')
#                 if header[-1] > 0:
#                     print(header,data.hex())
#                 else:
#                     print(header)
#                 state = STATE_IDLE

HDLC_FRAME      = 0x7E
HDLC_ESC        = 0x7D
HDLC_ESC_FRAME  = 0x5E
HDLC_ESC_ESC    = 0x5D

def read_packet(port):
    payload = bytearray()
    skip = False
    while True:
        c = port.read(1)
        if c[0] == HDLC_FRAME:
            skip = False
            if len(payload) == 0:
                continue
            else:
                #TODO Add crc check of last 2 bytes
                print('DAT:',payload[:-2].hex())
                print('CRC:',payload[-2:].hex())
                return payload[:-2]
        elif skip:
            continue
        elif c[0] == HDLC_ESC:
            c = port.read(1)
            if c[0] == HDLC_ESC_FRAME:
                payload += bytes([HDLC_FRAME])
            elif c[0] == HDLC_ESC_ESC:
                payload += bytes([HDLC_ESC])
            else:
                #invalid escape sequence
                skip = True
                payload = bytearray()
        else:
            payload += c

def write_packet(port, contents, address=0x03):
    port.write(bytes([HDLC_FRAME, address, 0x03]))
    port.write(bytes(contents))
    port.write(bytes([0,0])) #CRC
    port.write(bytes([HDLC_FRAME]))

def parse_packet(port):
    data = read_packet(port)

    address = data[0]
    msg_type = data[1]
    frame = data[2:]

    header, payload = struct.unpack('<BBHHH', frame[:8]), frame[8:]
    wp = ''
    if header[1] == 1:
        write_packet(port, [header[3]])
        wp = str(header[3])
    if header[-1] > 0:
        return(header, payload.hex(), wp)
    else:
        return(header, '', wp)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Test WPANUSB HDLC Uart')
    parser.add_argument('port', help='serial port')
    parser.add_argument('-t', action='store_true')

    args = parser.parse_args()
    print(args.port, args.t)

    with serial.Serial(args.port, 115200) as ser:
        if args.t:
            write_packet(ser, [0x55])
            print('55 packet written')
        while True:
            header, payload, wp = parse_packet(ser)
            print(header, payload, wp)
            print()
            write_packet(ser, b'hello ' + payload.encode('ascii') + b'\r\n', 0x05)



class MockSerial():
    def __init__(self, contents=None):
        self.in_buffer = bytearray(contents)
        self.out_buffer = bytearray()

    def setInputBuffer(self, contents):
        self.in_buffer = bytearray(contents)

    def appendInputBuffer(self, contents):
        self.in_buffer += contents

    def matchOutputBuffer(self, contents=[]):
        match = False
        if len(contents) == len(self.out_buffer):
            match = True
            for i in range(len(contents)):
                if contents[i] != self.out_buffer[i]:
                    match = False
                    break
        self.out_buffer = bytearray()
        return match

    def inputEmpty(self):
        return len(self.in_buffer) == 0

    def read(self, length):
        return bytes([self.in_buffer.pop(0)])

    def write(self, contents):
        self.out_buffer += contents

class TestWpan(unittest.TestCase):

    def test_reset(self):
        port = MockSerial([HDLC_FRAME,1,3,0x40,0,0,0,0,0,0,0,0,0,HDLC_FRAME])
        header, payload, wp = parse_packet(port)
        self.assertEqual(header, (0x40, 0, 0, 0, 0))
        self.assertEqual(payload, '')
        self.assertTrue(port.matchOutputBuffer())
        self.assertTrue(port.inputEmpty())

    def test_set_channel(self):
        port = MockSerial([HDLC_FRAME,1,3,0x40,4,0,0,0,0,2,0,0,0x14,0,0,HDLC_FRAME])
        header, payload, wp = parse_packet(port)
        self.assertEqual(header, (0x40, 4, 0, 0, 2))
        self.assertEqual(payload, '0014')
        self.assertTrue(port.matchOutputBuffer())
        self.assertTrue(port.inputEmpty())

    def test_set_channel_esc_frame(self):
        port = MockSerial([HDLC_FRAME,1,3,0x40,4,0,0,0,0,2,0,0,HDLC_ESC,HDLC_ESC_FRAME,0,0,HDLC_FRAME])
        header, payload, wp = parse_packet(port)
        self.assertEqual(header, (0x40, 4, 0, 0, 2))
        self.assertEqual(payload, '007e')
        self.assertTrue(port.matchOutputBuffer())
        self.assertTrue(port.inputEmpty())

    def test_set_channel_esc_esc(self):
        port = MockSerial([HDLC_FRAME,1,3,0x40,4,0,0,0,0,2,0,0,HDLC_ESC,HDLC_ESC_ESC,0,0,HDLC_FRAME])
        header, payload, wp = parse_packet(port)
        self.assertEqual(header, (0x40, 4, 0, 0, 2))
        self.assertEqual(payload, '007d')
        self.assertTrue(port.matchOutputBuffer())
        self.assertTrue(port.inputEmpty())

    def test_set_channel_invalid_esc(self):
        port = MockSerial([HDLC_FRAME,1,3,0x40,HDLC_ESC,0,0,0,0,2,0,0,0x14,HDLC_FRAME,HDLC_FRAME,1,3,0x40,4,0,0,0,0,2,0,0,0x15,0,0,HDLC_FRAME])
        header, payload, wp = parse_packet(port)
        self.assertEqual(header, (0x40, 4, 0, 0, 2))
        self.assertEqual(payload, '0015')
        self.assertTrue(port.matchOutputBuffer())
        self.assertTrue(port.inputEmpty())

    def test_tx(self):
        port = MockSerial([HDLC_FRAME,1,3,0x40,1,0,0,31,0,4,0,1,HDLC_ESC,HDLC_ESC_ESC,HDLC_ESC,HDLC_ESC_FRAME,4,0,0,HDLC_FRAME])
        header, payload, wp = parse_packet(port)
        self.assertEqual(header, (0x40, 1, 0, 31, 4))
        self.assertEqual(payload, '017d7e04')
        self.assertTrue(port.matchOutputBuffer([HDLC_FRAME,3,3,31,0,0,HDLC_FRAME]))
        self.assertTrue(port.inputEmpty())
