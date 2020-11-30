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
 * ringbuf.h
 *
 *  Created on: May 7, 2020
 *      Author: Erik Larson
 */

#ifndef RINGBUF_H_
#define RINGBUF_H_

#include <stdint.h>

typedef struct
{
    uint8_t *buffer;
    uint16_t size;
    volatile uint16_t head;
    volatile uint16_t tail;
}ringbuf_t;

//Thread-safe ring buffer
//Can hold max of size-1 values

void RINGBUF_init(ringbuf_t *ring, uint8_t *buffer, uint16_t size);
void RINGBUF_flush(ringbuf_t *ring);
uint16_t RINGBUF_empty(ringbuf_t *ring);
uint16_t RINGBUF_full(ringbuf_t *ring);
uint16_t RINGBUF_push(ringbuf_t *ring, uint8_t b);
uint8_t RINGBUF_pop_unsafe(ringbuf_t *ring);
uint16_t RINGBUF_getBytesInBuffer(ringbuf_t *ring);
uint16_t RINGBUF_receiveDataInBuffer(ringbuf_t *ring, uint8_t* buffer, uint16_t count);


#endif /* RINGBUF_H_ */
