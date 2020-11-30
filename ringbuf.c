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
 * ringbuf.c
 *
 *  Created on: May 7, 2020
 *      Author: Erik Larson
 */
#include "ringbuf.h"

#define NEXT(i, size) ((i == size-1)?(0):(i+1))

void RINGBUF_init(ringbuf_t *ring, uint8_t *buffer, uint16_t size)
{
    ring->buffer = buffer;
    ring->size = size;
    ring->head = 0;
    ring->tail = 0;
}

void RINGBUF_flush(ringbuf_t *ring)
{
    ring->head = 0;
    ring->tail = 0;
}

uint16_t RINGBUF_push(ringbuf_t *ring, uint8_t b)
{
    uint16_t head = NEXT(ring->head, ring->size);

    if(head != ring->tail)
    {
        ring->buffer[ring->head] = b;
        ring->head = head;
        return 1;
    }
    return 0;
}

uint8_t RINGBUF_pop_unsafe(ringbuf_t *ring)
{
    uint8_t c = ring->buffer[ring->tail];
    ring->tail = NEXT(ring->tail, ring->size);
    return c;
}

uint16_t RINGBUF_empty(ringbuf_t *ring)
{
    return ring->head == ring->tail;
}

uint16_t RINGBUF_full(ringbuf_t *ring)
{
    return NEXT(ring->head, ring->size) == ring->tail;
}

uint16_t RINGBUF_getBytesInBuffer(ringbuf_t *ring)
{
    if(ring->head < ring->tail) {
        return ring->head - ring->tail + ring->size;
    } else {
        return ring->head - ring->tail;
    }
}

uint16_t RINGBUF_receiveDataInBuffer(ringbuf_t *ring, uint8_t* buffer, uint16_t count)
{
    uint16_t numBytes = RINGBUF_getBytesInBuffer(ring);
    uint8_t *wrBuffer = buffer;
    if(numBytes > count) {
        numBytes = count;
    }
    for(int i=numBytes;i;i--) {
        *wrBuffer++ = ring->buffer[ring->tail];
        ring->tail = NEXT(ring->tail, ring->size);
    }
    return numBytes;
}

