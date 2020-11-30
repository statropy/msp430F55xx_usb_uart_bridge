#include <gtest/gtest.h>
extern "C" {
#include "ringbuf.h"
}

TEST(RingTest, Init) {
    ringbuf_t ring;
    uint8_t buffer[32];
    RINGBUF_init(&ring, buffer, 32);
    ASSERT_EQ(1, RINGBUF_empty(&ring));
    ASSERT_EQ(0, RINGBUF_full(&ring));
    ASSERT_EQ(0, RINGBUF_getBytesInBuffer(&ring));
}

TEST(RingTest, Push) {
    ringbuf_t ring;
    uint8_t buffer[32];
    RINGBUF_init(&ring, buffer, 32);
    ASSERT_EQ(1, RINGBUF_push(&ring, 1));
    ASSERT_EQ(0, RINGBUF_empty(&ring));
    ASSERT_EQ(0, RINGBUF_full(&ring));
    ASSERT_EQ(1, RINGBUF_getBytesInBuffer(&ring));
}

TEST(RingTest, FillAndEmpty) {
    ringbuf_t ring;
    uint8_t buffer[65535];
    RINGBUF_init(&ring, buffer, 65535);
    for(int i=0; i<65534; i++) {
        ASSERT_EQ(1, RINGBUF_push(&ring, i & 0xFF));
    }
    ASSERT_EQ(0, RINGBUF_empty(&ring));
    ASSERT_EQ(1, RINGBUF_full(&ring));
    ASSERT_EQ(65534, RINGBUF_getBytesInBuffer(&ring));

    ASSERT_EQ(0, RINGBUF_push(&ring, 0xFF));
    ASSERT_EQ(1, RINGBUF_full(&ring));
    ASSERT_EQ(65534, RINGBUF_getBytesInBuffer(&ring));

    for(int i=0; i<65534; i++) {
        ASSERT_EQ(i & 0xFF, RINGBUF_pop_unsafe(&ring));
    }
    ASSERT_EQ(1, RINGBUF_empty(&ring));
    ASSERT_EQ(0, RINGBUF_getBytesInBuffer(&ring));
}

TEST(RingTest, WrapAround) {
    ringbuf_t ring;
    uint8_t buffer[8];
    RINGBUF_init(&ring, buffer, 8);
    for(int loops=0; loops<10; loops++) {
        for(int i=0; i<5; i++) {
            ASSERT_EQ(1, RINGBUF_push(&ring, i));
        }
        ASSERT_EQ(0, RINGBUF_empty(&ring));
        ASSERT_EQ(5, RINGBUF_getBytesInBuffer(&ring));

        for(int i=0; i<5; i++) {
            ASSERT_EQ(i, RINGBUF_pop_unsafe(&ring));
        }
        ASSERT_EQ(1, RINGBUF_empty(&ring));
        ASSERT_EQ(0, RINGBUF_getBytesInBuffer(&ring));
    }
}
