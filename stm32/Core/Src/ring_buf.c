/*
 * ring_buf.c
 *
 *  Created on: Nov. 9, 2022
 *      Author: hmcga
 */

#include <stdio.h>

typedef struct {
    uint16_t size;
    uint16_t *data;
    uint16_t write_ptr;
    uint16_t read_ptr;
} RingBufStorage;

void ring_buf_init(RingBufStorage *storage, uint16_t size, uint16_t *data_arr) {
    storage->data = data_arr;
    storage->size = size;
    storage->write_ptr = 0;
    storage->read_ptr = 0;
}

void ring_buf_write()
#define TEST_ARR(size) ()
int main()
{
    test_func((1 + 0.3f) * 100);

    return 0;
}

