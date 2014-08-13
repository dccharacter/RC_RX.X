#include <xc.h>
#include <stdint.h>
#include "utils.h"
#include "main.h"

#define CB_MASK (CB_BUF_SIZE - 1)

uint8_t CB_IsEmpty(CB_STRUCT *cb_str) {
    return (cb_str->idx_in == cb_str->idx_out);
}

void CB_PutInBuf(CB_STRUCT *cb_str, uint8_t val) {
    cb_str->buf[cb_str->idx_in++] = val;
    cb_str->idx_in &= CB_MASK;
}

uint8_t CB_ReadFromBuf(CB_STRUCT *cb_str) {
    uint8_t val = cb_str->buf[cb_str->idx_out++];
    cb_str->idx_out &= CB_MASK;
    return val;
}

uint8_t CB_BytesInBuf(CB_STRUCT *cb_str) {
    if (cb_str->idx_in >= cb_str->idx_out) {
        return (cb_str->idx_in - cb_str->idx_out);
    } else {
        return ((CB_BUF_SIZE -  cb_str->idx_out) + cb_str->idx_in);
    }
}