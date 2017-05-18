/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2016 UAVCAN Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef CANARD_INTERNALS_H
#define CANARD_INTERNALS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef CANARD_INTERNAL
# define CANARD_INTERNAL static
#endif

#define TRANSFER_TIMEOUT_USEC 2000000

#define TRANSFER_ID_BIT_LEN 5
#define CANARD_EXT_ID_MASK       0x1FFFFFFFU
#define CANARD_CAN_FRAME_EFF    (1U << 31)  // extended frame format
#define CANARD_CAN_FRAME_RTR    (1U << 30)  // remote transmission
#define CANARD_CAN_FRAME_ERR    (1U << 29)  // error frame

#define CANARD_SOURCE_ID_FROM_ID(x) ((x)& (0X7F))
#define CANARD_SERVICE_NOT_MSG_FROM_ID(x)   (((x) >> 7)& 0X1)
#define CANARD_REQUEST_NOT_RESPONSE_FROM_ID(x) (((x) >> 15)& 0X1)
#define CANARD_DEST_ID_FROM_ID(x)   (((x) >> 8)& 0X7F)
#define CANARD_PRIORITY_FROM_ID(x)  (((x) >> 24)& 0X1F)
#define CANARD_MSG_TYPE_FROM_ID(x)  (((x) >> 8)& 0XFFFF)
#define CANARD_SRV_TYPE_FROM_ID(x)  (((x) >> 16)& 0XFF)

#define TRANSFER_ID_FROM_TAIL_BYTE(x) ((x)& 0X1F)

#define IS_START_OF_TRANSFER(x) (((x) >> 7)& 0X1)
#define IS_END_OF_TRANSFER(x) (((x) >> 6)& 0X1)
#define TOGGLE_BIT(x) (((x) >> 5)& 0X1)


CANARD_INTERNAL uint8_t canardTransferType(uint32_t id);
CANARD_INTERNAL uint16_t canardDataType(uint32_t id);

CANARD_INTERNAL int computeForwardDistance(uint8_t a, uint8_t b);
CANARD_INTERNAL void tidIncrement(uint8_t* transfer_id);


CANARD_INTERNAL uint16_t crcAddByte(uint16_t crc_val, uint8_t byte);
CANARD_INTERNAL uint16_t crcAddSignature(uint16_t crc_val, uint64_t data_type_signature);
CANARD_INTERNAL uint16_t crcAdd(uint16_t crc_val, const uint8_t* bytes, uint16_t len);



#ifdef __cplusplus
}
#endif
#endif
