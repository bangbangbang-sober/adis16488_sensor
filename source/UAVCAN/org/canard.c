/*
 * Copyright (c) 2016 UAVCAN Team
 *
 * Distributed under the MIT License, available in the file LICENSE.
 *
 * Author: Michael Sierra <sierramichael.a@gmail.com>
 *
 */

#include "canard.h"
#include "canard_internals.h"
#include "can.h"
#include <inttypes.h>
#include <string.h>
#include <stdint.h>

#define CANARD_MAKE_TRANSFER_DESCRIPTOR(data_type_id, transfer_type, \
                                        src_node_id, dst_node_id) \
    ((data_type_id) | ((transfer_type) << 16) | \
     ((src_node_id) << 18) | ((dst_node_id) << 25))

struct CanardTxQueueItem
{
    CanardTxQueueItem* next;
    CanardCANFrame frame;
};

/**
 *  API functions
 */
//extern CAN_HandleTypeDef hcan2;


/**
 * Sends a broadcast transfer.
 * If the node is in passive mode, only single frame transfers will be allowed.
 * 数据长度不超过8个字节
 */
int canardBroadcast(uint8_t node_id,
                    uint64_t data_type_signature,
                    uint16_t data_type_id,
										uint8_t transfer_type,
                    uint8_t* inout_transfer_id,
                    const void* payload,
                    uint16_t payload_len)
{
    uint32_t can_id;
    uint16_t crc = 0xFFFFU;
		uint8_t frame_index, last_frame;
	
    if (payload == NULL)
    {
        return -1;
    }
		frame_index = 0x00;
		last_frame = 0x80;
		
     can_id = ((uint32_t)data_type_id << 19) | ((uint32_t) transfer_type << 17) | ((uint32_t)node_id << 10) | *inout_transfer_id | 0x80;
		
		hcan2.pTxMsg->ExtId = can_id;
		hcan2.pTxMsg->IDE = CAN_ID_EXT;
		hcan2.pTxMsg->RTR = CAN_RTR_DATA;
		hcan2.pTxMsg->DLC = payload_len;
    memcpy(hcan2.pTxMsg->Data, payload, payload_len);		
		
		HAL_CAN_Transmit(&hcan2, 100000);
		
    tidIncrement(inout_transfer_id);
    return 1;
}


/**
 *  internal (static functions)
 *
 *
 */

/**
 * TransferID
 */
CANARD_INTERNAL int computeForwardDistance(uint8_t a, uint8_t b)
{
    int d = b - a;
    if (d < 0)
    {
        d += 1 << TRANSFER_ID_BIT_LEN;
    }
    return d;
}


CANARD_INTERNAL void tidIncrement(uint8_t* transfer_id)
{
    *transfer_id += 1;
    if (*transfer_id >= 32)
    {
        *transfer_id = 0;
    }
}



/**
 * returns true if priority of rhs is higher than id
 */
CANARD_INTERNAL bool priorityHigherThan(uint32_t rhs, uint32_t id)
{
    
		bool rtr,rhs_rtr,ext,rhs_ext;
		const uint32_t clean_id     = id    & CANARD_EXT_ID_MASK;
    const uint32_t rhs_clean_id = rhs   & CANARD_EXT_ID_MASK;
    /*
     * STD vs EXT - if 11 most significant bits are the same, EXT loses.
     */
    ext     = id     & CANARD_CAN_FRAME_EFF;
    rhs_ext = rhs & CANARD_CAN_FRAME_EFF;
    if (ext != rhs_ext)
    {
        uint32_t arb11     = ext     ? (clean_id >> 18)     : clean_id;
        uint32_t rhs_arb11 = rhs_ext ? (rhs_clean_id >> 18) : rhs_clean_id;
        if (arb11 != rhs_arb11)
        {
            return arb11 < rhs_arb11;
        }
        else
        {
            return rhs_ext;
        }
    }

    /*
     * RTR vs Data frame - if frame identifiers and frame types are the same, RTR loses.
     */
    rtr     = id     & CANARD_CAN_FRAME_RTR;
    rhs_rtr = rhs & CANARD_CAN_FRAME_RTR;
    if (clean_id == rhs_clean_id && rtr != rhs_rtr)
    {
        return rhs_rtr;
    }

    /*
     * Plain ID arbitration - greater value loses.
     */
    return clean_id < rhs_clean_id;
}




/**
 * CRC functions
 */
CANARD_INTERNAL uint16_t crcAddByte(uint16_t crc_val, uint8_t byte)
{
	  int j;
	
    crc_val ^= (uint16_t)((uint16_t)(byte) << 8);

    for (j = 0; j<8; j++)
    {
        if (crc_val & 0x8000U)
        {
            crc_val = (uint16_t)((uint16_t)(crc_val << 1) ^ 0x1021U);
        }
        else
        {
            crc_val = (uint16_t)(crc_val << 1);
        }
    }
    return crc_val;
}

CANARD_INTERNAL uint16_t crcAddSignature(uint16_t crc_val, uint64_t data_type_signature)
{
    int shift_val;
    for (shift_val = 0; shift_val<64; shift_val += 8)
    {
        crc_val = crcAddByte(crc_val, (uint8_t)(data_type_signature >> shift_val));
    }
    return crc_val;
}

CANARD_INTERNAL uint16_t crcAdd(uint16_t crc_val, const uint8_t* bytes, uint16_t len)
{
    while (len--)
    {
        crc_val = crcAddByte(crc_val, *bytes++);
    }
    return crc_val;
}


