/*
 * Copyright (c) 2016 UAVCAN Team
 *
 * Distributed under the MIT License, available in the file LICENSE.
 *
 * Author: Michael Sierra <sierramichael.a@gmail.com>
 *
 */


#ifndef CANARD_H
#define CANARD_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/** The size of a memory block in bytes. */
#define CANARD_MEM_BLOCK_SIZE 32

#define CANARD_CAN_FRAME_MAX_DATA_LEN 8

#define CANARD_BROADCAST_NODE_ID    0
#define CANARD_MIN_NODE_ID          1
#define CANARD_MAX_NODE_ID          127

#define CANARD_RX_PAYLOAD_HEAD_SIZE (CANARD_MEM_BLOCK_SIZE - offsetof(CanardRxState, buffer_head))
#define CANARD_BUFFER_BLOCK_DATA_SIZE (CANARD_MEM_BLOCK_SIZE - offsetof(CanardBufferBlock, data))

typedef struct
{
    uint32_t id;
    uint8_t data[CANARD_CAN_FRAME_MAX_DATA_LEN];
    uint8_t data_len;
} CanardCANFrame;

typedef enum
{
    CanardTransferTypeResponse  = 0,
    CanardTransferTypeRequest   = 1,
    CanardTransferTypeBroadcast = 2
} CanardTransferType;

typedef enum
{
    CanardResponse,
    CanardRequest
} CanardRequestResponse;

typedef struct CanardInstance CanardInstance;
typedef struct CanardRxTransfer CanardRxTransfer;
typedef struct CanardRxState CanardRxState;
typedef struct CanardTxQueueItem CanardTxQueueItem;

/**
 * This function will be invoked by the library every time a transfer is successfully received.
 * If the application needs to send another transfer from this callback, it is recommended
 * to call canardReleaseRxTransferPayload() first, so that the memory that was used for the block
 * buffer can be released and re-used by the TX queue.
 */
typedef void (*CanardOnTransferReception)(CanardInstance* ins, CanardRxTransfer* transfer);

/** A memory block used in the memory block allocator. */
typedef union CanardPoolAllocatorBlock_u
{
    char bytes[CANARD_MEM_BLOCK_SIZE];
    union CanardPoolAllocatorBlock_u* next;
} CanardPoolAllocatorBlock;

typedef struct
{
    CanardPoolAllocatorBlock* free_list;
} CanardPoolAllocator;

/** buffer block for rx data. */
typedef struct CanardBufferBlock
{
    struct CanardBufferBlock* next;

    uint8_t data[];
} CanardBufferBlock;

struct CanardRxState
{
    struct CanardRxState* next;

    CanardBufferBlock* buffer_blocks;

    uint64_t timestamp_usec;

    // uint32_t dtid_tt_snid_dnid;
    uint32_t dtid_tt_snid_dnid;

    uint16_t payload_crc;
    uint16_t calculated_crc;
    uint16_t payload_len : 10;
    uint16_t transfer_id : 5;
    uint16_t next_toggle : 1;

    uint8_t buffer_head[];
};

/**
 * This structure maintains the current canard instance of this node including the node_ID,
 * a function pointer, should_accept, which the application uses to decide whether to keep this or subsequent frames,
 * a function pointer, on_reception, which hands a completed transfer to the application,
 * the allocator and its blocks,
 * and
 * the CanardRxState list
 */
struct CanardInstance
{
    uint8_t node_id; // local node
                                                                                                        // allocator

    CanardRxState* rx_states;
    CanardTxQueueItem* tx_queue;
};


/**
 * This structure represents a received transfer for the application.
 * An instance of it is passed to the application via callback when
 * the library receives a new transfer.
 */
struct CanardRxTransfer
{
    /**
     * Timestamp at which the first frame of this transfer was received.
     */
    uint64_t timestamp_usec;

    /**
     * Payload is scattered across three storages:
     *  - Head points to CanardRxState.buffer_head (length of which is up to Canard_PAYLOAD_HEAD_SIZE), or to the
     *    payload field (possibly with offset) of the last received CAN frame.
     *  - Middle is located in the linked list of dynamic blocks.
     *  - Tail points to the payload field (possibly with offset) of the last received CAN frame
     *    (only for multi-frame transfers).
     *
     * The tail offset depends on how much data of the last frame was accommodated in the last allocated
     * block.
     *
     * For single-frame transfers, middle and tail will be NULL, and the head will point at first byte
     * of the payload of the CAN frame.
     *
     * In simple cases it should be possible to get data directly from the head and/or tail pointers.
     * Otherwise it is advised to use canardReadRxTransferPayload().
     */
    const uint8_t*           payload_head;   // /< Always valid, i.e. not NULL.
    CanardBufferBlock* payload_middle; // /< May be NULL if the buffer was not needed. Always NULL for single-frame
                                       // transfers.
    const uint8_t*           payload_tail;   // /< Last bytes of multi-frame transfers. Always NULL for single-frame
                                             // transfers.
    uint16_t payload_len;
    uint16_t middle_len;

    /**
     * These fields identify the transfer for the application logic.
     */
    uint16_t data_type_id;                  // /< 0 to 255 for services, 0 to 65535 for messages
    uint8_t transfer_type;                  // /< See @ref CanardTransferType
    uint8_t transfer_id;                    // /< 0 to 31
    uint8_t priority;                       // /< 0 to 31
    uint8_t source_node_id;                 // /< 1 to 127, or 0 if the source is anonymous
};


int canardBroadcast(uint8_t node_id, uint64_t data_type_signature, uint16_t data_type_id,
                    uint8_t transfer_type, uint8_t* inout_transfer_id,
                    const void* payload,
                    uint16_t payload_len);

#ifdef __cplusplus
}
#endif
#endif
