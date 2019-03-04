#include <libcanard/canard.h>
#include <cstdint>
#include <libcanard/drivers/stm32/canard_stm32.h>
#include <libcanard/helpers/RingBuffer.h>
#include <libcanard/helpers/helpers.h>

static RingBuffer<CanardCANFrame, CAN_RX_RB_SIZE> CAN_RX_RB;
static RingBuffer<CanardCANFrame, CAN_TX_RB_SIZE> CAN_TX_RB;

extern "C" {
int canardGetRXSize() {
    return CAN_RX_RB.size();
}

int canardGetTXSize() {
    return CAN_TX_RB.size();
}

int canardGetRXFree() {
    return CAN_RX_RB.free();
}

int canardGetTXFree() {
    return CAN_TX_RB.free();
}

}

extern "C" {
extern CanardInstance canard;

volatile int allTimeMaxRxSize;
volatile int allTimeMaxTxSize;
volatile int maxRxSize;
volatile int maxTxSize;

#if defined(STM32F0) || defined(STM32F3) || defined(STM32F7)
void USED CAN_IT_Callback() {
    if (!CAN_RX_RB.is_full()) {
        canardSTM32Receive(&CAN_RX_RB.get_next_write_entry());
        CAN_RX_RB.advance_write();
    } else {
        canardSTM32ReleaseFIFO();
        canard_errors.rx_errors++;
    }

    volatile int rxSize = canardGetRXSize();
    if (rxSize > maxRxSize) {
        maxRxSize = rxSize;
    }
    if (rxSize > allTimeMaxRxSize) {
        allTimeMaxRxSize = rxSize;
    }
}
#endif

volatile bool canTXTransmitting = false;

volatile canard_errors_t canard_errors = {
    .tx_errors = 0,
    .rx_errors = 0,
    .broadcast_errors = 0
};

void processRxQueue() {
    const uint64_t ts_usec = getMonotonicTimestamp_ms() * 1000UL;

    while (!CAN_RX_RB.is_empty()) {
        canardHandleRxFrame(&canard, &CAN_RX_RB.get_next_read_entry(), ts_usec);
        CAN_RX_RB.advance_read();
    }
}

void processTxQueue(void) {
    /* Call transmit function */
    if (!CAN_TX_RB.is_empty()) {
        CanardCANFrame &frame = CAN_TX_RB.get_next_read_entry();

        canTXTransmitting = true;
        const int tx_res = canardSTM32Transmit(&frame);
        if (tx_res < 0) {         // Failure - drop the frame and report
            canard_errors.tx_errors++;
            canTXTransmitting = false;
        } else if (tx_res == 0) {
            canard_errors.tx_errors++;
            canTXTransmitting = false;
        } else {
            CAN_TX_RB.advance_read();
        }

        volatile int txSize = canardGetTXSize();
        if (txSize > maxTxSize) {
            maxTxSize = txSize;
        }
        if (txSize > allTimeMaxTxSize) {
            allTimeMaxTxSize = txSize;
        }

    } else {
        canTXTransmitting = false;
    }
}

void processTxRxOnce() {
    NVIC_DisableIRQ(CANARD_TX_IRQn);
    __ISB();
    if (!canTXTransmitting) {
        processTxQueue();
    }
    NVIC_EnableIRQ(CANARD_TX_IRQn);
    __ISB();
    processRxQueue();
}


int enqueueTxFrames(CanardInstance *ins,
                    uint32_t can_id,
                    uint8_t *transfer_id,
                    uint16_t crc,
                    const uint8_t *payload,
                    uint16_t payload_len) {
    CANARD_ASSERT(ins != NULL);
    CANARD_ASSERT((can_id & CANARD_CAN_EXT_ID_MASK) == can_id);            // Flags must be cleared

    if (transfer_id == NULL) {
        return -CANARD_ERROR_INVALID_ARGUMENT;
    }

    if ((payload_len > 0) && (payload == NULL)) {
        return -CANARD_ERROR_INVALID_ARGUMENT;
    }

    int result = 0;

    if (payload_len < CANARD_CAN_FRAME_MAX_DATA_LEN)                        // Single frame transfer
    {
        if (!CAN_TX_RB.is_full()) {
            CanardCANFrame &frame = CAN_TX_RB.get_next_write_entry();

            memcpy(frame.data, payload, payload_len);

            frame.data_len = (uint8_t)(payload_len + 1);
            frame.data[payload_len] = (uint8_t)(0xC0 | (*transfer_id & 31));
            frame.id = can_id | CANARD_CAN_FRAME_EFF;
            CAN_TX_RB.advance_write();
            result++;
        } else {
            canard_errors.tx_errors++;
        }
    } else                                                                    // Multi frame transfer
    {
        uint16_t data_index = 0;
        uint8_t toggle = 0;
        uint8_t sot_eot = 0x80;

        while (payload_len - data_index != 0) {
            if (!CAN_TX_RB.is_full()) {
                CanardCANFrame &frame = CAN_TX_RB.get_next_write_entry();
                uint8_t i = 0;
                if (data_index == 0) {
                    // add crc
                    frame.data[0] = (uint8_t)(crc);
                    frame.data[1] = (uint8_t)(crc >> 8);
                    i = 2;
                } else {
                    i = 0;
                }

                for (; i < (CANARD_CAN_FRAME_MAX_DATA_LEN - 1) && data_index < payload_len; i++, data_index++) {
                    frame.data[i] = payload[data_index];
                }
                // tail byte
                sot_eot = (data_index == payload_len) ? (uint8_t)0x40 : sot_eot;

                frame.data[i] = (uint8_t)(sot_eot | (toggle << 5) | (*transfer_id & 31));
                frame.id = can_id | CANARD_CAN_FRAME_EFF;
                frame.data_len = (uint8_t)(i + 1);

                CAN_TX_RB.advance_write();

                result++;
                toggle ^= 1;
                sot_eot = 0;
            } else {
                // XXX partial transfer, look for lower priority one to replace
                canard_errors.tx_errors++;
                break;
            }
        }
    }

    return result;
}

} // extern "C"
