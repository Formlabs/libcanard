#include <canard.h>
#include <cstdint>
#include <drivers/stm32/canard_stm32.h>

// from: https://stackoverflow.com/questions/7038797/automatically-pick-a-variable-type-big-enough-to-hold-a-specified-number

#include <limits>
#include <type_traits>

template<class T, class U =
typename std::conditional<std::is_signed<T>::value,
    std::intmax_t,
    std::uintmax_t
>::type>
constexpr bool is_in_range(U x) {
    return (x >= std::numeric_limits<T>::min())
           && (x <= std::numeric_limits<T>::max());
}

template<std::intmax_t x>
using int_fit_type =
typename std::conditional<is_in_range<std::int8_t>(x),
    std::int8_t,
    typename std::conditional<is_in_range<std::int16_t>(x),
        std::int16_t,
        typename std::conditional<is_in_range<std::int32_t>(x),
            std::int32_t,
            typename std::enable_if<is_in_range<std::int64_t>(x), std::int64_t>::type
        >::type
    >::type
>::type;

template<std::uintmax_t x>
using uint_fit_type =
typename std::conditional<is_in_range<std::uint8_t>(x),
    std::uint8_t,
    typename std::conditional<is_in_range<std::uint16_t>(x),
        std::uint16_t,
        typename std::conditional<is_in_range<std::uint32_t>(x),
            std::uint32_t,
            typename std::enable_if<is_in_range<std::uint64_t>(x), std::uint64_t>::type
        >::type
    >::type
>::type;

template<
    typename T,
    std::uintmax_t SIZE>

class RingBuffer {
public:
    RingBuffer() = default;

    typedef uint_fit_type<SIZE> msize_t;

    static inline msize_t inc(msize_t v) {
        return (v + 1) % SIZE;
    }

    // No boundary checking, call IsFull() first
    inline T &get_next_write_entry() {
        return data[writeIdx];
    }

    inline T &get_next_read_entry() {
        return data[readIdx];
    }

    inline bool is_full() const {
        return inc(writeIdx) == readIdx;
    }

    inline bool is_empty() const {
        return readIdx == writeIdx;
    }

    inline void advance_write() {
        writeIdx = inc(writeIdx);
    }

    inline void advance_read() {
        readIdx = inc(readIdx);
    }

private:
    T data[SIZE];

    volatile msize_t readIdx = 0;
    volatile msize_t writeIdx = 0;
};

#define USED __attribute__((used))

static RingBuffer<CanardCANFrame, 64> CAN_RX_RB;
static RingBuffer<CanardCANFrame, 128> CAN_TX_RB;

extern "C" {
extern CanardInstance canard;

#if defined(STM32F0) || defined(STM32F3)
void USED CAN_IT_Callback() {
    if (!CAN_RX_RB.is_full()) {
        canardSTM32Receive(&CAN_RX_RB.get_next_write_entry());
        CAN_RX_RB.advance_write();
    } else {
        canardSTM32ReleaseFIFO();
        canard_errors.rx_errors++;
    }

    canardSTM32Receive_IT();
}
#endif

volatile bool canTXTransmitting = false;

volatile canard_errors_t canard_errors = {
    .tx_errors = 0,
    .rx_errors = 0,
    .broadcast_errors = 0
};

void processRxQueue() {
    const uint64_t ts_usec = getMonotonicTimestampMSec();

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
    } else {
        canTXTransmitting = false;
    }
}

void processTxRxOnce() {
    NVIC_DisableIRQ(CAN_TX_IRQn);
    __ISB();
    if (!canTXTransmitting) {
        processTxQueue();
    }
    NVIC_EnableIRQ(CAN_TX_IRQn);
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

#if 0
/**
 * Puts frame on on the TX queue. Higher priority placed first
 */
void pushTxQueue(CanardInstance *ins, CanardTxQueueItem *item) {
    NVIC_DisableIRQ(CAN_TX_IRQn);
    CANARD_ASSERT(ins != NULL);
    CANARD_ASSERT(item->frame.data_len > 0);       // UAVCAN doesn't allow zero-payload frames

    if (ins->tx_queue == NULL) {
        ins->tx_queue = item;
        return;
    }

    CanardTxQueueItem *queue = ins->tx_queue;
    CanardTxQueueItem *previous = ins->tx_queue;

    while (queue != NULL) {
        if (isPriorityHigher(queue->frame.id, item->frame.id)) // lower number wins
        {
            if (queue == ins->tx_queue) {
                item->next = queue;
                ins->tx_queue = item;
            } else {
                previous->next = item;
                item->next = queue;
            }
            NVIC_EnableIRQ(CAN_TX_IRQn);
            return;
        } else {
            if (queue->next == NULL) {
                queue->next = item;
                NVIC_EnableIRQ(CAN_TX_IRQn);
                return;
            } else {
                previous = queue;
                queue = queue->next;
            }
        }
    }
    NVIC_EnableIRQ(CAN_TX_IRQn);
}
#endif
}

