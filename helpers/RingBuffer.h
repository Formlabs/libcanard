#pragma once

#include <cstdint>
#include <libcanard/helpers/int_fit_type.h>

template<
    typename T,
    std::uintmax_t SIZE>

class RingBuffer {
public:
    RingBuffer() = default;

    typedef uint_fit_type<SIZE> msize_t;
    static const msize_t _SIZE = static_cast<msize_t>(SIZE);

    static inline msize_t inc(msize_t v) {
        return (v + 1) % _SIZE;
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

    inline msize_t size() const {
        if (writeIdx >= readIdx) {
            return writeIdx - readIdx;
        } else {
            return writeIdx + (_SIZE - readIdx);
        }
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

