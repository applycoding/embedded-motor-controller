#pragma once

namespace motor {
namespace hal {

// Non-blocking byte pipe. read() and write() return the number of bytes
// actually moved and never wait. A real port is a USART or Pico uart with
// a fixed hardware FIFO.
class Uart {
 public:
    virtual ~Uart() = default;
    virtual int read(char* dst, int max_len) = 0;
    virtual int write(const char* src, int len) = 0;
};

}  // namespace hal
}  // namespace motor
