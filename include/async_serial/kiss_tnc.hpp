#pragma once

#include <vector>
#include <stdint.h>

namespace async_serial {

static constexpr uint8_t FEND = 0xC0;
static constexpr uint8_t FESC = 0xDB;
static constexpr uint8_t TFEND = 0xDC;
static constexpr uint8_t TFESC = 0xDD;

static constexpr int NOT_START = -1;
static constexpr int BAD_PORT = -2;
static constexpr int TOO_SHORT = -3;
static constexpr int ESCAPE_ERROR = -4;
static constexpr int STILL_ERROR = -5;
static constexpr int IN_PROGRESS = -6;

class KissOutputStream {
public:
  KissOutputStream();
  ~KissOutputStream() = default;

  void init();
  void add_byte(const uint8_t b);
  std::vector<uint8_t> get_buffer();

private:
  std::vector<uint8_t> buf_;
};

class KissInputStream {
public:
  KissInputStream();
  ~KissInputStream() = default;
  void init();
  int add_byte(const uint8_t b);
  bool is_ready();
  std::vector<uint8_t> get_buffer(int& error, uint8_t& port);

private:
  enum class KissDecodeState
  {
    WAIT_FOR_START = 0,
    GET_PORT,
    ESCAPE_SEQUENCE,
    DECODING,
    ERROR,
    DONE,
  };
  KissDecodeState state_ {KissDecodeState::WAIT_FOR_START};
  std::vector<uint8_t> buf_;
};

}
