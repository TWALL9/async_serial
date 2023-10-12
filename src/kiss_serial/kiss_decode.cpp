#include "async_serial/kiss_tnc.hpp"

namespace async_serial
{

KissInputStream::KissInputStream()
{
  init();
}

void KissInputStream::init()
{
  buf_.clear();
  state_ = KissDecodeState::WAIT_FOR_START;
}

// TODO add multiple bytes to receive
int KissInputStream::addByte(const uint8_t b)
{
  int ret_code = 0;
  switch (state_)
  {
    case KissDecodeState::WAIT_FOR_START:
      if (b == FEND)
      {
        state_ = KissDecodeState::GET_PORT;
      }
      else
      {
        ret_code = NOT_START;
      }
      break;
    case KissDecodeState::GET_PORT:
      if (b == FEND || b == FESC || b == TFEND || b == TFESC)
      {
        state_ = KissDecodeState::ERROR;
        ret_code = TOO_SHORT;
      }
      else
      {
        state_ = KissDecodeState::DECODING;
        uint8_t port = (b & 0xF0) >> 4;
        buf_.push_back(port);
      }
      break;
    case KissDecodeState::DECODING:
      if (b == FEND)
      {
        state_ = KissDecodeState::DONE;
      }
      else if (b == FESC)
      {
        state_ = KissDecodeState::ESCAPE_SEQUENCE;
      }
      else
      {
        buf_.push_back(b);
      }
      break;
    case KissDecodeState::ESCAPE_SEQUENCE:
      if (b == TFEND)
      {
        buf_.push_back(FEND);
        state_ = KissDecodeState::DECODING;
      }
      else if (b == TFESC)
      {
        buf_.push_back(FESC);
        state_ = KissDecodeState::DECODING;
      }
      else
      {
        state_ = KissDecodeState::ERROR;
        ret_code = ESCAPE_ERROR;
      }
      break;
    case KissDecodeState::ERROR:
      ret_code = STILL_ERROR;
      break;
    case KissDecodeState::DONE:
    default:
      break;
  }

  return ret_code;
}

bool KissInputStream::isReady()
{
  return state_ == KissDecodeState::DONE;
}

std::vector<uint8_t> KissInputStream::getBuffer(int& error, uint8_t& port)
{
  if (state_ != KissDecodeState::DONE)
  {
    std::vector<uint8_t> out;
    error = IN_PROGRESS;
    return out;
  }
  else
  {
    port = buf_.front();
    buf_.erase(buf_.begin());
    std::vector<uint8_t> out(buf_);
    return out;
  }
}

}  // namespace async_serial
