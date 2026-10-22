#pragma once

#include <cstdint>

namespace hex_arm_hardware
{
// A mode bit or an empty local lease does not prove physical disable. Only a
// successful release/damped-stop RPC may establish this receipt. The original
// lease can remain present after the driver's terminal damped-stop operation.
class DisableReceipt
{
public:
  void acknowledge(std::uint32_t session) {session_ = session; acknowledged_ = session != 0;}
  void invalidate() {acknowledged_ = false;}
  bool acknowledged() const {return acknowledged_;}
  void observe_holder(std::uint32_t holder)
  {
    if (acknowledged_ && holder != 0 && holder != session_) {invalidate();}
  }
  bool confirms(bool fresh, bool inactive, std::uint32_t holder, bool restarted) const
  {
    return acknowledged_ && fresh && inactive && !restarted &&
           (holder == 0 || holder == session_);
  }
private:
  std::uint32_t session_{};
  bool acknowledged_{false};
};
}  // namespace hex_arm_hardware
