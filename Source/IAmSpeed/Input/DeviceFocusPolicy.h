#pragma once
#include <atomic>
#include <cstdint>
#include <optional>
namespace Speed::Input
{
// The requested policy of IAS acquisitions is immutable for the process lifetime, including across warm
// contexts and native API instances. Closing a session never silently resets it.
// The platform host claims it only on first Pump, never during inert creation.
class FDeviceProcessFocusPolicyLatch final
{
public:
 bool TryBind(bool Background) noexcept
 {
  const std::uint8_t Wanted=Background?2:1;std::uint8_t Empty=0;
  return Mode.compare_exchange_strong(Empty,Wanted,std::memory_order_acq_rel)||Empty==Wanted;
 }
 std::optional<bool> BoundBackground() const noexcept
 {const auto Value=Mode.load(std::memory_order_acquire);return Value?std::optional<bool>(Value==2):std::nullopt;}
private:
 std::atomic<std::uint8_t> Mode{0};
};
}
