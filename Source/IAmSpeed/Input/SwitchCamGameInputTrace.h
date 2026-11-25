#pragma once

#include "RawInput.h"
#include <atomic>

namespace Speed::Input::V2
{
class FInputActionContract;

namespace SwitchCamGameInputTrace
{
// One writer and one drain. Overflow quarantines the diagnostic, never input.
template<class T, std::size_t Capacity>
class TSpscTrace
{
    static_assert(Capacity > 0);
public:
    bool Push(const T& Value) noexcept
    {
        const auto Tail = Write.load(std::memory_order_relaxed);
        if (Tail - Read.load(std::memory_order_acquire) >= Capacity)
        {
            RecordDrop();
            return false;
        }
        Slots[Tail % Capacity] = Value;
        Write.store(Tail + 1, std::memory_order_release);
        return true;
    }

    bool Pop(T& Value) noexcept
    {
        const auto Head = Read.load(std::memory_order_relaxed);
        if (Head == Write.load(std::memory_order_acquire)) return false;
        Value = Slots[Head % Capacity];
        Read.store(Head + 1, std::memory_order_release);
        return true;
    }

    void RecordDrop() noexcept { Drops.fetch_add(1, std::memory_order_relaxed); }
    std::uint64_t Dropped() const noexcept { return Drops.load(std::memory_order_relaxed); }

private:
    std::array<T, Capacity> Slots{};
    std::atomic<std::uint64_t> Write{0};
    // Keep writer and reader cursors on different cache lines without over-aligned allocation.
    std::array<std::byte, 64 - sizeof(std::atomic<std::uint64_t>)> CursorSeparation{};
    std::atomic<std::uint64_t> Read{0};
    std::atomic<std::uint64_t> Drops{0};
};

struct FRawRecord
{
    std::uint64_t Device = 0, Generation = 0, Sequence = 0, TimestampUs = 0;
    FRawControl Control;
    std::uint32_t SdkCode = 0;
    float Before = 0, After = 0;
    bool Fresh = false;
};

struct FMapRecord
{
    std::uint64_t Epoch = 0, Frame = 0, SourceSequence = 0, RawSequence = 0;
    std::uint64_t Device = 0, Generation = 0;
    std::uint32_t Ordinal = 0;
    bool Pressed = false;
};

struct FSession
{
    struct FHeldControl { float Before = 0, After = 0; };
    static constexpr auto MaxPadControls = std::size_t(EPadButton::Count) + std::size_t(EPadAxis::Count);

    // Immutable after admission on the game thread.
    std::uint64_t Id = 0, First = 0;
    std::uint16_t Action = 0;
    std::array<FRawControl, MaxPadControls> Controls{};
    std::size_t ControlCount = 0;

    // Acquisition-lane state. Only Rings and AcquisitionClosed cross lanes.
    std::array<FHeldControl, MaxPadControls> Held{};
    std::uint64_t LastDevice = 0, LastGeneration = 0;
    std::atomic<bool> AcquisitionClosed{false};
    TSpscTrace<FRawRecord, 2048> Raw;
    TSpscTrace<FMapRecord, 512> Map;

    // Consumer only, for progress and final receipts.
    std::uint64_t RawRecords = 0, MapRecords = 0, LastReportedDrops = 0;
    bool ReportedClosed = false;
};

void RegisterSession(std::uint64_t Session, std::uint64_t First, const FInputActionContract& Contract);
FSession* FindSession(std::uint64_t Session) noexcept;
void Stop();

} // namespace SwitchCamGameInputTrace
} // namespace Speed::Input::V2
