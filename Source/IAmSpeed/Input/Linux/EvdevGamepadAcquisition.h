#pragma once

#include "../DeviceDiscovery.h"
#include "../InputAcquisitionWorker.h"
#include "../RawAcquisitionJournal.h"

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <fcntl.h>
#include <limits>
#include <linux/input.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <unistd.h>
#include <utility>

namespace Speed::Input::V2::Evdev
{
// This identifies an endpoint in the client's current device namespace. It is
// neither a hardware serial number nor a persistent controller identity.
inline constexpr std::array<std::uint8_t, 8> GamepadIdPrefix{'I', 'A', 'S', 'E', 'V', 'D', '1', 0};

inline bool HasGamepadIdPrefix(const FDeviceId& Id)
{
    return std::equal(GamepadIdPrefix.begin(), GamepadIdPrefix.end(), Id.begin());
}

inline FDeviceId GamepadId(const struct stat& Endpoint)
{
    FDeviceId Id{};
    std::copy(GamepadIdPrefix.begin(), GamepadIdPrefix.end(), Id.begin());
    const std::array<std::uint64_t, 3> Identity{
        std::uint64_t(Endpoint.st_dev), std::uint64_t(Endpoint.st_rdev), std::uint64_t(Endpoint.st_ino)};
    std::size_t Offset = GamepadIdPrefix.size();
    for (const auto Field : Identity)
        for (unsigned Byte = 0; Byte < 8; ++Byte)
            Id[Offset++] = std::uint8_t(Field >> (Byte * 8));
    return Id;
}

class FGamepadAcquisition final : public IInputAcquisition
{
public:
    FGamepadAcquisition(std::shared_ptr<FRawAcquisitionJournal> InJournal, FDeviceId InSelection)
        : Journal(std::move(InJournal)), Selection(InSelection) {}
    ~FGamepadAcquisition() override { CloseDescriptor(); }
    FGamepadAcquisition(const FGamepadAcquisition&) = delete;
    FGamepadAcquisition& operator=(const FGamepadAcquisition&) = delete;

    EAcquisitionPumpResult Pump() override
    {
        if (Closed) return EAcquisitionPumpResult::Closed;
        if (Retired) return EAcquisitionPumpResult::Neutralized;

        auto Reservation = Journal->ReserveAcquisition();
        const auto Ticket = Journal->BeginAcquisition();
        if (Fd < 0 && !OpenSelectedEndpoint()) return EAcquisitionPumpResult::NoChange;

        if (!Generation || Journal->NeedsFreshResume())
        {
            FCanonicalRawState State;
            const auto Result = ReadFreshState(State);
            if (Result == ESnapshot::Busy) return EAcquisitionPumpResult::NoChange;
            if (Result == ESnapshot::Fault || Generation == MaxCounter)
                return Retire(Ticket);
            FRawAcquisitionBatch Batch;
            if (!Append(Batch, State, Generation + 1, true) || !Publish(Ticket, Batch))
                return Retire(Ticket);
            Pending = State;
            return EAcquisitionPumpResult::Installed;
        }

        FRawAcquisitionBatch Batch;
        if (!ReadReports(Batch) || (Batch.Count && !Publish(Ticket, Batch)))
            return Retire(Ticket);
        return Batch.Count ? EAcquisitionPumpResult::Installed : EAcquisitionPumpResult::NoChange;
    }

    bool Close() override
    {
        if (!Closed)
        {
            Closed = true;
            Journal->Invalidate();
            CloseDescriptor();
        }
        return true;
    }

private:
    enum class ESnapshot { Ready, Busy, Fault };
    static constexpr auto MaxCounter = std::numeric_limits<std::uint64_t>::max();
    static constexpr std::size_t ButtonCount = std::size_t(EPadButton::Count);
    static constexpr std::array<int, ButtonCount> Buttons{
        BTN_SOUTH, BTN_EAST, BTN_WEST, BTN_NORTH, -1, -1, -1, -1,
        BTN_TL, BTN_TR, BTN_THUMBL, BTN_THUMBR, BTN_START, BTN_SELECT};
    static constexpr std::array<unsigned, std::size_t(EPadAxis::Count)> Axes{
        ABS_X, ABS_Y, ABS_RX, ABS_RY, ABS_Z, ABS_RZ};
    using FKeys = std::array<std::uint8_t, (KEY_MAX + 8) / 8>;
    using FEvents = std::array<input_event, 64>;

    static bool SameValues(const FCanonicalRawState& A, const FCanonicalRawState& B)
    {
        return A.Count == B.Count && std::equal(A.Values.begin(), A.Values.begin() + A.Count,
            B.Values.begin(), [](const FRawValue& L, const FRawValue& R) { return L.Value == R.Value; });
    }

    static void SetHat(FCanonicalRawState& State, unsigned Code, int Value)
    {
        const bool Vertical = Code == ABS_HAT0Y;
        const auto Negative = std::size_t(Vertical ? EPadButton::DPadUp : EPadButton::DPadLeft);
        const auto Positive = std::size_t(Vertical ? EPadButton::DPadDown : EPadButton::DPadRight);
        State.Values[Negative].Value = Value < 0 ? 1.0f : 0.0f;
        State.Values[Positive].Value = Value > 0 ? 1.0f : 0.0f;
    }

    static bool KeyBit(const FKeys& Keys, unsigned Code)
    {
        return (Keys[Code / 8] & (1u << (Code % 8))) != 0;
    }

    static float NormalizeAxis(std::size_t Axis, const input_absinfo& Range, int Value)
    {
        const std::int64_t Low = Range.minimum;
        const std::int64_t High = Range.maximum;
        const std::int64_t Clamped = std::clamp<std::int64_t>(Value, Low, High);
        if (Axis >= std::size_t(EPadAxis::LeftTrigger))
            return float(double(Clamped - Low) / double(High - Low));

        const std::int64_t Center = Low + (High - Low + 1) / 2;
        const auto Distance = Clamped - Center;
        const auto Span = Distance < 0 ? Center - Low : High - Center;
        if (!Span) return 0.0f;
        const auto Normalized = float(double(Distance) / double(Span));
        // evdev Y grows down; the canonical stick convention grows up.
        return Axis == std::size_t(EPadAxis::LeftY) || Axis == std::size_t(EPadAxis::RightY)
            ? -Normalized : Normalized;
    }

    bool OpenSelectedEndpoint()
    {
        const auto Now = std::chrono::steady_clock::now();
        if (Now < NextDiscovery) return false;
        NextDiscovery = Now + std::chrono::milliseconds(250);

        for (unsigned Index = 0; Index < 256; ++Index)
        {
            char Path[32];
            std::snprintf(Path, sizeof(Path), "/dev/input/event%u", Index);
            struct stat Before{};
            if (::lstat(Path, &Before) != 0 || !S_ISCHR(Before.st_mode) || GamepadId(Before) != Selection)
                continue;

            Fd = ::open(Path, O_RDONLY | O_NONBLOCK | O_CLOEXEC | O_NOFOLLOW);
            if (Fd < 0) return false;
            struct stat Opened{};
            if (::fstat(Fd, &Opened) != 0 || !S_ISCHR(Opened.st_mode) || GamepadId(Opened) != Selection
                || !ConfigureGamepad())
            {
                CloseDescriptor();
                return false;
            }
            return true;
        }
        return false;
    }

    bool ConfigureGamepad()
    {
        FKeys Capabilities{};
        input_id NativeId{};
        if (::ioctl(Fd, EVIOCGID, &NativeId) < 0
            || ::ioctl(Fd, EVIOCGBIT(EV_KEY, Capabilities.size()), Capabilities.data()) < 0)
            return false;
        for (const auto Code : Buttons)
            if (Code >= 0 && !KeyBit(Capabilities, unsigned(Code))) return false;
        for (std::size_t Axis = 0; Axis < Axes.size(); ++Axis)
        {
            auto& Range = Ranges[Axis];
            if (::ioctl(Fd, EVIOCGABS(Axes[Axis]), &Range) < 0 || Range.maximum <= Range.minimum)
                return false;
        }
        for (const unsigned Code : {ABS_HAT0X, ABS_HAT0Y})
        {
            input_absinfo Hat{};
            if (::ioctl(Fd, EVIOCGABS(Code), &Hat) < 0 || Hat.minimum != -1 || Hat.maximum != 1)
                return false;
        }
        Family = RawPadFamilyFromVendor(NativeId.vendor);
        return true;
    }

    // -1 is a fault; an empty queue is not a new physical observation.
    int ReadEvents(FEvents& Events)
    {
        const auto Bytes = ::read(Fd, Events.data(), sizeof(Events));
        if (Bytes < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) return 0;
        if (Bytes <= 0 || Bytes % sizeof(input_event)) return -1;
        return int(Bytes / sizeof(input_event));
    }

    bool QueryLevels(FCanonicalRawState& State)
    {
        FKeys Keys{};
        if (::ioctl(Fd, EVIOCGKEY(Keys.size()), Keys.data()) < 0) return false;
        State.Count = ButtonCount + Axes.size();
        for (std::size_t Button = 0; Button < Buttons.size(); ++Button)
        {
            const auto Code = Buttons[Button];
            State.Values[Button] = {{ERawControlKind::PadButton, std::uint16_t(Button)},
                Code >= 0 && KeyBit(Keys, unsigned(Code)) ? 1.0f : 0.0f};
        }
        for (std::size_t Axis = 0; Axis < Axes.size(); ++Axis)
        {
            input_absinfo AxisInfo{};
            if (::ioctl(Fd, EVIOCGABS(Axes[Axis]), &AxisInfo) < 0
                || AxisInfo.minimum != Ranges[Axis].minimum || AxisInfo.maximum != Ranges[Axis].maximum)
                return false;
            State.Values[ButtonCount + Axis] = {{ERawControlKind::PadAxis, std::uint16_t(Axis)},
                NormalizeAxis(Axis, AxisInfo, AxisInfo.value)};
        }
        for (const unsigned Code : {ABS_HAT0X, ABS_HAT0Y})
        {
            input_absinfo Hat{};
            if (::ioctl(Fd, EVIOCGABS(Code), &Hat) < 0 || Hat.value < -1 || Hat.value > 1)
                return false;
            SetHat(State, Code, Hat.value);
        }
        return true;
    }

    ESnapshot ReadFreshState(FCanonicalRawState& State)
    {
        FEvents Events{};
        for (unsigned Read = 0; Read < 4; ++Read)
        {
            const int Count = ReadEvents(Events);
            if (Count < 0) return ESnapshot::Fault;
            if (Count) continue; // Pre-baseline reports cannot contribute edges.
            FCanonicalRawState Other;
            if (!QueryLevels(State) || !QueryLevels(Other)) return ESnapshot::Fault;
            pollfd Event{Fd, POLLIN, 0};
            if (::poll(&Event, 1, 0) < 0 || (Event.revents & (POLLERR | POLLHUP | POLLNVAL)))
                return ESnapshot::Fault;
            // These ioctls are level reads, not an atomic evdev report. Defer
            // the baseline if the two reads disagree or new reports are queued.
            return !Event.revents && SameValues(State, Other) ? ESnapshot::Ready : ESnapshot::Busy;
        }
        return ESnapshot::Busy;
    }

    bool ReadReports(FRawAcquisitionBatch& Batch)
    {
        FEvents Events{};
        for (unsigned Read = 0; Read < 4; ++Read)
        {
            const int Count = ReadEvents(Events);
            if (Count < 0) return false;
            if (!Count) break;
            for (int Index = 0; Index < Count; ++Index)
            {
                const auto& Event = Events[Index];
                if (Event.type == EV_SYN)
                {
                    if (Event.code == SYN_DROPPED) return false;
                    if (Event.code == SYN_REPORT
                        && !SameValues(Pending, Batch.Count ? Batch.Readings[Batch.Count - 1].State : Last.State)
                        && !Append(Batch, Pending, Generation, false))
                        return false;
                }
                else if (Event.type == EV_ABS)
                {
                    if (Event.code == ABS_HAT0X || Event.code == ABS_HAT0Y)
                        SetHat(Pending, Event.code, Event.value);
                    else if (const auto Axis = std::find(Axes.begin(), Axes.end(), Event.code); Axis != Axes.end())
                    {
                        const auto AxisIndex = std::size_t(Axis - Axes.begin());
                        Pending.Values[ButtonCount + AxisIndex].Value = NormalizeAxis(AxisIndex, Ranges[AxisIndex], Event.value);
                    }
                }
                else if (Event.type == EV_KEY && (Event.value == 0 || Event.value == 1))
                {
                    if (const auto Button = std::find(Buttons.begin(), Buttons.end(), Event.code); Button != Buttons.end())
                        Pending.Values[std::size_t(Button - Buttons.begin())].Value = float(Event.value);
                }
            }
        }
        return true;
    }

    bool Append(FRawAcquisitionBatch& Batch, const FCanonicalRawState& State, std::uint64_t NewGeneration,
        bool Fresh, bool HasDevice = true) const
    {
        if (Batch.Count == Batch.Readings.size() || Sequence >= MaxCounter - Batch.Count) return false;
        auto& Reading = Batch.Readings[Batch.Count++];
        Reading.DeviceId = HasDevice ? 1 : FDeviceDiscovery::Capacity + 1;
        Reading.Generation = {NewGeneration};
        Reading.Sequence = Sequence + Batch.Count;
        Reading.TimestampMicroseconds = std::uint64_t(std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());
        Reading.Kind = ERawDeviceKind::Gamepad;
        Reading.FreshBaseline = Fresh;
        Reading.HasSelectedDevice = HasDevice;
        Reading.PadFamily = HasDevice ? Family : ERawPadFamily::Unknown;
        Reading.State = State;
        return true;
    }

    bool Publish(FAcquisitionTicket Ticket, const FRawAcquisitionBatch& Batch)
    {
        if (!Journal->Publish(Ticket, Batch)) return false;
        Last = Batch.Readings[Batch.Count - 1];
        Generation = Last.Generation.Value;
        Sequence = Last.Sequence;
        return true;
    }

    EAcquisitionPumpResult Retire(FAcquisitionTicket Ticket)
    {
        Retired = true;
        Journal->Invalidate();
        CloseDescriptor();
        // Reject a fault before any authentic baseline. After capture, retire
        // once and leave a neutral no-device state; never reopen this binding.
        if (!Generation || Generation == MaxCounter) return EAcquisitionPumpResult::Rejected;

        auto State = Last.State;
        for (std::size_t Index = 0; Index < State.Count; ++Index) State.Values[Index].Value = 0.0f;
        FRawAcquisitionBatch Batch;
        return Append(Batch, State, Generation + 1, true, false) && Publish(Ticket, Batch)
            ? EAcquisitionPumpResult::Neutralized : EAcquisitionPumpResult::Rejected;
    }

    void CloseDescriptor()
    {
        if (Fd >= 0) ::close(Fd);
        Fd = -1;
    }

    const std::shared_ptr<FRawAcquisitionJournal> Journal;
    const FDeviceId Selection;
    int Fd = -1;
    bool Retired = false, Closed = false;
    std::uint64_t Sequence = 0, Generation = 0;
    std::chrono::steady_clock::time_point NextDiscovery{};
    std::array<input_absinfo, Axes.size()> Ranges{};
    ERawPadFamily Family = ERawPadFamily::Unknown;
    FAcquiredRawState Last{};
    FCanonicalRawState Pending{};
};
}
