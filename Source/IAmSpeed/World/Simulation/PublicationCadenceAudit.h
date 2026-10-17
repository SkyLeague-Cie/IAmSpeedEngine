#pragma once
#include <array>
#include <cstdint>
#include <cmath>

// Local, noncanonical observability. The owner serializes Publish/Complete/Observe
// with the same lock as snapshot visibility. No schedule or physical state changes.
namespace Speed::PublicationCadence
{
struct FTiming
{
    std::uint64_t Epoch = 0;
    double StartSeconds = 0;
    bool Present = false;
};
struct FStartOutcome
{
    double ActualIntervalSeconds = 0;
    double TargetIntervalSeconds = 0;
    double BaseIntervalSeconds = 0;
    double RecoveryDebtSeconds = 0;
    bool HasPreviousStart = false;
    bool RecoveryLimited = false;
};
struct FReceipt
{
    std::uint64_t Frame = 0, Serial = 0;
    FTiming Timing;
    FStartOutcome Start;
    double VisibilitySeconds = 0;
    bool Complete = false;
};
struct FObserver
{
    std::uint64_t Serial = 0, Frame = 0, Epoch = 0;
    double ObservationSeconds = 0;
    bool Initialized = false;
};
struct FObservation
{
    bool Available = false, Rebased = false, HistoryComplete = true;
    bool EarlyStart = false, LateStart = false, FrameAccountingMismatch = false;
    std::uint64_t Publications = 0;
    double ObservationIntervalSeconds = 0, LatestPublicationAgeSeconds = 0;
    FReceipt Latest, FirstEarlyStart, FirstLateStart;
};
class FHistory final
{
public:
    static constexpr std::size_t Capacity = 256;
    // Called only for an accepted snapshot, under its publication lock.
    void Publish(std::uint64_t Frame, std::uint64_t Serial,
        double VisibilitySeconds, FTiming Timing)
    {
        FReceipt R; R.Frame=Frame; R.Serial=Serial; R.VisibilitySeconds=VisibilitySeconds;
        R.Timing=Timing; R.Complete=!Timing.Present;
        Slots[Serial%Capacity]=R;
        if (R.Complete) LatestCompleteSerial=Serial;
    }
    // Completes only diagnostic data after the unchanged physical pacer ran.
    // The producer is the sole snapshot writer; visibility/serial never change here.
    bool Complete(std::uint64_t Serial, double StartSeconds, const FStartOutcome& Outcome)
    {
        FReceipt& R=Slots[Serial%Capacity];
        if (R.Serial!=Serial || !R.Timing.Present || R.Complete ||
            R.Timing.StartSeconds!=StartSeconds) return false;
        R.Start=Outcome; R.Complete=true; LatestCompleteSerial=Serial; return true;
    }
    FObservation Observe(FObserver& Observer, double ObservationSeconds,
        double ToleranceSeconds) const
    {
        FObservation Out;
        if (!LatestCompleteSerial) return Out;
        const FReceipt& Latest=Slots[LatestCompleteSerial%Capacity];
        Out.Available=true; Out.Latest=Latest;
        Out.LatestPublicationAgeSeconds=ObservationSeconds-Latest.VisibilitySeconds;
        const auto Rebase=[&](){
            Observer={Latest.Serial,Latest.Frame,Latest.Timing.Epoch,ObservationSeconds,true};
            Out.Rebased=true;
        };
        if (!std::isfinite(ToleranceSeconds) || ToleranceSeconds<0 ||
            !std::isfinite(ObservationSeconds) ||
            !std::isfinite(Latest.VisibilitySeconds) ||
            Latest.VisibilitySeconds>ObservationSeconds ||
            (Latest.Timing.Present && (!std::isfinite(Latest.Timing.StartSeconds) ||
             Latest.Timing.StartSeconds>Latest.VisibilitySeconds)))
        { Out.HistoryComplete=false; return Out; }
        if (!Observer.Initialized || !Latest.Timing.Present ||
            Observer.Epoch!=Latest.Timing.Epoch)
        { Rebase(); return Out; }
        Out.ObservationIntervalSeconds=ObservationSeconds-Observer.ObservationSeconds;
        if (Out.ObservationIntervalSeconds<0 || Latest.Serial<Observer.Serial ||
            Latest.Serial-Observer.Serial>Capacity)
        { Out.HistoryComplete=false; Rebase(); return Out; }
        Out.Publications=Latest.Serial-Observer.Serial;
        std::uint64_t PreviousFrame=Observer.Frame;
        for (std::uint64_t S=Observer.Serial+1; S<=Latest.Serial; ++S)
        {
            const FReceipt& R=Slots[S%Capacity];
            if (R.Serial!=S || !R.Complete || !R.Timing.Present ||
                R.Timing.Epoch!=Observer.Epoch ||
                !std::isfinite(R.VisibilitySeconds) || R.VisibilitySeconds>ObservationSeconds ||
                !std::isfinite(R.Timing.StartSeconds) || R.Timing.StartSeconds>R.VisibilitySeconds)
            { Out.HistoryComplete=false; break; }
            Out.FrameAccountingMismatch |= R.Frame!=PreviousFrame+1;
            PreviousFrame=R.Frame;
            if (!R.Start.HasPreviousStart) continue;
            if (!std::isfinite(R.Start.ActualIntervalSeconds) ||
                !std::isfinite(R.Start.TargetIntervalSeconds) ||
                R.Start.TargetIntervalSeconds<0)
            { Out.HistoryComplete=false; break; }
            const bool Early=R.Start.ActualIntervalSeconds+ToleranceSeconds<R.Start.TargetIntervalSeconds;
            const bool Late=R.Start.RecoveryLimited &&
                R.Start.ActualIntervalSeconds>R.Start.TargetIntervalSeconds+ToleranceSeconds;
            if (Early && !Out.EarlyStart) Out.FirstEarlyStart=R;
            if (Late && !Out.LateStart) Out.FirstLateStart=R;
            Out.EarlyStart|=Early; Out.LateStart|=Late;
        }
        Observer={Latest.Serial,Latest.Frame,Latest.Timing.Epoch,ObservationSeconds,true};
        return Out;
    }
private:
    std::array<FReceipt,Capacity> Slots{};
    std::uint64_t LatestCompleteSerial=0;
};
}
