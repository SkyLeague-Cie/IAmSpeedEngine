#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace Speed::Replay
{
// Playback coordinates only. Never passes a signed delta to a physical solver.
// A seek has a new generation even when it selects the same source frame.
class FPlaybackClock
{
    uint64_t Count = 0, Generation = 0;
    uint32_t Frequency = 0;
    double Position = 0, Rate = 1, Correction = 0;
    bool Paused = true;
    double DisplayCoordinate() const
    {
        const double Nearest = std::round(Position);
        const double Tolerance = 8 * std::numeric_limits<double>::epsilon() * std::max(1., std::abs(Position));
        // Presentation addressing only: do not quantize recorded states or
        // discard compensated time accumulation. Adjacent render cadences can
        // land a few ULPs either side of the same exact physical-frame boundary.
        return std::abs(Position - Nearest) <= Tolerance ? Nearest : Position;
    }
public:
    bool Open(uint64_t InCount, uint32_t InFrequency)
    {
        if (!InCount || InCount > 1080000 || !InFrequency || InFrequency > 1000 ||
            Generation == std::numeric_limits<uint64_t>::max()) return false;
        Count = InCount; Frequency = InFrequency; Position = 0; Rate = 1;
        Correction = 0; Paused = true; ++Generation; return true;
    }
    bool Close()
    {
        Count = 0; Frequency = 0; Position = 0; Correction = 0; Paused = true;
        if (Generation == std::numeric_limits<uint64_t>::max()) return false;
        ++Generation; return true;
    }
    bool Seek(int64_t Ordinal)
    {
        if (!Count || Generation == std::numeric_limits<uint64_t>::max()) return false;
        Position = double(std::min(uint64_t(std::max(int64_t(0), Ordinal)), Count - 1));
        Correction = 0; ++Generation; return true;
    }
    bool SetRate(double Value)
    {
        if (!std::isfinite(Value) || std::abs(Value) < .125 || std::abs(Value) > 8) return false;
        // Direction changes preserve the selected snapshot and start its dwell
        // afresh. Switching floor/ceil addressing must not select a neighbour
        // merely because playback was reversed while inside a frame interval.
        if ((Value < 0) != (Rate < 0)) Position = double(Index());
        Rate = Value; Correction = 0; return true;
    }
    bool Play() { if (!Count) return false; Paused = false; return true; }
    void Pause() { Paused = true; }
    bool Advance(double Seconds)
    {
        if (!Count || !std::isfinite(Seconds) || Seconds < 0 || Seconds > 60) return false;
        if (Paused) return true;
        const double Delta = Seconds * double(Frequency) * Rate - Correction;
        const double Next = Position + Delta;
        Correction = (Next - Position) - Delta;
        // Every snapshot occupies one source interval, including the last
        // forward/first reverse snapshot: N frames last N/F seconds. The extra
        // time boundary never creates or addresses an extra recorded frame.
        Position = std::clamp(Next, -1., double(Count));
        if (Position != Next) Correction = 0;
        const double Display = DisplayCoordinate();
        if ((Rate > 0 && Display >= double(Count)) || (Rate < 0 && Display <= -1.)) Paused = true;
        return true;
    }
    uint64_t Index() const
    {
        if(!Count)return 0;
        const double Coordinate=DisplayCoordinate();
        const double Address=Rate<0?std::ceil(Coordinate):std::floor(Coordinate);
        return uint64_t(std::clamp(Address,0.,double(Count-1)));
    }
    uint64_t Epoch() const { return Generation; }
    uint64_t NumFrames() const { return Count; }
    uint32_t Hz() const { return Frequency; }
    bool IsPaused() const { return Paused; }
    double Speed() const { return Rate; }
    double Fraction() const
    {
        if(!Count)return 0;
        const double Coordinate=DisplayCoordinate(),Frame=double(Index());
        return std::clamp(Rate<0?Frame-Coordinate:Coordinate-Frame,0.,1.);
    }
};
// Unpaused, undilated world time feeds transport, not actor render cadence.
// Pending IO/preparation retains elapsed time without changing its exact target.
// Explicit controls reset this debt; no old interval bursts after pause/seek.
class FPlaybackElapsedBudget
{
    double Last = 0, Pending = 0; bool Anchored = false;
public:
    static constexpr double MaxPendingSeconds = 60.;
    bool Reset(double Now)
    { if(!std::isfinite(Now)||Now<0)return false;Last=Now;Pending=0;Anchored=true;return true; }
    void Close(){Last=Pending=0;Anchored=false;}
    bool Accumulate(double Now,bool Running)
    {
        if(!std::isfinite(Now)||Now<0)return false;
        if(!Anchored)return Reset(Now);
        if(Now<Last)return false;
        const double Delta=Now-Last;Last=Now;
        if(!Running){Pending=0;return true;}
        if(Delta>MaxPendingSeconds-Pending)return false;
        Pending+=Delta;return true;
    }
    double Take(){const double Out=Pending;Pending=0;return Out;}
    double PendingSeconds()const{return Pending;}
};

}
