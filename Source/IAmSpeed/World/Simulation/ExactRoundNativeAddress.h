#pragma once

#include "PreparedWorldCore.h"

namespace Speed
{
struct FExactRoundNativeOrigin
{
    PreparedWorld::U64 Authority = 0, WorldIncarnation = 0, TopologyVersion = 0;
    bool Valid() const noexcept { return Authority && WorldIncarnation && TopologyVersion; }
    bool operator==(const FExactRoundNativeOrigin& B) const noexcept
    {
        return Authority == B.Authority && WorldIncarnation == B.WorldIncarnation
            && TopologyVersion == B.TopologyVersion;
    }
};

// Local original-worker address. StateVersion is the serial of the actual
// completed F-1 snapshot. PublicationSerial denotes the expected neutral F
// publication and is never a claim that F has already executed.
struct FExactRoundNativeAddress
{
    FExactRoundNativeOrigin Origin;
    PreparedWorld::U64 StateVersion = 0, PublicationSerial = 0;
    PreparedWorld::U64 Frame = 0, Next = 0, InputHash = 0;

    static std::optional<FExactRoundNativeAddress> FromCompleted(FExactRoundNativeOrigin Origin,
        PreparedWorld::U64 Frame, PreparedWorld::U64 InputHash, PreparedWorld::U64 Serial) noexcept
    {
        if (!Origin.Valid() || !Serial || Serial == UINT64_MAX || Frame >= UINT32_MAX || !InputHash) return {};
        return FExactRoundNativeAddress{Origin, Serial, Serial + 1, Frame, Frame + 1, InputHash};
    }

    bool operator==(const FExactRoundNativeAddress& B) const noexcept
    {
        return Origin == B.Origin && StateVersion == B.StateVersion && PublicationSerial == B.PublicationSerial
            && Frame == B.Frame && Next == B.Next && InputHash == B.InputHash;
    }
    bool Matches(const PreparedWorld::FAddress& A) const noexcept
    {
        return Origin == FExactRoundNativeOrigin{A.Authority, A.WorldIncarnation, A.TopologyVersion}
            && StateVersion == A.StateVersion && PublicationSerial == A.PublicationSerial
            && Frame == A.Frame && Next == A.Next && InputHash == A.InputHash;
    }
    void Stamp(PreparedWorld::FAddress& A) const noexcept
    {
        A.Authority = Origin.Authority;
        A.WorldIncarnation = Origin.WorldIncarnation;
        A.TopologyVersion = Origin.TopologyVersion;
        A.StateVersion = StateVersion;
        A.PublicationSerial = PublicationSerial;
        A.Frame = Frame;
        A.Next = Next;
        A.InputHash = InputHash;
    }
};
}
