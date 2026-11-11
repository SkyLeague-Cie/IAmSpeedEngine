#pragma once

namespace Speed
{
    // Sweep kernels use a closed horizon. Accept its first hit, then retain
    // caller order for equal TOIs; an initial horizon alone is not a hit.
    inline constexpr bool ShouldSelectSweepTOI(float CandidateTOI, float CurrentTOI, bool HasSelection)
    {
        return CandidateTOI < CurrentTOI || (!HasSelection && CandidateTOI == CurrentTOI);
    }
}
