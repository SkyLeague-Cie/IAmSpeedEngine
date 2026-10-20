#pragma once
#include "CoreMinimal.h"
namespace Speed::Analytic::Private
{
#if !UE_BUILD_SHIPPING
// Values only. Allocation occurs only for a selected opt-in provider. No state/hash schema.
struct FExactCrossNormalCapture844 final
{
    struct FRow { FVector3d Cross; double Squared; int32 Axis; };
    static constexpr int32 Limit = 512;
    TArray<FRow> Rows;
    uint64 Dropped = 0;
    FExactCrossNormalCapture844() { Rows.Reserve(Limit); }
    void Record(const FVector3d& Cross, double Squared, int32 Axis)
    {
        if (Rows.Num() < Limit) Rows.Add({Cross, Squared, Axis});
        else if (Dropped != MAX_uint64) ++Dropped;
    }
    bool IsComplete() const { return Dropped == 0; }
};
#endif
}
