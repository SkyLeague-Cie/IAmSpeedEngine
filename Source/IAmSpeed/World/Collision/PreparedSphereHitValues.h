#pragma once
#include "PreparedSphereResponseBinding.h"
#include "IAmSpeed/Base/SHitResult.h"
namespace Speed::PreparedExecution
{
// Pure named payload from immutable mathematical quality. This is neither a
// native request nor permission to place a hit into a physical event queue.
struct FSelectedSphereHitValues
{
    FPreparedGeometrySelectionPtr Geometry;FSelectedSphereResponseQualityPtr Quality;
    SHitResult Hit;
};
IAMSPEED_API bool BuildSelectedSphereHitValues(FPreparedGeometrySelectionPtr geometry,FSelectedSphereResponseQualityPtr quality,
    uint32 frameTag,Analytic::Certified::FDistanceBudget& budget,FSelectedSphereHitValues& output,FString& reason);
// Numerical resolution goals derived from the SUPPLIED margin. Fractions only
// allocate precision work; final quality still compares its true upper bound
// with exactly that margin. No gameplay tolerance or acceptance is enlarged.
IAMSPEED_API bool BuildMarginDrivenSphereReferenceOptions(const FSphereResponseQueryValues& query,int32 remainingWork,
    Analytic::Certified::FReferenceSphereOptions& output);
IAMSPEED_API bool SamePreparedHitValues(const SHitResult& actual,const SHitResult& expected) noexcept;
}
