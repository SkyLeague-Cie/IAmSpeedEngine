#pragma once
#include "NativeRetainedBoxReferenceSearch.h"
#include "NativeRetainedBoxFloatIntersectionBracketValues.h"
namespace Speed::PreparedExecution
{
// Geometry+CAPTURED VALUES only, copied before output reset. No external
// provider/corner/sign/parameter/time endpoint hints. This does not capture or
// authenticate a live native request/Source/GT/Step/filter or choose a consumer
// event. Success means only a valid geometric bracket per fixed CP realization.
// Failure/exhaustion/unsupported/tangent or indeterminate samples are Unknown,
// never a full-Box miss. No native CCD/TOI/contact/normal/response/Ready grant.
inline bool TryFindNativeRetainedBoxFloatBracket(FPreparedGeometrySelectionPtr geometry,
    FNativeSolidStaticValues captured,int32 maximumWorkPerProofAttempt,Analytic::Certified::FDistanceBudget& budget,
    FNativeRetainedBoxFloatIntersectionBracketValuesPtr& out)
{
    const auto keepPrior=out;out.Reset();namespace C=Analytic::Certified;
    const auto numeric=Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(maximumWorkPerProofAttempt<=0||!geometry||!geometry->HasValidAliases()||
        geometry->GetModel()!=EPreparedGeometryModel::RetainedReference||!budget.Consume())return false;
    FNativeFixedBoxReferenceMotionValuesPtr motion;
    if(!FNativeFixedBoxReferenceMotionValues::TryBuild(captured,budget,motion)||!motion)return false;
    FNativeRetainedBoxReferenceFirstEntryValuesPtr first;
    if(!FindNativeRetainedBoxReferenceFirstEntry(geometry,motion,maximumWorkPerProofAttempt,budget,first)||!first)return false;
    const auto attempt=[&](auto&& call)
    {
        const int32 available=budget.Remaining-budget.ConsumptionFloor;if(available<=0)return false;
        const int32 allowed=FMath::Min(available,maximumWorkPerProofAttempt);
        C::FScopedDistanceBudgetFloor reservation(budget,budget.Remaining-allowed);
        return reservation.IsValid()&&call();
    };
    FNativeFixedBoxPhysicalFloatRangeClearanceValuesPtr initial;
    if(!attempt([&]{return FNativeFixedBoxPhysicalFloatRangeClearanceValues::TryProve(geometry,motion,0.f,0.f,budget,initial);})||!initial)return false;
    uint32 end=0;
    if(!C::FloorNativeSweepDetail::Ordinal(motion->GetOriginalHorizon(),end)||end==0)return false;
    TSet<uint32> sampled;
    const auto probe=[&](uint32 ordinal)
    {
        FNativeRetainedBoxAtFloatInteriorValuesPtr inside;
        if(!budget.Consume()||ordinal==0||sampled.Contains(ordinal))return inside;
        sampled.Add(ordinal); // Charged before allocation; no sample retries.
        const float seconds=C::FloorNativeSweepDetail::Seconds(ordinal);
        attempt([&]
        {
            FNativeFixedBoxAtFloatTimeValuesPtr actual;
            return FNativeFixedBoxAtFloatTimeValues::TryBuild(motion,seconds,budget,actual)&&actual&&
                FNativeRetainedBoxAtFloatInteriorValues::TryFind(first,actual,budget,inside);
        });
        return inside;
    };
    const auto publish=[&](FNativeRetainedBoxAtFloatInteriorValuesPtr inside)
    {
        FNativeRetainedBoxFloatIntersectionBracketValuesPtr bracket;
        if(!inside||!FNativeRetainedBoxFloatIntersectionBracketValues::TryJoin(initial,inside,budget,bracket)||!bracket)return false;
        // Refinement is optional. Unknown/work exhaustion never invalidates a
        // still-current existing bracket and never publishes partial new bounds.
        if(budget.Remaining>budget.ConsumptionFloor)
        {
            FNativeRetainedBoxFloatIntersectionBracketValuesPtr refined;
            if(FNativeRetainedBoxFloatIntersectionBracketValues::TryRefine(bracket,maximumWorkPerProofAttempt,budget,refined))
                bracket=MoveTemp(refined);
        }
        const auto finalNumeric=Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
        if(finalNumeric.ControlFingerprint()!=numeric.ControlFingerprint()||!bracket||!bracket->MatchesInputs(geometry,motion,captured))return false;
        out=MoveTemp(bracket);return true;
    };
    // Try the originalH first, without assuming it is inside or that an empty
    // endpoint excludes an earlier excursion. Original query/Start/End/H are
    // never clipped or rebuilt for the local geometric proof intervals.
    if(const auto inside=probe(end))return publish(inside);
    struct FCell {uint32 First=0,Last=0;};TArray<FCell,TInlineAllocator<64>> stack;
    if(!budget.Consume())return false;stack.Add({0,end});
    while(!stack.IsEmpty())
    {
        if(!budget.Consume())return false;const auto cell=stack.Pop(EAllowShrinking::No);
        if(cell.First>cell.Last||cell.Last>end)return false;
        const float lo=C::FloorNativeSweepDetail::Seconds(cell.First),hi=C::FloorNativeSweepDetail::Seconds(cell.Last);
        FNativeFixedBoxPhysicalFloatRangeClearanceValuesPtr clear;
        if(attempt([&]{return FNativeFixedBoxPhysicalFloatRangeClearanceValues::TryProve(geometry,motion,lo,hi,budget,clear);})&&clear)
            continue; // Only complete all-provider PHYSICAL-corner-hull cell exclusion.
        // A failed range proof is NOT an intersection. Independently evaluate
        // one original float; any failure remains an indeterminate sample.
        const uint32 span=cell.Last-cell.First,midpoint=cell.First+span/2;
        if(const auto inside=probe(midpoint))return publish(inside);
        if(span<=1)
        {
            if(span==1)if(const auto inside=probe(cell.Last))return publish(inside);
            continue; // No subdivision or midpoint loop on singleton/adjacent cells.
        }
        if(midpoint<=cell.First||midpoint>=cell.Last)return false;
        if(!budget.Consume())return false;stack.Add({cell.First,midpoint});
        if(!budget.Consume())return false;stack.Add({midpoint+1,cell.Last});
        // Higher ordinals searched first. This traversal is NOT an earliest
        // proof: only complete prefix+inside TryJoin/refinement provide bounds.
    }
    return false; // Unknown even if the bounded search visited every cell.
}
}
