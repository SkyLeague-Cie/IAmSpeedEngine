#pragma once
#include "NativeRetainedBoxReferenceFirstEntryValues.h"
namespace Speed::PreparedExecution
{
// Deterministic, bounded search for the EXISTING globally first REFERENCE
// entry proof. No caller provider/corner/sign/parameter/time hints. A failed
// search is Unknown, including empty candidates, unsupported features, initial
// contact, dyadic-boundary roots and exhaustion. NEVER a full-Box miss.
// Per-attempt work is a TECHNICAL quota, not spatial/normal/time tolerance.
// The same budget records every failed attempt; reservations never refund it.
inline bool FindNativeRetainedBoxReferenceFirstEntry(FPreparedGeometrySelectionPtr geometry,
    FNativeFixedBoxReferenceMotionValuesPtr motion,int32 maximumWorkPerProofAttempt,
    Analytic::Certified::FDistanceBudget& budget,FNativeRetainedBoxReferenceFirstEntryValuesPtr& out)
{
    const auto keepPrior=out;out.Reset();
    namespace C=Analytic::Certified;
    Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(maximumWorkPerProofAttempt<=0||!geometry||!geometry->HasValidAliases()||
        geometry->GetModel()!=EPreparedGeometryModel::RetainedReference||!motion||!budget.Consume())return false;
    C::FIntervalPoint fullReferenceBox;
    if(!FNativeRetainedBoxPreCellClearanceValues::BoundWholeBoxReference(*motion,{},false,budget,fullReferenceBox))return false;
    C::FReferenceBoundsCandidateSetPtr candidates;
    if(!C::FReferenceBoundsCandidateSet::Select(geometry->GetSelectedUnion(),fullReferenceBox,budget,candidates)||
        !candidates||candidates->GetScene()!=geometry->GetSelectedUnion())return false;
    struct FCell {int32 Provider=INDEX_NONE;C::FReferenceDyadicCell U,V,Time;};
    TArray<FCell> queue;TSet<FString> visited;
    const auto append=[&](const FCell& cell)
    {
        // Charge before key construction/allocation, even for a duplicate.
        if(!budget.Consume()||!cell.U.IsValid()||!cell.V.IsValid()||!cell.Time.IsValid())return false;
        const FString key=FString::Printf(TEXT("%d/%u/%llu/%u/%llu/%u/%llu"),cell.Provider,
            cell.U.Depth,static_cast<unsigned long long>(cell.U.Index),cell.V.Depth,static_cast<unsigned long long>(cell.V.Index),
            cell.Time.Depth,static_cast<unsigned long long>(cell.Time.Index));
        if(!visited.Contains(key)){visited.Add(key);queue.Add(cell);}return true;
    };
    const auto owners=geometry->GetNominalScene()->GetOwners();
    const auto mapping=geometry->GetReference()->GetProviderFaceIndices();const auto faces=geometry->GetReference()->GetFaces();
    for(int32 provider:candidates->GetCandidateIndices())
    {
        if(!budget.Consume()||provider<0||provider>=owners.Num()||provider>=mapping.Num())return false;
        const int32 face=mapping[provider];if(face<0||face>=faces.Num())return false;
        // This search's local root type supports rectangular curved tensors.
        // Skipping a plane here is only search incompleteness, NEVER a miss;
        // every success still proves prefix/domain against the WHOLE union.
        if(!owners[provider].FromPlane&&!faces[face].FromPlane&&!append(FCell{provider,{},{},{}}))return false;
    }
    for(int32 next=0;next<queue.Num();++next)
    {
        if(!budget.Consume())return false;const auto cell=queue[next];
        C::FIntervalTensorSurface original,tensor;
        if(!geometry->GetSelectedUnion()->CopySurfaceForQuery(cell.Provider,budget,original)||
            !C::RestrictReferenceTensorDyadic(original,cell.U,cell.V,budget,tensor))return false;
        C::Detail::FNode net;net.DegreeU=tensor.DegreeU;net.DegreeV=tensor.DegreeV;net.Net=tensor.ControlPoints;
        C::FIntervalPoint surfaceHull;if(!C::ContactDetail::NetHull(net,surfaceHull))return false;
        bool possible=false;
        for(int32 corner=0;corner<8;++corner)
        {
            if(!budget.Consume())return false;C::FIntervalBezierCenterPath path;
            if(!C::RestrictReferencePathDyadic(motion->GetCornerPathsForInspection()[corner],cell.Time,budget,path))return false;
            const auto pointHull=C::CenterDetail::Hull(path);if(!C::Detail::ValidPoint(pointHull))return false;
            bool separate=false;
            for(int32 d=0;d<3;++d)separate|=surfaceHull.Axis[d].Upper<pointHull.Axis[d].Lower||
                pointHull.Axis[d].Upper<surfaceHull.Axis[d].Lower;
            if(separate)continue; // Only THIS point/tensor root is excluded.
            possible=true;
            for(int32 sign:{1,-1})
            {
                if(!budget.Consume())return false;
                // A hard local proof may hit its quota. Future cells keep the
                // reserved work, while Remaining/Used retain ALL spent work.
                const int32 available=budget.Remaining-budget.ConsumptionFloor;
                if(available<=0)return false;
                const int32 allowed=FMath::Min(available,maximumWorkPerProofAttempt);
                C::FScopedDistanceBudgetFloor reservation(budget,budget.Remaining-allowed);
                if(!reservation.IsValid())return false;
                FNativeRetainedBoxCornerReferenceValuesPtr root;
                FNativeRetainedBoxPreCellClearanceValuesPtr prefix;
                FNativeRetainedBoxRootDomainClearanceValuesPtr domain;
                FNativeRetainedBoxReferenceFirstEntryValuesPtr first;
                if(FNativeRetainedBoxCornerReferenceValues::TryProve(geometry,motion,cell.Provider,cell.U,cell.V,cell.Time,
                    corner,sign,budget,root)&&root&&
                    FNativeRetainedBoxPreCellClearanceValues::TryProve(root,budget,prefix)&&
                    FNativeRetainedBoxRootDomainClearanceValues::TryProve(root,budget,domain)&&
                    FNativeRetainedBoxReferenceFirstEntryValues::TryProve(prefix,domain,budget,first)&&first&&
                    first->MatchesInputs(geometry,motion->GetCapturedForInspection()))
                {
                    Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
                    out=MoveTemp(first);return true;
                }
            }
        }
        if(!possible)continue;
        // Explore time-only as well as each spatial split. Forcing U/V first
        // could strand u=.5/v=.5 roots on a subdivision boundary forever.
        // All three routes share the SAME budget; visited keys prevent loops.
        for(int32 axis:{2,0,1})
        {
            const auto parent=axis==0?cell.U:(axis==1?cell.V:cell.Time);
            if(parent.Depth==52)continue;
            for(uint64 side:{uint64(0),uint64(1)})
            {
                FCell child=cell;C::FReferenceDyadicCell code;code.Depth=parent.Depth+1;code.Index=parent.Index*2+side;
                if(axis==0)child.U=code;else if(axis==1)child.V=code;else child.Time=code;
                if(!append(child))return false;
            }
        }
    }
    return false; // Never an all-provider miss, even when the queue is empty.
}
}
