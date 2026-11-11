#pragma once
#include "IAmSpeed/World/Analytic/SignedAffineSphereResidual.h"
#include "IAmSpeed/World/Analytic/SignedAffineSphereCorrelatedPrefix.h"
#include "ReferenceTransport.h"
#include "CanonicalFixedSphereWheelProbeMath.h"
#include <atomic>
#include "HAL/PlatformTime.h"

namespace Speed::Analytic::Prepared
{
namespace AffineProjectedObservation
{
// Passive problem-key census only. No result, authority, budget or frontier is reused.
IAMSPEED_API void Record(TConstArrayView<Certified::FIntervalTensorSurface> curves,
    const Certified::FIntervalBezierCenterPath& projectedPath,const Certified::FIntervalBezierCenterPath& originalPath,
    double radius,double timeGap,double spatialGap,int32 spatialNodes,double pointWidth,double normalWidth,
    int32 maximumRefinements,uint64 numeric);
IAMSPEED_API void PublishObservation();
}
class FReferenceAffineContactPlan;
// Query-local observation only; never a geometry, authority or budget input.
struct FReferenceAffineBuildObservation86
{
    uint64 InitialNoCommonAxis=0,CoverageNotApplicable=0,ProjectionNotApplicable=0,SubsetAttempts=0,
        BlockerTests=0,BlockerSeparated=0,BlockerUnresolved=0,SubsetUnknown=0,SubsetPlans=0,
        SubsetRetainedProviders=0,SubsetSeparatedProviders=0,SubsetProofWork=0,SubsetProofTicks=0;
    bool Integrity=true;
};

class FReferenceAffineContactResult final
{
    FReferenceTransportPtr Reference;
    Certified::FReferenceCandidateSetPtr Candidates;
    uint64 NumericControl;
    Certified::FTensorSphereContactEnclosure FixedMasterEnvelope;
    FReferenceAffineContactResult(FReferenceTransportPtr reference,Certified::FReferenceCandidateSetPtr candidates,
        uint64 numeric,Certified::FTensorSphereContactEnclosure&& bounds)
        :Reference(MoveTemp(reference)),Candidates(MoveTemp(candidates)),NumericControl(numeric),FixedMasterEnvelope(MoveTemp(bounds)){}
    friend class FReferenceAffineContactPlan;
public:
    // Deliberate extraction in an authenticated fixed-source branch only.
    // No conversion to a generic independent-coefficient-family theorem.
    const Certified::FTensorSphereContactEnclosure* InspectForSameFixedSource(
        const FReferenceTransportPtr& reference,const Certified::FReferenceCandidateSetPtr& candidates) const
    {
        if(!reference||!candidates||reference!=Reference||candidates!=Candidates||
            candidates->GetScene()!=reference->GetCanonicalUnion()||reference->GetExactAffineProofIdentity().IsEmpty()||
            RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush).ControlFingerprint()!=NumericControl)
            return nullptr;
        return &FixedMasterEnvelope;
    }
};
using FReferenceAffineContactResultPtr=TSharedPtr<const FReferenceAffineContactResult,ESPMode::ThreadSafe>;
class FReferenceSphereGeneralFirstArrivalRs;

// Only the native captured-reference producer can bind the exact P/V/A/H.
class FCapturedSignedSphereMotion final
{
    friend class FReferenceSphereGeneralFirstArrivalRs;
    friend class FReferenceAffineContactPlan;
    FVector3d Position,Velocity,Acceleration;
    float Horizon=0;
    FCapturedSignedSphereMotion(FVector3d p,FVector3d v,FVector3d a,float h)
        :Position(p),Velocity(v),Acceleration(a),Horizon(h){}
};

// Query-local single-use conditional solver for an AUTHENTICATED exact product
// atlas. Generic transport construction alone is not authentication. Native
// callers must retain their closed model/DA/filter/owner/trace proof gates.
class FReferenceAffineContactPlan final
{
    using Outcome=Certified::AffineReferenceDetail::EOutcome;
    FReferenceTransportPtr Reference;
    Certified::FReferenceCandidateSetPtr Candidates;
    TArray<Certified::AffineReferenceDetail::FProduct> Products;
    TArray<Certified::FIntervalTensorSurface> OriginalSubset;
    TArray<int32> OriginalIndices,DominanceRepresentatives;
    TArray<int32> StrictlySeparatedIndices86;
    Certified::AffineReferenceDetail::FProjection Projection;
    const Certified::FDistanceBudget* OriginalAccount=nullptr;
    uint64 MinimumUsed=0,NumericControl=0;
    int32 MaximumRemaining=0,MinimumFloor=0;
    mutable std::atomic<bool> Attempted{false};
    FReferenceAffineContactPlan()=default;
    FReferenceAffineContactResultPtr SealForFixedSource(Certified::FTensorSphereContactEnclosure&& bounds) const
    {return MakeShareable(new FReferenceAffineContactResult(Reference,Candidates,NumericControl,MoveTemp(bounds)));}
    // Only the old no-common-axis refusal enters this pre-search attempt.
    // Original candidates retain their identity; every omitted original is
    // strictly separated over the ENTIRE path, not merely at an endpoint.
    static Outcome BuildSeparatedBlockers86(FReferenceTransportPtr reference,
        Certified::FReferenceCandidateSetPtr candidates,Certified::FDistanceBudget& budget,
        TSharedPtr<const FReferenceAffineContactPlan,ESPMode::ThreadSafe>& out,
        FReferenceAffineBuildObservation86* observation,int32 maximumNodesThisCall)
    {
        namespace C=Certified;namespace A=::AnalyticCertified;
        out.Reset();Outcome status=Outcome::Unknown;
        if(observation)++observation->SubsetAttempts;
        struct FCost
        {
            C::FDistanceBudget& Budget;FReferenceAffineBuildObservation86* Observation;Outcome& Status;
            uint64 Before,Start;
            FCost(C::FDistanceBudget& b,FReferenceAffineBuildObservation86* o,Outcome& s)
                :Budget(b),Observation(o),Status(s),Before(b.Used),Start(o?FPlatformTime::Cycles64():0){}
            ~FCost()
            {
                if(!Observation)return;
                const uint64 end=FPlatformTime::Cycles64();
                if(Budget.Used<Before||end<Start)Observation->Integrity=false;
                else {Observation->SubsetProofWork+=Budget.Used-Before;Observation->SubsetProofTicks+=end-Start;}
                if(Status==Outcome::Unknown)++Observation->SubsetUnknown;
            }
        } cost(budget,observation,status);
        const auto previous=::Speed::CaptureFrameFloatingPointContext();
        if(!previous.bValid)return status;
        ::Speed::FScopedFrameFloatingPointContext numeric(previous);
        if(!numeric.IsApplied())return status;
        TSharedPtr<const FReferenceAffineContactPlan,ESPMode::ThreadSafe> draft;
        const auto attempt=[&]()->Outcome
        {
            const auto control=RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
            if(maximumNodesThisCall<=0||!budget.Consume())return Outcome::Unknown;
            const auto indices=candidates->GetCandidateIndices();const auto originals=candidates->GetSurfacesForFixedSource();
            if(indices.IsEmpty()||originals.Num()!=indices.Num())return Outcome::Unknown;
            TArray<uint32> masks;int32 counts[3]{};
            for(int32 index:indices)
            {
                if(!budget.Consume())return Outcome::Unknown;
                uint32 mask=0;
                for(const auto& fact:reference->GetExactAffineExtrusions(index))
                {
                    if(!budget.Consume()||fact.Provider!=index||fact.Product.FreeWorldAxis<0||fact.Product.FreeWorldAxis>=3||
                        fact.CorrelationProofIdentity!=reference->GetExactAffineProofIdentity())return Outcome::Unknown;
                    mask|=uint32(1)<<fact.Product.FreeWorldAxis;
                }
                masks.Add(mask);for(int32 axis=0;axis<3;++axis)if(mask&(uint32(1)<<axis))++counts[axis];
            }
            int32 axis=0;for(int32 i=1;i<3;++i)if(counts[i]>counts[axis])axis=i;
            // No eligible product: no distance proof, no vacuous affine plan.
            if(counts[axis]==0)return Outcome::NotApplicable;
            TArray<C::AffineReferenceDetail::FProduct> products;TArray<FString> groups;TArray<int32> locals;
            for(int32 local=0;local<indices.Num();++local)
            {
                if(!budget.Consume())return Outcome::Unknown;
                if(!(masks[local]&(uint32(1)<<axis)))continue;
                bool found=false;
                for(const auto& fact:reference->GetExactAffineExtrusions(indices[local]))
                {
                    if(!budget.Consume())return Outcome::Unknown;
                    if(fact.Product.FreeWorldAxis!=axis)continue;
                    products.Add(fact.Product);groups.Add(fact.ExactProjectedCurveIdentity);locals.Add(local);found=true;break;
                }
                if(!found)return Outcome::Unknown;
            }
            TArray<int32> retained,representatives;
            auto choice=C::AffineReferenceDetail::SelectStrictDominantProductCover(
                MakeArrayView(products),MakeArrayView(groups),candidates->GetPathForFixedSource(),budget,retained,representatives);
            if(choice!=Outcome::Complete)
            {if(observation&&choice==Outcome::NotApplicable)++observation->CoverageNotApplicable;return choice;}
            TSharedPtr<FReferenceAffineContactPlan,ESPMode::ThreadSafe> result(new FReferenceAffineContactPlan);
            result->Reference=reference;result->Candidates=candidates;
            TArray<uint8> retainedMask;retainedMask.Init(0,locals.Num());
            for(int32 local:retained)
            {
                if(!budget.Consume()||!locals.IsValidIndex(local)||retainedMask[local]||!originals.IsValidIndex(locals[local]))return Outcome::Unknown;
                retainedMask[local]=1;result->Products.Add(products[local]);
                result->OriginalSubset.Add(originals[locals[local]]);result->OriginalIndices.Add(indices[locals[local]]);
            }
            if(representatives.Num()!=locals.Num())return Outcome::Unknown;
            for(int32 representative:representatives)
                if(!budget.Consume()||!retainedMask.IsValidIndex(representative)||!retainedMask[representative])return Outcome::Unknown;
            result->DominanceRepresentatives=MoveTemp(representatives); // Affine members ONLY; separated is not dominated.
            choice=C::AffineReferenceDetail::ProjectCompleteCandidateUnion(MakeArrayView(result->Products),
                candidates->GetPathForFixedSource(),budget,result->Projection);
            if(choice!=Outcome::Complete)
            {if(observation&&choice==Outcome::NotApplicable)++observation->ProjectionNotApplicable;return choice;}
            // Cover/projection eligibility precedes every expensive blocker proof.
            if(!budget.Consume())return Outcome::Unknown;
            const auto center=C::CenterDetail::Hull(candidates->GetPathForFixedSource());
            const auto radiusSquared=A::Square(A::Interval::Exact(candidates->GetRadiusForFixedSource()));
            if(!C::Detail::ValidPoint(center)||!A::Valid(radiusSquared))return Outcome::Unknown;
            TArray<uint8> coverage;coverage.Init(0,indices.Num());
            for(int32 local:locals)
            {if(!budget.Consume()||!coverage.IsValidIndex(local)||coverage[local])return Outcome::Unknown;coverage[local]=1;}
            for(int32 local=0;local<indices.Num();++local)
            {
                if(!budget.Consume())return Outcome::Unknown;
                if(coverage[local])continue;
                if(observation)++observation->BlockerTests;
                if(budget.ConsumptionFloor<0||budget.Remaining<=budget.ConsumptionFloor)return Outcome::Unknown;
                const int32 cap=FMath::Min(maximumNodesThisCall,budget.Remaining-budget.ConsumptionFloor);
                // Universal original tensor theorem; no product identity is
                // inferred for blockers and no thread memo is mutated here.
                const auto bound=C::BoundTensorDistanceSquaredIntervalCenter(originals[local],center,budget,0.,cap,&radiusSquared,nullptr);
                if(bound.Status==C::EDistanceBoundStatus::Unknown)return Outcome::Unknown;
                if(!(bound.LowerSquared>radiusSquared.Upper))
                {if(observation)++observation->BlockerUnresolved;return Outcome::NotApplicable;}
                coverage[local]=2;result->StrictlySeparatedIndices86.Add(indices[local]);
                if(observation)++observation->BlockerSeparated;
            }
            for(uint8 member:coverage)if(!budget.Consume()||(member!=1&&member!=2))return Outcome::Unknown;
            if(result->Products.IsEmpty()||result->Projection.Curves.Num()!=result->Products.Num()||
                result->OriginalSubset.Num()!=result->Products.Num()||result->OriginalIndices.Num()!=result->Products.Num()||
                result->DominanceRepresentatives.Num()+result->StrictlySeparatedIndices86.Num()!=indices.Num())return Outcome::Unknown;
            result->OriginalAccount=&budget;result->MinimumUsed=budget.Used;result->MaximumRemaining=budget.Remaining;
            result->MinimumFloor=budget.ConsumptionFloor;result->NumericControl=control.ControlFingerprint();draft=MoveTemp(result);
            return Outcome::Complete;
        };
        status=attempt();
        // Optional new arithmetic must preserve all prior masked sticky flags.
        if(!numeric.Restore()){status=Outcome::Unknown;draft.Reset();return status;}
        if(status==Outcome::Complete)
        {
            if(!draft)return Outcome::Unknown;
            if(observation){++observation->SubsetPlans;observation->SubsetRetainedProviders+=draft->OriginalIndices.Num();observation->SubsetSeparatedProviders+=draft->StrictlySeparatedIndices86.Num();}
            out=MoveTemp(draft);
        }
        return status;
    }
public:
    int32 GetStrictlySeparatedProviderCountForInspection86() const{return StrictlySeparatedIndices86.Num();}
    bool AdmitsRoundedSignedSeparation() const
    {
        if(!Candidates||Candidates->GetPathForFixedSource().Degree!=0||Products.IsEmpty())return false;
        for(const auto& product:Products)
            if(!Certified::SignedAffineSphereResidual::Private::Admits(product))return false;
        return true;
    }
    // Negative-only methods accept a conservative wider cover while using the literal proof radius.
    bool MatchesRoundedSeparationSource(const FReferenceTransportPtr& reference,double radius) const
    {
        return reference&&Reference==reference&&Candidates&&Candidates->GetScene()==reference->GetCanonicalUnion()&&
            Certified::CandidateDetail::PositiveRadius(radius)&&radius<=Candidates->GetRadiusForFixedSource();
    }
    Certified::SignedAffineSphereResidual::EDecision ProveRoundedSignedSeparation(
        const Certified::FIntervalPoint& center,double radius,int32 maximumCells,Certified::FDistanceBudget& budget) const
    {
        return ProveRoundedSignedSeparationImpl(center,radius,maximumCells,budget,false);
    }
    // After the temporal proof closes, its work reservation can be replaced
    // by a publication reserve. Only a new proof at the literal native point
    // may reuse this partition at the lower floor. No previous sign is reused.
    Certified::SignedAffineSphereResidual::EDecision ProveFreshRoundedSignedPoint(
        const Certified::FIntervalPoint& point,double radius,int32 maximumCells,Certified::FDistanceBudget& budget) const
    {
        return ProveRoundedSignedSeparationImpl(point,radius,maximumCells,budget,true);
    }
private:
    Certified::SignedAffineSphereResidual::EDecision ProveRoundedSignedSeparationImpl(
        const Certified::FIntervalPoint& center,double radius,int32 maximumCells,Certified::FDistanceBudget& budget,bool freshPoint) const
    {
        namespace C=Certified;
        namespace R=C::SignedAffineSphereResidual;
        using E=R::EDecision;
        if(!AdmitsRoundedSignedSeparation())return E::NotApplicable;
        if(&budget!=OriginalAccount||budget.Used<MinimumUsed||budget.Remaining>MaximumRemaining||
            (budget.ConsumptionFloor<MinimumFloor&&!freshPoint)||budget.Remaining<budget.ConsumptionFloor||
            (freshPoint&&budget.ConsumptionFloor<MinimumFloor&&!C::ContactDetail::ProjectionSingleton(center))||
            budget.Used-MinimumUsed!=uint64(int64(MaximumRemaining)-int64(budget.Remaining))||
            RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush).ControlFingerprint()!=NumericControl||
            !MatchesRoundedSeparationSource(Reference,radius))
            return E::Arithmetic;
        const auto domain=C::CenterDetail::Hull(Candidates->GetPathForFixedSource());
        if(!C::Detail::ValidPoint(domain)||!C::Detail::ValidPoint(center))return E::Arithmetic;
        for(int32 axis=0;axis<3;++axis)
            if(center.Axis[axis].Lower<domain.Axis[axis].Lower||center.Axis[axis].Upper>domain.Axis[axis].Upper)
                return E::Arithmetic;
        if(!budget.Consume())return E::Work;
        R::FCenterRange shifted;
        if(!R::BuildRoundedRange(center,radius,shifted))return E::Arithmetic;
        for(const auto& product:Products)
        {
            const auto result=R::Exclude(product,shifted,maximumCells,budget);
            if(result!=E::Clear)return result;
        }
        return E::Clear;
    }
public:
    Certified::SignedAffineSphereResidual::EDecision ProveRoundedCorrelatedSeparation(
        const FCapturedSignedSphereMotion& motion,const Certified::FIntervalPoint& whole,
        float end,double radius,Certified::FDistanceBudget& budget,bool recenterVertical=false) const
    {
        namespace C=Certified;
        namespace Joint=C::SignedAffineSphereCorrelatedPrefix;
        using E=C::SignedAffineSphereResidual::EDecision;
        if(!AdmitsRoundedSignedSeparation()||Products.Num()>8||end>1.f)return E::NotApplicable;
        if(&budget!=OriginalAccount||budget.Used<MinimumUsed||budget.Remaining>MaximumRemaining||
            budget.ConsumptionFloor<MinimumFloor||budget.Remaining<budget.ConsumptionFloor||
            budget.Used-MinimumUsed!=uint64(int64(MaximumRemaining)-int64(budget.Remaining))||
            RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush).ControlFingerprint()!=NumericControl||
            !MatchesRoundedSeparationSource(Reference,radius)||!C::Detail::ValidPoint(whole)||
            !AnalyticCertified::Finite(double(end))||!(end>=0.f&&end<=motion.Horizon))return E::Arithmetic;
        const auto domain=C::CenterDetail::Hull(Candidates->GetPathForFixedSource());
        if(!C::Detail::ValidPoint(domain))return E::Arithmetic;
        for(int32 axis=0;axis<3;++axis)
            if(whole.Axis[axis].Lower<domain.Axis[axis].Lower||whole.Axis[axis].Upper>domain.Axis[axis].Upper)
                return E::Arithmetic;
        if(!budget.Consume())return E::Work;
        Joint::FCenterNet center;
        if(!Joint::BuildCenterNet(motion.Position,motion.Velocity,motion.Acceleration,end,
            radius,Projection.FreeWorldAxis,center))return E::Arithmetic;
        for(const auto& product:Products)
        {
            if(!recenterVertical)
            {
                const auto result=Joint::Exclude(product,center,budget);
                if(result!=E::Clear)return result;
                continue;
            }
            namespace A=::AnalyticCertified;
            if(!C::SignedAffineSphereResidual::Private::Admits(product))return E::NotApplicable;
            if(!budget.Consume())return E::Work;
            // Translate both Z inputs by exactly the same scalar. The
            // residual depends only on their difference; using a local origin
            // avoids losing the strict sign to interval cancellation.
            const auto origin=A::Interval::Exact(product.Curve.ControlPoints[0].Axis[2].Lower);
            auto localProduct=product;
            auto localCenter=center;
            for(auto& point:localProduct.Curve.ControlPoints)point.Axis[2]=A::Subtract(point.Axis[2],origin);
            for(auto& z:localCenter.Axis[1])z=A::Subtract(z,origin);
            const auto result=Joint::Exclude(localProduct,localCenter,budget);
            if(result!=E::Clear)return result;
        }
        return RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush).ControlFingerprint()==NumericControl?
            E::Clear:E::Arithmetic;
    }
    int32 GetDominatedProviderCountForInspection() const{return DominanceRepresentatives.Num()-OriginalIndices.Num();}
    int32 MapRetainedProviderForInspection(int32 local) const{return OriginalIndices.IsValidIndex(local)?OriginalIndices[local]:INDEX_NONE;}
    FReferenceAffineContactPlan(const FReferenceAffineContactPlan&)=delete;
    FReferenceAffineContactPlan& operator=(const FReferenceAffineContactPlan&)=delete;

    static Outcome Build(FReferenceTransportPtr reference,Certified::FReferenceCandidateSetPtr candidates,
        Certified::FDistanceBudget& budget,TSharedPtr<const FReferenceAffineContactPlan,ESPMode::ThreadSafe>& out,
        FReferenceAffineBuildObservation86* observation=nullptr,int32 maximumBlockerNodesThisCall=8192,bool enableSeparatedBlockers=true)
    {
        out.Reset();const auto numeric=RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
        if(!reference||!candidates||!budget.Consume()||candidates->GetScene()!=reference->GetCanonicalUnion())return Outcome::Unknown;
        const auto indices=candidates->GetCandidateIndices();
        if(indices.IsEmpty()||reference->GetExactAffineProofIdentity().IsEmpty())return Outcome::NotApplicable;
        uint32 common=7;
        for(int32 index:indices)
        {
            if(!budget.Consume())return Outcome::Unknown;
            uint32 axes=0;
            for(const auto& fact:reference->GetExactAffineExtrusions(index))
            {
                if(!budget.Consume()||fact.Provider!=index||fact.Product.FreeWorldAxis<0||fact.Product.FreeWorldAxis>=3||
                    fact.CorrelationProofIdentity!=reference->GetExactAffineProofIdentity())return Outcome::Unknown;
                axes|=uint32(1)<<fact.Product.FreeWorldAxis;
            }
            common&=axes;
            if(!common)
            {
                if(!enableSeparatedBlockers)return Outcome::NotApplicable;
                if(observation)++observation->InitialNoCommonAxis;
                return BuildSeparatedBlockers86(reference,candidates,budget,out,observation,maximumBlockerNodesThisCall);
            }
        }
        TSharedPtr<FReferenceAffineContactPlan,ESPMode::ThreadSafe> result(new FReferenceAffineContactPlan);
        result->Reference=reference;result->Candidates=candidates;
        Outcome status=Outcome::NotApplicable;
        for(int32 axis=0;axis<3;++axis)
        {
            if(!(common&(uint32(1)<<axis)))continue;
            result->Products.Reset();result->OriginalSubset.Reset();result->OriginalIndices.Reset();
            result->DominanceRepresentatives.Reset();
            TArray<Certified::AffineReferenceDetail::FProduct> allProducts;
            TArray<FString> groups;
            for(int32 index:indices)
            {
                bool found=false;
                for(const auto& fact:reference->GetExactAffineExtrusions(index))
                {
                    if(!budget.Consume())return Outcome::Unknown;
                    if(fact.Product.FreeWorldAxis!=axis)continue;
                    allProducts.Add(fact.Product);groups.Add(fact.ExactProjectedCurveIdentity);found=true;break;
                }
                if(!found)return Outcome::Unknown;
            }
            TArray<int32> retained,representatives;
            status=Certified::AffineReferenceDetail::SelectStrictDominantProductCover(
                MakeArrayView(allProducts),MakeArrayView(groups),candidates->GetPathForFixedSource(),budget,retained,representatives);
            if(observation&&status==Outcome::NotApplicable)++observation->CoverageNotApplicable;
            if(status==Outcome::Unknown)return status;
            if(status!=Outcome::Complete)continue;
            const auto originals=candidates->GetSurfacesForFixedSource();
            if(originals.Num()!=indices.Num())return Outcome::Unknown;
            for(int32 local:retained)
            {
                if(!budget.Consume()||!allProducts.IsValidIndex(local)||!originals.IsValidIndex(local)||!indices.IsValidIndex(local))return Outcome::Unknown;
                result->Products.Add(allProducts[local]);result->OriginalSubset.Add(originals[local]);result->OriginalIndices.Add(indices[local]);
            }
            result->DominanceRepresentatives=MoveTemp(representatives);
            status=Certified::AffineReferenceDetail::ProjectCompleteCandidateUnion(
                MakeArrayView(result->Products),candidates->GetPathForFixedSource(),budget,result->Projection);
            if(observation&&status==Outcome::NotApplicable)++observation->ProjectionNotApplicable;
            if(status==Outcome::Unknown)return status;
            if(status==Outcome::Complete)break;
        }
        if(status!=Outcome::Complete)return status;
        if(result->Products.IsEmpty()||result->Projection.Curves.Num()!=result->Products.Num()||
            result->OriginalSubset.Num()!=result->Products.Num()||result->OriginalIndices.Num()!=result->Products.Num()||
            result->DominanceRepresentatives.Num()!=indices.Num())return Outcome::Unknown;
        result->OriginalAccount=&budget;result->MinimumUsed=budget.Used;
        result->MaximumRemaining=budget.Remaining;result->MinimumFloor=budget.ConsumptionFloor;
        result->NumericControl=numeric.ControlFingerprint();
        out=MoveTemp(result);return Outcome::Complete;
    }

    bool ProveFixedMasterLeafRank(const Certified::FTensorSphereContactLeaf& leaf,
        Certified::FDistanceBudget& budget,Certified::FIntervalPoint& normal) const
    {
        normal={};if(&budget!=OriginalAccount||!Candidates||!Reference||
            budget.Used<MinimumUsed||budget.Remaining>MaximumRemaining||budget.ConsumptionFloor<MinimumFloor||
            budget.Remaining<budget.ConsumptionFloor||
            budget.Used-MinimumUsed!=uint64(int64(MaximumRemaining)-int64(budget.Remaining))||
            RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush).ControlFingerprint()!=NumericControl||
            !budget.Consume())return false;
        const auto& indices=OriginalIndices;int32 local=INDEX_NONE;
        for(int32 i=0;i<indices.Num();++i)
        {if(!budget.Consume())return false;if(indices[i]==leaf.SurfaceIndex){local=i;break;}}
        if(!Products.IsValidIndex(local))return false;
        Certified::FReferenceDyadicCell u,v;
        if(!Certified::FixedWheelProbeMath::DecodeLeafDyadic(leaf.MinU,leaf.MaxU,u)||
            !Certified::FixedWheelProbeMath::DecodeLeafDyadic(leaf.MinV,leaf.MaxV,v))return false;
        return Certified::AffineReferenceDetail::ProveFixedProductLeafRank(Products[local],u,v,budget,normal);
    }

    struct FScopedReferencePreclear
    {
        double Cap=0.;
        int32 FreeAxis=INDEX_NONE;
        uint64 CertificateWork=0;
        uint32 AcceptedCells=0,ReusedCells=0;
        bool Attempted=false,Certified=false;
    };

    // The returned envelope is for the SAME FIXED MASTER, not every independent
    // realization of the original full coefficient boxes. The private native
    // value producer retains Reference/atlas identity through its snapshot.
    FReferenceAffineContactResultPtr Contact(Certified::FDistanceBudget& budget,
        double timeGap,double spatialGap,int32 spatialNodesPerAttempt,double pointWidth,double normalWidth,
        const Certified::FContactEnclosureFilter& originalFilter,
        const FCapturedSignedSphereMotion* signedMotion=nullptr,bool useCorrelatedTimeProof=false,
        FScopedReferencePreclear* scopedPreclear=nullptr) const
    {
        namespace C=Certified;namespace A=::AnalyticCertified;
        C::FTensorSphereContactEnclosure unknown;
        unknown.Failure=C::ETensorSphereContactFailure::Input;unknown.WorkSite=TEXT("fixed_product_input");
        bool expected=false;
        if(!Attempted.compare_exchange_strong(expected,true)||&budget!=OriginalAccount||
            budget.Used<MinimumUsed||budget.Remaining>MaximumRemaining||budget.ConsumptionFloor<MinimumFloor||
            budget.Remaining<budget.ConsumptionFloor||
            budget.Used-MinimumUsed!=uint64(int64(MaximumRemaining)-int64(budget.Remaining))||!budget.Consume()||
            !C::TimeDetail::NonnegativeFinite(pointWidth)||!(pointWidth>0)||
            !C::TimeDetail::NonnegativeFinite(normalWidth)||!(normalWidth>0))return SealForFixedSource(MoveTemp(unknown));
        const auto numeric=RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
        if(numeric.ControlFingerprint()!=NumericControl)return SealForFixedSource(MoveTemp(unknown));
        const uint64 initialUsed=budget.Used;
        uint64 lastUsed=budget.Used;int32 lastRemaining=budget.Remaining;
        const int32 enteredFloor=budget.ConsumptionFloor;
        C::FTensorSphereContactEnclosure accepted;
        bool haveAccepted=false;
        const auto live=[&]()
        {
            const bool valid=budget.Used>=lastUsed&&budget.Remaining<=lastRemaining&&
                budget.ConsumptionFloor>=enteredFloor&&budget.Remaining>=enteredFloor&&
                budget.Used-lastUsed==uint64(int64(lastRemaining)-int64(budget.Remaining))&&
                RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush).ControlFingerprint()==NumericControl;
            lastUsed=budget.Used;lastRemaining=budget.Remaining;return valid;
        };
        const auto& originalIndices=OriginalIndices;
        const auto liftAndCheck=[&](const C::FTensorSphereContactEnclosure& projected)
        {
            C::ContactKernelProfile72::FScope timing(C::ContactKernelProfile72::AffineLift);
            if(!live())return C::EContactEnclosureDecision::Refuse;
            double target=pointWidth;
            for(int32 i=0;i<projected.ConsumerWidthRefinements;++i)target*=.25;
            const auto cellWidth=A::Multiply(A::Interval::Exact(target),A::Interval::Exact(.25));
            if(!A::Valid(cellWidth)||!(cellWidth.Lower>0))return C::EContactEnclosureDecision::Refuse;
            C::FTensorSphereContactEnclosure lifted;
            const auto status=C::AffineReferenceDetail::LiftFixedMasterContact(MakeArrayView(Products),
                MakeArrayView(OriginalSubset),Candidates->GetPathForFixedSource(),projected,
                cellWidth.Lower,target,budget,lifted);
            if(status==Outcome::NeedsRefinement)return C::EContactEnclosureDecision::Refine;
            if(status!=Outcome::Complete||!live())return C::EContactEnclosureDecision::Refuse;
            for(auto& leaf:lifted.Leaves)
            {
                if(!budget.Consume()||leaf.SurfaceIndex<0||leaf.SurfaceIndex>=originalIndices.Num())return C::EContactEnclosureDecision::Refuse;
                leaf.SurfaceIndex=originalIndices[leaf.SurfaceIndex];
            }
            // Exactly ONE local->global remapping, before the original callback.
            const auto decision=originalFilter(lifted);
            if(!live())return C::EContactEnclosureDecision::Refuse;
            if(decision==C::EContactEnclosureDecision::Accept)
            {accepted=MoveTemp(lifted);haveAccepted=true;}
            return decision;
        };
        const C::FContactEnclosureFilter filter(liftAndCheck);
        AffineProjectedObservation::Record(MakeArrayView(Projection.Curves),Projection.Path,
            Candidates->GetPathForFixedSource(),Candidates->GetRadiusForFixedSource(),timeGap,spatialGap,
            spatialNodesPerAttempt,pointWidth,normalWidth,24,NumericControl);
        using T=C::TimeDetail::ESignedTimeDecision;
        using S=C::SignedAffineSphereResidual::EDecision;
        namespace R=C::SignedAffineSphereResidual;
        const double radius=Candidates->GetRadiusForFixedSource();
        const auto signedAttempt=[&](double first,double last,C::FDistanceBudget& account)->C::TimeDetail::FSignedTimeInterval
        {
            if(!signedMotion||&account!=OriginalAccount||!live())return {T::Arithmetic};
            if(!account.Consume())return {T::Work};
            R::FCenterRange endpoint;
            if(!R::BuildReferenceRange(signedMotion->Position,signedMotion->Velocity,signedMotion->Acceleration,
                signedMotion->Horizon,radius,last,last,endpoint))return {T::Arithmetic};
            for(int32 index=0;index<Products.Num();++index)
            {
                const auto& product=Products[index];
                if(!account.Consume())return {T::Work};
                double v=0;
                if(!R::ProposeCurveV(product,endpoint,v))continue;
                R::FWitness witness;
                const auto result=R::ProveInside(product,endpoint,v,account,witness);
                if(result==S::Work)return {T::Work};
                if(result==S::Arithmetic)return {T::Arithmetic};
                if(result!=S::Inside)continue;
                // The 3D inside proof is stronger than its projection. The
                // projected solver must nevertheless carry projected UV/point.
                const double originalV=product.FreeAlongU?witness.OriginalUV.Y:witness.OriginalUV.X;
                witness.OriginalPoint.Axis[product.FreeWorldAxis]=A::Interval::Exact(0);
                return {T::Inside,index,FVector2d(0.,originalV),witness.OriginalPoint,witness.SignedSquaredGap};
            }
            if(!account.Consume())return {T::Work};
            R::FCenterRange range;
            if(!R::BuildReferenceRange(signedMotion->Position,signedMotion->Velocity,signedMotion->Acceleration,
                signedMotion->Horizon,radius,first,last,range))return {T::Arithmetic};
            for(const auto& product:Products)
            {
                const auto result=R::Exclude(product,range,96,account);
                if(result==S::Work)return {T::Work};
                if(result==S::Arithmetic)return {T::Arithmetic};
                if(result==S::NotApplicable)return {T::NotApplicable};
                if(result!=S::Clear)return {T::Indeterminate};
            }
            return {T::Clear};
        };
        namespace Joint=C::SignedAffineSphereCorrelatedPrefix;
        namespace J=Joint::DetailJoint;
        TArray<J::FNode,TInlineAllocator<2>> residuals;
        bool canBuildResiduals=useCorrelatedTimeProof&&signedMotion&&
            Products.Num()>0&&Products.Num()<=8&&Candidates->GetPathForFixedSource().Degree==2;
        const auto proveCells=[&](double first,double last,C::FDistanceBudget& account)
        {
            for(const auto& root:residuals)
            {
                if(J::PositiveCoefficients(root))continue;
                J::FNode restricted;
                if(!J::RestrictTime(root,first,last,account,restricted)||
                    J::ExcludeResidual(MoveTemp(restricted),account,true)!=S::Clear)return false;
            }
            return true;
        };
        const auto earlyClear=[&](double first,double last,C::FDistanceBudget& account)->bool
        {
            if(!canBuildResiduals||&account!=OriginalAccount||!live())return false;
            constexpr int32 buildPerProduct=145,maximumCellVisits=128,maximumRestrictionSplits=2;
            const int32 buildWork=residuals.IsEmpty()?buildPerProduct*Products.Num():0;
            const int32 checkWork=(maximumCellVisits+maximumRestrictionSplits)*Products.Num();
            const int32 available=account.Remaining-account.ConsumptionFloor;
            if(available<buildWork+checkWork)return false;
            if(residuals.IsEmpty())
            {
                // The fixed original quadratic path and exact products are
                // immutable for this operation, across every nested refinement.
                Joint::FCenterNet center;
                if(!Joint::BuildReferenceCenterNet(Candidates->GetPathForFixedSource(),signedMotion->Position,
                    signedMotion->Velocity,signedMotion->Acceleration,signedMotion->Horizon,radius,
                    Projection.FreeWorldAxis,center)){canBuildResiduals=false;return false;}
                for(const auto& product:Products)
                {
                    J::FNode net;
                    if(J::BuildResidual(product,center,account,net)!=S::Clear)
                    {canBuildResiduals=false;residuals.Reset();return false;}
                    residuals.Add(MoveTemp(net));
                }
            }
            if(scopedPreclear&&scopedPreclear->FreeAxis==Projection.FreeWorldAxis&&
                0.<=first&&first<last&&last<=scopedPreclear->Cap)
            {
                // Do not pre-prove a speculative cap before the original
                // frontier has reached its first contained cell.
                constexpr int32 originalSearchReserve=20000; // Same protected reserve as the isolated 698 reference proof.
                bool justCertified=false;
                if(!scopedPreclear->Attempted&&first==0.&&
                    account.Remaining-account.ConsumptionFloor>=checkWork+originalSearchReserve)
                {
                    scopedPreclear->Attempted=true;
                    const uint64 before=account.Used;
                    {
                        C::FScopedDistanceBudgetFloor paid(account,account.Remaining-checkWork);
                        justCertified=paid.IsValid()&&proveCells(0.,scopedPreclear->Cap,account);
                    }
                    scopedPreclear->CertificateWork=account.Used-before;
                    scopedPreclear->Certified=justCertified&&live();
                }
                if(scopedPreclear->Certified&&account.Consume()&&live())
                {
                    ++scopedPreclear->AcceptedCells;
                    if(!justCertified)++scopedPreclear->ReusedCells;
                    return true;
                }
            }
            return proveCells(first,last,account)&&live();
        };
        const C::TimeDetail::FSignedTimeQuery::FClearCallback clearCallback(earlyClear);
        const C::TimeDetail::FSignedTimeQuery signedQuery(signedAttempt,useCorrelatedTimeProof?&clearCallback:nullptr);
        auto reduced=signedMotion?
            C::BoundTensorSphereFirstContactGeneric(MakeArrayView(Projection.Curves),Projection.Path,radius,
                budget,timeGap,spatialGap,spatialNodesPerAttempt,pointWidth,normalWidth,24,&filter,
                &C::ContactDetail::ThreadContactCellGeometryMemo(),&signedQuery):
            C::BoundTensorSphereFirstContact(MakeArrayView(Projection.Curves),Projection.Path,radius,
                budget,timeGap,spatialGap,spatialNodesPerAttempt,pointWidth,normalWidth,24,&filter,
                &C::ContactDetail::ThreadContactCellGeometryMemo());
        if(!live())return SealForFixedSource(MoveTemp(unknown));
        if(reduced.Status==C::ETensorSphereContactStatus::FirstContactEnclosed)
        {
            if(!haveAccepted)return SealForFixedSource(MoveTemp(unknown));
            accepted.CallInitialUsed=initialUsed;return SealForFixedSource(MoveTemp(accepted));
        }
        // No fallback/restart after this consumed attempt, even on UNKNOWN.
        // Separation/initial-inside translate by the exact product distance
        // identity and the pre-search full-path strict-interior proof.
        reduced.CallInitialUsed=initialUsed;return SealForFixedSource(MoveTemp(reduced));
    }
};
using FReferenceAffineContactPlanPtr=TSharedPtr<const FReferenceAffineContactPlan,ESPMode::ThreadSafe>;
}
