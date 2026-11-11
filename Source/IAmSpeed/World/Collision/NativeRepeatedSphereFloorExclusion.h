#pragma once

#include "NativeFloorSphereContactGeometry.h"
#include "PreparedNativeSolidGeometry.h"
#include "ResolvedPairSet.h"
#include "IAmSpeed/SubBodies/Solid/SolidSubBody.h"

namespace Speed::PreparedExecution
{
// Lexical World selection witness. Never store this object in a result, cache
// or in an integration/response transaction. Only the component scope can
// construct it from the actual current pair set and the independently proved
// finite t=0 floor hit. It is not a mask of geometrically absent providers.
class FNativeRepeatedSphereFloorExclusion final
{
    friend class FComponentSweepScope;
    FNativeSolidStaticRequestPtr Request;
    const Collision::FResolvedPairSet* Pairs=nullptr;
    const USSubBody* Resolver=nullptr;
    SHitResult Floor;
    int32 Excluded=INDEX_NONE;
    uint64 Pair=0;
    uint8 Generation=0;

    FNativeRepeatedSphereFloorExclusion()=default;
    FNativeRepeatedSphereFloorExclusion(const FNativeRepeatedSphereFloorExclusion&)=delete;
    FNativeRepeatedSphereFloorExclusion& operator=(const FNativeRepeatedSphereFloorExclusion&)=delete;

    static bool MatchesOwner(const Analytic::Prepared::FOwner& owner,const SHitResult& hit)
    {
        return owner.SourceId==hit.SourceId&&owner.SurfaceId==hit.SurfaceId&&owner.FeatureId==hit.FeatureId&&
            owner.PrimitiveId==hit.PrimitiveId&&owner.CanonicalGroupId==hit.CanonicalGroupId&&
            owner.MaterialId==hit.MaterialId;
    }
    static bool Bind(const FNativeSolidStaticRequestPtr& request,const FNativeSolidGeometryValues& observed,
        const SHitResult& floor,const USSubBody& resolver,const Collision::FResolvedPairSet& actualPairs,
        Analytic::Certified::FDistanceBudget& budget,FNativeRepeatedSphereFloorExclusion& out)
    {
        namespace C=Analytic::Certified;
        if(out.Request||!request||request->GetValues().Body!=&resolver||
            request->GetValues().Shape!=ENativeSolidQueryShape::Sphere||request->GetStep().Iteration!=2||
            C::KinematicCenterDetail::Bits(request->GetStep().TimePassed)!=0||
            observed.Status!=ENativeSolidGeometryStatus::CapturedSphereOverlap||
            observed.SourceId!=floor.SourceId||observed.SurfaceId!=floor.SurfaceId||
            observed.FeatureId!=floor.FeatureId||observed.PrimitiveId!=floor.PrimitiveId||
            observed.CanonicalGroupId!=floor.CanonicalGroupId||observed.MaterialId!=floor.MaterialId||
            !floor.bHit||!floor.bBlockingHit||!floor.bStartPenetrating||floor.bSurfaceNormalMayVary||
            C::KinematicCenterDetail::Bits(floor.TOI)!=0||!floor.Component.IsValid()||
            !floor.SubBody.IsExplicitlyNull()||!floor.CanonicalGroupId||floor.ContactFeatureIndexThis>=0||
            !RevalidateNativeSolidCCD(*request))return false;
        const auto snapshot=request->GetSnapshot();
        bool ambiguous=false;
        const auto* binding=snapshot?snapshot->FindUniqueContactBinding(floor.SourceId,ambiguous):nullptr;
        if(ambiguous||!binding||!binding->ContactToken.HasSameIndexAndSerialNumber(floor.Component))return false;
        const auto geometry=snapshot->GetGeometrySelection();
        if(!geometry||!geometry->HasValidAliases())return false;
        const auto owners=geometry->GetNominalScene()->GetOwners();
        int32 selected=INDEX_NONE;
        for(int32 i=0;i<owners.Num();++i)
        {
            if(!budget.Consume())return false;
            if(!MatchesOwner(owners[i],floor))continue;
            if(selected!=INDEX_NONE)return false;
            selected=i;
        }
        if(selected==INDEX_NONE)return false;
        const uint64 key=USSubBody::MakeComponentSweepPairKey(&resolver,floor,true);
        // Curved Box exact progress is excluded by the original filter's
        // positive, nonpenetrating, varying-normal conditions on this hit.
        const bool eligible=resolver.CanResolveRepeatedContact(floor);
        if(!key||eligible||!actualPairs.Contains(key)||actualPairs.CanResolve(key,eligible))return false;
        out.Request=request;out.Pairs=&actualPairs;out.Resolver=&resolver;out.Floor=floor;
        out.Excluded=selected;out.Pair=key;out.Generation=actualPairs.GetGeneration(key);
        return out.IsCurrent(request,actualPairs);
    }

public:
    bool IsCurrent(const FNativeSolidStaticRequestPtr& request,const Collision::FResolvedPairSet& actualPairs) const
    {
        FStepIdentity now;
        return Request&&Request==request&&Pairs==&actualPairs&&Resolver&&Excluded!=INDEX_NONE&&
            RevalidateNativeSolidCCD(*Request)&&ReadCurrentStep(now)&&now.QuerySerial==Request->GetStep().QuerySerial&&
            now.QueryEpoch==Request->GetStep().QueryEpoch&&now.StepSerial==Request->GetStep().StepSerial&&
            now.NumericControl==Request->GetStep().NumericControl&&
            actualPairs.Contains(Pair)&&actualPairs.GetGeneration(Pair)==Generation&&
            !Resolver->CanResolveRepeatedContact(Floor)&&!actualPairs.CanResolve(Pair,false);
    }
    bool Excludes(const FPreparedGeometrySelectionPtr& geometry,int32 originalProvider) const
    {
        return Request&&geometry&&geometry==Request->GetSnapshot()->GetGeometrySelection()&&originalProvider==Excluded;
    }
    int32 ExcludedOriginalProvider() const {return Excluded;}
    const FNativeSolidStaticRequestPtr& GetCurrentRequest() const {return Request;}
    const Collision::FResolvedPairSet* GetCurrentPairs() const {return Pairs;}
    bool ProvesNextPair(const FNativeSolidStaticRequestPtr& request,const Collision::FResolvedPairSet& actualPairs,
        const SHitResult& hit) const
    {
        if(!IsCurrent(request,actualPairs)||!hit.bHit||!hit.bBlockingHit||hit.bStartPenetrating||
            !hit.Component.IsValid()||!hit.SubBody.IsExplicitlyNull()||!hit.SourceId||!hit.SurfaceId||
            !hit.CanonicalGroupId||!(hit.TOI>0.f&&hit.TOI<=request->GetValues().Horizon))return false;
        const auto key=USSubBody::MakeComponentSweepPairKey(Resolver,hit,true);
        // This first variant only admits a distinct previously unseen pair.
        // The real World loop still runs its original filter on the actual row.
        return key&&key!=Pair&&!actualPairs.Contains(key)&&actualPairs.CanResolve(key,Resolver->CanResolveRepeatedContact(hit));
    }
};
}
