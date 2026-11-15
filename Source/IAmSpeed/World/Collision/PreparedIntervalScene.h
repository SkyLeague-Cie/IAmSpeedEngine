#pragma once
#include "IAmSpeed/World/Analytic/IndexedIntervalUnion.h"
#include "IAmSpeed/World/Analytic/SpeedAnalyticCollisionAsset.h"
namespace Speed::Analytic::Prepared
{
namespace C=Speed::Analytic::Certified;
struct FOwner
{
    uint64 SourceId=0,SurfaceId=0,FeatureId=0,PrimitiveId=0,CanonicalGroupId=0;
    uint32 MaterialId=0,ObjectType=0;uint64 BlockingChannels=0;
    bool QueryEnabled=false,CompactOptIn=false,AuthorityEligible=false,FromPlane=false;
    FIntVector OriginalDomainIndices=FIntVector(-1,-1,-1);
};
class FScene;
using FScenePtr=TSharedPtr<const FScene,ESPMode::ThreadSafe>;
// Owned values-only geometric input for a checked World publication. This is
// deliberately NOT an authority certificate or permission to execute a frame.
// The caller separately authenticates the actual asset/complete union; this
// constructor checks structural closure and keeps every owner/frame together.
class FScene final
{
    C::FIndexedIntervalUnionPtr Union;
    TArray<FOwner> Owners;
    TArray<FSpeedAnalyticAuthoredSourceRecord> Sources;
    TArray<FBoundedPlane> Planes;
    uint64 SourceHash=0;
    FScene()=default;
    static bool SameOwner(const FOwner& a,const FOwner& b)
    {
        return a.SourceId==b.SourceId&&a.SurfaceId==b.SurfaceId&&a.FeatureId==b.FeatureId&&a.PrimitiveId==b.PrimitiveId&&
            a.CanonicalGroupId==b.CanonicalGroupId&&a.MaterialId==b.MaterialId&&a.ObjectType==b.ObjectType&&a.BlockingChannels==b.BlockingChannels&&
            a.QueryEnabled==b.QueryEnabled&&a.CompactOptIn==b.CompactOptIn&&a.AuthorityEligible==b.AuthorityEligible&&a.FromPlane==b.FromPlane;
    }
public:
    static bool Build(C::FIndexedIntervalUnionPtr scene,TConstArrayView<FOwner> owners,
        TConstArrayView<FSpeedAnalyticAuthoredSourceRecord> sources,TConstArrayView<FBoundedPlane> planes,
        uint64 sourceHash,C::FDistanceBudget& budget,FScenePtr& out,FString& reason)
    {
        const FScenePtr keepPriorOutput=out;out.Reset();reason.Reset();RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
        auto refuse=[&](const TCHAR* message){reason=message;return false;};
        if(!scene||!sourceHash||scene->Num()!=owners.Num()||owners.IsEmpty()||sources.IsEmpty()||
            budget.Remaining<=0||sources.Num()>budget.Remaining)return refuse(TEXT("Incomplete prepared scene inputs"));
        TSharedPtr<FScene,ESPMode::ThreadSafe> result(new FScene);result->Union=MoveTemp(scene);result->SourceHash=sourceHash;
        TSet<uint64> sourceIds;
        for(const auto& source:sources)
        {
            if(!budget.Consume())return refuse(TEXT("Prepared source copy budget exhausted"));
            if(source.SourceId==0||sourceIds.Contains(source.SourceId)||source.Kind!=ESpeedAnalyticSourceKind::AuthoredAnalytic||
                !source.IsValid(&reason))return refuse(TEXT("Invalid or duplicate prepared authored source"));
            sourceIds.Add(source.SourceId);result->Sources.Add(source);
        }
        TMap<uint64,FOwner> primitiveOwners;TMap<uint64,TSet<FIntVector>> domains;TSet<uint64> represented;
        for(const auto& owner:owners)
        {
            if(!budget.Consume())return refuse(TEXT("Prepared owner copy budget exhausted"));
            if(!sourceIds.Contains(owner.SourceId)||!owner.SurfaceId||!owner.FeatureId||!owner.PrimitiveId||owner.ObjectType>=64)
                return refuse(TEXT("Owner does not belong to declared prepared sources"));
            if(const auto* previous=primitiveOwners.Find(owner.PrimitiveId))
            {
                if(!owner.FromPlane||!SameOwner(*previous,owner)||domains.FindChecked(owner.PrimitiveId).Contains(owner.OriginalDomainIndices))
                    return refuse(TEXT("Prepared primitive alias or duplicate domain"));
            }
            else primitiveOwners.Add(owner.PrimitiveId,owner);
            if(owner.FromPlane)
            {
                const auto& ids=owner.OriginalDomainIndices;
                if(ids.X<0||ids.Y<0||ids.Z<0||ids.X==ids.Y||ids.X==ids.Z||ids.Y==ids.Z)return refuse(TEXT("Invalid original plane domain indices"));
                domains.FindOrAdd(owner.PrimitiveId).Add(ids);
            }
            represented.Add(owner.SourceId);result->Owners.Add(owner);
        }
        if(represented.Num()!=sourceIds.Num())return refuse(TEXT("Prepared source has no geometric provider"));
        TSet<uint64> planeIds;
        for(const auto& plane:planes)
        {
            if(!budget.Consume())return refuse(TEXT("Prepared plane copy budget exhausted"));
            const auto* owner=primitiveOwners.Find(plane.PrimitiveId);
            if(!owner||!owner->FromPlane||planeIds.Contains(plane.PrimitiveId)||!sourceIds.Contains(plane.SourceId)||
                owner->SourceId!=plane.SourceId||owner->SurfaceId!=plane.SurfaceId||owner->FeatureId!=plane.FeatureId||
                owner->MaterialId!=plane.MaterialId||owner->ObjectType!=plane.ObjectType||owner->BlockingChannels!=plane.BlockingChannels||
                owner->QueryEnabled!=plane.bQueryCollisionEnabled||owner->CompactOptIn!=plane.bRequiresCompactOptIn||owner->AuthorityEligible!=plane.bAuthorityEligible)
                return refuse(TEXT("Plane frame and canonical owners differ"));
            for(const auto& point:plane.DomainVertices)
                if(!budget.Consume()||!AnalyticCertified::Valid(C::Interval::Exact(point.X))||!AnalyticCertified::Valid(C::Interval::Exact(point.Y)))
                    return refuse(TEXT("Prepared plane domain unavailable"));
            for(const auto& ids:domains.FindChecked(plane.PrimitiveId))
                if(ids.X>=plane.DomainVertices.Num()||ids.Y>=plane.DomainVertices.Num()||ids.Z>=plane.DomainVertices.Num())return refuse(TEXT("Prepared domain outside original polygon"));
            planeIds.Add(plane.PrimitiveId);result->Planes.Add(plane);
        }
        if(planeIds.Num()!=domains.Num())return refuse(TEXT("Prepared finite-plane frame missing"));
        RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);out=result;return true;
    }
    const C::FIndexedIntervalUnionPtr& GetUnion() const {return Union;}
    TConstArrayView<FOwner> GetOwners() const {return MakeArrayView(Owners);}
    TConstArrayView<FSpeedAnalyticAuthoredSourceRecord> GetSources() const {return MakeArrayView(Sources);}
    TConstArrayView<FBoundedPlane> GetPlanes() const {return MakeArrayView(Planes);}
    uint64 GetSourceHash() const {return SourceHash;}
};
}
