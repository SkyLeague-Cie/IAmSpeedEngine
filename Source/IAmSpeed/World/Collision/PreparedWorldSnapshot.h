#pragma once
#include "PreparedGeometrySelection.h"
#include "PreparedPhysicalQueryDomain.h"
#include "PreparedNativeOwnerBindings.h"
#include "PreparedExternalLandscape.h"
#include "UObject/WeakObjectPtr.h"
#include "UObject/WeakObjectPtrTemplates.h"
class UPrimitiveComponent;
namespace Speed
{
class FHostedGTStaticWorldLease;
class FPreparedIntervalStaticWorld;
struct FPreparedSourceContactBinding
{
    uint64 SourceId=0;
    FWeakObjectPtr Source,Contact; // GT validation only. Never resolve on worker.
    const void* SourceIdentity=nullptr;
    const void* ContactIdentity=nullptr;
    // Typed token copied on GT, so the native Contact join needs no new weak
    // lookup or UObject->weak construction on the worker.
    TWeakObjectPtr<UPrimitiveComponent> ContactToken;
    uint32 SourceObjectId=0,SourceOwnerObjectId=0,ContactObjectId=0,ContactOwnerObjectId=0;
};
// Immutable value index for the old retained-binding exclusion scan only.
// No request authority, native token resolution or geometry proof is inferred.
// Earliest ordinals preserve the exact old false/Consume failure position.
class FPreparedTraceIgnoredIdIndex final
{
    TMap<uint32,int32> FirstBinding;
    TMap<uint64,int32> UniqueSourceBinding;
    TSet<uint32> ExternalIds;
    int32 BindingCount=0,FirstInvalid=0;
    static bool ConsumePrefix(Analytic::Certified::FDistanceBudget& budget,int32 count)
    {
        // Equivalent to count calls to Consume(), including partial exhaustion
        // and Used overflow. Empty old loops never inspected the account.
        if(count<=0)return true;
        if(budget.ConsumptionFloor<0||budget.Remaining<=budget.ConsumptionFloor||budget.Used==MAX_uint64)return false;
        const uint64 capacity=FMath::Min(uint64(int64(budget.Remaining)-int64(budget.ConsumptionFloor)),MAX_uint64-budget.Used);
        const uint64 taken=FMath::Min(uint64(count),capacity);
        budget.Remaining-=int32(taken);budget.Used+=taken;
        return taken==uint64(count);
    }
public:
    FPreparedTraceIgnoredIdIndex()=default;
    FPreparedTraceIgnoredIdIndex(TConstArrayView<FPreparedSourceContactBinding> bindings,
        TConstArrayView<FPreparedLandscapePlane> external)
        :BindingCount(bindings.Num()),FirstInvalid(bindings.Num())
    {
        FirstBinding.Reserve(bindings.Num());UniqueSourceBinding.Reserve(bindings.Num());
        for(int32 i=0;i<bindings.Num();++i)
        {
            const auto& b=bindings[i];
            if(int32* existing=UniqueSourceBinding.Find(b.SourceId))*existing=INDEX_NONE;
            else UniqueSourceBinding.Add(b.SourceId,i);
            if((!b.SourceObjectId||!b.SourceOwnerObjectId||!b.ContactObjectId||!b.ContactOwnerObjectId)&&i<FirstInvalid)
                FirstInvalid=i;
            for(uint32 id:{b.SourceObjectId,b.SourceOwnerObjectId,b.ContactObjectId,b.ContactOwnerObjectId})
                if(!FirstBinding.Contains(id))FirstBinding.Add(id,i);
        }
        // Every external member is checked after the binding loop, including
        // zero IDs if present: the old external loop did not validate them.
        for(const auto& plane:external)for(const auto& member:plane.Members)
        {ExternalIds.Add(member.ObjectId);ExternalIds.Add(member.OwnerObjectId);}
    }
    int32 FindUniqueSourceOrdinal(uint64 sourceId,bool& ambiguous) const
    {
        ambiguous=false;const int32* ordinal=UniqueSourceBinding.Find(sourceId);
        if(!ordinal)return INDEX_NONE;
        ambiguous=*ordinal==INDEX_NONE;return *ordinal;
    }
    bool Allows(TConstArrayView<uint32> sourceIgnored,TConstArrayView<uint32> componentIgnored,
        Analytic::Certified::FDistanceBudget& budget) const
    {
        int32 firstFailure=FirstInvalid;bool externalFailure=false;
        const auto visit=[&](TConstArrayView<uint32> ids)
        {
            for(uint32 id:ids)
            {
                if(const int32* first=FirstBinding.Find(id))firstFailure=FMath::Min(firstFailure,*first);
                externalFailure|=ExternalIds.Contains(id);
            }
        };
        visit(sourceIgnored);visit(componentIgnored);
        const bool bindingFailure=firstFailure<BindingCount;
        if(!ConsumePrefix(budget,bindingFailure?firstFailure+1:BindingCount))return false;
        return !bindingFailure&&!externalFailure;
    }
};

struct FPreparedWorldOrigin
{
    const void* World=nullptr;const void* Host=nullptr;const void* Bridge=nullptr;const void* Asset=nullptr;
    uint64 Generation=0;
};
class FPreparedWorldSnapshot;
using FPreparedWorldSnapshotPtr=TSharedPtr<const FPreparedWorldSnapshot,ESPMode::ThreadSafe>;
class FPreparedWorldSnapshot final
{
    FPreparedGeometrySelectionPtr Geometry;
    FPreparedPhysicalQueryDomainPtr Filters;
    FPreparedNativeOwnerBindingsPtr NativeOwners;
    TArray<FPreparedSourceContactBinding> Bindings;
    TArray<FPreparedSourceContactBinding> AllContactBindings;
    FPreparedTraceIgnoredIdIndex TraceIgnoredIdIndex;
    FPreparedExternalLandscapePtr ExternalLandscape;
    FPreparedWorldOrigin Origin;
    FPreparedWorldSnapshot(FPreparedGeometrySelectionPtr geometry,FPreparedPhysicalQueryDomainPtr filters,
        TArray<FPreparedSourceContactBinding> bindings={},FPreparedWorldOrigin origin={},FPreparedNativeOwnerBindingsPtr nativeOwners={},FPreparedExternalLandscapePtr externalLandscape={})
        :Geometry(MoveTemp(geometry)),Filters(MoveTemp(filters)),NativeOwners(MoveTemp(nativeOwners)),Bindings(MoveTemp(bindings)),Origin(origin)
    {
        if(!Geometry||!Geometry->HasValidAliases())throw std::runtime_error("Incomplete prepared geometry selection");
        ExternalLandscape=MoveTemp(externalLandscape);AllContactBindings=Bindings;
        if(ExternalLandscape)for(const auto& p:ExternalLandscape->GetPlanes())
        {
            if(p.Members.IsEmpty())throw std::runtime_error("External Landscape has no retained members");
            // Explicit group adapter: Contact represents the complete validated
            // rectangular group. Every physical tile is separately retained.
            const auto& member=p.Members[0];FPreparedSourceContactBinding b;
            b.SourceId=p.Plane.SourceId;b.Source=p.OwnerToken;b.Contact=static_cast<FWeakObjectPtr>(member.Component);
            b.SourceIdentity=p.OwnerIdentity;b.ContactIdentity=member.Identity;b.ContactToken=member.Component;
            b.SourceObjectId=b.SourceOwnerObjectId=p.OwnerObjectId;b.ContactObjectId=member.ObjectId;b.ContactOwnerObjectId=member.OwnerObjectId;
            for(const auto& existing:AllContactBindings)if(existing.SourceId==b.SourceId)
                throw std::runtime_error("External/canonical SourceId alias");
            AllContactBindings.Add(b);
        }
        TraceIgnoredIdIndex=FPreparedTraceIgnoredIdIndex(AllContactBindings,
            ExternalLandscape?ExternalLandscape->GetPlanes():TConstArrayView<FPreparedLandscapePlane>{});
    }
    friend class FHostedGTStaticWorldLease;
    friend class FPreparedIntervalStaticWorld;
public:
    const FPreparedGeometrySelectionPtr& GetGeometrySelection() const {return Geometry;}
    const Analytic::Prepared::FScenePtr& GetNominalScene() const {return Geometry->GetNominalScene();}
    const Analytic::Prepared::FReferenceTransportPtr& GetReference() const {return Geometry->GetReference();}
    const FPreparedPhysicalQueryDomainPtr& GetFilters() const {return Filters;}
    const FPreparedWorldOrigin& GetOrigin() const {return Origin;}
    const FPreparedNativeOwnerBindingsPtr& GetNativeOwners() const {return NativeOwners;}
    TConstArrayView<FPreparedSourceContactBinding> GetBindings() const {return Bindings;}
    TConstArrayView<FPreparedSourceContactBinding> GetAllContactBindings() const {return AllContactBindings;}
    const FPreparedExternalLandscapePtr& GetExternalLandscape() const {return ExternalLandscape;}
    bool AllowsNativeIgnoredIds(TConstArrayView<uint32> sourceIgnored,TConstArrayView<uint32> componentIgnored,
        Analytic::Certified::FDistanceBudget& budget) const
    {return TraceIgnoredIdIndex.Allows(sourceIgnored,componentIgnored,budget);}
    const FPreparedSourceContactBinding* FindUniqueContactBinding(uint64 sourceId,bool& ambiguous) const
    {
        ambiguous=false;
        if(Analytic::Certified::PlanarFamilyTimeShortcutControl::RuntimeEnabled())
        {
            const int32 ordinal=TraceIgnoredIdIndex.FindUniqueSourceOrdinal(sourceId,ambiguous);
            return AllContactBindings.IsValidIndex(ordinal)?&AllContactBindings[ordinal]:nullptr;
        }
        const FPreparedSourceContactBinding* found=nullptr;
        for(const auto& row:AllContactBindings)if(row.SourceId==sourceId)
        {if(found){ambiguous=true;return nullptr;}found=&row;}
        return found;
    }
    // Owns native immutable inputs only. Strong UObject pins belong to the GT
    // owner until the exact parked boundary/join; no last destruction on worker.
};
}
