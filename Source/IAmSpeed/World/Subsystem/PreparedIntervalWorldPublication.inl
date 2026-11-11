#include "IAmSpeed/World/Collision/PreparedIntervalStaticWorld.h"
#include "IAmSpeed/World/Collision/HostedGTStaticWorldLease.h"
namespace
{
bool CapturePreparedPhysicalFilters(UWorld& world,const TSet<ISpeedComponent*>& adapters,
    TArray<Speed::FPreparedPhysicalFilterRecord>& records,FString& reason)
{
    records.Reset();TSet<USSubBody*> bodies;
    for(auto* adapter:adapters)
    {
        if(!adapter){reason=TEXT("Null effective prepared adapter");return false;}
        for(auto* body:adapter->GetSubBodies())
        {
            if(!IsValid(body)||body->GetWorld()!=&world||!body->IsRegistered()||body->GetParentComponent()!=adapter||bodies.Contains(body)||uint32(body->GetCollisionChannel())>=64)
            {reason=TEXT("Prepared physical body inventory/ownership/channel is not closed");records.Reset();return false;}
            bodies.Add(body);uint64 mask=0;
            for(int32 channel=0;channel<UE_ARRAY_COUNT(FCollisionResponseContainer::EnumArray);++channel)
                if(body->GetResponseParams().CollisionResponse.GetResponse(ECollisionChannel(channel))==ECR_Block)mask|=uint64(1)<<channel;
            records.Add({adapter,body,body->GetPathName(),uint8(body->GetCollisionChannel()),mask});
        }
    }

    if(records.IsEmpty()){reason=TEXT("Prepared physical query inventory is empty");return false;}
    return true;
}
// Query-enabled metadata alone is not a Chaos scene collider. In particular
// rendering-only components inherit BlockAll without any BodySetup/actor.
// This is a fresh non-participation proof, never a class/name whitelist and
// never an exemption for unregistered custom IAmSpeed SubBodies.
bool HasNoPreparedWorldQueryGeometry(UPrimitiveComponent& primitive)
{
    const auto stable=[&]()
    {
        return primitive.IsRegistered()&&!primitive.DeferredCreatePhysicsStateScene&&
            !primitive.IsAsyncCreatePhysicsStateRunning()&&!primitive.IsAsyncDestroyPhysicsStateRunning()&&
            (primitive.IsPhysicsStateCreated()||!primitive.ShouldCreatePhysicsState());
    };
    if(!stable()||primitive.GetBodySetup()!=nullptr)return false;
    const auto* body=primitive.GetBodyInstance();
    if(body&&body->IsValidBodyInstance())return false;
    for(const auto* object:primitive.GetAllPhysicsObjects())if(object)return false;
    // Virtual getters may initialize component state. Close the same facts
    // after those calls, as well as at every later publication/readback gate.
    body=primitive.GetBodyInstance();
    return stable()&&primitive.GetBodySetup()==nullptr&&(!body||!body->IsValidBodyInstance());
}
bool SamePreparedAuthoredRecord(const FSpeedAnalyticAuthoredSourceRecord& a,const FSpeedAnalyticAuthoredSourceRecord& b)
{
    return a.Kind==b.Kind&&a.SourceId==b.SourceId&&a.RecipeSchema==b.RecipeSchema&&a.RecipeSha256==b.RecipeSha256&&
        a.PayloadSha256==b.PayloadSha256&&a.SourceComponentPath==b.SourceComponentPath&&a.ContactComponentPath==b.ContactComponentPath&&
        a.OwnerWorldTransform.Equals(b.OwnerWorldTransform,0.)&&a.ContactWorldTransform.Equals(b.ContactWorldTransform,0.)&&
        a.ObjectType==b.ObjectType&&a.CollisionEnabled==b.CollisionEnabled&&a.bQueryCollisionEnabled==b.bQueryCollisionEnabled&&
        a.BlockingChannels==b.BlockingChannels&&a.OverlapChannels==b.OverlapChannels;
}
}
bool USpeedWorldSubsystem::ValidatePreparedIntervalSources(const USpeedAnalyticCollisionAsset& asset,
    const Speed::Analytic::Prepared::FScene& scene,TMap<uint64,TWeakObjectPtr<UPrimitiveComponent>>& out,FString& reason,
    const Speed::FPreparedExternalLandscape* externalLandscape) const
{
    // Both callers hold PendingCS across this entire validation and commit/readback.
    out.Reset();reason.Reset();auto refuse=[&](const TCHAR* message){reason=message;out.Reset();return false;};
    auto* world=GetWorld();if(!IsInGameThread()||!world||world->bIsTearingDown||!world->HasBegunPlay()||world->WorldType!=EWorldType::Game)
        return refuse(TEXT("Prepared interval publication requires a live begun Game World on GT"));
    if(asset.GetClass()!=USpeedAnalyticCollisionAsset::StaticClass()||asset.SourceHash!=scene.GetSourceHash()||
        asset.AuthoredSources.Num()!=scene.GetSources().Num())return refuse(TEXT("Prepared source asset identity/hash/record count differs"));
    // The Module must authenticate the complete numeric payload inside this
    // same lexical lease, before supplying this immutable prepared input.
    // No legacy runtime builder, flag promotion, cache or mesh proxy is used.
    if(!asset.MeshSources.IsEmpty()||!asset.Triangles.IsEmpty()||!asset.VertexPositions.IsEmpty()||!asset.VertexNormals.IsEmpty()||
        !asset.IndexedTriangles.IsEmpty()||!asset.ExtrudedQuinticPatches.IsEmpty()||!asset.PiecewiseTensorBezierPatches.IsEmpty())
        return refuse(TEXT("Prepared interval scene cannot omit another geometry family"));
    int32 tensors=0;for(const auto& owner:scene.GetOwners())tensors+=owner.FromPlane?0:1;
    if(tensors!=asset.TensorBezierPatches.Num()||scene.GetPlanes().Num()!=asset.BoundedPlanes.Num())return refuse(TEXT("Prepared geometry family counts differ"));
    TMap<uint64,const FSpeedAnalyticAuthoredSourceRecord*> expected;
    for(const auto& record:scene.GetSources())
    {
        if(expected.Contains(record.SourceId))return refuse(TEXT("Duplicate prepared source identity"));
        const auto* live=asset.AuthoredSources.FindByPredicate([&](const auto& r){return r.SourceId==record.SourceId;});
        if(!live||!SamePreparedAuthoredRecord(*live,record))return refuse(TEXT("Prepared source provenance/policy/frame differs from actual asset"));
        expected.Add(record.SourceId,&record);
    }
    TMap<uint64,USpeedAnalyticSourceComponent*> declarations;TSet<UPrimitiveComponent*> contacts;
    for(TActorIterator<AActor> it(world);it;++it)
    {
        TArray<USpeedAnalyticSourceComponent*> sources;it->GetComponents(sources);
        for(auto* source:sources)
        {
            if(!source||!source->IsRegistered()||source->GetWorld()!=world||!source->AuthoredSourceId||source->bIncludeOwnedStaticMeshes||
                source->AnalyticCollisionAsset.Get()!=&asset||!expected.Contains(source->AuthoredSourceId)||declarations.Contains(source->AuthoredSourceId))
                return refuse(TEXT("Live source inventory contains an unbound, duplicate or legacy declaration"));
            auto* contact=source->ResolveAuthoredContact(*expected.FindChecked(source->AuthoredSourceId),asset,&reason);
            if(!contact||contact->GetWorld()!=world||contacts.Contains(contact))return refuse(TEXT("Live authored contact missing or aliased"));
            declarations.Add(source->AuthoredSourceId,source);contacts.Add(contact);out.Add(source->AuthoredSourceId,contact);
        }
    }
    if(declarations.Num()!=expected.Num())return refuse(TEXT("Prepared source inventory incomplete in live World"));
    TSet<ISpeedComponent*> admittedAdapters;
    for(auto* adapter:SimulationWorld.GetAdapters())if(adapter)admittedAdapters.Add(adapter);
    { for(const auto& op:PendingOps)if(op.Comp){if(op.bAdd)admittedAdapters.Add(op.Comp);else admittedAdapters.Remove(op.Comp);} }
    TArray<const void*> adapterInventory;for(auto* adapter:admittedAdapters)adapterInventory.Add(adapter);
    TArray<Speed::FPreparedPhysicalFilterRecord> physicalFilters;
    if(!CapturePreparedPhysicalFilters(*world,admittedAdapters,physicalFilters,reason))return false;
    if(PreparedIntervalScene)
    {
        const auto* base=SimulationWorld.GetStaticCollisionWorld();
        if(!base){return refuse(TEXT("Prepared physical provider was removed"));}
        const auto snapshot=base->GetPreparedSnapshot();
        if(!snapshot||snapshot->GetNominalScene()!=PreparedIntervalScene)return refuse(TEXT("Prepared provider snapshot type/identity differs"));
        const auto& frozen=snapshot->GetFilters();
        if(!frozen||!frozen->Matches(physicalFilters,adapterInventory))return refuse(TEXT("Prepared physical body/filter inventory changed"));
    }
    if(externalLandscape&&!externalLandscape->MatchesGT(*world,reason))return refuse(TEXT("Retained Landscape collision changed"));
    // Closure is limited to the ACTUAL physical query-side filters plus the
    // WorldStatic object-query domain. Presentation Camera/Visibility queries
    // are a separate purpose and remain outside this closed provider's authority.
    for(TActorIterator<AActor> it(world);it;++it)
    {
        TArray<UPrimitiveComponent*> primitives;it->GetComponents(primitives);
        for(auto* primitive:primitives)
        {
            if(!primitive||!primitive->IsRegistered()||!primitive->IsQueryCollisionEnabled()||contacts.Contains(primitive)||
                (externalLandscape&&externalLandscape->ContainsMember(primitive)))continue;
            uint64 blockingMask=0;
            for(int32 channel=0;channel<UE_ARRAY_COUNT(FCollisionResponseContainer::EnumArray);++channel)
                if(primitive->GetCollisionResponseToChannel(ECollisionChannel(channel))==ECR_Block)blockingMask|=uint64(1)<<channel;
            if(!Speed::FPreparedPhysicalQueryDomain::PrimitiveCanParticipate(uint32(primitive->GetCollisionObjectType()),blockingMask,physicalFilters))continue;
            if(auto* body=Cast<USSubBody>(primitive))
            {
                auto* owner=body->GetParentComponent();
                if(owner&&admittedAdapters.Contains(owner)&&owner->GetSubBodies().Contains(body))continue;
            }
            if(!Cast<USSubBody>(primitive)&&HasNoPreparedWorldQueryGeometry(*primitive))continue;
            // Both trace-side response tables matter. No object-type, component
            // name or channel is exempted by convention; object queries count
            // WorldStatic even if its trace responses ignore every channel.
            const auto* diagnosticBody=Cast<USSubBody>(primitive);
            const auto* diagnosticParent=diagnosticBody?diagnosticBody->GetParentComponent():nullptr;
            return refuse(*FString::Printf(TEXT("Live World contains unbound blocking collision outside the prepared union/adapters; Primitive=%s Class=%s Owner=%s Profile=%s ObjectType=%d Enabled=%d BlockingMask=%016llX SubBody=%d ParentPresent=%d ParentAdmitted=%d BodyListed=%d"),
                *primitive->GetPathName(),*primitive->GetClass()->GetPathName(),*GetNameSafe(primitive->GetOwner()),*primitive->GetCollisionProfileName().ToString(),
                int32(primitive->GetCollisionObjectType()),int32(primitive->GetCollisionEnabled()),blockingMask,diagnosticBody?1:0,diagnosticParent?1:0,
                diagnosticParent&&admittedAdapters.Contains(const_cast<ISpeedComponent*>(diagnosticParent))?1:0,
                diagnosticParent&&diagnosticParent->GetSubBodies().Contains(const_cast<USSubBody*>(diagnosticBody))?1:0));
        }
    }
    return true;
}
bool USpeedWorldSubsystem::TryPublishPreparedIntervalWorld(const Speed::FHostedGTStaticWorldLease& lease,
    const USpeedAnalyticCollisionAsset& asset,TSharedPtr<const Speed::Analytic::Prepared::FScene,ESPMode::ThreadSafe> scene,FString& reason)
{
    reason.Reset();
    if(!lease.IsFor(*this)||!bPreparedWorldResetComplete||!bPreparedIntervalRouteRequested||bCanonicalFrameActive||StepSerial!=0||PreparedIntervalScene||
        SimulationWorld.GetStaticCollisionWorld()||!scene||!Speed::Analytic::FStaticWorldQueryAudit::IsSurfaceAnalyticBackend())
    {reason=TEXT("Prepared interval World publication boundary or one-shot state refused");return false;}
    Speed::Analytic::RequireCurrentAnalyticNumericContext(Speed::Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
    // Keep registration operations stable across census, frozen policy capture
    // and publication. The live worker is already fenced by this exact lease.
    FScopeLock inputInventoryLock(&PendingCS);
    TMap<uint64,TWeakObjectPtr<UPrimitiveComponent>> bindings;
    if(!ValidatePreparedIntervalSources(asset,*scene,bindings,reason))return false;
    TSet<ISpeedComponent*> admittedAdapters;
    for(auto* adapter:SimulationWorld.GetAdapters())if(adapter)admittedAdapters.Add(adapter);
    {for(const auto& op:PendingOps)if(op.Comp){if(op.bAdd)admittedAdapters.Add(op.Comp);else admittedAdapters.Remove(op.Comp);}}
    TArray<const void*> adapterInventory;for(auto* adapter:admittedAdapters)adapterInventory.Add(adapter);
    TArray<Speed::FPreparedPhysicalFilterRecord> actualFilters;Speed::FPreparedPhysicalQueryDomainPtr filters;
    if(!CapturePreparedPhysicalFilters(*GetWorld(),admittedAdapters,actualFilters,reason)||!Speed::FPreparedPhysicalQueryDomain::Build(lease.GetGeneration(),MoveTemp(actualFilters),adapterInventory,filters))
    {if(reason.IsEmpty())reason=TEXT("Prepared physical filter domain could not be frozen");return false;}
    auto provider=MakeUnique<Speed::FPreparedIntervalStaticWorld>(scene,MoveTemp(filters));
    if(!lease.IsFor(*this)||bCanonicalFrameActive||StepSerial!=0){reason=TEXT("Prepared interval World boundary changed");return false;}
    // No callbacks or yield can observe the intermediate assignments. A paused
    // worker sees the owned provider and source map together after lease release.
    PreparedIntervalAsset=const_cast<USpeedAnalyticCollisionAsset*>(&asset);
    PreparedIntervalScene=MoveTemp(scene);AnalyticSourceComponents=MoveTemp(bindings);
    SimulationWorld.SetStaticCollisionWorld(MoveTemp(provider));
    return true; // Preparation published; provider still refuses every Step.
}
TSharedPtr<const Speed::Analytic::Prepared::FScene,ESPMode::ThreadSafe> USpeedWorldSubsystem::ReadPreparedIntervalWorldAtBoundary(
    const Speed::FHostedGTStaticWorldLease& lease) const
{
    if(!lease.IsFor(*this)||!bPreparedWorldResetComplete||bCanonicalFrameActive||StepSerial!=0||!PreparedIntervalScene||!PreparedIntervalAsset.IsValid())return {};
    FScopeLock inputInventoryLock(&PendingCS);
    TMap<uint64,TWeakObjectPtr<UPrimitiveComponent>> actual;FString reason;
    if(!ValidatePreparedIntervalSources(*PreparedIntervalAsset.Get(),*PreparedIntervalScene,actual,reason)||actual.Num()!=AnalyticSourceComponents.Num())return {};
    for(const auto& row:actual)if(!AnalyticSourceComponents.Contains(row.Key)||AnalyticSourceComponents.FindChecked(row.Key)!=row.Value)return {};
    const auto* provider=SimulationWorld.GetStaticCollisionWorld();
    const auto snapshot=provider?provider->GetPreparedSnapshot():Speed::FPreparedWorldSnapshotPtr{};
    if(!snapshot||snapshot->GetNominalScene()!=PreparedIntervalScene||!snapshot->GetFilters()||
        snapshot->GetFilters()->GetGeneration()!=lease.GetGeneration())return {};
    return PreparedIntervalScene;
}

namespace
{
bool CapturePreparedReferenceBindings(UWorld& world,const USpeedAnalyticCollisionAsset& asset,
    const TMap<uint64,TWeakObjectPtr<UPrimitiveComponent>>& contacts,TArray<Speed::FPreparedSourceContactBinding>& out)
{
    out.Reset();TSet<uint64> seen;
    for(TActorIterator<AActor> it(&world);it;++it)
    {
        TArray<USpeedAnalyticSourceComponent*> sources;it->GetComponents(sources);
        for(auto* source:sources)
        {
            if(!IsValid(source)||!source->IsRegistered()||source->GetWorld()!=&world||
                source->AnalyticCollisionAsset.Get()!=&asset||seen.Contains(source->AuthoredSourceId))return false;
            const auto* weak=contacts.Find(source->AuthoredSourceId);auto* contact=weak?weak->Get():nullptr;
            if(!IsValid(contact)||!contact->IsRegistered()||contact->GetWorld()!=&world)return false;
            Speed::FPreparedSourceContactBinding b;b.SourceId=source->AuthoredSourceId;
            b.Source=FWeakObjectPtr(source);b.Contact=FWeakObjectPtr(contact);
            if(!source->GetOwner()||!contact->GetOwner())return false;
            b.SourceIdentity=source;b.ContactIdentity=contact;b.ContactToken=contact;
            b.SourceObjectId=source->GetUniqueID();b.SourceOwnerObjectId=source->GetOwner()->GetUniqueID();
            b.ContactObjectId=contact->GetUniqueID();b.ContactOwnerObjectId=contact->GetOwner()->GetUniqueID();
            out.Add(b);seen.Add(b.SourceId);
        }
    }
    out.Sort([](const auto& a,const auto& b){return a.SourceId<b.SourceId;});
    return out.Num()==contacts.Num();
}
bool SamePreparedReferenceBindings(TConstArrayView<Speed::FPreparedSourceContactBinding> a,
    TConstArrayView<Speed::FPreparedSourceContactBinding> b)
{
    if(a.Num()!=b.Num())return false;
    for(int32 i=0;i<a.Num();++i)
        if(a[i].SourceId!=b[i].SourceId||a[i].Source!=b[i].Source||a[i].Contact!=b[i].Contact||
            a[i].SourceIdentity!=b[i].SourceIdentity||a[i].ContactIdentity!=b[i].ContactIdentity||
            !a[i].ContactToken.HasSameIndexAndSerialNumber(b[i].ContactToken)||
            a[i].SourceObjectId!=b[i].SourceObjectId||a[i].SourceOwnerObjectId!=b[i].SourceOwnerObjectId||
            a[i].ContactObjectId!=b[i].ContactObjectId||a[i].ContactOwnerObjectId!=b[i].ContactOwnerObjectId)return false;
    return true;
}
}
namespace Speed
{
bool FHostedGTStaticWorldLease::PublishCanonicalReferenceSnapshot(const USpeedAnalyticCollisionAsset& asset,
    Analytic::Prepared::FReferenceTransportPtr reference,FPreparedWorldSnapshotPtr& out,FString& reason,FPreparedNativeOwnerInputPtr nativeInput) const
{
    return PublishCanonicalReferenceSnapshot(asset,MoveTemp(reference),out,reason,MoveTemp(nativeInput),nullptr);
}
bool FHostedGTStaticWorldLease::PublishCanonicalReferenceSnapshot(const USpeedAnalyticCollisionAsset& asset,
    Analytic::Prepared::FReferenceTransportPtr reference,FPreparedWorldSnapshotPtr& out,FString& reason,
    FPreparedNativeOwnerInputPtr nativeInput,const FPreparedCanonicalQueryServiceFactoryGT* factory) const
{
    return PublishCanonicalReferenceSnapshot(asset,MoveTemp(reference),out,reason,MoveTemp(nativeInput),factory,nullptr);
}
bool FHostedGTStaticWorldLease::PublishCanonicalReferenceSnapshot(const USpeedAnalyticCollisionAsset& asset,
    Analytic::Prepared::FReferenceTransportPtr reference,FPreparedWorldSnapshotPtr& out,FString& reason,
    FPreparedNativeOwnerInputPtr nativeInput,const FPreparedCanonicalQueryServiceFactoryGT* factory,
    const FPreparedNativeFrameConsumerFactoryGT* nativeFactory) const
{
    out.Reset();reason.Reset();auto* bridge=Bridge.Get();auto* host=Host.Get();auto* world=World.Get();
    if(!bridge||!host||!world||!IsFor(*bridge)||!reference||!reference->GetNominalScene()||
        !bridge->bPreparedWorldResetComplete||!bridge->bPreparedIntervalRouteRequested||bridge->bCanonicalFrameActive||
        (bridge->StepSerial!=0&&!(NativeGenerationReplacement&&nativeFactory))||
        bridge->PreparedIntervalScene||bridge->SimulationWorld.GetStaticCollisionWorld()||
        !Analytic::FStaticWorldQueryAudit::IsSurfaceAnalyticBackend())
    {reason=TEXT("Canonical prepared snapshot publication boundary/one-shot state refused");return false;}
    const auto nominal=reference->GetNominalScene();
    if(!reference->GetCanonicalUnion()||reference->GetCanonicalUnion()->Num()!=nominal->GetOwners().Num())
    {reason=TEXT("Canonical reference/nominal structural closure differs");return false;}
    Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
    FScopeLock lock(&bridge->PendingCS);
    TMap<uint64,TWeakObjectPtr<UPrimitiveComponent>> contacts;
    // Full numerical DA authentication is performed by the closed game producer
    // INSIDE this same lease. This native check claims census/policy only.
    FPreparedExternalLandscapePtr externalLandscape;
    if(!FPreparedExternalLandscape::CaptureGT(*world,externalLandscape,reason)||
        !bridge->ValidatePreparedIntervalSources(asset,*nominal,contacts,reason,externalLandscape.Get()))return false;
    TArray<FPreparedSourceContactBinding> bindings;
    if(!CapturePreparedReferenceBindings(*world,asset,contacts,bindings))
    {reason=TEXT("Canonical Source/Contact instance binding unavailable");return false;}
    TSet<ISpeedComponent*> admitted;
    for(auto* adapter:bridge->SimulationWorld.GetAdapters())if(adapter)admitted.Add(adapter);
    for(const auto& op:bridge->PendingOps)if(op.Comp){if(op.bAdd)admitted.Add(op.Comp);else admitted.Remove(op.Comp);}
    TArray<const void*> inventory;for(auto* adapter:admitted)inventory.Add(adapter);
    TArray<FPreparedPhysicalFilterRecord> actualFilters;FPreparedPhysicalQueryDomainPtr filters;
    if(!CapturePreparedPhysicalFilters(*world,admitted,actualFilters,reason)||
        !FPreparedPhysicalQueryDomain::Build(Generation,MoveTemp(actualFilters),inventory,filters))
    {if(reason.IsEmpty())reason=TEXT("Canonical physical filter domain unavailable");return false;}
    FPreparedNativeOwnerBindingsPtr nativeOwners;
    if(nativeInput&&!FPreparedNativeOwnerBindings::SelectGT(nativeInput,*world,Generation,inventory,nativeOwners,reason))return false;
    FPreparedWorldOrigin origin;origin.World=world;origin.Host=host;origin.Bridge=bridge;origin.Asset=&asset;origin.Generation=Generation;
    FPreparedGeometrySelectionPtr selection;
    if(!FPreparedGeometrySelection::TryCreate(EPreparedGeometryModel::RetainedReference,nominal,MoveTemp(reference),selection,reason))return false;
    FPreparedWorldSnapshotPtr snapshot=MakeShareable(new FPreparedWorldSnapshot(MoveTemp(selection),MoveTemp(filters),MoveTemp(bindings),origin,MoveTemp(nativeOwners),MoveTemp(externalLandscape)));
    FPreparedCanonicalStaticQueryServicePtr service;
    if(factory&&(!(*factory)(snapshot,service,reason)||!service||!service->MatchesPreparedSnapshot(snapshot)))
    {if(reason.IsEmpty())reason=TEXT("Canonical query service factory refused or mixed its snapshot");return false;}
    FPreparedNativeFrameConsumerPtr nativeConsumer;
    if(nativeFactory&&(!factory||!service||!(*nativeFactory)(snapshot,service,nativeConsumer,reason)||
        !nativeConsumer||!nativeConsumer->Matches(snapshot,service)))
    {if(reason.IsEmpty())reason=TEXT("Compiled native frame factory refused its retained snapshot/service");return false;}
    // A factory is synchronous but still an external Source callback. Recheck
    // all instance/filter/native-owner bindings before the single publication.
    if(factory)
    {
        TMap<uint64,TWeakObjectPtr<UPrimitiveComponent>> currentContacts;
        TArray<FPreparedSourceContactBinding> currentBindings;
        TArray<FPreparedPhysicalFilterRecord> currentFilters;
        TSet<ISpeedComponent*> currentAdapters;
        for(auto* adapter:bridge->SimulationWorld.GetAdapters())if(adapter)currentAdapters.Add(adapter);
        for(const auto& op:bridge->PendingOps)if(op.Comp){if(op.bAdd)currentAdapters.Add(op.Comp);else currentAdapters.Remove(op.Comp);}
        TArray<const void*> currentInventory;for(auto* adapter:currentAdapters)currentInventory.Add(adapter);
        if(!IsFor(*bridge)||!bridge->ValidatePreparedIntervalSources(asset,*nominal,currentContacts,reason,snapshot->GetExternalLandscape().Get())||
            !CapturePreparedReferenceBindings(*world,asset,currentContacts,currentBindings)||
            !SamePreparedReferenceBindings(currentBindings,snapshot->GetBindings())||
            !CapturePreparedPhysicalFilters(*world,currentAdapters,currentFilters,reason)||
            !snapshot->GetFilters()->Matches(MoveTemp(currentFilters),currentInventory)||
            (snapshot->GetNativeOwners()&&!snapshot->GetNativeOwners()->MatchesGT(nativeInput,*world,Generation,currentInventory))||
            !service->MatchesPreparedSnapshot(snapshot))
        {if(reason.IsEmpty())reason=TEXT("Canonical query factory changed publication inputs");return false;}
    }
    auto provider=MakeUnique<FPreparedIntervalStaticWorld>(snapshot,service,nativeConsumer);
    if(!IsFor(*bridge)||bridge->bCanonicalFrameActive||(bridge->StepSerial!=0&&!(NativeGenerationReplacement&&nativeFactory))||bridge->PreparedIntervalScene||
        bridge->SimulationWorld.GetStaticCollisionWorld())
    {reason=TEXT("Canonical prepared publication boundary changed");return false;}
    // One commit, no callback/yield: provider owns the SAME complete snapshot.
    // The old scene field is a nominal alias/one-shot guard, never a second model.
    bridge->PreparedIntervalAsset=const_cast<USpeedAnalyticCollisionAsset*>(&asset);
    bridge->PreparedIntervalScene=nominal;bridge->AnalyticSourceComponents=MoveTemp(contacts);
    bridge->SimulationWorld.SetStaticCollisionWorld(MoveTemp(provider));
    out=MoveTemp(snapshot);return true; // Old overloads retain null consumer and remain non-Ready.
}
FPreparedWorldSnapshotPtr FHostedGTStaticWorldLease::ReadCanonicalReferenceSnapshot(const USpeedAnalyticCollisionAsset& asset,FPreparedNativeOwnerInputPtr nativeInput) const
{return ReadCanonicalReferenceSnapshotAtReadiness(asset,MoveTemp(nativeInput),false);}
FPreparedWorldSnapshotPtr FHostedGTStaticWorldLease::ReadCanonicalReferenceSnapshotForExecution(const USpeedAnalyticCollisionAsset& asset,FPreparedNativeOwnerInputPtr nativeInput) const
{return ReadCanonicalReferenceSnapshotAtReadiness(asset,MoveTemp(nativeInput),true);}
FPreparedWorldSnapshotPtr FHostedGTStaticWorldLease::ReadCanonicalReferenceSnapshotAtReadiness(const USpeedAnalyticCollisionAsset& asset,FPreparedNativeOwnerInputPtr nativeInput,bool expectedReady) const
{
    auto* bridge=Bridge.Get();auto* host=Host.Get();auto* world=World.Get();
    if(!bridge||!host||!world||!IsFor(*bridge)||!bridge->bPreparedWorldResetComplete||
        !bridge->bPreparedIntervalRouteRequested||bridge->bCanonicalFrameActive||
        !bridge->PreparedIntervalScene||bridge->PreparedIntervalAsset.Get()!=&asset)return {};
    const auto* provider=bridge->SimulationWorld.GetStaticCollisionWorld();
    const auto snapshot=provider?provider->GetPreparedSnapshot():FPreparedWorldSnapshotPtr{};
    const auto service=provider?provider->GetPreparedQueryService():FPreparedCanonicalStaticQueryServicePtr{};
    if(service&&!service->MatchesPreparedSnapshot(snapshot))return {};
    if(!snapshot||!snapshot->GetReference()||snapshot->GetNominalScene()!=bridge->PreparedIntervalScene||
        snapshot->GetReference()->GetNominalScene()!=snapshot->GetNominalScene()||!snapshot->GetFilters()||
        snapshot->GetFilters()->GetGeneration()!=Generation||provider->IsReadyForPhysicsStep()!=expectedReady)return {};
    const auto& origin=snapshot->GetOrigin();
    if(origin.World!=world||origin.Host!=host||origin.Bridge!=bridge||origin.Asset!=&asset||origin.Generation!=Generation)return {};
    FScopeLock lock(&bridge->PendingCS);
    TMap<uint64,TWeakObjectPtr<UPrimitiveComponent>> contacts;FString reason;
    if(!bridge->ValidatePreparedIntervalSources(asset,*snapshot->GetNominalScene(),contacts,reason,snapshot->GetExternalLandscape().Get())||
        contacts.Num()!=bridge->AnalyticSourceComponents.Num())return {};
    for(const auto& row:contacts)
        if(!bridge->AnalyticSourceComponents.Contains(row.Key)||bridge->AnalyticSourceComponents.FindChecked(row.Key)!=row.Value)return {};
    TArray<FPreparedSourceContactBinding> bindings;
    if(!CapturePreparedReferenceBindings(*world,asset,contacts,bindings)||
        !SamePreparedReferenceBindings(bindings,snapshot->GetBindings())||
        bridge->SimulationWorld.GetStaticCollisionWorld()!=provider||provider->GetPreparedSnapshot()!=snapshot||
        provider->GetPreparedQueryService()!=service||!IsFor(*bridge))return {};
    if(snapshot->GetNativeOwners())
    {
        TSet<ISpeedComponent*> admitted;
        for(auto* adapter:bridge->SimulationWorld.GetAdapters())if(adapter)admitted.Add(adapter);
        for(const auto& op:bridge->PendingOps)if(op.Comp){if(op.bAdd)admitted.Add(op.Comp);else admitted.Remove(op.Comp);}
        TArray<const void*> inventory;for(auto* adapter:admitted)inventory.Add(adapter);
        if(!snapshot->GetNativeOwners()->MatchesGT(nativeInput,*world,Generation,inventory))return {};
    }
    else if(nativeInput)return {}; // An older snapshot cannot acquire a new native contract by readback.
    return snapshot; // Liveness/census only; caller reauthenticates full DA on GT.
}
}
