#include "PreparedNativeFrameConsumer.h"
#include "PreparedPhysicalExecution.h"
#include "IAmSpeed/World/Simulation/SubBodyKinematicsTasks.h"
#include "IAmSpeed/World/Analytic/AnalyticNumericContext.h"
#include "PreparedCanonicalStaticQueryService.h"
#include "IAmSpeed/Components/ISpeedComponent.h"
#include "IAmSpeed/SubBodies/SSubBody.h"
#include "IAmSpeed/SubBodies/Sensor/SensorSubBody.h"
#include "HAL/IConsoleManager.h"
#include <stdexcept>
#if !UE_BUILD_SHIPPING
#include "HAL/PlatformTime.h"
namespace RoofPostProfile { extern thread_local uint64* Current; }
#endif
namespace Speed
{
namespace
{
TAutoConsoleVariable<int32> NativeBatchedResponseTargetCensus(
    TEXT("p.IAmSpeed.AnalyticWorld.NativeBatchedResponseTargetCensus"),0,
    TEXT("Default OFF. Validate each actual response token with the exact retained index under one live census bracket."));
[[noreturn]] void RefuseNativeDispatch(const char* why) {throw std::runtime_error(why);}
// The retained census is immutable. Only exact weak index/serial identity is
// inspected here; do not resolve UObject validity or equate stale references.
bool NativeResponseTargetsInCensus(const FWeakObjectPtr& bodyToken,const FWeakObjectPtr& componentToken,
    TConstArrayView<FPreparedNativeOwnerBinding> owners,TConstArrayView<FPreparedSourceContactBinding> contacts)
{
    bool bodyFound=bodyToken.IsExplicitlyNull(),componentFound=componentToken.IsExplicitlyNull();
    if(bodyFound&&componentFound)return true;
    for(const auto& owner:owners)for(const auto& body:owner.Bodies)
    {
        bodyFound|=bodyToken.HasSameIndexAndSerialNumber(body.Object);
        componentFound|=componentToken.HasSameIndexAndSerialNumber(body.Object);
        if(bodyFound&&componentFound)return true;
    }
    // Contact bindings cannot satisfy a SubBody target.
    if(!bodyFound)return false;
    for(const auto& source:contacts)
        if(componentToken.HasSameIndexAndSerialNumber(source.Contact))return true;
    return false;
}
struct FExactWeakTokenKeyFuncs : DefaultKeyFuncs<FWeakObjectPtr>
{
    static bool Matches(KeyInitType a,KeyInitType b)
    {return a.HasSameIndexAndSerialNumber(b);}
};

// One immutable census per calling thread, without retaining its snapshot or
// UObjects. The shared weak control prevents an expired address from aliasing a
// newly allocated snapshot. IsValid below is TWeakPtr, never FWeakObjectPtr.
class FNativeResponseTargetCensusIndex final
{
    TWeakPtr<const void,ESPMode::ThreadSafe> Identity;
    const void* Address=nullptr;
    TSet<FWeakObjectPtr,FExactWeakTokenKeyFuncs> Bodies,Components;
public:
    template<class TObject>
    void Bind(const TSharedPtr<TObject,ESPMode::ThreadSafe>& identity,
        TConstArrayView<FPreparedNativeOwnerBinding> owners,TConstArrayView<FPreparedSourceContactBinding> contacts)
    {
        if(identity.IsValid()&&identity.Get()==Address&&Identity.IsValid())return;
        Identity.Reset();Address=nullptr;Bodies.Empty();Components.Empty();
        if(!identity.IsValid())return;
        for(const auto& owner:owners)for(const auto& body:owner.Bodies)
        {
            if(body.Object.IsExplicitlyNull())continue;
            Bodies.Add(body.Object);Components.Add(body.Object);
        }
        for(const auto& contact:contacts)
            if(!contact.Contact.IsExplicitlyNull())Components.Add(contact.Contact);
        // Publish the cache identity only after both complete sets exist.
        Address=identity.Get();Identity=identity;
    }
    bool ContainsComponent(const FWeakObjectPtr& component) const
    {return component.IsExplicitlyNull()||Components.Contains(component);}
    bool ContainsTargets(const FWeakObjectPtr& body,const FWeakObjectPtr& component) const
    {return (body.IsExplicitlyNull()||Bodies.Contains(body))&&ContainsComponent(component);}
};

FNativeResponseTargetCensusIndex& NativeResponseTargetIndex()
{
    static thread_local FNativeResponseTargetCensusIndex index;
    return index;
}

}
FPreparedNativeFrameConsumer::FPreparedNativeFrameConsumer(FPreparedWorldSnapshotPtr snapshot,
    TSharedPtr<const FPreparedCanonicalStaticQueryService,ESPMode::ThreadSafe> service,
    TSharedPtr<const FNativeFrameRetentionToken,ESPMode::ThreadSafe> retention,TArray<FNativeFrameOwnerDispatch>&& owners)
    :Snapshot(MoveTemp(snapshot)),Service(MoveTemp(service)),Retention(MoveTemp(retention)),Owners(MoveTemp(owners))
{
    if(!Matches(Snapshot,Service))RefuseNativeDispatch("Incomplete compiled native frame dispatch contract");
    TSet<const ISpeedComponent*> uniqueOwners;TSet<const USSubBody*> uniqueBodies;
    for(const auto& route:Owners)
    {
        const auto* binding=Snapshot->GetNativeOwners()->Find(route.Owner);
        if(!route.Owner||!binding||route.Object!=binding->NativeObject||uniqueOwners.Contains(route.Owner)||
            !route.Prepare||!route.Sync||!route.Integrate||!route.Post||route.Bodies.Num()!=binding->Bodies.Num())
            RefuseNativeDispatch("Native frame owner dispatch does not cover actual GT census");
        uniqueOwners.Add(route.Owner);
        for(int32 i=0;i<route.Bodies.Num();++i)
        {
            const auto& body=route.Bodies[i];
            if(!body.Body||binding->Bodies[i].Body!=body.Body||uniqueBodies.Contains(body.Body)||
                !body.SetFuture||!body.Accept||!body.Resolve||!body.Post)
                RefuseNativeDispatch("Native frame body dispatch does not cover actual GT census");
            uniqueBodies.Add(body.Body);
        }
    }
}
bool FPreparedNativeFrameConsumer::Matches(const FPreparedWorldSnapshotPtr& snapshot,
    const TSharedPtr<const FPreparedCanonicalStaticQueryService,ESPMode::ThreadSafe>& service) const noexcept
{
    return Snapshot&&Snapshot==snapshot&&Service&&Service==service&&Service->MatchesPreparedSnapshot(snapshot)&&
        Retention&&Retention->IsRetained()&&Snapshot->GetNativeOwners()&&
        Snapshot->GetNativeOwners()->Num()==Owners.Num()&&!Owners.IsEmpty()&&
        Snapshot->GetNativeOwners()->GetGeneration()==Snapshot->GetOrigin().Generation;
}
bool FPreparedNativeFrameConsumer::IsContinuousStreamArmedFor(const FSimulationWorker* worker) const noexcept
{
    return worker&&ContinuousEnabled.load(std::memory_order_acquire)&&StreamWorker==worker&&
        Retention&&Retention->IsRetained();
}
bool FPreparedNativeFrameConsumer::ContainsOwner(const ISpeedComponent* owner) const noexcept
{for(const auto& row:Owners)if(row.Owner==owner)return owner!=nullptr;return false;}
bool FPreparedNativeFrameConsumer::ContainsBody(const USSubBody* body) const noexcept
{for(const auto& owner:Owners)for(const auto& row:owner.Bodies)if(row.Body==body)return body!=nullptr;return false;}
bool FPreparedNativeFrameConsumer::ContainsRetainedComponentToken(const FWeakObjectPtr& token) const noexcept
{
    if(!Matches(Snapshot,Service))return false;if(token.IsExplicitlyNull())return true;
    if(Analytic::Certified::PlanarFamilyTimeShortcutControl::RuntimeEnabled())
    {
        auto& index=NativeResponseTargetIndex();
        index.Bind(Snapshot,Snapshot->GetNativeOwners()->GetRecordsForInspection(),Snapshot->GetAllContactBindings());
        return index.ContainsComponent(token);
    }
    for(const auto& owner:Snapshot->GetNativeOwners()->GetRecordsForInspection())for(const auto& body:owner.Bodies)
        if(token.HasSameIndexAndSerialNumber(body.Object))return true;
    for(const auto& source:Snapshot->GetAllContactBindings())if(token.HasSameIndexAndSerialNumber(source.Contact))return true;
    return false;
}
const FNativeFrameOwnerDispatch& FPreparedNativeFrameConsumer::RequireOwner(const ISpeedComponent* owner) const
{
    if(!Matches(Snapshot,Service))RefuseNativeDispatch("Native frame object retention revoked");
    for(const auto& row:Owners)if(row.Owner==owner&&owner)return row;
    RefuseNativeDispatch("Native frame dispatch owner outside retained census");
}
const FNativeFrameBodyDispatch& FPreparedNativeFrameConsumer::RequireBody(const USSubBody* body) const
{
    if(!Matches(Snapshot,Service))RefuseNativeDispatch("Native frame object retention revoked");
    for(const auto& owner:Owners)for(const auto& row:owner.Bodies)if(row.Body==body&&body)return row;
    RefuseNativeDispatch("Native frame dispatch body outside retained census");
}
ENativeFrameBodyRoute FPreparedNativeFrameConsumer::GetBodyRoute(const USSubBody* body) const {return RequireBody(body).Route;}
bool FPreparedNativeFrameConsumer::ReadCurrentBodyLocalKinematicValues(const USSubBody* body,
    FrameParallel::FSubBodyKinematicInput& output) const
{
    output={};PreparedExecution::FStepIdentity step;FPreparedWorldSnapshotPtr current;
    if(!PreparedExecution::ReadCurrentStep(step)||!PreparedExecution::IsCurrentWorker(step.Worker)||
        !PreparedExecution::ReadCurrentSnapshot(current)||current!=Snapshot||!Snapshot->GetNativeOwners()||
        step.InputGeneration!=Snapshot->GetNativeOwners()->GetGeneration()||!step.Bridge||
        !PreparedExecution::FCanonicalFramePreflight::HasUnchangedRawPreparationCensus(*step.Bridge,*Snapshot,step.StepSerial))return false;
    const auto numeric=Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
    const auto& dispatch=RequireBody(body);if(!dispatch.ReadLocalKinematicValues)return false;
    FrameParallel::FSubBodyKinematicInput value;
    if(!dispatch.ReadLocalKinematicValues(*body,value)||value.RelativeLocation.ContainsNaN()||
        value.RelativeRotation.ContainsNaN()||!value.RelativeRotation.IsNormalized()||
        !FMath::IsFinite(value.WheelSpringDisplacement))return false;
    PreparedExecution::FStepIdentity after;FPreparedWorldSnapshotPtr still;
    if(!PreparedExecution::ReadCurrentStep(after)||!PreparedExecution::ReadCurrentSnapshot(still)||still!=Snapshot||
        after.Worker!=step.Worker||after.Bridge!=step.Bridge||after.InputGeneration!=step.InputGeneration||after.Frame!=step.Frame||
        after.StepSerial!=step.StepSerial||after.QuerySerial!=step.QuerySerial||after.QueryEpoch!=step.QueryEpoch||
        after.Lease!=step.Lease||after.WindowSerial!=step.WindowSerial||after.WindowNonce!=step.WindowNonce||after.ThreadId!=step.ThreadId||
        !PreparedExecution::FCanonicalFramePreflight::HasUnchangedRawPreparationCensus(*after.Bridge,*Snapshot,after.StepSerial)||
        &RequireBody(body)!=&dispatch||Analytic::RequireCurrentAnalyticNumericContext(
            Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush).ControlFingerprint()!=numeric.ControlFingerprint())return false;
    output=value;return true;
}

void FPreparedNativeFrameConsumer::PrepareOwner(ISpeedComponent& owner,const FCanonicalFrameContext& context) const
{RequireOwner(&owner).Prepare(owner,context);}
void FPreparedNativeFrameConsumer::SynchronizeOwner(ISpeedComponent& owner) const {RequireOwner(&owner).Sync(owner);}
void FPreparedNativeFrameConsumer::IntegrateOwner(ISpeedComponent& owner,float seconds) const {RequireOwner(&owner).Integrate(owner,seconds);}
void FPreparedNativeFrameConsumer::PostOwner(ISpeedComponent& owner,float seconds) const
{
#if !UE_BUILD_SHIPPING
    uint64* roofPost=::RoofPostProfile::Current;
    if(roofPost&&roofPost[2])roofPost=nullptr;
    if(roofPost)roofPost[2]=FPlatformTime::Cycles64();
#endif
    RequireResponseTargetsInCensus();
#if !UE_BUILD_SHIPPING
    if(roofPost)roofPost[3]=FPlatformTime::Cycles64();
#endif
    RequireOwner(&owner).Post(owner,seconds);
#if !UE_BUILD_SHIPPING
    if(roofPost)roofPost[4]=FPlatformTime::Cycles64();
#endif
}
void FPreparedNativeFrameConsumer::SetFutureHit(USSubBody& body,const SHitResult& hit) const
{RequireHitTargets(hit);RequireResponseTargetsInCensus();RequireBody(&body).SetFuture(body,hit);}
void FPreparedNativeFrameConsumer::AcceptHit(USSubBody& body) const
{RequireResponseTargetsInCensus();RequireBody(&body).Accept(body);}
void FPreparedNativeFrameConsumer::ResolveHit(USSubBody& body,float seconds,float simTime) const
{RequireResponseTargetsInCensus();RequireBody(&body).Resolve(body,seconds,simTime);}
void FPreparedNativeFrameConsumer::PostBody(USSubBody& body) const {RequireResponseTargetsInCensus();RequireBody(&body).Post(body);}
void FPreparedNativeFrameConsumer::RequireHitTargets(const SHitResult& hit) const
{
    if(!Matches(Snapshot,Service))RefuseNativeDispatch("Native response retention revoked");
    const FWeakObjectPtr bodyToken=static_cast<FWeakObjectPtr>(hit.SubBody);
    const FWeakObjectPtr componentToken=static_cast<FWeakObjectPtr>(hit.Component);
    if(bodyToken.IsExplicitlyNull()&&componentToken.IsExplicitlyNull())return;
    if(Analytic::Certified::PlanarFamilyTimeShortcutControl::RuntimeEnabled())
    {
        auto& index=NativeResponseTargetIndex();
        index.Bind(Snapshot,Snapshot->GetNativeOwners()->GetRecordsForInspection(),Snapshot->GetAllContactBindings());
        if(!index.ContainsTargets(bodyToken,componentToken))
            RefuseNativeDispatch("Native response target outside retained body/contact census");
        return;
    }
    if(!NativeResponseTargetsInCensus(bodyToken,componentToken,
        Snapshot->GetNativeOwners()->GetRecordsForInspection(),Snapshot->GetAllContactBindings()))
        RefuseNativeDispatch("Native response target outside retained body/contact census");
}
void FPreparedNativeFrameConsumer::RequireResponseTargetsInCensus() const
{
    if(!Matches(Snapshot,Service))RefuseNativeDispatch("Native response retention revoked");
    const bool batched=NativeBatchedResponseTargetCensus.GetValueOnAnyThread()==1;
    const auto visit=[this,batched](auto&& requireHit,auto&& requireSensor)
    {
        for(const auto& owner:Owners)for(const auto& row:owner.Bodies)
        {
            const auto& current=row.Body->GetHit();const auto& future=row.Body->GetFutureHit();
            requireHit(current);requireHit(future);
            // The GT factory assigns non-Sensor routes only to its exact
            // Sphere, Box and Wheel classes. Extensions retain the live cast.
            if(!batched||row.Route==ENativeFrameBodyRoute::Sensor)
                if(const auto* sensor=Cast<USensorSubBody>(row.Body))sensor->VisitNativeOverlapTargetTokens(requireSensor);
        }
    };
    if(!batched)
    {
        visit([this](const SHitResult& hit){RequireHitTargets(hit);},[this](const FWeakObjectPtr& token)
        {if(!ContainsRetainedComponentToken(token))RefuseNativeDispatch("Native Sensor overlap target outside retained census");});
        return;
    }

    // The canonical worker owns the hit/sensor values during this pure walk.
    // Snapshot/service/owner identities are immutable; retention only revokes
    // and GT roots remain until park or join. Recheck before any caller can
    // dispatch. An explicit null token never needs an index or a live probe.
    FNativeResponseTargetCensusIndex* index=nullptr;
    const auto targetIndex=[&]() -> const FNativeResponseTargetCensusIndex&
    {
        if(!index)
        {
            index=&NativeResponseTargetIndex();
            index->Bind(Snapshot,Snapshot->GetNativeOwners()->GetRecordsForInspection(),Snapshot->GetAllContactBindings());
        }
        return *index;
    };
    visit([&](const SHitResult& hit)
    {
        const FWeakObjectPtr body=static_cast<FWeakObjectPtr>(hit.SubBody);
        const FWeakObjectPtr component=static_cast<FWeakObjectPtr>(hit.Component);
        if(body.IsExplicitlyNull()&&component.IsExplicitlyNull())return;
        if(!targetIndex().ContainsTargets(body,component))
            RefuseNativeDispatch("Native response target outside retained body/contact census");
    },[&](const FWeakObjectPtr& token)
    {
        if(!token.IsExplicitlyNull()&&!targetIndex().ContainsComponent(token))
            RefuseNativeDispatch("Native Sensor overlap target outside retained census");
    });
    if(!Matches(Snapshot,Service))RefuseNativeDispatch("Native response retention revoked");
}
}

#if WITH_DEV_AUTOMATION_TESTS
#include "Misc/AutomationTest.h"
#include "UObject/StrongObjectPtr.h"
#include "Components/SceneComponent.h"
#include "UObject/UObjectGlobals.h"

namespace Speed
{
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FNativeResponseTargetCensusEarlyExitTest,
    "IAmSpeed.AnalyticWorld.NativeSolid.ResponseTargetCensusEarlyExit",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FNativeResponseTargetCensusEarlyExitTest::RunTest(const FString&)
{
    if(!TestTrue(TEXT("transient token fixture only created on GT"),IsInGameThread()))return false;
    const TStrongObjectPtr<USceneComponent> bodyA(NewObject<USceneComponent>()),bodyB(NewObject<USceneComponent>()),
        contactA(NewObject<USceneComponent>()),contactZ(NewObject<USceneComponent>()),
        staleRetained(NewObject<USceneComponent>()),staleForeign(NewObject<USceneComponent>());
    const FWeakObjectPtr a(bodyA.Get()),b(bodyB.Get()),c(contactA.Get()),z(contactZ.Get()),
        retained(staleRetained.Get()),foreign(staleForeign.Get()),none;
    // Owned test objects only. Retention membership is identity, not IsValid().
    staleRetained->MarkAsGarbage();staleForeign->MarkAsGarbage();
    TestFalse(TEXT("retained stale fixture is not a live object"),retained.IsValid());
    TestFalse(TEXT("foreign stale fixture is not a live object"),foreign.IsValid());
    TestFalse(TEXT("distinct stale tokens retain distinct index/serial identity"),retained.HasSameIndexAndSerialNumber(foreign));
    TArray<FPreparedNativeOwnerBinding> owners;owners.SetNum(2);
    owners[0].Bodies.AddDefaulted_GetRef().Object=a;
    owners[1].Bodies.AddDefaulted_GetRef().Object=b;
    TArray<FPreparedSourceContactBinding> contacts;contacts.SetNum(8);
    for(auto& contact:contacts)contact.Contact=c;
    contacts[6].Contact=retained;contacts.Last().Contact=z;
    const auto contains=[&](const FWeakObjectPtr& body,const FWeakObjectPtr& component)
    {return NativeResponseTargetsInCensus(body,component,owners,contacts);};
    TestTrue(TEXT("both explicit null targets accepted with empty census"),NativeResponseTargetsInCensus(none,none,{},{}));
    TestTrue(TEXT("first body target and null component accepted"),contains(a,none));
    TestTrue(TEXT("two targets in different owners accepted"),contains(a,b));
    TestTrue(TEXT("last contact binding remains reachable"),contains(b,z));
    TestFalse(TEXT("contact-only token is not a SubBody"),contains(c,c));
    TestTrue(TEXT("exact retained stale contact identity remains accepted"),contains(none,retained));
    TestFalse(TEXT("foreign stale contact never aliases another stale token"),contains(none,foreign));
    TestFalse(TEXT("foreign body rejected even with admitted contact"),contains(foreign,c));
    const FWeakObjectPtr tokens[]{none,a,b,c,z,retained,foreign};
    for(const auto& body:tokens)for(const auto& component:tokens)
    {
        // Independent membership oracle: the body set is {a,b}; the contact
        // set is {a,b,c,z,retained}. Null is the sole implicit member.
        const bool bodyExpected=body.IsExplicitlyNull()||body.HasSameIndexAndSerialNumber(a)||body.HasSameIndexAndSerialNumber(b);
        const bool componentExpected=component.IsExplicitlyNull()||component.HasSameIndexAndSerialNumber(a)||
            component.HasSameIndexAndSerialNumber(b)||component.HasSameIndexAndSerialNumber(c)||
            component.HasSameIndexAndSerialNumber(z)||component.HasSameIndexAndSerialNumber(retained);
        TestEqual(TEXT("complete target-pair membership table"),contains(body,component),bodyExpected&&componentExpected);
    }
    owners[1].Bodies[0].Object=a;
    TestFalse(TEXT("new invocation reads changed body membership"),contains(b,z));
    contacts.Last().Contact=c;
    TestFalse(TEXT("new invocation reads changed contact membership"),contains(a,z));
    return !HasAnyErrors();
}
}
#endif

#if WITH_DEV_AUTOMATION_TESTS
namespace Speed
{
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FNativeResponseTargetCensusIndexTest,
    "IAmSpeed.AnalyticWorld.NativeSolid.ResponseTargetCensusIndex",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FNativeResponseTargetCensusIndexTest::RunTest(const FString&)
{
    if(!TestTrue(TEXT("transient token fixture only created on GT"),IsInGameThread()))return false;
    const TStrongObjectPtr<USceneComponent> bodyA(NewObject<USceneComponent>()),bodyB(NewObject<USceneComponent>()),
        contact(NewObject<USceneComponent>()),foreign(NewObject<USceneComponent>());
    const FWeakObjectPtr a(bodyA.Get()),b(bodyB.Get()),c(contact.Get()),f(foreign.Get()),none;
    contact->MarkAsGarbage();foreign->MarkAsGarbage();
    TestFalse(TEXT("exact set comparator does not equate different stale tokens"),FExactWeakTokenKeyFuncs::Matches(c,f));
    TestFalse(TEXT("exact set comparator does not equate null and stale"),FExactWeakTokenKeyFuncs::Matches(none,c));
    TArray<FPreparedNativeOwnerBinding> owners;owners.SetNum(2);
    owners[0].Bodies.AddDefaulted_GetRef().Object=a;owners[1].Bodies.AddDefaulted_GetRef().Object=b;
    TArray<FPreparedSourceContactBinding> contacts;contacts.SetNum(2992);
    for(auto& row:contacts)row.Contact=c;
    FNativeResponseTargetCensusIndex index;
    TSharedPtr<const int,ESPMode::ThreadSafe> first=MakeShared<int,ESPMode::ThreadSafe>(1);
    const TWeakPtr<const int,ESPMode::ThreadSafe> firstWeak=first;
    index.Bind(first,owners,contacts);
    const FWeakObjectPtr tokens[]{none,a,b,c,f};
    for(const auto& body:tokens)for(const auto& component:tokens)
    {
        const bool expected=(body.IsExplicitlyNull()||body.HasSameIndexAndSerialNumber(a)||body.HasSameIndexAndSerialNumber(b))&&
            (component.IsExplicitlyNull()||component.HasSameIndexAndSerialNumber(a)||
                component.HasSameIndexAndSerialNumber(b)||component.HasSameIndexAndSerialNumber(c));
        TestEqual(TEXT("indexed membership matches complete body/contact sets"),index.ContainsTargets(body,component),expected);
        TestEqual(TEXT("indexed membership matches exact scan"),index.ContainsTargets(body,component),
            NativeResponseTargetsInCensus(body,component,owners,contacts));
    }
    TestTrue(TEXT("indexed sensor/contact-only route admits retained stale identity"),index.ContainsComponent(c));
    TestFalse(TEXT("indexed sensor/contact-only route refuses foreign stale identity"),index.ContainsComponent(f));
    first.Reset();TestFalse(TEXT("index does not keep prior snapshot identity alive"),firstWeak.IsValid());
    TSharedPtr<const int,ESPMode::ThreadSafe> second=MakeShared<int,ESPMode::ThreadSafe>(2);
    const TWeakPtr<const int,ESPMode::ThreadSafe> secondWeak=second;
    owners[0].Bodies[0].Object=b;owners.SetNum(1);contacts.SetNum(1);contacts[0].Contact=f;
    index.Bind(second,owners,contacts);
    TestTrue(TEXT("new identity admits its own complete census"),index.ContainsTargets(b,f));
    TestFalse(TEXT("new identity drops old body membership"),index.ContainsTargets(a,f));
    TestFalse(TEXT("new identity drops old contact membership"),index.ContainsComponent(c));
    second.Reset();TestFalse(TEXT("replacement identity is also weakly retained"),secondWeak.IsValid());
    TSharedPtr<const int,ESPMode::ThreadSafe> missing;
    index.Bind(missing,{},{});
    TestFalse(TEXT("empty binding clears the prior census"),index.ContainsComponent(f));
    TestTrue(TEXT("explicit null remains the only implicit member"),index.ContainsTargets(none,none));
    return !HasAnyErrors();
}
}
#endif
