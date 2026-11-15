#include "PreparedNativeOwnerBindings.h"
#include "IAmSpeed/Components/SpeedMovementComponent.h"
#include "IAmSpeed/Components/SpeedWheeledComponent.h"
#include "IAmSpeed/SubBodies/Solid/SphereSubBody.h"
#include "IAmSpeed/World/Simulation/SubBodyKinematicsTasks.h"
#include "IAmSpeed/Base/ExactRotatorQuaternionCache.h"
#include "Engine/World.h"
namespace Speed
{
namespace
{
const ISpeedComponent* ActualNativeInterfaceGT(const FPreparedNativeOwnerBinding& row)
{
    if(!IsInGameThread()||!row.NativeObject)return nullptr;
    if(row.NativeFamily==EPreparedNativeOwnerFamily::Movement)
    {
        const auto* typed=Cast<USpeedMovementComponent>(row.NativeObject);
        return typed?static_cast<const ISpeedComponent*>(typed):nullptr;
    }
    if(row.NativeFamily==EPreparedNativeOwnerFamily::Wheeled)
    {
        const auto* typed=Cast<USpeedWheeledComponent>(row.NativeObject);
        return typed?static_cast<const ISpeedComponent*>(typed):nullptr;
    }
    return nullptr;
}
bool SameRows(const FPreparedNativeOwnerBinding& a,const FPreparedNativeOwnerBinding& b)
{
    if(a.Adapter!=b.Adapter||a.NativeObject!=b.NativeObject||a.Class!=b.Class||a.Object!=b.Object||
        a.NativeFamily!=b.NativeFamily||a.BodyCensusCaptured!=b.BodyCensusCaptured||a.Implementation!=b.Implementation||a.Bodies.Num()!=b.Bodies.Num())return false;
    for(int32 i=0;i<a.Bodies.Num();++i)
    {
        const auto& x=a.Bodies[i];const auto& y=b.Bodies[i];
        if(x.Body!=y.Body||x.Class!=y.Class||x.Object!=y.Object||x.Index!=y.Index||x.Implementation!=y.Implementation||
            x.ObjectIdCaptured!=y.ObjectIdCaptured||x.ObjectId!=y.ObjectId||!SameInheritedMotionPolicy91(x.InheritedMotionPolicy91,y.InheritedMotionPolicy91))return false;
    }
    return true;
}
}
bool FPreparedNativeOwnerInputProducer::SealGT(const UWorld& world,TArray<FPreparedNativeOwnerBinding>&& rows,
    FPreparedNativeOwnerInputPtr& out,FString& reason)
{
    out.Reset();reason.Reset();
    if(!IsInGameThread()||world.bIsTearingDown||!world.HasBegunPlay())
    {reason=TEXT("Native owner structural seal requires begun World on GT");return false;}
    for(const auto& row:rows)
    {
        if(row.Implementation!=EPreparedNativeOwnerImplementation::Unqualified&&
            row.Implementation!=EPreparedNativeOwnerImplementation::InheritedNativeFreeOperations)
        {reason=TEXT("Unknown native owner operation declaration");return false;}
        for(const auto& body:row.Bodies)
        {
            if(!body.ObjectIdCaptured){reason=TEXT("Native child UObject ID was not captured on GT");return false;}
            if(body.Implementation!=EPreparedNativeChildImplementation::Unqualified&&
                body.Implementation!=EPreparedNativeChildImplementation::InheritedNativeSphereSync)
            {reason=TEXT("Unknown native child operation declaration");return false;}
        }
    }
    TSharedPtr<FPreparedNativeOwnerInput,ESPMode::ThreadSafe> input(new FPreparedNativeOwnerInput);
    input->World=&world;input->Rows=MoveTemp(rows);out=MoveTemp(input);return true;
}
bool FPreparedNativeOwnerBindings::SelectGT(const FPreparedNativeOwnerInputPtr& input,const UWorld& world,uint64 generation,
    const TArray<const void*>& adapters,FPreparedNativeOwnerBindingsPtr& out,FString& reason)
{
    out.Reset();reason.Reset();auto fail=[&](const TCHAR* text){reason=text;return false;};
    if(!IsInGameThread()||!input||input->World!=&world||!generation||world.bIsTearingDown||adapters.IsEmpty())
        return fail(TEXT("Native owner GT input/world/generation unavailable"));
    TSet<const ISpeedComponent*> seen;TSet<const void*> nativeObjects;
    // Validate EVERY captured candidate before selecting the effective subset.
    for(const auto& row:input->Rows)
    {
        const auto* object=row.NativeObject;
        if(!row.Adapter||!object||row.Object.Get()!=object||!IsValid(object)||object->IsTemplate()||!object->IsRegistered()||
            object->GetWorld()!=&world||object->GetClass()!=row.Class||ActualNativeInterfaceGT(row)!=row.Adapter||
            seen.Contains(row.Adapter)||nativeObjects.Contains(object))return fail(TEXT("Native owner object/interface association changed"));
        seen.Add(row.Adapter);nativeObjects.Add(object);
        if(row.BodyCensusCaptured)
        {
            const auto& bodies=row.Adapter->GetSubBodies();if(bodies.Num()!=row.Bodies.Num())return fail(TEXT("Native owner body inventory changed"));
            TSet<const USSubBody*> bodySeen;
            for(int32 i=0;i<bodies.Num();++i)
            {
                const auto& b=row.Bodies[i];const auto* body=bodies[i];
                if(row.Implementation==EPreparedNativeOwnerImplementation::Unqualified&&b.Implementation!=EPreparedNativeChildImplementation::Unqualified)
                    return fail(TEXT("Unqualified owner cannot claim child operations"));
                if(!body||!IsValid(body)||body->IsTemplate()||!body->IsRegistered()||body->GetWorld()!=&world||
                    b.Body!=body||b.Object.Get()!=body||b.Class!=body->GetClass()||b.Index!=i||
                    !b.ObjectIdCaptured||b.ObjectId!=body->GetUniqueID()||
                    body->GetParentComponent()!=row.Adapter||bodySeen.Contains(body)||
                    !SameInheritedMotionPolicy91(b.InheritedMotionPolicy91,body->GetInheritedMotionPolicyValues91()))return fail(TEXT("Native child GT identity/order changed"));
                bodySeen.Add(body);
            }
        }
        else if(!row.Bodies.IsEmpty()||row.Implementation!=EPreparedNativeOwnerImplementation::Unqualified)
            return fail(TEXT("Native operations require an actual complete child census"));
    }
    TSharedPtr<FPreparedNativeOwnerBindings,ESPMode::ThreadSafe> result(new FPreparedNativeOwnerBindings);
    result->Generation=generation;result->World=&world;TSet<const void*> inventory;
    for(const void* raw:adapters)
    {
        if(!raw||inventory.Contains(raw))return fail(TEXT("Native effective owner inventory is null or duplicated"));inventory.Add(raw);
        const auto* owner=static_cast<const ISpeedComponent*>(raw);
        const auto* row=input->Rows.FindByPredicate([&](const auto& v){return v.Adapter==owner;});
        if(row)result->Rows.Add(*row);
        else {auto& unknown=result->Rows.AddDefaulted_GetRef();unknown.Adapter=owner;}
    }
    out=MoveTemp(result);return true;
}
const FPreparedNativeOwnerBinding* FPreparedNativeOwnerBindings::Find(const ISpeedComponent* owner) const
{return Rows.FindByPredicate([&](const auto& v){return v.Adapter==owner;});}
bool FPreparedNativeOwnerBindings::MatchesGT(const FPreparedNativeOwnerInputPtr& input,const UWorld& world,uint64 generation,
    const TArray<const void*>& adapters) const
{
    if(!IsInGameThread()||World!=&world||Generation!=generation)return false;
    FPreparedNativeOwnerBindingsPtr fresh;FString reason;
    if(!SelectGT(input,world,generation,adapters,fresh,reason)||fresh->Rows.Num()!=Rows.Num())return false;
    for(const auto& row:Rows){const auto* other=fresh->Find(row.Adapter);if(!other||!SameRows(row,*other))return false;}
    return true;
}
bool FPreparedNativeOwnerBindings::CaptureSphereInput(const ISpeedComponent* owner,const USphereSubBody& body,
    FrameParallel::FSubBodyKinematicInput& out) const
{
    out={};const auto* row=Find(owner);
    if(!owner||!row||row->Implementation==EPreparedNativeOwnerImplementation::Unqualified||!row->NativeObject||
        row->NativeFamily!=EPreparedNativeOwnerFamily::Movement||!row->BodyCensusCaptured||row->NativeObject->GetClass()!=row->Class||
        body.GetWorld()!=World||body.GetParentComponent()!=owner)return false;
    const auto* b=row->Bodies.FindByPredicate([&](const auto& v){return v.Body==&body;});
    if(!b||b->Implementation==EPreparedNativeChildImplementation::Unqualified||b->Class!=body.GetClass()||
        !owner->GetSubBodies().IsValidIndex(b->Index)||owner->GetSubBodies()[b->Index]!=&body)return false;
    FrameParallel::FSubBodyKinematicInput input;input.RelativeLocation=body.GetRelativeLocation();
    static thread_local FExactRotatorQuaternionCache rotationCache;
    input.RelativeRotation=rotationCache.Get(body.GetRelativeRotation());input.bWheel=false;out=input;return true;
}
}
