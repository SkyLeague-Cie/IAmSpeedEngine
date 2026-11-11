#pragma once
#include "CoreMinimal.h"
#include "IAmSpeed/Base/InheritedMotionPolicyValues91.h"
#include "UObject/WeakObjectPtr.h"
class ISpeedComponent;class UActorComponent;class USSubBody;class USphereSubBody;class UWorld;
namespace Speed::FrameParallel {struct FSubBodyKinematicInput;}
namespace Speed
{
// These describe source implementations tied to actual GT objects. Neither a
// class name nor possession of this record grants an integration permission.
enum class EPreparedNativeOwnerFamily:uint8 {Unqualified,Movement,Wheeled};
enum class EPreparedNativeOwnerImplementation:uint8 {Unqualified,InheritedNativeFreeOperations};
enum class EPreparedNativeChildImplementation:uint8 {Unqualified,InheritedNativeSphereSync};
struct FPreparedNativeBodyBinding
{
    const USSubBody* Body=nullptr;const void* Class=nullptr;FWeakObjectPtr Object;
    // Genuine GT UObject ID used by native ordering, not a stable physical
    // identity. Raw pointer and weak index/serial remain mandatory: IDs recycle.
    uint32 ObjectId=0;bool ObjectIdCaptured=false;
    FInheritedMotionPolicyValues91 InheritedMotionPolicy91;
    int32 Index=INDEX_NONE;EPreparedNativeChildImplementation Implementation=EPreparedNativeChildImplementation::Unqualified;
};
struct FPreparedNativeOwnerBinding
{
    const ISpeedComponent* Adapter=nullptr;const UActorComponent* NativeObject=nullptr;
    EPreparedNativeOwnerFamily NativeFamily=EPreparedNativeOwnerFamily::Unqualified;
    // Actual GT body census is independent of any operation qualification.
    bool BodyCensusCaptured=false;
    const void* Class=nullptr;FWeakObjectPtr Object;
    EPreparedNativeOwnerImplementation Implementation=EPreparedNativeOwnerImplementation::Unqualified;
    TArray<FPreparedNativeBodyBinding> Bodies;
};
class FPreparedNativeOwnerInput;
using FPreparedNativeOwnerInputPtr=TSharedPtr<const FPreparedNativeOwnerInput,ESPMode::ThreadSafe>;
class FPreparedNativeOwnerInput final
{
    friend class FPreparedNativeOwnerInputProducer;
    friend class FPreparedNativeOwnerBindings;
    const UWorld* World=nullptr;TArray<FPreparedNativeOwnerBinding> Rows;
    FPreparedNativeOwnerInput()=default;
};
// Generic structural producer, following ReferenceTransportProducer. A subclass
// declaration is not source authentication or an advance capability. The closed
// game producer must establish the actual-object/compiled-operation relation.
class FPreparedNativeOwnerInputProducer
{
protected:
    static IAMSPEED_API bool SealGT(const UWorld& World,TArray<FPreparedNativeOwnerBinding>&& Rows,
        FPreparedNativeOwnerInputPtr& Out,FString& Reason);
};
class FPreparedNativeOwnerBindings;
using FPreparedNativeOwnerBindingsPtr=TSharedPtr<const FPreparedNativeOwnerBindings,ESPMode::ThreadSafe>;
class IAMSPEED_API FPreparedNativeOwnerBindings final
{
    uint64 Generation=0;const UWorld* World=nullptr;TArray<FPreparedNativeOwnerBinding> Rows;
    FPreparedNativeOwnerBindings()=default;
public:
    // Called only under the existing GT publication lease and pending lock.
    // Unknown non-UObject/other-class adapters remain explicit unqualified
    // rows; they never disappear from the complete owner inventory.
    static bool SelectGT(const FPreparedNativeOwnerInputPtr& Input,const UWorld& World,uint64 Generation,
        const TArray<const void*>& Adapters,FPreparedNativeOwnerBindingsPtr& Out,FString& Reason);
    bool MatchesGT(const FPreparedNativeOwnerInputPtr& Input,const UWorld& World,uint64 Generation,
        const TArray<const void*>& Adapters) const;
    const FPreparedNativeOwnerBinding* Find(const ISpeedComponent* Adapter) const;
    TConstArrayView<FPreparedNativeOwnerBinding> GetRecordsForInspection() const {return Rows;}
    int32 Num() const {return Rows.Num();}
    uint64 GetGeneration() const {return Generation;}
    // A weak reference is never resolved here. The future execution scope must
    // also retain GT strong pins through exact park ACK/join and bind a new
    // advance purpose. This capture alone does not provide either condition.
    bool CaptureSphereInput(const ISpeedComponent* Owner,const USphereSubBody& Body,
        FrameParallel::FSubBodyKinematicInput& Out) const;
};
}
