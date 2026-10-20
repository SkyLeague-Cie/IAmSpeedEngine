#pragma once
#include "CoreMinimal.h"
#if !UE_BUILD_SHIPPING
#include "FramePointerWitness.h"
class USpeedWorldSubsystem;
namespace Speed::Input::V2 { struct FRegistryFrame; }
namespace Speed
{
// Correctness-only observer. Disabled by default, never used for timing runs.
// Native completion is deliberately supplied later by the process controller.
struct FNativeFrameWitnessCaseIdentity
{
 FString RunUUID;uint64 DriverKey=0,Epoch=0,WorldGeneration=0,CanonicalFrame=0;
 int32 NativeCaseId=INDEX_NONE,BatchIndex=INDEX_NONE,Mode=0;
 bool bConfigured=false;
};
class IAMSPEED_API FNativeFrameWitnessRecorder
{
public:
 static TUniquePtr<FNativeFrameWitnessRecorder> Begin(uint64 Frame,float Delta,
   const Input::V2::FRegistryFrame* Installed,USpeedWorldSubsystem& World,
   const FNativeFrameWitnessCaseIdentity& Case,bool bNoSessionsAdmitted);
 static FNativeFrameWitnessRecorder* Current();
 ~FNativeFrameWitnessRecorder();
 const FFramePointerRegistry& PointerRegistry()const;
 bool ExtendPointerRegistry(TFunctionRef<bool(FFramePointerRegistry&)> Declare);
 uint64 CanonicalFrame()const;
 const FString& OutputDirectory()const;
 uint64 ReceiptCursor(uint64 Actor)const;
 void RecordActor(uint64 Actor,uint64 LocalFrame,const TArray<uint8>& RawMechanics,
   const TArray<uint8>& CanonicalMechanics,const TArray<uint8>& Ledger,
   const TArray<uint8>& AppliedEffects,const TArray<FFramePointerObservation>& Pointers,
   uint64 NextReceipt,bool bBoundary,bool bFields,bool bPointers,bool bLedger,bool bEffects);
 void Finish(const Input::V2::FRegistryFrame* Completed,
   const TArray<uint8>& SnapshotPayload,int32 ResolvedEventCount,
   const TArray<uint8>& CameraBytes,bool bCameraComplete,bool bNoSessionsAdmitted);
private:
 struct FImpl;TUniquePtr<FImpl> Impl;
 explicit FNativeFrameWitnessRecorder(TUniquePtr<FImpl>&& In);
};
}
#endif
