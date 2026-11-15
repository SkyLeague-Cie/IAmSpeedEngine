#pragma once
#include "PreparedFirstRsNonImpactResponse.h"
#include "PreparedRsPrefixAdvanceSource.h"

namespace Speed {struct FNativePrepareGTFields;}
namespace Speed::PreparationPrefix {class FClosedSourceProducer;}
namespace Speed::PreparedExecution
{
class FFirstRsResponseData;
using FFirstRsResponseDataPtr=TSharedPtr<const FFirstRsResponseData,ESPMode::ThreadSafe>;

// Immutable DATA identity, never a live query or physical capability. Its raw
// payload and prediction have BOTH hit annotations null, preventing a cycle.
// The private FStepScope separately mints one ResponseSerial and three stages.
class FFirstRsResponseData final
{
    friend class Speed::PreparationPrefix::FClosedSourceProducer;
    friend class FStepScope;
    FNativeSphereRequestPtr Origin;
    FRsPrefixAdvanceSourcePtr AdvanceSource;
    FFirstArrivalRsHitValues Values;
    FFirstRsNonImpactResponseExpectation Physical;
    const ISpeedComponent* Owner=nullptr;
    const USphereSubBody* Body=nullptr;
    const UClass* OwnerClass=nullptr;
    const UClass* BodyClass=nullptr;
    uint64 AdvanceSerial=0,NumericControl=0;
    float Seconds=0.0f,ActualTimePassed=0.0f,ActualResponseSimTime=0.0f;
    // p8-only pure proposition, sealed before response or sync effects.
    bool HasSynchronizationExpectation=false;
    FKinematicState BodyAfterSynchronization;
    FVector SynchronizationRelativeLocation=FVector::ZeroVector;
    FRotator SynchronizationRelativeEuler=FRotator::ZeroRotator;
    FFirstRsResponseData()=default;
public:
    const FStepIdentity& GetOriginIdentity() const {return Origin->GetStep();}
    uint64 GetAdvanceSerial() const {return AdvanceSerial;}
    uint64 GetNumericControl() const {return NumericControl;}
    const void* GetOriginAddress() const {return Origin.Get();}
    const void* GetAdvanceSourceAddress() const {return AdvanceSource.Get();}
    const void* GetArrivalAddress() const {return Values.Quality?Values.Quality->GetArrival().Get():nullptr;}
    const void* GetQualityAddress() const {return Values.Quality.Get();}
    float GetSeconds() const {return Seconds;}
};

// These functions only check/record the current private ticket. They never mint
// one from a caller's hit. Annotation present after Step closure must refuse.
IAMSPEED_API bool AppendFirstRsResponseDataFields(FNativePrepareGTFields& Out,const FString& Prefix,
    FFirstRsResponseDataPtr Data,Analytic::Certified::FDistanceBudget& Budget);
IAMSPEED_API bool AppendNativeHitIdentityFields(FNativePrepareGTFields& Out,const FString& Prefix,
    const SHitResult& Hit,Analytic::Certified::FDistanceBudget& Budget);

struct FFirstRsResponseActualCall
{
    EFirstRsResponseCall Call=EFirstRsResponseCall::Set;
    bool Returned=false,RawStateCaptured=false,SourceFieldsCaptured=false;
    TSharedPtr<const FNativePrepareGTFields,ESPMode::ThreadSafe> SourceBallFields;
    FString SourceFieldsRefusal;
    TArray<FKinematicState> Owners;
    TArray<TArray<FKinematicState>> Bodies;
    SHitResult Future,Current;
};
struct FTerminalFirstRsResponseSyncObservation
{
    uint64 ResponseSerial=0,NumericControl=0;
    FFirstRsResponseDataPtr Data;
    bool Attempted=false,Returned=false,CaptureAttempted=false,RawCaptured=false,SourceCaptured=false,Verified=false,Complete=false,Failed=false;
    TArray<FKinematicState> Owners;
    TArray<TArray<FKinematicState>> Bodies;
    TSharedPtr<const FNativePrepareGTFields,ESPMode::ThreadSafe> SourceBallFields;
    FString SourceFieldsRefusal;
};
struct FTerminalFirstRsResponseObservation
{
    uint64 Serial=0,AdvanceSerial=0,NumericControl=0;
    FFirstRsResponseDataPtr Data;
    int32 CompletedCalls=0,ActiveCall=INDEX_NONE;
    uint8 AttemptedMask=0,ReturnedMask=0,VerifiedMask=0,StateCaptureAttemptedMask=0,StateCapturedMask=0;
    bool Started=false,Complete=false,Failed=false;
    TArray<FFirstRsResponseActualCall> ActualCalls;
};
}
