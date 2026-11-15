#pragma once
#include "PreparedSphereRequest.h"
#include "PreparedSphereRsTemporalPrefix.h"
#include "PreparedSphereConsumedState.h"
namespace Speed::PreparationPrefix {class FClosedSourceProducer;}
namespace Speed::PreparedExecution
{
struct FNativeRsTraceValues;
struct FConsumedPrefixActualStates
{
    int32 AfterOwnerIndex=INDEX_NONE;bool CallbackReturned=false;
    TArray<FKinematicState> Owners;
    TArray<TArray<FKinematicState>> Bodies;
};
struct FTerminalConsumedPrefixAdvanceObservation
{
    uint64 Serial=0,NumericControl=0;float Seconds=0;
    int32 CompletedOwners=0,ActiveIndex=INDEX_NONE;
    uint8 AttemptedMask=0,ReturnedMask=0,VerifiedMask=0;
    bool Started=false,Complete=false;
    uint8 StateCaptureAttemptedMask=0,StateCapturedMask=0;
    TArray<FConsumedPrefixActualStates> ActualStates; // observed, NEVER implicitly verified
};
struct FRsPrefixOwnerValues
{
    const ISpeedComponent* Owner=nullptr;
    uint64 StableId=0;uint32 Frame=0;
    const FKinematicState* Stored=nullptr;const FKinematicState* Selected=nullptr;
    FKinematicState Before,After;
    TArray<FKinematicState> BodiesBefore,BodiesAfter;
    FVector LocalCOM=FVector::ZeroVector;
    bool Frozen=false;float MaximumSpeed=0,MaximumAngularSpeed=0;
};
class FRsPrefixAdvanceSource;
using FRsPrefixAdvanceSourcePtr=TSharedPtr<const FRsPrefixAdvanceSource,ESPMode::ThreadSafe>;
// A closed Source proposition sealed by the actual sphere member while its
// request is live. Possession alone grants no native mutation. A DIFFERENT
// one-use advance transition must validate it after real member revocation.
class FRsPrefixAdvanceSource final
{
    friend class Speed::PreparationPrefix::FClosedSourceProducer;
    friend class FSphereMemberSweepScope;friend class FStepScope;
    FNativeSphereRequestPtr Request;
    Analytic::Prepared::FReferenceSphereRsTemporalPrefixPtr Prefix;
    TSharedPtr<const void,ESPMode::ThreadSafe> PhysicalSourcePlan;
    TArray<FRsPrefixOwnerValues> Owners;
    TSharedPtr<const FNativeRsTraceValues,ESPMode::ThreadSafe> Trace;
    FSphereFreeAdvanceValues BallValues;
    FSphereFreeAdvanceExpectation BallExpected;
    uint64 NumericControl=0;
    mutable bool SealedDuringMember=false;
    FRsPrefixAdvanceSource()=default;
};
}
