#pragma once
#include "CoreMinimal.h"
#include "SimulationFrameJournal.h"
#include "CanonicalCommitObserverCore.h"
class ASpeedPhysicsSimulation;
namespace Speed::CanonicalObservers
{
using FHandle=CanonicalObserver20::FHandle;
using FConfig=CanonicalObserver20::FConfig;
using FStatus=CanonicalObserver20::FStatus;
using FCommitReceipt=CanonicalObserver20::FCommitReceipt;
using EReadiness=CanonicalObserver20::EReadiness;
using ECommitAction=CanonicalObserver20::ECommitAction;
// Borrowed local prepared values accepted by the successful same-frame
// publication. FSnapshotBuffer stamps serials only on its slot copies: the
// PreparedPresentationOutputs below retain their local serial (normally zero).
// FCommitReceipt.PublicationSerial is the actual published serial witness.
// These are not views into the stamped slot. No UObject/mutable engine API
// crosses this boundary. Never retain views after OnCommitted returns.
struct FCommittedValues
{
    TConstArrayView<uint8> Payload;
    TConstArrayView<FSimulationPresentationBody> PresentationBodies;
    TConstArrayView<FSimulationPresentationOutput> PreparedPresentationOutputs;
};
using IObserver=CanonicalObserver20::IObserver<FCommittedValues>;
struct FAccess20;
// GT only. All observers must be registered before first publication/frame0
// and before the first readiness decision seals their fixed slot set. Admission
// obtains the actual owned boundary ACK (or proves no worker exists yet).
IAMSPEED_API bool Register(ASpeedPhysicsSimulation&,const FConfig&,
    std::shared_ptr<IObserver>,FHandle& Out,FString& Failure);
// GT diagnostic only; not a frame admission or immutable final IO receipt.
IAMSPEED_API bool ReadStatus(ASpeedPhysicsSimulation&,const FHandle&,FStatus& Out,FString& Failure);
// GT only AFTER JoinOwnedSimulationForInputTeardown() has actually succeeded
// for this generation. Removal before join or from any callback is refused.
IAMSPEED_API bool RetireAfterJoin(ASpeedPhysicsSimulation&,const FHandle&,FString& Failure);
}
