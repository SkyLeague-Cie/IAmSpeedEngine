#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "SimulationFrameJournal.h"
#include "SpeedSimulation.generated.h"

// Common values/presentation contract only. Construction does not create a
// worker, input owner, physical world, file reader or canonical frame driver.
// Physics and recorded playback have distinct execution/lifecycle policies.
UCLASS(Abstract)
class IAMSPEED_API ASpeedSimulation : public AActor
{
    GENERATED_BODY()
public:
    ASpeedSimulation();
    virtual bool ReadLatestSimulationSnapshot(FSimulationSnapshot& OutSnapshot) const
        PURE_VIRTUAL(ASpeedSimulation::ReadLatestSimulationSnapshot, return false;);
    virtual bool ReadPresentationPose(uint64 StableId, FSimulationPoseConsumption& Out)
        PURE_VIRTUAL(ASpeedSimulation::ReadPresentationPose, return false;);
    virtual bool ReadPresentationOutput(uint64 StableId, uint32 Channel, FSimulationPresentationOutput& Out)
        PURE_VIRTUAL(ASpeedSimulation::ReadPresentationOutput, return false;);
    virtual bool AllowsCameraPresentation() const { return true; }
    virtual bool ReadCanonicalCameraSample(uint64 NumFrame, FCameraCanonicalSample& Out) const
    { Out = {}; return false; }
    virtual bool ReadCanonicalCameraSamples(uint64 FirstFrame, uint64 LastFrame, TArray<FCameraCanonicalSample>& Out) const
    { Out.Reset(); return false; }
    // Retirement of the publisher, not an implicit file IO join. The physical
    // implementation requires its actual worker join; replay has no such worker
    // and retires its GT publication before its independent IO owner drains.
    virtual TSharedPtr<const TAtomic<bool>, ESPMode::ThreadSafe> GetPresentationRetirementWitness() const
        PURE_VIRTUAL(ASpeedSimulation::GetPresentationRetirementWitness, return nullptr;);
#if !UE_BUILD_SHIPPING
    virtual void AuditPresentationFrameCadence(const AActor& Observer, float GameDeltaSeconds,
        Speed::PublicationCadence::FObserver& Observation) {}
#endif
};
