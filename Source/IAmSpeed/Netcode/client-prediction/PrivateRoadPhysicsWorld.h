#pragma once
#include "CoreMinimal.h"
#include "BranchInputFilter.h"
#include "IAmSpeed/Base/Kinematic.h"
#include "IAmSpeed/World/Simulation/SimulationFrameJournal.h"
#include <atomic>
#include <memory>
#include <variant>

namespace Speed::Netcode::Prediction
{
enum class EPrivateRoadStartPolicy : uint8 { AwaitInitialPublication, ExplicitUnitRecipe };
struct FPrivateRoadCarSeed
{
    U64 Actor=0,Incarnation=0;
    SKinematic Initial;
    // Network profile starts closed; countdown is only an explicit unit recipe.
    EPrivateRoadStartPolicy StartPolicy=EPrivateRoadStartPolicy::AwaitInitialPublication;
    uint16 CountdownFrames=0;
    EBranchFilterRole Role=EBranchFilterRole::RemoteRequests;
    std::shared_ptr<const BranchV2::FInputActionContract> Contract;
    BranchV2::FInputProcessingPolicy Processing;
    Speed::Input::FProducerIdentity Local;
    BranchV2::FStreamEpoch LocalEpoch;
    FBinding Remote;
    U64 PredictionHorizon=256;
};
struct FPrivateRoadRequest
{
    U64 Actor=0,Incarnation=0;
    std::variant<std::shared_ptr<const BranchV2::FInputFrame>,FPrediction> Evidence;
};
struct FPrivateRoadActorFrame
{
    U64 Actor=0,Incarnation=0,NativeStableId=0;
    FBranchFilterState Filter;
    std::shared_ptr<const FBranchProcessedInput> Input;
    bool CanMove=false,Frozen=false;
    uint16 CountdownFrames=0; // actual native values, diagnostics only
};
struct FPrivateRoadPublication
{
    U64 LocalKernelLifetime=0,Frame=0;
    bool AwaitingInitialPublication=false;
    uint32 NativeNetworkComponents=0; // actual object census, not just history
    uint32 NativeDebugDelegateBindings=0; // bound entries; not an in-flight-reader drain certificate
    bool RenderScenePresent=false,PhysicsScenePresent=false; // actual private world scene pointers
    FSimulationSnapshot Physics;
    std::vector<FPrivateRoadActorFrame> Actors;
};
// Experimental finite native flat-road kernel. Creates actual, private native
// cars and runs the unchanged canonical IAmSpeed solver at300Hz on the GT.
// This public API returns immutable VALUES. Engine/global enumeration and
// previously escaped native views require separate exclusion qualification.
// This factory does not authenticate seeds/input, select a connected client,
// supply a complete restore provider or certify lifecycle/view exclusion for
// an ISimulationRestoreBoundaryLease. Unsupported scene/game domains stay closed.
class IAMSPEED_API FPrivateRoadPhysicsWorld final
{
    struct FImpl;
    std::unique_ptr<FImpl> Impl;
    FPrivateRoadPhysicsWorld();
    bool HasExactRegisteredRoster(FString& Why) const;
public:
    ~FPrivateRoadPhysicsWorld();
    FPrivateRoadPhysicsWorld(const FPrivateRoadPhysicsWorld&)=delete;
    FPrivateRoadPhysicsWorld& operator=(const FPrivateRoadPhysicsWorld&)=delete;
    static std::unique_ptr<FPrivateRoadPhysicsWorld> Create(std::vector<FPrivateRoadCarSeed> Seeds,FString& Why);
    bool Step(const std::vector<FPrivateRoadRequest>& Requests,bool Resimulation,FString& Why);
    std::shared_ptr<const FPrivateRoadPublication> ReadLatest() const noexcept;
    // Local-owner transition after the session has compared its ACTUAL startup
    // publication with the original server and received the common activation.
    // This is NOT a complete hidden-state/restore certificate or source admission.
    bool ReleaseInitialPublication(const std::shared_ptr<const FPrivateRoadPublication>& OriginalBaseline,FString& Why);
    bool Close(); // GT only; never destroys native storage in a physical/publication tail
};
}
