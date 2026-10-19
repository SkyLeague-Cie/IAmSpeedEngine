// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "SSubBody.h"
#if !UE_BUILD_SHIPPING
#include "SensorDelegateCensusForTesting1075.h"
#endif
#include "SensorSubBody.generated.h"
#if !UE_BUILD_SHIPPING
namespace Speed { class FFramePointerRegistry; }
#endif

namespace Speed::Replay {struct FSensorReplayState;}

UCLASS(Abstract, ClassGroup = "Collision", editinlinenew, meta = (BlueprintSpawnableComponent))
class IAMSPEED_API USensorSubBody : public USSubBody
{
    GENERATED_UCLASS_BODY()

public:
#if !UE_BUILD_SHIPPING
    bool AppendOverlapStateForFrameWitnessForTesting(TArray<uint8>& Out,const Speed::FFramePointerRegistry& Registry) const;
#endif
	virtual void BeginPlay() override;
    virtual bool IsSensor() const override { return true; }

    // Called once at setup
    // Override this to configure what you want to detect
    void Initialize(ISpeedComponent* InParentComponent) override;

    void ResetForFrame(const float& Delta) override;

	void ResolveCurrentHit(const float& Delta, const float& SimTime) override;
	void ResolveCurrentHitPrv(const float& Delta, const float& SimTime) override;
    void PostPhysicsUpdate() override;

    // Canonical physical owner only. Captures exact ordered/set/touched
    // identities, never addresses or names; an unresolved weak reference fails.
    bool CaptureReplaySensorState(Speed::Replay::FSensorReplayState& Out) const;
    UPrimitiveComponent* GetFirstOverlappingComponent() const;
	bool HasOverlaps() const { return OverlapsOrdered.Num() > 0; }

    DECLARE_MULTICAST_DELEGATE_TwoParams(FOnSensorOverlap, USensorSubBody* /*Sensor*/, UPrimitiveComponent* /*Other*/);
#if !UE_BUILD_SHIPPING
    using FObservedSensorOverlap1075=Speed::TSensorDelegateCensus1075<FOnSensorOverlap>;
    static_assert(sizeof(FObservedSensorOverlap1075)==sizeof(FOnSensorOverlap),"observer cannot add delegate storage");
    static_assert(alignof(FObservedSensorOverlap1075)==alignof(FOnSensorOverlap),"observer cannot change delegate alignment");
    FObservedSensorOverlap1075 OnBeginOverlap;
    FObservedSensorOverlap1075 OnEndOverlap;
    FObservedSensorOverlap1075 OnContinuousOverlap;
#else
    FOnSensorOverlap OnBeginOverlap;
    FOnSensorOverlap OnEndOverlap;
    FOnSensorOverlap OnContinuousOverlap;
#endif

	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
protected:
    // --- shape is provided by derived (sphere/box) through GetCollisionShape() ---
    bool ShouldIgnore(const UPrimitiveComponent& Other) const;
    FCollisionQueryParams BuildTraceParams() const override;
	virtual void BindDelegates();
	virtual void UnbindDelegates();
protected:
    // Overlaps in strict begin order
    UPROPERTY()
    TArray<TWeakObjectPtr<UPrimitiveComponent>> OverlapsOrdered;
    // Fast contains
    TSet<TWeakObjectPtr<UPrimitiveComponent>> OverlapsSet;
	// Components we touched this frame, to trigger continuous overlap events
    TSet<TWeakObjectPtr<UPrimitiveComponent>> TouchedThisFrame;

	// Handles to the Continuous delegates, to be able to unbind them on destruction
    FDelegateHandle ContinuousOverlapHandle;
	// Handles to the Begin delegates, to be able to unbind them on destruction
    FDelegateHandle BeginOverlapHandle;
	// Handles to the End delegates, to be able to unbind them on destruction
    FDelegateHandle EndOverlapHandle;
};
