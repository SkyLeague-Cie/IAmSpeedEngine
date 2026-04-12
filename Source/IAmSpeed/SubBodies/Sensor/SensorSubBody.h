// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "SSubBody.h"
#include "SensorSubBody.generated.h"

UCLASS(Abstract, ClassGroup = "Collision", editinlinenew, meta = (BlueprintSpawnableComponent))
class IAMSPEED_API USensorSubBody : public USSubBody
{
    GENERATED_UCLASS_BODY()

public:
    virtual bool IsSensor() const override { return true; }

    // Called once at setup
    // Override this to configure what you want to detect
    void Initialize(ISpeedComponent* InParentComponent) override;

    void ResetForFrame(const float& Delta) override;

	void ResolveCurrentHit(const float& Delta, const float& SimTime) override;
	void ResolveCurrentHitPrv(const float& Delta, const float& SimTime) override;
    void PostPhysicsUpdate() override;

    UPrimitiveComponent* GetFirstOverlappingComponent() const;
	bool HasOverlaps() const { return OverlapsOrdered.Num() > 0; }

    DECLARE_MULTICAST_DELEGATE_TwoParams(FOnSensorOverlap, USensorSubBody* /*Sensor*/, UPrimitiveComponent* /*Other*/);
    FOnSensorOverlap OnBeginOverlap;
    FOnSensorOverlap OnEndOverlap;
    FOnSensorOverlap OnContinuousOverlap;

protected:
    // --- shape is provided by derived (sphere/box) through GetCollisionShape() ---
    bool ShouldIgnore(const UPrimitiveComponent& Other) const;
    FCollisionQueryParams BuildTraceParams() const override;
protected:
    // Overlaps in strict begin order
    UPROPERTY()
    TArray<TWeakObjectPtr<UPrimitiveComponent>> OverlapsOrdered;
    // Fast contains
    TSet<TWeakObjectPtr<UPrimitiveComponent>> OverlapsSet;
	// Components we touched this frame, to trigger continuous overlap events
    TSet<TWeakObjectPtr<UPrimitiveComponent>> TouchedThisFrame;
};
