// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "SensorSubBody.h"
#include "IAmSpeed/SubBodies/Common/IBoxSweeper.h"
#include "IAmSpeed/SubBodies/Common/ISensorSweeper.h"
#include "BoxSensor.generated.h"

UCLASS(ClassGroup = "Collision", editinlinenew, meta = (DisplayName = "Box Sensor", BlueprintSpawnableComponent))
class IAMSPEED_API UBoxSensor : public USensorSubBody, public IBoxSweeper, public ISensorSweeper
{
    GENERATED_UCLASS_BODY()

public:
    void Initialize(ISpeedComponent* InParentComponent) override;

    // Called by the solver each substep
    bool SweepTOI(const float& RemainingDelta, float& OutTOI) override;

    FPrimitiveSceneProxy* CreateSceneProxy() override;
    FBoxSphereBounds CalcBounds(const FTransform& LocalToWorld) const override;
    void UpdateBodySetup();

    FVector GetBoxExtent() const { return BoxExtent; }
    void SetBoxExtent(FVector E) { BoxExtent = E.ComponentMax(FVector::ZeroVector); }
    const Speed::FKinematicState& GetKinematicState() const { return USensorSubBody::GetKinematicState(); }

protected:
    FCollisionShape GetCollisionShape(float Inflation = 0.f) const override;

#if WITH_EDITOR
    FEngineShowFlags GetShowFlags() const { return ShowFlags; }
    void SetShowFlags(const FEngineShowFlags& InShowFlags);
#endif // WITH_EDITOR

    /*
    * Overriden methods
    */
    bool InternalSweep(UWorld* World, const FVector& Start,
        const FVector& End, SHitResult& OutHit, const float& delta) {
        return ISensorSweeper::InternalSweep(World, Start, End, OutHit, delta);
    }
    bool SweepVsGround(UWorld* World, SHitResult& OutHit,
        const float& DeltaTime, float& OutTOI) override {
        return IBoxSweeper::SweepVsGround(World, OutHit, DeltaTime, OutTOI);
    }
    bool SweepVsBoxes(UWorld* World, SHitResult& OutHit,
        const float& DeltaTime, float& OutTOI) override {
        return IBoxSweeper::SweepVsBoxes(World, OutHit, DeltaTime, OutTOI);
    }
    bool SweepVsSpheres(UWorld* World, SHitResult& OutHit, const float& Delta, float& OutTOI) override { return IBoxSweeper::SweepVsSpheres(World, OutHit, Delta, OutTOI); }
    bool SweepVsWheels(UWorld* World, SHitResult& OutHit, const float& Delta, float& OutTOI) override { return IBoxSweeper::SweepVsWheels(World, OutHit, Delta, OutTOI); }
    virtual ECollisionChannel GetCollisionChannel() const { return USensorSubBody::GetCollisionChannel(); }
    virtual const FCollisionResponseParams& GetResponseParams() const { return USensorSubBody::GetResponseParams(); }
    virtual FCollisionQueryParams BuildTraceParams() const { return USensorSubBody::BuildTraceParams(); }
    bool ComponentHasBeenIgnored(const UPrimitiveComponent& OtherComp) const { return USensorSubBody::ComponentHasBeenIgnored(OtherComp); }
    const TArray<TWeakObjectPtr<UBoxSubBody>> GetExternalBoxSubBodies() const { return USensorSubBody::GetExternalBoxSubBodies(); }
    const TArray<TWeakObjectPtr<USphereSubBody>> GetExternalSphereSubBodies() const { return USensorSubBody::GetExternalSphereSubBodies(); }
    const TArray<TWeakObjectPtr<USWheelSubBody>> GetExternalWheelSubBodies() const { return USensorSubBody::GetExternalWheelSubBodies(); }

protected:
    UPROPERTY(EditAnywhere, BlueprintReadOnly, export, Category = Shape)
    FVector BoxExtent = FVector::ZeroVector;
#if WITH_EDITOR
    /** List of all show flags this box component visualizer should respect. */
    FEngineShowFlags ShowFlags;
#endif // WITH_EDITOR
};
