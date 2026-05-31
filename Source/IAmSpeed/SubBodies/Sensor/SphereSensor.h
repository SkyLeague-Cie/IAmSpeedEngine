// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "SensorSubBody.h"
#include "IAmSpeed/SubBodies/Common/ISphereSweeper.h"
#include "IAmSpeed/SubBodies/Common/ISensorSweeper.h"
#include "SphereSensor.generated.h"

UCLASS(ClassGroup = "Collision", editinlinenew, meta = (DisplayName = "Sphere Sensor", BlueprintSpawnableComponent))
class IAMSPEED_API USphereSensor : public USensorSubBody, public ISphereSweeper, public ISensorSweeper
{
    GENERATED_UCLASS_BODY()

public:
    void Initialize(ISpeedComponent* InParentComponent) override;

    // Called by the solver each substep
    bool SweepTOI(const float& RemainingDelta, float& OutTOI) override;

    FPrimitiveSceneProxy* CreateSceneProxy() override;
    FBoxSphereBounds CalcBounds(const FTransform& LocalToWorld) const override;
    void UpdateBodySetup();

    const Speed::FKinematicState& GetKinematicState() const { return USensorSubBody::GetKinematicState(); }
    float GetRadius() const { return Radius; }
    float GetRadiusWithMargin() const;
    void  SetRadius(float R) { Radius = FMath::Max(0.f, R); }

protected:
    FCollisionShape GetCollisionShape(float Inflation = 0.f) const override;
    virtual float CollisionMargin() const { return KINDA_SMALL_NUMBER; }

    /*
    * Overriden methods
    */
    bool InternalSweep(UWorld* World, const FVector& Start,
        const FVector& End, SHitResult& OutHit, const float& delta) { return ISensorSweeper::InternalSweep(World, Start, End, OutHit, delta); }
    bool SweepVsGround(UWorld* World, SHitResult& OutHit,
        const float& DeltaTime, float& OutTOI) override { return ISphereSweeper::SweepVsGround(World, OutHit, DeltaTime, OutTOI);}
    bool SweepVsBoxes(UWorld* World, SHitResult& OutHit,
        const float& DeltaTime, float& OutTOI) override { return ISphereSweeper::SweepVsBoxes(World, OutHit, DeltaTime, OutTOI); }
    bool SweepVsSpheres(UWorld* World, SHitResult& OutHit, const float& Delta, float& OutTOI) override { return ISphereSweeper::SweepVsSpheres(World, OutHit, Delta, OutTOI); }
    bool SweepVsWheels(UWorld* World, SHitResult& OutHit, const float& Delta, float& OutTOI) override { return ISphereSweeper::SweepVsWheels(World, OutHit, Delta, OutTOI); }
    virtual ECollisionChannel GetCollisionChannel() const { return USensorSubBody::GetCollisionChannel(); }
    virtual const FCollisionResponseParams& GetResponseParams() const { return USensorSubBody::GetResponseParams(); }
    virtual FCollisionQueryParams BuildTraceParams() const { return USensorSubBody::BuildTraceParams(); }
    bool ComponentHasBeenIgnored(const UPrimitiveComponent& OtherComp) const { return USensorSubBody::ComponentHasBeenIgnored(OtherComp); }
    const TArray<TWeakObjectPtr<UBoxSubBody>> GetExternalBoxSubBodies() const { return USensorSubBody::GetExternalBoxSubBodies(); }
    const TArray<TWeakObjectPtr<USphereSubBody>> GetExternalSphereSubBodies() const { return USensorSubBody::GetExternalSphereSubBodies(); }
    const TArray<TWeakObjectPtr<USWheelSubBody>> GetExternalWheelSubBodies() const { return USensorSubBody::GetExternalWheelSubBodies(); }

protected:
    UPROPERTY(EditAnywhere, BlueprintReadOnly, export, Category = "Shape")
    float Radius = 32.f;
};
