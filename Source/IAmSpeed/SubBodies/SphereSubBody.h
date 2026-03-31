// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "SolidSubBody.h"
#include "UObject/ObjectMacros.h"
#include "SphereSubBody.generated.h"

class UBoxSubBody;

DECLARE_LOG_CATEGORY_EXTERN(SphereSubBodyLog, Log, All);

/**
 * USphereSubBody : A sub-body representing a sphere. It provides functionality for collision detection and response specific for each shape.
 */
UCLASS(ClassGroup = "Collision", editinlinenew, hidecategories = (Object, LOD, Lighting, TextureStreaming), meta = (DisplayName = "Sphere Collision", BlueprintSpawnableComponent))
class IAMSPEED_API USphereSubBody : public USolidSubBody
{
    GENERATED_UCLASS_BODY()
	
public:
	virtual void Initialize(ISpeedComponent* InParentComponent) override;
	void ResetForFrame(const float& Delta) override;

    // Called by the solver each substep
    virtual bool SweepTOI(const float& RemainingDelta, const float& TimePassed, float& OutTOI) override;
    SKinematic GetKinematicsFromOwner(const unsigned int& NumFrame) const;
    virtual SSphere MakeSphere(const unsigned int& NumFrame, const float& RemainingDelta, const float& TimePassed) const;
    FMatrix ComputeWorldInvInertiaTensor() const override;

    FPrimitiveSceneProxy* CreateSceneProxy() override;
    FBoxSphereBounds CalcBounds(const FTransform& LocalToWorld) const override;
    void UpdateBodySetup();
	void HandleMicroOscillation();
    float GetRadius() const { return Radius; }
protected:
    virtual FCollisionShape GetCollisionShape(float Inflation = 0.0f) const override;

    virtual bool SweepVsGround(SHitResult& OutHit, const float& DeltaTime, float& OutTOI);
    virtual bool SweepVsBoxes(SHitResult& OutHit, const float& DeltaTime, const float& TimePassed, float& OutTOI);
    virtual bool SweepVsSpheres(SHitResult& OutHit, const float& Delta, const float& TimePassed, float& OutTOI);
	virtual bool SweepVsWheels(SHitResult& OutHit, const float& Delta, const float& TimePassed, float& OutTOI);

    virtual void ResolveCurrentHitPrv(const float& delta, const float& TimePassed, const float& SimTime) override;
    void ResolveHitVsGround(const float& delta, const float& SimTime);
    void ResolveHitVsBox(UBoxSubBody& OtherBox, const float& delta, const float& TimePassed, const float& SimTime);
    void ResolveHitVsSphere(USphereSubBody& OtherSphere, const float& delta, const float& TimePassed, const float& SimTime);

	virtual float CollisionMargin() const { return KINDA_SMALL_NUMBER; }
	void SetRadius(const float& NewRadius);
    float GetRadiusWithMargin() const;
protected:
    FMatrix InitInvInertiaTensor() const override;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, export, Category = Shape)
    float Radius = 0.f; // Real radius of the sphere
	float MinSlopCm = 0.05f; // Minimum allowed penetration depth in cm, used as slop in the solver to prevent jittering when resolving penetrations

protected:
    SHitResult GroundHit;
    SHitResult BoxHit;
    SHitResult SphereHit;
	SHitResult WheelHit;
};
