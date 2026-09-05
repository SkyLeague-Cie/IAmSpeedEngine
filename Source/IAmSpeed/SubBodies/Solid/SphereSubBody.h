// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "SolidSubBody.h"
#include "IAmSpeed/SubBodies/Common/ISphereSweeper.h"
#include "IAmSpeed/SubBodies/Common/ISolidSweeper.h"
#include "UObject/ObjectMacros.h"
#include "SphereSubBody.generated.h"

class UBoxSubBody;

struct FSphereWorldStaticPenetrationDiagnostics
{
	bool bEnabled = false;
	int32 StartFrame = INDEX_NONE;
	int32 SweepInitialOverlapSamples = 0;
	int32 ProjectionInputSamples = 0;
	int32 ProjectionResidualSamples = 0;
	float MaximumSweepInitialOverlapCm = 0.0f;
	float MaximumProjectionInputDepthCm = 0.0f;
	float MaximumProjectionResidualDepthCm = 0.0f;
	int32 MaximumSweepInitialOverlapFrame = INDEX_NONE;
	int32 MaximumProjectionInputFrame = INDEX_NONE;
	int32 MaximumProjectionResidualFrame = INDEX_NONE;
};

DECLARE_LOG_CATEGORY_EXTERN(SphereSubBodyLog, Log, All);

/**
 * USphereSubBody : A sub-body representing a sphere. It provides functionality for collision detection and response specific for each shape.
 */
UCLASS(ClassGroup = "Collision", editinlinenew, hidecategories = (Object, LOD, Lighting, TextureStreaming), meta = (DisplayName = "Sphere Collision", BlueprintSpawnableComponent))
class IAMSPEED_API USphereSubBody : public USolidSubBody, public ISphereSweeper, public ISolidSweeper
{
    GENERATED_UCLASS_BODY()
	
public:
	virtual void Initialize(ISpeedComponent* InParentComponent) override;
	void ResetForFrame(const float& Delta) override;
	void PostPhysicsUpdate() override;
	void AcceptHit() override;

    // Called by the solver each substep
    virtual bool SweepTOI(const float& RemainingDelta, float& OutTOI) override;
    const Speed::FKinematicState& GetKinematicState() const { return USolidSubBody::GetKinematicState(); }
    SKinematic GetKinematicsFromOwner(const unsigned int& NumFrame) const;
    virtual SSphere MakeSphere() const;
    FMatrix ComputeWorldInvInertiaTensor() const override;

    FPrimitiveSceneProxy* CreateSceneProxy() override;
    FBoxSphereBounds CalcBounds(const FTransform& LocalToWorld) const override;
    void UpdateBodySetup();
	void HandleMicroOscillation();
	bool ProjectOutOfBox(UBoxSubBody& OtherBox);
	bool ProjectOutOfWorldStatic();
	void BeginWorldStaticPenetrationDiagnostics();
	const FSphereWorldStaticPenetrationDiagnostics& GetWorldStaticPenetrationDiagnostics() const
	{
		return WorldStaticPenetrationDiagnostics;
	}
	static bool IsSphereBoxProjectionEnabled();
	static bool IsWorldStaticProjectionEnabled();
    float GetRadius() const { return Radius; }
	void SetRadiusForConfiguration(float NewRadius)
	{
		SetRadius(NewRadius);
		InvInertiaLocal = InitInvInertiaTensor();
	}
	void SetSphereBoxTangentialContactArmScale(float InScale)
	{
		SphereBoxTangentialContactArmScale = FMath::Max(0.0f, InScale);
	}
	void SetSphereBoxTangentialContactArmSpeedRange(float InStartSpeed, float InFullSpeed)
	{
		SphereBoxTangentialContactArmStartSpeed = FMath::Max(0.0f, InStartSpeed);
		SphereBoxTangentialContactArmFullSpeed = FMath::Max(
			SphereBoxTangentialContactArmStartSpeed, InFullSpeed);
	}
	void SetSphereBoxTangentialContactArmKinematicWindow(
		float InMinSphereSpeed,
		float InMaxSphereSpeed,
		float InMinSphereAngularSpeed,
		float InMaxSphereAngularSpeed,
		float InMinBoxSpeed)
	{
		SphereBoxTangentialArmMinSphereSpeed = InMinSphereSpeed;
		SphereBoxTangentialArmMaxSphereSpeed = InMaxSphereSpeed;
		SphereBoxTangentialArmMinSphereAngularSpeed = InMinSphereAngularSpeed;
		SphereBoxTangentialArmMaxSphereAngularSpeed = InMaxSphereAngularSpeed;
		SphereBoxTangentialArmMinBoxSpeed = InMinBoxSpeed;
	}
	void ApplySphereBoxImpulse(
		const FVector& LinearImpulse,
		const FVector& WorldPoint,
		const FVector& ContactNormal,
		float RelativeNormalSpeed,
		const SKinematic& SphereState,
		const SKinematic& BoxState);
	bool ShouldMaintainSphereBoxContact(
		float RelativeNormalSpeed,
		const FVector& ContactNormal,
		const SKinematic& SphereState,
		const SKinematic& BoxState) const;
protected:
	bool IsTangentialContactArmEligible(
		const SKinematic& SphereState,
		const SKinematic& BoxState) const;
    virtual FCollisionShape GetCollisionShape(float Inflation = 0.0f) const override;

    virtual void ResolveCurrentHitPrv(const float& delta, const float& SimTime) override;
    void ResolveHitVsGround(const float& delta, const float& SimTime);
    void ResolveHitVsBox(UBoxSubBody& OtherBox, const float& delta, const float& SimTime);
    void ResolveHitVsSphere(USphereSubBody& OtherSphere, const float& delta, const float& SimTime);

    /*
    * Overriden methods
    */
    bool InternalSweep(UWorld* World, const FVector& Start,
        const FVector& End, SHitResult& OutHit, const float& delta) {
        return ISolidSweeper::InternalSweep(World, Start, End, OutHit, delta);
    }
    bool SweepVsGround(UWorld* World, SHitResult& OutHit,
        const float& DeltaTime, float& OutTOI) override {
        return ISphereSweeper::SweepVsGround(World, OutHit, DeltaTime, OutTOI);
    }
    bool SweepVsBoxes(UWorld* World, SHitResult& OutHit,
        const float& DeltaTime, float& OutTOI) override {
        return ISphereSweeper::SweepVsBoxes(World, OutHit, DeltaTime, OutTOI);
    }
    bool SweepVsSpheres(UWorld* World, SHitResult& OutHit, const float& Delta, float& OutTOI) override { return ISphereSweeper::SweepVsSpheres(World, OutHit, Delta, OutTOI); }
    bool SweepVsWheels(UWorld* World, SHitResult& OutHit, const float& Delta, float& OutTOI) override { return ISphereSweeper::SweepVsWheels(World, OutHit, Delta, OutTOI); }
    virtual ECollisionChannel GetCollisionChannel() const { return USolidSubBody::GetCollisionChannel(); }
    virtual const FCollisionResponseParams& GetResponseParams() const { return USolidSubBody::GetResponseParams(); }
    virtual FCollisionQueryParams BuildTraceParams() const { return USolidSubBody::BuildTraceParams(); }
	bool ShouldSkipBoxSweep(const UBoxSubBody& Box) const override;
    bool ComponentHasBeenIgnored(const UPrimitiveComponent& OtherComp) const { return USolidSubBody::ComponentHasBeenIgnored(OtherComp); }
    const TArray<TWeakObjectPtr<UBoxSubBody>>& GetExternalBoxSubBodies() const { return USolidSubBody::GetExternalBoxSubBodies(); }
    const TArray<TWeakObjectPtr<USphereSubBody>>& GetExternalSphereSubBodies() const { return USolidSubBody::GetExternalSphereSubBodies(); }
    const TArray<TWeakObjectPtr<USWheelSubBody>>& GetExternalWheelSubBodies() const { return USolidSubBody::GetExternalWheelSubBodies(); }

	virtual float CollisionMargin() const { return 0.01f; }
	void SetRadius(const float& NewRadius);
    float GetRadiusWithMargin() const;
protected:
    FMatrix InitInvInertiaTensor() const override;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, export, Category = Shape)
    float Radius = 0.f; // Real radius of the sphere
	float MinSlopCm = 0.05f; // Minimum allowed penetration depth in cm, used as slop in the solver to prevent jittering when resolving penetrations
	// Scales only the angular response caused by the tangential sphere/box
	// impulse. Translation keeps the full rigid impulse.
	float SphereBoxTangentialContactArmScale = 1.0f;
	float SphereBoxTangentialContactArmStartSpeed = 0.0f;
	float SphereBoxTangentialContactArmFullSpeed = 0.0f;
	float SphereBoxTangentialArmMinSphereSpeed = -1.0f;
	float SphereBoxTangentialArmMaxSphereSpeed = -1.0f;
	float SphereBoxTangentialArmMinSphereAngularSpeed = -1.0f;
	float SphereBoxTangentialArmMaxSphereAngularSpeed = -1.0f;
	float SphereBoxTangentialArmMinBoxSpeed = -1.0f;
	FSphereWorldStaticPenetrationDiagnostics WorldStaticPenetrationDiagnostics;
};
