// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Components/ShapeComponent.h"
#include "IAmSpeed/Base/SUtils.h"
#include "SSubBody.generated.h"

class ISpeedComponent;
class UBoxSubBody;
class USphereSubBody;
class USWheelSubBody;

/**
 * USubBody : Base class for any physics sub-body (wheel, hitbox, sphere) and detection SubBodies
 * It handles:
 *  - Kinematic prediction
 *  - CCD sweep
 *  - Time of impact extraction
 *  - Ignored components for the frame
 *  - Storing and resolving the current hit
 *
 * Subclasses must implement:
 *  - ResolveCurrentHitPrv for hit resolution specific to the shape
 */

UCLASS()
class IAMSPEED_API USSubBody : public UShapeComponent
{
    GENERATED_UCLASS_BODY()
	
public:

    enum class ESubBodyType
    {
        Unknown,
        Wheel,
        Hitbox,
        Sphere
	};

    virtual void Initialize(ISpeedComponent* InParentComponent);
    virtual bool IsSensor() const { return false; }
	virtual void AddExternalSubBodies(const TArray<USSubBody*>& SubBodies);
	virtual void RemoveExternalSubBodies(const TArray<USSubBody*>& SubBodies);

    // --- Called once per physics frame ---
    virtual void ResetForFrame(const float& Delta);
	virtual void PostPhysicsUpdate() {};

    // --- Set/Get kinematics ---
	virtual void UpdateKinematicsFromOwner(const SKinematic& ParentKinematic);
    void SetKinematicState(const Speed::FKinematicState& State) { Kinematics = State; }
    const Speed::FKinematicState& GetKinematicState() const { return Kinematics; }

    // --- Sweep for remaining delta time; return true if a hit occurred ---
    virtual bool SweepTOI(const float& RemainingDelta, float& OutTOI);

    // --- Advance the kinematic state by `t` seconds (where t <= delta) ---
    virtual void AdvanceToTOI(const float& t);

    // --- Accept the hit as current hit and ignore the component ---
    virtual void AcceptHit();
    bool ComponentHasBeenIgnored(const UPrimitiveComponent& OtherComp) const;

    // --- Resolve the impact (subclass-specific behaviour) ---
    virtual void ResolveCurrentHit(const float& delta, const float& SimTime);

    // --- Returns true if a valid hit exists ---
    bool HasHit() const { return CurrentHit.bBlockingHit; }
    bool WillHit() const { return FutureHit.bBlockingHit; }
    float GetFutureTOI() const { return FutureHit.TOI; }

	void SetHit(const SHitResult& Hit) { CurrentHit = Hit; }
    const SHitResult& GetHit() const { return CurrentHit; }
	const SHitResult& GetFutureHit() const { return FutureHit; }
	void SetFutureHit(const SHitResult& Hit) { FutureHit = Hit; }

	ESubBodyType GetSubBodyType() const { return SubBodyType; }
    FVector GetCenterOfMassWS() const { return Kinematics.Location; }

	ISpeedComponent* GetParentComponent() const { return ParentComponent; }

    // ====== REQUIRED PHYSICS SHAPE AND COLLISION PARAMS ======
    virtual FCollisionShape GetCollisionShape(float Inflation = 0.0f) const { return FCollisionShape::MakeSphere(10.f); }
    ECollisionChannel GetCollisionChannel() const { return CollisionChannel; }
    const FCollisionResponseParams& GetResponseParams() const { return ResponseParams; }

	FVector GetLocalOffset() const { return GetRelativeLocation(); }
	FQuat GetLocalRotation() const { return GetRelativeRotation().Quaternion(); }
    static uint64 MakePairKey(const USSubBody* A, const UPrimitiveComponent* OtherComp);
    static int32 SubBodyPriority(USSubBody::ESubBodyType T);
    static USSubBody* PickResolver(USSubBody* A, USSubBody* B);
protected:
    virtual void ResolveCurrentHitPrv(const float& delta, const float& SimTime) {};
    const TArray<TWeakObjectPtr<UBoxSubBody>> GetExternalBoxSubBodies() const;
    const TArray<TWeakObjectPtr<USphereSubBody>> GetExternalSphereSubBodies() const;
    const TArray<TWeakObjectPtr<USWheelSubBody>> GetExternalWheelSubBodies() const;
protected:

	ISpeedComponent* ParentComponent;

    // Kinematic state at start of frame or during substep
    Speed::FKinematicState Kinematics;

	ESubBodyType SubBodyType = ESubBodyType::Unknown;
	ECollisionChannel CollisionChannel = ECC_WorldDynamic;
    FCollisionResponseParams ResponseParams;

    // Cached hit info
    SHitResult FutureHit;
    SHitResult CurrentHit;
    SHitResult GroundHit;
    SHitResult BoxHit;
    SHitResult SphereHit;
    SHitResult WheelHit;

	// Components always ignored for this sub-body (e.g. the car body for the wheel sub-body)
	TArray<UPrimitiveComponent*> AlwaysIgnoredComponents;
    // Components ignored only during this frame
    TArray<UPrimitiveComponent*> IgnoredComponents;
	// Sub-body types always ignored for this sub-body (e.g. the wheel sub-body ignores other wheel sub-bodies)
    TArray<ESubBodyType> IgnoredSubBodyTypes;

    // cache external sub-bodies for sweeps
	TArray< TWeakObjectPtr<UBoxSubBody> > ExternalBoxSubBodies;
	TArray< TWeakObjectPtr<USphereSubBody> > ExternalSphereSubBodies;
	TArray< TWeakObjectPtr<USWheelSubBody> > ExternalWheelSubBodies;

    // Internal sweep helper
    bool InternalSweep(const FVector& Start, const FVector& End, SHitResult& OutHit, const float& delta);

    virtual FCollisionQueryParams BuildTraceParams() const;
};
