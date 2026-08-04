// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "IAmSpeed/SubBodies/SSubBody.h"
#include "SolidSubBody.generated.h"

UENUM()
enum class EMixMode : uint8
{
	E_Average UMETA(DisplayName = "Average"),
	E_Min UMETA(DisplayName = "Min"),
	E_Multiply UMETA(DisplayName = "Multiply"),
	E_Max UMETA(DisplayName = "Max"),
};

UENUM()
enum class ERestitutionResolveMode : uint8
{
	Global UMETA(DisplayName = "Global"),
	AtContact UMETA(DisplayName = "At Contact"),
	Custom UMETA(DisplayName = "Custom"),
};

struct FRestitutionResolveContext
{
	FVector ContactPointWS = FVector::ZeroVector;
	FVector BoxLocalContactPoint = FVector::ZeroVector;
	FVector BoxLocalContactNormal = FVector::ZeroVector;
	FVector BoxExtent = FVector::ZeroVector;
};

struct FFakePhysicsImpactContext
{
	SKinematic SelfParentKinematics;
	SKinematic OtherParentKinematics;
	// Scales the velocity delta produced by fake physics. Discrete impacts use
	// the default value; persistent manifolds can supply a time-normalized share.
	float FakeImpulseScale = 1.0f;
	// A supported manifold owns the normal response. Its fake-physics share may
	// therefore accelerate only along the current contact plane.
	bool bProjectFakeImpulseOntoContactPlane = false;
	// Velocity that a discrete solver would have accumulated between impacts but
	// which the continuously enforced normal constraint has already removed.
	FVector FakeRelativeVelocityBias = FVector::ZeroVector;
	// Persistent contacts may feed only a fraction of their already accumulated
	// tangential slip back into the shot-amplitude model.
	float FakeTangentialVelocityScale = 1.0f;
	// A moving-surface manifold measures slip where the two shapes touch rather
	// than reusing their center-of-mass relative velocity.
	bool bUseContactPointRelativeVelocity = false;

	FFakePhysicsImpactContext Inverted() const
	{
		FFakePhysicsImpactContext Result;
		Result.SelfParentKinematics = OtherParentKinematics;
		Result.OtherParentKinematics = SelfParentKinematics;
		Result.FakeImpulseScale = FakeImpulseScale;
		Result.bProjectFakeImpulseOntoContactPlane =
			bProjectFakeImpulseOntoContactPlane;
		Result.FakeRelativeVelocityBias = -FakeRelativeVelocityBias;
		Result.FakeTangentialVelocityScale = FakeTangentialVelocityScale;
		Result.bUseContactPointRelativeVelocity =
			bUseContactPointRelativeVelocity;
		return Result;
	}
};

/**
 * USolidSubBody : Base class for any solid physics sub-body (box, sphere, etc.)
 * It handles:
 *  - Mass and inertia
 *  - Fake physics impulses
 *  - Collision properties (restitution, friction, impact threshold)
 */
UCLASS()
class IAMSPEED_API USolidSubBody : public USSubBody
{
	GENERATED_BODY()
	
public:
	virtual void Initialize(ISpeedComponent* InParentComponent) override;

	float GetMass() const { return Mass; }
	void SetMass(float InMass) { Mass = InMass; }
	virtual FMatrix ComputeWorldInvInertiaTensor() const;
	int Idx() const { return idx; }
	void SetIdx(int InIdx) { idx = InIdx; }
	bool IsMainSubBody() const { return bIsMainSubBody; }
	bool IsFakePhysicsEnabled() const { return EnableFakePhysics; }
	float GetRestitution() const { return Restitution; }
	float GetSphereBoxRestitutionOverride() const { return SphereBoxRestitutionOverride; }
	float GetSphereBoxFrictionOverride() const { return SphereBoxFrictionOverride; }
	bool UsesCoupledContactImpulse() const { return bUseCoupledContactImpulse; }
	bool UsesPostNormalFrictionImpulse() const { return bUsePostNormalFrictionImpulse; }
	bool UsesPersistentBilateralContact() const { return bUsePersistentBilateralContact; }
	bool AllowsCoupledContactImpulseFor(const SKinematic& State) const
	{
		return (CoupledContactMaxLinearSpeed < 0.0f ||
			State.Velocity.Size() <= CoupledContactMaxLinearSpeed) &&
			(CoupledContactMaxAngularSpeed < 0.0f ||
			State.AngularVelocity.Size() <= CoupledContactMaxAngularSpeed);
	}
	bool AllowsCoupledContactNormal(const FVector& LocalNormal) const
	{
		return CoupledContactMaxAbsLocalNormalY < 0.0f ||
			FMath::Abs(LocalNormal.Y) <= CoupledContactMaxAbsLocalNormalY;
	}
	float GetStaticFriction() const { return StaticFriction; }
	float GetDynamicFriction() const { return DynamicFriction; }
	float GetImpactThreshold() const { return ImpactThreshold; }
	float GetHitDamping() const { return HitDamping; }
	UFUNCTION(BlueprintCallable, Category = "Physics")
	void SetSphereBoxRestitutionOverride(float InRestitution);
	void SetSphereBoxFrictionOverride(float InFriction)
	{
		SphereBoxFrictionOverride = InFriction < 0.0f ? -1.0f : InFriction;
	}
	void SetUseCoupledContactImpulse(bool bInEnabled)
	{
		bUseCoupledContactImpulse = bInEnabled;
	}
	void SetUsePostNormalFrictionImpulse(bool bInEnabled)
	{
		bUsePostNormalFrictionImpulse = bInEnabled;
	}
	void SetUsePersistentBilateralContact(bool bInEnabled)
	{
		bUsePersistentBilateralContact = bInEnabled;
	}
	void SetCoupledContactKinematicLimits(
		float MaxLinearSpeed,
		float MaxAngularSpeed)
	{
		CoupledContactMaxLinearSpeed = MaxLinearSpeed;
		CoupledContactMaxAngularSpeed = MaxAngularSpeed;
	}
	void SetCoupledContactMaxAbsLocalNormalY(float MaxAbsLocalNormalY)
	{
		CoupledContactMaxAbsLocalNormalY = MaxAbsLocalNormalY;
	}

	// Override this function to apply custom impulses on the subbody itself when a hit is resolved.
	virtual void ApplyImpulse(const FVector& LinearImpulse, const FVector& WorldPoint);
	// Override this function to apply custom impulses on the subbody itself or other subbodies when a hit is resolved.
	virtual void ApplyFakePhysicsOn(
		USolidSubBody& OtherSubBody,
		const SHitResult& Hit,
		const float& DeltaTime,
		const FFakePhysicsImpactContext* ImpactContext = nullptr) {};
	virtual float ResolveRestitutionCustom(
		const USolidSubBody& OtherSubBody,
		const FRestitutionResolveContext& Context,
		float GlobalRestitution) const { return GlobalRestitution; }
	virtual FVector GetVelocityAtPoint(const FVector& Point) const;

	static float MixRestitution(float eA, float eB, EMixMode Mode);
	static float MixFriction(float muA, float muB, EMixMode Mode);
	static float ResolveSphereBoxFriction(
		const USolidSubBody& Sphere,
		const USolidSubBody& Box,
		float FallbackFriction);
	static float ResolveSphereBoxRestitution(const USolidSubBody& Sphere, const USolidSubBody& Box, float FallbackRestitution);
	static float ResolveSphereBoxRestitutionAtContact(
		const USolidSubBody& Sphere,
		const USolidSubBody& Box,
		float FallbackRestitution,
		const FVector& BoxLocalContactPoint,
		const FVector& BoxLocalContactNormal,
		const FVector& BoxExtent,
		const FVector& ContactPointWS);
	static void SolveOverlap(ISpeedComponent& ThisComp, float MassA, const FMatrix& InvIA, const SKinematic& KA,
		ISpeedComponent* OtherComp, float MassB, const FMatrix& InvIB, const SKinematic& KB,
		const FVector& P, const FVector& N_OtherToThis,
		float PenDepth,
		float SlopCm = 0.05f,
		float Percent = 0.8f,
		float KillVelThreshold = 5.f // cm/s (or ImpactThreshold)
	);
	bool HasToApplyRestForce() const { return bApplyRestForce; }

protected:
	virtual FMatrix InitInvInertiaTensor() const { return FMatrix::Identity; }
	void RegisterCurrentHitAsConstraint();
	void RegisterContactAsConstraint(
		const FVector& ContactPointWS,
		const FVector& NormalWS,
		UPrimitiveComponent* OtherComponent,
		USolidSubBody* OtherSubBody = nullptr,
		float PenetrationDepth = 0.f,
		float TOI = 0.f,
		bool bPersistent = false
	);
	void RegisterContactManifoldAsConstraints(
		const TArray<FVector>& ContactPointsWS,
		const FVector& NormalWS,
		UPrimitiveComponent* OtherComponent,
		float PenetrationDepth = 0.f,
		float TOI = 0.f,
		bool bPersistent = false,
		int32 MaxContacts = 4
	);

	// Mass of the subbody, used for physics simulation and collision resolution. Note that the actual mass used in physics simulation may be different if the parent component implements a custom GetPhysMass() function that returns a different mass for the subbody.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, export, Category = Physics,
		meta = (ClampMin = "0.0", UIMin = "0.0"))
	float Mass = 1.0f;
	// If true, this subbody can update actor's kinematics
	UPROPERTY(EditAnywhere, BlueprintReadOnly, export, Category = Physics)
	bool bIsMainSubBody = false;
	// If true, component owner will apply rest force from subbody constraints
	UPROPERTY(EditAnywhere, BlueprintReadOnly, export, Category = Physics)
	bool bApplyRestForce = true;
	// If true, the subbody will be able to apply custom impulses on itself or other subbodies
	UPROPERTY(EditAnywhere, BlueprintReadOnly, export, Category = Physics)
	bool EnableFakePhysics = false;
	// If true, dynamic subbody contacts register constraints on this subbody's parent component.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, export, Category = Physics)
	bool bEnableConstraintOnSubBodyHit = false;

	// Restitution coefficient for the subbody, used in collision resolution
	UPROPERTY(EditAnywhere, BlueprintReadOnly, export, Category = Physics,
		meta = (ClampMin = "0.0", UIMin = "0.0"))
	float Restitution = 0.0f;
	// Optional restitution override for sphere/box contacts. Negative values keep the normal mix rule.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, export, Category = Physics,
		meta = (ClampMin = "-1.0", UIMin = "-1.0", ClampMax = "1.0", UIMax = "1.0"))
	float SphereBoxRestitutionOverride = -1.0f;
	// Optional contact-specific friction. This lets a vehicle hitbox use the
	// car/ball coefficient without changing ball/stadium contacts.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, export, Category = Physics,
		meta = (ClampMin = "-1.0", UIMin = "-1.0"))
	float SphereBoxFrictionOverride = -1.0f;
	// Opts contacts involving this subbody into the coupled normal/tangent
	// impulse solve. Kept per-body so unrelated collision families are stable.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, export, Category = Physics)
	bool bUseCoupledContactImpulse = false;
	// Resolves the normal impulse first, then evaluates friction from the
	// resulting point velocity. This mirrors Bullet-style sequential contacts
	// without changing the default simultaneous/legacy solvers.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, export, Category = Physics)
	bool bUsePostNormalFrictionImpulse = false;
	// Dynamic contacts between two opted-in solids are maintained by the
	// world-level finite-mass pair solver instead of unilateral projection.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, export, Category = Physics)
	bool bUsePersistentBilateralContact = false;
	// Optional eligibility bounds for this body's coupled contacts. Negative
	// values disable a bound. Useful when a solver model is calibrated only for
	// a known kinematic family.
	float CoupledContactMaxLinearSpeed = -1.0f;
	float CoupledContactMaxAngularSpeed = -1.0f;
	// Optional box-local contact-normal bound. This lets an opt-in box limit a
	// calibrated solver to near-centred impacts without coupling unrelated
	// lateral contacts. Negative values disable the bound.
	float CoupledContactMaxAbsLocalNormalY = -1.0f;
	// Static friction coefficient for the subbody, used in collision resolution
	UPROPERTY(EditAnywhere, BlueprintReadOnly, export, Category = Physics,
		meta = (ClampMin = "0.0", UIMin = "0.0"))
	float StaticFriction = 0.0f;
	// Dynamic friction coefficient for the subbody, used in collision resolution
	UPROPERTY(EditAnywhere, BlueprintReadOnly, export, Category = Physics,
		meta = (ClampMin = "0.0", UIMin = "0.0"))
	float DynamicFriction = 0.0f;
	// Impact threshold for the subbody (in cm/s), used in collision resolution
	UPROPERTY(EditAnywhere, BlueprintReadOnly, export, Category = Physics,
		meta = (ClampMin = "0.0", UIMin = "0.0"))
	float ImpactThreshold = 0.0f;
	// Damping factor applied to the hit impulse, used in collision resolution (1 means no damping, 0 means full damping)
	UPROPERTY(EditAnywhere, BlueprintReadOnly, export, Category = Physics,
		meta = (ClampMin = "0.0", UIMin = "0.0", ClampMax = "1.0", UIMax = "1.0"))
	float HitDamping = 1.0f;

	// diagonal of local inertia tensor inverse, in local space
	FMatrix InvInertiaLocal = FMatrix::Identity;

	// Index of the subbody (for logs)
	int idx = -1;
};
