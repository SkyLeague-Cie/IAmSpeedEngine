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
	float GetStaticFriction() const { return StaticFriction; }
	float GetDynamicFriction() const { return DynamicFriction; }
	float GetImpactThreshold() const { return ImpactThreshold; }
	float GetHitDamping() const { return HitDamping; }

	// Override this function to apply custom impulses on the subbody itself when a hit is resolved.
	virtual void ApplyImpulse(const FVector& LinearImpulse, const FVector& WorldPoint);
	// Override this function to apply custom impulses on the subbody itself or other subbodies when a hit is resolved.
	virtual void ApplyFakePhysicsOn(USolidSubBody& OtherSubBody, const SHitResult& Hit, const float& DeltaTime) {};
	virtual FVector GetVelocityAtPoint(const FVector& Point) const;

	static float MixRestitution(float eA, float eB, EMixMode Mode);
	static float MixFriction(float muA, float muB, EMixMode Mode);
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

	// Restitution coefficient for the subbody, used in collision resolution
	UPROPERTY(EditAnywhere, BlueprintReadOnly, export, Category = Physics,
		meta = (ClampMin = "0.0", UIMin = "0.0"))
	float Restitution = 0.0f;
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
