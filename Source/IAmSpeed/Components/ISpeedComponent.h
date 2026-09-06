#pragma once

#include "CoreMinimal.h"
#include "IAmSpeed/Base/PhysicalContactConstraint.h"
#include "IAmSpeed/Base/SHitResult.h"
#include "IAmSpeed/Base/Kinematic.h"
#include "IAmSpeed/Base/KinematicQuantizationCache.h"
#include "IAmSpeed/World/Simulation/SimulationSleepState.h"

class USSubBody;
class USolidSubBody;
struct SubBodyConfig;
struct FCanonicalFrameContext;
namespace Speed { class IStaticCollisionWorld; }

#if !UE_BUILD_SHIPPING
/** Diagnostic rejection stage; separate from a shape's geometric support certificate. */
enum class EStaticRestingReactionStatus : uint8
{
	NoWorld, Frozen, Moving, Spinning, AngularLoad, NoLoad, NoSupport, Supported
};
#endif

struct SComponentTOI
{
	bool bHit = false;
	float TOI = 0.f;
	TWeakObjectPtr<USSubBody> Resolver;     // SubBody that will resolve the hit at TOI
	SHitResult Hit;                         // Chosen hit result at TOI
	uint64 PairKey = 0;                     // Optional: anti-double resolution
};

/**
 * ISpeedComponent : Interface for components that own USSubBodies. It allows sub-bodies to query info about their parent component
 * Add this interface to any component that owns USSubBodies
 * (e.g. a car body component that owns wheel sub-bodies or hitbox) and implement
 * the GetRadiusFromSubBody function to return the appropriate radius for each sub-body (e.g. wheel radius)
 */
class IAMSPEED_API ISpeedComponent
{
public:
	// Runs component-owned preparation/gameplay for one integer-addressed frame.
	// This virtual dispatch also covers derived components declared by game modules.
	virtual void PrepareCanonicalFrame(const FCanonicalFrameContext& Context) = 0;
	/** Applies an opaque, frame-latched input payload addressed by FSimulationWorld. */
	virtual bool ApplySimulationInput(TConstArrayView<uint8> Payload) { return false; }
	/** Appends deterministic mechanic state not covered by the common kinematic snapshot. */
	virtual void AppendSimulationSnapshot(TArray<uint8>& OutPayload) const {}
	/** Validates component-specific bytes before an atomic world restore starts. */
	virtual bool CanRestoreSimulationSnapshot(TConstArrayView<uint8> Payload) const
	{
		return Payload.IsEmpty();
	}
	/** Restores the common rigid state, then delegates only component-specific bytes. */
	void RestoreSimulationSnapshot(
		const SKinematic& KinematicState,
		bool bFrozen,
		TConstArrayView<uint8> MechanicPayload);
	// Optional simulation-owned run control. A test/controller seals its complete
	// frame-addressed scenario before FastSimulation starts and marks completion
	// from the canonical lane after publishing the terminal result.
	virtual bool IsCanonicalRunController() const { return false; }
	virtual bool IsCanonicalRunReady() const { return false; }
	virtual bool IsCanonicalRunComplete() const { return false; }
	// overload this function to return the number of frames that have passed since the start of simulation
	virtual unsigned int NumFrame() const = 0;
	// overload this function to return the mass of the component (e.g. for a car body, this would be the mass of the car body without the wheels)
	virtual float GetPhysMass() const = 0;
	// Physical center of mass in world space. The canonical kinematic state is
	// stored at this point and can therefore be read without reconstruction.
	virtual const FVector& GetPhysCOM() const = 0;
	const FVector& GetPhysCOMLocation() const { return GetPhysCOM(); }
	// Center of mass relative to the component/mesh origin. This geometry is
	// fixed for the lifetime of a physics state and is used to derive origin data.
	virtual const FVector& GetPhysCenterOfMassLocal() const = 0;
	// overload this function to return the array of sub-bodies owned by this component
	virtual const TArray<USSubBody*>& GetSubBodies() const = 0;
	// overload this function to create the sub-bodies owned by this component (e.g. for a car body, this would be creating the wheel sub-bodies and the hitbox sub-body)
	virtual TArray<USSubBody*> CreateSubBodies() = 0;

	// Returns true if the movement is currently frozen (e.g. due to the game being paused)
	virtual bool IsFrozen() const = 0;
	/** True only for an unchanged, force-free pose on certified stable support. */
	bool IsPhysicsSleeping() const;
	// Freezes the Movement. It serves when we want to pause the game
	void FreezeMovement();
	// Unfreezes the Movement. It serves when we want to resume the game after being paused
	void UnFreezeMovement();

	// ==================== SubBody-specific info retrieval functions ====================
	virtual SubBodyConfig GetSubBodyConfig(const USSubBody& SubBody) const = 0;
	// A fresh principal-body contact can belong to a static surface on which a
	// subordinate support manifold is already established.  Components without
	// subordinate supports keep the ordinary free-impact response.
	virtual bool HasCompatibleEstablishedStaticSupport(
		const SHitResult& SurfaceHit) const { return false; }
	//===================================================================================

	// overload this function to return the appropriate kinematic state for SubBody (e.g. wheel kinematics or hitbox kinematics from car body)
	virtual SKinematic GetKinematicsOfSubBody(const USSubBody& SubBody, const unsigned int& NumFrame) const = 0;
	// overload this function to return the appropriate inertia tensor in world space
	virtual FMatrix ComputeWorldInvInertiaTensor() const = 0;
	// overload this function to return the appropriate inertia tensor in world space for the given sub-body (e.g. for a car body, the inertia tensor of the wheel sub-body would be different from the inertia tensor of the hitbox sub-body)
	virtual FMatrix ComputeWorldInvInertiaTensorOfSubBody(const USSubBody& SubBody) const = 0;

	// =========== Kinematics getters and setters =================
	virtual const SKinematic& GetKinematicState() const = 0;
	virtual const SKinematic& GetKinematicStateForFrame(const unsigned int& NumFrame) const = 0;
	// Derives the component/mesh-origin state for rendering and local sub-body geometry.
	SKinematic GetOriginKinematicState() const;
	SKinematic GetOriginKinematicStateForFrame(const unsigned int& NumFrame) const;
	/** Derives only origin position, retaining the same current-frame history lookup as the full state. */
	FVector GetPhysLocation() const;
	void SetPhysCOMLocation(const FVector& NewCOMLocation);
	void SetPhysLocation(const FVector& NewLocation);
	const FQuat& GetPhysRotation() const;
	void SetPhysRotation(const FQuat& NewRotation);
	// Applies an instantaneous orientation correction without teleporting the
	// rigid body's center of mass or changing its COM velocity.
	void SetPhysRotationPreserveCOM(const FQuat& NewRotation);
	/** Derives only origin velocity (COM velocity minus angular transport), without rebuilding acceleration. */
	FVector GetPhysVelocity() const;
	FVector GetPhysVelocityAtPoint(const FVector& Point) const;
	// The stored kinematic velocity is attached to the component origin.
	// Use this when gameplay needs the rigid body's center-of-mass velocity.
	const FVector& GetPhysCOMVelocity() const;
	const FVector& GetPhysAngularVelocity() const;
	virtual float GetPhysMaxSpeed() const = 0;
	virtual float GetPhysMaxAngularSpeed() const = 0;
	// Sets the rigid body's center-of-mass velocity and clamps that physical
	// velocity, while keeping the stored component-origin velocity coherent.
	void SetPhysCOMVelocity(const FVector& NewCOMVelocity);
	// Sets the velocity of the stored component origin. The speed limit is
	// nevertheless evaluated from the corresponding COM velocity.
	void SetPhysVelocity(const FVector& NewVelocity);
	void SetPhysVelocityAtPoint(const FVector& NewVelocity, const FVector& WorldPoint);
	void SetPhysAngularVelocity(const FVector& NewAngularVelocity);
	const FVector& GetPhysAcceleration() const;
	const FVector& GetPhysAngularAcceleration() const;
	void SetPhysAcceleration(const FVector& NewAcceleration);
	void SetPhysAngularAcceleration(const FVector& NewAngularAcceleration);
	FVector GetPhysAccelerationAtPoint(const FVector& Point) const;
	// =============================================================

	// ====== Kinematics Utils ======
	FVector GetPhysForwardVector() const { return GetPhysRotation().GetForwardVector(); }
	FVector GetPhysRightVector() const { return GetPhysRotation().GetRightVector(); }
	FVector GetPhysUpVector() const { return GetPhysRotation().GetUpVector(); }
	float GetPhysForwardSpeed() const { return FVector::DotProduct(GetPhysCOMVelocity(), GetPhysForwardVector()); }
	bool IsPhysGoingForward() const { return GetPhysForwardSpeed() > 0; }
	// ==============================

	// ===== Kinematics modifiers (e.g. for applying impulses) =======
	FVector AddPhysLocation(const FVector& DeltaLocation);
	FQuat AddPhysRotation(const FQuat& DeltaRotation);
	void AddPhysVelocity(const FVector& DeltaVelocity);
	void AddPhysAngularVelocity(const FVector& DeltaAngularVelocity);
	void AddPhysImpulseAtPoint(const FVector& Impulse, const FVector& WorldPoint, const USolidSubBody* SubBody = nullptr);
	void AddPhysImpulseBatchAtPoints(const TArray<FVector>& Impulses,
		const TArray<FVector>& WorldPoints, const USolidSubBody* SubBody = nullptr);
	void AddPhysAcceleration(const FVector& DeltaAcceleration);
	void AddPhysAngularAcceleration(const FVector& DeltaAngularAcceleration);
	void AddPhysAngularAccelerationLocal(const FVector& LocalAngularAccel);
	void AddPhysForceAtPoint(const FVector& Force, const FVector& WorldPoint, const USolidSubBody* SubBody = nullptr);
	void AddPhysForceBatchAtPoints(const TArray<FVector>& Forces,
		const TArray<FVector>& WorldPoints, const USolidSubBody* SubBody = nullptr);
	// overload this function to apply an impulse to the parent component at the given world point (e.g. for hitboxes to apply impulse to car body)
	virtual void ApplyImpulse(const FVector& LinearImpulse, const FVector& WorldPoint, const USolidSubBody* SubBody = nullptr);
	// ================================================================

	// This is used to advance the state to the time of impact after a hit is detected
	void IntegrateKinematics(const float& SubDelta);
	// Make already-established static contacts feasible without advancing time.
	// This is also run at the canonical frame boundary so a contact acquired at
	// the final TOI cannot expose a penetrating pose to observers.
	bool ProjectEstablishedStaticContacts(const float& Delta);
	void UpdateSubBodiesKinematics();
	/** Binds immutable query geometry for this frame; cleared at completion/restore, never snapshotted. */
	void SetStaticCollisionWorldForFrame(const Speed::IStaticCollisionWorld* World, float FrameHorizon = 0)
	{
		StaticRestingWorld = World;
		StaticSupportFrameHorizon = World ? FrameHorizon : 0;
	}
	/** Common conservative horizon for support admission in CCD and integration.
	 * It must not shorten just because the sweep returned an earlier impact. */
	float GetStaticSupportFrameHorizon() const { return StaticSupportFrameHorizon; }
	/** Returns a currently feasible resting reaction, or zero; does not mutate physical state. */
	FVector GetStaticRestingReaction(double* StopAfterSeconds = nullptr) const;
	/** True when certified support keeps this whole interval below the speed cap;
	 * do not cap a fictitious free-fall velocity before applying that reaction. */
	bool IsSpeedLimitPreservedByExactSupport(float Delta) const;
	/** Evaluates a shape's continuous contact force/torque without caching it across events. */
	void GetStaticContactAcceleration(FVector& Linear, FVector& Angular) const;
	/** Splits CCD at a certified material stop event, before friction could reverse slip. */
	float GetMaximumCanonicalSupportInterval(float Remaining) const;
	/** Nominal configured load used to certify support geometry, not a prepared frame force. */
	virtual FVector GetNominalGravityAcceleration() const { return FVector::ZeroVector; }
	/** Certifies the current stationary pose; never relies on a previous contact or sleeping state. */
	bool HasExactStaticRestingSupport() const;
	/** A stationary orientation may translate tangentially when its shape/material
	 * certifies a torque-free support law; this is not complete mechanical rest. */
	bool HasExactStaticFaceSupport() const;
	/** Frame-local immutable geometry, also available during preparation and canonical quantization. */
	const Speed::IStaticCollisionWorld* GetStaticCollisionWorldForFrame() const { return StaticRestingWorld; }

	// Called once per physics frame to reset any cached info in the component or its sub-bodies (e.g. hit info)
	virtual void ResetForFrame(const float& Delta);
	// overload this function to sweep all sub-bodies for the remaining delta time and return the earliest time of impact and the sub-body that should resolve it
	// (e.g. for a car body, if a wheel hits before the hitbox, then the wheel sub-body should resolve first)
	SComponentTOI SweepTOISubBodies(const float& RemainingDelta, const float& LastSubDelta);
	// overload this function for the component to perform any necessary updates after the physics state has been updated
	void PostPhysicsUpdate(const float& delta);
	// overload this function to set whether the component is upside down (e.g. for a car body, this would be whether the car is flipped over)
	virtual void SetIsUpsideDown(bool bUpsideDown) = 0;
	// overload this function to return whether the component is upside down (e.g. for a car body, this would be whether the car is flipped over)
	virtual bool IsUpsideDown() const = 0;
	// overload this function to know whether the main SubBody is in auto-recover mode (e.g. for a car body, this would be whether the car is trying to right itself after being flipped over)
	virtual bool IsInAutoRecover() const = 0;
	// overload this function to return whether the sub-bodies are in auto-recover mode (e.g. for a car body, this would be whether the wheels are trying to right themselves after being flipped over)
	virtual bool IsSubBodyInAutoRecoverMode() const = 0;
	virtual void RcvImpactOnSubBody(const USSubBody& SubBody, const FVector& Location) = 0;

	void AddExternalSubBodies(const TArray<USSubBody*>& ExtSubBodies);
	void RemoveExternalSubBodies(const TArray<USSubBody*>& ExtSubBodies);

	void RegisterPhysicalConstraint(const FPhysicalContactConstraint& Constraint);
	void ClearPhysicalConstraints();
	const TArray<FPhysicalContactConstraint>& GetPhysicalConstraints() const { return PhysicalConstraints; }
	/** True when another sub-body supplies a live constraint: an isolated
	 * rigid-body contact response must then defer to the coupled solver. */
	virtual bool HasActivePhysicalConstraintsOtherThan(const USSubBody* Source) const;

	virtual ~ISpeedComponent() = default;
protected:
#if !UE_BUILD_SHIPPING
	/** Opt-in observer of actual reaction evaluations; no diagnostic dispatch in normal runtime. */
	virtual void ObserveStaticRestingReaction(EStaticRestingReactionStatus Status) const {}
	bool bObserveStaticRestingReaction = false;
#endif
	/** Quantizes in place unless the exact state/reference was already proved a no-op. */
	void QuantizeKinematicState(SKinematic& State, const FQuat& PreviousRotation);
	/** Opt-in support certificate; unsupported component kinds remain awake. */
	virtual bool HasStableSleepSupport() const { return false; }
	// overload this function to advance the physics state of the parent component by `SubDelta` seconds
	// SubDelta is a fraction of delta during which there is NO collision, so the physics state should be advanced by SubDelta seconds without checking for collision.
	// This is used to advance the state to the time of impact after a hit is detected
	void IntegrateKinematicsPrv(const float& SubDelta);
	// overload this function to perform any necessary updates after the physics state has been updated for SubDelta seconds (e.g. for updating the kinematic state of sub-bodies based on the new physics state of the parent component)
	virtual void PostIntegrateKinematics(const float& SubDelta) {};

	// overload this function for the component to perform any necessary updates after the physics state has been updated
	virtual void PostPhysicsUpdatePrv(const float& delta) = 0;

	virtual void SetKinematicState(const SKinematic& NewKinematicState) = 0;
	void SetPhysCOMVelocityRaw(const FVector& NewVelocity);
	void SetPhysAngularVelocityRaw(const FVector& NewAngularVelocity);
	bool IsPhysicalConstraintStillRelevant(const FPhysicalContactConstraint& Constraint) const;
	void ProjectLinearDeltaAgainstConstraints(FVector& DeltaLinear) const;
	void ProjectAngularDeltaAgainstConstraints(FVector& DeltaAngular) const;
	void ProjectPointDeltaAgainstConstraints(FVector& DeltaLinear, FVector& DeltaAngular) const;
	static bool AreSimilarPhysicalConstraints(const FPhysicalContactConstraint& A, const FPhysicalContactConstraint& B);

	virtual void SetIsFrozen(bool bFrozen) = 0;
	/** Applies bytes previously emitted by AppendSimulationSnapshot after validation. */
	virtual void RestoreSimulationSnapshotPayload(TConstArrayView<uint8> Payload)
	{
		check(Payload.IsEmpty());
	}

	TArray<FPhysicalContactConstraint> PhysicalConstraints;
	Speed::FSimulationSleepState SleepState;
	Speed::FIdentityKinematicQuantizationCache KinematicQuantizationCache;
private:
	// Borrowed only during the world's canonical step; no historical/cache state.
	const Speed::IStaticCollisionWorld* StaticRestingWorld = nullptr;
	float StaticSupportFrameHorizon = 0;
};
