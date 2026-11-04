// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Components/ShapeComponent.h"
#include "IAmSpeed/Base/SUtils.h"
#include "IAmSpeed/Base/EpisodeShapeFieldsForTesting.h"
#include "SSubBody.generated.h"


#if !UE_BUILD_SHIPPING
namespace Speed { struct FNativeBallMechanicalWitnessForTesting; }
#endif
class ISpeedComponent;
class UBoxSubBody;
class USphereSubBody;
class USWheelSubBody;
class UCylinderSubBody;
namespace Speed { class IStaticCollisionWorld; }
namespace Speed::NativeKinematics { class FBodyRead; class FBodyWrite; }
namespace Speed::Netcode::Prediction {class FPrivateNativeOperation;}
class USpeedBranchBoxSubBody;

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
private:
    friend class Speed::NativeKinematics::FBodyRead;
    friend class Speed::NativeKinematics::FBodyWrite;
    friend class USWheelSubBody;
    friend class USpeedBranchBoxSubBody;
    class FLegacyBodyUpdateAccess;class FNativeBodyUpdateAccess;
    friend class USolidSubBody;
    class FLegacyBirthInitializeAccess;class FNativeBirthInitializeAccess;
    template<class TAccess> void InitializeBirthBody(TAccess& Storage,ISpeedComponent* InParentComponent);
    void InitializeNativeBase(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,ISpeedComponent* Parent);
    virtual void InitializeNative(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,ISpeedComponent* Parent);
    virtual void AcceptNativeHit(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation);
    virtual void ValidateNativeResetFrame(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation) const;
    virtual void ResetNativeForFrame(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const float& Delta);
    virtual void ResolveNativeCurrentHit(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const float& Delta,const float& SimTime);
    template<class TAccess> void UpdateKinematicsFromOwnerBody(TAccess& Storage,const SKinematic& ParentKinematic);
    void UpdateNativeBaseKinematicsFromOwner(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const SKinematic& ParentKinematic);
    virtual void UpdateNativeKinematicsFromOwner(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const SKinematic& ParentKinematic);
    // Custom storage adapters retain the old getter dispatch. Only the private
    // final road bodies override this public-view selector with legacy backing.
    virtual const Speed::FKinematicState& PublicKinematicView() const { return KinematicsStorage(); }
    virtual const SHitResult& PublicCurrentHitView() const { return CurrentHitStorage(); }
    virtual const SHitResult& PublicFutureHitView() const { return FutureHitStorage(); }
    // Default/custom storage adapters retain their previous virtual backing.
    // The final private bodies bind public writes to their actual facade.
    virtual SHitResult& PublicCurrentHitMutableView() { return CurrentHitStorage(); }
    virtual SHitResult& PublicFutureHitMutableView() { return FutureHitStorage(); }
#if !UE_BUILD_SHIPPING
    friend struct Speed::FNativeBallMechanicalWitnessForTesting;
#endif

	
public:
    void OnRegister() override;
#if !UE_BUILD_SHIPPING
    /** Read-only, same-run owner-state witness; no UObject or array-header copy. */
    virtual void AppendEpisodeStateForTesting(TArray<uint8>& Out) const
    {
        using B=Speed::FEpisodeStateBytesForTesting;
        B::Add(Out,ParentComponentBodyDomainStorage());
        const FTransform RelativePose=PhysicalLocalPose();B::Add(Out,RelativePose);
        B::Add(Out,KinematicsStorage());
        B::Add(Out,SubBodyTypeBodyDomainStorage());
        B::Add(Out,CollisionChannelBodyDomainStorage());
        for(int32 Channel=0;Channel<int32(ECC_MAX);++Channel)B::Add(Out,ResponseParamsBodyDomainStorage().CollisionResponse.GetResponse(ECollisionChannel(Channel)));
        B::Add(Out,FutureHitStorage());
        B::Add(Out,CurrentHitStorage());
        B::Add(Out,GroundHitStorage());
        B::Add(Out,BoxHitStorage());
        B::Add(Out,SphereHitStorage());
        B::Add(Out,WheelHitStorage());
        B::Array(Out,AlwaysIgnoredComponentsBodyDomainStorage());
        B::Array(Out,IgnoredComponentsBodyDomainStorage());
        B::Array(Out,IgnoredSubBodyTypesBodyDomainStorage());
        B::Array(Out,ExternalBoxSubBodiesBodyDomainStorage());
        B::Array(Out,ExternalSphereSubBodiesBodyDomainStorage());
        B::Array(Out,ExternalWheelSubBodiesBodyDomainStorage());
        B::Array(Out,ExternalCylinderSubBodiesBodyDomainStorage());
    }
#endif

    enum class ESubBodyType
    {
        Unknown,
        Wheel,
        Hitbox,
        Sphere,
        Cylinder
	};

    virtual void Initialize(ISpeedComponent* InParentComponent);
    virtual bool IsSensor() const { return false; }
	virtual void AddExternalSubBodies(const TArray<USSubBody*>& SubBodies);
	virtual void RemoveExternalSubBodies(const TArray<USSubBody*>& SubBodies);

    // --- Called once per physics frame ---
    virtual void ResetForFrame(const float& Delta);
	/** Shape-specific proof of a feasible resting reaction; unsupported shapes leave the load unchanged. */
	virtual FVector GetStaticRestingReaction(const Speed::IStaticCollisionWorld& World,
		double* StopAfterSeconds = nullptr) const { return FVector::ZeroVector; }
	/** Cheap candidate; the owner must certify support before splitting at it. */
	virtual double GetStaticSupportStopTimeCandidate() const { return TNumericLimits<double>::Max(); }
	/** Continuous normal reaction of already touching features, including torque; no impact impulse. */
	virtual bool TryGetStaticContactAcceleration(FVector& Linear, FVector& Angular) const { return false; }
	/** A genuine later arrival may revisit a pair already resolved in this frame. */
	virtual bool CanResolveRepeatedContact(const SHitResult& Hit) const { return false; }
	// Separate from positive-time repeat eligibility: seed only a resolved exact initial contact.
	virtual bool CanInitializeRepeatedContact(const SHitResult& Hit) const { return false; }
	/** Keeps canonical pose precision while a native exact contact is being solved. */
	virtual bool RequiresUnquantizedContactPose() const { return false; }
	/** A canonical rounding operation is not a physical sweep. Shapes can reject
	 * a proposed owner pose which would create static-world penetration. */
	virtual bool CanApplyQuantizedPose(const Speed::IStaticCollisionWorld& World,
		const SKinematic& OwnerPose) const { return true; }
	/** Shape-specific exact support under a nominal load, independent of transient prepared acceleration. */
	virtual bool HasExactStaticRestingSupport(const Speed::IStaticCollisionWorld& World, const FVector& Load) const { return false; }
	// Enforce an already-established static contact after unconstrained pose
	// prediction. Returns true when owner kinematics changed.
	virtual bool ProjectEstablishedStaticContact(const float& Delta) { return false; }
	// True when the owner must transport this contact through a varying
	// analytical normal during kinematic integration.
	virtual bool RequiresEstablishedStaticContactTransport() const { return false; }
	virtual void PostPhysicsUpdate() {};

    // --- Set/Get kinematics ---
	virtual void UpdateKinematicsFromOwner(const SKinematic& ParentKinematic);
    void SetKinematicState(const Speed::FKinematicState& State) { KinematicsStorage() = State; }
    const Speed::FKinematicState& GetKinematicState() const { return PublicKinematicView(); }

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
    bool HasHit() const { return PublicCurrentHitView().bBlockingHit; }
    bool WillHit() const { return PublicFutureHitView().bBlockingHit; }
    float GetFutureTOI() const { return PublicFutureHitView().TOI; }

	void SetHit(const SHitResult& Hit) { PublicCurrentHitMutableView() = Hit; }
    const SHitResult& GetHit() const { return PublicCurrentHitView(); }
	const SHitResult& GetFutureHit() const { return PublicFutureHitView(); }
	void SetFutureHit(const SHitResult& Hit) { PublicFutureHitMutableView() = Hit; }

	ESubBodyType GetSubBodyType() const { return SubBodyTypeBodyDomainStorage(); }
    FVector GetCenterOfMassWS() const { return KinematicsStorage().Location; }

	ISpeedComponent* GetParentComponent() const { return ParentComponentBodyDomainStorage(); }
    // An identity value only, never mutable physics or lifetime authority.
    // Private born graphs can copy their actual preacquired weak identity.
    virtual TWeakObjectPtr<USSubBody> GetPhysicsWeakIdentity() const;

    // ====== REQUIRED PHYSICS SHAPE AND COLLISION PARAMS ======
    virtual FCollisionShape GetCollisionShape(float Inflation = 0.0f) const { return FCollisionShape::MakeSphere(10.f); }
    ECollisionChannel GetCollisionChannel() const { return CollisionChannelBodyDomainStorage(); }
    const FCollisionResponseParams& GetResponseParams() const { return ResponseParamsBodyDomainStorage(); }

	FVector GetLocalOffset() const { return PhysicalLocalOffset(); }
	FQuat GetLocalRotation() const { return PhysicalLocalRotation(); }
    static uint64 MakePairKey(const USSubBody* A, const UPrimitiveComponent* OtherComp);
    static int32 SubBodyPriority(USSubBody::ESubBodyType T);
    static USSubBody* PickResolver(USSubBody* A, USSubBody* B);
protected:
    /** Clear transient exclusions, retaining bounded scratch and permanent-list order. */
    void ResetIgnoredComponents();
    friend class FIAmSpeedSubBodyIgnoredScratchTest;
    virtual void ResolveCurrentHitPrv(const float& delta, const float& SimTime) {};
    const TArray<TWeakObjectPtr<UBoxSubBody>>& GetExternalBoxSubBodies() const;
    const TArray<TWeakObjectPtr<USphereSubBody>>& GetExternalSphereSubBodies() const;
    const TArray<TWeakObjectPtr<USWheelSubBody>>& GetExternalWheelSubBodies() const;
    const TArray<TWeakObjectPtr<UCylinderSubBody>>& GetExternalCylinderSubBodies() const {return ExternalCylinderSubBodiesBodyDomainStorage();}
protected:

protected:
    virtual FVector PhysicalLocalOffset() const {return GetRelativeLocation();}
    virtual FQuat PhysicalLocalRotation() const {return GetRelativeRotation().Quaternion();}
    // Keep the original Euler input to the exact native rotation cache. An
    // isolated owner must not reread a mutable USceneComponent transform.
    virtual FRotator PhysicalLocalEulerRotation() const {return GetRelativeRotation();}
    virtual FTransform PhysicalLocalPose() const {return GetRelativeTransform();}
    virtual void SetPhysicalLocalOffset(const FVector& V) {SetRelativeLocation(V);}
    virtual void SetPhysicalLocalRotation(const FQuat& V) {SetRelativeRotation(V);}
    virtual ISpeedComponent*& ParentComponentBodyDomainStorage() {return ParentComponent;}
    virtual ISpeedComponent* const& ParentComponentBodyDomainStorage() const {return ParentComponent;}
    virtual ESubBodyType& SubBodyTypeBodyDomainStorage() {return SubBodyType;}
    virtual ESubBodyType const& SubBodyTypeBodyDomainStorage() const {return SubBodyType;}
    virtual ECollisionChannel& CollisionChannelBodyDomainStorage() {return CollisionChannel;}
    virtual ECollisionChannel const& CollisionChannelBodyDomainStorage() const {return CollisionChannel;}
    virtual FCollisionResponseParams& ResponseParamsBodyDomainStorage() {return ResponseParams;}
    virtual FCollisionResponseParams const& ResponseParamsBodyDomainStorage() const {return ResponseParams;}
    virtual TArray<UPrimitiveComponent*>& AlwaysIgnoredComponentsBodyDomainStorage() {return AlwaysIgnoredComponents;}
    virtual TArray<UPrimitiveComponent*> const& AlwaysIgnoredComponentsBodyDomainStorage() const {return AlwaysIgnoredComponents;}
    virtual TArray<UPrimitiveComponent*>& IgnoredComponentsBodyDomainStorage() {return IgnoredComponents;}
    virtual TArray<UPrimitiveComponent*> const& IgnoredComponentsBodyDomainStorage() const {return IgnoredComponents;}
    virtual TArray<ESubBodyType>& IgnoredSubBodyTypesBodyDomainStorage() {return IgnoredSubBodyTypes;}
    virtual TArray<ESubBodyType> const& IgnoredSubBodyTypesBodyDomainStorage() const {return IgnoredSubBodyTypes;}
    virtual TArray<TWeakObjectPtr<UBoxSubBody>>& ExternalBoxSubBodiesBodyDomainStorage() {return ExternalBoxSubBodies;}
    virtual TArray<TWeakObjectPtr<UBoxSubBody>> const& ExternalBoxSubBodiesBodyDomainStorage() const {return ExternalBoxSubBodies;}
    virtual TArray<TWeakObjectPtr<USphereSubBody>>& ExternalSphereSubBodiesBodyDomainStorage() {return ExternalSphereSubBodies;}
    virtual TArray<TWeakObjectPtr<USphereSubBody>> const& ExternalSphereSubBodiesBodyDomainStorage() const {return ExternalSphereSubBodies;}
    virtual TArray<TWeakObjectPtr<USWheelSubBody>>& ExternalWheelSubBodiesBodyDomainStorage() {return ExternalWheelSubBodies;}
    virtual TArray<TWeakObjectPtr<USWheelSubBody>> const& ExternalWheelSubBodiesBodyDomainStorage() const {return ExternalWheelSubBodies;}
    virtual TArray<TWeakObjectPtr<UCylinderSubBody>>& ExternalCylinderSubBodiesBodyDomainStorage() {return ExternalCylinderSubBodies;}
    virtual TArray<TWeakObjectPtr<UCylinderSubBody>> const& ExternalCylinderSubBodiesBodyDomainStorage() const {return ExternalCylinderSubBodies;}
	ISpeedComponent* ParentComponent;

    // Kinematic state at start of frame or during substep
    // Lifetime-invariant subtypes can route these value blocks to an owner.
    // Default adapters retain exactly their legacy fields. No restore or
    // geometry/configuration/lifetime completeness is granted by these hooks.
    virtual Speed::FKinematicState& KinematicsStorage() {return Kinematics;}
    virtual Speed::FKinematicState const& KinematicsStorage() const {return Kinematics;}
    virtual SHitResult& FutureHitStorage() {return FutureHit;}
    virtual SHitResult const& FutureHitStorage() const {return FutureHit;}
    virtual SHitResult& CurrentHitStorage() {return CurrentHit;}
    virtual SHitResult const& CurrentHitStorage() const {return CurrentHit;}
    virtual SHitResult& GroundHitStorage() {return GroundHit;}
    virtual SHitResult const& GroundHitStorage() const {return GroundHit;}
    virtual SHitResult& BoxHitStorage() {return BoxHit;}
    virtual SHitResult const& BoxHitStorage() const {return BoxHit;}
    virtual SHitResult& SphereHitStorage() {return SphereHit;}
    virtual SHitResult const& SphereHitStorage() const {return SphereHit;}
    virtual SHitResult& WheelHitStorage() {return WheelHit;}
    virtual SHitResult const& WheelHitStorage() const {return WheelHit;}
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
    TArray<TWeakObjectPtr<UCylinderSubBody>> ExternalCylinderSubBodies;

    // Internal sweep helper
    bool InternalSweep(const FVector& Start, const FVector& End, SHitResult& OutHit, const float& delta);

    virtual FCollisionQueryParams BuildTraceParams() const;
};
