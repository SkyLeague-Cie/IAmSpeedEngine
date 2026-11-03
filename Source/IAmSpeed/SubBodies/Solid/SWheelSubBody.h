// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "SphereSubBody.h"
#include "IAmSpeed/Base/EpisodeStateBytesForTesting.h"
#include "SWheelSubBody.generated.h"

namespace Speed::Analytic { struct FAnalyticPrefetchQuery1075; }
class ISpeedWheeledComponent;
namespace Speed::NativeKinematics { class FWheelBindingAccess; }
class UChaosVehicleWheel;
namespace Chaos
{
    class FSimpleWheelSim;
    class FSimpleSuspensionSim;
}

struct SWheelRenderData
{
    FQuat RollRotation = FQuat::Identity;      // Roll rotation (around Right)
    FQuat SteerRotation = FQuat::Identity;     // Steer rotation (around Up)
    FVector WorldPosition = FVector::ZeroVector;   // Final position of the wheel center
    FVector RightAxis = FVector::RightVector;       // Wheel right axis in world space
    FVector UpAxis = FVector::UpVector;
    FVector ForwardAxis = FVector::ForwardVector;
    float SpringOffset = 0.0f;      // Visual suspension offset
};

DECLARE_LOG_CATEGORY_EXTERN(WheelSubBodyLog, Log, All);

/**
 * 
 */
UCLASS()
class IAMSPEED_API USWheelSubBody : public USphereSubBody
{
    GENERATED_UCLASS_BODY()
private:
    friend class Speed::NativeKinematics::FWheelBindingAccess;
    void SetChaosWheelForPhysics(UChaosVehicleWheel* InChaosWheel);
    void SetWheelSimForPhysics(Chaos::FSimpleWheelSim* InPWheel);
    void SetSuspensionSimForPhysics(Chaos::FSimpleSuspensionSim* InPSuspension);
    virtual void ApplyPublicChaosWheel(UChaosVehicleWheel* Value) { SetChaosWheelForPhysics(Value); }
    virtual void ApplyPublicWheelSim(Chaos::FSimpleWheelSim* Value) { SetWheelSimForPhysics(Value); }
    virtual void ApplyPublicSuspensionSim(Chaos::FSimpleSuspensionSim* Value) { SetSuspensionSimForPhysics(Value); }
    virtual const UChaosVehicleWheel* PublicChaosWheelView() const { return ChaosWheelWheelDomainStorage(); }
    virtual const Chaos::FSimpleWheelSim* PublicWheelSimView() const { return PWheelWheelDomainStorage(); }
    virtual const Chaos::FSimpleSuspensionSim* PublicSuspensionSimView() const { return PSuspensionWheelDomainStorage(); }

#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
    friend class FSLBallEpisodeAdmissionOwnerTest; // sole native fixture mutates omitted-field categories
#endif
	
public:
    static constexpr float DefaultWheelMass=18.0f; // native constructor policy, shared with isolated owners

#if !UE_BUILD_SHIPPING
    /** Read-only, same-run owner-state witness; no UObject or array-header copy. */
    void AppendEpisodeStateForTesting(TArray<uint8>& Out) const override
    {
        USphereSubBody::AppendEpisodeStateForTesting(Out);
        using B=Speed::FEpisodeStateBytesForTesting;
        B::Add(Out,WheelComponentWheelDomainStorage());
        B::Add(Out,ChaosWheelWheelDomainStorage());
        B::Add(Out,PWheelWheelDomainStorage());
        B::Add(Out,PSuspensionWheelDomainStorage());
        B::Add(Out,WheelMassWheelDomainStorage());
        B::Add(Out,SuspensionSpringRateWheelDomainStorage());
        B::Add(Out,SuspensionDampingCompressionRatioWheelDomainStorage());
        B::Add(Out,SuspensionDampingReboundRatioWheelDomainStorage());
        B::Add(Out,bUseSuspensionForceModelOverrideWheelDomainStorage());
        B::Add(Out,bClampSuspensionForceToPositiveWheelDomainStorage());
        B::Add(Out,bClampPositiveUntilFirstCompressionWheelDomainStorage());
        B::Add(Out,bUseEffectiveSuspensionSweepRadiusWheelDomainStorage());
        B::Add(Out,bUseIncreasingDisplacementAsCompressionWheelDomainStorage());
        B::Add(Out,bUseCompressionCrossingContactImpulseWheelDomainStorage());
        B::Add(Out,GroundForceApplicationModeWheelDomainStorage());
        B::Add(Out,ContactImpulseApplicationModeWheelDomainStorage());
        B::Add(Out,SuspensionStiffnessForceScaleWheelDomainStorage());
        B::Add(Out,SuspensionStiffnessReferenceDisplacementWheelDomainStorage());
        B::Add(Out,SuspensionPreRestCompressionDampingScaleWheelDomainStorage());
        B::Add(Out,bClampNegativeSuspensionDisplacementWheelDomainStorage());
        B::Add(Out,bDisableSuspensionBumpStopForceWheelDomainStorage());
        B::Add(Out,SuspensionSpringRateCmOverrideWheelDomainStorage());
        B::Add(Out,SuspensionCompressionDampingOverrideWheelDomainStorage());
        B::Add(Out,SuspensionReboundDampingOverrideWheelDomainStorage());
        B::Add(Out,SuspensionMaxRaiseOverrideWheelDomainStorage());
        B::Add(Out,SuspensionMaxDropOverrideWheelDomainStorage());
        B::Add(Out,ConfiguredStaticSpringCompressionWheelDomainStorage());
        B::Add(Out,SuspensionForceRuntimeStorage());
        B::Add(Out,bIsJumpingRuntimeStorage());
        B::Add(Out,bJumpUnilateralSupportRuntimeStorage());
        B::Add(Out,ConsecutiveGroundFramesRuntimeStorage());
        B::Add(Out,SteeringAngleRuntimeStorage());
        B::Add(Out,OmegaRuntimeStorage());
        B::Add(Out,RollAngleRuntimeStorage());
        B::Add(Out,bWasOnGroundPrevFrameRuntimeStorage());
        B::Add(Out,bCountedGroundFrameRuntimeStorage());
        B::Add(Out,bContactVelocityLockedRuntimeStorage());
        B::Add(Out,bCrossedIntoCompressionThisFrameRuntimeStorage());
        B::Add(Out,ForwardAxisRuntimeStorage());
        B::Add(Out,RightAxisRuntimeStorage());
        B::Add(Out,UpAxisRuntimeStorage());
        B::Add(Out,RenderDataRuntimeStorage().RollRotation);B::Add(Out,RenderDataRuntimeStorage().SteerRotation);B::Add(Out,RenderDataRuntimeStorage().WorldPosition);B::Add(Out,RenderDataRuntimeStorage().RightAxis);B::Add(Out,RenderDataRuntimeStorage().UpAxis);B::Add(Out,RenderDataRuntimeStorage().ForwardAxis);B::Add(Out,RenderDataRuntimeStorage().SpringOffset);
    }
#endif

    // initializer methods
    virtual void Initialize(ISpeedComponent* InParentComponent) override;
    bool TryGetPrescribedSupportVelocity(const FVector& Point,FVector& Velocity) const;

    bool IsOnGround() const; // true if the wheel IS currently on the ground
    // Classify the current stored contact only when it has a live Component.
    // Missing identity/Stationary mobility is unknown: both helpers return false.
    // These queries do not certify contact freshness or a restored support's origin.
    bool IsOnStaticObject() const;
    bool IsOnDynamicObject() const;
    void SetOnGround(const bool& on_ground);
    void SetIsJumping(const uint8 NbFrames);
    uint8 IsJumping() const;
    void SetJumpUnilateralSupport(bool bEnabled);
    bool HasJumpUnilateralSupport() const;
    uint8 GetConsecutiveGroundFrames() const;
    void SetConsecutiveGroundFrames(uint8 Frames);

    // --- overrides SubBody behaviour ---
    virtual void ResetForFrame(const float& Delta) override;
    virtual void UpdateKinematicsFromOwner(const SKinematic& ParentKinematic) override;
    void AcceptHit() override;
    virtual bool SweepTOI(const float& RemainingDelta, float& OutTOI) override;
    virtual SSphere MakeSphere() const override;
    virtual FCollisionShape GetCollisionShape(float Inflation = 0.0f) const override;
    SKinematic GetKinematicsFromOwner(const unsigned int& NumFrame) const;
    // SSphere MakeSphere(const unsigned int& NumFrame, const float& RemainingDelta, const float& TimePassed) const override;
    void ApplyImpulse(const FVector& LinearImpulse, const FVector& WorldPoint) override;

    // --- steering methods ---
    bool IsSteeringEnabled() const;
    float MaxSteeringAngle() const;
    void UpdateSteerAngle(const float& delta);
    void UpdatePhysicsState(const float& delta);

    // --- suspension methods ---
    virtual void UpdateSuspension(const float& delta);
    // sweep for suspension
    virtual void SweepSuspension(const float& delta);
    // Unknown exact types must explicitly certify that the inherited query
    // construction remains pure; certification never executes on a worker.
    virtual bool IsGroundQueryCaptureCertified1075() const
    { return GetClass() == USWheelSubBody::StaticClass(); }
    bool TryCaptureGroundQuery1075(float Delta, Speed::Analytic::FAnalyticPrefetchQuery1075& Out) const;
	bool ProbeSuspensionOnGround(SHitResult& OutHit, float Delta) const;
	void GetSuspensionSweepSegment(float Delta, FVector& OutStart, FVector& OutEnd) const;
	virtual bool SweepSuspensionAlongNormal(
		const FVector& Normal, float SearchDistance, float Delta, SHitResult& OutHit) const;

    // Utils methods
    FVector WorldPos() const;
    FVector SteeringPos() const;
    FVector WorldPosFromCarTransform(const FTransform& CarTransform) const;
    FVector GetSuspensionDirectionWS() const;
    float Radius() const;
    float ChaosEffectiveRadius() const;
    const UChaosVehicleWheel* GetChaosWheelConfiguration() const { return PublicChaosWheelView(); }
    float AngularVelocity() const;
    void SetAngularVelocity(const float& InOmega);
    float SpringDisplacement() const; // current spring displacement (positive when compressed)
    float ContactSpringDisplacement() const;
    float ContactSpringDisplacement(const SHitResult& ContactHit) const;
    float GetLastDisplacement() const;
    void SetLastDisplacement(const float& displacement);
    float SpringLength() const;
    float MaxLength() const; // max suspension length (rest length + max drop)
    float SuspensionRestLength() const;
    float SuspensionMaxRaise() const;
    float SuspensionMaxDrop() const;
    // Vehicle layers may replace serialized Chaos travel before SetupVehicle.
    // IAmSpeed keeps no game-specific policy for choosing these limits.
    void ConfigureSuspensionTravel(float MaxRaise, float MaxDrop);
    bool HasConfiguredSuspensionTravel() const;
    bool IsAtSuspensionBumpStop() const;
    float SuspensionSpringRateCm() const;
    /** Stores the equilibrium compression derived while the immutable suspension setup is built. */
    void ConfigureStaticSpringCompression(float RestingForce);
    /** Returns configuration data only; it never samples the live Chaos suspension state. */
    float StaticSpringCompression() const;
    float SuspensionCompressionDamping() const;
    float SuspensionReboundDamping() const;
    bool UsesDirectSuspensionDamping() const;
    bool ClampsSuspensionForceToPositive() const;
    void ConfigureSuspensionForceModel(float SpringRateCm, float CompressionDamping,
		float ReboundDamping, bool bClampForceToPositive);
    void SetUseIncreasingDisplacementAsCompression(bool bEnabled);
    void SetClampPositiveUntilFirstCompression(bool bEnabled);
    void SetUseEffectiveSuspensionSweepRadius(bool bEnabled);
    void SetUseCompressionCrossingContactImpulse(bool bEnabled);
	void SetGroundForceApplicationMode(uint8 Mode);
	void SetContactImpulseApplicationMode(uint8 Mode);
	FVector GetSuspensionForceApplicationPoint() const;
	FVector GetContactImpulseApplicationPoint() const;
    void ConfigureSuspensionForceBehavior(float StiffnessScale,
        float StiffnessReferenceDisplacement,
        float PreRestCompressionDampingScale, bool bClampNegativeDisplacement,
        bool bDisableBumpStopForce);
    float GetSuspensionDampingReboundRatio() const;
    float GetSuspensionDampingCompressionRatio() const;
    float GetSuspensionForce() const;
    bool IsContactVelocityLocked() const;
    bool WasOnGroundPreviousFrame() const;
    float GetSuspensionOffset() const;
    FVector GetHitContactNormal() const;
    FVector GetHitContactPoint() const;
    void SetHitContactNormal(const FVector& ImpactNormal);
    const SWheelRenderData& GetRenderData() const;
    void SetRollAngle(const float& angle);
    float GetRollAngle() const;

	void SetChaosWheel(UChaosVehicleWheel* InChaosWheel);
	void SetWheelSim(Chaos::FSimpleWheelSim* InPWheel);
	void SetSuspensionSim(Chaos::FSimpleSuspensionSim* InPSuspension);
	bool HasSuspensionSim() const { return PublicSuspensionSimView() != nullptr; }
	const Chaos::FSimpleWheelSim* GetWheelSim() const { return PublicWheelSimView(); }
	const Chaos::FSimpleSuspensionSim* GetSuspensionSim() const { return PublicSuspensionSimView(); }
	void SetLocalOffset(const FVector& InLocalOffset);

    void HandleTimers();
protected:
    virtual bool SweepSuspensionOnGround(SHitResult& OutHit, const float& delta);
    virtual bool SweepSuspensionOnSpheres(SHitResult& OutHit, const float& delta);
    virtual bool SweepSuspensionOnBoxes(SHitResult& OutHit, const float& delta);
    virtual bool SweepSuspensionOnCylinders(SHitResult& OutHit,const float& Delta);

    // Predicts spring displacement / wheel travel
    float PredictNextDisplacement(const float& Delta) const;
    float ComputeNextAirLength(const float& DeltaTime) const;
    void UpdateContactVelocityLock();
    void ResetContactVelocityLock();

protected:
protected:
    virtual ISpeedWheeledComponent*& WheelComponentWheelDomainStorage() {return WheelComponent;}
    virtual ISpeedWheeledComponent* const& WheelComponentWheelDomainStorage() const {return WheelComponent;}
    virtual UChaosVehicleWheel*& ChaosWheelWheelDomainStorage() {return ChaosWheel;}
    virtual UChaosVehicleWheel* const& ChaosWheelWheelDomainStorage() const {return ChaosWheel;}
    virtual Chaos::FSimpleWheelSim*& PWheelWheelDomainStorage() {return PWheel;}
    virtual Chaos::FSimpleWheelSim* const& PWheelWheelDomainStorage() const {return PWheel;}
    virtual Chaos::FSimpleSuspensionSim*& PSuspensionWheelDomainStorage() {return PSuspension;}
    virtual Chaos::FSimpleSuspensionSim* const& PSuspensionWheelDomainStorage() const {return PSuspension;}
    virtual float& WheelMassWheelDomainStorage() {return WheelMass;}
    virtual float const& WheelMassWheelDomainStorage() const {return WheelMass;}
    virtual float& SuspensionSpringRateWheelDomainStorage() {return SuspensionSpringRate;}
    virtual float const& SuspensionSpringRateWheelDomainStorage() const {return SuspensionSpringRate;}
    virtual float& SuspensionDampingCompressionRatioWheelDomainStorage() {return SuspensionDampingCompressionRatio;}
    virtual float const& SuspensionDampingCompressionRatioWheelDomainStorage() const {return SuspensionDampingCompressionRatio;}
    virtual float& SuspensionDampingReboundRatioWheelDomainStorage() {return SuspensionDampingReboundRatio;}
    virtual float const& SuspensionDampingReboundRatioWheelDomainStorage() const {return SuspensionDampingReboundRatio;}
    virtual bool& bUseSuspensionForceModelOverrideWheelDomainStorage() {return bUseSuspensionForceModelOverride;}
    virtual bool const& bUseSuspensionForceModelOverrideWheelDomainStorage() const {return bUseSuspensionForceModelOverride;}
    virtual bool& bClampSuspensionForceToPositiveWheelDomainStorage() {return bClampSuspensionForceToPositive;}
    virtual bool const& bClampSuspensionForceToPositiveWheelDomainStorage() const {return bClampSuspensionForceToPositive;}
    virtual bool& bClampPositiveUntilFirstCompressionWheelDomainStorage() {return bClampPositiveUntilFirstCompression;}
    virtual bool const& bClampPositiveUntilFirstCompressionWheelDomainStorage() const {return bClampPositiveUntilFirstCompression;}
    virtual bool& bUseEffectiveSuspensionSweepRadiusWheelDomainStorage() {return bUseEffectiveSuspensionSweepRadius;}
    virtual bool const& bUseEffectiveSuspensionSweepRadiusWheelDomainStorage() const {return bUseEffectiveSuspensionSweepRadius;}
    virtual bool& bUseIncreasingDisplacementAsCompressionWheelDomainStorage() {return bUseIncreasingDisplacementAsCompression;}
    virtual bool const& bUseIncreasingDisplacementAsCompressionWheelDomainStorage() const {return bUseIncreasingDisplacementAsCompression;}
    virtual bool& bUseCompressionCrossingContactImpulseWheelDomainStorage() {return bUseCompressionCrossingContactImpulse;}
    virtual bool const& bUseCompressionCrossingContactImpulseWheelDomainStorage() const {return bUseCompressionCrossingContactImpulse;}
    virtual uint8& GroundForceApplicationModeWheelDomainStorage() {return GroundForceApplicationMode;}
    virtual uint8 const& GroundForceApplicationModeWheelDomainStorage() const {return GroundForceApplicationMode;}
    virtual uint8& ContactImpulseApplicationModeWheelDomainStorage() {return ContactImpulseApplicationMode;}
    virtual uint8 const& ContactImpulseApplicationModeWheelDomainStorage() const {return ContactImpulseApplicationMode;}
    virtual float& SuspensionStiffnessForceScaleWheelDomainStorage() {return SuspensionStiffnessForceScale;}
    virtual float const& SuspensionStiffnessForceScaleWheelDomainStorage() const {return SuspensionStiffnessForceScale;}
    virtual float& SuspensionStiffnessReferenceDisplacementWheelDomainStorage() {return SuspensionStiffnessReferenceDisplacement;}
    virtual float const& SuspensionStiffnessReferenceDisplacementWheelDomainStorage() const {return SuspensionStiffnessReferenceDisplacement;}
    virtual float& SuspensionPreRestCompressionDampingScaleWheelDomainStorage() {return SuspensionPreRestCompressionDampingScale;}
    virtual float const& SuspensionPreRestCompressionDampingScaleWheelDomainStorage() const {return SuspensionPreRestCompressionDampingScale;}
    virtual bool& bClampNegativeSuspensionDisplacementWheelDomainStorage() {return bClampNegativeSuspensionDisplacement;}
    virtual bool const& bClampNegativeSuspensionDisplacementWheelDomainStorage() const {return bClampNegativeSuspensionDisplacement;}
    virtual bool& bDisableSuspensionBumpStopForceWheelDomainStorage() {return bDisableSuspensionBumpStopForce;}
    virtual bool const& bDisableSuspensionBumpStopForceWheelDomainStorage() const {return bDisableSuspensionBumpStopForce;}
    virtual float& SuspensionSpringRateCmOverrideWheelDomainStorage() {return SuspensionSpringRateCmOverride;}
    virtual float const& SuspensionSpringRateCmOverrideWheelDomainStorage() const {return SuspensionSpringRateCmOverride;}
    virtual float& SuspensionCompressionDampingOverrideWheelDomainStorage() {return SuspensionCompressionDampingOverride;}
    virtual float const& SuspensionCompressionDampingOverrideWheelDomainStorage() const {return SuspensionCompressionDampingOverride;}
    virtual float& SuspensionReboundDampingOverrideWheelDomainStorage() {return SuspensionReboundDampingOverride;}
    virtual float const& SuspensionReboundDampingOverrideWheelDomainStorage() const {return SuspensionReboundDampingOverride;}
    virtual float& SuspensionMaxRaiseOverrideWheelDomainStorage() {return SuspensionMaxRaiseOverride;}
    virtual float const& SuspensionMaxRaiseOverrideWheelDomainStorage() const {return SuspensionMaxRaiseOverride;}
    virtual float& SuspensionMaxDropOverrideWheelDomainStorage() {return SuspensionMaxDropOverride;}
    virtual float const& SuspensionMaxDropOverrideWheelDomainStorage() const {return SuspensionMaxDropOverride;}
    virtual float& ConfiguredStaticSpringCompressionWheelDomainStorage() {return ConfiguredStaticSpringCompression;}
    virtual float const& ConfiguredStaticSpringCompressionWheelDomainStorage() const {return ConfiguredStaticSpringCompression;}
    // Ordinary callers preserve the exact existing live host configuration.
    // An original epoch can consume its explicit immutable authored value.
    virtual float PhysicalSuspensionRestLength() const;
    ISpeedWheeledComponent* WheelComponent = nullptr;

    // --- Wheel configuration ---
    UChaosVehicleWheel* ChaosWheel = nullptr;
    Chaos::FSimpleWheelSim* PWheel = nullptr;
    Chaos::FSimpleSuspensionSim* PSuspension = nullptr;

    UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = Wheel, meta = (AllowPrivateAccess = "true", ClampMin = "0.0", UIMin = "0.0"))
    float WheelMass = DefaultWheelMass;

    UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = Suspension, meta = (AllowPrivateAccess = "true", ClampMin = "0.0", UIMin = "0.0"))
    float SuspensionSpringRate = 2750.0f;

    UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = Suspension, meta = (AllowPrivateAccess = "true", ClampMin = "0.0", ClampMax = "1.0", UIMin = "0.0", UIMax = "1.0"))
    float SuspensionDampingCompressionRatio = 0.3f;
    UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = Suspension, meta = (AllowPrivateAccess = "true", ClampMin = "0.0", ClampMax = "1.0", UIMin = "0.0", UIMax = "1.0"))
    float SuspensionDampingReboundRatio = 0.5f;

    bool bUseSuspensionForceModelOverride = false;
    bool bClampSuspensionForceToPositive = false;
    bool bClampPositiveUntilFirstCompression = false;
    bool bUseEffectiveSuspensionSweepRadius = false;
    bool bUseIncreasingDisplacementAsCompression = false;
    bool bUseCompressionCrossingContactImpulse = false;
	uint8 GroundForceApplicationMode = 0;
	uint8 ContactImpulseApplicationMode = 0;
    float SuspensionStiffnessForceScale = 1.0f;
    float SuspensionStiffnessReferenceDisplacement = 0.0f;
    float SuspensionPreRestCompressionDampingScale = -1.0f;
    bool bClampNegativeSuspensionDisplacement = false;
    bool bDisableSuspensionBumpStopForce = false;
    float SuspensionSpringRateCmOverride = 0.0f;
    float SuspensionCompressionDampingOverride = 0.0f;
    float SuspensionReboundDampingOverride = 0.0f;
    float SuspensionMaxRaiseOverride = -1.0f;
    float SuspensionMaxDropOverride = -1.0f;
    float ConfiguredStaticSpringCompression = 0.0f;

    // Private owners may separate these runtime values from facade views.
    virtual float& SuspensionForceRuntimeStorage() {return SuspensionForce;}
    virtual float const& SuspensionForceRuntimeStorage() const {return SuspensionForce;}
    virtual uint8& bIsJumpingRuntimeStorage() {return bIsJumping;}
    virtual uint8 const& bIsJumpingRuntimeStorage() const {return bIsJumping;}
    virtual bool& bJumpUnilateralSupportRuntimeStorage() {return bJumpUnilateralSupport;}
    virtual bool const& bJumpUnilateralSupportRuntimeStorage() const {return bJumpUnilateralSupport;}
    virtual uint8& ConsecutiveGroundFramesRuntimeStorage() {return ConsecutiveGroundFrames;}
    virtual uint8 const& ConsecutiveGroundFramesRuntimeStorage() const {return ConsecutiveGroundFrames;}
    virtual float& SteeringAngleRuntimeStorage() {return SteeringAngle;}
    virtual float const& SteeringAngleRuntimeStorage() const {return SteeringAngle;}
    virtual float& OmegaRuntimeStorage() {return Omega;}
    virtual float const& OmegaRuntimeStorage() const {return Omega;}
    virtual float& RollAngleRuntimeStorage() {return RollAngle;}
    virtual float const& RollAngleRuntimeStorage() const {return RollAngle;}
    virtual bool& bWasOnGroundPrevFrameRuntimeStorage() {return bWasOnGroundPrevFrame;}
    virtual bool const& bWasOnGroundPrevFrameRuntimeStorage() const {return bWasOnGroundPrevFrame;}
    virtual bool& bCountedGroundFrameRuntimeStorage() {return bCountedGroundFrame;}
    virtual bool const& bCountedGroundFrameRuntimeStorage() const {return bCountedGroundFrame;}
    virtual bool& bContactVelocityLockedRuntimeStorage() {return bContactVelocityLocked;}
    virtual bool const& bContactVelocityLockedRuntimeStorage() const {return bContactVelocityLocked;}
    virtual bool& bCrossedIntoCompressionThisFrameRuntimeStorage() {return bCrossedIntoCompressionThisFrame;}
    virtual bool const& bCrossedIntoCompressionThisFrameRuntimeStorage() const {return bCrossedIntoCompressionThisFrame;}
    virtual FVector& ForwardAxisRuntimeStorage() {return ForwardAxis;}
    virtual FVector const& ForwardAxisRuntimeStorage() const {return ForwardAxis;}
    virtual FVector& RightAxisRuntimeStorage() {return RightAxis;}
    virtual FVector const& RightAxisRuntimeStorage() const {return RightAxis;}
    virtual FVector& UpAxisRuntimeStorage() {return UpAxis;}
    virtual FVector const& UpAxisRuntimeStorage() const {return UpAxis;}
    virtual SWheelRenderData& RenderDataRuntimeStorage() {return RenderData;}
    virtual SWheelRenderData const& RenderDataRuntimeStorage() const {return RenderData;}
    // --- State variables ---
    float SuspensionForce = 0.0; // suspension force to apply this frame
    uint8 bIsJumping = 0; // 0 = not jumping else countdown of frames before wheel is in max drop position
    bool bJumpUnilateralSupport = false;
    uint8 ConsecutiveGroundFrames = 0; // saturated to 31 for compact gameplay replication
    float SteeringAngle = 0.0f; // radians
    float Omega = 0.0f; // current angular velocity of the wheel (rad/s)
    float RollAngle = 0.0f; // current roll angle of the wheel (for rendering)
    bool bWasOnGroundPrevFrame = false;

    bool bCountedGroundFrame = false;
    bool bContactVelocityLocked = false;
    bool bCrossedIntoCompressionThisFrame = false;

    FVector ForwardAxis = FVector::ForwardVector;
    FVector RightAxis = FVector::RightVector;
    FVector UpAxis = FVector::UpVector;

    SWheelRenderData RenderData;
private:
    friend class USpeedBranchWheelSubBody;
    class FLegacyWheelUpdateAccess;class FNativeWheelUpdateAccess;
    class FLegacyWheelHitAccess;class FNativeWheelHitAccess;
    template<class TAccess> void AcceptWheelHitBody(TAccess& Storage);
    void AcceptNativeWheelHit(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation);
    template<class TAccess> void UpdateWheelKinematicsFromOwnerBody(TAccess& Storage,const SKinematic& ParentKinematic);
    void UpdateNativeWheelKinematicsFromOwner(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const SKinematic& ParentKinematic);

};
