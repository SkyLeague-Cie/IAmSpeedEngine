// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "SphereSubBody.h"
#include "SWheelSubBody.generated.h"

class ISpeedWheeledComponent;
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
	
public:

    // initializer methods
    virtual void Initialize(ISpeedComponent* InParentComponent) override;

    bool IsOnGround() const; // true if the wheel IS currently on the ground
    void SetOnGround(const bool& on_ground);
    void SetIsJumping(const uint8 NbFrames);
    uint8 IsJumping() const;

    // --- overrides SubBody behaviour ---
    virtual void ResetForFrame(const float& Delta) override;
    void AcceptHit() override;
    virtual bool SweepTOI(const float& RemainingDelta, float& OutTOI) override;
    SKinematic GetKinematicsFromOwner(const unsigned int& NumFrame) const;
    // SSphere MakeSphere(const unsigned int& NumFrame, const float& RemainingDelta, const float& TimePassed) const override;
    void ApplyImpulse(const FVector& LinearImpulse, const FVector& WorldPoint) override;

    // --- steering methods ---
    bool IsSteeringEnabled() const;
    float MaxSteeringAngle() const;
    void UpdateSteerAngle(const float& delta);
    void UpdatePhysicsState(const float& delta);

    // --- suspension methods ---
    void UpdateSuspension(const float& delta);
    // sweep for suspension
    void SweepSuspension(const float& delta);

    // Utils methods
    FVector WorldPos() const;
    FVector SteeringPos() const;
    FVector WorldPosFromCarTransform(const FTransform& CarTransform) const;
    FVector GetSuspensionDirectionWS() const;
    float Radius() const;
    float AngularVelocity() const;
    void SetAngularVelocity(const float& InOmega);
    float SpringDisplacement() const; // current spring displacement (positive when compressed)
    float GetLastDisplacement() const;
    void SetLastDisplacement(const float& displacement);
    float SpringLength() const;
    float MaxLength() const; // max suspension length (rest length + max drop)
    float SuspensionRestLength() const;
    float SuspensionMaxRaise() const;
    float SuspensionMaxDrop() const;
    float GetSuspensionForce() const;
    float GetSuspensionOffset() const;
    FVector GetHitContactNormal() const;
    void SetHitContactNormal(const FVector& ImpactNormal);
    const SWheelRenderData& GetRenderData() const;
    void SetRollAngle(const float& angle);
    float GetRollAngle() const;

	void SetChaosWheel(UChaosVehicleWheel* InChaosWheel);
	void SetWheelSim(Chaos::FSimpleWheelSim* InPWheel);
	void SetSuspensionSim(Chaos::FSimpleSuspensionSim* InPSuspension);
	void SetLocalOffset(const FVector& InLocalOffset);

    void HandleTimers();
private:
    bool SweepSuspensionOnGround(SHitResult& OutHit, const float& delta);
    bool SweepSuspensionOnSpheres(SHitResult& OutHit, const float& delta);
    bool SweepSuspensionOnBoxes(SHitResult& OutHit, const float& delta);

    // Predicts spring displacement / wheel travel
    float PredictNextDisplacement(const float& Delta) const;
    float ComputeNextAirLength(const float& DeltaTime) const;

private:
    ISpeedWheeledComponent* WheelComponent = nullptr;

    // --- Wheel configuration ---
    UChaosVehicleWheel* ChaosWheel = nullptr;
    Chaos::FSimpleWheelSim* PWheel = nullptr;
    Chaos::FSimpleSuspensionSim* PSuspension = nullptr;

    // --- State variables ---
    float SuspensionForce = 0.0; // suspension force to apply this frame
    uint8 bIsJumping = 0; // 0 = not jumping else countdown of frames before wheel is in max drop position
    float SteeringAngle = 0.0f; // radians
    float Omega = 0.0f; // current angular velocity of the wheel (rad/s)
    float RollAngle = 0.0f; // current roll angle of the wheel (for rendering)
    bool bWasOnGroundPrevFrame = false;

    FVector ForwardAxis = FVector::ForwardVector;
    FVector RightAxis = FVector::RightVector;
    FVector UpAxis = FVector::UpVector;

    SWheelRenderData RenderData;
};
