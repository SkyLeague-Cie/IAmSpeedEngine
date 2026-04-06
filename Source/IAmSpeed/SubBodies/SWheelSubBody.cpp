// Fill out your copyright notice in the Description page of Project Settings.


#include "SWheelSubBody.h"
#include "IAmSpeed/Components/ISpeedWheeledComponent.h"
#include "Configs/WheelSubBodyConfig.h"
#include "ChaosVehicleWheel.h"
#include "IAmSpeed/Base/SpeedConstant.h"
#include "IAmSpeed/SubBodies/BoxSubBody.h"

DEFINE_LOG_CATEGORY(WheelSubBodyLog);

USWheelSubBody::USWheelSubBody(const FObjectInitializer& ObjectInitializer):
	Super(ObjectInitializer)
{
	SubBodyType = ESubBodyType::Wheel;
    IgnoredSubBodyTypes.AddUnique(ESubBodyType::Wheel);
	bApplyRestForce = false; // wheel apply suspension force directly, so we do not want rest force to interfere with it
}

void USWheelSubBody::Initialize(ISpeedComponent* InParentComponent)
{
	Super::Initialize(InParentComponent);
	if (InParentComponent)
	{
		ParentComponent = InParentComponent;
		WheelComponent = static_cast<ISpeedWheeledComponent*>(InParentComponent);

		if (WheelComponent)
		{
			WheelSubBodyConfig Config = WheelComponent->GetWheelSubBodyConfig(*this);
            if (Config.bValid)
            {
                SetIdx(Config.WheelIndex);
                ChaosWheel = Config.ChaosWheel;
                PWheel = Config.PWheel;
                PSuspension = Config.PSuspension;
                // keep consistency with radius and mass
                SetRadius(PWheel->GetEffectiveRadius());
                SetMass(PWheel->MassPerWheel);
                InvInertiaLocal = InitInvInertiaTensor();
            }
		}
	}
}


void USWheelSubBody::ResetForFrame(const float& Delta)
{
    // CurrentHit = SHitResult(); // do not reset current hit, because netcode needs it to know if wheel is transitionning on wall/ground
    FutureHit = SHitResult();
    IgnoredComponents.Empty();
    bWasOnGroundPrevFrame = IsOnGround();
	IgnoredComponents.Append(AlwaysIgnoredComponents);
}

void USWheelSubBody::AcceptHit()
{
    // Do nothing; we keep CurrentHit as is (suspension handles it)
    // CurrentHit = FutureHit;
    // Ignore this component for the rest of the frame
    if (CurrentHit.Component.IsValid())
        IgnoredComponents.AddUnique(CurrentHit.Component.Get());
    else if (FutureHit.Component.IsValid())
        // for safety, ignore FutureHit component too
        IgnoredComponents.AddUnique(FutureHit.Component.Get());
}

SKinematic USWheelSubBody::GetKinematicsFromOwner(const unsigned int& NumFrame) const
{
    SKinematic WheelKinematics;
    if (!ParentComponent)
        return WheelKinematics;
    SKinematic CarKinematics = ParentComponent->GetKinematicStateForFrame(NumFrame);
    FTransform ChassisTM(CarKinematics.Rotation, CarKinematics.Location);
    auto WheelPos = WorldPosFromCarTransform(ChassisTM);
    auto WheelVel = ParentComponent->GetPhysVelocityAtPoint(WheelPos);
    auto WheelAcc = ParentComponent->GetPhysAccelerationAtPoint(WheelPos);

    WheelKinematics.Location = WheelPos;
    WheelKinematics.Velocity = WheelVel;
    WheelKinematics.Acceleration = WheelAcc;
    return WheelKinematics;
}


// ==================================================
// =============== SweepTOI methods =================
// ==================================================

bool USWheelSubBody::SweepTOI(const float& RemainingDelta, const float& TimePassed, float& OutTOI)
{
    // wheel does not do TOI sweep, it only sweeps for suspension.
    // We could implement it in the future if we want to detect early collision on wheels
    // (currently we rely on sweep suspension to detect collision with ground and walls, 
    // and rely on hitboxes and "real" spheres to detect collision with other cars and spheres which is sufficient for now)
	return false;
}

bool USWheelSubBody::SweepVsGround(SHitResult& OutHit, const float& delta, float& OutTOI)
{
    OutTOI = delta;
    float TOIground = delta;
    // integrate car kinematics to compute future wheel trace
    SKinematic CarState = ParentComponent->GetKinematicState();
    SKinematic FutureCarState = CarState.Integrate(delta);
    FVector FutureCarLocation = FutureCarState.Location;
    FQuat FutureCarRotation = FutureCarState.Rotation;
    FVector FutureCarUpVector = FutureCarRotation.GetUpVector();
    FTransform FutureChassisTM = FTransform(FutureCarRotation, FutureCarLocation);


    // Compute future wheel position
    float NewSpringDisplacement = PredictNextDisplacement(delta); // if wheel will be in air next frame
    FVector FutureWheelPos = WorldPosFromCarTransform(FutureChassisTM);
    FVector Start = WorldPos();
    // auto Sphere = SSphere(Start, Radius(), FVector::ZeroVector, FVector::ZeroVector);
    // Sphere.DrawDebug(GetWorld());
    FVector End = FutureWheelPos + (NewSpringDisplacement - SpringDisplacement()) * FutureCarUpVector; // take into account suspension compression/extension
    bool bHitGround = InternalSweep(Start, End, OutHit, delta);

    if (bHitGround)
    {
        OutTOI = OutHit.TOI;
        if (OutTOI > KINDA_SMALL_NUMBER)
        {
            // UE_LOG(WheelSubBodyLog, Log, TEXT("[%s][CCDWheel(%d)] NumFrame=%d, SweepVSGround at TOI=%f"), *ParentComponent->GetRole(),
            //    Idx(), ParentComponent->NumFrame(), OutTOI);
        }
        return true;
    }

    return false;
}


// ==================================================
// =========== Sweep suspension methods =============
// ==================================================

void USWheelSubBody::SweepSuspension(const float& delta)
{
    bool bHitSphere = SweepSuspensionOnSpheres(CurrentHit, delta);
    if (bHitSphere)
    {
        return;
    }
    bool bHitBox = SweepSuspensionOnBoxes(CurrentHit, delta);
    if (bHitBox)
    {
        return;
    }

    bool bHitGround = SweepSuspensionOnGround(CurrentHit, delta);
    if (!bHitGround)
    {
        SetOnGround(false);
        if (WheelComponent)
        {
            WheelComponent->UpdateWheelOnGroundStates();
        }
    }
}

bool USWheelSubBody::SweepSuspensionOnGround(SHitResult& OutHit, const float& delta)
{
    UWorld* World = GetWorld();
    if (!World || !ParentComponent) return false;

    FVector CurrentPos = WorldPos();

    FTransform ChassisTM(ParentComponent->GetPhysRotation(), ParentComponent->GetPhysLocation());
    const FVector Up = ChassisTM.GetUnitAxis(EAxis::Z);

    FVector WorldRestingPos = ChassisTM.TransformPosition(GetLocalOffset());
    FVector CarUpVector = ChassisTM.GetUnitAxis(EAxis::Z);
    FVector Start = WorldRestingPos + SuspensionMaxRaise() * CarUpVector;
    float NewSpringDisplacement = PredictNextDisplacement(delta); // if wheel will be in air next frame
    FVector End = CurrentPos + (NewSpringDisplacement - SpringDisplacement() - CollisionMargin()) * CarUpVector;
    // auto Sphere = SSphere(WorldPos(), Radius(), FVector::ZeroVector, FVector::ZeroVector);
    // Sphere.DrawDebug(GetWorld());

    FCollisionQueryParams Params(NAME_None, false);
    Params.bReturnFaceIndex = true;
    Params.bReturnPhysicalMaterial = true;
    SHitResult OldCurrentHit = CurrentHit;
    bool ret = false;
    FHitResult UnrealHit;
    if (World->SweepSingleByChannel(
        UnrealHit,
        Start,
        End,
        Kinematics.Rotation,
        GetCollisionChannel(),
        GetCollisionShape(),
        Params,
        GetResponseParams()
    ))
    {
        OutHit = SHitResult::FromUnrealHit(UnrealHit, delta);
        OutHit.ImpactNormal = Speed::QuantizeUnitNormal(OutHit.ImpactNormal);
        SetOnGround(true);
        ret = true;
        auto OldNormal = OldCurrentHit.ImpactNormal;
        auto NewNormal = OutHit.ImpactNormal;
        auto OldCurrentDot = FVector::DotProduct(OldCurrentHit.ImpactNormal, OutHit.ImpactNormal);
        // detect wheel is transitionning from ground to wall or wall to ground
        if (!NewNormal.Equals(OldNormal))
        {
            ParentComponent->RcvImpactOnSubBody(*this, OutHit.ImpactPoint);
            // UE_LOG(WheelSubBodyLog, Warning, TEXT("[CCDWheel(%d)] [%s] NumFrame=%d, Wheel transitioned between ground/wall contact"),
            //	Idx(), *ParentComponent->GetRole(), ParentComponent->NumFrame());
        }
    }
    else
    {
        SetOnGround(false);
    }
    if (WheelComponent)
    {
        WheelComponent->UpdateWheelOnGroundStates();
    }
    return ret;
}

bool USWheelSubBody::SweepSuspensionOnSpheres(SHitResult& OutHit,  const float& delta)
{
    // --------------------------------------------------
    // Build suspension sweep segment
    // --------------------------------------------------
    FTransform ChassisTM(ParentComponent->GetPhysRotation(), ParentComponent->GetPhysLocation());
    const FVector CarUp = ChassisTM.GetUnitAxis(EAxis::Z);

    const FVector WorldRest = ChassisTM.TransformPosition(GetLocalOffset());
    const FVector Start = WorldRest + SuspensionMaxRaise() * CarUp;

    const float NewDisp = PredictNextDisplacement(delta);
    const FVector End =
        WorldPos() +
        (NewDisp - SpringDisplacement() - CollisionMargin()) * CarUp;

    // --------------------------------------------------
    // Prepare This Sphere
    // --------------------------------------------------
    SSphere ThisSphere(
        Start,
        Radius(),
        FVector::ZeroVector,
        FVector::ZeroVector
    );

    bool bHit = false;
    float BestTOI = delta;

    SHitResult BestHit;
    TWeakObjectPtr<USphereSubBody> BestSphere = nullptr;
    const TArray<TWeakObjectPtr<USphereSubBody>>& OtherSpheres = ExternalSphereSubBodies;

    for (auto& OtherSphere : OtherSpheres)
    {
        if (!OtherSphere.IsValid()) continue;

        // Ignore already-hit Spheres this frame
        if (IgnoredComponents.Contains(OtherSphere.Get()))
            continue;

        SSphere OSphere = OtherSphere->MakeSphere(ParentComponent->NumFrame(), delta, /*TimePassed*/ 0.0f); // useless to pass TimePassed here since every SubBodies are updated to current TimePassed before sweeping

        SHitResult Hit = ThisSphere.IntersectDuringMovement(OSphere, Start, End, delta);
        if (!Hit.bHit)
            continue;

        if (Hit.TOI < BestTOI)
        {
            BestTOI = Hit.TOI;
            BestHit = Hit;
            BestSphere = OtherSphere;
            bHit = true;
        }
    }

    if (!bHit)
        return false;

    OutHit = BestHit;
    OutHit.ImpactNormal = Speed::QuantizeUnitNormal(OutHit.ImpactNormal);
    OutHit.Location = OutHit.ImpactPoint + Radius() * OutHit.ImpactNormal;

    SetOnGround(true);
    if (WheelComponent)
    {
        WheelComponent->UpdateWheelOnGroundStates();
    }
    // ParentComponent->SetHadImpactThisFrame(true);

    return true;
}

bool USWheelSubBody::SweepSuspensionOnBoxes(SHitResult& OutHit, const float& delta)
{
    // --------------------------------------------------
    // Build suspension sweep segment
    // --------------------------------------------------
    FTransform ChassisTM(ParentComponent->GetPhysRotation(), ParentComponent->GetPhysLocation());
    const FVector CarUp = ChassisTM.GetUnitAxis(EAxis::Z);

    const FVector WorldRest = ChassisTM.TransformPosition(GetLocalOffset());
    const FVector Start = WorldRest + SuspensionMaxRaise() * CarUp;

    const float NewDisp = PredictNextDisplacement(delta);
    const FVector End =
        WorldPos() +
        (NewDisp - SpringDisplacement() - CollisionMargin()) * CarUp;

    // --------------------------------------------------
    // Prepare This Sphere
    // --------------------------------------------------
    SSphere ThisSphere(
        Start,
        Radius(),
        FVector::ZeroVector,
        FVector::ZeroVector
    );

    bool bHit = false;
    float BestTOI = delta;

    SHitResult BestHit;
    TWeakObjectPtr<UBoxSubBody> BestBox = nullptr;
	const TArray<TWeakObjectPtr<UBoxSubBody>>& OtherBoxes = ExternalBoxSubBodies;

    for (auto& OtherBox : OtherBoxes)
    {
        if (!OtherBox.IsValid()) continue;
        // Ignore already-hit Boxes this frame
        if (IgnoredComponents.Contains(OtherBox.Get()))
            continue;

        SSBox OBox = OtherBox->MakeBox(ParentComponent->NumFrame(), /*TimePassed*/ 0.0f); // useless to pass TimePassed here since every SubBodies are updated to current TimePassed before sweeping

        SHitResult Hit = ThisSphere.IntersectDuringMovement(OBox, Start, End, delta);
        if (!Hit.bHit)
            continue;

        if (Hit.TOI < BestTOI)
        {
            BestTOI = Hit.TOI;
            BestHit = Hit;
            BestBox = OtherBox;
            bHit = true;
        }
    }

    if (!bHit)
        return false;

    OutHit = BestHit;
    OutHit.ImpactNormal = Speed::QuantizeUnitNormal(OutHit.ImpactNormal);
    OutHit.Location = OutHit.ImpactPoint + Radius() * OutHit.ImpactNormal;

    SetOnGround(true);
    if (WheelComponent)
    {
        WheelComponent->UpdateWheelOnGroundStates();
    }
    // ParentComponent->SetHadImpactThisFrame(true);

    return true;
}

void USWheelSubBody::UpdateSuspension(const float& delta)
{
    if (delta <= KINDA_SMALL_NUMBER)
        return;

    FTransform ChassisTM(ParentComponent->GetPhysRotation(), ParentComponent->GetPhysLocation());
    const FVector Up = ChassisTM.GetUnitAxis(EAxis::Z);
    // update steering angle
    UpdateSteerAngle(delta);

    if (IsOnGround())
    {
        // compute world anchor pos
        FVector SuspensionAnchorWorldPos = ChassisTM.TransformPosition(GetLocalOffset() +
            PSuspension->Setup().MaxLength * FVector::UpVector);
        const FVector SuspensionAxis = Up; // Car Up Vector
        const float ProjectedCompression = FVector::DotProduct(SuspensionAnchorWorldPos - CurrentHit.Location, SuspensionAxis);

        // update suspension state
        float LastDisplacement = SpringDisplacement();
        float NewDesiredLength = ProjectedCompression + Radius();
        PSuspension->SetSuspensionLength(NewDesiredLength, Radius());
        PSuspension->Simulate(delta);
        float CurrentDisplacement = SpringDisplacement();
        SuspensionForce = PSuspension->GetSuspensionForce();

        float Dot = FVector::DotProduct(CurrentHit.ImpactNormal, FVector::UpVector);
        // mapping : ground=1, wall=1, ceiling=0
        float SuspensionScale = FMath::Clamp(Dot + 1.f, 0.f, 1.f);
        SuspensionScale = FMath::Pow(SuspensionScale, 1.5f);  // soften transition

        // apply suspension scale
        SuspensionForce *= SuspensionScale;
        const float SuspensionMaxValue = 5e6;
        /*if (FMath::Abs(SuspensionForce) > SuspensionMaxValue)
        {
            UE_LOG(WheelSubBodyLog, Warning, TEXT("[%s] Suspension in frame %d is way too strong!!!! For Wheel num %d SuspensionForce = %f"),
                *ParentComponent->GetRole(), ParentComponent->NumFrame(), WheelIndex, SuspensionForce);
        }*/
        SuspensionForce = FMath::Clamp(SuspensionForce, -SuspensionMaxValue, SuspensionMaxValue);

        // Special Actor case: if we do not hit the ground but an other actor, we reduce suspension force
        // to avoid wheel to stick to it too much and create unrealistic behavior.
        // This is a quick fix to tune, we might want to find a more physical solution
        if (CurrentHit.Component.IsValid() &&
            CurrentHit.Component->Mobility !=
            EComponentMobility::Static)
        {
            SuspensionForce /= 100;
        }
#if !(UE_BUILD_SHIPPING)
        // UE_LOG(WheelSubBodyLog, Warning, TEXT("[%s] Suspension in frame %d for Wheel num %d SuspensionForce= %.2f N (Scale= %.2f, Dot= %.2f)"),
        //    *ParentComponent->GetRole(), ParentComponent->NumFrame(), WheelIndex, SuspensionForce, SuspensionScale, Dot);
#endif
    }
    else
    {
        SuspensionForce = 0.0;

        PSuspension->SetSuspensionLength(ComputeNextAirLength(delta), Radius());
        PWheel->SetWheelLoadForce(0.f);
        PSuspension->Simulate(delta);
    }
}



SSphere USWheelSubBody::MakeSphere(const unsigned int& NumFrame, const float& RemainingDelta, const float& TimePassed) const
{
    // --- 1) Car kinematics ---
    SKinematic CarKS0 = ParentComponent->GetKinematicStateForFrame(NumFrame);
    SKinematic CarKS1 = CarKS0.Integrate(TimePassed);
    SKinematic CarKS2 = CarKS1.Integrate(RemainingDelta);

    // --- 2) Chassis transforms ---
    const FTransform TM1(CarKS1.Rotation, CarKS1.Location);
    const FTransform TM2(CarKS2.Rotation, CarKS2.Location);

    const FVector Up1 = TM1.GetUnitAxis(EAxis::Z);
    const FVector Up2 = TM2.GetUnitAxis(EAxis::Z);

    // --- 3) Wheel positions ---
    const float Disp1 = PredictNextDisplacement(TimePassed);
    const float Disp2 = PredictNextDisplacement(TimePassed + RemainingDelta);

    const FVector Start =
        WorldPosFromCarTransform(TM1) +
        (Disp1 - SpringDisplacement()) * Up1;

    const FVector End =
        WorldPosFromCarTransform(TM2) +
        (Disp2 - SpringDisplacement()) * Up2;

    // --- 4) Wheel velocity at t = TimePassed ---
    const FVector r1 = Start - CarKS1.Location;
    const FVector v1 = CarKS1.Velocity + FVector::CrossProduct(CarKS1.AngularVelocity, r1);

    // --- 5) Constant acceleration matching Start -> End ---
    FVector a = FVector::ZeroVector;
    if (RemainingDelta > KINDA_SMALL_NUMBER)
    {
        a = 2.f * (End - Start - v1 * RemainingDelta)
            / (RemainingDelta * RemainingDelta);
    }

    // --- 6) Build sphere ---
    return SSphere(
        Start,
        Radius(),
        v1,
        a
    );
}

void USWheelSubBody::ApplyImpulse(const FVector& LinearImpulse, const FVector& WorldPoint)
{
    if (WheelComponent)
    {
        FVector N = CurrentHit.ImpactNormal.GetSafeNormal();

        const FVector VelAtPoint = WheelComponent->GetPhysVelocityAtPoint(WorldPoint);
        const float vN = FVector::DotProduct(VelAtPoint, N);

        if (vN >= 0.f)
            return;

        const FVector r = WorldPos() - WheelComponent->GetPhysCOM();
        const FMatrix InvI = WheelComponent->ComputeWorldInvInertiaTensor();
        const FVector rxn = FVector::CrossProduct(r, N);
        const FVector term = FVector::CrossProduct(
            InvI.TransformVector(rxn), r);

        const float invMassEff =
            (1.f / WheelComponent->GetPhysMass()) + FVector::DotProduct(N, term);

        // UE_LOG(WheelSubBodyLog, Log, TEXT("[CCDWheel(%d)] RegisterWheelGroundContact at NumFrame=%d"),
        //    Idx(), WheelComponent->NumFrame());
        WheelComponent->RegisterWheelGroundContact({
            this,
            WorldPos(),
            N,
            r,
            vN,
            invMassEff
            });
	}
}

float USWheelSubBody::PredictNextDisplacement(const float& delta) const
{
    float DisplacementInput = ComputeNextAirLength(delta) - PSuspension->Setup().RaycastSafetyMargin - Radius();
    DisplacementInput = FMath::Max(0.f, DisplacementInput);
    return MaxLength() - DisplacementInput;
}

float USWheelSubBody::ComputeNextAirLength(const float& DeltaTime) const
{
    auto TargetPos = MaxLength() + SuspensionMaxDrop() + Radius();
    auto CurrentPos = TargetPos - SpringDisplacement() - SuspensionMaxDrop();
    auto target_speed = 1.0;
    if (!IsJumping())
    {
        return TargetPos;
    }
    else
    {
        return FMath::FInterpTo(CurrentPos, TargetPos, DeltaTime, target_speed);
    }
}


void USWheelSubBody::HandleTimers()
{
    if (bIsJumping > 0)
    {
        bIsJumping--;
    }
}

void USWheelSubBody::UpdatePhysicsState(const float& delta)
{
    // --- 1. Base axes from chassis ---
    const FVector CarUp = ParentComponent->GetPhysUpVector();
    const FVector CarFwd = ParentComponent->GetPhysForwardVector();
    const FVector CarRight = ParentComponent->GetPhysRightVector();

    UpAxis = CarUp; // stays constant

    // --- 2. Compute steering rotation around UpAxis ---
    // SteeringAngle in radians
    FQuat SteerQuat(UpAxis, SteeringAngle);

    // --- 3. Apply steering to forward axis ---
    ForwardAxis = SteerQuat.RotateVector(CarFwd).GetSafeNormal();

    // --- 4. Right axis = orthogonal ---
    RightAxis = FVector::CrossProduct(UpAxis, ForwardAxis).GetSafeNormal();

    // --- 5. Update rolling angle (rotation around RightAxis) ---
    // update omega
    Omega = ParentComponent->GetPhysForwardSpeed() / FMath::Max(Radius(), KINDA_SMALL_NUMBER);
    float angVel = AngularVelocity(); // rad/s
    RollAngle += angVel * delta;
    RollAngle = FMath::Fmod(RollAngle, 2.f * PI);

    // --- 6. World position from suspension ---
    FVector pos = WorldPos();

    // --- 7. Fill render data struct for the visual thread ---
    RenderData.WorldPosition = pos;
    RenderData.UpAxis = UpAxis;
    RenderData.ForwardAxis = ForwardAxis;
    RenderData.RightAxis = RightAxis;

    RenderData.SpringOffset = SuspensionRestLength() - SpringLength();

    RenderData.SteerRotation = FQuat(UpAxis, SteeringAngle);
    RenderData.RollRotation = FQuat(RightAxis, RollAngle);
}

void USWheelSubBody::UpdateSteerAngle(const float& delta)
{
    float steerInput = 0.0;
    if (WheelComponent)
    {
		steerInput = WheelComponent->GetPhysSteeringInput(); // expected to be in range [-1, 1]
    }

    if (IsSteeringEnabled())
    {
        float speed = ParentComponent->GetPhysVelocity().Size();

        // saturation max speed
        float normSpeed = FMath::Clamp(speed / ParentComponent->GetPhysMaxSpeed(), 0.f, 1.f);

        // steering reduction curve
        float steerFactor =
            1.0f
            - 0.85f * normSpeed;  // 1.0 at low speeds, ~0.25 at max speed

        if (speed < 300.f)
            steerFactor *= 1.2f; // +20%

        // final angle
        SteeringAngle = FMath::DegreesToRadians(MaxSteeringAngle() * steerInput * steerFactor);
    }
    else
    {
        SteeringAngle = 0.0f;
    }
}

FVector USWheelSubBody::WorldPos() const
{
    FTransform ChassisTM(ParentComponent->GetPhysRotation(), ParentComponent->GetPhysLocation());
    return WorldPosFromCarTransform(ChassisTM);
}

FVector USWheelSubBody::SteeringPos() const
{
    const FVector ChassisUp = ParentComponent->GetPhysUpVector();

    // Compute world position of the wheel
    const FVector WheelPosWS = WorldPos();

    // Get the world center of mass of the vehicle
    const FVector CoMWorld = ParentComponent->GetPhysCOM();

    // Compute the vertical distance between the wheel and the center of mass
    const float VerticalDist = FVector::DotProduct((WheelPosWS - CoMWorld), ChassisUp);

    // Project the wheel onto the horizontal plane of the CoM
    const FVector SteeringPosWS = WheelPosWS - VerticalDist * ChassisUp;

    return SteeringPosWS;
}

FVector USWheelSubBody::WorldPosFromCarTransform(const FTransform& CarTransform) const
{
    return CarTransform.TransformPosition(GetLocalOffset() +
        (PSuspension->GetSpringLength() + PSuspension->Setup().MaxLength) * FVector::UpVector);
}

FVector USWheelSubBody::GetSuspensionDirectionWS() const
{
    return -ParentComponent->GetPhysUpVector();
}

float USWheelSubBody::Radius() const
{
    return PWheel->GetEffectiveRadius();
}

float USWheelSubBody::AngularVelocity() const
{
    return Omega;
}

void USWheelSubBody::SetAngularVelocity(const float& InOmega)
{
    Omega = InOmega;
}

float USWheelSubBody::SpringDisplacement() const
{
    return PSuspension->GetSpringLength() + PSuspension->Setup().MaxLength;
}
float USWheelSubBody::SpringLength() const
{
    return PSuspension->GetSpringLength();
}

float USWheelSubBody::GetLastDisplacement() const
{
    return PSuspension->GetLastDisplacement();
}

void USWheelSubBody::SetLastDisplacement(const float& displacement)
{
    PSuspension->SetLastDisplacement(displacement);
}

float USWheelSubBody::MaxLength() const
{
    return PSuspension->Setup().MaxLength;
}

float USWheelSubBody::SuspensionRestLength() const
{
    return ChaosWheel->SuspensionMaxRaise;
}

float USWheelSubBody::SuspensionMaxRaise() const
{
    return ChaosWheel->SuspensionMaxRaise;
}

float USWheelSubBody::SuspensionMaxDrop() const
{
    return ChaosWheel->SuspensionMaxDrop;
}

float USWheelSubBody::GetSuspensionForce() const
{
    return SuspensionForce;
}

bool USWheelSubBody::IsOnGround() const
{
    return PWheel->InContact();
}

void USWheelSubBody::SetOnGround(const bool& on_ground)
{
    bool bWasOnGround = IsOnGround();

    if (!bWasOnGround && on_ground)
    {
        const float NormalSpeed = FVector::DotProduct(ParentComponent->GetPhysVelocityAtPoint(WorldPos()), CurrentHit.ImpactNormal);

        // Real impact
        if (NormalSpeed < -5.0f) // cm/s, to tune
        {
			ParentComponent->RcvImpactOnSubBody(*this, CurrentHit.ImpactPoint);
        }
    }

    // RegisterWheelContact
    if (on_ground)
    {
        ApplyImpulse(FVector::UpVector, CurrentHit.ImpactPoint);
    }

    PWheel->SetOnGround(on_ground);
}

void USWheelSubBody::SetIsJumping(const uint8 NbFrames)
{
    bIsJumping = NbFrames;
}

uint8 USWheelSubBody::IsJumping() const
{
    return bIsJumping;
}

bool USWheelSubBody::IsSteeringEnabled() const
{
    return PWheel->SteeringEnabled;
}

float USWheelSubBody::MaxSteeringAngle() const
{
    return PWheel->MaxSteeringAngle;
}

float USWheelSubBody::GetSuspensionOffset() const
{
    return PSuspension->GetSuspensionOffset() + SuspensionMaxDrop();
}

FVector USWheelSubBody::GetHitContactNormal() const
{
    if (IsOnGround())
    {
        return CurrentHit.ImpactNormal;
    }
    else
    {
        return FVector::ZeroVector;
    }
}

void USWheelSubBody::SetHitContactNormal(const FVector& ImpactNormal)
{
    CurrentHit.ImpactNormal = ImpactNormal;
}

const SWheelRenderData& USWheelSubBody::GetRenderData() const
{
    return RenderData;
}

void USWheelSubBody::SetRollAngle(const float& angle)
{
    FRotator RollRot(RenderData.RollRotation);
    RollRot.Roll = FMath::RadiansToDegrees(angle);
    RenderData.RollRotation = FQuat(RollRot);
}

float USWheelSubBody::GetRollAngle() const
{
    FRotator RollRot(RenderData.RollRotation);
    return FMath::DegreesToRadians(RollRot.Roll);
}

void USWheelSubBody::SetChaosWheel(UChaosVehicleWheel* InChaosWheel)
{
    ChaosWheel = InChaosWheel;
}

void USWheelSubBody::SetWheelSim(Chaos::FSimpleWheelSim* InPWheel)
{
    if (!InPWheel)
    {
        return;
    }
    PWheel = InPWheel;
    SetRadius(PWheel->GetEffectiveRadius());
}

void USWheelSubBody::SetSuspensionSim(Chaos::FSimpleSuspensionSim* InPSuspension)
{
    PSuspension = InPSuspension;
	SetLocalOffset(PSuspension->GetLocalRestingPosition());
}

void USWheelSubBody::SetLocalOffset(const FVector& InLocalOffset)
{
    SetRelativeLocation(InLocalOffset);
}