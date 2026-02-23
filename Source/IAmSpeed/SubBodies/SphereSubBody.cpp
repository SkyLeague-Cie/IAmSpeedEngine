// Fill out your copyright notice in the Description page of Project Settings.


#include "SphereSubBody.h"
#include "BoxSubBody.h"
#include "SWheelSubBody.h"
#include "IAmSpeed/Base/SpeedConstant.h"
#include "IAmSpeed/Components/ISpeedComponent.h"
#include "Configs/SubBodyConfig.h"
#include "PhysicsEngine/BoxElem.h"
#include "PhysicsEngine/BodySetup.h"

DEFINE_LOG_CATEGORY(SphereSubBodyLog);

USphereSubBody::USphereSubBody(const FObjectInitializer& ObjectInitializer):
	Super(ObjectInitializer)
{
	SubBodyType = ESubBodyType::Sphere;
    SetRadius(32.0f);
    InvInertiaLocal = InitInvInertiaTensor();
    ShapeColor = FColor(255, 0, 0, 255);

    bUseEditorCompositing = true;
}

void USphereSubBody::Initialize(ISpeedComponent* InParentComponent)
{
	Super::Initialize(InParentComponent);
    if (ParentComponent)
    {
        SubBodyConfig Config = ParentComponent->GetSubBodyConfig(*this);
        if (Config.bValid)
        {
            SetRadius(FMath::Max(Config.Radius, KINDA_SMALL_NUMBER));
            InvInertiaLocal = InitInvInertiaTensor();
        }
        else
        {
            InvInertiaLocal = InitInvInertiaTensor();
        }
	}
}

void USphereSubBody::ResetForFrame(const float& Delta)
{
    Super::ResetForFrame(Delta);
    GroundHit = SHitResult();
    BoxHit = SHitResult();
    SphereHit = SHitResult();
	WheelHit = SHitResult();
}

//======================================================
// Sweep Methods
//======================================================

bool USphereSubBody::SweepTOI(const float& RemainingDelta, const float& TimePassed, float& OutTOI)
{
    if (!ParentComponent)
    {
        return false;
    }

    float TOI_ground = RemainingDelta;
    float TOI_Box = RemainingDelta;
    float TOI_Sphere = RemainingDelta;
	float TOI_Wheel = RemainingDelta;
    FutureHit = SHitResult();

    SHitResult GroundHitresult;
	SHitResult BoxHitresult;
	SHitResult SphereHitresult;
	SHitResult WheelHitresult;
    bool bHitGround = SweepVsGround(GroundHitresult, RemainingDelta, TOI_ground);
    bool bHitBox = SweepVsBoxes(BoxHitresult, RemainingDelta, TimePassed, TOI_Box);
    bool bHitSphere = SweepVsSpheres(SphereHitresult, RemainingDelta, TimePassed, TOI_Sphere);
	bool bHitWheel = SweepVsWheels(WheelHitresult, RemainingDelta, TimePassed, TOI_Wheel);

    if (!bHitGround && !bHitBox && !bHitSphere && !bHitWheel)
        return false;

    OutTOI = RemainingDelta;

    if (bHitGround && TOI_ground < OutTOI)
    {
        OutTOI = TOI_ground;
		GroundHit = GroundHitresult;
        FutureHit = GroundHit;
    }

    if (bHitBox && TOI_Box < OutTOI)
    {
        OutTOI = TOI_Box;
		BoxHit = BoxHitresult;
        FutureHit = BoxHit;
    }

    if (bHitSphere && TOI_Sphere < OutTOI)
    {
        OutTOI = TOI_Sphere;
		SphereHit = SphereHitresult;
        FutureHit = SphereHit;
    }

    if (bHitWheel && TOI_Wheel < OutTOI)
    {
		OutTOI = TOI_Wheel;
        WheelHit = WheelHitresult;
        FutureHit = WheelHit;
	}

    return true;
}

bool USphereSubBody::SweepVsGround(SHitResult& OutHit, const float& DeltaTime, float& OutTOI)
{
    OutTOI = DeltaTime;
    OutHit = SHitResult();

    const FVector Start = Kinematics.Location;
    const FVector End =
        Start +
        Kinematics.Velocity * DeltaTime +
        0.5f * Kinematics.Acceleration * DeltaTime * DeltaTime;

    SHitResult Hit;
    if (!InternalSweep(Start, End, Hit, DeltaTime))
        return false;

    OutHit = Hit;
    OutTOI = Hit.TOI;
    return true;
}

bool USphereSubBody::SweepVsBoxes(SHitResult& OutHit, const float& DeltaTime, const float& TimePassed, float& OutTOI)
{
    OutTOI = DeltaTime;
    OutHit = SHitResult();
    const SSphere ThisSphere(
        Kinematics.Location,
        GetRadiusWithMargin(),
        Kinematics.Velocity,
        Kinematics.Acceleration
    );

    const uint8 NbSteps = SpeedConstants::NbCCDSubsteps;

    bool bHit = false;
    float BestTime = DeltaTime + 1.f;
    SHitResult LocalBest(false, FVector::ZeroVector, FVector::ZeroVector, 0);
    TWeakObjectPtr<UBoxSubBody> BestBox = nullptr;
    const TArray<TWeakObjectPtr<UBoxSubBody>>& OtherBoxes = ExternalBoxSubBodies;

    for (auto& Box : OtherBoxes)
    {
        if (!Box.IsValid()) continue;

        // Ignore if box's hitbox already hit this frame
        if (IgnoredComponents.Contains(Box.Get()))
            continue;

        SSBox BoxShape = Box->MakeBox(ParentComponent->NumFrame(), TimePassed);
        SHitResult Hit = ThisSphere.IntersectNextFrame(BoxShape, DeltaTime, NbSteps);
        if (!Hit.bHit) continue;

        const float t = Hit.TOI;
        if (t < BestTime)
        {
            BestTime = t;
            LocalBest = Hit;
            BestBox = Box;
            bHit = true;
        }
    }

    if (!bHit)
        return false;

    OutTOI = BestTime;
	OutHit = LocalBest;
    OutHit.Component = BestBox;
	OutHit.bBlockingHit = true;
	OutHit.SubBody = BestBox;

    return true;
}

bool USphereSubBody::SweepVsSpheres(SHitResult& OutHit, const float& Delta, const float& TimePassed, float& OutTOI)
{
    OutTOI = Delta;
    OutHit = SHitResult();
    const SSphere ThisSphere(
        Kinematics.Location,
        GetRadiusWithMargin(),
        Kinematics.Velocity,
        Kinematics.Acceleration
    );

    bool bHit = false;
    float BestTOI = Delta;

    SHitResult BestHit;
    TWeakObjectPtr<USphereSubBody> BestSphere = nullptr;
	const TArray<TWeakObjectPtr<USphereSubBody>>& OtherSpheres = ExternalSphereSubBodies;

    for (auto& OtherSphere : OtherSpheres)
    {
        if (!OtherSphere.IsValid()) continue;

        // Ignore already-hit Spheres this frame
        if (IgnoredComponents.Contains(OtherSphere.Get()))
            continue;

		SSphere OSphere = OtherSphere->MakeSphere(ParentComponent->NumFrame(), Delta, /*TimePassed*/ 0.0f); // useless to pass TimePassed here since every SubBodies are updated to current TimePassed before sweeping

        SHitResult Hit = ThisSphere.IntersectNextFrame(OSphere, Delta);
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
    {
        return false;
    }

    // Fill hit result (DO NOT resolve yet)
	OutHit = BestHit;
    OutHit.bBlockingHit = true;
    OutHit.Component = BestSphere;
    OutHit.SubBody = BestSphere;
    OutTOI = BestTOI;
    return true;
}

bool USphereSubBody::SweepVsWheels(SHitResult& OutHit, const float& Delta, const float& TimePassed, float& OutTOI)
{
    OutTOI = Delta;
    OutHit = SHitResult();
    const SSphere ThisSphere(
        Kinematics.Location,
        GetRadiusWithMargin(),
        Kinematics.Velocity,
        Kinematics.Acceleration
    );

    bool bHit = false;
    float BestTOI = Delta;

    SHitResult BestHit;
    TWeakObjectPtr<USWheelSubBody> BestWheel = nullptr;
    const TArray<TWeakObjectPtr<USWheelSubBody>>& OtherWheels = ExternalWheelSubBodies;

    for (auto& OtherWheel : OtherWheels)
    {
        if (!OtherWheel.IsValid()) continue;

        // Ignore already-hit Wheels this frame
        if (IgnoredComponents.Contains(OtherWheel.Get()))
            continue;

        SSphere OSphere = OtherWheel->MakeSphere(ParentComponent->NumFrame(), Delta, /*TimePassed*/ 0.0f); // useless to pass TimePassed here since every SubBodies are updated to current TimePassed before sweeping

        SHitResult Hit = ThisSphere.IntersectNextFrame(OSphere, Delta);
        if (!Hit.bHit)
            continue;

        if (Hit.TOI < BestTOI)
        {
            BestTOI = Hit.TOI;
            BestHit = Hit;
            BestWheel = OtherWheel;
            bHit = true;
        }
    }

    if (!bHit)
        return false;

    // Fill hit result (DO NOT resolve yet)
    OutHit = BestHit;
    OutHit.bBlockingHit = true;
    OutHit.Component = BestWheel;
    OutHit.SubBody = BestWheel;
    OutTOI = BestTOI;
    return true;
}


//======================================================
// Hit resolution
//======================================================

void USphereSubBody::ResolveCurrentHitPrv(const float& delta, const float& TimePassed, const float& SimTime)
{
    if (!HasHit() || !ParentComponent)
    {
        return;
    }

    UPrimitiveComponent* OtherComponent = CurrentHit.Component.Get();
    USSubBody* OtherSubBody = Cast<USSubBody>(OtherComponent);
	UBoxSubBody* Box = Cast<UBoxSubBody>(OtherSubBody);
	USphereSubBody* Sphere = Cast<USphereSubBody>(OtherSubBody);

    if (OtherComponent && OtherComponent->GetCollisionObjectType() == ECC_WorldStatic)
    {
        ResolveHitVsGround(delta, SimTime);
    }
    else if (Sphere)
    {
        ResolveHitVsSphere(*Sphere, delta, TimePassed, SimTime);
    }
    else if (Box)
    {
        ResolveHitVsBox(*Box, delta, TimePassed, SimTime);
    }
}

void USphereSubBody::ResolveHitVsGround(const float& delta, const float& SimTime)
{
    const FVector N = CurrentHit.ImpactNormal.GetSafeNormal();
    const FVector P = CurrentHit.ImpactPoint;

    // Velocity at contact point
    const FVector Vp = ParentComponent->GetPhysVelocityAtPoint(P);
    const float vN = FVector::DotProduct(Vp, N);

    // ------------------------------------------------------------------
    // IMPACT / SUPPORT SEPARATION
    // ------------------------------------------------------------------

    // if it is a real impact
    if (vN < - GetImpactThreshold())
    {
        // ------------------------------------------------------------------
        // 1) NORMAL IMPULSE (bounce)
        // ------------------------------------------------------------------
        const float Rest = GetRestitution();

        // For a sphere vs plane: no angular coupling on normal
        const float jn = -(1.f + Rest) * vN * GetMass();
        ApplyImpulse(jn * N, P);
        // ParentComponent->NotifyLocalImpulseEvent();

        // ------------------------------------------------------------------
        // 2) TANGENTIAL IMPULSE (impact friction -> spin transfer)
        // ------------------------------------------------------------------
        const FVector VpPostN = ParentComponent->GetPhysVelocityAtPoint(P);
        const float vNpost = FVector::DotProduct(VpPostN, N);
        const FVector vT = VpPostN - vNpost * N;
        const float vTmag = vT.Size();

        if (vTmag > KINDA_SMALL_NUMBER)
        {
            const FVector T = vT / vTmag;

            // Effective mass for solid sphere rolling friction impulse
            // m_eff = 2m / 7
            const float mEff = (2.f / 7.f) * GetMass();

            float jt = -vTmag * mEff;

            // Coulomb clamp
            const float mu = GetStaticFriction();
            const float jtMax = mu * jn;
            jt = FMath::Clamp(jt, -jtMax, jtMax);

            ApplyImpulse(jt * T, P);
        }
    }

    // ------------------------------------------------------------------
    // 3) MICRO-OSCILLATION PREVENTION (VERY IMPORTANT)
    // ------------------------------------------------------------------
    HandleMicroOscillation();
}

void USphereSubBody::ResolveHitVsBox(UBoxSubBody& OtherBox, const float& delta, const float& TimePassed, const float& SimTime)
{
    //===============================
    // 1) Access kinematics at TOI
    //===============================
    const unsigned frame = ParentComponent->NumFrame();

    // Box state at TOI (its PhysicsTick may not have run yet)
	SSBox BBox = OtherBox.MakeBox(frame, /*TimePassed*/ 0.0f); // useless to pass TimePassed here since every SubBodies are updated to current TimePassed before sweeping
    SKinematic BoxKSAtTOI;
    BoxKSAtTOI.Location = BBox.WorldCenter;
    BoxKSAtTOI.Velocity = BBox.Vel;
    BoxKSAtTOI.Acceleration = BBox.Accel;
    BoxKSAtTOI.Rotation = BBox.Rot;
    BoxKSAtTOI.AngularVelocity = BBox.AngVel;
    BoxKSAtTOI.AngularAcceleration = BBox.AngAccel;

    // Sphere state at TOI (already integrated)
    const SKinematic& SphereKS = Kinematics;
    /*
    // --- Overlap branch sphere-box ---
    {
        const float epsTOI = 1e-6f;

        FVector Cp;
        const float sep = BBox.SphereOBBSeparation(BBox.Rot, BBox.WorldCenter, SphereKS.Location, GetRadius(), &Cp);
        const float pen = FMath::Max(0.f, -sep);

        if (pen > MinSlopCm)
        {
            FVector N = (SphereKS.Location - Cp); // Box -> Sphere
            if (!N.Normalize())
                N = BBox.UpVector();

            const FVector P = SphereKS.Location - N * GetRadius();

            ISpeedComponent* OtherComp = OtherBox.GetParentComponent(); // needs access; ok if you add getter

            SolveOverlap(
                *ParentComponent, GetMass(), ComputeWorldInvInertiaTensor(), SphereKS,
                OtherComp, OtherBox.GetMass(), OtherBox.ComputeWorldInvInertiaTensor(), BoxKSAtTOI,
                P, N,
                pen,
                0.05f, 0.8f,
                GetImpactThreshold()
            );

            if (IsFakePhysicsEnabled())
            {
				// UpdateKinematicsFromOwner(frame); // update kinematics to current frame before applying fake physics
				// OtherBox.UpdateKinematicsFromOwner(frame);
                ApplyFakePhysicsOn(OtherBox, CurrentHit, delta);
            }
            if (OtherBox.IsFakePhysicsEnabled())
            {
                SHitResult Inv = CurrentHit; Inv.ImpactNormal *= -1.f;
				// UpdateKinematicsFromOwner(frame); // update kinematics to current frame before applying fake physics
				// OtherBox.UpdateKinematicsFromOwner(frame);
                OtherBox.ApplyFakePhysicsOn(*this, Inv, delta);
            }
            return;
        }
    }*/

	// Compute collision impulse at TOI
    FVector ImpThis, ImpOther;
    bool ok = Speed::SImpulseSolver::ComputeCollisionImpulse(
        CurrentHit.ImpactPoint,
        CurrentHit.ImpactNormal,   // Other -> Sphere

        SphereKS, GetMass(), ComputeWorldInvInertiaTensor(),
        BoxKSAtTOI, OtherBox.GetMass(), OtherBox.ComputeWorldInvInertiaTensor(),

        MixRestitution(GetRestitution(), OtherBox.GetRestitution(), EMixMode::E_Max),
        MixFriction(GetStaticFriction(), OtherBox.GetStaticFriction(), EMixMode::E_Max),

        ImpThis,
        ImpOther,
        GetImpactThreshold()
    );

    if (ok)
    {
        // Debug
        /*UE_LOG(SphereSubBodyLog, Log, TEXT("[CCDSphere] Resolving Box hit for frame = %d, with ImpactPoint = %s, ImpactNormal = %s, at TOI = %fs"),
            ParentComponent->NumFrame(), *CurrentHit.ImpactPoint.ToString(), *CurrentHit.ImpactNormal.ToString(), TimePassed);
        UE_LOG(SphereSubBodyLog, Log, TEXT("[CCDSphere] Box KS at TOI: %s"), *BoxKSAtTOI.ToString());
        UE_LOG(SphereSubBodyLog, Log, TEXT("[CCDSphere] Sphere KS at TOI: %s"), *SphereKS.ToString());*/
        // UE_LOG(SphereSubBodyLog, Log, TEXT("[%s][CCDSphere] Sphere applies impulse %s at TOI = %fs at frame=%d"), *ParentComponent->GetRole(),
        //    *ImpThis.ToString(), TimePassed, ParentComponent->NumFrame());

        // Apply impulses
        ApplyImpulse(ImpThis, CurrentHit.ImpactPoint);
		OtherBox.ApplyImpulse(ImpOther, CurrentHit.ImpactPoint);

        // apply fake physics
        if (IsFakePhysicsEnabled())
        {
			// UpdateKinematicsFromOwner(frame); // update kinematics to current frame before applying fake physics
			// OtherBox.UpdateKinematicsFromOwner(frame);
            ApplyFakePhysicsOn(OtherBox, CurrentHit, delta);
        }
        if (OtherBox.IsFakePhysicsEnabled())
        {
			SHitResult InvertedHit = CurrentHit;
			InvertedHit.ImpactNormal *= -1.f; // invert normal for correct fake physics application on the box
			// UpdateKinematicsFromOwner(frame); // update kinematics to current frame before applying fake physics
			// OtherBox.UpdateKinematicsFromOwner(frame);
            OtherBox.ApplyFakePhysicsOn(*this, InvertedHit, delta);
		}
    }

    // ------------------------------------------------------------------
    // 3) MICRO-OSCILLATION PREVENTION (VERY IMPORTANT)
    // ------------------------------------------------------------------
	HandleMicroOscillation();
}

void USphereSubBody::ResolveHitVsSphere(USphereSubBody& OtherSphere, const float& delta, const float& TimePassed, const float& SimTime)
{
    const float TOI = CurrentHit.TOI;

    // --- Sphere kinematics at TOI (already advanced) ---
    const SKinematic& ThisKS = Kinematics;

    // --- Sphere kinematics at TOI ---
    SSphere OSSphere = OtherSphere.MakeSphere(GetParentComponent()->NumFrame(), delta, /*TimePassed*/ 0.0f); // useless to pass TimePassed here since every SubBodies are updated to current TimePassed before sweeping
    SKinematic OtherKS;
    OtherKS.Location = OSSphere.Center;
    OtherKS.Velocity = OSSphere.Vel;
    OtherKS.Acceleration = OSSphere.Accel;
    FVector ImpThis, ImpOther;

    // --- Overlap branch ---
    /* {
        const float epsTOI = 1e-6f;

        const FVector pA = ThisKS.Location;
        const FVector pB = OtherKS.Location;
        const float rA = GetRadius();
        const float rB = OtherSphere.GetRadius();
        const float rSum = rA + rB;

        FVector N = (pA - pB); // Other -> This
        float dist = N.Size();
        if (dist > KINDA_SMALL_NUMBER) N /= dist;
        else {
            const FVector vRel = ThisKS.Velocity - OtherKS.Velocity;
            N = vRel.SizeSquared() > KINDA_SMALL_NUMBER ? vRel.GetSafeNormal() : FVector::UpVector;
            dist = 0.f;
        }

        const float pen = rSum - dist;
        if (pen > MinSlopCm)
        {
            const FVector P = pA - N * rA;

            // Need access to other component to move it too.
            ISpeedComponent* OtherComp = OtherSphere.ParentComponent;

            SolveOverlap(
                *ParentComponent, GetMass(), ComputeWorldInvInertiaTensor(), ThisKS,
                OtherComp, OtherSphere.GetMass(), OtherSphere.ComputeWorldInvInertiaTensor(), OtherKS,
                P, N,
                pen,
                0.05f, // Slop
                0.8f, // percent
                GetImpactThreshold() // KillVelThreshold or 5..20
            );

            // Optional: small symmetric position correction already done; now fake physics
            if (IsFakePhysicsEnabled())
            {
				// UpdateKinematicsFromOwner(ParentComponent->NumFrame()); // update kinematics with latest position/velocity from physics (important for correct fake physics application)
				// OtherSphere.UpdateKinematicsFromOwner(ParentComponent->NumFrame());
                ApplyFakePhysicsOn(OtherSphere, CurrentHit, delta);
            }
            if (OtherSphere.IsFakePhysicsEnabled())
            {
                SHitResult Inv = CurrentHit; Inv.ImpactNormal *= -1.f;
				// UpdateKinematicsFromOwner(ParentComponent->NumFrame()); // update kinematics with latest position/velocity from physics (important for correct fake physics application)
				// OtherSphere.UpdateKinematicsFromOwner(ParentComponent->NumFrame());
                OtherSphere.ApplyFakePhysicsOn(*this, Inv, delta);
            }
            return;
        }
    }*/

	// --- Compute impulse ---
    const bool bOk = Speed::SImpulseSolver::ComputeCollisionImpulse(
        CurrentHit.ImpactPoint,
        CurrentHit.ImpactNormal, // Other -> This

        ThisKS, GetMass(), ComputeWorldInvInertiaTensor(),
        OtherKS, OtherSphere.GetMass(), OtherSphere.ComputeWorldInvInertiaTensor(),

        MixRestitution(GetRestitution(), OtherSphere.GetRestitution(), EMixMode::E_Min),
        MixFriction(GetStaticFriction(), OtherSphere.GetStaticFriction(), EMixMode::E_Min),
        ImpThis, ImpOther,
        GetImpactThreshold()
    );

    if (IsMainSubBody())
    {
        // Small position correction
        FVector correctPos = CurrentHit.ImpactPoint + CurrentHit.ImpactNormal * GetRadius();
        ParentComponent->SetPhysLocation(correctPos);
    }

    if (!bOk)
        return;

    /*UE_LOG(SphereSubBodyLog, Log, TEXT("[CCDSphere] Resolving Sphere(%d) hit for frame = %d, with ImpactPoint = %s, ImpactNormal = %s, at TOI = %fs"),
        OtherSphere.Idx(), ParentComponent->NumFrame(), *CurrentHit.ImpactPoint.ToString(), *CurrentHit.ImpactNormal.ToString(), TimePassed);
    UE_LOG(SphereSubBodyLog, Log, TEXT("[CCDSphere] Sphere KS at TOI: %s"),
        *ThisKS.ToString());
    UE_LOG(SphereSubBodyLog, Log, TEXT("[CCDSphere] Other KS at TOI: %s"),
        *OtherKS.ToString());
    UE_LOG(SphereSubBodyLog, Log, TEXT("[CCDSphere] Sphere applies impulse %s on OtherSphere at TOI = %fs"),
        *ImpThis.ToString(), TimePassed);*/

        // --- Apply impulse on Sphere only ---
    ApplyImpulse(ImpThis, CurrentHit.ImpactPoint);
	OtherSphere.ApplyImpulse(ImpOther, CurrentHit.ImpactPoint);

	// apply fake physics
    if (IsFakePhysicsEnabled())
    {
		// UpdateKinematicsFromOwner(ParentComponent->NumFrame()); // update kinematics with latest position/velocity from physics (important for correct fake physics application)
		// OtherSphere.UpdateKinematicsFromOwner(ParentComponent->NumFrame());
        ApplyFakePhysicsOn(OtherSphere, CurrentHit, delta);
    }
    if (OtherSphere.IsFakePhysicsEnabled())
    {
		// UpdateKinematicsFromOwner(ParentComponent->NumFrame()); // update kinematics with latest position/velocity from physics (important for correct fake physics application)
		// OtherSphere.UpdateKinematicsFromOwner(ParentComponent->NumFrame());
        OtherSphere.ApplyFakePhysicsOn(*this, OtherSphere.GetHit(), delta);
	}
}

void USphereSubBody::HandleMicroOscillation()
{
    if (!ParentComponent || !IsMainSubBody() || !CurrentHit.Component.IsValid())
    {
        return;
    }
    FVector vel = ParentComponent->GetPhysVelocity();
    float vN = FVector::DotProduct(vel, CurrentHit.ImpactNormal);
    FVector VelN = vN * CurrentHit.ImpactNormal;
    if (vel.Size() < 0.01f)
    {
        ParentComponent->AddPhysVelocity(-vel);
    }
    else if (FMath::Abs(vN) <= GetImpactThreshold())
    {
        // Remove tiny separating normal velocity
        ParentComponent->AddPhysVelocity(-VelN);
    }

    // Small position correction
    FVector correctPos = CurrentHit.ImpactPoint + CurrentHit.ImpactNormal * GetRadius();
    ParentComponent->SetPhysLocation(correctPos);
}


//======================================================
// Helpers
//======================================================

FCollisionShape USphereSubBody::GetCollisionShape(float Inflation) const
{
    return FCollisionShape::MakeSphere(GetRadiusWithMargin());
}

SSphere USphereSubBody::MakeSphere(const unsigned int& NumFrame, const float& RemainingDelta, const float& TimePassed) const
{
    SKinematic KS = GetKinematicsFromOwner(NumFrame);
    KS = KS.Integrate(TimePassed);
    return SSphere(
        KS.Location,
        GetRadiusWithMargin(),
        KS.Velocity,
        KS.Acceleration
    );
}

SKinematic USphereSubBody::GetKinematicsFromOwner(const unsigned int& NumFrame) const
{
	SKinematic K;
    if (!ParentComponent)
    {
		return K;
    }

    return ParentComponent->GetKinematicsOfSubBody(*this, NumFrame);
}

float USphereSubBody::GetRadiusWithMargin() const
{
    return GetRadius() + CollisionMargin();
}

FMatrix USphereSubBody::InitInvInertiaTensor() const
{
    // Inertia tensor for solid sphere: I = 2/5 m r^2
    const float I = (2.f / 5.f) * GetMass() * GetRadius() * GetRadius();
    const float InvI = I > KINDA_SMALL_NUMBER ? 1.f / I : 0.f;
    return FMatrix(
        FPlane(InvI, 0.f, 0.f, 0.f),
        FPlane(0.f, InvI, 0.f, 0.f),
        FPlane(0.f, 0.f, InvI, 0.f),
        FPlane(0.f, 0.f, 0.f, 1.f)
	);
}

FMatrix USphereSubBody::ComputeWorldInvInertiaTensor() const
{
    return InvInertiaLocal;
}

template <EShapeBodySetupHelper UpdateBodySetupAction, typename BodySetupType>
bool InvalidateOrUpdateSphereBodySetup(BodySetupType& ShapeBodySetup, bool bUseArchetypeBodySetup, float SphereRadius)
{
    check((bUseArchetypeBodySetup && UpdateBodySetupAction == EShapeBodySetupHelper::InvalidateSharingIfStale) || (!bUseArchetypeBodySetup && UpdateBodySetupAction == EShapeBodySetupHelper::UpdateBodySetup));
    check(ShapeBodySetup->AggGeom.SphereElems.Num() == 1);
    FKSphereElem* SphereElem = ShapeBodySetup->AggGeom.SphereElems.GetData();

    // check for mal formed values
    float Radius = SphereRadius;
    if (Radius < UE_KINDA_SMALL_NUMBER)
    {
        Radius = 0.1f;
    }

    if (UpdateBodySetupAction == EShapeBodySetupHelper::UpdateBodySetup)
    {
        // now set the PhysX data values
        SphereElem->Center = FVector::ZeroVector;
        SphereElem->Radius = Radius;
    }
    else
    {
        if (SphereElem->Radius != Radius)
        {
            ShapeBodySetup = nullptr;
            bUseArchetypeBodySetup = false;
        }
    }

    return bUseArchetypeBodySetup;
}

FPrimitiveSceneProxy* USphereSubBody::CreateSceneProxy()
{
    /** Represents a DrawLightRadiusComponent to the scene manager. */
    class FSphereSceneProxy final : public FPrimitiveSceneProxy
    {
    public:
        SIZE_T GetTypeHash() const override
        {
            static size_t UniquePointer;
            return reinterpret_cast<size_t>(&UniquePointer);
        }

        /** Initialization constructor. */
        FSphereSceneProxy(const USphereSubBody* InComponent)
            : FPrimitiveSceneProxy(InComponent)
            , bDrawOnlyIfSelected(InComponent->bDrawOnlyIfSelected)
            , SphereColor(InComponent->ShapeColor)
            , SphereRadius(InComponent->Radius)
            , LineThickness(InComponent->LineThickness)
        {
            bWillEverBeLit = false;
        }

        // FPrimitiveSceneProxy interface.

        virtual void GetDynamicMeshElements(const TArray<const FSceneView*>& Views, const FSceneViewFamily& ViewFamily, uint32 VisibilityMap, FMeshElementCollector& Collector) const override
        {
            QUICK_SCOPE_CYCLE_COUNTER(STAT_SphereSceneProxy_GetDynamicMeshElements);

            for (int32 ViewIndex = 0; ViewIndex < Views.Num(); ViewIndex++)
            {
                if (VisibilityMap & (1 << ViewIndex))
                {
                    const FSceneView* View = Views[ViewIndex];
                    FPrimitiveDrawInterface* PDI = Collector.GetPDI(ViewIndex);

                    const FMatrix& LocalToWorld = GetLocalToWorld();
                    const FLinearColor DrawSphereColor = GetViewSelectionColor(SphereColor, *View, IsSelected(), IsHovered(), false, IsIndividuallySelected());

                    // Taking into account the min and maximum drawing distance
                    const float DistanceSqr = (View->ViewMatrices.GetViewOrigin() - LocalToWorld.GetOrigin()).SizeSquared();
                    if (DistanceSqr < FMath::Square(GetMinDrawDistance()) || DistanceSqr > FMath::Square(GetMaxDrawDistance()))
                    {
                        continue;
                    }

                    float AbsScaleX = LocalToWorld.GetScaledAxis(EAxis::X).Size();
                    float AbsScaleY = LocalToWorld.GetScaledAxis(EAxis::Y).Size();
                    float AbsScaleZ = LocalToWorld.GetScaledAxis(EAxis::Z).Size();
                    float MinAbsScale = FMath::Min3(AbsScaleX, AbsScaleY, AbsScaleZ);

                    FVector ScaledX = LocalToWorld.GetUnitAxis(EAxis::X) * MinAbsScale;
                    FVector ScaledY = LocalToWorld.GetUnitAxis(EAxis::Y) * MinAbsScale;
                    FVector ScaledZ = LocalToWorld.GetUnitAxis(EAxis::Z) * MinAbsScale;

                    const int32 SphereSides = FMath::Clamp<int32>(SphereRadius / 4.f, 16, 64);
                    DrawCircle(PDI, LocalToWorld.GetOrigin(), ScaledX, ScaledY, DrawSphereColor, SphereRadius, SphereSides, SDPG_World, LineThickness);
                    DrawCircle(PDI, LocalToWorld.GetOrigin(), ScaledX, ScaledZ, DrawSphereColor, SphereRadius, SphereSides, SDPG_World, LineThickness);
                    DrawCircle(PDI, LocalToWorld.GetOrigin(), ScaledY, ScaledZ, DrawSphereColor, SphereRadius, SphereSides, SDPG_World, LineThickness);
                }
            }
        }

        virtual FPrimitiveViewRelevance GetViewRelevance(const FSceneView* View) const override
        {
            const bool bVisibleForSelection = !bDrawOnlyIfSelected || IsSelected();
            const bool bVisibleForShowFlags = true; // @TODO

            // Should we draw this because collision drawing is enabled, and we have collision
            const bool bShowForCollision = View->Family->EngineShowFlags.Collision && IsCollisionEnabled();

            FPrimitiveViewRelevance Result;
            Result.bDrawRelevance = (IsShown(View) && bVisibleForSelection && bVisibleForShowFlags) || bShowForCollision;
            Result.bDynamicRelevance = true;
            Result.bShadowRelevance = IsShadowCast(View);
            Result.bEditorPrimitiveRelevance = UseEditorCompositing(View);
            return Result;
        }

        virtual uint32 GetMemoryFootprint(void) const override { return(sizeof(*this) + GetAllocatedSize()); }
        uint32 GetAllocatedSize(void) const { return(FPrimitiveSceneProxy::GetAllocatedSize()); }

    private:
        const uint32				bDrawOnlyIfSelected : 1;
        const FColor				SphereColor;
        const float					SphereRadius;
        const float					LineThickness;
    };

    return new FSphereSceneProxy(this);
}

FBoxSphereBounds USphereSubBody::CalcBounds(const FTransform& LocalToWorld) const
{
    return FBoxSphereBounds(FVector::ZeroVector, FVector(Radius), Radius).TransformBy(LocalToWorld);
}

void USphereSubBody::UpdateBodySetup()
{
    if (PrepareSharedBodySetup<USphereSubBody>())
    {
        bUseArchetypeBodySetup = InvalidateOrUpdateSphereBodySetup<EShapeBodySetupHelper::InvalidateSharingIfStale>(ShapeBodySetup, bUseArchetypeBodySetup, Radius);
    }

    CreateShapeBodySetupIfNeeded<FKSphereElem>();

    if (!bUseArchetypeBodySetup)
    {
        InvalidateOrUpdateSphereBodySetup<EShapeBodySetupHelper::UpdateBodySetup>(ShapeBodySetup, bUseArchetypeBodySetup, Radius);
    }
}

void USphereSubBody::SetRadius(const float& NewRadius)
{
    Radius = NewRadius;
}
