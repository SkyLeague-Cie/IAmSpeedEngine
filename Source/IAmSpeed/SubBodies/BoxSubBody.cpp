// Fill out your copyright notice in the Description page of Project Settings.


#include "BoxSubBody.h"
#include "IAmSpeed/Base/SpeedConstant.h"
#include "IAmSpeed/Components/ISpeedComponent.h"
#include "SphereSubBody.h"
#include "SWheelSubBody.h"
#include "Configs/SubBodyConfig.h"
#include "PhysicsEngine/BoxElem.h"
#include "PhysicsEngine/BodySetup.h"

DEFINE_LOG_CATEGORY(BoxSubBodyLog);

UBoxSubBody::UBoxSubBody(const FObjectInitializer& ObjectInitializer):
	Super(ObjectInitializer)
#if WITH_EDITOR
    , ShowFlags(ESFIM_All0)
#endif // WITH_EDITOR
{
	SubBodyType = ESubBodyType::Hitbox;
    bUseEditorCompositing = true;
    BoxExtent = FVector(32.f, 32.f, 32.f);
}

void UBoxSubBody::Initialize(ISpeedComponent* InParentComponent)
{
	Super::Initialize(InParentComponent);
	if (ParentComponent)
	{
		SubBodyConfig Config = ParentComponent->GetSubBodyConfig(*this);
        if (Config.bValid)
        {
            BoxExtent = Config.BoxExtent;
			InvInertiaLocal = InitInvInertiaTensor();
        }
        else
        {
            InvInertiaLocal = InitInvInertiaTensor();
        }
	}
}

void UBoxSubBody::ResetForFrame(const float& Delta)
{
    Super::ResetForFrame(Delta);

    HitCountThisFrame.Empty();
    CurrentGroundContactsWS.Empty();
    CurrentGroundNormalsWS.Empty();

    bGroundHitFromSweep = false;
    bHasGroundContact = false;
}

void UBoxSubBody::AcceptHit()
{
    CurrentHit = FutureHit;
    FutureHit = SHitResult();

    if (!CurrentHit.Component.IsValid())
        return;

    UPrimitiveComponent* Comp = CurrentHit.Component.Get();
    if (CurrentHit.Component.IsValid() && CurrentHit.Component->GetMobility() == EComponentMobility::Static)
    {
        uint32& Count = HitCountThisFrame.FindOrAdd(Comp);
        Count++;

        // Ignore only after several hits on the same component in the same frame
        if (Count >= MaxHitsPerComponentPerFrame)
        {
            IgnoredComponents.AddUnique(Comp);
        }
    }
    else
    {
        IgnoredComponents.AddUnique(Comp);
    }
    IgnoredComponents.AddUnique(Comp);

    if (CurrentHit.SubBody.IsValid())
    {
        // if Other sub-body will hit, accept the hit for it too to ensure both sides are in sync and won't try to resolve the same hit again
        auto OtherSubBody = CurrentHit.SubBody.Get();
        if (OtherSubBody->WillHit())
        {
            OtherSubBody->AcceptHit();
        }
    }
}

//======================================================
// =============== Sweep Methods =======================
// =====================================================
bool UBoxSubBody::SweepTOI(const float& RemainingDelta, const float& TimePassed, float& OutTOI)
{
    if (!ParentComponent)
    {
        return false;
    }

    float TOI_ground = RemainingDelta;
    float TOI_Sphere = RemainingDelta;
    float TOI_Box = RemainingDelta;
    float TOI_Wheel = RemainingDelta;
    FutureHit = SHitResult();
    SHitResult OldGroundHit = GroundHit;

    SHitResult GroundHitresult;
    SHitResult BoxHitresult;
    SHitResult SphereHitresult;
	SHitResult WheelHitresult;
    bool bHitGround = SweepVsGround(GroundHitresult, RemainingDelta, TimePassed, TOI_ground);
    bool bHitBox = SweepVsBoxes(BoxHitresult, RemainingDelta, TimePassed, TOI_Box);
    bool bHitSphere = SweepVsSpheres(SphereHitresult, RemainingDelta, TimePassed, TOI_Sphere);
    bool bHitWheel = SweepVsWheels(WheelHitresult, RemainingDelta, TimePassed, TOI_Wheel);
    if (!bHitGround && !bHitBox && !bHitSphere && !bHitWheel)
    {
        // IMPORTANT : ground can exist even without a hit (persistent contact)
        if (HasPersistentGroundContact())
        {
            const FVector N = OldGroundHit.ImpactNormal.GetSafeNormal();
            const FVector vP = GetVelocityAtPoint(OldGroundHit.ImpactPoint); // 
            const float vN = FVector::DotProduct(vP, N);

            // DO NOT recreate contact if separating
            if (vN <= 0.f)
            {
                if (!IgnoredComponents.Contains(OldGroundHit.Component))
                {
                    OutTOI = 0.0f;
                    FutureHit = OldGroundHit; // reuse current ground hit
                    GroundHit = OldGroundHit;
                    FutureHit.TOI = 0.0f;
                    return true;
                }
                else
                {
                    GroundHit = OldGroundHit;
                }
            }
        }
        else
        {
            ParentComponent->SetIsUpsideDown(false);
        }
        return false;
    }

    OutTOI = RemainingDelta;

    if (bHitGround && TOI_ground < OutTOI)
    {
        OutTOI = TOI_ground;
		GroundHit = GroundHitresult;
        FutureHit = GroundHit;
    }

    if (bHitSphere && TOI_Sphere < OutTOI)
    {
        OutTOI = TOI_Sphere;
		SphereHit = SphereHitresult;
        FutureHit = SphereHit;
    }

    if (bHitBox && TOI_Box < OutTOI)
    {
        OutTOI = TOI_Box;
		BoxHit = BoxHitresult;
        FutureHit = BoxHit;
    }

    if (bHitWheel && TOI_Wheel < OutTOI)
    {
        OutTOI = TOI_Wheel;
		WheelHit = WheelHitresult;
        FutureHit = WheelHit;
    }

    return true;
}


bool UBoxSubBody::SweepVsGround(SHitResult& OutHit, const float& Delta, const float& TimePassed, float& OutTOI)
{
    OutTOI = Delta;
    // GroundHit = SHitResult();

    if (!ParentComponent)
        return false;

    // Kinematics at TimePassed
    // Kinematics = GetKinematicsFromOwner(ParentComponent->NumFrame());

    const FVector Start = Kinematics.Location;

    const FVector End =
        Start +
        Kinematics.Velocity * Delta +
        0.5f * Kinematics.Acceleration * Delta * Delta;

    SHitResult Hit;
    bGroundHitFromSweep = false;
    // SSBox CarBox(Kinematics.Location, BoxExtent, Kinematics.Rotation, Kinematics.Velocity, Kinematics.Acceleration, Kinematics.AngularVelocity, Kinematics.AngularAcceleration);
    // CarBox.DrawDebug(GetWorld());
    if (!InternalSweep(Start, End, Hit, Delta))
        return false;

    auto AddGroundNormal = [&](const FVector& Nnew)
        {
            const FVector nn = Nnew.GetSafeNormal();
            for (const FVector& Nprev : CurrentGroundNormalsWS)
            {
                if (FVector::DotProduct(nn, Nprev) > 0.995f) // ~5°
                    return; // already have essentially same normal
            }
            CurrentGroundNormalsWS.Add(nn);
        };


    OutHit = Hit;
    OutTOI = Hit.TOI;
    bGroundHitFromSweep = true;
    AddGroundNormal(OutHit.ImpactNormal);
    return true;
}

bool UBoxSubBody::SweepVsSpheres(SHitResult& OutHit, const float& Delta, const float& TimePassed, float& OutTOI)
{
    OutTOI = Delta;
    OutHit = SHitResult();
    bool bHit = false;
    float BestTOI = Delta;

    // --- Build hitbox OBB from SubBody kinematics ---
    SSBox ThisBox = MakeBox(ParentComponent->NumFrame(), 0.0f); // no need to integrate TimePassed because Box is already at TimePassed

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

        SHitResult Hit = ThisBox.IntersectNextFrame(OSphere, Delta, SpeedConstants::NbCCDSubsteps);
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

    // --- Fill OutHit ---
    OutHit = BestHit;
    OutHit.bBlockingHit = true;
    OutHit.Component = BestSphere;
    OutHit.SubBody = BestSphere;
    OutTOI = BestHit.TOI;
    return true;
}

bool UBoxSubBody::SweepVsWheels(SHitResult& OutHit, const float& Delta, const float& TimePassed, float& OutTOI)
{
    OutTOI = Delta;
    OutHit = SHitResult();
    bool bHit = false;
    float BestTOI = Delta;
    // --- Build hitbox OBB from SubBody kinematics ---
    SSBox ThisBox = MakeBox(ParentComponent->NumFrame(), 0.0f); // no need to integrate TimePassed because Box is already at TimePassed
    SHitResult BestHit;
    TWeakObjectPtr<USWheelSubBody> BestWheel = nullptr;
    const TArray<TWeakObjectPtr<USWheelSubBody>>& OtherWheels = ExternalWheelSubBodies;
    for (auto& OtherWheel : OtherWheels)
    {
        if (!OtherWheel.IsValid()) continue;

        // Ignore already-hit Wheels this frame
        if (IgnoredComponents.Contains(OtherWheel.Get()))
            continue;

        SSphere OWheel = OtherWheel->MakeSphere(ParentComponent->NumFrame(), Delta, /*TimePassed*/ 0.0f); // useless to pass TimePassed here since every SubBodies are updated to current TimePassed before sweeping

        SHitResult Hit = ThisBox.IntersectNextFrame(OWheel, Delta, SpeedConstants::NbCCDSubsteps);
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

    // --- Fill OutHit ---
    OutHit = BestHit;
    OutHit.bBlockingHit = true;
    OutHit.Component = BestWheel;
    OutHit.SubBody = BestWheel;
    OutTOI = BestHit.TOI;
	return true;
}

bool UBoxSubBody::SweepVsBoxes(SHitResult& OutHit, const float& Delta, const float& TimePassed, float& OutTOI)
{
    OutTOI = Delta;

    if (!ParentComponent)
        return false;

    BoxHit = SHitResult();

    bool bHit = false;
    float BestTOI = Delta + 1.f;
    SHitResult BestHit;
    SHitResult LocalBest;

    // This hitbox (already at TimePassed)
    SSBox ThisBox = MakeBox(ParentComponent->NumFrame(), 0.0f);
    TWeakObjectPtr<UBoxSubBody> BestBox = nullptr;
	const TArray<TWeakObjectPtr<UBoxSubBody>>& OtherBoxes = ExternalBoxSubBodies;

    for (auto& Box : OtherBoxes)
    {
        if (!Box.IsValid()) continue;

        // Ignore if box's hitbox already hit this frame
        if (IgnoredComponents.Contains(Box.Get()))
            continue;

		SSBox BoxShape = Box->MakeBox(ParentComponent->NumFrame(), /*TimePassed*/ 0.0f); // useless to pass TimePassed here since every SubBodies are updated to current TimePassed before sweeping
        SHitResult Hit = ThisBox.IntersectNextFrame(BoxShape, Delta, SpeedConstants::NbCCDSubsteps);
        if (!Hit.bHit) continue;

        const float t = Hit.TOI;
        if (t < BestTOI)
        {
            BestTOI = t;
            LocalBest = Hit;
            BestBox = Box;
            bHit = true;
        }
    }

    if (!bHit)
        return false;

    // Fill BoxHit (future hit result for selection)
    BoxHit.bBlockingHit = true;
    OutHit = LocalBest;
    OutHit.Component = BestBox;
    OutHit.bBlockingHit = true;
    OutHit.SubBody = BestBox;

    OutTOI = BestTOI;
    return true;
}

// ============================================================================
// ======= ResolveHit Methods =================================================
// ============================================================================

void UBoxSubBody::ResolveCurrentHitPrv(const float& delta, const float& TimePassed, const float& SimTime)
{
    if (!CurrentHit.bBlockingHit || !ParentComponent)
        return;

    USphereSubBody* Sphere = Cast<USphereSubBody>(CurrentHit.Component.Get());
    UBoxSubBody* Box = Cast<UBoxSubBody>(CurrentHit.Component.Get());

    if (CurrentHit.Component.IsValid() &&
        CurrentHit.Component->GetCollisionObjectType() == ECC_WorldStatic)
    {
        ResolveHitVsGround(delta, TimePassed);
    }
    else if (Sphere)
    {
        ResolveHitVsSphere(*Sphere, delta, TimePassed);
    }
    else if (Box)
    {
        ResolveHitVsBox(*Box, delta, TimePassed);
    }
}


// ============================================================================
//  ResolveHitVsGround
//     - Orchestrates: classify -> event solve -> update persistence -> apply support.
//     - Handles concave rule consistently.
// ============================================================================
void UBoxSubBody::ResolveHitVsGround(const float& Dt, const float& TimePassed)
{
    if (!ParentComponent) return;

    // Update composite normal set (you already maintain CurrentGroundNormalsWS)
    CompositeGroundNormal = BuildCompositeGroundNormal();

    const FVector N = CurrentHit.ImpactNormal.GetSafeNormal();
    const float UpDot = FVector::DotProduct(N, FVector::UpVector);

    const bool bIsSupportingContact = (UpDot > 0.85f);
    // Kinematics = GetKinematicsFromOwner(ParentComponent->NumFrame());
    const FVector HitboxUpVector = Kinematics.Rotation.GetUpVector();

    if (FMath::Abs(HitboxUpVector.Z) > 0.75f)
    {
        ParentComponent->SetIsUpsideDown(true);
    }

    // Early exit if separating at the impact point.
    constexpr float VN_EPS = 1.0f; // cm/s
    const float vN_atImpact = FVector::DotProduct(GetVelocityAtPoint(CurrentHit.ImpactPoint), N);
    // UE_LOG(BoxSubBodyLog, Log, TEXT("[CCDBox] Start of Hit vs Ground for frame = %d, vN_start = %.2f cm/s, bIsSupportingContact = %d, Box Kinematic = %s"),
    //    ParentComponent->NumFrame(), vN_atImpact, bIsSupportingContact, *ParentComponent->GetCurrentKinematicState().ToString());
    if (vN_atImpact >= VN_EPS)
    {
        // Still update persistence (you might be resting and sweep didn't hit)
        UpdatePersistentGroundContact(Dt);
        const bool bEdgeRecoverActive =
            ParentComponent->IsInAutoRecover();
        const bool bEdgeSupportNow = (GetGroundContacts().Num() == 2);
        if (HasPersistentGroundContact() && bIsSupportingContact)
        {
            if (!(bEdgeRecoverActive && bEdgeSupportNow))
            {
                ApplyPersistentSupportConstraint(Dt);
            }
        }
        return;
    }

    // Event resolution
    if (bIsSupportingContact)
    {
        ResolveDirectGroundSupport(Dt, CurrentHit);
    }
    else
    {
        // Wall / gutter / edge contact
        ResolveWallOrGutter(Dt, CurrentHit);

        // IMPORTANT: do NOT destroy the stored ground plane just because current event is a wall.
        // We want the support constraint to still run, as long as it remains valid.
        // (So: do NOT set bGroundPlaneValid=false here.)
    }

    // Refresh persistence after event solve (so the manifold is built from the post-impulse state)
    UpdatePersistentGroundContact(Dt);

    if (HasPersistentGroundContact() && bIsSupportingContact)
    {
        ApplyPersistentSupportConstraint(Dt);
    }

    // Bookkeeping for new contact logic in future frames
    PrevContactNormal = N;
    bHadContactPrevFrame = true;

    // Debug
    const FVector vImpactEnd = GetVelocityAtPoint(CurrentHit.ImpactPoint);
    const float vN_end = FVector::DotProduct(vImpactEnd, N);
    const FVector omega_end = ParentComponent->GetPhysAngularVelocity();
    // UE_LOG(BoxSubBodyLog, Log, TEXT("[CCDBox] End of Hit vs Ground for frame = %d, vN_end = %.2f cm/s, omega_end = %s rad/s, Rotation = %s"),
    //    ParentComponent->NumFrame(), vN_end, *omega_end.ToString(), *ParentComponent->GetPhysRotation().ToString());
}


void UBoxSubBody::ResolveHitVsSphere(USphereSubBody& Sphere, const float& delta, const float& TimePassed)
{
    // --- Box kinematics at TOI ---
    // Kinematics = GetKinematicsFromOwner(ParentComponent->NumFrame());
    const SKinematic& BoxKS = Kinematics; // already correct
    SSBox BBox = MakeBox(ParentComponent->NumFrame(), /*TimePassed*/ 0.0f); // useless to pass TimePassed here since every SubBodies are updated to current TimePassed before sweeping

    // --- Sphere kinematics at TOI ---
    const SKinematic& SphereKS0 =
        Sphere.GetKinematicState();
    // const SKinematic SphereKSAtTOI = SphereKS0.Integrate(TimePassed);

    // Current frame
    const unsigned frame = ParentComponent->NumFrame();

    // -------- Overlap branch (Box - Sphere) --------
    /*
    {
        const float epsTOI = 1e-6f;

        FVector Cp;
        const float sep = BBox.SphereOBBSeparation(BBox.Rot, BBox.WorldCenter, SphereKS0.Location, Sphere.GetRadius(), &Cp);
        const float pen = FMath::Max(0.f, -sep);

        if (pen > MinSlopCm)
        {
            FVector N = (SphereKS0.Location - Cp); // Box -> Sphere
            if (!N.Normalize())
                N = BBox.UpVector();

            const FVector P = SphereKS0.Location - N * Sphere.GetRadius();
            ISpeedComponent* OtherComp = Sphere.GetParentComponent(); // needs access; ok if you add getter

            SolveOverlap(
                *ParentComponent, GetMass(), ComputeWorldInvInertiaTensor(), BoxKS,
                OtherComp, Sphere.GetMass(), Sphere.ComputeWorldInvInertiaTensor(), SphereKS0,
                P, N,
                pen,
                0.05f, 0.8f,
                GetImpactThreshold()
            );
            if (IsFakePhysicsEnabled())
            {
                ApplyFakePhysicsOn(Sphere, CurrentHit, delta);
            }
            if (Sphere.IsFakePhysicsEnabled())
            {
                Sphere.ApplyFakePhysicsOn(*this, Sphere.GetHit(), delta);
            }
            return;
        }
    }*/

    FVector ImpThis, ImpOther;
    bool ok = Speed::SImpulseSolver::ComputeCollisionImpulse(
        CurrentHit.ImpactPoint,
		CurrentHit.ImpactNormal,   // Sphere -> Box

        BoxKS, GetMass(), ComputeWorldInvInertiaTensor(),
        SphereKS0, Sphere.GetMass(), Sphere.ComputeWorldInvInertiaTensor(),

        MixRestitution(Sphere.GetRestitution(), GetRestitution(), EMixMode::E_Max),
        MixFriction(Sphere.GetDynamicFriction(), GetDynamicFriction(), EMixMode::E_Max),
        ImpThis,
        ImpOther,
        GetImpactThreshold()
    );

    if (!ok)
        return;

    // --- Debug ---
    /*UE_LOG(BoxSubBodyLog, Log, TEXT("[%s(%d)][CCDBox] Resolving Sphere hit for frame = %d, with ImpactPoint = %s, ImpactNormal = %s, at TOI = %fs"), *GetOwner()->GetName(), GetOwnerRole(),
        ParentComponent->NumFrame(), *CurrentHit.ImpactPoint.ToString(), *CurrentHit.ImpactNormal.ToString(), TimePassed);
    UE_LOG(BoxSubBodyLog, Log, TEXT("[%s(%d)][CCDBox] Box KS at TOI: %s"), *GetOwner()->GetName(), GetOwnerRole(),
        *BoxKS.ToString());
    UE_LOG(BoxSubBodyLog, Log, TEXT("[%s(%d)][CCDBox] Sphere KS at TOI: %s"), *GetOwner()->GetName(), GetOwnerRole(),
        *SphereKS0.ToString());
    UE_LOG(BoxSubBodyLog, Log, TEXT("[%s(%d)][CCDBox] Box applies impulse %s on Sphere at TOI = %fs"), *GetOwner()->GetName(), GetOwnerRole(),
        *ImpOther.ToString(), TimePassed);*/

        // --- Apply impulses ---
    ApplyImpulse(ImpThis, CurrentHit.ImpactPoint);
    Sphere.ApplyImpulse(ImpOther, CurrentHit.ImpactPoint);

    if (IsFakePhysicsEnabled())
    {
		// (ParentComponent->NumFrame()); // update kinematics to current frame before applying fake physics
		// Sphere.UpdateKinematicsFromOwner(ParentComponent->NumFrame());
        ApplyFakePhysicsOn(Sphere, CurrentHit, delta);
    }
    if (Sphere.IsFakePhysicsEnabled())
    {
        // UpdateKinematicsFromOwner(ParentComponent->NumFrame()); // update kinematics to current frame before applying fake physics
        // Sphere.UpdateKinematicsFromOwner(ParentComponent->NumFrame());
        Sphere.ApplyFakePhysicsOn(*this, Sphere.GetHit(), delta);
    }

    // Handle micro-oscillations
    Sphere.HandleMicroOscillation();

    // --- Gameplay hooks ---
    ParentComponent->RcvImpactOnSubBody(*this, CurrentHit.ImpactPoint);
}

void UBoxSubBody::ResolveHitVsBox(
    UBoxSubBody& OtherBox,
    const float& delta,
    const float& TimePassed
)
{
    const unsigned frame = ParentComponent->NumFrame();
    const SKinematic& KA = Kinematics; // current

    // KB courant
    const SKinematic& KB = OtherBox.Kinematics; // si sync (sinon GetKinematicsFromOwner(frame))

    const FVector P = CurrentHit.ImpactPoint;
    const FVector N = CurrentHit.ImpactNormal.GetSafeNormal(); // Other -> This (entrant dans This)

    // -------- Overlap branch (Box - Box) --------
    // EstimateOBBOverlapAlongNormal + SolveOverlap
    {
        SSBox A = MakeBox(frame, 0.f);
        SSBox B = OtherBox.MakeBox(frame, 0.f);
        const float pen = SSBox::EstimateOBBOverlapAlongNormal(A, B, N);
        if (pen > MinSlopCm)
        {
            ISpeedComponent* OtherComp = OtherBox.GetParentComponent();
            SolveOverlap(*ParentComponent, GetMass(), ComputeWorldInvInertiaTensor(), KA,
                OtherComp, OtherBox.GetMass(), OtherBox.ComputeWorldInvInertiaTensor(), KB,
                P, N, pen, MinSlopCm, 0.8f, GetImpactThreshold());
            if (IsFakePhysicsEnabled())
            {
                ApplyFakePhysicsOn(OtherBox, CurrentHit, delta);
            }
            if (OtherBox.IsFakePhysicsEnabled())
            {
                OtherBox.ApplyFakePhysicsOn(*this, OtherBox.GetHit(), delta);
            }
            return;
        }
    }

    FVector ImpA, ImpB;
    const float e = MixRestitution(GetRestitution(), OtherBox.GetRestitution(), EMixMode::E_Max);
    const float mu = MixFriction(GetDynamicFriction(), OtherBox.GetDynamicFriction(), EMixMode::E_Max);

    if (!Speed::SImpulseSolver::ComputeCollisionImpulse(
        P, N,
        KA, GetMass(), ComputeWorldInvInertiaTensor(),
        KB, OtherBox.GetMass(), OtherBox.ComputeWorldInvInertiaTensor(),
        e, mu,
        ImpA, ImpB,
        GetImpactThreshold()))
    {
        return;
    }

    ApplyImpulse(ImpA, P);
    OtherBox.ApplyImpulse(ImpB, P);

    // Fake physics both sides
    if (IsFakePhysicsEnabled())
    {
        // UpdateKinematicsFromOwner(ParentComponent->NumFrame()); // update kinematics to current frame before applying fake physics
		// OtherBox.UpdateKinematicsFromOwner(ParentComponent->NumFrame());
        ApplyFakePhysicsOn(OtherBox, CurrentHit, delta);
    }
    if (OtherBox.IsFakePhysicsEnabled())
    {
        SHitResult InverseHit = CurrentHit;
        InverseHit.ImpactNormal *= -1.f; // invert normal for other box's perspective
		// UpdateKinematicsFromOwner(ParentComponent->NumFrame()); // update kinematics to current frame before applying fake physics
		// OtherBox.UpdateKinematicsFromOwner(ParentComponent->NumFrame());
        OtherBox.ApplyFakePhysicsOn(*this, InverseHit, delta);
    }
}

// ============================================================================
// ================= Helper Methods for ResolveHitVsGround ====================
// ============================================================================


// ============================================================================
//  ResolveWallOrGutter
//     - Single contact, no “support” logic, no persistence.
//     - Only prevents penetration along N and applies optional friction.
//     - Uses sequential impulse with minimal policy decisions.
// ============================================================================
void UBoxSubBody::ResolveWallOrGutter(const float& Dt, const SHitResult& Hit)
{
    if (!ParentComponent) return;

    const FVector N = Hit.ImpactNormal.GetSafeNormal();
    const FVector P = Hit.ImpactPoint;

    const FVector Vp = GetVelocityAtPoint(P);
    const float vN = FVector::DotProduct(Vp, N);

    // If not approaching, do nothing (prevents “sticky” contacts on edges).
    constexpr float VN_EPS = 1.0f; // cm/s
    if (vN >= VN_EPS)
        return;

    const FVector COM = GetCOM();
    const FVector r = P - COM;

    const float InvMass = 1.f / GetMass();
    const FMatrix InvI = ComputeWorldInvInertiaTensor();

    const float denomN = EffectiveMassAlongDir(InvMass, InvI, r, N);
    if (denomN <= KINDA_SMALL_NUMBER)
        return;

    // Debug
    // Kinematics = GetKinematicsFromOwner(ParentComponent->NumFrame());
    // UE_LOG(BoxSubBodyLog, Log, TEXT("[CCDBox] ResolveWallOrGutter for frame = %d ImpactPoint = %s, ImpactNormal = %s, Kinematic = %s"),
    //     ParentComponent->NumFrame(), *P.ToString(), *N.ToString(), *Kinematics.ToString());

    // Restitution policy for walls/gutters:
    // - Usually you want NO bounce in a gutter because it creates wedging artifacts.
    // - Keep it at 0 unless you have a very clear “impact” situation.
    // constexpr float ImpactThreshold = 80.f; // cm/s (higher than ground)
    const bool bImpact = (vN < -GetImpactThreshold());

    float Rest = 0.f;
    // If you really want micro-bounce on hard impacts, you can enable this:
    // if (bImpact) Rest = FMath::Clamp(GetRestitution(), 0.f, 0.15f);
    if (bImpact)
    {
        // ParentComponent->SetHadImpactThisFrame(true);
    }

    // Normal impulse to cancel vN (or bounce if enabled)
    float jn = -(1.f + Rest) * vN / denomN;

    // Guard against absurd impulses in 1-point concave contacts.
    // This is NOT a “wedging clamp”; it’s a numerical safety for edge cases.
    // Tune with care. If you prefer, delete it and rely on correct CCD.
    const float jnMax = 5.f * GetMass(); // (kg? but your Mass seems scalar) tweak
    jn = FMath::Clamp(jn, 0.f, jnMax);

    FVector J = jn * N;

    // Friction on walls/gutters:
    // - If your gutter is meant to be “smooth”, keep mu small.
    // - If you set mu too big, you’ll reintroduce wedging via tangential coupling.
    const float mu = 0.05f; // conservative; consider exposing as ParentComponent->HitboxFrictionWall

    const FVector vT = Vp - vN * N;
    const float vTmag = vT.Size();
    if (vTmag > KINDA_SMALL_NUMBER && mu > 0.f)
    {
        const FVector T = vT / vTmag;

        const float denomT = EffectiveMassAlongDir(InvMass, InvI, r, T);
        if (denomT > KINDA_SMALL_NUMBER)
        {
            // impulse to reduce tangential speed (simple Coulomb)
            float jt = -vTmag / denomT;

            // Coulomb clamp
            const float jtMax = mu * jn;
            jt = FMath::Clamp(jt, -jtMax, jtMax);

            J += jt * T;
        }
    }

    ApplyImpulse(J, P);

    // Optional positional correction when starting in penetration.
    // Use Hit.ImpactNormal (depenetration direction) when bStartPenetrating.
    if (Hit.bStartPenetrating && Hit.PenetrationDepth > 0.f)
    {
        const FVector DepenDir = Hit.ImpactNormal.GetSafeNormal();
        constexpr float Slop = 0.5f;     // cm
        constexpr float BetaPos = 0.35f; // stronger than ground to escape concave edges

        const float pushOut = FMath::Max(Hit.PenetrationDepth - Slop, 0.f);
        if (pushOut > 0.f)
        {
            ParentComponent->SetPhysLocation(ParentComponent->GetPhysLocation() + DepenDir * (BetaPos * pushOut));

            // Kill inward velocity along depen dir
            FVector V = ParentComponent->GetPhysVelocity();
            const float vIn = FVector::DotProduct(V, DepenDir);
            if (vIn < 0.f)
                ParentComponent->AddPhysVelocity(-vIn * DepenDir);
        }
    }
}

// ============================================================================
//  ResolveDirectGroundSupport
//     - Multi-point support contact resolution (sequential impulses)
//     - Restitution only for “sharp new impacts”
//     - Coulomb friction only for support contacts
//     - Stores plane for persistence when appropriate
// ============================================================================
void UBoxSubBody::ResolveDirectGroundSupport(const float& Dt, const SHitResult& Hit)
{
    if (!ParentComponent) return;

    const FVector Nraw = Hit.ImpactNormal.GetSafeNormal();
    const FVector N = Nraw; // Keep raw; if you want composite, do it in persistence step
    const FVector ImpactP = Hit.ImpactPoint;

    // If separating, don't solve.
    constexpr float VN_EPS = 1.0f; // cm/s
    const float vN_atImpact = FVector::DotProduct(GetVelocityAtPoint(ImpactP), N);
    if (vN_atImpact >= VN_EPS)
        return;

    // Build manifold from current pose (TOI pose).
    // Kinematics = GetKinematicsFromOwner(ParentComponent->NumFrame());
    const FVector CenterWS = Kinematics.Location;
    const FQuat BoxRotWS = Kinematics.Rotation;
    const FVector Ext = BoxExtent;

    TArray<FVector> ContactPtsWS;
    BuildSupportManifoldFromNormal(N, CenterWS, BoxRotWS, Ext, ContactPtsWS, /*eps*/0.5f);
    /*UE_LOG(BoxSubBodyLog, Log,
        TEXT("[EdgeLatchState] frame=%d latched=%d latchedLS=%d latchFrame=%d"),
        ParentComponent->NumFrame(),
        bEdgeSupportLatched ? 1 : 0,
        LatchedEdgeContactsLS.Num(),
        EdgeSupportLatchFrame);*/

        // ------------------------------------------------------------
        // PERSISTENT EDGE OVERRIDE
        // ------------------------------------------------------------
    const bool bUseLatchedEdge =
        bEdgeSupportLatched &&
        LatchedEdgeContactsLS.Num() == 2;

    if (bUseLatchedEdge && ContactPtsWS.Num() == 1)
    {
        const SKinematic K = GetKinematicsFromOwner(ParentComponent->NumFrame());

        const FVector A = K.Location + K.Rotation.RotateVector(LatchedEdgeContactsLS[0]);
        const FVector B = K.Location + K.Rotation.RotateVector(LatchedEdgeContactsLS[1]);

        ContactPtsWS.Reset();
        ContactPtsWS.Add(A);
        ContactPtsWS.Add(B);

        // IMPORTANT : renew latch so grace window doesn't expire
        if (vN_atImpact < -5.f)
        {
            bEdgeSupportLatched = true;
            EdgeSupportLatchFrame = ParentComponent->NumFrame();
        }

#if !UE_BUILD_SHIPPING
        // UE_LOG(BoxSubBodyLog, Log, TEXT("[PersistentEdgeOverride] Forcing edge support (renew latch)"));
#endif
    }

    CurrentGroundContactsWS = ContactPtsWS;

    // if we end up with 2 points, renew too (even without override)
    if (CurrentGroundContactsWS.Num() == 2 && bEdgeSupportLatched && LatchedEdgeContactsLS.Num() == 2)
    {
        EdgeSupportLatchFrame = ParentComponent->NumFrame();
    }


    // Store plane for persistence (only if the hit came from sweep and is “support-like”).
    if (bGroundHitFromSweep)
    {
        bGroundPlaneValid = true;
        GroundPlaneN = N;
        GroundPlanePointWS = Hit.ImpactPoint;
        GroundPlaneD = FVector::DotProduct(ImpactP, N);
        GroundComp = Hit.Component;
		ParentComponent->RcvImpactOnSubBody(*this, ImpactP);
    }

    // If concave, do not persist (your existing rule).
    if (IsConcaveGroundContact())
    {
        bGroundPlaneValid = false;
        bHasGroundContact = false;
        CurrentGroundContactsWS.Reset();
    }

    // Sequential impulse (warm-started)
    const int32 NumC = CurrentGroundContactsWS.Num();
    if (NumC <= 0) return;

    if (PrevLambdaN.Num() != NumC)
        PrevLambdaN.Reset();

    const int32 Iter = FMath::Clamp(NumC * 2, 6, 12);

    TArray<float> LambdaN;
    LambdaN.SetNum(NumC);
    for (int32 i = 0; i < NumC; ++i)
        LambdaN[i] = (i < PrevLambdaN.Num()) ? PrevLambdaN[i] : 0.f;

    const float InvMass = 1.f / GetMass();
    const FMatrix InvI = ComputeWorldInvInertiaTensor();
    // UE_LOG(BoxSubBodyLog, Log, TEXT("[CCDBox] ResolveDirectGroundSupport for frame = %d CurrentGroundContactsWS.Num = %d, ImpactNormal = %s, Kinematic = %s"),
    //     ParentComponent->NumFrame(), CurrentGroundContactsWS.Num(), *N.ToString(), *Kinematics.ToString());

    // Restitution policy (ground):
    // constexpr float ImpactThreshold = 30.f; // cm/s
    for (int32 it = 0; it < Iter; ++it)
    {
        if (NumC == 2)
        {
            SolveEdgeSupportConstraint(N, CurrentGroundContactsWS, Dt, /*bDoFriction=*/false, /*Mu=*/0.f);
            break;
        }
        for (int32 i = 0; i < NumC; ++i)
        {
            const FVector& P = CurrentGroundContactsWS[i];
            const FVector Vp = GetVelocityAtPoint(P);
            const float vN = FVector::DotProduct(Vp, N);
            if (vN >= 0.f)
                continue;

            const FVector COM = GetCOM();
            const FVector r = P - COM;

            const float denomN = EffectiveMassAlongDir(InvMass, InvI, r, N);
            if (denomN <= KINDA_SMALL_NUMBER)
                continue;

            // “Sharp impact” detection (optional, based on your old logic)
            float Rest = 0.f;
            const bool bImpact = (vN < -GetImpactThreshold());

            const float cosAngle = bHadContactPrevFrame ? FVector::DotProduct(N, PrevContactNormal) : 0.f;
            const bool bNewContact = (!bHadGroundContactPrevFrame && bGroundHitFromSweep);
            const bool bSharpImpact = (bNewContact && bImpact && cosAngle > 0.95f);

            if (bSharpImpact)
            {
                Rest = GetRestitution();
                // ParentComponent->SetHadImpactThisFrame(true);
            }

            // normal impulse increment
            const float targetVN = bImpact ? -(Rest * vN) : 0.f; // simple: drive vN to 0, with bounce if enabled
            const float dvN = (targetVN - vN);

            float dLambda = dvN / denomN;

            const float lambdaPrev = LambdaN[i];
            const float lambdaNew = FMath::Max(lambdaPrev + dLambda, 0.f);
            const float applied = lambdaNew - lambdaPrev;
            LambdaN[i] = lambdaNew;

            FVector J = applied * N;

            // friction (Coulomb) on supporting contact
            const float mu = GetDynamicFriction();
            if (mu > 0.f)
            {
                const FVector vT = Vp - vN * N;
                const float vTmag = vT.Size();
                if (vTmag > KINDA_SMALL_NUMBER)
                {
                    const FVector T = vT / vTmag;

                    const float denomT = EffectiveMassAlongDir(InvMass, InvI, r, T);
                    if (denomT > KINDA_SMALL_NUMBER)
                    {
                        float jt = -vTmag / denomT;
                        const float jtMax = mu * applied;
                        jt = FMath::Clamp(jt, -jtMax, jtMax);
                        J += jt * T;
                    }
                }
            }

            ApplyImpulse(J, P);
            // UE_LOG(BoxSubBodyLog, Log, TEXT("[CCDBox] ResolveDirectGroundSupport for frame = %d, On point %s, applied J= %s, Kinematic = %s"),
            //    ParentComponent->NumFrame(), *P.ToString(), *J.ToString(), *Kinematics.ToString());
        }
    }

    PrevLambdaN = LambdaN;

    // Positional correction (Baumgarte / split impulse style)
    if (Hit.bStartPenetrating && Hit.PenetrationDepth > 0.f)
    {
        const FVector DepenDir = Hit.ImpactNormal.GetSafeNormal();
        constexpr float Slop = 0.5f;     // cm
        constexpr float BetaPos = 0.2f;

        const float pushOut = FMath::Max(Hit.PenetrationDepth - Slop, 0.f);
        if (pushOut > 0.f)
        {
            ParentComponent->SetPhysLocation(ParentComponent->GetPhysLocation() + DepenDir * (BetaPos * pushOut));

            FVector V = ParentComponent->GetPhysVelocity();
            const float vIn = FVector::DotProduct(V, DepenDir);
            if (vIn < 0.f)
                ParentComponent->AddPhysVelocity(-vIn * DepenDir);
        }
    }

    // Track previous contact normal
    PrevContactNormal = N;
    bHadContactPrevFrame = true;
}

// ============================================================================
//  ApplyPersistentSupportConstraint
//     - This is the “keep rear edge down” constraint.
//     - Runs AFTER event resolution (wall/gutter and/or direct ground hit).
//     - Uses the stored ground plane and recomputes a manifold on that plane.
//     - Applies a small corrective impulse to kill normal velocity into the plane,
//       and a small positional correction to stay within tolerance.
// ============================================================================
void UBoxSubBody::ApplyPersistentSupportConstraint(const float& Dt)
{
    if (!ParentComponent) return;

	// if hitbox is in auto-recover and has 2 ground contacts, skip persistence to avoid fighting with edge recovery logic
    if (ParentComponent->IsInAutoRecover() &&
        GetGroundContacts().Num() == 2)
    {
        return;
    }

    // UpdatePersistentGroundContact must have set:
    //  - bHasGroundContact
    //  - GroundHit with ImpactNormal + ImpactPoint on plane
    //  - CurrentGroundContactsWS rebuilt (manifold)
    if (!HasPersistentGroundContact())
        return;

    // IMPORTANT: use the stored plane normal (support direction), not the latest wall normal
    // Use composite normal when inverted / in gutter
    FVector SupportN = GroundPlaneN.GetSafeNormal();

    const FVector CompositeN = CompositeGroundNormal.GetSafeNormal();

    // If the composite normal is significantly different, use it
    if (FVector::DotProduct(CompositeN, SupportN) < 0.85f)
    {
        SupportN = CompositeN;
    }
    const FVector N = SupportN;


    // If the plane became non-support (e.g. got overwritten), bail.
    if (FVector::DotProduct(N, FVector::UpVector) < 0.5f)
        return;

    // Concave rule: never persist there
    if (IsConcaveGroundContact())
        return;

    // Rebuild manifold from current pose *against the ground plane normal*.
    // This is key: it keeps 2+ points, so you don't pivot on the gutter edge.
    const SKinematic K = GetKinematicsFromOwner(ParentComponent->NumFrame());
    TArray<FVector> SupportPts;

    if (bEdgeSupportLatched && LatchedEdgeContactsLS.Num() == 2)
    {
        // Force edge support from latched memory
        SupportPts.Add(
            K.Location + K.Rotation.RotateVector(LatchedEdgeContactsLS[0])
        );
        SupportPts.Add(
            K.Location + K.Rotation.RotateVector(LatchedEdgeContactsLS[1])
        );
    }
    else
    {
        BuildSupportManifoldFromNormal(
            N, K.Location, K.Rotation, BoxExtent, SupportPts, 0.75f
        );
    }

    if (SupportPts.Num() < 2)
    {
        return;
    }

    // Measure “worst” normal velocity into the plane among support points.
    float worstInto = 0.f;
    for (const FVector& P : SupportPts)
    {
        const float vN = FVector::DotProduct(GetVelocityAtPoint(P), N);
        worstInto = FMath::Min(worstInto, vN); // most negative
    }

    // If not moving into plane, we don't need impulses (prevents sticky behavior).
    constexpr float VN_EPS = 0.0f; // cm/s (allow tiny bias)
    if (worstInto > VN_EPS)
    {
        bHadGroundContactPrevFrame = true;
        PrevGroundNormal = N;
        return;
    }

    // Apply a *distributed* correction impulse across manifold points.
    // Physics-style: sequential impulses over points, targeting vN -> 0.
    const float InvMass = 1.f / GetMass();
    const FMatrix InvI = ComputeWorldInvInertiaTensor();

    // Debug
    Kinematics = GetKinematicsFromOwner(ParentComponent->NumFrame());
    // UE_LOG(BoxSubBodyLog, Log, TEXT("[CCDBox] ResolveDirectGroundSupport for frame = %d, ImpactNormal = %s, Kinematic = %s"),
    //    ParentComponent->NumFrame(), *N.ToString(), *Kinematics.ToString());

    constexpr int32 Iter = 6;
    for (int32 it = 0; it < Iter; ++it)
    {
        if (SupportPts.Num() == 2)
        {
            SolveEdgeSupportConstraint(N, SupportPts, Dt, /*bDoFriction=*/false, /*Mu=*/0.f);
            break;
        }
        for (const FVector& P : SupportPts)
        {
            const FVector Vp = GetVelocityAtPoint(P);
            const float vN = FVector::DotProduct(Vp, N);

            // target is ZERO while support is active
            const float dvN = -vN;

            if (FMath::Abs(dvN) < 0.01f)
                continue;

            const FVector COM = GetCOM();
            const FVector r = P - COM;

            const float denomN = EffectiveMassAlongDir(InvMass, InvI, r, N);
            if (denomN <= KINDA_SMALL_NUMBER)
                continue;

            // Cancel normal velocity (no bounce for persistence)
            const float jn = (dvN) / denomN;
            ApplyImpulse(jn * N, P);
        }
    }

    // Positional drift correction toward plane (soft).
    // Keep within a small band so you don't accumulate penetration over many frames.
    TArray<FVector> Verts;
    GetBoxVertices(K.Location, K.Rotation, BoxExtent, Verts);

    float minDist = FLT_MAX;
    const FVector PlaneN = GroundPlaneN.GetSafeNormal();
    for (const FVector& V : Verts)
    {
        const float dist = PlaneSignedDist(V, PlaneN, GroundPlanePointWS);
        minDist = FMath::Min(minDist, dist);
    }

    constexpr float ContactTol = 0.05f; // cm
    constexpr float PenTol = 0.5f;  // cm
    constexpr float BetaPos = 0.1f;  // very gentle

    // If we drift too far into the plane, push out gently.
    if (minDist < -PenTol)
    {
        const float pushOut = (-PenTol - minDist);
        ParentComponent->SetPhysLocation(ParentComponent->GetPhysLocation() + N * (BetaPos * pushOut));

        // Kill inward normal component at COM
        FVector Vcm = ParentComponent->GetPhysVelocity();
        const float vIn = FVector::DotProduct(Vcm, N);
        if (vIn < 0.f)
            ParentComponent->AddPhysVelocity(-vIn * N);
    }
    else if (minDist > ContactTol)
    {
        // If we’re clearly leaving the plane, stop persisting.
        // (Otherwise you “float” and keep canceling gravity.)
        bGroundPlaneValid = false;
        bHasGroundContact = false;
    }

    // Hard release conditions for latched edge
    if (bEdgeSupportLatched)
    {
        const float upDot = FVector::DotProduct(N, FVector::UpVector);
        if (upDot < 0.5f || minDist > 2.0f)
        {
            bEdgeSupportLatched = false;
            LatchedEdgeContactsLS.Reset();
        }
    }

    bHadGroundContactPrevFrame = true;
    PrevGroundNormal = N;
}

void UBoxSubBody::UpdatePersistentGroundContact(const float& Dt)
{
    bHasGroundContact = false;
	const bool bEdgeRecoverActive = ParentComponent->IsInAutoRecover();

    if (!bGroundPlaneValid || !GroundComp.IsValid())
        return;

    if (FVector::DotProduct(GroundPlaneN, FVector::UpVector) < 0.5f)
        return;

    if (IsConcaveGroundContact())
    {
        bGroundPlaneValid = false;
        bHasGroundContact = false;
        bGroundContactStable = false;
        StableTime = 0.f;
        CurrentGroundContactsWS.Reset();
        PrevGroundContactsLS.Reset();
        return;
    }

    const FVector N = CompositeGroundNormal.GetSafeNormal();
    const SKinematic K = GetKinematicsFromOwner(ParentComponent->NumFrame());

    TArray<FVector> Verts;
    GetBoxVertices(K.Location, K.Rotation, BoxExtent, Verts);

    // ------------------------------------------------------------
    // Compute min distance to plane
    // ------------------------------------------------------------
    float minDist = FLT_MAX;
    for (const FVector& V : Verts)
    {
        const float dist = PlaneSignedDist(V, GroundPlaneN, GroundPlanePointWS);
        minDist = FMath::Min(minDist, dist);
    }

    float worstVN = 0.f;
    for (const FVector& P : CurrentGroundContactsWS)
    {
        const float vnP = FVector::DotProduct(GetVelocityAtPoint(P), N);
        worstVN = FMath::Max(worstVN, vnP);
    }

    constexpr float ContactTol = 0.10f;
    constexpr float PenTolSupport = 1.0f;
    constexpr float PenTolReject = 2.5f;
    constexpr float SepVelTol = 3.0f;

    const bool bWithinSupportBand =
        (minDist <= ContactTol) &&
        (minDist >= -PenTolSupport);

    const bool bNotExploding =
        (minDist >= -PenTolReject);

    const bool bStableNormalMotion =
        (worstVN <= SepVelTol);

    if (!(bWithinSupportBand && bStableNormalMotion && bNotExploding))
    {
        bEdgeSupportLatched = false;
        LatchedEdgeContactsLS.Reset();
        PrevGroundContactsLS.Reset();
        return;
    }
    // If clearly separating, drop persistence
    if (worstVN > 0.5f)
    {
        bHasGroundContact = false;
        bGroundPlaneValid = false;
        CurrentGroundContactsWS.Reset();
        PrevGroundContactsLS.Reset();
        return;
    }

    // ------------------------------------------------------------
    // Rebuild support manifold from vertices
    // ------------------------------------------------------------
    bHasGroundContact = true;

    CurrentGroundContactsWS.Reset();
    constexpr float ContactEps = 0.5f;

    for (const FVector& V : Verts)
    {
        const float dist = PlaneSignedDist(V, GroundPlaneN, GroundPlanePointWS);
        if (dist <= minDist + ContactEps)
            CurrentGroundContactsWS.Add(V);
    }

    // ------------------------------------------------------------
    // EDGE PERSISTENCE HYSTERESIS (local-space robust)
    // ------------------------------------------------------------
    if (!bEdgeRecoverActive && PrevGroundContactsLS.Num() == 2 && CurrentGroundContactsWS.Num() == 1)
    {
        const FVector& P = CurrentGroundContactsWS[0];

        // Rebuild previous edge in CURRENT pose using LOCAL stored endpoints
        const FVector A = K.Location + K.Rotation.RotateVector(PrevGroundContactsLS[0]);
        const FVector B = K.Location + K.Rotation.RotateVector(PrevGroundContactsLS[1]);

        const FVector Edge = (B - A);
        const float EdgeLen2 = Edge.SizeSquared();

        if (EdgeLen2 > KINDA_SMALL_NUMBER)
        {
            const FVector EdgeDir = Edge / FMath::Sqrt(EdgeLen2);

            // Project P on the edge line
            const float t = FVector::DotProduct(P - A, EdgeDir);
            const FVector Pproj = A + t * EdgeDir;

            // Snap tolerance should beat per-frame drift + tiny noise
            // (minimum 2cm, but also allow translation magnitude per frame)
            const float vMag = K.Velocity.Size();
            const float dynTol = FMath::Max(2.0f, 1.25f * vMag * Dt); // cm
            const float tol2 = dynTol * dynTol;

            if (FVector::DistSquared(P, Pproj) <= tol2)
            {
                CurrentGroundContactsWS.Add(Pproj);

#if !UE_BUILD_SHIPPING
                /*UE_LOG(BoxSubBodyLog, Log,
                    TEXT("[EdgePersistence] Promoted vertex -> edge | dynTol=%.2f P=%s Pproj=%s"),
                    dynTol, *P.ToString(), *Pproj.ToString());*/
#endif
            }
#if !UE_BUILD_SHIPPING
            else
            {
                /*UE_LOG(BoxSubBodyLog, Log,
                    TEXT("[EdgePersistence] Rejected | dynTol=%.2f dist=%.3f P=%s Pproj=%s"),
                    dynTol, FMath::Sqrt(FVector::DistSquared(P, Pproj)),
                    *P.ToString(), *Pproj.ToString());*/
            }
#endif
        }
    }

    // Latch edge contacts if we have exactly 2 contacts this frame
    if (!bEdgeRecoverActive && CurrentGroundContactsWS.Num() == 2)
    {
        const FVector& P0 = CurrentGroundContactsWS[0];
        const FVector& P1 = CurrentGroundContactsWS[1];

        const float EdgeLenWS = (P1 - P0).Size();

        constexpr float MinLatchEdgeLen = 1.0f; // cm (tunable)

        if (EdgeLenWS > MinLatchEdgeLen)
        {
            LatchedEdgeContactsLS.Reset(2);
            LatchedEdgeContactsLS.Add(
                K.Rotation.UnrotateVector(P0 - K.Location)
            );
            LatchedEdgeContactsLS.Add(
                K.Rotation.UnrotateVector(P1 - K.Location)
            );

            bEdgeSupportLatched = true;
            EdgeSupportLatchFrame = ParentComponent->NumFrame();
        }
#if !UE_BUILD_SHIPPING
        else
        {
            /*UE_LOG(
                BoxSubBodyLog,
                Log,
                TEXT("[EdgeLatch] Rejected degenerate edge (len=%.4f cm)"),
                EdgeLenWS
            );*/
        }
#endif
    }

    // ------------------------------------------------------------
    // Store previous manifold for next frame
    // ------------------------------------------------------------
    PrevGroundContactsLS.Reset();
    PrevGroundContactsLS.Reserve(CurrentGroundContactsWS.Num());

    for (const FVector& Pw : CurrentGroundContactsWS)
    {
        // local around COM frame: (Pw - K.Location) in hitbox local
        PrevGroundContactsLS.Add(K.Rotation.UnrotateVector(Pw - K.Location));
    }
    if (PrevGroundContactsLS.Num() == 2)
    {
        bEdgeSupportLatched = true;
        EdgeSupportLatchFrame = ParentComponent->NumFrame();
    }

    // ------------------------------------------------------------
    // Rebuild GroundHit coherently
    // ------------------------------------------------------------
    const FVector& Location = K.Location;
    const FVector Pbox =
        CurrentGroundContactsWS.Num() ?
        CurrentGroundContactsWS[0] :
        Location;

    const float dist = PlaneSignedDist(Pbox, GroundPlaneN, GroundPlanePointWS);
    const FVector Pplane = Pbox - dist * GroundPlaneN;

    GroundHit = SHitResult();
    GroundHit.bBlockingHit = true;
    GroundHit.Component = GroundComp;
    GroundHit.ImpactNormal = GroundPlaneN;
    GroundHit.TOI = 0.f;
    // GroundHit.Location = K.Location;
    GroundHit.ImpactPoint = Pplane;
}

// ============================================================================
// =========================== Helper Methods =================================
// ============================================================================

SKinematic UBoxSubBody::GetKinematicsFromOwner(const unsigned int& NumFrame) const
{
    if (!ParentComponent)
        return SKinematic();
    const SKinematic& CarKinematicState = ParentComponent->GetKinematicStateForFrame(NumFrame);
    return GetKinematicsFromOwnerKS(CarKinematicState);
}

SKinematic UBoxSubBody::GetKinematicsFromOwnerKS(const SKinematic& CarKinematicState) const
{
    auto HitboxRelativeLocation = GetLocalOffset();

    FQuat CarRot(CarKinematicState.Rotation);
    auto HitboxWorldLocation = CarRot.RotateVector(HitboxRelativeLocation);
    auto HitboxWolrdCenter = CarKinematicState.Location + HitboxWorldLocation;

    FQuat HitboxRelativeQuat(GetLocalRotation());
    FVector HitboxExtent = BoxExtent;
    FQuat HitboxRot(CarRot * HitboxRelativeQuat);

    auto CarVelocity = CarKinematicState.Velocity;
    auto CarAcceleration = CarKinematicState.Acceleration;
    auto CarAngularVelocity = CarKinematicState.AngularVelocity;
    auto CarAngularAcceleration = CarKinematicState.AngularAcceleration;

    auto HitboxVelocity = CarVelocity + FVector::CrossProduct(CarAngularVelocity, HitboxWorldLocation);
    auto HitboxAcceleration = CarAcceleration + FVector::CrossProduct(CarAngularAcceleration, HitboxWorldLocation) +
        FVector::CrossProduct(CarAngularVelocity, FVector::CrossProduct(CarAngularVelocity, HitboxWorldLocation));

    SKinematic CrtKinematics;
    CrtKinematics.Location = HitboxWolrdCenter;
    CrtKinematics.Velocity = HitboxVelocity;
    CrtKinematics.Acceleration = HitboxAcceleration;
    CrtKinematics.Rotation = HitboxRot;
    CrtKinematics.AngularVelocity = CarAngularVelocity;
    CrtKinematics.AngularAcceleration = CarAngularAcceleration;
    return CrtKinematics;
}

SSBox UBoxSubBody::MakeBox(const unsigned int& NumFrame, const float& TimePassed) const
{
    auto CrtKinematics = GetKinematicsFromOwner(NumFrame);
    CrtKinematics = CrtKinematics.Integrate(TimePassed);
    return SSBox(
        CrtKinematics.Location,
        BoxExtent,
        CrtKinematics.Rotation,
        CrtKinematics.Velocity,
        CrtKinematics.Acceleration,
        CrtKinematics.AngularVelocity,
        CrtKinematics.AngularAcceleration
    );
}

SSBox UBoxSubBody::MakeBoxFromKS(const SKinematic& BoxKinematicState) const
{
    auto CrtKinematics = GetKinematicsFromOwnerKS(BoxKinematicState);
    return SSBox(
        CrtKinematics.Location,
        BoxExtent,
        CrtKinematics.Rotation,
        CrtKinematics.Velocity,
        CrtKinematics.Acceleration,
        CrtKinematics.AngularVelocity,
        CrtKinematics.AngularAcceleration
    );
}

void UBoxSubBody::GetBoxVertices(const FVector& Center, const FQuat& Rot, const FVector& Ext, TArray<FVector>& OutVerts)
{
    OutVerts.Reset();
    for (int sx : {-1, 1})
    {
        for (int sy : {-1, 1})
        {
            for (int sz : {-1, 1})
            {
                FVector local = FVector(sx * Ext.X, sy * Ext.Y, sz * Ext.Z);
                OutVerts.Add(Center + Rot.RotateVector(local));
            }
        }
    }
}

FVector UBoxSubBody::ComputeBoxSupportPointWS(const FVector& Center, const FQuat& Rot, const FVector& Ext, const FVector& N)
{
    const FVector N_LS = Rot.UnrotateVector(N);

    const FVector SupportLS(
        (N_LS.X >= 0.f) ? -Ext.X : Ext.X,
        (N_LS.Y >= 0.f) ? -Ext.Y : Ext.Y,
        (N_LS.Z >= 0.f) ? -Ext.Z : Ext.Z
    );

    return Center + Rot.RotateVector(SupportLS);
}

const TArray<FVector>& UBoxSubBody::GetGroundContacts() const
{
    return CurrentGroundContactsWS;
}

bool UBoxSubBody::IsConcaveGroundContact() const
{
    if (CurrentGroundNormalsWS.Num() >= 2)
    {
        for (int i = 0; i < CurrentGroundNormalsWS.Num(); ++i)
        {
            for (int j = i + 1; j < CurrentGroundNormalsWS.Num(); ++j)
            {
                if (FVector::DotProduct(
                    CurrentGroundNormalsWS[i],
                    CurrentGroundNormalsWS[j]) < 0.95f)
                {
                    return true;
                }
            }
        }
    }
    return false;
}

FVector UBoxSubBody::GetGroundPlaneNormal() const
{
    return GroundPlaneN;
}

float UBoxSubBody::GetGroundPlaneD() const
{
    return GroundPlaneD;
}

bool UBoxSubBody::GetLatchedEdgeWS(FVector& OutA, FVector& OutB) const
{
    if (!bEdgeSupportLatched || LatchedEdgeContactsLS.Num() != 2)
        return false;

    const SKinematic K = GetKinematicsFromOwner(ParentComponent->NumFrame());

    OutA = K.Location + K.Rotation.RotateVector(LatchedEdgeContactsLS[0]);
    OutB = K.Location + K.Rotation.RotateVector(LatchedEdgeContactsLS[1]);
    return true;
}

bool UBoxSubBody::IsPointSupportedByPersistentContact(const FVector& P, float Margin) const
{
    if (!HasPersistentGroundContact())
        return false;

    // Distance to ground plane
    const float dist = PlaneSignedDist(P, GroundPlaneN, GroundPlanePointWS);

    // If point is above plane -> still supported
    if (dist > Margin)
        return true;

    return false;
}

bool UBoxSubBody::HasPersistentGroundContact() const
{
    return bHasGroundContact;
}

bool UBoxSubBody::HasPersistentEdgeSupport() const
{
    return bHasGroundContact && PrevGroundContactsLS.Num() == 2;
}


bool UBoxSubBody::IsInEdgeBalance() const
{
    if (!HasPersistentGroundContact())
        return false;

    const FVector GroundN = GetGroundPlaneNormal().GetSafeNormal();

    // Normal of the hitbox face most aligned with ground normal
    const FQuat Q = GetKinematicsFromOwner(ParentComponent->NumFrame()).Rotation;

    float bestDot = -1.f;
    int bestFace = -1;

    for (int i = 0; i < 6; ++i)
    {
        const FVector FaceN = GetFaceNormalWS(i, Q);
        const float d = FVector::DotProduct(FaceN, GroundN);
        if (d > bestDot)
        {
            bestDot = d;
            bestFace = i;
        }
    }

    // If no face is really aligned -> edge or corner
    constexpr float FaceAlignThreshold = 0.92f;
    return bestDot < FaceAlignThreshold;;
}

int UBoxSubBody::GetBestSupportingFace() const
{
    const FVector GroundN = GetGroundPlaneNormal().GetSafeNormal();
    const FQuat Q = GetKinematicsFromOwner(ParentComponent->NumFrame()).Rotation;

    float bestDot = -FLT_MAX;
    int bestFace = -1;

    for (int i = 0; i < 6; ++i)
    {
        const FVector FaceN = GetFaceNormalWS(i, Q);
        const float d = FVector::DotProduct(FaceN, GroundN);
        if (d > bestDot)
        {
            bestDot = d;
            bestFace = i;
        }
    }
    return bestFace;
}

int UBoxSubBody::GetLowestFaceIndex() const
{
    const FVector GroundN = GetGroundPlaneNormal().GetSafeNormal();
    const float GroundD = GetGroundPlaneD();

    const FQuat Q = GetKinematicsFromOwner(ParentComponent->NumFrame()).Rotation;
    const FVector C = GetKinematicsFromOwner(ParentComponent->NumFrame()).Location;

    float bestMinDist = FLT_MAX;
    int bestFace = -1;

    for (int face = 0; face < 6; ++face)
    {
        TArray<FVector> FaceVerts = GetFaceVerticesWS(face);

        float faceMin = FLT_MAX;
        for (const FVector& V : FaceVerts)
        {
            const float dist = FVector::DotProduct(V, GroundN) - GroundD;
            faceMin = FMath::Min(faceMin, dist);
        }

        if (faceMin < bestMinDist)
        {
            bestMinDist = faceMin;
            bestFace = face;
        }
    }

    return bestFace;
}

FVector UBoxSubBody::GetFaceNormalWS(const int& FaceIndex, const FQuat& HitboxRot) const
{
    const FVector Nlocal = FaceLocalNormal(FaceIndex);
    if (Nlocal.IsNearlyZero())
        return FVector::ZeroVector;

    return HitboxRot.RotateVector(Nlocal).GetSafeNormal();
}

TArray<FVector> UBoxSubBody::GetFaceVerticesWS(const int& FaceIndex) const
{
    TArray<FVector> OutVertsWS;
    if (!ParentComponent)
        return OutVertsWS;

    const SKinematic K = GetKinematicsFromOwner(ParentComponent->NumFrame());
    const FVector CenterWS = K.Location;
    const FQuat Q = K.Rotation;

    FVector Vlocal[4];
    FaceLocalVertices(FaceIndex, BoxExtent, Vlocal);

    OutVertsWS.Reserve(4);
    for (int i = 0; i < 4; ++i)
    {
        const FVector Vws = CenterWS + Q.RotateVector(Vlocal[i]);
        OutVertsWS.Add(Vws);
    }
    return OutVertsWS;
}


bool UBoxSubBody::ComputeEdgeLockAxis(const FVector& N, const TArray<FVector>& SupportPts,
    FVector& OutLockAxis) const
{
    if (SupportPts.Num() < 2)
        return false;

    // stable edge direction: pair of points which are the farest
    float bestD2 = -1.f;
    FVector EdgeDir = FVector::ZeroVector;

    for (int i = 0; i < SupportPts.Num(); ++i)
        for (int j = i + 1; j < SupportPts.Num(); ++j)
        {
            const FVector d = SupportPts[j] - SupportPts[i];
            const float d2 = d.SizeSquared();
            if (d2 > bestD2)
            {
                bestD2 = d2;
                EdgeDir = d;
            }
        }

    EdgeDir = EdgeDir.GetSafeNormal();
    if (EdgeDir.IsNearlyZero())
        return false;

    OutLockAxis = FVector::CrossProduct(N, EdgeDir).GetSafeNormal();
    return !OutLockAxis.IsNearlyZero();
}

void UBoxSubBody::ApplyImpulse(
    const FVector& LinearImpulse,
    const FVector& WorldPoint)
{
    ParentComponent->ApplyImpulse(LinearImpulse, WorldPoint, this);
}

FVector UBoxSubBody::GetCOM() const
{
    if (!ParentComponent)
    {
        return FVector::ZeroVector;
    }
    return ParentComponent->GetPhysLocation() + GetLocalOffset();
}

FVector UBoxSubBody::GetBoxExtent() const
{
	return BoxExtent;
}

FCollisionShape UBoxSubBody::GetCollisionShape(float Inflation) const
{
    return FCollisionShape::MakeBox(BoxExtent + FVector(Inflation));
}

void UBoxSubBody::BuildContactPoints(const FVector& Nworld, const FVector& CenterWS, const FQuat& BoxRotWS, const FVector& Ext,
    TArray<FVector>& OutPointsWS) const
{
    OutPointsWS.Reset();

    const FVector nLocal = BoxRotWS.UnrotateVector(Nworld).GetSafeNormal();
    const FVector absN = nLocal.GetAbs();

    int dominantAxes = 0;
    if (absN.X > 0.01f) dominantAxes++;
    if (absN.Y > 0.01f) dominantAxes++;
    if (absN.Z > 0.01f) dominantAxes++;

    enum class EFeature { Face, Edge, Corner };
    EFeature feature = (dominantAxes == 1) ? EFeature::Face : (dominantAxes == 2) ? EFeature::Edge : EFeature::Corner;

    TArray<FVector> localPts;

    if (feature == EFeature::Face)
    {
        // pick dominant axis
        if (absN.X >= absN.Y && absN.X >= absN.Z)
        {
            float sx = -FMath::Sign(nLocal.X) * Ext.X;
            localPts = { FVector(sx, +Ext.Y, +Ext.Z), FVector(sx, -Ext.Y, +Ext.Z),
                         FVector(sx, +Ext.Y, -Ext.Z), FVector(sx, -Ext.Y, -Ext.Z) };
        }
        else if (absN.Y >= absN.X && absN.Y >= absN.Z)
        {
            float sy = -FMath::Sign(nLocal.Y) * Ext.Y;
            localPts = { FVector(+Ext.X, sy, +Ext.Z), FVector(-Ext.X, sy, +Ext.Z),
                         FVector(+Ext.X, sy, -Ext.Z), FVector(-Ext.X, sy, -Ext.Z) };
        }
        else
        {
            float sz = -FMath::Sign(nLocal.Z) * Ext.Z;
            localPts = { FVector(+Ext.X, +Ext.Y, sz), FVector(-Ext.X, +Ext.Y, sz),
                         FVector(+Ext.X, -Ext.Y, sz), FVector(-Ext.X, -Ext.Y, sz) };
        }
    }
    else if (feature == EFeature::Edge)
    {
        float sx = (absN.X > 0.01f) ? -FMath::Sign(nLocal.X) * Ext.X : 0.f;
        float sy = (absN.Y > 0.01f) ? -FMath::Sign(nLocal.Y) * Ext.Y : 0.f;
        float sz = (absN.Z > 0.01f) ? -FMath::Sign(nLocal.Z) * Ext.Z : 0.f;

        bool useX = absN.X > 0.01f;
        bool useY = absN.Y > 0.01f;

        if (!useX)      localPts = { FVector(+Ext.X, sy, sz), FVector(-Ext.X, sy, sz) };
        else if (!useY) localPts = { FVector(sx, +Ext.Y, sz), FVector(sx, -Ext.Y, sz) };
        else            localPts = { FVector(sx, sy, +Ext.Z), FVector(sx, sy, -Ext.Z) };
    }
    else // Corner
    {
        float sx = -FMath::Sign(nLocal.X) * Ext.X;
        float sy = -FMath::Sign(nLocal.Y) * Ext.Y;
        float sz = -FMath::Sign(nLocal.Z) * Ext.Z;
        localPts = { FVector(sx, sy, sz) };
    }

    for (const FVector& pL : localPts)
    {
        OutPointsWS.Add(CenterWS + BoxRotWS.RotateVector(pL));
    }
}

FVector UBoxSubBody::BuildCompositeGroundNormal() const
{
    if (CurrentGroundNormalsWS.Num() == 0)
        return FVector::UpVector;

    FVector Nsum = FVector::ZeroVector;

    for (const FVector& N : CurrentGroundNormalsWS)
    {
        Nsum += N;
    }

    if (!Nsum.Normalize())
    {
        return CurrentGroundNormalsWS[0];
    }

    return Nsum;
}


bool UBoxSubBody::SolveEdgeSupportConstraint(
    const FVector& SupportN,
    const TArray<FVector>& SupportPts,
    const float Dt,
    const bool bDoFriction /*= false*/,
    const float Mu /*= 0.0f*/)
{
    if (!ParentComponent)
        return false;

    if (SupportPts.Num() != 2)
        return false;

    // ---------------------------------------------------------------------
    // Geometry
    // ---------------------------------------------------------------------
    const FVector N = SupportN.GetSafeNormal();
    if (N.IsNearlyZero())
        return false;

    const FVector P1 = SupportPts[0];
    const FVector P2 = SupportPts[1];

    FVector EdgeDir = P2 - P1;

    // Project edge onto the tangent plane (robustness)
    EdgeDir -= FVector::DotProduct(EdgeDir, N) * N;

    const float EdgeLen = EdgeDir.Size();
    if (EdgeLen < KINDA_SMALL_NUMBER)
        return false;

    EdgeDir /= EdgeLen;

    // Reject degenerate case (edge almost parallel to normal)
    if (FMath::Abs(FVector::DotProduct(EdgeDir, N)) > 0.95f)
        return false;

    // if plane is too far away -> NO SUPPORT
    const float distCOM = PlaneSignedDist(ParentComponent->GetPhysLocation(), N, GroundPlanePointWS);
    constexpr float MaxSupportDist = 1.0f; // cm
    if (distCOM > MaxSupportDist)
    {
        return false;
    }

    const float vN_com = FVector::DotProduct(
        ParentComponent->GetPhysVelocity(), N
    );
    // if COM is getting away from ground plane -> NO SUPPORT
    constexpr float COM_SEP_EPS = -0.5f; // cm/s
    if (vN_com > COM_SEP_EPS)
    {
        return false;
    }

    // ---------------------------------------------------------------------
    // 1) Linear support constraint (NO TORQUE)
    //    Cancel average velocity INTO the plane at COM
    // ---------------------------------------------------------------------
    const FVector v1 = GetVelocityAtPoint(P1);
    const FVector v2 = GetVelocityAtPoint(P2);
    const bool bIsInAutoRecover = ParentComponent->IsInAutoRecover();

    const float vN = bIsInAutoRecover ? vN_com :
        0.5f * (FVector::DotProduct(v1, N) + FVector::DotProduct(v2, N));

    constexpr float VN_EPS = -0.5f; // cm/s
    if (vN < VN_EPS)
    {
        if (!bIsInAutoRecover)
        {
            // Apply impulse at COM -> zero angular coupling by construction
            const float lambdaN = -vN * GetMass();
            const FVector J = lambdaN * N;

            ApplyImpulse(J, GetCOM());
#if !UE_BUILD_SHIPPING
            // UE_LOG(BoxSubBodyLog, Log, TEXT("[EdgeSupport] Linear support @COM vN=%.3f J=%s"), vN, *J.ToString());
#endif
        }
        else
        {
            // Apply impulse at COM to avoid torque which could cancel auto-recover
            ParentComponent->AddPhysVelocity(-vN * N);
#if !UE_BUILD_SHIPPING
            // UE_LOG(BoxSubBodyLog, Log, TEXT("[EdgeSupport] Linear support @COM vN=%.3f"), vN);
#endif
        }
    }

    // ---------------------------------------------------------------------
    // Detect if edge is still REALLY supporting (normal force meaningful)
    // ---------------------------------------------------------------------
    constexpr float SupportVNThreshold = -5.0f; // cm/s
    const bool bEdgeStillSupporting =
        vN < SupportVNThreshold &&
        FVector::DotProduct(N, FVector::UpVector) > 0.7f;

    if (ParentComponent->IsSubBodyInAutoRecoverMode())
    {
        // Do NOT project angular velocity
        return false; // skip edge support
    }

    // ---------------------------------------------------------------------
    // 2) Angular constraint (HARD projection)
    //    Allow rotation ONLY around the edge axis
    // ---------------------------------------------------------------------
    if (bEdgeStillSupporting)
    {
        if (!bIsInAutoRecover && IsMainSubBody())
        {
            const FVector Omega = ParentComponent->GetPhysAngularVelocity();
            const float wEdge = FVector::DotProduct(Omega, EdgeDir);

            // Project angular velocity onto edge axis
            const FVector OmegaProjected = wEdge * EdgeDir;

            ParentComponent->AddPhysAngularVelocity(OmegaProjected - Omega);

#if !UE_BUILD_SHIPPING
            /*UE_LOG(
                BoxSubBodyLog,
                Log,
                TEXT("[EdgeSupport] Angular projection Omega=(%.3f %.3f %.3f) Edge=(%.3f %.3f %.3f)"),
                OmegaProjected.X, OmegaProjected.Y, OmegaProjected.Z,
                EdgeDir.X, EdgeDir.Y, EdgeDir.Z
            );*/
#endif
        }
        else
        {
            // Mode auto-recover :
            // enable rotation only on Fall axis
            const FVector Omega = ParentComponent->GetPhysAngularVelocity();

            const FVector FallAxis =
                FVector::CrossProduct(EdgeDir, N).GetSafeNormal();

            const float wEdge = FVector::DotProduct(Omega, EdgeDir);
            const float wFall = FVector::DotProduct(Omega, FallAxis);

            // Conserver edge + fall, only suppress normal component
            ParentComponent->SetPhysAngularVelocity(
                wEdge * EdgeDir +
                wFall * FallAxis
            );
        }
    }

    return true;
}

FMatrix UBoxSubBody::InitInvInertiaTensor() const
{
    if (!ParentComponent)
    {
        return FMatrix::Identity;
    }
    FVector HitboxRelLoc = GetLocalOffset(); // cm
    FQuat HitboxRelRot = GetLocalRotation();

    // 1. Compute d in hitbox local frame (important)
    FVector d_localChassis = HitboxRelLoc; // -COMLocal; // chassis center is the COM so COMLocal = FVectore::ZeroVector
    FVector d_in_box = HitboxRelRot.UnrotateVector(d_localChassis);

	// 2. Compute inverse inertia tensor at COM, expressed in box local frame
    const FVector Idiag =
        SSBox::ComputeInertiaTensor(BoxExtent, GetMass(), 10.0f); // necessary to scale to have a more stable hitbox

    const FMatrix I_atCOM =
        SSBox::ConvertBoxInertiaToCOM(Idiag, d_in_box, GetMass());

    return I_atCOM.InverseFast();
}

FMatrix UBoxSubBody::ComputeWorldInvInertiaTensor() const
{
    if (!ParentComponent)
        return FMatrix::Identity;
    const FQuat ChassisQ = ParentComponent->GetPhysRotation();
    const FQuat HitboxRelQ = GetLocalRotation();

    // Hitbox local -> world
    const FQuat Qworld = ChassisQ * HitboxRelQ;

    const FMatrix R = FRotationMatrix::Make(Qworld.Rotator());
    const FMatrix Iworld = R * InvInertiaLocal * R.GetTransposed();

    return Iworld;
}


#if WITH_EDITOR
void UBoxSubBody::SetShowFlags(const FEngineShowFlags& InShowFlags)
{
    ShowFlags = InShowFlags;
    MarkRenderStateDirty();
}
#endif // WITH_EDITOR

FPrimitiveSceneProxy* UBoxSubBody::CreateSceneProxy()
{
    /** Represents a UBoxSubBody to the scene manager. */
    class FBoxSceneProxy final : public FPrimitiveSceneProxy
    {
    public:
        SIZE_T GetTypeHash() const override
        {
            static size_t UniquePointer;
            return reinterpret_cast<size_t>(&UniquePointer);
        }

        FBoxSceneProxy(const UBoxSubBody* InComponent)
            : FPrimitiveSceneProxy(InComponent)
            , bDrawOnlyIfSelected(InComponent->bDrawOnlyIfSelected)
            , BoxExtents(InComponent->BoxExtent)
            , BoxColor(InComponent->ShapeColor)
            , LineThickness(InComponent->LineThickness)
        {
            bWillEverBeLit = false;

#if WITH_EDITOR
            struct FIterSink
            {
                FIterSink(const FEngineShowFlags InSelectedShowFlags)
                    : SelectedShowFlags(InSelectedShowFlags)
                {
                    SelectedShowFlagIndices.SetNum(FEngineShowFlags::SF_FirstCustom, false);
                }

                bool HandleShowFlag(uint32 InIndex, const FString& InName)
                {
                    if (SelectedShowFlags.GetSingleFlag(InIndex) == true)
                    {
                        SelectedShowFlagIndices.PadToNum(InIndex + 1, false);
                        SelectedShowFlagIndices[InIndex] = true;
                    }

                    return true;
                }

                bool OnEngineShowFlag(uint32 InIndex, const FString& InName)
                {
                    return HandleShowFlag(InIndex, InName);
                }

                bool OnCustomShowFlag(uint32 InIndex, const FString& InName)
                {
                    return HandleShowFlag(InIndex, InName);
                }

                const FEngineShowFlags SelectedShowFlags;

                TBitArray<> SelectedShowFlagIndices;
            };

            FIterSink Sink(InComponent->ShowFlags);
            FEngineShowFlags::IterateAllFlags(Sink);
            SelectedShowFlagIndices = MoveTemp(Sink.SelectedShowFlagIndices);
#endif // WITH_EDITOR
        }

        virtual void GetDynamicMeshElements(const TArray<const FSceneView*>& Views, const FSceneViewFamily& ViewFamily, uint32 VisibilityMap, FMeshElementCollector& Collector) const override
        {
            QUICK_SCOPE_CYCLE_COUNTER(STAT_BoxSceneProxy_GetDynamicMeshElements);

            const FMatrix& LocalToWorld = GetLocalToWorld();

            for (int32 ViewIndex = 0; ViewIndex < Views.Num(); ViewIndex++)
            {
                if (VisibilityMap & (1 << ViewIndex))
                {
                    const FSceneView* View = Views[ViewIndex];

                    const FLinearColor DrawColor = GetViewSelectionColor(BoxColor, *View, IsSelected(), IsHovered(), false, IsIndividuallySelected());

                    FPrimitiveDrawInterface* PDI = Collector.GetPDI(ViewIndex);
                    DrawOrientedWireBox(PDI, LocalToWorld.GetOrigin(), LocalToWorld.GetScaledAxis(EAxis::X), LocalToWorld.GetScaledAxis(EAxis::Y), LocalToWorld.GetScaledAxis(EAxis::Z), BoxExtents, DrawColor, SDPG_World, LineThickness);
                }
            }
        }

        virtual FPrimitiveViewRelevance GetViewRelevance(const FSceneView* View) const override
        {
            const bool bProxyVisible = !bDrawOnlyIfSelected || IsSelected();

            // Should we draw this because collision drawing is enabled, and we have collision
            const bool bShowForCollision = View->Family->EngineShowFlags.Collision && IsCollisionEnabled();

            FPrimitiveViewRelevance Result;
            Result.bDrawRelevance = (IsShown(View) && bProxyVisible) || bShowForCollision;
#if WITH_EDITOR
            bool bAreAllSelectedFlagsEnabled = true;
            for (TConstSetBitIterator<> It(SelectedShowFlagIndices); It; ++It)
            {
                bAreAllSelectedFlagsEnabled &= View->Family->EngineShowFlags.GetSingleFlag(It.GetIndex());
            }

            Result.bDrawRelevance &= bAreAllSelectedFlagsEnabled;
#endif // WITH_EDITOR
            Result.bDynamicRelevance = true;
            Result.bShadowRelevance = IsShadowCast(View);
            Result.bEditorPrimitiveRelevance = UseEditorCompositing(View);
            return Result;
        }
        virtual uint32 GetMemoryFootprint(void) const override { return(sizeof(*this) + GetAllocatedSize()); }
        uint32 GetAllocatedSize(void) const { return(FPrimitiveSceneProxy::GetAllocatedSize()); }

    private:
        const uint32	bDrawOnlyIfSelected : 1;
        const FVector	BoxExtents;
        const FColor	BoxColor;
        const float		LineThickness;
#if WITH_EDITOR
        TBitArray<>		SelectedShowFlagIndices;
#endif // WITH_EDITOR
    };

    return new FBoxSceneProxy(this);
}

FBoxSphereBounds UBoxSubBody::CalcBounds(const FTransform& LocalToWorld) const
{
    return FBoxSphereBounds(FBox(-BoxExtent, BoxExtent)).TransformBy(LocalToWorld);
}

template <EShapeBodySetupHelper UpdateBodySetupAction, typename BodySetupType>
bool InvalidateOrUpdateBoxBodySetup(BodySetupType& ShapeBodySetup, bool bUseArchetypeBodySetup, FVector BoxExtent)
{
    check((bUseArchetypeBodySetup && UpdateBodySetupAction == EShapeBodySetupHelper::InvalidateSharingIfStale) || (!bUseArchetypeBodySetup && UpdateBodySetupAction == EShapeBodySetupHelper::UpdateBodySetup));
    check(ShapeBodySetup->AggGeom.BoxElems.Num() == 1);
    FKBoxElem* se = ShapeBodySetup->AggGeom.BoxElems.GetData();

    // @todo do we allow this now?
    // check for malformed values
    if (BoxExtent.X < UE_KINDA_SMALL_NUMBER)
    {
        BoxExtent.X = 1.0f;
    }

    if (BoxExtent.Y < UE_KINDA_SMALL_NUMBER)
    {
        BoxExtent.Y = 1.0f;
    }

    if (BoxExtent.Z < UE_KINDA_SMALL_NUMBER)
    {
        BoxExtent.Z = 1.0f;
    }

    float XExtent = BoxExtent.X * 2.f;
    float YExtent = BoxExtent.Y * 2.f;
    float ZExtent = BoxExtent.Z * 2.f;

    if (UpdateBodySetupAction == EShapeBodySetupHelper::UpdateBodySetup)
    {
        // now set the PhysX data values
        se->SetTransform(FTransform::Identity);
        se->X = XExtent;
        se->Y = YExtent;
        se->Z = ZExtent;
    }
    else if (se->X != XExtent || se->Y != YExtent || se->Z != ZExtent)
    {
        ShapeBodySetup = nullptr;
        bUseArchetypeBodySetup = false;
    }

    return bUseArchetypeBodySetup;
}

void UBoxSubBody::UpdateBodySetup()
{
    if (PrepareSharedBodySetup<UBoxSubBody>())
    {
        bUseArchetypeBodySetup = InvalidateOrUpdateBoxBodySetup<EShapeBodySetupHelper::InvalidateSharingIfStale>(ShapeBodySetup, bUseArchetypeBodySetup, BoxExtent);
    }

    CreateShapeBodySetupIfNeeded<FKBoxElem>();

    if (!bUseArchetypeBodySetup)
    {
        InvalidateOrUpdateBoxBodySetup<EShapeBodySetupHelper::UpdateBodySetup>(ShapeBodySetup, bUseArchetypeBodySetup, BoxExtent);
    }
}
