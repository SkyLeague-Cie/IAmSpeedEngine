// Fill out your copyright notice in the Description page of Project Settings.


#include "SolidSubBody.h"
#include "IAmSpeed/Components/ISpeedComponent.h"
#include "IAmSpeed/Base/PhysicalContactConstraint.h"
#include "Configs/SubBodyConfig.h"
#include "HAL/IConsoleManager.h"

static TAutoConsoleVariable<float> CVarIAmSpeedSphereBoxRestitutionOverride(
    TEXT("p.IAmSpeed.Collision.SphereBoxRestitutionOverride"),
    -1.0f,
    TEXT("Forces restitution for sphere/box contacts when >= 0. Negative values use subbody overrides or the normal restitution mix."));

static TAutoConsoleVariable<float> CVarIAmSpeedSphereBoxFrictionOverride(
    TEXT("p.IAmSpeed.Collision.SphereBoxFrictionOverride"),
    -1.0f,
    TEXT("Forces friction for sphere/box contacts when >= 0. Negative values use the normal friction mix."));

static TAutoConsoleVariable<int32> CVarIAmSpeedSphereBoxRestitutionMode(
    TEXT("p.IAmSpeed.Collision.SphereBoxRestitutionMode"),
    0,
    TEXT("Sphere/box restitution mode when SphereBoxRestitutionOverride < 0. 0=Global, 1=AtContact, 2=Custom."));

static TAutoConsoleVariable<float> CVarIAmSpeedSphereBoxRestitutionProfileCenter(
    TEXT("p.IAmSpeed.Collision.SphereBoxRestitutionProfile.Center"),
    0.50f,
    TEXT("Sphere/box restitution used near the box local XY center when AtContact mode is enabled."));

static TAutoConsoleVariable<float> CVarIAmSpeedSphereBoxRestitutionProfileEdge(
    TEXT("p.IAmSpeed.Collision.SphereBoxRestitutionProfile.Edge"),
    0.50f,
    TEXT("Sphere/box restitution used near the box local XY front/back/side edges when AtContact mode is enabled."));

static TAutoConsoleVariable<float> CVarIAmSpeedSphereBoxRestitutionProfileStartDistanceNorm(
    TEXT("p.IAmSpeed.Collision.SphereBoxRestitutionProfile.StartDistanceNorm"),
    0.0f,
    TEXT("Normalized local XY elliptical distance where the sphere/box restitution profile starts blending from center to edge."));

static TAutoConsoleVariable<float> CVarIAmSpeedSphereBoxRestitutionProfileEndDistanceNorm(
    TEXT("p.IAmSpeed.Collision.SphereBoxRestitutionProfile.EndDistanceNorm"),
    1.0f,
    TEXT("Normalized local XY elliptical distance where the sphere/box restitution profile reaches the edge value."));

static TAutoConsoleVariable<float> CVarIAmSpeedSphereBoxRestitutionProfileBlendPower(
    TEXT("p.IAmSpeed.Collision.SphereBoxRestitutionProfile.BlendPower"),
    1.0f,
    TEXT("Exponent applied to the normalized sphere/box restitution profile blend. < 1 reaches edge restitution earlier; > 1 later."));

static TAutoConsoleVariable<float> CVarIAmSpeedSphereBoxRestitutionProfileReferenceLocalX(
    TEXT("p.IAmSpeed.Collision.SphereBoxRestitutionProfile.ReferenceLocalX"),
    0.0f,
    TEXT("Local X reference point, in cm, used as the center of the AtContact sphere/box restitution profile."));

static TAutoConsoleVariable<float> CVarIAmSpeedSphereBoxRestitutionProfileReferenceLocalY(
    TEXT("p.IAmSpeed.Collision.SphereBoxRestitutionProfile.ReferenceLocalY"),
    0.0f,
    TEXT("Local Y reference point, in cm, used as the center of the AtContact sphere/box restitution profile."));

static TAutoConsoleVariable<float> CVarIAmSpeedSphereBoxRestitutionProfileReferenceLocalZ(
    TEXT("p.IAmSpeed.Collision.SphereBoxRestitutionProfile.ReferenceLocalZ"),
    0.0f,
    TEXT("Local Z reference point, in cm, used as the center of the AtContact sphere/box restitution profile."));

static TAutoConsoleVariable<float> CVarIAmSpeedSphereBoxRestitutionProfileXWeight(
    TEXT("p.IAmSpeed.Collision.SphereBoxRestitutionProfile.XWeight"),
    1.0f,
    TEXT("Weight of normalized local X distance in the AtContact sphere/box restitution profile."));

static TAutoConsoleVariable<float> CVarIAmSpeedSphereBoxRestitutionProfileYWeight(
    TEXT("p.IAmSpeed.Collision.SphereBoxRestitutionProfile.YWeight"),
    1.0f,
    TEXT("Weight of normalized local Y distance in the AtContact sphere/box restitution profile."));

static TAutoConsoleVariable<float> CVarIAmSpeedSphereBoxRestitutionProfileZWeight(
    TEXT("p.IAmSpeed.Collision.SphereBoxRestitutionProfile.ZWeight"),
    0.0f,
    TEXT("Weight of normalized local Z distance in the AtContact sphere/box restitution profile. 0 keeps the legacy local XY profile."));

namespace
{
    float ResolveSphereBoxGlobalRestitution(
        const USolidSubBody& Sphere,
        const USolidSubBody& Box,
        const float FallbackRestitution)
    {
        const float SphereOverride = Sphere.GetSphereBoxRestitutionOverride();
        if (SphereOverride >= 0.f)
        {
            return FMath::Clamp(SphereOverride, 0.f, 1.f);
        }

        const float BoxOverride = Box.GetSphereBoxRestitutionOverride();
        if (BoxOverride >= 0.f)
        {
            return FMath::Clamp(BoxOverride, 0.f, 1.f);
        }

        return FMath::Clamp(FallbackRestitution, 0.f, 1.f);
    }

    ERestitutionResolveMode GetSphereBoxRestitutionMode()
    {
        const int32 RawMode = CVarIAmSpeedSphereBoxRestitutionMode.GetValueOnAnyThread();
        switch (RawMode)
        {
        case 1:
            return ERestitutionResolveMode::AtContact;
        case 2:
            return ERestitutionResolveMode::Custom;
        case 0:
        default:
            return ERestitutionResolveMode::Global;
        }
    }

    float ResolveSphereBoxAtContactRestitution(const FRestitutionResolveContext& Context)
    {
        const float CenterRestitution = FMath::Clamp(
            CVarIAmSpeedSphereBoxRestitutionProfileCenter.GetValueOnAnyThread(), 0.f, 1.f);
        const float EdgeRestitution = FMath::Clamp(
            CVarIAmSpeedSphereBoxRestitutionProfileEdge.GetValueOnAnyThread(), 0.f, 1.f);
        const float StartDistanceNorm = FMath::Clamp(
            CVarIAmSpeedSphereBoxRestitutionProfileStartDistanceNorm.GetValueOnAnyThread(), 0.f, 1.f);
        const float EndDistanceNorm = FMath::Clamp(
            CVarIAmSpeedSphereBoxRestitutionProfileEndDistanceNorm.GetValueOnAnyThread(), StartDistanceNorm + KINDA_SMALL_NUMBER, 1.f);
        const float BlendPower = FMath::Max(
            KINDA_SMALL_NUMBER, CVarIAmSpeedSphereBoxRestitutionProfileBlendPower.GetValueOnAnyThread());
        const FVector ReferenceLocalPoint(
            CVarIAmSpeedSphereBoxRestitutionProfileReferenceLocalX.GetValueOnAnyThread(),
            CVarIAmSpeedSphereBoxRestitutionProfileReferenceLocalY.GetValueOnAnyThread(),
            CVarIAmSpeedSphereBoxRestitutionProfileReferenceLocalZ.GetValueOnAnyThread());
        const float XWeight = FMath::Max(0.f, CVarIAmSpeedSphereBoxRestitutionProfileXWeight.GetValueOnAnyThread());
        const float YWeight = FMath::Max(0.f, CVarIAmSpeedSphereBoxRestitutionProfileYWeight.GetValueOnAnyThread());
        const float ZWeight = FMath::Max(0.f, CVarIAmSpeedSphereBoxRestitutionProfileZWeight.GetValueOnAnyThread());

        const float XNorm = Context.BoxExtent.X > KINDA_SMALL_NUMBER
            ? (Context.BoxLocalContactPoint.X - ReferenceLocalPoint.X) / Context.BoxExtent.X
            : 0.f;
        const float YNorm = Context.BoxExtent.Y > KINDA_SMALL_NUMBER
            ? (Context.BoxLocalContactPoint.Y - ReferenceLocalPoint.Y) / Context.BoxExtent.Y
            : 0.f;
        const float ZNorm = Context.BoxExtent.Z > KINDA_SMALL_NUMBER
            ? (Context.BoxLocalContactPoint.Z - ReferenceLocalPoint.Z) / Context.BoxExtent.Z
            : 0.f;
        const float DistanceNorm = FMath::Clamp(FMath::Sqrt(
            XWeight * XWeight * XNorm * XNorm +
            YWeight * YWeight * YNorm * YNorm +
            ZWeight * ZWeight * ZNorm * ZNorm), 0.f, 1.f);
        const float RawBlend = FMath::Clamp(
            (DistanceNorm - StartDistanceNorm) / (EndDistanceNorm - StartDistanceNorm),
            0.f, 1.f);
        const float ShapedBlend = FMath::Pow(RawBlend, BlendPower);
        const float Blend = FMath::SmoothStep(0.f, 1.f, ShapedBlend);
        return FMath::Lerp(CenterRestitution, EdgeRestitution, Blend);
    }
}

void USolidSubBody::Initialize(ISpeedComponent* InParentComponent)
{
	Super::Initialize(InParentComponent);

	if (InParentComponent)
	{
		SubBodyConfig Config = InParentComponent->GetSubBodyConfig(*this);
        if (Config.bValid)
        {
            Mass = Config.Mass;
            Restitution = Config.Restitution;
            StaticFriction = Config.StaticFriction;
            DynamicFriction = Config.DynamicFriction;
            ImpactThreshold = Config.ImpactThreshold;
            bIsMainSubBody = Config.bIsMainSubBody;
            EnableFakePhysics = Config.bEnableFakePhysics;
            HitDamping = FMath::Clamp(Config.HitDamping, 0.f, 1.f);
        }
		InvInertiaLocal = InitInvInertiaTensor();
	}
}

void USolidSubBody::ApplyImpulse(const FVector& LinearImpulse, const FVector& WorldPoint)
{
	if (ParentComponent)
	{
		ParentComponent->ApplyImpulse(LinearImpulse, WorldPoint, this);
	}
}

void USolidSubBody::SetSphereBoxRestitutionOverride(float InRestitution)
{
    SphereBoxRestitutionOverride = InRestitution < 0.f ? -1.f : FMath::Clamp(InRestitution, 0.f, 1.f);
}

void USolidSubBody::RegisterCurrentHitAsConstraint()
{
    if (!CurrentHit.bBlockingHit || !CurrentHit.Component.IsValid())
    {
        return;
    }

    RegisterContactAsConstraint(
        CurrentHit.ImpactPoint,
        CurrentHit.ImpactNormal,
        CurrentHit.Component.Get(),
        Cast<USolidSubBody>(CurrentHit.SubBody.Get()),
        CurrentHit.PenetrationDepth,
        CurrentHit.TOI,
        false
    );
}

void USolidSubBody::RegisterContactAsConstraint(
    const FVector& ContactPointWS,
    const FVector& NormalWS,
    UPrimitiveComponent* OtherComponent,
    USolidSubBody* OtherSubBody,
    float PenetrationDepth,
    float TOI,
    bool bPersistent)
{
    if (!ParentComponent || !OtherComponent)
    {
        return;
    }

    if (!OtherSubBody)
    {
        OtherSubBody = Cast<USolidSubBody>(OtherComponent);
    }

    const bool bStaticConstraint = OtherComponent->GetCollisionObjectType() == ECC_WorldStatic;
    const bool bDynamicSolidConstraint = OtherSubBody && OtherSubBody->ParentComponent;
    if (!bStaticConstraint && !bDynamicSolidConstraint)
    {
        return;
    }

    const bool bRegisterThisConstraint =
        bStaticConstraint || (bDynamicSolidConstraint && bEnableConstraintOnSubBodyHit);
    const bool bRegisterOtherConstraint =
        bDynamicSolidConstraint &&
        OtherSubBody->bEnableConstraintOnSubBodyHit &&
        OtherSubBody->ParentComponent != ParentComponent;
    if (!bRegisterThisConstraint && !bRegisterOtherConstraint)
    {
        return;
    }

    const FVector N = NormalWS.GetSafeNormal();
    if (N.IsNearlyZero())
    {
        return;
    }

    FPhysicalContactConstraint Constraint;
    Constraint.Normal = N;
    Constraint.ContactPoint = ContactPointWS;
    Constraint.OtherComponent = OtherComponent;
    Constraint.SourceSubBody = this;
    Constraint.PenetrationDepth = PenetrationDepth;
    Constraint.TOI = TOI;
    Constraint.PairKey = USSubBody::MakePairKey(this, OtherComponent);
    Constraint.bPersistent = bStaticConstraint && bPersistent;

    if (bRegisterThisConstraint)
    {
        ParentComponent->RegisterPhysicalConstraint(Constraint);
    }

    if (bRegisterOtherConstraint)
    {
        FPhysicalContactConstraint OtherConstraint;
        OtherConstraint.Normal = -N;
        OtherConstraint.ContactPoint = ContactPointWS;
        OtherConstraint.OtherComponent = this;
        OtherConstraint.SourceSubBody = OtherSubBody;
        OtherConstraint.PenetrationDepth = PenetrationDepth;
        OtherConstraint.TOI = TOI;
        OtherConstraint.PairKey = USSubBody::MakePairKey(OtherSubBody, this);
        OtherConstraint.bPersistent = false;
        OtherSubBody->ParentComponent->RegisterPhysicalConstraint(OtherConstraint);

        /*
        UE_LOG(LogTemp, Log,
            TEXT("[Constraint] Frame=%d SubBody=%s P=%s N=%s Persistent=%d"),
            ParentComponent ? ParentComponent->NumFrame() : -1,
            *GetName(),
            *ContactPointWS.ToString(),
            *N.ToString(),
            bPersistent ? 1 : 0
        );
        */
    }
}

void USolidSubBody::RegisterContactManifoldAsConstraints(
    const TArray<FVector>& ContactPointsWS,
    const FVector& NormalWS,
    UPrimitiveComponent* OtherComponent,
    float PenetrationDepth,
    float TOI,
    bool bPersistent,
    int32 MaxContacts)
{
    if (!ParentComponent || !OtherComponent || ContactPointsWS.Num() == 0)
    {
        return;
    }

    const FVector N = NormalWS.GetSafeNormal();
    if (N.IsNearlyZero())
    {
        return;
    }

    // Cas simple : peu de points, on les enregistre tous.
    if (ContactPointsWS.Num() <= MaxContacts)
    {
        for (const FVector& P : ContactPointsWS)
        {
            RegisterContactAsConstraint(P, N, OtherComponent, nullptr, PenetrationDepth, TOI, bPersistent);
        }
        return;
    }

    // Cas rare : trop de points. On garde des points répartis autour du COM,
    // ce qui donne de meilleurs bras de levier pour la projection angulaire.
    const FVector COM = GetKinematicState().Location                     ;

    TArray<FVector> Sorted = ContactPointsWS;
    Sorted.Sort([&COM](const FVector& A, const FVector& B)
        {
            return FVector::DistSquared(A, COM) > FVector::DistSquared(B, COM);
        });

    const int32 NumToRegister = FMath::Min(MaxContacts, Sorted.Num());
    for (int32 i = 0; i < NumToRegister; ++i)
    {
        RegisterContactAsConstraint(Sorted[i], N, OtherComponent, nullptr, PenetrationDepth, TOI, bPersistent);
    }
}

float USolidSubBody::MixRestitution(float eA, float eB, EMixMode Mode)
{
    switch (Mode)
    {
    case EMixMode::E_Average:
        return 0.5f * (eA + eB);
    case EMixMode::E_Multiply:
        return eA * eB;
    case EMixMode::E_Max:
        return FMath::Max(eA, eB);
    case EMixMode::E_Min:
        return FMath::Min(eA, eB);
    }
	return 0.f; // default case, should not happen
}

float USolidSubBody::MixFriction(float muA, float muB, EMixMode Mode)
{
    switch (Mode)
    {
    case EMixMode::E_Average:
        return 0.5f * (muA + muB);
    case EMixMode::E_Multiply:
        return muA * muB;
    case EMixMode::E_Max:
        return FMath::Max(muA, muB);
    case EMixMode::E_Min:
        return FMath::Min(muA, muB);
    }
	return 0.f; // default case, should not happen
}

float USolidSubBody::ResolveSphereBoxFriction(float FallbackFriction)
{
    const float ForcedOverride = CVarIAmSpeedSphereBoxFrictionOverride.GetValueOnAnyThread();
    if (ForcedOverride >= 0.0f)
    {
        return FMath::Clamp(ForcedOverride, 0.0f, 1.0f);
    }
    return FallbackFriction;
}

float USolidSubBody::ResolveSphereBoxRestitution(const USolidSubBody& Sphere, const USolidSubBody& Box, float FallbackRestitution)
{
    const float ForcedOverride = CVarIAmSpeedSphereBoxRestitutionOverride.GetValueOnAnyThread();
    if (ForcedOverride >= 0.f)
    {
        return FMath::Clamp(ForcedOverride, 0.f, 1.f);
    }

    return ResolveSphereBoxGlobalRestitution(Sphere, Box, FallbackRestitution);
}

float USolidSubBody::ResolveSphereBoxRestitutionAtContact(
    const USolidSubBody& Sphere,
    const USolidSubBody& Box,
    float FallbackRestitution,
    const FVector& BoxLocalContactPoint,
    const FVector& BoxLocalContactNormal,
    const FVector& BoxExtent,
    const FVector& ContactPointWS)
{
    const float ForcedOverride = CVarIAmSpeedSphereBoxRestitutionOverride.GetValueOnAnyThread();
    if (ForcedOverride >= 0.f)
    {
        return FMath::Clamp(ForcedOverride, 0.f, 1.f);
    }

    const float GlobalRestitution = ResolveSphereBoxGlobalRestitution(Sphere, Box, FallbackRestitution);
    FRestitutionResolveContext Context;
    Context.ContactPointWS = ContactPointWS;
    Context.BoxLocalContactPoint = BoxLocalContactPoint;
    Context.BoxLocalContactNormal = BoxLocalContactNormal;
    Context.BoxExtent = BoxExtent;

    switch (GetSphereBoxRestitutionMode())
    {
    case ERestitutionResolveMode::AtContact:
        return ResolveSphereBoxAtContactRestitution(Context);
    case ERestitutionResolveMode::Custom:
        return FMath::Clamp(Box.ResolveRestitutionCustom(Sphere, Context, GlobalRestitution), 0.f, 1.f);
    case ERestitutionResolveMode::Global:
    default:
        return GlobalRestitution;
    }
}

FVector USolidSubBody::GetVelocityAtPoint(const FVector& Point) const
{
    if (ParentComponent)
    {
        return ParentComponent->GetPhysVelocityAtPoint(Point);
    }
    return FVector::ZeroVector;
}

void USolidSubBody::SolveOverlap(
    ISpeedComponent& ThisComp, float MassA, const FMatrix& InvIA, const SKinematic& KA,
    ISpeedComponent* OtherComp, float MassB, const FMatrix& InvIB, const SKinematic& KB,
    const FVector& P, const FVector& N_OtherToThis,
    float PenDepth,
    float SlopCm,
    float Percent,
    float KillVelThreshold // cm/s
)
{
    const FVector N = N_OtherToThis.GetSafeNormal();
    if (N.IsNearlyZero()) return;

    const float InvMassA = (MassA > KINDA_SMALL_NUMBER) ? 1.f / MassA : 0.f;
    const float InvMassB = (OtherComp && MassB > KINDA_SMALL_NUMBER) ? 1.f / MassB : 0.f;
    const float InvMassSum = InvMassA + InvMassB;

    // -------------------------
    // 1) Positional correction
    // -------------------------
    if (PenDepth > SlopCm && InvMassSum > KINDA_SMALL_NUMBER)
    {
        const float corrMag = (PenDepth - SlopCm) * Percent;
        const FVector corr = corrMag * N;

        const float wA = InvMassA / InvMassSum;
        const float wB = InvMassB / InvMassSum;

        ThisComp.SetPhysLocation(ThisComp.GetPhysLocation() + corr * wA);
        if (OtherComp)
            OtherComp->SetPhysLocation(OtherComp->GetPhysLocation() - corr * wB);
    }

    // -------------------------
    // 2) Kill inward normal velocity (resting impulse, e=0)
    // -------------------------
    const FVector rA = P - KA.Location;
    const FVector vA = KA.Velocity + FVector::CrossProduct(KA.AngularVelocity, rA);

    FVector vRel = vA;
    FVector rB(0.f);
    if (OtherComp)
    {
        rB = P - KB.Location;
        const FVector vB = KB.Velocity + FVector::CrossProduct(KB.AngularVelocity, rB);
        vRel = vA - vB;
    }

    const float vRelN = FVector::DotProduct(vRel, N);
    if (vRelN < -KillVelThreshold)
    {
        // denomN (same as your impulse solver)
        const FVector rAxN = FVector::CrossProduct(rA, N);
        const FVector invIA_rAxN = InvIA.TransformVector(rAxN);
        const float angA = FVector::DotProduct(N, FVector::CrossProduct(invIA_rAxN, rA));

        float angB = 0.f;
        if (OtherComp)
        {
            const FVector rBxN = FVector::CrossProduct(rB, N);
            const FVector invIB_rBxN = InvIB.TransformVector(rBxN);
            angB = FVector::DotProduct(N, FVector::CrossProduct(invIB_rBxN, rB));
        }

        const float denomN = InvMassA + InvMassB + angA + angB;
        if (denomN > KINDA_SMALL_NUMBER)
        {
            const float jn = -vRelN / denomN; // e=0
            const FVector J = jn * N;
            ThisComp.ApplyImpulse(J, P);
            if (OtherComp) OtherComp->ApplyImpulse(-J, P);
        }
    }
}

FMatrix USolidSubBody::ComputeWorldInvInertiaTensor() const
{
    if (ParentComponent)
    {
        return ParentComponent->ComputeWorldInvInertiaTensorOfSubBody(*this);
    }
    return FMatrix::Identity;
}
