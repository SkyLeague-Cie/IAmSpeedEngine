// Fill out your copyright notice in the Description page of Project Settings.


#include "SolidSubBody.h"
#include "IAmSpeed/Components/ISpeedComponent.h"
#include "IAmSpeed/Base/PhysicalContactConstraint.h"
#include "IAmSpeed/World/Simulation/ExactRoundContactSettings.h"
#include "IAmSpeed/World/Subsystem/SpeedWorldSubsystem.h"
#include "Configs/SubBodyConfig.h"
#include "HAL/IConsoleManager.h"

static TAutoConsoleVariable<int32> CVarIAmSpeedAerialShotContactModel(
    TEXT("p.IAmSpeed.Collision.AerialShotModel"), 1,
    TEXT("Use the shared Aerial car/ball contact model. Startup-only: inertia is cached at body initialization."),
    ECVF_ReadOnly);

bool USolidSubBody::IsAerialShotContactModelEnabled()
{
    if (const auto* RoundSettings = Speed::FScopedExactRoundContactSettings::Current())
        return RoundSettings->AerialShotModel;
    return CVarIAmSpeedAerialShotContactModel.GetValueOnAnyThread() != 0;
}

static TAutoConsoleVariable<float> CVarIAmSpeedSphereBoxRestitutionOverride(
    TEXT("p.IAmSpeed.Collision.SphereBoxRestitutionOverride"),
    -1.0f,
    TEXT("Forces restitution for sphere/box contacts when >= 0. Negative values use subbody overrides or the normal restitution mix."));

static TAutoConsoleVariable<float> CVarIAmSpeedSphereBoxFrictionOverride(
    TEXT("p.IAmSpeed.Collision.SphereBoxFrictionOverride"),
    -1.0f,
    TEXT("Forces friction for sphere/box contacts when >= 0. Values above 1 are valid. Negative values use the normal friction mix."));

static TAutoConsoleVariable<float> CVarIAmSpeedSphereBoxManifoldFrictionOverride(
    TEXT("p.IAmSpeed.Collision.SphereBoxManifoldFrictionOverride"),
    -1.0f,
    TEXT("Forces friction only for persistent sphere/box manifolds when >= 0. Negative values use subbody manifold overrides or ordinary sphere/box friction."));

bool Speed::FExactRoundContactSettings::CaptureSolidSettings() noexcept
{
    const float Restitution = CVarIAmSpeedSphereBoxRestitutionOverride.GetValueOnAnyThread();
    const float Friction = CVarIAmSpeedSphereBoxFrictionOverride.GetValueOnAnyThread();
    const float ManifoldFriction = CVarIAmSpeedSphereBoxManifoldFrictionOverride.GetValueOnAnyThread();
    if (!std::isfinite(Restitution) || !std::isfinite(Friction) || !std::isfinite(ManifoldFriction)) return false;
    if (Restitution >= 0.f) SphereBoxRestitution = FMath::Clamp(Restitution, 0.f, 1.f);
    if (Friction >= 0.f) SphereBoxFriction = Friction;
    if (ManifoldFriction >= 0.f) SphereBoxManifoldFriction = ManifoldFriction;
    AerialShotModel = CVarIAmSpeedAerialShotContactModel.GetValueOnAnyThread() != 0;
    return true;
}

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
}

bool USolidSubBody::AppendExactRoundSolidConfiguration(Speed::FExactRoundConfigurationWriter& Out) const
{
    // Foreign filters require a world identity before they can be encoded here.
    constexpr int32 MaxSlots = 256;
    constexpr int32 MaxIgnoredTypes = 32;
    if (!ParentComponent) return false;
    const auto& Slots = ParentComponent->GetSubBodies();
    if (Slots.Num() > MaxSlots || AlwaysIgnoredComponents.Num() > Slots.Num() ||
        IgnoredSubBodyTypes.Num() > MaxIgnoredTypes ||
        !Out.Unsigned(0x494153534f4c0001ull) ||
        !Out.Float(Mass) || !Out.Boolean(bIsMainSubBody) ||
        !Out.Boolean(bApplyRestForce) || !Out.Boolean(EnableFakePhysics) ||
        !Out.Boolean(bEnableConstraintOnSubBodyHit) ||
        !Out.Float(Restitution) || !Out.Float(StaticFriction) || !Out.Float(DynamicFriction) ||
        !Out.Float(SphereBoxRestitutionOverride) || !Out.Float(SphereBoxFrictionOverride) ||
        !Out.Float(SphereBoxManifoldFrictionOverride) || !Out.Boolean(UsesAerialShotContactModel()) ||
        !Out.Boolean(bUseCoupledContactImpulse) || !Out.Boolean(bUsePostNormalFrictionImpulse) ||
        !Out.Boolean(bUsePersistentBilateralContact) ||
        !Out.Float(CoupledContactMaxLinearSpeed) || !Out.Float(CoupledContactMaxAngularSpeed) ||
        !Out.Float(CoupledContactMaxAbsLocalNormalY) ||
        !Out.Float(ImpactThreshold) || !Out.Float(HitDamping) || !Out.Signed(idx)) return false;

    for (const auto& InertiaRow : InvInertiaLocal.M)
        for (const double Value : InertiaRow)
            if (!Out.Double(Value)) return false;

    if (!Out.Unsigned(AlwaysIgnoredComponents.Num())) return false;
    for (const UPrimitiveComponent* Filter : AlwaysIgnoredComponents)
    {
        const int32 Slot = Slots.IndexOfByPredicate([Filter](const USSubBody* Body) { return Body == Filter; });
        if (!Filter || !Slots.IsValidIndex(Slot) || Slots[Slot]->GetParentComponent() != ParentComponent ||
            !Out.Unsigned(Slot)) return false;
    }

    if (!Out.Unsigned(IgnoredSubBodyTypes.Num())) return false;
    for (const auto Type : IgnoredSubBodyTypes)
        if (!Out.Unsigned(static_cast<uint8>(Type))) return false;
    return true;
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

void USolidSubBody::ChangeSolidContactConfiguration(TFunctionRef<void()> Write)
{
    if (!USpeedWorldSubsystem::TryWriteSolidContactConfiguration(*this, Write))
    {
        UE_LOG(LogTemp, Warning, TEXT("Solid contact configuration is sealed by the exact round owner (%s)"),
            *GetName());
    }
}

void USolidSubBody::SetSphereBoxRestitutionOverride(float InRestitution)
{
    ChangeSolidContactConfiguration([this, InRestitution]
    {
        SphereBoxRestitutionOverride = InRestitution < 0.f ? -1.f : FMath::Clamp(InRestitution, 0.f, 1.f);
    });
}

void USolidSubBody::SetSphereBoxFrictionOverride(float InFriction)
{
    ChangeSolidContactConfiguration([this, InFriction]
    {
        SphereBoxFrictionOverride = InFriction < 0.0f ? -1.0f : InFriction;
    });
}

void USolidSubBody::SetSphereBoxManifoldFrictionOverride(float InFriction)
{
    ChangeSolidContactConfiguration([this, InFriction]
    {
        SphereBoxManifoldFrictionOverride = InFriction < 0.0f ? -1.0f : InFriction;
    });
}

void USolidSubBody::SetUseCoupledContactImpulse(bool bInEnabled)
{
    ChangeSolidContactConfiguration([this, bInEnabled]
    {
        bUseCoupledContactImpulse = bInEnabled;
    });
}

void USolidSubBody::SetUsePostNormalFrictionImpulse(bool bInEnabled)
{
    ChangeSolidContactConfiguration([this, bInEnabled]
    {
        bUsePostNormalFrictionImpulse = bInEnabled;
    });
}

void USolidSubBody::SetUsePersistentBilateralContact(bool bInEnabled)
{
    ChangeSolidContactConfiguration([this, bInEnabled]
    {
        bUsePersistentBilateralContact = bInEnabled;
    });
}

void USolidSubBody::SetCoupledContactKinematicLimits(float MaxLinearSpeed, float MaxAngularSpeed)
{
    ChangeSolidContactConfiguration([this, MaxLinearSpeed, MaxAngularSpeed]
    {
        CoupledContactMaxLinearSpeed = MaxLinearSpeed;
        CoupledContactMaxAngularSpeed = MaxAngularSpeed;
    });
}

void USolidSubBody::SetCoupledContactMaxAbsLocalNormalY(float MaxAbsLocalNormalY)
{
    ChangeSolidContactConfiguration([this, MaxAbsLocalNormalY]
    {
        CoupledContactMaxAbsLocalNormalY = MaxAbsLocalNormalY;
    });
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

float USolidSubBody::ResolveSphereBoxFriction(
    const USolidSubBody& Sphere,
    const USolidSubBody& Box,
    float FallbackFriction)
{
    const auto* RoundSettings = Speed::FScopedExactRoundContactSettings::Current();
    const float ForcedOverride = RoundSettings ? RoundSettings->SphereBoxFriction.value_or(-1.f) :
        CVarIAmSpeedSphereBoxFrictionOverride.GetValueOnAnyThread();
    if (ForcedOverride >= 0.0f)
    {
        return ForcedOverride;
    }
    const float SphereOverride = Sphere.GetSphereBoxFrictionOverride();
    const float BoxOverride = Box.GetSphereBoxFrictionOverride();
    if (SphereOverride >= 0.0f || BoxOverride >= 0.0f)
    {
        return FMath::Max(SphereOverride, BoxOverride);
    }
    return FallbackFriction;
}

float USolidSubBody::ResolveSphereBoxManifoldFriction(
    const USolidSubBody& Sphere,
    const USolidSubBody& Box,
    const float FallbackFriction)
{
    const auto* RoundSettings = Speed::FScopedExactRoundContactSettings::Current();
    const float ForcedOverride = RoundSettings ? RoundSettings->SphereBoxManifoldFriction.value_or(-1.f) :
        CVarIAmSpeedSphereBoxManifoldFrictionOverride.GetValueOnAnyThread();
    if (ForcedOverride >= 0.0f)
    {
        return ForcedOverride;
    }

    const float SphereOverride = Sphere.GetSphereBoxManifoldFrictionOverride();
    const float BoxOverride = Box.GetSphereBoxManifoldFrictionOverride();
    if (SphereOverride >= 0.0f || BoxOverride >= 0.0f)
    {
        return FMath::Max(SphereOverride, BoxOverride);
    }
    return ResolveSphereBoxFriction(Sphere, Box, FallbackFriction);
}

float USolidSubBody::ResolveSphereBoxRestitution(const USolidSubBody& Sphere, const USolidSubBody& Box, float FallbackRestitution)
{
    const auto* RoundSettings = Speed::FScopedExactRoundContactSettings::Current();
    const float ForcedOverride = RoundSettings ? RoundSettings->SphereBoxRestitution.value_or(-1.f) :
        CVarIAmSpeedSphereBoxRestitutionOverride.GetValueOnAnyThread();
    if (ForcedOverride >= 0.f)
    {
        return FMath::Clamp(ForcedOverride, 0.f, 1.f);
    }

    return ResolveSphereBoxGlobalRestitution(Sphere, Box, FallbackRestitution);
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
