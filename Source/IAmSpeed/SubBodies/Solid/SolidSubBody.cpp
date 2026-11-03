// Fill out your copyright notice in the Description page of Project Settings.


#include "SolidSubBody.h"
#include "IAmSpeed/Netcode/client-prediction/PrivateNativeStorageMap.h"
#include "IAmSpeed/World/Simulation/Internal/SolidPropertiesAccess.h"
#include "IAmSpeed/World/Simulation/Internal/ComponentContactWrite.h"
#include "IAmSpeed/World/Simulation/Internal/ComponentKinematicRead.h"
#include "IAmSpeed/World/Simulation/Internal/BodyKinematicRead.h"
#include "IAmSpeed/World/Simulation/RoadSettings/NativeRoadSettingsRead.h"
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
    TEXT("Forces friction for sphere/box contacts when >= 0. Values above 1 are valid. Negative values use the normal friction mix."));

static TAutoConsoleVariable<float> CVarIAmSpeedSphereBoxManifoldFrictionOverride(
    TEXT("p.IAmSpeed.Collision.SphereBoxManifoldFrictionOverride"),
    -1.0f,
    TEXT("Forces friction only for persistent sphere/box manifolds when >= 0. Negative values use subbody manifold overrides or ordinary sphere/box friction."));

float Speed::NativeKinematics::FSolidRestitutionPolicy::Resolve(
        const USolidSubBody& Sphere,
        const USolidSubBody& Box,
        const float FallbackRestitution)
    {
        const float SphereOverride = Speed::NativeKinematics::FSolidPropertiesAccess::GetSphereBoxRestitutionOverride(Sphere);
        if (SphereOverride >= 0.f)
        {
            return FMath::Clamp(SphereOverride, 0.f, 1.f);
        }

        const float BoxOverride = Speed::NativeKinematics::FSolidPropertiesAccess::GetSphereBoxRestitutionOverride(Box);
        if (BoxOverride >= 0.f)
        {
            return FMath::Clamp(BoxOverride, 0.f, 1.f);
        }

        return FMath::Clamp(FallbackRestitution, 0.f, 1.f);
    }

void USolidSubBody::Initialize(ISpeedComponent* InParentComponent)
{
	Super::Initialize(InParentComponent);

	if (InParentComponent)
	{
		SubBodyConfig Config = InParentComponent->GetSubBodyConfig(*this);
        if (Config.bValid)
        {
            MassSolidStorage() = Config.Mass;
            RestitutionSolidStorage() = Config.Restitution;
            StaticFrictionSolidStorage() = Config.StaticFriction;
            DynamicFrictionSolidStorage() = Config.DynamicFriction;
            ImpactThresholdSolidStorage() = Config.ImpactThreshold;
            bIsMainSubBodySolidStorage() = Config.bIsMainSubBody;
            EnableFakePhysicsSolidStorage() = Config.bEnableFakePhysics;
            HitDampingSolidStorage() = FMath::Clamp(Config.HitDamping, 0.f, 1.f);
        }
		InvInertiaLocalSolidStorage() = InitInvInertiaTensor();
	}
}

void USolidSubBody::ApplyImpulse(const FVector& LinearImpulse, const FVector& WorldPoint)
{
	if (ParentComponentBodyDomainStorage())
	{
		ParentComponentBodyDomainStorage()->ApplyImpulse(LinearImpulse, WorldPoint, this);
	}
}

void USolidSubBody::SetSphereBoxRestitutionOverride(float InRestitution)
{
    PublicSolid_SphereBoxRestitutionOverrideStorage() = InRestitution < 0.f ? -1.f : FMath::Clamp(InRestitution, 0.f, 1.f);
}

void USolidSubBody::RegisterCurrentHitAsConstraint()
{
    if (!CurrentHitStorage().bBlockingHit || !CurrentHitStorage().Component.IsValid())
    {
        return;
    }

    RegisterContactAsConstraint(
        CurrentHitStorage().ImpactPoint,
        CurrentHitStorage().ImpactNormal,
        CurrentHitStorage().Component,
        Cast<USolidSubBody>(CurrentHitStorage().SubBody.Get()),
        CurrentHitStorage().PenetrationDepth,
        CurrentHitStorage().TOI,
        false
    );
}

void USolidSubBody::RegisterContactAsConstraint(
    const FVector& P,const FVector& N,UPrimitiveComponent* Other,USolidSubBody* Solid,
    float Depth,float Time,bool Persistent)
{RegisterContactWithReference(P,N,Other,Solid,Depth,Time,Persistent,nullptr);}
void USolidSubBody::RegisterContactAsConstraint(
    const FVector& P,const FVector& N,const TWeakObjectPtr<UPrimitiveComponent>& Other,
    USolidSubBody* Solid,float Depth,float Time,bool Persistent)
{
    const auto Snapshot=Other; // immutable local copy spans the complete synchronous call
    RegisterContactWithReference(P,N,Snapshot.Get(),Solid,Depth,Time,Persistent,&Snapshot);
}

void USolidSubBody::RegisterContactWithReference(
    const FVector& ContactPointWS,const FVector& NormalWS,UPrimitiveComponent* OtherComponent,
    USolidSubBody* OtherSubBody,float PenetrationDepth,float TOI,bool bPersistent,
    const TWeakObjectPtr<UPrimitiveComponent>* ExistingReference)
{
    if (!ParentComponentBodyDomainStorage() || !OtherComponent)
    {
        return;
    }

    if (!OtherSubBody)
    {
        OtherSubBody = Cast<USolidSubBody>(OtherComponent);
    }

    const bool bStaticConstraint = OtherComponent->GetCollisionObjectType() == ECC_WorldStatic;
    const bool bDynamicSolidConstraint = OtherSubBody && OtherSubBody->ParentComponentBodyDomainStorage();
    if (!bStaticConstraint && !bDynamicSolidConstraint)
    {
        return;
    }

    const bool bRegisterThisConstraint =
        bStaticConstraint || (bDynamicSolidConstraint && bEnableConstraintOnSubBodyHitSolidStorage());
    const bool bRegisterOtherConstraint =
        bDynamicSolidConstraint &&
        OtherSubBody->bEnableConstraintOnSubBodyHitSolidStorage() &&
        OtherSubBody->ParentComponentBodyDomainStorage() != ParentComponentBodyDomainStorage();
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
    if(ExistingReference)Constraint.OtherComponent = *ExistingReference;
    else Constraint.OtherComponent = OtherComponent;
    Constraint.SourceSubBody = GetPhysicsWeakIdentity();
    Constraint.PenetrationDepth = PenetrationDepth;
    Constraint.TOI = TOI;
    Constraint.PairKey = USSubBody::MakePairKey(this, OtherComponent);
    Constraint.bPersistent = bStaticConstraint && bPersistent;

    if (bRegisterThisConstraint)
    {
        Speed::NativeKinematics::FComponentWrite::RegisterConstraint(*ParentComponentBodyDomainStorage(),Constraint);
    }

    if (bRegisterOtherConstraint)
    {
        FPhysicalContactConstraint OtherConstraint;
        OtherConstraint.Normal = -N;
        OtherConstraint.ContactPoint = ContactPointWS;
        OtherConstraint.OtherComponent = GetPhysicsWeakIdentity();
        OtherConstraint.SourceSubBody = OtherSubBody->GetPhysicsWeakIdentity();
        OtherConstraint.PenetrationDepth = PenetrationDepth;
        OtherConstraint.TOI = TOI;
        OtherConstraint.PairKey = USSubBody::MakePairKey(OtherSubBody, this);
        OtherConstraint.bPersistent = false;
        Speed::NativeKinematics::FComponentWrite::RegisterConstraint(*OtherSubBody->ParentComponentBodyDomainStorage(),OtherConstraint);

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
    const TArray<FVector>& Points,const FVector& N,UPrimitiveComponent* Other,
    float Depth,float Time,bool Persistent,int32 Max)
{RegisterManifoldWithReference(Points,N,Other,Depth,Time,Persistent,Max,nullptr);}
void USolidSubBody::RegisterContactManifoldAsConstraints(
    const TArray<FVector>& Points,const FVector& N,const TWeakObjectPtr<UPrimitiveComponent>& Other,
    float Depth,float Time,bool Persistent,int32 Max)
{
    const auto Snapshot=Other; // never retain a pointer to a caller's mutable hit cell
    RegisterManifoldWithReference(Points,N,Snapshot.Get(),Depth,Time,Persistent,Max,&Snapshot);
}

void USolidSubBody::RegisterManifoldWithReference(
    const TArray<FVector>& ContactPointsWS,const FVector& NormalWS,UPrimitiveComponent* OtherComponent,
    float PenetrationDepth,float TOI,bool bPersistent,int32 MaxContacts,
    const TWeakObjectPtr<UPrimitiveComponent>* ExistingReference)
{
    if (!ParentComponentBodyDomainStorage() || !OtherComponent || ContactPointsWS.Num() == 0)
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
            RegisterContactWithReference(P, N, OtherComponent, nullptr, PenetrationDepth, TOI, bPersistent,ExistingReference);
        }
        return;
    }

    // Cas rare : trop de points. On garde des points répartis autour du COM,
    // ce qui donne de meilleurs bras de levier pour la projection angulaire.
    const FVector COM = Speed::NativeKinematics::FBodyRead::Body(*this).Location                     ;

    TArray<FVector> Sorted = ContactPointsWS;
    Sorted.Sort([&COM](const FVector& A, const FVector& B)
        {
            return FVector::DistSquared(A, COM) > FVector::DistSquared(B, COM);
        });

    const int32 NumToRegister = FMath::Min(MaxContacts, Sorted.Num());
    for (int32 i = 0; i < NumToRegister; ++i)
    {
        RegisterContactWithReference(Sorted[i], N, OtherComponent, nullptr, PenetrationDepth, TOI, bPersistent,ExistingReference);
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
    const float ForcedOverride = Speed::RoadSettings::ReadOriginalOrLegacy<24>([](){ return CVarIAmSpeedSphereBoxFrictionOverride.GetValueOnAnyThread(); });
    if (ForcedOverride >= 0.0f)
    {
        return ForcedOverride;
    }
    const float SphereOverride = Speed::NativeKinematics::FSolidPropertiesAccess::GetSphereBoxFrictionOverride(Sphere);
    const float BoxOverride = Speed::NativeKinematics::FSolidPropertiesAccess::GetSphereBoxFrictionOverride(Box);
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
    const float ForcedOverride =
        Speed::RoadSettings::ReadOriginalOrLegacy<25>([](){ return CVarIAmSpeedSphereBoxManifoldFrictionOverride.GetValueOnAnyThread(); });
    if (ForcedOverride >= 0.0f)
    {
        return ForcedOverride;
    }

    const float SphereOverride = Speed::NativeKinematics::FSolidPropertiesAccess::GetSphereBoxManifoldFrictionOverride(Sphere);
    const float BoxOverride = Speed::NativeKinematics::FSolidPropertiesAccess::GetSphereBoxManifoldFrictionOverride(Box);
    if (SphereOverride >= 0.0f || BoxOverride >= 0.0f)
    {
        return FMath::Max(SphereOverride, BoxOverride);
    }
    return ResolveSphereBoxFriction(Sphere, Box, FallbackFriction);
}

float USolidSubBody::ResolveSphereBoxRestitution(const USolidSubBody& Sphere, const USolidSubBody& Box, float FallbackRestitution)
{
    const float ForcedOverride = Speed::RoadSettings::ReadOriginalOrLegacy<29>([](){ return CVarIAmSpeedSphereBoxRestitutionOverride.GetValueOnAnyThread(); });
    if (ForcedOverride >= 0.f)
    {
        return FMath::Clamp(ForcedOverride, 0.f, 1.f);
    }

    return Speed::NativeKinematics::FSolidRestitutionPolicy::Resolve(Sphere, Box, FallbackRestitution);
}

FVector USolidSubBody::GetVelocityAtPoint(const FVector& Point) const
{
    if (ParentComponentBodyDomainStorage())
    {
        return Speed::NativeKinematics::FComponentRead::Of(*ParentComponentBodyDomainStorage()).VelocityAtPoint(Point);
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

        ThisComp.SetPhysLocation(Speed::NativeKinematics::FComponentRead::Of(ThisComp).Location() + corr * wA);
        if (OtherComp)
            OtherComp->SetPhysLocation(Speed::NativeKinematics::FComponentRead::Of(*OtherComp).Location() - corr * wB);
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

void USolidSubBody::ApplyNativeFakePhysicsOn(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,USolidSubBody& OtherSubBody,const SHitResult& Hit,const float& DeltaTime,const FFakePhysicsImpactContext* ImpactContext)
{
    (void)Operation;(void)OtherSubBody;(void)Hit;(void)DeltaTime;(void)ImpactContext;
    // No public virtual fallback on an unsupported native subtype.
    throw Speed::Netcode::Prediction::FPrivateNativeStorageUnavailable{};
}

FMatrix USolidSubBody::ComputeNativeWorldInvInertiaTensor(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation) const
{
    (void)Operation;
    // Unsupported native subtype refuses BEFORE old Storage, virtual public
    // getters or any native state mutation. Missing is never a zero matrix.
    throw Speed::Netcode::Prediction::FPrivateNativeStorageUnavailable{};
}

FMatrix USolidSubBody::ComputeWorldInvInertiaTensor() const
{
    if (ParentComponentBodyDomainStorage())
    {
        return ParentComponentBodyDomainStorage()->ComputeWorldInvInertiaTensorOfSubBody(*this);
    }
    return FMatrix::Identity;
}
