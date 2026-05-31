// Fill out your copyright notice in the Description page of Project Settings.


#include "SolidSubBody.h"
#include "IAmSpeed/Components/ISpeedComponent.h"
#include "IAmSpeed/Base/PhysicalContactConstraint.h"
#include "Configs/SubBodyConfig.h"

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

    ParentComponent->RegisterPhysicalConstraint(Constraint);

    if (bDynamicSolidConstraint && OtherSubBody->ParentComponent != ParentComponent)
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
