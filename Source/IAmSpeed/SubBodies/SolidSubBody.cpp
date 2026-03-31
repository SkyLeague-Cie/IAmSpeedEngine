// Fill out your copyright notice in the Description page of Project Settings.


#include "SolidSubBody.h"
#include "IAmSpeed/Components/ISpeedComponent.h"
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