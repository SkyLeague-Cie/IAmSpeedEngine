#include "ISpeedComponent.h"
#include "IAmSpeed/SubBodies/Solid/SolidSubBody.h"

namespace
{
	constexpr uint8 MaxConstraintPersistenceFrames = 1;
	constexpr float ConstraintNormalDotThreshold = 0.995f;
	constexpr float ConstraintPointThresholdCm = 10.0f;
	constexpr int32 ConstraintProjectionPasses = 4;
	constexpr float ConstraintPersistSignedDistCm = 5.0f;
	constexpr float ConstraintPersistSeparatingSpeed = 50.0f;
}

void ISpeedComponent:: ApplyImpulse(const FVector& LinearImpulse, const FVector& WorldPoint, const USolidSubBody* SubBody)
{
	AddPhysImpulseAtPoint(LinearImpulse, WorldPoint, SubBody);
}

void ISpeedComponent::IntegrateKinematicsPrv(const float& SubDelta)
{
    // The stored kinematic state is attached to the component origin. Account
    // for the centripetal acceleration of that origin around the physical COM.
    const FVector OriginToCOM = GetPhysCOM() - GetPhysLocation();
    if (!OriginToCOM.IsNearlyZero())
    {
        const FVector AngularVelocity = GetPhysAngularVelocity();
        SetPhysAcceleration(GetPhysAcceleration() - FVector::CrossProduct(AngularVelocity, FVector::CrossProduct(AngularVelocity, OriginToCOM)));
    }
    SetKinematicState(GetKinematicState().Integrate(SubDelta));
}

void ISpeedComponent::IntegrateKinematics(const float& SubDelta)
{
	if (IsFrozen())
		return;

    if (SubDelta <= 0.f)
		return;

	// Advance component kinematics only
	IntegrateKinematicsPrv(SubDelta);

	// enforce max speed limits after integrating kinematics to the time of impact
    SetPhysVelocity(GetPhysVelocity());
	SetPhysAngularVelocity(GetPhysAngularVelocity());

	// update sub-body kinematics to the time of impact
	UpdateSubBodiesKinematics();

	// update physics state after integrating kinematics to the time of impact (e.g update suspension traces for wheels, update hit info for hitboxes, etc.)
	PostIntegrateKinematics(SubDelta);
    // UE_LOG(LogTemp, Log, TEXT("[IntegrateKinematics] NumFrame = %d, SubDelta = %f. Kinematics = %s"), NumFrame(), SubDelta, *GetKinematicState().ToString());
}

void ISpeedComponent::UpdateSubBodiesKinematics()
{
    for (USSubBody* SubBody : GetSubBodies())
    {
        SubBody->UpdateKinematicsFromOwner(GetKinematicState());
    }
}

void ISpeedComponent::ResetForFrame(const float& Delta)
{
	for (int32 ConstraintIdx = PhysicalConstraints.Num() - 1; ConstraintIdx >= 0; --ConstraintIdx)
	{
		FPhysicalContactConstraint& Constraint = PhysicalConstraints[ConstraintIdx];

		if (!Constraint.bPersistent)
		{
			PhysicalConstraints.RemoveAtSwap(ConstraintIdx);
			continue;
		}

		Constraint.FramesSinceSeen++;
		if (Constraint.FramesSinceSeen > MaxConstraintPersistenceFrames ||
			!IsPhysicalConstraintStillRelevant(Constraint))
		{
			PhysicalConstraints.RemoveAtSwap(ConstraintIdx);
		}
	}

	for (USSubBody* SubBody : GetSubBodies())
	{
		SubBody->ResetForFrame(Delta);
	}
}

SComponentTOI ISpeedComponent::SweepTOISubBodies(const float& RemainingDelta, const float& LastSubDelta)
{
    SComponentTOI Best;
    Best.bHit = false;
    Best.TOI = RemainingDelta;

    const TArray<USSubBody*>& InSubBodies = GetSubBodies();

    for (USSubBody* Sweeper : InSubBodies)
    {
        if (!Sweeper) continue;

        float TOI = RemainingDelta;
        constexpr float MinSubDelta = 0.0f;
        // if (LastSubDelta > MinSubDelta)
        {
            if (!Sweeper->SweepTOI(RemainingDelta, TOI))
                continue;
        }
        /*else if (Sweeper->WillHit())
        {
			TOI = Sweeper->GetFutureTOI();
        }*/

        if (TOI >= Best.TOI)
            continue;

        const SHitResult& H = Sweeper->GetFutureHit();
        USSubBody* OtherSB = H.SubBody.Get();
        USSubBody* Resolver = USSubBody::PickResolver(Sweeper, OtherSB);

        SHitResult HR = H;

        if (Resolver && Resolver != Sweeper)
        {
            HR.ImpactNormal *= -1.f;
        }

        Best.bHit = true;
        Best.TOI = TOI;
        Best.Resolver = Resolver;
        Best.Hit = HR;
        Best.PairKey = USSubBody::MakePairKey(Resolver ? Resolver : Sweeper, HR.Component.Get());
    }

    return Best;
}

void ISpeedComponent::AddExternalSubBodies(const TArray<USSubBody*>& ExtSubBodies)
{
    const TArray<USSubBody*>& InSubBodies = GetSubBodies();
    for (USSubBody* SubBody : InSubBodies)
    {
        if (SubBody)
        {
            SubBody->AddExternalSubBodies(ExtSubBodies);
        }
    }
}

void ISpeedComponent::RemoveExternalSubBodies(const TArray<USSubBody*>& ExtSubBodies)
{
    const TArray<USSubBody*>& InSubBodies = GetSubBodies();
    for (USSubBody* SubBody : InSubBodies)
    {
        if (SubBody)
        {
            SubBody->RemoveExternalSubBodies(ExtSubBodies);
        }
    }
}

void ISpeedComponent::PostPhysicsUpdate(const float& delta)
{
	// reset accelerations after physics update so that they can be set again during the next tick
	SetPhysAcceleration(FVector::ZeroVector);
	SetPhysAngularAcceleration(FVector::ZeroVector);

	const TArray<USSubBody*>& InSubBodies = GetSubBodies();
    for (USSubBody* SubBody : InSubBodies)
    {
        if (SubBody)
        {
            SubBody->PostPhysicsUpdate();
        }
	}

    PostPhysicsUpdatePrv(delta);
}

// ===================== Kinematics getters and setters =================
const FVector& ISpeedComponent::GetPhysLocation() const
{
    return GetKinematicState().Location;
}

void ISpeedComponent::SetPhysLocation(const FVector& NewLocation)
{
    SKinematic K = GetKinematicState();
    K.Location = NewLocation;
    SetKinematicState(K);
}

const FQuat& ISpeedComponent::GetPhysRotation() const
{
    return GetKinematicState().Rotation;
}

void ISpeedComponent::SetPhysRotation(const FQuat& NewRotation)
{
    SKinematic K = GetKinematicState();
    K.Rotation = NewRotation;
	SetKinematicState(K);
}

const FVector& ISpeedComponent::GetPhysVelocity() const
{
    return GetKinematicState().Velocity;
}

void ISpeedComponent::SetPhysVelocity(const FVector& NewVelocity)
{
	if (IsFrozen())
	{
		return;
	}
	SetPhysVelocityRaw(NewVelocity.GetClampedToMaxSize(GetPhysMaxSpeed()));
}

FVector ISpeedComponent::GetPhysVelocityAtPoint(const FVector& Point) const
{
    // v + w x r
    const FVector R = Point - GetPhysLocation();
    return GetPhysVelocity() + FVector::CrossProduct(GetPhysAngularVelocity(), R);
}

FVector ISpeedComponent::GetPhysCOMVelocity() const
{
	return GetPhysVelocityAtPoint(GetPhysCOM());
}

const FVector& ISpeedComponent::GetPhysAngularVelocity() const
{
	return GetKinematicState().AngularVelocity;
}

void ISpeedComponent::SetPhysAngularVelocity(const FVector& NewAngularVelocity)
{
	if (IsFrozen())
	{
		return;
	}
	const FVector ClampedAngularVelocity = NewAngularVelocity.GetClampedToMaxSize(GetPhysMaxAngularSpeed());
	const FVector OriginToCOM = GetPhysCOM() - GetPhysLocation();
	const FVector COMVelocity = GetPhysVelocity() + FVector::CrossProduct(GetPhysAngularVelocity(), OriginToCOM);
	SetPhysAngularVelocityRaw(ClampedAngularVelocity);
	SetPhysVelocityRaw(COMVelocity - FVector::CrossProduct(ClampedAngularVelocity, OriginToCOM));
}

void ISpeedComponent::SetPhysVelocityAtPoint(const FVector& NewVelocity, const FVector& Point)
{
    // v + w x r
    const FVector R = Point - GetPhysLocation();
    const FVector NewAngularVelocity = FVector::CrossProduct(R, NewVelocity - GetPhysVelocity()).GetSafeNormal() * GetPhysAngularVelocity().Size();
    SetPhysAngularVelocity(NewAngularVelocity);
    SetPhysVelocity(NewVelocity);
}

const FVector& ISpeedComponent::GetPhysAcceleration() const
{
    return GetKinematicState().Acceleration;
}

const FVector& ISpeedComponent::GetPhysAngularAcceleration() const
{
	return GetKinematicState().AngularAcceleration;
}

void ISpeedComponent::SetPhysAcceleration(const FVector& NewAcceleration)
{
	if (IsFrozen())
	{
		return;
	}

    SKinematic K = GetKinematicState();
    K.Acceleration = NewAcceleration;
	SetKinematicState(K);
}

void ISpeedComponent::SetPhysAngularAcceleration(const FVector& NewAngularAcceleration)
{
	if (IsFrozen())
	{
		return;
	}

    SKinematic K = GetKinematicState();
    K.AngularAcceleration = NewAngularAcceleration;
	SetKinematicState(K);
}

FVector ISpeedComponent::GetPhysAccelerationAtPoint(const FVector& Point) const
{
    // a + alpha x r + w x (w x r)
    const FVector R = Point - GetPhysLocation();
    return GetPhysAcceleration() + FVector::CrossProduct(GetPhysAngularAcceleration(), R) + FVector::CrossProduct(GetPhysAngularVelocity(), FVector::CrossProduct(GetPhysAngularVelocity(), R));
}

void ISpeedComponent::AddPhysVelocity(const FVector& DeltaVelocity)
{
	FVector ProjectedDeltaVelocity = DeltaVelocity;
	ProjectLinearDeltaAgainstConstraints(ProjectedDeltaVelocity);
	SetPhysVelocity(GetPhysVelocity() + ProjectedDeltaVelocity);
}

FVector ISpeedComponent::AddPhysLocation(const FVector& DeltaLocation)
{
	FVector ProjectedDeltaLocation = DeltaLocation;
	ProjectLinearDeltaAgainstConstraints(ProjectedDeltaLocation);
	if (!ProjectedDeltaLocation.IsNearlyZero())
	{
		SetPhysLocation(GetPhysLocation() + ProjectedDeltaLocation);
	}
	return ProjectedDeltaLocation;
}

FQuat ISpeedComponent::AddPhysRotation(const FQuat& DeltaRotation)
{
	FQuat NormalizedDeltaRotation = DeltaRotation.GetNormalized();
	if (NormalizedDeltaRotation.Equals(FQuat::Identity))
	{
		return FQuat::Identity;
	}

	FVector Axis = FVector::ZeroVector;
	float Angle = 0.f;
	NormalizedDeltaRotation.ToAxisAndAngle(Axis, Angle);
	if (!Axis.Normalize())
	{
		return FQuat::Identity;
	}

	if (Angle > PI)
	{
		Angle -= 2.f * PI;
	}

	FVector DeltaAngular = Axis * Angle;
	ProjectAngularDeltaAgainstConstraints(DeltaAngular);

	const float ProjectedAngle = DeltaAngular.Size();
	if (ProjectedAngle <= KINDA_SMALL_NUMBER)
	{
		return FQuat::Identity;
	}

	const FQuat AppliedDeltaRotation(DeltaAngular / ProjectedAngle, ProjectedAngle);
	SetPhysRotation((GetPhysRotation() * AppliedDeltaRotation).GetNormalized());
	return AppliedDeltaRotation;
}
void ISpeedComponent::AddPhysAngularVelocity(const FVector& DeltaAngularVelocity)
{
	FVector ProjectedDeltaAngularVelocity = DeltaAngularVelocity;
	ProjectAngularDeltaAgainstConstraints(ProjectedDeltaAngularVelocity);
	SetPhysAngularVelocity(GetPhysAngularVelocity() + ProjectedDeltaAngularVelocity);
}

void ISpeedComponent::AddPhysAcceleration(const FVector& DeltaAcceleration)
{
	FVector ProjectedDeltaAcceleration = DeltaAcceleration;
	ProjectLinearDeltaAgainstConstraints(ProjectedDeltaAcceleration);
    SetPhysAcceleration(GetPhysAcceleration() + ProjectedDeltaAcceleration);
}

void ISpeedComponent::AddPhysAngularAcceleration(const FVector& DeltaAngularAcceleration)
{
	FVector ProjectedDeltaAngularAcceleration = DeltaAngularAcceleration;
	ProjectAngularDeltaAgainstConstraints(ProjectedDeltaAngularAcceleration);
    const FVector OriginToCOM = GetPhysCOM() - GetPhysLocation();
    AddPhysAcceleration(-FVector::CrossProduct(ProjectedDeltaAngularAcceleration, OriginToCOM));
    SetPhysAngularAcceleration(GetPhysAngularAcceleration() + ProjectedDeltaAngularAcceleration);
}

void ISpeedComponent::AddPhysAngularAccelerationLocal(const FVector& LocalAngularAccel)
{
    // alpha_world = R * alpha_local
    const FQuat Rot = GetPhysRotation();
    const FVector DeltaAngularAccelWorld = Rot.RotateVector(LocalAngularAccel);
	AddPhysAngularAcceleration(DeltaAngularAccelWorld);
}

void ISpeedComponent::AddPhysImpulseAtPoint(const FVector& Impulse, const FVector& Point, const USolidSubBody* SubBody)
{
    // delta v = impulse / mass
    float PhysMass = GetPhysMass() > 0.f ? GetPhysMass() : 1.f; // avoid divide by zero
    FVector DeltaV = Impulse / PhysMass;
    // delta w = I^-1 * (r x impulse)
	const FVector COM = GetPhysCOM();
    const FVector R = Point - COM;
	const FMatrix WorldInvInertiaTensor = SubBody? SubBody->ComputeWorldInvInertiaTensor() : ComputeWorldInvInertiaTensor();
    FVector DeltaW = WorldInvInertiaTensor.TransformVector(FVector::CrossProduct(R, Impulse));

	ProjectPointDeltaAgainstConstraints(DeltaV, DeltaW);
	const FVector OriginToCOM = COM - GetPhysLocation();
    SetPhysVelocityRaw(GetPhysVelocity() + DeltaV - FVector::CrossProduct(DeltaW, OriginToCOM));
    SetPhysAngularVelocityRaw((GetPhysAngularVelocity() + DeltaW).GetClampedToMaxSize(GetPhysMaxAngularSpeed()));
}

void ISpeedComponent::AddPhysForceAtPoint(const FVector& Force, const FVector& WorldPoint, const USolidSubBody* SubBody)
{
    // F = m * a => a = F / m
    float PhysMass = GetPhysMass() > 0.f ? GetPhysMass() : 1.f; // avoid divide by zero
    FVector DeltaA = Force / PhysMass;
    // alpha = I^-1 * (r x F)
	const FVector COM = GetPhysCOM();
    const FVector R = WorldPoint - COM;
	const FMatrix WorldInvInertiaTensor = SubBody? SubBody->ComputeWorldInvInertiaTensor() : ComputeWorldInvInertiaTensor();
    FVector DeltaAlpha = WorldInvInertiaTensor.TransformVector(FVector::CrossProduct(R, Force));

	ProjectPointDeltaAgainstConstraints(DeltaA, DeltaAlpha);
	const FVector OriginToCOM = COM - GetPhysLocation();
    SetPhysAcceleration(GetPhysAcceleration() + DeltaA - FVector::CrossProduct(DeltaAlpha, OriginToCOM));
    SetPhysAngularAcceleration(GetPhysAngularAcceleration() + DeltaAlpha);
}

void ISpeedComponent::SetPhysVelocityRaw(const FVector& NewVelocity)
{
    SKinematic K = GetKinematicState();
    K.Velocity = NewVelocity;
	SetKinematicState(K);
}

void ISpeedComponent::SetPhysAngularVelocityRaw(const FVector& NewAngularVelocity)
{
    SKinematic K = GetKinematicState();
    K.AngularVelocity = NewAngularVelocity;
	SetKinematicState(K);
}

void ISpeedComponent::RegisterPhysicalConstraint(const FPhysicalContactConstraint& Constraint)
{
	if (!Constraint.IsValid())
	{
		return;
	}

	FPhysicalContactConstraint NewConstraint = Constraint;
	NewConstraint.Normal = NewConstraint.Normal.GetSafeNormal();
	NewConstraint.FramesSinceSeen = 0;
	if (NewConstraint.Normal.IsNearlyZero())
	{
		return;
	}

	for (FPhysicalContactConstraint& ExistingConstraint : PhysicalConstraints)
	{
		if (!AreSimilarPhysicalConstraints(ExistingConstraint, NewConstraint))
		{
			continue;
		}

		if (NewConstraint.PenetrationDepth >= ExistingConstraint.PenetrationDepth)
		{
			ExistingConstraint = NewConstraint;
		}
		else
		{
			ExistingConstraint.FramesSinceSeen = 0;
		}
		return;
	}

	PhysicalConstraints.Add(NewConstraint);
}

void ISpeedComponent::ClearPhysicalConstraints()
{
	PhysicalConstraints.Reset();
}

bool ISpeedComponent::IsPhysicalConstraintStillRelevant(const FPhysicalContactConstraint& Constraint) const
{
	if (!Constraint.IsValid())
	{
		return false;
	}

	if (Constraint.FramesSinceSeen == 0)
	{
		return true;
	}

	const FVector Normal = Constraint.Normal.GetSafeNormal();
	if (Normal.IsNearlyZero())
	{
		return false;
	}

	const float SignedDist = FVector::DotProduct(GetPhysCOM() - Constraint.ContactPoint, Normal);
	if (SignedDist > ConstraintPersistSignedDistCm)
	{
		return false;
	}

	const float PointSpeedAlongNormal = FVector::DotProduct(GetPhysVelocityAtPoint(Constraint.ContactPoint), Normal);
	return PointSpeedAlongNormal <= ConstraintPersistSeparatingSpeed;
}

void ISpeedComponent::ProjectLinearDeltaAgainstConstraints(FVector& DeltaLinear) const
{
	if (DeltaLinear.IsNearlyZero())
	{
		return;
	}

	for (int32 PassIdx = 0; PassIdx < ConstraintProjectionPasses; ++PassIdx)
	{
		bool bChanged = false;
		for (const FPhysicalContactConstraint& Constraint : PhysicalConstraints)
		{
			if (!IsPhysicalConstraintStillRelevant(Constraint))
			{
				continue;
			}

			const FVector Normal = Constraint.Normal.GetSafeNormal();
			const float IntoSurface = FVector::DotProduct(DeltaLinear, Normal);
			if (IntoSurface < 0.0f)
			{
				DeltaLinear -= IntoSurface * Normal;
				bChanged = true;
			}
		}

		if (!bChanged)
		{
			break;
		}
	}
}

void ISpeedComponent::ProjectAngularDeltaAgainstConstraints(FVector& DeltaAngular) const
{
	if (DeltaAngular.IsNearlyZero())
	{
		return;
	}

	const FVector COM = GetPhysCOM();
	for (int32 PassIdx = 0; PassIdx < ConstraintProjectionPasses; ++PassIdx)
	{
		bool bChanged = false;
		for (const FPhysicalContactConstraint& Constraint : PhysicalConstraints)
		{
			if (!IsPhysicalConstraintStillRelevant(Constraint))
			{
				continue;
			}

			const FVector Normal = Constraint.Normal.GetSafeNormal();
			const FVector LeverArmAxis = FVector::CrossProduct(Constraint.ContactPoint - COM, Normal);
			const float AxisSizeSquared = LeverArmAxis.SizeSquared();
			if (AxisSizeSquared <= KINDA_SMALL_NUMBER)
			{
				continue;
			}

			const float IntoSurface = FVector::DotProduct(DeltaAngular, LeverArmAxis);
			if (IntoSurface < 0.0f)
			{
				DeltaAngular -= (IntoSurface / AxisSizeSquared) * LeverArmAxis;
				bChanged = true;
			}
		}

		if (!bChanged)
		{
			break;
		}
	}
}

void ISpeedComponent::ProjectPointDeltaAgainstConstraints(FVector& DeltaLinear, FVector& DeltaAngular) const
{
	if (DeltaLinear.IsNearlyZero() && DeltaAngular.IsNearlyZero())
	{
		return;
	}

	const FVector COM = GetPhysCOM();

	for (int32 PassIdx = 0; PassIdx < ConstraintProjectionPasses; ++PassIdx)
	{
		bool bChanged = false;

		for (const FPhysicalContactConstraint& Constraint : PhysicalConstraints)
		{
			if (!IsPhysicalConstraintStillRelevant(Constraint))
			{
				continue;
			}

			const FVector N = Constraint.Normal.GetSafeNormal();
			if (N.IsNearlyZero())
			{
				continue;
			}

			const FVector R = Constraint.ContactPoint - COM;
			const FVector DeltaAtPoint =
				DeltaLinear + FVector::CrossProduct(DeltaAngular, R);

			const float IntoSurface = FVector::DotProduct(DeltaAtPoint, N);
			if (IntoSurface >= 0.f)
			{
				continue;
			}

			const FVector LeverArmAxis = FVector::CrossProduct(R, N);
			const float AxisSizeSq = LeverArmAxis.SizeSquared();

			// Prefer angular correction, because this is a point constraint.
			if (AxisSizeSq > KINDA_SMALL_NUMBER && !DeltaAngular.IsNearlyZero())
			{
				DeltaAngular -= (IntoSurface / AxisSizeSq) * LeverArmAxis;
				bChanged = true;
			}
			else if (!DeltaLinear.IsNearlyZero())
			{
				DeltaLinear -= IntoSurface * N;
				bChanged = true;
			}
		}

		if (!bChanged)
		{
			break;
		}
	}
}

bool ISpeedComponent::AreSimilarPhysicalConstraints(const FPhysicalContactConstraint& A, const FPhysicalContactConstraint& B)
{
	if (!A.OtherComponent.IsValid() || !B.OtherComponent.IsValid())
	{
		return false;
	}

	if (A.OtherComponent != B.OtherComponent)
	{
		return false;
	}

	const FVector ANormal = A.Normal.GetSafeNormal();
	const FVector BNormal = B.Normal.GetSafeNormal();
	if (ANormal.IsNearlyZero() || BNormal.IsNearlyZero())
	{
		return false;
	}

	if (FVector::DotProduct(ANormal, BNormal) < ConstraintNormalDotThreshold)
	{
		return false;
	}

	return FVector::DistSquared(A.ContactPoint, B.ContactPoint) <= FMath::Square(ConstraintPointThresholdCm);
}

void ISpeedComponent::FreezeMovement()
{
	SetPhysVelocityRaw(FVector::ZeroVector);
	SetPhysAngularVelocityRaw(FVector::ZeroVector);
	SetPhysAcceleration(FVector::ZeroVector);
	SetPhysAngularAcceleration(FVector::ZeroVector);

	SetIsFrozen(true);
}

void ISpeedComponent::UnFreezeMovement()
{
	SetIsFrozen(false);
}
