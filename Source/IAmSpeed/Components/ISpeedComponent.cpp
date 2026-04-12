#include "ISpeedComponent.h"
#include "IAmSpeed/SubBodies/Solid/SolidSubBody.h"


void ISpeedComponent:: ApplyImpulse(const FVector& LinearImpulse, const FVector& WorldPoint, const USolidSubBody* SubBody)
{
	AddPhysImpulseAtPoint(LinearImpulse, WorldPoint, SubBody);
}

void ISpeedComponent::IntegrateKinematicsPrv(const float& SubDelta)
{
    SetKinematicState(GetKinematicState().Integrate(SubDelta));
}

void ISpeedComponent::IntegrateKinematics(const float& SubDelta)
{
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

FVector ISpeedComponent::GetPhysVelocityAtPoint(const FVector& Point) const
{
    // v + w x r
    const FVector R = Point - GetPhysLocation();
    return GetPhysVelocity() + FVector::CrossProduct(GetPhysAngularVelocity(), R);
}

const FVector& ISpeedComponent::GetPhysAngularVelocity() const
{
	return GetKinematicState().AngularVelocity;
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
    SKinematic K = GetKinematicState();
    K.Acceleration = NewAcceleration;
	SetKinematicState(K);
}

void ISpeedComponent::SetPhysAngularAcceleration(const FVector& NewAngularAcceleration)
{
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

void ISpeedComponent::AddPhysAcceleration(const FVector& DeltaAcceleration)
{
    SetPhysAcceleration(GetPhysAcceleration() + DeltaAcceleration);
}

void ISpeedComponent::AddPhysAngularAcceleration(const FVector& DeltaAngularAcceleration)
{
    SetPhysAngularAcceleration(GetPhysAngularAcceleration() + DeltaAngularAcceleration);
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
    const FVector DeltaV = Impulse / PhysMass;
    AddPhysVelocity(DeltaV);
    // delta w = I^-1 * (r x impulse)
    const FVector R = Point - GetPhysLocation();
	const FMatrix WorldInvInertiaTensor = SubBody? SubBody->ComputeWorldInvInertiaTensor() : ComputeWorldInvInertiaTensor();
    const FVector DeltaW = WorldInvInertiaTensor.TransformVector(FVector::CrossProduct(R, Impulse));
    AddPhysAngularVelocity(DeltaW);
}

void ISpeedComponent::AddPhysForceAtPoint(const FVector& Force, const FVector& WorldPoint, const USolidSubBody* SubBody)
{
    // F = m * a => a = F / m
    float PhysMass = GetPhysMass() > 0.f ? GetPhysMass() : 1.f; // avoid divide by zero
    const FVector DeltaA = Force / PhysMass;
    AddPhysAcceleration(DeltaA);
    // alpha = I^-1 * (r x F)
    const FVector R = WorldPoint - GetPhysLocation();
	const FMatrix WorldInvInertiaTensor = SubBody? SubBody->ComputeWorldInvInertiaTensor() : ComputeWorldInvInertiaTensor();
    const FVector DeltaAlpha = WorldInvInertiaTensor.TransformVector(FVector::CrossProduct(R, Force));
    AddPhysAngularAcceleration(DeltaAlpha);
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
