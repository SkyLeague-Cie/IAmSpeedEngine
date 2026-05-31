#include "ISpeedWheeledComponent.h"
#include "IAmSpeed/SubBodies/Solid/SWheelSubBody.h"

void ISpeedWheeledComponent::ResolveGroupedWheelGroundContacts(const float& delta)
{
	TArray<SWheelGroundContact>& PendingWheelGroundContacts = GetPendingWheelContacts();
	if (PendingWheelGroundContacts.Num() == 0) return;

	const float dt = FMath::Max(delta, 1e-6f);

	constexpr float NormalVelTimeConstant = 0.0075f; // sec (greater values => car falls faster into the ground) -> be careful with greater values than 0.075, car can be blocked in convex surfaces
	const float gamma = 1.f - FMath::Exp(-dt / NormalVelTimeConstant);

	// --- Deadzones ---
	constexpr float VNDeadzone = 0.001f;        // cm/s : ignore micro-jitter
	constexpr float JDeadzone = 0.5f;        // impulse units (same as your AddImpulseAtLocation expects)
	// (smaller are those, more network stability)

	// softening (CFM-ish)
	constexpr float Softness = 0.0008f;

	// Snapshot
	const FVector V0 = GetPhysVelocity();
	const FVector W0 = GetPhysAngularVelocity();

	for (const SWheelGroundContact& C : PendingWheelGroundContacts)
	{
		if (C.InvMassEff <= SMALL_NUMBER) continue;

		const FVector N = C.Normal.GetSafeNormal();

		const FVector vContact = V0 + FVector::CrossProduct(W0, C.r);
		const float vN = FVector::DotProduct(vContact, N);

		// only if meaningfully moving into the surface
		if (vN >= -VNDeadzone)
			continue;

		const float denom = C.InvMassEff + Softness;
		if (denom <= SMALL_NUMBER) continue;

		float jn = -gamma * vN / denom;
		if (jn <= 0.f) continue;

		const FVector Impulse = jn * N;

		// ignore tiny impulses (stops “buzzing” at rest)
		if (Impulse.SizeSquared() < (JDeadzone * JDeadzone))
			continue;

		AddPhysImpulseAtPoint(Impulse, C.WorldPos);
	}

	PendingWheelGroundContacts.Reset();
}

FVector ISpeedWheeledComponent::GetNormalFromWheels() const
{
	FVector Normal = FVector::ZeroVector;
	const auto& WheelSubBodies = GetWheelSubBodies();
	for (const auto& Wheel : WheelSubBodies)
	{
		if (!Wheel) continue;
		Normal += Wheel->GetHitContactNormal();
	}
	if (Normal.IsZero())
	{
		return Normal;
	}
	return QuantizeUnitNormal(Normal);
}

bool ISpeedWheeledComponent::IsOnTheGround() const
{
	const auto& WheelSubBodies = GetWheelSubBodies();
	for (const auto& Wheel : WheelSubBodies)
	{
		if (!Wheel) continue;
		if (!Wheel->IsOnGround()) return false;
	}
	return true;
}

bool ISpeedWheeledComponent::OneWheelOnGround() const
{
	const auto& WheelSubBodies = GetWheelSubBodies();
	for (const auto& Wheel : WheelSubBodies)
	{
		if (!Wheel) continue;
		if (Wheel->IsOnGround()) return true;
	}
	return false;
}

bool ISpeedWheeledComponent::NoWheelOnGround() const
{
	return !OneWheelOnGround();
}

bool ISpeedWheeledComponent::WheelIdxIsOnGround(const int32& WheelIdx) const
{
	const auto& WheelSubBodies = GetWheelSubBodies();
	if (WheelIdx < 0 || WheelIdx >= WheelSubBodies.Num()) return false;
	return WheelSubBodies[WheelIdx] && WheelSubBodies[WheelIdx]->IsOnGround();
}

void ISpeedWheeledComponent::PostIntegrateKinematics(const float& delta)
{
	for (auto& Wheel : GetWheelSubBodies())
	{
		if (Wheel)
		{
			Wheel->SweepSuspension(delta);
		}
	}
}

FVector ISpeedWheeledComponent::QuantizeUnitNormal(const FVector& n, float q)
{
	FVector nCopy = n;
	if (!nCopy.Normalize())
	{
		return FVector::UpVector;
	}
	nCopy.X = FMath::RoundToFloat(nCopy.X / q) * q;
	nCopy.Y = FMath::RoundToFloat(nCopy.Y / q) * q;
	nCopy.Z = FMath::RoundToFloat(nCopy.Z / q) * q;
	nCopy.Normalize();
	return nCopy;
}

void ISpeedWheeledComponent::PostPhysicsUpdatePrv(const float& delta)
{
	ResolveGroupedWheelGroundContacts(delta);
}
