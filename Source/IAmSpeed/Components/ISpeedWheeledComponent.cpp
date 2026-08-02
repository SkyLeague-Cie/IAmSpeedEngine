#include "ISpeedWheeledComponent.h"
#include "IAmSpeed/SubBodies/Solid/SWheelSubBody.h"
#include "HAL/IConsoleManager.h"

static TAutoConsoleVariable<float> CVarIAmSpeedWheelContactNormalVelTimeConstant(
	TEXT("p.IAmSpeed.WheelContact.NormalVelTimeConstant"),
	0.0075f,
	TEXT("Time constant used by the grouped wheel contact solver to reduce inward normal velocity. Lower values absorb contact velocity faster."),
	ECVF_Default);

static TAutoConsoleVariable<float> CVarIAmSpeedWheelContactSoftness(
	TEXT("p.IAmSpeed.WheelContact.Softness"),
	0.0008f,
	TEXT("Softness added to the effective inverse mass denominator in the grouped wheel contact solver."),
	ECVF_Default);

static TAutoConsoleVariable<float> CVarIAmSpeedWheelContactNormalVelocityDeadzone(
	TEXT("p.IAmSpeed.WheelContact.NormalVelocityDeadzone"),
	0.001f,
	TEXT("Normal contact velocity deadzone in cm/s for the grouped wheel contact solver."),
	ECVF_Default);

static TAutoConsoleVariable<float> CVarIAmSpeedWheelContactImpulseDeadzone(
	TEXT("p.IAmSpeed.WheelContact.ImpulseDeadzone"),
	0.5f,
	TEXT("Minimum wheel contact impulse magnitude applied by the grouped wheel contact solver."),
	ECVF_Default);

static TAutoConsoleVariable<int32> CVarIAmSpeedWheelContactLockedSeparatingDamping(
	TEXT("p.IAmSpeed.WheelContact.LockedSeparatingDamping"),
	0,
	TEXT("When non-zero, damps separating normal velocity after a wheel has passed its first contact rebound."),
	ECVF_Default);

static TAutoConsoleVariable<float> CVarIAmSpeedWheelContactLockedSeparatingNormalVelTimeConstant(
	TEXT("p.IAmSpeed.WheelContact.LockedSeparatingNormalVelTimeConstant"),
	0.0075f,
	TEXT("Time constant used to damp separating normal wheel contact velocity after contact lock."),
	ECVF_Default);

static TAutoConsoleVariable<float> CVarIAmSpeedWheelContactLockedSeparatingMaxNormalVelocity(
	TEXT("p.IAmSpeed.WheelContact.LockedSeparatingMaxNormalVelocity"),
	100000.0f,
	TEXT("Maximum separating normal velocity, in cm/s, damped after wheel contact lock."),
	ECVF_Default);

static TAutoConsoleVariable<int32> CVarIAmSpeedWheelContactDebug(
	TEXT("p.IAmSpeed.WheelContact.Debug"),
	0,
	TEXT("Logs grouped wheel contact solver impulses when non-zero."),
	ECVF_Default);

void ISpeedWheeledComponent::ResolveGroupedWheelGroundContacts(const float& delta)
{
	TArray<SWheelGroundContact>& PendingWheelGroundContacts = GetPendingWheelContacts();
	if (PendingWheelGroundContacts.Num() == 0) return;

	const float dt = FMath::Max(delta, 1e-6f);

	const float NormalVelTimeConstant = FMath::Max(CVarIAmSpeedWheelContactNormalVelTimeConstant.GetValueOnAnyThread(), 1e-6f);
	const float gamma = 1.f - FMath::Exp(-dt / NormalVelTimeConstant);
	const float LockedSeparatingNormalVelTimeConstant = FMath::Max(CVarIAmSpeedWheelContactLockedSeparatingNormalVelTimeConstant.GetValueOnAnyThread(), 1e-6f);
	const float LockedSeparatingGamma = 1.f - FMath::Exp(-dt / LockedSeparatingNormalVelTimeConstant);
	const float LockedSeparatingMaxNormalVelocity = FMath::Max(CVarIAmSpeedWheelContactLockedSeparatingMaxNormalVelocity.GetValueOnAnyThread(), 0.0f);
	const bool bLockedSeparatingDampingEnabled = CVarIAmSpeedWheelContactLockedSeparatingDamping.GetValueOnAnyThread() != 0;

	// --- Deadzones ---
	const float VNDeadzone = FMath::Max(CVarIAmSpeedWheelContactNormalVelocityDeadzone.GetValueOnAnyThread(), 0.0f);
	const float JDeadzone = FMath::Max(CVarIAmSpeedWheelContactImpulseDeadzone.GetValueOnAnyThread(), 0.0f);
	// (smaller are those, more network stability)

	// softening (CFM-ish)
	const float Softness = FMath::Max(CVarIAmSpeedWheelContactSoftness.GetValueOnAnyThread(), 0.0f);

	// Snapshot
	const FVector V0 = GetPhysCOMVelocity();
	const FVector W0 = GetPhysAngularVelocity();

	for (const SWheelGroundContact& C : PendingWheelGroundContacts)
	{
		if (C.InvMassEff <= SMALL_NUMBER) continue;

		const FVector N = C.Normal.GetSafeNormal();

		const FVector vContact = V0 + FVector::CrossProduct(W0, C.r);
		const float vN = FVector::DotProduct(vContact, N);

		const bool bUnlockedMovingIntoSurface = !C.bVelocityLocked && vN < -VNDeadzone;
		const bool bMovingIntoSurface = C.bVelocityLocked && vN < -VNDeadzone;
		const bool bDampSeparating =
			bLockedSeparatingDampingEnabled &&
			C.bVelocityLocked &&
			vN > VNDeadzone;

#if !(UE_BUILD_SHIPPING)
		if (bUnlockedMovingIntoSurface && CVarIAmSpeedWheelContactDebug.GetValueOnAnyThread() != 0)
		{
			UE_LOG(LogTemp, Log,
				TEXT("[WheelContactSolver] Mode=UnlockedInwardSkipped dt=%.5f vN=%.3f SpringDisp=%.3f Locked=%d Pos=%s Normal=%s"),
				dt,
				vN,
				C.SpringDisplacement,
				C.bVelocityLocked ? 1 : 0,
				*C.WorldPos.ToString(),
				*N.ToString());
		}
#endif

		if (!bMovingIntoSurface && !bDampSeparating)
			continue;

		const float denom = C.InvMassEff + Softness;
		if (denom <= SMALL_NUMBER) continue;

		const float NormalVelocityToSolve = bMovingIntoSurface
			? -vN
			: FMath::Min(vN, LockedSeparatingMaxNormalVelocity);
		const float SolverGamma = bMovingIntoSurface ? gamma : LockedSeparatingGamma;
		float jn = SolverGamma * NormalVelocityToSolve / denom;
		if (jn <= 0.f) continue;

		const FVector Impulse = (bMovingIntoSurface ? jn : -jn) * N;

		// ignore tiny impulses (stops buzzing at rest)
		if (Impulse.SizeSquared() < (JDeadzone * JDeadzone))
			continue;

		AddPhysImpulseAtPoint(Impulse, C.WorldPos);

#if !(UE_BUILD_SHIPPING)
		if (CVarIAmSpeedWheelContactDebug.GetValueOnAnyThread() != 0)
		{
			UE_LOG(LogTemp, Log,
				TEXT("[WheelContactSolver] Mode=%s dt=%.5f Tau=%.5f Gamma=%.4f vN=%.3f SolvedVN=%.3f SpringDisp=%.3f Locked=%d InvMassEff=%.6f Softness=%.6f Impulse=%s Pos=%s Normal=%s"),
				bMovingIntoSurface ? TEXT("Inward") : TEXT("Separating"),
				dt,
				bMovingIntoSurface ? NormalVelTimeConstant : LockedSeparatingNormalVelTimeConstant,
				SolverGamma,
				vN,
				NormalVelocityToSolve,
				C.SpringDisplacement,
				C.bVelocityLocked ? 1 : 0,
				C.InvMassEff,
				Softness,
				*Impulse.ToString(),
				*C.WorldPos.ToString(),
				*N.ToString());
		}
#endif
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
