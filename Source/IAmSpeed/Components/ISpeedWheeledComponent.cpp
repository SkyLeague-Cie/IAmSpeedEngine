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

static TAutoConsoleVariable<int32> CVarIAmSpeedCanonicalWheelSupportPose(
	TEXT("p.IAmSpeed.WheelContact.CanonicalSupportPose"),
	1,
	TEXT("When non-zero, projects low-energy coplanar wheel support onto the deterministic static spring pose."),
	ECVF_Default);

static TAutoConsoleVariable<float> CVarIAmSpeedCanonicalWheelSupportMaxNormalSpeed(
	TEXT("p.IAmSpeed.WheelContact.CanonicalSupportMaxNormalSpeed"),
	2.5f,
	TEXT("Maximum chassis normal speed in cm/s for canonical wheel support projection."),
	ECVF_Default);

static TAutoConsoleVariable<float> CVarIAmSpeedCanonicalWheelSupportMaxAngularSpeed(
	TEXT("p.IAmSpeed.WheelContact.CanonicalSupportMaxAngularSpeed"),
	0.05f,
	TEXT("Maximum chassis angular speed in rad/s for canonical wheel support projection."),
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

		const bool bMovingIntoSurface =
			(C.bNewContact || C.bAtBumpStop) && vN < -VNDeadzone;
		const bool bDampSeparating =
			bLockedSeparatingDampingEnabled &&
			C.bVelocityLocked &&
			vN > VNDeadzone;

#if !(UE_BUILD_SHIPPING)
		if (!bMovingIntoSurface && vN < -VNDeadzone && CVarIAmSpeedWheelContactDebug.GetValueOnAnyThread() != 0)
		{
			UE_LOG(LogTemp, Log,
				TEXT("[WheelContactSolver] Mode=PersistentSupportSkipped dt=%.5f vN=%.3f SpringDisp=%.3f Locked=%d Pos=%s Normal=%s"),
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

bool ISpeedWheeledComponent::ProjectWheelSupportNonPenetration()
{
	bool bProjected = false;
	for (int32 Pass = 0; Pass < 4; ++Pass)
	{
		float LargestPenetration = 0.01f;
		FVector CorrectionNormal = FVector::ZeroVector;
		for (const USWheelSubBody* Wheel : GetWheelSubBodies())
		{
			if (!Wheel || !Wheel->IsOnGround() || !Wheel->IsAtSuspensionBumpStop())
			{
				continue;
			}
			const FVector Normal = Wheel->GetHitContactNormal().GetSafeNormal();
			const float Penetration = FVector::DotProduct(
				Wheel->GetHitContactPoint() + Wheel->Radius() * Normal - Wheel->WorldPos(),
				Normal);
			if (Penetration > LargestPenetration)
			{
				LargestPenetration = Penetration;
				CorrectionNormal = Normal;
			}
		}
		if (CorrectionNormal.IsNearlyZero())
		{
			break;
		}
		AddPhysLocation(LargestPenetration * CorrectionNormal);
		UpdateSubBodiesKinematics();
		bProjected = true;
	}
	return bProjected;
}

bool ISpeedWheeledComponent::TryProjectCanonicalWheelSupportPose()
{
	if (CVarIAmSpeedCanonicalWheelSupportPose.GetValueOnAnyThread() == 0 ||
		FMath::Abs(GetPhysThrottleInput()) > KINDA_SMALL_NUMBER ||
		FMath::Abs(GetPhysBrakeInput()) > KINDA_SMALL_NUMBER ||
		FMath::Abs(GetPhysSteeringInput()) > KINDA_SMALL_NUMBER)
	{
		return false;
	}

	const TArray<TObjectPtr<USWheelSubBody>>& Wheels = GetWheelSubBodies();
	if (Wheels.Num() < 3)
	{
		return false;
	}

	FVector PlaneNormal = FVector::ZeroVector;
	float PlaneOffsetSum = 0.0f;
	for (const USWheelSubBody* Wheel : Wheels)
	{
		if (!Wheel || !Wheel->IsOnGround() || Wheel->GetConsecutiveGroundFrames() < 5)
		{
			return false;
		}
		PlaneNormal += Wheel->GetHitContactNormal().GetSafeNormal();
	}
	PlaneNormal = PlaneNormal.GetSafeNormal();
	if (PlaneNormal.IsNearlyZero())
	{
		return false;
	}

	for (const USWheelSubBody* Wheel : Wheels)
	{
		const FVector Normal = Wheel->GetHitContactNormal().GetSafeNormal();
		if (FVector::DotProduct(Normal, PlaneNormal) < 0.9999f)
		{
			return false;
		}
		PlaneOffsetSum += FVector::DotProduct(Wheel->GetHitContactPoint(), PlaneNormal);
	}
	const float PlaneOffset = PlaneOffsetSum / Wheels.Num();
	for (const USWheelSubBody* Wheel : Wheels)
	{
		if (FMath::Abs(FVector::DotProduct(Wheel->GetHitContactPoint(), PlaneNormal) - PlaneOffset) > 0.05f)
		{
			return false;
		}
	}

	const float NormalSpeed = FVector::DotProduct(GetPhysCOMVelocity(), PlaneNormal);
	if (FMath::Abs(NormalSpeed) > FMath::Max(0.0f,
		CVarIAmSpeedCanonicalWheelSupportMaxNormalSpeed.GetValueOnAnyThread()) ||
		GetPhysAngularVelocity().Size() > FMath::Max(0.0f,
			CVarIAmSpeedCanonicalWheelSupportMaxAngularSpeed.GetValueOnAnyThread()))
	{
		return false;
	}

	FVector Forward = FVector::VectorPlaneProject(GetPhysForwardVector(), PlaneNormal).GetSafeNormal();
	if (Forward.IsNearlyZero())
	{
		Forward = FVector::VectorPlaneProject(GetPhysRightVector(), PlaneNormal).GetSafeNormal();
	}
	if (Forward.IsNearlyZero())
	{
		return false;
	}
	const FVector Right = FVector::CrossProduct(PlaneNormal, Forward).GetSafeNormal();
	Forward = FVector::CrossProduct(Right, PlaneNormal).GetSafeNormal();
	FMatrix PlaneMatrix = FMatrix::Identity;
	PlaneMatrix.SetAxes(&Forward, &Right, &PlaneNormal);
	const FQuat PlaneRotation(PlaneMatrix);

	auto EvaluatePose = [&Wheels, &PlaneNormal, PlaneOffset, &Forward, &Right, &PlaneRotation](
		const double Pitch, const double Roll, TArray<double>& RequiredOriginOffsets)
	{
		const FQuat Rotation =
			FQuat(Right, static_cast<float>(Pitch)) *
			FQuat(Forward, static_cast<float>(Roll)) * PlaneRotation;
		RequiredOriginOffsets.Reset(Wheels.Num());
		for (const USWheelSubBody* Wheel : Wheels)
		{
			const FVector StaticWheelLocal = Wheel->GetLocalOffset() +
				Wheel->StaticSpringCompression() * FVector::UpVector;
			RequiredOriginOffsets.Add(static_cast<double>(PlaneOffset + Wheel->Radius() -
				FVector::DotProduct(Rotation.RotateVector(StaticWheelLocal), PlaneNormal)));
		}
		return Rotation;
	};

	double Pitch = 0.0;
	double Roll = 0.0;
	constexpr double DerivativeStep = 1.0e-4;
	TArray<double> BaseOffsets;
	for (int32 Iteration = 0; Iteration < 6; ++Iteration)
	{
		EvaluatePose(Pitch, Roll, BaseOffsets);
		double Mean = 0.0;
		for (const double Value : BaseOffsets) Mean += Value;
		Mean /= BaseOffsets.Num();

		TArray<double> PitchOffsets;
		TArray<double> RollOffsets;
		EvaluatePose(Pitch + DerivativeStep, Roll, PitchOffsets);
		EvaluatePose(Pitch, Roll + DerivativeStep, RollOffsets);
		double PitchMean = 0.0;
		double RollMean = 0.0;
		for (int32 Index = 0; Index < BaseOffsets.Num(); ++Index)
		{
			PitchMean += PitchOffsets[Index];
			RollMean += RollOffsets[Index];
		}
		PitchMean /= BaseOffsets.Num();
		RollMean /= BaseOffsets.Num();

		double H00 = 0.0;
		double H01 = 0.0;
		double H11 = 0.0;
		double G0 = 0.0;
		double G1 = 0.0;
		for (int32 Index = 0; Index < BaseOffsets.Num(); ++Index)
		{
			const double Residual = BaseOffsets[Index] - Mean;
			const double J0 = ((PitchOffsets[Index] - PitchMean) - Residual) / DerivativeStep;
			const double J1 = ((RollOffsets[Index] - RollMean) - Residual) / DerivativeStep;
			H00 += J0 * J0;
			H01 += J0 * J1;
			H11 += J1 * J1;
			G0 += J0 * Residual;
			G1 += J1 * Residual;
		}
		const double Determinant = H00 * H11 - H01 * H01;
		if (FMath::Abs(Determinant) <= 1.0e-12)
		{
			break;
		}
		Pitch += (-H11 * G0 + H01 * G1) / Determinant;
		Roll += (H01 * G0 - H00 * G1) / Determinant;
	}

	const FQuat TargetRotation = EvaluatePose(Pitch, Roll, BaseOffsets).GetNormalized();
	double TargetOriginNormal = 0.0;
	for (const double Value : BaseOffsets) TargetOriginNormal += Value;
	TargetOriginNormal /= BaseOffsets.Num();
	const FVector CurrentOrigin = GetPhysLocation();
	const FVector TargetOrigin = CurrentOrigin +
		(static_cast<float>(TargetOriginNormal) - FVector::DotProduct(CurrentOrigin, PlaneNormal)) * PlaneNormal;
	const float PoseAngleError = TargetRotation.AngularDistance(GetPhysRotation());
	const float PoseNormalError = FMath::Abs(FVector::DotProduct(TargetOrigin - CurrentOrigin, PlaneNormal));
	if (PoseAngleError > FMath::DegreesToRadians(2.0f) || PoseNormalError > 2.0f)
	{
		return false;
	}

	SetPhysRotation(TargetRotation);
	SetPhysLocation(TargetOrigin);
	SetPhysCOMVelocity(GetPhysCOMVelocity() - NormalSpeed * PlaneNormal);
	SetPhysAngularVelocity(FVector::ZeroVector);
	for (const TObjectPtr<USWheelSubBody>& WheelPtr : Wheels)
	{
		USWheelSubBody* Wheel = WheelPtr.Get();
		Wheel->SetLastDisplacement(Wheel->StaticSpringCompression());
	}
	UpdateSubBodiesKinematics();
	return true;
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
	ProjectWheelSupportNonPenetration();
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
	ProjectWheelSupportNonPenetration();
	TryProjectCanonicalWheelSupportPose();
}
