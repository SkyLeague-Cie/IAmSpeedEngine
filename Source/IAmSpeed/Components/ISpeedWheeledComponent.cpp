#include "ISpeedWheeledComponent.h"
#include "IAmSpeed/SubBodies/Solid/SWheelSubBody.h"
#include "HAL/IConsoleManager.h"

static TAutoConsoleVariable<float> CVarIAmSpeedWheelContactNormalVelTimeConstant(
	TEXT("p.IAmSpeed.WheelContact.NormalVelTimeConstant"),
	-1.0f,
	TEXT("Diagnostic override, in seconds, for the grouped wheel contact inward-normal-velocity response. Negative values keep the vehicle preset."),
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

static TAutoConsoleVariable<int32> CVarIAmSpeedWheelContactSolverMode(
	TEXT("p.IAmSpeed.WheelContact.SolverMode"),
	1,
	TEXT("Grouped-wheel solver: 0 keeps the legacy shared snapshot, 1 solves inward contacts as one deterministic block."),
	ECVF_Default);

static TAutoConsoleVariable<int32> CVarIAmSpeedWheelSupportTransactionalProjection(
	TEXT("p.IAmSpeed.WheelSupport.TransactionalProjection"),
	1,
	TEXT("Probes all static ground supports, locally reacquires established patches, projects the coupled support set, then publishes one final wheel mask."),
	ECVF_Default);

static TAutoConsoleVariable<float> CVarIAmSpeedWheelSupportProjectionMaxGap(
	TEXT("p.IAmSpeed.WheelSupport.ProjectionMaxGap"),
	5.0f,
	TEXT("Maximum local same-surface separation, in cm, admitted by support projection."),
	ECVF_Default);

static TAutoConsoleVariable<float> CVarIAmSpeedWheelSupportProjectionNormalDot(
	TEXT("p.IAmSpeed.WheelSupport.ProjectionNormalDot"),
	0.995f,
	TEXT("Minimum old/new support-normal dot for same-surface projection."),
	ECVF_Default);

static TAutoConsoleVariable<float> CVarIAmSpeedWheelSupportProjectionMinGravityAlignment(
	TEXT("p.IAmSpeed.WheelSupport.ProjectionMinGravityAlignment"),
	0.9f,
	TEXT("Minimum support-normal alignment with world up. Near-vertical wall and gutter retention remains on the established sweep path."),
	ECVF_Default);

static TAutoConsoleVariable<float> CVarIAmSpeedWheelSupportProjectionPatchTravelSlack(
	TEXT("p.IAmSpeed.WheelSupport.ProjectionPatchTravelSlack"),
	5.0f,
	TEXT("Tangential patch-travel tolerance, in cm, added to the rigid-point displacement predicted for one physics step."),
	ECVF_Default);

static TAutoConsoleVariable<int32> CVarIAmSpeedWheelSupportProjectionPasses(
	TEXT("p.IAmSpeed.WheelSupport.ProjectionPasses"),
	12,
	TEXT("Bounded deterministic Gauss-Seidel passes used by support projection."),
	ECVF_Default);

static TAutoConsoleVariable<float> CVarIAmSpeedWheelSupportProjectionRotationLength(
	TEXT("p.IAmSpeed.WheelSupport.ProjectionRotationLength"),
	50.0f,
	TEXT("Characteristic length, in cm, balancing translation and rotation in support projection."),
	ECVF_Default);

static TAutoConsoleVariable<float> CVarIAmSpeedWheelSupportProjectionMaxRotationDegrees(
	TEXT("p.IAmSpeed.WheelSupport.ProjectionMaxRotationDegrees"),
	10.0f,
	TEXT("Maximum total support-projection rotation in degrees."),
	ECVF_Default);

static TAutoConsoleVariable<float> CVarIAmSpeedWheelSupportProjectionReachSkin(
	TEXT("p.IAmSpeed.WheelSupport.ProjectionReachSkin"),
	0.05f,
	TEXT("Small inward reach margin, in cm, used to make the projected pose robust to sweep boundary tolerance."),
	ECVF_Default);

static TAutoConsoleVariable<int32> CVarIAmSpeedWheelSupportProjectionDebug(
	TEXT("p.IAmSpeed.WheelSupport.ProjectionDebug"),
	0,
	TEXT("Logs established-support patch admission and pose projection when non-zero."),
	ECVF_Default);

float ISpeedWheeledComponent::GetWheelContactNormalVelocityTimeConstantOverride()
{
	return CVarIAmSpeedWheelContactNormalVelTimeConstant.GetValueOnAnyThread();
}

void ISpeedWheeledComponent::NotifyWheelOnGroundStateChanged()
{
	if (!bDeferWheelGroundStateUpdate)
	{
		UpdateWheelOnGroundStates();
	}
}

void ISpeedWheeledComponent::BeginDeferredWheelGroundStateUpdate()
{
	bDeferWheelGroundStateUpdate = true;
}

void ISpeedWheeledComponent::EndDeferredWheelGroundStateUpdate()
{
	bDeferWheelGroundStateUpdate = false;
	UpdateWheelOnGroundStates();
}

float ISpeedWheeledComponent::GetWheelContactNormalVelocityTimeConstant() const
{
	const float DiagnosticOverride = GetWheelContactNormalVelocityTimeConstantOverride();
	return DiagnosticOverride >= 0.0f ? DiagnosticOverride : 0.0075f;
}

float ISpeedWheeledComponent::GetWheelContactNormalVelocityDeadzone() const
{
	return CVarIAmSpeedWheelContactNormalVelocityDeadzone.GetValueOnAnyThread();
}

float ISpeedWheeledComponent::GetWheelContactMaxInwardNormalVelocityToSolve() const
{
	return TNumericLimits<float>::Max();
}

bool ISpeedWheeledComponent::TryComputeWheelSuspensionForceOverride(
	const USWheelSubBody& Wheel,
	const float LastDisplacement,
	const float CurrentDisplacement,
	const float NormalVelocity,
	float& OutForce) const
{
	return false;
}

void ISpeedWheeledComponent::ResolveGroupedWheelGroundContacts(const float& delta)
{
	TArray<SWheelGroundContact>& PendingWheelGroundContacts = GetPendingWheelContacts();
	if (PendingWheelGroundContacts.Num() == 0) return;

	const float dt = FMath::Max(delta, 1e-6f);

	// A response faster than one simulation step is not representable by this
	// first-order solver. Keep tuning in seconds while bounding its discrete form.
	const float NormalVelTimeConstant = FMath::Max(GetWheelContactNormalVelocityTimeConstant(), dt);
	const float gamma = 1.f - FMath::Exp(-dt / NormalVelTimeConstant);
	const float LockedSeparatingNormalVelTimeConstant = FMath::Max(CVarIAmSpeedWheelContactLockedSeparatingNormalVelTimeConstant.GetValueOnAnyThread(), 1e-6f);
	const float LockedSeparatingGamma = 1.f - FMath::Exp(-dt / LockedSeparatingNormalVelTimeConstant);
	const float LockedSeparatingMaxNormalVelocity = FMath::Max(CVarIAmSpeedWheelContactLockedSeparatingMaxNormalVelocity.GetValueOnAnyThread(), 0.0f);
	const float MaxInwardNormalVelocityToSolve = FMath::Max(
		GetWheelContactMaxInwardNormalVelocityToSolve(), 0.0f);
	const bool bLockedSeparatingDampingEnabled = CVarIAmSpeedWheelContactLockedSeparatingDamping.GetValueOnAnyThread() != 0;
	// --- Deadzones ---
	const float VNDeadzone = FMath::Max(GetWheelContactNormalVelocityDeadzone(), 0.0f);
	const float JDeadzone = FMath::Max(CVarIAmSpeedWheelContactImpulseDeadzone.GetValueOnAnyThread(), 0.0f);
	// (smaller are those, more network stability)

	// softening (CFM-ish)
	const float Softness = FMath::Max(CVarIAmSpeedWheelContactSoftness.GetValueOnAnyThread(), 0.0f);

	// Legacy solves every wheel from the same snapshot. The diagnostic coupled
	// modes update the virtual rigid state after each impulse so later contacts
	// observe the response already supplied by earlier contacts.
	const FVector V0 = GetPhysCOMVelocity();
	const FVector W0 = GetPhysAngularVelocity();
	FVector CurrentV = V0;
	FVector CurrentW = W0;
	const int32 SolverMode = FMath::Clamp(
		CVarIAmSpeedWheelContactSolverMode.GetValueOnAnyThread(), 0, 1);
	TArray<int32, TInlineAllocator<4>> ContactOrder;
	ContactOrder.Reserve(PendingWheelGroundContacts.Num());
	for (int32 ContactIndex = 0; ContactIndex < PendingWheelGroundContacts.Num(); ++ContactIndex)
	{
		ContactOrder.Add(ContactIndex);
	}
	ContactOrder.Sort([&PendingWheelGroundContacts](const int32 A, const int32 B)
	{
		const USWheelSubBody* WheelA = PendingWheelGroundContacts[A].Wheel;
		const USWheelSubBody* WheelB = PendingWheelGroundContacts[B].Wheel;
		const int32 WheelIndexA = WheelA ? WheelA->Idx() : INDEX_NONE;
		const int32 WheelIndexB = WheelB ? WheelB->Idx() : INDEX_NONE;
		return WheelIndexA < WheelIndexB;
	});
	if (SolverMode == 1 && ContactOrder.Num() <= 4)
	{
		TArray<int32, TInlineAllocator<4>> ActiveContacts;
		TArray<double, TInlineAllocator<4>> TargetNormalVelocityChanges;
		bool bHasSeparatingConstraint = false;
		for (const int32 ContactIndex : ContactOrder)
		{
			const SWheelGroundContact& C = PendingWheelGroundContacts[ContactIndex];
			if (C.InvMassEff <= SMALL_NUMBER)
			{
				continue;
			}
			const FVector N = C.Normal.GetSafeNormal();
			const float vN = FVector::DotProduct(
				V0 + FVector::CrossProduct(W0, C.r), N);
			const bool bMovingIntoSurface =
				(C.bNewContact || C.bAtBumpStop) && vN < -VNDeadzone;
			const bool bDampSeparating =
				bLockedSeparatingDampingEnabled && C.bVelocityLocked && vN > VNDeadzone;
			if (bDampSeparating)
			{
				bHasSeparatingConstraint = true;
				break;
			}
			if (!bMovingIntoSurface)
			{
				continue;
			}
			ActiveContacts.Add(ContactIndex);
			TargetNormalVelocityChanges.Add(double(gamma) * double(FMath::Min(
				-vN, MaxInwardNormalVelocityToSolve)));
		}

		// Separating damping deliberately applies an attractive impulse and is not
		// part of the unilateral inward-contact complementarity problem.
		if (!bHasSeparatingConstraint && ActiveContacts.Num() > 0)
		{
			const int32 ConstraintCount = ActiveContacts.Num();
			double Response[4][4] = {};
			const double InvMass = 1.0 / double(FMath::Max(GetPhysMass(), 1.0f));
			const FMatrix InvInertia = ComputeWorldInvInertiaTensor();
			for (int32 Row = 0; Row < ConstraintCount; ++Row)
			{
				const SWheelGroundContact& ContactI =
					PendingWheelGroundContacts[ActiveContacts[Row]];
				const FVector NormalI = ContactI.Normal.GetSafeNormal();
				for (int32 Column = 0; Column < ConstraintCount; ++Column)
				{
					const SWheelGroundContact& ContactJ =
						PendingWheelGroundContacts[ActiveContacts[Column]];
					const FVector NormalJ = ContactJ.Normal.GetSafeNormal();
					const FVector DeltaAngularVelocity = InvInertia.TransformVector(
						FVector::CrossProduct(ContactJ.r, NormalJ));
					const FVector DeltaVelocityAtI = float(InvMass) * NormalJ
						+ FVector::CrossProduct(DeltaAngularVelocity, ContactI.r);
					Response[Row][Column] = double(FVector::DotProduct(
						DeltaVelocityAtI, NormalI));
					if (Row == Column)
					{
						Response[Row][Column] += double(Softness);
					}
				}
			}

			auto SolveSubset = [&Response, &TargetNormalVelocityChanges,
				ConstraintCount](const uint32 ActiveMask, double OutLambda[4])
			{
				int32 SubsetIndices[4] = {};
				int32 SubsetCount = 0;
				for (int32 Index = 0; Index < ConstraintCount; ++Index)
				{
					OutLambda[Index] = 0.0;
					if ((ActiveMask & (1u << Index)) != 0)
					{
						SubsetIndices[SubsetCount++] = Index;
					}
				}
				double Augmented[4][5] = {};
				for (int32 Row = 0; Row < SubsetCount; ++Row)
				{
					for (int32 Column = 0; Column < SubsetCount; ++Column)
					{
						Augmented[Row][Column] =
							Response[SubsetIndices[Row]][SubsetIndices[Column]];
					}
					Augmented[Row][SubsetCount] =
						TargetNormalVelocityChanges[SubsetIndices[Row]];
				}
				for (int32 PivotColumn = 0; PivotColumn < SubsetCount; ++PivotColumn)
				{
					int32 PivotRow = PivotColumn;
					for (int32 Candidate = PivotColumn + 1; Candidate < SubsetCount; ++Candidate)
					{
						if (FMath::Abs(Augmented[Candidate][PivotColumn]) >
							FMath::Abs(Augmented[PivotRow][PivotColumn]))
						{
							PivotRow = Candidate;
						}
					}
					if (FMath::Abs(Augmented[PivotRow][PivotColumn]) <= 1.0e-12)
					{
						return false;
					}
					if (PivotRow != PivotColumn)
					{
						for (int32 Column = PivotColumn; Column <= SubsetCount; ++Column)
						{
							Swap(Augmented[PivotColumn][Column], Augmented[PivotRow][Column]);
						}
					}
					const double Pivot = Augmented[PivotColumn][PivotColumn];
					for (int32 Column = PivotColumn; Column <= SubsetCount; ++Column)
					{
						Augmented[PivotColumn][Column] /= Pivot;
					}
					for (int32 Row = 0; Row < SubsetCount; ++Row)
					{
						if (Row == PivotColumn)
						{
							continue;
						}
						const double Factor = Augmented[Row][PivotColumn];
						for (int32 Column = PivotColumn; Column <= SubsetCount; ++Column)
						{
							Augmented[Row][Column] -= Factor * Augmented[PivotColumn][Column];
						}
					}
				}
				for (int32 Row = 0; Row < SubsetCount; ++Row)
				{
					OutLambda[SubsetIndices[Row]] = Augmented[Row][SubsetCount];
				}
				return true;
			};

			double Lambda[4] = {};
			bool bFoundSolution = false;
			const uint32 SubsetCount = 1u << ConstraintCount;
			for (uint32 ActiveMask = 1; ActiveMask < SubsetCount && !bFoundSolution; ++ActiveMask)
			{
				double CandidateLambda[4] = {};
				if (!SolveSubset(ActiveMask, CandidateLambda))
				{
					continue;
				}
				bool bFeasible = true;
				for (int32 Row = 0; Row < ConstraintCount && bFeasible; ++Row)
				{
					if ((ActiveMask & (1u << Row)) != 0 && CandidateLambda[Row] <= 1.0e-9)
					{
						bFeasible = false;
						break;
					}
					double AchievedChange = 0.0;
					for (int32 Column = 0; Column < ConstraintCount; ++Column)
					{
						AchievedChange += Response[Row][Column] * CandidateLambda[Column];
					}
					if ((ActiveMask & (1u << Row)) == 0 &&
						AchievedChange + 1.0e-7 < TargetNormalVelocityChanges[Row])
					{
						bFeasible = false;
					}
				}
				if (bFeasible)
				{
					for (int32 Index = 0; Index < ConstraintCount; ++Index)
					{
						Lambda[Index] = CandidateLambda[Index];
					}
					bFoundSolution = true;
				}
			}

			if (bFoundSolution)
			{
				for (int32 Index = 0; Index < ConstraintCount; ++Index)
				{
					const SWheelGroundContact& C =
						PendingWheelGroundContacts[ActiveContacts[Index]];
					const FVector Impulse = float(Lambda[Index]) * C.Normal.GetSafeNormal();
					if (Impulse.SizeSquared() < JDeadzone * JDeadzone)
					{
						continue;
					}
					AddPhysImpulseAtPoint(Impulse, C.WorldPos);
				}
				PendingWheelGroundContacts.Reset();
				return;
			}
		}
	}

	for (const int32 ContactIndex : ContactOrder)
	{
		const SWheelGroundContact& C = PendingWheelGroundContacts[ContactIndex];
		if (C.InvMassEff <= SMALL_NUMBER) continue;

		const FVector N = C.Normal.GetSafeNormal();

		const FVector& SolverV = SolverMode == 0 ? V0 : CurrentV;
		const FVector& SolverW = SolverMode == 0 ? W0 : CurrentW;
		const FVector vContact = SolverV + FVector::CrossProduct(SolverW, C.r);
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
			? FMath::Min(-vN, MaxInwardNormalVelocityToSolve)
			: FMath::Min(vN, LockedSeparatingMaxNormalVelocity);
		const float SolverGamma = bMovingIntoSurface ? gamma : LockedSeparatingGamma;
		float jn = SolverGamma * NormalVelocityToSolve / denom;
		if (jn <= 0.f) continue;

		const FVector Impulse = (bMovingIntoSurface ? jn : -jn) * N;

		// ignore tiny impulses (stops buzzing at rest)
		if (Impulse.SizeSquared() < (JDeadzone * JDeadzone))
			continue;

		AddPhysImpulseAtPoint(Impulse, C.WorldPos);
		if (SolverMode != 0)
		{
			CurrentV = GetPhysCOMVelocity();
			CurrentW = GetPhysAngularVelocity();
		}

#if !(UE_BUILD_SHIPPING)
		if (CVarIAmSpeedWheelContactDebug.GetValueOnAnyThread() != 0)
		{
			UE_LOG(LogTemp, Log,
				TEXT("[WheelContactSolver] Mode=%s Coupling=%d dt=%.5f Tau=%.5f Gamma=%.4f vN=%.3f SolvedVN=%.3f SpringDisp=%.3f Locked=%d InvMassEff=%.6f Softness=%.6f Impulse=%s Pos=%s Normal=%s"),
				bMovingIntoSurface ? TEXT("Inward") : TEXT("Separating"),
				SolverMode,
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
	if (CVarIAmSpeedCanonicalWheelSupportPose.GetValueOnAnyThread() == 0)
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
	const bool bFirstMovableFrame = CanBypassCanonicalSupportContactWarmup();
	for (const USWheelSubBody* Wheel : Wheels)
	{
		if (!Wheel || !Wheel->IsOnGround() ||
			(!bFirstMovableFrame && Wheel->GetConsecutiveGroundFrames() < 5))
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
	const FVector TangentialAngularVelocity = GetPhysAngularVelocity()
		- FVector::DotProduct(GetPhysAngularVelocity(), PlaneNormal) * PlaneNormal;
	const bool bPreserveNormalRotation = CanPreserveCanonicalSupportNormalRotation();
	const float SupportAngularSpeed = bPreserveNormalRotation
		? TangentialAngularVelocity.Size()
		: GetPhysAngularVelocity().Size();
	if (FMath::Abs(NormalSpeed) > FMath::Max(0.0f,
		CVarIAmSpeedCanonicalWheelSupportMaxNormalSpeed.GetValueOnAnyThread()) ||
		SupportAngularSpeed > FMath::Max(0.0f,
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
	// The canonical support pose owns only pitch/roll. Preserve rotation around
	// the support normal so throttle steering and powerslide can still build the
	// intended yaw response without changing ride height or chassis attitude.
	SetPhysAngularVelocity(bPreserveNormalRotation
		? FVector::DotProduct(GetPhysAngularVelocity(), PlaneNormal) * PlaneNormal
		: FVector::ZeroVector);
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
	TArray<TPair<USWheelSubBody*, SHitResult>, TInlineAllocator<4>> EstablishedSupports;
	bool bDeferredWheelGroundState = false;
	const bool bTransactionalProjection =
		CVarIAmSpeedWheelSupportTransactionalProjection.GetValueOnAnyThread() != 0;
	if (bTransactionalProjection)
	{
		struct FGroundProbe
		{
			USWheelSubBody* Wheel = nullptr;
			bool bWasGrounded = false;
			SHitResult PreviousHit;
			bool bHasProbeHit = false;
			SHitResult ProbeHit;
		};

		TArray<FGroundProbe, TInlineAllocator<4>> Probes;
		Probes.Reserve(GetWheelSubBodies().Num());
		for (USWheelSubBody* Wheel : GetWheelSubBodies())
		{
			if (!Wheel)
			{
				continue;
			}
			FGroundProbe& Probe = Probes.AddDefaulted_GetRef();
			Probe.Wheel = Wheel;
			Probe.bWasGrounded = Wheel->IsOnGround();
			Probe.PreviousHit = Wheel->GetHit();
			Probe.bHasProbeHit = Wheel->ProbeSuspensionOnGround(Probe.ProbeHit, delta);
		}

		TArray<int32, TInlineAllocator<4>> EstablishedMisses;
		const float MaxGap = FMath::Max(
			0.0f, CVarIAmSpeedWheelSupportProjectionMaxGap.GetValueOnAnyThread());
		const float NormalDot = FMath::Clamp(
			CVarIAmSpeedWheelSupportProjectionNormalDot.GetValueOnAnyThread(), -1.0f, 1.0f);
		const float MinGravityAlignment = FMath::Clamp(
			CVarIAmSpeedWheelSupportProjectionMinGravityAlignment.GetValueOnAnyThread(), -1.0f, 1.0f);
		const float PatchTravelSlack = FMath::Max(0.0f,
			CVarIAmSpeedWheelSupportProjectionPatchTravelSlack.GetValueOnAnyThread());
		for (int32 Index = 0; Index < Probes.Num(); ++Index)
		{
			const FGroundProbe& Probe = Probes[Index];
			USWheelSubBody* Wheel = Probe.Wheel;
			UPrimitiveComponent* PreviousSurface = Probe.PreviousHit.Component.Get();
			const FVector PreviousNormal = Probe.PreviousHit.ImpactNormal.GetSafeNormal();
#if !(UE_BUILD_SHIPPING)
			if (CVarIAmSpeedWheelSupportProjectionDebug.GetValueOnAnyThread() != 0 &&
				Probe.bWasGrounded && !Probe.bHasProbeHit)
			{
				UE_LOG(LogTemp, Log,
					TEXT("[WheelSupportProjectionCandidate] Frame=%d Wheel=%d Surface=%d Static=%d NormalZ=%.3f Locked=%d Jump=%d Unilateral=%d"),
					NumFrame(), Index, PreviousSurface ? 1 : 0,
					PreviousSurface && PreviousSurface->Mobility == EComponentMobility::Static ? 1 : 0,
					PreviousNormal.Z, Wheel->IsContactVelocityLocked() ? 1 : 0,
					Wheel->IsJumping() ? 1 : 0,
					Wheel->HasJumpUnilateralSupport() ? 1 : 0);
			}
#endif
			if (!Probe.bWasGrounded || Probe.bHasProbeHit || !PreviousSurface ||
				PreviousSurface->Mobility != EComponentMobility::Static ||
				PreviousNormal.IsNearlyZero() || PreviousNormal.Z < MinGravityAlignment ||
				Wheel->HasJumpUnilateralSupport())
			{
				continue;
			}

			// Reacquire the patch locally under this wheel. Another wheel hitting the
			// same stadium component is not evidence that this historical plane still
			// exists here (notably across gutter seams).
			SHitResult LocalPatchHit;
			const bool bHasLocalPatch = Wheel->SweepSuspensionAlongNormal(
				PreviousNormal, MaxGap, delta, LocalPatchHit);
			const FVector LocalNormal = LocalPatchHit.ImpactNormal.GetSafeNormal();
			const FVector PatchTravel = LocalPatchHit.ImpactPoint - Probe.PreviousHit.ImpactPoint;
			const float TangentialPatchTravel = FVector::VectorPlaneProject(
				PatchTravel, PreviousNormal).Size();
			const float PredictedPatchTravel =
				GetPhysVelocityAtPoint(Probe.PreviousHit.ImpactPoint).Size() * delta;
			const float MaxPatchTravel = PredictedPatchTravel + PatchTravelSlack;
			const bool bSameLocalPatch = bHasLocalPatch &&
				LocalPatchHit.Component.Get() == PreviousSurface &&
				!LocalNormal.IsNearlyZero() &&
				FVector::DotProduct(LocalNormal, PreviousNormal) >= NormalDot &&
				TangentialPatchTravel <= MaxPatchTravel;
			if (!bSameLocalPatch)
			{
#if !(UE_BUILD_SHIPPING)
				if (CVarIAmSpeedWheelSupportProjectionDebug.GetValueOnAnyThread() != 0)
				{
					UE_LOG(LogTemp, Log,
						TEXT("[WheelSupportPatchRejected] Frame=%d Wheel=%d LocalHit=%d SameComponent=%d NormalDot=%.5f TangentialTravel=%.3f MaxTravel=%.3f PreviousFace=%d LocalFace=%d PreviousPoint=%s LocalPoint=%s"),
						NumFrame(), Index, bHasLocalPatch ? 1 : 0,
						bHasLocalPatch && LocalPatchHit.Component.Get() == PreviousSurface ? 1 : 0,
						bHasLocalPatch ? FVector::DotProduct(LocalNormal, PreviousNormal) : -1.0f,
						TangentialPatchTravel, MaxPatchTravel,
						Probe.PreviousHit.FaceIndex, LocalPatchHit.FaceIndex,
						*Probe.PreviousHit.ImpactPoint.ToString(),
						*LocalPatchHit.ImpactPoint.ToString());
				}
#endif
				continue;
			}

			FVector SweepStart = FVector::ZeroVector;
			FVector SweepEnd = FVector::ZeroVector;
			Wheel->GetSuspensionSweepSegment(delta, SweepStart, SweepEnd);
			const float SweepRadius = Wheel->GetCollisionShape().GetSphereRadius();
			const float ReachGap = FVector::DotProduct(
				SweepEnd - LocalPatchHit.ImpactPoint, LocalNormal) - SweepRadius;
#if !(UE_BUILD_SHIPPING)
			if (CVarIAmSpeedWheelSupportProjectionDebug.GetValueOnAnyThread() != 0)
			{
				UE_LOG(LogTemp, Log,
					TEXT("[WheelSupportProjectionCandidateResult] Frame=%d Wheel=%d LocalPatch=1 Gap=%.4f PreviousFace=%d LocalFace=%d NormalDot=%.5f TangentialTravel=%.3f MaxTravel=%.3f"),
					NumFrame(), Index, ReachGap, Probe.PreviousHit.FaceIndex,
					LocalPatchHit.FaceIndex, FVector::DotProduct(LocalNormal, PreviousNormal),
					TangentialPatchTravel, MaxPatchTravel);
			}
#endif
			if (ReachGap <= MaxGap && ReachGap >= -MaxGap)
			{
				EstablishedSupports.Emplace(Wheel, LocalPatchHit);
				Probes[Index].PreviousHit = LocalPatchHit;
				if (ReachGap > 0.01f)
				{
					EstablishedMisses.Add(Index);
				}
			}
		}

		if (EstablishedMisses.Num() > 0)
		{
			const FVector OriginalCOM = GetPhysCOM();
			const FQuat OriginalRotation = GetPhysRotation();
			const float RotationLength = FMath::Max(1.0f,
				CVarIAmSpeedWheelSupportProjectionRotationLength.GetValueOnAnyThread());
			const float RotationLengthSquared = RotationLength * RotationLength;
			const float ReachSkin = FMath::Max(
				0.0f, CVarIAmSpeedWheelSupportProjectionReachSkin.GetValueOnAnyThread());
			const int32 Passes = FMath::Clamp(
				CVarIAmSpeedWheelSupportProjectionPasses.GetValueOnAnyThread(), 1, 32);

			auto ApplyConstraint = [this, RotationLengthSquared](
				const FVector& Direction, const FVector& WorldPoint, const float Violation)
			{
				if (Violation <= 0.001f)
				{
					return;
				}
				const FVector N = Direction.GetSafeNormal();
				const FVector AngularJacobian = FVector::CrossProduct(
					WorldPoint - GetPhysCOM(), N);
				const float Denominator = 1.0f
					+ AngularJacobian.SizeSquared() / RotationLengthSquared;
				const float Lambda = Violation / FMath::Max(Denominator, 1.0f);
				SetPhysCOMLocation(GetPhysCOM() + Lambda * N);
				const FVector DeltaAngular =
					(Lambda / RotationLengthSquared) * AngularJacobian;
				const float DeltaAngle = DeltaAngular.Size();
				if (DeltaAngle > SMALL_NUMBER)
				{
					const FQuat WorldDelta(DeltaAngular / DeltaAngle, DeltaAngle);
					SetPhysRotation((WorldDelta * GetPhysRotation()).GetNormalized());
				}
				UpdateSubBodiesKinematics();
			};

			for (int32 Pass = 0; Pass < Passes; ++Pass)
			{
				for (const int32 Index : EstablishedMisses)
				{
					const FGroundProbe& Probe = Probes[Index];
					const FVector N = Probe.PreviousHit.ImpactNormal.GetSafeNormal();
					FVector SweepStart = FVector::ZeroVector;
					FVector SweepEnd = FVector::ZeroVector;
					Probe.Wheel->GetSuspensionSweepSegment(delta, SweepStart, SweepEnd);
					const float Radius = Probe.Wheel->GetCollisionShape().GetSphereRadius();
					const float Gap = FVector::DotProduct(
						SweepEnd - Probe.PreviousHit.ImpactPoint, N) - Radius;
					ApplyConstraint(-N, SweepEnd, Gap + ReachSkin);
				}

				for (const FGroundProbe& Probe : Probes)
				{
					if (!Probe.bHasProbeHit)
					{
						continue;
					}
					const FVector N = Probe.ProbeHit.ImpactNormal.GetSafeNormal();
					const float Radius = Probe.Wheel->GetCollisionShape().GetSphereRadius();
					FVector SweepStart = FVector::ZeroVector;
					FVector SweepEnd = FVector::ZeroVector;
					Probe.Wheel->GetSuspensionSweepSegment(delta, SweepStart, SweepEnd);
					const float ReachGap = FVector::DotProduct(
						SweepEnd - Probe.ProbeHit.ImpactPoint, N) - Radius;
					ApplyConstraint(-N, SweepEnd, ReachGap);

					const float Clearance = FVector::DotProduct(
						Probe.Wheel->WorldPos() - Probe.ProbeHit.ImpactPoint, N) - Radius;
					ApplyConstraint(N, Probe.Wheel->WorldPos(), -Clearance);
				}
			}

			FQuat RelativeRotation = (OriginalRotation.Inverse()
				* GetPhysRotation()).GetNormalized();
			float RotationAngle = 0.0f;
			FVector RotationAxis = FVector::ZeroVector;
			RelativeRotation.ToAxisAndAngle(RotationAxis, RotationAngle);
			RotationAngle = FMath::Min(RotationAngle, 2.0f * PI - RotationAngle);
			const float MaxRotationRadians = FMath::DegreesToRadians(FMath::Max(0.0f,
				CVarIAmSpeedWheelSupportProjectionMaxRotationDegrees.GetValueOnAnyThread()));
			const bool bWithinBounds =
				(GetPhysCOM() - OriginalCOM).Size() <= MaxGap &&
				RotationAngle <= MaxRotationRadians;
			if (!bWithinBounds)
			{
				SetPhysCOMLocation(OriginalCOM);
				SetPhysRotation(OriginalRotation);
				UpdateSubBodiesKinematics();
			}

#if !(UE_BUILD_SHIPPING)
			if (CVarIAmSpeedWheelSupportProjectionDebug.GetValueOnAnyThread() != 0)
			{
				UE_LOG(LogTemp, Log,
					TEXT("[WheelSupportProjection] Frame=%d Retained=%d Applied=%d Translation=%.3f RotationDeg=%.3f"),
					NumFrame(), EstablishedMisses.Num(), bWithinBounds ? 1 : 0,
					(GetPhysCOM() - OriginalCOM).Size(),
					FMath::RadiansToDegrees(RotationAngle));
				for (const int32 Index : EstablishedMisses)
				{
					const FGroundProbe& Probe = Probes[Index];
					const FVector N = Probe.PreviousHit.ImpactNormal.GetSafeNormal();
					FVector SweepStart = FVector::ZeroVector;
					FVector SweepEnd = FVector::ZeroVector;
					Probe.Wheel->GetSuspensionSweepSegment(delta, SweepStart, SweepEnd);
					const float Radius = Probe.Wheel->GetCollisionShape().GetSphereRadius();
					const float Gap = FVector::DotProduct(
						SweepEnd - Probe.PreviousHit.ImpactPoint, N) - Radius;
					SHitResult VerificationHit;
					const bool bVerificationHit =
						Probe.Wheel->ProbeSuspensionOnGround(VerificationHit, delta);
					UE_LOG(LogTemp, Log,
						TEXT("[WheelSupportProjectionWheel] Frame=%d Wheel=%d Gap=%.4f Resweep=%d"),
						NumFrame(), Index, Gap, bVerificationHit ? 1 : 0);
				}
			}
#endif
		}

		if (EstablishedSupports.Num() > 0)
		{
			BeginDeferredWheelGroundStateUpdate();
			bDeferredWheelGroundState = true;
		}
	}

	for (auto& Wheel : GetWheelSubBodies())
	{
		if (Wheel)
		{
			Wheel->SweepSuspension(delta);
		}
	}
	if (bTransactionalProjection && bDeferredWheelGroundState)
	{
		for (const TPair<USWheelSubBody*, SHitResult>& Support : EstablishedSupports)
		{
			USWheelSubBody* Wheel = Support.Key;
			if (!Wheel || Wheel->IsOnGround())
			{
				continue;
			}
			const FVector N = Support.Value.ImpactNormal.GetSafeNormal();
			FVector SweepStart = FVector::ZeroVector;
			FVector SweepEnd = FVector::ZeroVector;
			Wheel->GetSuspensionSweepSegment(delta, SweepStart, SweepEnd);
			const float Radius = Wheel->GetCollisionShape().GetSphereRadius();
			const float SignedCenterDistance = FVector::DotProduct(
				SweepEnd - Support.Value.ImpactPoint, N);
			const float ReachGap = SignedCenterDistance - Radius;
			if (ReachGap <= 0.01f)
			{
				SHitResult RetainedHit = Support.Value;
				RetainedHit.Location = SweepEnd - ReachGap * N;
				RetainedHit.ImpactPoint = RetainedHit.Location - Radius * N;
				RetainedHit.PenetrationDepth = FMath::Max(0.0f, -ReachGap);
				Wheel->SetHit(RetainedHit);
				Wheel->SetOnGround(true);
				NotifyWheelOnGroundStateChanged();
			}
		}
		EndDeferredWheelGroundStateUpdate();
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
