#include "ISpeedWheeledComponent.h"
#include "IAmSpeed/SubBodies/Solid/BoxSubBody.h"
#include "IAmSpeed/SubBodies/Solid/SWheelSubBody.h"
#include "IAmSpeed/World/Analytic/StaticWorldQueryAudit.h"
#include "HAL/IConsoleManager.h"
#include "Engine/World.h"
#include "IAmSpeed/World/Subsystem/SpeedWorldSubsystem.h"
#include "IAmSpeed/World/Analytic/AnalyticWorldData.h"

#if !(UE_BUILD_SHIPPING)
// Read-only diagnostics over the admitted world data. No contact decision uses this mapping.
static FString CoupledPoseDiagnosticVector(const FVector3d& V)
{
	return FString::Printf(TEXT("(%.17g,%.17g,%.17g)"), V.X, V.Y, V.Z);
}

static void LogCoupledPosePrimitiveDomain(UWorld* World, const int32 Frame, const int32 Wheel,
	const TCHAR* Role, const uint64 SourceId, const uint64 SurfaceId,
	const uint64 FeatureId, const uint64 PrimitiveId)
{
	const USpeedWorldSubsystem* Subsystem = World ? World->GetSubsystem<USpeedWorldSubsystem>() : nullptr;
	const Speed::Analytic::FAnalyticWorldData* Data = Subsystem ? Subsystem->GetAnalyticWorldData() : nullptr;
	if (!Data || PrimitiveId == 0)
	{
		UE_LOG(LogTemp, Log, TEXT("[CoupledPosePrimitiveDomain] Frame=%d Wheel=%d Role=%s Primitive=%016llx Status=Unknown"), Frame, Wheel, Role, PrimitiveId);
		return;
	}
	int32 Examined = 0;
	bool bMatched = false;
	constexpr int32 MaximumCandidates = 32768;
	for (const auto& Patch : Data->ExtrudedQuinticPatches)
	{
		if (Patch.SourceId != SourceId || Patch.SurfaceId != SurfaceId || Patch.FeatureId != FeatureId) continue;
		for (int32 Segment = 0; Segment + 1 < Patch.SectionPolyline.Num(); ++Segment)
		{
			if (++Examined > MaximumCandidates) break;
			if (Speed::Analytic::CombineStableIds(Patch.PrimitiveId, uint64(Segment + 1)) != PrimitiveId) continue;
			bMatched = true;
			const FVector3d A = Patch.SectionPolyline[Segment];
			const FVector3d B = Patch.SectionPolyline[Segment + 1];
			UE_LOG(LogTemp, Log, TEXT("[CoupledPosePrimitiveDomain] Frame=%d Wheel=%d Role=%s Primitive=%016llx Status=Mapped Provider=Extruded Parent=%016llx Group=%016llx Authority=%d C2=%d Segment=%d MinExtrusion=%.17g MaxExtrusion=%.17g Axis=%s A=%s B=%s ErrorCm=%.17g"),
				Frame, Wheel, Role, PrimitiveId, Patch.PrimitiveId, Patch.CanonicalGroupId,
				Patch.bAuthorityEligible ? 1 : 0, Patch.bCanonicalC2ByConstruction ? 1 : 0,
				Segment, Patch.MinimumExtrusionCoordinate, Patch.MaximumExtrusionCoordinate,
				*CoupledPoseDiagnosticVector(Patch.ExtrusionAxis), *CoupledPoseDiagnosticVector(A), *CoupledPoseDiagnosticVector(B), Patch.MaximumChordErrorCm);
		}
	}
	const auto LogTensor = [&](const TCHAR* Provider, const uint64 Parent, const uint64 Group,
		const bool Authority, const auto& Cells)
	{
		constexpr int32 Indices[2][3] = {{0,2,3},{0,3,1}};
		for (int32 CellIndex = 0; CellIndex < Cells.Num(); ++CellIndex)
		for (int32 Triangle = 0; Triangle < 2; ++Triangle)
		{
			if (++Examined > MaximumCandidates) return;
			if (Speed::Analytic::CombineStableIds(Parent, uint64(2 * CellIndex + Triangle + 1)) != PrimitiveId) continue;
			bMatched = true;
			const auto& Cell = Cells[CellIndex];
			UE_LOG(LogTemp, Log, TEXT("[CoupledPosePrimitiveDomain] Frame=%d Wheel=%d Role=%s Primitive=%016llx Status=Mapped Provider=%s Parent=%016llx Group=%016llx Authority=%d ApproxCell=%d Triangle=%d U0=%.17g U1=%.17g V0=%.17g V1=%.17g A=%s B=%s C=%s ErrorCm=%.17g"),
				Frame, Wheel, Role, PrimitiveId, Provider, Parent, Group, Authority ? 1 : 0, CellIndex, Triangle,
				Cell.MinimumU, Cell.MaximumU, Cell.MinimumV, Cell.MaximumV,
				*CoupledPoseDiagnosticVector(Cell.Corners[Indices[Triangle][0]]),
				*CoupledPoseDiagnosticVector(Cell.Corners[Indices[Triangle][1]]),
				*CoupledPoseDiagnosticVector(Cell.Corners[Indices[Triangle][2]]), Cell.MaximumErrorCm);
		}
	};
	for (const auto& Patch : Data->TensorBezierPatches)
	{
		if (Patch.SourceId == SourceId && Patch.SurfaceId == SurfaceId && Patch.FeatureId == FeatureId)
			LogTensor(TEXT("Tensor"), Patch.PrimitiveId, Patch.CanonicalGroupId, Patch.bAuthorityEligible, Patch.ApproximationCells);
	}
	for (const auto& Patch : Data->PiecewiseTensorBezierPatches)
	{
		if (Patch.SourceId != SourceId || Patch.SurfaceId != SurfaceId) continue;
		for (const auto& Cell : Patch.Cells)
		{
			if (Cell.FeatureId == FeatureId)
				LogTensor(TEXT("PiecewiseTensor"), Cell.PrimitiveId, Patch.CanonicalGroupId, Patch.bAuthorityEligible, Cell.ApproximationCells);
		}
	}
	UE_LOG(LogTemp, Log, TEXT("[CoupledPosePrimitiveDomainSummary] Frame=%d Wheel=%d Role=%s Primitive=%016llx Matched=%d Complete=%d Examined=%d WorldHash=%016llx"), Frame, Wheel, Role, PrimitiveId, bMatched ? 1 : 0, Examined <= MaximumCandidates ? 1 : 0, Examined, Data->SourceHash);
}
#endif


static TAutoConsoleVariable<int32> CVarIAmSpeedCertifiedExtrudedProjection(
	TEXT("p.IAmSpeed.WheelSupport.ProjectionCertifiedExtruded"), 1,
	TEXT("Admit bounded missed-wheel projection on same or adjacent certified C2 extruded chords, including exact duplicate profiles inside their common finite domain; zero preserves the gravity-alignment baseline."), ECVF_Default);

static bool HaveCertifiedExtrudedSupport(UWorld* World,
	const uint64 SourceId, const uint64 SurfaceId, const uint64 FeatureId,
	const uint64 PreviousPrimitiveId, const uint64 CurrentPrimitiveId,
	const FVector& PreviousPoint, const FVector& CurrentPoint, const float Radius)
{
	if (!World || PreviousPrimitiveId == 0 || CurrentPrimitiveId == 0 ||
		!Speed::Analytic::FStaticWorldQueryAudit::IsSurfaceAnalyticBackend()) return false;
	const USpeedWorldSubsystem* Subsystem = World->GetSubsystem<USpeedWorldSubsystem>();
	const auto* Data = Subsystem ? Subsystem->GetAnalyticWorldData() : nullptr;
	if (!Data) return false;
	const Speed::Analytic::FExtrudedQuinticPatch* PreviousPatch = nullptr;
	const Speed::Analytic::FExtrudedQuinticPatch* CurrentPatch = nullptr;
	int32 Previous = INDEX_NONE, Current = INDEX_NONE;
	int32 PreviousMatches = 0, CurrentMatches = 0, Examined = 0;
	for (const auto& Patch : Data->ExtrudedQuinticPatches)
	{
		if (Patch.SourceId != SourceId || Patch.SurfaceId != SurfaceId || Patch.FeatureId != FeatureId) continue;
		if (Patch.SectionPolyline.Num() < 2 || Patch.SectionPolyline.Num() > 32768) return false;
		for (int32 Segment = 0; Segment + 1 < Patch.SectionPolyline.Num(); ++Segment)
		{
			if (++Examined > 32768) return false;
			const uint64 Id = Speed::Analytic::CombineStableIds(Patch.PrimitiveId, uint64(Segment + 1));
			if (Id == PreviousPrimitiveId) { PreviousPatch = &Patch; Previous = Segment; ++PreviousMatches; }
			if (Id == CurrentPrimitiveId) { CurrentPatch = &Patch; Current = Segment; ++CurrentMatches; }
		}
	}
	if (PreviousMatches != 1 || CurrentMatches != 1 || FMath::Abs(Previous - Current) > 1) return false;
	if (!PreviousPatch->bAuthorityEligible || !CurrentPatch->bAuthorityEligible ||
		!PreviousPatch->bQueryCollisionEnabled || !CurrentPatch->bQueryCollisionEnabled ||
		!PreviousPatch->bCanonicalC2ByConstruction || !CurrentPatch->bCanonicalC2ByConstruction ||
		!PreviousPatch->IsValid() || !CurrentPatch->IsValid()) return false;
	const auto SamePoint = [](const FVector3d& A, const FVector3d& B)
	{
		return !A.ContainsNaN() && !B.ContainsNaN() && A.X == B.X && A.Y == B.Y && A.Z == B.Z;
	};
	if (PreviousPatch != CurrentPatch)
	{
		// Two admitted records may parameterize the same physical polynomial on
		// overlapping finite extrusion domains. A shared group alone is insufficient.
		if (PreviousPatch->CanonicalGroupId == 0 ||
			PreviousPatch->CanonicalGroupId != CurrentPatch->CanonicalGroupId ||
			PreviousPatch->MaterialId != CurrentPatch->MaterialId ||
			PreviousPatch->ObjectType != CurrentPatch->ObjectType ||
			PreviousPatch->BlockingChannels != CurrentPatch->BlockingChannels ||
			PreviousPatch->CanonicalSymmetryAxisMask != CurrentPatch->CanonicalSymmetryAxisMask ||
			PreviousPatch->AdditionalResidualAgreementAllowanceCm != CurrentPatch->AdditionalResidualAgreementAllowanceCm ||
			!SamePoint(PreviousPatch->ExtrusionAxis, CurrentPatch->ExtrusionAxis) ||
			PreviousPatch->SectionPolyline.Num() != CurrentPatch->SectionPolyline.Num() ||
			PreviousPatch->SectionParameters.Num() != CurrentPatch->SectionParameters.Num() ||
			PreviousPatch->MaximumChordErrorCm != CurrentPatch->MaximumChordErrorCm) return false;
		for (int32 I = 0; I < 6; ++I)
			if (!SamePoint(PreviousPatch->SectionControlPoints[I], CurrentPatch->SectionControlPoints[I])) return false;
		for (int32 I = 0; I < 2; ++I)
			if (!SamePoint(PreviousPatch->InteriorCorrectionControlPoints[I], CurrentPatch->InteriorCorrectionControlPoints[I])) return false;
		for (int32 I = 0; I < PreviousPatch->SectionPolyline.Num(); ++I)
			if (!SamePoint(PreviousPatch->SectionPolyline[I], CurrentPatch->SectionPolyline[I])) return false;
		for (int32 I = 0; I < PreviousPatch->SectionParameters.Num(); ++I)
			if (!FMath::IsFinite(PreviousPatch->SectionParameters[I]) ||
				PreviousPatch->SectionParameters[I] != CurrentPatch->SectionParameters[I]) return false;
		if (PreviousPoint.ContainsNaN() || CurrentPoint.ContainsNaN() || !FMath::IsFinite(Radius) || Radius < 0.0f) return false;
		const double CommonMinimum = FMath::Max(PreviousPatch->MinimumExtrusionCoordinate, CurrentPatch->MinimumExtrusionCoordinate);
		const double CommonMaximum = FMath::Min(PreviousPatch->MaximumExtrusionCoordinate, CurrentPatch->MaximumExtrusionCoordinate);
		const double PreviousCoordinate = FVector3d::DotProduct(PreviousPoint, PreviousPatch->ExtrusionAxis);
		const double CurrentCoordinate = FVector3d::DotProduct(CurrentPoint, CurrentPatch->ExtrusionAxis);
		// The radius-expanded segment between the two real contact points must
		// remain inside the common domain; no tolerance enlarges either boundary.
		if (CommonMaximum <= CommonMinimum ||
			FMath::Min(PreviousCoordinate, CurrentCoordinate) - Radius < CommonMinimum ||
			FMath::Max(PreviousCoordinate, CurrentCoordinate) + Radius > CommonMaximum) return false;
	}
	const FVector3d& A0 = PreviousPatch->SectionPolyline[Previous];
	const FVector3d& A1 = PreviousPatch->SectionPolyline[Previous + 1];
	const FVector3d& B0 = CurrentPatch->SectionPolyline[Current];
	const FVector3d& B1 = CurrentPatch->SectionPolyline[Current + 1];
	if (A0.ContainsNaN() || A1.ContainsNaN() || B0.ContainsNaN() || B1.ContainsNaN()) return false;
	const FVector3d N0 = FVector3d::CrossProduct(A1 - A0, PreviousPatch->ExtrusionAxis).GetSafeNormal();
	const FVector3d N1 = FVector3d::CrossProduct(B1 - B0, CurrentPatch->ExtrusionAxis).GetSafeNormal();
	return !N0.IsNearlyZero() && !N1.IsNearlyZero() && FVector3d::DotProduct(N0, N1) >= 0.995;
}


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

static TAutoConsoleVariable<int32> CVarIAmSpeedWheelContactRequireFinalSupport(
	TEXT("p.IAmSpeed.WheelContact.RequireFinalSupport"),
	0,
	TEXT("When non-zero, delayed wheel contacts participate in coupled-pose and impulse solves only while the wheel remains supported in the final integrated pose."),
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
	TEXT("Minimum support-normal alignment with world up. Established analytic vertical walls use bounded same-patch retention; gutters retain this alignment gate."),
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

static TAutoConsoleVariable<int32> CVarIAmSpeedWheelSupportProjectionPreserveProbeHits(
	TEXT("p.IAmSpeed.WheelSupport.ProjectionPreserveProbeHits"),
	0,
	TEXT("Experimental standalone projection: retain the existing inward reach margin for initial probe hits and reject actual support loss. Disabled until qualification."),
	ECVF_Default);

static TAutoConsoleVariable<int32> CVarIAmSpeedWheelSupportProjectionCoherentSpringState(
	TEXT("p.IAmSpeed.WheelSupport.ProjectionCoherentSpringState"),
	0,
	TEXT("Experimental atomic projection/contact displacement transaction. Requires PreserveProbeHits; disabled until qualification."),
	ECVF_Default);

static TAutoConsoleVariable<int32> CVarIAmSpeedWheelSupportProjectionDebug(
	TEXT("p.IAmSpeed.WheelSupport.ProjectionDebug"),
	0,
	TEXT("Logs established-support patch admission and pose projection when non-zero."),
	ECVF_Default);

static TAutoConsoleVariable<int32> CVarIAmSpeedCoupledSubBodyPoseSolver(
	TEXT("p.IAmSpeed.CoupledPose.Enabled"),
	1,
	TEXT("Experimental lexicographic component-pose solve: principal hitbox feasibility first, then current-frame wheel-patch retention."),
	ECVF_Default);

static TAutoConsoleVariable<float> CVarIAmSpeedCoupledPoseHitboxSlopCm(
	TEXT("p.IAmSpeed.CoupledPose.HitboxSlopCm"),
	0.05f,
	TEXT("Accepted principal-hitbox overlap during coupled component-pose projection. Strict analytical authority always enforces zero."),
	ECVF_Default);

static TAutoConsoleVariable<int32> CVarIAmSpeedCoupledPosePasses(
	TEXT("p.IAmSpeed.CoupledPose.Passes"),
	24,
	TEXT("Maximum deterministic active-set projection passes per integrated segment."),
	ECVF_Default);

static TAutoConsoleVariable<float> CVarIAmSpeedCoupledPoseRotationLengthCm(
	TEXT("p.IAmSpeed.CoupledPose.RotationLengthCm"),
	50.0f,
	TEXT("Generic pose-projection characteristic length for hitbox, wall, and gutter contacts."),
	ECVF_Default);

static TAutoConsoleVariable<float> CVarIAmSpeedCoupledPoseFloorPivotRotationLengthCm(
	TEXT("p.IAmSpeed.CoupledPose.FloorPivotRotationLengthCm"),
	100.0f,
	TEXT("Pose-projection characteristic length for a gravity-aligned floor pivot or lateral wheel axle."),
	ECVF_Default);

static TAutoConsoleVariable<float> CVarIAmSpeedCoupledPoseWheelManifoldRotationLengthCm(
	TEXT("p.IAmSpeed.CoupledPose.WheelManifoldRotationLengthCm"),
	60.0f,
	TEXT("Pose-projection characteristic length while preserving a wheel manifold other than a lateral axle."),
	ECVF_Default);

static TAutoConsoleVariable<float> CVarIAmSpeedCoupledPoseWheelGapCm(
	TEXT("p.IAmSpeed.CoupledPose.WheelGapCm"),
	0.05f,
	TEXT("Maximum retained wheel-patch separation after hitbox-feasible projection."),
	ECVF_Default);

static TAutoConsoleVariable<int32> CVarIAmSpeedCoupledPoseSharedWallPlaneRetention(
	TEXT("p.IAmSpeed.CoupledPose.SharedWallPlaneRetention"),
	0,
	TEXT("Experimental final simultaneous retention on one real static wall plane. Disabled until matrix and player qualification."),
	ECVF_Default);

static TAutoConsoleVariable<int32> CVarIAmSpeedCoupledPoseDebug(
	TEXT("p.IAmSpeed.CoupledPose.Debug"),
	0,
	TEXT("Logs coupled hitbox/wheel pose transactions when non-zero."),
	ECVF_Default);

static TAutoConsoleVariable<int32> CVarIAmSpeedCoupledPoseVelocityCorrection(
	TEXT("p.IAmSpeed.CoupledPose.VelocityCorrection"),
	0,
	TEXT("When non-zero, removes inward principal-hitbox point velocity after an accepted pose projection. Kept separate for energy A/B validation."),
	ECVF_Default);

static TAutoConsoleVariable<int32> CVarIAmSpeedCoupledPoseSupportedSurfaceContactPoint(
	TEXT("p.IAmSpeed.CoupledPose.SupportedSurfaceContactPoint"),
	0,
	TEXT("When non-zero, principal-hitbox feasibility on an established subordinate support manifold applies at the contact point in every orientation, allowing the coupled pose to rotate instead of translating only."),
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

float ISpeedWheeledComponent::GetCanonicalWheelSupportCompression(
	const USWheelSubBody& Wheel) const
{
	return Wheel.StaticSpringCompression();
}

void ISpeedWheeledComponent::ResolveGroupedWheelGroundContacts(const float& delta)
{
	TArray<SWheelGroundContact>& PendingWheelGroundContacts = GetPendingWheelContacts();
	if (PendingWheelGroundContacts.Num() == 0) return;

	const float dt = FMath::Max(delta, 1e-6f);

	// A response faster than one simulation step is not representable by this
	// first-order solver. Keep tuning in seconds while bounding its discrete form.
	const float NormalVelTimeConstant = FMath::Max(
		GetWheelContactNormalVelocityTimeConstant(), dt);
	const float gamma = 1.f - FMath::Exp(-dt / NormalVelTimeConstant);
	const float LockedSeparatingNormalVelTimeConstant = FMath::Max(CVarIAmSpeedWheelContactLockedSeparatingNormalVelTimeConstant.GetValueOnAnyThread(), 1e-6f);
	const float LockedSeparatingGamma = 1.f - FMath::Exp(-dt / LockedSeparatingNormalVelTimeConstant);
	const float LockedSeparatingMaxNormalVelocity = FMath::Max(CVarIAmSpeedWheelContactLockedSeparatingMaxNormalVelocity.GetValueOnAnyThread(), 0.0f);
	const float MaxInwardNormalVelocityToSolve = FMath::Max(
		GetWheelContactMaxInwardNormalVelocityToSolve(), 0.0f);
	const bool bLockedSeparatingDampingEnabled =
		CVarIAmSpeedWheelContactLockedSeparatingDamping.GetValueOnAnyThread() != 0;
	// --- Deadzones ---
	const float VNDeadzone = FMath::Max(GetWheelContactNormalVelocityDeadzone(), 0.0f);
	const float JDeadzone = FMath::Max(CVarIAmSpeedWheelContactImpulseDeadzone.GetValueOnAnyThread(), 0.0f);
	// (smaller are those, more network stability)

	const float Softness = FMath::Max(
		CVarIAmSpeedWheelContactSoftness.GetValueOnAnyThread(), 0.0f);

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
		const SWheelGroundContact& Contact =
			PendingWheelGroundContacts[ContactIndex];
		const FVector ContactNormal = Contact.Normal.GetSafeNormal();
		// A delayed analytic contact on the transverse upper cage curve can
		// outlive the wheel support that produced it. Applying its compression-
		// crossing impulse after the transactional pose has already detached the
		// wheel creates a response bifurcation. Keep the rejection bounded to
		// the transverse upper-curve regime; lower gutters and legacy contacts
		// retain their existing delayed-contact behaviour.
		const bool bDetachedAnalyticUpperCurveContact =
			Speed::Analytic::FStaticWorldQueryAudit::IsSurfaceAnalyticBackend() &&
			Contact.SpringDisplacement <= -6.0f &&
			FMath::Abs(ContactNormal.X) <= 0.10f &&
			FMath::Abs(ContactNormal.Y) >= 0.40f &&
			ContactNormal.Z <= -0.20f;
		const bool bRequireFinalSupport =
			CVarIAmSpeedWheelContactRequireFinalSupport.GetValueOnAnyThread() != 0 ||
			bDetachedAnalyticUpperCurveContact;
		if (bRequireFinalSupport &&
			(!Contact.Wheel || !Contact.Wheel->IsOnGround()))
		{
#if !(UE_BUILD_SHIPPING)
			if (CVarIAmSpeedWheelContactDebug.GetValueOnAnyThread() != 0)
			{
				UE_LOG(LogTemp, Log,
					TEXT("[WheelContactSolver] Mode=FinalSupportRejected Frame=%d Wheel=%d SpringDisp=%.3f SurfaceSource=%llu Surface=%llu Feature=%llu Pos=%s Normal=%s"),
					NumFrame(),
					Contact.Wheel ? Contact.Wheel->Idx() : INDEX_NONE,
					Contact.SpringDisplacement,
					Contact.SurfaceSourceId,
					Contact.SurfaceId,
					Contact.SurfaceFeatureId,
					*Contact.WorldPos.ToString(),
					*Contact.Normal.ToString());
			}
#endif
			continue;
		}
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

	auto EvaluatePose = [this, &Wheels, &PlaneNormal, PlaneOffset, &Forward, &Right, &PlaneRotation](
		const double Pitch, const double Roll, TArray<double>& RequiredOriginOffsets)
	{
		const FQuat Rotation =
			FQuat(Right, static_cast<float>(Pitch)) *
			FQuat(Forward, static_cast<float>(Roll)) * PlaneRotation;
		RequiredOriginOffsets.Reset(Wheels.Num());
		for (const USWheelSubBody* Wheel : Wheels)
		{
			const FVector StaticWheelLocal = Wheel->GetLocalOffset() +
				GetCanonicalWheelSupportCompression(*Wheel) * FVector::UpVector;
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
		Wheel->SetLastDisplacement(GetCanonicalWheelSupportCompression(*Wheel));
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

bool ISpeedWheeledComponent::HasActivePhysicalConstraintsOtherThan(const USSubBody* Source) const
{
	if (ISpeedComponent::HasActivePhysicalConstraintsOtherThan(Source)) return true;
	// A swept contact awaiting the grouped solve still owns a response even
	// if a later pose probe temporarily cleared the wheel's on-ground flag.
	for (const SWheelGroundContact& Contact : GetPendingWheelContacts())
		if (Contact.Wheel && Contact.Wheel != Source && Contact.SurfaceComponent.IsValid()) return true;
	for (const USWheelSubBody* Wheel : GetWheelSubBodies())
		if (Wheel && Wheel != Source && Wheel->IsOnGround()) return true;
	return false;
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

bool ISpeedWheeledComponent::HasCompatibleEstablishedStaticSupport(
	const SHitResult& SurfaceHit) const
{
	UPrimitiveComponent* SurfaceComponent = SurfaceHit.Component.Get();
	if (!SurfaceComponent ||
		SurfaceComponent->Mobility != EComponentMobility::Static)
	{
		return false;
	}

	const bool bHasAnalyticIdentity = SurfaceHit.SourceId != 0 &&
		SurfaceHit.SurfaceId != 0 && SurfaceHit.FeatureId != 0;
	int32 CompatibleSupports = 0;
	for (const TObjectPtr<USWheelSubBody>& WheelPtr : GetWheelSubBodies())
	{
		const USWheelSubBody* Wheel = WheelPtr.Get();
		if (!Wheel || !Wheel->IsOnGround())
		{
			continue;
		}
		const SHitResult& WheelHit = Wheel->GetHit();
		if (WheelHit.Component.Get() != SurfaceComponent ||
			WheelHit.ImpactNormal.IsNearlyZero())
		{
			continue;
		}
		const bool bCompatibleIdentity = bHasAnalyticIdentity
			? WheelHit.SourceId == SurfaceHit.SourceId &&
				WheelHit.SurfaceId == SurfaceHit.SurfaceId &&
				WheelHit.FeatureId == SurfaceHit.FeatureId
			: (SurfaceHit.FaceIndex == INDEX_NONE ||
				WheelHit.FaceIndex == INDEX_NONE ||
				WheelHit.FaceIndex == SurfaceHit.FaceIndex);
		CompatibleSupports += bCompatibleIdentity ? 1 : 0;
	}
	// One point is not a support manifold and must not soften an ordinary
	// aerial chassis impact.  An axle or broader set is sufficient.
	return CompatibleSupports >= 2;
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
			float OriginalDisplacement = 0.0f;
#if !(UE_BUILD_SHIPPING)
			FVector DiagnosticInitialEnd = FVector::ZeroVector;
			SHitResult DiagnosticInitialPlane;
#endif
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
			Probe.OriginalDisplacement = Wheel->GetLastDisplacement();
			Probe.bHasProbeHit = Wheel->ProbeSuspensionOnGround(Probe.ProbeHit, delta);
#if !(UE_BUILD_SHIPPING)
			if (CVarIAmSpeedWheelSupportProjectionDebug.GetValueOnAnyThread() >= 3)
			{
				FVector DiagnosticStart;
				Wheel->GetSuspensionSweepSegment(delta, DiagnosticStart, Probe.DiagnosticInitialEnd);
				Probe.DiagnosticInitialPlane = Probe.bHasProbeHit ? Probe.ProbeHit : Probe.PreviousHit;
			}
#endif
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
			const bool bNativeVariableNormalSupport =
				Speed::Analytic::FStaticWorldQueryAudit::IsSurfaceAnalyticBackend() &&
				Probe.PreviousHit.bSurfaceNormalMayVary &&
				Probe.PreviousHit.SourceId != 0 &&
				Probe.PreviousHit.SurfaceId != 0 &&
				Probe.PreviousHit.CanonicalGroupId != 0 &&
				PreviousNormal.Z <= -0.995f;
			// A wall contact can exhaust its ordinary suspension sweep while the
			// same authored patch remains within the bounded correction budget.
			// Admit only an established analytic wall identity, then reacquire it
			// locally below; a shared stadium component alone is insufficient.
			const bool bNativeStaticWallSupport =
				Speed::Analytic::FStaticWorldQueryAudit::IsSurfaceAnalyticBackend() &&
				Probe.PreviousHit.SourceId != 0 &&
				Probe.PreviousHit.SurfaceId != 0 &&
				Probe.PreviousHit.FeatureId != 0 &&
				FMath::Abs(PreviousNormal.Z) <= 0.10f;
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
				if (CVarIAmSpeedWheelSupportProjectionDebug.GetValueOnAnyThread() >= 2)
				{
					UE_LOG(LogTemp, Log,
						TEXT("[WheelSupportProjectionIdentity] Frame=%d Wheel=%d Source=%016llx Surface=%016llx Feature=%016llx Primitive=%016llx Group=%016llx Component=%s Hit=%d HitFrame=%u Point=(%.17g,%.17g,%.17g) Normal=(%.17g,%.17g,%.17g)"),
						NumFrame(), Index,
						static_cast<unsigned long long>(Probe.PreviousHit.SourceId),
						static_cast<unsigned long long>(Probe.PreviousHit.SurfaceId),
						static_cast<unsigned long long>(Probe.PreviousHit.FeatureId),
						static_cast<unsigned long long>(Probe.PreviousHit.PrimitiveId),
						static_cast<unsigned long long>(Probe.PreviousHit.CanonicalGroupId),
						PreviousSurface ? *PreviousSurface->GetPathName() : TEXT("None"),
						Probe.PreviousHit.bHit ? 1 : 0, Probe.PreviousHit.FrameTag,
						Probe.PreviousHit.ImpactPoint.X, Probe.PreviousHit.ImpactPoint.Y, Probe.PreviousHit.ImpactPoint.Z,
						PreviousNormal.X, PreviousNormal.Y, PreviousNormal.Z);
				}
			}
#endif
			if (!Probe.bWasGrounded || Probe.bHasProbeHit || !PreviousSurface ||
				PreviousSurface->Mobility != EComponentMobility::Static ||
				PreviousNormal.IsNearlyZero() ||
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
			const bool bNeedsCertifiedSmoothSupport = PreviousNormal.Z < MinGravityAlignment &&
				!bNativeVariableNormalSupport && !bNativeStaticWallSupport;
			// A smooth gutter is not a gravity-aligned floor. Only an established,
			// currently swept finite certified patch may bypass that admission gate.
			// Unknown geometry, creases and remote chords retain the original rule.
			if (bNeedsCertifiedSmoothSupport &&
				(!bHasLocalPatch || CVarIAmSpeedCertifiedExtrudedProjection.GetValueOnAnyThread() == 0 ||
				 !HaveCertifiedExtrudedSupport(Wheel->GetWorld(),
					Probe.PreviousHit.SourceId, Probe.PreviousHit.SurfaceId, Probe.PreviousHit.FeatureId,
					Probe.PreviousHit.PrimitiveId, LocalPatchHit.PrimitiveId,
					Probe.PreviousHit.ImpactPoint, LocalPatchHit.ImpactPoint,
					Wheel->GetCollisionShape().GetSphereRadius()) ||
				 LocalPatchHit.SourceId != Probe.PreviousHit.SourceId ||
				 LocalPatchHit.SurfaceId != Probe.PreviousHit.SurfaceId ||
				 LocalPatchHit.FeatureId != Probe.PreviousHit.FeatureId))
			{
#if !(UE_BUILD_SHIPPING)
				// Existing debug mode 2 identifies the rejected real local sweep.
				// Published-domain logging is read-only and does not admit contact.
				if (CVarIAmSpeedWheelSupportProjectionDebug.GetValueOnAnyThread() >= 2 &&
					CVarIAmSpeedCertifiedExtrudedProjection.GetValueOnAnyThread() != 0)
				{
					UE_LOG(LogTemp, Log,
						TEXT("[WheelSupportCertifiedProjectionRejected] Frame=%d Wheel=%d LocalHit=%d PreviousSource=%016llx LocalSource=%016llx PreviousSurface=%016llx LocalSurface=%016llx PreviousFeature=%016llx LocalFeature=%016llx PreviousPrimitive=%016llx LocalPrimitive=%016llx PreviousGroup=%016llx LocalGroup=%016llx PreviousPoint=%s LocalPoint=%s"),
						NumFrame(), Index, bHasLocalPatch ? 1 : 0,
						Probe.PreviousHit.SourceId, LocalPatchHit.SourceId,
						Probe.PreviousHit.SurfaceId, LocalPatchHit.SurfaceId,
						Probe.PreviousHit.FeatureId, LocalPatchHit.FeatureId,
						Probe.PreviousHit.PrimitiveId, LocalPatchHit.PrimitiveId,
						Probe.PreviousHit.CanonicalGroupId, LocalPatchHit.CanonicalGroupId,
						*Probe.PreviousHit.ImpactPoint.ToString(), *LocalPatchHit.ImpactPoint.ToString());
					if (bHasLocalPatch)
					{
						LogCoupledPosePrimitiveDomain(Wheel->GetWorld(), NumFrame(), Index, TEXT("ProjectionPrevious"),
							Probe.PreviousHit.SourceId, Probe.PreviousHit.SurfaceId, Probe.PreviousHit.FeatureId, Probe.PreviousHit.PrimitiveId);
						LogCoupledPosePrimitiveDomain(Wheel->GetWorld(), NumFrame(), Index, TEXT("ProjectionLocal"),
							LocalPatchHit.SourceId, LocalPatchHit.SurfaceId, LocalPatchHit.FeatureId, LocalPatchHit.PrimitiveId);
					}
				}
#endif
				continue;
			}
			const FVector LocalNormal = LocalPatchHit.ImpactNormal.GetSafeNormal();
			const FVector PatchTravel = LocalPatchHit.ImpactPoint - Probe.PreviousHit.ImpactPoint;
			const float TangentialPatchTravel = FVector::VectorPlaneProject(
				PatchTravel, PreviousNormal).Size();
			const float PredictedPatchTravel =
				GetPhysVelocityAtPoint(Probe.PreviousHit.ImpactPoint).Size() * delta;
			const float MaxPatchTravel = PredictedPatchTravel + PatchTravelSlack;
			const bool bSameWallIdentity = !bNativeStaticWallSupport ||
				(LocalPatchHit.SourceId == Probe.PreviousHit.SourceId &&
					LocalPatchHit.SurfaceId == Probe.PreviousHit.SurfaceId &&
					LocalPatchHit.FeatureId == Probe.PreviousHit.FeatureId);
			const bool bSameLocalPatch = bHasLocalPatch && bSameWallIdentity &&
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
			const bool bPreserveProbeHits =
				CVarIAmSpeedWheelSupportProjectionPreserveProbeHits.GetValueOnAnyThread() != 0;
			bool bCoherentSpringState = bPreserveProbeHits &&
				CVarIAmSpeedWheelSupportProjectionCoherentSpringState.GetValueOnAnyThread() != 0;
			for (const FGroundProbe& Probe : Probes)
			{
				if (Probe.Wheel->HasJumpUnilateralSupport()) bCoherentSpringState = false;
			}
			bool bCoherentStateValid = true;
			auto RefreshContactDisplacement = [bCoherentSpringState, MaxGap, &bCoherentStateValid](
				const FGroundProbe& Probe, const SHitResult& Hit)
			{
				if (!bCoherentSpringState) return;
				if (Hit.Location.ContainsNaN() || !FMath::IsFinite(Probe.OriginalDisplacement))
				{
					bCoherentStateValid = false;
					return;
				}
				const float Displacement = Probe.Wheel->ContactSpringDisplacement(Hit);
				if (!FMath::IsFinite(Displacement) || !FMath::IsFinite(Probe.OriginalDisplacement) ||
					FMath::Abs(Displacement - Probe.OriginalDisplacement) > MaxGap)
				{
					bCoherentStateValid = false;
					return;
				}
				// Provisional stored displacement only: no force simulation or support publication.
				Probe.Wheel->SetLastDisplacement(Displacement);
			};
			const int32 Passes = FMath::Clamp(
				CVarIAmSpeedWheelSupportProjectionPasses.GetValueOnAnyThread(), 1, 32);

#if !(UE_BUILD_SHIPPING)
			// Bounded read-only solver residuals; emit only when an existing final
			// actual query rejects the proposed pose. No diagnostic contact queries.
			struct FProjectionPassResidual
			{
				int32 Pass, Wheel;
				bool bInitialHit;
				double ReachGap, Clearance, ReachViolation;
			};
			TArray<FProjectionPassResidual, TInlineAllocator<128>> DiagnosticResiduals;
			bool bDiagnosticResidualsTruncated = false;
			const bool bRecordResiduals = bPreserveProbeHits &&
				CVarIAmSpeedWheelSupportProjectionDebug.GetValueOnAnyThread() >= 4;
#endif

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
					if (bCoherentSpringState)
					{
						// A newly reacquired wheel participates in the same displacement and
						// clearance solve as an initial hit. Use a real current query only.
						SHitResult AcquiredHit;
						if (Probe.Wheel->ProbeSuspensionOnGround(AcquiredHit, delta) &&
							!AcquiredHit.Location.ContainsNaN() && !AcquiredHit.ImpactPoint.ContainsNaN() &&
							!AcquiredHit.ImpactNormal.ContainsNaN() &&
							AcquiredHit.SourceId == Probe.PreviousHit.SourceId &&
							AcquiredHit.SurfaceId == Probe.PreviousHit.SurfaceId &&
							FVector::DotProduct(AcquiredHit.ImpactNormal.GetSafeNormal(), N) >= 0.9f)
						{
							RefreshContactDisplacement(Probe, AcquiredHit);
							const float Clearance = FVector::DotProduct(
								Probe.Wheel->WorldPos() - AcquiredHit.ImpactPoint, N) - Radius;
							ApplyConstraint(N, Probe.Wheel->WorldPos(), -Clearance);
						}
					}
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
					// Use the existing inward reach target for each initial real hit too.
					// A small positive solver residual must not strand an acquired wheel.
					ApplyConstraint(-N, SweepEnd, ReachGap + (bPreserveProbeHits ? ReachSkin : 0.0f));

					RefreshContactDisplacement(Probe, Probe.ProbeHit);
					const float Clearance = FVector::DotProduct(
						Probe.Wheel->WorldPos() - Probe.ProbeHit.ImpactPoint, N) - Radius;
					ApplyConstraint(N, Probe.Wheel->WorldPos(), -Clearance);
				}
#if !(UE_BUILD_SHIPPING)
				if (bRecordResiduals)
				{
					for (int32 Index = 0; Index < Probes.Num(); ++Index)
					{
						const FGroundProbe& Probe = Probes[Index];
						if (!Probe.bHasProbeHit && !EstablishedMisses.Contains(Index))
						{
							continue;
						}
						if (DiagnosticResiduals.Num() >= 128)
						{
							bDiagnosticResidualsTruncated = true;
							continue;
						}
						const SHitResult& Plane = Probe.bHasProbeHit ? Probe.ProbeHit : Probe.PreviousHit;
						const FVector N = Plane.ImpactNormal.GetSafeNormal();
						FVector Start, End;
						Probe.Wheel->GetSuspensionSweepSegment(delta, Start, End);
						const double Radius = Probe.Wheel->GetCollisionShape().GetSphereRadius();
						const double Gap = FVector::DotProduct(End - Plane.ImpactPoint, N) - Radius;
						const double Clearance = FVector::DotProduct(Probe.Wheel->WorldPos() - Plane.ImpactPoint, N) - Radius;
						DiagnosticResiduals.Add({Pass + 1, Probe.Wheel->Idx(), Probe.bHasProbeHit,
							Gap, Clearance, Gap + static_cast<double>(ReachSkin)});
					}
				}
#endif
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
			bool bPreservedProbeHits = true;
			if (bPreserveProbeHits && bWithinBounds)
			{
				// Acceptance uses actual final queries, never a synthetic retained hit.
				// Keep the entire pose transaction or restore the original pose.
				for (int32 ProbeIndex = 0; ProbeIndex < Probes.Num(); ++ProbeIndex)
				{
					const FGroundProbe& Probe = Probes[ProbeIndex];
					const bool bAcquiredConstraint = bCoherentSpringState &&
						EstablishedMisses.Contains(ProbeIndex);
					if (!Probe.bHasProbeHit && !bAcquiredConstraint)
					{
						continue;
					}
					SHitResult VerificationHit;
					if (!Probe.Wheel->ProbeSuspensionOnGround(VerificationHit, delta))
					{
#if !(UE_BUILD_SHIPPING)
						if (Probe.bHasProbeHit && CVarIAmSpeedWheelSupportProjectionDebug.GetValueOnAnyThread() >= 3)
						{
							// The already-executed actual query rejected this proposed pose.
							// Record it before rollback; never publish the failed hit.
							FVector ProposedStart, ProposedEnd;
							Probe.Wheel->GetSuspensionSweepSegment(delta, ProposedStart, ProposedEnd);
							const FVector InitialNormal = Probe.ProbeHit.ImpactNormal.GetSafeNormal();
							const double Radius = Probe.Wheel->GetCollisionShape().GetSphereRadius();
							const double ProposedGap = FVector::DotProduct(ProposedEnd - Probe.ProbeHit.ImpactPoint, InitialNormal) - Radius;
							UE_LOG(LogTemp, Log,
								TEXT("[WheelSupportProjectionRejectedProbe] ComponentFrame=%u Wheel=%d InitialHit=1 ProposedActualHit=0 InitialSource=%016llx ProposedGapCm=%.17g RadiusCm=%.17g ProposedStart=%s ProposedEnd=%s InitialPlanePoint=%s InitialPlaneNormal=%s OriginalCOM=%s ProposedCOM=%s ProposedRotationDeg=%.17g"),
								NumFrame(), Probe.Wheel->Idx(), static_cast<unsigned long long>(Probe.ProbeHit.SourceId),
								ProposedGap, Radius, *CoupledPoseDiagnosticVector(ProposedStart), *CoupledPoseDiagnosticVector(ProposedEnd),
								*CoupledPoseDiagnosticVector(Probe.ProbeHit.ImpactPoint), *CoupledPoseDiagnosticVector(InitialNormal),
								*CoupledPoseDiagnosticVector(OriginalCOM), *CoupledPoseDiagnosticVector(GetPhysCOM()),
								static_cast<double>(FMath::RadiansToDegrees(RotationAngle)));
						}
#endif
#if !(UE_BUILD_SHIPPING)
						if (bRecordResiduals)
						{
							for (const FProjectionPassResidual& Row : DiagnosticResiduals)
							{
								UE_LOG(LogTemp, Log,
									TEXT("[WheelSupportProjectionPassResidual] ComponentFrame=%u RejectedWheel=%d Pass=%d Wheel=%d InitialHit=%d ReachGapCm=%.17g ClearanceCm=%.17g ReachViolationCm=%.17g Truncated=%d"),
									NumFrame(), Probe.Wheel->Idx(), Row.Pass, Row.Wheel, Row.bInitialHit ? 1 : 0,
									Row.ReachGap, Row.Clearance, Row.ReachViolation, bDiagnosticResidualsTruncated ? 1 : 0);
							}
						}
#endif
						bPreservedProbeHits = false;
						break;
					}
					if (bAcquiredConstraint &&
						(VerificationHit.SourceId != Probe.PreviousHit.SourceId ||
						 VerificationHit.SurfaceId != Probe.PreviousHit.SurfaceId ||
						 FVector::DotProduct(VerificationHit.ImpactNormal.GetSafeNormal(),
							 Probe.PreviousHit.ImpactNormal.GetSafeNormal()) < 0.9f))
					{
						bPreservedProbeHits = false;
						break;
					}
					RefreshContactDisplacement(Probe, VerificationHit);
				}
			}
			if (bCoherentSpringState && bWithinBounds && bPreservedProbeHits && bCoherentStateValid)
			{
				// Refreshing displacement is part of the transaction. Validate the final
				// stored state, including reacquired wheels, before any support publication.
				for (int32 ProbeIndex = 0; ProbeIndex < Probes.Num(); ++ProbeIndex)
				{
					const FGroundProbe& Probe = Probes[ProbeIndex];
					if (!Probe.bHasProbeHit && !EstablishedMisses.Contains(ProbeIndex)) continue;
					SHitResult FinalStateHit;
					const SHitResult& ExpectedHit = Probe.bHasProbeHit ? Probe.ProbeHit : Probe.PreviousHit;
					if (!Probe.Wheel->ProbeSuspensionOnGround(FinalStateHit, delta) ||
						FinalStateHit.SourceId != ExpectedHit.SourceId ||
						FinalStateHit.SurfaceId != ExpectedHit.SurfaceId ||
						FinalStateHit.Location.ContainsNaN() || FinalStateHit.ImpactPoint.ContainsNaN() ||
						FinalStateHit.ImpactNormal.ContainsNaN() ||
						FVector::DotProduct(FinalStateHit.ImpactNormal.GetSafeNormal(),
							ExpectedHit.ImpactNormal.GetSafeNormal()) < 0.9f)
					{
						bCoherentStateValid = false;
						break;
					}
					const FVector N = FinalStateHit.ImpactNormal.GetSafeNormal();
					const float Penetration = FVector::DotProduct(
						FinalStateHit.ImpactPoint + Probe.Wheel->Radius() * N - Probe.Wheel->WorldPos(), N);
					if (!FMath::IsFinite(Penetration) ||
						(Probe.Wheel->IsAtSuspensionBumpStop() && Penetration > 0.01f))
					{
						bCoherentStateValid = false;
						break;
					}
				}
			}
			const bool bAcceptProjectedPose = bWithinBounds && bPreservedProbeHits && bCoherentStateValid;
			if (!bAcceptProjectedPose)
			{
				SetPhysCOMLocation(OriginalCOM);
				SetPhysRotation(OriginalRotation);
				if (bCoherentSpringState)
				{
					for (const FGroundProbe& Probe : Probes)
						Probe.Wheel->SetLastDisplacement(Probe.OriginalDisplacement);
				}
				UpdateSubBodiesKinematics();
			}

#if !(UE_BUILD_SHIPPING)
			if (CVarIAmSpeedWheelSupportProjectionDebug.GetValueOnAnyThread() != 0)
			{
				UE_LOG(LogTemp, Log,
					TEXT("[WheelSupportProjection] Frame=%d Retained=%d Applied=%d Translation=%.3f RotationDeg=%.3f CoherentSpring=%d CoherentStateValid=%d"),
					NumFrame(), EstablishedMisses.Num(), bAcceptProjectedPose ? 1 : 0,
					(GetPhysCOM() - OriginalCOM).Size(),
					FMath::RadiansToDegrees(RotationAngle), bCoherentSpringState ? 1 : 0, bCoherentStateValid ? 1 : 0);
				if (CVarIAmSpeedWheelSupportProjectionDebug.GetValueOnAnyThread() >= 3)
				{
					// Read-only witnesses for all original probes, including initial hits.
					// Do not publish these results or manufacture wheel support.
					for (const FGroundProbe& Probe : Probes)
					{
						FVector AfterStart, AfterEnd;
						Probe.Wheel->GetSuspensionSweepSegment(delta, AfterStart, AfterEnd);
						SHitResult AfterHit;
						const bool bActualAfterHit = Probe.Wheel->ProbeSuspensionOnGround(AfterHit, delta);
						const SHitResult& Plane = Probe.DiagnosticInitialPlane;
						const FVector N = Plane.ImpactNormal.GetSafeNormal();
						const double Radius = Probe.Wheel->GetCollisionShape().GetSphereRadius();
						const double BeforeGap = FVector::DotProduct(Probe.DiagnosticInitialEnd - Plane.ImpactPoint, N) - Radius;
						const double AfterGap = FVector::DotProduct(AfterEnd - Plane.ImpactPoint, N) - Radius;
						UE_LOG(LogTemp, Log,
							TEXT("[WheelSupportProjectionTransaction] ComponentFrame=%u Wheel=%d WasGrounded=%d BeforeHit=%d AfterHit=%d WithinBounds=%d MissConstraints=%d BeforeSource=%016llx AfterSource=%016llx BeforeGapCm=%.17g AfterGapCm=%.17g RadiusCm=%.17g BeforeEnd=%s AfterEnd=%s PlanePoint=%s PlaneNormal=%s BeforeCOM=%s AfterCOM=%s RotationDeg=%.17g AcceptedPose=%d ProposedRotationDeg=%.17g"),
							NumFrame(), Probe.Wheel->Idx(), Probe.bWasGrounded ? 1 : 0, Probe.bHasProbeHit ? 1 : 0,
							bActualAfterHit ? 1 : 0, bWithinBounds ? 1 : 0, EstablishedMisses.Num(),
							static_cast<unsigned long long>(Plane.SourceId),
							static_cast<unsigned long long>(bActualAfterHit ? AfterHit.SourceId : 0), BeforeGap, AfterGap, Radius,
							*CoupledPoseDiagnosticVector(Probe.DiagnosticInitialEnd), *CoupledPoseDiagnosticVector(AfterEnd),
							*CoupledPoseDiagnosticVector(Plane.ImpactPoint), *CoupledPoseDiagnosticVector(N),
							*CoupledPoseDiagnosticVector(OriginalCOM), *CoupledPoseDiagnosticVector(GetPhysCOM()),
							static_cast<double>(bAcceptProjectedPose ? FMath::RadiansToDegrees(RotationAngle) : 0.0f),
							bAcceptProjectedPose ? 1 : 0, static_cast<double>(FMath::RadiansToDegrees(RotationAngle)));
					}
				}
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
	ProjectCoupledSubBodyPose(delta);
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
	// Contact impulses are still resolved first. A changed pose, velocity or
	// force revokes sleep and takes the full feasibility-projection path.
	if (IsPhysicsSleeping()) return;
	ProjectWheelSupportNonPenetration();
	TryProjectCanonicalWheelSupportPose();
	// Wheel and canonical-support corrections are allowed to move the rigid
	// pose after integration. Reassert the coupled feasibility hierarchy with
	// an exact analytical hitbox gate last, so PostPhysics observers cannot
	// sample a residual overlap without changing substep response tolerances.
	ProjectEstablishedStaticContacts(delta);
	ProjectCoupledSubBodyPose(delta, true);
}

bool ISpeedWheeledComponent::ProjectCoupledSubBodyPose(
	const float Delta, const bool bStrictHitboxGate)
{
	if (CVarIAmSpeedCoupledSubBodyPoseSolver.GetValueOnAnyThread() == 0)
	{
		return false;
	}

	UBoxSubBody* PrincipalHitbox = nullptr;
	for (USSubBody* SubBody : GetSubBodies())
	{
		UBoxSubBody* Box = Cast<UBoxSubBody>(SubBody);
		if (Box && Box->GetSubBodyType() == USSubBody::ESubBodyType::Hitbox)
		{
			PrincipalHitbox = Box;
			break;
		}
	}
	if (!PrincipalHitbox)
	{
		return false;
	}

	const FVector TransactionCOM = GetPhysCOM();
	const FQuat TransactionRotation = GetPhysRotation();
	const float ConfiguredSlop = FMath::Max(0.0f,
		CVarIAmSpeedCoupledPoseHitboxSlopCm.GetValueOnAnyThread());
	const float Slop =
		bStrictHitboxGate &&
		Speed::Analytic::FStaticWorldQueryAudit::IsSurfaceAnalyticBackend()
			? 0.0f
			: ConfiguredSlop;
	const float ProjectionTargetSlop = FMath::Max(0.0f, Slop - 0.01f);
	TArray<USWheelSubBody*, TInlineAllocator<4>> WheelPatchCandidates;
	bool bWheelPatchesOnGravityAlignedSurface = true;
	bool bWheelPatchesFaceChassisSupportSide = true;
	const FVector ChassisUp = GetPhysUpVector().GetSafeNormal();
	for (USWheelSubBody* Wheel : GetWheelSubBodies())
	{
		if (Wheel && Wheel->IsOnGround())
		{
			WheelPatchCandidates.AddUnique(Wheel);
			const FVector PatchNormal = Wheel->GetHit().ImpactNormal.GetSafeNormal();
			bWheelPatchesOnGravityAlignedSurface =
				bWheelPatchesOnGravityAlignedSurface
				&& !PatchNormal.IsNearlyZero()
				&& FMath::Abs(PatchNormal.Z) >= 0.90f;
			bWheelPatchesFaceChassisSupportSide =
				bWheelPatchesFaceChassisSupportSide
				&& FVector::DotProduct(ChassisUp, PatchNormal) >= 0.0f;
		}
	}
	for (const SWheelGroundContact& Contact : GetPendingWheelContacts())
	{
		if (Contact.Wheel &&
			(CVarIAmSpeedWheelContactRequireFinalSupport.GetValueOnAnyThread() == 0 ||
				Contact.Wheel->IsOnGround()) &&
			Contact.SurfaceComponent.IsValid()
			&& Contact.SurfaceComponent->GetMobility() == EComponentMobility::Static
			&& !Contact.Normal.IsNearlyZero())
		{
			WheelPatchCandidates.AddUnique(Contact.Wheel);
			const FVector PatchNormal = Contact.Normal.GetSafeNormal();
			bWheelPatchesOnGravityAlignedSurface =
				bWheelPatchesOnGravityAlignedSurface
				&& FMath::Abs(PatchNormal.Z) >= 0.90f;
			bWheelPatchesFaceChassisSupportSide =
				bWheelPatchesFaceChassisSupportSide
				&& FVector::DotProduct(ChassisUp, PatchNormal) >= 0.0f;
		}
	}
	const bool bHasWheelPatchCandidate = !WheelPatchCandidates.IsEmpty();
	FVector WheelManifoldNormal = FVector::ZeroVector;
	for (const USWheelSubBody* Wheel : WheelPatchCandidates)
	{
		if (Wheel)
		{
			WheelManifoldNormal += Wheel->GetHit().ImpactNormal.GetSafeNormal();
		}
	}
	WheelManifoldNormal.Normalize();
	const FVector LocalAngularVelocity =
		GetPhysRotation().UnrotateVector(GetPhysAngularVelocity());
	const bool bPitchDominatedFreeHitboxContact = !bHasWheelPatchCandidate
		&& FMath::Abs(LocalAngularVelocity.Y) > FMath::Abs(LocalAngularVelocity.X);
	bool bWheelPatchesFormLateralAxle = false;
	if (WheelPatchCandidates.Num() == 2)
	{
		FVector SweepStartA = FVector::ZeroVector;
		FVector SweepEndA = FVector::ZeroVector;
		FVector SweepStartB = FVector::ZeroVector;
		FVector SweepEndB = FVector::ZeroVector;
		WheelPatchCandidates[0]->GetSuspensionSweepSegment(Delta, SweepStartA, SweepEndA);
		WheelPatchCandidates[1]->GetSuspensionSweepSegment(Delta, SweepStartB, SweepEndB);
		const FVector LocalPatchSpan = GetPhysRotation().UnrotateVector(
			SweepStartB - SweepStartA);
		bWheelPatchesFormLateralAxle =
			FMath::Abs(LocalPatchSpan.Y) > FMath::Abs(LocalPatchSpan.X);
	}
	const bool bUseWheelManifoldRotationLength =
		bHasWheelPatchCandidate && bWheelPatchesOnGravityAlignedSurface
		&& bWheelPatchesFaceChassisSupportSide
		&& !bWheelPatchesFormLateralAxle;
	const float DefaultRotationLength = FMath::Max(1.0f,
		CVarIAmSpeedCoupledPoseRotationLengthCm.GetValueOnAnyThread());
	const float FloorPivotRotationLength = FMath::Max(1.0f,
		CVarIAmSpeedCoupledPoseFloorPivotRotationLengthCm.GetValueOnAnyThread());
	const float WheelManifoldRotationLength = FMath::Max(1.0f,
		CVarIAmSpeedCoupledPoseWheelManifoldRotationLengthCm.GetValueOnAnyThread());
	const float WheelConstraintRotationLength =
		bWheelPatchesOnGravityAlignedSurface && bWheelPatchesFaceChassisSupportSide
			? (bUseWheelManifoldRotationLength
				? WheelManifoldRotationLength : FloorPivotRotationLength)
			: DefaultRotationLength;
	const int32 MaxPasses = FMath::Clamp(
		CVarIAmSpeedCoupledPosePasses.GetValueOnAnyThread(), 1, 64);

	struct FHitboxPlaneConstraint
	{
		TWeakObjectPtr<UPrimitiveComponent> Component;
		int32 FaceIndex = INDEX_NONE;
		FVector Point = FVector::ZeroVector;
		FVector Normal = FVector::ZeroVector;
		float ObservedDepth = 0.0f;
	};
	TArray<FHitboxPlaneConstraint, TInlineAllocator<8>> HitboxConstraints;
	TArray<FHitResult> InitialPenetrationHits;
	PrincipalHitbox->GatherStaticPenetrationHits(InitialPenetrationHits);
	float InitialMaximumDepth = 0.0f;
	for (const FHitResult& Hit : InitialPenetrationHits)
	{
		InitialMaximumDepth = FMath::Max(InitialMaximumDepth, Hit.PenetrationDepth);
	}
	if (InitialMaximumDepth <= 0.0f &&
		Speed::Analytic::FStaticWorldQueryAudit::IsSurfaceAnalyticBackend())
	{
		LastCertifiedCoupledPoseCOM = TransactionCOM;
		LastCertifiedCoupledPoseRotation = TransactionRotation;
		LastCertifiedCoupledPoseFrame = static_cast<int32>(NumFrame());
	}
	auto TryRestoreRecentCertifiedPose = [&]()
	{
		const int32 CertifiedPoseAge = static_cast<int32>(NumFrame()) -
			LastCertifiedCoupledPoseFrame;
		if (!Speed::Analytic::FStaticWorldQueryAudit::IsSurfaceAnalyticBackend() ||
			LastCertifiedCoupledPoseFrame == INDEX_NONE || CertifiedPoseAge < 0 ||
			CertifiedPoseAge > 1)
		{
			return false;
		}

		SetPhysCOMLocation(LastCertifiedCoupledPoseCOM);
		SetPhysRotation(LastCertifiedCoupledPoseRotation);
		UpdateSubBodiesKinematics();
		TArray<FHitResult> CertifiedPoseHits;
		PrincipalHitbox->GatherStaticPenetrationHits(CertifiedPoseHits);
		if (!CertifiedPoseHits.IsEmpty())
		{
			SetPhysCOMLocation(TransactionCOM);
			SetPhysRotation(TransactionRotation);
			UpdateSubBodiesKinematics();
			return false;
		}
		LastCertifiedCoupledPoseCOM = GetPhysCOM();
		LastCertifiedCoupledPoseRotation = GetPhysRotation();
		LastCertifiedCoupledPoseFrame = static_cast<int32>(NumFrame());

		BeginDeferredWheelGroundStateUpdate();
		for (USWheelSubBody* Wheel : GetWheelSubBodies())
		{
			if (!Wheel)
			{
				continue;
			}
			SHitResult FinalHit;
			const bool bOnGround = Wheel->ProbeSuspensionOnGround(FinalHit, Delta);
			if (bOnGround)
			{
				Wheel->SetHit(FinalHit);
			}
			Wheel->SetOnGround(bOnGround);
		}
		EndDeferredWheelGroundStateUpdate();
		return true;
	};
	// A pose projection can observe a residual after the dynamics response has
	// already crossed a curved analytical seam. Prefer the
	// immediately preceding exactly-free pose, and revalidate it analytically,
	// before entering the iterative solver, but only when the reported depth fits
	// inside the actual one-frame rigid-pose sweep. Re-certifying that restored
	// pose for the current frame keeps consecutive fixed-step boundary checks
	// bounded without ever accepting penetration or consuming the solver ceiling.
	const float CertifiedPoseSweepCm = LastCertifiedCoupledPoseFrame != INDEX_NONE
		? (TransactionCOM - LastCertifiedCoupledPoseCOM).Size() +
			TransactionRotation.AngularDistance(LastCertifiedCoupledPoseRotation) *
			PrincipalHitbox->GetBoxExtent().Size()
		: 0.0f;
	const bool bBoundedOneFrameRollback = CertifiedPoseSweepCm <= 15.0f &&
		InitialMaximumDepth <= 15.0f;
	if (bStrictHitboxGate && InitialMaximumDepth <= 0.0f)
	{
		// No analytical overlap exists at this boundary. Keep the strict pass a
		// true no-op so support retention and response metrics remain those of the
		// preceding dynamics solve.
		return false;
	}

	auto ApplyConstraint = [this](const FVector& Direction, const FVector& WorldPoint,
		const float Violation, const float RotationLength)
	{
		if (Violation <= 0.001f)
		{
			return;
		}
		const FVector N = Direction.GetSafeNormal();
		if (N.IsNearlyZero())
		{
			return;
		}
		const FVector AngularJacobian = FVector::CrossProduct(WorldPoint - GetPhysCOM(), N);
		const float RotationLengthSquared = RotationLength * RotationLength;
		const FVector AngularMobility = AngularJacobian / RotationLengthSquared;
		const float Denominator = 1.0f
			+ FVector::DotProduct(AngularJacobian, AngularMobility);
		const float Lambda = Violation / FMath::Max(Denominator, 1.0f);
		SetPhysCOMLocation(GetPhysCOM() + Lambda * N);
		const FVector DeltaAngular = Lambda * AngularMobility;
		const float DeltaAngle = DeltaAngular.Size();
		if (DeltaAngle > SMALL_NUMBER)
		{
			const FQuat WorldDelta(DeltaAngular / DeltaAngle, DeltaAngle);
			SetPhysRotation((WorldDelta * GetPhysRotation()).GetNormalized());
		}
		UpdateSubBodiesKinematics();
	};

	auto AddHitboxConstraints = [&HitboxConstraints](const TArray<FHitResult>& Hits)
	{
		for (const FHitResult& Hit : Hits)
		{
			FVector N = Hit.Normal.GetSafeNormal();
			if (N.IsNearlyZero())
			{
				N = Hit.ImpactNormal.GetSafeNormal();
			}
			if (N.IsNearlyZero())
			{
				continue;
			}
			const int32 ExistingIndex = HitboxConstraints.IndexOfByPredicate(
				[&Hit, &N](const FHitboxPlaneConstraint& Constraint)
				{
					return Constraint.Component == Hit.Component &&
						Constraint.FaceIndex == Hit.FaceIndex &&
						FVector::DotProduct(Constraint.Normal, N) >= 0.999f;
				});
			FHitboxPlaneConstraint Constraint;
			Constraint.Component = Hit.Component;
			Constraint.FaceIndex = Hit.FaceIndex;
			Constraint.Point = Hit.ImpactPoint;
			Constraint.Normal = N;
			Constraint.ObservedDepth = Hit.PenetrationDepth;
			if (ExistingIndex == INDEX_NONE)
			{
				HitboxConstraints.Add(Constraint);
			}
			else
			{
				HitboxConstraints[ExistingIndex] = Constraint;
			}
		}
	};
	auto SolveHitboxFeasibility = [&](const int32 PassLimit)
	{
		TArray<FHitResult> Hits;
		for (int32 Pass = 0; Pass < PassLimit; ++Pass)
		{
			PrincipalHitbox->GatherStaticPenetrationHits(Hits);
			float MaximumDepth = 0.0f;
			for (const FHitResult& Hit : Hits)
			{
				MaximumDepth = FMath::Max(MaximumDepth, Hit.PenetrationDepth);
			}
			if (MaximumDepth <= Slop)
			{
				return true;
			}
			HitboxConstraints.Reset();
			AddHitboxConstraints(Hits);
			if (HitboxConstraints.IsEmpty())
			{
				break;
			}

			for (const FHitboxPlaneConstraint& Constraint : HitboxConstraints)
			{
				// A supported rigid hitbox constraint acts at its stable impact point.
				// After a pitch-dominated floor launch, keep that angular Jacobian when
				// the wheel patch has just separated so the actor can rotate back onto
				// its axle. Curved gutter and wall contacts retain the conservative COM
				// projection; treating those free hitbox contacts as a floor pivot
				// changes their measured surface-traversal energy.
				const bool bUseFreeFloorContactPoint = !bStrictHitboxGate &&
					bPitchDominatedFreeHitboxContact
					&& FMath::Abs(Constraint.Normal.Z) >= 0.90f
					&& FVector::DotProduct(ChassisUp, Constraint.Normal) >= 0.0f;
				const bool bUseSupportedFloorContactPoint = !bStrictHitboxGate &&
					bHasWheelPatchCandidate && bWheelPatchesFaceChassisSupportSide &&
					(bWheelPatchesOnGravityAlignedSurface ||
						(!WheelManifoldNormal.IsNearlyZero() &&
							bWheelPatchesFormLateralAxle &&
							FMath::Abs(Constraint.Normal.X) <= 0.10f &&
							FMath::Abs(Constraint.Normal.Z) >= 0.90f &&
							FMath::Abs(FVector::DotProduct(WheelManifoldNormal,
								Constraint.Normal)) <= 0.25f) ||
						CVarIAmSpeedCoupledPoseSupportedSurfaceContactPoint
							.GetValueOnAnyThread() != 0);
				const float HitboxRotationLength =
					bUseFreeFloorContactPoint ? FloorPivotRotationLength
					: (bUseSupportedFloorContactPoint
						? WheelConstraintRotationLength : DefaultRotationLength);
				ApplyConstraint(Constraint.Normal,
					bUseSupportedFloorContactPoint || bUseFreeFloorContactPoint
						? Constraint.Point : GetPhysCOM(),
					Constraint.ObservedDepth - ProjectionTargetSlop +
						(bStrictHitboxGate ? 0.01f : 0.0f),
					HitboxRotationLength);
			}
		}

		TArray<FHitResult> FinalHits;
		PrincipalHitbox->GatherStaticPenetrationHits(FinalHits);
		for (const FHitResult& Hit : FinalHits)
		{
			if (Hit.PenetrationDepth > Slop)
			{
				return false;
			}
		}
		return true;
	};

	auto ClampHitboxInwardVelocity = [&]()
	{
		if (CVarIAmSpeedCoupledPoseVelocityCorrection.GetValueOnAnyThread() == 0)
		{
			return;
		}
		const float Mass = FMath::Max(GetPhysMass(), 1.0f);
		const FMatrix InvInertia = ComputeWorldInvInertiaTensor();
		for (const FHitboxPlaneConstraint& Constraint : HitboxConstraints)
		{
			const FVector N = Constraint.Normal.GetSafeNormal();
			if (N.IsNearlyZero())
			{
				continue;
			}
			const Speed::FKinematicState& State = PrincipalHitbox->GetKinematicState();
			const FVector ContactPoint = UBoxSubBody::ComputeBoxSupportPointWS(
				State.Location, State.Rotation, PrincipalHitbox->GetBoxExtent(), -N);
			const FVector R = ContactPoint - GetPhysCOM();
			const float InwardSpeed = FVector::DotProduct(
				GetPhysVelocityAtPoint(ContactPoint), N);
			if (InwardSpeed >= -0.001f)
			{
				continue;
			}
			const FVector RxN = FVector::CrossProduct(R, N);
			const FVector AngularTerm = FVector::CrossProduct(
				InvInertia.TransformVector(RxN), R);
			const float EffectiveInvMass = 1.0f / Mass
				+ FVector::DotProduct(N, AngularTerm);
			if (EffectiveInvMass > SMALL_NUMBER)
			{
				AddPhysImpulseAtPoint(
					(-InwardSpeed / EffectiveInvMass) * N, ContactPoint, PrincipalHitbox);
			}
		}
	};

	bool bHitboxFeasible = false;
	if (InitialMaximumDepth > Slop && bBoundedOneFrameRollback)
	{
		// Give the local supported manifold a small opportunity to preserve its
		// rotational response. If it cannot produce an exact analytical pose in
		// four projections, restore the transaction and use the certified CCD
		// rollback without walking the configured solver ceiling.
		bHitboxFeasible = SolveHitboxFeasibility(FMath::Min(MaxPasses, 4));
		if (!bHitboxFeasible)
		{
			SetPhysCOMLocation(TransactionCOM);
			SetPhysRotation(TransactionRotation);
			UpdateSubBodiesKinematics();
			if (TryRestoreRecentCertifiedPose())
			{
				return true;
			}
		}
	}
	else
	{
		bHitboxFeasible = SolveHitboxFeasibility(MaxPasses);
	}
	if (!bHitboxFeasible)
	{
#if !(UE_BUILD_SHIPPING)
		if (CVarIAmSpeedCoupledPoseDebug.GetValueOnAnyThread() != 0 &&
			InitialMaximumDepth > Slop)
		{
			UE_LOG(LogTemp, Warning,
				TEXT("[CoupledPose][HitboxRejected] Frame=%d InitialDepth=%.3f InitialHits=%d Constraints=%d Translation=%.3f RotationDeg=%.3f Strict=%d CertifiedAge=%d CertifiedSweep=%.6f"),
				NumFrame(), InitialMaximumDepth, InitialPenetrationHits.Num(),
				HitboxConstraints.Num(), (GetPhysCOM() - TransactionCOM).Size(),
				FMath::RadiansToDegrees(GetPhysRotation().AngularDistance(TransactionRotation)),
				bStrictHitboxGate ? 1 : 0,
				static_cast<int32>(NumFrame()) - LastCertifiedCoupledPoseFrame, CertifiedPoseSweepCm);
		}
#endif
		SetPhysCOMLocation(TransactionCOM);
		SetPhysRotation(TransactionRotation);
		UpdateSubBodiesKinematics();
		return false;
	}
	ClampHitboxInwardVelocity();
	if (bStrictHitboxGate && InitialMaximumDepth > Slop)
	{
		// The strict pass is a final analytical penetration certificate. Do not
		// let wheel retention reject and roll back an already-feasible OBB pose;
		// wheel support is solved in the preceding post-physics phases and will
		// be reprobed after this bounded positional correction.
		const bool bPoseChanged = !GetPhysCOM().Equals(TransactionCOM, 0.001f) ||
			GetPhysRotation().AngularDistance(TransactionRotation) > 1.0e-5f;
		if (bPoseChanged)
		{
			BeginDeferredWheelGroundStateUpdate();
			for (USWheelSubBody* Wheel : GetWheelSubBodies())
			{
				if (!Wheel)
				{
					continue;
				}
				SHitResult FinalHit;
				const bool bOnGround = Wheel->ProbeSuspensionOnGround(FinalHit, Delta);
				if (bOnGround)
				{
					Wheel->SetHit(FinalHit);
				}
				Wheel->SetOnGround(bOnGround);
			}
			EndDeferredWheelGroundStateUpdate();
		}
		return bPoseChanged;
	}

	const FVector HitboxFeasibleCOM = GetPhysCOM();
	const FQuat HitboxFeasibleRotation = GetPhysRotation();
	const float MaxWheelGap = FMath::Max(0.0f,
		CVarIAmSpeedCoupledPoseWheelGapCm.GetValueOnAnyThread());
	struct FWheelPatchConstraint
	{
		USWheelSubBody* Wheel = nullptr;
		TWeakObjectPtr<UPrimitiveComponent> SurfaceComponent;
		FVector SurfacePoint = FVector::ZeroVector;
		FVector Normal = FVector::ZeroVector;
		int32 SurfaceFaceIndex = INDEX_NONE;
		uint64 SurfaceSourceId = 0;
		uint64 SurfaceId = 0;
		uint64 SurfaceFeatureId = 0;
		uint64 DiagnosticPrimitiveId = 0;
	};
	TArray<FWheelPatchConstraint, TInlineAllocator<4>> WheelConstraints;
	for (USWheelSubBody* Wheel : GetWheelSubBodies())
	{
		if (!Wheel || !Wheel->IsOnGround())
		{
			continue;
		}
		const SHitResult& Hit = Wheel->GetHit();
		if (!Hit.Component.IsValid() ||
			Hit.Component->GetMobility() != EComponentMobility::Static ||
			Hit.ImpactNormal.IsNearlyZero())
		{
			continue;
		}
		FWheelPatchConstraint& Constraint = WheelConstraints.AddDefaulted_GetRef();
		Constraint.Wheel = Wheel;
		Constraint.SurfaceComponent = Hit.Component;
		Constraint.SurfacePoint = Hit.ImpactPoint;
		Constraint.Normal = Hit.ImpactNormal.GetSafeNormal();
		Constraint.SurfaceFaceIndex = Hit.FaceIndex;
		Constraint.SurfaceSourceId = Hit.SourceId;
		Constraint.SurfaceId = Hit.SurfaceId;
		Constraint.SurfaceFeatureId = Hit.FeatureId;
		Constraint.DiagnosticPrimitiveId = Hit.PrimitiveId;
	}
	for (const SWheelGroundContact& Contact : GetPendingWheelContacts())
	{
		if (!Contact.Wheel || !Contact.SurfaceComponent.IsValid() ||
			(CVarIAmSpeedWheelContactRequireFinalSupport.GetValueOnAnyThread() != 0 &&
				!Contact.Wheel->IsOnGround()) ||
			Contact.SurfaceComponent->GetMobility() != EComponentMobility::Static ||
			Contact.Normal.IsNearlyZero())
		{
			continue;
		}
		const bool bAlreadyPresent = WheelConstraints.ContainsByPredicate(
			[&Contact](const FWheelPatchConstraint& Existing)
			{
				return Existing.Wheel == Contact.Wheel;
			});
		if (!bAlreadyPresent)
		{
			FWheelPatchConstraint& Constraint = WheelConstraints.AddDefaulted_GetRef();
			Constraint.Wheel = Contact.Wheel;
			Constraint.SurfaceComponent = Contact.SurfaceComponent;
			Constraint.SurfacePoint = Contact.SurfacePoint;
			Constraint.Normal = Contact.Normal.GetSafeNormal();
			Constraint.SurfaceFaceIndex = Contact.SurfaceFaceIndex;
			Constraint.SurfaceSourceId = Contact.SurfaceSourceId;
			Constraint.SurfaceId = Contact.SurfaceId;
			Constraint.SurfaceFeatureId = Contact.SurfaceFeatureId;
			// Only report the cached primitive if it identifies this pending contact.
			const SHitResult& CachedHit = Contact.Wheel->GetHit();
			if (CachedHit.Component == Contact.SurfaceComponent &&
				CachedHit.FaceIndex == Contact.SurfaceFaceIndex &&
				CachedHit.SourceId == Contact.SurfaceSourceId &&
				CachedHit.SurfaceId == Contact.SurfaceId &&
				CachedHit.FeatureId == Contact.SurfaceFeatureId &&
				CachedHit.ImpactPoint == Contact.SurfacePoint)
			{
				Constraint.DiagnosticPrimitiveId = CachedHit.PrimitiveId;
			}
		}
	}
	WheelConstraints.Sort([](const FWheelPatchConstraint& A, const FWheelPatchConstraint& B)
	{
		return A.Wheel && B.Wheel ? A.Wheel->Idx() < B.Wheel->Idx() : A.Wheel != nullptr;
	});

	int32 RetainedWheels = 0;
	for (const FWheelPatchConstraint& Contact : WheelConstraints)
	{
		USWheelSubBody* Wheel = Contact.Wheel;
		if (!Wheel)
		{
			continue;
		}
		const FVector BeforeWheelCOM = GetPhysCOM();
		const FQuat BeforeWheelRotation = GetPhysRotation();
		const FVector N = Contact.Normal.GetSafeNormal();
		SHitResult LocalPatchHit;
		const bool bHasLocalPatch = Wheel->SweepSuspensionAlongNormal(
			N, FMath::Max(5.0f, Wheel->SuspensionMaxDrop()), Delta, LocalPatchHit);
		const bool bSameAnalyticSurface =
			Contact.SurfaceSourceId != 0 && Contact.SurfaceId != 0 &&
			Contact.SurfaceFeatureId != 0 &&
			LocalPatchHit.SourceId == Contact.SurfaceSourceId &&
			LocalPatchHit.SurfaceId == Contact.SurfaceId &&
			LocalPatchHit.FeatureId == Contact.SurfaceFeatureId;
		const bool bSameLocalFace =
			Contact.SurfaceFaceIndex == INDEX_NONE ||
			LocalPatchHit.FaceIndex == INDEX_NONE ||
			LocalPatchHit.FaceIndex == Contact.SurfaceFaceIndex;
		// A vertical authored wall or a ceiling-facing terminal bend is one support
		// surface even when its compact representation advances to an adjacent face.
		// Intermediate gutters and floor-facing ramps retain the stricter local-face
		// test so the coupled pose cannot bridge an internal bend or stale plane.
		const bool bUseBoundedAnalyticIdentity =
			Contact.SurfaceSourceId != 0 && Contact.SurfaceId != 0 &&
			Contact.SurfaceFeatureId != 0 &&
			(FMath::Abs(N.Z) <= 0.10f || N.Z <= -0.50f);
		const bool bSameSurfaceIdentity = bUseBoundedAnalyticIdentity
			? bSameAnalyticSurface
			: bSameLocalFace;
		const bool bSamePatch = bHasLocalPatch &&
			LocalPatchHit.Component == Contact.SurfaceComponent &&
			FVector::DotProduct(LocalPatchHit.ImpactNormal.GetSafeNormal(), N) >= 0.995f &&
			bSameSurfaceIdentity;
		if (!bSamePatch)
		{
#if !(UE_BUILD_SHIPPING)
			if (CVarIAmSpeedCoupledPoseDebug.GetValueOnAnyThread() != 0)
			{
				UE_LOG(LogTemp, Log,
					TEXT("[CoupledPoseWheelRetention] Frame=%d Wheel=%d Reason=LocalPatch HasPatch=%d SameComponent=%d NormalDot=%.6f AnalyticIdentity=%d SameAnalytic=%d SameFace=%d ExpectedSource=%016llx ExpectedSurface=%016llx ExpectedFeature=%016llx ActualSource=%016llx ActualSurface=%016llx ActualFeature=%016llx ExpectedFace=%d ActualFace=%d ExpectedPrimitive=%016llx ActualPrimitive=%016llx ExpectedPoint=%s ActualPoint=%s ExpectedNormal=%s ActualNormal=%s"),
					NumFrame(), Wheel->Idx(), bHasLocalPatch ? 1 : 0,
					LocalPatchHit.Component == Contact.SurfaceComponent ? 1 : 0,
					FVector::DotProduct(LocalPatchHit.ImpactNormal.GetSafeNormal(), N),
					bUseBoundedAnalyticIdentity ? 1 : 0, bSameAnalyticSurface ? 1 : 0,
					bSameLocalFace ? 1 : 0,
					static_cast<unsigned long long>(Contact.SurfaceSourceId),
					static_cast<unsigned long long>(Contact.SurfaceId),
					static_cast<unsigned long long>(Contact.SurfaceFeatureId),
					static_cast<unsigned long long>(LocalPatchHit.SourceId),
					static_cast<unsigned long long>(LocalPatchHit.SurfaceId),
					static_cast<unsigned long long>(LocalPatchHit.FeatureId),
					Contact.SurfaceFaceIndex, LocalPatchHit.FaceIndex,
					static_cast<unsigned long long>(Contact.DiagnosticPrimitiveId),
					static_cast<unsigned long long>(LocalPatchHit.PrimitiveId),
					*Contact.SurfacePoint.ToString(), *LocalPatchHit.ImpactPoint.ToString(),
					*N.ToString(), *LocalPatchHit.ImpactNormal.ToString());
				LogCoupledPosePrimitiveDomain(Wheel->GetWorld(), NumFrame(), Wheel->Idx(), TEXT("Expected"),
					Contact.SurfaceSourceId, Contact.SurfaceId, Contact.SurfaceFeatureId, Contact.DiagnosticPrimitiveId);
				LogCoupledPosePrimitiveDomain(Wheel->GetWorld(), NumFrame(), Wheel->Idx(), TEXT("Actual"),
					LocalPatchHit.SourceId, LocalPatchHit.SurfaceId, LocalPatchHit.FeatureId, LocalPatchHit.PrimitiveId);
			}
#endif
			continue;
		}

		FVector SweepStart = FVector::ZeroVector;
		FVector SweepEnd = FVector::ZeroVector;
		Wheel->GetSuspensionSweepSegment(Delta, SweepStart, SweepEnd);
		// The final state is checked with the actual suspension sweep shape.
		// A positive reach gap cannot count as retained support, even when it
		// fits the coupled-pose separation budget.
		const float SweepRadius = Wheel->GetCollisionShape().GetSphereRadius();
		const float Gap = FVector::DotProduct(
			SweepEnd - LocalPatchHit.ImpactPoint, N) - SweepRadius;
		if (Gap > 0.0f)
		{
			const float ReachSkin = FMath::Max(0.0f,
				CVarIAmSpeedWheelSupportProjectionReachSkin.GetValueOnAnyThread());
			bool bAppliedWallTangentProjection = false;
			// Preserve the active hitbox plane while closing an established wheel
			// reach gap. Independent inward/outward translations can undo each
			// other; remove the hitbox-normal degree of freedom from this wheel
			// correction using the same positional/angular mobility as above.
			if (!bStrictHitboxGate && bSameAnalyticSurface &&
				Speed::Analytic::FStaticWorldQueryAudit::IsSurfaceAnalyticBackend() &&
				FMath::Abs(N.Z) <= 0.10f && HitboxConstraints.Num() == 1 &&
				Wheel->IsContactVelocityLocked() && !Wheel->IsJumping() &&
				!Wheel->HasJumpUnilateralSupport())
			{
				const FHitboxPlaneConstraint& HitboxContact = HitboxConstraints[0];
				const FVector HitboxNormal = HitboxContact.Normal.GetSafeNormal();
				if (HitboxContact.Component == Contact.SurfaceComponent &&
					FVector::DotProduct(HitboxNormal, N) >= 0.995f)
				{
					const Speed::FKinematicState& HitboxState = PrincipalHitbox->GetKinematicState();
					const FVector HitboxPoint = UBoxSubBody::ComputeBoxSupportPointWS(
						HitboxState.Location, HitboxState.Rotation,
						PrincipalHitbox->GetBoxExtent(), HitboxNormal);
					const FVector WheelDirection = -N;
					const FVector WheelAngular = FVector::CrossProduct(
						SweepEnd - GetPhysCOM(), WheelDirection);
					const FVector HitboxAngular = FVector::CrossProduct(
						HitboxPoint - GetPhysCOM(), HitboxNormal);
					const float RotationLengthSquared =
						WheelConstraintRotationLength * WheelConstraintRotationLength;
					const float HitboxMobility = 1.0f +
						HitboxAngular.SizeSquared() / RotationLengthSquared;
					const float Coupling = (FVector::DotProduct(HitboxNormal, WheelDirection) +
						FVector::DotProduct(HitboxAngular, WheelAngular) / RotationLengthSquared)
						/ HitboxMobility;
					const FVector TranslationMobility = WheelDirection - Coupling * HitboxNormal;
					const FVector AngularMobility =
						(WheelAngular - Coupling * HitboxAngular) / RotationLengthSquared;
					const float WheelMobility = FVector::DotProduct(WheelDirection, TranslationMobility) +
						FVector::DotProduct(WheelAngular, AngularMobility);
					if (WheelMobility > SMALL_NUMBER)
					{
						const float Lambda = (Gap + ReachSkin) / WheelMobility;
						const FVector Translation = Lambda * TranslationMobility;
						const FVector DeltaAngular = Lambda * AngularMobility;
						const float DeltaAngle = DeltaAngular.Size();
						const float MaxTranslation = FMath::Max(0.0f,
							CVarIAmSpeedWheelSupportProjectionMaxGap.GetValueOnAnyThread());
						const float MaxRotation = FMath::DegreesToRadians(FMath::Max(0.0f,
							CVarIAmSpeedWheelSupportProjectionMaxRotationDegrees.GetValueOnAnyThread()));
						if (!Translation.ContainsNaN() && !DeltaAngular.ContainsNaN() &&
							Translation.Size() <= MaxTranslation && DeltaAngle <= MaxRotation)
						{
							SetPhysCOMLocation(GetPhysCOM() + Translation);
							if (DeltaAngle > SMALL_NUMBER)
							{
								const FQuat WorldDelta(DeltaAngular / DeltaAngle, DeltaAngle);
								SetPhysRotation((WorldDelta * GetPhysRotation()).GetNormalized());
							}
							UpdateSubBodiesKinematics();
							bAppliedWallTangentProjection = true;
						}
					}
				}
			}
#if !(UE_BUILD_SHIPPING)
			if (CVarIAmSpeedCoupledPoseDebug.GetValueOnAnyThread() != 0)
			{
				UE_LOG(LogTemp, Log,
					TEXT("[CoupledPoseWallTangent] Frame=%d Wheel=%d Applied=%d GapCm=%.6f HitboxConstraints=%d Locked=%d Jump=%d Unilateral=%d"),
					NumFrame(), Wheel->Idx(), bAppliedWallTangentProjection ? 1 : 0,
					Gap, HitboxConstraints.Num(), Wheel->IsContactVelocityLocked() ? 1 : 0,
					Wheel->IsJumping() ? 1 : 0, Wheel->HasJumpUnilateralSupport() ? 1 : 0);
			}
#endif
			if (!bAppliedWallTangentProjection)
			{
				ApplyConstraint(-N, SweepEnd, Gap + ReachSkin,
					WheelConstraintRotationLength);
			}
		}

		if (!SolveHitboxFeasibility(MaxPasses))
		{
#if !(UE_BUILD_SHIPPING)
			if (CVarIAmSpeedCoupledPoseDebug.GetValueOnAnyThread() != 0)
			{
				UE_LOG(LogTemp, Log,
					TEXT("[CoupledPoseWheelRetention] Frame=%d Wheel=%d Reason=HitboxFeasibility GapCm=%.6f RadiusCm=%.6f"),
					NumFrame(), Wheel->Idx(), Gap, SweepRadius);
			}
#endif
			SetPhysCOMLocation(BeforeWheelCOM);
			SetPhysRotation(BeforeWheelRotation);
			UpdateSubBodiesKinematics();
			continue;
		}

		Wheel->GetSuspensionSweepSegment(Delta, SweepStart, SweepEnd);
		const float FinalGap = FVector::DotProduct(
			SweepEnd - LocalPatchHit.ImpactPoint, N) - SweepRadius;
		if (FinalGap > MaxWheelGap + 0.01f)
		{
#if !(UE_BUILD_SHIPPING)
			if (CVarIAmSpeedCoupledPoseDebug.GetValueOnAnyThread() != 0)
			{
				UE_LOG(LogTemp, Log,
					TEXT("[CoupledPoseWheelRetention] Frame=%d Wheel=%d Reason=FinalGap GapCm=%.6f FinalGapCm=%.6f MaxGapCm=%.6f RadiusCm=%.6f"),
					NumFrame(), Wheel->Idx(), Gap, FinalGap, MaxWheelGap, SweepRadius);
			}
#endif
			SetPhysCOMLocation(BeforeWheelCOM);
			SetPhysRotation(BeforeWheelRotation);
			UpdateSubBodiesKinematics();
			continue;
		}
		++RetainedWheels;
	}
	// A later angular correction can undo an earlier wheel's reach constraint.
	// On a single established wall plane, one bounded normal translation closes
	// every remaining reach gap simultaneously, without changing wheel order.
	// Admission uses real local patches; acceptance requires real final sweeps.
	if (CVarIAmSpeedCoupledPoseSharedWallPlaneRetention.GetValueOnAnyThread() != 0 &&
		!bStrictHitboxGate && WheelConstraints.Num() >= 2)
	{
		const FWheelPatchConstraint& First = WheelConstraints[0];
		const FVector SharedNormal = First.Normal.GetSafeNormal();
		bool bSharedPlane = First.SurfaceSourceId != 0 && First.SurfaceId != 0 &&
			First.SurfaceFeatureId != 0 && FMath::Abs(SharedNormal.Z) <= 0.10f;
		uint64 SharedPrimitive = 0;
		double SharedPlaneD = 0.0;
		float MaximumGap = 0.0f;
		for (const FWheelPatchConstraint& Contact : WheelConstraints)
		{
			USWheelSubBody* Wheel = Contact.Wheel;
			SHitResult Patch;
			if (!bSharedPlane || !Wheel || Wheel->IsJumping() || Wheel->HasJumpUnilateralSupport() ||
				Contact.SurfaceComponent != First.SurfaceComponent ||
				Contact.SurfaceSourceId != First.SurfaceSourceId || Contact.SurfaceId != First.SurfaceId ||
				Contact.SurfaceFeatureId != First.SurfaceFeatureId ||
				Contact.Normal.GetSafeNormal() != SharedNormal ||
				!Wheel->SweepSuspensionAlongNormal(SharedNormal, FMath::Max(5.0f, Wheel->SuspensionMaxDrop()), Delta, Patch) ||
				Patch.Component != Contact.SurfaceComponent || Patch.SourceId != First.SurfaceSourceId ||
				Patch.SurfaceId != First.SurfaceId || Patch.FeatureId != First.SurfaceFeatureId ||
				Patch.ImpactNormal.GetSafeNormal() != SharedNormal || Patch.PrimitiveId == 0)
			{
				bSharedPlane = false;
				break;
			}
			const double PlaneD = FVector::DotProduct(Patch.ImpactPoint, SharedNormal);
			if (SharedPrimitive == 0)
			{
				SharedPrimitive = Patch.PrimitiveId;
				SharedPlaneD = PlaneD;
			}
			else if (Patch.PrimitiveId != SharedPrimitive ||
				!FMath::IsNearlyEqual(PlaneD, SharedPlaneD, UE_DOUBLE_SMALL_NUMBER))
			{
				bSharedPlane = false;
				break;
			}
			FVector Start, End;
			Wheel->GetSuspensionSweepSegment(Delta, Start, End);
			MaximumGap = FMath::Max(MaximumGap, static_cast<float>(
				FVector::DotProduct(End - Patch.ImpactPoint, SharedNormal) - Wheel->GetCollisionShape().GetSphereRadius()));
		}
		const float ReachSkin = FMath::Max(0.0f, CVarIAmSpeedWheelSupportProjectionReachSkin.GetValueOnAnyThread());
		const float Translation = MaximumGap + ReachSkin;
		if (bSharedPlane && MaximumGap > 0.0f &&
			Translation <= FMath::Max(0.0f, CVarIAmSpeedWheelSupportProjectionMaxGap.GetValueOnAnyThread()))
		{
			const FVector BeforeSharedCOM = GetPhysCOM();
			const FQuat BeforeSharedRotation = GetPhysRotation();
			SetPhysCOMLocation(BeforeSharedCOM - SharedNormal * Translation);
			UpdateSubBodiesKinematics();
			bool bAllRealSupport = SolveHitboxFeasibility(MaxPasses);
			for (const FWheelPatchConstraint& Contact : WheelConstraints)
			{
				SHitResult ActualHit;
				if (!bAllRealSupport || !Contact.Wheel->ProbeSuspensionOnGround(ActualHit, Delta) ||
					ActualHit.Component != Contact.SurfaceComponent || ActualHit.SourceId != First.SurfaceSourceId ||
					ActualHit.SurfaceId != First.SurfaceId || ActualHit.FeatureId != First.SurfaceFeatureId ||
					ActualHit.PrimitiveId != SharedPrimitive || ActualHit.ImpactNormal.GetSafeNormal() != SharedNormal)
				{
					bAllRealSupport = false;
					break;
				}
			}
			if (!bAllRealSupport)
			{
				SetPhysCOMLocation(BeforeSharedCOM);
				SetPhysRotation(BeforeSharedRotation);
				UpdateSubBodiesKinematics();
			}
#if !(UE_BUILD_SHIPPING)
			if (CVarIAmSpeedCoupledPoseDebug.GetValueOnAnyThread() != 0)
			{
				UE_LOG(LogTemp, Log, TEXT("[CoupledPoseSharedWallPlane] Frame=%u Wheels=%d GapCm=%.9g TranslationCm=%.9g ActualAllSupport=%d"),
					NumFrame(), WheelConstraints.Num(), MaximumGap, Translation, bAllRealSupport ? 1 : 0);
			}
#endif
		}
	}
	ClampHitboxInwardVelocity();

#if !(UE_BUILD_SHIPPING)
	if (CVarIAmSpeedCoupledPoseDebug.GetValueOnAnyThread() != 0 &&
		(!GetPhysCOM().Equals(TransactionCOM, 0.001f) ||
			GetPhysRotation().AngularDistance(TransactionRotation) > 1.0e-5f))
	{
		UE_LOG(LogTemp, Log,
			TEXT("[CoupledPose] Frame=%d HitboxTranslation=%.3f TotalTranslation=%.3f RotationDeg=%.3f HitboxConstraints=%d WheelConstraints=%d RetainedWheels=%d"),
			NumFrame(), (HitboxFeasibleCOM - TransactionCOM).Size(),
			(GetPhysCOM() - TransactionCOM).Size(),
			FMath::RadiansToDegrees(GetPhysRotation().AngularDistance(TransactionRotation)),
			HitboxConstraints.Num(), WheelConstraints.Num(), RetainedWheels);
	}
#endif
	const bool bPoseChanged = !GetPhysCOM().Equals(TransactionCOM, 0.001f) ||
		GetPhysRotation().AngularDistance(TransactionRotation) > 1.0e-5f;
	if (bPoseChanged)
	{
		BeginDeferredWheelGroundStateUpdate();
		for (USWheelSubBody* Wheel : GetWheelSubBodies())
		{
			if (!Wheel)
			{
				continue;
			}
#if !(UE_BUILD_SHIPPING)
			const bool bTraceFinalQuery = CVarIAmSpeedCoupledPoseDebug.GetValueOnAnyThread() >= 2;
			const bool bPreviousGround = Wheel->IsOnGround();
			const SHitResult PreviousQueryHit = Wheel->GetHit();
#endif
			SHitResult FinalHit;
			const bool bOnGround = Wheel->ProbeSuspensionOnGround(FinalHit, Delta);
#if !(UE_BUILD_SHIPPING)
			if (bTraceFinalQuery)
			{
				FVector QueryStart, QueryEnd;
				Wheel->GetSuspensionSweepSegment(Delta, QueryStart, QueryEnd);
				const FVector PreviousQueryNormal = PreviousQueryHit.ImpactNormal.GetSafeNormal();
				const float PreviousPlaneGap = FVector::DotProduct(QueryEnd - PreviousQueryHit.ImpactPoint, PreviousQueryNormal) - Wheel->GetCollisionShape().GetSphereRadius();
				UE_LOG(LogTemp, Log,
					TEXT("[CoupledPoseFinalQuery] ComponentFrame=%u Wheel=%d BeforeGround=%d ActualHit=%d StrictHitbox=%d RetainedConstraints=%d PreviousSource=%016llx FinalSource=%016llx PreviousGapCm=%.9g InwardSpeed=%.9g Start=(%.17g,%.17g,%.17g) End=(%.17g,%.17g,%.17g) PreviousNormal=(%.17g,%.17g,%.17g)"),
					NumFrame(), Wheel->Idx(), bPreviousGround ? 1 : 0, bOnGround ? 1 : 0,
					bStrictHitboxGate ? 1 : 0, RetainedWheels,
					static_cast<unsigned long long>(PreviousQueryHit.SourceId),
					static_cast<unsigned long long>(bOnGround ? FinalHit.SourceId : 0),
					PreviousPlaneGap, FVector::DotProduct(GetPhysCOMVelocity(), PreviousQueryNormal),
					QueryStart.X, QueryStart.Y, QueryStart.Z, QueryEnd.X, QueryEnd.Y, QueryEnd.Z,
					PreviousQueryNormal.X, PreviousQueryNormal.Y, PreviousQueryNormal.Z);
			}
#endif
			if (bOnGround)
			{
				Wheel->SetHit(FinalHit);
			}
			Wheel->SetOnGround(bOnGround);
		}
		EndDeferredWheelGroundStateUpdate();
	}
	return bPoseChanged;
}
