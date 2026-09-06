#include "BoxRestingSupport.h"
#include <cfloat>

namespace Speed
{
bool CanStabilizeBoxMicroRocking(
	const IStaticCollisionWorld& World, const Analytic::FWorldQuery& BoxQuery,
	const FKinematicState& Incoming, const FMatrix& WorldInverseInertia,
	double InverseMass, double ImpactRestitution, double DynamicFriction)
{
	// IAMSPEED.PHYS.BOX_EQUILIBRIUM.MICRO_ROCKING.V1. This is deliberate,
	// dissipative numerical closure of accumulating impacts, not an exact
	// impulse solution or a velocity-based sleep shortcut.
	if (ImpactRestitution != 0 || !FMath::IsFinite(DynamicFriction) || DynamicFriction <= 0 ||
		!FMath::IsFinite(InverseMass) || InverseMass <= 0 || WorldInverseInertia.ContainsNaN() ||
		WorldInverseInertia.Determinant() <= 0 || Incoming.Velocity.ContainsNaN() ||
		Incoming.AngularVelocity.ContainsNaN() || Incoming.Acceleration.ContainsNaN() ||
		!Incoming.AngularAcceleration.IsZero()) return false;
	const double Load = Incoming.Acceleration.Size();
	if (!FMath::IsFinite(Load) || Load <= 0) return false;
	const FVector N = -Incoming.Acceleration / Load;
	// Friction on a slide or spin has its own law; only rocking is regularized.
	if (FVector::CrossProduct(Incoming.AngularVelocity, N).IsZero()) return false;
	const FMatrix Inertia = WorldInverseInertia.Inverse();
	const double AngularEnergyPerMass = .5 * InverseMass * FVector::DotProduct(
		Incoming.AngularVelocity, FVector(Inertia.TransformVector(Incoming.AngularVelocity)));
	const double EnergyPerMass = .5 * Incoming.Velocity.SizeSquared() + AngularEnergyPerMass;
	constexpr double HalfPositionStepCm = .5 / FKinematicState::PositionQuantizationScale;
	if (!FMath::IsFinite(EnergyPerMass) || AngularEnergyPerMass < 0 ||
		EnergyPerMass >= Load * HalfPositionStepCm * FMath::Min(1.0, DynamicFriction)) return false;
	// The friction-work bound above matters even when rocking is tiny: a low
	// total energy can still carry a nearly frictionless slide a long distance.
	FBoxRestingSupport Support;
	if (!TryBuildBoxRestingSupport(World, BoxQuery, Incoming.Location, Incoming.Acceleration, Support))
		return false;
	const FVector U = Support.Points[1] - Support.Points[0];
	const FVector V = Support.Points[2] - Support.Points[0];
	const FVector Offset = Incoming.Location - Support.Points[0];
	const double ULength = U.Size(), VLength = V.Size();
	const double COMU = FVector::DotProduct(Offset, U) / ULength;
	const double COMV = FVector::DotProduct(Offset, V) / VLength;
	const double Margin = FMath::Min(FMath::Min(COMU, ULength - COMU), FMath::Min(COMV, VLength - COMV));
	const double Height = FVector::DotProduct(Offset, N);
	// Limit the possible rocking excursion of every corner and the COM, not
	// just COM height. Also stay below the energy barrier of the nearest edge:
	// an almost-tipping COM must not be captured merely because it is inside.
	const double PivotRadiusBound = 2 * BoxQuery.HalfExtent.Size() + (Incoming.Location - BoxQuery.Start).Size();
	const double Angle = FMath::Min(HalfPositionStepCm / PivotRadiusBound, FMath::Atan2(Margin, Height));
	const double Rise = Margin * FMath::Sin(Angle) - 2 * Height * FMath::Square(FMath::Sin(.5 * Angle));
	if (Margin <= 0 || !FMath::IsFinite(Rise) || Rise <= 0 || EnergyPerMass >= Load * Rise) return false;
	for (const FVector& Point : Support.Points)
	{
		const FVector PointVelocity = Incoming.Velocity +
			FVector::CrossProduct(Incoming.AngularVelocity, Point - Incoming.Location);
		if (FVector::DotProduct(PointVelocity, N) < 0) return true;
	}
	return false; // Separating impulse: no incoming impact, hence no capture.
}

bool TryBuildBoxRestingSupport(
	const IStaticCollisionWorld& World,
	const Analytic::FWorldQuery& BoxQuery,
	const FVector& CenterOfMass,
	const FVector& ExternalAcceleration,
	FBoxRestingSupport& OutSupport)
{
	// IAMSPEED.PHYS.BOX_STANDING_STILL.V1. A certificate is local to this call;
	// failed/released support must never leave a usable previous reaction behind.
	OutSupport = {};
	if (BoxQuery.Shape != Analytic::EQueryShape::Box ||
		BoxQuery.Start != BoxQuery.End || BoxQuery.Start.ContainsNaN() ||
		CenterOfMass.ContainsNaN() || ExternalAcceleration.ContainsNaN() ||
		ExternalAcceleration.IsZero() || !BoxQuery.Rotation.IsNormalized() ||
		BoxQuery.HalfExtent.ContainsNaN() || BoxQuery.HalfExtent.GetMin() <= 0.0)
		return false;

	const FVector Normal = -ExternalAcceleration.GetSafeNormal();
	OutSupport.Result = EBoxRestingSupportResult::UnsupportedLoad;
	if (Normal.IsZero()) return false;
	const FVector Axes[3] = { BoxQuery.Rotation.GetAxisX(),
		BoxQuery.Rotation.GetAxisY(), BoxQuery.Rotation.GetAxisZ() };
	int32 FaceAxis = INDEX_NONE;
	double FaceSign = 0.0;
	for (int32 Axis = 0; Axis < 3; ++Axis)
	{
		const double Dot = FVector::DotProduct(Axes[Axis], Normal);
		if (FMath::Abs(Dot) >= 1.0 - 32.0 * DBL_EPSILON)
		{
			FaceAxis = Axis;
			FaceSign = Dot > 0.0 ? -1.0 : 1.0;
			break;
		}
	}
	if (FaceAxis == INDEX_NONE) return false;
	const int32 U = (FaceAxis + 1) % 3, V = (FaceAxis + 2) % 3;
	const FVector FaceCenter = BoxQuery.Start +
		Axes[FaceAxis] * (FaceSign * BoxQuery.HalfExtent[FaceAxis]);
	const FVector COMOffset = CenterOfMass - FaceCenter;
	const double COMU = FVector::DotProduct(COMOffset, Axes[U]) / BoxQuery.HalfExtent[U];
	const double COMV = FVector::DotProduct(COMOffset, Axes[V]) / BoxQuery.HalfExtent[V];
	OutSupport.Result = EBoxRestingSupportResult::UnstableCOM;
	if (FMath::Abs(COMU) >= 1.0 || FMath::Abs(COMV) >= 1.0 ||
		FVector::DotProduct(COMOffset, Normal) <= 0.0)
		return false;

	// Roundoff in transformed vertices, not a gameplay contact skin. The query
	// still rejects any positive penetration and never translates the body.
	const double Scale = FMath::Max(1.0, BoxQuery.Start.GetAbsMax() + BoxQuery.HalfExtent.GetMax());
	const double RoundoffCm = 64.0 * DBL_EPSILON * Scale;
	Analytic::FWorldQuery Query = BoxQuery;
	Query.InitialOverlapTolerance = 0.0;
	Query.DomainTolerance = 0.0;
	const Analytic::FWorldHit Overlap = World.SweepSingle(Query);
	OutSupport.Result = EBoxRestingSupportResult::Penetrating;
	if (Overlap.bStartPenetrating || Overlap.PenetrationDepth > 0.0)
	{
		OutSupport.ProviderWitness = Overlap;
		OutSupport.WitnessGapCm = -Overlap.PenetrationDepth;
		return false;
	}

	FBoxRestingSupport Candidate;
	Candidate.Normal = Normal;
	Query.Shape = Analytic::EQueryShape::Ray;
	Query.HalfExtent = FVector::ZeroVector;
	// The ray straddles each actual corner. Its length locates a witness only;
	// the accepted gap below is bounded by floating-point roundoff, not this ray.
	constexpr double ProbeHalfLengthCm = 0.01;
	for (int32 PointIndex = 0; PointIndex < 4; ++PointIndex)
	{
		const double SignU = (PointIndex & 1) ? 1.0 : -1.0;
		const double SignV = (PointIndex & 2) ? 1.0 : -1.0;
		FVector LocalCorner = FVector::ZeroVector;
		LocalCorner[FaceAxis] = FaceSign * BoxQuery.HalfExtent[FaceAxis];
		LocalCorner[U] = SignU * BoxQuery.HalfExtent[U];
		LocalCorner[V] = SignV * BoxQuery.HalfExtent[V];
		const FVector Corner = BoxQuery.Start + BoxQuery.Rotation.RotateVector(LocalCorner);
		Query.Start = Corner + Normal * ProbeHalfLengthCm;
		Query.End = Corner - Normal * ProbeHalfLengthCm;
		const Analytic::FWorldHit Hit = World.SweepSingle(Query);
		OutSupport.Result = EBoxRestingSupportResult::UncertifiedGeometry;
		OutSupport.WitnessIndex = PointIndex;
		if (!Hit.bHit || Hit.bStartPenetrating || Hit.PenetrationDepth > 0.0 ||
			Hit.bSurfaceNormalMayVary || Hit.GeometricErrorBoundCm != 0.0 ||
			Hit.SurfaceFeatureKind != EContactFeatureKind::Face ||
			!Hit.Normal.Equals(Normal, 32.0 * DBL_EPSILON))
			return false;
		const double Gap = FVector::DotProduct(Corner - Hit.Point, Normal);
		OutSupport.Result = EBoxRestingSupportResult::NotTouching;
		OutSupport.WitnessGapCm = Gap;
		if (Gap < 0.0 || Gap > RoundoffCm) return false;
		Candidate.Points[PointIndex] = Hit.Point;
		if (PointIndex == 0) Candidate.ProviderWitness = Hit;
		// Bilinear barycentric weights put the center of pressure directly under
		// the COM. All are positive inside the face: no tensile contact or torque.
		Candidate.LoadFractions[PointIndex] =
			0.25 * (1.0 + SignU * COMU) * (1.0 + SignV * COMV);
	}
	// Apply the certified resultant once at COM; summing four cancelling torques
	// numerically would manufacture a spin at an otherwise exact equilibrium.
	Candidate.ResultantAcceleration = -ExternalAcceleration;
	Candidate.Result = EBoxRestingSupportResult::Supported;
	OutSupport = Candidate;
	return true;
}

bool TryBuildBoxSlidingSupport(
	const IStaticCollisionWorld& World, const Analytic::FWorldQuery& BoxQuery,
	const FVector& CenterOfMass, const FVector& ExternalAcceleration,
	const FVector& TangentialVelocity, double DynamicFriction,
	FBoxRestingSupport& OutSupport, double HorizonSeconds)
{
	OutSupport = {};
	if (TangentialVelocity.ContainsNaN() || TangentialVelocity.IsZero() ||
		FVector::DotProduct(TangentialVelocity, ExternalAcceleration) != 0 ||
		!FMath::IsFinite(DynamicFriction) || DynamicFriction < 0 ||
		!FMath::IsFinite(HorizonSeconds) || HorizonSeconds < 0) return false;
	FBoxRestingSupport Support;
	if (!TryBuildBoxRestingSupport(World, BoxQuery, CenterOfMass, ExternalAcceleration, Support)) return false;
	const double NormalLoad = ExternalAcceleration.Size();
	const double Speed = TangentialVelocity.Size();
	const FVector Direction = TangentialVelocity / Speed;
	const double Height = FVector::DotProduct(CenterOfMass - Support.Points[0], Support.Normal);
	const FVector PressureCenter = CenterOfMass - Height * Support.Normal + Height * DynamicFriction * Direction;
	const FVector U = Support.Points[1] - Support.Points[0], V = Support.Points[2] - Support.Points[0];
	const FVector Offset = PressureCenter - Support.Points[0];
	const double BaryU = FVector::DotProduct(Offset, U) / U.SizeSquared();
	const double BaryV = FVector::DotProduct(Offset, V) / V.SizeSquared();
	if (BaryU <= 0 || BaryU >= 1 || BaryV <= 0 || BaryV >= 1)
	{
		OutSupport.Result = EBoxRestingSupportResult::UnstableCOM;
		return false;
	}
	Support.LoadFractions[0] = (1 - BaryU) * (1 - BaryV);
	Support.LoadFractions[1] = BaryU * (1 - BaryV);
	Support.LoadFractions[2] = (1 - BaryU) * BaryV;
	Support.LoadFractions[3] = BaryU * BaryV;
	Support.ResultantAcceleration = -ExternalAcceleration - DynamicFriction * NormalLoad * Direction;
	if (DynamicFriction > 0) Support.StopAfterSeconds = Speed / (DynamicFriction * NormalLoad);
	if (HorizonSeconds > 0)
	{
		const double T = FMath::Min(HorizonSeconds, Support.StopAfterSeconds);
		const double TravelTime = DynamicFriction > 0 ? T * (1 - .5 * T / Support.StopAfterSeconds) : T;
		if (!World.IsPlanarSupportTranslationCertified(BoxQuery, MakeArrayView(Support.Points),
			Support.Normal, TangentialVelocity * TravelTime))
		{
			OutSupport.Result = EBoxRestingSupportResult::UncertifiedGeometry;
			return false;
		}
	}
	OutSupport = Support;
	return true;
}
}
