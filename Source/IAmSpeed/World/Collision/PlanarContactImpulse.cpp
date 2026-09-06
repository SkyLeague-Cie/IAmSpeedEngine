#include "PlanarContactImpulse.h"
#include <cfloat>

namespace Speed
{
namespace
{
// A contact array has no geometric winding contract. Reconstruct the two
// adjacent edges from the opposite (longest-distance) corner, in constant
// space, and keep weights mapped to the caller's original point indices.
bool FindRectangleAxes(TConstArrayView<FVector> Points, int32 (&Indices)[4], FVector& U, FVector& V)
{
	if (Points.Num() != 4) return false;
	Indices[0] = 0;
	Indices[3] = 1;
	for (int32 I = 0; I < 4; ++I)
	{
		if (Points[I].ContainsNaN()) return false;
		if ((Points[I] - Points[0]).SizeSquared() > (Points[Indices[3]] - Points[0]).SizeSquared()) Indices[3] = I;
	}
	int32 Next = 1;
	for (int32 I = 1; I < 4; ++I) if (I != Indices[3]) Indices[Next++] = I;
	U = Points[Indices[1]] - Points[0];
	V = Points[Indices[2]] - Points[0];
	const double Roundoff = 128 * DBL_EPSILON * FMath::Max(1.0, U.Size() + V.Size());
	return U.SizeSquared() > 0 && V.SizeSquared() > 0 &&
		(Points[Indices[3]] - Points[0] - U - V).Size() <= Roundoff &&
		FMath::Abs(FVector::DotProduct(U, V)) <= Roundoff * FMath::Max(U.Size(), V.Size());
}

void SetRectangleWeights(FPlanarNormalImpulse& Result, const int32 (&Indices)[4], double Total, double S, double T)
{
	Result.Impulses[Indices[0]] = Total * (1 - S) * (1 - T);
	Result.Impulses[Indices[1]] = Total * S * (1 - T);
	Result.Impulses[Indices[2]] = Total * (1 - S) * T;
	Result.Impulses[Indices[3]] = Total * S * T;
}

// Two sticking points leave one degree of freedom: rotation about their edge.
// Conserve angular momentum about that edge to find the dissipative endpoint,
// then certify two non-attractive impulses inside their individual Coulomb cones.
// No small-speed rule, iteration budget increase or kinematic pose correction.
bool TryCertifyEdgeSticking(TConstArrayView<FVector> Points, const FVector& Normal,
	const FVector& COM, const FVector& Velocity, const FVector& AngularVelocity,
	double InverseMass, const FMatrix& InverseInertia, double Friction, FPlanarNormalImpulse& Out)
{
	const FVector Edge = Points[1] - Points[0];
	const double Length = Edge.Size();
	if (Length <= 0 || !FMath::IsFinite(Length) ||
		FMath::Abs(FVector::DotProduct(Edge, Normal)) > 64 * DBL_EPSILON * Length) return false;
	const FVector Axis = Edge / Length, R0 = Points[0] - COM;
	const double Mass = 1.0 / InverseMass;
	const FMatrix Inertia = InverseInertia.Inverse();
	const FVector AngularMomentum = Inertia.TransformVector(AngularVelocity);
	const double EdgeInertia = FVector::DotProduct(Axis, Inertia.TransformVector(Axis)) +
		Mass * FVector::CrossProduct(R0, Axis).SizeSquared();
	if (EdgeInertia <= 0 || !FMath::IsFinite(EdgeInertia) || AngularMomentum.ContainsNaN()) return false;
	const double EdgeMomentum = FVector::DotProduct(Axis,
		AngularMomentum - FVector::CrossProduct(R0, Mass * Velocity));
	const FVector FinalW = Axis * (EdgeMomentum / EdgeInertia);
	const FVector FinalV = -FVector::CrossProduct(FinalW, R0);
	const FVector TotalJ = Mass * (FinalV - Velocity);
	const FVector Torque = Inertia.TransformVector(FinalW - AngularVelocity);
	const FVector MomentAbout0 = Torque - FVector::CrossProduct(R0, TotalJ);
	const double MomentRoundoff = 256 * DBL_EPSILON * FMath::Max(1.0, Torque.Size() + R0.Size() * TotalJ.Size());
	if (FMath::Abs(FVector::DotProduct(MomentAbout0, Axis)) > MomentRoundoff) return false;
	const FVector J1Perpendicular = FVector::CrossProduct(MomentAbout0, Axis) / Length;
	const double AlongEdge = FVector::DotProduct(TotalJ, Axis);
	const double N1 = FVector::DotProduct(J1Perpendicular, Normal);
	const double N0 = FVector::DotProduct(TotalJ, Normal) - N1;
	if (N0 < 0 || N1 < 0) return false;
	const FVector T1 = J1Perpendicular - N1 * Normal;
	const FVector T0 = TotalJ - J1Perpendicular - AlongEdge * Axis - N0 * Normal;
	const double Budget0 = FMath::Square(Friction * N0) - T0.SizeSquared();
	const double Budget1 = FMath::Square(Friction * N1) - T1.SizeSquared();
	if (Budget0 < 0 || Budget1 < 0 || !FMath::IsFinite(Budget0) || !FMath::IsFinite(Budget1)) return false;
	const double Radius0 = FMath::Sqrt(Budget0), Radius1 = FMath::Sqrt(Budget1);
	const double Lower = FMath::Max(-Radius1, AlongEdge - Radius0);
	const double Upper = FMath::Min(Radius1, AlongEdge + Radius0);
	if (Lower > Upper) return false;
	// Internal edge traction is indeterminate. Pick the least squared impulse
	// in the feasible interval; reversing the input points yields the same wrench.
	const FVector J1 = J1Perpendicular + FMath::Clamp(.5 * AlongEdge, Lower, Upper) * Axis;
	Out.Impulses[0] = N0; Out.Impulses[1] = N1;
	Out.TangentImpulses[0] = TotalJ - J1 - N0 * Normal;
	Out.TangentImpulses[1] = J1 - N1 * Normal;
	Out.DeltaVelocity = FinalV - Velocity;
	Out.DeltaAngularVelocity = FinalW - AngularVelocity;
	Out.bSolved = true;
	return true;
}
}

FPlanarNormalImpulse SolvePlanarNormalImpulse(
	TConstArrayView<FVector> Points, const FVector& Normal,
	const FVector& COM, const FVector& Velocity, const FVector& AngularVelocity,
	double InverseMass, const FMatrix& WorldInverseInertia, double Restitution,
	TConstArrayView<double> PointNormalBias)
{
	FPlanarNormalImpulse Result;
	const int32 Count = Points.Num();
	if (Count < 1 || Count > 4 || (!PointNormalBias.IsEmpty() && PointNormalBias.Num() != Count) || InverseMass <= 0 || !FMath::IsFinite(InverseMass) ||
		Normal.ContainsNaN() || FMath::Abs(Normal.SizeSquared() - 1) > 64 * DBL_EPSILON ||
		COM.ContainsNaN() || Velocity.ContainsNaN() || AngularVelocity.ContainsNaN() ||
		Restitution < 0 || Restitution > 1 || !FMath::IsFinite(Restitution)) return Result;
	FVector AngularJacobian[4], AngularResponse[4];
	double Matrix[4][4] = {}, Free[4] = {}, Scale = 1;
	for (int32 I = 0; I < Count; ++I)
	{
		if (Points[I].ContainsNaN()) return Result;
		AngularJacobian[I] = FVector::CrossProduct(Points[I] - COM, Normal);
		AngularResponse[I] = WorldInverseInertia.TransformVector(AngularJacobian[I]);
		if (AngularResponse[I].ContainsNaN()) return Result;
		const double ClosingSpeed = FVector::DotProduct(Velocity, Normal) +
			FVector::DotProduct(AngularJacobian[I], AngularVelocity);
		Free[I] = ClosingSpeed + Restitution * FMath::Min(ClosingSpeed, 0.0) +
			(PointNormalBias.IsEmpty() ? 0.0 : PointNormalBias[I]);
		if (!FMath::IsFinite(Free[I])) return Result;
		Scale = FMath::Max(Scale, FMath::Abs(Free[I]));
	}
	for (int32 I = 0; I < Count; ++I)
	for (int32 J = 0; J < Count; ++J)
		Matrix[I][J] = InverseMass + FVector::DotProduct(AngularJacobian[I], AngularResponse[J]);
	const double SpeedRoundoff = 256 * DBL_EPSILON * Scale;
	const double ImpulseRoundoff = SpeedRoundoff / InverseMass;
	for (uint32 Mask = 0; Mask < (1u << Count); ++Mask)
	{
		int32 Active[3] = {}, Size = 0;
		for (int32 I = 0; I < Count; ++I)
			if (Mask & (1u << I)) { if (Size < 3) Active[Size] = I; ++Size; }
		if (Size > 3) continue;
		double Reduced[3][4] = {}, MatrixScale = InverseMass;
		for (int32 I = 0; I < Size; ++I)
		{
			for (int32 J = 0; J < Size; ++J)
			{
				Reduced[I][J] = Matrix[Active[I]][Active[J]];
				MatrixScale = FMath::Max(MatrixScale, FMath::Abs(Reduced[I][J]));
			}
			Reduced[I][Size] = -Free[Active[I]];
		}
		bool bIndependent = true;
		for (int32 Column = 0; Column < Size; ++Column)
		{
			int32 Pivot = Column;
			for (int32 Row = Column + 1; Row < Size; ++Row)
				if (FMath::Abs(Reduced[Row][Column]) > FMath::Abs(Reduced[Pivot][Column])) Pivot = Row;
			if (FMath::Abs(Reduced[Pivot][Column]) <= 64 * DBL_EPSILON * MatrixScale)
			{ bIndependent = false; break; }
			for (int32 J = Column; J <= Size; ++J) Swap(Reduced[Column][J], Reduced[Pivot][J]);
			const double Divisor = Reduced[Column][Column];
			for (int32 J = Column; J <= Size; ++J) Reduced[Column][J] /= Divisor;
			for (int32 Row = 0; Row < Size; ++Row)
			{
				if (Row == Column) continue;
				const double Factor = Reduced[Row][Column];
				for (int32 J = Column; J <= Size; ++J) Reduced[Row][J] -= Factor * Reduced[Column][J];
			}
		}
		if (!bIndependent) continue;
		double Lambda[4] = {};
		bool bFeasible = true;
		for (int32 I = 0; I < Size; ++I)
		{
			const double Value = Reduced[I][Size];
			if (!FMath::IsFinite(Value) || Value < -ImpulseRoundoff) { bFeasible = false; break; }
			Lambda[Active[I]] = FMath::Max(Value, 0.0);
		}
		if (!bFeasible) continue;
		double ClosingResidual = 0;
		for (int32 I = 0; I < Count; ++I)
		{
			double Residual = Free[I];
			for (int32 J = 0; J < Count; ++J) Residual += Matrix[I][J] * Lambda[J];
			ClosingResidual = FMath::Max(ClosingResidual, -Residual);
			if (Residual < -SpeedRoundoff || (Lambda[I] > ImpulseRoundoff && FMath::Abs(Residual) > SpeedRoundoff))
			{ bFeasible = false; break; }
		}
		if (!bFeasible) continue;
		for (int32 I = 0; I < Count; ++I)
		{
			Result.Impulses[I] = Lambda[I];
			Result.DeltaVelocity += Normal * (InverseMass * Lambda[I]);
			Result.DeltaAngularVelocity += AngularResponse[I] * Lambda[I];
		}
		Result.MaximumClosingResidual = ClosingResidual;
		bool bEveryRowActive = true;
		for (int32 I = 0; I < Count; ++I)
		{
			double Residual = Free[I];
			for (int32 J = 0; J < Count; ++J) Residual += Matrix[I][J] * Lambda[J];
			bEveryRowActive &= FMath::Abs(Residual) <= SpeedRoundoff;
		}
		if (Count == 4 && bEveryRowActive)
		{
			FVector U, V;
			int32 Indices[4];
			const double Total = Lambda[0] + Lambda[1] + Lambda[2] + Lambda[3];
			if (Total > 0 && FindRectangleAxes(Points, Indices, U, V))
			{
				FVector PressureOffset = FVector::ZeroVector;
				for (int32 I = 0; I < 4; ++I) PressureOffset += (Points[I] - Points[0]) * (Lambda[I] / Total);
				const double S = FVector::DotProduct(PressureOffset, U) / U.SizeSquared();
				const double T = FVector::DotProduct(PressureOffset, V) / V.SizeSquared();
				if (S >= 0 && S <= 1 && T >= 0 && T <= 1)
				{
					// Four normal rows are redundant. Preserve their already-solved
					// resultant/COP but choose symmetric positive face weights, so
					// friction does not inherit an arbitrary active-set diagonal.
					SetRectangleWeights(Result, Indices, Total, S, T);
				}
			}
		}
		Result.bSolved = true;
		return Result;
	}
	return Result;
}

FPlanarNormalImpulse SolvePlanarSlidingReaction(
	TConstArrayView<FVector> Points, const FVector& Normal, const FVector& COM,
	const FVector& Velocity, const FVector& AngularVelocity,
	const FVector& Acceleration, const FVector& AngularAcceleration,
	double InverseMass, const FMatrix& WorldInverseInertia, double Friction, double HorizonSeconds)
{
	const int32 Count = Points.Num();
	if (Count < 1 || Count > 4 || Friction < 0 || !FMath::IsFinite(Friction) ||
		HorizonSeconds <= 0 || !FMath::IsFinite(HorizonSeconds)) return {};
	FVector Slip[4], Direction[4];
	double Bias[4];
	for (int32 I = 0; I < Count; ++I)
	{
		const FVector R = Points[I] - COM;
		const FVector VP = Velocity + FVector::CrossProduct(AngularVelocity, R);
		Slip[I] = VP - FVector::DotProduct(VP, Normal) * Normal;
		const double Speed = Slip[I].Size();
		if (!FMath::IsFinite(Speed) || Speed <= 0) return {}; // sticking is a different law
		Direction[I] = -Friction * Slip[I] / Speed;
		Bias[I] = FVector::DotProduct(FVector::CrossProduct(AngularVelocity,
			FVector::CrossProduct(AngularVelocity, R)), Normal);
	}
	FVector FrictionA = FVector::ZeroVector, FrictionAlpha = FVector::ZeroVector;
	const double Tolerance = 4096 * DBL_EPSILON * FMath::Max(1.0, Acceleration.Size() + AngularAcceleration.Size());
	for (uint16 Iteration = 0; Iteration < 32; ++Iteration)
	{
		auto Result = SolvePlanarNormalImpulse(Points, Normal, COM, Acceleration + FrictionA,
			AngularAcceleration + FrictionAlpha, InverseMass, WorldInverseInertia, 0, MakeArrayView(Bias, Count));
		if (!Result.bSolved) return {};
		FVector NextA = FVector::ZeroVector, NextAlpha = FVector::ZeroVector;
		for (int32 I = 0; I < Count; ++I)
		{
			Result.TangentImpulses[I] = Direction[I] * Result.Impulses[I];
			NextA += InverseMass * Result.TangentImpulses[I];
			NextAlpha += WorldInverseInertia.TransformVector(FVector::CrossProduct(Points[I] - COM, Result.TangentImpulses[I]));
		}
		if ((NextA - FrictionA).Size() <= Tolerance && (NextAlpha - FrictionAlpha).Size() <= Tolerance)
		{
			Result.DeltaVelocity += NextA;
			Result.DeltaAngularVelocity += NextAlpha;
			Result.IterationCount = Iteration + 1;
			for (int32 I = 0; I < Count; ++I)
			{
				const FVector R = Points[I] - COM;
				const FVector AP = Acceleration + Result.DeltaVelocity +
					FVector::CrossProduct(AngularAcceleration + Result.DeltaAngularVelocity, R) +
					FVector::CrossProduct(AngularVelocity, FVector::CrossProduct(AngularVelocity, R));
				// Every supplied row must remain supported. Other vertices remain
				// free to approach the plane and are still handled by CCD.
				if (FMath::Abs(FVector::DotProduct(AP, Normal)) > 64 * Tolerance ||
					AP.Size() * HorizonSeconds >= Slip[I].Size()) return {};
			}
			return Result;
		}
		FrictionA = NextA; FrictionAlpha = NextAlpha;
	}
	return {};
}

FPlanarNormalImpulse SolvePlanarCoulombImpulse(
	TConstArrayView<FVector> Points, const FVector& Normal,
	const FVector& COM, const FVector& Velocity, const FVector& AngularVelocity,
	double InverseMass, const FMatrix& WorldInverseInertia, double Restitution, double Friction)
{
	FPlanarNormalImpulse Result;
	const int32 Count = Points.Num();
	if (Friction < 0 || !FMath::IsFinite(Friction) || Count < 1 || Count > 4 ||
		InverseMass <= 0 || !FMath::IsFinite(InverseMass) || Restitution < 0 || Restitution > 1 ||
		!FMath::IsFinite(Restitution) || Normal.ContainsNaN() ||
		FMath::Abs(Normal.SizeSquared() - 1) > 64 * DBL_EPSILON || COM.ContainsNaN() ||
		Velocity.ContainsNaN() || AngularVelocity.ContainsNaN()) return Result;
	if (Friction == 0) return SolvePlanarNormalImpulse(Points, Normal, COM, Velocity, AngularVelocity,
		InverseMass, WorldInverseInertia, Restitution);
	if (Count == 2 && Restitution == 0 && TryCertifyEdgeSticking(Points, Normal, COM,
		Velocity, AngularVelocity, InverseMass, WorldInverseInertia, Friction, Result)) return Result;
	if (Count == 4 && Restitution == 0)
	{
		// Zero post-impact kinetic energy is the global dissipative optimum if
		// the required linear AND angular impulses admit positive face weights
		// inside the Coulomb cone. This is an impulse-feasibility certificate,
		// not a small-speed threshold or an orientation/position correction.
		const FVector Impulse = -Velocity / InverseMass;
		const FVector Torque = WorldInverseInertia.Inverse().TransformVector(-AngularVelocity);
		const double NormalImpulse = FVector::DotProduct(Impulse, Normal);
		const FVector TangentImpulse = Impulse - NormalImpulse * Normal;
		FVector U, V;
		int32 Indices[4];
		const double TorqueRoundoff = 128 * DBL_EPSILON * FMath::Max(1.0, Torque.Size() * Impulse.Size());
		if (NormalImpulse > 0 && !Torque.ContainsNaN() &&
			TangentImpulse.Size() <= Friction * NormalImpulse &&
			FMath::Abs(FVector::DotProduct(Torque, Impulse)) <= TorqueRoundoff &&
			FindRectangleAxes(Points, Indices, U, V))
		{
			FVector Lever = FVector::CrossProduct(Impulse, Torque) / Impulse.SizeSquared();
			Lever += Impulse * (FVector::DotProduct(Points[0] - COM - Lever, Normal) / NormalImpulse);
			const FVector Offset = COM + Lever - Points[0];
			const double S = FVector::DotProduct(Offset, U) / U.SizeSquared();
			const double T = FVector::DotProduct(Offset, V) / V.SizeSquared();
			if (S >= 0 && S <= 1 && T >= 0 && T <= 1)
			{
				SetRectangleWeights(Result, Indices, NormalImpulse, S, T);
				for (int32 I = 0; I < Count; ++I)
					Result.TangentImpulses[I] = TangentImpulse * (Result.Impulses[I] / NormalImpulse);
				Result.DeltaVelocity = -Velocity;
				Result.DeltaAngularVelocity = -AngularVelocity;
				Result.bSolved = true;
				return Result;
			}
		}
	}
	FVector U, V;
	Normal.FindBestAxisVectors(U, V);
	FVector R[4], TangentImpulses[4] = {};
	double K00[4], K01[4], K11[4], Determinant[4], BounceBias[4];
	for (int32 I = 0; I < Count; ++I)
	{
		R[I] = Points[I] - COM;
		const FVector RU = FVector::CrossProduct(R[I], U), RV = FVector::CrossProduct(R[I], V);
		K00[I] = InverseMass + FVector::DotProduct(RU, WorldInverseInertia.TransformVector(RU));
		K01[I] = FVector::DotProduct(RU, WorldInverseInertia.TransformVector(RV));
		K11[I] = InverseMass + FVector::DotProduct(RV, WorldInverseInertia.TransformVector(RV));
		Determinant[I] = K00[I] * K11[I] - K01[I] * K01[I];
		if (Determinant[I] <= 0 || !FMath::IsFinite(Determinant[I])) return Result;
		BounceBias[I] = Restitution * FMath::Min(0.0,
			FVector::DotProduct(Velocity + FVector::CrossProduct(AngularVelocity, R[I]), Normal));
	}
	const double Tolerance = 256 * DBL_EPSILON * FMath::Max(1.0, Velocity.Size() + AngularVelocity.Size() * R[0].Size());
	FVector Linear = Velocity, Angular = AngularVelocity;
	for (int32 Iteration = 0; Iteration < 256; ++Iteration)
	{
		Linear = Velocity; Angular = AngularVelocity;
		for (int32 I = 0; I < Count; ++I)
		{
			Linear += TangentImpulses[I] * InverseMass;
			Angular += WorldInverseInertia.TransformVector(FVector::CrossProduct(R[I], TangentImpulses[I]));
		}
		Result = SolvePlanarNormalImpulse(Points, Normal, COM, Linear, Angular,
			InverseMass, WorldInverseInertia, 0, MakeArrayView(BounceBias, Count));
		Result.IterationCount = static_cast<uint16>(Iteration + 1);
		if (!Result.bSolved) return Result;
		Linear += Result.DeltaVelocity;
		Angular += Result.DeltaAngularVelocity;
		double MaximumChange = 0;
		for (int32 I = 0; I < Count; ++I)
		{
			const FVector PointVelocity = Linear + FVector::CrossProduct(Angular, R[I]);
			const double VU = FVector::DotProduct(PointVelocity, U), VV = FVector::DotProduct(PointVelocity, V);
			FVector NewImpulse = TangentImpulses[I] -
				U * ((K11[I] * VU - K01[I] * VV) / Determinant[I]) -
				V * ((K00[I] * VV - K01[I] * VU) / Determinant[I]);
			const double Limit = Friction * Result.Impulses[I];
			const double Magnitude = NewImpulse.Size();
			if (Magnitude > Limit) NewImpulse *= Limit / Magnitude;
			const FVector Change = NewImpulse - TangentImpulses[I];
			TangentImpulses[I] = NewImpulse;
			Linear += Change * InverseMass;
			Angular += WorldInverseInertia.TransformVector(FVector::CrossProduct(R[I], Change));
			MaximumChange = FMath::Max(MaximumChange, Change.Size() * InverseMass);
		}
		if (MaximumChange <= Tolerance)
		{
			for (int32 I = 0; I < Count; ++I) Result.TangentImpulses[I] = TangentImpulses[I];
			Result.DeltaVelocity = Linear - Velocity;
			Result.DeltaAngularVelocity = Angular - AngularVelocity;
			return Result;
		}
	}
	Result.bSolved = false;
	return Result;
}
}
