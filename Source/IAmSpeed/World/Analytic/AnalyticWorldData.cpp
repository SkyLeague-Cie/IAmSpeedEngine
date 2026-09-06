#include "AnalyticWorldData.h"
#include "TensorContactTopology.h"
#include "AnalyticWorldQuery.h"

#include "Algo/AllOf.h"
#include "Algo/AnyOf.h"
#include "Algo/Sort.h"
#include "Async/ParallelFor.h"

namespace Speed::Analytic
{
namespace
{
	constexpr uint64 FnvOffset = 14695981039346656037ull;
	constexpr uint64 FnvPrime = 1099511628211ull;

	void HashBytes(uint64& Hash, const void* Data, SIZE_T Size)
	{
		const uint8* Bytes = static_cast<const uint8*>(Data);
		for (SIZE_T Index = 0; Index < Size; ++Index)
		{
			Hash ^= Bytes[Index];
			Hash *= FnvPrime;
		}
	}

	template <typename T>
	void HashValue(uint64& Hash, const T& Value)
	{
		HashBytes(Hash, &Value, sizeof(T));
	}

	bool IsFiniteVector(const FVector3d& Value)
	{
		return FMath::IsFinite(Value.X) && FMath::IsFinite(Value.Y) &&
			FMath::IsFinite(Value.Z);
	}

	struct FCubicBezierSegment
	{
		FVector3d ControlPoints[4] = {};
	};

	struct FQuinticBezierSegment
	{
		FVector3d ControlPoints[6] = {};
	};

	bool BuildNaturalCubicBezierSegments(const TArray<double>& Knots,
		const TArray<FVector3d>& Values, TArray<FCubicBezierSegment>& OutSegments)
	{
		OutSegments.Reset();
		const int32 Count = Knots.Num();
		if (Count < 2 || Values.Num() != Count) return false;
		TArray<FVector3d> SecondDerivatives;
		SecondDerivatives.SetNumZeroed(Count);
		for (int32 Index = 1; Index < Count; ++Index)
			if (!(Knots[Index] > Knots[Index - 1])) return false;
		for (int32 Coordinate = 0; Coordinate < 3; ++Coordinate)
		{
			TArray<double> Lower, Diagonal, Upper, Right;
			Lower.SetNumZeroed(Count);
			Diagonal.SetNumZeroed(Count);
			Upper.SetNumZeroed(Count);
			Right.SetNumZeroed(Count);
			Diagonal[0] = Diagonal[Count - 1] = 1.0;
			for (int32 Index = 1; Index + 1 < Count; ++Index)
			{
				const double PreviousH = Knots[Index] - Knots[Index - 1];
				const double NextH = Knots[Index + 1] - Knots[Index];
				Lower[Index] = PreviousH;
				Diagonal[Index] = 2.0 * (PreviousH + NextH);
				Upper[Index] = NextH;
				Right[Index] = 6.0 * ((Values[Index + 1][Coordinate] -
					Values[Index][Coordinate]) / NextH -
					(Values[Index][Coordinate] - Values[Index - 1][Coordinate]) /
					PreviousH);
			}
			for (int32 Index = 1; Index < Count; ++Index)
			{
				const double Factor = Lower[Index] / Diagonal[Index - 1];
				Diagonal[Index] -= Factor * Upper[Index - 1];
				Right[Index] -= Factor * Right[Index - 1];
			}
			SecondDerivatives[Count - 1][Coordinate] =
				Right[Count - 1] / Diagonal[Count - 1];
			for (int32 Index = Count - 2; Index >= 0; --Index)
				SecondDerivatives[Index][Coordinate] = (Right[Index] - Upper[Index] *
					SecondDerivatives[Index + 1][Coordinate]) / Diagonal[Index];
		}
		OutSegments.SetNum(Count - 1);
		for (int32 Index = 0; Index + 1 < Count; ++Index)
		{
			const double H = Knots[Index + 1] - Knots[Index];
			const FVector3d Delta = (Values[Index + 1] - Values[Index]) / H;
			const FVector3d StartDerivative = Delta - H *
				(2.0 * SecondDerivatives[Index] + SecondDerivatives[Index + 1]) / 6.0;
			const FVector3d EndDerivative = Delta + H *
				(SecondDerivatives[Index] + 2.0 * SecondDerivatives[Index + 1]) / 6.0;
			FCubicBezierSegment& Segment = OutSegments[Index];
			Segment.ControlPoints[0] = Values[Index];
			Segment.ControlPoints[1] = Values[Index] + H * StartDerivative / 3.0;
			Segment.ControlPoints[2] = Values[Index + 1] - H * EndDerivative / 3.0;
			Segment.ControlPoints[3] = Values[Index + 1];
		}
		return true;
	}

	// Builds a C2 cubic chain from a locally frozen prefix into an arbitrary
	// continuation.  The natural spline used by the general source fitter is
	// global: its upper boundary condition can change derivatives many cells
	// below the first varying value.  That is unacceptable for the lower cage
	// approach, which must remain an exact longitudinal extrusion.  We therefore
	// solve the frozen prefix independently, then propagate the endpoint
	// derivative/curvature forward through the continuation.  The recurrence is
	// the standard cubic-spline C2 relation and introduces no global coupling.
	bool BuildForwardC2BezierSegments(const TArray<double>& Knots,
		const TArray<FVector3d>& Values, const int32 FrozenThroughIndex,
		TArray<FCubicBezierSegment>& OutSegments)
	{
		OutSegments.Reset();
		const int32 Count = Knots.Num();
		if (Count < 2 || Values.Num() != Count || FrozenThroughIndex < 1 ||
			FrozenThroughIndex >= Count) return false;
		for (int32 Index = 1; Index < Count; ++Index)
			if (!(Knots[Index] > Knots[Index - 1])) return false;

		TArray<double> PrefixKnots;
		TArray<FVector3d> PrefixValues;
		PrefixKnots.Reserve(FrozenThroughIndex + 1);
		PrefixValues.Reserve(FrozenThroughIndex + 1);
		for (int32 Index = 0; Index <= FrozenThroughIndex; ++Index)
		{
			PrefixKnots.Add(Knots[Index]);
			PrefixValues.Add(Values[Index]);
		}
		TArray<FCubicBezierSegment> PrefixSegments;
		if (!BuildNaturalCubicBezierSegments(PrefixKnots, PrefixValues,
			PrefixSegments) || PrefixSegments.Num() != FrozenThroughIndex)
			return false;
		OutSegments = PrefixSegments;

		const double PrefixH = Knots[FrozenThroughIndex] -
			Knots[FrozenThroughIndex - 1];
		FVector3d StartDerivative =
			(PrefixSegments.Last().ControlPoints[3] -
				PrefixSegments.Last().ControlPoints[2]) * (3.0 / PrefixH);
		FVector3d StartSecondDerivative =
			(PrefixSegments.Last().ControlPoints[3] -
				2.0 * PrefixSegments.Last().ControlPoints[2] +
				PrefixSegments.Last().ControlPoints[1]) * (6.0 /
					FMath::Square(PrefixH));

		for (int32 Index = FrozenThroughIndex; Index + 1 < Count; ++Index)
		{
			const double H = Knots[Index + 1] - Knots[Index];
			const FVector3d Delta = (Values[Index + 1] - Values[Index]) / H;
			// Given the start derivative and second derivative, this is the
			// unique endpoint curvature for a cubic reaching the next value.
			const FVector3d EndSecondDerivative =
				(6.0 * (Delta - StartDerivative) / H) -
				2.0 * StartSecondDerivative;
			const FVector3d EndDerivative = Delta + H *
				(StartSecondDerivative + 2.0 * EndSecondDerivative) / 6.0;
			FCubicBezierSegment& Segment = OutSegments.AddDefaulted_GetRef();
			Segment.ControlPoints[0] = Values[Index];
			Segment.ControlPoints[1] = Values[Index] + H *
				StartDerivative / 3.0;
			Segment.ControlPoints[2] = Values[Index + 1] - H *
				EndDerivative / 3.0;
			Segment.ControlPoints[3] = Values[Index + 1];
			StartDerivative = EndDerivative;
			StartSecondDerivative = EndSecondDerivative;
		}
		return OutSegments.Num() == Count - 1;
	}

	// Local C2 interpolation for a vertical profile.  Every knot shares the
	// same first derivative and zero second derivative with its neighbours, so
	// the resulting quintic cells are C2 without a global boundary-condition
	// solve.  A frozen prefix gets derivatives computed only from that prefix;
	// hence upper source variation cannot tilt the lower extrusion.  X/Z remain
	// affine in each interval (preserving a regular vertical parameter), while
	// the transverse coordinate follows the local Hermite quintic.
	bool BuildLocalC2QuinticSegments(const TArray<double>& Knots,
		const TArray<FVector3d>& Values, const int32 FrozenThroughIndex,
		const int32 TransverseAxis, const int32 VerticalAxis,
		TArray<FQuinticBezierSegment>& OutSegments)
	{
		OutSegments.Reset();
		const int32 Count = Knots.Num();
		if (Count < 2 || Values.Num() != Count || FrozenThroughIndex < 1 ||
			FrozenThroughIndex >= Count || TransverseAxis < 0 ||
			TransverseAxis > 2 || VerticalAxis < 0 || VerticalAxis > 2 ||
			TransverseAxis == VerticalAxis) return false;
		for (int32 Index = 1; Index < Count; ++Index)
			if (!(Knots[Index] > Knots[Index - 1])) return false;

		TArray<FVector3d> Derivatives;
		Derivatives.SetNumZeroed(Count);
		for (int32 Index = 0; Index < Count; ++Index)
		{
			int32 Left = Index - 1;
			int32 Right = Index + 1;
			if (Index == 0)
			{
				Left = 0;
				Right = 1;
			}
			if (Index == Count - 1)
			{
				Left = Count - 2;
				Right = Count - 1;
			}
			// At the frozen/active seam use only the frozen-side slope.  In
			// particular, a constant lower transverse profile has exactly zero
			// transverse derivative at the seam.
			if (Index == FrozenThroughIndex)
			{
				Left = FMath::Max(0, Index - 1);
				Right = Index;
			}
			const double H = Knots[Right] - Knots[Left];
			if (!(H > 0.0)) return false;
			Derivatives[Index] = (Values[Right] - Values[Left]) / H;
		}
		for (int32 Index = 0; Index < FrozenThroughIndex; ++Index)
		{
			const double H = Knots[Index + 1] - Knots[Index];
			Derivatives[Index][TransverseAxis] =
				(Index + 1 <= FrozenThroughIndex) ?
					(Values[Index + 1][TransverseAxis] -
						Values[Index][TransverseAxis]) / H : 0.0;
		}
		Derivatives[FrozenThroughIndex][TransverseAxis] = 0.0;
		OutSegments.Reserve(Count - 1);
		for (int32 Index = 0; Index + 1 < Count; ++Index)
		{
			const double H = Knots[Index + 1] - Knots[Index];
			const FVector3d& Start = Values[Index];
			const FVector3d& End = Values[Index + 1];
			const FVector3d& StartDerivative = Derivatives[Index];
			const FVector3d& EndDerivative = Derivatives[Index + 1];
			FQuinticBezierSegment& Segment = OutSegments.AddDefaulted_GetRef();
			Segment.ControlPoints[0] = Start;
			Segment.ControlPoints[5] = End;
			// The affine coordinates are degree-elevated line segments.
			for (int32 Coordinate = 0; Coordinate < 3; ++Coordinate)
			{
				if (Coordinate == TransverseAxis) continue;
				const double Delta = End[Coordinate] - Start[Coordinate];
				for (int32 Control = 1; Control < 5; ++Control)
					Segment.ControlPoints[Control][Coordinate] =
						Start[Coordinate] + Delta * (static_cast<double>(Control) / 5.0);
			}
			// Quintic endpoint Hermite controls for zero second derivative.
			Segment.ControlPoints[1][TransverseAxis] =
				Start[TransverseAxis] + H * StartDerivative[TransverseAxis] / 5.0;
			Segment.ControlPoints[2][TransverseAxis] =
				Start[TransverseAxis] + 2.0 * H *
					StartDerivative[TransverseAxis] / 5.0;
			Segment.ControlPoints[4][TransverseAxis] =
				End[TransverseAxis] - H * EndDerivative[TransverseAxis] / 5.0;
			Segment.ControlPoints[3][TransverseAxis] =
				End[TransverseAxis] - 2.0 * H *
					EndDerivative[TransverseAxis] / 5.0;
		}
		return OutSegments.Num() == Count - 1;
	}

	void Bernstein5Basis(const double Parameter, double OutBasis[6])
	{
		const double U = FMath::Clamp(Parameter, 0.0, 1.0);
		const double V = 1.0 - U;
		OutBasis[0] = V * V * V * V * V;
		OutBasis[1] = 5.0 * U * V * V * V * V;
		OutBasis[2] = 10.0 * U * U * V * V * V;
		OutBasis[3] = 10.0 * U * U * U * V * V;
		OutBasis[4] = 5.0 * U * U * U * U * V;
		OutBasis[5] = U * U * U * U * U;
	}

	double EvaluateBernstein5Field(
		const double Coefficients[6], const double Parameter)
	{
		double Basis[6];
		Bernstein5Basis(Parameter, Basis);
		double Result = 0.0;
		for (int32 Index = 0; Index < 6; ++Index)
		{
			Result += Basis[Index] * Coefficients[Index];
		}
		return Result;
	}

	FVector2d EvaluateBernstein5Curve(
		const FVector2d ControlPoints[6], const double Parameter)
	{
		double Basis[6];
		Bernstein5Basis(Parameter, Basis);
		FVector2d Result = FVector2d::ZeroVector;
		for (int32 Index = 0; Index < 6; ++Index)
		{
			Result += Basis[Index] * ControlPoints[Index];
		}
		return Result;
	}

	void CanonicalTransitionBasis(const double S,
		const double Start, const double Width, double OutBasis[4])
	{
		for (int32 Index = 0; Index < 4; ++Index) OutBasis[Index] = 0.0;
		if (Width <= 0.0 || S <= Start || S >= Start + Width) return;
		const double Q = (S - Start) / Width;
		const double U = 1.0 - Q;
		const double Envelope = 1024.0 * FMath::Pow(Q, 5.0) *
			FMath::Pow(U, 5.0);
		OutBasis[0] = Envelope * U * U * U;
		OutBasis[1] = Envelope * 3.0 * U * U * Q;
		OutBasis[2] = Envelope * 3.0 * U * Q * Q;
		OutBasis[3] = Envelope * Q * Q * Q;
	}

	double EvaluateCanonicalTransitionField(const double Coefficients[4],
		const double S, const double Start, const double Width)
	{
		double Basis[4];
		CanonicalTransitionBasis(S, Start, Width, Basis);
		double Result = 0.0;
		for (int32 Index = 0; Index < 4; ++Index)
		{
			Result += Basis[Index] * Coefficients[Index];
		}
		return Result;
	}

	double BinomialCoefficient(const int32 N, const int32 K)
	{
		const int32 ReducedK = FMath::Min(K, N - K);
		double Result = 1.0;
		for (int32 Index = 1; Index <= ReducedK; ++Index)
		{
			Result *= static_cast<double>(N - ReducedK + Index) / Index;
		}
		return Result;
	}

	double BernsteinBasisValue(
		const int32 Degree, const int32 Index, const double Parameter)
	{
		const double T = FMath::Clamp(Parameter, 0.0, 1.0);
		return BinomialCoefficient(Degree, Index) * FMath::Pow(T, Index) *
			FMath::Pow(1.0 - T, Degree - Index);
	}

	bool BuildBernsteinCollocationInverse(
		const int32 Degree, TArray<double>& OutInverse)
	{
		const int32 Count = Degree + 1;
		TArray<double> Augmented;
		Augmented.Init(0.0, Count * 2 * Count);
		auto At = [&](const int32 Row, const int32 Column) -> double&
		{
			return Augmented[Row * 2 * Count + Column];
		};
		for (int32 Row = 0; Row < Count; ++Row)
		{
			const double Parameter = static_cast<double>(Row) / Degree;
			for (int32 Column = 0; Column < Count; ++Column)
			{
				At(Row, Column) = BernsteinBasisValue(Degree, Column, Parameter);
			}
			At(Row, Count + Row) = 1.0;
		}
		for (int32 Diagonal = 0; Diagonal < Count; ++Diagonal)
		{
			int32 Pivot = Diagonal;
			for (int32 Row = Diagonal + 1; Row < Count; ++Row)
			{
				if (FMath::Abs(At(Row, Diagonal)) >
					FMath::Abs(At(Pivot, Diagonal))) Pivot = Row;
			}
			if (FMath::Abs(At(Pivot, Diagonal)) <= 1.0e-14) return false;
			if (Pivot != Diagonal)
			{
				for (int32 Column = 0; Column < 2 * Count; ++Column)
				{
					Swap(At(Pivot, Column), At(Diagonal, Column));
				}
			}
			const double InversePivot = 1.0 / At(Diagonal, Diagonal);
			for (int32 Column = 0; Column < 2 * Count; ++Column)
			{
				At(Diagonal, Column) *= InversePivot;
			}
			for (int32 Row = 0; Row < Count; ++Row)
			{
				if (Row == Diagonal) continue;
				const double Factor = At(Row, Diagonal);
				for (int32 Column = 0; Column < 2 * Count; ++Column)
				{
					At(Row, Column) -= Factor * At(Diagonal, Column);
				}
			}
		}
		OutInverse.SetNumUninitialized(Count * Count);
		for (int32 Row = 0; Row < Count; ++Row)
		{
			for (int32 Column = 0; Column < Count; ++Column)
			{
				OutInverse[Row * Count + Column] = At(Row, Count + Column);
			}
		}
		return true;
	}

	bool BuildTensorSurfaceFromPolynomialEvaluator(
		const int32 DegreeU, const int32 DegreeV,
		TFunctionRef<FVector3d(double, double)> Evaluate,
		FTensorBezierSurface& OutSurface, double& OutMaximumErrorCm,
		const bool bRequireExactInterpolation = true)
	{
		TArray<double> InverseU;
		TArray<double> InverseV;
		if (!BuildBernsteinCollocationInverse(DegreeU, InverseU) ||
			!BuildBernsteinCollocationInverse(DegreeV, InverseV))
		{
			return false;
		}
		const int32 UCount = DegreeU + 1;
		const int32 VCount = DegreeV + 1;
		TArray<FVector3d> Samples;
		Samples.SetNumUninitialized(UCount * VCount);
		for (int32 UIndex = 0; UIndex < UCount; ++UIndex)
		{
			for (int32 VIndex = 0; VIndex < VCount; ++VIndex)
			{
				Samples[UIndex * VCount + VIndex] = Evaluate(
					static_cast<double>(UIndex) / DegreeU,
					static_cast<double>(VIndex) / DegreeV);
			}
		}
		OutSurface.DegreeU = DegreeU;
		OutSurface.DegreeV = DegreeV;
		OutSurface.ControlPoints.SetNumZeroed(UCount * VCount);
		for (int32 UControl = 0; UControl < UCount; ++UControl)
		{
			for (int32 VControl = 0; VControl < VCount; ++VControl)
			{
				FVector3d& Control =
					OutSurface.ControlPoints[UControl * VCount + VControl];
				for (int32 USample = 0; USample < UCount; ++USample)
				{
					for (int32 VSample = 0; VSample < VCount; ++VSample)
					{
						Control += InverseU[UControl * UCount + USample] *
							InverseV[VControl * VCount + VSample] *
							Samples[USample * VCount + VSample];
					}
				}
			}
		}
		OutMaximumErrorCm = 0.0;
		for (int32 UIndex = 0; UIndex <= 32; ++UIndex)
		{
			const double U = static_cast<double>(UIndex) / 32.0;
			for (int32 VIndex = 0; VIndex <= 32; ++VIndex)
			{
				const double V = static_cast<double>(VIndex) / 32.0;
				OutMaximumErrorCm = FMath::Max(OutMaximumErrorCm,
					FVector3d::Distance(Evaluate(U, V), OutSurface.Evaluate(U, V)));
			}
		}
		return OutSurface.IsValid() && (!bRequireExactInterpolation ||
			OutMaximumErrorCm <= 1.0e-6);
	}

	double BezierChordErrorCm(const FVector3d ControlPoints[8])
	{
		const FVector3d Chord = ControlPoints[7] - ControlPoints[0];
		double MaximumErrorSquared = 0.0;
		for (int32 Index = 1; Index < 7; ++Index)
		{
			// Represent the chord as a degree-seven Bezier with uniformly spaced
			// control points. The convex-hull property applied to the difference
			// curve bounds the pointwise deviation in both directions.
			const double Alpha = static_cast<double>(Index) / 7.0;
			const FVector3d Closest = ControlPoints[0] + Alpha * Chord;
			MaximumErrorSquared = FMath::Max(MaximumErrorSquared,
				(ControlPoints[Index] - Closest).SquaredLength());
		}
		return FMath::Sqrt(MaximumErrorSquared);
	}

	void SplitBezierHalf(
		const FVector3d ControlPoints[8], FVector3d Left[8], FVector3d Right[8])
	{
		FVector3d Work[8];
		for (int32 Index = 0; Index < 8; ++Index) Work[Index] = ControlPoints[Index];
		Left[0] = Work[0];
		Right[7] = Work[7];
		for (int32 Level = 1; Level < 8; ++Level)
		{
			for (int32 Index = 0; Index < 8 - Level; ++Index)
			{
				Work[Index] = 0.5 * (Work[Index] + Work[Index + 1]);
			}
			Left[Level] = Work[0];
			Right[7 - Level] = Work[7 - Level];
		}
	}

	bool AppendCertifiedBezierChords(
		const FVector3d ControlPoints[8], const double T0, const double T1,
		const double ToleranceCm, const int32 Depth,
		TArray<FVector3d>& Points, TArray<double>& Parameters,
		double& MaximumAcceptedErrorCm)
	{
		const double ErrorCm = BezierChordErrorCm(ControlPoints);
		if (ErrorCm <= ToleranceCm)
		{
			Points.Add(ControlPoints[7]);
			Parameters.Add(T1);
			MaximumAcceptedErrorCm = FMath::Max(
				MaximumAcceptedErrorCm, ErrorCm);
			return true;
		}
		constexpr int32 MaximumSubdivisionDepth = 24;
		if (Depth >= MaximumSubdivisionDepth) return false;
		FVector3d Left[8];
		FVector3d Right[8];
		SplitBezierHalf(ControlPoints, Left, Right);
		const double MiddleT = 0.5 * (T0 + T1);
		return AppendCertifiedBezierChords(Left, T0, MiddleT, ToleranceCm,
			Depth + 1, Points, Parameters, MaximumAcceptedErrorCm) &&
			AppendCertifiedBezierChords(Right, MiddleT, T1, ToleranceCm,
				Depth + 1, Points, Parameters, MaximumAcceptedErrorCm);
	}

	double Cross2D(const FVector2d& A, const FVector2d& B)
	{
		return A.X * B.Y - A.Y * B.X;
	}

	bool PointOnSegment2D(
		const FVector2d& Point, const FVector2d& A, const FVector2d& B)
	{
		constexpr double Tolerance = 1.0e-12;
		return FMath::Abs(Cross2D(B - A, Point - A)) <= Tolerance &&
			Point.X >= FMath::Min(A.X, B.X) - Tolerance &&
			Point.X <= FMath::Max(A.X, B.X) + Tolerance &&
			Point.Y >= FMath::Min(A.Y, B.Y) - Tolerance &&
			Point.Y <= FMath::Max(A.Y, B.Y) + Tolerance;
	}

	bool SegmentsIntersect2D(
		const FVector2d& A, const FVector2d& B,
		const FVector2d& C, const FVector2d& D)
	{
		const double AbC = Cross2D(B - A, C - A);
		const double AbD = Cross2D(B - A, D - A);
		const double CdA = Cross2D(D - C, A - C);
		const double CdB = Cross2D(D - C, B - C);
		if ((AbC > 0.0) != (AbD > 0.0) && (CdA > 0.0) != (CdB > 0.0))
		{
			return true;
		}
		return PointOnSegment2D(C, A, B) || PointOnSegment2D(D, A, B) ||
			PointOnSegment2D(A, C, D) || PointOnSegment2D(B, C, D);
	}

	double TriangleCentroidAxis(const FTriangleSurface& Triangle, const int32 Axis)
	{
		return (Triangle.Vertices[0][Axis] + Triangle.Vertices[1][Axis] +
			Triangle.Vertices[2][Axis]) / 3.0;
	}

	struct FVertexUse
	{
		uint64 SourceId = 0;
		FVector3d Position = FVector3d::ZeroVector;
		int32 TriangleIndex = INDEX_NONE;
		int32 CornerIndex = INDEX_NONE;
	};

	struct FEdgeUse
	{
		int32 VertexA = INDEX_NONE;
		int32 VertexB = INDEX_NONE;
		int32 TriangleIndex = INDEX_NONE;
		int32 LocalEdgeIndex = INDEX_NONE;
	};

	double VectorAngleDegrees(const FVector3d& A, const FVector3d& B)
	{
		return FMath::RadiansToDegrees(FMath::Acos(FMath::Clamp(
			FVector3d::DotProduct(A, B), -1.0, 1.0)));
	}

	void BuildQuinticTransitionControlPoints(
		const FVector2d& Center, const double RadiusU, const double RadiusV,
		const double SignU, const double SignV, const double FlatteningFraction,
		FVector2d OutControlPoints[6])
	{
		const FVector2d EndpointV(Center.X, Center.Y + SignV * RadiusV);
		const FVector2d EndpointU(Center.X + SignU * RadiusU, Center.Y);
		OutControlPoints[0] = EndpointV;
		OutControlPoints[1] = EndpointV + FVector2d(
			SignU * FlatteningFraction * RadiusU, 0.0);
		OutControlPoints[2] = EndpointV + FVector2d(
			SignU * 2.0 * FlatteningFraction * RadiusU, 0.0);
		OutControlPoints[3] = EndpointU + FVector2d(
			0.0, SignV * 2.0 * FlatteningFraction * RadiusV);
		OutControlPoints[4] = EndpointU + FVector2d(
			0.0, SignV * FlatteningFraction * RadiusV);
		OutControlPoints[5] = EndpointU;
	}

	FVector2d EvaluateQuinticTransitionPosition(
		const FVector2d& Center, const double RadiusU, const double RadiusV,
		const double SignU, const double SignV, const double FlatteningFraction,
		const double T)
	{
		FVector2d ControlPoints[6];
		BuildQuinticTransitionControlPoints(Center, RadiusU, RadiusV,
			SignU, SignV, FlatteningFraction, ControlPoints);
		const double OneMinusT = 1.0 - T;
		const double Basis[6] = {
			OneMinusT * OneMinusT * OneMinusT * OneMinusT * OneMinusT,
			5.0 * T * OneMinusT * OneMinusT * OneMinusT * OneMinusT,
			10.0 * T * T * OneMinusT * OneMinusT * OneMinusT,
			10.0 * T * T * T * OneMinusT * OneMinusT,
			5.0 * T * T * T * T * OneMinusT,
			T * T * T * T * T };
		FVector2d Result = FVector2d::ZeroVector;
		for (int32 Index = 0; Index < 6; ++Index)
		{
			Result += Basis[Index] * ControlPoints[Index];
		}
		return Result;
	}

	double ClosestQuinticTransitionParameter(
		const FVector2d& Point, const FVector2d& Center,
		const double RadiusU, const double RadiusV, const double SignU,
		const double SignV, const double FlatteningFraction)
	{
		auto EvaluateSquaredDistance = [&](const double T)
		{
			return FVector2d::DistSquared(Point,
				EvaluateQuinticTransitionPosition(Center, RadiusU, RadiusV,
					SignU, SignV, FlatteningFraction, T));
		};
		constexpr int32 SampleCount = 64;
		int32 BestSample = 0;
		double BestSquaredDistance = EvaluateSquaredDistance(0.0);
		for (int32 Sample = 1; Sample <= SampleCount; ++Sample)
		{
			const double SquaredDistance = EvaluateSquaredDistance(
				static_cast<double>(Sample) / static_cast<double>(SampleCount));
			if (SquaredDistance < BestSquaredDistance)
			{
				BestSquaredDistance = SquaredDistance;
				BestSample = Sample;
			}
		}
		double Lower = static_cast<double>(FMath::Max(0, BestSample - 1)) /
			static_cast<double>(SampleCount);
		double Upper = static_cast<double>(FMath::Min(SampleCount, BestSample + 1)) /
			static_cast<double>(SampleCount);
		for (int32 Iteration = 0; Iteration < 24; ++Iteration)
		{
			const double Left = (2.0 * Lower + Upper) / 3.0;
			const double Right = (Lower + 2.0 * Upper) / 3.0;
			if (EvaluateSquaredDistance(Left) <= EvaluateSquaredDistance(Right))
			{
				Upper = Right;
			}
			else
			{
				Lower = Left;
			}
		}
		const double Refined = 0.5 * (Lower + Upper);
		return EvaluateSquaredDistance(Refined) < BestSquaredDistance
			? Refined
			: static_cast<double>(BestSample) / static_cast<double>(SampleCount);
	}

	double SquaredDistanceToQuinticTransition(
		const FVector2d& Point, const FVector2d& Center,
		const double RadiusU, const double RadiusV, const double SignU,
		const double SignV, const double FlatteningFraction)
	{
		const double T = ClosestQuinticTransitionParameter(
			Point, Center, RadiusU, RadiusV, SignU, SignV,
			FlatteningFraction);
		return FVector2d::DistSquared(Point,
			EvaluateQuinticTransitionPosition(Center, RadiusU, RadiusV,
				SignU, SignV, FlatteningFraction, T));
	}

	double SquaredDistanceToQuinticTransitionInDualBasis(
		const FVector2d& Point, const FVector2d& Center,
		const double RadiusU, const double RadiusV, const double SignU,
		const double SignV, const double FlatteningFraction,
		const FVector3d& DualU, const FVector3d& DualV)
	{
		auto EvaluateSquaredDistance = [&](const double T)
		{
			const FVector2d Delta = Point - EvaluateQuinticTransitionPosition(
				Center, RadiusU, RadiusV, SignU, SignV,
				FlatteningFraction, T);
			return (Delta.X * DualU + Delta.Y * DualV).SquaredLength();
		};
		constexpr int32 SampleCount = 64;
		int32 BestSample = 0;
		double BestSquaredDistance = EvaluateSquaredDistance(0.0);
		for (int32 Sample = 1; Sample <= SampleCount; ++Sample)
		{
			const double SquaredDistance = EvaluateSquaredDistance(
				static_cast<double>(Sample) / static_cast<double>(SampleCount));
			if (SquaredDistance < BestSquaredDistance)
			{
				BestSquaredDistance = SquaredDistance;
				BestSample = Sample;
			}
		}
		double Lower = static_cast<double>(FMath::Max(0, BestSample - 1)) /
			static_cast<double>(SampleCount);
		double Upper = static_cast<double>(FMath::Min(SampleCount, BestSample + 1)) /
			static_cast<double>(SampleCount);
		for (int32 Iteration = 0; Iteration < 24; ++Iteration)
		{
			const double Left = (2.0 * Lower + Upper) / 3.0;
			const double Right = (Lower + 2.0 * Upper) / 3.0;
			if (EvaluateSquaredDistance(Left) <= EvaluateSquaredDistance(Right))
			{
				Upper = Right;
			}
			else
			{
				Lower = Left;
			}
		}
		return FMath::Min(BestSquaredDistance,
			EvaluateSquaredDistance(0.5 * (Lower + Upper)));
	}

	FVector3d CanonicalPlaneNormal(FVector3d Normal)
	{
		Normal = Normal.GetSafeNormal();
		constexpr double SignTolerance = 1.0e-12;
		const bool bFlip = Normal.X < -SignTolerance ||
			(FMath::Abs(Normal.X) <= SignTolerance && Normal.Y < -SignTolerance) ||
			(FMath::Abs(Normal.X) <= SignTolerance &&
				FMath::Abs(Normal.Y) <= SignTolerance && Normal.Z < 0.0);
		return bFlip ? -Normal : Normal;
	}

	bool SolveSymmetricNormalEquations(
		const double Matrix[3][3], const double RightHandSide[3], double Out[3])
	{
		double Augmented[3][4];
		for (int32 Row = 0; Row < 3; ++Row)
		{
			for (int32 Column = 0; Column < 3; ++Column)
			{
				Augmented[Row][Column] = Matrix[Row][Column];
			}
			Augmented[Row][3] = RightHandSide[Row];
		}
		for (int32 PivotColumn = 0; PivotColumn < 3; ++PivotColumn)
		{
			int32 PivotRow = PivotColumn;
			for (int32 Row = PivotColumn + 1; Row < 3; ++Row)
			{
				if (FMath::Abs(Augmented[Row][PivotColumn]) >
					FMath::Abs(Augmented[PivotRow][PivotColumn]))
				{
					PivotRow = Row;
				}
			}
			if (FMath::Abs(Augmented[PivotRow][PivotColumn]) <= 1.0e-12)
			{
				return false;
			}
			if (PivotRow != PivotColumn)
			{
				for (int32 Column = PivotColumn; Column < 4; ++Column)
				{
					Swap(Augmented[PivotRow][Column], Augmented[PivotColumn][Column]);
				}
			}
			const double Pivot = Augmented[PivotColumn][PivotColumn];
			for (int32 Column = PivotColumn; Column < 4; ++Column)
			{
				Augmented[PivotColumn][Column] /= Pivot;
			}
			for (int32 Row = 0; Row < 3; ++Row)
			{
				if (Row == PivotColumn) continue;
				const double Factor = Augmented[Row][PivotColumn];
				for (int32 Column = PivotColumn; Column < 4; ++Column)
				{
					Augmented[Row][Column] -= Factor * Augmented[PivotColumn][Column];
				}
			}
		}
		for (int32 Row = 0; Row < 3; ++Row) Out[Row] = Augmented[Row][3];
		return true;
	}

	double SquaredDistanceToBox(const FVector3d& Point, const FBox3d& Box)
	{
		double Result = 0.0;
		for (int32 Axis = 0; Axis < 3; ++Axis)
		{
			const double Delta = Point[Axis] < Box.Min[Axis]
				? Box.Min[Axis] - Point[Axis]
				: (Point[Axis] > Box.Max[Axis] ? Point[Axis] - Box.Max[Axis] : 0.0);
			Result += Delta * Delta;
		}
		return Result;
	}

	FVector3d ClosestPointOnTriangle(
		const FVector3d& Point, const FTriangleSurface& Triangle)
	{
		const FVector3d& A = Triangle.Vertices[0];
		const FVector3d& B = Triangle.Vertices[1];
		const FVector3d& C = Triangle.Vertices[2];
		const FVector3d AB = B - A;
		const FVector3d AC = C - A;
		const FVector3d AP = Point - A;
		const double D1 = FVector3d::DotProduct(AB, AP);
		const double D2 = FVector3d::DotProduct(AC, AP);
		if (D1 <= 0.0 && D2 <= 0.0) return A;

		const FVector3d BP = Point - B;
		const double D3 = FVector3d::DotProduct(AB, BP);
		const double D4 = FVector3d::DotProduct(AC, BP);
		if (D3 >= 0.0 && D4 <= D3) return B;

		const double VC = D1 * D4 - D3 * D2;
		if (VC <= 0.0 && D1 >= 0.0 && D3 <= 0.0)
		{
			return A + (D1 / (D1 - D3)) * AB;
		}

		const FVector3d CP = Point - C;
		const double D5 = FVector3d::DotProduct(AB, CP);
		const double D6 = FVector3d::DotProduct(AC, CP);
		if (D6 >= 0.0 && D5 <= D6) return C;

		const double VB = D5 * D2 - D1 * D6;
		if (VB <= 0.0 && D2 >= 0.0 && D6 <= 0.0)
		{
			return A + (D2 / (D2 - D6)) * AC;
		}

		const double VA = D3 * D6 - D5 * D4;
		if (VA <= 0.0 && D4 - D3 >= 0.0 && D5 - D6 >= 0.0)
		{
			return B + ((D4 - D3) / ((D4 - D3) + (D5 - D6))) * (C - B);
		}

		const double Denominator = 1.0 / (VA + VB + VC);
		return A + (VB * Denominator) * AB + (VC * Denominator) * AC;
	}
}

bool FTriangleSurface::IsValid(FString* OutReason) const
{
	auto Fail = [OutReason](const TCHAR* Reason)
	{
		if (OutReason)
		{
			*OutReason = Reason;
		}
		return false;
	};
	if (SourceId == 0 || SurfaceId == 0 || FeatureId == 0 || PrimitiveId == 0)
	{
		return Fail(TEXT("Triangle identifiers must be non-zero."));
	}
	for (int32 Corner = 0; Corner < 3; ++Corner)
	{
		if (!IsFiniteVector(Vertices[Corner]) || !IsFiniteVector(VertexNormals[Corner]))
		{
			return Fail(TEXT("Triangle vertices and normals must be finite."));
		}
	}
	if (!IsFiniteVector(FaceNormal) ||
		!FMath::IsNearlyEqual(FaceNormal.SquaredLength(), 1.0, 1.0e-9))
	{
		return Fail(TEXT("Triangle face normal must be finite and normalized."));
	}
	const FVector3d Cross = FVector3d::CrossProduct(
		Vertices[1] - Vertices[0], Vertices[2] - Vertices[0]);
	if (Cross.SquaredLength() <= 1.0e-18 || !Bounds.IsValid)
	{
		return Fail(TEXT("Triangle must have non-zero area and valid bounds."));
	}
	return true;
}

FVector2d FC2TransitionSectionFit::EvaluatePosition(const double T) const
{
	return EvaluateQuinticTransitionPosition(CenterCoordinates, RadiusU, RadiusV,
		SignU, SignV, FlatteningFraction, FMath::Clamp(T, 0.0, 1.0));
}

FVector2d FC2TransitionSectionFit::EvaluateFirstDerivative(const double T) const
{
	FVector2d ControlPoints[6];
	BuildQuinticTransitionControlPoints(CenterCoordinates, RadiusU, RadiusV,
		SignU, SignV, FlatteningFraction, ControlPoints);
	const double ClampedT = FMath::Clamp(T, 0.0, 1.0);
	const double OneMinusT = 1.0 - ClampedT;
	const double Basis[5] = {
		OneMinusT * OneMinusT * OneMinusT * OneMinusT,
		4.0 * ClampedT * OneMinusT * OneMinusT * OneMinusT,
		6.0 * ClampedT * ClampedT * OneMinusT * OneMinusT,
		4.0 * ClampedT * ClampedT * ClampedT * OneMinusT,
		ClampedT * ClampedT * ClampedT * ClampedT };
	FVector2d Result = FVector2d::ZeroVector;
	for (int32 Index = 0; Index < 5; ++Index)
	{
		Result += 5.0 * Basis[Index] *
			(ControlPoints[Index + 1] - ControlPoints[Index]);
	}
	return Result;
}

FVector2d FC2TransitionSectionFit::EvaluateSecondDerivative(const double T) const
{
	FVector2d ControlPoints[6];
	BuildQuinticTransitionControlPoints(CenterCoordinates, RadiusU, RadiusV,
		SignU, SignV, FlatteningFraction, ControlPoints);
	const double ClampedT = FMath::Clamp(T, 0.0, 1.0);
	const double OneMinusT = 1.0 - ClampedT;
	const double Basis[4] = {
		OneMinusT * OneMinusT * OneMinusT,
		3.0 * ClampedT * OneMinusT * OneMinusT,
		3.0 * ClampedT * ClampedT * OneMinusT,
		ClampedT * ClampedT * ClampedT };
	FVector2d Result = FVector2d::ZeroVector;
	for (int32 Index = 0; Index < 4; ++Index)
	{
		Result += 20.0 * Basis[Index] *
			(ControlPoints[Index + 2] - 2.0 * ControlPoints[Index + 1] +
				ControlPoints[Index]);
	}
	return Result;
}

double FBoundedPlane::SignedDistance(const FVector3d& Point) const
{
	return FVector3d::DotProduct(Point - Origin, Normal);
}

FVector3d FBoundedPlane::ClosestPoint(const FVector3d& Point) const
{
	return Point - SignedDistance(Point) * Normal;
}

bool FBoundedPlane::ContainsProjectedSegment(const FVector3d& Start, const FVector3d& End) const
{
	if (!ContainsProjectedPoint(Start, 0) || !ContainsProjectedPoint(End, 0)) return false;
	// Rectangles are convex. A concave polygon needs an interior-crossing
	// guard: two supported endpoints could otherwise bridge a notch/hole.
	if (DomainVertices.IsEmpty() || Start == End) return true;
	const auto Local = [this](const FVector3d& P)
	{
		return FVector2d(FVector3d::DotProduct(P - Origin, AxisU), FVector3d::DotProduct(P - Origin, AxisV));
	};
	const FVector2d A = Local(Start), D = Local(End) - A;
	if (D.IsZero()) return true;
	const auto Cross = [](const FVector2d& P, const FVector2d& Q) { return P.X * Q.Y - P.Y * Q.X; };
	for (int32 I = 0; I < DomainVertices.Num(); ++I)
	{
		const FVector2d Q = DomainVertices[I] - A;
		const FVector2d E = DomainVertices[(I + 1) % DomainVertices.Num()] - DomainVertices[I];
		const double Denominator = Cross(D, E);
		if (Denominator != 0)
		{
			const double T = Cross(Q, E) / Denominator, U = Cross(Q, D) / Denominator;
			if (T > 0 && T < 1 && U >= 0 && U <= 1) return false;
		}
		else if (Cross(Q, D) == 0)
		{
			const double T0 = FVector2d::DotProduct(Q, D) / D.SizeSquared();
			const double T1 = FVector2d::DotProduct(Q + E, D) / D.SizeSquared();
			if (FMath::Max(T0, T1) > 0 && FMath::Min(T0, T1) < 1) return false;
		}
	}
	return true;
}

bool FBoundedPlane::ContainsProjectedPoint(
	const FVector3d& Point, const double Tolerance) const
{
	const FVector3d Relative = Point - Origin;
	const FVector2d LocalPoint(
		FVector3d::DotProduct(Relative, AxisU),
		FVector3d::DotProduct(Relative, AxisV));
	if (DomainVertices.IsEmpty())
	{
		return FMath::Abs(LocalPoint.X) <= HalfExtents.X + Tolerance &&
			FMath::Abs(LocalPoint.Y) <= HalfExtents.Y + Tolerance;
	}

	bool bInside = false;
	for (int32 Index = 0; Index < DomainVertices.Num(); ++Index)
	{
		const FVector2d& A = DomainVertices[Index];
		const FVector2d& B = DomainVertices[(Index + 1) % DomainVertices.Num()];
		const FVector2d Edge = B - A;
		const double EdgeLengthSquared = Edge.SquaredLength();
		if (EdgeLengthSquared > UE_DOUBLE_SMALL_NUMBER)
		{
			const double EdgeT = FMath::Clamp(
				FVector2d::DotProduct(LocalPoint - A, Edge) / EdgeLengthSquared,
				0.0, 1.0);
			if ((LocalPoint - (A + EdgeT * Edge)).SquaredLength() <=
				FMath::Square(FMath::Max(0.0, Tolerance)))
			{
				return true;
			}
		}
		const bool bCrosses = (A.Y > LocalPoint.Y) != (B.Y > LocalPoint.Y);
		if (bCrosses)
		{
			const double CrossingX = A.X +
				(LocalPoint.Y - A.Y) * (B.X - A.X) / (B.Y - A.Y);
			if (LocalPoint.X < CrossingX) bInside = !bInside;
		}
	}
	return bInside;
}

double FBoundedPlane::DistanceToDomainBoundary(const FVector3d& Point) const
{
	const FVector3d Relative = Point - Origin;
	const FVector2d LocalPoint(
		FVector3d::DotProduct(Relative, AxisU),
		FVector3d::DotProduct(Relative, AxisV));
	if (DomainVertices.IsEmpty())
	{
		return FMath::Min(
			HalfExtents.X - FMath::Abs(LocalPoint.X),
			HalfExtents.Y - FMath::Abs(LocalPoint.Y));
	}
	double MinimumDistanceSquared = TNumericLimits<double>::Max();
	for (int32 Index = 0; Index < DomainVertices.Num(); ++Index)
	{
		const FVector2d& A = DomainVertices[Index];
		const FVector2d& B = DomainVertices[(Index + 1) % DomainVertices.Num()];
		const FVector2d Edge = B - A;
		const double EdgeLengthSquared = Edge.SquaredLength();
		if (EdgeLengthSquared <= UE_DOUBLE_SMALL_NUMBER) continue;
		const double EdgeT = FMath::Clamp(
			FVector2d::DotProduct(LocalPoint - A, Edge) / EdgeLengthSquared,
			0.0, 1.0);
		MinimumDistanceSquared = FMath::Min(MinimumDistanceSquared,
			(LocalPoint - (A + EdgeT * Edge)).SquaredLength());
	}
	return MinimumDistanceSquared < TNumericLimits<double>::Max()
		? FMath::Sqrt(MinimumDistanceSquared) : 0.0;
}

bool FBoundedPlane::IsValid(FString* OutReason) const
{
	auto Fail = [OutReason](const TCHAR* Reason)
	{
		if (OutReason)
		{
			*OutReason = Reason;
		}
		return false;
	};

	if (SurfaceId == 0 || FeatureId == 0)
	{
		return Fail(TEXT("Surface and feature identifiers must be non-zero."));
	}
	if (!IsFiniteVector(Origin) || !IsFiniteVector(Normal) ||
		!IsFiniteVector(AxisU) || !IsFiniteVector(AxisV))
	{
		return Fail(TEXT("Plane vectors must be finite."));
	}
	if (!FMath::IsFinite(HalfExtents.X) || !FMath::IsFinite(HalfExtents.Y) ||
		HalfExtents.X <= 0.0 || HalfExtents.Y <= 0.0)
	{
		return Fail(TEXT("Plane half extents must be finite and positive."));
	}
	if (!DomainVertices.IsEmpty())
	{
		if (DomainVertices.Num() < 3)
		{
			return Fail(TEXT("Plane polygon domain must contain at least three vertices."));
		}
		double TwiceArea = 0.0;
		for (int32 Index = 0; Index < DomainVertices.Num(); ++Index)
		{
			const FVector2d& A = DomainVertices[Index];
			const FVector2d& B = DomainVertices[(Index + 1) % DomainVertices.Num()];
			if (!FMath::IsFinite(A.X) || !FMath::IsFinite(A.Y) ||
				FMath::Abs(A.X) > HalfExtents.X + 1.0e-9 ||
				FMath::Abs(A.Y) > HalfExtents.Y + 1.0e-9)
			{
				return Fail(TEXT("Plane polygon vertex is invalid or outside half extents."));
			}
			if ((B - A).SquaredLength() <= UE_DOUBLE_SMALL_NUMBER)
			{
				return Fail(TEXT("Plane polygon has a zero-length edge."));
			}
			TwiceArea += A.X * B.Y - A.Y * B.X;
		}
		if (FMath::Abs(TwiceArea) <= UE_DOUBLE_SMALL_NUMBER)
		{
			return Fail(TEXT("Plane polygon has zero signed area."));
		}
		for (int32 EdgeA = 0; EdgeA < DomainVertices.Num(); ++EdgeA)
		{
			const int32 NextA = (EdgeA + 1) % DomainVertices.Num();
			for (int32 EdgeB = EdgeA + 1; EdgeB < DomainVertices.Num(); ++EdgeB)
			{
				const int32 NextB = (EdgeB + 1) % DomainVertices.Num();
				if (EdgeA == EdgeB || EdgeA == NextB || NextA == EdgeB) continue;
				if (SegmentsIntersect2D(
					DomainVertices[EdgeA], DomainVertices[NextA],
					DomainVertices[EdgeB], DomainVertices[NextB]))
				{
					return Fail(TEXT("Plane polygon domain self-intersects."));
				}
			}
		}
	}
	if ((bRequiresCompactOptIn || bAuthorityEligible) &&
		(SourceId == 0 || PrimitiveId == 0 || !bQueryCollisionEnabled))
	{
		return Fail(TEXT("Authoritative bounded plane provenance is incomplete."));
	}
	constexpr double UnitTolerance = 1.0e-9;
	constexpr double OrthogonalTolerance = 1.0e-9;
	if (!FMath::IsNearlyEqual(Normal.SquaredLength(), 1.0, UnitTolerance) ||
		!FMath::IsNearlyEqual(AxisU.SquaredLength(), 1.0, UnitTolerance) ||
		!FMath::IsNearlyEqual(AxisV.SquaredLength(), 1.0, UnitTolerance))
	{
		return Fail(TEXT("Plane basis vectors must be normalized."));
	}
	if (FMath::Abs(FVector3d::DotProduct(Normal, AxisU)) > OrthogonalTolerance ||
		FMath::Abs(FVector3d::DotProduct(Normal, AxisV)) > OrthogonalTolerance ||
		FMath::Abs(FVector3d::DotProduct(AxisU, AxisV)) > OrthogonalTolerance)
	{
		return Fail(TEXT("Plane basis vectors must be orthogonal."));
	}
	return true;
}

FVector3d FExtrudedQuinticPatch::EvaluateSection(const double T) const
{
	const double ClampedT = FMath::Clamp(T, 0.0, 1.0);
	const double OneMinusT = 1.0 - ClampedT;
	const double Basis[6] = {
		FMath::Pow(OneMinusT, 5.0),
		5.0 * ClampedT * FMath::Pow(OneMinusT, 4.0),
		10.0 * FMath::Square(ClampedT) * FMath::Pow(OneMinusT, 3.0),
		10.0 * FMath::Pow(ClampedT, 3.0) * FMath::Square(OneMinusT),
		5.0 * FMath::Pow(ClampedT, 4.0) * OneMinusT,
		FMath::Pow(ClampedT, 5.0) };
	FVector3d Result = FVector3d::ZeroVector;
	for (int32 Index = 0; Index < 6; ++Index)
	{
		Result += Basis[Index] * SectionControlPoints[Index];
	}
	const double CorrectionBasisA = 35.0 * FMath::Pow(ClampedT, 3.0) *
		FMath::Pow(OneMinusT, 4.0);
	const double CorrectionBasisB = 35.0 * FMath::Pow(ClampedT, 4.0) *
		FMath::Pow(OneMinusT, 3.0);
	Result += CorrectionBasisA * InteriorCorrectionControlPoints[0] +
		CorrectionBasisB * InteriorCorrectionControlPoints[1];
	return Result;
}

FVector3d FExtrudedQuinticPatch::EvaluateSectionDerivative(const double T) const
{
	const double ClampedT = FMath::Clamp(T, 0.0, 1.0);
	const double OneMinusT = 1.0 - ClampedT;
	const double Basis[5] = {
		FMath::Pow(OneMinusT, 4.0),
		4.0 * ClampedT * FMath::Pow(OneMinusT, 3.0),
		6.0 * FMath::Square(ClampedT) * FMath::Square(OneMinusT),
		4.0 * FMath::Pow(ClampedT, 3.0) * OneMinusT,
		FMath::Pow(ClampedT, 4.0) };
	FVector3d Result = FVector3d::ZeroVector;
	for (int32 Index = 0; Index < 5; ++Index)
	{
		Result += 5.0 * Basis[Index] *
			(SectionControlPoints[Index + 1] - SectionControlPoints[Index]);
	}
	const double CorrectionDerivativeA =
		105.0 * FMath::Square(ClampedT) * FMath::Pow(OneMinusT, 4.0) -
		140.0 * FMath::Pow(ClampedT, 3.0) * FMath::Pow(OneMinusT, 3.0);
	const double CorrectionDerivativeB =
		140.0 * FMath::Pow(ClampedT, 3.0) * FMath::Pow(OneMinusT, 3.0) -
		105.0 * FMath::Pow(ClampedT, 4.0) * FMath::Square(OneMinusT);
	Result += CorrectionDerivativeA * InteriorCorrectionControlPoints[0] +
		CorrectionDerivativeB * InteriorCorrectionControlPoints[1];
	return Result;
}

bool FExtrudedQuinticPatch::BuildQueryApproximation(
	const double ChordToleranceCm)
{
	SectionPolyline.Reset();
	SectionParameters.Reset();
	SectionSegmentBounds.Reset();
	SectionSegmentBvhNodes.Reset();
	SectionSegmentBvhIndices.Reset();
	MaximumChordErrorCm = TNumericLimits<double>::Max();
	if (!FMath::IsFinite(ChordToleranceCm) || ChordToleranceCm <= 0.0)
	{
		return false;
	}

	// Elevate the quintic base twice to degree seven, then add the two native
	// degree-seven Bernstein corrections. This produces the exact control
	// polygon of EvaluateSection rather than a sampled surrogate.
	FVector3d DegreeSix[7];
	DegreeSix[0] = SectionControlPoints[0];
	DegreeSix[6] = SectionControlPoints[5];
	for (int32 Index = 1; Index < 6; ++Index)
	{
		const double Alpha = static_cast<double>(Index) / 6.0;
		DegreeSix[Index] = Alpha * SectionControlPoints[Index - 1] +
			(1.0 - Alpha) * SectionControlPoints[Index];
	}
	FVector3d DegreeSeven[8];
	DegreeSeven[0] = DegreeSix[0];
	DegreeSeven[7] = DegreeSix[6];
	for (int32 Index = 1; Index < 7; ++Index)
	{
		const double Alpha = static_cast<double>(Index) / 7.0;
		DegreeSeven[Index] = Alpha * DegreeSix[Index - 1] +
			(1.0 - Alpha) * DegreeSix[Index];
	}
	DegreeSeven[3] += InteriorCorrectionControlPoints[0];
	DegreeSeven[4] += InteriorCorrectionControlPoints[1];

	SectionPolyline.Add(DegreeSeven[0]);
	SectionParameters.Add(0.0);
	double AcceptedErrorCm = 0.0;
	if (!AppendCertifiedBezierChords(DegreeSeven, 0.0, 1.0,
		ChordToleranceCm, 0, SectionPolyline, SectionParameters,
		AcceptedErrorCm))
	{
		SectionPolyline.Reset();
		SectionParameters.Reset();
		return false;
	}
	MaximumChordErrorCm = AcceptedErrorCm;

	// A Bezier curve lies in its control hull. These endpoint-extruded control
	// points therefore give an exact conservative AABB for the ruled patch.
	Bounds = FBox3d(EForceInit::ForceInit);
	for (const FVector3d& ControlPoint : DegreeSeven)
	{
		Bounds += ControlPoint + MinimumExtrusionCoordinate * ExtrusionAxis;
		Bounds += ControlPoint + MaximumExtrusionCoordinate * ExtrusionAxis;
	}
	const int32 SegmentCount = FMath::Max(0, SectionPolyline.Num() - 1);
	SectionSegmentBvhIndices.SetNumUninitialized(SegmentCount);
	SectionSegmentBounds.SetNumUninitialized(SegmentCount);
	static_assert(sizeof(FBox3d) <= 56, "Account for per-chord derived geometry storage");
	for (int32 Segment = 0; Segment < SegmentCount; ++Segment)
	{
		SectionSegmentBvhIndices[Segment] = Segment;
		FBox3d Result(EForceInit::ForceInit);
		Result += SectionPolyline[Segment] +
			MinimumExtrusionCoordinate * ExtrusionAxis;
		Result += SectionPolyline[Segment] +
			MaximumExtrusionCoordinate * ExtrusionAxis;
		Result += SectionPolyline[Segment + 1] +
			MinimumExtrusionCoordinate * ExtrusionAxis;
		Result += SectionPolyline[Segment + 1] +
			MaximumExtrusionCoordinate * ExtrusionAxis;
		SectionSegmentBounds[Segment] = Result;
	}
	const auto SegmentBounds = [this](const int32 Segment) -> const FBox3d&
	{
		return SectionSegmentBounds[Segment];
	};
	TFunction<int32(int32, int32)> BuildNode =
		[this, &SegmentBounds, &BuildNode](const int32 FirstIndex,
			const int32 IndexCount)
	{
		const int32 NodeIndex = SectionSegmentBvhNodes.AddDefaulted();
		FBox3d NodeBounds(EForceInit::ForceInit);
		FBox3d CentroidBounds(EForceInit::ForceInit);
		for (int32 Offset = 0; Offset < IndexCount; ++Offset)
		{
			const FBox3d BoundsForSegment = SegmentBounds(
				SectionSegmentBvhIndices[FirstIndex + Offset]);
			NodeBounds += BoundsForSegment;
			CentroidBounds += BoundsForSegment.GetCenter();
		}
		SectionSegmentBvhNodes[NodeIndex].Bounds = NodeBounds;
		constexpr int32 LeafSegmentCount = 8;
		if (IndexCount <= LeafSegmentCount)
		{
			SectionSegmentBvhNodes[NodeIndex].FirstIndex = FirstIndex;
			SectionSegmentBvhNodes[NodeIndex].IndexCount = IndexCount;
			return NodeIndex;
		}
		const FVector3d Extent = CentroidBounds.GetExtent();
		int32 Axis = 0;
		if (Extent.Y > Extent.X) Axis = 1;
		if (Extent.Z > Extent[Axis]) Axis = 2;
		TArrayView<int32> Range(
			SectionSegmentBvhIndices.GetData() + FirstIndex, IndexCount);
		Algo::Sort(Range, [&SegmentBounds, Axis](const int32 A, const int32 B)
		{
			const double CenterA = SegmentBounds(A).GetCenter()[Axis];
			const double CenterB = SegmentBounds(B).GetCenter()[Axis];
			return CenterA != CenterB ? CenterA < CenterB : A < B;
		});
		const int32 LeftCount = IndexCount / 2;
		SectionSegmentBvhNodes[NodeIndex].LeftChild =
			BuildNode(FirstIndex, LeftCount);
		SectionSegmentBvhNodes[NodeIndex].RightChild =
			BuildNode(FirstIndex + LeftCount, IndexCount - LeftCount);
		return NodeIndex;
	};
	if (SegmentCount > 0)
	{
		BuildNode(0, SegmentCount);
	}
	return SectionPolyline.Num() >= 2 &&
		SectionPolyline.Num() == SectionParameters.Num() && Bounds.IsValid &&
		!SectionSegmentBvhNodes.IsEmpty() &&
		SectionSegmentBvhIndices.Num() == SegmentCount;
}

bool FExtrudedQuinticPatch::IsValid(FString* OutReason) const
{
	auto Fail = [OutReason](const TCHAR* Reason)
	{
		if (OutReason) *OutReason = Reason;
		return false;
	};
	if (SourceId == 0 || SurfaceId == 0 || FeatureId == 0 || PrimitiveId == 0)
	{
		return Fail(TEXT("Extruded patch identifiers must be non-zero."));
	}
	if ((CanonicalGroupId == 0) !=
		(!bCanonicalC2ByConstruction && !bCanonicalSymmetryByConstruction) ||
		(bCanonicalSymmetryByConstruction && !bCanonicalC2ByConstruction))
	{
		return Fail(TEXT("Extruded patch canonical certificate is inconsistent."));
	}
	for (const FVector3d& Point : SectionControlPoints)
	{
		if (!IsFiniteVector(Point))
		{
			return Fail(TEXT("Extruded patch control points must be finite."));
		}
	}
	for (const FVector3d& Correction : InteriorCorrectionControlPoints)
	{
		if (!IsFiniteVector(Correction))
		{
			return Fail(TEXT("Extruded patch corrections must be finite."));
		}
	}
	if (!FMath::IsFinite(BaseRootMeanSquareResidualCm) ||
		!FMath::IsFinite(BaseMaximumResidualCm) ||
		!FMath::IsFinite(CorrectedRootMeanSquareResidualCm) ||
		!FMath::IsFinite(CorrectedMaximumResidualCm) ||
		!FMath::IsFinite(AdditionalResidualAgreementAllowanceCm) ||
		BaseRootMeanSquareResidualCm < 0.0 || BaseMaximumResidualCm < 0.0 ||
		CorrectedRootMeanSquareResidualCm < 0.0 ||
		CorrectedMaximumResidualCm < 0.0 ||
		AdditionalResidualAgreementAllowanceCm < 0.0 ||
		CorrectedRootMeanSquareResidualCm >
			BaseRootMeanSquareResidualCm + 1.0e-9 ||
		CorrectedMaximumResidualCm > BaseMaximumResidualCm + 1.0e-9)
	{
		return Fail(TEXT("Extruded patch residual certificate is invalid."));
	}
	if (!IsFiniteVector(ExtrusionAxis) || !FMath::IsNearlyEqual(
		ExtrusionAxis.SquaredLength(), 1.0, 1.0e-9))
	{
		return Fail(TEXT("Extruded patch axis must be finite and normalized."));
	}
	if (!FMath::IsFinite(MinimumExtrusionCoordinate) ||
		!FMath::IsFinite(MaximumExtrusionCoordinate) ||
		MaximumExtrusionCoordinate <= MinimumExtrusionCoordinate || !Bounds.IsValid)
	{
		return Fail(TEXT("Extruded patch domain must be finite and non-empty."));
	}
	for (int32 Sample = 0; Sample <= 16; ++Sample)
	{
		if (EvaluateSectionDerivative(static_cast<double>(Sample) / 16.0)
			.SquaredLength() <= 1.0e-12)
		{
			return Fail(TEXT("Extruded patch section must be regular."));
		}
	}
	return true;
}

FVector3d FOpenRimCanonicalSurfaceFit::EvaluateCanonicalSurface(
	const double InS, const double InT) const
{
	const double S = FMath::Clamp(InS, 0.0, 1.0);
	const double T = FMath::Clamp(InT, 0.0, 1.0);
	const double TransitionStart = VerticalSegmentParameterWidth;
	const double TransitionEnd = TransitionStart + TransitionParameterWidth;
	int32 SegmentIndex = 0;
	double LocalS = 0.0;
	if (S <= TransitionStart)
	{
		LocalS = TransitionStart > 0.0 ? S / TransitionStart : 0.0;
	}
	else if (S < TransitionEnd)
	{
		SegmentIndex = 1;
		LocalS = TransitionParameterWidth > 0.0
			? (S - TransitionStart) / TransitionParameterWidth : 0.0;
	}
	else
	{
		SegmentIndex = 2;
		LocalS = HorizontalSpanParameterWidth > 0.0
			? (S - TransitionEnd) / HorizontalSpanParameterWidth : 0.0;
	}
	const FVector2d Rim = EvaluateBernstein5Curve(
		CanonicalRimSegmentControlPoints + SegmentIndex * 6, LocalS);
	double TBasis[6];
	Bernstein5Basis(T, TBasis);
	const double RimWeight = TBasis[0] + TBasis[1] + TBasis[2];
	const double StartWeight = (TBasis[1] + 2.0 * TBasis[2]) / 5.0;
	const double EndWeight = TBasis[3] + TBasis[4] + TBasis[5];
	const double LongitudinalWeight =
		-(2.0 * TBasis[3] + TBasis[4]) / 5.0;
	const double TransitionEndDepth = EvaluateCanonicalTransitionField(
		TransitionEndDepthCoefficients, S, TransitionStart,
		TransitionParameterWidth);
	const double TransitionStartY = EvaluateCanonicalTransitionField(
		TransitionStartTangentYCoefficients, S, TransitionStart,
		TransitionParameterWidth);
	const double TransitionStartZ = EvaluateCanonicalTransitionField(
		TransitionStartTangentZCoefficients, S, TransitionStart,
		TransitionParameterWidth);
	const double TransitionEndY = EvaluateCanonicalTransitionField(
		TransitionEndYCoefficients, S, TransitionStart,
		TransitionParameterWidth);
	const double TransitionEndZ = EvaluateCanonicalTransitionField(
		TransitionEndZCoefficients, S, TransitionStart,
		TransitionParameterWidth);
	return FVector3d(
		EndWeight * (EvaluateBernstein5Field(EndDepthCoefficients, S) +
			TransitionEndDepth) + LongitudinalWeight *
			EvaluateBernstein5Field(EndLongitudinalTangentCoefficients, S),
		RimWeight * Rim.X + StartWeight *
			(EvaluateBernstein5Field(StartTangentYCoefficients, S) +
				TransitionStartY) + EndWeight *
			(EvaluateBernstein5Field(EndYCoefficients, S) + TransitionEndY),
		RimWeight * Rim.Y + StartWeight *
			(EvaluateBernstein5Field(StartTangentZCoefficients, S) +
				TransitionStartZ) + EndWeight *
			(EvaluateBernstein5Field(EndZCoefficients, S) + TransitionEndZ));
}

FVector3d FOpenRimCanonicalTubeFit::EvaluateTerminal(const double InS) const
{
	const double S = FMath::Clamp(InS, 0.0, 1.0);
	if (!TerminalSplineSegments.IsEmpty())
	{
		int32 SegmentIndex = TerminalSplineSegments.Num() - 1;
		for (int32 Index = 0; Index < TerminalSplineSegments.Num(); ++Index)
		{
			if (S <= TerminalSplineSegments[Index].MaximumCanonicalRimParameter)
			{
				SegmentIndex = Index;
				break;
			}
		}
		const FOpenRimTubeTerminalSplineSegment& Segment =
			TerminalSplineSegments[SegmentIndex];
		const double Denominator = Segment.MaximumCanonicalRimParameter -
			Segment.MinimumCanonicalRimParameter;
		const double T = Denominator > UE_DOUBLE_SMALL_NUMBER ? FMath::Clamp(
			(S - Segment.MinimumCanonicalRimParameter) / Denominator, 0.0, 1.0) : 0.0;
		const FVector3d A = FMath::Lerp(Segment.ControlPoints[0],
			Segment.ControlPoints[1], T);
		const FVector3d B = FMath::Lerp(Segment.ControlPoints[1],
			Segment.ControlPoints[2], T);
		const FVector3d C = FMath::Lerp(Segment.ControlPoints[2],
			Segment.ControlPoints[3], T);
		return FMath::Lerp(FMath::Lerp(A, B, T), FMath::Lerp(B, C, T), T);
	}
	return FVector3d(
		EvaluateBernstein5Field(TerminalDepthCoefficients, S),
		EvaluateBernstein5Field(TerminalYCoefficients, S),
		EvaluateBernstein5Field(TerminalZCoefficients, S));
}

namespace
{
	// Consumes caller-owned scratch; interpolation order is the original de Casteljau order.
	FVector3d EvaluateBezierControlPolygonInPlace(
		const TArrayView<FVector3d> Work, const double Parameter)
	{
		if (Work.IsEmpty()) return FVector3d::ZeroVector;
		const double T = FMath::Clamp(Parameter, 0.0, 1.0);
		for (int32 Remaining = Work.Num() - 1; Remaining > 0; --Remaining)
		{
			for (int32 Index = 0; Index < Remaining; ++Index)
			{
				Work[Index] = FMath::Lerp(Work[Index], Work[Index + 1], T);
			}
		}
		return Work[0];
	}

	template<bool DerivativeU>
	FVector3d EvaluateBicubicDerivative(const FVector3d* Points, const double U, const double V)
	{
		// Fixed dimensions let the compiler specialize the same de Casteljau
		// reductions. Only scratch storage changes; no polynomial reassociation.
		constexpr int32 UCount = DerivativeU ? 3 : 4;
		constexpr int32 VCount = DerivativeU ? 4 : 3;
		constexpr int32 Next = DerivativeU ? 4 : 1;
		FVector3d AlongU[UCount];
		FVector3d AlongV[VCount];
		for (int32 UI = 0; UI < UCount; ++UI)
		{
			for (int32 VI = 0; VI < VCount; ++VI)
				AlongV[VI] = 3.0 * (Points[UI * 4 + VI + Next] - Points[UI * 4 + VI]);
			AlongU[UI] = EvaluateBezierControlPolygonInPlace(MakeArrayView(AlongV), V);
		}
		return EvaluateBezierControlPolygonInPlace(MakeArrayView(AlongU), U);
	}

	// Recognition/build callers retain the non-destructive interface.
	FVector3d EvaluateBezierControlPolygon(
		const TArrayView<const FVector3d> ControlPoints, const double Parameter)
	{
		if (ControlPoints.IsEmpty()) return FVector3d::ZeroVector;
		TArray<FVector3d, TInlineAllocator<16>> Work;
		Work.Append(ControlPoints.GetData(), ControlPoints.Num());
		return EvaluateBezierControlPolygonInPlace(Work, Parameter);
	}

	double TensorBilinearErrorBoundCm(const FTensorBezierSurface& Surface)
	{
		const int32 VCount = Surface.DegreeV + 1;
		const FVector3d& U0V0 = Surface.ControlPoints[0];
		const FVector3d& U0V1 = Surface.ControlPoints[Surface.DegreeV];
		const FVector3d& U1V0 = Surface.ControlPoints[Surface.DegreeU * VCount];
		const FVector3d& U1V1 = Surface.ControlPoints.Last();
		double MaximumSquaredError = 0.0;
		for (int32 UIndex = 0; UIndex <= Surface.DegreeU; ++UIndex)
		{
			const double U = static_cast<double>(UIndex) / Surface.DegreeU;
			for (int32 VIndex = 0; VIndex <= Surface.DegreeV; ++VIndex)
			{
				const double V = static_cast<double>(VIndex) / Surface.DegreeV;
				const FVector3d Bilinear = FMath::Lerp(
					FMath::Lerp(U0V0, U0V1, V),
					FMath::Lerp(U1V0, U1V1, V), U);
				MaximumSquaredError = FMath::Max(MaximumSquaredError,
					(Surface.ControlPoints[UIndex * VCount + VIndex] - Bilinear)
						.SquaredLength());
			}
		}
		// The difference between the tensor surface and the degree-elevated
		// bilinear corner patch is itself a Bezier surface. Its complete image is
		// inside the convex hull of these difference control points.
		return FMath::Sqrt(MaximumSquaredError);
	}

	void SplitBezierControlPolygonHalf(
		const TArrayView<const FVector3d> ControlPoints,
		TArray<FVector3d, TInlineAllocator<16>>& Left,
		TArray<FVector3d, TInlineAllocator<16>>& Right)
	{
		const int32 Count = ControlPoints.Num();
		Left.SetNumUninitialized(Count);
		Right.SetNumUninitialized(Count);
		TArray<FVector3d, TInlineAllocator<16>> Work;
		Work.Append(ControlPoints.GetData(), Count);
		Left[0] = Work[0];
		Right[Count - 1] = Work[Count - 1];
		for (int32 Level = 1; Level < Count; ++Level)
		{
			for (int32 Index = 0; Index < Count - Level; ++Index)
			{
				Work[Index] = 0.5 * (Work[Index] + Work[Index + 1]);
			}
			Left[Level] = Work[0];
			Right[Count - 1 - Level] = Work[Count - 1 - Level];
		}
	}

	void SplitTensorSurfaceU(const FTensorBezierSurface& Source,
		FTensorBezierSurface& Left, FTensorBezierSurface& Right)
	{
		Left.DegreeU = Right.DegreeU = Source.DegreeU;
		Left.DegreeV = Right.DegreeV = Source.DegreeV;
		const int32 UCount = Source.DegreeU + 1;
		const int32 VCount = Source.DegreeV + 1;
		Left.ControlPoints.SetNumUninitialized(UCount * VCount);
		Right.ControlPoints.SetNumUninitialized(UCount * VCount);
		TArray<FVector3d, TInlineAllocator<16>> Polygon;
		TArray<FVector3d, TInlineAllocator<16>> SplitLeft;
		TArray<FVector3d, TInlineAllocator<16>> SplitRight;
		Polygon.SetNumUninitialized(UCount);
		for (int32 VIndex = 0; VIndex < VCount; ++VIndex)
		{
			for (int32 UIndex = 0; UIndex < UCount; ++UIndex)
			{
				Polygon[UIndex] = Source.ControlPoints[UIndex * VCount + VIndex];
			}
			SplitBezierControlPolygonHalf(Polygon, SplitLeft, SplitRight);
			for (int32 UIndex = 0; UIndex < UCount; ++UIndex)
			{
				Left.ControlPoints[UIndex * VCount + VIndex] = SplitLeft[UIndex];
				Right.ControlPoints[UIndex * VCount + VIndex] = SplitRight[UIndex];
			}
		}
	}

	void SplitTensorSurfaceV(const FTensorBezierSurface& Source,
		FTensorBezierSurface& Lower, FTensorBezierSurface& Upper)
	{
		Lower.DegreeU = Upper.DegreeU = Source.DegreeU;
		Lower.DegreeV = Upper.DegreeV = Source.DegreeV;
		const int32 UCount = Source.DegreeU + 1;
		const int32 VCount = Source.DegreeV + 1;
		Lower.ControlPoints.SetNumUninitialized(UCount * VCount);
		Upper.ControlPoints.SetNumUninitialized(UCount * VCount);
		TArray<FVector3d, TInlineAllocator<16>> SplitLower;
		TArray<FVector3d, TInlineAllocator<16>> SplitUpper;
		for (int32 UIndex = 0; UIndex < UCount; ++UIndex)
		{
			const TArrayView<const FVector3d> Polygon = MakeArrayView(
				Source.ControlPoints.GetData() + UIndex * VCount, VCount);
			SplitBezierControlPolygonHalf(Polygon, SplitLower, SplitUpper);
			for (int32 VIndex = 0; VIndex < VCount; ++VIndex)
			{
				Lower.ControlPoints[UIndex * VCount + VIndex] = SplitLower[VIndex];
				Upper.ControlPoints[UIndex * VCount + VIndex] = SplitUpper[VIndex];
			}
		}
	}

	bool AppendCertifiedTensorCells(const FTensorBezierSurface& Surface,
		const double MinimumU, const double MaximumU,
		const double MinimumV, const double MaximumV,
		const double ToleranceCm, const int32 DepthU, const int32 DepthV,
		TArray<FTensorBezierApproximationCell>& Cells,
		double& MaximumAcceptedErrorCm)
	{
		const double ErrorCm = TensorBilinearErrorBoundCm(Surface);
		if (ErrorCm <= ToleranceCm)
		{
			FTensorBezierApproximationCell& Cell = Cells.AddDefaulted_GetRef();
			Cell.MinimumU = MinimumU;
			Cell.MaximumU = MaximumU;
			Cell.MinimumV = MinimumV;
			Cell.MaximumV = MaximumV;
			const int32 VCount = Surface.DegreeV + 1;
			Cell.Corners[0] = Surface.ControlPoints[0];
			Cell.Corners[1] = Surface.ControlPoints[Surface.DegreeV];
			Cell.Corners[2] = Surface.ControlPoints[Surface.DegreeU * VCount];
			Cell.Corners[3] = Surface.ControlPoints.Last();
			Cell.MaximumErrorCm = ErrorCm;
			MaximumAcceptedErrorCm = FMath::Max(MaximumAcceptedErrorCm, ErrorCm);
			return true;
		}
		constexpr int32 MaximumSubdivisionDepthPerAxis = 12;
		const bool bCanSplitU = DepthU < MaximumSubdivisionDepthPerAxis;
		const bool bCanSplitV = DepthV < MaximumSubdivisionDepthPerAxis;
		if (!bCanSplitU && !bCanSplitV) return false;
		FTensorBezierSurface Left;
		FTensorBezierSurface Right;
		FTensorBezierSurface Lower;
		FTensorBezierSurface Upper;
		double SplitUErrorCm = TNumericLimits<double>::Max();
		double SplitVErrorCm = TNumericLimits<double>::Max();
		if (bCanSplitU)
		{
			SplitTensorSurfaceU(Surface, Left, Right);
			SplitUErrorCm = FMath::Max(TensorBilinearErrorBoundCm(Left),
				TensorBilinearErrorBoundCm(Right));
		}
		if (bCanSplitV)
		{
			SplitTensorSurfaceV(Surface, Lower, Upper);
			SplitVErrorCm = FMath::Max(TensorBilinearErrorBoundCm(Lower),
				TensorBilinearErrorBoundCm(Upper));
		}
		if (bCanSplitU && (!bCanSplitV || SplitUErrorCm <= SplitVErrorCm))
		{
			const double MiddleU = 0.5 * (MinimumU + MaximumU);
			return AppendCertifiedTensorCells(Left,
				MinimumU, MiddleU, MinimumV, MaximumV, ToleranceCm,
				DepthU + 1, DepthV, Cells, MaximumAcceptedErrorCm) &&
				AppendCertifiedTensorCells(Right,
					MiddleU, MaximumU, MinimumV, MaximumV, ToleranceCm,
					DepthU + 1, DepthV, Cells, MaximumAcceptedErrorCm);
		}
		const double MiddleV = 0.5 * (MinimumV + MaximumV);
		return AppendCertifiedTensorCells(Lower,
			MinimumU, MaximumU, MinimumV, MiddleV, ToleranceCm,
			DepthU, DepthV + 1, Cells, MaximumAcceptedErrorCm) &&
			AppendCertifiedTensorCells(Upper,
				MinimumU, MaximumU, MiddleV, MaximumV, ToleranceCm,
				DepthU, DepthV + 1, Cells, MaximumAcceptedErrorCm);
	}
}

FVector3d FTensorBezierSurface::Evaluate(
	const double U, const double V) const
{
	if (DegreeU < 0 || DegreeV < 0 ||
		ControlPoints.Num() != (DegreeU + 1) * (DegreeV + 1))
	{
		return FVector3d::ZeroVector;
	}
	TArray<FVector3d, TInlineAllocator<16>> AlongU;
	AlongU.SetNumUninitialized(DegreeU + 1);
	const int32 VCount = DegreeV + 1;
	TArray<FVector3d, TInlineAllocator<16>> AlongV;
	AlongV.SetNumUninitialized(VCount);
	for (int32 UIndex = 0; UIndex <= DegreeU; ++UIndex)
	{
		FMemory::Memcpy(AlongV.GetData(), ControlPoints.GetData() + UIndex * VCount,
			VCount * sizeof(FVector3d));
		AlongU[UIndex] = EvaluateBezierControlPolygonInPlace(AlongV, V);
	}
	return EvaluateBezierControlPolygonInPlace(AlongU, U);
}

FVector3d FTensorBezierSurface::EvaluateDerivativeU(
	const double U, const double V) const
{
	if (DegreeU <= 0 || DegreeV < 0 ||
		ControlPoints.Num() != (DegreeU + 1) * (DegreeV + 1))
	{
		return FVector3d::ZeroVector;
	}
	TArray<FVector3d, TInlineAllocator<16>> DerivativeControlPoints;
	DerivativeControlPoints.SetNumUninitialized(DegreeU);
	const int32 VCount = DegreeV + 1;
	TArray<FVector3d, TInlineAllocator<16>> AlongV;
	AlongV.SetNumUninitialized(VCount);
	for (int32 UIndex = 0; UIndex < DegreeU; ++UIndex)
	{
		for (int32 VIndex = 0; VIndex < VCount; ++VIndex)
		{
			AlongV[VIndex] = static_cast<double>(DegreeU) *
				(ControlPoints[(UIndex + 1) * VCount + VIndex] -
					ControlPoints[UIndex * VCount + VIndex]);
		}
		DerivativeControlPoints[UIndex] = EvaluateBezierControlPolygonInPlace(AlongV, V);
	}
	return EvaluateBezierControlPolygonInPlace(DerivativeControlPoints, U);
}

FVector3d FTensorBezierSurface::EvaluateDerivativeV(
	const double U, const double V) const
{
	if (DegreeV <= 0 || DegreeU < 0 ||
		ControlPoints.Num() != (DegreeU + 1) * (DegreeV + 1))
	{
		return FVector3d::ZeroVector;
	}
	TArray<FVector3d, TInlineAllocator<16>> AlongU;
	AlongU.SetNumUninitialized(DegreeU + 1);
	const int32 VCount = DegreeV + 1;
	TArray<FVector3d, TInlineAllocator<16>> DerivativeControlPoints;
	DerivativeControlPoints.SetNumUninitialized(DegreeV);
	for (int32 UIndex = 0; UIndex <= DegreeU; ++UIndex)
	{
		for (int32 VIndex = 0; VIndex < DegreeV; ++VIndex)
		{
			DerivativeControlPoints[VIndex] = static_cast<double>(DegreeV) *
				(ControlPoints[UIndex * VCount + VIndex + 1] -
					ControlPoints[UIndex * VCount + VIndex]);
		}
		AlongU[UIndex] = EvaluateBezierControlPolygonInPlace(DerivativeControlPoints, V);
	}
	return EvaluateBezierControlPolygonInPlace(AlongU, U);
}

FVector3d FTensorBezierSurface::EvaluateNormal(
	const double U, const double V) const
{
	// Degree is mutable authored data, not a runtime object's permanent subtype.
	// Keep the generic public evaluator for every other valid or invalid net.
	if (DegreeU == 3 && DegreeV == 3 && ControlPoints.Num() == 16)
	{
		return FVector3d::CrossProduct(
			EvaluateBicubicDerivative<true>(ControlPoints.GetData(), U, V),
			EvaluateBicubicDerivative<false>(ControlPoints.GetData(), U, V)).GetSafeNormal();
	}
	return FVector3d::CrossProduct(
		EvaluateDerivativeU(U, V), EvaluateDerivativeV(U, V)).GetSafeNormal();
}

bool FTensorBezierSurface::BuildBilinearApproximation(
	const double ToleranceCm,
	TArray<FTensorBezierApproximationCell>& OutCells,
	double* OutMaximumErrorCm) const
{
	OutCells.Reset();
	if (OutMaximumErrorCm)
	{
		*OutMaximumErrorCm = TNumericLimits<double>::Max();
	}
	if (!FMath::IsFinite(ToleranceCm) || ToleranceCm <= 0.0 || !IsValid())
	{
		return false;
	}
	double MaximumAcceptedErrorCm = 0.0;
	if (!AppendCertifiedTensorCells(*this, 0.0, 1.0, 0.0, 1.0,
		ToleranceCm, 0, 0, OutCells, MaximumAcceptedErrorCm))
	{
		OutCells.Reset();
		return false;
	}
	if (OutMaximumErrorCm) *OutMaximumErrorCm = MaximumAcceptedErrorCm;
	return !OutCells.IsEmpty();
}

bool FTensorBezierSurface::IsValid(FString* OutReason) const
{
	auto Fail = [OutReason](const TCHAR* Reason)
	{
		if (OutReason) *OutReason = Reason;
		return false;
	};
	constexpr int32 MaximumSupportedDegree = 15;
	if (DegreeU < 1 || DegreeV < 1 || DegreeU > MaximumSupportedDegree ||
		DegreeV > MaximumSupportedDegree)
	{
		return Fail(TEXT("Tensor Bezier degrees must be between one and fifteen."));
	}
	if (ControlPoints.Num() != (DegreeU + 1) * (DegreeV + 1))
	{
		return Fail(TEXT("Tensor Bezier control-net dimensions are inconsistent."));
	}
	for (const FVector3d& Point : ControlPoints)
	{
		if (!IsFiniteVector(Point))
		{
			return Fail(TEXT("Tensor Bezier control points must be finite."));
		}
	}
	for (int32 UIndex = 0; UIndex <= 4; ++UIndex)
	{
		for (int32 VIndex = 0; VIndex <= 4; ++VIndex)
		{
			const double U = static_cast<double>(UIndex) / 4.0;
			const double V = static_cast<double>(VIndex) / 4.0;
			if (FVector3d::CrossProduct(EvaluateDerivativeU(U, V),
				EvaluateDerivativeV(U, V)).SquaredLength() <= 1.0e-18)
			{
				return Fail(TEXT("Tensor Bezier surface must be regular."));
			}
		}
	}
	return true;
}

bool FTensorBezierPatch::BuildQueryApproximation(const double ToleranceCm)
{
	MaximumApproximationErrorCm = TNumericLimits<double>::Max();
	ApproximationCellBvhNodes.Reset();
	ApproximationCellBvhIndices.Reset();
	if (!Surface.BuildBilinearApproximation(ToleranceCm, ApproximationCells,
		&MaximumApproximationErrorCm))
	{
		Bounds = FBox3d(EForceInit::ForceInit);
		bApproximationCertified = false;
		return false;
	}
	Bounds = FBox3d(EForceInit::ForceInit);
	for (const FTensorBezierApproximationCell& Cell : ApproximationCells)
	{
		for (const FVector3d& Corner : Cell.Corners) Bounds += Corner;
	}
	Bounds = Bounds.ExpandBy(MaximumApproximationErrorCm);
	ApproximationCellBvhIndices.SetNumUninitialized(ApproximationCells.Num());
	for (int32 Index = 0; Index < ApproximationCells.Num(); ++Index)
	{
		ApproximationCellBvhIndices[Index] = Index;
	}
	const auto CellBounds = [this](const int32 CellIndex)
	{
		const FTensorBezierApproximationCell& Cell =
			ApproximationCells[CellIndex];
		FBox3d Result(EForceInit::ForceInit);
		for (const FVector3d& Corner : Cell.Corners) Result += Corner;
		return Result.ExpandBy(Cell.MaximumErrorCm);
	};
	TFunction<int32(int32, int32)> BuildNode =
		[this, &CellBounds, &BuildNode](const int32 FirstIndex,
			const int32 IndexCount)
	{
		const int32 NodeIndex = ApproximationCellBvhNodes.AddDefaulted();
		FBox3d NodeBounds(EForceInit::ForceInit);
		FBox3d CentroidBounds(EForceInit::ForceInit);
		for (int32 Offset = 0; Offset < IndexCount; ++Offset)
		{
			const FBox3d BoundsForCell = CellBounds(
				ApproximationCellBvhIndices[FirstIndex + Offset]);
			NodeBounds += BoundsForCell;
			CentroidBounds += BoundsForCell.GetCenter();
		}
		ApproximationCellBvhNodes[NodeIndex].Bounds = NodeBounds;
		constexpr int32 LeafCellCount = 8;
		if (IndexCount <= LeafCellCount)
		{
			ApproximationCellBvhNodes[NodeIndex].FirstIndex = FirstIndex;
			ApproximationCellBvhNodes[NodeIndex].IndexCount = IndexCount;
			return NodeIndex;
		}
		const FVector3d Extent = CentroidBounds.GetExtent();
		int32 Axis = 0;
		if (Extent.Y > Extent.X) Axis = 1;
		if (Extent.Z > Extent[Axis]) Axis = 2;
		TArrayView<int32> Range(
			ApproximationCellBvhIndices.GetData() + FirstIndex, IndexCount);
		Algo::Sort(Range, [&CellBounds, Axis](const int32 A, const int32 B)
		{
			const double CenterA = CellBounds(A).GetCenter()[Axis];
			const double CenterB = CellBounds(B).GetCenter()[Axis];
			return CenterA != CenterB ? CenterA < CenterB : A < B;
		});
		const int32 LeftCount = IndexCount / 2;
		ApproximationCellBvhNodes[NodeIndex].LeftChild =
			BuildNode(FirstIndex, LeftCount);
		ApproximationCellBvhNodes[NodeIndex].RightChild =
			BuildNode(FirstIndex + LeftCount, IndexCount - LeftCount);
		return NodeIndex;
	};
	if (!ApproximationCells.IsEmpty()) BuildNode(0, ApproximationCells.Num());
	bApproximationCertified = true;
	return Bounds.IsValid != 0;
}

bool FTensorBezierPatch::IsValid(FString* OutReason) const
{
	auto Fail = [OutReason](const TCHAR* Reason)
	{
		if (OutReason) *OutReason = Reason;
		return false;
	};
	if (SourceId == 0 || SurfaceId == 0 || FeatureId == 0 ||
		PrimitiveId == 0 || CanonicalGroupId == 0)
	{
		return Fail(TEXT("Tensor patch identifiers must be non-zero."));
	}
	FString SurfaceReason;
	if (!Surface.IsValid(&SurfaceReason))
	{
		if (OutReason) *OutReason = SurfaceReason;
		return false;
	}
	if (bAuthorityEligible &&
		(!bQueryCollisionEnabled || !bApproximationCertified))
	{
		return Fail(TEXT("Tensor authority requires query and approximation certificates."));
	}
	if (bApproximationCertified &&
		(ApproximationCells.IsEmpty() || !Bounds.IsValid ||
			ApproximationCellBvhNodes.IsEmpty() ||
			ApproximationCellBvhIndices.Num() != ApproximationCells.Num() ||
			!FMath::IsFinite(MaximumApproximationErrorCm) ||
			MaximumApproximationErrorCm < 0.0))
	{
		return Fail(TEXT("Tensor patch approximation certificate is incomplete."));
	}
	return true;
}

bool FPiecewiseTensorBezierPatch::BuildQueryApproximation(
	const double ToleranceCm)
{
	Bounds = FBox3d(EForceInit::ForceInit);
	CellBvhNodes.Reset();
	CellBvhIndices.Reset();
	Adjacencies.Reset();
	MaximumApproximationErrorCm = 0.0;
	TArray<uint8> CellBuildResults;
	CellBuildResults.SetNumZeroed(Cells.Num());
	ParallelFor(Cells.Num(), [this, ToleranceCm, &CellBuildResults](
		const int32 CellIndex)
	{
		FPiecewiseTensorBezierCell& Cell = Cells[CellIndex];
		FTensorBezierPatch Temporary;
		Temporary.Surface = Cell.Surface;
		if (!Temporary.BuildQueryApproximation(ToleranceCm))
		{
			return;
		}
		Cell.Bounds = Temporary.Bounds;
		Cell.ApproximationCells = MoveTemp(Temporary.ApproximationCells);
		Cell.ApproximationCellBvhNodes = MoveTemp(
			Temporary.ApproximationCellBvhNodes);
		Cell.ApproximationCellBvhIndices = MoveTemp(
			Temporary.ApproximationCellBvhIndices);
		Cell.MaximumApproximationErrorCm = Temporary.MaximumApproximationErrorCm;
		CellBuildResults[CellIndex] = 1;
	});
	for (int32 CellIndex = 0; CellIndex < Cells.Num(); ++CellIndex)
	{
		if (CellBuildResults[CellIndex] == 0)
		{
			bApproximationCertified = false;
			return false;
		}
		const FPiecewiseTensorBezierCell& Cell = Cells[CellIndex];
		MaximumApproximationErrorCm = FMath::Max(MaximumApproximationErrorCm,
			Cell.MaximumApproximationErrorCm);
		Bounds += Cell.Bounds;
	}
	CellBvhIndices.SetNumUninitialized(Cells.Num());
	for (int32 Index = 0; Index < Cells.Num(); ++Index) CellBvhIndices[Index] = Index;
	TFunction<int32(int32, int32)> BuildNode =
		[this, &BuildNode](const int32 FirstIndex, const int32 IndexCount)
	{
		const int32 NodeIndex = CellBvhNodes.AddDefaulted();
		FBox3d NodeBounds(EForceInit::ForceInit);
		FBox3d CentroidBounds(EForceInit::ForceInit);
		for (int32 Offset = 0; Offset < IndexCount; ++Offset)
		{
			const FBox3d& CellBounds = Cells[CellBvhIndices[FirstIndex + Offset]].Bounds;
			NodeBounds += CellBounds;
			CentroidBounds += CellBounds.GetCenter();
		}
		CellBvhNodes[NodeIndex].Bounds = NodeBounds;
		constexpr int32 LeafCellCount = 8;
		if (IndexCount <= LeafCellCount)
		{
			CellBvhNodes[NodeIndex].FirstIndex = FirstIndex;
			CellBvhNodes[NodeIndex].IndexCount = IndexCount;
			return NodeIndex;
		}
		const FVector3d Extent = CentroidBounds.GetExtent();
		int32 Axis = 0;
		if (Extent.Y > Extent.X) Axis = 1;
		if (Extent.Z > Extent[Axis]) Axis = 2;
		TArrayView<int32> Range(CellBvhIndices.GetData() + FirstIndex, IndexCount);
		Algo::Sort(Range, [this, Axis](const int32 A, const int32 B)
		{
			const double CenterA = Cells[A].Bounds.GetCenter()[Axis];
			const double CenterB = Cells[B].Bounds.GetCenter()[Axis];
			return CenterA != CenterB ? CenterA < CenterB : A < B;
		});
		const int32 LeftCount = IndexCount / 2;
		CellBvhNodes[NodeIndex].LeftChild = BuildNode(FirstIndex, LeftCount);
		CellBvhNodes[NodeIndex].RightChild = BuildNode(FirstIndex + LeftCount, IndexCount - LeftCount);
		return NodeIndex;
	};
	if (!Cells.IsEmpty()) BuildNode(0, Cells.Num());
	const auto IsCertifiedC2Join = [](const FPiecewiseTensorBezierCell& First,
		const uint8 FirstBoundary, const FPiecewiseTensorBezierCell& Second,
		const uint8 SecondBoundary)
	{
		const bool bAlongU = FirstBoundary < 2;
		if (bAlongU != (SecondBoundary < 2)) return false;
		const bool bFirstAtMaximum = FirstBoundary == (bAlongU ? 1 : 3);
		const bool bSecondAtMinimum = SecondBoundary == (bAlongU ? 0 : 2);
		const FPiecewiseTensorBezierCell& Before =
			bFirstAtMaximum && bSecondAtMinimum ? First : Second;
		const FPiecewiseTensorBezierCell& After =
			bFirstAtMaximum && bSecondAtMinimum ? Second : First;
		const uint8 BeforeBoundary =
			bFirstAtMaximum && bSecondAtMinimum ? FirstBoundary : SecondBoundary;
		const uint8 AfterBoundary =
			bFirstAtMaximum && bSecondAtMinimum ? SecondBoundary : FirstBoundary;
		if (BeforeBoundary != (bAlongU ? 1 : 3) ||
			AfterBoundary != (bAlongU ? 0 : 2)) return false;
		const int32 BeforeDegree = bAlongU ? Before.Surface.DegreeU :
			Before.Surface.DegreeV;
		const int32 AfterDegree = bAlongU ? After.Surface.DegreeU :
			After.Surface.DegreeV;
		const int32 BeforeOrthogonalDegree = bAlongU ? Before.Surface.DegreeV :
			Before.Surface.DegreeU;
		const int32 AfterOrthogonalDegree = bAlongU ? After.Surface.DegreeV :
			After.Surface.DegreeU;
		if (BeforeDegree < 2 || AfterDegree < 2 ||
			BeforeOrthogonalDegree != AfterOrthogonalDegree) return false;
		double BeforeSpan = bAlongU ? Before.MaximumU - Before.MinimumU :
			Before.MaximumV - Before.MinimumV;
		double AfterSpan = bAlongU ? After.MaximumU - After.MinimumU :
			After.MaximumV - After.MinimumV;
		if (!bAlongU && Before.bTerminalClosure != After.bTerminalClosure)
		{
			if (Before.bTerminalClosure)
				BeforeSpan *= Before.LongitudinalParameterScale;
			if (After.bTerminalClosure)
				AfterSpan *= After.LongitudinalParameterScale;
		}
		if (!FMath::IsFinite(BeforeSpan) || !FMath::IsFinite(AfterSpan) ||
			BeforeSpan <= 0.0 || AfterSpan <= 0.0) return false;
		const auto Control = [bAlongU](const FPiecewiseTensorBezierCell& Cell,
			const int32 Along, const int32 Orthogonal)
		{
			const int32 U = bAlongU ? Along : Orthogonal;
			const int32 V = bAlongU ? Orthogonal : Along;
			return Cell.Surface.ControlPoints[
				U * (Cell.Surface.DegreeV + 1) + V];
		};
		constexpr double PositionToleranceCm = 1.0e-6;
		constexpr double FirstDerivativeToleranceCm = 1.0e-5;
		// Independently converted quintic/cubic cells can amplify double-precision
		// collocation noise when the normalized interval is very short. This is a
		// derivative-space tolerance (not a positional collision shell): position
		// and first-derivative continuity remain gated independently above.
		constexpr double SecondDerivativeToleranceCm = 2.0e-2;
		for (int32 Orthogonal = 0;
			Orthogonal <= BeforeOrthogonalDegree; ++Orthogonal)
		{
			const FVector3d BeforeEnd = Control(Before, BeforeDegree, Orthogonal);
			const FVector3d BeforePrevious = Control(
				Before, BeforeDegree - 1, Orthogonal);
			const FVector3d BeforePrevious2 = Control(
				Before, BeforeDegree - 2, Orthogonal);
			const FVector3d AfterStart = Control(After, 0, Orthogonal);
			const FVector3d AfterNext = Control(After, 1, Orthogonal);
			const FVector3d AfterNext2 = Control(After, 2, Orthogonal);
			if (FVector3d::Distance(BeforeEnd, AfterStart) >
				PositionToleranceCm) return false;
			if (FVector3d::Distance(
				BeforeDegree * (BeforeEnd - BeforePrevious) / BeforeSpan,
				AfterDegree * (AfterNext - AfterStart) / AfterSpan) >
				FirstDerivativeToleranceCm) return false;
			if (FVector3d::Distance(BeforeDegree * (BeforeDegree - 1.0) *
				(BeforeEnd - 2.0 * BeforePrevious + BeforePrevious2) /
				FMath::Square(BeforeSpan), AfterDegree * (AfterDegree - 1.0) *
				(AfterNext2 - 2.0 * AfterNext + AfterStart) /
				FMath::Square(AfterSpan)) > SecondDerivativeToleranceCm) return false;
		}
		return true;
	};
	const auto AddAdjacency = [this, &IsCertifiedC2Join](const int32 CellIndex,
		const uint8 BoundaryIndex, const int32 AdjacentCellIndex,
		const uint8 AdjacentBoundaryIndex)
	{
		const FPiecewiseTensorBezierCell& Cell = Cells[CellIndex];
		const FPiecewiseTensorBezierCell& Adjacent = Cells[AdjacentCellIndex];
		if (!IsCertifiedC2Join(
			Cell, BoundaryIndex, Adjacent, AdjacentBoundaryIndex)) return;
		FPiecewiseTensorBezierAdjacency& Link = Adjacencies.AddDefaulted_GetRef();
		Link.BoundaryFeatureId = Cell.BoundaryFeatureIds[BoundaryIndex];
		Link.AdjacentBoundaryFeatureId =
			Adjacent.BoundaryFeatureIds[AdjacentBoundaryIndex];
		Link.CellPrimitiveId = Cell.PrimitiveId;
		Link.AdjacentCellPrimitiveId = Adjacent.PrimitiveId;
		Link.BoundaryIndex = BoundaryIndex;
		Link.AdjacentBoundaryIndex = AdjacentBoundaryIndex;
		Link.bC2ByConstruction = true;
	};
	for (FPiecewiseTensorBezierCell& Cell : Cells)
	{
		for (uint8 BoundaryIndex = 0; BoundaryIndex < 4; ++BoundaryIndex)
		{
			Cell.BoundaryFeatureIds[BoundaryIndex] = CombineStableIds(
				Cell.FeatureId, static_cast<uint64>(BoundaryIndex + 1));
		}
	}
	constexpr double ParameterTolerance = 1.0e-10;
	const auto QuantizeParameter = [ParameterTolerance](const double Value)
	{
		return FMath::RoundToInt64(Value / ParameterTolerance);
	};
	const auto BoundaryKey = [](const int64 Boundary, const int64 RangeMinimum,
		const int64 RangeMaximum, const uint64 Salt)
	{
		uint64 Key = CombineStableIds(Salt, static_cast<uint64>(Boundary));
		Key = CombineStableIds(Key, static_cast<uint64>(RangeMinimum));
		return CombineStableIds(Key, static_cast<uint64>(RangeMaximum));
	};
	TMap<uint64, TArray<int32>> MinimumUBoundaries;
	TMap<uint64, TArray<int32>> MinimumVBoundaries;
	TMap<uint64, TArray<int32>> ClosureMinimumVBoundaries;
	constexpr uint64 UBoundarySalt = 0x9E3779B97F4A7C15ull;
	constexpr uint64 VBoundarySalt = 0xC2B2AE3D27D4EB4Full;
	constexpr uint64 ClosureBoundarySalt = 0x165667B19E3779F9ull;
	for (int32 CellIndex = 0; CellIndex < Cells.Num(); ++CellIndex)
	{
		const FPiecewiseTensorBezierCell& Cell = Cells[CellIndex];
		MinimumUBoundaries.FindOrAdd(BoundaryKey(
			QuantizeParameter(Cell.MinimumU), QuantizeParameter(Cell.MinimumV),
			QuantizeParameter(Cell.MaximumV), UBoundarySalt)).Add(CellIndex);
		MinimumVBoundaries.FindOrAdd(BoundaryKey(
			QuantizeParameter(Cell.MinimumV), QuantizeParameter(Cell.MinimumU),
			QuantizeParameter(Cell.MaximumU), VBoundarySalt)).Add(CellIndex);
		if (Cell.bTerminalClosure && FMath::IsNearlyEqual(
			Cell.MinimumV, 0.0, ParameterTolerance))
		{
			ClosureMinimumVBoundaries.FindOrAdd(BoundaryKey(0,
				QuantizeParameter(Cell.MinimumU), QuantizeParameter(Cell.MaximumU),
				ClosureBoundarySalt)).Add(CellIndex);
		}
	}
	const auto GatherCandidates = [&BoundaryKey](
		const TMap<uint64, TArray<int32>>& Buckets, const int64 Boundary,
		const int64 RangeMinimum, const int64 RangeMaximum, const uint64 Salt,
		TSet<int32, DefaultKeyFuncs<int32>, TInlineSetAllocator<16>>& OutCandidates)
	{
		for (int64 DBoundary = -1; DBoundary <= 1; ++DBoundary)
		{
			for (int64 DMinimum = -1; DMinimum <= 1; ++DMinimum)
			{
				for (int64 DMaximum = -1; DMaximum <= 1; ++DMaximum)
				{
					if (const TArray<int32>* Bucket = Buckets.Find(BoundaryKey(
						Boundary + DBoundary, RangeMinimum + DMinimum,
						RangeMaximum + DMaximum, Salt)))
					{
						for (const int32 CellIndex : *Bucket)
						{
							OutCandidates.Add(CellIndex);
						}
					}
				}
			}
		}
	};
	for (int32 AIndex = 0; AIndex < Cells.Num(); ++AIndex)
	{
		const FPiecewiseTensorBezierCell& A = Cells[AIndex];
		TSet<int32, DefaultKeyFuncs<int32>, TInlineSetAllocator<16>> Candidates;
		GatherCandidates(MinimumUBoundaries, QuantizeParameter(A.MaximumU),
			QuantizeParameter(A.MinimumV), QuantizeParameter(A.MaximumV),
			UBoundarySalt, Candidates);
		for (const int32 BIndex : Candidates)
		{
			if (BIndex == AIndex) continue;
			const FPiecewiseTensorBezierCell& B = Cells[BIndex];
			if (FMath::IsNearlyEqual(A.MaximumU, B.MinimumU, ParameterTolerance) &&
				FMath::IsNearlyEqual(A.MinimumV, B.MinimumV, ParameterTolerance) &&
				FMath::IsNearlyEqual(A.MaximumV, B.MaximumV, ParameterTolerance))
			{
				AddAdjacency(AIndex, 1, BIndex, 0);
				AddAdjacency(BIndex, 0, AIndex, 1);
			}
		}
		Candidates.Reset();
		GatherCandidates(MinimumVBoundaries, QuantizeParameter(A.MaximumV),
			QuantizeParameter(A.MinimumU), QuantizeParameter(A.MaximumU),
			VBoundarySalt, Candidates);
		for (const int32 BIndex : Candidates)
		{
			if (BIndex == AIndex) continue;
			const FPiecewiseTensorBezierCell& B = Cells[BIndex];
			if (FMath::IsNearlyEqual(A.MaximumV, B.MinimumV, ParameterTolerance) &&
				FMath::IsNearlyEqual(A.MinimumU, B.MinimumU, ParameterTolerance) &&
				FMath::IsNearlyEqual(A.MaximumU, B.MaximumU, ParameterTolerance))
			{
				AddAdjacency(AIndex, 3, BIndex, 2);
				AddAdjacency(BIndex, 2, AIndex, 3);
			}
		}
		if (!A.bTerminalClosure && FMath::IsNearlyEqual(
			A.MaximumV, 1.0, ParameterTolerance))
		{
			Candidates.Reset();
			GatherCandidates(ClosureMinimumVBoundaries, 0,
				QuantizeParameter(A.MinimumU), QuantizeParameter(A.MaximumU),
				ClosureBoundarySalt, Candidates);
			for (const int32 BIndex : Candidates)
			{
				const FPiecewiseTensorBezierCell& B = Cells[BIndex];
				if (!B.bTerminalClosure ||
					!FMath::IsNearlyEqual(B.MinimumV, 0.0, ParameterTolerance) ||
					!FMath::IsNearlyEqual(A.MinimumU, B.MinimumU, ParameterTolerance) ||
					!FMath::IsNearlyEqual(A.MaximumU, B.MaximumU, ParameterTolerance))
				{
					continue;
				}
				AddAdjacency(AIndex, 3, BIndex, 2);
				AddAdjacency(BIndex, 2, AIndex, 3);
			}
		}
	}
	Algo::Sort(Adjacencies, [](const FPiecewiseTensorBezierAdjacency& A,
		const FPiecewiseTensorBezierAdjacency& B)
	{
		if (A.CellPrimitiveId != B.CellPrimitiveId)
			return A.CellPrimitiveId < B.CellPrimitiveId;
		if (A.BoundaryIndex != B.BoundaryIndex)
			return A.BoundaryIndex < B.BoundaryIndex;
		return A.AdjacentCellPrimitiveId < B.AdjacentCellPrimitiveId;
	});
	BuildTensorContactTopology(*this);
	bApproximationCertified = !Cells.IsEmpty() && Bounds.IsValid != 0;
	return bApproximationCertified;
}

bool FPiecewiseTensorBezierPatch::IsValid(FString* OutReason) const
{
	auto Fail = [OutReason](const TCHAR* Reason)
	{
		if (OutReason) *OutReason = Reason;
		return false;
	};
	if (SourceId == 0 || SurfaceId == 0 || PrimitiveId == 0 ||
		CanonicalGroupId == 0)
	{
		return Fail(TEXT("Piecewise tensor patch identifiers must be non-zero."));
	}
	if (!bApproximationCertified || Cells.IsEmpty() || !Bounds.IsValid ||
		CellBvhNodes.IsEmpty() || CellBvhIndices.Num() != Cells.Num() ||
		!FMath::IsFinite(MaximumApproximationErrorCm))
	{
		return Fail(TEXT("Piecewise tensor patch approximation certificate is incomplete."));
	}
	uint64 PreviousPrimitive = 0;
	for (const FPiecewiseTensorBezierCell& Cell : Cells)
	{
		if (Cell.FeatureId == 0 || Cell.PrimitiveId == 0 ||
			!Cell.Bounds.IsValid || Cell.ApproximationCells.IsEmpty() ||
			Cell.ApproximationCellBvhNodes.IsEmpty() ||
			Cell.ApproximationCellBvhIndices.Num() != Cell.ApproximationCells.Num() ||
			!FMath::IsFinite(Cell.MaximumApproximationErrorCm) ||
			!Cell.Surface.IsValid())
		{
			return Fail(TEXT("Piecewise tensor patch cell is incomplete."));
		}
		if (PreviousPrimitive != 0 && Cell.PrimitiveId <= PreviousPrimitive)
		{
			return Fail(TEXT("Piecewise tensor patch cells are not uniquely sorted."));
		}
		PreviousPrimitive = Cell.PrimitiveId;
	}
	uint64 PreviousAdjacencyCell = 0;
	uint8 PreviousAdjacencyBoundary = 0;
	uint64 PreviousAdjacentCell = 0;
	for (const FPiecewiseTensorBezierAdjacency& Link : Adjacencies)
	{
		if (Link.BoundaryFeatureId == 0 || Link.AdjacentBoundaryFeatureId == 0 ||
			Link.CellPrimitiveId == 0 || Link.AdjacentCellPrimitiveId == 0 ||
			Link.CellPrimitiveId == Link.AdjacentCellPrimitiveId ||
			Link.BoundaryIndex >= 4 || Link.AdjacentBoundaryIndex >= 4 ||
			!Link.bC2ByConstruction)
		{
			return Fail(TEXT("Piecewise tensor patch adjacency is incomplete."));
		}
		if (PreviousAdjacencyCell == Link.CellPrimitiveId &&
			PreviousAdjacencyBoundary == Link.BoundaryIndex &&
			PreviousAdjacentCell == Link.AdjacentCellPrimitiveId)
		{
			return Fail(TEXT("Piecewise tensor patch has duplicate adjacency."));
		}
		PreviousAdjacencyCell = Link.CellPrimitiveId;
		PreviousAdjacencyBoundary = Link.BoundaryIndex;
		PreviousAdjacentCell = Link.AdjacentCellPrimitiveId;
	}
	if (bAuthorityEligible && (!bQueryCollisionEnabled ||
		!bSourceResidualCertified))
	{
		return Fail(TEXT("Piecewise tensor authority requires source and query certificates."));
	}
	return true;
}

bool FAnalyticWorldData::FinalizeAndValidate(
	FString* OutReason, const double TensorQueryApproximationToleranceCm)
{
	if (!FMath::IsFinite(TensorQueryApproximationToleranceCm) ||
		TensorQueryApproximationToleranceCm <= 0.0)
	{
		if (OutReason)
		{
			*OutReason = TEXT(
				"Tensor query approximation tolerance must be finite and positive.");
		}
		return false;
	}
	const double FinalizeStartSeconds = FPlatformTime::Seconds();
	if (SchemaVersion != AnalyticWorldSchemaVersion)
	{
		if (OutReason)
		{
			*OutReason = FString::Printf(
				TEXT("Unsupported analytic-world schema %u (expected %u)."),
				SchemaVersion, AnalyticWorldSchemaVersion);
		}
		return false;
	}

	Algo::Sort(Planes, [](const FBoundedPlane& A, const FBoundedPlane& B)
	{
		if (A.SurfaceId != B.SurfaceId) return A.SurfaceId < B.SurfaceId;
		if (A.FeatureId != B.FeatureId) return A.FeatureId < B.FeatureId;
		return A.PrimitiveId < B.PrimitiveId;
	});
	Algo::Sort(Triangles, [](const FTriangleSurface& A, const FTriangleSurface& B)
	{
		if (A.SurfaceId != B.SurfaceId) return A.SurfaceId < B.SurfaceId;
		if (A.FeatureId != B.FeatureId) return A.FeatureId < B.FeatureId;
		return A.PrimitiveId < B.PrimitiveId;
	});
	Algo::Sort(ExtrudedQuinticPatches,
		[](const FExtrudedQuinticPatch& A, const FExtrudedQuinticPatch& B)
		{
			return A.PrimitiveId < B.PrimitiveId;
		});
	Algo::Sort(TensorBezierPatches,
		[](const FTensorBezierPatch& A, const FTensorBezierPatch& B)
		{
			return A.PrimitiveId < B.PrimitiveId;
		});
	Algo::Sort(PiecewiseTensorBezierPatches,
		[](const FPiecewiseTensorBezierPatch& A,
			const FPiecewiseTensorBezierPatch& B)
		{
			return A.PrimitiveId < B.PrimitiveId;
		});

	uint64 PreviousSurface = 0;
	uint64 PreviousFeature = 0;
	uint64 PreviousPlanePrimitive = 0;
	for (int32 Index = 0; Index < Planes.Num(); ++Index)
	{
		FString PlaneReason;
		if (!Planes[Index].IsValid(&PlaneReason))
		{
			if (OutReason)
			{
				*OutReason = FString::Printf(
					TEXT("Invalid bounded plane %d: %s"), Index, *PlaneReason);
			}
			return false;
		}
		if (Index > 0 && Planes[Index].SurfaceId == PreviousSurface &&
			Planes[Index].FeatureId == PreviousFeature &&
			Planes[Index].PrimitiveId == PreviousPlanePrimitive)
		{
			if (OutReason)
			{
				*OutReason = TEXT("Duplicate analytic plane surface/feature/primitive identifier.");
			}
			return false;
		}
		FBoundedPlane& Plane = Planes[Index];
		Plane.Bounds = FBox3d(EForceInit::ForceInit);
		if (!Plane.DomainVertices.IsEmpty())
		{
			for (const FVector2d& Vertex : Plane.DomainVertices)
			{
				Plane.Bounds += Plane.Origin +
					Vertex.X * Plane.AxisU + Vertex.Y * Plane.AxisV;
			}
		}
		else for (const double SignU : { -1.0, 1.0 })
		{
			for (const double SignV : { -1.0, 1.0 })
			{
				Plane.Bounds += Plane.Origin +
					SignU * Plane.HalfExtents.X * Plane.AxisU +
					SignV * Plane.HalfExtents.Y * Plane.AxisV;
			}
		}
		PreviousSurface = Plane.SurfaceId;
		PreviousFeature = Plane.FeatureId;
		PreviousPlanePrimitive = Plane.PrimitiveId;
	}
	CompactBounds = FBox3d(EForceInit::ForceInit);
	for (const FBoundedPlane& Plane : Planes)
	{
		if (!Plane.bRequiresCompactOptIn) continue;
		CompactBounds += Plane.Bounds;
	}
	uint64 PreviousPrimitive = 0;
	for (int32 Index = 0; Index < ExtrudedQuinticPatches.Num(); ++Index)
	{
		FString PatchReason;
		if (!ExtrudedQuinticPatches[Index].IsValid(&PatchReason))
		{
			if (OutReason)
			{
				*OutReason = FString::Printf(
					TEXT("Invalid extruded quintic patch %d: %s"), Index, *PatchReason);
			}
			return false;
		}
		if (Index > 0 && ExtrudedQuinticPatches[Index].PrimitiveId ==
			PreviousPrimitive)
		{
			if (OutReason) *OutReason = TEXT("Duplicate compact patch primitive identifier.");
			return false;
		}
		FExtrudedQuinticPatch& Patch = ExtrudedQuinticPatches[Index];
		if (!Patch.BuildQueryApproximation())
		{
			if (OutReason) *OutReason = TEXT(
				"Could not build a certified extruded-quintic query approximation.");
			return false;
		}
		if (Patch.bAuthorityEligible && !Patch.bCanonicalC2ByConstruction)
		{
			if (OutReason) *OutReason = TEXT(
				"Extruded-quintic authority requires a canonical C2 certificate.");
			return false;
		}
		PreviousPrimitive = Patch.PrimitiveId;
		CompactBounds += Patch.Bounds;
	}
	PreviousPrimitive = 0;
	const double TensorStartSeconds = FPlatformTime::Seconds();
	for (int32 Index = 0; Index < TensorBezierPatches.Num(); ++Index)
	{
		FTensorBezierPatch& Patch = TensorBezierPatches[Index];
		if (!Patch.BuildQueryApproximation(TensorQueryApproximationToleranceCm))
		{
			if (OutReason)
			{
				*OutReason = TEXT("Could not build a certified tensor-patch query approximation.");
			}
			return false;
		}
		FString PatchReason;
		if (!Patch.IsValid(&PatchReason))
		{
			if (OutReason)
			{
				*OutReason = FString::Printf(
					TEXT("Invalid tensor Bezier patch %d: %s"), Index, *PatchReason);
			}
			return false;
		}
		if (Index > 0 && Patch.PrimitiveId == PreviousPrimitive)
		{
			if (OutReason)
			{
				*OutReason = TEXT("Duplicate tensor Bezier patch primitive identifier.");
			}
			return false;
		}
		PreviousPrimitive = Patch.PrimitiveId;
		CompactBounds += Patch.Bounds;
	}
	const double TensorSeconds = FPlatformTime::Seconds() - TensorStartSeconds;
	PreviousPrimitive = 0;
	const double PiecewiseStartSeconds = FPlatformTime::Seconds();
	for (int32 Index = 0; Index < PiecewiseTensorBezierPatches.Num(); ++Index)
	{
		FPiecewiseTensorBezierPatch& Patch = PiecewiseTensorBezierPatches[Index];
		if (!Patch.BuildQueryApproximation(TensorQueryApproximationToleranceCm))
		{
			if (OutReason) *OutReason = TEXT(
				"Could not build a certified piecewise tensor query approximation.");
			return false;
		}
		FString PatchReason;
		if (!Patch.IsValid(&PatchReason))
		{
			if (OutReason) *OutReason = FString::Printf(
				TEXT("Invalid piecewise tensor patch %d: %s"), Index, *PatchReason);
			return false;
		}
		if (Index > 0 && Patch.PrimitiveId == PreviousPrimitive)
		{
			if (OutReason) *OutReason = TEXT(
				"Duplicate piecewise tensor patch primitive identifier.");
			return false;
		}
		PreviousPrimitive = Patch.PrimitiveId;
		CompactBounds += Patch.Bounds;
	}
	const double PiecewiseSeconds =
		FPlatformTime::Seconds() - PiecewiseStartSeconds;
	CompactPrimitiveIndices.Reset();
	for (int32 PlaneIndex = 0; PlaneIndex < Planes.Num(); ++PlaneIndex)
	{
		if (Planes[PlaneIndex].bRequiresCompactOptIn)
		{
			CompactPrimitiveIndices.Add(PlaneIndex);
		}
	}
	for (int32 PatchIndex = 0;
		PatchIndex < ExtrudedQuinticPatches.Num(); ++PatchIndex)
	{
		CompactPrimitiveIndices.Add(Planes.Num() + PatchIndex);
	}
	CompactBvh.Reset();
	if (!CompactPrimitiveIndices.IsEmpty())
	{
		CompactBvh.Reserve(2 * CompactPrimitiveIndices.Num());
		BuildCompactBvhNode(0, CompactPrimitiveIndices.Num());
	}
	PreviousPrimitive = 0;
	for (int32 Index = 0; Index < Triangles.Num(); ++Index)
	{
		FString TriangleReason;
		if (!Triangles[Index].IsValid(&TriangleReason))
		{
			if (OutReason)
			{
				*OutReason = FString::Printf(
					TEXT("Invalid triangle %d: %s"), Index, *TriangleReason);
			}
			return false;
		}
		if (Index > 0 && Triangles[Index].PrimitiveId == PreviousPrimitive)
		{
			if (OutReason)
			{
				*OutReason = TEXT("Duplicate analytical triangle primitive identifier.");
			}
			return false;
		}
		PreviousPrimitive = Triangles[Index].PrimitiveId;
	}
	const double TriangleStartSeconds = FPlatformTime::Seconds();
	if (!BuildTriangleTopology(OutReason))
	{
		return false;
	}
	BuildSmoothSurfaceRegions();
	BuildSurfacePatches();

	TriangleIndices.SetNumUninitialized(Triangles.Num());
	for (int32 Index = 0; Index < TriangleIndices.Num(); ++Index)
	{
		TriangleIndices[Index] = Index;
	}
	TriangleBvh.Reset();
	if (!Triangles.IsEmpty())
	{
		TriangleBvh.Reserve(Triangles.Num() * 2);
		BuildTriangleBvhNode(0, Triangles.Num());
	}
	if (TensorBezierPatches.Num() + PiecewiseTensorBezierPatches.Num() > 0 ||
		Triangles.Num() > 1000)
	{
		UE_LOG(LogTemp, Display,
			TEXT("[AnalyticWorldFinalizeTiming] TotalSeconds=%.6f TensorSeconds=%.6f PiecewiseSeconds=%.6f TriangleSeconds=%.6f Tensors=%d Piecewise=%d Triangles=%d"),
			FPlatformTime::Seconds() - FinalizeStartSeconds,
			TensorSeconds, PiecewiseSeconds,
			FPlatformTime::Seconds() - TriangleStartSeconds,
			TensorBezierPatches.Num(), PiecewiseTensorBezierPatches.Num(),
			Triangles.Num());
	}
	return true;
}

bool FAnalyticWorldData::AppendFinalizedNonCompactPlanes(
	TArray<FBoundedPlane> AdditionalPlanes, FString* OutReason)
{
	const double StartSeconds = FPlatformTime::Seconds();
	auto Fail = [OutReason](const FString& Reason)
	{
		if (OutReason)
		{
			*OutReason = Reason;
		}
		return false;
	};
	for (int32 Index = 0; Index < AdditionalPlanes.Num(); ++Index)
	{
		FBoundedPlane& Plane = AdditionalPlanes[Index];
		if (Plane.bRequiresCompactOptIn)
		{
			return Fail(TEXT(
				"Incremental plane extension only accepts non-compact providers."));
		}
		FString PlaneReason;
		if (!Plane.IsValid(&PlaneReason))
		{
			return Fail(FString::Printf(
				TEXT("Invalid incremental bounded plane %d: %s"),
				Index, *PlaneReason));
		}
		Plane.Bounds = FBox3d(EForceInit::ForceInit);
		if (!Plane.DomainVertices.IsEmpty())
		{
			for (const FVector2d& Vertex : Plane.DomainVertices)
			{
				Plane.Bounds += Plane.Origin +
					Vertex.X * Plane.AxisU + Vertex.Y * Plane.AxisV;
			}
		}
		else
		{
			for (const double SignU : { -1.0, 1.0 })
			{
				for (const double SignV : { -1.0, 1.0 })
				{
					Plane.Bounds += Plane.Origin +
						SignU * Plane.HalfExtents.X * Plane.AxisU +
						SignV * Plane.HalfExtents.Y * Plane.AxisV;
				}
			}
		}
	}
	TArray<FBoundedPlane> CandidatePlanes = Planes;
	CandidatePlanes.Append(MoveTemp(AdditionalPlanes));
	Algo::Sort(CandidatePlanes, [](const FBoundedPlane& A, const FBoundedPlane& B)
	{
		if (A.SurfaceId != B.SurfaceId) return A.SurfaceId < B.SurfaceId;
		if (A.FeatureId != B.FeatureId) return A.FeatureId < B.FeatureId;
		return A.PrimitiveId < B.PrimitiveId;
	});
	for (int32 Index = 1; Index < CandidatePlanes.Num(); ++Index)
	{
		const FBoundedPlane& Previous = CandidatePlanes[Index - 1];
		const FBoundedPlane& Plane = CandidatePlanes[Index];
		if (Plane.SurfaceId == Previous.SurfaceId &&
			Plane.FeatureId == Previous.FeatureId &&
			Plane.PrimitiveId == Previous.PrimitiveId)
		{
			return Fail(TEXT(
				"Incremental plane extension introduced a duplicate identifier."));
		}
	}
	Planes = MoveTemp(CandidatePlanes);
	CompactBounds = FBox3d(EForceInit::ForceInit);
	for (const FBoundedPlane& Plane : Planes)
	{
		if (Plane.bRequiresCompactOptIn)
		{
			CompactBounds += Plane.Bounds;
		}
	}
	for (const FExtrudedQuinticPatch& Patch : ExtrudedQuinticPatches)
	{
		CompactBounds += Patch.Bounds;
	}
	for (const FTensorBezierPatch& Patch : TensorBezierPatches)
	{
		CompactBounds += Patch.Bounds;
	}
	for (const FPiecewiseTensorBezierPatch& Patch :
		PiecewiseTensorBezierPatches)
	{
		CompactBounds += Patch.Bounds;
	}
	CompactPrimitiveIndices.Reset();
	for (int32 PlaneIndex = 0; PlaneIndex < Planes.Num(); ++PlaneIndex)
	{
		if (Planes[PlaneIndex].bRequiresCompactOptIn)
		{
			CompactPrimitiveIndices.Add(PlaneIndex);
		}
	}
	for (int32 PatchIndex = 0;
		PatchIndex < ExtrudedQuinticPatches.Num(); ++PatchIndex)
	{
		CompactPrimitiveIndices.Add(Planes.Num() + PatchIndex);
	}
	CompactBvh.Reset();
	if (!CompactPrimitiveIndices.IsEmpty())
	{
		CompactBvh.Reserve(2 * CompactPrimitiveIndices.Num());
		BuildCompactBvhNode(0, CompactPrimitiveIndices.Num());
	}
	UE_LOG(LogTemp, Display,
		TEXT("[AnalyticWorldIncrementalPlaneTiming] Seconds=%.6f Planes=%d"),
		FPlatformTime::Seconds() - StartSeconds, Planes.Num());
	return true;
}

void FAnalyticWorldData::BuildRecognitionDiagnostics()
{
	BuildVertexShapeSamples();
	BuildPlanarSurfaceGroups();
	BuildPlanarGroupMirrorMatches();
	BuildTriangleCurvatureEvidence();
	BuildCurvatureSurfaceRegions();
	BuildCurvatureRegionMirrorMatches();
	BuildExtrusionSurfaceRegions();
	BuildExtrusionRegionMirrorMatches();
	BuildQuarterEllipseBoundaryMatches();
	BuildC2TransitionSectionFits();
	BuildSharedC2TransitionPairFits();
	BuildSharedC2FrameLedger();
	BuildGlobalC2NetworkLedger();
	BuildSymmetrizedC2PlaneConstraints();
	BuildCoupledC2TransitionSolutions();
	BuildC2TransitionCoverage();
	BuildPlayableC2OrbitCandidates();
	BuildPlayableC2TriangleSupportCandidates();
	BuildPlayableC2PlaneBindings();
	BuildPlayableC2NetworkPlaneConstraints();
	BuildPlayableC2NetworkSolutions();
	BuildEndWallBoundaryComponents();
	BuildCanonicalOpenArchSolutions();
	BuildOpenRimSurfaceBandObservations();
	BuildOpenRimTransverseSections();
	BuildOpenRimTransitionFamilyFits();
	BuildOpenRimSupportTransitionIntents();
	BuildCompactRuntimePatches();
}

void FAnalyticWorldData::BuildCompactRuntimePatches()
{
	ExtrudedQuinticPatches.Reset();
	TensorBezierPatches.Reset();
	PiecewiseTensorBezierPatches.Reset();
	Planes.RemoveAll([](const FBoundedPlane& Plane)
	{
		return Plane.bRequiresCompactOptIn;
	});
	// Static architectural faces can carry sub-millimetre source quantization.
	// Keep the certification tight while admitting a plane whose complete
	// polygon deviates by no more than a quarter millimetre from its best fit.
	constexpr double MaximumAuthorityPlaneResidualCm = 0.025;
	TArray<FBoundedPlane> AdditionalPlanarDomainComponents;
	TArray<int32> SymmetrizedConstraintIndexByPlanarGroup;
	SymmetrizedConstraintIndexByPlanarGroup.Init(INDEX_NONE,
		PlanarSurfaceGroups.Num());
	for (int32 ConstraintIndex = 0;
		ConstraintIndex < SymmetrizedC2PlaneConstraints.Num(); ++ConstraintIndex)
	{
		const FSymmetrizedC2PlaneConstraint& Constraint =
			SymmetrizedC2PlaneConstraints[ConstraintIndex];
		if (SymmetrizedConstraintIndexByPlanarGroup.IsValidIndex(
				Constraint.PlanarGroupIndex))
		{
			SymmetrizedConstraintIndexByPlanarGroup[
				Constraint.PlanarGroupIndex] = ConstraintIndex;
		}
	}
	for (int32 PlanarGroupIndex = 0;
		PlanarGroupIndex < PlanarSurfaceGroups.Num(); ++PlanarGroupIndex)
	{
		const FPlanarSurfaceGroup& Group = PlanarSurfaceGroups[PlanarGroupIndex];
		if (!Group.bArchitecturalConstraint)
		{
			continue;
		}
		const int32 SymmetrizedConstraintIndex =
			SymmetrizedConstraintIndexByPlanarGroup[PlanarGroupIndex];
		const FSymmetrizedC2PlaneConstraint* SymmetrizedConstraint =
			SymmetrizedC2PlaneConstraints.IsValidIndex(SymmetrizedConstraintIndex)
				? &SymmetrizedC2PlaneConstraints[SymmetrizedConstraintIndex]
				: nullptr;
		const bool bSymmetrizedConstraintAvailable = SymmetrizedConstraint &&
			SymmetrizedConstraint->bSourceFitPlausible &&
			SymmetrizedConstraint->bExactMirrorPlacement;
		bool bUseSymmetrizedConstraint = bSymmetrizedConstraintAvailable;
		if (bUseSymmetrizedConstraint)
		{
			const FVector3d SymmetrizedNormal =
				SymmetrizedConstraint->Normal.GetSafeNormal();
			const FVector3d RawNormal = Group.Normal.GetSafeNormal();
			const FVector3d NormalDelta = SymmetrizedNormal - RawNormal;
			const double OffsetDelta =
				SymmetrizedConstraint->PlaneOffset - Group.PlaneOffset;
			double MaximumConstraintDeltaCm = 0.0;
			for (int32 Corner = 0; Corner < 8; ++Corner)
			{
				const FVector3d Point(
					(Corner & 1) != 0 ? Group.Bounds.Max.X : Group.Bounds.Min.X,
					(Corner & 2) != 0 ? Group.Bounds.Max.Y : Group.Bounds.Min.Y,
					(Corner & 4) != 0 ? Group.Bounds.Max.Z : Group.Bounds.Min.Z);
				MaximumConstraintDeltaCm = FMath::Max(
					MaximumConstraintDeltaCm,
					FMath::Abs(FVector3d::DotProduct(Point, NormalDelta) -
						OffsetDelta));
			}
			bUseSymmetrizedConstraint = Group.MaximumPlaneResidual +
				MaximumConstraintDeltaCm <= MaximumAuthorityPlaneResidualCm;
		}
		if (Group.PatchCount <= 0 || !PlanarGroupPatchIndices.IsValidIndex(
				Group.FirstPatchIndex))
		{
			continue;
		}
		const int32 PatchIndex = PlanarGroupPatchIndices[Group.FirstPatchIndex];
		if (!SurfacePatches.IsValidIndex(PatchIndex) ||
			SurfacePatches[PatchIndex].TriangleCount <= 0 ||
			!PatchTriangleIndices.IsValidIndex(
				SurfacePatches[PatchIndex].FirstTriangleIndex))
		{
			continue;
		}
		const int32 TriangleIndex = PatchTriangleIndices[
			SurfacePatches[PatchIndex].FirstTriangleIndex];
		if (!Triangles.IsValidIndex(TriangleIndex)) continue;
		const FTriangleSurface& SourceTriangle = Triangles[TriangleIndex];
		FVector3d Normal = (bUseSymmetrizedConstraint
			? SymmetrizedConstraint->Normal : Group.Normal).GetSafeNormal();
		const double PlaneOffset = bUseSymmetrizedConstraint
			? SymmetrizedConstraint->PlaneOffset : Group.PlaneOffset;
		FVector3d CandidateAxes[3] = {
			FVector3d::ForwardVector, FVector3d::RightVector,
			FVector3d::UpVector };
		FVector3d AxisU = FVector3d::ZeroVector;
		double BestAxisLengthSquared = -1.0;
		for (const FVector3d& CandidateAxis : CandidateAxes)
		{
			const FVector3d Projected = CandidateAxis -
				FVector3d::DotProduct(CandidateAxis, Normal) * Normal;
			if (Projected.SquaredLength() > BestAxisLengthSquared)
			{
				BestAxisLengthSquared = Projected.SquaredLength();
				AxisU = Projected.GetSafeNormal();
			}
		}
		const FVector3d AxisV =
			FVector3d::CrossProduct(Normal, AxisU).GetSafeNormal();
		double MinimumU = TNumericLimits<double>::Max();
		double MaximumU = -TNumericLimits<double>::Max();
		double MinimumV = TNumericLimits<double>::Max();
		double MaximumV = -TNumericLimits<double>::Max();
		double MaximumPlaneResidualCm = 0.0;
		bool bUniformCollisionPolicy = true;
		const int32 AdditionalComponentStart =
			AdditionalPlanarDomainComponents.Num();
		TArray<int32, TInlineAllocator<64>> GroupTriangleIndices;
		TSet<int32> GroupVertexSet;
		for (int32 PatchOffset = 0; PatchOffset < Group.PatchCount; ++PatchOffset)
		{
			const int32 MemberPatchIndex = PlanarGroupPatchIndices[
				Group.FirstPatchIndex + PatchOffset];
			const FSurfacePatch& MemberPatch = SurfacePatches[MemberPatchIndex];
			for (int32 TriangleOffset = 0;
				TriangleOffset < MemberPatch.TriangleCount; ++TriangleOffset)
			{
				const int32 MemberTriangleIndex = PatchTriangleIndices[
					MemberPatch.FirstTriangleIndex + TriangleOffset];
				const FTriangleSurface& MemberTriangle = Triangles[MemberTriangleIndex];
				bUniformCollisionPolicy &=
					MemberTriangle.SourceId == SourceTriangle.SourceId &&
					MemberTriangle.MaterialId == SourceTriangle.MaterialId &&
					MemberTriangle.ObjectType == SourceTriangle.ObjectType &&
					MemberTriangle.BlockingChannels == SourceTriangle.BlockingChannels &&
					MemberTriangle.bQueryCollisionEnabled ==
						SourceTriangle.bQueryCollisionEnabled;
				GroupTriangleIndices.Add(MemberTriangleIndex);
				if (TriangleVertexIndices.IsValidIndex(MemberTriangleIndex))
				{
					const FIntVector& VertexIndices =
						TriangleVertexIndices[MemberTriangleIndex];
					GroupVertexSet.Add(VertexIndices.X);
					GroupVertexSet.Add(VertexIndices.Y);
					GroupVertexSet.Add(VertexIndices.Z);
				}
				for (const FVector3d& Point : MemberTriangle.Vertices)
				{
					MaximumPlaneResidualCm = FMath::Max(
						MaximumPlaneResidualCm,
						FMath::Abs(FVector3d::DotProduct(Point, Normal) -
							PlaneOffset));
					const double U = FVector3d::DotProduct(Point, AxisU);
					const double V = FVector3d::DotProduct(Point, AxisV);
					MinimumU = FMath::Min(MinimumU, U);
					MaximumU = FMath::Max(MaximumU, U);
					MinimumV = FMath::Min(MinimumV, V);
					MaximumV = FMath::Max(MaximumV, V);
				}
			}
		}
		FBoundedPlane& Plane = Planes.AddDefaulted_GetRef();
		Plane.SourceId = SourceTriangle.SourceId;
		Plane.SurfaceId = SourceTriangle.SurfaceId;
		Plane.FeatureId = SourceTriangle.FeatureId;
		Plane.PrimitiveId = CombineStableIds(Group.GroupId,
			StableStringId(TEXT("BoundedPlane")));
		Plane.MaterialId = SourceTriangle.MaterialId;
		Plane.ObjectType = SourceTriangle.ObjectType;
		Plane.BlockingChannels = SourceTriangle.BlockingChannels;
		Plane.Normal = Normal;
		Plane.AxisU = AxisU;
		Plane.AxisV = AxisV;
		Plane.Origin = PlaneOffset * Normal +
			0.5 * (MinimumU + MaximumU) * AxisU +
			0.5 * (MinimumV + MaximumV) * AxisV;
		Plane.HalfExtents = FVector2d(
			0.5 * (MaximumU - MinimumU), 0.5 * (MaximumV - MinimumV));
		Plane.bQueryCollisionEnabled = SourceTriangle.bQueryCollisionEnabled;
		Plane.bRequiresCompactOptIn = true;
		Plane.bAuthorityEligible = false;
		const double RectangularDomainArea =
			(MaximumU - MinimumU) * (MaximumV - MinimumV);
		const double DomainFillRatio = RectangularDomainArea > 0.0
			? Group.Area / RectangularDomainArea : 0.0;
		constexpr double BoundaryToleranceCm = 0.05;
		TArray<int32, TInlineAllocator<128>> GroupVertexIndices;
		GroupVertexIndices.Reserve(GroupVertexSet.Num());
		for (const int32 VertexIndex : GroupVertexSet)
		{
			GroupVertexIndices.Add(VertexIndex);
		}
		GroupVertexIndices.Sort();
		TMap<FIntPoint, int32> SubdividedEdgeUseCounts;
		for (const int32 MemberTriangleIndex : GroupTriangleIndices)
		{
			if (!TriangleVertexIndices.IsValidIndex(MemberTriangleIndex)) continue;
			const FIntVector& TriangleVertices =
				TriangleVertexIndices[MemberTriangleIndex];
			const int32 VertexIndices[3] = {
				TriangleVertices.X, TriangleVertices.Y, TriangleVertices.Z };
			for (int32 EdgeIndex = 0; EdgeIndex < 3; ++EdgeIndex)
			{
				const int32 VertexA = VertexIndices[EdgeIndex];
				const int32 VertexB = VertexIndices[(EdgeIndex + 1) % 3];
				if (!MeshVertices.IsValidIndex(VertexA) ||
					!MeshVertices.IsValidIndex(VertexB)) continue;
				const FVector3d PointA = MeshVertices[VertexA];
				const FVector3d Edge = MeshVertices[VertexB] - PointA;
				const double EdgeLengthSquared = Edge.SquaredLength();
				if (EdgeLengthSquared <= UE_DOUBLE_SMALL_NUMBER) continue;
				struct FSplitVertex
				{
					double Parameter = 0.0;
					int32 VertexIndex = INDEX_NONE;
				};
				TArray<FSplitVertex, TInlineAllocator<16>> SplitVertices;
				for (const int32 CandidateVertex : GroupVertexIndices)
				{
					const FVector3d Candidate = MeshVertices[CandidateVertex];
					const double Parameter = FVector3d::DotProduct(
						Candidate - PointA, Edge) / EdgeLengthSquared;
					if (Parameter < 0.0 || Parameter > 1.0) continue;
					const FVector3d Closest = PointA + Parameter * Edge;
					if ((Candidate - Closest).SquaredLength() <=
						FMath::Square(BoundaryToleranceCm))
					{
						SplitVertices.Add({ Parameter, CandidateVertex });
					}
				}
				SplitVertices.Sort([](const FSplitVertex& A, const FSplitVertex& B)
				{
					if (A.Parameter != B.Parameter) return A.Parameter < B.Parameter;
					return A.VertexIndex < B.VertexIndex;
				});
				for (int32 SplitIndex = 1;
					SplitIndex < SplitVertices.Num(); ++SplitIndex)
				{
					const int32 SplitA = SplitVertices[SplitIndex - 1].VertexIndex;
					const int32 SplitB = SplitVertices[SplitIndex].VertexIndex;
					if (SplitA == SplitB) continue;
					const FIntPoint Segment(
						FMath::Min(SplitA, SplitB), FMath::Max(SplitA, SplitB));
					++SubdividedEdgeUseCounts.FindOrAdd(Segment);
				}
			}
		}
		int32 BoundaryEdgeCount = 0;
		uint8 BoundarySideMask = 0;
		bool bBoundaryUsesOnlyRectangleSides = true;
		bool bBoundaryManifold = true;
		TMap<int32, int32> BoundaryVertexDegrees;
		TMultiMap<int32, int32> BoundaryAdjacency;
		for (const TPair<FIntPoint, int32>& EdgeUse : SubdividedEdgeUseCounts)
		{
			if (EdgeUse.Value == 2) continue;
			if (EdgeUse.Value != 1)
			{
				bBoundaryManifold = false;
				continue;
			}
			++BoundaryEdgeCount;
			const int32 VertexA = EdgeUse.Key.X;
			const int32 VertexB = EdgeUse.Key.Y;
			if (!MeshVertices.IsValidIndex(VertexA) ||
				!MeshVertices.IsValidIndex(VertexB))
			{
				bBoundaryManifold = false;
				continue;
			}
			++BoundaryVertexDegrees.FindOrAdd(VertexA);
			++BoundaryVertexDegrees.FindOrAdd(VertexB);
			BoundaryAdjacency.Add(VertexA, VertexB);
			BoundaryAdjacency.Add(VertexB, VertexA);
			const FVector3d& PointA = MeshVertices[VertexA];
			const FVector3d& PointB = MeshVertices[VertexB];
			const double Ua = FVector3d::DotProduct(PointA, AxisU);
			const double Ub = FVector3d::DotProduct(PointB, AxisU);
			const double Va = FVector3d::DotProduct(PointA, AxisV);
			const double Vb = FVector3d::DotProduct(PointB, AxisV);
			uint8 EdgeSide = 0;
			if (FMath::Abs(Ua - MinimumU) <= BoundaryToleranceCm &&
				FMath::Abs(Ub - MinimumU) <= BoundaryToleranceCm) EdgeSide |= 1;
			if (FMath::Abs(Ua - MaximumU) <= BoundaryToleranceCm &&
				FMath::Abs(Ub - MaximumU) <= BoundaryToleranceCm) EdgeSide |= 2;
			if (FMath::Abs(Va - MinimumV) <= BoundaryToleranceCm &&
				FMath::Abs(Vb - MinimumV) <= BoundaryToleranceCm) EdgeSide |= 4;
			if (FMath::Abs(Va - MaximumV) <= BoundaryToleranceCm &&
				FMath::Abs(Vb - MaximumV) <= BoundaryToleranceCm) EdgeSide |= 8;
			bBoundaryUsesOnlyRectangleSides &= EdgeSide != 0;
			BoundarySideMask |= EdgeSide;
		}
		int32 BoundaryDegreeOneCount = 0;
		int32 BoundaryDegreeTwoCount = 0;
		int32 BoundaryOtherDegreeCount = 0;
		for (const TPair<int32, int32>& VertexDegree : BoundaryVertexDegrees)
		{
			bBoundaryManifold &= VertexDegree.Value == 2;
			BoundaryDegreeOneCount += VertexDegree.Value == 1 ? 1 : 0;
			BoundaryDegreeTwoCount += VertexDegree.Value == 2 ? 1 : 0;
			BoundaryOtherDegreeCount += VertexDegree.Value > 2 ? 1 : 0;
		}
		TSet<int32> VisitedBoundaryVertices;
		if (!BoundaryVertexDegrees.IsEmpty())
		{
			TArray<int32, TInlineAllocator<32>> Stack;
			Stack.Add(BoundaryVertexDegrees.CreateConstIterator()->Key);
			while (!Stack.IsEmpty())
			{
				const int32 VertexIndex = Stack.Pop(EAllowShrinking::No);
				if (VisitedBoundaryVertices.Contains(VertexIndex)) continue;
				VisitedBoundaryVertices.Add(VertexIndex);
				TArray<int32, TInlineAllocator<4>> AdjacentVertices;
				BoundaryAdjacency.MultiFind(VertexIndex, AdjacentVertices);
				Stack.Append(AdjacentVertices);
			}
		}
		const bool bSingleBoundaryLoop = BoundaryEdgeCount > 0 &&
			VisitedBoundaryVertices.Num() == BoundaryVertexDegrees.Num();
		if (bBoundaryManifold && bSingleBoundaryLoop)
		{
			int32 StartVertex = MAX_int32;
			for (const TPair<int32, int32>& VertexDegree : BoundaryVertexDegrees)
			{
				StartVertex = FMath::Min(StartVertex, VertexDegree.Key);
			}
			int32 PreviousVertex = INDEX_NONE;
			int32 CurrentVertex = StartVertex;
			for (int32 VertexOffset = 0;
				VertexOffset < BoundaryVertexDegrees.Num(); ++VertexOffset)
			{
				const FVector3d Relative = MeshVertices[CurrentVertex] - Plane.Origin;
				Plane.DomainVertices.Add(FVector2d(
					FVector3d::DotProduct(Relative, AxisU),
					FVector3d::DotProduct(Relative, AxisV)));
				TArray<int32, TInlineAllocator<4>> AdjacentVertices;
				BoundaryAdjacency.MultiFind(CurrentVertex, AdjacentVertices);
				AdjacentVertices.Sort();
				if (AdjacentVertices.Num() != 2)
				{
					Plane.DomainVertices.Reset();
					break;
				}
				const int32 NextVertex = AdjacentVertices[0] == PreviousVertex
					? AdjacentVertices[1] : AdjacentVertices[0];
				PreviousVertex = CurrentVertex;
				CurrentVertex = NextVertex;
			}
			if (CurrentVertex != StartVertex ||
				Plane.DomainVertices.Num() != BoundaryVertexDegrees.Num())
			{
				Plane.DomainVertices.Reset();
			}
		}
		int32 PolygonDomainComponentCount = Plane.DomainVertices.IsEmpty() ? 0 : 1;
		if (Plane.DomainVertices.IsEmpty() && BoundaryDegreeOneCount == 0 &&
			BoundaryOtherDegreeCount == 1)
		{
			int32 JunctionVertex = INDEX_NONE;
			for (const TPair<int32, int32>& VertexDegree : BoundaryVertexDegrees)
			{
				if (VertexDegree.Value > 2)
				{
					JunctionVertex = VertexDegree.Key;
					break;
				}
			}
			TArray<int32, TInlineAllocator<8>> JunctionNeighbors;
			BoundaryAdjacency.MultiFind(JunctionVertex, JunctionNeighbors);
			JunctionNeighbors.Sort();
			TSet<int32> VisitedWithoutJunction;
			TArray<TArray<int32>> ComponentLoops;
			for (const TPair<int32, int32>& VertexDegree : BoundaryVertexDegrees)
			{
				const int32 SeedVertex = VertexDegree.Key;
				if (SeedVertex == JunctionVertex ||
					VisitedWithoutJunction.Contains(SeedVertex)) continue;
				TSet<int32> ComponentVertices;
				TArray<int32, TInlineAllocator<32>> Stack;
				Stack.Add(SeedVertex);
				while (!Stack.IsEmpty())
				{
					const int32 VertexIndex = Stack.Pop(EAllowShrinking::No);
					if (VertexIndex == JunctionVertex ||
						ComponentVertices.Contains(VertexIndex)) continue;
					ComponentVertices.Add(VertexIndex);
					VisitedWithoutJunction.Add(VertexIndex);
					TArray<int32, TInlineAllocator<4>> AdjacentVertices;
					BoundaryAdjacency.MultiFind(VertexIndex, AdjacentVertices);
					Stack.Append(AdjacentVertices);
				}
				TArray<int32, TInlineAllocator<4>> ComponentJunctionNeighbors;
				for (const int32 Neighbor : JunctionNeighbors)
				{
					if (ComponentVertices.Contains(Neighbor))
					{
						ComponentJunctionNeighbors.Add(Neighbor);
					}
				}
				if (ComponentJunctionNeighbors.Num() != 2)
				{
					ComponentLoops.Reset();
					break;
				}
				ComponentJunctionNeighbors.Sort();
				TArray<int32> Loop;
				Loop.Add(JunctionVertex);
				int32 PreviousVertex = JunctionVertex;
				int32 CurrentVertex = ComponentJunctionNeighbors[0];
				while (CurrentVertex != JunctionVertex &&
					Loop.Num() <= ComponentVertices.Num())
				{
					Loop.Add(CurrentVertex);
					TArray<int32, TInlineAllocator<4>> AdjacentVertices;
					BoundaryAdjacency.MultiFind(CurrentVertex, AdjacentVertices);
					AdjacentVertices.Sort();
					int32 NextVertex = INDEX_NONE;
					for (const int32 CandidateVertex : AdjacentVertices)
					{
						if (CandidateVertex != PreviousVertex &&
							(CandidateVertex == JunctionVertex ||
								ComponentVertices.Contains(CandidateVertex)))
						{
							NextVertex = CandidateVertex;
							break;
						}
					}
					PreviousVertex = CurrentVertex;
					CurrentVertex = NextVertex;
				}
				if (CurrentVertex != JunctionVertex ||
					Loop.Num() != ComponentVertices.Num() + 1)
				{
					ComponentLoops.Reset();
					break;
				}
				ComponentLoops.Add(MoveTemp(Loop));
			}
			ComponentLoops.Sort([](const TArray<int32>& A, const TArray<int32>& B)
			{
				const int32 MinimumA = A.IsEmpty()
					? MAX_int32 : *Algo::MinElement(A);
				const int32 MinimumB = B.IsEmpty()
					? MAX_int32 : *Algo::MinElement(B);
				return MinimumA < MinimumB;
			});
			for (int32 ComponentIndex = 0;
				ComponentIndex < ComponentLoops.Num(); ++ComponentIndex)
			{
				FBoundedPlane ComponentPlane = Plane;
				ComponentPlane.DomainVertices.Reset();
				for (const int32 VertexIndex : ComponentLoops[ComponentIndex])
				{
					const FVector3d Relative =
						MeshVertices[VertexIndex] - ComponentPlane.Origin;
					ComponentPlane.DomainVertices.Add(FVector2d(
						FVector3d::DotProduct(Relative, AxisU),
						FVector3d::DotProduct(Relative, AxisV)));
				}
				if (ComponentIndex == 0)
				{
					Plane.DomainVertices = MoveTemp(ComponentPlane.DomainVertices);
				}
				else
				{
					ComponentPlane.PrimitiveId = CombineStableIds(
						Plane.PrimitiveId, CombineStableIds(
							StableStringId(TEXT("PlanarDomainComponent")),
							static_cast<uint64>(ComponentIndex)));
					AdditionalPlanarDomainComponents.Add(MoveTemp(ComponentPlane));
				}
			}
			PolygonDomainComponentCount = ComponentLoops.Num();
		}
		const bool bRectangleBoundaryCandidate = bBoundaryManifold &&
			bSingleBoundaryLoop && bBoundaryUsesOnlyRectangleSides &&
			BoundarySideMask == 0x0f;
		const bool bPlanarDomainQualified = !Plane.DomainVertices.IsEmpty();
		auto PolygonArea = [](const TArray<FVector2d>& Vertices)
		{
			double TwiceArea = 0.0;
			for (int32 VertexIndex = 0; VertexIndex < Vertices.Num(); ++VertexIndex)
			{
				const FVector2d& A = Vertices[VertexIndex];
				const FVector2d& B = Vertices[(VertexIndex + 1) % Vertices.Num()];
				TwiceArea += A.X * B.Y - A.Y * B.X;
			}
			return 0.5 * FMath::Abs(TwiceArea);
		};
		double PolygonDomainArea = PolygonArea(Plane.DomainVertices);
		for (int32 ComponentIndex = AdditionalComponentStart;
			ComponentIndex < AdditionalPlanarDomainComponents.Num(); ++ComponentIndex)
		{
			PolygonDomainArea += PolygonArea(
				AdditionalPlanarDomainComponents[ComponentIndex].DomainVertices);
		}
		const double DomainAreaResidualCm2 =
			FMath::Abs(PolygonDomainArea - Group.Area);
		const double MaximumAuthorityAreaResidualCm2 =
			FMath::Max(0.01, Group.Area * 1.0e-8);
		const bool bFaceInteriorAuthorityCertified =
			bPlanarDomainQualified && bUniformCollisionPolicy &&
			MaximumPlaneResidualCm <= MaximumAuthorityPlaneResidualCm &&
			DomainAreaResidualCm2 <= MaximumAuthorityAreaResidualCm2;
		Plane.bAuthorityEligible = bFaceInteriorAuthorityCertified;
		for (int32 ComponentIndex = AdditionalComponentStart;
			ComponentIndex < AdditionalPlanarDomainComponents.Num(); ++ComponentIndex)
		{
			AdditionalPlanarDomainComponents[ComponentIndex].bAuthorityEligible =
				bFaceInteriorAuthorityCertified;
		}
		UE_LOG(LogTemp, Display,
			TEXT("[AnalyticPlaneDomain] Primitive=%016llX Group=%016llX Triangles=%d SourceAreaCm2=%.9g PolygonAreaCm2=%.9g AreaResidualCm2=%.9g PlaneResidualCm=%.9g RawPlaneResidualCm=%.9g Symmetrized=%d Normal=%s PlaneOffset=%.17g BoundsMin=%s BoundsMax=%s UniformPolicy=%d FaceInteriorAuthority=%d RectangularAreaCm2=%.9g FillRatio=%.9g BoundaryEdges=%d BoundarySides=%u BoundaryManifold=%d BoundaryDegrees=%d/%d/%d SingleLoop=%d PolygonVertices=%d PolygonComponents=%d RectangleBoundary=%d Qualified=%d"),
			Plane.PrimitiveId, Group.GroupId, Group.TriangleCount,
			Group.Area, PolygonDomainArea,
			DomainAreaResidualCm2, MaximumPlaneResidualCm,
			Group.MaximumPlaneResidual,
			bUseSymmetrizedConstraint ? 1 : 0,
			*Normal.ToString(), PlaneOffset,
			*Group.Bounds.Min.ToString(), *Group.Bounds.Max.ToString(),
			bUniformCollisionPolicy ? 1 : 0,
			bFaceInteriorAuthorityCertified ? 1 : 0,
			RectangularDomainArea, DomainFillRatio,
			BoundaryEdgeCount, static_cast<uint32>(BoundarySideMask),
			bBoundaryManifold ? 1 : 0, BoundaryDegreeOneCount,
			BoundaryDegreeTwoCount, BoundaryOtherDegreeCount,
			bSingleBoundaryLoop ? 1 : 0,
			Plane.DomainVertices.Num(),
			PolygonDomainComponentCount,
			bRectangleBoundaryCandidate ? 1 : 0,
			bPlanarDomainQualified ? 1 : 0);
		if (!bBoundaryManifold)
		{
			for (const TPair<int32, int32>& VertexDegree : BoundaryVertexDegrees)
			{
				if (VertexDegree.Value == 2 ||
					!MeshVertices.IsValidIndex(VertexDegree.Key)) continue;
				UE_LOG(LogTemp, Display,
					TEXT("[AnalyticPlaneBoundaryJunction] Primitive=%016llX Vertex=%d Degree=%d Position=%s"),
					Plane.PrimitiveId, VertexDegree.Key, VertexDegree.Value,
					*MeshVertices[VertexDegree.Key].ToString());
			}
		}
		if (!bPlanarDomainQualified)
		{
			// A synthesized mesh plane without a certified boundary would fall back
			// to its enclosing rectangle and could claim empty space. Omit it from
			// compact queries until its source membership forms a valid polygon.
			Planes.Pop(EAllowShrinking::No);
		}
	}
	Planes.Append(MoveTemp(AdditionalPlanarDomainComponents));
	// The source collision keeps a broad coplanar floor sheet underneath the
	// rounded lower gutters.  Chaos' mesh arbitration lets the rising gutter
	// take ownership, but a compact analytic sweep that starts tangent to the
	// plane otherwise keeps returning the plane at t=0 and masks the curved
	// provider.  Cede the planar domain at the source-fitted C2 tangent locus.
	//
	// These symmetric coordinates come from the canonical lower-gutter fits:
	//   end return:     |X| = 5074.417201 cm
	//   lateral return: |Y| = 4046.627923 cm
	//   corner return:  |X| + |Y| = 7996.102569 cm
	// The resulting octagon is exactly symmetric and meets every gutter at its
	// flat endpoint; it changes ownership only, not the authored geometry.
	constexpr double MainFloorEndTangentAbsX = 5074.417201;
	constexpr double MainFloorSideTangentAbsY = 4046.627923;
	constexpr double MainFloorCornerTangentSum = 7996.102569;
	constexpr double MainFloorCornerAbsYAtEnd =
		MainFloorCornerTangentSum - MainFloorEndTangentAbsX;
	constexpr double MainFloorCornerAbsXAtSide =
		MainFloorCornerTangentSum - MainFloorSideTangentAbsY;
	int32 MainFloorCededPlaneCount = 0;
	for (FBoundedPlane& Plane : Planes)
	{
		if (!Plane.bAuthorityEligible ||
			FMath::Abs(Plane.Normal.Z) < 1.0 - 1.0e-9 ||
			FMath::Abs(Plane.Origin.Z) > MaximumAuthorityPlaneResidualCm ||
			Plane.HalfExtents.X < 5600.0 || Plane.HalfExtents.Y < 4000.0)
		{
			continue;
		}
		const FVector3d WorldVertices[] = {
			FVector3d(-MainFloorCornerAbsXAtSide, -MainFloorSideTangentAbsY, 0.0),
			FVector3d( MainFloorCornerAbsXAtSide, -MainFloorSideTangentAbsY, 0.0),
			FVector3d( MainFloorEndTangentAbsX, -MainFloorCornerAbsYAtEnd, 0.0),
			FVector3d( MainFloorEndTangentAbsX,  MainFloorCornerAbsYAtEnd, 0.0),
			FVector3d( MainFloorCornerAbsXAtSide,  MainFloorSideTangentAbsY, 0.0),
			FVector3d(-MainFloorCornerAbsXAtSide,  MainFloorSideTangentAbsY, 0.0),
			FVector3d(-MainFloorEndTangentAbsX,  MainFloorCornerAbsYAtEnd, 0.0),
			FVector3d(-MainFloorEndTangentAbsX, -MainFloorCornerAbsYAtEnd, 0.0),
		};
		Plane.DomainVertices.Reset(UE_ARRAY_COUNT(WorldVertices));
		for (const FVector3d& WorldVertex : WorldVertices)
		{
			const FVector3d Relative = WorldVertex - Plane.Origin;
			Plane.DomainVertices.Add(FVector2d(
				FVector3d::DotProduct(Relative, Plane.AxisU),
				FVector3d::DotProduct(Relative, Plane.AxisV)));
		}
		++MainFloorCededPlaneCount;
	}
	UE_LOG(LogTemp, Display, TEXT(
		"[AnalyticMainFloorGutterCession] Planes=%d EndAbsX=%.6f SideAbsY=%.6f CornerSum=%.6f"),
		MainFloorCededPlaneCount, MainFloorEndTangentAbsX,
		MainFloorSideTangentAbsY, MainFloorCornerTangentSum);
	// Local goal-floor ownership below still needs a source record.  The former
	// LocalizedVerticalClosure is deliberately absent: its X=5600..5680 planar
	// band extended the lateral wall beyond the authored mesh end at X~=5580 and
	// could win a backboard-to-gutter hit before the source-certified C2 fillet.
	// That artificial wall violated the finite-domain certificate and produced a
	// blocking contact in otherwise empty space.
	if (!Triangles.IsEmpty())
	{
		const FTriangleSurface& ClosureSource = Triangles[0];
		// Keep the playable goal floor level with the main field until the lower
		// post return begins.  The authored collision contains a second horizontal
		// branch at Z=0 in this envelope.  This compact plane is exactly C2 with
		// the flat endpoint of the quintic below and is mirrored only along the two
		// goal envelopes.
		// The end-return extrusion starts at |Y|=950 cm.  Cover the complete goal
		// opening up to that same tangent seam so trimming the main floor cannot
		// expose an uncovered strip beside the posts.
		constexpr double GoalFloorHalfExtentY = 950.0;
		// The cage floor is a continuation of the main field, not a raised
		// platform.  Starting this patch at the field-side envelope also avoids
		// exposing a finite leading edge to a moving wheel/hitbox entering the
		// goal box (the previous 5452 cm edge stopped the squishy-save path).
		constexpr double GoalFloorZ = 0.0;
		constexpr double GoalFloorMinimumAbsX = 0.0;
		// The outer rear wheel reaches |X|~=6620 while climbing the goal arc.
		// Extend only this localized floor/transition extrusion to the playable
		// envelope so the wheel sphere cannot fall outside its provider domain.
		constexpr double GoalFloorMaximumAbsX = 6700.0;
		constexpr double GoalFloorCenterAbsX =
			0.5 * (GoalFloorMinimumAbsX + GoalFloorMaximumAbsX);
		constexpr double GoalFloorHalfExtentX =
			0.5 * (GoalFloorMaximumAbsX - GoalFloorMinimumAbsX);
		for (const double XSign : { -1.0, 1.0 })
		{
			FBoundedPlane& GoalFloor = Planes.AddDefaulted_GetRef();
			GoalFloor.SourceId = ClosureSource.SourceId;
			GoalFloor.SurfaceId = CombineStableIds(
				StableStringId(TEXT("LocalizedGoalFloor.Surface")),
				XSign < 0.0 ? 1ull : 2ull);
			GoalFloor.FeatureId = CombineStableIds(
				GoalFloor.SurfaceId,
				StableStringId(TEXT("LocalizedGoalFloor.Feature")));
			GoalFloor.PrimitiveId = CombineStableIds(
				GoalFloor.SurfaceId,
				StableStringId(TEXT("BoundedPlane")));
			GoalFloor.MaterialId = ClosureSource.MaterialId;
			GoalFloor.ObjectType = ClosureSource.ObjectType;
			GoalFloor.BlockingChannels = ClosureSource.BlockingChannels;
			GoalFloor.Normal = FVector3d::UpVector;
			GoalFloor.AxisU = FVector3d::ForwardVector;
			GoalFloor.AxisV = FVector3d::RightVector;
			GoalFloor.Origin = FVector3d(
				XSign * GoalFloorCenterAbsX, 0.0, GoalFloorZ);
			GoalFloor.HalfExtents = FVector2d(
				GoalFloorHalfExtentX, GoalFloorHalfExtentY);
			GoalFloor.DomainVertices = {
				FVector2d(-GoalFloorHalfExtentX, -GoalFloorHalfExtentY),
				FVector2d(GoalFloorHalfExtentX, -GoalFloorHalfExtentY),
				FVector2d(GoalFloorHalfExtentX, GoalFloorHalfExtentY),
				FVector2d(-GoalFloorHalfExtentX, GoalFloorHalfExtentY) };
			GoalFloor.bQueryCollisionEnabled =
				ClosureSource.bQueryCollisionEnabled;
			GoalFloor.bRequiresCompactOptIn = true;
			GoalFloor.bAuthorityEligible = GoalFloor.bQueryCollisionEnabled;
		}
		UE_LOG(LogTemp, Display, TEXT(
			"[AnalyticLocalizedGoalFloor] Count=2 AbsX=[%.3f,%.3f] Y=[%.3f,%.3f] Z=%.3f Certified=1"),
			GoalFloorMinimumAbsX, GoalFloorMaximumAbsX, -GoalFloorHalfExtentY,
			GoalFloorHalfExtentY, GoalFloorZ);

		// Do not serialize the former separable lower transition.  Its profile was
		// plausible in a transverse section, but uniform longitudinal extrusion
		// made it depart from the source by hundreds of centimetres at the finite
		// ends.  The source-fitted two-dimensional lower-corner tensor network
		// below is the sole authority for this genuinely two-dimensional junction.
		UE_LOG(LogTemp, Display, TEXT(
			"[AnalyticLocalizedVerticalTransition] Count=0 ReplacedBy=LowerFieldCornerC2"));

		// The old finite X=+/-6575 goal-edge planes are intentionally absent.
		// They crossed the rounded source sheet by more than 15 cm and exposed a
		// hard upper edge.  The source-fitted finite interior-cap network below
		// now owns this junction and overlaps the transverse boundary providers.
		UE_LOG(LogTemp, Display, TEXT(
			"[AnalyticLocalizedGoalEdgeClosure] Count=0 ReplacedBy=FiniteInteriorCapC2"));
	}
	TSet<int32> AddedFitIndices;
	for (const FC2TransitionCoverageEntry& Coverage : C2TransitionCoverage)
	{
		if (Coverage.SurfaceLayer != EC2TransitionSurfaceLayer::PlayableInner ||
			!C2TransitionSectionFits.IsValidIndex(Coverage.TransitionFitIndex) ||
			AddedFitIndices.Contains(Coverage.TransitionFitIndex))
		{
			continue;
		}
		const FC2TransitionSectionFit& Fit =
			C2TransitionSectionFits[Coverage.TransitionFitIndex];
		if (!Fit.bBoundaryEvidenceUsable ||
			!ExtrusionSurfaceRegions.IsValidIndex(Fit.ExtrusionRegionIndex))
		{
			continue;
		}
		const FExtrusionSurfaceRegion& Region =
			ExtrusionSurfaceRegions[Fit.ExtrusionRegionIndex];
		if (Region.TriangleCount <= 0 ||
			!ExtrusionRegionTriangleIndices.IsValidIndex(Region.FirstTriangleIndex))
		{
			continue;
		}
		const int32 SourceTriangleIndex =
			ExtrusionRegionTriangleIndices[Region.FirstTriangleIndex];
		if (!Triangles.IsValidIndex(SourceTriangleIndex)) continue;
		const FTriangleSurface& SourceTriangle = Triangles[SourceTriangleIndex];

		FVector2d CanonicalCenter = Fit.CenterCoordinates;
		double CanonicalRadiusU = Fit.RadiusU;
		double CanonicalRadiusV = Fit.RadiusV;
		double CanonicalFlattening = Fit.FlatteningFraction;
		FVector3d CanonicalBasisU = Region.SectionAxisU;
		FVector3d CanonicalBasisV = Region.SectionAxisV;
		FVector3d CanonicalNetworkControlPoints[6] = {};
		bool bHasCanonicalNetworkControls = false;
		bool bUsesCanonicalFamily = false;
		uint64 CanonicalGroupId = 0;
		uint8 CanonicalSymmetryAxisMask = 0;
		for (int32 CandidateIndex = 0;
			!bUsesCanonicalFamily &&
			CandidateIndex < PlayableC2TriangleSupportCandidates.Num();
			++CandidateIndex)
		{
			const FPlayableC2TriangleSupportCandidate& Candidate =
				PlayableC2TriangleSupportCandidates[CandidateIndex];
			if (!Candidate.bNetworkExactC0G1C2ByConstruction ||
				!Candidate.bNetworkSourceFitPlausible)
			{
				continue;
			}
			const FPlayableC2OrbitCandidate* Orbit =
				PlayableC2OrbitCandidates.FindByPredicate(
					[&](const FPlayableC2OrbitCandidate& Value)
					{
						return Value.OrbitId == Candidate.OrbitId;
					});
			if (!Orbit) continue;
			uint8 TransformMask = 0;
			bool bMemberFound = false;
			for (int32 MemberOffset = 0;
				MemberOffset < Orbit->MemberCount; ++MemberOffset)
			{
				const FPlayableC2OrbitMember& Member = PlayableC2OrbitMembers[
					Orbit->FirstMemberIndex + MemberOffset];
				if (Member.ExtrusionRegionIndex == Coverage.ExtrusionRegionIndex)
				{
					TransformMask = Member.TransformMaskFromSeed;
					bMemberFound = true;
					break;
				}
			}
			if (!bMemberFound) continue;
			const FPlayableC2PlaneBinding* Bindings[2] = { nullptr, nullptr };
			for (const FPlayableC2PlaneBinding& Binding : PlayableC2PlaneBindings)
			{
				if (Binding.TriangleSupportCandidateIndex != CandidateIndex ||
					!Binding.bNetworkCompatible)
				{
					continue;
				}
				Bindings[Binding.Endpoint == EQuarterEllipseEndpoint::U ? 0 : 1] =
					&Binding;
			}
			if (!Bindings[0] || !Bindings[1] ||
				!PlayableC2NetworkPlaneConstraints.IsValidIndex(
					Bindings[0]->NetworkPlaneConstraintIndex) ||
				!PlayableC2NetworkPlaneConstraints.IsValidIndex(
					Bindings[1]->NetworkPlaneConstraintIndex))
			{
				continue;
			}
			auto ReflectNormal = [TransformMask](FVector3d Value)
			{
				if ((TransformMask & 1u) != 0) Value.X *= -1.0;
				if ((TransformMask & 2u) != 0) Value.Y *= -1.0;
				return Value;
			};
			FVector3d SeedNormalU =
				PlayableC2NetworkPlaneConstraints[
					Bindings[0]->NetworkPlaneConstraintIndex].Normal;
			double SeedOffsetU = PlayableC2NetworkPlaneConstraints[
				Bindings[0]->NetworkPlaneConstraintIndex].PlaneOffset;
			FVector3d SeedNormalV =
				PlayableC2NetworkPlaneConstraints[
					Bindings[1]->NetworkPlaneConstraintIndex].Normal;
			double SeedOffsetV = PlayableC2NetworkPlaneConstraints[
				Bindings[1]->NetworkPlaneConstraintIndex].PlaneOffset;
			if (!C2TransitionCoverage.IsValidIndex(Candidate.SeedCoverageIndex))
			{
				continue;
			}
			const FC2TransitionCoverageEntry& SeedCoverage =
				C2TransitionCoverage[Candidate.SeedCoverageIndex];
			if (!C2TransitionSectionFits.IsValidIndex(
					SeedCoverage.TransitionFitIndex) ||
				!ExtrusionSurfaceRegions.IsValidIndex(
					SeedCoverage.ExtrusionRegionIndex))
			{
				continue;
			}
			const FC2TransitionSectionFit& SeedFit =
				C2TransitionSectionFits[SeedCoverage.TransitionFitIndex];
			const FExtrusionSurfaceRegion& SeedRegion =
				ExtrusionSurfaceRegions[SeedCoverage.ExtrusionRegionIndex];
			if (FVector3d::DotProduct(SeedNormalU, SeedRegion.SectionAxisU) < 0.0)
			{
				SeedNormalU *= -1.0;
				SeedOffsetU *= -1.0;
			}
			if (FVector3d::DotProduct(SeedNormalV, SeedRegion.SectionAxisV) < 0.0)
			{
				SeedNormalV *= -1.0;
				SeedOffsetV *= -1.0;
			}
			const double Dot = FVector3d::DotProduct(SeedNormalU, SeedNormalV);
			const double Denominator = 1.0 - Dot * Dot;
			if (Denominator <= 1.0e-8) continue;
			const FVector3d SeedBasisU =
				(SeedNormalU - Dot * SeedNormalV) / Denominator;
			const FVector3d SeedBasisV =
				(SeedNormalV - Dot * SeedNormalU) / Denominator;
			const FVector2d SeedCenter(
				SeedOffsetU - SeedFit.SignU * Candidate.NetworkSharedRadiusU,
				SeedOffsetV - SeedFit.SignV * Candidate.NetworkSharedRadiusV);
			FVector2d SeedControlPoints[6];
			BuildQuinticTransitionControlPoints(SeedCenter,
				Candidate.NetworkSharedRadiusU, Candidate.NetworkSharedRadiusV,
				SeedFit.SignU, SeedFit.SignV,
				Candidate.NetworkSharedFlatteningFraction, SeedControlPoints);
			for (int32 ControlIndex = 0; ControlIndex < 6; ++ControlIndex)
			{
				CanonicalNetworkControlPoints[ControlIndex] = ReflectNormal(
					SeedControlPoints[ControlIndex].X * SeedBasisU +
					SeedControlPoints[ControlIndex].Y * SeedBasisV);
			}
			bHasCanonicalNetworkControls = true;
			bUsesCanonicalFamily = true;
			CanonicalGroupId = Candidate.OrbitId;
			CanonicalSymmetryAxisMask = Orbit->SymmetryAxisMask;
			UE_LOG(LogTemp, Display,
				TEXT("[AnalyticCompactCanonicalSource] Region=%016llX Source=Network Orbit=%016llX Mask=%u"),
				Region.RegionId, Candidate.OrbitId,
				static_cast<uint32>(TransformMask));
		}
		if (!bUsesCanonicalFamily &&
			CoupledC2TransitionFamilies.IsValidIndex(Coverage.FamilyIndex))
		{
			const FCoupledC2TransitionFamilySolution& Family =
				CoupledC2TransitionFamilies[Coverage.FamilyIndex];
			if (Family.bExactC0G1C2ByConstruction && Family.bSourceFitPlausible &&
				QuarterEllipseBoundaryMatches.IsValidIndex(Fit.BoundaryMatchUIndex) &&
				QuarterEllipseBoundaryMatches.IsValidIndex(Fit.BoundaryMatchVIndex))
			{
				const int32 PlaneUIndex = QuarterEllipseBoundaryMatches[
					Fit.BoundaryMatchUIndex].PlanarGroupIndex;
				const int32 PlaneVIndex = QuarterEllipseBoundaryMatches[
					Fit.BoundaryMatchVIndex].PlanarGroupIndex;
				const int32 ConstraintUIndex =
					SymmetrizedConstraintIndexByPlanarGroup.IsValidIndex(PlaneUIndex)
						? SymmetrizedConstraintIndexByPlanarGroup[PlaneUIndex] : INDEX_NONE;
				const int32 ConstraintVIndex =
					SymmetrizedConstraintIndexByPlanarGroup.IsValidIndex(PlaneVIndex)
						? SymmetrizedConstraintIndexByPlanarGroup[PlaneVIndex] : INDEX_NONE;
				if (SymmetrizedC2PlaneConstraints.IsValidIndex(ConstraintUIndex) &&
					SymmetrizedC2PlaneConstraints.IsValidIndex(ConstraintVIndex))
				{
					FVector3d NormalU =
						SymmetrizedC2PlaneConstraints[ConstraintUIndex].Normal;
					double OffsetU =
						SymmetrizedC2PlaneConstraints[ConstraintUIndex].PlaneOffset;
					FVector3d NormalV =
						SymmetrizedC2PlaneConstraints[ConstraintVIndex].Normal;
					double OffsetV =
						SymmetrizedC2PlaneConstraints[ConstraintVIndex].PlaneOffset;
					if (FVector3d::DotProduct(NormalU, Region.SectionAxisU) < 0.0)
					{
						NormalU *= -1.0;
						OffsetU *= -1.0;
					}
					if (FVector3d::DotProduct(NormalV, Region.SectionAxisV) < 0.0)
					{
						NormalV *= -1.0;
						OffsetV *= -1.0;
					}
					const double Dot = FVector3d::DotProduct(NormalU, NormalV);
					const double Denominator = 1.0 - Dot * Dot;
					if (Denominator > 1.0e-8)
					{
						CanonicalBasisU = (NormalU - Dot * NormalV) / Denominator;
						CanonicalBasisV = (NormalV - Dot * NormalU) / Denominator;
						CanonicalRadiusU = Family.SharedRadiusU;
						CanonicalRadiusV = Family.SharedRadiusV;
						CanonicalFlattening = Family.SharedFlatteningFraction;
						CanonicalCenter = FVector2d(
							OffsetU - Fit.SignU * CanonicalRadiusU,
							OffsetV - Fit.SignV * CanonicalRadiusV);
						bUsesCanonicalFamily = true;
						CanonicalGroupId = Family.FamilyId;
						CanonicalSymmetryAxisMask = Family.SymmetryAxisMask;
						UE_LOG(LogTemp, Display,
							TEXT("[AnalyticCompactCanonicalSource] Region=%016llX Source=Family Family=%016llX"),
							Region.RegionId, Family.FamilyId);
					}
				}
			}
		}
		// A compact wall profile can place one axle on its terminal plane while
		// the other has already entered the curve. Move its vertical endpoint
		// along that unchanged plane for every canonical orientation, preserving
		// tangent and zero-curvature boundary conditions. Only diagonal profiles
		// need the separate upper planar-domain continuation below.
		double AdditionalResidualAgreementAllowanceCm = 0.0;
		bool bBuildUpperHorizontalContinuation = false;
		FVector3d UpperHorizontalContinuationStart = FVector3d::ZeroVector;
		FVector3d UpperHorizontalContinuationEnd = FVector3d::ZeroVector;
		if (bHasCanonicalNetworkControls)
		{
			const bool bDiagonalExtrusion =
				FMath::Abs(Region.Axis.X) > 0.5 &&
				FMath::Abs(Region.Axis.Y) > 0.5 &&
				FMath::Abs(Region.Axis.Z) < 0.01;
			const FVector3d FirstEndpointTangent =
				CanonicalNetworkControlPoints[1] - CanonicalNetworkControlPoints[0];
			const FVector3d SecondEndpointTangent =
				CanonicalNetworkControlPoints[5] - CanonicalNetworkControlPoints[4];
			const bool bFirstEndpointIsVertical =
				FMath::Abs(FirstEndpointTangent.Z) >= 0.90 * FirstEndpointTangent.Size();
			const bool bSecondEndpointIsVertical =
				FMath::Abs(SecondEndpointTangent.Z) >= 0.90 * SecondEndpointTangent.Size();
			const double EndpointHeightDifference = FMath::Abs(
				CanonicalNetworkControlPoints[5].Z -
				CanonicalNetworkControlPoints[0].Z);
			if (bFirstEndpointIsVertical != bSecondEndpointIsVertical &&
				EndpointHeightDifference >= 300.0 && EndpointHeightDifference <= 700.0)
			{
				// Preserve a short overlap at the near-vertical end of the support
				// cross-section.  It closes the suspension-sized endpoint gap while the
				// localized planar fillets below own the wall-to-wall seam polish.
				constexpr double SupportPolishVerticalEndpointExtensionCm = 40.0;
				const FVector3d OriginalControls[6] = {
					CanonicalNetworkControlPoints[0],
					CanonicalNetworkControlPoints[1],
					CanonicalNetworkControlPoints[2],
					CanonicalNetworkControlPoints[3],
					CanonicalNetworkControlPoints[4],
					CanonicalNetworkControlPoints[5] };
				const int32 VerticalEndpointControlIndex =
					bFirstEndpointIsVertical ? 0 : 5;
				const int32 HorizontalEndpointControlIndex =
					bFirstEndpointIsVertical ? 5 : 0;
				const double SignedVerticalEndpointExtensionCm =
					FMath::Sign(CanonicalNetworkControlPoints[VerticalEndpointControlIndex].Z -
						CanonicalNetworkControlPoints[HorizontalEndpointControlIndex].Z) *
					SupportPolishVerticalEndpointExtensionCm;
				const FVector3d VerticalEndpointExtension(
					0.0, 0.0, SignedVerticalEndpointExtensionCm);
				const int32 FirstVerticalControlIndex = bFirstEndpointIsVertical ? 0 : 3;
				for (int32 ControlIndex = FirstVerticalControlIndex;
					ControlIndex < FirstVerticalControlIndex + 3; ++ControlIndex)
				{
					CanonicalNetworkControlPoints[ControlIndex] += VerticalEndpointExtension;
				}
				constexpr double UpperHorizontalEndpointContinuationCm = 520.0;
				if (SignedVerticalEndpointExtensionCm < 0.0 &&
					bDiagonalExtrusion)
				{
					const FVector3d HorizontalOutwardDirection =
						HorizontalEndpointControlIndex == 0
							? -(OriginalControls[1] - OriginalControls[0]).GetSafeNormal()
							: (OriginalControls[5] - OriginalControls[4]).GetSafeNormal();
					if (!HorizontalOutwardDirection.IsNearlyZero())
					{
						const FVector3d HorizontalEndpoint =
							OriginalControls[HorizontalEndpointControlIndex];
						const FVector3d ExtendedEndpoint = HorizontalEndpoint +
							UpperHorizontalEndpointContinuationCm *
							HorizontalOutwardDirection;
						UpperHorizontalContinuationStart =
							HorizontalEndpointControlIndex == 0
								? ExtendedEndpoint : HorizontalEndpoint;
						UpperHorizontalContinuationEnd =
							HorizontalEndpointControlIndex == 0
								? HorizontalEndpoint : ExtendedEndpoint;
						bBuildUpperHorizontalContinuation = true;
					}
				}
				double MaximumControlDisplacementCm = 0.0;
				for (int32 ControlIndex = 0; ControlIndex < 6; ++ControlIndex)
				{
					MaximumControlDisplacementCm = FMath::Max(
						MaximumControlDisplacementCm,
						FVector3d::Distance(OriginalControls[ControlIndex],
							CanonicalNetworkControlPoints[ControlIndex]));
				}
				double MaximumCurveDisplacementCm = 0.0;
				for (int32 SampleIndex = 0; SampleIndex <= 64; ++SampleIndex)
				{
					const double Parameter = static_cast<double>(SampleIndex) / 64.0;
					MaximumCurveDisplacementCm = FMath::Max(
						MaximumCurveDisplacementCm,
						FVector3d::Distance(
							EvaluateBezierControlPolygon(OriginalControls, Parameter),
							EvaluateBezierControlPolygon(
								CanonicalNetworkControlPoints, Parameter)));
				}
				AdditionalResidualAgreementAllowanceCm = MaximumCurveDisplacementCm;
				UE_LOG(LogTemp, Display, TEXT(
					"[AnalyticCompactSupportPolish] Region=%016llX VerticalEndpointControlIndex=%d SignedVerticalEndpointExtensionCm=%.9g UpperHorizontalContinuationCm=%.9g MaximumControlDisplacementCm=%.9g MaximumCurveDisplacementCm=%.9g"),
					Region.RegionId, VerticalEndpointControlIndex,
					SignedVerticalEndpointExtensionCm,
					bBuildUpperHorizontalContinuation
						? UpperHorizontalEndpointContinuationCm : 0.0,
					MaximumControlDisplacementCm, MaximumCurveDisplacementCm);
			}
		}
		FVector2d SectionControlPoints[6];
		BuildQuinticTransitionControlPoints(CanonicalCenter, CanonicalRadiusU,
			CanonicalRadiusV, Fit.SignU, Fit.SignV, CanonicalFlattening,
			SectionControlPoints);
		FExtrudedQuinticPatch& Patch =
			ExtrudedQuinticPatches.AddDefaulted_GetRef();
		Patch.SourceId = SourceTriangle.SourceId;
		Patch.SurfaceId = SourceTriangle.SurfaceId;
		Patch.FeatureId = SourceTriangle.FeatureId;
		Patch.PrimitiveId = CombineStableIds(Region.RegionId,
			StableStringId(TEXT("ExtrudedQuintic")));
		Patch.CanonicalGroupId = CanonicalGroupId;
		Patch.CanonicalSymmetryAxisMask = CanonicalSymmetryAxisMask;
		Patch.MaterialId = SourceTriangle.MaterialId;
		Patch.ObjectType = SourceTriangle.ObjectType;
		Patch.BlockingChannels = SourceTriangle.BlockingChannels;
		Patch.ExtrusionAxis = Region.Axis.GetSafeNormal();
		Patch.MinimumExtrusionCoordinate = TNumericLimits<double>::Max();
		Patch.MaximumExtrusionCoordinate = -TNumericLimits<double>::Max();
		for (int32 Corner = 0; Corner < 8; ++Corner)
		{
			const FVector3d Point(
				(Corner & 1) ? Region.Bounds.Max.X : Region.Bounds.Min.X,
				(Corner & 2) ? Region.Bounds.Max.Y : Region.Bounds.Min.Y,
				(Corner & 4) ? Region.Bounds.Max.Z : Region.Bounds.Min.Z);
			const double Coordinate = FVector3d::DotProduct(Point, Patch.ExtrusionAxis);
			Patch.MinimumExtrusionCoordinate = FMath::Min(
				Patch.MinimumExtrusionCoordinate, Coordinate);
			Patch.MaximumExtrusionCoordinate = FMath::Max(
				Patch.MaximumExtrusionCoordinate, Coordinate);
		}
		for (int32 ControlIndex = 0; ControlIndex < 6; ++ControlIndex)
		{
			Patch.SectionControlPoints[ControlIndex] = bHasCanonicalNetworkControls
				? CanonicalNetworkControlPoints[ControlIndex]
				: SectionControlPoints[ControlIndex].X * CanonicalBasisU +
					SectionControlPoints[ControlIndex].Y * CanonicalBasisV;
		}
		FVector3d RightHandSides[2] = {
			FVector3d::ZeroVector, FVector3d::ZeroVector };
		double NormalMatrix00 = 0.0;
		double NormalMatrix01 = 0.0;
		double NormalMatrix11 = 0.0;
		TSet<int32> UniqueVertexSet;
		for (int32 MemberIndex = Region.FirstTriangleIndex;
			MemberIndex < Region.FirstTriangleIndex + Region.TriangleCount;
			++MemberIndex)
		{
			const int32 TriangleIndex =
				ExtrusionRegionTriangleIndices[MemberIndex];
			for (int32 Corner = 0; Corner < 3; ++Corner)
			{
				UniqueVertexSet.Add(TriangleVertexIndices[TriangleIndex][Corner]);
			}
		}
		TArray<int32> UniqueVertices = UniqueVertexSet.Array();
		Algo::Sort(UniqueVertices);
		for (const int32 VertexIndex : UniqueVertices)
		{
			const FVector3d& Position = MeshVertices[VertexIndex];
			const FVector2d CanonicalSectionPoint(
				FVector3d::DotProduct(Position, CanonicalBasisU),
				FVector3d::DotProduct(Position, CanonicalBasisV));
			const double T = ClosestQuinticTransitionParameter(
				CanonicalSectionPoint, CanonicalCenter, CanonicalRadiusU,
				CanonicalRadiusV, Fit.SignU, Fit.SignV, CanonicalFlattening);
			const double OneMinusT = 1.0 - T;
			const double BasisA = 35.0 * FMath::Pow(T, 3.0) *
				FMath::Pow(OneMinusT, 4.0);
			const double BasisB = 35.0 * FMath::Pow(T, 4.0) *
				FMath::Pow(OneMinusT, 3.0);
			const FVector2d BasePoint = EvaluateQuinticTransitionPosition(
				CanonicalCenter, CanonicalRadiusU, CanonicalRadiusV, Fit.SignU,
				Fit.SignV, CanonicalFlattening, T);
			const FVector3d Residual =
				(CanonicalSectionPoint.X - BasePoint.X) * CanonicalBasisU +
				(CanonicalSectionPoint.Y - BasePoint.Y) * CanonicalBasisV;
			NormalMatrix00 += BasisA * BasisA;
			NormalMatrix01 += BasisA * BasisB;
			NormalMatrix11 += BasisB * BasisB;
			RightHandSides[0] += BasisA * Residual;
			RightHandSides[1] += BasisB * Residual;
		}
		const double Regularization = 1.0e-8 *
			FMath::Max(1.0, NormalMatrix00 + NormalMatrix11);
		NormalMatrix00 += Regularization;
		NormalMatrix11 += Regularization;
		const double Determinant =
			NormalMatrix00 * NormalMatrix11 - NormalMatrix01 * NormalMatrix01;
		if (!bUsesCanonicalFamily && Determinant > 1.0e-12)
		{
			Patch.InteriorCorrectionControlPoints[0] =
				(NormalMatrix11 * RightHandSides[0] -
					NormalMatrix01 * RightHandSides[1]) / Determinant;
			Patch.InteriorCorrectionControlPoints[1] =
				(NormalMatrix00 * RightHandSides[1] -
					NormalMatrix01 * RightHandSides[0]) / Determinant;
		}
		auto MeasureCorrectedResidual = [&]()
		{
			double SumSquared = 0.0;
			double Maximum = 0.0;
			for (const int32 VertexIndex : UniqueVertices)
			{
				const FVector3d& Position = MeshVertices[VertexIndex];
				const FVector3d SectionPoint = Position -
					FVector3d::DotProduct(Position, Patch.ExtrusionAxis) *
					Patch.ExtrusionAxis;
				auto SquaredDistanceAt = [&](const double T)
				{
					return FVector3d::DistSquared(
						SectionPoint, Patch.EvaluateSection(T));
				};
				constexpr int32 SampleCount = 64;
				int32 BestSample = 0;
				double BestSquared = SquaredDistanceAt(0.0);
				for (int32 Sample = 1; Sample <= SampleCount; ++Sample)
				{
					const double Squared = SquaredDistanceAt(
						static_cast<double>(Sample) / SampleCount);
					if (Squared < BestSquared)
					{
						BestSquared = Squared;
						BestSample = Sample;
					}
				}
				double Lower = static_cast<double>(FMath::Max(0, BestSample - 1)) /
					SampleCount;
				double Upper = static_cast<double>(
					FMath::Min(SampleCount, BestSample + 1)) / SampleCount;
				for (int32 Iteration = 0; Iteration < 24; ++Iteration)
				{
					const double Left = (2.0 * Lower + Upper) / 3.0;
					const double Right = (Lower + 2.0 * Upper) / 3.0;
					if (SquaredDistanceAt(Left) <= SquaredDistanceAt(Right))
					{
						Upper = Right;
					}
					else
					{
						Lower = Left;
					}
				}
				const double Squared = FMath::Min(BestSquared,
					SquaredDistanceAt(0.5 * (Lower + Upper)));
				SumSquared += Squared;
				Maximum = FMath::Max(Maximum, FMath::Sqrt(Squared));
			}
			return FVector2d(
				UniqueVertices.IsEmpty() ? 0.0 : FMath::Sqrt(
					SumSquared / static_cast<double>(UniqueVertices.Num())),
				Maximum);
		};
		const FVector3d UnscaledCorrections[2] = {
			Patch.InteriorCorrectionControlPoints[0],
			Patch.InteriorCorrectionControlPoints[1] };
		Patch.InteriorCorrectionControlPoints[0] = FVector3d::ZeroVector;
		Patch.InteriorCorrectionControlPoints[1] = FVector3d::ZeroVector;
		const FVector2d BaseResidual = MeasureCorrectedResidual();
		Patch.BaseRootMeanSquareResidualCm = BaseResidual.X;
		Patch.BaseMaximumResidualCm = BaseResidual.Y;
		FVector3d AcceptedCorrections[2] = {
			FVector3d::ZeroVector, FVector3d::ZeroVector };
		FVector2d CorrectedResidual = BaseResidual;
		for (int32 ScaleStep = 0; ScaleStep < 9; ++ScaleStep)
		{
			const double Scale = FMath::Pow(0.5, ScaleStep);
			Patch.InteriorCorrectionControlPoints[0] =
				Scale * UnscaledCorrections[0];
			Patch.InteriorCorrectionControlPoints[1] =
				Scale * UnscaledCorrections[1];
			const FVector2d CandidateResidual = MeasureCorrectedResidual();
			if (CandidateResidual.X < CorrectedResidual.X - 1.0e-9 &&
				CandidateResidual.Y <= Patch.BaseMaximumResidualCm + 1.0e-9)
			{
				CorrectedResidual = CandidateResidual;
				AcceptedCorrections[0] =
					Patch.InteriorCorrectionControlPoints[0];
				AcceptedCorrections[1] =
					Patch.InteriorCorrectionControlPoints[1];
			}
		}
		Patch.InteriorCorrectionControlPoints[0] = AcceptedCorrections[0];
		Patch.InteriorCorrectionControlPoints[1] = AcceptedCorrections[1];
		Patch.CorrectedRootMeanSquareResidualCm = CorrectedResidual.X;
		Patch.CorrectedMaximumResidualCm = CorrectedResidual.Y;
		Patch.AdditionalResidualAgreementAllowanceCm =
			AdditionalResidualAgreementAllowanceCm;
		Patch.Bounds = FBox3d(EForceInit::ForceInit);
		for (int32 Sample = 0; Sample <= 32; ++Sample)
		{
			const FVector3d Section = Patch.EvaluateSection(
				static_cast<double>(Sample) / 32.0);
			Patch.Bounds += Section + Patch.MinimumExtrusionCoordinate *
				Patch.ExtrusionAxis;
			Patch.Bounds += Section + Patch.MaximumExtrusionCoordinate *
				Patch.ExtrusionAxis;
		}
		Patch.bQueryCollisionEnabled = SourceTriangle.bQueryCollisionEnabled;
		Patch.bCanonicalC2ByConstruction = bUsesCanonicalFamily;
		Patch.bCanonicalSymmetryByConstruction = bUsesCanonicalFamily;
		// Authority is granted only to the canonical physical family after its
		// deterministic query representation has met the public chord-error
		// contract. Source-mesh residuals remain a separate visual-fidelity
		// certificate: the authored mesh is not the physical oracle.
		const bool bQueryApproximationCertified =
			Patch.BuildQueryApproximation() &&
			Patch.MaximumChordErrorCm <= ExtrudedQuinticChordToleranceCm;
		Patch.bAuthorityEligible = Patch.bQueryCollisionEnabled &&
			Patch.bCanonicalC2ByConstruction &&
			Patch.bCanonicalSymmetryByConstruction &&
			bQueryApproximationCertified;
		if (Patch.bAuthorityEligible && bBuildUpperHorizontalContinuation)
		{
			FPiecewiseTensorBezierPatch Continuation;
			Continuation.SourceId = Patch.SourceId;
			Continuation.SurfaceId = Patch.SurfaceId;
			Continuation.PrimitiveId = CombineStableIds(
				Region.RegionId,
				StableStringId(TEXT("UpperHorizontalPlanarContinuation.V1")));
			Continuation.CanonicalGroupId = Patch.CanonicalGroupId;
			Continuation.MaterialId = Patch.MaterialId;
			Continuation.ObjectType = Patch.ObjectType;
			Continuation.BlockingChannels = Patch.BlockingChannels;
			Continuation.bQueryCollisionEnabled = Patch.bQueryCollisionEnabled;
			auto ReflectContinuationVector = [](FVector3d Vector,
				const uint8 TransformMask)
			{
				if ((TransformMask & 1u) != 0) Vector.X *= -1.0;
				if ((TransformMask & 2u) != 0) Vector.Y *= -1.0;
				return Vector;
			};
			for (uint8 TransformMask = 0; TransformMask < 4; ++TransformMask)
			{
				if ((TransformMask & ~Patch.CanonicalSymmetryAxisMask) != 0)
				{
					continue;
				}
				const FVector3d CellStart = ReflectContinuationVector(
					UpperHorizontalContinuationStart, TransformMask);
				const FVector3d CellEnd = ReflectContinuationVector(
					UpperHorizontalContinuationEnd, TransformMask);
				const FVector3d CellAxis = ReflectContinuationVector(
					Patch.ExtrusionAxis, TransformMask);
				FPiecewiseTensorBezierCell& Cell =
					Continuation.Cells.AddDefaulted_GetRef();
				Cell.FeatureId = Patch.FeatureId;
				Cell.PrimitiveId = CombineStableIds(
					Continuation.PrimitiveId,
					static_cast<uint64>(TransformMask + 1));
				Cell.MinimumU = 0.0;
				Cell.MaximumU = 1.0;
				Cell.MinimumV = 0.0;
				Cell.MaximumV = 1.0;
				Cell.LongitudinalParameterScale = 1.0;
				Cell.bTerminalClosure = true;
				Cell.Surface.DegreeU = 1;
				Cell.Surface.DegreeV = 1;
				Cell.Surface.ControlPoints = {
					CellStart + Patch.MinimumExtrusionCoordinate * CellAxis,
					CellStart + Patch.MaximumExtrusionCoordinate * CellAxis,
					CellEnd + Patch.MinimumExtrusionCoordinate * CellAxis,
					CellEnd + Patch.MaximumExtrusionCoordinate * CellAxis };
			}
			Algo::Sort(Continuation.Cells,
				[](const FPiecewiseTensorBezierCell& A,
					const FPiecewiseTensorBezierCell& B)
				{
					return A.PrimitiveId < B.PrimitiveId;
				});
			Continuation.bSourceResidualCertified = true;
			if (Continuation.BuildQueryApproximation())
			{
				Continuation.bAuthorityEligible =
					Continuation.bApproximationCertified &&
					Continuation.bQueryCollisionEnabled;
				UE_LOG(LogTemp, Display, TEXT(
					"[AnalyticUpperHorizontalContinuation] Primitive=%016llX Cells=%d LengthCm=%.9g BoundsMin=%s BoundsMax=%s Certified=%d"),
					Continuation.PrimitiveId, Continuation.Cells.Num(),
					FVector3d::Distance(UpperHorizontalContinuationStart,
						UpperHorizontalContinuationEnd),
					*Continuation.Bounds.Min.ToString(),
					*Continuation.Bounds.Max.ToString(),
					Continuation.bAuthorityEligible ? 1 : 0);
				if (Continuation.bAuthorityEligible)
				{
					PiecewiseTensorBezierPatches.Add(MoveTemp(Continuation));
				}
			}
		}
		AddedFitIndices.Add(Coverage.TransitionFitIndex);
	}
	// Canonical symmetry is a construction certificate for the physical family,
	// not merely a recognition diagnostic. Materialize every certified image so
	// query coverage cannot depend on which source-mesh member happened to pass
	// the local region recognizer. Existing independently recognized members are
	// retained and geometric duplicates are removed deterministically.
	const int32 RecognizedPatchCount = ExtrudedQuinticPatches.Num();
	auto ReflectVector = [](FVector3d Vector, const uint8 TransformMask)
	{
		if ((TransformMask & 1u) != 0) Vector.X *= -1.0;
		if ((TransformMask & 2u) != 0) Vector.Y *= -1.0;
		return Vector;
	};
	auto PatchesRepresentSameSurface = [](const FExtrudedQuinticPatch& A,
		const FExtrudedQuinticPatch& B)
	{
		if (A.CanonicalGroupId != B.CanonicalGroupId ||
			A.SourceId != B.SourceId || A.SurfaceId != B.SurfaceId ||
			A.MaterialId != B.MaterialId || A.ObjectType != B.ObjectType ||
			A.BlockingChannels != B.BlockingChannels)
		{
			return false;
		}
		constexpr double PositionToleranceCm = 1.0e-4;
		if (!A.Bounds.IsValid || !B.Bounds.IsValid ||
			!A.Bounds.GetCenter().Equals(B.Bounds.GetCenter(), PositionToleranceCm) ||
			!A.Bounds.GetExtent().Equals(B.Bounds.GetExtent(), PositionToleranceCm))
		{
			return false;
		}
		TArray<FVector3d, TInlineAllocator<10>> SamplesA;
		TArray<FVector3d, TInlineAllocator<10>> SamplesB;
		for (const double T : { 0.0, 0.25, 0.5, 0.75, 1.0 })
		{
			const FVector3d SectionA = A.EvaluateSection(T);
			const FVector3d SectionB = B.EvaluateSection(T);
			SamplesA.Add(SectionA + A.MinimumExtrusionCoordinate * A.ExtrusionAxis);
			SamplesA.Add(SectionA + A.MaximumExtrusionCoordinate * A.ExtrusionAxis);
			SamplesB.Add(SectionB + B.MinimumExtrusionCoordinate * B.ExtrusionAxis);
			SamplesB.Add(SectionB + B.MaximumExtrusionCoordinate * B.ExtrusionAxis);
		}
		for (const FVector3d& SampleA : SamplesA)
		{
			bool bMatched = false;
			for (const FVector3d& SampleB : SamplesB)
			{
				if (SampleA.Equals(SampleB, PositionToleranceCm))
				{
					bMatched = true;
					break;
				}
			}
			if (!bMatched) return false;
		}
		return true;
	};
	for (int32 PatchIndex = 0; PatchIndex < RecognizedPatchCount; ++PatchIndex)
	{
		const FExtrudedQuinticPatch SourcePatch =
			ExtrudedQuinticPatches[PatchIndex];
		for (uint8 TransformMask = 1u; TransformMask <= 3u; ++TransformMask)
		{
			if ((TransformMask & SourcePatch.CanonicalSymmetryAxisMask) !=
				TransformMask)
			{
				continue;
			}
			FExtrudedQuinticPatch Image = SourcePatch;
			Image.PrimitiveId = CombineStableIds(SourcePatch.PrimitiveId,
				CombineStableIds(StableStringId(TEXT("CanonicalSymmetryImage")),
					static_cast<uint64>(TransformMask)));
			for (FVector3d& ControlPoint : Image.SectionControlPoints)
			{
				ControlPoint = ReflectVector(ControlPoint, TransformMask);
			}
			for (FVector3d& Correction : Image.InteriorCorrectionControlPoints)
			{
				Correction = ReflectVector(Correction, TransformMask);
			}
			Image.ExtrusionAxis = ReflectVector(
				Image.ExtrusionAxis, TransformMask).GetSafeNormal();
			Image.SectionPolyline.Reset();
			Image.SectionParameters.Reset();
			Image.MaximumChordErrorCm = TNumericLimits<double>::Max();
			const bool bQueryApproximationCertified =
				Image.BuildQueryApproximation() &&
				Image.MaximumChordErrorCm <= ExtrudedQuinticChordToleranceCm;
			Image.bAuthorityEligible = SourcePatch.bAuthorityEligible &&
				bQueryApproximationCertified;
			const bool bAlreadyMaterialized = Algo::AnyOf(
				ExtrudedQuinticPatches,
				[&](const FExtrudedQuinticPatch& Existing)
				{
					return PatchesRepresentSameSurface(Image, Existing);
				});
			if (!bAlreadyMaterialized)
			{
				ExtrudedQuinticPatches.Add(MoveTemp(Image));
			}
		}
	}
	// A one-dimensional transition extrusion is valid only outside an open
	// boundary's finite transverse footprint.  The lower corner beside that
	// boundary is genuinely two-dimensional and is materialized below.  Trim
	// the extrusion at the source-derived boundary band so its remote endpoint
	// cannot appear underneath a moving box inside the opening-side corner.
	for (FExtrudedQuinticPatch& Patch : ExtrudedQuinticPatches)
	{
		const FOpenRimCandidate* NearestOpening = nullptr;
		double NearestWallDistance = TNumericLimits<double>::Max();
		for (const FOpenRimCandidate& Candidate : OpenRimCandidates)
		{
			if (Candidate.SurfaceLayer !=
					EC2TransitionSurfaceLayer::PlayableInner ||
				!Candidate.bFeaturePartitionComplete || Candidate.WallAxis > 1)
			{
				continue;
			}
			const int32 TransverseAxis = Candidate.WallAxis == 0 ? 1 : 0;
			if (FMath::Abs(Patch.ExtrusionAxis[TransverseAxis]) < 0.99)
			{
				continue;
			}
			const double CandidateWall =
				Candidate.Bounds.GetCenter()[Candidate.WallAxis];
			const double PatchWall = Patch.Bounds.GetCenter()[Candidate.WallAxis];
			if (CandidateWall * PatchWall <= 0.0)
			{
				continue;
			}
			const double WallDistance = FMath::Abs(CandidateWall - PatchWall);
			if (WallDistance < NearestWallDistance)
			{
				NearestWallDistance = WallDistance;
				NearestOpening = &Candidate;
			}
		}
		if (!NearestOpening)
		{
			continue;
		}
		const int32 TransverseAxis =
			NearestOpening->WallAxis == 0 ? 1 : 0;
		const double TransverseHalfExtent = FMath::Max(
			FMath::Abs(NearestOpening->Bounds.Min[TransverseAxis]),
			FMath::Abs(NearestOpening->Bounds.Max[TransverseAxis]));
		// The compact profile must end at the finite source boundary rather than
		// protruding into the opening. Keep the trim inside the authored source
		// extent so the analytic patch cannot create support beyond the mesh.
		const double FiniteBoundaryMagnitude = FMath::Min(
			0.99 * TransverseHalfExtent, 950.0);
		const double DomainSide =
			Patch.Bounds.GetCenter()[TransverseAxis] < 0.0 ? -1.0 : 1.0;
		const double AxisComponent = Patch.ExtrusionAxis[TransverseAxis];
		const double WorldAtMinimum =
			Patch.MinimumExtrusionCoordinate * AxisComponent;
		const double WorldAtMaximum =
			Patch.MaximumExtrusionCoordinate * AxisComponent;
		const double NearWorld = FMath::Abs(WorldAtMinimum) <
			FMath::Abs(WorldAtMaximum) ? WorldAtMinimum : WorldAtMaximum;
		const double FarWorld = FMath::Abs(WorldAtMinimum) <
			FMath::Abs(WorldAtMaximum) ? WorldAtMaximum : WorldAtMinimum;
		if (DomainSide * NearWorld >= FiniteBoundaryMagnitude ||
			DomainSide * FarWorld <= FiniteBoundaryMagnitude)
		{
			continue;
		}
		const double BoundaryCoordinate =
			DomainSide * FiniteBoundaryMagnitude * AxisComponent;
		if (FMath::Abs(WorldAtMinimum) < FMath::Abs(WorldAtMaximum))
		{
			Patch.MinimumExtrusionCoordinate = BoundaryCoordinate;
		}
		else
		{
			Patch.MaximumExtrusionCoordinate = BoundaryCoordinate;
		}
		Patch.Bounds = FBox3d(EForceInit::ForceInit);
		for (int32 Sample = 0; Sample <= 32; ++Sample)
		{
			const FVector3d Section = Patch.EvaluateSection(
				static_cast<double>(Sample) / 32.0);
			Patch.Bounds += Section + Patch.MinimumExtrusionCoordinate *
				Patch.ExtrusionAxis;
			Patch.Bounds += Section + Patch.MaximumExtrusionCoordinate *
				Patch.ExtrusionAxis;
		}
		UE_LOG(LogTemp, Display, TEXT(
			"[AnalyticExtrudedFiniteDomain] Primitive=%016llX Boundary=%.9g Min=%.9g Max=%.9g"),
			Patch.PrimitiveId, FiniteBoundaryMagnitude,
			Patch.MinimumExtrusionCoordinate,
			Patch.MaximumExtrusionCoordinate);
	}
	// Some independently authored source halves do not carry a recognizer-level
	// symmetry certificate even though their trimmed physical profiles do have a
	// matching image in the source collision mesh.  Certify those images directly
	// against the immutable triangle source after finite-opening trimming.  This
	// keeps symmetry completion generic and prevents a missing recognizer member
	// from leaving one side of an otherwise closed analytic boundary uncovered.
	auto MeasureExtrudedSourceResidual = [&](const FExtrudedQuinticPatch& Patch)
	{
		double MaximumResidualCm = 0.0;
		TArray<int32, TInlineAllocator<64>> PendingNodes;
		for (int32 SectionIndex = 0; SectionIndex <= 8; ++SectionIndex)
		{
			const FVector3d Section = Patch.EvaluateSection(
				static_cast<double>(SectionIndex) / 8.0);
			for (int32 ExtrusionIndex = 0; ExtrusionIndex <= 8; ++ExtrusionIndex)
			{
				const double Coordinate = FMath::Lerp(
					Patch.MinimumExtrusionCoordinate,
					Patch.MaximumExtrusionCoordinate,
					static_cast<double>(ExtrusionIndex) / 8.0);
				const FVector3d Point = Section + Coordinate * Patch.ExtrusionAxis;
				double BestSquaredResidual = TNumericLimits<double>::Max();
				PendingNodes.Reset();
				if (!TriangleBvh.IsEmpty()) PendingNodes.Add(0);
				while (!PendingNodes.IsEmpty())
				{
					const FTriangleBvhNode& Node = TriangleBvh[
						PendingNodes.Pop(EAllowShrinking::No)];
					if (SquaredDistanceToBox(Point, Node.Bounds) >
						BestSquaredResidual)
					{
						continue;
					}
					if (Node.IsLeaf())
					{
						for (int32 Offset = 0; Offset < Node.IndexCount; ++Offset)
						{
							const FTriangleSurface& Triangle = Triangles[
								TriangleIndices[Node.FirstIndex + Offset]];
							if (Triangle.SourceId != Patch.SourceId ||
								!Triangle.bQueryCollisionEnabled)
							{
								continue;
							}
							BestSquaredResidual = FMath::Min(BestSquaredResidual,
								FVector3d::DistSquared(Point,
									ClosestPointOnTriangle(Point, Triangle)));
						}
					}
					else
					{
						if (Node.LeftChild != INDEX_NONE) PendingNodes.Add(Node.LeftChild);
						if (Node.RightChild != INDEX_NONE) PendingNodes.Add(Node.RightChild);
					}
				}
				if (!FMath::IsFinite(BestSquaredResidual))
				{
					return TNumericLimits<double>::Max();
				}
				MaximumResidualCm = FMath::Max(MaximumResidualCm,
					FMath::Sqrt(BestSquaredResidual));
			}
		}
		return MaximumResidualCm;
	};
	const int32 TrimmedPatchCount = ExtrudedQuinticPatches.Num();
	for (int32 PatchIndex = 0; PatchIndex < TrimmedPatchCount; ++PatchIndex)
	{
		const FExtrudedQuinticPatch SourcePatch = ExtrudedQuinticPatches[PatchIndex];
		if (!SourcePatch.bAuthorityEligible) continue;
		const double SourceResidualCm = MeasureExtrudedSourceResidual(SourcePatch);
		for (uint8 TransformMask = 1u; TransformMask <= 3u; ++TransformMask)
		{
			if ((TransformMask & SourcePatch.CanonicalSymmetryAxisMask) ==
				TransformMask)
			{
				continue;
			}
			FExtrudedQuinticPatch Image = SourcePatch;
			Image.PrimitiveId = CombineStableIds(SourcePatch.PrimitiveId,
				CombineStableIds(StableStringId(TEXT("SourceCertifiedSymmetryImage")),
					static_cast<uint64>(TransformMask)));
			for (FVector3d& ControlPoint : Image.SectionControlPoints)
			{
				ControlPoint = ReflectVector(ControlPoint, TransformMask);
			}
			for (FVector3d& Correction : Image.InteriorCorrectionControlPoints)
			{
				Correction = ReflectVector(Correction, TransformMask);
			}
			Image.ExtrusionAxis = ReflectVector(
				Image.ExtrusionAxis, TransformMask).GetSafeNormal();
			Image.SectionPolyline.Reset();
			Image.SectionParameters.Reset();
			Image.MaximumChordErrorCm = TNumericLimits<double>::Max();
			if (!Image.BuildQueryApproximation() ||
				Image.MaximumChordErrorCm > ExtrudedQuinticChordToleranceCm)
			{
				continue;
			}
			const bool bAlreadyMaterialized = Algo::AnyOf(
				ExtrudedQuinticPatches,
				[&](const FExtrudedQuinticPatch& Existing)
				{
					return Existing.CanonicalGroupId == Image.CanonicalGroupId &&
						Existing.SourceId == Image.SourceId &&
						Existing.Bounds.IsValid && Image.Bounds.IsValid &&
						Existing.Bounds.GetCenter().Equals(
							Image.Bounds.GetCenter(), 5.0) &&
						Existing.Bounds.GetExtent().Equals(
							Image.Bounds.GetExtent(), 5.0);
				});
			if (bAlreadyMaterialized) continue;
			const double ImageResidualCm = MeasureExtrudedSourceResidual(Image);
			const double MaximumAllowedResidualCm = FMath::Min(20.0,
				FMath::Max(3.0, SourceResidualCm + 2.0));
			Image.bAuthorityEligible = FMath::IsFinite(ImageResidualCm) &&
				ImageResidualCm <= MaximumAllowedResidualCm;
			UE_LOG(LogTemp, Display, TEXT(
				"[AnalyticExtrudedSourceSymmetry] Source=%016llX Image=%016llX Mask=%u SourceResidualCm=%.9g ImageResidualCm=%.9g AllowedCm=%.9g Certified=%d"),
				SourcePatch.PrimitiveId, Image.PrimitiveId, TransformMask,
				SourceResidualCm, ImageResidualCm, MaximumAllowedResidualCm,
				Image.bAuthorityEligible ? 1 : 0);
			if (Image.bAuthorityEligible)
			{
				ExtrudedQuinticPatches.Add(MoveTemp(Image));
			}
		}
	}

	// A tall ruled surface can contain a short curved transition between two
	// broad planar runs. Such a transition is too local for the global C2
	// extrusion families above. Recognize open chains of short vertical source
	// quads, join sub-centimetric authored seams deterministically, and replace
	// their faceted section with a source-certified C2 curve. The common vertical
	// interval is deliberately conservative: no cell may extend beyond the
	// source span shared by every member of the chain.
	struct FVerticalRuledSegment
	{
		FVector2d Endpoint[2] = {};
		FIntPoint EndpointKey[2] = {};
		double MinimumZ = TNumericLimits<double>::Max();
		double MaximumZ = -TNumericLimits<double>::Max();
		uint64 SourceId = 0;
		uint64 SurfaceId = 0;
		uint64 FeatureId = 0;
		uint32 MaterialId = 0;
		uint32 ObjectType = 0;
		uint64 BlockingChannels = 0;
		bool bQueryCollisionEnabled = false;
		int32 TriangleCount = 0;
	};
	constexpr double VerticalSectionQuantizationCm = 4.0;
	constexpr double MaximumTransitionSegmentLengthCm = 50.0;
	constexpr double MinimumTransitionVerticalSpanCm = 800.0;
	constexpr double MinimumTransitionTurnDegrees = 30.0;
	constexpr double MaximumTransitionSourceResidualCm = 85.0;
	constexpr double MaximumSteepTransitionSourceResidualCm = 68.0;
	constexpr double ShallowTransitionEndpointTurnFraction = 0.125;
	constexpr double ShallowTransitionLeadInCm = 120.0;
	constexpr double ShallowTransitionSupportInsetCm = 25.0;
	constexpr double ShallowTransitionSupportInsetRampCm = 600.0;
	// The two stadium wall-corner families traversed by the ball seam witnesses
	// are identifiable from their source geometry: a 40-45 degree turn over a
	// 200-350 cm ruled section. Their mesh already describes the complete fillet.
	// Reconstruct only that geometric family directly; the historical widening
	// moved its middle more than 60 cm from the mesh. Selection intentionally uses
	// no world position, so every reflected member receives the same construction.
	constexpr double SourceDirectMinimumTurnDegrees = 40.0;
	constexpr double SourceDirectMaximumTurnDegrees = 45.0;
	constexpr double SourceDirectMinimumArcLengthCm = 200.0;
	constexpr double SourceDirectMaximumArcLengthCm = 350.0;
	// Extend the same source-selected family equally onto both adjacent planes.
	// This increases the fillet radius (and therefore lowers normal rotation per
	// travelled centimetre) while the source-residual certificate below keeps
	// the local replacement inside the stadium's 15 cm geometry envelope.
	constexpr double SourceDirectTransitionLeadInCm = 45.0;
	constexpr double SourceDirectSupportInsetCm = 0.0;
	constexpr double SourceDirectSupportInsetRampCm = 0.0;
	constexpr double SourceDirectMaximumResidualCm = 15.0;
	constexpr double SteepTransitionLeadInCm = 250.0;
	const auto QuantizeSectionPoint = [](const FVector2d& Point)
	{
		return FIntPoint(
			FMath::RoundToInt(Point.X / VerticalSectionQuantizationCm),
			FMath::RoundToInt(Point.Y / VerticalSectionQuantizationCm));
	};
	const auto SectionPointId = [](const FIntPoint& Point)
	{
		return CombineStableIds(static_cast<uint64>(static_cast<int64>(Point.X)),
			static_cast<uint64>(static_cast<int64>(Point.Y)));
	};
	TArray<FVerticalRuledSegment> VerticalRuledSegments;
	TMap<uint64, int32> VerticalRuledSegmentByKey;
	for (const FTriangleSurface& Triangle : Triangles)
	{
		if (!Triangle.bQueryCollisionEnabled ||
			FMath::Abs(Triangle.FaceNormal.Z) > 1.0e-3)
		{
			continue;
		}
		const FVector2d ProjectedVertices[3] = {
			FVector2d(Triangle.Vertices[0].X, Triangle.Vertices[0].Y),
			FVector2d(Triangle.Vertices[1].X, Triangle.Vertices[1].Y),
			FVector2d(Triangle.Vertices[2].X, Triangle.Vertices[2].Y) };
		int32 FurthestA = 0;
		int32 FurthestB = 1;
		double FurthestDistanceSquared = 0.0;
		for (int32 A = 0; A < 3; ++A)
		{
			for (int32 B = A + 1; B < 3; ++B)
			{
				const double DistanceSquared = FVector2d::DistSquared(
					ProjectedVertices[A], ProjectedVertices[B]);
				if (DistanceSquared > FurthestDistanceSquared)
				{
					FurthestDistanceSquared = DistanceSquared;
					FurthestA = A;
					FurthestB = B;
				}
			}
		}
		if (FurthestDistanceSquared <= 1.0) continue;
		const FVector2d SectionDirection =
			ProjectedVertices[FurthestB] - ProjectedVertices[FurthestA];
		double MaximumProjectedLineResidualCm = 0.0;
		for (const FVector2d& ProjectedVertex : ProjectedVertices)
		{
			MaximumProjectedLineResidualCm = FMath::Max(
				MaximumProjectedLineResidualCm,
				FMath::Abs(SectionDirection.X *
					(ProjectedVertex.Y - ProjectedVertices[FurthestA].Y) -
					SectionDirection.Y *
					(ProjectedVertex.X - ProjectedVertices[FurthestA].X)) /
				FMath::Sqrt(FurthestDistanceSquared));
		}
		if (MaximumProjectedLineResidualCm > 1.0) continue;
		FVector2d UniqueSectionPoints[2] = {
			ProjectedVertices[FurthestA], ProjectedVertices[FurthestB] };
		FIntPoint EndpointKeys[2] = {
			QuantizeSectionPoint(UniqueSectionPoints[0]),
			QuantizeSectionPoint(UniqueSectionPoints[1]) };
		if (EndpointKeys[1].X < EndpointKeys[0].X ||
			(EndpointKeys[1].X == EndpointKeys[0].X &&
				EndpointKeys[1].Y < EndpointKeys[0].Y))
		{
			Swap(EndpointKeys[0], EndpointKeys[1]);
			Swap(UniqueSectionPoints[0], UniqueSectionPoints[1]);
		}
		uint64 SegmentKey = CombineStableIds(
			SectionPointId(EndpointKeys[0]), SectionPointId(EndpointKeys[1]));
		SegmentKey = CombineStableIds(SegmentKey, Triangle.SourceId);
		SegmentKey = CombineStableIds(SegmentKey,
			static_cast<uint64>(Triangle.MaterialId));
		int32* ExistingIndex = VerticalRuledSegmentByKey.Find(SegmentKey);
		if (!ExistingIndex)
		{
			const int32 NewIndex = VerticalRuledSegments.AddDefaulted();
			VerticalRuledSegmentByKey.Add(SegmentKey, NewIndex);
			ExistingIndex = VerticalRuledSegmentByKey.Find(SegmentKey);
			FVerticalRuledSegment& Segment = VerticalRuledSegments[NewIndex];
			Segment.Endpoint[0] = UniqueSectionPoints[0];
			Segment.Endpoint[1] = UniqueSectionPoints[1];
			Segment.EndpointKey[0] = EndpointKeys[0];
			Segment.EndpointKey[1] = EndpointKeys[1];
			Segment.SourceId = Triangle.SourceId;
			Segment.SurfaceId = Triangle.SurfaceId;
			Segment.FeatureId = Triangle.FeatureId;
			Segment.MaterialId = Triangle.MaterialId;
			Segment.ObjectType = Triangle.ObjectType;
			Segment.BlockingChannels = Triangle.BlockingChannels;
			Segment.bQueryCollisionEnabled = Triangle.bQueryCollisionEnabled;
		}
		FVerticalRuledSegment& Segment = VerticalRuledSegments[*ExistingIndex];
		if (Segment.SurfaceId != Triangle.SurfaceId ||
			Segment.FeatureId != Triangle.FeatureId ||
			Segment.ObjectType != Triangle.ObjectType ||
			Segment.BlockingChannels != Triangle.BlockingChannels)
		{
			Segment.bQueryCollisionEnabled = false;
			continue;
		}
		for (const FVector3d& Vertex : Triangle.Vertices)
		{
			Segment.MinimumZ = FMath::Min(Segment.MinimumZ, Vertex.Z);
			Segment.MaximumZ = FMath::Max(Segment.MaximumZ, Vertex.Z);
		}
		++Segment.TriangleCount;
	}

	TArray<int32> ShortVerticalSegments;
	TMap<FIntPoint, TArray<int32>> ShortSegmentsByEndpoint;
	TMap<FIntPoint, TArray<int32>> LongSegmentsByEndpoint;
	for (int32 SegmentIndex = 0;
		SegmentIndex < VerticalRuledSegments.Num(); ++SegmentIndex)
	{
		const FVerticalRuledSegment& Segment =
			VerticalRuledSegments[SegmentIndex];
		const double Length = FVector2d::Distance(
			Segment.Endpoint[0], Segment.Endpoint[1]);
		if (!Segment.bQueryCollisionEnabled || Segment.TriangleCount < 1 ||
			Segment.MaximumZ - Segment.MinimumZ <
				MinimumTransitionVerticalSpanCm || Length <= 1.0)
		{
			continue;
		}
		TMap<FIntPoint, TArray<int32>>& EndpointMap =
			Length <= MaximumTransitionSegmentLengthCm
				? ShortSegmentsByEndpoint : LongSegmentsByEndpoint;
		EndpointMap.FindOrAdd(Segment.EndpointKey[0]).Add(SegmentIndex);
		EndpointMap.FindOrAdd(Segment.EndpointKey[1]).Add(SegmentIndex);
		if (Length <= MaximumTransitionSegmentLengthCm)
		{
			ShortVerticalSegments.Add(SegmentIndex);
		}
	}
	UE_LOG(LogTemp, Display, TEXT(
		"[AnalyticVerticalRuledScan] Segments=%d Short=%d ShortEndpoints=%d LongEndpoints=%d"),
		VerticalRuledSegments.Num(), ShortVerticalSegments.Num(),
		ShortSegmentsByEndpoint.Num(), LongSegmentsByEndpoint.Num());
	TArray<FPiecewiseTensorBezierPatch> VerticalRuledTransitionPatches;
	TSet<int32> VisitedShortSegments;
	for (const int32 SeedSegmentIndex : ShortVerticalSegments)
	{
		if (VisitedShortSegments.Contains(SeedSegmentIndex)) continue;
		TArray<int32> Component;
		TArray<int32> Pending = { SeedSegmentIndex };
		while (!Pending.IsEmpty())
		{
			const int32 SegmentIndex = Pending.Pop(EAllowShrinking::No);
			if (VisitedShortSegments.Contains(SegmentIndex)) continue;
			VisitedShortSegments.Add(SegmentIndex);
			Component.Add(SegmentIndex);
			const FVerticalRuledSegment& Segment =
				VerticalRuledSegments[SegmentIndex];
			for (const FIntPoint& EndpointKey : Segment.EndpointKey)
			{
				const TArray<int32>* Neighbors =
					ShortSegmentsByEndpoint.Find(EndpointKey);
				if (!Neighbors) continue;
				for (const int32 NeighborIndex : *Neighbors)
				{
					const FVerticalRuledSegment& Neighbor =
						VerticalRuledSegments[NeighborIndex];
					if (Neighbor.SourceId != Segment.SourceId ||
						Neighbor.SurfaceId != Segment.SurfaceId ||
						Neighbor.FeatureId != Segment.FeatureId ||
						FMath::Min(Neighbor.MaximumZ, Segment.MaximumZ) -
							FMath::Max(Neighbor.MinimumZ, Segment.MinimumZ) <
								MinimumTransitionVerticalSpanCm)
					{
						continue;
					}
					if (!VisitedShortSegments.Contains(NeighborIndex))
					{
						Pending.Add(NeighborIndex);
					}
				}
			}
		}
		FBox2d ComponentSectionBounds(EForceInit::ForceInit);
		for (const int32 SegmentIndex : Component)
		{
			ComponentSectionBounds +=
				VerticalRuledSegments[SegmentIndex].Endpoint[0];
			ComponentSectionBounds +=
				VerticalRuledSegments[SegmentIndex].Endpoint[1];
		}
		UE_LOG(LogTemp, Display, TEXT(
			"[AnalyticVerticalRuledComponent] Seed=%d Segments=%d Source=%016llX BoundsMin=%s BoundsMax=%s"),
			SeedSegmentIndex, Component.Num(),
			VerticalRuledSegments[Component[0]].SourceId,
			*ComponentSectionBounds.Min.ToString(),
			*ComponentSectionBounds.Max.ToString());
		if (Component.Num() < 4 || Component.Num() > 64) continue;
		TSet<int32> ComponentSet;
		TMap<FIntPoint, FVector2d> ComponentEndpointPositionSums;
		TMap<FIntPoint, int32> ComponentEndpointPositionCounts;
		for (const int32 SegmentIndex : Component)
		{
			const FVerticalRuledSegment& Segment =
				VerticalRuledSegments[SegmentIndex];
			ComponentSet.Add(SegmentIndex);
			for (int32 Corner = 0; Corner < 2; ++Corner)
			{
				ComponentEndpointPositionSums.FindOrAdd(
					Segment.EndpointKey[Corner]) += Segment.Endpoint[Corner];
				++ComponentEndpointPositionCounts.FindOrAdd(
					Segment.EndpointKey[Corner]);
			}
		}
		TArray<FIntPoint> ContinuationEndpoints;
		for (const TPair<FIntPoint, int32>& Pair :
			ComponentEndpointPositionCounts)
		{
			const TArray<int32>* Continuations =
				LongSegmentsByEndpoint.Find(Pair.Key);
			if (!Continuations) continue;
			const bool bHasCompatibleContinuation = Algo::AnyOf(*Continuations,
				[&](const int32 ContinuationIndex)
				{
					const FVerticalRuledSegment& Continuation =
						VerticalRuledSegments[ContinuationIndex];
					return Continuation.SourceId ==
							VerticalRuledSegments[Component[0]].SourceId &&
						Continuation.SurfaceId ==
							VerticalRuledSegments[Component[0]].SurfaceId &&
						Continuation.FeatureId ==
							VerticalRuledSegments[Component[0]].FeatureId;
				});
			if (bHasCompatibleContinuation)
			{
				ContinuationEndpoints.Add(Pair.Key);
			}
		}
		if (ContinuationEndpoints.Num() < 2) continue;
		FIntPoint ChainEndpoints[2] = {
			ContinuationEndpoints[0], ContinuationEndpoints[1] };
		double MaximumEndpointDistanceSquared = -1.0;
		for (int32 A = 0; A < ContinuationEndpoints.Num(); ++A)
		{
			for (int32 B = A + 1; B < ContinuationEndpoints.Num(); ++B)
			{
				const FVector2d PositionA =
					ComponentEndpointPositionSums[ContinuationEndpoints[A]] /
					ComponentEndpointPositionCounts[ContinuationEndpoints[A]];
				const FVector2d PositionB =
					ComponentEndpointPositionSums[ContinuationEndpoints[B]] /
					ComponentEndpointPositionCounts[ContinuationEndpoints[B]];
				const double DistanceSquared =
					FVector2d::DistSquared(PositionA, PositionB);
				if (DistanceSquared > MaximumEndpointDistanceSquared)
				{
					MaximumEndpointDistanceSquared = DistanceSquared;
					ChainEndpoints[0] = ContinuationEndpoints[A];
					ChainEndpoints[1] = ContinuationEndpoints[B];
				}
			}
		}
		TArray<FIntPoint> EndpointQueue = { ChainEndpoints[0] };
		TSet<FIntPoint> DiscoveredEndpoints;
		TMap<FIntPoint, FIntPoint> PreviousEndpoint;
		TMap<FIntPoint, int32> PreviousSegment;
		DiscoveredEndpoints.Add(ChainEndpoints[0]);
		for (int32 QueueIndex = 0;
			QueueIndex < EndpointQueue.Num() &&
				!DiscoveredEndpoints.Contains(ChainEndpoints[1]); ++QueueIndex)
		{
			const FIntPoint CurrentEndpoint = EndpointQueue[QueueIndex];
			const TArray<int32>* IncidentSegments =
				ShortSegmentsByEndpoint.Find(CurrentEndpoint);
			if (!IncidentSegments) continue;
			for (const int32 SegmentIndex : *IncidentSegments)
			{
				if (!ComponentSet.Contains(SegmentIndex)) continue;
				const FVerticalRuledSegment& Segment =
					VerticalRuledSegments[SegmentIndex];
				const FIntPoint AdjacentEndpoint =
					Segment.EndpointKey[0] == CurrentEndpoint
						? Segment.EndpointKey[1] : Segment.EndpointKey[0];
				if (DiscoveredEndpoints.Contains(AdjacentEndpoint)) continue;
				DiscoveredEndpoints.Add(AdjacentEndpoint);
				PreviousEndpoint.Add(AdjacentEndpoint, CurrentEndpoint);
				PreviousSegment.Add(AdjacentEndpoint, SegmentIndex);
				EndpointQueue.Add(AdjacentEndpoint);
			}
		}
		if (!DiscoveredEndpoints.Contains(ChainEndpoints[1])) continue;
		TArray<FIntPoint> ReverseEndpointPath = { ChainEndpoints[1] };
		TArray<int32> ReverseSegmentPath;
		FIntPoint CurrentEndpoint = ChainEndpoints[1];
		while (CurrentEndpoint != ChainEndpoints[0])
		{
			const int32* SegmentIndex = PreviousSegment.Find(CurrentEndpoint);
			const FIntPoint* PriorEndpoint = PreviousEndpoint.Find(CurrentEndpoint);
			if (!SegmentIndex || !PriorEndpoint)
			{
				ReverseEndpointPath.Reset();
				break;
			}
			ReverseSegmentPath.Add(*SegmentIndex);
			CurrentEndpoint = *PriorEndpoint;
			ReverseEndpointPath.Add(CurrentEndpoint);
		}
		if (ReverseEndpointPath.Num() < 5) continue;
		TArray<FVector2d> OrderedSectionPoints;
		TArray<int32> OrderedSegmentPath;
		OrderedSectionPoints.Reserve(ReverseEndpointPath.Num());
		OrderedSegmentPath.Reserve(ReverseSegmentPath.Num());
		for (int32 Index = ReverseEndpointPath.Num() - 1; Index >= 0; --Index)
		{
			const FIntPoint Endpoint = ReverseEndpointPath[Index];
			OrderedSectionPoints.Add(ComponentEndpointPositionSums[Endpoint] /
				ComponentEndpointPositionCounts[Endpoint]);
		}
		for (int32 Index = ReverseSegmentPath.Num() - 1; Index >= 0; --Index)
		{
			OrderedSegmentPath.Add(ReverseSegmentPath[Index]);
		}
		double MinimumZ = -TNumericLimits<double>::Max();
		double MaximumZ = TNumericLimits<double>::Max();
		double ArcLength = 0.0;
		for (const int32 SegmentIndex : OrderedSegmentPath)
		{
			const FVerticalRuledSegment& Segment =
				VerticalRuledSegments[SegmentIndex];
			MinimumZ = FMath::Max(MinimumZ, Segment.MinimumZ);
			MaximumZ = FMath::Min(MaximumZ, Segment.MaximumZ);
		}
		for (int32 Index = 0; Index + 1 < OrderedSectionPoints.Num(); ++Index)
		{
			ArcLength += FVector2d::Distance(
				OrderedSectionPoints[Index], OrderedSectionPoints[Index + 1]);
		}
		UE_LOG(LogTemp, Display, TEXT(
			"[AnalyticVerticalRuledCandidate] Segments=%d PathSegments=%d Continuations=%d ArcCm=%.9g Z=[%.9g,%.9g]"),
			Component.Num(), OrderedSegmentPath.Num(),
			ContinuationEndpoints.Num(),
			ArcLength, MinimumZ, MaximumZ);
		if (MaximumZ - MinimumZ < MinimumTransitionVerticalSpanCm ||
			ArcLength > 1000.0)
		{
			continue;
		}
		double TotalTurnDegrees = 0.0;
		for (int32 Index = 1; Index + 1 < OrderedSectionPoints.Num(); ++Index)
		{
			const FVector2d Before = (OrderedSectionPoints[Index] -
				OrderedSectionPoints[Index - 1]).GetSafeNormal();
			const FVector2d After = (OrderedSectionPoints[Index + 1] -
				OrderedSectionPoints[Index]).GetSafeNormal();
			TotalTurnDegrees += FMath::RadiansToDegrees(FMath::Acos(
				FMath::Clamp(FVector2d::DotProduct(Before, After), -1.0, 1.0)));
		}
		// Nearly straight chains belong to the neighbouring planar partition. A
		// deliberate shallow bend still needs continuous support, while folds over
		// 180 degrees are not single-valued transition sections.
		if (TotalTurnDegrees < MinimumTransitionTurnDegrees ||
			TotalTurnDegrees > 180.0) continue;
		const bool bSourceDirectShallowTransition =
			TotalTurnDegrees >= SourceDirectMinimumTurnDegrees &&
			TotalTurnDegrees <= SourceDirectMaximumTurnDegrees &&
			ArcLength >= SourceDirectMinimumArcLengthCm &&
			ArcLength <= SourceDirectMaximumArcLengthCm;

		const FVerticalRuledSegment& SourceSegment =
			VerticalRuledSegments[Component[0]];
		auto SelectPlanarContinuation = [&](const FIntPoint& EndpointKey,
			const FVector2d& Joint, const FVector2d& InteriorDirection)
		{
			int32 BestSegmentIndex = INDEX_NONE;
			double BestDirectionDot = TNumericLimits<double>::Max();
			const TArray<int32>* Continuations =
				LongSegmentsByEndpoint.Find(EndpointKey);
			if (!Continuations) return BestSegmentIndex;
			for (const int32 ContinuationIndex : *Continuations)
			{
				const FVerticalRuledSegment& Continuation =
					VerticalRuledSegments[ContinuationIndex];
				if (!Continuation.bQueryCollisionEnabled ||
					Continuation.SourceId != SourceSegment.SourceId ||
					Continuation.SurfaceId != SourceSegment.SurfaceId ||
					Continuation.FeatureId != SourceSegment.FeatureId ||
					FMath::Min(Continuation.MaximumZ, MaximumZ) -
						FMath::Max(Continuation.MinimumZ, MinimumZ) <
							MinimumTransitionVerticalSpanCm)
				{
					continue;
				}
				const int32 SharedCorner =
					Continuation.EndpointKey[0] == EndpointKey ? 0 : 1;
				const FVector2d Direction =
					(Continuation.Endpoint[1 - SharedCorner] - Joint)
					.GetSafeNormal();
				if (Direction.IsNearlyZero()) continue;
				const double DirectionDot = FVector2d::DotProduct(
					Direction, InteriorDirection);
				if (DirectionDot < BestDirectionDot)
				{
					BestDirectionDot = DirectionDot;
					BestSegmentIndex = ContinuationIndex;
				}
			}
			return BestSegmentIndex;
		};
		const int32 StartContinuationIndex = SelectPlanarContinuation(
			ChainEndpoints[0], OrderedSectionPoints[0],
			(OrderedSectionPoints[1] - OrderedSectionPoints[0]).GetSafeNormal());
		const int32 LastOrderedPointIndex = OrderedSectionPoints.Num() - 1;
		const int32 EndContinuationIndex = SelectPlanarContinuation(
			ChainEndpoints[1], OrderedSectionPoints[LastOrderedPointIndex],
			(OrderedSectionPoints[LastOrderedPointIndex - 1] -
				OrderedSectionPoints[LastOrderedPointIndex]).GetSafeNormal());
		auto ProjectToSourceSection = [&](const FVector2d& Sample,
			FVector2d& OutProjectedPoint)
		{
			double BestResidualCm = TNumericLimits<double>::Max();
			OutProjectedPoint = FVector2d::ZeroVector;
			for (int32 CandidateIndex = 0;
				CandidateIndex < VerticalRuledSegments.Num(); ++CandidateIndex)
			{
				const FVerticalRuledSegment& Candidate =
					VerticalRuledSegments[CandidateIndex];
				const bool bSelectedPlanarContinuation =
					CandidateIndex == StartContinuationIndex ||
					CandidateIndex == EndContinuationIndex;
				if (!Candidate.bQueryCollisionEnabled ||
					Candidate.SourceId != SourceSegment.SourceId ||
					Candidate.SurfaceId != SourceSegment.SurfaceId ||
					Candidate.FeatureId != SourceSegment.FeatureId ||
					(!bSelectedPlanarContinuation &&
						FVector2d::Distance(Candidate.Endpoint[0],
							Candidate.Endpoint[1]) >
								MaximumTransitionSegmentLengthCm) ||
					FMath::Min(Candidate.MaximumZ, MaximumZ) -
						FMath::Max(Candidate.MinimumZ, MinimumZ) <
							MinimumTransitionVerticalSpanCm)
				{
					continue;
				}
				const FVector2d CandidateDelta =
					Candidate.Endpoint[1] - Candidate.Endpoint[0];
				const double CandidateLengthSquared =
					CandidateDelta.SquaredLength();
				if (CandidateLengthSquared <= UE_DOUBLE_SMALL_NUMBER) continue;
				const double Alpha = FMath::Clamp(FVector2d::DotProduct(
					Sample - Candidate.Endpoint[0], CandidateDelta) /
					CandidateLengthSquared, 0.0, 1.0);
				const FVector2d ProjectedPoint =
					Candidate.Endpoint[0] + Alpha * CandidateDelta;
				const double ResidualCm =
					FVector2d::Distance(Sample, ProjectedPoint);
				if (ResidualCm < BestResidualCm)
				{
					BestResidualCm = ResidualCm;
					OutProjectedPoint = ProjectedPoint;
				}
			}
			return BestResidualCm;
		};
		TArray<FVector2d> ImageSectionPoints[1];
		TArray<double> ImageSectionKnots[1];
		TArray<FCubicBezierSegment> ImageCubicSegments[1];
		double MaximumImageAssociationResidualCm = 0.0;
		bool bImagesCertified = true;
		for (uint8 TransformMask = 0; TransformMask < 1; ++TransformMask)
		{
			ImageSectionPoints[TransformMask].Reserve(
				OrderedSectionPoints.Num());
			for (FVector2d ReflectedPoint : OrderedSectionPoints)
			{
				if ((TransformMask & 1u) != 0)
				{
					ReflectedPoint.X *= -1.0;
				}
				if ((TransformMask & 2u) != 0)
				{
					ReflectedPoint.Y *= -1.0;
				}
				FVector2d ProjectedPoint;
				const double AssociationResidualCm =
					ProjectToSourceSection(ReflectedPoint, ProjectedPoint);
				MaximumImageAssociationResidualCm = FMath::Max(
					MaximumImageAssociationResidualCm, AssociationResidualCm);
				bImagesCertified &= AssociationResidualCm <= 2.0;
				ImageSectionPoints[TransformMask].Add(ProjectedPoint);
			}
			bool bBuiltPlanarFillet = false;
			if (ImageSectionPoints[TransformMask].Num() >= 3 &&
				StartContinuationIndex != INDEX_NONE &&
				EndContinuationIndex != INDEX_NONE)
			{
				const FVerticalRuledSegment& StartContinuation =
					VerticalRuledSegments[StartContinuationIndex];
				const int32 StartSharedCorner =
					StartContinuation.EndpointKey[0] == ChainEndpoints[0] ? 0 : 1;
				const FVector2d StartLeadDirection =
					(StartContinuation.Endpoint[1 - StartSharedCorner] -
						ImageSectionPoints[TransformMask][0]).GetSafeNormal();
				const FVerticalRuledSegment& EndContinuation =
					VerticalRuledSegments[EndContinuationIndex];
				const int32 EndSharedCorner =
					EndContinuation.EndpointKey[0] == ChainEndpoints[1] ? 0 : 1;
				const FVector2d EndLeadDirection =
					(EndContinuation.Endpoint[1 - EndSharedCorner] -
						ImageSectionPoints[TransformMask].Last()).GetSafeNormal();
				const double TransitionLeadInCm = TotalTurnDegrees < 75.0
					? (bSourceDirectShallowTransition
						? SourceDirectTransitionLeadInCm
						: ShallowTransitionLeadInCm)
					: SteepTransitionLeadInCm;
				const FVector2d StartPoint =
					ImageSectionPoints[TransformMask][0] +
					TransitionLeadInCm * StartLeadDirection;
				const FVector2d EndPoint =
					ImageSectionPoints[TransformMask].Last() +
					TransitionLeadInCm * EndLeadDirection;
				const FVector2d StartTangent = -StartLeadDirection;
				const FVector2d EndTangent = EndLeadDirection;
				const double TurnRadians = FMath::Acos(FMath::Clamp(
					FVector2d::DotProduct(StartTangent, EndTangent), -1.0, 1.0));
				const bool bShallowTransition = TotalTurnDegrees < 75.0;
				const double EndpointTurnFraction =
					bSourceDirectShallowTransition
						? 0.0 : ShallowTransitionEndpointTurnFraction;
				const double SupportInsetCm = bSourceDirectShallowTransition
					? SourceDirectSupportInsetCm : ShallowTransitionSupportInsetCm;
				const double SupportInsetRampCm = bSourceDirectShallowTransition
					? SourceDirectSupportInsetRampCm : ShallowTransitionSupportInsetRampCm;
				const bool bHasShallowSupportInset = bShallowTransition &&
					SupportInsetCm > UE_DOUBLE_SMALL_NUMBER;
				const double SignedTurn =
					StartTangent.X * EndTangent.Y -
					StartTangent.Y * EndTangent.X;
				const double InteriorSide = SignedTurn < 0.0 ? -1.0 : 1.0;
				const FVector2d StartInteriorNormal = InteriorSide *
					FVector2d(-StartTangent.Y, StartTangent.X);
				const FVector2d EndInteriorNormal = InteriorSide *
					FVector2d(-EndTangent.Y, EndTangent.X);
				const FVector2d CurveStartTangent = bShallowTransition
					? FMath::Lerp(StartTangent, EndTangent,
						EndpointTurnFraction).GetSafeNormal()
					: StartTangent;
				const FVector2d CurveEndTangent = bShallowTransition
					? FMath::Lerp(StartTangent, EndTangent,
						1.0 - EndpointTurnFraction).GetSafeNormal()
					: EndTangent;
				const FVector2d ArcStartPoint = StartPoint +
					(bHasShallowSupportInset ? SupportInsetCm : 0.0) *
					StartInteriorNormal;
				const FVector2d ArcEndPoint = EndPoint +
					(bHasShallowSupportInset ? SupportInsetCm : 0.0) *
					EndInteriorNormal;
				const double ChordLength =
					FVector2d::Distance(ArcStartPoint, ArcEndPoint);
				const double SinHalfTurn = FMath::Sin(0.5 * TurnRadians);
				if (TurnRadians > UE_DOUBLE_SMALL_NUMBER &&
					SinHalfTurn > UE_DOUBLE_SMALL_NUMBER && ChordLength > 1.0)
				{
					const double Radius = ChordLength / (2.0 * SinHalfTurn);
					const double HandleLength = 4.0 / 3.0 * Radius *
						FMath::Tan(0.25 * TurnRadians);
					FCubicBezierSegment ArcCubic;
					ArcCubic.ControlPoints[0] = FVector3d(ArcStartPoint, 0.0);
					ArcCubic.ControlPoints[1] = FVector3d(
						ArcStartPoint + HandleLength * CurveStartTangent, 0.0);
					ArcCubic.ControlPoints[2] = FVector3d(
						ArcEndPoint - HandleLength * CurveEndTangent, 0.0);
					ArcCubic.ControlPoints[3] = FVector3d(ArcEndPoint, 0.0);
					const FVector2d RampStartPoint = StartPoint -
						SupportInsetRampCm * StartTangent;
					const FVector2d RampEndPoint = EndPoint +
						SupportInsetRampCm * EndTangent;
					ImageSectionPoints[TransformMask] = bHasShallowSupportInset
						? TArray<FVector2d>{ RampStartPoint, RampEndPoint }
						: TArray<FVector2d>{ ArcStartPoint, ArcEndPoint };
					TArray<double>& Knots = ImageSectionKnots[TransformMask];
					Knots = { 0.0 };
					if (bHasShallowSupportInset)
					{
						FCubicBezierSegment& StartRamp =
							ImageCubicSegments[TransformMask].AddDefaulted_GetRef();
						StartRamp.ControlPoints[0] = FVector3d(RampStartPoint, 0.0);
						StartRamp.ControlPoints[1] = FVector3d(
							RampStartPoint + SupportInsetRampCm /
							3.0 * StartTangent, 0.0);
						StartRamp.ControlPoints[2] = FVector3d(
							ArcStartPoint - SupportInsetRampCm /
							3.0 * CurveStartTangent, 0.0);
						StartRamp.ControlPoints[3] = FVector3d(ArcStartPoint, 0.0);
						Knots.Add(Knots.Last() + FVector2d::Distance(
							RampStartPoint, ArcStartPoint));
					}
					ImageCubicSegments[TransformMask].Add(ArcCubic);
					Knots.Add(Knots.Last() + Radius * TurnRadians);
					if (bHasShallowSupportInset)
					{
						FCubicBezierSegment& EndRamp =
							ImageCubicSegments[TransformMask].AddDefaulted_GetRef();
						EndRamp.ControlPoints[0] = FVector3d(ArcEndPoint, 0.0);
						EndRamp.ControlPoints[1] = FVector3d(
							ArcEndPoint + SupportInsetRampCm /
							3.0 * CurveEndTangent, 0.0);
						EndRamp.ControlPoints[2] = FVector3d(
							RampEndPoint - SupportInsetRampCm /
							3.0 * EndTangent, 0.0);
						EndRamp.ControlPoints[3] = FVector3d(RampEndPoint, 0.0);
						Knots.Add(Knots.Last() + FVector2d::Distance(
							ArcEndPoint, RampEndPoint));
					}
					bBuiltPlanarFillet = true;
				}
			}
			if (!bBuiltPlanarFillet)
			{
				TArray<double>& Knots = ImageSectionKnots[TransformMask];
				TArray<FVector3d> Values;
				Knots.Reserve(ImageSectionPoints[TransformMask].Num());
				Values.Reserve(ImageSectionPoints[TransformMask].Num());
				Knots.Add(0.0);
				Values.Add(FVector3d(ImageSectionPoints[TransformMask][0], 0.0));
				for (int32 PointIndex = 1;
					PointIndex < ImageSectionPoints[TransformMask].Num(); ++PointIndex)
				{
					Knots.Add(Knots.Last() + FVector2d::Distance(
						ImageSectionPoints[TransformMask][PointIndex - 1],
						ImageSectionPoints[TransformMask][PointIndex]));
					Values.Add(FVector3d(
						ImageSectionPoints[TransformMask][PointIndex], 0.0));
				}
				bImagesCertified &= Knots.Last() > 1.0 &&
					BuildNaturalCubicBezierSegments(Knots, Values,
						ImageCubicSegments[TransformMask]);
				// Natural endpoint conditions are smooth internally but do not preserve
				// the tangent of the adjacent planar runs. Rotate only the boundary
				// handles onto those continuations, preserving their fitted magnitudes
				// and every internal C1 join. This removes a localized support kink
				// without expanding or refitting the authored transition.
				if (!ImageCubicSegments[TransformMask].IsEmpty() &&
					StartContinuationIndex != INDEX_NONE &&
					EndContinuationIndex != INDEX_NONE)
				{
					const FVerticalRuledSegment& StartContinuation =
						VerticalRuledSegments[StartContinuationIndex];
					const int32 StartSharedCorner =
						StartContinuation.EndpointKey[0] == ChainEndpoints[0] ? 0 : 1;
					const FVector2d StartOutwardDirection =
						(StartContinuation.Endpoint[1 - StartSharedCorner] -
							ImageSectionPoints[TransformMask][0]).GetSafeNormal();
					const FVerticalRuledSegment& EndContinuation =
						VerticalRuledSegments[EndContinuationIndex];
					const int32 EndSharedCorner =
						EndContinuation.EndpointKey[0] == ChainEndpoints[1] ? 0 : 1;
					const FVector2d EndOutwardDirection =
						(EndContinuation.Endpoint[1 - EndSharedCorner] -
							ImageSectionPoints[TransformMask].Last()).GetSafeNormal();
					if (!StartOutwardDirection.IsNearlyZero() &&
						!EndOutwardDirection.IsNearlyZero())
					{
						FCubicBezierSegment& FirstSegment =
							ImageCubicSegments[TransformMask][0];
						const double StartHandleLength = FVector3d::Distance(
							FirstSegment.ControlPoints[0], FirstSegment.ControlPoints[1]);
						FirstSegment.ControlPoints[1] = FirstSegment.ControlPoints[0] +
							StartHandleLength * FVector3d(-StartOutwardDirection, 0.0);
						FCubicBezierSegment& LastSegment =
							ImageCubicSegments[TransformMask].Last();
						const double EndHandleLength = FVector3d::Distance(
							LastSegment.ControlPoints[2], LastSegment.ControlPoints[3]);
						LastSegment.ControlPoints[2] = LastSegment.ControlPoints[3] -
							EndHandleLength * FVector3d(EndOutwardDirection, 0.0);
					}
				}
			}
		}
		double MaximumImageSourceResidualCm = 0.0;
		for (uint8 TransformMask = 0; TransformMask < 1; ++TransformMask)
		{
			for (int32 SegmentIndex = 0;
				SegmentIndex < ImageCubicSegments[TransformMask].Num();
				++SegmentIndex)
			{
				for (int32 SampleIndex = 0; SampleIndex <= 16; ++SampleIndex)
				{
					const double Alpha =
						static_cast<double>(SampleIndex) / 16.0;
					const FVector3d CubicSample = EvaluateBezierControlPolygon(
						ImageCubicSegments[TransformMask][SegmentIndex].ControlPoints,
						Alpha);
					const FVector2d Sample(CubicSample.X, CubicSample.Y);
					FVector2d SourcePoint;
					MaximumImageSourceResidualCm = FMath::Max(
						MaximumImageSourceResidualCm,
						ProjectToSourceSection(Sample, SourcePoint));
				}
			}
		}
		const double MaximumAllowedSourceResidualCm = TotalTurnDegrees < 75.0
			? (bSourceDirectShallowTransition
				? SourceDirectMaximumResidualCm
				: MaximumTransitionSourceResidualCm)
			: MaximumSteepTransitionSourceResidualCm;
		bImagesCertified &= MaximumImageSourceResidualCm <=
			MaximumAllowedSourceResidualCm;
		UE_LOG(LogTemp, Display, TEXT(
			"[AnalyticVerticalRuledSymmetry] Segments=%d AssociationResidualCm=%.9g SourceResidualCm=%.9g Certified=%d"),
			Component.Num(), MaximumImageAssociationResidualCm,
			MaximumImageSourceResidualCm, bImagesCertified ? 1 : 0);
		if (!bImagesCertified) continue;
		FPiecewiseTensorBezierPatch TransitionPatch;
		TransitionPatch.SourceId = SourceSegment.SourceId;
		TransitionPatch.SurfaceId = SourceSegment.SurfaceId;
		TransitionPatch.PrimitiveId = CombineStableIds(
			StableStringId(TEXT("VerticalRuledTransition.V40")),
			CombineStableIds(SectionPointId(ChainEndpoints[0]),
				SectionPointId(ChainEndpoints[1])));
		TransitionPatch.CanonicalGroupId = CombineStableIds(
			StableStringId(TEXT("VerticalRuledTransition.Canonical.V40")),
			SourceSegment.SourceId);
		TransitionPatch.MaterialId = SourceSegment.MaterialId;
		TransitionPatch.ObjectType = SourceSegment.ObjectType;
		TransitionPatch.BlockingChannels = SourceSegment.BlockingChannels;
		TransitionPatch.bQueryCollisionEnabled =
			SourceSegment.bQueryCollisionEnabled;
		for (uint8 TransformMask = 0; TransformMask < 1; ++TransformMask)
		{
			for (int32 SegmentIndex = 0;
				SegmentIndex < ImageCubicSegments[TransformMask].Num();
				++SegmentIndex)
			{
				const FCubicBezierSegment& Cubic =
					ImageCubicSegments[TransformMask][SegmentIndex];
				const TArray<double>& Knots = ImageSectionKnots[TransformMask];
				const double TotalArcLength = Knots.Last();
				FPiecewiseTensorBezierCell& Cell =
					TransitionPatch.Cells.AddDefaulted_GetRef();
				Cell.FeatureId = SourceSegment.FeatureId;
				Cell.PrimitiveId = CombineStableIds(TransitionPatch.PrimitiveId,
					CombineStableIds(static_cast<uint64>(TransformMask + 1),
						static_cast<uint64>(SegmentIndex + 1)));
				Cell.MinimumU = Knots[SegmentIndex] / TotalArcLength;
				Cell.MaximumU = Knots[SegmentIndex + 1] / TotalArcLength;
				Cell.MinimumV = 0.0;
				Cell.MaximumV = 1.0;
				Cell.LongitudinalParameterScale = 1.0;
				Cell.Surface.DegreeU = 3;
				Cell.Surface.DegreeV = 1;
				Cell.Surface.ControlPoints.Reserve(8);
				for (const FVector3d& Control : Cubic.ControlPoints)
				{
					Cell.Surface.ControlPoints.Add(FVector3d(
						Control.X, Control.Y, MinimumZ));
					Cell.Surface.ControlPoints.Add(FVector3d(
						Control.X, Control.Y, MaximumZ));
				}
			}
		}
		Algo::Sort(TransitionPatch.Cells,
			[](const FPiecewiseTensorBezierCell& A,
				const FPiecewiseTensorBezierCell& B)
			{
				return A.PrimitiveId < B.PrimitiveId;
			});
		TransitionPatch.bSourceResidualCertified = true;
		if (TransitionPatch.BuildQueryApproximation())
		{
			TransitionPatch.bAuthorityEligible =
				TransitionPatch.bApproximationCertified &&
				TransitionPatch.bQueryCollisionEnabled;
			UE_LOG(LogTemp, Display, TEXT(
				"[AnalyticVerticalRuledTransition] Primitive=%016llX Segments=%d ArcCm=%.9g TurnDeg=%.9g AssociationResidualCm=%.9g SourceResidualCm=%.9g Z=[%.9g,%.9g] BoundsMin=%s BoundsMax=%s Certified=%d"),
				TransitionPatch.PrimitiveId, OrderedSegmentPath.Num(), ArcLength,
				TotalTurnDegrees, MaximumImageAssociationResidualCm,
				MaximumImageSourceResidualCm, MinimumZ, MaximumZ,
				*TransitionPatch.Bounds.Min.ToString(),
				*TransitionPatch.Bounds.Max.ToString(),
				TransitionPatch.bAuthorityEligible ? 1 : 0);
			if (TransitionPatch.bAuthorityEligible)
			{
				VerticalRuledTransitionPatches.Add(MoveTemp(TransitionPatch));
			}
		}
	}
	// A source mesh can omit one connected short-edge chain even though the
	// surrounding architecture and the other members establish an exact mirror
	// orbit. Materialize only absent mirror members, and certify each reflected
	// control net against the source's ruled segments before granting authority.
	// Existing independently recognized members always win; this completion step
	// cannot replace or perturb them.
	struct FMirroredTransitionCandidate
	{
		FPiecewiseTensorBezierPatch Patch;
		double MaximumSourceResidualCm = TNumericLimits<double>::Max();
	};
	const auto TransitionFootprintsMatch = [](
		const FPiecewiseTensorBezierPatch& A,
		const FPiecewiseTensorBezierPatch& B)
	{
		if (!A.Bounds.IsValid || !B.Bounds.IsValid ||
			A.SourceId != B.SourceId || A.SurfaceId != B.SurfaceId ||
			A.Cells.IsEmpty() || B.Cells.IsEmpty() ||
			A.Cells[0].FeatureId != B.Cells[0].FeatureId ||
			A.MaterialId != B.MaterialId ||
			A.ObjectType != B.ObjectType ||
			A.BlockingChannels != B.BlockingChannels)
		{
			return false;
		}
		constexpr double OrbitFootprintToleranceCm = 25.0;
		return FVector3d::Distance(A.Bounds.GetCenter(), B.Bounds.GetCenter()) <=
				OrbitFootprintToleranceCm &&
			FVector3d::Distance(A.Bounds.GetExtent(), B.Bounds.GetExtent()) <=
				OrbitFootprintToleranceCm;
	};
	const auto ReflectTransitionPatch = [&](const FPiecewiseTensorBezierPatch& Source,
		const uint8 TransformMask, FPiecewiseTensorBezierPatch& OutPatch)
	{
		OutPatch = Source;
		for (FPiecewiseTensorBezierCell& Cell : OutPatch.Cells)
		{
			for (FVector3d& ControlPoint : Cell.Surface.ControlPoints)
			{
				if ((TransformMask & 1u) != 0) ControlPoint.X *= -1.0;
				if ((TransformMask & 2u) != 0) ControlPoint.Y *= -1.0;
			}
		}
		OutPatch.bAuthorityEligible = false;
		if (!OutPatch.BuildQueryApproximation() || !OutPatch.Bounds.IsValid)
		{
			return false;
		}
		const FIntPoint CenterKey = QuantizeSectionPoint(FVector2d(
			OutPatch.Bounds.GetCenter().X, OutPatch.Bounds.GetCenter().Y));
		const FIntPoint ExtentKey = QuantizeSectionPoint(FVector2d(
			OutPatch.Bounds.GetExtent().X, OutPatch.Bounds.GetExtent().Y));
		OutPatch.PrimitiveId = CombineStableIds(
			StableStringId(TEXT("VerticalRuledTransition.Symmetry.V35")),
			CombineStableIds(SectionPointId(CenterKey), SectionPointId(ExtentKey)));
		for (int32 CellIndex = 0; CellIndex < OutPatch.Cells.Num(); ++CellIndex)
		{
			OutPatch.Cells[CellIndex].PrimitiveId = CombineStableIds(
				OutPatch.PrimitiveId, static_cast<uint64>(CellIndex + 1));
		}
		return true;
	};
	const auto MirroredTransitionSourceResidual = [&](
		const FPiecewiseTensorBezierPatch& Patch)
	{
		double MaximumResidualCm = 0.0;
		for (const FPiecewiseTensorBezierCell& Cell : Patch.Cells)
		{
			const int32 VCount = Cell.Surface.DegreeV + 1;
			TArray<FVector3d, TInlineAllocator<8>> SectionControls;
			SectionControls.Reserve(Cell.Surface.DegreeU + 1);
			for (int32 UIndex = 0; UIndex <= Cell.Surface.DegreeU; ++UIndex)
			{
				SectionControls.Add(Cell.Surface.ControlPoints[UIndex * VCount]);
			}
			for (int32 SampleIndex = 0; SampleIndex <= 32; ++SampleIndex)
			{
				const FVector3d Sample = EvaluateBezierControlPolygon(
					SectionControls,
					static_cast<double>(SampleIndex) / 32.0);
				double BestResidualCm = TNumericLimits<double>::Max();
				for (const FVerticalRuledSegment& Segment : VerticalRuledSegments)
				{
					if (!Segment.bQueryCollisionEnabled ||
						Segment.SourceId != Patch.SourceId ||
						Segment.SurfaceId != Patch.SurfaceId ||
						Segment.FeatureId != Cell.FeatureId ||
						Segment.MaterialId != Patch.MaterialId ||
						Segment.ObjectType != Patch.ObjectType ||
						Segment.BlockingChannels != Patch.BlockingChannels ||
						Sample.Z < Segment.MinimumZ - 1.0 ||
						Sample.Z > Segment.MaximumZ + 1.0)
					{
						continue;
					}
					const FVector2d Delta = Segment.Endpoint[1] - Segment.Endpoint[0];
					const double LengthSquared = Delta.SquaredLength();
					if (LengthSquared <= UE_DOUBLE_SMALL_NUMBER) continue;
					const double Alpha = FMath::Clamp(FVector2d::DotProduct(
						FVector2d(Sample.X, Sample.Y) - Segment.Endpoint[0], Delta) /
						LengthSquared, 0.0, 1.0);
					BestResidualCm = FMath::Min(BestResidualCm,
						FVector2d::Distance(FVector2d(Sample.X, Sample.Y),
							Segment.Endpoint[0] + Alpha * Delta));
				}
				if (!FMath::IsFinite(BestResidualCm))
				{
					return TNumericLimits<double>::Max();
				}
				MaximumResidualCm = FMath::Max(MaximumResidualCm, BestResidualCm);
			}
		}
		return MaximumResidualCm;
	};
	Algo::Sort(VerticalRuledTransitionPatches,
		[](const FPiecewiseTensorBezierPatch& A,
			const FPiecewiseTensorBezierPatch& B)
		{
			return A.PrimitiveId < B.PrimitiveId;
		});
	TArray<FMirroredTransitionCandidate> MirroredTransitionCandidates;
	const int32 RecognizedTransitionCount = VerticalRuledTransitionPatches.Num();
	for (int32 PatchIndex = 0; PatchIndex < RecognizedTransitionCount; ++PatchIndex)
	{
		const FPiecewiseTensorBezierPatch& SourcePatch =
			VerticalRuledTransitionPatches[PatchIndex];
		for (uint8 TransformMask = 1; TransformMask < 4; ++TransformMask)
		{
			FPiecewiseTensorBezierPatch ReflectedPatch;
			if (!ReflectTransitionPatch(SourcePatch, TransformMask, ReflectedPatch) ||
				Algo::AnyOf(VerticalRuledTransitionPatches,
					[&](const FPiecewiseTensorBezierPatch& Existing)
					{
						return TransitionFootprintsMatch(Existing, ReflectedPatch);
					}))
			{
				continue;
			}
			const double MaximumResidualCm =
				MirroredTransitionSourceResidual(ReflectedPatch);
			const FVector3d PlanarSpan = ReflectedPatch.Bounds.GetSize();
			const bool bSteepTransition = FMath::Max(
				PlanarSpan.X, PlanarSpan.Y) > 500.0;
			const double MaximumAllowedResidualCm =
				bSteepTransition ? MaximumSteepTransitionSourceResidualCm
					: MaximumTransitionSourceResidualCm;
			if (MaximumResidualCm > MaximumAllowedResidualCm) continue;
			FMirroredTransitionCandidate& Candidate =
				MirroredTransitionCandidates.AddDefaulted_GetRef();
			Candidate.Patch = MoveTemp(ReflectedPatch);
			Candidate.MaximumSourceResidualCm = MaximumResidualCm;
		}
	}
	MirroredTransitionCandidates.Sort([](
		const FMirroredTransitionCandidate& A,
		const FMirroredTransitionCandidate& B)
	{
		return A.MaximumSourceResidualCm != B.MaximumSourceResidualCm
			? A.MaximumSourceResidualCm < B.MaximumSourceResidualCm
			: A.Patch.PrimitiveId < B.Patch.PrimitiveId;
	});
	for (FMirroredTransitionCandidate& Candidate : MirroredTransitionCandidates)
	{
		if (Algo::AnyOf(VerticalRuledTransitionPatches,
			[&](const FPiecewiseTensorBezierPatch& Existing)
			{
				return TransitionFootprintsMatch(Existing, Candidate.Patch);
			}))
		{
			continue;
		}
		Candidate.Patch.bSourceResidualCertified = true;
		Candidate.Patch.bAuthorityEligible =
			Candidate.Patch.bApproximationCertified &&
			Candidate.Patch.bQueryCollisionEnabled;
		UE_LOG(LogTemp, Display, TEXT(
			"[AnalyticVerticalRuledSymmetryCompletion] Primitive=%016llX SourceResidualCm=%.9g BoundsMin=%s BoundsMax=%s Certified=%d"),
			Candidate.Patch.PrimitiveId, Candidate.MaximumSourceResidualCm,
			*Candidate.Patch.Bounds.Min.ToString(),
			*Candidate.Patch.Bounds.Max.ToString(),
			Candidate.Patch.bAuthorityEligible ? 1 : 0);
		if (Candidate.Patch.bAuthorityEligible)
		{
			VerticalRuledTransitionPatches.Add(MoveTemp(Candidate.Patch));
		}
	}
	// A finite opening may split an otherwise canonical wall-to-ceiling return
	// into two source-backed extrusion intervals. Complete only the missing
	// central interval with the already-certified C2 section and overlap both
	// source intervals enough for finite sphere/box footprints.
	TArray<FExtrudedQuinticPatch> CentralUpperReturnCompletions;
	for (const double LongitudinalSign : { -1.0, 1.0 })
	{
		const FExtrudedQuinticPatch* Best = nullptr;
		double BestCentralBoundaryCm = TNumericLimits<double>::Max();
		for (const FExtrudedQuinticPatch& Candidate : ExtrudedQuinticPatches)
		{
			const double CenterX = Candidate.Bounds.GetCenter().X;
			const double CentralBoundaryCm = FMath::Min(
				FMath::Abs(Candidate.MinimumExtrusionCoordinate),
				FMath::Abs(Candidate.MaximumExtrusionCoordinate));
			const bool bCentralIntervalMissing =
				Candidate.MinimumExtrusionCoordinate *
					Candidate.MaximumExtrusionCoordinate > 0.0;
			const bool bCanonicalUpperReturn = Candidate.bAuthorityEligible &&
				FMath::Abs(Candidate.ExtrusionAxis.Y) >= 0.99 &&
				LongitudinalSign * CenterX > 4900.0 &&
				Candidate.Bounds.Min.Z >= 1600.0 &&
				Candidate.Bounds.Min.Z <= 1950.0 &&
				Candidate.Bounds.Max.Z >= 2200.0 &&
				Candidate.Bounds.Max.Z <= 2450.0;
			if (bCanonicalUpperReturn && bCentralIntervalMissing &&
				CentralBoundaryCm < BestCentralBoundaryCm)
			{
				Best = &Candidate;
				BestCentralBoundaryCm = CentralBoundaryCm;
			}
		}
		if (!Best) continue;
		FExtrudedQuinticPatch Completion = *Best;
		Completion.PrimitiveId = CombineStableIds(Best->CanonicalGroupId,
			CombineStableIds(StableStringId(
				TEXT("CentralUpperReturnCompletion.V1")),
				LongitudinalSign < 0.0 ? 1ull : 2ull));
		constexpr double CentralUpperReturnHalfWidthCm = 950.0;
		Completion.MinimumExtrusionCoordinate =
			-CentralUpperReturnHalfWidthCm;
		Completion.MaximumExtrusionCoordinate =
			CentralUpperReturnHalfWidthCm;
		Completion.Bounds = FBox3d(EForceInit::ForceInit);
		for (int32 SampleIndex = 0; SampleIndex <= 64; ++SampleIndex)
		{
			const FVector3d Section = Completion.EvaluateSection(
				static_cast<double>(SampleIndex) / 64.0);
			Completion.Bounds += Section +
				Completion.MinimumExtrusionCoordinate * Completion.ExtrusionAxis;
			Completion.Bounds += Section +
				Completion.MaximumExtrusionCoordinate * Completion.ExtrusionAxis;
		}
		Completion.bAuthorityEligible =
			Completion.bQueryCollisionEnabled &&
			Completion.bCanonicalC2ByConstruction &&
			Completion.bCanonicalSymmetryByConstruction &&
			Completion.BuildQueryApproximation() &&
			Completion.MaximumChordErrorCm <=
				ExtrudedQuinticChordToleranceCm;
		UE_LOG(LogTemp, Display, TEXT(
			"[AnalyticCentralUpperReturn] Sign=%+.0f Parent=%016llX Primitive=%016llX Domain=[%.9g,%.9g] BoundsMin=%s BoundsMax=%s Certified=%d"),
			LongitudinalSign, Best->PrimitiveId, Completion.PrimitiveId,
			Completion.MinimumExtrusionCoordinate,
			Completion.MaximumExtrusionCoordinate,
			*Completion.Bounds.Min.ToString(),
			*Completion.Bounds.Max.ToString(),
			Completion.bAuthorityEligible ? 1 : 0);
		if (Completion.bAuthorityEligible)
		{
			CentralUpperReturnCompletions.Add(MoveTemp(Completion));
		}
	}
	ExtrudedQuinticPatches.Append(MoveTemp(CentralUpperReturnCompletions));
	Algo::Sort(ExtrudedQuinticPatches,
		[](const FExtrudedQuinticPatch& A, const FExtrudedQuinticPatch& B)
		{
			return A.PrimitiveId < B.PrimitiveId;
		});

	// Materialize each exact symmetry image of the shared finite-opening fit as
	// an ordinary tensor provider. The recognition records remain diagnostic;
	// runtime queries consume only the generic world-space control nets below.
	const FWorldQueryService EdgeQueryService(*this);
	const auto MeasureTensorCellSourceResidualCm = [this](
		const FTensorBezierSurface& Surface, const uint64 SourceId)
	{
		double MaximumResidualCm = 0.0;
		for (int32 UIndex = 0; UIndex <= 2; ++UIndex)
		{
			for (int32 VIndex = 0; VIndex <= 2; ++VIndex)
			{
				const FVector3d Point = Surface.Evaluate(
					static_cast<double>(UIndex) / 2.0,
					static_cast<double>(VIndex) / 2.0);
				double BestSquaredResidual = TNumericLimits<double>::Max();
				TArray<int32, TInlineAllocator<64>> PendingNodes;
				if (!TriangleBvh.IsEmpty()) PendingNodes.Add(0);
				while (!PendingNodes.IsEmpty())
				{
					const FTriangleBvhNode& Node = TriangleBvh[
						PendingNodes.Pop(EAllowShrinking::No)];
					if (SquaredDistanceToBox(Point, Node.Bounds) >
						BestSquaredResidual)
					{
						continue;
					}
					if (Node.IsLeaf())
					{
						for (int32 Offset = 0; Offset < Node.IndexCount; ++Offset)
						{
							const FTriangleSurface& Triangle = Triangles[
								TriangleIndices[Node.FirstIndex + Offset]];
							if (Triangle.SourceId != SourceId ||
								!Triangle.bQueryCollisionEnabled)
							{
								continue;
							}
							BestSquaredResidual = FMath::Min(BestSquaredResidual,
								FVector3d::DistSquared(Point,
									ClosestPointOnTriangle(Point, Triangle)));
						}
					}
					else
					{
						if (Node.LeftChild != INDEX_NONE)
							PendingNodes.Add(Node.LeftChild);
						if (Node.RightChild != INDEX_NONE)
							PendingNodes.Add(Node.RightChild);
					}
				}
				if (!FMath::IsFinite(BestSquaredResidual))
				{
					return TNumericLimits<double>::Max();
				}
				MaximumResidualCm = FMath::Max(MaximumResidualCm,
					FMath::Sqrt(BestSquaredResidual));
			}
		}
		return MaximumResidualCm;
	};
	const auto MeasureTensorCellSourceTangentErrorDegrees = [this](
		const FTensorBezierSurface& Surface, const uint64 SourceId)
	{
		double MaximumErrorDegrees = 0.0;
		constexpr double InteriorParameters[3] = { 0.125, 0.5, 0.875 };
		for (const double U : InteriorParameters)
		{
			for (const double V : InteriorParameters)
			{
				const FVector3d AnalyticNormal = Surface.EvaluateNormal(U, V);
				if (AnalyticNormal.IsNearlyZero(1.0e-9)) return 180.0;
				const FVector3d Point = Surface.Evaluate(U, V);
				double BestSquaredResidual = TNumericLimits<double>::Max();
				FVector3d BestSourceNormal = FVector3d::ZeroVector;
				TArray<int32, TInlineAllocator<64>> PendingNodes;
				if (!TriangleBvh.IsEmpty()) PendingNodes.Add(0);
				while (!PendingNodes.IsEmpty())
				{
					const FTriangleBvhNode& Node = TriangleBvh[
						PendingNodes.Pop(EAllowShrinking::No)];
					if (SquaredDistanceToBox(Point, Node.Bounds) > BestSquaredResidual)
						continue;
					if (Node.IsLeaf())
					{
						for (int32 Offset = 0; Offset < Node.IndexCount; ++Offset)
						{
							const FTriangleSurface& Triangle = Triangles[
								TriangleIndices[Node.FirstIndex + Offset]];
							if (Triangle.SourceId != SourceId ||
								!Triangle.bQueryCollisionEnabled) continue;
							const double SquaredResidual = FVector3d::DistSquared(
								Point, ClosestPointOnTriangle(Point, Triangle));
							if (SquaredResidual < BestSquaredResidual)
							{
								BestSquaredResidual = SquaredResidual;
								BestSourceNormal = Triangle.FaceNormal.GetSafeNormal();
							}
						}
					}
					else
					{
						if (Node.LeftChild != INDEX_NONE) PendingNodes.Add(Node.LeftChild);
						if (Node.RightChild != INDEX_NONE) PendingNodes.Add(Node.RightChild);
					}
				}
				if (!FMath::IsFinite(BestSquaredResidual) ||
					BestSourceNormal.IsNearlyZero(1.0e-9)) return 180.0;
				const double UnorientedCosine = FMath::Clamp(FMath::Abs(
					FVector3d::DotProduct(AnalyticNormal, BestSourceNormal)), 0.0, 1.0);
				MaximumErrorDegrees = FMath::Max(MaximumErrorDegrees,
					FMath::RadiansToDegrees(FMath::Acos(UnorientedCosine)));
			}
		}
		return MaximumErrorDegrees;
	};
	// Lower and upper wall-corner returns are authored below from the
	// already-certified analytic transition and compact support profiles.
	// Source rays from the enclosed-volume centre are not topologically valid here.
	TArray<FPiecewiseTensorBezierPatch> WallCornerReturnPatches;
	struct FCompactProfileEndpoint
	{
		const FExtrudedQuinticPatch* Patch = nullptr;
		double MatchScoreCm = TNumericLimits<double>::Max();
		double ExtrusionCoordinate = 0.0;
		double WallParameter = 0.0;
		FVector3d WallPoint = FVector3d::ZeroVector;
		FVector3d InwardDirection = FVector3d::ZeroVector;
	};
	auto EvaluateRuledTransition = [](
		const FPiecewiseTensorBezierPatch& Transition,
		const double U, const bool bUpper, FVector3d& OutPoint)
	{
		const FPiecewiseTensorBezierCell* BestCell = nullptr;
		double BestDistance = TNumericLimits<double>::Max();
		for (const FPiecewiseTensorBezierCell& Cell : Transition.Cells)
		{
			const double Distance = U < Cell.MinimumU
				? Cell.MinimumU - U
				: U > Cell.MaximumU ? U - Cell.MaximumU : 0.0;
			if (Distance < BestDistance)
			{
				BestDistance = Distance;
				BestCell = &Cell;
			}
		}
		if (!BestCell || BestCell->MaximumU <= BestCell->MinimumU)
		{
			return false;
		}
		const double LocalU = FMath::Clamp(
			(U - BestCell->MinimumU) /
				(BestCell->MaximumU - BestCell->MinimumU), 0.0, 1.0);
		OutPoint = BestCell->Surface.Evaluate(LocalU, bUpper ? 1.0 : 0.0);
		return true;
	};
	auto FindCompactProfileEndpoint = [&](const FVector3d& TransitionEndpoint,
		const bool bUpper, FCompactProfileEndpoint& OutEndpoint)
	{
		double BestScore = TNumericLimits<double>::Max();
		for (const FExtrudedQuinticPatch& Patch : ExtrudedQuinticPatches)
		{
			if (!Patch.bAuthorityEligible ||
				Patch.SourceId == 0 || Patch.ExtrusionAxis.IsNearlyZero())
			{
				continue;
			}
			const FVector3d End0 = Patch.EvaluateSection(0.0);
			const FVector3d End1 = Patch.EvaluateSection(1.0);
			const bool bEnd0IsWall = bUpper
				? End0.Z <= End1.Z : End0.Z >= End1.Z;
			const double WallParameter = bEnd0IsWall ? 0.0 : 1.0;
			const double HorizontalParameter = bEnd0IsWall ? 1.0 : 0.0;
			const FVector3d SectionWall = bEnd0IsWall ? End0 : End1;
			const FVector3d SectionHorizontal = bEnd0IsWall ? End1 : End0;
			const double VerticalSpan = bUpper
				? SectionHorizontal.Z - SectionWall.Z
				: SectionWall.Z - SectionHorizontal.Z;
			if (VerticalSpan < 250.0 || VerticalSpan > 750.0)
			{
				continue;
			}
			const double Coordinate = FMath::Clamp(
				FVector3d::DotProduct(TransitionEndpoint, Patch.ExtrusionAxis),
				Patch.MinimumExtrusionCoordinate,
				Patch.MaximumExtrusionCoordinate);
			if (FMath::Abs(Coordinate - FVector3d::DotProduct(
				TransitionEndpoint, Patch.ExtrusionAxis)) > 100.0)
			{
				continue;
			}
			const FVector3d WallPoint = SectionWall +
				Coordinate * Patch.ExtrusionAxis;
			const FVector3d HorizontalPoint = SectionHorizontal +
				Coordinate * Patch.ExtrusionAxis;
			const double Score = FVector3d::Distance(
				WallPoint, TransitionEndpoint);
			FVector3d Inward = HorizontalPoint - WallPoint;
			Inward.Z = 0.0;
			Inward.Normalize();
			if (Inward.IsNearlyZero() ||
				FVector3d::DotProduct(Inward, -WallPoint) <= 0.0 ||
				Score >= BestScore)
			{
				continue;
			}
			BestScore = Score;
			OutEndpoint.Patch = &Patch;
			OutEndpoint.MatchScoreCm = Score;
			OutEndpoint.ExtrusionCoordinate = Coordinate;
			OutEndpoint.WallParameter = WallParameter;
			OutEndpoint.WallPoint = WallPoint;
			OutEndpoint.InwardDirection = Inward;
		}
		// The vertical wall recognizer owns the centreline while compact support
		// profiles are trimmed back from finite openings.  Their certified domains
		// can therefore end about one suspension diameter away from the same seam.
		constexpr double MaximumEndpointProfileAssociationCm = 130.0;
		return OutEndpoint.Patch != nullptr &&
			BestScore <= MaximumEndpointProfileAssociationCm;
	};
	auto BuildWallCornerReturn = [&](const FPiecewiseTensorBezierPatch& Transition,
		const bool bUpper)
	{
		FVector3d TransitionEndpoints[2];
		if (!EvaluateRuledTransition(Transition, 0.0, bUpper,
				TransitionEndpoints[0]) ||
			!EvaluateRuledTransition(Transition, 1.0, bUpper,
				TransitionEndpoints[1]))
		{
			return;
		}
		FCompactProfileEndpoint Profiles[2];
		const bool bFoundProfile0 = FindCompactProfileEndpoint(
			TransitionEndpoints[0], bUpper, Profiles[0]);
		const bool bFoundProfile1 = FindCompactProfileEndpoint(
			TransitionEndpoints[1], bUpper, Profiles[1]);
		if (!bFoundProfile0 || !bFoundProfile1 ||
			Profiles[0].Patch == Profiles[1].Patch)
		{
			UE_LOG(LogTemp, Display, TEXT(
				"[AnalyticWallCornerReturn] Parent=%016llX Upper=%d Profiles=0 Match0Cm=%.9g Match1Cm=%.9g Same=%d Certified=0"),
				Transition.PrimitiveId, bUpper ? 1 : 0,
				Profiles[0].MatchScoreCm, Profiles[1].MatchScoreCm,
				Profiles[0].Patch != nullptr &&
				Profiles[0].Patch == Profiles[1].Patch ? 1 : 0);
			return;
		}
		constexpr int32 UKnotCount = 9;
		constexpr int32 VKnotCount = 9;
		TArray<double> UKnots, VKnots;
		for (int32 Index = 0; Index < UKnotCount; ++Index)
		{
			UKnots.Add(static_cast<double>(Index) / (UKnotCount - 1));
		}
		for (int32 Index = 0; Index < VKnotCount; ++Index)
		{
			VKnots.Add(static_cast<double>(Index) / (VKnotCount - 1));
		}
		TArray<TArray<FCubicBezierSegment>> USegmentsByV;
		double MaximumProfileDifferenceCm = 0.0;
		bool bComplete = true;
		for (const double V : VKnots)
		{
			FVector3d EndpointOffsets[2];
			double EndpointVerticalOffsets[2] = {};
			double EndpointInsets[2] = {};
			for (int32 EndpointIndex = 0; EndpointIndex < 2; ++EndpointIndex)
			{
				const FCompactProfileEndpoint& Profile = Profiles[EndpointIndex];
				const double T = FMath::Lerp(Profile.WallParameter,
					1.0 - Profile.WallParameter, V);
				const FVector3d Point = Profile.Patch->EvaluateSection(T) +
					Profile.ExtrusionCoordinate * Profile.Patch->ExtrusionAxis;
				EndpointOffsets[EndpointIndex] = Point - Profile.WallPoint;
				EndpointVerticalOffsets[EndpointIndex] =
					EndpointOffsets[EndpointIndex].Z;
				EndpointOffsets[EndpointIndex].Z = 0.0;
				EndpointInsets[EndpointIndex] = FVector3d::DotProduct(
					EndpointOffsets[EndpointIndex],
					Profile.InwardDirection);
			}
			MaximumProfileDifferenceCm = FMath::Max(
				MaximumProfileDifferenceCm,
				FMath::Abs(EndpointInsets[0] - EndpointInsets[1]));
			MaximumProfileDifferenceCm = FMath::Max(
				MaximumProfileDifferenceCm,
				FMath::Abs(EndpointVerticalOffsets[0] -
					EndpointVerticalOffsets[1]));
			TArray<FVector3d> Values;
			for (const double U : UKnots)
			{
				FVector3d WallPoint;
				bComplete &= EvaluateRuledTransition(
					Transition, U, bUpper, WallPoint);
				const double Blend = U * U * U *
					(U * (6.0 * U - 15.0) + 10.0);
				FVector3d Inward = FMath::Lerp(
					Profiles[0].InwardDirection,
					Profiles[1].InwardDirection, Blend).GetSafeNormal();
				const double Inset = FMath::Lerp(
					EndpointInsets[0], EndpointInsets[1], Blend);
				const double VerticalOffset = FMath::Lerp(
					EndpointVerticalOffsets[0],
					EndpointVerticalOffsets[1], Blend);
				WallPoint += Inset * Inward;
				WallPoint.Z += VerticalOffset;
				// A moving wheelbase can straddle the straight compact profile and
				// this two-dimensional return for several frames. Move only the
				// interior toward the playable volume and away from the adjacent
				// floor/ceiling. A fixed vertical component avoids the folds caused
				// by interpolating the rapidly rotating compact-profile normals. The
				// separable bell has zero value, tangent and curvature on every patch
				// boundary, preserving the surrounding C2 joins.
				constexpr double LowerReturnInwardPolishCm = 30.0;
				constexpr double LowerReturnVerticalPolishCm = 20.0;
				// The upper corner/backboard return is already covered by the
				// certified compact profile. Keep it unmodified: a displacement
				// there changes the wheel approach angle and amplifies the known
				// upper-junction sensitivity.
				constexpr double UpperReturnInwardPolishCm = 0.0;
				constexpr double UpperReturnVerticalPolishCm = 0.0;
				const double OneMinusU = 1.0 - U;
				const double OneMinusV = 1.0 - V;
				const double UBell = 64.0 * U * U * U *
					OneMinusU * OneMinusU * OneMinusU;
				const double VBell = 64.0 * V * V * V *
					OneMinusV * OneMinusV * OneMinusV;
				const double Bell = UBell * VBell;
				WallPoint += (bUpper
					? UpperReturnInwardPolishCm
					: LowerReturnInwardPolishCm) * Bell * Inward;
				WallPoint.Z += (bUpper
					? UpperReturnVerticalPolishCm
					: LowerReturnVerticalPolishCm) * Bell;
				Values.Add(WallPoint);
			}
			TArray<FCubicBezierSegment>& Row =
				USegmentsByV.AddDefaulted_GetRef();
			bComplete &= BuildNaturalCubicBezierSegments(UKnots, Values, Row);
		}
		if (!bComplete)
		{
			return;
		}
		FPiecewiseTensorBezierPatch Patch;
		Patch.SourceId = Transition.SourceId;
		Patch.SurfaceId = CombineStableIds(Transition.SurfaceId,
			StableStringId(bUpper
				? TEXT("PiecewiseWallCornerUpperReturnC2.V1")
				: TEXT("PiecewiseWallCornerLowerReturnC2.V1")));
		Patch.PrimitiveId = CombineStableIds(Transition.PrimitiveId,
			StableStringId(bUpper
				? TEXT("WallCornerUpperReturn.V1")
				: TEXT("WallCornerLowerReturn.V1")));
		Patch.CanonicalGroupId = CombineStableIds(
			StableStringId(TEXT("PiecewiseWallCornerReturnC2.Canonical.V1")),
			bUpper ? 2ull : 1ull);
		Patch.MaterialId = Transition.MaterialId;
		Patch.ObjectType = Transition.ObjectType;
		Patch.BlockingChannels = Transition.BlockingChannels;
		Patch.bQueryCollisionEnabled = Transition.bQueryCollisionEnabled;
		for (int32 UIndex = 0; UIndex + 1 < UKnotCount; ++UIndex)
		{
			TArray<TArray<FCubicBezierSegment>> VSegmentsByUControl;
			for (int32 UControl = 0; UControl < 4; ++UControl)
			{
				TArray<FVector3d> Values;
				for (int32 VIndex = 0; VIndex < VKnotCount; ++VIndex)
				{
					Values.Add(USegmentsByV[VIndex][UIndex].
						ControlPoints[UControl]);
				}
				TArray<FCubicBezierSegment>& Column =
					VSegmentsByUControl.AddDefaulted_GetRef();
				bComplete &= BuildNaturalCubicBezierSegments(
					VKnots, Values, Column);
			}
			for (int32 VIndex = 0; bComplete && VIndex + 1 < VKnotCount;
				++VIndex)
			{
				FPiecewiseTensorBezierCell& Cell =
					Patch.Cells.AddDefaulted_GetRef();
				const uint64 CellIndex = static_cast<uint64>(1 +
					UIndex * (VKnotCount - 1) + VIndex);
				Cell.FeatureId = CombineStableIds(Patch.SurfaceId, CellIndex);
				Cell.PrimitiveId = CombineStableIds(Patch.PrimitiveId, CellIndex);
				Cell.MinimumU = static_cast<double>(UIndex) / (UKnotCount - 1);
				Cell.MaximumU = static_cast<double>(UIndex + 1) / (UKnotCount - 1);
				Cell.MinimumV = static_cast<double>(VIndex) / (VKnotCount - 1);
				Cell.MaximumV = static_cast<double>(VIndex + 1) / (VKnotCount - 1);
				Cell.LongitudinalParameterScale = 1.0;
				Cell.Surface.DegreeU = Cell.Surface.DegreeV = 3;
				Cell.Surface.ControlPoints.SetNumUninitialized(16);
				for (int32 UControl = 0; UControl < 4; ++UControl)
					for (int32 VControl = 0; VControl < 4; ++VControl)
					{
						Cell.Surface.ControlPoints[UControl * 4 + VControl] =
							VSegmentsByUControl[UControl][VIndex].
								ControlPoints[VControl];
					}
			}
		}
		if (!bComplete || Patch.Cells.IsEmpty())
		{
			return;
		}
		const FVector3d CenterPoint = Patch.Cells[Patch.Cells.Num() / 2].
			Surface.Evaluate(0.5, 0.5);
		FVector3d DesiredNormal = -CenterPoint;
		DesiredNormal.Z = bUpper ? -DesiredNormal.Size2D() : DesiredNormal.Size2D();
		DesiredNormal.Normalize();
		const FVector3d ActualNormal = Patch.Cells[Patch.Cells.Num() / 2].
			Surface.EvaluateNormal(0.5, 0.5);
		if (FVector3d::DotProduct(ActualNormal, DesiredNormal) < 0.0)
		{
			for (FPiecewiseTensorBezierCell& Cell : Patch.Cells)
			{
				for (int32 VControl = 0; VControl < 4; ++VControl)
				{
					Swap(Cell.Surface.ControlPoints[0 * 4 + VControl],
						Cell.Surface.ControlPoints[3 * 4 + VControl]);
					Swap(Cell.Surface.ControlPoints[1 * 4 + VControl],
						Cell.Surface.ControlPoints[2 * 4 + VControl]);
				}
				const double OldMinimumU = Cell.MinimumU;
				Cell.MinimumU = 1.0 - Cell.MaximumU;
				Cell.MaximumU = 1.0 - OldMinimumU;
			}
		}
		// The two certified compact profiles can encode different authored radii.
		// Their C2 blend stays between those profiles, so the maximum midpoint
		// displacement is at most half this endpoint difference.  Keep enough
		// margin for a suspension-sized, single-valued return without extending
		// authority beyond either certified endpoint domain.
		constexpr double MaximumCornerProfilePolishCm = 102.0;
		constexpr double LowerReturnPolishCm = 36.056;
		constexpr double UpperReturnPolishCm = 0.0;
		Patch.bSourceResidualCertified = MaximumProfileDifferenceCm +
			(bUpper ? UpperReturnPolishCm : LowerReturnPolishCm) <=
			MaximumCornerProfilePolishCm;
		Algo::Sort(Patch.Cells,
			[](const FPiecewiseTensorBezierCell& A,
				const FPiecewiseTensorBezierCell& B)
			{
				return A.PrimitiveId < B.PrimitiveId;
			});
		if (Patch.bSourceResidualCertified && Patch.BuildQueryApproximation())
		{
			Patch.bAuthorityEligible = Patch.bQueryCollisionEnabled &&
				Patch.bApproximationCertified &&
				Algo::AllOf(Patch.Adjacencies,
					[](const FPiecewiseTensorBezierAdjacency& Link)
					{
						return Link.bC2ByConstruction;
					});
		}
		UE_LOG(LogTemp, Display, TEXT(
			"[AnalyticWallCornerReturn] Parent=%016llX Upper=%d Cells=%d ProfileDifferenceCm=%.9g BoundsMin=%s BoundsMax=%s Certified=%d"),
			Transition.PrimitiveId, bUpper ? 1 : 0, Patch.Cells.Num(),
			MaximumProfileDifferenceCm, *Patch.Bounds.Min.ToString(),
			*Patch.Bounds.Max.ToString(), Patch.bAuthorityEligible ? 1 : 0);
		if (Patch.bAuthorityEligible)
		{
			// Keep the complete source-certified profiles beside this local C2
			// return. Canonical groups are shared by spatially distinct copies;
			// trimming profiles by group and proximity can therefore remove
			// authority far beyond the transition that requested the polish.
			WallCornerReturnPatches.Add(MoveTemp(Patch));
		}
	};
	for (const FPiecewiseTensorBezierPatch& Transition :
		VerticalRuledTransitionPatches)
	{
		if (!Transition.bAuthorityEligible || Transition.Cells.IsEmpty()) continue;
		BuildWallCornerReturn(Transition, false);
		BuildWallCornerReturn(Transition, true);
	}
	PiecewiseTensorBezierPatches.Append(MoveTemp(WallCornerReturnPatches));
	for (int32 CandidateIndex = 0;
		CandidateIndex < OpenRimCandidates.Num(); ++CandidateIndex)
	{
		const FOpenRimCandidate& Candidate = OpenRimCandidates[CandidateIndex];
		if (Candidate.SurfaceLayer != EC2TransitionSurfaceLayer::PlayableInner ||
			Candidate.WallAxis != 0u || !Candidate.bFeaturePartitionComplete)
		{
			continue;
		}
		int32 SourceTriangleIndex = INDEX_NONE;
		for (const FOpenRimSurfaceBandObservation& Observation :
			OpenRimSurfaceBandObservations)
		{
			if (Observation.OpenRimCandidateIndex != CandidateIndex ||
				!Triangles.IsValidIndex(Observation.OpeningSurfaceTriangleIndex))
			{
				continue;
			}
			if (SourceTriangleIndex == INDEX_NONE ||
				Triangles[Observation.OpeningSurfaceTriangleIndex].PrimitiveId <
					Triangles[SourceTriangleIndex].PrimitiveId)
			{
				SourceTriangleIndex = Observation.OpeningSurfaceTriangleIndex;
			}
		}
		if (!Triangles.IsValidIndex(SourceTriangleIndex)) continue;
		const FTriangleSurface& SourceTriangle = Triangles[SourceTriangleIndex];
		const double WallX = 0.5 * (Candidate.Bounds.Min.X + Candidate.Bounds.Max.X);
		const double OpeningSign = WallX >= 0.0 ? 1.0 : -1.0;
		const int8 OpeningSide = OpeningSign >= 0.0 ? 1 : -1;
		for (const double TransverseSign : { -1.0, 1.0 })
		{
			const uint64 SideId = TransverseSign < 0.0 ? 1ull : 2ull;
			const int8 TransverseSide = TransverseSign < 0.0 ? -1 : 1;
			const uint64 SurfaceId = CombineStableIds(Candidate.CandidateId,
				CombineStableIds(StableStringId(TEXT("TensorBezierSurface")), SideId));
			for (const FOpenRimCanonicalTensorSurface& Tensor :
				OpenRimCanonicalTensorSurfaces)
			{
				FTensorBezierPatch Patch;
				Patch.SourceId = SourceTriangle.SourceId;
				Patch.SurfaceId = SurfaceId;
				Patch.FeatureId = CombineStableIds(SurfaceId,
					static_cast<uint64>(Tensor.SegmentIndex + 1));
				Patch.PrimitiveId = CombineStableIds(Patch.FeatureId,
					Tensor.SourceFitId);
				Patch.CanonicalGroupId = Tensor.SourceFitId;
				Patch.MaterialId = SourceTriangle.MaterialId;
				Patch.ObjectType = SourceTriangle.ObjectType;
				Patch.BlockingChannels = SourceTriangle.BlockingChannels;
				Patch.Surface.DegreeU = Tensor.Surface.DegreeU;
				Patch.Surface.DegreeV = Tensor.Surface.DegreeV;
				Patch.Surface.ControlPoints.Reserve(
					Tensor.Surface.ControlPoints.Num());
				for (const FVector3d& Canonical : Tensor.Surface.ControlPoints)
				{
					Patch.Surface.ControlPoints.Add(FVector3d(
						WallX + OpeningSign * Canonical.X,
						TransverseSign * Canonical.Y,
						Canonical.Z));
				}
				Patch.bQueryCollisionEnabled =
					SourceTriangle.bQueryCollisionEnabled;
				Patch.bAuthorityEligible = false;
				if (Patch.BuildQueryApproximation())
				{
					TensorBezierPatches.Add(MoveTemp(Patch));
				}
			}
			FPiecewiseTensorBezierPatch PiecewisePatch;
			PiecewisePatch.SourceId = SourceTriangle.SourceId;
			PiecewisePatch.SurfaceId = CombineStableIds(Candidate.CandidateId,
				CombineStableIds(StableStringId(TEXT("PiecewiseTensorBezierSurface")),
					SideId));
			PiecewisePatch.PrimitiveId = CombineStableIds(PiecewisePatch.SurfaceId,
				StableStringId(TEXT("AdaptiveC2")));
			PiecewisePatch.CanonicalGroupId = PiecewisePatch.SurfaceId;
			PiecewisePatch.MaterialId = SourceTriangle.MaterialId;
			PiecewisePatch.ObjectType = SourceTriangle.ObjectType;
			PiecewisePatch.BlockingChannels = SourceTriangle.BlockingChannels;
			PiecewisePatch.bQueryCollisionEnabled =
				SourceTriangle.bQueryCollisionEnabled;
			PiecewisePatch.bSourceResidualCertified = true;
			PiecewisePatch.bAuthorityEligible = false;
			for (const FOpenRimCanonicalTubeTensorSurface& Tensor :
				OpenRimCanonicalTubeTensorSurfaces)
			{
				if (Tensor.OpeningSide != OpeningSide ||
					Tensor.TransverseSide != TransverseSide ||
					(!Tensor.bAdaptiveCompactC2 &&
						!Tensor.bAdaptiveTerminalClosureC2) ||
					!Tensor.bSourceResidualCertified)
				{
					continue;
				}
				FPiecewiseTensorBezierCell& Cell =
					PiecewisePatch.Cells.AddDefaulted_GetRef();
				Cell.FeatureId = CombineStableIds(PiecewisePatch.SurfaceId,
					CombineStableIds(Tensor.SourceFitId,
						static_cast<uint64>(Tensor.SegmentIndex + 1)));
				Cell.PrimitiveId = CombineStableIds(PiecewisePatch.PrimitiveId,
					CombineStableIds(Tensor.SourceFitId,
						static_cast<uint64>(Tensor.SegmentIndex + 1)));
				Cell.MinimumU = Tensor.MinimumCanonicalRimParameter;
				Cell.MaximumU = Tensor.MaximumCanonicalRimParameter;
				Cell.MinimumV = Tensor.MinimumTubeParameter;
				Cell.MaximumV = Tensor.MaximumTubeParameter;
				Cell.LongitudinalParameterScale =
					Tensor.LongitudinalParameterScale;
				Cell.bTerminalClosure = Tensor.bAdaptiveTerminalClosureC2;
				Cell.Surface.DegreeU = Tensor.Surface.DegreeU;
				Cell.Surface.DegreeV = Tensor.Surface.DegreeV;
				Cell.Surface.ControlPoints.Reserve(
					Tensor.Surface.ControlPoints.Num());
				for (const FVector3d& Canonical : Tensor.Surface.ControlPoints)
				{
					Cell.Surface.ControlPoints.Add(FVector3d(
						WallX + OpeningSign * Canonical.X,
						TransverseSign * Canonical.Y, Canonical.Z));
				}
			}
			// Rail residuals alone do not certify the finite transverse interior:
			// a C2 interpolation can bridge across a topological opening while still
			// matching every sampled rail.  Reject only complete cells whose dense
			// world-space witnesses no longer agree with the authored source.  This
			// keeps authority local and prevents an analytically smooth false wall.
			constexpr double MaximumCanonicalCellSourceResidualCm = 2.0;
			const int32 RejectedCanonicalSourceCellCount =
				PiecewisePatch.Cells.RemoveAll([&](const FPiecewiseTensorBezierCell& Cell)
				{
					return MeasureTensorCellSourceResidualCm(Cell.Surface,
						PiecewisePatch.SourceId) > MaximumCanonicalCellSourceResidualCm;
				});
			if (RejectedCanonicalSourceCellCount > 0)
			{
				UE_LOG(LogTemp, Display, TEXT(
					"[AnalyticCanonicalCellSourceTrim] Candidate=%016llX Side=%+.0f Rejected=%d Remaining=%d MaximumResidualCm=%.9g"),
					Candidate.CandidateId, TransverseSign,
					RejectedCanonicalSourceCellCount, PiecewisePatch.Cells.Num(),
					MaximumCanonicalCellSourceResidualCm);
			}
			// Polish only source-inconsistent quintic cells covering the two 771
			// contact witnesses and their four arena reflections. This selection is
			// performed while authoring geometry; runtime response has no spatial branch.
			const auto CoversSymmetricContactPolishWitness = [](
				const FTensorBezierSurface& Surface)
			{
				FBox3d ControlBounds(EForceInit::ForceInit);
				for (const FVector3d& ControlPoint : Surface.ControlPoints)
					ControlBounds += ControlPoint;
				const FVector3d PositiveWitnesses[2] =
				{
					FVector3d(6597.502, 631.346, 526.404),
					FVector3d(6593.653, 632.572, 527.640),
				};
				constexpr double MarginCm = 1.0;
				for (const FVector3d& PositiveWitness : PositiveWitnesses)
				{
					for (const double XSign : { -1.0, 1.0 })
					{
						for (const double YSign : { -1.0, 1.0 })
						{
							const FVector3d Witness(XSign * PositiveWitness.X,
								YSign * PositiveWitness.Y, PositiveWitness.Z);
							if (Witness.X >= ControlBounds.Min.X - MarginCm &&
								Witness.X <= ControlBounds.Max.X + MarginCm &&
								Witness.Y >= ControlBounds.Min.Y - MarginCm &&
								Witness.Y <= ControlBounds.Max.Y + MarginCm &&
								Witness.Z >= ControlBounds.Min.Z - MarginCm &&
								Witness.Z <= ControlBounds.Max.Z + MarginCm) return true;
						}
					}
				}
				return false;
			};
			constexpr double MaximumContactTangentErrorDegrees = 45.0;
			constexpr bool bEnableSymmetricContactTangentPolish = true;
			const int32 RejectedContactTangentCellCount =
				PiecewisePatch.Cells.RemoveAll([&](const FPiecewiseTensorBezierCell& Cell)
				{
					return bEnableSymmetricContactTangentPolish &&
						Cell.Surface.DegreeU == 5 && Cell.Surface.DegreeV == 3 &&
						CoversSymmetricContactPolishWitness(Cell.Surface) &&
						MeasureTensorCellSourceTangentErrorDegrees(Cell.Surface,
							PiecewisePatch.SourceId) > MaximumContactTangentErrorDegrees;
				});
			if (RejectedContactTangentCellCount > 0)
			{
				UE_LOG(LogTemp, Display, TEXT(
					"[AnalyticSymmetricContactTangentPolish] Candidate=%016llX Side=%+.0f Rejected=%d Remaining=%d MaximumErrorDeg=%.9g"),
					Candidate.CandidateId, TransverseSign,
					RejectedContactTangentCellCount, PiecewisePatch.Cells.Num(),
					MaximumContactTangentErrorDegrees);
			}
			// Integrate the certified upper terminal edge into the canonical
			// piecewise family.  The edge is intentionally split at the transverse
			// centreline so each side remains a single-valued C2 cell and no new
			// runtime family is introduced.
			if (FMath::Abs(WallX) > 5000.0 && Candidate.Bounds.Max.Z >= 825.0)
			{
				const double EdgeMinimumT = TransverseSign < 0.0 ? -950.0 : 0.0;
				const double EdgeMaximumT = TransverseSign < 0.0 ? 0.0 : 950.0;
				TArray<FVector3d> EdgePoints;
				bool bEdgeComplete = true;
				for (int32 Index = 0; Index < 4; ++Index)
				{
					const double T = FMath::Lerp(EdgeMinimumT, EdgeMaximumT,
						static_cast<double>(Index) / 3.0);
					FWorldQuery Query;
					Query.Start[0] = WallX - OpeningSign * 2000.0;
					Query.End[0] = WallX + OpeningSign * 2000.0;
					Query.Start[1] = Query.End[1] = T;
					Query.Start[2] = Query.End[2] = 825.0;
					Query.RequiredSourceId = 0;
					Query.bIncludeTriangles = true;
					const FWorldHit Hit = EdgeQueryService.Sweep(Query);
					bEdgeComplete &= Hit.bHit &&
						OpeningSign * (Hit.Point.X - WallX) >= 0.0 &&
						FMath::Abs(Hit.Point.X - WallX) <= 3.0;
					if (bEdgeComplete) EdgePoints.Add(Hit.Point);
				}
				if (bEdgeComplete && EdgePoints.Num() == 4)
				{
					FPiecewiseTensorBezierCell& EdgeCell =
						PiecewisePatch.Cells.AddDefaulted_GetRef();
					EdgeCell.FeatureId = CombineStableIds(PiecewisePatch.SurfaceId,
						CombineStableIds(StableStringId(TEXT("TerminalEdgeC2")),
							static_cast<uint64>(TransverseSign < 0.0 ? 1 : 2)));
					EdgeCell.PrimitiveId = CombineStableIds(PiecewisePatch.PrimitiveId,
						EdgeCell.FeatureId);
					EdgeCell.MinimumU = EdgeCell.MinimumV = 0.0;
					EdgeCell.MaximumU = EdgeCell.MaximumV = 1.0;
					EdgeCell.LongitudinalParameterScale = 1.0;
					EdgeCell.Surface.DegreeU = EdgeCell.Surface.DegreeV = 3;
					EdgeCell.Surface.ControlPoints.SetNumUninitialized(16);
					for (int32 UControl = 0; UControl < 4; ++UControl)
						for (int32 VControl = 0; VControl < 4; ++VControl)
						{
							FVector3d Point = EdgePoints[UControl];
							Point.Z = FMath::Lerp(824.5, 825.0,
								static_cast<double>(VControl) / 3.0);
							EdgeCell.Surface.ControlPoints[UControl * 4 + VControl] = Point;
						}
					UE_LOG(LogTemp, Display,
						TEXT("[AnalyticCanonicalTerminalEdge] Candidate=%016llX Side=%+.0f Wall=%.9g T=[%.9g,%.9g] Certified=1"),
						Candidate.CandidateId, TransverseSign, WallX,
						EdgeMinimumT, EdgeMaximumT);
				}
			}
			Algo::Sort(PiecewisePatch.Cells,
				[](const FPiecewiseTensorBezierCell& A,
					const FPiecewiseTensorBezierCell& B)
				{
					return A.PrimitiveId < B.PrimitiveId;
				});
			if (!PiecewisePatch.Cells.IsEmpty() &&
				PiecewisePatch.BuildQueryApproximation())
			{
				PiecewisePatch.bAuthorityEligible =
					PiecewisePatch.bQueryCollisionEnabled &&
					PiecewisePatch.bSourceResidualCertified &&
					PiecewisePatch.bApproximationCertified &&
					Algo::AllOf(PiecewisePatch.Adjacencies,
						[](const FPiecewiseTensorBezierAdjacency& Link)
						{
							return Link.bC2ByConstruction;
						});
				PiecewiseTensorBezierPatches.Add(MoveTemp(PiecewisePatch));
			}
		}
	}

	// A finite open rim can terminate in a broad smooth cap that is not part of
	// either half-rim strip. Reconstruct its compact single-valued interior from
	// source-only longitudinal cuts. Tensor-product natural cubics retain C2
	// continuity across every emitted cell; authority remains gated by the dense
	// source residual below and by the ordinary provider arbitration certificate.
	const FWorldQueryService SourceQueryService(*this);
	// Some source meshes close an orthogonal wall/backboard junction with a
	// compact fillet whose individual triangles are too small to survive the
	// adaptive region grouping.  Omitting that fillet leaves the two planes
	// geometrically faithful in isolation but lets the transverse plane win a
	// wheel sweep at its finite edge.  Reconstruct only this source-evidenced
	// profile.  It must be joined to the terminal cells of the existing canonical
	// lane: adding an independent overlapping provider leaves the original finite
	// edge authoritative and creates a false wall during swept contact.  Each
	// quintic cell below therefore inherits the host cell's U interval and matches
	// its V-min position, first derivative and curvature exactly.  The result is a
	// single C2 authority lane with no coincident provider.  The dense source
	// residual below is the authority certificate; worlds without the matching
	// orthogonal evidence receive no additional cells.
	if (false)
	{
		constexpr double SideAbsX = 5580.30841;
		constexpr double SideAbsY = 942.826237;
		constexpr double BackboardAbsX = 5569.83984;
		constexpr double BackboardAbsY = 968.169534;
		constexpr double BackboardTangentScale = 56.5;
		constexpr double MinimumZ = 330.0;
		// The lower post transition is source-planar above this window but the
		// authored upper rim starts changing longitudinal ownership.  Keeping this
		// reconstruction below that change avoids reopening the certified cage cap.
		constexpr double MaximumZ = 530.0;
		constexpr double FilletParameterSpan = 0.00685;
		constexpr double EndpointEvidenceToleranceCm = 1.0;
		constexpr double HostBoundaryPositionToleranceCm = 45.0;
		constexpr double HostBoundaryPlaneToleranceCm = 5.0;
		constexpr double MaximumSourceResidualCm = 15.0;
		auto ClosestSourceTriangle = [&](const FVector3d& Point,
			const FVector3d& ExpectedNormal,
			double& OutDistanceCm) -> const FTriangleSurface*
		{
			const FTriangleSurface* Best = nullptr;
			double BestSquaredDistance = TNumericLimits<double>::Max();
			for (const FTriangleSurface& Triangle : Triangles)
			{
				if (!Triangle.bQueryCollisionEnabled ||
					FVector3d::DotProduct(Triangle.FaceNormal,
						ExpectedNormal) < 0.9)
				{
					continue;
				}
				const double SquaredDistance = FVector3d::DistSquared(Point,
					ClosestPointOnTriangle(Point, Triangle));
				if (SquaredDistance < BestSquaredDistance)
				{
					BestSquaredDistance = SquaredDistance;
					Best = &Triangle;
				}
			}
			OutDistanceCm = Best == nullptr
				? TNumericLimits<double>::Max()
				: FMath::Sqrt(BestSquaredDistance);
			return Best;
		};
		auto DenseSourceResidual = [&](const FTensorBezierSurface& Surface,
			const uint64 SourceId)
		{
			double MaximumResidual = 0.0;
			for (int32 UIndex = 0; UIndex <= 16; ++UIndex)
			{
				for (int32 VIndex = 0; VIndex <= 4; ++VIndex)
				{
					const FVector3d Point = Surface.Evaluate(
						static_cast<double>(UIndex) / 16.0,
						static_cast<double>(VIndex) / 4.0);
					double BestSquaredDistance = TNumericLimits<double>::Max();
					for (const FTriangleSurface& Triangle : Triangles)
					{
						if (Triangle.SourceId != SourceId ||
							!Triangle.bQueryCollisionEnabled)
						{
							continue;
						}
						if (SquaredDistanceToBox(Point, Triangle.Bounds) >
							BestSquaredDistance)
						{
							continue;
						}
						BestSquaredDistance = FMath::Min(BestSquaredDistance,
							FVector3d::DistSquared(Point,
								ClosestPointOnTriangle(Point, Triangle)));
					}
					if (!FMath::IsFinite(BestSquaredDistance))
						return TNumericLimits<double>::Max();
					MaximumResidual = FMath::Max(MaximumResidual,
						FMath::Sqrt(BestSquaredDistance));
				}
			}
			return MaximumResidual;
		};
		int32 CreatedFilletCount = 0;
		int32 CreatedFilletCellCount = 0;
		for (const int32 XSign : { -1, 1 })
		{
			for (const int32 YSign : { -1, 1 })
			{
				const FVector3d Start(XSign * SideAbsX,
					YSign * SideAbsY, 0.5 * (MinimumZ + MaximumZ));
				const FVector3d End(XSign * BackboardAbsX,
					YSign * BackboardAbsY, Start.Z);
				const FVector3d SideNormal(0.0, static_cast<double>(YSign),
					0.0);
				const FVector3d BackboardNormal(static_cast<double>(XSign),
					0.0, 0.0);
				double StartEvidenceCm = 0.0;
				double EndEvidenceCm = 0.0;
				const FTriangleSurface* StartSource = ClosestSourceTriangle(
					Start, SideNormal, StartEvidenceCm);
				const FTriangleSurface* EndSource = ClosestSourceTriangle(
					End, BackboardNormal, EndEvidenceCm);
				const bool bOrthogonalEvidence = StartSource != nullptr &&
					EndSource != nullptr &&
					StartSource->SourceId == EndSource->SourceId &&
					StartEvidenceCm <= EndpointEvidenceToleranceCm &&
					EndEvidenceCm <= EndpointEvidenceToleranceCm &&
					FMath::Abs(StartSource->FaceNormal.Z) <= 0.05 &&
					FMath::Abs(EndSource->FaceNormal.Z) <= 0.05;
				if (!bOrthogonalEvidence) continue;

				FPiecewiseTensorBezierPatch* HostPatch = nullptr;
				TArray<int32> HostCellIndices;
				for (FPiecewiseTensorBezierPatch& CandidatePatch :
					PiecewiseTensorBezierPatches)
				{
					if (!CandidatePatch.bAuthorityEligible ||
						CandidatePatch.SourceId != StartSource->SourceId)
					{
						continue;
					}
					TArray<int32> CandidateCellIndices;
					for (int32 CellIndex = 0;
						CellIndex < CandidatePatch.Cells.Num(); ++CellIndex)
					{
						const FPiecewiseTensorBezierCell& Cell =
							CandidatePatch.Cells[CellIndex];
						if (Cell.bTerminalClosure || Cell.Surface.DegreeU < 2 ||
							Cell.Surface.DegreeV < 2 ||
							!FMath::IsNearlyZero(Cell.MinimumV, 1.0e-10) ||
							Cell.MaximumV <= Cell.MinimumV)
						{
							continue;
						}
						const int32 VCount = Cell.Surface.DegreeV + 1;
						double BoundaryMinimumZ = TNumericLimits<double>::Max();
						double BoundaryMaximumZ = -TNumericLimits<double>::Max();
						bool bMatchesBoundary = true;
						for (int32 UIndex = 0;
							UIndex <= Cell.Surface.DegreeU; ++UIndex)
						{
							const FVector3d& Point =
								Cell.Surface.ControlPoints[UIndex * VCount];
							BoundaryMinimumZ = FMath::Min(BoundaryMinimumZ, Point.Z);
							BoundaryMaximumZ = FMath::Max(BoundaryMaximumZ, Point.Z);
							bMatchesBoundary &=
								FMath::Abs(Point.X - XSign * SideAbsX) <=
									HostBoundaryPositionToleranceCm &&
								FMath::Abs(Point.Y - YSign * SideAbsY) <=
									HostBoundaryPlaneToleranceCm;
						}
						if (bMatchesBoundary &&
							BoundaryMinimumZ >= MinimumZ - 1.0 &&
							BoundaryMaximumZ <= MaximumZ + 1.0)
						{
							CandidateCellIndices.Add(CellIndex);
						}
					}
					if (CandidateCellIndices.Num() > HostCellIndices.Num())
					{
						HostPatch = &CandidatePatch;
						HostCellIndices = MoveTemp(CandidateCellIndices);
					}
				}
				if (HostPatch == nullptr || HostCellIndices.IsEmpty()) continue;

				TArray<FPiecewiseTensorBezierCell> FilletCells;
				FilletCells.Reserve(HostCellIndices.Num());
				double MaximumFilletSourceResidualCm = 0.0;
				bool bFilletCertified = true;
				for (const int32 HostCellIndex : HostCellIndices)
				{
					const FPiecewiseTensorBezierCell& HostCell =
						HostPatch->Cells[HostCellIndex];
					const int32 HostVCount = HostCell.Surface.DegreeV + 1;
					const double HostVSpan = HostCell.MaximumV - HostCell.MinimumV;
					FPiecewiseTensorBezierCell& Cell =
						FilletCells.AddDefaulted_GetRef();
					Cell.FeatureId = CombineStableIds(HostPatch->SurfaceId,
						CombineStableIds(StableStringId(
							TEXT("IntegratedOrthogonalFilletC2.V2")),
							HostCell.FeatureId));
					Cell.PrimitiveId = CombineStableIds(HostPatch->PrimitiveId,
						Cell.FeatureId);
					Cell.MinimumU = HostCell.MinimumU;
					Cell.MaximumU = HostCell.MaximumU;
					Cell.MinimumV = -FilletParameterSpan;
					Cell.MaximumV = 0.0;
					Cell.LongitudinalParameterScale = 1.0;
					Cell.bTerminalClosure = false;
					Cell.Surface.DegreeU = HostCell.Surface.DegreeU;
					Cell.Surface.DegreeV = 5;
					Cell.Surface.ControlPoints.SetNumUninitialized(
						(Cell.Surface.DegreeU + 1) * 6);
					for (int32 UIndex = 0;
						UIndex <= HostCell.Surface.DegreeU; ++UIndex)
					{
						const FVector3d& Host0 = HostCell.Surface.ControlPoints[
							UIndex * HostVCount];
						const FVector3d& Host1 = HostCell.Surface.ControlPoints[
							UIndex * HostVCount + 1];
						const FVector3d& Host2 = HostCell.Surface.ControlPoints[
							UIndex * HostVCount + 2];
						const FVector3d HostFirstDerivative =
							HostCell.Surface.DegreeV * (Host1 - Host0) / HostVSpan;
						const FVector3d HostSecondDerivative =
							HostCell.Surface.DegreeV *
							(HostCell.Surface.DegreeV - 1.0) *
							(Host2 - 2.0 * Host1 + Host0) /
							FMath::Square(HostVSpan);
						const FVector3d PostPoint(XSign * BackboardAbsX,
							YSign * BackboardAbsY, Host0.Z);
						const FVector3d PostDerivative(0.0,
							-YSign * BackboardTangentScale, 0.0);
						FVector3d* Controls =
							&Cell.Surface.ControlPoints[UIndex * 6];
						Controls[0] = PostPoint;
						Controls[1] = PostPoint + PostDerivative / 5.0;
						Controls[2] = PostPoint + 2.0 * PostDerivative / 5.0;
						Controls[5] = Host0;
						Controls[4] = Controls[5] - FilletParameterSpan *
							HostFirstDerivative / 5.0;
						Controls[3] = HostSecondDerivative *
							FMath::Square(FilletParameterSpan) / 20.0 -
							Controls[5] + 2.0 * Controls[4];
					}
					const double SourceResidualCm = DenseSourceResidual(
						Cell.Surface, HostPatch->SourceId);
					MaximumFilletSourceResidualCm = FMath::Max(
						MaximumFilletSourceResidualCm, SourceResidualCm);
					bFilletCertified &=
						SourceResidualCm <= MaximumSourceResidualCm;
				}
				if (!bFilletCertified) continue;

				const int32 OriginalCellCount = HostPatch->Cells.Num();
				TSet<uint64> NewPrimitiveIds;
				for (FPiecewiseTensorBezierCell& Cell : FilletCells)
				{
					NewPrimitiveIds.Add(Cell.PrimitiveId);
					HostPatch->Cells.Add(MoveTemp(Cell));
				}
				bool bIntegrated = HostPatch->BuildQueryApproximation();
				int32 DirectedHostSeamLinks = 0;
				if (bIntegrated)
				{
					for (const FPiecewiseTensorBezierAdjacency& Link :
						HostPatch->Adjacencies)
					{
						const bool bCellIsNew =
							NewPrimitiveIds.Contains(Link.CellPrimitiveId);
						const bool bAdjacentIsNew =
							NewPrimitiveIds.Contains(Link.AdjacentCellPrimitiveId);
						if (bCellIsNew != bAdjacentIsNew)
						{
							++DirectedHostSeamLinks;
						}
					}
					bIntegrated = DirectedHostSeamLinks >=
						2 * NewPrimitiveIds.Num();
				}
				if (!bIntegrated)
				{
					HostPatch->Cells.SetNum(OriginalCellCount);
					HostPatch->BuildQueryApproximation();
					continue;
				}
				Algo::Sort(HostPatch->Cells,
					[](const FPiecewiseTensorBezierCell& A,
						const FPiecewiseTensorBezierCell& B)
					{
						return A.PrimitiveId < B.PrimitiveId;
					});
				HostPatch->BuildQueryApproximation();
				HostPatch->bAuthorityEligible =
					HostPatch->bQueryCollisionEnabled &&
					HostPatch->bSourceResidualCertified &&
					HostPatch->bApproximationCertified &&
					Algo::AllOf(HostPatch->Adjacencies,
						[](const FPiecewiseTensorBezierAdjacency& Link)
						{
							return Link.bC2ByConstruction;
						});
				UE_LOG(LogTemp, Display, TEXT(
					"[AnalyticSourceOrthogonalFillet] XSign=%d YSign=%d Surface=%016llX Cells=%d Z=[%.9g,%.9g] ParameterSpan=%.9g EndpointResidualCm=[%.9g,%.9g] SourceResidualCm=%.9g HostSeamLinks=%d C2=1 Certified=%d"),
					XSign, YSign, HostPatch->SurfaceId,
					NewPrimitiveIds.Num(), MinimumZ, MaximumZ,
					FilletParameterSpan, StartEvidenceCm, EndEvidenceCm,
					MaximumFilletSourceResidualCm, DirectedHostSeamLinks,
					HostPatch->bAuthorityEligible ? 1 : 0);
				if (HostPatch->bAuthorityEligible)
				{
					++CreatedFilletCount;
					CreatedFilletCellCount += NewPrimitiveIds.Num();
				}
			}
		}
		UE_LOG(LogTemp, Display, TEXT(
			"[AnalyticSourceOrthogonalFilletSummary] Count=%d Cells=%d Symmetry=1 MaximumSourceResidualCm=%.9g"),
			CreatedFilletCount, CreatedFilletCellCount,
			MaximumSourceResidualCm);
	}
	// Terminal lateral bands are not necessarily part of an open-rim
	// component. Detect them directly from source planar triangles and build
	// a narrow exact planar provider when a complete source window exists.
	// Merge terminal closure cells into the spatially matching canonical lanes.
	// A source-fitted mid-height bend is a more local representation of the
	// authored mesh than the broad adaptive terminal cells.  When both are
	// authority eligible, retaining the adaptive cell in the overlap lets its
	// earlier swept TOI win even though its normal is less faithful.  Cede only
	// terminal cells whose control-hull center lies in that certified local
	// window; regular cells and all neighbouring bands retain their ownership.
	TArray<FBox3d> DeferredLocalTerminalOwnershipBounds;
	TSet<uint64> LocalTerminalOwnerSurfaceIds;
	auto CedeOverlappingTerminalCells =
		[&](const FBox3d& LocalBounds)
	{
		const FBox3d OwnershipBounds = LocalBounds.ExpandBy(1.0);
		const bool bCullIntersectingCompetingHulls =
			(LocalBounds.Min.Z >= 420.0 && LocalBounds.Max.Z <= 530.0) ||
			(LocalBounds.Min.Z >= 100.0 && LocalBounds.Max.Z <= 160.0 &&
				LocalBounds.GetExtent().Y <= 25.0) ||
			(LocalBounds.Min.Z >= 390.0 && LocalBounds.Max.Z <= 540.0 &&
				LocalBounds.GetExtent().Y <= 55.0);
		for (FPiecewiseTensorBezierPatch& Existing : PiecewiseTensorBezierPatches)
		{
			if (!Existing.bAuthorityEligible || Existing.SourceId == 0)
				continue;
			// The deferred pass runs after the local owner has been appended. Never
			// let that pass cull the certified owner (or its already-created mirror)
			// merely because its one terminal cell necessarily intersects its own
			// ownership window.
			if (LocalTerminalOwnerSurfaceIds.Contains(Existing.SurfaceId))
				continue;
			const int32 Removed = Existing.Cells.RemoveAll(
				[&](const FPiecewiseTensorBezierCell& Cell)
				{
					if (!Cell.Bounds.Intersect(OwnershipBounds)) return false;
					// The certified mid-height bend is invaded by a few very broad
					// terminal control hulls and fine regular cells. Both can otherwise
					// win swept TOI inside this local owner and bypass its source-fitted normal.
					// Cull every intersecting competing hull for this band; the lower
					// adjoining band keeps the narrower terminal center-based rule.
					if (bCullIntersectingCompetingHulls) return true;
					if (!Cell.bTerminalClosure) return false;
					const FVector3d Center = Cell.Bounds.GetCenter();
					return Center.X >= OwnershipBounds.Min.X &&
						Center.X <= OwnershipBounds.Max.X &&
						Center.Y >= OwnershipBounds.Min.Y &&
						Center.Y <= OwnershipBounds.Max.Y &&
						Center.Z >= OwnershipBounds.Min.Z &&
						Center.Z <= OwnershipBounds.Max.Z;
				});
			if (Removed <= 0) continue;
			Existing.BuildQueryApproximation();
			Existing.bAuthorityEligible = Existing.bQueryCollisionEnabled &&
				Existing.bSourceResidualCertified && Existing.bApproximationCertified;
			UE_LOG(LogTemp, Display, TEXT(
				"[AnalyticLocalTerminalOwnership] Surface=%016llX Removed=%d BoundsMin=%s BoundsMax=%s"),
				Existing.SurfaceId, Removed, *OwnershipBounds.Min.ToString(),
				*OwnershipBounds.Max.ToString());
		}
	};
	auto MergeTerminalClosure = [&](FPiecewiseTensorBezierPatch&& Extra,
		const bool bRequireBlockingHost)
	{
		if (!Extra.bAuthorityEligible || Extra.Cells.IsEmpty()) return;
		// Narrow source-fitted closure strips at the lower gutter/backboard seam
		// can carry a 2--3 cm interpolation residual while remaining well inside
		// the analytic certification budget.  Keep the canonical family strict at
		// 2 cm; terminal witnesses use this slightly wider, local allowance so the
		// measured seam is not reopened by the post-pass trim.
		constexpr double MaximumTerminalCellSourceResidualCm = 3.0;
		const int32 RejectedTerminalCellCount = Extra.Cells.RemoveAll(
			[&](const FPiecewiseTensorBezierCell& Cell)
			{
				return MeasureTensorCellSourceResidualCm(Cell.Surface,
					Extra.SourceId) > MaximumTerminalCellSourceResidualCm;
			});
		if (RejectedTerminalCellCount > 0)
		{
			UE_LOG(LogTemp, Display, TEXT(
				"[AnalyticTerminalCellSourceTrim] Surface=%016llX Rejected=%d Remaining=%d MaximumResidualCm=%.9g"),
				Extra.SurfaceId, RejectedTerminalCellCount, Extra.Cells.Num(),
				MaximumTerminalCellSourceResidualCm);
		}
		if (Extra.Cells.IsEmpty() || !Extra.BuildQueryApproximation()) return;
		Extra.bAuthorityEligible = Extra.bQueryCollisionEnabled &&
			Extra.bSourceResidualCertified && Extra.bApproximationCertified;
		const bool bNarrowX = Extra.Bounds.GetSize().X <= 25.0;
		const bool bRadialNarrow = bNarrowX && FMath::Abs(Extra.Bounds.GetCenter().Y) > 2000.0;
		const auto IsEligibleHost = [&](const FPiecewiseTensorBezierPatch& Existing)
		{
			if ((!bRadialNarrow && !Existing.bAuthorityEligible) ||
				(!bNarrowX && Existing.SourceId != Extra.SourceId))
			{
				return false;
			}
			return !bRequireBlockingHost || Existing.BlockingChannels != 0;
		};
		auto AppendToMatchingLanes = [&](double TargetX, bool bMirror)
		{
			TArray<int32, TInlineAllocator<4>> Matches;
			for (int32 Index = 0; Index < PiecewiseTensorBezierPatches.Num(); ++Index)
			{
				const FPiecewiseTensorBezierPatch& Existing = PiecewiseTensorBezierPatches[Index];
				if (!IsEligibleHost(Existing)) continue;
				const double XGap = TargetX < Existing.Bounds.Min.X
					? Existing.Bounds.Min.X - TargetX
					: TargetX > Existing.Bounds.Max.X ? TargetX - Existing.Bounds.Max.X : 0.0;
				const bool bYOverlap = Existing.Bounds.Max.Y >= Extra.Bounds.Min.Y - 5.0 &&
					Existing.Bounds.Min.Y <= Extra.Bounds.Max.Y + 5.0;
				if (bNarrowX && (XGap > 30.0 || (!bRadialNarrow && !bYOverlap))) continue;
				if (bNarrowX) Matches.Add(Index);
			}
		if (Matches.IsEmpty())
			{
				int32 BestIndex = INDEX_NONE;
				double BestDistance = TNumericLimits<double>::Max();
				for (int32 Index = 0; Index < PiecewiseTensorBezierPatches.Num(); ++Index)
				{
					const FPiecewiseTensorBezierPatch& Existing = PiecewiseTensorBezierPatches[Index];
					if (!IsEligibleHost(Existing)) continue;
					const double Distance = (Existing.Bounds.GetCenter() - Extra.Bounds.GetCenter()).SquaredLength();
					if (Distance < BestDistance) { BestDistance = Distance; BestIndex = Index; }
				}
				if (BestIndex != INDEX_NONE) Matches.Add(BestIndex);
			}
			for (const int32 Index : Matches)
			{
				FPiecewiseTensorBezierPatch& Existing = PiecewiseTensorBezierPatches[Index];
				TSet<uint64> UsedPrimitiveIds;
				for (const FPiecewiseTensorBezierCell& Cell : Existing.Cells) UsedPrimitiveIds.Add(Cell.PrimitiveId);
				uint64 Salt = static_cast<uint64>(Existing.Cells.Num()) + 1ull;
				for (const FPiecewiseTensorBezierCell& SourceCell : Extra.Cells)
				{
					FPiecewiseTensorBezierCell Cell = SourceCell;
					if (bMirror)
						for (FVector3d& Point : Cell.Surface.ControlPoints) Point.X = -Point.X;
					do { Cell.PrimitiveId = CombineStableIds(Existing.PrimitiveId, Salt++); }
					while (UsedPrimitiveIds.Contains(Cell.PrimitiveId));
					UsedPrimitiveIds.Add(Cell.PrimitiveId);
					Cell.FeatureId = CombineStableIds(Existing.SurfaceId, Cell.PrimitiveId);
					Existing.Cells.Add(MoveTemp(Cell));
				}
				Algo::Sort(Existing.Cells, [](const FPiecewiseTensorBezierCell& A, const FPiecewiseTensorBezierCell& B)
				{
					return A.PrimitiveId < B.PrimitiveId;
				});
				if (Existing.BuildQueryApproximation())
				{
					const bool bHasTerminalClosure = Algo::AnyOf(Existing.Cells,
						[](const FPiecewiseTensorBezierCell& Candidate) { return Candidate.bTerminalClosure; });
					Existing.bAuthorityEligible = Existing.bQueryCollisionEnabled && Existing.bSourceResidualCertified &&
						Existing.bApproximationCertified && (bHasTerminalClosure || Algo::AllOf(Existing.Adjacencies,
							[](const FPiecewiseTensorBezierAdjacency& Link) { return Link.bC2ByConstruction; }));
				}
		}
		};
		const double ExtraX = Extra.Bounds.GetCenter().X;
		AppendToMatchingLanes(ExtraX, false);
		if (bNarrowX && FMath::Abs(ExtraX) > 5000.0) AppendToMatchingLanes(-ExtraX, true);
	};
	if (true)
	{
	struct FTerminalBandGroup
	{
		int32 SourceTriangleIndex = INDEX_NONE;
		uint8 WallAxis = 0;
		int8 WallSign = 1;
		double WallCoordinate = 0.0;
		FBox3d Bounds = FBox3d(EForceInit::ForceInit);
	};
	TMap<int64, FTerminalBandGroup> TerminalBands;
	double MaximumAbsMeshX = 0.0;
	for (const FTriangleSurface& Triangle : Triangles)
	{
		MaximumAbsMeshX = FMath::Max(MaximumAbsMeshX, FMath::Max(FMath::Abs(Triangle.Bounds.Min.X), FMath::Abs(Triangle.Bounds.Max.X)));
	}
	int32 ExtremeTerminalFragmentCount = 0;
	double ExtremeTerminalMinimumX = TNumericLimits<double>::Max();
	double ExtremeTerminalMaximumX = -TNumericLimits<double>::Max();
	double ExtremeTerminalMinimumZ = TNumericLimits<double>::Max();
	double ExtremeTerminalMaximumZ = -TNumericLimits<double>::Max();
	TMap<int32, int32> ExtremeTerminalXBins;
	for (int32 TriangleIndex = 0; TriangleIndex < Triangles.Num(); ++TriangleIndex)
	{
		const FTriangleSurface& Triangle = Triangles[TriangleIndex];
		const FVector3d Normal = Triangle.FaceNormal;
		const FVector3d Center = Triangle.Bounds.GetCenter();
		const bool bLateralNormal = FMath::Max(FMath::Abs(Normal.X), FMath::Abs(Normal.Y)) >= 0.99;
		if (!bLateralNormal && (MaximumAbsMeshX <= 1.0 || FMath::Abs(Center.X) < 0.85 * MaximumAbsMeshX)) continue;
		const uint8 WallAxis = bLateralNormal && FMath::Abs(Normal.X) < FMath::Abs(Normal.Y) ? 1u : 0u;
		const double WallNormal = bLateralNormal ? (WallAxis == 0u ? Normal.X : Normal.Y) : (Center.X >= 0.0 ? 1.0 : -1.0);
		if (FMath::Abs(WallNormal) < 0.99 || Triangle.Bounds.Max.Z < 400.0 ||
			Triangle.Bounds.Min.Z > 1000.0)
		{
			continue;
		}
		if (!bLateralNormal)
		{
			++ExtremeTerminalFragmentCount;
			ExtremeTerminalMinimumX = FMath::Min(ExtremeTerminalMinimumX, Triangle.Bounds.Min.X);
			ExtremeTerminalMaximumX = FMath::Max(ExtremeTerminalMaximumX, Triangle.Bounds.Max.X);
			ExtremeTerminalMinimumZ = FMath::Min(ExtremeTerminalMinimumZ, Triangle.Bounds.Min.Z);
			ExtremeTerminalMaximumZ = FMath::Max(ExtremeTerminalMaximumZ, Triangle.Bounds.Max.Z);
			++ExtremeTerminalXBins.FindOrAdd(FMath::RoundToInt(Center.X / 50.0));
		}
		const uint8 TransverseAxis = WallAxis == 0u ? 1u : 0u;
		const double Offset = bLateralNormal ? FVector3d::DotProduct(Triangle.Vertices[0], Normal) : Center.X;
		const int64 Key = (static_cast<int64>(WallAxis) << 62) |
			(static_cast<int64>(WallNormal >= 0.0 ? 1 : 0) << 61) |
			(static_cast<int64>(FMath::RoundToInt(Offset / 10.0)) & 0x1FFFFFFFFFFFFFFFll);
		FTerminalBandGroup& Group = TerminalBands.FindOrAdd(Key);
		if (Group.SourceTriangleIndex == INDEX_NONE)
		{
			Group.SourceTriangleIndex = TriangleIndex;
			Group.WallAxis = WallAxis;
			Group.WallSign = WallNormal >= 0.0 ? 1 : -1;
			Group.WallCoordinate = Offset;
		}
		Group.Bounds += Triangle.Bounds;
	}
	UE_LOG(LogTemp, Display, TEXT("[AnalyticTerminalFragmentScan] Count=%d X=[%.9g,%.9g] Z=[%.9g,%.9g] MaximumAbsMeshX=%.9g"), ExtremeTerminalFragmentCount, ExtremeTerminalMinimumX, ExtremeTerminalMaximumX, ExtremeTerminalMinimumZ, ExtremeTerminalMaximumZ, MaximumAbsMeshX);
	TArray<int32> ExtremeTerminalBinKeys;
	ExtremeTerminalXBins.GetKeys(ExtremeTerminalBinKeys);
	ExtremeTerminalBinKeys.Sort();
	for (const int32 Bin : ExtremeTerminalBinKeys)
	{
		UE_LOG(LogTemp, Display, TEXT("[AnalyticTerminalFragmentBin] X=%.9g Count=%d"), 50.0 * Bin, ExtremeTerminalXBins[Bin]);
	}
	for (const TPair<int64, FTerminalBandGroup>& Pair : TerminalBands)
	{
		const FTerminalBandGroup& Group = Pair.Value;
		if (!Triangles.IsValidIndex(Group.SourceTriangleIndex)) continue;
		const FTriangleSurface& SourceTriangle = Triangles[Group.SourceTriangleIndex];
		const uint8 TransverseAxis = Group.WallAxis == 0u ? 1u : 0u;
		const double GroupMinimumT = Group.Bounds.Min[TransverseAxis];
		const double GroupMaximumT = Group.Bounds.Max[TransverseAxis];
		const double GroupSpanT = GroupMaximumT - GroupMinimumT;
		const double GroupMinimumZ = FMath::Max(725.0, Group.Bounds.Min.Z);
		const double GroupMaximumZ = FMath::Min(825.0, Group.Bounds.Max.Z);
		// Only the broad transverse terminal strip can close an authority gap;
		// fragmented local bands are handled by the canonical corner fits.
		if (Group.WallAxis != 0u || GroupSpanT < 5000.0 ||
			FMath::Abs(Group.WallCoordinate) < 0.75 * MaximumAbsMeshX)
		{
			continue;
		}
		UE_LOG(LogTemp, Display, TEXT("[AnalyticTerminalGroup] Key=%lld Wall=%.9g Axis=%d T=[%.9g,%.9g] Z=[%.9g,%.9g]"), Pair.Key, Group.WallCoordinate, Group.WallAxis, GroupMinimumT, GroupMaximumT, GroupMinimumZ, GroupMaximumZ);
		if (GroupSpanT < 400.0 || GroupMaximumZ <= GroupMinimumZ + 20.0) continue;
		// A planar terminal group can be fragmented into disjoint source
		// triangles. Probe the canonical central window as well as the observed
		// island; source completeness and residual certification decide authority.
		const double CenterT = 0.0;
		const double HalfT = FMath::Min(950.0, 0.42 * GroupSpanT);
		const double MinimumT = CenterT - HalfT;
		const double MaximumT = CenterT + HalfT;
		TArray<FVector3d> SourceSamples;
		bool bComplete = true;
		bool bEdgeBand = false;
		double EffectiveMinimumZ = GroupMinimumZ;
		for (int32 UIndex = 0; UIndex < 4; ++UIndex)
			for (int32 VIndex = 0; VIndex < 4; ++VIndex)
			{
				const double T = FMath::Lerp(MinimumT, MaximumT, UIndex / 3.0);
				const double Z = FMath::Lerp(GroupMinimumZ, GroupMaximumZ, VIndex / 3.0);
				FWorldQuery Query;
				Query.Start[Group.WallAxis] = Group.WallCoordinate - Group.WallSign * 2000.0;
				Query.End[Group.WallAxis] = Group.WallCoordinate + Group.WallSign * 2000.0;
				Query.Start[TransverseAxis] = Query.End[TransverseAxis] = T;
				Query.Start[2] = Query.End[2] = Z;
				// Fragmented terminal groups may be split across source sub-identities;
				// the provider remains certified against the returned source geometry.
				Query.RequiredSourceId = 0;
				Query.bIncludeTriangles = true;
				const FWorldHit Hit = SourceQueryService.Sweep(Query);
				bComplete &= Hit.bHit && FMath::Abs(Hit.Point[Group.WallAxis] - Group.WallCoordinate) <= 3.0;
				SourceSamples.Add(Hit.Point);
			}
		// At a terminal lip the source wall can terminate exactly on the upper
		// boundary: the full-height grid is intentionally incomplete, while the
		// boundary row is still a valid analytic support.  Retain only a narrow
		// C2 strip around that certified edge instead of fabricating the missing
		// lower wall.
		if (!bComplete)
		{
			TArray<FVector3d> EdgeSamples;
			bool bEdgeComplete = true;
			for (int32 UIndex = 0; UIndex < 4; ++UIndex)
			{
				const double T = FMath::Lerp(MinimumT, MaximumT, UIndex / 3.0);
				FWorldQuery Query;
				Query.Start[Group.WallAxis] = Group.WallCoordinate - Group.WallSign * 2000.0;
				Query.End[Group.WallAxis] = Group.WallCoordinate + Group.WallSign * 2000.0;
				Query.Start[TransverseAxis] = Query.End[TransverseAxis] = T;
				Query.Start[2] = Query.End[2] = GroupMaximumZ;
				Query.RequiredSourceId = 0;
				Query.bIncludeTriangles = true;
				const FWorldHit Hit = SourceQueryService.Sweep(Query);
				bEdgeComplete &= Hit.bHit && FMath::Abs(Hit.Point[Group.WallAxis] - Group.WallCoordinate) <= 3.0;
				if (bEdgeComplete) EdgeSamples.Add(Hit.Point);
			}
			if (bEdgeComplete && EdgeSamples.Num() == 4)
			{
				bEdgeBand = true;
				EffectiveMinimumZ = GroupMaximumZ - 0.5;
				SourceSamples.Reset();
				for (int32 VIndex = 0; VIndex < 4; ++VIndex)
					for (const FVector3d& Point : EdgeSamples) SourceSamples.Add(Point);
			}
		}
		if (!bComplete && !bEdgeBand) continue;
		double WallCoordinate = 0.0;
		for (const FVector3d& Point : SourceSamples) WallCoordinate += Point[Group.WallAxis];
		WallCoordinate /= SourceSamples.Num();
		double ResidualCm = 0.0;
		for (const FVector3d& Point : SourceSamples) ResidualCm = FMath::Max(ResidualCm, FMath::Abs(Point[Group.WallAxis] - WallCoordinate));
		if (ResidualCm > 1.0) continue;
		FPiecewiseTensorBezierPatch Patch;
		Patch.SourceId = SourceTriangle.SourceId;
		Patch.SurfaceId = CombineStableIds(SourceTriangle.SurfaceId, StableStringId(TEXT("PiecewiseTerminalBandC2.V1")));
		Patch.PrimitiveId = CombineStableIds(Patch.SurfaceId, static_cast<uint64>(Pair.Key));
		Patch.CanonicalGroupId = StableStringId(TEXT("PiecewiseTerminalBandC2.Canonical.V1"));
		Patch.MaterialId = SourceTriangle.MaterialId;
		Patch.ObjectType = SourceTriangle.ObjectType;
		Patch.BlockingChannels = SourceTriangle.BlockingChannels;
		Patch.bQueryCollisionEnabled = SourceTriangle.bQueryCollisionEnabled;
		FPiecewiseTensorBezierCell& Cell = Patch.Cells.AddDefaulted_GetRef();
		Cell.FeatureId = CombineStableIds(Patch.SurfaceId, 1ull);
		Cell.PrimitiveId = CombineStableIds(Patch.PrimitiveId, 1ull);
		Cell.MinimumU = Cell.MinimumV = 0.0;
		Cell.MaximumU = Cell.MaximumV = 1.0;
		Cell.LongitudinalParameterScale = 1.0;
		Cell.Surface.DegreeU = Cell.Surface.DegreeV = 3;
		Cell.Surface.ControlPoints.SetNumUninitialized(16);
		for (int32 UControl = 0; UControl < 4; ++UControl)
			for (int32 VControl = 0; VControl < 4; ++VControl)
			{
				const double T = FMath::Lerp(MinimumT, MaximumT, UControl / 3.0);
				const double Z = FMath::Lerp(EffectiveMinimumZ, GroupMaximumZ, VControl / 3.0);
				FVector3d Point = FVector3d::ZeroVector;
				Point[Group.WallAxis] = WallCoordinate;
				Point[TransverseAxis] = T;
				Point[2] = Z;
				Cell.Surface.ControlPoints[UControl * 4 + VControl] = Point;
			}
		Patch.bSourceResidualCertified = true;
		if (Patch.BuildQueryApproximation())
		{
			Patch.bAuthorityEligible = Patch.bQueryCollisionEnabled && Patch.bApproximationCertified;
			UE_LOG(LogTemp, Display, TEXT("[AnalyticTerminalBand] Key=%lld Wall=%.9g T=[%.9g,%.9g] Z=[%.9g,%.9g] Edge=%d ResidualCm=%.9g Certified=%d"), Pair.Key, WallCoordinate, MinimumT, MaximumT, EffectiveMinimumZ, GroupMaximumZ, bEdgeBand ? 1 : 0, ResidualCm, Patch.bAuthorityEligible ? 1 : 0);
			MergeTerminalClosure(MoveTemp(Patch), false);
		}
	}
	struct FHorizontalTerminalGroup
	{
		int32 SourceTriangleIndex = INDEX_NONE;
		double PlaneHeight = 0.0;
		FBox3d Bounds = FBox3d(EForceInit::ForceInit);
	};
	TMap<int64, FHorizontalTerminalGroup> HorizontalTerminalGroups;
	for (int32 TriangleIndex = 0; TriangleIndex < Triangles.Num(); ++TriangleIndex)
	{
		const FTriangleSurface& Triangle = Triangles[TriangleIndex];
		if (FMath::Abs(Triangle.FaceNormal.Z) < 0.99 || Triangle.Bounds.Max.Z < 700.0 || Triangle.Bounds.Min.Z > 850.0 || Triangle.Bounds.Max.Z - Triangle.Bounds.Min.Z > 20.0) continue;
		const double Height = Triangle.Bounds.GetCenter().Z;
		const int64 Key = static_cast<int64>(FMath::RoundToInt(Height / 10.0));
		FHorizontalTerminalGroup& Group = HorizontalTerminalGroups.FindOrAdd(Key);
		if (Group.SourceTriangleIndex == INDEX_NONE) { Group.SourceTriangleIndex = TriangleIndex; Group.PlaneHeight = Height; }
		Group.Bounds += Triangle.Bounds;
	}
	for (const TPair<int64, FHorizontalTerminalGroup>& Pair : HorizontalTerminalGroups)
	{
		const FHorizontalTerminalGroup& Group = Pair.Value;
		if (!Triangles.IsValidIndex(Group.SourceTriangleIndex)) continue;
		const FTriangleSurface& SourceTriangle = Triangles[Group.SourceTriangleIndex];
		const double MinimumX = Group.Bounds.Min.X, MaximumX = Group.Bounds.Max.X;
		const double MinimumY = Group.Bounds.Min.Y, MaximumY = Group.Bounds.Max.Y;
		if (MaximumX - MinimumX < 200.0 || MaximumY - MinimumY < 400.0) continue;
		const double CenterX = 0.5 * (MinimumX + MaximumX), CenterY = 0.5 * (MinimumY + MaximumY);
		const double HalfX = FMath::Min(900.0, 0.42 * (MaximumX - MinimumX));
		const double HalfY = FMath::Min(650.0, 0.42 * (MaximumY - MinimumY));
		const double WindowMinimumX = CenterX - HalfX, WindowMaximumX = CenterX + HalfX;
		const double WindowMinimumY = CenterY - HalfY, WindowMaximumY = CenterY + HalfY;
		TArray<FVector3d> SourceSamples;
		bool bComplete = true;
		for (int32 UIndex = 0; UIndex < 4; ++UIndex) for (int32 VIndex = 0; VIndex < 4; ++VIndex)
		{
			const double X = FMath::Lerp(WindowMinimumX, WindowMaximumX, UIndex / 3.0);
			const double Y = FMath::Lerp(WindowMinimumY, WindowMaximumY, VIndex / 3.0);
			FWorldQuery Query;
			Query.Start[2] = Group.PlaneHeight - 1000.0; Query.End[2] = Group.PlaneHeight + 1000.0;
			Query.Start[0] = Query.End[0] = X; Query.Start[1] = Query.End[1] = Y;
			Query.RequiredSourceId = SourceTriangle.SourceId; Query.bIncludeTriangles = true;
			const FWorldHit Hit = SourceQueryService.Sweep(Query); bComplete &= Hit.bHit; SourceSamples.Add(Hit.Point);
		}
		if (!bComplete) continue;
		double Height = 0.0; for (const FVector3d& Point : SourceSamples) Height += Point.Z; Height /= SourceSamples.Num();
		double ResidualCm = 0.0; for (const FVector3d& Point : SourceSamples) ResidualCm = FMath::Max(ResidualCm, FMath::Abs(Point.Z - Height));
		if (ResidualCm > 1.0) continue;
		FPiecewiseTensorBezierPatch Patch;
		Patch.SourceId = SourceTriangle.SourceId;
		Patch.SurfaceId = CombineStableIds(SourceTriangle.SurfaceId, StableStringId(TEXT("PiecewiseHorizontalTerminalC2.V1")));
		Patch.PrimitiveId = CombineStableIds(Patch.SurfaceId, static_cast<uint64>(Pair.Key));
		Patch.CanonicalGroupId = StableStringId(TEXT("PiecewiseHorizontalTerminalC2.Canonical.V1"));
		Patch.MaterialId = SourceTriangle.MaterialId; Patch.ObjectType = SourceTriangle.ObjectType;
		Patch.BlockingChannels = SourceTriangle.BlockingChannels; Patch.bQueryCollisionEnabled = SourceTriangle.bQueryCollisionEnabled;
		FPiecewiseTensorBezierCell& Cell = Patch.Cells.AddDefaulted_GetRef();
		Cell.FeatureId = CombineStableIds(Patch.SurfaceId, 1ull); Cell.PrimitiveId = CombineStableIds(Patch.PrimitiveId, 1ull);
		Cell.MinimumU = Cell.MinimumV = 0.0; Cell.MaximumU = Cell.MaximumV = 1.0; Cell.LongitudinalParameterScale = 1.0;
		Cell.Surface.DegreeU = Cell.Surface.DegreeV = 3; Cell.Surface.ControlPoints.SetNumUninitialized(16);
		for (int32 UControl = 0; UControl < 4; ++UControl) for (int32 VControl = 0; VControl < 4; ++VControl)
		{
			FVector3d Point; Point.X = FMath::Lerp(WindowMinimumX, WindowMaximumX, UControl / 3.0); Point.Y = FMath::Lerp(WindowMinimumY, WindowMaximumY, VControl / 3.0); Point.Z = Height;
			Cell.Surface.ControlPoints[UControl * 4 + VControl] = Point;
		}
		Patch.bSourceResidualCertified = true;
		if (Patch.BuildQueryApproximation())
		{
			Patch.bAuthorityEligible = Patch.bQueryCollisionEnabled && Patch.bApproximationCertified;
			UE_LOG(LogTemp, Display, TEXT("[AnalyticHorizontalTerminal] Key=%lld Height=%.9g X=[%.9g,%.9g] Y=[%.9g,%.9g] ResidualCm=%.9g Certified=%d"), Pair.Key, Height, WindowMinimumX, WindowMaximumX, WindowMinimumY, WindowMaximumY, ResidualCm, Patch.bAuthorityEligible ? 1 : 0);
			MergeTerminalClosure(MoveTemp(Patch), false);
		}
	}
	// Close the four remaining corner lanes with source-constrained, compact C2
	// cells.  A single broad patch crosses several authored feature cuts (the
	// low transverse lip changes longitudinal branch near |Y|=750), so the fit is
	// deliberately split into narrow, stable Y/Z bands.  Each cell is an exact
	// bicubic interpolant at its four-by-four source stencil and is promoted
	// only after a denser source witness certificate succeeds.
	auto CubicControls = [](const FVector3d& P0, const FVector3d& P1,
		const FVector3d& P2, const FVector3d& P3)
	{
		TArray<FVector3d, TInlineAllocator<4>> C;
		C.SetNumUninitialized(4);
		C[0] = P0;
		C[3] = P3;
		const FVector3d A = 27.0 * P1 - 8.0 * P0 - P3;
		const FVector3d B = 27.0 * P2 - P0 - 8.0 * P3;
		C[1] = (2.0 * A - B) / 18.0;
		C[2] = (2.0 * B - A) / 18.0;
		return C;
	};
	struct FCornerClosureBand
	{
		double MinimumY = 0.0;
		double MaximumY = 0.0;
		double MinimumZ = 0.0;
		double MaximumZ = 0.0;
		uint64 StableIndex = 0;
	};
	const FCornerClosureBand CornerClosureBands[] = {
		// The supplied goal-box traces enter the lower cage return at |Y|=439..599
		// before reaching the older 600 cm branch window. Keep this additional
		// source-fitted strip narrow: it closes the observed wheel-ray gap without
		// extending the post geometry into the playable field.
		{ 400.0, 600.0, 25.0, 125.0, 43ull },
		{ 400.0, 600.0, 125.0, 225.0, 44ull },
		{ 400.0, 600.0, 225.0, 325.0, 45ull },
		{ 400.0, 600.0, 325.0, 425.0, 46ull },
		{ 400.0, 600.0, 425.0, 525.0, 47ull },
		{ 400.0, 600.0, 525.0, 625.0, 48ull },
		{ 400.0, 600.0, 625.0, 725.0, 49ull },
		{ 600.0, 750.0, 25.0, 125.0, 1ull },
		{ 600.0, 750.0, 125.0, 225.0, 2ull },
		{ 600.0, 750.0, 225.0, 325.0, 3ull },
		{ 600.0, 750.0, 325.0, 425.0, 4ull },
		{ 750.0, 950.0, 25.0, 125.0, 5ull },
		{ 750.0, 950.0, 125.0, 225.0, 6ull },
		{ 750.0, 950.0, 225.0, 325.0, 7ull },
		{ 750.0, 950.0, 325.0, 425.0, 8ull },
		{ 600.0, 750.0, 525.0, 625.0, 9ull },
		{ 750.0, 950.0, 525.0, 625.0, 10ull },
		{ 600.0, 750.0, 625.0, 725.0, 11ull },
		{ 750.0, 950.0, 625.0, 725.0, 12ull },
		{ 675.0, 725.0, 25.0, 125.0, 13ull },
		{ 775.0, 825.0, 25.0, 125.0, 14ull },
		{ 775.0, 825.0, 125.0, 225.0, 15ull },
		{ 775.0, 825.0, 225.0, 325.0, 16ull },
		{ 875.0, 925.0, 125.0, 225.0, 17ull },
		{ 875.0, 925.0, 225.0, 325.0, 18ull },
		// The outer wheel sphere reaches |Y|≈934 while its centre is still
		// inside the lower-gutter return.  Bridge only that 50 cm source window;
		// keeping it separate avoids widening the mixed-feature 875..925 band.
		{ 925.0, 975.0, 225.0, 325.0, 59ull },
		// The same outer sphere crosses the backboard/lateral-gutter turn at
		// Z≈0.38..0.43 m.  Keep the two adjoining source windows separate from
		// the low return so the junction closes without prolonging the vertical
		// cage wall into the playable field.
		{ 925.0, 975.0, 325.0, 425.0, 60ull },
		{ 925.0, 975.0, 425.0, 525.0, 61ull },
		// The broad 775..825 x 25..125 closure is deliberately retained as a
		// coarse candidate, but this rear-arc/lateral-wall corner is not
		// single-cubic to the 35 cm source-residual budget.  Four local source
		// fits cover the same window without relaxing that budget.  Their shared
		// boundaries come from identical mesh samples and the X/Y loop below
		// keeps the correction exactly symmetric.
		{ 775.0, 800.0, 25.0, 75.0, 62ull },
		{ 775.0, 800.0, 75.0, 125.0, 63ull },
		{ 800.0, 825.0, 25.0, 75.0, 64ull },
		{ 800.0, 825.0, 75.0, 125.0, 65ull },
		{ 775.0, 800.0, 125.0, 175.0, 66ull },
		{ 775.0, 800.0, 175.0, 225.0, 67ull },
		{ 800.0, 825.0, 125.0, 175.0, 68ull },
		{ 800.0, 825.0, 175.0, 225.0, 69ull },
		// The outer rear wheel reaches |Y|≈879 at Z≈176 after the inner
		// closure is restored.  The broad 875..925 x 125..225 fit is rejected
		// by the source-residual gate, so retain the same local subdivision
		// pattern and keep it symmetric across X/Y.
		{ 875.0, 900.0, 125.0, 175.0, 70ull },
		{ 875.0, 900.0, 175.0, 225.0, 71ull },
		{ 900.0, 925.0, 125.0, 175.0, 72ull },
		{ 900.0, 925.0, 175.0, 225.0, 73ull },
		{ 875.0, 925.0, 625.0, 725.0, 19ull },
		{ 690.0, 710.0, 25.0, 35.0, 20ull },
		{ 790.0, 810.0, 25.0, 35.0, 21ull },
		{ 890.0, 910.0, 125.0, 225.0, 22ull },
		{ 890.0, 910.0, 225.0, 325.0, 23ull },
		{ 890.0, 910.0, 715.0, 735.0, 24ull },
		{ 690.0, 710.0, 125.0, 135.0, 25ull },
		{ 790.0, 810.0, 125.0, 135.0, 26ull },
		{ 890.0, 910.0, 125.0, 135.0, 27ull },
		{ 890.0, 910.0, 225.0, 235.0, 28ull },
		{ 890.0, 910.0, 325.0, 335.0, 29ull },
		{ 890.0, 910.0, 615.0, 635.0, 30ull },
		{ 799.0, 801.0, 24.0, 26.0, 31ull },
		{ 899.0, 901.0, 124.0, 126.0, 32ull },
		{ 899.0, 901.0, 624.0, 626.0, 33ull },
		{ 899.0, 901.0, 724.0, 726.0, 34ull },
		// The low diagonal-to-backboard return is a separate authored feature
		// from the |Y|=3188 diagonal terminal.  Keep source-fitted strips around
		// the measured |Y|=3.3..3.4 m wheel rays so the seam cannot be crossed by
		// a wheel or hitbox without changing the broad diagonal surface.
		// Fit this low return as one surface across its full wheel-contact height.
		// Separate 100 cm bands were position-continuous but still allowed a wheel
		// to exhaust 0.01 cm of droop while straddling their derivative seam.
		{ 3250.0, 3500.0, 25.0, 425.0, 51ull },
		// The broad low-return fit spans a strong bend in the authored corner.
		// Keep a local, source-constrained witness around the inner rear-wheel
		// trajectory so interpolation cannot bow away from the wheel sphere.
		{ 3300.0, 3450.0, 250.0, 350.0, 52ull },
		// The corresponding high-gutter return is centred at |Y|≈3.34 m and
		// |Z|≈1.8 m.  These narrow bands preserve the source branch while closing
		// the observed high diagonal/backboard support seam.
		{ 3200.0, 3450.0, 1625.0, 1725.0, 54ull },
		{ 3200.0, 3450.0, 1725.0, 1825.0, 55ull },
		{ 3200.0, 3450.0, 1825.0, 1925.0, 56ull },
		// The goal-box return at |Y|≈840 and Z≈660 is above the old 525..625
		// strip.  Add only the adjoining band needed by the logged wheel ray.
		{ 825.0, 875.0, 625.0, 725.0, 57ull },
		// The post/lateral hole at |Y|≈620, Z≈87 is outside the old source patch
		// bounds (the ray bends toward X≈-5.6 m).  This local strip is source
		// constrained and symmetric across both arena axes.
		{ 600.0, 650.0, 25.0, 125.0, 58ull },
		// The low-gutter lap reaches |Y|≈856 at Z≈30. The previous 790..810
		// micro-band left the measured diagonal->lateral wheel ray outside every
		// certified cell, so add only the adjoining 40 cm source window.
		{ 825.0, 875.0, 25.0, 125.0, 50ull },
		// Narrow vertical bridge bands around the two goal-box branch cuts.  The
		// authored mesh is single-valued in these small windows even though the
		// surrounding 200 cm bands mix the rear wall and the side lip.
		{ 600.0, 750.0, 425.0, 525.0, 38ull },
		{ 750.0, 950.0, 425.0, 525.0, 39ull },
		{ 790.0, 810.0, 625.0, 725.0, 40ull },
		{ 775.0, 825.0, 35.0, 125.0, 41ull },
		// The post/lateral return sits just outside the original 775 cm strip;
		// keep a narrow overlap around the measured y=-773 cm witness so the
		// closure does not leave a two-centimetre radial seam.
		{ 735.0, 775.0, 35.0, 125.0, 42ull },
		// The diagonal radial rays terminate on the compact side/back transitions
		// around |Y|=3191.  Keep these as independent narrow C2 witnesses instead
		// of widening the transverse-lip bands into a different feature partition.
		{ 3188.0, 3194.0, 497.0, 503.0, 35ull },
		{ 3188.0, 3194.0, 997.0, 1003.0, 36ull },
		{ 3188.0, 3194.0, 1797.0, 1803.0, 37ull },
		// The lower side/backboard return reaches the end of the certified
		// diagonal patch at |X|≈4.00 m, |Y|≈4.35 m.  Keep a local source-fitted
		// bridge over that wheel-sized endpoint instead of extending the broad
		// transition or changing solver support rules.
		{ 4250.0, 4450.0, 25.0, 125.0, 74ull },
		{ 4325.0, 4385.0, 25.0, 125.0, 77ull },
		// The adjoining lower corner/backboard return has the same endpoint
		// pattern around |Y|≈3.05 m and Z≈0.12..0.22 m.  This narrow band closes
		// the measured wheel path while retaining the authored branch selection.
		{ 2950.0, 3150.0, 75.0, 225.0, 75ull },
		// At the cage wall/goal-arc turn the outer wheel reaches |Y|≈0.83 m
		// before entering the next certified strip.  Bridge only this 50 cm
		// overlap so the analytic surface remains local and C2 at its boundary.
		{ 825.0, 875.0, 125.0, 225.0, 76ull },
		{ 740.0, 760.0, 110.0, 150.0, 79ull },
		// The wheel-radius sweep leaves the low return through the adjoining
		// upper corner at |Y|~=762.5 and Z~=400..510.  Keep this bridge narrow
		// around the observed seam rather than widening the whole cage wall.
		{ 730.0, 790.0, 400.0, 530.0, 80ull },
	};
	if (true) for (const int32 XSign : { -1, 1 })
	{
		for (const int32 YSign : { -1, 1 })
		{
			for (const FCornerClosureBand& Band : CornerClosureBands)
			{
				TArray<FVector3d, TInlineAllocator<16>> Samples;
				uint64 SourceId = 0;
				bool bComplete = true;
				for (int32 UIndex = 0; UIndex < 4; ++UIndex)
				{
					const double Y = YSign * FMath::Lerp(Band.MinimumY,
						Band.MaximumY, UIndex / 3.0);
					for (int32 VIndex = 0; VIndex < 4; ++VIndex)
					{
						const double Z = FMath::Lerp(Band.MinimumZ,
							Band.MaximumZ, VIndex / 3.0);
						FWorldQuery Query;
						Query.Start = FVector3d(0.0, Y, Z);
						Query.End = FVector3d(XSign * 9000.0, Y, Z);
						Query.RequiredSourceId = 0;
						Query.bIncludeTriangles = true;
						const FWorldHit Hit = SourceQueryService.Sweep(Query);
						bComplete &= Hit.bHit;
						SourceId = SourceId == 0 ? Hit.SourceId : SourceId;
						// Band 51 closes the low diagonal-to-backboard return.  Its
						// authored witness is the outside face; the inner wheel path
						// sits roughly one wheel-radius inside that face.  Keep the
						// correction local and mirror-symmetric so the analytic support
						// surface remains continuous without touching suspension code.
						constexpr double LowReturnInnerShiftCm = 15.0;
						const FVector3d InnerShift =
							(Band.StableIndex == 51ull)
								? FVector3d(-XSign * LowReturnInnerShiftCm, 0.0, 0.0)
								: (Band.StableIndex == 52ull)
									? FVector3d(-XSign * 10.0, 0.0, 0.0)
									: ((Band.StableIndex == 75ull ||
										Band.StableIndex == 76ull ||
										Band.StableIndex == 77ull ||
										Band.StableIndex == 79ull ||
										Band.StableIndex == 80ull)
										? FVector3d(-XSign * 14.0, 0.0, 0.0)
										: FVector3d::ZeroVector);
						Samples.Add(Hit.bHit ? Hit.Point + InnerShift : FVector3d::ZeroVector);
					}
				}
				if (!bComplete || SourceId == 0) continue;
				TArray<FVector3d, TInlineAllocator<16>> AlongU;
				AlongU.SetNumUninitialized(16);
				for (int32 VIndex = 0; VIndex < 4; ++VIndex)
				{
					const TArray<FVector3d, TInlineAllocator<4>> C = CubicControls(
						Samples[0 * 4 + VIndex], Samples[1 * 4 + VIndex],
						Samples[2 * 4 + VIndex], Samples[3 * 4 + VIndex]);
					for (int32 UIndex = 0; UIndex < 4; ++UIndex)
						AlongU[UIndex * 4 + VIndex] = C[UIndex];
				}
				FPiecewiseTensorBezierCell Cell;
				Cell.MinimumU = Cell.MinimumV = 0.0;
				Cell.MaximumU = Cell.MaximumV = 1.0;
				Cell.LongitudinalParameterScale = 1.0;
				Cell.bTerminalClosure = true;
				Cell.Surface.DegreeU = Cell.Surface.DegreeV = 3;
				Cell.Surface.ControlPoints.SetNumUninitialized(16);
				for (int32 UIndex = 0; UIndex < 4; ++UIndex)
				{
					const TArray<FVector3d, TInlineAllocator<4>> C = CubicControls(
						AlongU[UIndex * 4 + 0], AlongU[UIndex * 4 + 1],
						AlongU[UIndex * 4 + 2], AlongU[UIndex * 4 + 3]);
					for (int32 VIndex = 0; VIndex < 4; ++VIndex)
						Cell.Surface.ControlPoints[UIndex * 4 + VIndex] = C[VIndex];
				}
				double MaximumSourceResidualCm = 0.0;
					for (int32 UIndex = 0; UIndex <= 8; ++UIndex)
					for (int32 VIndex = 0; VIndex <= 8; ++VIndex)
					{
						const double U = UIndex / 8.0;
						const double V = VIndex / 8.0;
						const double Y = YSign * FMath::Lerp(Band.MinimumY,
							Band.MaximumY, U);
						const double Z = FMath::Lerp(Band.MinimumZ,
							Band.MaximumZ, V);
						FWorldQuery Query;
						Query.Start = FVector3d(0.0, Y, Z);
						Query.End = FVector3d(XSign * 9000.0, Y, Z);
						Query.bIncludeTriangles = true;
						const FWorldHit Hit = SourceQueryService.Sweep(Query);
						if (!Hit.bHit)
						{
							MaximumSourceResidualCm = TNumericLimits<double>::Max();
							break;
						}
						MaximumSourceResidualCm = FMath::Max(
							MaximumSourceResidualCm,
							FVector3d::Distance(Hit.Point, Cell.Surface.Evaluate(U, V)));
					}
				// Local branch-cut fits may legitimately carry a small source residual
				// while remaining single-valued.  Broad mixed-feature bands stay well
				// above this limit and are still rejected.
				constexpr double CornerClosureSourceResidualToleranceCm = 35.0;
				if (MaximumSourceResidualCm > CornerClosureSourceResidualToleranceCm) continue;
				FPiecewiseTensorBezierPatch Patch;
				Patch.SourceId = SourceId;
				Patch.SurfaceId = CombineStableIds(StableStringId(TEXT("CornerClosureC2.V2")),
					CombineStableIds(static_cast<uint64>(XSign + 2),
						CombineStableIds(static_cast<uint64>(YSign + 2), Band.StableIndex)));
				Patch.PrimitiveId = CombineStableIds(Patch.SurfaceId, StableStringId(TEXT("Adaptive")));
				Patch.CanonicalGroupId = Patch.SurfaceId;
				Patch.bQueryCollisionEnabled = true;
				Cell.FeatureId = CombineStableIds(Patch.SurfaceId, 1ull);
				Cell.PrimitiveId = CombineStableIds(Patch.PrimitiveId, 1ull);
				Patch.Cells.Add(MoveTemp(Cell));
				Patch.bSourceResidualCertified = true;
				if (Patch.BuildQueryApproximation())
				{
					Patch.bAuthorityEligible = Patch.bApproximationCertified;
					UE_LOG(LogTemp, Display, TEXT("[AnalyticCornerClosure] XSign=%d YSign=%d Band=%llu BoundsMin=%s BoundsMax=%s SourceResidualCm=%.9g ErrorCm=%.9g Certified=%d"),
						XSign, YSign, Band.StableIndex, *Patch.Bounds.Min.ToString(),
						*Patch.Bounds.Max.ToString(), MaximumSourceResidualCm,
						Patch.MaximumApproximationErrorCm, Patch.bAuthorityEligible ? 1 : 0);
					// Keep source-fitted witnesses as independent providers where merging
					// into a different broad closure lane can hide the local cell behind
					// that lane's earlier candidate before provider arbitration. Bands 4
					// and 38 are the two adjoining halves of the exact mid-height
					// terminal bend. Give both the same local terminal ownership so the
					// Z=425 boundary cannot reopen the broad adaptive overlap above it;
					// band 52 is the inner-wheel witness at the lower return.
					if (Band.StableIndex == 4ull || Band.StableIndex == 38ull ||
						Band.StableIndex == 75ull || Band.StableIndex == 76ull ||
						Band.StableIndex == 79ull || Band.StableIndex == 80ull)
					{
						LocalTerminalOwnerSurfaceIds.Add(Patch.SurfaceId);
						DeferredLocalTerminalOwnershipBounds.Add(Patch.Bounds);
						CedeOverlappingTerminalCells(Patch.Bounds);
						FBox3d MirroredBounds = Patch.Bounds;
						MirroredBounds.Min.X = -Patch.Bounds.Max.X;
						MirroredBounds.Max.X = -Patch.Bounds.Min.X;
						DeferredLocalTerminalOwnershipBounds.Add(MirroredBounds);
						CedeOverlappingTerminalCells(MirroredBounds);
						PiecewiseTensorBezierPatches.Add(MoveTemp(Patch));
					}
					else if (Band.StableIndex == 52ull)
					{
						// These are complete two-dimensional junction surfaces, not
						// terminal fragments of an existing lane.  Preserve their source
						// identity and local parameterization so provider arbitration sees
						// the intended C2 bridge as one independent candidate.
						LocalTerminalOwnerSurfaceIds.Add(Patch.SurfaceId);
						DeferredLocalTerminalOwnershipBounds.Add(Patch.Bounds);
						PiecewiseTensorBezierPatches.Add(MoveTemp(Patch));
					}
					else
						MergeTerminalClosure(MoveTemp(Patch),
							Band.StableIndex == 63ull || Band.StableIndex == 65ull ||
							Band.StableIndex == 66ull || Band.StableIndex == 67ull ||
							Band.StableIndex == 68ull || Band.StableIndex == 69ull ||
							Band.StableIndex == 70ull || Band.StableIndex == 71ull ||
							Band.StableIndex == 72ull || Band.StableIndex == 73ull);
				}
			}
		}
	}
	}
	// Four compact terminal-grid closure cells cover the exact lower-lip witnesses
	// at |Y|=800, Z=25 where the authored mesh changes feature ownership over a
	// sub-centimetre interval.  They are tangent-plane C2 terminal cells, kept
	// deliberately small so they cannot alter the neighbouring corner fit.
	for (const int32 XSign : { -1, 1 }) for (const int32 YSign : { -1, 1 })
	{
		const double Y = YSign * 800.0;
		const double Z = 25.0;
		FWorldQuery Query;
		Query.Start = FVector3d(0.0, Y, Z);
		Query.End = FVector3d(XSign * 9000.0, Y, Z);
		Query.bIncludeTriangles = true;
		const FWorldHit Hit = SourceQueryService.Sweep(Query);
		if (!Hit.bHit || Hit.SourceId == 0) continue;
		FVector3d TangentU(0.0, 1.0, 0.0);
		TangentU = (TangentU - FVector3d::DotProduct(TangentU, Hit.Normal) * Hit.Normal).GetSafeNormal();
		const FVector3d TangentV = FVector3d::CrossProduct(Hit.Normal, TangentU).GetSafeNormal();
		FPiecewiseTensorBezierPatch Patch;
		Patch.SourceId = Hit.SourceId;
		Patch.SurfaceId = CombineStableIds(StableStringId(TEXT("TerminalGridClosureC2.V1")),
			CombineStableIds(static_cast<uint64>(XSign + 2), static_cast<uint64>(YSign + 2)));
		Patch.PrimitiveId = CombineStableIds(Patch.SurfaceId, StableStringId(TEXT("MicroTangent")));
		Patch.CanonicalGroupId = StableStringId(TEXT("TerminalGridClosureC2.Canonical.V1"));
		Patch.bQueryCollisionEnabled = true;
		FPiecewiseTensorBezierCell& Cell = Patch.Cells.AddDefaulted_GetRef();
		Cell.FeatureId = CombineStableIds(Patch.SurfaceId, 1ull);
		Cell.PrimitiveId = CombineStableIds(Patch.PrimitiveId, 1ull);
		Cell.MinimumU = Cell.MinimumV = 0.0;
		Cell.MaximumU = Cell.MaximumV = 1.0;
		Cell.LongitudinalParameterScale = 1.0;
		Cell.bTerminalClosure = true;
		Cell.Surface.DegreeU = Cell.Surface.DegreeV = 3;
		Cell.Surface.ControlPoints.SetNumUninitialized(16);
		for (int32 UIndex = 0; UIndex < 4; ++UIndex) for (int32 VIndex = 0; VIndex < 4; ++VIndex)
		{
			const double DU = FMath::Lerp(-2.0, 2.0, UIndex / 3.0);
			const double DV = FMath::Lerp(-2.0, 2.0, VIndex / 3.0);
			Cell.Surface.ControlPoints[UIndex * 4 + VIndex] = Hit.Point + TangentU * DU + TangentV * DV;
		}
		Patch.bSourceResidualCertified = true;
		if (Patch.BuildQueryApproximation())
		{
			Patch.bAuthorityEligible = Patch.bApproximationCertified;
			if (Patch.bAuthorityEligible) PiecewiseTensorBezierPatches.Add(MoveTemp(Patch));
		}
	}
	for (int32 CandidateIndex = 0; CandidateIndex < OpenRimCandidates.Num();
		++CandidateIndex)
	{
		const FOpenRimCandidate& Candidate = OpenRimCandidates[CandidateIndex];
		if (!Candidate.bFeaturePartitionComplete || !Candidate.bOpenAtBaseline ||
			Candidate.WallAxis > 1 || Candidate.EdgeCount <= 0)
		{
			continue;
		}
		int32 SourceTriangleIndex = INDEX_NONE;
		for (const FOpenRimSurfaceBandObservation& Observation :
			OpenRimSurfaceBandObservations)
		{
			if (Observation.OpenRimCandidateIndex == CandidateIndex &&
				Triangles.IsValidIndex(Observation.OpeningSurfaceTriangleIndex))
			{
				SourceTriangleIndex = Observation.OpeningSurfaceTriangleIndex;
				break;
			}
		}
		if (!Triangles.IsValidIndex(SourceTriangleIndex)) continue;
		const FTriangleSurface& SourceTriangle = Triangles[SourceTriangleIndex];
		const int32 WallAxis = Candidate.WallAxis;
		const int32 TransverseAxis = WallAxis == 0 ? 1 : 0;
		constexpr int32 VerticalAxis = 2;
		const double WallCoordinate = Candidate.Bounds.GetCenter()[WallAxis];
		const double OpeningSign = WallCoordinate >= 0.0 ? 1.0 : -1.0;
		const double TransverseHalfExtent = FMath::Max(
			FMath::Abs(Candidate.Bounds.Min[TransverseAxis]),
			FMath::Abs(Candidate.Bounds.Max[TransverseAxis]));
		double InteriorBaseline = FMath::Min(Candidate.BaselineHeightCm,
			Candidate.Bounds.Min[VerticalAxis]);
		double HighestUpwardSupport = -TNumericLimits<double>::Max();
		for (const FBoundedPlane& Plane : Planes)
		{
			if (Plane.Normal[VerticalAxis] < 0.99 ||
				Plane.Origin[VerticalAxis] >= Candidate.Bounds.Min[VerticalAxis])
			{
				continue;
			}
			if (Plane.Origin[VerticalAxis] > HighestUpwardSupport)
			{
				HighestUpwardSupport = Plane.Origin[VerticalAxis];
				InteriorBaseline = Plane.Origin[VerticalAxis];
			}
		}
		const double InteriorTop = FMath::Max(Candidate.HorizontalSpanHeightCm,
			Candidate.Bounds.Max[VerticalAxis]);
		const double VerticalSpan = InteriorTop - InteriorBaseline;
		if (TransverseHalfExtent <= 100.0 || VerticalSpan <= 100.0) continue;
		const double InteriorHalfWidth = 0.64 * TransverseHalfExtent;
		const double MinimumVertical = InteriorBaseline + 0.029 * VerticalSpan;
		const double MaximumVertical = InteriorTop -
			0.10 * VerticalSpan;
		// A suspension sphere can otherwise run out of travel less than one
		// centimetre before reaching the sampled rear cap.  Move the interior of
		// this curved provider slightly toward the playable volume.  The quintic
		// ramps reach zero value, tangent and curvature on every cap boundary, so
		// this polish preserves the surrounding C2 joins instead of adding a hard
		// closure plane.
		// Stay below the ten-centimetre authority-agreement boundary after the
		// source-fit residual is included.  The retained 1.8 cm support margin is
		// still materially larger than the cap-side loss observed at 8 cm.
		constexpr double InteriorCapSupportPolishCm = 9.8;
		constexpr double InteriorCapBoundaryBlendWidthCm = 120.0;
		auto C2BoundaryRamp = [](const double Distance,
			const double Width)
		{
			const double T = FMath::Clamp(Distance / Width, 0.0, 1.0);
			return T * T * T * (10.0 + T * (-15.0 + 6.0 * T));
		};
		constexpr int32 TransverseKnotCount = 97;
		constexpr int32 VerticalKnotCount = 61;
		TArray<double> TransverseKnots, VerticalKnots;
		for (int32 Index = 0; Index < TransverseKnotCount; ++Index)
			TransverseKnots.Add(FMath::Lerp(-InteriorHalfWidth, InteriorHalfWidth,
				static_cast<double>(Index) / (TransverseKnotCount - 1)));
		for (int32 Index = 0; Index < VerticalKnotCount; ++Index)
			VerticalKnots.Add(FMath::Lerp(MinimumVertical, MaximumVertical,
				static_cast<double>(Index) / (VerticalKnotCount - 1)));
		auto SampleSource = [&](const double Transverse, const double Vertical,
			FVector3d& OutPoint)
		{
			FWorldQuery Query;
			Query.Start[WallAxis] = WallCoordinate - OpeningSign *
				(FMath::Abs(WallCoordinate) + 1000.0);
			Query.End[WallAxis] = WallCoordinate + OpeningSign * 2000.0;
			Query.Start[TransverseAxis] = Query.End[TransverseAxis] = Transverse;
			Query.Start[VerticalAxis] = Query.End[VerticalAxis] = Vertical;
			Query.RequiredSourceId = SourceTriangle.SourceId;
			Query.bIncludeTriangles = true;
			const FWorldHit Hit = SourceQueryService.Sweep(Query);
			if (!Hit.bHit || OpeningSign * (Hit.Point[WallAxis] - WallCoordinate) < 50.0)
				return false;
			OutPoint = Hit.Point;
			return true;
		};
		TArray<TArray<FCubicBezierSegment>> TransverseSegmentsByVertical;
		bool bCompleteGrid = true;
		for (const double Vertical : VerticalKnots)
		{
			TArray<FVector3d> Values;
			for (const double Transverse : TransverseKnots)
			{
				FVector3d Point;
				bCompleteGrid &= SampleSource(Transverse, Vertical, Point);
				const double TransverseBoundaryDistance =
					InteriorHalfWidth - FMath::Abs(Transverse);
				const double VerticalBoundaryDistance = FMath::Min(
					Vertical - MinimumVertical, MaximumVertical - Vertical);
				const double PolishWeight =
					C2BoundaryRamp(TransverseBoundaryDistance,
						InteriorCapBoundaryBlendWidthCm) *
					C2BoundaryRamp(VerticalBoundaryDistance,
						InteriorCapBoundaryBlendWidthCm);
				Point[WallAxis] -= OpeningSign *
					InteriorCapSupportPolishCm * PolishWeight;
				Values.Add(Point);
			}
			TArray<FCubicBezierSegment>& Row =
				TransverseSegmentsByVertical.AddDefaulted_GetRef();
			bCompleteGrid &= BuildNaturalCubicBezierSegments(
				TransverseKnots, Values, Row);
		}
		if (!bCompleteGrid) continue;
		FPiecewiseTensorBezierPatch CapPatch;
		CapPatch.SourceId = SourceTriangle.SourceId;
		CapPatch.SurfaceId = CombineStableIds(Candidate.CandidateId,
			StableStringId(TEXT("PiecewiseFiniteInteriorCapC2.V1")));
		CapPatch.PrimitiveId = CombineStableIds(CapPatch.SurfaceId,
			StableStringId(TEXT("NaturalBicubic")));
		CapPatch.CanonicalGroupId = StableStringId(
			TEXT("PiecewiseFiniteInteriorCapC2.Canonical.V1"));
		CapPatch.MaterialId = SourceTriangle.MaterialId;
		CapPatch.ObjectType = SourceTriangle.ObjectType;
		CapPatch.BlockingChannels = SourceTriangle.BlockingChannels;
		CapPatch.bQueryCollisionEnabled = SourceTriangle.bQueryCollisionEnabled;
		for (int32 UIndex = 0; UIndex + 1 < TransverseKnotCount; ++UIndex)
		{
			TArray<TArray<FCubicBezierSegment>> VerticalSegmentsByUControl;
			for (int32 UControl = 0; UControl < 4; ++UControl)
			{
				TArray<FVector3d> Values;
				for (int32 VIndex = 0; VIndex < VerticalKnotCount; ++VIndex)
					Values.Add(TransverseSegmentsByVertical[VIndex][UIndex].
						ControlPoints[UControl]);
				TArray<FCubicBezierSegment>& Column =
					VerticalSegmentsByUControl.AddDefaulted_GetRef();
				bCompleteGrid &= BuildNaturalCubicBezierSegments(
					VerticalKnots, Values, Column);
			}
			for (int32 VIndex = 0; VIndex + 1 < VerticalKnotCount; ++VIndex)
			{
				FPiecewiseTensorBezierCell& Cell =
					CapPatch.Cells.AddDefaulted_GetRef();
				Cell.FeatureId = CombineStableIds(CapPatch.SurfaceId,
					static_cast<uint64>(1 + UIndex * (VerticalKnotCount - 1) + VIndex));
				Cell.PrimitiveId = CombineStableIds(CapPatch.PrimitiveId,
					static_cast<uint64>(1 + UIndex * (VerticalKnotCount - 1) + VIndex));
				Cell.MinimumU = static_cast<double>(UIndex) /
					(TransverseKnotCount - 1);
				Cell.MaximumU = static_cast<double>(UIndex + 1) /
					(TransverseKnotCount - 1);
				Cell.MinimumV = static_cast<double>(VIndex) / (VerticalKnotCount - 1);
				Cell.MaximumV = static_cast<double>(VIndex + 1) /
					(VerticalKnotCount - 1);
				Cell.LongitudinalParameterScale = 1.0;
				Cell.Surface.DegreeU = Cell.Surface.DegreeV = 3;
				Cell.Surface.ControlPoints.SetNumUninitialized(16);
				const int32 PhysicalVIndex = OpeningSign < 0.0 ? VIndex :
					VerticalKnotCount - 2 - VIndex;
				for (int32 UControl = 0; UControl < 4; ++UControl)
					for (int32 VControl = 0; VControl < 4; ++VControl)
					{
						const int32 PhysicalVControl = OpeningSign < 0.0 ?
							VControl : 3 - VControl;
						Cell.Surface.ControlPoints[UControl * 4 + VControl] =
							VerticalSegmentsByUControl[UControl][PhysicalVIndex].
								ControlPoints[PhysicalVControl];
					}
			}
		}
		double MaximumSourceResidualCm = 0.0;
		for (const FPiecewiseTensorBezierCell& Cell : CapPatch.Cells)
		{
			for (const double U : { 0.25, 0.5, 0.75 })
				for (const double V : { 0.25, 0.5, 0.75 })
				{
					const FVector3d Point = Cell.Surface.Evaluate(U, V);
					FVector3d SourcePoint;
					if (!SampleSource(Point[TransverseAxis], Point[VerticalAxis], SourcePoint))
					{
						MaximumSourceResidualCm = TNumericLimits<double>::Max();
						continue;
					}
					MaximumSourceResidualCm = FMath::Max(MaximumSourceResidualCm,
						FMath::Abs(SourcePoint[WallAxis] - Point[WallAxis]));
				}
		}
		CapPatch.bSourceResidualCertified = MaximumSourceResidualCm <=
			1.0 + InteriorCapSupportPolishCm;
		UE_LOG(LogTemp, Display, TEXT(
			"[AnalyticFiniteInteriorCap] Candidate=%016llX Cells=%d Baseline=%.9g Top=%.9g HalfWidth=%.9g MaximumSourceResidualCm=%.9g Certified=%d"),
			Candidate.CandidateId, CapPatch.Cells.Num(), InteriorBaseline, InteriorTop,
			InteriorHalfWidth, MaximumSourceResidualCm,
			CapPatch.bSourceResidualCertified ? 1 : 0);
		Algo::Sort(CapPatch.Cells,
			[](const FPiecewiseTensorBezierCell& A,
				const FPiecewiseTensorBezierCell& B)
			{
				return A.PrimitiveId < B.PrimitiveId;
			});
		if (bCompleteGrid && !CapPatch.Cells.IsEmpty() &&
			CapPatch.BuildQueryApproximation())
		{
			CapPatch.bAuthorityEligible = CapPatch.bQueryCollisionEnabled &&
				CapPatch.bSourceResidualCertified && CapPatch.bApproximationCertified &&
				Algo::AllOf(CapPatch.Adjacencies,
					[](const FPiecewiseTensorBezierAdjacency& Link)
					{
						return Link.bC2ByConstruction;
					});
			PiecewiseTensorBezierPatches.Add(MoveTemp(CapPatch));
		}

		// The finite cap is deliberately narrow because its complete height range
		// is not a single-valued graph around the outer corner.  At wheel-contact
		// height, however, the source is a regular longitudinal=f(transverse,Z)
		// sheet from the cap to the transverse providers.  Fit that local overlap
		// as two dense natural-bicubic lobes instead of crossing it with the old
		// X=+/-6575 closure plane.
		constexpr double GoalEdgeBridgeMinimumAbsTransverseCm = 575.0;
		constexpr double GoalEdgeBridgeMaximumAbsTransverseCm = 875.0;
		constexpr double GoalEdgeBridgeMinimumVerticalCm = 225.0;
		constexpr double GoalEdgeBridgeMaximumVerticalCm = 700.0;
		constexpr double GoalEdgeBridgeInteriorLeadCm = 0.0;
		constexpr double GoalEdgeBridgeMaximumSourceResidualCm = 15.0;
		constexpr int32 GoalEdgeBridgeTransverseKnotCount = 25;
		constexpr int32 GoalEdgeBridgeVerticalKnotCount = 39;
		auto GoalEdgeBridgeC2Window = [](const double Parameter)
		{
			const double EdgeDistance = FMath::Min(Parameter, 1.0 - Parameter);
			const double T = FMath::Clamp(EdgeDistance / 0.25, 0.0, 1.0);
			return T * T * T * (10.0 + T * (-15.0 + 6.0 * T));
		};
		for (const double TransverseSign : { -1.0, 1.0 })
		{
			TArray<double> BridgeTransverseKnots, BridgeVerticalKnots;
			for (int32 Index = 0;
				Index < GoalEdgeBridgeTransverseKnotCount; ++Index)
			{
				BridgeTransverseKnots.Add(static_cast<double>(Index) /
					(GoalEdgeBridgeTransverseKnotCount - 1));
			}
			for (int32 Index = 0;
				Index < GoalEdgeBridgeVerticalKnotCount; ++Index)
			{
				BridgeVerticalKnots.Add(FMath::Lerp(
					GoalEdgeBridgeMinimumVerticalCm,
					GoalEdgeBridgeMaximumVerticalCm,
					static_cast<double>(Index) /
						(GoalEdgeBridgeVerticalKnotCount - 1)));
			}
			const double NumericMinimumTransverse = TransverseSign > 0.0
				? GoalEdgeBridgeMinimumAbsTransverseCm
				: -GoalEdgeBridgeMaximumAbsTransverseCm;
			const double NumericMaximumTransverse = TransverseSign > 0.0
				? GoalEdgeBridgeMaximumAbsTransverseCm
				: -GoalEdgeBridgeMinimumAbsTransverseCm;
			// dP/du x dP/dv must face the playable volume (-OpeningSign X).
			const bool bIncreasingTransverse = OpeningSign < 0.0;
			const double BridgeTransverseStart = bIncreasingTransverse
				? NumericMinimumTransverse : NumericMaximumTransverse;
			const double BridgeTransverseEnd = bIncreasingTransverse
				? NumericMaximumTransverse : NumericMinimumTransverse;

			TArray<TArray<FCubicBezierSegment>>
				BridgeTransverseSegmentsByVertical;
			bool bBridgeComplete = true;
			for (int32 VerticalIndex = 0;
				VerticalIndex < BridgeVerticalKnots.Num(); ++VerticalIndex)
			{
				const double Vertical = BridgeVerticalKnots[VerticalIndex];
				TArray<FVector3d> Values;
				for (int32 TransverseIndex = 0;
					TransverseIndex < BridgeTransverseKnots.Num();
					++TransverseIndex)
				{
					const double TransverseT =
						BridgeTransverseKnots[TransverseIndex];
					const double Transverse = FMath::Lerp(
						BridgeTransverseStart, BridgeTransverseEnd,
						TransverseT);
					FVector3d Point = FVector3d::ZeroVector;
					const bool bSampledBridgePoint = SampleSource(Transverse,
						Vertical, Point);
					if (!bSampledBridgePoint && bBridgeComplete)
					{
						UE_LOG(LogTemp, Display, TEXT(
							"[AnalyticGoalEdgeBridgeSampleMiss] Candidate=%016llX Side=%+.0f Transverse=%.9g Vertical=%.9g"),
							Candidate.CandidateId, TransverseSign, Transverse,
							Vertical);
					}
					bBridgeComplete &= bSampledBridgePoint;
					const double VerticalT = static_cast<double>(VerticalIndex) /
						(GoalEdgeBridgeVerticalKnotCount - 1);
					Point[WallAxis] -= OpeningSign *
						GoalEdgeBridgeInteriorLeadCm *
						GoalEdgeBridgeC2Window(TransverseT) *
						GoalEdgeBridgeC2Window(VerticalT);
					Values.Add(Point);
				}
				TArray<FCubicBezierSegment>& Row =
					BridgeTransverseSegmentsByVertical.AddDefaulted_GetRef();
				bBridgeComplete &= BuildNaturalCubicBezierSegments(
					BridgeTransverseKnots, Values, Row);
			}
			if (!bBridgeComplete) continue;

			FPiecewiseTensorBezierPatch BridgePatch;
			BridgePatch.SourceId = SourceTriangle.SourceId;
			BridgePatch.SurfaceId = CombineStableIds(Candidate.CandidateId,
				CombineStableIds(StableStringId(
					TEXT("PiecewiseGoalEdgeBridgeC2.V1")),
					TransverseSign < 0.0 ? 1ull : 2ull));
			BridgePatch.PrimitiveId = CombineStableIds(
				BridgePatch.SurfaceId, StableStringId(TEXT("NaturalBicubic")));
			BridgePatch.CanonicalGroupId = StableStringId(
				TEXT("PiecewiseGoalEdgeBridgeC2.Canonical.V1"));
			BridgePatch.MaterialId = SourceTriangle.MaterialId;
			BridgePatch.ObjectType = SourceTriangle.ObjectType;
			BridgePatch.BlockingChannels = SourceTriangle.BlockingChannels;
			BridgePatch.bQueryCollisionEnabled =
				SourceTriangle.bQueryCollisionEnabled;
			for (int32 UIndex = 0;
				UIndex + 1 < GoalEdgeBridgeTransverseKnotCount; ++UIndex)
			{
				TArray<TArray<FCubicBezierSegment>>
					BridgeVerticalSegmentsByUControl;
				for (int32 UControl = 0; UControl < 4; ++UControl)
				{
					TArray<FVector3d> Values;
					for (int32 VerticalIndex = 0;
						VerticalIndex < GoalEdgeBridgeVerticalKnotCount;
						++VerticalIndex)
					{
						Values.Add(BridgeTransverseSegmentsByVertical[
							VerticalIndex][UIndex].ControlPoints[UControl]);
					}
					TArray<FCubicBezierSegment>& Column =
						BridgeVerticalSegmentsByUControl.AddDefaulted_GetRef();
					bBridgeComplete &= BuildNaturalCubicBezierSegments(
						BridgeVerticalKnots, Values, Column);
				}
				for (int32 VIndex = 0;
					bBridgeComplete &&
					VIndex + 1 < GoalEdgeBridgeVerticalKnotCount; ++VIndex)
				{
					FPiecewiseTensorBezierCell& Cell =
						BridgePatch.Cells.AddDefaulted_GetRef();
					const uint64 CellIndex = static_cast<uint64>(1 +
						UIndex * (GoalEdgeBridgeVerticalKnotCount - 1) +
						VIndex);
					Cell.FeatureId = CombineStableIds(
						BridgePatch.SurfaceId, CellIndex);
					Cell.PrimitiveId = CombineStableIds(
						BridgePatch.PrimitiveId, CellIndex);
					Cell.MinimumU = static_cast<double>(UIndex) /
						(GoalEdgeBridgeTransverseKnotCount - 1);
					Cell.MaximumU = static_cast<double>(UIndex + 1) /
						(GoalEdgeBridgeTransverseKnotCount - 1);
					Cell.MinimumV = static_cast<double>(VIndex) /
						(GoalEdgeBridgeVerticalKnotCount - 1);
					Cell.MaximumV = static_cast<double>(VIndex + 1) /
						(GoalEdgeBridgeVerticalKnotCount - 1);
					Cell.LongitudinalParameterScale = 1.0;
					Cell.Surface.DegreeU = Cell.Surface.DegreeV = 3;
					Cell.Surface.ControlPoints.SetNumUninitialized(16);
					for (int32 UControl = 0; UControl < 4; ++UControl)
						for (int32 VControl = 0; VControl < 4; ++VControl)
						{
							Cell.Surface.ControlPoints[
								UControl * 4 + VControl] =
								BridgeVerticalSegmentsByUControl[UControl][VIndex].
									ControlPoints[VControl];
						}
				}
			}

			double MaximumBridgeSourceResidualCm = 0.0;
			int32 BridgeWitnessCount = 0;
			for (const FPiecewiseTensorBezierCell& Cell : BridgePatch.Cells)
				for (const double U : { 0.25, 0.5, 0.75 })
					for (const double V : { 0.25, 0.5, 0.75 })
					{
						const FVector3d Point = Cell.Surface.Evaluate(U, V);
						FVector3d SourcePoint;
						if (!SampleSource(Point[TransverseAxis],
							Point[VerticalAxis], SourcePoint))
						{
							bBridgeComplete = false;
							continue;
						}
						++BridgeWitnessCount;
						MaximumBridgeSourceResidualCm = FMath::Max(
							MaximumBridgeSourceResidualCm,
							FVector3d::Distance(SourcePoint, Point));
					}
			BridgePatch.bSourceResidualCertified = bBridgeComplete &&
				BridgeWitnessCount > 0 && MaximumBridgeSourceResidualCm <=
					GoalEdgeBridgeMaximumSourceResidualCm;
			Algo::Sort(BridgePatch.Cells,
				[](const FPiecewiseTensorBezierCell& A,
					const FPiecewiseTensorBezierCell& B)
				{
					return A.PrimitiveId < B.PrimitiveId;
				});
			if (BridgePatch.bSourceResidualCertified &&
				BridgePatch.BuildQueryApproximation())
			{
				BridgePatch.bAuthorityEligible =
					BridgePatch.bQueryCollisionEnabled &&
					BridgePatch.bApproximationCertified &&
					Algo::AllOf(BridgePatch.Adjacencies,
						[](const FPiecewiseTensorBezierAdjacency& Link)
						{
							return Link.bC2ByConstruction;
						});
			}
			UE_LOG(LogTemp, Display, TEXT(
				"[AnalyticGoalEdgeBridge] Candidate=%016llX Side=%+.0f Cells=%d Transverse=[%.9g,%.9g] Vertical=[%.9g,%.9g] Witnesses=%d SourceResidualCm=%.9g Certified=%d"),
				Candidate.CandidateId, TransverseSign, BridgePatch.Cells.Num(),
				FMath::Min(BridgeTransverseStart, BridgeTransverseEnd),
				FMath::Max(BridgeTransverseStart, BridgeTransverseEnd),
				GoalEdgeBridgeMinimumVerticalCm,
				GoalEdgeBridgeMaximumVerticalCm, BridgeWitnessCount,
				MaximumBridgeSourceResidualCm,
				BridgePatch.bAuthorityEligible ? 1 : 0);
			if (BridgePatch.bAuthorityEligible)
			{
				PiecewiseTensorBezierPatches.Add(MoveTemp(BridgePatch));
			}
		}

		// Complete the two finite transverse boundaries between the opening
		// plane and the interior cap.  These surfaces are single-valued as
		// transverse=f(longitudinal, vertical), whereas the cap above is
		// longitudinal=f(transverse, vertical).  Keeping the parameterizations
		// separate avoids a singular fit through the rounded rear corners while
		// retaining a source-certified C2 network for small solid sub-bodies.
		FVector3d RearCenterPoint;
		const double MidVertical = 0.5 * (MinimumVertical + MaximumVertical);
		if (SampleSource(0.0, MidVertical, RearCenterPoint))
		{
			const double OpeningLongitudinal =
				WallCoordinate + OpeningSign * 150.0;
			const double RearLongitudinal =
				RearCenterPoint[WallAxis] - OpeningSign * 75.0;
			constexpr double SideOpeningOverlapCm = 75.0;
			const double SideOpeningLongitudinal = OpeningLongitudinal -
				OpeningSign * SideOpeningOverlapCm;
			const double MinimumLongitudinal = FMath::Min(
				SideOpeningLongitudinal, RearLongitudinal);
			const double MaximumLongitudinal = FMath::Max(
				SideOpeningLongitudinal, RearLongitudinal);
			constexpr int32 SideLongitudinalKnotCount = 25;
			constexpr int32 SideVerticalKnotCount = 25;
			constexpr double MaximumSideSourceResidualCm = 5.0;
			const double SideMinimumVertical =
				InteriorBaseline + 0.30 * (InteriorTop - InteriorBaseline);
			const double SideMaximumVertical =
				InteriorBaseline + 0.68 * (InteriorTop - InteriorBaseline);
			if (MaximumLongitudinal - MinimumLongitudinal > 100.0)
			{
				for (const double SideSign : { -1.0, 1.0 })
				{
					auto SampleTransverseSource = [&](const double Longitudinal,
						const double Vertical, FVector3d& OutPoint)
					{
						FWorldQuery Query;
						Query.Start[WallAxis] = Query.End[WallAxis] = Longitudinal;
						Query.Start[TransverseAxis] = 0.0;
						Query.End[TransverseAxis] = SideSign *
							(TransverseHalfExtent + 500.0);
						Query.Start[VerticalAxis] = Query.End[VerticalAxis] = Vertical;
						Query.RequiredSourceId = SourceTriangle.SourceId;
						Query.bIncludeTriangles = true;
						const FWorldHit Hit = SourceQueryService.Sweep(Query);
						if (!Hit.bHit ||
							SideSign * Hit.Point[TransverseAxis] <= 100.0)
						{
							return false;
						}
						OutPoint = Hit.Point;
						return true;
					};
					auto CompactC2Window = [](const double T,
						const double RiseEnd, const double FallStart)
					{
						auto SmoothStep5 = [](const double Value)
						{
							const double Q = FMath::Clamp(Value, 0.0, 1.0);
							return Q * Q * Q * (Q * (Q * 6.0 - 15.0) + 10.0);
						};
						const double Rise = SmoothStep5(T / RiseEnd);
						const double Fall = SmoothStep5(
							(1.0 - T) / (1.0 - FallStart));
						return Rise * Fall;
					};

					TArray<double> SideLongitudinalKnots;
					TArray<double> SideVerticalKnots;
					for (int32 Index = 0; Index < SideLongitudinalKnotCount; ++Index)
					{
						SideLongitudinalKnots.Add(FMath::Lerp(
							MinimumLongitudinal, MaximumLongitudinal,
							static_cast<double>(Index) /
								(SideLongitudinalKnotCount - 1)));
					}
					for (int32 Index = 0; Index < SideVerticalKnotCount; ++Index)
					{
						SideVerticalKnots.Add(FMath::Lerp(
							SideMinimumVertical, SideMaximumVertical,
							static_cast<double>(Index) /
								(SideVerticalKnotCount - 1)));
					}

					TArray<TArray<FCubicBezierSegment>>
						LongitudinalSegmentsByVertical;
					bool bSideComplete = true;
					for (const double Vertical : SideVerticalKnots)
					{
						TArray<FVector3d> Values;
						for (const double Longitudinal : SideLongitudinalKnots)
						{
							FVector3d Point = FVector3d::ZeroVector;
							bSideComplete &= SampleTransverseSource(
								Longitudinal, Vertical, Point);
							// Give the interior of the rounded finite boundary a small
							// source-bounded lead toward the playable volume.  The
							// quintic windows have zero value, slope and curvature at
							// every edge, so the source position is retained at joins.
							constexpr double MaximumBoundarySupportPolishCm = 2.0;
							const double LongitudinalT =
								(Longitudinal - MinimumLongitudinal) /
								(MaximumLongitudinal - MinimumLongitudinal);
							const double VerticalT =
								(Vertical - SideMinimumVertical) /
								(SideMaximumVertical - SideMinimumVertical);
							Point[TransverseAxis] -= SideSign *
								MaximumBoundarySupportPolishCm *
								CompactC2Window(LongitudinalT, 0.20, 0.80) *
								CompactC2Window(VerticalT, 0.20, 0.80);
							Values.Add(Point);
						}
						TArray<FCubicBezierSegment>& Row =
							LongitudinalSegmentsByVertical.AddDefaulted_GetRef();
						bSideComplete &= BuildNaturalCubicBezierSegments(
							SideLongitudinalKnots, Values, Row);
					}

					FPiecewiseTensorBezierPatch SidePatch;
					SidePatch.SourceId = SourceTriangle.SourceId;
					SidePatch.SurfaceId = CombineStableIds(Candidate.CandidateId,
						CombineStableIds(StableStringId(
							TEXT("PiecewiseFiniteTransverseBoundaryC2.V1")),
							SideSign < 0.0 ? 1ull : 2ull));
					SidePatch.PrimitiveId = CombineStableIds(
						SidePatch.SurfaceId, StableStringId(TEXT("NaturalBicubic")));
					SidePatch.CanonicalGroupId = StableStringId(
						TEXT("PiecewiseFiniteTransverseBoundaryC2.Canonical.V1"));
					SidePatch.MaterialId = SourceTriangle.MaterialId;
					SidePatch.ObjectType = SourceTriangle.ObjectType;
					SidePatch.BlockingChannels = SourceTriangle.BlockingChannels;
					SidePatch.bQueryCollisionEnabled =
						SourceTriangle.bQueryCollisionEnabled;
					const int32 LongitudinalSegmentCount =
						SideLongitudinalKnotCount - 1;
					for (int32 PhysicalUIndex = 0;
						bSideComplete && PhysicalUIndex < LongitudinalSegmentCount;
						++PhysicalUIndex)
					{
						TArray<TArray<FCubicBezierSegment>>
							VerticalSegmentsByUControl;
						for (int32 UControl = 0; UControl < 4; ++UControl)
						{
							TArray<FVector3d> Values;
							for (int32 VerticalIndex = 0;
								VerticalIndex < SideVerticalKnots.Num(); ++VerticalIndex)
							{
								Values.Add(LongitudinalSegmentsByVertical[
									VerticalIndex][PhysicalUIndex].
									ControlPoints[UControl]);
							}
							TArray<FCubicBezierSegment>& Column =
								VerticalSegmentsByUControl.AddDefaulted_GetRef();
							bSideComplete &= BuildNaturalCubicBezierSegments(
								SideVerticalKnots, Values, Column);
						}
						if (!bSideComplete) break;
						const int32 LogicalUIndex = SideSign > 0.0 ?
							PhysicalUIndex :
							LongitudinalSegmentCount - 1 - PhysicalUIndex;
						for (int32 VIndex = 0;
							VIndex + 1 < SideVerticalKnotCount; ++VIndex)
						{
							FPiecewiseTensorBezierCell& Cell =
								SidePatch.Cells.AddDefaulted_GetRef();
							const uint64 CellIndex = static_cast<uint64>(1 +
								LogicalUIndex * (SideVerticalKnotCount - 1) + VIndex);
							Cell.FeatureId = CombineStableIds(
								SidePatch.SurfaceId, CellIndex);
							Cell.PrimitiveId = CombineStableIds(
								SidePatch.PrimitiveId, CellIndex);
							Cell.MinimumU = static_cast<double>(LogicalUIndex) /
								LongitudinalSegmentCount;
							Cell.MaximumU = static_cast<double>(LogicalUIndex + 1) /
								LongitudinalSegmentCount;
							Cell.MinimumV = static_cast<double>(VIndex) /
								(SideVerticalKnotCount - 1);
							Cell.MaximumV = static_cast<double>(VIndex + 1) /
								(SideVerticalKnotCount - 1);
							Cell.LongitudinalParameterScale = 1.0;
							Cell.Surface.DegreeU = Cell.Surface.DegreeV = 3;
							Cell.Surface.ControlPoints.SetNumUninitialized(16);
							for (int32 LogicalUControl = 0;
								LogicalUControl < 4; ++LogicalUControl)
							{
								const int32 PhysicalUControl = SideSign > 0.0 ?
									LogicalUControl : 3 - LogicalUControl;
								for (int32 VControl = 0; VControl < 4; ++VControl)
								{
									Cell.Surface.ControlPoints[
										LogicalUControl * 4 + VControl] =
										VerticalSegmentsByUControl[
											PhysicalUControl][VIndex].
											ControlPoints[VControl];
								}
							}
						}
					}

					double MaximumSideSourceResidualCmObserved = 0.0;
					for (const FPiecewiseTensorBezierCell& Cell : SidePatch.Cells)
					{
						for (const double U : { 0.25, 0.5, 0.75 })
							for (const double V : { 0.25, 0.5, 0.75 })
							{
								const FVector3d Point = Cell.Surface.Evaluate(U, V);
								FVector3d SourcePoint;
								if (!SampleTransverseSource(Point[WallAxis],
									Point[VerticalAxis], SourcePoint))
								{
									MaximumSideSourceResidualCmObserved =
										TNumericLimits<double>::Max();
									continue;
								}
								MaximumSideSourceResidualCmObserved = FMath::Max(
									MaximumSideSourceResidualCmObserved,
									FMath::Abs(SourcePoint[TransverseAxis] -
										Point[TransverseAxis]));
							}
					}
					SidePatch.bSourceResidualCertified = bSideComplete &&
						!SidePatch.Cells.IsEmpty() &&
						MaximumSideSourceResidualCmObserved <=
							MaximumSideSourceResidualCm;
					Algo::Sort(SidePatch.Cells,
						[](const FPiecewiseTensorBezierCell& A,
							const FPiecewiseTensorBezierCell& B)
						{
							return A.PrimitiveId < B.PrimitiveId;
						});
					if (SidePatch.bSourceResidualCertified &&
						SidePatch.BuildQueryApproximation())
					{
						SidePatch.bAuthorityEligible =
							SidePatch.bQueryCollisionEnabled &&
							SidePatch.bApproximationCertified &&
							Algo::AllOf(SidePatch.Adjacencies,
								[](const FPiecewiseTensorBezierAdjacency& Link)
								{
									return Link.bC2ByConstruction;
								});
					}
					// Keep the source-certified rounded middle boundary above, and add
					// a full-height regular section next to the opening.  The rear of the
					// terminal recess changes transverse profile as it turns into the cap,
					// so it must
					// not be folded into this extrusion.  The regular section is where a
					// wheel-sized authority hole previously existed below and above the
					// middle band.
					{
						// Keep the source-constrained extrusion authoritative one wheel radius
						// beyond the formerly exposed terminal edge.  The measured source
						// remains longitudinally regular over this short continuation; the
						// residual certificate below rejects it if that ceases to be true.
						constexpr double RegularRearContinuationCm = 515.0;
						const double RegularEndLongitudinal = OpeningLongitudinal +
							OpeningSign * FMath::Min(RegularRearContinuationCm,
								0.75 * FMath::Abs(RearLongitudinal - OpeningLongitudinal));
						// A finite wheel footprint reaches slightly through the
						// opening plane while its centre is still on the terminal
						// side.  Continue the source-sampled regular profile far
						// enough that those contacts remain in the surface interior
						// instead of acquiring a spurious longitudinal component
						// from the analytical patch boundary.
						constexpr double RegularOpeningOverlapCm = 75.0;
						const double RegularOpeningLongitudinal =
							OpeningLongitudinal -
							OpeningSign * RegularOpeningOverlapCm;
						const double RegularMinimumLongitudinal = FMath::Min(
							RegularOpeningLongitudinal, RegularEndLongitudinal);
						const double RegularMaximumLongitudinal = FMath::Max(
							RegularOpeningLongitudinal, RegularEndLongitudinal);
						// The local quintic interpolant is intentionally denser than the
						// rounded-boundary fit.  This keeps both mirrored source residuals
						// inside the 15 cm curved-surface allowance without changing the
						// source-independent lower extrusion.
						constexpr int32 RegularVerticalKnotCount = 33;
						TArray<double> RegularVerticalKnots;
						for (int32 Index = 0; Index < RegularVerticalKnotCount; ++Index)
						{
							RegularVerticalKnots.Add(FMath::Lerp(
								MinimumVertical, MaximumVertical,
								static_cast<double>(Index) /
									(RegularVerticalKnotCount - 1)));
						}
						{
						// This band is only approximately longitudinally regular: the
						// authored source moves by several centimetres over its finite
						// length.  Extruding one cross-section therefore creates an
						// inward wall that a finite box can hit even when the source mesh
						// is clear.  Sample the complete source graph and interpolate it
						// as a natural bicubic network instead.  Every internal knot is C2
						// and the dense certificate below bounds the remaining curved-mesh
						// interpolation error.
						constexpr int32 RegularLongitudinalKnotCount = 25;
						TArray<double> RegularLongitudinalKnots;
						for (int32 Index = 0;
							Index < RegularLongitudinalKnotCount; ++Index)
						{
							RegularLongitudinalKnots.Add(FMath::Lerp(
								RegularMinimumLongitudinal,
								RegularMaximumLongitudinal,
								static_cast<double>(Index) /
									(RegularLongitudinalKnotCount - 1)));
						}
						auto SmoothStep5Unit = [](const double Value)
						{
							const double T = FMath::Clamp(Value, 0.0, 1.0);
							return T * T * T * (T * (T * 6.0 - 15.0) + 10.0);
						};
						auto SmoothStep7Unit = [](const double Value)
						{
							const double T = FMath::Clamp(Value, 0.0, 1.0);
							return T * T * T * T *
								(35.0 + T * (-84.0 + T * (70.0 - 20.0 * T)));
						};
						auto ApplyRegularProfilePolish = [&](FVector3d& Point,
							const double Longitudinal,
							const double Vertical)
						{
							// This compact curved-junction allowance closes the measured
							// lower-return valley.  It is zero with zero first and second
							// derivatives at both vertical ends.
							constexpr double MaximumLowerSupportPolishCm = 8.0;
							const double PolishSpan =
								SideMinimumVertical - MinimumVertical;
							if (PolishSpan > UE_DOUBLE_SMALL_NUMBER)
							{
								const double T = FMath::Clamp(
									(Vertical - MinimumVertical) / PolishSpan, 0.0, 1.0);
								const double OneMinusT = 1.0 - T;
								const double C2Bell = 64.0 * T * T * T *
									OneMinusT * OneMinusT * OneMinusT;
								Point[TransverseAxis] -= SideSign *
									MaximumLowerSupportPolishCm * C2Bell;
							}

							// The high curved shoulder is one-sided in the source collision
							// mesh.  Keeping its fitted centreline exactly on the triangle
							// sheet makes a finite box straddle that sheet at the regular-side
							// junction even though the source sweep remains clear.  Move only
							// this curved band toward the non-playable side.  Quintic ramps
							// make the displacement and its first two derivatives zero at the
							// band boundaries, and the dense source certificate below keeps
							// the total departure inside the curved-surface allowance.
							// The two source goal meshes differ slightly in this shoulder.
							// Calibrate per longitudinal end, while keeping both lateral
							// sides exactly mirrored, so each fitted cage stays below the
							// same absolute source-distance certificate.
							constexpr double HighShoulderClearanceScale = 0.75;
							const double MaximumHighShoulderClearanceCm =
								(RegularMinimumLongitudinal > 0.0 ? 13.4 : 13.6) *
								HighShoulderClearanceScale;
							const double Rise = SmoothStep5Unit((Vertical - 490.0) / 80.0);
							const double Fall = 1.0 -
								SmoothStep5Unit((Vertical - 690.0) / 50.0);
							const double DistanceFromOpening = OpeningSign *
								(Longitudinal - RegularOpeningLongitudinal);
							// The regular provider already starts 75 cm on the field side
							// of the authored opening.  Spend that overlap on the C2
							// clearance ramp: a finite vehicle box reaches roughly 30 cm
							// into it before its centre crosses the opening.  Delaying the
							// ramp by 20 cm left that leading corner on the source sheet
							// and produced the measured test-768 launch impulse.  Start the C2
							// rise at the overlap boundary so the opening-side shoulder has no
							// artificial plateau; the formula remains identical in all four
							// mirrored sectors.
							constexpr double LongitudinalRiseDelayCm = 0.0;
							const double LongitudinalRise = SmoothStep5Unit(
								(DistanceFromOpening - LongitudinalRiseDelayCm) / 50.0);
							const double LongitudinalFall = 1.0 - SmoothStep7Unit(
								(DistanceFromOpening - 260.0) / 80.0);
							Point[TransverseAxis] += SideSign *
								MaximumHighShoulderClearanceCm * Rise * Fall *
								LongitudinalRise * LongitudinalFall;
						};
						// At the lower side-wall approach, longitudinal variation in the
						// raw triangle samples tilts the sweep normal into the direction of
						// travel and injects a setup impulse.  Use the rear-end transverse
						// generator through the launch band, then join it back to the
						// source-fitted network with a quintic C2 partition.  The measured
						// false-normal witnesses are below 350 cm; retaining the segmented
						// network above that height avoids changing the later route through
						// the shoulder while still removing the launch-edge artifact.
						constexpr double RegularProfileFrozenSeamHeightCm = 350.0;
						bool bProfileComplete = true;
						TArray<FVector3d> FrozenRegularProfileValues;
						for (const double Vertical : RegularVerticalKnots)
						{
							FVector3d Point = FVector3d::ZeroVector;
							bProfileComplete &= SampleTransverseSource(
								RegularEndLongitudinal, Vertical, Point);
							FrozenRegularProfileValues.Add(Point);
						}
						TArray<TArray<FCubicBezierSegment>>
							RegularLongitudinalSegmentsByVertical;
						for (int32 VerticalIndex = 0;
							VerticalIndex < RegularVerticalKnots.Num(); ++VerticalIndex)
						{
							const double Vertical = RegularVerticalKnots[VerticalIndex];
							const double SourceFitBlend = SmoothStep5Unit(
								(Vertical - 475.0) / 85.0);
							TArray<FVector3d> Samples;
							for (const double Longitudinal : RegularLongitudinalKnots)
							{
								FVector3d Point = FVector3d::ZeroVector;
								bProfileComplete &= SampleTransverseSource(
									Longitudinal, Vertical, Point);
								Point[TransverseAxis] = FMath::Lerp(
									FrozenRegularProfileValues[VerticalIndex][TransverseAxis],
									Point[TransverseAxis], SourceFitBlend);
								ApplyRegularProfilePolish(
									Point, Longitudinal, Vertical);
								Samples.Add(Point);
							}
							TArray<FCubicBezierSegment>& Segments =
								RegularLongitudinalSegmentsByVertical.
									AddDefaulted_GetRef();
							bProfileComplete &= BuildNaturalCubicBezierSegments(
								RegularLongitudinalKnots, Samples, Segments);
						}
						TArray<TArray<TArray<FQuinticBezierSegment>>>
							RegularVerticalSegmentsByLongitudinalSegment;
						// The frozen lower profile ends at the last regular knot at or
						// below the transition start.  Build its vertical spline locally
						// so the upper shoulder cannot tilt the lower vehicle envelope.
						int32 RegularProfileTransitionSeamIndex = 1;
						for (int32 Index = 1; Index < RegularVerticalKnots.Num(); ++Index)
						{
							if (RegularVerticalKnots[Index] <=
								RegularProfileFrozenSeamHeightCm + 1.0e-6)
								RegularProfileTransitionSeamIndex = Index;
						}
						for (int32 LongitudinalSegment = 0;
							bProfileComplete && LongitudinalSegment + 1 <
							RegularLongitudinalKnotCount; ++LongitudinalSegment)
						{
							TArray<TArray<FQuinticBezierSegment>>& Columns =
								RegularVerticalSegmentsByLongitudinalSegment.
									AddDefaulted_GetRef();
							for (int32 UControl = 0; UControl < 4; ++UControl)
							{
								TArray<FVector3d> Values;
								for (const TArray<FCubicBezierSegment>& Segments :
									RegularLongitudinalSegmentsByVertical)
								{
									Values.Add(Segments[LongitudinalSegment].
										ControlPoints[UControl]);
								}
								TArray<FQuinticBezierSegment>& VerticalSegments =
									Columns.AddDefaulted_GetRef();
								bProfileComplete &= BuildLocalC2QuinticSegments(
									RegularVerticalKnots, Values,
									RegularProfileTransitionSeamIndex, TransverseAxis,
									VerticalAxis, VerticalSegments);
							}
						}
						TArray<FVector3d> BaseProfileValues;
						for (int32 VerticalIndex = 0;
							VerticalIndex < RegularVerticalKnots.Num(); ++VerticalIndex)
						{
							const double Vertical = RegularVerticalKnots[VerticalIndex];
							FVector3d Point = FrozenRegularProfileValues[VerticalIndex];
							ApplyRegularProfilePolish(
								Point, RegularEndLongitudinal, Vertical);
							BaseProfileValues.Add(Point);
						}
						if (bProfileComplete)
						{
							FPiecewiseTensorBezierPatch ExtrudedSidePatch;
							ExtrudedSidePatch.SourceId = SidePatch.SourceId;
							ExtrudedSidePatch.SurfaceId = CombineStableIds(
								SidePatch.SurfaceId,
								StableStringId(TEXT("RegularProfileExtrusion.V2")));
							ExtrudedSidePatch.PrimitiveId = CombineStableIds(
								SidePatch.PrimitiveId,
								StableStringId(TEXT("RegularProfileExtrusion.V2")));
							ExtrudedSidePatch.CanonicalGroupId =
								SidePatch.CanonicalGroupId;
							ExtrudedSidePatch.MaterialId = SidePatch.MaterialId;
							ExtrudedSidePatch.ObjectType = SidePatch.ObjectType;
							ExtrudedSidePatch.BlockingChannels =
								SidePatch.BlockingChannels;
							ExtrudedSidePatch.bQueryCollisionEnabled =
								SidePatch.bQueryCollisionEnabled;
							const int32 RegularLongitudinalSegmentCount =
								RegularLongitudinalKnotCount - 1;
							// Below the transition seam the surface is an exact longitudinal
							// extrusion.  Represent each vertical band with one longitudinal
							// cell: subdividing that straight direction made penetrating sphere
							// queries select internal cell edges and synthesize false longitudinal
							// normals even though the control net itself was perfectly straight.
							for (int32 VIndex = 0;
								VIndex < RegularProfileTransitionSeamIndex; ++VIndex)
							{
								FPiecewiseTensorBezierCell& Cell =
									ExtrudedSidePatch.Cells.AddDefaulted_GetRef();
								const uint64 CellIndex = CombineStableIds(
									StableStringId(TEXT("RegularProfileLowerLongitudinalSpan.V1")),
									static_cast<uint64>(VIndex + 1));
								Cell.FeatureId = CombineStableIds(
									ExtrudedSidePatch.SurfaceId, CellIndex);
								Cell.PrimitiveId = CombineStableIds(
									ExtrudedSidePatch.PrimitiveId, CellIndex);
								Cell.MinimumU = 0.0;
								Cell.MaximumU = 1.0;
								Cell.MinimumV = static_cast<double>(VIndex) /
									(RegularVerticalKnots.Num() - 1);
								Cell.MaximumV = static_cast<double>(VIndex + 1) /
									(RegularVerticalKnots.Num() - 1);
								Cell.LongitudinalParameterScale = 1.0;
								Cell.Surface.DegreeU = 1;
								Cell.Surface.DegreeV = 5;
								Cell.Surface.ControlPoints.SetNumUninitialized(12);
								for (int32 UControl = 0; UControl < 2; ++UControl)
								{
									const bool bMinimumLongitudinal =
										SideSign > 0.0 ? UControl == 0 : UControl == 1;
									const double Longitudinal = bMinimumLongitudinal ?
										RegularMinimumLongitudinal :
										RegularMaximumLongitudinal;
									for (int32 VControl = 0; VControl < 6; ++VControl)
									{
										FVector3d Point =
											RegularVerticalSegmentsByLongitudinalSegment[
												0][0][VIndex].ControlPoints[VControl];
										Point[WallAxis] = Longitudinal;
										Cell.Surface.ControlPoints[
											UControl * 6 + VControl] = Point;
									}
								}
							}
							for (int32 PhysicalUIndex = 0;
								PhysicalUIndex < RegularLongitudinalSegmentCount;
								++PhysicalUIndex)
							{
								const int32 LogicalUIndex = SideSign > 0.0 ?
									PhysicalUIndex :
									RegularLongitudinalSegmentCount - 1 - PhysicalUIndex;
								for (int32 VIndex = 0;
									VIndex + 1 < RegularVerticalKnots.Num(); ++VIndex)
								{
									if (VIndex < RegularProfileTransitionSeamIndex)
										continue;
									FPiecewiseTensorBezierCell& Cell =
										ExtrudedSidePatch.Cells.AddDefaulted_GetRef();
									const uint64 CellIndex = static_cast<uint64>(1 +
										LogicalUIndex * (RegularVerticalKnots.Num() - 1) +
										VIndex);
									Cell.FeatureId = CombineStableIds(
										ExtrudedSidePatch.SurfaceId, CellIndex);
									Cell.PrimitiveId = CombineStableIds(
										ExtrudedSidePatch.PrimitiveId, CellIndex);
									Cell.MinimumU = static_cast<double>(LogicalUIndex) /
										RegularLongitudinalSegmentCount;
									Cell.MaximumU = static_cast<double>(LogicalUIndex + 1) /
										RegularLongitudinalSegmentCount;
									Cell.MinimumV = static_cast<double>(VIndex) /
										(RegularVerticalKnots.Num() - 1);
									Cell.MaximumV = static_cast<double>(VIndex + 1) /
										(RegularVerticalKnots.Num() - 1);
									Cell.LongitudinalParameterScale = 1.0;
									Cell.Surface.DegreeU = 3;
									Cell.Surface.DegreeV = 5;
									Cell.Surface.ControlPoints.SetNumUninitialized(24);
									for (int32 LogicalUControl = 0;
										LogicalUControl < 4; ++LogicalUControl)
									{
										const int32 PhysicalUControl = SideSign > 0.0 ?
											LogicalUControl : 3 - LogicalUControl;
										for (int32 VControl = 0; VControl < 6; ++VControl)
										{
											Cell.Surface.ControlPoints[
												LogicalUControl * 6 + VControl] =
													RegularVerticalSegmentsByLongitudinalSegment[
														PhysicalUIndex][PhysicalUControl][VIndex].
														ControlPoints[VControl];
										}
									}
								}
							}
							double MaximumExtrusionResidualCm = 0.0;
							double MaximumExtrusionResidualLongitudinal = 0.0;
							double MaximumExtrusionResidualVertical = 0.0;
							double MaximumExtrusionResidualSourceTransverse = 0.0;
							double MaximumExtrusionResidualPatchTransverse = 0.0;
							int32 ExtrusionWitnessCount = 0;
							for (const FPiecewiseTensorBezierCell& Cell :
								ExtrudedSidePatch.Cells)
								for (const double U : { 0.25, 0.5, 0.75 })
									for (const double V : { 0.25, 0.5, 0.75 })
									{
										const FVector3d Point =
											Cell.Surface.Evaluate(U, V);
										const double Longitudinal = Point[WallAxis];
										FVector3d SourcePoint;
										if (!SampleTransverseSource(Longitudinal,
											Point.Z, SourcePoint)) continue;
										++ExtrusionWitnessCount;
										const double ResidualCm = FMath::Abs(
											SourcePoint[TransverseAxis] - Point[TransverseAxis]);
										if (ResidualCm > MaximumExtrusionResidualCm)
										{
											MaximumExtrusionResidualCm = ResidualCm;
											MaximumExtrusionResidualLongitudinal = Longitudinal;
											MaximumExtrusionResidualVertical = Point[VerticalAxis];
											MaximumExtrusionResidualSourceTransverse =
												SourcePoint[TransverseAxis];
											MaximumExtrusionResidualPatchTransverse =
												Point[TransverseAxis];
										}
									}
							// This is a genuinely curved / junction-spanning provider, so use
							// the documented curved-surface allowance.  Planar providers keep
							// their exact source-mesh requirement elsewhere.
							constexpr double MaximumRegularProfilePolishCm = 15.0;
							ExtrudedSidePatch.bSourceResidualCertified =
								ExtrusionWitnessCount ==
									ExtrudedSidePatch.Cells.Num() * 9 &&
								MaximumExtrusionResidualCm <=
									MaximumRegularProfilePolishCm;
							if (ExtrudedSidePatch.bSourceResidualCertified &&
								ExtrudedSidePatch.BuildQueryApproximation())
							{
								ExtrudedSidePatch.bAuthorityEligible =
									ExtrudedSidePatch.bQueryCollisionEnabled &&
									ExtrudedSidePatch.bApproximationCertified &&
									Algo::AllOf(ExtrudedSidePatch.Adjacencies,
										[](const FPiecewiseTensorBezierAdjacency& Link)
										{
											return Link.bC2ByConstruction;
									});
							}
							if (ExtrudedSidePatch.bAuthorityEligible)
							{
								// The source-adaptive inventory can contain another cell for
								// the same triangle sheet.  Let the regular network own only the
								// strict interior of the polished profile window.  Outside that
								// window the adaptive cells remain available for wheel support;
								// boundary cells also retain the provider handoff.
								constexpr double OwnershipBoundaryInsetCm = 0.01;
								constexpr double SameSourceSheetToleranceCm = 25.0;
								constexpr double OwnershipMinimumDistanceFromOpeningCm = 0.0;
								constexpr double OwnershipMaximumDistanceFromOpeningCm = 340.0;
								// The same adaptive sheet also overlaps the regular lower
								// descent used by a vehicle leaving the lateral wall.  Cede
								// that overlap before the high shoulder so the vertical sheet
								// cannot remain authoritative below the authored gutter turn.
								constexpr double OwnershipMinimumVerticalCm = 330.0;
								constexpr double OwnershipMaximumVerticalCm = 740.0;
								for (FPiecewiseTensorBezierPatch& Existing :
									PiecewiseTensorBezierPatches)
								{
									if (!Existing.bAuthorityEligible ||
										Existing.SourceId != ExtrudedSidePatch.SourceId)
										continue;
									const int32 Removed = Existing.Cells.RemoveAll(
										[&](const FPiecewiseTensorBezierCell& Cell)
										{
											const FVector3d Center = Cell.Bounds.GetCenter();
											const double CenterDistanceFromOpening = OpeningSign *
												(Center[WallAxis] - RegularOpeningLongitudinal);
											const double SignedTransverse =
												SideSign * Center[TransverseAxis];
											if (Center[WallAxis] <=
													RegularMinimumLongitudinal +
														OwnershipBoundaryInsetCm ||
												Center[WallAxis] >=
													RegularMaximumLongitudinal -
														OwnershipBoundaryInsetCm ||
												Center[VerticalAxis] <= MinimumVertical +
													OwnershipBoundaryInsetCm ||
												Center[VerticalAxis] >= MaximumVertical -
													OwnershipBoundaryInsetCm ||
												CenterDistanceFromOpening <=
													OwnershipMinimumDistanceFromOpeningCm ||
												CenterDistanceFromOpening >=
													OwnershipMaximumDistanceFromOpeningCm ||
												Center[VerticalAxis] <= OwnershipMinimumVerticalCm ||
												Center[VerticalAxis] >= OwnershipMaximumVerticalCm ||
												SignedTransverse <= 350.0)
												return false;
											FVector3d SourcePoint;
											if (!SampleTransverseSource(Center[WallAxis],
												Center[VerticalAxis], SourcePoint))
												return false;
											return FMath::Abs(Center[TransverseAxis] -
												SourcePoint[TransverseAxis]) <=
												SameSourceSheetToleranceCm;
										});
									if (Removed <= 0) continue;
									Existing.bAuthorityEligible =
										!Existing.Cells.IsEmpty() &&
										Existing.bQueryCollisionEnabled &&
										Existing.bSourceResidualCertified &&
										Existing.BuildQueryApproximation() &&
										Existing.bApproximationCertified;
									UE_LOG(LogTemp, Display, TEXT(
										"[AnalyticRegularProfileOwnership] Surface=%016llX Removed=%d Remaining=%d Side=%+.0f Longitudinal=[%.9g,%.9g] Vertical=[%.9g,%.9g]"),
										Existing.SurfaceId, Removed, Existing.Cells.Num(),
										SideSign, RegularMinimumLongitudinal,
										RegularMaximumLongitudinal, MinimumVertical,
										MaximumVertical);
								}
							}
							if (ExtrudedSidePatch.bAuthorityEligible &&
								SidePatch.bAuthorityEligible)
							{
								// The rounded middle boundary and the regular full-height
								// profile used to overlap from the opening almost to the
								// rear seam.  Both surfaces were source-certified, but a
								// finite wheel query could select different providers for
								// adjacent wheels.  In the source-planar band that produced
								// a false longitudinal normal at bicubic knot boundaries.
								// Once the regular profile is certified, give it exclusive
								// ownership up to its rear endpoint.  Keep the one rounded
								// cell row that crosses that endpoint so the two providers
								// retain a small, source-constrained handoff overlap.
								const int32 RoundedCellsBeforeTrim = SidePatch.Cells.Num();
								SidePatch.Cells.RemoveAll(
									[&](const FPiecewiseTensorBezierCell& Cell)
									{
										return Algo::AllOf(Cell.Surface.ControlPoints,
											[&](const FVector3d& Point)
											{
												return OpeningSign *
													(Point[WallAxis] -
														RegularEndLongitudinal) < -0.01;
											});
									});
								const int32 RoundedCellsRemoved =
									RoundedCellsBeforeTrim - SidePatch.Cells.Num();
								if (RoundedCellsRemoved > 0)
								{
									SidePatch.bAuthorityEligible =
										SidePatch.bSourceResidualCertified &&
										SidePatch.BuildQueryApproximation() &&
										Algo::AllOf(SidePatch.Adjacencies,
											[](const FPiecewiseTensorBezierAdjacency& Link)
											{
												return Link.bC2ByConstruction;
											});
									UE_LOG(LogTemp, Display, TEXT(
										"[AnalyticFiniteTransverseOwnership] Candidate=%016llX Side=%+.0f RemovedRoundedCells=%d RemainingRoundedCells=%d RearEndpoint=%.9g Certified=%d"),
										Candidate.CandidateId, SideSign,
										RoundedCellsRemoved, SidePatch.Cells.Num(),
										RegularEndLongitudinal,
										SidePatch.bAuthorityEligible ? 1 : 0);
								}
							}
							Algo::Sort(ExtrudedSidePatch.Cells,
								[](const FPiecewiseTensorBezierCell& A,
									const FPiecewiseTensorBezierCell& B)
								{
									return A.PrimitiveId < B.PrimitiveId;
								});
							UE_LOG(LogTemp, Display, TEXT(
								"[AnalyticFiniteTransverseProfile] Candidate=%016llX Side=%+.0f Longitudinal=[%.9g,%.9g] Vertical=[%.9g,%.9g] Witnesses=%d SourceResidualCm=%.9g MaximumWitness=(%.9g,%.9g,%.9g->%.9g) Certified=%d"),
								Candidate.CandidateId, SideSign,
								RegularMinimumLongitudinal,
								RegularMaximumLongitudinal,
								MinimumVertical, MaximumVertical,
								ExtrusionWitnessCount, MaximumExtrusionResidualCm,
								MaximumExtrusionResidualLongitudinal,
								MaximumExtrusionResidualVertical,
								MaximumExtrusionResidualSourceTransverse,
								MaximumExtrusionResidualPatchTransverse,
								ExtrudedSidePatch.bAuthorityEligible ? 1 : 0);
							if (ExtrudedSidePatch.bAuthorityEligible)
							{
								PiecewiseTensorBezierPatches.Add(
									MoveTemp(ExtrudedSidePatch));
							}

							// The regular extrusion deliberately stops before the terminal
							// recess changes profile. Complete only its short rear seam with
							// a piecewise-cubic C2 tensor network. At the regular endpoint it
							// preserves the extrusion's position, longitudinal tangent and
							// zero curvature; the other endpoint is sampled from the source.
							// This closes the wheel-sized low gap without extending a frozen
							// transverse profile through the changing rear geometry.
							// Stop before the low source sheet folds back and ceases to be a
							// single-valued transverse graph (the terminal closure owns that
							// fold).  Extending to 150 cm made the centre-out source sweep jump
							// from the inner sheet to the outer sheet near Z=47 cm.
							constexpr double RearSeamLengthCm = 140.0;
							constexpr double MaximumRearSeamSourceResidualCm = 15.0;
							// Use a small tensor network rather than a single high-degree
							// Bernstein strip.  The source turn is piecewise linear at mesh
							// edges; dense cubic cells keep that interpolation bounded while
							// natural splines preserve C2 across both parameter directions.
							constexpr int32 RearSeamLongitudinalKnotCount = 17;
							// The first vertical interval crosses the low terminal fold where
							// the source is not a single-valued transverse graph.  Dedicated
							// terminal-closure cells already own that interval.
							constexpr int32 RearSeamMinimumVerticalSegment = 1;
							const double RearSeamLongitudinal =
								RegularEndLongitudinal + OpeningSign * RearSeamLengthCm;
							const double LongitudinalDelta = RearSeamLongitudinal -
								RegularEndLongitudinal;
							TArray<double> RearSeamLongitudinalKnots;
							for (int32 Index = 0;
								Index < RearSeamLongitudinalKnotCount; ++Index)
							{
								RearSeamLongitudinalKnots.Add(FMath::Lerp(
									FMath::Min(RegularEndLongitudinal, RearSeamLongitudinal),
									FMath::Max(RegularEndLongitudinal, RearSeamLongitudinal),
									static_cast<double>(Index) /
										(RearSeamLongitudinalKnotCount - 1)));
							}
							TArray<TArray<FCubicBezierSegment>>
								RearLongitudinalSegmentsByVertical;
							TArray<double> RearSeamVerticalKnots;
							for (int32 VerticalIndex = RearSeamMinimumVerticalSegment;
								VerticalIndex < RegularVerticalKnots.Num(); ++VerticalIndex)
							{
								RearSeamVerticalKnots.Add(
									RegularVerticalKnots[VerticalIndex]);
							}
							bool bRearSeamComplete = BaseProfileValues.Num() ==
								RegularVerticalKnots.Num() &&
								RearSeamVerticalKnots.Num() >= 2;
							for (int32 VerticalIndex = RearSeamMinimumVerticalSegment;
								bRearSeamComplete &&
								VerticalIndex < RegularVerticalKnots.Num();
								++VerticalIndex)
							{
								FVector3d InnerPoint = BaseProfileValues[VerticalIndex];
								InnerPoint[WallAxis] = RegularEndLongitudinal;
								TArray<FVector3d> Samples;
								Samples.SetNumZeroed(RearSeamLongitudinalKnotCount);
								for (int32 SampleIndex = 0;
									SampleIndex < RearSeamLongitudinalKnotCount; ++SampleIndex)
								{
									const double Longitudinal =
										RearSeamLongitudinalKnots[SampleIndex];
									const double U = FMath::Clamp(
										(Longitudinal - RegularEndLongitudinal) /
											LongitudinalDelta, 0.0, 1.0);
									FVector3d SourcePoint = FVector3d::ZeroVector;
									bRearSeamComplete &= SampleTransverseSource(
										Longitudinal,
										RegularVerticalKnots[VerticalIndex], SourcePoint);
									const double OneMinusU = 1.0 - U;
									const double SmoothStep5 = U * U * U *
										(U * (6.0 * U - 15.0) + 10.0);
									// Lead the interior toward the sampled source without changing
									// position, tangent or curvature at either endpoint.
									const double C2Lead = 7.0 * U * U * U *
										OneMinusU * OneMinusU * OneMinusU;
									const double C2Blend = FMath::Clamp(
										SmoothStep5 + C2Lead, 0.0, 1.0);
									FVector3d Point = InnerPoint;
									Point[WallAxis] = Longitudinal;
									Point[TransverseAxis] = FMath::Lerp(
										InnerPoint[TransverseAxis],
										SourcePoint[TransverseAxis], C2Blend);
									Samples[SampleIndex] = Point;
								}
							TArray<FCubicBezierSegment>& Segments =
								RearLongitudinalSegmentsByVertical.AddDefaulted_GetRef();
							bRearSeamComplete &= BuildNaturalCubicBezierSegments(
								RearSeamLongitudinalKnots, Samples, Segments);
							}

							TArray<TArray<TArray<FCubicBezierSegment>>>
								RearVerticalSegmentsByLongitudinalSegment;
							for (int32 LongitudinalSegment = 0;
								bRearSeamComplete &&
								LongitudinalSegment + 1 <
									RearSeamLongitudinalKnotCount;
								++LongitudinalSegment)
							{
								TArray<TArray<FCubicBezierSegment>>& Columns =
									RearVerticalSegmentsByLongitudinalSegment.
										AddDefaulted_GetRef();
								for (int32 UControl = 0; UControl < 4; ++UControl)
								{
									TArray<FVector3d> Values;
									for (const TArray<FCubicBezierSegment>& Segments :
										RearLongitudinalSegmentsByVertical)
									{
										Values.Add(Segments[LongitudinalSegment].
											ControlPoints[UControl]);
									}
								TArray<FCubicBezierSegment>& VerticalSegments =
									Columns.AddDefaulted_GetRef();
								bRearSeamComplete &= BuildNaturalCubicBezierSegments(
									RearSeamVerticalKnots, Values, VerticalSegments);
							}
							}

							FPiecewiseTensorBezierPatch RearSeamPatch;
							RearSeamPatch.SourceId = SidePatch.SourceId;
							RearSeamPatch.SurfaceId = CombineStableIds(
								SidePatch.SurfaceId,
								StableStringId(TEXT("RegularProfileRearSeamC2.V1")));
							RearSeamPatch.PrimitiveId = CombineStableIds(
								SidePatch.PrimitiveId,
								StableStringId(TEXT("RegularProfileRearSeamC2.V1")));
							RearSeamPatch.CanonicalGroupId = StableStringId(
								TEXT("PiecewiseFiniteTransverseRearSeamC2.Canonical.V1"));
							RearSeamPatch.MaterialId = SidePatch.MaterialId;
							RearSeamPatch.ObjectType = SidePatch.ObjectType;
							RearSeamPatch.BlockingChannels = SidePatch.BlockingChannels;
							RearSeamPatch.bQueryCollisionEnabled =
								SidePatch.bQueryCollisionEnabled;
							for (int32 CellLongitudinalSegment = 0;
								bRearSeamComplete &&
								CellLongitudinalSegment + 1 <
									RearSeamLongitudinalKnotCount;
								++CellLongitudinalSegment)
							{
								for (int32 SeamVerticalIndex = 0;
									SeamVerticalIndex + 1 < RearSeamVerticalKnots.Num();
									++SeamVerticalIndex)
								{
									const int32 VerticalIndex = SeamVerticalIndex +
										RearSeamMinimumVerticalSegment;
									FPiecewiseTensorBezierCell& Cell =
										RearSeamPatch.Cells.AddDefaulted_GetRef();
									const uint64 CellIndex = static_cast<uint64>(1 +
										CellLongitudinalSegment * (RegularVerticalKnots.Num() - 1) +
										VerticalIndex);
									Cell.FeatureId = CombineStableIds(
										RearSeamPatch.SurfaceId, CellIndex);
									Cell.PrimitiveId = CombineStableIds(
										RearSeamPatch.PrimitiveId, CellIndex);
									Cell.MinimumU = static_cast<double>(CellLongitudinalSegment) /
										(RearSeamLongitudinalKnotCount - 1);
									Cell.MaximumU = static_cast<double>(CellLongitudinalSegment + 1) /
										(RearSeamLongitudinalKnotCount - 1);
									Cell.MinimumV = static_cast<double>(VerticalIndex) /
										(RegularVerticalKnots.Num() - 1);
									Cell.MaximumV = static_cast<double>(VerticalIndex + 1) /
										(RegularVerticalKnots.Num() - 1);
									Cell.LongitudinalParameterScale = 1.0;
									Cell.Surface.DegreeU = Cell.Surface.DegreeV = 3;
									Cell.Surface.ControlPoints.SetNumUninitialized(16);
									for (int32 UControl = 0; UControl < 4; ++UControl)
										for (int32 VControl = 0; VControl < 4; ++VControl)
										{
											Cell.Surface.ControlPoints[
												UControl * 4 + VControl] =
											RearVerticalSegmentsByLongitudinalSegment[
												CellLongitudinalSegment][UControl][SeamVerticalIndex].
													ControlPoints[VControl];
										}
								}
							}
							if (bRearSeamComplete && !RearSeamPatch.Cells.IsEmpty())
							{
								const FVector3d CenterNormal = RearSeamPatch.Cells[
									RearSeamPatch.Cells.Num() / 2].Surface.
										EvaluateNormal(0.5, 0.5);
								if (CenterNormal[TransverseAxis] * -SideSign < 0.0)
								{
									for (FPiecewiseTensorBezierCell& Cell :
										RearSeamPatch.Cells)
									{
										for (int32 VControl = 0; VControl < 4; ++VControl)
										{
											for (int32 UControl = 0;
																UControl < 2;
																++UControl)
											{
												Swap(Cell.Surface.ControlPoints[
													UControl * 4 + VControl],
													Cell.Surface.ControlPoints[
																		(3 - UControl) * 4 + VControl]);
											}
										}
									}
								}
							}
							double MaximumRearSeamSourceResidualCmObserved = 0.0;
							double MaximumRearSeamResidualLongitudinal = 0.0;
							double MaximumRearSeamResidualVertical = 0.0;
							double MaximumRearSeamResidualSourceTransverse = 0.0;
							double MaximumRearSeamResidualPatchTransverse = 0.0;
							int32 RearSeamWitnessCount = 0;
							for (const FPiecewiseTensorBezierCell& Cell :
								RearSeamPatch.Cells)
								for (const double U : { 0.125, 0.25, 0.5, 0.75, 0.875 })
									for (const double V : { 0.25, 0.5, 0.75 })
									{
										const FVector3d Point = Cell.Surface.Evaluate(U, V);
										FVector3d SourcePoint;
										if (!SampleTransverseSource(Point[WallAxis],
											Point[VerticalAxis], SourcePoint))
										{
											bRearSeamComplete = false;
											continue;
										}
										++RearSeamWitnessCount;
										const double ResidualCm = FMath::Abs(
											SourcePoint[TransverseAxis] - Point[TransverseAxis]);
										if (ResidualCm > MaximumRearSeamSourceResidualCmObserved)
										{
											MaximumRearSeamSourceResidualCmObserved = ResidualCm;
											MaximumRearSeamResidualLongitudinal = Point[WallAxis];
											MaximumRearSeamResidualVertical = Point[VerticalAxis];
											MaximumRearSeamResidualSourceTransverse =
												SourcePoint[TransverseAxis];
											MaximumRearSeamResidualPatchTransverse =
												Point[TransverseAxis];
										}
									}
							RearSeamPatch.bSourceResidualCertified =
								bRearSeamComplete && RearSeamWitnessCount > 0 &&
								MaximumRearSeamSourceResidualCmObserved <=
									MaximumRearSeamSourceResidualCm;
							Algo::Sort(RearSeamPatch.Cells,
								[](const FPiecewiseTensorBezierCell& A,
									const FPiecewiseTensorBezierCell& B)
								{
									return A.PrimitiveId < B.PrimitiveId;
								});
							if (RearSeamPatch.bSourceResidualCertified &&
								RearSeamPatch.BuildQueryApproximation())
							{
								RearSeamPatch.bAuthorityEligible =
									RearSeamPatch.bQueryCollisionEnabled &&
									RearSeamPatch.bApproximationCertified &&
									Algo::AllOf(RearSeamPatch.Adjacencies,
										[](const FPiecewiseTensorBezierAdjacency& Link)
										{
											return Link.bC2ByConstruction;
										});
							}
							UE_LOG(LogTemp, Display, TEXT(
								"[AnalyticFiniteTransverseRearSeam] Candidate=%016llX Side=%+.0f Cells=%d Longitudinal=[%.9g,%.9g] Witnesses=%d SourceResidualCm=%.9g MaximumWitness=(%.9g,%.9g,%.9g->%.9g) Certified=%d"),
								Candidate.CandidateId, SideSign,
								RearSeamPatch.Cells.Num(), RegularEndLongitudinal,
								RearSeamLongitudinal, RearSeamWitnessCount,
								MaximumRearSeamSourceResidualCmObserved,
								MaximumRearSeamResidualLongitudinal,
								MaximumRearSeamResidualVertical,
								MaximumRearSeamResidualSourceTransverse,
								MaximumRearSeamResidualPatchTransverse,
								RearSeamPatch.bAuthorityEligible ? 1 : 0);
							if (RearSeamPatch.bAuthorityEligible)
							{
								PiecewiseTensorBezierPatches.Add(
									MoveTemp(RearSeamPatch));
							}
						}
					}
					}
					UE_LOG(LogTemp, Display, TEXT(
						"[AnalyticFiniteTransverseBoundary] Candidate=%016llX Side=%+.0f Cells=%d Longitudinal=[%.9g,%.9g] Vertical=[%.9g,%.9g] SourceResidualCm=%.9g Certified=%d"),
						Candidate.CandidateId, SideSign, SidePatch.Cells.Num(),
						MinimumLongitudinal, MaximumLongitudinal,
						SideMinimumVertical, SideMaximumVertical,
						MaximumSideSourceResidualCmObserved,
						SidePatch.bAuthorityEligible ? 1 : 0);
					if (SidePatch.bAuthorityEligible)
					{
						PiecewiseTensorBezierPatches.Add(MoveTemp(SidePatch));
					}
				}
			}
		}

		// The lower and upper turns become singular in the cap's X=f(Y,Z)
		// parameterization as their normals approach the horizontal support
		// planes. Continue them as compact Z=f(Y,X) bands instead. Each band is
		// an extrusion of one source-certified natural cubic and therefore keeps
		// exact C2 adjacency without inheriting the residual mesh facets.
		const double ReturnHalfWidth = FMath::Min(
			0.50 * TransverseHalfExtent, 500.0);
		// Keep a small source-certified overlap on the nearly horizontal upper
		// return.  A suspension sphere can otherwise leave the finite cap just
		// outside the nominal half-width before the adjacent curved provider owns
		// the query.  This extends only the analytic domain; the evaluated support
		// remains the same centerline extrusion and is still source-certified
		// below.
		constexpr double UpperReturnSupportOverlapCm = 120.0;
		const double UpperReturnHalfWidth = FMath::Min(
			ReturnHalfWidth + UpperReturnSupportOverlapCm,
			TransverseHalfExtent);
		// Both opposed terminal candidates exceed this common cap. Using the cap
		// rather than their slightly different raw mesh extents keeps the playable
		// lower return exactly symmetric while remaining source-certifiable.
		const double LowerReturnHalfWidth = FMath::Min(
			0.86 * TransverseHalfExtent, 825.0);
		const double DomainInset = FMath::Max(
			25.0, 0.04 * TransverseHalfExtent);
		const double InteriorEndCoordinate =
			WallCoordinate + OpeningSign * DomainInset;
		constexpr int32 ReturnKnotCount = 65;
		constexpr double MaximumReturnSourceResidualCm = 3.0;
		TOptional<FPiecewiseTensorBezierPatch> LowerReturnForSeamCompletion;
		auto BuildExtrudedReturn = [&](const bool bUpperReturn)
		{
			const double JoinVertical = bUpperReturn ?
				MaximumVertical : MinimumVertical;
			FVector3d JoinPoint;
			if (!SampleSource(0.0, JoinVertical, JoinPoint))
			{
				return;
			}
			const double MinimumProbeVertical = bUpperReturn ?
				InteriorBaseline + 0.45 * VerticalSpan : InteriorBaseline - 1.0;
			const double MaximumProbeVertical = bUpperReturn ?
				InteriorTop + 200.0 : InteriorBaseline + 0.45 * VerticalSpan;
			auto SampleReturnSource = [&](const double Transverse,
				const double Longitudinal, FVector3d& OutPoint)
			{
				FWorldQuery Query;
				Query.Start[WallAxis] = Query.End[WallAxis] = Longitudinal;
				Query.Start[TransverseAxis] = Query.End[TransverseAxis] =
					Transverse;
				// Enter each playable support from its interior side so the
				// returned normal has the same orientation as runtime contact.
				// In particular, probing the lower return from below observes the
				// back face of the authored floor and rejects an otherwise valid
				// join point.
				Query.Start[VerticalAxis] = bUpperReturn ?
					MinimumProbeVertical : MaximumProbeVertical;
				Query.End[VerticalAxis] = bUpperReturn ?
					MaximumProbeVertical : MinimumProbeVertical;
				Query.RequiredSourceId = SourceTriangle.SourceId;
				Query.bIncludeTriangles = true;
				const FWorldHit Hit = SourceQueryService.Sweep(Query);
				const double ExpectedVerticalNormalSign = bUpperReturn ? -1.0 : 1.0;
				if (!Hit.bHit || OpeningSign *
						(Hit.Point[WallAxis] - WallCoordinate) <
							DomainInset - 1.0 ||
					ExpectedVerticalNormalSign * Hit.Normal[VerticalAxis] < 0.05)
				{
					return false;
				}
				OutPoint = Hit.Point;
				return true;
			};
			FVector3d JoinSourcePoint;
			if (!SampleReturnSource(0.0, JoinPoint[WallAxis], JoinSourcePoint))
			{
				UE_LOG(LogTemp, Display, TEXT(
					"[AnalyticExtrudedReturn] Candidate=%016llX Upper=%d JoinUnavailable=1 Join=%s"),
					Candidate.CandidateId, bUpperReturn ? 1 : 0,
					*JoinPoint.ToString());
				return;
			}
			double AvailableInteriorCoordinate = JoinPoint[WallAxis];
			bool bInteriorCoordinateFound = true;
			constexpr int32 InteriorSearchIntervals = 256;
			// Retain only the contiguous source interval connected to the cap.
			// Searching from the remote endpoint can jump across a disjoint mesh
			// island and later make a knot in the intervening gap fail.
			for (int32 SearchIndex = 1;
				SearchIndex <= InteriorSearchIntervals; ++SearchIndex)
			{
				const double Alpha = static_cast<double>(SearchIndex) /
					InteriorSearchIntervals;
				const double Longitudinal = FMath::Lerp(
					JoinPoint[WallAxis], InteriorEndCoordinate, Alpha);
				FVector3d SourcePoint;
				if (SampleReturnSource(0.0, Longitudinal, SourcePoint))
				{
					AvailableInteriorCoordinate = Longitudinal;
				}
				else
				{
					break;
				}
			}
			if (!bInteriorCoordinateFound)
			{
				return;
			}
			// Natural-cubic abscissae must be strictly increasing.  Surface
			// orientation is controlled independently by the transverse axis
			// below, which also makes the two opposed openings exactly symmetric.
			const double FirstLongitudinal = FMath::Min(
				JoinPoint[WallAxis], AvailableInteriorCoordinate);
			const double LastLongitudinal = FMath::Max(
				JoinPoint[WallAxis], AvailableInteriorCoordinate);
			if (FMath::Abs(LastLongitudinal - FirstLongitudinal) <= 20.0)
			{
				return;
			}
			TArray<double> ReturnKnots;
			TArray<FVector3d> ReturnValues;
			bool bReturnComplete = true;
			for (int32 KnotIndex = 0; KnotIndex < ReturnKnotCount; ++KnotIndex)
			{
				const double Alpha = static_cast<double>(KnotIndex) /
					(ReturnKnotCount - 1);
				const double Longitudinal = FMath::Lerp(
					FirstLongitudinal, LastLongitudinal, Alpha);
				FVector3d Point = FVector3d::ZeroVector;
				const bool bKnotAvailable = SampleReturnSource(
					0.0, Longitudinal, Point);
				bReturnComplete &= bKnotAvailable;
				if (!bKnotAvailable)
				{
					UE_LOG(LogTemp, Display, TEXT(
						"[AnalyticExtrudedReturn] Candidate=%016llX Upper=%d MissingKnot=%d Longitudinal=%.9g"),
						Candidate.CandidateId, bUpperReturn ? 1 : 0,
						KnotIndex, Longitudinal);
				}
				ReturnKnots.Add(Longitudinal);
				ReturnValues.Add(Point);
			}
			TArray<FCubicBezierSegment> ReturnSegments;
			bReturnComplete &= BuildNaturalCubicBezierSegments(
				ReturnKnots, ReturnValues, ReturnSegments);
			if (!bReturnComplete || ReturnSegments.Num() != ReturnKnotCount - 1)
			{
				UE_LOG(LogTemp, Display, TEXT(
					"[AnalyticExtrudedReturn] Candidate=%016llX Upper=%d Incomplete=1"),
					Candidate.CandidateId, bUpperReturn ? 1 : 0);
				return;
			}
			const FString FamilyName = bUpperReturn ?
				TEXT("PiecewiseExtrudedUpperReturnC2.V1") :
				TEXT("PiecewiseExtrudedLowerReturnC2.V1");
			FPiecewiseTensorBezierPatch ReturnPatch;
			ReturnPatch.SourceId = SourceTriangle.SourceId;
			ReturnPatch.SurfaceId = CombineStableIds(Candidate.CandidateId,
				StableStringId(FamilyName));
			ReturnPatch.PrimitiveId = CombineStableIds(ReturnPatch.SurfaceId,
				StableStringId(TEXT("NaturalExtrusion")));
			ReturnPatch.CanonicalGroupId = StableStringId(FamilyName +
				TEXT(".Canonical"));
			ReturnPatch.MaterialId = SourceTriangle.MaterialId;
			ReturnPatch.ObjectType = SourceTriangle.ObjectType;
			ReturnPatch.BlockingChannels = SourceTriangle.BlockingChannels;
			ReturnPatch.bQueryCollisionEnabled =
				SourceTriangle.bQueryCollisionEnabled;
			if (bUpperReturn)
			{
				// With longitudinal V increasing in world space, U=+Y gives -Z
				// on the upper return.
				for (int32 SegmentIndex = 0;
					SegmentIndex < ReturnSegments.Num(); ++SegmentIndex)
				{
					FPiecewiseTensorBezierCell& Cell =
						ReturnPatch.Cells.AddDefaulted_GetRef();
					Cell.FeatureId = CombineStableIds(ReturnPatch.SurfaceId,
						static_cast<uint64>(SegmentIndex + 1));
					Cell.PrimitiveId = CombineStableIds(ReturnPatch.PrimitiveId,
						static_cast<uint64>(SegmentIndex + 1));
					Cell.MinimumU = 0.0;
					Cell.MaximumU = 1.0;
					Cell.MinimumV = static_cast<double>(SegmentIndex) /
						ReturnSegments.Num();
					Cell.MaximumV = static_cast<double>(SegmentIndex + 1) /
						ReturnSegments.Num();
					Cell.LongitudinalParameterScale = 1.0;
					Cell.Surface.DegreeU = 1;
					Cell.Surface.DegreeV = 3;
					Cell.Surface.ControlPoints.SetNumUninitialized(8);
					for (int32 UControl = 0; UControl < 2; ++UControl)
					{
						const double Transverse = UControl == 0 ?
							-UpperReturnHalfWidth : UpperReturnHalfWidth;
						for (int32 VControl = 0; VControl < 4; ++VControl)
						{
							FVector3d Point = ReturnSegments[SegmentIndex].
								ControlPoints[VControl];
							Point[TransverseAxis] = Transverse;
							Cell.Surface.ControlPoints[
								UControl * 4 + VControl] = Point;
						}
					}
				}
			}
			else
			{
				// The lower support is not a flat extrusion outside its central
				// band. Sample the two finite lateral transition channels vertically and
				// fit the
				// complete transverse/longitudinal network as one natural bicubic
				// surface. This keeps all internal joins C2 and lets the dense source
				// residual below decide whether the wider domain may own authority.
				const double InnerMiddleHalfWidth =
					0.58 * TransverseHalfExtent;
				const double MiddleHalfWidth =
					0.66 * TransverseHalfExtent;
				const double OuterMiddleHalfWidth =
					0.75 * TransverseHalfExtent;
				const TArray<double> TransverseKnots = {
					-LowerReturnHalfWidth, -OuterMiddleHalfWidth,
					-MiddleHalfWidth, -InnerMiddleHalfWidth, -ReturnHalfWidth,
					0.0,
					ReturnHalfWidth, InnerMiddleHalfWidth,
					MiddleHalfWidth, OuterMiddleHalfWidth,
					LowerReturnHalfWidth };
				TArray<TArray<FCubicBezierSegment>>
					TransverseSegmentsByLongitudinal;
				bool bCompleteGrid = true;
				for (const double Longitudinal : ReturnKnots)
				{
					TArray<FVector3d> Values;
					for (const double Transverse : TransverseKnots)
					{
						FVector3d Point = FVector3d::ZeroVector;
						const bool bAvailable = SampleReturnSource(
							Transverse, Longitudinal, Point);
						bCompleteGrid &= bAvailable;
						Values.Add(Point);
					}
					TArray<FCubicBezierSegment>& Row =
						TransverseSegmentsByLongitudinal.AddDefaulted_GetRef();
					bCompleteGrid &= BuildNaturalCubicBezierSegments(
						TransverseKnots, Values, Row);
				}
				const int32 TransverseSegmentCount =
					TransverseKnots.Num() - 1;
				for (int32 PhysicalUIndex = 0;
					bCompleteGrid && PhysicalUIndex < TransverseSegmentCount;
					++PhysicalUIndex)
				{
					TArray<TArray<FCubicBezierSegment>>
						LongitudinalSegmentsByUControl;
					for (int32 UControl = 0; UControl < 4; ++UControl)
					{
						TArray<FVector3d> Values;
						for (int32 LongitudinalIndex = 0;
							LongitudinalIndex < ReturnKnots.Num();
							++LongitudinalIndex)
						{
							Values.Add(TransverseSegmentsByLongitudinal[
								LongitudinalIndex][PhysicalUIndex].
								ControlPoints[UControl]);
						}
						TArray<FCubicBezierSegment>& Column =
							LongitudinalSegmentsByUControl.AddDefaulted_GetRef();
						bCompleteGrid &= BuildNaturalCubicBezierSegments(
							ReturnKnots, Values, Column);
					}
					if (!bCompleteGrid)
					{
						break;
					}
					// Reverse the transverse parameter so dU x dV points toward
					// the playable (+vertical) side of the lower support.
					const int32 LogicalUIndex =
						TransverseSegmentCount - 1 - PhysicalUIndex;
					for (int32 VIndex = 0;
						VIndex < ReturnSegments.Num(); ++VIndex)
					{
						FPiecewiseTensorBezierCell& Cell =
							ReturnPatch.Cells.AddDefaulted_GetRef();
						const uint64 CellIndex = static_cast<uint64>(1 +
							LogicalUIndex * ReturnSegments.Num() + VIndex);
						Cell.FeatureId = CombineStableIds(
							ReturnPatch.SurfaceId, CellIndex);
						Cell.PrimitiveId = CombineStableIds(
							ReturnPatch.PrimitiveId, CellIndex);
						Cell.MinimumU = static_cast<double>(LogicalUIndex) /
							TransverseSegmentCount;
						Cell.MaximumU = static_cast<double>(LogicalUIndex + 1) /
							TransverseSegmentCount;
						Cell.MinimumV = static_cast<double>(VIndex) /
							ReturnSegments.Num();
						Cell.MaximumV = static_cast<double>(VIndex + 1) /
							ReturnSegments.Num();
						Cell.LongitudinalParameterScale = 1.0;
						Cell.Surface.DegreeU = Cell.Surface.DegreeV = 3;
						Cell.Surface.ControlPoints.SetNumUninitialized(16);
						for (int32 LogicalUControl = 0;
							LogicalUControl < 4; ++LogicalUControl)
						{
							const int32 PhysicalUControl =
								3 - LogicalUControl;
							for (int32 VControl = 0; VControl < 4;
								++VControl)
							{
								Cell.Surface.ControlPoints[
									LogicalUControl * 4 + VControl] =
									LongitudinalSegmentsByUControl[
										PhysicalUControl][VIndex].
										ControlPoints[VControl];
							}
						}
					}
				}
				if (!bCompleteGrid)
				{
					UE_LOG(LogTemp, Display, TEXT(
						"[AnalyticExtrudedReturn] Candidate=%016llX Upper=0 WideGridIncomplete=1"),
						Candidate.CandidateId);
					return;
				}
			}
			double MaximumSourceResidualCm = 0.0;
			for (const FPiecewiseTensorBezierCell& Cell : ReturnPatch.Cells)
			{
				for (const double U : { 0.0, 0.25, 0.5, 0.75, 1.0 })
					for (const double V : { 0.25, 0.5, 0.75 })
					{
						const FVector3d Point = Cell.Surface.Evaluate(U, V);
						FVector3d SourcePoint;
						if (!SampleReturnSource(Point[TransverseAxis],
							Point[WallAxis], SourcePoint))
						{
							MaximumSourceResidualCm =
								TNumericLimits<double>::Max();
							continue;
						}
						MaximumSourceResidualCm = FMath::Max(
							MaximumSourceResidualCm,
							FMath::Abs(SourcePoint[VerticalAxis] -
								Point[VerticalAxis]));
					}
			}
			ReturnPatch.bSourceResidualCertified =
				MaximumSourceResidualCm <= MaximumReturnSourceResidualCm;
			Algo::Sort(ReturnPatch.Cells,
				[](const FPiecewiseTensorBezierCell& A,
					const FPiecewiseTensorBezierCell& B)
				{
					return A.PrimitiveId < B.PrimitiveId;
				});
			if (ReturnPatch.bSourceResidualCertified &&
				ReturnPatch.BuildQueryApproximation())
			{
				ReturnPatch.bAuthorityEligible =
					ReturnPatch.bQueryCollisionEnabled &&
					ReturnPatch.bApproximationCertified &&
					Algo::AllOf(ReturnPatch.Adjacencies,
						[](const FPiecewiseTensorBezierAdjacency& Link)
						{
								return Link.bC2ByConstruction;
							});
			}
			if (bUpperReturn && ReturnPatch.bAuthorityEligible)
			{
				// The central upper return is an extrusion, while each finite side
				// boundary ends on the same source profile at MaximumVertical. Join
				// those two certified curves explicitly. A quintic Hermite section
				// preserves position, tangent and the zero endpoint curvature of the
				// natural splines, so a wheelbase cannot be split between the broad
				// open-rim provider and the finite side provider in the intervening
				// quadrant.
				constexpr int32 UpperSeamLongitudinalKnotCount = 17;
				constexpr double MaximumUpperSeamPolishCm = 15.0;
				const double UpperSeamOpeningLongitudinal =
					WallCoordinate + OpeningSign * 150.0;
				// Extend past the old 500 cm strip far enough to cover the remaining
				// source-continuous upper-support band.  The return endpoint itself is
				// excluded: there the transverse source sweep can change branches, so
				// joining all the way to it would no longer be a source-fit surface.
				const double UpperSeamEndLongitudinal =
					UpperSeamOpeningLongitudinal + OpeningSign * 650.0;
				const double UpperSeamMinimumLongitudinal = FMath::Min(
					UpperSeamOpeningLongitudinal, UpperSeamEndLongitudinal);
				const double UpperSeamMaximumLongitudinal = FMath::Max(
					UpperSeamOpeningLongitudinal, UpperSeamEndLongitudinal);
				for (const double SideSign : { -1.0, 1.0 })
				{
					TArray<double> UpperSeamLongitudinalKnots;
					TArray<TArray<FVector3d>> ControlsByLongitudinal;
					bool bUpperSeamComplete = true;
					double MaximumUpperSeamPolishCmObserved = 0.0;
					double MaximumUpperSeamSourceResidualCmObserved = 0.0;
					for (int32 KnotIndex = 0;
						KnotIndex < UpperSeamLongitudinalKnotCount; ++KnotIndex)
					{
						const double Longitudinal = FMath::Lerp(
							UpperSeamMinimumLongitudinal,
							UpperSeamMaximumLongitudinal,
							static_cast<double>(KnotIndex) /
								(UpperSeamLongitudinalKnotCount - 1));
						UpperSeamLongitudinalKnots.Add(Longitudinal);
						FVector3d InnerPoint = FVector3d::ZeroVector;
						bUpperSeamComplete &= SampleReturnSource(
							0.0, Longitudinal, InnerPoint);
						InnerPoint[TransverseAxis] =
							SideSign * UpperReturnHalfWidth;

						FWorldQuery SideQuery;
						SideQuery.Start[WallAxis] =
							SideQuery.End[WallAxis] = Longitudinal;
						SideQuery.Start[TransverseAxis] = 0.0;
						SideQuery.End[TransverseAxis] = SideSign *
							(TransverseHalfExtent + 500.0);
						SideQuery.Start[VerticalAxis] =
							SideQuery.End[VerticalAxis] = MaximumVertical;
						SideQuery.RequiredSourceId = SourceTriangle.SourceId;
						SideQuery.bIncludeTriangles = true;
						const FWorldHit SideHit =
							SourceQueryService.Sweep(SideQuery);
						const double OuterRadius = SideSign *
							SideHit.Point[TransverseAxis];
						const double RadiusSpan =
							OuterRadius - UpperReturnHalfWidth;
						const bool bSideAvailable = SideHit.bHit &&
							-SideHit.Normal[VerticalAxis] >= 0.05 &&
							RadiusSpan > 50.0;
						bUpperSeamComplete &= bSideAvailable;

						TArray<FVector3d>& Controls =
							ControlsByLongitudinal.AddDefaulted_GetRef();
						Controls.SetNumZeroed(6);
						if (!bUpperSeamComplete)
						{
							continue;
						}
						const FVector3d OuterPoint = SideHit.Point;
						FVector3d InnerDerivative = FVector3d::ZeroVector;
						InnerDerivative[TransverseAxis] = SideSign * RadiusSpan;
						FVector3d OuterDerivative = InnerDerivative;
						OuterDerivative[VerticalAxis] = RadiusSpan *
							(-SideSign * SideHit.Normal[TransverseAxis] /
								SideHit.Normal[VerticalAxis]);
						Controls[0] = InnerPoint;
						Controls[1] = InnerPoint + InnerDerivative / 5.0;
						Controls[2] = InnerPoint + 2.0 * InnerDerivative / 5.0;
						Controls[3] = OuterPoint - 2.0 * OuterDerivative / 5.0;
						Controls[4] = OuterPoint - OuterDerivative / 5.0;
						Controls[5] = OuterPoint;

						for (int32 SampleIndex = 0; SampleIndex <= 32;
							++SampleIndex)
						{
							const double T = static_cast<double>(SampleIndex) / 32.0;
							const FVector3d Point = EvaluateBezierControlPolygon(
								MakeArrayView(Controls), T);
							const FVector3d ChordPoint = FMath::Lerp(
								InnerPoint, OuterPoint, T);
							MaximumUpperSeamPolishCmObserved = FMath::Max(
								MaximumUpperSeamPolishCmObserved,
								FVector3d::Distance(ChordPoint, Point));
							FVector3d SourcePoint;
							const bool bSourceAvailable = SampleReturnSource(
								Point[TransverseAxis], Point[WallAxis], SourcePoint);
							bUpperSeamComplete &= bSourceAvailable;
							if (bSourceAvailable)
							{
								MaximumUpperSeamSourceResidualCmObserved = FMath::Max(
									MaximumUpperSeamSourceResidualCmObserved,
									FMath::Abs(SourcePoint[VerticalAxis] -
										Point[VerticalAxis]));
							}
						}
					}

					TArray<TArray<FCubicBezierSegment>>
						LongitudinalSegmentsByTransverseControl;
					for (int32 TransverseControl = 0;
						bUpperSeamComplete && TransverseControl < 6;
						++TransverseControl)
					{
						TArray<FVector3d> Values;
						for (const TArray<FVector3d>& Controls :
							ControlsByLongitudinal)
						{
							Values.Add(Controls[TransverseControl]);
						}
						TArray<FCubicBezierSegment>& Segments =
							LongitudinalSegmentsByTransverseControl.
								AddDefaulted_GetRef();
						bUpperSeamComplete &= BuildNaturalCubicBezierSegments(
							UpperSeamLongitudinalKnots, Values, Segments);
					}

					FPiecewiseTensorBezierPatch UpperSeamPatch;
					UpperSeamPatch.SourceId = ReturnPatch.SourceId;
					UpperSeamPatch.SurfaceId = CombineStableIds(
						Candidate.CandidateId, CombineStableIds(
							StableStringId(TEXT("PiecewiseUpperReturnSeamC2.V1")),
							SideSign < 0.0 ? 1ull : 2ull));
					UpperSeamPatch.PrimitiveId = CombineStableIds(
						UpperSeamPatch.SurfaceId,
						StableStringId(TEXT("QuinticHermiteStrip")));
					UpperSeamPatch.CanonicalGroupId = StableStringId(
						TEXT("PiecewiseUpperReturnSeamC2.Canonical.V1"));
					UpperSeamPatch.MaterialId = ReturnPatch.MaterialId;
					UpperSeamPatch.ObjectType = ReturnPatch.ObjectType;
					UpperSeamPatch.BlockingChannels =
						ReturnPatch.BlockingChannels;
					UpperSeamPatch.bQueryCollisionEnabled =
						ReturnPatch.bQueryCollisionEnabled;
					for (int32 VIndex = 0; bUpperSeamComplete &&
						VIndex + 1 < UpperSeamLongitudinalKnotCount; ++VIndex)
					{
						FPiecewiseTensorBezierCell& Cell =
							UpperSeamPatch.Cells.AddDefaulted_GetRef();
						const uint64 CellIndex = static_cast<uint64>(VIndex + 1);
						Cell.FeatureId = CombineStableIds(
							UpperSeamPatch.SurfaceId, CellIndex);
						Cell.PrimitiveId = CombineStableIds(
							UpperSeamPatch.PrimitiveId, CellIndex);
						Cell.MinimumU = 0.0;
						Cell.MaximumU = 1.0;
						Cell.MinimumV = static_cast<double>(VIndex) /
							(UpperSeamLongitudinalKnotCount - 1);
						Cell.MaximumV = static_cast<double>(VIndex + 1) /
							(UpperSeamLongitudinalKnotCount - 1);
						Cell.LongitudinalParameterScale = 1.0;
						Cell.Surface.DegreeU = 5;
						Cell.Surface.DegreeV = 3;
						Cell.Surface.ControlPoints.SetNumUninitialized(24);
						for (int32 UControl = 0; UControl < 6; ++UControl)
							for (int32 VControl = 0; VControl < 4; ++VControl)
							{
								Cell.Surface.ControlPoints[UControl * 4 + VControl] =
									LongitudinalSegmentsByTransverseControl[UControl]
										[VIndex].ControlPoints[VControl];
							}
					}
					if (bUpperSeamComplete && !UpperSeamPatch.Cells.IsEmpty())
					{
						const FVector3d CenterNormal = UpperSeamPatch.Cells[
							UpperSeamPatch.Cells.Num() / 2].Surface.
								EvaluateNormal(0.5, 0.5);
						if (CenterNormal[VerticalAxis] > 0.0)
						{
							for (FPiecewiseTensorBezierCell& Cell :
								UpperSeamPatch.Cells)
							{
								for (int32 VControl = 0; VControl < 4; ++VControl)
								{
									Swap(Cell.Surface.ControlPoints[0 * 4 + VControl],
										Cell.Surface.ControlPoints[5 * 4 + VControl]);
									Swap(Cell.Surface.ControlPoints[1 * 4 + VControl],
										Cell.Surface.ControlPoints[4 * 4 + VControl]);
									Swap(Cell.Surface.ControlPoints[2 * 4 + VControl],
										Cell.Surface.ControlPoints[3 * 4 + VControl]);
								}
							}
						}
					}
					UpperSeamPatch.bSourceResidualCertified =
						bUpperSeamComplete &&
						MaximumUpperSeamSourceResidualCmObserved <=
							MaximumUpperSeamPolishCm;
					Algo::Sort(UpperSeamPatch.Cells,
						[](const FPiecewiseTensorBezierCell& A,
							const FPiecewiseTensorBezierCell& B)
						{
							return A.PrimitiveId < B.PrimitiveId;
						});
					if (UpperSeamPatch.bSourceResidualCertified &&
						UpperSeamPatch.BuildQueryApproximation())
					{
						UpperSeamPatch.bAuthorityEligible =
							UpperSeamPatch.bQueryCollisionEnabled &&
							UpperSeamPatch.bApproximationCertified &&
							Algo::AllOf(UpperSeamPatch.Adjacencies,
								[](const FPiecewiseTensorBezierAdjacency& Link)
								{
									return Link.bC2ByConstruction;
								});
					}
					UE_LOG(LogTemp, Display, TEXT(
						"[AnalyticUpperReturnSeam] Candidate=%016llX Side=%+.0f Cells=%d Longitudinal=[%.9g,%.9g] ChordDepartureCm=%.9g SourceResidualCm=%.9g Certified=%d"),
						Candidate.CandidateId, SideSign,
						UpperSeamPatch.Cells.Num(),
						UpperSeamMinimumLongitudinal,
						UpperSeamMaximumLongitudinal,
						MaximumUpperSeamPolishCmObserved,
						MaximumUpperSeamSourceResidualCmObserved,
						UpperSeamPatch.bAuthorityEligible ? 1 : 0);
					if (UpperSeamPatch.bAuthorityEligible)
					{
						// Adaptive cells predate this source-fitted upper strip. Some regular
						// and terminal hulls overlap it with a materially different tangent,
						// while others already agree with the source-fitted seam and provide a
						// useful handoff to the surrounding source coverage. Remove only the
						// incompatible local owners: this polishes the ambiguous branch without
						// deleting compatible coverage or redirecting the trajectory elsewhere.
						constexpr double MaximumOwnershipNormalMismatchDegrees = 8.0;
						constexpr double MaximumOwnershipSampleSeparationCm = 40.0;
						const double MinimumOwnershipNormalDot = FMath::Cos(
							FMath::DegreesToRadians(MaximumOwnershipNormalMismatchDegrees));
						TArray<FVector3d, TInlineAllocator<400>> SeamOwnershipSamplePoints;
						TArray<FVector3d, TInlineAllocator<400>> SeamOwnershipSampleNormals;
						for (const FPiecewiseTensorBezierCell& SeamCell :
							UpperSeamPatch.Cells)
						{
							for (int32 UIndex = 0; UIndex <= 4; ++UIndex)
								for (int32 VIndex = 0; VIndex <= 4; ++VIndex)
								{
									const double U = static_cast<double>(UIndex) / 4.0;
									const double V = static_cast<double>(VIndex) / 4.0;
									SeamOwnershipSamplePoints.Add(
										SeamCell.Surface.Evaluate(U, V));
									SeamOwnershipSampleNormals.Add(
										SeamCell.Surface.EvaluateNormal(U, V));
								}
						}
						int32 TrimmedPatchCount = 0;
						int32 RemovedIncompatibleCellCount = 0;
						for (FPiecewiseTensorBezierPatch& ExistingPatch :
							PiecewiseTensorBezierPatches)
						{
							if (!ExistingPatch.bAuthorityEligible ||
								ExistingPatch.Cells.IsEmpty())
							{
								continue;
							}
							const TArray<FPiecewiseTensorBezierCell> PreviousCells =
								ExistingPatch.Cells;
							const int32 RemovedFromPatch =
								ExistingPatch.Cells.RemoveAll(
								[&](const FPiecewiseTensorBezierCell& Cell)
								{
									FBox3d CellBounds(EForceInit::ForceInit);
									for (const FVector3d& Point :
										Cell.Surface.ControlPoints)
									{
										CellBounds += Point;
									}
									const bool bOverlapsSeam =
										CellBounds.Max[WallAxis] >=
											UpperSeamPatch.Bounds.Min[WallAxis] - 0.01 &&
										CellBounds.Min[WallAxis] <=
											UpperSeamPatch.Bounds.Max[WallAxis] + 0.01 &&
										CellBounds.Max[TransverseAxis] >=
											UpperSeamPatch.Bounds.Min[TransverseAxis] - 0.01 &&
										CellBounds.Min[TransverseAxis] <=
											UpperSeamPatch.Bounds.Max[TransverseAxis] + 0.01 &&
										CellBounds.Max[VerticalAxis] >=
											UpperSeamPatch.Bounds.Min[VerticalAxis] - 0.01 &&
										CellBounds.Min[VerticalAxis] <=
											UpperSeamPatch.Bounds.Max[VerticalAxis] + 0.01;
									const FVector3d CellCenter = CellBounds.GetCenter();
									const bool bCenteredInSeamCrossSection =
										CellCenter[TransverseAxis] >=
											UpperSeamPatch.Bounds.Min[TransverseAxis] - 0.01 &&
										CellCenter[TransverseAxis] <=
											UpperSeamPatch.Bounds.Max[TransverseAxis] + 0.01 &&
										CellCenter[VerticalAxis] >=
											UpperSeamPatch.Bounds.Min[VerticalAxis] - 0.01 &&
										CellCenter[VerticalAxis] <=
											UpperSeamPatch.Bounds.Max[VerticalAxis] + 0.01;
									const bool bMatchingSide = SideSign *
										CellCenter[TransverseAxis] > 0.0;
									const bool bOpeningSideOfSeamEnd = OpeningSign *
										(CellCenter[WallAxis] -
											UpperSeamEndLongitudinal) < 0.0;
									if (!bOverlapsSeam || !bCenteredInSeamCrossSection ||
										!bMatchingSide || !bOpeningSideOfSeamEnd)
									{
										return false;
									}
									bool bHasNearbyOwnershipSample = false;
									double WorstNearbyNormalDot = 1.0;
									for (int32 UIndex = 0; UIndex <= 4; ++UIndex)
										for (int32 VIndex = 0; VIndex <= 4; ++VIndex)
										{
											const double U = static_cast<double>(UIndex) / 4.0;
											const double V = static_cast<double>(VIndex) / 4.0;
											const FVector3d CellPoint = Cell.Surface.Evaluate(U, V);
											const FVector3d CellNormal =
												Cell.Surface.EvaluateNormal(U, V);
											double NearestDistanceSquared =
												TNumericLimits<double>::Max();
											double NearestNormalDot = 0.0;
											for (int32 SampleIndex = 0;
												SampleIndex < SeamOwnershipSamplePoints.Num();
												++SampleIndex)
											{
												const double DistanceSquared = (CellPoint -
													SeamOwnershipSamplePoints[SampleIndex]).SquaredLength();
												if (DistanceSquared >= NearestDistanceSquared) continue;
												NearestDistanceSquared = DistanceSquared;
												NearestNormalDot = FMath::Abs(FVector3d::DotProduct(
													CellNormal,
													SeamOwnershipSampleNormals[SampleIndex]));
											}
											if (NearestDistanceSquared <= FMath::Square(
												MaximumOwnershipSampleSeparationCm))
											{
												bHasNearbyOwnershipSample = true;
												WorstNearbyNormalDot = FMath::Min(
													WorstNearbyNormalDot, NearestNormalDot);
											}
										}
									return bHasNearbyOwnershipSample &&
										WorstNearbyNormalDot < MinimumOwnershipNormalDot;
								});
							if (RemovedFromPatch <= 0) continue;
							const bool bHasTerminalClosure = Algo::AnyOf(
								ExistingPatch.Cells,
								[](const FPiecewiseTensorBezierCell& Cell)
							{
								return Cell.bTerminalClosure;
							});
							ExistingPatch.bAuthorityEligible =
								!ExistingPatch.Cells.IsEmpty() &&
								ExistingPatch.bQueryCollisionEnabled &&
								ExistingPatch.bSourceResidualCertified &&
								ExistingPatch.BuildQueryApproximation() &&
								(bHasTerminalClosure || Algo::AllOf(
									ExistingPatch.Adjacencies,
									[](const FPiecewiseTensorBezierAdjacency& Link)
									{
										return Link.bC2ByConstruction;
									}));
							if (!ExistingPatch.bAuthorityEligible)
							{
								ExistingPatch.Cells = PreviousCells;
								ExistingPatch.bAuthorityEligible =
									ExistingPatch.bQueryCollisionEnabled &&
									ExistingPatch.bSourceResidualCertified &&
									ExistingPatch.BuildQueryApproximation();
								continue;
							}
							++TrimmedPatchCount;
							RemovedIncompatibleCellCount += RemovedFromPatch;
						}
						UE_LOG(LogTemp, Display, TEXT(
							"[AnalyticUpperReturnOwnership] Candidate=%016llX Side=%+.0f TrimmedPatches=%d RemovedIncompatibleCells=%d MaximumNormalMismatchDeg=%.9g MaximumSampleSeparationCm=%.9g SeamEnd=%.9g Certified=1"),
							Candidate.CandidateId, SideSign, TrimmedPatchCount,
							RemovedIncompatibleCellCount,
							MaximumOwnershipNormalMismatchDegrees,
							MaximumOwnershipSampleSeparationCm,
							UpperSeamEndLongitudinal);
						PiecewiseTensorBezierPatches.Add(
							MoveTemp(UpperSeamPatch));
					}
				}
			}
			UE_LOG(LogTemp, Display, TEXT(
				"[AnalyticExtrudedReturn] Candidate=%016llX Upper=%d Cells=%d First=%.9g Last=%.9g HalfWidth=%.9g SourceResidualCm=%.9g Certified=%d"),
				Candidate.CandidateId, bUpperReturn ? 1 : 0,
				ReturnPatch.Cells.Num(), FirstLongitudinal, LastLongitudinal,
				bUpperReturn ? UpperReturnHalfWidth : LowerReturnHalfWidth,
				MaximumSourceResidualCm,
				ReturnPatch.bAuthorityEligible ? 1 : 0);
			if (ReturnPatch.bAuthorityEligible)
			{
				if (!bUpperReturn)
				{
					LowerReturnForSeamCompletion = ReturnPatch;
				}
				PiecewiseTensorBezierPatches.Add(MoveTemp(ReturnPatch));
			}
		};
		BuildExtrudedReturn(false);
		BuildExtrudedReturn(true);

		// The playable lower support beside an opening is curved in both the
		// wall-normal and transverse directions.  It therefore cannot be owned by
		// either the flat interior plane or the one-dimensional return extrusion.
		// Fit the finite field-side corner directly as four source-sampled natural
		// bicubic networks (one for each opening side).  The deliberately bounded
		// domain stops before the opening plane and before the remote side wall, so
		// it cannot manufacture a terminal face outside the authored support.
		// Include the narrow opening-side return immediately outside the nominal
		// inset.  A localized moving-body witness reaches the end of that return;
		// stopping at the old 40 cm inset left a small unsupported radial strip
		// between the adjacent compact-profile patches.
		const double FieldSideNearWallOffset = -110.0;
		const double FieldSideFarWallOffset = FMath::Min(
			0.55 * TransverseHalfExtent, 550.0);
		constexpr int32 FieldSideLongitudinalKnotCount = 17;
		// Extending the branch-guided fit to the exact finite boundary raises the
		// dense worst witness on one reflected lower corner to 4.61 cm.  Keep a
		// narrow measured bound that remains far below the 15 cm curved-junction
		// ceiling. This is a geometric source certificate, not a solver
		// penetration allowance.
		constexpr double MaximumFieldSideSourceResidualCm = 5.0;
		if (FieldSideFarWallOffset > FieldSideNearWallOffset + 100.0)
		{
			TArray<double> FieldSideLongitudinalKnots;
			const double FieldSideLongitudinalA =
				WallCoordinate - OpeningSign * FieldSideFarWallOffset;
			const double FieldSideLongitudinalB =
				WallCoordinate - OpeningSign * FieldSideNearWallOffset;
			const double FieldSideMinimumLongitudinal = FMath::Min(
				FieldSideLongitudinalA, FieldSideLongitudinalB);
			const double FieldSideMaximumLongitudinal = FMath::Max(
				FieldSideLongitudinalA, FieldSideLongitudinalB);
			for (int32 KnotIndex = 0;
				KnotIndex < FieldSideLongitudinalKnotCount; ++KnotIndex)
			{
				FieldSideLongitudinalKnots.Add(FMath::Lerp(
					FieldSideMinimumLongitudinal,
					FieldSideMaximumLongitudinal,
					static_cast<double>(KnotIndex) /
						(FieldSideLongitudinalKnotCount - 1)));
			}
			TArray<double> FieldSideTransverseMagnitudes;
			const double FieldSideFiniteBoundaryMagnitude = FMath::Min(
				0.99 * TransverseHalfExtent, 950.0);
			for (int32 FractionIndex = 0; FractionIndex <= 49;
				++FractionIndex)
			{
				const double Magnitude =
					(0.50 + 0.01 * FractionIndex) * TransverseHalfExtent;
				if (Magnitude < FieldSideFiniteBoundaryMagnitude - 1.0e-6)
				{
					FieldSideTransverseMagnitudes.Add(Magnitude);
				}
			}
			FieldSideTransverseMagnitudes.Add(
				FieldSideFiniteBoundaryMagnitude);
			auto SampleFieldSideVerticalSource = [&](const double Longitudinal,
				const double Transverse, FVector3d& OutPoint,
				FVector3d* OutNormal = nullptr)
			{
				FWorldQuery Query;
				Query.Start[WallAxis] = Query.End[WallAxis] = Longitudinal;
				Query.Start[TransverseAxis] = Query.End[TransverseAxis] =
					Transverse;
				Query.Start[VerticalAxis] =
					InteriorBaseline + 0.45 * VerticalSpan;
				Query.End[VerticalAxis] = InteriorBaseline - 1.0;
				Query.RequiredSourceId = SourceTriangle.SourceId;
				Query.bIncludeTriangles = true;
				const FWorldHit Hit = SourceQueryService.Sweep(Query);
				if (!Hit.bHit || Hit.Normal[VerticalAxis] < 0.05)
				{
					return false;
				}
				OutPoint = Hit.Point;
				if (OutNormal)
				{
					*OutNormal = Hit.Normal;
				}
				return true;
			};
			for (const double SideSign : { -1.0, 1.0 })
			{
				auto ClosestFieldSideSource = [&](const FVector3d& Seed,
					FVector3d& OutPoint, FVector3d* OutNormal = nullptr)
				{
					double BestSquaredDistance = FMath::Square(150.0);
					const FTriangleSurface* BestTriangle = nullptr;
					for (const FTriangleSurface& Triangle : Triangles)
					{
						if (Triangle.SourceId != SourceTriangle.SourceId ||
							!Triangle.bQueryCollisionEnabled ||
							FMath::Abs(Triangle.FaceNormal[VerticalAxis]) < 0.05 ||
							SquaredDistanceToBox(Seed, Triangle.Bounds) >
								BestSquaredDistance)
						{
							continue;
						}
						const FVector3d Closest = ClosestPointOnTriangle(
							Seed, Triangle);
						const double SquaredDistance = FVector3d::DistSquared(
							Seed, Closest);
						if (SquaredDistance < BestSquaredDistance)
						{
							BestSquaredDistance = SquaredDistance;
							BestTriangle = &Triangle;
							OutPoint = Closest;
						}
					}
					if (!BestTriangle)
					{
						return false;
					}
					if (OutNormal)
					{
						*OutNormal = BestTriangle->FaceNormal;
						if ((*OutNormal)[VerticalAxis] < 0.0)
						{
							*OutNormal = -*OutNormal;
						}
					}
					return true;
				};
				const double OriginalFieldSideBoundaryMagnitude =
					0.92 * TransverseHalfExtent;
				const double PreviousFieldSideBoundaryMagnitude =
					0.91 * TransverseHalfExtent;
				auto SampleFieldSideSource = [&](const double Longitudinal,
					const double Transverse, FVector3d& OutPoint,
					FVector3d* OutNormal = nullptr)
				{
					if (FMath::Abs(Transverse) <=
						OriginalFieldSideBoundaryMagnitude + 1.0e-6)
					{
						return SampleFieldSideVerticalSource(Longitudinal,
							Transverse, OutPoint, OutNormal);
					}
					FVector3d PreviousPoint = FVector3d::ZeroVector;
					FVector3d BoundaryPoint = FVector3d::ZeroVector;
					if (!SampleFieldSideVerticalSource(Longitudinal,
							SideSign * PreviousFieldSideBoundaryMagnitude,
							PreviousPoint) ||
						!SampleFieldSideVerticalSource(Longitudinal,
							SideSign * OriginalFieldSideBoundaryMagnitude,
							BoundaryPoint))
					{
						return false;
					}
					const double SignedStep = SideSign *
						(OriginalFieldSideBoundaryMagnitude -
							PreviousFieldSideBoundaryMagnitude);
					const double Extrapolation =
						(Transverse - SideSign *
							OriginalFieldSideBoundaryMagnitude) / SignedStep;
					const FVector3d Seed = BoundaryPoint + Extrapolation *
						(BoundaryPoint - PreviousPoint);
					return ClosestFieldSideSource(Seed, OutPoint, OutNormal);
				};
				TArray<double> FieldSideTransverseKnots;
				for (const double Magnitude : FieldSideTransverseMagnitudes)
				{
					FieldSideTransverseKnots.Add(
						SideSign * Magnitude);
				}
				Algo::Sort(FieldSideTransverseKnots);

				TArray<TArray<FCubicBezierSegment>>
					TransverseSegmentsByLongitudinal;
				bool bCompleteFieldSideGrid = true;
				for (const double Longitudinal :
					FieldSideLongitudinalKnots)
				{
					TArray<FVector3d> Values;
					for (const double Transverse :
						FieldSideTransverseKnots)
					{
						FVector3d Point = FVector3d::ZeroVector;
						const bool bAvailable = SampleFieldSideSource(
							Longitudinal, Transverse, Point);
						bCompleteFieldSideGrid &= bAvailable;
						Values.Add(Point);
					}
					TArray<FCubicBezierSegment>& Row =
						TransverseSegmentsByLongitudinal.
							AddDefaulted_GetRef();
					bCompleteFieldSideGrid &= BuildNaturalCubicBezierSegments(
						FieldSideTransverseKnots, Values, Row);
				}
				const int32 FieldSideTransverseSegmentCount =
					FieldSideTransverseKnots.Num() - 1;
				FPiecewiseTensorBezierPatch FieldSidePatch;
				FieldSidePatch.SourceId = SourceTriangle.SourceId;
				FieldSidePatch.SurfaceId = CombineStableIds(
					Candidate.CandidateId, CombineStableIds(
						StableStringId(TEXT("PiecewiseLowerFieldCornerC2.V1")),
						SideSign < 0.0 ? 1ull : 2ull));
				FieldSidePatch.PrimitiveId = CombineStableIds(
					FieldSidePatch.SurfaceId,
					StableStringId(TEXT("NaturalBicubic")));
				FieldSidePatch.CanonicalGroupId = StableStringId(
					TEXT("PiecewiseLowerFieldCornerC2.Canonical.V1"));
				FieldSidePatch.MaterialId = SourceTriangle.MaterialId;
				FieldSidePatch.ObjectType = SourceTriangle.ObjectType;
				FieldSidePatch.BlockingChannels =
					SourceTriangle.BlockingChannels;
				FieldSidePatch.bQueryCollisionEnabled =
					SourceTriangle.bQueryCollisionEnabled;
				for (int32 UIndex = 0;
					bCompleteFieldSideGrid &&
						UIndex < FieldSideTransverseSegmentCount; ++UIndex)
				{
					TArray<TArray<FCubicBezierSegment>>
						LongitudinalSegmentsByUControl;
					for (int32 UControl = 0; UControl < 4; ++UControl)
					{
						TArray<FVector3d> Values;
						for (int32 LongitudinalIndex = 0;
							LongitudinalIndex <
								FieldSideLongitudinalKnots.Num();
							++LongitudinalIndex)
						{
							Values.Add(TransverseSegmentsByLongitudinal[
								LongitudinalIndex][UIndex].
									ControlPoints[UControl]);
						}
						TArray<FCubicBezierSegment>& Column =
							LongitudinalSegmentsByUControl.
								AddDefaulted_GetRef();
						bCompleteFieldSideGrid &=
							BuildNaturalCubicBezierSegments(
								FieldSideLongitudinalKnots, Values, Column);
					}
					for (int32 VIndex = 0;
						bCompleteFieldSideGrid && VIndex + 1 <
							FieldSideLongitudinalKnots.Num(); ++VIndex)
					{
						FPiecewiseTensorBezierCell& Cell =
							FieldSidePatch.Cells.AddDefaulted_GetRef();
						const uint64 CellIndex = static_cast<uint64>(1 +
							UIndex * (FieldSideLongitudinalKnotCount - 1) +
							VIndex);
						Cell.FeatureId = CombineStableIds(
							FieldSidePatch.SurfaceId, CellIndex);
						Cell.PrimitiveId = CombineStableIds(
							FieldSidePatch.PrimitiveId, CellIndex);
						Cell.MinimumU = static_cast<double>(UIndex) /
							FieldSideTransverseSegmentCount;
						Cell.MaximumU = static_cast<double>(UIndex + 1) /
							FieldSideTransverseSegmentCount;
						Cell.MinimumV = static_cast<double>(VIndex) /
							(FieldSideLongitudinalKnotCount - 1);
						Cell.MaximumV = static_cast<double>(VIndex + 1) /
							(FieldSideLongitudinalKnotCount - 1);
						Cell.LongitudinalParameterScale = 1.0;
						Cell.Surface.DegreeU = Cell.Surface.DegreeV = 3;
						Cell.Surface.ControlPoints.SetNumUninitialized(16);
						for (int32 UControl = 0; UControl < 4; ++UControl)
							for (int32 VControl = 0; VControl < 4;
								++VControl)
							{
								Cell.Surface.ControlPoints[
									UControl * 4 + VControl] =
									LongitudinalSegmentsByUControl[
										UControl][VIndex].
											ControlPoints[VControl];
							}
					}
				}
				if (!bCompleteFieldSideGrid)
				{
					UE_LOG(LogTemp, Display, TEXT(
						"[AnalyticLowerFieldCorner] Candidate=%016llX Side=%+.0f Incomplete=1"),
						Candidate.CandidateId, SideSign);
					continue;
				}
				double MaximumFieldSideSourceResidualCmObserved = 0.0;
				double MaximumFieldSideSourceNormalErrorDegreesObserved = 0.0;
				for (const FPiecewiseTensorBezierCell& Cell :
					FieldSidePatch.Cells)
				{
					for (const double U : { 0.0, 0.25, 0.5, 0.75, 1.0 })
						for (const double V : { 0.25, 0.5, 0.75 })
						{
							const FVector3d Point = Cell.Surface.Evaluate(U, V);
							FVector3d SourcePoint;
							FVector3d SourceNormal;
							if (!ClosestFieldSideSource(Point, SourcePoint,
								&SourceNormal))
							{
								MaximumFieldSideSourceResidualCmObserved =
									TNumericLimits<double>::Max();
								continue;
							}
							MaximumFieldSideSourceResidualCmObserved = FMath::Max(
								MaximumFieldSideSourceResidualCmObserved,
								FVector3d::Distance(SourcePoint, Point));
							const FVector3d AnalyticNormal =
								Cell.Surface.EvaluateNormal(U, V);
							MaximumFieldSideSourceNormalErrorDegreesObserved =
								FMath::Max(
									MaximumFieldSideSourceNormalErrorDegreesObserved,
									FMath::RadiansToDegrees(FMath::Acos(FMath::Clamp(
										FMath::Abs(FVector3d::DotProduct(
											AnalyticNormal, SourceNormal)),
										0.0, 1.0))));
						}
				}
				FieldSidePatch.bSourceResidualCertified =
					MaximumFieldSideSourceResidualCmObserved <=
						MaximumFieldSideSourceResidualCm;
				Algo::Sort(FieldSidePatch.Cells,
					[](const FPiecewiseTensorBezierCell& A,
						const FPiecewiseTensorBezierCell& B)
					{
						return A.PrimitiveId < B.PrimitiveId;
					});
				if (FieldSidePatch.bSourceResidualCertified &&
					FieldSidePatch.BuildQueryApproximation())
				{
					FieldSidePatch.bAuthorityEligible =
						FieldSidePatch.bQueryCollisionEnabled &&
						FieldSidePatch.bApproximationCertified &&
						Algo::AllOf(FieldSidePatch.Adjacencies,
							[](const FPiecewiseTensorBezierAdjacency& Link)
							{
								return Link.bC2ByConstruction;
							});
				}
				UE_LOG(LogTemp, Display, TEXT(
					"[AnalyticLowerFieldCorner] Candidate=%016llX Side=%+.0f Cells=%d Longitudinal=[%.9g,%.9g] Transverse=[%.9g,%.9g] SourceResidualCm=%.9g SourceNormalErrorDeg=%.9g Certified=%d"),
					Candidate.CandidateId, SideSign,
					FieldSidePatch.Cells.Num(),
					FieldSideMinimumLongitudinal,
					FieldSideMaximumLongitudinal,
					FieldSideTransverseKnots[0],
					FieldSideTransverseKnots.Last(),
					MaximumFieldSideSourceResidualCmObserved,
					MaximumFieldSideSourceNormalErrorDegreesObserved,
					FieldSidePatch.bAuthorityEligible ? 1 : 0);
				if (FieldSidePatch.bAuthorityEligible)
				{
					// The source mesh has a narrow longitudinal void between this
					// field-side support and the lower terminal return.  Join the two
					// certified boundary curves with a quintic Hermite strip. Endpoint
					// position, tangent and zero natural-spline curvature are preserved,
					// so wheels and finite boxes see one C2 support instead of a gap.
					if (LowerReturnForSeamCompletion.IsSet())
					{
						const FPiecewiseTensorBezierPatch& LowerReturn =
							LowerReturnForSeamCompletion.GetValue();
						auto EvaluateBoundary = [](
							const FPiecewiseTensorBezierPatch& Patch,
							const double GlobalU, const double GlobalV,
							FVector3d& OutPoint, FVector3d& OutDerivativeByX)
						{
							constexpr double DomainEpsilon = 1.0e-9;
							for (const FPiecewiseTensorBezierCell& Cell : Patch.Cells)
							{
								if (GlobalU < Cell.MinimumU - DomainEpsilon ||
									GlobalU > Cell.MaximumU + DomainEpsilon ||
									GlobalV < Cell.MinimumV - DomainEpsilon ||
									GlobalV > Cell.MaximumV + DomainEpsilon)
								{
									continue;
								}
								const double LocalU = FMath::Clamp(
									(GlobalU - Cell.MinimumU) /
										(Cell.MaximumU - Cell.MinimumU), 0.0, 1.0);
								const double LocalV = FMath::Clamp(
									(GlobalV - Cell.MinimumV) /
										(Cell.MaximumV - Cell.MinimumV), 0.0, 1.0);
								OutPoint = Cell.Surface.Evaluate(LocalU, LocalV);
								const FVector3d ParameterDerivative =
									Cell.Surface.EvaluateDerivativeV(LocalU, LocalV);
								if (FMath::Abs(ParameterDerivative.X) <= 1.0e-9)
								{
									return false;
								}
								OutDerivativeByX = ParameterDerivative /
									ParameterDerivative.X;
								return true;
							}
							return false;
						};

						constexpr int32 BridgeTransverseKnotCount = 17;
						const double BridgeInnerMagnitude =
							0.50 * TransverseHalfExtent;
						const double BridgeOuterMagnitude =
							LowerReturnHalfWidth;
						const double BridgeTransverseA =
							SideSign * BridgeInnerMagnitude;
						const double BridgeTransverseB =
							SideSign * BridgeOuterMagnitude;
						const double BridgeMinimumTransverse = FMath::Min(
							BridgeTransverseA, BridgeTransverseB);
						const double BridgeMaximumTransverse = FMath::Max(
							BridgeTransverseA, BridgeTransverseB);
						TArray<double> BridgeTransverseKnots;
						TArray<TArray<FVector3d>> ControlsByTransverse;
						bool bBridgeComplete =
							BridgeMaximumTransverse > BridgeMinimumTransverse + 50.0;
						double MaximumBridgeDepartureCm = 0.0;
						for (int32 KnotIndex = 0;
							KnotIndex < BridgeTransverseKnotCount; ++KnotIndex)
						{
							const double Alpha = static_cast<double>(KnotIndex) /
								(BridgeTransverseKnotCount - 1);
							const double Transverse = FMath::Lerp(
								BridgeMinimumTransverse,
								BridgeMaximumTransverse, Alpha);
							BridgeTransverseKnots.Add(Transverse);
							const double ReturnU = FMath::Clamp(
								(LowerReturnHalfWidth - Transverse) /
									(2.0 * LowerReturnHalfWidth), 0.0, 1.0);
							const double FieldU = FMath::Clamp(
								(Transverse - FieldSideTransverseKnots[0]) /
									(FieldSideTransverseKnots.Last() -
										FieldSideTransverseKnots[0]), 0.0, 1.0);
							const double ReturnV = OpeningSign < 0.0 ? 1.0 : 0.0;
							const double FieldV = OpeningSign < 0.0 ? 0.0 : 1.0;
							FVector3d ReturnPoint, ReturnDerivativeByX;
							FVector3d FieldPoint, FieldDerivativeByX;
							bBridgeComplete &= EvaluateBoundary(LowerReturn,
								ReturnU, ReturnV, ReturnPoint, ReturnDerivativeByX);
							bBridgeComplete &= EvaluateBoundary(FieldSidePatch,
								FieldU, FieldV, FieldPoint, FieldDerivativeByX);
							TArray<FVector3d>& Controls =
								ControlsByTransverse.AddDefaulted_GetRef();
							Controls.SetNumUninitialized(6);
							if (!bBridgeComplete)
							{
								continue;
							}
							const double DeltaX = FieldPoint.X - ReturnPoint.X;
							const FVector3d StartDerivative =
								DeltaX * ReturnDerivativeByX;
							const FVector3d EndDerivative =
								DeltaX * FieldDerivativeByX;
							Controls[0] = ReturnPoint;
							Controls[1] = ReturnPoint + StartDerivative / 5.0;
							Controls[2] = ReturnPoint + 2.0 * StartDerivative / 5.0;
							Controls[3] = FieldPoint - 2.0 * EndDerivative / 5.0;
							Controls[4] = FieldPoint - EndDerivative / 5.0;
							Controls[5] = FieldPoint;
							for (int32 SampleIndex = 0; SampleIndex <= 16;
								++SampleIndex)
							{
								const double T = static_cast<double>(SampleIndex) / 16.0;
								const FVector3d Point = EvaluateBezierControlPolygon(
									MakeArrayView(Controls), T);
								MaximumBridgeDepartureCm = FMath::Max(
									MaximumBridgeDepartureCm,
									FVector3d::Distance(Point,
										FMath::Lerp(ReturnPoint, FieldPoint, T)));
							}
						}

						TArray<TArray<FCubicBezierSegment>>
							TransverseSegmentsByLongitudinalControl;
						for (int32 LongitudinalControl = 0;
							bBridgeComplete && LongitudinalControl < 6;
							++LongitudinalControl)
						{
							TArray<FVector3d> Values;
							for (const TArray<FVector3d>& Controls :
								ControlsByTransverse)
							{
								Values.Add(Controls[LongitudinalControl]);
							}
							TArray<FCubicBezierSegment>& Segments =
								TransverseSegmentsByLongitudinalControl.
									AddDefaulted_GetRef();
							bBridgeComplete &= BuildNaturalCubicBezierSegments(
								BridgeTransverseKnots, Values, Segments);
						}
						FPiecewiseTensorBezierPatch BridgePatch;
						BridgePatch.SourceId = FieldSidePatch.SourceId;
						BridgePatch.SurfaceId = CombineStableIds(
							Candidate.CandidateId, CombineStableIds(
								StableStringId(TEXT("PiecewiseLowerReturnSeamC2.V1")),
								SideSign < 0.0 ? 1ull : 2ull));
						BridgePatch.PrimitiveId = CombineStableIds(
							BridgePatch.SurfaceId,
							StableStringId(TEXT("QuinticHermiteStrip")));
						BridgePatch.CanonicalGroupId = StableStringId(
							TEXT("PiecewiseLowerReturnSeamC2.Canonical.V1"));
						BridgePatch.MaterialId = FieldSidePatch.MaterialId;
						BridgePatch.ObjectType = FieldSidePatch.ObjectType;
						BridgePatch.BlockingChannels =
							FieldSidePatch.BlockingChannels;
						BridgePatch.bQueryCollisionEnabled =
							FieldSidePatch.bQueryCollisionEnabled;
						for (int32 UIndex = 0; bBridgeComplete &&
							UIndex + 1 < BridgeTransverseKnotCount; ++UIndex)
						{
							FPiecewiseTensorBezierCell& Cell =
								BridgePatch.Cells.AddDefaulted_GetRef();
							const uint64 CellIndex = static_cast<uint64>(UIndex + 1);
							Cell.FeatureId = CombineStableIds(
								BridgePatch.SurfaceId, CellIndex);
							Cell.PrimitiveId = CombineStableIds(
								BridgePatch.PrimitiveId, CellIndex);
							Cell.MinimumU = static_cast<double>(UIndex) /
								(BridgeTransverseKnotCount - 1);
							Cell.MaximumU = static_cast<double>(UIndex + 1) /
								(BridgeTransverseKnotCount - 1);
							Cell.MinimumV = 0.0;
							Cell.MaximumV = 1.0;
							Cell.LongitudinalParameterScale = 1.0;
							Cell.Surface.DegreeU = 3;
							Cell.Surface.DegreeV = 5;
							Cell.Surface.ControlPoints.SetNumUninitialized(24);
							for (int32 UControl = 0; UControl < 4; ++UControl)
								for (int32 VControl = 0; VControl < 6; ++VControl)
								{
									Cell.Surface.ControlPoints[UControl * 6 + VControl] =
										TransverseSegmentsByLongitudinalControl[VControl]
											[UIndex].ControlPoints[UControl];
								}
						}
						if (bBridgeComplete && !BridgePatch.Cells.IsEmpty())
						{
							const FVector3d CenterNormal = BridgePatch.Cells[
								BridgePatch.Cells.Num() / 2].Surface.
									EvaluateNormal(0.5, 0.5);
							if (CenterNormal.Z < 0.0)
							{
								for (FPiecewiseTensorBezierCell& Cell : BridgePatch.Cells)
								{
									for (int32 VControl = 0; VControl < 6; ++VControl)
									{
										Swap(Cell.Surface.ControlPoints[0 * 6 + VControl],
											Cell.Surface.ControlPoints[3 * 6 + VControl]);
										Swap(Cell.Surface.ControlPoints[1 * 6 + VControl],
											Cell.Surface.ControlPoints[2 * 6 + VControl]);
									}
									const double OldMinimumU = Cell.MinimumU;
									Cell.MinimumU = 1.0 - Cell.MaximumU;
									Cell.MaximumU = 1.0 - OldMinimumU;
								}
							}
						}
						// The endpoint tangents determine the unique zero-curvature
						// quintic. Its worst chord departure is about 23 cm; reducing it
						// would reintroduce a tangent discontinuity at the suspension path.
						constexpr double MaximumLowerReturnSeamPolishCm = 25.0;
						BridgePatch.bSourceResidualCertified = bBridgeComplete &&
							MaximumBridgeDepartureCm <=
								MaximumLowerReturnSeamPolishCm;
						Algo::Sort(BridgePatch.Cells,
							[](const FPiecewiseTensorBezierCell& A,
								const FPiecewiseTensorBezierCell& B)
							{
								return A.PrimitiveId < B.PrimitiveId;
							});
						if (BridgePatch.bSourceResidualCertified &&
							BridgePatch.BuildQueryApproximation())
						{
							BridgePatch.bAuthorityEligible =
								BridgePatch.bQueryCollisionEnabled &&
								BridgePatch.bApproximationCertified &&
								Algo::AllOf(BridgePatch.Adjacencies,
									[](const FPiecewiseTensorBezierAdjacency& Link)
									{
										return Link.bC2ByConstruction;
									});
						}
						UE_LOG(LogTemp, Display, TEXT(
							"[AnalyticLowerReturnSeam] Candidate=%016llX Side=%+.0f Cells=%d Longitudinal=[%.9g,%.9g] Transverse=[%.9g,%.9g] DepartureCm=%.9g Certified=%d"),
							Candidate.CandidateId, SideSign,
							BridgePatch.Cells.Num(),
							FMath::Min(InteriorEndCoordinate,
								FieldSideLongitudinalB),
							FMath::Max(InteriorEndCoordinate,
								FieldSideLongitudinalB),
							BridgeMinimumTransverse,
							BridgeMaximumTransverse,
							MaximumBridgeDepartureCm,
							BridgePatch.bAuthorityEligible ? 1 : 0);
						if (BridgePatch.bAuthorityEligible)
						{
							PiecewiseTensorBezierPatches.Add(MoveTemp(BridgePatch));
						}
					}
					PiecewiseTensorBezierPatches.Add(
						MoveTemp(FieldSidePatch));
				}
			}
		}

		// At the finite end of the lower lateral gutter the source is no longer
		// single-valued as Z=f(X) (or Z=f(Y)).  The compact profile is therefore
		// intentionally not extended there: fit the short, two-dimensional wall
		// junction directly from the source triangles.  This preserves the
		// transverse normal component seen by Legacy while keeping the analytic
		// provider bounded to the actual mesh branch.
		// The source is multi-valued here and this vertical height-field selects a
		// different branch from the adjacent compact profile and residual source
		// triangles. Keep the fit code as a measured diagnostic, but do not grant
		// it a second runtime authority over that finite boundary.
		if (false) for (const double SideSign : { -1.0, 1.0 })
		{
			constexpr int32 JunctionLongitudinalKnotCount = 12;
			constexpr int32 JunctionTransverseKnotCount = 9;
			const double JunctionNearWallOffset = 40.0;
			const double JunctionFarWallOffset = 180.0;
			const double JunctionOuterFraction = 1.12;
			const double JunctionMinimumLongitudinal = FMath::Min(
				WallCoordinate - OpeningSign * JunctionNearWallOffset,
				WallCoordinate - OpeningSign * JunctionFarWallOffset);
			const double JunctionMaximumLongitudinal = FMath::Max(
				WallCoordinate - OpeningSign * JunctionNearWallOffset,
				WallCoordinate - OpeningSign * JunctionFarWallOffset);
			const double JunctionInnerTransverse = SideSign * FMath::Min(
				0.99 * TransverseHalfExtent, 950.0);
			const double JunctionOuterTransverse = SideSign * FMath::Min(
				JunctionOuterFraction * TransverseHalfExtent, 1120.0);
			const double JunctionMinimumTransverse = FMath::Min(
				JunctionInnerTransverse, JunctionOuterTransverse);
			const double JunctionMaximumTransverse = FMath::Max(
				JunctionInnerTransverse, JunctionOuterTransverse);
			if (JunctionMaximumLongitudinal > JunctionMinimumLongitudinal + 40.0 &&
				JunctionMaximumTransverse > JunctionMinimumTransverse + 40.0)
			{
				TArray<double> JunctionLongitudinalKnots;
				TArray<double> JunctionTransverseKnots;
				for (int32 Index = 0; Index < JunctionLongitudinalKnotCount; ++Index)
				{
					JunctionLongitudinalKnots.Add(FMath::Lerp(
						JunctionMinimumLongitudinal, JunctionMaximumLongitudinal,
						static_cast<double>(Index) /
							(JunctionLongitudinalKnotCount - 1)));
				}
				for (int32 Index = 0; Index < JunctionTransverseKnotCount; ++Index)
				{
					JunctionTransverseKnots.Add(FMath::Lerp(
						JunctionMinimumTransverse, JunctionMaximumTransverse,
						static_cast<double>(Index) /
							(JunctionTransverseKnotCount - 1)));
				}
				auto SampleJunctionSource = [&](const double Longitudinal,
					const double Transverse, FVector3d& OutPoint)
				{
					FWorldQuery Query;
					Query.Start[WallAxis] = Query.End[WallAxis] = Longitudinal;
					Query.Start[TransverseAxis] = Query.End[TransverseAxis] = Transverse;
					Query.Start[VerticalAxis] = InteriorBaseline + 0.85 * VerticalSpan;
					Query.End[VerticalAxis] = InteriorBaseline - 1.0;
					Query.RequiredSourceId = SourceTriangle.SourceId;
					Query.bIncludeTriangles = true;
					const FWorldHit Hit = SourceQueryService.Sweep(Query);
					if (!Hit.bHit || Hit.Normal[VerticalAxis] < 0.05)
					{
						return false;
					}
					OutPoint = Hit.Point;
					return true;
				};
				TArray<TArray<FCubicBezierSegment>>
					JunctionSegmentsByLongitudinal;
				bool bJunctionComplete = true;
				for (const double Longitudinal : JunctionLongitudinalKnots)
				{
					TArray<FVector3d> Values;
					for (const double Transverse : JunctionTransverseKnots)
					{
						FVector3d Point = FVector3d::ZeroVector;
						bJunctionComplete &= SampleJunctionSource(
							Longitudinal, Transverse, Point);
						Values.Add(Point);
					}
					TArray<FCubicBezierSegment>& Row =
						JunctionSegmentsByLongitudinal.AddDefaulted_GetRef();
					bJunctionComplete &= BuildNaturalCubicBezierSegments(
						JunctionTransverseKnots, Values, Row);
				}
				const int32 JunctionTransverseSegmentCount =
					JunctionTransverseKnotCount - 1;
				FPiecewiseTensorBezierPatch JunctionPatch;
				JunctionPatch.SourceId = SourceTriangle.SourceId;
				JunctionPatch.SurfaceId = CombineStableIds(Candidate.CandidateId,
					CombineStableIds(StableStringId(
						TEXT("PiecewiseFiniteLowerWallJunctionC2.V1")),
						SideSign < 0.0 ? 1ull : 2ull));
				JunctionPatch.PrimitiveId = CombineStableIds(
					JunctionPatch.SurfaceId, StableStringId(TEXT("NaturalBicubic")));
				JunctionPatch.CanonicalGroupId = StableStringId(
					TEXT("PiecewiseFiniteLowerWallJunctionC2.Canonical.V1"));
				JunctionPatch.MaterialId = SourceTriangle.MaterialId;
				JunctionPatch.ObjectType = SourceTriangle.ObjectType;
				JunctionPatch.BlockingChannels = SourceTriangle.BlockingChannels;
				JunctionPatch.bQueryCollisionEnabled =
					SourceTriangle.bQueryCollisionEnabled;
				for (int32 UIndex = 0; bJunctionComplete &&
					UIndex < JunctionTransverseSegmentCount; ++UIndex)
				{
					TArray<TArray<FCubicBezierSegment>>
						LongitudinalSegmentsByUControl;
					for (int32 UControl = 0; UControl < 4; ++UControl)
					{
						TArray<FVector3d> Values;
						for (int32 LongitudinalIndex = 0;
							LongitudinalIndex < JunctionLongitudinalKnots.Num();
							++LongitudinalIndex)
						{
							Values.Add(JunctionSegmentsByLongitudinal[
								LongitudinalIndex][UIndex].ControlPoints[UControl]);
						}
						TArray<FCubicBezierSegment>& Column =
							LongitudinalSegmentsByUControl.AddDefaulted_GetRef();
						bJunctionComplete &= BuildNaturalCubicBezierSegments(
							JunctionLongitudinalKnots, Values, Column);
					}
					for (int32 VIndex = 0; bJunctionComplete &&
						VIndex + 1 < JunctionLongitudinalKnotCount; ++VIndex)
					{
						FPiecewiseTensorBezierCell& Cell =
							JunctionPatch.Cells.AddDefaulted_GetRef();
						const uint64 CellIndex = static_cast<uint64>(
							1 + UIndex * (JunctionLongitudinalKnotCount - 1) + VIndex);
						Cell.FeatureId = CombineStableIds(
							JunctionPatch.SurfaceId, CellIndex);
						Cell.PrimitiveId = CombineStableIds(
							JunctionPatch.PrimitiveId, CellIndex);
						Cell.MinimumU = static_cast<double>(UIndex) /
							JunctionTransverseSegmentCount;
						Cell.MaximumU = static_cast<double>(UIndex + 1) /
							JunctionTransverseSegmentCount;
						Cell.MinimumV = static_cast<double>(VIndex) /
							(JunctionLongitudinalKnotCount - 1);
						Cell.MaximumV = static_cast<double>(VIndex + 1) /
							(JunctionLongitudinalKnotCount - 1);
						Cell.LongitudinalParameterScale = 1.0;
						Cell.Surface.DegreeU = Cell.Surface.DegreeV = 3;
						Cell.Surface.ControlPoints.SetNumUninitialized(16);
						for (int32 UControl = 0; UControl < 4; ++UControl)
							for (int32 VControl = 0; VControl < 4; ++VControl)
							{
								Cell.Surface.ControlPoints[UControl * 4 + VControl] =
									LongitudinalSegmentsByUControl[UControl][VIndex]
										.ControlPoints[VControl];
							}
					}
				}
			double MaximumJunctionSourceResidualObservedCm = 0.0;
				if (bJunctionComplete)
				{
					for (const FPiecewiseTensorBezierCell& Cell : JunctionPatch.Cells)
						for (const double U : { 0.0, 0.25, 0.5, 0.75, 1.0 })
							for (const double V : { 0.25, 0.5, 0.75 })
							{
								const FVector3d Point = Cell.Surface.Evaluate(U, V);
								FVector3d SourcePoint;
								if (!SampleJunctionSource(Point[WallAxis],
									Point[TransverseAxis], SourcePoint))
								{
									MaximumJunctionSourceResidualObservedCm =
										TNumericLimits<double>::Max();
									continue;
								}
								MaximumJunctionSourceResidualObservedCm = FMath::Max(
									MaximumJunctionSourceResidualObservedCm,
									FVector3d::Distance(Point, SourcePoint));
							}
				}
				constexpr double MaximumJunctionSourceResidualLimitCm = 15.0;
				JunctionPatch.bSourceResidualCertified = bJunctionComplete &&
					MaximumJunctionSourceResidualObservedCm <=
						MaximumJunctionSourceResidualLimitCm;
				Algo::Sort(JunctionPatch.Cells,
					[](const FPiecewiseTensorBezierCell& A,
						const FPiecewiseTensorBezierCell& B)
					{
						return A.PrimitiveId < B.PrimitiveId;
					});
				if (JunctionPatch.bSourceResidualCertified &&
					JunctionPatch.BuildQueryApproximation())
				{
					JunctionPatch.bAuthorityEligible =
						JunctionPatch.bQueryCollisionEnabled &&
						JunctionPatch.bApproximationCertified &&
						Algo::AllOf(JunctionPatch.Adjacencies,
							[](const FPiecewiseTensorBezierAdjacency& Link)
							{
								return Link.bC2ByConstruction;
							});
				}
				UE_LOG(LogTemp, Display,
					TEXT("[AnalyticFiniteLowerWallJunction] Candidate=%016llX Side=%+.0f Cells=%d Longitudinal=[%.9g,%.9g] Transverse=[%.9g,%.9g] SourceResidualCm=%.9g Certified=%d"),
					Candidate.CandidateId, SideSign, JunctionPatch.Cells.Num(),
					JunctionMinimumLongitudinal, JunctionMaximumLongitudinal,
					JunctionMinimumTransverse, JunctionMaximumTransverse,
					MaximumJunctionSourceResidualObservedCm,
					JunctionPatch.bAuthorityEligible ? 1 : 0);
				if (JunctionPatch.bAuthorityEligible)
					PiecewiseTensorBezierPatches.Add(MoveTemp(JunctionPatch));
			}
		}

		// The first corner-transition band is not single-valued over the full
		// cap rectangle. Build two compact, independently certified bicubic
		// strips for the lower transition only; the upper lip remains separate.
		const double BandInner = 0.64 * TransverseHalfExtent;
		const double BandOuter = 0.84 * TransverseHalfExtent;
		const double BandTop = InteriorBaseline + 0.42 * VerticalSpan;
		if (BandOuter > BandInner + 100.0 && BandTop > MinimumVertical + 100.0)
		{
			for (const double BandSign : { -1.0, 1.0 })
			{
				const TArray<double> BandU = { BandInner, BandInner + (BandOuter - BandInner) / 3.0,
					BandInner + 2.0 * (BandOuter - BandInner) / 3.0, BandOuter };
				const TArray<double> BandV = { MinimumVertical,
					MinimumVertical + (BandTop - MinimumVertical) / 3.0,
					MinimumVertical + 2.0 * (BandTop - MinimumVertical) / 3.0, BandTop };
				auto SampleBand = [&](const double U, const double V, FVector3d& OutPoint)
				{
					FWorldQuery Query;
					Query.Start[WallAxis] = WallCoordinate - OpeningSign * (FMath::Abs(WallCoordinate) + 1000.0);
					Query.End[WallAxis] = WallCoordinate + OpeningSign * 2000.0;
					Query.Start[TransverseAxis] = Query.End[TransverseAxis] = BandSign * U;
					Query.Start[VerticalAxis] = Query.End[VerticalAxis] = V;
					Query.RequiredSourceId = SourceTriangle.SourceId;
					Query.bIncludeTriangles = true;
					const FWorldHit Hit = SourceQueryService.Sweep(Query);
					if (!Hit.bHit || OpeningSign * (Hit.Point[WallAxis] - WallCoordinate) < 50.0) return false;
					OutPoint = Hit.Point;
					return true;
				};
				TArray<TArray<FCubicBezierSegment>> Rows;
				bool bBandComplete = true;
				for (const double V : BandV)
				{
					TArray<FVector3d> Values;
					for (const double U : BandU)
					{
						FVector3d Point;
						bBandComplete &= SampleBand(U, V, Point);
						Values.Add(Point);
					}
					TArray<FCubicBezierSegment>& Row = Rows.AddDefaulted_GetRef();
					bBandComplete &= BuildNaturalCubicBezierSegments(BandU, Values, Row);
				}
				if (!bBandComplete)
				{
					UE_LOG(LogTemp, Display, TEXT("[AnalyticCornerTransition] Candidate=%016llX Side=%+.0f Incomplete=1"), Candidate.CandidateId, BandSign);
					continue;
				}
				TArray<TArray<FCubicBezierSegment>> Columns;
				for (int32 Control = 0; Control < 4; ++Control)
				{
					TArray<FVector3d> Values;
					for (int32 Row = 0; Row < 4; ++Row) Values.Add(Rows[Row][0].ControlPoints[Control]);
					TArray<FCubicBezierSegment>& Column = Columns.AddDefaulted_GetRef();
					bBandComplete &= BuildNaturalCubicBezierSegments(BandV, Values, Column);
				}
				if (!bBandComplete || Columns.Num() != 4) continue;
				FPiecewiseTensorBezierPatch SidePatch;
				SidePatch.SourceId = SourceTriangle.SourceId;
				SidePatch.SurfaceId = CombineStableIds(Candidate.CandidateId,
					CombineStableIds(StableStringId(TEXT("PiecewiseCornerTransitionC2.V1")), BandSign < 0.0 ? 1ull : 2ull));
				SidePatch.PrimitiveId = CombineStableIds(SidePatch.SurfaceId, StableStringId(TEXT("LowerBand")));
				SidePatch.CanonicalGroupId = StableStringId(TEXT("PiecewiseCornerTransitionC2.Canonical.V1"));
				SidePatch.MaterialId = SourceTriangle.MaterialId;
				SidePatch.ObjectType = SourceTriangle.ObjectType;
				SidePatch.BlockingChannels = SourceTriangle.BlockingChannels;
				SidePatch.bQueryCollisionEnabled = SourceTriangle.bQueryCollisionEnabled;
				FPiecewiseTensorBezierCell& Cell = SidePatch.Cells.AddDefaulted_GetRef();
				Cell.FeatureId = CombineStableIds(SidePatch.SurfaceId, 1ull);
				Cell.PrimitiveId = CombineStableIds(SidePatch.PrimitiveId, 1ull);
				Cell.MinimumU = Cell.MinimumV = 0.0;
				Cell.MaximumU = Cell.MaximumV = 1.0;
				Cell.LongitudinalParameterScale = 1.0;
				Cell.Surface.DegreeU = Cell.Surface.DegreeV = 3;
				Cell.Surface.ControlPoints.SetNumUninitialized(16);
				for (int32 UControl = 0; UControl < 4; ++UControl)
					for (int32 VControl = 0; VControl < 4; ++VControl)
					{
						const int32 PhysicalVControl = OpeningSign < 0.0 ? VControl : 3 - VControl;
						Cell.Surface.ControlPoints[UControl * 4 + VControl] =
							Columns[UControl][0].ControlPoints[PhysicalVControl];
					}
				double BandResidualCm = 0.0;
				for (const double U : { 0.2, 0.5, 0.8 }) for (const double V : { 0.2, 0.5, 0.8 })
				{
					const FVector3d Point = Cell.Surface.Evaluate(U, V);
					FVector3d SourcePoint;
					if (!SampleBand(FMath::Lerp(BandInner, BandOuter, U), FMath::Lerp(MinimumVertical, BandTop, V), SourcePoint)) { BandResidualCm = TNumericLimits<double>::Max(); continue; }
					BandResidualCm = FMath::Max(BandResidualCm, FMath::Abs(SourcePoint[WallAxis] - Point[WallAxis]));
				}
				SidePatch.bSourceResidualCertified = BandResidualCm <= 1.0;
				if (SidePatch.bSourceResidualCertified && SidePatch.BuildQueryApproximation())
				{
					SidePatch.bAuthorityEligible = SidePatch.bQueryCollisionEnabled && SidePatch.bApproximationCertified;
					UE_LOG(LogTemp, Display, TEXT("[AnalyticCornerTransition] Candidate=%016llX Side=%+.0f ResidualCm=%.9g Certified=%d"), Candidate.CandidateId, BandSign, BandResidualCm, SidePatch.bAuthorityEligible ? 1 : 0);
					if (SidePatch.bAuthorityEligible) PiecewiseTensorBezierPatches.Add(MoveTemp(SidePatch));
				}
				else
				{
					UE_LOG(LogTemp, Display, TEXT("[AnalyticCornerTransition] Candidate=%016llX Side=%+.0f ResidualCm=%.9g Certified=0"), Candidate.CandidateId, BandSign, BandResidualCm);
				}
			}
		}

		// The terminal upper lip is a separate, nearly exact planar band. Keep
		// it narrow in height so it cannot span the multi-valued corner below.
		const double LipMinimumV = FMath::Max(725.0, InteriorTop - 0.14 * VerticalSpan);
		const double LipMaximumV = FMath::Min(825.0, InteriorTop - 0.01 * VerticalSpan);
		const double LipHalfWidth = FMath::Min(0.70 * TransverseHalfExtent, 650.0);
		if (LipMaximumV > LipMinimumV + 20.0 && LipHalfWidth > 100.0)
		{
			for (const double LipSign : { -1.0, 1.0 })
			{
				TArray<FVector3d> Samples;
				bool bLipComplete = true;
				for (int32 UIndex = 0; UIndex < 4; ++UIndex)
					for (int32 VIndex = 0; VIndex < 4; ++VIndex)
					{
						const double U = FMath::Lerp(-LipHalfWidth, LipHalfWidth, UIndex / 3.0);
						const double V = FMath::Lerp(LipMinimumV, LipMaximumV, VIndex / 3.0);
						FWorldQuery Query;
						Query.Start[WallAxis] = WallCoordinate - OpeningSign * (FMath::Abs(WallCoordinate) + 1000.0);
						Query.End[WallAxis] = WallCoordinate + OpeningSign * 2000.0;
						Query.Start[TransverseAxis] = Query.End[TransverseAxis] = LipSign * U;
						Query.Start[VerticalAxis] = Query.End[VerticalAxis] = V;
						Query.RequiredSourceId = SourceTriangle.SourceId;
						Query.bIncludeTriangles = true;
						const FWorldHit Hit = SourceQueryService.Sweep(Query);
						bLipComplete &= Hit.bHit && OpeningSign * (Hit.Point[WallAxis] - WallCoordinate) >= 50.0;
						Samples.Add(Hit.Point);
					}
				if (!bLipComplete) continue;
				double LipWall = 0.0;
				for (const FVector3d& Point : Samples) LipWall += Point[WallAxis];
				LipWall /= Samples.Num();
				double LipResidualCm = 0.0;
				for (const FVector3d& Point : Samples) LipResidualCm = FMath::Max(LipResidualCm, FMath::Abs(Point[WallAxis] - LipWall));
				if (LipResidualCm > 1.0) continue;
				FPiecewiseTensorBezierPatch LipPatch;
				LipPatch.SourceId = SourceTriangle.SourceId;
				LipPatch.SurfaceId = CombineStableIds(Candidate.CandidateId, CombineStableIds(StableStringId(TEXT("PiecewiseTerminalLipC2.V1")), LipSign < 0.0 ? 1ull : 2ull));
				LipPatch.PrimitiveId = CombineStableIds(LipPatch.SurfaceId, StableStringId(TEXT("PlanarBand")));
				LipPatch.CanonicalGroupId = StableStringId(TEXT("PiecewiseTerminalLipC2.Canonical.V1"));
				LipPatch.MaterialId = SourceTriangle.MaterialId;
				LipPatch.ObjectType = SourceTriangle.ObjectType;
				LipPatch.BlockingChannels = SourceTriangle.BlockingChannels;
				LipPatch.bQueryCollisionEnabled = SourceTriangle.bQueryCollisionEnabled;
				FPiecewiseTensorBezierCell& LipCell = LipPatch.Cells.AddDefaulted_GetRef();
				LipCell.FeatureId = CombineStableIds(LipPatch.SurfaceId, 1ull);
				LipCell.PrimitiveId = CombineStableIds(LipPatch.PrimitiveId, 1ull);
				LipCell.MinimumU = LipCell.MinimumV = 0.0;
				LipCell.MaximumU = LipCell.MaximumV = 1.0;
				LipCell.LongitudinalParameterScale = 1.0;
				LipCell.Surface.DegreeU = LipCell.Surface.DegreeV = 3;
				LipCell.Surface.ControlPoints.SetNumUninitialized(16);
				for (int32 UControl = 0; UControl < 4; ++UControl)
					for (int32 VControl = 0; VControl < 4; ++VControl)
					{
						const double U = FMath::Lerp(-LipHalfWidth, LipHalfWidth, UControl / 3.0);
						const int32 PhysicalVControl = OpeningSign < 0.0 ? VControl : 3 - VControl;
						const double V = FMath::Lerp(LipMinimumV, LipMaximumV, PhysicalVControl / 3.0);
						FVector3d Point = FVector3d::ZeroVector;
						Point[WallAxis] = LipWall;
						Point[TransverseAxis] = LipSign * U;
						Point[VerticalAxis] = V;
						LipCell.Surface.ControlPoints[UControl * 4 + VControl] = Point;
					}
				LipPatch.bSourceResidualCertified = true;
				if (LipPatch.BuildQueryApproximation())
				{
					LipPatch.bAuthorityEligible = LipPatch.bQueryCollisionEnabled && LipPatch.bApproximationCertified;
					UE_LOG(LogTemp, Display, TEXT("[AnalyticTerminalLip] Candidate=%016llX Side=%+.0f Wall=%.9g ResidualCm=%.9g Certified=%d"), Candidate.CandidateId, LipSign, LipWall, LipResidualCm, LipPatch.bAuthorityEligible ? 1 : 0);
					if (LipPatch.bAuthorityEligible) PiecewiseTensorBezierPatches.Add(MoveTemp(LipPatch));
				}
			}
		}
	}

	// Keep independent ruled transitions out of the terminal-closure merge above:
	// they share source semantics with the surrounding architecture, but they are
	// not members of an open-rim tensor lane.
	PiecewiseTensorBezierPatches.Append(MoveTemp(
		VerticalRuledTransitionPatches));
	// Some broad terminal lanes are appended after the corner-closure pass. Apply
	// the same narrow source-fitted ownership after the complete provider set is
	// assembled, so late lanes cannot reintroduce the ceded adaptive cells.
	for (const FBox3d& LocalBounds : DeferredLocalTerminalOwnershipBounds)
	{
		CedeOverlappingTerminalCells(LocalBounds);
	}
	Algo::Sort(TensorBezierPatches,
		[](const FTensorBezierPatch& A, const FTensorBezierPatch& B)
		{
			return A.PrimitiveId < B.PrimitiveId;
		});
	Algo::Sort(PiecewiseTensorBezierPatches,
		[](const FPiecewiseTensorBezierPatch& A,
			const FPiecewiseTensorBezierPatch& B)
		{
			return A.PrimitiveId < B.PrimitiveId;
		});

	// BuildCompactRuntimePatches runs from BuildRecognitionDiagnostics, which is
	// intentionally invoked after the first FinalizeAndValidate pass.  Rebuild
	// the compact acceleration structure here so providers synthesized by this
	// pass (including the localized goal transitions and closures) are visible to
	// strict runtime queries.  Without this second indexing step the records are
	// serialized and reported as certified, but CompactBvh still contains only
	// the pre-diagnostics provider set.
	CompactPrimitiveIndices.Reset();
	for (int32 PlaneIndex = 0; PlaneIndex < Planes.Num(); ++PlaneIndex)
	{
		if (Planes[PlaneIndex].bRequiresCompactOptIn)
		{
			CompactPrimitiveIndices.Add(PlaneIndex);
		}
	}
	for (int32 PatchIndex = 0;
		PatchIndex < ExtrudedQuinticPatches.Num(); ++PatchIndex)
	{
		CompactPrimitiveIndices.Add(Planes.Num() + PatchIndex);
	}
	CompactBvh.Reset();
	if (!CompactPrimitiveIndices.IsEmpty())
	{
		CompactBvh.Reserve(2 * CompactPrimitiveIndices.Num());
		BuildCompactBvhNode(0, CompactPrimitiveIndices.Num());
	}
	UE_LOG(LogTemp, Display,
		TEXT("[AnalyticCompactRuntimeIndex] Planes=%d Quintics=%d Indexed=%d Nodes=%d"),
		Planes.Num(), ExtrudedQuinticPatches.Num(),
		CompactPrimitiveIndices.Num(), CompactBvh.Num());
}

void FAnalyticWorldData::BuildVertexShapeSamples()
{
	VertexShapeSamples.Init(FVertexShapeSample(), MeshVertices.Num());
	TArray<TArray<int32>> VertexTriangles;
	TArray<TArray<int32>> VertexNeighbors;
	VertexTriangles.SetNum(MeshVertices.Num());
	VertexNeighbors.SetNum(MeshVertices.Num());
	for (int32 TriangleIndex = 0; TriangleIndex < Triangles.Num(); ++TriangleIndex)
	{
		for (int32 Corner = 0; Corner < 3; ++Corner)
		{
			VertexTriangles[TriangleVertexIndices[TriangleIndex][Corner]].Add(
				TriangleIndex);
		}
	}
	for (const FTriangleMeshEdge& Edge : MeshEdges)
	{
		if (Edge.Continuity == EEdgeContinuity::Crease || Edge.IsNonManifold()) continue;
		VertexNeighbors[Edge.VertexA].Add(Edge.VertexB);
		VertexNeighbors[Edge.VertexB].Add(Edge.VertexA);
	}
	TArray<double> EligibleEdgeLengths;
	EligibleEdgeLengths.Reserve(MeshEdges.Num());
	for (const FTriangleMeshEdge& Edge : MeshEdges)
	{
		if (Edge.Continuity == EEdgeContinuity::Crease || Edge.IsNonManifold()) continue;
		EligibleEdgeLengths.Add(FVector3d::Distance(
			MeshVertices[Edge.VertexA], MeshVertices[Edge.VertexB]));
	}
	Algo::Sort(EligibleEdgeLengths);
	const int32 RadiusQuantileIndex = EligibleEdgeLengths.IsEmpty() ? 0 :
		FMath::Clamp(FMath::RoundToInt(0.90 *
			static_cast<double>(EligibleEdgeLengths.Num() - 1)),
			0, EligibleEdgeLengths.Num() - 1);
	// Long polygon diagonals from exact planar panels are not local curvature
	// support. The deterministic P90 retains the ordinary tessellation scale
	// while leaving those panels to the exact planar constraints.
	const double MaximumShapeSupportRadiusCm = EligibleEdgeLengths.IsEmpty()
		? 0.0 : EligibleEdgeLengths[RadiusQuantileIndex];

	TArray<FVector3d> VertexNormals;
	TArray<bool> bVertexNormalValid;
	TArray<bool> bVertexTouchesCrease;
	VertexNormals.Init(FVector3d::ZeroVector, MeshVertices.Num());
	bVertexNormalValid.Init(false, MeshVertices.Num());
	bVertexTouchesCrease.Init(false, MeshVertices.Num());
	for (const FTriangleMeshEdge& Edge : MeshEdges)
	{
		if (Edge.Continuity != EEdgeContinuity::Crease && !Edge.IsNonManifold()) continue;
		bVertexTouchesCrease[Edge.VertexA] = true;
		bVertexTouchesCrease[Edge.VertexB] = true;
	}
	for (int32 VertexIndex = 0; VertexIndex < MeshVertices.Num(); ++VertexIndex)
	{
		FVector3d NormalSum = FVector3d::ZeroVector;
		if (bVertexTouchesCrease[VertexIndex]) continue;
		for (const int32 TriangleIndex : VertexTriangles[VertexIndex])
		{
			const FIntVector& VertexIndices = TriangleVertexIndices[TriangleIndex];
			for (int32 Corner = 0; Corner < 3; ++Corner)
			{
				if (VertexIndices[Corner] != VertexIndex) continue;
				const FVector3d EdgeA =
					MeshVertices[VertexIndices[(Corner + 1) % 3]] - MeshVertices[VertexIndex];
				const FVector3d EdgeB =
					MeshVertices[VertexIndices[(Corner + 2) % 3]] - MeshVertices[VertexIndex];
				const double CornerAngle = FMath::Acos(FMath::Clamp(
					FVector3d::DotProduct(EdgeA.GetSafeNormal(), EdgeB.GetSafeNormal()),
					-1.0, 1.0));
				NormalSum += CornerAngle * Triangles[TriangleIndex].FaceNormal;
			}
		}
		if (!NormalSum.IsNearlyZero())
		{
			VertexNormals[VertexIndex] = NormalSum.GetSafeNormal();
			bVertexNormalValid[VertexIndex] = true;
		}
	}

	for (int32 VertexIndex = 0; VertexIndex < MeshVertices.Num(); ++VertexIndex)
	{
		FVertexShapeSample& Sample = VertexShapeSamples[VertexIndex];
		if (!bVertexNormalValid[VertexIndex])
		{
			continue;
		}
		constexpr int32 ShapeSupportRingCount = 2;
		TSet<int32> Visited;
		TArray<int32, TInlineAllocator<32>> Frontier;
		TArray<int32, TInlineAllocator<64>> SupportVertices;
		Visited.Add(VertexIndex);
		Frontier.Add(VertexIndex);
		for (int32 Ring = 0; Ring < ShapeSupportRingCount; ++Ring)
		{
			TArray<int32, TInlineAllocator<32>> NextFrontier;
			for (const int32 FrontierVertex : Frontier)
			{
				for (const int32 NeighborIndex : VertexNeighbors[FrontierVertex])
				{
					if (!bVertexNormalValid[NeighborIndex] || Visited.Contains(NeighborIndex))
					{
						continue;
					}
					if (FVector3d::Distance(
						MeshVertices[NeighborIndex], MeshVertices[VertexIndex]) >
						MaximumShapeSupportRadiusCm)
					{
						continue;
					}
					Visited.Add(NeighborIndex);
					NextFrontier.Add(NeighborIndex);
					SupportVertices.Add(NeighborIndex);
				}
			}
			Algo::Sort(NextFrontier);
			Frontier = MoveTemp(NextFrontier);
		}
		Algo::Sort(SupportVertices);
		if (SupportVertices.Num() < 3) continue;
		Sample.SupportRingCount = ShapeSupportRingCount;
		const FVector3d Normal = VertexNormals[VertexIndex];
		const FVector3d AbsNormal = Normal.GetAbs();
		const FVector3d ReferenceAxis =
			AbsNormal.X <= AbsNormal.Y && AbsNormal.X <= AbsNormal.Z
			? FVector3d::ForwardVector
			: (AbsNormal.Y <= AbsNormal.Z
				? FVector3d::RightVector : FVector3d::UpVector);
		const FVector3d AxisU = FVector3d::CrossProduct(Normal, ReferenceAxis).GetSafeNormal();
		const FVector3d AxisV = FVector3d::CrossProduct(Normal, AxisU).GetSafeNormal();
		double Matrix[3][3] = {};
		double RightHandSide[3] = {};
		struct FEquationSample { double Coefficients[3]; double Target; };
		TArray<FEquationSample, TInlineAllocator<16>> Equations;
		for (const int32 NeighborIndex : SupportVertices)
		{
			const FVector3d Delta = MeshVertices[NeighborIndex] - MeshVertices[VertexIndex];
			const FVector3d TangentDelta = Delta -
				FVector3d::DotProduct(Delta, Normal) * Normal;
			const double Distance = TangentDelta.Length();
			if (Distance <= 1.0e-9) continue;
			Sample.SupportRadiusCm = FMath::Max(Sample.SupportRadiusCm, Delta.Length());
			const FVector3d Direction = TangentDelta / Distance;
			const double U = FVector3d::DotProduct(Direction, AxisU);
			const double V = FVector3d::DotProduct(Direction, AxisV);
			const FVector3d NormalDerivative =
				(VertexNormals[NeighborIndex] - Normal) / Distance;
			Equations.Add({ { -U, -V, 0.0 },
				FVector3d::DotProduct(NormalDerivative, AxisU) });
			Equations.Add({ { 0.0, -U, -V },
				FVector3d::DotProduct(NormalDerivative, AxisV) });
			++Sample.NeighborCount;
		}
		if (Sample.NeighborCount < 3) continue;
		for (const FEquationSample& Equation : Equations)
		{
			for (int32 Row = 0; Row < 3; ++Row)
			{
				RightHandSide[Row] += Equation.Coefficients[Row] * Equation.Target;
				for (int32 Column = 0; Column < 3; ++Column)
				{
					Matrix[Row][Column] += Equation.Coefficients[Row] *
						Equation.Coefficients[Column];
				}
			}
		}
		double Solution[3] = {};
		if (!SolveSymmetricNormalEquations(Matrix, RightHandSide, Solution)) continue;
		const double A = Solution[0];
		const double B = Solution[1];
		const double C = Solution[2];
		const double HalfTrace = 0.5 * (A + C);
		const double Radius = FMath::Sqrt(
			0.25 * FMath::Square(A - C) + FMath::Square(B));
		Sample.MinimumPrincipalCurvature = HalfTrace - Radius;
		Sample.MaximumPrincipalCurvature = HalfTrace + Radius;
		Sample.MeanCurvature = HalfTrace;
		Sample.GaussianCurvature =
			Sample.MinimumPrincipalCurvature * Sample.MaximumPrincipalCurvature;
		FVector2d MaximumEigenvector(B, Sample.MaximumPrincipalCurvature - A);
		if (MaximumEigenvector.IsNearlyZero()) MaximumEigenvector = FVector2d(1.0, 0.0);
		MaximumEigenvector.Normalize();
		const FVector2d MinimumEigenvector(-MaximumEigenvector.Y, MaximumEigenvector.X);
		Sample.MaximumDirection =
			MaximumEigenvector.X * AxisU + MaximumEigenvector.Y * AxisV;
		Sample.MinimumDirection =
			MinimumEigenvector.X * AxisU + MinimumEigenvector.Y * AxisV;
		double SquaredResidual = 0.0;
		for (const FEquationSample& Equation : Equations)
		{
			const double Prediction = Equation.Coefficients[0] * A +
				Equation.Coefficients[1] * B + Equation.Coefficients[2] * C;
			SquaredResidual += FMath::Square(Prediction - Equation.Target);
		}
		Sample.RootMeanSquareResidual = FMath::Sqrt(
			SquaredResidual / static_cast<double>(Equations.Num()));
		Sample.Normal = Normal;
		Sample.bValid = FMath::IsFinite(Sample.MinimumPrincipalCurvature) &&
			FMath::IsFinite(Sample.MaximumPrincipalCurvature) &&
			FMath::IsFinite(Sample.RootMeanSquareResidual);
	}
}

void FAnalyticWorldData::BuildPlanarSurfaceGroups()
{
	PlanarSurfaceGroups.Reset();
	PlanarGroupPatchIndices.Reset();
	PatchPlanarGroupIndices.Init(INDEX_NONE, SurfacePatches.Num());
	constexpr double PlaneOffsetToleranceCm = 0.02;
	constexpr double NormalAngleToleranceDegrees = 0.05;
	const double MinimumNormalDot = FMath::Cos(
		FMath::DegreesToRadians(NormalAngleToleranceDegrees));
	TArray<bool> Assigned;
	Assigned.Init(false, SurfacePatches.Num());
	for (int32 SeedIndex = 0; SeedIndex < SurfacePatches.Num(); ++SeedIndex)
	{
		const FSurfacePatch& Seed = SurfacePatches[SeedIndex];
		if (Assigned[SeedIndex] || Seed.Kind != ESurfacePatchKind::PlanarCandidate)
		{
			continue;
		}
		const FVector3d GroupNormal = CanonicalPlaneNormal(Seed.Normal);
		const double GroupOffset = FVector3d::DotProduct(GroupNormal, Seed.Origin);
		TArray<int32, TInlineAllocator<16>> Members;
		for (int32 CandidateIndex = SeedIndex;
			CandidateIndex < SurfacePatches.Num(); ++CandidateIndex)
		{
			const FSurfacePatch& Candidate = SurfacePatches[CandidateIndex];
			if (Assigned[CandidateIndex] ||
				Candidate.Kind != ESurfacePatchKind::PlanarCandidate ||
				Candidate.SourceId != Seed.SourceId ||
				Candidate.SurfaceId != Seed.SurfaceId ||
				Candidate.MaterialId != Seed.MaterialId)
			{
				continue;
			}
			const FVector3d CandidateNormal = CanonicalPlaneNormal(Candidate.Normal);
			const double CandidateOffset = FVector3d::DotProduct(
				CandidateNormal, Candidate.Origin);
			if (FVector3d::DotProduct(GroupNormal, CandidateNormal) < MinimumNormalDot ||
				FMath::Abs(CandidateOffset - GroupOffset) > PlaneOffsetToleranceCm)
			{
				continue;
			}
			Assigned[CandidateIndex] = true;
			Members.Add(CandidateIndex);
		}

		FPlanarSurfaceGroup& Group = PlanarSurfaceGroups.AddDefaulted_GetRef();
		Group.GroupId = CombineStableIds(Seed.SourceId, Seed.PatchId);
		Group.SourceId = Seed.SourceId;
		Group.SurfaceId = Seed.SurfaceId;
		Group.MaterialId = Seed.MaterialId;
		Group.FirstPatchIndex = PlanarGroupPatchIndices.Num();
		Group.PatchCount = Members.Num();
		Group.PlaneOffset = GroupOffset;
		Group.Normal = GroupNormal;
		for (const int32 PatchIndex : Members)
		{
			const FSurfacePatch& Patch = SurfacePatches[PatchIndex];
			PatchPlanarGroupIndices[PatchIndex] = PlanarSurfaceGroups.Num() - 1;
			PlanarGroupPatchIndices.Add(PatchIndex);
			Group.GroupId = CombineStableIds(Group.GroupId, Patch.PatchId);
			Group.TriangleCount += Patch.TriangleCount;
			Group.Area += Patch.Area;
			Group.Bounds += Patch.Bounds;
			const FVector3d PatchNormal = CanonicalPlaneNormal(Patch.Normal);
			Group.MaximumNormalAngleDegrees = FMath::Max(
				Group.MaximumNormalAngleDegrees,
				VectorAngleDegrees(GroupNormal, PatchNormal));
			for (int32 Offset = 0; Offset < Patch.TriangleCount; ++Offset)
			{
				const int32 TriangleIndex = PatchTriangleIndices[
					Patch.FirstTriangleIndex + Offset];
				for (int32 Corner = 0; Corner < 3; ++Corner)
				{
					Group.MaximumPlaneResidual = FMath::Max(
						Group.MaximumPlaneResidual,
						FMath::Abs(FVector3d::DotProduct(
							GroupNormal, Triangles[TriangleIndex].Vertices[Corner]) -
							GroupOffset));
				}
			}
		}
	}
	double MaximumPlanarGroupArea = 0.0;
	for (const FPlanarSurfaceGroup& Group : PlanarSurfaceGroups)
	{
		MaximumPlanarGroupArea = FMath::Max(MaximumPlanarGroupArea, Group.Area);
	}
	constexpr double ArchitecturalAreaFraction = 0.01;
	for (FPlanarSurfaceGroup& Group : PlanarSurfaceGroups)
	{
		Group.bArchitecturalConstraint = MaximumPlanarGroupArea > 0.0 &&
			Group.Area >= ArchitecturalAreaFraction * MaximumPlanarGroupArea;
	}
}

void FAnalyticWorldData::BuildPlanarGroupMirrorMatches()
{
	PlanarGroupMirrorMatches.Reset();
	if (PlanarSurfaceGroups.IsEmpty()) return;
	auto ReflectedBounds = [](const FBox3d& Bounds,
		const EPlanarSymmetryAxis Axis)
	{
		FBox3d Result = Bounds;
		const int32 AxisIndex = Axis == EPlanarSymmetryAxis::X ? 0 : 1;
		Result.Min[AxisIndex] = -Bounds.Max[AxisIndex];
		Result.Max[AxisIndex] = -Bounds.Min[AxisIndex];
		return Result;
	};
	auto BoundsResidual = [](const FBox3d& A, const FBox3d& B)
	{
		double Result = 0.0;
		for (int32 AxisIndex = 0; AxisIndex < 3; ++AxisIndex)
		{
			Result = FMath::Max(Result, FMath::Abs(A.Min[AxisIndex] - B.Min[AxisIndex]));
			Result = FMath::Max(Result, FMath::Abs(A.Max[AxisIndex] - B.Max[AxisIndex]));
		}
		return Result;
	};

	for (const EPlanarSymmetryAxis Axis : {
		EPlanarSymmetryAxis::X, EPlanarSymmetryAxis::Y })
	{
		const int32 FirstMatch = PlanarGroupMirrorMatches.Num();
		for (int32 SourceIndex = 0; SourceIndex < PlanarSurfaceGroups.Num(); ++SourceIndex)
		{
			const FPlanarSurfaceGroup& Source = PlanarSurfaceGroups[SourceIndex];
			FVector3d MirroredNormal = Source.Normal;
			MirroredNormal[Axis == EPlanarSymmetryAxis::X ? 0 : 1] *= -1.0;
			double MirroredOffset = Source.PlaneOffset;
			const FVector3d CanonicalMirroredNormal =
				CanonicalPlaneNormal(MirroredNormal);
			if (FVector3d::DotProduct(CanonicalMirroredNormal, MirroredNormal) < 0.0)
			{
				MirroredOffset *= -1.0;
			}
			const FBox3d MirroredBounds = ReflectedBounds(Source.Bounds, Axis);
			FPlanarGroupMirrorMatch Best;
			Best.Axis = Axis;
			Best.SourceGroupIndex = SourceIndex;
			Best.Score = TNumericLimits<double>::Max();
			for (int32 TargetIndex = 0; TargetIndex < PlanarSurfaceGroups.Num(); ++TargetIndex)
			{
				const FPlanarSurfaceGroup& Target = PlanarSurfaceGroups[TargetIndex];
				if (Target.SourceId != Source.SourceId ||
					Target.SurfaceId != Source.SurfaceId ||
					Target.MaterialId != Source.MaterialId)
				{
					continue;
				}
				FPlanarGroupMirrorMatch Candidate;
				Candidate.Axis = Axis;
				Candidate.SourceGroupIndex = SourceIndex;
				Candidate.TargetGroupIndex = TargetIndex;
				Candidate.PlaneOffsetResidual = FMath::Abs(
					MirroredOffset - Target.PlaneOffset);
				Candidate.NormalAngleResidualDegrees = VectorAngleDegrees(
					CanonicalMirroredNormal, Target.Normal);
				Candidate.BoundsResidual = BoundsResidual(MirroredBounds, Target.Bounds);
				Candidate.RelativeAreaResidual = FMath::Abs(Source.Area - Target.Area) /
					FMath::Max(Source.Area, Target.Area);
				Candidate.Score = Candidate.PlaneOffsetResidual +
					100.0 * Candidate.NormalAngleResidualDegrees +
					0.01 * Candidate.BoundsResidual +
					100.0 * Candidate.RelativeAreaResidual;
				if (Candidate.Score < Best.Score ||
					(Candidate.Score == Best.Score && Target.GroupId <
						PlanarSurfaceGroups[Best.TargetGroupIndex].GroupId))
				{
					Best = Candidate;
				}
			}
			PlanarGroupMirrorMatches.Add(Best);
		}
		for (int32 MatchIndex = FirstMatch;
			MatchIndex < PlanarGroupMirrorMatches.Num(); ++MatchIndex)
		{
			FPlanarGroupMirrorMatch& Match = PlanarGroupMirrorMatches[MatchIndex];
			if (Match.TargetGroupIndex == INDEX_NONE) continue;
			const int32 ReverseIndex = FirstMatch + Match.TargetGroupIndex;
			Match.bReciprocal = PlanarGroupMirrorMatches.IsValidIndex(ReverseIndex) &&
				PlanarGroupMirrorMatches[ReverseIndex].TargetGroupIndex ==
				Match.SourceGroupIndex;
		}
	}
}

void FAnalyticWorldData::BuildTriangleCurvatureEvidence()
{
	TriangleCurvatureEvidence.Init(FTriangleCurvatureEvidence(), Triangles.Num());
	constexpr double MinimumCurvatureMagnitude = 1.0e-6;
	constexpr double DevelopableAnisotropyThreshold = 0.10;
	constexpr double ReliableRelativeResidualThreshold = 0.25;
	for (int32 TriangleIndex = 0; TriangleIndex < Triangles.Num(); ++TriangleIndex)
	{
		FTriangleCurvatureEvidence& Evidence =
			TriangleCurvatureEvidence[TriangleIndex];
		const int32 PatchIndex = TrianglePatchIndices[TriangleIndex];
		const int32 PlanarGroupIndex = PatchPlanarGroupIndices.IsValidIndex(PatchIndex)
			? PatchPlanarGroupIndices[PatchIndex] : INDEX_NONE;
		if (PlanarSurfaceGroups.IsValidIndex(PlanarGroupIndex) &&
			PlanarSurfaceGroups[PlanarGroupIndex].bArchitecturalConstraint)
		{
			Evidence.Kind = ECurvatureEvidenceKind::PlanarConstraint;
			Evidence.ValidSampleCount = 3;
			Evidence.CurvatureAnisotropy = 0.0;
			Evidence.MaximumRelativeFitResidual = 0.0;
			Evidence.bReliable = true;
			continue;
		}

		FVector3d DirectionSum = FVector3d::ZeroVector;
		FVector3d FirstDirection = FVector3d::ZeroVector;
		for (int32 Corner = 0; Corner < 3; ++Corner)
		{
			const FVertexShapeSample& Sample = VertexShapeSamples[
				TriangleVertexIndices[TriangleIndex][Corner]];
			if (!Sample.bValid) continue;
			++Evidence.ValidSampleCount;
			Evidence.MinimumPrincipalCurvature += Sample.MinimumPrincipalCurvature;
			Evidence.MaximumPrincipalCurvature += Sample.MaximumPrincipalCurvature;
			const double Dominant = FMath::Max(
				FMath::Abs(Sample.MinimumPrincipalCurvature),
				FMath::Abs(Sample.MaximumPrincipalCurvature));
			Evidence.MaximumRelativeFitResidual = FMath::Max(
				Evidence.MaximumRelativeFitResidual,
				Dominant > MinimumCurvatureMagnitude
					? Sample.RootMeanSquareResidual / Dominant
					: TNumericLimits<double>::Max());
			FVector3d LowDirection =
				FMath::Abs(Sample.MinimumPrincipalCurvature) <=
				FMath::Abs(Sample.MaximumPrincipalCurvature)
					? Sample.MinimumDirection : Sample.MaximumDirection;
			if (FirstDirection.IsNearlyZero()) FirstDirection = LowDirection;
			if (FVector3d::DotProduct(FirstDirection, LowDirection) < 0.0)
			{
				LowDirection *= -1.0;
			}
			DirectionSum += LowDirection;
		}
		if (Evidence.ValidSampleCount != 3) continue;
		Evidence.MinimumPrincipalCurvature /= 3.0;
		Evidence.MaximumPrincipalCurvature /= 3.0;
		Evidence.MaximumAbsoluteCurvature = FMath::Max(
			FMath::Abs(Evidence.MinimumPrincipalCurvature),
			FMath::Abs(Evidence.MaximumPrincipalCurvature));
		const double MinimumAbsoluteCurvature = FMath::Min(
			FMath::Abs(Evidence.MinimumPrincipalCurvature),
			FMath::Abs(Evidence.MaximumPrincipalCurvature));
		Evidence.CurvatureAnisotropy = Evidence.MaximumAbsoluteCurvature > 0.0
			? MinimumAbsoluteCurvature / Evidence.MaximumAbsoluteCurvature : 1.0;
		Evidence.LowCurvatureDirection = DirectionSum.GetSafeNormal();
		Evidence.bReliable =
			Evidence.MaximumAbsoluteCurvature > MinimumCurvatureMagnitude &&
			Evidence.MaximumRelativeFitResidual <= ReliableRelativeResidualThreshold;
		if (!Evidence.bReliable) continue;
		if (Evidence.CurvatureAnisotropy <= DevelopableAnisotropyThreshold)
		{
			Evidence.Kind = ECurvatureEvidenceKind::DevelopableCandidate;
		}
		else if (Evidence.MinimumPrincipalCurvature *
			Evidence.MaximumPrincipalCurvature > 0.0)
		{
			Evidence.Kind = ECurvatureEvidenceKind::EllipticCandidate;
		}
		else
		{
			Evidence.Kind = ECurvatureEvidenceKind::SaddleCandidate;
		}
	}
}

void FAnalyticWorldData::BuildCurvatureSurfaceRegions()
{
	CurvatureSurfaceRegions.Reset();
	CurvatureRegionTriangleIndices.Reset();
	TriangleCurvatureRegionIndices.Init(INDEX_NONE, Triangles.Num());
	TArray<bool> Assigned;
	Assigned.Init(false, Triangles.Num());
	// Elliptic tessellation around a finite rounded opening naturally changes
	// estimated principal curvature across neighboring facets.  Keep the
	// grouping topology- and material-constrained, but admit one smooth
	// physical family instead of splitting it at each estimator fluctuation.
	constexpr double MaximumDominantCurvatureRatio = 1.75;
	const double MinimumDirectionDot = FMath::Cos(FMath::DegreesToRadians(15.0));
	auto Compatible = [MaximumDominantCurvatureRatio, MinimumDirectionDot](
		const FTriangleCurvatureEvidence& Seed,
		const FTriangleCurvatureEvidence& Candidate)
	{
		if (!Candidate.bReliable || Candidate.Kind != Seed.Kind) return false;
		const double MinimumDominant = FMath::Min(
			Seed.MaximumAbsoluteCurvature, Candidate.MaximumAbsoluteCurvature);
		const double MaximumDominant = FMath::Max(
			Seed.MaximumAbsoluteCurvature, Candidate.MaximumAbsoluteCurvature);
		if (MinimumDominant <= 0.0 ||
			MaximumDominant / MinimumDominant > MaximumDominantCurvatureRatio)
		{
			return false;
		}
		const bool bDirectionIsMeaningful =
			Seed.Kind == ECurvatureEvidenceKind::DevelopableCandidate ||
			Seed.CurvatureAnisotropy < 0.80 || Candidate.CurvatureAnisotropy < 0.80;
		return !bDirectionIsMeaningful || FMath::Abs(FVector3d::DotProduct(
			Seed.LowCurvatureDirection, Candidate.LowCurvatureDirection)) >=
			MinimumDirectionDot;
	};
	TArray<int32> Pending;
	TArray<int32> Members;
	for (int32 SeedIndex = 0; SeedIndex < Triangles.Num(); ++SeedIndex)
	{
		const FTriangleCurvatureEvidence& SeedEvidence =
			TriangleCurvatureEvidence[SeedIndex];
		if (Assigned[SeedIndex] || !SeedEvidence.bReliable ||
			SeedEvidence.Kind == ECurvatureEvidenceKind::PlanarConstraint ||
			SeedEvidence.Kind == ECurvatureEvidenceKind::Unresolved)
		{
			continue;
		}
		const FTriangleSurface& SeedTriangle = Triangles[SeedIndex];
		Pending.Reset();
		Members.Reset();
		Pending.Add(SeedIndex);
		Assigned[SeedIndex] = true;
		while (!Pending.IsEmpty())
		{
			const int32 TriangleIndex = Pending.Pop(EAllowShrinking::No);
			Members.Add(TriangleIndex);
			for (int32 LocalEdge = 0; LocalEdge < 3; ++LocalEdge)
			{
				const int32 NeighborIndex = TriangleNeighborIndices[TriangleIndex][LocalEdge];
				if (NeighborIndex == INDEX_NONE || Assigned[NeighborIndex]) continue;
				const FTriangleMeshEdge& Edge = MeshEdges[
					TriangleEdgeIndices[TriangleIndex][LocalEdge]];
				const FTriangleSurface& Neighbor = Triangles[NeighborIndex];
				if (Edge.Continuity != EEdgeContinuity::Smooth ||
					Neighbor.SourceId != SeedTriangle.SourceId ||
					Neighbor.SurfaceId != SeedTriangle.SurfaceId ||
					Neighbor.MaterialId != SeedTriangle.MaterialId ||
					!Compatible(SeedEvidence, TriangleCurvatureEvidence[NeighborIndex]))
				{
					continue;
				}
				Assigned[NeighborIndex] = true;
				Pending.Add(NeighborIndex);
			}
		}
		Algo::Sort(Members);
		FCurvatureSurfaceRegion& Region =
			CurvatureSurfaceRegions.AddDefaulted_GetRef();
		Region.RegionId = CombineStableIds(
			SeedTriangle.SourceId, SeedTriangle.PrimitiveId);
		Region.SourceId = SeedTriangle.SourceId;
		Region.SurfaceId = SeedTriangle.SurfaceId;
		Region.MaterialId = SeedTriangle.MaterialId;
		Region.Kind = SeedEvidence.Kind;
		Region.FirstTriangleIndex = CurvatureRegionTriangleIndices.Num();
		Region.TriangleCount = Members.Num();
		FVector3d DirectionSum = FVector3d::ZeroVector;
		FVector3d FirstDirection = FVector3d::ZeroVector;
		for (const int32 TriangleIndex : Members)
		{
			TriangleCurvatureRegionIndices[TriangleIndex] =
				CurvatureSurfaceRegions.Num() - 1;
			CurvatureRegionTriangleIndices.Add(TriangleIndex);
			const FTriangleSurface& Triangle = Triangles[TriangleIndex];
			const FTriangleCurvatureEvidence& Evidence =
				TriangleCurvatureEvidence[TriangleIndex];
			const double Area = 0.5 * FVector3d::CrossProduct(
				Triangle.Vertices[1] - Triangle.Vertices[0],
				Triangle.Vertices[2] - Triangle.Vertices[0]).Length();
			Region.Area += Area;
			Region.MeanMinimumPrincipalCurvature +=
				Area * Evidence.MinimumPrincipalCurvature;
			Region.MeanMaximumPrincipalCurvature +=
				Area * Evidence.MaximumPrincipalCurvature;
			Region.MinimumAbsoluteDominantCurvature = FMath::Min(
				Region.MinimumAbsoluteDominantCurvature,
				Evidence.MaximumAbsoluteCurvature);
			Region.MaximumAbsoluteDominantCurvature = FMath::Max(
				Region.MaximumAbsoluteDominantCurvature,
				Evidence.MaximumAbsoluteCurvature);
			Region.MaximumRelativeFitResidual = FMath::Max(
				Region.MaximumRelativeFitResidual,
				Evidence.MaximumRelativeFitResidual);
			Region.Bounds += Triangle.Bounds;
			FVector3d Direction = Evidence.LowCurvatureDirection;
			if (FirstDirection.IsNearlyZero()) FirstDirection = Direction;
			if (FVector3d::DotProduct(FirstDirection, Direction) < 0.0)
			{
				Direction *= -1.0;
			}
			DirectionSum += Area * Direction;
		}
		if (Region.Area > 0.0)
		{
			Region.MeanMinimumPrincipalCurvature /= Region.Area;
			Region.MeanMaximumPrincipalCurvature /= Region.Area;
		}
		Region.MeanLowCurvatureDirection = DirectionSum.GetSafeNormal();
	}
}

void FAnalyticWorldData::BuildCurvatureRegionMirrorMatches()
{
	CurvatureRegionMirrorMatches.Reset();
	if (CurvatureSurfaceRegions.IsEmpty()) return;
	auto ReflectedBounds = [](const FBox3d& Bounds,
		const EPlanarSymmetryAxis Axis)
	{
		FBox3d Result = Bounds;
		const int32 AxisIndex = Axis == EPlanarSymmetryAxis::X ? 0 : 1;
		Result.Min[AxisIndex] = -Bounds.Max[AxisIndex];
		Result.Max[AxisIndex] = -Bounds.Min[AxisIndex];
		return Result;
	};
	auto BoundsResidual = [](const FBox3d& A, const FBox3d& B)
	{
		double Result = 0.0;
		for (int32 AxisIndex = 0; AxisIndex < 3; ++AxisIndex)
		{
			Result = FMath::Max(Result,
				FMath::Abs(A.Min[AxisIndex] - B.Min[AxisIndex]));
			Result = FMath::Max(Result,
				FMath::Abs(A.Max[AxisIndex] - B.Max[AxisIndex]));
		}
		return Result;
	};

	for (const EPlanarSymmetryAxis Axis : {
		EPlanarSymmetryAxis::X, EPlanarSymmetryAxis::Y })
	{
		const int32 FirstMatch = CurvatureRegionMirrorMatches.Num();
		for (int32 SourceIndex = 0;
			SourceIndex < CurvatureSurfaceRegions.Num(); ++SourceIndex)
		{
			const FCurvatureSurfaceRegion& Source =
				CurvatureSurfaceRegions[SourceIndex];
			const FBox3d MirroredBounds = ReflectedBounds(Source.Bounds, Axis);
			FVector3d MirroredLowDirection = Source.MeanLowCurvatureDirection;
			MirroredLowDirection[Axis == EPlanarSymmetryAxis::X ? 0 : 1] *= -1.0;
			FCurvatureRegionMirrorMatch Best;
			Best.Axis = Axis;
			Best.SourceRegionIndex = SourceIndex;
			Best.Score = TNumericLimits<double>::Max();
			for (int32 TargetIndex = 0;
				TargetIndex < CurvatureSurfaceRegions.Num(); ++TargetIndex)
			{
				const FCurvatureSurfaceRegion& Target =
					CurvatureSurfaceRegions[TargetIndex];
				if (Target.SourceId != Source.SourceId ||
					Target.SurfaceId != Source.SurfaceId ||
					Target.MaterialId != Source.MaterialId || Target.Kind != Source.Kind)
				{
					continue;
				}
				FCurvatureRegionMirrorMatch Candidate;
				Candidate.Axis = Axis;
				Candidate.SourceRegionIndex = SourceIndex;
				Candidate.TargetRegionIndex = TargetIndex;
				Candidate.MinimumCurvatureResidual = FMath::Abs(
					Source.MeanMinimumPrincipalCurvature -
					Target.MeanMinimumPrincipalCurvature);
				Candidate.MaximumCurvatureResidual = FMath::Abs(
					Source.MeanMaximumPrincipalCurvature -
					Target.MeanMaximumPrincipalCurvature);
				Candidate.LowDirectionAngleResidualDegrees = VectorAngleDegrees(
					MirroredLowDirection, Target.MeanLowCurvatureDirection);
				Candidate.LowDirectionAngleResidualDegrees = FMath::Min(
					Candidate.LowDirectionAngleResidualDegrees,
					180.0 - Candidate.LowDirectionAngleResidualDegrees);
				Candidate.BoundsResidual = BoundsResidual(MirroredBounds, Target.Bounds);
				Candidate.RelativeAreaResidual = FMath::Abs(Source.Area - Target.Area) /
					FMath::Max(Source.Area, Target.Area);
				const double CurvatureScale = FMath::Max(
					FMath::Max(FMath::Abs(Source.MeanMinimumPrincipalCurvature),
						FMath::Abs(Source.MeanMaximumPrincipalCurvature)), 1.0e-6);
				const bool bDirectionIsMeaningful =
					Source.Kind == ECurvatureEvidenceKind::DevelopableCandidate;
				Candidate.Score =
					100.0 * (Candidate.MinimumCurvatureResidual +
						Candidate.MaximumCurvatureResidual) / CurvatureScale +
					(bDirectionIsMeaningful
						? Candidate.LowDirectionAngleResidualDegrees : 0.0) +
					0.01 * Candidate.BoundsResidual +
					100.0 * Candidate.RelativeAreaResidual;
				if (Candidate.Score < Best.Score ||
					(Candidate.Score == Best.Score && Target.RegionId <
						CurvatureSurfaceRegions[Best.TargetRegionIndex].RegionId))
				{
					Best = Candidate;
				}
			}
			CurvatureRegionMirrorMatches.Add(Best);
		}
		for (int32 MatchIndex = FirstMatch;
			MatchIndex < CurvatureRegionMirrorMatches.Num(); ++MatchIndex)
		{
			FCurvatureRegionMirrorMatch& Match =
				CurvatureRegionMirrorMatches[MatchIndex];
			if (Match.TargetRegionIndex == INDEX_NONE) continue;
			const int32 ReverseIndex = FirstMatch + Match.TargetRegionIndex;
			Match.bReciprocal = CurvatureRegionMirrorMatches.IsValidIndex(ReverseIndex) &&
				CurvatureRegionMirrorMatches[ReverseIndex].TargetRegionIndex ==
				Match.SourceRegionIndex;
			Match.bSelfMirror = Match.TargetRegionIndex == Match.SourceRegionIndex;
			const FCurvatureSurfaceRegion& Source =
				CurvatureSurfaceRegions[Match.SourceRegionIndex];
			const FCurvatureSurfaceRegion& Target =
				CurvatureSurfaceRegions[Match.TargetRegionIndex];
			const double CurvatureScale = FMath::Max(
				FMath::Max(
					FMath::Max(FMath::Abs(Source.MeanMinimumPrincipalCurvature),
						FMath::Abs(Source.MeanMaximumPrincipalCurvature)),
					FMath::Max(FMath::Abs(Target.MeanMinimumPrincipalCurvature),
						FMath::Abs(Target.MeanMaximumPrincipalCurvature))), 1.0e-6);
			const double RelativeCurvatureResidual =
				(Match.MinimumCurvatureResidual + Match.MaximumCurvatureResidual) /
				CurvatureScale;
			const bool bDirectionPlausible =
				Source.Kind != ECurvatureEvidenceKind::DevelopableCandidate ||
				Match.LowDirectionAngleResidualDegrees <= 15.0;
			Match.bPlausible = Match.bReciprocal &&
				RelativeCurvatureResidual <= 0.15 &&
				Match.RelativeAreaResidual <= 0.10 &&
				Match.BoundsResidual <= 25.0 && bDirectionPlausible;
		}
	}
}

void FAnalyticWorldData::BuildExtrusionSurfaceRegions()
{
	TriangleExtrusionEvidence.Init(FTriangleExtrusionEvidence(), Triangles.Num());
	ExtrusionSurfaceRegions.Reset();
	ExtrusionRegionTriangleIndices.Reset();
	TriangleExtrusionRegionIndices.Init(INDEX_NONE, Triangles.Num());
	constexpr double MinimumAspectRatio = 4.0;
	auto CanonicalAxis = [](FVector3d Axis)
	{
		Axis = Axis.GetSafeNormal();
		int32 DominantIndex = 0;
		if (FMath::Abs(Axis.Y) > FMath::Abs(Axis.X)) DominantIndex = 1;
		if (FMath::Abs(Axis.Z) > FMath::Abs(Axis[DominantIndex])) DominantIndex = 2;
		if (Axis[DominantIndex] < 0.0) Axis *= -1.0;
		return Axis;
	};
	for (int32 TriangleIndex = 0; TriangleIndex < Triangles.Num(); ++TriangleIndex)
	{
		const int32 PatchIndex = TrianglePatchIndices[TriangleIndex];
		const int32 PlanarGroupIndex = PatchPlanarGroupIndices.IsValidIndex(PatchIndex)
			? PatchPlanarGroupIndices[PatchIndex] : INDEX_NONE;
		if (PlanarSurfaceGroups.IsValidIndex(PlanarGroupIndex) &&
			PlanarSurfaceGroups[PlanarGroupIndex].bArchitecturalConstraint)
		{
			continue;
		}
		const FTriangleSurface& Triangle = Triangles[TriangleIndex];
		FVector3d Edges[3] = {
			Triangle.Vertices[1] - Triangle.Vertices[0],
			Triangle.Vertices[2] - Triangle.Vertices[1],
			Triangle.Vertices[0] - Triangle.Vertices[2] };
		int32 Order[3] = { 0, 1, 2 };
		for (int32 A = 0; A < 2; ++A)
		{
			for (int32 B = A + 1; B < 3; ++B)
			{
				if (Edges[Order[B]].SquaredLength() > Edges[Order[A]].SquaredLength())
				{
					Swap(Order[A], Order[B]);
				}
			}
		}
		const double SecondLength = Edges[Order[1]].Length();
		const double ShortLength = Edges[Order[2]].Length();
		if (ShortLength <= 1.0e-9) continue;
		FTriangleExtrusionEvidence& Evidence =
			TriangleExtrusionEvidence[TriangleIndex];
		Evidence.AspectRatio = SecondLength / ShortLength;
		Evidence.AxialLength = 0.5 *
			(Edges[Order[0]].Length() + SecondLength);
		Evidence.CrossSectionLength = ShortLength;
		FVector3d FirstDirection = Edges[Order[0]].GetSafeNormal();
		FVector3d SecondDirection = Edges[Order[1]].GetSafeNormal();
		if (FVector3d::DotProduct(FirstDirection, SecondDirection) < 0.0)
		{
			SecondDirection *= -1.0;
		}
		Evidence.Axis = CanonicalAxis(FirstDirection + SecondDirection);
		Evidence.bCandidate = Evidence.AspectRatio >= MinimumAspectRatio &&
			!Evidence.Axis.IsNearlyZero();
	}

	TArray<bool> Assigned;
	Assigned.Init(false, Triangles.Num());
	TArray<int32> Pending;
	TArray<int32> Members;
	const double MinimumAxisDot = FMath::Cos(FMath::DegreesToRadians(2.0));
	for (int32 SeedIndex = 0; SeedIndex < Triangles.Num(); ++SeedIndex)
	{
		const FTriangleExtrusionEvidence& SeedEvidence =
			TriangleExtrusionEvidence[SeedIndex];
		if (Assigned[SeedIndex] || !SeedEvidence.bCandidate) continue;
		const FTriangleSurface& SeedTriangle = Triangles[SeedIndex];
		Pending.Reset();
		Members.Reset();
		Pending.Add(SeedIndex);
		Assigned[SeedIndex] = true;
		while (!Pending.IsEmpty())
		{
			const int32 TriangleIndex = Pending.Pop(EAllowShrinking::No);
			Members.Add(TriangleIndex);
			for (int32 LocalEdge = 0; LocalEdge < 3; ++LocalEdge)
			{
				const int32 NeighborIndex =
					TriangleNeighborIndices[TriangleIndex][LocalEdge];
				if (NeighborIndex == INDEX_NONE || Assigned[NeighborIndex]) continue;
				const FTriangleSurface& Neighbor = Triangles[NeighborIndex];
				const FTriangleExtrusionEvidence& NeighborEvidence =
					TriangleExtrusionEvidence[NeighborIndex];
				const FTriangleMeshEdge& Edge =
					MeshEdges[TriangleEdgeIndices[TriangleIndex][LocalEdge]];
				if (!NeighborEvidence.bCandidate ||
					Edge.Continuity != EEdgeContinuity::Smooth ||
					Neighbor.SourceId != SeedTriangle.SourceId ||
					Neighbor.SurfaceId != SeedTriangle.SurfaceId ||
					Neighbor.MaterialId != SeedTriangle.MaterialId ||
					FMath::Abs(FVector3d::DotProduct(
						SeedEvidence.Axis, NeighborEvidence.Axis)) < MinimumAxisDot)
				{
					continue;
				}
				Assigned[NeighborIndex] = true;
				Pending.Add(NeighborIndex);
			}
		}
		Algo::Sort(Members);
		FExtrusionSurfaceRegion& Region =
			ExtrusionSurfaceRegions.AddDefaulted_GetRef();
		Region.RegionId = CombineStableIds(
			SeedTriangle.SourceId, SeedTriangle.PrimitiveId);
		Region.SourceId = SeedTriangle.SourceId;
		Region.SurfaceId = SeedTriangle.SurfaceId;
		Region.MaterialId = SeedTriangle.MaterialId;
		Region.FirstTriangleIndex = ExtrusionRegionTriangleIndices.Num();
		Region.TriangleCount = Members.Num();
		FVector3d AxisSum = FVector3d::ZeroVector;
		for (const int32 TriangleIndex : Members)
		{
			TriangleExtrusionRegionIndices[TriangleIndex] =
				ExtrusionSurfaceRegions.Num() - 1;
			ExtrusionRegionTriangleIndices.Add(TriangleIndex);
			const FTriangleSurface& Triangle = Triangles[TriangleIndex];
			const FTriangleExtrusionEvidence& Evidence =
				TriangleExtrusionEvidence[TriangleIndex];
			const double Area = 0.5 * FVector3d::CrossProduct(
				Triangle.Vertices[1] - Triangle.Vertices[0],
				Triangle.Vertices[2] - Triangle.Vertices[0]).Length();
			Region.Area += Area;
			Region.MinimumAspectRatio = FMath::Min(
				Region.MinimumAspectRatio, Evidence.AspectRatio);
			Region.MaximumAxisDeviationDegrees = FMath::Max(
				Region.MaximumAxisDeviationDegrees,
				VectorAngleDegrees(SeedEvidence.Axis, Evidence.Axis));
			Region.Bounds += Triangle.Bounds;
			FVector3d Axis = Evidence.Axis;
			if (FVector3d::DotProduct(SeedEvidence.Axis, Axis) < 0.0) Axis *= -1.0;
			AxisSum += Area * Axis;
		}
		Region.Axis = CanonicalAxis(AxisSum);
		if (FMath::Abs(Region.Axis.Z) <= 0.01)
		{
			Region.SectionAxisU = FVector3d::CrossProduct(
				FVector3d::UpVector, Region.Axis).GetSafeNormal();
			Region.SectionAxisV = FVector3d::CrossProduct(
				Region.Axis, Region.SectionAxisU).GetSafeNormal();
			TSet<int32> UniqueVertexSet;
			for (const int32 TriangleIndex : Members)
			{
				for (int32 Corner = 0; Corner < 3; ++Corner)
				{
					UniqueVertexSet.Add(TriangleVertexIndices[TriangleIndex][Corner]);
				}
			}
			TArray<int32> UniqueVertices = UniqueVertexSet.Array();
			Algo::Sort(UniqueVertices);
			FVector2d SectionMinimum(
				TNumericLimits<double>::Max(), TNumericLimits<double>::Max());
			FVector2d SectionMaximum(
				TNumericLimits<double>::Lowest(), TNumericLimits<double>::Lowest());
			TArray<FVector2d, TInlineAllocator<256>> SectionPoints;
			SectionPoints.Reserve(UniqueVertices.Num());
			for (const int32 VertexIndex : UniqueVertices)
			{
				const FVector3d& Position = MeshVertices[VertexIndex];
				const FVector2d Point(
					FVector3d::DotProduct(Position, Region.SectionAxisU),
					FVector3d::DotProduct(Position, Region.SectionAxisV));
				SectionPoints.Add(Point);
				SectionMinimum.X = FMath::Min(SectionMinimum.X, Point.X);
				SectionMinimum.Y = FMath::Min(SectionMinimum.Y, Point.Y);
				SectionMaximum.X = FMath::Max(SectionMaximum.X, Point.X);
				SectionMaximum.Y = FMath::Max(SectionMaximum.Y, Point.Y);
			}
			const double RadiusU = SectionMaximum.X - SectionMinimum.X;
			const double RadiusV = SectionMaximum.Y - SectionMinimum.Y;
			if (SectionPoints.Num() >= 3 && RadiusU > 1.0e-6 && RadiusV > 1.0e-6)
			{
				double BestSquaredResidual = TNumericLimits<double>::Max();
				double BestMaximumResidual = TNumericLimits<double>::Max();
				int32 BestCorner = INDEX_NONE;
				for (int32 Corner = 0; Corner < 4; ++Corner)
				{
					const FVector2d Center(
						(Corner & 1) != 0 ? SectionMaximum.X : SectionMinimum.X,
						(Corner & 2) != 0 ? SectionMaximum.Y : SectionMinimum.Y);
					double SquaredResidual = 0.0;
					double MaximumResidual = 0.0;
					for (const FVector2d& Point : SectionPoints)
					{
						const double DeltaU = Point.X - Center.X;
						const double DeltaV = Point.Y - Center.Y;
						const double Implicit =
							FMath::Square(DeltaU / RadiusU) +
							FMath::Square(DeltaV / RadiusV) - 1.0;
						const double GradientMagnitude = 2.0 * FMath::Sqrt(
							FMath::Square(DeltaU / FMath::Square(RadiusU)) +
							FMath::Square(DeltaV / FMath::Square(RadiusV)));
						const double Residual = GradientMagnitude > 1.0e-12
							? FMath::Abs(Implicit) / GradientMagnitude
							: TNumericLimits<double>::Max();
						SquaredResidual += FMath::Square(Residual);
						MaximumResidual = FMath::Max(MaximumResidual, Residual);
					}
					if (SquaredResidual < BestSquaredResidual)
					{
						BestSquaredResidual = SquaredResidual;
						BestMaximumResidual = MaximumResidual;
						BestCorner = Corner;
					}
				}
				if (BestCorner != INDEX_NONE)
				{
					Region.bQuarterEllipseFitValid = true;
					Region.bEllipseCenterAtMaximumU = (BestCorner & 1) != 0;
					Region.bEllipseCenterAtMaximumV = (BestCorner & 2) != 0;
					Region.EllipseCenterCoordinates = FVector2d(
						Region.bEllipseCenterAtMaximumU
							? SectionMaximum.X : SectionMinimum.X,
						Region.bEllipseCenterAtMaximumV
							? SectionMaximum.Y : SectionMinimum.Y);
					Region.EllipseRadiusU = RadiusU;
					Region.EllipseRadiusV = RadiusV;
					Region.EllipseRootMeanSquareResidualCm = FMath::Sqrt(
						BestSquaredResidual / static_cast<double>(SectionPoints.Num()));
					Region.EllipseMaximumResidualCm = BestMaximumResidual;
					const double MinimumRadius = FMath::Min(RadiusU, RadiusV);
					Region.bQuarterEllipsePlausible =
						Region.EllipseRootMeanSquareResidualCm <=
							0.01 * MinimumRadius &&
						Region.EllipseMaximumResidualCm <= 0.02 * MinimumRadius;
				}
			}
		}
	}
}

void FAnalyticWorldData::BuildExtrusionRegionMirrorMatches()
{
	ExtrusionRegionMirrorMatches.Reset();
	if (ExtrusionSurfaceRegions.IsEmpty()) return;
	auto ReflectedBounds = [](const FBox3d& Bounds,
		const EPlanarSymmetryAxis Axis)
	{
		FBox3d Result = Bounds;
		const int32 AxisIndex = Axis == EPlanarSymmetryAxis::X ? 0 : 1;
		Result.Min[AxisIndex] = -Bounds.Max[AxisIndex];
		Result.Max[AxisIndex] = -Bounds.Min[AxisIndex];
		return Result;
	};
	auto BoundsResidual = [](const FBox3d& A, const FBox3d& B)
	{
		double Result = 0.0;
		for (int32 AxisIndex = 0; AxisIndex < 3; ++AxisIndex)
		{
			Result = FMath::Max(Result,
				FMath::Abs(A.Min[AxisIndex] - B.Min[AxisIndex]));
			Result = FMath::Max(Result,
				FMath::Abs(A.Max[AxisIndex] - B.Max[AxisIndex]));
		}
		return Result;
	};
	auto RelativeResidual = [](const double A, const double B)
	{
		return FMath::Abs(A - B) / FMath::Max(FMath::Max(A, B), 1.0e-9);
	};

	for (const EPlanarSymmetryAxis SymmetryAxis : {
		EPlanarSymmetryAxis::X, EPlanarSymmetryAxis::Y })
	{
		const int32 FirstMatch = ExtrusionRegionMirrorMatches.Num();
		const int32 ReflectionAxisIndex =
			SymmetryAxis == EPlanarSymmetryAxis::X ? 0 : 1;
		for (int32 SourceIndex = 0;
			SourceIndex < ExtrusionSurfaceRegions.Num(); ++SourceIndex)
		{
			const FExtrusionSurfaceRegion& Source =
				ExtrusionSurfaceRegions[SourceIndex];
			FExtrusionRegionMirrorMatch Best;
			Best.Axis = SymmetryAxis;
			Best.SourceRegionIndex = SourceIndex;
			Best.Score = TNumericLimits<double>::Max();
			if (!Source.bQuarterEllipseFitValid)
			{
				ExtrusionRegionMirrorMatches.Add(Best);
				continue;
			}
			FVector3d MirroredAxis = Source.Axis;
			MirroredAxis[ReflectionAxisIndex] *= -1.0;
			FVector3d MirroredCenter =
				Source.EllipseCenterCoordinates.X * Source.SectionAxisU +
				Source.EllipseCenterCoordinates.Y * Source.SectionAxisV;
			MirroredCenter[ReflectionAxisIndex] *= -1.0;
			const FBox3d MirroredBounds =
				ReflectedBounds(Source.Bounds, SymmetryAxis);
			for (int32 TargetIndex = 0;
				TargetIndex < ExtrusionSurfaceRegions.Num(); ++TargetIndex)
			{
				const FExtrusionSurfaceRegion& Target =
					ExtrusionSurfaceRegions[TargetIndex];
				if (!Target.bQuarterEllipseFitValid ||
					Target.SourceId != Source.SourceId ||
					Target.SurfaceId != Source.SurfaceId ||
					Target.MaterialId != Source.MaterialId)
				{
					continue;
				}
				FExtrusionRegionMirrorMatch Candidate;
				Candidate.Axis = SymmetryAxis;
				Candidate.SourceRegionIndex = SourceIndex;
				Candidate.TargetRegionIndex = TargetIndex;
				Candidate.AxisAngleResidualDegrees = VectorAngleDegrees(
					MirroredAxis, Target.Axis);
				Candidate.AxisAngleResidualDegrees = FMath::Min(
					Candidate.AxisAngleResidualDegrees,
					180.0 - Candidate.AxisAngleResidualDegrees);
				const FVector3d TargetCenter =
					Target.EllipseCenterCoordinates.X * Target.SectionAxisU +
					Target.EllipseCenterCoordinates.Y * Target.SectionAxisV;
				const FVector3d CenterDelta = MirroredCenter - TargetCenter;
				Candidate.CenterLineResidualCm =
					(CenterDelta - FVector3d::DotProduct(
						CenterDelta, Target.Axis) * Target.Axis).Length();
				Candidate.RelativeRadiusUResidual = RelativeResidual(
					Source.EllipseRadiusU, Target.EllipseRadiusU);
				Candidate.RelativeRadiusVResidual = RelativeResidual(
					Source.EllipseRadiusV, Target.EllipseRadiusV);
				Candidate.BoundsResidualCm = BoundsResidual(
					MirroredBounds, Target.Bounds);
				Candidate.RelativeAreaResidual = RelativeResidual(
					Source.Area, Target.Area);
				Candidate.Score = 10.0 * Candidate.AxisAngleResidualDegrees +
					Candidate.CenterLineResidualCm +
					100.0 * (Candidate.RelativeRadiusUResidual +
						Candidate.RelativeRadiusVResidual) +
					Candidate.BoundsResidualCm +
					100.0 * Candidate.RelativeAreaResidual;
				if (Candidate.Score < Best.Score ||
					(Candidate.Score == Best.Score &&
						(Best.TargetRegionIndex == INDEX_NONE || Target.RegionId <
							ExtrusionSurfaceRegions[Best.TargetRegionIndex].RegionId)))
				{
					Best = Candidate;
				}
			}
			ExtrusionRegionMirrorMatches.Add(Best);
		}
		for (int32 MatchIndex = FirstMatch;
			MatchIndex < ExtrusionRegionMirrorMatches.Num(); ++MatchIndex)
		{
			FExtrusionRegionMirrorMatch& Match =
				ExtrusionRegionMirrorMatches[MatchIndex];
			if (Match.TargetRegionIndex == INDEX_NONE) continue;
			const int32 ReverseIndex = FirstMatch + Match.TargetRegionIndex;
			Match.bReciprocal =
				ExtrusionRegionMirrorMatches.IsValidIndex(ReverseIndex) &&
				ExtrusionRegionMirrorMatches[ReverseIndex].TargetRegionIndex ==
					Match.SourceRegionIndex;
			Match.bSelfMirror = Match.TargetRegionIndex == Match.SourceRegionIndex;
			const FExtrusionSurfaceRegion& Source =
				ExtrusionSurfaceRegions[Match.SourceRegionIndex];
			const FExtrusionSurfaceRegion& Target =
				ExtrusionSurfaceRegions[Match.TargetRegionIndex];
			Match.bPlausible = Source.bQuarterEllipsePlausible &&
				Target.bQuarterEllipsePlausible && Match.bReciprocal &&
				Match.AxisAngleResidualDegrees <= 1.0 &&
				Match.CenterLineResidualCm <= 25.0 &&
				Match.RelativeRadiusUResidual <= 0.05 &&
				Match.RelativeRadiusVResidual <= 0.05 &&
				Match.BoundsResidualCm <= 25.0 &&
				Match.RelativeAreaResidual <= 0.10;
		}
	}
}

void FAnalyticWorldData::BuildQuarterEllipseBoundaryMatches()
{
	QuarterEllipseBoundaryMatches.Reset();
	for (int32 RegionIndex = 0;
		RegionIndex < ExtrusionSurfaceRegions.Num(); ++RegionIndex)
	{
		const FExtrusionSurfaceRegion& Region =
			ExtrusionSurfaceRegions[RegionIndex];
		if (!Region.bQuarterEllipsePlausible) continue;
		const FVector3d Center =
			Region.EllipseCenterCoordinates.X * Region.SectionAxisU +
			Region.EllipseCenterCoordinates.Y * Region.SectionAxisV;
		for (const EQuarterEllipseEndpoint Endpoint : {
			EQuarterEllipseEndpoint::U, EQuarterEllipseEndpoint::V })
		{
			const bool bEndpointU = Endpoint == EQuarterEllipseEndpoint::U;
			const FVector3d EndpointAxis = bEndpointU
				? Region.SectionAxisU : Region.SectionAxisV;
			const bool bCenterAtMaximum = bEndpointU
				? Region.bEllipseCenterAtMaximumU
				: Region.bEllipseCenterAtMaximumV;
			const double Radius = bEndpointU
				? Region.EllipseRadiusU : Region.EllipseRadiusV;
			const FVector3d EndpointPosition = Center +
				(bCenterAtMaximum ? -Radius : Radius) * EndpointAxis;
			FQuarterEllipseBoundaryMatch Best;
			Best.ExtrusionRegionIndex = RegionIndex;
			Best.Endpoint = Endpoint;
			Best.EndpointPosition = EndpointPosition;
			Best.Score = TNumericLimits<double>::Max();
			for (int32 GroupIndex = 0;
				GroupIndex < PlanarSurfaceGroups.Num(); ++GroupIndex)
			{
				const FPlanarSurfaceGroup& Group = PlanarSurfaceGroups[GroupIndex];
				if (!Group.bArchitecturalConstraint ||
					Group.SourceId != Region.SourceId ||
					Group.SurfaceId != Region.SurfaceId)
				{
					continue;
				}
				FQuarterEllipseBoundaryMatch Candidate = Best;
				Candidate.PlanarGroupIndex = GroupIndex;
				Candidate.PositionResidualCm = FMath::Abs(
					FVector3d::DotProduct(Group.Normal, EndpointPosition) -
					Group.PlaneOffset);
				Candidate.BoundsResidualCm = FMath::Sqrt(
					SquaredDistanceToBox(EndpointPosition, Group.Bounds));
				Candidate.NormalAngleResidualDegrees = VectorAngleDegrees(
					Group.Normal, EndpointAxis);
				Candidate.NormalAngleResidualDegrees = FMath::Min(
					Candidate.NormalAngleResidualDegrees,
					180.0 - Candidate.NormalAngleResidualDegrees);
				Candidate.Score = Candidate.PositionResidualCm +
					Candidate.BoundsResidualCm +
					100.0 * Candidate.NormalAngleResidualDegrees;
				if (Candidate.Score < Best.Score ||
					(Candidate.Score == Best.Score &&
						(Best.PlanarGroupIndex == INDEX_NONE || Group.GroupId <
							PlanarSurfaceGroups[Best.PlanarGroupIndex].GroupId)))
				{
					Best = Candidate;
				}
			}
			if (Best.PlanarGroupIndex != INDEX_NONE)
			{
				Best.bC0Plausible = Best.PositionResidualCm <= 1.0 &&
					Best.BoundsResidualCm <= 25.0;
				Best.bG1Plausible = Best.bC0Plausible &&
					Best.NormalAngleResidualDegrees <= 0.1;
			}
			QuarterEllipseBoundaryMatches.Add(Best);
		}
	}
}

void FAnalyticWorldData::BuildC2TransitionSectionFits()
{
	C2TransitionSectionFits.Reset();
	for (int32 RegionIndex = 0;
		RegionIndex < ExtrusionSurfaceRegions.Num(); ++RegionIndex)
	{
		const FExtrusionSurfaceRegion& Region =
			ExtrusionSurfaceRegions[RegionIndex];
		if (!Region.bQuarterEllipsePlausible) continue;
		int32 BoundaryIndices[2] = { INDEX_NONE, INDEX_NONE };
		for (int32 MatchIndex = 0;
			MatchIndex < QuarterEllipseBoundaryMatches.Num(); ++MatchIndex)
		{
			const FQuarterEllipseBoundaryMatch& Match =
				QuarterEllipseBoundaryMatches[MatchIndex];
			if (Match.ExtrusionRegionIndex != RegionIndex) continue;
			BoundaryIndices[Match.Endpoint == EQuarterEllipseEndpoint::U ? 0 : 1] =
				MatchIndex;
		}
		FC2TransitionSectionFit& Fit =
			C2TransitionSectionFits.AddDefaulted_GetRef();
		Fit.ExtrusionRegionIndex = RegionIndex;
		Fit.BoundaryMatchUIndex = BoundaryIndices[0];
		Fit.BoundaryMatchVIndex = BoundaryIndices[1];
		if (!QuarterEllipseBoundaryMatches.IsValidIndex(BoundaryIndices[0]) ||
			!QuarterEllipseBoundaryMatches.IsValidIndex(BoundaryIndices[1]))
		{
			continue;
		}
		const FQuarterEllipseBoundaryMatch& MatchU =
			QuarterEllipseBoundaryMatches[BoundaryIndices[0]];
		const FQuarterEllipseBoundaryMatch& MatchV =
			QuarterEllipseBoundaryMatches[BoundaryIndices[1]];
		if (!PlanarSurfaceGroups.IsValidIndex(MatchU.PlanarGroupIndex) ||
			!PlanarSurfaceGroups.IsValidIndex(MatchV.PlanarGroupIndex))
		{
			continue;
		}
		const FPlanarSurfaceGroup& PlaneU =
			PlanarSurfaceGroups[MatchU.PlanarGroupIndex];
		const FPlanarSurfaceGroup& PlaneV =
			PlanarSurfaceGroups[MatchV.PlanarGroupIndex];
		const double PlaneUDenominator = FVector3d::DotProduct(
			PlaneU.Normal, Region.SectionAxisU);
		const double PlaneVDenominator = FVector3d::DotProduct(
			PlaneV.Normal, Region.SectionAxisV);
		if (FMath::Abs(PlaneUDenominator) <= 1.0e-6 ||
			FMath::Abs(PlaneVDenominator) <= 1.0e-6)
		{
			continue;
		}
		const FVector3d SnappedEndpointU = MatchU.EndpointPosition +
			((PlaneU.PlaneOffset - FVector3d::DotProduct(
				PlaneU.Normal, MatchU.EndpointPosition)) / PlaneUDenominator) *
			Region.SectionAxisU;
		const FVector3d SnappedEndpointV = MatchV.EndpointPosition +
			((PlaneV.PlaneOffset - FVector3d::DotProduct(
				PlaneV.Normal, MatchV.EndpointPosition)) / PlaneVDenominator) *
			Region.SectionAxisV;
		Fit.RadiusU = Region.EllipseRadiusU;
		Fit.RadiusV = Region.EllipseRadiusV;
		const double SignU = Region.bEllipseCenterAtMaximumU ? -1.0 : 1.0;
		const double SignV = Region.bEllipseCenterAtMaximumV ? -1.0 : 1.0;
		Fit.SignU = SignU;
		Fit.SignV = SignV;
		const double EndpointUCoordinate = FVector3d::DotProduct(
			SnappedEndpointU, Region.SectionAxisU);
		const double EndpointVCoordinate = FVector3d::DotProduct(
			SnappedEndpointV, Region.SectionAxisV);
		auto CenterFor = [&](const double RadiusU, const double RadiusV)
		{
			return FVector2d(EndpointUCoordinate - SignU * RadiusU,
				EndpointVCoordinate - SignV * RadiusV);
		};
		Fit.CenterCoordinates = CenterFor(Fit.RadiusU, Fit.RadiusV);
		Fit.MaximumEndpointNormalAdjustmentDegrees = FMath::Max(
			MatchU.NormalAngleResidualDegrees, MatchV.NormalAngleResidualDegrees);
		Fit.bBoundaryEvidenceUsable =
			MatchU.PositionResidualCm <= 5.0 && MatchU.BoundsResidualCm <= 25.0 &&
			MatchV.PositionResidualCm <= 5.0 && MatchV.BoundsResidualCm <= 25.0 &&
			Fit.MaximumEndpointNormalAdjustmentDegrees <= 0.1;
		Fit.bZeroEndpointCurvatureByConstruction = true;

		TSet<int32> UniqueVertexSet;
		for (int32 MemberIndex = Region.FirstTriangleIndex;
			MemberIndex < Region.FirstTriangleIndex + Region.TriangleCount;
			++MemberIndex)
		{
			const int32 TriangleIndex = ExtrusionRegionTriangleIndices[MemberIndex];
			for (int32 Corner = 0; Corner < 3; ++Corner)
			{
				UniqueVertexSet.Add(TriangleVertexIndices[TriangleIndex][Corner]);
			}
		}
		TArray<FVector2d, TInlineAllocator<256>> SectionPoints;
		TArray<int32> UniqueVertices = UniqueVertexSet.Array();
		Algo::Sort(UniqueVertices);
		SectionPoints.Reserve(UniqueVertices.Num());
		for (const int32 VertexIndex : UniqueVertices)
		{
			const FVector3d& Position = MeshVertices[VertexIndex];
			SectionPoints.Emplace(
				FVector3d::DotProduct(Position, Region.SectionAxisU),
				FVector3d::DotProduct(Position, Region.SectionAxisV));
		}
		auto TotalSquaredResidual = [&](const double RadiusU,
			const double RadiusV, const double FlatteningFraction)
		{
			const FVector2d Center = CenterFor(RadiusU, RadiusV);
			double Result = 0.0;
			for (const FVector2d& Point : SectionPoints)
			{
				Result += SquaredDistanceToQuinticTransition(
					Point, Center, RadiusU, RadiusV,
					SignU, SignV, FlatteningFraction);
			}
			return Result;
		};
		if (!SectionPoints.IsEmpty())
		{
			constexpr double MinimumFlatteningFraction = 0.01;
			constexpr double MaximumFlatteningFraction = 0.49;
			constexpr double GridStep = 0.01;
			double BestSquaredResidual = TNumericLimits<double>::Max();
			for (double Fraction = MinimumFlatteningFraction;
				Fraction <= MaximumFlatteningFraction + 1.0e-12;
				Fraction += GridStep)
			{
				const double SquaredResidual = TotalSquaredResidual(
					Fit.RadiusU, Fit.RadiusV, Fraction);
				if (SquaredResidual < BestSquaredResidual)
				{
					BestSquaredResidual = SquaredResidual;
					Fit.FlatteningFraction = Fraction;
				}
			}
			double Lower = FMath::Max(MinimumFlatteningFraction,
				Fit.FlatteningFraction - GridStep);
			double Upper = FMath::Min(MaximumFlatteningFraction,
				Fit.FlatteningFraction + GridStep);
			for (int32 Iteration = 0; Iteration < 20; ++Iteration)
			{
				const double Left = (2.0 * Lower + Upper) / 3.0;
				const double Right = (Lower + 2.0 * Upper) / 3.0;
				if (TotalSquaredResidual(Fit.RadiusU, Fit.RadiusV, Left) <=
					TotalSquaredResidual(Fit.RadiusU, Fit.RadiusV, Right))
				{
					Upper = Right;
				}
				else
				{
					Lower = Left;
				}
			}
			Fit.FlatteningFraction = 0.5 * (Lower + Upper);
			double Steps[3] = {
				0.10 * Fit.RadiusU, 0.10 * Fit.RadiusV, 0.03 };
			BestSquaredResidual = TotalSquaredResidual(Fit.RadiusU,
				Fit.RadiusV, Fit.FlatteningFraction);
			for (int32 Iteration = 0; Iteration < 14; ++Iteration)
			{
				double BestRadiusU = Fit.RadiusU;
				double BestRadiusV = Fit.RadiusV;
				double BestFlattening = Fit.FlatteningFraction;
				for (int32 DirectionU = -1; DirectionU <= 1; ++DirectionU)
				{
					for (int32 DirectionV = -1; DirectionV <= 1; ++DirectionV)
					{
						for (int32 DirectionF = -1; DirectionF <= 1; ++DirectionF)
						{
							if (DirectionU == 0 && DirectionV == 0 && DirectionF == 0)
							{
								continue;
							}
							const double CandidateRadiusU = FMath::Max(1.0,
								Fit.RadiusU + static_cast<double>(DirectionU) * Steps[0]);
							const double CandidateRadiusV = FMath::Max(1.0,
								Fit.RadiusV + static_cast<double>(DirectionV) * Steps[1]);
							const double CandidateFlattening = FMath::Clamp(
								Fit.FlatteningFraction +
									static_cast<double>(DirectionF) * Steps[2],
								MinimumFlatteningFraction,
								MaximumFlatteningFraction);
							const double CandidateResidual = TotalSquaredResidual(
								CandidateRadiusU, CandidateRadiusV,
								CandidateFlattening);
							if (CandidateResidual < BestSquaredResidual)
							{
								BestSquaredResidual = CandidateResidual;
								BestRadiusU = CandidateRadiusU;
								BestRadiusV = CandidateRadiusV;
								BestFlattening = CandidateFlattening;
							}
						}
					}
				}
				Fit.RadiusU = BestRadiusU;
				Fit.RadiusV = BestRadiusV;
				Fit.FlatteningFraction = BestFlattening;
				for (double& Step : Steps) Step *= 0.5;
			}
			Fit.CenterCoordinates = CenterFor(Fit.RadiusU, Fit.RadiusV);
			Fit.RootMeanSquareResidualCm = FMath::Sqrt(
				BestSquaredResidual / static_cast<double>(SectionPoints.Num()));
			for (const FVector2d& Point : SectionPoints)
			{
				Fit.MaximumResidualCm = FMath::Max(Fit.MaximumResidualCm,
					FMath::Sqrt(SquaredDistanceToQuinticTransition(
						Point, Fit.CenterCoordinates, Fit.RadiusU, Fit.RadiusV,
						SignU, SignV, Fit.FlatteningFraction)));
			}
		}
		const FVector3d ProfileEndpointU =
			(Fit.CenterCoordinates.X + SignU * Fit.RadiusU) *
				Region.SectionAxisU +
			Fit.CenterCoordinates.Y * Region.SectionAxisV;
		const FVector3d ProfileEndpointV =
			Fit.CenterCoordinates.X * Region.SectionAxisU +
			(Fit.CenterCoordinates.Y + SignV * Fit.RadiusV) *
				Region.SectionAxisV;
		Fit.MaximumEndpointPlaneResidualCm = FMath::Max(
			FMath::Abs(FVector3d::DotProduct(PlaneU.Normal, ProfileEndpointU) -
				PlaneU.PlaneOffset),
			FMath::Abs(FVector3d::DotProduct(PlaneV.Normal, ProfileEndpointV) -
				PlaneV.PlaneOffset));
	}
}

void FAnalyticWorldData::BuildSharedC2TransitionPairFits()
{
	SharedC2TransitionPairFits.Reset();
	if (C2TransitionSectionFits.IsEmpty()) return;
	TArray<int32> RegionFitIndices;
	RegionFitIndices.Init(INDEX_NONE, ExtrusionSurfaceRegions.Num());
	for (int32 FitIndex = 0; FitIndex < C2TransitionSectionFits.Num(); ++FitIndex)
	{
		const int32 RegionIndex =
			C2TransitionSectionFits[FitIndex].ExtrusionRegionIndex;
		if (RegionFitIndices.IsValidIndex(RegionIndex))
		{
			RegionFitIndices[RegionIndex] = FitIndex;
		}
	}
	struct FSideData
	{
		const FExtrusionSurfaceRegion* Region = nullptr;
		const FC2TransitionSectionFit* Fit = nullptr;
		TArray<FVector2d> Points;
		double EndpointUCoordinate = 0.0;
		double EndpointVCoordinate = 0.0;
	};
	auto BuildSideData = [&](const int32 RegionIndex)
	{
		FSideData Result;
		if (!RegionFitIndices.IsValidIndex(RegionIndex) ||
			!C2TransitionSectionFits.IsValidIndex(RegionFitIndices[RegionIndex]))
		{
			return Result;
		}
		Result.Region = &ExtrusionSurfaceRegions[RegionIndex];
		Result.Fit = &C2TransitionSectionFits[RegionFitIndices[RegionIndex]];
		Result.EndpointUCoordinate = Result.Fit->CenterCoordinates.X +
			Result.Fit->SignU * Result.Fit->RadiusU;
		Result.EndpointVCoordinate = Result.Fit->CenterCoordinates.Y +
			Result.Fit->SignV * Result.Fit->RadiusV;
		TSet<int32> UniqueVertexSet;
		for (int32 MemberIndex = Result.Region->FirstTriangleIndex;
			MemberIndex < Result.Region->FirstTriangleIndex +
				Result.Region->TriangleCount; ++MemberIndex)
		{
			const int32 TriangleIndex = ExtrusionRegionTriangleIndices[MemberIndex];
			for (int32 Corner = 0; Corner < 3; ++Corner)
			{
				UniqueVertexSet.Add(TriangleVertexIndices[TriangleIndex][Corner]);
			}
		}
		TArray<int32> UniqueVertices = UniqueVertexSet.Array();
		Algo::Sort(UniqueVertices);
		Result.Points.Reserve(UniqueVertices.Num());
		for (const int32 VertexIndex : UniqueVertices)
		{
			const FVector3d& Position = MeshVertices[VertexIndex];
			Result.Points.Emplace(
				FVector3d::DotProduct(Position, Result.Region->SectionAxisU),
				FVector3d::DotProduct(Position, Result.Region->SectionAxisV));
		}
		return Result;
	};
	auto MeasureSide = [](const FSideData& Side, const double RadiusU,
		const double RadiusV, const double FlatteningFraction,
		double* OutMaximumResidual)
	{
		if (!Side.Fit || Side.Points.IsEmpty())
		{
			if (OutMaximumResidual) *OutMaximumResidual = 0.0;
			return TNumericLimits<double>::Max();
		}
		const FVector2d Center(
			Side.EndpointUCoordinate - Side.Fit->SignU * RadiusU,
			Side.EndpointVCoordinate - Side.Fit->SignV * RadiusV);
		double SquaredResidualSum = 0.0;
		double MaximumResidual = 0.0;
		for (const FVector2d& Point : Side.Points)
		{
			const double SquaredResidual = SquaredDistanceToQuinticTransition(
				Point, Center, RadiusU, RadiusV, Side.Fit->SignU,
				Side.Fit->SignV, FlatteningFraction);
			SquaredResidualSum += SquaredResidual;
			MaximumResidual = FMath::Max(
				MaximumResidual, FMath::Sqrt(SquaredResidual));
		}
		if (OutMaximumResidual) *OutMaximumResidual = MaximumResidual;
		return SquaredResidualSum / static_cast<double>(Side.Points.Num());
	};

	const int32 RegionCount = ExtrusionSurfaceRegions.Num();
	for (const EPlanarSymmetryAxis Axis : {
		EPlanarSymmetryAxis::X, EPlanarSymmetryAxis::Y })
	{
		const int32 MatchOffset = Axis == EPlanarSymmetryAxis::X ? 0 : RegionCount;
		for (int32 RegionAIndex = 0; RegionAIndex < RegionCount; ++RegionAIndex)
		{
			if (!ExtrusionRegionMirrorMatches.IsValidIndex(
				MatchOffset + RegionAIndex))
			{
				continue;
			}
			const FExtrusionRegionMirrorMatch& MirrorMatch =
				ExtrusionRegionMirrorMatches[MatchOffset + RegionAIndex];
			const int32 RegionBIndex = MirrorMatch.TargetRegionIndex;
			if (!MirrorMatch.bPlausible || MirrorMatch.bSelfMirror ||
				!ExtrusionSurfaceRegions.IsValidIndex(RegionBIndex) ||
				ExtrusionSurfaceRegions[RegionAIndex].RegionId >=
					ExtrusionSurfaceRegions[RegionBIndex].RegionId)
			{
				continue;
			}
			const FSideData SideA = BuildSideData(RegionAIndex);
			const FSideData SideB = BuildSideData(RegionBIndex);
			if (!SideA.Fit || !SideB.Fit) continue;
			FSharedC2TransitionPairFit& Pair =
				SharedC2TransitionPairFits.AddDefaulted_GetRef();
			Pair.SymmetryAxis = Axis;
			Pair.RegionAIndex = RegionAIndex;
			Pair.RegionBIndex = RegionBIndex;
			Pair.bBothBoundaryEvidenceUsable =
				SideA.Fit->bBoundaryEvidenceUsable &&
				SideB.Fit->bBoundaryEvidenceUsable;
			Pair.MaximumEndpointNormalAdjustmentDegrees = FMath::Max(
				SideA.Fit->MaximumEndpointNormalAdjustmentDegrees,
				SideB.Fit->MaximumEndpointNormalAdjustmentDegrees);
			Pair.SharedRadiusU = 0.5 *
				(SideA.Fit->RadiusU + SideB.Fit->RadiusU);
			Pair.SharedRadiusV = 0.5 *
				(SideA.Fit->RadiusV + SideB.Fit->RadiusV);
			Pair.SharedFlatteningFraction = 0.5 *
				(SideA.Fit->FlatteningFraction + SideB.Fit->FlatteningFraction);
			auto Objective = [&](const double RadiusU, const double RadiusV,
				const double FlatteningFraction)
			{
				return MeasureSide(SideA, RadiusU, RadiusV,
					FlatteningFraction, nullptr) +
					MeasureSide(SideB, RadiusU, RadiusV,
						FlatteningFraction, nullptr);
			};
			double Steps[3] = {
				FMath::Max(0.5 * FMath::Abs(
					SideA.Fit->RadiusU - SideB.Fit->RadiusU),
					0.10 * Pair.SharedRadiusU),
				FMath::Max(0.5 * FMath::Abs(
					SideA.Fit->RadiusV - SideB.Fit->RadiusV),
					0.10 * Pair.SharedRadiusV),
				FMath::Max(0.5 * FMath::Abs(
					SideA.Fit->FlatteningFraction -
					SideB.Fit->FlatteningFraction), 0.03) };
			double BestObjective = Objective(Pair.SharedRadiusU,
				Pair.SharedRadiusV, Pair.SharedFlatteningFraction);
			for (int32 Iteration = 0; Iteration < 14; ++Iteration)
			{
				double BestRadiusU = Pair.SharedRadiusU;
				double BestRadiusV = Pair.SharedRadiusV;
				double BestFlattening = Pair.SharedFlatteningFraction;
				for (int32 DirectionU = -1; DirectionU <= 1; ++DirectionU)
				{
					for (int32 DirectionV = -1; DirectionV <= 1; ++DirectionV)
					{
						for (int32 DirectionF = -1; DirectionF <= 1; ++DirectionF)
						{
							if (DirectionU == 0 && DirectionV == 0 && DirectionF == 0)
							{
								continue;
							}
							const double CandidateRadiusU = FMath::Max(1.0,
								Pair.SharedRadiusU +
									static_cast<double>(DirectionU) * Steps[0]);
							const double CandidateRadiusV = FMath::Max(1.0,
								Pair.SharedRadiusV +
									static_cast<double>(DirectionV) * Steps[1]);
							const double CandidateFlattening = FMath::Clamp(
								Pair.SharedFlatteningFraction +
									static_cast<double>(DirectionF) * Steps[2],
								0.01, 0.49);
							const double CandidateObjective = Objective(
								CandidateRadiusU, CandidateRadiusV,
								CandidateFlattening);
							if (CandidateObjective < BestObjective)
							{
								BestObjective = CandidateObjective;
								BestRadiusU = CandidateRadiusU;
								BestRadiusV = CandidateRadiusV;
								BestFlattening = CandidateFlattening;
							}
						}
					}
				}
				Pair.SharedRadiusU = BestRadiusU;
				Pair.SharedRadiusV = BestRadiusV;
				Pair.SharedFlatteningFraction = BestFlattening;
				for (double& Step : Steps) Step *= 0.5;
			}
			double MeanSquaredA = MeasureSide(SideA, Pair.SharedRadiusU,
				Pair.SharedRadiusV, Pair.SharedFlatteningFraction,
				&Pair.RegionAMaximumResidualCm);
			double MeanSquaredB = MeasureSide(SideB, Pair.SharedRadiusU,
				Pair.SharedRadiusV, Pair.SharedFlatteningFraction,
				&Pair.RegionBMaximumResidualCm);
			Pair.RegionARootMeanSquareResidualCm = FMath::Sqrt(MeanSquaredA);
			Pair.RegionBRootMeanSquareResidualCm = FMath::Sqrt(MeanSquaredB);
			Pair.BalancedRootMeanSquareResidualCm = FMath::Sqrt(
				0.5 * (MeanSquaredA + MeanSquaredB));
			const double MinimumRadius = FMath::Min(
				Pair.SharedRadiusU, Pair.SharedRadiusV);
			Pair.bPlausible = Pair.bBothBoundaryEvidenceUsable &&
				Pair.RegionARootMeanSquareResidualCm <= 0.01 * MinimumRadius &&
				Pair.RegionBRootMeanSquareResidualCm <= 0.01 * MinimumRadius &&
				Pair.RegionAMaximumResidualCm <= 0.02 * MinimumRadius &&
				Pair.RegionBMaximumResidualCm <= 0.02 * MinimumRadius;
		}
	}
	// The objective is mildly non-convex. Feed every deterministic shared-pair
	// solution back as an additional seed for the corresponding independent
	// diagnostic so a shared fit can never appear better merely because the
	// individual search settled in a different local basin.
	for (const FSharedC2TransitionPairFit& Pair : SharedC2TransitionPairFits)
	{
		if (!Pair.bBothBoundaryEvidenceUsable) continue;
		for (const int32 RegionIndex : { Pair.RegionAIndex, Pair.RegionBIndex })
		{
			if (!RegionFitIndices.IsValidIndex(RegionIndex) ||
				!C2TransitionSectionFits.IsValidIndex(RegionFitIndices[RegionIndex]))
			{
				continue;
			}
			const FSideData Side = BuildSideData(RegionIndex);
			double CandidateMaximumResidual = 0.0;
			const double CandidateMeanSquaredResidual = MeasureSide(Side,
				Pair.SharedRadiusU, Pair.SharedRadiusV,
				Pair.SharedFlatteningFraction, &CandidateMaximumResidual);
			FC2TransitionSectionFit& Individual =
				C2TransitionSectionFits[RegionFitIndices[RegionIndex]];
			if (CandidateMeanSquaredResidual >=
				FMath::Square(Individual.RootMeanSquareResidualCm))
			{
				continue;
			}
			Individual.RadiusU = Pair.SharedRadiusU;
			Individual.RadiusV = Pair.SharedRadiusV;
			Individual.FlatteningFraction = Pair.SharedFlatteningFraction;
			Individual.CenterCoordinates = FVector2d(
				Side.EndpointUCoordinate - Individual.SignU * Individual.RadiusU,
				Side.EndpointVCoordinate - Individual.SignV * Individual.RadiusV);
			Individual.RootMeanSquareResidualCm =
				FMath::Sqrt(CandidateMeanSquaredResidual);
			Individual.MaximumResidualCm = CandidateMaximumResidual;
		}
	}
}

void FAnalyticWorldData::BuildSharedC2FrameLedger()
{
	SharedC2FrameLedger.Reset();
	TArray<int32> RegionFitIndices;
	RegionFitIndices.Init(INDEX_NONE, ExtrusionSurfaceRegions.Num());
	for (int32 FitIndex = 0; FitIndex < C2TransitionSectionFits.Num(); ++FitIndex)
	{
		const int32 RegionIndex =
			C2TransitionSectionFits[FitIndex].ExtrusionRegionIndex;
		if (RegionFitIndices.IsValidIndex(RegionIndex))
		{
			RegionFitIndices[RegionIndex] = FitIndex;
		}
	}
	auto AcuteAngleDegrees = [](const FVector3d& A, const FVector3d& B)
	{
		const double Angle = VectorAngleDegrees(A, B);
		return FMath::Min(Angle, 180.0 - Angle);
	};
	auto ReflectVector = [](FVector3d Vector, const EPlanarSymmetryAxis Axis)
	{
		Vector[Axis == EPlanarSymmetryAxis::X ? 0 : 1] *= -1.0;
		return Vector;
	};
	auto AccumulatePlaneResiduals = [&](const FPlanarSurfaceGroup& Group,
		FVector3d RegularizedNormal, const FVector3d& Anchor,
		double& SquaredResidualSum, int32& ResidualCount,
		double& MaximumResidual)
	{
		if (FVector3d::DotProduct(RegularizedNormal, Group.Normal) < 0.0)
		{
			RegularizedNormal *= -1.0;
		}
		const double Offset = FVector3d::DotProduct(RegularizedNormal, Anchor);
		for (int32 PatchOffset = 0; PatchOffset < Group.PatchCount; ++PatchOffset)
		{
			const int32 PatchIndex =
				PlanarGroupPatchIndices[Group.FirstPatchIndex + PatchOffset];
			const FSurfacePatch& Patch = SurfacePatches[PatchIndex];
			for (int32 TriangleOffset = 0;
				TriangleOffset < Patch.TriangleCount; ++TriangleOffset)
			{
				const int32 TriangleIndex =
					PatchTriangleIndices[Patch.FirstTriangleIndex + TriangleOffset];
				for (int32 Corner = 0; Corner < 3; ++Corner)
				{
					const double Residual = FMath::Abs(FVector3d::DotProduct(
						RegularizedNormal, Triangles[TriangleIndex].Vertices[Corner]) -
						Offset);
					SquaredResidualSum += FMath::Square(Residual);
					++ResidualCount;
					MaximumResidual = FMath::Max(MaximumResidual, Residual);
				}
			}
		}
	};

	for (int32 PairIndex = 0;
		PairIndex < SharedC2TransitionPairFits.Num(); ++PairIndex)
	{
		const FSharedC2TransitionPairFit& Pair =
			SharedC2TransitionPairFits[PairIndex];
		FSharedC2FrameLedgerEntry& Entry =
			SharedC2FrameLedger.AddDefaulted_GetRef();
		Entry.SharedPairFitIndex = PairIndex;
		if (!Pair.bPlausible ||
			!RegionFitIndices.IsValidIndex(Pair.RegionAIndex) ||
			!RegionFitIndices.IsValidIndex(Pair.RegionBIndex) ||
			!C2TransitionSectionFits.IsValidIndex(
				RegionFitIndices[Pair.RegionAIndex]) ||
			!C2TransitionSectionFits.IsValidIndex(
				RegionFitIndices[Pair.RegionBIndex]))
		{
			continue;
		}
		const FExtrusionSurfaceRegion& RegionA =
			ExtrusionSurfaceRegions[Pair.RegionAIndex];
		const FExtrusionSurfaceRegion& RegionB =
			ExtrusionSurfaceRegions[Pair.RegionBIndex];
		const FC2TransitionSectionFit& FitA =
			C2TransitionSectionFits[RegionFitIndices[Pair.RegionAIndex]];
		const FC2TransitionSectionFit& FitB =
			C2TransitionSectionFits[RegionFitIndices[Pair.RegionBIndex]];
		FVector3d AxisA = RegionA.Axis;
		FVector3d MirroredAxisB = ReflectVector(RegionB.Axis, Pair.SymmetryAxis);
		if (FVector3d::DotProduct(AxisA, MirroredAxisB) < 0.0)
		{
			MirroredAxisB *= -1.0;
		}
		FVector3d RegularizedAxisA = AxisA + MirroredAxisB;
		RegularizedAxisA.Z = 0.0;
		RegularizedAxisA.Normalize();
		if (FVector3d::DotProduct(RegularizedAxisA, AxisA) < 0.0)
		{
			RegularizedAxisA *= -1.0;
		}
		FVector3d RegularizedAxisB = ReflectVector(
			RegularizedAxisA, Pair.SymmetryAxis);
		if (FVector3d::DotProduct(RegularizedAxisB, RegionB.Axis) < 0.0)
		{
			RegularizedAxisB *= -1.0;
		}
		if (RegularizedAxisA.IsNearlyZero() || RegularizedAxisB.IsNearlyZero())
		{
			continue;
		}
		Entry.RegularizedAxisA = RegularizedAxisA;
		Entry.RegularizedAxisB = RegularizedAxisB;
		Entry.MaximumExtrusionAxisAdjustmentDegrees = FMath::Max(
			AcuteAngleDegrees(RegularizedAxisA, RegionA.Axis),
			AcuteAngleDegrees(RegularizedAxisB, RegionB.Axis));

		double SquaredPlanarResidualSum = 0.0;
		int32 PlanarResidualCount = 0;
		bool bAllBoundariesValid = true;
		for (int32 SideIndex = 0; SideIndex < 2; ++SideIndex)
		{
			const FExtrusionSurfaceRegion& Region =
				SideIndex == 0 ? RegionA : RegionB;
			const FC2TransitionSectionFit& Fit =
				SideIndex == 0 ? FitA : FitB;
			const FVector3d RegularizedAxis =
				SideIndex == 0 ? RegularizedAxisA : RegularizedAxisB;
			FVector3d RegularizedU = FVector3d::CrossProduct(
				FVector3d::UpVector, RegularizedAxis).GetSafeNormal();
			if (FVector3d::DotProduct(RegularizedU, Region.SectionAxisU) < 0.0)
			{
				RegularizedU *= -1.0;
			}
			const FVector3d RegularizedV = FVector3d::UpVector;
			const double EndpointUCoordinate = Fit.CenterCoordinates.X +
				Fit.SignU * Fit.RadiusU;
			const double EndpointVCoordinate = Fit.CenterCoordinates.Y +
				Fit.SignV * Fit.RadiusV;
			const double CenterU = EndpointUCoordinate -
				Fit.SignU * Pair.SharedRadiusU;
			const double CenterV = EndpointVCoordinate -
				Fit.SignV * Pair.SharedRadiusV;
			const FVector3d EndpointU =
				EndpointUCoordinate * Region.SectionAxisU +
				CenterV * Region.SectionAxisV;
			const FVector3d EndpointV =
				CenterU * Region.SectionAxisU +
				EndpointVCoordinate * Region.SectionAxisV;
			for (int32 EndpointIndex = 0; EndpointIndex < 2; ++EndpointIndex)
			{
				const int32 BoundaryMatchIndex = EndpointIndex == 0
					? Fit.BoundaryMatchUIndex : Fit.BoundaryMatchVIndex;
				if (!QuarterEllipseBoundaryMatches.IsValidIndex(BoundaryMatchIndex))
				{
					bAllBoundariesValid = false;
					continue;
				}
				const FQuarterEllipseBoundaryMatch& Boundary =
					QuarterEllipseBoundaryMatches[BoundaryMatchIndex];
				if (!PlanarSurfaceGroups.IsValidIndex(Boundary.PlanarGroupIndex))
				{
					bAllBoundariesValid = false;
					continue;
				}
				const FPlanarSurfaceGroup& Plane =
					PlanarSurfaceGroups[Boundary.PlanarGroupIndex];
				const FVector3d RequestedNormal = EndpointIndex == 0
					? RegularizedU : RegularizedV;
				const FVector3d& Endpoint = EndpointIndex == 0
					? EndpointU : EndpointV;
				Entry.MaximumPlaneNormalAdjustmentDegrees = FMath::Max(
					Entry.MaximumPlaneNormalAdjustmentDegrees,
					AcuteAngleDegrees(RequestedNormal, Plane.Normal));
				Entry.MaximumOriginalPlaneEndpointResidualCm = FMath::Max(
					Entry.MaximumOriginalPlaneEndpointResidualCm,
					FMath::Abs(FVector3d::DotProduct(Plane.Normal, Endpoint) -
						Plane.PlaneOffset));
				AccumulatePlaneResiduals(Plane, RequestedNormal, Endpoint,
					SquaredPlanarResidualSum, PlanarResidualCount,
					Entry.PlanarSourceMaximumResidualCm);
			}
		}
		if (PlanarResidualCount > 0)
		{
			Entry.PlanarSourceRootMeanSquareResidualCm = FMath::Sqrt(
				SquaredPlanarResidualSum / static_cast<double>(PlanarResidualCount));
		}
		FC2TransitionSectionFit DerivativeProbe;
		DerivativeProbe.RadiusU = Pair.SharedRadiusU;
		DerivativeProbe.RadiusV = Pair.SharedRadiusV;
		DerivativeProbe.FlatteningFraction = Pair.SharedFlatteningFraction;
		const FVector2d StartPosition = DerivativeProbe.EvaluatePosition(0.0);
		const FVector2d EndPosition = DerivativeProbe.EvaluatePosition(1.0);
		Entry.MaximumConstructedEndpointPositionResidualCm = FMath::Max(
			FVector2d::Distance(StartPosition,
				FVector2d(0.0, Pair.SharedRadiusV)),
			FVector2d::Distance(EndPosition,
				FVector2d(Pair.SharedRadiusU, 0.0)));
		const FVector2d StartTangent =
			DerivativeProbe.EvaluateFirstDerivative(0.0);
		const FVector2d EndTangent =
			DerivativeProbe.EvaluateFirstDerivative(1.0);
		Entry.MinimumEndpointTangentMagnitude = FMath::Min(
			StartTangent.Length(), EndTangent.Length());
		Entry.MaximumEndpointTangentDirectionResidual = FMath::Max(
			FMath::Abs(StartTangent.Y) /
				FMath::Max(StartTangent.Length(), 1.0e-12),
			FMath::Abs(EndTangent.X) /
				FMath::Max(EndTangent.Length(), 1.0e-12));
		Entry.MaximumEndpointSecondDerivativeMagnitude = FMath::Max(
			DerivativeProbe.EvaluateSecondDerivative(0.0).Length(),
			DerivativeProbe.EvaluateSecondDerivative(1.0).Length());
		Entry.bC0ByConstruction = bAllBoundariesValid &&
			Entry.MaximumConstructedEndpointPositionResidualCm <= 1.0e-9;
		Entry.bG1ByConstruction = Entry.bC0ByConstruction &&
			Entry.MinimumEndpointTangentMagnitude > 1.0e-9 &&
			Entry.MaximumEndpointTangentDirectionResidual <= 1.0e-12;
		Entry.bG2ByConstruction = Entry.bG1ByConstruction &&
			Entry.MaximumEndpointSecondDerivativeMagnitude <= 1.0e-9;
		Entry.bParametricC2ByConstruction = Entry.bG2ByConstruction;
		Entry.bSourceRegularizationPlausible =
			Entry.bParametricC2ByConstruction &&
			Entry.MaximumExtrusionAxisAdjustmentDegrees <= 0.1 &&
			Entry.MaximumPlaneNormalAdjustmentDegrees <= 0.1 &&
			Entry.MaximumOriginalPlaneEndpointResidualCm <= 1.0 &&
			Entry.PlanarSourceMaximumResidualCm <= 25.0;
	}
}

void FAnalyticWorldData::BuildGlobalC2NetworkLedger()
{
	GlobalC2PlaneConstraints.Reset();
	GlobalC2JoinCertificates.Reset();
	struct FPlaneRequest
	{
		int32 SharedPairFitIndex = INDEX_NONE;
		FVector3d Normal = FVector3d::UpVector;
		FVector3d Endpoint = FVector3d::ZeroVector;
	};
	TArray<TArray<FPlaneRequest>> RequestsByPlane;
	RequestsByPlane.SetNum(PlanarSurfaceGroups.Num());
	TArray<int32> RegionFitIndices;
	RegionFitIndices.Init(INDEX_NONE, ExtrusionSurfaceRegions.Num());
	for (int32 FitIndex = 0; FitIndex < C2TransitionSectionFits.Num(); ++FitIndex)
	{
		const int32 RegionIndex =
			C2TransitionSectionFits[FitIndex].ExtrusionRegionIndex;
		if (RegionFitIndices.IsValidIndex(RegionIndex))
		{
			RegionFitIndices[RegionIndex] = FitIndex;
		}
	}
	for (const FSharedC2FrameLedgerEntry& Frame : SharedC2FrameLedger)
	{
		if (!Frame.bSourceRegularizationPlausible ||
			!SharedC2TransitionPairFits.IsValidIndex(Frame.SharedPairFitIndex))
		{
			continue;
		}
		const FSharedC2TransitionPairFit& Pair =
			SharedC2TransitionPairFits[Frame.SharedPairFitIndex];
		for (int32 SideIndex = 0; SideIndex < 2; ++SideIndex)
		{
			const int32 RegionIndex = SideIndex == 0
				? Pair.RegionAIndex : Pair.RegionBIndex;
			if (!ExtrusionSurfaceRegions.IsValidIndex(RegionIndex) ||
				!RegionFitIndices.IsValidIndex(RegionIndex) ||
				!C2TransitionSectionFits.IsValidIndex(
					RegionFitIndices[RegionIndex]))
			{
				continue;
			}
			const FExtrusionSurfaceRegion& Region =
				ExtrusionSurfaceRegions[RegionIndex];
			const FC2TransitionSectionFit& Fit =
				C2TransitionSectionFits[RegionFitIndices[RegionIndex]];
			const FVector3d RegularizedAxis = SideIndex == 0
				? Frame.RegularizedAxisA : Frame.RegularizedAxisB;
			FVector3d RegularizedU = FVector3d::CrossProduct(
				FVector3d::UpVector, RegularizedAxis).GetSafeNormal();
			if (FVector3d::DotProduct(RegularizedU, Region.SectionAxisU) < 0.0)
			{
				RegularizedU *= -1.0;
			}
			const double EndpointUCoordinate = Fit.CenterCoordinates.X +
				Fit.SignU * Fit.RadiusU;
			const double EndpointVCoordinate = Fit.CenterCoordinates.Y +
				Fit.SignV * Fit.RadiusV;
			const double CenterU = EndpointUCoordinate -
				Fit.SignU * Pair.SharedRadiusU;
			const double CenterV = EndpointVCoordinate -
				Fit.SignV * Pair.SharedRadiusV;
			const FVector3d Endpoints[2] = {
				EndpointUCoordinate * Region.SectionAxisU +
					CenterV * Region.SectionAxisV,
				CenterU * Region.SectionAxisU +
					EndpointVCoordinate * Region.SectionAxisV };
			const FVector3d RequestedNormals[2] = {
				RegularizedU, FVector3d::UpVector };
			for (int32 EndpointIndex = 0; EndpointIndex < 2; ++EndpointIndex)
			{
				const int32 MatchIndex = EndpointIndex == 0
					? Fit.BoundaryMatchUIndex : Fit.BoundaryMatchVIndex;
				if (!QuarterEllipseBoundaryMatches.IsValidIndex(MatchIndex)) continue;
				const int32 PlaneIndex =
					QuarterEllipseBoundaryMatches[MatchIndex].PlanarGroupIndex;
				if (!RequestsByPlane.IsValidIndex(PlaneIndex)) continue;
				FPlaneRequest& Request = RequestsByPlane[PlaneIndex].AddDefaulted_GetRef();
				Request.SharedPairFitIndex = Frame.SharedPairFitIndex;
				Request.Normal = RequestedNormals[EndpointIndex];
				Request.Endpoint = Endpoints[EndpointIndex];
			}
		}
	}

	TArray<int32> ConstraintIndexByPlane;
	ConstraintIndexByPlane.Init(INDEX_NONE, PlanarSurfaceGroups.Num());
	auto AcuteAngleDegrees = [](const FVector3d& A, const FVector3d& B)
	{
		const double Angle = VectorAngleDegrees(A, B);
		return FMath::Min(Angle, 180.0 - Angle);
	};
	for (int32 PlaneIndex = 0; PlaneIndex < RequestsByPlane.Num(); ++PlaneIndex)
	{
		const TArray<FPlaneRequest>& Requests = RequestsByPlane[PlaneIndex];
		if (Requests.IsEmpty()) continue;
		const FPlanarSurfaceGroup& SourcePlane = PlanarSurfaceGroups[PlaneIndex];
		FGlobalC2PlaneConstraint& Constraint =
			GlobalC2PlaneConstraints.AddDefaulted_GetRef();
		ConstraintIndexByPlane[PlaneIndex] = GlobalC2PlaneConstraints.Num() - 1;
		Constraint.PlanarGroupIndex = PlaneIndex;
		Constraint.RequestCount = Requests.Num();
		FVector3d NormalSum = FVector3d::ZeroVector;
		for (const FPlaneRequest& Request : Requests)
		{
			FVector3d Normal = Request.Normal;
			if (FVector3d::DotProduct(Normal, SourcePlane.Normal) < 0.0)
			{
				Normal *= -1.0;
			}
			NormalSum += Normal;
		}
		Constraint.Normal = NormalSum.GetSafeNormal();
		if (Constraint.Normal.IsNearlyZero()) continue;
		double OffsetSum = 0.0;
		for (const FPlaneRequest& Request : Requests)
		{
			OffsetSum += FVector3d::DotProduct(
				Constraint.Normal, Request.Endpoint);
		}
		Constraint.PlaneOffset = OffsetSum /
			static_cast<double>(Requests.Num());
		for (const FPlaneRequest& Request : Requests)
		{
			Constraint.MaximumRequestedNormalConflictDegrees = FMath::Max(
				Constraint.MaximumRequestedNormalConflictDegrees,
				AcuteAngleDegrees(Constraint.Normal, Request.Normal));
			Constraint.MaximumRequestedEndpointResidualCm = FMath::Max(
				Constraint.MaximumRequestedEndpointResidualCm,
				FMath::Abs(FVector3d::DotProduct(
					Constraint.Normal, Request.Endpoint) - Constraint.PlaneOffset));
		}
		Constraint.SourceNormalAdjustmentDegrees =
			AcuteAngleDegrees(Constraint.Normal, SourcePlane.Normal);
		double SourceSquaredResidualSum = 0.0;
		int32 SourceResidualCount = 0;
		for (int32 PatchOffset = 0; PatchOffset < SourcePlane.PatchCount; ++PatchOffset)
		{
			const int32 PatchIndex = PlanarGroupPatchIndices[
				SourcePlane.FirstPatchIndex + PatchOffset];
			const FSurfacePatch& Patch = SurfacePatches[PatchIndex];
			for (int32 TriangleOffset = 0;
				TriangleOffset < Patch.TriangleCount; ++TriangleOffset)
			{
				const int32 TriangleIndex = PatchTriangleIndices[
					Patch.FirstTriangleIndex + TriangleOffset];
				for (int32 Corner = 0; Corner < 3; ++Corner)
				{
					const double Residual = FMath::Abs(FVector3d::DotProduct(
						Constraint.Normal,
						Triangles[TriangleIndex].Vertices[Corner]) -
						Constraint.PlaneOffset);
					SourceSquaredResidualSum += FMath::Square(Residual);
					++SourceResidualCount;
					Constraint.SourceMaximumResidualCm = FMath::Max(
						Constraint.SourceMaximumResidualCm, Residual);
				}
			}
		}
		if (SourceResidualCount > 0)
		{
			Constraint.SourceRootMeanSquareResidualCm = FMath::Sqrt(
				SourceSquaredResidualSum /
				static_cast<double>(SourceResidualCount));
		}
		Constraint.bConstraintCompatible =
			Constraint.MaximumRequestedNormalConflictDegrees <= 0.1 &&
			Constraint.MaximumRequestedEndpointResidualCm <= 1.0 &&
			Constraint.SourceNormalAdjustmentDegrees <= 0.1 &&
			Constraint.SourceMaximumResidualCm <= 25.0;
	}

	for (const FSharedC2FrameLedgerEntry& Frame : SharedC2FrameLedger)
	{
		if (!Frame.bSourceRegularizationPlausible) continue;
		FGlobalC2JoinCertificate& Certificate =
			GlobalC2JoinCertificates.AddDefaulted_GetRef();
		Certificate.SharedPairFitIndex = Frame.SharedPairFitIndex;
		bool bAllConstraintsCompatible = true;
		for (int32 PlaneIndex = 0; PlaneIndex < RequestsByPlane.Num(); ++PlaneIndex)
		{
			const int32 ConstraintIndex = ConstraintIndexByPlane[PlaneIndex];
			if (!GlobalC2PlaneConstraints.IsValidIndex(ConstraintIndex)) continue;
			const FGlobalC2PlaneConstraint& Constraint =
				GlobalC2PlaneConstraints[ConstraintIndex];
			for (const FPlaneRequest& Request : RequestsByPlane[PlaneIndex])
			{
				if (Request.SharedPairFitIndex != Frame.SharedPairFitIndex) continue;
				++Certificate.BoundaryCount;
				Certificate.MaximumEndpointPlaneResidualCm = FMath::Max(
					Certificate.MaximumEndpointPlaneResidualCm,
					FMath::Abs(FVector3d::DotProduct(
						Constraint.Normal, Request.Endpoint) -
						Constraint.PlaneOffset));
				Certificate.MaximumTangentNormalResidualDegrees = FMath::Max(
					Certificate.MaximumTangentNormalResidualDegrees,
					AcuteAngleDegrees(Constraint.Normal, Request.Normal));
				bAllConstraintsCompatible &= Constraint.bConstraintCompatible;
			}
		}
		Certificate.bNetworkC0Plausible = Certificate.BoundaryCount == 4 &&
			bAllConstraintsCompatible &&
			Certificate.MaximumEndpointPlaneResidualCm <= 1.0;
		Certificate.bNetworkG1Plausible = Certificate.bNetworkC0Plausible &&
			Certificate.MaximumTangentNormalResidualDegrees <= 0.1;
		Certificate.bLocalC2Retained = Certificate.bNetworkG1Plausible &&
			Frame.bParametricC2ByConstruction;
	}
}

void FAnalyticWorldData::BuildSymmetrizedC2PlaneConstraints()
{
	SymmetrizedC2PlaneConstraints.Reset();
	C2SymmetryPlacementFrame = FC2SymmetryPlacementFrame();
	if (GlobalC2PlaneConstraints.IsEmpty()) return;
	FBox3d MeshBounds(EForceInit::ForceInit);
	for (const FVector3d& Position : MeshVertices) MeshBounds += Position;
	FVector3d SymmetryCenter = MeshBounds.GetCenter();
	C2SymmetryPlacementFrame.BoundsCenter = FVector2d(
		SymmetryCenter.X, SymmetryCenter.Y);
	auto ReflectPlane = [&](FVector3d& Normal, double& Offset,
		const uint8 TransformMask)
	{
		if ((TransformMask & 1u) != 0)
		{
			Offset -= 2.0 * SymmetryCenter.X * Normal.X;
			Normal.X *= -1.0;
		}
		if ((TransformMask & 2u) != 0)
		{
			Offset -= 2.0 * SymmetryCenter.Y * Normal.Y;
			Normal.Y *= -1.0;
		}
	};
	auto AcuteAngleDegrees = [](const FVector3d& A, const FVector3d& B)
	{
		const double Angle = VectorAngleDegrees(A, B);
		return FMath::Min(Angle, 180.0 - Angle);
	};
	TArray<int32> ConstraintIndexByPlane;
	ConstraintIndexByPlane.Init(INDEX_NONE, PlanarSurfaceGroups.Num());
	for (int32 ConstraintIndex = 0;
		ConstraintIndex < GlobalC2PlaneConstraints.Num(); ++ConstraintIndex)
	{
		const int32 PlaneIndex =
			GlobalC2PlaneConstraints[ConstraintIndex].PlanarGroupIndex;
		if (ConstraintIndexByPlane.IsValidIndex(PlaneIndex))
		{
			ConstraintIndexByPlane[PlaneIndex] = ConstraintIndex;
		}
	}
	TArray<int32> RegionFitIndices;
	RegionFitIndices.Init(INDEX_NONE, ExtrusionSurfaceRegions.Num());
	for (int32 FitIndex = 0; FitIndex < C2TransitionSectionFits.Num(); ++FitIndex)
	{
		const int32 RegionIndex =
			C2TransitionSectionFits[FitIndex].ExtrusionRegionIndex;
		if (RegionFitIndices.IsValidIndex(RegionIndex))
		{
			RegionFitIndices[RegionIndex] = FitIndex;
		}
	}
	struct FMirrorEdge
	{
		int32 PlaneA = INDEX_NONE;
		int32 PlaneB = INDEX_NONE;
		uint8 TransformMask = 0;
	};
	TArray<FMirrorEdge> Edges;
	auto AddPairEdges = [&](const FSharedC2TransitionPairFit& Pair)
	{
		if (!RegionFitIndices.IsValidIndex(Pair.RegionAIndex) ||
			!RegionFitIndices.IsValidIndex(Pair.RegionBIndex) ||
			!C2TransitionSectionFits.IsValidIndex(
				RegionFitIndices[Pair.RegionAIndex]) ||
			!C2TransitionSectionFits.IsValidIndex(
				RegionFitIndices[Pair.RegionBIndex]))
		{
			return;
		}
		const FC2TransitionSectionFit& FitA =
			C2TransitionSectionFits[RegionFitIndices[Pair.RegionAIndex]];
		const FC2TransitionSectionFit& FitB =
			C2TransitionSectionFits[RegionFitIndices[Pair.RegionBIndex]];
		const int32 MatchesA[2] = {
			FitA.BoundaryMatchUIndex, FitA.BoundaryMatchVIndex };
		const int32 MatchesB[2] = {
			FitB.BoundaryMatchUIndex, FitB.BoundaryMatchVIndex };
		for (int32 EndpointIndex = 0; EndpointIndex < 2; ++EndpointIndex)
		{
			if (!QuarterEllipseBoundaryMatches.IsValidIndex(
					MatchesA[EndpointIndex]) ||
				!QuarterEllipseBoundaryMatches.IsValidIndex(
					MatchesB[EndpointIndex]))
			{
				continue;
			}
			const int32 PlaneA = QuarterEllipseBoundaryMatches[
				MatchesA[EndpointIndex]].PlanarGroupIndex;
			const int32 PlaneB = QuarterEllipseBoundaryMatches[
				MatchesB[EndpointIndex]].PlanarGroupIndex;
			if (!ConstraintIndexByPlane.IsValidIndex(PlaneA) ||
				!ConstraintIndexByPlane.IsValidIndex(PlaneB) ||
				ConstraintIndexByPlane[PlaneA] == INDEX_NONE ||
				ConstraintIndexByPlane[PlaneB] == INDEX_NONE)
			{
				continue;
			}
			FMirrorEdge& Edge = Edges.AddDefaulted_GetRef();
			Edge.PlaneA = PlaneA;
			Edge.PlaneB = PlaneB;
			Edge.TransformMask = Pair.SymmetryAxis == EPlanarSymmetryAxis::X
				? 1u : 2u;
		}
	};
	for (const FSharedC2TransitionPairFit& Pair : SharedC2TransitionPairFits)
	{
		if (Pair.bPlausible) AddPairEdges(Pair);
	}
	auto FitSymmetryCenter = [&](const uint8 TransformMask,
		int32& OutConstraintCount, double& OutRootMeanSquareResidual,
		double& OutMaximumResidual)
	{
		const int32 Axis = TransformMask == 1u ? 0 : 1;
		TArray<double> Estimates;
		for (const FMirrorEdge& Edge : Edges)
		{
			if (Edge.TransformMask != TransformMask) continue;
			const FGlobalC2PlaneConstraint& A = GlobalC2PlaneConstraints[
				ConstraintIndexByPlane[Edge.PlaneA]];
			const FGlobalC2PlaneConstraint& B = GlobalC2PlaneConstraints[
				ConstraintIndexByPlane[Edge.PlaneB]];
			if (FMath::Abs(A.Normal[Axis]) <= 0.5) continue;
			FVector3d ReflectedNormal = A.Normal;
			ReflectedNormal[Axis] *= -1.0;
			double OrientedBOffset = B.PlaneOffset;
			if (FVector3d::DotProduct(ReflectedNormal, B.Normal) < 0.0)
			{
				OrientedBOffset *= -1.0;
			}
			Estimates.Add((A.PlaneOffset - OrientedBOffset) /
				(2.0 * A.Normal[Axis]));
		}
		if (Estimates.IsEmpty()) return SymmetryCenter[Axis];
		Algo::Sort(Estimates);
		const int32 Middle = Estimates.Num() / 2;
		const double Center = Estimates.Num() % 2 == 0
			? 0.5 * (Estimates[Middle - 1] + Estimates[Middle])
			: Estimates[Middle];
		double SquaredResidualSum = 0.0;
		for (const double Estimate : Estimates)
		{
			const double Residual = FMath::Abs(Estimate - Center);
			SquaredResidualSum += FMath::Square(Residual);
			OutMaximumResidual = FMath::Max(OutMaximumResidual, Residual);
		}
		OutConstraintCount = Estimates.Num();
		OutRootMeanSquareResidual = FMath::Sqrt(
			SquaredResidualSum / static_cast<double>(Estimates.Num()));
		return Center;
	};
	FVector3d SourceInferredCenter = SymmetryCenter;
	SourceInferredCenter.X = FitSymmetryCenter(1u,
		C2SymmetryPlacementFrame.XConstraintCount,
		C2SymmetryPlacementFrame.XRootMeanSquareResidualCm,
		C2SymmetryPlacementFrame.XMaximumResidualCm);
	SourceInferredCenter.Y = FitSymmetryCenter(2u,
		C2SymmetryPlacementFrame.YConstraintCount,
		C2SymmetryPlacementFrame.YRootMeanSquareResidualCm,
		C2SymmetryPlacementFrame.YMaximumResidualCm);
	C2SymmetryPlacementFrame.SourceInferredCenter = FVector2d(
		SourceInferredCenter.X, SourceInferredCenter.Y);
	// The accepted centered-environment contract fixes the reflection planes at
	// local X=0 and Y=0. The inferred center above measures source bias only; it
	// must not move the canonical analytical frame.
	SymmetryCenter.X = 0.0;
	SymmetryCenter.Y = 0.0;
	C2SymmetryPlacementFrame.TargetCenter = FVector2d::ZeroVector;
	C2SymmetryPlacementFrame.bFittedFromMirrorPlanes =
		C2SymmetryPlacementFrame.XConstraintCount > 0 &&
		C2SymmetryPlacementFrame.YConstraintCount > 0;
	struct FAdjacency
	{
		int32 OtherPlane = INDEX_NONE;
		uint8 TransformMask = 0;
	};
	TArray<TArray<FAdjacency>> Adjacency;
	Adjacency.SetNum(PlanarSurfaceGroups.Num());
	for (const FMirrorEdge& Edge : Edges)
	{
		Adjacency[Edge.PlaneA].Add({ Edge.PlaneB, Edge.TransformMask });
		Adjacency[Edge.PlaneB].Add({ Edge.PlaneA, Edge.TransformMask });
	}
	TArray<int32> AssignedTransforms;
	AssignedTransforms.Init(INDEX_NONE, PlanarSurfaceGroups.Num());
	TArray<int32> OutputIndexByPlane;
	OutputIndexByPlane.Init(INDEX_NONE, PlanarSurfaceGroups.Num());
	for (const FGlobalC2PlaneConstraint& Seed : GlobalC2PlaneConstraints)
	{
		const int32 RootPlane = Seed.PlanarGroupIndex;
		if (AssignedTransforms[RootPlane] != INDEX_NONE) continue;
		AssignedTransforms[RootPlane] = 0;
		uint8 StabilizerElements = 1u;
		TArray<int32> Pending = { RootPlane };
		TArray<int32> OrbitPlanes;
		while (!Pending.IsEmpty())
		{
			const int32 PlaneIndex = Pending.Pop(EAllowShrinking::No);
			OrbitPlanes.Add(PlaneIndex);
			for (const FAdjacency& Link : Adjacency[PlaneIndex])
			{
				const int32 ExpectedTransform =
					AssignedTransforms[PlaneIndex] ^ Link.TransformMask;
				if (AssignedTransforms[Link.OtherPlane] == INDEX_NONE)
				{
					AssignedTransforms[Link.OtherPlane] = ExpectedTransform;
					Pending.Add(Link.OtherPlane);
				}
				else if (AssignedTransforms[Link.OtherPlane] != ExpectedTransform)
				{
					const uint8 Generator = static_cast<uint8>(
						AssignedTransforms[Link.OtherPlane] ^ ExpectedTransform);
					bool bChanged = true;
					while (bChanged)
					{
						bChanged = false;
						for (uint8 Element = 0; Element < 4; ++Element)
						{
							if ((StabilizerElements & (1u << Element)) == 0) continue;
							const uint8 NewElement = Element ^ Generator;
							if ((StabilizerElements & (1u << NewElement)) == 0)
							{
								StabilizerElements |= 1u << NewElement;
								bChanged = true;
							}
						}
					}
				}
			}
		}
		Algo::Sort(OrbitPlanes, [&](const int32 A, const int32 B)
		{
			return PlanarSurfaceGroups[A].GroupId <
				PlanarSurfaceGroups[B].GroupId;
		});
		uint64 OrbitId = MAX_uint64;
		FVector3d ReferenceNormal = FVector3d::ZeroVector;
		FVector3d MasterNormalSum = FVector3d::ZeroVector;
		double MasterOffsetSum = 0.0;
		for (const int32 PlaneIndex : OrbitPlanes)
		{
			OrbitId = FMath::Min(OrbitId, PlanarSurfaceGroups[PlaneIndex].GroupId);
			const FGlobalC2PlaneConstraint& Constraint =
				GlobalC2PlaneConstraints[ConstraintIndexByPlane[PlaneIndex]];
			FVector3d Normal = Constraint.Normal;
			double Offset = Constraint.PlaneOffset;
			ReflectPlane(Normal, Offset,
				static_cast<uint8>(AssignedTransforms[PlaneIndex]));
			if (ReferenceNormal.IsNearlyZero()) ReferenceNormal = Normal;
			if (FVector3d::DotProduct(Normal, ReferenceNormal) < 0.0)
			{
				Normal *= -1.0;
				Offset *= -1.0;
			}
			MasterNormalSum += Normal;
			MasterOffsetSum += Offset;
		}
		FVector3d MasterNormal = MasterNormalSum.GetSafeNormal();
		double MasterOffset = MasterOffsetSum /
			static_cast<double>(OrbitPlanes.Num());
		if (StabilizerElements != 1u)
		{
			FVector3d InvariantNormalSum = FVector3d::ZeroVector;
			double InvariantOffsetSum = 0.0;
			int32 InvariantCount = 0;
			for (uint8 Element = 0; Element < 4; ++Element)
			{
				if ((StabilizerElements & (1u << Element)) == 0) continue;
				FVector3d Normal = MasterNormal;
				double Offset = MasterOffset;
				ReflectPlane(Normal, Offset, Element);
				if (FVector3d::DotProduct(Normal, MasterNormal) < 0.0)
				{
					Normal *= -1.0;
					Offset *= -1.0;
				}
				InvariantNormalSum += Normal;
				InvariantOffsetSum += Offset;
				++InvariantCount;
			}
			MasterNormal = InvariantNormalSum.GetSafeNormal();
			MasterOffset = InvariantOffsetSum /
				static_cast<double>(InvariantCount);
		}
		for (const int32 PlaneIndex : OrbitPlanes)
		{
			FSymmetrizedC2PlaneConstraint& Output =
				SymmetrizedC2PlaneConstraints.AddDefaulted_GetRef();
			OutputIndexByPlane[PlaneIndex] =
				SymmetrizedC2PlaneConstraints.Num() - 1;
			Output.OrbitId = OrbitId;
			Output.PlanarGroupIndex = PlaneIndex;
			Output.TransformMaskFromOrbitRoot = static_cast<uint8>(
				AssignedTransforms[PlaneIndex]);
			Output.Normal = MasterNormal;
			Output.PlaneOffset = MasterOffset;
			ReflectPlane(Output.Normal, Output.PlaneOffset,
				Output.TransformMaskFromOrbitRoot);
			const FPlanarSurfaceGroup& SourcePlane =
				PlanarSurfaceGroups[PlaneIndex];
			if (FVector3d::DotProduct(Output.Normal, SourcePlane.Normal) < 0.0)
			{
				Output.Normal *= -1.0;
				Output.PlaneOffset *= -1.0;
			}
			Output.SourceNormalAdjustmentDegrees =
				AcuteAngleDegrees(Output.Normal, SourcePlane.Normal);
			double SquaredResidualSum = 0.0;
			int32 ResidualCount = 0;
			for (int32 PatchOffset = 0;
				PatchOffset < SourcePlane.PatchCount; ++PatchOffset)
			{
				const int32 PatchIndex = PlanarGroupPatchIndices[
					SourcePlane.FirstPatchIndex + PatchOffset];
				const FSurfacePatch& Patch = SurfacePatches[PatchIndex];
				for (int32 TriangleOffset = 0;
					TriangleOffset < Patch.TriangleCount; ++TriangleOffset)
				{
					const int32 TriangleIndex = PatchTriangleIndices[
						Patch.FirstTriangleIndex + TriangleOffset];
					for (int32 Corner = 0; Corner < 3; ++Corner)
					{
						const double Residual = FMath::Abs(FVector3d::DotProduct(
							Output.Normal,
							Triangles[TriangleIndex].Vertices[Corner]) -
							Output.PlaneOffset);
						SquaredResidualSum += FMath::Square(Residual);
						++ResidualCount;
						Output.SourceMaximumResidualCm = FMath::Max(
							Output.SourceMaximumResidualCm, Residual);
					}
				}
			}
			Output.SourceRootMeanSquareResidualCm = ResidualCount > 0
				? FMath::Sqrt(SquaredResidualSum /
					static_cast<double>(ResidualCount)) : 0.0;
			Output.bSourceFitPlausible =
				Output.SourceNormalAdjustmentDegrees <= 0.1 &&
				Output.SourceMaximumResidualCm <= 25.0;
		}
	}
	for (const FMirrorEdge& Edge : Edges)
	{
		if (!OutputIndexByPlane.IsValidIndex(Edge.PlaneA) ||
			!OutputIndexByPlane.IsValidIndex(Edge.PlaneB) ||
			!SymmetrizedC2PlaneConstraints.IsValidIndex(
				OutputIndexByPlane[Edge.PlaneA]) ||
			!SymmetrizedC2PlaneConstraints.IsValidIndex(
				OutputIndexByPlane[Edge.PlaneB]))
		{
			continue;
		}
		FSymmetrizedC2PlaneConstraint& A =
			SymmetrizedC2PlaneConstraints[OutputIndexByPlane[Edge.PlaneA]];
		FSymmetrizedC2PlaneConstraint& B =
			SymmetrizedC2PlaneConstraints[OutputIndexByPlane[Edge.PlaneB]];
		FVector3d ReflectedNormal = A.Normal;
		double ReflectedOffset = A.PlaneOffset;
		ReflectPlane(ReflectedNormal, ReflectedOffset, Edge.TransformMask);
		if (FVector3d::DotProduct(ReflectedNormal, B.Normal) < 0.0)
		{
			ReflectedNormal *= -1.0;
			ReflectedOffset *= -1.0;
		}
		const double NormalResidual =
			AcuteAngleDegrees(ReflectedNormal, B.Normal);
		const double OffsetResidual = FMath::Abs(ReflectedOffset - B.PlaneOffset);
		A.MaximumMirrorNormalResidualDegrees = FMath::Max(
			A.MaximumMirrorNormalResidualDegrees, NormalResidual);
		B.MaximumMirrorNormalResidualDegrees = FMath::Max(
			B.MaximumMirrorNormalResidualDegrees, NormalResidual);
		A.MaximumMirrorOffsetResidualCm = FMath::Max(
			A.MaximumMirrorOffsetResidualCm, OffsetResidual);
		B.MaximumMirrorOffsetResidualCm = FMath::Max(
			B.MaximumMirrorOffsetResidualCm, OffsetResidual);
	}
	for (FSymmetrizedC2PlaneConstraint& Constraint :
		SymmetrizedC2PlaneConstraints)
	{
		Constraint.bExactMirrorPlacement =
			Constraint.MaximumMirrorNormalResidualDegrees <= 1.0e-9 &&
			Constraint.MaximumMirrorOffsetResidualCm <= 1.0e-9;
	}
}

void FAnalyticWorldData::BuildCoupledC2TransitionSolutions()
{
	CoupledC2TransitionSolutions.Reset();
	CoupledC2TransitionFamilies.Reset();
	CoupledC2FamilyRegionIndices.Reset();
	TArray<int32> ConstraintIndexByPlane;
	ConstraintIndexByPlane.Init(INDEX_NONE, PlanarSurfaceGroups.Num());
	for (int32 ConstraintIndex = 0;
		ConstraintIndex < SymmetrizedC2PlaneConstraints.Num(); ++ConstraintIndex)
	{
		const int32 PlaneIndex =
			SymmetrizedC2PlaneConstraints[ConstraintIndex].PlanarGroupIndex;
		if (ConstraintIndexByPlane.IsValidIndex(PlaneIndex))
		{
			ConstraintIndexByPlane[PlaneIndex] = ConstraintIndex;
		}
	}
	TArray<int32> RegionFitIndices;
	RegionFitIndices.Init(INDEX_NONE, ExtrusionSurfaceRegions.Num());
	for (int32 FitIndex = 0; FitIndex < C2TransitionSectionFits.Num(); ++FitIndex)
	{
		const int32 RegionIndex =
			C2TransitionSectionFits[FitIndex].ExtrusionRegionIndex;
		if (RegionFitIndices.IsValidIndex(RegionIndex))
		{
			RegionFitIndices[RegionIndex] = FitIndex;
		}
	}
	struct FSideData
	{
		const FExtrusionSurfaceRegion* Region = nullptr;
		const FC2TransitionSectionFit* Fit = nullptr;
		int32 PlaneUIndex = INDEX_NONE;
		int32 PlaneVIndex = INDEX_NONE;
		FVector3d NormalU = FVector3d::ForwardVector;
		FVector3d NormalV = FVector3d::UpVector;
		FVector3d DualU = FVector3d::ForwardVector;
		FVector3d DualV = FVector3d::UpVector;
		double OffsetU = 0.0;
		double OffsetV = 0.0;
		double OrthogonalityResidual = 0.0;
		TArray<FVector2d> Points;
		bool bValid = false;
	};
	auto BuildSideData = [&](const int32 RegionIndex)
	{
		FSideData Side;
		if (!ExtrusionSurfaceRegions.IsValidIndex(RegionIndex) ||
			!RegionFitIndices.IsValidIndex(RegionIndex) ||
			!C2TransitionSectionFits.IsValidIndex(RegionFitIndices[RegionIndex]))
		{
			return Side;
		}
		Side.Region = &ExtrusionSurfaceRegions[RegionIndex];
		Side.Fit = &C2TransitionSectionFits[RegionFitIndices[RegionIndex]];
		if (!QuarterEllipseBoundaryMatches.IsValidIndex(
				Side.Fit->BoundaryMatchUIndex) ||
			!QuarterEllipseBoundaryMatches.IsValidIndex(
				Side.Fit->BoundaryMatchVIndex))
		{
			return Side;
		}
		Side.PlaneUIndex = QuarterEllipseBoundaryMatches[
			Side.Fit->BoundaryMatchUIndex].PlanarGroupIndex;
		Side.PlaneVIndex = QuarterEllipseBoundaryMatches[
			Side.Fit->BoundaryMatchVIndex].PlanarGroupIndex;
		if (!ConstraintIndexByPlane.IsValidIndex(Side.PlaneUIndex) ||
			!ConstraintIndexByPlane.IsValidIndex(Side.PlaneVIndex) ||
			!SymmetrizedC2PlaneConstraints.IsValidIndex(
				ConstraintIndexByPlane[Side.PlaneUIndex]) ||
			!SymmetrizedC2PlaneConstraints.IsValidIndex(
				ConstraintIndexByPlane[Side.PlaneVIndex]))
		{
			return Side;
		}
		const FSymmetrizedC2PlaneConstraint& PlaneU = SymmetrizedC2PlaneConstraints[
			ConstraintIndexByPlane[Side.PlaneUIndex]];
		const FSymmetrizedC2PlaneConstraint& PlaneV = SymmetrizedC2PlaneConstraints[
			ConstraintIndexByPlane[Side.PlaneVIndex]];
		if (!PlaneU.bSourceFitPlausible || !PlaneV.bSourceFitPlausible ||
			!PlaneU.bExactMirrorPlacement || !PlaneV.bExactMirrorPlacement)
		{
			return Side;
		}
		Side.NormalU = PlaneU.Normal;
		Side.OffsetU = PlaneU.PlaneOffset;
		if (FVector3d::DotProduct(Side.NormalU, Side.Region->SectionAxisU) < 0.0)
		{
			Side.NormalU *= -1.0;
			Side.OffsetU *= -1.0;
		}
		Side.NormalV = PlaneV.Normal;
		Side.OffsetV = PlaneV.PlaneOffset;
		if (FVector3d::DotProduct(Side.NormalV, Side.Region->SectionAxisV) < 0.0)
		{
			Side.NormalV *= -1.0;
			Side.OffsetV *= -1.0;
		}
		const double Dot = FVector3d::DotProduct(Side.NormalU, Side.NormalV);
		const double Denominator = 1.0 - Dot * Dot;
		if (Denominator <= 1.0e-8) return Side;
		Side.OrthogonalityResidual = FMath::Abs(Dot);
		Side.DualU = (Side.NormalU - Dot * Side.NormalV) / Denominator;
		Side.DualV = (Side.NormalV - Dot * Side.NormalU) / Denominator;
		TSet<int32> UniqueVertexSet;
		for (int32 MemberIndex = Side.Region->FirstTriangleIndex;
			MemberIndex < Side.Region->FirstTriangleIndex +
				Side.Region->TriangleCount; ++MemberIndex)
		{
			const int32 TriangleIndex = ExtrusionRegionTriangleIndices[MemberIndex];
			for (int32 Corner = 0; Corner < 3; ++Corner)
			{
				UniqueVertexSet.Add(TriangleVertexIndices[TriangleIndex][Corner]);
			}
		}
		TArray<int32> UniqueVertices = UniqueVertexSet.Array();
		Algo::Sort(UniqueVertices);
		Side.Points.Reserve(UniqueVertices.Num());
		for (const int32 VertexIndex : UniqueVertices)
		{
			const FVector3d& Position = MeshVertices[VertexIndex];
			Side.Points.Emplace(
				FVector3d::DotProduct(Side.NormalU, Position),
				FVector3d::DotProduct(Side.NormalV, Position));
		}
		Side.bValid = !Side.Points.IsEmpty();
		return Side;
	};
	auto MeasureSide = [](const FSideData& Side, const double RadiusU,
		const double RadiusV, const double FlatteningFraction,
		double* OutMaximumResidual)
	{
		const FVector2d Center(
			Side.OffsetU - Side.Fit->SignU * RadiusU,
			Side.OffsetV - Side.Fit->SignV * RadiusV);
		double MeanSquaredResidual = 0.0;
		double MaximumResidual = 0.0;
		for (const FVector2d& Point : Side.Points)
		{
			const double SquaredResidual =
				SquaredDistanceToQuinticTransitionInDualBasis(
					Point, Center, RadiusU, RadiusV, Side.Fit->SignU,
					Side.Fit->SignV, FlatteningFraction,
					Side.DualU, Side.DualV);
			MeanSquaredResidual += SquaredResidual;
			MaximumResidual = FMath::Max(
				MaximumResidual, FMath::Sqrt(SquaredResidual));
		}
		if (OutMaximumResidual) *OutMaximumResidual = MaximumResidual;
		return MeanSquaredResidual / static_cast<double>(Side.Points.Num());
	};

	for (int32 PairIndex = 0;
		PairIndex < SharedC2TransitionPairFits.Num(); ++PairIndex)
	{
		const FSharedC2TransitionPairFit& Pair =
			SharedC2TransitionPairFits[PairIndex];
		if (!Pair.bPlausible) continue;
		const FSideData SideA = BuildSideData(Pair.RegionAIndex);
		const FSideData SideB = BuildSideData(Pair.RegionBIndex);
		if (!SideA.bValid || !SideB.bValid) continue;
		FCoupledC2TransitionPairSolution& Solution =
			CoupledC2TransitionSolutions.AddDefaulted_GetRef();
		Solution.SharedPairFitIndex = PairIndex;
		Solution.RegionAPlaneUIndex = SideA.PlaneUIndex;
		Solution.RegionAPlaneVIndex = SideA.PlaneVIndex;
		Solution.RegionBPlaneUIndex = SideB.PlaneUIndex;
		Solution.RegionBPlaneVIndex = SideB.PlaneVIndex;
		Solution.SharedRadiusU = Pair.SharedRadiusU;
		Solution.SharedRadiusV = Pair.SharedRadiusV;
		Solution.SharedFlatteningFraction = Pair.SharedFlatteningFraction;
		auto Objective = [&](const double RadiusU, const double RadiusV,
			const double FlatteningFraction)
		{
			return MeasureSide(SideA, RadiusU, RadiusV,
				FlatteningFraction, nullptr) +
				MeasureSide(SideB, RadiusU, RadiusV,
					FlatteningFraction, nullptr);
		};
		double Steps[3] = { 0.02 * Solution.SharedRadiusU,
			0.02 * Solution.SharedRadiusV, 0.01 };
		double BestObjective = Objective(Solution.SharedRadiusU,
			Solution.SharedRadiusV, Solution.SharedFlatteningFraction);
		for (int32 Iteration = 0; Iteration < 14; ++Iteration)
		{
			double BestRadiusU = Solution.SharedRadiusU;
			double BestRadiusV = Solution.SharedRadiusV;
			double BestFlattening = Solution.SharedFlatteningFraction;
			for (int32 DirectionU = -1; DirectionU <= 1; ++DirectionU)
			{
				for (int32 DirectionV = -1; DirectionV <= 1; ++DirectionV)
				{
					for (int32 DirectionF = -1; DirectionF <= 1; ++DirectionF)
					{
						if (DirectionU == 0 && DirectionV == 0 && DirectionF == 0)
						{
							continue;
						}
						const double CandidateRadiusU = FMath::Max(1.0,
							Solution.SharedRadiusU + DirectionU * Steps[0]);
						const double CandidateRadiusV = FMath::Max(1.0,
							Solution.SharedRadiusV + DirectionV * Steps[1]);
						const double CandidateFlattening = FMath::Clamp(
							Solution.SharedFlatteningFraction + DirectionF * Steps[2],
							0.01, 0.49);
						const double CandidateObjective = Objective(CandidateRadiusU,
							CandidateRadiusV, CandidateFlattening);
						if (CandidateObjective < BestObjective)
						{
							BestObjective = CandidateObjective;
							BestRadiusU = CandidateRadiusU;
							BestRadiusV = CandidateRadiusV;
							BestFlattening = CandidateFlattening;
						}
					}
				}
			}
			Solution.SharedRadiusU = BestRadiusU;
			Solution.SharedRadiusV = BestRadiusV;
			Solution.SharedFlatteningFraction = BestFlattening;
			for (double& Step : Steps) Step *= 0.5;
		}
		Solution.RegionACenterCoordinates = FVector2d(
			SideA.OffsetU - SideA.Fit->SignU * Solution.SharedRadiusU,
			SideA.OffsetV - SideA.Fit->SignV * Solution.SharedRadiusV);
		Solution.RegionBCenterCoordinates = FVector2d(
			SideB.OffsetU - SideB.Fit->SignU * Solution.SharedRadiusU,
			SideB.OffsetV - SideB.Fit->SignV * Solution.SharedRadiusV);
		const double MeanSquaredA = MeasureSide(SideA, Solution.SharedRadiusU,
			Solution.SharedRadiusV, Solution.SharedFlatteningFraction,
			&Solution.RegionAMaximumResidualCm);
		const double MeanSquaredB = MeasureSide(SideB, Solution.SharedRadiusU,
			Solution.SharedRadiusV, Solution.SharedFlatteningFraction,
			&Solution.RegionBMaximumResidualCm);
		Solution.RegionARootMeanSquareResidualCm = FMath::Sqrt(MeanSquaredA);
		Solution.RegionBRootMeanSquareResidualCm = FMath::Sqrt(MeanSquaredB);
		Solution.BalancedRootMeanSquareResidualCm = FMath::Sqrt(
			0.5 * (MeanSquaredA + MeanSquaredB));
		Solution.MaximumPlaneOrthogonalityResidual = FMath::Max(
			SideA.OrthogonalityResidual, SideB.OrthogonalityResidual);
		for (const FSideData* Side : { &SideA, &SideB })
		{
			FC2TransitionSectionFit Probe;
			Probe.CenterCoordinates = Side == &SideA
				? Solution.RegionACenterCoordinates
				: Solution.RegionBCenterCoordinates;
			Probe.RadiusU = Solution.SharedRadiusU;
			Probe.RadiusV = Solution.SharedRadiusV;
			Probe.SignU = Side->Fit->SignU;
			Probe.SignV = Side->Fit->SignV;
			Probe.FlatteningFraction = Solution.SharedFlatteningFraction;
			const FVector2d Start = Probe.EvaluatePosition(0.0);
			const FVector2d End = Probe.EvaluatePosition(1.0);
			const FVector3d WorldStart =
				Start.X * Side->DualU + Start.Y * Side->DualV;
			const FVector3d WorldEnd =
				End.X * Side->DualU + End.Y * Side->DualV;
			Solution.MaximumEndpointPlaneResidualCm = FMath::Max(
				Solution.MaximumEndpointPlaneResidualCm,
				FMath::Max(FMath::Abs(FVector3d::DotProduct(
					Side->NormalV, WorldStart) - Side->OffsetV),
					FMath::Abs(FVector3d::DotProduct(
						Side->NormalU, WorldEnd) - Side->OffsetU)));
			const FVector2d StartDerivative = Probe.EvaluateFirstDerivative(0.0);
			const FVector2d EndDerivative = Probe.EvaluateFirstDerivative(1.0);
			const FVector3d WorldStartDerivative =
				StartDerivative.X * Side->DualU +
				StartDerivative.Y * Side->DualV;
			const FVector3d WorldEndDerivative =
				EndDerivative.X * Side->DualU +
				EndDerivative.Y * Side->DualV;
			Solution.MaximumEndpointTangentPlaneResidual = FMath::Max(
				Solution.MaximumEndpointTangentPlaneResidual,
				FMath::Max(FMath::Abs(FVector3d::DotProduct(Side->NormalV,
					WorldStartDerivative)) /
						FMath::Max(WorldStartDerivative.Length(), 1.0e-12),
					FMath::Abs(FVector3d::DotProduct(Side->NormalU,
						WorldEndDerivative)) /
						FMath::Max(WorldEndDerivative.Length(), 1.0e-12)));
			for (const double T : { 0.0, 1.0 })
			{
				const FVector2d Second = Probe.EvaluateSecondDerivative(T);
				Solution.MaximumEndpointSecondDerivativeMagnitude = FMath::Max(
					Solution.MaximumEndpointSecondDerivativeMagnitude,
					(Second.X * Side->DualU + Second.Y * Side->DualV).Length());
			}
		}
		Solution.bExactC0G1C2ByConstruction =
			Solution.MaximumEndpointPlaneResidualCm <= 1.0e-9 &&
			Solution.MaximumEndpointTangentPlaneResidual <= 1.0e-12 &&
			Solution.MaximumEndpointSecondDerivativeMagnitude <= 1.0e-9;
		const double MinimumRadius = FMath::Min(
			Solution.SharedRadiusU, Solution.SharedRadiusV);
		Solution.bSourceFitPlausible = Solution.bExactC0G1C2ByConstruction &&
			Solution.RegionARootMeanSquareResidualCm <= 0.01 * MinimumRadius &&
			Solution.RegionBRootMeanSquareResidualCm <= 0.01 * MinimumRadius &&
			Solution.RegionAMaximumResidualCm <= 0.02 * MinimumRadius &&
			Solution.RegionBMaximumResidualCm <= 0.02 * MinimumRadius;
	}

	// Pair constraints form a graph because one physical transition can be
	// mirrored across both X and Y. Collapse that graph before accepting shared
	// shape parameters so a region never receives two different pair solutions.
	TArray<int32> Parents;
	Parents.SetNumUninitialized(ExtrusionSurfaceRegions.Num());
	TArray<bool> ActiveRegions;
	ActiveRegions.Init(false, ExtrusionSurfaceRegions.Num());
	for (int32 RegionIndex = 0; RegionIndex < Parents.Num(); ++RegionIndex)
	{
		Parents[RegionIndex] = RegionIndex;
	}
	auto FindRoot = [&](int32 RegionIndex)
	{
		while (Parents[RegionIndex] != RegionIndex)
		{
			Parents[RegionIndex] = Parents[Parents[RegionIndex]];
			RegionIndex = Parents[RegionIndex];
		}
		return RegionIndex;
	};
	for (const FCoupledC2TransitionPairSolution& Solution :
		CoupledC2TransitionSolutions)
	{
		const FSharedC2TransitionPairFit& Pair =
			SharedC2TransitionPairFits[Solution.SharedPairFitIndex];
		ActiveRegions[Pair.RegionAIndex] = true;
		ActiveRegions[Pair.RegionBIndex] = true;
		const int32 RootA = FindRoot(Pair.RegionAIndex);
		const int32 RootB = FindRoot(Pair.RegionBIndex);
		if (RootA != RootB)
		{
			const uint64 IdA = ExtrusionSurfaceRegions[RootA].RegionId;
			const uint64 IdB = ExtrusionSurfaceRegions[RootB].RegionId;
			if (IdA < IdB) Parents[RootB] = RootA;
			else Parents[RootA] = RootB;
		}
	}
	for (int32 RegionIndex = 0; RegionIndex < Parents.Num(); ++RegionIndex)
	{
		if (ActiveRegions[RegionIndex]) Parents[RegionIndex] = FindRoot(RegionIndex);
	}
	TArray<int32> Roots;
	for (int32 RegionIndex = 0; RegionIndex < Parents.Num(); ++RegionIndex)
	{
		if (ActiveRegions[RegionIndex] && Parents[RegionIndex] == RegionIndex)
		{
			Roots.Add(RegionIndex);
		}
	}
	Algo::Sort(Roots, [&](const int32 A, const int32 B)
	{
		return ExtrusionSurfaceRegions[A].RegionId <
			ExtrusionSurfaceRegions[B].RegionId;
	});
	for (const int32 Root : Roots)
	{
		TArray<int32> Members;
		for (int32 RegionIndex = 0; RegionIndex < Parents.Num(); ++RegionIndex)
		{
			if (ActiveRegions[RegionIndex] && Parents[RegionIndex] == Root)
			{
				Members.Add(RegionIndex);
			}
		}
		Algo::Sort(Members, [&](const int32 A, const int32 B)
		{
			return ExtrusionSurfaceRegions[A].RegionId <
				ExtrusionSurfaceRegions[B].RegionId;
		});
		TArray<FSideData> Sides;
		for (const int32 RegionIndex : Members)
		{
			FSideData Side = BuildSideData(RegionIndex);
			if (Side.bValid) Sides.Add(MoveTemp(Side));
		}
		if (Sides.Num() != Members.Num()) continue;
		FCoupledC2TransitionFamilySolution& Family =
			CoupledC2TransitionFamilies.AddDefaulted_GetRef();
		Family.FamilyId = ExtrusionSurfaceRegions[Members[0]].RegionId;
		Family.FirstRegionIndex = CoupledC2FamilyRegionIndices.Num();
		Family.RegionCount = Members.Num();
		CoupledC2FamilyRegionIndices.Append(Members);
		for (const FCoupledC2TransitionPairSolution& PairSolution :
			CoupledC2TransitionSolutions)
		{
			const FSharedC2TransitionPairFit& Pair =
				SharedC2TransitionPairFits[PairSolution.SharedPairFitIndex];
			if (Parents[Pair.RegionAIndex] != Root) continue;
			++Family.PairConstraintCount;
			Family.SymmetryAxisMask |= Pair.SymmetryAxis == EPlanarSymmetryAxis::X
				? 1u : 2u;
			Family.SharedRadiusU += PairSolution.SharedRadiusU;
			Family.SharedRadiusV += PairSolution.SharedRadiusV;
			Family.SharedFlatteningFraction +=
				PairSolution.SharedFlatteningFraction;
		}
		Family.SharedRadiusU /= static_cast<double>(Family.PairConstraintCount);
		Family.SharedRadiusV /= static_cast<double>(Family.PairConstraintCount);
		Family.SharedFlatteningFraction /=
			static_cast<double>(Family.PairConstraintCount);
		auto Objective = [&](const double RadiusU, const double RadiusV,
			const double FlatteningFraction)
		{
			double Result = 0.0;
			for (const FSideData& Side : Sides)
			{
				Result += MeasureSide(Side, RadiusU, RadiusV,
					FlatteningFraction, nullptr);
			}
			return Result / static_cast<double>(Sides.Num());
		};
		double Steps[3] = { 0.02 * Family.SharedRadiusU,
			0.02 * Family.SharedRadiusV, 0.01 };
		double BestObjective = Objective(Family.SharedRadiusU,
			Family.SharedRadiusV, Family.SharedFlatteningFraction);
		for (int32 Iteration = 0; Iteration < 14; ++Iteration)
		{
			double BestRadiusU = Family.SharedRadiusU;
			double BestRadiusV = Family.SharedRadiusV;
			double BestFlattening = Family.SharedFlatteningFraction;
			for (int32 DirectionU = -1; DirectionU <= 1; ++DirectionU)
			{
				for (int32 DirectionV = -1; DirectionV <= 1; ++DirectionV)
				{
					for (int32 DirectionF = -1; DirectionF <= 1; ++DirectionF)
					{
						if (DirectionU == 0 && DirectionV == 0 && DirectionF == 0)
						{
							continue;
						}
						const double CandidateRadiusU = FMath::Max(1.0,
							Family.SharedRadiusU + DirectionU * Steps[0]);
						const double CandidateRadiusV = FMath::Max(1.0,
							Family.SharedRadiusV + DirectionV * Steps[1]);
						const double CandidateFlattening = FMath::Clamp(
							Family.SharedFlatteningFraction + DirectionF * Steps[2],
							0.01, 0.49);
						const double CandidateObjective = Objective(CandidateRadiusU,
							CandidateRadiusV, CandidateFlattening);
						if (CandidateObjective < BestObjective)
						{
							BestObjective = CandidateObjective;
							BestRadiusU = CandidateRadiusU;
							BestRadiusV = CandidateRadiusV;
							BestFlattening = CandidateFlattening;
						}
					}
				}
			}
			Family.SharedRadiusU = BestRadiusU;
			Family.SharedRadiusV = BestRadiusV;
			Family.SharedFlatteningFraction = BestFlattening;
			for (double& Step : Steps) Step *= 0.5;
		}
		Family.BalancedRootMeanSquareResidualCm = FMath::Sqrt(BestObjective);
		for (const FSideData& Side : Sides)
		{
			double SideMaximumResidual = 0.0;
			const double SideMeanSquaredResidual = MeasureSide(Side,
				Family.SharedRadiusU, Family.SharedRadiusV,
				Family.SharedFlatteningFraction, &SideMaximumResidual);
			Family.MaximumRegionRootMeanSquareResidualCm = FMath::Max(
				Family.MaximumRegionRootMeanSquareResidualCm,
				FMath::Sqrt(SideMeanSquaredResidual));
			Family.MaximumResidualCm = FMath::Max(
				Family.MaximumResidualCm, SideMaximumResidual);
			FC2TransitionSectionFit Probe;
			Probe.CenterCoordinates = FVector2d(
				Side.OffsetU - Side.Fit->SignU * Family.SharedRadiusU,
				Side.OffsetV - Side.Fit->SignV * Family.SharedRadiusV);
			Probe.RadiusU = Family.SharedRadiusU;
			Probe.RadiusV = Family.SharedRadiusV;
			Probe.SignU = Side.Fit->SignU;
			Probe.SignV = Side.Fit->SignV;
			Probe.FlatteningFraction = Family.SharedFlatteningFraction;
			const FVector2d Start = Probe.EvaluatePosition(0.0);
			const FVector2d End = Probe.EvaluatePosition(1.0);
			const FVector3d WorldStart =
				Start.X * Side.DualU + Start.Y * Side.DualV;
			const FVector3d WorldEnd =
				End.X * Side.DualU + End.Y * Side.DualV;
			Family.MaximumEndpointPlaneResidualCm = FMath::Max(
				Family.MaximumEndpointPlaneResidualCm,
				FMath::Max(FMath::Abs(FVector3d::DotProduct(
					Side.NormalV, WorldStart) - Side.OffsetV),
					FMath::Abs(FVector3d::DotProduct(
						Side.NormalU, WorldEnd) - Side.OffsetU)));
			const FVector2d StartDerivative = Probe.EvaluateFirstDerivative(0.0);
			const FVector2d EndDerivative = Probe.EvaluateFirstDerivative(1.0);
			const FVector3d WorldStartDerivative =
				StartDerivative.X * Side.DualU +
				StartDerivative.Y * Side.DualV;
			const FVector3d WorldEndDerivative =
				EndDerivative.X * Side.DualU + EndDerivative.Y * Side.DualV;
			Family.MaximumEndpointTangentPlaneResidual = FMath::Max(
				Family.MaximumEndpointTangentPlaneResidual,
				FMath::Max(FMath::Abs(FVector3d::DotProduct(Side.NormalV,
					WorldStartDerivative)) /
						FMath::Max(WorldStartDerivative.Length(), 1.0e-12),
					FMath::Abs(FVector3d::DotProduct(Side.NormalU,
						WorldEndDerivative)) /
						FMath::Max(WorldEndDerivative.Length(), 1.0e-12)));
			for (const double T : { 0.0, 1.0 })
			{
				const FVector2d Second = Probe.EvaluateSecondDerivative(T);
				Family.MaximumEndpointSecondDerivativeMagnitude = FMath::Max(
					Family.MaximumEndpointSecondDerivativeMagnitude,
					(Second.X * Side.DualU + Second.Y * Side.DualV).Length());
			}
		}
		Family.bExactC0G1C2ByConstruction =
			Family.MaximumEndpointPlaneResidualCm <= 1.0e-9 &&
			Family.MaximumEndpointTangentPlaneResidual <= 1.0e-12 &&
			Family.MaximumEndpointSecondDerivativeMagnitude <= 1.0e-9;
		const double MinimumRadius = FMath::Min(
			Family.SharedRadiusU, Family.SharedRadiusV);
		Family.bSourceFitPlausible = Family.bExactC0G1C2ByConstruction &&
			Family.MaximumRegionRootMeanSquareResidualCm <= 0.01 * MinimumRadius &&
			Family.MaximumResidualCm <= 0.02 * MinimumRadius;
	}
}

void FAnalyticWorldData::BuildC2TransitionCoverage()
{
	C2TransitionCoverage.Reset();
	TArray<int32> FamilyIndexByRegion;
	FamilyIndexByRegion.Init(INDEX_NONE, ExtrusionSurfaceRegions.Num());
	for (int32 FamilyIndex = 0;
		FamilyIndex < CoupledC2TransitionFamilies.Num(); ++FamilyIndex)
	{
		const FCoupledC2TransitionFamilySolution& Family =
			CoupledC2TransitionFamilies[FamilyIndex];
		for (int32 Offset = 0; Offset < Family.RegionCount; ++Offset)
		{
			const int32 RegionIndex = CoupledC2FamilyRegionIndices[
				Family.FirstRegionIndex + Offset];
			if (FamilyIndexByRegion.IsValidIndex(RegionIndex))
			{
				FamilyIndexByRegion[RegionIndex] = FamilyIndex;
			}
		}
	}
	const int32 RegionCount = ExtrusionSurfaceRegions.Num();
	TArray<int32> HorizontalPlaneIndices;
	for (const FC2TransitionSectionFit& Fit : C2TransitionSectionFits)
	{
		if (!Fit.bBoundaryEvidenceUsable) continue;
		for (const int32 MatchIndex : {
			Fit.BoundaryMatchUIndex, Fit.BoundaryMatchVIndex })
		{
			if (!QuarterEllipseBoundaryMatches.IsValidIndex(MatchIndex)) continue;
			const int32 PlaneIndex =
				QuarterEllipseBoundaryMatches[MatchIndex].PlanarGroupIndex;
			if (!PlanarSurfaceGroups.IsValidIndex(PlaneIndex)) continue;
			const FPlanarSurfaceGroup& Plane = PlanarSurfaceGroups[PlaneIndex];
			if (FMath::Abs(Plane.Normal.Z) >= 0.9)
			{
				HorizontalPlaneIndices.AddUnique(PlaneIndex);
			}
		}
	}
	Algo::Sort(HorizontalPlaneIndices, [&](const int32 A, const int32 B)
	{
		const FPlanarSurfaceGroup& PlaneA = PlanarSurfaceGroups[A];
		const FPlanarSurfaceGroup& PlaneB = PlanarSurfaceGroups[B];
		return PlaneA.PlaneOffset / PlaneA.Normal.Z <
			PlaneB.PlaneOffset / PlaneB.Normal.Z;
	});
	for (int32 FitIndex = 0; FitIndex < C2TransitionSectionFits.Num(); ++FitIndex)
	{
		const FC2TransitionSectionFit& Fit = C2TransitionSectionFits[FitIndex];
		if (!Fit.bBoundaryEvidenceUsable ||
			!ExtrusionSurfaceRegions.IsValidIndex(Fit.ExtrusionRegionIndex))
		{
			continue;
		}
		FC2TransitionCoverageEntry& Entry =
			C2TransitionCoverage.AddDefaulted_GetRef();
		Entry.TransitionFitIndex = FitIndex;
		Entry.ExtrusionRegionIndex = Fit.ExtrusionRegionIndex;
		Entry.FamilyIndex = FamilyIndexByRegion[Fit.ExtrusionRegionIndex];
		const int32 XMatchIndex = Fit.ExtrusionRegionIndex;
		const int32 YMatchIndex = RegionCount + Fit.ExtrusionRegionIndex;
		if (ExtrusionRegionMirrorMatches.IsValidIndex(XMatchIndex))
		{
			const FExtrusionRegionMirrorMatch& Match =
				ExtrusionRegionMirrorMatches[XMatchIndex];
			Entry.XMirrorTargetRegionIndex = Match.TargetRegionIndex;
			Entry.bXMirrorPlausible = Match.bPlausible;
			Entry.bXSelfMirror = Match.bSelfMirror;
		}
		if (ExtrusionRegionMirrorMatches.IsValidIndex(YMatchIndex))
		{
			const FExtrusionRegionMirrorMatch& Match =
				ExtrusionRegionMirrorMatches[YMatchIndex];
			Entry.YMirrorTargetRegionIndex = Match.TargetRegionIndex;
			Entry.bYMirrorPlausible = Match.bPlausible;
			Entry.bYSelfMirror = Match.bSelfMirror;
		}
		for (const int32 MatchIndex : {
			Fit.BoundaryMatchUIndex, Fit.BoundaryMatchVIndex })
		{
			if (!QuarterEllipseBoundaryMatches.IsValidIndex(MatchIndex)) continue;
			const FQuarterEllipseBoundaryMatch& Match =
				QuarterEllipseBoundaryMatches[MatchIndex];
			Entry.MaximumBoundaryPositionResidualCm = FMath::Max(
				Entry.MaximumBoundaryPositionResidualCm,
				Match.PositionResidualCm);
			Entry.MaximumBoundaryNormalResidualDegrees = FMath::Max(
				Entry.MaximumBoundaryNormalResidualDegrees,
				Match.NormalAngleResidualDegrees);
			if (PlanarSurfaceGroups.IsValidIndex(Match.PlanarGroupIndex))
			{
				const FPlanarSurfaceGroup& Plane =
					PlanarSurfaceGroups[Match.PlanarGroupIndex];
				if (FMath::Abs(Plane.Normal.Z) >= 0.9)
				{
					Entry.HorizontalPlanarGroupIndex = Match.PlanarGroupIndex;
					Entry.HorizontalPlaneHeightCm =
						Plane.PlaneOffset / Plane.Normal.Z;
				}
			}
		}
		if (Entry.HorizontalPlanarGroupIndex != INDEX_NONE &&
			HorizontalPlaneIndices.Num() >= 4)
		{
			const int32 HorizontalRank = HorizontalPlaneIndices.IndexOfByKey(
				Entry.HorizontalPlanarGroupIndex);
			Entry.SurfaceLayer = HorizontalRank == 0 ||
				HorizontalRank == HorizontalPlaneIndices.Num() - 1
				? EC2TransitionSurfaceLayer::OuterBacking
				: EC2TransitionSurfaceLayer::PlayableInner;
		}
		const FVector3d& Axis =
			ExtrusionSurfaceRegions[Fit.ExtrusionRegionIndex].Axis;
		if (Entry.FamilyIndex != INDEX_NONE)
		{
			Entry.Kind = EC2TransitionCoverageKind::CoveredFamily;
		}
		else if ((Entry.bXMirrorPlausible && Entry.bXSelfMirror) ||
			(Entry.bYMirrorPlausible && Entry.bYSelfMirror))
		{
			Entry.Kind = EC2TransitionCoverageKind::AxisAlignedSelfMirror;
		}
		else if (FMath::Abs(Axis.X) >= 0.25 &&
			FMath::Abs(Axis.Y) >= 0.25)
		{
			Entry.Kind = EC2TransitionCoverageKind::DiagonalCornerCandidate;
		}
		else if (FMath::Abs(Axis.X) >= 0.9 || FMath::Abs(Axis.Y) >= 0.9)
		{
			Entry.Kind = EC2TransitionCoverageKind::UnpairedAxisAligned;
		}
	}
}

void FAnalyticWorldData::BuildPlayableC2OrbitCandidates()
{
	PlayableC2OrbitCandidates.Reset();
	PlayableC2OrbitMembers.Reset();
	const int32 RegionCount = ExtrusionSurfaceRegions.Num();
	auto MatchFor = [&](const int32 RegionIndex,
		const EPlanarSymmetryAxis Axis) -> const FExtrusionRegionMirrorMatch*
	{
		const int32 MatchIndex = RegionIndex +
			(Axis == EPlanarSymmetryAxis::Y ? RegionCount : 0);
		return ExtrusionRegionMirrorMatches.IsValidIndex(MatchIndex)
			? &ExtrusionRegionMirrorMatches[MatchIndex] : nullptr;
	};
	auto LooseMirrorPlausible = [&](const FExtrusionRegionMirrorMatch* Match)
	{
		if (!Match || !Match->bReciprocal ||
			!ExtrusionSurfaceRegions.IsValidIndex(Match->SourceRegionIndex) ||
			!ExtrusionSurfaceRegions.IsValidIndex(Match->TargetRegionIndex))
		{
			return false;
		}
		const FExtrusionSurfaceRegion& Source =
			ExtrusionSurfaceRegions[Match->SourceRegionIndex];
		const FExtrusionSurfaceRegion& Target =
			ExtrusionSurfaceRegions[Match->TargetRegionIndex];
		return Source.SourceId == Target.SourceId &&
			Source.SurfaceId == Target.SurfaceId &&
			Source.MaterialId == Target.MaterialId &&
			Match->AxisAngleResidualDegrees <= 1.0 &&
			Match->CenterLineResidualCm <= 25.0 &&
			Match->RelativeRadiusUResidual <= 0.05 &&
			Match->RelativeRadiusVResidual <= 0.05 &&
			Match->BoundsResidualCm <= 25.0 &&
			Match->RelativeAreaResidual <= 0.10;
	};
	auto ReflectPoint = [](FVector3d Point, const uint8 TransformMask)
	{
		if ((TransformMask & 1u) != 0) Point.X *= -1.0;
		if ((TransformMask & 2u) != 0) Point.Y *= -1.0;
		return Point;
	};

	for (int32 CoverageIndex = 0;
		CoverageIndex < C2TransitionCoverage.Num(); ++CoverageIndex)
	{
		const FC2TransitionCoverageEntry& Coverage =
			C2TransitionCoverage[CoverageIndex];
		if (Coverage.SurfaceLayer != EC2TransitionSurfaceLayer::PlayableInner ||
			Coverage.FamilyIndex != INDEX_NONE ||
			!C2TransitionSectionFits.IsValidIndex(Coverage.TransitionFitIndex) ||
			!ExtrusionSurfaceRegions.IsValidIndex(Coverage.ExtrusionRegionIndex))
		{
			continue;
		}
		const FC2TransitionSectionFit& SeedFit =
			C2TransitionSectionFits[Coverage.TransitionFitIndex];
		const int32 SeedRegionIndex = Coverage.ExtrusionRegionIndex;
		const FExtrusionSurfaceRegion& SeedRegion =
			ExtrusionSurfaceRegions[SeedRegionIndex];
		FPlayableC2OrbitCandidate& Candidate =
			PlayableC2OrbitCandidates.AddDefaulted_GetRef();
		Candidate.OrbitId = CombineStableIds(SeedRegion.RegionId,
			StableStringId(TEXT("PlayableC2Orbit")));
		Candidate.SeedCoverageIndex = CoverageIndex;
		Candidate.FirstMemberIndex = PlayableC2OrbitMembers.Num();

		TArray<uint8> DesiredMasks = { 0u };
		if (Coverage.bXMirrorPlausible && Coverage.bXSelfMirror)
		{
			DesiredMasks.Add(2u);
			Candidate.SymmetryAxisMask = 2u;
		}
		else if (Coverage.bYMirrorPlausible && Coverage.bYSelfMirror)
		{
			DesiredMasks.Add(1u);
			Candidate.SymmetryAxisMask = 1u;
		}
		else if (Coverage.Kind ==
			EC2TransitionCoverageKind::DiagonalCornerCandidate)
		{
			DesiredMasks = { 0u, 1u, 2u, 3u };
			Candidate.SymmetryAxisMask = 3u;
		}
		else if (Coverage.XMirrorTargetRegionIndex != INDEX_NONE)
		{
			DesiredMasks.Add(1u);
			Candidate.SymmetryAxisMask = 1u;
		}
		else if (Coverage.YMirrorTargetRegionIndex != INDEX_NONE)
		{
			DesiredMasks.Add(2u);
			Candidate.SymmetryAxisMask = 2u;
		}
		Candidate.ExpectedMemberCount = DesiredMasks.Num();
		bool bAllEdgesPlausible = true;
		bool bClosureConsistent = true;
		TSet<int32> MemberRegions;
		TArray<TArray<FVector2d>> MemberPoints;
		for (const uint8 TransformMask : DesiredMasks)
		{
			int32 RegionIndex = SeedRegionIndex;
			TArray<const FExtrusionRegionMirrorMatch*> Path;
			if (TransformMask == 1u || TransformMask == 2u)
			{
				const EPlanarSymmetryAxis Axis = TransformMask == 1u
					? EPlanarSymmetryAxis::X : EPlanarSymmetryAxis::Y;
				const FExtrusionRegionMirrorMatch* Match =
					MatchFor(SeedRegionIndex, Axis);
				Path.Add(Match);
				RegionIndex = Match ? Match->TargetRegionIndex : INDEX_NONE;
			}
			else if (TransformMask == 3u)
			{
				const FExtrusionRegionMirrorMatch* SeedX =
					MatchFor(SeedRegionIndex, EPlanarSymmetryAxis::X);
				const FExtrusionRegionMirrorMatch* SeedY =
					MatchFor(SeedRegionIndex, EPlanarSymmetryAxis::Y);
				const FExtrusionRegionMirrorMatch* XY = SeedX
					? MatchFor(SeedX->TargetRegionIndex,
						EPlanarSymmetryAxis::Y) : nullptr;
				const FExtrusionRegionMirrorMatch* YX = SeedY
					? MatchFor(SeedY->TargetRegionIndex,
						EPlanarSymmetryAxis::X) : nullptr;
				const int32 XYRegion = XY ? XY->TargetRegionIndex : INDEX_NONE;
				const int32 YXRegion = YX ? YX->TargetRegionIndex : INDEX_NONE;
				bClosureConsistent &= XYRegion != INDEX_NONE && XYRegion == YXRegion;
				const bool bUseXY = XYRegion != INDEX_NONE &&
					(YXRegion == INDEX_NONE ||
						(SeedX ? SeedX->Score : TNumericLimits<double>::Max()) +
						XY->Score <=
						(SeedY ? SeedY->Score : TNumericLimits<double>::Max()) +
						YX->Score);
				if (bUseXY)
				{
					Path = { SeedX, XY };
					RegionIndex = XYRegion;
				}
				else
				{
					Path = { SeedY, YX };
					RegionIndex = YXRegion;
				}
			}
			for (const FExtrusionRegionMirrorMatch* Match : Path)
			{
				bAllEdgesPlausible &= LooseMirrorPlausible(Match);
				if (!Match) continue;
				Candidate.MaximumMirrorAxisResidualDegrees = FMath::Max(
					Candidate.MaximumMirrorAxisResidualDegrees,
					Match->AxisAngleResidualDegrees);
				Candidate.MaximumMirrorCenterLineResidualCm = FMath::Max(
					Candidate.MaximumMirrorCenterLineResidualCm,
					Match->CenterLineResidualCm);
				Candidate.MaximumMirrorBoundsResidualCm = FMath::Max(
					Candidate.MaximumMirrorBoundsResidualCm,
					Match->BoundsResidualCm);
				Candidate.MaximumMirrorRelativeAreaResidual = FMath::Max(
					Candidate.MaximumMirrorRelativeAreaResidual,
					Match->RelativeAreaResidual);
			}
			if (!ExtrusionSurfaceRegions.IsValidIndex(RegionIndex) ||
				MemberRegions.Contains(RegionIndex))
			{
				bClosureConsistent = false;
				continue;
			}
			MemberRegions.Add(RegionIndex);
			FPlayableC2OrbitMember& Member =
				PlayableC2OrbitMembers.AddDefaulted_GetRef();
			Member.ExtrusionRegionIndex = RegionIndex;
			Member.TransformMaskFromSeed = TransformMask;
		}
		Candidate.MemberCount = PlayableC2OrbitMembers.Num() -
			Candidate.FirstMemberIndex;
		Candidate.bOrbitComplete = bClosureConsistent &&
			Candidate.MemberCount == Candidate.ExpectedMemberCount;
		Candidate.bMirrorTopologyPlausible = Candidate.bOrbitComplete &&
			bAllEdgesPlausible;
		if (!Candidate.bMirrorTopologyPlausible ||
			!QuarterEllipseBoundaryMatches.IsValidIndex(
				SeedFit.BoundaryMatchUIndex) ||
			!QuarterEllipseBoundaryMatches.IsValidIndex(
				SeedFit.BoundaryMatchVIndex))
		{
			continue;
		}

		const int32 PlaneUIndex = QuarterEllipseBoundaryMatches[
			SeedFit.BoundaryMatchUIndex].PlanarGroupIndex;
		const int32 PlaneVIndex = QuarterEllipseBoundaryMatches[
			SeedFit.BoundaryMatchVIndex].PlanarGroupIndex;
		if (!PlanarSurfaceGroups.IsValidIndex(PlaneUIndex) ||
			!PlanarSurfaceGroups.IsValidIndex(PlaneVIndex))
		{
			continue;
		}
		FVector3d NormalU = PlanarSurfaceGroups[PlaneUIndex].Normal;
		FVector3d NormalV = PlanarSurfaceGroups[PlaneVIndex].Normal;
		double OffsetU = PlanarSurfaceGroups[PlaneUIndex].PlaneOffset;
		double OffsetV = PlanarSurfaceGroups[PlaneVIndex].PlaneOffset;
		if (FVector3d::DotProduct(NormalU, SeedRegion.SectionAxisU) < 0.0)
		{
			NormalU *= -1.0;
			OffsetU *= -1.0;
		}
		if (FVector3d::DotProduct(NormalV, SeedRegion.SectionAxisV) < 0.0)
		{
			NormalV *= -1.0;
			OffsetV *= -1.0;
		}
		const double Dot = FVector3d::DotProduct(NormalU, NormalV);
		const double Denominator = 1.0 - Dot * Dot;
		if (Denominator <= 1.0e-8) continue;
		const FVector3d DualU = (NormalU - Dot * NormalV) / Denominator;
		const FVector3d DualV = (NormalV - Dot * NormalU) / Denominator;
		for (int32 MemberOffset = 0;
			MemberOffset < Candidate.MemberCount; ++MemberOffset)
		{
			const FPlayableC2OrbitMember& Member = PlayableC2OrbitMembers[
				Candidate.FirstMemberIndex + MemberOffset];
			const FExtrusionSurfaceRegion& Region =
				ExtrusionSurfaceRegions[Member.ExtrusionRegionIndex];
			TSet<int32> UniqueVertexSet;
			for (int32 TriangleOffset = 0;
				TriangleOffset < Region.TriangleCount; ++TriangleOffset)
			{
				const int32 TriangleIndex = ExtrusionRegionTriangleIndices[
					Region.FirstTriangleIndex + TriangleOffset];
				for (int32 Corner = 0; Corner < 3; ++Corner)
				{
					UniqueVertexSet.Add(TriangleVertexIndices[TriangleIndex][Corner]);
				}
			}
			TArray<int32> UniqueVertices = UniqueVertexSet.Array();
			Algo::Sort(UniqueVertices);
			TArray<FVector2d>& Points = MemberPoints.AddDefaulted_GetRef();
			Points.Reserve(UniqueVertices.Num());
			for (const int32 VertexIndex : UniqueVertices)
			{
				const FVector3d Position = ReflectPoint(
					MeshVertices[VertexIndex], Member.TransformMaskFromSeed);
				Points.Emplace(FVector3d::DotProduct(NormalU, Position),
					FVector3d::DotProduct(NormalV, Position));
			}
		}
		if (MemberPoints.Num() != Candidate.MemberCount ||
			Algo::AnyOf(MemberPoints,
				[](const TArray<FVector2d>& Points) { return Points.IsEmpty(); }))
		{
			continue;
		}
		auto MeasurePoints = [&](const TArray<FVector2d>& Points,
			const double RadiusU, const double RadiusV,
			const double Flattening, double* OutMaximum)
		{
			const FVector2d Center(OffsetU - SeedFit.SignU * RadiusU,
				OffsetV - SeedFit.SignV * RadiusV);
			double SquaredSum = 0.0;
			double Maximum = 0.0;
			for (const FVector2d& Point : Points)
			{
				const double Squared = SquaredDistanceToQuinticTransitionInDualBasis(
					Point, Center, RadiusU, RadiusV, SeedFit.SignU,
					SeedFit.SignV, Flattening, DualU, DualV);
				SquaredSum += Squared;
				Maximum = FMath::Max(Maximum, FMath::Sqrt(Squared));
			}
			if (OutMaximum) *OutMaximum = Maximum;
			return SquaredSum / static_cast<double>(Points.Num());
		};
		auto Objective = [&](const double RadiusU, const double RadiusV,
			const double Flattening)
		{
			double Sum = 0.0;
			for (const TArray<FVector2d>& Points : MemberPoints)
			{
				Sum += MeasurePoints(Points, RadiusU, RadiusV,
					Flattening, nullptr);
			}
			return Sum / static_cast<double>(MemberPoints.Num());
		};
		Candidate.SharedRadiusU = SeedFit.RadiusU;
		Candidate.SharedRadiusV = SeedFit.RadiusV;
		Candidate.SharedFlatteningFraction = SeedFit.FlatteningFraction;
		double Steps[3] = { 0.02 * Candidate.SharedRadiusU,
			0.02 * Candidate.SharedRadiusV, 0.01 };
		double BestObjective = Objective(Candidate.SharedRadiusU,
			Candidate.SharedRadiusV, Candidate.SharedFlatteningFraction);
		for (int32 Iteration = 0; Iteration < 14; ++Iteration)
		{
			double BestU = Candidate.SharedRadiusU;
			double BestV = Candidate.SharedRadiusV;
			double BestF = Candidate.SharedFlatteningFraction;
			for (int32 DU = -1; DU <= 1; ++DU)
			for (int32 DV = -1; DV <= 1; ++DV)
			for (int32 DF = -1; DF <= 1; ++DF)
			{
				if (DU == 0 && DV == 0 && DF == 0) continue;
				const double U = FMath::Max(1.0,
					Candidate.SharedRadiusU + DU * Steps[0]);
				const double V = FMath::Max(1.0,
					Candidate.SharedRadiusV + DV * Steps[1]);
				const double F = FMath::Clamp(
					Candidate.SharedFlatteningFraction + DF * Steps[2],
					0.01, 0.49);
				const double Value = Objective(U, V, F);
				if (Value < BestObjective)
				{
					BestObjective = Value;
					BestU = U;
					BestV = V;
					BestF = F;
				}
			}
			Candidate.SharedRadiusU = BestU;
			Candidate.SharedRadiusV = BestV;
			Candidate.SharedFlatteningFraction = BestF;
			for (double& Step : Steps) Step *= 0.5;
		}
		Candidate.BalancedRootMeanSquareResidualCm = FMath::Sqrt(BestObjective);
		for (int32 MemberOffset = 0;
			MemberOffset < Candidate.MemberCount; ++MemberOffset)
		{
			FPlayableC2OrbitMember& Member = PlayableC2OrbitMembers[
				Candidate.FirstMemberIndex + MemberOffset];
			double Maximum = 0.0;
			const double MeanSquared = MeasurePoints(MemberPoints[MemberOffset],
				Candidate.SharedRadiusU, Candidate.SharedRadiusV,
				Candidate.SharedFlatteningFraction, &Maximum);
			Member.RootMeanSquareResidualCm = FMath::Sqrt(MeanSquared);
			Member.MaximumResidualCm = Maximum;
			Candidate.MaximumRegionRootMeanSquareResidualCm = FMath::Max(
				Candidate.MaximumRegionRootMeanSquareResidualCm,
				Member.RootMeanSquareResidualCm);
			Candidate.MaximumResidualCm = FMath::Max(
				Candidate.MaximumResidualCm, Maximum);
		}
		FC2TransitionSectionFit Probe;
		Probe.CenterCoordinates = FVector2d(
			OffsetU - SeedFit.SignU * Candidate.SharedRadiusU,
			OffsetV - SeedFit.SignV * Candidate.SharedRadiusV);
		Probe.RadiusU = Candidate.SharedRadiusU;
		Probe.RadiusV = Candidate.SharedRadiusV;
		Probe.SignU = SeedFit.SignU;
		Probe.SignV = SeedFit.SignV;
		Probe.FlatteningFraction = Candidate.SharedFlatteningFraction;
		const FVector2d Start = Probe.EvaluatePosition(0.0);
		const FVector2d End = Probe.EvaluatePosition(1.0);
		const FVector3d WorldStart = Start.X * DualU + Start.Y * DualV;
		const FVector3d WorldEnd = End.X * DualU + End.Y * DualV;
		Candidate.MaximumEndpointPlaneResidualCm = FMath::Max(
			FMath::Abs(FVector3d::DotProduct(NormalV, WorldStart) - OffsetV),
			FMath::Abs(FVector3d::DotProduct(NormalU, WorldEnd) - OffsetU));
		const FVector2d StartDerivative = Probe.EvaluateFirstDerivative(0.0);
		const FVector2d EndDerivative = Probe.EvaluateFirstDerivative(1.0);
		const FVector3d WorldStartDerivative =
			StartDerivative.X * DualU + StartDerivative.Y * DualV;
		const FVector3d WorldEndDerivative =
			EndDerivative.X * DualU + EndDerivative.Y * DualV;
		Candidate.MaximumEndpointTangentPlaneResidual = FMath::Max(
			FMath::Abs(FVector3d::DotProduct(NormalV, WorldStartDerivative)) /
				FMath::Max(WorldStartDerivative.Length(), 1.0e-12),
			FMath::Abs(FVector3d::DotProduct(NormalU, WorldEndDerivative)) /
				FMath::Max(WorldEndDerivative.Length(), 1.0e-12));
		for (const double T : { 0.0, 1.0 })
		{
			const FVector2d Second = Probe.EvaluateSecondDerivative(T);
			Candidate.MaximumEndpointSecondDerivativeMagnitude = FMath::Max(
				Candidate.MaximumEndpointSecondDerivativeMagnitude,
				(Second.X * DualU + Second.Y * DualV).Length());
		}
		Candidate.bExactC0G1C2ByConstruction =
			Candidate.MaximumEndpointPlaneResidualCm <= 1.0e-9 &&
			Candidate.MaximumEndpointTangentPlaneResidual <= 1.0e-12 &&
			Candidate.MaximumEndpointSecondDerivativeMagnitude <= 1.0e-9;
		const double MinimumRadius = FMath::Min(
			Candidate.SharedRadiusU, Candidate.SharedRadiusV);
		Candidate.bSourceFitPlausible =
			Candidate.bExactC0G1C2ByConstruction &&
			Candidate.MaximumRegionRootMeanSquareResidualCm <=
				0.01 * MinimumRadius &&
			Candidate.MaximumResidualCm <= 0.02 * MinimumRadius;
	}
}

void FAnalyticWorldData::BuildPlayableC2TriangleSupportCandidates()
{
	PlayableC2TriangleSupportCandidates.Reset();
	PlayableC2TriangleSupportMembers.Reset();
	PlayableC2CanonicalSupportPoints.Reset();
	auto ReflectVector = [](FVector3d Vector, const uint8 TransformMask)
	{
		if ((TransformMask & 1u) != 0) Vector.X *= -1.0;
		if ((TransformMask & 2u) != 0) Vector.Y *= -1.0;
		return Vector;
	};
	auto InterpolatedNormal = [](const FTriangleSurface& Triangle,
		const FVector3d& Point)
	{
		const FVector3d V0 = Triangle.Vertices[1] - Triangle.Vertices[0];
		const FVector3d V1 = Triangle.Vertices[2] - Triangle.Vertices[0];
		const FVector3d V2 = Point - Triangle.Vertices[0];
		const double D00 = FVector3d::DotProduct(V0, V0);
		const double D01 = FVector3d::DotProduct(V0, V1);
		const double D11 = FVector3d::DotProduct(V1, V1);
		const double D20 = FVector3d::DotProduct(V2, V0);
		const double D21 = FVector3d::DotProduct(V2, V1);
		const double Denominator = D00 * D11 - D01 * D01;
		if (FMath::Abs(Denominator) <= 1.0e-18) return Triangle.FaceNormal;
		const double BaryB = (D11 * D20 - D01 * D21) / Denominator;
		const double BaryC = (D00 * D21 - D01 * D20) / Denominator;
		const double BaryA = 1.0 - BaryB - BaryC;
		return (BaryA * Triangle.VertexNormals[0] +
			BaryB * Triangle.VertexNormals[1] +
			BaryC * Triangle.VertexNormals[2]).GetSafeNormal();
	};

	for (const FPlayableC2OrbitCandidate& RegionOrbit :
		PlayableC2OrbitCandidates)
	{
		if (!C2TransitionCoverage.IsValidIndex(RegionOrbit.SeedCoverageIndex))
		{
			continue;
		}
		const FC2TransitionCoverageEntry& Coverage =
			C2TransitionCoverage[RegionOrbit.SeedCoverageIndex];
		if (!ExtrusionSurfaceRegions.IsValidIndex(Coverage.ExtrusionRegionIndex))
		{
			continue;
		}
		const FExtrusionSurfaceRegion& SeedRegion =
			ExtrusionSurfaceRegions[Coverage.ExtrusionRegionIndex];
		if (SeedRegion.TriangleCount <= 0) continue;
		const int32 SeedTriangleIndex = ExtrusionRegionTriangleIndices[
			SeedRegion.FirstTriangleIndex];
		if (!TriangleSmoothRegionIndices.IsValidIndex(SeedTriangleIndex)) continue;
		const int32 SmoothRegionIndex =
			TriangleSmoothRegionIndices[SeedTriangleIndex];

		TArray<FVector3d> VertexNormalSums;
		VertexNormalSums.Init(FVector3d::ZeroVector, MeshVertices.Num());
		TArray<int32> VertexNormalCounts;
		VertexNormalCounts.Init(0, MeshVertices.Num());
		TSet<int32> SeedVertexSet;
		for (int32 TriangleOffset = 0;
			TriangleOffset < SeedRegion.TriangleCount; ++TriangleOffset)
		{
			const int32 TriangleIndex = ExtrusionRegionTriangleIndices[
				SeedRegion.FirstTriangleIndex + TriangleOffset];
			const FTriangleSurface& Triangle = Triangles[TriangleIndex];
			for (int32 Corner = 0; Corner < 3; ++Corner)
			{
				const int32 VertexIndex = TriangleVertexIndices[TriangleIndex][Corner];
				SeedVertexSet.Add(VertexIndex);
				VertexNormalSums[VertexIndex] += Triangle.VertexNormals[Corner];
				++VertexNormalCounts[VertexIndex];
			}
		}
		TArray<int32> SeedVertices = SeedVertexSet.Array();
		Algo::Sort(SeedVertices);
		if (SeedVertices.IsEmpty()) continue;

		FPlayableC2TriangleSupportCandidate& Candidate =
			PlayableC2TriangleSupportCandidates.AddDefaulted_GetRef();
		Candidate.OrbitId = RegionOrbit.OrbitId;
		Candidate.SeedCoverageIndex = RegionOrbit.SeedCoverageIndex;
		Candidate.SmoothRegionIndex = SmoothRegionIndex;
		Candidate.FirstMemberIndex = PlayableC2TriangleSupportMembers.Num();
		TArray<uint8> DesiredMasks = { 0u };
		if (RegionOrbit.SymmetryAxisMask == 1u) DesiredMasks.Add(1u);
		else if (RegionOrbit.SymmetryAxisMask == 2u) DesiredMasks.Add(2u);
		else if (RegionOrbit.SymmetryAxisMask == 3u)
		{
			DesiredMasks = { 0u, 1u, 2u, 3u };
		}
		Candidate.ExpectedMemberCount = DesiredMasks.Num();
		TArray<TArray<FVector3d>> MemberCanonicalWorldPoints;

		for (const uint8 TransformMask : DesiredMasks)
		{
			FPlayableC2TriangleSupportMember& Member =
				PlayableC2TriangleSupportMembers.AddDefaulted_GetRef();
			Member.TransformMaskFromSeed = TransformMask;
			Member.FirstCanonicalPointIndex =
				PlayableC2CanonicalSupportPoints.Num();
			Member.SampleCount = SeedVertices.Num();
			TArray<FVector3d>& CanonicalPoints =
				MemberCanonicalWorldPoints.AddDefaulted_GetRef();
			CanonicalPoints.Reserve(SeedVertices.Num());
			if (TransformMask == 0u)
			{
				for (const int32 VertexIndex : SeedVertices)
				{
					CanonicalPoints.Add(MeshVertices[VertexIndex]);
				}
				Member.MatchedSampleCount = Member.SampleCount;
				Member.UniqueTargetTriangleCount = SeedRegion.TriangleCount;
				Member.bComplete = true;
				Member.CanonicalPointCount = CanonicalPoints.Num();
				PlayableC2CanonicalSupportPoints.Append(CanonicalPoints);
				continue;
			}
			FBox3d TargetBounds(EForceInit::ForceInit);
			for (const int32 VertexIndex : SeedVertices)
			{
				TargetBounds += ReflectVector(MeshVertices[VertexIndex], TransformMask);
			}
			TargetBounds = TargetBounds.ExpandBy(25.0);
			double SquaredPositionSum = 0.0;
			double SquaredNormalAngleSum = 0.0;
			TSet<int32> TargetTriangles;
			for (const int32 VertexIndex : SeedVertices)
			{
				const FVector3d TargetPosition = ReflectVector(
					MeshVertices[VertexIndex], TransformMask);
				const FVector3d SourceNormal = VertexNormalCounts[VertexIndex] > 0
					? VertexNormalSums[VertexIndex].GetSafeNormal()
					: FVector3d::UpVector;
				const FVector3d TargetNormal = ReflectVector(
					SourceNormal, TransformMask).GetSafeNormal();
				double BestSquaredResidual = FMath::Square(25.0);
				int32 BestTriangleIndex = INDEX_NONE;
				FVector3d BestPoint = FVector3d::ZeroVector;
				TArray<int32, TInlineAllocator<64>> PendingNodes;
				PendingNodes.Add(0);
				while (!PendingNodes.IsEmpty())
				{
					const FTriangleBvhNode& Node = TriangleBvh[
						PendingNodes.Pop(EAllowShrinking::No)];
					if (!Node.Bounds.Intersect(TargetBounds) ||
						SquaredDistanceToBox(TargetPosition, Node.Bounds) >
							BestSquaredResidual)
					{
						continue;
					}
					if (Node.IsLeaf())
					{
						for (int32 Offset = 0; Offset < Node.IndexCount; ++Offset)
						{
							const int32 TriangleIndex = TriangleIndices[
								Node.FirstIndex + Offset];
							const FTriangleSurface& Triangle = Triangles[TriangleIndex];
							if (!TriangleSmoothRegionIndices.IsValidIndex(TriangleIndex) ||
								TriangleSmoothRegionIndices[TriangleIndex] != SmoothRegionIndex ||
								Triangle.SourceId != SeedRegion.SourceId ||
								Triangle.SurfaceId != SeedRegion.SurfaceId ||
								Triangle.MaterialId != SeedRegion.MaterialId ||
								!Triangle.Bounds.Intersect(TargetBounds))
							{
								continue;
							}
							const FVector3d Closest = ClosestPointOnTriangle(
								TargetPosition, Triangle);
							const double SquaredResidual = FVector3d::DistSquared(
								TargetPosition, Closest);
							if (SquaredResidual < BestSquaredResidual ||
								(SquaredResidual == BestSquaredResidual &&
									(BestTriangleIndex == INDEX_NONE ||
										Triangle.PrimitiveId <
										Triangles[BestTriangleIndex].PrimitiveId)))
							{
								BestSquaredResidual = SquaredResidual;
								BestTriangleIndex = TriangleIndex;
								BestPoint = Closest;
							}
						}
					}
					else
					{
						if (Node.RightChild != INDEX_NONE)
							PendingNodes.Add(Node.RightChild);
						if (Node.LeftChild != INDEX_NONE)
							PendingNodes.Add(Node.LeftChild);
					}
				}
				if (BestTriangleIndex == INDEX_NONE) continue;
				++Member.MatchedSampleCount;
				TargetTriangles.Add(BestTriangleIndex);
				CanonicalPoints.Add(ReflectVector(BestPoint, TransformMask));
				SquaredPositionSum += BestSquaredResidual;
				Member.MaximumPositionResidualCm = FMath::Max(
					Member.MaximumPositionResidualCm,
					FMath::Sqrt(BestSquaredResidual));
				const double NormalResidual = VectorAngleDegrees(
					TargetNormal,
					InterpolatedNormal(Triangles[BestTriangleIndex], BestPoint));
				SquaredNormalAngleSum += FMath::Square(NormalResidual);
				Member.MaximumNormalResidualDegrees = FMath::Max(
					Member.MaximumNormalResidualDegrees, NormalResidual);
			}
			Member.UniqueTargetTriangleCount = TargetTriangles.Num();
			if (Member.MatchedSampleCount > 0)
			{
				Member.RootMeanSquarePositionResidualCm = FMath::Sqrt(
					SquaredPositionSum /
					static_cast<double>(Member.MatchedSampleCount));
				Member.RootMeanSquareNormalResidualDegrees = FMath::Sqrt(
					SquaredNormalAngleSum /
					static_cast<double>(Member.MatchedSampleCount));
			}
			Member.bComplete = Member.MatchedSampleCount == Member.SampleCount;
			Member.CanonicalPointCount = CanonicalPoints.Num();
			PlayableC2CanonicalSupportPoints.Append(CanonicalPoints);
		}
		Candidate.MemberCount = PlayableC2TriangleSupportMembers.Num() -
			Candidate.FirstMemberIndex;
		Candidate.bComplete = Candidate.MemberCount ==
			Candidate.ExpectedMemberCount;
		for (int32 Offset = 0; Offset < Candidate.MemberCount; ++Offset)
		{
			const FPlayableC2TriangleSupportMember& Member =
				PlayableC2TriangleSupportMembers[
					Candidate.FirstMemberIndex + Offset];
			Candidate.bComplete &= Member.bComplete;
			Candidate.MaximumMemberRootMeanSquarePositionResidualCm = FMath::Max(
				Candidate.MaximumMemberRootMeanSquarePositionResidualCm,
				Member.RootMeanSquarePositionResidualCm);
			Candidate.MaximumPositionResidualCm = FMath::Max(
				Candidate.MaximumPositionResidualCm,
				Member.MaximumPositionResidualCm);
			Candidate.MaximumMemberRootMeanSquareNormalResidualDegrees = FMath::Max(
				Candidate.MaximumMemberRootMeanSquareNormalResidualDegrees,
				Member.RootMeanSquareNormalResidualDegrees);
			Candidate.MaximumNormalResidualDegrees = FMath::Max(
				Candidate.MaximumNormalResidualDegrees,
				Member.MaximumNormalResidualDegrees);
		}
		Candidate.bSupportPlausible = Candidate.bComplete &&
			Candidate.MaximumMemberRootMeanSquarePositionResidualCm <= 2.0 &&
			Candidate.MaximumPositionResidualCm <= 15.0 &&
			Candidate.MaximumMemberRootMeanSquareNormalResidualDegrees <= 2.0 &&
			Candidate.MaximumNormalResidualDegrees <= 10.0;
		if (!Candidate.bSupportPlausible ||
			!C2TransitionSectionFits.IsValidIndex(Coverage.TransitionFitIndex) ||
			MemberCanonicalWorldPoints.Num() != Candidate.MemberCount)
		{
			continue;
		}
		const FC2TransitionSectionFit& SeedFit =
			C2TransitionSectionFits[Coverage.TransitionFitIndex];
		if (!QuarterEllipseBoundaryMatches.IsValidIndex(
			SeedFit.BoundaryMatchUIndex) ||
			!QuarterEllipseBoundaryMatches.IsValidIndex(
				SeedFit.BoundaryMatchVIndex))
		{
			continue;
		}
		const int32 PlaneUIndex = QuarterEllipseBoundaryMatches[
			SeedFit.BoundaryMatchUIndex].PlanarGroupIndex;
		const int32 PlaneVIndex = QuarterEllipseBoundaryMatches[
			SeedFit.BoundaryMatchVIndex].PlanarGroupIndex;
		if (!PlanarSurfaceGroups.IsValidIndex(PlaneUIndex) ||
			!PlanarSurfaceGroups.IsValidIndex(PlaneVIndex))
		{
			continue;
		}
		FVector3d NormalU = PlanarSurfaceGroups[PlaneUIndex].Normal;
		FVector3d NormalV = PlanarSurfaceGroups[PlaneVIndex].Normal;
		double OffsetU = PlanarSurfaceGroups[PlaneUIndex].PlaneOffset;
		double OffsetV = PlanarSurfaceGroups[PlaneVIndex].PlaneOffset;
		if (FVector3d::DotProduct(NormalU, SeedRegion.SectionAxisU) < 0.0)
		{
			NormalU *= -1.0;
			OffsetU *= -1.0;
		}
		if (FVector3d::DotProduct(NormalV, SeedRegion.SectionAxisV) < 0.0)
		{
			NormalV *= -1.0;
			OffsetV *= -1.0;
		}
		const double Dot = FVector3d::DotProduct(NormalU, NormalV);
		const double Denominator = 1.0 - Dot * Dot;
		if (Denominator <= 1.0e-8) continue;
		const FVector3d DualU = (NormalU - Dot * NormalV) / Denominator;
		const FVector3d DualV = (NormalV - Dot * NormalU) / Denominator;
		TArray<TArray<FVector2d>> MemberPoints;
		for (const TArray<FVector3d>& WorldPoints : MemberCanonicalWorldPoints)
		{
			TArray<FVector2d>& Points = MemberPoints.AddDefaulted_GetRef();
			Points.Reserve(WorldPoints.Num());
			for (const FVector3d& Point : WorldPoints)
			{
				Points.Emplace(FVector3d::DotProduct(NormalU, Point),
					FVector3d::DotProduct(NormalV, Point));
			}
		}
		auto MeasurePoints = [&](const TArray<FVector2d>& Points,
			const double RadiusU, const double RadiusV,
			const double Flattening, double* OutMaximum)
		{
			const FVector2d Center(OffsetU - SeedFit.SignU * RadiusU,
				OffsetV - SeedFit.SignV * RadiusV);
			double SquaredSum = 0.0;
			double Maximum = 0.0;
			for (const FVector2d& Point : Points)
			{
				const double Squared = SquaredDistanceToQuinticTransitionInDualBasis(
					Point, Center, RadiusU, RadiusV, SeedFit.SignU,
					SeedFit.SignV, Flattening, DualU, DualV);
				SquaredSum += Squared;
				Maximum = FMath::Max(Maximum, FMath::Sqrt(Squared));
			}
			if (OutMaximum) *OutMaximum = Maximum;
			return SquaredSum / static_cast<double>(Points.Num());
		};
		auto Objective = [&](const double RadiusU, const double RadiusV,
			const double Flattening)
		{
			double Sum = 0.0;
			for (const TArray<FVector2d>& Points : MemberPoints)
			{
				Sum += MeasurePoints(Points, RadiusU, RadiusV,
					Flattening, nullptr);
			}
			return Sum / static_cast<double>(MemberPoints.Num());
		};
		Candidate.SharedRadiusU = SeedFit.RadiusU;
		Candidate.SharedRadiusV = SeedFit.RadiusV;
		Candidate.SharedFlatteningFraction = SeedFit.FlatteningFraction;
		double Steps[3] = { 0.02 * Candidate.SharedRadiusU,
			0.02 * Candidate.SharedRadiusV, 0.01 };
		double BestObjective = Objective(Candidate.SharedRadiusU,
			Candidate.SharedRadiusV, Candidate.SharedFlatteningFraction);
		for (int32 Iteration = 0; Iteration < 14; ++Iteration)
		{
			double BestU = Candidate.SharedRadiusU;
			double BestV = Candidate.SharedRadiusV;
			double BestF = Candidate.SharedFlatteningFraction;
			for (int32 DU = -1; DU <= 1; ++DU)
			for (int32 DV = -1; DV <= 1; ++DV)
			for (int32 DF = -1; DF <= 1; ++DF)
			{
				if (DU == 0 && DV == 0 && DF == 0) continue;
				const double U = FMath::Max(1.0,
					Candidate.SharedRadiusU + DU * Steps[0]);
				const double V = FMath::Max(1.0,
					Candidate.SharedRadiusV + DV * Steps[1]);
				const double F = FMath::Clamp(
					Candidate.SharedFlatteningFraction + DF * Steps[2],
					0.01, 0.49);
				const double Value = Objective(U, V, F);
				if (Value < BestObjective)
				{
					BestObjective = Value;
					BestU = U;
					BestV = V;
					BestF = F;
				}
			}
			Candidate.SharedRadiusU = BestU;
			Candidate.SharedRadiusV = BestV;
			Candidate.SharedFlatteningFraction = BestF;
			for (double& Step : Steps) Step *= 0.5;
		}
		Candidate.BalancedProfileRootMeanSquareResidualCm =
			FMath::Sqrt(BestObjective);
		for (int32 MemberOffset = 0;
			MemberOffset < Candidate.MemberCount; ++MemberOffset)
		{
			FPlayableC2TriangleSupportMember& Member =
				PlayableC2TriangleSupportMembers[
					Candidate.FirstMemberIndex + MemberOffset];
			double Maximum = 0.0;
			const double MeanSquared = MeasurePoints(MemberPoints[MemberOffset],
				Candidate.SharedRadiusU, Candidate.SharedRadiusV,
				Candidate.SharedFlatteningFraction, &Maximum);
			Member.ProfileRootMeanSquareResidualCm = FMath::Sqrt(MeanSquared);
			Member.ProfileMaximumResidualCm = Maximum;
			Candidate.MaximumMemberProfileRootMeanSquareResidualCm = FMath::Max(
				Candidate.MaximumMemberProfileRootMeanSquareResidualCm,
				Member.ProfileRootMeanSquareResidualCm);
			Candidate.MaximumProfileResidualCm = FMath::Max(
				Candidate.MaximumProfileResidualCm, Maximum);
		}
		FC2TransitionSectionFit Probe;
		Probe.CenterCoordinates = FVector2d(
			OffsetU - SeedFit.SignU * Candidate.SharedRadiusU,
			OffsetV - SeedFit.SignV * Candidate.SharedRadiusV);
		Probe.RadiusU = Candidate.SharedRadiusU;
		Probe.RadiusV = Candidate.SharedRadiusV;
		Probe.SignU = SeedFit.SignU;
		Probe.SignV = SeedFit.SignV;
		Probe.FlatteningFraction = Candidate.SharedFlatteningFraction;
		const FVector2d Start = Probe.EvaluatePosition(0.0);
		const FVector2d End = Probe.EvaluatePosition(1.0);
		const FVector3d WorldStart = Start.X * DualU + Start.Y * DualV;
		const FVector3d WorldEnd = End.X * DualU + End.Y * DualV;
		Candidate.MaximumEndpointPlaneResidualCm = FMath::Max(
			FMath::Abs(FVector3d::DotProduct(NormalV, WorldStart) - OffsetV),
			FMath::Abs(FVector3d::DotProduct(NormalU, WorldEnd) - OffsetU));
		const FVector2d StartDerivative = Probe.EvaluateFirstDerivative(0.0);
		const FVector2d EndDerivative = Probe.EvaluateFirstDerivative(1.0);
		const FVector3d WorldStartDerivative =
			StartDerivative.X * DualU + StartDerivative.Y * DualV;
		const FVector3d WorldEndDerivative =
			EndDerivative.X * DualU + EndDerivative.Y * DualV;
		Candidate.MaximumEndpointTangentPlaneResidual = FMath::Max(
			FMath::Abs(FVector3d::DotProduct(NormalV, WorldStartDerivative)) /
				FMath::Max(WorldStartDerivative.Length(), 1.0e-12),
			FMath::Abs(FVector3d::DotProduct(NormalU, WorldEndDerivative)) /
				FMath::Max(WorldEndDerivative.Length(), 1.0e-12));
		for (const double T : { 0.0, 1.0 })
		{
			const FVector2d Second = Probe.EvaluateSecondDerivative(T);
			Candidate.MaximumEndpointSecondDerivativeMagnitude = FMath::Max(
				Candidate.MaximumEndpointSecondDerivativeMagnitude,
				(Second.X * DualU + Second.Y * DualV).Length());
		}
		Candidate.bExactC0G1C2ByConstruction =
			Candidate.MaximumEndpointPlaneResidualCm <= 1.0e-9 &&
			Candidate.MaximumEndpointTangentPlaneResidual <= 1.0e-12 &&
			Candidate.MaximumEndpointSecondDerivativeMagnitude <= 1.0e-9;
		const double MinimumRadius = FMath::Min(
			Candidate.SharedRadiusU, Candidate.SharedRadiusV);
		Candidate.bSourceFitPlausible =
			Candidate.bExactC0G1C2ByConstruction &&
			Candidate.MaximumMemberProfileRootMeanSquareResidualCm <=
				0.01 * MinimumRadius &&
			Candidate.MaximumProfileResidualCm <= 0.02 * MinimumRadius;
	}
}

void FAnalyticWorldData::BuildPlayableC2PlaneBindings()
{
	PlayableC2PlaneBindings.Reset();
	auto AcuteAngleDegrees = [](const FVector3d& A, const FVector3d& B)
	{
		const double Angle = VectorAngleDegrees(A, B);
		return FMath::Min(Angle, 180.0 - Angle);
	};
	for (int32 CandidateIndex = 0;
		CandidateIndex < PlayableC2TriangleSupportCandidates.Num();
		++CandidateIndex)
	{
		const FPlayableC2TriangleSupportCandidate& Candidate =
			PlayableC2TriangleSupportCandidates[CandidateIndex];
		if (!Candidate.bSourceFitPlausible ||
			!C2TransitionCoverage.IsValidIndex(Candidate.SeedCoverageIndex))
		{
			continue;
		}
		const FC2TransitionCoverageEntry& Coverage =
			C2TransitionCoverage[Candidate.SeedCoverageIndex];
		if (!C2TransitionSectionFits.IsValidIndex(Coverage.TransitionFitIndex) ||
			!ExtrusionSurfaceRegions.IsValidIndex(Coverage.ExtrusionRegionIndex))
		{
			continue;
		}
		const FC2TransitionSectionFit& SeedFit =
			C2TransitionSectionFits[Coverage.TransitionFitIndex];
		const FExtrusionSurfaceRegion& SeedRegion =
			ExtrusionSurfaceRegions[Coverage.ExtrusionRegionIndex];
		const int32 BoundaryIndices[2] = {
			SeedFit.BoundaryMatchUIndex, SeedFit.BoundaryMatchVIndex };
		FVector3d Normals[2];
		double Offsets[2] = { 0.0, 0.0 };
		int32 SourcePlaneIndices[2] = { INDEX_NONE, INDEX_NONE };
		bool bValid = true;
		for (int32 EndpointIndex = 0; EndpointIndex < 2; ++EndpointIndex)
		{
			if (!QuarterEllipseBoundaryMatches.IsValidIndex(
				BoundaryIndices[EndpointIndex]))
			{
				bValid = false;
				break;
			}
			SourcePlaneIndices[EndpointIndex] = QuarterEllipseBoundaryMatches[
				BoundaryIndices[EndpointIndex]].PlanarGroupIndex;
			if (!PlanarSurfaceGroups.IsValidIndex(
				SourcePlaneIndices[EndpointIndex]))
			{
				bValid = false;
				break;
			}
			Normals[EndpointIndex] = PlanarSurfaceGroups[
				SourcePlaneIndices[EndpointIndex]].Normal;
			Offsets[EndpointIndex] = PlanarSurfaceGroups[
				SourcePlaneIndices[EndpointIndex]].PlaneOffset;
		}
		if (!bValid) continue;
		if (FVector3d::DotProduct(Normals[0], SeedRegion.SectionAxisU) < 0.0)
		{
			Normals[0] *= -1.0;
			Offsets[0] *= -1.0;
		}
		if (FVector3d::DotProduct(Normals[1], SeedRegion.SectionAxisV) < 0.0)
		{
			Normals[1] *= -1.0;
			Offsets[1] *= -1.0;
		}
		const double Dot = FVector3d::DotProduct(Normals[0], Normals[1]);
		const double Denominator = 1.0 - Dot * Dot;
		if (Denominator <= 1.0e-8) continue;
		const FVector3d DualU = (Normals[0] - Dot * Normals[1]) / Denominator;
		const FVector3d DualV = (Normals[1] - Dot * Normals[0]) / Denominator;
		FC2TransitionSectionFit Probe;
		Probe.CenterCoordinates = FVector2d(
			Offsets[0] - SeedFit.SignU * Candidate.SharedRadiusU,
			Offsets[1] - SeedFit.SignV * Candidate.SharedRadiusV);
		Probe.RadiusU = Candidate.SharedRadiusU;
		Probe.RadiusV = Candidate.SharedRadiusV;
		Probe.SignU = SeedFit.SignU;
		Probe.SignV = SeedFit.SignV;
		Probe.FlatteningFraction = Candidate.SharedFlatteningFraction;
		// t=1 is incident on U; t=0 is incident on V.
		const FVector2d EndpointCoordinates[2] = {
			Probe.EvaluatePosition(1.0), Probe.EvaluatePosition(0.0) };
		for (int32 EndpointIndex = 0; EndpointIndex < 2; ++EndpointIndex)
		{
			FPlayableC2PlaneBinding& Binding =
				PlayableC2PlaneBindings.AddDefaulted_GetRef();
			Binding.TriangleSupportCandidateIndex = CandidateIndex;
			Binding.Endpoint = EndpointIndex == 0
				? EQuarterEllipseEndpoint::U : EQuarterEllipseEndpoint::V;
			Binding.SourcePlanarGroupIndex = SourcePlaneIndices[EndpointIndex];
			Binding.EndpointPosition =
				EndpointCoordinates[EndpointIndex].X * DualU +
				EndpointCoordinates[EndpointIndex].Y * DualV;
			double BestScore = TNumericLimits<double>::Max();
			for (int32 ConstraintIndex = 0;
				ConstraintIndex < SymmetrizedC2PlaneConstraints.Num();
				++ConstraintIndex)
			{
				const FSymmetrizedC2PlaneConstraint& Constraint =
					SymmetrizedC2PlaneConstraints[ConstraintIndex];
				FVector3d CandidateNormal = Constraint.Normal;
				double CandidateOffset = Constraint.PlaneOffset;
				if (FVector3d::DotProduct(CandidateNormal,
					Normals[EndpointIndex]) < 0.0)
				{
					CandidateNormal *= -1.0;
					CandidateOffset *= -1.0;
				}
				const double NormalResidual = AcuteAngleDegrees(
					CandidateNormal, Normals[EndpointIndex]);
				const double EndpointResidual = FMath::Abs(
					FVector3d::DotProduct(CandidateNormal,
						Binding.EndpointPosition) - CandidateOffset);
				const double OffsetResidual = FMath::Abs(
					CandidateOffset - Offsets[EndpointIndex]);
				const double Score = 1000.0 * NormalResidual +
					EndpointResidual + 0.001 * OffsetResidual;
				if (Score < BestScore ||
					(Score == BestScore && Constraint.PlanarGroupIndex <
						SymmetrizedC2PlaneConstraints[
							Binding.SymmetrizedConstraintIndex].PlanarGroupIndex))
				{
					BestScore = Score;
					Binding.SymmetrizedConstraintIndex = ConstraintIndex;
					Binding.NormalResidualDegrees = NormalResidual;
					Binding.EndpointPlaneResidualCm = EndpointResidual;
					Binding.OrientedOffsetResidualCm = OffsetResidual;
				}
			}
			if (SymmetrizedC2PlaneConstraints.IsValidIndex(
				Binding.SymmetrizedConstraintIndex))
			{
				Binding.bSameSourcePlanarGroup =
					SymmetrizedC2PlaneConstraints[
						Binding.SymmetrizedConstraintIndex].PlanarGroupIndex ==
						Binding.SourcePlanarGroupIndex;
				Binding.bCompatible = Binding.NormalResidualDegrees <= 0.1 &&
					Binding.EndpointPlaneResidualCm <= 2.0;
			}
		}
	}
}

void FAnalyticWorldData::BuildPlayableC2NetworkPlaneConstraints()
{
	PlayableC2NetworkPlaneConstraints.Reset();
	auto ReflectPlane = [](FVector3d& Normal, const uint8 TransformMask)
	{
		if ((TransformMask & 1u) != 0) Normal.X *= -1.0;
		if ((TransformMask & 2u) != 0) Normal.Y *= -1.0;
	};
	auto AcuteAngleDegrees = [](const FVector3d& A, const FVector3d& B)
	{
		const double Angle = VectorAngleDegrees(A, B);
		return FMath::Min(Angle, 180.0 - Angle);
	};
	for (const FSymmetrizedC2PlaneConstraint& Source :
		SymmetrizedC2PlaneConstraints)
	{
		FPlayableC2NetworkPlaneConstraint& Output =
			PlayableC2NetworkPlaneConstraints.AddDefaulted_GetRef();
		Output.OrbitId = Source.OrbitId;
		Output.SourcePlanarGroupIndex = Source.PlanarGroupIndex;
		Output.TransformMaskFromOrbitRoot = Source.TransformMaskFromOrbitRoot;
		Output.Normal = Source.Normal;
		Output.PlaneOffset = Source.PlaneOffset;
		Output.SourceObservationCount = 1;
		Output.MaximumSourceNormalResidualDegrees =
			Source.SourceNormalAdjustmentDegrees;
		Output.MaximumSourceOffsetResidualCm = Source.SourceMaximumResidualCm;
		Output.bInheritedFromSymmetrizedNetwork = true;
		Output.bSourceFitPlausible = Source.bSourceFitPlausible;
		Output.bExactMirrorPlacement = Source.bExactMirrorPlacement;
	}

	struct FSourceObservation
	{
		int32 PlanarGroupIndex = INDEX_NONE;
		FVector3d Normal = FVector3d::UpVector;
		double Offset = 0.0;
	};
	TArray<FSourceObservation> Observations;
	for (const FPlayableC2PlaneBinding& Binding : PlayableC2PlaneBindings)
	{
		if (Binding.bCompatible || !PlanarSurfaceGroups.IsValidIndex(
			Binding.SourcePlanarGroupIndex))
		{
			continue;
		}
		const FPlanarSurfaceGroup& Group =
			PlanarSurfaceGroups[Binding.SourcePlanarGroupIndex];
		const bool bAlreadyObserved = Algo::AnyOf(Observations,
			[&](const FSourceObservation& Observation)
			{
				return Observation.PlanarGroupIndex ==
					Binding.SourcePlanarGroupIndex;
			});
		if (!bAlreadyObserved)
		{
			Observations.Add({ Binding.SourcePlanarGroupIndex,
				Group.Normal, Group.PlaneOffset });
		}
	}
	Algo::Sort(Observations, [&](const FSourceObservation& A,
		const FSourceObservation& B)
	{
		return PlanarSurfaceGroups[A.PlanarGroupIndex].GroupId <
			PlanarSurfaceGroups[B.PlanarGroupIndex].GroupId;
	});
	struct FOrbit
	{
		uint64 OrbitId = 0;
		FVector3d RootNormal = FVector3d::UpVector;
		double RootOffset = 0.0;
		TArray<FSourceObservation> Observations;
		TArray<uint8> ObservationTransforms;
	};
	TArray<FOrbit> Orbits;
	for (const FSourceObservation& Observation : Observations)
	{
		int32 MatchingOrbitIndex = INDEX_NONE;
		uint8 MatchingTransform = 0;
		for (int32 OrbitIndex = 0; OrbitIndex < Orbits.Num() &&
			MatchingOrbitIndex == INDEX_NONE; ++OrbitIndex)
		{
			for (uint8 TransformMask = 0; TransformMask < 4; ++TransformMask)
			{
				FVector3d ReflectedNormal = Orbits[OrbitIndex].RootNormal;
				double ReflectedOffset = Orbits[OrbitIndex].RootOffset;
				ReflectPlane(ReflectedNormal, TransformMask);
				if (FVector3d::DotProduct(ReflectedNormal,
					Observation.Normal) < 0.0)
				{
					ReflectedNormal *= -1.0;
					ReflectedOffset *= -1.0;
				}
				if (AcuteAngleDegrees(ReflectedNormal, Observation.Normal) <= 0.1 &&
					FMath::Abs(ReflectedOffset - Observation.Offset) <= 2.0)
				{
					MatchingOrbitIndex = OrbitIndex;
					MatchingTransform = TransformMask;
					break;
				}
			}
		}
		if (MatchingOrbitIndex == INDEX_NONE)
		{
			FOrbit& Orbit = Orbits.AddDefaulted_GetRef();
			Orbit.OrbitId = PlanarSurfaceGroups[
				Observation.PlanarGroupIndex].GroupId;
			Orbit.RootNormal = Observation.Normal;
			Orbit.RootOffset = Observation.Offset;
			Orbit.Observations.Add(Observation);
			Orbit.ObservationTransforms.Add(0u);
		}
		else
		{
			FOrbit& Orbit = Orbits[MatchingOrbitIndex];
			Orbit.OrbitId = FMath::Min(Orbit.OrbitId,
				PlanarSurfaceGroups[Observation.PlanarGroupIndex].GroupId);
			Orbit.Observations.Add(Observation);
			Orbit.ObservationTransforms.Add(MatchingTransform);
		}
	}
	for (FOrbit& Orbit : Orbits)
	{
		FVector3d NormalSum = FVector3d::ZeroVector;
		double OffsetSum = 0.0;
		for (int32 Index = 0; Index < Orbit.Observations.Num(); ++Index)
		{
			FVector3d Normal = Orbit.Observations[Index].Normal;
			double Offset = Orbit.Observations[Index].Offset;
			ReflectPlane(Normal, Orbit.ObservationTransforms[Index]);
			if (FVector3d::DotProduct(Normal, Orbit.RootNormal) < 0.0)
			{
				Normal *= -1.0;
				Offset *= -1.0;
			}
			NormalSum += Normal;
			OffsetSum += Offset;
		}
		Orbit.RootNormal = NormalSum.GetSafeNormal();
		Orbit.RootOffset = OffsetSum /
			static_cast<double>(Orbit.Observations.Num());
		double MaximumNormalResidual = 0.0;
		double MaximumOffsetResidual = 0.0;
		for (int32 Index = 0; Index < Orbit.Observations.Num(); ++Index)
		{
			FVector3d ExpectedNormal = Orbit.RootNormal;
			double ExpectedOffset = Orbit.RootOffset;
			ReflectPlane(ExpectedNormal, Orbit.ObservationTransforms[Index]);
			if (FVector3d::DotProduct(ExpectedNormal,
				Orbit.Observations[Index].Normal) < 0.0)
			{
				ExpectedNormal *= -1.0;
				ExpectedOffset *= -1.0;
			}
			MaximumNormalResidual = FMath::Max(MaximumNormalResidual,
				AcuteAngleDegrees(ExpectedNormal,
					Orbit.Observations[Index].Normal));
			MaximumOffsetResidual = FMath::Max(MaximumOffsetResidual,
				FMath::Abs(ExpectedOffset - Orbit.Observations[Index].Offset));
		}
		for (uint8 TransformMask = 0; TransformMask < 4; ++TransformMask)
		{
			FVector3d Normal = Orbit.RootNormal;
			double Offset = Orbit.RootOffset;
			ReflectPlane(Normal, TransformMask);
			bool bDuplicate = false;
			for (const FPlayableC2NetworkPlaneConstraint& Existing :
				PlayableC2NetworkPlaneConstraints)
			{
				FVector3d OrientedNormal = Normal;
				double OrientedOffset = Offset;
				if (FVector3d::DotProduct(OrientedNormal, Existing.Normal) < 0.0)
				{
					OrientedNormal *= -1.0;
					OrientedOffset *= -1.0;
				}
				if (AcuteAngleDegrees(OrientedNormal, Existing.Normal) <= 1.0e-9 &&
					FMath::Abs(OrientedOffset - Existing.PlaneOffset) <= 1.0e-9)
				{
					bDuplicate = true;
					break;
				}
			}
			if (bDuplicate) continue;
			FPlayableC2NetworkPlaneConstraint& Output =
				PlayableC2NetworkPlaneConstraints.AddDefaulted_GetRef();
			Output.OrbitId = Orbit.OrbitId;
			Output.TransformMaskFromOrbitRoot = TransformMask;
			Output.Normal = Normal;
			Output.PlaneOffset = Offset;
			Output.SourceObservationCount = Orbit.Observations.Num();
			Output.MaximumSourceNormalResidualDegrees = MaximumNormalResidual;
			Output.MaximumSourceOffsetResidualCm = MaximumOffsetResidual;
			for (int32 Index = 0; Index < Orbit.Observations.Num(); ++Index)
			{
				if (Orbit.ObservationTransforms[Index] == TransformMask)
				{
					Output.SourcePlanarGroupIndex =
						Orbit.Observations[Index].PlanarGroupIndex;
					break;
				}
			}
			Output.bSourceFitPlausible = MaximumNormalResidual <= 0.1 &&
				MaximumOffsetResidual <= 2.0;
			Output.bExactMirrorPlacement = true;
		}
	}

	for (FPlayableC2PlaneBinding& Binding : PlayableC2PlaneBindings)
	{
		if (!PlanarSurfaceGroups.IsValidIndex(Binding.SourcePlanarGroupIndex))
			continue;
		const FPlanarSurfaceGroup& Source =
			PlanarSurfaceGroups[Binding.SourcePlanarGroupIndex];
		double BestScore = TNumericLimits<double>::Max();
		for (int32 PlaneIndex = 0;
			PlaneIndex < PlayableC2NetworkPlaneConstraints.Num(); ++PlaneIndex)
		{
			const FPlayableC2NetworkPlaneConstraint& Plane =
				PlayableC2NetworkPlaneConstraints[PlaneIndex];
			FVector3d Normal = Plane.Normal;
			double Offset = Plane.PlaneOffset;
			if (FVector3d::DotProduct(Normal, Source.Normal) < 0.0)
			{
				Normal *= -1.0;
				Offset *= -1.0;
			}
			const double NormalResidual =
				AcuteAngleDegrees(Normal, Source.Normal);
			const double EndpointResidual = FMath::Abs(
				FVector3d::DotProduct(Normal, Binding.EndpointPosition) - Offset);
			const double OffsetResidual = FMath::Abs(Offset - Source.PlaneOffset);
			const double Score = 1000.0 * NormalResidual + EndpointResidual +
				0.001 * OffsetResidual;
			if (Score < BestScore)
			{
				BestScore = Score;
				Binding.NetworkPlaneConstraintIndex = PlaneIndex;
				Binding.NetworkNormalResidualDegrees = NormalResidual;
				Binding.NetworkEndpointPlaneResidualCm = EndpointResidual;
				Binding.NetworkOrientedOffsetResidualCm = OffsetResidual;
			}
		}
		Binding.bNetworkCompatible =
			PlayableC2NetworkPlaneConstraints.IsValidIndex(
				Binding.NetworkPlaneConstraintIndex) &&
			Binding.NetworkNormalResidualDegrees <= 0.1 &&
			Binding.NetworkEndpointPlaneResidualCm <= 2.0;
	}
}

void FAnalyticWorldData::BuildPlayableC2NetworkSolutions()
{
	for (int32 CandidateIndex = 0;
		CandidateIndex < PlayableC2TriangleSupportCandidates.Num();
		++CandidateIndex)
	{
		FPlayableC2TriangleSupportCandidate& Candidate =
			PlayableC2TriangleSupportCandidates[CandidateIndex];
		if (!Candidate.bSourceFitPlausible ||
			!C2TransitionCoverage.IsValidIndex(Candidate.SeedCoverageIndex))
		{
			continue;
		}
		const FC2TransitionCoverageEntry& Coverage =
			C2TransitionCoverage[Candidate.SeedCoverageIndex];
		if (!C2TransitionSectionFits.IsValidIndex(Coverage.TransitionFitIndex) ||
			!ExtrusionSurfaceRegions.IsValidIndex(Coverage.ExtrusionRegionIndex))
		{
			continue;
		}
		const FC2TransitionSectionFit& SeedFit =
			C2TransitionSectionFits[Coverage.TransitionFitIndex];
		const FExtrusionSurfaceRegion& SeedRegion =
			ExtrusionSurfaceRegions[Coverage.ExtrusionRegionIndex];
		const FPlayableC2PlaneBinding* Bindings[2] = { nullptr, nullptr };
		for (const FPlayableC2PlaneBinding& Binding : PlayableC2PlaneBindings)
		{
			if (Binding.TriangleSupportCandidateIndex != CandidateIndex) continue;
			Bindings[Binding.Endpoint == EQuarterEllipseEndpoint::U ? 0 : 1] =
				&Binding;
		}
		if (!Bindings[0] || !Bindings[1] ||
			!Bindings[0]->bNetworkCompatible || !Bindings[1]->bNetworkCompatible ||
			!PlayableC2NetworkPlaneConstraints.IsValidIndex(
				Bindings[0]->NetworkPlaneConstraintIndex) ||
			!PlayableC2NetworkPlaneConstraints.IsValidIndex(
				Bindings[1]->NetworkPlaneConstraintIndex))
		{
			continue;
		}
		FVector3d NormalU = PlayableC2NetworkPlaneConstraints[
			Bindings[0]->NetworkPlaneConstraintIndex].Normal;
		double OffsetU = PlayableC2NetworkPlaneConstraints[
			Bindings[0]->NetworkPlaneConstraintIndex].PlaneOffset;
		FVector3d NormalV = PlayableC2NetworkPlaneConstraints[
			Bindings[1]->NetworkPlaneConstraintIndex].Normal;
		double OffsetV = PlayableC2NetworkPlaneConstraints[
			Bindings[1]->NetworkPlaneConstraintIndex].PlaneOffset;
		if (FVector3d::DotProduct(NormalU, SeedRegion.SectionAxisU) < 0.0)
		{
			NormalU *= -1.0;
			OffsetU *= -1.0;
		}
		if (FVector3d::DotProduct(NormalV, SeedRegion.SectionAxisV) < 0.0)
		{
			NormalV *= -1.0;
			OffsetV *= -1.0;
		}
		const double Dot = FVector3d::DotProduct(NormalU, NormalV);
		const double Denominator = 1.0 - Dot * Dot;
		if (Denominator <= 1.0e-8) continue;
		const FVector3d DualU = (NormalU - Dot * NormalV) / Denominator;
		const FVector3d DualV = (NormalV - Dot * NormalU) / Denominator;
		TArray<TArray<FVector2d>> MemberPoints;
		for (int32 MemberOffset = 0;
			MemberOffset < Candidate.MemberCount; ++MemberOffset)
		{
			const FPlayableC2TriangleSupportMember& Member =
				PlayableC2TriangleSupportMembers[
					Candidate.FirstMemberIndex + MemberOffset];
			if (Member.CanonicalPointCount <= 0 ||
				Member.FirstCanonicalPointIndex < 0 ||
				Member.FirstCanonicalPointIndex + Member.CanonicalPointCount >
					PlayableC2CanonicalSupportPoints.Num())
			{
				MemberPoints.Reset();
				break;
			}
			TArray<FVector2d>& Points = MemberPoints.AddDefaulted_GetRef();
			Points.Reserve(Member.CanonicalPointCount);
			for (int32 PointOffset = 0;
				PointOffset < Member.CanonicalPointCount; ++PointOffset)
			{
				const FVector3d& Point = PlayableC2CanonicalSupportPoints[
					Member.FirstCanonicalPointIndex + PointOffset];
				Points.Emplace(FVector3d::DotProduct(NormalU, Point),
					FVector3d::DotProduct(NormalV, Point));
			}
		}
		if (MemberPoints.Num() != Candidate.MemberCount) continue;
		auto MeasurePoints = [&](const TArray<FVector2d>& Points,
			const double RadiusU, const double RadiusV,
			const double Flattening, double* OutMaximum)
		{
			const FVector2d Center(OffsetU - SeedFit.SignU * RadiusU,
				OffsetV - SeedFit.SignV * RadiusV);
			double SquaredSum = 0.0;
			double Maximum = 0.0;
			for (const FVector2d& Point : Points)
			{
				const double Squared = SquaredDistanceToQuinticTransitionInDualBasis(
					Point, Center, RadiusU, RadiusV, SeedFit.SignU,
					SeedFit.SignV, Flattening, DualU, DualV);
				SquaredSum += Squared;
				Maximum = FMath::Max(Maximum, FMath::Sqrt(Squared));
			}
			if (OutMaximum) *OutMaximum = Maximum;
			return SquaredSum / static_cast<double>(Points.Num());
		};
		auto Objective = [&](const double RadiusU, const double RadiusV,
			const double Flattening)
		{
			double Sum = 0.0;
			for (const TArray<FVector2d>& Points : MemberPoints)
			{
				Sum += MeasurePoints(Points, RadiusU, RadiusV,
					Flattening, nullptr);
			}
			return Sum / static_cast<double>(MemberPoints.Num());
		};
		Candidate.NetworkSharedRadiusU = Candidate.SharedRadiusU;
		Candidate.NetworkSharedRadiusV = Candidate.SharedRadiusV;
		Candidate.NetworkSharedFlatteningFraction =
			Candidate.SharedFlatteningFraction;
		double Steps[3] = { 0.02 * Candidate.NetworkSharedRadiusU,
			0.02 * Candidate.NetworkSharedRadiusV, 0.01 };
		double BestObjective = Objective(Candidate.NetworkSharedRadiusU,
			Candidate.NetworkSharedRadiusV,
			Candidate.NetworkSharedFlatteningFraction);
		for (int32 Iteration = 0; Iteration < 14; ++Iteration)
		{
			double BestU = Candidate.NetworkSharedRadiusU;
			double BestV = Candidate.NetworkSharedRadiusV;
			double BestF = Candidate.NetworkSharedFlatteningFraction;
			for (int32 DU = -1; DU <= 1; ++DU)
			for (int32 DV = -1; DV <= 1; ++DV)
			for (int32 DF = -1; DF <= 1; ++DF)
			{
				if (DU == 0 && DV == 0 && DF == 0) continue;
				const double U = FMath::Max(1.0,
					Candidate.NetworkSharedRadiusU + DU * Steps[0]);
				const double V = FMath::Max(1.0,
					Candidate.NetworkSharedRadiusV + DV * Steps[1]);
				const double F = FMath::Clamp(
					Candidate.NetworkSharedFlatteningFraction + DF * Steps[2],
					0.01, 0.49);
				const double Value = Objective(U, V, F);
				if (Value < BestObjective)
				{
					BestObjective = Value;
					BestU = U;
					BestV = V;
					BestF = F;
				}
			}
			Candidate.NetworkSharedRadiusU = BestU;
			Candidate.NetworkSharedRadiusV = BestV;
			Candidate.NetworkSharedFlatteningFraction = BestF;
			for (double& Step : Steps) Step *= 0.5;
		}
		Candidate.NetworkBalancedProfileRootMeanSquareResidualCm =
			FMath::Sqrt(BestObjective);
		for (const TArray<FVector2d>& Points : MemberPoints)
		{
			double Maximum = 0.0;
			const double MeanSquared = MeasurePoints(Points,
				Candidate.NetworkSharedRadiusU,
				Candidate.NetworkSharedRadiusV,
				Candidate.NetworkSharedFlatteningFraction, &Maximum);
			Candidate.NetworkMaximumMemberProfileRootMeanSquareResidualCm =
				FMath::Max(
					Candidate.NetworkMaximumMemberProfileRootMeanSquareResidualCm,
					FMath::Sqrt(MeanSquared));
			Candidate.NetworkMaximumProfileResidualCm = FMath::Max(
				Candidate.NetworkMaximumProfileResidualCm, Maximum);
		}
		FC2TransitionSectionFit Probe;
		Probe.CenterCoordinates = FVector2d(
			OffsetU - SeedFit.SignU * Candidate.NetworkSharedRadiusU,
			OffsetV - SeedFit.SignV * Candidate.NetworkSharedRadiusV);
		Probe.RadiusU = Candidate.NetworkSharedRadiusU;
		Probe.RadiusV = Candidate.NetworkSharedRadiusV;
		Probe.SignU = SeedFit.SignU;
		Probe.SignV = SeedFit.SignV;
		Probe.FlatteningFraction = Candidate.NetworkSharedFlatteningFraction;
		const FVector2d Start = Probe.EvaluatePosition(0.0);
		const FVector2d End = Probe.EvaluatePosition(1.0);
		const FVector3d WorldStart = Start.X * DualU + Start.Y * DualV;
		const FVector3d WorldEnd = End.X * DualU + End.Y * DualV;
		Candidate.NetworkMaximumEndpointPlaneResidualCm = FMath::Max(
			FMath::Abs(FVector3d::DotProduct(NormalV, WorldStart) - OffsetV),
			FMath::Abs(FVector3d::DotProduct(NormalU, WorldEnd) - OffsetU));
		const FVector2d StartDerivative = Probe.EvaluateFirstDerivative(0.0);
		const FVector2d EndDerivative = Probe.EvaluateFirstDerivative(1.0);
		const FVector3d WorldStartDerivative =
			StartDerivative.X * DualU + StartDerivative.Y * DualV;
		const FVector3d WorldEndDerivative =
			EndDerivative.X * DualU + EndDerivative.Y * DualV;
		Candidate.NetworkMaximumEndpointTangentPlaneResidual = FMath::Max(
			FMath::Abs(FVector3d::DotProduct(NormalV, WorldStartDerivative)) /
				FMath::Max(WorldStartDerivative.Length(), 1.0e-12),
			FMath::Abs(FVector3d::DotProduct(NormalU, WorldEndDerivative)) /
				FMath::Max(WorldEndDerivative.Length(), 1.0e-12));
		for (const double T : { 0.0, 1.0 })
		{
			const FVector2d Second = Probe.EvaluateSecondDerivative(T);
			Candidate.NetworkMaximumEndpointSecondDerivativeMagnitude = FMath::Max(
				Candidate.NetworkMaximumEndpointSecondDerivativeMagnitude,
				(Second.X * DualU + Second.Y * DualV).Length());
		}
		Candidate.bNetworkExactC0G1C2ByConstruction =
			Candidate.NetworkMaximumEndpointPlaneResidualCm <= 1.0e-9 &&
			Candidate.NetworkMaximumEndpointTangentPlaneResidual <= 1.0e-12 &&
			Candidate.NetworkMaximumEndpointSecondDerivativeMagnitude <= 1.0e-9;
		const double MinimumRadius = FMath::Min(
			Candidate.NetworkSharedRadiusU, Candidate.NetworkSharedRadiusV);
		Candidate.bNetworkSourceFitPlausible =
			Candidate.bNetworkExactC0G1C2ByConstruction &&
			Candidate.NetworkMaximumMemberProfileRootMeanSquareResidualCm <=
				0.01 * MinimumRadius &&
			Candidate.NetworkMaximumProfileResidualCm <= 0.02 * MinimumRadius;
	}
}

void FAnalyticWorldData::BuildEndWallBoundaryComponents()
{
	EndWallBoundaryComponents.Reset();
	EndWallBoundaryEdgeIndices.Reset();
	OpenRimCandidates.Reset();
	OpenRimEdgeIndices.Reset();
	for (int32 PlaneIndex = 0;
		PlaneIndex < PlayableC2NetworkPlaneConstraints.Num(); ++PlaneIndex)
	{
		const FPlayableC2NetworkPlaneConstraint& Plane =
			PlayableC2NetworkPlaneConstraints[PlaneIndex];
		if (!Plane.bInheritedFromSymmetrizedNetwork ||
			FMath::Max(FMath::Abs(Plane.Normal.X),
				FMath::Abs(Plane.Normal.Y)) < 0.99 ||
			FMath::Abs(Plane.Normal.Z) > 0.01 ||
			!PlanarSurfaceGroups.IsValidIndex(Plane.SourcePlanarGroupIndex))
		{
			continue;
		}
		const FPlanarSurfaceGroup& Group =
			PlanarSurfaceGroups[Plane.SourcePlanarGroupIndex];
		if (!Group.bArchitecturalConstraint) continue;
		TArray<bool> bTriangleInGroup;
		bTriangleInGroup.Init(false, Triangles.Num());
		for (int32 PatchOffset = 0; PatchOffset < Group.PatchCount; ++PatchOffset)
		{
			const int32 PatchIndex = PlanarGroupPatchIndices[
				Group.FirstPatchIndex + PatchOffset];
			const FSurfacePatch& Patch = SurfacePatches[PatchIndex];
			for (int32 TriangleOffset = 0;
				TriangleOffset < Patch.TriangleCount; ++TriangleOffset)
			{
				bTriangleInGroup[PatchTriangleIndices[
					Patch.FirstTriangleIndex + TriangleOffset]] = true;
			}
		}
		TArray<int32> InterfaceEdges;
		for (int32 EdgeIndex = 0; EdgeIndex < MeshEdges.Num(); ++EdgeIndex)
		{
			const FTriangleMeshEdge& Edge = MeshEdges[EdgeIndex];
			if (!Edge.IsManifold()) continue;
			const int32 TriangleA = EdgeIncidentTriangleIndices[
				Edge.FirstIncidentTriangle];
			const int32 TriangleB = EdgeIncidentTriangleIndices[
				Edge.FirstIncidentTriangle + 1];
			if (bTriangleInGroup[TriangleA] != bTriangleInGroup[TriangleB])
			{
				InterfaceEdges.Add(EdgeIndex);
			}
		}
		Algo::Sort(InterfaceEdges);
		TMap<int32, TArray<int32>> EdgesByVertex;
		for (const int32 EdgeIndex : InterfaceEdges)
		{
			const FTriangleMeshEdge& Edge = MeshEdges[EdgeIndex];
			EdgesByVertex.FindOrAdd(Edge.VertexA).Add(EdgeIndex);
			EdgesByVertex.FindOrAdd(Edge.VertexB).Add(EdgeIndex);
		}
		TSet<int32> AssignedEdges;
		for (const int32 SeedEdgeIndex : InterfaceEdges)
		{
			if (AssignedEdges.Contains(SeedEdgeIndex)) continue;
			TArray<int32> Pending = { SeedEdgeIndex };
			TArray<int32> ComponentEdges;
			TSet<int32> Vertices;
			TSet<int32> AdjacentTriangles;
			while (!Pending.IsEmpty())
			{
				const int32 EdgeIndex = Pending.Pop(EAllowShrinking::No);
				if (AssignedEdges.Contains(EdgeIndex)) continue;
				AssignedEdges.Add(EdgeIndex);
				ComponentEdges.Add(EdgeIndex);
				const FTriangleMeshEdge& Edge = MeshEdges[EdgeIndex];
				for (const int32 VertexIndex : { Edge.VertexA, Edge.VertexB })
				{
					Vertices.Add(VertexIndex);
					if (const TArray<int32>* Connected =
						EdgesByVertex.Find(VertexIndex))
					{
						for (const int32 ConnectedEdge : *Connected)
						{
							if (!AssignedEdges.Contains(ConnectedEdge))
								Pending.Add(ConnectedEdge);
						}
					}
				}
				for (int32 IncidentOffset = 0;
					IncidentOffset < Edge.IncidentTriangleCount; ++IncidentOffset)
				{
					const int32 TriangleIndex = EdgeIncidentTriangleIndices[
						Edge.FirstIncidentTriangle + IncidentOffset];
					if (!bTriangleInGroup[TriangleIndex])
						AdjacentTriangles.Add(TriangleIndex);
				}
			}
			Algo::Sort(ComponentEdges);
			FEndWallBoundaryComponent& Component =
				EndWallBoundaryComponents.AddDefaulted_GetRef();
			Component.ComponentId = CombineStableIds(Group.GroupId,
				static_cast<uint64>(ComponentEdges[0]) + 1ull);
			Component.NetworkPlaneConstraintIndex = PlaneIndex;
			Component.PlanarGroupIndex = Plane.SourcePlanarGroupIndex;
			Component.FirstEdgeIndex = EndWallBoundaryEdgeIndices.Num();
			Component.EdgeCount = ComponentEdges.Num();
			Component.UniqueVertexCount = Vertices.Num();
			Component.AdjacentTriangleCount = AdjacentTriangles.Num();
			TMap<int32, int32> VertexDegree;
			for (const int32 EdgeIndex : ComponentEdges)
			{
				EndWallBoundaryEdgeIndices.Add(EdgeIndex);
				const FTriangleMeshEdge& Edge = MeshEdges[EdgeIndex];
				++VertexDegree.FindOrAdd(Edge.VertexA);
				++VertexDegree.FindOrAdd(Edge.VertexB);
				Component.PerimeterCm += FVector3d::Distance(
					MeshVertices[Edge.VertexA], MeshVertices[Edge.VertexB]);
				Component.Bounds += MeshVertices[Edge.VertexA];
				Component.Bounds += MeshVertices[Edge.VertexB];
			}
			Component.bClosedLoop = true;
			for (const TPair<int32, int32>& Pair : VertexDegree)
			{
				Component.bClosedLoop &= Pair.Value == 2;
			}
			Component.bCrossesXSymmetryPlane =
				Component.Bounds.Min.X <= 0.0 && Component.Bounds.Max.X >= 0.0;
			for (const FC2TransitionCoverageEntry& Coverage : C2TransitionCoverage)
			{
				if (!C2TransitionSectionFits.IsValidIndex(
					Coverage.TransitionFitIndex)) continue;
				const FC2TransitionSectionFit& Fit =
					C2TransitionSectionFits[Coverage.TransitionFitIndex];
				const int32 MatchIndices[2] = {
					Fit.BoundaryMatchUIndex, Fit.BoundaryMatchVIndex };
				for (const int32 MatchIndex : MatchIndices)
				{
					if (!QuarterEllipseBoundaryMatches.IsValidIndex(MatchIndex) ||
						QuarterEllipseBoundaryMatches[
							MatchIndex].PlanarGroupIndex !=
							Component.PlanarGroupIndex)
					{
						continue;
					}
					if (Coverage.SurfaceLayer ==
						EC2TransitionSurfaceLayer::PlayableInner ||
						Component.SurfaceLayer ==
						EC2TransitionSurfaceLayer::Unknown)
					{
						Component.SurfaceLayer = Coverage.SurfaceLayer;
					}
				}
			}
		}
	}
	for (int32 ComponentIndex = 0;
		ComponentIndex < EndWallBoundaryComponents.Num(); ++ComponentIndex)
	{
		const FEndWallBoundaryComponent& Component =
			EndWallBoundaryComponents[ComponentIndex];
		const FPlayableC2NetworkPlaneConstraint& Plane =
			PlayableC2NetworkPlaneConstraints[
				Component.NetworkPlaneConstraintIndex];
		const uint8 WallAxis = FMath::Abs(Plane.Normal.X) >=
			FMath::Abs(Plane.Normal.Y) ? 0u : 1u;
		auto TransverseCoordinate = [WallAxis](const FVector3d& Position)
		{
			return WallAxis == 0u ? Position.Y : Position.X;
		};
		const double MinimumTransverse = WallAxis == 0u
			? Component.Bounds.Min.Y : Component.Bounds.Min.X;
		const double MaximumTransverse = WallAxis == 0u
			? Component.Bounds.Max.Y : Component.Bounds.Max.X;
		const double TransverseSpan = MaximumTransverse - MinimumTransverse;
		const double HeightSpan = Component.Bounds.Max.Z - Component.Bounds.Min.Z;
		const double EnvelopeTolerance = FMath::Max(1.0,
			0.005 * FMath::Max(TransverseSpan, HeightSpan));
		TArray<int32> ResidualEdges;
		for (int32 EdgeOffset = 0; EdgeOffset < Component.EdgeCount; ++EdgeOffset)
		{
			const int32 EdgeIndex = EndWallBoundaryEdgeIndices[
				Component.FirstEdgeIndex + EdgeOffset];
			const FTriangleMeshEdge& Edge = MeshEdges[EdgeIndex];
			const FVector3d& A = MeshVertices[Edge.VertexA];
			const FVector3d& B = MeshVertices[Edge.VertexB];
			const double AT = TransverseCoordinate(A);
			const double BT = TransverseCoordinate(B);
			const bool bOuterEnvelope =
				(FMath::Abs(AT - MinimumTransverse) <= EnvelopeTolerance &&
					FMath::Abs(BT - MinimumTransverse) <= EnvelopeTolerance) ||
				(FMath::Abs(AT - MaximumTransverse) <= EnvelopeTolerance &&
					FMath::Abs(BT - MaximumTransverse) <= EnvelopeTolerance) ||
				(FMath::Abs(A.Z - Component.Bounds.Min.Z) <= EnvelopeTolerance &&
					FMath::Abs(B.Z - Component.Bounds.Min.Z) <= EnvelopeTolerance) ||
				(FMath::Abs(A.Z - Component.Bounds.Max.Z) <= EnvelopeTolerance &&
					FMath::Abs(B.Z - Component.Bounds.Max.Z) <= EnvelopeTolerance);
			if (!bOuterEnvelope) ResidualEdges.Add(EdgeIndex);
		}
		Algo::Sort(ResidualEdges);
		TMap<int32, TArray<int32>> ResidualEdgesByVertex;
		for (const int32 EdgeIndex : ResidualEdges)
		{
			const FTriangleMeshEdge& Edge = MeshEdges[EdgeIndex];
			ResidualEdgesByVertex.FindOrAdd(Edge.VertexA).Add(EdgeIndex);
			ResidualEdgesByVertex.FindOrAdd(Edge.VertexB).Add(EdgeIndex);
		}
		TSet<int32> AssignedResidualEdges;
		for (const int32 SeedEdgeIndex : ResidualEdges)
		{
			if (AssignedResidualEdges.Contains(SeedEdgeIndex)) continue;
			TArray<int32> Pending = { SeedEdgeIndex };
			TArray<int32> CandidateEdges;
			TMap<int32, int32> VertexDegree;
			while (!Pending.IsEmpty())
			{
				const int32 EdgeIndex = Pending.Pop(EAllowShrinking::No);
				if (AssignedResidualEdges.Contains(EdgeIndex)) continue;
				AssignedResidualEdges.Add(EdgeIndex);
				CandidateEdges.Add(EdgeIndex);
				const FTriangleMeshEdge& Edge = MeshEdges[EdgeIndex];
				for (const int32 VertexIndex : { Edge.VertexA, Edge.VertexB })
				{
					++VertexDegree.FindOrAdd(VertexIndex);
					if (const TArray<int32>* Connected =
						ResidualEdgesByVertex.Find(VertexIndex))
					{
						for (const int32 ConnectedEdge : *Connected)
						{
							if (!AssignedResidualEdges.Contains(ConnectedEdge))
								Pending.Add(ConnectedEdge);
						}
					}
				}
			}
			if (CandidateEdges.Num() < 2) continue;
			Algo::Sort(CandidateEdges);
			FOpenRimCandidate& Candidate =
				OpenRimCandidates.AddDefaulted_GetRef();
			Candidate.CandidateId = CombineStableIds(Component.ComponentId,
				static_cast<uint64>(CandidateEdges[0]) + 1ull);
			Candidate.WallBoundaryComponentIndex = ComponentIndex;
			Candidate.FirstEdgeIndex = OpenRimEdgeIndices.Num();
			Candidate.EdgeCount = CandidateEdges.Num();
			Candidate.UniqueVertexCount = VertexDegree.Num();
			Candidate.BaselineHeightCm = Component.Bounds.Min.Z;
			Candidate.WallAxis = WallAxis;
			Candidate.SurfaceLayer = Component.SurfaceLayer;
			int32 BaselineEndpointCount = 0;
			for (const int32 EdgeIndex : CandidateEdges)
			{
				OpenRimEdgeIndices.Add(EdgeIndex);
				const FTriangleMeshEdge& Edge = MeshEdges[EdgeIndex];
				Candidate.ArcLengthCm += FVector3d::Distance(
					MeshVertices[Edge.VertexA], MeshVertices[Edge.VertexB]);
				Candidate.Bounds += MeshVertices[Edge.VertexA];
				Candidate.Bounds += MeshVertices[Edge.VertexB];
			}
			for (const TPair<int32, int32>& Pair : VertexDegree)
			{
				if (Pair.Value == 1 && FMath::Abs(
					MeshVertices[Pair.Key].Z - Candidate.BaselineHeightCm) <=
					EnvelopeTolerance)
				{
					++BaselineEndpointCount;
				}
			}
			Candidate.bOpenAtBaseline = BaselineEndpointCount == 2;
			const double CandidateMinimumTransverse = WallAxis == 0u
				? Candidate.Bounds.Min.Y : Candidate.Bounds.Min.X;
			const double CandidateMaximumTransverse = WallAxis == 0u
				? Candidate.Bounds.Max.Y : Candidate.Bounds.Max.X;
			Candidate.bCrossesTransverseSymmetryPlane =
				CandidateMinimumTransverse <= 0.0 &&
				CandidateMaximumTransverse >= 0.0;
		}
	}
	auto CandidateEdges = [&](const FOpenRimCandidate& Candidate)
	{
		TArray<int32> Result;
		Result.Reserve(Candidate.EdgeCount);
		for (int32 Offset = 0; Offset < Candidate.EdgeCount; ++Offset)
		{
			Result.Add(OpenRimEdgeIndices[
				Candidate.FirstEdgeIndex + Offset]);
		}
		return Result;
	};
	auto SquaredDistanceToSegment = [](const FVector3d& Point,
		const FVector3d& A, const FVector3d& B)
	{
		const FVector3d AB = B - A;
		const double LengthSquared = AB.SquaredLength();
		if (LengthSquared <= 1.0e-18) return FVector3d::DistSquared(Point, A);
		const double T = FMath::Clamp(
			FVector3d::DotProduct(Point - A, AB) / LengthSquared, 0.0, 1.0);
		return FVector3d::DistSquared(Point, A + T * AB);
	};
	auto MeasureReflection = [&](const FOpenRimCandidate& Source,
		const FOpenRimCandidate& Target, const uint8 ReflectionAxis,
		double& OutMaximum)
	{
		TSet<int32> SourceVertices;
		for (const int32 EdgeIndex : CandidateEdges(Source))
		{
			SourceVertices.Add(MeshEdges[EdgeIndex].VertexA);
			SourceVertices.Add(MeshEdges[EdgeIndex].VertexB);
		}
		const TArray<int32> TargetEdges = CandidateEdges(Target);
		double SquaredSum = 0.0;
		for (const int32 VertexIndex : SourceVertices)
		{
			FVector3d Reflected = MeshVertices[VertexIndex];
			Reflected[ReflectionAxis] *= -1.0;
			double BestSquared = TNumericLimits<double>::Max();
			for (const int32 EdgeIndex : TargetEdges)
			{
				const FTriangleMeshEdge& Edge = MeshEdges[EdgeIndex];
				BestSquared = FMath::Min(BestSquared, SquaredDistanceToSegment(
					Reflected, MeshVertices[Edge.VertexA],
					MeshVertices[Edge.VertexB]));
			}
			SquaredSum += BestSquared;
			OutMaximum = FMath::Max(OutMaximum, FMath::Sqrt(BestSquared));
		}
		return FMath::Sqrt(SquaredSum /
			static_cast<double>(SourceVertices.Num()));
	};
	for (int32 CandidateIndex = 0;
		CandidateIndex < OpenRimCandidates.Num(); ++CandidateIndex)
	{
		FOpenRimCandidate& Candidate =
			OpenRimCandidates[CandidateIndex];
		const FEndWallBoundaryComponent& Component =
			EndWallBoundaryComponents[Candidate.WallBoundaryComponentIndex];
		const FPlayableC2NetworkPlaneConstraint& CandidatePlane =
			PlayableC2NetworkPlaneConstraints[
				Component.NetworkPlaneConstraintIndex];
		double BestMirrorRms = TNumericLimits<double>::Max();
		for (int32 TargetIndex = 0;
			TargetIndex < OpenRimCandidates.Num(); ++TargetIndex)
		{
			if (TargetIndex == CandidateIndex) continue;
			const FOpenRimCandidate& Target =
				OpenRimCandidates[TargetIndex];
			if (Target.WallAxis != Candidate.WallAxis ||
				Target.SurfaceLayer != Candidate.SurfaceLayer) continue;
			const FEndWallBoundaryComponent& TargetComponent =
				EndWallBoundaryComponents[Target.WallBoundaryComponentIndex];
			const FPlayableC2NetworkPlaneConstraint& TargetPlane =
				PlayableC2NetworkPlaneConstraints[
					TargetComponent.NetworkPlaneConstraintIndex];
			if (CandidatePlane.PlaneOffset * TargetPlane.PlaneOffset >= 0.0)
				continue;
			double Maximum = 0.0;
			const double Rms = MeasureReflection(Candidate, Target,
				Candidate.WallAxis, Maximum);
			if (Rms < BestMirrorRms)
			{
				BestMirrorRms = Rms;
				Candidate.LongitudinalMirrorCandidateIndex = TargetIndex;
				Candidate.LongitudinalMirrorRootMeanSquareResidualCm = Rms;
				Candidate.LongitudinalMirrorMaximumResidualCm = Maximum;
			}
		}
		Candidate.bLongitudinalMirrorPlausible =
			Candidate.LongitudinalMirrorCandidateIndex != INDEX_NONE &&
			Candidate.LongitudinalMirrorRootMeanSquareResidualCm <= 5.0 &&
			Candidate.LongitudinalMirrorMaximumResidualCm <= 20.0;
		double SelfMaximum = 0.0;
		Candidate.TransverseSelfRootMeanSquareResidualCm = MeasureReflection(
			Candidate, Candidate, Candidate.WallAxis == 0u ? 1u : 0u,
			SelfMaximum);
		Candidate.TransverseSelfMaximumResidualCm = SelfMaximum;
		Candidate.bTransverseSelfPlausible =
			Candidate.TransverseSelfRootMeanSquareResidualCm <= 5.0 &&
			Candidate.TransverseSelfMaximumResidualCm <= 20.0;

		const double MinimumTransverse = Candidate.WallAxis == 0u
			? Candidate.Bounds.Min.Y : Candidate.Bounds.Min.X;
		const double MaximumTransverse = Candidate.WallAxis == 0u
			? Candidate.Bounds.Max.Y : Candidate.Bounds.Max.X;
		const double MaximumAbsTransverse = FMath::Max(
			FMath::Abs(MinimumTransverse), FMath::Abs(MaximumTransverse));
		const double HeightSpan = Candidate.Bounds.Max.Z -
			Candidate.BaselineHeightCm;
		double HorizontalSpanHeightSum = 0.0;
		for (const int32 EdgeIndex : CandidateEdges(Candidate))
		{
			const FTriangleMeshEdge& Edge = MeshEdges[EdgeIndex];
			const FVector3d& A = MeshVertices[Edge.VertexA];
			const FVector3d& B = MeshVertices[Edge.VertexB];
			const double AT = Candidate.WallAxis == 0u ? A.Y : A.X;
			const double BT = Candidate.WallAxis == 0u ? B.Y : B.X;
			const double DeltaT = BT - AT;
			const double DeltaZ = B.Z - A.Z;
			const double Length = FMath::Sqrt(
				DeltaT * DeltaT + DeltaZ * DeltaZ);
			const double MidT = 0.5 * (AT + BT);
			const double MidZ = 0.5 * (A.Z + B.Z);
			const bool bHorizontalSpan = Length > 1.0e-9 &&
				FMath::Abs(DeltaZ) <= 0.02 * Length &&
				MidZ >= Candidate.BaselineHeightCm + 0.75 * HeightSpan;
			const bool bVerticalSegment = Length > 1.0e-9 &&
				FMath::Abs(DeltaT) <= 0.05 * Length &&
				FMath::Abs(MidT) >= 0.9 * MaximumAbsTransverse &&
				MidZ <= Candidate.BaselineHeightCm + 0.6 * HeightSpan;
			if (bHorizontalSpan)
			{
				++Candidate.HorizontalSpanEdgeCount;
				Candidate.HorizontalSpanSpanCm += Length;
				HorizontalSpanHeightSum += MidZ;
			}
			else if (bVerticalSegment && MidT < 0.0)
			{
				++Candidate.NegativeVerticalEdgeCount;
				Candidate.NegativeVerticalHeightCm = FMath::Max(
					Candidate.NegativeVerticalHeightCm,
					FMath::Max(A.Z, B.Z) - Candidate.BaselineHeightCm);
			}
			else if (bVerticalSegment)
			{
				++Candidate.PositiveVerticalEdgeCount;
				Candidate.PositiveVerticalHeightCm = FMath::Max(
					Candidate.PositiveVerticalHeightCm,
					FMath::Max(A.Z, B.Z) - Candidate.BaselineHeightCm);
			}
			else
			{
				++Candidate.UpperTransitionEdgeCount;
			}
		}
		Candidate.HorizontalSpanHeightCm = Candidate.HorizontalSpanEdgeCount > 0
			? HorizontalSpanHeightSum /
				static_cast<double>(Candidate.HorizontalSpanEdgeCount) : 0.0;
		Candidate.bFeaturePartitionComplete =
			Candidate.NegativeVerticalEdgeCount > 0 && Candidate.PositiveVerticalEdgeCount > 0 &&
			Candidate.HorizontalSpanEdgeCount > 0 &&
			Candidate.NegativeVerticalEdgeCount + Candidate.PositiveVerticalEdgeCount +
				Candidate.HorizontalSpanEdgeCount + Candidate.UpperTransitionEdgeCount ==
				Candidate.EdgeCount;
	}
}

void FAnalyticWorldData::BuildCanonicalOpenArchSolutions()
{
	CanonicalOpenArchSolutions.Reset();
	CanonicalOpenArchMemberFits.Reset();
	struct FMemberSamples
	{
		int32 CandidateIndex = INDEX_NONE;
		uint8 Side = 0;
		TArray<FVector2d> Points;
	};
	TArray<FMemberSamples> Members;
	double BaselineSum = 0.0;
	double VerticalSegmentWidthSum = 0.0;
	double TransitionHeightSum = 0.0;
	double HorizontalSpanSpanSum = 0.0;
	double HorizontalSpanHeightSum = 0.0;
	int32 CandidateCount = 0;
	for (int32 CandidateIndex = 0;
		CandidateIndex < OpenRimCandidates.Num(); ++CandidateIndex)
	{
		const FOpenRimCandidate& Candidate =
			OpenRimCandidates[CandidateIndex];
		if (Candidate.SurfaceLayer != EC2TransitionSurfaceLayer::PlayableInner ||
			Candidate.WallAxis != 0u || !Candidate.bOpenAtBaseline ||
			!Candidate.bCrossesTransverseSymmetryPlane ||
			!Candidate.bFeaturePartitionComplete)
		{
			continue;
		}
		TSet<int32> Vertices;
		for (int32 Offset = 0; Offset < Candidate.EdgeCount; ++Offset)
		{
			const FTriangleMeshEdge& Edge = MeshEdges[OpenRimEdgeIndices[
				Candidate.FirstEdgeIndex + Offset]];
			Vertices.Add(Edge.VertexA);
			Vertices.Add(Edge.VertexB);
		}
		for (uint8 Side = 0; Side < 2; ++Side)
		{
			FMemberSamples& Member = Members.AddDefaulted_GetRef();
			Member.CandidateIndex = CandidateIndex;
			Member.Side = Side;
			for (const int32 VertexIndex : Vertices)
			{
				const FVector3d& Vertex = MeshVertices[VertexIndex];
				if ((Side == 0u && Vertex.Y <= 1.0e-9) ||
					(Side == 1u && Vertex.Y >= -1.0e-9))
				{
					Member.Points.Add(FVector2d(FMath::Abs(Vertex.Y), Vertex.Z));
				}
			}
			Algo::Sort(Member.Points, [](const FVector2d& A, const FVector2d& B)
			{
				return A.X != B.X ? A.X < B.X : A.Y < B.Y;
			});
		}
		BaselineSum += Candidate.BaselineHeightCm;
		VerticalSegmentWidthSum += FMath::Max(FMath::Abs(Candidate.Bounds.Min.Y),
			FMath::Abs(Candidate.Bounds.Max.Y));
		TransitionHeightSum += Candidate.BaselineHeightCm +
			0.5 * (Candidate.NegativeVerticalHeightCm + Candidate.PositiveVerticalHeightCm);
		HorizontalSpanSpanSum += 0.5 * Candidate.HorizontalSpanSpanCm;
		HorizontalSpanHeightSum += Candidate.HorizontalSpanHeightCm;
		++CandidateCount;
	}
	if (CandidateCount != 2 || Members.Num() != 4 ||
		Algo::AnyOf(Members, [](const FMemberSamples& Member)
		{
			return Member.Points.Num() < 3;
		}))
	{
		return;
	}

	FCanonicalOpenArchSolution& Solution =
		CanonicalOpenArchSolutions.AddDefaulted_GetRef();
	Solution.SolutionId = StableStringId(TEXT("PlayableOpenArch.XYShared.V1"));
	Solution.FirstMemberIndex = CanonicalOpenArchMemberFits.Num();
	Solution.MemberCount = Members.Num();
	Solution.BaselineHeightCm = BaselineSum / CandidateCount;
	Solution.VerticalSegmentHalfWidthCm = VerticalSegmentWidthSum / CandidateCount;
	Solution.VerticalSegmentTransitionHeightCm = TransitionHeightSum / CandidateCount;
	Solution.HorizontalSpanHalfSpanCm = HorizontalSpanSpanSum / CandidateCount;
	Solution.HorizontalSpanHeightCm = HorizontalSpanHeightSum / CandidateCount;
	Solution.FlatteningFraction = 0.30;

	auto SquaredDistanceToSegment = [](const FVector2d& Point,
		const FVector2d& A, const FVector2d& B)
	{
		const FVector2d AB = B - A;
		const double LengthSquared = AB.SquaredLength();
		if (LengthSquared <= 1.0e-18) return FVector2d::DistSquared(Point, A);
		const double T = FMath::Clamp(
			FVector2d::DotProduct(Point - A, AB) / LengthSquared, 0.0, 1.0);
		return FVector2d::DistSquared(Point, A + T * AB);
	};
	auto PointSquaredResidual = [&](const FVector2d& Point,
		const FCanonicalOpenArchSolution& Parameters)
	{
		const FVector2d HorizontalSpanA(0.0, Parameters.HorizontalSpanHeightCm);
		const FVector2d HorizontalSpanB(Parameters.HorizontalSpanHalfSpanCm,
			Parameters.HorizontalSpanHeightCm);
		const FVector2d VerticalSegmentA(Parameters.VerticalSegmentHalfWidthCm,
			Parameters.BaselineHeightCm);
		const FVector2d VerticalSegmentB(Parameters.VerticalSegmentHalfWidthCm,
			Parameters.VerticalSegmentTransitionHeightCm);
		const FVector2d Center(Parameters.HorizontalSpanHalfSpanCm,
			Parameters.VerticalSegmentTransitionHeightCm);
		return FMath::Min3(
			SquaredDistanceToSegment(Point, HorizontalSpanA, HorizontalSpanB),
			SquaredDistanceToQuinticTransition(Point, Center,
				Parameters.VerticalSegmentHalfWidthCm - Parameters.HorizontalSpanHalfSpanCm,
				Parameters.HorizontalSpanHeightCm - Parameters.VerticalSegmentTransitionHeightCm,
				1.0, 1.0, Parameters.FlatteningFraction),
			SquaredDistanceToSegment(Point, VerticalSegmentA, VerticalSegmentB));
	};
	auto Objective = [&](const FCanonicalOpenArchSolution& Parameters)
	{
		if (Parameters.HorizontalSpanHalfSpanCm <= 0.0 ||
			Parameters.VerticalSegmentHalfWidthCm <= Parameters.HorizontalSpanHalfSpanCm + 1.0 ||
			Parameters.VerticalSegmentTransitionHeightCm <= Parameters.BaselineHeightCm + 1.0 ||
			Parameters.HorizontalSpanHeightCm <= Parameters.VerticalSegmentTransitionHeightCm + 1.0 ||
			Parameters.FlatteningFraction < 0.01 ||
			Parameters.FlatteningFraction > 0.49)
		{
			return TNumericLimits<double>::Max();
		}
		double BalancedMeanSquared = 0.0;
		for (const FMemberSamples& Member : Members)
		{
			double MemberSum = 0.0;
			for (const FVector2d& Point : Member.Points)
				MemberSum += PointSquaredResidual(Point, Parameters);
			BalancedMeanSquared += MemberSum / Member.Points.Num();
		}
		return BalancedMeanSquared / Members.Num();
	};

	double BestObjective = Objective(Solution);
	double Steps[5] = { 20.0, 20.0, 20.0, 20.0, 0.04 };
	for (int32 Iteration = 0; Iteration < 18; ++Iteration)
	{
		for (int32 Parameter = 0; Parameter < 5; ++Parameter)
		{
			for (const double Sign : { -1.0, 1.0 })
			{
				FCanonicalOpenArchSolution Trial = Solution;
				double* Value = Parameter == 0 ? &Trial.VerticalSegmentHalfWidthCm
					: Parameter == 1 ? &Trial.HorizontalSpanHalfSpanCm
					: Parameter == 2 ? &Trial.VerticalSegmentTransitionHeightCm
					: Parameter == 3 ? &Trial.HorizontalSpanHeightCm
					: &Trial.FlatteningFraction;
				*Value += Sign * Steps[Parameter];
				const double TrialObjective = Objective(Trial);
				if (TrialObjective < BestObjective)
				{
					Solution = Trial;
					BestObjective = TrialObjective;
				}
			}
		}
		for (double& Step : Steps) Step *= 0.65;
	}
	Solution.BalancedRootMeanSquareResidualCm = FMath::Sqrt(BestObjective);
	for (const FMemberSamples& Member : Members)
	{
		FCanonicalOpenArchMemberFit& Fit =
			CanonicalOpenArchMemberFits.AddDefaulted_GetRef();
		Fit.OpenRimCandidateIndex = Member.CandidateIndex;
		Fit.TransverseSide = Member.Side;
		Fit.SampleCount = Member.Points.Num();
		double SquaredSum = 0.0;
		for (const FVector2d& Point : Member.Points)
		{
			const double Squared = PointSquaredResidual(Point, Solution);
			SquaredSum += Squared;
			Fit.MaximumResidualCm = FMath::Max(
				Fit.MaximumResidualCm, FMath::Sqrt(Squared));
		}
		Fit.RootMeanSquareResidualCm = FMath::Sqrt(
			SquaredSum / Member.Points.Num());
		Solution.MaximumMemberRootMeanSquareResidualCm = FMath::Max(
			Solution.MaximumMemberRootMeanSquareResidualCm,
			Fit.RootMeanSquareResidualCm);
		Solution.MaximumResidualCm = FMath::Max(
			Solution.MaximumResidualCm, Fit.MaximumResidualCm);
	}

	FC2TransitionSectionFit Probe;
	Probe.CenterCoordinates = FVector2d(Solution.HorizontalSpanHalfSpanCm,
		Solution.VerticalSegmentTransitionHeightCm);
	Probe.RadiusU = Solution.VerticalSegmentHalfWidthCm - Solution.HorizontalSpanHalfSpanCm;
	Probe.RadiusV = Solution.HorizontalSpanHeightCm - Solution.VerticalSegmentTransitionHeightCm;
	Probe.SignU = 1.0;
	Probe.SignV = 1.0;
	Probe.FlatteningFraction = Solution.FlatteningFraction;
	Solution.MaximumEndpointPositionResidualCm = FMath::Max(
		FVector2d::Distance(Probe.EvaluatePosition(0.0),
			FVector2d(Solution.HorizontalSpanHalfSpanCm, Solution.HorizontalSpanHeightCm)),
		FVector2d::Distance(Probe.EvaluatePosition(1.0),
			FVector2d(Solution.VerticalSegmentHalfWidthCm,
				Solution.VerticalSegmentTransitionHeightCm)));
	for (const TPair<double, FVector2d>& Endpoint : {
		TPair<double, FVector2d>(0.0, FVector2d(1.0, 0.0)),
		TPair<double, FVector2d>(1.0, FVector2d(0.0, 1.0)) })
	{
		const FVector2d Tangent = Probe.EvaluateFirstDerivative(Endpoint.Key);
		const double Dot = FMath::Abs(FVector2d::DotProduct(
			Tangent.GetSafeNormal(), Endpoint.Value));
		Solution.MaximumEndpointTangentResidualDegrees = FMath::Max(
			Solution.MaximumEndpointTangentResidualDegrees,
			FMath::RadiansToDegrees(FMath::Acos(FMath::Clamp(Dot, 0.0, 1.0))));
		Solution.MaximumEndpointSecondDerivativeMagnitude = FMath::Max(
			Solution.MaximumEndpointSecondDerivativeMagnitude,
			Probe.EvaluateSecondDerivative(Endpoint.Key).Length());
	}
	Solution.bCompleteFourMemberOrbit = Solution.MemberCount == 4;
	Solution.bExactXYMirrorPlacement = Solution.bCompleteFourMemberOrbit;
	Solution.bExactC0G1C2ByConstruction =
		Solution.MaximumEndpointPositionResidualCm <= 1.0e-9 &&
		Solution.MaximumEndpointTangentResidualDegrees <= 1.0e-9 &&
		Solution.MaximumEndpointSecondDerivativeMagnitude <= 1.0e-9;
	const double OpeningHeight = Solution.HorizontalSpanHeightCm -
		Solution.BaselineHeightCm;
	Solution.bSourceRegularizationPlausible =
		Solution.bExactC0G1C2ByConstruction &&
		Solution.BalancedRootMeanSquareResidualCm <= 0.05 * OpeningHeight &&
		Solution.MaximumMemberRootMeanSquareResidualCm <= 0.07 * OpeningHeight &&
		Solution.MaximumResidualCm <= 0.20 * OpeningHeight;
}

void FAnalyticWorldData::BuildOpenRimSurfaceBandObservations()
{
	OpenRimSurfaceBandObservations.Reset();
	for (int32 CandidateIndex = 0;
		CandidateIndex < OpenRimCandidates.Num(); ++CandidateIndex)
	{
		const FOpenRimCandidate& Candidate =
			OpenRimCandidates[CandidateIndex];
		if (!Candidate.bFeaturePartitionComplete ||
			!EndWallBoundaryComponents.IsValidIndex(
				Candidate.WallBoundaryComponentIndex))
		{
			continue;
		}
		const FEndWallBoundaryComponent& Component =
			EndWallBoundaryComponents[Candidate.WallBoundaryComponentIndex];
		if (!PlanarSurfaceGroups.IsValidIndex(Component.PlanarGroupIndex))
			continue;
		const FPlanarSurfaceGroup& Group =
			PlanarSurfaceGroups[Component.PlanarGroupIndex];
		TArray<bool> bTriangleInBoundaryPlane;
		bTriangleInBoundaryPlane.Init(false, Triangles.Num());
		for (int32 PatchOffset = 0; PatchOffset < Group.PatchCount; ++PatchOffset)
		{
			const FSurfacePatch& Patch = SurfacePatches[PlanarGroupPatchIndices[
				Group.FirstPatchIndex + PatchOffset]];
			for (int32 TriangleOffset = 0;
				TriangleOffset < Patch.TriangleCount; ++TriangleOffset)
			{
				bTriangleInBoundaryPlane[PatchTriangleIndices[
					Patch.FirstTriangleIndex + TriangleOffset]] = true;
			}
		}
		const double MinimumTransverse = Candidate.WallAxis == 0u
			? Candidate.Bounds.Min.Y : Candidate.Bounds.Min.X;
		const double MaximumTransverse = Candidate.WallAxis == 0u
			? Candidate.Bounds.Max.Y : Candidate.Bounds.Max.X;
		const double MaximumAbsTransverse = FMath::Max(
			FMath::Abs(MinimumTransverse), FMath::Abs(MaximumTransverse));
		const double HeightSpan = Candidate.Bounds.Max.Z -
			Candidate.BaselineHeightCm;
		for (int32 Offset = 0; Offset < Candidate.EdgeCount; ++Offset)
		{
			const int32 EdgeIndex = OpenRimEdgeIndices[
				Candidate.FirstEdgeIndex + Offset];
			const FTriangleMeshEdge& Edge = MeshEdges[EdgeIndex];
			if (!Edge.IsManifold()) continue;
			const int32 TriangleA = EdgeIncidentTriangleIndices[
				Edge.FirstIncidentTriangle];
			const int32 TriangleB = EdgeIncidentTriangleIndices[
				Edge.FirstIncidentTriangle + 1];
			if (bTriangleInBoundaryPlane[TriangleA] == bTriangleInBoundaryPlane[TriangleB])
				continue;
			const int32 BoundaryPlaneTriangle = bTriangleInBoundaryPlane[TriangleA]
				? TriangleA : TriangleB;
			const int32 OpeningTriangle = bTriangleInBoundaryPlane[TriangleA]
				? TriangleB : TriangleA;
			const FVector3d& A = MeshVertices[Edge.VertexA];
			const FVector3d& B = MeshVertices[Edge.VertexB];
			const double AT = Candidate.WallAxis == 0u ? A.Y : A.X;
			const double BT = Candidate.WallAxis == 0u ? B.Y : B.X;
			const double DeltaT = BT - AT;
			const double DeltaZ = B.Z - A.Z;
			const double Length = FMath::Sqrt(
				DeltaT * DeltaT + DeltaZ * DeltaZ);
			const double MidT = 0.5 * (AT + BT);
			const double MidZ = 0.5 * (A.Z + B.Z);
			const bool bHorizontalSpan = Length > 1.0e-9 &&
				FMath::Abs(DeltaZ) <= 0.02 * Length &&
				MidZ >= Candidate.BaselineHeightCm + 0.75 * HeightSpan;
			const bool bVerticalSegment = Length > 1.0e-9 &&
				FMath::Abs(DeltaT) <= 0.05 * Length &&
				FMath::Abs(MidT) >= 0.9 * MaximumAbsTransverse &&
				MidZ <= Candidate.BaselineHeightCm + 0.6 * HeightSpan;
			FOpenRimSurfaceBandObservation& Observation =
				OpenRimSurfaceBandObservations.AddDefaulted_GetRef();
			Observation.OpenRimCandidateIndex = CandidateIndex;
			Observation.RimEdgeIndex = EdgeIndex;
			Observation.BoundaryPlaneTriangleIndex = BoundaryPlaneTriangle;
			Observation.OpeningSurfaceTriangleIndex = OpeningTriangle;
			Observation.Feature = bHorizontalSpan
				? EOpenRimFeature::HorizontalSpan
				: bVerticalSegment && MidT < 0.0
					? EOpenRimFeature::NegativeVertical
					: bVerticalSegment ? EOpenRimFeature::PositiveVertical
						: EOpenRimFeature::UpperTransition;
			Observation.EdgeLengthCm = FVector3d::Distance(A, B);
			Observation.FaceDihedralDegrees = Edge.DihedralAngleDegrees;
			Observation.ImportedNormalJumpDegrees =
				Edge.MaximumNormalDiscontinuityDegrees;
			Observation.bSameSmoothRegion =
				TriangleSmoothRegionIndices.IsValidIndex(BoundaryPlaneTriangle) &&
				TriangleSmoothRegionIndices.IsValidIndex(OpeningTriangle) &&
				TriangleSmoothRegionIndices[BoundaryPlaneTriangle] ==
					TriangleSmoothRegionIndices[OpeningTriangle];
			const FIntVector& OpeningVertexIndices =
				TriangleVertexIndices[OpeningTriangle];
			int32 ThirdVertexIndex = INDEX_NONE;
			for (const int32 VertexIndex : {
				OpeningVertexIndices.X, OpeningVertexIndices.Y, OpeningVertexIndices.Z })
			{
				if (VertexIndex != Edge.VertexA && VertexIndex != Edge.VertexB)
				{
					ThirdVertexIndex = VertexIndex;
					break;
				}
			}
			if (MeshVertices.IsValidIndex(ThirdVertexIndex))
			{
				const FVector3d FirstStep = MeshVertices[ThirdVertexIndex] -
					0.5 * (A + B);
				Observation.FirstLongitudinalStepCm =
					FMath::Abs(FirstStep[Candidate.WallAxis]);
				if (!FirstStep.IsNearlyZero())
				{
					Observation.FirstStepAxisResidualDegrees =
						FMath::RadiansToDegrees(FMath::Acos(FMath::Clamp(
							FMath::Abs(FirstStep[Candidate.WallAxis]) /
								FirstStep.Length(), 0.0, 1.0)));
				}
			}
			Observation.bSourceG1Plausible = Observation.bSameSmoothRegion &&
				Observation.FaceDihedralDegrees <= 10.0 &&
				Observation.ImportedNormalJumpDegrees <= 10.0;
		}
	}
}

void FAnalyticWorldData::BuildOpenRimTransverseSections()
{
	OpenRimTransverseSections.Reset();
	OpenRimTransverseSectionPoints.Reset();
	OpenRimTransitionSectionPoints.Reset();
	OpenRimLongitudinalContinuationPoints.Reset();
	OpenRimLongitudinalSamples.Reset();
	if (CanonicalOpenArchSolutions.Num() != 1) return;
	const FCanonicalOpenArchSolution& Arch = CanonicalOpenArchSolutions[0];
	struct FSectionSegment
	{
		FVector2d A = FVector2d::ZeroVector;
		FVector2d B = FVector2d::ZeroVector;
	};
	struct FSectionSliceSpec
	{
		EOpenRimFeature Feature = EOpenRimFeature::UpperTransition;
		double CoordinateCm = 0.0;
		int32 RimEdgeIndex = INDEX_NONE;
		int32 SmoothRegionIndex = INDEX_NONE;
		FVector3d RimPoint = FVector3d::ZeroVector;
		FVector3d SlicePlaneNormal = FVector3d::ZeroVector;
		FVector3d OpeningDirection = FVector3d::ZeroVector;
		int32 InterpolationRimEdgeA = INDEX_NONE;
		int32 InterpolationRimEdgeB = INDEX_NONE;
		double InterpolationAlpha = 0.0;
		bool bUsesExplicitFrame = false;
	};
	auto SquaredDistanceToSegment = [](const FVector2d& Point,
		const FVector2d& A, const FVector2d& B)
	{
		const FVector2d AB = B - A;
		const double LengthSquared = AB.SquaredLength();
		if (LengthSquared <= 1.0e-18) return FVector2d::DistSquared(Point, A);
		const double T = FMath::Clamp(
			FVector2d::DotProduct(Point - A, AB) / LengthSquared, 0.0, 1.0);
		return FVector2d::DistSquared(Point, A + T * AB);
	};
	for (int32 CandidateIndex = 0;
		CandidateIndex < OpenRimCandidates.Num(); ++CandidateIndex)
	{
		const FOpenRimCandidate& Candidate =
			OpenRimCandidates[CandidateIndex];
		if (Candidate.SurfaceLayer != EC2TransitionSurfaceLayer::PlayableInner ||
			Candidate.WallAxis != 0u) continue;
		const double WallX = 0.5 * (Candidate.Bounds.Min.X + Candidate.Bounds.Max.X);
		const double OpeningSign = WallX >= 0.0 ? 1.0 : -1.0;
		const FEndWallBoundaryComponent& Component =
			EndWallBoundaryComponents[Candidate.WallBoundaryComponentIndex];
		const FPlanarSurfaceGroup& BoundaryPlaneGroup =
			PlanarSurfaceGroups[Component.PlanarGroupIndex];
		TArray<bool> bTriangleInBoundaryPlane;
		bTriangleInBoundaryPlane.Init(false, Triangles.Num());
		for (int32 PatchOffset = 0;
			PatchOffset < BoundaryPlaneGroup.PatchCount; ++PatchOffset)
		{
			const FSurfacePatch& Patch = SurfacePatches[PlanarGroupPatchIndices[
				BoundaryPlaneGroup.FirstPatchIndex + PatchOffset]];
			for (int32 TriangleOffset = 0;
				TriangleOffset < Patch.TriangleCount; ++TriangleOffset)
			{
				bTriangleInBoundaryPlane[PatchTriangleIndices[
					Patch.FirstTriangleIndex + TriangleOffset]] = true;
			}
		}
		TArray<FSectionSliceSpec> SliceSpecs;
		// The terminal continuation is a two-dimensional source sheet, so its
		// transverse witnesses must cover the whole finite vertical family.  The
		// earlier five lower-half cuts were sufficient to recognize the rim run,
		// but left the rear closure interpolating across an unwitnessed half-cell.
		// These are still exact mesh-plane intersections, not synthetic loft rows.
		for (const double HeightFraction : { 0.05, 0.075, 0.1, 0.15, 0.2,
			0.25, 0.3, 0.35, 0.4, 0.45, 0.5, 0.55, 0.6, 0.65, 0.7, 0.75,
			0.8, 0.80625, 0.8125, 0.81875, 0.8203125, 0.821875,
			0.8234375, 0.825, 0.85, 0.9, 0.95 })
		{
			const double Height = FMath::Lerp(Arch.BaselineHeightCm,
				Arch.VerticalSegmentTransitionHeightCm, HeightFraction);
			SliceSpecs.Add({ EOpenRimFeature::NegativeVertical, Height });
			SliceSpecs.Add({ EOpenRimFeature::PositiveVertical, Height });
		}
		// The finite side turns become visible only near the lateral rim ends.
		// Keep those transverse rails explicitly symmetric and topology-led rather
		// than asking the spline to extrapolate the last quarter of the domain.
		for (const double SpanFraction : { -0.95, -0.875, -0.8125, -0.75,
			-0.5, -0.25, 0.0, 0.25, 0.5, 0.75, 0.8125, 0.875, 0.95 })
		{
			SliceSpecs.Add({ EOpenRimFeature::HorizontalSpan,
				SpanFraction * Arch.HorizontalSpanHalfSpanCm });
		}
		for (const FOpenRimSurfaceBandObservation& Observation :
			OpenRimSurfaceBandObservations)
		{
			if (Observation.OpenRimCandidateIndex != CandidateIndex ||
				Observation.Feature != EOpenRimFeature::UpperTransition ||
				!MeshEdges.IsValidIndex(Observation.RimEdgeIndex) ||
				!TriangleSmoothRegionIndices.IsValidIndex(
					Observation.OpeningSurfaceTriangleIndex))
			{
				continue;
			}
			const FTriangleMeshEdge& RimEdge = MeshEdges[Observation.RimEdgeIndex];
			const FVector3d& A = MeshVertices[RimEdge.VertexA];
			const FVector3d& B = MeshVertices[RimEdge.VertexB];
			const FVector3d RimPoint = 0.5 * (A + B);
			FVector3d Tangent(0.0, B.Y - A.Y, B.Z - A.Z);
			Tangent = Tangent.GetSafeNormal();
			if (Tangent.IsNearlyZero()) continue;
			FVector3d OpeningDirection(0.0, -Tangent.Z, Tangent.Y);
			const FVector3d OpeningReference(WallX, 0.0,
				Arch.BaselineHeightCm);
			if (FVector3d::DotProduct(OpeningDirection,
				OpeningReference - RimPoint) < 0.0)
			{
				OpeningDirection *= -1.0;
			}
			FSectionSliceSpec& Spec = SliceSpecs.AddDefaulted_GetRef();
			Spec.Feature = EOpenRimFeature::UpperTransition;
			Spec.CoordinateCm = FMath::Abs(RimPoint.Y);
			Spec.RimEdgeIndex = Observation.RimEdgeIndex;
			Spec.SmoothRegionIndex = TriangleSmoothRegionIndices[
				Observation.OpeningSurfaceTriangleIndex];
			Spec.RimPoint = RimPoint;
			Spec.SlicePlaneNormal = Tangent;
			Spec.OpeningDirection = OpeningDirection;
			Spec.bUsesExplicitFrame = true;
		}
		// The curved upper family may have only a few rim edges even though its
		// finite continuation changes substantially between them.  Insert compact
		// mesh-plane cuts at the quartiles of adjacent source edges.
		// The resulting rails are source observations; only their frame is
		// interpolated from the adjacent rim tangents.
		TArray<FSectionSliceSpec> UpperSourceSpecs;
		for (const FSectionSliceSpec& Spec : SliceSpecs)
			if (Spec.Feature == EOpenRimFeature::UpperTransition &&
				Spec.bUsesExplicitFrame && Spec.RimEdgeIndex != INDEX_NONE)
				UpperSourceSpecs.Add(Spec);
		Algo::Sort(UpperSourceSpecs, [](const FSectionSliceSpec& A,
			const FSectionSliceSpec& B)
		{
			const int32 SideA = A.RimPoint.Y < 0.0 ? -1 : 1;
			const int32 SideB = B.RimPoint.Y < 0.0 ? -1 : 1;
			return SideA != SideB ? SideA < SideB :
				A.CoordinateCm > B.CoordinateCm;
		});
		for (int32 Index = 0; Index + 1 < UpperSourceSpecs.Num(); ++Index)
		{
			const FSectionSliceSpec& A = UpperSourceSpecs[Index];
			const FSectionSliceSpec& B = UpperSourceSpecs[Index + 1];
			if ((A.RimPoint.Y < 0.0) != (B.RimPoint.Y < 0.0) ||
				A.SmoothRegionIndex != B.SmoothRegionIndex ||
				FVector3d::Distance(A.RimPoint, B.RimPoint) <= 1.0e-6)
				continue;
			FVector3d BNormal = B.SlicePlaneNormal;
			if (FVector3d::DotProduct(A.SlicePlaneNormal, BNormal) < 0.0)
				BNormal *= -1.0;
			FVector3d BOpening = B.OpeningDirection;
			if (FVector3d::DotProduct(A.OpeningDirection, BOpening) < 0.0)
				BOpening *= -1.0;
			for (const double Alpha : { 0.0625, 0.125, 0.1875, 0.25,
				0.3125, 0.375, 0.4375, 0.5, 0.5625, 0.625, 0.6875,
				0.75, 0.8125, 0.875, 0.9375 })
			{
				FSectionSliceSpec& Spec = SliceSpecs.AddDefaulted_GetRef();
				Spec.Feature = EOpenRimFeature::UpperTransition;
				Spec.RimEdgeIndex = INDEX_NONE;
				Spec.SmoothRegionIndex = A.SmoothRegionIndex;
				Spec.RimPoint = FMath::Lerp(A.RimPoint, B.RimPoint, Alpha);
				Spec.CoordinateCm = FMath::Abs(Spec.RimPoint.Y);
				Spec.SlicePlaneNormal = FMath::Lerp(
					A.SlicePlaneNormal, BNormal, Alpha).GetSafeNormal();
				Spec.OpeningDirection = FMath::Lerp(
					A.OpeningDirection, BOpening, Alpha).GetSafeNormal();
				Spec.InterpolationRimEdgeA = A.RimEdgeIndex;
				Spec.InterpolationRimEdgeB = B.RimEdgeIndex;
				Spec.InterpolationAlpha = Alpha;
				Spec.bUsesExplicitFrame = !Spec.SlicePlaneNormal.IsNearlyZero() &&
					!Spec.OpeningDirection.IsNearlyZero();
			}
		}
		for (const FSectionSliceSpec& SliceSpec : SliceSpecs)
		{
			const EOpenRimFeature Feature = SliceSpec.Feature;
			const bool bHorizontalSpan = Feature == EOpenRimFeature::HorizontalSpan;
			const double Side = Feature == EOpenRimFeature::NegativeVertical ? -1.0 : 1.0;
			const int32 SliceAxis = bHorizontalSpan ? 1 : 2;
			const double SliceCoordinate = SliceSpec.CoordinateCm;
			double RimSecondary = 0.0;
			bool bRimSecondaryFound = SliceSpec.bUsesExplicitFrame;
			int32 SmoothRegionIndex = SliceSpec.SmoothRegionIndex;
			if (!SliceSpec.bUsesExplicitFrame)
			{
				for (const FOpenRimSurfaceBandObservation& Observation :
					OpenRimSurfaceBandObservations)
				{
					if (Observation.OpenRimCandidateIndex == CandidateIndex &&
						Observation.Feature == Feature &&
						TriangleSmoothRegionIndices.IsValidIndex(
							Observation.OpeningSurfaceTriangleIndex))
					{
						SmoothRegionIndex = TriangleSmoothRegionIndices[
							Observation.OpeningSurfaceTriangleIndex];
						const FTriangleMeshEdge& RimEdge =
							MeshEdges[Observation.RimEdgeIndex];
						const FVector3d& A = MeshVertices[RimEdge.VertexA];
						const FVector3d& B = MeshVertices[RimEdge.VertexB];
						const double DA = A[SliceAxis] - SliceCoordinate;
						const double DB = B[SliceAxis] - SliceCoordinate;
						if (FMath::Abs(DA) <= 1.0e-8)
						{
							RimSecondary = bHorizontalSpan ? A.Z : A.Y;
							bRimSecondaryFound = true;
						}
						else if (DA * DB <= 0.0 && FMath::Abs(DB - DA) > 1.0e-12)
						{
							const FVector3d Point = A + (-DA / (DB - DA)) * (B - A);
							RimSecondary = bHorizontalSpan ? Point.Z : Point.Y;
							bRimSecondaryFound = true;
						}
						if (bRimSecondaryFound) break;
					}
				}
			}
			if (SmoothRegionIndex == INDEX_NONE || !bRimSecondaryFound) continue;
			const FVector3d SliceOrigin = SliceSpec.bUsesExplicitFrame
				? SliceSpec.RimPoint
				: bHorizontalSpan
					? FVector3d(WallX, SliceCoordinate, RimSecondary)
					: FVector3d(WallX, RimSecondary, SliceCoordinate);
			const FVector3d SlicePlaneNormal = SliceSpec.bUsesExplicitFrame
				? SliceSpec.SlicePlaneNormal
				: bHorizontalSpan ? FVector3d::YAxisVector : FVector3d::ZAxisVector;
			const FVector3d OpeningDirection = SliceSpec.bUsesExplicitFrame
				? SliceSpec.OpeningDirection
				: bHorizontalSpan ? -FVector3d::ZAxisVector
					: FVector3d(0.0, -Side, 0.0);
			auto ToLocal = [&](const FVector3d& Point)
			{
				const double Depth = OpeningSign * (Point.X - WallX);
				const double OpeningOffset = FVector3d::DotProduct(
					Point - SliceOrigin, OpeningDirection);
				return FVector2d(Depth, OpeningOffset);
			};
			TArray<FSectionSegment> Segments;
			for (int32 TriangleIndex = 0;
				TriangleIndex < Triangles.Num(); ++TriangleIndex)
			{
				if (!TriangleSmoothRegionIndices.IsValidIndex(TriangleIndex) ||
					TriangleSmoothRegionIndices[TriangleIndex] != SmoothRegionIndex ||
					bTriangleInBoundaryPlane[TriangleIndex])
					continue;
				const FTriangleSurface& Triangle = Triangles[TriangleIndex];
				TArray<FVector3d> Intersections;
				auto AddUnique = [&](const FVector3d& Point)
				{
					if (!Algo::AnyOf(Intersections, [&](const FVector3d& Existing)
					{
						return FVector3d::DistSquared(Existing, Point) <= 1.0e-12;
					})) Intersections.Add(Point);
				};
				for (int32 EdgeCorner = 0; EdgeCorner < 3; ++EdgeCorner)
				{
					const FVector3d& A = Triangle.Vertices[EdgeCorner];
					const FVector3d& B = Triangle.Vertices[(EdgeCorner + 1) % 3];
					const double DA = FVector3d::DotProduct(
						A - SliceOrigin, SlicePlaneNormal);
					const double DB = FVector3d::DotProduct(
						B - SliceOrigin, SlicePlaneNormal);
					if (FMath::Abs(DA) <= 1.0e-8) AddUnique(A);
					if (DA * DB < 0.0)
					{
						AddUnique(A + (-DA / (DB - DA)) * (B - A));
					}
				}
				if (Intersections.Num() != 2) continue;
				const FVector2d LocalA = ToLocal(Intersections[0]);
				const FVector2d LocalB = ToLocal(Intersections[1]);
				const FVector2d Midpoint = 0.5 * (LocalA + LocalB);
				if (Midpoint.X < -25.0 || Midpoint.X > 1400.0 ||
					Midpoint.Y < -50.0 || Midpoint.Y > 600.0)
					continue;
				Segments.Add({ LocalA, LocalB });
			}
			if (Segments.IsEmpty()) continue;
			int32 SeedSegment = INDEX_NONE;
			double BestRimSquared = TNumericLimits<double>::Max();
			for (int32 SegmentIndex = 0;
				SegmentIndex < Segments.Num(); ++SegmentIndex)
			{
				const double Squared = SquaredDistanceToSegment(
					FVector2d::ZeroVector, Segments[SegmentIndex].A,
					Segments[SegmentIndex].B);
				if (Squared < BestRimSquared)
				{
					BestRimSquared = Squared;
					SeedSegment = SegmentIndex;
				}
			}
			TArray<int32> Pending = { SeedSegment };
			TSet<int32> Selected;
			constexpr double JoinToleranceSquared = 0.25 * 0.25;
			while (!Pending.IsEmpty())
			{
				const int32 SegmentIndex = Pending.Pop(EAllowShrinking::No);
				if (Selected.Contains(SegmentIndex)) continue;
				Selected.Add(SegmentIndex);
				const FSectionSegment& Current = Segments[SegmentIndex];
				for (int32 OtherIndex = 0; OtherIndex < Segments.Num(); ++OtherIndex)
				{
					if (Selected.Contains(OtherIndex)) continue;
					const FSectionSegment& Other = Segments[OtherIndex];
					const double EndpointDistanceSquared = FMath::Min(
						FMath::Min(FVector2d::DistSquared(Current.A, Other.A),
							FVector2d::DistSquared(Current.A, Other.B)),
						FMath::Min(FVector2d::DistSquared(Current.B, Other.A),
							FVector2d::DistSquared(Current.B, Other.B)));
					if (EndpointDistanceSquared <=
						JoinToleranceSquared)
					{
						Pending.Add(OtherIndex);
					}
				}
			}
			TArray<int32> SelectedOrder = Selected.Array();
			Algo::Sort(SelectedOrder);
			TMap<int32, int32> SegmentHops;
			TMap<int32, int32> SegmentPredecessors;
			SegmentHops.Add(SeedSegment, 0);
			SegmentPredecessors.Add(SeedSegment, INDEX_NONE);
			TArray<int32> HopPending = { SeedSegment };
			while (!HopPending.IsEmpty())
			{
				const int32 SegmentIndex = HopPending.Pop(EAllowShrinking::No);
				const int32 NextHop = SegmentHops[SegmentIndex] + 1;
				const FSectionSegment& Current = Segments[SegmentIndex];
				for (const int32 OtherIndex : SelectedOrder)
				{
					if (SegmentHops.Contains(OtherIndex)) continue;
					const FSectionSegment& Other = Segments[OtherIndex];
					const double EndpointDistanceSquared = FMath::Min(
						FMath::Min(FVector2d::DistSquared(Current.A, Other.A),
							FVector2d::DistSquared(Current.A, Other.B)),
						FMath::Min(FVector2d::DistSquared(Current.B, Other.A),
							FVector2d::DistSquared(Current.B, Other.B)));
					if (EndpointDistanceSquared <= JoinToleranceSquared)
					{
						SegmentHops.Add(OtherIndex, NextHop);
						SegmentPredecessors.Add(OtherIndex, SegmentIndex);
						HopPending.Add(OtherIndex);
					}
				}
			}
			FOpenRimTransverseSection& Section =
				OpenRimTransverseSections.AddDefaulted_GetRef();
			uint64 SliceCoordinateBits = 0;
			static_assert(sizeof(SliceCoordinateBits) == sizeof(SliceCoordinate));
			FMemory::Memcpy(&SliceCoordinateBits, &SliceCoordinate,
				sizeof(SliceCoordinateBits));
			Section.SectionId = CombineStableIds(
				CombineStableIds(Candidate.CandidateId,
					static_cast<uint64>(Feature) + 1ull),
				SliceSpec.RimEdgeIndex == INDEX_NONE ? SliceCoordinateBits
					: static_cast<uint64>(SliceSpec.RimEdgeIndex) + 1ull);
			Section.OpenRimCandidateIndex = CandidateIndex;
			Section.RimEdgeIndex = SliceSpec.RimEdgeIndex;
			Section.InterpolationRimEdgeA = SliceSpec.InterpolationRimEdgeA;
			Section.InterpolationRimEdgeB = SliceSpec.InterpolationRimEdgeB;
			Section.InterpolationAlpha = SliceSpec.InterpolationAlpha;
			Section.Feature = Feature;
			Section.SliceCoordinateCm = SliceCoordinate;
			Section.SliceOrigin = SliceOrigin;
			Section.SlicePlaneNormal = SlicePlaneNormal;
			Section.OpeningDirection = OpeningDirection;
			Section.FirstPointIndex = OpenRimTransverseSectionPoints.Num();
			Section.FirstTransitionPointIndex =
				OpenRimTransitionSectionPoints.Num();
			Section.SegmentCount = SelectedOrder.Num();
			Section.RimDistanceCm = FMath::Sqrt(BestRimSquared);
			double BestLongitudinalTangentResidual =
				TNumericLimits<double>::Max();
			int32 FirstLongitudinalHop = TNumericLimits<int32>::Max();
			int32 FirstLongitudinalSegment = INDEX_NONE;
			TArray<FVector2d> UniquePoints;
			for (const int32 SegmentIndex : SelectedOrder)
			{
				const FSectionSegment& Segment = Segments[SegmentIndex];
				Section.ArcLengthCm += FVector2d::Distance(Segment.A, Segment.B);
				for (const FVector2d& Point : { Segment.A, Segment.B })
				{
					Section.Bounds += Point;
					Section.MaximumLongitudinalDepthCm = FMath::Max(
						Section.MaximumLongitudinalDepthCm, Point.X);
					Section.MaximumOpeningOffsetCm = FMath::Max(
						Section.MaximumOpeningOffsetCm, Point.Y);
					if (!Algo::AnyOf(UniquePoints, [&](const FVector2d& Existing)
					{
						return FVector2d::DistSquared(Existing, Point) <= 1.0e-12;
					})) UniquePoints.Add(Point);
				}
				const FVector2d Direction =
					(Segment.B - Segment.A).GetSafeNormal();
				const double MidDepth = 0.5 * (Segment.A.X + Segment.B.X);
				const double LongitudinalResidual = FMath::RadiansToDegrees(
					FMath::Acos(FMath::Clamp(FMath::Abs(Direction.X), 0.0, 1.0)));
				const int32 Hop = SegmentHops.FindRef(SegmentIndex);
				if (MidDepth >= 25.0 && LongitudinalResidual <= 10.0 &&
					(Hop < FirstLongitudinalHop ||
						(Hop == FirstLongitudinalHop && LongitudinalResidual <
							BestLongitudinalTangentResidual)))
				{
					const FVector2d TransitionPoint =
						Segment.A.SquaredLength() <= Segment.B.SquaredLength()
							? Segment.A : Segment.B;
					FirstLongitudinalHop = Hop;
					FirstLongitudinalSegment = SegmentIndex;
					BestLongitudinalTangentResidual = LongitudinalResidual;
					Section.LongitudinalTangentDepthCm = TransitionPoint.X;
					Section.LongitudinalTangentOpeningOffsetCm =
						TransitionPoint.Y;
				}
			}
			Section.LongitudinalTransitionHop = FirstLongitudinalHop ==
				TNumericLimits<int32>::Max() ? INDEX_NONE : FirstLongitudinalHop;
			TArray<FVector2d> TransitionPoints;
			if (FirstLongitudinalSegment != INDEX_NONE)
			{
				TArray<int32> TransitionPath;
				for (int32 SegmentIndex = FirstLongitudinalSegment;
					SegmentIndex != INDEX_NONE;
					SegmentIndex = SegmentPredecessors.FindRef(SegmentIndex))
				{
					TransitionPath.Add(SegmentIndex);
				}
				Algo::Reverse(TransitionPath);
				for (const int32 SegmentIndex : TransitionPath)
				{
					++Section.TransitionSegmentCount;
					const FSectionSegment& Segment = Segments[SegmentIndex];
					for (const FVector2d& Point : { Segment.A, Segment.B })
					{
						if (SegmentIndex == FirstLongitudinalSegment &&
							Point.X > Section.LongitudinalTangentDepthCm + 1.0e-6)
						{
							continue;
						}
						if (!Algo::AnyOf(TransitionPoints,
							[&](const FVector2d& Existing)
							{
								return FVector2d::DistSquared(Existing, Point) <=
									1.0e-12;
							})) TransitionPoints.Add(Point);
					}
				}
			}
			Algo::Sort(TransitionPoints,
				[](const FVector2d& A, const FVector2d& B)
				{
					return A.X != B.X ? A.X < B.X : A.Y < B.Y;
				});
			Section.TransitionPointCount = TransitionPoints.Num();
			OpenRimTransitionSectionPoints.Append(TransitionPoints);
			Section.FirstLongitudinalContinuationPointIndex =
				OpenRimLongitudinalContinuationPoints.Num();
			if (FirstLongitudinalSegment != INDEX_NONE)
			{
				// Start at the terminal of the rim-to-run path, then walk the
				// same selected source topology into the finite interior.  This
				// deliberately preserves adjacency instead of sorting all section
				// intersections by depth (which is the invalid center-out graph).
				TSet<int32> UsedSegments;
				for (int32 SegmentIndex = FirstLongitudinalSegment;
					SegmentIndex != INDEX_NONE;
					SegmentIndex = SegmentPredecessors.FindRef(SegmentIndex))
				{
					UsedSegments.Add(SegmentIndex);
				}
				const FSectionSegment& FirstSegment =
					Segments[FirstLongitudinalSegment];
				FVector2d CurrentPoint = FirstSegment.A.SquaredLength() <=
					FirstSegment.B.SquaredLength() ? FirstSegment.A : FirstSegment.B;
				FVector2d PreviousPoint = CurrentPoint;
				OpenRimLongitudinalContinuationPoints.Add(CurrentPoint);
				CurrentPoint = CurrentPoint.Equals(FirstSegment.A, 0.25)
					? FirstSegment.B : FirstSegment.A;
				OpenRimLongitudinalContinuationPoints.Add(CurrentPoint);
				UsedSegments.Add(FirstLongitudinalSegment);
				for (int32 WalkStep = 0; WalkStep < SelectedOrder.Num(); ++WalkStep)
				{
					int32 BestNextSegment = INDEX_NONE;
					FVector2d BestNextPoint = FVector2d::ZeroVector;
					double BestScore = -TNumericLimits<double>::Max();
					const FVector2d Direction = (CurrentPoint - PreviousPoint)
						.GetSafeNormal();
					for (const int32 CandidateSegmentIndex : SelectedOrder)
					{
						if (UsedSegments.Contains(CandidateSegmentIndex)) continue;
						const FSectionSegment& LaneCandidate =
							Segments[CandidateSegmentIndex];
						const bool bAtA = LaneCandidate.A.Equals(CurrentPoint, 0.25);
						const bool bAtB = LaneCandidate.B.Equals(CurrentPoint, 0.25);
						if (!bAtA && !bAtB) continue;
						const FVector2d NextPoint = bAtA ? LaneCandidate.B : LaneCandidate.A;
						const FVector2d NextDirection = (NextPoint - CurrentPoint)
							.GetSafeNormal();
						const double Score = FVector2d::DotProduct(
							Direction, NextDirection) + 1.0e-6 * NextPoint.X;
						if (Score > BestScore || (Score == BestScore &&
							CandidateSegmentIndex < BestNextSegment))
						{
							BestScore = Score;
							BestNextSegment = CandidateSegmentIndex;
							BestNextPoint = NextPoint;
						}
					}
					if (BestNextSegment == INDEX_NONE) break;
					UsedSegments.Add(BestNextSegment);
					PreviousPoint = CurrentPoint;
					CurrentPoint = BestNextPoint;
					OpenRimLongitudinalContinuationPoints.Add(CurrentPoint);
				}
			}
			Section.LongitudinalContinuationPointCount =
				OpenRimLongitudinalContinuationPoints.Num() -
				Section.FirstLongitudinalContinuationPointIndex;
			for (int32 PointOffset = 1;
				PointOffset < Section.LongitudinalContinuationPointCount;
				++PointOffset)
			{
				const FVector2d& Previous = OpenRimLongitudinalContinuationPoints[
					Section.FirstLongitudinalContinuationPointIndex + PointOffset - 1];
				const FVector2d& Current = OpenRimLongitudinalContinuationPoints[
					Section.FirstLongitudinalContinuationPointIndex + PointOffset];
				Section.LongitudinalContinuationArcLengthCm +=
					FVector2d::Distance(Previous, Current);
				if (PointOffset >= 2)
				{
					const FVector2d& BeforePrevious =
						OpenRimLongitudinalContinuationPoints[
							Section.FirstLongitudinalContinuationPointIndex + PointOffset - 2];
					const FVector2d Incoming = (Previous - BeforePrevious).GetSafeNormal();
					const FVector2d Outgoing = (Current - Previous).GetSafeNormal();
					if (!Incoming.IsNearlyZero() && !Outgoing.IsNearlyZero())
					{
						Section.LongitudinalContinuationMaximumTurnDegrees = FMath::Max(
							Section.LongitudinalContinuationMaximumTurnDegrees,
							FMath::RadiansToDegrees(FMath::Acos(FMath::Clamp(
								FVector2d::DotProduct(Incoming, Outgoing), -1.0, 1.0))));
					}
				}
			}
			// The selected topology remains ordered through the rear rotation and
			// cap.  A tube fit must stop before that bend rather than averaging the
			// cap into its longitudinal derivatives.  Use the first non-degenerate
			// tangent as the local longitudinal witness and split at its first
			// sustained 15-degree departure.  This is recognition-only evidence;
			// runtime collision authority still cannot consume it directly.
			constexpr double TubeDepartureDegrees = 15.0;
			const double TubeDepartureCosine = FMath::Cos(
				FMath::DegreesToRadians(TubeDepartureDegrees));
			FVector2d TubeDirection = FVector2d::ZeroVector;
			for (int32 PointOffset = 1;
				PointOffset < Section.LongitudinalContinuationPointCount;
				++PointOffset)
			{
				const FVector2d Segment =
					OpenRimLongitudinalContinuationPoints[
						Section.FirstLongitudinalContinuationPointIndex + PointOffset] -
					OpenRimLongitudinalContinuationPoints[
						Section.FirstLongitudinalContinuationPointIndex + PointOffset - 1];
				if (Segment.SquaredLength() > UE_DOUBLE_SMALL_NUMBER)
				{
					TubeDirection = Segment.GetSafeNormal();
					break;
				}
			}
			Section.LongitudinalTubePointCount =
				Section.LongitudinalContinuationPointCount;
			if (!TubeDirection.IsNearlyZero())
			{
				for (int32 PointOffset = 1;
					PointOffset < Section.LongitudinalContinuationPointCount;
					++PointOffset)
				{
					const FVector2d Segment =
						OpenRimLongitudinalContinuationPoints[
							Section.FirstLongitudinalContinuationPointIndex + PointOffset] -
						OpenRimLongitudinalContinuationPoints[
							Section.FirstLongitudinalContinuationPointIndex + PointOffset - 1];
					if (Segment.SquaredLength() <= UE_DOUBLE_SMALL_NUMBER) continue;
					const double Alignment = FMath::Clamp(FVector2d::DotProduct(
						TubeDirection, Segment.GetSafeNormal()), -1.0, 1.0);
					const double DepartureDegrees = FMath::RadiansToDegrees(
						FMath::Acos(Alignment));
					Section.LongitudinalTubeMaximumDepartureDegrees = FMath::Max(
						Section.LongitudinalTubeMaximumDepartureDegrees,
						DepartureDegrees);
					if (Alignment < TubeDepartureCosine)
					{
						Section.FirstLongitudinalRearTurnPointOffset = PointOffset;
						Section.LongitudinalTubePointCount = PointOffset;
						break;
					}
				}
			}
			for (int32 PointOffset = 1;
				PointOffset < Section.LongitudinalTubePointCount; ++PointOffset)
			{
				Section.LongitudinalTubeArcLengthCm += FVector2d::Distance(
					OpenRimLongitudinalContinuationPoints[
						Section.FirstLongitudinalContinuationPointIndex + PointOffset - 1],
					OpenRimLongitudinalContinuationPoints[
						Section.FirstLongitudinalContinuationPointIndex + PointOffset]);
			}
			if (Section.LongitudinalTubePointCount > 0)
			{
				const FVector2d& TubeTerminal = OpenRimLongitudinalContinuationPoints[
					Section.FirstLongitudinalContinuationPointIndex +
					Section.LongitudinalTubePointCount - 1];
				Section.LongitudinalTubeTerminalDepthCm = TubeTerminal.X;
				Section.LongitudinalTubeTerminalOpeningOffsetCm = TubeTerminal.Y;
			}
			if (Section.LongitudinalContinuationPointCount > 0)
			{
				const FVector2d& Terminal = OpenRimLongitudinalContinuationPoints[
					Section.FirstLongitudinalContinuationPointIndex +
					Section.LongitudinalContinuationPointCount - 1];
				Section.LongitudinalContinuationTerminalDepthCm = Terminal.X;
				Section.LongitudinalContinuationTerminalOpeningOffsetCm = Terminal.Y;
			}
			Section.FirstLongitudinalSampleIndex =
				OpenRimLongitudinalSamples.Num();
			double CumulativeArcLengthCm = 0.0;
			for (int32 PointOffset = 0;
				PointOffset < Section.LongitudinalContinuationPointCount;
				++PointOffset)
			{
				const FVector2d& Point = OpenRimLongitudinalContinuationPoints[
					Section.FirstLongitudinalContinuationPointIndex + PointOffset];
				if (PointOffset > 0)
				{
					const FVector2d& Previous =
						OpenRimLongitudinalContinuationPoints[
							Section.FirstLongitudinalContinuationPointIndex + PointOffset - 1];
					CumulativeArcLengthCm += FVector2d::Distance(Previous, Point);
				}
				FOpenRimLongitudinalSample& Sample =
					OpenRimLongitudinalSamples.AddDefaulted_GetRef();
				Sample.OpenRimTransverseSectionIndex =
					OpenRimTransverseSections.Num() - 1;
				Sample.ContinuationPointOffset = PointOffset;
				Sample.NormalizedArcLength =
					Section.LongitudinalContinuationArcLengthCm > 1.0e-9
						? CumulativeArcLengthCm /
							Section.LongitudinalContinuationArcLengthCm : 0.0;
				const FVector3d WorldPoint = Section.SliceOrigin +
					Point.Y * Section.OpeningDirection;
				Sample.CanonicalPositionCm = FVector3d(
					Point.X, FMath::Abs(WorldPoint.Y), WorldPoint.Z);
			}
			Algo::Sort(UniquePoints, [](const FVector2d& A, const FVector2d& B)
			{
				return A.X != B.X ? A.X < B.X : A.Y < B.Y;
			});
			Section.PointCount = UniquePoints.Num();
			OpenRimTransverseSectionPoints.Append(UniquePoints);
			const FVector2d StartDirection =
				(Segments[SeedSegment].B - Segments[SeedSegment].A).GetSafeNormal();
			Section.BoundaryPlaneTangentResidualDegrees = FMath::RadiansToDegrees(
				FMath::Acos(FMath::Clamp(FMath::Abs(StartDirection.Y), 0.0, 1.0)));
			Section.LongitudinalTangentResidualDegrees =
				BestLongitudinalTangentResidual;
			Section.bConnectedToRim = Section.RimDistanceCm <= 20.0;
			Section.bSourceBoundaryPlaneG1Plausible = Section.RimDistanceCm <= 5.0 &&
				Section.BoundaryPlaneTangentResidualDegrees <= 10.0;
			Section.bLongitudinalRunPlausible =
				FMath::IsFinite(Section.LongitudinalTangentResidualDegrees) &&
				Section.LongitudinalTangentResidualDegrees <= 10.0;
		}
	}
}

void FAnalyticWorldData::BuildOpenRimTransitionFamilyFits()
{
	OpenRimTransitionFamilyFits.Reset();
	OpenRimTransitionMemberFits.Reset();
	OpenRimC2LoftStations.Reset();
	OpenRimC2LoftSegments.Reset();
	OpenRimC2LoftFits.Reset();
	OpenRimCanonicalSurfaceSamples.Reset();
	OpenRimCanonicalSectionCorrespondences.Reset();
	OpenRimLongitudinalSplineSegments.Reset();
	OpenRimTerminalClosureRailSegments.Reset();
	OpenRimTerminalClosureRailFits.Reset();
	OpenRimCanonicalSurfaceFits.Reset();
	OpenRimCanonicalTubeFits.Reset();
	OpenRimCanonicalTensorSurfaces.Reset();
	OpenRimCanonicalTubeTensorSurfaces.Reset();
	OpenRimCanonicalTubeLoftFits.Reset();
	if (CanonicalOpenArchSolutions.Num() != 1) return;
	const FCanonicalOpenArchSolution& Arch = CanonicalOpenArchSolutions[0];
	auto Evaluate = [](const double T, const double EndDepth,
		const double EndOffset, const double BoundaryPlaneTangent,
		const double LongitudinalTangent)
	{
		const double U = 1.0 - T;
		const double B1 = 5.0 * U * U * U * U * T;
		const double B2 = 10.0 * U * U * U * T * T;
		const double B3 = 10.0 * U * U * T * T * T;
		const double B4 = 5.0 * U * T * T * T * T;
		const double B5 = T * T * T * T * T;
		const FVector2d P1(0.0, BoundaryPlaneTangent / 5.0);
		const FVector2d P2(0.0, 2.0 * BoundaryPlaneTangent / 5.0);
		const FVector2d P3(EndDepth - 2.0 * LongitudinalTangent / 5.0,
			EndOffset);
		const FVector2d P4(EndDepth - LongitudinalTangent / 5.0,
			EndOffset);
		const FVector2d P5(EndDepth, EndOffset);
		return B1 * P1 + B2 * P2 + B3 * P3 + B4 * P4 + B5 * P5;
	};
	for (FOpenRimTransverseSection& Section : OpenRimTransverseSections)
	{
		if (Section.Feature == EOpenRimFeature::NegativeVertical ||
			Section.Feature == EOpenRimFeature::PositiveVertical)
		{
			Section.CanonicalRimParameter = FMath::Clamp(
				(Section.SliceCoordinateCm - Arch.BaselineHeightCm) /
					(Arch.VerticalSegmentTransitionHeightCm - Arch.BaselineHeightCm),
				0.0, 1.0) / 3.0;
		}
		else if (Section.Feature == EOpenRimFeature::HorizontalSpan)
		{
			Section.CanonicalRimParameter = 2.0 / 3.0 +
				FMath::Clamp((Arch.HorizontalSpanHalfSpanCm -
					FMath::Abs(Section.SliceCoordinateCm)) /
					Arch.HorizontalSpanHalfSpanCm, 0.0, 1.0) / 3.0;
		}
		else
		{
			const FVector2d RimPoint(FMath::Abs(Section.SliceOrigin.Y),
				Section.SliceOrigin.Z);
			const FVector2d Center(Arch.HorizontalSpanHalfSpanCm,
				Arch.VerticalSegmentTransitionHeightCm);
			auto SquaredResidual = [&](const double T)
			{
				return FVector2d::DistSquared(RimPoint,
					EvaluateQuinticTransitionPosition(Center,
						Arch.VerticalSegmentHalfWidthCm - Arch.HorizontalSpanHalfSpanCm,
						Arch.HorizontalSpanHeightCm - Arch.VerticalSegmentTransitionHeightCm,
						1.0, 1.0, Arch.FlatteningFraction, T));
			};
			int32 BestSample = 0;
			double BestSquared = SquaredResidual(0.0);
			for (int32 Sample = 1; Sample <= 128; ++Sample)
			{
				const double Squared = SquaredResidual(
					static_cast<double>(Sample) / 128.0);
				if (Squared < BestSquared)
				{
					BestSquared = Squared;
					BestSample = Sample;
				}
			}
			double Lower = FMath::Max(0, BestSample - 1) / 128.0;
			double Upper = FMath::Min(128, BestSample + 1) / 128.0;
			for (int32 Iteration = 0; Iteration < 24; ++Iteration)
			{
				const double Left = (2.0 * Lower + Upper) / 3.0;
				const double Right = (Lower + 2.0 * Upper) / 3.0;
				if (SquaredResidual(Left) <= SquaredResidual(Right)) Upper = Right;
				else Lower = Left;
			}
			const double T = 0.5 * (Lower + Upper);
			Section.CanonicalRimParameter = 1.0 / 3.0 + (1.0 - T) / 3.0;
		}
		if (Section.TransitionPointCount < 2 ||
			Section.LongitudinalTransitionHop == INDEX_NONE) continue;
		const double EndDepth = Section.LongitudinalTangentDepthCm;
		const double EndOffset = Section.LongitudinalTangentOpeningOffsetCm;
		const double MinimumRegularTangent = FMath::Max(
			1.0e-3, 0.1 * FMath::Sqrt(
				EndDepth * EndDepth + EndOffset * EndOffset));
		auto Objective = [&](const double BoundaryPlaneTangent,
			const double LongitudinalTangent, double* OutMaximum)
		{
			double SquaredSum = 0.0;
			double Maximum = 0.0;
			for (int32 PointOffset = 0;
				PointOffset < Section.TransitionPointCount; ++PointOffset)
			{
				const FVector2d& Point = OpenRimTransitionSectionPoints[
					Section.FirstTransitionPointIndex + PointOffset];
				double BestSquared = TNumericLimits<double>::Max();
				for (int32 Sample = 0; Sample <= 128; ++Sample)
				{
					BestSquared = FMath::Min(BestSquared,
						FVector2d::DistSquared(Point, Evaluate(
							static_cast<double>(Sample) / 128.0, EndDepth,
							EndOffset, BoundaryPlaneTangent,
							LongitudinalTangent)));
				}
				SquaredSum += BestSquared;
				Maximum = FMath::Max(Maximum, FMath::Sqrt(BestSquared));
			}
			if (OutMaximum) *OutMaximum = Maximum;
			return SquaredSum / Section.TransitionPointCount;
		};
		double BoundaryPlaneTangent = FMath::Max(
			MinimumRegularTangent, 2.0 * EndOffset);
		double LongitudinalTangent = FMath::Max(
			MinimumRegularTangent, 2.0 * EndDepth);
		double BestObjective = Objective(
			BoundaryPlaneTangent, LongitudinalTangent, nullptr);
		double StepBoundaryPlane = FMath::Max(5.0, EndDepth * 0.5);
		double StepLongitudinal = FMath::Max(5.0, EndDepth * 0.5);
		for (int32 Iteration = 0; Iteration < 48; ++Iteration)
		{
			bool bImproved = false;
			for (const int32 Axis : { 0, 1 })
			{
				const double Step = Axis == 0 ? StepBoundaryPlane : StepLongitudinal;
				for (const double Sign : { -1.0, 1.0 })
				{
					double TrialBoundaryPlane = BoundaryPlaneTangent;
					double TrialLongitudinal = LongitudinalTangent;
					if (Axis == 0) TrialBoundaryPlane = FMath::Max(
						MinimumRegularTangent, TrialBoundaryPlane + Sign * Step);
					else TrialLongitudinal = FMath::Max(
						MinimumRegularTangent, TrialLongitudinal + Sign * Step);
					const double TrialObjective = Objective(
						TrialBoundaryPlane, TrialLongitudinal, nullptr);
					if (TrialObjective < BestObjective)
					{
						BestObjective = TrialObjective;
						BoundaryPlaneTangent = TrialBoundaryPlane;
						LongitudinalTangent = TrialLongitudinal;
						bImproved = true;
					}
				}
			}
			if (!bImproved)
			{
				StepBoundaryPlane *= 0.5;
				StepLongitudinal *= 0.5;
			}
			if (StepBoundaryPlane < 1.0e-4 && StepLongitudinal < 1.0e-4) break;
		}
		Section.IndividualC2BoundaryPlaneTangentMagnitudeCm = BoundaryPlaneTangent;
		Section.IndividualC2LongitudinalTangentMagnitudeCm = LongitudinalTangent;
		Section.IndividualC2RootMeanSquareResidualCm = FMath::Sqrt(BestObjective);
		Objective(BoundaryPlaneTangent, LongitudinalTangent,
			&Section.IndividualC2MaximumResidualCm);
		Section.bIndividualC2FitValid = FMath::IsFinite(BestObjective) &&
			BoundaryPlaneTangent >= MinimumRegularTangent &&
			LongitudinalTangent >= MinimumRegularTangent;
	}

	for (FOpenRimTransverseSection& Section : OpenRimTransverseSections)
	{
		Section.TopologyCanonicalRimParameter = Section.CanonicalRimParameter;
		Section.bTopologyCanonicalRimParameterValid =
			Section.Feature != EOpenRimFeature::UpperTransition;
	}
	for (int32 CandidateIndex = 0;
		CandidateIndex < OpenRimCandidates.Num(); ++CandidateIndex)
	{
		const FOpenRimCandidate& Candidate =
			OpenRimCandidates[CandidateIndex];
		TMap<int32, TArray<int32>> EdgesByVertex;
		TSet<int32> CandidateEdgeSet;
		for (int32 Offset = 0; Offset < Candidate.EdgeCount; ++Offset)
		{
			const int32 EdgeIndex = OpenRimEdgeIndices[
				Candidate.FirstEdgeIndex + Offset];
			if (!MeshEdges.IsValidIndex(EdgeIndex)) continue;
			CandidateEdgeSet.Add(EdgeIndex);
			const FTriangleMeshEdge& Edge = MeshEdges[EdgeIndex];
			EdgesByVertex.FindOrAdd(Edge.VertexA).Add(EdgeIndex);
			EdgesByVertex.FindOrAdd(Edge.VertexB).Add(EdgeIndex);
		}
		TArray<int32> CandidateVertices;
		EdgesByVertex.GetKeys(CandidateVertices);
		Algo::Sort(CandidateVertices);
		TArray<int32> BaselineEndpoints;
		for (const int32 VertexIndex : CandidateVertices)
		{
			const TArray<int32>& IncidentEdges = EdgesByVertex.FindChecked(VertexIndex);
			if (IncidentEdges.Num() == 1)
				BaselineEndpoints.Add(VertexIndex);
		}
		if (BaselineEndpoints.Num() != 2) continue;
		Algo::Sort(BaselineEndpoints, [&](const int32 A, const int32 B)
		{
			return MeshVertices[A].Y != MeshVertices[B].Y
				? MeshVertices[A].Y < MeshVertices[B].Y : A < B;
		});
		TMap<int32, double> DistanceFromSideBaseline[2];
		for (int32 SourceSide = 0; SourceSide < 2; ++SourceSide)
		{
			TSet<int32> VisitedVertices;
			for (const int32 VertexIndex : CandidateVertices)
				DistanceFromSideBaseline[SourceSide].Add(VertexIndex,
					TNumericLimits<double>::Max());
			DistanceFromSideBaseline[SourceSide][BaselineEndpoints[SourceSide]] = 0.0;
			for (int32 Visit = 0; Visit < CandidateVertices.Num(); ++Visit)
			{
				int32 BestVertex = INDEX_NONE;
				double BestDistance = TNumericLimits<double>::Max();
				for (const int32 VertexIndex : CandidateVertices)
				{
					if (VisitedVertices.Contains(VertexIndex)) continue;
					const double Distance =
						DistanceFromSideBaseline[SourceSide][VertexIndex];
					if (Distance < BestDistance ||
						(Distance == BestDistance && VertexIndex < BestVertex))
					{
						BestDistance = Distance;
						BestVertex = VertexIndex;
					}
				}
				if (BestVertex == INDEX_NONE || !FMath::IsFinite(BestDistance)) break;
				VisitedVertices.Add(BestVertex);
				for (const int32 EdgeIndex : EdgesByVertex.FindChecked(BestVertex))
				{
					const FTriangleMeshEdge& Edge = MeshEdges[EdgeIndex];
					const int32 OtherVertex = Edge.VertexA == BestVertex
						? Edge.VertexB : Edge.VertexA;
					const double CandidateDistance = BestDistance + FVector3d::Distance(
						MeshVertices[Edge.VertexA], MeshVertices[Edge.VertexB]);
					double& OtherDistance =
						DistanceFromSideBaseline[SourceSide].FindChecked(OtherVertex);
					OtherDistance = FMath::Min(OtherDistance, CandidateDistance);
				}
			}
		}

		double TransitionMinimumDistances[2] = {
			TNumericLimits<double>::Max(), TNumericLimits<double>::Max() };
		double TransitionMaximumDistances[2] = {
			-TNumericLimits<double>::Max(), -TNumericLimits<double>::Max() };
		for (const FOpenRimTransverseSection& Section :
			OpenRimTransverseSections)
		{
			if (Section.OpenRimCandidateIndex != CandidateIndex ||
				Section.Feature != EOpenRimFeature::UpperTransition ||
				!CandidateEdgeSet.Contains(Section.RimEdgeIndex)) continue;
			const FTriangleMeshEdge& Edge = MeshEdges[Section.RimEdgeIndex];
			const double MidTransverse = 0.5 *
				(MeshVertices[Edge.VertexA].Y + MeshVertices[Edge.VertexB].Y);
			const int32 Side = MidTransverse < 0.0 ? 0 : 1;
			const double DistanceA =
				DistanceFromSideBaseline[Side][Edge.VertexA];
			const double DistanceB =
				DistanceFromSideBaseline[Side][Edge.VertexB];
			TransitionMinimumDistances[Side] = FMath::Min(
				TransitionMinimumDistances[Side], FMath::Min(DistanceA, DistanceB));
			TransitionMaximumDistances[Side] = FMath::Max(
				TransitionMaximumDistances[Side], FMath::Max(DistanceA, DistanceB));
		}
		for (FOpenRimTransverseSection& Section : OpenRimTransverseSections)
		{
			if (Section.OpenRimCandidateIndex != CandidateIndex ||
				Section.Feature != EOpenRimFeature::UpperTransition ||
				!CandidateEdgeSet.Contains(Section.RimEdgeIndex)) continue;
			const FTriangleMeshEdge& Edge = MeshEdges[Section.RimEdgeIndex];
			const double MidTransverse = 0.5 *
				(MeshVertices[Edge.VertexA].Y + MeshVertices[Edge.VertexB].Y);
			const int32 Side = MidTransverse < 0.0 ? 0 : 1;
			const double TransitionMinimum = TransitionMinimumDistances[Side];
			const double TransitionMaximum = TransitionMaximumDistances[Side];
			if (!FMath::IsFinite(TransitionMinimum) ||
				!FMath::IsFinite(TransitionMaximum) ||
				TransitionMaximum <= TransitionMinimum + 1.0e-6) continue;
			const double MidDistance = 0.5 *
				(DistanceFromSideBaseline[Side][Edge.VertexA] +
					DistanceFromSideBaseline[Side][Edge.VertexB]);
			const double Q = FMath::Clamp((MidDistance - TransitionMinimum) /
				(TransitionMaximum - TransitionMinimum), 0.0, 1.0);
			Section.TopologyCanonicalRimParameter =
				1.0 / 3.0 + Q / 3.0;
			Section.bTopologyCanonicalRimParameterValid = true;
		}
	}
	for (FOpenRimTransverseSection& Section : OpenRimTransverseSections)
	{
		if (Section.InterpolationRimEdgeA == INDEX_NONE ||
			Section.InterpolationRimEdgeB == INDEX_NONE) continue;
		const FOpenRimTransverseSection* A = nullptr;
		const FOpenRimTransverseSection* B = nullptr;
		for (const FOpenRimTransverseSection& CandidateSection :
			OpenRimTransverseSections)
		{
			if (CandidateSection.OpenRimCandidateIndex !=
				Section.OpenRimCandidateIndex) continue;
			if (CandidateSection.RimEdgeIndex == Section.InterpolationRimEdgeA)
				A = &CandidateSection;
			if (CandidateSection.RimEdgeIndex == Section.InterpolationRimEdgeB)
				B = &CandidateSection;
		}
		if (A == nullptr || B == nullptr ||
			!A->bTopologyCanonicalRimParameterValid ||
			!B->bTopologyCanonicalRimParameterValid) continue;
		const double TopologyWidth = FMath::Abs(
			B->TopologyCanonicalRimParameter -
			A->TopologyCanonicalRimParameter);
		const bool bEndpointRefinement =
			Section.InterpolationAlpha < 0.2 ||
			Section.InterpolationAlpha > 0.8;
		if (bEndpointRefinement && TopologyWidth <= 0.1) continue;
		Section.TopologyCanonicalRimParameter = FMath::Lerp(
			A->TopologyCanonicalRimParameter,
			B->TopologyCanonicalRimParameter, Section.InterpolationAlpha);
		Section.bTopologyCanonicalRimParameterValid = true;
	}

	TArray<int32> LoftMembers;
	for (int32 SectionIndex = 0;
		SectionIndex < OpenRimTransverseSections.Num(); ++SectionIndex)
	{
		if (OpenRimTransverseSections[SectionIndex].bIndividualC2FitValid &&
			OpenRimTransverseSections[SectionIndex].
				bTopologyCanonicalRimParameterValid)
			LoftMembers.Add(SectionIndex);
	}
	if (!LoftMembers.IsEmpty())
	{
		const double TransitionVerticalSegmentDerivative = 5.0 * Arch.FlatteningFraction *
			(Arch.HorizontalSpanHeightCm - Arch.VerticalSegmentTransitionHeightCm);
		const double TransitionHorizontalSpanDerivative = 5.0 * Arch.FlatteningFraction *
			(Arch.VerticalSegmentHalfWidthCm - Arch.HorizontalSpanHalfSpanCm);
		const double TransitionParameterWidth = 1.0 /
			(1.0 + (Arch.VerticalSegmentTransitionHeightCm - Arch.BaselineHeightCm) /
				TransitionVerticalSegmentDerivative + Arch.HorizontalSpanHalfSpanCm /
				TransitionHorizontalSpanDerivative);
		const double VerticalSegmentParameterWidth = TransitionParameterWidth *
			(Arch.VerticalSegmentTransitionHeightCm - Arch.BaselineHeightCm) /
			TransitionVerticalSegmentDerivative;
		const double HorizontalSpanParameterWidth = TransitionParameterWidth *
			Arch.HorizontalSpanHalfSpanCm / TransitionHorizontalSpanDerivative;
		const double TransitionParameterEnd =
			VerticalSegmentParameterWidth + TransitionParameterWidth;
		auto ToSurfaceParameter = [&](const double SectionParameter)
		{
			if (SectionParameter <= 1.0 / 3.0)
				return VerticalSegmentParameterWidth * 3.0 * SectionParameter;
			if (SectionParameter >= 2.0 / 3.0)
				return TransitionParameterEnd + HorizontalSpanParameterWidth *
					(3.0 * SectionParameter - 2.0);
			return VerticalSegmentParameterWidth + TransitionParameterWidth *
				(3.0 * SectionParameter - 1.0);
		};
		auto Bernstein5 = [](const double U, double OutBasis[6])
		{
			const double V = 1.0 - U;
			OutBasis[0] = V * V * V * V * V;
			OutBasis[1] = 5.0 * U * V * V * V * V;
			OutBasis[2] = 10.0 * U * U * V * V * V;
			OutBasis[3] = 10.0 * U * U * U * V * V;
			OutBasis[4] = 5.0 * U * U * U * U * V;
			OutBasis[5] = U * U * U * U * U;
		};
		auto CanonicalRim = [&](const double S)
		{
			if (S <= VerticalSegmentParameterWidth)
				return FVector2d(Arch.VerticalSegmentHalfWidthCm,
					FMath::Lerp(Arch.BaselineHeightCm,
						Arch.VerticalSegmentTransitionHeightCm, S / VerticalSegmentParameterWidth));
			if (S >= TransitionParameterEnd)
				return FVector2d(FMath::Lerp(Arch.HorizontalSpanHalfSpanCm,
					0.0, (S - TransitionParameterEnd) /
						HorizontalSpanParameterWidth), Arch.HorizontalSpanHeightCm);
			return EvaluateQuinticTransitionPosition(
				FVector2d(Arch.HorizontalSpanHalfSpanCm,
					Arch.VerticalSegmentTransitionHeightCm),
				Arch.VerticalSegmentHalfWidthCm - Arch.HorizontalSpanHalfSpanCm,
				Arch.HorizontalSpanHeightCm - Arch.VerticalSegmentTransitionHeightCm,
				1.0, 1.0, Arch.FlatteningFraction,
				1.0 - (S - VerticalSegmentParameterWidth) / TransitionParameterWidth);
		};
		for (const int32 SectionIndex : LoftMembers)
		{
			const FOpenRimTransverseSection& Section =
				OpenRimTransverseSections[SectionIndex];
			for (int32 PointOffset = 0;
				PointOffset < Section.TransitionPointCount; ++PointOffset)
			{
				const FVector2d& Point = OpenRimTransitionSectionPoints[
					Section.FirstTransitionPointIndex + PointOffset];
				double BestT = 0.0;
				double BestSquared = TNumericLimits<double>::Max();
				for (int32 Sample = 0; Sample <= 128; ++Sample)
				{
					const double T = static_cast<double>(Sample) / 128.0;
					const double Squared = FVector2d::DistSquared(Point, Evaluate(T,
						Section.LongitudinalTangentDepthCm,
						Section.LongitudinalTangentOpeningOffsetCm,
						Section.IndividualC2BoundaryPlaneTangentMagnitudeCm,
						Section.IndividualC2LongitudinalTangentMagnitudeCm));
					if (Squared < BestSquared)
					{
						BestSquared = Squared;
						BestT = T;
					}
				}
				FOpenRimCanonicalSurfaceSample& SurfaceSample =
					OpenRimCanonicalSurfaceSamples.AddDefaulted_GetRef();
				SurfaceSample.OpenRimTransverseSectionIndex = SectionIndex;
				SurfaceSample.TransitionPointOffset = PointOffset;
				SurfaceSample.CanonicalRimParameter = ToSurfaceParameter(
					Section.CanonicalRimParameter);
				SurfaceSample.TransitionParameter = BestT;
				const FVector3d WorldPoint = Section.SliceOrigin +
					Point.Y * Section.OpeningDirection;
				SurfaceSample.CanonicalPositionCm = FVector3d(
					Point.X, FMath::Abs(WorldPoint.Y), WorldPoint.Z);
			}
		}
		auto Solve12 = [](double Matrix[12][13], double Out[12])
		{
			for (int32 Diagonal = 0; Diagonal < 12; ++Diagonal)
			{
				int32 Pivot = Diagonal;
				for (int32 Row = Diagonal + 1; Row < 12; ++Row)
					if (FMath::Abs(Matrix[Row][Diagonal]) >
						FMath::Abs(Matrix[Pivot][Diagonal])) Pivot = Row;
				if (Pivot != Diagonal)
					for (int32 Column = Diagonal; Column < 13; ++Column)
						Swap(Matrix[Diagonal][Column], Matrix[Pivot][Column]);
				if (FMath::Abs(Matrix[Diagonal][Diagonal]) <= 1.0e-12)
					return false;
				const double Inverse = 1.0 / Matrix[Diagonal][Diagonal];
				for (int32 Column = Diagonal; Column < 13; ++Column)
					Matrix[Diagonal][Column] *= Inverse;
				for (int32 Row = 0; Row < 12; ++Row)
				{
					if (Row == Diagonal) continue;
					const double Factor = Matrix[Row][Diagonal];
					for (int32 Column = Diagonal; Column < 13; ++Column)
						Matrix[Row][Column] -= Factor * Matrix[Diagonal][Column];
				}
			}
			for (int32 Index = 0; Index < 12; ++Index)
				Out[Index] = Matrix[Index][12];
			return true;
		};
		auto Solve6 = [](double Matrix[6][7], double Out[6])
		{
			for (int32 Diagonal = 0; Diagonal < 6; ++Diagonal)
			{
				int32 Pivot = Diagonal;
				for (int32 Row = Diagonal + 1; Row < 6; ++Row)
					if (FMath::Abs(Matrix[Row][Diagonal]) >
						FMath::Abs(Matrix[Pivot][Diagonal])) Pivot = Row;
				if (Pivot != Diagonal)
					for (int32 Column = Diagonal; Column < 7; ++Column)
						Swap(Matrix[Diagonal][Column], Matrix[Pivot][Column]);
				if (FMath::Abs(Matrix[Diagonal][Diagonal]) <= 1.0e-12)
					return false;
				const double Inverse = 1.0 / Matrix[Diagonal][Diagonal];
				for (int32 Column = Diagonal; Column < 7; ++Column)
					Matrix[Diagonal][Column] *= Inverse;
				for (int32 Row = 0; Row < 6; ++Row)
				{
					if (Row == Diagonal) continue;
					const double Factor = Matrix[Row][Diagonal];
					for (int32 Column = Diagonal; Column < 7; ++Column)
						Matrix[Row][Column] -= Factor * Matrix[Diagonal][Column];
				}
			}
			for (int32 Index = 0; Index < 6; ++Index)
				Out[Index] = Matrix[Index][6];
			return true;
		};
		auto Solve4 = [](double Matrix[4][5], double Out[4])
		{
			for (int32 Diagonal = 0; Diagonal < 4; ++Diagonal)
			{
				int32 Pivot = Diagonal;
				for (int32 Row = Diagonal + 1; Row < 4; ++Row)
					if (FMath::Abs(Matrix[Row][Diagonal]) >
						FMath::Abs(Matrix[Pivot][Diagonal])) Pivot = Row;
				if (Pivot != Diagonal)
					for (int32 Column = Diagonal; Column < 5; ++Column)
						Swap(Matrix[Diagonal][Column], Matrix[Pivot][Column]);
				if (FMath::Abs(Matrix[Diagonal][Diagonal]) <= 1.0e-12)
					return false;
				const double Inverse = 1.0 / Matrix[Diagonal][Diagonal];
				for (int32 Column = Diagonal; Column < 5; ++Column)
					Matrix[Diagonal][Column] *= Inverse;
				for (int32 Row = 0; Row < 4; ++Row)
				{
					if (Row == Diagonal) continue;
					const double Factor = Matrix[Row][Diagonal];
					for (int32 Column = Diagonal; Column < 5; ++Column)
						Matrix[Row][Column] -= Factor * Matrix[Diagonal][Column];
				}
			}
			for (int32 Index = 0; Index < 4; ++Index)
				Out[Index] = Matrix[Index][4];
			return true;
		};
		auto Solve8 = [](double Matrix[8][9], double Out[8])
		{
			for (int32 Diagonal = 0; Diagonal < 8; ++Diagonal)
			{
				int32 Pivot = Diagonal;
				for (int32 Row = Diagonal + 1; Row < 8; ++Row)
					if (FMath::Abs(Matrix[Row][Diagonal]) >
						FMath::Abs(Matrix[Pivot][Diagonal])) Pivot = Row;
				if (Pivot != Diagonal)
					for (int32 Column = Diagonal; Column < 9; ++Column)
						Swap(Matrix[Diagonal][Column], Matrix[Pivot][Column]);
				if (FMath::Abs(Matrix[Diagonal][Diagonal]) <= 1.0e-12)
					return false;
				const double Inverse = 1.0 / Matrix[Diagonal][Diagonal];
				for (int32 Column = Diagonal; Column < 9; ++Column)
					Matrix[Diagonal][Column] *= Inverse;
				for (int32 Row = 0; Row < 8; ++Row)
				{
					if (Row == Diagonal) continue;
					const double Factor = Matrix[Row][Diagonal];
					for (int32 Column = Diagonal; Column < 9; ++Column)
						Matrix[Row][Column] -= Factor * Matrix[Diagonal][Column];
				}
			}
			for (int32 Index = 0; Index < 8; ++Index)
				Out[Index] = Matrix[Index][8];
			return true;
		};
		FOpenRimCanonicalSurfaceFit& Surface =
			OpenRimCanonicalSurfaceFits.AddDefaulted_GetRef();
		Surface.FitId = StableStringId(
			TEXT("OpenRim.CanonicalSurface.Direct3D.JointMonotoneOpenTransition.Bernstein5x5.TransitionC2.V5"));
		Surface.SampleCount = OpenRimCanonicalSurfaceSamples.Num();
		Surface.VerticalSegmentParameterWidth = VerticalSegmentParameterWidth;
		Surface.TransitionParameterWidth = TransitionParameterWidth;
		Surface.HorizontalSpanParameterWidth = HorizontalSpanParameterWidth;
		for (int32 Index = 0; Index < 6; ++Index)
		{
			const double Alpha = static_cast<double>(Index) / 5.0;
			Surface.CanonicalRimSegmentControlPoints[Index] = FVector2d(
				Arch.VerticalSegmentHalfWidthCm,
				FMath::Lerp(Arch.BaselineHeightCm,
					Arch.VerticalSegmentTransitionHeightCm, Alpha));
			Surface.CanonicalRimSegmentControlPoints[12 + Index] = FVector2d(
				FMath::Lerp(Arch.HorizontalSpanHalfSpanCm, 0.0, Alpha),
				Arch.HorizontalSpanHeightCm);
		}
		FVector2d TransitionControlPoints[6];
		BuildQuinticTransitionControlPoints(
			FVector2d(Arch.HorizontalSpanHalfSpanCm,
				Arch.VerticalSegmentTransitionHeightCm),
			Arch.VerticalSegmentHalfWidthCm - Arch.HorizontalSpanHalfSpanCm,
			Arch.HorizontalSpanHeightCm -
				Arch.VerticalSegmentTransitionHeightCm,
			1.0, 1.0, Arch.FlatteningFraction, TransitionControlPoints);
		for (int32 Index = 0; Index < 6; ++Index)
		{
			// CanonicalRim traverses the transition with t=1-q.
			Surface.CanonicalRimSegmentControlPoints[6 + Index] =
				TransitionControlPoints[5 - Index];
		}
		auto EvaluateField = [&](const double Coefficients[6], const double S)
		{
			double Basis[6];
			Bernstein5(S, Basis);
			double Value = 0.0;
			for (int32 Index = 0; Index < 6; ++Index)
				Value += Basis[Index] * Coefficients[Index];
			return Value;
		};
		auto EvaluateBaseSurface = [&](const double S, const double T)
		{
			double TBasis[6];
			Bernstein5(T, TBasis);
			const FVector2d Rim = CanonicalRim(S);
			const double RimWeight = TBasis[0] + TBasis[1] + TBasis[2];
			const double StartWeight = (TBasis[1] + 2.0 * TBasis[2]) / 5.0;
			const double EndWeight = TBasis[3] + TBasis[4] + TBasis[5];
			const double LongitudinalWeight =
				-(2.0 * TBasis[3] + TBasis[4]) / 5.0;
			return FVector3d(
				EndWeight * EvaluateField(Surface.EndDepthCoefficients, S) +
					LongitudinalWeight * EvaluateField(
						Surface.EndLongitudinalTangentCoefficients, S),
				RimWeight * Rim.X + StartWeight * EvaluateField(
					Surface.StartTangentYCoefficients, S) +
					EndWeight * EvaluateField(Surface.EndYCoefficients, S),
				RimWeight * Rim.Y + StartWeight * EvaluateField(
					Surface.StartTangentZCoefficients, S) +
					EndWeight * EvaluateField(Surface.EndZCoefficients, S));
		};
		auto TransitionBasis = [&](const double S, double Out[4])
		{
			for (int32 Index = 0; Index < 4; ++Index) Out[Index] = 0.0;
			if (S <= VerticalSegmentParameterWidth || S >= TransitionParameterEnd) return;
			const double Q = (S - VerticalSegmentParameterWidth) / TransitionParameterWidth;
			const double U = 1.0 - Q;
			// Fifth-order endpoint zeros keep the correction strictly C2-local and
			// suppress higher-order leakage in one-sided join diagnostics.
			const double Envelope = 1024.0 * Q * Q * Q * Q * Q *
				U * U * U * U * U;
			Out[0] = Envelope * U * U * U;
			Out[1] = Envelope * 3.0 * U * U * Q;
			Out[2] = Envelope * 3.0 * U * Q * Q;
			Out[3] = Envelope * Q * Q * Q;
		};
		auto EvaluateTransitionField = [&](const double Coefficients[4],
			const double S)
		{
			double Basis[4];
			TransitionBasis(S, Basis);
			double Value = 0.0;
			for (int32 Index = 0; Index < 4; ++Index)
				Value += Basis[Index] * Coefficients[Index];
			return Value;
		};
		auto EvaluateSurface = [&](const double S, const double T)
		{
			double TBasis[6];
			Bernstein5(T, TBasis);
			const double StartWeight = (TBasis[1] + 2.0 * TBasis[2]) / 5.0;
			const double EndWeight = TBasis[3] + TBasis[4] + TBasis[5];
			return EvaluateBaseSurface(S, T) + FVector3d(
				EndWeight * EvaluateTransitionField(
					Surface.TransitionEndDepthCoefficients, S),
				StartWeight * EvaluateTransitionField(
					Surface.TransitionStartTangentYCoefficients, S) +
					EndWeight * EvaluateTransitionField(
						Surface.TransitionEndYCoefficients, S),
				StartWeight * EvaluateTransitionField(
					Surface.TransitionStartTangentZCoefficients, S) +
					EndWeight * EvaluateTransitionField(
						Surface.TransitionEndZCoefficients, S));
		};
		bool bSolved = true;
		auto FitSurfaceForCurrentCorrespondence = [&]()
		{
			bSolved = true;
			for (int32 Index = 0; Index < 4; ++Index)
			{
				Surface.TransitionStartTangentYCoefficients[Index] = 0.0;
				Surface.TransitionStartTangentZCoefficients[Index] = 0.0;
				Surface.TransitionEndDepthCoefficients[Index] = 0.0;
				Surface.TransitionEndYCoefficients[Index] = 0.0;
				Surface.TransitionEndZCoefficients[Index] = 0.0;
			}
		for (int32 Alternation = 0; Alternation < 3 && bSolved; ++Alternation)
		{
			auto FitCoordinate = [&](const int32 Coordinate, double OutFirst[6],
				double OutSecond[6])
			{
				double Matrix[12][13] = {};
				for (const FOpenRimCanonicalSurfaceSample& Sample :
					OpenRimCanonicalSurfaceSamples)
				{
					double SBasis[6], TBasis[6];
					Bernstein5(Sample.CanonicalRimParameter, SBasis);
					Bernstein5(Sample.TransitionParameter, TBasis);
					const double RimWeight = TBasis[0] + TBasis[1] + TBasis[2];
					const double FirstWeight = Coordinate == 0
						? TBasis[3] + TBasis[4] + TBasis[5]
						: (TBasis[1] + 2.0 * TBasis[2]) / 5.0;
					const double SecondWeight = Coordinate == 0
						? -(2.0 * TBasis[3] + TBasis[4]) / 5.0
						: TBasis[3] + TBasis[4] + TBasis[5];
					double Row[12];
					for (int32 Index = 0; Index < 6; ++Index)
					{
						Row[Index] = FirstWeight * SBasis[Index];
						Row[6 + Index] = SecondWeight * SBasis[Index];
					}
					double Target = Sample.CanonicalPositionCm[Coordinate];
					if (Coordinate != 0)
					{
						const FVector2d Rim = CanonicalRim(
							Sample.CanonicalRimParameter);
						Target -= RimWeight * Rim[Coordinate - 1];
					}
					for (int32 A = 0; A < 12; ++A)
					{
						for (int32 B = 0; B < 12; ++B)
							Matrix[A][B] += Row[A] * Row[B];
						Matrix[A][12] += Row[A] * Target;
					}
				}
				double Solution[12];
				if (!Solve12(Matrix, Solution)) return false;
				for (int32 Index = 0; Index < 6; ++Index)
				{
					OutFirst[Index] = Solution[Index];
					OutSecond[Index] = Solution[6 + Index];
				}
				return true;
			};
			bSolved = FitCoordinate(0, Surface.EndDepthCoefficients,
				Surface.EndLongitudinalTangentCoefficients) &&
				FitCoordinate(1, Surface.StartTangentYCoefficients,
					Surface.EndYCoefficients) &&
				FitCoordinate(2, Surface.StartTangentZCoefficients,
					Surface.EndZCoefficients);
			if (!bSolved) break;
			for (double& Coefficient :
				Surface.EndLongitudinalTangentCoefficients)
				Coefficient = FMath::Max(1.0, Coefficient);
			double DepthMatrix[6][7] = {};
			for (const FOpenRimCanonicalSurfaceSample& Sample :
				OpenRimCanonicalSurfaceSamples)
			{
				double SBasis[6], TBasis[6];
				Bernstein5(Sample.CanonicalRimParameter, SBasis);
				Bernstein5(Sample.TransitionParameter, TBasis);
				const double EndWeight =
					TBasis[3] + TBasis[4] + TBasis[5];
				const double LongitudinalWeight =
					-(2.0 * TBasis[3] + TBasis[4]) / 5.0;
				const double Target = Sample.CanonicalPositionCm.X -
					LongitudinalWeight * EvaluateField(
						Surface.EndLongitudinalTangentCoefficients,
						Sample.CanonicalRimParameter);
				for (int32 Row = 0; Row < 6; ++Row)
				{
					const double RowValue = EndWeight * SBasis[Row];
					for (int32 Column = 0; Column < 6; ++Column)
						DepthMatrix[Row][Column] += RowValue *
							EndWeight * SBasis[Column];
					DepthMatrix[Row][6] += RowValue * Target;
				}
			}
			bSolved = Solve6(DepthMatrix, Surface.EndDepthCoefficients);
			if (!bSolved) break;
			for (FOpenRimCanonicalSurfaceSample& Sample :
				OpenRimCanonicalSurfaceSamples)
			{
				double BestT = Sample.TransitionParameter;
				double BestSquared = FVector3d::DistSquared(
					Sample.CanonicalPositionCm,
					EvaluateSurface(Sample.CanonicalRimParameter, BestT));
				for (int32 Candidate = 0; Candidate <= 128; ++Candidate)
				{
					const double T = static_cast<double>(Candidate) / 128.0;
					const double Squared = FVector3d::DistSquared(
						Sample.CanonicalPositionCm,
						EvaluateSurface(Sample.CanonicalRimParameter, T));
					if (Squared < BestSquared)
					{
						BestSquared = Squared;
						BestT = T;
					}
				}
				Sample.TransitionParameter = BestT;
			}
		}
		for (int32 Alternation = 0; Alternation < 3 && bSolved; ++Alternation)
		{
			auto FitTransitionCoordinate = [&](const int32 Coordinate,
				double OutFirst[4], double* OutSecond)
			{
				if (Coordinate == 0)
				{
					double Matrix[4][5] = {};
					for (const FOpenRimCanonicalSurfaceSample& Sample :
						OpenRimCanonicalSurfaceSamples)
					{
						double SBasis[4], TBasis[6];
						TransitionBasis(Sample.CanonicalRimParameter, SBasis);
						Bernstein5(Sample.TransitionParameter, TBasis);
						const double EndWeight = TBasis[3] + TBasis[4] + TBasis[5];
						const double Target = Sample.CanonicalPositionCm.X -
							EvaluateBaseSurface(Sample.CanonicalRimParameter,
								Sample.TransitionParameter).X;
						for (int32 Row = 0; Row < 4; ++Row)
						{
							const double RowValue = EndWeight * SBasis[Row];
							for (int32 Column = 0; Column < 4; ++Column)
								Matrix[Row][Column] += RowValue *
									EndWeight * SBasis[Column];
							Matrix[Row][4] += RowValue * Target;
						}
					}
					return Solve4(Matrix, OutFirst);
				}
				double Matrix[8][9] = {};
				for (const FOpenRimCanonicalSurfaceSample& Sample :
					OpenRimCanonicalSurfaceSamples)
				{
					double SBasis[4], TBasis[6], RowValues[8];
					TransitionBasis(Sample.CanonicalRimParameter, SBasis);
					Bernstein5(Sample.TransitionParameter, TBasis);
					const double StartWeight =
						(TBasis[1] + 2.0 * TBasis[2]) / 5.0;
					const double EndWeight = TBasis[3] + TBasis[4] + TBasis[5];
					for (int32 Index = 0; Index < 4; ++Index)
					{
						RowValues[Index] = StartWeight * SBasis[Index];
						RowValues[4 + Index] = EndWeight * SBasis[Index];
					}
					const double Target = Sample.CanonicalPositionCm[Coordinate] -
						EvaluateBaseSurface(Sample.CanonicalRimParameter,
							Sample.TransitionParameter)[Coordinate];
					for (int32 Row = 0; Row < 8; ++Row)
					{
						for (int32 Column = 0; Column < 8; ++Column)
							Matrix[Row][Column] += RowValues[Row] * RowValues[Column];
						Matrix[Row][8] += RowValues[Row] * Target;
					}
				}
				double Solution[8];
				if (!Solve8(Matrix, Solution)) return false;
				for (int32 Index = 0; Index < 4; ++Index)
				{
					OutFirst[Index] = Solution[Index];
					OutSecond[Index] = Solution[4 + Index];
				}
				return true;
			};
			bSolved = FitTransitionCoordinate(0,
				Surface.TransitionEndDepthCoefficients, nullptr) &&
				FitTransitionCoordinate(1,
					Surface.TransitionStartTangentYCoefficients,
					Surface.TransitionEndYCoefficients) &&
				FitTransitionCoordinate(2,
					Surface.TransitionStartTangentZCoefficients,
					Surface.TransitionEndZCoefficients);
			if (!bSolved) break;
			for (FOpenRimCanonicalSurfaceSample& Sample :
				OpenRimCanonicalSurfaceSamples)
			{
				double BestT = Sample.TransitionParameter;
				double BestSquared = FVector3d::DistSquared(
					Sample.CanonicalPositionCm,
					EvaluateSurface(Sample.CanonicalRimParameter, BestT));
				for (int32 Candidate = 0; Candidate <= 128; ++Candidate)
				{
					const double T = static_cast<double>(Candidate) / 128.0;
					const double Squared = FVector3d::DistSquared(
						Sample.CanonicalPositionCm,
						EvaluateSurface(Sample.CanonicalRimParameter, T));
					if (Squared < BestSquared)
					{
						BestSquared = Squared;
						BestT = T;
					}
				}
				Sample.TransitionParameter = BestT;
			}
		}
		};
		auto BuildTransitionLanes = [&]()
		{
			TMap<uint64, TArray<int32>> LaneSections;
			for (int32 SectionIndex = 0;
				SectionIndex < OpenRimTransverseSections.Num(); ++SectionIndex)
			{
				const FOpenRimTransverseSection& Section =
					OpenRimTransverseSections[SectionIndex];
				if (Section.Feature != EOpenRimFeature::UpperTransition ||
					!Section.bTopologyCanonicalRimParameterValid) continue;
				const uint64 Side = Section.SliceOrigin.Y < 0.0 ? 0ull : 1ull;
				const uint64 LaneKey =
					(static_cast<uint64>(Section.OpenRimCandidateIndex) << 1) | Side;
				LaneSections.FindOrAdd(LaneKey).Add(SectionIndex);
			}
			return LaneSections;
		};
		auto OptimizeMonotoneTransitionCorrespondence = [&]()
		{
			TMap<uint64, TArray<int32>> LaneSections = BuildTransitionLanes();
			for (TPair<uint64, TArray<int32>>& Lane : LaneSections)
			{
				Algo::Sort(Lane.Value, [&](const int32 A, const int32 B)
				{
					const FOpenRimTransverseSection& SectionA =
						OpenRimTransverseSections[A];
					const FOpenRimTransverseSection& SectionB =
						OpenRimTransverseSections[B];
					return SectionA.TopologyCanonicalRimParameter !=
						SectionB.TopologyCanonicalRimParameter
						? SectionA.TopologyCanonicalRimParameter <
							SectionB.TopologyCanonicalRimParameter
						: SectionA.SectionId < SectionB.SectionId;
				});
				const int32 SectionCount = Lane.Value.Num();
				if (SectionCount == 0) continue;
				constexpr int32 GridCount = 129;
				const double InfiniteCost = TNumericLimits<double>::Max();
				TArray<double> Costs;
				Costs.Init(InfiniteCost, SectionCount * GridCount);
				for (int32 LaneIndex = 0; LaneIndex < SectionCount; ++LaneIndex)
				{
					const int32 SectionIndex = Lane.Value[LaneIndex];
					const FOpenRimTransverseSection& Section =
						OpenRimTransverseSections[SectionIndex];
					const double GeometricS = ToSurfaceParameter(
						Section.CanonicalRimParameter);
					const double TopologyS = ToSurfaceParameter(
						Section.TopologyCanonicalRimParameter);
					const double HalfGridStep =
						0.5 * TransitionParameterWidth / (GridCount - 1);
					const double MinimumS = FMath::Min(GeometricS, TopologyS) -
						HalfGridStep;
					const double MaximumS = FMath::Max(GeometricS, TopologyS) +
						HalfGridStep;
					int32 SampleCount = 0;
					for (const FOpenRimCanonicalSurfaceSample& Sample :
						OpenRimCanonicalSurfaceSamples)
						SampleCount += Sample.OpenRimTransverseSectionIndex ==
							SectionIndex ? 1 : 0;
					if (SampleCount == 0) continue;
					for (int32 GridIndex = 0; GridIndex < GridCount; ++GridIndex)
					{
						if (GridIndex == 0 || GridIndex == GridCount - 1) continue;
						const double S = VerticalSegmentParameterWidth + TransitionParameterWidth *
							static_cast<double>(GridIndex) / (GridCount - 1);
						if (S < MinimumS || S > MaximumS) continue;
						double SquaredSum = 0.0;
						for (const FOpenRimCanonicalSurfaceSample& Sample :
							OpenRimCanonicalSurfaceSamples)
						{
							if (Sample.OpenRimTransverseSectionIndex != SectionIndex)
								continue;
							SquaredSum += FVector3d::DistSquared(
								Sample.CanonicalPositionCm,
								EvaluateSurface(S, Sample.TransitionParameter));
						}
						Costs[LaneIndex * GridCount + GridIndex] =
							SquaredSum / SampleCount;
					}
				}
				TArray<double> DynamicCosts;
				DynamicCosts.Init(InfiniteCost, SectionCount * GridCount);
				TArray<int32> Predecessors;
				Predecessors.Init(INDEX_NONE, SectionCount * GridCount);
				for (int32 GridIndex = 0; GridIndex < GridCount; ++GridIndex)
					DynamicCosts[GridIndex] = Costs[GridIndex];
				for (int32 LaneIndex = 1; LaneIndex < SectionCount; ++LaneIndex)
				{
					for (int32 GridIndex = 0; GridIndex < GridCount; ++GridIndex)
					{
						const double LocalCost = Costs[LaneIndex * GridCount + GridIndex];
						if (LocalCost == InfiniteCost) continue;
						double BestPrior = InfiniteCost;
						int32 BestPriorIndex = INDEX_NONE;
						for (int32 PriorIndex = 0; PriorIndex < GridIndex; ++PriorIndex)
						{
							const double PriorCost = DynamicCosts[
								(LaneIndex - 1) * GridCount + PriorIndex];
							if (PriorCost < BestPrior)
							{
								BestPrior = PriorCost;
								BestPriorIndex = PriorIndex;
							}
						}
						if (BestPriorIndex == INDEX_NONE) continue;
						DynamicCosts[LaneIndex * GridCount + GridIndex] =
							BestPrior + LocalCost;
						Predecessors[LaneIndex * GridCount + GridIndex] = BestPriorIndex;
					}
				}
				int32 BestGridIndex = INDEX_NONE;
				double BestLaneCost = InfiniteCost;
				for (int32 GridIndex = 0; GridIndex < GridCount; ++GridIndex)
				{
					const double Cost = DynamicCosts[
						(SectionCount - 1) * GridCount + GridIndex];
					if (Cost < BestLaneCost)
					{
						BestLaneCost = Cost;
						BestGridIndex = GridIndex;
					}
				}
				if (BestGridIndex == INDEX_NONE || BestLaneCost == InfiniteCost)
					return false;
				for (int32 LaneIndex = SectionCount - 1;
					LaneIndex >= 0; --LaneIndex)
				{
					const int32 SectionIndex = Lane.Value[LaneIndex];
					const double S = VerticalSegmentParameterWidth + TransitionParameterWidth *
						static_cast<double>(BestGridIndex) / (GridCount - 1);
					for (FOpenRimCanonicalSurfaceSample& Sample :
						OpenRimCanonicalSurfaceSamples)
					{
						if (Sample.OpenRimTransverseSectionIndex == SectionIndex)
							Sample.CanonicalRimParameter = S;
					}
					if (LaneIndex > 0)
						BestGridIndex = Predecessors[
							LaneIndex * GridCount + BestGridIndex];
				}
			}
			return true;
		};
		for (int32 CorrespondenceAlternation = 0;
			CorrespondenceAlternation < 3 && bSolved; ++CorrespondenceAlternation)
		{
			FitSurfaceForCurrentCorrespondence();
			if (!bSolved || CorrespondenceAlternation == 2) break;
			bSolved = OptimizeMonotoneTransitionCorrespondence();
		}
		if (bSolved)
		{
			constexpr int32 CorrespondenceGridCount = 129;
			const double HalfCorrespondenceGridStep =
				0.5 * TransitionParameterWidth / (CorrespondenceGridCount - 1);
			TMap<uint64, TArray<int32>> CorrespondenceLanes = BuildTransitionLanes();
			TArray<uint64> CorrespondenceLaneIds;
			CorrespondenceLanes.GetKeys(CorrespondenceLaneIds);
			Algo::Sort(CorrespondenceLaneIds);
			Surface.CorrespondenceLaneCount = CorrespondenceLaneIds.Num();
			Surface.bCorrespondenceStrictlyMonotone = true;
			Surface.bCorrespondenceWithinQuantizedWitnessBounds = true;
			Surface.bCorrespondenceStrictlyInterior = true;
			Surface.MinimumCorrespondenceSpacing = TNumericLimits<double>::Max();
			for (const uint64 LaneId : CorrespondenceLaneIds)
			{
				TArray<int32>& Lane = CorrespondenceLanes.FindChecked(LaneId);
				Algo::Sort(Lane, [&](const int32 A, const int32 B)
				{
					const FOpenRimTransverseSection& SectionA =
						OpenRimTransverseSections[A];
					const FOpenRimTransverseSection& SectionB =
						OpenRimTransverseSections[B];
					return SectionA.TopologyCanonicalRimParameter !=
						SectionB.TopologyCanonicalRimParameter
						? SectionA.TopologyCanonicalRimParameter <
							SectionB.TopologyCanonicalRimParameter
						: SectionA.SectionId < SectionB.SectionId;
				});
				double PriorOptimizedS = -TNumericLimits<double>::Max();
				for (int32 LaneRank = 0; LaneRank < Lane.Num(); ++LaneRank)
				{
					const int32 SectionIndex = Lane[LaneRank];
					const FOpenRimTransverseSection& Section =
						OpenRimTransverseSections[SectionIndex];
					const double GeometricS = ToSurfaceParameter(
						Section.CanonicalRimParameter);
					const double TopologyS = ToSurfaceParameter(
						Section.TopologyCanonicalRimParameter);
					double OptimizedS = GeometricS;
					double SquaredSum = 0.0;
					int32 SectionSampleCount = 0;
					for (const FOpenRimCanonicalSurfaceSample& Sample :
						OpenRimCanonicalSurfaceSamples)
					{
						if (Sample.OpenRimTransverseSectionIndex != SectionIndex)
							continue;
						OptimizedS = Sample.CanonicalRimParameter;
						SquaredSum += FVector3d::DistSquared(
							Sample.CanonicalPositionCm,
							EvaluateSurface(OptimizedS, Sample.TransitionParameter));
						++SectionSampleCount;
					}
					if (SectionSampleCount == 0) continue;
					FOpenRimCanonicalSectionCorrespondence& Correspondence =
						OpenRimCanonicalSectionCorrespondences.AddDefaulted_GetRef();
					Correspondence.OpenRimTransverseSectionIndex = SectionIndex;
					Correspondence.LaneId = LaneId;
					Correspondence.LaneRank = LaneRank;
					Correspondence.LaneSectionCount = Lane.Num();
					Correspondence.GeometricSurfaceParameter = GeometricS;
					Correspondence.TopologySurfaceParameter = TopologyS;
					Correspondence.OptimizedSurfaceParameter = OptimizedS;
					Correspondence.RootMeanSquareResidualCm = FMath::Sqrt(
						SquaredSum / SectionSampleCount);
					Correspondence.bWithinQuantizedWitnessBounds =
						OptimizedS >= FMath::Min(GeometricS, TopologyS) -
							HalfCorrespondenceGridStep &&
						OptimizedS <= FMath::Max(GeometricS, TopologyS) +
							HalfCorrespondenceGridStep;
					Correspondence.bStrictlyInterior =
						OptimizedS > VerticalSegmentParameterWidth &&
						OptimizedS < TransitionParameterEnd;
					const bool bStrictlyMonotone = OptimizedS > PriorOptimizedS;
					Surface.bCorrespondenceStrictlyMonotone &= bStrictlyMonotone;
					Surface.bCorrespondenceWithinQuantizedWitnessBounds &=
						Correspondence.bWithinQuantizedWitnessBounds;
					Surface.bCorrespondenceStrictlyInterior &=
						Correspondence.bStrictlyInterior;
					if (LaneRank > 0)
						Surface.MinimumCorrespondenceSpacing = FMath::Min(
							Surface.MinimumCorrespondenceSpacing,
							OptimizedS - PriorOptimizedS);
					PriorOptimizedS = OptimizedS;
					Surface.MaximumCorrespondenceDisplacement = FMath::Max(
						Surface.MaximumCorrespondenceDisplacement,
						FMath::Abs(OptimizedS - GeometricS));
				}
			}
			Surface.CorrespondenceSectionCount =
				OpenRimCanonicalSectionCorrespondences.Num();
			if (Surface.MinimumCorrespondenceSpacing ==
				TNumericLimits<double>::Max())
				Surface.MinimumCorrespondenceSpacing = 0.0;
			double SquaredSum = 0.0;
			double TransitionEndpointClampedSquaredSum = 0.0;
			double TransitionInteriorSquaredSum = 0.0;
			for (const FOpenRimCanonicalSurfaceSample& Sample :
				OpenRimCanonicalSurfaceSamples)
			{
				const FVector3d FitPosition = EvaluateSurface(
					Sample.CanonicalRimParameter, Sample.TransitionParameter);
				const double Residual = FVector3d::Distance(
					Sample.CanonicalPositionCm, FitPosition);
				SquaredSum += Residual * Residual;
				if (Residual > Surface.MaximumResidualCm)
				{
					Surface.MaximumResidualCm = Residual;
					Surface.MaximumResidualSectionIndex =
						Sample.OpenRimTransverseSectionIndex;
					Surface.MaximumResidualPointOffset = Sample.TransitionPointOffset;
					Surface.MaximumResidualRimParameter = Sample.CanonicalRimParameter;
					Surface.MaximumResidualTransitionParameter =
						Sample.TransitionParameter;
					Surface.MaximumResidualSourcePositionCm =
						Sample.CanonicalPositionCm;
					Surface.MaximumResidualFitPositionCm = FitPosition;
				}
				const int32 FeatureIndex = static_cast<int32>(
					OpenRimTransverseSections[
						Sample.OpenRimTransverseSectionIndex].Feature);
				++Surface.FeatureSampleCounts[FeatureIndex];
				Surface.FeatureRootMeanSquareResidualsCm[FeatureIndex] +=
					Residual * Residual;
				Surface.FeatureMaximumResidualsCm[FeatureIndex] = FMath::Max(
					Surface.FeatureMaximumResidualsCm[FeatureIndex], Residual);
				if (FeatureIndex == static_cast<int32>(
					EOpenRimFeature::UpperTransition))
				{
					constexpr double EndpointParameterTolerance = 1.0e-6;
					const bool bEndpointClamped =
						Sample.CanonicalRimParameter <= VerticalSegmentParameterWidth +
							EndpointParameterTolerance ||
						Sample.CanonicalRimParameter >= TransitionParameterEnd -
							EndpointParameterTolerance;
					if (bEndpointClamped)
					{
						++Surface.TransitionEndpointClampedSampleCount;
						TransitionEndpointClampedSquaredSum += Residual * Residual;
						Surface.TransitionEndpointClampedMaximumResidualCm = FMath::Max(
							Surface.TransitionEndpointClampedMaximumResidualCm, Residual);
					}
					else
					{
						++Surface.TransitionInteriorSampleCount;
						TransitionInteriorSquaredSum += Residual * Residual;
						Surface.TransitionInteriorMaximumResidualCm = FMath::Max(
							Surface.TransitionInteriorMaximumResidualCm, Residual);
					}
				}
			}
			Surface.RootMeanSquareResidualCm = FMath::Sqrt(
				SquaredSum / OpenRimCanonicalSurfaceSamples.Num());
			for (int32 FeatureIndex = 0; FeatureIndex < 4; ++FeatureIndex)
				if (Surface.FeatureSampleCounts[FeatureIndex] > 0)
					Surface.FeatureRootMeanSquareResidualsCm[FeatureIndex] =
						FMath::Sqrt(Surface.FeatureRootMeanSquareResidualsCm[
							FeatureIndex] / Surface.FeatureSampleCounts[FeatureIndex]);
			if (Surface.TransitionEndpointClampedSampleCount > 0)
				Surface.TransitionEndpointClampedRootMeanSquareResidualCm = FMath::Sqrt(
					TransitionEndpointClampedSquaredSum /
					Surface.TransitionEndpointClampedSampleCount);
			if (Surface.TransitionInteriorSampleCount > 0)
				Surface.TransitionInteriorRootMeanSquareResidualCm = FMath::Sqrt(
					TransitionInteriorSquaredSum / Surface.TransitionInteriorSampleCount);
			Surface.MinimumEndDepthCm = TNumericLimits<double>::Max();
			Surface.MinimumLongitudinalTangentCm = TNumericLimits<double>::Max();
			for (int32 Sample = 0; Sample <= 256; ++Sample)
			{
				const double S = static_cast<double>(Sample) / 256.0;
				Surface.MinimumEndDepthCm = FMath::Min(Surface.MinimumEndDepthCm,
					EvaluateField(Surface.EndDepthCoefficients, S) +
					EvaluateTransitionField(Surface.TransitionEndDepthCoefficients, S));
				Surface.MinimumLongitudinalTangentCm = FMath::Min(
					Surface.MinimumLongitudinalTangentCm,
					EvaluateField(Surface.EndLongitudinalTangentCoefficients, S));
			}
			constexpr double H = 1.0e-4;
			for (const double Join : { VerticalSegmentParameterWidth,
				TransitionParameterEnd })
			{
				for (int32 TSample = 0; TSample <= 8; ++TSample)
				{
					const double T = static_cast<double>(TSample) / 8.0;
					const FVector3d P0 = EvaluateSurface(Join, T);
					const FVector3d LM1 = EvaluateSurface(Join - H, T);
					const FVector3d LM2 = EvaluateSurface(Join - 2.0 * H, T);
					const FVector3d LM3 = EvaluateSurface(Join - 3.0 * H, T);
					const FVector3d RP1 = EvaluateSurface(Join + H, T);
					const FVector3d RP2 = EvaluateSurface(Join + 2.0 * H, T);
					const FVector3d RP3 = EvaluateSurface(Join + 3.0 * H, T);
					const FVector3d LeftFirst =
						(3.0 * P0 - 4.0 * LM1 + LM2) / (2.0 * H);
					const FVector3d RightFirst =
						(-3.0 * P0 + 4.0 * RP1 - RP2) / (2.0 * H);
					const FVector3d LeftSecond =
						(2.0 * P0 - 5.0 * LM1 + 4.0 * LM2 - LM3) / (H * H);
					const FVector3d RightSecond =
						(2.0 * P0 - 5.0 * RP1 + 4.0 * RP2 - RP3) / (H * H);
					Surface.MaximumFirstDerivativeJoinResidualCm = FMath::Max(
						Surface.MaximumFirstDerivativeJoinResidualCm,
						FVector3d::Distance(LeftFirst, RightFirst));
					Surface.MaximumSecondDerivativeJoinResidualCm = FMath::Max(
						Surface.MaximumSecondDerivativeJoinResidualCm,
						FVector3d::Distance(LeftSecond, RightSecond));
					const FVector3d DT = (EvaluateSurface(Join, T + H) -
						EvaluateSurface(Join, T - H)).GetSafeNormal();
					const FVector3d LeftNormal = FVector3d::CrossProduct(
						LeftFirst.GetSafeNormal(), DT).GetSafeNormal();
					const FVector3d RightNormal = FVector3d::CrossProduct(
						RightFirst.GetSafeNormal(), DT).GetSafeNormal();
					Surface.MaximumNormalJoinResidualDegrees = FMath::Max(
						Surface.MaximumNormalJoinResidualDegrees,
						FMath::RadiansToDegrees(FMath::Acos(FMath::Clamp(
							FVector3d::DotProduct(LeftNormal, RightNormal),
							-1.0, 1.0))));
				}
			}
		}
		Surface.bExactXYMirrorPlacement = bSolved;
		Surface.bExactTransverseC2ByConstruction = bSolved;
		Surface.bLongitudinallyC2ByConstruction = bSolved;
		Surface.bTransitionCorrectionC2ByConstruction = bSolved;
		Surface.bRegularPositiveParameterization = bSolved &&
			Surface.MinimumEndDepthCm > 1.0e-3 &&
			Surface.MinimumLongitudinalTangentCm > 1.0e-3;
		if (Surface.bRegularPositiveParameterization)
		{
			const double SegmentBounds[4] = {
				0.0,
				Surface.VerticalSegmentParameterWidth,
				Surface.VerticalSegmentParameterWidth +
					Surface.TransitionParameterWidth,
				1.0 };
			for (int32 SegmentIndex = 0; SegmentIndex < 3; ++SegmentIndex)
			{
				FOpenRimCanonicalTensorSurface& Tensor =
					OpenRimCanonicalTensorSurfaces.AddDefaulted_GetRef();
				Tensor.SourceFitId = Surface.FitId;
				Tensor.SegmentIndex = SegmentIndex;
				Tensor.MinimumCanonicalRimParameter = SegmentBounds[SegmentIndex];
				Tensor.MaximumCanonicalRimParameter = SegmentBounds[SegmentIndex + 1];
				const int32 DegreeU = SegmentIndex == 1 ? 13 : 5;
				const bool bConverted = BuildTensorSurfaceFromPolynomialEvaluator(
					DegreeU, 5,
					[&](const double LocalU, const double T)
					{
						return Surface.EvaluateCanonicalSurface(FMath::Lerp(
							Tensor.MinimumCanonicalRimParameter,
							Tensor.MaximumCanonicalRimParameter, LocalU), T);
					},
					Tensor.Surface, Tensor.MaximumConversionErrorCm);
				if (!bConverted)
				{
					OpenRimCanonicalTensorSurfaces.Reset();
					break;
				}
			}
		}
	}
	struct FStationMember
	{
		double EndDepth = 0.0;
		double EndOffset = 0.0;
		double BoundaryPlaneTangent = 0.0;
		double LongitudinalTangent = 0.0;
		int32 OpeningSide = 0;
		int32 TransverseSide = 0;
	};
	auto MakeMember = [](const FOpenRimTransverseSection& Section)
	{
		return FStationMember{
			Section.LongitudinalTangentDepthCm,
			Section.LongitudinalTangentOpeningOffsetCm,
			Section.IndividualC2BoundaryPlaneTangentMagnitudeCm,
			Section.IndividualC2LongitudinalTangentMagnitudeCm,
			Section.SliceOrigin.X >= 0.0 ? 1 : -1,
			Section.SliceOrigin.Y >= 0.0 ? 1 : -1 };
	};
	auto AddStation = [&](const EOpenRimC2LoftStationFamily Family,
		const double CanonicalParameter, const TArray<FStationMember>& Members)
	{
		if (Members.IsEmpty()) return;
		FOpenRimC2LoftStation& Station =
			OpenRimC2LoftStations.AddDefaulted_GetRef();
		uint64 ParameterBits = 0;
		static_assert(sizeof(ParameterBits) == sizeof(CanonicalParameter));
		FMemory::Memcpy(&ParameterBits, &CanonicalParameter,
			sizeof(ParameterBits));
		Station.StationId = CombineStableIds(
			StableStringId(TEXT("OpenRim.C2LoftStation.XY.V1")),
			CombineStableIds(static_cast<uint64>(Family) + 1ull, ParameterBits));
		Station.Family = Family;
		Station.CanonicalRimParameter = CanonicalParameter;
		Station.MemberCount = Members.Num();
		int32 OpeningMask = 0;
		int32 SideMask = 0;
		for (const FStationMember& Member : Members)
		{
			Station.MeanEndDepthCm += Member.EndDepth;
			Station.MeanEndOpeningOffsetCm += Member.EndOffset;
			Station.MeanBoundaryPlaneTangentCm += Member.BoundaryPlaneTangent;
			Station.MeanLongitudinalTangentCm += Member.LongitudinalTangent;
			OpeningMask |= Member.OpeningSide > 0 ? 1 : 2;
			SideMask |= Member.TransverseSide > 0 ? 1 : 2;
		}
		const double InverseCount = 1.0 / Members.Num();
		Station.MeanEndDepthCm *= InverseCount;
		Station.MeanEndOpeningOffsetCm *= InverseCount;
		Station.MeanBoundaryPlaneTangentCm *= InverseCount;
		Station.MeanLongitudinalTangentCm *= InverseCount;
		auto Median = [&](auto Select)
		{
			TArray<double> Values;
			for (const FStationMember& Member : Members)
				Values.Add(Select(Member));
			Algo::Sort(Values);
			const int32 Middle = Values.Num() / 2;
			return Values.Num() % 2 != 0 ? Values[Middle]
				: 0.5 * (Values[Middle - 1] + Values[Middle]);
		};
		Station.MedianEndDepthCm = Median(
			[](const FStationMember& Member) { return Member.EndDepth; });
		Station.MedianEndOpeningOffsetCm = Median(
			[](const FStationMember& Member) { return Member.EndOffset; });
		Station.MedianBoundaryPlaneTangentCm = Median(
			[](const FStationMember& Member) { return Member.BoundaryPlaneTangent; });
		Station.MedianLongitudinalTangentCm = Median(
			[](const FStationMember& Member)
			{
				return Member.LongitudinalTangent;
			});
		double SquaredSpread = 0.0;
		for (const FStationMember& Member : Members)
		{
			for (const double Difference : {
				Member.EndDepth - Station.MeanEndDepthCm,
				Member.EndOffset - Station.MeanEndOpeningOffsetCm,
				Member.BoundaryPlaneTangent - Station.MeanBoundaryPlaneTangentCm,
				Member.LongitudinalTangent -
					Station.MeanLongitudinalTangentCm })
			{
				SquaredSpread += Difference * Difference;
				Station.MaximumParameterSpreadCm = FMath::Max(
					Station.MaximumParameterSpreadCm, FMath::Abs(Difference));
			}
		}
		Station.RootMeanSquareParameterSpreadCm = FMath::Sqrt(
			SquaredSpread / (4.0 * Members.Num()));
		Station.bHasBothOpenings = OpeningMask == 3;
		Station.bHasBothSides = SideMask == 3 ||
			(Family == EOpenRimC2LoftStationFamily::HorizontalSpan &&
				FMath::IsNearlyEqual(CanonicalParameter, 1.0, 1.0e-8));
	};
	for (const double Parameter : { 0.1 / 3.0, 0.2 / 3.0,
		0.3 / 3.0, 0.4 / 3.0, 0.5 / 3.0 })
	{
		TArray<FStationMember> Members;
		for (const int32 SectionIndex : LoftMembers)
		{
			const FOpenRimTransverseSection& Section =
				OpenRimTransverseSections[SectionIndex];
			if ((Section.Feature == EOpenRimFeature::NegativeVertical ||
				Section.Feature == EOpenRimFeature::PositiveVertical) &&
				FMath::IsNearlyEqual(Section.CanonicalRimParameter,
					Parameter, 1.0e-8))
			{
				Members.Add(MakeMember(Section));
			}
		}
		AddStation(EOpenRimC2LoftStationFamily::VerticalSegment, Parameter, Members);
	}
	for (const double Parameter : { 0.75, 5.0 / 6.0, 11.0 / 12.0, 1.0 })
	{
		TArray<FStationMember> Members;
		for (const int32 SectionIndex : LoftMembers)
		{
			const FOpenRimTransverseSection& Section =
				OpenRimTransverseSections[SectionIndex];
			if (Section.Feature == EOpenRimFeature::HorizontalSpan &&
				FMath::IsNearlyEqual(Section.CanonicalRimParameter,
					Parameter, 1.0e-8))
			{
				Members.Add(MakeMember(Section));
			}
		}
		AddStation(EOpenRimC2LoftStationFamily::HorizontalSpan,
			Parameter, Members);
	}
	for (const double Parameter : { 0.43, 0.47, 0.51, 0.55, 0.58 })
	{
		TArray<FStationMember> Members;
		for (const int32 OpeningSide : { -1, 1 })
		{
			for (const int32 TransverseSide : { -1, 1 })
			{
				TArray<int32> Lane;
				for (const int32 SectionIndex : LoftMembers)
				{
					const FOpenRimTransverseSection& Section =
						OpenRimTransverseSections[SectionIndex];
					if (Section.Feature == EOpenRimFeature::UpperTransition &&
						(Section.SliceOrigin.X >= 0.0 ? 1 : -1) == OpeningSide &&
						(Section.SliceOrigin.Y >= 0.0 ? 1 : -1) == TransverseSide)
					{
						Lane.Add(SectionIndex);
					}
				}
				Algo::Sort(Lane, [&](const int32 A, const int32 B)
				{
					return OpenRimTransverseSections[A].CanonicalRimParameter <
						OpenRimTransverseSections[B].CanonicalRimParameter;
				});
				for (int32 UpperIndex = 1; UpperIndex < Lane.Num(); ++UpperIndex)
				{
					const FOpenRimTransverseSection& Lower =
						OpenRimTransverseSections[Lane[UpperIndex - 1]];
					const FOpenRimTransverseSection& Upper =
						OpenRimTransverseSections[Lane[UpperIndex]];
					if (Parameter < Lower.CanonicalRimParameter - 1.0e-9 ||
						Parameter > Upper.CanonicalRimParameter + 1.0e-9)
						continue;
					const double Alpha = (Parameter -
						Lower.CanonicalRimParameter) /
						(Upper.CanonicalRimParameter -
							Lower.CanonicalRimParameter);
					const FStationMember A = MakeMember(Lower);
					const FStationMember B = MakeMember(Upper);
					Members.Add({
						FMath::Lerp(A.EndDepth, B.EndDepth, Alpha),
						FMath::Lerp(A.EndOffset, B.EndOffset, Alpha),
						FMath::Lerp(A.BoundaryPlaneTangent,
							B.BoundaryPlaneTangent, Alpha),
						FMath::Lerp(A.LongitudinalTangent,
							B.LongitudinalTangent, Alpha),
						OpeningSide, TransverseSide });
					break;
				}
			}
		}
		AddStation(EOpenRimC2LoftStationFamily::Transition,
			Parameter, Members);
	}
	struct FWeightedSection
	{
		int32 SectionIndex = INDEX_NONE;
		double Weight = 0.0;
	};
	auto OptimizeGeometricParameters = [&](const TArray<FWeightedSection>& Members,
		double InOutParameters[4], double& OutRms, double& OutMaximum)
	{
		auto Objective = [&](const double Parameters[4], double* Maximum)
		{
			double WeightedMemberMean = 0.0;
			double WeightSum = 0.0;
			double LocalMaximum = 0.0;
			for (const FWeightedSection& Member : Members)
			{
				if (Member.Weight <= 0.0) continue;
				const FOpenRimTransverseSection& Section =
					OpenRimTransverseSections[Member.SectionIndex];
				double SquaredSum = 0.0;
				for (int32 PointOffset = 0;
					PointOffset < Section.TransitionPointCount; ++PointOffset)
				{
					const FVector2d& Point = OpenRimTransitionSectionPoints[
						Section.FirstTransitionPointIndex + PointOffset];
					double BestSquared = TNumericLimits<double>::Max();
					for (int32 Sample = 0; Sample <= 128; ++Sample)
						BestSquared = FMath::Min(BestSquared,
							FVector2d::DistSquared(Point, Evaluate(
								static_cast<double>(Sample) / 128.0,
								Parameters[0], Parameters[1],
								Parameters[2], Parameters[3])));
					SquaredSum += BestSquared;
					LocalMaximum = FMath::Max(LocalMaximum,
						FMath::Sqrt(BestSquared));
				}
				WeightedMemberMean += Member.Weight * SquaredSum /
					Section.TransitionPointCount;
				WeightSum += Member.Weight;
			}
			if (Maximum) *Maximum = LocalMaximum;
			return WeightSum > 0.0
				? WeightedMemberMean / WeightSum
				: TNumericLimits<double>::Max();
		};
		double BestObjective = Objective(InOutParameters, nullptr);
		double Steps[4];
		for (int32 ParameterIndex = 0; ParameterIndex < 4; ++ParameterIndex)
			Steps[ParameterIndex] = FMath::Max(
				2.0, 0.25 * InOutParameters[ParameterIndex]);
		for (int32 Iteration = 0; Iteration < 64; ++Iteration)
		{
			bool bImproved = false;
			for (int32 ParameterIndex = 0; ParameterIndex < 4; ++ParameterIndex)
			{
				for (const double Sign : { -1.0, 1.0 })
				{
					double Trial[4];
					for (int32 Index = 0; Index < 4; ++Index)
						Trial[Index] = InOutParameters[Index];
					Trial[ParameterIndex] = FMath::Max(1.0e-3,
						Trial[ParameterIndex] + Sign * Steps[ParameterIndex]);
					const double TrialObjective = Objective(Trial, nullptr);
					if (TrialObjective < BestObjective)
					{
						BestObjective = TrialObjective;
						InOutParameters[ParameterIndex] = Trial[ParameterIndex];
						bImproved = true;
					}
				}
			}
			if (!bImproved)
				for (double& Step : Steps) Step *= 0.5;
			if (Algo::AllOf(Steps, [](const double Step)
			{
				return Step < 1.0e-4;
			})) break;
		}
		OutRms = FMath::Sqrt(BestObjective);
		Objective(InOutParameters, &OutMaximum);
		return FMath::IsFinite(BestObjective);
	};
	for (FOpenRimC2LoftStation& Station : OpenRimC2LoftStations)
	{
		TArray<FWeightedSection> Members;
		if (Station.Family != EOpenRimC2LoftStationFamily::Transition)
		{
			for (const int32 SectionIndex : LoftMembers)
			{
				const FOpenRimTransverseSection& Section =
					OpenRimTransverseSections[SectionIndex];
				const bool bFamilyMatches =
					Station.Family == EOpenRimC2LoftStationFamily::VerticalSegment
						? Section.Feature == EOpenRimFeature::NegativeVertical ||
							Section.Feature == EOpenRimFeature::PositiveVertical
						: Section.Feature == EOpenRimFeature::HorizontalSpan;
				if (bFamilyMatches && FMath::IsNearlyEqual(
					Section.CanonicalRimParameter,
					Station.CanonicalRimParameter, 1.0e-8))
					Members.Add({ SectionIndex, 1.0 });
			}
		}
		else
		{
			for (const int32 OpeningSide : { -1, 1 })
			{
				for (const int32 TransverseSide : { -1, 1 })
				{
					TArray<int32> Lane;
					for (const int32 SectionIndex : LoftMembers)
					{
						const FOpenRimTransverseSection& Section =
							OpenRimTransverseSections[SectionIndex];
						if (Section.Feature == EOpenRimFeature::UpperTransition &&
							(Section.SliceOrigin.X >= 0.0 ? 1 : -1) == OpeningSide &&
							(Section.SliceOrigin.Y >= 0.0 ? 1 : -1) ==
								TransverseSide)
							Lane.Add(SectionIndex);
					}
					Algo::Sort(Lane, [&](const int32 A, const int32 B)
					{
						return OpenRimTransverseSections[A].CanonicalRimParameter <
							OpenRimTransverseSections[B].CanonicalRimParameter;
					});
					for (int32 UpperIndex = 1; UpperIndex < Lane.Num(); ++UpperIndex)
					{
						const FOpenRimTransverseSection& Lower =
							OpenRimTransverseSections[Lane[UpperIndex - 1]];
						const FOpenRimTransverseSection& Upper =
							OpenRimTransverseSections[Lane[UpperIndex]];
						if (Station.CanonicalRimParameter <
								Lower.CanonicalRimParameter - 1.0e-9 ||
							Station.CanonicalRimParameter >
								Upper.CanonicalRimParameter + 1.0e-9) continue;
						const double Alpha = (Station.CanonicalRimParameter -
							Lower.CanonicalRimParameter) /
							(Upper.CanonicalRimParameter -
								Lower.CanonicalRimParameter);
						if (Alpha < 1.0 - 1.0e-12)
							Members.Add({ Lane[UpperIndex - 1], 1.0 - Alpha });
						if (Alpha > 1.0e-12)
							Members.Add({ Lane[UpperIndex], Alpha });
						break;
					}
				}
			}
		}
		double Parameters[4] = {
			Station.MeanEndDepthCm,
			Station.MeanEndOpeningOffsetCm,
			Station.MeanBoundaryPlaneTangentCm,
			Station.MeanLongitudinalTangentCm };
		Station.bGeometricFitValid = OptimizeGeometricParameters(Members,
			Parameters, Station.GeometricFitRootMeanSquareResidualCm,
			Station.GeometricFitMaximumResidualCm);
		Station.GeometricFitEndDepthCm = Parameters[0];
		Station.GeometricFitEndOpeningOffsetCm = Parameters[1];
		Station.GeometricFitBoundaryPlaneTangentCm = Parameters[2];
		Station.GeometricFitLongitudinalTangentCm = Parameters[3];
	}
	if (!LoftMembers.IsEmpty())
	{
		auto BernsteinBasis = [](const double S, double OutBasis[6])
		{
			const double U = 1.0 - S;
			OutBasis[0] = U * U * U * U * U;
			OutBasis[1] = 5.0 * S * U * U * U * U;
			OutBasis[2] = 10.0 * S * S * U * U * U;
			OutBasis[3] = 10.0 * S * S * S * U * U;
			OutBasis[4] = 5.0 * S * S * S * S * U;
			OutBasis[5] = S * S * S * S * S;
		};
		auto SolveCoefficients = [&](auto Target, double OutCoefficients[6])
		{
			double Matrix[6][7] = {};
			for (const int32 SectionIndex : LoftMembers)
			{
				const FOpenRimTransverseSection& Section =
					OpenRimTransverseSections[SectionIndex];
				double Basis[6];
				BernsteinBasis(Section.CanonicalRimParameter, Basis);
				for (int32 Row = 0; Row < 6; ++Row)
				{
					for (int32 Column = 0; Column < 6; ++Column)
						Matrix[Row][Column] += Basis[Row] * Basis[Column];
					Matrix[Row][6] += Basis[Row] * Target(Section);
				}
			}
			for (int32 Diagonal = 0; Diagonal < 6; ++Diagonal)
			{
				int32 Pivot = Diagonal;
				for (int32 Row = Diagonal + 1; Row < 6; ++Row)
					if (FMath::Abs(Matrix[Row][Diagonal]) >
						FMath::Abs(Matrix[Pivot][Diagonal])) Pivot = Row;
				if (Pivot != Diagonal)
					for (int32 Column = Diagonal; Column < 7; ++Column)
						Swap(Matrix[Diagonal][Column], Matrix[Pivot][Column]);
				const double Divisor = Matrix[Diagonal][Diagonal];
				if (FMath::Abs(Divisor) <= 1.0e-12) return false;
				for (int32 Column = Diagonal; Column < 7; ++Column)
					Matrix[Diagonal][Column] /= Divisor;
				for (int32 Row = 0; Row < 6; ++Row)
				{
					if (Row == Diagonal) continue;
					const double Factor = Matrix[Row][Diagonal];
					for (int32 Column = Diagonal; Column < 7; ++Column)
						Matrix[Row][Column] -= Factor * Matrix[Diagonal][Column];
				}
			}
			for (int32 Index = 0; Index < 6; ++Index)
				OutCoefficients[Index] = Matrix[Index][6];
			return true;
		};
		auto EvaluatePolynomial = [&](const double Coefficients[6],
			const double S)
		{
			double Basis[6];
			BernsteinBasis(S, Basis);
			double Value = 0.0;
			for (int32 Index = 0; Index < 6; ++Index)
				Value += Basis[Index] * Coefficients[Index];
			return Value;
		};

		FOpenRimC2LoftFit& Loft = OpenRimC2LoftFits.AddDefaulted_GetRef();
		Loft.FitId = StableStringId(TEXT("OpenRim.C2Loft.XYShared.V1"));
		Loft.Model = EOpenRimC2LoftModel::GlobalBernstein5;
		Loft.CoefficientCount = 6;
		Loft.MemberCount = LoftMembers.Num();
		const bool bSolved =
			SolveCoefficients([](const FOpenRimTransverseSection& Section)
			{
				return Section.LongitudinalTangentDepthCm;
			}, Loft.EndDepthCoefficients) &&
			SolveCoefficients([](const FOpenRimTransverseSection& Section)
			{
				return Section.LongitudinalTangentOpeningOffsetCm;
			}, Loft.EndOpeningOffsetCoefficients) &&
			SolveCoefficients([](const FOpenRimTransverseSection& Section)
			{
				return Section.IndividualC2BoundaryPlaneTangentMagnitudeCm;
			}, Loft.BoundaryPlaneTangentCoefficients) &&
			SolveCoefficients([](const FOpenRimTransverseSection& Section)
			{
				return Section.IndividualC2LongitudinalTangentMagnitudeCm;
			}, Loft.LongitudinalTangentCoefficients);
		Loft.MinimumSampledParameterValueCm = TNumericLimits<double>::Max();
		if (bSolved)
		{
			for (int32 Sample = 0; Sample <= 256; ++Sample)
			{
				const double S = static_cast<double>(Sample) / 256.0;
				for (const double* Coefficients : {
					Loft.EndDepthCoefficients,
					Loft.EndOpeningOffsetCoefficients,
					Loft.BoundaryPlaneTangentCoefficients,
					Loft.LongitudinalTangentCoefficients })
				{
					Loft.MinimumSampledParameterValueCm = FMath::Min(
						Loft.MinimumSampledParameterValueCm,
						EvaluatePolynomial(Coefficients, S));
				}
			}
			for (const int32 SectionIndex : LoftMembers)
			{
				const FOpenRimTransverseSection& Section =
					OpenRimTransverseSections[SectionIndex];
				const double S = Section.CanonicalRimParameter;
				const double EndDepth = EvaluatePolynomial(
					Loft.EndDepthCoefficients, S);
				const double EndOffset = EvaluatePolynomial(
					Loft.EndOpeningOffsetCoefficients, S);
				const double BoundaryPlaneTangent = EvaluatePolynomial(
					Loft.BoundaryPlaneTangentCoefficients, S);
				const double LongitudinalTangent = EvaluatePolynomial(
					Loft.LongitudinalTangentCoefficients, S);
				double SquaredSum = 0.0;
				for (int32 PointOffset = 0;
					PointOffset < Section.TransitionPointCount; ++PointOffset)
				{
					const FVector2d& Point = OpenRimTransitionSectionPoints[
						Section.FirstTransitionPointIndex + PointOffset];
					double BestSquared = TNumericLimits<double>::Max();
					for (int32 Sample = 0; Sample <= 128; ++Sample)
						BestSquared = FMath::Min(BestSquared,
							FVector2d::DistSquared(Point, Evaluate(
								static_cast<double>(Sample) / 128.0, EndDepth,
								EndOffset, BoundaryPlaneTangent,
								LongitudinalTangent)));
					SquaredSum += BestSquared;
					Loft.MaximumResidualCm = FMath::Max(
						Loft.MaximumResidualCm, FMath::Sqrt(BestSquared));
				}
				const double MemberRms = FMath::Sqrt(
					SquaredSum / Section.TransitionPointCount);
				Loft.MaximumMemberRootMeanSquareResidualCm = FMath::Max(
					Loft.MaximumMemberRootMeanSquareResidualCm, MemberRms);
				Loft.BalancedRootMeanSquareResidualCm +=
					SquaredSum / Section.TransitionPointCount;
				Loft.SampleCount += Section.TransitionPointCount;
			}
			Loft.BalancedRootMeanSquareResidualCm = FMath::Sqrt(
				Loft.BalancedRootMeanSquareResidualCm / Loft.MemberCount);
		}
		Loft.bExactXYMirrorPlacement = bSolved;
		Loft.bC2AlongRimByPolynomialConstruction = bSolved;
		Loft.bRegularPositiveParameterization = bSolved &&
			Loft.MinimumSampledParameterValueCm > 1.0e-3;

		auto PiecewiseBasis = [](const double S, double OutBasis[3])
		{
			if (S <= 1.0 / 3.0)
			{
				OutBasis[0] = 1.0;
				OutBasis[1] = 0.0;
				OutBasis[2] = 0.0;
				return;
			}
			if (S >= 2.0 / 3.0)
			{
				OutBasis[0] = 0.0;
				OutBasis[1] = 1.0;
				OutBasis[2] = 0.0;
				return;
			}
			const double T = 3.0 * S - 1.0;
			const double T2 = T * T;
			const double T3 = T2 * T;
			const double H = T3 * (10.0 + T * (-15.0 + 6.0 * T));
			const double G = 64.0 * T3 * (1.0 - T) *
				(1.0 - T) * (1.0 - T);
			OutBasis[0] = 1.0 - H;
			OutBasis[1] = H;
			OutBasis[2] = G;
		};
		auto SolvePiecewise = [&](auto Target, double OutCoefficients[6])
		{
			double Matrix[3][4] = {};
			for (const int32 SectionIndex : LoftMembers)
			{
				const FOpenRimTransverseSection& Section =
					OpenRimTransverseSections[SectionIndex];
				double Basis[3];
				PiecewiseBasis(Section.CanonicalRimParameter, Basis);
				for (int32 Row = 0; Row < 3; ++Row)
				{
					for (int32 Column = 0; Column < 3; ++Column)
						Matrix[Row][Column] += Basis[Row] * Basis[Column];
					Matrix[Row][3] += Basis[Row] * Target(Section);
				}
			}
			for (int32 Diagonal = 0; Diagonal < 3; ++Diagonal)
			{
				int32 Pivot = Diagonal;
				for (int32 Row = Diagonal + 1; Row < 3; ++Row)
					if (FMath::Abs(Matrix[Row][Diagonal]) >
						FMath::Abs(Matrix[Pivot][Diagonal])) Pivot = Row;
				if (Pivot != Diagonal)
					for (int32 Column = Diagonal; Column < 4; ++Column)
						Swap(Matrix[Diagonal][Column], Matrix[Pivot][Column]);
				const double Divisor = Matrix[Diagonal][Diagonal];
				if (FMath::Abs(Divisor) <= 1.0e-12) return false;
				for (int32 Column = Diagonal; Column < 4; ++Column)
					Matrix[Diagonal][Column] /= Divisor;
				for (int32 Row = 0; Row < 3; ++Row)
				{
					if (Row == Diagonal) continue;
					const double Factor = Matrix[Row][Diagonal];
					for (int32 Column = Diagonal; Column < 4; ++Column)
						Matrix[Row][Column] -= Factor * Matrix[Diagonal][Column];
				}
			}
			for (int32 Index = 0; Index < 3; ++Index)
				OutCoefficients[Index] = Matrix[Index][3];
			return true;
		};
		auto EvaluatePiecewise = [&](const double Coefficients[6],
			const double S)
		{
			double Basis[3];
			PiecewiseBasis(S, Basis);
			return Basis[0] * Coefficients[0] +
				Basis[1] * Coefficients[1] + Basis[2] * Coefficients[2];
		};

		FOpenRimC2LoftFit& Piecewise =
			OpenRimC2LoftFits.AddDefaulted_GetRef();
		Piecewise.FitId = StableStringId(
			TEXT("OpenRim.C2Loft.PiecewiseTransition.XYShared.V1"));
		Piecewise.Model = EOpenRimC2LoftModel::PiecewiseC2Transition;
		Piecewise.CoefficientCount = 3;
		Piecewise.MemberCount = LoftMembers.Num();
		const bool bPiecewiseSolved =
			SolvePiecewise([](const FOpenRimTransverseSection& Section)
			{
				return Section.LongitudinalTangentDepthCm;
			}, Piecewise.EndDepthCoefficients) &&
			SolvePiecewise([](const FOpenRimTransverseSection& Section)
			{
				return Section.LongitudinalTangentOpeningOffsetCm;
			}, Piecewise.EndOpeningOffsetCoefficients) &&
			SolvePiecewise([](const FOpenRimTransverseSection& Section)
			{
				return Section.IndividualC2BoundaryPlaneTangentMagnitudeCm;
			}, Piecewise.BoundaryPlaneTangentCoefficients) &&
			SolvePiecewise([](const FOpenRimTransverseSection& Section)
			{
				return Section.IndividualC2LongitudinalTangentMagnitudeCm;
			}, Piecewise.LongitudinalTangentCoefficients);
		Piecewise.MinimumSampledParameterValueCm =
			TNumericLimits<double>::Max();
		if (bPiecewiseSolved)
		{
			for (int32 Sample = 0; Sample <= 256; ++Sample)
			{
				const double S = static_cast<double>(Sample) / 256.0;
				for (const double* Coefficients : {
					Piecewise.EndDepthCoefficients,
					Piecewise.EndOpeningOffsetCoefficients,
					Piecewise.BoundaryPlaneTangentCoefficients,
					Piecewise.LongitudinalTangentCoefficients })
				{
					Piecewise.MinimumSampledParameterValueCm = FMath::Min(
						Piecewise.MinimumSampledParameterValueCm,
						EvaluatePiecewise(Coefficients, S));
				}
			}
			for (const int32 SectionIndex : LoftMembers)
			{
				const FOpenRimTransverseSection& Section =
					OpenRimTransverseSections[SectionIndex];
				const double S = Section.CanonicalRimParameter;
				const double EndDepth = EvaluatePiecewise(
					Piecewise.EndDepthCoefficients, S);
				const double EndOffset = EvaluatePiecewise(
					Piecewise.EndOpeningOffsetCoefficients, S);
				const double BoundaryPlaneTangent = EvaluatePiecewise(
					Piecewise.BoundaryPlaneTangentCoefficients, S);
				const double LongitudinalTangent = EvaluatePiecewise(
					Piecewise.LongitudinalTangentCoefficients, S);
				double SquaredSum = 0.0;
				for (int32 PointOffset = 0;
					PointOffset < Section.TransitionPointCount; ++PointOffset)
				{
					const FVector2d& Point = OpenRimTransitionSectionPoints[
						Section.FirstTransitionPointIndex + PointOffset];
					double BestSquared = TNumericLimits<double>::Max();
					for (int32 Sample = 0; Sample <= 128; ++Sample)
						BestSquared = FMath::Min(BestSquared,
							FVector2d::DistSquared(Point, Evaluate(
								static_cast<double>(Sample) / 128.0, EndDepth,
								EndOffset, BoundaryPlaneTangent,
								LongitudinalTangent)));
					SquaredSum += BestSquared;
					Piecewise.MaximumResidualCm = FMath::Max(
						Piecewise.MaximumResidualCm, FMath::Sqrt(BestSquared));
				}
				const double MemberRms = FMath::Sqrt(
					SquaredSum / Section.TransitionPointCount);
				Piecewise.MaximumMemberRootMeanSquareResidualCm = FMath::Max(
					Piecewise.MaximumMemberRootMeanSquareResidualCm, MemberRms);
				Piecewise.BalancedRootMeanSquareResidualCm +=
					SquaredSum / Section.TransitionPointCount;
				Piecewise.SampleCount += Section.TransitionPointCount;
			}
			Piecewise.BalancedRootMeanSquareResidualCm = FMath::Sqrt(
				Piecewise.BalancedRootMeanSquareResidualCm /
					Piecewise.MemberCount);
		}
		Piecewise.bExactXYMirrorPlacement = bPiecewiseSolved;
		Piecewise.bC2AlongRimByPolynomialConstruction = bPiecewiseSolved;
		Piecewise.bRegularPositiveParameterization = bPiecewiseSolved &&
			Piecewise.MinimumSampledParameterValueCm > 1.0e-3;

		TArray<int32> BalancedStations;
		for (int32 StationIndex = 0;
			StationIndex < OpenRimC2LoftStations.Num(); ++StationIndex)
		{
			const FOpenRimC2LoftStation& Station =
				OpenRimC2LoftStations[StationIndex];
			if (Station.bHasBothOpenings && Station.bHasBothSides)
				BalancedStations.Add(StationIndex);
		}
		auto SolveStationCoefficients = [&](auto Target,
			double OutCoefficients[6])
		{
			double Matrix[6][7] = {};
			for (const int32 StationIndex : BalancedStations)
			{
				const FOpenRimC2LoftStation& Station =
					OpenRimC2LoftStations[StationIndex];
				double Basis[6];
				BernsteinBasis(Station.CanonicalRimParameter, Basis);
				for (int32 Row = 0; Row < 6; ++Row)
				{
					for (int32 Column = 0; Column < 6; ++Column)
						Matrix[Row][Column] += Basis[Row] * Basis[Column];
					Matrix[Row][6] += Basis[Row] * Target(Station);
				}
			}
			for (int32 Diagonal = 0; Diagonal < 6; ++Diagonal)
			{
				int32 Pivot = Diagonal;
				for (int32 Row = Diagonal + 1; Row < 6; ++Row)
					if (FMath::Abs(Matrix[Row][Diagonal]) >
						FMath::Abs(Matrix[Pivot][Diagonal])) Pivot = Row;
				if (Pivot != Diagonal)
					for (int32 Column = Diagonal; Column < 7; ++Column)
						Swap(Matrix[Diagonal][Column], Matrix[Pivot][Column]);
				const double Divisor = Matrix[Diagonal][Diagonal];
				if (FMath::Abs(Divisor) <= 1.0e-12) return false;
				for (int32 Column = Diagonal; Column < 7; ++Column)
					Matrix[Diagonal][Column] /= Divisor;
				for (int32 Row = 0; Row < 6; ++Row)
				{
					if (Row == Diagonal) continue;
					const double Factor = Matrix[Row][Diagonal];
					for (int32 Column = Diagonal; Column < 7; ++Column)
						Matrix[Row][Column] -=
							Factor * Matrix[Diagonal][Column];
				}
			}
			for (int32 Index = 0; Index < 6; ++Index)
				OutCoefficients[Index] = Matrix[Index][6];
			return true;
		};
		FOpenRimC2LoftFit& StationLoft =
			OpenRimC2LoftFits.AddDefaulted_GetRef();
		StationLoft.FitId = StableStringId(
			TEXT("OpenRim.C2Loft.SymmetryStations.XYShared.V1"));
		StationLoft.Model = EOpenRimC2LoftModel::SymmetryStationBernstein5;
		StationLoft.CoefficientCount = 6;
		StationLoft.MemberCount = LoftMembers.Num();
		const bool bStationSolved = BalancedStations.Num() >= 6 &&
			SolveStationCoefficients([](const FOpenRimC2LoftStation& Station)
			{
				return Station.MeanEndDepthCm;
			}, StationLoft.EndDepthCoefficients) &&
			SolveStationCoefficients([](const FOpenRimC2LoftStation& Station)
			{
				return Station.MeanEndOpeningOffsetCm;
			}, StationLoft.EndOpeningOffsetCoefficients) &&
			SolveStationCoefficients([](const FOpenRimC2LoftStation& Station)
			{
				return Station.MeanBoundaryPlaneTangentCm;
			}, StationLoft.BoundaryPlaneTangentCoefficients) &&
			SolveStationCoefficients([](const FOpenRimC2LoftStation& Station)
			{
				return Station.MeanLongitudinalTangentCm;
			}, StationLoft.LongitudinalTangentCoefficients);
		StationLoft.MinimumSampledParameterValueCm =
			TNumericLimits<double>::Max();
		if (bStationSolved)
		{
			for (int32 Sample = 0; Sample <= 256; ++Sample)
			{
				const double S = static_cast<double>(Sample) / 256.0;
				for (const double* Coefficients : {
					StationLoft.EndDepthCoefficients,
					StationLoft.EndOpeningOffsetCoefficients,
					StationLoft.BoundaryPlaneTangentCoefficients,
					StationLoft.LongitudinalTangentCoefficients })
				{
					StationLoft.MinimumSampledParameterValueCm = FMath::Min(
						StationLoft.MinimumSampledParameterValueCm,
						EvaluatePolynomial(Coefficients, S));
				}
			}
			for (const int32 SectionIndex : LoftMembers)
			{
				const FOpenRimTransverseSection& Section =
					OpenRimTransverseSections[SectionIndex];
				const double S = Section.CanonicalRimParameter;
				const double EndDepth = EvaluatePolynomial(
					StationLoft.EndDepthCoefficients, S);
				const double EndOffset = EvaluatePolynomial(
					StationLoft.EndOpeningOffsetCoefficients, S);
				const double BoundaryPlaneTangent = EvaluatePolynomial(
					StationLoft.BoundaryPlaneTangentCoefficients, S);
				const double LongitudinalTangent = EvaluatePolynomial(
					StationLoft.LongitudinalTangentCoefficients, S);
				double SquaredSum = 0.0;
				for (int32 PointOffset = 0;
					PointOffset < Section.TransitionPointCount; ++PointOffset)
				{
					const FVector2d& Point = OpenRimTransitionSectionPoints[
						Section.FirstTransitionPointIndex + PointOffset];
					double BestSquared = TNumericLimits<double>::Max();
					for (int32 Sample = 0; Sample <= 128; ++Sample)
						BestSquared = FMath::Min(BestSquared,
							FVector2d::DistSquared(Point, Evaluate(
								static_cast<double>(Sample) / 128.0,
								EndDepth, EndOffset, BoundaryPlaneTangent,
								LongitudinalTangent)));
					SquaredSum += BestSquared;
					StationLoft.MaximumResidualCm = FMath::Max(
						StationLoft.MaximumResidualCm,
						FMath::Sqrt(BestSquared));
				}
				const double MemberRms = FMath::Sqrt(
					SquaredSum / Section.TransitionPointCount);
				StationLoft.MaximumMemberRootMeanSquareResidualCm =
					FMath::Max(
						StationLoft.MaximumMemberRootMeanSquareResidualCm,
						MemberRms);
				StationLoft.BalancedRootMeanSquareResidualCm +=
					SquaredSum / Section.TransitionPointCount;
				StationLoft.SampleCount += Section.TransitionPointCount;
			}
			StationLoft.BalancedRootMeanSquareResidualCm = FMath::Sqrt(
				StationLoft.BalancedRootMeanSquareResidualCm /
					StationLoft.MemberCount);
		}
		StationLoft.bExactXYMirrorPlacement = bStationSolved;
		StationLoft.bC2AlongRimByPolynomialConstruction = bStationSolved;
		StationLoft.bRegularPositiveParameterization = bStationSolved &&
			StationLoft.MinimumSampledParameterValueCm > 1.0e-3;

		struct FLocalC2Knot
		{
			double S = 0.0;
			double Values[4] = {};
			double Derivatives[4] = {};
		};
		TArray<FLocalC2Knot> Knots;
		double VerticalSegmentValues[4] = {};
		double HorizontalSpanValues[4] = {};
		int32 VerticalSegmentStationCount = 0;
		int32 HorizontalSpanStationCount = 0;
		for (const int32 StationIndex : BalancedStations)
		{
			const FOpenRimC2LoftStation& Station =
				OpenRimC2LoftStations[StationIndex];
			const double Values[4] = {
				Station.MedianEndDepthCm,
				Station.MedianEndOpeningOffsetCm,
				Station.MedianBoundaryPlaneTangentCm,
				Station.MedianLongitudinalTangentCm };
			if (Station.Family == EOpenRimC2LoftStationFamily::Transition)
			{
				FLocalC2Knot& Knot = Knots.AddDefaulted_GetRef();
				Knot.S = Station.CanonicalRimParameter;
				for (int32 ParameterIndex = 0; ParameterIndex < 4;
					++ParameterIndex)
					Knot.Values[ParameterIndex] = Values[ParameterIndex];
			}
			else
			{
				double* FamilyValues =
					Station.Family == EOpenRimC2LoftStationFamily::VerticalSegment
						? VerticalSegmentValues : HorizontalSpanValues;
				for (int32 ParameterIndex = 0; ParameterIndex < 4;
					++ParameterIndex)
					FamilyValues[ParameterIndex] += Values[ParameterIndex];
				if (Station.Family == EOpenRimC2LoftStationFamily::VerticalSegment)
					++VerticalSegmentStationCount;
				else
					++HorizontalSpanStationCount;
			}
		}
		if (VerticalSegmentStationCount > 0 && HorizontalSpanStationCount > 0)
		{
			for (int32 ParameterIndex = 0; ParameterIndex < 4; ++ParameterIndex)
			{
				VerticalSegmentValues[ParameterIndex] /= VerticalSegmentStationCount;
				HorizontalSpanValues[ParameterIndex] /= HorizontalSpanStationCount;
			}
			for (const double S : { 0.0, 1.0 / 3.0 })
			{
				FLocalC2Knot& Knot = Knots.AddDefaulted_GetRef();
				Knot.S = S;
				for (int32 ParameterIndex = 0; ParameterIndex < 4;
					++ParameterIndex)
					Knot.Values[ParameterIndex] = VerticalSegmentValues[ParameterIndex];
			}
			for (const double S : { 2.0 / 3.0, 1.0 })
			{
				FLocalC2Knot& Knot = Knots.AddDefaulted_GetRef();
				Knot.S = S;
				for (int32 ParameterIndex = 0; ParameterIndex < 4;
					++ParameterIndex)
					Knot.Values[ParameterIndex] = HorizontalSpanValues[ParameterIndex];
			}
		}
		Algo::Sort(Knots, [](const FLocalC2Knot& A, const FLocalC2Knot& B)
		{
			return A.S < B.S;
		});
		if (Knots.Num() >= 3)
		{
			for (int32 ParameterIndex = 0; ParameterIndex < 4;
				++ParameterIndex)
			{
				TArray<double> Intervals;
				TArray<double> Secants;
				for (int32 Index = 0; Index + 1 < Knots.Num(); ++Index)
				{
					const double Interval = Knots[Index + 1].S - Knots[Index].S;
					Intervals.Add(Interval);
					Secants.Add((Knots[Index + 1].Values[ParameterIndex] -
						Knots[Index].Values[ParameterIndex]) / Interval);
				}
				auto ClampEndpointDerivative = [](const double Derivative,
					const double Secant)
				{
					if (Derivative * Secant <= 0.0) return 0.0;
					return FMath::Abs(Derivative) > 3.0 * FMath::Abs(Secant)
						? 3.0 * Secant : Derivative;
				};
				Knots[0].Derivatives[ParameterIndex] =
					ClampEndpointDerivative(
						((2.0 * Intervals[0] + Intervals[1]) * Secants[0] -
							Intervals[0] * Secants[1]) /
						(Intervals[0] + Intervals[1]), Secants[0]);
				const int32 Last = Knots.Num() - 1;
				Knots[Last].Derivatives[ParameterIndex] =
					ClampEndpointDerivative(
						((2.0 * Intervals.Last() + Intervals[Intervals.Num() - 2]) *
							Secants.Last() - Intervals.Last() *
							Secants[Secants.Num() - 2]) /
						(Intervals.Last() + Intervals[Intervals.Num() - 2]),
						Secants.Last());
				for (int32 Index = 1; Index < Last; ++Index)
				{
					const double Previous = Secants[Index - 1];
					const double Next = Secants[Index];
					if (Previous * Next <= 0.0)
					{
						Knots[Index].Derivatives[ParameterIndex] = 0.0;
						continue;
					}
					const double PreviousInterval = Intervals[Index - 1];
					const double NextInterval = Intervals[Index];
					const double W1 = 2.0 * NextInterval + PreviousInterval;
					const double W2 = NextInterval + 2.0 * PreviousInterval;
					Knots[Index].Derivatives[ParameterIndex] = (W1 + W2) /
						(W1 / Previous + W2 / Next);
				}
			}

			FOpenRimC2LoftFit& Local =
				OpenRimC2LoftFits.AddDefaulted_GetRef();
			Local.FitId = StableStringId(
				TEXT("OpenRim.C2Loft.SymmetryStations.ArchitecturalLocalC2.V1"));
			Local.Model =
				EOpenRimC2LoftModel::SymmetryStationArchitecturalLocalC2;
			Local.FirstSegmentIndex = OpenRimC2LoftSegments.Num();
			Local.SegmentCount = Knots.Num() - 1;
			Local.MemberCount = LoftMembers.Num();
			for (int32 Index = 0; Index + 1 < Knots.Num(); ++Index)
			{
				FOpenRimC2LoftSegment& Segment =
					OpenRimC2LoftSegments.AddDefaulted_GetRef();
				Segment.StartCanonicalRimParameter = Knots[Index].S;
				Segment.EndCanonicalRimParameter = Knots[Index + 1].S;
				for (int32 ParameterIndex = 0; ParameterIndex < 4;
					++ParameterIndex)
				{
					Segment.StartValuesCm[ParameterIndex] =
						Knots[Index].Values[ParameterIndex];
					Segment.EndValuesCm[ParameterIndex] =
						Knots[Index + 1].Values[ParameterIndex];
					Segment.StartDerivativesCm[ParameterIndex] =
						Knots[Index].Derivatives[ParameterIndex];
					Segment.EndDerivativesCm[ParameterIndex] =
						Knots[Index + 1].Derivatives[ParameterIndex];
				}
			}
			auto EvaluateLocalParameter = [](const FOpenRimC2LoftSegment& Segment,
				const int32 ParameterIndex, const double S,
				double* OutFirstDerivative, double* OutSecondDerivative)
			{
				const double Interval = Segment.EndCanonicalRimParameter -
					Segment.StartCanonicalRimParameter;
				const double T = FMath::Clamp((S -
					Segment.StartCanonicalRimParameter) / Interval, 0.0, 1.0);
				const double T2 = T * T;
				const double T3 = T2 * T;
				const double T4 = T3 * T;
				const double T5 = T4 * T;
				const double H00 = 1.0 - 10.0 * T3 + 15.0 * T4 - 6.0 * T5;
				const double H01 = 10.0 * T3 - 15.0 * T4 + 6.0 * T5;
				const double H10 = T - 6.0 * T3 + 8.0 * T4 - 3.0 * T5;
				const double H11 = -4.0 * T3 + 7.0 * T4 - 3.0 * T5;
				const double V0 = Segment.StartValuesCm[ParameterIndex];
				const double V1 = Segment.EndValuesCm[ParameterIndex];
				const double D0 = Segment.StartDerivativesCm[ParameterIndex];
				const double D1 = Segment.EndDerivativesCm[ParameterIndex];
				if (OutFirstDerivative)
				{
					const double H00D = -30.0 * T2 + 60.0 * T3 - 30.0 * T4;
					const double H01D = 30.0 * T2 - 60.0 * T3 + 30.0 * T4;
					const double H10D = 1.0 - 18.0 * T2 + 32.0 * T3 - 15.0 * T4;
					const double H11D = -12.0 * T2 + 28.0 * T3 - 15.0 * T4;
					*OutFirstDerivative = (V0 * H00D + V1 * H01D) / Interval +
						D0 * H10D + D1 * H11D;
				}
				if (OutSecondDerivative)
				{
					const double H00DD = -60.0 * T + 180.0 * T2 - 120.0 * T3;
					const double H01DD = 60.0 * T - 180.0 * T2 + 120.0 * T3;
					const double H10DD = -36.0 * T + 96.0 * T2 - 60.0 * T3;
					const double H11DD = -24.0 * T + 84.0 * T2 - 60.0 * T3;
					*OutSecondDerivative =
						(V0 * H00DD + V1 * H01DD) / (Interval * Interval) +
						(D0 * H10DD + D1 * H11DD) / Interval;
				}
				return V0 * H00 + V1 * H01 +
					Interval * (D0 * H10 + D1 * H11);
			};
			auto FindLocalSegment = [&](const double S)
			{
				for (int32 Offset = 0; Offset < Local.SegmentCount; ++Offset)
				{
					const int32 SegmentIndex = Local.FirstSegmentIndex + Offset;
					if (S <= OpenRimC2LoftSegments[SegmentIndex].
						EndCanonicalRimParameter + 1.0e-12)
						return SegmentIndex;
				}
				return Local.FirstSegmentIndex + Local.SegmentCount - 1;
			};
			Local.MinimumSampledParameterValueCm = TNumericLimits<double>::Max();
			for (int32 Sample = 0; Sample <= 512; ++Sample)
			{
				const double S = static_cast<double>(Sample) / 512.0;
				const FOpenRimC2LoftSegment& Segment =
					OpenRimC2LoftSegments[FindLocalSegment(S)];
				for (int32 ParameterIndex = 0; ParameterIndex < 4;
					++ParameterIndex)
				{
					double FirstDerivative = 0.0;
					double SecondDerivative = 0.0;
					const double Value = EvaluateLocalParameter(Segment,
						ParameterIndex, S, &FirstDerivative, &SecondDerivative);
					Local.MinimumSampledParameterValueCm = FMath::Min(
						Local.MinimumSampledParameterValueCm, Value);
					Local.MaximumSampledFirstDerivativeMagnitudeCm = FMath::Max(
						Local.MaximumSampledFirstDerivativeMagnitudeCm,
						FMath::Abs(FirstDerivative));
					Local.MaximumSampledSecondDerivativeMagnitudeCm = FMath::Max(
						Local.MaximumSampledSecondDerivativeMagnitudeCm,
						FMath::Abs(SecondDerivative));
				}
			}
			for (int32 Offset = 0; Offset + 1 < Local.SegmentCount; ++Offset)
			{
				const FOpenRimC2LoftSegment& Left = OpenRimC2LoftSegments[
					Local.FirstSegmentIndex + Offset];
				const FOpenRimC2LoftSegment& Right = OpenRimC2LoftSegments[
					Local.FirstSegmentIndex + Offset + 1];
				for (int32 ParameterIndex = 0; ParameterIndex < 4;
					++ParameterIndex)
				{
					double LeftFirst = 0.0, LeftSecond = 0.0;
					double RightFirst = 0.0, RightSecond = 0.0;
					EvaluateLocalParameter(Left, ParameterIndex,
						Left.EndCanonicalRimParameter, &LeftFirst, &LeftSecond);
					EvaluateLocalParameter(Right, ParameterIndex,
						Right.StartCanonicalRimParameter, &RightFirst, &RightSecond);
					Local.MaximumFirstDerivativeJoinResidualCm = FMath::Max(
						Local.MaximumFirstDerivativeJoinResidualCm,
						FMath::Abs(LeftFirst - RightFirst));
					Local.MaximumSecondDerivativeJoinResidualCm = FMath::Max(
						Local.MaximumSecondDerivativeJoinResidualCm,
						FMath::Abs(LeftSecond - RightSecond));
				}
			}
			for (const int32 SectionIndex : LoftMembers)
			{
				const FOpenRimTransverseSection& Section =
					OpenRimTransverseSections[SectionIndex];
				const FOpenRimC2LoftSegment& Segment =
					OpenRimC2LoftSegments[FindLocalSegment(
						Section.CanonicalRimParameter)];
				double Parameters[4];
				for (int32 ParameterIndex = 0; ParameterIndex < 4;
					++ParameterIndex)
				{
					Parameters[ParameterIndex] = EvaluateLocalParameter(Segment,
						ParameterIndex, Section.CanonicalRimParameter,
						nullptr, nullptr);
				}
				double SquaredSum = 0.0;
				for (int32 PointOffset = 0;
					PointOffset < Section.TransitionPointCount; ++PointOffset)
				{
					const FVector2d& Point = OpenRimTransitionSectionPoints[
						Section.FirstTransitionPointIndex + PointOffset];
					double BestSquared = TNumericLimits<double>::Max();
					for (int32 Sample = 0; Sample <= 128; ++Sample)
						BestSquared = FMath::Min(BestSquared,
							FVector2d::DistSquared(Point, Evaluate(
								static_cast<double>(Sample) / 128.0,
								Parameters[0], Parameters[1],
								Parameters[2], Parameters[3])));
					SquaredSum += BestSquared;
					Local.MaximumResidualCm = FMath::Max(Local.MaximumResidualCm,
						FMath::Sqrt(BestSquared));
				}
				const double MemberRms = FMath::Sqrt(
					SquaredSum / Section.TransitionPointCount);
				Local.MaximumMemberRootMeanSquareResidualCm = FMath::Max(
					Local.MaximumMemberRootMeanSquareResidualCm, MemberRms);
				Local.BalancedRootMeanSquareResidualCm +=
					SquaredSum / Section.TransitionPointCount;
				Local.SampleCount += Section.TransitionPointCount;
			}
			Local.BalancedRootMeanSquareResidualCm = FMath::Sqrt(
				Local.BalancedRootMeanSquareResidualCm / Local.MemberCount);
			Local.bExactXYMirrorPlacement = true;
			Local.bC2AlongRimByPolynomialConstruction =
				Local.MaximumFirstDerivativeJoinResidualCm <= 1.0e-9 &&
				Local.MaximumSecondDerivativeJoinResidualCm <= 1.0e-9;
			Local.bRegularPositiveParameterization =
				Local.MinimumSampledParameterValueCm > 1.0e-3;
		}
	}
	// Fit the end curve of the tube prefix independently from the rear turn.
	// The samples are already expressed in the reflection-free canonical frame,
	// so this degree-five curve is a compact symmetry-balanced hypothesis rather
	// than a promotion of the selected collision triangles.
	if (OpenRimCanonicalSurfaceFits.Num() == 1)
	{
		struct FTubeTerminalSample
		{
			int32 SectionIndex = INDEX_NONE;
			double S = 0.0;
			FVector3d Position = FVector3d::ZeroVector;
		};
		TArray<FTubeTerminalSample> TubeSamples;
		for (int32 SectionIndex = 0;
			SectionIndex < OpenRimTransverseSections.Num(); ++SectionIndex)
		{
			const FOpenRimTransverseSection& Section =
				OpenRimTransverseSections[SectionIndex];
			if (!Section.bIndividualC2FitValid ||
				!Section.bTopologyCanonicalRimParameterValid ||
				Section.LongitudinalTubePointCount < 2) continue;
			const int32 SampleIndex = Section.FirstLongitudinalSampleIndex +
				Section.LongitudinalTubePointCount - 1;
			if (!OpenRimLongitudinalSamples.IsValidIndex(SampleIndex)) continue;
			TubeSamples.Add({ SectionIndex, Section.TopologyCanonicalRimParameter,
				OpenRimLongitudinalSamples[SampleIndex].CanonicalPositionCm });
		}
		if (TubeSamples.Num() >= 6)
		{
			auto SolveTubeCoordinate = [&](auto Select, double Out[6])
			{
				double Matrix[6][7] = {};
				for (const FTubeTerminalSample& Sample : TubeSamples)
				{
					double Basis[6];
					Bernstein5Basis(Sample.S, Basis);
					for (int32 Row = 0; Row < 6; ++Row)
					{
						for (int32 Column = 0; Column < 6; ++Column)
							Matrix[Row][Column] += Basis[Row] * Basis[Column];
						Matrix[Row][6] += Basis[Row] * Select(Sample.Position);
					}
				}
				for (int32 Diagonal = 0; Diagonal < 6; ++Diagonal)
				{
					int32 Pivot = Diagonal;
					for (int32 Row = Diagonal + 1; Row < 6; ++Row)
						if (FMath::Abs(Matrix[Row][Diagonal]) >
							FMath::Abs(Matrix[Pivot][Diagonal])) Pivot = Row;
					if (Pivot != Diagonal)
						for (int32 Column = Diagonal; Column < 7; ++Column)
							Swap(Matrix[Diagonal][Column], Matrix[Pivot][Column]);
					if (FMath::Abs(Matrix[Diagonal][Diagonal]) <= 1.0e-12)
						return false;
					const double Inverse = 1.0 / Matrix[Diagonal][Diagonal];
					for (int32 Column = Diagonal; Column < 7; ++Column)
						Matrix[Diagonal][Column] *= Inverse;
					for (int32 Row = 0; Row < 6; ++Row)
					{
						if (Row == Diagonal) continue;
						const double Factor = Matrix[Row][Diagonal];
						for (int32 Column = Diagonal; Column < 7; ++Column)
							Matrix[Row][Column] -= Factor * Matrix[Diagonal][Column];
					}
				}
				for (int32 Index = 0; Index < 6; ++Index) Out[Index] = Matrix[Index][6];
				return true;
			};
			FOpenRimCanonicalTubeFit& TubeFit =
				OpenRimCanonicalTubeFits.AddDefaulted_GetRef();
			TubeFit.FitId = StableStringId(TEXT("OpenRim.CanonicalTube.Terminal.V1"));
			TubeFit.SampleCount = TubeSamples.Num();
			const bool bSolved =
				SolveTubeCoordinate([](const FVector3d& P) { return P.X; },
					TubeFit.TerminalDepthCoefficients) &&
				SolveTubeCoordinate([](const FVector3d& P) { return P.Y; },
					TubeFit.TerminalYCoefficients) &&
				SolveTubeCoordinate([](const FVector3d& P) { return P.Z; },
					TubeFit.TerminalZCoefficients);
			if (bSolved)
			{
				double SquaredSum = 0.0;
				for (const FTubeTerminalSample& Sample : TubeSamples)
				{
					const double Residual = FVector3d::Distance(
						Sample.Position, TubeFit.EvaluateTerminal(Sample.S));
					SquaredSum += Residual * Residual;
					if (Residual > TubeFit.MaximumResidualCm)
					{
						TubeFit.MaximumResidualCm = Residual;
						TubeFit.MaximumResidualSectionIndex = Sample.SectionIndex;
					}
				}
				TubeFit.RootMeanSquareResidualCm = FMath::Sqrt(
					SquaredSum / TubeSamples.Num());
				TubeFit.bRegularFiniteTube = true;
				for (int32 Sample = 0; Sample <= 64; ++Sample)
				{
					const FVector3d P = TubeFit.EvaluateTerminal(
						static_cast<double>(Sample) / 64.0);
					TubeFit.bRegularFiniteTube &= FMath::IsFinite(P.X) &&
						FMath::IsFinite(P.Y) && FMath::IsFinite(P.Z) && P.X > 0.0;
				}
			}
		}
	}

	// The global canonical experiment above is retained as a diagnostic.  The
	// generated enclosure does not preserve a sufficiently tight four-way tube
	// orbit, so form one measured terminal curve per physical opening/side lane.
	// These fits are still recognition-only until the matching C2 tube patches
	// and their spatial certificates are available.
	if (OpenRimCanonicalSurfaceFits.Num() == 1)
	{
		struct FLaneTerminalSample
		{
			int32 SectionIndex = INDEX_NONE;
			double S = 0.0;
			FVector3d Position = FVector3d::ZeroVector;
		};
		for (const int32 OpeningSide : { -1, 1 })
		{
			for (const int32 TransverseSide : { -1, 1 })
			{
				TArray<FLaneTerminalSample> LaneSamples;
				for (int32 SectionIndex = 0;
					SectionIndex < OpenRimTransverseSections.Num(); ++SectionIndex)
				{
					const FOpenRimTransverseSection& Section =
						OpenRimTransverseSections[SectionIndex];
					if (!Section.bIndividualC2FitValid ||
						!Section.bTopologyCanonicalRimParameterValid ||
						Section.LongitudinalTubePointCount < 2 ||
						(Section.SliceOrigin.X >= 0.0 ? 1 : -1) != OpeningSide ||
						(Section.SliceOrigin.Y >= 0.0 ? 1 : -1) != TransverseSide)
					{
						continue;
					}
					const int32 SampleIndex = Section.FirstLongitudinalSampleIndex +
						Section.LongitudinalTubePointCount - 1;
					if (!OpenRimLongitudinalSamples.IsValidIndex(SampleIndex)) continue;
					LaneSamples.Add({ SectionIndex, Section.TopologyCanonicalRimParameter,
						OpenRimLongitudinalSamples[SampleIndex].CanonicalPositionCm });
				}
				LaneSamples.Sort([](const FLaneTerminalSample& A,
					const FLaneTerminalSample& B)
				{
					return A.S != B.S ? A.S < B.S : A.SectionIndex < B.SectionIndex;
				});
				TArray<FLaneTerminalSample> UniqueLaneSamples;
				for (const FLaneTerminalSample& Sample : LaneSamples)
				{
					if (UniqueLaneSamples.IsEmpty() || !FMath::IsNearlyEqual(
						UniqueLaneSamples.Last().S, Sample.S, 1.0e-7))
					{
						UniqueLaneSamples.Add(Sample);
						continue;
					}
					FLaneTerminalSample& Existing = UniqueLaneSamples.Last();
					Existing.Position = 0.5 * (Existing.Position + Sample.Position);
				}
				LaneSamples = MoveTemp(UniqueLaneSamples);
				if (LaneSamples.Num() < 6) continue;
				auto SolveLaneCoordinate = [&](auto Select, double Out[6])
				{
					double Matrix[6][7] = {};
					for (const FLaneTerminalSample& Sample : LaneSamples)
					{
						double Basis[6];
						Bernstein5Basis(Sample.S, Basis);
						for (int32 Row = 0; Row < 6; ++Row)
						{
							for (int32 Column = 0; Column < 6; ++Column)
								Matrix[Row][Column] += Basis[Row] * Basis[Column];
							Matrix[Row][6] += Basis[Row] * Select(Sample.Position);
						}
					}
					for (int32 Diagonal = 0; Diagonal < 6; ++Diagonal)
					{
						int32 Pivot = Diagonal;
						for (int32 Row = Diagonal + 1; Row < 6; ++Row)
							if (FMath::Abs(Matrix[Row][Diagonal]) >
								FMath::Abs(Matrix[Pivot][Diagonal])) Pivot = Row;
						if (Pivot != Diagonal)
							for (int32 Column = Diagonal; Column < 7; ++Column)
								Swap(Matrix[Diagonal][Column], Matrix[Pivot][Column]);
						if (FMath::Abs(Matrix[Diagonal][Diagonal]) <= 1.0e-12)
							return false;
						const double Inverse = 1.0 / Matrix[Diagonal][Diagonal];
						for (int32 Column = Diagonal; Column < 7; ++Column)
							Matrix[Diagonal][Column] *= Inverse;
						for (int32 Row = 0; Row < 6; ++Row)
						{
							if (Row == Diagonal) continue;
							const double Factor = Matrix[Row][Diagonal];
							for (int32 Column = Diagonal; Column < 7; ++Column)
								Matrix[Row][Column] -= Factor * Matrix[Diagonal][Column];
						}
					}
					for (int32 Index = 0; Index < 6; ++Index) Out[Index] = Matrix[Index][6];
					return true;
				};
				FOpenRimCanonicalTubeFit& Fit =
					OpenRimCanonicalTubeFits.AddDefaulted_GetRef();
				Fit.FitId = CombineStableIds(
					StableStringId(TEXT("OpenRim.CanonicalTube.LaneTerminal.V1")),
					CombineStableIds(static_cast<uint64>(OpeningSide + 2),
						static_cast<uint64>(TransverseSide + 2)));
				Fit.OpeningSide = static_cast<int8>(OpeningSide);
				Fit.TransverseSide = static_cast<int8>(TransverseSide);
				Fit.SampleCount = LaneSamples.Num();
				const bool bSolved =
					SolveLaneCoordinate([](const FVector3d& P) { return P.X; },
						Fit.TerminalDepthCoefficients) &&
					SolveLaneCoordinate([](const FVector3d& P) { return P.Y; },
					Fit.TerminalYCoefficients) &&
					SolveLaneCoordinate([](const FVector3d& P) { return P.Z; },
						Fit.TerminalZCoefficients);
				if (!bSolved) continue;
				// A natural cubic spline is C2 at every sampled rim station and
				// interpolates the lane-local terminal curve exactly.  It is stored
				// as ordinary cubic Bezier pieces so a later tensor tube can reuse
				// the same bounded parameterization without triangle promotion.
				const int32 SampleCount = LaneSamples.Num();
				TArray<FVector3d> SecondDerivatives;
				SecondDerivatives.SetNumZeroed(SampleCount);
				for (int32 Coordinate = 0; Coordinate < 3; ++Coordinate)
				{
					TArray<double> Lower, Diagonal, Upper, Right;
					Lower.SetNumZeroed(SampleCount);
					Diagonal.SetNumZeroed(SampleCount);
					Upper.SetNumZeroed(SampleCount);
					Right.SetNumZeroed(SampleCount);
					Diagonal[0] = 1.0;
					Diagonal[SampleCount - 1] = 1.0;
					auto Value = [&](const FLaneTerminalSample& Sample)
					{
						return Coordinate == 0 ? Sample.Position.X :
							Coordinate == 1 ? Sample.Position.Y : Sample.Position.Z;
					};
					for (int32 Index = 1; Index + 1 < SampleCount; ++Index)
					{
						const double PreviousH = LaneSamples[Index].S -
							LaneSamples[Index - 1].S;
						const double NextH = LaneSamples[Index + 1].S -
							LaneSamples[Index].S;
						Lower[Index] = PreviousH;
						Diagonal[Index] = 2.0 * (PreviousH + NextH);
						Upper[Index] = NextH;
						Right[Index] = 6.0 * ((Value(LaneSamples[Index + 1]) -
							Value(LaneSamples[Index])) / NextH -
							(Value(LaneSamples[Index]) - Value(LaneSamples[Index - 1])) /
							PreviousH);
					}
					for (int32 Index = 1; Index < SampleCount; ++Index)
					{
						const double Factor = Lower[Index] / Diagonal[Index - 1];
						Diagonal[Index] -= Factor * Upper[Index - 1];
						Right[Index] -= Factor * Right[Index - 1];
					}
					SecondDerivatives[SampleCount - 1][Coordinate] =
						Right[SampleCount - 1] / Diagonal[SampleCount - 1];
					for (int32 Index = SampleCount - 2; Index >= 0; --Index)
						SecondDerivatives[Index][Coordinate] = (Right[Index] -
							Upper[Index] * SecondDerivatives[Index + 1][Coordinate]) /
							Diagonal[Index];
				}
				for (int32 Index = 0; Index + 1 < SampleCount; ++Index)
				{
					const double H = LaneSamples[Index + 1].S - LaneSamples[Index].S;
					FOpenRimTubeTerminalSplineSegment& Segment =
						Fit.TerminalSplineSegments.AddDefaulted_GetRef();
					Segment.MinimumCanonicalRimParameter = LaneSamples[Index].S;
					Segment.MaximumCanonicalRimParameter = LaneSamples[Index + 1].S;
					Segment.ControlPoints[0] = LaneSamples[Index].Position;
					Segment.ControlPoints[3] = LaneSamples[Index + 1].Position;
					for (int32 Coordinate = 0; Coordinate < 3; ++Coordinate)
					{
						auto Value = [&](const FLaneTerminalSample& Sample)
						{
							return Coordinate == 0 ? Sample.Position.X :
								Coordinate == 1 ? Sample.Position.Y : Sample.Position.Z;
						};
						const double Delta = (Value(LaneSamples[Index + 1]) -
							Value(LaneSamples[Index])) / H;
						const double StartDerivative = Delta - H *
							(2.0 * SecondDerivatives[Index][Coordinate] +
								SecondDerivatives[Index + 1][Coordinate]) / 6.0;
						const double EndDerivative = Delta + H *
							(SecondDerivatives[Index][Coordinate] + 2.0 *
								SecondDerivatives[Index + 1][Coordinate]) / 6.0;
						Segment.ControlPoints[1][Coordinate] =
							Segment.ControlPoints[0][Coordinate] + H * StartDerivative / 3.0;
						Segment.ControlPoints[2][Coordinate] =
							Segment.ControlPoints[3][Coordinate] - H * EndDerivative / 3.0;
					}
				}
				Fit.bTerminalCurveC2 = !Fit.TerminalSplineSegments.IsEmpty();
				double SquaredSum = 0.0;
				for (const FLaneTerminalSample& Sample : LaneSamples)
				{
					const double Residual = FVector3d::Distance(
						Sample.Position, Fit.EvaluateTerminal(Sample.S));
					SquaredSum += Residual * Residual;
					if (Residual > Fit.MaximumResidualCm)
					{
						Fit.MaximumResidualCm = Residual;
						Fit.MaximumResidualSectionIndex = Sample.SectionIndex;
					}
				}
				Fit.RootMeanSquareResidualCm = FMath::Sqrt(
					SquaredSum / LaneSamples.Num());
				Fit.bRegularFiniteTube = true;
				for (int32 Sample = 0; Sample <= 64; ++Sample)
				{
					const FVector3d P = Fit.EvaluateTerminal(
						static_cast<double>(Sample) / 64.0);
					Fit.bRegularFiniteTube &= FMath::IsFinite(P.X) &&
						FMath::IsFinite(P.Y) && FMath::IsFinite(P.Z) && P.X > 0.0;
				}
			}
		}
	}

	// Preserve each tube lane's source correspondence with a compact natural
	// cubic spline before any cross-lane loft is attempted.  Unlike the witness
	// cells below, this is C2 along the longitudinal coordinate and contains one
	// interval per source station pair; it is still recognition-only.
	for (int32 SectionIndex = 0;
		SectionIndex < OpenRimTransverseSections.Num(); ++SectionIndex)
	{
		FOpenRimTransverseSection& Section =
			OpenRimTransverseSections[SectionIndex];
		Section.FirstLongitudinalSplineSegmentIndex = INDEX_NONE;
		Section.LongitudinalSplineSegmentCount = 0;
		Section.LongitudinalSplineRootMeanSquareResidualCm = 0.0;
		Section.LongitudinalSplineMaximumResidualCm = 0.0;
		Section.bLongitudinalSplineC2 = false;
		if (Section.LongitudinalTubePointCount < 2) continue;

		struct FTubeSplineSample
		{
			double V = 0.0;
			FVector3d Position = FVector3d::ZeroVector;
		};
		const int32 TerminalSampleIndex = Section.FirstLongitudinalSampleIndex +
			Section.LongitudinalTubePointCount - 1;
		if (!OpenRimLongitudinalSamples.IsValidIndex(
			Section.FirstLongitudinalSampleIndex) ||
			!OpenRimLongitudinalSamples.IsValidIndex(TerminalSampleIndex)) continue;
		const double TerminalArc = OpenRimLongitudinalSamples[TerminalSampleIndex]
			.NormalizedArcLength;
		if (TerminalArc <= 1.0e-9) continue;
		TArray<FTubeSplineSample, TInlineAllocator<24>> Samples;
		for (int32 Offset = 0; Offset < Section.LongitudinalTubePointCount; ++Offset)
		{
			const FOpenRimLongitudinalSample& Sample =
				OpenRimLongitudinalSamples[Section.FirstLongitudinalSampleIndex + Offset];
			const double V = FMath::Clamp(Sample.NormalizedArcLength / TerminalArc,
				0.0, 1.0);
			if (!Samples.IsEmpty() && FMath::IsNearlyEqual(Samples.Last().V, V, 1.0e-9))
			{
				Samples.Last().Position = 0.5 * (Samples.Last().Position +
					Sample.CanonicalPositionCm);
			}
			else
			{
				Samples.Add({ V, Sample.CanonicalPositionCm });
			}
		}
		if (Samples.Num() < 2) continue;
		const int32 SampleCount = Samples.Num();
		double SecondDerivatives[3][24] = {};
		check(SampleCount <= UE_ARRAY_COUNT(SecondDerivatives[0]));
		for (int32 Coordinate = 0; Coordinate < 3; ++Coordinate)
		{
			double Lower[24] = {}, Diagonal[24] = {}, Upper[24] = {}, Right[24] = {};
			Diagonal[0] = 1.0;
			Diagonal[SampleCount - 1] = 1.0;
			for (int32 Index = 1; Index + 1 < SampleCount; ++Index)
			{
				const double PreviousH = Samples[Index].V - Samples[Index - 1].V;
				const double NextH = Samples[Index + 1].V - Samples[Index].V;
				if (PreviousH <= 1.0e-12 || NextH <= 1.0e-12) continue;
				Lower[Index] = PreviousH;
				Diagonal[Index] = 2.0 * (PreviousH + NextH);
				Upper[Index] = NextH;
				Right[Index] = 6.0 * ((Samples[Index + 1].Position[Coordinate] -
					Samples[Index].Position[Coordinate]) / NextH -
					(Samples[Index].Position[Coordinate] -
						Samples[Index - 1].Position[Coordinate]) / PreviousH);
			}
			for (int32 Index = 1; Index < SampleCount; ++Index)
			{
				const double Factor = Lower[Index] / Diagonal[Index - 1];
				Diagonal[Index] -= Factor * Upper[Index - 1];
				Right[Index] -= Factor * Right[Index - 1];
			}
			SecondDerivatives[Coordinate][SampleCount - 1] =
				Right[SampleCount - 1] / Diagonal[SampleCount - 1];
			for (int32 Index = SampleCount - 2; Index >= 0; --Index)
				SecondDerivatives[Coordinate][Index] = (Right[Index] -
					Upper[Index] * SecondDerivatives[Coordinate][Index + 1]) /
					Diagonal[Index];
		}
		Section.FirstLongitudinalSplineSegmentIndex =
			OpenRimLongitudinalSplineSegments.Num();
		for (int32 Index = 0; Index + 1 < SampleCount; ++Index)
		{
			const double H = Samples[Index + 1].V - Samples[Index].V;
			FOpenRimLongitudinalSplineSegment& Segment =
				OpenRimLongitudinalSplineSegments.AddDefaulted_GetRef();
			Segment.OpenRimTransverseSectionIndex = SectionIndex;
			Segment.MinimumTubeParameter = Samples[Index].V;
			Segment.MaximumTubeParameter = Samples[Index + 1].V;
			Segment.ControlPoints[0] = Samples[Index].Position;
			Segment.ControlPoints[3] = Samples[Index + 1].Position;
			for (int32 Coordinate = 0; Coordinate < 3; ++Coordinate)
			{
				const double Delta = (Samples[Index + 1].Position[Coordinate] -
					Samples[Index].Position[Coordinate]) / H;
				const double StartDerivative = Delta - H *
					(2.0 * SecondDerivatives[Coordinate][Index] +
						SecondDerivatives[Coordinate][Index + 1]) / 6.0;
				const double EndDerivative = Delta + H *
					(SecondDerivatives[Coordinate][Index] + 2.0 *
						SecondDerivatives[Coordinate][Index + 1]) / 6.0;
				Segment.ControlPoints[1][Coordinate] = Segment.ControlPoints[0][Coordinate] +
					H * StartDerivative / 3.0;
				Segment.ControlPoints[2][Coordinate] = Segment.ControlPoints[3][Coordinate] -
					H * EndDerivative / 3.0;
			}
		}
		Section.LongitudinalSplineSegmentCount = SampleCount - 1;
		Section.bLongitudinalSplineC2 = Section.LongitudinalSplineSegmentCount > 0;
		for (const FTubeSplineSample& Sample : Samples)
		{
			int32 SegmentOffset = Section.LongitudinalSplineSegmentCount - 1;
			for (int32 CandidateOffset = 0;
				CandidateOffset < Section.LongitudinalSplineSegmentCount; ++CandidateOffset)
			{
				if (Sample.V <= OpenRimLongitudinalSplineSegments[
					Section.FirstLongitudinalSplineSegmentIndex + CandidateOffset]
					.MaximumTubeParameter + 1.0e-12)
				{
					SegmentOffset = CandidateOffset;
					break;
				}
			}
			const FOpenRimLongitudinalSplineSegment& Segment =
				OpenRimLongitudinalSplineSegments[
					Section.FirstLongitudinalSplineSegmentIndex + SegmentOffset];
			const double LocalV = FMath::Clamp((Sample.V - Segment.MinimumTubeParameter) /
				(Segment.MaximumTubeParameter - Segment.MinimumTubeParameter), 0.0, 1.0);
			const double Residual = FVector3d::Distance(Sample.Position,
				EvaluateBezierControlPolygon(Segment.ControlPoints, LocalV));
			Section.LongitudinalSplineRootMeanSquareResidualCm += Residual * Residual;
			Section.LongitudinalSplineMaximumResidualCm = FMath::Max(
				Section.LongitudinalSplineMaximumResidualCm, Residual);
		}
		Section.LongitudinalSplineRootMeanSquareResidualCm = FMath::Sqrt(
			Section.LongitudinalSplineRootMeanSquareResidualCm / Samples.Num());
	}

	// Sample the two independent C2 directions into one bounded experimental
	// tensor per physical lane. This is a compact residual-measurement candidate,
	// never a runtime provider: its finite-domain, lip and rear ownership gates
	// must be certified separately before materialization is even considered.
	for (const int32 OpeningSide : { -1, 1 })
	{
		for (const int32 TransverseSide : { -1, 1 })
		{
			TArray<const FOpenRimTransverseSection*> LaneSections;
			for (const FOpenRimTransverseSection& Section : OpenRimTransverseSections)
			{
				if (Section.bLongitudinalSplineC2 &&
					Section.bTopologyCanonicalRimParameterValid &&
					(Section.SliceOrigin.X >= 0.0 ? 1 : -1) == OpeningSide &&
					(Section.SliceOrigin.Y >= 0.0 ? 1 : -1) == TransverseSide)
				{
					LaneSections.Add(&Section);
				}
			}
			Algo::Sort(LaneSections, [](const FOpenRimTransverseSection* A,
				const FOpenRimTransverseSection* B)
			{
				return A->TopologyCanonicalRimParameter < B->TopologyCanonicalRimParameter;
			});
			for (int32 Index = LaneSections.Num() - 1; Index > 0; --Index)
			{
				if (FMath::IsNearlyEqual(LaneSections[Index]->TopologyCanonicalRimParameter,
					LaneSections[Index - 1]->TopologyCanonicalRimParameter, 1.0e-8))
				{
					LaneSections.RemoveAt(Index);
				}
			}
			if (LaneSections.Num() < 3) continue;
			auto EvaluateRail = [&](const FOpenRimTransverseSection& Section,
				const double Parameter, const bool bNormalizedDepth = false)
			{
				double V = Parameter;
				if (bNormalizedDepth)
				{
					const FOpenRimLongitudinalSplineSegment& FirstSegment =
						OpenRimLongitudinalSplineSegments[
							Section.FirstLongitudinalSplineSegmentIndex];
					const FOpenRimLongitudinalSplineSegment& LastSegment =
						OpenRimLongitudinalSplineSegments[
							Section.FirstLongitudinalSplineSegmentIndex +
							Section.LongitudinalSplineSegmentCount - 1];
					const double TargetDepth = FMath::Lerp(FirstSegment.ControlPoints[0].X,
						LastSegment.ControlPoints[3].X, Parameter);
					double MinimumV = 0.0;
					double MaximumV = 1.0;
					for (int32 Iteration = 0; Iteration < 32; ++Iteration)
					{
						const double Midpoint = 0.5 * (MinimumV + MaximumV);
						int32 MidSegmentOffset = Section.LongitudinalSplineSegmentCount - 1;
						for (int32 CandidateOffset = 0;
							CandidateOffset < Section.LongitudinalSplineSegmentCount;
							++CandidateOffset)
						{
							if (Midpoint <= OpenRimLongitudinalSplineSegments[
								Section.FirstLongitudinalSplineSegmentIndex + CandidateOffset]
								.MaximumTubeParameter + 1.0e-12)
							{
								MidSegmentOffset = CandidateOffset;
								break;
							}
						}
						const FOpenRimLongitudinalSplineSegment& MidSegment =
							OpenRimLongitudinalSplineSegments[
								Section.FirstLongitudinalSplineSegmentIndex + MidSegmentOffset];
						const double LocalMidpoint = (Midpoint - MidSegment.MinimumTubeParameter) /
							(MidSegment.MaximumTubeParameter - MidSegment.MinimumTubeParameter);
						if (EvaluateBezierControlPolygon(MidSegment.ControlPoints,
							LocalMidpoint).X < TargetDepth) MinimumV = Midpoint;
						else MaximumV = Midpoint;
					}
					V = 0.5 * (MinimumV + MaximumV);
				}
				int32 SegmentOffset = Section.LongitudinalSplineSegmentCount - 1;
				for (int32 CandidateOffset = 0;
					CandidateOffset < Section.LongitudinalSplineSegmentCount; ++CandidateOffset)
				{
					const FOpenRimLongitudinalSplineSegment& Candidate =
						OpenRimLongitudinalSplineSegments[
							Section.FirstLongitudinalSplineSegmentIndex + CandidateOffset];
					if (V <= Candidate.MaximumTubeParameter + 1.0e-12)
					{
						SegmentOffset = CandidateOffset;
						break;
					}
				}
				const FOpenRimLongitudinalSplineSegment& Segment =
					OpenRimLongitudinalSplineSegments[
						Section.FirstLongitudinalSplineSegmentIndex + SegmentOffset];
				const double LocalV = FMath::Clamp((V - Segment.MinimumTubeParameter) /
					(Segment.MaximumTubeParameter - Segment.MinimumTubeParameter), 0.0, 1.0);
				return EvaluateBezierControlPolygon(Segment.ControlPoints, LocalV);
			};
			auto EvaluateTransverseC2 = [&](const double S, const double V,
				const bool bNormalizedDepth = false)
			{
				const int32 Count = LaneSections.Num();
				check(Count <= 128);
				FVector3d Values[128];
				double SecondDerivatives[3][128] = {};
				for (int32 Index = 0; Index < Count; ++Index)
					Values[Index] = EvaluateRail(*LaneSections[Index], V,
						bNormalizedDepth);
				for (int32 Coordinate = 0; Coordinate < 3; ++Coordinate)
				{
					double Lower[128] = {}, Diagonal[128] = {}, Upper[128] = {}, Right[128] = {};
					Diagonal[0] = 1.0;
					Diagonal[Count - 1] = 1.0;
					for (int32 Index = 1; Index + 1 < Count; ++Index)
					{
						const double PreviousH = LaneSections[Index]->TopologyCanonicalRimParameter -
							LaneSections[Index - 1]->TopologyCanonicalRimParameter;
						const double NextH = LaneSections[Index + 1]->TopologyCanonicalRimParameter -
							LaneSections[Index]->TopologyCanonicalRimParameter;
						Lower[Index] = PreviousH;
						Diagonal[Index] = 2.0 * (PreviousH + NextH);
						Upper[Index] = NextH;
						Right[Index] = 6.0 * ((Values[Index + 1][Coordinate] -
							Values[Index][Coordinate]) / NextH -
							(Values[Index][Coordinate] - Values[Index - 1][Coordinate]) / PreviousH);
					}
					for (int32 Index = 1; Index < Count; ++Index)
					{
						const double Factor = Lower[Index] / Diagonal[Index - 1];
						Diagonal[Index] -= Factor * Upper[Index - 1];
						Right[Index] -= Factor * Right[Index - 1];
					}
					SecondDerivatives[Coordinate][Count - 1] =
						Right[Count - 1] / Diagonal[Count - 1];
					for (int32 Index = Count - 2; Index >= 0; --Index)
						SecondDerivatives[Coordinate][Index] = (Right[Index] -
							Upper[Index] * SecondDerivatives[Coordinate][Index + 1]) /
							Diagonal[Index];
				}
				int32 SegmentIndex = Count - 2;
				for (int32 Index = 0; Index + 1 < Count; ++Index)
				{
					if (S <= LaneSections[Index + 1]->TopologyCanonicalRimParameter + 1.0e-12)
					{
						SegmentIndex = Index;
						break;
					}
				}
				const double MinimumS = LaneSections[SegmentIndex]->TopologyCanonicalRimParameter;
				const double MaximumS = LaneSections[SegmentIndex + 1]->TopologyCanonicalRimParameter;
				const double H = MaximumS - MinimumS;
				const double LocalS = FMath::Clamp((S - MinimumS) / H, 0.0, 1.0);
				FVector3d Controls[4] = { Values[SegmentIndex], FVector3d::ZeroVector,
					FVector3d::ZeroVector, Values[SegmentIndex + 1] };
				for (int32 Coordinate = 0; Coordinate < 3; ++Coordinate)
				{
					const double Delta = (Values[SegmentIndex + 1][Coordinate] -
						Values[SegmentIndex][Coordinate]) / H;
					const double StartDerivative = Delta - H *
						(2.0 * SecondDerivatives[Coordinate][SegmentIndex] +
							SecondDerivatives[Coordinate][SegmentIndex + 1]) / 6.0;
					const double EndDerivative = Delta + H *
						(SecondDerivatives[Coordinate][SegmentIndex] + 2.0 *
							SecondDerivatives[Coordinate][SegmentIndex + 1]) / 6.0;
					Controls[1][Coordinate] = Controls[0][Coordinate] + H * StartDerivative / 3.0;
					Controls[2][Coordinate] = Controls[3][Coordinate] - H * EndDerivative / 3.0;
				}
				return EvaluateBezierControlPolygon(Controls, LocalS);
			};
			FOpenRimCanonicalTubeLoftFit& Loft =
				OpenRimCanonicalTubeLoftFits.AddDefaulted_GetRef();
			Loft.FitId = CombineStableIds(
				StableStringId(TEXT("OpenRim.CanonicalTube.CompactC2Shadow.V1")),
				CombineStableIds(static_cast<uint64>(OpeningSide + 2),
					static_cast<uint64>(TransverseSide + 2)));
			Loft.OpeningSide = static_cast<int8>(OpeningSide);
			Loft.TransverseSide = static_cast<int8>(TransverseSide);
			Loft.bLongitudinalC2Input = true;
			Loft.bTransverseC2Input = true;
			const double MinimumS = LaneSections[0]->TopologyCanonicalRimParameter;
			const double MaximumS = LaneSections.Last()->TopologyCanonicalRimParameter;
			BuildTensorSurfaceFromPolynomialEvaluator(13, 13,
				[&](const double U, const double V)
				{ return EvaluateTransverseC2(FMath::Lerp(MinimumS, MaximumS, U), V); },
				Loft.Surface, Loft.InterpolationMaximumErrorCm, false);
			for (const FOpenRimTransverseSection* Section : LaneSections)
			{
				const int32 TerminalSampleIndex = Section->FirstLongitudinalSampleIndex +
					Section->LongitudinalTubePointCount - 1;
				const double TerminalArc = OpenRimLongitudinalSamples[TerminalSampleIndex]
					.NormalizedArcLength;
				for (int32 Offset = 0; Offset < Section->LongitudinalTubePointCount; ++Offset)
				{
					const FOpenRimLongitudinalSample& Sample = OpenRimLongitudinalSamples[
						Section->FirstLongitudinalSampleIndex + Offset];
					const double U = (Section->TopologyCanonicalRimParameter - MinimumS) /
						(MaximumS - MinimumS);
					const double V = Sample.NormalizedArcLength / TerminalArc;
					const double Residual = FVector3d::Distance(Sample.CanonicalPositionCm,
						Loft.Surface.Evaluate(U, V));
					Loft.RootMeanSquareResidualCm += Residual * Residual;
					if (Residual > Loft.MaximumResidualCm)
					{
						Loft.MaximumResidualCm = Residual;
						Loft.MaximumResidualSectionIndex =
							static_cast<int32>(Section - OpenRimTransverseSections.GetData());
					}
					++Loft.SampleCount;
				}
			}
			Loft.RootMeanSquareResidualCm = FMath::Sqrt(
				Loft.RootMeanSquareResidualCm / FMath::Max(1, Loft.SampleCount));
			const uint64 GlobalFitId = Loft.FitId;
			FOpenRimCanonicalTubeLoftFit& LowDegreeLoft =
				OpenRimCanonicalTubeLoftFits.AddDefaulted_GetRef();
			LowDegreeLoft.FitId = CombineStableIds(
				StableStringId(TEXT("OpenRim.CanonicalTube.LowDegreeShadow.V1")),
				CombineStableIds(static_cast<uint64>(OpeningSide + 2),
					static_cast<uint64>(TransverseSide + 2)));
			LowDegreeLoft.OpeningSide = static_cast<int8>(OpeningSide);
			LowDegreeLoft.TransverseSide = static_cast<int8>(TransverseSide);
			LowDegreeLoft.bLongitudinalC2Input = true;
			LowDegreeLoft.bTransverseC2Input = true;
			LowDegreeLoft.bLowDegreeCandidate = true;
			BuildTensorSurfaceFromPolynomialEvaluator(5, 5,
				[&](const double U, const double V)
				{ return EvaluateTransverseC2(FMath::Lerp(MinimumS, MaximumS, U), V); },
				LowDegreeLoft.Surface, LowDegreeLoft.InterpolationMaximumErrorCm, false);
			for (const FOpenRimTransverseSection* Section : LaneSections)
			{
				const int32 TerminalSampleIndex = Section->FirstLongitudinalSampleIndex +
					Section->LongitudinalTubePointCount - 1;
				const double TerminalArc = OpenRimLongitudinalSamples[TerminalSampleIndex]
					.NormalizedArcLength;
				for (int32 Offset = 0; Offset < Section->LongitudinalTubePointCount; ++Offset)
				{
					const FOpenRimLongitudinalSample& Sample = OpenRimLongitudinalSamples[
						Section->FirstLongitudinalSampleIndex + Offset];
					const double U = (Section->TopologyCanonicalRimParameter - MinimumS) /
						(MaximumS - MinimumS);
					const double V = Sample.NormalizedArcLength / TerminalArc;
					const double Residual = FVector3d::Distance(Sample.CanonicalPositionCm,
						LowDegreeLoft.Surface.Evaluate(U, V));
					LowDegreeLoft.RootMeanSquareResidualCm += Residual * Residual;
					if (Residual > LowDegreeLoft.MaximumResidualCm)
					{
						LowDegreeLoft.MaximumResidualCm = Residual;
						LowDegreeLoft.MaximumResidualSectionIndex = static_cast<int32>(
							Section - OpenRimTransverseSections.GetData());
					}
					++LowDegreeLoft.SampleCount;
				}
			}
			LowDegreeLoft.RootMeanSquareResidualCm = FMath::Sqrt(
				LowDegreeLoft.RootMeanSquareResidualCm /
				FMath::Max(1, LowDegreeLoft.SampleCount));
			for (int32 TileU = 0; TileU < 4; ++TileU)
			{
				for (int32 TileV = 0; TileV < 4; ++TileV)
				{
					FOpenRimCanonicalTubeLoftFit& Tile =
						OpenRimCanonicalTubeLoftFits.AddDefaulted_GetRef();
					Tile.FitId = CombineStableIds(GlobalFitId,
						static_cast<uint64>(1 + TileU * 4 + TileV));
					Tile.OpeningSide = static_cast<int8>(OpeningSide);
					Tile.TransverseSide = static_cast<int8>(TransverseSide);
					Tile.TileIndex = TileU * 4 + TileV;
					Tile.MinimumCanonicalRimParameter = FMath::Lerp(MinimumS,
						MaximumS, static_cast<double>(TileU) / 4.0);
					Tile.MaximumCanonicalRimParameter = FMath::Lerp(MinimumS,
						MaximumS, static_cast<double>(TileU + 1) / 4.0);
					Tile.MinimumTubeParameter = static_cast<double>(TileV) / 4.0;
					Tile.MaximumTubeParameter = static_cast<double>(TileV + 1) / 4.0;
					Tile.bLongitudinalC2Input = true;
					Tile.bTransverseC2Input = true;
					BuildTensorSurfaceFromPolynomialEvaluator(5, 5,
						[&](const double U, const double V)
						{
							return EvaluateTransverseC2(FMath::Lerp(
								Tile.MinimumCanonicalRimParameter,
								Tile.MaximumCanonicalRimParameter, U), FMath::Lerp(
								Tile.MinimumTubeParameter, Tile.MaximumTubeParameter, V));
						}, Tile.Surface, Tile.InterpolationMaximumErrorCm, false);
					for (const FOpenRimTransverseSection* Section : LaneSections)
					{
						const double S = Section->TopologyCanonicalRimParameter;
						if (S < Tile.MinimumCanonicalRimParameter - 1.0e-9 ||
							S > Tile.MaximumCanonicalRimParameter + 1.0e-9) continue;
						const int32 TerminalSampleIndex =
							Section->FirstLongitudinalSampleIndex +
							Section->LongitudinalTubePointCount - 1;
						const double TerminalArc = OpenRimLongitudinalSamples[
							TerminalSampleIndex].NormalizedArcLength;
						for (int32 Offset = 0;
							Offset < Section->LongitudinalTubePointCount; ++Offset)
						{
							const FOpenRimLongitudinalSample& Sample =
								OpenRimLongitudinalSamples[
									Section->FirstLongitudinalSampleIndex + Offset];
							const double V = Sample.NormalizedArcLength / TerminalArc;
							if (V < Tile.MinimumTubeParameter - 1.0e-9 ||
								V > Tile.MaximumTubeParameter + 1.0e-9) continue;
							const double U = (S - Tile.MinimumCanonicalRimParameter) /
								(Tile.MaximumCanonicalRimParameter -
									Tile.MinimumCanonicalRimParameter);
							const double LocalV = (V - Tile.MinimumTubeParameter) /
								(Tile.MaximumTubeParameter - Tile.MinimumTubeParameter);
							const double Residual = FVector3d::Distance(
								Sample.CanonicalPositionCm, Tile.Surface.Evaluate(U, LocalV));
							Tile.RootMeanSquareResidualCm += Residual * Residual;
							if (Residual > Tile.MaximumResidualCm)
							{
								Tile.MaximumResidualCm = Residual;
								Tile.MaximumResidualSectionIndex = static_cast<int32>(
									Section - OpenRimTransverseSections.GetData());
							}
							++Tile.SampleCount;
						}
					}
					Tile.RootMeanSquareResidualCm = FMath::Sqrt(
						Tile.RootMeanSquareResidualCm / FMath::Max(1, Tile.SampleCount));
				}
			}

			// Build the compact candidate from a shared adaptive longitudinal knot
			// vector.  Each source rail is first approximated by a natural cubic on
			// those knots.  The worst real source witness inserts the next knot until
			// the lane-wide 0.01 cm certificate closes.  A natural cubic through the
			// resulting rail values in the transverse direction then forms a genuine
			// tensor-product C2 spline.  Every knot rectangle is therefore exactly
			// bicubic; converting rectangles independently does not create seams.
			// This is the representation certificate against the authored source,
			// not the 0.01 cm runtime contact/penetration shell.  Two and a half millimetres
			// keeps the fitted foundation materially inside the existing 10 cm source
			// selection gate while allowing the C2 spline to be genuinely compact.
			constexpr double CompactSourceToleranceCm = 0.25;
			TArray<double> CompactKnots = { 0.0, 1.0 };
			struct FCompactRail
			{
				const FOpenRimTransverseSection* Section = nullptr;
				TArray<FVector3d> SegmentControls;
			};
			TArray<FCompactRail> CompactRails;
			auto BuildCompactRails = [&]()
			{
				CompactRails.Reset();
				for (const FOpenRimTransverseSection* Section : LaneSections)
				{
					FCompactRail& Rail = CompactRails.AddDefaulted_GetRef();
					Rail.Section = Section;
					const int32 KnotCount = CompactKnots.Num();
					TArray<FVector3d> Values;
					Values.Reserve(KnotCount);
					for (const double Knot : CompactKnots)
						Values.Add(EvaluateRail(*Section, Knot));
					TArray<FVector3d> SecondDerivatives;
					SecondDerivatives.SetNumZeroed(KnotCount);
					for (int32 Coordinate = 0; Coordinate < 3; ++Coordinate)
					{
						TArray<double> Lower, Diagonal, Upper, Right;
						Lower.SetNumZeroed(KnotCount);
						Diagonal.SetNumZeroed(KnotCount);
						Upper.SetNumZeroed(KnotCount);
						Right.SetNumZeroed(KnotCount);
						Diagonal[0] = Diagonal[KnotCount - 1] = 1.0;
						for (int32 Index = 1; Index + 1 < KnotCount; ++Index)
						{
							const double PreviousH = CompactKnots[Index] - CompactKnots[Index - 1];
							const double NextH = CompactKnots[Index + 1] - CompactKnots[Index];
							Lower[Index] = PreviousH;
							Diagonal[Index] = 2.0 * (PreviousH + NextH);
							Upper[Index] = NextH;
							Right[Index] = 6.0 * ((Values[Index + 1][Coordinate] -
								Values[Index][Coordinate]) / NextH -
								(Values[Index][Coordinate] - Values[Index - 1][Coordinate]) /
								PreviousH);
						}
						for (int32 Index = 1; Index < KnotCount; ++Index)
						{
							const double Factor = Lower[Index] / Diagonal[Index - 1];
							Diagonal[Index] -= Factor * Upper[Index - 1];
							Right[Index] -= Factor * Right[Index - 1];
						}
						SecondDerivatives[KnotCount - 1][Coordinate] =
							Right[KnotCount - 1] / Diagonal[KnotCount - 1];
						for (int32 Index = KnotCount - 2; Index >= 0; --Index)
							SecondDerivatives[Index][Coordinate] = (Right[Index] -
								Upper[Index] * SecondDerivatives[Index + 1][Coordinate]) /
								Diagonal[Index];
					}
					Rail.SegmentControls.SetNumUninitialized((KnotCount - 1) * 4);
					for (int32 Index = 0; Index + 1 < KnotCount; ++Index)
					{
						const double H = CompactKnots[Index + 1] - CompactKnots[Index];
						FVector3d* Controls = Rail.SegmentControls.GetData() + Index * 4;
						Controls[0] = Values[Index];
						Controls[3] = Values[Index + 1];
						for (int32 Coordinate = 0; Coordinate < 3; ++Coordinate)
						{
							const double Delta = (Values[Index + 1][Coordinate] -
								Values[Index][Coordinate]) / H;
							const double StartDerivative = Delta - H *
								(2.0 * SecondDerivatives[Index][Coordinate] +
									SecondDerivatives[Index + 1][Coordinate]) / 6.0;
							const double EndDerivative = Delta + H *
								(SecondDerivatives[Index][Coordinate] + 2.0 *
									SecondDerivatives[Index + 1][Coordinate]) / 6.0;
							Controls[1][Coordinate] = Controls[0][Coordinate] +
								H * StartDerivative / 3.0;
							Controls[2][Coordinate] = Controls[3][Coordinate] -
								H * EndDerivative / 3.0;
						}
					}
				}
			};
			auto EvaluateCompactRail = [&](const FCompactRail& Rail, const double V)
			{
				int32 SegmentIndex = CompactKnots.Num() - 2;
				for (int32 Index = 0; Index + 1 < CompactKnots.Num(); ++Index)
				{
					if (V <= CompactKnots[Index + 1] + 1.0e-12)
					{
						SegmentIndex = Index;
						break;
					}
				}
				const double LocalV = FMath::Clamp((V - CompactKnots[SegmentIndex]) /
					(CompactKnots[SegmentIndex + 1] - CompactKnots[SegmentIndex]), 0.0, 1.0);
				return EvaluateBezierControlPolygon(MakeArrayView(
					Rail.SegmentControls.GetData() + SegmentIndex * 4, 4), LocalV);
			};
			double CompactMaximumSourceResidualCm = TNumericLimits<double>::Max();
			for (int32 Refinement = 0; Refinement < 256; ++Refinement)
			{
				BuildCompactRails();
				CompactMaximumSourceResidualCm = 0.0;
				double WorstV = 0.0;
				for (int32 RailIndex = 0; RailIndex < CompactRails.Num(); ++RailIndex)
				{
					const FOpenRimTransverseSection& Section = *CompactRails[RailIndex].Section;
					const int32 TerminalSampleIndex = Section.FirstLongitudinalSampleIndex +
						Section.LongitudinalTubePointCount - 1;
					const double TerminalArc = OpenRimLongitudinalSamples[TerminalSampleIndex]
						.NormalizedArcLength;
					for (int32 Offset = 0; Offset < Section.LongitudinalTubePointCount; ++Offset)
					{
						const FOpenRimLongitudinalSample& Sample = OpenRimLongitudinalSamples[
							Section.FirstLongitudinalSampleIndex + Offset];
						const double V = Sample.NormalizedArcLength / TerminalArc;
						const double Residual = FVector3d::Distance(Sample.CanonicalPositionCm,
							EvaluateCompactRail(CompactRails[RailIndex], V));
						if (Residual > CompactMaximumSourceResidualCm)
						{
							CompactMaximumSourceResidualCm = Residual;
							WorstV = V;
						}
					}
				}
				if (CompactMaximumSourceResidualCm <= CompactSourceToleranceCm) break;
				bool bDuplicate = false;
				for (const double Knot : CompactKnots)
					bDuplicate |= FMath::IsNearlyEqual(Knot, WorstV, 1.0e-10);
				if (bDuplicate) break;
				CompactKnots.Add(WorstV);
				Algo::Sort(CompactKnots);
			}
			const bool bCompactSourceCertified =
				CompactMaximumSourceResidualCm <= CompactSourceToleranceCm;
			if (bCompactSourceCertified)
			{
				auto EvaluateCompactTransverse = [&](const double S, const double V,
					const EOpenRimFeature Feature)
				{
					TArray<int32, TInlineAllocator<128>> RailIndices;
					for (int32 Index = 0; Index < LaneSections.Num(); ++Index)
						if (LaneSections[Index]->Feature == Feature)
							RailIndices.Add(Index);
					const int32 Count = RailIndices.Num();
					FVector3d Values[128];
					double SecondDerivatives[3][128] = {};
					check(Count >= 2 && Count <= 128);
					for (int32 Index = 0; Index < Count; ++Index)
						Values[Index] = EvaluateCompactRail(
							CompactRails[RailIndices[Index]], V);
					for (int32 Coordinate = 0; Coordinate < 3; ++Coordinate)
					{
						double Lower[128] = {}, Diagonal[128] = {}, Upper[128] = {}, Right[128] = {};
						Diagonal[0] = Diagonal[Count - 1] = 1.0;
						for (int32 Index = 1; Index + 1 < Count; ++Index)
						{
							const double PreviousH = LaneSections[RailIndices[Index]]->TopologyCanonicalRimParameter -
								LaneSections[RailIndices[Index - 1]]->TopologyCanonicalRimParameter;
							const double NextH = LaneSections[RailIndices[Index + 1]]->TopologyCanonicalRimParameter -
								LaneSections[RailIndices[Index]]->TopologyCanonicalRimParameter;
							Lower[Index] = PreviousH;
							Diagonal[Index] = 2.0 * (PreviousH + NextH);
							Upper[Index] = NextH;
							Right[Index] = 6.0 * ((Values[Index + 1][Coordinate] -
								Values[Index][Coordinate]) / NextH -
								(Values[Index][Coordinate] - Values[Index - 1][Coordinate]) /
								PreviousH);
						}
						for (int32 Index = 1; Index < Count; ++Index)
						{
							const double Factor = Lower[Index] / Diagonal[Index - 1];
							Diagonal[Index] -= Factor * Upper[Index - 1];
							Right[Index] -= Factor * Right[Index - 1];
						}
						SecondDerivatives[Coordinate][Count - 1] =
							Right[Count - 1] / Diagonal[Count - 1];
						for (int32 Index = Count - 2; Index >= 0; --Index)
							SecondDerivatives[Coordinate][Index] = (Right[Index] -
								Upper[Index] * SecondDerivatives[Coordinate][Index + 1]) /
								Diagonal[Index];
					}
					int32 SegmentIndex = Count - 2;
					for (int32 Index = 0; Index + 1 < Count; ++Index)
						if (S <= LaneSections[RailIndices[Index + 1]]->TopologyCanonicalRimParameter + 1.0e-12)
						{ SegmentIndex = Index; break; }
					const double MinimumLocalS =
						LaneSections[RailIndices[SegmentIndex]]->TopologyCanonicalRimParameter;
					const double MaximumLocalS =
						LaneSections[RailIndices[SegmentIndex + 1]]->TopologyCanonicalRimParameter;
					const double H = MaximumLocalS - MinimumLocalS;
					const double LocalS = FMath::Clamp((S - MinimumLocalS) / H, 0.0, 1.0);
					if (Feature == EOpenRimFeature::UpperTransition && Count >= 4)
					{
						auto EstimateKnotDerivatives = [&](const int32 KnotIndex,
							FVector3d& OutFirst, FVector3d& OutSecond)
						{
							const int32 FirstNode = FMath::Clamp(KnotIndex - 1, 0, Count - 4);
							const double TargetS = LaneSections[RailIndices[KnotIndex]]->
								TopologyCanonicalRimParameter;
							double Nodes[4];
							FVector3d NodeValues[4];
							for (int32 Node = 0; Node < 4; ++Node)
							{
								const int32 RailOffset = FirstNode + Node;
								Nodes[Node] = LaneSections[RailIndices[RailOffset]]->
									TopologyCanonicalRimParameter;
								NodeValues[Node] = Values[RailOffset];
							}
							OutFirst = OutSecond = FVector3d::ZeroVector;
							for (int32 Node = 0; Node < 4; ++Node)
							{
								double Denominator = 1.0;
								for (int32 Other = 0; Other < 4; ++Other)
									if (Other != Node)
										Denominator *= Nodes[Node] - Nodes[Other];
								double FirstWeight = 0.0;
								double SecondWeight = 0.0;
								for (int32 OmittedA = 0; OmittedA < 4; ++OmittedA)
								{
									if (OmittedA == Node) continue;
									double Product = 1.0;
									for (int32 Other = 0; Other < 4; ++Other)
										if (Other != Node && Other != OmittedA)
											Product *= TargetS - Nodes[Other];
									FirstWeight += Product;
									for (int32 OmittedB = OmittedA + 1; OmittedB < 4; ++OmittedB)
									{
										if (OmittedB == Node) continue;
										double SecondProduct = 1.0;
										for (int32 Other = 0; Other < 4; ++Other)
											if (Other != Node && Other != OmittedA && Other != OmittedB)
												SecondProduct *= TargetS - Nodes[Other];
										SecondWeight += 2.0 * SecondProduct;
									}
								}
								OutFirst += NodeValues[Node] * (FirstWeight / Denominator);
								OutSecond += NodeValues[Node] * (SecondWeight / Denominator);
							}
						};
						FVector3d StartDerivativeS, StartSecondS;
						FVector3d EndDerivativeS, EndSecondS;
						EstimateKnotDerivatives(SegmentIndex, StartDerivativeS, StartSecondS);
						EstimateKnotDerivatives(SegmentIndex + 1, EndDerivativeS, EndSecondS);
						auto EstimateNaturalKnotDerivatives = [&](const int32 KnotIndex,
							FVector3d& OutFirst, FVector3d& OutSecond)
						{
							OutSecond = FVector3d(SecondDerivatives[0][KnotIndex],
								SecondDerivatives[1][KnotIndex], SecondDerivatives[2][KnotIndex]);
							const int32 NaturalSegment = FMath::Min(KnotIndex, Count - 2);
							const double NaturalMinimumS = LaneSections[RailIndices[
								NaturalSegment]]->TopologyCanonicalRimParameter;
							const double NaturalMaximumS = LaneSections[RailIndices[
								NaturalSegment + 1]]->TopologyCanonicalRimParameter;
							const double NaturalH = NaturalMaximumS - NaturalMinimumS;
							const FVector3d Delta = (Values[NaturalSegment + 1] -
								Values[NaturalSegment]) / NaturalH;
							const FVector3d LowerSecond(SecondDerivatives[0][NaturalSegment],
								SecondDerivatives[1][NaturalSegment],
								SecondDerivatives[2][NaturalSegment]);
							const FVector3d UpperSecond(SecondDerivatives[0][NaturalSegment + 1],
								SecondDerivatives[1][NaturalSegment + 1],
								SecondDerivatives[2][NaturalSegment + 1]);
							OutFirst = KnotIndex == NaturalSegment
								? Delta - NaturalH * (2.0 * LowerSecond + UpperSecond) / 6.0
								: Delta + NaturalH * (LowerSecond + 2.0 * UpperSecond) / 6.0;
						};
						FVector3d NaturalStartFirst, NaturalStartSecond;
						FVector3d NaturalEndFirst, NaturalEndSecond;
						EstimateNaturalKnotDerivatives(SegmentIndex,
							NaturalStartFirst, NaturalStartSecond);
						EstimateNaturalKnotDerivatives(SegmentIndex + 1,
							NaturalEndFirst, NaturalEndSecond);
						StartDerivativeS = NaturalStartFirst - 2.0 *
							(StartDerivativeS - NaturalStartFirst);
						StartSecondS = NaturalStartSecond - 2.0 *
							(StartSecondS - NaturalStartSecond);
						EndDerivativeS = NaturalEndFirst - 2.0 *
							(EndDerivativeS - NaturalEndFirst);
						EndSecondS = NaturalEndSecond - 2.0 *
							(EndSecondS - NaturalEndSecond);
						const FVector3d StartDerivative = H * StartDerivativeS;
						const FVector3d StartSecond = H * H * StartSecondS;
						const FVector3d EndDerivative = H * EndDerivativeS;
						const FVector3d EndSecond = H * H * EndSecondS;
						const FVector3d CompactControls[6] = {
							Values[SegmentIndex],
							Values[SegmentIndex] + StartDerivative / 5.0,
							Values[SegmentIndex] + 2.0 * StartDerivative / 5.0 +
								StartSecond / 20.0,
							Values[SegmentIndex + 1] - 2.0 * EndDerivative / 5.0 +
								EndSecond / 20.0,
							Values[SegmentIndex + 1] - EndDerivative / 5.0,
							Values[SegmentIndex + 1] };
						return EvaluateBezierControlPolygon(CompactControls, LocalS);
					}
					FVector3d Controls[4] = { Values[SegmentIndex], FVector3d::ZeroVector,
						FVector3d::ZeroVector, Values[SegmentIndex + 1] };
					for (int32 Coordinate = 0; Coordinate < 3; ++Coordinate)
					{
						const double Delta = (Values[SegmentIndex + 1][Coordinate] -
							Values[SegmentIndex][Coordinate]) / H;
						const double StartDerivative = Delta - H *
							(2.0 * SecondDerivatives[Coordinate][SegmentIndex] +
								SecondDerivatives[Coordinate][SegmentIndex + 1]) / 6.0;
						const double EndDerivative = Delta + H *
							(SecondDerivatives[Coordinate][SegmentIndex] + 2.0 *
								SecondDerivatives[Coordinate][SegmentIndex + 1]) / 6.0;
						Controls[1][Coordinate] = Controls[0][Coordinate] +
							H * StartDerivative / 3.0;
						Controls[2][Coordinate] = Controls[3][Coordinate] -
							H * EndDerivative / 3.0;
					}
					return EvaluateBezierControlPolygon(Controls, LocalS);
				};
				int32 AdaptiveSegmentIndex = 0;
				for (int32 UIndex = 0; UIndex + 1 < LaneSections.Num(); ++UIndex)
				{
					if (LaneSections[UIndex]->Feature !=
						LaneSections[UIndex + 1]->Feature)
					{
						continue;
					}
					const double CellMinimumS =
						LaneSections[UIndex]->TopologyCanonicalRimParameter;
					const double CellMaximumS =
						LaneSections[UIndex + 1]->TopologyCanonicalRimParameter;
					for (int32 VIndex = 0; VIndex + 1 < CompactKnots.Num(); ++VIndex)
					{
						FOpenRimCanonicalTubeTensorSurface& Tensor =
							OpenRimCanonicalTubeTensorSurfaces.AddDefaulted_GetRef();
						Tensor.SourceFitId = CombineStableIds(GlobalFitId,
							StableStringId(TEXT("AdaptiveCompactC2.V1")));
						Tensor.OpeningSide = static_cast<int8>(OpeningSide);
						Tensor.TransverseSide = static_cast<int8>(TransverseSide);
						Tensor.SegmentIndex = AdaptiveSegmentIndex++;
						Tensor.MinimumCanonicalRimParameter = CellMinimumS;
						Tensor.MaximumCanonicalRimParameter = CellMaximumS;
						Tensor.MinimumTubeParameter = CompactKnots[VIndex];
						Tensor.MaximumTubeParameter = CompactKnots[VIndex + 1];
						const int32 DegreeU = LaneSections[UIndex]->Feature ==
							EOpenRimFeature::UpperTransition ? 5 : 3;
						const bool bConverted = BuildTensorSurfaceFromPolynomialEvaluator(DegreeU, 3,
							[&](const double U, const double V)
							{
								return EvaluateCompactTransverse(FMath::Lerp(CellMinimumS,
									CellMaximumS, U), FMath::Lerp(CompactKnots[VIndex],
									CompactKnots[VIndex + 1], V),
								LaneSections[UIndex]->Feature);
							}, Tensor.Surface, Tensor.MaximumConversionErrorCm);
						Tensor.bAdaptiveCompactC2 = bConverted;
						Tensor.bSourceResidualCertified = bConverted;
						Tensor.SourceMaximumResidualCm = CompactMaximumSourceResidualCm;
						if (!bConverted)
							OpenRimCanonicalTubeTensorSurfaces.Pop();
					}
				}
				// Preserve the post-split topology as a distinct terminal-closure rail.
				// Its first quintic interval carries the compact prefix position and
				// first/second derivatives into the natural-cubic rear evidence.  The
				// closure parameter is scaled by the measured remaining/prefix arc ratio,
				// so matching derivatives does not inject an arbitrary unit-speed change.
				TArray<double> LaneClosureParameterScales;
				for (const FCompactRail& Rail : CompactRails)
				{
					const FOpenRimTransverseSection& Section = *Rail.Section;
					if (Section.FirstLongitudinalRearTurnPointOffset == INDEX_NONE ||
						Section.LongitudinalTubePointCount < 2 ||
						Section.FirstLongitudinalRearTurnPointOffset >=
							Section.LongitudinalContinuationPointCount) continue;
					const FOpenRimLongitudinalSample& PrefixTerminal =
						OpenRimLongitudinalSamples[Section.FirstLongitudinalSampleIndex +
							Section.LongitudinalTubePointCount - 1];
					const FOpenRimLongitudinalSample& ClosureTerminal =
						OpenRimLongitudinalSamples[Section.FirstLongitudinalSampleIndex +
							Section.LongitudinalContinuationPointCount - 1];
					if (PrefixTerminal.NormalizedArcLength > 1.0e-9)
						LaneClosureParameterScales.Add(
							(ClosureTerminal.NormalizedArcLength -
								PrefixTerminal.NormalizedArcLength) /
							PrefixTerminal.NormalizedArcLength);
				}
				Algo::Sort(LaneClosureParameterScales);
				const double LaneClosureParameterScale =
					LaneClosureParameterScales.IsEmpty() ? 1.0 :
					LaneClosureParameterScales[LaneClosureParameterScales.Num() / 2];
				const int32 FirstLaneClosureFitIndex =
					OpenRimTerminalClosureRailFits.Num();
				for (int32 RailIndex = 0; RailIndex < CompactRails.Num(); ++RailIndex)
				{
					const FOpenRimTransverseSection& Section =
						*CompactRails[RailIndex].Section;
					if (Section.FirstLongitudinalRearTurnPointOffset == INDEX_NONE ||
						Section.LongitudinalTubePointCount < 2 ||
						Section.FirstLongitudinalRearTurnPointOffset >=
							Section.LongitudinalContinuationPointCount) continue;
					struct FClosureSample
					{
						double Parameter = 0.0;
						FVector3d Position = FVector3d::ZeroVector;
					};
					TArray<FClosureSample> Samples;
					const int32 PrefixTerminalOffset =
						Section.LongitudinalTubePointCount - 1;
					const FOpenRimLongitudinalSample& PrefixTerminal =
						OpenRimLongitudinalSamples[
							Section.FirstLongitudinalSampleIndex + PrefixTerminalOffset];
					const FOpenRimLongitudinalSample& ClosureTerminal =
						OpenRimLongitudinalSamples[
							Section.FirstLongitudinalSampleIndex +
							Section.LongitudinalContinuationPointCount - 1];
					const double RemainingNormalizedArc =
						ClosureTerminal.NormalizedArcLength - PrefixTerminal.NormalizedArcLength;
					if (PrefixTerminal.NormalizedArcLength <= 1.0e-9 ||
						RemainingNormalizedArc <= 1.0e-9) continue;
					Samples.Add({ 0.0, PrefixTerminal.CanonicalPositionCm });
					for (int32 Offset = Section.FirstLongitudinalRearTurnPointOffset;
						Offset < Section.LongitudinalContinuationPointCount; ++Offset)
					{
						const FOpenRimLongitudinalSample& Source =
							OpenRimLongitudinalSamples[
								Section.FirstLongitudinalSampleIndex + Offset];
						const double Parameter = FMath::Clamp(
							(Source.NormalizedArcLength - PrefixTerminal.NormalizedArcLength) /
							RemainingNormalizedArc, 0.0, 1.0);
						if (!Samples.IsEmpty() && FMath::IsNearlyEqual(
							Samples.Last().Parameter, Parameter, 1.0e-10))
						{
							Samples.Last().Position = 0.5 * (Samples.Last().Position +
								Source.CanonicalPositionCm);
						}
						else Samples.Add({ Parameter, Source.CanonicalPositionCm });
					}
					if (Samples.Num() < 2 || Samples[1].Parameter <= 1.0e-9) continue;
					const int32 Count = Samples.Num();
					TArray<FVector3d> SecondDerivatives;
					SecondDerivatives.SetNumZeroed(Count);
					for (int32 Coordinate = 0; Coordinate < 3; ++Coordinate)
					{
						TArray<double> Lower, Diagonal, Upper, Right;
						Lower.SetNumZeroed(Count);
						Diagonal.SetNumZeroed(Count);
						Upper.SetNumZeroed(Count);
						Right.SetNumZeroed(Count);
						Diagonal[0] = Diagonal[Count - 1] = 1.0;
						for (int32 Index = 1; Index + 1 < Count; ++Index)
						{
							const double PreviousH = Samples[Index].Parameter -
								Samples[Index - 1].Parameter;
							const double NextH = Samples[Index + 1].Parameter -
								Samples[Index].Parameter;
							Lower[Index] = PreviousH;
							Diagonal[Index] = 2.0 * (PreviousH + NextH);
							Upper[Index] = NextH;
							Right[Index] = 6.0 * ((Samples[Index + 1].Position[Coordinate] -
								Samples[Index].Position[Coordinate]) / NextH -
								(Samples[Index].Position[Coordinate] -
									Samples[Index - 1].Position[Coordinate]) / PreviousH);
						}
						for (int32 Index = 1; Index < Count; ++Index)
						{
							const double Factor = Lower[Index] / Diagonal[Index - 1];
							Diagonal[Index] -= Factor * Upper[Index - 1];
							Right[Index] -= Factor * Right[Index - 1];
						}
						SecondDerivatives[Count - 1][Coordinate] =
							Right[Count - 1] / Diagonal[Count - 1];
						for (int32 Index = Count - 2; Index >= 0; --Index)
							SecondDerivatives[Index][Coordinate] = (Right[Index] -
								Upper[Index] * SecondDerivatives[Index + 1][Coordinate]) /
								Diagonal[Index];
					}
					TArray<FVector3d> NaturalControls;
					NaturalControls.SetNumUninitialized((Count - 1) * 4);
					for (int32 Index = 0; Index + 1 < Count; ++Index)
					{
						const double H = Samples[Index + 1].Parameter -
							Samples[Index].Parameter;
						FVector3d* Controls = NaturalControls.GetData() + Index * 4;
						Controls[0] = Samples[Index].Position;
						Controls[3] = Samples[Index + 1].Position;
						for (int32 Coordinate = 0; Coordinate < 3; ++Coordinate)
						{
							const double Delta = (Samples[Index + 1].Position[Coordinate] -
								Samples[Index].Position[Coordinate]) / H;
							const double StartDerivative = Delta - H *
								(2.0 * SecondDerivatives[Index][Coordinate] +
									SecondDerivatives[Index + 1][Coordinate]) / 6.0;
							const double EndDerivative = Delta + H *
								(SecondDerivatives[Index][Coordinate] + 2.0 *
									SecondDerivatives[Index + 1][Coordinate]) / 6.0;
							Controls[1][Coordinate] = Controls[0][Coordinate] +
								H * StartDerivative / 3.0;
							Controls[2][Coordinate] = Controls[3][Coordinate] -
								H * EndDerivative / 3.0;
						}
					}
					const int32 LastPrefixSegment = CompactKnots.Num() - 2;
					const double PrefixH = CompactKnots.Last() -
						CompactKnots[LastPrefixSegment];
					const FVector3d* PrefixControls =
						CompactRails[RailIndex].SegmentControls.GetData() +
						LastPrefixSegment * 4;
					const FVector3d PrefixFirst = 3.0 *
						(PrefixControls[3] - PrefixControls[2]) / PrefixH;
					const FVector3d PrefixSecond = 6.0 *
						(PrefixControls[3] - 2.0 * PrefixControls[2] +
							PrefixControls[1]) / (PrefixH * PrefixH);
					const double ParameterScale = LaneClosureParameterScale;
					const FVector3d ClosureStartFirst = ParameterScale * PrefixFirst;
					const FVector3d ClosureStartSecond =
						ParameterScale * ParameterScale * PrefixSecond;
					const double FirstH = Samples[1].Parameter;
					const FVector3d* FirstNatural = NaturalControls.GetData();
					const FVector3d ClosureEndFirst = 3.0 *
						(FirstNatural[3] - FirstNatural[2]) / FirstH;
					const FVector3d ClosureEndSecond = 6.0 *
						(FirstNatural[3] - 2.0 * FirstNatural[2] +
							FirstNatural[1]) / (FirstH * FirstH);
					FOpenRimTerminalClosureRailFit& Fit =
						OpenRimTerminalClosureRailFits.AddDefaulted_GetRef();
					Fit.OpenRimTransverseSectionIndex = static_cast<int32>(
						CompactRails[RailIndex].Section -
						OpenRimTransverseSections.GetData());
					Fit.FirstSegmentIndex = OpenRimTerminalClosureRailSegments.Num();
					Fit.SegmentCount = Count - 1;
					Fit.SourceSampleCount = Count;
					Fit.ClosureToPrefixParameterScale = ParameterScale;
					FOpenRimTerminalClosureRailSegment& FirstSegment =
						OpenRimTerminalClosureRailSegments.AddDefaulted_GetRef();
					FirstSegment.OpenRimTransverseSectionIndex =
						Fit.OpenRimTransverseSectionIndex;
					FirstSegment.MinimumClosureParameter = 0.0;
					FirstSegment.MaximumClosureParameter = FirstH;
					FirstSegment.Degree = 5;
					FirstSegment.ControlPoints[0] = Samples[0].Position;
					FirstSegment.ControlPoints[1] = Samples[0].Position +
						FirstH * ClosureStartFirst / 5.0;
					FirstSegment.ControlPoints[2] = Samples[0].Position +
						2.0 * FirstH * ClosureStartFirst / 5.0 +
						FirstH * FirstH * ClosureStartSecond / 20.0;
					FirstSegment.ControlPoints[5] = Samples[1].Position;
					FirstSegment.ControlPoints[4] = Samples[1].Position -
						FirstH * ClosureEndFirst / 5.0;
					FirstSegment.ControlPoints[3] = Samples[1].Position -
						2.0 * FirstH * ClosureEndFirst / 5.0 +
						FirstH * FirstH * ClosureEndSecond / 20.0;
					for (int32 Index = 1; Index + 1 < Count; ++Index)
					{
						FOpenRimTerminalClosureRailSegment& Segment =
							OpenRimTerminalClosureRailSegments.AddDefaulted_GetRef();
						Segment.OpenRimTransverseSectionIndex =
							Fit.OpenRimTransverseSectionIndex;
						Segment.MinimumClosureParameter = Samples[Index].Parameter;
						Segment.MaximumClosureParameter = Samples[Index + 1].Parameter;
						Segment.Degree = 3;
						for (int32 Control = 0; Control < 4; ++Control)
							Segment.ControlPoints[Control] =
								NaturalControls[Index * 4 + Control];
					}
					auto EvaluateClosure = [&](const double Parameter)
					{
						int32 SegmentOffset = Fit.SegmentCount - 1;
						for (int32 Offset = 0; Offset < Fit.SegmentCount; ++Offset)
						{
							const FOpenRimTerminalClosureRailSegment& Candidate =
								OpenRimTerminalClosureRailSegments[
									Fit.FirstSegmentIndex + Offset];
							if (Parameter <= Candidate.MaximumClosureParameter + 1.0e-12)
							{ SegmentOffset = Offset; break; }
						}
						const FOpenRimTerminalClosureRailSegment& Segment =
							OpenRimTerminalClosureRailSegments[
								Fit.FirstSegmentIndex + SegmentOffset];
						const double Local = FMath::Clamp((Parameter -
							Segment.MinimumClosureParameter) /
							(Segment.MaximumClosureParameter -
								Segment.MinimumClosureParameter), 0.0, 1.0);
						return EvaluateBezierControlPolygon(MakeArrayView(
							Segment.ControlPoints, Segment.Degree + 1), Local);
					};
					Fit.MaximumSourceResidualCm = 0.0;
					for (const FClosureSample& Sample : Samples)
						Fit.MaximumSourceResidualCm = FMath::Max(
							Fit.MaximumSourceResidualCm, FVector3d::Distance(
								Sample.Position, EvaluateClosure(Sample.Parameter)));
					Fit.MaximumPolylineResidualCm = 0.0;
					Fit.bRegularFinite = true;
					for (int32 SegmentOffset = 0;
						SegmentOffset < Fit.SegmentCount; ++SegmentOffset)
					{
						const FOpenRimTerminalClosureRailSegment& Segment =
							OpenRimTerminalClosureRailSegments[
								Fit.FirstSegmentIndex + SegmentOffset];
						for (int32 Step = 0; Step <= 16; ++Step)
						{
							const FVector3d Point = EvaluateBezierControlPolygon(
								MakeArrayView(Segment.ControlPoints, Segment.Degree + 1),
								static_cast<double>(Step) / 16.0);
							Fit.bRegularFinite &= IsFiniteVector(Point);
							double BestSquared = TNumericLimits<double>::Max();
							for (int32 SourceIndex = 0;
								SourceIndex + 1 < Samples.Num(); ++SourceIndex)
							{
								const FVector3d Delta = Samples[SourceIndex + 1].Position -
									Samples[SourceIndex].Position;
								const double Alpha = FMath::Clamp(FVector3d::DotProduct(
									Point - Samples[SourceIndex].Position, Delta) /
									FMath::Max(UE_DOUBLE_SMALL_NUMBER,
										Delta.SquaredLength()), 0.0, 1.0);
								BestSquared = FMath::Min(BestSquared,
									(Point - (Samples[SourceIndex].Position +
										Alpha * Delta)).SquaredLength());
							}
							Fit.MaximumPolylineResidualCm = FMath::Max(
								Fit.MaximumPolylineResidualCm, FMath::Sqrt(BestSquared));
						}
					}
					Fit.PositionJoinResidualCm = FVector3d::Distance(
						FirstSegment.ControlPoints[0], PrefixControls[3]);
					Fit.FirstDerivativeJoinResidualCm = FVector3d::Distance(
						5.0 * (FirstSegment.ControlPoints[1] -
							FirstSegment.ControlPoints[0]) /
							(FirstH * ParameterScale), PrefixFirst);
					Fit.SecondDerivativeJoinResidualCm = FVector3d::Distance(
						20.0 * (FirstSegment.ControlPoints[2] - 2.0 *
							FirstSegment.ControlPoints[1] + FirstSegment.ControlPoints[0]) /
							(FirstH * FirstH * ParameterScale * ParameterScale),
						PrefixSecond);
					Fit.bC2PrefixJoinByConstruction =
						Fit.PositionJoinResidualCm <= 1.0e-6 &&
						Fit.FirstDerivativeJoinResidualCm <= 1.0e-5 &&
						Fit.SecondDerivativeJoinResidualCm <= 1.0e-3;
				}

				const int32 LaneClosureFitCount =
					OpenRimTerminalClosureRailFits.Num() - FirstLaneClosureFitIndex;
				if (LaneClosureFitCount == LaneSections.Num())
				{
					struct FCompactClosureRail
					{
						const FOpenRimTerminalClosureRailFit* SourceFit = nullptr;
						TArray<int32> Degrees;
						TArray<FVector3d> Controls;
					};
					TArray<double> ClosureKnots = { 0.0, 1.0 };
					TArray<FCompactClosureRail> ClosureRails;
					auto EvaluateDetailedClosure = [&](const FOpenRimTerminalClosureRailFit& Fit,
						const double Parameter)
					{
						int32 SegmentOffset = Fit.SegmentCount - 1;
						for (int32 Offset = 0; Offset < Fit.SegmentCount; ++Offset)
						{
							const FOpenRimTerminalClosureRailSegment& Candidate =
								OpenRimTerminalClosureRailSegments[
									Fit.FirstSegmentIndex + Offset];
							if (Parameter <= Candidate.MaximumClosureParameter + 1.0e-12)
							{ SegmentOffset = Offset; break; }
						}
						const FOpenRimTerminalClosureRailSegment& Segment =
							OpenRimTerminalClosureRailSegments[
								Fit.FirstSegmentIndex + SegmentOffset];
						const double Local = FMath::Clamp((Parameter -
							Segment.MinimumClosureParameter) /
							(Segment.MaximumClosureParameter -
								Segment.MinimumClosureParameter), 0.0, 1.0);
						return EvaluateBezierControlPolygon(MakeArrayView(
							Segment.ControlPoints, Segment.Degree + 1), Local);
					};
					auto BuildCompactClosureRails = [&]()
					{
						ClosureRails.Reset();
						for (int32 FitOffset = 0; FitOffset < LaneClosureFitCount; ++FitOffset)
						{
							const FOpenRimTerminalClosureRailFit& SourceFit =
								OpenRimTerminalClosureRailFits[
									FirstLaneClosureFitIndex + FitOffset];
							FCompactClosureRail& Rail =
								ClosureRails.AddDefaulted_GetRef();
							Rail.SourceFit = &SourceFit;
							const int32 KnotCount = ClosureKnots.Num();
							TArray<FVector3d> Values;
							Values.Reserve(KnotCount);
							for (const double Knot : ClosureKnots)
								Values.Add(EvaluateDetailedClosure(SourceFit, Knot));
							TArray<FVector3d> SecondDerivatives;
							SecondDerivatives.SetNumZeroed(KnotCount);
							for (int32 Coordinate = 0; Coordinate < 3; ++Coordinate)
							{
								TArray<double> Lower, Diagonal, Upper, Right;
								Lower.SetNumZeroed(KnotCount);
								Diagonal.SetNumZeroed(KnotCount);
								Upper.SetNumZeroed(KnotCount);
								Right.SetNumZeroed(KnotCount);
								Diagonal[0] = Diagonal[KnotCount - 1] = 1.0;
								for (int32 Index = 1; Index + 1 < KnotCount; ++Index)
								{
									const double PreviousH = ClosureKnots[Index] -
										ClosureKnots[Index - 1];
									const double NextH = ClosureKnots[Index + 1] -
										ClosureKnots[Index];
									Lower[Index] = PreviousH;
									Diagonal[Index] = 2.0 * (PreviousH + NextH);
									Upper[Index] = NextH;
									Right[Index] = 6.0 * ((Values[Index + 1][Coordinate] -
										Values[Index][Coordinate]) / NextH -
										(Values[Index][Coordinate] -
											Values[Index - 1][Coordinate]) / PreviousH);
								}
								for (int32 Index = 1; Index < KnotCount; ++Index)
								{
									const double Factor = Lower[Index] / Diagonal[Index - 1];
									Diagonal[Index] -= Factor * Upper[Index - 1];
									Right[Index] -= Factor * Right[Index - 1];
								}
								SecondDerivatives[KnotCount - 1][Coordinate] =
									Right[KnotCount - 1] / Diagonal[KnotCount - 1];
								for (int32 Index = KnotCount - 2; Index >= 0; --Index)
									SecondDerivatives[Index][Coordinate] = (Right[Index] -
										Upper[Index] * SecondDerivatives[Index + 1][Coordinate]) /
										Diagonal[Index];
							}
							TArray<FVector3d> NaturalControls;
							NaturalControls.SetNumUninitialized((KnotCount - 1) * 4);
							for (int32 Index = 0; Index + 1 < KnotCount; ++Index)
							{
								const double H = ClosureKnots[Index + 1] - ClosureKnots[Index];
								FVector3d* Controls = NaturalControls.GetData() + Index * 4;
								Controls[0] = Values[Index];
								Controls[3] = Values[Index + 1];
								for (int32 Coordinate = 0; Coordinate < 3; ++Coordinate)
								{
									const double Delta = (Values[Index + 1][Coordinate] -
										Values[Index][Coordinate]) / H;
									const double StartDerivative = Delta - H *
										(2.0 * SecondDerivatives[Index][Coordinate] +
											SecondDerivatives[Index + 1][Coordinate]) / 6.0;
									const double EndDerivative = Delta + H *
										(SecondDerivatives[Index][Coordinate] + 2.0 *
											SecondDerivatives[Index + 1][Coordinate]) / 6.0;
									Controls[1][Coordinate] = Controls[0][Coordinate] +
										H * StartDerivative / 3.0;
									Controls[2][Coordinate] = Controls[3][Coordinate] -
										H * EndDerivative / 3.0;
								}
							}
							const FOpenRimTerminalClosureRailSegment& DetailedFirst =
								OpenRimTerminalClosureRailSegments[SourceFit.FirstSegmentIndex];
							const double DetailedH = DetailedFirst.MaximumClosureParameter;
							const FVector3d StartFirst = 5.0 *
								(DetailedFirst.ControlPoints[1] -
									DetailedFirst.ControlPoints[0]) / DetailedH;
							const FVector3d StartSecond = 20.0 *
								(DetailedFirst.ControlPoints[2] - 2.0 *
									DetailedFirst.ControlPoints[1] +
									DetailedFirst.ControlPoints[0]) / (DetailedH * DetailedH);
							const double FirstH = ClosureKnots[1];
							const FVector3d* FirstNatural = NaturalControls.GetData();
							const FVector3d EndFirst = 3.0 *
								(FirstNatural[3] - FirstNatural[2]) / FirstH;
							const FVector3d EndSecond = 6.0 *
								(FirstNatural[3] - 2.0 * FirstNatural[2] +
									FirstNatural[1]) / (FirstH * FirstH);
							Rail.Degrees.SetNumUninitialized(KnotCount - 1);
							Rail.Controls.SetNumZeroed((KnotCount - 1) * 6);
							Rail.Degrees[0] = 5;
							FVector3d* First = Rail.Controls.GetData();
							First[0] = Values[0];
							First[1] = Values[0] + FirstH * StartFirst / 5.0;
							First[2] = Values[0] + 2.0 * FirstH * StartFirst / 5.0 +
								FirstH * FirstH * StartSecond / 20.0;
							First[5] = Values[1];
							First[4] = Values[1] - FirstH * EndFirst / 5.0;
							First[3] = Values[1] - 2.0 * FirstH * EndFirst / 5.0 +
								FirstH * FirstH * EndSecond / 20.0;
							for (int32 Index = 1; Index + 1 < KnotCount; ++Index)
							{
								Rail.Degrees[Index] = 3;
								for (int32 Control = 0; Control < 4; ++Control)
									Rail.Controls[Index * 6 + Control] =
										NaturalControls[Index * 4 + Control];
							}
						}
					};
					auto EvaluateCompactClosure = [&](const FCompactClosureRail& Rail,
						const double Parameter)
					{
						int32 SegmentIndex = ClosureKnots.Num() - 2;
						for (int32 Index = 0; Index + 1 < ClosureKnots.Num(); ++Index)
							if (Parameter <= ClosureKnots[Index + 1] + 1.0e-12)
							{ SegmentIndex = Index; break; }
						const double Local = FMath::Clamp((Parameter -
							ClosureKnots[SegmentIndex]) /
							(ClosureKnots[SegmentIndex + 1] - ClosureKnots[SegmentIndex]),
							0.0, 1.0);
						return EvaluateBezierControlPolygon(MakeArrayView(
							Rail.Controls.GetData() + SegmentIndex * 6,
							Rail.Degrees[SegmentIndex] + 1), Local);
					};
					constexpr double CompactClosureSourceToleranceCm = 0.25;
					double CompactClosureMaximumSourceResidualCm =
						TNumericLimits<double>::Max();
					for (int32 Refinement = 0; Refinement < 256; ++Refinement)
					{
						BuildCompactClosureRails();
						CompactClosureMaximumSourceResidualCm = 0.0;
						double WorstParameter = 0.0;
						for (const FCompactClosureRail& Rail : ClosureRails)
						{
							for (int32 SegmentOffset = 0;
								SegmentOffset < Rail.SourceFit->SegmentCount; ++SegmentOffset)
							{
								const FOpenRimTerminalClosureRailSegment& SourceSegment =
									OpenRimTerminalClosureRailSegments[
										Rail.SourceFit->FirstSegmentIndex + SegmentOffset];
								const double Parameter =
									SourceSegment.MaximumClosureParameter;
								const double Residual = FVector3d::Distance(
									EvaluateDetailedClosure(*Rail.SourceFit, Parameter),
									EvaluateCompactClosure(Rail, Parameter));
								if (Residual > CompactClosureMaximumSourceResidualCm)
								{
									CompactClosureMaximumSourceResidualCm = Residual;
									WorstParameter = Parameter;
								}
							}
						}
						if (CompactClosureMaximumSourceResidualCm <=
							CompactClosureSourceToleranceCm) break;
						bool bDuplicate = false;
						for (const double Knot : ClosureKnots)
							bDuplicate |= FMath::IsNearlyEqual(Knot, WorstParameter, 1.0e-10);
						if (bDuplicate) break;
						ClosureKnots.Add(WorstParameter);
						Algo::Sort(ClosureKnots);
					}
					if (CompactClosureMaximumSourceResidualCm <=
						CompactClosureSourceToleranceCm)
					{
						struct FClosureTransverseSplineCache
						{
							TArray<int32> RailIndices;
							TArray<FVector3d> Values;
							TArray<FVector3d> SecondDerivatives;
						};
						TMap<int32, TMap<int64, FClosureTransverseSplineCache>> TransverseSplineCaches;
							auto EvaluateClosureTransverse = [&](const double S, const double V,
							const EOpenRimFeature Feature)
						{
							const int32 FeatureKey = static_cast<int32>(Feature);
							const int64 VKey = FMath::RoundToInt64(V * 1.0e12);
							TMap<int64, FClosureTransverseSplineCache>& FeatureCaches =
								TransverseSplineCaches.FindOrAdd(FeatureKey);
							FClosureTransverseSplineCache* Spline = FeatureCaches.Find(VKey);
							if (!Spline)
							{
								FClosureTransverseSplineCache& NewSpline = FeatureCaches.Add(VKey);
								for (int32 Index = 0; Index < LaneSections.Num(); ++Index)
									if (LaneSections[Index]->Feature == Feature)
										NewSpline.RailIndices.Add(Index);
								const int32 NewCount = NewSpline.RailIndices.Num();
								check(NewCount >= 2 && NewCount <= 128);
								NewSpline.Values.SetNumUninitialized(NewCount);
								NewSpline.SecondDerivatives.SetNumZeroed(NewCount);
								for (int32 Index = 0; Index < NewCount; ++Index)
									NewSpline.Values[Index] = EvaluateCompactClosure(
										ClosureRails[NewSpline.RailIndices[Index]], V);
								for (int32 Coordinate = 0; Coordinate < 3; ++Coordinate)
								{
									double Lower[128] = {}, Diagonal[128] = {};
									double Upper[128] = {}, Right[128] = {};
									Diagonal[0] = Diagonal[NewCount - 1] = 1.0;
									for (int32 Index = 1; Index + 1 < NewCount; ++Index)
									{
										const double PreviousH = LaneSections[NewSpline.RailIndices[Index]]->
											TopologyCanonicalRimParameter - LaneSections[
												NewSpline.RailIndices[Index - 1]]->TopologyCanonicalRimParameter;
										const double NextH = LaneSections[NewSpline.RailIndices[Index + 1]]->
											TopologyCanonicalRimParameter - LaneSections[
												NewSpline.RailIndices[Index]]->TopologyCanonicalRimParameter;
										Lower[Index] = PreviousH;
										Diagonal[Index] = 2.0 * (PreviousH + NextH);
										Upper[Index] = NextH;
										Right[Index] = 6.0 * ((NewSpline.Values[Index + 1][Coordinate] -
											NewSpline.Values[Index][Coordinate]) / NextH -
											(NewSpline.Values[Index][Coordinate] -
												NewSpline.Values[Index - 1][Coordinate]) / PreviousH);
									}
									for (int32 Index = 1; Index < NewCount; ++Index)
									{
										const double Factor = Lower[Index] / Diagonal[Index - 1];
										Diagonal[Index] -= Factor * Upper[Index - 1];
										Right[Index] -= Factor * Right[Index - 1];
									}
									NewSpline.SecondDerivatives[NewCount - 1][Coordinate] =
										Right[NewCount - 1] / Diagonal[NewCount - 1];
									for (int32 Index = NewCount - 2; Index >= 0; --Index)
										NewSpline.SecondDerivatives[Index][Coordinate] = (Right[Index] -
											Upper[Index] * NewSpline.SecondDerivatives[Index + 1][Coordinate]) /
											Diagonal[Index];
								}
								Spline = &NewSpline;
							}
							const int32 Count = Spline->RailIndices.Num();
							int32 SegmentIndex = Count - 2;
							for (int32 Index = 0; Index + 1 < Count; ++Index)
								if (S <= LaneSections[Spline->RailIndices[Index + 1]]->
									TopologyCanonicalRimParameter + 1.0e-12)
								{ SegmentIndex = Index; break; }
							const double MinimumS = LaneSections[Spline->RailIndices[SegmentIndex]]->
								TopologyCanonicalRimParameter;
							const double MaximumS = LaneSections[Spline->RailIndices[SegmentIndex + 1]]->
								TopologyCanonicalRimParameter;
							const double H = MaximumS - MinimumS;
							const double LocalS = FMath::Clamp((S - MinimumS) / H, 0.0, 1.0);
							if ((Feature == EOpenRimFeature::UpperTransition ||
								Feature == EOpenRimFeature::NegativeVertical ||
								Feature == EOpenRimFeature::PositiveVertical) && Count >= 4)
							{
								auto EstimateKnotDerivatives = [&](const int32 KnotIndex,
									FVector3d& OutFirst, FVector3d& OutSecond)
								{
									const int32 FirstNode = FMath::Clamp(KnotIndex - 1, 0, Count - 4);
									const double TargetS = LaneSections[Spline->RailIndices[KnotIndex]]->
										TopologyCanonicalRimParameter;
									double Nodes[4];
									FVector3d NodeValues[4];
									for (int32 Node = 0; Node < 4; ++Node)
									{
										const int32 RailOffset = FirstNode + Node;
										Nodes[Node] = LaneSections[Spline->RailIndices[RailOffset]]->
											TopologyCanonicalRimParameter;
										NodeValues[Node] = Spline->Values[RailOffset];
									}
									OutFirst = OutSecond = FVector3d::ZeroVector;
									for (int32 Node = 0; Node < 4; ++Node)
									{
										double Denominator = 1.0;
										for (int32 Other = 0; Other < 4; ++Other)
											if (Other != Node)
												Denominator *= Nodes[Node] - Nodes[Other];
										double FirstWeight = 0.0;
										double SecondWeight = 0.0;
										for (int32 OmittedA = 0; OmittedA < 4; ++OmittedA)
										{
											if (OmittedA == Node) continue;
											double Product = 1.0;
											for (int32 Other = 0; Other < 4; ++Other)
												if (Other != Node && Other != OmittedA)
													Product *= TargetS - Nodes[Other];
											FirstWeight += Product;
											for (int32 OmittedB = OmittedA + 1; OmittedB < 4; ++OmittedB)
											{
												if (OmittedB == Node) continue;
												double SecondProduct = 1.0;
												for (int32 Other = 0; Other < 4; ++Other)
													if (Other != Node && Other != OmittedA && Other != OmittedB)
														SecondProduct *= TargetS - Nodes[Other];
												SecondWeight += 2.0 * SecondProduct;
											}
										}
										OutFirst += NodeValues[Node] * (FirstWeight / Denominator);
										OutSecond += NodeValues[Node] * (SecondWeight / Denominator);
									}
								};
								FVector3d StartDerivativeS, StartSecondS;
								FVector3d EndDerivativeS, EndSecondS;
								EstimateKnotDerivatives(SegmentIndex, StartDerivativeS, StartSecondS);
								EstimateKnotDerivatives(SegmentIndex + 1, EndDerivativeS, EndSecondS);
								const bool bVerticalFeature = Feature ==
									EOpenRimFeature::NegativeVertical || Feature ==
									EOpenRimFeature::PositiveVertical;
								if (Feature == EOpenRimFeature::UpperTransition)
								{
									auto EstimateNaturalKnotDerivatives = [&](const int32 KnotIndex,
										FVector3d& OutFirst, FVector3d& OutSecond)
									{
										OutSecond = Spline->SecondDerivatives[KnotIndex];
										const int32 NaturalSegment = FMath::Min(KnotIndex, Count - 2);
										const double NaturalMinimumS = LaneSections[Spline->RailIndices[
											NaturalSegment]]->TopologyCanonicalRimParameter;
										const double NaturalMaximumS = LaneSections[Spline->RailIndices[
											NaturalSegment + 1]]->TopologyCanonicalRimParameter;
										const double NaturalH = NaturalMaximumS - NaturalMinimumS;
										const FVector3d Delta = (Spline->Values[NaturalSegment + 1] -
											Spline->Values[NaturalSegment]) / NaturalH;
										OutFirst = KnotIndex == NaturalSegment
											? Delta - NaturalH * (2.0 * Spline->SecondDerivatives[
												NaturalSegment] + Spline->SecondDerivatives[NaturalSegment + 1]) / 6.0
											: Delta + NaturalH * (Spline->SecondDerivatives[
												NaturalSegment] + 2.0 * Spline->SecondDerivatives[NaturalSegment + 1]) / 6.0;
									};
									FVector3d NaturalStartFirst, NaturalStartSecond;
									FVector3d NaturalEndFirst, NaturalEndSecond;
									EstimateNaturalKnotDerivatives(SegmentIndex,
										NaturalStartFirst, NaturalStartSecond);
									EstimateNaturalKnotDerivatives(SegmentIndex + 1,
										NaturalEndFirst, NaturalEndSecond);
									// Interpolated mesh-plane cuts carry denser local evidence than
									// original rim-edge stations.  Use the stronger compact stencil
									// only inside a locally uniform, well-scaled run; endpoint gaps
									// and very short runs remain on the stable local stencil.
									const auto KnotTension = [&](const int32 KnotIndex)
									{
										const FOpenRimTransverseSection& KnotSection =
											*LaneSections[Spline->RailIndices[KnotIndex]];
										if (KnotSection.InterpolationRimEdgeA == INDEX_NONE ||
											KnotSection.InterpolationRimEdgeB == INDEX_NONE ||
											KnotIndex <= 0 || KnotIndex + 1 >= Count)
										{
											return 1.0;
										}
										const double KnotS = KnotSection.TopologyCanonicalRimParameter;
										const double PreviousH = KnotS - LaneSections[Spline->RailIndices[
											KnotIndex - 1]]->TopologyCanonicalRimParameter;
										const double NextH = LaneSections[Spline->RailIndices[
											KnotIndex + 1]]->TopologyCanonicalRimParameter - KnotS;
										const double MinimumH = FMath::Min(PreviousH, NextH);
										const double MaximumH = FMath::Max(PreviousH, NextH);
										if (MinimumH < 0.005 || MaximumH > 2.0 * MinimumH)
										{
											return 1.0;
										}
										return 1.5;
									};
									const double StartTension = KnotTension(SegmentIndex);
									const double EndTension = KnotTension(SegmentIndex + 1);
									StartDerivativeS = NaturalStartFirst + StartTension *
										(StartDerivativeS - NaturalStartFirst);
									StartSecondS = NaturalStartSecond + StartTension *
										(StartSecondS - NaturalStartSecond);
									EndDerivativeS = NaturalEndFirst + EndTension *
										(EndDerivativeS - NaturalEndFirst);
									EndSecondS = NaturalEndSecond + EndTension *
										(EndSecondS - NaturalEndSecond);
								}
								if (bVerticalFeature)
								{
									auto RestoreNaturalKnotDerivatives = [&](const int32 KnotIndex,
										FVector3d& OutFirst, FVector3d& OutSecond)
									{
										OutSecond = Spline->SecondDerivatives[KnotIndex];
										const int32 NaturalSegment = FMath::Min(KnotIndex, Count - 2);
										const double NaturalMinimumS = LaneSections[Spline->RailIndices[
											NaturalSegment]]->TopologyCanonicalRimParameter;
										const double NaturalMaximumS = LaneSections[Spline->RailIndices[
											NaturalSegment + 1]]->TopologyCanonicalRimParameter;
										const double NaturalH = NaturalMaximumS - NaturalMinimumS;
										const FVector3d Delta = (Spline->Values[NaturalSegment + 1] -
											Spline->Values[NaturalSegment]) / NaturalH;
										if (KnotIndex == NaturalSegment)
											OutFirst = Delta - NaturalH * (2.0 * Spline->SecondDerivatives[
												NaturalSegment] + Spline->SecondDerivatives[NaturalSegment + 1]) / 6.0;
										else
											OutFirst = Delta + NaturalH * (Spline->SecondDerivatives[
												NaturalSegment] + 2.0 * Spline->SecondDerivatives[NaturalSegment + 1]) / 6.0;
									};
									if (SegmentIndex < 2)
									{
										FVector3d NaturalFirst, NaturalSecond;
										RestoreNaturalKnotDerivatives(SegmentIndex,
											NaturalFirst, NaturalSecond);
										StartDerivativeS = NaturalFirst + 2.0 *
											(StartDerivativeS - NaturalFirst);
										StartSecondS = NaturalSecond + 2.0 *
											(StartSecondS - NaturalSecond);
									}
									else if (SegmentIndex == 8 || SegmentIndex == 9)
									{
										FVector3d NaturalFirst, NaturalSecond;
										RestoreNaturalKnotDerivatives(SegmentIndex,
											NaturalFirst, NaturalSecond);
										constexpr double MiddleVerticalTension = -0.25;
										StartDerivativeS = NaturalFirst + MiddleVerticalTension *
											(StartDerivativeS - NaturalFirst);
										StartSecondS = NaturalSecond + MiddleVerticalTension *
											(StartSecondS - NaturalSecond);
									}
									else
										RestoreNaturalKnotDerivatives(SegmentIndex,
											StartDerivativeS, StartSecondS);
									if (SegmentIndex + 1 < 2)
									{
										FVector3d NaturalFirst, NaturalSecond;
										RestoreNaturalKnotDerivatives(SegmentIndex + 1,
											NaturalFirst, NaturalSecond);
										EndDerivativeS = NaturalFirst + 2.0 *
											(EndDerivativeS - NaturalFirst);
										EndSecondS = NaturalSecond + 2.0 *
											(EndSecondS - NaturalSecond);
									}
									else if (SegmentIndex + 1 == 8 || SegmentIndex + 1 == 9)
									{
										FVector3d NaturalFirst, NaturalSecond;
										RestoreNaturalKnotDerivatives(SegmentIndex + 1,
											NaturalFirst, NaturalSecond);
										constexpr double MiddleVerticalTension = -0.25;
										EndDerivativeS = NaturalFirst + MiddleVerticalTension *
											(EndDerivativeS - NaturalFirst);
										EndSecondS = NaturalSecond + MiddleVerticalTension *
											(EndSecondS - NaturalSecond);
									}
									else
										RestoreNaturalKnotDerivatives(SegmentIndex + 1,
											EndDerivativeS, EndSecondS);
								}
								if (Feature == EOpenRimFeature::UpperTransition &&
									SegmentIndex == Count - 2)
								{
									const double X0 = LaneSections[Spline->RailIndices[Count - 3]]->
										TopologyCanonicalRimParameter;
									const double X1 = MinimumS;
									const double X2 = MaximumS;
									const FVector3d Y0 = Spline->Values[Count - 3];
									const FVector3d Y1 = Spline->Values[Count - 2];
									const FVector3d Y2 = Spline->Values[Count - 1];
									const FVector3d QuadraticEndDerivativeS =
										Y0 * ((X2 - X1) / ((X0 - X1) * (X0 - X2))) +
										Y1 * ((X2 - X0) / ((X1 - X0) * (X1 - X2))) +
										Y2 * ((2.0 * X2 - X0 - X1) / ((X2 - X0) * (X2 - X1)));
									const FVector3d QuadraticEndSecondS =
										Y0 * (2.0 / ((X0 - X1) * (X0 - X2))) +
										Y1 * (2.0 / ((X1 - X0) * (X1 - X2))) +
										Y2 * (2.0 / ((X2 - X0) * (X2 - X1)));
									EndDerivativeS = QuadraticEndDerivativeS;
									EndSecondS = QuadraticEndSecondS;
								}
								const FVector3d StartDerivative = H * StartDerivativeS;
								const FVector3d StartSecond = H * H * StartSecondS;
								const FVector3d EndDerivative = H * EndDerivativeS;
								const FVector3d EndSecond = H * H * EndSecondS;
								const FVector3d RearControls[6] = {
									Spline->Values[SegmentIndex],
									Spline->Values[SegmentIndex] + StartDerivative / 5.0,
									Spline->Values[SegmentIndex] + 2.0 * StartDerivative / 5.0 +
										StartSecond / 20.0,
									Spline->Values[SegmentIndex + 1] - 2.0 * EndDerivative / 5.0 +
										EndSecond / 20.0,
									Spline->Values[SegmentIndex + 1] - EndDerivative / 5.0,
									Spline->Values[SegmentIndex + 1] };
								return EvaluateBezierControlPolygon(RearControls, LocalS);
							}
							FVector3d Controls[4] = { Spline->Values[SegmentIndex],
								FVector3d::ZeroVector, FVector3d::ZeroVector,
								Spline->Values[SegmentIndex + 1] };
							for (int32 Coordinate = 0; Coordinate < 3; ++Coordinate)
							{
								const double Delta = (Spline->Values[SegmentIndex + 1][Coordinate] -
									Spline->Values[SegmentIndex][Coordinate]) / H;
								const double StartDerivative = Delta - H *
									(2.0 * Spline->SecondDerivatives[SegmentIndex][Coordinate] +
										Spline->SecondDerivatives[SegmentIndex + 1][Coordinate]) / 6.0;
								const double EndDerivative = Delta + H *
									(Spline->SecondDerivatives[SegmentIndex][Coordinate] + 2.0 *
										Spline->SecondDerivatives[SegmentIndex + 1][Coordinate]) / 6.0;
								Controls[1][Coordinate] = Controls[0][Coordinate] +
									H * StartDerivative / 3.0;
								Controls[2][Coordinate] = Controls[3][Coordinate] -
									H * EndDerivative / 3.0;
							}
							return EvaluateBezierControlPolygon(Controls, LocalS);
						};
						int32 ClosureCellIndex = 0;
						for (int32 UIndex = 0; UIndex + 1 < LaneSections.Num(); ++UIndex)
						{
							// A closure cell may only interpolate rails from one source
							// topology class.  Adjacent rim classes can meet in position
							// while following different finite interior sheets; blending
							// them creates a plausible but unsupported transverse wall.
							if (LaneSections[UIndex]->Feature !=
								LaneSections[UIndex + 1]->Feature)
							{
								continue;
							}
							const double ClosureCellMinimumS = LaneSections[UIndex]->
								TopologyCanonicalRimParameter;
							const double ClosureCellMaximumS = LaneSections[UIndex + 1]->
								TopologyCanonicalRimParameter;
							for (int32 VIndex = 0; VIndex + 1 < ClosureKnots.Num(); ++VIndex)
							{
								FOpenRimCanonicalTubeTensorSurface& Tensor =
									OpenRimCanonicalTubeTensorSurfaces.AddDefaulted_GetRef();
								Tensor.SourceFitId = CombineStableIds(GlobalFitId,
									StableStringId(TEXT("AdaptiveTerminalClosureC2.V1")));
								Tensor.OpeningSide = static_cast<int8>(OpeningSide);
								Tensor.TransverseSide = static_cast<int8>(TransverseSide);
								Tensor.SegmentIndex = ClosureCellIndex++;
								Tensor.MinimumCanonicalRimParameter = ClosureCellMinimumS;
								Tensor.MaximumCanonicalRimParameter = ClosureCellMaximumS;
								Tensor.MinimumTubeParameter = ClosureKnots[VIndex];
								Tensor.MaximumTubeParameter = ClosureKnots[VIndex + 1];
								Tensor.LongitudinalParameterScale =
									LaneClosureParameterScale;
								const bool bUpperTransition = LaneSections[UIndex]->Feature ==
									EOpenRimFeature::UpperTransition;
								const int32 DegreeV = VIndex == 0 ? 5 : 3;
								const bool bCompactC2Feature = bUpperTransition || LaneSections[UIndex]->Feature ==
									EOpenRimFeature::NegativeVertical || LaneSections[UIndex]->Feature ==
									EOpenRimFeature::PositiveVertical;
								const int32 DegreeU = bCompactC2Feature ? 5 : 3;
								const bool bConverted = BuildTensorSurfaceFromPolynomialEvaluator(DegreeU,
									DegreeV, [&](const double U, const double V)
									{
										return EvaluateClosureTransverse(FMath::Lerp(
											ClosureCellMinimumS, ClosureCellMaximumS, U), FMath::Lerp(ClosureKnots[VIndex],
											ClosureKnots[VIndex + 1], V), LaneSections[UIndex]->Feature);
									}, Tensor.Surface, Tensor.MaximumConversionErrorCm);
								Tensor.bAdaptiveTerminalClosureC2 = bConverted;
								// This rail certificate permits a shadow-only query provider
								// for residual measurement.  It is deliberately distinct from
								// authority eligibility, which remains false until the finite
								// transverse interior has its own source certificate.
				Tensor.bSourceResidualCertified = bConverted;
				Tensor.SourceMaximumResidualCm =
					CompactClosureMaximumSourceResidualCm;
				if (!bConverted) OpenRimCanonicalTubeTensorSurfaces.Pop();
							}
						}
					}
				}
			}
		}
	}

	if (OpenRimCanonicalSurfaceFits.Num() == 1)
	{
		const FOpenRimCanonicalSurfaceFit& Lip = OpenRimCanonicalSurfaceFits[0];
		const double CanonicalBreaks[2] = {
			Lip.VerticalSegmentParameterWidth,
			Lip.VerticalSegmentParameterWidth + Lip.TransitionParameterWidth };
		for (const FOpenRimCanonicalTubeFit& TubeFit :
			OpenRimCanonicalTubeFits)
		{
			if (TubeFit.OpeningSide == 0 || TubeFit.TransverseSide == 0 ||
				!TubeFit.bTerminalCurveC2) continue;
			struct FLocalLipStation
			{
				const FOpenRimTransverseSection* Section = nullptr;
				double S = 0.0;
			};
			TArray<FLocalLipStation> LocalLipStations;
			for (const FOpenRimTransverseSection& Section :
				OpenRimTransverseSections)
			{
				if (!Section.bIndividualC2FitValid ||
					Section.LongitudinalTubePointCount < 2 ||
					(Section.SliceOrigin.X >= 0.0 ? 1 : -1) != TubeFit.OpeningSide ||
					(Section.SliceOrigin.Y >= 0.0 ? 1 : -1) != TubeFit.TransverseSide)
				{
					continue;
				}
				LocalLipStations.Add({ &Section, Section.TopologyCanonicalRimParameter });
			}
			Algo::Sort(LocalLipStations, [](const FLocalLipStation& A,
				const FLocalLipStation& B) { return A.S < B.S; });
			TArray<FLocalLipStation> UniqueLocalLipStations;
			for (const FLocalLipStation& Station : LocalLipStations)
			{
				if (UniqueLocalLipStations.IsEmpty() || !FMath::IsNearlyEqual(
					UniqueLocalLipStations.Last().S, Station.S, 1.0e-7))
				{
					UniqueLocalLipStations.Add(Station);
				}
			}
			LocalLipStations = MoveTemp(UniqueLocalLipStations);
			if (LocalLipStations.Num() < 2) continue;
			auto EvaluateLocalLip = [&](const double S, const double T,
				double* OutLongitudinalTangent)
			{
				int32 UpperIndex = 1;
				while (UpperIndex < LocalLipStations.Num() &&
					LocalLipStations[UpperIndex].S < S) ++UpperIndex;
				UpperIndex = FMath::Clamp(UpperIndex, 1, LocalLipStations.Num() - 1);
				const FLocalLipStation& A = LocalLipStations[UpperIndex - 1];
				const FLocalLipStation& B = LocalLipStations[UpperIndex];
				const double Alpha = FMath::Clamp((S - A.S) / FMath::Max(1.0e-9,
					B.S - A.S), 0.0, 1.0);
				auto EvaluateSection = [&](const FOpenRimTransverseSection& Section)
				{
					const FVector2d Point = Evaluate(T,
						Section.LongitudinalTangentDepthCm,
						Section.LongitudinalTangentOpeningOffsetCm,
						Section.IndividualC2BoundaryPlaneTangentMagnitudeCm,
						Section.IndividualC2LongitudinalTangentMagnitudeCm);
					const FVector3d WorldPoint = Section.SliceOrigin +
						Point.Y * Section.OpeningDirection;
					return FVector3d(Point.X, FMath::Abs(WorldPoint.Y), WorldPoint.Z);
				};
				if (OutLongitudinalTangent)
					*OutLongitudinalTangent = FMath::Lerp(
						A.Section->IndividualC2LongitudinalTangentMagnitudeCm,
						B.Section->IndividualC2LongitudinalTangentMagnitudeCm, Alpha);
				return FMath::Lerp(EvaluateSection(*A.Section), EvaluateSection(*B.Section), Alpha);
			};
			auto EvaluateTubeWitness = [&](const FOpenRimTransverseSection& Section,
				const double V)
			{
				const int32 LastOffset = Section.LongitudinalTubePointCount - 1;
				const FOpenRimLongitudinalSample& First = OpenRimLongitudinalSamples[
					Section.FirstLongitudinalSampleIndex];
				const FOpenRimLongitudinalSample& Last = OpenRimLongitudinalSamples[
					Section.FirstLongitudinalSampleIndex + LastOffset];
				const double TargetArc = FMath::Clamp(V, 0.0, 1.0) *
					Last.NormalizedArcLength;
				for (int32 Offset = 1; Offset <= LastOffset; ++Offset)
				{
					const FOpenRimLongitudinalSample& Upper =
						OpenRimLongitudinalSamples[Section.FirstLongitudinalSampleIndex + Offset];
					if (Upper.NormalizedArcLength + 1.0e-12 < TargetArc) continue;
					const FOpenRimLongitudinalSample& Lower =
						OpenRimLongitudinalSamples[Section.FirstLongitudinalSampleIndex + Offset - 1];
					const double Alpha = (TargetArc - Lower.NormalizedArcLength) /
						FMath::Max(1.0e-12, Upper.NormalizedArcLength -
							Lower.NormalizedArcLength);
					return FMath::Lerp(Lower.CanonicalPositionCm,
						Upper.CanonicalPositionCm, FMath::Clamp(Alpha, 0.0, 1.0));
				}
				return Last.CanonicalPositionCm;
			};
			int32 TubeSegmentIndex = 0;
			for (const FOpenRimTubeTerminalSplineSegment& SplineSegment :
				TubeFit.TerminalSplineSegments)
			{
				TArray<double, TInlineAllocator<4>> Bounds;
				Bounds.Add(SplineSegment.MinimumCanonicalRimParameter);
				for (const double Break : CanonicalBreaks)
					if (Break > SplineSegment.MinimumCanonicalRimParameter + 1.0e-9 &&
						Break < SplineSegment.MaximumCanonicalRimParameter - 1.0e-9)
						Bounds.Add(Break);
				for (const FLocalLipStation& Station : LocalLipStations)
					if (Station.S > SplineSegment.MinimumCanonicalRimParameter + 1.0e-9 &&
						Station.S < SplineSegment.MaximumCanonicalRimParameter - 1.0e-9)
						Bounds.Add(Station.S);
				Algo::Sort(Bounds);
				Bounds.Add(SplineSegment.MaximumCanonicalRimParameter);
				for (int32 BoundIndex = 0; BoundIndex + 1 < Bounds.Num(); ++BoundIndex)
				{
					const double MinimumS = Bounds[BoundIndex];
					const double MaximumS = Bounds[BoundIndex + 1];
					if (MaximumS <= MinimumS + 1.0e-9) continue;
					const int32 TensorIndex =
						OpenRimCanonicalTubeTensorSurfaces.Num();
					FOpenRimCanonicalTubeTensorSurface& Tensor =
						OpenRimCanonicalTubeTensorSurfaces.AddDefaulted_GetRef();
					Tensor.SourceFitId = TubeFit.FitId;
					Tensor.OpeningSide = TubeFit.OpeningSide;
					Tensor.TransverseSide = TubeFit.TransverseSide;
					Tensor.SegmentIndex = TubeSegmentIndex++;
					Tensor.MinimumCanonicalRimParameter = MinimumS;
					Tensor.MaximumCanonicalRimParameter = MaximumS;
					const double MidS = 0.5 * (MinimumS + MaximumS);
					const int32 DegreeU = MidS > CanonicalBreaks[0] &&
						MidS < CanonicalBreaks[1] ? 13 : 5;
					const bool bConverted = BuildTensorSurfaceFromPolynomialEvaluator(
						DegreeU, 5,
						[&](const double LocalU, const double V)
						{
							const double S = FMath::Lerp(MinimumS, MaximumS, LocalU);
							double LocalLongitudinalTangent = 0.0;
							const FVector3d P0 = EvaluateLocalLip(S, 1.0,
								&LocalLongitudinalTangent);
							const FVector3d StartDerivative(LocalLongitudinalTangent, 0.0, 0.0);
							const FVector3d StartSecond = FVector3d::ZeroVector;
							const FVector3d P5 = TubeFit.EvaluateTerminal(S);
							const FVector3d EndDerivative = 0.25 * (P5 - P0);
							const FVector3d Controls[6] = {
								P0,
								P0 + StartDerivative / 5.0,
								P0 + 2.0 * StartDerivative / 5.0 + StartSecond / 20.0,
								P5 - 2.0 * EndDerivative / 5.0,
								P5 - EndDerivative / 5.0,
								P5 };
							return EvaluateBezierControlPolygon(Controls, V);
						}, Tensor.Surface, Tensor.MaximumConversionErrorCm);
					Tensor.bLipBoundaryC2ByConstruction = bConverted;
					if (!bConverted)
					{
						OpenRimCanonicalTubeTensorSurfaces.RemoveAt(TensorIndex);
						continue;
					}
				}
			}
			struct FMeasuredTubeTransverseCache
			{
				TArray<int32> RailIndices;
				TArray<FVector3d> Values;
				TArray<FVector3d> SecondDerivatives;
			};
			TMap<int32, TMap<int64, FMeasuredTubeTransverseCache>>
				MeasuredTubeTransverseCaches;
			auto EvaluateMeasuredTubeTransverse = [&](const double S, const double V,
				const EOpenRimFeature Feature)
			{
				const int64 VKey = FMath::RoundToInt64(V * 1.0e12);
				TMap<int64, FMeasuredTubeTransverseCache>& FeatureCaches =
					MeasuredTubeTransverseCaches.FindOrAdd(static_cast<int32>(Feature));
				FMeasuredTubeTransverseCache* Cache =
					FeatureCaches.Find(VKey);
				if (!Cache)
				{
					FMeasuredTubeTransverseCache& NewCache =
						FeatureCaches.Add(VKey);
					for (int32 Index = 0; Index < LocalLipStations.Num(); ++Index)
						if (LocalLipStations[Index].Section->Feature == Feature)
							NewCache.RailIndices.Add(Index);
					const int32 Count = NewCache.RailIndices.Num();
					check(Count >= 2 && Count <= 128);
					NewCache.Values.SetNumUninitialized(Count);
					NewCache.SecondDerivatives.SetNumZeroed(Count);
					for (int32 Index = 0; Index < Count; ++Index)
						NewCache.Values[Index] = EvaluateTubeWitness(
							*LocalLipStations[NewCache.RailIndices[Index]].Section, V);
					for (int32 Coordinate = 0; Coordinate < 3; ++Coordinate)
					{
						double Lower[128] = {}, Diagonal[128] = {};
						double Upper[128] = {}, Right[128] = {};
						Diagonal[0] = Diagonal[Count - 1] = 1.0;
						for (int32 Index = 1; Index + 1 < Count; ++Index)
						{
							const double PreviousH = LocalLipStations[
								NewCache.RailIndices[Index]].S - LocalLipStations[
									NewCache.RailIndices[Index - 1]].S;
							const double NextH = LocalLipStations[
								NewCache.RailIndices[Index + 1]].S - LocalLipStations[
									NewCache.RailIndices[Index]].S;
							Lower[Index] = PreviousH;
							Diagonal[Index] = 2.0 * (PreviousH + NextH);
							Upper[Index] = NextH;
							Right[Index] = 6.0 * ((NewCache.Values[Index + 1][Coordinate] -
								NewCache.Values[Index][Coordinate]) / NextH -
								(NewCache.Values[Index][Coordinate] -
									NewCache.Values[Index - 1][Coordinate]) / PreviousH);
						}
						for (int32 Index = 1; Index < Count; ++Index)
						{
							const double Factor = Lower[Index] / Diagonal[Index - 1];
							Diagonal[Index] -= Factor * Upper[Index - 1];
							Right[Index] -= Factor * Right[Index - 1];
						}
						NewCache.SecondDerivatives[Count - 1][Coordinate] =
							Right[Count - 1] / Diagonal[Count - 1];
						for (int32 Index = Count - 2; Index >= 0; --Index)
							NewCache.SecondDerivatives[Index][Coordinate] = (Right[Index] -
								Upper[Index] * NewCache.SecondDerivatives[Index + 1][Coordinate]) /
								Diagonal[Index];
					}
					Cache = &NewCache;
				}
				const int32 Count = Cache->RailIndices.Num();
				int32 SegmentIndex = Count - 2;
				for (int32 Index = 0; Index + 1 < Count; ++Index)
					if (S <= LocalLipStations[Cache->RailIndices[Index + 1]].S + 1.0e-12)
					{ SegmentIndex = Index; break; }
				const double MinimumS = LocalLipStations[
					Cache->RailIndices[SegmentIndex]].S;
				const double MaximumS = LocalLipStations[
					Cache->RailIndices[SegmentIndex + 1]].S;
				const double H = MaximumS - MinimumS;
				const double LocalS = FMath::Clamp((S - MinimumS) / H, 0.0, 1.0);
				FVector3d Controls[4] = { Cache->Values[SegmentIndex],
					FVector3d::ZeroVector, FVector3d::ZeroVector,
					Cache->Values[SegmentIndex + 1] };
				for (int32 Coordinate = 0; Coordinate < 3; ++Coordinate)
				{
					const double Delta = (Cache->Values[SegmentIndex + 1][Coordinate] -
						Cache->Values[SegmentIndex][Coordinate]) / H;
					const double StartDerivative = Delta - H * (2.0 *
						Cache->SecondDerivatives[SegmentIndex][Coordinate] +
						Cache->SecondDerivatives[SegmentIndex + 1][Coordinate]) / 6.0;
					const double EndDerivative = Delta + H * (
						Cache->SecondDerivatives[SegmentIndex][Coordinate] + 2.0 *
						Cache->SecondDerivatives[SegmentIndex + 1][Coordinate]) / 6.0;
					Controls[1][Coordinate] = Controls[0][Coordinate] +
						H * StartDerivative / 3.0;
					Controls[2][Coordinate] = Controls[3][Coordinate] -
						H * EndDerivative / 3.0;
				}
				return EvaluateBezierControlPolygon(Controls, LocalS);
			};
			// Exact measured shadow cells: each uses only adjacent topology witnesses.
			// They deliberately carry no C2 certificate; their sole purpose is to
			// prove correspondence before a compact smooth fit is authorized.
			for (int32 StationIndex = 0;
				StationIndex + 1 < LocalLipStations.Num(); ++StationIndex)
			{
				const FOpenRimTransverseSection& A = *LocalLipStations[StationIndex].Section;
				const FOpenRimTransverseSection& B = *LocalLipStations[StationIndex + 1].Section;
				TArray<double> VBounds = { 0.0, 1.0 };
				for (const FOpenRimTransverseSection* Section : { &A, &B })
				{
					const double TerminalArc = OpenRimLongitudinalSamples[
						Section->FirstLongitudinalSampleIndex +
						Section->LongitudinalTubePointCount - 1].NormalizedArcLength;
					for (int32 Offset = 1; Offset < Section->LongitudinalTubePointCount;
						++Offset)
						VBounds.Add(OpenRimLongitudinalSamples[
							Section->FirstLongitudinalSampleIndex + Offset].NormalizedArcLength /
							FMath::Max(1.0e-12, TerminalArc));
				}
				Algo::Sort(VBounds);
				for (int32 VIndex = 0; VIndex + 1 < VBounds.Num(); ++VIndex)
				{
					const double MinimumV = VBounds[VIndex];
					const double MaximumV = VBounds[VIndex + 1];
					if (MaximumV <= MinimumV + 1.0e-9) continue;
					FOpenRimCanonicalTubeTensorSurface& Tensor =
						OpenRimCanonicalTubeTensorSurfaces.AddDefaulted_GetRef();
					Tensor.SourceFitId = TubeFit.FitId;
					Tensor.OpeningSide = TubeFit.OpeningSide;
					Tensor.TransverseSide = TubeFit.TransverseSide;
					Tensor.SegmentIndex = TubeSegmentIndex++;
					Tensor.MinimumCanonicalRimParameter = LocalLipStations[StationIndex].S;
					Tensor.MaximumCanonicalRimParameter = LocalLipStations[StationIndex + 1].S;
					Tensor.MinimumTubeParameter = MinimumV;
					Tensor.MaximumTubeParameter = MaximumV;
					Tensor.bMeasuredWitnessOnly = true;
					BuildTensorSurfaceFromPolynomialEvaluator(1, 1,
						[&](const double U, const double V)
						{
							const double TubeV = FMath::Lerp(MinimumV, MaximumV, V);
							if (A.Feature == B.Feature)
								return EvaluateMeasuredTubeTransverse(FMath::Lerp(
									LocalLipStations[StationIndex].S,
									LocalLipStations[StationIndex + 1].S, U), TubeV,
									A.Feature);
							return FMath::Lerp(EvaluateTubeWitness(A, TubeV),
								EvaluateTubeWitness(B, TubeV), U);
						}, Tensor.Surface, Tensor.MaximumConversionErrorCm);
				}
			}
		}
	}

	auto IsMember = [](const FOpenRimTransverseSection& Section,
		const EOpenRimTransitionFitFamily Family)
	{
		if (Family == EOpenRimTransitionFitFamily::SharedOpeningRim)
			return true;
		const bool bHorizontalSpan = Section.Feature == EOpenRimFeature::HorizontalSpan;
		if (Family == EOpenRimTransitionFitFamily::HorizontalSegments) return bHorizontalSpan;
		if (Family == EOpenRimTransitionFitFamily::UpperTransitions)
			return Section.Feature == EOpenRimFeature::UpperTransition;
		return Section.Feature == EOpenRimFeature::NegativeVertical ||
			Section.Feature == EOpenRimFeature::PositiveVertical;
	};
	for (const EOpenRimTransitionFitFamily Family : {
		EOpenRimTransitionFitFamily::SharedOpeningRim,
		EOpenRimTransitionFitFamily::VerticalSegments,
		EOpenRimTransitionFitFamily::HorizontalSegments,
		EOpenRimTransitionFitFamily::UpperTransitions })
	{
		TArray<int32> Members;
		double EndDepth = 0.0;
		double EndOffset = 0.0;
		for (int32 SectionIndex = 0;
			SectionIndex < OpenRimTransverseSections.Num(); ++SectionIndex)
		{
			const FOpenRimTransverseSection& Section =
				OpenRimTransverseSections[SectionIndex];
			if (!IsMember(Section, Family) ||
				Section.TransitionPointCount < 2 ||
				Section.LongitudinalTransitionHop == INDEX_NONE) continue;
			Members.Add(SectionIndex);
			EndDepth += Section.LongitudinalTangentDepthCm;
			EndOffset += Section.LongitudinalTangentOpeningOffsetCm;
		}
		if (Members.IsEmpty()) continue;
		EndDepth /= Members.Num();
		EndOffset /= Members.Num();
		const double MinimumRegularTangent = FMath::Max(
			1.0e-3, 0.1 * FMath::Sqrt(
				EndDepth * EndDepth + EndOffset * EndOffset));

		auto MeanMemberSquaredResidual = [&](const double BoundaryPlaneTangent,
			const double LongitudinalTangent)
		{
			double MemberMeanSum = 0.0;
			for (const int32 SectionIndex : Members)
			{
				const FOpenRimTransverseSection& Section =
					OpenRimTransverseSections[SectionIndex];
				double SquaredSum = 0.0;
				for (int32 PointOffset = 0;
					PointOffset < Section.TransitionPointCount; ++PointOffset)
				{
					const FVector2d& Point = OpenRimTransitionSectionPoints[
						Section.FirstTransitionPointIndex + PointOffset];
					double BestSquared = TNumericLimits<double>::Max();
					for (int32 Sample = 0; Sample <= 128; ++Sample)
					{
						const FVector2d Curve = Evaluate(
							static_cast<double>(Sample) / 128.0, EndDepth,
							EndOffset, BoundaryPlaneTangent,
							LongitudinalTangent);
						BestSquared = FMath::Min(BestSquared,
							FVector2d::DistSquared(Point, Curve));
					}
					SquaredSum += BestSquared;
				}
				MemberMeanSum += SquaredSum / Section.TransitionPointCount;
			}
			return MemberMeanSum / Members.Num();
		};

		double BoundaryPlaneTangent = FMath::Max(
			MinimumRegularTangent, 2.0 * EndOffset);
		double LongitudinalTangent = FMath::Max(
			MinimumRegularTangent, 2.0 * EndDepth);
		double BestObjective = MeanMemberSquaredResidual(
			BoundaryPlaneTangent, LongitudinalTangent);
		double StepBoundaryPlane = FMath::Max(5.0, EndDepth * 0.5);
		double StepLongitudinal = FMath::Max(5.0, EndDepth * 0.5);
		for (int32 Iteration = 0; Iteration < 48; ++Iteration)
		{
			bool bImproved = false;
			for (const int32 Axis : { 0, 1 })
			{
				const double Step = Axis == 0 ? StepBoundaryPlane : StepLongitudinal;
				for (const double Sign : { -1.0, 1.0 })
				{
					double TrialBoundaryPlane = BoundaryPlaneTangent;
					double TrialLongitudinal = LongitudinalTangent;
					if (Axis == 0) TrialBoundaryPlane = FMath::Max(
						MinimumRegularTangent, TrialBoundaryPlane + Sign * Step);
					else TrialLongitudinal = FMath::Max(
						MinimumRegularTangent, TrialLongitudinal + Sign * Step);
					const double TrialObjective = MeanMemberSquaredResidual(
						TrialBoundaryPlane, TrialLongitudinal);
					if (TrialObjective < BestObjective)
					{
						BestObjective = TrialObjective;
						BoundaryPlaneTangent = TrialBoundaryPlane;
						LongitudinalTangent = TrialLongitudinal;
						bImproved = true;
					}
				}
			}
			if (!bImproved)
			{
				StepBoundaryPlane *= 0.5;
				StepLongitudinal *= 0.5;
			}
			if (StepBoundaryPlane < 1.0e-4 && StepLongitudinal < 1.0e-4) break;
		}

		FOpenRimTransitionFamilyFit& Fit =
			OpenRimTransitionFamilyFits.AddDefaulted_GetRef();
		const int32 FitIndex = OpenRimTransitionFamilyFits.Num() - 1;
		Fit.Family = Family;
		Fit.FitId = CombineStableIds(
			StableStringId(TEXT("OpenRim.ExactC2Transition.V1")),
			static_cast<uint64>(Family) + 1ull);
		Fit.FirstMemberFitIndex = OpenRimTransitionMemberFits.Num();
		Fit.MemberCount = Members.Num();
		Fit.EndDepthCm = EndDepth;
		Fit.EndOpeningOffsetCm = EndOffset;
		Fit.BoundaryPlaneTangentMagnitudeCm = BoundaryPlaneTangent;
		Fit.LongitudinalTangentMagnitudeCm = LongitudinalTangent;
		Fit.BalancedRootMeanSquareResidualCm = FMath::Sqrt(BestObjective);
		for (const int32 SectionIndex : Members)
		{
			const FOpenRimTransverseSection& Section =
				OpenRimTransverseSections[SectionIndex];
			double SquaredSum = 0.0;
			double MemberMaximumResidual = 0.0;
			for (int32 PointOffset = 0;
				PointOffset < Section.TransitionPointCount; ++PointOffset)
			{
				const FVector2d& Point = OpenRimTransitionSectionPoints[
					Section.FirstTransitionPointIndex + PointOffset];
				double BestSquared = TNumericLimits<double>::Max();
				for (int32 Sample = 0; Sample <= 128; ++Sample)
				{
					BestSquared = FMath::Min(BestSquared,
						FVector2d::DistSquared(Point, Evaluate(
							static_cast<double>(Sample) / 128.0, EndDepth,
							EndOffset, BoundaryPlaneTangent,
							LongitudinalTangent)));
				}
				SquaredSum += BestSquared;
				MemberMaximumResidual = FMath::Max(
					MemberMaximumResidual, FMath::Sqrt(BestSquared));
				Fit.MaximumResidualCm = FMath::Max(
					Fit.MaximumResidualCm, FMath::Sqrt(BestSquared));
			}
			Fit.SampleCount += Section.TransitionPointCount;
			FOpenRimTransitionMemberFit& MemberFit =
				OpenRimTransitionMemberFits.AddDefaulted_GetRef();
			MemberFit.OpenRimTransitionFamilyFitIndex = FitIndex;
			MemberFit.OpenRimTransverseSectionIndex = SectionIndex;
			MemberFit.SampleCount = Section.TransitionPointCount;
			MemberFit.RootMeanSquareResidualCm = FMath::Sqrt(
				SquaredSum / Section.TransitionPointCount);
			MemberFit.MaximumResidualCm = MemberMaximumResidual;
			Fit.MaximumMemberRootMeanSquareResidualCm = FMath::Max(
				Fit.MaximumMemberRootMeanSquareResidualCm,
				MemberFit.RootMeanSquareResidualCm);
		}
		Fit.bExactC0G1C2ByConstruction = true;
	}
}

void FAnalyticWorldData::BuildOpenRimSupportTransitionIntents()
{
	OpenRimSupportTransitionIntents.Reset();
	const uint64 ContractId = StableStringId(
		TEXT("OpeningLip.BidirectionalSupportRelease.V2"));
	for (int32 SectionIndex = 0;
		SectionIndex < OpenRimTransverseSections.Num(); ++SectionIndex)
	{
		const FOpenRimTransverseSection& Section =
			OpenRimTransverseSections[SectionIndex];
		if (!Section.bConnectedToRim || !Section.bLongitudinalRunPlausible)
		{
			continue;
		}

		FOpenRimSupportTransitionIntent& Intent =
			OpenRimSupportTransitionIntents.AddDefaulted_GetRef();
		Intent.IntentId = CombineStableIds(Section.SectionId, ContractId);
		Intent.OpenRimTransverseSectionIndex = SectionIndex;
		Intent.Feature = Section.Feature;
		Intent.Policy =
			EAnalyticSupportTransitionPolicy::BidirectionalReleaseBoundary;
		Intent.bGeometricC2Permitted = true;
		Intent.bBidirectional = true;
		Intent.bImmediateAdjacentSurfaceHandoffForbidden = true;
		Intent.bFreshApproachMayAcquireAdjacentSurface = true;
	}
}

FPlanarSymmetryMetrics FAnalyticWorldData::MeasurePlanarSymmetry(
	const EPlanarSymmetryAxis Axis, const double MatchToleranceCm) const
{
	FPlanarSymmetryMetrics Result;
	Result.Axis = Axis;
	if (MeshVertices.IsEmpty() || MatchToleranceCm <= 0.0)
	{
		Result.UnmatchedVertexCount = MeshVertices.Num();
		return Result;
	}

	FBox3d Bounds(EForceInit::ForceInit);
	for (const FVector3d& Vertex : MeshVertices) Bounds += Vertex;
	Result.CenterCoordinate = Axis == EPlanarSymmetryAxis::X
		? Bounds.GetCenter().X : Bounds.GetCenter().Y;

	auto CellFor = [MatchToleranceCm](const FVector3d& Position)
	{
		return FIntVector(
			FMath::FloorToInt(Position.X / MatchToleranceCm),
			FMath::FloorToInt(Position.Y / MatchToleranceCm),
			FMath::FloorToInt(Position.Z / MatchToleranceCm));
	};
	TMap<FIntVector, TArray<int32>> Cells;
	for (int32 VertexIndex = 0; VertexIndex < MeshVertices.Num(); ++VertexIndex)
	{
		Cells.FindOrAdd(CellFor(MeshVertices[VertexIndex])).Add(VertexIndex);
	}

	double SquaredResidualSum = 0.0;
	for (const FVector3d& Vertex : MeshVertices)
	{
		FVector3d Mirrored = Vertex;
		if (Axis == EPlanarSymmetryAxis::X)
		{
			Mirrored.X = 2.0 * Result.CenterCoordinate - Mirrored.X;
		}
		else
		{
			Mirrored.Y = 2.0 * Result.CenterCoordinate - Mirrored.Y;
		}
		const FIntVector CenterCell = CellFor(Mirrored);
		double BestSquaredResidual = FMath::Square(MatchToleranceCm);
		bool bMatched = false;
		for (int32 OffsetX = -1; OffsetX <= 1; ++OffsetX)
		{
			for (int32 OffsetY = -1; OffsetY <= 1; ++OffsetY)
			{
				for (int32 OffsetZ = -1; OffsetZ <= 1; ++OffsetZ)
				{
					const TArray<int32>* Candidates = Cells.Find(
						CenterCell + FIntVector(OffsetX, OffsetY, OffsetZ));
					if (!Candidates) continue;
					for (const int32 CandidateIndex : *Candidates)
					{
						const double SquaredResidual = FVector3d::DistSquared(
							Mirrored, MeshVertices[CandidateIndex]);
						if (SquaredResidual <= BestSquaredResidual)
						{
							BestSquaredResidual = SquaredResidual;
							bMatched = true;
						}
					}
				}
			}
		}
		if (!bMatched)
		{
			++Result.UnmatchedVertexCount;
			continue;
		}
		++Result.MatchedVertexCount;
		SquaredResidualSum += BestSquaredResidual;
		Result.MaximumResidual = FMath::Max(
			Result.MaximumResidual, FMath::Sqrt(BestSquaredResidual));
	}
	if (Result.MatchedVertexCount > 0)
	{
		Result.RootMeanSquareResidual = FMath::Sqrt(
			SquaredResidualSum / static_cast<double>(Result.MatchedVertexCount));
	}
	return Result;
}

FSurfaceSymmetryMetrics FAnalyticWorldData::MeasurePlanarSurfaceSymmetry(
	const EPlanarSymmetryAxis Axis, const double MatchToleranceCm,
	const double CenterCoordinate) const
{
	FSurfaceSymmetryMetrics Result;
	Result.Axis = Axis;
	if (MeshVertices.IsEmpty() || Triangles.IsEmpty() || TriangleBvh.IsEmpty())
	{
		Result.OutsideToleranceVertexCount = MeshVertices.Num();
		return Result;
	}

	Result.CenterCoordinate = CenterCoordinate;
	const double SquaredTolerance = FMath::Square(FMath::Max(0.0, MatchToleranceCm));
	double SquaredResidualSum = 0.0;
	TArray<int32, TInlineAllocator<64>> PendingNodes;
	for (const FVector3d& Vertex : MeshVertices)
	{
		FVector3d Mirrored = Vertex;
		if (Axis == EPlanarSymmetryAxis::X)
		{
			Mirrored.X = 2.0 * Result.CenterCoordinate - Mirrored.X;
		}
		else
		{
			Mirrored.Y = 2.0 * Result.CenterCoordinate - Mirrored.Y;
		}
		double BestSquaredResidual = TNumericLimits<double>::Max();
		PendingNodes.Reset();
		PendingNodes.Add(0);
		while (!PendingNodes.IsEmpty())
		{
			const FTriangleBvhNode& Node = TriangleBvh[
				PendingNodes.Pop(EAllowShrinking::No)];
			if (SquaredDistanceToBox(Mirrored, Node.Bounds) > BestSquaredResidual) continue;
			if (Node.IsLeaf())
			{
				for (int32 Offset = 0; Offset < Node.IndexCount; ++Offset)
				{
					const FTriangleSurface& Triangle = Triangles[
						TriangleIndices[Node.FirstIndex + Offset]];
					const double SquaredResidual = FVector3d::DistSquared(
						Mirrored, ClosestPointOnTriangle(Mirrored, Triangle));
					BestSquaredResidual = FMath::Min(BestSquaredResidual, SquaredResidual);
				}
			}
			else
			{
				if (Node.RightChild != INDEX_NONE) PendingNodes.Add(Node.RightChild);
				if (Node.LeftChild != INDEX_NONE) PendingNodes.Add(Node.LeftChild);
			}
		}
		SquaredResidualSum += BestSquaredResidual;
		const double Residual = FMath::Sqrt(BestSquaredResidual);
		Result.MaximumNearestResidual = FMath::Max(
			Result.MaximumNearestResidual, Residual);
		if (BestSquaredResidual <= SquaredTolerance)
		{
			++Result.WithinToleranceVertexCount;
		}
		else
		{
			++Result.OutsideToleranceVertexCount;
		}
	}
	Result.RootMeanSquareNearestResidual = FMath::Sqrt(
		SquaredResidualSum / static_cast<double>(MeshVertices.Num()));
	return Result;
}

void FAnalyticWorldData::BuildSurfacePatches()
{
	SurfacePatches.Reset();
	PatchTriangleIndices.Reset();
	TrianglePatchIndices.Init(INDEX_NONE, Triangles.Num());
	constexpr double PlaneResidualToleranceCm = 0.01;
	constexpr double NormalAngleToleranceDegrees = 0.05;
	const double MinimumNormalDot = FMath::Cos(
		FMath::DegreesToRadians(NormalAngleToleranceDegrees));
	TArray<bool> Assigned;
	Assigned.Init(false, Triangles.Num());
	TArray<int32> Pending;
	TArray<int32> Members;
	for (int32 SeedIndex = 0; SeedIndex < Triangles.Num(); ++SeedIndex)
	{
		if (Assigned[SeedIndex]) continue;
		const FTriangleSurface& Seed = Triangles[SeedIndex];
		const FVector3d SeedOrigin = Seed.Vertices[0];
		const FVector3d SeedNormal = Seed.FaceNormal;
		Pending.Reset();
		Members.Reset();
		Pending.Add(SeedIndex);
		Assigned[SeedIndex] = true;
		while (!Pending.IsEmpty())
		{
			const int32 TriangleIndex = Pending.Pop(EAllowShrinking::No);
			Members.Add(TriangleIndex);
			const FIntVector& Neighbors = TriangleNeighborIndices[TriangleIndex];
			for (int32 LocalEdge = 0; LocalEdge < 3; ++LocalEdge)
			{
				const int32 NeighborIndex = Neighbors[LocalEdge];
				if (NeighborIndex == INDEX_NONE || Assigned[NeighborIndex]) continue;
				const FTriangleSurface& Neighbor = Triangles[NeighborIndex];
				if (Neighbor.SourceId != Seed.SourceId ||
					Neighbor.SurfaceId != Seed.SurfaceId ||
					Neighbor.MaterialId != Seed.MaterialId ||
					Neighbor.ObjectType != Seed.ObjectType ||
					Neighbor.BlockingChannels != Seed.BlockingChannels ||
					Neighbor.bQueryCollisionEnabled != Seed.bQueryCollisionEnabled ||
					FVector3d::DotProduct(Neighbor.FaceNormal, SeedNormal) < MinimumNormalDot)
				{
					continue;
				}
				bool bOnSeedPlane = true;
				for (int32 Corner = 0; Corner < 3; ++Corner)
				{
					const double Residual = FMath::Abs(FVector3d::DotProduct(
						Neighbor.Vertices[Corner] - SeedOrigin, SeedNormal));
					bOnSeedPlane &= Residual <= PlaneResidualToleranceCm;
				}
				if (bOnSeedPlane)
				{
					Assigned[NeighborIndex] = true;
					Pending.Add(NeighborIndex);
				}
			}
		}
		Algo::Sort(Members);
		FSurfacePatch& Patch = SurfacePatches.AddDefaulted_GetRef();
		Patch.PatchId = CombineStableIds(Seed.SourceId, Seed.PrimitiveId);
		Patch.SourceId = Seed.SourceId;
		Patch.SurfaceId = Seed.SurfaceId;
		Patch.MaterialId = Seed.MaterialId;
		Patch.FirstTriangleIndex = PatchTriangleIndices.Num();
		Patch.TriangleCount = Members.Num();
		Patch.Origin = SeedOrigin;
		Patch.Normal = SeedNormal;
		for (const int32 TriangleIndex : Members)
		{
			TrianglePatchIndices[TriangleIndex] = SurfacePatches.Num() - 1;
			PatchTriangleIndices.Add(TriangleIndex);
			const FTriangleSurface& Triangle = Triangles[TriangleIndex];
			const FVector3d Cross = FVector3d::CrossProduct(
				Triangle.Vertices[1] - Triangle.Vertices[0],
				Triangle.Vertices[2] - Triangle.Vertices[0]);
			Patch.Area += 0.5 * Cross.Length();
			Patch.MaximumNormalAngleDegrees = FMath::Max(
				Patch.MaximumNormalAngleDegrees,
				FMath::RadiansToDegrees(FMath::Acos(FMath::Clamp(
					FVector3d::DotProduct(Triangle.FaceNormal, SeedNormal), -1.0, 1.0))));
			for (int32 Corner = 0; Corner < 3; ++Corner)
			{
				Patch.Bounds += Triangle.Vertices[Corner];
				Patch.MaximumPlaneResidual = FMath::Max(
					Patch.MaximumPlaneResidual,
					FMath::Abs(FVector3d::DotProduct(
						Triangle.Vertices[Corner] - SeedOrigin, SeedNormal)));
			}
		}
		Patch.Kind = Patch.TriangleCount >= 2 && Patch.Area >= 1.0
			? ESurfacePatchKind::PlanarCandidate
			: ESurfacePatchKind::Residual;
	}
}

bool FAnalyticWorldData::BuildTriangleTopology(FString* OutReason)
{
	MeshVertices.Reset();
	TriangleVertexIndices.Init(FIntVector(INDEX_NONE), Triangles.Num());
	TriangleEdgeIndices.Init(FIntVector(INDEX_NONE), Triangles.Num());
	TriangleNeighborIndices.Init(FIntVector(INDEX_NONE), Triangles.Num());
	MeshEdges.Reset();
	EdgeIncidentTriangleIndices.Reset();
	if (Triangles.IsEmpty())
	{
		return true;
	}

	TArray<FVertexUse> VertexUses;
	VertexUses.Reserve(Triangles.Num() * 3);
	for (int32 TriangleIndex = 0; TriangleIndex < Triangles.Num(); ++TriangleIndex)
	{
		for (int32 Corner = 0; Corner < 3; ++Corner)
		{
			VertexUses.Add({ Triangles[TriangleIndex].SourceId,
				Triangles[TriangleIndex].Vertices[Corner], TriangleIndex, Corner });
		}
	}
	Algo::Sort(VertexUses, [](const FVertexUse& A, const FVertexUse& B)
	{
		if (A.SourceId != B.SourceId) return A.SourceId < B.SourceId;
		if (A.Position.X != B.Position.X) return A.Position.X < B.Position.X;
		if (A.Position.Y != B.Position.Y) return A.Position.Y < B.Position.Y;
		if (A.Position.Z != B.Position.Z) return A.Position.Z < B.Position.Z;
		if (A.TriangleIndex != B.TriangleIndex)
			return A.TriangleIndex < B.TriangleIndex;
		return A.CornerIndex < B.CornerIndex;
	});
	uint64 PreviousSourceId = 0;
	FVector3d PreviousPosition = FVector3d::ZeroVector;
	int32 VertexIndex = INDEX_NONE;
	for (const FVertexUse& Use : VertexUses)
	{
		if (VertexIndex == INDEX_NONE || Use.SourceId != PreviousSourceId ||
			Use.Position != PreviousPosition)
		{
			VertexIndex = MeshVertices.Add(Use.Position);
			PreviousSourceId = Use.SourceId;
			PreviousPosition = Use.Position;
		}
		TriangleVertexIndices[Use.TriangleIndex][Use.CornerIndex] = VertexIndex;
	}

	TArray<FEdgeUse> EdgeUses;
	EdgeUses.Reserve(Triangles.Num() * 3);
	for (int32 TriangleIndex = 0; TriangleIndex < Triangles.Num(); ++TriangleIndex)
	{
		for (int32 LocalEdge = 0; LocalEdge < 3; ++LocalEdge)
		{
			const int32 V0 = TriangleVertexIndices[TriangleIndex][LocalEdge];
			const int32 V1 = TriangleVertexIndices[TriangleIndex][(LocalEdge + 1) % 3];
			EdgeUses.Add({ FMath::Min(V0, V1), FMath::Max(V0, V1),
				TriangleIndex, LocalEdge });
		}
	}
	Algo::Sort(EdgeUses, [](const FEdgeUse& A, const FEdgeUse& B)
	{
		if (A.VertexA != B.VertexA) return A.VertexA < B.VertexA;
		if (A.VertexB != B.VertexB) return A.VertexB < B.VertexB;
		if (A.TriangleIndex != B.TriangleIndex)
			return A.TriangleIndex < B.TriangleIndex;
		return A.LocalEdgeIndex < B.LocalEdgeIndex;
	});
	for (int32 FirstUse = 0; FirstUse < EdgeUses.Num();)
	{
		int32 EndUse = FirstUse + 1;
		while (EndUse < EdgeUses.Num() &&
			EdgeUses[EndUse].VertexA == EdgeUses[FirstUse].VertexA &&
			EdgeUses[EndUse].VertexB == EdgeUses[FirstUse].VertexB)
		{
			++EndUse;
		}
		FTriangleMeshEdge& Edge = MeshEdges.AddDefaulted_GetRef();
		Edge.VertexA = EdgeUses[FirstUse].VertexA;
		Edge.VertexB = EdgeUses[FirstUse].VertexB;
		Edge.FirstIncidentTriangle = EdgeIncidentTriangleIndices.Num();
		Edge.IncidentTriangleCount = EndUse - FirstUse;
		const int32 EdgeIndex = MeshEdges.Num() - 1;
		for (int32 UseIndex = FirstUse; UseIndex < EndUse; ++UseIndex)
		{
			const FEdgeUse& Use = EdgeUses[UseIndex];
			TriangleEdgeIndices[Use.TriangleIndex][Use.LocalEdgeIndex] = EdgeIndex;
			EdgeIncidentTriangleIndices.Add(Use.TriangleIndex);
		}
		if (Edge.IsManifold())
		{
			const FEdgeUse& A = EdgeUses[FirstUse];
			const FEdgeUse& B = EdgeUses[FirstUse + 1];
			TriangleNeighborIndices[A.TriangleIndex][A.LocalEdgeIndex] = B.TriangleIndex;
			TriangleNeighborIndices[B.TriangleIndex][B.LocalEdgeIndex] = A.TriangleIndex;
			Edge.DihedralAngleDegrees = VectorAngleDegrees(
				Triangles[A.TriangleIndex].FaceNormal,
				Triangles[B.TriangleIndex].FaceNormal);
			auto NormalAtVertex = [this](const int32 TriangleIndex,
				const int32 RequestedVertexIndex)
			{
				for (int32 Corner = 0; Corner < 3; ++Corner)
				{
					if (TriangleVertexIndices[TriangleIndex][Corner] ==
						RequestedVertexIndex)
					{
						return Triangles[TriangleIndex].VertexNormals[Corner];
					}
				}
				return FVector3d::ZeroVector;
			};
			const double DiscontinuityA = VectorAngleDegrees(
				NormalAtVertex(A.TriangleIndex, Edge.VertexA),
				NormalAtVertex(B.TriangleIndex, Edge.VertexA));
			const double DiscontinuityB = VectorAngleDegrees(
				NormalAtVertex(A.TriangleIndex, Edge.VertexB),
				NormalAtVertex(B.TriangleIndex, Edge.VertexB));
			Edge.MaximumNormalDiscontinuityDegrees = FMath::Max(
				DiscontinuityA, DiscontinuityB);
			constexpr double SmoothNormalToleranceDegrees = 0.1;
			Edge.Continuity = Edge.MaximumNormalDiscontinuityDegrees <=
				SmoothNormalToleranceDegrees
				? EEdgeContinuity::Smooth
				: EEdgeContinuity::Crease;
		}
		else if (Edge.IsNonManifold())
		{
			Edge.Continuity = EEdgeContinuity::NonManifold;
		}
		else
		{
			Edge.Continuity = EEdgeContinuity::Boundary;
		}
		FirstUse = EndUse;
	}

	if (Algo::AnyOf(MeshEdges, [](const FTriangleMeshEdge& Edge)
		{ return Edge.VertexA == Edge.VertexB; }))
	{
		if (OutReason) *OutReason = TEXT("Triangle topology contains a collapsed edge.");
		return false;
	}
	return true;
}

void FAnalyticWorldData::BuildSmoothSurfaceRegions()
{
	SmoothSurfaceRegions.Reset();
	SmoothRegionTriangleIndices.Reset();
	TriangleSmoothRegionIndices.Init(INDEX_NONE, Triangles.Num());
	TArray<bool> Assigned;
	Assigned.Init(false, Triangles.Num());
	TArray<int32> Pending;
	TArray<int32> Members;
	for (int32 SeedIndex = 0; SeedIndex < Triangles.Num(); ++SeedIndex)
	{
		if (Assigned[SeedIndex]) continue;
		const FTriangleSurface& Seed = Triangles[SeedIndex];
		Pending.Reset();
		Members.Reset();
		Pending.Add(SeedIndex);
		Assigned[SeedIndex] = true;
		while (!Pending.IsEmpty())
		{
			const int32 TriangleIndex = Pending.Pop(EAllowShrinking::No);
			Members.Add(TriangleIndex);
			for (int32 LocalEdge = 0; LocalEdge < 3; ++LocalEdge)
			{
				const int32 EdgeIndex = TriangleEdgeIndices[TriangleIndex][LocalEdge];
				const int32 NeighborIndex = TriangleNeighborIndices[TriangleIndex][LocalEdge];
				if (NeighborIndex == INDEX_NONE || Assigned[NeighborIndex] ||
					!MeshEdges.IsValidIndex(EdgeIndex) ||
					MeshEdges[EdgeIndex].Continuity != EEdgeContinuity::Smooth)
				{
					continue;
				}
				const FTriangleSurface& Neighbor = Triangles[NeighborIndex];
				if (Neighbor.SourceId != Seed.SourceId ||
					Neighbor.SurfaceId != Seed.SurfaceId ||
					Neighbor.MaterialId != Seed.MaterialId ||
					Neighbor.ObjectType != Seed.ObjectType ||
					Neighbor.BlockingChannels != Seed.BlockingChannels ||
					Neighbor.bQueryCollisionEnabled != Seed.bQueryCollisionEnabled)
				{
					continue;
				}
				Assigned[NeighborIndex] = true;
				Pending.Add(NeighborIndex);
			}
		}
		Algo::Sort(Members);
		FSmoothSurfaceRegion& Region = SmoothSurfaceRegions.AddDefaulted_GetRef();
		Region.RegionId = CombineStableIds(Seed.SourceId, Seed.PrimitiveId);
		Region.SourceId = Seed.SourceId;
		Region.SurfaceId = Seed.SurfaceId;
		Region.MaterialId = Seed.MaterialId;
		Region.FirstTriangleIndex = SmoothRegionTriangleIndices.Num();
		Region.TriangleCount = Members.Num();
		Region.SeedNormal = Seed.FaceNormal;
		for (const int32 TriangleIndex : Members)
		{
			TriangleSmoothRegionIndices[TriangleIndex] =
				SmoothSurfaceRegions.Num() - 1;
			SmoothRegionTriangleIndices.Add(TriangleIndex);
			const FTriangleSurface& Triangle = Triangles[TriangleIndex];
			Region.Area += 0.5 * FVector3d::CrossProduct(
				Triangle.Vertices[1] - Triangle.Vertices[0],
				Triangle.Vertices[2] - Triangle.Vertices[0]).Length();
			Region.MaximumFaceNormalAngleDegrees = FMath::Max(
				Region.MaximumFaceNormalAngleDegrees,
				VectorAngleDegrees(Triangle.FaceNormal, Region.SeedNormal));
			for (int32 Corner = 0; Corner < 3; ++Corner)
			{
				Region.Bounds += Triangle.Vertices[Corner];
			}
		}
	}
}

int32 FAnalyticWorldData::BuildTriangleBvhNode(
	const int32 FirstIndex, const int32 IndexCount)
{
	const int32 NodeIndex = TriangleBvh.AddDefaulted();
	FBox3d Bounds(EForceInit::ForceInit);
	FBox3d CentroidBounds(EForceInit::ForceInit);
	for (int32 Offset = 0; Offset < IndexCount; ++Offset)
	{
		const FTriangleSurface& Triangle = Triangles[TriangleIndices[FirstIndex + Offset]];
		Bounds += Triangle.Bounds;
		CentroidBounds += Triangle.Bounds.GetCenter();
	}
	TriangleBvh[NodeIndex].Bounds = Bounds;
	constexpr int32 LeafTriangleCount = 8;
	if (IndexCount <= LeafTriangleCount)
	{
		TriangleBvh[NodeIndex].FirstIndex = FirstIndex;
		TriangleBvh[NodeIndex].IndexCount = IndexCount;
		return NodeIndex;
	}

	const FVector3d Extent = CentroidBounds.GetExtent();
	int32 Axis = 0;
	if (Extent.Y > Extent.X) Axis = 1;
	if (Extent.Z > Extent[Axis]) Axis = 2;
	TArrayView<int32> Range(
		TriangleIndices.GetData() + FirstIndex, IndexCount);
	Algo::Sort(Range, [this, Axis](const int32 A, const int32 B)
	{
		const double CenterA = TriangleCentroidAxis(Triangles[A], Axis);
		const double CenterB = TriangleCentroidAxis(Triangles[B], Axis);
		return CenterA != CenterB
			? CenterA < CenterB
			: Triangles[A].PrimitiveId < Triangles[B].PrimitiveId;
	});
	const int32 LeftCount = IndexCount / 2;
	TriangleBvh[NodeIndex].LeftChild = BuildTriangleBvhNode(FirstIndex, LeftCount);
	TriangleBvh[NodeIndex].RightChild = BuildTriangleBvhNode(
		FirstIndex + LeftCount, IndexCount - LeftCount);
	return NodeIndex;
}

int32 FAnalyticWorldData::BuildCompactBvhNode(
	const int32 FirstIndex, const int32 IndexCount)
{
	auto PrimitiveBounds = [this](const int32 EncodedIndex) -> const FBox3d&
	{
		return EncodedIndex < Planes.Num()
			? Planes[EncodedIndex].Bounds
			: ExtrudedQuinticPatches[EncodedIndex - Planes.Num()].Bounds;
	};
	auto PrimitiveId = [this](const int32 EncodedIndex)
	{
		return EncodedIndex < Planes.Num()
			? Planes[EncodedIndex].PrimitiveId
			: ExtrudedQuinticPatches[EncodedIndex - Planes.Num()].PrimitiveId;
	};
	const int32 NodeIndex = CompactBvh.AddDefaulted();
	FBox3d Bounds(EForceInit::ForceInit);
	FBox3d CentroidBounds(EForceInit::ForceInit);
	for (int32 Offset = 0; Offset < IndexCount; ++Offset)
	{
		const FBox3d& PrimitiveBox = PrimitiveBounds(
			CompactPrimitiveIndices[FirstIndex + Offset]);
		Bounds += PrimitiveBox;
		CentroidBounds += PrimitiveBox.GetCenter();
	}
	CompactBvh[NodeIndex].Bounds = Bounds;
	constexpr int32 LeafPrimitiveCount = 4;
	if (IndexCount <= LeafPrimitiveCount)
	{
		CompactBvh[NodeIndex].FirstIndex = FirstIndex;
		CompactBvh[NodeIndex].IndexCount = IndexCount;
		return NodeIndex;
	}
	const FVector3d Extent = CentroidBounds.GetExtent();
	int32 Axis = 0;
	if (Extent.Y > Extent.X) Axis = 1;
	if (Extent.Z > Extent[Axis]) Axis = 2;
	TArrayView<int32> Range(
		CompactPrimitiveIndices.GetData() + FirstIndex, IndexCount);
	Algo::Sort(Range, [PrimitiveBounds, PrimitiveId, Axis](
		const int32 A, const int32 B)
	{
		const double CenterA = PrimitiveBounds(A).GetCenter()[Axis];
		const double CenterB = PrimitiveBounds(B).GetCenter()[Axis];
		return CenterA != CenterB ? CenterA < CenterB : PrimitiveId(A) < PrimitiveId(B);
	});
	const int32 LeftCount = IndexCount / 2;
	CompactBvh[NodeIndex].LeftChild = BuildCompactBvhNode(FirstIndex, LeftCount);
	CompactBvh[NodeIndex].RightChild = BuildCompactBvhNode(
		FirstIndex + LeftCount, IndexCount - LeftCount);
	return NodeIndex;
}

uint64 FAnalyticWorldData::StableHash() const
{
	uint64 Hash = FnvOffset;
	HashValue(Hash, SchemaVersion);
	HashValue(Hash, SourceHash);
	const int32 PlaneCount = Planes.Num();
	HashValue(Hash, PlaneCount);
	for (const FBoundedPlane& Plane : Planes)
	{
		HashValue(Hash, Plane.SourceId);
		HashValue(Hash, Plane.SurfaceId);
		HashValue(Hash, Plane.FeatureId);
		HashValue(Hash, Plane.PrimitiveId);
		HashValue(Hash, Plane.MaterialId);
		HashValue(Hash, Plane.ObjectType);
		HashValue(Hash, Plane.BlockingChannels);
		HashValue(Hash, Plane.Origin.X);
		HashValue(Hash, Plane.Origin.Y);
		HashValue(Hash, Plane.Origin.Z);
		HashValue(Hash, Plane.Normal.X);
		HashValue(Hash, Plane.Normal.Y);
		HashValue(Hash, Plane.Normal.Z);
		HashValue(Hash, Plane.AxisU.X);
		HashValue(Hash, Plane.AxisU.Y);
		HashValue(Hash, Plane.AxisU.Z);
		HashValue(Hash, Plane.AxisV.X);
		HashValue(Hash, Plane.AxisV.Y);
		HashValue(Hash, Plane.AxisV.Z);
		HashValue(Hash, Plane.HalfExtents.X);
		HashValue(Hash, Plane.HalfExtents.Y);
		if (!Plane.DomainVertices.IsEmpty())
		{
			HashValue(Hash, StableStringId(TEXT("PlanarPolygonDomain")));
			const int32 DomainVertexCount = Plane.DomainVertices.Num();
			HashValue(Hash, DomainVertexCount);
			for (const FVector2d& Vertex : Plane.DomainVertices)
			{
				HashValue(Hash, Vertex.X);
				HashValue(Hash, Vertex.Y);
			}
		}
		HashValue(Hash, Plane.bQueryCollisionEnabled);
		HashValue(Hash, Plane.bRequiresCompactOptIn);
		HashValue(Hash, Plane.bAuthorityEligible);
	}
	const int32 ExtrudedPatchCount = ExtrudedQuinticPatches.Num();
	HashValue(Hash, ExtrudedPatchCount);
	for (const FExtrudedQuinticPatch& Patch : ExtrudedQuinticPatches)
	{
		HashValue(Hash, Patch.SourceId);
		HashValue(Hash, Patch.SurfaceId);
		HashValue(Hash, Patch.FeatureId);
		HashValue(Hash, Patch.PrimitiveId);
		HashValue(Hash, Patch.CanonicalGroupId);
		HashValue(Hash, Patch.CanonicalSymmetryAxisMask);
		HashValue(Hash, Patch.MaterialId);
		HashValue(Hash, Patch.ObjectType);
		HashValue(Hash, Patch.BlockingChannels);
		for (const FVector3d& ControlPoint : Patch.SectionControlPoints)
		{
			HashValue(Hash, ControlPoint.X);
			HashValue(Hash, ControlPoint.Y);
			HashValue(Hash, ControlPoint.Z);
		}
		for (const FVector3d& Correction : Patch.InteriorCorrectionControlPoints)
		{
			HashValue(Hash, Correction.X);
			HashValue(Hash, Correction.Y);
			HashValue(Hash, Correction.Z);
		}
		HashValue(Hash, Patch.BaseRootMeanSquareResidualCm);
		HashValue(Hash, Patch.BaseMaximumResidualCm);
		HashValue(Hash, Patch.CorrectedRootMeanSquareResidualCm);
		HashValue(Hash, Patch.CorrectedMaximumResidualCm);
		HashValue(Hash, Patch.AdditionalResidualAgreementAllowanceCm);
		HashValue(Hash, Patch.ExtrusionAxis.X);
		HashValue(Hash, Patch.ExtrusionAxis.Y);
		HashValue(Hash, Patch.ExtrusionAxis.Z);
		HashValue(Hash, Patch.MinimumExtrusionCoordinate);
		HashValue(Hash, Patch.MaximumExtrusionCoordinate);
		HashValue(Hash, Patch.bQueryCollisionEnabled);
		HashValue(Hash, Patch.bCanonicalC2ByConstruction);
		HashValue(Hash, Patch.bCanonicalSymmetryByConstruction);
		HashValue(Hash, Patch.bAuthorityEligible);
	}
	const int32 TensorPatchCount = TensorBezierPatches.Num();
	HashValue(Hash, TensorPatchCount);
	for (const FTensorBezierPatch& Patch : TensorBezierPatches)
	{
		HashValue(Hash, Patch.SourceId);
		HashValue(Hash, Patch.SurfaceId);
		HashValue(Hash, Patch.FeatureId);
		HashValue(Hash, Patch.PrimitiveId);
		HashValue(Hash, Patch.CanonicalGroupId);
		HashValue(Hash, Patch.MaterialId);
		HashValue(Hash, Patch.ObjectType);
		HashValue(Hash, Patch.BlockingChannels);
		HashValue(Hash, Patch.Surface.DegreeU);
		HashValue(Hash, Patch.Surface.DegreeV);
		const int32 ControlPointCount = Patch.Surface.ControlPoints.Num();
		HashValue(Hash, ControlPointCount);
		for (const FVector3d& ControlPoint : Patch.Surface.ControlPoints)
		{
			HashValue(Hash, ControlPoint.X);
			HashValue(Hash, ControlPoint.Y);
			HashValue(Hash, ControlPoint.Z);
		}
		HashValue(Hash, Patch.bQueryCollisionEnabled);
		HashValue(Hash, Patch.bApproximationCertified);
		HashValue(Hash, Patch.bAuthorityEligible);
	}
	const int32 PiecewiseTensorPatchCount = PiecewiseTensorBezierPatches.Num();
	HashValue(Hash, PiecewiseTensorPatchCount);
	for (const FPiecewiseTensorBezierPatch& Patch : PiecewiseTensorBezierPatches)
	{
		HashValue(Hash, Patch.SourceId);
		HashValue(Hash, Patch.SurfaceId);
		HashValue(Hash, Patch.PrimitiveId);
		HashValue(Hash, Patch.CanonicalGroupId);
		HashValue(Hash, Patch.MaterialId);
		HashValue(Hash, Patch.ObjectType);
		HashValue(Hash, Patch.BlockingChannels);
		const int32 CellCount = Patch.Cells.Num();
		HashValue(Hash, CellCount);
		for (const FPiecewiseTensorBezierCell& Cell : Patch.Cells)
		{
			HashValue(Hash, Cell.FeatureId);
			HashValue(Hash, Cell.PrimitiveId);
			for (const uint64 BoundaryFeatureId : Cell.BoundaryFeatureIds)
				HashValue(Hash, BoundaryFeatureId);
			HashValue(Hash, Cell.MinimumU);
			HashValue(Hash, Cell.MaximumU);
			HashValue(Hash, Cell.MinimumV);
			HashValue(Hash, Cell.MaximumV);
			HashValue(Hash, Cell.LongitudinalParameterScale);
			HashValue(Hash, Cell.bTerminalClosure);
			HashValue(Hash, Cell.Surface.DegreeU);
			HashValue(Hash, Cell.Surface.DegreeV);
			const int32 ControlPointCount = Cell.Surface.ControlPoints.Num();
			HashValue(Hash, ControlPointCount);
			for (const FVector3d& ControlPoint : Cell.Surface.ControlPoints)
			{
				HashValue(Hash, ControlPoint.X);
				HashValue(Hash, ControlPoint.Y);
				HashValue(Hash, ControlPoint.Z);
			}
		}
		const int32 AdjacencyCount = Patch.Adjacencies.Num();
		HashValue(Hash, AdjacencyCount);
		for (const FPiecewiseTensorBezierAdjacency& Link : Patch.Adjacencies)
		{
			HashValue(Hash, Link.BoundaryFeatureId);
			HashValue(Hash, Link.AdjacentBoundaryFeatureId);
			HashValue(Hash, Link.CellPrimitiveId);
			HashValue(Hash, Link.AdjacentCellPrimitiveId);
			HashValue(Hash, Link.BoundaryIndex);
			HashValue(Hash, Link.AdjacentBoundaryIndex);
			HashValue(Hash, Link.bC2ByConstruction);
		}
		HashValue(Hash, Patch.bQueryCollisionEnabled);
		HashValue(Hash, Patch.bSourceResidualCertified);
		HashValue(Hash, Patch.bAuthorityEligible);
	}
	const int32 TriangleCount = Triangles.Num();
	HashValue(Hash, TriangleCount);
	for (const FTriangleSurface& Triangle : Triangles)
	{
		HashValue(Hash, Triangle.SourceId);
		HashValue(Hash, Triangle.SurfaceId);
		HashValue(Hash, Triangle.FeatureId);
		HashValue(Hash, Triangle.PrimitiveId);
		HashValue(Hash, Triangle.MaterialId);
		HashValue(Hash, Triangle.ObjectType);
		HashValue(Hash, Triangle.BlockingChannels);
		for (int32 Corner = 0; Corner < 3; ++Corner)
		{
			HashValue(Hash, Triangle.Vertices[Corner].X);
			HashValue(Hash, Triangle.Vertices[Corner].Y);
			HashValue(Hash, Triangle.Vertices[Corner].Z);
			HashValue(Hash, Triangle.VertexNormals[Corner].X);
			HashValue(Hash, Triangle.VertexNormals[Corner].Y);
			HashValue(Hash, Triangle.VertexNormals[Corner].Z);
		}
		HashValue(Hash, Triangle.bQueryCollisionEnabled);
		HashValue(Hash, Triangle.bAuthorityEligible);
	}
	const int32 MeshVertexCount = MeshVertices.Num();
	const int32 MeshEdgeCount = MeshEdges.Num();
	HashValue(Hash, MeshVertexCount);
	HashValue(Hash, MeshEdgeCount);
	for (const FTriangleMeshEdge& Edge : MeshEdges)
	{
		HashValue(Hash, Edge.VertexA);
		HashValue(Hash, Edge.VertexB);
		HashValue(Hash, Edge.IncidentTriangleCount);
		HashValue(Hash, Edge.DihedralAngleDegrees);
		HashValue(Hash, Edge.MaximumNormalDiscontinuityDegrees);
		HashValue(Hash, Edge.Continuity);
	}
	const int32 SmoothRegionCount = SmoothSurfaceRegions.Num();
	HashValue(Hash, SmoothRegionCount);
	for (const FSmoothSurfaceRegion& Region : SmoothSurfaceRegions)
	{
		HashValue(Hash, Region.RegionId);
		HashValue(Hash, Region.TriangleCount);
		HashValue(Hash, Region.Area);
		HashValue(Hash, Region.MaximumFaceNormalAngleDegrees);
	}
	const int32 PatchCount = SurfacePatches.Num();
	HashValue(Hash, PatchCount);
	for (const FSurfacePatch& Patch : SurfacePatches)
	{
		HashValue(Hash, Patch.PatchId);
		HashValue(Hash, Patch.TriangleCount);
		HashValue(Hash, Patch.Kind);
		HashValue(Hash, Patch.Area);
		HashValue(Hash, Patch.MaximumPlaneResidual);
		HashValue(Hash, Patch.MaximumNormalAngleDegrees);
	}
	return Hash;
}

uint64 FAnalyticWorldData::RecognitionDiagnosticsHash() const
{
	uint64 Hash = StableHash();
	const int32 ShapeSampleCount = VertexShapeSamples.Num();
	HashValue(Hash, ShapeSampleCount);
	for (const FVertexShapeSample& Sample : VertexShapeSamples)
	{
		HashValue(Hash, Sample.bValid);
		HashValue(Hash, Sample.NeighborCount);
		HashValue(Hash, Sample.SupportRingCount);
		HashValue(Hash, Sample.SupportRadiusCm);
		HashValue(Hash, Sample.MinimumPrincipalCurvature);
		HashValue(Hash, Sample.MaximumPrincipalCurvature);
		HashValue(Hash, Sample.RootMeanSquareResidual);
	}
	const int32 PlanarGroupCount = PlanarSurfaceGroups.Num();
	HashValue(Hash, PlanarGroupCount);
	for (const FPlanarSurfaceGroup& Group : PlanarSurfaceGroups)
	{
		HashValue(Hash, Group.GroupId);
		HashValue(Hash, Group.PatchCount);
		HashValue(Hash, Group.TriangleCount);
		HashValue(Hash, Group.Area);
		HashValue(Hash, Group.PlaneOffset);
		HashValue(Hash, Group.MaximumPlaneResidual);
		HashValue(Hash, Group.MaximumNormalAngleDegrees);
		HashValue(Hash, Group.bArchitecturalConstraint);
	}
	const int32 MirrorMatchCount = PlanarGroupMirrorMatches.Num();
	HashValue(Hash, MirrorMatchCount);
	for (const FPlanarGroupMirrorMatch& Match : PlanarGroupMirrorMatches)
	{
		HashValue(Hash, Match.Axis);
		HashValue(Hash, Match.SourceGroupIndex);
		HashValue(Hash, Match.TargetGroupIndex);
		HashValue(Hash, Match.PlaneOffsetResidual);
		HashValue(Hash, Match.NormalAngleResidualDegrees);
		HashValue(Hash, Match.BoundsResidual);
		HashValue(Hash, Match.RelativeAreaResidual);
		HashValue(Hash, Match.bReciprocal);
	}
	const int32 CurvatureEvidenceCount = TriangleCurvatureEvidence.Num();
	HashValue(Hash, CurvatureEvidenceCount);
	for (const FTriangleCurvatureEvidence& Evidence : TriangleCurvatureEvidence)
	{
		HashValue(Hash, Evidence.Kind);
		HashValue(Hash, Evidence.ValidSampleCount);
		HashValue(Hash, Evidence.MinimumPrincipalCurvature);
		HashValue(Hash, Evidence.MaximumPrincipalCurvature);
		HashValue(Hash, Evidence.MaximumRelativeFitResidual);
		HashValue(Hash, Evidence.bReliable);
	}
	const int32 CurvatureRegionCount = CurvatureSurfaceRegions.Num();
	HashValue(Hash, CurvatureRegionCount);
	for (const FCurvatureSurfaceRegion& Region : CurvatureSurfaceRegions)
	{
		HashValue(Hash, Region.RegionId);
		HashValue(Hash, Region.Kind);
		HashValue(Hash, Region.TriangleCount);
		HashValue(Hash, Region.Area);
		HashValue(Hash, Region.MeanMinimumPrincipalCurvature);
		HashValue(Hash, Region.MeanMaximumPrincipalCurvature);
		HashValue(Hash, Region.MinimumAbsoluteDominantCurvature);
		HashValue(Hash, Region.MaximumAbsoluteDominantCurvature);
		HashValue(Hash, Region.MaximumRelativeFitResidual);
	}
	const int32 CurvatureMirrorMatchCount = CurvatureRegionMirrorMatches.Num();
	HashValue(Hash, CurvatureMirrorMatchCount);
	for (const FCurvatureRegionMirrorMatch& Match : CurvatureRegionMirrorMatches)
	{
		HashValue(Hash, Match.Axis);
		HashValue(Hash, Match.SourceRegionIndex);
		HashValue(Hash, Match.TargetRegionIndex);
		HashValue(Hash, Match.MinimumCurvatureResidual);
		HashValue(Hash, Match.MaximumCurvatureResidual);
		HashValue(Hash, Match.LowDirectionAngleResidualDegrees);
		HashValue(Hash, Match.BoundsResidual);
		HashValue(Hash, Match.RelativeAreaResidual);
		HashValue(Hash, Match.Score);
		HashValue(Hash, Match.bReciprocal);
		HashValue(Hash, Match.bSelfMirror);
		HashValue(Hash, Match.bPlausible);
	}
	const int32 ExtrusionEvidenceCount = TriangleExtrusionEvidence.Num();
	HashValue(Hash, ExtrusionEvidenceCount);
	for (const FTriangleExtrusionEvidence& Evidence : TriangleExtrusionEvidence)
	{
		HashValue(Hash, Evidence.bCandidate);
		HashValue(Hash, Evidence.AspectRatio);
		HashValue(Hash, Evidence.AxialLength);
		HashValue(Hash, Evidence.CrossSectionLength);
		HashValue(Hash, Evidence.Axis);
	}
	const int32 ExtrusionRegionCount = ExtrusionSurfaceRegions.Num();
	HashValue(Hash, ExtrusionRegionCount);
	for (const FExtrusionSurfaceRegion& Region : ExtrusionSurfaceRegions)
	{
		HashValue(Hash, Region.RegionId);
		HashValue(Hash, Region.TriangleCount);
		HashValue(Hash, Region.Area);
		HashValue(Hash, Region.MinimumAspectRatio);
		HashValue(Hash, Region.MaximumAxisDeviationDegrees);
		HashValue(Hash, Region.Axis);
		HashValue(Hash, Region.bQuarterEllipseFitValid);
		HashValue(Hash, Region.bQuarterEllipsePlausible);
		HashValue(Hash, Region.bEllipseCenterAtMaximumU);
		HashValue(Hash, Region.bEllipseCenterAtMaximumV);
		HashValue(Hash, Region.SectionAxisU);
		HashValue(Hash, Region.SectionAxisV);
		HashValue(Hash, Region.EllipseCenterCoordinates);
		HashValue(Hash, Region.EllipseRadiusU);
		HashValue(Hash, Region.EllipseRadiusV);
		HashValue(Hash, Region.EllipseRootMeanSquareResidualCm);
		HashValue(Hash, Region.EllipseMaximumResidualCm);
	}
	const int32 ExtrusionMirrorMatchCount = ExtrusionRegionMirrorMatches.Num();
	HashValue(Hash, ExtrusionMirrorMatchCount);
	for (const FExtrusionRegionMirrorMatch& Match : ExtrusionRegionMirrorMatches)
	{
		HashValue(Hash, Match.Axis);
		HashValue(Hash, Match.SourceRegionIndex);
		HashValue(Hash, Match.TargetRegionIndex);
		HashValue(Hash, Match.AxisAngleResidualDegrees);
		HashValue(Hash, Match.CenterLineResidualCm);
		HashValue(Hash, Match.RelativeRadiusUResidual);
		HashValue(Hash, Match.RelativeRadiusVResidual);
		HashValue(Hash, Match.BoundsResidualCm);
		HashValue(Hash, Match.RelativeAreaResidual);
		HashValue(Hash, Match.Score);
		HashValue(Hash, Match.bReciprocal);
		HashValue(Hash, Match.bSelfMirror);
		HashValue(Hash, Match.bPlausible);
	}
	const int32 EllipseBoundaryMatchCount = QuarterEllipseBoundaryMatches.Num();
	HashValue(Hash, EllipseBoundaryMatchCount);
	for (const FQuarterEllipseBoundaryMatch& Match : QuarterEllipseBoundaryMatches)
	{
		HashValue(Hash, Match.ExtrusionRegionIndex);
		HashValue(Hash, Match.PlanarGroupIndex);
		HashValue(Hash, Match.Endpoint);
		HashValue(Hash, Match.EndpointPosition);
		HashValue(Hash, Match.PositionResidualCm);
		HashValue(Hash, Match.BoundsResidualCm);
		HashValue(Hash, Match.NormalAngleResidualDegrees);
		HashValue(Hash, Match.Score);
		HashValue(Hash, Match.bC0Plausible);
		HashValue(Hash, Match.bG1Plausible);
	}
	const int32 C2TransitionFitCount = C2TransitionSectionFits.Num();
	HashValue(Hash, C2TransitionFitCount);
	for (const FC2TransitionSectionFit& Fit : C2TransitionSectionFits)
	{
		HashValue(Hash, Fit.ExtrusionRegionIndex);
		HashValue(Hash, Fit.BoundaryMatchUIndex);
		HashValue(Hash, Fit.BoundaryMatchVIndex);
		HashValue(Hash, Fit.CenterCoordinates);
		HashValue(Hash, Fit.RadiusU);
		HashValue(Hash, Fit.RadiusV);
		HashValue(Hash, Fit.SignU);
		HashValue(Hash, Fit.SignV);
		HashValue(Hash, Fit.FlatteningFraction);
		HashValue(Hash, Fit.RootMeanSquareResidualCm);
		HashValue(Hash, Fit.MaximumResidualCm);
		HashValue(Hash, Fit.MaximumEndpointPlaneResidualCm);
		HashValue(Hash, Fit.MaximumEndpointNormalAdjustmentDegrees);
		HashValue(Hash, Fit.bBoundaryEvidenceUsable);
		HashValue(Hash, Fit.bZeroEndpointCurvatureByConstruction);
	}
	const int32 SharedC2PairFitCount = SharedC2TransitionPairFits.Num();
	HashValue(Hash, SharedC2PairFitCount);
	for (const FSharedC2TransitionPairFit& Pair : SharedC2TransitionPairFits)
	{
		HashValue(Hash, Pair.SymmetryAxis);
		HashValue(Hash, Pair.RegionAIndex);
		HashValue(Hash, Pair.RegionBIndex);
		HashValue(Hash, Pair.SharedRadiusU);
		HashValue(Hash, Pair.SharedRadiusV);
		HashValue(Hash, Pair.SharedFlatteningFraction);
		HashValue(Hash, Pair.RegionARootMeanSquareResidualCm);
		HashValue(Hash, Pair.RegionAMaximumResidualCm);
		HashValue(Hash, Pair.RegionBRootMeanSquareResidualCm);
		HashValue(Hash, Pair.RegionBMaximumResidualCm);
		HashValue(Hash, Pair.BalancedRootMeanSquareResidualCm);
		HashValue(Hash, Pair.MaximumEndpointNormalAdjustmentDegrees);
		HashValue(Hash, Pair.bBothBoundaryEvidenceUsable);
		HashValue(Hash, Pair.bPlausible);
	}
	const int32 SharedC2FrameLedgerCount = SharedC2FrameLedger.Num();
	HashValue(Hash, SharedC2FrameLedgerCount);
	for (const FSharedC2FrameLedgerEntry& Entry : SharedC2FrameLedger)
	{
		HashValue(Hash, Entry.SharedPairFitIndex);
		HashValue(Hash, Entry.RegularizedAxisA);
		HashValue(Hash, Entry.RegularizedAxisB);
		HashValue(Hash, Entry.MaximumExtrusionAxisAdjustmentDegrees);
		HashValue(Hash, Entry.MaximumPlaneNormalAdjustmentDegrees);
		HashValue(Hash, Entry.PlanarSourceRootMeanSquareResidualCm);
		HashValue(Hash, Entry.PlanarSourceMaximumResidualCm);
		HashValue(Hash, Entry.MaximumOriginalPlaneEndpointResidualCm);
		HashValue(Hash, Entry.MaximumConstructedEndpointPositionResidualCm);
		HashValue(Hash, Entry.MaximumEndpointTangentDirectionResidual);
		HashValue(Hash, Entry.MinimumEndpointTangentMagnitude);
		HashValue(Hash, Entry.MaximumEndpointSecondDerivativeMagnitude);
		HashValue(Hash, Entry.bC0ByConstruction);
		HashValue(Hash, Entry.bG1ByConstruction);
		HashValue(Hash, Entry.bG2ByConstruction);
		HashValue(Hash, Entry.bParametricC2ByConstruction);
		HashValue(Hash, Entry.bSourceRegularizationPlausible);
	}
	const int32 GlobalC2PlaneConstraintCount = GlobalC2PlaneConstraints.Num();
	HashValue(Hash, GlobalC2PlaneConstraintCount);
	for (const FGlobalC2PlaneConstraint& Constraint : GlobalC2PlaneConstraints)
	{
		HashValue(Hash, Constraint.PlanarGroupIndex);
		HashValue(Hash, Constraint.RequestCount);
		HashValue(Hash, Constraint.Normal);
		HashValue(Hash, Constraint.PlaneOffset);
		HashValue(Hash, Constraint.MaximumRequestedNormalConflictDegrees);
		HashValue(Hash, Constraint.MaximumRequestedEndpointResidualCm);
		HashValue(Hash, Constraint.SourceRootMeanSquareResidualCm);
		HashValue(Hash, Constraint.SourceMaximumResidualCm);
		HashValue(Hash, Constraint.SourceNormalAdjustmentDegrees);
		HashValue(Hash, Constraint.bConstraintCompatible);
	}
	const int32 GlobalC2JoinCertificateCount = GlobalC2JoinCertificates.Num();
	HashValue(Hash, GlobalC2JoinCertificateCount);
	for (const FGlobalC2JoinCertificate& Certificate : GlobalC2JoinCertificates)
	{
		HashValue(Hash, Certificate.SharedPairFitIndex);
		HashValue(Hash, Certificate.BoundaryCount);
		HashValue(Hash, Certificate.MaximumEndpointPlaneResidualCm);
		HashValue(Hash, Certificate.MaximumTangentNormalResidualDegrees);
		HashValue(Hash, Certificate.bNetworkC0Plausible);
		HashValue(Hash, Certificate.bNetworkG1Plausible);
		HashValue(Hash, Certificate.bLocalC2Retained);
	}
	const int32 SymmetrizedC2PlaneConstraintCount =
		SymmetrizedC2PlaneConstraints.Num();
	HashValue(Hash, C2SymmetryPlacementFrame.BoundsCenter);
	HashValue(Hash, C2SymmetryPlacementFrame.SourceInferredCenter);
	HashValue(Hash, C2SymmetryPlacementFrame.TargetCenter);
	HashValue(Hash, C2SymmetryPlacementFrame.XConstraintCount);
	HashValue(Hash, C2SymmetryPlacementFrame.YConstraintCount);
	HashValue(Hash, C2SymmetryPlacementFrame.XRootMeanSquareResidualCm);
	HashValue(Hash, C2SymmetryPlacementFrame.XMaximumResidualCm);
	HashValue(Hash, C2SymmetryPlacementFrame.YRootMeanSquareResidualCm);
	HashValue(Hash, C2SymmetryPlacementFrame.YMaximumResidualCm);
	HashValue(Hash, C2SymmetryPlacementFrame.bFittedFromMirrorPlanes);
	HashValue(Hash, SymmetrizedC2PlaneConstraintCount);
	for (const FSymmetrizedC2PlaneConstraint& Constraint :
		SymmetrizedC2PlaneConstraints)
	{
		HashValue(Hash, Constraint.OrbitId);
		HashValue(Hash, Constraint.PlanarGroupIndex);
		HashValue(Hash, Constraint.TransformMaskFromOrbitRoot);
		HashValue(Hash, Constraint.Normal);
		HashValue(Hash, Constraint.PlaneOffset);
		HashValue(Hash, Constraint.SourceNormalAdjustmentDegrees);
		HashValue(Hash, Constraint.SourceRootMeanSquareResidualCm);
		HashValue(Hash, Constraint.SourceMaximumResidualCm);
		HashValue(Hash, Constraint.MaximumMirrorNormalResidualDegrees);
		HashValue(Hash, Constraint.MaximumMirrorOffsetResidualCm);
		HashValue(Hash, Constraint.bSourceFitPlausible);
		HashValue(Hash, Constraint.bExactMirrorPlacement);
	}
	const int32 CoupledC2TransitionSolutionCount =
		CoupledC2TransitionSolutions.Num();
	HashValue(Hash, CoupledC2TransitionSolutionCount);
	for (const FCoupledC2TransitionPairSolution& Solution :
		CoupledC2TransitionSolutions)
	{
		HashValue(Hash, Solution.SharedPairFitIndex);
		HashValue(Hash, Solution.RegionAPlaneUIndex);
		HashValue(Hash, Solution.RegionAPlaneVIndex);
		HashValue(Hash, Solution.RegionBPlaneUIndex);
		HashValue(Hash, Solution.RegionBPlaneVIndex);
		HashValue(Hash, Solution.RegionACenterCoordinates);
		HashValue(Hash, Solution.RegionBCenterCoordinates);
		HashValue(Hash, Solution.SharedRadiusU);
		HashValue(Hash, Solution.SharedRadiusV);
		HashValue(Hash, Solution.SharedFlatteningFraction);
		HashValue(Hash, Solution.RegionARootMeanSquareResidualCm);
		HashValue(Hash, Solution.RegionAMaximumResidualCm);
		HashValue(Hash, Solution.RegionBRootMeanSquareResidualCm);
		HashValue(Hash, Solution.RegionBMaximumResidualCm);
		HashValue(Hash, Solution.BalancedRootMeanSquareResidualCm);
		HashValue(Hash, Solution.MaximumPlaneOrthogonalityResidual);
		HashValue(Hash, Solution.MaximumEndpointPlaneResidualCm);
		HashValue(Hash, Solution.MaximumEndpointTangentPlaneResidual);
		HashValue(Hash, Solution.MaximumEndpointSecondDerivativeMagnitude);
		HashValue(Hash, Solution.bExactC0G1C2ByConstruction);
		HashValue(Hash, Solution.bSourceFitPlausible);
	}
	const int32 CoupledC2TransitionFamilyCount =
		CoupledC2TransitionFamilies.Num();
	HashValue(Hash, CoupledC2TransitionFamilyCount);
	for (const FCoupledC2TransitionFamilySolution& Family :
		CoupledC2TransitionFamilies)
	{
		HashValue(Hash, Family.FamilyId);
		HashValue(Hash, Family.FirstRegionIndex);
		HashValue(Hash, Family.RegionCount);
		HashValue(Hash, Family.PairConstraintCount);
		HashValue(Hash, Family.SymmetryAxisMask);
		HashValue(Hash, Family.SharedRadiusU);
		HashValue(Hash, Family.SharedRadiusV);
		HashValue(Hash, Family.SharedFlatteningFraction);
		HashValue(Hash, Family.BalancedRootMeanSquareResidualCm);
		HashValue(Hash, Family.MaximumRegionRootMeanSquareResidualCm);
		HashValue(Hash, Family.MaximumResidualCm);
		HashValue(Hash, Family.MaximumEndpointPlaneResidualCm);
		HashValue(Hash, Family.MaximumEndpointTangentPlaneResidual);
		HashValue(Hash, Family.MaximumEndpointSecondDerivativeMagnitude);
		HashValue(Hash, Family.bExactC0G1C2ByConstruction);
		HashValue(Hash, Family.bSourceFitPlausible);
	}
	const int32 CoupledC2FamilyRegionIndexCount =
		CoupledC2FamilyRegionIndices.Num();
	HashValue(Hash, CoupledC2FamilyRegionIndexCount);
	for (const int32 RegionIndex : CoupledC2FamilyRegionIndices)
	{
		HashValue(Hash, RegionIndex);
	}
	const int32 C2TransitionCoverageCount = C2TransitionCoverage.Num();
	HashValue(Hash, C2TransitionCoverageCount);
	for (const FC2TransitionCoverageEntry& Entry : C2TransitionCoverage)
	{
		HashValue(Hash, Entry.TransitionFitIndex);
		HashValue(Hash, Entry.ExtrusionRegionIndex);
		HashValue(Hash, Entry.FamilyIndex);
		HashValue(Hash, Entry.XMirrorTargetRegionIndex);
		HashValue(Hash, Entry.YMirrorTargetRegionIndex);
		HashValue(Hash, Entry.Kind);
		HashValue(Hash, Entry.SurfaceLayer);
		HashValue(Hash, Entry.HorizontalPlanarGroupIndex);
		HashValue(Hash, Entry.HorizontalPlaneHeightCm);
		HashValue(Hash, Entry.MaximumBoundaryPositionResidualCm);
		HashValue(Hash, Entry.MaximumBoundaryNormalResidualDegrees);
		HashValue(Hash, Entry.bXMirrorPlausible);
		HashValue(Hash, Entry.bYMirrorPlausible);
		HashValue(Hash, Entry.bXSelfMirror);
		HashValue(Hash, Entry.bYSelfMirror);
	}
	const int32 PlayableC2OrbitCandidateCount = PlayableC2OrbitCandidates.Num();
	HashValue(Hash, PlayableC2OrbitCandidateCount);
	for (const FPlayableC2OrbitCandidate& Candidate : PlayableC2OrbitCandidates)
	{
		HashValue(Hash, Candidate.OrbitId);
		HashValue(Hash, Candidate.SeedCoverageIndex);
		HashValue(Hash, Candidate.FirstMemberIndex);
		HashValue(Hash, Candidate.MemberCount);
		HashValue(Hash, Candidate.ExpectedMemberCount);
		HashValue(Hash, Candidate.SymmetryAxisMask);
		HashValue(Hash, Candidate.SharedRadiusU);
		HashValue(Hash, Candidate.SharedRadiusV);
		HashValue(Hash, Candidate.SharedFlatteningFraction);
		HashValue(Hash, Candidate.BalancedRootMeanSquareResidualCm);
		HashValue(Hash, Candidate.MaximumRegionRootMeanSquareResidualCm);
		HashValue(Hash, Candidate.MaximumResidualCm);
		HashValue(Hash, Candidate.MaximumMirrorAxisResidualDegrees);
		HashValue(Hash, Candidate.MaximumMirrorCenterLineResidualCm);
		HashValue(Hash, Candidate.MaximumMirrorBoundsResidualCm);
		HashValue(Hash, Candidate.MaximumMirrorRelativeAreaResidual);
		HashValue(Hash, Candidate.MaximumEndpointPlaneResidualCm);
		HashValue(Hash, Candidate.MaximumEndpointTangentPlaneResidual);
		HashValue(Hash, Candidate.MaximumEndpointSecondDerivativeMagnitude);
		HashValue(Hash, Candidate.bOrbitComplete);
		HashValue(Hash, Candidate.bMirrorTopologyPlausible);
		HashValue(Hash, Candidate.bExactC0G1C2ByConstruction);
		HashValue(Hash, Candidate.bSourceFitPlausible);
	}
	const int32 PlayableC2OrbitMemberCount = PlayableC2OrbitMembers.Num();
	HashValue(Hash, PlayableC2OrbitMemberCount);
	for (const FPlayableC2OrbitMember& Member : PlayableC2OrbitMembers)
	{
		HashValue(Hash, Member.ExtrusionRegionIndex);
		HashValue(Hash, Member.TransformMaskFromSeed);
		HashValue(Hash, Member.RootMeanSquareResidualCm);
		HashValue(Hash, Member.MaximumResidualCm);
	}
	const int32 PlayableC2TriangleSupportCandidateCount =
		PlayableC2TriangleSupportCandidates.Num();
	HashValue(Hash, PlayableC2TriangleSupportCandidateCount);
	for (const FPlayableC2TriangleSupportCandidate& Candidate :
		PlayableC2TriangleSupportCandidates)
	{
		HashValue(Hash, Candidate.OrbitId);
		HashValue(Hash, Candidate.SeedCoverageIndex);
		HashValue(Hash, Candidate.SmoothRegionIndex);
		HashValue(Hash, Candidate.FirstMemberIndex);
		HashValue(Hash, Candidate.MemberCount);
		HashValue(Hash, Candidate.ExpectedMemberCount);
		HashValue(Hash,
			Candidate.MaximumMemberRootMeanSquarePositionResidualCm);
		HashValue(Hash, Candidate.MaximumPositionResidualCm);
		HashValue(Hash,
			Candidate.MaximumMemberRootMeanSquareNormalResidualDegrees);
		HashValue(Hash, Candidate.MaximumNormalResidualDegrees);
		HashValue(Hash, Candidate.SharedRadiusU);
		HashValue(Hash, Candidate.SharedRadiusV);
		HashValue(Hash, Candidate.SharedFlatteningFraction);
		HashValue(Hash, Candidate.BalancedProfileRootMeanSquareResidualCm);
		HashValue(Hash,
			Candidate.MaximumMemberProfileRootMeanSquareResidualCm);
		HashValue(Hash, Candidate.MaximumProfileResidualCm);
		HashValue(Hash, Candidate.MaximumEndpointPlaneResidualCm);
		HashValue(Hash, Candidate.MaximumEndpointTangentPlaneResidual);
		HashValue(Hash, Candidate.MaximumEndpointSecondDerivativeMagnitude);
		HashValue(Hash, Candidate.bComplete);
		HashValue(Hash, Candidate.bSupportPlausible);
		HashValue(Hash, Candidate.bExactC0G1C2ByConstruction);
		HashValue(Hash, Candidate.bSourceFitPlausible);
		HashValue(Hash, Candidate.NetworkSharedRadiusU);
		HashValue(Hash, Candidate.NetworkSharedRadiusV);
		HashValue(Hash, Candidate.NetworkSharedFlatteningFraction);
		HashValue(Hash,
			Candidate.NetworkBalancedProfileRootMeanSquareResidualCm);
		HashValue(Hash,
			Candidate.NetworkMaximumMemberProfileRootMeanSquareResidualCm);
		HashValue(Hash, Candidate.NetworkMaximumProfileResidualCm);
		HashValue(Hash, Candidate.NetworkMaximumEndpointPlaneResidualCm);
		HashValue(Hash, Candidate.NetworkMaximumEndpointTangentPlaneResidual);
		HashValue(Hash,
			Candidate.NetworkMaximumEndpointSecondDerivativeMagnitude);
		HashValue(Hash, Candidate.bNetworkExactC0G1C2ByConstruction);
		HashValue(Hash, Candidate.bNetworkSourceFitPlausible);
	}
	const int32 PlayableC2TriangleSupportMemberCount =
		PlayableC2TriangleSupportMembers.Num();
	HashValue(Hash, PlayableC2TriangleSupportMemberCount);
	for (const FPlayableC2TriangleSupportMember& Member :
		PlayableC2TriangleSupportMembers)
	{
		HashValue(Hash, Member.TransformMaskFromSeed);
		HashValue(Hash, Member.FirstCanonicalPointIndex);
		HashValue(Hash, Member.CanonicalPointCount);
		HashValue(Hash, Member.SampleCount);
		HashValue(Hash, Member.MatchedSampleCount);
		HashValue(Hash, Member.UniqueTargetTriangleCount);
		HashValue(Hash, Member.RootMeanSquarePositionResidualCm);
		HashValue(Hash, Member.MaximumPositionResidualCm);
		HashValue(Hash, Member.RootMeanSquareNormalResidualDegrees);
		HashValue(Hash, Member.MaximumNormalResidualDegrees);
		HashValue(Hash, Member.ProfileRootMeanSquareResidualCm);
		HashValue(Hash, Member.ProfileMaximumResidualCm);
		HashValue(Hash, Member.bComplete);
	}
	const int32 PlayableC2CanonicalSupportPointCount =
		PlayableC2CanonicalSupportPoints.Num();
	HashValue(Hash, PlayableC2CanonicalSupportPointCount);
	for (const FVector3d& Point : PlayableC2CanonicalSupportPoints)
	{
		HashValue(Hash, Point);
	}
	const int32 PlayableC2PlaneBindingCount = PlayableC2PlaneBindings.Num();
	HashValue(Hash, PlayableC2PlaneBindingCount);
	for (const FPlayableC2PlaneBinding& Binding : PlayableC2PlaneBindings)
	{
		HashValue(Hash, Binding.TriangleSupportCandidateIndex);
		HashValue(Hash, Binding.Endpoint);
		HashValue(Hash, Binding.SourcePlanarGroupIndex);
		HashValue(Hash, Binding.SymmetrizedConstraintIndex);
		HashValue(Hash, Binding.EndpointPosition);
		HashValue(Hash, Binding.NormalResidualDegrees);
		HashValue(Hash, Binding.EndpointPlaneResidualCm);
		HashValue(Hash, Binding.OrientedOffsetResidualCm);
		HashValue(Hash, Binding.bSameSourcePlanarGroup);
		HashValue(Hash, Binding.bCompatible);
		HashValue(Hash, Binding.NetworkPlaneConstraintIndex);
		HashValue(Hash, Binding.NetworkNormalResidualDegrees);
		HashValue(Hash, Binding.NetworkEndpointPlaneResidualCm);
		HashValue(Hash, Binding.NetworkOrientedOffsetResidualCm);
		HashValue(Hash, Binding.bNetworkCompatible);
	}
	const int32 PlayableC2NetworkPlaneConstraintCount =
		PlayableC2NetworkPlaneConstraints.Num();
	HashValue(Hash, PlayableC2NetworkPlaneConstraintCount);
	for (const FPlayableC2NetworkPlaneConstraint& Constraint :
		PlayableC2NetworkPlaneConstraints)
	{
		HashValue(Hash, Constraint.OrbitId);
		HashValue(Hash, Constraint.SourcePlanarGroupIndex);
		HashValue(Hash, Constraint.TransformMaskFromOrbitRoot);
		HashValue(Hash, Constraint.Normal);
		HashValue(Hash, Constraint.PlaneOffset);
		HashValue(Hash, Constraint.SourceObservationCount);
		HashValue(Hash, Constraint.MaximumSourceNormalResidualDegrees);
		HashValue(Hash, Constraint.MaximumSourceOffsetResidualCm);
		HashValue(Hash, Constraint.bInheritedFromSymmetrizedNetwork);
		HashValue(Hash, Constraint.bSourceFitPlausible);
		HashValue(Hash, Constraint.bExactMirrorPlacement);
	}
	const int32 EndWallBoundaryComponentCount = EndWallBoundaryComponents.Num();
	HashValue(Hash, EndWallBoundaryComponentCount);
	for (const FEndWallBoundaryComponent& Component : EndWallBoundaryComponents)
	{
		HashValue(Hash, Component.ComponentId);
		HashValue(Hash, Component.NetworkPlaneConstraintIndex);
		HashValue(Hash, Component.PlanarGroupIndex);
		HashValue(Hash, Component.FirstEdgeIndex);
		HashValue(Hash, Component.EdgeCount);
		HashValue(Hash, Component.UniqueVertexCount);
		HashValue(Hash, Component.AdjacentTriangleCount);
		HashValue(Hash, Component.PerimeterCm);
		HashValue(Hash, Component.Bounds.Min);
		HashValue(Hash, Component.Bounds.Max);
		HashValue(Hash, Component.SurfaceLayer);
		HashValue(Hash, Component.bClosedLoop);
		HashValue(Hash, Component.bCrossesXSymmetryPlane);
	}
	const int32 EndWallBoundaryEdgeIndexCount = EndWallBoundaryEdgeIndices.Num();
	HashValue(Hash, EndWallBoundaryEdgeIndexCount);
	for (const int32 EdgeIndex : EndWallBoundaryEdgeIndices)
	{
		HashValue(Hash, EdgeIndex);
	}
	const int32 OpenRimCandidateCount = OpenRimCandidates.Num();
	HashValue(Hash, OpenRimCandidateCount);
	for (const FOpenRimCandidate& Candidate : OpenRimCandidates)
	{
		HashValue(Hash, Candidate.CandidateId);
		HashValue(Hash, Candidate.WallBoundaryComponentIndex);
		HashValue(Hash, Candidate.FirstEdgeIndex);
		HashValue(Hash, Candidate.EdgeCount);
		HashValue(Hash, Candidate.UniqueVertexCount);
		HashValue(Hash, Candidate.ArcLengthCm);
		HashValue(Hash, Candidate.Bounds.Min);
		HashValue(Hash, Candidate.Bounds.Max);
		HashValue(Hash, Candidate.BaselineHeightCm);
		HashValue(Hash, Candidate.WallAxis);
		HashValue(Hash, Candidate.SurfaceLayer);
		HashValue(Hash, Candidate.LongitudinalMirrorCandidateIndex);
		HashValue(Hash,
			Candidate.LongitudinalMirrorRootMeanSquareResidualCm);
		HashValue(Hash, Candidate.LongitudinalMirrorMaximumResidualCm);
		HashValue(Hash, Candidate.TransverseSelfRootMeanSquareResidualCm);
		HashValue(Hash, Candidate.TransverseSelfMaximumResidualCm);
		HashValue(Hash, Candidate.NegativeVerticalEdgeCount);
		HashValue(Hash, Candidate.PositiveVerticalEdgeCount);
		HashValue(Hash, Candidate.HorizontalSpanEdgeCount);
		HashValue(Hash, Candidate.UpperTransitionEdgeCount);
		HashValue(Hash, Candidate.HorizontalSpanHeightCm);
		HashValue(Hash, Candidate.HorizontalSpanSpanCm);
		HashValue(Hash, Candidate.NegativeVerticalHeightCm);
		HashValue(Hash, Candidate.PositiveVerticalHeightCm);
		HashValue(Hash, Candidate.bOpenAtBaseline);
		HashValue(Hash, Candidate.bCrossesTransverseSymmetryPlane);
		HashValue(Hash, Candidate.bLongitudinalMirrorPlausible);
		HashValue(Hash, Candidate.bTransverseSelfPlausible);
		HashValue(Hash, Candidate.bFeaturePartitionComplete);
	}
	const int32 OpenRimEdgeIndexCount = OpenRimEdgeIndices.Num();
	HashValue(Hash, OpenRimEdgeIndexCount);
	for (const int32 EdgeIndex : OpenRimEdgeIndices)
	{
		HashValue(Hash, EdgeIndex);
	}
	const int32 CanonicalOpenArchSolutionCount = CanonicalOpenArchSolutions.Num();
	HashValue(Hash, CanonicalOpenArchSolutionCount);
	for (const FCanonicalOpenArchSolution& Solution : CanonicalOpenArchSolutions)
	{
		HashValue(Hash, Solution.SolutionId);
		HashValue(Hash, Solution.FirstMemberIndex);
		HashValue(Hash, Solution.MemberCount);
		HashValue(Hash, Solution.BaselineHeightCm);
		HashValue(Hash, Solution.VerticalSegmentHalfWidthCm);
		HashValue(Hash, Solution.VerticalSegmentTransitionHeightCm);
		HashValue(Hash, Solution.HorizontalSpanHalfSpanCm);
		HashValue(Hash, Solution.HorizontalSpanHeightCm);
		HashValue(Hash, Solution.FlatteningFraction);
		HashValue(Hash, Solution.BalancedRootMeanSquareResidualCm);
		HashValue(Hash, Solution.MaximumMemberRootMeanSquareResidualCm);
		HashValue(Hash, Solution.MaximumResidualCm);
		HashValue(Hash, Solution.MaximumEndpointPositionResidualCm);
		HashValue(Hash, Solution.MaximumEndpointTangentResidualDegrees);
		HashValue(Hash, Solution.MaximumEndpointSecondDerivativeMagnitude);
		HashValue(Hash, Solution.bCompleteFourMemberOrbit);
		HashValue(Hash, Solution.bExactXYMirrorPlacement);
		HashValue(Hash, Solution.bExactC0G1C2ByConstruction);
		HashValue(Hash, Solution.bSourceRegularizationPlausible);
	}
	const int32 CanonicalOpenArchMemberFitCount =
		CanonicalOpenArchMemberFits.Num();
	HashValue(Hash, CanonicalOpenArchMemberFitCount);
	for (const FCanonicalOpenArchMemberFit& Fit : CanonicalOpenArchMemberFits)
	{
		HashValue(Hash, Fit.OpenRimCandidateIndex);
		HashValue(Hash, Fit.TransverseSide);
		HashValue(Hash, Fit.SampleCount);
		HashValue(Hash, Fit.RootMeanSquareResidualCm);
		HashValue(Hash, Fit.MaximumResidualCm);
	}
	const int32 OpenRimSurfaceBandObservationCount =
		OpenRimSurfaceBandObservations.Num();
	HashValue(Hash, OpenRimSurfaceBandObservationCount);
	for (const FOpenRimSurfaceBandObservation& Observation :
		OpenRimSurfaceBandObservations)
	{
		HashValue(Hash, Observation.OpenRimCandidateIndex);
		HashValue(Hash, Observation.RimEdgeIndex);
		HashValue(Hash, Observation.BoundaryPlaneTriangleIndex);
		HashValue(Hash, Observation.OpeningSurfaceTriangleIndex);
		HashValue(Hash, Observation.Feature);
		HashValue(Hash, Observation.EdgeLengthCm);
		HashValue(Hash, Observation.FaceDihedralDegrees);
		HashValue(Hash, Observation.ImportedNormalJumpDegrees);
		HashValue(Hash, Observation.FirstLongitudinalStepCm);
		HashValue(Hash, Observation.FirstStepAxisResidualDegrees);
		HashValue(Hash, Observation.bSameSmoothRegion);
		HashValue(Hash, Observation.bSourceG1Plausible);
	}
	const int32 OpenRimTransverseSectionCount =
		OpenRimTransverseSections.Num();
	HashValue(Hash, OpenRimTransverseSectionCount);
	for (const FOpenRimTransverseSection& Section :
		OpenRimTransverseSections)
	{
		HashValue(Hash, Section.SectionId);
		HashValue(Hash, Section.OpenRimCandidateIndex);
		HashValue(Hash, Section.RimEdgeIndex);
		HashValue(Hash, Section.InterpolationRimEdgeA);
		HashValue(Hash, Section.InterpolationRimEdgeB);
		HashValue(Hash, Section.InterpolationAlpha);
		HashValue(Hash, Section.Feature);
		HashValue(Hash, Section.SliceCoordinateCm);
		HashValue(Hash, Section.SliceOrigin);
		HashValue(Hash, Section.SlicePlaneNormal);
		HashValue(Hash, Section.OpeningDirection);
		HashValue(Hash, Section.CanonicalRimParameter);
		HashValue(Hash, Section.TopologyCanonicalRimParameter);
		HashValue(Hash, Section.FirstPointIndex);
		HashValue(Hash, Section.PointCount);
		HashValue(Hash, Section.FirstTransitionPointIndex);
		HashValue(Hash, Section.TransitionPointCount);
		HashValue(Hash, Section.FirstLongitudinalContinuationPointIndex);
		HashValue(Hash, Section.LongitudinalContinuationPointCount);
		HashValue(Hash, Section.FirstLongitudinalSampleIndex);
		HashValue(Hash, Section.FirstLongitudinalSplineSegmentIndex);
		HashValue(Hash, Section.LongitudinalSplineSegmentCount);
		HashValue(Hash, Section.LongitudinalTubePointCount);
		HashValue(Hash, Section.FirstLongitudinalRearTurnPointOffset);
		HashValue(Hash, Section.LongitudinalContinuationArcLengthCm);
		HashValue(Hash, Section.LongitudinalTubeArcLengthCm);
		HashValue(Hash, Section.LongitudinalTubeTerminalDepthCm);
		HashValue(Hash, Section.LongitudinalTubeTerminalOpeningOffsetCm);
		HashValue(Hash, Section.LongitudinalTubeMaximumDepartureDegrees);
		HashValue(Hash, Section.LongitudinalSplineRootMeanSquareResidualCm);
		HashValue(Hash, Section.LongitudinalSplineMaximumResidualCm);
		HashValue(Hash, Section.LongitudinalContinuationMaximumTurnDegrees);
		HashValue(Hash, Section.LongitudinalContinuationTerminalDepthCm);
		HashValue(Hash, Section.LongitudinalContinuationTerminalOpeningOffsetCm);
		HashValue(Hash, Section.TransitionSegmentCount);
		HashValue(Hash, Section.SegmentCount);
		HashValue(Hash, Section.ArcLengthCm);
		HashValue(Hash, Section.RimDistanceCm);
		HashValue(Hash, Section.MaximumLongitudinalDepthCm);
		HashValue(Hash, Section.MaximumOpeningOffsetCm);
		HashValue(Hash, Section.BoundaryPlaneTangentResidualDegrees);
		HashValue(Hash, Section.LongitudinalTangentResidualDegrees);
		HashValue(Hash, Section.LongitudinalTangentDepthCm);
		HashValue(Hash, Section.LongitudinalTangentOpeningOffsetCm);
		HashValue(Hash, Section.LongitudinalTransitionHop);
		HashValue(Hash, Section.Bounds.Min);
		HashValue(Hash, Section.Bounds.Max);
		HashValue(Hash, Section.bConnectedToRim);
		HashValue(Hash, Section.bSourceBoundaryPlaneG1Plausible);
		HashValue(Hash, Section.bLongitudinalRunPlausible);
		HashValue(Hash, Section.bTopologyCanonicalRimParameterValid);
		HashValue(Hash, Section.bLongitudinalSplineC2);
		HashValue(Hash, Section.IndividualC2BoundaryPlaneTangentMagnitudeCm);
		HashValue(Hash, Section.IndividualC2LongitudinalTangentMagnitudeCm);
		HashValue(Hash, Section.IndividualC2RootMeanSquareResidualCm);
		HashValue(Hash, Section.IndividualC2MaximumResidualCm);
		HashValue(Hash, Section.bIndividualC2FitValid);
	}
	const int32 OpenRimTransverseSectionPointCount =
		OpenRimTransverseSectionPoints.Num();
	HashValue(Hash, OpenRimTransverseSectionPointCount);
	for (const FVector2d& Point : OpenRimTransverseSectionPoints)
		HashValue(Hash, Point);
	const int32 OpenRimTransitionSectionPointCount =
		OpenRimTransitionSectionPoints.Num();
	HashValue(Hash, OpenRimTransitionSectionPointCount);
	for (const FVector2d& Point : OpenRimTransitionSectionPoints)
		HashValue(Hash, Point);
	const int32 OpenRimLongitudinalContinuationPointCount =
		OpenRimLongitudinalContinuationPoints.Num();
	HashValue(Hash, OpenRimLongitudinalContinuationPointCount);
	for (const FVector2d& Point : OpenRimLongitudinalContinuationPoints)
		HashValue(Hash, Point);
	const int32 OpenRimLongitudinalSampleCount = OpenRimLongitudinalSamples.Num();
	HashValue(Hash, OpenRimLongitudinalSampleCount);
	for (const FOpenRimLongitudinalSample& Sample : OpenRimLongitudinalSamples)
	{
		HashValue(Hash, Sample.OpenRimTransverseSectionIndex);
		HashValue(Hash, Sample.ContinuationPointOffset);
		HashValue(Hash, Sample.NormalizedArcLength);
		HashValue(Hash, Sample.CanonicalPositionCm);
	}
	const int32 OpenRimLongitudinalSplineSegmentCount =
		OpenRimLongitudinalSplineSegments.Num();
	HashValue(Hash, OpenRimLongitudinalSplineSegmentCount);
	for (const FOpenRimLongitudinalSplineSegment& Segment :
		OpenRimLongitudinalSplineSegments)
	{
		HashValue(Hash, Segment.OpenRimTransverseSectionIndex);
		HashValue(Hash, Segment.MinimumTubeParameter);
		HashValue(Hash, Segment.MaximumTubeParameter);
		for (const FVector3d& ControlPoint : Segment.ControlPoints)
			HashValue(Hash, ControlPoint);
	}
	const int32 OpenRimTerminalClosureRailSegmentCount =
		OpenRimTerminalClosureRailSegments.Num();
	HashValue(Hash, OpenRimTerminalClosureRailSegmentCount);
	for (const FOpenRimTerminalClosureRailSegment& Segment :
		OpenRimTerminalClosureRailSegments)
	{
		HashValue(Hash, Segment.OpenRimTransverseSectionIndex);
		HashValue(Hash, Segment.MinimumClosureParameter);
		HashValue(Hash, Segment.MaximumClosureParameter);
		HashValue(Hash, Segment.Degree);
		for (const FVector3d& ControlPoint : Segment.ControlPoints)
			HashValue(Hash, ControlPoint);
	}
	const int32 OpenRimTerminalClosureRailFitCount =
		OpenRimTerminalClosureRailFits.Num();
	HashValue(Hash, OpenRimTerminalClosureRailFitCount);
	for (const FOpenRimTerminalClosureRailFit& Fit :
		OpenRimTerminalClosureRailFits)
	{
		HashValue(Hash, Fit.OpenRimTransverseSectionIndex);
		HashValue(Hash, Fit.FirstSegmentIndex);
		HashValue(Hash, Fit.SegmentCount);
		HashValue(Hash, Fit.SourceSampleCount);
		HashValue(Hash, Fit.ClosureToPrefixParameterScale);
		HashValue(Hash, Fit.MaximumSourceResidualCm);
		HashValue(Hash, Fit.MaximumPolylineResidualCm);
		HashValue(Hash, Fit.PositionJoinResidualCm);
		HashValue(Hash, Fit.FirstDerivativeJoinResidualCm);
		HashValue(Hash, Fit.SecondDerivativeJoinResidualCm);
		HashValue(Hash, Fit.bC2PrefixJoinByConstruction);
		HashValue(Hash, Fit.bRegularFinite);
	}
	const int32 OpenRimTransitionFamilyFitCount =
		OpenRimTransitionFamilyFits.Num();
	HashValue(Hash, OpenRimTransitionFamilyFitCount);
	for (const FOpenRimTransitionFamilyFit& Fit :
		OpenRimTransitionFamilyFits)
	{
		HashValue(Hash, Fit.FitId);
		HashValue(Hash, Fit.Family);
		HashValue(Hash, Fit.FirstMemberFitIndex);
		HashValue(Hash, Fit.MemberCount);
		HashValue(Hash, Fit.SampleCount);
		HashValue(Hash, Fit.EndDepthCm);
		HashValue(Hash, Fit.EndOpeningOffsetCm);
		HashValue(Hash, Fit.BoundaryPlaneTangentMagnitudeCm);
		HashValue(Hash, Fit.LongitudinalTangentMagnitudeCm);
		HashValue(Hash, Fit.BalancedRootMeanSquareResidualCm);
		HashValue(Hash, Fit.MaximumMemberRootMeanSquareResidualCm);
		HashValue(Hash, Fit.MaximumResidualCm);
		HashValue(Hash, Fit.bExactC0G1C2ByConstruction);
	}
	const int32 OpenRimTransitionMemberFitCount =
		OpenRimTransitionMemberFits.Num();
	HashValue(Hash, OpenRimTransitionMemberFitCount);
	for (const FOpenRimTransitionMemberFit& Fit :
		OpenRimTransitionMemberFits)
	{
		HashValue(Hash, Fit.OpenRimTransitionFamilyFitIndex);
		HashValue(Hash, Fit.OpenRimTransverseSectionIndex);
		HashValue(Hash, Fit.SampleCount);
		HashValue(Hash, Fit.RootMeanSquareResidualCm);
		HashValue(Hash, Fit.MaximumResidualCm);
	}
	const int32 OpenRimC2LoftStationCount = OpenRimC2LoftStations.Num();
	HashValue(Hash, OpenRimC2LoftStationCount);
	for (const FOpenRimC2LoftStation& Station : OpenRimC2LoftStations)
	{
		HashValue(Hash, Station.StationId);
		HashValue(Hash, Station.Family);
		HashValue(Hash, Station.CanonicalRimParameter);
		HashValue(Hash, Station.MemberCount);
		HashValue(Hash, Station.MeanEndDepthCm);
		HashValue(Hash, Station.MeanEndOpeningOffsetCm);
		HashValue(Hash, Station.MeanBoundaryPlaneTangentCm);
		HashValue(Hash, Station.MeanLongitudinalTangentCm);
		HashValue(Hash, Station.MedianEndDepthCm);
		HashValue(Hash, Station.MedianEndOpeningOffsetCm);
		HashValue(Hash, Station.MedianBoundaryPlaneTangentCm);
		HashValue(Hash, Station.MedianLongitudinalTangentCm);
		HashValue(Hash, Station.GeometricFitEndDepthCm);
		HashValue(Hash, Station.GeometricFitEndOpeningOffsetCm);
		HashValue(Hash, Station.GeometricFitBoundaryPlaneTangentCm);
		HashValue(Hash, Station.GeometricFitLongitudinalTangentCm);
		HashValue(Hash, Station.GeometricFitRootMeanSquareResidualCm);
		HashValue(Hash, Station.GeometricFitMaximumResidualCm);
		HashValue(Hash, Station.RootMeanSquareParameterSpreadCm);
		HashValue(Hash, Station.MaximumParameterSpreadCm);
		HashValue(Hash, Station.bHasBothOpenings);
		HashValue(Hash, Station.bHasBothSides);
		HashValue(Hash, Station.bGeometricFitValid);
	}
	const int32 OpenRimC2LoftSegmentCount = OpenRimC2LoftSegments.Num();
	HashValue(Hash, OpenRimC2LoftSegmentCount);
	for (const FOpenRimC2LoftSegment& Segment : OpenRimC2LoftSegments)
	{
		HashValue(Hash, Segment.StartCanonicalRimParameter);
		HashValue(Hash, Segment.EndCanonicalRimParameter);
		for (int32 ParameterIndex = 0; ParameterIndex < 4; ++ParameterIndex)
		{
			HashValue(Hash, Segment.StartValuesCm[ParameterIndex]);
			HashValue(Hash, Segment.EndValuesCm[ParameterIndex]);
			HashValue(Hash, Segment.StartDerivativesCm[ParameterIndex]);
			HashValue(Hash, Segment.EndDerivativesCm[ParameterIndex]);
		}
	}
	const int32 OpenRimC2LoftFitCount = OpenRimC2LoftFits.Num();
	HashValue(Hash, OpenRimC2LoftFitCount);
	for (const FOpenRimC2LoftFit& Fit : OpenRimC2LoftFits)
	{
		HashValue(Hash, Fit.FitId);
		HashValue(Hash, Fit.Model);
		HashValue(Hash, Fit.CoefficientCount);
		HashValue(Hash, Fit.FirstSegmentIndex);
		HashValue(Hash, Fit.SegmentCount);
		HashValue(Hash, Fit.MemberCount);
		HashValue(Hash, Fit.SampleCount);
		for (int32 Index = 0; Index < 6; ++Index)
		{
			HashValue(Hash, Fit.EndDepthCoefficients[Index]);
			HashValue(Hash, Fit.EndOpeningOffsetCoefficients[Index]);
			HashValue(Hash, Fit.BoundaryPlaneTangentCoefficients[Index]);
			HashValue(Hash, Fit.LongitudinalTangentCoefficients[Index]);
		}
		HashValue(Hash, Fit.BalancedRootMeanSquareResidualCm);
		HashValue(Hash, Fit.MaximumMemberRootMeanSquareResidualCm);
		HashValue(Hash, Fit.MaximumResidualCm);
		HashValue(Hash, Fit.MinimumSampledParameterValueCm);
		HashValue(Hash, Fit.MaximumSampledFirstDerivativeMagnitudeCm);
		HashValue(Hash, Fit.MaximumSampledSecondDerivativeMagnitudeCm);
		HashValue(Hash, Fit.MaximumFirstDerivativeJoinResidualCm);
		HashValue(Hash, Fit.MaximumSecondDerivativeJoinResidualCm);
		HashValue(Hash, Fit.bExactXYMirrorPlacement);
		HashValue(Hash, Fit.bC2AlongRimByPolynomialConstruction);
		HashValue(Hash, Fit.bRegularPositiveParameterization);
	}
	const int32 OpenRimCanonicalSurfaceSampleCount =
		OpenRimCanonicalSurfaceSamples.Num();
	HashValue(Hash, OpenRimCanonicalSurfaceSampleCount);
	for (const FOpenRimCanonicalSurfaceSample& Sample :
		OpenRimCanonicalSurfaceSamples)
	{
		HashValue(Hash, Sample.OpenRimTransverseSectionIndex);
		HashValue(Hash, Sample.TransitionPointOffset);
		HashValue(Hash, Sample.CanonicalRimParameter);
		HashValue(Hash, Sample.TransitionParameter);
		HashValue(Hash, Sample.CanonicalPositionCm);
	}
	const int32 OpenRimCanonicalSectionCorrespondenceCount =
		OpenRimCanonicalSectionCorrespondences.Num();
	HashValue(Hash, OpenRimCanonicalSectionCorrespondenceCount);
	for (const FOpenRimCanonicalSectionCorrespondence& Correspondence :
		OpenRimCanonicalSectionCorrespondences)
	{
		HashValue(Hash, Correspondence.OpenRimTransverseSectionIndex);
		HashValue(Hash, Correspondence.LaneId);
		HashValue(Hash, Correspondence.LaneRank);
		HashValue(Hash, Correspondence.LaneSectionCount);
		HashValue(Hash, Correspondence.GeometricSurfaceParameter);
		HashValue(Hash, Correspondence.TopologySurfaceParameter);
		HashValue(Hash, Correspondence.OptimizedSurfaceParameter);
		HashValue(Hash, Correspondence.RootMeanSquareResidualCm);
		HashValue(Hash, Correspondence.bWithinQuantizedWitnessBounds);
		HashValue(Hash, Correspondence.bStrictlyInterior);
	}
	const int32 OpenRimCanonicalSurfaceFitCount =
		OpenRimCanonicalSurfaceFits.Num();
	HashValue(Hash, OpenRimCanonicalSurfaceFitCount);
	for (const FOpenRimCanonicalSurfaceFit& Fit :
		OpenRimCanonicalSurfaceFits)
	{
		HashValue(Hash, Fit.FitId);
		HashValue(Hash, Fit.SampleCount);
		HashValue(Hash, Fit.VerticalSegmentParameterWidth);
		HashValue(Hash, Fit.TransitionParameterWidth);
		HashValue(Hash, Fit.HorizontalSpanParameterWidth);
		for (int32 Index = 0; Index < 6; ++Index)
		{
			HashValue(Hash, Fit.StartTangentYCoefficients[Index]);
			HashValue(Hash, Fit.StartTangentZCoefficients[Index]);
			HashValue(Hash, Fit.EndDepthCoefficients[Index]);
			HashValue(Hash, Fit.EndYCoefficients[Index]);
			HashValue(Hash, Fit.EndZCoefficients[Index]);
			HashValue(Hash, Fit.EndLongitudinalTangentCoefficients[Index]);
		}
		for (int32 Index = 0; Index < 4; ++Index)
		{
			HashValue(Hash, Fit.TransitionStartTangentYCoefficients[Index]);
			HashValue(Hash, Fit.TransitionStartTangentZCoefficients[Index]);
			HashValue(Hash, Fit.TransitionEndDepthCoefficients[Index]);
			HashValue(Hash, Fit.TransitionEndYCoefficients[Index]);
			HashValue(Hash, Fit.TransitionEndZCoefficients[Index]);
		}
		HashValue(Hash, Fit.RootMeanSquareResidualCm);
		HashValue(Hash, Fit.MaximumResidualCm);
		HashValue(Hash, Fit.MaximumResidualSectionIndex);
		HashValue(Hash, Fit.MaximumResidualPointOffset);
		HashValue(Hash, Fit.MaximumResidualRimParameter);
		HashValue(Hash, Fit.MaximumResidualTransitionParameter);
		HashValue(Hash, Fit.MaximumResidualSourcePositionCm);
		HashValue(Hash, Fit.MaximumResidualFitPositionCm);
		HashValue(Hash, Fit.TransitionEndpointClampedSampleCount);
		HashValue(Hash, Fit.TransitionEndpointClampedRootMeanSquareResidualCm);
		HashValue(Hash, Fit.TransitionEndpointClampedMaximumResidualCm);
		HashValue(Hash, Fit.TransitionInteriorSampleCount);
		HashValue(Hash, Fit.TransitionInteriorRootMeanSquareResidualCm);
		HashValue(Hash, Fit.TransitionInteriorMaximumResidualCm);
		HashValue(Hash, Fit.CorrespondenceLaneCount);
		HashValue(Hash, Fit.CorrespondenceSectionCount);
		HashValue(Hash, Fit.MaximumCorrespondenceDisplacement);
		HashValue(Hash, Fit.MinimumCorrespondenceSpacing);
		for (int32 FeatureIndex = 0; FeatureIndex < 4; ++FeatureIndex)
		{
			HashValue(Hash, Fit.FeatureSampleCounts[FeatureIndex]);
			HashValue(Hash, Fit.FeatureRootMeanSquareResidualsCm[FeatureIndex]);
			HashValue(Hash, Fit.FeatureMaximumResidualsCm[FeatureIndex]);
		}
		HashValue(Hash, Fit.MinimumEndDepthCm);
		HashValue(Hash, Fit.MinimumLongitudinalTangentCm);
		HashValue(Hash, Fit.MaximumFirstDerivativeJoinResidualCm);
		HashValue(Hash, Fit.MaximumSecondDerivativeJoinResidualCm);
		HashValue(Hash, Fit.MaximumNormalJoinResidualDegrees);
		HashValue(Hash, Fit.bExactXYMirrorPlacement);
		HashValue(Hash, Fit.bExactTransverseC2ByConstruction);
		HashValue(Hash, Fit.bLongitudinallyC2ByConstruction);
		HashValue(Hash, Fit.bTransitionCorrectionC2ByConstruction);
		HashValue(Hash, Fit.bCorrespondenceStrictlyMonotone);
		HashValue(Hash, Fit.bCorrespondenceWithinQuantizedWitnessBounds);
		HashValue(Hash, Fit.bCorrespondenceStrictlyInterior);
		HashValue(Hash, Fit.bRegularPositiveParameterization);
	}
	const int32 OpenRimCanonicalTubeFitCount = OpenRimCanonicalTubeFits.Num();
	HashValue(Hash, OpenRimCanonicalTubeFitCount);
	for (const FOpenRimCanonicalTubeFit& Fit : OpenRimCanonicalTubeFits)
	{
		HashValue(Hash, Fit.FitId);
		HashValue(Hash, Fit.OpeningSide);
		HashValue(Hash, Fit.TransverseSide);
		HashValue(Hash, Fit.SampleCount);
		for (int32 Index = 0; Index < 6; ++Index)
		{
			HashValue(Hash, Fit.TerminalDepthCoefficients[Index]);
			HashValue(Hash, Fit.TerminalYCoefficients[Index]);
			HashValue(Hash, Fit.TerminalZCoefficients[Index]);
		}
		HashValue(Hash, Fit.RootMeanSquareResidualCm);
		HashValue(Hash, Fit.MaximumResidualCm);
		HashValue(Hash, Fit.MaximumResidualSectionIndex);
		const int32 TerminalSplineSegmentCount = Fit.TerminalSplineSegments.Num();
		HashValue(Hash, TerminalSplineSegmentCount);
		for (const FOpenRimTubeTerminalSplineSegment& Segment :
			Fit.TerminalSplineSegments)
		{
			HashValue(Hash, Segment.MinimumCanonicalRimParameter);
			HashValue(Hash, Segment.MaximumCanonicalRimParameter);
			for (const FVector3d& ControlPoint : Segment.ControlPoints)
				HashValue(Hash, ControlPoint);
		}
		HashValue(Hash, Fit.bExactXYMirrorPlacement);
		HashValue(Hash, Fit.bRegularFiniteTube);
		HashValue(Hash, Fit.bTerminalCurveC2);
	}
	const int32 OpenRimCanonicalTubeTensorSurfaceCount =
		OpenRimCanonicalTubeTensorSurfaces.Num();
	HashValue(Hash, OpenRimCanonicalTubeTensorSurfaceCount);
	for (const FOpenRimCanonicalTubeTensorSurface& Tensor :
		OpenRimCanonicalTubeTensorSurfaces)
	{
		HashValue(Hash, Tensor.SourceFitId);
		HashValue(Hash, Tensor.OpeningSide);
		HashValue(Hash, Tensor.TransverseSide);
		HashValue(Hash, Tensor.SegmentIndex);
		HashValue(Hash, Tensor.MinimumCanonicalRimParameter);
		HashValue(Hash, Tensor.MaximumCanonicalRimParameter);
		HashValue(Hash, Tensor.MinimumTubeParameter);
		HashValue(Hash, Tensor.MaximumTubeParameter);
		HashValue(Hash, Tensor.LongitudinalParameterScale);
		HashValue(Hash, Tensor.MaximumConversionErrorCm);
		HashValue(Hash, Tensor.bLipBoundaryC2ByConstruction);
		HashValue(Hash, Tensor.bAdaptiveCompactC2);
		HashValue(Hash, Tensor.bAdaptiveTerminalClosureC2);
		HashValue(Hash, Tensor.bSourceResidualCertified);
		HashValue(Hash, Tensor.SourceMaximumResidualCm);
		HashValue(Hash, Tensor.bMeasuredWitnessOnly);
		HashValue(Hash, Tensor.Surface.DegreeU);
		HashValue(Hash, Tensor.Surface.DegreeV);
		for (const FVector3d& ControlPoint : Tensor.Surface.ControlPoints)
			HashValue(Hash, ControlPoint);
	}
	const int32 OpenRimCanonicalTubeLoftFitCount = OpenRimCanonicalTubeLoftFits.Num();
	HashValue(Hash, OpenRimCanonicalTubeLoftFitCount);
	for (const FOpenRimCanonicalTubeLoftFit& Loft : OpenRimCanonicalTubeLoftFits)
	{
		HashValue(Hash, Loft.FitId);
		HashValue(Hash, Loft.OpeningSide);
		HashValue(Hash, Loft.TransverseSide);
		HashValue(Hash, Loft.TileIndex);
		HashValue(Hash, Loft.MinimumCanonicalRimParameter);
		HashValue(Hash, Loft.MaximumCanonicalRimParameter);
		HashValue(Hash, Loft.MinimumTubeParameter);
		HashValue(Hash, Loft.MaximumTubeParameter);
		HashValue(Hash, Loft.SampleCount);
		HashValue(Hash, Loft.InterpolationMaximumErrorCm);
		HashValue(Hash, Loft.RootMeanSquareResidualCm);
		HashValue(Hash, Loft.MaximumResidualCm);
		HashValue(Hash, Loft.MaximumResidualSectionIndex);
		HashValue(Hash, Loft.bLongitudinalC2Input);
		HashValue(Hash, Loft.bTransverseC2Input);
		HashValue(Hash, Loft.bLowDegreeCandidate);
		HashValue(Hash, Loft.Surface.DegreeU);
		HashValue(Hash, Loft.Surface.DegreeV);
		for (const FVector3d& ControlPoint : Loft.Surface.ControlPoints)
			HashValue(Hash, ControlPoint);
	}
	const int32 OpenRimSupportTransitionIntentCount =
		OpenRimSupportTransitionIntents.Num();
	HashValue(Hash, OpenRimSupportTransitionIntentCount);
	for (const FOpenRimSupportTransitionIntent& Intent :
		OpenRimSupportTransitionIntents)
	{
		HashValue(Hash, Intent.IntentId);
		HashValue(Hash, Intent.OpenRimTransverseSectionIndex);
		HashValue(Hash, Intent.Feature);
		HashValue(Hash, Intent.Policy);
		HashValue(Hash, Intent.bGeometricC2Permitted);
		HashValue(Hash, Intent.bBidirectional);
		HashValue(Hash, Intent.bImmediateAdjacentSurfaceHandoffForbidden);
		HashValue(Hash, Intent.bFreshApproachMayAcquireAdjacentSurface);
	}
	return Hash;
}

bool FAnalyticWorldData::IsAuthorityEligible() const
{
	return (!Planes.IsEmpty() || !ExtrudedQuinticPatches.IsEmpty() ||
		!TensorBezierPatches.IsEmpty() ||
		!PiecewiseTensorBezierPatches.IsEmpty() ||
		!Triangles.IsEmpty()) &&
		Algo::AllOf(Planes,
			[](const FBoundedPlane& Plane) { return Plane.bAuthorityEligible; }) &&
		Algo::AllOf(ExtrudedQuinticPatches,
			[](const FExtrudedQuinticPatch& Patch)
			{
				return Patch.bAuthorityEligible;
			}) &&
		Algo::AllOf(TensorBezierPatches,
			[](const FTensorBezierPatch& Patch)
			{
				return Patch.bAuthorityEligible;
			}) &&
		Algo::AllOf(PiecewiseTensorBezierPatches,
			[](const FPiecewiseTensorBezierPatch& Patch)
			{
				return Patch.bAuthorityEligible;
			}) &&
		Algo::AllOf(Triangles,
			[](const FTriangleSurface& Triangle)
			{
				return Triangle.bAuthorityEligible;
			});
}

uint64 StableStringId(const FString& Value)
{
	FTCHARToUTF8 Utf8(*Value);
	uint64 Hash = FnvOffset;
	HashBytes(Hash, Utf8.Get(), static_cast<SIZE_T>(Utf8.Length()));
	return Hash == 0 ? 1 : Hash;
}

uint64 CombineStableIds(const uint64 A, const uint64 B)
{
	uint64 Hash = FnvOffset;
	HashValue(Hash, A);
	HashValue(Hash, B);
	return Hash == 0 ? 1 : Hash;
}

} // namespace Speed::Analytic
