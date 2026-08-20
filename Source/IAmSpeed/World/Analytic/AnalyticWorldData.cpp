#include "AnalyticWorldData.h"

#include "Algo/AllOf.h"
#include "Algo/AnyOf.h"
#include "Algo/Sort.h"

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

bool FBoundedPlane::ContainsProjectedPoint(
	const FVector3d& Point, const double Tolerance) const
{
	const FVector3d Relative = Point - Origin;
	return FMath::Abs(FVector3d::DotProduct(Relative, AxisU)) <=
		HalfExtents.X + Tolerance &&
		FMath::Abs(FVector3d::DotProduct(Relative, AxisV)) <=
		HalfExtents.Y + Tolerance;
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
		BaseRootMeanSquareResidualCm < 0.0 || BaseMaximumResidualCm < 0.0 ||
		CorrectedRootMeanSquareResidualCm < 0.0 ||
		CorrectedMaximumResidualCm < 0.0 ||
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

bool FAnalyticWorldData::FinalizeAndValidate(FString* OutReason)
{
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
		for (const double SignU : { -1.0, 1.0 })
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
		PreviousPrimitive = ExtrudedQuinticPatches[Index].PrimitiveId;
		CompactBounds += ExtrudedQuinticPatches[Index].Bounds;
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
	Planes.RemoveAll([](const FBoundedPlane& Plane)
	{
		return Plane.bRequiresCompactOptIn;
	});
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
		const bool bUseSymmetrizedConstraint = SymmetrizedConstraint &&
			SymmetrizedConstraint->bSourceFitPlausible &&
			SymmetrizedConstraint->bExactMirrorPlacement;
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
		for (int32 Corner = 0; Corner < 8; ++Corner)
		{
			const FVector3d Point(
				(Corner & 1) ? Group.Bounds.Max.X : Group.Bounds.Min.X,
				(Corner & 2) ? Group.Bounds.Max.Y : Group.Bounds.Min.Y,
				(Corner & 4) ? Group.Bounds.Max.Z : Group.Bounds.Min.Z);
			const double U = FVector3d::DotProduct(Point, AxisU);
			const double V = FVector3d::DotProduct(Point, AxisV);
			MinimumU = FMath::Min(MinimumU, U);
			MaximumU = FMath::Max(MaximumU, U);
			MinimumV = FMath::Min(MinimumV, V);
			MaximumV = FMath::Max(MaximumV, V);
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
		Patch.bAuthorityEligible = false;
		AddedFitIndices.Add(Coverage.TransitionFitIndex);
	}
	Algo::Sort(ExtrudedQuinticPatches,
		[](const FExtrudedQuinticPatch& A, const FExtrudedQuinticPatch& B)
		{
			return A.PrimitiveId < B.PrimitiveId;
		});
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
	constexpr double MaximumDominantCurvatureRatio = 1.25;
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
		for (const double HeightFraction : { 0.1, 0.2, 0.3, 0.4, 0.5 })
		{
			const double Height = FMath::Lerp(Arch.BaselineHeightCm,
				Arch.VerticalSegmentTransitionHeightCm, HeightFraction);
			SliceSpecs.Add({ EOpenRimFeature::NegativeVertical, Height });
			SliceSpecs.Add({ EOpenRimFeature::PositiveVertical, Height });
		}
		for (const double SpanFraction :
			{ -0.75, -0.5, -0.25, 0.0, 0.25, 0.5, 0.75 })
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
	OpenRimCanonicalSurfaceFits.Reset();
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

	TArray<int32> LoftMembers;
	for (int32 SectionIndex = 0;
		SectionIndex < OpenRimTransverseSections.Num(); ++SectionIndex)
	{
		if (OpenRimTransverseSections[SectionIndex].bIndividualC2FitValid)
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
		!Triangles.IsEmpty()) &&
		Algo::AllOf(Planes,
			[](const FBoundedPlane& Plane) { return Plane.bAuthorityEligible; }) &&
		Algo::AllOf(ExtrudedQuinticPatches,
			[](const FExtrudedQuinticPatch& Patch)
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
