#include "AnalyticWorldQuery.h"

namespace Speed::Analytic
{
namespace
{
	struct FContactWitness
	{
		FVector3d QueryPoint = FVector3d::ZeroVector;
		FVector3d SurfacePoint = FVector3d::ZeroVector;
		Speed::EContactFeatureKind QueryKind = Speed::EContactFeatureKind::Unknown;
		Speed::EContactFeatureKind SurfaceKind = Speed::EContactFeatureKind::Unknown;
		int8 QueryIndex = INDEX_NONE;
		int8 SurfaceIndex = INDEX_NONE;
		double DistanceSquared = TNumericLimits<double>::Max();
	};

	struct FPlaneTriangle
	{
		FVector3d Vertex[3];
		int32 PolygonVertex[3] = { INDEX_NONE, INDEX_NONE, INDEX_NONE };
	};

	double Cross2D(const FVector2d& A, const FVector2d& B)
	{
		return A.X * B.Y - A.Y * B.X;
	}

	bool PointInTriangle2D(
		const FVector2d& Point, const FVector2d& A, const FVector2d& B,
		const FVector2d& C, const double Winding)
	{
		constexpr double Tolerance = 1.0e-12;
		return Winding * Cross2D(B - A, Point - A) >= -Tolerance &&
			Winding * Cross2D(C - B, Point - B) >= -Tolerance &&
			Winding * Cross2D(A - C, Point - C) >= -Tolerance;
	}

	void TriangulatePlaneDomain(
		const FBoundedPlane& Plane, TArray<FPlaneTriangle, TInlineAllocator<32>>& Out)
	{
		TArray<FVector2d, TInlineAllocator<32>> Vertices;
		if (Plane.DomainVertices.IsEmpty())
		{
			Vertices = {
				FVector2d(-Plane.HalfExtents.X, -Plane.HalfExtents.Y),
				FVector2d( Plane.HalfExtents.X, -Plane.HalfExtents.Y),
				FVector2d( Plane.HalfExtents.X,  Plane.HalfExtents.Y),
				FVector2d(-Plane.HalfExtents.X,  Plane.HalfExtents.Y) };
		}
		else
		{
			Vertices.Append(Plane.DomainVertices);
		}

		double TwiceArea = 0.0;
		for (int32 Index = 0; Index < Vertices.Num(); ++Index)
		{
			TwiceArea += Cross2D(
				Vertices[Index], Vertices[(Index + 1) % Vertices.Num()]);
		}
		const double Winding = TwiceArea >= 0.0 ? 1.0 : -1.0;
		TArray<int32, TInlineAllocator<32>> Remaining;
		for (int32 Index = 0; Index < Vertices.Num(); ++Index) Remaining.Add(Index);
		while (Remaining.Num() > 3)
		{
			bool bFoundEar = false;
			for (int32 Slot = 0; Slot < Remaining.Num(); ++Slot)
			{
				const int32 Previous = Remaining[
					(Slot + Remaining.Num() - 1) % Remaining.Num()];
				const int32 Current = Remaining[Slot];
				const int32 Next = Remaining[(Slot + 1) % Remaining.Num()];
				if (Winding * Cross2D(
					Vertices[Current] - Vertices[Previous],
					Vertices[Next] - Vertices[Current]) <= 1.0e-12)
				{
					continue;
				}
				bool bContainsVertex = false;
				for (const int32 Candidate : Remaining)
				{
					if (Candidate == Previous || Candidate == Current || Candidate == Next)
					{
						continue;
					}
					if (PointInTriangle2D(Vertices[Candidate], Vertices[Previous],
						Vertices[Current], Vertices[Next], Winding))
					{
						bContainsVertex = true;
						break;
					}
				}
				if (bContainsVertex) continue;

				FPlaneTriangle& Triangle = Out.AddDefaulted_GetRef();
				const int32 PolygonIndices[3] = { Previous, Current, Next };
				for (int32 Corner = 0; Corner < 3; ++Corner)
				{
					const FVector2d& Local = Vertices[PolygonIndices[Corner]];
					Triangle.Vertex[Corner] = Plane.Origin + Local.X * Plane.AxisU +
						Local.Y * Plane.AxisV;
					Triangle.PolygonVertex[Corner] = PolygonIndices[Corner];
				}
				Remaining.RemoveAt(Slot, 1, EAllowShrinking::No);
				bFoundEar = true;
				break;
			}
			if (!bFoundEar)
			{
				// IsValid rejects self intersections and collapsed edges. Reaching
				// this path therefore means the numerical ear certificate failed;
				// declining the query is safer than inventing coverage.
				Out.Reset();
				return;
			}
		}
		if (Remaining.Num() == 3)
		{
			FPlaneTriangle& Triangle = Out.AddDefaulted_GetRef();
			for (int32 Corner = 0; Corner < 3; ++Corner)
			{
				const int32 PolygonIndex = Remaining[Corner];
				const FVector2d& Local = Vertices[PolygonIndex];
				Triangle.Vertex[Corner] = Plane.Origin + Local.X * Plane.AxisU +
					Local.Y * Plane.AxisV;
				Triangle.PolygonVertex[Corner] = PolygonIndex;
			}
		}
	}

	FVector3d ClosestPointOnTriangle(
		const FVector3d& Point, const FVector3d& A, const FVector3d& B,
		const FVector3d& C, EContactFeatureKind* OutKind = nullptr,
		int8* OutIndex = nullptr)
	{
		const FVector3d AB = B - A;
		const FVector3d AC = C - A;
		const FVector3d AP = Point - A;
		const double D1 = FVector3d::DotProduct(AB, AP);
		const double D2 = FVector3d::DotProduct(AC, AP);
		auto SetFeature = [OutKind, OutIndex](const EContactFeatureKind Kind,
			const int8 Index)
		{
			if (OutKind) *OutKind = Kind;
			if (OutIndex) *OutIndex = Index;
		};
		if (D1 <= 0.0 && D2 <= 0.0)
		{
			SetFeature(EContactFeatureKind::Vertex, 0); return A;
		}
		const FVector3d BP = Point - B;
		const double D3 = FVector3d::DotProduct(AB, BP);
		const double D4 = FVector3d::DotProduct(AC, BP);
		if (D3 >= 0.0 && D4 <= D3)
		{
			SetFeature(EContactFeatureKind::Vertex, 1); return B;
		}
		const double VC = D1 * D4 - D3 * D2;
		if (VC <= 0.0 && D1 >= 0.0 && D3 <= 0.0)
		{
			SetFeature(EContactFeatureKind::Edge, 0); return A + D1 / (D1 - D3) * AB;
		}
		const FVector3d CP = Point - C;
		const double D5 = FVector3d::DotProduct(AB, CP);
		const double D6 = FVector3d::DotProduct(AC, CP);
		if (D6 >= 0.0 && D5 <= D6)
		{
			SetFeature(EContactFeatureKind::Vertex, 2); return C;
		}
		const double VB = D5 * D2 - D1 * D6;
		if (VB <= 0.0 && D2 >= 0.0 && D6 <= 0.0)
		{
			SetFeature(EContactFeatureKind::Edge, 2); return A + D2 / (D2 - D6) * AC;
		}
		const double VA = D3 * D6 - D5 * D4;
		if (VA <= 0.0 && D4 - D3 >= 0.0 && D5 - D6 >= 0.0)
		{
			SetFeature(EContactFeatureKind::Edge, 1);
			return B + (D4 - D3) / ((D4 - D3) + (D5 - D6)) * (C - B);
		}
		const double Denominator = 1.0 / (VA + VB + VC);
		SetFeature(EContactFeatureKind::Face, 0);
		return A + (VB * Denominator) * AB + (VC * Denominator) * AC;
	}

	FVector3d ClosestPointOnBox(
		const FVector3d& Point, const FVector3d& Center,
		const FVector3d Axis[3], const FVector3d& HalfExtent,
		EContactFeatureKind* OutKind = nullptr, int8* OutIndex = nullptr)
	{
		FVector3d Result = Center;
		int32 BoundaryCount = 0;
		int8 FeatureIndex = 0;
		for (int32 AxisIndex = 0; AxisIndex < 3; ++AxisIndex)
		{
			const double Extent = HalfExtent[AxisIndex];
			const double Coordinate = FVector3d::DotProduct(Point - Center, Axis[AxisIndex]);
			const double Clamped = FMath::Clamp(Coordinate, -Extent, Extent);
			Result += Clamped * Axis[AxisIndex];
			if (FMath::Abs(FMath::Abs(Clamped) - Extent) <= 1.0e-8)
			{
				++BoundaryCount;
				FeatureIndex |= (Clamped >= 0.0 ? 1 : 0) << AxisIndex;
			}
		}
		if (OutKind)
		{
			*OutKind = BoundaryCount >= 3 ? EContactFeatureKind::Vertex :
				BoundaryCount == 2 ? EContactFeatureKind::Edge :
				BoundaryCount == 1 ? EContactFeatureKind::Face :
				EContactFeatureKind::Unknown;
		}
		if (OutIndex) *OutIndex = FeatureIndex;
		return Result;
	}

	void ClosestPointsOnSegments(
		const FVector3d& P0, const FVector3d& P1, const FVector3d& Q0,
		const FVector3d& Q1, FVector3d& OutP, FVector3d& OutQ)
	{
		const FVector3d D1 = P1 - P0;
		const FVector3d D2 = Q1 - Q0;
		const FVector3d R = P0 - Q0;
		const double A = D1.SquaredLength();
		const double E = D2.SquaredLength();
		const double F = FVector3d::DotProduct(D2, R);
		double S = 0.0;
		double T = 0.0;
		if (A <= UE_DOUBLE_SMALL_NUMBER)
		{
			T = E > UE_DOUBLE_SMALL_NUMBER ? FMath::Clamp(F / E, 0.0, 1.0) : 0.0;
		}
		else
		{
			const double C = FVector3d::DotProduct(D1, R);
			if (E <= UE_DOUBLE_SMALL_NUMBER)
			{
				S = FMath::Clamp(-C / A, 0.0, 1.0);
			}
			else
			{
				const double B = FVector3d::DotProduct(D1, D2);
				const double Denominator = A * E - B * B;
				if (Denominator > UE_DOUBLE_SMALL_NUMBER)
				{
					S = FMath::Clamp((B * F - C * E) / Denominator, 0.0, 1.0);
				}
				T = (B * S + F) / E;
				if (T < 0.0)
				{
					T = 0.0; S = FMath::Clamp(-C / A, 0.0, 1.0);
				}
				else if (T > 1.0)
				{
					T = 1.0; S = FMath::Clamp((B - C) / A, 0.0, 1.0);
				}
			}
		}
		OutP = P0 + S * D1;
		OutQ = Q0 + T * D2;
	}

	void ConsiderWitness(FContactWitness& Best, const FVector3d& QueryPoint,
		const FVector3d& SurfacePoint, const EContactFeatureKind QueryKind,
		const int8 QueryIndex, const EContactFeatureKind SurfaceKind,
		const int8 SurfaceIndex)
	{
		const double DistanceSquared = (QueryPoint - SurfacePoint).SquaredLength();
		if (DistanceSquared + 1.0e-18 >= Best.DistanceSquared) return;
		Best.QueryPoint = QueryPoint;
		Best.SurfacePoint = SurfacePoint;
		Best.QueryKind = QueryKind;
		Best.QueryIndex = QueryIndex;
		Best.SurfaceKind = SurfaceKind;
		Best.SurfaceIndex = SurfaceIndex;
		Best.DistanceSquared = DistanceSquared;
	}

	FContactWitness ClosestBoxTriangleWitness(
		const FVector3d& Center, const FVector3d Axis[3],
		const FVector3d& HalfExtent, const FPlaneTriangle& Triangle)
	{
		FContactWitness Best;
		for (int32 Corner = 0; Corner < 3; ++Corner)
		{
			EContactFeatureKind BoxKind;
			int8 BoxIndex;
			const FVector3d BoxPoint = ClosestPointOnBox(
				Triangle.Vertex[Corner], Center, Axis, HalfExtent, &BoxKind, &BoxIndex);
			ConsiderWitness(Best, BoxPoint, Triangle.Vertex[Corner], BoxKind,
				BoxIndex, EContactFeatureKind::Vertex,
				static_cast<int8>(Triangle.PolygonVertex[Corner]));
		}

		FVector3d BoxVertices[8];
		for (int32 Vertex = 0; Vertex < 8; ++Vertex)
		{
			BoxVertices[Vertex] = Center;
			for (int32 AxisIndex = 0; AxisIndex < 3; ++AxisIndex)
			{
				BoxVertices[Vertex] += ((Vertex & (1 << AxisIndex)) ? 1.0 : -1.0) *
					HalfExtent[AxisIndex] * Axis[AxisIndex];
			}
			EContactFeatureKind TriangleKind;
			int8 TriangleIndex;
			const FVector3d TrianglePoint = ClosestPointOnTriangle(BoxVertices[Vertex],
				Triangle.Vertex[0], Triangle.Vertex[1], Triangle.Vertex[2],
				&TriangleKind, &TriangleIndex);
			if (TriangleKind == EContactFeatureKind::Vertex)
			{
				TriangleIndex = static_cast<int8>(
					Triangle.PolygonVertex[TriangleIndex]);
			}
			ConsiderWitness(Best, BoxVertices[Vertex], TrianglePoint,
				EContactFeatureKind::Vertex, static_cast<int8>(Vertex), TriangleKind,
				TriangleIndex);
		}

		static constexpr int32 BoxEdges[12][2] = {
			{0,1},{2,3},{4,5},{6,7}, {0,2},{1,3},{4,6},{5,7},
			{0,4},{1,5},{2,6},{3,7} };
		for (int32 BoxEdge = 0; BoxEdge < 12; ++BoxEdge)
		{
			for (int32 TriangleEdge = 0; TriangleEdge < 3; ++TriangleEdge)
			{
				FVector3d BoxPoint;
				FVector3d TrianglePoint;
				ClosestPointsOnSegments(
					BoxVertices[BoxEdges[BoxEdge][0]], BoxVertices[BoxEdges[BoxEdge][1]],
					Triangle.Vertex[TriangleEdge],
					Triangle.Vertex[(TriangleEdge + 1) % 3], BoxPoint, TrianglePoint);
				ConsiderWitness(Best, BoxPoint, TrianglePoint,
					EContactFeatureKind::Edge, static_cast<int8>(BoxEdge),
					EContactFeatureKind::Edge, static_cast<int8>(TriangleEdge));
			}
		}
		return Best;
	}

	bool SweepBoxTriangle(
		const FWorldQuery& Query, const FPlaneTriangle& Triangle,
		FWorldHit& OutHit)
	{
		const FVector3d Axis[3] = {
			Query.Rotation.RotateVector(FVector3d::ForwardVector).GetSafeNormal(),
			Query.Rotation.RotateVector(FVector3d::RightVector).GetSafeNormal(),
			Query.Rotation.RotateVector(FVector3d::UpVector).GetSafeNormal() };
		const FVector3d Edges[3] = {
			Triangle.Vertex[1] - Triangle.Vertex[0],
			Triangle.Vertex[2] - Triangle.Vertex[1],
			Triangle.Vertex[0] - Triangle.Vertex[2] };
		const FVector3d TriangleNormal = FVector3d::CrossProduct(
			Edges[0], Triangle.Vertex[2] - Triangle.Vertex[0]).GetSafeNormal();
		TArray<FVector3d, TInlineAllocator<16>> SeparatingAxes;
		SeparatingAxes.Add(TriangleNormal);
		for (const FVector3d& BoxAxis : Axis) SeparatingAxes.Add(BoxAxis);
		for (const FVector3d& BoxAxis : Axis)
		{
			for (const FVector3d& Edge : Edges)
			{
				const FVector3d Cross = FVector3d::CrossProduct(BoxAxis, Edge);
				if (Cross.SquaredLength() > 1.0e-20) SeparatingAxes.Add(Cross.GetSafeNormal());
			}
		}

		const FVector3d Motion = Query.End - Query.Start;
		double EnterTime = 0.0;
		double ExitTime = 1.0;
		FVector3d EntryNormal = FVector3d::ZeroVector;
		double MinimumDepth = TNumericLimits<double>::Max();
		FVector3d MinimumTranslationNormal = FVector3d::ZeroVector;
		bool bInitiallyOverlapping = true;
		for (const FVector3d& TestAxis : SeparatingAxes)
		{
			const double Radius =
				FMath::Abs(FVector3d::DotProduct(Axis[0], TestAxis)) * Query.HalfExtent.X +
				FMath::Abs(FVector3d::DotProduct(Axis[1], TestAxis)) * Query.HalfExtent.Y +
				FMath::Abs(FVector3d::DotProduct(Axis[2], TestAxis)) * Query.HalfExtent.Z;
			const double BoxCenter = FVector3d::DotProduct(Query.Start, TestAxis);
			const double BoxMinimum = BoxCenter - Radius;
			const double BoxMaximum = BoxCenter + Radius;
			double TriangleMinimum = TNumericLimits<double>::Max();
			double TriangleMaximum = -TNumericLimits<double>::Max();
			for (const FVector3d& Vertex : Triangle.Vertex)
			{
				const double Projection = FVector3d::DotProduct(Vertex, TestAxis);
				TriangleMinimum = FMath::Min(TriangleMinimum, Projection);
				TriangleMaximum = FMath::Max(TriangleMaximum, Projection);
			}
			const bool bAxisOverlaps = BoxMaximum >= TriangleMinimum &&
				BoxMinimum <= TriangleMaximum;
			bInitiallyOverlapping &= bAxisOverlaps;
			if (bAxisOverlaps)
			{
				const double PositiveDepth = TriangleMaximum - BoxMinimum;
				const double NegativeDepth = BoxMaximum - TriangleMinimum;
				const double AxisDepth = FMath::Min(PositiveDepth, NegativeDepth);
				if (AxisDepth < MinimumDepth)
				{
					MinimumDepth = AxisDepth;
					MinimumTranslationNormal = PositiveDepth <= NegativeDepth ?
						TestAxis : -TestAxis;
				}
			}

			const double Speed = FVector3d::DotProduct(Motion, TestAxis);
			if (FMath::Abs(Speed) <= 1.0e-15)
			{
				if (BoxMaximum < TriangleMinimum - Query.DomainTolerance ||
					BoxMinimum > TriangleMaximum + Query.DomainTolerance)
				{
					return false;
				}
				continue;
			}
			// DomainTolerance is a robustness allowance for a stationary axis,
			// not an artificial inflation of the moving shapes. Keeping it out
			// of the interval times preserves the geometrically exact TOI.
			double AxisEnter = (TriangleMinimum - BoxMaximum) / Speed;
			double AxisExit = (TriangleMaximum - BoxMinimum) / Speed;
			if (AxisEnter > AxisExit) Swap(AxisEnter, AxisExit);
			if (AxisEnter > EnterTime)
			{
				EnterTime = AxisEnter;
				EntryNormal = Speed > 0.0 ? -TestAxis : TestAxis;
			}
			ExitTime = FMath::Min(ExitTime, AxisExit);
			if (EnterTime > ExitTime || ExitTime < 0.0 || EnterTime > 1.0)
			{
				return false;
			}
		}

		const double Time = bInitiallyOverlapping ? 0.0 : FMath::Max(0.0, EnterTime);
		FVector3d Normal = bInitiallyOverlapping ? MinimumTranslationNormal : EntryNormal;
		if (Normal.IsNearlyZero())
		{
			Normal = FVector3d::DotProduct(Query.Start - Triangle.Vertex[0],
				TriangleNormal) >= 0.0 ? TriangleNormal : -TriangleNormal;
		}
		Normal.Normalize();
		double PenetrationDepth = bInitiallyOverlapping ?
			FMath::Max(0.0, MinimumDepth) : 0.0;
		bool bStartPenetrating = bInitiallyOverlapping &&
			PenetrationDepth > Query.DomainTolerance;
		if (bStartPenetrating &&
			PenetrationDepth <= FMath::Max(0.0, Query.InitialOverlapTolerance))
		{
			const double ClosingDistance = -FVector3d::DotProduct(Motion, Normal);
			if (ClosingDistance <= Query.DomainTolerance)
			{
				return false;
			}
			// The query begins inside the declared contact shell but moves into
			// the surface. Preserve a blocking time-zero contact without asking
			// the solver to depenetrate accepted shell depth.
			bStartPenetrating = false;
			PenetrationDepth = 0.0;
		}
		const FVector3d Center = FMath::Lerp(Query.Start, Query.End, Time);
		const FVector3d WitnessCenter = bStartPenetrating ?
			Center + PenetrationDepth * Normal : Center;
		FContactWitness Witness;
		Witness.QueryPoint = WitnessCenter;
		int32 ActiveSupportAxes = 0;
		int8 BoxVertexIndex = 0;
		for (int32 AxisIndex = 0; AxisIndex < 3; ++AxisIndex)
		{
			const double DirectionProjection = FVector3d::DotProduct(
				- Normal, Axis[AxisIndex]);
			// Positive is the deterministic support-map tie break. It matches
			// the vertex convention used by the native box geometry and avoids
			// choosing an arbitrary triangulation vertex inside a face manifold.
			const double Sign = DirectionProjection >= 0.0 ? 1.0 : -1.0;
			if (Sign > 0.0) BoxVertexIndex |= 1 << AxisIndex;
			Witness.QueryPoint += Sign * Query.HalfExtent[AxisIndex] * Axis[AxisIndex];
			if (FMath::Abs(DirectionProjection) > 1.0e-10) ++ActiveSupportAxes;
		}
		Witness.QueryKind = ActiveSupportAxes <= 1 ? EContactFeatureKind::Face :
			ActiveSupportAxes == 2 ? EContactFeatureKind::Edge :
			EContactFeatureKind::Vertex;
		Witness.QueryIndex = BoxVertexIndex;
		Witness.SurfacePoint = ClosestPointOnTriangle(Witness.QueryPoint,
			Triangle.Vertex[0], Triangle.Vertex[1], Triangle.Vertex[2],
			&Witness.SurfaceKind, &Witness.SurfaceIndex);
		if (Witness.SurfaceKind == EContactFeatureKind::Vertex)
		{
			Witness.SurfaceIndex = static_cast<int8>(
				Triangle.PolygonVertex[Witness.SurfaceIndex]);
		}
		if (bStartPenetrating)
		{
			Witness.QueryPoint -= PenetrationDepth * Normal;
		}

		OutHit.bHit = true;
		OutHit.bStartPenetrating = bStartPenetrating;
		OutHit.Time = Time;
		OutHit.PenetrationDepth = bStartPenetrating ? PenetrationDepth : 0.0;
		OutHit.Location = Center;
		OutHit.Point = Witness.SurfacePoint;
		OutHit.QueryPoint = Witness.QueryPoint;
		OutHit.Normal = Normal;
		OutHit.QueryFeatureKind = Witness.QueryKind;
		OutHit.SurfaceFeatureKind = Witness.SurfaceKind;
		OutHit.QueryFeatureIndex = Witness.QueryIndex;
		OutHit.SurfaceFeatureIndex = Witness.SurfaceIndex;
		return true;
	}

	bool PointInTriangle(
		const FVector3d& Point,
		const FTriangleSurface& Triangle,
		const FVector3d& Normal,
		const double Tolerance)
	{
		for (int32 Edge = 0; Edge < 3; ++Edge)
		{
			const FVector3d& A = Triangle.Vertices[Edge];
			const FVector3d& B = Triangle.Vertices[(Edge + 1) % 3];
			if (FVector3d::DotProduct(
				FVector3d::CrossProduct(B - A, Point - A), Normal) < -Tolerance)
			{
				return false;
			}
		}
		return true;
	}

	FBox3d SweptQueryBounds(const FWorldQuery& Query)
	{
		FBox3d Bounds(EForceInit::ForceInit);
		Bounds += Query.Start;
		Bounds += Query.End;
		double Expansion = 0.0;
		if (Query.Shape == EQueryShape::Sphere)
		{
			Expansion = FMath::Max(0.0, Query.Radius);
		}
		else if (Query.Shape == EQueryShape::Box)
		{
			Expansion = Query.HalfExtent.Length();
		}
		return Bounds.ExpandBy(Expansion + Query.DomainTolerance);
	}
}

double FWorldQueryService::SupportRadius(
	const FWorldQuery& Query, const FVector3d& Normal)
{
	switch (Query.Shape)
	{
	case EQueryShape::Ray:
		return 0.0;
	case EQueryShape::Sphere:
		return FMath::Max(0.0, Query.Radius);
	case EQueryShape::Box:
	{
		const FVector3d AxisX = Query.Rotation.RotateVector(FVector3d::ForwardVector);
		const FVector3d AxisY = Query.Rotation.RotateVector(FVector3d::RightVector);
		const FVector3d AxisZ = Query.Rotation.RotateVector(FVector3d::UpVector);
		return FMath::Abs(FVector3d::DotProduct(AxisX, Normal)) * Query.HalfExtent.X +
			FMath::Abs(FVector3d::DotProduct(AxisY, Normal)) * Query.HalfExtent.Y +
			FMath::Abs(FVector3d::DotProduct(AxisZ, Normal)) * Query.HalfExtent.Z;
	}
	default:
		return 0.0;
	}
}

FWorldHit FWorldQueryService::SweepPlane(
	const FWorldQuery& Query, const FBoundedPlane& Plane)
{
	if (Query.Shape == EQueryShape::Box)
	{
		return SweepBoxPlane(Query, Plane);
	}
	FWorldHit Hit;
	FVector3d Normal = Plane.Normal;
	double RawStartDistance = Plane.SignedDistance(Query.Start);
	if (RawStartDistance < 0.0)
	{
		Normal = -Normal;
		RawStartDistance = -RawStartDistance;
	}
	const double Support = SupportRadius(Query, Normal);
	const double StartDistance = RawStartDistance - Support;
	const double EndDistance = FVector3d::DotProduct(
		Query.End - Plane.Origin, Normal) - Support;

	double Time = 1.0;
	if (StartDistance <= 0.0)
	{
		Time = 0.0;
		Hit.bStartPenetrating = StartDistance < 0.0;
		Hit.PenetrationDepth = FMath::Max(0.0, -StartDistance);
	}
	else
	{
		const double Denominator = StartDistance - EndDistance;
		if (Denominator <= 0.0 || EndDistance > 0.0)
		{
			return Hit;
		}
		Time = StartDistance / Denominator;
		if (Time < 0.0 || Time > 1.0)
		{
			return Hit;
		}
	}

	const FVector3d Center = FMath::Lerp(Query.Start, Query.End, Time);
	const FVector3d ContactPoint = Center - Support * Normal;
	if (!Plane.ContainsProjectedPoint(ContactPoint, Query.DomainTolerance))
	{
		return FWorldHit();
	}
	if (Query.bAuthorityOnly)
	{
		double RequiredInteriorMargin = 0.0;
		if (Query.Shape == EQueryShape::Sphere)
		{
			RequiredInteriorMargin = FMath::Max(0.0, Query.Radius);
		}
		if (Plane.DistanceToDomainBoundary(ContactPoint) +
			Query.DomainTolerance < RequiredInteriorMargin)
		{
			return FWorldHit();
		}
	}

	Hit.bHit = true;
	Hit.Time = Time;
	Hit.Location = Center;
	Hit.Point = Plane.ClosestPoint(ContactPoint);
	Hit.QueryPoint = ContactPoint;
	Hit.Normal = Normal;
	Hit.QueryFeatureKind = Query.Shape == EQueryShape::Sphere ?
		EContactFeatureKind::Vertex : EContactFeatureKind::Unknown;
	Hit.SurfaceFeatureKind = EContactFeatureKind::Face;
	Hit.SourceId = Plane.SourceId;
	Hit.SurfaceId = Plane.SurfaceId;
	Hit.FeatureId = Plane.FeatureId;
	Hit.PrimitiveId = Plane.PrimitiveId;
	Hit.MaterialId = Plane.MaterialId;
	return Hit;
}

FWorldHit FWorldQueryService::SweepBoxPlane(
	const FWorldQuery& Query, const FBoundedPlane& Plane)
{
	FWorldHit Best;
	TArray<FPlaneTriangle, TInlineAllocator<32>> Triangles;
	TriangulatePlaneDomain(Plane, Triangles);
	for (const FPlaneTriangle& Triangle : Triangles)
	{
		FWorldHit Candidate;
		if (!SweepBoxTriangle(Query, Triangle, Candidate)) continue;
		Candidate.SourceId = Plane.SourceId;
		Candidate.SurfaceId = Plane.SurfaceId;
		Candidate.FeatureId = Plane.FeatureId;
		Candidate.PrimitiveId = Plane.PrimitiveId;
		Candidate.MaterialId = Plane.MaterialId;
		if (IsBetterHit(Candidate, Best)) Best = Candidate;
	}
	return Best;
}

bool FWorldQueryService::TrianglePassesFilter(
	const FWorldQuery& Query, const FTriangleSurface& Triangle)
{
	if (!Query.bApplyCollisionFilter)
	{
		return true;
	}
	if (!Triangle.bQueryCollisionEnabled)
	{
		return false;
	}
	if (Query.bObjectQuery)
	{
		return Triangle.ObjectType < 64 &&
			(Query.ObjectTypes & (1ull << Triangle.ObjectType)) != 0;
	}
	return Query.TraceChannel < 64 &&
		(Triangle.BlockingChannels & (1ull << Query.TraceChannel)) != 0 &&
		Triangle.ObjectType < 64 &&
		(Query.BlockingObjectTypes & (1ull << Triangle.ObjectType)) != 0;
}

bool FWorldQueryService::PatchPassesFilter(
	const FWorldQuery& Query, const FExtrudedQuinticPatch& Patch)
{
	if (!Query.bApplyCollisionFilter)
	{
		return true;
	}
	if (!Patch.bQueryCollisionEnabled)
	{
		return false;
	}
	if (Query.bObjectQuery)
	{
		return Patch.ObjectType < 64 &&
			(Query.ObjectTypes & (1ull << Patch.ObjectType)) != 0;
	}
	return Query.TraceChannel < 64 &&
		(Patch.BlockingChannels & (1ull << Query.TraceChannel)) != 0 &&
		Patch.ObjectType < 64 &&
		(Query.BlockingObjectTypes & (1ull << Patch.ObjectType)) != 0;
}

bool FWorldQueryService::PlanePassesFilter(
	const FWorldQuery& Query, const FBoundedPlane& Plane)
{
	if (Plane.bRequiresCompactOptIn && !Query.bIncludeCompactPatches)
	{
		return false;
	}
	if (!Query.bApplyCollisionFilter) return true;
	if (!Plane.bQueryCollisionEnabled) return false;
	if (Query.bObjectQuery)
	{
		return Plane.ObjectType < 64 &&
			(Query.ObjectTypes & (1ull << Plane.ObjectType)) != 0;
	}
	return Query.TraceChannel < 64 &&
		(Plane.BlockingChannels & (1ull << Query.TraceChannel)) != 0 &&
		Plane.ObjectType < 64 &&
		(Query.BlockingObjectTypes & (1ull << Plane.ObjectType)) != 0;
}

FWorldHit FWorldQueryService::SweepExtrudedQuintic(
	const FWorldQuery& Query, const FBox3d& QueryBounds,
	const FExtrudedQuinticPatch& Patch)
{
	FWorldHit Best;
	if (!PatchPassesFilter(Query, Patch))
	{
		return Best;
	}
	if (!Patch.Bounds.Intersect(QueryBounds))
	{
		return Best;
	}
	if (Patch.SectionPolyline.Num() < 2 ||
		Patch.SectionPolyline.Num() != Patch.SectionParameters.Num() ||
		!FMath::IsFinite(Patch.MaximumChordErrorCm))
	{
		return Best;
	}
	const double ExtrusionCenter = 0.5 *
		(Patch.MinimumExtrusionCoordinate + Patch.MaximumExtrusionCoordinate);
	const double ExtrusionHalfExtent = 0.5 *
		(Patch.MaximumExtrusionCoordinate - Patch.MinimumExtrusionCoordinate);
	FVector3d SectionA = Patch.SectionPolyline[0];
	for (int32 Segment = 0; Segment + 1 < Patch.SectionPolyline.Num(); ++Segment)
	{
		const FVector3d SectionB = Patch.SectionPolyline[Segment + 1];
		const FVector3d Chord = SectionB - SectionA;
		const double ChordLength = Chord.Length();
		if (ChordLength <= UE_DOUBLE_SMALL_NUMBER)
		{
			SectionA = SectionB;
			continue;
		}
		FBoundedPlane Face;
		Face.SurfaceId = Patch.SurfaceId;
		Face.SourceId = Patch.SourceId;
		Face.FeatureId = Patch.FeatureId;
		Face.MaterialId = Patch.MaterialId;
		Face.AxisU = Chord / ChordLength;
		Face.AxisV = Patch.ExtrusionAxis;
		Face.Normal = FVector3d::CrossProduct(Face.AxisU, Face.AxisV).GetSafeNormal();
		Face.Origin = 0.5 * (SectionA + SectionB) +
			ExtrusionCenter * Patch.ExtrusionAxis;
		if (FVector3d::DotProduct(Query.Start - Face.Origin, Face.Normal) < 0.0)
		{
			Face.Normal = -Face.Normal;
		}
		Face.HalfExtents = FVector2d(0.5 * ChordLength, ExtrusionHalfExtent);
		// Chord endpoints inside the adaptive polyline are approximation seams,
		// not authored surface boundaries.  SweepPlane's authority margin is
		// correct for a standalone bounded plane, but applying it to every chord
		// excludes any sphere whose footprint crosses an internal seam.  Evaluate
		// the finite face first, then retain the footprint margin only at the four
		// actual patch boundaries (the two section endpoints and two extrusion
		// endpoints).
		FWorldQuery FaceQuery = Query;
		FaceQuery.bAuthorityOnly = false;
		FWorldHit Candidate = SweepPlane(FaceQuery, Face);
		if (Candidate.bHit && Query.bAuthorityOnly &&
			Query.Shape == EQueryShape::Sphere)
		{
			const double RequiredMargin = FMath::Max(0.0, Query.Radius);
			const double ExtrusionCoordinate = FVector3d::DotProduct(
				Candidate.Point, Patch.ExtrusionAxis);
			const double AlongChord = FVector3d::DotProduct(
				Candidate.Point - SectionA, Face.AxisU);
			const bool bCrossesExtrusionBoundary =
				ExtrusionCoordinate + Query.DomainTolerance <
					Patch.MinimumExtrusionCoordinate + RequiredMargin ||
				ExtrusionCoordinate - Query.DomainTolerance >
					Patch.MaximumExtrusionCoordinate - RequiredMargin;
			const bool bCrossesSectionStart = Segment == 0 &&
				AlongChord + Query.DomainTolerance < RequiredMargin;
			const bool bCrossesSectionEnd =
				Segment + 2 == Patch.SectionPolyline.Num() &&
				AlongChord - Query.DomainTolerance > ChordLength - RequiredMargin;
			if (bCrossesExtrusionBoundary || bCrossesSectionStart ||
				bCrossesSectionEnd)
			{
				Candidate = FWorldHit();
			}
		}
		Candidate.PrimitiveId = CombineStableIds(
			Patch.PrimitiveId, static_cast<uint64>(Segment + 1));
		Candidate.CanonicalGroupId = Patch.CanonicalGroupId;
		Candidate.GeometricErrorBoundCm = Patch.MaximumChordErrorCm;
		if (Candidate.bHit && !Candidate.bStartPenetrating &&
			Candidate.SurfaceFeatureKind == EContactFeatureKind::Face)
		{
			// The certified chords provide conservative finite-domain TOI and
			// witnesses, but an internal chord boundary is not a physical crease.
			// Report the authored curve derivative so suspension/support solvers
			// see a continuous normal while the positional witness remains covered
			// by GeometricErrorBoundCm. Initial-overlap MTDs deliberately retain
			// their SAT axis because changing it would invalidate the measured depth.
			const double ChordAlpha = FMath::Clamp(
				FVector3d::DotProduct(Candidate.Point - SectionA, Chord) /
					FMath::Square(ChordLength),
				0.0, 1.0);
			const double SectionT = FMath::Lerp(
				Patch.SectionParameters[Segment],
				Patch.SectionParameters[Segment + 1], ChordAlpha);
			const FVector3d SmoothTangent =
				Patch.EvaluateSectionDerivative(SectionT).GetSafeNormal();
			FVector3d SmoothNormal = FVector3d::CrossProduct(
				SmoothTangent, Patch.ExtrusionAxis).GetSafeNormal();
			if (!SmoothNormal.IsNearlyZero())
			{
				if (FVector3d::DotProduct(SmoothNormal, Candidate.Normal) < 0.0)
				{
					SmoothNormal = -SmoothNormal;
				}
				Candidate.Normal = SmoothNormal;
			}
		}
		if (Candidate.bHit && IsBetterHit(Candidate, Best))
		{
			Best = Candidate;
		}
		SectionA = SectionB;
	}
	return Best;
}

FWorldHit FWorldQueryService::SweepTriangleFace(
	const FWorldQuery& Query, const FTriangleSurface& Triangle)
{
	FWorldHit Hit;
	if (!TrianglePassesFilter(Query, Triangle))
	{
		return Hit;
	}
	FVector3d Normal = Triangle.FaceNormal;
	double RawStartDistance = FVector3d::DotProduct(
		Query.Start - Triangle.Vertices[0], Normal);
	if (RawStartDistance < 0.0)
	{
		Normal = -Normal;
		RawStartDistance = -RawStartDistance;
	}
	const double Support = SupportRadius(Query, Normal);
	const double StartDistance = RawStartDistance - Support;
	const double EndDistance = FVector3d::DotProduct(
		Query.End - Triangle.Vertices[0], Normal) - Support;
	double Time = 1.0;
	if (StartDistance <= 0.0)
	{
		Time = 0.0;
		Hit.bStartPenetrating = StartDistance < 0.0;
		Hit.PenetrationDepth = FMath::Max(0.0, -StartDistance);
	}
	else
	{
		const double Denominator = StartDistance - EndDistance;
		if (Denominator <= 0.0 || EndDistance > 0.0)
		{
			return Hit;
		}
		Time = StartDistance / Denominator;
		if (Time < 0.0 || Time > 1.0)
		{
			return Hit;
		}
	}

	const FVector3d Center = FMath::Lerp(Query.Start, Query.End, Time);
	const FVector3d ContactPoint = Center - Support * Normal;
	const FVector3d PlanePoint = ContactPoint - FVector3d::DotProduct(
		ContactPoint - Triangle.Vertices[0], Normal) * Normal;
	// The contact normal is flipped toward the query for two-sided collision,
	// but the triangle domain keeps the source winding.  Testing the domain
	// against the flipped normal rejects every valid back-face contact.
	if (!PointInTriangle(
		PlanePoint, Triangle, Triangle.FaceNormal, Query.DomainTolerance))
	{
		return FWorldHit();
	}
	Hit.bHit = true;
	Hit.Time = Time;
	Hit.Location = Center;
	Hit.Point = PlanePoint;
	Hit.QueryPoint = ContactPoint;
	Hit.Normal = Normal;
	Hit.QueryFeatureKind = Query.Shape == EQueryShape::Sphere ?
		EContactFeatureKind::Vertex : EContactFeatureKind::Unknown;
	Hit.SurfaceFeatureKind = EContactFeatureKind::Face;
	Hit.SourceId = Triangle.SourceId;
	Hit.SurfaceId = Triangle.SurfaceId;
	Hit.FeatureId = Triangle.FeatureId;
	Hit.PrimitiveId = Triangle.PrimitiveId;
	Hit.MaterialId = Triangle.MaterialId;
	return Hit;
}

bool FWorldQueryService::IsBetterHit(
	const FWorldHit& Candidate, const FWorldHit& Best)
{
	return !Best.bHit || Candidate.Time < Best.Time - 1.0e-12 ||
		(FMath::IsNearlyEqual(Candidate.Time, Best.Time, 1.0e-12) &&
			((Candidate.QueryPoint - Candidate.Point).SquaredLength() <
				(Best.QueryPoint - Best.Point).SquaredLength() - 1.0e-12 ||
			(FMath::IsNearlyEqual(
				(Candidate.QueryPoint - Candidate.Point).SquaredLength(),
				(Best.QueryPoint - Best.Point).SquaredLength(), 1.0e-12) &&
			(Candidate.SurfaceId < Best.SurfaceId ||
				(Candidate.SurfaceId == Best.SurfaceId &&
					(Candidate.FeatureId < Best.FeatureId ||
						(Candidate.FeatureId == Best.FeatureId &&
							Candidate.PrimitiveId < Best.PrimitiveId)))))));
}

bool FWorldQueryService::HasAuthorityCoverage(const FWorldQuery& Query) const
{
	static thread_local bool bLoggedFirstCoverageMismatch = false;
	// AnalyticHybrid is a migration tool. The certified primitive must produce
	// the winning hit not only inside its own broad phase, but also against every
	// known draft provider. This keeps an underlying certified surface from
	// replacing a nearer, not-yet-certified authored surface. SurfaceAnalytic
	// instead requires a complete-world certificate and treats a miss as final.
	FWorldQuery AuthorityQuery = Query;
	AuthorityQuery.bAuthorityOnly = true;
	const FWorldHit AuthorityHit = Sweep(AuthorityQuery);
	if (!AuthorityHit.bHit)
	{
		if (!bLoggedFirstCoverageMismatch)
		{
			bLoggedFirstCoverageMismatch = true;
			UE_LOG(LogTemp, Display,
				TEXT("[AnalyticAuthorityCoverageMismatch] Shape=%u AuthorityHit=0 Start=%s End=%s HalfExtent=%s Radius=%.17g"),
				static_cast<uint8>(Query.Shape),
				*FVector(Query.Start).ToString(), *FVector(Query.End).ToString(),
				*FVector(Query.HalfExtent).ToString(), Query.Radius);
		}
		return false;
	}

	FWorldQuery ProviderQuery = Query;
	ProviderQuery.bAuthorityOnly = false;
	ProviderQuery.bIncludeCompactPatches = true;
	ProviderQuery.bIncludeTriangles = true;
	const FWorldHit ProviderHit = Sweep(ProviderQuery);
	constexpr double ProviderTimeTolerance = 1.0e-9;
	const bool bCovered = ProviderHit.bHit &&
		ProviderHit.SourceId == AuthorityHit.SourceId &&
		AuthorityHit.Time <= ProviderHit.Time + ProviderTimeTolerance;
	if (!bCovered && !bLoggedFirstCoverageMismatch)
	{
		bLoggedFirstCoverageMismatch = true;
		UE_LOG(LogTemp, Display,
			TEXT("[AnalyticAuthorityCoverageMismatch] Shape=Box Start=%s End=%s HalfExtent=%s AuthorityHit=%d AuthorityTime=%.17g AuthoritySource=%016llX AuthoritySurface=%016llX AuthorityPrimitive=%016llX ProviderHit=%d ProviderTime=%.17g ProviderSource=%016llX ProviderSurface=%016llX ProviderPrimitive=%016llX"),
			*FVector(Query.Start).ToString(), *FVector(Query.End).ToString(),
			*FVector(Query.HalfExtent).ToString(), AuthorityHit.bHit ? 1 : 0,
			AuthorityHit.Time, AuthorityHit.SourceId, AuthorityHit.SurfaceId,
			AuthorityHit.PrimitiveId, ProviderHit.bHit ? 1 : 0, ProviderHit.Time,
			ProviderHit.SourceId, ProviderHit.SurfaceId, ProviderHit.PrimitiveId);
	}
	return bCovered;
}

FWorldHit FWorldQueryService::Sweep(const FWorldQuery& Query) const
{
	FWorldHit Best;
	const FBox3d QueryBounds = SweptQueryBounds(Query);
	for (const FBoundedPlane& Plane : World.Planes)
	{
		if (Plane.bRequiresCompactOptIn) continue;
		if (Query.bAuthorityOnly && !Plane.bAuthorityEligible) continue;
		if (!PlanePassesFilter(Query, Plane)) continue;
		if (!Plane.Bounds.Intersect(QueryBounds)) continue;
		const FWorldHit Candidate = SweepPlane(Query, Plane);
		if (!Candidate.bHit)
		{
			continue;
		}
		if (IsBetterHit(Candidate, Best))
		{
			Best = Candidate;
		}
	}
	if (Query.bIncludeCompactPatches && !World.CompactBvh.IsEmpty())
	{
		TArray<int32, TInlineAllocator<16>> Stack;
		Stack.Add(0);
		while (!Stack.IsEmpty())
		{
			const FTriangleBvhNode& Node = World.CompactBvh[
				Stack.Pop(EAllowShrinking::No)];
			if (!Node.Bounds.Intersect(QueryBounds)) continue;
			if (!Node.IsLeaf())
			{
				Stack.Add(Node.RightChild);
				Stack.Add(Node.LeftChild);
				continue;
			}
			for (int32 Offset = 0; Offset < Node.IndexCount; ++Offset)
			{
				const int32 EncodedIndex = World.CompactPrimitiveIndices[
					Node.FirstIndex + Offset];
				FWorldHit Candidate;
				if (EncodedIndex < World.Planes.Num())
				{
					const FBoundedPlane& Plane = World.Planes[EncodedIndex];
					if (Query.bAuthorityOnly && !Plane.bAuthorityEligible) continue;
					if (!PlanePassesFilter(Query, Plane) ||
						!Plane.Bounds.Intersect(QueryBounds)) continue;
					Candidate = SweepPlane(Query, Plane);
				}
				else
				{
					const FExtrudedQuinticPatch& Patch =
						World.ExtrudedQuinticPatches[
							EncodedIndex - World.Planes.Num()];
					if (Query.bAuthorityOnly && !Patch.bAuthorityEligible) continue;
					Candidate = SweepExtrudedQuintic(Query, QueryBounds, Patch);
				}
				if (Candidate.bHit && IsBetterHit(Candidate, Best)) Best = Candidate;
			}
		}
	}
	if (Query.bIncludeTriangles && !World.TriangleBvh.IsEmpty())
	{
		TArray<int32, TInlineAllocator<64>> Stack;
		Stack.Add(0);
		while (!Stack.IsEmpty())
		{
			const int32 NodeIndex = Stack.Pop(EAllowShrinking::No);
			const FTriangleBvhNode& Node = World.TriangleBvh[NodeIndex];
			if (!Node.Bounds.Intersect(QueryBounds))
			{
				continue;
			}
			if (!Node.IsLeaf())
			{
				Stack.Add(Node.RightChild);
				Stack.Add(Node.LeftChild);
				continue;
			}
			for (int32 Offset = 0; Offset < Node.IndexCount; ++Offset)
			{
				const FTriangleSurface& Triangle = World.Triangles[
					World.TriangleIndices[Node.FirstIndex + Offset]];
				if (Query.bAuthorityOnly && !Triangle.bAuthorityEligible) continue;
				const FWorldHit Candidate = SweepTriangleFace(Query, Triangle);
				if (Candidate.bHit && IsBetterHit(Candidate, Best))
				{
					Best = Candidate;
				}
			}
		}
	}
	return Best;
}

} // namespace Speed::Analytic
