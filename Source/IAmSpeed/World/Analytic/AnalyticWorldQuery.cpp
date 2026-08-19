#include "AnalyticWorldQuery.h"

namespace Speed::Analytic
{
namespace
{
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
	FWorldHit Hit;
	const double Support = SupportRadius(Query, Plane.Normal);
	const double StartDistance = Plane.SignedDistance(Query.Start) - Support;
	const double EndDistance = Plane.SignedDistance(Query.End) - Support;

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
	const FVector3d ContactPoint = Center - Support * Plane.Normal;
	if (!Plane.ContainsProjectedPoint(ContactPoint, Query.DomainTolerance))
	{
		return FWorldHit();
	}

	Hit.bHit = true;
	Hit.Time = Time;
	Hit.Location = Center;
	Hit.Point = Plane.ClosestPoint(ContactPoint);
	Hit.Normal = Plane.Normal;
	Hit.SurfaceId = Plane.SurfaceId;
	Hit.FeatureId = Plane.FeatureId;
	Hit.MaterialId = Plane.MaterialId;
	return Hit;
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
		(Triangle.BlockingChannels & (1ull << Query.TraceChannel)) != 0;
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
	Hit.Normal = Normal;
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
			(Candidate.SurfaceId < Best.SurfaceId ||
				(Candidate.SurfaceId == Best.SurfaceId &&
					(Candidate.FeatureId < Best.FeatureId ||
						(Candidate.FeatureId == Best.FeatureId &&
							Candidate.PrimitiveId < Best.PrimitiveId)))));
}

FWorldHit FWorldQueryService::Sweep(const FWorldQuery& Query) const
{
	FWorldHit Best;
	for (const FBoundedPlane& Plane : World.Planes)
	{
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
	if (Query.bIncludeTriangles && !World.TriangleBvh.IsEmpty())
	{
		FBox3d QueryBounds(EForceInit::ForceInit);
		QueryBounds += Query.Start;
		QueryBounds += Query.End;
		double Expansion = 0.0;
		if (Query.Shape == EQueryShape::Sphere)
		{
			Expansion = FMath::Max(0.0, Query.Radius);
		}
		else if (Query.Shape == EQueryShape::Box)
		{
			Expansion = Query.HalfExtent.Length();
		}
		QueryBounds = QueryBounds.ExpandBy(Expansion + Query.DomainTolerance);
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
