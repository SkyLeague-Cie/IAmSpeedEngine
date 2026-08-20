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

	Hit.bHit = true;
	Hit.Time = Time;
	Hit.Location = Center;
	Hit.Point = Plane.ClosestPoint(ContactPoint);
	Hit.Normal = Normal;
	Hit.SourceId = Plane.SourceId;
	Hit.SurfaceId = Plane.SurfaceId;
	Hit.FeatureId = Plane.FeatureId;
	Hit.PrimitiveId = Plane.PrimitiveId;
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
	constexpr int32 SectionSegmentCount = 32;
	const double ExtrusionCenter = 0.5 *
		(Patch.MinimumExtrusionCoordinate + Patch.MaximumExtrusionCoordinate);
	const double ExtrusionHalfExtent = 0.5 *
		(Patch.MaximumExtrusionCoordinate - Patch.MinimumExtrusionCoordinate);
	FVector3d SectionA = Patch.EvaluateSection(0.0);
	for (int32 Segment = 0; Segment < SectionSegmentCount; ++Segment)
	{
		const FVector3d SectionB = Patch.EvaluateSection(
			static_cast<double>(Segment + 1) / SectionSegmentCount);
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
		FWorldHit Candidate = SweepPlane(Query, Face);
		Candidate.PrimitiveId = CombineStableIds(
			Patch.PrimitiveId, static_cast<uint64>(Segment + 1));
		Candidate.CanonicalGroupId = Patch.CanonicalGroupId;
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
	Hit.Normal = Normal;
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
			(Candidate.SurfaceId < Best.SurfaceId ||
				(Candidate.SurfaceId == Best.SurfaceId &&
					(Candidate.FeatureId < Best.FeatureId ||
						(Candidate.FeatureId == Best.FeatureId &&
							Candidate.PrimitiveId < Best.PrimitiveId)))));
}

bool FWorldQueryService::HasAuthorityCoverage(const FWorldQuery& Query) const
{
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
		return false;
	}

	FWorldQuery ProviderQuery = Query;
	ProviderQuery.bAuthorityOnly = false;
	ProviderQuery.bIncludeCompactPatches = true;
	ProviderQuery.bIncludeTriangles = true;
	const FWorldHit ProviderHit = Sweep(ProviderQuery);
	return ProviderHit.bHit && ProviderHit.SourceId == AuthorityHit.SourceId;
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
