#pragma once

#include "AnalyticWorldData.h"
#include "IAmSpeed/Base/ContactFeature.h"

namespace Speed::Analytic
{

using EContactFeatureKind = Speed::EContactFeatureKind;

enum class EQueryShape : uint8
{
	Ray = 0,
	Sphere = 1,
	Box = 2,
};

// Geometric classification of the two witnesses carried by a query result.
// This is deliberately independent from the authored FeatureId: the latter
// identifies a stable surface feature, while this describes the local contact
// manifold selected for the current sweep.
struct IAMSPEED_API FWorldQuery
{
	EQueryShape Shape = EQueryShape::Ray;
	FVector3d Start = FVector3d::ZeroVector;
	FVector3d End = FVector3d::ZeroVector;
	FQuat4d Rotation = FQuat4d::Identity;
	double Radius = 0.0;
	FVector3d HalfExtent = FVector3d::ZeroVector;
	double DomainTolerance = 1.0e-6;
	// Initial overlaps no deeper than this contact shell are released when the
	// query is stationary, tangent, or separating. Closing motion still yields
	// a time-zero blocking contact so the shell cannot be crossed inward.
	double InitialOverlapTolerance = 0.05;
	uint8 TraceChannel = 0;
	uint64 ObjectTypes = 0;
	// Query-side response mask. Trace queries require both collision responses
	// to block; object queries continue to use ObjectTypes directly.
	uint64 BlockingObjectTypes = MAX_uint64;
	bool bObjectQuery = false;
	bool bApplyCollisionFilter = false;
	// Authority execution may consume only explicitly certified primitives.
	// Shadow queries intentionally leave this false to compare draft geometry.
	bool bAuthorityOnly = false;
	bool bIncludeCompactPatches = false;
	bool bIncludeTriangles = false;
};

struct IAMSPEED_API FWorldHit
{
	bool bHit = false;
	bool bStartPenetrating = false;
	double Time = 1.0;
	double PenetrationDepth = 0.0;
	// Zero for exact primitives. Positive values bound the positional
	// approximation error of the analytical provider in centimetres.
	double GeometricErrorBoundCm = 0.0;
	FVector3d Location = FVector3d::ZeroVector;
	// Witness on the static analytical primitive.
	FVector3d Point = FVector3d::ZeroVector;
	// Witness on the moving query shape. At a non-penetrating TOI this equals
	// Point within numerical tolerance. For an initial overlap, Point-
	// QueryPoint is the minimum-translation vector represented by Normal and
	// PenetrationDepth.
	FVector3d QueryPoint = FVector3d::ZeroVector;
	FVector3d Normal = FVector3d::ZeroVector;
	Speed::EContactFeatureKind QueryFeatureKind = Speed::EContactFeatureKind::Unknown;
	Speed::EContactFeatureKind SurfaceFeatureKind = Speed::EContactFeatureKind::Unknown;
	int8 QueryFeatureIndex = INDEX_NONE;
	int8 SurfaceFeatureIndex = INDEX_NONE;
	uint64 SourceId = 0;
	uint64 SurfaceId = 0;
	uint64 FeatureId = 0;
	uint64 PrimitiveId = 0;
	uint64 CanonicalGroupId = 0;
	uint32 MaterialId = 0;
};

class IAMSPEED_API FWorldQueryService
{
public:
	explicit FWorldQueryService(const FAnalyticWorldData& InWorld) : World(InWorld) {}

	bool HasAuthorityCoverage(const FWorldQuery& Query) const;
	FWorldHit Sweep(const FWorldQuery& Query) const;

private:
	const FAnalyticWorldData& World;
	static double SupportRadius(const FWorldQuery& Query, const FVector3d& Normal);
	static FWorldHit SweepPlane(const FWorldQuery& Query, const FBoundedPlane& Plane);
	static FWorldHit SweepBoxPlane(
		const FWorldQuery& Query, const FBoundedPlane& Plane);
	static FWorldHit SweepTriangleFace(
		const FWorldQuery& Query, const FTriangleSurface& Triangle);
	static FWorldHit SweepExtrudedQuintic(
		const FWorldQuery& Query, const FBox3d& QueryBounds,
		const FExtrudedQuinticPatch& Patch);
	static bool TrianglePassesFilter(
		const FWorldQuery& Query, const FTriangleSurface& Triangle);
	static bool PatchPassesFilter(
		const FWorldQuery& Query, const FExtrudedQuinticPatch& Patch);
	static bool PlanePassesFilter(
		const FWorldQuery& Query, const FBoundedPlane& Plane);
	static bool IsBetterHit(const FWorldHit& Candidate, const FWorldHit& Best);
};

} // namespace Speed::Analytic
