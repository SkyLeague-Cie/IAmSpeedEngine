#pragma once

#include "AnalyticWorldData.h"
#include "IAmSpeed/Base/ContactFeature.h"

namespace Speed::Collision { class FOrderedBoundsIndex; }
#if WITH_DEV_AUTOMATION_TESTS
class FIAmSpeedPlaneSweepTest;
#endif

namespace Speed::Analytic
{

namespace Private { struct FBoxSweepContext; }

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
	// Optional stable provider restriction used to reacquire an established
	// contact without allowing an adjacent primitive to replace it implicitly.
	uint64 RequiredSourceId = 0;
	uint64 RequiredSurfaceId = 0;
	uint64 RequiredCanonicalGroupId = 0;
	// Optional continuity restriction for reacquiring an already-established
	// smooth contact. Fresh collision queries leave this disabled so a genuine
	// new impact is never discarded because of a previous contact normal.
	FVector3d ReferenceNormal = FVector3d::ZeroVector;
	double MinimumReferenceNormalDot = -1.0;
	// An established contact may retain an exact face hit whose witness remains
	// inside a finite domain even when the query footprint overlaps a certified
	// adjacent surface. Fresh authority queries keep the full-footprint margin.
	bool bAllowEstablishedFaceContactAtBoundary = false;
	bool bObjectQuery = false;
	bool bApplyCollisionFilter = false;
	// Authority execution may consume only explicitly certified primitives.
	// Shadow queries intentionally leave this false to compare draft geometry.
	bool bAuthorityOnly = false;
	// Provider-selection follow-up queries may skip certified primitives already
	// evaluated by the authority pass. TrySweepAuthority preserves the certified
	// residual-triangle winner separately and merges it into the draft result.
	bool bExcludeAuthorityEligible = false;
	bool bIncludeCompactPatches = false;
	bool bIncludeTriangles = false;
	/** True only when the query-side filter alone excludes every possible provider. */
	bool RejectsAllCollision() const
	{
		return bApplyCollisionFilter && (bObjectQuery
			? ObjectTypes == 0
			: (BlockingObjectTypes == 0 || TraceChannel >= 64));
	}
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
	// Provider-local allowance for Hybrid comparison with a residual source
	// representation. This does not enlarge strict query geometry or its
	// approximation error; zero retains the public classification tolerance.
	double AdditionalResidualAgreementAllowanceCm = 0.0;
	// True when advancing along the same authored primitive may change its
	// contact normal. Solvers must reacquire such contacts instead of extending
	// a previous tangent plane across simulation frames.
	bool bSurfaceNormalMayVary = false;
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
#if WITH_DEV_AUTOMATION_TESTS
	friend class ::FIAmSpeedPlaneSweepTest;
#endif
public:
	explicit FWorldQueryService(const FAnalyticWorldData& InWorld);

	bool HasAuthorityCoverage(const FWorldQuery& Query) const;
	bool TrySweepAuthority(
		const FWorldQuery& Query, FWorldHit& OutAuthorityHit) const;
	FWorldHit Sweep(const FWorldQuery& Query) const;
	/** Uses immutable bounds to visit every eligible plane near a conservative
	 * sphere trajectory. Returned plane witnesses are candidates, never TOIs. */
	void VisitPlanarCandidates(const FWorldQuery& Query, TFunctionRef<void(const FWorldHit&)> Visitor) const;
	/** Each support point must retain an exact finite planar provider over the
	 * whole translation. A seam requiring provider handover is conservatively refused. */
	bool IsPlanarSupportTranslationCertified(const FWorldQuery& Query,
		TConstArrayView<FVector3d> Points, const FVector3d& Normal, const FVector3d& Translation) const;

private:
	struct FSourceAuthorityCoverage
	{
		uint64 SourceId = 0;
		FBox3d Bounds = FBox3d(EForceInit::ForceInit);
		uint64 ObjectTypes = 0;
		uint64 BlockingChannels = 0;
		bool bHasQueryPrimitive = false;
		bool bHasQueryTriangle = false;
		bool bAllQueryTrianglesAuthorityEligible = true;
	};
	struct FTensorBezierQueryView
	{
		uint64 SourceId = 0;
		uint64 SurfaceId = 0;
		uint64 FeatureId = 0;
		uint64 PrimitiveId = 0;
		uint64 CanonicalGroupId = 0;
		uint32 MaterialId = 0;
		uint32 ObjectType = 0;
		uint64 BlockingChannels = 0;
		const FTensorBezierSurface* Surface = nullptr;
		FBox3d Bounds = FBox3d(EForceInit::ForceInit);
		const TArray<FTensorBezierApproximationCell>* ApproximationCells = nullptr;
		const TArray<FTriangleBvhNode>* ApproximationCellBvhNodes = nullptr;
		const TArray<int32>* ApproximationCellBvhIndices = nullptr;
		bool bQueryCollisionEnabled = false;
		bool bApproximationCertified = false;
		bool bAuthorityEligible = false;
	};

	const FAnalyticWorldData& World;
	// Constructed from this service's immutable world; no spatial order reaches the solver.
	TSharedPtr<const Speed::Collision::FOrderedBoundsIndex> PiecewiseProviderIndex;
	TSharedPtr<const Speed::Collision::FOrderedBoundsIndex> PlaneProviderIndex;
	TArray<FSourceAuthorityCoverage> SourceAuthorityCoverage;
	struct FCachedSweep
	{
		FWorldQuery Query;
		FWorldHit Hit;
		bool bValid = false;
	};
	// The analytical world is immutable and canonical physics owns one query
	// lane. Exactly identical requests therefore have the same result. A small
	// fixed table retains the interleaved wheel/hitbox projection probes without
	// introducing a tolerance, eviction-dependent result, or unbounded storage.
	static constexpr uint8 SweepCacheCapacity = 64;
	static constexpr uint8 SweepCacheWays = 4;
	mutable FCachedSweep SweepCache[SweepCacheCapacity];
	mutable uint8 NextCacheWay[SweepCacheCapacity / SweepCacheWays] = {};
	static_assert(sizeof(SweepCache) + sizeof(NextCacheWay) <= 64 * 1024, "Keep the per-world query cache bounded to 64 KiB");
#if !UE_BUILD_SHIPPING
	struct FCacheProfile
	{
		uint64 Queries = 0;
		uint64 Hits[3] = {};
		uint64 Misses[3] = {};
		uint64 HitWays[SweepCacheWays] = {};
		uint64 Evictions = 0;
	};
	mutable FCacheProfile CacheProfile;
	static_assert(sizeof(FCacheProfile) <= 128, "Keep opt-in cache attribution bounded");
	/** Counts full-query cache reuse only; empty filters and direct detailed queries are excluded. */
	void RecordCacheProfile(EQueryShape Shape, int32 HitWay, bool bEvicted) const;
#endif
	FWorldHit SweepDetailed(
		const FWorldQuery& Query, FWorldHit* OutBestTriangle) const;
	bool IsAuthorityMissDefinitive(
		const FWorldQuery& Query, const FBox3d& QueryBounds) const;
	static double SupportRadius(const FWorldQuery& Query, const FVector3d& Normal);
	static FWorldHit SweepPlane(
		const FWorldQuery& Query, const FBoundedPlane& Plane,
		const FAnalyticWorldData* PlaneUnionWorld = nullptr,
		const Private::FBoxSweepContext* CachedBoxContext = nullptr);
	/** Ray/sphere plane kernel: a miss leaves OutHit untouched; a hit replaces every field. */
	static bool TrySweepRoundPlane(
		const FWorldQuery& Query, const FBoundedPlane& Plane,
		FWorldHit& OutHit, const FAnalyticWorldData* PlaneUnionWorld = nullptr);
	static double DistanceToCoplanarSemanticUnionBoundary(
		const FAnalyticWorldData& UnionWorld, const FWorldQuery& Query,
		const FBoundedPlane& SeedPlane, const FVector3d& Point);
	static FWorldHit SweepBoxPlane(
		const FWorldQuery& Query, const FBoundedPlane& Plane,
		const Private::FBoxSweepContext* CachedBoxContext = nullptr);
	static FWorldHit SweepTriangleFace(
		const FWorldQuery& Query, const FTriangleSurface& Triangle);
	static FWorldHit SweepExtrudedQuintic(
		const FWorldQuery& Query, const FBox3d& QueryBounds,
		const FExtrudedQuinticPatch& Patch);
	static FWorldHit SweepTensorBezier(
		const FWorldQuery& Query, const FBox3d& QueryBounds,
		const FTensorBezierPatch& Patch,
		double MaximumSearchTime = 1.0);
	static FWorldHit SweepTensorBezierApproximation(
		const FWorldQuery& Query, const FBox3d& QueryBounds,
		const FTensorBezierQueryView& Patch,
		double MaximumSearchTime = 1.0,
		const FVector3d* CachedBoxAxes = nullptr);
	static FWorldHit SweepPiecewiseTensorBezier(
		const FWorldQuery& Query, const FBox3d& QueryBounds,
		const FPiecewiseTensorBezierPatch& Patch,
		double MaximumSearchTime = 1.0);
	static bool TrianglePassesFilter(
		const FWorldQuery& Query, const FTriangleSurface& Triangle);
	static bool PatchPassesFilter(
		const FWorldQuery& Query, const FExtrudedQuinticPatch& Patch);
	static bool PatchPassesFilter(
		const FWorldQuery& Query, const FTensorBezierPatch& Patch);
	static bool PatchPassesFilter(
		const FWorldQuery& Query, const FPiecewiseTensorBezierPatch& Patch);
	static bool PlanePassesFilter(
		const FWorldQuery& Query, const FBoundedPlane& Plane);
	static bool IsSameQuery(const FWorldQuery& A, const FWorldQuery& B);
	static bool IsBetterHit(const FWorldHit& Candidate, const FWorldHit& Best);
	static bool HitPassesReferenceNormal(
		const FWorldQuery& Query, const FWorldHit& Hit);
};

} // namespace Speed::Analytic
