#pragma once

#include "IAmSpeed/World/Analytic/AnalyticWorldQuery.h"
#include "StaticQueryBatch.h"
#include <atomic>

namespace Speed
{

enum class EStaticCollisionBackend : uint8
{
	UnrealLegacy = 0,
	AnalyticHybrid,
	SurfaceAnalytic,
};

class IAMSPEED_API IStaticCollisionWorld
{
public:
	virtual ~IStaticCollisionWorld() = default;
	virtual Analytic::FWorldHit SweepSingle(
		const Analytic::FWorldQuery& Query) const = 0;
    // Caller has already proved query independence. Synchronous barrier; index
    // order is retained. Only the selected immutable backend may parallelize.
    virtual bool SweepIndependentBatch(TConstArrayView<Analytic::FWorldQuery> Queries,
        TArrayView<Analytic::FWorldHit> OutHits,
        FIndependentQueryBatchStats* OutStats = nullptr) const;
    virtual int32 GetIndependentQueryLaneLimit() const { return 1; }
    // Prefetch admission never evaluates a query on policy/diagnostic fallback.
    virtual bool CanPrefetchIndependentQueries1075() const { return false; }
    virtual bool SweepPrefetchBatch1075(TConstArrayView<Analytic::FWorldQuery>,
        TArrayView<Analytic::FWorldHit>, FIndependentQueryBatchStats* = nullptr) const { return false; }
    virtual FImmutableStaticQueryIdentity1075 GetImmutableQueryIdentity1075() const { return {}; }
    virtual int32 GetMinimumIndependentQueryItems() const { return 2; }
    // Owner-only observations at a joined frame barrier, never during dispatch.
    virtual Analytic::FFacetParallelTotals GetFacetParallelTotals() const { return {}; }
    virtual void ResetFacetParallelTotals() const {}

	/** Greatest positive finite overlap at one pose, not the first CCD contact.
	 * False means unsupported/invalid query; true with no hit means no overlap.
	 * Start must equal End. Tangencies do not mask penetrating constraints. */
	virtual bool TryFindDeepestOverlap(const Analytic::FWorldQuery& Query,
		Analytic::FWorldHit& OutHit) const = 0;
	virtual bool HasAuthorityCoverage(
		const Analytic::FWorldQuery& Query) const = 0;
	virtual bool TrySweepAuthority(
		const Analytic::FWorldQuery& Query,
		Analytic::FWorldHit& OutAuthorityHit) const = 0;
	virtual EStaticCollisionBackend GetBackend() const = 0;
    virtual void VisitTorusCandidates(const Analytic::FWorldQuery& Query,
        TFunctionRef<void(const Analytic::FTorusSurface&)> Visitor) const {}

	/** Visits finite planar broad-phase candidates, not accepted contacts.
	 * The caller must refine its actual rotating trajectory and finite witness. */
	virtual void VisitPlanarCandidates(const Analytic::FWorldQuery& Query,
		TFunctionRef<void(const Analytic::FWorldHit&)> Visitor) const {}
	/** Certifies continuous support, not a pair of endpoint samples or an impact sweep. */
	virtual bool IsPlanarSupportTranslationCertified(const Analytic::FWorldQuery& Query,
		TConstArrayView<FVector3d> Points, const FVector3d& Normal, const FVector3d& Translation) const { return false; }
};

// Result of projecting a predicted query pose back into the feasible region of
// one already-established static contact. New contacts remain the
// responsibility of continuous collision detection.
struct IAMSPEED_API FEstablishedStaticContactProjection
{
	Analytic::FWorldHit Hit;
	FVector3d Location = FVector3d::ZeroVector;
	double MaximumPenetrationDepth = 0.0;
	double ResidualPenetrationDepth = 0.0;
	uint32 CorrectionIterations = 0;
	bool bContact = false;
	bool bConverged = false;
};

// Projects translation only. The caller owns the rigid-body velocity response
// because it has the mass, inertia and contact-point velocity. Query must be
// restricted to the immutable provider of an established contact.
IAMSPEED_API FEstablishedStaticContactProjection ProjectEstablishedStaticContact(
	const IStaticCollisionWorld& World,
	Analytic::FWorldQuery Query,
	uint32 MaxCorrectionIterations = 4,
	double PenetrationToleranceCm = 1.0e-6);

// Parallel-transports linear velocity between two analytical contact tangent
// spaces, then removes only any residual closing speed contributed by the
// transported linear and rotational point velocities. This preserves
// tangential speed without applying witness-dependent angular impulses.
IAMSPEED_API FVector TransportEstablishedContactVelocity(
	const FVector& LinearVelocity,
	const FVector& RotationalPointVelocity,
	const FVector& PreviousNormal,
	const FVector& CurrentNormal);

class IAMSPEED_API FAnalyticStaticCollisionWorld final
	: public IStaticCollisionWorld
{
public:

    // Construct only at the owning world's admission/reset barrier. InWorld
    // is finalized and remains immutable until this object is destroyed.
    explicit FAnalyticStaticCollisionWorld(
        const Analytic::FAnalyticWorldData& InWorld, int32 InIndependentQueryLanes = 1,
        int32 InMinimumIndependentQueryItems = 4, int32 InMinimumFacetItems = 32);
    bool SweepIndependentBatch(TConstArrayView<Analytic::FWorldQuery> Queries,
        TArrayView<Analytic::FWorldHit> OutHits,
        FIndependentQueryBatchStats* OutStats = nullptr) const override;
    int32 GetIndependentQueryLaneLimit() const override { return IndependentQueryLanes; }
    bool CanPrefetchIndependentQueries1075() const override;
    bool SweepPrefetchBatch1075(TConstArrayView<Analytic::FWorldQuery> Queries,
        TArrayView<Analytic::FWorldHit> OutHits,
        FIndependentQueryBatchStats* OutStats = nullptr) const override;
    FImmutableStaticQueryIdentity1075 GetImmutableQueryIdentity1075() const override
    { return {&WorldData, &QueryService, ImmutableQueryGeneration1075}; }
    int32 GetMinimumIndependentQueryItems() const override { return MinimumIndependentQueryItems; }
    Analytic::FFacetParallelTotals GetFacetParallelTotals() const override
    {
        auto Totals = QueryService.GetFacetParallelTotals();
        for (const auto& Service : IndependentQueryServices) Totals.Accumulate(Service->GetFacetParallelTotals());
        return Totals;
    }
    void ResetFacetParallelTotals() const override
    {
        QueryService.ResetFacetParallelTotals();
        for (const auto& Service : IndependentQueryServices) Service->ResetFacetParallelTotals();
    }


	Analytic::FWorldHit SweepSingle(
		const Analytic::FWorldQuery& Query) const override
	{
		// Strict collision has no legacy fallback. A conservative replacement
		// footprint is not its physical surface domain: retain proven finite
		// contacts even when the volume extends across an authored boundary.
		Analytic::FWorldQuery CollisionQuery = Query;
		CollisionQuery.bUseFiniteContactDomain = true;
		return QueryService.Sweep(CollisionQuery);
	}

	bool TryFindDeepestOverlap(const Analytic::FWorldQuery& Query,
		Analytic::FWorldHit& OutHit) const override
	{
		return QueryService.TryFindDeepestOverlap(Query, OutHit);
	}

    void VisitTorusCandidates(const Analytic::FWorldQuery& Query,
        TFunctionRef<void(const Analytic::FTorusSurface&)> Visitor) const override
    { QueryService.VisitTorusCandidates(Query,Visitor); }
	bool HasAuthorityCoverage(
		const Analytic::FWorldQuery& Query) const override
	{
		return QueryService.HasAuthorityCoverage(Query);
	}

	bool TrySweepAuthority(
		const Analytic::FWorldQuery& Query,
		Analytic::FWorldHit& OutAuthorityHit) const override
	{
		return QueryService.TrySweepAuthority(Query, OutAuthorityHit);
	}

	EStaticCollisionBackend GetBackend() const override
	{
		return EStaticCollisionBackend::SurfaceAnalytic;
	}
	void VisitPlanarCandidates(const Analytic::FWorldQuery& Query,
		TFunctionRef<void(const Analytic::FWorldHit&)> Visitor) const override
	{
		QueryService.VisitPlanarCandidates(Query, Visitor);
	}
	bool IsPlanarSupportTranslationCertified(const Analytic::FWorldQuery& Query,
		TConstArrayView<FVector3d> Points, const FVector3d& Normal, const FVector3d& Translation) const override
	{
		return QueryService.IsPlanarSupportTranslationCertified(Query, Points, Normal, Translation);
	}

private:
    bool SweepIndependentBatchImpl1075(TConstArrayView<Analytic::FWorldQuery> Queries,
        TArrayView<Analytic::FWorldHit> OutHits, FIndependentQueryBatchStats* OutStats,
        bool bRejectSerialFallback) const;
    const Analytic::FAnalyticWorldData& WorldData;
    const uint64 ImmutableQueryGeneration1075;
    Analytic::FWorldQueryService QueryService;
    int32 IndependentQueryLanes = 1;
    int32 MinimumIndependentQueryItems = 4;
    // Separate from canonical QueryService: it can never share a cache with
    // a dispatched worker. Every logical lane has exclusive persistent state.
    TArray<TUniquePtr<Analytic::FWorldQueryService>> IndependentQueryServices;
    mutable std::atomic<bool> IndependentBatchBusy{false};
};

} // namespace Speed
