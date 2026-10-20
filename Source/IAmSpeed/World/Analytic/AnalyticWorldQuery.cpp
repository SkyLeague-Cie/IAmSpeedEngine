#include "AnalyticWorldQuery.h"
#include "TorusWorldQuery.h"
#include "IAmSpeed/World/Collision/StaticQueryBatch.h"
#include "JoinedPiecewiseProviderWave1075.h"
#include "JoinedTensorCellWave1213.h"
#include "AnalyticFacetWorkPlan1075.h"
#include "AnalyticCausalStats1075.h"
#include "IAmSpeed/World/Simulation/FrameParallelExecutor.h"
#include <exception>
#include <stdexcept>
#include "HAL/PlatformTLS.h"
#include "IAmSpeed/World/Simulation/FrameParallelSettings.h"
#include "IAmSpeed/World/Simulation/FrameSpeculationSafety1213.h"
#include "AnalyticBoxSweepContext.h"
#include "ExactCrossNormalCache784.h"
#include "ExactCrossNormalCostProbe844.h"
#include "ReplayProbe925.h"
#include "AnalyticCaseSatDiagnostics.h"
#include "HAL/UnrealMemory.h"
#include <limits>
#include "IAmSpeed/World/Collision/OrderedBoundsIndex.h"

#include "HAL/IConsoleManager.h"
#include "HAL/PlatformTime.h"
#include "ProfilingDebugging/CpuProfilerTrace.h"

namespace
{
TAutoConsoleVariable<int32> CVarPiecewiseAdmissionBounds1146(
    TEXT("p.IAmSpeed.Parallel.PiecewiseAdmissionBounds1146"), 0,
    TEXT("Scheduling experiment, default0. Reject cheap provider misses from speculative admission only. Original provider loop and bound remain authoritative."));
TAutoConsoleVariable<int32> CVarMinimumFacetItemsPerLane1075(
    TEXT("p.IAmSpeed.Parallel.MinFacetItemsPerLane1075"), 0,
    TEXT("Scheduling experiment:0=1074 strided assignment; positive=minimum facets per contiguous lane. No arithmetic/reduction changes."));
}

// Private measurement variant659: coarse owned-frame counters remain enabled.
// This removes only six query fine CPU scopes, never physical math or traversal.
#ifndef SL_ANALYTIC_QUERY_FINE_CPU_SCOPES659
#define SL_ANALYTIC_QUERY_FINE_CPU_SCOPES659 0
#endif
#if !UE_BUILD_SHIPPING
namespace
{
TAutoConsoleVariable<int32> CVarFineCpuScopesCompiled659(
    TEXT("p.IAmSpeed.AnalyticWorld.FineCpuScopesCompiled659"), SL_ANALYTIC_QUERY_FINE_CPU_SCOPES659,
    TEXT("Read-only build identity:1query fine CPU scopes compiled,0coarse-frame-only variant. Not a physics or runtime optimization flag."), ECVF_ReadOnly);
}
#endif


// Private, value-only scratch for one immutable extruded-provider traversal.
// No TLS, global cache, world ownership, context-member growth or public API.
namespace Speed::Analytic::Private
{
    struct FPrimarySatProjection
    {
        double Radius, BoxCenter, BoxMinimum, BoxMaximum, Speed;
    };
#if !UE_BUILD_SHIPPING
    struct FPrimarySatProjectionScope final
    {
        FPrimarySatProjectionScope(const FWorldQuery& InQuery, const FBoxSweepContext& InContext)
            : Query(InQuery), Context(InContext), bFiniteInputs(FiniteInputs(InQuery, InContext))
        {
            static_assert(sizeof(double) == sizeof(uint64) && std::numeric_limits<double>::is_iec559,
                "Primary SAT scratch finite classification requires binary64");
        }
        FPrimarySatProjectionScope(const FPrimarySatProjectionScope&) = delete;
        FPrimarySatProjectionScope& operator=(const FPrimarySatProjectionScope&) = delete;

        bool IsBoundTo(const FWorldQuery& InQuery, const FBoxSweepContext& InContext) const
        {
            return bFiniteInputs && &InQuery == &Query && &InContext == &Context;
        }
        // Caller proves primary Index0..2 and exact Query/Context object identity.
        const FPrimarySatProjection& GetProjection(const int32 Index, const FVector3d& TestAxis) const
        {
            const uint8 Bit = uint8(1u << Index);
            FPrimarySatProjection& Projection = Projections[Index];
            if ((ProjectionMask & Bit) == 0)
            {
                const FVector3d* Axis = Context.Axis;
                Projection.Radius =
                    FMath::Abs(FVector3d::DotProduct(Axis[0], TestAxis)) * Query.HalfExtent.X +
                    FMath::Abs(FVector3d::DotProduct(Axis[1], TestAxis)) * Query.HalfExtent.Y +
                    FMath::Abs(FVector3d::DotProduct(Axis[2], TestAxis)) * Query.HalfExtent.Z;
                Projection.BoxCenter = FVector3d::DotProduct(Query.Start, TestAxis);
                Projection.BoxMinimum = Projection.BoxCenter - Projection.Radius;
                Projection.BoxMaximum = Projection.BoxCenter + Projection.Radius;
                if (FiniteDoubleBits(Projection.Radius) && FiniteDoubleBits(Projection.BoxCenter) &&
                    FiniteDoubleBits(Projection.BoxMinimum) && FiniteDoubleBits(Projection.BoxMaximum))
                {
                    ProjectionMask |= Bit;
                } // Finite-input overflow/NaN stays per-visit arithmetic, never prepared.
            }
            return Projection;
        }
        // Called at the original Speed position, after triangle overlap work.
        double GetMotionProjection(const int32 Index, const FVector3d& TestAxis) const
        {
            const uint8 Bit = uint8(1u << Index);
            FPrimarySatProjection& Projection = Projections[Index];
            if ((MotionMask & Bit) == 0)
            {
                Projection.Speed = FVector3d::DotProduct(Context.Motion, TestAxis);
                if (FiniteDoubleBits(Projection.Speed)) MotionMask |= Bit;
            }
            return Projection.Speed;
        }
    private:
        static bool FiniteDoubleBits(const double& Value)
        {
            uint64 Bits = 0;
            FMemory::Memcpy(&Bits, &Value, sizeof(Bits));
            return (Bits & 0x7ff0000000000000ull) != 0x7ff0000000000000ull;
        }
        static bool FiniteVectorBits(const FVector3d& Value)
        { return FiniteDoubleBits(Value.X) && FiniteDoubleBits(Value.Y) && FiniteDoubleBits(Value.Z); }
        static bool FiniteInputs(const FWorldQuery& Q, const FBoxSweepContext& C)
        {
            return Q.Shape == EQueryShape::Box &&
                FiniteVectorBits(Q.Start) && FiniteVectorBits(Q.End) && FiniteVectorBits(Q.HalfExtent) &&
                FiniteVectorBits(C.Motion) && FiniteVectorBits(C.Axis[0]) &&
                FiniteVectorBits(C.Axis[1]) && FiniteVectorBits(C.Axis[2]) &&
                FiniteDoubleBits(Q.Rotation.X) && FiniteDoubleBits(Q.Rotation.Y) &&
                FiniteDoubleBits(Q.Rotation.Z) && FiniteDoubleBits(Q.Rotation.W) &&
                FiniteDoubleBits(Q.DomainTolerance) && FiniteDoubleBits(Q.InitialOverlapTolerance);
        }
        const FWorldQuery& Query;
        const FBoxSweepContext& Context;
        mutable FPrimarySatProjection Projections[3];
        mutable uint8 ProjectionMask = 0, MotionMask = 0;
        const bool bFiniteInputs;
    };
#endif
}

namespace
{
#if !UE_BUILD_SHIPPING
	TAutoConsoleVariable<int32> CVarIAmSpeedAnalyticWorldCacheBucketMode(
		TEXT("p.IAmSpeed.AnalyticWorld.CacheBucketMode"), 0,
		TEXT("Experimental exact-cache placement. 0: original position hash (default); 1: discriminate shape, selection and provider policy. Same64 entries and full equality."),
		ECVF_Default);
	TAutoConsoleVariable<int32> CVarIAmSpeedAnalyticWorldStaticSphereBounds(
		TEXT("p.IAmSpeed.AnalyticWorld.StaticSphereBounds"), 0,
		TEXT("Experimental conservative Euclidean bounds rejection for stationary spheres. 0: existing bounds (default); 1: extra rejection only; unchanged query kernel and hit selection."),
		ECVF_Default);
	TAutoConsoleVariable<int32> CVarIAmSpeedAnalyticWorldMovingSphereBounds(
		TEXT("p.IAmSpeed.AnalyticWorld.MovingSphereBounds"), 0,
		TEXT("Experimental conservative moving-sphere segment-envelope distance rejection. 0: existing bounds (default); 1: extra rejection. Same kernels and hit arbitration."),
		ECVF_Default);
	TAutoConsoleVariable<int32> CVarIAmSpeedAnalyticWorldExtrudedSphereFacetBounds(
		TEXT("p.IAmSpeed.AnalyticWorld.ExtrudedSphereFacetBounds"), 0,
		TEXT("Experimental conservative sphere rejection per extruded facet. 0: original node-only sphere bounds (default); 1: reject unrelated facets before plane construction. Same kernels and arbitration."),
		ECVF_Default);
	TAutoConsoleVariable<int32> CVarIAmSpeedAnalyticWorldExtrudedTriangleContextConstruction541(
		TEXT("p.IAmSpeed.AnalyticWorld.ExtrudedTriangleContextConstruction541"), 0,
		TEXT("Experimental extruded Box SAT scratch construction only:0legacy(default),1skip unused local context when a cached query context exists. No geometry/traversal/state changes."));

	TAutoConsoleVariable<int32> CVarIAmSpeedAnalyticWorldExtrudedTriangleAxisSpecialization573(
		TEXT("p.IAmSpeed.AnalyticWorld.ExtrudedTriangleAxisSpecialization573"), 0,
		TEXT("Experimental extruded box SAT axis schedule specialization:0dynamic(default),1dispatch once to same eager/lazy scalar algorithm. No arithmetic, tolerance, ordering or diagnostics changes."));

	TAutoConsoleVariable<int32> CVarIAmSpeedAnalyticWorldExtrudedFacetGeometryCache(
		TEXT("p.IAmSpeed.AnalyticWorld.ExtrudedFacetGeometryCache"), 0,
		TEXT("Experimental immutable per-chord geometry reuse. 0: original query arithmetic (default); 1: reuse identical derived chord/frame arithmetic. Same kernels, bounds and hit arbitration."),
		ECVF_Default);
	TAutoConsoleVariable<int32> CVarIAmSpeedAnalyticWorldExtrudedBoxBoundsQueryReuse435(
		TEXT("p.IAmSpeed.AnalyticWorld.ExtrudedBoxBoundsQueryReuse435"), 0,
		TEXT("Experimental query-owned extruded OBB bounds context reuse. Default 0; identical scalar arithmetic, provider order and full time interval."),
		ECVF_Default);
	TAutoConsoleVariable<int32> CVarIAmSpeedAnalyticWorldLazyExtrudedBoxSatAxes(
		TEXT("p.IAmSpeed.AnalyticWorld.LazyExtrudedBoxSatAxes"), 0,
		TEXT("Experimental ordered on-demand cross axes for extruded box SAT. 0: eager original arithmetic (default); 1: generate only axes reached before the same rejection. Same thresholds, axis order and witnesses."),
		ECVF_Default);
	TAutoConsoleVariable<int32> CVarIAmSpeedAnalyticWorldExtrudedBoxPrimarySatScopeCache(
		TEXT("p.IAmSpeed.AnalyticWorld.ExtrudedBoxPrimarySatScopeCache"), 0,
		TEXT("Experimental private-scope reuse of three primary OBB SAT projections. 0: original per-visit arithmetic (default); 1: finite scratch bound to one immutable extruded Query/Context. Same RAW formulas, order, thresholds and witnesses. Benefit unqualified."),
		ECVF_Default);
    TAutoConsoleVariable<int32> CVarExactCrossNormalCache784(
        TEXT("p.IAmSpeed.AnalyticWorld.ExactCrossNormalCache784"), 0,
        TEXT("Private experiment.0off(default),1observe exact edge0 Cross/LengthSquared reuse but ALWAYS return original math,2reuse finite exact-bit normalized values in one extruded provider. No traversal/tolerance/state changes. Gain unqualified."));
    TAutoConsoleVariable<int32> CVarExactCrossCostProbe844(
        TEXT("p.IAmSpeed.AnalyticWorld.ExactCrossCostProbe844"), 0,
        TEXT("Private default0 diagnostic.1 samples one provider every4096 per thread, max32 per thread lifetime; only with784=0. Records at most512 actual normalization keys; physics returns original math. Recorded-input throughput only, never live frame CPU/gain/fullcoverage."));
    thread_local uint64 GCostProviderOrdinal844 = 0;
    thread_local uint32 GCostSamples844 = 0;
    TAutoConsoleVariable<int32> CVarReplayDecomposition925(
        TEXT("p.IAmSpeed.AnalyticWorld.ReplayDecomposition925"), 0,
        TEXT("Private default0 recorded-input diagnosis.1 selects owned Mode1 providers every4096/thread; max32 captures/process,2^24 replay rows,2048-byte diagnostic payload rows.784/844 must stay0. Original physics math; no live exclusive cost/gain/fullcoverage."));
    thread_local uint64 GReplayProviderOrdinal925 = 0;

    struct FExactCrossProbeTotals784
    {
        uint64 Providers = 0;
        Speed::Analytic::Private::FExactCrossNormalScope::FStats Work;
        bool Overflow = false;
    };
    thread_local FExactCrossProbeTotals784 GExactCrossProbe784;
    void RecordExactCrossProbe784(const Speed::Analytic::Private::FExactCrossNormalScope::FStats& S)
    {
        auto& T = GExactCrossProbe784;
        auto Add = [&T](uint64& D, const uint64 V)
        { if (V > MAX_uint64 - D) T.Overflow = true; else D += V; };
        Add(T.Providers, 1);
        Add(T.Work.Eligible, S.Eligible); Add(T.Work.Hits, S.Hits);
        Add(T.Work.Misses, S.Misses); Add(T.Work.Evictions, S.Evictions);
        Add(T.Work.Bypass, S.Bypass); Add(T.Work.RecomputedBitMismatches, S.RecomputedBitMismatches);
        if (T.Providers < 4096) return;
        UE_LOG(LogTemp, Display, TEXT("[ExactCrossNormalProbe784] Providers=%llu Eligible=%llu Hits=%llu Misses=%llu Evictions=%llu Bypass=%llu RecomputedBitMismatches=%llu Overflow=%u Scope=ProviderLocalExactBits DiagnosticOnly=1 FullCoverage=UNKNOWN"),
            T.Providers, T.Work.Eligible, T.Work.Hits, T.Work.Misses, T.Work.Evictions,
            T.Work.Bypass, T.Work.RecomputedBitMismatches, uint32(T.Overflow));
        T = FExactCrossProbeTotals784{};
    }
	TAutoConsoleVariable<int32> CVarIAmSpeedAnalyticWorldCacheProfile(
		TEXT("p.IAmSpeed.AnalyticWorld.CacheProfile"), 0,
		TEXT("Log exact sweep-cache hits, misses and hit ways every 1000 eligible queries. Development only."),
		ECVF_Default);

	TAutoConsoleVariable<int32> CVarIAmSpeedAnalyticWorldPhaseTiming(
		TEXT("p.IAmSpeed.AnalyticWorld.PhaseTiming"),
		0,
		TEXT("Accumulate and periodically log strict analytical query phase timing.\n")
		TEXT("0: disabled (default)\n")
		TEXT("1: log average phase cost every 10000 queries\n")
		TEXT("2: log every 1000 queries and also sample slow piecewise-provider traversals"),
		ECVF_Default);

	struct FAnalyticQueryPhaseTiming
	{
		uint64 Queries = 0;
		double TotalSeconds = 0.0;
		double PlaneSeconds = 0.0;
		double CompactSeconds = 0.0;
		double TensorSeconds = 0.0;
		double PiecewiseSeconds = 0.0;
		double TriangleSeconds = 0.0;
		double ArbitrationSeconds = 0.0;
		double PlaneEvaluationSeconds = 0.0;
		double PlaneTriangulationSeconds = 0.0;
		double CoplanarBoundarySeconds = 0.0;
		double ExtrudedEvaluationSeconds = 0.0;
		uint64 PlaneCalls = 0;
		uint64 TriangulationCalls = 0;
		uint64 CoplanarBoundaryCalls = 0;
		uint64 ExtrudedCalls = 0;
		uint64 BoxSatCalls = 0;
		uint64 BoxSatCrossProducts = 0;
		uint64 BoxSatNormalizedCrossAxes = 0;
		uint64 BoxSatVisitedAxes = 0;
	};

	thread_local FAnalyticQueryPhaseTiming GAnalyticQueryPhaseTiming;
	thread_local bool GAnalyticQueryDetailedTimingEnabled = false;
	thread_local uint64 GAnalyticApproximationNodeVisits = 0;
	thread_local uint64 GAnalyticApproximationCellVisits = 0;
	thread_local uint32 GAnalyticPiecewiseSlowQueryLogs = 0;
	constexpr uint64 AnalyticQueryPhaseTimingLogInterval = 10000;

	// Inclusive nested timings: triangulation/union are part of plane evaluation,
	// and a synthetic facet's plane evaluation is part of an extruded query.
	class FScopedAnalyticKernelTiming
	{
	public:
		FScopedAnalyticKernelTiming(double& Seconds, uint64& Calls)
			: Accumulator(GAnalyticQueryDetailedTimingEnabled ? &Seconds : nullptr)
		{
			if (Accumulator) { ++Calls; StartSeconds = FPlatformTime::Seconds(); }
		}
		~FScopedAnalyticKernelTiming()
		{
			if (Accumulator) *Accumulator += FPlatformTime::Seconds() - StartSeconds;
		}
	private:
		double* Accumulator;
		double StartSeconds = 0.0;
	};

	void RecordAnalyticQueryPhaseTiming(
		const double TotalSeconds,
		const double PlaneSeconds,
		const double CompactSeconds,
		const double TensorSeconds,
		const double PiecewiseSeconds,
		const double TriangleSeconds,
		const double ArbitrationSeconds)
	{
		FAnalyticQueryPhaseTiming& Timing = GAnalyticQueryPhaseTiming;
		++Timing.Queries;
		Timing.TotalSeconds += TotalSeconds;
		Timing.PlaneSeconds += PlaneSeconds;
		Timing.CompactSeconds += CompactSeconds;
		Timing.TensorSeconds += TensorSeconds;
		Timing.PiecewiseSeconds += PiecewiseSeconds;
		Timing.TriangleSeconds += TriangleSeconds;
		Timing.ArbitrationSeconds += ArbitrationSeconds;
		const uint64 LogInterval = GAnalyticQueryDetailedTimingEnabled
			? 1000 : AnalyticQueryPhaseTimingLogInterval;
		if (Timing.Queries < LogInterval)
		{
			return;
		}

		const double ToAverageMicroseconds = 1.0e6 /
			static_cast<double>(Timing.Queries);
		const double ClassifiedSeconds = Timing.PlaneSeconds +
			Timing.CompactSeconds + Timing.TensorSeconds +
			Timing.PiecewiseSeconds + Timing.TriangleSeconds +
			Timing.ArbitrationSeconds;
		UE_LOG(LogTemp, Display,
			TEXT("[AnalyticQueryPhaseTiming] Queries=%llu TotalUs=%.6f ")
			TEXT("PlaneUs=%.6f CompactUs=%.6f TensorUs=%.6f ")
			TEXT("PiecewiseUs=%.6f TriangleUs=%.6f ArbitrationUs=%.6f OtherUs=%.6f"),
			static_cast<unsigned long long>(Timing.Queries),
			Timing.TotalSeconds * ToAverageMicroseconds,
			Timing.PlaneSeconds * ToAverageMicroseconds,
			Timing.CompactSeconds * ToAverageMicroseconds,
			Timing.TensorSeconds * ToAverageMicroseconds,
			Timing.PiecewiseSeconds * ToAverageMicroseconds,
			Timing.TriangleSeconds * ToAverageMicroseconds,
			Timing.ArbitrationSeconds * ToAverageMicroseconds,
			FMath::Max(0.0, Timing.TotalSeconds - ClassifiedSeconds) *
				ToAverageMicroseconds);
		if (GAnalyticQueryDetailedTimingEnabled)
		{
			UE_LOG(LogTemp, Display,
				TEXT("[AnalyticQueryKernelTiming] Queries=%llu PlaneUs=%.6f TriangulationUs=%.6f ")
				TEXT("CoplanarUnionUs=%.6f ExtrudedUs=%.6f PlaneCalls=%llu TriangulationCalls=%llu UnionCalls=%llu ExtrudedCalls=%llu"),
				Timing.Queries, Timing.PlaneEvaluationSeconds * ToAverageMicroseconds,
				Timing.PlaneTriangulationSeconds * ToAverageMicroseconds,
				Timing.CoplanarBoundarySeconds * ToAverageMicroseconds,
				Timing.ExtrudedEvaluationSeconds * ToAverageMicroseconds,
				Timing.PlaneCalls, Timing.TriangulationCalls, Timing.CoplanarBoundaryCalls, Timing.ExtrudedCalls);
			UE_LOG(LogTemp, Display,
				TEXT("[AnalyticBoxSatWork] Queries=%llu Calls=%llu PotentialCrossProducts=%llu CrossProducts=%llu NormalizedCrossAxes=%llu VisitedAxes=%llu"),
				Timing.Queries, Timing.BoxSatCalls, 9 * Timing.BoxSatCalls,
				Timing.BoxSatCrossProducts, Timing.BoxSatNormalizedCrossAxes, Timing.BoxSatVisitedAxes);
		}
		Timing = FAnalyticQueryPhaseTiming{};
	}
#endif
}

namespace Speed::Analytic
{


namespace
{
bool HasIncompatibleFacetDiagnostics()
{
#if !UE_BUILD_SHIPPING
    static const TCHAR* Names[] = {
        TEXT("p.IAmSpeed.AnalyticWorld.AuthorityChaosShadow"),
        TEXT("p.IAmSpeed.AnalyticWorld.CacheProfile"),
        TEXT("p.IAmSpeed.AnalyticWorld.CaseCpuPhase400"),
        TEXT("p.IAmSpeed.AnalyticWorld.CaseSatDiagnostics"),
        TEXT("p.IAmSpeed.AnalyticWorld.CompactShadow"),
        TEXT("p.IAmSpeed.AnalyticWorld.ExactCrossCostProbe844"),
        TEXT("p.IAmSpeed.AnalyticWorld.MeshShadow"),
        TEXT("p.IAmSpeed.AnalyticWorld.PhaseTiming"),
        TEXT("p.IAmSpeed.AnalyticWorld.ReplayDecomposition925"),
        TEXT("p.IAmSpeed.AnalyticWorld.ResidualSelectionAudit"),
        TEXT("p.IAmSpeed.AnalyticWorld.Shadow"),
        TEXT("p.IAmSpeed.AnalyticWorld.StrictQueryDetail"),
        TEXT("p.IAmSpeed.Simulation.ActorProfile"),
        TEXT("p.IAmSpeed.StaticWorldQuery.Audit"),
    };
    // Module-owned diagnostics are registered before queries are admitted.
    // Cache their handles, while sampling their values on every call so that
    // changing diagnostics still forces the exact same serial fallback.
    static const auto Variables = []
    {
        TArray<const IConsoleVariable*, TInlineAllocator<16>> Result;
        for (const TCHAR* Name : Names)
            Result.Add(IConsoleManager::Get().FindConsoleVariable(Name));
        return Result;
    }();
    for (const IConsoleVariable* Variable : Variables)
        if (Variable && Variable->GetInt() != 0) return true;
#endif
    return false;
}
}

FWorldQueryService::FWorldQueryService(const FAnalyticWorldData& InWorld,
    int32 InFacetLanes, int32 InMinimumFacetItems)
    : World(InWorld), FacetLanes(FMath::Clamp(InFacetLanes, 1, Speed::FFrameParallelPolicy::MaximumLanes)),
      MinimumFacetItems(FMath::Max(2, InMinimumFacetItems))
{
	TArray<FBox3d> PlaneBounds;
	PlaneBounds.Reserve(World.Planes.Num());
	for (const FBoundedPlane& Plane : World.Planes) PlaneBounds.Add(Plane.Bounds);
	PlaneProviderIndex = Speed::Collision::FOrderedBoundsIndex::Create(PlaneBounds);
	TArray<FBox3d> PiecewiseBounds;
	PiecewiseBounds.Reserve(World.PiecewiseTensorBezierPatches.Num());
	for (const FPiecewiseTensorBezierPatch& Patch : World.PiecewiseTensorBezierPatches)
		PiecewiseBounds.Add(Patch.Bounds);
	PiecewiseProviderIndex = Speed::Collision::FOrderedBoundsIndex::Create(PiecewiseBounds);
#if !UE_BUILD_SHIPPING
	if (CVarIAmSpeedAnalyticWorldPhaseTiming.GetValueOnAnyThread() != 0)
		UE_LOG(LogTemp, Display, TEXT("[AnalyticQueryStorage] PiecewiseProviders=%d ProviderIndexBytes=%llu"),
			PiecewiseBounds.Num(), uint64(PiecewiseProviderIndex->GetAllocatedSize()));
#endif
	TMap<uint64, FSourceAuthorityCoverage> CoverageBySource;
	const auto RegisterPrimitive = [&CoverageBySource](
		const uint64 SourceId, const FBox3d& Bounds, const uint32 ObjectType,
		const uint64 BlockingChannels, const bool bQueryCollisionEnabled,
		const bool bTriangle, const bool bAuthorityEligible)
	{
		if (!bQueryCollisionEnabled)
		{
			return;
		}
		FSourceAuthorityCoverage& Coverage =
			CoverageBySource.FindOrAdd(SourceId);
		Coverage.SourceId = SourceId;
		Coverage.Bounds += Bounds;
		if (ObjectType < 64)
		{
			Coverage.ObjectTypes |= 1ull << ObjectType;
		}
		Coverage.BlockingChannels |= BlockingChannels;
		Coverage.bHasQueryPrimitive = true;
		if (bTriangle)
		{
			Coverage.bHasQueryTriangle = true;
			Coverage.bAllQueryTrianglesAuthorityEligible &= bAuthorityEligible;
		}
	};

	for (const FBoundedPlane& Plane : World.Planes)
	{
		RegisterPrimitive(Plane.SourceId, Plane.Bounds, Plane.ObjectType,
			Plane.BlockingChannels, Plane.bQueryCollisionEnabled, false,
			Plane.bAuthorityEligible);
	}
	for (const FExtrudedQuinticPatch& Patch : World.ExtrudedQuinticPatches)
	{
		RegisterPrimitive(Patch.SourceId, Patch.Bounds, Patch.ObjectType,
			Patch.BlockingChannels, Patch.bQueryCollisionEnabled, false,
			Patch.bAuthorityEligible);
	}
	for (const FTensorBezierPatch& Patch : World.TensorBezierPatches)
	{
		RegisterPrimitive(Patch.SourceId, Patch.Bounds, Patch.ObjectType,
			Patch.BlockingChannels, Patch.bQueryCollisionEnabled, false,
			Patch.bAuthorityEligible);
	}
	for (const FPiecewiseTensorBezierPatch& Patch : World.PiecewiseTensorBezierPatches)
	{
		RegisterPrimitive(Patch.SourceId, Patch.Bounds, Patch.ObjectType,
			Patch.BlockingChannels, Patch.bQueryCollisionEnabled, false,
			Patch.bAuthorityEligible);
	}
	for (const FTriangleSurface& Triangle : World.Triangles)
	{
		RegisterPrimitive(Triangle.SourceId, Triangle.Bounds, Triangle.ObjectType,
			Triangle.BlockingChannels, Triangle.bQueryCollisionEnabled, true,
			Triangle.bAuthorityEligible);
	}
    for (const auto& Torus:World.Tori)
        RegisterPrimitive(Torus.SourceId,Torus.Bounds,Torus.ObjectType,Torus.BlockingChannels,
            Torus.bQueryCollisionEnabled,false,Torus.bAuthorityEligible);
	CoverageBySource.GenerateValueArray(SourceAuthorityCoverage);
	SourceAuthorityCoverage.Sort(
		[](const FSourceAuthorityCoverage& A, const FSourceAuthorityCoverage& B)
		{
			return A.SourceId < B.SourceId;
		});
}

bool FWorldQueryService::IsAuthorityMissDefinitive(
	const FWorldQuery& Query, const FBox3d& QueryBounds) const
{
	for (const FSourceAuthorityCoverage& Coverage : SourceAuthorityCoverage)
	{
		if (!Coverage.bHasQueryPrimitive ||
			(Query.RequiredSourceId != 0 &&
				Coverage.SourceId != Query.RequiredSourceId))
		{
			continue;
		}
		if (Query.bApplyCollisionFilter)
		{
			const bool bCanBlock = Query.bObjectQuery
				? (Coverage.ObjectTypes & Query.ObjectTypes) != 0
				: Query.TraceChannel < 64 &&
					(Coverage.BlockingChannels & (1ull << Query.TraceChannel)) != 0 &&
					(Coverage.ObjectTypes & Query.BlockingObjectTypes) != 0;
			if (!bCanBlock)
			{
				continue;
			}
		}
		const bool bClosedResidualAuthority = Coverage.bHasQueryTriangle &&
			Coverage.bAllQueryTrianglesAuthorityEligible;
		if (!bClosedResidualAuthority && Coverage.Bounds.Intersect(QueryBounds))
		{
			return false;
		}
	}
	return true;
}
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

	using Private::FBoxSweepContext;
	using Private::FBoxBoundsSweepContext;

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
#if !UE_BUILD_SHIPPING
#if SL_ANALYTIC_QUERY_FINE_CPU_SCOPES659
        CaseSatDiagnostics::FScopedCpuPhase400 Phase609(CaseSatDiagnostics::ECpuPhase400::PlaneTriangulation);
#endif
		FScopedAnalyticKernelTiming ScopedTiming(GAnalyticQueryPhaseTiming.PlaneTriangulationSeconds,
			GAnalyticQueryPhaseTiming.TriangulationCalls);
#endif
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
		const auto AppendTriangle = [&](const int32 A, const int32 B, const int32 C)
		{
			FPlaneTriangle& Triangle = Out.AddDefaulted_GetRef();
			const int32 PolygonIndices[3] = { A, B, C };
			for (int32 Corner = 0; Corner < 3; ++Corner)
			{
				const FVector2d& Local = Vertices[PolygonIndices[Corner]];
				Triangle.Vertex[Corner] = Plane.Origin + Local.X * Plane.AxisU +
					Local.Y * Plane.AxisV;
				Triangle.PolygonVertex[Corner] = PolygonIndices[Corner];
			}
		};
		if (Plane.DomainVertices.IsEmpty() &&
			Plane.HalfExtents.X > 0.0 && Plane.HalfExtents.Y > 0.0 &&
			Cross2D(Vertices[0] - Vertices[3], Vertices[1] - Vertices[0]) > 1.0e-12)
		{
			// A non-degenerate rectangle always clips the same first ear. Keep
			// its original diagonal, vertex indices and floating-point operations;
			// authored polygons and numerically collapsed rectangles use the
			// generic certificate below instead of inventing a triangulation.
			AppendTriangle(3, 0, 1);
			AppendTriangle(1, 2, 3);
			return;
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

				AppendTriangle(Previous, Current, Next);
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
			AppendTriangle(Remaining[0], Remaining[1], Remaining[2]);
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

	template<int32 LazyMode573>
	bool SweepBoxTriangleImpl573(
		const FWorldQuery& Query, const FPlaneTriangle& Triangle,
		FWorldHit& OutHit, const FBoxSweepContext* CachedContext,
		const double MaximumSearchTime,
		const Private::FPrimarySatProjectionScope* PrimarySatScope,
		const Private::FExactCrossNormalScope* CrossNormalScope)
	{
#if !UE_BUILD_SHIPPING
#if SL_ANALYTIC_QUERY_FINE_CPU_SCOPES659
        CaseSatDiagnostics::FScopedCpuPhase400 Phase400(CaseSatDiagnostics::ECpuPhase400::BoxTriangle);
#endif
#endif
		// The original branch constructs scratch even with a supplied context.
		// Flag541 skips only that unread object; keep the fallback scalar path.
		TOptional<FBoxSweepContext> LocalContext;
		if (!CachedContext || !CachedContext->bSkipUnusedTriangleContext541)
		{
			LocalContext.Emplace();
		}
		if (CachedContext == nullptr)
		{
			LocalContext.GetValue().Initialize(Query);
		}
		const FBoxSweepContext& Context = CachedContext != nullptr
			? *CachedContext
			: LocalContext.GetValue();
		const FVector3d* Axis = Context.Axis;
#if !UE_BUILD_SHIPPING
		const bool bUsePrimarySatScope = PrimarySatScope && PrimarySatScope->IsBoundTo(Query, Context);
        const bool bUseCrossNormalScope = CrossNormalScope && CrossNormalScope->IsBoundTo(Query, Context);
#endif
		const FVector3d Edges[3] = {
			Triangle.Vertex[1] - Triangle.Vertex[0],
			Triangle.Vertex[2] - Triangle.Vertex[1],
			Triangle.Vertex[0] - Triangle.Vertex[2] };
		const FVector3d TriangleNormal = FVector3d::CrossProduct(
			Edges[0], Triangle.Vertex[2] - Triangle.Vertex[0]).GetSafeNormal();
		static_assert(LazyMode573 >= -1 && LazyMode573 <= 1, "Bound schedule variants");
		const bool bLazyAxes = LazyMode573 < 0 ? Context.bLazyExtrudedSatAxes : LazyMode573 != 0;
#if !UE_BUILD_SHIPPING
        CaseSatDiagnostics::FSatCounters* CaseSat = GAnalyticQueryDetailedTimingEnabled
            ? CaseSatDiagnostics::GetActiveSatCounters(bLazyAxes) : nullptr;
        uint8 DiagnosticLogicalAxes[13]; // Written/read only while CaseSat is active.
        bool bVisitedCross = false;
        uint8 DiagnosticLogicalSlot = 0;
        if (CaseSat) ++CaseSat->Calls;
#endif
#if !UE_BUILD_SHIPPING
		if (GAnalyticQueryDetailedTimingEnabled) ++GAnalyticQueryPhaseTiming.BoxSatCalls;
#endif
		FVector3d SeparatingAxes[13];
		int32 SeparatingAxisCount = 0;
		if (!bLazyAxes)
		{
#if !UE_BUILD_SHIPPING
        if (CaseSat) DiagnosticLogicalAxes[SeparatingAxisCount]=0;
#endif
		SeparatingAxes[SeparatingAxisCount++] = TriangleNormal;
		for (int32 AxisIndex = 0; AxisIndex < 3; ++AxisIndex)
		{
#if !UE_BUILD_SHIPPING
            if (CaseSat) DiagnosticLogicalAxes[SeparatingAxisCount]=uint8(AxisIndex+1);
#endif
			SeparatingAxes[SeparatingAxisCount++] = Axis[AxisIndex];
		}
		for (int32 AxisIndex = 0; AxisIndex < 3; ++AxisIndex)
		{
			const FVector3d& BoxAxis = Axis[AxisIndex];
#if !UE_BUILD_SHIPPING
            int32 DiagnosticEdgeIndex = 0;
#endif
			for (const FVector3d& Edge : Edges)
			{
#if !UE_BUILD_SHIPPING
                uint8 CrossLogicalSlot=0;
                if (CaseSat)
                {
                    CrossLogicalSlot=uint8(4+3*AxisIndex+DiagnosticEdgeIndex);
                    ++DiagnosticEdgeIndex;
                    ++CaseSat->CrossProducts;
                }
#endif
				const FVector3d Cross = FVector3d::CrossProduct(BoxAxis, Edge);
#if !UE_BUILD_SHIPPING
				if (GAnalyticQueryDetailedTimingEnabled) ++GAnalyticQueryPhaseTiming.BoxSatCrossProducts;
#endif
				const double CrossLengthSquared = Cross.SquaredLength();
				if (CrossLengthSquared > 1.0e-20)
				{
					// This axis already passed the SAT degeneracy criterion. The
					// much larger default GetSafeNormal threshold could turn it
					// into zero, whose zero overlap depth masks real penetration.
#if !UE_BUILD_SHIPPING
                    if (CaseSat)
                    {
                        DiagnosticLogicalAxes[SeparatingAxisCount]=CrossLogicalSlot;
                        ++CaseSat->NormalizedCrossAxes;
                    }
#endif
					SeparatingAxes[SeparatingAxisCount++] = Cross / FMath::Sqrt(CrossLengthSquared);
#if !UE_BUILD_SHIPPING
					if (GAnalyticQueryDetailedTimingEnabled) ++GAnalyticQueryPhaseTiming.BoxSatNormalizedCrossAxes;
#endif
				}
#if !UE_BUILD_SHIPPING
                else if (CaseSat) ++CaseSat->DegenerateCrossSlots[CrossLogicalSlot-4];
#endif
			}
		}
		}
		else
		{
			// Four primary axes, then nine logical cross slots. Degenerate slots
			// are skipped exactly as in eager construction; valid order is unchanged.
			SeparatingAxisCount = 13;
		}

		const FVector3d& Motion = Context.Motion;
		double EnterTime = 0.0;
		double ExitTime = FMath::Clamp(MaximumSearchTime, 0.0, 1.0);
		FVector3d EntryNormal = FVector3d::ZeroVector;
		double MinimumDepth = TNumericLimits<double>::Max();
		FVector3d MinimumTranslationNormal = FVector3d::ZeroVector;
		bool bInitiallyOverlapping = true;
		for (int32 SeparatingAxisIndex = 0;
			SeparatingAxisIndex < SeparatingAxisCount;
			++SeparatingAxisIndex)
		{
			FVector3d LazyAxis;
			if (bLazyAxes)
			{
				if (SeparatingAxisIndex == 0) LazyAxis = TriangleNormal;
				else if (SeparatingAxisIndex < 4) LazyAxis = Axis[SeparatingAxisIndex - 1];
				else
				{
					const int32 CrossIndex = SeparatingAxisIndex - 4;
					const FVector3d Cross = FVector3d::CrossProduct(Axis[CrossIndex / 3], Edges[CrossIndex % 3]);
					const double CrossLengthSquared = Cross.SquaredLength();
#if !UE_BUILD_SHIPPING
					if (GAnalyticQueryDetailedTimingEnabled) ++GAnalyticQueryPhaseTiming.BoxSatCrossProducts;
#endif
#if !UE_BUILD_SHIPPING
                    if (CaseSat) ++CaseSat->CrossProducts;
#endif
					if (!(CrossLengthSquared > 1.0e-20))
                    {
#if !UE_BUILD_SHIPPING
                        if (CaseSat) ++CaseSat->DegenerateCrossSlots[CrossIndex];
#endif
                        continue;
                    }
#if !UE_BUILD_SHIPPING
                    if (CaseSat) ++CaseSat->NormalizedCrossAxes;
#endif
#if !UE_BUILD_SHIPPING
                    // Existing normalized-axis counters remain LOGICAL visits;
                    // opt-in probe784 separately counts exact reuse opportunities.
                    if (bUseCrossNormalScope && CrossIndex % 3 == 0)
                        LazyAxis = CrossNormalScope->Normalize(Cross, CrossLengthSquared, CrossIndex / 3);
                    else
#endif
                    LazyAxis = Cross / FMath::Sqrt(CrossLengthSquared);
#if !UE_BUILD_SHIPPING
					if (GAnalyticQueryDetailedTimingEnabled) ++GAnalyticQueryPhaseTiming.BoxSatNormalizedCrossAxes;
#endif
				}
			}
			const FVector3d& TestAxis = bLazyAxes ? LazyAxis : SeparatingAxes[SeparatingAxisIndex];
#if !UE_BUILD_SHIPPING
			if (GAnalyticQueryDetailedTimingEnabled) ++GAnalyticQueryPhaseTiming.BoxSatVisitedAxes;
            if (CaseSat)
            {
                DiagnosticLogicalSlot=bLazyAxes?uint8(SeparatingAxisIndex):DiagnosticLogicalAxes[SeparatingAxisIndex];
                ++CaseSat->VisitedAxes;
                if (DiagnosticLogicalSlot>=4 && !bVisitedCross)
                { ++CaseSat->ReachedFirstValidCross; bVisitedCross=true; }
            }
#endif
			const int32 PrimaryAxisIndex = SeparatingAxisIndex - 1;
			const Private::FPrimarySatProjection* CachedPrimaryProjection = nullptr;
#if !UE_BUILD_SHIPPING
			if (bUsePrimarySatScope && PrimaryAxisIndex >= 0 && PrimaryAxisIndex < 3)
			{
				CachedPrimaryProjection = &PrimarySatScope->GetProjection(PrimaryAxisIndex, TestAxis);
			}
#endif
			const double Radius = CachedPrimaryProjection ? CachedPrimaryProjection->Radius :
				FMath::Abs(FVector3d::DotProduct(Axis[0], TestAxis)) * Query.HalfExtent.X +
				FMath::Abs(FVector3d::DotProduct(Axis[1], TestAxis)) * Query.HalfExtent.Y +
				FMath::Abs(FVector3d::DotProduct(Axis[2], TestAxis)) * Query.HalfExtent.Z;
			const double BoxCenter = CachedPrimaryProjection ? CachedPrimaryProjection->BoxCenter : FVector3d::DotProduct(Query.Start, TestAxis);
			const double BoxMinimum = CachedPrimaryProjection ? CachedPrimaryProjection->BoxMinimum : BoxCenter - Radius;
			const double BoxMaximum = CachedPrimaryProjection ? CachedPrimaryProjection->BoxMaximum : BoxCenter + Radius;
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

#if !UE_BUILD_SHIPPING
			const double Speed = CachedPrimaryProjection
				? PrimarySatScope->GetMotionProjection(PrimaryAxisIndex, TestAxis)
				: FVector3d::DotProduct(Motion, TestAxis);
#else
			const double Speed = FVector3d::DotProduct(Motion, TestAxis);
#endif
			if (FMath::Abs(Speed) <= 1.0e-15)
			{
				if (BoxMaximum < TriangleMinimum - Query.DomainTolerance ||
					BoxMinimum > TriangleMaximum + Query.DomainTolerance)
				{
#if !UE_BUILD_SHIPPING
                    if (CaseSat) ++CaseSat->StationaryReject[DiagnosticLogicalSlot];
#endif
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
#if !UE_BUILD_SHIPPING
                if (CaseSat) ++CaseSat->IntervalReject[DiagnosticLogicalSlot];
#endif
				return false;
			}
		}
#if !UE_BUILD_SHIPPING
        if (CaseSat) ++CaseSat->CompletedAllAxes;
#endif

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
#if !UE_BUILD_SHIPPING
                if (CaseSat) ++CaseSat->PostSatShellRejected;
#endif
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

#if !UE_BUILD_SHIPPING
        if (CaseSat) ++CaseSat->Contacts;
#endif
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

	bool SweepBoxTriangle(
		const FWorldQuery& Query, const FPlaneTriangle& Triangle,
		FWorldHit& OutHit, const FBoxSweepContext* CachedContext = nullptr,
		const double MaximumSearchTime = 1.0,
		const Private::FPrimarySatProjectionScope* PrimarySatScope = nullptr,
		const Private::FExactCrossNormalScope* CrossNormalScope = nullptr)
	{
#if !UE_BUILD_SHIPPING
		if (CachedContext && CachedContext->bSpecializeTriangleAxisSchedule573)
		{
			if (CachedContext->bLazyExtrudedSatAxes)
				return SweepBoxTriangleImpl573<1>(Query, Triangle, OutHit, CachedContext, MaximumSearchTime, PrimarySatScope, CrossNormalScope);
			return SweepBoxTriangleImpl573<0>(Query, Triangle, OutHit, CachedContext, MaximumSearchTime, PrimarySatScope, CrossNormalScope);
		}
#endif
		return SweepBoxTriangleImpl573<-1>(Query, Triangle, OutHit, CachedContext, MaximumSearchTime, PrimarySatScope, CrossNormalScope);
	}

	bool SweepSphereTriangleGeometry(
		const FWorldQuery& Query, const FVector3d TriangleVertices[3],
		const FVector3d& TriangleFaceNormal, FWorldHit& OutHit,
		const double MaximumSearchTime = 1.0)
	{
		// Conservative advancement against the convex triangle distance field
		// covers face, edge, and vertex contacts for a linearly translated sphere.
		const double Radius = FMath::Max(0.0, Query.Radius);
		const FVector3d Delta = Query.End - Query.Start;
		const double FaceNormalLengthSquared = TriangleFaceNormal.SquaredLength();
		if (FaceNormalLengthSquared > UE_DOUBLE_SMALL_NUMBER)
		{
			const double StartPlaneDistance = FVector3d::DotProduct(
				Query.Start - TriangleVertices[0], TriangleFaceNormal);
			const double EndPlaneDistance = StartPlaneDistance +
				FVector3d::DotProduct(Delta, TriangleFaceNormal);
			const double PlaneContactRadius = Radius + 1.0e-8;
			const double SquaredPlaneContactThreshold =
				PlaneContactRadius * PlaneContactRadius * FaceNormalLengthSquared;
			if ((StartPlaneDistance > 0.0 &&
					EndPlaneDistance > 0.0 &&
					StartPlaneDistance * StartPlaneDistance >
						SquaredPlaneContactThreshold &&
					EndPlaneDistance * EndPlaneDistance >
						SquaredPlaneContactThreshold) ||
				(StartPlaneDistance < 0.0 &&
					EndPlaneDistance < 0.0 &&
					StartPlaneDistance * StartPlaneDistance >
						SquaredPlaneContactThreshold &&
					EndPlaneDistance * EndPlaneDistance >
						SquaredPlaneContactThreshold))
				{
					// Do not reject solely from the plane slab: the segment can still
					// reach a triangle edge/vertex while remaining on one plane side.
				}
		}
		double Time = 0.0;
		FVector3d Center = Query.Start;
		FVector3d SurfacePoint = FVector3d::ZeroVector;
		FVector3d Normal = FVector3d::ZeroVector;
		EContactFeatureKind SurfaceKind = EContactFeatureKind::Unknown;
		int8 SurfaceIndex = INDEX_NONE;
		double Distance = 0.0;
		constexpr double ContactToleranceCm = 1.0e-8;
		const auto TrySegmentContact = [&]()
		{
			// Conservative advancement can reject a valid edge/vertex contact when
			// the closest-feature normal is momentarily orthogonal to the motion.
			// Probe the remaining segment deterministically and bisect the first
			// shell crossing; this is still pure analytic geometry and only runs on
			// that degenerate advancement path.
			const double BeginTime = Time;
			const FVector3d Begin = Center;
			const double Remaining = FMath::Max(0.0, 1.0 - BeginTime);
			if (Remaining <= 0.0) return false;
			int32 Bracket = INDEX_NONE;
			for (int32 Sample = 1; Sample <= 32; ++Sample)
			{
				const double Local = Sample / 32.0;
				const FVector3d Probe = FMath::Lerp(Begin, Query.End, Local);
				const FVector3d ProbePoint = ClosestPointOnTriangle(Probe,
					TriangleVertices[0], TriangleVertices[1], TriangleVertices[2],
					&SurfaceKind, &SurfaceIndex);
				if (FVector3d::Distance(Probe, ProbePoint) <= Radius + ContactToleranceCm)
				{
					Bracket = Sample;
					break;
				}
			}
			if (Bracket == INDEX_NONE) return false;
			double Low = (Bracket - 1) / 32.0;
			double High = Bracket / 32.0;
			for (int32 Iter = 0; Iter < 20; ++Iter)
			{
				const double Mid = 0.5 * (Low + High);
				const FVector3d Probe = FMath::Lerp(Begin, Query.End, Mid);
				const FVector3d ProbePoint = ClosestPointOnTriangle(Probe,
					TriangleVertices[0], TriangleVertices[1], TriangleVertices[2],
					&SurfaceKind, &SurfaceIndex);
				if (FVector3d::Distance(Probe, ProbePoint) <= Radius + ContactToleranceCm) High = Mid;
				else Low = Mid;
			}
			Time = BeginTime + Remaining * High;
			Center = FMath::Lerp(Query.Start, Query.End, Time);
			SurfacePoint = ClosestPointOnTriangle(Center, TriangleVertices[0],
				TriangleVertices[1], TriangleVertices[2], &SurfaceKind, &SurfaceIndex);
			const FVector3d Separation = Center - SurfacePoint;
			Distance = Separation.Length();
			Normal = Distance > UE_DOUBLE_SMALL_NUMBER ? Separation / Distance : TriangleFaceNormal.GetSafeNormal();
			OutHit.bHit = true;
			OutHit.Time = Time;
			OutHit.Location = Center;
			OutHit.Point = SurfacePoint;
			OutHit.QueryPoint = Center - Radius * Normal;
			OutHit.Normal = Normal;
			OutHit.bStartPenetrating = false;
			OutHit.PenetrationDepth = 0.0;
			OutHit.QueryFeatureKind = EContactFeatureKind::Vertex;
			OutHit.SurfaceFeatureKind = SurfaceKind;
			OutHit.SurfaceFeatureIndex = SurfaceIndex;
			return true;
		};
		for (int32 Iteration = 0; Iteration < 32; ++Iteration)
		{
			SurfacePoint = ClosestPointOnTriangle(Center,
				TriangleVertices[0], TriangleVertices[1], TriangleVertices[2],
				&SurfaceKind, &SurfaceIndex);
			const FVector3d Separation = Center - SurfacePoint;
			Distance = Separation.Length();
			Normal = Distance > UE_DOUBLE_SMALL_NUMBER
				? Separation / Distance
				: TriangleFaceNormal.GetSafeNormal();
			if (Normal.IsNearlyZero()) Normal = FVector3d::UpVector;
			const double Gap = Distance - Radius;
			if (Gap <= ContactToleranceCm)
			{
				OutHit.bHit = true;
				break;
			}
			const double ApproachSpeed = -FVector3d::DotProduct(Delta, Normal);
			if (ApproachSpeed <= UE_DOUBLE_SMALL_NUMBER)
				return TrySegmentContact();
			const double Step = Gap / ApproachSpeed;
			if (!FMath::IsFinite(Step) || Step <= 0.0) return false;
			const double SearchLimit = FMath::Min(1.0, MaximumSearchTime);
			// The conservative step is derived from the current distance field
			// linearization and can land infinitesimally beyond the finite query
			// interval even when the endpoint is already inside the contact shell.
			// Evaluate that endpoint before rejecting the sweep; otherwise a valid
			// tangent/end-point contact is lost to floating-point roundoff.
			if (Time + Step > SearchLimit)
			{
				Time = SearchLimit;
				Center = FMath::Lerp(Query.Start, Query.End, Time);
				SurfacePoint = ClosestPointOnTriangle(Center,
					TriangleVertices[0], TriangleVertices[1], TriangleVertices[2],
					&SurfaceKind, &SurfaceIndex);
				const FVector3d EndpointSeparation = Center - SurfacePoint;
				Distance = EndpointSeparation.Length();
				if (Distance <= Radius + ContactToleranceCm)
				{
					Normal = Distance > UE_DOUBLE_SMALL_NUMBER
						? EndpointSeparation / Distance
						: TriangleFaceNormal.GetSafeNormal();
					OutHit.bHit = true;
					break;
				}
				return false;
			}
			Time += Step;
			Time = FMath::Min(Time, 1.0);
			Center = FMath::Lerp(Query.Start, Query.End, Time);
		}
		if (!OutHit.bHit) return false;
		double PenetrationDepth = Time == 0.0
			? FMath::Max(0.0, Radius - Distance)
			: 0.0;
		bool bStartPenetrating = Time == 0.0 &&
			PenetrationDepth > Query.DomainTolerance;
		bool bAcceptedContactShell = false;
		if (bStartPenetrating && PenetrationDepth <=
			FMath::Max(0.0, Query.InitialOverlapTolerance))
		{
			const double ClosingDistance = -FVector3d::DotProduct(Delta, Normal);
			if (ClosingDistance <= Query.DomainTolerance) return false;
			bStartPenetrating = false;
			PenetrationDepth = 0.0;
			bAcceptedContactShell = true;
		}
		OutHit.Time = Time;
		OutHit.Location = Center;
		OutHit.Point = SurfacePoint;
		OutHit.QueryPoint = bAcceptedContactShell
			? SurfacePoint
			: Center - Radius * Normal;
		OutHit.Normal = Normal;
		OutHit.bStartPenetrating = bStartPenetrating;
		OutHit.PenetrationDepth = PenetrationDepth;
		OutHit.QueryFeatureKind = EContactFeatureKind::Vertex;
		OutHit.SurfaceFeatureKind = SurfaceKind;
		OutHit.SurfaceFeatureIndex = SurfaceIndex;
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
		if (Query.Shape == EQueryShape::Box)
		{
			const FVector3d AxisX = Query.Rotation.RotateVector(
				FVector3d::ForwardVector).GetSafeNormal();
			const FVector3d AxisY = Query.Rotation.RotateVector(
				FVector3d::RightVector).GetSafeNormal();
			const FVector3d AxisZ = Query.Rotation.RotateVector(
				FVector3d::UpVector).GetSafeNormal();
			const FVector3d Extent =
				AxisX.GetAbs() * FMath::Max(0.0, Query.HalfExtent.X) +
				AxisY.GetAbs() * FMath::Max(0.0, Query.HalfExtent.Y) +
				AxisZ.GetAbs() * FMath::Max(0.0, Query.HalfExtent.Z);
			Bounds += Query.Start - Extent;
			Bounds += Query.Start + Extent;
			Bounds += Query.End - Extent;
			Bounds += Query.End + Extent;
			// This is a candidate bound, not a contact skin. Axis normalization
			// and projected extents may round inward relative to the actual eight
			// quaternion-transformed corners. Retain those last-ULP candidates;
			// the narrow phase still decides the exact sign of their clearance.
			const double Roundoff = 64 * DBL_EPSILON * FMath::Max(1.0,
				FMath::Max(Query.Start.GetAbsMax(), Query.End.GetAbsMax()) + Query.HalfExtent.GetMax());
			Bounds = Bounds.ExpandBy(Roundoff);
		}
		else
		{
			Bounds += Query.Start;
			Bounds += Query.End;
			if (Query.Shape == EQueryShape::Sphere)
			{
				Bounds = Bounds.ExpandBy(FMath::Max(0.0, Query.Radius));
			}
		}
		return Bounds.ExpandBy(FMath::Max(0.0, Query.DomainTolerance));
	}

	bool SegmentIntersectsExpandedBounds(
		const FVector3d& Start, const FVector3d& End,
		const FBox3d& Bounds, const double Expansion,
		const double MaximumSearchTime = 1.0)
	{
		if (!Bounds.IsValid) return false;
		const FBox3d Expanded = Bounds.ExpandBy(FMath::Max(0.0, Expansion));
		const FVector3d Motion = End - Start;
		double MinimumTime = 0.0;
		double MaximumTime = FMath::Clamp(MaximumSearchTime, 0.0, 1.0);
		for (int32 Axis = 0; Axis < 3; ++Axis)
		{
			if (FMath::Abs(Motion[Axis]) <= 1.0e-18)
			{
				if (Start[Axis] < Expanded.Min[Axis] ||
					Start[Axis] > Expanded.Max[Axis])
				{
					return false;
				}
				continue;
			}
			double Entry = (Expanded.Min[Axis] - Start[Axis]) / Motion[Axis];
			double Exit = (Expanded.Max[Axis] - Start[Axis]) / Motion[Axis];
			if (Entry > Exit) Swap(Entry, Exit);
			MinimumTime = FMath::Max(MinimumTime, Entry);
			MaximumTime = FMath::Min(MaximumTime, Exit);
			if (MinimumTime > MaximumTime) return false;
		}
		return true;
	}

	bool SweptBoxIntersectsBounds(
		const FWorldQuery& Query, const FBoxBoundsSweepContext& Context,
		const FBox3d& Bounds, const double MaximumSearchTime = 1.0)
	{
		if (!Bounds.IsValid) return false;
		const FVector3d BoundsCenter = Bounds.GetCenter();
		const FVector3d BoundsExtent = Bounds.GetExtent();
		double MinimumTime = 0.0;
		double MaximumTime = FMath::Clamp(MaximumSearchTime, 0.0, 1.0);
		const auto IntersectProjectedAxis = [&MinimumTime, &MaximumTime](
			const double StartDistance, const double MotionDistance,
			double Radius)
		{
			Radius += 1.0e-10 * FMath::Max(1.0, Radius);
			if (FMath::Abs(MotionDistance) <= 1.0e-18)
			{
				return FMath::Abs(StartDistance) <= Radius;
			}
			double Entry = (-Radius - StartDistance) / MotionDistance;
			double Exit = (Radius - StartDistance) / MotionDistance;
			if (Entry > Exit) Swap(Entry, Exit);
			MinimumTime = FMath::Max(MinimumTime, Entry);
			MaximumTime = FMath::Min(MaximumTime, Exit);
			return MinimumTime <= MaximumTime;
		};
		const auto IntersectGeneralAxis = [&](const FBoxBoundsSweepContext::FProjection& Projection)
		{
			if (Projection.bDegenerate) return true;
			const FVector3d& Axis = Projection.Axis;
			const double BoundsRadius =
				BoundsExtent.X * FMath::Abs(Axis.X) +
				BoundsExtent.Y * FMath::Abs(Axis.Y) +
				BoundsExtent.Z * FMath::Abs(Axis.Z);
			const double Radius = Projection.BoxRadius + BoundsRadius + Projection.ToleranceRadius;
			const double StartDistance = FVector3d::DotProduct(
				Query.Start - BoundsCenter, Axis);
			return IntersectProjectedAxis(
				StartDistance, Projection.MotionDistance, Radius);
		};
		for (int32 AxisIndex = 0; AxisIndex < 3; ++AxisIndex)
		{
			if (!IntersectProjectedAxis(
				Query.Start[AxisIndex] - BoundsCenter[AxisIndex],
				Context.Motion[AxisIndex],
				Context.WorldExtent[AxisIndex] + BoundsExtent[AxisIndex] +
					Context.DomainTolerance))
			{
				return false;
			}
		}
		for (int32 AxisIndex = 0; AxisIndex < 3; ++AxisIndex)
		{
			const FVector3d& Axis = Context.Axis[AxisIndex];
			const double BoundsRadius =
				BoundsExtent.X * FMath::Abs(Axis.X) +
				BoundsExtent.Y * FMath::Abs(Axis.Y) +
				BoundsExtent.Z * FMath::Abs(Axis.Z);
			if (!IntersectProjectedAxis(
				FVector3d::DotProduct(Query.Start - BoundsCenter, Axis),
				FVector3d::DotProduct(Context.Motion, Axis),
				Context.HalfExtent[AxisIndex] + BoundsRadius +
					Context.DomainTolerance))
			{
				return false;
			}
		}
		for (int32 BoxAxisIndex = 0; BoxAxisIndex < 3; ++BoxAxisIndex)
		{
			for (int32 WorldAxisIndex = 0; WorldAxisIndex < 3; ++WorldAxisIndex)
			{
				if (!IntersectGeneralAxis(
					Context.GetCrossAxis(BoxAxisIndex, WorldAxisIndex)))
				{
					return false;
				}
			}
		}
		return true;
	}

	bool SweptShapeIntersectsBounds(
		const FWorldQuery& Query, const FBox3d& Bounds,
		const FBoxBoundsSweepContext* BoxContext = nullptr,
		const double MaximumSearchTime = 1.0)
	{
#if !UE_BUILD_SHIPPING
#if SL_ANALYTIC_QUERY_FINE_CPU_SCOPES659
        CaseSatDiagnostics::FScopedCpuPhase400 Phase609(CaseSatDiagnostics::ECpuPhase400::SweptBounds);
#endif
#endif
		if (Query.Shape == EQueryShape::Box)
		{
			if (BoxContext) return SweptBoxIntersectsBounds(
				Query, *BoxContext, Bounds, MaximumSearchTime);
			const FBoxBoundsSweepContext LocalContext(Query);
			return SweptBoxIntersectsBounds(
				Query, LocalContext, Bounds, MaximumSearchTime);
		}
		const double Radius = Query.Shape == EQueryShape::Sphere
			? FMath::Max(0.0, Query.Radius)
			: 0.0;
		if (!SegmentIntersectsExpandedBounds(Query.Start, Query.End, Bounds,
			Radius + FMath::Max(0.0, Query.DomainTolerance), MaximumSearchTime)) return false;
#if !UE_BUILD_SHIPPING
		if (Query.Shape == EQueryShape::Sphere && Query.Start == Query.End &&
			CVarIAmSpeedAnalyticWorldStaticSphereBounds.GetValueOnAnyThread() != 0)
		{
			// A stationary sphere does not reach the corners of its enclosing
			// cube. Retain an enclosing Euclidean radius for the domain's
			// per-coordinate tolerance and initial-contact tolerance; never
			// replace the narrow phase or its exact hit arbitration.
			const double Scale = FMath::Max(1.0, FMath::Max(
				Query.Start.GetAbsMax(), FMath::Max(Bounds.Min.GetAbsMax(), Bounds.Max.GetAbsMax())));
			const double EnclosingRadius = Radius + FMath::Sqrt(3.0) *
				FMath::Max(0.0, Query.DomainTolerance) +
				FMath::Max(0.0, Query.InitialOverlapTolerance) + 1.0e-9 * Scale;
			double DistanceSquared = 0.0;
			for (int32 Axis = 0; Axis < 3; ++Axis)
			{
				const double Gap = FMath::Max(0.0, FMath::Max(
					Bounds.Min[Axis] - Query.Start[Axis], Query.Start[Axis] - Bounds.Max[Axis]));
				DistanceSquared += Gap * Gap;
			}
			const double RadiusSquared = EnclosingRadius * EnclosingRadius;
			// Non-finite arithmetic keeps the original conservative answer.
			if (FMath::IsFinite(DistanceSquared) && FMath::IsFinite(RadiusSquared) &&
				DistanceSquared > RadiusSquared) return false;
		}
		if (Query.Shape == EQueryShape::Sphere && Query.Start != Query.End &&
			CVarIAmSpeedAnalyticWorldMovingSphereBounds.GetValueOnAnyThread() != 0)
		{
			// The complete segment lies inside its endpoint AABB. Distance
			// between that enclosing AABB and the surface bounds is therefore
			// a lower bound for every center on the segment, including any
			// MaximumSearchTime prefix. No approximation or hit rule changes.
			const double Scale = FMath::Max(1.0, FMath::Max(
				FMath::Max(Query.Start.GetAbsMax(), Query.End.GetAbsMax()),
				FMath::Max(Bounds.Min.GetAbsMax(), Bounds.Max.GetAbsMax())));
			const double EnclosingRadius = Radius + FMath::Sqrt(3.0) *
				FMath::Max(0.0, Query.DomainTolerance) +
				FMath::Max(0.0, Query.InitialOverlapTolerance) + 1.0e-9 * Scale;
			double DistanceSquared = 0.0;
			for (int32 Axis = 0; Axis < 3; ++Axis)
			{
				const double SegmentMin = FMath::Min(Query.Start[Axis], Query.End[Axis]);
				const double SegmentMax = FMath::Max(Query.Start[Axis], Query.End[Axis]);
				const double Gap = FMath::Max(0.0, FMath::Max(
					Bounds.Min[Axis] - SegmentMax, SegmentMin - Bounds.Max[Axis]));
				DistanceSquared += Gap * Gap;
			}
			const double RadiusSquared = EnclosingRadius * EnclosingRadius;
			if (FMath::IsFinite(DistanceSquared) && FMath::IsFinite(RadiusSquared) &&
				DistanceSquared > RadiusSquared) return false;
		}
#endif
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
	const FWorldQuery& Query, const FBoundedPlane& Plane,
	const FAnalyticWorldData* PlaneUnionWorld,
	const FBoxSweepContext* CachedBoxContext, const EHitSelection Selection,
	FIntPoint* OutPolygonEdge,
	const Private::FPrimarySatProjectionScope* PrimarySatScope,
		const Private::FExactCrossNormalScope* CrossNormalScope)
{
	if (OutPolygonEdge) *OutPolygonEdge = FIntPoint(INDEX_NONE, INDEX_NONE);
	if (Query.Shape == EQueryShape::Box)
	{
#if !UE_BUILD_SHIPPING
		FScopedAnalyticKernelTiming ScopedTiming(GAnalyticQueryPhaseTiming.PlaneEvaluationSeconds,
			GAnalyticQueryPhaseTiming.PlaneCalls);
#endif
		return SweepBoxPlane(Query, Plane, CachedBoxContext, Selection, OutPolygonEdge, PrimarySatScope, CrossNormalScope);
	}
	FWorldHit Hit;
	TrySweepRoundPlane(Query, Plane, Hit, PlaneUnionWorld);
	return Hit;
}

bool FWorldQueryService::TrySweepRoundPlane(
	const FWorldQuery& Query, const FBoundedPlane& Plane,
	FWorldHit& OutHit, const FAnalyticWorldData* PlaneUnionWorld)
{
#if !UE_BUILD_SHIPPING
	FScopedAnalyticKernelTiming ScopedTiming(GAnalyticQueryPhaseTiming.PlaneEvaluationSeconds,
		GAnalyticQueryPhaseTiming.PlaneCalls);
#endif
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
	bool bStartPenetrating = false;
	double PenetrationDepth = 0.0;
	if (StartDistance <= 0.0)
	{
		Time = 0.0;
		bStartPenetrating = StartDistance < 0.0;
		PenetrationDepth = FMath::Max(0.0, -StartDistance);
	}
	else
	{
		const double Denominator = StartDistance - EndDistance;
		if (Denominator <= 0.0 || EndDistance > 0.0)
		{
			return false;
		}
		Time = StartDistance / Denominator;
		if (Time < 0.0 || Time > 1.0)
		{
			return false;
		}
	}

	const FVector3d Center = FMath::Lerp(Query.Start, Query.End, Time);
	const FVector3d ContactPoint = Center - Support * Normal;
	if (!Plane.ContainsProjectedPoint(ContactPoint, Query.DomainTolerance))
	{
		return false;
	}
	if (Query.bAuthorityOnly && !Query.bUseFiniteContactDomain &&
		!Query.bAllowEstablishedFaceContactAtBoundary)
	{
		double RequiredInteriorMargin = 0.0;
		if (Query.Shape == EQueryShape::Sphere)
		{
			RequiredInteriorMargin = FMath::Max(0.0, Query.Radius);
		}
		double DistanceToAuthorityBoundary =
			Plane.DistanceToDomainBoundary(ContactPoint);
		if (PlaneUnionWorld && DistanceToAuthorityBoundary +
			Query.DomainTolerance < RequiredInteriorMargin)
		{
			DistanceToAuthorityBoundary =
				DistanceToCoplanarSemanticUnionBoundary(
					*PlaneUnionWorld, Query, Plane, ContactPoint);
		}
		if (DistanceToAuthorityBoundary +
			Query.DomainTolerance < RequiredInteriorMargin)
		{
			return false;
		}
	}

	// Reinitialize only a successful witness, including fields a caller may
	// have decorated after the previous hit. Misses do not write caller scratch.
	OutHit = FWorldHit();
	FWorldHit& Hit = OutHit;
	Hit.bHit = true;
	Hit.bStartPenetrating = bStartPenetrating;
	Hit.PenetrationDepth = PenetrationDepth;
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
	return true;
}

double FWorldQueryService::DistanceToCoplanarSemanticUnionBoundary(
	const FAnalyticWorldData& UnionWorld, const FWorldQuery& Query,
	const FBoundedPlane& SeedPlane, const FVector3d& Point)
{
#if !UE_BUILD_SHIPPING
	FScopedAnalyticKernelTiming ScopedTiming(GAnalyticQueryPhaseTiming.CoplanarBoundarySeconds,
		GAnalyticQueryPhaseTiming.CoplanarBoundaryCalls);
#endif
	struct FPlaneDomainEdge
	{
		FVector3d A = FVector3d::ZeroVector;
		FVector3d B = FVector3d::ZeroVector;
		int32 PlaneIndex = INDEX_NONE;
	};

	TArray<FPlaneDomainEdge, TInlineAllocator<64>> Edges;
	int32 CompatiblePlaneCount = 0;
	constexpr double CoplanarNormalTolerance = 1.0e-12;
	constexpr double CoplanarDistanceTolerance = 1.0e-6;
	const auto IsCompatible = [&](const FBoundedPlane& Candidate)
	{
		if (&Candidate != &SeedPlane &&
			(Candidate.SourceId != SeedPlane.SourceId ||
			 Candidate.SurfaceId != SeedPlane.SurfaceId ||
			 Candidate.FeatureId != SeedPlane.FeatureId ||
			 Candidate.MaterialId != SeedPlane.MaterialId ||
			 Candidate.ObjectType != SeedPlane.ObjectType ||
			 Candidate.BlockingChannels != SeedPlane.BlockingChannels))
		{
			return false;
		}
		if (!Candidate.bAuthorityEligible ||
			(Query.bExcludeAuthorityEligible && Candidate.bAuthorityEligible) ||
			!PlanePassesFilter(Query, Candidate))
		{
			return false;
		}
		const double NormalAlignment = FMath::Abs(FVector3d::DotProduct(
			SeedPlane.Normal, Candidate.Normal));
		return NormalAlignment >= 1.0 - CoplanarNormalTolerance &&
			FMath::Abs(SeedPlane.SignedDistance(Candidate.Origin)) <=
				CoplanarDistanceTolerance;
	};
	const auto AddDomainEdges = [&](const FBoundedPlane& Plane, const int32 PlaneIndex)
	{
		TArray<FVector2d, TInlineAllocator<16>> Vertices;
		if (Plane.DomainVertices.IsEmpty())
		{
			Vertices.Add(FVector2d(-Plane.HalfExtents.X, -Plane.HalfExtents.Y));
			Vertices.Add(FVector2d( Plane.HalfExtents.X, -Plane.HalfExtents.Y));
			Vertices.Add(FVector2d( Plane.HalfExtents.X,  Plane.HalfExtents.Y));
			Vertices.Add(FVector2d(-Plane.HalfExtents.X,  Plane.HalfExtents.Y));
		}
		else
		{
			Vertices.Append(Plane.DomainVertices);
		}
		for (int32 VertexIndex = 0; VertexIndex < Vertices.Num(); ++VertexIndex)
		{
			const FVector2d& LocalA = Vertices[VertexIndex];
			const FVector2d& LocalB = Vertices[(VertexIndex + 1) % Vertices.Num()];
			FPlaneDomainEdge& Edge = Edges.AddDefaulted_GetRef();
			Edge.A = Plane.Origin + LocalA.X * Plane.AxisU + LocalA.Y * Plane.AxisV;
			Edge.B = Plane.Origin + LocalB.X * Plane.AxisU + LocalB.Y * Plane.AxisV;
			Edge.PlaneIndex = PlaneIndex;
		}
	};

	for (int32 PlaneIndex = 0; PlaneIndex < UnionWorld.Planes.Num(); ++PlaneIndex)
	{
		const FBoundedPlane& Candidate = UnionWorld.Planes[PlaneIndex];
		if (!IsCompatible(Candidate)) continue;
		++CompatiblePlaneCount;
		AddDomainEdges(Candidate, PlaneIndex);
	}
	if (CompatiblePlaneCount <= 1)
	{
		return SeedPlane.DistanceToDomainBoundary(Point);
	}

	// Recognition can split one authored coplanar face into polygon components
	// whose nominally coincident mesh boundaries differ by sub-centimetre source
	// quantization. Treat only that tiny crack as an internal seam. Split each
	// edge at nearby foreign endpoints and exact crossings so a partially shared
	// edge retains every genuinely exterior interval.
	constexpr double SeamClosureToleranceCm = 1.0;
	const auto Cross2D = [](const FVector2d& A, const FVector2d& B)
	{
		return A.X * B.Y - A.Y * B.X;
	};
	const auto ProjectToSeedPlane = [&SeedPlane](const FVector3d& WorldPoint)
	{
		const FVector3d Relative = WorldPoint - SeedPlane.Origin;
		return FVector2d(
			FVector3d::DotProduct(Relative, SeedPlane.AxisU),
			FVector3d::DotProduct(Relative, SeedPlane.AxisV));
	};
	double MinimumExternalDistanceSquared = TNumericLimits<double>::Max();
	for (int32 EdgeIndex = 0; EdgeIndex < Edges.Num(); ++EdgeIndex)
	{
		const FPlaneDomainEdge& Edge = Edges[EdgeIndex];
		const FVector3d Segment = Edge.B - Edge.A;
		const double SegmentLengthSquared = Segment.SquaredLength();
		if (SegmentLengthSquared <= UE_DOUBLE_SMALL_NUMBER) continue;
		TArray<double, TInlineAllocator<32>> BreakParameters;
		BreakParameters.Add(0.0);
		BreakParameters.Add(1.0);
		const FVector2d EdgeA2d = ProjectToSeedPlane(Edge.A);
		const FVector2d EdgeB2d = ProjectToSeedPlane(Edge.B);
		const FVector2d EdgeSegment2d = EdgeB2d - EdgeA2d;
		for (int32 OtherIndex = 0; OtherIndex < Edges.Num(); ++OtherIndex)
		{
			if (OtherIndex == EdgeIndex ||
				Edges[OtherIndex].PlaneIndex == Edge.PlaneIndex)
			{
				continue;
			}
			const FPlaneDomainEdge& Other = Edges[OtherIndex];
			for (const FVector3d& OtherEndpoint : { Other.A, Other.B })
			{
				const double EndpointT = FVector3d::DotProduct(
					OtherEndpoint - Edge.A, Segment) / SegmentLengthSquared;
				if (EndpointT < 0.0 || EndpointT > 1.0) continue;
				const FVector3d ProjectedEndpoint = Edge.A + EndpointT * Segment;
				if ((OtherEndpoint - ProjectedEndpoint).SquaredLength() <=
					FMath::Square(SeamClosureToleranceCm))
				{
					BreakParameters.Add(EndpointT);
				}
			}
			const FVector2d OtherA2d = ProjectToSeedPlane(Other.A);
			const FVector2d OtherB2d = ProjectToSeedPlane(Other.B);
			const FVector2d OtherSegment2d = OtherB2d - OtherA2d;
			const double Denominator = Cross2D(EdgeSegment2d, OtherSegment2d);
			if (FMath::Abs(Denominator) > 1.0e-12)
			{
				const FVector2d BetweenStarts = OtherA2d - EdgeA2d;
				const double EdgeT = Cross2D(BetweenStarts, OtherSegment2d) /
					Denominator;
				const double OtherT = Cross2D(BetweenStarts, EdgeSegment2d) /
					Denominator;
				if (EdgeT >= 0.0 && EdgeT <= 1.0 &&
					OtherT >= 0.0 && OtherT <= 1.0)
				{
					BreakParameters.Add(EdgeT);
				}
			}
		}
		BreakParameters.Sort();
		int32 UniqueCount = 0;
		for (const double Parameter : BreakParameters)
		{
			if (UniqueCount == 0 || FMath::Abs(
				Parameter - BreakParameters[UniqueCount - 1]) > 1.0e-10)
			{
				BreakParameters[UniqueCount++] = Parameter;
			}
		}
		BreakParameters.SetNum(UniqueCount, EAllowShrinking::No);
		for (int32 IntervalIndex = 0;
			IntervalIndex + 1 < BreakParameters.Num(); ++IntervalIndex)
		{
			const double StartT = BreakParameters[IntervalIndex];
			const double EndT = BreakParameters[IntervalIndex + 1];
			if (EndT - StartT <= 1.0e-10) continue;
			const FVector3d Midpoint = Edge.A +
				(0.5 * (StartT + EndT)) * Segment;
			bool bInternalInterval = false;
			for (int32 PlaneIndex = 0; PlaneIndex < UnionWorld.Planes.Num(); ++PlaneIndex)
			{
				if (PlaneIndex == Edge.PlaneIndex) continue;
				const FBoundedPlane& Candidate = UnionWorld.Planes[PlaneIndex];
				if (IsCompatible(Candidate) && Candidate.ContainsProjectedPoint(
					Midpoint, SeamClosureToleranceCm))
				{
					bInternalInterval = true;
					break;
				}
			}
			if (bInternalInterval) continue;
			const FVector3d IntervalStart = Edge.A + StartT * Segment;
			const FVector3d IntervalSegment = (EndT - StartT) * Segment;
			const double IntervalLengthSquared = IntervalSegment.SquaredLength();
			const double IntervalT = FMath::Clamp(
				FVector3d::DotProduct(Point - IntervalStart, IntervalSegment) /
					IntervalLengthSquared,
				0.0, 1.0);
			MinimumExternalDistanceSquared = FMath::Min(
				MinimumExternalDistanceSquared,
				(Point - (IntervalStart + IntervalT * IntervalSegment)).
					SquaredLength());
		}
	}
	return MinimumExternalDistanceSquared < TNumericLimits<double>::Max()
		? FMath::Sqrt(MinimumExternalDistanceSquared)
		: SeedPlane.DistanceToDomainBoundary(Point);
}

bool FWorldQueryService::IsPlanarSupportTranslationCertified(const FWorldQuery& Query,
	TConstArrayView<FVector3d> Points, const FVector3d& Normal, const FVector3d& Translation) const
{
	if (Points.IsEmpty() || Points.Num() > 4 || Normal.ContainsNaN() || Translation.ContainsNaN() ||
		Query.RejectsAllCollision() || FMath::Abs(Normal.SizeSquared() - 1) > 64 * DBL_EPSILON) return false;
	FBox3d Bounds(EForceInit::ForceInit);
	for (const FVector3d& P : Points)
	{
		if (P.ContainsNaN()) return false;
		Bounds += P; Bounds += P + Translation;
	}
	const double Roundoff = 64 * DBL_EPSILON * FMath::Max(1.0, Bounds.Min.GetAbsMax() + Bounds.Max.GetAbsMax());
	Bounds = Bounds.ExpandBy(Roundoff);
	Speed::Collision::FOrderedBoundsScratch Scratch;
	const auto Candidates = PlaneProviderIndex->FindCandidates(Bounds, Scratch);
	for (const FVector3d& P : Points)
	{
		bool bSupported = false;
		for (int32 Index : Candidates)
		{
			const FBoundedPlane& Plane = World.Planes[Index];
			if (!PlanePassesFilter(Query, Plane) || !Plane.bAuthorityEligible ||
				(!Plane.Normal.Equals(Normal, 32 * DBL_EPSILON) && !Plane.Normal.Equals(-Normal, 32 * DBL_EPSILON)) ||
				FMath::Abs(Plane.SignedDistance(P)) > Roundoff ||
				FMath::Abs(Plane.SignedDistance(P + Translation)) > Roundoff) continue;
			if (Plane.ContainsProjectedSegment(P, P + Translation)) { bSupported = true; break; }
		}
		if (!bSupported) return false;
	}
	return true;
}

void FWorldQueryService::VisitPlanarCandidates(const FWorldQuery& Query,
	TFunctionRef<void(const FWorldHit&)> Visitor) const
{
	if (Query.Shape != EQueryShape::Sphere || Query.Radius <= 0 || Query.RejectsAllCollision()) return;
	FBox3d Bounds(Query.Start, Query.Start);
	Bounds += Query.End;
	Bounds = Bounds.ExpandBy(Query.Radius);
	Speed::Collision::FOrderedBoundsScratch Scratch;
	for (int32 Index : PlaneProviderIndex->FindCandidates(Bounds, Scratch))
	{
		const FBoundedPlane& Plane = World.Planes[Index];
		if (!PlanePassesFilter(Query, Plane) || (Query.bAuthorityOnly && !Plane.bAuthorityEligible) ||
			!Plane.Bounds.Intersect(Bounds)) continue;
		FWorldHit Candidate;
		Candidate.Normal = Plane.SignedDistance(Query.Start) >= 0 ? Plane.Normal : -Plane.Normal;
		Candidate.Point = Plane.Origin;
		Candidate.SourceId = Plane.SourceId; Candidate.SurfaceId = Plane.SurfaceId;
		Candidate.FeatureId = Plane.FeatureId; Candidate.PrimitiveId = Plane.PrimitiveId;
		Candidate.MaterialId = Plane.MaterialId;
		Candidate.SurfaceFeatureKind = EContactFeatureKind::Face;
		Visitor(Candidate);
	}
}

FWorldHit FWorldQueryService::SweepBoxPlane(
	const FWorldQuery& Query, const FBoundedPlane& Plane,
	const FBoxSweepContext* CachedBoxContext, const EHitSelection Selection,
	FIntPoint* OutPolygonEdge,
	const Private::FPrimarySatProjectionScope* PrimarySatScope,
		const Private::FExactCrossNormalScope* CrossNormalScope)
{
#if !UE_BUILD_SHIPPING
#if SL_ANALYTIC_QUERY_FINE_CPU_SCOPES659
    CaseSatDiagnostics::FScopedCpuPhase400 Phase609(CaseSatDiagnostics::ECpuPhase400::BoxPlane);
#endif
#endif
	FWorldHit Best;
	if (OutPolygonEdge) *OutPolygonEdge = FIntPoint(INDEX_NONE, INDEX_NONE);
	// Every facet and triangle sees the same box pose/path. Reuse an outer
	// extrusion's immutable context, or construct one for a standalone plane.
	FBoxSweepContext LocalContext;
	if (!CachedBoxContext)
	{
		LocalContext.Initialize(Query);
		CachedBoxContext = &LocalContext;
	}
	// If a disc enclosing the entire box trajectory fits strictly inside the
	// finite domain, only the actual plane face can collide. Triangle diagonals
	// are not physical edges and must not alter the witness or overlap depth.
	// The boundary-distance certificate also applies to concave simple polygons;
	// edge/vertex approaches outside this conservative interior use finite SAT.
	const FVector3d Motion = Query.End - Query.Start;
	const double Envelope = Query.HalfExtent.Length() + Motion.Length();
	if (Plane.ContainsProjectedPoint(Query.Start, 0) &&
		Plane.DistanceToDomainBoundary(Query.Start) > Envelope)
	{
		const FVector3d* Axes = CachedBoxContext->Axis;
		const double Radius = FMath::Abs(FVector3d::DotProduct(Axes[0], Plane.Normal)) * Query.HalfExtent.X +
			FMath::Abs(FVector3d::DotProduct(Axes[1], Plane.Normal)) * Query.HalfExtent.Y +
			FMath::Abs(FVector3d::DotProduct(Axes[2], Plane.Normal)) * Query.HalfExtent.Z;
		const double SignedDistance = FVector3d::DotProduct(Query.Start - Plane.Origin, Plane.Normal);
		const FVector3d Normal = SignedDistance >= 0 ? Plane.Normal : -Plane.Normal;
		const double Distance = FMath::Abs(SignedDistance);
		const double NormalMotion = FVector3d::DotProduct(Motion, Normal);
		double Gap = Distance - Radius;
		// A composed quaternion's axis/radius formula can disagree by a few ULPs
		// with RotateVector on the actual corners. Near zero, evaluate the latter
		// explicitly: do not turn arithmetic disagreement into an overlap impulse,
		// and do not suppress real negative clearances with a contact tolerance.
		const double Roundoff = 64 * DBL_EPSILON * FMath::Max(1.0,
			Query.Start.GetAbsMax() + Plane.Origin.GetAbsMax() + Query.HalfExtent.GetMax());
		int32 RefinedVertex = INDEX_NONE;
		FVector3d RefinedPoint = FVector3d::ZeroVector;
		if (FMath::Abs(Gap) <= Roundoff)
		{
			Gap = DBL_MAX;
			for (int32 Vertex = 0; Vertex < 8; ++Vertex)
			{
				const FVector3d Corner = Query.Start + Query.Rotation.RotateVector(FVector3d(
					(Vertex & 1) ? Query.HalfExtent.X : -Query.HalfExtent.X,
					(Vertex & 2) ? Query.HalfExtent.Y : -Query.HalfExtent.Y,
					(Vertex & 4) ? Query.HalfExtent.Z : -Query.HalfExtent.Z));
				const double CornerGap = FVector3d::DotProduct(Corner - Plane.Origin, Normal);
				if (CornerGap < Gap) { Gap = CornerGap; RefinedVertex = Vertex; RefinedPoint = Corner; }
			}
		}
		double Time = 0, Depth = FMath::Max(0.0, -Gap);
		if (Gap > 0)
		{
			if (NormalMotion >= 0) return Best;
			Time = -Gap / NormalMotion;
			if (Time > 1) return Best;
		}
		bool bPenetrating = Depth > Query.DomainTolerance;
		if (bPenetrating && Depth <= FMath::Max(0.0, Query.InitialOverlapTolerance))
		{
			if (-NormalMotion <= Query.DomainTolerance) return Best;
			bPenetrating = false;
		}
		Best.bHit = true;
		Best.Time = Time;
		Best.Location = Query.Start + Time * Motion;
		Best.Normal = Normal;
		Best.bStartPenetrating = bPenetrating;
		Best.PenetrationDepth = bPenetrating ? Depth : 0;
		Best.QueryPoint = Best.Location;
		int32 ActiveAxes = 0;
		int8 Vertex = 0;
		for (int32 Axis = 0; Axis < 3; ++Axis)
		{
			const double Projection = FVector3d::DotProduct(-Normal, Axes[Axis]);
			const double Sign = Projection >= 0 ? 1 : -1;
			if (Sign > 0) Vertex |= 1 << Axis;
			Best.QueryPoint += Sign * Query.HalfExtent[Axis] * Axes[Axis];
			if (FMath::Abs(Projection) > 1.e-10) ++ActiveAxes;
		}
		if (RefinedVertex != INDEX_NONE)
		{
			Best.QueryPoint = RefinedPoint + Time * Motion;
			Vertex = static_cast<int8>(RefinedVertex);
		}
		Best.Point = Best.QueryPoint - FVector3d::DotProduct(Best.QueryPoint - Plane.Origin, Normal) * Normal;
		Best.QueryFeatureKind = ActiveAxes <= 1 ? EContactFeatureKind::Face :
			ActiveAxes == 2 ? EContactFeatureKind::Edge : EContactFeatureKind::Vertex;
		Best.QueryFeatureIndex = Vertex;
		Best.SurfaceFeatureKind = EContactFeatureKind::Face;
		Best.SourceId = Plane.SourceId; Best.SurfaceId = Plane.SurfaceId;
		Best.FeatureId = Plane.FeatureId; Best.PrimitiveId = Plane.PrimitiveId;
		Best.MaterialId = Plane.MaterialId;
		return HitPassesReferenceNormal(Query, Best) ? Best : FWorldHit();
	}
	TArray<FPlaneTriangle, TInlineAllocator<32>> Triangles;
	TriangulatePlaneDomain(Plane, Triangles);
	for (const FPlaneTriangle& Triangle : Triangles)
	{
		FWorldHit Candidate;
		if (!SweepBoxTriangle(Query, Triangle, Candidate, CachedBoxContext, 1.0, PrimarySatScope, CrossNormalScope)) continue;
		Candidate.SourceId = Plane.SourceId;
		Candidate.SurfaceId = Plane.SurfaceId;
		Candidate.FeatureId = Plane.FeatureId;
		Candidate.PrimitiveId = Plane.PrimitiveId;
		Candidate.MaterialId = Plane.MaterialId;
		if (HitPassesReferenceNormal(Query, Candidate) &&
			IsBetterHit(Candidate, Best, Selection))
		{
			Best = Candidate;
			if (OutPolygonEdge)
			{
				*OutPolygonEdge = FIntPoint(INDEX_NONE, INDEX_NONE);
				if (Candidate.SurfaceFeatureKind == EContactFeatureKind::Edge &&
					Candidate.SurfaceFeatureIndex >= 0 && Candidate.SurfaceFeatureIndex < 3)
				{
					const int32 Edge = Candidate.SurfaceFeatureIndex;
					*OutPolygonEdge = FIntPoint(Triangle.PolygonVertex[Edge],
						Triangle.PolygonVertex[(Edge + 1) % 3]);
				}
			}
		}
	}
	return Best;
}

bool FWorldQueryService::TrianglePassesFilter(
	const FWorldQuery& Query, const FTriangleSurface& Triangle)
{
	if ((Query.RequiredSourceId != 0 &&
		Triangle.SourceId != Query.RequiredSourceId) ||
		(Query.RequiredSurfaceId != 0 &&
			Triangle.SurfaceId != Query.RequiredSurfaceId) ||
		Query.RequiredCanonicalGroupId != 0)
	{
		return false;
	}
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
	if ((Query.RequiredSourceId != 0 &&
		Patch.SourceId != Query.RequiredSourceId) ||
		(Query.RequiredSurfaceId != 0 &&
			Patch.SurfaceId != Query.RequiredSurfaceId) ||
		(Query.RequiredCanonicalGroupId != 0 &&
			Patch.CanonicalGroupId != Query.RequiredCanonicalGroupId))
	{
		return false;
	}
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

bool FWorldQueryService::PatchPassesFilter(
	const FWorldQuery& Query, const FTensorBezierPatch& Patch)
{
	if ((Query.RequiredSourceId != 0 &&
		Patch.SourceId != Query.RequiredSourceId) ||
		(Query.RequiredSurfaceId != 0 &&
			Patch.SurfaceId != Query.RequiredSurfaceId) ||
		(Query.RequiredCanonicalGroupId != 0 &&
			Patch.CanonicalGroupId != Query.RequiredCanonicalGroupId))
	{
		return false;
	}
	if (!Query.bApplyCollisionFilter) return true;
	if (!Patch.bQueryCollisionEnabled) return false;
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

bool FWorldQueryService::PatchPassesFilter(
	const FWorldQuery& Query, const FPiecewiseTensorBezierPatch& Patch)
{
	if ((Query.RequiredSourceId != 0 && Patch.SourceId != Query.RequiredSourceId) ||
		(Query.RequiredSurfaceId != 0 && Patch.SurfaceId != Query.RequiredSurfaceId) ||
		(Query.RequiredCanonicalGroupId != 0 &&
			Patch.CanonicalGroupId != Query.RequiredCanonicalGroupId))
	{
		return false;
	}
	if (!Query.bApplyCollisionFilter) return true;
	if (!Patch.bQueryCollisionEnabled) return false;
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
	if ((Query.RequiredSourceId != 0 &&
		Plane.SourceId != Query.RequiredSourceId) ||
		(Query.RequiredSurfaceId != 0 &&
			Plane.SurfaceId != Query.RequiredSurfaceId) ||
		Query.RequiredCanonicalGroupId != 0)
	{
		return false;
	}
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
	const FExtrudedQuinticPatch& Patch, const EHitSelection Selection,
	Private::FBoxBoundsSweepContext* CachedBoundsContext435,
    int32 InFacetLanes, int32 InMinimumFacetItems,
    Speed::FIndependentQueryBatchStats* OutDispatch, int32* OutEvaluatedFacets)
{
#if !UE_BUILD_SHIPPING
#if SL_ANALYTIC_QUERY_FINE_CPU_SCOPES659
    CaseSatDiagnostics::FScopedCpuPhase400 Phase400(CaseSatDiagnostics::ECpuPhase400::Extruded);
#endif
    CaseSatDiagnostics::FScopedExtrudedProvider CaseSatExtrudedProvider;
	FScopedAnalyticKernelTiming ScopedTiming(GAnalyticQueryPhaseTiming.ExtrudedEvaluationSeconds,
		GAnalyticQueryPhaseTiming.ExtrudedCalls);
#endif
    Speed::FIndependentQueryBatchStats Dispatch;
    Dispatch.RequestedLanes = FMath::Clamp(InFacetLanes, 1, Speed::FFrameParallelPolicy::MaximumLanes);
    Dispatch.Fallback = Speed::EIndependentQueryFallback::SerialPolicy;
    FScopedAnalyticCausalObservation1075 Causal(EAnalyticCausalKind1075::ExtrudedProvider, &Dispatch);
    if (OutDispatch) *OutDispatch = Dispatch;
    if (OutEvaluatedFacets) *OutEvaluatedFacets = 0;
	FWorldHit Best;
	TRACE_CPUPROFILER_EVENT_SCOPE(IAmSpeed_AnalyticSweepExtrudedQuintic);
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
		Patch.SectionSegmentBounds.Num() != Patch.SectionPolyline.Num() - 1 ||
		!FMath::IsFinite(Patch.MaximumChordErrorCm))
	{
		return Best;
	}
	const double ExtrusionCenter = 0.5 *
		(Patch.MinimumExtrusionCoordinate + Patch.MaximumExtrusionCoordinate);
	const double ExtrusionHalfExtent = 0.5 *
		(Patch.MaximumExtrusionCoordinate - Patch.MinimumExtrusionCoordinate);
	// Test the continuously swept shape against each node, not only its swept
	// AABB. Thin rotated boxes otherwise visit many unrelated chord facets.
	// Keep the full time interval and traversal order: equal-time witnesses must
	// still reach the existing deterministic arbitration unchanged.
	// Construct fallback storage only when this provider owns its context.
	TOptional<FBoxBoundsSweepContext> LocalBoxContext435;
	if (!CachedBoundsContext435) LocalBoxContext435.Emplace();
	FBoxBoundsSweepContext& BoxContext = CachedBoundsContext435
		? *CachedBoundsContext435 : LocalBoxContext435.GetValue();
	const bool bBoxQuery = Query.Shape == EQueryShape::Box;
	if (bBoxQuery && !CachedBoundsContext435) BoxContext.Initialize(Query);
#if !UE_BUILD_SHIPPING
	if (bBoxQuery) BoxContext.bLazyExtrudedSatAxes =
		CVarIAmSpeedAnalyticWorldLazyExtrudedBoxSatAxes.GetValueOnAnyThread() != 0;
	if (bBoxQuery) BoxContext.bSkipUnusedTriangleContext541 =
		CVarIAmSpeedAnalyticWorldExtrudedTriangleContextConstruction541.GetValueOnAnyThread() != 0;
	if (bBoxQuery) BoxContext.bSpecializeTriangleAxisSchedule573 =
		CVarIAmSpeedAnalyticWorldExtrudedTriangleAxisSpecialization573.GetValueOnAnyThread() != 0;
#endif
	const double ExtrusionRadius = bBoxQuery ? SupportRadius(Query, Patch.ExtrusionAxis) : 0;
	const double StartExtrusion = FVector3d::DotProduct(Query.Start, Patch.ExtrusionAxis);
	const double EndExtrusion = FVector3d::DotProduct(Query.End, Patch.ExtrusionAxis);
	const bool bInsideExtrusion = bBoxQuery &&
		FMath::Min(StartExtrusion, EndExtrusion) - ExtrusionRadius > Patch.MinimumExtrusionCoordinate &&
		FMath::Max(StartExtrusion, EndExtrusion) + ExtrusionRadius < Patch.MaximumExtrusionCoordinate;
	// These provider-level query changes are identical for every finite facet.
	// Internal chord seams must not receive a standalone plane's footprint margin;
	// the true patch boundaries are checked after a hit below.
	// True const value: no mutable alias escapes initialization. Every forwarded
	// const reference below denotes this same object until the provider returns.
	const FWorldQuery FaceQuery = [&Query]()
	{
		FWorldQuery Face = Query;
		Face.bAuthorityOnly = false;
		// The patch already validated its canonical family; synthetic planes have none.
		Face.RequiredCanonicalGroupId = 0;
		return Face;
	}();
	const Private::FPrimarySatProjectionScope* PrimarySatScope = nullptr;
    const Private::FExactCrossNormalScope* CrossNormalScope = nullptr;
	bool bSphereFacetBounds = false;
	bool bFacetGeometryCache = false;
#if !UE_BUILD_SHIPPING
	bSphereFacetBounds = Query.Shape == EQueryShape::Sphere &&
		CVarIAmSpeedAnalyticWorldExtrudedSphereFacetBounds.GetValueOnAnyThread() != 0;
	bFacetGeometryCache = CVarIAmSpeedAnalyticWorldExtrudedFacetGeometryCache.GetValueOnAnyThread() != 0 &&
		Patch.SectionFacetGeometry.Num() == Patch.SectionPolyline.Num() - 1;
#endif
    const auto EvaluateSegment = [&](const int32 Segment,
        Private::FBoxBoundsSweepContext& SegmentBoxContext,
        const Private::FPrimarySatProjectionScope* SegmentPrimaryScope,
        const Private::FExactCrossNormalScope* SegmentCrossScope) -> FWorldHit
    {
        FWorldHit Candidate;
		// The legacy finite-domain predicate uses dot products onto AxisU/AxisV.
		// For oblique axes its accepted domain is the dual-frame rectangle, not
		// necessarily the physical corner parallelogram enclosed by FacetBounds.
		// Such patches are valid inputs: preserve their original kernel path.
		// Exact perpendicularity is deliberately strict; near-perpendicular and
		// non-unit frames fall back instead of extending a geometric certificate.
		if (bSphereFacetBounds && Patch.ExtrusionAxis.SquaredLength() == 1.0 &&
			FVector3d::DotProduct(Patch.SectionPolyline[Segment + 1] -
				Patch.SectionPolyline[Segment], Patch.ExtrusionAxis) == 0.0)
		{
			// The finite chord rectangle is enclosed by this exact derived AABB.
			// Keep all hit work/order unchanged; enlarge for finite-domain tolerance
			// before using the existing conservative continuously swept-shape test.
			const double Margin = FMath::Sqrt(3.0) * FMath::Max(0.0, Query.DomainTolerance);
			if (!SweptShapeIntersectsBounds(Query,
				Patch.SectionSegmentBounds[Segment].ExpandBy(Margin))) return FWorldHit();
		}

		// A leaf can contain eight chords. Reject its unrelated facets before
		// constructing/triangulating planes for OBBs. Sphere/ray plane tests are
		// already cheap; their broad phase stays at the node level.
		if (bBoxQuery)
		{
			const FBox3d& FacetBounds = Patch.SectionSegmentBounds[Segment];
			if (!FacetBounds.Intersect(QueryBounds) ||
				!SweptShapeIntersectsBounds(Query, FacetBounds, &SegmentBoxContext)) return FWorldHit();
		}
		const FVector3d SectionA = Patch.SectionPolyline[Segment];
		const FVector3d SectionB = Patch.SectionPolyline[Segment + 1];
		const FExtrudedQuinticFacetGeometry* Geometry = bFacetGeometryCache
			? &Patch.SectionFacetGeometry[Segment] : nullptr;
		const FVector3d Chord = Geometry ? Geometry->Chord : SectionB - SectionA;
		const double ChordLength = Geometry ? Geometry->ChordLength : Chord.Length();
		if (ChordLength <= UE_DOUBLE_SMALL_NUMBER)
		{
			return FWorldHit();
		}
		FBoundedPlane Face;
		Face.SurfaceId = Patch.SurfaceId;
		Face.SourceId = Patch.SourceId;
		Face.FeatureId = Patch.FeatureId;
		Face.MaterialId = Patch.MaterialId;
		Face.AxisU = Geometry ? Geometry->AxisU : Chord / ChordLength;
		Face.AxisV = Patch.ExtrusionAxis;
		Face.Normal = Geometry ? Geometry->Normal : FVector3d::CrossProduct(Face.AxisU, Face.AxisV).GetSafeNormal();
		Face.Origin = Geometry ? Geometry->Origin : 0.5 * (SectionA + SectionB) + ExtrusionCenter * Patch.ExtrusionAxis;
		if (FVector3d::DotProduct(Query.Start - Face.Origin, Face.Normal) < 0.0)
		{
			Face.Normal = -Face.Normal;
		}
		Face.HalfExtents = Geometry ? Geometry->HalfExtents : FVector2d(0.5 * ChordLength, ExtrusionHalfExtent);
		// Chord endpoints inside the adaptive polyline are approximation seams,
		// not authored surface boundaries.  SweepPlane's authority margin is
		// correct for a standalone bounded plane, but applying it to every chord
		// excludes any sphere whose footprint crosses an internal seam.  Evaluate
		// the finite face first, then retain the footprint margin only at the four
		// actual patch boundaries (the two section endpoints and two extrusion
		// endpoints).
		if (bBoxQuery)
		{
			// A concave polyline's internal chord joins/triangulation diagonals
			// are not physical walls. Both neighbouring chords must lie on the
			// query side of this plane; convex creases and actual patch endpoints
			// retain the finite SAT kernel. Full extrusion-footprint containment
			// also excludes the authored end edges from this face-only path.
			const bool bConcaveInternalFacet = bInsideExtrusion && Segment > 0 &&
				Segment + 2 < Patch.SectionPolyline.Num() &&
				FVector3d::DotProduct(Patch.SectionPolyline[Segment - 1] - SectionA, Face.Normal) >= 0 &&
				FVector3d::DotProduct(Patch.SectionPolyline[Segment + 2] - SectionB, Face.Normal) >= 0;
			FIntPoint PolygonEdge(INDEX_NONE, INDEX_NONE);
			Candidate = SweepPlane(FaceQuery, Face, nullptr, &SegmentBoxContext, Selection, &PolygonEdge, SegmentPrimaryScope, SegmentCrossScope);
			const int32 EdgeMin = FMath::Min(PolygonEdge.X, PolygonEdge.Y);
			const int32 EdgeMax = FMath::Max(PolygonEdge.X, PolygonEdge.Y);
			// A terminal facet still has one INTERNAL section edge. Its actual
			// opposite boundary must keep SAT, so classify the winning triangle's
			// polygon corners, never its ambiguous triangle-local edge index.
			const bool bConcaveSectionEdge = bInsideExtrusion &&
				((EdgeMin == 0 && EdgeMax == 3 && Segment > 0 &&
					FVector3d::DotProduct(Patch.SectionPolyline[Segment - 1] - SectionA, Face.Normal) >= 0) ||
				 (EdgeMin == 1 && EdgeMax == 2 && Segment + 2 < Patch.SectionPolyline.Num() &&
					FVector3d::DotProduct(Patch.SectionPolyline[Segment + 2] - SectionB, Face.Normal) >= 0));
			if ((bConcaveInternalFacet || bConcaveSectionEdge) && Candidate.bHit)
			{
				// Normal projection of a penetrating support vertex can lie in
				// the face even when the ORIGINAL OBB misses that finite facet.
				// Keep finite SAT as the overlap/TOI proof; bounds pruning must
				// remain observationally equivalent to an unpruned narrow phase.
				const FVector3d N = Face.Normal;
				FVector3d SupportPoint = Query.Start;
				int32 ActiveAxes = 0;
				int8 Vertex = 0;
				for (int32 Axis = 0; Axis < 3; ++Axis)
				{
					const double D = FVector3d::DotProduct(-N, SegmentBoxContext.Axis[Axis]);
					const double Sign = D >= 0 ? 1 : -1;
					if (Sign > 0) Vertex |= 1 << Axis;
					if (FMath::Abs(D) > 1.0e-10) ++ActiveAxes;
					SupportPoint += Sign * Query.HalfExtent[Axis] * SegmentBoxContext.Axis[Axis];
				}
				const FVector3d Motion = Query.End - Query.Start;
				const double Gap = FVector3d::DotProduct(SupportPoint - Face.Origin, N);
				const double NormalMotion = FVector3d::DotProduct(Motion, N);
				double Time = 0;
				if (Gap > 0)
				{
					if (NormalMotion >= 0) return FWorldHit();
					Time = -Gap / NormalMotion;
				}
				if (Time > 1 || Time + 1.0e-12 < Candidate.Time) return FWorldHit();
				const double Depth = FMath::Max(0.0, -Gap);
				SupportPoint += Time * Motion;
				const FVector3d FacePoint = SupportPoint + Depth * N;
				if (!Face.ContainsProjectedPoint(FacePoint, Query.DomainTolerance)) return FWorldHit();
				// Recompute the normal/depth/witness as one finite face constraint;
				// smoothing only the SAT normal would misrepresent penetration.
				Candidate.bStartPenetrating = Depth > Query.DomainTolerance;
				if (Candidate.bStartPenetrating && Depth <= FMath::Max(0.0, Query.InitialOverlapTolerance))
				{
					if (-FVector3d::DotProduct(Query.End - Query.Start, N) <= Query.DomainTolerance) return FWorldHit();
					Candidate.bStartPenetrating = false;
				}
				Candidate.Normal = N;
				Candidate.Time = Time;
				Candidate.Location = Query.Start + Time * Motion;
				Candidate.Point = FacePoint;
				Candidate.QueryPoint = SupportPoint;
				Candidate.PenetrationDepth = Candidate.bStartPenetrating ? Depth : 0;
				Candidate.SurfaceFeatureKind = EContactFeatureKind::Face;
				Candidate.SurfaceFeatureIndex = INDEX_NONE;
				Candidate.QueryFeatureKind = ActiveAxes <= 1 ? EContactFeatureKind::Face :
					ActiveAxes == 2 ? EContactFeatureKind::Edge : EContactFeatureKind::Vertex;
				Candidate.QueryFeatureIndex = Vertex;
			}
		}
		else if (!TrySweepRoundPlane(FaceQuery, Face, Candidate))
		{
			return FWorldHit();
		}
		// A miss cannot enter arbitration. In particular, do not hash a stable
		// primitive id or populate witness metadata for every rejected facet.
		if (!Candidate.bHit) return FWorldHit();
		const bool bProvenFiniteContact = Query.bUseFiniteContactDomain &&
			(bBoxQuery || (Query.Shape == EQueryShape::Sphere &&
				Candidate.SurfaceFeatureKind == EContactFeatureKind::Face));
		if ((Query.bAuthorityOnly || Patch.bAuthorityEligible) && !bProvenFiniteContact)
		{
			// This is conservative replacement coverage, not the geometry of a
			// finite intersection. The round witness or finite OBB SAT has already
			// proved contact. Erosion in strict mode can hide a real intersection
			// until its chosen witness abruptly enters this smaller coverage area.
			// A certified provider owns only the source-certified finite domain. A
			// volume contacting within one support radius of an authored patch
			// endpoint cannot use the synthetic terminal edge as a wall; a
			// neighbouring provider (or a definitive miss) must own that part of
			// the query instead. Apply the rule to certified providers as well as
			// explicitly authority-only queries because an authority-pruned runtime
			// world can issue ordinary queries after filtering providers at build
			// time.
			const double RequiredExtrusionMargin = SupportRadius(
				Query, Patch.ExtrusionAxis);
			const double RequiredSectionMargin = SupportRadius(Query, Face.AxisU);
			const double ExtrusionCoordinate = FVector3d::DotProduct(
				Candidate.Point, Patch.ExtrusionAxis);
			const double AlongChord = FVector3d::DotProduct(
				Candidate.Point - SectionA, Face.AxisU);
			const bool bCrossesExtrusionBoundary =
				ExtrusionCoordinate + Query.DomainTolerance <
					Patch.MinimumExtrusionCoordinate + RequiredExtrusionMargin ||
				ExtrusionCoordinate - Query.DomainTolerance >
					Patch.MaximumExtrusionCoordinate - RequiredExtrusionMargin;
			const bool bCrossesSectionStart = Segment == 0 &&
				AlongChord + Query.DomainTolerance < RequiredSectionMargin;
			const bool bCrossesSectionEnd =
				Segment + 2 == Patch.SectionPolyline.Num() &&
				AlongChord - Query.DomainTolerance >
					ChordLength - RequiredSectionMargin;
			if (bCrossesExtrusionBoundary || bCrossesSectionStart ||
				bCrossesSectionEnd)
			{
				return FWorldHit();
			}
		}
		Candidate.PrimitiveId = CombineStableIds(
			Patch.PrimitiveId, static_cast<uint64>(Segment + 1));
		Candidate.CanonicalGroupId = Patch.CanonicalGroupId;
		Candidate.GeometricErrorBoundCm = Patch.MaximumChordErrorCm;
		Candidate.AdditionalResidualAgreementAllowanceCm =
			Patch.AdditionalResidualAgreementAllowanceCm;
		Candidate.bSurfaceNormalMayVary = Candidate.bHit;
		// A bounded-plane triangulation can report its internal diagonal as an
		// edge witness.  That diagonal is not a physical crease.  In the gutter
		// transition, its near-horizontal SAT normal creates false support; use
		// the authored derivative only for internal, near-horizontal witnesses.
		// Endpoint and oblique wall edges retain their physical boundary normal.
		const bool bInternalSectionEdge =
			Candidate.SurfaceFeatureKind == EContactFeatureKind::Edge &&
			Segment > 0 && Segment + 1 < Patch.SectionPolyline.Num() - 1 &&
			FMath::Abs(Candidate.Normal.Z) > 0.90;
		if (Candidate.bHit && !Candidate.bStartPenetrating &&
			(Candidate.SurfaceFeatureKind == EContactFeatureKind::Face ||
				bInternalSectionEdge))
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
		return Candidate;
    };
    const auto FoldSegment = [&](const FWorldHit& Candidate)
    {
		if (Candidate.bHit && HitPassesReferenceNormal(Query, Candidate) &&
			IsBetterHit(Candidate, Best, Selection))
		{
			Best = Candidate;
		}
    };
    const auto SweepSegment = [&](int32 Segment)
    {
        if (OutEvaluatedFacets) ++*OutEvaluatedFacets;
        FoldSegment(EvaluateSegment(Segment, BoxContext, PrimarySatScope, CrossNormalScope));
    };
	const auto VisitProviderFacets = [&](TFunctionRef<void(int32)> Visit)
	{
		TArray<int32, TInlineAllocator<32>> NodeStack;
		NodeStack.Add(0);
		while (!NodeStack.IsEmpty())
		{
			const int32 NodeIndex = NodeStack.Pop(EAllowShrinking::No);
			if (!Patch.SectionSegmentBvhNodes.IsValidIndex(NodeIndex)) continue;
			const FTriangleBvhNode& Node = Patch.SectionSegmentBvhNodes[NodeIndex];
			if (!Node.Bounds.Intersect(QueryBounds)) continue;
			if (!SweptShapeIntersectsBounds(Query, Node.Bounds, &BoxContext)) continue;
			if (!Node.IsLeaf())
			{
				NodeStack.Add(Node.RightChild);
				NodeStack.Add(Node.LeftChild);
				continue;
			}
			for (int32 Offset = 0; Offset < Node.IndexCount; ++Offset)
			{
				const int32 PermutationIndex = Node.FirstIndex + Offset;
				if (Patch.SectionSegmentBvhIndices.IsValidIndex(PermutationIndex))
				{
					Visit(Patch.SectionSegmentBvhIndices[PermutationIndex]);
				}
			}
		}
	};
    const auto SweepProviderFacets = [&]()
    {
        Causal.MarkPrepared();
        if (!Causal.IsEnabled()) { VisitProviderFacets(SweepSegment); return; }
        VisitProviderFacets([&](int32 Segment)
        {
            if (OutEvaluatedFacets) ++*OutEvaluatedFacets;
            Causal.AddItem();
            FWorldHit Candidate;
            {
                FScopedAnalyticCycles1075 Numeric(Causal.NumericTarget(0));
                Candidate = EvaluateSegment(Segment, BoxContext, PrimarySatScope, CrossNormalScope);
            }
            {
                FScopedAnalyticCycles1075 Fold(Causal.FoldTarget());
                FoldSegment(Candidate);
            }
        });
    };
    const bool bNestedTask = Dispatch.RequestedLanes > 1 && Speed::IsInsideFrameParallelTask();
    bool bDiagnosticsCompatible = Dispatch.RequestedLanes == 1 || !HasIncompatibleFacetDiagnostics();
#if !UE_BUILD_SHIPPING
    // These optional scopes contain mutable per-provider projections/captures.
    // Their existing serial execution and witnesses remain authoritative.
    if (Dispatch.RequestedLanes > 1)
        bDiagnosticsCompatible &= CVarExactCrossNormalCache784.GetValueOnAnyThread() == 0 &&
            CVarIAmSpeedAnalyticWorldExtrudedBoxPrimarySatScopeCache.GetValueOnAnyThread() == 0;
#endif
    // A cached world may retain lanes chosen under a different FP mask. Gate
    // the live owner BEFORE the extra facet eligibility traversal or dispatch.
    const bool bNumericSpeculationSafe1213 = Dispatch.RequestedLanes > 1 &&
        !bNestedTask && bDiagnosticsCompatible && Speed::AreCurrentFrameFloatingPointExceptionsMasked1213();
    // Capture the existing owner environment once, before any worker dispatch.
    const Speed::FFrameFloatingPointContext OwnerNumericContext = bNumericSpeculationSafe1213
        ? Speed::CaptureFrameFloatingPointContext() : Speed::FFrameFloatingPointContext{};
    if (bNumericSpeculationSafe1213 && OwnerNumericContext.bValid)
    {
        constexpr int32 MaximumFacetBatch = 4096;
        TArray<int32, TInlineAllocator<64>> OrderedSegments;
        bool bCapacityExceeded = false;
        VisitProviderFacets([&](int32 Segment)
        {
            if (OrderedSegments.Num() < MaximumFacetBatch) OrderedSegments.Add(Segment);
            else bCapacityExceeded = true;
        });
        const FAnalyticFacetWorkPlan1075 WorkPlan = FAnalyticFacetWorkPlan1075::Resolve(
            OrderedSegments.Num(), Dispatch.RequestedLanes, InMinimumFacetItems,
            FMath::Max(0, CVarMinimumFacetItemsPerLane1075.GetValueOnAnyThread()));
        if (!bCapacityExceeded && WorkPlan.Lanes > 1)
        {
            Dispatch.DispatchedLogicalLanes = WorkPlan.Lanes;
            Causal.SetItems(OrderedSegments.Num());
            Dispatch.Fallback = Speed::EIndependentQueryFallback::None;
            TArray<FWorldHit> FacetHits;
            FacetHits.SetNum(OrderedSegments.Num());
            // Never copy unread lazy-projection storage. Initialize each lane
            // with identical scalar operations; only prepared flags are copied.
            Private::FBoxBoundsSweepContext LaneContexts[Speed::FFrameParallelPolicy::MaximumLanes];
            for (int32 Lane = 0; Lane < Dispatch.DispatchedLogicalLanes; ++Lane)
            {
                if (bBoxQuery) LaneContexts[Lane].Initialize(Query);
                LaneContexts[Lane].bLazyExtrudedSatAxes = BoxContext.bLazyExtrudedSatAxes;
                LaneContexts[Lane].bSkipUnusedTriangleContext541 = BoxContext.bSkipUnusedTriangleContext541;
                LaneContexts[Lane].bSpecializeTriangleAxisSchedule573 = BoxContext.bSpecializeTriangleAxisSchedule573;
            }
            std::exception_ptr Failures[Speed::FFrameParallelPolicy::MaximumLanes];
            Causal.MarkPrepared();
            Speed::DispatchFrameParallelLanes(Dispatch.DispatchedLogicalLanes,
                [&, OwnerNumericContext](int32 Lane)
                {
                    Speed::FScopedFrameParallelTask TaskScope(OwnerNumericContext);
                    Dispatch.NativeThreadIds[Lane] = FPlatformTLS::GetCurrentThreadId();
                    try
                    {
                        if (!TaskScope.IsNumericContextApplied())
                            throw std::runtime_error("IAmSpeed facet numeric context unavailable");
                        FScopedAnalyticCycles1075 Numeric(Causal.NumericTarget(Lane));
                        for (int32 Index = WorkPlan.First(Lane); Index < WorkPlan.End(Lane); Index += WorkPlan.Stride())
                            FacetHits[Index] = EvaluateSegment(OrderedSegments[Index], LaneContexts[Lane], nullptr, nullptr);
                    }
                    catch (...) { Failures[Lane] = std::current_exception(); }
                    if (!TaskScope.RestoreNumericContext())
                        Failures[Lane] = std::make_exception_ptr(std::runtime_error(
                            "IAmSpeed facet numeric restoration failed"));
                }, Causal.DispatchObservation());
            // A failed worker never contributes an unqualified partial hit.
            for (int32 Lane = 0; Lane < Dispatch.DispatchedLogicalLanes; ++Lane)
                if (Failures[Lane]) std::rethrow_exception(Failures[Lane]);
            // Hit comparison is not an associative reduction. Replay the exact
            // original visitation order after the barrier, including ties.
            {
                FScopedAnalyticCycles1075 Fold(Causal.FoldTarget());
                for (const FWorldHit& Hit : FacetHits) FoldSegment(Hit);
            }
            for (int32 Lane = 0; Lane < Dispatch.DispatchedLogicalLanes; ++Lane)
            {
                bool Seen = false;
                for (int32 Prior = 0; Prior < Lane; ++Prior)
                    Seen |= Dispatch.NativeThreadIds[Prior] == Dispatch.NativeThreadIds[Lane];
                if (!Seen) ++Dispatch.DistinctNativeThreads;
            }
            if (OutDispatch) *OutDispatch = Dispatch;
            if (OutEvaluatedFacets) *OutEvaluatedFacets = OrderedSegments.Num();
            return Best;
        }
        Dispatch.Fallback = bCapacityExceeded ? Speed::EIndependentQueryFallback::BatchCapacityExceeded :
            Speed::EIndependentQueryFallback::TooFewQueryItems;
    }
    else if (bNestedTask) Dispatch.Fallback = Speed::EIndependentQueryFallback::NestedTask;
    else if (!bDiagnosticsCompatible) Dispatch.Fallback = Speed::EIndependentQueryFallback::ThreadOwnedDiagnostics;
    else if (Dispatch.RequestedLanes > 1 && !OwnerNumericContext.bValid)
        Dispatch.Fallback = Speed::EIndependentQueryFallback::NumericEnvironmentUnavailable;
    Dispatch.DispatchedLogicalLanes = 1;
    Dispatch.NativeThreadIds[0] = FPlatformTLS::GetCurrentThreadId();
    Dispatch.DistinctNativeThreads = 1;
    if (OutDispatch) *OutDispatch = Dispatch;

#if !UE_BUILD_SHIPPING
    const int32 ExactCrossMode784 = CVarExactCrossNormalCache784.GetValueOnAnyThread();
    if (bBoxQuery && BoxContext.bLazyExtrudedSatAxes && ExactCrossMode784 == 0 &&
        CVarExactCrossCostProbe844.GetValueOnAnyThread() == 0 &&
        CVarReplayDecomposition925.GetValueOnAnyThread() == 1 &&
        CVarIAmSpeedAnalyticWorldExtrudedBoxPrimarySatScopeCache.GetValueOnAnyThread() == 0)
    {
        CaseSatDiagnostics::FOwnedForwardIdentity917 Identity;
        if (!CaseSatDiagnostics::TryCopyOwnedForwardIdentity917(Identity)) Private::FReplayProbe925::Unavailable();
        else if (GReplayProviderOrdinal925 == MAX_uint64) Private::FReplayProbe925::State().Overflow.store(true,std::memory_order_relaxed);
        else if (++GReplayProviderOrdinal925 % 4096 == 0)
        {
            uint64 ReportId=0;
            if (Private::FReplayProbe925::Claim(ReportId))
            {
                Private::FExactCrossNormalCapture844 Capture;
                Private::FExactCrossNormalScope ProbeScope(FaceQuery,BoxContext,true,&Capture);
                CrossNormalScope=&ProbeScope;
                SweepProviderFacets();
                Private::FReplayProbe925::Report(Capture,FaceQuery,BoxContext,Identity,ReportId);
                return Best;
            }
        }
    }

    if (bBoxQuery && BoxContext.bLazyExtrudedSatAxes && ExactCrossMode784 == 0 &&
        CVarExactCrossCostProbe844.GetValueOnAnyThread() == 1)
    {
        // Bounded diagnostic ownership is thread/provider only. Case/frame/tails UNKNOWN.
        if (GCostProviderOrdinal844 != MAX_uint64) ++GCostProviderOrdinal844;
        if (GCostSamples844 < 32 && GCostProviderOrdinal844 % 4096 == 0)
        {
            ++GCostSamples844;
            Private::FExactCrossNormalCapture844 Capture;
            Private::FExactCrossNormalScope ProbeScope(FaceQuery, BoxContext, true, &Capture);
            CrossNormalScope = &ProbeScope;
            if (CVarIAmSpeedAnalyticWorldExtrudedBoxPrimarySatScopeCache.GetValueOnAnyThread() != 0)
            {
                Private::FPrimarySatProjectionScope PrimaryScope(FaceQuery, BoxContext);
                PrimarySatScope = &PrimaryScope;
                SweepProviderFacets();
            }
            else SweepProviderFacets();
            Private::FExactCrossCostProbe844::Report(Capture, FaceQuery, BoxContext, GCostProviderOrdinal844);
            return Best;
        }
    }
    if (bBoxQuery && BoxContext.bLazyExtrudedSatAxes && (ExactCrossMode784 == 1 || ExactCrossMode784 == 2))
    {
        Private::FExactCrossNormalScope CrossScope(FaceQuery, BoxContext, ExactCrossMode784 == 1);
        CrossNormalScope = &CrossScope;
        if (CVarIAmSpeedAnalyticWorldExtrudedBoxPrimarySatScopeCache.GetValueOnAnyThread() != 0)
        {
            Private::FPrimarySatProjectionScope PrimaryScope(FaceQuery, BoxContext);
            PrimarySatScope = &PrimaryScope;
            SweepProviderFacets();
        }
        else SweepProviderFacets();
        if (ExactCrossMode784 == 1) RecordExactCrossProbe784(CrossScope.GetObservationStats());
        return Best;
    }
	if (bBoxQuery && CVarIAmSpeedAnalyticWorldExtrudedBoxPrimarySatScopeCache.GetValueOnAnyThread() != 0)
	{
		// Scratch has a lifetime only in this flag1 Box branch. It and both
		// bound objects outlive every synchronous forwarded SAT call.
		Private::FPrimarySatProjectionScope Scope(FaceQuery, BoxContext);
		PrimarySatScope = &Scope;
		SweepProviderFacets();
		return Best;
	}
#endif
	SweepProviderFacets();
	return Best;
}


#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
// Retained exact 938 scalar provider, test-only. There is no production CVar,
// shipping branch, or physical admission change selecting this reference.
FWorldHit FWorldQueryService::SweepExtrudedQuinticRetained938(
	const FWorldQuery& Query, const FBox3d& QueryBounds,
	const FExtrudedQuinticPatch& Patch, const EHitSelection Selection,
	Private::FBoxBoundsSweepContext* CachedBoundsContext435)
{
#if !UE_BUILD_SHIPPING
#if SL_ANALYTIC_QUERY_FINE_CPU_SCOPES659
    CaseSatDiagnostics::FScopedCpuPhase400 Phase400(CaseSatDiagnostics::ECpuPhase400::Extruded);
#endif
    CaseSatDiagnostics::FScopedExtrudedProvider CaseSatExtrudedProvider;
	FScopedAnalyticKernelTiming ScopedTiming(GAnalyticQueryPhaseTiming.ExtrudedEvaluationSeconds,
		GAnalyticQueryPhaseTiming.ExtrudedCalls);
#endif
	FWorldHit Best;
	TRACE_CPUPROFILER_EVENT_SCOPE(IAmSpeed_AnalyticSweepExtrudedQuintic);
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
		Patch.SectionSegmentBounds.Num() != Patch.SectionPolyline.Num() - 1 ||
		!FMath::IsFinite(Patch.MaximumChordErrorCm))
	{
		return Best;
	}
	const double ExtrusionCenter = 0.5 *
		(Patch.MinimumExtrusionCoordinate + Patch.MaximumExtrusionCoordinate);
	const double ExtrusionHalfExtent = 0.5 *
		(Patch.MaximumExtrusionCoordinate - Patch.MinimumExtrusionCoordinate);
	// Test the continuously swept shape against each node, not only its swept
	// AABB. Thin rotated boxes otherwise visit many unrelated chord facets.
	// Keep the full time interval and traversal order: equal-time witnesses must
	// still reach the existing deterministic arbitration unchanged.
	// Construct fallback storage only when this provider owns its context.
	TOptional<FBoxBoundsSweepContext> LocalBoxContext435;
	if (!CachedBoundsContext435) LocalBoxContext435.Emplace();
	FBoxBoundsSweepContext& BoxContext = CachedBoundsContext435
		? *CachedBoundsContext435 : LocalBoxContext435.GetValue();
	const bool bBoxQuery = Query.Shape == EQueryShape::Box;
	if (bBoxQuery && !CachedBoundsContext435) BoxContext.Initialize(Query);
#if !UE_BUILD_SHIPPING
	if (bBoxQuery) BoxContext.bLazyExtrudedSatAxes =
		CVarIAmSpeedAnalyticWorldLazyExtrudedBoxSatAxes.GetValueOnAnyThread() != 0;
	if (bBoxQuery) BoxContext.bSkipUnusedTriangleContext541 =
		CVarIAmSpeedAnalyticWorldExtrudedTriangleContextConstruction541.GetValueOnAnyThread() != 0;
	if (bBoxQuery) BoxContext.bSpecializeTriangleAxisSchedule573 =
		CVarIAmSpeedAnalyticWorldExtrudedTriangleAxisSpecialization573.GetValueOnAnyThread() != 0;
#endif
	const double ExtrusionRadius = bBoxQuery ? SupportRadius(Query, Patch.ExtrusionAxis) : 0;
	const double StartExtrusion = FVector3d::DotProduct(Query.Start, Patch.ExtrusionAxis);
	const double EndExtrusion = FVector3d::DotProduct(Query.End, Patch.ExtrusionAxis);
	const bool bInsideExtrusion = bBoxQuery &&
		FMath::Min(StartExtrusion, EndExtrusion) - ExtrusionRadius > Patch.MinimumExtrusionCoordinate &&
		FMath::Max(StartExtrusion, EndExtrusion) + ExtrusionRadius < Patch.MaximumExtrusionCoordinate;
	// These provider-level query changes are identical for every finite facet.
	// Internal chord seams must not receive a standalone plane's footprint margin;
	// the true patch boundaries are checked after a hit below.
	// True const value: no mutable alias escapes initialization. Every forwarded
	// const reference below denotes this same object until the provider returns.
	const FWorldQuery FaceQuery = [&Query]()
	{
		FWorldQuery Face = Query;
		Face.bAuthorityOnly = false;
		// The patch already validated its canonical family; synthetic planes have none.
		Face.RequiredCanonicalGroupId = 0;
		return Face;
	}();
	const Private::FPrimarySatProjectionScope* PrimarySatScope = nullptr;
    const Private::FExactCrossNormalScope* CrossNormalScope = nullptr;
	FWorldHit Candidate;
	bool bSphereFacetBounds = false;
	bool bFacetGeometryCache = false;
#if !UE_BUILD_SHIPPING
	bSphereFacetBounds = Query.Shape == EQueryShape::Sphere &&
		CVarIAmSpeedAnalyticWorldExtrudedSphereFacetBounds.GetValueOnAnyThread() != 0;
	bFacetGeometryCache = CVarIAmSpeedAnalyticWorldExtrudedFacetGeometryCache.GetValueOnAnyThread() != 0 &&
		Patch.SectionFacetGeometry.Num() == Patch.SectionPolyline.Num() - 1;
#endif
	const auto SweepSegment = [&](const int32 Segment)
	{
		// The legacy finite-domain predicate uses dot products onto AxisU/AxisV.
		// For oblique axes its accepted domain is the dual-frame rectangle, not
		// necessarily the physical corner parallelogram enclosed by FacetBounds.
		// Such patches are valid inputs: preserve their original kernel path.
		// Exact perpendicularity is deliberately strict; near-perpendicular and
		// non-unit frames fall back instead of extending a geometric certificate.
		if (bSphereFacetBounds && Patch.ExtrusionAxis.SquaredLength() == 1.0 &&
			FVector3d::DotProduct(Patch.SectionPolyline[Segment + 1] -
				Patch.SectionPolyline[Segment], Patch.ExtrusionAxis) == 0.0)
		{
			// The finite chord rectangle is enclosed by this exact derived AABB.
			// Keep all hit work/order unchanged; enlarge for finite-domain tolerance
			// before using the existing conservative continuously swept-shape test.
			const double Margin = FMath::Sqrt(3.0) * FMath::Max(0.0, Query.DomainTolerance);
			if (!SweptShapeIntersectsBounds(Query,
				Patch.SectionSegmentBounds[Segment].ExpandBy(Margin))) return;
		}

		// A leaf can contain eight chords. Reject its unrelated facets before
		// constructing/triangulating planes for OBBs. Sphere/ray plane tests are
		// already cheap; their broad phase stays at the node level.
		if (bBoxQuery)
		{
			const FBox3d& FacetBounds = Patch.SectionSegmentBounds[Segment];
			if (!FacetBounds.Intersect(QueryBounds) ||
				!SweptShapeIntersectsBounds(Query, FacetBounds, &BoxContext)) return;
		}
		const FVector3d SectionA = Patch.SectionPolyline[Segment];
		const FVector3d SectionB = Patch.SectionPolyline[Segment + 1];
		const FExtrudedQuinticFacetGeometry* Geometry = bFacetGeometryCache
			? &Patch.SectionFacetGeometry[Segment] : nullptr;
		const FVector3d Chord = Geometry ? Geometry->Chord : SectionB - SectionA;
		const double ChordLength = Geometry ? Geometry->ChordLength : Chord.Length();
		if (ChordLength <= UE_DOUBLE_SMALL_NUMBER)
		{
			return;
		}
		FBoundedPlane Face;
		Face.SurfaceId = Patch.SurfaceId;
		Face.SourceId = Patch.SourceId;
		Face.FeatureId = Patch.FeatureId;
		Face.MaterialId = Patch.MaterialId;
		Face.AxisU = Geometry ? Geometry->AxisU : Chord / ChordLength;
		Face.AxisV = Patch.ExtrusionAxis;
		Face.Normal = Geometry ? Geometry->Normal : FVector3d::CrossProduct(Face.AxisU, Face.AxisV).GetSafeNormal();
		Face.Origin = Geometry ? Geometry->Origin : 0.5 * (SectionA + SectionB) + ExtrusionCenter * Patch.ExtrusionAxis;
		if (FVector3d::DotProduct(Query.Start - Face.Origin, Face.Normal) < 0.0)
		{
			Face.Normal = -Face.Normal;
		}
		Face.HalfExtents = Geometry ? Geometry->HalfExtents : FVector2d(0.5 * ChordLength, ExtrusionHalfExtent);
		// Chord endpoints inside the adaptive polyline are approximation seams,
		// not authored surface boundaries.  SweepPlane's authority margin is
		// correct for a standalone bounded plane, but applying it to every chord
		// excludes any sphere whose footprint crosses an internal seam.  Evaluate
		// the finite face first, then retain the footprint margin only at the four
		// actual patch boundaries (the two section endpoints and two extrusion
		// endpoints).
		if (bBoxQuery)
		{
			// A concave polyline's internal chord joins/triangulation diagonals
			// are not physical walls. Both neighbouring chords must lie on the
			// query side of this plane; convex creases and actual patch endpoints
			// retain the finite SAT kernel. Full extrusion-footprint containment
			// also excludes the authored end edges from this face-only path.
			const bool bConcaveInternalFacet = bInsideExtrusion && Segment > 0 &&
				Segment + 2 < Patch.SectionPolyline.Num() &&
				FVector3d::DotProduct(Patch.SectionPolyline[Segment - 1] - SectionA, Face.Normal) >= 0 &&
				FVector3d::DotProduct(Patch.SectionPolyline[Segment + 2] - SectionB, Face.Normal) >= 0;
			FIntPoint PolygonEdge(INDEX_NONE, INDEX_NONE);
			Candidate = SweepPlane(FaceQuery, Face, nullptr, &BoxContext, Selection, &PolygonEdge, PrimarySatScope, CrossNormalScope);
			const int32 EdgeMin = FMath::Min(PolygonEdge.X, PolygonEdge.Y);
			const int32 EdgeMax = FMath::Max(PolygonEdge.X, PolygonEdge.Y);
			// A terminal facet still has one INTERNAL section edge. Its actual
			// opposite boundary must keep SAT, so classify the winning triangle's
			// polygon corners, never its ambiguous triangle-local edge index.
			const bool bConcaveSectionEdge = bInsideExtrusion &&
				((EdgeMin == 0 && EdgeMax == 3 && Segment > 0 &&
					FVector3d::DotProduct(Patch.SectionPolyline[Segment - 1] - SectionA, Face.Normal) >= 0) ||
				 (EdgeMin == 1 && EdgeMax == 2 && Segment + 2 < Patch.SectionPolyline.Num() &&
					FVector3d::DotProduct(Patch.SectionPolyline[Segment + 2] - SectionB, Face.Normal) >= 0));
			if ((bConcaveInternalFacet || bConcaveSectionEdge) && Candidate.bHit)
			{
				// Normal projection of a penetrating support vertex can lie in
				// the face even when the ORIGINAL OBB misses that finite facet.
				// Keep finite SAT as the overlap/TOI proof; bounds pruning must
				// remain observationally equivalent to an unpruned narrow phase.
				const FVector3d N = Face.Normal;
				FVector3d SupportPoint = Query.Start;
				int32 ActiveAxes = 0;
				int8 Vertex = 0;
				for (int32 Axis = 0; Axis < 3; ++Axis)
				{
					const double D = FVector3d::DotProduct(-N, BoxContext.Axis[Axis]);
					const double Sign = D >= 0 ? 1 : -1;
					if (Sign > 0) Vertex |= 1 << Axis;
					if (FMath::Abs(D) > 1.0e-10) ++ActiveAxes;
					SupportPoint += Sign * Query.HalfExtent[Axis] * BoxContext.Axis[Axis];
				}
				const FVector3d Motion = Query.End - Query.Start;
				const double Gap = FVector3d::DotProduct(SupportPoint - Face.Origin, N);
				const double NormalMotion = FVector3d::DotProduct(Motion, N);
				double Time = 0;
				if (Gap > 0)
				{
					if (NormalMotion >= 0) return;
					Time = -Gap / NormalMotion;
				}
				if (Time > 1 || Time + 1.0e-12 < Candidate.Time) return;
				const double Depth = FMath::Max(0.0, -Gap);
				SupportPoint += Time * Motion;
				const FVector3d FacePoint = SupportPoint + Depth * N;
				if (!Face.ContainsProjectedPoint(FacePoint, Query.DomainTolerance)) return;
				// Recompute the normal/depth/witness as one finite face constraint;
				// smoothing only the SAT normal would misrepresent penetration.
				Candidate.bStartPenetrating = Depth > Query.DomainTolerance;
				if (Candidate.bStartPenetrating && Depth <= FMath::Max(0.0, Query.InitialOverlapTolerance))
				{
					if (-FVector3d::DotProduct(Query.End - Query.Start, N) <= Query.DomainTolerance) return;
					Candidate.bStartPenetrating = false;
				}
				Candidate.Normal = N;
				Candidate.Time = Time;
				Candidate.Location = Query.Start + Time * Motion;
				Candidate.Point = FacePoint;
				Candidate.QueryPoint = SupportPoint;
				Candidate.PenetrationDepth = Candidate.bStartPenetrating ? Depth : 0;
				Candidate.SurfaceFeatureKind = EContactFeatureKind::Face;
				Candidate.SurfaceFeatureIndex = INDEX_NONE;
				Candidate.QueryFeatureKind = ActiveAxes <= 1 ? EContactFeatureKind::Face :
					ActiveAxes == 2 ? EContactFeatureKind::Edge : EContactFeatureKind::Vertex;
				Candidate.QueryFeatureIndex = Vertex;
			}
		}
		else if (!TrySweepRoundPlane(FaceQuery, Face, Candidate))
		{
			return;
		}
		// A miss cannot enter arbitration. In particular, do not hash a stable
		// primitive id or populate witness metadata for every rejected facet.
		if (!Candidate.bHit) return;
		const bool bProvenFiniteContact = Query.bUseFiniteContactDomain &&
			(bBoxQuery || (Query.Shape == EQueryShape::Sphere &&
				Candidate.SurfaceFeatureKind == EContactFeatureKind::Face));
		if ((Query.bAuthorityOnly || Patch.bAuthorityEligible) && !bProvenFiniteContact)
		{
			// This is conservative replacement coverage, not the geometry of a
			// finite intersection. The round witness or finite OBB SAT has already
			// proved contact. Erosion in strict mode can hide a real intersection
			// until its chosen witness abruptly enters this smaller coverage area.
			// A certified provider owns only the source-certified finite domain. A
			// volume contacting within one support radius of an authored patch
			// endpoint cannot use the synthetic terminal edge as a wall; a
			// neighbouring provider (or a definitive miss) must own that part of
			// the query instead. Apply the rule to certified providers as well as
			// explicitly authority-only queries because an authority-pruned runtime
			// world can issue ordinary queries after filtering providers at build
			// time.
			const double RequiredExtrusionMargin = SupportRadius(
				Query, Patch.ExtrusionAxis);
			const double RequiredSectionMargin = SupportRadius(Query, Face.AxisU);
			const double ExtrusionCoordinate = FVector3d::DotProduct(
				Candidate.Point, Patch.ExtrusionAxis);
			const double AlongChord = FVector3d::DotProduct(
				Candidate.Point - SectionA, Face.AxisU);
			const bool bCrossesExtrusionBoundary =
				ExtrusionCoordinate + Query.DomainTolerance <
					Patch.MinimumExtrusionCoordinate + RequiredExtrusionMargin ||
				ExtrusionCoordinate - Query.DomainTolerance >
					Patch.MaximumExtrusionCoordinate - RequiredExtrusionMargin;
			const bool bCrossesSectionStart = Segment == 0 &&
				AlongChord + Query.DomainTolerance < RequiredSectionMargin;
			const bool bCrossesSectionEnd =
				Segment + 2 == Patch.SectionPolyline.Num() &&
				AlongChord - Query.DomainTolerance >
					ChordLength - RequiredSectionMargin;
			if (bCrossesExtrusionBoundary || bCrossesSectionStart ||
				bCrossesSectionEnd)
			{
				return;
			}
		}
		Candidate.PrimitiveId = CombineStableIds(
			Patch.PrimitiveId, static_cast<uint64>(Segment + 1));
		Candidate.CanonicalGroupId = Patch.CanonicalGroupId;
		Candidate.GeometricErrorBoundCm = Patch.MaximumChordErrorCm;
		Candidate.AdditionalResidualAgreementAllowanceCm =
			Patch.AdditionalResidualAgreementAllowanceCm;
		Candidate.bSurfaceNormalMayVary = Candidate.bHit;
		// A bounded-plane triangulation can report its internal diagonal as an
		// edge witness.  That diagonal is not a physical crease.  In the gutter
		// transition, its near-horizontal SAT normal creates false support; use
		// the authored derivative only for internal, near-horizontal witnesses.
		// Endpoint and oblique wall edges retain their physical boundary normal.
		const bool bInternalSectionEdge =
			Candidate.SurfaceFeatureKind == EContactFeatureKind::Edge &&
			Segment > 0 && Segment + 1 < Patch.SectionPolyline.Num() - 1 &&
			FMath::Abs(Candidate.Normal.Z) > 0.90;
		if (Candidate.bHit && !Candidate.bStartPenetrating &&
			(Candidate.SurfaceFeatureKind == EContactFeatureKind::Face ||
				bInternalSectionEdge))
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
		if (Candidate.bHit && HitPassesReferenceNormal(Query, Candidate) &&
			IsBetterHit(Candidate, Best, Selection))
		{
			Best = Candidate;
		}
	};
	const auto SweepProviderFacets = [&]()
	{
		TArray<int32, TInlineAllocator<32>> NodeStack;
		NodeStack.Add(0);
		while (!NodeStack.IsEmpty())
		{
			const int32 NodeIndex = NodeStack.Pop(EAllowShrinking::No);
			if (!Patch.SectionSegmentBvhNodes.IsValidIndex(NodeIndex)) continue;
			const FTriangleBvhNode& Node = Patch.SectionSegmentBvhNodes[NodeIndex];
			if (!Node.Bounds.Intersect(QueryBounds)) continue;
			if (!SweptShapeIntersectsBounds(Query, Node.Bounds, &BoxContext)) continue;
			if (!Node.IsLeaf())
			{
				NodeStack.Add(Node.RightChild);
				NodeStack.Add(Node.LeftChild);
				continue;
			}
			for (int32 Offset = 0; Offset < Node.IndexCount; ++Offset)
			{
				const int32 PermutationIndex = Node.FirstIndex + Offset;
				if (Patch.SectionSegmentBvhIndices.IsValidIndex(PermutationIndex))
				{
					SweepSegment(Patch.SectionSegmentBvhIndices[PermutationIndex]);
				}
			}
		}
	};
#if !UE_BUILD_SHIPPING
    const int32 ExactCrossMode784 = CVarExactCrossNormalCache784.GetValueOnAnyThread();
    if (bBoxQuery && BoxContext.bLazyExtrudedSatAxes && ExactCrossMode784 == 0 &&
        CVarExactCrossCostProbe844.GetValueOnAnyThread() == 0 &&
        CVarReplayDecomposition925.GetValueOnAnyThread() == 1 &&
        CVarIAmSpeedAnalyticWorldExtrudedBoxPrimarySatScopeCache.GetValueOnAnyThread() == 0)
    {
        CaseSatDiagnostics::FOwnedForwardIdentity917 Identity;
        if (!CaseSatDiagnostics::TryCopyOwnedForwardIdentity917(Identity)) Private::FReplayProbe925::Unavailable();
        else if (GReplayProviderOrdinal925 == MAX_uint64) Private::FReplayProbe925::State().Overflow.store(true,std::memory_order_relaxed);
        else if (++GReplayProviderOrdinal925 % 4096 == 0)
        {
            uint64 ReportId=0;
            if (Private::FReplayProbe925::Claim(ReportId))
            {
                Private::FExactCrossNormalCapture844 Capture;
                Private::FExactCrossNormalScope ProbeScope(FaceQuery,BoxContext,true,&Capture);
                CrossNormalScope=&ProbeScope;
                SweepProviderFacets();
                Private::FReplayProbe925::Report(Capture,FaceQuery,BoxContext,Identity,ReportId);
                return Best;
            }
        }
    }

    if (bBoxQuery && BoxContext.bLazyExtrudedSatAxes && ExactCrossMode784 == 0 &&
        CVarExactCrossCostProbe844.GetValueOnAnyThread() == 1)
    {
        // Bounded diagnostic ownership is thread/provider only. Case/frame/tails UNKNOWN.
        if (GCostProviderOrdinal844 != MAX_uint64) ++GCostProviderOrdinal844;
        if (GCostSamples844 < 32 && GCostProviderOrdinal844 % 4096 == 0)
        {
            ++GCostSamples844;
            Private::FExactCrossNormalCapture844 Capture;
            Private::FExactCrossNormalScope ProbeScope(FaceQuery, BoxContext, true, &Capture);
            CrossNormalScope = &ProbeScope;
            if (CVarIAmSpeedAnalyticWorldExtrudedBoxPrimarySatScopeCache.GetValueOnAnyThread() != 0)
            {
                Private::FPrimarySatProjectionScope PrimaryScope(FaceQuery, BoxContext);
                PrimarySatScope = &PrimaryScope;
                SweepProviderFacets();
            }
            else SweepProviderFacets();
            Private::FExactCrossCostProbe844::Report(Capture, FaceQuery, BoxContext, GCostProviderOrdinal844);
            return Best;
        }
    }
    if (bBoxQuery && BoxContext.bLazyExtrudedSatAxes && (ExactCrossMode784 == 1 || ExactCrossMode784 == 2))
    {
        Private::FExactCrossNormalScope CrossScope(FaceQuery, BoxContext, ExactCrossMode784 == 1);
        CrossNormalScope = &CrossScope;
        if (CVarIAmSpeedAnalyticWorldExtrudedBoxPrimarySatScopeCache.GetValueOnAnyThread() != 0)
        {
            Private::FPrimarySatProjectionScope PrimaryScope(FaceQuery, BoxContext);
            PrimarySatScope = &PrimaryScope;
            SweepProviderFacets();
        }
        else SweepProviderFacets();
        if (ExactCrossMode784 == 1) RecordExactCrossProbe784(CrossScope.GetObservationStats());
        return Best;
    }
	if (bBoxQuery && CVarIAmSpeedAnalyticWorldExtrudedBoxPrimarySatScopeCache.GetValueOnAnyThread() != 0)
	{
		// Scratch has a lifetime only in this flag1 Box branch. It and both
		// bound objects outlive every synchronous forwarded SAT call.
		Private::FPrimarySatProjectionScope Scope(FaceQuery, BoxContext);
		PrimarySatScope = &Scope;
		SweepProviderFacets();
		return Best;
	}
#endif
	SweepProviderFacets();
	return Best;
}

FWorldHit FWorldQueryService::TestRetainedExtrudedSweep938(
    const FWorldQuery& Query, const FExtrudedQuinticPatch& Patch,
    bool bRetained938, int32 Lanes, bool bDeepest,
    Speed::FIndependentQueryBatchStats* OutDispatch)
{
    const EHitSelection Selection = bDeepest ? EHitSelection::DeepestOverlap : EHitSelection::FirstSweepHit;
    const FBox3d Bounds = SweptQueryBounds(Query);
    if (bRetained938)
    {
        if (OutDispatch) *OutDispatch = {};
        return SweepExtrudedQuinticRetained938(Query, Bounds, Patch, Selection, nullptr);
    }
    return SweepExtrudedQuintic(Query, Bounds, Patch, Selection, nullptr, Lanes, 2, OutDispatch, nullptr);
}
#endif

FWorldHit FWorldQueryService::SweepTensorBezier(
	const FWorldQuery& Query, const FBox3d& QueryBounds,
	const FTensorBezierPatch& Patch, const double MaximumSearchTime,
	const EHitSelection Selection)
{
    FScopedAnalyticCausalObservation1075 Causal(EAnalyticCausalKind1075::TensorProvider);
    Causal.SetItems(1);
	FWorldHit Best;
	TRACE_CPUPROFILER_EVENT_SCOPE(IAmSpeed_AnalyticSweepTensorBezier);
	if (!PatchPassesFilter(Query, Patch) ||
		!Patch.Bounds.Intersect(QueryBounds) ||
		!Patch.bApproximationCertified)
	{
		return Best;
	}
	FTensorBezierQueryView View;
	View.SourceId = Patch.SourceId;
	View.SurfaceId = Patch.SurfaceId;
	View.FeatureId = Patch.FeatureId;
	View.PrimitiveId = Patch.PrimitiveId;
	View.CanonicalGroupId = Patch.CanonicalGroupId;
	View.MaterialId = Patch.MaterialId;
	View.ObjectType = Patch.ObjectType;
	View.BlockingChannels = Patch.BlockingChannels;
	View.Surface = &Patch.Surface;
	View.Bounds = Patch.Bounds;
	View.ApproximationCells = &Patch.ApproximationCells;
	View.ApproximationCellBvhNodes = &Patch.ApproximationCellBvhNodes;
	View.ApproximationCellBvhIndices = &Patch.ApproximationCellBvhIndices;
	View.bQueryCollisionEnabled = Patch.bQueryCollisionEnabled;
	View.bApproximationCertified = Patch.bApproximationCertified;
	View.bAuthorityEligible = Patch.bAuthorityEligible;
	return SweepTensorBezierApproximation(
		Query, QueryBounds, View, MaximumSearchTime, nullptr, Selection);
}

FWorldHit FWorldQueryService::SweepTensorBezierApproximation(
	const FWorldQuery& Query, const FBox3d& QueryBounds,
	const FTensorBezierQueryView& Patch, const double MaximumSearchTime,
	const FVector3d* CachedBoxAxes, const EHitSelection Selection)
{
    FScopedAnalyticCausalObservation1075 Causal(EAnalyticCausalKind1075::TensorApproximation);
    Causal.SetItems(0);
	FWorldHit Best;
	int32 BestApproximationIndex = INDEX_NONE;
	int32 BestTriangleIndex = INDEX_NONE;
	const double InitialMaximumSearchTime =
		FMath::Clamp(MaximumSearchTime, 0.0, 1.0);
	const auto CurrentMaximumSearchTime = [&Best, InitialMaximumSearchTime]()
	{
		return Best.bHit
			? FMath::Min(InitialMaximumSearchTime, Best.Time + 1.0e-12)
			: InitialMaximumSearchTime;
	};
	if (!Patch.Surface || !Patch.ApproximationCells ||
		!Patch.ApproximationCellBvhNodes ||
		!Patch.ApproximationCellBvhIndices ||
		!Patch.Bounds.Intersect(QueryBounds) ||
		!Patch.bApproximationCertified)
	{
		return Best;
	}
	FBoxBoundsSweepContext BoxSweepContext;
	if (Query.Shape == EQueryShape::Box)
	{
		BoxSweepContext.Initialize(Query, CachedBoxAxes);
	}
	Causal.MarkPrepared();
	const auto SweepCell = [&](const int32 CellIndex)
	{
        FScopedAnalyticCycles1075 CellNumeric(Causal.NumericTarget(0));
        Causal.AddItem();
#if !UE_BUILD_SHIPPING
		if (GAnalyticQueryDetailedTimingEnabled)
		{
			++GAnalyticApproximationCellVisits;
		}
#endif
		const FTensorBezierApproximationCell& Cell =
			(*Patch.ApproximationCells)[CellIndex];
		FBox3d CellBounds(EForceInit::ForceInit);
		for (const FVector3d& Corner : Cell.Corners)
		{
			CellBounds += Corner;
		}
		if (!SweptShapeIntersectsBounds(Query,
			CellBounds.ExpandBy(Cell.MaximumErrorCm),
			Query.Shape == EQueryShape::Box ? &BoxSweepContext : nullptr,
			CurrentMaximumSearchTime()))
		{
			return;
		}
		static constexpr int32 CornerIndices[2][3] = {
			{ 0, 2, 3 }, { 0, 3, 1 } };
		const FVector2d ParameterCorners[4] = {
			FVector2d(Cell.MinimumU, Cell.MinimumV),
			FVector2d(Cell.MinimumU, Cell.MaximumV),
			FVector2d(Cell.MaximumU, Cell.MinimumV),
			FVector2d(Cell.MaximumU, Cell.MaximumV) };
		for (int32 TriangleIndex = 0; TriangleIndex < 2; ++TriangleIndex)
		{
			FVector3d TriangleVertices[3];
			for (int32 Corner = 0; Corner < 3; ++Corner)
			{
				const int32 SourceCorner = CornerIndices[TriangleIndex][Corner];
				TriangleVertices[Corner] = Cell.Corners[SourceCorner];
			}
			FBox3d TriangleBounds(EForceInit::ForceInit);
			for (const FVector3d& Vertex : TriangleVertices)
			{
				TriangleBounds += Vertex;
			}
			if (!TriangleBounds.ExpandBy(Cell.MaximumErrorCm).Intersect(QueryBounds))
			{
				continue;
			}
			FWorldHit Candidate;
			if (Query.Shape == EQueryShape::Box)
			{
				FPlaneTriangle PlaneTriangle;
				for (int32 Corner = 0; Corner < 3; ++Corner)
				{
					PlaneTriangle.Vertex[Corner] = TriangleVertices[Corner];
					PlaneTriangle.PolygonVertex[Corner] = static_cast<int8>(Corner);
				}
				if (!SweepBoxTriangle(
					Query, PlaneTriangle, Candidate, &BoxSweepContext,
					CurrentMaximumSearchTime()))
				{
					continue;
				}
				bool bOutsideInternalCornerCone = false;
				if (Candidate.bStartPenetrating && Patch.InternalCornerNormalCones &&
					Patch.InternalCornerNormalConeIndices)
				{
					double MaximumProjection = -TNumericLimits<double>::Max();
					for (const auto& P : TriangleVertices)
						MaximumProjection=FMath::Max(MaximumProjection,FVector3d::DotProduct(P,Candidate.Normal));
					for (int32 K=0; K<3; ++K)
					{
						// Inspect the triangle's actual extreme vertex along the MTD,
						// not the support-corner shortcut's possibly clipped witness.
						if (FVector3d::DotProduct(TriangleVertices[K],Candidate.Normal) < MaximumProjection-Query.DomainTolerance) continue;
						const auto& UV=ParameterCorners[CornerIndices[TriangleIndex][K]];
						if ((UV.X!=0 && UV.X!=1) || (UV.Y!=0 && UV.Y!=1)) continue;
						const int32 Corner=(UV.X==1 ? 2 : 0)+(UV.Y==1 ? 1 : 0);
						const int32 Index=Patch.InternalCornerNormalConeIndices[Corner];
						if (!Patch.InternalCornerNormalCones->IsValidIndex(Index)) continue;
						const auto& Cone=(*Patch.InternalCornerNormalCones)[Index];
						FVector3d Axis(Cone.X,Cone.Y,Cone.Z);
						if (FVector3d::DotProduct(Query.Start-TriangleVertices[K],Axis)<0) Axis=-Axis;
						bOutsideInternalCornerCone |= FVector3d::DotProduct(Candidate.Normal,Axis)<Cone.W-1.0e-12;
					}
				}
				const bool bEdgeWitness=Candidate.SurfaceFeatureKind==EContactFeatureKind::Edge &&
					Candidate.SurfaceFeatureIndex>=0 && Candidate.SurfaceFeatureIndex<3;
				if (Candidate.bStartPenetrating && (bEdgeWitness || bOutsideInternalCornerCone))
				{
					// Non-overlapping tensor hits already recover the exact smooth
					// normal below. Only an initial MTD couples an invalid internal
					// edge direction to a penetration depth and needs reconstruction.
					FVector3d N = FVector3d::CrossProduct(TriangleVertices[1]-TriangleVertices[0],
						TriangleVertices[2]-TriangleVertices[0]).GetSafeNormal();
					const bool bPositiveSide = FVector3d::DotProduct(Query.Start-TriangleVertices[0],N) >= 0;
					const uint8 ConcaveEdges = bPositiveSide ? Cell.PositiveConcaveEdges[TriangleIndex] :
						Cell.NegativeConcaveEdges[TriangleIndex];
					if (bOutsideInternalCornerCone || (bEdgeWitness && (ConcaveEdges & (1u << Candidate.SurfaceFeatureIndex))))
					{
						// SAT proves a finite collision, but its concave INTERNAL edge
						// escape axis is not a boundary of the joined surface. Rebuild
						// a coherent finite face constraint, or let the incident facet
						// own the contact when this face's support projection is outside.
						// True boundaries, convex edges and uncertified joins keep SAT.
						if (!bPositiveSide) N = -N;
						FVector3d SupportPoint = Query.Start;
						int32 ActiveAxes = 0;
						int8 Vertex = 0;
						for (int32 Axis=0; Axis<3; ++Axis)
						{
							const double D = FVector3d::DotProduct(-N,BoxSweepContext.Axis[Axis]);
							const double Sign = D >= 0 ? 1 : -1;
							if (Sign > 0) Vertex |= 1 << Axis;
							if (FMath::Abs(D)>1.0e-10) ++ActiveAxes;
							SupportPoint += Sign*Query.HalfExtent[Axis]*BoxSweepContext.Axis[Axis];
						}
						const FVector3d Motion = Query.End-Query.Start;
						const double Gap = FVector3d::DotProduct(SupportPoint-TriangleVertices[0],N);
						const double NormalMotion = FVector3d::DotProduct(Motion,N);
						double Time = 0;
						if (Gap > 0)
						{
							if (NormalMotion >= 0) continue;
							Time = -Gap/NormalMotion;
						}
						if (Time>CurrentMaximumSearchTime() || Time+1.0e-12<Candidate.Time) continue;
						const double Depth = FMath::Max(0.0,-Gap);
						SupportPoint += Time*Motion;
						const FVector3d FacePoint = SupportPoint+Depth*N;
						const FVector3d Closest = ClosestPointOnTriangle(FacePoint,
							TriangleVertices[0],TriangleVertices[1],TriangleVertices[2]);
						// Projection and barycentric reconstruction round differently,
						// even for a point strictly inside this finite face. A zero
						// domain allowance must not erase a SAT-proven overlap. Bound
						// only the membership arithmetic by coordinate-scale roundoff;
						// keep SAT, physical depth and its acceptance threshold intact.
						const double MembershipRoundoff = 64 * DBL_EPSILON * FMath::Max(1.0,
							FacePoint.GetAbsMax() + TriangleBounds.Min.GetAbsMax() + TriangleBounds.Max.GetAbsMax());
						if (!Closest.Equals(FacePoint,FMath::Max(Query.DomainTolerance,MembershipRoundoff))) continue;
						Candidate.bStartPenetrating = Depth>Query.DomainTolerance;
						if (Candidate.bStartPenetrating && Depth<=FMath::Max(0.0,Query.InitialOverlapTolerance))
						{
							if (-NormalMotion<=Query.DomainTolerance) continue;
							Candidate.bStartPenetrating = false;
						}
						Candidate.Normal=N;
						Candidate.Time=Time;
						Candidate.Location=Query.Start+Time*Motion;
						Candidate.Point=FacePoint;
						Candidate.QueryPoint=SupportPoint;
						Candidate.PenetrationDepth=Candidate.bStartPenetrating ? Depth : 0;
						Candidate.SurfaceFeatureKind=EContactFeatureKind::Face;
						Candidate.SurfaceFeatureIndex=INDEX_NONE;
						Candidate.QueryFeatureKind=ActiveAxes<=1 ? EContactFeatureKind::Face :
							ActiveAxes==2 ? EContactFeatureKind::Edge : EContactFeatureKind::Vertex;
						Candidate.QueryFeatureIndex=Vertex;
					}
				}
			}
			else if (Query.Shape == EQueryShape::Sphere)
			{
				const FVector3d TriangleFaceNormal = FVector3d::CrossProduct(
					TriangleVertices[1] - TriangleVertices[0],
					TriangleVertices[2] - TriangleVertices[0]);
				// The runtime triangle is a bilinear approximation of the
				// certified Bezier cell.  Its Hausdorff error is carried by the
				// cell and must enlarge the swept shell during detection; using
				// only the triangle itself creates false-negative contacts near
				// tangent/end-point crossings.  Keep this allowance local to the
				// approximation cell so exact providers are unaffected.
				if (!SweepSphereTriangleGeometry(
					Query, TriangleVertices, TriangleFaceNormal, Candidate,
					CurrentMaximumSearchTime()))
				{
					continue;
				}
			}
			else
			{
				FTriangleSurface Triangle;
				const FVector3d TriangleFaceNormal = FVector3d::CrossProduct(
					TriangleVertices[1] - TriangleVertices[0],
					TriangleVertices[2] - TriangleVertices[0]);
				Triangle.SourceId = Patch.SourceId;
				Triangle.SurfaceId = Patch.SurfaceId;
				Triangle.FeatureId = Patch.FeatureId;
				Triangle.MaterialId = Patch.MaterialId;
				Triangle.ObjectType = Patch.ObjectType;
				Triangle.BlockingChannels = Patch.BlockingChannels;
				Triangle.bQueryCollisionEnabled = Patch.bQueryCollisionEnabled;
				Triangle.bAuthorityEligible = Patch.bAuthorityEligible;
				Triangle.FaceNormal = TriangleFaceNormal.GetSafeNormal();
				Triangle.Bounds = TriangleBounds;
				for (int32 Corner = 0; Corner < 3; ++Corner)
				{
					Triangle.Vertices[Corner] = TriangleVertices[Corner];
					Triangle.VertexNormals[Corner] = TriangleFaceNormal;
				}
				FWorldQuery TriangleQuery = Query;
				// Implementation triangles inherit the owning canonical family.
				TriangleQuery.RequiredCanonicalGroupId = 0;
				Candidate = SweepTriangleFace(TriangleQuery, Triangle);
			}
			if (!Candidate.bHit) continue;
			Candidate.SourceId = Patch.SourceId;
			Candidate.SurfaceId = Patch.SurfaceId;
			Candidate.FeatureId = Patch.FeatureId;
			// Synthetic triangle ids do not participate in the local geometric
			// kernel/filter. Decorate only real hits, before any provider arbitration.
			Candidate.PrimitiveId = CombineStableIds(Patch.PrimitiveId,
				static_cast<uint64>(2 * CellIndex + TriangleIndex + 1));
			Candidate.MaterialId = Patch.MaterialId;
			Candidate.CanonicalGroupId = Patch.CanonicalGroupId;
			Candidate.GeometricErrorBoundCm = Cell.MaximumErrorCm;
			Candidate.bSurfaceNormalMayVary = true;
			if (!Candidate.bStartPenetrating)
			{
				const FVector3d A = TriangleVertices[0];
				const FVector3d B = TriangleVertices[1];
				const FVector3d C = TriangleVertices[2];
				const FVector3d V0 = B - A;
				const FVector3d V1 = C - A;
				const FVector3d V2 = Candidate.Point - A;
				const double D00 = FVector3d::DotProduct(V0, V0);
				const double D01 = FVector3d::DotProduct(V0, V1);
				const double D11 = FVector3d::DotProduct(V1, V1);
				const double D20 = FVector3d::DotProduct(V2, V0);
				const double D21 = FVector3d::DotProduct(V2, V1);
				const double Denominator = D00 * D11 - D01 * D01;
				if (FMath::Abs(Denominator) > 1.0e-18)
				{
					const double WeightB = (D11 * D20 - D01 * D21) /
						Denominator;
					const double WeightC = (D00 * D21 - D01 * D20) /
						Denominator;
					const double WeightA = 1.0 - WeightB - WeightC;
					const FVector2d UV =
						WeightA * ParameterCorners[CornerIndices[TriangleIndex][0]] +
						WeightB * ParameterCorners[CornerIndices[TriangleIndex][1]] +
						WeightC * ParameterCorners[CornerIndices[TriangleIndex][2]];
					FVector3d SmoothNormal =
						Patch.Surface->EvaluateNormal(UV.X, UV.Y);
					if (FVector3d::DotProduct(SmoothNormal, Candidate.Normal) < 0.0)
					{
						SmoothNormal = -SmoothNormal;
					}
					if (!SmoothNormal.IsNearlyZero()) Candidate.Normal = SmoothNormal;
				}
			}
			const bool bSameInitialOverlap = Best.bHit &&
				Candidate.bStartPenetrating && Best.bStartPenetrating &&
				FMath::IsNearlyEqual(Candidate.Time, Best.Time, 1.0e-12);
			const bool bDeeperTensorConstraint = bSameInitialOverlap &&
				Candidate.PenetrationDepth > Best.PenetrationDepth + 1.0e-12;
			const bool bEqualTensorConstraint = bSameInitialOverlap &&
				FMath::IsNearlyEqual(Candidate.PenetrationDepth,
					Best.PenetrationDepth, 1.0e-12);
			// Preserve the historical tensor CCD rule exactly. Pose overlap must
			// also reject tangent incumbents and order exact depth ties by identity.
			const bool bPreferCandidate = Selection == EHitSelection::DeepestOverlap
				? IsBetterHit(Candidate, Best, Selection)
				: (bDeeperTensorConstraint ||
					(!bSameInitialOverlap && IsBetterHit(Candidate, Best)) ||
					(bEqualTensorConstraint && IsBetterHit(Candidate, Best)));
			if (HitPassesReferenceNormal(Query, Candidate) && bPreferCandidate)
			{
				Best = Candidate;
				BestApproximationIndex = CellIndex;
				BestTriangleIndex = TriangleIndex;
			}
		}
	};
	TArray<int32, TInlineAllocator<64>> NodeStack;
	NodeStack.Add(0);
	while (!NodeStack.IsEmpty())
	{
#if !UE_BUILD_SHIPPING
		if (GAnalyticQueryDetailedTimingEnabled)
		{
			++GAnalyticApproximationNodeVisits;
		}
#endif
		const int32 NodeIndex = NodeStack.Pop(EAllowShrinking::No);
		if (!Patch.ApproximationCellBvhNodes->IsValidIndex(NodeIndex)) continue;
		const FTriangleBvhNode& Node =
			(*Patch.ApproximationCellBvhNodes)[NodeIndex];
		if (!Node.Bounds.Intersect(QueryBounds) ||
			!SweptShapeIntersectsBounds(Query, Node.Bounds,
				Query.Shape == EQueryShape::Box ? &BoxSweepContext : nullptr,
				CurrentMaximumSearchTime()))
		{
			continue;
		}
		if (Node.IsLeaf())
		{
			for (int32 Offset = 0; Offset < Node.IndexCount; ++Offset)
			{
				const int32 PermutationIndex = Node.FirstIndex + Offset;
				if (Patch.ApproximationCellBvhIndices->IsValidIndex(
					PermutationIndex))
				{
					SweepCell((*Patch.ApproximationCellBvhIndices)[
						PermutationIndex]);
				}
			}
		}
		else
		{
			// Push right first so the stable left branch is evaluated first.
			if (Node.RightChild != INDEX_NONE) NodeStack.Add(Node.RightChild);
			if (Node.LeftChild != INDEX_NONE) NodeStack.Add(Node.LeftChild);
		}
	}
	if (Best.bStartPenetrating && Query.Shape == EQueryShape::Box &&
		BestApproximationIndex != INDEX_NONE &&
		FMath::Abs(FVector3d::DotProduct(Best.Point-Best.QueryPoint,Best.Normal)-
			Best.PenetrationDepth) > Query.DomainTolerance)
	{
		// A support corner is not necessarily the touching witness when the
		// minimum SAT axis belongs to an edge. Recover the actual closest pair
		// on the TRANSLATED box, then undo only the measured MTD on its witness.
		// This runs once for the selected inconsistent overlap, not per facet;
		// normal, depth, TOI and finite intersection/arbitration stay unchanged.
		static constexpr int32 Indices[2][3] = {{0,2,3},{0,3,1}};
		FPlaneTriangle Triangle;
		const auto& Cell = (*Patch.ApproximationCells)[BestApproximationIndex];
		for (int32 Corner=0; Corner<3; ++Corner)
		{
			Triangle.Vertex[Corner]=Cell.Corners[Indices[BestTriangleIndex][Corner]];
			Triangle.PolygonVertex[Corner]=static_cast<int8>(Corner);
		}
		const FContactWitness Witness = ClosestBoxTriangleWitness(
			Best.Location+Best.PenetrationDepth*Best.Normal,
			BoxSweepContext.Axis,Query.HalfExtent,Triangle);
		if (Witness.DistanceSquared <= FMath::Square(Query.DomainTolerance))
		{
			Best.Point=Witness.SurfacePoint;
			Best.QueryPoint=Witness.QueryPoint-Best.PenetrationDepth*Best.Normal;
			Best.SurfaceFeatureKind=Witness.SurfaceKind;
			Best.SurfaceFeatureIndex=Witness.SurfaceIndex;
			Best.QueryFeatureKind=Witness.QueryKind;
			Best.QueryFeatureIndex=Witness.QueryIndex;
		}
	}
	return Best;
}

FWorldHit FWorldQueryService::SweepPiecewiseTensorBezier(
	const FWorldQuery& Query, const FBox3d& QueryBounds,
	const FPiecewiseTensorBezierPatch& Patch,
	const double MaximumSearchTime, const EHitSelection Selection, const int32 InTensorCellLanes1213)
{
    FScopedAnalyticCausalObservation1075 Causal(EAnalyticCausalKind1075::PiecewiseProvider);
    Causal.SetItems(0);
	FWorldHit Best;
	const double InitialMaximumSearchTime =
		FMath::Clamp(MaximumSearchTime, 0.0, 1.0);
	const auto CurrentMaximumSearchTime = [&Best, InitialMaximumSearchTime]()
	{
		return Best.bHit
			? FMath::Min(InitialMaximumSearchTime, Best.Time + 1.0e-12)
			: InitialMaximumSearchTime;
	};
	TRACE_CPUPROFILER_EVENT_SCOPE(IAmSpeed_AnalyticSweepPiecewiseTensorBezier);
#if !UE_BUILD_SHIPPING
	const double DetailedStartSeconds = GAnalyticQueryDetailedTimingEnabled
		? FPlatformTime::Seconds()
		: 0.0;
	const uint64 ApproximationNodeVisitsBefore =
		GAnalyticApproximationNodeVisits;
	const uint64 ApproximationCellVisitsBefore =
		GAnalyticApproximationCellVisits;
	uint64 ProviderNodeVisits = 0;
	uint64 ProviderCellVisits = 0;
#endif
	if (!PatchPassesFilter(Query, Patch) || !Patch.bApproximationCertified ||
		!Patch.Bounds.Intersect(QueryBounds))
	{
		return Best;
	}
	FBoxBoundsSweepContext ProviderBoxContext;
	if (Query.Shape == EQueryShape::Box) ProviderBoxContext.Initialize(Query);
	Causal.MarkPrepared();
	if (!SweptShapeIntersectsBounds(Query, Patch.Bounds,
		Query.Shape == EQueryShape::Box ? &ProviderBoxContext : nullptr,
		CurrentMaximumSearchTime()))
	{
		return Best;
	}
    const auto MakeCellView1213=[&](const FPiecewiseTensorBezierCell& Cell)
    {
        FTensorBezierQueryView CellView;
        CellView.SourceId = Patch.SourceId;
        CellView.SurfaceId = Patch.SurfaceId;
        CellView.FeatureId = Cell.FeatureId;
        CellView.PrimitiveId = Cell.PrimitiveId;
        CellView.CanonicalGroupId = Patch.CanonicalGroupId;
        CellView.MaterialId = Patch.MaterialId;
        CellView.ObjectType = Patch.ObjectType;
        CellView.BlockingChannels = Patch.BlockingChannels;
        CellView.Surface = &Cell.Surface;
        CellView.Bounds = Cell.Bounds;
        CellView.ApproximationCells = &Cell.ApproximationCells;
        CellView.ApproximationCellBvhNodes =
            &Cell.ApproximationCellBvhNodes;
        CellView.ApproximationCellBvhIndices =
            &Cell.ApproximationCellBvhIndices;
        CellView.InternalCornerNormalCones=&Patch.InternalCornerNormalCones;
        CellView.InternalCornerNormalConeIndices=Cell.InternalCornerNormalCone;
        CellView.bQueryCollisionEnabled = Patch.bQueryCollisionEnabled;
        CellView.bApproximationCertified = true;
        CellView.bAuthorityEligible = Patch.bAuthorityEligible;
        return CellView;
    };
    TOptional<FJoinedTensorCellWave1213> CellWave1213;
    const FTensorCellWaveIdentity1213 CellIdentity1213{
        &Patch,Patch.CellBvhNodes.GetData(),&ProviderBoxContext,uint8(Selection)};
    const bool CanTryCellWave1213=IsTensorCellWave1213Enabled() && InTensorCellLanes1213>1 &&
        Patch.Cells.Num()>FJoinedTensorCellWave1213::MinimumCandidateItems() &&
        !IsInsideFrameParallelTask() && CanSpeculateIndependentAnalyticGeometry1075();
    bool TriedCellWave1213=!CanTryCellWave1213;
    int32 FirstCanonicalCell1213=INDEX_NONE;
    const int32 CellWindowItems1534=CanTryCellWave1213?FJoinedTensorCellWave1213::WindowCandidateItems1534():0;
    const bool UseCellWindows1534=CanTryCellWave1213&&CellWindowItems1534>0;
    bool StopCellWindows1534=false,RebaseCellWindow1534=false;
    int32 SerialSettleRemaining1534=0;
    const auto PrepareCellWave1213=[&]()
    {
        TArray<int32,TInlineAllocator<64>> Eligible1213;
        {
        FScopedTensorCellEligibilityObservation1213 EligibilityObservation1213;
        const auto EntryNumeric1213=CaptureFrameFloatingPointContext();
        if(EntryNumeric1213.bValid)
        {
            FScopedFrameFloatingPointContext AdmissionNumeric1213(EntryNumeric1213);
            if(AdmissionNumeric1213.IsApplied())
            {
                FBoxBoundsSweepContext AdmissionBox1213;
                if(Query.Shape==EQueryShape::Box)AdmissionBox1213.Initialize(Query,ProviderBoxContext.Axis);
                const double Bound1213=CurrentMaximumSearchTime();
                TArray<int32,TInlineAllocator<64>> AdmissionStack1213;
                AdmissionStack1213.Add(0);
                while(!AdmissionStack1213.IsEmpty() && Eligible1213.Num()<=FJoinedTensorCellWave1213::MaximumProviders)
                {
                    const int32 NodeIndex1213=AdmissionStack1213.Pop(EAllowShrinking::No);
                    if(!Patch.CellBvhNodes.IsValidIndex(NodeIndex1213))continue;
                    const auto& Node1213=Patch.CellBvhNodes[NodeIndex1213];
                    if(!Node1213.Bounds.Intersect(QueryBounds) || !SweptShapeIntersectsBounds(Query,
                        Node1213.Bounds,Query.Shape==EQueryShape::Box?&AdmissionBox1213:nullptr,Bound1213))continue;
                    if(!Node1213.IsLeaf())
                    {
                        if(Node1213.RightChild!=INDEX_NONE)AdmissionStack1213.Add(Node1213.RightChild);
                        if(Node1213.LeftChild!=INDEX_NONE)AdmissionStack1213.Add(Node1213.LeftChild);
                        continue;
                    }
                    for(int32 Offset1213=0;Offset1213<Node1213.IndexCount;++Offset1213)
                    {
                        const int32 Permutation1213=Node1213.FirstIndex+Offset1213;
                        if(!Patch.CellBvhIndices.IsValidIndex(Permutation1213))continue;
                        const int32 CellIndex1213=Patch.CellBvhIndices[Permutation1213];
                        if(CellIndex1213==FirstCanonicalCell1213 || !Patch.Cells.IsValidIndex(CellIndex1213))continue;
                        if(SweptShapeIntersectsBounds(Query,Patch.Cells[CellIndex1213].Bounds,
                            Query.Shape==EQueryShape::Box?&AdmissionBox1213:nullptr,Bound1213))
                            Eligible1213.Add(CellIndex1213);
                        if(Eligible1213.Num()>FJoinedTensorCellWave1213::MaximumProviders)break;
                    }
                }
            }
            if(!AdmissionNumeric1213.Restore())
                throw std::runtime_error("IAmSpeed Tensor cell admission numeric restoration failed");
        }
        } // Eligibility scope closes before wave construction or any dispatch.
        const auto CaptureCellBound1213=[&](){return CurrentMaximumSearchTime();};
        const auto EvaluateCell1213=[&](int32 CellIndex1213,double Bound1213)
        {
            // A complete Tensor invocation owns its local Best, stack, SAT cache,
            // causal observer and scratch. Shared provider axes are immutable.
#if !UE_BUILD_SHIPPING
            TGuardValue<bool> Detail1213(GAnalyticQueryDetailedTimingEnabled,false);
#endif
            const auto View1213=MakeCellView1213(Patch.Cells[CellIndex1213]);
            return SweepTensorBezierApproximation(Query,QueryBounds,View1213,Bound1213,
                Query.Shape==EQueryShape::Box?ProviderBoxContext.Axis:nullptr,Selection);
        };
        CellWave1213.Emplace(Query,QueryBounds,CellIdentity1213,
            Eligible1213,Eligible1213.Num(),InTensorCellLanes1213,CaptureCellBound1213,EvaluateCell1213);
    };
	TArray<int32, TInlineAllocator<64>> CellNodeStack;
	CellNodeStack.Add(0);
    const auto PrepareCellWindow1534=[&](const FTriangleBvhNode& CurrentLeaf1534,
        int32 CurrentOffset1534,ETensorCellWindowReason1534 Reason1534)
    {
        // The old wave has already joined. Destroy its unused records before
        // admission; no stale-bound result can reach a later canonical cell.
        CellWave1213.Reset();
        RebaseCellWindow1534=false;
        TArray<int32,TInlineAllocator<64>> Eligible1534;
        {
            FScopedTensorCellEligibilityObservation1213 EligibilityObservation1534;
            const auto EntryNumeric1534=CaptureFrameFloatingPointContext();
            if(EntryNumeric1534.bValid)
            {
                FScopedFrameFloatingPointContext AdmissionNumeric1534(EntryNumeric1534);
                if(AdmissionNumeric1534.IsApplied())
                {
                    FBoxBoundsSweepContext AdmissionBox1534;
                    if(Query.Shape==EQueryShape::Box)AdmissionBox1534.Initialize(Query,ProviderBoxContext.Axis);
                    const double Bound1534=CurrentMaximumSearchTime();
                    const auto AdmitLeaf1534=[&](const FTriangleBvhNode& Leaf1534,int32 BeginOffset1534)
                    {
                        for(int32 Offset1534=BeginOffset1534;Offset1534<Leaf1534.IndexCount&&Eligible1534.Num()<CellWindowItems1534;++Offset1534)
                        {
                            const int32 Permutation1534=Leaf1534.FirstIndex+Offset1534;
                            if(!Patch.CellBvhIndices.IsValidIndex(Permutation1534))continue;
                            const int32 CellIndex1534=Patch.CellBvhIndices[Permutation1534];
                            if(!Patch.Cells.IsValidIndex(CellIndex1534))continue;
                            if(SweptShapeIntersectsBounds(Query,Patch.Cells[CellIndex1534].Bounds,
                                Query.Shape==EQueryShape::Box?&AdmissionBox1534:nullptr,Bound1534))
                                Eligible1534.Add(CellIndex1534);
                        }
                    };
                    // CurrentLeaf is already popped by the original owner.
                    // Probe only its unconsumed suffix, then a private copy of
                    // the owner's remaining stack, with identical L/R order.
                    AdmitLeaf1534(CurrentLeaf1534,CurrentOffset1534);
                    TArray<int32,TInlineAllocator<64>> AdmissionStack1534=CellNodeStack;
                    while(!AdmissionStack1534.IsEmpty()&&Eligible1534.Num()<CellWindowItems1534)
                    {
                        const int32 NodeIndex1534=AdmissionStack1534.Pop(EAllowShrinking::No);
                        if(!Patch.CellBvhNodes.IsValidIndex(NodeIndex1534))continue;
                        const auto& Node1534=Patch.CellBvhNodes[NodeIndex1534];
                        if(!Node1534.Bounds.Intersect(QueryBounds)||!SweptShapeIntersectsBounds(Query,
                            Node1534.Bounds,Query.Shape==EQueryShape::Box?&AdmissionBox1534:nullptr,Bound1534))continue;
                        if(Node1534.IsLeaf())AdmitLeaf1534(Node1534,0);
                        else
                        {
                            if(Node1534.RightChild!=INDEX_NONE)AdmissionStack1534.Add(Node1534.RightChild);
                            if(Node1534.LeftChild!=INDEX_NONE)AdmissionStack1534.Add(Node1534.LeftChild);
                        }
                    }
                }
                if(!AdmissionNumeric1534.Restore())
                    throw std::runtime_error("IAmSpeed Tensor cell window admission numeric restoration failed");
            }
        } // Restore the exact owner flags before the existing wave constructor.
        const auto CaptureCellBound1534=[&](){return CurrentMaximumSearchTime();};
        const auto EvaluateCell1534=[&](int32 CellIndex1534,double Bound1534)
        {
#if !UE_BUILD_SHIPPING
            TGuardValue<bool> Detail1534(GAnalyticQueryDetailedTimingEnabled,false);
#endif
            const auto View1534=MakeCellView1213(Patch.Cells[CellIndex1534]);
            return SweepTensorBezierApproximation(Query,QueryBounds,View1534,Bound1534,
                Query.Shape==EQueryShape::Box?ProviderBoxContext.Axis:nullptr,Selection);
        };
        CellWave1213.Emplace(Query,QueryBounds,CellIdentity1213,Eligible1534,Eligible1534.Num(),
            InTensorCellLanes1213,CaptureCellBound1534,EvaluateCell1534,Reason1534);
        // A small tail or any policy/numeric/executor refusal stays serial for
        // the remainder of this invocation; never repeat a suffix probe per cell.
        if(!CellWave1213->IsReady())StopCellWindows1534=true;
    };
	while (!CellNodeStack.IsEmpty())
	{
#if !UE_BUILD_SHIPPING
		if (GAnalyticQueryDetailedTimingEnabled) ++ProviderNodeVisits;
#endif
		const int32 NodeIndex = CellNodeStack.Pop(EAllowShrinking::No);
		if (!Patch.CellBvhNodes.IsValidIndex(NodeIndex)) continue;
		const FTriangleBvhNode& Node = Patch.CellBvhNodes[NodeIndex];
		if (!Node.Bounds.Intersect(QueryBounds) ||
			!SweptShapeIntersectsBounds(Query, Node.Bounds,
				Query.Shape == EQueryShape::Box ? &ProviderBoxContext : nullptr,
				CurrentMaximumSearchTime()))
		{
			continue;
		}
		if (!Node.IsLeaf())
		{
			if (Node.RightChild != INDEX_NONE) CellNodeStack.Add(Node.RightChild);
			if (Node.LeftChild != INDEX_NONE) CellNodeStack.Add(Node.LeftChild);
			continue;
		}
		for (int32 Offset = 0; Offset < Node.IndexCount; ++Offset)
		{
#if !UE_BUILD_SHIPPING
			if (GAnalyticQueryDetailedTimingEnabled) ++ProviderCellVisits;
#endif
			const int32 PermutationIndex = Node.FirstIndex + Offset;
			if (!Patch.CellBvhIndices.IsValidIndex(PermutationIndex)) continue;
			const FPiecewiseTensorBezierCell& Cell =
				Patch.Cells[Patch.CellBvhIndices[PermutationIndex]];
			if (!SweptShapeIntersectsBounds(Query, Cell.Bounds,
				Query.Shape == EQueryShape::Box ? &ProviderBoxContext : nullptr,
				CurrentMaximumSearchTime()))
			{
				continue;
			}
            Causal.AddItem(); // Count canonical coarse-cell kernel consumptions, including validated reuse.
            const auto CellView=MakeCellView1213(Cell);
            if(!TriedCellWave1213)
            {
                if(FirstCanonicalCell1213==INDEX_NONE)
                    FirstCanonicalCell1213=Patch.CellBvhIndices[PermutationIndex];
                else
                {
                    // First actual cell keeps its original serial kernel/fold.
                    // Capture the learned TOI and sticky FP state only here,
                    // before the second real kernel. Never speculate the prefix.
                    TriedCellWave1213=true;
                    if(UseCellWindows1534)PrepareCellWindow1534(Node,Offset,ETensorCellWindowReason1534::First);
                    else PrepareCellWave1213();
                }
            }
            if(UseCellWindows1534&&TriedCellWave1213&&FirstCanonicalCell1213!=INDEX_NONE&&!StopCellWindows1534)
            {
                if(RebaseCellWindow1534&&CellWave1213.IsSet())
                {
                    const auto CaptureCurrentBound1534=[&](){return CurrentMaximumSearchTime();};
                    if(CellWave1213->HasSameBound1534(CaptureCurrentBound1534))
                    {
                        // A TOI bit change need not change the clamped bound.
                        // Keep the joined wave only for an exact guarded match.
                        RebaseCellWindow1534=false;SerialSettleRemaining1534=0;
                    }
                    else CellWave1213.Reset();
                }
                // A changing TOI can otherwise speculatively evaluate the same
                // future window once per improving cell. A count-based serial
                // settling prefix prevents that retry amplification. Further
                // canonical TOI changes restart this prefix after their fold.
                if(SerialSettleRemaining1534>0)--SerialSettleRemaining1534;
                else if(!CellWave1213.IsSet()||!CellWave1213->ContainsProvider1534(Patch.CellBvhIndices[PermutationIndex]))
                    PrepareCellWindow1534(Node,Offset,RebaseCellWindow1534?
                        ETensorCellWindowReason1534::BoundChanged:ETensorCellWindowReason1534::Advance);
            }
            const FWorldHit Candidate = [&]()
            {
                FScopedAnalyticCycles1075 Numeric(Causal.NumericTarget(0));
                if(CellWave1213.IsSet())
                {
                    const double Bound1213=CurrentMaximumSearchTime();
                    FWorldHit Joined1213;
                    const int32 CellIndex1213=Patch.CellBvhIndices[PermutationIndex];
                    if(CellWave1213->TryConsume(CellIndex1213,Query,QueryBounds,
                        CellIdentity1213,Bound1213,Joined1213))return Joined1213;
                    return SweepTensorBezierApproximation(Query,QueryBounds,CellView,Bound1213,
                        Query.Shape==EQueryShape::Box?ProviderBoxContext.Axis:nullptr,Selection);
                }
                return SweepTensorBezierApproximation(
                    Query, QueryBounds, CellView, CurrentMaximumSearchTime(),
                    Query.Shape == EQueryShape::Box ? ProviderBoxContext.Axis : nullptr, Selection);
            }();
            {
                FScopedAnalyticCycles1075 Fold(Causal.FoldTarget());
                if (Candidate.bHit && HitPassesReferenceNormal(Query, Candidate) &&
                    IsBetterHit(Candidate, Best, Selection))
                {
                    // Integer/bit comparison adds no FP operations or sticky
                    // flags. The next actual canonical cell captures its new
                    // bound inside the restored guard, exactly as before.
                    if(UseCellWindows1534&&TriedCellWave1213&&!StopCellWindows1534&&
                        (!Best.bHit||FMemory::Memcmp(&Candidate.Time,&Best.Time,sizeof(Candidate.Time))!=0))
                    {
                        RebaseCellWindow1534=true;
                        SerialSettleRemaining1534=CellWindowItems1534;
                    }
                    Best = Candidate;
                }
            }
		}
	}
#if !UE_BUILD_SHIPPING
	if (GAnalyticQueryDetailedTimingEnabled &&
		GAnalyticPiecewiseSlowQueryLogs < 128)
	{
		const double ElapsedMicroseconds =
			(FPlatformTime::Seconds() - DetailedStartSeconds) * 1.0e6;
		if (ElapsedMicroseconds >= 10.0)
		{
			++GAnalyticPiecewiseSlowQueryLogs;
			UE_LOG(LogTemp, Display,
				TEXT("[AnalyticPiecewiseSlowQuery] Surface=%016llX Primitive=%016llX ")
				TEXT("Shape=%u ElapsedUs=%.6f ProviderNodes=%llu ProviderCells=%llu ")
				TEXT("ApproximationNodes=%llu ApproximationCells=%llu Start=%s End=%s"),
				Patch.SurfaceId, Patch.PrimitiveId,
				static_cast<uint8>(Query.Shape), ElapsedMicroseconds,
				static_cast<unsigned long long>(ProviderNodeVisits),
				static_cast<unsigned long long>(ProviderCellVisits),
				static_cast<unsigned long long>(GAnalyticApproximationNodeVisits -
					ApproximationNodeVisitsBefore),
				static_cast<unsigned long long>(GAnalyticApproximationCellVisits -
					ApproximationCellVisitsBefore),
				*FVector(Query.Start).ToString(), *FVector(Query.End).ToString());
		}
	}
#endif
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
	if (Query.Shape == EQueryShape::Box)
	{
		FPlaneTriangle PlaneTriangle;
		for (int32 Corner = 0; Corner < 3; ++Corner)
		{
			PlaneTriangle.Vertex[Corner] = Triangle.Vertices[Corner];
			PlaneTriangle.PolygonVertex[Corner] = static_cast<int8>(Corner);
		}
		if (!SweepBoxTriangle(Query, PlaneTriangle, Hit)) return FWorldHit();
		Hit.SourceId = Triangle.SourceId;
		Hit.SurfaceId = Triangle.SurfaceId;
		Hit.FeatureId = Triangle.FeatureId;
		Hit.PrimitiveId = Triangle.PrimitiveId;
		Hit.MaterialId = Triangle.MaterialId;
		return Hit;
	}
	if (Query.Shape == EQueryShape::Sphere)
	{
		if (!SweepSphereTriangleGeometry(
			Query, Triangle.Vertices, Triangle.FaceNormal, Hit))
		{
			return FWorldHit();
		}
		Hit.SourceId = Triangle.SourceId;
		Hit.SurfaceId = Triangle.SurfaceId;
		Hit.FeatureId = Triangle.FeatureId;
		Hit.PrimitiveId = Triangle.PrimitiveId;
		Hit.MaterialId = Triangle.MaterialId;
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
	const FWorldHit& Candidate, const FWorldHit& Best, const EHitSelection Selection)
{
	if (Selection == EHitSelection::DeepestOverlap)
	{
		if (!Candidate.bHit || !Candidate.bStartPenetrating ||
			!FMath::IsFinite(Candidate.PenetrationDepth) || Candidate.PenetrationDepth <= 0)
			return false;
		if (!Best.bHit || Candidate.PenetrationDepth > Best.PenetrationDepth) return true;
		if (Candidate.PenetrationDepth < Best.PenetrationDepth) return false;
		// Exact depth comparison avoids order-dependent epsilon chains. Witness
		// distance and approximation error are not extra physical penetration.
		if (Candidate.SourceId != Best.SourceId) return Candidate.SourceId < Best.SourceId;
		if (Candidate.SurfaceId != Best.SurfaceId) return Candidate.SurfaceId < Best.SurfaceId;
		if (Candidate.FeatureId != Best.FeatureId) return Candidate.FeatureId < Best.FeatureId;
		return Candidate.PrimitiveId < Best.PrimitiveId;
	}
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

bool FWorldQueryService::HitPassesReferenceNormal(
	const FWorldQuery& Query, const FWorldHit& Hit)
{
	if (Query.MinimumReferenceNormalDot <= -1.0 ||
		Query.ReferenceNormal.IsNearlyZero())
	{
		return true;
	}
	const FVector3d ReferenceNormal = Query.ReferenceNormal.GetSafeNormal();
	const FVector3d HitNormal = Hit.Normal.GetSafeNormal();
	return !HitNormal.IsNearlyZero() &&
		FVector3d::DotProduct(ReferenceNormal, HitNormal) >=
			Query.MinimumReferenceNormalDot;
}

bool FWorldQueryService::IsSameQuery(
	const FWorldQuery& A, const FWorldQuery& B)
{
	return A.Shape == B.Shape &&
		A.Start == B.Start &&
		A.End == B.End &&
		A.Rotation == B.Rotation &&
		A.Radius == B.Radius &&
		A.HalfExtent == B.HalfExtent &&
		A.DomainTolerance == B.DomainTolerance &&
		A.InitialOverlapTolerance == B.InitialOverlapTolerance &&
		A.TraceChannel == B.TraceChannel &&
		A.ObjectTypes == B.ObjectTypes &&
		A.BlockingObjectTypes == B.BlockingObjectTypes &&
		A.RequiredSourceId == B.RequiredSourceId &&
		A.RequiredSurfaceId == B.RequiredSurfaceId &&
		A.RequiredCanonicalGroupId == B.RequiredCanonicalGroupId &&
		A.ReferenceNormal == B.ReferenceNormal &&
		A.MinimumReferenceNormalDot == B.MinimumReferenceNormalDot &&
		A.bAllowEstablishedFaceContactAtBoundary ==
			B.bAllowEstablishedFaceContactAtBoundary &&
		A.bUseFiniteContactDomain == B.bUseFiniteContactDomain &&
		A.bObjectQuery == B.bObjectQuery &&
		A.bApplyCollisionFilter == B.bApplyCollisionFilter &&
		A.bAuthorityOnly == B.bAuthorityOnly &&
		A.bExcludeAuthorityEligible == B.bExcludeAuthorityEligible && A.bExcludeTori == B.bExcludeTori &&
		A.bIncludeCompactPatches == B.bIncludeCompactPatches &&
		A.bIncludeTriangles == B.bIncludeTriangles;
}

bool FWorldQueryService::HasAuthorityCoverage(const FWorldQuery& Query) const
{
	FWorldHit AuthorityHit;
	return TrySweepAuthority(Query, AuthorityHit);
}

bool FWorldQueryService::TrySweepAuthority(
	const FWorldQuery& Query, FWorldHit& OutAuthorityHit) const
{
	TRACE_CPUPROFILER_EVENT_SCOPE(IAmSpeed_AnalyticTrySweepAuthority);
	static thread_local bool bLoggedFirstCoverageMismatch = false;
	// AnalyticHybrid is a migration tool. The certified primitive must produce
	// the winning hit not only inside its own broad phase, but also against every
	// known draft provider. This keeps an underlying certified surface from
	// replacing a nearer, not-yet-certified authored surface. SurfaceAnalytic
	// instead requires a complete-world certificate and treats a miss as final.
	FWorldQuery AuthorityQuery = Query;
	AuthorityQuery.bAuthorityOnly = true;
	AuthorityQuery.bUseFiniteContactDomain = false;
	FWorldHit AuthorityTriangle;
	const FWorldHit AuthorityHit = SweepDetailed(
		AuthorityQuery, &AuthorityTriangle);
	OutAuthorityHit = AuthorityHit;
	if (!AuthorityHit.bHit)
	{
		// A miss is authoritative when every source touched by the query either
		// has a closed residual triangle certificate or is outside the swept
		// broad phase. This is especially important in Hybrid: an empty-space
		// query must not pay for both the analytical world and an Unreal sweep.
		if (IsAuthorityMissDefinitive(Query, SweptQueryBounds(Query)))
		{
			return true;
		}
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
	ProviderQuery.bUseFiniteContactDomain = false;
	ProviderQuery.bExcludeAuthorityEligible = true;
	ProviderQuery.bIncludeCompactPatches = true;
	// SweepDetailed above has already visited every authority-eligible residual
	// triangle and retained AuthorityTriangle.  A second BVH walk can contribute
	// only non-authority triangles.  Skip it when the query bounds are covered
	// exclusively by closed residual triangle sources; this is a proof-based
	// Hybrid fast path, not an approximation or a change in hit precedence.
	ProviderQuery.bIncludeTriangles = !IsAuthorityMissDefinitive(
		Query, SweptQueryBounds(Query));
	FWorldHit ProviderHit = Sweep(ProviderQuery);
	if (AuthorityTriangle.bHit && IsBetterHit(AuthorityTriangle, ProviderHit))
	{
		ProviderHit = AuthorityTriangle;
	}
	constexpr double ProviderTimeTolerance = 1.0e-9;
	const bool bCovered = !ProviderHit.bHit ||
		(ProviderHit.SourceId == AuthorityHit.SourceId
			? AuthorityHit.Time <= ProviderHit.Time + ProviderTimeTolerance
			: !IsBetterHit(ProviderHit, AuthorityHit));
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

#if !UE_BUILD_SHIPPING
void FWorldQueryService::RecordCacheProfile(
	const EQueryShape Shape, const int32 HitWay, const bool bEvicted) const
{
	const uint32 ShapeIndex = static_cast<uint32>(Shape);
	if (ShapeIndex >= UE_ARRAY_COUNT(CacheProfile.Hits)) return;
	++CacheProfile.Queries;
	if (HitWay != INDEX_NONE)
	{
		++CacheProfile.Hits[ShapeIndex];
		++CacheProfile.HitWays[HitWay];
	}
	else
	{
		++CacheProfile.Misses[ShapeIndex];
		CacheProfile.Evictions += bEvicted ? 1 : 0;
	}
	if (CacheProfile.Queries < 1000) return;
	UE_LOG(LogTemp, Display,
		TEXT("[AnalyticQueryCacheProfile] Queries=%llu RayHits=%llu RayMisses=%llu ")
		TEXT("SphereHits=%llu SphereMisses=%llu BoxHits=%llu BoxMisses=%llu ")
		TEXT("Way0=%llu Way1=%llu Way2=%llu Way3=%llu Evictions=%llu"),
		CacheProfile.Queries, CacheProfile.Hits[0], CacheProfile.Misses[0],
		CacheProfile.Hits[1], CacheProfile.Misses[1], CacheProfile.Hits[2], CacheProfile.Misses[2],
		CacheProfile.HitWays[0], CacheProfile.HitWays[1], CacheProfile.HitWays[2], CacheProfile.HitWays[3],
		CacheProfile.Evictions);
	CacheProfile = FCacheProfile{};
}
#endif

FWorldHit FWorldQueryService::Sweep(const FWorldQuery& Query) const
{
	return QueryCached(Query, EHitSelection::FirstSweepHit);
}

bool FWorldQueryService::TryFindDeepestOverlap(const FWorldQuery& Query, FWorldHit& OutHit) const
{
	OutHit = FWorldHit();
	if (Query.Start != Query.End || Query.Start.ContainsNaN() ||
		Query.Rotation.ContainsNaN() || !Query.Rotation.IsNormalized() ||
		!FMath::IsFinite(Query.DomainTolerance) || Query.DomainTolerance < 0 ||
		(Query.Shape != EQueryShape::Sphere && Query.Shape != EQueryShape::Box)) return false;
	if (Query.Shape == EQueryShape::Sphere && (!FMath::IsFinite(Query.Radius) || Query.Radius <= 0)) return false;
	if (Query.Shape == EQueryShape::Box && (Query.HalfExtent.ContainsNaN() || Query.HalfExtent.GetMin() <= 0)) return false;
	FWorldQuery PoseQuery = Query;
	PoseQuery.bUseFiniteContactDomain = true;
	// This is a geometric observation/correction, not CCD contact release.
	PoseQuery.InitialOverlapTolerance = 0.0;
	OutHit = QueryCached(PoseQuery, EHitSelection::DeepestOverlap);
	return true;
}

FWorldHit FWorldQueryService::QueryCached(const FWorldQuery& Query, const EHitSelection Selection) const
{
    FScopedAnalyticCausalObservation1075 Causal(EAnalyticCausalKind1075::WorldSweep);
    Causal.SetItems(1);
	// Impossible filtered queries must not traverse providers or evict useful
	// cached support probes. Dynamic sub-body overlaps are a separate caller path.
	if (Query.RejectsAllCollision()) return FWorldHit();
#if !UE_BUILD_SHIPPING
	const bool bProfileCache = CVarIAmSpeedAnalyticWorldCacheProfile.GetValueOnAnyThread() != 0;
	// Do not mix partial windows across a disabled interval in an interactive run.
	if (!bProfileCache && CacheProfile.Queries != 0) CacheProfile = FCacheProfile{};
#endif
	// Slot collisions only cause recomputation; full query equality decides reuse.
	uint32 BucketHash = HashCombineFast(GetTypeHash(Query.Start), GetTypeHash(Query.End));
#if !UE_BUILD_SHIPPING
	if (CVarIAmSpeedAnalyticWorldCacheBucketMode.GetValueOnAnyThread() == 1)
	{
		// Different probes at the same pose otherwise all compete for four ways.
		// This changes placement only: IsSameQuery and Selection still decide
		// reuse, and every miss executes the unchanged physical kernel.
		BucketHash = HashCombineFast(BucketHash, GetTypeHash(static_cast<uint8>(Query.Shape)));
		BucketHash = HashCombineFast(BucketHash, GetTypeHash(static_cast<uint8>(Selection)));
		BucketHash = HashCombineFast(BucketHash, GetTypeHash(Query.Radius));
		BucketHash = HashCombineFast(BucketHash, GetTypeHash(Query.HalfExtent));
		BucketHash = HashCombineFast(BucketHash, GetTypeHash(Query.RequiredSourceId));
		BucketHash = HashCombineFast(BucketHash, GetTypeHash(Query.RequiredSurfaceId));
		BucketHash = HashCombineFast(BucketHash, GetTypeHash(Query.RequiredCanonicalGroupId));
		BucketHash = HashCombineFast(BucketHash, GetTypeHash(Query.ObjectTypes));
		BucketHash = HashCombineFast(BucketHash, GetTypeHash(Query.BlockingObjectTypes));
		BucketHash = HashCombineFast(BucketHash, GetTypeHash(Query.TraceChannel));
		BucketHash = HashCombineFast(BucketHash, GetTypeHash(Query.InitialOverlapTolerance));
		const uint32 Policy = uint32(Query.bUseFiniteContactDomain) |
			(uint32(Query.bAuthorityOnly) << 1) | (uint32(Query.bExcludeAuthorityEligible) << 2) |
			(uint32(Query.bApplyCollisionFilter) << 3) | (uint32(Query.bObjectQuery) << 4) |
			(uint32(Query.bIncludeCompactPatches) << 5) | (uint32(Query.bIncludeTriangles) << 6) |
			(uint32(Query.bAllowEstablishedFaceContactAtBoundary) << 7);
		BucketHash = HashCombineFast(BucketHash, Policy);
	}
#endif
	const uint32 Bucket = BucketHash & (SweepCacheCapacity / SweepCacheWays - 1);
	const uint32 First = Bucket * SweepCacheWays;
	for (uint32 Way = 0; Way < SweepCacheWays; ++Way)
	{
		const FCachedSweep& Cached = SweepCache[First + Way];
		if (Cached.bValid && Cached.Selection == Selection && IsSameQuery(Query, Cached.Query))
		{
#if !UE_BUILD_SHIPPING
			if (bProfileCache) RecordCacheProfile(Query.Shape, Way, false);
#endif
			return Cached.Hit;
		}
	}
	FCachedSweep& Entry = SweepCache[First + NextCacheWay[Bucket]];
#if !UE_BUILD_SHIPPING
	if (bProfileCache) RecordCacheProfile(Query.Shape, INDEX_NONE, Entry.bValid);
#endif
	NextCacheWay[Bucket] = (NextCacheWay[Bucket] + 1) & (SweepCacheWays - 1);
	Entry.Hit = SweepDetailed(Query, nullptr, Selection);
	Entry.Query = Query;
	Entry.Selection = Selection;
	Entry.bValid = true;
	return Entry.Hit;
}

FWorldHit FWorldQueryService::SweepDetailed(
	const FWorldQuery& Query, FWorldHit* OutBestTriangle, const EHitSelection Selection) const
{
    FScopedAnalyticCausalObservation1075 Causal(EAnalyticCausalKind1075::DetailedWorldQuery);
    Causal.SetItems(1);
#if !UE_BUILD_SHIPPING
#if SL_ANALYTIC_QUERY_FINE_CPU_SCOPES659
    CaseSatDiagnostics::FScopedCpuPhase400 Phase400(CaseSatDiagnostics::ECpuPhase400::SweepDetailed);
#endif
#endif
	if (Query.RejectsAllCollision())
	{
		if (OutBestTriangle) *OutBestTriangle = FWorldHit();
		return FWorldHit();
	}
	TRACE_CPUPROFILER_EVENT_SCOPE(IAmSpeed_AnalyticSweepDetailed);
#if !UE_BUILD_SHIPPING
	const int32 PhaseTimingMode =
		CVarIAmSpeedAnalyticWorldPhaseTiming.GetValueOnAnyThread();
	const bool bPhaseTimingEnabled = PhaseTimingMode != 0;
	GAnalyticQueryDetailedTimingEnabled = PhaseTimingMode >= 2;
	const double TotalStartSeconds = bPhaseTimingEnabled
		? FPlatformTime::Seconds()
		: 0.0;
#endif
	FWorldHit Best;
	TArray<FWorldHit, TInlineAllocator<128>> AuthorityProviderHits;
	const bool bCollectAuthorityProviderHits =
		Query.bAuthorityOnly && Query.bIncludeTriangles && Selection == EHitSelection::FirstSweepHit;
	const auto ConsiderProviderHit = [&](const FWorldHit& Candidate)
	{
		if (!Candidate.bHit || !HitPassesReferenceNormal(Query, Candidate)) return;
		if (bCollectAuthorityProviderHits) AuthorityProviderHits.Add(Candidate);
		if (IsBetterHit(Candidate, Best, Selection)) Best = Candidate;
	};
	const auto CurrentMaximumSearchTime = [&Best, bCollectAuthorityProviderHits]()
	{
		// Hybrid authority arbitration must retain every polished provider that
		// can agree with the residual triangle, including one geometrically
		// behind an earlier out-of-domain provider. Strict provider-only queries
		// have no such residual arbitration and may tighten monotonically.
		if (bCollectAuthorityProviderHits) return 1.0;
		return Best.bHit ? FMath::Min(1.0, Best.Time + 1.0e-12) : 1.0;
	};
	const FBox3d QueryBounds = SweptQueryBounds(Query);
    if (!Query.bExcludeTori)
        for (const auto& Torus:World.Tori)
            if (TorusPassesFilter(Query,Torus) && Torus.Bounds.Intersect(QueryBounds))
                ConsiderProviderHit(SweepTorusSurface(Query,Torus));

#if !UE_BUILD_SHIPPING
	// Optional storage starts empty. First admitted OBB provider derives it from
	// this immutable Query; it cannot survive this SweepDetailed invocation.
	TOptional<FBoxBoundsSweepContext> ExtrudedBoundsContext435;
#endif
#if !UE_BUILD_SHIPPING
	const double PlaneStartSeconds = bPhaseTimingEnabled
		? FPlatformTime::Seconds()
		: 0.0;
#endif
	Speed::Collision::FOrderedBoundsScratch PlaneScratch;
	for (int32 PlaneIndex : PlaneProviderIndex->FindCandidates(QueryBounds, PlaneScratch))
	{
		const FBoundedPlane& Plane = World.Planes[PlaneIndex];
		if (Plane.bRequiresCompactOptIn) continue;
		if (Query.bAuthorityOnly && !Plane.bAuthorityEligible) continue;
		if (Query.bExcludeAuthorityEligible && Plane.bAuthorityEligible) continue;
		if (!PlanePassesFilter(Query, Plane)) continue;
		// A certified polygon domain remains authoritative even when an older
		// serialized asset lacks a valid acceleration box; the exact projected
		// domain test below is the source of truth in that case.
		if (Plane.Bounds.IsValid && !Plane.Bounds.Intersect(QueryBounds)) continue;
		const FWorldHit Candidate = SweepPlane(Query, Plane, &World, nullptr, Selection);
		if (!Candidate.bHit)
		{
			continue;
		}
		ConsiderProviderHit(Candidate);
	}
#if !UE_BUILD_SHIPPING
	const double PlaneSeconds = bPhaseTimingEnabled
		? FPlatformTime::Seconds() - PlaneStartSeconds
		: 0.0;
	const double CompactStartSeconds = bPhaseTimingEnabled
		? FPlatformTime::Seconds()
		: 0.0;
#endif
	if (Query.bIncludeCompactPatches && !World.CompactBvh.IsEmpty())
	{
		TRACE_CPUPROFILER_EVENT_SCOPE(IAmSpeed_AnalyticCompactBvh);
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
					if (Query.bExcludeAuthorityEligible && Plane.bAuthorityEligible) continue;
					if (!PlanePassesFilter(Query, Plane) ||
						(Plane.Bounds.IsValid && !Plane.Bounds.Intersect(QueryBounds))) continue;
					Candidate = SweepPlane(Query, Plane, &World, nullptr, Selection);
				}
				else
				{
					const FExtrudedQuinticPatch& Patch =
						World.ExtrudedQuinticPatches[
							EncodedIndex - World.Planes.Num()];
					if (Query.bAuthorityOnly && !Patch.bAuthorityEligible) continue;
					if (Query.bExcludeAuthorityEligible && Patch.bAuthorityEligible) continue;
					// Reject distant/filtered providers before entering the kernel and
					// constructing its miss result. These are the kernel's exact guards;
					// the leaf and provider arbitration order remain unchanged.
					if (!Patch.Bounds.Intersect(QueryBounds) || !PatchPassesFilter(Query, Patch)) continue;
					FBoxBoundsSweepContext* SharedBoundsContext435 = nullptr;
#if !UE_BUILD_SHIPPING
					if (Query.Shape == EQueryShape::Box &&
						CVarIAmSpeedAnalyticWorldExtrudedBoxBoundsQueryReuse435.GetValueOnAnyThread() != 0)
					{
						if (!ExtrudedBoundsContext435.IsSet()) ExtrudedBoundsContext435.Emplace(Query);
						SharedBoundsContext435 = &ExtrudedBoundsContext435.GetValue();
					}
#endif
					Speed::FIndependentQueryBatchStats FacetDispatch;
                    int32 EvaluatedFacets = 0;
                    Candidate = SweepExtrudedQuintic(Query, QueryBounds, Patch, Selection,
                        SharedBoundsContext435, FacetLanes, MinimumFacetItems, &FacetDispatch, &EvaluatedFacets);
                    FacetParallelTotals.Add(FacetDispatch, EvaluatedFacets);
				}
				ConsiderProviderHit(Candidate);
			}
		}
	}
#if !UE_BUILD_SHIPPING
	const double CompactSeconds = bPhaseTimingEnabled
		? FPlatformTime::Seconds() - CompactStartSeconds
		: 0.0;
	double TensorSeconds = 0.0;
	double PiecewiseSeconds = 0.0;
#endif
	if (Query.bIncludeCompactPatches)
	{
		TRACE_CPUPROFILER_EVENT_SCOPE(IAmSpeed_AnalyticTensorProviders);
#if !UE_BUILD_SHIPPING
		const double TensorStartSeconds = bPhaseTimingEnabled
			? FPlatformTime::Seconds()
			: 0.0;
#endif
		for (const FTensorBezierPatch& Patch : World.TensorBezierPatches)
		{
			if (Query.bAuthorityOnly && !Patch.bAuthorityEligible) continue;
			if (Query.bExcludeAuthorityEligible && Patch.bAuthorityEligible) continue;
			const FWorldHit Candidate = SweepTensorBezier(
				Query, QueryBounds, Patch, CurrentMaximumSearchTime(), Selection);
			ConsiderProviderHit(Candidate);
		}
#if !UE_BUILD_SHIPPING
		if (bPhaseTimingEnabled)
		{
			TensorSeconds = FPlatformTime::Seconds() - TensorStartSeconds;
		}
		const double PiecewiseStartSeconds = bPhaseTimingEnabled
			? FPlatformTime::Seconds()
			: 0.0;
#endif
		Speed::Collision::FOrderedBoundsScratch ProviderScratch;
        const auto& ProviderCandidates1075=PiecewiseProviderIndex->FindCandidates(QueryBounds,ProviderScratch);
        const FPiecewiseProviderWaveIdentity1075 ProviderIdentity1075{
            &World,PiecewiseProviderIndex.Get(),&ProviderScratch,uint8(Selection)};
        TOptional<FJoinedPiecewiseProviderWave1075> ProviderWave1075;
        if(IsPiecewiseProviderWave1075Enabled())
        {
            TArray<int32,TInlineAllocator<16>> Eligible1075;
            if(ProviderCandidates1075.Num()<=FJoinedPiecewiseProviderWave1075::MaximumProviders &&
                FacetLanes>1 && CanSpeculateIndependentAnalyticGeometry1075())
            {
                const bool AdmissionBounds1146=CVarPiecewiseAdmissionBounds1146.GetValueOnAnyThread()!=0;
                if(!AdmissionBounds1146)
                {
                    for(const int32 Index1075:ProviderCandidates1075)
                    {
                        const auto& Patch1075=World.PiecewiseTensorBezierPatches[Index1075];
                        if(Query.bAuthorityOnly&&!Patch1075.bAuthorityEligible)continue;
                        if(Query.bExcludeAuthorityEligible&&Patch1075.bAuthorityEligible)continue;
                        if(!Patch1075.Bounds.Intersect(QueryBounds))continue;
                        Eligible1075.Add(Index1075);
                    }
                }
                else
                {
                    // This branch chooses speculative work only. Every provider
                    // still executes or consumes at its original owner-loop point.
                    // Extra geometric tests leave no FP status/control behind.
                    const auto Previous1146=CaptureFrameFloatingPointContext();
                    if(Previous1146.bValid)
                    {
                        FScopedFrameFloatingPointContext Numeric1146(Previous1146);
                        if(Numeric1146.IsApplied())
                        {
                            FBoxBoundsSweepContext AdmissionBox1146;
                            if(Query.Shape==EQueryShape::Box)AdmissionBox1146.Initialize(Query);
                            const double Bound1146=CurrentMaximumSearchTime();
                            for(const int32 Index1146:ProviderCandidates1075)
                            {
                                const auto& Patch1146=World.PiecewiseTensorBezierPatches[Index1146];
                                if(!PatchPassesFilter(Query,Patch1146)||!Patch1146.bApproximationCertified||
                                    !Patch1146.Bounds.Intersect(QueryBounds))continue;
                                if(!SweptShapeIntersectsBounds(Query,Patch1146.Bounds,
                                    Query.Shape==EQueryShape::Box?&AdmissionBox1146:nullptr,Bound1146))continue;
                                Eligible1075.Add(Index1146);
                            }
                        }
                        if(!Numeric1146.Restore())
                            throw std::runtime_error("IAmSpeed admission1146 numeric restoration failed");
                    }
                }
            }
            const auto Bound1075=[&](){return CurrentMaximumSearchTime();};
            const auto Evaluate1075=[&](int32 Index1075,double Maximum1075)
            {
#if !UE_BUILD_SHIPPING
                // A previous throwing query may leave this worker-local diagnostic
                // flag set. Speculation has no canonical query diagnostic identity.
                TGuardValue<bool> Detail1075(GAnalyticQueryDetailedTimingEnabled,false);
#endif
                return SweepPiecewiseTensorBezier(Query,QueryBounds,
                    World.PiecewiseTensorBezierPatches[Index1075],Maximum1075,Selection,FacetLanes);
            };
            ProviderWave1075.Emplace(Query,QueryBounds,ProviderIdentity1075,
                Eligible1075,ProviderCandidates1075.Num(),FacetLanes,Bound1075,Evaluate1075);
        }
		for (const int32 ProviderIndex : ProviderCandidates1075)
		{
			const FPiecewiseTensorBezierPatch& Patch = World.PiecewiseTensorBezierPatches[ProviderIndex];
			if (Query.bAuthorityOnly && !Patch.bAuthorityEligible) continue;
			if (Query.bExcludeAuthorityEligible && Patch.bAuthorityEligible) continue;
			// Reject distant providers before constructing a result and entering
			// their instrumented traversal. This is the same conservative bound
			// checked by SweepPiecewiseTensorBezier; candidate order is unchanged.
			if (!Patch.Bounds.Intersect(QueryBounds)) continue;
            const FWorldHit Candidate = [&]()
            {
                if(ProviderWave1075.IsSet())
                {
                    const double Bound1075=CurrentMaximumSearchTime();
                    FWorldHit Joined1075;
                    if(ProviderWave1075->TryConsume(ProviderIndex,Query,QueryBounds,
                        ProviderIdentity1075,Bound1075,Joined1075))return Joined1075;
                    // A tightened bound or changed context re-executes this exact
                    // provider at the original current bound, never a wider one.
                    return SweepPiecewiseTensorBezier(Query,QueryBounds,Patch,Bound1075,Selection,FacetLanes);
                }
                return SweepPiecewiseTensorBezier(
                    Query, QueryBounds, Patch, CurrentMaximumSearchTime(), Selection,FacetLanes);
            }();
			ConsiderProviderHit(Candidate);
		}
#if !UE_BUILD_SHIPPING
		if (bPhaseTimingEnabled)
		{
			PiecewiseSeconds = FPlatformTime::Seconds() - PiecewiseStartSeconds;
		}
#endif
	}
	// Certified indexed faces are a residual complete-world provider. A compact
	// primitive remains preferred while both representations agree within the
	// public spatial classification tolerance. Large TOI/depth disagreement is
	// evidence that the compact primitive is outside its certified domain, so
	// the closed finite-face provider wins instead of creating a false wall.
	FWorldHit BestTriangle;
#if !UE_BUILD_SHIPPING
	const double TriangleStartSeconds = bPhaseTimingEnabled
		? FPlatformTime::Seconds()
		: 0.0;
#endif
	if (Query.bIncludeTriangles && !World.TriangleBvh.IsEmpty())
	{
		TRACE_CPUPROFILER_EVENT_SCOPE(IAmSpeed_AnalyticTriangleBvh);
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
				if (Query.bExcludeAuthorityEligible && Triangle.bAuthorityEligible) continue;
				// A leaf bound only proves that at least one of its faces can touch
				// the swept volume.  In dense authored regions (notably the finite
				// partitioned curved meshes), dispatching every sibling to the
				// exact SAT/ray solver dominates the physical frame.  The face bound
				// is derived from the same immutable vertices and the query bound
				// already includes the swept shape extent, so this is an exact broad
				// phase rejection rather than an approximation or authority change.
				if (!Triangle.Bounds.Intersect(QueryBounds)) continue;
				const FWorldHit Candidate = SweepTriangleFace(Query, Triangle);
				if (Candidate.bHit &&
					HitPassesReferenceNormal(Query, Candidate) &&
					IsBetterHit(Candidate, BestTriangle, Selection))
				{
					BestTriangle = Candidate;
				}
			}
		}
	}
#if !UE_BUILD_SHIPPING
	const double TriangleSeconds = bPhaseTimingEnabled
		? FPlatformTime::Seconds() - TriangleStartSeconds
		: 0.0;
	const double ArbitrationStartSeconds = bPhaseTimingEnabled
		? FPlatformTime::Seconds()
		: 0.0;
#endif
	if (OutBestTriangle != nullptr)
	{
		*OutBestTriangle = BestTriangle;
	}
	if (BestTriangle.bHit)
	{
		if (!Best.bHit)
		{
			Best = BestTriangle;
		}
		else if (!Query.bAuthorityOnly || Selection == EHitSelection::DeepestOverlap)
		{
			if (IsBetterHit(BestTriangle, Best, Selection)) Best = BestTriangle;
		}
		else
		{
			constexpr double ResidualProviderAgreementToleranceCm = 10.0;
			const double TravelCm = FVector3d::Distance(Query.Start, Query.End);
			FWorldHit BestAgreeingProvider;
			for (const FWorldHit& ProviderHit : AuthorityProviderHits)
			{
				const double ToiDisagreementCm = TravelCm * FMath::Abs(
					ProviderHit.Time - BestTriangle.Time);
				const double DepthDisagreementCm = FMath::Abs(
					ProviderHit.PenetrationDepth - BestTriangle.PenetrationDepth);
				const double ProviderAgreementToleranceCm =
					ResidualProviderAgreementToleranceCm +
					ProviderHit.AdditionalResidualAgreementAllowanceCm;
				if (ToiDisagreementCm <= ProviderAgreementToleranceCm &&
					DepthDisagreementCm <= ProviderAgreementToleranceCm &&
					IsBetterHit(ProviderHit, BestAgreeingProvider))
				{
					BestAgreeingProvider = ProviderHit;
				}
			}
			Best = BestAgreeingProvider.bHit ? BestAgreeingProvider : BestTriangle;
		}
	}
	if (Best.bStartPenetrating && Query.Shape == EQueryShape::Box &&
		Query.bUseFiniteContactDomain &&
		(Best.Point-Best.QueryPoint-Best.PenetrationDepth*Best.Normal).SquaredLength() >
			FMath::Square(Query.DomainTolerance))
	{
		// A support-map tie corner can overhang a finite face while its clamped
		// surface witness already touches the translated box. Repair only the
		// body's matching witness after WORLD arbitration: IsBetterHit itself
		// uses witness distance, so doing this per provider could change its win.
		const FVector3d Axes[3]={
			Query.Rotation.RotateVector(FVector3d::ForwardVector),
			Query.Rotation.RotateVector(FVector3d::RightVector),
			Query.Rotation.RotateVector(FVector3d::UpVector)};
		EContactFeatureKind BoxKind;
		int8 BoxIndex;
		const FVector3d TranslatedPoint=ClosestPointOnBox(Best.Point,
			Best.Location+Best.PenetrationDepth*Best.Normal,Axes,
			Query.HalfExtent,&BoxKind,&BoxIndex);
		// ClosestPointOnBox clamps to a volume; an interior point is not a
		// contact witness. Keep unresolved pairs intact for further attribution.
		if (BoxKind != EContactFeatureKind::Unknown &&
			(TranslatedPoint-Best.Point).SquaredLength() <= FMath::Square(Query.DomainTolerance))
		{
			Best.QueryPoint=TranslatedPoint-Best.PenetrationDepth*Best.Normal;
			Best.QueryFeatureKind=BoxKind;
			Best.QueryFeatureIndex=BoxIndex;
		}
	}
#if !UE_BUILD_SHIPPING
	if (bPhaseTimingEnabled)
	{
		const double EndSeconds = FPlatformTime::Seconds();
		RecordAnalyticQueryPhaseTiming(
			EndSeconds - TotalStartSeconds,
			PlaneSeconds,
			CompactSeconds,
			TensorSeconds,
			PiecewiseSeconds,
			TriangleSeconds,
			EndSeconds - ArbitrationStartSeconds);
	}
	GAnalyticQueryDetailedTimingEnabled = false;
#endif
	return Best;
}

} // namespace Speed::Analytic

void Speed::Analytic::FWorldQueryService::VisitTorusCandidates(const FWorldQuery& Query,
    TFunctionRef<void(const FTorusSurface&)> Visitor) const
{
    const FBox3d Bounds=SweptQueryBounds(Query);
    for (const auto& Torus:World.Tori)
        if (TorusPassesFilter(Query,Torus) && Torus.Bounds.Intersect(Bounds)) Visitor(Torus);
}
