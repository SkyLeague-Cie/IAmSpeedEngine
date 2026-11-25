#include "StaticCollisionWorld.h"
#include "IAmSpeed/World/Analytic/AnalyticCausalStats1075.h"
#include "IAmSpeed/World/Simulation/FrameParallelExecutor.h"
#include "HAL/PlatformTLS.h"
#include "HAL/IConsoleManager.h"
#include "IAmSpeed/World/Simulation/FrameParallelSettings.h"
#include "IAmSpeed/World/Simulation/FrameSpeculationSafety1213.h"
#include <bit>
#include <cmath>
#include <exception>
#include <stdexcept>
#if !UE_BUILD_SHIPPING
#include "../Analytic/StaticWorldQueryAudit.h"
#endif

namespace
{
std::atomic<uint64> GImmutableQueryGeneration1075{1};
TAutoConsoleVariable<int32> CVarIndependentQueryChunkItems1075(
    TEXT("p.IAmSpeed.Parallel.QueryChunkItems1075"), 0,
    TEXT("Independent whole-sweep scheduling:0=1074 round robin; positive=dynamic chunk size (1..256). Private cache stays with its lane."));
}

namespace Speed
{

namespace
{
bool HasThreadOwnedQueryDiagnostics()
{
#if !UE_BUILD_SHIPPING
    // These counters/capture identities belong to the calling native thread.
    // A parallel run must not silently lose them or borrow another identity.
    static const TCHAR* Names[] = {
        TEXT("p.IAmSpeed.AnalyticWorld.AuthorityChaosShadow"),
        TEXT("p.IAmSpeed.AnalyticWorld.CacheProfile"),
        TEXT("p.IAmSpeed.AnalyticWorld.CaseCpuPhase400"),
        TEXT("p.IAmSpeed.AnalyticWorld.CaseSatDiagnostics"),
        TEXT("p.IAmSpeed.AnalyticWorld.CompactShadow"),
        TEXT("p.IAmSpeed.AnalyticWorld.ExactCrossCostProbe844"),
        TEXT("p.IAmSpeed.AnalyticWorld.ExactCrossNormalCache784"),
        TEXT("p.IAmSpeed.AnalyticWorld.ExtrudedBoxPrimarySatScopeCache"),
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
void FinishParticipation(FIndependentQueryBatchStats& Stats)
{
    for (int32 Lane = 0; Lane < Stats.DispatchedLogicalLanes; ++Lane)
    {
        bool Seen = false;
        for (int32 Earlier = 0; Earlier < Lane; ++Earlier)
            Seen |= Stats.NativeThreadIds[Earlier] == Stats.NativeThreadIds[Lane];
        if (!Seen) ++Stats.DistinctNativeThreads;
        if (Stats.bWorkCountsKnown1075 && Stats.ExecutedItems1075[Lane]>0) ++Stats.NonEmptyLanes1075;
    }
}
}

bool CanSpeculateIndependentAnalyticGeometry1075()
{
    return !IsInsideFrameParallelTask() && !HasThreadOwnedQueryDiagnostics()
        && AreCurrentFrameFloatingPointExceptionsMasked1213();
}

bool IStaticCollisionWorld::SweepIndependentBatch(
    TConstArrayView<Analytic::FWorldQuery> Queries,
    TArrayView<Analytic::FWorldHit> OutHits,
    FIndependentQueryBatchStats* OutStats) const
{
    FIndependentQueryBatchStats Stats;
    if (Queries.Num() != OutHits.Num())
    {
        Stats.Fallback = EIndependentQueryFallback::InvalidOutputExtent;
        if (OutStats) *OutStats = Stats;
        return false;
    }
    Stats.Fallback = EIndependentQueryFallback::SerialPolicy;
    Stats.bWorkCountsKnown1075 = true;
    Stats.ExecutedItems1075[0] = Queries.Num();
    Stats.NonEmptyLanes1075 = Queries.IsEmpty() ? 0 : 1;
    if (!Queries.IsEmpty())
    {
        Stats.DispatchedLogicalLanes = 1;
        Stats.NativeThreadIds[0] = FPlatformTLS::GetCurrentThreadId();
        Stats.DistinctNativeThreads = 1;
    }
    for (int32 Index = 0; Index < Queries.Num(); ++Index)
        OutHits[Index] = SweepSingle(Queries[Index]);
    if (OutStats) *OutStats = Stats;
    return true;
}

FAnalyticStaticCollisionWorld::FAnalyticStaticCollisionWorld(
    const Analytic::FAnalyticWorldData& InWorld, int32 InIndependentQueryLanes,
    int32 InMinimumIndependentQueryItems, int32 InMinimumFacetItems)
    : WorldData(InWorld), ImmutableQueryGeneration1075(GImmutableQueryGeneration1075.fetch_add(1, std::memory_order_relaxed)),
      QueryService(InWorld, FMath::Clamp(InIndependentQueryLanes, 1, FFrameParallelPolicy::MaximumLanes),
        FMath::Max(2, InMinimumFacetItems))
{
    Analytic::RequireCurrentAnalyticNumericContext(WorldData.NumericContract);

    IndependentQueryLanes = FMath::Clamp(InIndependentQueryLanes, 1, FFrameParallelPolicy::MaximumLanes);
    MinimumIndependentQueryItems = FMath::Max(2, InMinimumIndependentQueryItems);
    if (IndependentQueryLanes > 1)
    {
        IndependentQueryServices.Reserve(IndependentQueryLanes);
        for (int32 Lane = 0; Lane < IndependentQueryLanes; ++Lane)
            IndependentQueryServices.Add(MakeUnique<Analytic::FWorldQueryService>(WorldData, 1, InMinimumFacetItems));
    }
}

namespace
{
class FExactRoundTensorDigest final
{
public:
    // The scan admits at most 64 MiB of encoded scalar input and allocates no storage.
    static constexpr uint32 MaxWords = 8 * 1024 * 1024;

    bool Word(uint64 Value) noexcept
    {
        if (Words == MaxWords) return false;
        ++Words;
        for (unsigned Byte = 0; Byte < sizeof(Value); ++Byte)
        {
            Hash = (Hash ^ static_cast<uint8>(Value)) * 1099511628211ull;
            Value >>= 8;
        }
        return true;
    }

    bool Integer(int32 Value) noexcept { return Word(static_cast<uint64>(Value)); }

    bool Number(double Value) noexcept
    {
        return std::isfinite(Value) && Word(std::bit_cast<uint64>(Value));
    }

    bool Vector(const FVector3d& Value) noexcept
    {
        return Number(Value.X) && Number(Value.Y) && Number(Value.Z);
    }

    bool Bounds(const FBox3d& Value) noexcept
    {
        return Value.IsValid && Vector(Value.Min) && Vector(Value.Max);
    }

    bool Bvh(const TArray<Analytic::FTriangleBvhNode>& Nodes,
        const TArray<int32>& Indices, int32 Items) noexcept
    {
        if (Items <= 0 || Nodes.IsEmpty() || Indices.Num() != Items || !Integer(Nodes.Num())) return false;
        for (const auto& Node : Nodes)
        {
            if (!Bounds(Node.Bounds) || !Integer(Node.FirstIndex) || !Integer(Node.IndexCount) ||
                !Integer(Node.LeftChild) || !Integer(Node.RightChild)) return false;
        }
        for (int32 Index : Indices)
        {
            if (Index < 0 || Index >= Items || !Integer(Index)) return false;
        }
        return true;
    }

    bool Approximation(const TArray<Analytic::FTensorBezierApproximationCell>& Cells,
        const TArray<Analytic::FTriangleBvhNode>& Nodes, const TArray<int32>& Indices) noexcept
    {
        if (!Integer(Cells.Num())) return false;
        for (const auto& Cell : Cells)
        {
            if (!Number(Cell.MinimumU) || !Number(Cell.MaximumU) || !Number(Cell.MinimumV) ||
                !Number(Cell.MaximumV) || !Number(Cell.MaximumErrorCm)) return false;
            for (const auto& Corner : Cell.Corners)
                if (!Vector(Corner)) return false;
            for (const auto Edges : Cell.PositiveConcaveEdges)
                if (!Word(Edges)) return false;
            for (const auto Edges : Cell.NegativeConcaveEdges)
                if (!Word(Edges)) return false;
        }
        return Bvh(Nodes, Indices, Cells.Num());
    }

    uint64 Value() const noexcept { return Hash; }

private:
    uint64 Hash = 14695981039346656037ull;
    uint32 Words = 0;
};
}

std::optional<FExactRoundTensorGeometryWitness>
FAnalyticStaticCollisionWorld::BuildExactRoundTensorGeometryWitness() const noexcept
{
    const auto Contract = GetNumericContract();
    if (WorldData.SchemaVersion != Analytic::AnalyticWorldSchemaVersion ||
        (Contract != Analytic::EAnalyticNumericContract::Unspecified &&
         Contract != Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush)) return {};

    FExactRoundTensorDigest Digest;
    constexpr auto Schema = FExactRoundTensorGeometryWitness::Schema;
    if (!Digest.Word(0x49415354454e0000ull | Schema) || !Digest.Word(WorldData.SchemaVersion) ||
        !Digest.Integer(WorldData.TensorBezierPatches.Num())) return {};

    for (const auto& Patch : WorldData.TensorBezierPatches)
    {
        if (!Patch.bApproximationCertified || !Digest.Word(Patch.PrimitiveId) ||
            !Digest.Bounds(Patch.Bounds) || !Digest.Number(Patch.MaximumApproximationErrorCm) ||
            !Digest.Approximation(Patch.ApproximationCells,
                Patch.ApproximationCellBvhNodes, Patch.ApproximationCellBvhIndices)) return {};
    }

    if (!Digest.Integer(WorldData.PiecewiseTensorBezierPatches.Num())) return {};
    for (const auto& Patch : WorldData.PiecewiseTensorBezierPatches)
    {
        if (!Patch.bApproximationCertified || !Digest.Word(Patch.PrimitiveId) ||
            !Digest.Bounds(Patch.Bounds) || !Digest.Number(Patch.MaximumApproximationErrorCm) ||
            !Digest.Integer(Patch.Cells.Num())) return {};
        for (const auto& Cell : Patch.Cells)
        {
            if (!Digest.Word(Cell.PrimitiveId) || !Digest.Bounds(Cell.Bounds) ||
                !Digest.Number(Cell.MaximumApproximationErrorCm) ||
                !Digest.Approximation(Cell.ApproximationCells,
                    Cell.ApproximationCellBvhNodes, Cell.ApproximationCellBvhIndices)) return {};
            for (const int32 Cone : Cell.InternalCornerNormalCone)
                if (!Digest.Integer(Cone)) return {};
        }
        if (!Digest.Integer(Patch.InternalCornerNormalCones.Num())) return {};
        for (const auto& Cone : Patch.InternalCornerNormalCones)
        {
            if (!Digest.Number(Cone.X) || !Digest.Number(Cone.Y) || !Digest.Number(Cone.Z) ||
                !Digest.Number(Cone.W)) return {};
        }
        if (!Digest.Bvh(Patch.CellBvhNodes, Patch.CellBvhIndices, Patch.Cells.Num())) return {};
    }

    if (!Digest.Value()) return {};
    return FExactRoundTensorGeometryWitness{Schema, WorldData.SchemaVersion, Digest.Value(), Contract};
}

bool FAnalyticStaticCollisionWorld::SweepIndependentBatch(
    TConstArrayView<Analytic::FWorldQuery> Queries, TArrayView<Analytic::FWorldHit> OutHits,
    FIndependentQueryBatchStats* OutStats) const
{
    return SweepIndependentBatchImpl1075(Queries, OutHits, OutStats, false);
}
bool FAnalyticStaticCollisionWorld::CanPrefetchIndependentQueries1075() const
{
    Analytic::RequireCurrentAnalyticNumericContext(WorldData.NumericContract);

    return IndependentQueryLanes > 1 && !IsInsideFrameParallelTask() &&
        AreCurrentFrameFloatingPointExceptionsMasked1213() &&
        !HasThreadOwnedQueryDiagnostics() && !IndependentBatchBusy.load(std::memory_order_relaxed);
}
bool FAnalyticStaticCollisionWorld::SweepPrefetchBatch1075(
    TConstArrayView<Analytic::FWorldQuery> Queries, TArrayView<Analytic::FWorldHit> OutHits,
    FIndependentQueryBatchStats* OutStats) const
{
    return SweepIndependentBatchImpl1075(Queries, OutHits, OutStats, true);
}
bool FAnalyticStaticCollisionWorld::SweepIndependentBatchImpl1075(
    TConstArrayView<Analytic::FWorldQuery> Queries, TArrayView<Analytic::FWorldHit> OutHits,
    FIndependentQueryBatchStats* OutStats, bool bRejectSerialFallback) const
{
    Analytic::RequireCurrentAnalyticNumericContext(WorldData.NumericContract);

    FIndependentQueryBatchStats Stats;
    Stats.RequestedLanes = IndependentQueryLanes;
    Analytic::FScopedAnalyticCausalObservation1075 Causal(Analytic::EAnalyticCausalKind1075::IndependentSweepBatch, &Stats);
    Causal.SetItems(Queries.Num());
    if (Queries.Num() != OutHits.Num())
    {
        Stats.Fallback = EIndependentQueryFallback::InvalidOutputExtent;
        if (OutStats) *OutStats = Stats;
        return false; // Outputs are untouched.
    }
    const auto Serial = [&](EIndependentQueryFallback Reason)
    {
        if (bRejectSerialFallback)
        {
            Stats.Fallback = Reason;
            if (OutStats) *OutStats = Stats;
            return false; // No query, callback, recorder, or output publication.
        }
        Causal.MarkPrepared();
        {
            Analytic::FScopedAnalyticCycles1075 Numeric(Causal.NumericTarget(0));
            // A refused item never exposes preceding partial physical hits.
            TArray<Analytic::FWorldHit> PrivateSerialHits;
            PrivateSerialHits.SetNum(Queries.Num());
            if (!IStaticCollisionWorld::SweepIndependentBatch(Queries, PrivateSerialHits, &Stats)) return false;
            Analytic::RequireCurrentAnalyticNumericContext(WorldData.NumericContract);
            for (int32 I=0; I<Queries.Num(); ++I) OutHits[I]=MoveTemp(PrivateSerialHits[I]);
        }
        Stats.RequestedLanes = IndependentQueryLanes;
        Stats.Fallback = Reason;
        if (OutStats) *OutStats = Stats;
        EFrameParallelFallback StageFallback = EFrameParallelFallback::SerialPolicy;
        switch (Reason)
        {
        case EIndependentQueryFallback::TooFewQueryItems: StageFallback = EFrameParallelFallback::TooFewItems; break;
        case EIndependentQueryFallback::NestedTask: StageFallback = EFrameParallelFallback::NestedTask; break;
        case EIndependentQueryFallback::ThreadOwnedDiagnostics: StageFallback = EFrameParallelFallback::ThreadOwnedDiagnostics; break;
        case EIndependentQueryFallback::BatchCapacityExceeded: StageFallback = EFrameParallelFallback::BatchCapacityExceeded; break;
        case EIndependentQueryFallback::NumericEnvironmentUnavailable: StageFallback = EFrameParallelFallback::NumericEnvironmentUnavailable; break;
        default: break;
        }
        RecordFrameParallelStage(EFrameParallelStage::QueryBatch, Queries.Num(),
            Stats.DispatchedLogicalLanes, Stats.DistinctNativeThreads, StageFallback);
        return true;
    };
    if (IndependentQueryLanes == 1) return Serial(EIndependentQueryFallback::SerialPolicy);
    if (IsInsideFrameParallelTask()) return Serial(EIndependentQueryFallback::NestedTask);
    if (Queries.Num() > 256) return Serial(EIndependentQueryFallback::BatchCapacityExceeded);
    if (Queries.Num() < MinimumIndependentQueryItems) return Serial(EIndependentQueryFallback::TooFewQueryItems);
    if (HasThreadOwnedQueryDiagnostics()) return Serial(EIndependentQueryFallback::ThreadOwnedDiagnostics);
    if (!AreCurrentFrameFloatingPointExceptionsMasked1213())
        return Serial(EIndependentQueryFallback::NumericEnvironmentUnavailable);
    const FFrameFloatingPointContext OwnerNumericContext = CaptureFrameFloatingPointContext();
    Analytic::RequireAnalyticNumericTransfer(WorldData.NumericContract,OwnerNumericContext.bValid);
    if (!OwnerNumericContext.bValid) return Serial(EIndependentQueryFallback::NumericEnvironmentUnavailable);
    bool Expected = false;
    if (!IndependentBatchBusy.compare_exchange_strong(Expected, true, std::memory_order_acquire))
    {
        Stats.Fallback = EIndependentQueryFallback::Busy;
        if (OutStats) *OutStats = Stats;
        return false; // Concurrent/reentrant batches do not share lane scratch.
    }
    struct FRelease
    {
        std::atomic<bool>& Flag;
        ~FRelease() { Flag.store(false, std::memory_order_release); }
    } Release{IndependentBatchBusy};
    Stats.DispatchedLogicalLanes = FMath::Min(IndependentQueryLanes, Queries.Num());
    Stats.bWorkCountsKnown1075 = true;
    const int32 ChunkItems = FMath::Clamp(CVarIndependentQueryChunkItems1075.GetValueOnAnyThread(), 0, 256);
    std::atomic<int32> NextQuery{0};
    TArray<Analytic::FWorldHit, TInlineAllocator<8>> PrivateHits;
    PrivateHits.SetNum(Queries.Num());
    std::exception_ptr Failures[FFrameParallelPolicy::MaximumLanes];
    const auto ExecuteLane = [&, OwnerNumericContext](int32 Lane)
    {
        FScopedFrameParallelTask TaskScope(OwnerNumericContext);
        Stats.NativeThreadIds[Lane] = FPlatformTLS::GetCurrentThreadId();
        try
        {
            Analytic::RequireAnalyticNumericTransfer(WorldData.NumericContract,TaskScope.IsNumericContextApplied());
            if (!TaskScope.IsNumericContextApplied())
                throw std::runtime_error("IAmSpeed query batch numeric context unavailable");
            Analytic::RequireCurrentAnalyticNumericContext(WorldData.NumericContract);
            const Analytic::FWorldQueryService& Service = *IndependentQueryServices[Lane];
            const auto SweepIndex = [&](int32 Index)
            {
                Analytic::RequireCurrentAnalyticNumericContext(WorldData.NumericContract);
                Analytic::FWorldQuery CollisionQuery = Queries[Index];
                CollisionQuery.bUseFiniteContactDomain = true;
                PrivateHits[Index] = Service.Sweep(CollisionQuery);
                ++Stats.ExecutedItems1075[Lane];
            };
            Analytic::FScopedAnalyticCycles1075 Numeric(Causal.NumericTarget(Lane));
            if (ChunkItems == 0)
            {
                for (int32 Index = Lane; Index < Queries.Num(); Index += Stats.DispatchedLogicalLanes)
                    SweepIndex(Index);
            }
            else
            {
                for (;;)
                {
                    const int32 Begin = NextQuery.fetch_add(ChunkItems, std::memory_order_relaxed);
                    if (Begin >= Queries.Num()) break;
                    const int32 End = FMath::Min(Begin + ChunkItems, Queries.Num());
                    for (int32 Index = Begin; Index < End; ++Index) SweepIndex(Index);
                }
            }
        }
        catch (...) { Failures[Lane] = std::current_exception(); }
        if (!TaskScope.RestoreNumericContext())
            Failures[Lane] = Analytic::AnalyticNumericTransferException(WorldData.NumericContract,Analytic::EAnalyticNumericFailure::ContextRestoration,"IAmSpeed query batch numeric restoration failed");
    };
    Causal.MarkPrepared();
    DispatchFrameParallelLanes(Stats.DispatchedLogicalLanes, ExecuteLane, Causal.DispatchObservation());
    FinishParticipation(Stats);
    RecordFrameParallelStage(EFrameParallelStage::QueryBatch, Queries.Num(),
        Stats.DispatchedLogicalLanes, Stats.DistinctNativeThreads, EFrameParallelFallback::None);
    for (int32 Lane = 0; Lane < Stats.DispatchedLogicalLanes; ++Lane)
        if (Failures[Lane])
        {
            Stats.Fallback = EIndependentQueryFallback::NumericEnvironmentUnavailable;
            if (OutStats) *OutStats = Stats;
            std::rethrow_exception(Failures[Lane]);
        }
    // Caller outputs become visible only after all private lanes restore and join.
    {
        Analytic::FScopedAnalyticCycles1075 Publish(Causal.FoldTarget());
        for (int32 Index = 0; Index < Queries.Num(); ++Index) OutHits[Index] = MoveTemp(PrivateHits[Index]);
    }
    if (OutStats) *OutStats = Stats;
    return true;
}


FEstablishedStaticContactProjection ProjectEstablishedStaticContact(
	const IStaticCollisionWorld& World,
	Analytic::FWorldQuery Query,
	const uint32 MaxCorrectionIterations,
	const double PenetrationToleranceCm)
{
	FEstablishedStaticContactProjection Result;
	Result.Location = Query.Start;
	if (Query.RequiredSourceId == 0 || Query.RequiredSurfaceId == 0 ||
		Query.Start != Query.End || MaxCorrectionIterations == 0)
	{
		return Result;
	}

	const double Tolerance = FMath::Max(0.0, PenetrationToleranceCm);
	Query.InitialOverlapTolerance = 0.0;
	for (uint32 Iteration = 0; Iteration < MaxCorrectionIterations; ++Iteration)
	{
		Query.Start = Result.Location;
		Query.End = Result.Location;
		const Analytic::FWorldHit Candidate = World.SweepSingle(Query);
#if !UE_BUILD_SHIPPING
		Analytic::FStaticWorldQueryAudit::RecordEstablishedContactQuery(Query, Candidate, Iteration, false);
#endif
		if (!Candidate.bHit)
		{
			Result.ResidualPenetrationDepth = 0.0;
			Result.bConverged = Result.bContact;
			return Result;
		}
		Result.Hit = Candidate;
		Result.bContact = true;

		const double Depth = Result.Hit.bStartPenetrating
			? FMath::Max(0.0, Result.Hit.PenetrationDepth) : 0.0;
		Result.MaximumPenetrationDepth =
			FMath::Max(Result.MaximumPenetrationDepth, Depth);
		Result.ResidualPenetrationDepth = Depth;
		if (Depth <= Tolerance)
		{
			Result.bConverged = true;
			return Result;
		}

		const FVector3d Normal = Result.Hit.Normal.GetSafeNormal();
		if (Normal.IsNearlyZero())
		{
			return Result;
		}
		Result.Location += Depth * Normal;
		++Result.CorrectionIterations;
	}

	// The loop's final correction has not been measured yet. One validation
	// query keeps the correction count bounded while reporting the true residual.
	Query.Start = Result.Location;
	Query.End = Result.Location;
	const Analytic::FWorldHit ValidationHit = World.SweepSingle(Query);
#if !UE_BUILD_SHIPPING
	Analytic::FStaticWorldQueryAudit::RecordEstablishedContactQuery(Query, ValidationHit, MaxCorrectionIterations, true);
#endif
	if (ValidationHit.bHit)
	{
		Result.Hit = ValidationHit;
		Result.bContact = true;
	}
	Result.ResidualPenetrationDepth =
		ValidationHit.bHit && ValidationHit.bStartPenetrating
		? FMath::Max(0.0, ValidationHit.PenetrationDepth) : 0.0;
	Result.bConverged = Result.ResidualPenetrationDepth <= Tolerance;
	return Result;
}

FVector TransportEstablishedContactVelocity(
	const FVector& LinearVelocity,
	const FVector& RotationalPointVelocity,
	const FVector& PreviousNormal,
	const FVector& CurrentNormal)
{
	const FVector From = PreviousNormal.GetSafeNormal();
	const FVector To = CurrentNormal.GetSafeNormal();
	if (From.IsNearlyZero() || To.IsNearlyZero() ||
		FVector::DotProduct(From, To) <= 0.0f)
	{
		return LinearVelocity;
	}

	FVector Result = FQuat::FindBetweenNormals(From, To).RotateVector(
		LinearVelocity);
	const float NormalSpeed = FVector::DotProduct(
		Result + RotationalPointVelocity, To);
	if (NormalSpeed < 0.0f)
	{
		Result -= NormalSpeed * To;
	}
	return Result;
}

} // namespace Speed
