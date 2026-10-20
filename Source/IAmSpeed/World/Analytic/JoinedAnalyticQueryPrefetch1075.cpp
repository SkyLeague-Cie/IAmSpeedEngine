#include "JoinedAnalyticQueryPrefetch1075.h"
#include <mutex>

namespace Speed::Analytic
{
namespace
{
template <typename T> bool SameBitsJoinedPrefetch1075(const T& A, const T& B)
{ return FMemory::Memcmp(&A, &B, sizeof(T)) == 0; }
bool SameVectorBitsJoinedPrefetch1075(const FVector3d& A, const FVector3d& B)
{ return SameBitsJoinedPrefetch1075(A.X,B.X) && SameBitsJoinedPrefetch1075(A.Y,B.Y) && SameBitsJoinedPrefetch1075(A.Z,B.Z); }
std::mutex GJoinedPrefetchStatsMutex1075;
FAnalyticPrefetchStats1075 GJoinedPrefetchStats1075;
}
bool AreAnalyticPrefetchQueriesBitwiseEqual1075(const FWorldQuery& A, const FWorldQuery& B)
{
    // The canonical full cache key owns the field list; numeric comparisons are
    // additionally bitwise here, including signed zero. No padding is read.
    return FWorldQueryService::QueriesHaveSameCacheKey1075(A,B) &&
        SameVectorBitsJoinedPrefetch1075(A.Start,B.Start) && SameVectorBitsJoinedPrefetch1075(A.End,B.End) &&
        SameBitsJoinedPrefetch1075(A.Rotation.X,B.Rotation.X) && SameBitsJoinedPrefetch1075(A.Rotation.Y,B.Rotation.Y) &&
        SameBitsJoinedPrefetch1075(A.Rotation.Z,B.Rotation.Z) && SameBitsJoinedPrefetch1075(A.Rotation.W,B.Rotation.W) &&
        SameBitsJoinedPrefetch1075(A.Radius,B.Radius) && SameVectorBitsJoinedPrefetch1075(A.HalfExtent,B.HalfExtent) &&
        SameBitsJoinedPrefetch1075(A.DomainTolerance,B.DomainTolerance) &&
        SameBitsJoinedPrefetch1075(A.InitialOverlapTolerance,B.InitialOverlapTolerance) &&
        SameVectorBitsJoinedPrefetch1075(A.ReferenceNormal,B.ReferenceNormal) &&
        SameBitsJoinedPrefetch1075(A.MinimumReferenceNormalDot,B.MinimumReferenceNormalDot);
}
FJoinedAnalyticQueryCache1075::FJoinedAnalyticQueryCache1075(
    TConstArrayView<FAnalyticPrefetchQuery1075> Queries, TConstArrayView<FWorldHit> Hits,
    const FAnalyticPrefetchContext1075& InContext) : Context(InContext)
{
    if (Queries.Num()!=Hits.Num() || Queries.Num()>MaximumQueries ||
        !Context.World || !Context.RuntimeBridge || !Context.ImmutableQuery.IsValid() ||
        !Context.NumericContext.bValid) return;
    Records.Reserve(Queries.Num());
    for (int32 Index=0; Index<Queries.Num(); ++Index)
    {
        FRecord Record; Record.Captured=Queries[Index]; Record.Hit=Hits[Index];
        Records.Add(MoveTemp(Record));
    }
}
EAnalyticPrefetchReuse1075 FJoinedAnalyticQueryCache1075::TryConsume(
    const FAnalyticPrefetchContext1075& CurrentContext,
    const FAnalyticPrefetchConsumer1075& Consumer, const FWorldQuery& Query,
    double MaximumSearchTime, FWorldHit& OutHit)
{
    if (!Consumer.Owner) return EAnalyticPrefetchReuse1075::NoEntry;
    for (FRecord& Record : Records)
    {
        if (Record.Captured.Consumer.Owner!=Consumer.Owner || Record.bConsumed || Record.bRejected) continue;
        EAnalyticPrefetchReuse1075 Result=EAnalyticPrefetchReuse1075::Reused;
        if (CurrentContext.World!=Context.World || CurrentContext.RuntimeBridge!=Context.RuntimeBridge ||
            CurrentContext.Frame!=Context.Frame || !(CurrentContext.ImmutableQuery==Context.ImmutableQuery))
            Result=EAnalyticPrefetchReuse1075::ContextChanged;
        else if (!CurrentContext.NumericContext.bValid ||
            CurrentContext.NumericContext.VectorControl!=Context.NumericContext.VectorControl ||
            FMemory::Memcmp(&CurrentContext.NumericContext.Environment,
                &Context.NumericContext.Environment,sizeof(std::fenv_t))!=0)
            Result=EAnalyticPrefetchReuse1075::NumericEnvironmentChanged;
        else if (Consumer.ExactType!=Record.Captured.Consumer.ExactType)
            Result=EAnalyticPrefetchReuse1075::ExactTypeChanged;
        else if (Consumer.DeltaBits!=Record.Captured.Consumer.DeltaBits)
            Result=EAnalyticPrefetchReuse1075::DeltaChanged;
        else if (!SameBitsJoinedPrefetch1075(MaximumSearchTime,Record.Captured.MaximumSearchTime))
            Result=EAnalyticPrefetchReuse1075::SearchBoundChanged;
        else if (!AreAnalyticPrefetchQueriesBitwiseEqual1075(Query,Record.Captured.Query))
            Result=EAnalyticPrefetchReuse1075::QueryChanged;
        if (Result!=EAnalyticPrefetchReuse1075::Reused)
        {
            Record.bRejected=true; ++RejectedRecords; ++RejectedReasons[int32(Result)];
            return Result;
        }
        OutHit=Record.Hit; Record.bConsumed=true; ++ConsumedRecords;
        return Result;
    }
    return EAnalyticPrefetchReuse1075::NoEntry;
}
void Private::AccumulateAnalyticPrefetchStats1075(const FAnalyticPrefetchStats1075& S)
{
    std::lock_guard<std::mutex> Lock(GJoinedPrefetchStatsMutex1075);
    GJoinedPrefetchStats1075.ScopeAttempts+=S.ScopeAttempts; GJoinedPrefetchStats1075.PreparedBatches+=S.PreparedBatches;
    GJoinedPrefetchStats1075.FallbackScopes+=S.FallbackScopes; GJoinedPrefetchStats1075.Captured+=S.Captured;
    GJoinedPrefetchStats1075.Computed+=S.Computed; GJoinedPrefetchStats1075.Reused+=S.Reused;
    GJoinedPrefetchStats1075.Rejected+=S.Rejected; GJoinedPrefetchStats1075.Unused+=S.Unused;
    for (int32 Reason=0; Reason<8; ++Reason) GJoinedPrefetchStats1075.RejectedReasons[Reason]+=S.RejectedReasons[Reason];
    GJoinedPrefetchStats1075.MaximumLogicalLanes=FMath::Max(GJoinedPrefetchStats1075.MaximumLogicalLanes,S.MaximumLogicalLanes);
    GJoinedPrefetchStats1075.MaximumNativeThreads=FMath::Max(GJoinedPrefetchStats1075.MaximumNativeThreads,S.MaximumNativeThreads);
    GJoinedPrefetchStats1075.MaximumNonEmptyLanes=FMath::Max(GJoinedPrefetchStats1075.MaximumNonEmptyLanes,S.MaximumNonEmptyLanes);
}
FAnalyticPrefetchStats1075 SnapshotAnalyticPrefetchStats1075()
{ std::lock_guard<std::mutex> Lock(GJoinedPrefetchStatsMutex1075); return GJoinedPrefetchStats1075; }
void ResetAnalyticPrefetchStats1075()
{ std::lock_guard<std::mutex> Lock(GJoinedPrefetchStatsMutex1075); GJoinedPrefetchStats1075=FAnalyticPrefetchStats1075{}; }
void LogAnalyticPrefetchStats1075(const TCHAR* Context)
{
    const auto S=SnapshotAnalyticPrefetchStats1075();
    UE_LOG(LogTemp,Display,TEXT("[IAmSpeedAnalyticPrefetch1075] Context=%s Scopes=%llu Batches=%llu FallbackScopes=%llu Captured=%llu Computed=%llu Reused=%llu Rejected=%llu Unused=%llu LogicalMax=%d NativeMax=%d UsefulLaneMax=%d RejectContext=%llu RejectType=%llu RejectDt=%llu RejectBound=%llu RejectQuery=%llu RejectNumeric=%llu Scope=SpeculativeGeometryOnly Consumption=OriginalSerialOrder WheelRequested=%d WheelEffective=0 WheelStatus=%s"),
        Context?Context:TEXT(""),S.ScopeAttempts,S.PreparedBatches,S.FallbackScopes,S.Captured,
        S.Computed,S.Reused,S.Rejected,S.Unused,S.MaximumLogicalLanes,S.MaximumNativeThreads,S.MaximumNonEmptyLanes,
        S.RejectedReasons[2],S.RejectedReasons[3],S.RejectedReasons[4],S.RejectedReasons[5],S.RejectedReasons[6],S.RejectedReasons[7],IsWheelSuspensionPrefetch1075Requested()?1:0,WheelSuspensionPrefetch1075Status());
}
}
