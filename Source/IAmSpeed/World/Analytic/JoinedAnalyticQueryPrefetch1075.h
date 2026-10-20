#pragma once

#include "AnalyticWorldQuery.h"
#include "IAmSpeed/World/Simulation/FrameFloatingPointContext.h"
#include "IAmSpeed/World/Collision/StaticQueryBatch.h"

class USWheelSubBody;

namespace Speed::Analytic
{
struct FAnalyticPrefetchConsumer1075
{
    const void* Owner = nullptr;
    const void* ExactType = nullptr;
    uint32 DeltaBits = 0;
};
struct FAnalyticPrefetchQuery1075
{
    FAnalyticPrefetchConsumer1075 Consumer;
    FWorldQuery Query;
    // SweepSingle has the full normalized interval. Internal tightening remains
    // entirely inside that unchanged sweep, never across speculative queries.
    double MaximumSearchTime = 1.0;
};
struct FAnalyticPrefetchContext1075
{
    const void* World = nullptr;
    const void* RuntimeBridge = nullptr;
    FImmutableStaticQueryIdentity1075 ImmutableQuery;
    uint64 Frame = 0;
    FFrameFloatingPointContext NumericContext;
};
enum class EAnalyticPrefetchReuse1075 : uint8
{
    Reused, NoEntry, ContextChanged, ExactTypeChanged, DeltaChanged,
    SearchBoundChanged, QueryChanged, NumericEnvironmentChanged
};

IAMSPEED_API bool AreAnalyticPrefetchQueriesBitwiseEqual1075(
    const FWorldQuery& A, const FWorldQuery& B);

// Owner-only joined result cache. It never runs a query, dereferences identity
// tokens, visits UObjects, or changes authoritative state. Every record can be
// consumed once. A changed input rejects that record and leaves OutHit intact.
class IAMSPEED_API FJoinedAnalyticQueryCache1075
{
public:
    static constexpr int32 MaximumQueries = 64;
    FJoinedAnalyticQueryCache1075(TConstArrayView<FAnalyticPrefetchQuery1075> Queries,
        TConstArrayView<FWorldHit> Hits, const FAnalyticPrefetchContext1075& InContext);
    EAnalyticPrefetchReuse1075 TryConsume(const FAnalyticPrefetchContext1075& CurrentContext,
        const FAnalyticPrefetchConsumer1075& Consumer, const FWorldQuery& Query,
        double MaximumSearchTime, FWorldHit& OutHit);
    int32 Num() const { return Records.Num(); }
    int32 Consumed() const { return ConsumedRecords; }
    int32 Rejected() const { return RejectedRecords; }
    uint64 RejectedFor(EAnalyticPrefetchReuse1075 Reason) const
    { return RejectedReasons[int32(Reason)]; }
private:
    struct FRecord
    {
        FAnalyticPrefetchQuery1075 Captured;
        FWorldHit Hit;
        bool bConsumed = false, bRejected = false;
    };
    FAnalyticPrefetchContext1075 Context;
    TArray<FRecord> Records;
    int32 ConsumedRecords = 0, RejectedRecords = 0;
    uint64 RejectedReasons[8] = {};
};

struct FAnalyticPrefetchStats1075
{
    uint64 ScopeAttempts = 0, PreparedBatches = 0, FallbackScopes = 0;
    uint64 Captured = 0, Computed = 0, Reused = 0, Rejected = 0, Unused = 0;
    uint64 RejectedReasons[8] = {};
    int32 MaximumLogicalLanes = 0, MaximumNativeThreads = 0, MaximumNonEmptyLanes = 0;
};
// Read/reset/log only after joined case boundaries. Workers never log.
IAMSPEED_API FAnalyticPrefetchStats1075 SnapshotAnalyticPrefetchStats1075();
IAMSPEED_API void ResetAnalyticPrefetchStats1075();
IAMSPEED_API void LogAnalyticPrefetchStats1075(const TCHAR* Context);
namespace Private { void AccumulateAnalyticPrefetchStats1075(const FAnalyticPrefetchStats1075& Stats); }

IAMSPEED_API bool IsWheelSuspensionPrefetch1075Requested();
IAMSPEED_API const TCHAR* WheelSuspensionPrefetch1075Status();
IAMSPEED_API bool IsWheelSuspensionPrefetch1075Enabled();
class IAMSPEED_API FScopedWheelSuspensionPrefetch1075
{
public:
    struct FImpl;
    FScopedWheelSuspensionPrefetch1075(TConstArrayView<TObjectPtr<USWheelSubBody>> Wheels,
        float Delta);
    ~FScopedWheelSuspensionPrefetch1075();
    FScopedWheelSuspensionPrefetch1075(const FScopedWheelSuspensionPrefetch1075&) = delete;
    FScopedWheelSuspensionPrefetch1075& operator=(const FScopedWheelSuspensionPrefetch1075&) = delete;
private:
    TUniquePtr<FImpl> State;
};

// The original acquisition probe supplies its identity/dt at consumption time.
// OFF does only a TLS-null check; it does not inspect the wheel/type or log.
class IAMSPEED_API FScopedAnalyticPrefetchConsumer1075
{
public:
    FScopedAnalyticPrefetchConsumer1075(const USWheelSubBody* Wheel, float Delta);
    ~FScopedAnalyticPrefetchConsumer1075();
    FScopedAnalyticPrefetchConsumer1075(const FScopedAnalyticPrefetchConsumer1075&) = delete;
    FScopedAnalyticPrefetchConsumer1075& operator=(const FScopedAnalyticPrefetchConsumer1075&) = delete;
private:
    FAnalyticPrefetchConsumer1075 Previous;
    bool bInstalled = false;
};
}
