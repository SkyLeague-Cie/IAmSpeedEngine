#pragma once
#include "PreparedWorldSnapshot.h"
#include "IAmSpeed/World/Simulation/CanonicalFrameContext.h"
#include <atomic>

class ASpeedPhysicsSimulation;
class USpeedWorldSubsystem;
class FSimulationWorker;
struct SComponentTOI;
namespace Speed
{
namespace FrameParallel {struct FSubBodyKinematicInput;}
struct SHitResult;
class FPreparedCanonicalStaticQueryService;
class FNativeFrameConsumerFactoryGT;
class FNativeFrameObjectRetentionGT;
namespace PreparedExecution {class FCanonicalFramePreflight;class FStepScope;}
namespace OnePausedWork {class FNativeFrameStreamScope;}

// This token contains no GC roots. The actual game-thread retention owner
// keeps every captured object alive until that generation is joined.
class FNativeFrameRetentionToken final
{
    friend class FNativeFrameConsumerFactoryGT;
    friend class FNativeFrameObjectRetentionGT;
    std::atomic<bool> Retained{true};
    FNativeFrameRetentionToken()=default;
public:
    ~FNativeFrameRetentionToken()=default;
    bool IsRetained() const noexcept {return Retained.load(std::memory_order_acquire);}
};

enum class ENativeFrameBodyRoute:uint8 {Sphere,Box,Wheel,Sensor};
struct FNativeFrameBodyDispatch
{
    const USSubBody* Body=nullptr;
    ENativeFrameBodyRoute Route=ENativeFrameBodyRoute::Sphere;
    void (*SetFuture)(USSubBody&,const SHitResult&)=nullptr;
    void (*Accept)(USSubBody&)=nullptr;
    void (*Resolve)(USSubBody&,float,float)=nullptr;
    void (*Post)(USSubBody&)=nullptr;
    // Compiled values reader ONLY; not a sync/primitive/query capability.
    bool (*ReadLocalKinematicValues)(const USSubBody&,FrameParallel::FSubBodyKinematicInput&)=nullptr;
};
struct FNativeFrameOwnerDispatch
{
    const ISpeedComponent* Owner=nullptr;
    const UActorComponent* Object=nullptr;
    void (*Prepare)(ISpeedComponent&,const FCanonicalFrameContext&)=nullptr;
    void (*Sync)(ISpeedComponent&)=nullptr;
    void (*Integrate)(ISpeedComponent&,float)=nullptr;
    void (*Post)(ISpeedComponent&,float)=nullptr;
    TArray<FNativeFrameBodyDispatch> Bodies;
};

class FPreparedNativeFrameConsumer;
using FPreparedNativeFrameConsumerPtr=TSharedPtr<const FPreparedNativeFrameConsumer,ESPMode::ThreadSafe>;

// A compiled native dispatch contract permits an exploratory call. Its return
// and observed effects are results of that call, never prerequisites invented
// from a predicted trajectory. Geometry can still refuse any actual query.
// Only the game module's closed native factory can construct this object.
class IAMSPEED_API FPreparedNativeFrameConsumer final
{
    friend class FNativeFrameConsumerFactoryGT;
    friend class FHostedGTStaticWorldLease;
    friend class OnePausedWork::FNativeFrameStreamScope;
    friend class PreparedExecution::FCanonicalFramePreflight;
    friend class PreparedExecution::FStepScope;
    const FPreparedWorldSnapshotPtr Snapshot;
    const TSharedPtr<const FPreparedCanonicalStaticQueryService,ESPMode::ThreadSafe> Service;
    const TSharedPtr<const FNativeFrameRetentionToken,ESPMode::ThreadSafe> Retention;
    const TArray<FNativeFrameOwnerDispatch> Owners;
    mutable std::atomic<bool> ContinuousEnabled{false};
    mutable std::atomic<uint64> LastSnapshotClosure{MAX_uint64};
    mutable const FSimulationWorker* StreamWorker=nullptr; // written only under the actual parked GT lease

    FPreparedNativeFrameConsumer(FPreparedWorldSnapshotPtr,
        TSharedPtr<const FPreparedCanonicalStaticQueryService,ESPMode::ThreadSafe>,
        TSharedPtr<const FNativeFrameRetentionToken,ESPMode::ThreadSafe>,TArray<FNativeFrameOwnerDispatch>&&);
    const FNativeFrameOwnerDispatch& RequireOwner(const ISpeedComponent*) const;
    const FNativeFrameBodyDispatch& RequireBody(const USSubBody*) const;
    void RequireHitTargets(const SHitResult&) const;
public:
    static constexpr uint32 CompiledDispatchVersion=2;
    bool Matches(const FPreparedWorldSnapshotPtr&,
        const TSharedPtr<const FPreparedCanonicalStaticQueryService,ESPMode::ThreadSafe>&) const noexcept;
    // Ordinary resume is permitted only after the real first FullFrame closed
    // and the retained parked lease armed this exact worker. This never arms it.
    bool IsContinuousStreamArmedFor(const FSimulationWorker*) const noexcept;
    bool ContainsOwner(const ISpeedComponent*) const noexcept;
    bool ContainsBody(const USSubBody*) const noexcept;
    bool ContainsRetainedComponentToken(const FWeakObjectPtr&) const noexcept;
    ENativeFrameBodyRoute GetBodyRoute(const USSubBody*) const;
    bool ReadCurrentBodyLocalKinematicValues(const USSubBody*,FrameParallel::FSubBodyKinematicInput&) const;
private:
    // All entry points validate the retained target before native dispatch.
    void PrepareOwner(ISpeedComponent&,const FCanonicalFrameContext&) const;
    void SynchronizeOwner(ISpeedComponent&) const;
    void IntegrateOwner(ISpeedComponent&,float) const;
    void PostOwner(ISpeedComponent&,float) const;
    void SetFutureHit(USSubBody&,const SHitResult&) const;
    void AcceptHit(USSubBody&) const;
    void ResolveHit(USSubBody&,float,float) const;
    void PostBody(USSubBody&) const;
    // Recursion targets are checked before Accept/Resolve can mutate them.
    void RequireResponseTargetsInCensus() const;
};
using FPreparedNativeFrameConsumerFactoryGT=TFunction<bool(const FPreparedWorldSnapshotPtr&,
    const TSharedPtr<const FPreparedCanonicalStaticQueryService,ESPMode::ThreadSafe>&,
    FPreparedNativeFrameConsumerPtr&,FString&)>;
}
