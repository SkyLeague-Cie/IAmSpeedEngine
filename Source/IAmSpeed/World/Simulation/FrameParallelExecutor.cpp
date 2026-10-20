#include "FrameParallelExecutor.h"
#include "FrameParallelPolicy.h"
#include "HAL/IConsoleManager.h"
#include "HAL/PlatformTime.h"
#include <array>
#include <atomic>
#include <chrono>
#include <type_traits>
#include <condition_variable>
#include <exception>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <vector>

namespace Speed
{
namespace
{
constexpr int32 MaxLanes = FFrameParallelPolicy::MaximumLanes;
constexpr int32 QueueCapacity = (MaxLanes - 1) * 8;
thread_local bool GInsidePrivateFrameExecutor = false;
TAutoConsoleVariable<int32> CVarFrameExecutorBackend(
    TEXT("p.IAmSpeed.Parallel.WorkerBackend"), 0,
    TEXT("Configure before simulation:0=auto owned pool,1=explicit owned pool. Legacy backend2 is rejected. "
         "Capacity changes require a new simulation process; no global Engine pool resize."));

TAutoConsoleVariable<int32> CVarPrivateIdleSpinMicros1403(
    TEXT("p.IAmSpeed.Parallel.PrivateIdleSpinMicros1403"), 0,
    TEXT("Experimental native idle-wait policy:0=condition-variable sleep (default);1..1000=bounded microsecond polling after a job. No callbacks execute while idle."));
int32 PrivateIdleSpinMicros1403()
{
    return FMath::Clamp(CVarPrivateIdleSpinMicros1403.GetValueOnAnyThread(),0,1000);
}

struct FWorkGroup
{
    explicit FWorkGroup(int32 Count, TFunctionRef<void(int32)> InWork,
        FFrameDispatchObservation1075* InObservation = nullptr)
        : Work(InWork), Remaining(Count), Observation(InObservation) {}
    TFunctionRef<void(int32)> Work;
    std::array<std::exception_ptr, MaxLanes> Failures{};
    std::mutex Mutex;
    std::condition_variable Joined;
    int32 Remaining;
    FFrameDispatchObservation1075* Observation;
    void Execute(int32 Lane) noexcept
    {
        const uint64 Start = Observation ? FPlatformTime::Cycles64() : 0;
        const bool Previous = GInsidePrivateFrameExecutor;
        GInsidePrivateFrameExecutor = true;
        try { Work(Lane); } catch (...) { Failures[Lane] = std::current_exception(); }
        GInsidePrivateFrameExecutor = Previous;
        if (Observation) Observation->LaneCallbackCycles[Lane] = FPlatformTime::Cycles64() - Start;
        {
            std::lock_guard<std::mutex> Lock(Mutex);
            if (--Remaining == 0) Joined.notify_all();
        }
    }
    void WaitAndRethrow(int32 Lanes, bool bProducerFirst = false)
    {
        const uint64 Start = Observation ? FPlatformTime::Cycles64() : 0;
        {
            std::unique_lock<std::mutex> Lock(Mutex);
            Joined.wait(Lock, [&] { return Remaining == 0; });
        }
        if (Observation) Observation->JoinWaitCycles = FPlatformTime::Cycles64() - Start;
        if (bProducerFirst && Lanes == 2 && Failures[1])
            std::rethrow_exception(Failures[1]);
        for (int32 Lane = 0; Lane < Lanes; ++Lane)
            if (Failures[Lane]) std::rethrow_exception(Failures[Lane]);
    }
};
struct FJob { std::shared_ptr<FWorkGroup> Group; int32 Lane = 0; };
class FPrivateFramePool
{
public:
    explicit FPrivateFramePool(int32 Count)
    {
        Workers.reserve(Count);
        try
        {
            for (int32 Index = 0; Index < Count; ++Index)
                Workers.emplace_back([this] { Worker(); });
        }
        catch (...) { StopAndJoin(); throw; }
    }
    ~FPrivateFramePool() { StopAndJoin(); }
    int32 NumWorkers() const { return static_cast<int32>(Workers.size()); }
    bool EnqueueAll(const std::shared_ptr<FWorkGroup>& Group, int32 Lanes)
    {
        {
            std::lock_guard<std::mutex> Lock(Mutex);
            if (bStopping || QueueCapacity - Queued < Lanes - 1) return false;
            // Fixed storage makes admission atomic: no partial queue append can
            // throw and leave callbacks referencing a returned owner's stack.
            for (int32 Lane = 1; Lane < Lanes; ++Lane)
            {
                Queue[Tail].Group = Group; Queue[Tail].Lane = Lane;
                Tail = (Tail + 1) % QueueCapacity; ++Queued;
            }
            PendingHint1403.store(Queued,std::memory_order_release);
        }
        // Wake only the admitted non-caller lanes, rather than waking all11
        // idle workers for every small two/four-lane batch.
        for (int32 Lane = 1; Lane < Lanes; ++Lane) Ready.notify_one();
        return true;
    }
    void StopAndJoin() noexcept
    {
        { std::lock_guard<std::mutex> Lock(Mutex); bStopping = true; StopHint1403.store(true,std::memory_order_release); }
        Ready.notify_all();
        for (std::thread& Thread : Workers) if (Thread.joinable()) Thread.join();
    }
private:
    void WaitBrieflyForWork1403() noexcept
    {
        // This is only a wake-latency hint. The locked queue predicate below
        // remains authoritative, including on shutdown and a stale hint.
        const int32 Micros=PrivateIdleSpinMicros1403();
        if(Micros==0 || PendingHint1403.load(std::memory_order_acquire)>0 ||
            StopHint1403.load(std::memory_order_acquire))return;
        using FIdleClock=std::chrono::steady_clock;
        static_assert(std::is_integral<FIdleClock::rep>::value,
            "Idle wait clock must not perform floating-point arithmetic");
        const auto Deadline=FIdleClock::now()+std::chrono::microseconds(Micros);
        do
        {
            if(PendingHint1403.load(std::memory_order_acquire)>0 ||
                StopHint1403.load(std::memory_order_acquire))return;
            std::atomic_signal_fence(std::memory_order_seq_cst);
        }while(FIdleClock::now()<Deadline);
    }
    void Worker() noexcept
    {
        for (;;)
        {
            FJob Job;
            {
                std::unique_lock<std::mutex> Lock(Mutex);
                Ready.wait(Lock, [&] { return bStopping || Queued != 0; });
                if (Queued == 0 && bStopping) return;
                Job = std::move(Queue[Head]);
                Head = (Head + 1) % QueueCapacity; --Queued;
                PendingHint1403.store(Queued,std::memory_order_release);
            }
            Job.Group->Execute(Job.Lane);
            Job.Group.reset();
            WaitBrieflyForWork1403();
        }
    }
    std::mutex Mutex;
    std::condition_variable Ready;
    std::array<FJob, QueueCapacity> Queue{};
    int32 Head = 0, Tail = 0, Queued = 0;
    bool bStopping = false;
    // Hints never authorize dequeuing or callback execution without Mutex.
    alignas(64) std::atomic<int32> PendingHint1403{0};
    std::atomic<bool> StopHint1403{false};
    std::vector<std::thread> Workers;
};
struct FExecutorState
{
    std::mutex Mutex;
    std::condition_variable Idle;
    std::shared_ptr<FPrivateFramePool> Pool;
    int32 ActiveDispatches = 0;
    bool bStopping = false;
};
FExecutorState& State()
{
    // Explicit module shutdown joins threads before DLL unload. Avoid a CRT
    // static destructor trying to join threads while Windows holds loader lock.
    static FExecutorState* Instance = new FExecutorState;
    return *Instance;
}
std::shared_ptr<FPrivateFramePool> GetPoolLocked(FExecutorState& S, int32 Workers)
{
    if (S.bStopping) return {};
    if (!S.Pool) S.Pool = std::make_shared<FPrivateFramePool>(FMath::Clamp(Workers, 1, MaxLanes - 1));
    return S.Pool; // Never resize a live pool to satisfy an in-frame request.
}
struct FDispatchLease
{
    FExecutorState& S;
    std::shared_ptr<FPrivateFramePool> Pool;
    explicit FDispatchLease(FExecutorState& In) : S(In)
    {
        std::lock_guard<std::mutex> Lock(S.Mutex);
        Pool = GetPoolLocked(S, MaxLanes - 1);
        if (!Pool) throw std::runtime_error("IAmSpeed private executor is shutting down");
        ++S.ActiveDispatches;
    }
    ~FDispatchLease()
    {
        std::lock_guard<std::mutex> Lock(S.Mutex);
        if (--S.ActiveDispatches == 0) S.Idle.notify_all();
    }
};
}

int32 RequestedFrameExecutorBackend()
{
    static const int32 Backend = CVarFrameExecutorBackend.GetValueOnAnyThread();
    return Backend; // Startup setting; never switch a live frame's executor.
}
int32 EnsurePrivateFrameWorkers(int32 Workers)
{
    if (Workers < 1 || Workers >= MaxLanes) return 0;
    try
    {
        auto& S = State(); std::lock_guard<std::mutex> Lock(S.Mutex);
        auto Pool = GetPoolLocked(S, Workers); return Pool ? Pool->NumWorkers() : 0;
    }
    catch (...) { return 0; } // Owner policy reports unavailable capacity.
}
FFrameExecutorAvailability ConfigureFrameParallelExecutor(int32 HardwareLaneCapacity, int32 RequestedLanes)
{
    // Host policy, affinity and quotas bound capacity independently of Unreal.
    const int32 Backend = RequestedFrameExecutorBackend();
    if (Backend != 0 && Backend != 1)
        throw std::invalid_argument("IAmSpeed frame execution only supports its owned pool");
    (void)RequestedLanes; // Per-frame policy still chooses the admitted lane count.
    FFrameExecutorAvailability A;
    A.bPrivateSelected = true;
    const int32 Workers = FMath::Clamp(HardwareLaneCapacity - 1, 0, MaxLanes - 1);
    A.PrivateWorkers = Workers ? EnsurePrivateFrameWorkers(Workers) : 0;
    A.SelectedWorkers = A.PrivateWorkers;
    A.bReady = Workers == 0 || A.PrivateWorkers > 0;
    return A;
}
EFrameExecutorBackend DispatchPrivateFrameParallelLanes(int32 Lanes, TFunctionRef<void(int32)> Work,
    FFrameDispatchObservation1075* Observation)
{
    if (Lanes < 1 || Lanes > MaxLanes) throw std::invalid_argument("IAmSpeed lane count outside bounded capacity");
    if (Observation) *Observation = FFrameDispatchObservation1075{};
    const uint64 Start = Observation ? FPlatformTime::Cycles64() : 0;
    FDispatchLease Lease(State());
    auto Group = std::make_shared<FWorkGroup>(Lanes, Work, Observation);
    const bool Nested = GInsidePrivateFrameExecutor;
    const bool Queued = !Nested && Lease.Pool->EnqueueAll(Group, Lanes);
    if (Observation)
    {
        Observation->bAdmissionCompleted = true;
        Observation->AdmissionCycles = FPlatformTime::Cycles64() - Start;
        Observation->Fallback = Queued ? EFrameDispatchFallback1075::None :
            Nested ? EFrameDispatchFallback1075::Nested : EFrameDispatchFallback1075::QueueCapacity;
    }
    const uint64 CallerStart = Observation ? FPlatformTime::Cycles64() : 0;
    if (Queued) Group->Execute(0);
    else for (int32 Lane = 0; Lane < Lanes; ++Lane) Group->Execute(Lane);
    if (Observation) Observation->CallerExecutionCycles = FPlatformTime::Cycles64() - CallerStart;
    Group->WaitAndRethrow(Lanes);
    return Queued ? EFrameExecutorBackend::PrivatePool : EFrameExecutorBackend::SerialQueueFallback;
}
bool TryDispatchPrivateFrameParallelLanes1075(int32 Lanes, TFunctionRef<void(int32)> Work,
    FFrameDispatchObservation1075* Observation)
{
    if (Lanes < 1 || Lanes > MaxLanes) throw std::invalid_argument("IAmSpeed lane count outside bounded capacity");
    if (Observation) *Observation = FFrameDispatchObservation1075{};
    const uint64 Start = Observation ? FPlatformTime::Cycles64() : 0;
    FDispatchLease Lease(State());
    auto Group = std::make_shared<FWorkGroup>(Lanes, Work, Observation);
    const bool Nested = GInsidePrivateFrameExecutor;
    const bool Queued = !Nested && Lease.Pool->EnqueueAll(Group, Lanes);
    if (Observation)
    {
        Observation->bAdmissionCompleted = true;
        Observation->AdmissionCycles = FPlatformTime::Cycles64() - Start;
        Observation->Fallback = Queued ? EFrameDispatchFallback1075::None :
            Nested ? EFrameDispatchFallback1075::Nested : EFrameDispatchFallback1075::QueueCapacity;
    }
    if (!Queued) return false; // EnqueueAll accepts every worker task or none.
    const uint64 CallerStart = Observation ? FPlatformTime::Cycles64() : 0;
    Group->Execute(0);
    if (Observation) Observation->CallerExecutionCycles = FPlatformTime::Cycles64() - CallerStart;
    Group->WaitAndRethrow(Lanes);
    return true;
}
EFrameExecutorBackend DispatchFrameParallelLanes(int32 Lanes, TFunctionRef<void(int32)> Work,
    FFrameDispatchObservation1075* Observation)
{
    const int32 Backend = RequestedFrameExecutorBackend();
    if (Backend != 0 && Backend != 1)
        throw std::invalid_argument("IAmSpeed frame execution only supports its owned pool");
    return DispatchPrivateFrameParallelLanes(Lanes, Work, Observation);
}
EFrameExecutorBackend DispatchPrivateFramePair(
    TFunctionRef<void()> ProducerWork, TFunctionRef<void()> OwnerWork)
{
    if (GInsidePrivateFrameExecutor)
    {
        ProducerWork();
        OwnerWork();
        return EFrameExecutorBackend::SerialQueueFallback;
    }
    FDispatchLease Lease(State());
    const auto Work = [&](int32 Lane) { if (Lane == 0) OwnerWork(); else ProducerWork(); };
    auto Group = std::make_shared<FWorkGroup>(2, Work);
    if (!Lease.Pool->EnqueueAll(Group, 2))
    {
        ProducerWork();
        OwnerWork();
        return EFrameExecutorBackend::SerialQueueFallback;
    }
    Group->Execute(0);
    Group->WaitAndRethrow(2, true);
    return EFrameExecutorBackend::PrivatePool;
}
void ShutdownFrameParallelExecutor()
{
    auto& S = State(); std::shared_ptr<FPrivateFramePool> Pool;
    {
        std::unique_lock<std::mutex> Lock(S.Mutex); S.bStopping = true;
        S.Idle.wait(Lock, [&] { return S.ActiveDispatches == 0; });
        Pool = std::move(S.Pool);
    }
    if (Pool) Pool->StopAndJoin();
}
}
