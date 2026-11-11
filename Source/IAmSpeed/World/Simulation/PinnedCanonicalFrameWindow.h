#pragma once
#include "CoreMinimal.h"
#include "NativePreparationPurpose.h"
#include <atomic>
#include <cstring>
#if WITH_DEV_AUTOMATION_TESTS
class FCanonicalFrameWindowValueTest;
#endif
class ASpeedPhysicsSimulation;class USpeedWorldSubsystem;class FSimulationWorker;
namespace Speed {class FHostedGTStaticWorldLease;class IStaticCollisionWorld;class FPreparedWorldSnapshot;}
namespace Speed::OnePausedWork
{
class FNativeFrameStreamScope;
class FCanonicalFrameWindow;
using FCanonicalFrameWindowPtr=TSharedPtr<const FCanonicalFrameWindow,ESPMode::ThreadSafe>;
// Scheduling/lifetime WINDOW DESCRIPTOR, not a strong UObject pin or physical
// authority. The closed GT caller must keep its pins until matching park/join.
// Copies retain immutable native data only; Close is monotone and shared.
class FCanonicalFrameWindow final
{
    friend class Speed::FHostedGTStaticWorldLease;
    friend class ::FSimulationWorker;
    friend class FNativeFrameStreamScope;
#if WITH_DEV_AUTOMATION_TESTS
    friend class ::FCanonicalFrameWindowValueTest;
#endif
    static IAMSPEED_API uint64 AllocateSerialOnGT();
    static IAMSPEED_API uint64 AllocateSerialForNativeWork();
    TSharedPtr<const Speed::FPreparedWorldSnapshot,ESPMode::ThreadSafe> Snapshot;
    const void* World=nullptr;const ASpeedPhysicsSimulation* Host=nullptr;
    const USpeedWorldSubsystem* Bridge=nullptr;const Speed::IStaticCollisionWorld* Provider=nullptr;
    const FSimulationWorker* Worker=nullptr;
    uint64 Generation=0,Frame=0,StepBefore=0,Serial=0;
    uint32 DeltaBits=0,TimeBits=0;
    const EPurpose Purpose;
    const PreparationPrefix::FContractPtr PreparationContract;
    mutable std::atomic<bool> Open{true};
    mutable std::atomic<uint64> Nonce{0};
    FCanonicalFrameWindow(TSharedPtr<const Speed::FPreparedWorldSnapshot,ESPMode::ThreadSafe> snapshot,
        const void* world,const ASpeedPhysicsSimulation* host,const USpeedWorldSubsystem* bridge,
        const Speed::IStaticCollisionWorld* provider,const FSimulationWorker* worker,
        uint64 generation,uint64 frame,uint64 step,uint64 serial,float dt,float time,
        EPurpose purpose=EPurpose::FullFrame,PreparationPrefix::FContractPtr contract={})
        :Snapshot(MoveTemp(snapshot)),World(world),Host(host),Bridge(bridge),Provider(provider),Worker(worker),
         Generation(generation),Frame(frame),StepBefore(step),Serial(serial),Purpose(purpose),PreparationContract(MoveTemp(contract))
    {std::memcpy(&DeltaBits,&dt,4);std::memcpy(&TimeBits,&time,4);}
    bool BindNonce(uint64 nonce) const noexcept
    {uint64 empty=0;return nonce&&IsOpen()&&Nonce.compare_exchange_strong(empty,nonce);}
public:
    EPurpose GetPurpose() const noexcept {return Purpose;}
    const auto& GetPreparationContract() const {return PreparationContract;}
    IAMSPEED_API bool HasRequiredProviderReadiness(const Speed::IStaticCollisionWorld*) const noexcept;
    bool IsOpen() const noexcept {return Open.load();}
    void Close() const noexcept {Open.store(false);}
    const void* GetWorldIdentity() const noexcept {return World;}
    uint64 GetSerial() const noexcept {return Serial;}
    uint64 GetNonce() const noexcept {return Nonce.load();}
    uint64 GetStepBefore() const noexcept {return StepBefore;}
    uint64 GetExpectedStep() const noexcept {return StepBefore+1;}
    const auto& GetSnapshot() const {return Snapshot;}
    bool Matches(const ASpeedPhysicsSimulation* host,const USpeedWorldSubsystem* bridge,
        const Speed::IStaticCollisionWorld* provider,const FSimulationWorker* worker,uint64 generation,
        uint64 frame,float dt,float time) const noexcept
    {
        uint32 d=0,t=0;std::memcpy(&d,&dt,4);std::memcpy(&t,&time,4);
        return IsOpen()&&Snapshot&&World&&Host==host&&Bridge==bridge&&Provider==provider&&Worker==worker&&
            Generation==generation&&Frame==frame&&DeltaBits==d&&TimeBits==t&&Serial&&StepBefore<MAX_uint64;
    }
};
}
