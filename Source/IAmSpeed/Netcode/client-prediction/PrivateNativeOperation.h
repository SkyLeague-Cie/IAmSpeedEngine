#pragma once
// Private operation-memory/root/mutex boundary only. NOT a source, lifecycle,
// worker/module join, escaped-view or complete-state restoration lease.
#include "CoreMinimal.h"
#include <atomic>
#include <exception>
#include <memory>
#include <mutex>

namespace Speed::Netcode::Prediction
{
class FPrivateRoadPhysicsWorld;
class FPrivateNativeOperation;
class FPrivateNativeStorageMap;
class FPrivateNativeStorageAccess;
class FPrivateNativeGraphIdentity final
{
    friend class FPrivateRoadPhysicsWorld;
    friend class FPrivateNativeOperation;
    const std::weak_ptr<void> Root;
    const std::mutex* const Mutex;
    mutable std::atomic<bool> Retired{false};
    // GT-only observation of the REAL currently owning stack operation.
    // Used only to REFUSE recursive acquisition; never grants storage access.
    mutable const FPrivateNativeOperation* ActiveGTActualOperation=nullptr;
    FPrivateNativeGraphIdentity(const std::shared_ptr<void>& ActualRoot,const std::mutex& ActualMutex)
        :Root(ActualRoot),Mutex(&ActualMutex)
    {
        if(!ActualRoot||!IsInGameThread())std::terminate();
    }
    FPrivateNativeGraphIdentity(const FPrivateNativeGraphIdentity&)=delete;
    FPrivateNativeGraphIdentity& operator=(const FPrivateNativeGraphIdentity&)=delete;
public:
    ~FPrivateNativeGraphIdentity()=default; // actual shared allocation destruction
};
class FPrivateNativeOperation final
{
    friend class FPrivateRoadPhysicsWorld;
    friend class FPrivateNativeStorageMap;
    friend class FPrivateNativeStorageAccess;
    enum class EKind : uint8 { Birth, Step, Activation, Retirement };
    // Declaration order is deliberate: the actual lock is destroyed BEFORE
    // the retained root. Deferred Close belongs to the OUTER real Call object,
    // after this entire operation and all native consumers have returned.
    const std::shared_ptr<void> Root;
    const std::shared_ptr<const FPrivateNativeGraphIdentity> Identity;
    const std::shared_ptr<FPrivateNativeStorageMap> Storage;
    const EKind Kind;
    std::unique_lock<std::mutex> Lock;
    FPrivateNativeOperation(std::shared_ptr<void> ActualRoot,
        std::shared_ptr<const FPrivateNativeGraphIdentity> ActualIdentity,
        std::shared_ptr<FPrivateNativeStorageMap> ActualStorage,EKind ActualKind,
        std::unique_lock<std::mutex>&& ActualLock)
        :Root(std::move(ActualRoot)),Identity(std::move(ActualIdentity)),Storage(std::move(ActualStorage)),Kind(ActualKind),Lock(std::move(ActualLock))
    {
        // Check the real mutex and exact allocation/control block, not a
        // caller-supplied generation integer or a fabricated ownership flag.
        if(!Storage||!HasActualRootAndOwnedMutex(Identity)||Identity->ActiveGTActualOperation)std::terminate();
        Identity->ActiveGTActualOperation=this; // only after actual successful lock transfer
    }
    bool HasActualRootAndOwnedMutex(const std::shared_ptr<const FPrivateNativeGraphIdentity>& Expected) const noexcept
    {
        if(!IsInGameThread()||!Root||!Identity||Identity!=Expected
            ||Identity->Retired.load()||!Lock.owns_lock()||Lock.mutex()!=Identity->Mutex)return false;
        const auto OriginalRoot=Identity->Root.lock();
        return OriginalRoot&&OriginalRoot.get()==Root.get()
            &&!OriginalRoot.owner_before(Root)&&!Root.owner_before(OriginalRoot);
    }
    bool HasExactRootAndGate(const std::shared_ptr<const FPrivateNativeGraphIdentity>& Expected) const noexcept
    {
        return HasActualRootAndOwnedMutex(Expected)&&Identity->ActiveGTActualOperation==this;
    }
    template<class T> T* TryNativePart(const void* ActualSubject) const noexcept;
    FPrivateNativeOperation(const FPrivateNativeOperation&)=delete;
    FPrivateNativeOperation& operator=(const FPrivateNativeOperation&)=delete;
    FPrivateNativeOperation(FPrivateNativeOperation&&)=delete;
    FPrivateNativeOperation& operator=(FPrivateNativeOperation&&)=delete;
    ~FPrivateNativeOperation()
    {
        // All native consumers and nested scopes have REALLY ended. Keep the
        // actual active-operation refusal installed through the actual unlock,
        // then clear it. Root remains strongly retained through both returns.
        // Retired may now be true after successful Close; that is intentional.
        if(!IsInGameThread()||Identity->ActiveGTActualOperation!=this
            ||!Lock.owns_lock()||Lock.mutex()!=Identity->Mutex)std::terminate();
        Lock.unlock();
        Identity->ActiveGTActualOperation=nullptr;
    }
};
}

#include "PrivateNativeStorageMap.h"
