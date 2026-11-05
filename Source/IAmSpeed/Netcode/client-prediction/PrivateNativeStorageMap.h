#pragma once
// Private retained native backing lookup, not a snapshot, lease or coverage mask.
#include "PrivateNativeOperation.h"
#include <type_traits>
#include <unordered_map>
#include <functional>

class ISpeedComponent;class ISpeedWheeledComponent;
class USpeedMovementComponent;class USpeedWheeledComponent;class ASpeedPhysicsSimulation;class USpeedWorldSubsystem;
class USSubBody;class USolidSubBody;class UBoxSubBody;class USphereSubBody;class USWheelSubBody;
class USpeedBranchBoxSubBody;class USpeedBranchWheelSubBody;
namespace Speed { class FSimulationWorld; }
namespace Speed::NativeKinematics
{
class FBodyRead;class FBodyWrite;class FComponentRead;class FComponentWrite;
class FWheelBindingAccess;class FSolidMassAccess;class FSolidPropertiesAccess;
}
namespace Speed::Netcode::Prediction
{
struct FPrivateNativeStorageUnavailable final {};
class FPrivateNativeStorageMap final
{
    friend class FPrivateRoadPhysicsWorld;
    friend class FPrivateNativeOperation;
    struct FSubjectType
    {
        const void* Subject=nullptr;
        const void* ActualType=nullptr;
        bool operator==(const FSubjectType&) const noexcept=default;
    };
    struct FSubjectTypeHash
    {
        size_t operator()(const FSubjectType& Key) const noexcept
        {
            const auto A=std::hash<const void*>{}(Key.Subject);
            const auto B=std::hash<const void*>{}(Key.ActualType);
            return A^(B+size_t(0x9e3779b9u)+(A<<6)+(A>>2));
        }
    };
    struct FSlot
    {
        std::shared_ptr<void> ActualOwner;
        void* ActualValue=nullptr;
    };
    const std::shared_ptr<const FPrivateNativeGraphIdentity> Identity;
    std::unordered_map<FSubjectType,FSlot,FSubjectTypeHash> Slots;
    bool BirthRegistrationClosed=false; // one-way mutation state, NOT completeness
    explicit FPrivateNativeStorageMap(std::shared_ptr<const FPrivateNativeGraphIdentity> ActualIdentity)
        :Identity(std::move(ActualIdentity))
    {if(!Identity||!IsInGameThread())std::terminate();}
    template<class T> static const void* TypeIdentity() noexcept
    {
        // A distinct writable C++ object per actual instantiated type, never
        // a caller type id, pointer arithmetic or fabricated ownership token.
        static unsigned char ActualTypeObject=0;
        return &ActualTypeObject;
    }
    template<class T> bool BindAtBirth(const FPrivateNativeOperation& Operation,
        const void* ActualSubject,const std::shared_ptr<T>& ActualPart)
    {
        static_assert(!std::is_const_v<T>);
        if(BirthRegistrationClosed||Operation.Kind!=FPrivateNativeOperation::EKind::Birth
            ||!Operation.HasExactRootAndGate(Identity)||Operation.Storage.get()!=this
            ||!ActualSubject||!ActualPart)return false;
        const auto Type=TypeIdentity<T>();
        const std::shared_ptr<void> Owner=ActualPart; // actual alias control block
        const FSubjectType Key{ActualSubject,Type};
        if(const auto It=Slots.find(Key);It!=Slots.end())
            return It->second.ActualValue==ActualPart.get()&&!It->second.ActualOwner.owner_before(Owner)
                &&!Owner.owner_before(It->second.ActualOwner); // same real alias, no ABA repair
        return Slots.emplace(Key,FSlot{Owner,ActualPart.get()}).second;
    }
    bool CloseBirthRegistration(const FPrivateNativeOperation& Operation)
    {
        if(BirthRegistrationClosed||Operation.Kind!=FPrivateNativeOperation::EKind::Birth
            ||!Operation.HasExactRootAndGate(Identity)||Operation.Storage.get()!=this)return false;
        BirthRegistrationClosed=true;return true; // actual table mutation ends only
    }
    template<class T> T* Find(const FPrivateNativeOperation& Operation,const void* ActualSubject) const noexcept
    {
        using TValue=std::remove_cv_t<T>;
        if(!Operation.HasExactRootAndGate(Identity)||Operation.Storage.get()!=this||!ActualSubject
            ||(!BirthRegistrationClosed&&Operation.Kind!=FPrivateNativeOperation::EKind::Birth))return nullptr;
        const auto Type=TypeIdentity<TValue>();
        const auto It=Slots.find(FSubjectType{ActualSubject,Type});
        return It==Slots.end()?nullptr:static_cast<T*>(It->second.ActualValue);
        // Unavailable/wrong type/graph stays unavailable, never zero state.
        // Actual bound T and its strong original alias owner remain together.
    }
    bool ReleaseAtRetirement(const FPrivateNativeOperation& Operation)
    {
        if(Operation.Kind!=FPrivateNativeOperation::EKind::Retirement
            ||!Operation.HasExactRootAndGate(Identity)||Operation.Storage.get()!=this)return false;
        Slots.clear();return true; // after actual callers/native destructor phases returned
    }
    FPrivateNativeStorageMap(const FPrivateNativeStorageMap&)=delete;
    FPrivateNativeStorageMap& operator=(const FPrivateNativeStorageMap&)=delete;
public:
    ~FPrivateNativeStorageMap()=default;
};
template<class T> T* FPrivateNativeOperation::TryNativePart(const void* ActualSubject) const noexcept
{
    return Storage?Storage->Find<T>(*this,ActualSubject):nullptr;
}
class FPrivateNativeStorageAccess final
{
    friend class FPrivateRoadPhysicsWorld;
    friend class ::ISpeedComponent;friend class ::ISpeedWheeledComponent;
    friend class ::USpeedMovementComponent;friend class ::USpeedWheeledComponent;
    friend class ::ASpeedPhysicsSimulation;friend class ::USpeedWorldSubsystem;
    friend class ::USSubBody;friend class ::USolidSubBody;friend class ::UBoxSubBody;
    friend class ::USphereSubBody;friend class ::USWheelSubBody;
    friend class ::USpeedBranchBoxSubBody;friend class ::USpeedBranchWheelSubBody;
    friend class Speed::FSimulationWorld;
    friend class Speed::NativeKinematics::FBodyRead;friend class Speed::NativeKinematics::FBodyWrite;
    friend class Speed::NativeKinematics::FComponentRead;friend class Speed::NativeKinematics::FComponentWrite;
    friend class Speed::NativeKinematics::FWheelBindingAccess;friend class Speed::NativeKinematics::FSolidMassAccess;
    friend class Speed::NativeKinematics::FSolidPropertiesAccess;
    static void RequireBirth(const FPrivateNativeOperation& Operation)
    {
        if(Operation.Kind!=FPrivateNativeOperation::EKind::Birth
            ||!Operation.HasExactRootAndGate(Operation.Identity))throw FPrivateNativeStorageUnavailable{};
    }
    static void RequireBirthRoot(const FPrivateNativeOperation& Operation,const std::shared_ptr<void>& ActualRoot)
    {
        RequireBirth(Operation);
        if(!ActualRoot||Operation.Root.get()!=ActualRoot.get()
            ||Operation.Root.owner_before(ActualRoot)||ActualRoot.owner_before(Operation.Root))
            throw FPrivateNativeStorageUnavailable{};
    }
    static void RequireStep(const FPrivateNativeOperation& Operation)
    {
        if(Operation.Kind!=FPrivateNativeOperation::EKind::Step
            ||!Operation.HasExactRootAndGate(Operation.Identity))throw FPrivateNativeStorageUnavailable{};
    }
    template<class T> static T& Part(const FPrivateNativeOperation& Operation,const void* ActualSubject)
    {
        if(auto* Value=Operation.TryNativePart<T>(ActualSubject))return *Value;
        throw FPrivateNativeStorageUnavailable{};
    }
};
}
