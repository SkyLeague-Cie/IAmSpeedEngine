#pragma once
#include "CoreMinimal.h"
#include "IAmSpeed/Base/Kinematic.h"
class ISpeedComponent;class ISpeedWheeledComponent;class USpeedMovementComponent;class USpeedWheeledComponent;
class UBoxSubBody;class USSubBody;class USWheelSubBody;
namespace Speed {struct SHitResult;}
namespace Speed::PreparedExecution
{
struct FOrdinaryIntegrationOwnerValues;
class FStepScope;class FPlanarChildRefreshValues;struct FOrdinaryResponseEntryValues;
using SHitResult=::Speed::SHitResult;
// Only the two production setter bodies can create/complete this lexical
// witness. Geometry alone never authorizes a pose assignment or a child sync.
class IAMSPEED_API FNativeOwnerPoseMutationScope final
{
    friend class ::USpeedMovementComponent;friend class ::USpeedWheeledComponent;
    struct FState;TUniquePtr<FState> State;
    FNativeOwnerPoseMutationScope(const ISpeedComponent&,const FKinematicState& ActualBefore,const FKinematicState& Candidate);
    FOrdinaryIntegrationOwnerValues& Require() const;
    void Returned();
public:
    // Lexical witness only for the original wheel bump-stop projection caller.
    class IAMSPEED_API FFinalWheelSupport final
    {
        friend class ::ISpeedWheeledComponent;friend class FStepScope;
        friend class FNativeOwnerPoseMutationScope;friend class FNativePlanarResponseCommitScope;
        struct FState;
        TUniquePtr<FState> State;
        FFinalWheelSupport(const ISpeedComponent&,int32 pass);
        ~FFinalWheelSupport() noexcept;
        FFinalWheelSupport(const FFinalWheelSupport&)=delete;
        FFinalWheelSupport& operator=(const FFinalWheelSupport&)=delete;
        static FState& Require();
        static bool BeginSetter(const ISpeedComponent&,const FKinematicState&,const FKinematicState&);
        static void EndSetter(const ISpeedComponent&,const FKinematicState&);
        static bool BeforeChildSync(const ISpeedComponent&,const FKinematicState&);
        static bool AfterChildSync(const ISpeedComponent&,const FKinematicState&);
        static bool HasPendingSync() noexcept;
    };
    ~FNativeOwnerPoseMutationScope() noexcept;
    FNativeOwnerPoseMutationScope(const FNativeOwnerPoseMutationScope&)=delete;
    FNativeOwnerPoseMutationScope& operator=(const FNativeOwnerPoseMutationScope&)=delete;
    FNativeOwnerPoseMutationScope(FNativeOwnerPoseMutationScope&&)=delete;
    FNativeOwnerPoseMutationScope& operator=(FNativeOwnerPoseMutationScope&&)=delete;
};
using FNativeFinalWheelSupportScope=FNativeOwnerPoseMutationScope::FFinalWheelSupport;
// UNINSTALLED declaration fragment, intended in existing narrow pose header.
// Forward declarations required in the parent header: UBoxSubBody, USSubBody,
// USWheelSubBody and Speed::SHitResult, FStepScope/FOrdinaryResponseEntryValues.
class IAMSPEED_API FNativePlanarResponseCommitScope final
{
    friend class ::UBoxSubBody;friend class ::ISpeedComponent;
    friend class ::USSubBody;friend class ::USWheelSubBody;
    friend class FNativeOwnerPoseMutationScope;friend class FStepScope;
    struct FState;TUniquePtr<FState> State;
    FState& Require() const;
    static FPlanarChildRefreshValues& RequireRefresh();
    static void CloseStep(FStepScope&) noexcept;
    FNativePlanarResponseCommitScope(const ISpeedComponent&,const UBoxSubBody&,const SHitResult&,
        const FKinematicState& NativeBefore,const FKinematicState& LocalSolved);
    static bool BeforeOwnerSetter(const ISpeedComponent&,const FKinematicState&,const FKinematicState&);
    static bool AfterOwnerSetter(const ISpeedComponent&,const FKinematicState&);
    static void ObserveNativeClamp(const ISpeedComponent&,const FVector& ActualInput,double ActualCap,
        const FVector& ActualOutput,bool Angular);
    static void BeforeChildSync(const ISpeedComponent&,const FKinematicState& ActualOrigin);
    static bool AfterChildSync(const ISpeedComponent&,const FKinematicState& ActualOrigin);
    static bool BeforeNativeBaseChildWrite(const USSubBody&,const FKinematicState& ActualOrigin,
        const FKinematicState& ActualBefore,const FVector& ActualLocalOffset,const FQuat& ActualLocalRotation);
    static void ReadbackOrderedBaseChildWrite(const USSubBody&,const FKinematicState&,bool complete);
    static void BeforeNativeWheelSpringWrite(const USWheelSubBody&,const FKinematicState& ActualOrigin,
        const FKinematicState& ActualBase,float ActualSpring);
    static void RequireClosedResponseEffect(FStepScope&,const FOrdinaryResponseEntryValues&);
public:
    // Observation used by ALL query entry points. It grants no operation.
    static bool HasUnclosedBodySync() noexcept;
    static void RequireQueriesClosed();
    ~FNativePlanarResponseCommitScope() noexcept;
    FNativePlanarResponseCommitScope(const FNativePlanarResponseCommitScope&)=delete;
    FNativePlanarResponseCommitScope& operator=(const FNativePlanarResponseCommitScope&)=delete;
    FNativePlanarResponseCommitScope(FNativePlanarResponseCommitScope&&)=delete;
    FNativePlanarResponseCommitScope& operator=(FNativePlanarResponseCommitScope&&)=delete;
};

}
