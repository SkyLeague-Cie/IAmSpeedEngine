#pragma once
#include "CoreMinimal.h"
class ISpeedComponent;class USWheelSubBody;class USphereSubBody;class USpeedWorldSubsystem;
struct FCollisionShape;struct FCollisionQueryParams;
namespace Speed {
struct SHitResult;class FPreparedCanonicalStaticQueryService;class FPreparedWorldSnapshot;
namespace Analytic {enum class EStaticQuerySite:uint8;struct FWorldQuery;}
}
namespace Speed::PreparedExecution
{
class FStepScope;
enum class ENativeOwnerProbePhase:uint8 {Prepare,PostIntegrate,FinalProjection,FinalPost,ResponseResolve,IntegrationProjection};
// Actual lexical call identity ONLY, not compiled-operation/effect coverage.
// FullFrame preflight/Ready and all integration/resolve refusals stay closed.
class IAMSPEED_API FOrdinaryOwnerProbePhaseScope final
{
    friend class ::USpeedWorldSubsystem;friend class ::ISpeedComponent;friend class FNativeWheelProbeScope;friend class FStepScope;friend class FNativeOwnerPoseMutationScope;friend class FNativePlanarResponseCommitScope;
    friend class FNativeSphereProjectionScope;
    struct FState;TUniquePtr<FState> State;
    FOrdinaryOwnerProbePhaseScope(ENativeOwnerProbePhase,const ISpeedComponent&,float);
    bool Check() const;
public:
    ~FOrdinaryOwnerProbePhaseScope() noexcept;
    void Returned();
    FOrdinaryOwnerProbePhaseScope(const FOrdinaryOwnerProbePhaseScope&)=delete;
    FOrdinaryOwnerProbePhaseScope& operator=(const FOrdinaryOwnerProbePhaseScope&)=delete;
};
// Constructed by the TWO actual wheel probes before their UObject/getter reads.
// Synchronous native query values only; no escaping event/MTD/response ticket.
class IAMSPEED_API FNativeWheelProbeScope final
{
    friend class ::USWheelSubBody;
    struct FState;TUniquePtr<FState> State;
    FNativeWheelProbeScope(const USWheelSubBody&,float);
    bool Check() const;
public:
    ~FNativeWheelProbeScope() noexcept;
    bool IsActive() const {return bool(State);}
    bool Sweep(const FVector&,const FVector&,const FQuat&,const FCollisionShape&,
        const FCollisionQueryParams&,Analytic::EStaticQuerySite,SHitResult&);
    FNativeWheelProbeScope(const FNativeWheelProbeScope&)=delete;
    FNativeWheelProbeScope& operator=(const FNativeWheelProbeScope&)=delete;
};
// Lexical authorization for the actual inflated stationary sphere probe.
// It cannot escape the unchanged projection pass or authorize another query.
class IAMSPEED_API FNativeSphereProjectionScope final
{
    friend class ::USphereSubBody;
    friend class ::Speed::FPreparedCanonicalStaticQueryService;
    const USphereSubBody* Body=nullptr;
    const ISpeedComponent* Owner=nullptr;
    const FStepScope* Step=nullptr;
    const FOrdinaryOwnerProbePhaseScope* Phase=nullptr;
    const FPreparedWorldSnapshot* Snapshot=nullptr;
    FVector Center=FVector::ZeroVector;
    float Radius=0;
    int32 Pass=INDEX_NONE;
    FNativeSphereProjectionScope(const USphereSubBody&,const ISpeedComponent&,const FVector&,float,int32);
    bool Matches(const Analytic::FWorldQuery&,const FPreparedCanonicalStaticQueryService&) const;
public:
    bool IsActive() const {return Step&&Phase;}
    FNativeSphereProjectionScope(const FNativeSphereProjectionScope&)=delete;
    FNativeSphereProjectionScope& operator=(const FNativeSphereProjectionScope&)=delete;
};
}
