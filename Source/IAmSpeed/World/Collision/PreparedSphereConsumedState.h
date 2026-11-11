#pragma once
#include "PreparedSphereRequest.h"
#include "PreparedSphereResponseQuality.h"
#include "IAmSpeed/Base/SHitResult.h"
#include "IAmSpeed/World/Simulation/SubBodyKinematicsTasks.h"
namespace Speed::PreparedExecution
{
class FSealedSphereSweepState;
using FSealedSphereSweepStatePtr=TSharedPtr<const FSealedSphereSweepState,ESPMode::ThreadSafe>;
// Derived immutable capture, minted while the genuine member request is live.
// Possession never revives that expired request or permits native integration.
class FSealedSphereSweepState final
{
    friend class FSphereMemberSweepScope;friend class FComponentSweepScope;friend class FStepScope;
#if !UE_BUILD_SHIPPING
    friend IAMSPEED_API void AppendPreparedGroundEvidenceState(TArray<uint8>&,const TSharedPtr<const FGroundSweepEvidence,ESPMode::ThreadSafe>&);
#endif
    FStepIdentity Step;FSphereValues Values;FPreparedGeometrySelectionPtr Geometry;
    TSharedPtr<const FPreparedWorldSnapshot,ESPMode::ThreadSafe> Snapshot;
    Analytic::Certified::FZeroOffsetKinematicCenterReference Reference;
    FKinematicState OwnerHistoryAtFrame;FrameParallel::FSubBodyKinematicInput NativeLocalInput;
    SHitResult NamedGround;FSelectedSphereResponseQualityPtr Quality;
    bool GroundHit=false,WholeHullExclusion=false,NativeLocalInputSupported=false;
    float GroundSeconds=0;
    FSealedSphereSweepState()=default;
public:
    const FSphereValues& GetCapturedValuesForInspection() const {return Values;}
    const FStepIdentity& GetOriginForInspection() const {return Step;}
};
// Pure mathematical expectation for the unchanged, single free integration
// primitive and child sync. Not proof of virtual setters, callbacks, transport,
// projection, sleep, clamps, actual historical routing or native execution.
struct FSphereFreeAdvanceValues
{
    FKinematicState OwnerBefore,BodyBefore;FVector LocalCOM=FVector::ZeroVector;
    FrameParallel::FSubBodyKinematicInput Local;
    float Seconds=0,MaximumSpeed=0,MaximumAngularSpeed=0;
};
struct FSphereFreeAdvanceExpectation
{
    // OriginAfter is applicable only when PerformsIntegration is true.
    bool Known=false,PerformsIntegration=false;
    FKinematicState OwnerAfter,OriginAfter,BodyAfter;float Seconds=0;
};
IAMSPEED_API bool BuildSphereFreeAdvanceExpectation(const FSphereFreeAdvanceValues& Values,
    FSphereFreeAdvanceExpectation& Out,FString& Reason);
IAMSPEED_API bool SameSphereFreeAdvanceValues(const FSphereFreeAdvanceValues& A,const FSphereFreeAdvanceValues& B) noexcept;
}
