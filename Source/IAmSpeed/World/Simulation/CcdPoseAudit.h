#pragma once
#include "CoreMinimal.h"
#if WITH_DEV_AUTOMATION_TESTS
class USpeedWorldSubsystem;
class ISpeedComponent;
namespace Speed::Testing
{
// Development-only observation; never canonical state or a collision authority.
// These are actual solver endpoints, not a certificate between endpoints.
enum class ECcdPoseAuditStage : uint8
{
    FrameBegin, AfterIntegrate, BeforeResolve, AfterResolve, AfterProjection, AfterPost,
    SphereSweepCapture, BeforePrimitiveIntegrate, AfterPrimitiveIntegrate
};
// Values copied synchronously at the actual sweep/primitive integration sites.
// Identity pointers are opaque equality tokens, never dereferenced from a saved
// record. The fixture must hold its own owner lifetime and bound its storage.
struct FCcdLinearStateValue
{
    FVector3d Position=FVector3d::ZeroVector,Velocity=FVector3d::ZeroVector,Acceleration=FVector3d::ZeroVector;
};
struct FCcdPrimitiveCapture
{
    const void* OwnerIdentity=nullptr;
    const void* BodyIdentity=nullptr;
    uint64 RunGeneration=0,Serial=0;
    uint32 OwnerFrame=0;
    FCcdLinearStateValue Owner,SubBody;
    FVector3d LocalBodyOffset=FVector3d::ZeroVector,LocalCenterOfMass=FVector3d::ZeroVector;
    FVector3d QueryStart=FVector3d::ZeroVector,QueryEnd=FVector3d::ZeroVector;
    double EffectiveSphereRadius=0;
    float Delta=0;
    bool WorldWasObserved=false,HasWorld=false,IsSphereSubBody=false,IsWheelSubBody=false;
    uint64 NumericControlFingerprint=0;bool NumericContextSupported=false;
};
struct FCcdPoseAuditSample
{
    const USpeedWorldSubsystem* World;
    uint32 Frame;
    ECcdPoseAuditStage Stage;
    float TimePassed, SubDelta;
    // Non-null only for the explicitly enabled primitive stages. Valid for this
    // synchronous callback ONLY: copy the value, never retain this pointer.
    const FCcdPrimitiveCapture* Primitive=nullptr;
};
using FCcdPoseAuditObserver = void (*)(void*, const FCcdPoseAuditSample&) noexcept;
IAMSPEED_API bool SetCcdPoseAuditObserverForCurrentThread(FCcdPoseAuditObserver Observer, void* Context) noexcept;
IAMSPEED_API bool ClearCcdPoseAuditObserverForCurrentThread(FCcdPoseAuditObserver Observer, void* Context) noexcept;
// Opt-in on the SAME existing TLS observer. Disabled on registration/clear;
// ordinary CcdPoseAudit consumers continue receiving only the old six stages.
IAMSPEED_API bool EnableCcdPrimitiveCaptureForCurrentThread(FCcdPoseAuditObserver Observer,void* Context,const ISpeedComponent* OwnerIdentity) noexcept;
IAMSPEED_API bool IsCcdPrimitiveCaptureEnabledForCurrentThread(const ISpeedComponent* OwnerIdentity) noexcept;
struct FCcdPrimitiveStamp { uint64 RunGeneration=0,Serial=0; };
IAMSPEED_API FCcdPrimitiveStamp NextCcdPrimitiveCaptureStampForCurrentThread() noexcept;
IAMSPEED_API bool CcdPrimitiveCaptureHadViolationForCurrentThread() noexcept;
IAMSPEED_API void MarkCcdPrimitiveCaptureViolationForCurrentThread() noexcept;
// Callback contract: values-only, bounded, no throwing or mutation of physics,
// floating controls, or observer registration. Violations never restore physics.
inline bool CcdLinearValuesHaveEqualBits(const FCcdLinearStateValue& a,const FCcdLinearStateValue& b) noexcept
{
    return FMemory::Memcmp(&a.Position,&b.Position,sizeof(a.Position))==0 &&
        FMemory::Memcmp(&a.Velocity,&b.Velocity,sizeof(a.Velocity))==0 &&
        FMemory::Memcmp(&a.Acceleration,&b.Acceleration,sizeof(a.Acceleration))==0;
}
IAMSPEED_API void ObserveCcdPrimitiveForCurrentThread(ECcdPoseAuditStage Stage,FCcdPrimitiveCapture Sample) noexcept;

}
#endif
