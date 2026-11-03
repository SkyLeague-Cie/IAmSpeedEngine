#pragma once
// Private native mutations; no exported issuer or restoration/source lease.
#include "IAmSpeed/Base/SHitResult.h"
class USSubBody;
class ISpeedWheeledComponent;
class USolidSubBody;
struct FFakePhysicsImpactContext;
class USpeedWorldSubsystem;
namespace Speed::Netcode::Prediction { class FPrivateRoadPhysicsWorld; class FPrivateNativeOperation; }
namespace Speed::NativeKinematics
{
class FBodyWrite final
{
    friend class ::ISpeedWheeledComponent;
    friend class ::USolidSubBody;
    friend class ::USpeedWorldSubsystem;
    friend class Speed::Netcode::Prediction::FPrivateRoadPhysicsWorld;
    static void FakePhysics(USolidSubBody* Body,USolidSubBody* Other,const SHitResult& Hit,const float& DeltaTime,const FFakePhysicsImpactContext* ImpactContext);
    static void FakePhysics(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,USolidSubBody* Body,USolidSubBody* Other,const SHitResult& Hit,const float& DeltaTime,const FFakePhysicsImpactContext* ImpactContext);
    static void Impulse(USolidSubBody& Body,const FVector& Impulse,const FVector& Point);
    static void Impulse(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,USolidSubBody& Body,const FVector& Impulse,const FVector& Point);
    static void Hit(USSubBody& Body,const SHitResult& Hit);
    static void Hit(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,USSubBody& Body,const SHitResult& Hit);
    static void FutureHit(USSubBody& Body,const SHitResult& Hit);
    static void FutureHit(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,USSubBody& Body,const SHitResult& Hit);
};
}
