#pragma once
// Private native mutations; no exported issuer or restoration/source lease.
#include "IAmSpeed/Base/SHitResult.h"
#include "IAmSpeed/Base/Kinematic.h"
class USSubBody;class USpeedWheeledComponent;
class UBoxSubBody;
class ISpeedWheeledComponent;
class ISpeedComponent;
class USolidSubBody;
struct FFakePhysicsImpactContext;
class USpeedWorldSubsystem;
namespace Speed::Netcode::Prediction { class FPrivateRoadPhysicsWorld; class FPrivateNativeOperation; }
namespace Speed::NativeKinematics
{
class FBodyWrite final
{
    friend class ::UBoxSubBody;
    friend class ::USpeedWheeledComponent;
    friend class ::ISpeedWheeledComponent;
    friend class ::ISpeedComponent;
    friend class ::USolidSubBody;
    friend class ::USpeedWorldSubsystem;
    friend class Speed::Netcode::Prediction::FPrivateRoadPhysicsWorld;
    static void FakePhysics(USolidSubBody* Body,USolidSubBody* Other,const SHitResult& Hit,const float& DeltaTime,const FFakePhysicsImpactContext* ImpactContext);
    static void FakePhysics(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,USolidSubBody* Body,USolidSubBody* Other,const SHitResult& Hit,const float& DeltaTime,const FFakePhysicsImpactContext* ImpactContext);
    static void Initialize(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,USSubBody* Body,ISpeedComponent* Parent);
    static void AcceptHit(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,USSubBody* Body);
    static void UpdateFromOwner(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,USSubBody* Body,const SKinematic& ParentKinematic);
    static void Impulse(USolidSubBody& Body,const FVector& Impulse,const FVector& Point);
    static void Impulse(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,USolidSubBody& Body,const FVector& Impulse,const FVector& Point);
    static void Hit(USSubBody& Body,const SHitResult& Hit);
    static void Hit(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,USSubBody& Body,const SHitResult& Hit);
    static void FutureHit(USSubBody& Body,const SHitResult& Hit);
    static void FutureHit(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,USSubBody& Body,const SHitResult& Hit);
};
}
