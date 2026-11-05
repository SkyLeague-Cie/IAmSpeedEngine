#pragma once
#include "CoreMinimal.h"
// Private original solid mass accesses. No public issuer or restore lease.
class USolidSubBody;
class UBoxSubBody;
class USphereSubBody;
class USWheelSubBody;
class USpeedWheeledComponent;
namespace Speed::Netcode::Prediction { class FPrivateRoadPhysicsWorld; class FPrivateNativeOperation; }
namespace Speed::NativeKinematics
{
#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
// Copied observations from a completed diagnostic call, never an access issuer
// or a certificate of all-state/source/lifetime coverage.
struct FCanonicalMassAppendDiagnostics
{
    float CanonicalMass=0.f,FacadeMass=0.f;
    int32 MassOffset=INDEX_NONE,MassBytes=0,EncodedBytes=0,OriginalEncodingBytes=0;
    bool SourceValidated=false,EncodedMassMatches=false;
    bool OriginalEncodingMatches=false,FacadeDistinct=false,ValuesFinite=false;
};
#endif
class FSolidMassAccess final
{
    friend class ::UBoxSubBody;
    friend class ::USphereSubBody;
    friend class ::USWheelSubBody;
    friend class ::USpeedWheeledComponent;
    friend class Speed::Netcode::Prediction::FPrivateRoadPhysicsWorld;
#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
    static FCanonicalMassAppendDiagnostics AppendCanonicalEpisodeForTesting(
        const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,
        const USolidSubBody& Body,TArray<uint8>& Out);
    static void AppendCanonicalSolidPrefixForTesting(
        const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,
        const USolidSubBody& Body,TArray<uint8>& Out,FCanonicalMassAppendDiagnostics& Observation);
#endif
    static float Read(const USolidSubBody& Body);
    static float Read(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const USolidSubBody& Body);
    static void Write(USolidSubBody& Body,float Value);
    static void Write(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,USolidSubBody& Body,float Value);
};
}
