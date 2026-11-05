#include "SolidMassAccess.h"
#include "IAmSpeed/Netcode/client-prediction/PrivateNativeStorageMap.h"
#include "IAmSpeed/Netcode/client-prediction/PrivateRoadNativeObjects.h"
#include "IAmSpeed/SubBodies/Solid/SolidSubBody.h"
namespace Speed::NativeKinematics
{
float FSolidMassAccess::Read(const USolidSubBody& Body) { return Body.MassSolidStorage(); }
void FSolidMassAccess::Write(USolidSubBody& Body,float Value) { Body.MassSolidStorage()=Value; }

// Actual owner operation overloads. Legacy/default/custom entries above remain unchanged.
float FSolidMassAccess::Read(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const USolidSubBody& Body)
{ return Speed::Netcode::Prediction::FPrivateNativeStorageAccess::Part<const Speed::Netcode::Prediction::FPrivateRoadSolidValues>(Operation,&Body).Mass; }
void FSolidMassAccess::Write(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,USolidSubBody& Body,float Value)
{ Speed::Netcode::Prediction::FPrivateNativeStorageAccess::Part<Speed::Netcode::Prediction::FPrivateRoadSolidValues>(Operation,&Body).Mass=Value; }

#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
void FSolidMassAccess::AppendCanonicalSolidPrefixForTesting(
    const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,
    const USolidSubBody& Body,TArray<uint8>& Out,FCanonicalMassAppendDiagnostics& Observation)
{
    // Called only by the fixed shape virtual paths after the entry validation.
    // Preserve the old prefix, mass position and own-tail getter order.
    Speed::Netcode::Prediction::FPrivateNativeStorageAccess::RequireStep(Operation);
    // The private prefix also validates its own subject/parts before any bytes,
    // even if an existing friend calls it directly without the staged entry.
    (void)Speed::Netcode::Prediction::FPrivateNativeStorageAccess::Part<const Speed::Netcode::Prediction::FPrivateRoadSubBodyValues>(Operation,&Body);
    (void)Speed::Netcode::Prediction::FPrivateNativeStorageAccess::Part<const Speed::Netcode::Prediction::FPrivateRoadSolidValues>(Operation,&Body);
    Body.USSubBody::AppendEpisodeStateForTesting(Out);
    Observation.CanonicalMass=Read(Operation,Body);
    Observation.SourceValidated=true;
    Observation.MassOffset=Out.Num();
    Speed::FEpisodeStateBytesForTesting::Add(Out,Observation.CanonicalMass);
    Observation.MassBytes=Out.Num()-Observation.MassOffset;
    Body.AppendSolidEpisodeTailForTesting(Out);
}

FCanonicalMassAppendDiagnostics FSolidMassAccess::AppendCanonicalEpisodeForTesting(
    const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,
    const USolidSubBody& Body,TArray<uint8>& Out)
{
    namespace P=Speed::Netcode::Prediction;
    P::FPrivateNativeStorageAccess::RequireStep(Operation);
    (void)P::FPrivateNativeStorageAccess::Part<const P::FPrivateRoadSolidValues>(Operation,&Body);
    // A failed or unsupported capture may discard its unpublished local bytes;
    // it cannot mutate caller output or roll back any physical state.
    TArray<uint8> Candidate;FCanonicalMassAppendDiagnostics Observation;
    Body.AppendCanonicalEpisodeStateForTesting(Operation,Candidate,Observation);
    if(!Observation.SourceValidated||Observation.MassOffset<0||Observation.MassBytes<=0
        ||Observation.MassOffset>Candidate.Num()||Observation.MassBytes>Candidate.Num()-Observation.MassOffset)
        throw P::FPrivateNativeStorageUnavailable{};
    TArray<uint8> NativeMassBytes;
    Speed::FEpisodeStateBytesForTesting::Add(NativeMassBytes,Observation.CanonicalMass);
    if(NativeMassBytes.Num()!=Observation.MassBytes)throw P::FPrivateNativeStorageUnavailable{};
    Observation.EncodedMassMatches=FMemory::Memcmp(Candidate.GetData()+Observation.MassOffset,
        NativeMassBytes.GetData(),NativeMassBytes.Num())==0; // scalar codec bytes, no struct padding
    Observation.EncodedBytes=Candidate.Num();
    TArray<uint8> Original;
    Body.AppendEpisodeStateForTesting(Original);
    Observation.OriginalEncodingBytes=Original.Num();
    Observation.OriginalEncodingMatches=Candidate==Original;
    Observation.FacadeMass=Body.GetMass();
    Observation.ValuesFinite=FMath::IsFinite(Observation.CanonicalMass)&&FMath::IsFinite(Observation.FacadeMass);
    Observation.FacadeDistinct=Observation.ValuesFinite&&Observation.CanonicalMass!=Observation.FacadeMass;
    Out.Append(Candidate);
    return Observation;
}
#endif

}
