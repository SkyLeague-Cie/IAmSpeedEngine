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

}
