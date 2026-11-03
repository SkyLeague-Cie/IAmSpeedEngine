#include "WheelBindingAccess.h"
#include "IAmSpeed/Netcode/client-prediction/PrivateNativeStorageMap.h"
#include "IAmSpeed/Netcode/client-prediction/PrivateRoadNativeObjects.h"
#include "IAmSpeed/SubBodies/Solid/SWheelSubBody.h"
#include "ChaosVehicleWheel.h"
#include "WheelSystem.h"
namespace Speed::NativeKinematics
{
const UChaosVehicleWheel* FWheelBindingAccess::Configuration(const USWheelSubBody& Body) { return Body.ChaosWheelWheelDomainStorage(); }
const Chaos::FSimpleWheelSim* FWheelBindingAccess::Wheel(const USWheelSubBody& Body) { return Body.PWheelWheelDomainStorage(); }
const Chaos::FSimpleSuspensionSim* FWheelBindingAccess::Suspension(const USWheelSubBody& Body) { return Body.PSuspensionWheelDomainStorage(); }
bool FWheelBindingAccess::HasSuspension(const USWheelSubBody& Body) { return Body.PSuspensionWheelDomainStorage()!=nullptr; }
void FWheelBindingAccess::BindChaosWheel(USWheelSubBody& Body,UChaosVehicleWheel* Value) { Body.SetChaosWheelForPhysics(Value); }
void FWheelBindingAccess::BindWheel(USWheelSubBody& Body,Chaos::FSimpleWheelSim* Value) { Body.SetWheelSimForPhysics(Value); }
void FWheelBindingAccess::BindSuspension(USWheelSubBody& Body,Chaos::FSimpleSuspensionSim* Value) { Body.SetSuspensionSimForPhysics(Value); }

// Actual owner operation overloads. Legacy/default/custom entries above remain unchanged.
const UChaosVehicleWheel* FWheelBindingAccess::Configuration(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const USWheelSubBody& Body)
{ return Speed::Netcode::Prediction::FPrivateNativeStorageAccess::Part<const Speed::Netcode::Prediction::FPrivateRoadWheelDomain>(Operation,&Body).ChaosWheel; }
const Chaos::FSimpleWheelSim* FWheelBindingAccess::Wheel(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const USWheelSubBody& Body)
{ return Speed::Netcode::Prediction::FPrivateNativeStorageAccess::Part<const Speed::Netcode::Prediction::FPrivateRoadWheelDomain>(Operation,&Body).PWheel; }
const Chaos::FSimpleSuspensionSim* FWheelBindingAccess::Suspension(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const USWheelSubBody& Body)
{ return Speed::Netcode::Prediction::FPrivateNativeStorageAccess::Part<const Speed::Netcode::Prediction::FPrivateRoadWheelDomain>(Operation,&Body).PSuspension; }
bool FWheelBindingAccess::HasSuspension(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const USWheelSubBody& Body)
{ return Speed::Netcode::Prediction::FPrivateNativeStorageAccess::Part<const Speed::Netcode::Prediction::FPrivateRoadWheelDomain>(Operation,&Body).PSuspension!=nullptr; }
void FWheelBindingAccess::BindChaosWheel(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,USWheelSubBody& Body,UChaosVehicleWheel* Value)
{
    auto& Values=Speed::Netcode::Prediction::FPrivateNativeStorageAccess::Part<Speed::Netcode::Prediction::FPrivateRoadWheelDomain>(Operation,&Body);
    Values.ChaosWheel=Value;
    if(Values.ChaosWheel&&Values.SuspensionMaxRaiseOverride>=0.0f&&Values.SuspensionMaxDropOverride>=0.0f)
    {
        Values.ChaosWheel->SuspensionMaxRaise=Values.SuspensionMaxRaiseOverride;
        Values.ChaosWheel->SuspensionMaxDrop=Values.SuspensionMaxDropOverride;
    }
}
void FWheelBindingAccess::BindWheel(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,USWheelSubBody& Body,Chaos::FSimpleWheelSim* Value)
{
    auto& Values=Speed::Netcode::Prediction::FPrivateNativeStorageAccess::Part<Speed::Netcode::Prediction::FPrivateRoadWheelDomain>(Operation,&Body);
    // Acquire both actual backing parts before mutation; a missing part refuses.
    auto& Solid=Speed::Netcode::Prediction::FPrivateNativeStorageAccess::Part<Speed::Netcode::Prediction::FPrivateRoadSolidValues>(Operation,&Body);
    Values.PWheel=Value;
    if(!Values.PWheel)return;
    Values.PWheel->SetMassPerWheel(Values.WheelMass);
    Solid.Mass=Values.WheelMass;
}
void FWheelBindingAccess::BindSuspension(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,USWheelSubBody& Body,Chaos::FSimpleSuspensionSim* Value)
{ Speed::Netcode::Prediction::FPrivateNativeStorageAccess::Part<Speed::Netcode::Prediction::FPrivateRoadWheelDomain>(Operation,&Body).PSuspension=Value; }

}
