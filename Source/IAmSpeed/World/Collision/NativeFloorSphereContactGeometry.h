#pragma once
#include "PreparedNativeSolidGeometry.h"
namespace Speed::PreparedExecution
{
struct FNativeFloorSphereContactGeometryValues
{
    // Contact geometry for SUPPORT Rs, including shallow captured overlap.
    // Its depth is not the historical rounded query Rc depth, nor a physical
    // Rp correction. Resolve/MTD/projection permissions are not carried here.
    float Seconds=0;uint32 OwnerFrame=0;Analytic::FWorldHit Hit;
};
namespace NativeFloorSphereContactDetail
{
// IAS-internal geometry helper for the existing math unit. No exported raw
// authority path: external consumers use the live opaque-result entry below.
bool Build(const FNativeSolidStaticValues&,const FNativeSolidGeometryValues&,
    FNativeFloorSphereContactGeometryValues&);
}
// Geometry values only; false is Unknown, never a miss. Implemented inside
// IAS so a consumer in another module never links the private SBox symbol.
// No native Contact/evidence/response/advance/readiness permission follows.
IAMSPEED_API bool BuildNativeFloorSphereContactGeometry(const FPreparedNativeSolidGeometryPtr&,
    FNativeFloorSphereContactGeometryValues&);
}
