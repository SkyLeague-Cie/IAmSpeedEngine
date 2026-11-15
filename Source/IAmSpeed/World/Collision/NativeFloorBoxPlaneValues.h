#pragma once
#include "PreparedNativeSolidGeometry.h"
#include "IAmSpeed/Base/SHitResult.h"
#include "NativeBoxPlanarRefinementValues.h"
namespace Speed::PreparedExecution::NativeFloorBoxPlaneDetail
{
// IAS-internal raw GEOMETRY values for the existing math unit. A descriptor
// has no hit flag or UObject token; only the actual Box member may refine it.
bool Build(const FNativeSolidStaticValues&,const FNativeSolidGeometryValues&,SHitResult&);
bool MatchesRefinement(const FNativeSolidStaticValues&,const FNativeSolidGeometryValues&,const SHitResult&);
bool MatchesReadbackInputs(const FNativeSolidStaticValues&,const FVector&,const FVector&,const FNativeBoxPlanarRefinementValues&);
bool MatchesInitialReadback(const FNativeSolidStaticValues&,const SHitResult&,const FNativeBoxPlanarRefinementValues&);
bool MatchesNoNewReadback(const FNativeSolidStaticValues&,const FNativeSolidGeometryValues&,const FNativeBoxPlanarRefinementValues&);
bool BuildSupportDescriptor(const FNativeSolidStaticValues&,const FNativeSolidGeometryValues&,SHitResult&);
}
