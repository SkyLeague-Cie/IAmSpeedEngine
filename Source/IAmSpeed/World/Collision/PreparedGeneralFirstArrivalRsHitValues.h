#pragma once
#include "PreparedGeneralFirstArrivalRsResponseQuality.h"
#include "IAmSpeed/Base/SHitResult.h"
namespace Speed::PreparedExecution
{
// Pure values: ALL retained Rs contact leaves must agree on the complete
// source/surface/feature/primitive/group/material tuple. No physical permission.
struct FGeneralFirstArrivalRsHitValues
{
    FPreparedGeometrySelectionPtr Geometry;
    FGeneralFirstArrivalRsResponseQualityPtr Quality;
    SHitResult Hit;
};
IAMSPEED_API bool BuildGeneralFirstArrivalRsHitValues(FPreparedGeometrySelectionPtr Geometry,
    FGeneralFirstArrivalRsResponseQualityPtr Quality,uint32 Frame,
    Analytic::Certified::FDistanceBudget& Budget,FGeneralFirstArrivalRsHitValues& Out,FString& Reason);
}
