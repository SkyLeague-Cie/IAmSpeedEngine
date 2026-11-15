#pragma once
#include "PreparedFirstArrivalRsResponseQuality.h"
#include "IAmSpeed/Base/SHitResult.h"
namespace Speed::PreparedExecution
{
// Pure values: ALL retained Rs contact leaves must agree on the complete
// source/surface/feature/primitive/group/material tuple. No physical permission.
struct FFirstArrivalRsHitValues
{
    FPreparedGeometrySelectionPtr Geometry;
    FFirstArrivalRsResponseQualityPtr Quality;
    SHitResult Hit;
};
IAMSPEED_API bool BuildFirstArrivalRsHitValues(FPreparedGeometrySelectionPtr Geometry,
    FFirstArrivalRsResponseQualityPtr Quality,uint32 Frame,
    Analytic::Certified::FDistanceBudget& Budget,FFirstArrivalRsHitValues& Out,FString& Reason);
}
