#pragma once
#include "AnalyticWorldQuery.h"
#include "TorusSurface.h"

namespace Speed::Analytic
{
IAMSPEED_API bool TorusPassesFilter(const FWorldQuery& Query,const FTorusSurface& Torus);
IAMSPEED_API FWorldHit SweepTorusSurface(const FWorldQuery& Query,const FTorusSurface& Torus);
IAMSPEED_API FWorldHit SweepTorusTrajectory(const FWorldQuery& Query,const FTorusSurface& Torus,
    TFunctionRef<FTransform(double)> WorldPoseAt,double MotionBoundCm);
}
