#pragma once
#include "PreparedSphereResponseQuality.h"
namespace Speed::PreparedExecution
{
// Values-only query inputs. This is deliberately not FSphereValues and cannot
// manufacture a native body/member/World capture for a test or caller.
struct FSphereResponseQueryValues
{
    FVector3d Position=FVector3d::ZeroVector,Velocity=FVector3d::ZeroVector,Acceleration=FVector3d::ZeroVector;
    float Horizon=0,CollisionRadius=0,PhysicalRadius=0,ResponseMargin=0,StaticContactOffsetCm=0;
    Analytic::Certified::FReferenceSphereOptions Options;Analytic::FWorldQuery Filter;
};
IAMSPEED_API bool PrepareSelectedSphereResponseQualityValues(FPreparedGeometrySelectionPtr geometry,const FSphereResponseQueryValues& query,
    Analytic::Certified::FDistanceBudget& budget,FSphereResponseQualityValues& output,FString& reason);
struct FNativeSphereResponseQualityEvaluation
{
    FNativeSphereRequestPtr Request;FPreparedGeometrySelectionPtr Geometry;
    FSelectedSphereResponseQualityPtr Quality;
};
// Actual request adapter, with before/after same member/Step/snapshot/margin
// revalidation. Mathematical quality ONLY; never a hit or advance/Ready ticket.
IAMSPEED_API FNativeSphereResponseQualityEvaluation EvaluateNativeSphereResponseQuality(
    FNativeSphereRequestPtr request,const Analytic::Certified::FReferenceSphereOptions& options,
    Analytic::Certified::FDistanceBudget& budget,FString& reason);
}
