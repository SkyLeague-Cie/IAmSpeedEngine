#pragma once
#include "IAmSpeed/Base/Kinematic.h"
#include "IAmSpeed/World/Analytic/AnalyticWorldQuery.h"
namespace Speed::PreparedExecution
{
// Readback of values ALREADY computed by the real const UBox refiner.
// Not an operation, support/contact certificate, World query or response grant.
enum class ENativeBoxPlanarRefinementOutcome:uint8 {Incomplete,NoCorner,WitnessRejected,Hit};
struct FNativeBoxPlanarRefinementValues
{
    ENativeBoxPlanarRefinementOutcome Outcome=ENativeBoxPlanarRefinementOutcome::Incomplete;
    bool Computed=false,Established=false,InitialEvent=false;
    // Actual already-executed native finite ray, never a replacement witness.
    bool WitnessComputed=false;Analytic::FWorldQuery WitnessQuery;Analytic::FWorldHit Witness;
    FKinematicState Initial,StartPose;
    FVector LocalCOM=FVector::ZeroVector,LocalCenter=FVector::ZeroVector,Extent=FVector::ZeroVector;
    FQuat LocalRotation=FQuat::Identity;
    FVector Normal=FVector::ZeroVector,Point=FVector::ZeroVector;
    double InitialGaps[8]={},InitialNormalSpeeds[8]={};
    double MinimumInitial=0,Roundoff=0,SpeedRoundoff=0;
    uint8 EstablishedMask=0;int32 FirstCorner=INDEX_NONE;
    float Horizon=0,FirstTime=0; // NoCorner FirstTime is the native initialization H, NOT a TOI.
};
}
