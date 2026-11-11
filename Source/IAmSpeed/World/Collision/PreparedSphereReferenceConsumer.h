#pragma once
#include "PreparedSphereRequest.h"
#include "PreparedIntervalScene.h"
#include "IAmSpeed/World/Analytic/ReferenceSphereConsumer.h"
namespace Speed::PreparedExecution
{
// Pure mathematical wrapper on an explicitly selected model. It accepts
// values/policy, never manufactures a native request or physical authority.
struct FSelectedSphereReferenceEvaluation
{
    FPreparedGeometrySelectionPtr Geometry;Analytic::Certified::FReferenceSphereEvaluation Reference;
    bool UsesGeometry(const FPreparedGeometrySelectionPtr& expected) const
    {return expected&&expected->HasValidAliases()&&Geometry==expected&&Reference.Status!=Analytic::Certified::EReferenceSphereStatus::Unknown&&
        Reference.Candidates&&Reference.Candidates->GetScene()==expected->GetSelectedUnion();}
};
inline FSelectedSphereReferenceEvaluation EvaluateSelectedSphereReference(FPreparedGeometrySelectionPtr geometry,
    FVector3d position,FVector3d velocity,FVector3d acceleration,float horizon,double collisionRadius,double physicalRadius,
    const Analytic::Certified::FReferenceSphereOptions& options,Analytic::Certified::FDistanceBudget& budget,TFunctionRef<bool(int32)> accepts)
{
    if(!geometry||!geometry->HasValidAliases())return {};
    auto result=Analytic::Certified::EvaluateReferenceSphereValues(geometry->GetSelectedUnion(),position,velocity,acceleration,
        horizon,collisionRadius,physicalRadius,options,budget,accepts);
    if(result.Status==Analytic::Certified::EReferenceSphereStatus::Unknown||!result.Candidates||
        result.Candidates->GetScene()!=geometry->GetSelectedUnion())return {};
    return {MoveTemp(geometry),MoveTemp(result)};
}
// The SAME native predicate is used by both reference and quality adapters.
// A complete owned bitmap records policy values; no callback identity escapes.
inline bool AcceptPreparedSphereOwner(const Analytic::Prepared::FOwner& o,const Analytic::FWorldQuery& q)
{
    if(!o.QueryEnabled||(o.CompactOptIn&&!q.bIncludeCompactPatches)||(q.bAuthorityOnly&&!o.AuthorityEligible)||
        (q.bExcludeAuthorityEligible&&o.AuthorityEligible)||(q.RequiredSourceId&&q.RequiredSourceId!=o.SourceId)||
        (q.RequiredSurfaceId&&q.RequiredSurfaceId!=o.SurfaceId)||(q.RequiredCanonicalGroupId&&q.RequiredCanonicalGroupId!=o.CanonicalGroupId))return false;
    if(!q.bApplyCollisionFilter)return true;if(o.ObjectType>=64)return false;
    const uint64 mask=uint64(1)<<o.ObjectType;
    return q.bObjectQuery?(q.ObjectTypes&mask)!=0:q.TraceChannel<64&&(q.BlockingObjectTypes&mask)!=0&&(o.BlockingChannels&(uint64(1)<<q.TraceChannel))!=0;
}
inline bool BuildPreparedSphereAcceptanceBitmap(TConstArrayView<Analytic::Prepared::FOwner> owners,const Analytic::FWorldQuery& q,
    Analytic::Certified::FDistanceBudget& budget,TArray<uint8>& output)
{
    output.Reset();if(owners.IsEmpty()||owners.Num()>budget.Remaining||uint64(owners.Num())>MAX_uint64-budget.Used)return false;
    // Debit the complete bounded traversal BEFORE allocating its owned result.
    for(int32 i=0;i<owners.Num();++i)if(!budget.Consume())return false;
    TArray<uint8> result;result.Reserve(owners.Num());for(const auto& owner:owners)result.Add(AcceptPreparedSphereOwner(owner,q)?1:0);
    output=MoveTemp(result);return true;
}
struct FNativeSphereReferenceEvaluation
{
    FNativeSphereRequestPtr Request;FPreparedGeometrySelectionPtr Geometry;
    Analytic::Prepared::FScenePtr Scene;
    Analytic::Certified::FReferenceSphereEvaluation Reference;
};
// Authentic immutable request wrapper. Both validations must hold in the same
// live Step/query epoch. The retained complete scene owns ALL leaf metadata,
// source records and original finite-plane domain indices. A representative
// scalar leaf never implies unique ownership. This result grants no advance,
// response, SHitResult, authority eligibility or physics readiness.
inline FNativeSphereReferenceEvaluation EvaluateNativeSphereReference(
    FNativeSphereRequestPtr request,const Analytic::Certified::FReferenceSphereOptions& options,
    Analytic::Certified::FDistanceBudget& budget)
{
    using namespace Analytic::Certified;
    if(!request||!RevalidateSphereCCD(*request))return {};
    const auto geometry=request->GetGeometrySelection();
    const auto scene=request->GetScene();if(!scene||!geometry||!geometry->HasValidAliases()||geometry->GetNominalScene()!=scene)return {};
    const auto& values=request->GetValues();TArray<uint8> accepted;
    if(!BuildPreparedSphereAcceptanceBitmap(scene->GetOwners(),values.Filter,budget,accepted))return {};
    auto accepts=[&](int32 index){return accepted.IsValidIndex(index)&&accepted[index]==1;};
    auto selected=EvaluateSelectedSphereReference(geometry,values.BodyState.Location,values.BodyState.Velocity,
        values.BodyState.Acceleration,values.Horizon,double(values.CollisionRadius),double(values.PhysicalRadius),options,budget,accepts);
    if(!selected.UsesGeometry(geometry)||!RevalidateSphereCCD(*request))return {};
    FNativeSphereReferenceEvaluation result;result.Request=MoveTemp(request);result.Geometry=geometry;result.Scene=scene;result.Reference=MoveTemp(selected.Reference);return result;
}
}
