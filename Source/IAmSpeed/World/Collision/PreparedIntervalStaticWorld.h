#pragma once
#include "PreparedIntervalScene.h"
#include "StaticCollisionWorld.h"
#include "PreparedPhysicalQueryDomain.h"
#include "PreparedWorldSnapshot.h"
#include "PreparedCanonicalStaticQueryService.h"
#include "PreparedNativeFrameConsumer.h"
#include <stdexcept>
namespace Speed
{
// Owns the complete input at a real World boundary. The typed-motion and
// response contracts are still deliberately CLOSED, so no frame may consume
// this as physical authority. Diagnostic access uses the owned input directly.
class FPreparedIntervalStaticWorld final:public IStaticCollisionWorld
{
    FPreparedWorldSnapshotPtr Snapshot;
    const FPreparedCanonicalStaticQueryServicePtr QueryService;
    const FPreparedNativeFrameConsumerPtr FrameConsumer;
    [[noreturn]] static void Unavailable()
    { throw std::runtime_error("Prepared interval World awaits its typed physical consumer; no miss or fallback"); }
    static FPreparedWorldSnapshotPtr MakeNominalSnapshot(Analytic::Prepared::FScenePtr input,FPreparedPhysicalQueryDomainPtr filters)
    {
        FPreparedGeometrySelectionPtr selection;FString reason;
        if(!FPreparedGeometrySelection::TryCreate(EPreparedGeometryModel::NominalInput,MoveTemp(input),{},selection,reason))
            throw std::runtime_error("Missing/mixed nominal prepared geometry");
        return MakeShareable(new FPreparedWorldSnapshot(MoveTemp(selection),MoveTemp(filters)));
    }
public:
    explicit FPreparedIntervalStaticWorld(Analytic::Prepared::FScenePtr input,FPreparedPhysicalQueryDomainPtr filters={})
        :Snapshot(MakeNominalSnapshot(MoveTemp(input),MoveTemp(filters)))
    {if(!Snapshot->GetNominalScene())throw std::runtime_error("Missing prepared interval scene");}
    explicit FPreparedIntervalStaticWorld(FPreparedWorldSnapshotPtr snapshot,FPreparedCanonicalStaticQueryServicePtr service={},
        FPreparedNativeFrameConsumerPtr consumer={})
        :Snapshot(MoveTemp(snapshot)),QueryService(MoveTemp(service)),FrameConsumer(MoveTemp(consumer))
    {
        if(!Snapshot||!Snapshot->GetGeometrySelection()||!Snapshot->GetGeometrySelection()->HasValidAliases()||!Snapshot->GetNominalScene()||!Snapshot->GetFilters()||
            (Snapshot->GetReference()&&Snapshot->GetReference()->GetNominalScene()!=Snapshot->GetNominalScene()))
            throw std::runtime_error("Incomplete/mixed prepared World snapshot");
        if(QueryService&&!QueryService->MatchesPreparedSnapshot(Snapshot))
            throw std::runtime_error("Prepared query service belongs to another snapshot");
        if(FrameConsumer&&!FrameConsumer->Matches(Snapshot,QueryService))
            throw std::runtime_error("Native frame consumer belongs to another snapshot/service or has no retained objects");
    }
    FPreparedWorldSnapshotPtr GetPreparedSnapshot() const override {return Snapshot;}
    FPreparedCanonicalStaticQueryServicePtr GetPreparedQueryService() const override {return QueryService;}
    FPreparedNativeFrameConsumerPtr GetPreparedNativeFrameConsumer() const override {return FrameConsumer;}
    // Ready means a retained compiled native route can attempt this frame.
    // Individual geometric queries and actual phase outcomes still may refuse.
    bool IsReadyForPhysicsStep() const override {return FrameConsumer&&FrameConsumer->Matches(Snapshot,QueryService);}
    Analytic::EAnalyticNumericContract GetNumericContract() const override
    { return Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush; }
    Analytic::FWorldHit SweepSingle(const Analytic::FWorldQuery& q) const override
    {if(!QueryService)Unavailable();return QueryService->SweepSingle(q);}
    bool TryFindDeepestOverlap(const Analytic::FWorldQuery& q,Analytic::FWorldHit& out) const override
    {if(!QueryService)Unavailable();return QueryService->TryFindDeepestOverlap(q,out);}
    bool HasAuthorityCoverage(const Analytic::FWorldQuery& q) const override
    {return QueryService&&QueryService->HasAuthorityCoverage(q);}
    bool TrySweepAuthority(const Analytic::FWorldQuery& q,Analytic::FWorldHit& out) const override
    {if(!QueryService)Unavailable();return QueryService->TrySweepAuthority(q,out);}
    void VisitPlanarCandidates(const Analytic::FWorldQuery& q,TFunctionRef<void(const Analytic::FWorldHit&)> visitor) const override
    {if(!QueryService)Unavailable();QueryService->VisitPlanarCandidates(q,visitor);}
    void VisitTorusCandidates(const Analytic::FWorldQuery& q,TFunctionRef<void(const Analytic::FTorusSurface&)> visitor) const override
    {if(!QueryService)Unavailable();QueryService->VisitTorusCandidates(q,visitor);}
    bool IsPlanarSupportTranslationCertified(const Analytic::FWorldQuery& q,TConstArrayView<FVector3d> points,const FVector3d& normal,const FVector3d& translation) const override
    {if(!QueryService)Unavailable();return QueryService->IsPlanarSupportTranslationCertified(q,points,normal,translation);}
    EStaticCollisionBackend GetBackend() const override {return EStaticCollisionBackend::SurfaceAnalytic;}
    const Analytic::Prepared::FScenePtr& GetPreparedScene() const {return Snapshot->GetNominalScene();}
    const FPreparedPhysicalQueryDomainPtr& GetPhysicalFilterDomain() const {return Snapshot->GetFilters();}
};
}
