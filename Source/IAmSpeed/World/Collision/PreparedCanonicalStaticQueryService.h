#pragma once
#include "PreparedNativeOwnerPoseMutation.h"
#include "PreparedNativeWheelProbe.h"
#include "PreparedWorldSnapshot.h"
#include "BoundedApproxFixedWheelProbe.h"
#include "StaticCollisionWorld.h"
#include "PreparedNativeSolidGeometry.h"
#include "NativeGeneralSphereContactValues.h"
#include "PreparedSleepingSphereSupport.h"
#include "IAmSpeed/World/Analytic/CanonicalBoxOverlapBounds.h"
#include "IAmSpeed/World/Analytic/AnalyticNumericContext.h"
#include "NativeBoxPoseCorrectionGeometryValues.h"
#include "IAmSpeed/World/Analytic/CanonicalGroupedBoxOverlapBounds.h"
#include <stdexcept>
namespace Speed
{
namespace CanonicalBoxOverlapReuseDetail
{
using EContract=Analytic::EAnalyticNumericContract;
using FShape=Analytic::Certified::FCanonicalGroupedBoxOverlapBoundsPtr;
using FProof=Analytic::Certified::FReferenceBoxVolumeStrictSeparationPtr;
enum class EReuse:uint8 {Unavailable,Ready,Refused};
// Only complete owned geometry can reach Ready. Every live compatibility,
// external-region and numerical check remains on both successful paths.
template<class FScene,class FReuse,class FEnvelope,class FEvaluate,class FExternal,class FMatches>
bool Run(const Analytic::FWorldQuery& captured,FScene&& readScene,
    EContract contract,FReuse&& reuse,FEnvelope&& envelope,FEvaluate&& evaluate,FExternal&& external,FMatches&& matches)
{
    Analytic::RequireCurrentAnalyticNumericContext(contract);
    if(!matches())return false;
    // Do not dereference the snapshot/selection while forming call arguments.
    // This immutable owner is read only after the original live entry guards.
    const auto scene=readScene();
    FShape shape;FProof proof;
    const auto cached=reuse(captured,shape,proof);
    if(cached==EReuse::Refused)return false; // No retry or fresh budget after a failed live gate.
    if(cached!=EReuse::Ready)
    {
        // Literal original cold order. In particular malformed/overflowing
        // envelope refuses BEFORE the original producer and its work account.
        Analytic::Certified::FIntervalPoint hull;
        if(!envelope(captured,hull)||!evaluate(captured,shape,proof))return false;
    }
    if(!shape||!proof||!shape->MatchesQuery(captured)||
        !proof->Matches(scene,shape->GetVolumeForInspection()))return false;
    if(!external(captured,shape->GetVolumeForInspection().BroadHull))return false;
    Analytic::RequireCurrentAnalyticNumericContext(contract);
    return matches()&&shape->MatchesQuery(captured)&&proof->Matches(scene,shape->GetVolumeForInspection());
}
}

enum class EPreparedStaticQueryResult:uint8 {Unknown,KnownMiss,KnownHit};
class FPreparedCanonicalStaticQueryService;
using FPreparedCanonicalStaticQueryServicePtr=TSharedPtr<const FPreparedCanonicalStaticQueryService,ESPMode::ThreadSafe>;
// Immutable geometric consumer shared by suspension/support probes and solid
// collision callers. Its Source implementation must authenticate coefficients,
// policies and complete-union coverage for each query. It owns native immutable
// inputs only; UObject pins and native Prepare/Step/Resolve permissions remain
// with the actual generation owner. In particular a service is NOT Ready.
class IAMSPEED_API FPreparedCanonicalStaticQueryService:public IStaticCollisionWorld
{
    friend class PreparedExecution::FStepScope;
    friend class PreparedExecution::FComponentSweepScope;
    const FPreparedWorldSnapshotPtr Snapshot;
    static bool ValidateImmutableSnapshot(const FPreparedWorldSnapshotPtr& snapshot)
    {
        return snapshot&&snapshot->GetGeometrySelection()&&snapshot->GetGeometrySelection()->HasValidAliases()&&
            snapshot->GetGeometrySelection()->GetModel()==EPreparedGeometryModel::RetainedReference&&
            snapshot->GetReference()&&snapshot->GetFilters()&&
            snapshot->GetFilters()->GetGeneration()==snapshot->GetOrigin().Generation;
    }
    [[noreturn]] static void Unknown()
    {throw std::runtime_error("Canonical prepared static query is unknown; no miss or fallback");}
    static Analytic::FWorldHit ReadResult(EPreparedStaticQueryResult status,Analytic::FWorldHit hit)
    {
        if(status==EPreparedStaticQueryResult::KnownMiss&&!hit.bHit)return {};
        if(status!=EPreparedStaticQueryResult::KnownHit||!hit.bHit||
            !FMath::IsFinite(hit.Time)||hit.Time<0||hit.Time>1||
            !FMath::IsFinite(hit.PenetrationDepth)||hit.PenetrationDepth<0||
            !FMath::IsFinite(hit.GeometricErrorBoundCm)||hit.GeometricErrorBoundCm<0||
            hit.Location.ContainsNaN()||hit.Point.ContainsNaN()||hit.QueryPoint.ContainsNaN()||
            hit.Normal.ContainsNaN()||!FMath::IsFinite(hit.Normal.SizeSquared())||hit.Normal.IsZero()||
            !hit.SourceId||!hit.SurfaceId||!hit.PrimitiveId)Unknown();
        return hit;
    }
protected:
    explicit FPreparedCanonicalStaticQueryService(FPreparedWorldSnapshotPtr snapshot):Snapshot(MoveTemp(snapshot))
    {
        if(!ValidateImmutableSnapshot(Snapshot))
            throw std::runtime_error("Canonical static query service has incomplete or mixed retained geometry");
    }
    virtual bool EvaluateNativeBoxPrimitiveGeometryValues(const PreparedExecution::FNativeSolidStaticValues&,
        const FKinematicState&,float,PreparedExecution::FNativeBoxPrimitiveGeometryValuesPtr& out) const {out.Reset();return false;}
    // Optional pure prediction geometry. Other services keep their original
    // primitive evaluator; no caller gains an advance or query capability.
    virtual bool EvaluateNativeBoxPrimitiveGeometryWithPrediction(const PreparedExecution::FNativeSolidStaticValues& source,
        const FKinematicState& actual,float seconds,const PreparedExecution::FNativeCanonicalBoxSweepClearanceValuesPtr&,
        PreparedExecution::FNativeBoxPrimitiveGeometryValuesPtr& out) const
    {return EvaluateNativeBoxPrimitiveGeometryValues(source,actual,seconds,out);}
    virtual bool EvaluateNativeBoxPoseCorrectionGeometryValues(const PreparedExecution::FNativeSolidStaticValues&,
        const FKinematicState&,const FKinematicState&,PreparedExecution::FNativeBoxPoseCorrectionGeometryValuesPtr& out,
        const FPreparedNativeSolidGeometryPtr&,
        const PreparedExecution::FNativeCanonicalBoxSweepClearanceValuesPtr&) const {out.Reset();return false;}
    virtual bool EvaluateNativeSolidGeometry(const PreparedExecution::FNativeSolidStaticRequest&,FNativeSolidGeometryValues&) const=0;
    // Read-only candidate geometry after an ordinary refusal. No geometry
    // seal, delivery, selection, Ready, response or advance can follow.
    virtual bool ObserveNativeRepeatedSphereFloorAtZero(const PreparedExecution::FNativeSolidStaticRequest&,
        FNativeSolidGeometryValues& output) const {output={};return false;}
    virtual bool EvaluateNativeNextEligibleSphere(const PreparedExecution::FNativeSolidStaticRequest&,
        const PreparedExecution::FNativeRepeatedSphereFloorExclusion&,FNativeSolidGeometryValues& output) const
    {output={};return false;}
    bool CompleteNativeNextEligibleSphere(const PreparedExecution::FNativeSolidStaticRequestPtr& request,
        const PreparedExecution::FNativeRepeatedSphereFloorExclusion& excluded,FPreparedNativeSolidGeometryPtr& output) const
    {
        PreparedExecution::FNativePlanarResponseCommitScope::RequireQueriesClosed();
        output.Reset();Analytic::RequireCurrentAnalyticNumericContext(GetNumericContract());
        const auto current=[&]()
        {
            PreparedExecution::FStepIdentity now;
            return request&&request->GetSnapshot()==Snapshot&&MatchesPreparedSnapshot(Snapshot)&&
                PreparedExecution::RevalidateNativeSolidCCD(*request)&&PreparedExecution::ReadCurrentStep(now)&&
                now.QuerySerial==request->GetStep().QuerySerial;
        };
        FNativeSolidGeometryValues values;
        if(!current()||!EvaluateNativeNextEligibleSphere(*request,excluded,values)||
            values.Status!=ENativeSolidGeometryStatus::GeneralSphereFirstContact||!values.GeneralSphere||
            !values.GeneralSphere->IsNextEligible()||values.GeneralSphere->GetGeometryForInspection()!=Snapshot->GetGeometrySelection()||
            !Analytic::Certified::Detail::ValidPoint(values.QueryEnvelope)||
            !Analytic::Certified::GroupedBoxDetail::SamePoint(values.QueryEnvelope,values.GeneralSphere->GetEnvelopeForInspection())||
            (Snapshot->GetExternalLandscape()&&!Snapshot->GetExternalLandscape()->StrictlySeparated(request->GetValues().Query,values.QueryEnvelope))||
            !current())return false;
        output=MakeShareable(new FPreparedNativeSolidGeometry(request,MoveTemp(values)));return true;
    }
    virtual bool EvaluateNativeBoxFloorSupportGeometry(const PreparedExecution::FNativeSolidStaticRequest&,
        const Analytic::Certified::FNativeBoxCachedPlaneValues&,FNativeSolidGeometryValues& out) const {out={};return false;}
    // Geometry-only observation of an actual cached General refiner. This
    // does not run PhysicalClear, create evidence, authorize a miss or advance.
    virtual bool EvaluateNativeGeneralBoxFloorRefinerObservation(const PreparedExecution::FNativeSolidStaticRequest&,
        const Analytic::Certified::FNativeBoxCachedPlaneValues&,
        Analytic::Certified::FNativeGeneralBoxFloorDescriptorBounds& output) const {output={};return false;}

    virtual bool EvaluateNativeGeneralCachedFloorArrivalGeometry(const PreparedExecution::FNativeSolidStaticRequest&,
        const Analytic::Certified::FNativeGeneralBoxFloorDescriptorBounds&,float,FNativeSolidGeometryValues& output) const {output={};return false;}

    virtual bool EvaluateNativeFixedBoxCachedFloorArrivalGeometry(const PreparedExecution::FNativeSolidStaticRequest&,
        const FNativeSolidGeometryValues&,float,FNativeSolidGeometryValues& output) const {output={};return false;}

    virtual bool EvaluateSerializedSphereSupport(const PreparedExecution::FNativeSolidStaticValues&,
        const PreparedExecution::FSerializedSphereSupportManifold&,PreparedExecution::FSerializedSphereSupportGeometryValues&) const=0;
    virtual EPreparedStaticQueryResult EvaluateNativeFixedProbe(const Analytic::FWorldQuery&,
        const PreparedExecution::FNativeSolidTraceValues&,Analytic::FWorldHit&) const=0;
    // Default preserves other implementations' original native evaluation.
    virtual EPreparedStaticQueryResult EvaluateNativeFixedProbeFromCovered(const Analytic::FWorldQuery& q,
        const PreparedExecution::FNativeSolidTraceValues& trace,EPreparedStaticQueryResult,
        const Analytic::FWorldHit&,Analytic::FWorldHit& out) const
    {return EvaluateNativeFixedProbe(q,trace,out);}
    virtual EPreparedStaticQueryResult EvaluateSweep(const Analytic::FWorldQuery&,Analytic::FWorldHit&) const=0;
    virtual EPreparedStaticQueryResult EvaluateOverlap(const Analytic::FWorldQuery&,Analytic::FWorldHit&) const=0;
    virtual EPreparedStaticQueryResult EvaluateNativeSphereProjection(const Analytic::FWorldQuery& q,
        TFunctionRef<bool()>,Analytic::FWorldHit& out) const {return EvaluateOverlap(q,out);}
    virtual bool EvaluateNativeCurvedBoxPhysicalProjectionClear(const Analytic::FWorldQuery&,
        const PreparedExecution::FNativeCurvedBoxPhysicalProjectionContext&) const
    {return false;}
    virtual bool EvaluateCanonicalFixedBoxOverlapClearance(const Analytic::FWorldQuery&,
        const Analytic::Certified::FIntervalPoint&,
        Analytic::Certified::FReferenceBoundsStrictSeparationPtr& out) const {out.Reset();return false;}
    virtual bool EvaluateCanonicalGroupedFixedBoxOverlapClearance(const Analytic::FWorldQuery&,
        Analytic::Certified::FCanonicalGroupedBoxOverlapBoundsPtr& shape,
        Analytic::Certified::FReferenceBoxVolumeStrictSeparationPtr& proof) const
    {shape.Reset();proof.Reset();return false;}
    virtual bool EvaluateExactBoxSupportContinuation(const Analytic::FWorldQuery&,
        const Analytic::Certified::FNativeBoxCachedPlaneValues&,Analytic::FWorldHit& out) const
    {out={};return false;}

    virtual CanonicalBoxOverlapReuseDetail::EReuse TryReuseCanonicalGroupedFixedBoxOverlapClearance(
        const Analytic::FWorldQuery&,Analytic::Certified::FCanonicalGroupedBoxOverlapBoundsPtr& shape,
        Analytic::Certified::FReferenceBoxVolumeStrictSeparationPtr& proof) const
    {shape.Reset();proof.Reset();return CanonicalBoxOverlapReuseDetail::EReuse::Unavailable;}
    virtual bool BuildCanonicalFixedBoxOverlapEnvelopeForQuery(const Analytic::FWorldQuery& query,
        Analytic::Certified::FIntervalPoint& hull) const
    {return Analytic::Certified::BuildCanonicalFixedBoxOverlapEnvelope(query,hull);}

    // Visitors are invoked only AFTER a complete successful evaluation. A
    // partially emitted candidate list must never survive an Unknown result.
    virtual bool GatherPlanarCandidates(const Analytic::FWorldQuery&,TArray<Analytic::FWorldHit>&) const=0;
    virtual bool GatherTorusCandidates(const Analytic::FWorldQuery&,TArray<Analytic::FTorusSurface>&) const=0;
    virtual bool EvaluatePlanarSupport(const Analytic::FWorldQuery&,TConstArrayView<FVector3d>,
        const FVector3d&,const FVector3d&,bool&) const=0;
public:
    // Optional immutable routing domain for completed fixed-wheel fallbacks.
    // Non-null promises that this service's fixed coverage calculation is a
    // deterministic function of its pinned snapshot, complete query and numeric
    // controls under the currently matching routing profile. Trace policies and
    // all execution guards remain fresh. Other services opt out by default.
    virtual TSharedPtr<const void,ESPMode::ThreadSafe> GetHistoricalFixedCoverageDomain() const {return {};}
    // Order suggestion only: true still uses the complete coverage/native
    // closure; false preserves the ordinary route.
    virtual bool PreferPreparedFixedProbeCoverage(const Analytic::FWorldQuery&) const {return false;}
#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
    // Private observation only. No result, permission or mutable hit is returned.
    // Optional private bounded-SPATIAL proposal, never an exact contact,
    // miss or readiness capability. Complete source coverage is mandatory.
    virtual PreparedExecution::EBoundedApproxWheelAttempt TryBoundedLegacyWheelProposal(
        const Analytic::FWorldQuery&,Analytic::Certified::FDistanceBudget&,int32,
        PreparedExecution::FBoundedApproxWheelProposal& output) const
    {output={};return PreparedExecution::EBoundedApproxWheelAttempt::NotApplicable;}
    // Same optional approximation, with a complete original-CP BVH inventory
    // instead of copied canonical partitions. Unsupported services decline.
    virtual PreparedExecution::EBoundedApproxWheelAttempt TryBoundedLegacyWheelInventoryProposal(
        const Analytic::FWorldQuery&,Analytic::Certified::FDistanceBudget&,int32,
        PreparedExecution::FBoundedApproxWheelProposal& output) const
    {output={};return PreparedExecution::EBoundedApproxWheelAttempt::NotApplicable;}
    virtual void PrepareLegacyQuinticShadowAtPublication() const {}
    virtual bool HasPreparedLegacyQuinticShadow() const {return false;}
    virtual void ObserveLegacyQuinticShadow(const Analytic::FWorldQuery&,const Analytic::FWorldHit&,uint64,bool) const {}
#endif
    bool MatchesPreparedSnapshot(const FPreparedWorldSnapshotPtr& expected) const
    {
        // The complete immutable predicate is checked directly by the
        // constructor. Fresh retention/scope/numeric/pose guards stay at callers.
        return expected&&Snapshot==expected;
    }
    FPreparedWorldSnapshotPtr GetPreparedSnapshot() const final {return Snapshot;}
    bool IsReadyForPhysicsStep() const final {return false;}
    // Ordinary FullFrame callers only. The exact serial is checked both sides
    // of the complete evaluation; older same-epoch observations cannot seal a
    // result. This does NOT grant readiness or a native collision delivery.
    bool QueryNativeSolidGeometry(const PreparedExecution::FNativeSolidStaticRequestPtr& request,
        FPreparedNativeSolidGeometryPtr& output,FNativeCase202AtlasObservationPtr* observation=nullptr) const
    {
        PreparedExecution::FNativePlanarResponseCommitScope::RequireQueriesClosed();
        output.Reset();if(observation)observation->Reset();
        Analytic::RequireCurrentAnalyticNumericContext(GetNumericContract());
        const auto current=[&]()
        {
            PreparedExecution::FStepIdentity now;
            return request&&request->GetSnapshot()==Snapshot&&MatchesPreparedSnapshot(Snapshot)&&
                PreparedExecution::RevalidateNativeSolidCCD(*request)&&PreparedExecution::ReadCurrentStep(now)&&
                now.QuerySerial==request->GetStep().QuerySerial;
        };
        if(!current())return false;FNativeSolidGeometryValues values;
        if(!EvaluateNativeSolidGeometry(*request,values))
        {
            if(observation&&values.Status==ENativeSolidGeometryStatus::Unknown&&values.Case202AtlasObservation&&current())
                *observation=MoveTemp(values.Case202AtlasObservation);
            return false;
        }
        if(values.Case202AtlasObservation||values.Status==ENativeSolidGeometryStatus::Unknown||
            values.Status==ENativeSolidGeometryStatus::GeneralBoxFloorCachedArrivalDescriptor||
            !Analytic::Certified::Detail::ValidPoint(values.QueryEnvelope))return false;
        if(values.Status==ENativeSolidGeometryStatus::GeneralBoxFloorMotionDescriptor)
        {
            const auto& g=values.GeneralBoxFloorMotionDescriptor;
            if(!g.HasMatchingPhysicalMotion()||!PreparedExecution::SameNativeSolidStaticValues(g.Captured,request->GetValues())||
                !Analytic::Certified::GroupedBoxDetail::SamePoint(g.QueryEnvelope,values.QueryEnvelope))return false;
        }
        if(values.Status==ENativeSolidGeometryStatus::GeneralBoxCurvedArrivalDescriptor)
        {
            float seconds=0;Analytic::FWorldHit hit;
            if(!PreparedExecution::ReadNativeCurvedBoxArrivalValues(Snapshot,request->GetValues(),values.CanonicalBoxClearance,seconds,hit)||
                !Analytic::Certified::GroupedBoxDetail::SamePoint(values.QueryEnvelope,values.CanonicalBoxClearance->GetEnvelopeForInspection()))return false;
        }
        else if(values.Status==ENativeSolidGeometryStatus::GeneralBoxCurvedActualMotionClear)
        {
            if(!PreparedExecution::MatchesNativeCurvedBoxActualMotionClearValues(Snapshot,request->GetValues(),
                values.CanonicalBoxClearance,values.QueryEnvelope))return false;
        }
        else if(values.CanonicalBoxClearance||values.Status==ENativeSolidGeometryStatus::CanonicalReferenceBoxSweepPathClear)
        {
            if(values.Status!=ENativeSolidGeometryStatus::CanonicalReferenceBoxSweepPathClear||
                !PreparedExecution::MatchesNativeCanonicalBoxSweepClearanceValues(request->GetValues(),Snapshot->GetGeometrySelection(),
                    values.CanonicalBoxClearance,values.QueryEnvelope))return false;
        }
        if(values.CanonicalAxialBoxPlane||values.Status==ENativeSolidGeometryStatus::CanonicalAxialBoxPlaneDescriptor)
        {
            if(values.Status!=ENativeSolidGeometryStatus::CanonicalAxialBoxPlaneDescriptor||
                !PreparedExecution::MatchesNativeCanonicalAxialBoxPlaneValues(request->GetValues(),Snapshot->GetGeometrySelection(),
                    Snapshot->GetNominalScene(),values.CanonicalAxialBoxPlane,values.QueryEnvelope))return false;
        }
        Analytic::RequireCurrentAnalyticNumericContext(GetNumericContract());
        if(!current())return false;
        if(Snapshot->GetExternalLandscape()&&!Snapshot->GetExternalLandscape()->StrictlySeparated(request->GetValues().Query,values.QueryEnvelope))return false;
        output=MakeShareable(new FPreparedNativeSolidGeometry(request,MoveTemp(values)));return true;
    }
    // Same-request continuation from a retained Atlas observation. This does
    // not run a second static query or grant a selected World result.
    bool CompleteCase202Serial3Curve(const PreparedExecution::FNativeSolidStaticRequestPtr& request,
        const FNativeCase202AtlasObservationPtr& atlas,FPreparedNativeSolidGeometryPtr& output) const
    {
        PreparedExecution::FNativePlanarResponseCommitScope::RequireQueriesClosed();
        output.Reset();
        const auto current=[&]()
        {
            PreparedExecution::FStepIdentity now;
            return request&&request->GetSnapshot()==Snapshot&&MatchesPreparedSnapshot(Snapshot)&&
                PreparedExecution::RevalidateNativeSolidCCD(*request)&&PreparedExecution::ReadCurrentStep(now)&&
                now.QuerySerial==request->GetStep().QuerySerial;
        };
        if(!current()||!atlas||!atlas->PositivePrefixPartition)return false;
        FNativeSolidGeometryValues values;
        auto& proof=values.CanonicalBoxClearance;
        if(!PreparedExecution::BuildCase202Serial3CurveArrival(*request,atlas,proof)||!proof||!proof->IsCase202Serial3Prefix())return false;
        values.QueryEnvelope=proof->GetEnvelopeForInspection();
        float seconds=0;Analytic::FWorldHit hit;
        if(!Analytic::Certified::Detail::ValidPoint(values.QueryEnvelope)||
            !PreparedExecution::ReadNativeCurvedBoxArrivalValues(Snapshot,request->GetValues(),proof,seconds,hit)||
            !current()||(Snapshot->GetExternalLandscape()&&
                !Snapshot->GetExternalLandscape()->StrictlySeparated(request->GetValues().Query,values.QueryEnvelope)))return false;
        values.Status=ENativeSolidGeometryStatus::GeneralBoxCurvedArrivalDescriptor;
        output=MakeShareable(new FPreparedNativeSolidGeometry(request,MoveTemp(values)));
        return true;
    }

    // Pure actual-primitive geometry, not a fabricated World query. Its
    // provenance and one-time consumption are private to FStepScope.
    bool DescribeNativeBoxPrimitiveGeometryValues(const PreparedExecution::FNativeSolidStaticValues& original,
        const FKinematicState& actual,float dt,PreparedExecution::FNativeBoxPrimitiveGeometryValuesPtr& output,
        const PreparedExecution::FNativeCanonicalBoxSweepClearanceValuesPtr& prediction={}) const
    {
        PreparedExecution::FNativePlanarResponseCommitScope::RequireQueriesClosed();
        output.Reset();Analytic::RequireCurrentAnalyticNumericContext(GetNumericContract());
        if(!MatchesPreparedSnapshot(Snapshot))return false;
        PreparedExecution::FNativeBoxPrimitiveGeometryValuesPtr result;
        if(!EvaluateNativeBoxPrimitiveGeometryWithPrediction(original,actual,dt,prediction,result)||!result||
            !result->Matches(Snapshot,original,actual,dt))return false;
        if(Snapshot->GetExternalLandscape()&&!Snapshot->GetExternalLandscape()->StrictlySeparated(original.Query,result->GetEnvelopeForInspection()))return false;
        Analytic::RequireCurrentAnalyticNumericContext(GetNumericContract());
        if(!MatchesPreparedSnapshot(Snapshot))return false;output=MoveTemp(result);return true;
    }
    bool DescribeNativeReleasedBoxFloorPrimitiveGeometryValues(const PreparedExecution::FNativeSolidStaticValues& original,
        const FKinematicState& actual,float seconds,const FPreparedNativeSolidGeometryPtr& descriptor,
        PreparedExecution::FNativeBoxPrimitiveGeometryValuesPtr& output) const
    {
        PreparedExecution::FNativePlanarResponseCommitScope::RequireQueriesClosed();
        output.Reset();Analytic::RequireCurrentAnalyticNumericContext(GetNumericContract());
        if(!MatchesPreparedSnapshot(Snapshot))return false;
        const int64 work=2ll*Snapshot->GetGeometrySelection()->GetSelectedUnion()->Num()+8193;
        if(work<=0||work>MAX_int32)return false;Analytic::Certified::FDistanceBudget budget{int32(work)};
        PreparedExecution::FNativeBoxPrimitiveGeometryValuesPtr result;
        if(!PreparedExecution::BuildNativeReleasedBoxFloorPrimitiveGeometryValues(Snapshot,original,actual,seconds,descriptor,budget,result)||
            !result||!result->Matches(Snapshot,original,actual,seconds))return false;
        if(Snapshot->GetExternalLandscape()&&!Snapshot->GetExternalLandscape()->StrictlySeparated(original.Query,result->GetEnvelopeForInspection()))return false;
        Analytic::RequireCurrentAnalyticNumericContext(GetNumericContract());
        if(!MatchesPreparedSnapshot(Snapshot))return false;output=MoveTemp(result);return true;
    }
    bool DescribeNativeGeneralBoxFloorMotionPrimitiveGeometryValues(const PreparedExecution::FNativeSolidStaticValues& original,
        const FKinematicState& actual,float seconds,float actualWorldSeconds,const FPreparedNativeSolidGeometryPtr& descriptor,
        PreparedExecution::FNativeBoxPrimitiveGeometryValuesPtr& output) const
    {
        PreparedExecution::FNativePlanarResponseCommitScope::RequireQueriesClosed();
        output.Reset();Analytic::RequireCurrentAnalyticNumericContext(GetNumericContract());
        if(!MatchesPreparedSnapshot(Snapshot))return false;
        const int64 work=2ll*Snapshot->GetGeometrySelection()->GetSelectedUnion()->Num()+8193;
        if(work<=0||work>MAX_int32)return false;Analytic::Certified::FDistanceBudget budget{int32(work)};
        PreparedExecution::FNativeBoxPrimitiveGeometryValuesPtr result;
        if(!PreparedExecution::BuildNativeGeneralBoxFloorMotionPrimitiveGeometryValues(Snapshot,original,actual,seconds,actualWorldSeconds,descriptor,budget,result)||
            !result||!result->MatchesFloorMotion(Snapshot,original,actual,seconds,actualWorldSeconds,descriptor))return false;
        if(Snapshot->GetExternalLandscape()&&!Snapshot->GetExternalLandscape()->StrictlySeparated(original.Query,result->GetEnvelopeForInspection()))return false;
        Analytic::RequireCurrentAnalyticNumericContext(GetNumericContract());
        if(!MatchesPreparedSnapshot(Snapshot))return false;output=MoveTemp(result);return true;
    }
    // Distinct pure geometry of REAL correction operands, never a fabricated
    // Integrate(dt), World query, TOI or advance capability. The real setter
    // scope authenticates and consumes it only around the actual assignment.
    bool DescribeNativeBoxPoseCorrectionGeometryValues(const PreparedExecution::FNativeSolidStaticValues& original,
        const FKinematicState& before,const FKinematicState& candidate,
        PreparedExecution::FNativeBoxPoseCorrectionGeometryValuesPtr& output,
        const FPreparedNativeSolidGeometryPtr& establishedFloor={},
        const PreparedExecution::FNativeCanonicalBoxSweepClearanceValuesPtr& curvedPrediction={}) const
    {
        PreparedExecution::FNativePlanarResponseCommitScope::RequireQueriesClosed();
        output.Reset();Analytic::RequireCurrentAnalyticNumericContext(GetNumericContract());
        if(!MatchesPreparedSnapshot(Snapshot))return false;
        PreparedExecution::FNativeBoxPoseCorrectionGeometryValuesPtr result;
        if((establishedFloor&&curvedPrediction)||
            !EvaluateNativeBoxPoseCorrectionGeometryValues(original,before,candidate,result,establishedFloor,curvedPrediction)||!result||
            !result->Matches(Snapshot,original,before,candidate)||
            !result->MatchesEstablishedFloorBinding(establishedFloor)||
            !result->MatchesCurvedArrivalBinding(curvedPrediction))return false;
        if(Snapshot->GetExternalLandscape()&&!Snapshot->GetExternalLandscape()->StrictlySeparated(original.Query,result->GetEnvelopeForInspection()))return false;
        Analytic::RequireCurrentAnalyticNumericContext(GetNumericContract());
        if(!MatchesPreparedSnapshot(Snapshot))return false;output=MoveTemp(result);return true;
    }
    // Explicitly selected before any released-motion proof. A false result
    // is terminal at the caller; no retry of a failed PhysicalClear occurs.
    bool DescribeNativeGeneralBoxFloorRefinerObservation(const PreparedExecution::FNativeSolidStaticRequestPtr& request,
        const Analytic::Certified::FNativeBoxCachedPlaneValues& cache,
        FNativeGeneralBoxFloorObservationPreludePtr& output) const
    {
        PreparedExecution::FNativePlanarResponseCommitScope::RequireQueriesClosed();
        output.Reset();const auto numeric=Analytic::RequireCurrentAnalyticNumericContext(GetNumericContract());
        const auto current=[&]()
        {
            PreparedExecution::FStepIdentity now;
            return request&&request->GetSnapshot()==Snapshot&&MatchesPreparedSnapshot(Snapshot)&&
                Analytic::Certified::NativeBoxNeedsGeneralFloorDescriptor(request->GetValues())&&
                PreparedExecution::RevalidateNativeSolidCCD(*request)&&PreparedExecution::ReadCurrentStep(now)&&
                now.QuerySerial==request->GetStep().QuerySerial;
        };
        if(!current())return false;Analytic::Certified::FNativeGeneralBoxFloorDescriptorBounds result;
        if(!EvaluateNativeGeneralBoxFloorRefinerObservation(*request,cache,result)||!result.Complete||result.PhysicalClear||
            result.ObservationRequestIdentity!=request.Get()||result.ObservationWorkLimit<=0||result.ObservationWorkRemaining<0||
            !result.ObservationWorkUsed||result.ObservationWorkUsed>uint64(result.ObservationWorkLimit)||result.ObservationConsumptionFloor!=0||
            result.ObservationWorkUsed+uint64(result.ObservationWorkRemaining)!=uint64(result.ObservationWorkLimit)||
            !PreparedExecution::SameNativeSolidStaticValues(result.Captured,request->GetValues())||
            result.NumericFingerprint!=numeric.ControlFingerprint()||
            result.OriginalMotion.Corners.Num()!=32||result.AtZero.Corners.Num()!=32||
            !Analytic::Certified::Detail::ValidPoint(result.QueryEnvelope))return false;
        if(result.ObservationFinitePartition)
        {
            namespace C=Analytic::Certified;
            const auto& partition=result.ObservationFinitePartition;
            if(partition->GetScene()!=Snapshot->GetGeometrySelection()->GetSelectedUnion()||
                !C::GroupedBoxDetail::Within(partition->GetQueryHull(),result.QueryEnvelope)||
                !C::GroupedBoxDetail::Within(partition->GetQueryHull(),result.ObservationInitialEnvelope)||
                (result.ObservationCurvedProvider!=INDEX_NONE&&
                    !partition->GetCandidateIndices().Contains(result.ObservationCurvedProvider)))return false;
        }
        else if(result.ObservationCurvedProvider!=INDEX_NONE)return false;
        const auto& actual=result.CachedPlane;
        if(!Analytic::Certified::FloorNativeSweepDetail::SameVector(actual.Point,cache.Point)||
            !Analytic::Certified::FloorNativeSweepDetail::SameVector(actual.Normal,cache.Normal)||
            actual.SourceId!=cache.SourceId||actual.SurfaceId!=cache.SurfaceId||actual.FeatureId!=cache.FeatureId||
            actual.PrimitiveId!=cache.PrimitiveId||actual.CanonicalGroupId!=cache.CanonicalGroupId||actual.MaterialId!=cache.MaterialId||
            !actual.SourceId||!actual.SurfaceId||!actual.PrimitiveId)return false;
        if(Snapshot->GetExternalLandscape()&&!Snapshot->GetExternalLandscape()->StrictlySeparated(request->GetValues().Query,result.QueryEnvelope))return false;
        if(Analytic::RequireCurrentAnalyticNumericContext(GetNumericContract()).ControlFingerprint()!=numeric.ControlFingerprint()||!current())return false;
        output=MakeShareable(new FNativeGeneralBoxFloorObservationPrelude(request,MoveTemp(result)));return true;
    }

    bool QueryNativeEstablishedGeneralFloorOutcome(const PreparedExecution::FNativeSolidStaticRequestPtr& request,
        const FNativeGeneralBoxFloorObservationPreludePtr& handle,FPreparedNativeSolidGeometryPtr& output) const
    {
        namespace C=Analytic::Certified;
        PreparedExecution::FNativePlanarResponseCommitScope::RequireQueriesClosed();
        output.Reset();
        const auto numeric=Analytic::RequireCurrentAnalyticNumericContext(GetNumericContract());
        const auto current=[&]()
        {
            PreparedExecution::FStepIdentity now;
            return request&&request->GetSnapshot()==Snapshot&&MatchesPreparedSnapshot(Snapshot)&&
                PreparedExecution::RevalidateNativeSolidCCD(*request)&&PreparedExecution::ReadCurrentStep(now)&&
                now.QuerySerial==request->GetStep().QuerySerial;
        };
        if(!handle||handle->Request!=request||!current())return false;
        const auto& prelude=handle->Values;
        if(!prelude.Complete||prelude.PhysicalClear||prelude.ObservationRequestIdentity!=request.Get()||
            !PreparedExecution::SameNativeSolidStaticValues(prelude.Captured,request->GetValues())||
            prelude.NumericFingerprint!=numeric.ControlFingerprint()||prelude.ObservationWorkLimit<=0||
            prelude.ObservationWorkRemaining<0||!prelude.ObservationWorkUsed||
            prelude.ObservationWorkUsed+uint64(prelude.ObservationWorkRemaining)!=uint64(prelude.ObservationWorkLimit)||
            prelude.ObservationConsumptionFloor!=0||!handle->ClaimCompletion())return false;
        C::FDistanceBudget budget{prelude.ObservationWorkRemaining,prelude.ObservationWorkUsed,prelude.ObservationConsumptionFloor};
        FNativeSolidGeometryValues values;
        auto& g=values.GeneralBoxFloorDescriptor;
        g=prelude;
        if(g.ObservationCurvedProvider!=INDEX_NONE)
        {
            if(!PreparedExecution::BuildNativeEstablishedFloorCurveArrival(Snapshot,request->GetValues(),g,budget,
                values.CanonicalBoxClearance)||!values.CanonicalBoxClearance)return false;
            values.Status=ENativeSolidGeometryStatus::GeneralBoxCurvedArrivalDescriptor;
            values.QueryEnvelope=values.CanonicalBoxClearance->GetEnvelopeForInspection();
        }
        else
        {
            const auto& plane=g.CachedPlane;
            if(!C::TryProveGeneralBoxPhysicalFloorClear(g.GetMotionInputForInspection(),plane.Point,plane.Normal,budget,g.PhysicalClear,true)||
                !C::MatchesNativeGeneralBoxFloorPhysicalClear(g))return false;
            values.Status=ENativeSolidGeometryStatus::GeneralBoxFloorRefinerDescriptor;
            values.QueryEnvelope=g.QueryEnvelope;
            values.SourceId=plane.SourceId;values.SurfaceId=plane.SurfaceId;values.FeatureId=plane.FeatureId;
            values.PrimitiveId=plane.PrimitiveId;values.CanonicalGroupId=plane.CanonicalGroupId;values.MaterialId=plane.MaterialId;
        }
        if(Snapshot->GetExternalLandscape()&&!Snapshot->GetExternalLandscape()->StrictlySeparated(request->GetValues().Query,values.QueryEnvelope))return false;
        if(Analytic::RequireCurrentAnalyticNumericContext(GetNumericContract()).ControlFingerprint()!=numeric.ControlFingerprint()||!current())return false;
        output=MakeShareable(new FPreparedNativeSolidGeometry(request,MoveTemp(values)));return true;
    }

    bool QueryNativeGeneralCachedFloorArrivalGeometry(const PreparedExecution::FNativeSolidStaticRequestPtr& request,
        const FNativeGeneralBoxFloorObservationPreludePtr& handle,float actualTOI,FPreparedNativeSolidGeometryPtr& output) const
    {
        PreparedExecution::FNativePlanarResponseCommitScope::RequireQueriesClosed();output.Reset();
        const auto numeric=Analytic::RequireCurrentAnalyticNumericContext(GetNumericContract());
        const auto current=[&]()
        {
            PreparedExecution::FStepIdentity now;
            return request&&request->GetSnapshot()==Snapshot&&MatchesPreparedSnapshot(Snapshot)&&
                PreparedExecution::RevalidateNativeSolidCCD(*request)&&PreparedExecution::ReadCurrentStep(now)&&
                now.QuerySerial==request->GetStep().QuerySerial;
        };
        if(!handle||handle->Request!=request||!current())return false;
        const auto& prelude=handle->Values;
        if(prelude.ObservationRequestIdentity!=request.Get()||!prelude.Complete||prelude.PhysicalClear||
            prelude.ObservationFinitePartition||
            !Analytic::Certified::NativeBoxNeedsGeneralFloorDescriptor(request->GetValues())||
            !PreparedExecution::SameNativeSolidStaticValues(prelude.Captured,request->GetValues())||
            prelude.NumericFingerprint!=numeric.ControlFingerprint()||prelude.ObservationWorkLimit<=0||
            prelude.ObservationWorkRemaining<0||!prelude.ObservationWorkUsed||
            prelude.ObservationWorkUsed>uint64(prelude.ObservationWorkLimit)||prelude.ObservationConsumptionFloor!=0||
            prelude.ObservationWorkUsed+uint64(prelude.ObservationWorkRemaining)!=uint64(prelude.ObservationWorkLimit)||
            !FMath::IsFinite(actualTOI)||!(actualTOI>0.f)||actualTOI>request->GetValues().Horizon)return false;
        // Claim precedes the first continuation operation. A failed proof
        // cannot refund the prelude or start a second budget on this handle.
        if(!handle->ClaimCompletion())return false;
        FNativeSolidGeometryValues values;
        if(!EvaluateNativeGeneralCachedFloorArrivalGeometry(*request,prelude,actualTOI,values)||
            values.Status!=ENativeSolidGeometryStatus::GeneralBoxFloorCachedArrivalDescriptor||
            !Analytic::Certified::GroupedBoxDetail::SamePoint(values.QueryEnvelope,prelude.QueryEnvelope))return false;
        const auto& g=values.GeneralBoxCachedFloorArrivalDescriptor;const auto& a=g.Plane;const auto& b=prelude.CachedPlane;
        if(!g.HasMatchingPhysicalPrefix()||!PreparedExecution::SameNativeSolidStaticValues(g.Captured,request->GetValues())||
            !Analytic::Certified::GroupedBoxDetail::SamePoint(g.QueryEnvelope,prelude.QueryEnvelope)||
            g.OriginalWorkLimit!=prelude.ObservationWorkLimit||g.PreludeWorkUsed!=prelude.ObservationWorkUsed||
            !g.CachedPrefix->Matches(prelude.GetMotionInputForInspection(),b.Point,b.Normal,actualTOI)||
            !Analytic::Certified::FloorNativeSweepDetail::SameVector(a.Point,b.Point)||
            !Analytic::Certified::FloorNativeSweepDetail::SameVector(a.Normal,b.Normal)||
            a.SourceId!=b.SourceId||a.SurfaceId!=b.SurfaceId||a.FeatureId!=b.FeatureId||a.PrimitiveId!=b.PrimitiveId||
            a.CanonicalGroupId!=b.CanonicalGroupId||a.MaterialId!=b.MaterialId||
            values.SourceId!=a.SourceId||values.SurfaceId!=a.SurfaceId||values.FeatureId!=a.FeatureId||values.PrimitiveId!=a.PrimitiveId||
            values.CanonicalGroupId!=a.CanonicalGroupId||values.MaterialId!=a.MaterialId)return false;
        if(Snapshot->GetExternalLandscape()&&!Snapshot->GetExternalLandscape()->StrictlySeparated(request->GetValues().Query,values.QueryEnvelope))return false;
        if(Analytic::RequireCurrentAnalyticNumericContext(GetNumericContract()).ControlFingerprint()!=numeric.ControlFingerprint()||!current())return false;
        output=MakeShareable(new FPreparedNativeSolidGeometry(request,MoveTemp(values)));return true;
    }

    bool CompleteNativeFixedBoxCachedFloorArrivalGeometry(
        const PreparedExecution::FNativeSolidStaticRequestPtr& request,
        const FPreparedNativeSolidGeometryPtr& fixedSupport,float actualTOI,FPreparedNativeSolidGeometryPtr& output) const
    {
        namespace C=Analytic::Certified;
        PreparedExecution::FNativePlanarResponseCommitScope::RequireQueriesClosed();
        output.Reset();
        const auto numeric=Analytic::RequireCurrentAnalyticNumericContext(GetNumericContract()).ControlFingerprint();
        const auto current=[&]()
        {
            PreparedExecution::FStepIdentity now;
            if(!request||request->GetSnapshot()!=Snapshot||!MatchesPreparedSnapshot(Snapshot)||
                !PreparedExecution::RevalidateNativeSolidCCD(*request)||!PreparedExecution::ReadCurrentStep(now))return false;
            const auto& step=request->GetStep();
            return now.Frame==step.Frame&&now.StepSerial==step.StepSerial&&now.Iteration==step.Iteration&&
                now.QuerySerial==step.QuerySerial&&now.QueryEpoch==step.QueryEpoch&&now.InputGeneration==step.InputGeneration&&
                now.Provider==step.Provider;
        };
        if(!fixedSupport||fixedSupport->Request!=request||!current())return false;
        const auto& old=fixedSupport->Values;
        const auto& v=request->GetValues();const auto& step=request->GetStep();
        if(old.Status!=ENativeSolidGeometryStatus::BoxFloorSupportDescriptor||!old.FixedBoxFloorWork.IsSet()||
            !old.BoxSupport.Complete||old.BoxSupport.Motion!=C::EBoxFloorSupportMotion::Fixed||
            !C::GroupedBoxDetail::SamePoint(old.QueryEnvelope,old.BoxSupport.QueryEnvelope)||
            v.Shape!=PreparedExecution::ENativeSolidQueryShape::Box||v.OwnerStableId!=3||v.BodyIndex!=0||
            v.FrameClock.OwnerLocalFrame!=4||step.Frame!=3||step.StepSerial!=4||step.Iteration!=2||
            step.QuerySerial!=4||step.QueryEpoch!=4||!FMath::IsFinite(actualTOI)||
            !(actualTOI>0.f)||actualTOI>v.Horizon)return false;
        const auto& paid=old.FixedBoxFloorWork.GetValue();
        if(paid.Request!=request.Get()||paid.NumericFingerprint!=numeric||paid.Limit<=0||paid.Remaining<0||
            !paid.Used||paid.Used>uint64(paid.Limit)||paid.ConsumptionFloor!=0||
            paid.Used+uint64(paid.Remaining)!=uint64(paid.Limit)||!fixedSupport->ClaimFixedBoxFloorCompletion())return false;
        FNativeSolidGeometryValues values;
        if(!EvaluateNativeFixedBoxCachedFloorArrivalGeometry(*request,old,actualTOI,values)||
            values.Status!=ENativeSolidGeometryStatus::GeneralBoxFloorCachedArrivalDescriptor||
            !C::GroupedBoxDetail::Within(values.QueryEnvelope,old.QueryEnvelope))return false;
        const auto& g=values.GeneralBoxCachedFloorArrivalDescriptor;
        const auto& a=g.Plane;const auto& b=old.BoxSupport.CachedPlane;
        if(!g.HasMatchingPhysicalPrefix()||!g.CachedPrefix->IsFixedCardinal()||
            !PreparedExecution::SameNativeSolidStaticValues(g.Captured,v)||
            !C::GroupedBoxDetail::SamePoint(g.QueryEnvelope,values.QueryEnvelope)||
            g.OriginalWorkLimit!=paid.Limit||g.PreludeWorkUsed!=paid.Used||g.TotalWorkUsed<=paid.Used||
            !g.CachedPrefix->Matches(g.GetMotionInputForInspection(),b.Point,b.Normal,actualTOI)||
            !C::FloorNativeSweepDetail::SameVector(a.Point,b.Point)||!C::FloorNativeSweepDetail::SameVector(a.Normal,b.Normal)||
            a.SourceId!=b.SourceId||a.SurfaceId!=b.SurfaceId||a.FeatureId!=b.FeatureId||a.PrimitiveId!=b.PrimitiveId||
            a.CanonicalGroupId!=b.CanonicalGroupId||a.MaterialId!=b.MaterialId||
            values.SourceId!=a.SourceId||values.SurfaceId!=a.SurfaceId||values.FeatureId!=a.FeatureId||
            values.PrimitiveId!=a.PrimitiveId||values.CanonicalGroupId!=a.CanonicalGroupId||values.MaterialId!=a.MaterialId)return false;
        if(Snapshot->GetExternalLandscape()&&!Snapshot->GetExternalLandscape()->StrictlySeparated(v.Query,values.QueryEnvelope))return false;
        if(Analytic::RequireCurrentAnalyticNumericContext(GetNumericContract()).ControlFingerprint()!=numeric||!current())return false;
        output=MakeShareable(new FPreparedNativeSolidGeometry(request,MoveTemp(values)));return true;
    }

    // A floor DESCRIPTOR plus complete geometric bounds for a real cached
    // Box refinement. Neither a native sweep miss nor a new collision event.
    bool QueryNativeBoxFloorSupportGeometry(const PreparedExecution::FNativeSolidStaticRequestPtr& request,
        const Analytic::Certified::FNativeBoxCachedPlaneValues& cache,FPreparedNativeSolidGeometryPtr& output) const
    {
        PreparedExecution::FNativePlanarResponseCommitScope::RequireQueriesClosed();
        output.Reset();Analytic::RequireCurrentAnalyticNumericContext(GetNumericContract());
        // Failure-only observation of the ALREADY copied request/cache.
        // No new getter, query, refiner, numeric predicate or admission result.
        const auto failed=[&](const TCHAR* stage)
        {
            if(request)
            {
                const auto& v=request->GetValues();
                UE_LOG(LogTemp,Display,TEXT("[NativeBoxFloorSupportWrapperUnknown] Stage=%s RequestKnown=1 Frame=%u Serial=%llu H=%.17g Shape=%u Frozen=%d Prescribed=%d"),
                    stage,v.FrameClock.OwnerLocalFrame,request->GetStep().QuerySerial,double(v.Horizon),uint32(v.Shape),int32(v.Frozen),int32(v.Prescribed));
            }
            else UE_LOG(LogTemp,Display,TEXT("[NativeBoxFloorSupportWrapperUnknown] Stage=%s RequestKnown=0"),stage);
            return false;
        };
        const auto current=[&]()
        {
            PreparedExecution::FStepIdentity now;
            return request&&request->GetSnapshot()==Snapshot&&MatchesPreparedSnapshot(Snapshot)&&
                request->GetValues().Shape==PreparedExecution::ENativeSolidQueryShape::Box&&
                PreparedExecution::RevalidateNativeSolidCCD(*request)&&PreparedExecution::ReadCurrentStep(now)&&
                now.QuerySerial==request->GetStep().QuerySerial;
        };
        if(!current())return failed(TEXT("context_before"));FNativeSolidGeometryValues values;
        if(!EvaluateNativeBoxFloorSupportGeometry(*request,cache,values))return failed(TEXT("service"));
        const bool general=values.Status==ENativeSolidGeometryStatus::GeneralBoxFloorRefinerDescriptor;
        if(!general&&values.Status!=ENativeSolidGeometryStatus::BoxFloorSupportDescriptor)return failed(TEXT("result_status"));
        if(general)
        {
            const auto& g=values.GeneralBoxFloorDescriptor;
            if(!g.Complete||!PreparedExecution::SameNativeSolidStaticValues(g.Captured,request->GetValues())||
                g.NumericFingerprint!=Analytic::RequireCurrentAnalyticNumericContext(GetNumericContract()).ControlFingerprint())
                return failed(TEXT("general_descriptor_incomplete_or_changed"));
        }
        else if(!values.BoxSupport.Complete)return failed(TEXT("result_incomplete"));
        if(!Analytic::Certified::Detail::ValidPoint(values.QueryEnvelope))return failed(TEXT("result_envelope"));
        if(!values.SourceId||!values.SurfaceId||!values.PrimitiveId)return failed(TEXT("result_identity"));
        Analytic::RequireCurrentAnalyticNumericContext(GetNumericContract());if(!current())return failed(TEXT("context_after"));
        if(Snapshot->GetExternalLandscape()&&!Snapshot->GetExternalLandscape()->StrictlySeparated(request->GetValues().Query,values.QueryEnvelope))return failed(TEXT("landscape"));
        output=MakeShareable(new FPreparedNativeSolidGeometry(request,MoveTemp(values)));return true;
    }
    Analytic::EAnalyticNumericContract GetNumericContract() const final
    {return Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush;}
    EStaticCollisionBackend GetBackend() const final {return EStaticCollisionBackend::SurfaceAnalytic;}
    Analytic::FWorldHit SweepSingle(const Analytic::FWorldQuery& query) const final
    {
        PreparedExecution::FNativePlanarResponseCommitScope::RequireQueriesClosed();
        Analytic::RequireCurrentAnalyticNumericContext(GetNumericContract());
        Analytic::FWorldHit hit;const auto status=EvaluateSweep(query,hit);
        Analytic::RequireCurrentAnalyticNumericContext(GetNumericContract());
        return ReadResult(status,MoveTemp(hit));
    }
    // Optional per-corner Box manifold witness after a known targeted miss.
    // The result names its own surface and carries no sweep or response authority.
    bool TryExactBoxSupportContinuation(const Analytic::FWorldQuery& query,
        const Analytic::Certified::FNativeBoxCachedPlaneValues& seed,Analytic::FWorldHit& out) const
    {
        PreparedExecution::FNativePlanarResponseCommitScope::RequireQueriesClosed();
        out={};
        const auto numeric=Analytic::RequireCurrentAnalyticNumericContext(GetNumericContract());
        Analytic::FWorldHit hit;
        if(!EvaluateExactBoxSupportContinuation(query,seed,hit))return false;
        if(Analytic::RequireCurrentAnalyticNumericContext(GetNumericContract()).ControlFingerprint()!=numeric.ControlFingerprint()||
            !hit.bHit||hit.bStartPenetrating||hit.bSurfaceNormalMayVary||hit.PenetrationDepth!=0.||
            hit.GeometricErrorBoundCm!=0.||hit.SurfaceFeatureKind!=EContactFeatureKind::Face||
            hit.SourceId!=seed.SourceId||!hit.SurfaceId||hit.SurfaceId==seed.SurfaceId||
            !hit.PrimitiveId||hit.MaterialId!=seed.MaterialId)return false;
        out=ReadResult(EPreparedStaticQueryResult::KnownHit,MoveTemp(hit));return true;
    }
    // Serialized-support geometry description, not an executed native query.
    // Unknown is false and cleared output, never a sweep miss or a contact.
    bool DescribeSerializedSphereSupportValues(const PreparedExecution::FNativeSolidStaticValues& values,
        const PreparedExecution::FSerializedSphereSupportManifold& manifold,PreparedExecution::FSerializedSphereSupportGeometryValues& output) const
    {
        PreparedExecution::FNativePlanarResponseCommitScope::RequireQueriesClosed();
        output={};Analytic::RequireCurrentAnalyticNumericContext(GetNumericContract());
        PreparedExecution::FSerializedSphereSupportGeometryValues result;
        if(!EvaluateSerializedSphereSupport(values,manifold,result))return false;
        Analytic::RequireCurrentAnalyticNumericContext(GetNumericContract());
        if(!result.SourceId||!result.SurfaceId||!result.PrimitiveId||!Analytic::Certified::Detail::ValidPoint(result.Envelope.Hull))return false;
        if(Snapshot->GetExternalLandscape()&&!Snapshot->GetExternalLandscape()->StrictlySeparated(values.Query,result.Envelope.Hull))return false;
        output=MoveTemp(result);return true;
    }
    // Geometry/filter values only, including real UE trace options/ignores.
    // Caller must bind its actual phase/owner/body and retained Contact token.
    Analytic::FWorldHit SweepNativeFixedProbeValues(const Analytic::FWorldQuery& query,
        const PreparedExecution::FNativeSolidTraceValues& trace) const
    {
        PreparedExecution::FNativePlanarResponseCommitScope::RequireQueriesClosed();
        Analytic::RequireCurrentAnalyticNumericContext(GetNumericContract());
        Analytic::FWorldHit hit;const auto status=EvaluateNativeFixedProbe(query,trace,hit);
        Analytic::RequireCurrentAnalyticNumericContext(GetNumericContract());
        return ReadResult(status,MoveTemp(hit));
    }
    // false ONLY for unknown generic coverage. A known coverage followed by
    // unknown native trace/policy keeps the existing hard refusal. No cache.
    bool TrySweepNativeFixedProbeWithCoverage(const Analytic::FWorldQuery& query,
        const PreparedExecution::FNativeSolidTraceValues& trace,Analytic::FWorldHit& out) const;
    bool TrySweepAuthority(const Analytic::FWorldQuery& query,Analytic::FWorldHit& out) const final
    {out={};out=SweepSingle(query);return true;}
    bool HasAuthorityCoverage(const Analytic::FWorldQuery& query) const final
    {
        PreparedExecution::FNativePlanarResponseCommitScope::RequireQueriesClosed();
        Analytic::RequireCurrentAnalyticNumericContext(GetNumericContract());
        Analytic::FWorldHit hit;const auto status=EvaluateSweep(query,hit);
        Analytic::RequireCurrentAnalyticNumericContext(GetNumericContract());
        if(status==EPreparedStaticQueryResult::Unknown)return false;
        ReadResult(status,MoveTemp(hit));return true;
    }
    // Explicit canonical pose operation. Its geometric clearance is never
    // converted to KnownMiss for a retained legacy facet kernel. The consumer
    // may publish an empty penetration list only for this exact query volume.
    bool TryCanonicalFixedBoxOverlapClearance(const Analytic::FWorldQuery& query) const
    {
        PreparedExecution::FNativePlanarResponseCommitScope::RequireQueriesClosed();
        const Analytic::FWorldQuery captured=query;
        return CanonicalBoxOverlapReuseDetail::Run(captured,[this](){return Snapshot->GetGeometrySelection()->GetSelectedUnion();},GetNumericContract(),
            [this](const auto& q,auto& shape,auto& proof){return TryReuseCanonicalGroupedFixedBoxOverlapClearance(q,shape,proof);},
            [this](const auto& q,auto& hull){return BuildCanonicalFixedBoxOverlapEnvelopeForQuery(q,hull);},
            [this](const auto& q,auto& shape,auto& proof){return EvaluateCanonicalGroupedFixedBoxOverlapClearance(q,shape,proof);},
            [this](const auto& q,const auto& hull){return !Snapshot->GetExternalLandscape()||Snapshot->GetExternalLandscape()->StrictlySeparated(q,hull);},
            [this](){return MatchesPreparedSnapshot(Snapshot);});
    }
    // Distinct negative support operation. The supplied actual Box and four
    // support rays are all enclosed; strict whole-reference/external clearance
    // excludes a four-point reaction without claiming a nominal kernel miss.
    bool TryExcludeCanonicalBoxRestingSupport(const Analytic::FWorldQuery& box,
        TConstArrayView<Analytic::FWorldQuery> actualProbes) const
    {
        PreparedExecution::FNativePlanarResponseCommitScope::RequireQueriesClosed();
        const Analytic::FWorldQuery captured=box;
        Analytic::RequireCurrentAnalyticNumericContext(GetNumericContract());
        if(!MatchesPreparedSnapshot(Snapshot))return false;
        Analytic::Certified::FIntervalPoint hull;
        if(!Analytic::Certified::BuildCanonicalFixedBoxSupportProbeEnvelope(captured,actualProbes,hull))return false;
        Analytic::Certified::FReferenceBoundsStrictSeparationPtr proof;
        if(!EvaluateCanonicalFixedBoxOverlapClearance(captured,hull,proof)||!proof||
            !proof->Matches(Snapshot->GetGeometrySelection()->GetSelectedUnion(),hull))return false;
        if(Snapshot->GetExternalLandscape()&&
            !Snapshot->GetExternalLandscape()->StrictlySeparated(captured,hull))return false;
        Analytic::RequireCurrentAnalyticNumericContext(GetNumericContract());
        return MatchesPreparedSnapshot(Snapshot);
    }
    bool TryFindDeepestOverlap(const Analytic::FWorldQuery& query,Analytic::FWorldHit& out) const final
    {
        PreparedExecution::FNativePlanarResponseCommitScope::RequireQueriesClosed();
        out={};Analytic::RequireCurrentAnalyticNumericContext(GetNumericContract());
        Analytic::FWorldHit hit;const auto status=EvaluateOverlap(query,hit);
        Analytic::RequireCurrentAnalyticNumericContext(GetNumericContract());
        out=ReadResult(status,MoveTemp(hit));return true;
    }
    bool TryFindDeepestSphereProjection(const Analytic::FWorldQuery& query,
        const PreparedExecution::FNativeSphereProjectionScope& scope,Analytic::FWorldHit& out) const
    {
        PreparedExecution::FNativePlanarResponseCommitScope::RequireQueriesClosed();
        out={};Analytic::RequireCurrentAnalyticNumericContext(GetNumericContract());
        bool admitted=false;
        const auto admit=[&]()
        {
            if(admitted)return false;
            admitted=scope.Matches(query,*this);
            return admitted;
        };
        Analytic::FWorldHit hit;const auto status=EvaluateNativeSphereProjection(query,admit,hit);
        if(admitted&&!scope.Matches(query,*this))Unknown();
        Analytic::RequireCurrentAnalyticNumericContext(GetNumericContract());
        out=ReadResult(status,MoveTemp(hit));return true;
    }
    void VisitPlanarCandidates(const Analytic::FWorldQuery& query,TFunctionRef<void(const Analytic::FWorldHit&)> visitor) const final
    {
        PreparedExecution::FNativePlanarResponseCommitScope::RequireQueriesClosed();
        Analytic::RequireCurrentAnalyticNumericContext(GetNumericContract());
        TArray<Analytic::FWorldHit> candidates;if(!GatherPlanarCandidates(query,candidates))Unknown();
        Analytic::RequireCurrentAnalyticNumericContext(GetNumericContract());
        for(const auto& candidate:candidates)visitor(candidate);
    }
    void VisitTorusCandidates(const Analytic::FWorldQuery& query,TFunctionRef<void(const Analytic::FTorusSurface&)> visitor) const final
    {
        PreparedExecution::FNativePlanarResponseCommitScope::RequireQueriesClosed();
        Analytic::RequireCurrentAnalyticNumericContext(GetNumericContract());
        TArray<Analytic::FTorusSurface> candidates;if(!GatherTorusCandidates(query,candidates))Unknown();
        Analytic::RequireCurrentAnalyticNumericContext(GetNumericContract());
        for(const auto& candidate:candidates)visitor(candidate);
    }
    bool IsPlanarSupportTranslationCertified(const Analytic::FWorldQuery& query,TConstArrayView<FVector3d> points,
        const FVector3d& normal,const FVector3d& translation) const final
    {
        PreparedExecution::FNativePlanarResponseCommitScope::RequireQueriesClosed();
        Analytic::RequireCurrentAnalyticNumericContext(GetNumericContract());
        bool certified=false;if(!EvaluatePlanarSupport(query,points,normal,translation,certified))Unknown();
        Analytic::RequireCurrentAnalyticNumericContext(GetNumericContract());
        return certified;
    }

    bool DescribeNativeBoxPoseCorrectionWithCompleteClear(
        const PreparedExecution::FNativeSolidStaticRequestPtr& request,const FKinematicState& before,const FKinematicState& candidate,
        PreparedExecution::FNativeBoxPoseCorrectionGeometryValuesPtr& output,
        const PreparedExecution::FNativeCanonicalBoxSweepClearanceValuesPtr& completeClear) const
    {
        PreparedExecution::FNativePlanarResponseCommitScope::RequireQueriesClosed();
        output.Reset();Analytic::RequireCurrentAnalyticNumericContext(GetNumericContract());
        if(!request||request->GetSnapshot()!=Snapshot||!MatchesPreparedSnapshot(Snapshot)||
            !completeClear||!completeClear->RequiresGeneralBoxTransport()||completeClear->IsCurvedArrivalPrefix())return false;
        const auto& original=request->GetValues();
        PreparedExecution::FNativeBoxPoseCorrectionGeometryValuesPtr result;
        if(!EvaluateNativeBoxPoseCorrectionWithCompleteClear(original,before,candidate,result,completeClear)||!result||
            !result->Matches(Snapshot,original,before,candidate)||!result->MatchesEstablishedFloorBinding({})||
            !result->MatchesCurvedArrivalBinding({}))return false;
        if(Snapshot->GetExternalLandscape()&&!Snapshot->GetExternalLandscape()->StrictlySeparated(original.Query,result->GetEnvelopeForInspection()))return false;
        Analytic::RequireCurrentAnalyticNumericContext(GetNumericContract());
        if(!MatchesPreparedSnapshot(Snapshot))return false;output=MoveTemp(result);return true;
    }
protected:
    // Append the private hook without changing the existing evaluator slots.
    virtual bool EvaluateNativeBoxPoseCorrectionWithCompleteClear(const PreparedExecution::FNativeSolidStaticValues& source,
        const FKinematicState& before,const FKinematicState& candidate,PreparedExecution::FNativeBoxPoseCorrectionGeometryValuesPtr& out,
        const PreparedExecution::FNativeCanonicalBoxSweepClearanceValuesPtr&) const
    {return EvaluateNativeBoxPoseCorrectionGeometryValues(source,before,candidate,out,{},{});}
};
// Called synchronously inside the actual GT publication lease, before its
// single commit. The caller retains all Source authentication and UObject pins.
using FPreparedCanonicalQueryServiceFactoryGT=TFunctionRef<bool(const FPreparedWorldSnapshotPtr&,
    FPreparedCanonicalStaticQueryServicePtr&,FString&)>;
}
