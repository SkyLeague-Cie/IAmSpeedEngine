#pragma once
#include "PreparedSphereRequest.h"
#include "PreparedIntervalScene.h"
#include "PreparedSphereHitValues.h"
#include "PreparedSphereConsumedState.h"
#include "PreparedNativeSolidGeometry.h"
#include "PreparedSleepingSphereSupport.h"
#include "NativeBoxPlanarRefinementValues.h"
#include "NativeBoxGroundReuseValues.h"
#include "PreparedNativeAnalyticContactValues.h"
class USSubBody;class UBoxSubBody;class ISpeedComponent;class ISolidSweeper;
namespace Speed::PreparedExecution
{
namespace C=Analytic::Certified;
namespace NativeGeneralCachedFloorArrivalDetail
{
IAMSPEED_API bool Match(const FNativeSolidStaticValues&,const C::FNativeGeneralCachedFloorArrivalDescriptorBounds&,
    const SHitResult&,const SHitResult&,bool,const FNativeBoxPlanarRefinementValues&,float);
}
namespace NativeGeneralFloorMotionDetail
{
IAMSPEED_API bool Match(const FNativeSolidStaticValues&,const C::FNativeGeneralBoxFloorMotionDescriptorBounds&,
    const SHitResult&,const SHitResult&,bool,const FNativeBoxPlanarRefinementValues&,float);
}
// Complete rounded SBox hull partition, without precision knobs or contact
// thresholds. A nonempty candidate set is UNKNOWN, never an admitted miss.
IAMSPEED_API C::FReferenceCandidateSetPtr SelectWholeConsumedSphereHull(
    C::FIndexedIntervalUnionPtr Scene,const C::FZeroOffsetKinematicCenterReference& Model,
    float Horizon,double Radius,C::FDistanceBudget& Budget);
// Values-only selected partition. This is not a sealed native sweep ticket.
struct FSelectedWholeHullPartition
{
    FPreparedGeometrySelectionPtr Geometry;C::FReferenceCandidateSetPtr Partition;
    bool UsesGeometry(const FPreparedGeometrySelectionPtr& expected) const
    {return expected&&expected->HasValidAliases()&&Geometry==expected&&Partition&&Partition->GetScene()==expected->GetSelectedUnion();}
};
inline FSelectedWholeHullPartition SelectSelectedWholeConsumedSphereHull(FPreparedGeometrySelectionPtr geometry,
    const C::FZeroOffsetKinematicCenterReference& model,float horizon,double radius,C::FDistanceBudget& budget)
{
    if(!geometry||!geometry->HasValidAliases())return {};
    auto partition=SelectWholeConsumedSphereHull(geometry->GetSelectedUnion(),model,horizon,radius,budget);
    if(!partition||partition->GetScene()!=geometry->GetSelectedUnion())return {};
    return {MoveTemp(geometry),MoveTemp(partition)};
}
class FGroundSweepEvidence;
using FGroundSweepEvidencePtr=TSharedPtr<const FGroundSweepEvidence,ESPMode::ThreadSafe>;
class FComponentSweepEvidence;
using FComponentSweepEvidencePtr=TSharedPtr<const FComponentSweepEvidence,ESPMode::ThreadSafe>;
class IAMSPEED_API FGroundSweepEvidence final
{
    friend class FSphereMemberSweepScope;friend class FComponentSweepScope;friend class FStepScope;
    #if !UE_BUILD_SHIPPING
    friend IAMSPEED_API void AppendPreparedGroundEvidenceState(TArray<uint8>&,const FGroundSweepEvidencePtr&);
    #endif
    FNativeSphereRequestPtr Request;FPreparedGeometrySelectionPtr Geometry;C::FReferenceCandidateSetPtr Partition;
    enum class EKind:uint8 {Unknown,WholeHullExclusion,FirstContact};
    EKind Kind=EKind::Unknown;FSelectedSphereResponseQualityPtr Quality;SHitResult PendingHit;
    FSealedSphereSweepStatePtr StateSeed;
    bool Sealed=false;
    FGroundSweepEvidence()=default;
public:
    bool IsSealedWholeHullExclusion() const
    {return Sealed&&Kind==EKind::WholeHullExclusion&&Request&&Geometry&&Geometry==Request->GetGeometrySelection()&&Partition&&
        Partition->GetScene()==Geometry->GetSelectedUnion()&&Partition->GetCandidateIndices().IsEmpty();}
    bool IsSealedFirstContact() const
    {return Sealed&&Kind==EKind::FirstContact&&Request&&Geometry&&Geometry==Request->GetGeometrySelection()&&Quality&&
        Quality->GetGeometry()==Geometry&&Quality->GetStatus()==ESphereResponseQualityStatus::BoundedWithinInputMargin&&
        Quality->GetReference().ConsumedPrefixClear&&Quality->GetReference().Status==C::EReferenceSphereStatus::ReferenceContactBounded&&
        PendingHit.bHit&&PendingHit.bBlockingHit&&!PendingHit.PreparedGroundEvidence;}
    const FNativeSphereRequestPtr& GetCaptureForInspection() const {return Request;}
    const C::FReferenceCandidateSetPtr& GetCompletePartition() const {return Partition;}
};
// Closed World-query VALUES only, separate from historical single-Sphere
// StateSeed. A ground candidate need not win dynamic/cache selection, and this
// record grants no global event, native response, integration or readiness.
struct FNativeBoxGroundCacheValues
{
    SHitResult CurrentHit,GroundHit;int32 LastResolvedFrame=INDEX_NONE;
    bool HasGround=false,PlaneValid=false;FVector PlanePoint=FVector::ZeroVector,PlaneNormal=FVector::ZeroVector;
    float PlaneD=0;TWeakObjectPtr<UPrimitiveComponent> GroundComponent;
};
// Native outer cache producer, separate from OrdinaryGround's World/refiner
// result. Old cache metadata may be historical and bHit may remain false even
// when SweepTOI returns true. No geometry, support force or advance grant.
class FOrdinaryBoxGroundReuseObservation final
{
    friend class FComponentSweepScope;
    FStepIdentity EntryStep,DecisionStep,ReturnStep;FPreparedWorldSnapshotPtr Snapshot;
    const UBoxSubBody* Body=nullptr;const ISpeedComponent* Owner=nullptr;
    uint64 OwnerStableId=0,ScopeSerial=0;int32 BodyIndex=INDEX_NONE;
    FKinematicState OwnerState,BodyState;FCanonicalPreparedFrameClock FrameClock;
    SHitResult OldGround,EntryFuture,BeforeFuture,AfterFuture;
    FNativeBoxGroundCacheValues EntryCache,BeforeCache,AfterCache;
    TArray<const UPrimitiveComponent*> EntryIgnored,BeforeIgnored,AfterIgnored;
    FNativeBoxGroundReuseDecisionValues Decision;
    ENativeBoxGroundReuseOutcome Outcome=ENativeBoxGroundReuseOutcome::NotEntered;
    float Horizon=0,BeforeSeconds=0,AfterSeconds=0;uint8 CandidateFlags=0xff;
    bool Entered=false,NativeReturnObserved=false,ReturnedHit=false,CallbackClosed=false;
    FOrdinaryBoxGroundReuseObservation()=default;
public:
    bool IsClosedObservation() const{return CallbackClosed&&Snapshot&&Body&&Owner&&OwnerStableId&&ScopeSerial&&BodyIndex!=INDEX_NONE;}
    ENativeBoxGroundReuseOutcome GetOutcomeForInspection() const{return Outcome;}
};
class FOrdinaryGroundSweepEvidence final
{
    friend class FSphereMemberSweepScope;friend class FComponentSweepScope;
    FNativeSolidStaticRequestPtr Request;FPreparedNativeSolidGeometryPtr Geometry;
    SHitResult NamedGround;float Seconds=0,BoxRefinementHorizon=0;bool Closed=false,BoxRefinementReturned=false;
    bool BoxReleasedFloorMotion=false,BoxEstablishedFloorClear=false,BoxGeneralFirstFloorArrival=false,BoxGeneralCachedFloorArrival=false;
    SHitResult BoxMotionRawInput,BoxMotionRawHit;
    bool BoxNoNewUnmaskedCorner=false,CanonicalReferenceBoxSweepPathClear=false;FNativeBoxPlanarRefinementValues BoxRefinement;
    FNativeBoxGroundCacheValues BoxCache;
    struct FCase202Serial3Join
    {
        FNativeCase202AtlasObservationPtr Atlas;
        SHitResult FloorInput,FloorHit;
        FNativeBoxPlanarRefinementValues FloorRefinement;
        FWeakObjectPtr DynamicBall;
        ::AnalyticCertified::Interval DynamicGap;
        uint64 FilteredFloorKey=0,ProgressCurveKey=0;
    };
    TOptional<FCase202Serial3Join> Case202Serial3;
    FOrdinaryGroundSweepEvidence()=default;
public:
    bool IsClosedQueryValues() const
    {
        if(!Closed||!Request||!Geometry||Geometry->GetRequest()!=Request)return false;
        if(Geometry->GetValues().Status==ENativeSolidGeometryStatus::GeneralBoxFloorCachedArrivalDescriptor)
        {
            const auto& v=Request->GetValues();const auto& g=Geometry->GetValues().GeneralBoxCachedFloorArrivalDescriptor;
            if(!BoxGeneralCachedFloorArrival||BoxGeneralFirstFloorArrival||BoxReleasedFloorMotion||BoxNoNewUnmaskedCorner||
                CanonicalReferenceBoxSweepPathClear||!BoxRefinementReturned||!g.HasMatchingPhysicalPrefix()||
                !SameNativeSolidStaticValues(g.Captured,v)||
                !NativeGeneralCachedFloorArrivalDetail::Match(v,g,BoxMotionRawInput,BoxMotionRawHit,true,BoxRefinement,BoxRefinementHorizon)||
                C::KinematicCenterDetail::Bits(Seconds)!=C::KinematicCenterDetail::Bits(g.CachedPrefix->GetMaximumAdvanceSeconds()))return false;
            bool bindingAmbiguous=false;
            const FPreparedSourceContactBinding* binding=Request->GetSnapshot()->FindUniqueContactBinding(BoxMotionRawHit.SourceId,bindingAmbiguous);
            if(bindingAmbiguous)return false;
            if(!binding||!binding->SourceIdentity||!binding->ContactIdentity||binding->ContactToken.IsExplicitlyNull())return false;
            auto expected=BoxMotionRawHit;expected.Component=binding->ContactToken;expected.FrameTag=v.FrameClock.OwnerLocalFrame;
            expected.bHit=expected.bBlockingHit=true;return SameNativeAnalyticContactValues(NamedGround,expected);
        }
        if(Geometry->GetValues().Status==ENativeSolidGeometryStatus::GeneralBoxFloorMotionDescriptor)
        {
            const auto& v=Request->GetValues();const auto& g=Geometry->GetValues().GeneralBoxFloorMotionDescriptor;
            if(BoxGeneralCachedFloorArrival||BoxReleasedFloorMotion||BoxNoNewUnmaskedCorner||CanonicalReferenceBoxSweepPathClear||
                v.Shape!=ENativeSolidQueryShape::Box||!g.HasMatchingPhysicalMotion()||
                !SameNativeSolidStaticValues(g.Captured,v)||
                !NativeGeneralFloorMotionDetail::Match(v,g,BoxMotionRawInput,BoxMotionRawHit,BoxRefinementReturned,BoxRefinement,BoxRefinementHorizon)||
                C::KinematicCenterDetail::Bits(Seconds)!=C::KinematicCenterDetail::Bits(g.Motion->GetClearSeconds()))return false;
            if(g.Motion->GetStatus()==C::EGeneralPhysicalFloorMotion::WholePhysicalClear)
                return !BoxGeneralFirstFloorArrival&&!BoxRefinementReturned&&!NamedGround.bHit&&!NamedGround.bBlockingHit&&
                    C::KinematicCenterDetail::Bits(Seconds)==C::KinematicCenterDetail::Bits(v.Horizon);
            if(!BoxGeneralFirstFloorArrival||!BoxRefinementReturned||
                g.Motion->GetStatus()!=C::EGeneralPhysicalFloorMotion::FirstAdjacentArrival)return false;
            bool bindingAmbiguous=false;
            const FPreparedSourceContactBinding* binding=Request->GetSnapshot()->FindUniqueContactBinding(BoxMotionRawHit.SourceId,bindingAmbiguous);
            if(bindingAmbiguous)return false;
            if(!binding||!binding->SourceIdentity||!binding->ContactIdentity||binding->ContactToken.IsExplicitlyNull())return false;
            auto expected=BoxMotionRawHit;expected.Component=binding->ContactToken;expected.FrameTag=v.FrameClock.OwnerLocalFrame;
            expected.bHit=expected.bBlockingHit=true;
            return SameNativeAnalyticContactValues(NamedGround,expected);
        }
        if(BoxGeneralFirstFloorArrival||BoxGeneralCachedFloorArrival)return false;
        if(BoxEstablishedFloorClear&&!BoxReleasedFloorMotion)return false;
        if(BoxReleasedFloorMotion)
        {
            const auto& g=Geometry->GetValues().GeneralBoxFloorDescriptor;
            if(BoxEstablishedFloorClear)
            {
                if(!BoxRefinement.EstablishedMask||g.ObservationRequestIdentity!=Request.Get()||
                    !SameNativeAnalyticContactValues(BoxMotionRawInput,BoxMotionRawHit)||
                    !C::FloorNativeSweepDetail::SameVector(BoxMotionRawInput.ImpactNormal,g.CachedPlane.Normal)||
                    !C::FloorNativeSweepDetail::SameVector(BoxMotionRawInput.ImpactPoint,g.CachedPlane.Point))return false;
            }
            else if(BoxRefinement.EstablishedMask||g.ObservationRequestIdentity)return false;
            return Request->GetValues().Shape==ENativeSolidQueryShape::Box&&!NamedGround.bHit&&!NamedGround.bBlockingHit&&
                !CanonicalReferenceBoxSweepPathClear&&!BoxNoNewUnmaskedCorner&&!BoxRefinementReturned&&
                Geometry->GetValues().Status==ENativeSolidGeometryStatus::GeneralBoxFloorRefinerDescriptor&&
                C::MatchesNativeGeneralBoxFloorPhysicalClear(Geometry->GetValues().GeneralBoxFloorDescriptor)&&
                SameNativeSolidStaticValues(Geometry->GetValues().GeneralBoxFloorDescriptor.Captured,Request->GetValues())&&
                BoxRefinement.Computed&&BoxRefinement.Outcome==ENativeBoxPlanarRefinementOutcome::NoCorner&&
                BoxRefinement.Established&&!BoxRefinement.InitialEvent&&!BoxRefinement.WitnessComputed&&
                BoxRefinement.FirstCorner==INDEX_NONE&&
                C::KinematicCenterDetail::Bits(Seconds)==C::KinematicCenterDetail::Bits(Request->GetValues().Horizon)&&
                C::KinematicCenterDetail::Bits(BoxRefinementHorizon)==C::KinematicCenterDetail::Bits(Request->GetValues().Horizon)&&
                C::KinematicCenterDetail::Bits(BoxRefinement.Horizon)==C::KinematicCenterDetail::Bits(Request->GetValues().Horizon)&&
                C::KinematicCenterDetail::Bits(BoxRefinement.FirstTime)==C::KinematicCenterDetail::Bits(Request->GetValues().Horizon);
        }
        if(Geometry->GetValues().Status==ENativeSolidGeometryStatus::GeneralBoxFloorRefinerDescriptor)return false;
        if(CanonicalReferenceBoxSweepPathClear)
            return Request->GetValues().Shape==ENativeSolidQueryShape::Box&&!NamedGround.bHit&&!NamedGround.bBlockingHit&&
                !BoxRefinementReturned&&!BoxNoNewUnmaskedCorner&&
                C::KinematicCenterDetail::Bits(Seconds)==C::KinematicCenterDetail::Bits(Request->GetValues().Horizon)&&
                Geometry->GetValues().Status==ENativeSolidGeometryStatus::CanonicalReferenceBoxSweepPathClear&&
                MatchesNativeCanonicalBoxSweepClearanceValues(Request->GetValues(),Request->GetSnapshot()->GetGeometrySelection(),
                    Geometry->GetValues().CanonicalBoxClearance,Geometry->GetValues().QueryEnvelope);
        if(Geometry->GetValues().Status==ENativeSolidGeometryStatus::CanonicalReferenceBoxSweepPathClear)return false;
        if(Geometry->GetValues().Status==ENativeSolidGeometryStatus::GeneralBoxCurvedActualMotionClear)
            return Request->GetValues().Shape==ENativeSolidQueryShape::Box&&!NamedGround.bHit&&!NamedGround.bBlockingHit&&
                !BoxRefinementReturned&&!BoxNoNewUnmaskedCorner&&
                C::KinematicCenterDetail::Bits(Seconds)==C::KinematicCenterDetail::Bits(Request->GetValues().Horizon)&&
                MatchesNativeCurvedBoxActualMotionClearValues(Request->GetSnapshot(),Request->GetValues(),
                    Geometry->GetValues().CanonicalBoxClearance,Geometry->GetValues().QueryEnvelope);
        if(Geometry->GetValues().Status==ENativeSolidGeometryStatus::GeneralBoxCurvedArrivalDescriptor)
        {
            const auto& v=Request->GetValues();const auto snapshot=Request->GetSnapshot();
            float seconds=0;Analytic::FWorldHit hit;
            if(v.Shape!=ENativeSolidQueryShape::Box||BoxRefinementReturned||BoxNoNewUnmaskedCorner||
                !ReadNativeCurvedBoxArrivalValues(snapshot,v,Geometry->GetValues().CanonicalBoxClearance,seconds,hit)||
                C::KinematicCenterDetail::Bits(Seconds)!=C::KinematicCenterDetail::Bits(seconds))return false;
            if(Geometry->GetValues().CanonicalBoxClearance->IsCase202Serial3Prefix()!=Case202Serial3.IsSet())return false;
            if(Case202Serial3.IsSet())
            {
                const auto& joined=Case202Serial3.GetValue();
                if(!joined.Atlas||!joined.Atlas->PositivePrefixPartition||
                    !::AnalyticCertified::Valid(joined.DynamicGap)||joined.DynamicGap.Lower<=0.||
                    joined.DynamicBall.IsExplicitlyNull()||!joined.FilteredFloorKey||!joined.ProgressCurveKey||
                    joined.FilteredFloorKey==joined.ProgressCurveKey||!joined.FloorRefinement.Computed||
                    !joined.FloorRefinement.WitnessComputed||!joined.FloorRefinement.InitialEvent||
                    joined.FloorRefinement.Established||joined.FloorRefinement.FirstCorner!=joined.FloorHit.ContactFeatureIndexThis||
                    !joined.FloorHit.SourceId||joined.FloorHit.SourceId!=joined.FloorInput.SourceId||
                    joined.FloorHit.SurfaceId!=joined.FloorInput.SurfaceId||joined.FloorHit.TOI!=0.f||
                    !joined.FloorHit.Component.IsValid())return false;
            }
            const auto& floor=Geometry->GetValues().GeneralBoxFloorDescriptor;
            if(floor.ObservationCurvedProvider!=INDEX_NONE)
            {
                const auto horizon=C::KinematicCenterDetail::Bits(v.Horizon);
                if(!floor.Complete||floor.PhysicalClear||!floor.ObservationFinitePartition||
                    floor.ObservationRequestIdentity!=Request.Get()||!SameNativeSolidStaticValues(floor.Captured,v)||
                    !BoxRefinement.Computed||BoxRefinement.Outcome!=ENativeBoxPlanarRefinementOutcome::NoCorner||
                    !BoxRefinement.Established||!BoxRefinement.EstablishedMask||BoxRefinement.InitialEvent||BoxRefinement.WitnessComputed||
                    BoxRefinement.FirstCorner!=INDEX_NONE||
                    C::KinematicCenterDetail::Bits(BoxRefinementHorizon)!=horizon||
                    C::KinematicCenterDetail::Bits(BoxRefinement.Horizon)!=horizon||
                    C::KinematicCenterDetail::Bits(BoxRefinement.FirstTime)!=horizon||
                    !SameNativeAnalyticContactValues(BoxMotionRawInput,BoxMotionRawHit)||
                    !C::FloorNativeSweepDetail::SameVector(BoxRefinement.Normal,floor.CachedPlane.Normal)||
                    !C::FloorNativeSweepDetail::SameVector(BoxRefinement.Point,floor.CachedPlane.Point))return false;
            }
            bool ambiguous=false;const auto* binding=snapshot->FindUniqueContactBinding(hit.SourceId,ambiguous);
            SHitResult expected;
            return !ambiguous&&binding&&BuildNativeAnalyticContactValues(hit,seconds,v.FrameClock.OwnerLocalFrame,*binding,expected)&&
                SameNativeAnalyticContactValues(NamedGround,expected);
        }
        if(Geometry->GetValues().Status==ENativeSolidGeometryStatus::CanonicalAxialBoxPlaneDescriptor)
        {
            const auto& value=Geometry->GetValues().CanonicalAxialBoxPlane;
            return Request->GetValues().Shape==ENativeSolidQueryShape::Box&&NamedGround.bHit&&NamedGround.bBlockingHit&&
                BoxRefinementReturned&&!BoxNoNewUnmaskedCorner&&value&&BoxRefinement.Computed&&BoxRefinement.WitnessComputed&&
                BoxRefinement.Outcome==ENativeBoxPlanarRefinementOutcome::Hit&&!BoxRefinement.InitialEvent&&!BoxRefinement.Established&&
                C::KinematicCenterDetail::Bits(BoxRefinementHorizon)==C::KinematicCenterDetail::Bits(value->GetArrivalForInspection().InsideSeconds)&&
                C::KinematicCenterDetail::Bits(Seconds)==C::KinematicCenterDetail::Bits(value->GetArrivalForInspection().ClearPrefixSeconds)&&
                MatchesNativeCanonicalAxialBoxPlaneValues(Request->GetValues(),Request->GetSnapshot()->GetGeometrySelection(),
                    Request->GetSnapshot()->GetNominalScene(),value,Geometry->GetValues().QueryEnvelope);
        }
        if(BoxNoNewUnmaskedCorner)
            return Request->GetValues().Shape==ENativeSolidQueryShape::Box&&!NamedGround.bHit&&!BoxRefinementReturned&&
                Geometry->GetValues().Status==ENativeSolidGeometryStatus::BoxFloorSupportDescriptor&&Geometry->GetValues().BoxSupport.Complete&&
                BoxRefinement.Computed&&BoxRefinement.Outcome==ENativeBoxPlanarRefinementOutcome::NoCorner&&
                BoxRefinement.Established&&!BoxRefinement.InitialEvent&&BoxRefinement.EstablishedMask&&BoxRefinement.FirstCorner==INDEX_NONE&&
                C::KinematicCenterDetail::Bits(BoxRefinementHorizon)==C::KinematicCenterDetail::Bits(Request->GetValues().Horizon)&&
                C::KinematicCenterDetail::Bits(BoxRefinement.Horizon)==C::KinematicCenterDetail::Bits(Request->GetValues().Horizon)&&
                C::KinematicCenterDetail::Bits(BoxRefinement.FirstTime)==C::KinematicCenterDetail::Bits(Request->GetValues().Horizon);
        if(Geometry->GetValues().Status==ENativeSolidGeometryStatus::BoxFloorSupportDescriptor)return false;
        if(Geometry->GetValues().Status==ENativeSolidGeometryStatus::BoxInitialPlaneDescriptor)
            return NamedGround.bHit&&BoxRefinementReturned&&BoxRefinement.Computed&&
                BoxRefinement.Outcome==ENativeBoxPlanarRefinementOutcome::Hit&&BoxRefinement.InitialEvent&&!BoxRefinement.Established&&
                !BoxRefinement.EstablishedMask&&C::KinematicCenterDetail::Bits(BoxRefinement.FirstTime)==0&&
                C::KinematicCenterDetail::Bits(BoxRefinementHorizon)==C::KinematicCenterDetail::Bits(Request->GetValues().Horizon);
        return Request->GetValues().Shape!=ENativeSolidQueryShape::Box||!NamedGround.bHit||
            (BoxRefinementReturned&&BoxRefinementHorizon>0&&BoxRefinementHorizon<=Request->GetValues().Horizon);
    }
    bool IsCurvedBoxArrivalValues() const
    {return Geometry&&Geometry->GetValues().Status==ENativeSolidGeometryStatus::GeneralBoxCurvedArrivalDescriptor&&IsClosedQueryValues();}
    bool IsCanonicalAxialBoxPlaneContactValues() const
    {return IsClosedQueryValues()&&Geometry->GetValues().Status==ENativeSolidGeometryStatus::CanonicalAxialBoxPlaneDescriptor;}
    bool IsReleasedBoxFloorMotionValues() const{return IsClosedQueryValues()&&BoxReleasedFloorMotion;}
    bool IsEstablishedBoxFloorClearValues() const{return BoxEstablishedFloorClear&&IsReleasedBoxFloorMotionValues();}
    bool IsFirstGeneralBoxFloorContactValues() const{return IsClosedQueryValues()&&(BoxGeneralFirstFloorArrival||BoxGeneralCachedFloorArrival);}
    bool IsWholeGeneralBoxFloorClearValues() const
    {return IsClosedQueryValues()&&Geometry->GetValues().Status==ENativeSolidGeometryStatus::GeneralBoxFloorMotionDescriptor&&
        Geometry->GetValues().GeneralBoxFloorMotionDescriptor.Motion->GetStatus()==C::EGeneralPhysicalFloorMotion::WholePhysicalClear;}
    float GetGeneralFloorMotionMaximumAdvanceSeconds() const
    {return !Geometry?0.f:BoxGeneralCachedFloorArrival?Geometry->GetValues().GeneralBoxCachedFloorArrivalDescriptor.CachedPrefix->GetMaximumAdvanceSeconds():
        Geometry->GetValues().Status==ENativeSolidGeometryStatus::GeneralBoxFloorMotionDescriptor?
            Geometry->GetValues().GeneralBoxFloorMotionDescriptor.Motion->GetClearSeconds():0.f;}
    const C::FNativeBoxCachedPlaneValues& GetFirstGeneralFloorPlaneForInspection() const
    {return BoxGeneralCachedFloorArrival?Geometry->GetValues().GeneralBoxCachedFloorArrivalDescriptor.Plane:Geometry->GetValues().GeneralBoxFloorMotionDescriptor.Plane;}
    bool HasMatchingFirstGeneralFloorGeometry() const
    {return Geometry&&(BoxGeneralCachedFloorArrival?Geometry->GetValues().Status==ENativeSolidGeometryStatus::GeneralBoxFloorCachedArrivalDescriptor&&
        Geometry->GetValues().GeneralBoxCachedFloorArrivalDescriptor.HasMatchingPhysicalPrefix():BoxGeneralFirstFloorArrival&&
        Geometry->GetValues().Status==ENativeSolidGeometryStatus::GeneralBoxFloorMotionDescriptor&&Geometry->GetValues().GeneralBoxFloorMotionDescriptor.HasMatchingPhysicalMotion());}
    const FPreparedNativeSolidGeometryPtr& GetGeneralFloorMotionDescriptorForInspection() const{return Geometry;}
    const FPreparedNativeSolidGeometryPtr& GetReleasedFloorDescriptorForInspection() const{return Geometry;}
    bool IsNoNewBoxCornerValues() const{return IsClosedQueryValues()&&BoxNoNewUnmaskedCorner;}
    bool IsCanonicalReferenceBoxSweepPathClearValues() const{return IsClosedQueryValues()&&CanonicalReferenceBoxSweepPathClear;}
    bool IsCurvedBoxActualMotionClearValues() const
    {return IsClosedQueryValues()&&Geometry->GetValues().Status==ENativeSolidGeometryStatus::GeneralBoxCurvedActualMotionClear;}
    const FNativeCanonicalBoxSweepClearanceValuesPtr& GetCanonicalBoxClearanceForInspection() const {return Geometry->GetValues().CanonicalBoxClearance;}
    const FNativeSolidStaticRequestPtr& GetRequestForInspection() const {return Request;}
    const SHitResult& GetGroundValuesForInspection() const {return NamedGround;}
};
// Closed native branch observation, deliberately distinct from ground-query
// evidence. Geometry joins serialized support to the finite floor and excludes
// other static providers; no query, tangency or advancement is asserted.
class FOrdinarySleepingSupportObservation final
{
    friend class FSphereMemberSweepScope;friend class FComponentSweepScope;
    FStepIdentity Step;FPreparedWorldSnapshotPtr Snapshot;FNativeSolidStaticValues Intrinsic;
    FNativeSleepingSupportDecisionValues Decision;FSerializedSphereSupportGeometryValues Geometry;
    uint64 MemberSerial=0;bool Closed=false;
    FOrdinarySleepingSupportObservation()=default;
public:
    bool IsClosedObservation() const {return Closed&&Snapshot&&MemberSerial&&Geometry.SourceId&&Geometry.SurfaceId&&Geometry.PrimitiveId;}
};
// Records actual return values, including false. Dynamic results below remain
// observations, NOT collision/clearance certificates for Box/Sphere/Wheel.
struct FBodySweepEvidence
{
    TWeakObjectPtr<USSubBody> Body;const USSubBody* RawBodyIdentity=nullptr;int32 Index=INDEX_NONE;
    bool Returned=false,ReportedHit=false;float ReportedTOI=0;
    // Actual callback values, including a false return with a retained cache.
    // These observations do not certify World/dynamic/cache geometry.
    bool FutureHitObserved=false;SHitResult ReturnedFutureHit;
    FGroundSweepEvidencePtr Ground;
    TSharedPtr<const FOrdinaryGroundSweepEvidence,ESPMode::ThreadSafe> OrdinaryGround;
    TSharedPtr<const FOrdinarySleepingSupportObservation,ESPMode::ThreadSafe> SleepingSupportBypass;
    TSharedPtr<FOrdinaryBoxGroundReuseObservation,ESPMode::ThreadSafe> BoxGroundReuse;
    FSealedSphereSweepStatePtr FinalSphereState; // linked to actual final body return, not just ground callback
};
class IAMSPEED_API FComponentSweepEvidence final
{
    friend class FComponentSweepScope;friend class FStepScope;friend class FSphereMemberSweepScope;
    FStepIdentity Step;const ISpeedComponent* Owner=nullptr;uint64 OwnerStableId=0,ScopeSerial=0;
    FKinematicState SweepOwnerState;TArray<FBodySweepEvidence> Bodies;
    // Actual component minimum AFTER PickResolver/reorientation/PairKey. Keep
    // the origin body separately: a dynamic resolver may belong to another owner.
    bool SelectionObserved=false,SelectedHit=false;int32 SelectedBodyIndex=INDEX_NONE;
    float SelectedSeconds=0;uint64 SelectedPairKey=0;
    TWeakObjectPtr<USSubBody> SelectedResolver;
    // Identity values only. The future World selector still has to join the
    // actual sort ID and full batch; these fields grant no effect permission.
    const USSubBody* SelectedRawResolver=nullptr;uint32 SelectedResolverGTObjectId=0;
    const ISpeedComponent* SelectedResolverOwner=nullptr;int32 SelectedResolverBodyIndex=INDEX_NONE;
    SHitResult SelectedRawHit,SelectedOrientedHit;
    FStepIdentity CompletionStep;
    bool Sealed=false;
    FComponentSweepEvidence()=default;
public:
    bool IsSealedInventory() const {return Sealed;}
    TConstArrayView<FBodySweepEvidence> GetBodies() const {return MakeArrayView(Bodies);}
};
class IAMSPEED_API FComponentSweepScope final
{
    friend class ::ISpeedComponent;friend class ::UBoxSubBody;friend class FSphereMemberSweepScope;friend class FStepScope;
    TSharedPtr<FComponentSweepEvidence,ESPMode::ThreadSafe> Pending;
    bool Active=false,Completed=false,Interrupted=false;int32 CurrentIndex=INDEX_NONE,NextIndex=0;
    const USSubBody* TerminalRsRawBody=nullptr;
    explicit FComponentSweepScope(const ISpeedComponent& Owner);
    void BeginBody(int32 Index,USSubBody* Body);
    void ObserveBody(USSubBody* Body,bool Hit,float TOI);
    void ObserveWinner(int32 BodyIndex,const SHitResult& Raw,const SHitResult& Oriented,const TWeakObjectPtr<USSubBody>& Resolver,const USSubBody* RawResolver,uint64 PairKey);
    FComponentSweepEvidencePtr Finish(const ::SComponentTOI& ActualResult);
    void SealGround(FGroundSweepEvidencePtr Ground);
    void SealOrdinaryGround(TSharedPtr<const FOrdinaryGroundSweepEvidence,ESPMode::ThreadSafe>);
    static void ReadBoxGroundCacheValues(const UBoxSubBody&,FNativeBoxGroundCacheValues&);
    static void ReadBoxOuterState(const UBoxSubBody&,FNativeBoxGroundCacheValues&,SHitResult&,TArray<const UPrimitiveComponent*>&);
    static void BeginNativeBoxOuterSweep(const UBoxSubBody*,const SHitResult&,float);
    static void BeginNativeBoxGroundReuse(const UBoxSubBody*,const SHitResult&,float,const float&,uint8);
    static void FinishNativeBoxGroundReuse(const UBoxSubBody*,const FNativeBoxGroundReuseDecisionValues&,bool,const float&);
    static void CloseNativeBoxOuterReturn(const UBoxSubBody*,bool,float);
    static void ObserveCase202Serial3DynamicDispatches(const UBoxSubBody*,bool,bool,bool,bool);
    static TSharedPtr<FOrdinaryBoxGroundReuseObservation,ESPMode::ThreadSafe> ReadNativeBoxOuterSite(const UBoxSubBody*,FStepIdentity&);
    void SealSleepingSupport(TSharedPtr<const FOrdinarySleepingSupportObservation,ESPMode::ThreadSafe>);
    // Actual Box member only: consume a plane descriptor through the real
    // native refinement, never a contact synthesized at a geometric bracket.
    static bool EvaluateOrdinaryNativeBoxGround(const UWorld*,const ISolidSweeper*,const UBoxSubBody*,
        FVector,FVector,const FCollisionShape&,const FCollisionQueryParams&,float,SHitResult&);
    bool CloseInterruptedTerminalRsCapture() noexcept;
    bool CloseInterruptedPostSyncRsCapture() noexcept;
public:
    // Authenticates only the currently executing ordinary solid World call,
    // using raw GT body metadata before dereferencing the supplied interface.
    static bool EvaluateOrdinaryNativeGround(const UWorld*,const ISolidSweeper*,const USolidSubBody*,
        FVector,FVector,const FCollisionShape&,const FCollisionQueryParams&,float,SHitResult&);
    static bool ReadNativeSolidSite(const ISolidSweeper*,const USolidSubBody*,const ISpeedComponent*&,int32&) noexcept;
    ~FComponentSweepScope() noexcept;
    FComponentSweepScope(const FComponentSweepScope&)=delete;
    FComponentSweepScope& operator=(const FComponentSweepScope&)=delete;
};
}
