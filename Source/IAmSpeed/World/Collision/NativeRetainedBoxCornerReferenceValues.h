#pragma once
#include "PreparedGeometrySelection.h"
#include "NativeFixedBoxReferenceMotionValues.h"
namespace Speed::PreparedExecution
{
class FNativeRetainedBoxCornerReferenceValues;
using FNativeRetainedBoxCornerReferenceValuesPtr=TSharedPtr<const FNativeRetainedBoxCornerReferenceValues,ESPMode::ThreadSafe>;
// ONE local curved-provider reference root. The constructor derives the tensor
// from the retained union, never from a caller-labelled tensor/root. This is
// values/provenance, not packet authentication, GT/Step/trace/ignore admission,
// whole-Box/whole-union earliest CCD, native normal/error/response or Ready.
class FNativeRetainedBoxCornerReferenceValues final
{
    FPreparedGeometrySelectionPtr Geometry;
    FNativeFixedBoxCornerReferenceValuesPtr CornerReference;
    Analytic::Prepared::FOwner Owner;
    int32 ProviderIndex=INDEX_NONE,FaceIndex=INDEX_NONE;
    Analytic::Certified::FReferenceDyadicCell UCell,VCell;
    FNativeRetainedBoxCornerReferenceValues()=default;
    static bool Policy(const Analytic::FWorldQuery& q,const Analytic::Prepared::FOwner& o)
    {
        // Same retained canonical Source/Surface/Group/Compact/Object/Trace
        // description as the current service. AuthorityEligible is deliberately
        // neither read nor changed: the real closed producer must authenticate
        // canonical inputs, native filters/ignored objects and the live Step.
        if(q.Shape!=Analytic::EQueryShape::Box||!q.bApplyCollisionFilter||!q.bAuthorityOnly||q.bExcludeAuthorityEligible||
            q.bObjectQuery||q.TraceChannel>=64||
            (AnalyticCertified::Bits(q.ReferenceNormal.X)&AnalyticCertified::Magnitude)||
            (AnalyticCertified::Bits(q.ReferenceNormal.Y)&AnalyticCertified::Magnitude)||
            (AnalyticCertified::Bits(q.ReferenceNormal.Z)&AnalyticCertified::Magnitude)||
            q.MinimumReferenceNormalDot!=-1||q.bAllowEstablishedFaceContactAtBoundary||
            (q.RequiredSourceId&&q.RequiredSourceId!=o.SourceId)||(q.RequiredSurfaceId&&q.RequiredSurfaceId!=o.SurfaceId)||
            (q.RequiredCanonicalGroupId&&q.RequiredCanonicalGroupId!=o.CanonicalGroupId)||!o.QueryEnabled||
            (o.CompactOptIn&&!q.bIncludeCompactPatches)||o.ObjectType>=64)return false;
        return (o.BlockingChannels&(uint64(1)<<q.TraceChannel))&&(q.BlockingObjectTypes&(uint64(1)<<o.ObjectType));
    }
public:
    static bool TryProve(FPreparedGeometrySelectionPtr geometry,FNativeFixedBoxReferenceMotionValuesPtr motion,
        int32 provider,const Analytic::Certified::FReferenceDyadicCell& u,const Analytic::Certified::FReferenceDyadicCell& v,
        const Analytic::Certified::FReferenceDyadicCell& time,int32 corner,int32 normalSign,
        Analytic::Certified::FDistanceBudget& budget,FNativeRetainedBoxCornerReferenceValuesPtr& out)
    {
        const auto keepPrior=out;out.Reset();
        Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
        if(!budget.Consume()||!geometry||!geometry->HasValidAliases()||
            geometry->GetModel()!=EPreparedGeometryModel::RetainedReference||!motion||!u.IsValid()||!v.IsValid()||!time.IsValid())return false;
        const auto reference=geometry->GetReference();const auto owners=geometry->GetNominalScene()->GetOwners();
        const auto mapping=reference->GetProviderFaceIndices();const auto faces=reference->GetFaces();
        if(provider<0||provider>=owners.Num()||provider>=mapping.Num()||provider>=geometry->GetSelectedUnion()->Num())return false;
        const int32 face=mapping[provider];
        if(face<0||face>=faces.Num()||owners[provider].FromPlane||faces[face].FromPlane||
            !Policy(motion->GetCapturedForInspection().Query,owners[provider]))return false;
        // First route is rectangular curved tensors only. A triangulated
        // plane's {a,c,b,b} chart is not a FBoundedPlane UV chart.
        Analytic::Certified::FIntervalTensorSurface original,restricted;
        if(!geometry->GetSelectedUnion()->CopySurfaceForQuery(provider,budget,original)||
            !Analytic::Certified::RestrictReferenceTensorDyadic(original,u,v,budget,restricted))return false;
        FNativeFixedBoxCornerReferenceValuesPtr root;
        if(!FNativeFixedBoxCornerReferenceValues::TryProveInTimeCell(MoveTemp(motion),restricted,corner,normalSign,time,budget,root)||!root)return false;
        TSharedPtr<FNativeRetainedBoxCornerReferenceValues,ESPMode::ThreadSafe> value(new FNativeRetainedBoxCornerReferenceValues);
        value->Geometry=MoveTemp(geometry);value->CornerReference=MoveTemp(root);value->Owner=owners[provider];
        value->ProviderIndex=provider;value->FaceIndex=face;value->UCell=u;value->VCell=v;
        Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
        out=MoveTemp(value);return true;
    }
    bool MatchesInputs(const FPreparedGeometrySelectionPtr& geometry,const FNativeSolidStaticValues& captured,int32 provider,
        const Analytic::Certified::FReferenceDyadicCell& u,const Analytic::Certified::FReferenceDyadicCell& v,
        const Analytic::Certified::FReferenceDyadicCell& time,int32 corner,int32 normalSign) const
    {
        return Geometry&&Geometry==geometry&&Geometry->HasValidAliases()&&Geometry->GetModel()==EPreparedGeometryModel::RetainedReference&&
            ProviderIndex==provider&&UCell.SameCode(u)&&VCell.SameCode(v)&&CornerReference&&
            CornerReference->MatchesInputsInTimeCell(captured,CornerReference->GetRootForInspection()->GetTensorForInspection(),corner,normalSign,time);
    }
    AnalyticCertified::Interval GetOriginalProviderU() const
    {return CornerReference?UCell.MapLocal(CornerReference->GetRootForInspection()->GetLocalRootBounds().Axis[0]):AnalyticCertified::Interval{};}
    AnalyticCertified::Interval GetOriginalProviderV() const
    {return CornerReference?VCell.MapLocal(CornerReference->GetRootForInspection()->GetLocalRootBounds().Axis[1]):AnalyticCertified::Interval{};}
    const auto& GetGeometryForInspection() const{return Geometry;}
    const auto& GetCornerReferenceForInspection() const{return CornerReference;}
    const auto& GetOwnerForInspection() const{return Owner;}
    const auto& GetFaceForInspection() const{return Geometry->GetReference()->GetFaces()[FaceIndex];}
    const auto& GetUCellForInspection() const{return UCell;}
    const auto& GetVCellForInspection() const{return VCell;}
    int32 GetProviderIndex() const{return ProviderIndex;}
    int32 GetFaceIndex() const{return FaceIndex;}
};
}
