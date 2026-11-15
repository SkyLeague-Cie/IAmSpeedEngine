// Include in the native execution TU, after the Rs quality implementation.
namespace Speed::PreparedExecution
{
bool BuildFirstArrivalRsHitValues(FPreparedGeometrySelectionPtr geometry,
    FFirstArrivalRsResponseQualityPtr quality,uint32 frame,
    Analytic::Certified::FDistanceBudget& budget,FFirstArrivalRsHitValues& out,FString& reason)
{
    out={};reason.Reset();const auto refuse=[&](const TCHAR* why){reason=why;return false;};
    if(!geometry||!geometry->HasValidAliases()||!quality||
        quality->GetStatus()!=ESphereResponseQualityStatus::BoundedWithinInputMargin||
        !quality->GetArrival()||!quality->GetArrival()->GetInitial()||
        !quality->GetArrival()->GetInitial()->GetClosestAtHeight()||
        quality->GetArrival()->GetInitial()->GetClosestAtHeight()->GetGeometry()!=geometry||
        !quality->MatchesPolicy(quality->GetArrival(),quality->GetPolicy()))
        return refuse(TEXT("Rs named hit requires the same immutable bounded quality/geometry"));
    const auto arrival=quality->GetArrival();const auto& r=arrival->GetReference();
    const auto& input=arrival->GetInitial()->GetInput();const auto& scalar=quality->GetReboundedScalar();
    if(r.Status!=Analytic::Certified::EReferenceSphereStatus::ReferenceContactBounded||!r.ConsumedPrefixClear||
        r.Contact.Leaves.IsEmpty()||!quality->UsesRadialPoint()||!FMath::IsFinite(r.ConsumedSeconds)||
        r.ConsumedSeconds<0||r.ConsumedSeconds>=arrival->GetHorizon()||!geometry->GetReference())
        return refuse(TEXT("Rs first-contact/prefix/seconds/radial branch is unavailable"));
    const auto owners=geometry->GetNominalScene()->GetOwners();
    const auto contracts=geometry->GetReference()->GetSphereResponseContracts();
    const auto mapping=geometry->GetReference()->GetProviderFaceIndices();
    const auto& first=r.Contact.Leaves[0];
    if(!owners.IsValidIndex(first.SurfaceIndex))return refuse(TEXT("Rs first-contact identity unavailable"));
    const auto& id=owners[first.SurfaceIndex];
    if(!id.SourceId||!id.SurfaceId)return refuse(TEXT("Rs named hit has no static provider identity"));
    // The first tuple is only a reference for equality over ALL leaves.
    for(const auto& leaf:r.Contact.Leaves)
    {
        if(!budget.Consume()||!owners.IsValidIndex(leaf.SurfaceIndex)||
            !contracts.IsValidIndex(leaf.SurfaceIndex)||!mapping.IsValidIndex(leaf.SurfaceIndex))
            return refuse(TEXT("Rs ALL-leaf identity/contract budget unavailable"));
        const auto& o=owners[leaf.SurfaceIndex];const auto& c=contracts[leaf.SurfaceIndex];
        if(!c.StructurallyValid(mapping[leaf.SurfaceIndex])||!c.IsRadialReacquisitionV1()||
            o.ObjectType!=ECC_WorldStatic||o.SourceId!=id.SourceId||o.SurfaceId!=id.SurfaceId||
            o.FeatureId!=id.FeatureId||o.PrimitiveId!=id.PrimitiveId||
            o.CanonicalGroupId!=id.CanonicalGroupId||o.MaterialId!=id.MaterialId)
            return refuse(TEXT("Rs ALL-leaf source/feature/primitive/group/material identity differs"));
    }
    Analytic::FWorldHit h;h.bHit=true;h.Location=scalar.Location;h.Point=scalar.ImpactPoint;h.Normal=scalar.NormalStored;
    h.QueryPoint=quality->GetResponsePoint();h.bSurfaceNormalMayVary=true;h.GeometricErrorBoundCm=scalar.SurfaceErrorCm;
    h.SourceId=id.SourceId;h.SurfaceId=id.SurfaceId;h.FeatureId=id.FeatureId;h.PrimitiveId=id.PrimitiveId;
    h.CanonicalGroupId=id.CanonicalGroupId;h.MaterialId=id.MaterialId;
    const auto hit=SHitResult::FromAnalyticHitAtSeconds(h,r.ConsumedSeconds,frame);
    const auto n=hit.ImpactNormal.GetSafeNormal();
    const auto p=SphereStaticResponse::Point(hit.Location,input.PhysicalRadius,hit.ImpactPoint,hit.ImpactNormal,
        hit.SurfaceId,hit.bSurfaceNormalMayVary,hit.bStartPenetrating,input.StaticContactOffset);
    if(!FMath::IsFinite(hit.GeometricErrorBoundCm)||hit.GeometricErrorBoundCm<0||
        double(hit.GeometricErrorBoundCm)<scalar.SurfaceErrorCm||
        !quality->MatchesConsumedValues(hit.TOI,hit.Location,hit.ImpactPoint,hit.ImpactNormal,n,p)||
        !ResponseQualityDetail::SamePoint(p,hit.ContactPointThis)||
        !hit.Component.IsExplicitlyNull()||!hit.SubBody.IsExplicitlyNull()||hit.PreparedGroundEvidence||
        !SphereStaticResponse::UsesRadialPoint(hit.SurfaceId,hit.bSurfaceNormalMayVary,hit.bStartPenetrating,input.StaticContactOffset))
        return refuse(TEXT("Rs named hit scalar/normal/Rp bits or outward witness bound differs"));
    out={MoveTemp(geometry),MoveTemp(quality),hit};return true;
}
}
