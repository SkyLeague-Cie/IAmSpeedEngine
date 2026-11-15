// Included in PreparedPhysicalExecution.cpp after ResponseQualityDetail. No
// second evaluation at Rc: retain the SAME immutable GeneralFirstArrivalRs contact.
namespace Speed::PreparedExecution
{
bool FGeneralFirstArrivalRsPolicyAdmission::TryBuild(FPreparedGeometrySelectionPtr geometry,
    TFunctionRef<bool(const Analytic::Prepared::FOwner&)> accepts,
    Analytic::Certified::FDistanceBudget& budget,FPtr& out,FString& reason)
{
    out.Reset();reason.Reset();
    const auto numeric=Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
    const auto refuse=[&](const TCHAR* why){reason=why;return false;};
    if(!geometry||!geometry->HasValidAliases()||geometry->GetModel()!=EPreparedGeometryModel::RetainedReference||
        !geometry->GetReference()||!geometry->GetSelectedUnion())return refuse(TEXT("General Sphere policy source unavailable"));
    const auto owners=geometry->GetNominalScene()->GetOwners();
    const auto contracts=geometry->GetReference()->GetSphereResponseContracts();
    const auto mapping=geometry->GetReference()->GetProviderFaceIndices();
    if(owners.IsEmpty()||owners.Num()!=geometry->GetSelectedUnion()->Num()||
        contracts.Num()!=owners.Num()||mapping.Num()!=owners.Num())
        return refuse(TEXT("General complete native policy dimensions differ"));
    TSharedPtr<FGeneralFirstArrivalRsPolicyAdmission,ESPMode::ThreadSafe> admission(new FGeneralFirstArrivalRsPolicyAdmission);
    admission->Policy.AcceptedProviders.Reserve(owners.Num());
    admission->Policy.Branches.Reserve(owners.Num());
    for(int32 i=0;i<owners.Num();++i)
    {
        if(!budget.Consume())return refuse(TEXT("General Sphere complete native policy work exhausted"));
        if(!contracts[i].StructurallyValid(mapping[i])||!contracts[i].IsRadialReacquisitionV1())
            return refuse(TEXT("General Sphere producer radial first-contact contract differs"));
        if(!accepts(owners[i]))return refuse(TEXT("General Sphere mixed/incomplete native policy remains Unknown"));
        admission->Policy.AcceptedProviders.Add(1);
        admission->Policy.Branches.Add({owners[i].SurfaceId,true,false});
    }
    if(Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush).ControlFingerprint()!=
        numeric.ControlFingerprint())return refuse(TEXT("General Sphere policy numeric context changed"));
    admission->Geometry=MoveTemp(geometry);admission->NumericControl=numeric.ControlFingerprint();
    out=MoveTemp(admission);return true;
}
bool FGeneralFirstArrivalRsPolicyAdmission::Matches(const FPreparedGeometrySelectionPtr& geometry,
    const FGeneralFirstArrivalRsResponsePolicy& policy) const
{
    return Geometry&&Geometry==geometry&&(&Policy==&policy)&&
        Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush).ControlFingerprint()==NumericControl;
}
bool FGeneralFirstArrivalRsResponseQuality::MatchesPolicy(
    const Analytic::Prepared::FReferenceSphereGeneralFirstArrivalRsPtr& arrival,
    const FGeneralFirstArrivalRsResponsePolicy& policy) const
{
    if(!Arrival||Arrival!=arrival)return false;
    if(&Policy==&policy)return true;
    if(Policy.AcceptedProviders!=policy.AcceptedProviders||
        Policy.Branches.Num()!=policy.Branches.Num())return false;
    for(int32 i=0;i<Policy.Branches.Num();++i)
        if(Policy.Branches[i].SurfaceId!=policy.Branches[i].SurfaceId||
            Policy.Branches[i].NormalMayVary!=policy.Branches[i].NormalMayVary||
            Policy.Branches[i].StartPenetrating!=policy.Branches[i].StartPenetrating)return false;
    return true;
}
bool FGeneralFirstArrivalRsResponseQuality::MatchesConsumedValues(float seconds,FVector3d location,FVector3d impact,
    FVector3d stored,FVector3d consumed,FVector3d point) const
{
    using namespace ResponseQualityDetail;
    return Arrival&&FloatBits(seconds)==FloatBits(Arrival->GetReference().ConsumedSeconds)&&
        SamePoint(location,Scalar.Location)&&SamePoint(impact,Scalar.ImpactPoint)&&
        SamePoint(stored,Scalar.NormalStored)&&SamePoint(consumed,Scalar.NormalConsumed)&&SamePoint(point,ResponsePoint);
}
ESphereResponseQualityStatus EvaluateGeneralFirstArrivalRsResponseQuality(
    Analytic::Prepared::FReferenceSphereGeneralFirstArrivalRsPtr arrival,const FGeneralFirstArrivalRsResponsePolicy& policy,
    Analytic::Certified::FDistanceBudget& budget,FGeneralFirstArrivalRsResponseQualityPtr& out,FString& reason,
    const FGeneralFirstArrivalRsPolicyAdmission* admission)
{
    using namespace Analytic::Certified;using namespace ResponseQualityDetail;
    // Policy may be borrowed from the sole previous output. Preserve its
    // lifetime through all validation and the new immutable policy copy.
    const auto keepPrevious=out;
    out.Reset();reason.Reset();
    Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
    const auto refuse=[&](const TCHAR* why){reason=why;return ESphereResponseQualityStatus::Unknown;};
    if(!arrival||!arrival->HasBoundInitialAndFresh()||!budget.Consume())
        return refuse(TEXT("Rs response quality requires a retained first-arrival object"));
    const auto& input=arrival->GetCapturedInput();const auto& reference=arrival->GetReference();
    const auto geometry=arrival->GetGeometry();
    if(!geometry||!geometry->HasValidAliases()||geometry->GetModel()!=EPreparedGeometryModel::RetainedReference||
        !geometry->GetReference()||!IsSupportedSphereResponseMargin(input.ActualMargin)||
        reference.Status!=EReferenceSphereStatus::ReferenceContactBounded||!reference.ConsumedPrefixClear||
        reference.Contact.Status!=ETensorSphereContactStatus::FirstContactEnclosed||reference.Contact.Leaves.IsEmpty()||
        !reference.Candidates||reference.Candidates->GetScene()!=geometry->GetSelectedUnion()||
        reference.Contact.CollisionRadiusBits!=AnalyticCertified::Bits(input.SupportRadius)||
        !arrival->GetRcPartition()||arrival->GetRcPartition()->GetScene()!=geometry->GetSelectedUnion())
        return refuse(TEXT("Rs contact/all-leaf/whole-prefix/actual-Rc coverage identity unavailable"));
    const auto owners=geometry->GetNominalScene()->GetOwners();
    const auto contracts=geometry->GetReference()->GetSphereResponseContracts();
    const auto mapping=geometry->GetReference()->GetProviderFaceIndices();
    if(policy.Branches.Num()!=owners.Num()||policy.AcceptedProviders.Num()!=owners.Num()||
        contracts.Num()!=owners.Num()||mapping.Num()!=owners.Num())
        return refuse(TEXT("Rs response policy/producer contracts do not cover all providers"));
    if(admission&&!admission->Matches(geometry,policy))
        return refuse(TEXT("Rs response policy admission differs from exact live input"));
    if(!admission)
    {
        for(int32 i=0;i<owners.Num();++i)
        {
            if(!budget.Consume())return refuse(TEXT("Rs complete branch validation work exhausted"));
            if(policy.AcceptedProviders[i]>1||policy.Branches[i].SurfaceId!=owners[i].SurfaceId||
                !contracts[i].StructurallyValid(mapping[i])||!contracts[i].IsRadialReacquisitionV1()||
                !policy.Branches[i].NormalMayVary||policy.Branches[i].StartPenetrating)
                return refuse(TEXT("Rs complete branch values differ from the producer radial first-contact contract"));
        }
    }
    // No post-contact leaf deletion. The complete Rs set AND complete actual
    // Rc hull must obey the supplied conservative policy; outside providers
    // remain accounted by the retained exclusion proofs.
    for(const auto& partition:{reference.Candidates,arrival->GetRcPartition()})
        for(int32 i:partition->GetCandidateIndices())
            if(!budget.Consume()||!policy.AcceptedProviders.IsValidIndex(i)||policy.AcceptedProviders[i]!=1)
                return refuse(TEXT("Rs/Rc candidate not accepted by the complete supplied policy"));
    bool radial=false,have=false;
    for(const auto& leaf:reference.Contact.Leaves)
    {
        if(!budget.Consume()||arrival->IsRepeatedProvider(leaf.SurfaceIndex)||!policy.Branches.IsValidIndex(leaf.SurfaceIndex)||
            policy.AcceptedProviders[leaf.SurfaceIndex]!=1)return refuse(TEXT("Rs first-contact leaf policy is missing"));
        const auto& b=policy.Branches[leaf.SurfaceIndex];
        const bool next=SphereStaticResponse::UsesRadialPoint(b.SurfaceId,b.NormalMayVary,b.StartPenetrating,input.StaticContactOffset);
        if(have&&radial!=next)return refuse(TEXT("Rs first-contact leaves require incompatible response branches"));
        radial=next;have=true;
    }
    const float seconds=reference.ConsumedSeconds;const uint32 time=FloatBits(seconds);
    if((time&0x80000000u)||time>=0x7f800000u||(time&&time<0x00800000u))
        return refuse(TEXT("Rs consumed native seconds unsupported"));
    // +0 remains the native exact no-op, not P+V*0 with an imagined zero sign.
    FVector3d center(input.State.Location.X,input.State.Location.Y,input.State.Location.Z);
    if(time)center=SBox::AdvancePosition(input.State.Location,input.State.Velocity,input.State.Acceleration,seconds);
    if(!SamePoint(center,reference.Scalar.Location)||!SamePoint(center,arrival->GetFreshCenter()))
        return refuse(TEXT("Rs consumed actual center differs from the retained native-DAG point"));
    FScalarSphereContact scalar;
    if(ConvertSphereContact(reference.Contact,0,reference.Scalar.ImpactPoint,center,input.SupportRadius,
        double(input.PhysicalRadius),scalar)!=EScalarSphereContactStatus::Bounded)
        return refuse(TEXT("Rs scalar/normal/Rp-arm re-bound unavailable"));
    const auto& b=policy.Branches[reference.Contact.Leaves[0].SurfaceIndex];
    const auto point=SphereStaticResponse::Point(center,input.PhysicalRadius,scalar.ImpactPoint,scalar.NormalStored,
        b.SurfaceId,b.NormalMayVary,b.StartPenetrating,input.StaticContactOffset);
    if(!SamePoint(point,radial?scalar.RadialResponsePoint:scalar.ImpactPoint))
        return refuse(TEXT("Rs actual response arithmetic differs from the bounded scalar bits"));
    const double error=radial?scalar.ResponseErrorCm:scalar.SurfaceErrorCm;
    if(!TimeDetail::NonnegativeFinite(error))return refuse(TEXT("Rs response error unavailable"));
    TSharedPtr<FGeneralFirstArrivalRsResponseQuality,ESPMode::ThreadSafe> value(new FGeneralFirstArrivalRsResponseQuality);
    value->Arrival=MoveTemp(arrival);value->Policy=policy;value->Scalar=MoveTemp(scalar);value->ResponsePoint=point;
    value->PositionErrorCm=error;value->Radial=radial;
    value->Status=error<=double(input.ActualMargin)?ESphereResponseQualityStatus::BoundedWithinInputMargin:
        ESphereResponseQualityStatus::ErrorBoundExceedsInputMargin;
    Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
    out=MoveTemp(value);return out->GetStatus();
}
}
