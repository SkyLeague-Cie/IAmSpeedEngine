// Existing IAS TU. Finite canonical geometry and actual native return values;
// no live lookup, World fallback, response operation or readiness grant.
namespace Speed::PreparedExecution
{
bool BuildNativeCanonicalAxialBoxPlaneValues(FPreparedGeometrySelectionPtr geometry,Analytic::Prepared::FScenePtr nominal,
    const FNativeSolidStaticValues& v,C::FDistanceBudget& budget,FNativeCanonicalAxialBoxPlaneValuesPtr& out)
{
    const auto keepPrior=out;out.Reset();Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(!geometry||!geometry->HasValidAliases()||geometry->GetModel()!=EPreparedGeometryModel::RetainedReference||
        !nominal||nominal!=geometry->GetNominalScene()||v.Shape!=ENativeSolidQueryShape::Box||v.SphereMarginCaptured)return false;
    C::FFloorNativeSolidModel fixed;C::FIntervalPoint envelope;C::FCanonicalAxialPlaneBounds finite;
    if(!C::BuildNativeSolidSweepVolume(v,fixed,envelope)||
        !C::CloseCanonicalAxialPlaneBounds(geometry->GetSelectedUnion(),nominal,v.Query,envelope,budget,finite))return false;
    const auto& p=finite.Plane;const int32 axis=finite.NormalAxis;
    // Orientation is selected from the actual captured center relative to the
    // explicitly chosen plane, not from BodyState.Z or a reordered quaternion.
    const double delta=v.BodyState.Location[axis]-p.Origin[axis];
    if(!FMath::IsFinite(delta)||delta==0)return false;
    FVector normal=FVector::ZeroVector;normal[axis]=delta>0?1.:-1.;
    C::FFixedAxialBoxMotionModel motion;
    if(!C::BuildFixedAxialBoxMotionModel(v,p.Origin,normal,motion))return false;
    const auto arrival=C::BoundFixedAxialBoxFirstArrival(motion,budget);
    if(arrival.Status!=C::EFloorNativeSweepStatus::ArrivalBracket)return false;
    SHitResult descriptor;descriptor.ImpactPoint=p.Origin;descriptor.ImpactNormal=normal;
    descriptor.SourceId=p.SourceId;descriptor.SurfaceId=p.SurfaceId;descriptor.FeatureId=p.FeatureId;descriptor.PrimitiveId=p.PrimitiveId;
    descriptor.CanonicalGroupId=finite.CanonicalGroupId;descriptor.MaterialId=p.MaterialId;
    descriptor.ContactFeatureOther=EContactFeatureKind::Face;
    TSharedPtr<FNativeCanonicalAxialBoxPlaneValues,ESPMode::ThreadSafe> value(new FNativeCanonicalAxialBoxPlaneValues);
    value->Geometry=MoveTemp(geometry);value->Nominal=MoveTemp(nominal);value->Captured=v;
    value->Finite=MoveTemp(finite);value->Arrival=arrival;value->Descriptor=MoveTemp(descriptor);out=MoveTemp(value);return true;
}
bool MatchesNativeCanonicalAxialBoxPlaneValues(const FNativeSolidStaticValues& v,const FPreparedGeometrySelectionPtr& geometry,
    const Analytic::Prepared::FScenePtr& nominal,const FNativeCanonicalAxialBoxPlaneValuesPtr& value,const C::FIntervalPoint& envelope)
{
    return value&&value->Geometry==geometry&&value->Nominal==nominal&&geometry&&geometry->HasValidAliases()&&
        geometry->GetModel()==EPreparedGeometryModel::RetainedReference&&geometry->GetNominalScene()==nominal&&value->Finite.Complete&&
        value->Finite.Partition&&value->Finite.Partition->GetScene()==geometry->GetSelectedUnion()&&
        C::CanonicalAxialPlaneDetail::SamePoint(envelope,value->Finite.Envelope)&&
        SameNativeSolidStaticValues(v,value->Captured)&&value->Arrival.Status==C::EFloorNativeSweepStatus::ArrivalBracket;
}
bool MatchNativeCanonicalAxialBoxRefinement(const FNativeSolidStaticValues& v,
    const FNativeCanonicalAxialBoxPlaneValuesPtr& value,const SHitResult& hit,const FNativeBoxPlanarRefinementValues& r)
{
    if(!value)return false;const auto& a=value->GetArrivalForInspection();const auto& d=value->GetDescriptorForInspection();
    const auto& finite=value->GetFiniteForInspection();
    // The real refiner's end test can miss entry followed by exit before H.
    // Its actual Delta is InsideSeconds; ORIGINAL request H remains captured.
    auto inputs=v;inputs.Horizon=a.InsideSeconds;
    if(!NativeFloorBoxPlaneDetail::MatchesReadbackInputs(inputs,d.ImpactNormal,d.ImpactPoint,r)||
        r.Outcome!=ENativeBoxPlanarRefinementOutcome::Hit||r.InitialEvent||r.Established||r.EstablishedMask||
        !r.WitnessComputed||r.FirstCorner<0||r.FirstCorner>=8||!SameFloat(r.FirstTime,a.ClearPrefixSeconds)||
        !SameFloat(hit.TOI,a.ClearPrefixSeconds)||r.MinimumInitial<=r.Roundoff||
        hit.bHit||hit.bBlockingHit||hit.bStartPenetrating||hit.bSurfaceNormalMayVary||
        !SameFloat(hit.PenetrationDepth,0.f)||!SameFloat(hit.GeometricErrorBoundCm,0.f)||
        !hit.Component.IsExplicitlyNull()||!hit.SubBody.IsExplicitlyNull()||hit.PreparedGroundEvidence||hit.FirstRsResponseData||
        hit.SourceId!=d.SourceId||hit.SurfaceId!=d.SurfaceId||hit.FeatureId!=d.FeatureId||hit.PrimitiveId!=d.PrimitiveId||
        hit.CanonicalGroupId!=d.CanonicalGroupId||hit.MaterialId!=d.MaterialId||!SameVector(hit.ImpactNormal,d.ImpactNormal)||
        hit.ContactFeatureThis!=EContactFeatureKind::Vertex||hit.ContactFeatureIndexThis!=r.FirstCorner||
        hit.ContactFeatureOther!=EContactFeatureKind::Face||hit.ContactFeatureIndexOther!=INDEX_NONE)return false;
    auto pose=v.OwnerState.Integrate(hit.TOI);pose.Location-=pose.Rotation.RotateVector(v.LocalCOM);
    pose.Location+=pose.Rotation.RotateVector(v.LocalOffset);pose.Rotation=pose.Rotation*v.LocalRotation;
    FVector local;for(int32 axis=0;axis<3;++axis)local[axis]=(r.FirstCorner&(1<<axis))?v.PhysicalHalfExtent[axis]:-v.PhysicalHalfExtent[axis];
    const FVector vertex=pose.Location+pose.Rotation.RotateVector(local);
    const FVector projected=vertex-FVector::DotProduct(vertex-d.ImpactPoint,d.ImpactNormal)*d.ImpactNormal;
    const auto& q=r.WitnessQuery;const auto& w=r.Witness;
    // Check the actual query and returned finite witness; never substitute a
    // forced coordinate for its projection/Lerp/ClosestPoint floating DAG.
    if(!SameVector(hit.Location,pose.Location)||!SameVector(hit.ContactPointThis,vertex)||
        !SameVector(hit.ImpactPoint,w.Point)||!SameVector(hit.ContactPointOther,w.Point)||
        q.Shape!=Analytic::EQueryShape::Ray||!SameVector(q.Start,projected+d.ImpactNormal*.01)||
        !SameVector(q.End,projected-d.ImpactNormal*.01)||q.RequiredSourceId!=d.SourceId||q.RequiredSurfaceId!=d.SurfaceId||
        q.RequiredCanonicalGroupId||!q.bAuthorityOnly||!q.bIncludeCompactPatches||!q.bApplyCollisionFilter||
        q.bObjectQuery||q.bExcludeAuthorityEligible||q.TraceChannel!=v.Query.TraceChannel||
        q.BlockingObjectTypes!=v.Query.BlockingObjectTypes||q.DomainTolerance!=0||q.InitialOverlapTolerance!=0||
        !w.bHit||w.bStartPenetrating||w.bSurfaceNormalMayVary||w.GeometricErrorBoundCm!=0||w.PenetrationDepth!=0||
        !SameVector(w.Normal,d.ImpactNormal)||w.SurfaceFeatureKind!=EContactFeatureKind::Face||
        w.SourceId!=d.SourceId||w.SurfaceId!=d.SurfaceId||w.FeatureId!=d.FeatureId||w.PrimitiveId!=d.PrimitiveId||
        w.CanonicalGroupId!=d.CanonicalGroupId||w.MaterialId!=d.MaterialId)return false;
    C::FDistanceBudget budget{1024};
    return C::CanonicalAxialPlaneContains(finite,w.Point,budget)&&C::CanonicalAxialPlaneContains(finite,w.QueryPoint,budget);
}
}
