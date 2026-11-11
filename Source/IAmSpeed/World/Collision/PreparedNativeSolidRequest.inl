// Included in Speed::PreparedExecution; shares existing native Step/TLS and
// bitwise primitives. This installs no terminal purpose or physical consumer.
bool NextNativeSolidQuery(FStepIdentity& out) noexcept
{
    out={};FStepIdentity now;
    if(!ReadCurrentStep(now)||!StepScope||!StepScope->Window||
        StepScope->Window->GetPurpose()!=OnePausedWork::EPurpose::FullFrame)return false;
    return NextSphereQuery(out);
}
bool FComponentSweepScope::ReadNativeSolidSite(const ISolidSweeper* sweeper,const USolidSubBody* expected,
    const ISpeedComponent*& owner,int32& index) noexcept
{
    owner=nullptr;index=INDEX_NONE;FStepIdentity now;const auto* scope=ComponentSweepScope;
    if(!sweeper||!expected||!ReadCurrentStep(now)||!StepScope||!StepScope->Window||!StepScope->SweepPhase||
        StepScope->Window->GetPurpose()!=OnePausedWork::EPurpose::FullFrame||
        !scope||!scope->Active||scope->Completed||scope->Interrupted||!scope->Pending||
        scope->CurrentIndex!=scope->NextIndex||!scope->Pending->Bodies.IsValidIndex(scope->CurrentIndex)||
        scope->Pending->Step.QueryEpoch!=now.QueryEpoch||scope->Pending->Step.StepSerial!=now.StepSerial||
        scope->Pending->Step.WindowSerial!=now.WindowSerial||scope->Pending->Step.WindowNonce!=now.WindowNonce||
        !StepScope->Snapshot||!StepScope->Snapshot->GetNativeOwners())return false;
    const auto* row=StepScope->Snapshot->GetNativeOwners()->Find(scope->Pending->Owner);
    if(!row||!row->BodyCensusCaptured||!row->Bodies.IsValidIndex(scope->CurrentIndex))return false;
    const auto& child=row->Bodies[scope->CurrentIndex];
    // Pointer equality precedes any access to the caller's expected Body or
    // interface. The raw association was pinned/captured on GT for this frame.
    if(child.Body!=static_cast<const USSubBody*>(expected)||child.Index!=scope->CurrentIndex||!child.Body)return false;
    if(child.Body->GetClass()!=child.Class||!row->Adapter||!row->NativeObject||row->NativeObject->GetClass()!=row->Class)return false;
    const auto* sphere=Cast<USphereSubBody>(child.Body);const auto* box=Cast<UBoxSubBody>(child.Body);
    if(Cast<USWheelSubBody>(child.Body)||(!sphere&&!box))return false;
    const ISolidSweeper* actual=sphere?static_cast<const ISolidSweeper*>(sphere):static_cast<const ISolidSweeper*>(box);
    if(actual!=sweeper||child.Body->GetClass()!=child.Class||child.Body->GetParentComponent()!=row->Adapter||
        !row->Adapter||!row->NativeObject||row->NativeObject->GetClass()!=row->Class||
        row->Adapter->GetSubBodies().Num()!=row->Bodies.Num()||row->Adapter->GetSubBodies()[child.Index]!=child.Body||
        !SameKinematicBits(scope->Pending->SweepOwnerState,row->Adapter->GetKinematicState()))return false;
    owner=row->Adapter;index=child.Index;return true;
}
bool CaptureNativeSolidTraceValues(const ISolidSweeper& sweeper,const FCollisionQueryParams& params,FNativeSolidTraceValues& out)
{
    out={};out.TraceComplex=params.bTraceComplex;out.FindInitialOverlaps=params.bFindInitialOverlaps;
    out.ReturnFaceIndex=params.bReturnFaceIndex;out.ReturnPhysicalMaterial=params.bReturnPhysicalMaterial;
    out.IgnoreBlocks=params.bIgnoreBlocks;out.IgnoreTouches=params.bIgnoreTouches;
    out.SkipNarrowPhase=params.bSkipNarrowPhase;out.TraceIntoSubComponents=params.bTraceIntoSubComponents;
    out.ReplaceHitWithSubComponents=params.bReplaceHitWithSubComponents;out.IgnoreMask=uint64(params.IgnoreMask);
    out.Mobility=uint8(params.MobilityType);out.TraceChannel=uint8(sweeper.GetCollisionChannel());
    for(int32 c=0;c<UE_ARRAY_COUNT(FCollisionResponseContainer::EnumArray);++c)
        out.Responses.Add(uint8(sweeper.GetResponseParams().CollisionResponse.GetResponse(ECollisionChannel(c))));
    const auto& sources=params.GetIgnoredSourceObjects();out.IgnoredSourceObjects.Append(sources.GetData(),sources.Num());
    const auto& components=params.GetIgnoredComponents();out.IgnoredComponents.Append(components.GetData(),components.Num());return true;
}
bool SameNativeSolidResponseConfigValues(const FNativeSolidResponseConfigValues& a,
    const FNativeSolidResponseConfigValues& b) noexcept
{
    if(!SameFloat(a.BodyMass,b.BodyMass)||!SameFloat(a.OwnerPhysMass,b.OwnerPhysMass)||
        !SameFloat(a.Restitution,b.Restitution)||!SameFloat(a.ImpactThreshold,b.ImpactThreshold)||
        !SameFloat(a.StaticFriction,b.StaticFriction)||!SameFloat(a.DynamicFriction,b.DynamicFriction)||
        !SameFloat(a.HitDamping,b.HitDamping)||!SameFloat(a.OwnerMaxSpeed,b.OwnerMaxSpeed)||
        !SameFloat(a.OwnerMaxAngularSpeed,b.OwnerMaxAngularSpeed)||a.MainSubBody!=b.MainSubBody||
        a.ApplyRestForce!=b.ApplyRestForce||a.AerialShotContactModel!=b.AerialShotContactModel||
        a.SphereProjectionPolicyCaptured!=b.SphereProjectionPolicyCaptured||
        a.SphereWorldStaticProjectionEnabled!=b.SphereWorldStaticProjectionEnabled)return false;
    for(int32 row=0;row<4;++row)for(int32 col=0;col<4;++col)
        if(!SameDouble(a.CachedBodyLocalInvInertia.M[row][col],b.CachedBodyLocalInvInertia.M[row][col]))return false;
    return true;
}
namespace NativeSolidDetail
{
// Caller has already authenticated the real live GT row/body association.
// Kept separate from request/pose/trace revalidation: after AcceptHit/advance
// a future response scope must reread THIS configuration and authenticate its
// legitimately changed consume state, rather than reuse the whole old request.
bool ReadResponseConfig(const USolidSubBody& body,const ISpeedComponent& owner,FNativeSolidResponseConfigValues& out)
{
    out={};if(body.GetParentComponent()!=&owner)return false;
    FNativeSolidResponseConfigValues v;
    v.BodyMass=body.GetMass();v.OwnerPhysMass=owner.GetPhysMass();v.Restitution=body.GetRestitution();
    v.ImpactThreshold=body.GetImpactThreshold();v.StaticFriction=body.GetStaticFriction();v.DynamicFriction=body.GetDynamicFriction();
    v.HitDamping=body.GetHitDamping();v.OwnerMaxSpeed=owner.GetPhysMaxSpeed();v.OwnerMaxAngularSpeed=owner.GetPhysMaxAngularSpeed();
    v.CachedBodyLocalInvInertia=body.GetCachedLocalInvInertiaTensor();
    v.MainSubBody=body.IsMainSubBody();v.ApplyRestForce=body.HasToApplyRestForce();v.AerialShotContactModel=body.UsesAerialShotContactModel();
    if(Cast<USphereSubBody>(&body))
    {v.SphereProjectionPolicyCaptured=true;v.SphereWorldStaticProjectionEnabled=USphereSubBody::IsWorldStaticProjectionEnabled();}
    const float scalars[]={v.BodyMass,v.OwnerPhysMass,v.Restitution,v.ImpactThreshold,v.StaticFriction,v.DynamicFriction,
        v.HitDamping,v.OwnerMaxSpeed,v.OwnerMaxAngularSpeed};
    for(float value:scalars)if(!FMath::IsFinite(value))return false;
    for(int32 row=0;row<4;++row)for(int32 col=0;col<4;++col)
        if(!FMath::IsFinite(v.CachedBodyLocalInvInertia.M[row][col]))return false;
    // No range clamp, mass/inertia model, gameplay preset or permission is
    // inferred. The actual scalar and cached off-diagonal bits are retained.
    out=MoveTemp(v);return true;
}
bool SameQueryBits(const Analytic::FWorldQuery& a,const Analytic::FWorldQuery& b) noexcept
{
    return a.Shape==b.Shape&&SameVector(a.Start,b.Start)&&SameVector(a.End,b.End)&&SameQuat(a.Rotation,b.Rotation)&&
        SameDouble(a.Radius,b.Radius)&&SameVector(a.HalfExtent,b.HalfExtent)&&SameDouble(a.DomainTolerance,b.DomainTolerance)&&
        SameDouble(a.InitialOverlapTolerance,b.InitialOverlapTolerance)&&a.TraceChannel==b.TraceChannel&&a.ObjectTypes==b.ObjectTypes&&
        a.BlockingObjectTypes==b.BlockingObjectTypes&&a.RequiredSourceId==b.RequiredSourceId&&a.RequiredSurfaceId==b.RequiredSurfaceId&&
        a.RequiredCanonicalGroupId==b.RequiredCanonicalGroupId&&SameVector(a.ReferenceNormal,b.ReferenceNormal)&&
        SameDouble(a.MinimumReferenceNormalDot,b.MinimumReferenceNormalDot)&&
        a.bAllowEstablishedFaceContactAtBoundary==b.bAllowEstablishedFaceContactAtBoundary&&a.bUseFiniteContactDomain==b.bUseFiniteContactDomain&&
        a.bObjectQuery==b.bObjectQuery&&a.bApplyCollisionFilter==b.bApplyCollisionFilter&&a.bAuthorityOnly==b.bAuthorityOnly&&
        a.bExcludeAuthorityEligible==b.bExcludeAuthorityEligible&&a.bExcludeTori==b.bExcludeTori&&
        a.bIncludeCompactPatches==b.bIncludeCompactPatches&&a.bIncludeTriangles==b.bIncludeTriangles;
}
bool Read(const UWorld* world,const ISolidSweeper* sweeper,const USolidSubBody* body,FVector start,FVector end,
    const FCollisionShape& shape,const FCollisionQueryParams& trace,float horizon,const FStepIdentity& step,
    const FPreparedWorldSnapshotPtr& snapshot,FNativeSolidStaticValues& out)
{
    out={};const ISpeedComponent* owner=nullptr;int32 index=INDEX_NONE;
    if(!FComponentSweepScope::ReadNativeSolidSite(sweeper,body,owner,index)||!snapshot||!snapshot->GetFilters()||
        !owner||world!=step.World||body->GetWorld()!=world||sweeper->GetSolidSweepBody()!=body||
        !SameFloat(horizon,step.Remaining)||!FMath::IsFinite(horizon)||horizon<=0)return false;
    FNativeSolidStaticValues v;v.Sweeper=sweeper;v.Body=body;v.Owner=owner;v.BodyClass=body->GetClass();v.BodyIndex=index;
    if(!JoinCanonicalPreparedFrameClock(step.Frame,owner->NumFrame(),v.FrameClock))return false;
    v.OwnerStableId=FStepScope::ReadOwnerStableId(owner);
    if(!v.OwnerStableId||v.OwnerStableId!=owner->GetPublishedSimulationStableId())return false;
    const auto* inheritedOwner=snapshot->GetNativeOwners()?snapshot->GetNativeOwners()->Find(owner):nullptr;
    if(!inheritedOwner||!inheritedOwner->Bodies.IsValidIndex(index))return false;
    const auto& inheritedBody=inheritedOwner->Bodies[index];
    if(inheritedBody.Body!=body||inheritedBody.Index!=index)return false;
    v.InheritedMotionPolicy91=inheritedBody.InheritedMotionPolicy91; // immutable GT values, no new UObject getter
    v.OwnerState=owner->GetKinematicState();v.BodyState=body->GetKinematicState();
    v.LocalOffset=body->GetLocalOffset();v.LocalCOM=owner->GetPhysCenterOfMassLocal();v.LocalRotation=body->GetLocalRotation();
    v.Frozen=owner->IsFrozen();v.Prescribed=owner->IsPrescribedMotion();v.Start=start;v.End=end;v.Horizon=horizon;
    if(!FiniteState(v.OwnerState)||!FiniteState(v.BodyState)||!SameKinematicBits(v.BodyState,sweeper->GetKinematicState())||
        start.ContainsNaN()||end.ContainsNaN()||v.LocalOffset.ContainsNaN()||v.LocalCOM.ContainsNaN()||
        v.LocalRotation.ContainsNaN()||!SameVector(start,v.BodyState.Location))return false;
    const FVector expectedEnd=start+v.BodyState.Velocity*horizon+0.5f*v.BodyState.Acceleration*horizon*horizon;
    if(!SameVector(end,expectedEnd))return false; // actual World-call DAG, not a continuous/native integration theorem
    const auto actual=sweeper->GetWorldStaticCollisionShape();
    if(shape.IsSphere())
    {
        const auto* sphere=Cast<USphereSubBody>(body);if(!sphere||!actual.IsSphere())return false;
        v.Shape=ENativeSolidQueryShape::Sphere;v.QueryRadius=shape.GetSphereRadius();v.PhysicalRadius=sphere->GetRadius();
        v.SupportRadius=sphere->GetWorldStaticSupportRadius();v.StaticContactOffset=sphere->GetWorldStaticContactOffsetCm();
        v.StaticImpactFriction=sphere->GetWorldStaticImpactFriction();
        if(!SameFloat(actual.GetSphereRadius(),v.QueryRadius)||!FMath::IsFinite(v.QueryRadius)||v.QueryRadius<=0||
            !FMath::IsFinite(v.PhysicalRadius)||v.PhysicalRadius<=0||!FMath::IsFinite(v.SupportRadius)||v.SupportRadius<=0||
            !FMath::IsFinite(v.StaticContactOffset)||v.StaticContactOffset<0||!FMath::IsFinite(v.StaticImpactFriction)||v.StaticImpactFriction<0||
            !SameDouble(v.SupportRadius,double(v.PhysicalRadius)+double(v.StaticContactOffset))||
            !ReadSphereMemberSweep(sphere,owner,v.SphereMemberSerial,v.SphereMargin))return false;
        v.SphereMarginCaptured=true;
    }
    else if(shape.IsBox())
    {
        const auto* box=Cast<UBoxSubBody>(body);if(!box||!actual.IsBox())return false;
        v.Shape=ENativeSolidQueryShape::Box;v.QueryHalfExtent=FVector(shape.GetBox());v.PhysicalHalfExtent=box->GetBoxExtent();
        if(!SameVector(FVector(actual.GetBox()),v.QueryHalfExtent)||v.QueryHalfExtent.ContainsNaN()||v.QueryHalfExtent.GetMin()<=0||
            v.PhysicalHalfExtent.ContainsNaN()||v.PhysicalHalfExtent.GetMin()<=0)return false;
    }
    else return false;
    if(!ReadResponseConfig(*body,*owner,v.ResponseConfig)||!CaptureNativeSolidTraceValues(*sweeper,trace,v.Trace))return false;
    v.Query=Analytic::FStaticWorldQueryAudit::MakeStrictSingleQuery1075(start,end,v.BodyState.Rotation,shape,
        uint8(sweeper->GetCollisionChannel()),sweeper->GetResponseParams());
    if(!snapshot->GetFilters()->CoversPhysicalFilter(true,step.InputGeneration,owner,body,v.Query))return false;
    out=MoveTemp(v);return true;
}
}
bool SameNativeSolidStaticValues(const FNativeSolidStaticValues& a,const FNativeSolidStaticValues& b) noexcept
{
    return a.Sweeper==b.Sweeper&&a.Body==b.Body&&a.Owner==b.Owner&&a.BodyClass==b.BodyClass&&
        a.OwnerStableId==b.OwnerStableId&&a.BodyIndex==b.BodyIndex&&a.Shape==b.Shape&&a.FrameClock==b.FrameClock&&
        SameKinematicBits(a.OwnerState,b.OwnerState)&&SameKinematicBits(a.BodyState,b.BodyState)&&
        SameInheritedMotionPolicy91(a.InheritedMotionPolicy91,b.InheritedMotionPolicy91)&&
        SameVector(a.LocalOffset,b.LocalOffset)&&SameVector(a.LocalCOM,b.LocalCOM)&&SameQuat(a.LocalRotation,b.LocalRotation)&&
        SameVector(a.Start,b.Start)&&SameVector(a.End,b.End)&&SameFloat(a.Horizon,b.Horizon)&&a.Frozen==b.Frozen&&a.Prescribed==b.Prescribed&&
        SameVector(a.QueryHalfExtent,b.QueryHalfExtent)&&SameVector(a.PhysicalHalfExtent,b.PhysicalHalfExtent)&&
        SameFloat(a.QueryRadius,b.QueryRadius)&&SameFloat(a.PhysicalRadius,b.PhysicalRadius)&&SameDouble(a.SupportRadius,b.SupportRadius)&&
        SameFloat(a.StaticContactOffset,b.StaticContactOffset)&&SameFloat(a.StaticImpactFriction,b.StaticImpactFriction)&&
        a.SphereMarginCaptured==b.SphereMarginCaptured&&SameFloat(a.SphereMargin,b.SphereMargin)&&a.SphereMemberSerial==b.SphereMemberSerial&&
        SameNativeSolidResponseConfigValues(a.ResponseConfig,b.ResponseConfig)&&
        NativeSolidDetail::SameQueryBits(a.Query,b.Query)&&a.Trace.Same(b.Trace);
}
bool CaptureNativeSolidCCD(const UWorld* world,const ISolidSweeper* sweeper,const USolidSubBody* solid,FVector start,FVector end,
    const FCollisionShape& shape,const FCollisionQueryParams& trace,float horizon,FNativeSolidStaticRequestPtr& out)
{
        PreparedExecution::FNativePlanarResponseCommitScope::RequireQueriesClosed();
    out.Reset();const ISpeedComponent* owner=nullptr;int32 index=INDEX_NONE;
    if(!FComponentSweepScope::ReadNativeSolidSite(sweeper,solid,owner,index))return false;
    FStepIdentity step;if(!NextNativeSolidQuery(step))return false;FPreparedWorldSnapshotPtr snapshot;
    if(!ReadCurrentSnapshot(snapshot)||!snapshot||!snapshot->GetGeometrySelection()||!snapshot->GetGeometrySelection()->HasValidAliases())return false;
    FNativeSolidStaticValues values;if(!NativeSolidDetail::Read(world,sweeper,solid,start,end,shape,trace,horizon,step,snapshot,values))return false;
    TSharedPtr<FNativeSolidStaticRequest,ESPMode::ThreadSafe> request(new FNativeSolidStaticRequest);
    request->Step=step;request->Values=MoveTemp(values);request->Snapshot=snapshot;
    if(!RevalidateNativeSolidCCD(*request))return false;out=MoveTemp(request);return true;
}
bool RevalidateNativeSolidCCD(const FNativeSolidStaticRequest& request) noexcept
{
    try
    {
    FStepIdentity now;FPreparedWorldSnapshotPtr snapshot;
    if(!ReadCurrentStep(now)||!ReadCurrentSnapshot(snapshot)||snapshot!=request.Snapshot||
        now.Host!=request.Step.Host||now.Bridge!=request.Step.Bridge||now.World!=request.Step.World||now.Worker!=request.Step.Worker||
        now.Provider!=request.Step.Provider||now.Lease!=request.Step.Lease||now.InputGeneration!=request.Step.InputGeneration||
        now.Frame!=request.Step.Frame||now.WindowSerial!=request.Step.WindowSerial||now.WindowNonce!=request.Step.WindowNonce||
        now.StepSerial!=request.Step.StepSerial||now.QueryEpoch!=request.Step.QueryEpoch||now.Iteration!=request.Step.Iteration||
        now.NumericControl!=request.Step.NumericControl||now.ThreadId!=request.Step.ThreadId||!request.Step.QuerySerial||
        request.Step.QuerySerial>now.QuerySerial||!SameFloat(now.Delta,request.Step.Delta)||!SameFloat(now.SimTime,request.Step.SimTime)||
        !SameFloat(now.Remaining,request.Step.Remaining)||!SameFloat(now.TimePassed,request.Step.TimePassed))return false;
    const ISpeedComponent* owner=nullptr;int32 index=INDEX_NONE;
    if(!FComponentSweepScope::ReadNativeSolidSite(request.Values.Sweeper,request.Values.Body,owner,index))return false;
    const auto shape=request.Values.Sweeper->GetWorldStaticCollisionShape();const auto trace=request.Values.Sweeper->BuildTraceParams();
    FNativeSolidStaticValues values;
    return NativeSolidDetail::Read(now.World,request.Values.Sweeper,request.Values.Body,request.Values.Start,request.Values.End,
        shape,trace,request.Values.Horizon,now,snapshot,values)&&SameNativeSolidStaticValues(values,request.Values);
    }
    catch(...){return false;}
}
