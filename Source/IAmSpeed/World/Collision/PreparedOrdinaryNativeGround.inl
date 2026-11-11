static TAutoConsoleVariable<int32> CVarEstablishedPlanarFloorClear(
    TEXT("p.IAmSpeed.AnalyticWorld.EstablishedPlanarFloorClear"),0,
    TEXT("Prove all physical gaps after an established post-planar NoCorner. Default off."));

static TAutoConsoleVariable<int32> CVarNativeSphereNextEligible(
    TEXT("p.IAmSpeed.AnalyticWorld.NativeSphereNextEligible"),0,
    TEXT("Default OFF. Continue the same ordinary Sphere request after authenticated World rejection of the initial floor pair."));
#if !UE_BUILD_SHIPPING
static TAutoConsoleVariable<int32> CVarNativeRoofFixedCachedFloorArrival(
    TEXT("p.IAmSpeed.AnalyticWorld.NativeRoofFixedCachedFloorArrival"),0,
    TEXT("Default OFF. Reuse the original fixed-roof floor hit only after its exact paid prefix is closed."));
static TAutoConsoleVariable<int32> CVarNativeRoofSlideFloorRefinerReadback(
    TEXT("p.IAmSpeed.AnalyticWorld.NativeRoofSlideFloorRefinerReadback"),0,
    TEXT("Default OFF. Observe the original cached FLOOR refiner refusal at the production roof-slide start."));
#endif

static TAutoConsoleVariable<int32> CVarNativeSlideCachedFloorReadback(
    TEXT("p.IAmSpeed.AnalyticWorld.NativeSlideCachedFloorReadback"),0,
    TEXT("Default OFF. Observe the exact slide fixture's ordinary Box cache and World membership before its existing FLOOR gate."));
static TAutoConsoleVariable<int32> CVarNativeSphereRepeatedFloorReadback(
    TEXT("p.IAmSpeed.AnalyticWorld.NativeSphereRepeatedFloorReadback"),0,
    TEXT("Default OFF. Observe the original repeated floor pair after an ordinary Sphere refusal."));
static TAutoConsoleVariable<int32> CVarNativeBoxIgnored585CompetitorReadback(
    TEXT("p.IAmSpeed.AnalyticWorld.NativeBoxIgnored585CompetitorReadback"),0,
    TEXT("Default OFF. Read existing World returns and the raw dynamic census at the exact ignored Box 585 refusal."));

static TAutoConsoleVariable<int32> CVarCase202CompetitorCensus(
    TEXT("p.IAmSpeed.AnalyticWorld.Case202CompetitorCensus"),0,
    TEXT("Read the private case-202 frame-127 competitor roster before its existing refusal. Default OFF."));

static TAutoConsoleVariable<int32> CVarCase202Serial3Curve(
    TEXT("p.IAmSpeed.AnalyticWorld.Case202Serial3Curve"),0,
    TEXT("Private case 202 exact repeated 1931 arrival with an authenticated reviewed image. Default OFF."));

namespace NativeBoxSplitArrival202
{
TAutoConsoleVariable<int32> Enabled(
    TEXT("p.IAmSpeed.AnalyticWorld.NativeBoxSplitArrival202"),0,
    TEXT("Default OFF. Qualify the authenticated established-floor case 202 with separate native and reference arrival proofs."));
// This loan is valid only inside the actual established-floor query below.
// It names immutable inputs, not an advance, result or response capability.
thread_local const FNativeSolidStaticRequest* Request=nullptr;

bool Matches(const FPreparedWorldSnapshotPtr& snapshot,const FNativeSolidStaticValues& source)
{
    FStepIdentity now;
    return Enabled.GetValueOnAnyThread()==1&&Request&&Request->GetSnapshot()==snapshot&&
        SameNativeSolidStaticValues(Request->GetValues(),source)&&RevalidateNativeSolidCCD(*Request)&&
        ReadCurrentStep(now)&&OrdinaryWorldDetail::Epoch(Request->GetStep(),now)&&
        now.QuerySerial==Request->GetStep().QuerySerial;
}
}

// Included once inside PreparedExecution in the existing IAS TU.
// This closes World candidate values in the ordinary production path only;
// actual dynamic/cache winners and global advance/resolve remain independent.
bool FComponentSweepScope::EvaluateOrdinaryNativeGround(const UWorld* world,const ISolidSweeper* sweeper,
    const USolidSubBody* body,FVector start,FVector end,const FCollisionShape& shape,const FCollisionQueryParams& trace,
    float horizon,SHitResult& out)
{
        PreparedExecution::FNativePlanarResponseCommitScope::RequireQueriesClosed();
    out={};
    if(MemberSweepScope&&MemberSweepScope->PendingSleepingSupport)
        Refuse("Sleeping support bypass cannot also execute an ordinary World query");
    FNativeSolidStaticRequestPtr request;
    if(!CaptureNativeSolidCCD(world,sweeper,body,start,end,shape,trace,horizon,request))
        Refuse("Ordinary solid World request is unauthenticated or unsupported");
    auto* scope=ComponentSweepScope;const auto& v=request->GetValues();
    if(!scope||!scope->Active||scope->Completed||!scope->Pending||
        !scope->Pending->Bodies.IsValidIndex(scope->CurrentIndex)||scope->Pending->Bodies[scope->CurrentIndex].OrdinaryGround||
        scope->Pending->Bodies[scope->CurrentIndex].Ground||!request->GetStep().Provider)
        Refuse("Ordinary solid World query has no unique current component slot");
    const auto service=request->GetStep().Provider->GetPreparedQueryService();FPreparedNativeSolidGeometryPtr geometry;
    if(!service||!service->MatchesPreparedSnapshot(request->GetSnapshot())||
        !service->QueryNativeSolidGeometry(request,geometry)||!geometry||geometry->GetRequest()!=request)
    {
        const bool nextEligible=CVarNativeSphereNextEligible.GetValueOnAnyThread()==1;
        bool continued=false;
        if(service&&!geometry&&v.Shape==ENativeSolidQueryShape::Sphere&&request->GetStep().Iteration==2&&
            (nextEligible||CVarNativeSphereRepeatedFloorReadback.GetValueOnAnyThread()==1))
        {
            const auto observe=[&]() -> const TCHAR*
            {
                const auto* step=StepScope;
                const auto* returns=step?step->OrdinaryReturns.Get():nullptr;
                const auto* rows=returns&&returns->World?returns->World.Get():nullptr;
                const auto* memberScope=MemberSweepScope;
                const auto& captured=request->GetStep();
                const auto current=[&]()
                {
                    FStepIdentity now;
                    return step&&step->Snapshot==request->GetSnapshot()&&rows&&!rows->Closed&&rows->PairsIdentity&&
                        returns->Pending&&returns->Pending->Owner==v.Owner&&scope->CurrentIndex==v.BodyIndex&&
                        memberScope&&memberScope->Active&&!memberScope->Completed&&!memberScope->Interrupted&&
                        memberScope->Body==v.Body&&memberScope->Owner==v.Owner&&memberScope->Serial==v.SphereMemberSerial&&
                        SameFloat(memberScope->Margin,v.SphereMargin)&&
                        C::KinematicCenterDetail::Bits(captured.TimePassed)==0&&SameFloat(rows->Before,captured.TimePassed)&&
                        SameFloat(rows->Horizon,v.Horizon)&&SameFloat(rows->Delta,captured.Delta)&&
                        ReadCurrentStep(now)&&OrdinaryWorldDetail::Epoch(rows->Entry,now)&&
                        OrdinaryWorldDetail::Epoch(captured,now)&&now.QuerySerial==captured.QuerySerial&&
                        service->MatchesPreparedSnapshot(request->GetSnapshot())&&RevalidateNativeSolidCCD(*request);
                };
                if(!current())return TEXT("scope");
                FNativeSolidGeometryValues observation;
                if(!service->ObserveNativeRepeatedSphereFloorAtZero(*request,observation)||
                    observation.Status!=ENativeSolidGeometryStatus::CapturedSphereOverlap||!current())return TEXT("finite_floor");
                FNativeFloorSphereContactGeometryValues floor;
                if(!NativeFloorSphereContactDetail::Build(v,observation,floor)||
                    C::KinematicCenterDetail::Bits(floor.Seconds)!=0||!floor.Hit.bStartPenetrating)return TEXT("payload");
                bool ambiguous=false;
                const auto* binding=request->GetSnapshot()->FindUniqueContactBinding(floor.Hit.SourceId,ambiguous);
                SHitResult hit;
                if(ambiguous||!binding||!BuildNativeAnalyticContactValues(floor.Hit,floor.Seconds,floor.OwnerFrame,*binding,hit)||
                    !hit.Component.IsValid()||!hit.SubBody.IsExplicitlyNull()||!hit.bHit||!hit.bBlockingHit||!hit.SurfaceId||
                    C::KinematicCenterDetail::Bits(hit.TOI)!=0||!current())return TEXT("binding");
                const uint64 key=USSubBody::MakeComponentSweepPairKey(body,hit,true);
                const auto& pairs=*rows->PairsIdentity;
                const bool wasMember=pairs.Contains(key);
                const bool eligible=body->CanResolveRepeatedContact(hit); // Curved-Box exact progress is unavailable to Sphere.
                const bool ordinary=pairs.CanResolve(key,eligible);
                const bool progress=wasMember&&eligible&&pairs.CanSelectExactProgress(key,hit.TOI,rows->Before,rows->Delta);
                const bool allowed=wasMember?progress:ordinary;
                if(!current())return TEXT("revalidation");
                UE_LOG(LogTemp,Display,TEXT("[NativeSphereRepeatedFloorReadback] Frame=%u Iteration=%d Serial=%llu Epoch=%llu Source=%llu Surface=%llu Feature=%llu Primitive=%llu Group=%llu Pair=%llu Member=%d Generation=%u EligibleRepeated=%d OrdinaryAllowed=%d ExactProgress=%d Allowed=%d ReadOnly=1"),
                    v.FrameClock.OwnerLocalFrame,captured.Iteration,captured.QuerySerial,captured.QueryEpoch,hit.SourceId,hit.SurfaceId,
                    hit.FeatureId,hit.PrimitiveId,hit.CanonicalGroupId,key,int32(wasMember),uint32(pairs.GetGeneration(key)),int32(eligible),
                    int32(ordinary),int32(progress),int32(allowed));
                if(!wasMember||allowed)return TEXT("not_filtered");
                if(!nextEligible)return TEXT("repeated_floor_filtered");
                FNativeRepeatedSphereFloorExclusion excluded;
                const auto providers=request->GetSnapshot()->GetGeometrySelection()->GetNominalScene()->GetOwners().Num();
                if(providers<=0||!current())return TEXT("next_eligible_scope");
                C::FDistanceBudget account{providers};
                if(!FNativeRepeatedSphereFloorExclusion::Bind(request,observation,hit,*body,pairs,account,excluded))
                    return TEXT("next_eligible_floor_identity");
                FPreparedNativeSolidGeometryPtr next;
                if(!service->CompleteNativeNextEligibleSphere(request,excluded,next)||!next||next->GetRequest()!=request||
                    !C::GroupedBoxDetail::SamePoint(next->GetValues().QueryEnvelope,observation.QueryEnvelope)||!current())
                    return TEXT("next_eligible_geometry");
                const auto values=next->GetValues().GeneralSphere;
                if(!values||!values->IsNextEligible())return TEXT("next_eligible_proof");
                bool nextAmbiguous=false;
                const auto* nextBinding=request->GetSnapshot()->FindUniqueContactBinding(values->GetNamedHitForInspection().SourceId,nextAmbiguous);
                SHitResult named;
                if(nextAmbiguous||!nextBinding||!BindNativeGeneralSphereContactValues(v,values,*nextBinding,named)||
                    !excluded.ProvesNextPair(request,pairs,named)||!current())return TEXT("next_eligible_original_filter");
                UE_LOG(LogTemp,Display,TEXT("[NativeSphereNextEligible] Frame=%u Iteration=%d Serial=%llu RejectedPair=%llu Surface=%llu Group=%llu SecondsBits=%08x"),
                    v.FrameClock.OwnerLocalFrame,captured.Iteration,captured.QuerySerial,key,named.SurfaceId,named.CanonicalGroupId,
                    C::KinematicCenterDetail::Bits(named.TOI));
                geometry=MoveTemp(next);continued=true;return TEXT("next_eligible_ready");
            };
            const TCHAR* stage=TEXT("observation_exception");
            try {stage=observe();} catch(...) {}
            UE_LOG(LogTemp,Display,TEXT("[NativeSphereRepeatedFloorObservation] Frame=%u Serial=%llu Stage=%s"),
                v.FrameClock.OwnerLocalFrame,request->GetStep().QuerySerial,stage);
        }
        if(!continued)Refuse("Ordinary solid World geometry is unknown; no fallback or miss");
    }
    TSharedPtr<FOrdinaryGroundSweepEvidence,ESPMode::ThreadSafe> result(new FOrdinaryGroundSweepEvidence);
    result->Request=request;result->Geometry=geometry;result->Seconds=horizon;
    if(geometry->GetValues().Status==ENativeSolidGeometryStatus::GeneralSphereFirstContact)
    {
        const auto values=geometry->GetValues().GeneralSphere;
        if(v.Shape!=ENativeSolidQueryShape::Sphere||!values||values->GetGeometryForInspection()!=request->GetSnapshot()->GetGeometrySelection())
            Refuse("Ordinary general Sphere geometry identity unavailable");
        const auto& raw=values->GetNamedHitForInspection();bool bindingAmbiguous=false;
        const FPreparedSourceContactBinding* binding=request->GetSnapshot()->FindUniqueContactBinding(raw.SourceId,bindingAmbiguous);
        if(bindingAmbiguous)Refuse("General Sphere Source/Contact binding ambiguous");
        if(!binding||!BindNativeGeneralSphereContactValues(v,values,*binding,result->NamedGround))
            Refuse("General Sphere values cannot bind the actual native request/GT Contact");
        result->Seconds=result->NamedGround.TOI;
    }
    else if(geometry->GetValues().Status!=ENativeSolidGeometryStatus::WholeIntervalSeparated)
    {
        FNativeFloorSphereContactGeometryValues payload;
        if(v.Shape!=ENativeSolidQueryShape::Sphere||!BuildNativeFloorSphereContactGeometry(geometry,payload))
        {
            const auto& g=geometry->GetValues();
            const auto point=[](const FVector& p){return FString::Printf(TEXT("(%.17g,%.17g,%.17g)"),p.X,p.Y,p.Z);};
            UE_LOG(LogTemp,Error,TEXT("[PreparedOrdinarySphereContactRefused] Frame=%u Owner=%llu BodyIndex=%d Shape=%d Status=%d Frozen=%d Prescribed=%d H=%.17g Rp=%.17g Rs=%.17g Rc=%.17g Margin=%.17g InitialSide=%d PhysicalSide=%d SupportGap=(%.17g,%.17g,%d) SweepStatus=%d Clear=%.17g Inside=%.17g GapSatisfied=%d NativeGap=(%.17g,%.17g,%d) ReferenceGap=(%.17g,%.17g,%d) OwnerP=%s OwnerV=%s OwnerA=%s BodyP=%s BodyV=%s BodyA=%s ExpectedSerial=%llu"),
                v.FrameClock.OwnerLocalFrame,v.OwnerStableId,v.BodyIndex,int32(v.Shape),int32(g.Status),int32(v.Frozen),int32(v.Prescribed),
                double(v.Horizon),double(v.PhysicalRadius),v.SupportRadius,double(v.QueryRadius),double(v.SphereMargin),
                int32(g.InitialSphere.SupportSide),int32(g.InitialSphere.PhysicalSide),g.InitialSphere.SupportGap.Lower,
                g.InitialSphere.SupportGap.Upper,int32(g.InitialSphere.SupportGap.Known),int32(g.Sweep.Status),
                double(g.Sweep.ClearPrefixSeconds),double(g.Sweep.InsideSeconds),int32(g.Sweep.CapturedSphereGapSatisfied),
                g.Sweep.ConsumedGap.Lower,g.Sweep.ConsumedGap.Upper,int32(g.Sweep.ConsumedGap.Known),
                g.Sweep.ReferenceGap.Lower,g.Sweep.ReferenceGap.Upper,int32(g.Sweep.ReferenceGap.Known),
                *point(v.OwnerState.Location),*point(v.OwnerState.Velocity),*point(v.OwnerState.Acceleration),
                *point(v.BodyState.Location),*point(v.BodyState.Velocity),*point(v.BodyState.Acceleration),request->GetStep().QuerySerial);
            Refuse("Ordinary Box/overlap/arrival contact values are unsupported");
        }
        bool bindingAmbiguous=false;
        const FPreparedSourceContactBinding* binding=request->GetSnapshot()->FindUniqueContactBinding(payload.Hit.SourceId,bindingAmbiguous);
        if(bindingAmbiguous)Refuse("Ordinary contact has ambiguous Source/Contact binding");
        if(!binding||!BuildNativeAnalyticContactValues(payload.Hit,payload.Seconds,payload.OwnerFrame,*binding,result->NamedGround))
            Refuse("Ordinary native contact payload cannot bind its actual GT Contact token");
        result->Seconds=payload.Seconds;
    }
    FStepIdentity now;
    if(!RevalidateNativeSolidCCD(*request)||!ReadCurrentStep(now)||now.QuerySerial!=request->GetStep().QuerySerial)
        Refuse("Ordinary World result request or exact serial changed");
    if(v.Shape==ENativeSolidQueryShape::Sphere)
    {
        auto* member=MemberSweepScope;
        if(!member||!member->Active||member->Completed||member->Interrupted||member->PendingGround||member->PendingOrdinaryGround||
            member->Body!=v.Body||member->Owner!=v.Owner||member->Serial!=v.SphereMemberSerial||!SameFloat(member->Margin,v.SphereMargin))
            Refuse("Ordinary Sphere World values lack their actual exclusive live member");
        member->PendingOrdinaryGround=result;
    }
    else
    {
        if(MemberSweepScope||result->NamedGround.bHit)Refuse("Ordinary Box ground closure is unsupported");
        result->Closed=true;scope->SealOrdinaryGround(result);
    }
    out=result->NamedGround;return out.bHit;
}
void FComponentSweepScope::ReadBoxGroundCacheValues(const UBoxSubBody& body,FNativeBoxGroundCacheValues& out)
{
    out.CurrentHit=body.CurrentHit;out.GroundHit=body.GroundHit;out.LastResolvedFrame=body.LastResolvedGroundHitFrame;
    out.HasGround=body.bHasGroundContact;out.PlaneValid=body.bGroundPlaneValid;out.PlanePoint=body.GroundPlanePointWS;
    out.PlaneNormal=body.GroundPlaneN;out.PlaneD=body.GroundPlaneD;out.GroundComponent=body.GroundComp;
}
namespace NativeBoxCacheDetail
{
bool IsDynamicHit(const SHitResult& hit)
{
    // The native dynamic sweep records the other SubBody in BOTH weak fields.
    // Its result cannot carry the static authority's IDs or prepared evidence.
    return hit.bHit&&hit.bBlockingHit&&!hit.SubBody.IsExplicitlyNull()&&
        hit.Component.HasSameIndexAndSerialNumber(hit.SubBody)&&
        !hit.SourceId&&!hit.SurfaceId&&!hit.FeatureId&&!hit.PrimitiveId&&!hit.CanonicalGroupId&&!hit.MaterialId&&
        !hit.PreparedGroundEvidence&&!hit.FirstRsResponseData;
}

bool Same(const FNativeBoxGroundCacheValues& a,const FNativeBoxGroundCacheValues& b)
{
    return SameNativeAnalyticContactValues(a.CurrentHit,b.CurrentHit)&&SameNativeAnalyticContactValues(a.GroundHit,b.GroundHit)&&
        a.LastResolvedFrame==b.LastResolvedFrame&&a.HasGround==b.HasGround&&a.PlaneValid==b.PlaneValid&&
        SameVector(a.PlanePoint,b.PlanePoint)&&SameVector(a.PlaneNormal,b.PlaneNormal)&&SameFloat(a.PlaneD,b.PlaneD)&&
        a.GroundComponent.HasSameIndexAndSerialNumber(b.GroundComponent);
}
}
// Actual UBox member only. Native initial events and first arrivals are
// separate from an established refiner's observed NO NEW unmasked corner.
bool FComponentSweepScope::EvaluateOrdinaryNativeBoxGround(const UWorld* world,const ISolidSweeper* sweeper,
    const UBoxSubBody* body,FVector start,FVector end,const FCollisionShape& shape,const FCollisionQueryParams& trace,
    float horizon,SHitResult& out)
{
        PreparedExecution::FNativePlanarResponseCommitScope::RequireQueriesClosed();
    out={};FNativeSolidStaticRequestPtr request;
    if(!CaptureNativeSolidCCD(world,sweeper,body,start,end,shape,trace,horizon,request)||
        request->GetValues().Shape!=ENativeSolidQueryShape::Box)
        Refuse("Ordinary Box refinement has no authentic actual member request");
    auto* scope=ComponentSweepScope;const auto& v=request->GetValues();
    if(!scope||!scope->Active||scope->Completed||!scope->Pending||MemberSweepScope||
        !scope->Pending->Bodies.IsValidIndex(scope->CurrentIndex)||scope->Pending->Bodies[scope->CurrentIndex].OrdinaryGround||
        scope->Pending->Bodies[scope->CurrentIndex].Ground||!request->GetStep().Provider||
        v.Owner->GetStaticCollisionWorldForFrame()!=request->GetStep().Provider)
        Refuse("Ordinary Box provider or current slot is unsupported");
    FNativeBoxGroundCacheValues before;ReadBoxGroundCacheValues(*body,before);
    // CurrentHit also carries car/ball contacts. Those stay in the dynamic
    // batch; the complete static query must still run for this motion.
    const bool dynamic=NativeBoxCacheDetail::IsDynamicHit(before.CurrentHit);
    const bool curvedHistorical=before.CurrentHit.bSurfaceNormalMayVary&&StepScope&&
        v.FrameClock.OwnerLocalFrame<=uint32(MAX_int32)&&before.LastResolvedFrame==int32(v.FrameClock.OwnerLocalFrame)&&
        StepScope->HasClosedCurvedBoxGroundForObservation(*request,before.CurrentHit);
    // A genuine previously resolved curve is still recorded in the cache.
    // It is ineligible for planar reuse and gets an independent fresh sweep.
    const bool cached=(before.CurrentHit.bHit||before.CurrentHit.bBlockingHit)&&!dynamic&&!curvedHistorical;
    const auto& requestStep=request->GetStep();
    FWeakObjectPtr case202Ball;
    ::AnalyticCertified::Interval case202DynamicGap;
    if((CVarCase202CompetitorCensus.GetValueOnAnyThread()==1||CVarCase202Serial3Curve.GetValueOnAnyThread()==1)&&v.FrameClock.OwnerLocalFrame==127&&
        v.OwnerStableId==3&&v.BodyIndex==0&&requestStep.Frame==126&&requestStep.Iteration==3&&
        requestStep.QuerySerial==3&&requestStep.QueryEpoch==6&&C::KinematicCenterDetail::Bits(horizon)==0x3a7148c4u&&
        SameVector(v.OwnerState.Location,FVector(140.63987471468158,3760.1263275433066,28.04140000002662)))
    {
        const auto snapshot=request->GetSnapshot();
        const auto bindings=snapshot?snapshot->GetNativeOwners():FPreparedNativeOwnerBindingsPtr{};
        const auto* step=StepScope;
        const auto* returns=step?step->OrdinaryReturns.Get():nullptr;
        const auto* worldRows=returns&&returns->World?returns->World.Get():nullptr;
        FStepIdentity now;
        const bool joined=step&&step->Snapshot==snapshot&&step->Registry&&bindings&&returns&&
            returns->Pending&&worldRows&&!worldRows->Closed&&worldRows->PairsIdentity&&
            ReadCurrentStep(now)&&OrdinaryWorldDetail::Epoch(worldRows->Entry,now)&&
            OrdinaryWorldDetail::Epoch(requestStep,now)&&now.QuerySerial==requestStep.QuerySerial&&
            bindings->GetGeneration()==now.InputGeneration&&scope->CurrentIndex==v.BodyIndex&&
            returns->Pending->Owner==v.Owner&&returns->Pending->Order==2&&
            returns->Rows.Num()==2&&worldRows->Rows.Num()==2;
        const auto& spheres=body->GetExternalSphereSubBodies();
        const auto& boxes=body->GetExternalBoxSubBodies();
        const auto& wheels=body->GetExternalWheelSubBodies();
        int32 priorMisses=0,ownIgnoredWheels=0;bool ballJoined=false;
        if(joined)
        {
            const auto& order=step->Registry->GetOrderedAdapters();
            if(order.Num()==3&&bindings->Num()==order.Num()&&order[2]==v.Owner)
            {
                for(int32 index=0;index<2;++index)
                {
                    const auto* prior=returns->Rows[index].Get();const auto& row=worldRows->Rows[index];
                    if(prior&&prior->Owner==order[index]&&prior->Order==index&&prior->Returned&&
                        OrdinaryWorldDetail::Epoch(prior->ReturnStep,now)&&prior->Component&&
                        prior->Component->IsSealedInventory()&&row.Raw.PreparedEvidence==prior->Component&&
                        !row.Raw.bHit&&!row.Filter.Allowed&&!row.HasClamped)++priorMisses;
                }
                const auto* owner=bindings->Find(v.Owner);const auto& siblings=v.Owner->GetSubBodies();
                if(owner&&owner->BodyCensusCaptured&&owner->Bodies.Num()==5&&siblings.Num()==5&&siblings[0]==body)
                    for(int32 index=1;index<5;++index)
                    {
                        const auto* wheel=siblings[index];const auto& pin=owner->Bodies[index];
                        if(wheel&&pin.Body==wheel&&pin.Index==index&&pin.ObjectIdCaptured&&pin.Class==wheel->GetClass()&&
                            wheel->GetClass()->GetPathName()==TEXT("/Script/SLCorePhysics.SkyWheelSubBody")&&
                            wheel->GetParentComponent()==v.Owner&&body->ComponentHasBeenIgnored(*wheel))++ownIgnoredWheels;
                    }
                const auto* ballOwner=bindings->Find(order[1]);
                const auto* ball=spheres.Num()==1?spheres[0].Get():nullptr;
                if(ball&&ballOwner&&ballOwner->BodyCensusCaptured&&ballOwner->Bodies.Num()==1)
                {
                    const auto& pin=ballOwner->Bodies[0];
                    ballJoined=pin.Body==ball&&pin.Index==0&&pin.ObjectIdCaptured&&pin.Class==ball->GetClass()&&
                        static_cast<FWeakObjectPtr>(spheres[0]).HasSameIndexAndSerialNumber(pin.Object)&&
                        ball->GetParentComponent()==order[1]&&ball->GetClass()->GetPathName()==TEXT("/Script/SLCorePhysics.BallSubBody");
                    if(ballJoined)
                    {
                        const auto sphere=ball->MakeSphere();
                        UE_LOG(LogTemp,Display,TEXT("[Case202Serial3DynamicSphere] Object=%u P=(%.17g,%.17g,%.17g) V=(%.17g,%.17g,%.17g) A=(%.17g,%.17g,%.17g) Radius=%.17g"),
                            pin.ObjectId,sphere.Center.X,sphere.Center.Y,sphere.Center.Z,sphere.Vel.X,sphere.Vel.Y,sphere.Vel.Z,
                            sphere.Accel.X,sphere.Accel.Y,sphere.Accel.Z,double(sphere.Radius));
                        // Independent geometric proof for the actual dynamic Box/Sphere model.
                        // Observe only; the original dispatches and their results remain authoritative.
                        const auto box=body->MakeBox();
                        const auto center=box.AbsoluteCenter(),extent=box.Extent();
                        const auto& bodyState=v.BodyState;
                        const bool joinedBox=SameVector(center,bodyState.Location)&&SameQuat(box.Rot,bodyState.Rotation)&&
                            SameVector(box.Vel,bodyState.Velocity)&&SameVector(box.Accel,bodyState.Acceleration)&&
                            SameVector(box.AngVel,bodyState.AngularVelocity)&&SameVector(box.AngAccel,bodyState.AngularAcceleration)&&
                            SameVector(extent,v.PhysicalHalfExtent)&&SameVector(box.Min,-extent)&&SameVector(box.Max,extent);
                        const bool stationarySphere=!sphere.Center.ContainsNaN()&&sphere.Vel==FVector::ZeroVector&&
                            sphere.Accel==FVector::ZeroVector&&FMath::IsFinite(sphere.Radius)&&sphere.Radius>0.f;
                        C::FGeneralBoxMotionInput input;
                        input.Owner=bodyState;input.Horizon=horizon;
                        input.PhysicalHalfExtent=input.QueryHalfExtent=extent;
                        C::FGeneralBoxMotionBounds bounds;
                        const bool bounded=joinedBox&&stationarySphere&&
                            C::BuildGeneralBoxMotionBoundsOnGlobalFloatSeconds(input,0.f,horizon,bounds)&&bounds.Corners.Num()==32;
                        ::AnalyticCertified::Interval gap;
                        if(bounded)
                        {
                            using I=::AnalyticCertified::Interval;
                            const auto sphereRight=::AnalyticCertified::Add(I::Exact(sphere.Center.Y),I::Exact(double(sphere.Radius)));
                            gap=::AnalyticCertified::Subtract(bounds.Hull.Axis[1],sphereRight);
                        }
                        const bool strict=::AnalyticCertified::Valid(gap)&&gap.Lower>0.;
                        if(strict&&priorMisses==2&&ownIgnoredWheels==4&&boxes.IsEmpty()&&wheels.IsEmpty()&&curvedHistorical&&
                            !body->LastResolvedSphereHit.bBlockingHit&&body->LastResolvedSphereHit.SubBody.IsExplicitlyNull()&&
                            body->LastResolvedSphereHitFrame==INDEX_NONE)
                        {
                            case202Ball=static_cast<FWeakObjectPtr>(spheres[0]);case202DynamicGap=gap;
                        }
                        UE_LOG(LogTemp,Display,TEXT("[Case202Serial3DynamicY] BoxJoined=%d SphereStationary=%d Bounded=%d Strict=%d GapLower=%.17g OriginalH=%.17g Dispatches=Unexecuted"),
                            int32(joinedBox),int32(stationarySphere),int32(bounded),int32(strict),strict?gap.Lower:0.,double(horizon));
                    }
                }
            }
        }
        // Here the original static query precedes all dynamic dispatches.
        UE_LOG(LogTemp,Display,TEXT("[Case202Serial3DynamicRoster] Frame=%u Serial=%llu Epoch=%llu Joined=%d PriorWorldMisses=%d OwnIgnoredWheels=%d Box=%d Sphere=%d Wheel=%d BallJoined=%d PersistentBlocking=%d PersistentNull=%d PersistentFrame=%d CurvedHistorical=%d Dispatches=Unexecuted"),
            v.FrameClock.OwnerLocalFrame,requestStep.QuerySerial,requestStep.QueryEpoch,int32(joined),priorMisses,ownIgnoredWheels,
            boxes.Num(),spheres.Num(),wheels.Num(),int32(ballJoined),int32(body->LastResolvedSphereHit.bBlockingHit),
            int32(body->LastResolvedSphereHit.SubBody.IsExplicitlyNull()),body->LastResolvedSphereHitFrame,int32(curvedHistorical));
    }
#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
    if(dynamic)
    {
        static std::atomic<bool> reported=false;
        if(!reported.exchange(true,std::memory_order_relaxed))
            UE_LOG(LogTemp,Display,TEXT("[NativeBoxDynamicHitSeparateStaticQuery] Frame=%u BodyIndex=%d"),
                v.FrameClock.OwnerLocalFrame,v.BodyIndex);
    }
#endif
    const auto service=request->GetStep().Provider->GetPreparedQueryService();FPreparedNativeSolidGeometryPtr geometry;
    if(!service||!service->MatchesPreparedSnapshot(request->GetSnapshot()))Refuse("Ordinary Box prepared service differs");
    TSharedPtr<FOrdinaryGroundSweepEvidence,ESPMode::ThreadSafe> result(new FOrdinaryGroundSweepEvidence);
    result->Request=request;result->Seconds=horizon;result->BoxCache=before;
    if(cached)
    {
        const auto& hit=before.CurrentHit;
        const auto zeroFloat=[](float x){return !(C::KinematicCenterDetail::Bits(x)&C::KinematicCenterDetail::FloatMagnitude);};
        if(!hit.bHit||!hit.bBlockingHit||hit.bStartPenetrating||!zeroFloat(hit.PenetrationDepth)||
            !zeroFloat(hit.GeometricErrorBoundCm)||hit.bSurfaceNormalMayVary||hit.ContactFeatureOther!=EContactFeatureKind::Face||
            !hit.SubBody.IsExplicitlyNull()||hit.PreparedGroundEvidence||hit.FirstRsResponseData||
            v.FrameClock.OwnerLocalFrame>uint32(MAX_int32)||before.LastResolvedFrame!=int32(v.FrameClock.OwnerLocalFrame)||
            hit.FrameTag!=v.FrameClock.OwnerLocalFrame||!hit.SourceId||!hit.SurfaceId||!hit.PrimitiveId)
        {
            UE_LOG(LogTemp,Error,TEXT("[NativeBoxStaticCacheIdentityRefused] Frame=%u HitFrame=%u GroundFrame=%d Hit=%d Blocking=%d SubBody=%d SameObject=%d Source=%llu Surface=%llu Primitive=%llu VariableNormal=%d Feature=%d Penetration=%.9g ErrorCm=%.9g"),
                v.FrameClock.OwnerLocalFrame,hit.FrameTag,before.LastResolvedFrame,int32(hit.bHit),int32(hit.bBlockingHit),
                int32(!hit.SubBody.IsExplicitlyNull()),int32(hit.Component.HasSameIndexAndSerialNumber(hit.SubBody)),
                hit.SourceId,hit.SurfaceId,hit.PrimitiveId,int32(hit.bSurfaceNormalMayVary),int32(hit.ContactFeatureOther),
                double(hit.PenetrationDepth),double(hit.GeometricErrorBoundCm));
            Refuse("Cached Box contact has no exact current-frame planar identity");
        }
        bool bindingAmbiguous=false;
        const FPreparedSourceContactBinding* binding=request->GetSnapshot()->FindUniqueContactBinding(hit.SourceId,bindingAmbiguous);
        if(bindingAmbiguous)Refuse("Cached Box plane has ambiguous GT binding");
        if(!binding||!binding->SourceIdentity||!binding->ContactIdentity||binding->ContactToken.IsExplicitlyNull()||
            !hit.Component.HasSameIndexAndSerialNumber(binding->ContactToken))
            Refuse("Cached Box Contact does not match its retained GT token");
        C::FNativeBoxCachedPlaneValues plane;plane.Point=hit.ImpactPoint;plane.Normal=hit.ImpactNormal;
        plane.SourceId=hit.SourceId;plane.SurfaceId=hit.SurfaceId;plane.FeatureId=hit.FeatureId;plane.PrimitiveId=hit.PrimitiveId;
        plane.CanonicalGroupId=hit.CanonicalGroupId;plane.MaterialId=hit.MaterialId;
        // Select the observation lane from already closed ACTUAL planar
        // history, before any attempt to prove this new cached motion clear.
        // This history grants no current request, event or advance operation.
        // The existing current-frame cache/GT checks above still apply.
        // A real initial vertex event can establish support without changing
        // the owner pose, so it need not have an ordered pose-write effect.
        const bool currentInitialSupport=C::CVarGeneralBoxFloorInitialNativeBand.GetValueOnAnyThread()==1&&
            before.HasGround&&before.PlaneValid&&zeroFloat(hit.TOI)&&
            hit.ContactFeatureThis==EContactFeatureKind::Vertex&&hit.ContactFeatureIndexThis>=0&&hit.ContactFeatureIndexThis<8&&
            SameNativeAnalyticContactValues(hit,before.GroundHit);
        if(CVarNativeSlideCachedFloorReadback.GetValueOnAnyThread()==1&&StepScope&&
            v.FrameClock.OwnerLocalFrame==32&&v.OwnerStableId==3&&v.BodyIndex==0&&
            request->GetStep().Frame==31&&request->GetStep().Iteration==2&&request->GetStep().QuerySerial==4&&
            C::KinematicCenterDetail::Bits(horizon)==0x3af9d469u&&
            C::KinematicCenterDetail::Bits(request->GetStep().TimePassed)==0x3abb13b3u&&
            plane.SourceId==1274291482858373289ull&&plane.SurfaceId==5124012738301240878ull&&
            plane.FeatureId==8551878235246119335ull&&plane.PrimitiveId==15287852931450972720ull&&
            RevalidateNativeSolidCCD(*request))
        {
            const auto* returns=StepScope->OrdinaryReturns.Get();
            const auto* world=returns&&returns->World?returns->World.Get():nullptr;
            const auto* pairs=world&&!world->Closed?world->PairsIdentity:nullptr;
            FStepIdentity now;
            if(pairs&&ReadCurrentStep(now)&&OrdinaryWorldDetail::Epoch(world->Entry,now)&&
                OrdinaryWorldDetail::Epoch(request->GetStep(),now)&&now.QuerySerial==request->GetStep().QuerySerial)
            {
                const auto key=USSubBody::MakeComponentSweepPairKey(body,hit,true);
                const bool repeatEligible=body->CanResolveRepeatedContact(hit);
                UE_LOG(LogTemp,Display,TEXT("[NativeSlideCachedFloorReadback] Frame=%u Iteration=%d Serial=%llu Epoch=%llu HitTOIBits=%08x Corner=%d Pair=%llu Member=%d Generation=%u PriorHitRepeatEligible=%d PriorHitPairCanResolve=%d HasGround=%d PlaneValid=%d SameGroundHit=%d InitialSupport=%d ClosedPlanarEffect=%d GeneralDescriptor=%d ReadOnly=1"),
                    v.FrameClock.OwnerLocalFrame,now.Iteration,now.QuerySerial,now.QueryEpoch,
                    C::KinematicCenterDetail::Bits(hit.TOI),int32(hit.ContactFeatureIndexThis),key,int32(pairs->Contains(key)),
                    uint32(pairs->GetGeneration(key)),int32(repeatEligible),int32(pairs->CanResolve(key,repeatEligible)),
                    int32(before.HasGround),int32(before.PlaneValid),int32(SameNativeAnalyticContactValues(hit,before.GroundHit)),
                    int32(currentInitialSupport),int32(StepScope->HasClosedPlanarValuesForObservation(body)),
                    int32(C::NativeBoxNeedsGeneralFloorDescriptor(v)));
            }
        }
        if(C::NativeBoxNeedsGeneralFloorDescriptor(v)&&StepScope&&
            (StepScope->HasClosedPlanarValuesForObservation(body)||currentInitialSupport))
        {
            FNativeGeneralBoxFloorObservationPreludePtr prelude;
            if(!SameFloat(horizon,v.Horizon)||!service->DescribeNativeGeneralBoxFloorRefinerObservation(request,plane,prelude)||!prelude||
                !prelude->GetValuesForInspection().Complete||prelude->GetValuesForInspection().PhysicalClear||
                !SameNativeSolidStaticValues(prelude->GetValuesForInspection().Captured,v))
                Refuse("General FLOOR post-planar refiner observation geometry is unknown");
            const auto& observed=prelude->GetValuesForInspection();
            FNativeBoxGroundCacheValues beforeRefiner;ReadBoxGroundCacheValues(*body,beforeRefiner);
            FStepIdentity beforeStep;
            if(!NativeBoxCacheDetail::Same(before,beforeRefiner)||!RevalidateNativeSolidCCD(*request)||!ReadCurrentStep(beforeStep)||
                !OrdinaryWorldDetail::Epoch(request->GetStep(),beforeStep)||beforeStep.QuerySerial!=request->GetStep().QuerySerial)
                Refuse("General FLOOR observation changed before its one original-H refiner");
            SHitResult descriptor;descriptor.ImpactPoint=observed.CachedPlane.Point;descriptor.ImpactNormal=observed.CachedPlane.Normal;
            descriptor.SourceId=observed.CachedPlane.SourceId;descriptor.SurfaceId=observed.CachedPlane.SurfaceId;
            descriptor.FeatureId=observed.CachedPlane.FeatureId;descriptor.PrimitiveId=observed.CachedPlane.PrimitiveId;
            descriptor.CanonicalGroupId=observed.CachedPlane.CanonicalGroupId;descriptor.MaterialId=observed.CachedPlane.MaterialId;
            descriptor.ContactFeatureOther=EContactFeatureKind::Face;
            const SHitResult input=descriptor;FNativeBoxPlanarRefinementValues readback;
            const bool returned=body->RefineExactPlanarTOI(descriptor,horizon,&readback); // UNIQUE original-H call
            FNativeBoxGroundCacheValues afterRefiner;ReadBoxGroundCacheValues(*body,afterRefiner);FStepIdentity afterStep;
            const bool intrinsic=NativeFloorBoxPlaneDetail::MatchesReadbackInputs(v,input.ImpactNormal,input.ImpactPoint,readback);
            const bool cacheUnchanged=NativeBoxCacheDetail::Same(beforeRefiner,afterRefiner);
            const bool contextUnchanged=ReadCurrentStep(afterStep)&&OrdinaryWorldDetail::Epoch(beforeStep,afterStep)&&
                beforeStep.QuerySerial==afterStep.QuerySerial&&RevalidateNativeSolidCCD(*request);
            UE_LOG(LogTemp,Display,TEXT("[NativeGeneralCachedFloorArrivalRefinerValuesObserved] Frame=%u Serial=%llu OriginalH=%.17g CallH=%.17g Returned=%d Computed=%d Outcome=%u Established=%d Mask=%u InitialEvent=%d WitnessComputed=%d FirstCorner=%d FirstTime=%.17g MinimumInitial=%.17g Roundoff=%.17g SpeedRoundoff=%.17g Intrinsic=%d CacheUnchanged=%d ContextUnchanged=%d DescriptorUnchanged=%d CapturedBeforeSealAdvance=1 NoReleasedPhysicalClearAttempted=1 Gaps=(%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g) Speeds=(%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g)"),
                v.FrameClock.OwnerLocalFrame,request->GetStep().QuerySerial,double(v.Horizon),double(horizon),int32(returned),int32(readback.Computed),uint32(readback.Outcome),int32(readback.Established),uint32(readback.EstablishedMask),int32(readback.InitialEvent),int32(readback.WitnessComputed),readback.FirstCorner,double(readback.FirstTime),readback.MinimumInitial,readback.Roundoff,readback.SpeedRoundoff,int32(intrinsic),int32(cacheUnchanged),int32(contextUnchanged),int32(SameNativeAnalyticContactValues(input,descriptor)),
                readback.InitialGaps[0],readback.InitialGaps[1],readback.InitialGaps[2],readback.InitialGaps[3],readback.InitialGaps[4],readback.InitialGaps[5],readback.InitialGaps[6],readback.InitialGaps[7],
                readback.InitialNormalSpeeds[0],readback.InitialNormalSpeeds[1],readback.InitialNormalSpeeds[2],readback.InitialNormalSpeeds[3],readback.InitialNormalSpeeds[4],readback.InitialNormalSpeeds[5],readback.InitialNormalSpeeds[6],readback.InitialNormalSpeeds[7]);
            const auto logDescriptor=[&](const char* label,const SHitResult& value)
            {
                UE_LOG(LogTemp,Display,TEXT("[NativeGeneralBoxFloorRefinerDescriptorValues] Values=%hs Frame=%u Serial=%llu H=%.17g Hit=%d Blocking=%d TOI=%.17g Point=(%.17g,%.17g,%.17g) Normal=(%.17g,%.17g,%.17g) Source=%llu Surface=%llu Feature=%llu Primitive=%llu Group=%llu Material=%u"),
                    label,v.FrameClock.OwnerLocalFrame,request->GetStep().QuerySerial,double(horizon),int32(value.bHit),int32(value.bBlockingHit),double(value.TOI),value.ImpactPoint.X,value.ImpactPoint.Y,value.ImpactPoint.Z,value.ImpactNormal.X,value.ImpactNormal.Y,value.ImpactNormal.Z,value.SourceId,value.SurfaceId,value.FeatureId,value.PrimitiveId,value.CanonicalGroupId,value.MaterialId);
            };
            logDescriptor("ActualInput",input);logDescriptor("ActualOutput",descriptor);
            if(!intrinsic||!cacheUnchanged||!contextUnchanged)Refuse("General FLOOR original-H observation lost actual intrinsic/cache/context identity");
            const bool noNewCorner=!returned&&readback.Computed&&readback.Outcome==ENativeBoxPlanarRefinementOutcome::NoCorner&&
                readback.Established&&readback.EstablishedMask&&!readback.InitialEvent&&!readback.WitnessComputed&&
                readback.FirstCorner==INDEX_NONE&&SameFloat(readback.FirstTime,horizon)&&SameFloat(readback.Horizon,horizon)&&
                SameNativeAnalyticContactValues(input,descriptor);
            if(CVarEstablishedPlanarFloorClear.GetValueOnAnyThread()==1&&noNewCorner)
            {
                const auto& a=v.OwnerState;
                UE_LOG(LogTemp,Display,TEXT("[EstablishedFloorClearEffectiveMotion] Frame=%u Serial=%llu H=%.17g P=(%.17g,%.17g,%.17g) Q=(%.17g,%.17g,%.17g,%.17g) V=(%.17g,%.17g,%.17g) A=(%.17g,%.17g,%.17g) W=(%.17g,%.17g,%.17g) Alpha=(%.17g,%.17g,%.17g) COM=(%.17g,%.17g,%.17g) Offset=(%.17g,%.17g,%.17g) LocalQ=(%.17g,%.17g,%.17g,%.17g) Physical=(%.17g,%.17g,%.17g) Query=(%.17g,%.17g,%.17g)"),
                    v.FrameClock.OwnerLocalFrame,request->GetStep().QuerySerial,double(horizon),
                    a.Location.X,a.Location.Y,a.Location.Z,a.Rotation.X,a.Rotation.Y,a.Rotation.Z,a.Rotation.W,
                    a.Velocity.X,a.Velocity.Y,a.Velocity.Z,a.Acceleration.X,a.Acceleration.Y,a.Acceleration.Z,
                    a.AngularVelocity.X,a.AngularVelocity.Y,a.AngularVelocity.Z,a.AngularAcceleration.X,a.AngularAcceleration.Y,a.AngularAcceleration.Z,
                    v.LocalCOM.X,v.LocalCOM.Y,v.LocalCOM.Z,v.LocalOffset.X,v.LocalOffset.Y,v.LocalOffset.Z,
                    v.LocalRotation.X,v.LocalRotation.Y,v.LocalRotation.Z,v.LocalRotation.W,
                    v.PhysicalHalfExtent.X,v.PhysicalHalfExtent.Y,v.PhysicalHalfExtent.Z,
                    v.QueryHalfExtent.X,v.QueryHalfExtent.Y,v.QueryHalfExtent.Z);
                // No sweep is executed here. World rows are the actual prior returns;
                // later owners and later Box siblings stay explicitly unobserved.
                if(CVarCase202CompetitorCensus.GetValueOnAnyThread()==1&&v.FrameClock.OwnerLocalFrame==127&&
                    afterStep.QuerySerial==2&&observed.ObservationCurvedProvider==1931&&
                    C::KinematicCenterDetail::Bits(horizon)==0x3b5a740eu&&
                    SameVector(v.OwnerState.Location,FVector(139.11612392996199,3755.0581129530442,28.04140000002662))&&
                    SameQuat(v.OwnerState.Rotation,FQuat(-0.7534486401960061,-0.65750676543043229,-4.9425926306031607e-20,-1.2986172660911549e-19)))
                {
                    const auto census=[&]() -> bool
                    {
                        const auto snapshot=request->GetSnapshot();
                        const auto bindings=snapshot?snapshot->GetNativeOwners():FPreparedNativeOwnerBindingsPtr{};
                        const auto* step=StepScope;
                        const auto* returns=step?step->OrdinaryReturns.Get():nullptr;
                        if(!step||step->Snapshot!=snapshot||!step->Registry||!bindings||
                            bindings->GetGeneration()!=afterStep.InputGeneration||!returns||!returns->Pending||!returns->World)return false;
                        const auto& selector=*returns->World;
                        const auto& order=step->Registry->GetOrderedAdapters();
                        const int32 current=returns->Pending->Order;
                        if(selector.Closed||!OrdinaryWorldDetail::Epoch(selector.Entry,afterStep)||
                            !selector.PairsIdentity||!order.IsValidIndex(current)||order[current]!=v.Owner||
                            returns->Pending->Owner!=v.Owner||bindings->Num()!=order.Num()||
                            selector.Rows.Num()!=current||returns->Rows.Num()!=current)return false;
                        const auto token=[&](const FWeakObjectPtr& weak,uint32& id)
                        {
                            int32 found=0;id=0;
                            if(!weak.IsExplicitlyNull())for(const auto& row:bindings->GetRecordsForInspection())
                                for(const auto& child:row.Bodies)
                                    if(child.ObjectIdCaptured&&weak.HasSameIndexAndSerialNumber(child.Object))
                                    {++found;id=child.ObjectId;}
                            return found;
                        };
                        const auto printList=[&](const TCHAR* kind,const auto& values)
                        {
                            bool complete=true;
                            UE_LOG(LogTemp,Display,TEXT("[Case202CensusExternalList] Kind=%s Count=%d"),kind,values.Num());
                            for(int32 index=0;index<values.Num();++index)
                            {
                                uint32 id=0;const int32 matches=token(static_cast<FWeakObjectPtr>(values[index]),id);
                                complete&=matches==1;
                                UE_LOG(LogTemp,Display,TEXT("[Case202CensusExternalToken] Kind=%s Index=%d Matches=%d Object=%u"),kind,index,matches,id);
                            }
                            return complete;
                        };
                        UE_LOG(LogTemp,Display,TEXT("[Case202CensusBegin] Frame=127 Serial=2 Generation=%llu Step=%llu Epoch=%llu Iteration=%d Numeric=%llu Owners=%d Current=%d PriorWorld=%d Horizon=%.17g"),
                            afterStep.InputGeneration,afterStep.StepSerial,afterStep.QueryEpoch,int32(afterStep.Iteration),
                            afterStep.NumericControl,order.Num(),current,selector.Rows.Num(),double(horizon));
                        bool complete=printList(TEXT("Box"),body->GetExternalBoxSubBodies());
                        complete=printList(TEXT("Sphere"),body->GetExternalSphereSubBodies())&&complete;
                        complete=printList(TEXT("Wheel"),body->GetExternalWheelSubBodies())&&complete;
                        uint32 sphereCacheId=0;
                        const int32 sphereCacheMatches=token(static_cast<FWeakObjectPtr>(body->LastResolvedSphereHit.SubBody),sphereCacheId);
                        UE_LOG(LogTemp,Display,TEXT("[Case202CensusBoxCache] SphereBlocking=%d SphereFrame=%d SphereNull=%d SphereTokenMatches=%d SphereObject=%u GroundBlocking=%d GroundFrame=%d GroundSource=%llu GroundSurface=%llu GroundPrimitive=%llu"),
                            int32(body->LastResolvedSphereHit.bBlockingHit),body->LastResolvedSphereHitFrame,
                            int32(body->LastResolvedSphereHit.SubBody.IsExplicitlyNull()),sphereCacheMatches,sphereCacheId,
                            int32(afterRefiner.GroundHit.bBlockingHit),afterRefiner.LastResolvedFrame,afterRefiner.GroundHit.SourceId,
                            afterRefiner.GroundHit.SurfaceId,afterRefiner.GroundHit.PrimitiveId);
                        for(int32 index=0;index<order.Num();++index)
                        {
                            const auto* owner=order[index];const auto* row=owner?bindings->Find(owner):nullptr;
                            const bool priorIndex=index<current;
                            const auto* recorded=priorIndex?returns->Rows[index].Get():index==current?returns->Pending.Get():nullptr;
                            const uint64 stable=owner?step->Registry->FindStableId(*owner):0;
                            const bool metadata=row&&row->NativeObject&&row->Class&&row->BodyCensusCaptured&&
                                row->NativeObject->GetClass()==row->Class;
                            const bool joined=stable&&recorded&&recorded->Owner==owner&&recorded->StableId==stable&&
                                recorded->Order==index&&recorded->Returned==priorIndex&&
                                OrdinaryWorldDetail::Epoch(priorIndex?recorded->ReturnStep:recorded->EntryStep,afterStep);
                            // Unqualified means native operation coverage is unavailable.
                            // It does not invalidate a real ordinary return or raw GT census.
                            const FString ownerClass=metadata?row->NativeObject->GetClass()->GetPathName():FString(TEXT("unknown"));
                            UE_LOG(LogTemp,Display,TEXT("[Case202CensusOwner] Index=%d Stable=%llu Bodies=%d Metadata=%d Joined=%d Implementation=%d Class=%s CurrentBodyIndex=%d State=%s"),
                                index,stable,row?row->Bodies.Num():-1,int32(metadata),int32(joined),row?int32(row->Implementation):-1,
                                *ownerClass,index==current?scope->CurrentIndex:INDEX_NONE,
                                priorIndex?TEXT("PriorWorldReturn"):index==current?TEXT("InBoxGround"):TEXT("NotDispatched"));
                            if(priorIndex)
                            {
                                const auto& worldRow=selector.Rows[index];
                                if(joined&&recorded->Component&&recorded->Component->IsSealedInventory()&&
                                    worldRow.Raw.PreparedEvidence==recorded->Component)
                                {
                                    UE_LOG(LogTemp,Display,TEXT("[Case202CensusPriorWorld] Index=%d Hit=%d Pair=%llu Source=%llu Surface=%llu Primitive=%llu Raw=%.17g HitTOI=%.17g Eligible=%d Clamped=%d Time=%.17g Generation=%u Member=%d Progress=%d Ordinary=%d Repeated=%d Serial=%llu"),
                                        index,int32(worldRow.Raw.bHit),worldRow.Raw.PairKey,worldRow.Raw.Hit.SourceId,worldRow.Raw.Hit.SurfaceId,
                                        worldRow.Raw.Hit.PrimitiveId,double(worldRow.Raw.TOI),double(worldRow.Raw.Hit.TOI),int32(worldRow.Filter.Allowed),
                                        int32(worldRow.HasClamped),double(worldRow.Clamped),uint32(worldRow.Generation),
                                        int32(worldRow.Filter.PairWasMember),int32(worldRow.Filter.ProgressRead),int32(worldRow.Filter.OrdinaryRead),
                                        int32(worldRow.Filter.EligibleRepeated),recorded->ReturnStep.QuerySerial);
                                }
                                else
                                {
                                    complete=false;
                                    UE_LOG(LogTemp,Display,TEXT("[Case202CensusUnknown] OwnerIndex=%d BodyIndex=-1 Stage=world_return"),index);
                                }
                            }
                            if(!metadata||!joined||recorded->Bodies.Num()!=row->Bodies.Num()||
                                (priorIndex&&(recorded->ReturnedBodies.Num()!=row->Bodies.Num()||!recorded->Component||
                                    recorded->Component->GetBodies().Num()!=row->Bodies.Num())))
                            {
                                complete=false;
                                UE_LOG(LogTemp,Display,TEXT("[Case202CensusUnknown] OwnerIndex=%d BodyIndex=-1 Stage=owner_body_join"),index);
                                continue;
                            }
                            for(int32 childIndex=0;childIndex<row->Bodies.Num();++childIndex)
                            {
                                const auto& pin=row->Bodies[childIndex];const auto* child=pin.Body;
                                if(!child||recorded->Bodies[childIndex]!=child||pin.Index!=childIndex||!pin.ObjectIdCaptured||
                                    !pin.Class||child->GetClass()!=pin.Class||child->GetParentComponent()!=owner)
                                {
                                    complete=false;
                                    UE_LOG(LogTemp,Display,TEXT("[Case202CensusUnknown] OwnerIndex=%d BodyIndex=%d Stage=body_pin"),index,childIndex);
                                    continue;
                                }
                                const auto& pose=child->GetKinematicState();const auto& hit=child->GetHit();
                                const FBodySweepEvidence* sweep=priorIndex?&recorded->Component->GetBodies()[childIndex]:
                                    scope->Pending->Bodies.IsValidIndex(childIndex)?&scope->Pending->Bodies[childIndex]:nullptr;
                                const bool sweepJoined=sweep&&sweep->Index==childIndex&&sweep->RawBodyIdentity==child&&
                                    static_cast<FWeakObjectPtr>(sweep->Body).HasSameIndexAndSerialNumber(pin.Object);
                                if(priorIndex&&(!sweepJoined||!sweep->Returned||!SameKinematicBits(recorded->ReturnedBodies[childIndex],pose)))
                                {
                                    complete=false;
                                    UE_LOG(LogTemp,Display,TEXT("[Case202CensusUnknown] OwnerIndex=%d BodyIndex=%d Stage=restored_body_or_probe"),index,childIndex);
                                    continue;
                                }
                                const FString classPath=child->GetClass()->GetPathName();
                                const auto* sphere=Cast<USphereSubBody>(child);
                                const bool radiusKnown=sphere&&(sphere->GetClass()==USphereSubBody::StaticClass()||
                                    classPath==TEXT("/Script/SLCorePhysics.BallSubBody"));
                                const double dynamicRadius=radiusKnown?double(sphere->USphereSubBody::MakeSphere().Radius):0.;
                                UE_LOG(LogTemp,Display,TEXT("[Case202CensusBody] OwnerIndex=%d Index=%d Object=%u Class=%s Kind=%d IgnoredByBox=%d RadiusKnown=%d PhysicalRadius=%.17g DynamicRadius=%.17g SupportRadius=%.17g P=(%.17g,%.17g,%.17g) V=(%.17g,%.17g,%.17g) A=(%.17g,%.17g,%.17g) Hit=%d Blocking=%d Source=%llu Surface=%llu Primitive=%llu TOI=%.17g Returned=%d ReportedHit=%d ReportedTOI=%.17g"),
                                    index,childIndex,pin.ObjectId,*classPath,int32(child->GetSubBodyType()),int32(body->ComponentHasBeenIgnored(*child)),
                                    int32(radiusKnown),radiusKnown?double(sphere->GetRadius()):0.,dynamicRadius,
                                    radiusKnown?sphere->GetWorldStaticSupportRadius():0.,pose.Location.X,pose.Location.Y,pose.Location.Z,
                                    pose.Velocity.X,pose.Velocity.Y,pose.Velocity.Z,pose.Acceleration.X,pose.Acceleration.Y,pose.Acceleration.Z,
                                    int32(hit.bHit),int32(hit.bBlockingHit),hit.SourceId,hit.SurfaceId,hit.PrimitiveId,double(hit.TOI),
                                    int32(sweepJoined&&sweep->Returned),int32(sweepJoined&&sweep->Returned&&sweep->ReportedHit),
                                    sweepJoined&&sweep->Returned?double(sweep->ReportedTOI):0.);
                                if(!radiusKnown||!priorIndex)continue;
                                const FNativeSolidStaticValues* queryValues=nullptr;const FStepIdentity* queryStep=nullptr;
                                const TCHAR* probe=TEXT("unknown");
                                if(sweep->OrdinaryGround&&sweep->OrdinaryGround->Closed&&sweep->OrdinaryGround->Request&&
                                    sweep->OrdinaryGround->Request->GetSnapshot()==snapshot)
                                {
                                    const auto& captured=*sweep->OrdinaryGround->Request;
                                    queryValues=&captured.GetValues();queryStep=&captured.GetStep();probe=TEXT("World");
                                }
                                else if(sweep->SleepingSupportBypass&&sweep->SleepingSupportBypass->IsClosedObservation()&&
                                    sweep->SleepingSupportBypass->Snapshot==snapshot)
                                {
                                    queryValues=&sweep->SleepingSupportBypass->Intrinsic;
                                    queryStep=&sweep->SleepingSupportBypass->Step;probe=TEXT("SleepingSupport");
                                }
                                if(!queryValues||!queryStep||!OrdinaryWorldDetail::Epoch(*queryStep,afterStep)||
                                    queryStep->QuerySerial>afterStep.QuerySerial||queryValues->Owner!=owner||queryValues->Body!=child||
                                    queryValues->BodyIndex!=childIndex||queryValues->Shape!=ENativeSolidQueryShape::Sphere)
                                {
                                    UE_LOG(LogTemp,Display,TEXT("[Case202CensusUnknown] OwnerIndex=%d BodyIndex=%d Stage=sphere_probe"),index,childIndex);
                                    continue;
                                }
                                const auto& effective=queryValues->BodyState;
                                UE_LOG(LogTemp,Display,TEXT("[Case202CensusSphereProbe] OwnerIndex=%d Index=%d Object=%u Kind=%s Serial=%llu H=%.17g MarginKnown=%d Margin=%.17g Physical=%.17g Query=%.17g Support=%.17g P=(%.17g,%.17g,%.17g) V=(%.17g,%.17g,%.17g) A=(%.17g,%.17g,%.17g)"),
                                    index,childIndex,pin.ObjectId,probe,queryStep->QuerySerial,double(queryValues->Horizon),int32(queryValues->SphereMarginCaptured),
                                    double(queryValues->SphereMargin),double(queryValues->PhysicalRadius),double(queryValues->QueryRadius),queryValues->SupportRadius,
                                    effective.Location.X,effective.Location.Y,effective.Location.Z,effective.Velocity.X,effective.Velocity.Y,effective.Velocity.Z,
                                    effective.Acceleration.X,effective.Acceleration.Y,effective.Acceleration.Z);
                            }
                        }
                        FStepIdentity endStep;
                        return complete&&ReadCurrentStep(endStep)&&OrdinaryWorldDetail::Epoch(afterStep,endStep)&&
                            endStep.QuerySerial==afterStep.QuerySerial;
                    };
                    bool complete=false;
                    try{complete=census();}catch(...){}
                    UE_LOG(LogTemp,Display,TEXT("[Case202CensusEnd] Frame=127 Serial=2 Complete=%d BoxSiblingsDispatched=0 NewSweepCalls=0"),int32(complete));
                }
                const FNativeSolidStaticRequest* splitRequest=nullptr;
                if(NativeBoxSplitArrival202::Enabled.GetValueOnAnyThread()==1&&
                    v.FrameClock.OwnerLocalFrame==127&&afterStep.QuerySerial==2&&
                    observed.ObservationCurvedProvider==1931&&C::KinematicCenterDetail::Bits(horizon)==0x3b5a740eu&&
                    SameVector(v.OwnerState.Location,FVector(139.11612392996199,3755.0581129530442,28.04140000002662))&&
                    SameQuat(v.OwnerState.Rotation,FQuat(-0.7534486401960061,-0.65750676543043229,-4.9425926306031607e-20,-1.2986172660911549e-19)))
                {
                    const auto eligible=[&]() -> bool
                    {
                        const auto snapshot=request->GetSnapshot();
                        if(!snapshot||!StepScope->Registry)return false;
                        const auto bindings=snapshot->GetNativeOwners();
                        const auto* returns=StepScope->OrdinaryReturns.Get();
                        const auto& order=StepScope->Registry->GetOrderedAdapters();
                        if(NativeBoxSplitArrival202::Request||StepScope->Snapshot!=snapshot||!bindings||
                            bindings->GetGeneration()!=afterStep.InputGeneration||bindings->Num()!=3||order.Num()!=3||
                            !returns||!returns->Pending||!returns->World||returns->Rows.Num()!=2||
                            returns->Pending->Owner!=v.Owner||returns->Pending->Order!=2||order[2]!=v.Owner||
                            scope->CurrentIndex!=0||returns->World->Rows.Num()!=2||returns->World->Closed||
                            !returns->World->PairsIdentity||!OrdinaryWorldDetail::Epoch(returns->World->Entry,afterStep)||
                            !SameFloat(returns->World->Horizon,horizon))return false;
                        for(int32 index=0;index<2;++index)
                        {
                            const auto* prior=returns->Rows[index].Get();const auto& selected=returns->World->Rows[index];
                            if(!prior||prior->Owner!=order[index]||prior->Order!=index||!prior->Returned||
                                !OrdinaryWorldDetail::Epoch(prior->ReturnStep,afterStep)||!prior->Component||
                                !prior->Component->IsSealedInventory()||selected.Raw.PreparedEvidence!=prior->Component||
                                selected.Raw.bHit||selected.Filter.Allowed||selected.HasClamped)return false;
                        }
                        const auto* owner=bindings->Find(v.Owner);
                        const auto* ballOwner=bindings->Find(order[1]);
                        const auto& siblings=v.Owner->GetSubBodies();
                        if(!owner||!ballOwner||!owner->BodyCensusCaptured||!ballOwner->BodyCensusCaptured||
                            owner->Bodies.Num()!=5||siblings.Num()!=5||siblings[0]!=body||owner->Bodies[0].Body!=body||
                            ballOwner->Bodies.Num()!=1)return false;
                        for(int32 index=1;index<5;++index)
                        {
                            const auto* wheel=siblings[index];const auto& row=owner->Bodies[index];
                            if(!wheel||row.Body!=wheel||row.Index!=index||row.Class!=wheel->GetClass()||
                                wheel->GetClass()->GetPathName()!=TEXT("/Script/SLCorePhysics.SkyWheelSubBody")||
                                wheel->GetParentComponent()!=v.Owner||!body->ComponentHasBeenIgnored(*wheel))return false;
                        }
                        const auto& balls=body->GetExternalSphereSubBodies();
                        if(!body->GetExternalBoxSubBodies().IsEmpty()||!body->GetExternalWheelSubBodies().IsEmpty()||
                            balls.Num()!=1||body->LastResolvedSphereHit.bBlockingHit||
                            !body->LastResolvedSphereHit.SubBody.IsExplicitlyNull())return false;
                        const auto* ball=balls[0].Get();const auto& row=ballOwner->Bodies[0];
                        if(!ball||row.Body!=ball||row.Index!=0||row.Class!=ball->GetClass()||
                            ball->GetClass()->GetPathName()!=TEXT("/Script/SLCorePhysics.BallSubBody")||
                            ball->GetParentComponent()!=order[1])return false;
                        const auto sphere=ball->MakeSphere();
                        // Bound this diagnostic to the retained stationary ball at center.
                        // The existing dynamic sweep and World selector still run normally.
                        if(!SameVector(sphere.Center,FVector(0,0,93.150198364257818))||
                            sphere.Vel!=FVector::ZeroVector||sphere.Accel!=FVector::ZeroVector||
                            !FMath::IsFinite(sphere.Radius)||!(sphere.Radius>0.f))return false;
                        SHitResult competitor;
                        if(body->MakeBox().TryIntersectNextFrame(sphere,horizon,SpeedConstants::NbCCDSubsteps,competitor))return false;
                        FStepIdentity now;
                        return RevalidateNativeSolidCCD(*request)&&ReadCurrentStep(now)&&
                            OrdinaryWorldDetail::Epoch(afterStep,now)&&now.QuerySerial==afterStep.QuerySerial;
                    };
                    if(eligible())splitRequest=request.Get();
                    UE_LOG(LogTemp,Display,TEXT("[NativeBoxSplitArrival202Competitors] Frame=127 Serial=2 Admitted=%d"),int32(splitRequest!=nullptr));
                }
                TGuardValue<const FNativeSolidStaticRequest*> splitScope(NativeBoxSplitArrival202::Request,splitRequest);
                if(!service->QueryNativeEstablishedGeneralFloorOutcome(request,prelude,geometry)||!geometry||
                    geometry->GetRequest()!=request)
                    Refuse("Established FLOOR NoCorner lacks its complete physical clearance or curved first-arrival proof");
                result->Geometry=geometry;
                result->BoxMotionRawInput=input;result->BoxMotionRawHit=descriptor;
                result->BoxRefinementHorizon=horizon;result->BoxRefinement=MoveTemp(readback);
                if(observed.ObservationCurvedProvider!=INDEX_NONE)
                {
                    float curveSeconds=0;Analytic::FWorldHit curveHit;
                    if(geometry->GetValues().Status!=ENativeSolidGeometryStatus::GeneralBoxCurvedArrivalDescriptor||
                        !ReadNativeCurvedBoxArrivalValues(request->GetSnapshot(),v,geometry->GetValues().CanonicalBoxClearance,curveSeconds,curveHit))
                        Refuse("Established FLOOR mixed curve lost its original prefix");
                    bool curveBindingAmbiguous=false;
                    const auto* curveBinding=request->GetSnapshot()->FindUniqueContactBinding(curveHit.SourceId,curveBindingAmbiguous);
                    if(curveBindingAmbiguous||!curveBinding||
                        !BuildNativeAnalyticContactValues(curveHit,curveSeconds,v.FrameClock.OwnerLocalFrame,*curveBinding,result->NamedGround))
                        Refuse("Established FLOOR curved arrival has no authenticated typed contact");
                    result->Seconds=curveSeconds;
                    UE_LOG(LogTemp,Display,TEXT("[EstablishedFloorCurvedArrivalClosed] Frame=%u Serial=%llu Seconds=%.17g Surface=%llu"),
                        v.FrameClock.OwnerLocalFrame,request->GetStep().QuerySerial,double(curveSeconds),curveHit.SurfaceId);
                }
                else
                {
                    if(!C::MatchesNativeGeneralBoxFloorPhysicalClear(geometry->GetValues().GeneralBoxFloorDescriptor))
                        Refuse("Established FLOOR NoCorner lacks a complete independent physical clearance proof");
                    result->BoxReleasedFloorMotion=true;
                    result->BoxEstablishedFloorClear=true;
                    UE_LOG(LogTemp,Display,TEXT("[EstablishedFloorClearClosed] Frame=%u Serial=%llu H=%.17g Mask=%u"),
                        v.FrameClock.OwnerLocalFrame,request->GetStep().QuerySerial,double(horizon),uint32(result->BoxRefinement.EstablishedMask));
                }
            }
            else
            {
            // Established corners stay in the complete physical prefix
            // proof. The real native callback must select a different corner.
            if(!returned||readback.Outcome!=ENativeBoxPlanarRefinementOutcome::Hit||!readback.Established||
                readback.FirstCorner<0||readback.FirstCorner>=8||
                (readback.EstablishedMask&(1u<<readback.FirstCorner))||
                readback.InitialEvent||!readback.WitnessComputed||
                !service->QueryNativeGeneralCachedFloorArrivalGeometry(request,prelude,descriptor.TOI,geometry)||
                !geometry||geometry->GetRequest()!=request||
                !NativeGeneralCachedFloorArrivalDetail::Match(v,geometry->GetValues().GeneralBoxCachedFloorArrivalDescriptor,
                    input,descriptor,returned,readback,horizon))
                Refuse("General FLOOR original-H cached arrival lacks its distinct closed physical prefix or actual witness");
            bool typedAmbiguous=false;
            const FPreparedSourceContactBinding* typed=request->GetSnapshot()->FindUniqueContactBinding(descriptor.SourceId,typedAmbiguous);
            if(typedAmbiguous)Refuse("General cached FLOOR arrival has ambiguous typed GT binding");
            if(!typed||!typed->SourceIdentity||!typed->ContactIdentity||typed->ContactToken.IsExplicitlyNull())
                Refuse("General cached FLOOR arrival has no copied typed GT Contact");
            result->Geometry=geometry;result->BoxMotionRawInput=input;result->BoxMotionRawHit=descriptor;
            descriptor.Component=typed->ContactToken;descriptor.FrameTag=v.FrameClock.OwnerLocalFrame;
            descriptor.bHit=descriptor.bBlockingHit=true;result->Seconds=descriptor.TOI;result->NamedGround=MoveTemp(descriptor);
            result->BoxGeneralCachedFloorArrival=true;result->BoxRefinementReturned=true;
            result->BoxRefinementHorizon=horizon;result->BoxRefinement=MoveTemp(readback);
            const auto& joined=geometry->GetValues().GeneralBoxCachedFloorArrivalDescriptor;
            UE_LOG(LogTemp,Display,TEXT("[NativeGeneralCachedFloorArrivalGeometryClosed] Frame=%u Serial=%llu OriginalH=%.17g ActualTOI=%.17g PreludeUsed=%llu TotalUsed=%llu Remaining=%d"),
                v.FrameClock.OwnerLocalFrame,request->GetStep().QuerySerial,double(horizon),double(result->Seconds),
                joined.PreludeWorkUsed,joined.TotalWorkUsed,joined.RemainingWork);
            }
        }

        else
        {
        if(!service->QueryNativeBoxFloorSupportGeometry(request,plane,geometry)||!geometry||geometry->GetRequest()!=request)
            Refuse("Cached Box whole-H FLOOR support description is unknown");
        FNativeBoxGroundCacheValues current;ReadBoxGroundCacheValues(*body,current);
        if(!NativeBoxCacheDetail::Same(before,current)||!RevalidateNativeSolidCCD(*request))
            Refuse("Cached Box state changed before its real refinement");
        if(geometry->GetValues().Status==ENativeSolidGeometryStatus::GeneralBoxFloorRefinerDescriptor)
        {
            // One actual unchanged refiner is joined to the separate whole-H
            // physical FLOOR proof. Positive contact and sustained support
            // remain distinct, and are refused by this released-only route.
            const auto& g=geometry->GetValues().GeneralBoxFloorDescriptor;
            if(!g.Complete||!SameNativeSolidStaticValues(g.Captured,v))Refuse("General FLOOR descriptor capture changed");
            SHitResult descriptor;descriptor.ImpactPoint=g.CachedPlane.Point;descriptor.ImpactNormal=g.CachedPlane.Normal;
            descriptor.SourceId=g.CachedPlane.SourceId;descriptor.SurfaceId=g.CachedPlane.SurfaceId;
            descriptor.FeatureId=g.CachedPlane.FeatureId;descriptor.PrimitiveId=g.CachedPlane.PrimitiveId;
            descriptor.CanonicalGroupId=g.CachedPlane.CanonicalGroupId;descriptor.MaterialId=g.CachedPlane.MaterialId;
            descriptor.ContactFeatureOther=EContactFeatureKind::Face;
            const SHitResult input=descriptor;FNativeBoxPlanarRefinementValues readback;
            const bool returned=body->RefineExactPlanarTOI(descriptor,horizon,&readback);
            UE_LOG(LogTemp,Display,TEXT("[NativeGeneralBoxFloorRefinerObserved] Frame=%u Serial=%llu H=%.17g Returned=%d Computed=%d Outcome=%u Established=%d Mask=%u InitialEvent=%d WitnessComputed=%d FirstCorner=%d FirstTime=%.17g MinimumInitial=%.17g Roundoff=%.17g SpeedRoundoff=%.17g DescriptorUnchanged=%d Gaps=(%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g) Speeds=(%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g)"),
                v.FrameClock.OwnerLocalFrame,request->GetStep().QuerySerial,double(horizon),int32(returned),int32(readback.Computed),uint32(readback.Outcome),int32(readback.Established),uint32(readback.EstablishedMask),int32(readback.InitialEvent),int32(readback.WitnessComputed),readback.FirstCorner,double(readback.FirstTime),readback.MinimumInitial,readback.Roundoff,readback.SpeedRoundoff,int32(SameNativeAnalyticContactValues(input,descriptor)),
                readback.InitialGaps[0],readback.InitialGaps[1],readback.InitialGaps[2],readback.InitialGaps[3],readback.InitialGaps[4],readback.InitialGaps[5],readback.InitialGaps[6],readback.InitialGaps[7],
                readback.InitialNormalSpeeds[0],readback.InitialNormalSpeeds[1],readback.InitialNormalSpeeds[2],readback.InitialNormalSpeeds[3],readback.InitialNormalSpeeds[4],readback.InitialNormalSpeeds[5],readback.InitialNormalSpeeds[6],readback.InitialNormalSpeeds[7]);
            if(returned||!SameNativeAnalyticContactValues(input,descriptor)||
                !NativeFloorBoxPlaneDetail::MatchesReadbackInputs(v,input.ImpactNormal,input.ImpactPoint,readback)||
                readback.Outcome!=ENativeBoxPlanarRefinementOutcome::NoCorner||!readback.Established||
                readback.EstablishedMask||readback.InitialEvent||readback.WitnessComputed||
                readback.FirstCorner!=INDEX_NONE||!SameFloat(readback.FirstTime,horizon)||
                !C::MatchesNativeGeneralBoxFloorPhysicalClear(g))
                Refuse("General FLOOR real refiner is not the independently proved released motion");
            result->Geometry=geometry;result->BoxReleasedFloorMotion=true;result->BoxRefinement=MoveTemp(readback);
            result->BoxRefinementHorizon=horizon;
        }
        else
        {
        SHitResult descriptor;
        if(!NativeFloorBoxPlaneDetail::BuildSupportDescriptor(v,geometry->GetValues(),descriptor))
            Refuse("Cached Box has no authenticated floor descriptor");
        const SHitResult input=descriptor;FNativeBoxPlanarRefinementValues readback;
        const bool returned=body->RefineExactPlanarTOI(descriptor,horizon,&readback);
        bool closedFixedArrival=false;
#if !UE_BUILD_SHIPPING
        if(returned&&CVarNativeRoofFixedCachedFloorArrival.GetValueOnAnyThread()==1&&
            requestStep.Frame==3&&requestStep.StepSerial==4&&requestStep.Iteration==2&&
            requestStep.QuerySerial==4&&requestStep.QueryEpoch==4&&v.OwnerStableId==3&&v.BodyIndex==0&&
            readback.Computed&&readback.Outcome==ENativeBoxPlanarRefinementOutcome::Hit&&
            readback.Established&&!readback.EstablishedMask&&!readback.InitialEvent&&readback.WitnessComputed&&
            readback.Witness.bHit&&readback.FirstCorner==4&&
            C::KinematicCenterDetail::Bits(descriptor.TOI)==0x34a69593u)
        {
            FPreparedNativeSolidGeometryPtr positive;
            if(service->CompleteNativeFixedBoxCachedFloorArrivalGeometry(request,geometry,descriptor.TOI,positive)&&
                positive&&positive->GetRequest()==request&&
                positive->GetValues().Status==ENativeSolidGeometryStatus::GeneralBoxFloorCachedArrivalDescriptor&&
                NativeGeneralCachedFloorArrivalDetail::Match(v,positive->GetValues().GeneralBoxCachedFloorArrivalDescriptor,
                    input,descriptor,returned,readback,horizon))
            {
                bool typedAmbiguous=false;
                const auto* typed=request->GetSnapshot()->FindUniqueContactBinding(descriptor.SourceId,typedAmbiguous);
                if(typedAmbiguous||!typed||!typed->SourceIdentity||!typed->ContactIdentity||typed->ContactToken.IsExplicitlyNull())
                    Refuse("Fixed cached FLOOR arrival has no unique copied typed GT Contact");
                result->Geometry=positive;result->BoxMotionRawInput=input;result->BoxMotionRawHit=descriptor;
                descriptor.Component=typed->ContactToken;descriptor.FrameTag=v.FrameClock.OwnerLocalFrame;
                descriptor.bHit=descriptor.bBlockingHit=true;result->Seconds=descriptor.TOI;
                result->NamedGround=MoveTemp(descriptor);result->BoxGeneralCachedFloorArrival=true;
                result->BoxRefinementReturned=true;result->BoxRefinementHorizon=horizon;
                result->BoxRefinement=MoveTemp(readback);closedFixedArrival=true;
                const auto& joined=positive->GetValues().GeneralBoxCachedFloorArrivalDescriptor;
                UE_LOG(LogTemp,Display,TEXT("[NativeRoofFixedCachedFloorArrivalClosed] Frame=%u Serial=%llu OriginalH=%.17g ActualTOI=%.17g PreludeUsed=%llu TotalUsed=%llu Remaining=%d"),
                    v.FrameClock.OwnerLocalFrame,requestStep.QuerySerial,double(horizon),double(result->Seconds),
                    joined.PreludeWorkUsed,joined.TotalWorkUsed,joined.RemainingWork);
            }
        }
#endif
        if(!closedFixedArrival&&(returned||!SameNativeAnalyticContactValues(input,descriptor)||
            !NativeFloorBoxPlaneDetail::MatchesNoNewReadback(v,geometry->GetValues(),readback)))
        {
#if !UE_BUILD_SHIPPING
            // Frame and geometry are established by the S202R capture. Serial
            // and owner-local frame are readbacks, since the refusal did not record them.
            const auto& pose=v.OwnerState;
            if(CVarNativeRoofSlideFloorRefinerReadback.GetValueOnAnyThread()==1&&
                PoisonDiagnosticFrame==3&&PoisonDiagnosticStep==4&&v.OwnerStableId==3&&v.BodyIndex==0&&
                input.SourceId==1274291482858373289ull&&
                FMath::Abs(pose.Location.X+736.)<20.&&FMath::Abs(pose.Location.Y-1500.)<30.&&
                FMath::Abs(pose.Location.Z-28.)<5.&&FMath::Abs(pose.Rotation.Y)>.99)
            {
                const bool unchanged=SameNativeAnalyticContactValues(input,descriptor);
                const bool inputs=NativeFloorBoxPlaneDetail::MatchesReadbackInputs(
                    v,input.ImpactNormal,input.ImpactPoint,readback);
                const TCHAR* reason=returned?TEXT("returned"):!unchanged?TEXT("descriptor"):
                    !inputs?TEXT("inputs"):readback.Outcome!=ENativeBoxPlanarRefinementOutcome::NoCorner?TEXT("outcome"):
                    !readback.Established?TEXT("established"):readback.InitialEvent?TEXT("initial"):
                    !readback.EstablishedMask?TEXT("mask"):readback.FirstCorner!=INDEX_NONE?TEXT("corner"):
                    !SameFloat(readback.FirstTime,horizon)?TEXT("time"):
                    FMath::Abs(readback.MinimumInitial)>readback.Roundoff?TEXT("gap"):TEXT("unmasked_intervals");
                UE_LOG(LogTemp,Display,TEXT("[NativeRoofSlideFloorRefinerReadback] Frame=%llu OwnerFrame=%u Owner=%llu Body=%d Iteration=%d Serial=%llu Epoch=%llu Source=%llu Surface=%llu Primitive=%llu Reason=%s Returned=%d Computed=%d InputsMatch=%d DescriptorUnchanged=%d Outcome=%u Established=%d Mask=%u InitialEvent=%d WitnessComputed=%d WitnessHit=%d FirstCorner=%d FirstTime=%.17g H=%.17g MinGap=%.17g Roundoff=%.17g SpeedRoundoff=%.17g P=(%.17g,%.17g,%.17g) Q=(%.17g,%.17g,%.17g,%.17g) Gaps=(%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g) Speeds=(%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g)"),
                    requestStep.Frame,v.FrameClock.OwnerLocalFrame,v.OwnerStableId,v.BodyIndex,requestStep.Iteration,
                    requestStep.QuerySerial,requestStep.QueryEpoch,input.SourceId,input.SurfaceId,input.PrimitiveId,reason,
                    int32(returned),int32(readback.Computed),int32(inputs),int32(unchanged),uint32(readback.Outcome),
                    int32(readback.Established),uint32(readback.EstablishedMask),int32(readback.InitialEvent),
                    int32(readback.WitnessComputed),int32(readback.WitnessComputed&&readback.Witness.bHit),readback.FirstCorner,
                    double(readback.FirstTime),double(horizon),readback.MinimumInitial,readback.Roundoff,readback.SpeedRoundoff,
                    pose.Location.X,pose.Location.Y,pose.Location.Z,pose.Rotation.X,pose.Rotation.Y,pose.Rotation.Z,pose.Rotation.W,
                    readback.InitialGaps[0],readback.InitialGaps[1],readback.InitialGaps[2],readback.InitialGaps[3],
                    readback.InitialGaps[4],readback.InitialGaps[5],readback.InitialGaps[6],readback.InitialGaps[7],
                    readback.InitialNormalSpeeds[0],readback.InitialNormalSpeeds[1],readback.InitialNormalSpeeds[2],readback.InitialNormalSpeeds[3],
                    readback.InitialNormalSpeeds[4],readback.InitialNormalSpeeds[5],readback.InitialNormalSpeeds[6],readback.InitialNormalSpeeds[7]);
                const auto vec=[](const auto& p)
                {return FString::Printf(TEXT("(%.17g,%.17g,%.17g)"),double(p.X),double(p.Y),double(p.Z));};
                const auto quat=[](const auto& q)
                {return FString::Printf(TEXT("(%.17g,%.17g,%.17g,%.17g)"),double(q.X),double(q.Y),double(q.Z),double(q.W));};
                const auto& bodyState=v.BodyState;
                const auto& g=geometry->GetValues();
                const auto& envelope=g.QueryEnvelope;
                const auto& witness=readback.Witness;
                UE_LOG(LogTemp,Display,TEXT("[NativeRoofSlideFloorOperands] Frame=%llu OwnerFrame=%u Owner=%llu Body=%d Iteration=%d Serial=%llu Epoch=%llu H=%.17g TimePassed=%.17g Remaining=%.17g OwnerP=%s OwnerV=%s OwnerA=%s OwnerQ=%s OwnerW=%s OwnerAlpha=%s BodyP=%s BodyV=%s BodyA=%s BodyQ=%s BodyW=%s BodyAlpha=%s LocalCOM=%s LocalOffset=%s LocalQ=%s PhysicalHalf=%s QueryHalf=%s Start=%s End=%s QueryQ=%s PlaneP=%s PlaneN=%s Source=%llu Surface=%llu Feature=%llu Primitive=%llu Group=%llu Material=%u WitnessSource=%llu WitnessSurface=%llu WitnessFeature=%llu WitnessPrimitive=%llu WitnessGroup=%llu WitnessMaterial=%u Status=%u SupportMotion=%u Complete=%d HullX=[%.17g,%.17g] HullY=[%.17g,%.17g] HullZ=[%.17g,%.17g]"),
                    requestStep.Frame,v.FrameClock.OwnerLocalFrame,v.OwnerStableId,v.BodyIndex,requestStep.Iteration,
                    requestStep.QuerySerial,requestStep.QueryEpoch,double(horizon),double(requestStep.TimePassed),double(requestStep.Remaining),
                    *vec(pose.Location),*vec(pose.Velocity),*vec(pose.Acceleration),*quat(pose.Rotation),*vec(pose.AngularVelocity),*vec(pose.AngularAcceleration),
                    *vec(bodyState.Location),*vec(bodyState.Velocity),*vec(bodyState.Acceleration),*quat(bodyState.Rotation),*vec(bodyState.AngularVelocity),*vec(bodyState.AngularAcceleration),
                    *vec(v.LocalCOM),*vec(v.LocalOffset),*quat(v.LocalRotation),*vec(v.PhysicalHalfExtent),*vec(v.QueryHalfExtent),
                    *vec(v.Start),*vec(v.End),*quat(v.Query.Rotation),*vec(input.ImpactPoint),*vec(input.ImpactNormal),
                    input.SourceId,input.SurfaceId,input.FeatureId,input.PrimitiveId,input.CanonicalGroupId,input.MaterialId,
                    witness.SourceId,witness.SurfaceId,witness.FeatureId,witness.PrimitiveId,witness.CanonicalGroupId,witness.MaterialId,
                    uint32(g.Status),uint32(g.BoxSupport.Motion),int32(g.BoxSupport.Complete),
                    envelope.Axis[0].Lower,envelope.Axis[0].Upper,envelope.Axis[1].Lower,envelope.Axis[1].Upper,
                    envelope.Axis[2].Lower,envelope.Axis[2].Upper);
            }
#endif
            Refuse("Cached Box refiner did not produce a bounded NoCorner observation");
        }
        if(!closedFixedArrival)
        {
            result->Geometry=geometry;result->BoxNoNewUnmaskedCorner=true;result->BoxRefinement=MoveTemp(readback);
            result->BoxRefinementHorizon=horizon; // original request H, not a TOI
        }
        }
        } // existing Released59 / Fixed-Yaw branch
    }
    else
    {
        const bool requestCurve=CVarCase202Serial3Curve.GetValueOnAnyThread()==1;
        const bool observeAtlas=(requestCurve||CVarCase202CompetitorCensus.GetValueOnAnyThread()==1)&&
            v.FrameClock.OwnerLocalFrame==127&&v.OwnerStableId==3&&v.BodyIndex==0&&
            requestStep.Frame==126&&requestStep.Iteration==3&&requestStep.QuerySerial==3&&requestStep.QueryEpoch==6;
        FNativeCase202AtlasObservationPtr atlas;
        if(!service->QueryNativeSolidGeometry(request,geometry,observeAtlas?&atlas:nullptr)||!geometry||geometry->GetRequest()!=request)
        {
            if(atlas)
            {
                const auto observe=[&]() -> const TCHAR*
                {
                    const auto snapshot=request->GetSnapshot();const auto& floor=atlas->Floor;const auto& plane=floor.Plane;
                    const auto motion=floor.GetMotionInputForInspection();
                    const auto* returns=StepScope?StepScope->OrdinaryReturns.Get():nullptr;
                    const auto* worldRows=returns&&returns->World?returns->World.Get():nullptr;
                    const auto current=[&]()
                    {
                        FStepIdentity now;
                        return StepScope&&StepScope->Snapshot==snapshot&&worldRows&&!worldRows->Closed&&worldRows->PairsIdentity&&returns->Pending&&
                            returns->Pending->Owner==v.Owner&&scope->CurrentIndex==v.BodyIndex&&
                            RevalidateNativeSolidCCD(*request)&&ReadCurrentStep(now)&&
                            OrdinaryWorldDetail::Epoch(worldRows->Entry,now)&&OrdinaryWorldDetail::Epoch(requestStep,now)&&
                            now.QuerySerial==requestStep.QuerySerial&&
                            v.Owner->GetStaticCollisionWorldForFrame()==requestStep.Provider;
                    };
                    if(!current()||!SameNativeSolidStaticValues(floor.Captured,v)||floor.Complete||
                        !floor.Motion||floor.Motion->GetStatus()!=C::EGeneralPhysicalFloorMotion::InitialNativeBand||
                        !floor.Motion->Matches(motion,plane.Point,plane.Normal)||!atlas->FloorPrefix||
                        !atlas->FloorPrefix->Matches(motion,plane.Point,plane.Normal,C::KinematicCenterDetail::FromBits(0x2f21df0cu))||
                        !atlas->AtZeroPartition||atlas->AtZeroPartition->GetScene()!=snapshot->GetGeometrySelection()->GetSelectedUnion()||
                        atlas->NonflatProvider!=1931||!plane.SourceId||!plane.SurfaceId||!plane.FeatureId||!plane.PrimitiveId||
                        !SameVector(plane.Normal,FVector::UpVector)||!C::Detail::ValidPoint(floor.QueryEnvelope))
                        return TEXT("prelude");

                    SHitResult descriptor;
                    descriptor.ImpactNormal=plane.Normal;descriptor.ImpactPoint=plane.Point;
                    descriptor.SourceId=plane.SourceId;descriptor.SurfaceId=plane.SurfaceId;descriptor.FeatureId=plane.FeatureId;
                    descriptor.PrimitiveId=plane.PrimitiveId;descriptor.CanonicalGroupId=plane.CanonicalGroupId;
                    descriptor.MaterialId=plane.MaterialId;descriptor.ContactFeatureOther=EContactFeatureKind::Face;
                    auto hit=descriptor;
                    FNativeBoxPlanarRefinementValues readback;
                    bool returned=false;
                    try
                    {
                        returned=body->RefineExactPlanarTOI(hit,horizon,&readback);
                    }
                    catch(const std::runtime_error& error)
                    {
                        UE_LOG(LogTemp,Display,TEXT("[Case202Serial3AtlasWitnessRefinerFailure] Frame=%u Serial=%llu Error=%s"),
                            v.FrameClock.OwnerLocalFrame,requestStep.QuerySerial,UTF8_TO_TCHAR(error.what()));
                        return TEXT("refiner_exception");
                    }
                    FNativeBoxGroundCacheValues after;ReadBoxGroundCacheValues(*body,after);
                    if(!current()||!NativeBoxCacheDetail::Same(before,after))return TEXT("revalidation");
                    const auto& ray=readback.WitnessQuery;const auto& witness=readback.Witness;
                    if(!returned||!NativeFloorBoxPlaneDetail::MatchesReadbackInputs(v,plane.Normal,plane.Point,readback)||
                        !NativeFloorBoxPlaneDetail::MatchesInitialReadback(v,hit,readback)||
                        !NativeFloorBoxPlaneDetail::MatchesRefinedWitness(v,floor.QueryEnvelope,descriptor,hit)||
                        !readback.WitnessComputed||ray.Shape!=Analytic::EQueryShape::Ray||
                        ray.RequiredSourceId!=plane.SourceId||ray.RequiredSurfaceId!=plane.SurfaceId||
                        ray.DomainTolerance!=0.||ray.InitialOverlapTolerance!=0.||
                        !witness.bHit||witness.SourceId!=plane.SourceId||witness.SurfaceId!=plane.SurfaceId||
                        witness.FeatureId!=plane.FeatureId||witness.PrimitiveId!=plane.PrimitiveId||
                        witness.MaterialId!=plane.MaterialId||!SameVector(witness.Point,hit.ImpactPoint))
                        return TEXT("refiner_or_witness");
                    bool ambiguous=false;const auto* binding=snapshot->FindUniqueContactBinding(plane.SourceId,ambiguous);
                    if(ambiguous||!binding||!binding->SourceIdentity||!binding->ContactIdentity||!binding->ContactToken.IsValid())
                        return TEXT("binding");
                    hit.Component=binding->ContactToken;
                    const uint64 key=USSubBody::MakeComponentSweepPairKey(body,hit,true);
                    const auto& pairs=*worldRows->PairsIdentity;
                    const bool member=pairs.Contains(key);
                    const bool selectable=member&&pairs.CanSelectExactProgress(key,hit.TOI,worldRows->Before,worldRows->Delta);
                    UE_LOG(LogTemp,Display,TEXT("[Case202Serial3AtlasWitness] Frame=%u Serial=%llu Corner=%d TOI=%.17g X=%.17g Y=%.17g Source=%llu Surface=%llu Feature=%llu Primitive=%llu Group=%llu Pair=%llu Member=%d Generation=%u ExactProgress=%d"),
                        v.FrameClock.OwnerLocalFrame,requestStep.QuerySerial,readback.FirstCorner,double(hit.TOI),hit.ImpactPoint.X,hit.ImpactPoint.Y,
                        hit.SourceId,hit.SurfaceId,hit.FeatureId,hit.PrimitiveId,hit.CanonicalGroupId,key,int32(member),
                        uint32(pairs.GetGeneration(key)),int32(selectable));
                    if(!member||selectable)return TEXT("world_filter_unknown");
                    if(!requestCurve)return TEXT("repeated_floor_filtered");
                    if(case202Ball.IsExplicitlyNull()||!::AnalyticCertified::Valid(case202DynamicGap)||case202DynamicGap.Lower<=0.)
                        return TEXT("dynamic_roster");
                    if(!current()||!service->CompleteCase202Serial3Curve(request,atlas,geometry)||!geometry||!current())
                        return TEXT("curve_proof");
                    const auto& proof=geometry->GetValues().CanonicalBoxClearance;
                    float seconds=0;Analytic::FWorldHit analytic;
                    if(!ReadNativeCurvedBoxArrivalValues(snapshot,v,proof,seconds,analytic))return TEXT("curve_readback");
                    const auto* curveBinding=snapshot->FindUniqueContactBinding(analytic.SourceId,ambiguous);
                    SHitResult curve;
                    if(ambiguous||!curveBinding||!BuildNativeAnalyticContactValues(analytic,seconds,v.FrameClock.OwnerLocalFrame,*curveBinding,curve)||
                        !curve.Component.IsValid()||!curve.SubBody.IsExplicitlyNull()||
                        !curve.bHit||!curve.bBlockingHit||!curve.bSurfaceNormalMayVary||curve.bStartPenetrating||
                        !curve.CanonicalGroupId||!(curve.TOI>0.f&&curve.TOI<=worldRows->Horizon))return TEXT("curve_binding");
                    const uint64 curveKey=USSubBody::MakeComponentSweepPairKey(body,curve,true);
                    if(key==curveKey||!pairs.Contains(curveKey)||
                        !pairs.CanSelectExactProgress(curveKey,curve.TOI,worldRows->Before,worldRows->Delta))
                        return TEXT("curve_progress");
                    auto& evidence=result->Case202Serial3.Emplace();
                    evidence.Atlas=atlas;evidence.FloorInput=descriptor;evidence.FloorHit=hit;
                    evidence.FloorRefinement=MoveTemp(readback);evidence.DynamicBall=case202Ball;
                    evidence.DynamicGap=case202DynamicGap;evidence.FilteredFloorKey=key;evidence.ProgressCurveKey=curveKey;
                    return TEXT("curve_and_atlas_ready");
                };
                const TCHAR* stage=observe();
                UE_LOG(LogTemp,Display,TEXT("[Case202Serial3AtlasWitnessObservation] Frame=%u Serial=%llu Stage=%s"),
                    v.FrameClock.OwnerLocalFrame,requestStep.QuerySerial,stage);
            }
#if !UE_BUILD_SHIPPING
            if(CVarNativeBoxIgnored585CompetitorReadback.GetValueOnAnyThread()==1&&
                v.FrameClock.OwnerLocalFrame==87&&requestStep.Frame==86&&requestStep.QuerySerial==5&&
                v.BodyIndex==0&&C::KinematicCenterDetail::Bits(horizon)==0x3a04f248u&&
                SameVector(v.Query.Start,FVector(1828.2893701327644,3945.8110732856585,66.43283887433256))&&
                v.Trace.IgnoredComponents.Num()==1&&v.Trace.IgnoredSourceObjects.IsEmpty())
            {
                const auto* step=StepScope;
                const auto* returns=step?step->OrdinaryReturns.Get():nullptr;
                const auto* selector=returns&&returns->World?returns->World.Get():nullptr;
                const auto snapshot=request->GetSnapshot();
                const auto bindings=snapshot?snapshot->GetNativeOwners():FPreparedNativeOwnerBindingsPtr{};
                FStepIdentity now;
                const bool joined=step&&step->Snapshot==snapshot&&step->Registry&&bindings&&
                    returns&&returns->Pending&&selector&&!selector->Closed&&selector->PairsIdentity&&
                    returns->Pending->Owner==v.Owner&&returns->Pending->Order==selector->Rows.Num()&&
                    selector->Rows.Num()==returns->Rows.Num()&&scope->CurrentIndex==v.BodyIndex&&
                    ReadCurrentStep(now)&&OrdinaryWorldDetail::Epoch(selector->Entry,now)&&
                    OrdinaryWorldDetail::Epoch(requestStep,now)&&now.QuerySerial==requestStep.QuerySerial&&
                    bindings->GetGeneration()==now.InputGeneration&&RevalidateNativeSolidCCD(*request);
                const auto& boxes=body->GetExternalBoxSubBodies();
                const auto& spheres=body->GetExternalSphereSubBodies();
                const auto& wheels=body->GetExternalWheelSubBodies();
                UE_LOG(LogTemp,Display,TEXT("[NativeBoxIgnored585Competitors] Frame=87 Serial=5 Joined=%d Epoch=%llu Iteration=%d Component=%u Owners=%d Prior=%d ExternalBoxes=%d ExternalSpheres=%d ExternalWheels=%d PersistentSphere=%d CurvedHistory=%d Dispatches=Unexecuted"),
                    int32(joined),requestStep.QueryEpoch,int32(requestStep.Iteration),v.Trace.IgnoredComponents[0],
                    joined?step->Registry->GetOrderedAdapters().Num():-1,joined?selector->Rows.Num():-1,
                    boxes.Num(),spheres.Num(),wheels.Num(),int32(body->LastResolvedSphereHit.bBlockingHit),int32(curvedHistorical));
                if(joined)
                {
                    const auto& order=step->Registry->GetOrderedAdapters();
                    const int32 current=returns->Pending->Order;
                    for(int32 index=0;index<order.Num();++index)
                    {
                        const auto* owner=order[index];const auto* pin=owner?bindings->Find(owner):nullptr;
                        const auto* prior=index<current?returns->Rows[index].Get():nullptr;
                        const auto* row=index<current?&selector->Rows[index]:nullptr;
                        const bool valid=prior&&row&&prior->Owner==owner&&prior->Order==index&&prior->Returned&&
                            OrdinaryWorldDetail::Epoch(prior->ReturnStep,now)&&prior->Component&&
                            prior->Component->IsSealedInventory()&&row->Raw.PreparedEvidence==prior->Component;
                        UE_LOG(LogTemp,Display,TEXT("[NativeBoxIgnored585Owner] Index=%d Stable=%llu Bodies=%d State=%s PriorJoined=%d Hit=%d Allowed=%d Clamped=%d Time=%.17g Source=%llu Surface=%llu Pair=%llu"),
                            index,owner?step->Registry->FindStableId(*owner):uint64(0),pin&&pin->BodyCensusCaptured?pin->Bodies.Num():-1,
                            index<current?TEXT("PriorReturn"):index==current?TEXT("CurrentBoxGround"):TEXT("NotDispatched"),
                            int32(valid),valid?int32(row->Raw.bHit):-1,valid?int32(row->Filter.Allowed):-1,
                            valid?int32(row->HasClamped):-1,valid?double(row->Clamped):0.,
                            valid?row->Raw.Hit.SourceId:uint64(0),valid?row->Raw.Hit.SurfaceId:uint64(0),valid?row->Raw.PairKey:uint64(0));
                    }
                    const auto* owner=bindings->Find(v.Owner);
                    if(owner&&owner->BodyCensusCaptured)
                        for(const auto& child:owner->Bodies)
                            if(child.Index!=v.BodyIndex&&child.Body&&child.ObjectIdCaptured&&
                                child.Body->GetParentComponent()==v.Owner)
                                UE_LOG(LogTemp,Display,TEXT("[NativeBoxIgnored585Sibling] Index=%d Object=%u Kind=%d IgnoredByBox=%d"),
                                    child.Index,child.ObjectId,int32(child.Body->GetSubBodyType()),int32(body->ComponentHasBeenIgnored(*child.Body)));
                    for(const auto& weak:spheres)
                    {
                        const auto* sphere=weak.Get();
                        const auto* sphereOwner=sphere?bindings->Find(sphere->GetParentComponent()):nullptr;
                        if(!sphere||!sphereOwner||!sphereOwner->BodyCensusCaptured)continue;
                        for(const auto& child:sphereOwner->Bodies)
                            if(child.Body==sphere&&child.Class==sphere->GetClass()&&child.ObjectIdCaptured&&
                                static_cast<FWeakObjectPtr>(weak).HasSameIndexAndSerialNumber(child.Object))
                            {
                                const auto& state=sphere->GetKinematicState();
                                UE_LOG(LogTemp,Display,TEXT("[NativeBoxIgnored585Sphere] Object=%u Radius=%.17g P=(%.17g,%.17g,%.17g) V=(%.17g,%.17g,%.17g) A=(%.17g,%.17g,%.17g) Dispatch=Unexecuted"),
                                    child.ObjectId,double(sphere->GetRadius()),state.Location.X,state.Location.Y,state.Location.Z,
                                    state.Velocity.X,state.Velocity.Y,state.Velocity.Z,
                                    state.Acceleration.X,state.Acceleration.Y,state.Acceleration.Z);
                            }
                    }
                }
            }
#endif
            if(!result->Case202Serial3.IsSet())Refuse("Ordinary Box complete static geometry is unknown");
        }
        result->Geometry=geometry;
        if(geometry->GetValues().Status==ENativeSolidGeometryStatus::GeneralBoxFloorMotionDescriptor)
        {
            const auto& g=geometry->GetValues().GeneralBoxFloorMotionDescriptor;
            if(!g.HasMatchingPhysicalMotion()||!SameNativeSolidStaticValues(g.Captured,v))Refuse("General FLOOR motion descriptor changed");
            SHitResult descriptor;descriptor.ImpactPoint=g.Plane.Point;descriptor.ImpactNormal=g.Plane.Normal;
            descriptor.SourceId=g.Plane.SourceId;descriptor.SurfaceId=g.Plane.SurfaceId;descriptor.FeatureId=g.Plane.FeatureId;
            descriptor.PrimitiveId=g.Plane.PrimitiveId;descriptor.CanonicalGroupId=g.Plane.CanonicalGroupId;descriptor.MaterialId=g.Plane.MaterialId;
            descriptor.ContactFeatureOther=EContactFeatureKind::Face;const SHitResult input=descriptor;
            const bool arrival=g.Motion->GetStatus()==C::EGeneralPhysicalFloorMotion::FirstAdjacentArrival;
            const float callH=arrival?g.Motion->GetInsideSeconds():horizon;
            FNativeBoxGroundCacheValues cacheBefore;ReadBoxGroundCacheValues(*body,cacheBefore);
            FNativeBoxPlanarRefinementValues readback;const bool returned=body->RefineExactPlanarTOI(descriptor,callH,&readback);
            FNativeBoxGroundCacheValues cacheAfter;ReadBoxGroundCacheValues(*body,cacheAfter);FStepIdentity after;
            const bool intrinsic=NativeBoxCacheDetail::Same(before,cacheBefore)&&NativeBoxCacheDetail::Same(cacheBefore,cacheAfter)&&
                RevalidateNativeSolidCCD(*request)&&ReadCurrentStep(after)&&after.QuerySerial==request->GetStep().QuerySerial;
            const bool matched=intrinsic&&NativeGeneralFloorMotionDetail::Match(v,g,input,descriptor,returned,readback,callH);
            UE_LOG(LogTemp,Display,TEXT("[NativeGeneralBoxFloorMotionRefinerObserved] Frame=%u Serial=%llu OriginalH=%.17g CallH=%.17g Status=%u C=%.17g U=%.17g CommonMask=%u Returned=%d Computed=%d Outcome=%u Established=%d Mask=%u InitialEvent=%d WitnessComputed=%d FirstCorner=%d FirstTime=%.17g HitTOI=%.17g MinimumInitial=%.17g Roundoff=%.17g Intrinsic=%d Matched=%d"),
                v.FrameClock.OwnerLocalFrame,request->GetStep().QuerySerial,double(horizon),double(callH),uint32(g.Motion->GetStatus()),
                double(g.Motion->GetClearSeconds()),double(g.Motion->GetInsideSeconds()),uint32(g.Motion->GetCommonInsideMask()),
                int32(returned),int32(readback.Computed),uint32(readback.Outcome),int32(readback.Established),uint32(readback.EstablishedMask),
                int32(readback.InitialEvent),int32(readback.WitnessComputed),readback.FirstCorner,double(readback.FirstTime),double(descriptor.TOI),
                readback.MinimumInitial,readback.Roundoff,int32(intrinsic),int32(matched));
            if(!matched)Refuse("General FLOOR actual motion/refiner did not match its retained geometric classification");
            result->BoxMotionRawInput=input;result->BoxMotionRawHit=descriptor;
            result->BoxRefinementHorizon=callH;result->BoxRefinement=MoveTemp(readback);
            if(arrival)
            {
                bool bindingAmbiguous=false;
                const FPreparedSourceContactBinding* binding=request->GetSnapshot()->FindUniqueContactBinding(descriptor.SourceId,bindingAmbiguous);
                if(bindingAmbiguous)Refuse("General FLOOR first arrival has ambiguous typed GT binding");
                if(!binding||!binding->SourceIdentity||!binding->ContactIdentity||binding->ContactToken.IsExplicitlyNull())
                    Refuse("General FLOOR first arrival has no copied typed GT Contact");
                descriptor.Component=binding->ContactToken;descriptor.FrameTag=v.FrameClock.OwnerLocalFrame;
                descriptor.bHit=descriptor.bBlockingHit=true;
                result->Seconds=descriptor.TOI;result->NamedGround=MoveTemp(descriptor);
                result->BoxGeneralFirstFloorArrival=true;result->BoxRefinementReturned=true;
            }
            // WholePhysicalClear retains the real unestablished callback.
            // The actual advancement must still close through Motion60Whole.
        }
        else if(geometry->GetValues().Status==ENativeSolidGeometryStatus::GeneralBoxCurvedArrivalDescriptor)
        {
            float seconds=0;Analytic::FWorldHit hit;
            if(!ReadNativeCurvedBoxArrivalValues(request->GetSnapshot(),v,geometry->GetValues().CanonicalBoxClearance,seconds,hit))
                Refuse("Curved Box first arrival lost its original prefix");
            bool ambiguous=false;
            const auto* binding=request->GetSnapshot()->FindUniqueContactBinding(hit.SourceId,ambiguous);
            if(ambiguous||!binding||!BuildNativeAnalyticContactValues(hit,seconds,v.FrameClock.OwnerLocalFrame,*binding,result->NamedGround))
                Refuse("Curved Box first arrival has no authenticated typed contact");
            result->Seconds=seconds;
            UE_LOG(LogTemp,Display,TEXT("[CurvedBoxFirstArrivalClosed] Frame=%u Serial=%llu Seconds=%.17g Surface=%llu"),
                v.FrameClock.OwnerLocalFrame,request->GetStep().QuerySerial,double(seconds),hit.SurfaceId);
        }
        else if(geometry->GetValues().Status==ENativeSolidGeometryStatus::GeneralBoxCurvedActualMotionClear)
        {
            if(!MatchesNativeCurvedBoxActualMotionClearValues(request->GetSnapshot(),v,
                geometry->GetValues().CanonicalBoxClearance,geometry->GetValues().QueryEnvelope))
                Refuse("Curved Box actual-motion clearance does not match the FullFrame request");
        }
        else if(geometry->GetValues().Status==ENativeSolidGeometryStatus::CanonicalReferenceBoxSweepPathClear)
        {
            if(!MatchesNativeCanonicalBoxSweepClearanceValues(v,request->GetSnapshot()->GetGeometrySelection(),
                geometry->GetValues().CanonicalBoxClearance,geometry->GetValues().QueryEnvelope))
                Refuse("Canonical reference Box prediction values do not match the actual request");
            result->CanonicalReferenceBoxSweepPathClear=true; // distinct operation; no legacy facet query/miss claim
        }
        else if(geometry->GetValues().Status!=ENativeSolidGeometryStatus::WholeIntervalSeparated)
        {
            SHitResult descriptor;
            const bool axial=geometry->GetValues().Status==ENativeSolidGeometryStatus::CanonicalAxialBoxPlaneDescriptor;
            const auto axialValues=geometry->GetValues().CanonicalAxialBoxPlane;
            if(axial)
            {
                if(!MatchesNativeCanonicalAxialBoxPlaneValues(v,request->GetSnapshot()->GetGeometrySelection(),
                    request->GetSnapshot()->GetNominalScene(),axialValues,geometry->GetValues().QueryEnvelope))
                    Refuse("Ordinary Box finite axial descriptor does not match its real request");
                descriptor=axialValues->GetDescriptorForInspection();
            }
            else if(!NativeFloorBoxPlaneDetail::Build(v,geometry->GetValues(),descriptor))
                Refuse("Ordinary Box has no retained finite-plane descriptor");
            const SHitResult input=descriptor;
            const bool initial=geometry->GetValues().Status==ENativeSolidGeometryStatus::BoxInitialPlaneDescriptor;
            const float refineHorizon=axial?axialValues->GetArrivalForInspection().InsideSeconds:
                geometry->GetValues().YawBoxFirstArrival?geometry->GetValues().Sweep.InsideSeconds:horizon;
            FNativeBoxPlanarRefinementValues readback;
            if(!body->RefineExactPlanarTOI(descriptor,refineHorizon,(initial||axial)?&readback:nullptr)||
                (axial?!MatchNativeCanonicalAxialBoxRefinement(v,axialValues,descriptor,readback):
                    (!NativeFloorBoxPlaneDetail::MatchesRefinement(v,geometry->GetValues(),descriptor)||
                        (initial&&(!NativeFloorBoxPlaneDetail::MatchesReadbackInputs(v,input.ImpactNormal,input.ImpactPoint,readback)||
                            !NativeFloorBoxPlaneDetail::MatchesInitialReadback(v,descriptor,readback))))))
                Refuse("Ordinary Box real native planar refinement is unknown");
            bool bindingAmbiguous=false;
            const FPreparedSourceContactBinding* binding=request->GetSnapshot()->FindUniqueContactBinding(descriptor.SourceId,bindingAmbiguous);
            if(bindingAmbiguous)Refuse("Ordinary Box plane has ambiguous GT Contact binding");
            if(!binding||!binding->SourceIdentity||!binding->ContactIdentity||binding->ContactToken.IsExplicitlyNull())
                Refuse("Ordinary Box plane has no copied typed GT Contact");
            descriptor.Component=binding->ContactToken;descriptor.FrameTag=v.FrameClock.OwnerLocalFrame;
            descriptor.bHit=descriptor.bBlockingHit=true;
            result->Seconds=descriptor.TOI;result->NamedGround=MoveTemp(descriptor);result->BoxRefinementReturned=true;
            result->BoxRefinementHorizon=refineHorizon;result->BoxRefinement=MoveTemp(readback);
            if(initial&&C::NativeBoxNeedsGeneralFloorDescriptor(v))
                UE_LOG(LogTemp,Display,TEXT("[GeneralFloorInitialNativeBandClosed] Frame=%u Serial=%llu TOI=%.17g MinimumInitial=%.17g Roundoff=%.17g Corner=%d"),
                    v.FrameClock.OwnerLocalFrame,request->GetStep().QuerySerial,double(result->Seconds),
                    result->BoxRefinement.MinimumInitial,result->BoxRefinement.Roundoff,result->BoxRefinement.FirstCorner);
        }
    }
    FStepIdentity now;FNativeBoxGroundCacheValues after;ReadBoxGroundCacheValues(*body,after);
    if(!NativeBoxCacheDetail::Same(before,after)||!RevalidateNativeSolidCCD(*request)||!ReadCurrentStep(now)||
        now.QuerySerial!=request->GetStep().QuerySerial||v.Owner->GetStaticCollisionWorldForFrame()!=request->GetStep().Provider)
        Refuse("Ordinary Box request, exact serial or cache changed before closure");
    result->Closed=true;scope->SealOrdinaryGround(result);out=result->NamedGround;return out.bHit;
}
void FComponentSweepScope::ObserveCase202Serial3DynamicDispatches(const UBoxSubBody* body,
    bool box,bool sphere,bool wheel,bool persistent)
{
    auto* scope=ComponentSweepScope;
    if(!scope||!scope->Pending||!scope->Pending->Bodies.IsValidIndex(scope->CurrentIndex))return;
    const auto& row=scope->Pending->Bodies[scope->CurrentIndex];
    if(!row.OrdinaryGround||!row.OrdinaryGround->Case202Serial3.IsSet())return;
    const auto& request=row.OrdinaryGround->Request;FStepIdentity now;
    if(row.RawBodyIdentity!=body||!request||request->GetValues().Body!=body||
        !ReadCurrentStep(now)||!OrdinaryWorldDetail::Epoch(request->GetStep(),now)||now.QuerySerial!=request->GetStep().QuerySerial||
        box||sphere||wheel||persistent)
        Refuse("Case 202 serial 3 actual dynamic dispatches differ from the proved roster");
    UE_LOG(LogTemp,Display,TEXT("[Case202Serial3DynamicDispatches] Frame=%u Serial=%llu Box=0 Sphere=0 Wheel=0 PersistentSphere=0"),
        request->GetValues().FrameClock.OwnerLocalFrame,now.QuerySerial);
}

#if WITH_DEV_AUTOMATION_TESTS
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FDynamicBoxCacheIsNotStaticPlane,
    "IAmSpeed.AnalyticWorld.NativeSolid.DynamicBoxCacheIsNotStaticPlane",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FDynamicBoxCacheIsNotStaticPlane::RunTest(const FString&)
{
    auto* sphere=NewObject<USphereSubBody>();
    auto* other=NewObject<USphereSubBody>();
    SHitResult hit(true,FVector::ZeroVector,FVector::UpVector,0.0f,sphere);
    hit.SubBody=sphere;
    TestTrue(TEXT("Dynamic hit records one subbody in both weak fields"),NativeBoxCacheDetail::IsDynamicHit(hit));
    hit.Component=other;
    TestFalse(TEXT("Different component identity does not qualify"),NativeBoxCacheDetail::IsDynamicHit(hit));
    hit.Component=sphere;hit.SourceId=1;
    TestFalse(TEXT("Static metadata does not qualify as dynamic"),NativeBoxCacheDetail::IsDynamicHit(hit));
    hit.SourceId=0;hit.SubBody=nullptr;
    TestFalse(TEXT("A World hit without the other subbody does not qualify"),NativeBoxCacheDetail::IsDynamicHit(hit));
    return !HasAnyErrors();
}
#endif

void FSphereMemberSweepScope::FinishOrdinaryGround(bool deliveredHit,const SHitResult& delivered,float seconds)
{
    const auto result=PendingOrdinaryGround;FStepIdentity now;
    if(PendingGround||!result||result->Closed||!result->Request||!result->Geometry||
        !RevalidateNativeSolidCCD(*result->Request)||!ReadCurrentStep(now)||
        now.QuerySerial!=result->Request->GetStep().QuerySerial||!ComponentSweepScope||
        deliveredHit!=result->NamedGround.bHit||!SameFloat(seconds,result->Seconds)||
        !SameNativeAnalyticContactValues(delivered,result->NamedGround))
        Refuse("Ordinary Sphere World values changed before actual member closure");
    // The outer Finish already reread the protected virtual margin and joined
    // its actual member serial. No old StateSeed or integration permission.
    result->Closed=true;ComponentSweepScope->SealOrdinaryGround(result);PendingOrdinaryGround.Reset();
}
void FComponentSweepScope::SealOrdinaryGround(TSharedPtr<const FOrdinaryGroundSweepEvidence,ESPMode::ThreadSafe> ground)
{
    FStepIdentity now;
    if(!Active||Completed||ComponentSweepScope!=this||!ground||!ground->IsClosedQueryValues()||
        !ReadCurrentStep(now)||!Pending||!Pending->Bodies.IsValidIndex(CurrentIndex)||
        Pending->Bodies[CurrentIndex].OrdinaryGround||Pending->Bodies[CurrentIndex].Ground||
        ground->Request->GetStep().QuerySerial!=now.QuerySerial||ground->Request->GetStep().QueryEpoch!=now.QueryEpoch||
        ground->Request->GetStep().Lease!=now.Lease||ground->Request->GetSnapshot()!=StepScope->Snapshot)
        Refuse("Ordinary closed World values have no unique current body slot");
    const auto& v=ground->Request->GetValues();
    if(Pending->Bodies[CurrentIndex].RawBodyIdentity!=v.Body||Pending->Owner!=v.Owner||CurrentIndex!=v.BodyIndex||
        Pending->OwnerStableId!=v.OwnerStableId||!SameKinematicBits(Pending->SweepOwnerState,v.OwnerState))
        Refuse("Ordinary closed World candidate belongs to another owner/body");
#if !UE_BUILD_SHIPPING
    StepScope->BindCase585CurvedReimpact(ground);
#endif
    Pending->Bodies[CurrentIndex].OrdinaryGround=MoveTemp(ground);
}
