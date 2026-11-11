#include "GroupedBoxVolumeBounds.h"
#include "NativeGroupedBoxVolumeBounds.h"
#include "ReferenceBoundsStrictSeparation.h"

namespace GroupedBoxTestDetail
{
using namespace Speed::Analytic;
using namespace Speed::Analytic::Certified;
inline FBoxVolumeGroup Box(FVector3d center,FVector3d extent,FQuat4d rotation=FQuat4d::Identity)
{
    FWorldQuery query;query.Shape=EQueryShape::Box;query.Start=query.End=center;
    query.HalfExtent=extent;query.Rotation=rotation;
    FBoxVolumeGroup group;NativeGroupedBoxDetail::FixedQueryGroup(query,group);return group;
}
inline bool Volume(TConstArrayView<FBoxVolumeGroup> groups,FQuat4d rotation,FGroupedBoxVolumeBounds& result)
{
    FIntervalPoint hull;
    for(const auto& group:groups)for(const auto& point:group.Corners)
        hull=Detail::ValidPoint(hull)?TimeDetail::Hull(hull,point):point;
    TArray<FVector3d,TInlineAllocator<8>> axes;
    for(auto axis:{FVector3d::ForwardVector,FVector3d::RightVector,FVector3d::UpVector})axes.Add(axis);
    for(auto axis:{FVector3d::ForwardVector,FVector3d::RightVector,FVector3d::UpVector})axes.Add(rotation.RotateVector(axis));
    FDistanceBudget budget{4096};return CloseGroupedBoxVolumeBounds(hull,groups,axes,budget,result);
}
inline FIntervalTensorSurface PlaneX()
{
    FIntervalTensorSurface plane;plane.DegreeU=plane.DegreeV=1;
    for(double y:{-5.,5.})for(double z:{-5.,5.})plane.ControlPoints.Add(Detail::Point(FVector3d(.25,y,z)));
    return plane;
}
inline FIndexedIntervalUnionPtr Scene(const FIntervalTensorSurface& surface)
{
    FDistanceBudget budget{8192};FIndexedIntervalUnionPtr scene;
    FIndexedIntervalUnion::Build(MakeArrayView(&surface,1),budget,scene);return scene;
}
inline bool Prove(const FIndexedIntervalUnionPtr& scene,const FGroupedBoxVolumeBounds& volume,
    FDistanceBudget& budget,FReferenceBoxVolumeStrictSeparationPtr& proof)
{
    proof.Reset();
    FReferenceBoundsCandidateSetPtr partition;
    return FReferenceBoundsCandidateSet::Select(scene,volume.BroadHull,budget,partition)&&
        FReferenceBoxVolumeStrictSeparation::TryProve(partition,volume,budget,proof);
}
inline bool Contains(const FIntervalPoint& range,const FVector3d& point)
{
    if(!Detail::ValidPoint(range))return false;
    for(int32 d=0;d<3;++d)if(!FMath::IsFinite(point[d])||point[d]<range.Axis[d].Lower||point[d]>range.Axis[d].Upper)return false;
    return true;
}
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAnalyticGroupedBoxVolumeTest,
    "IAmSpeed.AnalyticWorld.GeneralBox.GroupedVolumeSeparation",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FAnalyticGroupedBoxVolumeTest::RunTest(const FString&)
{
    using namespace Speed::Analytic::Certified;using namespace GroupedBoxTestDetail;
    FTensorBoundsTestContext context;if(!TestTrue(TEXT("complete RNE fixture"),context.Ready))return false;
    const uint32 original=VectorGetControlRegister();
    for(uint32 flush:{0u,64u,32768u,32832u})
    {
        VectorSetControlRegister((original&~uint32(0x8040))|flush);
        const FQuat4d rotation(FVector3d::UpVector,UE_DOUBLE_PI/4.);
        TArray<FBoxVolumeGroup> groups;groups.Add(Box(FVector3d::ZeroVector,FVector3d(1),rotation));
        FGroupedBoxVolumeBounds volume;
        if(!TestTrue(TEXT("whole oriented Box group retained"),Volume(groups,rotation,volume)))return false;
        FIntervalTensorSurface point;point.ControlPoints.Add(Detail::Point(FVector3d(1.3,1.3,0)));
        const auto scene=Scene(point);FDistanceBudget budget{1024};FReferenceBoxVolumeStrictSeparationPtr proof;
        if(!TestTrue(TEXT("oriented volume separated from point inside its broad AABB"),Prove(scene,volume,budget,proof)&&proof&&proof->Matches(scene,volume)))return false;
        TestTrue(TEXT("broad Hull actually contains the point"),GroupedBoxDetail::Within(volume.BroadHull,point.ControlPoints[0]));
        FDistanceBudget oldBudget{128};FReferenceBoundsCandidateSetPtr partition;FReferenceBoundsStrictSeparationPtr old;
        TestTrue(TEXT("old AABB partition exists"),FReferenceBoundsCandidateSet::Select(scene,volume.BroadHull,oldBudget,partition));
        TestFalse(TEXT("old complete AABB theorem remains false"),FReferenceBoundsStrictSeparation::TryProve(partition,oldBudget,old));
        TestTrue(TEXT("complete actual endpoint fits one region"),proof->ContainsCompleteEndpointGroup(groups[0].Corners));
        const auto escaped=Box(FVector3d(1.3,1.3,0),FVector3d(.01));
        TestFalse(TEXT("endpoint in broad Hull but outside an oriented slab refused"),proof->ContainsCompleteEndpointGroup(escaped.Corners));
        auto wrong=volume;wrong.Slabs[0].Upper+=.25;
        TestFalse(TEXT("changed projection cannot match"),proof->Matches(scene,wrong));
        FDistanceBudget forgedBudget{1024};FReferenceBoxVolumeStrictSeparationPtr forged;
        TestFalse(TEXT("caller cannot forge projection by altering stored slabs"),Prove(scene,wrong,forgedBudget,forged));
        FDistanceBudget directBadBudget{1024};auto stale=proof;
        TestFalse(TEXT("direct producer refuses changed slab"),FReferenceBoxVolumeStrictSeparation::TryProve(partition,wrong,directBadBudget,stale));
        TestFalse(TEXT("direct failed producer clears an existing proof"),stale.IsValid());
        TestFalse(TEXT("different immutable scene cannot match"),proof->Matches(Scene(point),volume));
        auto crossingScene=Scene(PlaneX());FDistanceBudget crossingBudget{512};FReferenceBoxVolumeStrictSeparationPtr crossing;
        for(const auto& corner:groups[0].Corners)
            TestTrue(TEXT("each corner enclosure is strictly off the interior plane"),corner.Axis[0].Upper<.25||corner.Axis[0].Lower>.25);
        TestFalse(TEXT("surface through Box interior refused although all8 corners miss it"),Prove(crossingScene,volume,crossingBudget,crossing));
        auto multi=groups;multi.Add(Box(FVector3d(1.3,1.3,0),FVector3d(.1)));
        FGroupedBoxVolumeBounds multiVolume;TestTrue(TEXT("second complete group retained"),Volume(multi,rotation,multiVolume));
        FDistanceBudget multiBudget{512};FReferenceBoxVolumeStrictSeparationPtr multiProof;
        TestFalse(TEXT("one unresolved group prevents whole-volume proof"),Prove(scene,multiVolume,multiBudget,multiProof));
        TArray<FBoxVolumeGroup> apart;
        apart.Add(Box(FVector3d(-3,0,0),FVector3d(1)));apart.Add(Box(FVector3d(3,0,0),FVector3d(1)));
        FGroupedBoxVolumeBounds apartVolume;TestTrue(TEXT("separate full regions"),Volume(apart,FQuat4d::Identity,apartVolume));
        FDistanceBudget apartBudget{2048};FReferenceBoxVolumeStrictSeparationPtr apartProof;
        if(!TestTrue(TEXT("both full regions clear of central plane"),Prove(crossingScene,apartVolume,apartBudget,apartProof)))return false;
        const auto bridge=Box(FVector3d::ZeroVector,FVector3d(3,.5,.5));
        TestFalse(TEXT("different groups per endpoint corner cannot admit bridging interior"),apartProof->ContainsCompleteEndpointGroup(bridge.Corners));
        auto swept=apart[0];for(auto p:apart[1].Corners)swept.Corners.Add(p);
        TArray<FBoxVolumeGroup> sweepGroups;sweepGroups.Add(swept);FGroupedBoxVolumeBounds sweepVolume;
        TestTrue(TEXT("one full swept convex group contains both endpoint boxes"),Volume(sweepGroups,FQuat4d::Identity,sweepVolume));
        FDistanceBudget sweepBudget{512};FReferenceBoxVolumeStrictSeparationPtr sweepProof;
        TestFalse(TEXT("intermediate collision refused when both endpoint boxes are clear"),Prove(crossingScene,sweepVolume,sweepBudget,sweepProof));
        TArray<FBoxVolumeGroup,TInlineAllocator<16>> skinGroups;
        skinGroups.Add(Box(FVector3d::ZeroVector,FVector3d(1)));
        FIntervalTensorSurface skinPoint;skinPoint.ControlPoints.Add(Detail::Point(FVector3d(0,0,1.05)));
        const auto skinScene=Scene(skinPoint);FGroupedBoxVolumeBounds noSkin,skin;
        TestTrue(TEXT("unpadded whole Box"),Volume(skinGroups,FQuat4d::Identity,noSkin));
        FDistanceBudget noSkinBudget{1024};FReferenceBoxVolumeStrictSeparationPtr noSkinProof;
        TestTrue(TEXT("point outside unpadded Box"),Prove(skinScene,noSkin,noSkinBudget,noSkinProof));
        TestTrue(TEXT("original XYZ shell arithmetic applied before projections"),NativeGroupedBoxDetail::Inflate(skinGroups,.1,0));
        TestTrue(TEXT("inflated full volume retained"),Volume(skinGroups,FQuat4d::Identity,skin));
        FDistanceBudget skinBudget{512};FReferenceBoxVolumeStrictSeparationPtr skinProof;
        TestFalse(TEXT("shell-only intersection cannot be erased"),Prove(skinScene,skin,skinBudget,skinProof));
        FDistanceBudget zero{0};FReferenceBoxVolumeStrictSeparationPtr absent=proof;
        TestFalse(TEXT("no allowance cannot build a geometry proof"),Prove(scene,volume,zero,absent));
        TestTrue(TEXT("failed attempt does not refund any work"),zero.Used==0&&zero.Remaining==0);
        FDistanceBudget limited{400,11,399};FReferenceBoxVolumeStrictSeparationPtr limitedProof;
        TestFalse(TEXT("consumption floor honored throughout prover"),Prove(scene,volume,limited,limitedProof));
        TestTrue(TEXT("floor and prior debit retained"),limited.Used==12&&limited.Remaining==399&&limited.ConsumptionFloor==399);
        FDistanceBudget lateFloor{100,19,76};auto late=proof;
        TestFalse(TEXT("floor reached after revalidation/copy during actual cell projection"),
            FReferenceBoxVolumeStrictSeparation::TryProve(partition,volume,lateFloor,late));
        TestTrue(TEXT("mid-prover denial resets output and preserves all consumed work/floor"),
            !late&&lateFloor.Used==43&&lateFloor.Remaining==76&&lateFloor.ConsumptionFloor==76);
    }
    return TestTrue(TEXT("all numeric controls restored"),context.Restore.Restore())&&!HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAnalyticGroupedNativeBoxModelTest,
    "IAmSpeed.AnalyticWorld.GeneralBox.GroupedNativeModelOperands",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FAnalyticGroupedNativeBoxModelTest::RunTest(const FString&)
{
    using namespace Speed;using namespace Speed::Analytic;using namespace Speed::Analytic::Certified;
    FTensorBoundsTestContext context;if(!TestTrue(TEXT("complete RNE fixture"),context.Ready))return false;
    const uint32 original=VectorGetControlRegister();
    for(uint32 flush:{0u,64u,32768u,32832u})
    {
        VectorSetControlRegister((original&~uint32(0x8040))|flush);
        PreparedExecution::FNativeSolidStaticValues v;v.Shape=PreparedExecution::ENativeSolidQueryShape::Box;
        v.OwnerState.Location=FVector(13,-9,70);v.OwnerState.Rotation=FQuat(FVector::UpVector,.7);
        v.OwnerState.Velocity=FVector(12,-3,2);v.OwnerState.Acceleration=FVector(-7,4,-650);
        v.OwnerState.AngularVelocity=FVector(.4,-.25,.7);v.OwnerState.AngularAcceleration=FVector(.05,.1,-.2);
        v.LocalCOM=FVector(2,-3,1);v.LocalOffset=FVector(7,1,-2);v.LocalRotation=FQuat(FVector::RightVector,-.55);
        v.PhysicalHalfExtent=FVector(65.66,43.5852,15.9472);
        v.QueryHalfExtent=FVector(float(65.66),float(43.5852),float(15.9472));v.Horizon=1.f/300.f;
        v.BodyState.Location=(FVector(v.OwnerState.Location)-v.OwnerState.Rotation.RotateVector(v.LocalCOM))+
            v.OwnerState.Rotation.RotateVector(v.LocalOffset);
        v.BodyState.Rotation=v.OwnerState.Rotation*v.LocalRotation;
        v.BodyState.Velocity=FVector(15,-4,1);v.BodyState.Acceleration=FVector(-2,3,-640);
        v.Query.Shape=EQueryShape::Box;v.Query.Rotation=v.BodyState.Rotation;v.Query.HalfExtent=v.QueryHalfExtent;
        v.Query.Start=v.BodyState.Location;
        v.Query.End=v.BodyState.Location+v.BodyState.Velocity*v.Horizon+0.5f*v.BodyState.Acceleration*v.Horizon*v.Horizon;
        v.Query.DomainTolerance=.000001;v.Query.InitialOverlapTolerance=.05;
        FDistanceBudget budget{8192};FGroupedBoxVolumeBounds volume;
        if(!TestTrue(TEXT("all native/reference/query operands retained together"),BuildGeneralNativeBoxGroupedVolume(v,budget,volume)))return false;
        if(!TestTrue(TEXT("two original-time cells retain14 complete groups and6 axes"),volume.Groups.Num()==14&&volume.Axes.Num()==6&&volume.Slabs.Num()==84))return false;
        for(const auto& group:volume.Groups)if(!TestTrue(TEXT("no individual-corner clearance substituted"),group.Corners.Num()==8))return false;
        for(float seconds:{0.f,v.Horizon*.25f,v.Horizon*.5f,v.Horizon*.75f,v.Horizon})
        {
            const auto actual=v.OwnerState.Integrate(seconds);
            const FVector center=(actual.Location-actual.Rotation.RotateVector(v.LocalCOM))+actual.Rotation.RotateVector(v.LocalOffset);
            const FQuat bodyRotation=actual.Rotation*v.LocalRotation;
            for(int32 kind=0;kind<2;++kind)for(int32 k=0;k<8;++k)
            {
                const auto extent=kind?v.QueryHalfExtent:v.PhysicalHalfExtent;FVector local;
                for(int32 d=0;d<3;++d)local[d]=(k&(1<<d))?extent[d]:-extent[d];
                TestTrue(TEXT("real native corner remains in its OWN complete motion group"),
                    GroupedBoxTestDetail::Contains(volume.Groups[(seconds<=v.Horizon*.5f?0:4)+kind].Corners[k],center+bodyRotation.RotateVector(local)));
            }
        }
        FGeneralBoxMotionInput originalMotion;originalMotion.Owner=v.OwnerState;originalMotion.Horizon=v.Horizon;
        originalMotion.LocalCOM=v.LocalCOM;originalMotion.LocalOffset=v.LocalOffset;originalMotion.LocalRotation=v.LocalRotation;
        originalMotion.PhysicalHalfExtent=v.PhysicalHalfExtent;originalMotion.QueryHalfExtent=v.QueryHalfExtent;
        for(int32 cell=0;cell<2;++cell)
        {
            FGeneralBoxMotionBounds part;const float middle=v.Horizon*.5f;FDistanceBudget partBudget{8192};
            if(!TestTrue(TEXT("original per-corner contracted family is available in each closed half"),
                NativeGroupedBoxDetail::ContractedGlobalMotionPiece(originalMotion,cell?middle:0.f,cell?v.Horizon:middle,partBudget,part)&&part.Corners.Num()==32))return false;
            for(int32 g=0;g<4;++g)for(int32 k=0;k<8;++k)
            {
                auto expected=part.Corners[g*8+k];
                for(double shell:{v.Query.DomainTolerance,v.Query.InitialOverlapTolerance})for(int32 d=0;d<3;++d)
                    expected.Axis[d]=AnalyticCertified::Add(expected.Axis[d],Interval{-shell,shell,true});
                TestTrue(TEXT("each global cell retains all four complete roles in their exact order"),
                    GroupedBoxDetail::SamePoint(expected,volume.Groups[cell*4+g].Corners[k]));
            }
        }
        FDistanceBudget primitiveBudget{8192};FGroupedBoxVolumeBounds primitive;
        TestTrue(TEXT("primitive retains both global cells as8completegroups"),
            BuildGeneralBoxPrimitiveGroupedVolume(originalMotion,v.Query.Rotation,v.Query.DomainTolerance,v.Query.InitialOverlapTolerance,primitiveBudget,primitive)&&
            primitive.Groups.Num()==8&&primitive.Slabs.Num()==48);
        auto bad=v;bad.Query.End.X+=1.;FGroupedBoxVolumeBounds rejected=volume;FDistanceBudget badBudget{8192};
        TestFalse(TEXT("changed actual chord is not a different predicted route"),BuildGeneralNativeBoxGroupedVolume(bad,badBudget,rejected));
        TestTrue(TEXT("failure clears previous geometry"),rejected.Groups.IsEmpty()&&rejected.Slabs.IsEmpty());
        FBoxPoseCorridorInput corridor;corridor.BeforeCOM=v.OwnerState.Location;
        corridor.AfterCOM=v.OwnerState.Location+FVector(.125,-.0625,.25);
        corridor.BeforeRotation=v.OwnerState.Rotation;
        corridor.AfterRotation=(FQuat(FVector::ForwardVector,.001)*v.OwnerState.Rotation).GetNormalized();
        corridor.LocalCOM=v.LocalCOM;corridor.LocalOffset=v.LocalOffset;corridor.LocalRotation=v.LocalRotation;
        corridor.PhysicalHalfExtent=v.PhysicalHalfExtent;corridor.QueryHalfExtent=v.QueryHalfExtent;
        corridor.DomainShell=v.Query.DomainTolerance;corridor.OverlapShell=v.Query.InitialOverlapTolerance;
        FDistanceBudget corridorBudget{8192};FGroupedBoxVolumeBounds corridorVolume;
        if(!TestTrue(TEXT("both complete real correction-shape families retained"),
            BuildBoxPoseCorridorGroupedVolume(corridor,corridorBudget,corridorVolume)&&
            corridorVolume.Groups.Num()==2&&corridorVolume.Groups[0].Corners.Num()==8&&corridorVolume.Groups[1].Corners.Num()==8))return false;
        for(double s:{0.,.5,1.})
        {
            const FVector com=corridor.BeforeCOM*(1.-s)+corridor.AfterCOM*s;
            const FQuat raw(corridor.BeforeRotation.X*(1.-s)+corridor.AfterRotation.X*s,
                corridor.BeforeRotation.Y*(1.-s)+corridor.AfterRotation.Y*s,
                corridor.BeforeRotation.Z*(1.-s)+corridor.AfterRotation.Z*s,
                corridor.BeforeRotation.W*(1.-s)+corridor.AfterRotation.W*s);
            for(const FQuat& q:{raw,raw.GetNormalized()})for(int32 kind=0;kind<2;++kind)for(int32 k=0;k<8;++k)
            {
                const FVector center=(com-q.RotateVector(corridor.LocalCOM))+q.RotateVector(corridor.LocalOffset);
                const auto extent=kind?corridor.QueryHalfExtent:corridor.PhysicalHalfExtent;FVector local;
                for(int32 d=0;d<3;++d)local[d]=(k&(1<<d))?extent[d]:-extent[d];
                TestTrue(TEXT("both actual native correction-shape extents stay in their OWN group"),
                    GroupedBoxTestDetail::Contains(corridorVolume.Groups[kind].Corners[k],center+(q*corridor.LocalRotation).RotateVector(local)));
            }
        }
    }
    return TestTrue(TEXT("all numeric controls restored"),context.Restore.Restore())&&!HasAnyErrors();
}

#include "CanonicalGroupedBoxOverlapBounds.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAnalyticCanonicalGroupedOverlapBindingTest,
    "IAmSpeed.AnalyticWorld.GeneralBox.FixedOverlapQueryBinding",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FAnalyticCanonicalGroupedOverlapBindingTest::RunTest(const FString&)
{
    using namespace Speed::Analytic;using namespace Speed::Analytic::Certified;
    FTensorBoundsTestContext context;if(!TestTrue(TEXT("complete numeric fixture"),context.Ready))return false;
    const uint32 controls=VectorGetControlRegister();
    for(uint32 flush:{0u,64u,32768u,32832u})
    {
        VectorSetControlRegister((controls&~uint32(0x8040))|flush);
        // Exact captured Fast20 query783 values, pure math only. No actual
        // actor/filter/Source/provider/physical result follows from this test.
        FWorldQuery q;q.Shape=EQueryShape::Box;q.bAuthorityOnly=q.bApplyCollisionFilter=true;
        q.bObjectQuery=true;q.ObjectTypes=1;q.BlockingObjectTypes=MAX_uint64;
        q.Start=q.End=FVector3d(-.020819040221504739,3862.8409775620598,34.211963004952253);
        q.Rotation=FQuat4d(.013574134631303201,-.013568881388721629,.70697377537564388,.70697928488604156);
        q.HalfExtent=FVector3d(65.660003662109375,43.585201263427734,15.947199821472168);
        q.DomainTolerance=1.e-6;q.InitialOverlapTolerance=.05;
        FDistanceBudget budget{4096};FCanonicalGroupedBoxOverlapBoundsPtr value;
        if(!TestTrue(TEXT("exact stationary query closes one opaque whole volume binding"),
            FCanonicalGroupedBoxOverlapBounds::Build(q,budget,value)&&value&&value->MatchesQuery(q)))return false;
        const auto& volume=value->GetVolumeForInspection();
        if(!TestTrue(TEXT("one complete Box group and six fixed-axis slabs"),
            volume.Groups.Num()==1&&volume.Groups[0].Corners.Num()==8&&volume.Axes.Num()==6&&volume.Slabs.Num()==6))return false;
        Private::FBoxSweepContext axes;axes.Initialize(q);
        for(int32 k=0;k<8;++k)
        {
            FVector3d corner;
            for(int32 d=0;d<3;++d)corner[d]=(k&(1<<d))?q.HalfExtent[d]:-q.HalfExtent[d];
            const FVector3d rotated=q.Start+q.Rotation.RotateVector(corner);
            const FVector3d sat=q.Start+axes.Axis[0]*corner.X+axes.Axis[1]*corner.Y+axes.Axis[2]*corner.Z;
            for(const auto& point:{rotated,sat})for(int32 d=0;d<3;++d)
                TestTrue(TEXT("actual captured Rotate and SAT whole-Box corners enclosed with shells"),
                    point[d]>=volume.Groups[0].Corners[k].Axis[d].Lower&&point[d]<=volume.Groups[0].Corners[k].Axis[d].Upper);
        }
        // Every query field is retained, not just position/extent/normal.
        for(int32 change=0;change<12;++change)
        {
            auto different=q;
            switch(change)
            {
            case 0:different.ObjectTypes=2;break;
            case 1:different.TraceChannel=1;break;
            case 2:different.RequiredSourceId=1;break;
            case 3:different.RequiredSurfaceId=1;break;
            case 4:different.RequiredCanonicalGroupId=1;break;
            case 5:different.bIncludeCompactPatches=!q.bIncludeCompactPatches;break;
            case 6:different.bIncludeTriangles=!q.bIncludeTriangles;break;
            case 7:different.bExcludeTori=!q.bExcludeTori;break;
            case 8:different.MinimumReferenceNormalDot=0;break;
            case 9:different.ReferenceNormal.X=1;break;
            case 10:different.Radius=AnalyticCertified::FromBits(0x8000000000000000ULL);break;
            default:different.InitialOverlapTolerance=.1;break;
            }
            TestFalse(TEXT("different copied query cannot borrow prior geometric binding"),value->MatchesQuery(different));
        }
        for(int32 change=0;change<12;++change)
        {
            auto bad=q;
            switch(change)
            {
            case 0:bad.Shape=EQueryShape::Sphere;break;
            case 1:bad.bAuthorityOnly=false;break;
            case 2:bad.bApplyCollisionFilter=false;break;
            case 3:bad.bExcludeAuthorityEligible=true;break;
            case 4:bad.End.Y+=.125;break;
            case 5:bad.HalfExtent.X=0;break;
            case 6:bad.Rotation=FQuat4d(0,0,0,0);break;
            case 7:bad.ReferenceNormal.Z=std::numeric_limits<double>::quiet_NaN();break;
            case 8:bad.MinimumReferenceNormalDot=std::numeric_limits<double>::infinity();break;
            case 9:bad.MinimumReferenceNormalDot=1.1;break;
            case 10:bad.DomainTolerance=-1;break;
            default:bad.InitialOverlapTolerance=-.1;break;
            }
            FDistanceBudget rejectedBudget{4096};auto rejected=value;
            TestFalse(TEXT("malformed query never reaches a valid grouped pose operation"),FCanonicalGroupedBoxOverlapBounds::Build(bad,rejectedBudget,rejected));
            TestFalse(TEXT("refused opaque output clears prior value"),rejected.IsValid());
        }
        auto signedZero=q;signedZero.Start.X=0;signedZero.End.X=AnalyticCertified::FromBits(0x8000000000000000ULL);
        FDistanceBudget signedBudget{4096};FCanonicalGroupedBoxOverlapBoundsPtr signedValue;
        TestFalse(TEXT("stationary Start/End equality is exact by bits"),FCanonicalGroupedBoxOverlapBounds::Build(signedZero,signedBudget,signedValue));
        FDistanceBudget stopped{25,7,24};auto stale=value;
        TestFalse(TEXT("single shared budget cannot reset after geometry construction starts"),FCanonicalGroupedBoxOverlapBounds::Build(q,stopped,stale));
        TestTrue(TEXT("opaque failure remains closed and does not refund consumed work"),!stale&&stopped.Used==8&&stopped.Remaining==24&&stopped.ConsumptionFloor==24);
        FDistanceBudget aliasBudget{4096};
        TestTrue(TEXT("captured-query/output alias is retained safely before reset"),
            FCanonicalGroupedBoxOverlapBounds::Build(value->GetCapturedForInspection(),aliasBudget,value)&&value&&value->MatchesQuery(q));
    }
    return TestTrue(TEXT("original numeric controls restored"),context.Restore.Restore())&&!HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAnalyticGeneralBoxGlobalTimeTest,
    "IAmSpeed.AnalyticWorld.GeneralBox.GlobalTimeRanges",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FAnalyticGeneralBoxGlobalTimeTest::RunTest(const FString&)
{
    using namespace Speed;using namespace Speed::Analytic::Certified;
    FTensorBoundsTestContext context;if(!TestTrue(TEXT("full numeric context"),context.Ready))return false;
    const uint32 control=VectorGetControlRegister();
    for(uint32 flush:{0u,64u,32768u,32832u})
    {
        VectorSetControlRegister((control&~uint32(0x8040))|flush);
        FGeneralBoxMotionInput input;input.Owner.Location=FVector(13,-9,70);input.Owner.Rotation=FQuat(FVector::UpVector,.7);
        input.Owner.Velocity=FVector(12,-3,2);input.Owner.Acceleration=FVector(-7,4,-650);
        input.Owner.AngularVelocity=FVector(.4,-.25,.7);input.Owner.AngularAcceleration=FVector(.05,.1,-.2);
        input.LocalCOM=FVector(2,-3,1);input.LocalOffset=FVector(7,1,-2);input.LocalRotation=FQuat(FVector::RightVector,-.55);
        input.PhysicalHalfExtent=FVector(65.66,43.5852,15.9472);input.QueryHalfExtent=FVector(float(65.66),float(43.5852),float(15.9472));
        input.Horizon=1.f/300.f;FGeneralBoxMotionBounds old,whole;
        if(!TestTrue(TEXT("whole global range retains original theorem"),BuildGeneralBoxMotionBounds(input,old)&&
            BuildGeneralBoxMotionBoundsOnGlobalFloatSeconds(input,0.f,input.Horizon,whole)&&old.Corners.Num()==32&&whole.Corners.Num()==32))return false;
        TestTrue(TEXT("original full Hull bits retained"),GroupedBoxDetail::SamePoint(old.Hull,whole.Hull));
        TestTrue(TEXT("original native center bits retained"),GroupedBoxDetail::SamePoint(old.NativeCenter,whole.NativeCenter));
        TestTrue(TEXT("original reference center bits retained"),GroupedBoxDetail::SamePoint(old.ReferenceCenter,whole.ReferenceCenter));
        for(int32 k=0;k<32;++k)TestTrue(TEXT("all original full-range corner bounds unchanged"),GroupedBoxDetail::SamePoint(old.Corners[k],whole.Corners[k]));
        for(const auto& range:{TPair<float,float>(0.f,input.Horizon*.5f),TPair<float,float>(input.Horizon*.5f,input.Horizon)})
        {
            FGeneralBoxMotionBounds part;if(!TestTrue(TEXT("global cell valid"),BuildGeneralBoxMotionBoundsOnGlobalFloatSeconds(input,range.Key,range.Value,part)&&part.Corners.Num()==32))return false;
            for(float time:{range.Key,(range.Key+range.Value)*.5f,range.Value})
            {
                const auto actual=input.Owner.Integrate(time);const auto q=actual.Rotation*input.LocalRotation;
                const auto center=(actual.Location-actual.Rotation.RotateVector(input.LocalCOM))+actual.Rotation.RotateVector(input.LocalOffset);
                for(int32 kind=0;kind<2;++kind)for(int32 k=0;k<8;++k)
                {
                    FVector local;const auto extent=kind?input.QueryHalfExtent:input.PhysicalHalfExtent;
                    for(int32 d=0;d<3;++d)local[d]=(k&(1<<d))?extent[d]:-extent[d];
                    TestTrue(TEXT("actual native evaluation uses original state and global seconds"),GroupedBoxTestDetail::Contains(part.Corners[kind*8+k],center+q.RotateVector(local)));
                }
            }
        }
        for(const auto& range:{TPair<float,float>(-0.f,input.Horizon),TPair<float,float>(-1.f,input.Horizon),
            TPair<float,float>(input.Horizon,0.f),TPair<float,float>(0.f,input.Horizon*2),
            TPair<float,float>(std::numeric_limits<float>::denorm_min(),input.Horizon),TPair<float,float>(0.f,std::numeric_limits<float>::quiet_NaN())})
        {
            auto rejected=whole;TestFalse(TEXT("unsupported global seconds refused"),BuildGeneralBoxMotionBoundsOnGlobalFloatSeconds(input,range.Key,range.Value,rejected));
            TestTrue(TEXT("global range refusal clears all old geometry"),rejected.Corners.IsEmpty()&&!Detail::ValidPoint(rejected.Hull)&&!Detail::ValidPoint(rejected.NativeCenter)&&!Detail::ValidPoint(rejected.ReferenceCenter));
        }
        const auto containsQuaternion=[](const GeneralBoxDetail::FQuaternion& bounds,const FQuat& actual)
        {
            const double values[]={actual.X,actual.Y,actual.Z,actual.W};
            for(int32 k=0;k<4;++k)if(!GeneralBoxDetail::Contains(GeneralBoxDetail::Expanded(bounds.Axis[k]),values[k]))return false;
            return true;
        };
        auto tiny=input;tiny.Horizon=std::numeric_limits<float>::min();FGeneralBoxMotionBounds oldTiny;
        TArray<FGeneralBoxMotionBounds,TInlineAllocator<2>> tinyPieces;FDistanceBudget tinyBudget{8192};
        const bool tinyOldKnown=BuildGeneralBoxMotionBounds(tiny,oldTiny),tinyKnown=NativeGroupedBoxDetail::GlobalMotionPieces(tiny,tinyBudget,tinyPieces);
        if(!TestTrue(TEXT("unsupported half-horizon preserves original whole-time status"),tinyKnown==tinyOldKnown))return false;
        if(tinyKnown)
        {
            if(!TestTrue(TEXT("unsupported midpoint retains exactly one whole family"),tinyPieces.Num()==1))return false;
            if(!TestTrue(TEXT("fallback preserves both complete32corner families"),oldTiny.Corners.Num()==32&&tinyPieces[0].Corners.Num()==32))return false;
            TestTrue(TEXT("fallback retains original Hull and centers"),GroupedBoxDetail::SamePoint(oldTiny.Hull,tinyPieces[0].Hull)&&
                GroupedBoxDetail::SamePoint(oldTiny.NativeCenter,tinyPieces[0].NativeCenter)&&GroupedBoxDetail::SamePoint(oldTiny.ReferenceCenter,tinyPieces[0].ReferenceCenter));
            for(int32 k=0;k<32;++k)TestTrue(TEXT("fallback retains each original corner"),GroupedBoxDetail::SamePoint(oldTiny.Corners[k],tinyPieces[0].Corners[k]));
        }
        else TestTrue(TEXT("fallback failure leaves no partial family"),tinyPieces.IsEmpty());
        FKinematicState zero;zero.Rotation=FQuat::Identity;GeneralBoxDetail::FQuaternion q;
        TestTrue(TEXT("static zero branch retained at positive time"),GeneralBoxGlobalTimeDetail::RotationFamilyOnGlobalFloatSeconds(zero,.25f,.5f,true,q)&&containsQuaternion(q,zero.Integrate(.5f).Rotation));
        zero.AngularVelocity=FVector(1,0,0);zero.AngularAcceleration=FVector(-2,0,0);
        TestTrue(TEXT("exact zero cancellation at positive time retains initial quaternion"),GeneralBoxGlobalTimeDetail::RotationFamilyOnGlobalFloatSeconds(zero,1.f,1.f,true,q)&&containsQuaternion(q,zero.Integrate(1.f).Rotation));
        zero.AngularAcceleration=FVector::ZeroVector;
        TestTrue(TEXT("strict nonzero global theta excludes impossible initial quaternion"),GeneralBoxGlobalTimeDetail::RotationFamilyOnGlobalFloatSeconds(zero,.25f,.25f,true,q)&&GeneralBoxDetail::Expanded(q.Axis[0]).Lower>0&&containsQuaternion(q,zero.Integrate(.25f).Rotation));
        zero.AngularVelocity=FVector(std::numeric_limits<double>::denorm_min(),0,0);
        TestTrue(TEXT("possible flushed tiny theta keeps zero branch"),GeneralBoxGlobalTimeDetail::RotationFamilyOnGlobalFloatSeconds(zero,1.f,1.f,true,q)&&containsQuaternion(q,zero.Integrate(1.f).Rotation));
    }
    return true;
}

namespace GlobalBoxCaptured790Detail
{
inline double Value(uint64 bits){double value;FMemory::Memcpy(&value,&bits,sizeof(value));return value;}
// Actual provider1118 C2_OUTER_HIGH_BODY_V1/XNYP/side/LowerArc1, complete36CP enclosures.
// Packet090d4115..., block2136799/1728B SHA6412dfaa... ; pure single-provider regression only.
static const uint64 ControlBits[]={
    0xc0ac1fd4dc8e8476ULL,0xc0ac1fd4dc8e8475ULL,0x40aec3ef1535754bULL,0x40aec3ef1535754cULL,0x40337ca1866b95ceULL,0x40337ca1866b95cfULL,
    0xc0ac1fd4dc8e8476ULL,0xc0ac1fd4dc8e8475ULL,0x40aee915d761c9e9ULL,0x40aee915d761c9eaULL,0x403b2e5f9ff717bfULL,0x403b2e5f9ff717c0ULL,
    0xc0ac1fd4dc8e8476ULL,0xc0ac1fd4dc8e8475ULL,0x40af0cb9d77e832fULL,0x40af0cb9d77e8330ULL,0x4042597cb8670af3ULL,0x4042597cb8670af4ULL,
    0xc0ac1fd4dc8e8476ULL,0xc0ac1fd4dc8e8475ULL,0x40af2e60d35523d4ULL,0x40af2e60d35523d5ULL,0x4047f892ffbb3959ULL,0x4047f892ffbb395aULL,
    0xc0ac1fd4dc8e8476ULL,0xc0ac1fd4dc8e8475ULL,0x40af4d9aae3cf7ebULL,0x40af4d9aae3cf7ecULL,0x404e61b85b441c72ULL,0x404e61b85b441c73ULL,
    0xc0ac1fd4dc8e8476ULL,0xc0ac1fd4dc8e8475ULL,0x40af6a09e667f3bcULL,0x40af6a09e667f3bdULL,0x4052bec333018866ULL,0x4052bec333018867ULL,
    0xc0a67fdd7d3ed05fULL,0xc0a67fdd7d3ed05eULL,0x40aec3ef1535754bULL,0x40aec3ef1535754cULL,0x40337ca1866b95ceULL,0x40337ca1866b95cfULL,
    0xc0a67fdd7d3ed05fULL,0xc0a67fdd7d3ed05eULL,0x40aee915d761c9e9ULL,0x40aee915d761c9eaULL,0x403b2e5f9ff717bfULL,0x403b2e5f9ff717c0ULL,
    0xc0a67fdd7d3ed05fULL,0xc0a67fdd7d3ed05eULL,0x40af0cb9d77e832fULL,0x40af0cb9d77e8330ULL,0x4042597cb8670af3ULL,0x4042597cb8670af4ULL,
    0xc0a67fdd7d3ed05fULL,0xc0a67fdd7d3ed05eULL,0x40af2e60d35523d4ULL,0x40af2e60d35523d5ULL,0x4047f892ffbb3959ULL,0x4047f892ffbb395aULL,
    0xc0a67fdd7d3ed05fULL,0xc0a67fdd7d3ed05eULL,0x40af4d9aae3cf7ebULL,0x40af4d9aae3cf7ecULL,0x404e61b85b441c72ULL,0x404e61b85b441c73ULL,
    0xc0a67fdd7d3ed05fULL,0xc0a67fdd7d3ed05eULL,0x40af6a09e667f3bcULL,0x40af6a09e667f3bdULL,0x4052bec333018866ULL,0x4052bec333018867ULL,
    0xc0a0dfe61def1c47ULL,0xc0a0dfe61def1c46ULL,0x40aec3ef1535754bULL,0x40aec3ef1535754cULL,0x40337ca1866b95ceULL,0x40337ca1866b95cfULL,
    0xc0a0dfe61def1c47ULL,0xc0a0dfe61def1c46ULL,0x40aee915d761c9e9ULL,0x40aee915d761c9eaULL,0x403b2e5f9ff717bfULL,0x403b2e5f9ff717c0ULL,
    0xc0a0dfe61def1c47ULL,0xc0a0dfe61def1c46ULL,0x40af0cb9d77e832fULL,0x40af0cb9d77e8330ULL,0x4042597cb8670af3ULL,0x4042597cb8670af4ULL,
    0xc0a0dfe61def1c47ULL,0xc0a0dfe61def1c46ULL,0x40af2e60d35523d4ULL,0x40af2e60d35523d5ULL,0x4047f892ffbb3959ULL,0x4047f892ffbb395aULL,
    0xc0a0dfe61def1c47ULL,0xc0a0dfe61def1c46ULL,0x40af4d9aae3cf7ebULL,0x40af4d9aae3cf7ecULL,0x404e61b85b441c72ULL,0x404e61b85b441c73ULL,
    0xc0a0dfe61def1c47ULL,0xc0a0dfe61def1c46ULL,0x40af6a09e667f3bcULL,0x40af6a09e667f3bdULL,0x4052bec333018866ULL,0x4052bec333018867ULL,
    0xc0967fdd7d3ed05fULL,0xc0967fdd7d3ed05eULL,0x40aec3ef1535754bULL,0x40aec3ef1535754cULL,0x40337ca1866b95ceULL,0x40337ca1866b95cfULL,
    0xc0967fdd7d3ed05fULL,0xc0967fdd7d3ed05eULL,0x40aee915d761c9e9ULL,0x40aee915d761c9eaULL,0x403b2e5f9ff717bfULL,0x403b2e5f9ff717c0ULL,
    0xc0967fdd7d3ed05fULL,0xc0967fdd7d3ed05eULL,0x40af0cb9d77e832fULL,0x40af0cb9d77e8330ULL,0x4042597cb8670af3ULL,0x4042597cb8670af4ULL,
    0xc0967fdd7d3ed05fULL,0xc0967fdd7d3ed05eULL,0x40af2e60d35523d4ULL,0x40af2e60d35523d5ULL,0x4047f892ffbb3959ULL,0x4047f892ffbb395aULL,
    0xc0967fdd7d3ed05fULL,0xc0967fdd7d3ed05eULL,0x40af4d9aae3cf7ebULL,0x40af4d9aae3cf7ecULL,0x404e61b85b441c72ULL,0x404e61b85b441c73ULL,
    0xc0967fdd7d3ed05fULL,0xc0967fdd7d3ed05eULL,0x40af6a09e667f3bcULL,0x40af6a09e667f3bdULL,0x4052bec333018866ULL,0x4052bec333018867ULL,
    0xc0867fdd7d3ed05fULL,0xc0867fdd7d3ed05eULL,0x40aec3ef1535754bULL,0x40aec3ef1535754cULL,0x40337ca1866b95ceULL,0x40337ca1866b95cfULL,
    0xc0867fdd7d3ed05fULL,0xc0867fdd7d3ed05eULL,0x40aee915d761c9e9ULL,0x40aee915d761c9eaULL,0x403b2e5f9ff717bfULL,0x403b2e5f9ff717c0ULL,
    0xc0867fdd7d3ed05fULL,0xc0867fdd7d3ed05eULL,0x40af0cb9d77e832fULL,0x40af0cb9d77e8330ULL,0x4042597cb8670af3ULL,0x4042597cb8670af4ULL,
    0xc0867fdd7d3ed05fULL,0xc0867fdd7d3ed05eULL,0x40af2e60d35523d4ULL,0x40af2e60d35523d5ULL,0x4047f892ffbb3959ULL,0x4047f892ffbb395aULL,
    0xc0867fdd7d3ed05fULL,0xc0867fdd7d3ed05eULL,0x40af4d9aae3cf7ebULL,0x40af4d9aae3cf7ecULL,0x404e61b85b441c72ULL,0x404e61b85b441c73ULL,
    0xc0867fdd7d3ed05fULL,0xc0867fdd7d3ed05eULL,0x40af6a09e667f3bcULL,0x40af6a09e667f3bdULL,0x4052bec333018866ULL,0x4052bec333018867ULL,
    0x0000000000000000ULL,0x0000000000000000ULL,0x40aec3ef1535754bULL,0x40aec3ef1535754cULL,0x40337ca1866b95ceULL,0x40337ca1866b95cfULL,
    0x0000000000000000ULL,0x0000000000000000ULL,0x40aee915d761c9e9ULL,0x40aee915d761c9eaULL,0x403b2e5f9ff717bfULL,0x403b2e5f9ff717c0ULL,
    0x0000000000000000ULL,0x0000000000000000ULL,0x40af0cb9d77e832fULL,0x40af0cb9d77e8330ULL,0x4042597cb8670af3ULL,0x4042597cb8670af4ULL,
    0x0000000000000000ULL,0x0000000000000000ULL,0x40af2e60d35523d4ULL,0x40af2e60d35523d5ULL,0x4047f892ffbb3959ULL,0x4047f892ffbb395aULL,
    0x0000000000000000ULL,0x0000000000000000ULL,0x40af4d9aae3cf7ebULL,0x40af4d9aae3cf7ecULL,0x404e61b85b441c72ULL,0x404e61b85b441c73ULL,
    0x0000000000000000ULL,0x0000000000000000ULL,0x40af6a09e667f3bcULL,0x40af6a09e667f3bdULL,0x4052bec333018866ULL,0x4052bec333018867ULL
};
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAnalyticCapturedGeneralBox790Test,
    "IAmSpeed.AnalyticWorld.GeneralBox.CapturedGutterBox790",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FAnalyticCapturedGeneralBox790Test::RunTest(const FString&)
{
    using namespace Speed;using namespace Speed::Analytic;using namespace Speed::Analytic::Certified;
    FTensorBoundsTestContext context;if(!TestTrue(TEXT("full numeric context"),context.Ready))return false;
    const uint32 control=VectorGetControlRegister();
    for(uint32 flush:{0u,64u,32768u,32832u})
    {
        VectorSetControlRegister((control&~uint32(0x8040))|flush);
        PreparedExecution::FNativeSolidStaticValues v;v.Shape=PreparedExecution::ENativeSolidQueryShape::Box;v.Horizon=1.f/300.f;
        v.OwnerState.Location=FVector(-.020930030577393673,3881.4387734064535,25.900026224510487);
        v.OwnerState.Rotation=FQuat(.05001753468123158,-.049916214272657973,.70515744050889895,.70552080187053068);
        v.OwnerState.Velocity=FVector(.05615215488566689,1359.432011069915,197.30565564314531);
        v.OwnerState.Acceleration=FVector(11.523763243138758,-392.03818799999249,657.96829772131002);
        v.OwnerState.AngularVelocity=FVector(5.4997421457046354,.022048290607730309,-.048478898929900524);
        v.OwnerState.AngularAcceleration=FVector(-.0017249283061223162,-.95121225419631217,-.58541089587304229);
        v.BodyState=v.OwnerState;v.BodyState.Location=FVector(-.014931684634486516,3888.6524032285188,39.143421404560044);
        v.BodyState.Velocity=FVector(.69785521151002206,1286.5964616527681,236.97862734579374);
        v.BodyState.Acceleration=FVector(.49315492801744654,-610.24107916587764,257.37067113669411);
        v.LocalCOM=FVector(-14.008570000000001,0,17.415800000000001);v.LocalOffset=FVector(-5,0,29.510000000000002);v.LocalRotation=FQuat(0,-0.,0,1);
        v.PhysicalHalfExtent=FVector(65.659999999999997,43.5852,15.9472);v.QueryHalfExtent=FVector(65.660003662109375,43.585201263427734,15.947199821472168);
        v.Query.Shape=EQueryShape::Box;v.Query.Start=v.BodyState.Location;v.Query.End=FVector(-.012602760790059876,3892.9376679761599,39.934780018638108);
        v.Query.Rotation=v.BodyState.Rotation;v.Query.HalfExtent=v.QueryHalfExtent;v.Query.DomainTolerance=.000001;v.Query.InitialOverlapTolerance=.05;
        FIntervalTensorSurface surface;surface.DegreeU=surface.DegreeV=5;
        for(int32 k=0;k<36;++k)
        {
            FIntervalPoint point;for(int32 d=0;d<3;++d)point.Axis[d]={GlobalBoxCaptured790Detail::Value(GlobalBoxCaptured790Detail::ControlBits[k*6+d*2]),GlobalBoxCaptured790Detail::Value(GlobalBoxCaptured790Detail::ControlBits[k*6+d*2+1]),true};
            surface.ControlPoints.Add(point);
        }
        const auto scene=GroupedBoxTestDetail::Scene(surface);FDistanceBudget budget{26439};FGroupedBoxVolumeBounds volume;FReferenceBoxVolumeStrictSeparationPtr proof;
        if(!TestTrue(TEXT("captured actual motion/query yields14completegroups"),BuildGeneralNativeBoxGroupedVolume(v,budget,volume)&&volume.Groups.Num()==14))return false;
        if(!TestTrue(TEXT("captured provider separation uses all regions on same budget"),GroupedBoxTestDetail::Prove(scene,volume,budget,proof)&&proof&&proof->Matches(scene,volume)))return false;
        AddInfo(FString::Printf(TEXT("captured_Box790 flush=%u groups=%d used=%llu remaining=%d pure_single_provider_only=1"),flush,volume.Groups.Num(),budget.Used,budget.Remaining));
    }
    return true;
}


IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAnalyticInheritedGroupedBoxSeparationTest,
    "IAmSpeed.AnalyticWorld.GeneralBox.InheritedGroupSeparation",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FAnalyticInheritedGroupedBoxSeparationTest::RunTest(const FString&)
{
    using namespace Speed::Analytic::Certified;using namespace GroupedBoxTestDetail;
    FTensorBoundsTestContext context;if(!TestTrue(TEXT("complete RNE inheritance fixture"),context.Ready))return false;
    const uint32 controls=VectorGetControlRegister();
    for(uint32 flush:{0u,64u,32768u,32832u})
    {
        VectorSetControlRegister((controls&~uint32(0x8040))|flush);
        // Exact Pz=(2u-1)^2 >=0, although its ORIGINAL middle CP is -1.
        // First group unresolved at root, second clear: all groups must be
        // scanned before passing only the first obligation to BOTH children.
        FIntervalTensorSurface curved;curved.DegreeU=2;curved.DegreeV=1;
        const double x[3]={-2,0,2},z[3]={1,-1,1};
        for(int32 u=0;u<3;++u)for(double y:{-1.,1.})curved.ControlPoints.Add(Detail::Point(FVector3d(x[u],y,z[u])));
        TArray<FBoxVolumeGroup> groups{Box(FVector3d(0,0,-.5),FVector3d(.125)),Box(FVector3d(0,0,4),FVector3d(.125))};
        FGroupedBoxVolumeBounds volume;if(!TestTrue(TEXT("two complete groups retained"),Volume(groups,FQuat4d::Identity,volume)))return false;
        const auto scene=Scene(curved);FDistanceBudget budget{1024};FReferenceBoundsCandidateSetPtr partition;FReferenceBoxVolumeStrictSeparationPtr proof;FGroupedBoxSeparationObservation observation;
        if(!TestTrue(TEXT("ancestor obligations close the SAME complete scene and volume"),FReferenceBoundsCandidateSet::Select(scene,volume.BroadHull,budget,partition)&&
            FReferenceBoxVolumeStrictSeparation::TryProve(partition,volume,budget,proof,&observation)&&proof&&proof->Matches(scene,volume)))return false;
        TestTrue(TEXT("later group scanned after unresolved first group and inherited in both children"),observation.VisitedCells>=3&&observation.InheritedGroupSkips>=2);
        for(const auto& group:groups)TestTrue(TEXT("all original complete endpoint groups remain admitted"),proof->ContainsCompleteEndpointGroup(group.Corners));
        // A clear left sibling must NEVER discharge a genuinely intersecting
        // right sibling of the same original affine surface.
        FIntervalTensorSurface plane;plane.DegreeU=plane.DegreeV=1;
        for(double u:{-1.,1.})for(double v:{-1.,1.})plane.ControlPoints.Add(Detail::Point(FVector3d(u,v,0)));
        TArray<FBoxVolumeGroup> right{Box(FVector3d(.75,0,0),FVector3d(.125))};FGroupedBoxVolumeBounds rightVolume;
        TestTrue(TEXT("sibling-counterexample volume"),Volume(right,FQuat4d::Identity,rightVolume));
        FDistanceBudget siblingBudget{512};auto stale=proof;TestFalse(TEXT("clear sibling does not suppress intersecting sibling"),Prove(Scene(plane),rightVolume,siblingBudget,stale));TestFalse(TEXT("sibling refusal clears old proof"),stale.IsValid());
        // A provider that closes its whole mask cannot empty the next one's.
        TArray<FIntervalTensorSurface> providers;
        FIntervalTensorSurface far;far.ControlPoints.Add(Detail::Point(FVector3d(0,0,3)));providers.Add(far);
        FIntervalTensorSurface inside;inside.ControlPoints.Add(Detail::Point(FVector3d(.75,0,0)));providers.Add(inside);
        FDistanceBudget build{1024};FIndexedIntervalUnionPtr two;if(!TestTrue(TEXT("two provider scene"),FIndexedIntervalUnion::Build(MakeArrayView(providers),build,two)))return false;
        // BroadHull explicitly contains the far provider, while all actual
        // regions remain unchanged; otherwise broadphase would hide this test.
        rightVolume.BroadHull=TimeDetail::Hull(rightVolume.BroadHull,far.ControlPoints[0]);
        FDistanceBudget resetBudget{1024};FReferenceBoundsCandidateSetPtr resetPartition;
        if(!TestTrue(TEXT("both providers retained in original order; clear first then intersecting second"),FReferenceBoundsCandidateSet::Select(two,rightVolume.BroadHull,resetBudget,resetPartition)&&
            resetPartition&&resetPartition->GetCandidateIndices().Num()==2&&resetPartition->GetCandidateIndices()[0]==0&&resetPartition->GetCandidateIndices()[1]==1))return false;
        stale=proof;TestFalse(TEXT("each provider starts with every original group obligation"),FReferenceBoxVolumeStrictSeparation::TryProve(resetPartition,rightVolume,resetBudget,stale));TestFalse(TEXT("provider refusal clears old proof"),stale.IsValid());
        TArray<FBoxVolumeGroup> sixteen;
        for(int32 g=0;g<15;++g)sixteen.Add(Box(FVector3d(0,0,4.+g),FVector3d(.125)));
        sixteen.Add(Box(FVector3d::ZeroVector,FVector3d(1)));FGroupedBoxVolumeBounds sixteenVolume;
        TestTrue(TEXT("all sixteen group bits retained"),Volume(sixteen,FQuat4d::Identity,sixteenVolume));
        FIntervalTensorSurface tangent;tangent.ControlPoints.Add(Detail::Point(FVector3d(0,0,1)));
        FDistanceBudget tangentBudget{1024};stale=proof;TestFalse(TEXT("exact face touch in bit15 never becomes strict separation"),Prove(Scene(tangent),sixteenVolume,tangentBudget,stale));TestFalse(TEXT("tangent refuses all partial output"),stale.IsValid());
        FDistanceBudget stopped{100,7,51};stale=proof;FGroupedBoxSeparationObservation stoppedAt;
        TestFalse(TEXT("budget denial in active group scan cannot emit inherited partial proof"),FReferenceBoxVolumeStrictSeparation::TryProve(partition,volume,stopped,stale,&stoppedAt));
        TestTrue(TEXT("denial occurs AFTER unresolved first group, BEFORE classifying second"),FString(stoppedAt.Stage)==TEXT("cell_group_or_budget")&&stoppedAt.Group==1&&stoppedAt.TestedGroups==1);
        TestTrue(TEXT("same floor and all prior debits preserved"),!stale&&stopped.Used==56&&stopped.Remaining==51&&stopped.ConsumptionFloor==51);
    }
    return TestTrue(TEXT("numeric inheritance context restored"),context.Restore.Restore())&&!HasAnyErrors();
}


namespace GlobalBoxCaptured800Detail
{
inline double Value(uint64 bits){double value;FMemory::Memcpy(&value,&bits,sizeof(value));return value;}
static const uint64 CP118[]={
    0xc0ac1fd4dc8e8476ULL,0xc0ac1fd4dc8e8475ULL,0x40ae000000000000ULL,0x40ae000000000000ULL,0x0000000000000000ULL,0x0000000000000000ULL,
    0xc0ac1fd4dc8e8476ULL,0xc0ac1fd4dc8e8475ULL,0x40ae28365eed39e1ULL,0x40ae28365eed39e2ULL,0x0000000000000000ULL,0x0000000000000000ULL,
    0xc0ac1fd4dc8e8476ULL,0xc0ac1fd4dc8e8475ULL,0x40ae506cbdda73c3ULL,0x40ae506cbdda73c4ULL,0x0000000000000000ULL,0x0000000000000000ULL,
    0xc0ac1fd4dc8e8476ULL,0xc0ac1fd4dc8e8475ULL,0x40ae781ececd30b4ULL,0x40ae781ececd30b5ULL,0x4017b0042a803897ULL,0x4017b0042a803898ULL,
    0xc0ac1fd4dc8e8476ULL,0xc0ac1fd4dc8e8475ULL,0x40ae9ec8530920acULL,0x40ae9ec8530920adULL,0x402795c6d9c027bcULL,0x402795c6d9c027bdULL,
    0xc0ac1fd4dc8e8476ULL,0xc0ac1fd4dc8e8475ULL,0x40aec3ef1535754bULL,0x40aec3ef1535754cULL,0x40337ca1866b95ceULL,0x40337ca1866b95cfULL,
    0xc0a67fdd7d3ed05fULL,0xc0a67fdd7d3ed05eULL,0x40ae000000000000ULL,0x40ae000000000000ULL,0x0000000000000000ULL,0x0000000000000000ULL,
    0xc0a67fdd7d3ed05fULL,0xc0a67fdd7d3ed05eULL,0x40ae28365eed39e1ULL,0x40ae28365eed39e2ULL,0x0000000000000000ULL,0x0000000000000000ULL,
    0xc0a67fdd7d3ed05fULL,0xc0a67fdd7d3ed05eULL,0x40ae506cbdda73c3ULL,0x40ae506cbdda73c4ULL,0x0000000000000000ULL,0x0000000000000000ULL,
    0xc0a67fdd7d3ed05fULL,0xc0a67fdd7d3ed05eULL,0x40ae781ececd30b4ULL,0x40ae781ececd30b5ULL,0x4017b0042a803897ULL,0x4017b0042a803898ULL,
    0xc0a67fdd7d3ed05fULL,0xc0a67fdd7d3ed05eULL,0x40ae9ec8530920acULL,0x40ae9ec8530920adULL,0x402795c6d9c027bcULL,0x402795c6d9c027bdULL,
    0xc0a67fdd7d3ed05fULL,0xc0a67fdd7d3ed05eULL,0x40aec3ef1535754bULL,0x40aec3ef1535754cULL,0x40337ca1866b95ceULL,0x40337ca1866b95cfULL,
    0xc0a0dfe61def1c47ULL,0xc0a0dfe61def1c46ULL,0x40ae000000000000ULL,0x40ae000000000000ULL,0x0000000000000000ULL,0x0000000000000000ULL,
    0xc0a0dfe61def1c47ULL,0xc0a0dfe61def1c46ULL,0x40ae28365eed39e1ULL,0x40ae28365eed39e2ULL,0x0000000000000000ULL,0x0000000000000000ULL,
    0xc0a0dfe61def1c47ULL,0xc0a0dfe61def1c46ULL,0x40ae506cbdda73c3ULL,0x40ae506cbdda73c4ULL,0x0000000000000000ULL,0x0000000000000000ULL,
    0xc0a0dfe61def1c47ULL,0xc0a0dfe61def1c46ULL,0x40ae781ececd30b4ULL,0x40ae781ececd30b5ULL,0x4017b0042a803897ULL,0x4017b0042a803898ULL,
    0xc0a0dfe61def1c47ULL,0xc0a0dfe61def1c46ULL,0x40ae9ec8530920acULL,0x40ae9ec8530920adULL,0x402795c6d9c027bcULL,0x402795c6d9c027bdULL,
    0xc0a0dfe61def1c47ULL,0xc0a0dfe61def1c46ULL,0x40aec3ef1535754bULL,0x40aec3ef1535754cULL,0x40337ca1866b95ceULL,0x40337ca1866b95cfULL,
    0xc0967fdd7d3ed05fULL,0xc0967fdd7d3ed05eULL,0x40ae000000000000ULL,0x40ae000000000000ULL,0x0000000000000000ULL,0x0000000000000000ULL,
    0xc0967fdd7d3ed05fULL,0xc0967fdd7d3ed05eULL,0x40ae28365eed39e1ULL,0x40ae28365eed39e2ULL,0x0000000000000000ULL,0x0000000000000000ULL,
    0xc0967fdd7d3ed05fULL,0xc0967fdd7d3ed05eULL,0x40ae506cbdda73c3ULL,0x40ae506cbdda73c4ULL,0x0000000000000000ULL,0x0000000000000000ULL,
    0xc0967fdd7d3ed05fULL,0xc0967fdd7d3ed05eULL,0x40ae781ececd30b4ULL,0x40ae781ececd30b5ULL,0x4017b0042a803897ULL,0x4017b0042a803898ULL,
    0xc0967fdd7d3ed05fULL,0xc0967fdd7d3ed05eULL,0x40ae9ec8530920acULL,0x40ae9ec8530920adULL,0x402795c6d9c027bcULL,0x402795c6d9c027bdULL,
    0xc0967fdd7d3ed05fULL,0xc0967fdd7d3ed05eULL,0x40aec3ef1535754bULL,0x40aec3ef1535754cULL,0x40337ca1866b95ceULL,0x40337ca1866b95cfULL,
    0xc0867fdd7d3ed05fULL,0xc0867fdd7d3ed05eULL,0x40ae000000000000ULL,0x40ae000000000000ULL,0x0000000000000000ULL,0x0000000000000000ULL,
    0xc0867fdd7d3ed05fULL,0xc0867fdd7d3ed05eULL,0x40ae28365eed39e1ULL,0x40ae28365eed39e2ULL,0x0000000000000000ULL,0x0000000000000000ULL,
    0xc0867fdd7d3ed05fULL,0xc0867fdd7d3ed05eULL,0x40ae506cbdda73c3ULL,0x40ae506cbdda73c4ULL,0x0000000000000000ULL,0x0000000000000000ULL,
    0xc0867fdd7d3ed05fULL,0xc0867fdd7d3ed05eULL,0x40ae781ececd30b4ULL,0x40ae781ececd30b5ULL,0x4017b0042a803897ULL,0x4017b0042a803898ULL,
    0xc0867fdd7d3ed05fULL,0xc0867fdd7d3ed05eULL,0x40ae9ec8530920acULL,0x40ae9ec8530920adULL,0x402795c6d9c027bcULL,0x402795c6d9c027bdULL,
    0xc0867fdd7d3ed05fULL,0xc0867fdd7d3ed05eULL,0x40aec3ef1535754bULL,0x40aec3ef1535754cULL,0x40337ca1866b95ceULL,0x40337ca1866b95cfULL,
    0x0000000000000000ULL,0x0000000000000000ULL,0x40ae000000000000ULL,0x40ae000000000000ULL,0x0000000000000000ULL,0x0000000000000000ULL,
    0x0000000000000000ULL,0x0000000000000000ULL,0x40ae28365eed39e1ULL,0x40ae28365eed39e2ULL,0x0000000000000000ULL,0x0000000000000000ULL,
    0x0000000000000000ULL,0x0000000000000000ULL,0x40ae506cbdda73c3ULL,0x40ae506cbdda73c4ULL,0x0000000000000000ULL,0x0000000000000000ULL,
    0x0000000000000000ULL,0x0000000000000000ULL,0x40ae781ececd30b4ULL,0x40ae781ececd30b5ULL,0x4017b0042a803897ULL,0x4017b0042a803898ULL,
    0x0000000000000000ULL,0x0000000000000000ULL,0x40ae9ec8530920acULL,0x40ae9ec8530920adULL,0x402795c6d9c027bcULL,0x402795c6d9c027bdULL,
    0x0000000000000000ULL,0x0000000000000000ULL,0x40aec3ef1535754bULL,0x40aec3ef1535754cULL,0x40337ca1866b95ceULL,0x40337ca1866b95cfULL
};

static const uint64 CP726[]={
    0x0000000000000000ULL,0x0000000000000000ULL,0x40aec3ef1535754bULL,0x40aec3ef1535754cULL,0x40337ca1866b95ceULL,0x40337ca1866b95cfULL,
    0x0000000000000000ULL,0x0000000000000000ULL,0x40aee915d761c9e9ULL,0x40aee915d761c9eaULL,0x403b2e5f9ff717bfULL,0x403b2e5f9ff717c0ULL,
    0x0000000000000000ULL,0x0000000000000000ULL,0x40af0cb9d77e832fULL,0x40af0cb9d77e8330ULL,0x4042597cb8670af3ULL,0x4042597cb8670af4ULL,
    0x0000000000000000ULL,0x0000000000000000ULL,0x40af2e60d35523d4ULL,0x40af2e60d35523d5ULL,0x4047f892ffbb3959ULL,0x4047f892ffbb395aULL,
    0x0000000000000000ULL,0x0000000000000000ULL,0x40af4d9aae3cf7ebULL,0x40af4d9aae3cf7ecULL,0x404e61b85b441c72ULL,0x404e61b85b441c73ULL,
    0x0000000000000000ULL,0x0000000000000000ULL,0x40af6a09e667f3bcULL,0x40af6a09e667f3bdULL,0x4052bec333018866ULL,0x4052bec333018867ULL,
    0x40867fdd7d3ed05eULL,0x40867fdd7d3ed05fULL,0x40aec3ef1535754bULL,0x40aec3ef1535754cULL,0x40337ca1866b95ceULL,0x40337ca1866b95cfULL,
    0x40867fdd7d3ed05eULL,0x40867fdd7d3ed05fULL,0x40aee915d761c9e9ULL,0x40aee915d761c9eaULL,0x403b2e5f9ff717bfULL,0x403b2e5f9ff717c0ULL,
    0x40867fdd7d3ed05eULL,0x40867fdd7d3ed05fULL,0x40af0cb9d77e832fULL,0x40af0cb9d77e8330ULL,0x4042597cb8670af3ULL,0x4042597cb8670af4ULL,
    0x40867fdd7d3ed05eULL,0x40867fdd7d3ed05fULL,0x40af2e60d35523d4ULL,0x40af2e60d35523d5ULL,0x4047f892ffbb3959ULL,0x4047f892ffbb395aULL,
    0x40867fdd7d3ed05eULL,0x40867fdd7d3ed05fULL,0x40af4d9aae3cf7ebULL,0x40af4d9aae3cf7ecULL,0x404e61b85b441c72ULL,0x404e61b85b441c73ULL,
    0x40867fdd7d3ed05eULL,0x40867fdd7d3ed05fULL,0x40af6a09e667f3bcULL,0x40af6a09e667f3bdULL,0x4052bec333018866ULL,0x4052bec333018867ULL,
    0x40967fdd7d3ed05eULL,0x40967fdd7d3ed05fULL,0x40aec3ef1535754bULL,0x40aec3ef1535754cULL,0x40337ca1866b95ceULL,0x40337ca1866b95cfULL,
    0x40967fdd7d3ed05eULL,0x40967fdd7d3ed05fULL,0x40aee915d761c9e9ULL,0x40aee915d761c9eaULL,0x403b2e5f9ff717bfULL,0x403b2e5f9ff717c0ULL,
    0x40967fdd7d3ed05eULL,0x40967fdd7d3ed05fULL,0x40af0cb9d77e832fULL,0x40af0cb9d77e8330ULL,0x4042597cb8670af3ULL,0x4042597cb8670af4ULL,
    0x40967fdd7d3ed05eULL,0x40967fdd7d3ed05fULL,0x40af2e60d35523d4ULL,0x40af2e60d35523d5ULL,0x4047f892ffbb3959ULL,0x4047f892ffbb395aULL,
    0x40967fdd7d3ed05eULL,0x40967fdd7d3ed05fULL,0x40af4d9aae3cf7ebULL,0x40af4d9aae3cf7ecULL,0x404e61b85b441c72ULL,0x404e61b85b441c73ULL,
    0x40967fdd7d3ed05eULL,0x40967fdd7d3ed05fULL,0x40af6a09e667f3bcULL,0x40af6a09e667f3bdULL,0x4052bec333018866ULL,0x4052bec333018867ULL,
    0x40a0dfe61def1c46ULL,0x40a0dfe61def1c47ULL,0x40aec3ef1535754bULL,0x40aec3ef1535754cULL,0x40337ca1866b95ceULL,0x40337ca1866b95cfULL,
    0x40a0dfe61def1c46ULL,0x40a0dfe61def1c47ULL,0x40aee915d761c9e9ULL,0x40aee915d761c9eaULL,0x403b2e5f9ff717bfULL,0x403b2e5f9ff717c0ULL,
    0x40a0dfe61def1c46ULL,0x40a0dfe61def1c47ULL,0x40af0cb9d77e832fULL,0x40af0cb9d77e8330ULL,0x4042597cb8670af3ULL,0x4042597cb8670af4ULL,
    0x40a0dfe61def1c46ULL,0x40a0dfe61def1c47ULL,0x40af2e60d35523d4ULL,0x40af2e60d35523d5ULL,0x4047f892ffbb3959ULL,0x4047f892ffbb395aULL,
    0x40a0dfe61def1c46ULL,0x40a0dfe61def1c47ULL,0x40af4d9aae3cf7ebULL,0x40af4d9aae3cf7ecULL,0x404e61b85b441c72ULL,0x404e61b85b441c73ULL,
    0x40a0dfe61def1c46ULL,0x40a0dfe61def1c47ULL,0x40af6a09e667f3bcULL,0x40af6a09e667f3bdULL,0x4052bec333018866ULL,0x4052bec333018867ULL,
    0x40a67fdd7d3ed05eULL,0x40a67fdd7d3ed05fULL,0x40aec3ef1535754bULL,0x40aec3ef1535754cULL,0x40337ca1866b95ceULL,0x40337ca1866b95cfULL,
    0x40a67fdd7d3ed05eULL,0x40a67fdd7d3ed05fULL,0x40aee915d761c9e9ULL,0x40aee915d761c9eaULL,0x403b2e5f9ff717bfULL,0x403b2e5f9ff717c0ULL,
    0x40a67fdd7d3ed05eULL,0x40a67fdd7d3ed05fULL,0x40af0cb9d77e832fULL,0x40af0cb9d77e8330ULL,0x4042597cb8670af3ULL,0x4042597cb8670af4ULL,
    0x40a67fdd7d3ed05eULL,0x40a67fdd7d3ed05fULL,0x40af2e60d35523d4ULL,0x40af2e60d35523d5ULL,0x4047f892ffbb3959ULL,0x4047f892ffbb395aULL,
    0x40a67fdd7d3ed05eULL,0x40a67fdd7d3ed05fULL,0x40af4d9aae3cf7ebULL,0x40af4d9aae3cf7ecULL,0x404e61b85b441c72ULL,0x404e61b85b441c73ULL,
    0x40a67fdd7d3ed05eULL,0x40a67fdd7d3ed05fULL,0x40af6a09e667f3bcULL,0x40af6a09e667f3bdULL,0x4052bec333018866ULL,0x4052bec333018867ULL,
    0x40ac1fd4dc8e8475ULL,0x40ac1fd4dc8e8476ULL,0x40aec3ef1535754bULL,0x40aec3ef1535754cULL,0x40337ca1866b95ceULL,0x40337ca1866b95cfULL,
    0x40ac1fd4dc8e8475ULL,0x40ac1fd4dc8e8476ULL,0x40aee915d761c9e9ULL,0x40aee915d761c9eaULL,0x403b2e5f9ff717bfULL,0x403b2e5f9ff717c0ULL,
    0x40ac1fd4dc8e8475ULL,0x40ac1fd4dc8e8476ULL,0x40af0cb9d77e832fULL,0x40af0cb9d77e8330ULL,0x4042597cb8670af3ULL,0x4042597cb8670af4ULL,
    0x40ac1fd4dc8e8475ULL,0x40ac1fd4dc8e8476ULL,0x40af2e60d35523d4ULL,0x40af2e60d35523d5ULL,0x4047f892ffbb3959ULL,0x4047f892ffbb395aULL,
    0x40ac1fd4dc8e8475ULL,0x40ac1fd4dc8e8476ULL,0x40af4d9aae3cf7ebULL,0x40af4d9aae3cf7ecULL,0x404e61b85b441c72ULL,0x404e61b85b441c73ULL,
    0x40ac1fd4dc8e8475ULL,0x40ac1fd4dc8e8476ULL,0x40af6a09e667f3bcULL,0x40af6a09e667f3bdULL,0x4052bec333018866ULL,0x4052bec333018867ULL
};

static const uint64 CP1118[]={
    0xc0ac1fd4dc8e8476ULL,0xc0ac1fd4dc8e8475ULL,0x40aec3ef1535754bULL,0x40aec3ef1535754cULL,0x40337ca1866b95ceULL,0x40337ca1866b95cfULL,
    0xc0ac1fd4dc8e8476ULL,0xc0ac1fd4dc8e8475ULL,0x40aee915d761c9e9ULL,0x40aee915d761c9eaULL,0x403b2e5f9ff717bfULL,0x403b2e5f9ff717c0ULL,
    0xc0ac1fd4dc8e8476ULL,0xc0ac1fd4dc8e8475ULL,0x40af0cb9d77e832fULL,0x40af0cb9d77e8330ULL,0x4042597cb8670af3ULL,0x4042597cb8670af4ULL,
    0xc0ac1fd4dc8e8476ULL,0xc0ac1fd4dc8e8475ULL,0x40af2e60d35523d4ULL,0x40af2e60d35523d5ULL,0x4047f892ffbb3959ULL,0x4047f892ffbb395aULL,
    0xc0ac1fd4dc8e8476ULL,0xc0ac1fd4dc8e8475ULL,0x40af4d9aae3cf7ebULL,0x40af4d9aae3cf7ecULL,0x404e61b85b441c72ULL,0x404e61b85b441c73ULL,
    0xc0ac1fd4dc8e8476ULL,0xc0ac1fd4dc8e8475ULL,0x40af6a09e667f3bcULL,0x40af6a09e667f3bdULL,0x4052bec333018866ULL,0x4052bec333018867ULL,
    0xc0a67fdd7d3ed05fULL,0xc0a67fdd7d3ed05eULL,0x40aec3ef1535754bULL,0x40aec3ef1535754cULL,0x40337ca1866b95ceULL,0x40337ca1866b95cfULL,
    0xc0a67fdd7d3ed05fULL,0xc0a67fdd7d3ed05eULL,0x40aee915d761c9e9ULL,0x40aee915d761c9eaULL,0x403b2e5f9ff717bfULL,0x403b2e5f9ff717c0ULL,
    0xc0a67fdd7d3ed05fULL,0xc0a67fdd7d3ed05eULL,0x40af0cb9d77e832fULL,0x40af0cb9d77e8330ULL,0x4042597cb8670af3ULL,0x4042597cb8670af4ULL,
    0xc0a67fdd7d3ed05fULL,0xc0a67fdd7d3ed05eULL,0x40af2e60d35523d4ULL,0x40af2e60d35523d5ULL,0x4047f892ffbb3959ULL,0x4047f892ffbb395aULL,
    0xc0a67fdd7d3ed05fULL,0xc0a67fdd7d3ed05eULL,0x40af4d9aae3cf7ebULL,0x40af4d9aae3cf7ecULL,0x404e61b85b441c72ULL,0x404e61b85b441c73ULL,
    0xc0a67fdd7d3ed05fULL,0xc0a67fdd7d3ed05eULL,0x40af6a09e667f3bcULL,0x40af6a09e667f3bdULL,0x4052bec333018866ULL,0x4052bec333018867ULL,
    0xc0a0dfe61def1c47ULL,0xc0a0dfe61def1c46ULL,0x40aec3ef1535754bULL,0x40aec3ef1535754cULL,0x40337ca1866b95ceULL,0x40337ca1866b95cfULL,
    0xc0a0dfe61def1c47ULL,0xc0a0dfe61def1c46ULL,0x40aee915d761c9e9ULL,0x40aee915d761c9eaULL,0x403b2e5f9ff717bfULL,0x403b2e5f9ff717c0ULL,
    0xc0a0dfe61def1c47ULL,0xc0a0dfe61def1c46ULL,0x40af0cb9d77e832fULL,0x40af0cb9d77e8330ULL,0x4042597cb8670af3ULL,0x4042597cb8670af4ULL,
    0xc0a0dfe61def1c47ULL,0xc0a0dfe61def1c46ULL,0x40af2e60d35523d4ULL,0x40af2e60d35523d5ULL,0x4047f892ffbb3959ULL,0x4047f892ffbb395aULL,
    0xc0a0dfe61def1c47ULL,0xc0a0dfe61def1c46ULL,0x40af4d9aae3cf7ebULL,0x40af4d9aae3cf7ecULL,0x404e61b85b441c72ULL,0x404e61b85b441c73ULL,
    0xc0a0dfe61def1c47ULL,0xc0a0dfe61def1c46ULL,0x40af6a09e667f3bcULL,0x40af6a09e667f3bdULL,0x4052bec333018866ULL,0x4052bec333018867ULL,
    0xc0967fdd7d3ed05fULL,0xc0967fdd7d3ed05eULL,0x40aec3ef1535754bULL,0x40aec3ef1535754cULL,0x40337ca1866b95ceULL,0x40337ca1866b95cfULL,
    0xc0967fdd7d3ed05fULL,0xc0967fdd7d3ed05eULL,0x40aee915d761c9e9ULL,0x40aee915d761c9eaULL,0x403b2e5f9ff717bfULL,0x403b2e5f9ff717c0ULL,
    0xc0967fdd7d3ed05fULL,0xc0967fdd7d3ed05eULL,0x40af0cb9d77e832fULL,0x40af0cb9d77e8330ULL,0x4042597cb8670af3ULL,0x4042597cb8670af4ULL,
    0xc0967fdd7d3ed05fULL,0xc0967fdd7d3ed05eULL,0x40af2e60d35523d4ULL,0x40af2e60d35523d5ULL,0x4047f892ffbb3959ULL,0x4047f892ffbb395aULL,
    0xc0967fdd7d3ed05fULL,0xc0967fdd7d3ed05eULL,0x40af4d9aae3cf7ebULL,0x40af4d9aae3cf7ecULL,0x404e61b85b441c72ULL,0x404e61b85b441c73ULL,
    0xc0967fdd7d3ed05fULL,0xc0967fdd7d3ed05eULL,0x40af6a09e667f3bcULL,0x40af6a09e667f3bdULL,0x4052bec333018866ULL,0x4052bec333018867ULL,
    0xc0867fdd7d3ed05fULL,0xc0867fdd7d3ed05eULL,0x40aec3ef1535754bULL,0x40aec3ef1535754cULL,0x40337ca1866b95ceULL,0x40337ca1866b95cfULL,
    0xc0867fdd7d3ed05fULL,0xc0867fdd7d3ed05eULL,0x40aee915d761c9e9ULL,0x40aee915d761c9eaULL,0x403b2e5f9ff717bfULL,0x403b2e5f9ff717c0ULL,
    0xc0867fdd7d3ed05fULL,0xc0867fdd7d3ed05eULL,0x40af0cb9d77e832fULL,0x40af0cb9d77e8330ULL,0x4042597cb8670af3ULL,0x4042597cb8670af4ULL,
    0xc0867fdd7d3ed05fULL,0xc0867fdd7d3ed05eULL,0x40af2e60d35523d4ULL,0x40af2e60d35523d5ULL,0x4047f892ffbb3959ULL,0x4047f892ffbb395aULL,
    0xc0867fdd7d3ed05fULL,0xc0867fdd7d3ed05eULL,0x40af4d9aae3cf7ebULL,0x40af4d9aae3cf7ecULL,0x404e61b85b441c72ULL,0x404e61b85b441c73ULL,
    0xc0867fdd7d3ed05fULL,0xc0867fdd7d3ed05eULL,0x40af6a09e667f3bcULL,0x40af6a09e667f3bdULL,0x4052bec333018866ULL,0x4052bec333018867ULL,
    0x0000000000000000ULL,0x0000000000000000ULL,0x40aec3ef1535754bULL,0x40aec3ef1535754cULL,0x40337ca1866b95ceULL,0x40337ca1866b95cfULL,
    0x0000000000000000ULL,0x0000000000000000ULL,0x40aee915d761c9e9ULL,0x40aee915d761c9eaULL,0x403b2e5f9ff717bfULL,0x403b2e5f9ff717c0ULL,
    0x0000000000000000ULL,0x0000000000000000ULL,0x40af0cb9d77e832fULL,0x40af0cb9d77e8330ULL,0x4042597cb8670af3ULL,0x4042597cb8670af4ULL,
    0x0000000000000000ULL,0x0000000000000000ULL,0x40af2e60d35523d4ULL,0x40af2e60d35523d5ULL,0x4047f892ffbb3959ULL,0x4047f892ffbb395aULL,
    0x0000000000000000ULL,0x0000000000000000ULL,0x40af4d9aae3cf7ebULL,0x40af4d9aae3cf7ecULL,0x404e61b85b441c72ULL,0x404e61b85b441c73ULL,
    0x0000000000000000ULL,0x0000000000000000ULL,0x40af6a09e667f3bcULL,0x40af6a09e667f3bdULL,0x4052bec333018866ULL,0x4052bec333018867ULL
};

static const uint64 CP1931[]={
    0x0000000000000000ULL,0x0000000000000000ULL,0x40ae000000000000ULL,0x40ae000000000000ULL,0x0000000000000000ULL,0x0000000000000000ULL,
    0x0000000000000000ULL,0x0000000000000000ULL,0x40ae28365eed39e1ULL,0x40ae28365eed39e2ULL,0x0000000000000000ULL,0x0000000000000000ULL,
    0x0000000000000000ULL,0x0000000000000000ULL,0x40ae506cbdda73c3ULL,0x40ae506cbdda73c4ULL,0x0000000000000000ULL,0x0000000000000000ULL,
    0x0000000000000000ULL,0x0000000000000000ULL,0x40ae781ececd30b4ULL,0x40ae781ececd30b5ULL,0x4017b0042a803897ULL,0x4017b0042a803898ULL,
    0x0000000000000000ULL,0x0000000000000000ULL,0x40ae9ec8530920acULL,0x40ae9ec8530920adULL,0x402795c6d9c027bcULL,0x402795c6d9c027bdULL,
    0x0000000000000000ULL,0x0000000000000000ULL,0x40aec3ef1535754bULL,0x40aec3ef1535754cULL,0x40337ca1866b95ceULL,0x40337ca1866b95cfULL,
    0x40867fdd7d3ed05eULL,0x40867fdd7d3ed05fULL,0x40ae000000000000ULL,0x40ae000000000000ULL,0x0000000000000000ULL,0x0000000000000000ULL,
    0x40867fdd7d3ed05eULL,0x40867fdd7d3ed05fULL,0x40ae28365eed39e1ULL,0x40ae28365eed39e2ULL,0x0000000000000000ULL,0x0000000000000000ULL,
    0x40867fdd7d3ed05eULL,0x40867fdd7d3ed05fULL,0x40ae506cbdda73c3ULL,0x40ae506cbdda73c4ULL,0x0000000000000000ULL,0x0000000000000000ULL,
    0x40867fdd7d3ed05eULL,0x40867fdd7d3ed05fULL,0x40ae781ececd30b4ULL,0x40ae781ececd30b5ULL,0x4017b0042a803897ULL,0x4017b0042a803898ULL,
    0x40867fdd7d3ed05eULL,0x40867fdd7d3ed05fULL,0x40ae9ec8530920acULL,0x40ae9ec8530920adULL,0x402795c6d9c027bcULL,0x402795c6d9c027bdULL,
    0x40867fdd7d3ed05eULL,0x40867fdd7d3ed05fULL,0x40aec3ef1535754bULL,0x40aec3ef1535754cULL,0x40337ca1866b95ceULL,0x40337ca1866b95cfULL,
    0x40967fdd7d3ed05eULL,0x40967fdd7d3ed05fULL,0x40ae000000000000ULL,0x40ae000000000000ULL,0x0000000000000000ULL,0x0000000000000000ULL,
    0x40967fdd7d3ed05eULL,0x40967fdd7d3ed05fULL,0x40ae28365eed39e1ULL,0x40ae28365eed39e2ULL,0x0000000000000000ULL,0x0000000000000000ULL,
    0x40967fdd7d3ed05eULL,0x40967fdd7d3ed05fULL,0x40ae506cbdda73c3ULL,0x40ae506cbdda73c4ULL,0x0000000000000000ULL,0x0000000000000000ULL,
    0x40967fdd7d3ed05eULL,0x40967fdd7d3ed05fULL,0x40ae781ececd30b4ULL,0x40ae781ececd30b5ULL,0x4017b0042a803897ULL,0x4017b0042a803898ULL,
    0x40967fdd7d3ed05eULL,0x40967fdd7d3ed05fULL,0x40ae9ec8530920acULL,0x40ae9ec8530920adULL,0x402795c6d9c027bcULL,0x402795c6d9c027bdULL,
    0x40967fdd7d3ed05eULL,0x40967fdd7d3ed05fULL,0x40aec3ef1535754bULL,0x40aec3ef1535754cULL,0x40337ca1866b95ceULL,0x40337ca1866b95cfULL,
    0x40a0dfe61def1c46ULL,0x40a0dfe61def1c47ULL,0x40ae000000000000ULL,0x40ae000000000000ULL,0x0000000000000000ULL,0x0000000000000000ULL,
    0x40a0dfe61def1c46ULL,0x40a0dfe61def1c47ULL,0x40ae28365eed39e1ULL,0x40ae28365eed39e2ULL,0x0000000000000000ULL,0x0000000000000000ULL,
    0x40a0dfe61def1c46ULL,0x40a0dfe61def1c47ULL,0x40ae506cbdda73c3ULL,0x40ae506cbdda73c4ULL,0x0000000000000000ULL,0x0000000000000000ULL,
    0x40a0dfe61def1c46ULL,0x40a0dfe61def1c47ULL,0x40ae781ececd30b4ULL,0x40ae781ececd30b5ULL,0x4017b0042a803897ULL,0x4017b0042a803898ULL,
    0x40a0dfe61def1c46ULL,0x40a0dfe61def1c47ULL,0x40ae9ec8530920acULL,0x40ae9ec8530920adULL,0x402795c6d9c027bcULL,0x402795c6d9c027bdULL,
    0x40a0dfe61def1c46ULL,0x40a0dfe61def1c47ULL,0x40aec3ef1535754bULL,0x40aec3ef1535754cULL,0x40337ca1866b95ceULL,0x40337ca1866b95cfULL,
    0x40a67fdd7d3ed05eULL,0x40a67fdd7d3ed05fULL,0x40ae000000000000ULL,0x40ae000000000000ULL,0x0000000000000000ULL,0x0000000000000000ULL,
    0x40a67fdd7d3ed05eULL,0x40a67fdd7d3ed05fULL,0x40ae28365eed39e1ULL,0x40ae28365eed39e2ULL,0x0000000000000000ULL,0x0000000000000000ULL,
    0x40a67fdd7d3ed05eULL,0x40a67fdd7d3ed05fULL,0x40ae506cbdda73c3ULL,0x40ae506cbdda73c4ULL,0x0000000000000000ULL,0x0000000000000000ULL,
    0x40a67fdd7d3ed05eULL,0x40a67fdd7d3ed05fULL,0x40ae781ececd30b4ULL,0x40ae781ececd30b5ULL,0x4017b0042a803897ULL,0x4017b0042a803898ULL,
    0x40a67fdd7d3ed05eULL,0x40a67fdd7d3ed05fULL,0x40ae9ec8530920acULL,0x40ae9ec8530920adULL,0x402795c6d9c027bcULL,0x402795c6d9c027bdULL,
    0x40a67fdd7d3ed05eULL,0x40a67fdd7d3ed05fULL,0x40aec3ef1535754bULL,0x40aec3ef1535754cULL,0x40337ca1866b95ceULL,0x40337ca1866b95cfULL,
    0x40ac1fd4dc8e8475ULL,0x40ac1fd4dc8e8476ULL,0x40ae000000000000ULL,0x40ae000000000000ULL,0x0000000000000000ULL,0x0000000000000000ULL,
    0x40ac1fd4dc8e8475ULL,0x40ac1fd4dc8e8476ULL,0x40ae28365eed39e1ULL,0x40ae28365eed39e2ULL,0x0000000000000000ULL,0x0000000000000000ULL,
    0x40ac1fd4dc8e8475ULL,0x40ac1fd4dc8e8476ULL,0x40ae506cbdda73c3ULL,0x40ae506cbdda73c4ULL,0x0000000000000000ULL,0x0000000000000000ULL,
    0x40ac1fd4dc8e8475ULL,0x40ac1fd4dc8e8476ULL,0x40ae781ececd30b4ULL,0x40ae781ececd30b5ULL,0x4017b0042a803897ULL,0x4017b0042a803898ULL,
    0x40ac1fd4dc8e8475ULL,0x40ac1fd4dc8e8476ULL,0x40ae9ec8530920acULL,0x40ae9ec8530920adULL,0x402795c6d9c027bcULL,0x402795c6d9c027bdULL,
    0x40ac1fd4dc8e8475ULL,0x40ac1fd4dc8e8476ULL,0x40aec3ef1535754bULL,0x40aec3ef1535754cULL,0x40337ca1866b95ceULL,0x40337ca1866b95cfULL
};

}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAnalyticCapturedGeneralBox800Test,
    "IAmSpeed.AnalyticWorld.GeneralBox.CapturedGutterBox800",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FAnalyticCapturedGeneralBox800Test::RunTest(const FString&)
{
    using namespace Speed;using namespace Speed::Analytic;using namespace Speed::Analytic::Certified;
    FTensorBoundsTestContext context;if(!TestTrue(TEXT("captured800 full numeric context"),context.Ready))return false;
    const uint32 controls=VectorGetControlRegister();
    for(uint32 flush:{0u,64u,32768u,32832u})
    {
        VectorSetControlRegister((controls&~uint32(0x8040))|flush);
        PreparedExecution::FNativeSolidStaticValues v;v.Shape=PreparedExecution::ENativeSolidQueryShape::Box;v.Horizon=1.f/300.f;
        v.OwnerState.Location=FVector(-.0058453350660885531,3923.9770411524073,37.817226917593175);
        v.OwnerState.Rotation=FQuat(.11494654736557122,-.11365954457161662,.69625748183613645,.69935278519764932);
        v.OwnerState.Velocity=FVector(1.1735750224900199,1261.756030105191,383.40658789379199);
        v.OwnerState.Acceleration=FVector(74.391028133140239,-1137.1767157036011,2346.7556217442993);
        v.OwnerState.AngularVelocity=FVector(5.4955618839678468,.11952141671682387,-.18578000545205836);
        v.OwnerState.AngularAcceleration=FVector(.15179391265740189,-5.0026102464766833,1.5171636516429587);
        v.BodyState=v.OwnerState;v.BodyState.Location=FVector(.048885822468068142,3928.6560879329013,52.153458389330865);
        v.BodyState.Velocity=FVector(3.7563350547630696,1182.960214914624,409.11403748849119);
        v.BodyState.Acceleration=FVector(-15.992526202907115,-1281.0265375481658,1914.4036974965481);
        v.LocalCOM=FVector(-14.008570000000001,0,17.415800000000001);v.LocalOffset=FVector(-5,0,29.510000000000002);v.LocalRotation=FQuat(0,-0.,0,1);
        v.PhysicalHalfExtent=FVector(65.659999999999997,43.5852,15.9472);v.QueryHalfExtent=FVector(65.660003662109375,43.585201263427734,15.947199821472168);
        v.Query.Shape=EQueryShape::Box;v.Query.Start=v.BodyState.Location;v.Query.End=FVector(.061318092248372848,3932.5921719303233,53.527807457259591);
        v.Query.Rotation=v.BodyState.Rotation;v.Query.HalfExtent=v.QueryHalfExtent;v.Query.DomainTolerance=.000001;v.Query.InitialOverlapTolerance=.05;
        TArray<FIntervalTensorSurface> surfaces;
        for(const uint64* cp:{GlobalBoxCaptured800Detail::CP118,GlobalBoxCaptured800Detail::CP726,GlobalBoxCaptured800Detail::CP1118,GlobalBoxCaptured800Detail::CP1931})
        {
            FIntervalTensorSurface surface;surface.DegreeU=surface.DegreeV=5;
            for(int32 k=0;k<36;++k){FIntervalPoint point;for(int32 d=0;d<3;++d)point.Axis[d]={GlobalBoxCaptured800Detail::Value(cp[k*6+d*2]),GlobalBoxCaptured800Detail::Value(cp[k*6+d*2+1]),true};surface.ControlPoints.Add(point);}
            surfaces.Add(MoveTemp(surface));
        }
        FDistanceBudget build{8192};FIndexedIntervalUnionPtr scene;if(!TestTrue(TEXT("four original neighboring CP providers"),FIndexedIntervalUnion::Build(MakeArrayView(surfaces),build,scene)))return false;
        FDistanceBudget budget{26439};FGroupedBoxVolumeBounds volume;FReferenceBoxVolumeStrictSeparationPtr proof;FReferenceBoundsCandidateSetPtr partition;FGroupedBoxSeparationObservation observed;
        if(!TestTrue(TEXT("actual800 original query/horizon retains all14groups84slabs"),BuildGeneralNativeBoxGroupedVolume(v,budget,volume)&&volume.Groups.Num()==14&&volume.Slabs.Num()==84))return false;
        const bool known=FReferenceBoundsCandidateSet::Select(scene,volume.BroadHull,budget,partition)&&FReferenceBoxVolumeStrictSeparation::TryProve(partition,volume,budget,proof,&observed);
        AddInfo(FString::Printf(TEXT("captured_Box800 flush=%u Known=%u Used=%llu Remaining=%d Cells=%llu TestedGroups=%llu InheritedSkips=%llu ActiveMask=%u pure_four_neighbor_providers_only=1 NOT_fullWorld_budget_equivalence=1"),flush,uint32(known),budget.Used,budget.Remaining,observed.VisitedCells,observed.TestedGroups,observed.InheritedGroupSkips,observed.ActiveGroupMask));
        TestTrue(TEXT("captured800 mathematical neighboring volumes close under unchanged nominal budget"),known&&proof&&proof->Matches(scene,volume));
        TestTrue(TEXT("same account no refill"),budget.Used+uint64(budget.Remaining)==26439);
    }
    return TestTrue(TEXT("captured800 numeric context restored"),context.Restore.Restore())&&!HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAnalyticGlobalPieceContractionTest,
    "IAmSpeed.AnalyticWorld.GeneralBox.GlobalPieceContraction",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FAnalyticGlobalPieceContractionTest::RunTest(const FString&)
{
    using namespace Speed;using namespace Speed::Analytic;using namespace Speed::Analytic::Certified;
    FTensorBoundsTestContext context;if(!TestTrue(TEXT("RNE numeric fixture"),context.Ready))return false;
    const uint32 original=VectorGetControlRegister();
    for(uint32 flush:{0u,64u,32768u,32832u})
    {
        VectorSetControlRegister((original&~uint32(0x8040))|flush);
        FGeneralBoxMotionInput input;input.Horizon=1.f/300.f;
        input.Owner.Location=FVector(13,-9,70);input.Owner.Rotation=FQuat(FVector::UpVector,.7);
        input.Owner.Velocity=FVector(12,-3,2);input.Owner.Acceleration=FVector(-7,4,-650);
        input.Owner.AngularVelocity=FVector(.4,-.25,.7);input.Owner.AngularAcceleration=FVector(.05,.1,-.2);
        input.LocalCOM=FVector(2,-3,1);input.LocalOffset=FVector(7,1,-2);input.LocalRotation=FQuat(FVector::RightVector,-.55);
        input.PhysicalHalfExtent=FVector(65.66,43.5852,15.9472);
        input.QueryHalfExtent=FVector(float(65.66),float(43.5852),float(15.9472));
        FDistanceBudget budget{8192};FGeneralBoxMotionBounds bounds;
        if(!TestTrue(TEXT("closed original-time contraction is available"),
            NativeGroupedBoxDetail::ContractedGlobalMotionPiece(input,0.f,input.Horizon,budget,bounds)&&bounds.Corners.Num()==32))return false;
        // Independent real consumed poses, including all leaf boundaries and
        // interiors. This observes whole-Box role retention, not a clearance.
        for(int32 sample=0;sample<=16;++sample)
        {
            const float seconds=input.Horizon*(float(sample)/16.f);const auto actual=input.Owner.Integrate(seconds);
            const FVector center=(actual.Location-actual.Rotation.RotateVector(input.LocalCOM))+actual.Rotation.RotateVector(input.LocalOffset);
            const FQuat rotation=actual.Rotation*input.LocalRotation;
            TestTrue(TEXT("actual consumed center survives indexed hull"),GroupedBoxTestDetail::Contains(bounds.NativeCenter,center));
            for(int32 kind=0;kind<2;++kind)for(int32 corner=0;corner<8;++corner)
            {
                const auto extent=kind?input.QueryHalfExtent:input.PhysicalHalfExtent;FVector local;
                for(int32 d=0;d<3;++d)local[d]=(corner&(1<<d))?extent[d]:-extent[d];
                TestTrue(TEXT("each real consumed corner stays in its OWN role"),
                    GroupedBoxTestDetail::Contains(bounds.Corners[kind*8+corner],center+rotation.RotateVector(local)));
            }
        }
        FDistanceBudget singletonBudget{8192};FGeneralBoxMotionBounds singleton;
        TestTrue(TEXT("singleton original-time interval is retained, without rebasing"),
            NativeGroupedBoxDetail::ContractedGlobalMotionPiece(input,input.Horizon,input.Horizon,singletonBudget,singleton)&&singleton.Corners.Num()==32);
        auto rejected=bounds;FDistanceBudget denied{3,5,1};
        TestFalse(TEXT("same global floor stops contraction before unpaid model work"),
            NativeGroupedBoxDetail::ContractedGlobalMotionPiece(input,0.f,input.Horizon,denied,rejected));
        TestTrue(TEXT("budget refusal preserves floor and clears opaque output"),denied.Remaining==1&&denied.Used==7&&rejected.Corners.IsEmpty()&&!Detail::ValidPoint(rejected.Hull));
        FDistanceBudget invalidBudget{8192};rejected=bounds;
        TestFalse(TEXT("reversed global interval is rejected"),
            NativeGroupedBoxDetail::ContractedGlobalMotionPiece(input,input.Horizon,0.f,invalidBudget,rejected));
        TestTrue(TEXT("invalid interval cannot retain old output"),rejected.Corners.IsEmpty());
    }
    VectorSetControlRegister(original);return true;
}


IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAnalyticNativePrimitiveCoarseVolumeTest,
    "IAmSpeed.AnalyticWorld.GeneralBox.NativePrimitiveCoarseVolume",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FAnalyticNativePrimitiveCoarseVolumeTest::RunTest(const FString&)
{
    using namespace Speed;using namespace Speed::Analytic;using namespace Speed::Analytic::Certified;
    FTensorBoundsTestContext context;if(!TestTrue(TEXT("complete RNE fixture"),context.Ready))return false;
    const uint32 original=VectorGetControlRegister();
    for(uint32 flush:{0u,64u,32768u,32832u})
    {
        VectorSetControlRegister((original&~uint32(0x8040))|flush);
        FGeneralBoxMotionInput input;input.Owner.Location=FVector(0,0,20);
        input.Owner.Rotation=FQuat::Identity;input.Owner.Velocity=FVector(10,-3,1);
        input.Owner.Acceleration=FVector(2,1,-650);input.Owner.AngularVelocity=FVector(.1,-.05,.2);
        input.Owner.AngularAcceleration=FVector(.01,.02,-.03);input.Horizon=1.f/300.f;
        input.LocalRotation=FQuat::Identity;input.PhysicalHalfExtent=FVector(1,2,3);
        input.QueryHalfExtent=FVector(1.01,2.01,3.01);
        FGeneralBoxMotionBounds whole;
        if(!TestTrue(TEXT("one original whole-time model"),BuildGeneralBoxMotionBounds(input,whole)&&whole.Corners.Num()==32))return false;
        FIntervalTensorSurface far;far.ControlPoints.Add(Detail::Point(FVector3d(1000,1000,1000)));
        const auto scene=GroupedBoxTestDetail::Scene(far);if(!TestTrue(TEXT("far scene built"),scene.IsValid()))return false;
        const int32 fineReserve=8192,allowance=scene->Num()+1024;
        FDistanceBudget budget{fineReserve+allowance};FGroupedBoxVolumeBounds coarse;
        FReferenceBoundsCandidateSetPtr partition;FReferenceBoxVolumeStrictSeparationPtr proof;
        {
            FScopedDistanceBudgetFloor reserve(budget,fineReserve);
            if(!TestTrue(TEXT("original fine account protected"),reserve.IsValid()&&budget.Consume()))return false;
            if(!TestTrue(TEXT("coarse exact volume uses four complete shape families"),
                NativeGroupedBoxDetail::CoarsePrimitiveFromWhole(whole,input.Owner.Rotation*input.LocalRotation,1.e-6,.05,budget,coarse)&&
                coarse.Groups.Num()==4&&coarse.Axes.Num()==6))return false;
            if(!TestTrue(TEXT("coarse success is an actual complete-union proof"),
                FReferenceBoundsCandidateSet::Select(scene,coarse.BroadHull,budget,partition)&&
                FReferenceBoxVolumeStrictSeparation::TryProve(partition,coarse,budget,proof)&&proof&&proof->Matches(scene,coarse)))return false;
        }
        TestTrue(TEXT("coarse consumed real work without touching fine reserve"),budget.Used>0&&budget.Remaining>=fineReserve);
        TestEqual(TEXT("reservation release never refunds consumed work"),budget.Used+uint64(budget.Remaining),uint64(fineReserve+allowance));
        TestEqual(TEXT("reservation restores original floor only"),budget.ConsumptionFloor,0);
        for(float seconds:{0.f,input.Horizon*.25f,input.Horizon*.5f,input.Horizon*.75f,input.Horizon})
        {
            const auto actual=input.Owner.Integrate(seconds);
            FWorldQuery pose;pose.Shape=EQueryShape::Box;pose.Start=pose.End=
                (actual.Location-actual.Rotation.RotateVector(input.LocalCOM))+actual.Rotation.RotateVector(input.LocalOffset);
            pose.Rotation=actual.Rotation*input.LocalRotation;
            if(!TestTrue(TEXT("actual endpoint still passes existing whole-group consumer"),
                GroupedBoxProofContainsEndpoint(proof,pose,input.PhysicalHalfExtent,input.QueryHalfExtent)))return false;
        }
        FGroupedBoxVolumeBounds directFine,sharedFine;FDistanceBudget directBudget{fineReserve},sharedBudget{fineReserve};
        if(!TestTrue(TEXT("original and shared-model fine constructors complete"),
            BuildGeneralBoxPrimitiveGroupedVolume(input,input.Owner.Rotation*input.LocalRotation,1.e-6,.05,directBudget,directFine)&&
            NativeGroupedBoxDetail::RefinedPrimitiveFromWhole(input,whole,input.Owner.Rotation*input.LocalRotation,1.e-6,.05,sharedBudget,sharedFine)))return false;
        TestTrue(TEXT("shared full model changes no fine volume bits"),GroupedBoxDetail::Same(directFine,sharedFine));
        TestEqual(TEXT("shared full model changes no original fine debit"),directBudget.Used,sharedBudget.Used);
        FIntervalTensorSurface inside;inside.ControlPoints.Add(Detail::Point(FVector3d(0,0,20)));
        const auto crossing=GroupedBoxTestDetail::Scene(inside);FReferenceBoxVolumeStrictSeparationPtr denied;
        FDistanceBudget deniedBudget{fineReserve+allowance};
        {
            FScopedDistanceBudgetFloor reserve(deniedBudget,fineReserve);
            TestTrue(TEXT("crossing scene remains a valid candidate partition"),FReferenceBoundsCandidateSet::Select(crossing,coarse.BroadHull,deniedBudget,partition));
            TestFalse(TEXT("coarse family crossing a provider cannot grant clearance"),FReferenceBoxVolumeStrictSeparation::TryProve(partition,coarse,deniedBudget,denied));
        }
        TestTrue(TEXT("coarse rejection preserves complete fine reserve and clears proof"),!denied&&deniedBudget.Used>0&&deniedBudget.Remaining>=fineReserve);
        const uint64 spent=deniedBudget.Used;const int32 fineFloor=deniedBudget.Remaining-fineReserve;
        {
            FScopedDistanceBudgetFloor reserveUnusedCoarse(deniedBudget,fineFloor);FGroupedBoxVolumeBounds afterRejection;
            TestTrue(TEXT("fine still executes after coarse rejection on same account"),
                NativeGroupedBoxDetail::RefinedPrimitiveFromWhole(input,whole,input.Owner.Rotation*input.LocalRotation,1.e-6,.05,deniedBudget,afterRejection)&&
                GroupedBoxDetail::Same(afterRejection,directFine));
        }
        TestTrue(TEXT("coarse and fine debits both retained"),deniedBudget.Used>spent&&deniedBudget.Used-spent==sharedBudget.Used);
        TestEqual(TEXT("whole account never reset or refunded"),deniedBudget.Used+uint64(deniedBudget.Remaining),uint64(fineReserve+allowance));
    }
    return TestTrue(TEXT("numeric controls restored"),context.Restore.Restore())&&!HasAnyErrors();
}
