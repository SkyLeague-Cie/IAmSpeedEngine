#include "TensorSphereContactBounds.h"
#include "TensorPointRootBounds.h"
#include "IAmSpeed/World/Collision/NativeFixedBoxReferenceMotionValues.h"
#include "Misc/ScopeExit.h"
#include "Misc/ScopedCVar.h"
#include "IAmSpeed/World/Collision/NativeRetainedBoxCornerReferenceValues.h"
#include "Engine/EngineTypes.h"
#include "IAmSpeed/World/Collision/NativeRetainedBoxPreCellClearanceValues.h"
#include "IAmSpeed/World/Collision/NativeRetainedBoxRootDomainClearanceValues.h"
#include "IAmSpeed/World/Collision/NativeRetainedBoxReferenceFirstEntryValues.h"
#include "IAmSpeed/World/Collision/NativeFixedBoxAtFloatTimeValues.h"
#include "IAmSpeed/World/Collision/NativeFixedBoxFloatRangeGeometryValues.h"
#include "IAmSpeed/World/Collision/NativeRetainedBoxReferenceSearch.h"
#include "IAmSpeed/World/Collision/NativeRetainedBoxAtFloatInteriorValues.h"
#include "IAmSpeed/World/Collision/NativeRetainedBoxFloatIntersectionBracketValues.h"
#include "IAmSpeed/World/Collision/NativeRetainedBoxAutomaticFloatBracket.h"
#include "IAmSpeed/World/Collision/NativeRetainedBoxFloatProviderDomainValues.h"
#include "IAmSpeed/World/Collision/NativeRetainedBoxChartSampleValues.h"
#include "IAmSpeed/World/Collision/NativeRetainedBoxResponsePointSampleValues.h"
namespace
{
// Generic mathematical STRUCTURE fixture, not an authenticated stadium
// packet/source, GT request or gameplay pose. AuthorityEligible remains false.
class FContactUnitRetainedProducer:public Speed::Analytic::Prepared::FReferenceTransportProducer
{
public:
    static bool Build(const Speed::Analytic::Certified::FIntervalTensorSurface& surface,
        Speed::Analytic::Certified::FDistanceBudget& budget,Speed::FPreparedGeometrySelectionPtr& out,bool interiorBlocker=false,double extraPatchZ=0)
    {
        namespace C=Speed::Analytic::Certified;namespace P=Speed::Analytic::Prepared;
        TArray<C::FIntervalTensorSurface> surfaces;surfaces.Add(surface);
        if(interiorBlocker)
        {
            C::FIntervalTensorSurface patch;patch.DegreeU=patch.DegreeV=1;
            for(int32 u=0;u<2;++u)for(int32 v=0;v<2;++v)
                patch.ControlPoints.Add(C::Detail::Point(FVector3d(u?-.25:-.375,v?-.375:-.5,extraPatchZ)));
            surfaces.Add(MoveTemp(patch));
        }
        C::FIndexedIntervalUnionPtr net;
        if(!C::FIndexedIntervalUnion::Build(surfaces,budget,net))return false;
        P::FOwner owner;owner.SourceId=1;owner.SurfaceId=2;owner.FeatureId=3;owner.PrimitiveId=4;
        owner.QueryEnabled=true;owner.CompactOptIn=true;owner.BlockingChannels=1;
        FSpeedAnalyticAuthoredSourceRecord source;source.Kind=ESpeedAnalyticSourceKind::AuthoredAnalytic;source.SourceId=1;
        source.RecipeSchema=TEXT("unit.coefficient.structure.only");source.RecipeSha256=FString::ChrN(64,TCHAR('1'));
        source.PayloadSha256=FString::ChrN(64,TCHAR('2'));source.SourceComponentPath=TEXT("Source");source.ContactComponentPath=TEXT("Contact");
        source.CollisionEnabled=uint8(ECollisionEnabled::QueryOnly);source.bQueryCollisionEnabled=true;source.BlockingChannels=1;
        TArray<P::FOwner> owners;owners.Add(owner);
        if(interiorBlocker){auto other=owner;other.PrimitiveId=5;other.FeatureId=6;other.SurfaceId=7;owners.Add(other);}
        P::FScenePtr scene;FString reason;
        if(!P::FScene::Build(net,owners,MakeArrayView(&source,1),{},1,budget,scene,reason))return false;
        P::FReferenceTransportData data;data.CanonicalUnion=net;data.NominalScene=scene;
        P::FReferenceFace face;face.OpaqueId=TEXT("unit.face");face.CanonicalRecordIdentity=TEXT("unit.net");
        face.BoundRecordIdentity=TEXT("unit.bound");face.DomainAndChart.Add(TEXT("provider square [0,1]^2"));data.Faces.Add(face);
        data.ProviderFaceIndices.Add(0);P::FReferenceSphereResponseContract unused;unused.SourceFaceIndex=0;
        data.SphereResponseContracts.Add(unused);
        if(interiorBlocker)
        {
            face.OpaqueId=TEXT("unit.interior.blocker");face.CanonicalRecordIdentity=TEXT("unit.interior.net");
            data.Faces.Add(face);data.ProviderFaceIndices.Add(1);unused.SourceFaceIndex=1;data.SphereResponseContracts.Add(unused);
        }
        data.ProofIdentities.Add(TEXT("unit.structure.not.certificate"));
        data.Packet.Add(1);data.Model=TEXT("unit.structure");data.ExternalPacketIdentity=TEXT("unit.not.authenticated");
        P::FReferenceTransportPtr retained;
        return SealReference(MoveTemp(data),budget,retained,reason)&&
            Speed::FPreparedGeometrySelection::TryCreate(Speed::EPreparedGeometryModel::RetainedReference,scene,retained,out,reason);
    }
};
Speed::Analytic::FTensorBezierSurface ContactUnitBowl()
{
    Speed::Analytic::FTensorBezierSurface p;p.DegreeU=p.DegreeV=2;
    const double linear[3]={-1,0,1},quadratic[3]={1,-1,1};
    for(int32 i=0;i<3;++i)for(int32 j=0;j<3;++j)
        p.ControlPoints.Add(FVector3d(linear[i],linear[j],0.25*(quadratic[i]+quadratic[j])));
    return p; //S(x,y)=(x,y,(x*x+y*y)/4), x,y in[-1,1].
}
}

namespace
{
// Compare fields, never padding or pointers. This includes passive failure and
// partial-budget diagnostics: warming geometry must not alter any of them.
TArray<uint64> ContactMemo71Signature(const Speed::Analytic::Certified::FTensorSphereContactEnclosure& r)
{
    TArray<uint64> bits;
    const auto value=[&](uint64 x){bits.Add(x);};
    const auto real=[&](double x){value(AnalyticCertified::Bits(x));};
    const auto interval=[&](const AnalyticCertified::Interval& x){real(x.Lower);real(x.Upper);value(x.Known);};
    const auto point=[&](const Speed::Analytic::Certified::FIntervalPoint& p){for(const auto& x:p.Axis)interval(x);};
    value(uint64(r.Failure));for(const TCHAR* p=r.WorkSite;*p;++p)value(uint64(*p));value(0);
    value(r.HasCallAccounting);value(r.CallInitialUsed);value(r.TimeWorkUsed);value(r.SpatialVisitedCells);
    value(r.SpatialSplitsU);value(r.SpatialSplitsV);value(r.TemporalWidthRefinements);value(r.ConsumerWidthRefinements);
    value(r.TimeFailure);value(r.TimeDyadicSite);value(r.TimeDyadicDepth);value(r.TimeAnchorRefinements);
    real(r.TimeDyadicA);real(r.TimeDyadicB);value(r.HasWidthDiagnostic);value(r.WidthRefinement);value(r.WidthSurface);
    value(r.WidthDepthU);value(r.WidthDepthV);real(r.WidthTimeLower);real(r.WidthTimeUpper);
    real(r.WidthPointTarget);real(r.WidthNormalTarget);real(r.WidthMinU);real(r.WidthMaxU);real(r.WidthMinV);real(r.WidthMaxV);
    point(r.WidthPointUnion);point(r.WidthNormalUnion);point(r.WidthLastDu);point(r.WidthLastDv);interval(r.WidthLastGu);interval(r.WidthLastGv);
    value(uint64(r.Status));real(r.LowerTime);real(r.UpperTime);point(r.SurfacePoint);point(r.RadialUnitNormal);value(r.CollisionRadiusBits);
    value(r.Leaves.Num());for(const auto& leaf:r.Leaves)
    {
        value(leaf.SurfaceIndex);real(leaf.MinU);real(leaf.MaxU);real(leaf.MinV);real(leaf.MaxV);
        point(leaf.PointHull);point(leaf.RadialNormalHull);point(leaf.FixedParameterPoint);real(leaf.FixedParameter.X);real(leaf.FixedParameter.Y);
    }
    return bits;
}
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAnalyticContactGeometryMemo71Test,
    "IAmSpeed.AnalyticWorld.TensorContact.GeometryMemoEquivalence",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FAnalyticContactGeometryMemo71Test::RunTest(const FString&)
{
    FScopedCVar<int32> oldDirectory(TEXT("p.IAmSpeed.AnalyticWorld.ContactCellRootLru"),0,FName(TEXT("GeometryMemoLegacyDirectory")));
    using namespace Speed::Analytic;using namespace Speed::Analytic::Certified;
    using Memo=ContactDetail::FContactCellGeometryMemo;using Net=Memo::ENet;
    FTensorBoundsTestContext context;if(!TestTrue(TEXT("RNE fixture"),context.Ready))return false;
    const uint32 original=VectorGetControlRegister();ON_SCOPE_EXIT{VectorSetControlRegister(original);};
    const auto samePoint=[](const FIntervalPoint& a,const FIntervalPoint& b)
    {
        for(int32 d=0;d<3;++d)if(a.Axis[d].Known!=b.Axis[d].Known||
            AnalyticCertified::Bits(a.Axis[d].Lower)!=AnalyticCertified::Bits(b.Axis[d].Lower)||
            AnalyticCertified::Bits(a.Axis[d].Upper)!=AnalyticCertified::Bits(b.Axis[d].Upper))return false;
        return true;
    };
    const auto sameNode=[&](const Detail::FNode& a,const Detail::FNode& b)
    {
        if(a.DegreeU!=b.DegreeU||a.DegreeV!=b.DegreeV||a.DepthU!=b.DepthU||a.DepthV!=b.DepthV||a.Net.Num()!=b.Net.Num()||
            AnalyticCertified::Bits(a.MinU)!=AnalyticCertified::Bits(b.MinU)||AnalyticCertified::Bits(a.MaxU)!=AnalyticCertified::Bits(b.MaxU)||
            AnalyticCertified::Bits(a.MinV)!=AnalyticCertified::Bits(b.MinV)||AnalyticCertified::Bits(a.MaxV)!=AnalyticCertified::Bits(b.MaxV))return false;
        for(int32 i=0;i<a.Net.Num();++i)if(!samePoint(a.Net[i],b.Net[i]))return false;return true;
    };
    for(uint32 flush:{0u,64u,32768u,32832u})
    {
        VectorSetControlRegister((original&~uint32(0x807f))|flush|0x3f);
        const uint64 control=RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush).ControlFingerprint();
        // Nets, midpoint, hulls and ordered child paths, including maximum degree.
        for(int32 degree:{0,1,5,16})
        {
            Detail::FNode p,du,dv;p.DegreeU=p.DegreeV=degree;
            for(int32 u=0;u<=degree;++u)for(int32 v=0;v<=degree;++v)
                p.Net.Add(Detail::Point(FVector3d(.1*u,.3*v,.01*(u*u+v*v))));
            if(!TestTrue(TEXT("original derivative nets"),ContactDetail::OriginalDerivativeNet(p,true,du)&&ContactDetail::OriginalDerivativeNet(p,false,dv)))return false;
            Memo memo,foreign;Memo::FCallScope scope(&memo,control),other(&foreign,control);
            auto root=scope.FindRoot(p,du,dv);if(!TestTrue(TEXT("owned root"),bool(root)))return false;
            TestTrue(TEXT("same full coefficient identity reuses root"),scope.FindRoot(p,du,dv)==root);
            auto changed=p;changed.Net[0].Axis[0]=Interval::Exact(-1);
            TestTrue(TEXT("changed coefficient identity cannot reuse root"),scope.FindRoot(changed,du,dv)!=root);
            FIntervalPoint expected,actual;ContactDetail::NetHull(p,expected);
            TestTrue(TEXT("cold hull equals original"),scope.Hull(root,Net::Position,root->Get(Net::Position),actual)&&samePoint(actual,expected));
            const uint64 hits=memo.GetCountersForInspection().Hits;
            TestTrue(TEXT("warm hull equals original"),scope.Hull(root,Net::Position,root->Get(Net::Position),actual)&&samePoint(actual,expected)&&memo.GetCountersForInspection().Hits==hits+1);
            TestTrue(TEXT("foreign memo cannot publish or hit another owner's cell"),
                other.Hull(root,Net::Position,root->Get(Net::Position),actual)&&samePoint(actual,expected)&&foreign.GetCountersForInspection().Builds==0&&foreign.GetCountersForInspection().Hits==0);
            TestTrue(TEXT("midpoint exact bits"),samePoint(scope.Midpoint(root,root->Get(Net::Position)),Detail::Midpoint(p))&&
                samePoint(scope.Midpoint(root,root->Get(Net::Position)),Detail::Midpoint(p)));
            TestTrue(TEXT("split direction exact"),scope.Direction(root,root->Get(Net::Position))==Detail::ChooseTensorDistanceSplitU(p));
            const auto split=[&](const Memo::FCellPtr& cell,bool alongU)
            {
                Detail::FNode lp,ld,lv,rp,rd,rv;
                const bool built=Detail::Split(cell->Get(Net::Position),alongU,lp,rp)&&Detail::Split(cell->Get(Net::Du),alongU,ld,rd)&&Detail::Split(cell->Get(Net::Dv),alongU,lv,rv);
                if(!built)return Memo::FCellPtr{};
                const auto oldLp=lp,oldLd=ld,oldLv=lv,oldRp=rp,oldRd=rd,oldRv=rv;
                Memo::FCellPtr left,right;
                if(!TestTrue(TEXT("controlled old split construction"),scope.SplitChildren(cell,alongU,
                    cell->Get(Net::Position),cell->Get(Net::Du),cell->Get(Net::Dv),lp,ld,lv,rp,rd,rv,left,right)))return Memo::FCellPtr{};
                TestTrue(TEXT("published exact three split nets"),left&&right&&sameNode(left->Get(Net::Position),oldLp)&&sameNode(left->Get(Net::Du),oldLd)&&sameNode(left->Get(Net::Dv),oldLv)&&
                    sameNode(right->Get(Net::Position),oldRp)&&sameNode(right->Get(Net::Du),oldRd)&&sameNode(right->Get(Net::Dv),oldRv));
                Memo::FCellPtr warmLeft,warmRight;
                TestTrue(TEXT("warm ordered children are owned stable handles"),scope.Children(cell,alongU,warmLeft,warmRight)&&warmLeft==left&&warmRight==right);
                TestFalse(TEXT("opposite split is not interchangeable"),scope.Children(cell,!alongU,warmLeft,warmRight));
                return left;
            };
            auto u=split(root,true);auto uv=u?split(u,false):Memo::FCellPtr{};
            auto v=split(root,false);auto vu=v?split(v,true):Memo::FCellPtr{};
            TestTrue(TEXT("U then V and V then U never merge by UV rectangle alone"),uv&&vu&&uv!=vu&&uv->Get(Net::Position).MinU==vu->Get(Net::Position).MinU&&uv->Get(Net::Position).MaxU==vu->Get(Net::Position).MaxU&&uv->Get(Net::Position).MinV==vu->Get(Net::Position).MinV&&uv->Get(Net::Position).MaxV==vu->Get(Net::Position).MaxV);
            VectorSetControlRegister((original&~uint32(0x807f))|flush);
            Memo::FCellPtr missingLeft,missingRight;const uint32 clean=VectorGetControlRegister();
            TestFalse(TEXT("cleared sticky flags decline saved children"),scope.Children(root,false,missingLeft,missingRight));
            TestTrue(TEXT("declined children never write FENV"),VectorGetControlRegister()==clean);
            VectorSetControlRegister((original&~uint32(0x807f))|flush|0x3f);
            auto wrongControl=control^(uint64(1)<<40);Memo::FCallScope wrong(&memo,wrongControl);
            TestFalse(TEXT("full scalar fingerprint mismatch disables memo"),bool(wrong.FindRoot(p,du,dv)));
        }
        // Real contact oracle: same full outputs, leaves, rejection site, debits,
        // callbacks and final FP flags with cold/warm geometry and changed paths.
        TArray<FTensorBezierSurface> nominal{ContactUnitBowl()};TArray<FIntervalTensorSurface> surfaces;
        if(!TestTrue(TEXT("promote polynomial fixture"),Detail::PromoteTensors(nominal,surfaces)))return false;
        Memo memo;
        for(uint32 initialFlags:{0u,63u})for(int32 limit:{0,1,32,512,131072})for(int32 query:{0,1,2})
        {
            FIntervalBezierCenterPath path;const double x=query==1?.125:0.;
            if(!TestTrue(TEXT("fixed path"),CenterDetail::Linear(FVector3d(x,0,1),FVector3d(x,0,-1),path)))return false;
            int32 referenceCalls=0;const auto referenceCallback=[&](const FTensorSphereContactEnclosure&)
                {++referenceCalls;return query==2&&referenceCalls==1?EContactEnclosureDecision::Refine:EContactEnclosureDecision::Accept;};
            FContactEnclosureFilter referenceFilter=referenceCallback;
            VectorSetControlRegister((original&~uint32(0x807f))|flush|initialFlags);
            FDistanceBudget oldWork{limit,7,0};const auto old=BoundTensorSphereFirstContact(surfaces,path,.25,oldWork,1./65536.,1.e-7,256,.01,.05,24,&referenceFilter,nullptr);
            const auto oldBits=ContactMemo71Signature(old);const uint32 oldFlags=VectorGetControlRegister();
            if(query==2&&limit==131072)TestTrue(TEXT("nonvacuous original consumer refinement then second callback"),
                referenceCalls>=2&&old.ConsumerWidthRefinements>0&&old.Status==ETensorSphereContactStatus::FirstContactEnclosed);
            for(int32 repeat=0;repeat<2;++repeat)
            {
                int32 calls=0;const auto callback=[&](const FTensorSphereContactEnclosure&)
                    {++calls;return query==2&&calls==1?EContactEnclosureDecision::Refine:EContactEnclosureDecision::Accept;};
                FContactEnclosureFilter filter=callback;
                VectorSetControlRegister((original&~uint32(0x807f))|flush|initialFlags);
                FDistanceBudget work{limit,7,0};const auto result=BoundTensorSphereFirstContact(surfaces,path,.25,work,1./65536.,1.e-7,256,.01,.05,24,&filter,&memo);
                const uint32 flags=VectorGetControlRegister();
                TestTrue(TEXT("full contact oracle fields exact cold/warm/changed path/refine/budget"),ContactMemo71Signature(result)==oldBits&&work.Remaining==oldWork.Remaining&&work.Used==oldWork.Used&&work.ConsumptionFloor==oldWork.ConsumptionFloor&&calls==referenceCalls&&flags==oldFlags);
            }
        }
        TestTrue(TEXT("oracle exercises actual geometry hits"),memo.GetCountersForInspection().ChildrenHits>0&&memo.GetCountersForInspection().HullHits>0);
        // Directory eviction must not discount pinned geometry or invalidate it.
        Memo pinned;TArray<Memo::FCellPtr> handles;Detail::FNode p,du,dv;p.Net.Add(Detail::Point(FVector3d::ZeroVector));
        ContactDetail::OriginalDerivativeNet(p,true,du);ContactDetail::OriginalDerivativeNet(p,false,dv);
        {
            Memo::FCallScope scope(&pinned,control);
            for(int32 k=0;k<65;++k){p.Net[0]=Detail::Point(FVector3d(k,0,0));auto cell=scope.FindRoot(p,du,dv);if(cell)handles.Add(cell);else TestTrue(TEXT("bounded root quota"),k==64);}
            {
                Memo::FCallScope nested(&pinned,control);
                TestTrue(TEXT("nested scope cannot evict an active outer call"),pinned.GetCountersForInspection().GeometryEvictions==0&&pinned.GetLiveCellsForInspection()==64);
            }
        }
        TestTrue(TEXT("64 roots retained"),handles.Num()==64&&pinned.GetLiveCellsForInspection()==64);
        {
            Memo::FCallScope scope(&pinned,control);
            TestTrue(TEXT("eviction retains pinned live accounting"),pinned.GetCountersForInspection().GeometryEvictions==1&&pinned.GetLiveCellsForInspection()==64);
            TestTrue(TEXT("pinned value remains alive"),samePoint(handles[0]->Get(Net::Position).Net[0],Detail::Point(FVector3d::ZeroVector)));
        }
        handles.Empty();TestTrue(TEXT("last pinned handles release live accounting"),pinned.GetLiveCellsForInspection()==0&&pinned.GetLivePointsForInspection()==0);
    }
    return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAnalyticTensorCurvedContactTest,
 "IAmSpeed.AnalyticWorld.TensorContact.CurvedPointAndNormal",
 EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FAnalyticTensorCurvedContactTest::RunTest(const FString&)
{
    FTensorBoundsTestContext context;if(!TestTrue(TEXT("RNE fixture established"),context.Ready))return false;
    using namespace Speed::Analytic;using namespace Speed::Analytic::Certified;
    // Pure point/tensor root fixtures in this EXISTING contact unit. No
    // Box/Sphere event, gameplay pose or native consumer is constructed.
    const uint32 rootControl=VectorGetControlRegister();
    {
        ON_SCOPE_EXIT{VectorSetControlRegister(rootControl);};
        for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})
        {
            VectorSetControlRegister((rootControl&~uint32(0x8040))|flush);
            FIntervalTensorSurface curve;curve.DegreeU=2;curve.DegreeV=1;
            const double x[]={-.5,0,.5},z[]={.25,-.25,.25};
            for(int32 i=0;i<3;++i)for(int32 j=0;j<2;++j)
                curve.ControlPoints.Add(Detail::Point(FVector3d(x[i],j?.5:-.5,z[i])));
            FIntervalBezierCenterPath path;FTensorPointRootBoundsPtr root;FDistanceBudget rootWork{4096};
            if(!TestTrue(TEXT("reference point path is independent of rounded native Integrate/Lerp"),
                CenterDetail::Linear(FVector3d(0,0,1),FVector3d(0,0,-1),path)))return false;
            if(!TestTrue(TEXT("actual curved point/tensor intersection has a unique local root for fixed coefficients"),
                FTensorPointRootBounds::TryProve(curve,path,rootWork,root)&&root))return false;
            TestTrue(TEXT("root retains exact coefficient input identity and strict contraction"),
                root->MatchesInputs(curve,path)&&root->GetContractionUpperBound()<1);
            for(int32 d=0;d<3;++d)
                TestTrue(TEXT("u=v=tau=1/2 lies strictly in the local root enclosure"),
                    root->GetLocalRootBounds().Axis[d].Lower<=.5&&root->GetLocalRootBounds().Axis[d].Upper>=.5&&
                    root->GetLocalRootBounds().Axis[d].Lower>0&&root->GetLocalRootBounds().Axis[d].Upper<1);
            auto changed=curve;changed.ControlPoints[0].Axis[0]=Interval::Exact(-.25);
            TestFalse(TEXT("different coefficient tensor cannot reuse a root"),root->MatchesInputs(changed,path));
            if(!TestTrue(TEXT("sole output may lend its own tensor/path across reset"),
                FTensorPointRootBounds::TryProve(root->GetTensorForInspection(),root->GetPointPathForInspection(),rootWork,root)&&root))return false;
            auto family=curve;
            for(auto& cp:family.ControlPoints)
                cp.Axis[2]={cp.Axis[2].Lower-1./1048576.,cp.Axis[2].Upper+1./1048576.,true};
            if(!TestTrue(TEXT("uniform root existence holds for each fixed polynomial in a coefficient family"),
                FTensorPointRootBounds::TryProve(family,path,rootWork,root)&&root))return false;
            TestTrue(TEXT("different constant vertical shifts give different roots inside the SAME enclosure"),
                root->GetLocalRootBounds().Axis[2].Lower<=.5-.5/1048576.&&
                root->GetLocalRootBounds().Axis[2].Upper>=.5+.5/1048576.);
            FIntervalPoint cloud[8];
            for(int32 k=0;k<8;++k)cloud[k]=Detail::Point(FVector3d((k&1)?1:-1,(k&2)?1:-1,(k&4)?1:-1));
            FIntervalPoint rawNormal;
            TestFalse(TEXT("a face or edge tie cannot become a unique support vertex"),
                root->ProveTranslatedCloudVertexApproach(cloud,3,1,rootWork,rawNormal));
            auto tilted=curve;
            for(int32 i=0;i<3;++i)for(int32 k=0;k<2;++k)
                tilted.ControlPoints[i*2+k].Axis[2]=Interval::Exact(2*x[i]+2*(k?.5:-.5)+z[i]);
            if(!TestTrue(TEXT("tilted curved reference surface keeps a strict transverse root"),
                FTensorPointRootBounds::TryProve(tilted,path,rootWork,root)&&root))return false;
            TestTrue(TEXT("pairwise normal cone isolates supplied support corner3 and closing approach"),
                root->ProveTranslatedCloudVertexApproach(cloud,3,1,rootWork,rawNormal));
            TestTrue(TEXT("raw curved normal is bounded without fabricating a unit normal"),
                Detail::ValidPoint(rawNormal)&&rawNormal.Axis[0].Upper<0&&rawNormal.Axis[1].Upper<0&&rawNormal.Axis[2].Lower>0);
            TestFalse(TEXT("wrong support corner cannot reuse the vertex root"),
                root->ProveTranslatedCloudVertexApproach(cloud,7,1,rootWork,rawNormal));
            TestFalse(TEXT("reversed normal does not satisfy the same closing approach"),
                root->ProveTranslatedCloudVertexApproach(cloud,3,-1,rootWork,rawNormal));
            auto aliasCloud=cloud[0];
            TestTrue(TEXT("raw-normal output can alias one captured cloud point without changing the original proof inputs"),
                root->ProveTranslatedCloudVertexApproach(cloud,3,1,rootWork,cloud[0]));
            cloud[0]=aliasCloud;
            TestFalse(TEXT("unsupported orientation is not silently normalized"),
                root->ProveTranslatedCloudVertexApproach(cloud,3,0,rootWork,rawNormal));
            // Captured-value fixture, never a GT/World request or car pose.
            using namespace Speed::PreparedExecution;
            FNativeSolidStaticValues native;native.Shape=ENativeSolidQueryShape::Box;native.Horizon=1;
            native.OwnerState.Location=FVector(-1,-1,2);native.OwnerState.Velocity=FVector(0,0,-2);
            native.BodyState=native.OwnerState;native.PhysicalHalfExtent=FVector(1,1,1);
            native.QueryHalfExtent=FVector(1.001,1.002,1.003);
            FNativeFixedBoxReferenceMotionValuesPtr motion;FNativeFixedBoxCornerReferenceValuesPtr joined;
            FDistanceBudget motionWork{8192};
            if(!TestTrue(TEXT("fixed Box captured values produce one owner-COM model and all8 physical corner paths"),
                FNativeFixedBoxReferenceMotionValues::TryBuild(native,motionWork,motion)&&motion))return false;
            TestTrue(TEXT("motion retains every captured field and original Hfloat"),motion->MatchesCapturedValues(native)&&motion->GetOriginalHorizon()==native.Horizon);
            if(!TestTrue(TEXT("same captured Box corner3 reference root and supplied support cloud are joined"),
                FNativeFixedBoxCornerReferenceValues::TryProve(motion,tilted,3,1,motionWork,joined)&&joined))return false;
            TestTrue(TEXT("joined root cannot be transferred to another corner or orientation"),
                joined->MatchesInputs(native,tilted,3,1)&&!joined->MatchesInputs(native,tilted,7,1)&&!joined->MatchesInputs(native,tilted,3,-1));
            auto differentH=native;differentH.Horizon=.5f;
            TestFalse(TEXT("changed H cannot reuse Box reference motion"),motion->MatchesCapturedValues(differentH));
            if(!TestTrue(TEXT("sole joined result can lend motion and tensor across output reset"),
                FNativeFixedBoxCornerReferenceValues::TryProve(joined->GetMotionForInspection(),
                    joined->GetRootForInspection()->GetTensorForInspection(),3,1,motionWork,joined)&&joined))return false;
            FReferenceDyadicCell deepest;deepest.Depth=52;deepest.Index=(uint64(1)<<52)-1;
            const auto lastRange=deepest.GetRange();
            TestTrue(TEXT("depth52 final dyadic cell has exact endpoints without int32 overflow"),
                AnalyticCertified::Valid(lastRange)&&lastRange.Lower==1.-1./4503599627370496.&&lastRange.Upper==1.);
            auto invalidCell=deepest;invalidCell.Depth=53;
            TestFalse(TEXT("depth53 refused before any shift or floating subdivision"),invalidCell.IsValid());
            invalidCell.Depth=2;invalidCell.Index=4;
            TestFalse(TEXT("out-of-range cell index refused"),invalidCell.IsValid());
            FReferenceDyadicCell asymmetric;asymmetric.Depth=2;asymmetric.Index=2;
            FIntervalBezierCenterPath unitPath,asymmetricPath;FDistanceBudget cellWork{8192};
            CenterDetail::Linear(FVector3d(0,0,0),FVector3d(1,0,0),unitPath);
            if(!TestTrue(TEXT("dyadic time-cell bits are applied most-significant first"),
                RestrictReferencePathDyadic(unitPath,asymmetric,cellWork,asymmetricPath)))return false;
            TestTrue(TEXT("index2 depth2 restricts to1/2 through3/4"),
                asymmetricPath.ControlPoints[0].Axis[0].Lower<=.5&&asymmetricPath.ControlPoints[0].Axis[0].Upper>=.5&&
                asymmetricPath.ControlPoints.Last().Axis[0].Lower<=.75&&asymmetricPath.ControlPoints.Last().Axis[0].Upper>=.75);
            auto aliasedPath=unitPath;
            TestTrue(TEXT("path restriction preserves input when output aliases it"),
                RestrictReferencePathDyadic(aliasedPath,asymmetric,cellWork,aliasedPath));
            FReferenceDyadicCell lower;lower.Depth=1;
            FIntervalTensorSurface restricted;
            if(!TestTrue(TEXT("tensor U and V cells derived by deCasteljau before root isolation"),
                RestrictReferenceTensorDyadic(tilted,lower,lower,cellWork,restricted)))return false;
            auto cellNative=native;cellNative.OwnerState.Location=FVector(-1.125,-1.25,1.015625);cellNative.BodyState=cellNative.OwnerState;
            FNativeFixedBoxReferenceMotionValuesPtr cellMotion;FNativeFixedBoxCornerReferenceValuesPtr cellRoot;
            if(!TestTrue(TEXT("same captured Box motion supplies the internally restricted time path"),
                FNativeFixedBoxReferenceMotionValues::TryBuild(cellNative,cellWork,cellMotion)&&
                FNativeFixedBoxCornerReferenceValues::TryProveInTimeCell(cellMotion,restricted,3,1,lower,cellWork,cellRoot)&&cellRoot))return false;
            const auto& localRoot=cellRoot->GetRootForInspection()->GetLocalRootBounds();
            const auto globalU=lower.MapLocal(localRoot.Axis[0]),globalV=lower.MapLocal(localRoot.Axis[1]);
            const auto globalTau=cellRoot->GetOriginalNormalizedTimeBounds(),globalSeconds=cellRoot->GetOriginalReferenceSecondsBounds();
            TestTrue(TEXT("local root maps to original u3/8 v1/4 tau3/8 without float time cast"),
                AnalyticCertified::Valid(globalU)&&AnalyticCertified::Valid(globalV)&&AnalyticCertified::Valid(globalTau)&&AnalyticCertified::Valid(globalSeconds)&&
                globalU.Lower<=.375&&globalU.Upper>=.375&&globalV.Lower<=.25&&globalV.Upper>=.25&&
                globalTau.Lower<=.375&&globalTau.Upper>=.375&&globalSeconds.Lower<=.375&&globalSeconds.Upper>=.375);
            TestTrue(TEXT("restricted matcher retains actual time code; old full-time matcher refuses it"),
                cellRoot->MatchesInputsInTimeCell(cellNative,restricted,3,1,lower)&&!cellRoot->MatchesInputs(cellNative,restricted,3,1));
            TestFalse(TEXT("different time code cannot reuse a local root"),cellRoot->MatchesInputsInTimeCell(cellNative,restricted,3,1,asymmetric));
            if(!TestTrue(TEXT("restricted sole output can lend its own motion/tensor/code"),
                FNativeFixedBoxCornerReferenceValues::TryProveInTimeCell(cellRoot->GetMotionForInspection(),
                    cellRoot->GetRootForInspection()->GetTensorForInspection(),3,1,cellRoot->GetTimeCellForInspection(),cellWork,cellRoot)&&cellRoot))return false;
            FNativeFixedBoxCornerReferenceValuesPtr boundaryCell;
            TestFalse(TEXT("true root on a dyadic subdivision boundary is not moved into the cell"),
                FNativeFixedBoxCornerReferenceValues::TryProveInTimeCell(joined->GetMotionForInspection(),tilted,3,1,lower,cellWork,boundaryCell));
            // Provider, original UV/time cells and captured canonical policy
            // derive the SAME local root without accepting an external tensor.
            FDistanceBudget retainedWork{32768};Speed::FPreparedGeometrySelectionPtr geometry;
            if(!TestTrue(TEXT("structural retained provider fixture, not authenticated stadium"),
                FContactUnitRetainedProducer::Build(tilted,retainedWork,geometry)))return false;
            auto policyNative=cellNative;policyNative.Query.Shape=EQueryShape::Box;policyNative.Query.bApplyCollisionFilter=true;
            policyNative.Query.bAuthorityOnly=true;policyNative.Query.bIncludeCompactPatches=true;policyNative.Query.BlockingObjectTypes=1;
            policyNative.Query.HalfExtent=policyNative.QueryHalfExtent;policyNative.Query.Rotation=policyNative.BodyState.Rotation;
            policyNative.Start=policyNative.Query.Start=policyNative.BodyState.Location;
            policyNative.End=policyNative.Query.End=policyNative.OwnerState.Integrate(policyNative.Horizon).Location;
            FNativeFixedBoxReferenceMotionValuesPtr policyMotion;FNativeRetainedBoxCornerReferenceValuesPtr retainedRoot;
            if(!TestTrue(TEXT("retained curved root uses copied provider tensor and internally restricted UV/time"),
                FNativeFixedBoxReferenceMotionValues::TryBuild(policyNative,retainedWork,policyMotion)&&
                FNativeRetainedBoxCornerReferenceValues::TryProve(geometry,policyMotion,0,lower,lower,lower,3,1,retainedWork,retainedRoot)&&retainedRoot))return false;
            const auto retainedU=retainedRoot->GetOriginalProviderU(),retainedV=retainedRoot->GetOriginalProviderV();
            TestTrue(TEXT("local proof owns exact provider and face, original UV, and unchanged false authority flag"),
                retainedRoot->GetGeometryForInspection()==geometry&&retainedRoot->GetProviderIndex()==0&&retainedRoot->GetFaceIndex()==0&&
                retainedRoot->GetFaceForInspection().OpaqueId==TEXT("unit.face")&&!retainedRoot->GetOwnerForInspection().AuthorityEligible&&
                retainedRoot->GetOwnerForInspection().PrimitiveId==4&&retainedU.Lower<=.375&&retainedU.Upper>=.375&&retainedV.Lower<=.25&&retainedV.Upper>=.25);
            TestTrue(TEXT("only exact geometry/provider/capture/codes/corner/sign can match local retained root"),
                retainedRoot->MatchesInputs(geometry,policyNative,0,lower,lower,lower,3,1)&&
                !retainedRoot->MatchesInputs(geometry,policyNative,1,lower,lower,lower,3,1)&&
                !retainedRoot->MatchesInputs(geometry,policyNative,0,asymmetric,lower,lower,3,1)&&
                !retainedRoot->MatchesInputs(geometry,policyNative,0,lower,lower,asymmetric,3,1)&&
                !retainedRoot->MatchesInputs(geometry,policyNative,0,lower,lower,lower,7,1));
            auto changedPolicy=policyNative;changedPolicy.Query.RequiredSourceId=5;
            TestFalse(TEXT("changed full query capture cannot reuse retained local root"),retainedRoot->MatchesInputs(geometry,changedPolicy,0,lower,lower,lower,3,1));
            FNativeFixedBoxReferenceMotionValuesPtr rejectedMotion;FNativeRetainedBoxCornerReferenceValuesPtr refused=retainedRoot;
            TestTrue(TEXT("mismatching Source restriction refuses before root and clears output"),
                FNativeFixedBoxReferenceMotionValues::TryBuild(changedPolicy,retainedWork,rejectedMotion)&&
                !FNativeRetainedBoxCornerReferenceValues::TryProve(geometry,rejectedMotion,0,lower,lower,lower,3,1,retainedWork,refused)&&!refused);
            Speed::FPreparedGeometrySelectionPtr nominal;FString nominalReason;
            if(!TestTrue(TEXT("equal coefficients still have explicit nominal model"),Speed::FPreparedGeometrySelection::TryCreate(
                Speed::EPreparedGeometryModel::NominalInput,geometry->GetNominalScene(),{},nominal,nominalReason)))return false;
            refused=retainedRoot;
            TestTrue(TEXT("nominal geometry and wrong provider index cannot relabel retained proof"),
                !retainedRoot->MatchesInputs(nominal,policyNative,0,lower,lower,lower,3,1)&&
                !FNativeRetainedBoxCornerReferenceValues::TryProve(nominal,policyMotion,0,lower,lower,lower,3,1,retainedWork,refused)&&!refused&&
                !FNativeRetainedBoxCornerReferenceValues::TryProve(geometry,policyMotion,1,lower,lower,lower,3,1,retainedWork,refused));
            geometry.Reset();policyMotion.Reset();
            if(!TestTrue(TEXT("sole result retains provider face/motion/cells across aliased output reset"),
                FNativeRetainedBoxCornerReferenceValues::TryProve(retainedRoot->GetGeometryForInspection(),
                    retainedRoot->GetCornerReferenceForInspection()->GetMotionForInspection(),0,retainedRoot->GetUCellForInspection(),
                    retainedRoot->GetVCellForInspection(),retainedRoot->GetCornerReferenceForInspection()->GetTimeCellForInspection(),3,1,retainedWork,retainedRoot)&&retainedRoot))return false;
            auto normalFiltered=policyNative;normalFiltered.Query.ReferenceNormal.X=AnalyticCertified::FromBits(uint64(1));
            FNativeFixedBoxReferenceMotionValuesPtr normalFilteredMotion;refused=retainedRoot;
            TestTrue(TEXT("subnormal reference-normal policy is not mistaken for disabled zero under DAZ"),
                FNativeFixedBoxReferenceMotionValues::TryBuild(normalFiltered,retainedWork,normalFilteredMotion)&&
                !FNativeRetainedBoxCornerReferenceValues::TryProve(retainedRoot->GetGeometryForInspection(),normalFilteredMotion,
                    0,lower,lower,lower,3,1,retainedWork,refused)&&!refused);
            FDistanceBudget noRetainedWork{0};refused=retainedRoot;
            TestTrue(TEXT("no work budget refuses retained root without publishing partial provenance"),
                !FNativeRetainedBoxCornerReferenceValues::TryProve(retainedRoot->GetGeometryForInspection(),
                    retainedRoot->GetCornerReferenceForInspection()->GetMotionForInspection(),0,lower,lower,lower,3,1,noRetainedWork,refused)&&!refused);
            FReferenceDyadicCell quarterStart;quarterStart.Depth=2;quarterStart.Index=1;
            FDistanceBudget prefixWork{262144};FNativeRetainedBoxCornerReferenceValuesPtr quarterCorner;
            if(!TestTrue(TEXT("same local root may be isolated in time cell [1/4,1/2]"),
                FNativeRetainedBoxCornerReferenceValues::TryProve(retainedRoot->GetGeometryForInspection(),
                    retainedRoot->GetCornerReferenceForInspection()->GetMotionForInspection(),0,lower,lower,quarterStart,3,1,prefixWork,quarterCorner)))return false;
            FNativeRetainedBoxPreCellClearanceValuesPtr prefix;
            if(!TestTrue(TEXT("all retained providers exclude the FULL reference Box over closed [0,1/4]"),
                FNativeRetainedBoxPreCellClearanceValues::TryProve(quarterCorner,prefixWork,prefix)&&prefix))return false;
            TestTrue(TEXT("proof stops at time-cell start, not local root or float event time"),
                prefix->GetReferencePrefixEndNormalized()==.25&&prefix->GetSlabsForInspection().Num()==1&&
                !prefix->GetSlabsForInspection()[0].InitialPointOnly&&prefix->MatchesInputs(quarterCorner,policyNative)&&
                !prefix->MatchesInputs(retainedRoot,policyNative)&&!prefix->MatchesInputs(quarterCorner,changedPolicy));
            FNativeRetainedBoxPreCellClearanceValuesPtr initialPrefix;
            if(!TestTrue(TEXT("zero-start root cell requires explicit initial full-Box proof, not vacuous list"),
                FNativeRetainedBoxPreCellClearanceValues::TryProve(retainedRoot,prefixWork,initialPrefix)&&initialPrefix))return false;
            TestTrue(TEXT("tau0 prefix is labelled initial point only"),initialPrefix->GetReferencePrefixEndNormalized()==0&&
                initialPrefix->GetSlabsForInspection().Num()==1&&initialPrefix->GetSlabsForInspection()[0].InitialPointOnly);
            // A second coefficient fixture exercises a noncontiguous MSB
            // prefix cover: [0,1/2] union [1/2,5/8], root at11/16.
            auto laterNative=policyNative;laterNative.OwnerState.Location.Z=1.640625;laterNative.BodyState=laterNative.OwnerState;
            FNativeFixedBoxReferenceMotionValuesPtr laterMotion;FNativeRetainedBoxCornerReferenceValuesPtr laterCorner;
            FReferenceDyadicCell laterCell;laterCell.Depth=3;laterCell.Index=5;
            FNativeRetainedBoxPreCellClearanceValuesPtr laterPrefix;
            if(!TestTrue(TEXT("multi-bit time cell retains one closed ordered cover from the same later reference motion"),
                FNativeFixedBoxReferenceMotionValues::TryBuild(laterNative,prefixWork,laterMotion)&&
                FNativeRetainedBoxCornerReferenceValues::TryProve(retainedRoot->GetGeometryForInspection(),laterMotion,
                    0,quarterStart,lower,laterCell,3,1,prefixWork,laterCorner)&&
                FNativeRetainedBoxPreCellClearanceValues::TryProve(laterCorner,prefixWork,laterPrefix)&&laterPrefix))return false;
            const auto laterSlabs=laterPrefix->GetSlabsForInspection();
            TestTrue(TEXT("depth3 index5 emits left siblings (1,0) then (3,4), ending exactly5/8"),
                laterSlabs.Num()==2&&laterSlabs[0].Cell.Depth==1&&laterSlabs[0].Cell.Index==0&&
                laterSlabs[1].Cell.Depth==3&&laterSlabs[1].Cell.Index==4&&laterPrefix->GetReferencePrefixEndNormalized()==.625);
            Speed::FPreparedGeometrySelectionPtr blockedGeometry;FNativeRetainedBoxCornerReferenceValuesPtr blockedCorner;
            if(!TestTrue(TEXT("second structural tensor lies inside the swept Box, while same local curved root still exists"),
                FContactUnitRetainedProducer::Build(tilted,prefixWork,blockedGeometry,true)&&
                FNativeRetainedBoxCornerReferenceValues::TryProve(blockedGeometry,
                    retainedRoot->GetCornerReferenceForInspection()->GetMotionForInspection(),0,lower,lower,quarterStart,3,1,prefixWork,blockedCorner)))return false;
            bool allCornerTracesMissBlocker=true;
            for(const auto& p:retainedRoot->GetCornerReferenceForInspection()->GetMotionForInspection()->GetCornerPathsForInspection())
            {
                const auto h=CenterDetail::Hull(p);
                allCornerTracesMissBlocker&=h.Axis[0].Upper<-.375||h.Axis[0].Lower>-.25;
            }
            TestTrue(TEXT("all8 corner trajectories miss finite interior blocker, so corner-only checks are insufficient"),allCornerTracesMissBlocker);
            FDistanceBudget blockedWork{8192};auto blockedPrefix=prefix;
            TestTrue(TEXT("full-Box/all-provider proof refuses interior blocker and clears prior output"),
                !FNativeRetainedBoxPreCellClearanceValues::TryProve(blockedCorner,blockedWork,blockedPrefix)&&!blockedPrefix);
            if(!TestTrue(TEXT("sole pre-cell proof can lend owned corner across output reset"),
                FNativeRetainedBoxPreCellClearanceValues::TryProve(prefix->GetCornerForInspection(),prefixWork,prefix)&&prefix))return false;
            FDistanceBudget noPrefixWork{0};blockedPrefix=prefix;
            TestTrue(TEXT("exhausted pre-cell work never publishes partial all-provider coverage"),
                !FNativeRetainedBoxPreCellClearanceValues::TryProve(quarterCorner,noPrefixWork,blockedPrefix)&&!blockedPrefix);
            FDistanceBudget domainWork{262144};FNativeRetainedBoxRootDomainClearanceValuesPtr rootDomain;
            if(!TestTrue(TEXT("whole reference Box excludes all providers outside OPEN UV-root cell throughout time cell"),
                FNativeRetainedBoxRootDomainClearanceValues::TryProve(quarterCorner,domainWork,rootDomain)&&rootDomain))return false;
            TestTrue(TEXT("two complementary tensor rectangles and four true boundaries retained; not earliest"),
                rootDomain->GetFragmentsForInspection().Num()==6&&rootDomain->MatchesInputs(quarterCorner,policyNative)&&
                !rootDomain->MatchesInputs(retainedRoot,policyNative)&&!rootDomain->MatchesInputs(quarterCorner,changedPolicy));
            bool edgesClosed=true;int32 edgeCount=0;
            for(const auto& f:rootDomain->GetFragmentsForInspection())
            {
                edgesClosed&=f.ProviderIndex==0&&f.Distance.Status!=EDistanceBoundStatus::Unknown&&f.Distance.LowerSquared>0;
                if(f.Part==EReferenceRootDomainPart::UMin||f.Part==EReferenceRootDomainPart::UMax)
                {++edgeCount;edgesClosed&=f.Tensor.DegreeU==0&&f.Tensor.DegreeV==1;}
                if(f.Part==EReferenceRootDomainPart::VMin||f.Part==EReferenceRootDomainPart::VMax)
                {++edgeCount;edgesClosed&=f.Tensor.DegreeV==0&&f.Tensor.DegreeU==2;}
            }
            TestTrue(TEXT("degree-zero edges use original row/column strides and explicit all-boundary coverage"),edgesClosed&&edgeCount==4);
            // With FULL UV there are no complement rectangles. The four real
            // boundaries still matter: they enter the Box later in full H.
            FNativeRetainedBoxCornerReferenceValuesPtr fullCellCorner;
            if(!TestTrue(TEXT("same reference point root exists in full UV/time cube"),
                FNativeRetainedBoxCornerReferenceValues::TryProve(retainedRoot->GetGeometryForInspection(),
                    retainedRoot->GetCornerReferenceForInspection()->GetMotionForInspection(),0,{},{},{},3,1,domainWork,fullCellCorner)))return false;
            FDistanceBudget boundaryWork{8192};auto rejectedDomain=rootDomain;
            TestTrue(TEXT("no complementary rectangles does not omit physical provider boundaries intersecting later in cell"),
                !FNativeRetainedBoxRootDomainClearanceValues::TryProve(fullCellCorner,boundaryWork,rejectedDomain)&&!rejectedDomain);
            FDistanceBudget otherProviderWork{8192};rejectedDomain=rootDomain;
            TestTrue(TEXT("every OTHER provider remains included even when all corner traces miss its interior patch"),
                !FNativeRetainedBoxRootDomainClearanceValues::TryProve(blockedCorner,otherProviderWork,rejectedDomain)&&!rejectedDomain);
            if(!TestTrue(TEXT("root-domain result can lend its sole owned corner across output reset"),
                FNativeRetainedBoxRootDomainClearanceValues::TryProve(rootDomain->GetCornerForInspection(),domainWork,rootDomain)&&rootDomain))return false;
            FDistanceBudget noDomainWork{0};rejectedDomain=rootDomain;
            TestTrue(TEXT("budget0 never publishes partial domain/provider coverage"),
                !FNativeRetainedBoxRootDomainClearanceValues::TryProve(quarterCorner,noDomainWork,rejectedDomain)&&!rejectedDomain);
            FDistanceBudget firstWork{4096};FNativeRetainedBoxReferenceFirstEntryValuesPtr firstReference;
            if(!TestTrue(TEXT("same pre-cell/domain/local-root/common full Box close FIRST REFERENCE entry only"),
                FNativeRetainedBoxReferenceFirstEntryValues::TryProve(prefix,rootDomain,firstWork,firstReference)&&firstReference))return false;
            const auto firstTau=firstReference->GetFirstReferenceNormalizedTimeBounds(),firstSeconds=firstReference->GetFirstReferenceSecondsBounds();
            const auto dimension=firstReference->GetDimensionDeterminantForInspection();
            // Search receives only the retained geometry and SAME captured
            // motion: provider/corner/sign/UV/time codes are NOT fixture hints.
            FDistanceBudget searchWork{262144};FNativeRetainedBoxReferenceFirstEntryValuesPtr searched;
            if(!TestTrue(TEXT("bounded automatic search derives a globally first REFERENCE entry without fixture cell hints"),
                FindNativeRetainedBoxReferenceFirstEntry(quarterCorner->GetGeometryForInspection(),
                    quarterCorner->GetCornerReferenceForInspection()->GetMotionForInspection(),8192,searchWork,searched)&&searched))return false;
            const auto found=searched->GetFirstReferenceNormalizedTimeBounds();
            TestTrue(TEXT("searched proof owns same geometry/full capture, contains3/8, and leaves authority false"),
                searched->MatchesInputs(quarterCorner->GetGeometryForInspection(),policyNative)&&found.Lower<=.375&&found.Upper>=.375&&
                !searched->GetPrefixForInspection()->GetCornerForInspection()->GetOwnerForInspection().AuthorityEligible);
            auto failedSearch=searched;FDistanceBudget tinySearch{0};
            TestTrue(TEXT("search budget0 clears prior output; exhausted search is Unknown and never a miss"),
                !FindNativeRetainedBoxReferenceFirstEntry(quarterCorner->GetGeometryForInspection(),
                    quarterCorner->GetCornerReferenceForInspection()->GetMotionForInspection(),8192,tinySearch,failedSearch)&&!failedSearch);
            failedSearch=searched;FDistanceBudget invalidSearch{4096};
            TestTrue(TEXT("nonpositive technical attempt limit refuses without spending or publishing"),
                !FindNativeRetainedBoxReferenceFirstEntry(quarterCorner->GetGeometryForInspection(),
                    quarterCorner->GetCornerReferenceForInspection()->GetMotionForInspection(),0,invalidSearch,failedSearch)&&
                !failedSearch&&invalidSearch.Remaining==4096&&invalidSearch.Used==0);
            failedSearch=searched;FDistanceBudget nominalSearch{4096};
            TestTrue(TEXT("automatic search cannot relabel nominal-only geometry as retained reference"),
                !FindNativeRetainedBoxReferenceFirstEntry(nominal,quarterCorner->GetCornerReferenceForInspection()->GetMotionForInspection(),
                    1024,nominalSearch,failedSearch)&&!failedSearch);
            failedSearch=searched;FDistanceBudget blockedSearch{16384};
            TestTrue(TEXT("interior OTHER provider cannot be omitted by automatic root candidate pruning"),
                !FindNativeRetainedBoxReferenceFirstEntry(blockedGeometry,
                    quarterCorner->GetCornerReferenceForInspection()->GetMotionForInspection(),1024,blockedSearch,failedSearch)&&!failedSearch&&
                blockedSearch.Used>0&&blockedSearch.Remaining<16384);
            // Sole output lends its retained inputs; failed work remains charged.
            FDistanceBudget aliasSearch{262144};
            if(!TestTrue(TEXT("automatic search preserves sole output input lifetime without supplied provider codes"),
                FindNativeRetainedBoxReferenceFirstEntry(searched->GetPrefixForInspection()->GetCornerForInspection()->GetGeometryForInspection(),
                    searched->GetPrefixForInspection()->GetCornerForInspection()->GetCornerReferenceForInspection()->GetMotionForInspection(),
                    8192,aliasSearch,searched)&&searched))return false;

            TestTrue(TEXT("first reference bounds contain3/8 and SAME physical-Box determinant excludes zero"),
                firstTau.Lower<=.375&&firstTau.Upper>=.375&&firstSeconds.Lower<=.375&&firstSeconds.Upper>=.375&&
                dimension.Lower>0&&dimension.Lower<=8&&dimension.Upper>=8&&
                firstReference->MatchesInputs(quarterCorner->GetGeometryForInspection(),policyNative)&&
                !firstReference->MatchesInputs(quarterCorner->GetGeometryForInspection(),changedPolicy));

            // Strict geometric intersection with the ACTUAL stored float-time
            // corner hull. It does not choose or authenticate a native event.
            FDistanceBudget interiorWork{262144};FNativeFixedBoxAtFloatTimeValuesPtr laterActual;
            if(!TestTrue(TEXT("actual native corner values evaluated at PROVIDED original float3/4"),
                FNativeFixedBoxAtFloatTimeValues::TryBuild(quarterCorner->GetCornerReferenceForInspection()->GetMotionForInspection(),
                    .75f,interiorWork,laterActual)&&laterActual))return false;
            FNativeRetainedBoxAtFloatInteriorValuesPtr interior;
            if(!TestTrue(TEXT("retained tensor has a strict interior witness in hull of SAME actual float-time corners"),
                FNativeRetainedBoxAtFloatInteriorValues::TryFind(firstReference,laterActual,interiorWork,interior)&&interior))return false;
            TestTrue(TEXT("inside proof keeps actual sample/provider/float and never changes false authority"),
                interior->MatchesInputs(firstReference,laterActual,quarterCorner->GetGeometryForInspection(),policyNative,.75f)&&
                !interior->MatchesInputs(firstReference,laterActual,quarterCorner->GetGeometryForInspection(),policyNative,.5f)&&
                interior->GetProviderCellUForInspection().IsValid()&&interior->GetProviderCellVForInspection().IsValid()&&
                !firstReference->GetPrefixForInspection()->GetCornerForInspection()->GetOwnerForInspection().AuthorityEligible);
            FNativeFixedBoxAtFloatTimeValuesPtr earlyActual;
            if(!TestTrue(TEXT("strictly precontact native sample remains separate from later inside sample"),
                FNativeFixedBoxAtFloatTimeValues::TryBuild(quarterCorner->GetCornerReferenceForInspection()->GetMotionForInspection(),
                    .125f,interiorWork,earlyActual)))return false;
            auto refusedInterior=interior;FDistanceBudget earlyWork{8192};
            TestTrue(TEXT("clear provided float cannot become inside merely by sharing first-reference Motion"),
                !FNativeRetainedBoxAtFloatInteriorValues::TryFind(firstReference,earlyActual,earlyWork,refusedInterior)&&!refusedInterior);
            refusedInterior=interior;FDistanceBudget noInteriorWork{0};
            TestTrue(TEXT("inside budget0 clears old output without partial tetrahedron evidence"),
                !FNativeRetainedBoxAtFloatInteriorValues::TryFind(firstReference,laterActual,noInteriorWork,refusedInterior)&&!refusedInterior);
            if(!TestTrue(TEXT("sole interior output lends both owned inputs safely across reset"),
                FNativeRetainedBoxAtFloatInteriorValues::TryFind(interior->GetFirstReferenceForInspection(),
                    interior->GetActualAtFloatForInspection(),interiorWork,interior)&&interior))return false;
            FIntervalPoint tetra[4]={Detail::Point(FVector3d(0,0,0)),Detail::Point(FVector3d(1,0,0)),
                Detail::Point(FVector3d(0,1,0)),Detail::Point(FVector3d(0,0,1))};
            Interval tetraDet[5];FDistanceBudget tetraWork{4096};
            TestTrue(TEXT("five strictly positive determinants enclose every barycentric numerator"),
                FNativeRetainedBoxAtFloatInteriorValues::BoundStrictTetrahedronInterior(tetra,Detail::Point(FVector3d(.125,.25,.125)),tetraWork,tetraDet));
            bool positiveFive=true;for(const auto& d:tetraDet)positiveFive&=AnalyticCertified::Valid(d)&&d.Lower>0;
            TestTrue(TEXT("all five determinant enclosures remain inspectable"),positiveFive);
            Swap(tetra[1],tetra[2]);
            TestTrue(TEXT("negative tetrahedron orientation is equally admissible"),
                FNativeRetainedBoxAtFloatInteriorValues::BoundStrictTetrahedronInterior(tetra,Detail::Point(FVector3d(.125,.25,.125)),tetraWork,tetraDet));
            bool negativeFive=true;for(const auto& d:tetraDet)negativeFive&=AnalyticCertified::Valid(d)&&d.Upper<0;
            TestTrue(TEXT("all five signs reverse together"),negativeFive);
            TestTrue(TEXT("face boundary is Unknown without an epsilon and clears every determinant"),
                !FNativeRetainedBoxAtFloatInteriorValues::BoundStrictTetrahedronInterior(tetra,Detail::Point(FVector3d(0,.25,.25)),tetraWork,tetraDet)&&
                !tetraDet[0].Known&&!tetraDet[1].Known&&!tetraDet[2].Known&&!tetraDet[3].Known&&!tetraDet[4].Known);
            auto uncertainPoint=Detail::Point(FVector3d(.125,.25,.125));uncertainPoint.Axis[0]={-.125,.125,true};
            TestFalse(TEXT("scalar midpoint cannot hide a point interval crossing outside tetrahedron"),
                FNativeRetainedBoxAtFloatInteriorValues::BoundStrictTetrahedronInterior(tetra,uncertainPoint,tetraWork,tetraDet));
            tetra[3]=tetra[0];
            TestFalse(TEXT("collapsed tetrahedron supplies no inside proof"),
                FNativeRetainedBoxAtFloatInteriorValues::BoundStrictTetrahedronInterior(tetra,Detail::Point(FVector3d(.125,.25,.125)),tetraWork,tetraDet));
            auto refusedFirst=firstReference;
            TestTrue(TEXT("same captures cannot splice pre-cell and domain proofs from different root time cells"),
                !FNativeRetainedBoxReferenceFirstEntryValues::TryProve(initialPrefix,rootDomain,firstWork,refusedFirst)&&!refusedFirst);
            refusedFirst=firstReference;
            TestTrue(TEXT("no first reference entry without complete exterior and boundary proof"),
                !FNativeRetainedBoxReferenceFirstEntryValues::TryProve(prefix,{},firstWork,refusedFirst)&&!refusedFirst);
            FNativeRetainedBoxCornerReferenceValuesPtr reversedCorner;
            TestTrue(TEXT("opposite normal cannot substitute for uniform selected-support/closing premise"),
                !FNativeRetainedBoxCornerReferenceValues::TryProve(quarterCorner->GetGeometryForInspection(),
                    quarterCorner->GetCornerReferenceForInspection()->GetMotionForInspection(),0,lower,lower,quarterStart,3,-1,firstWork,reversedCorner)&&!reversedCorner);
            FIntervalPoint fullCloud[8];
            for(int32 k=0;k<8;++k)fullCloud[k]=Detail::Point(FVector3d((k&1)?1:-1,(k&2)?1:-1,(k&4)?1:-1));
            Interval cloudDimension;
            TestTrue(TEXT("generic outward determinant proves full cloud without assuming orthonormal native axes"),
                FNativeRetainedBoxReferenceFirstEntryValues::BoundReferenceCloudFullDimension(fullCloud,firstWork,cloudDimension)&&
                cloudDimension.Lower>0&&cloudDimension.Lower<=8&&cloudDimension.Upper>=8);
            for(auto& p:fullCloud)p.Axis[0]=AnalyticCertified::Multiply(Interval::Exact(-1),p.Axis[0]);
            TestTrue(TEXT("negative orientation is also full dimension"),
                FNativeRetainedBoxReferenceFirstEntryValues::BoundReferenceCloudFullDimension(fullCloud,firstWork,cloudDimension)&&
                cloudDimension.Upper<0&&cloudDimension.Lower<=-8&&cloudDimension.Upper>=-8);
            TestTrue(TEXT("determinant output may alias one cloud coordinate while input lifetime is preserved"),
                FNativeRetainedBoxReferenceFirstEntryValues::BoundReferenceCloudFullDimension(fullCloud,firstWork,fullCloud[0].Axis[0]));
            for(auto& p:fullCloud)p.Axis[2]=Interval::Exact(0);
            TestTrue(TEXT("flattened cloud cannot supply full-dimensional first-contact premise and clears output"),
                !FNativeRetainedBoxReferenceFirstEntryValues::BoundReferenceCloudFullDimension(fullCloud,firstWork,cloudDimension)&&
                !AnalyticCertified::Valid(cloudDimension));
            prefix.Reset();rootDomain.Reset();
            if(!TestTrue(TEXT("sole first-reference result can lend both owned proofs across output reset"),
                FNativeRetainedBoxReferenceFirstEntryValues::TryProve(firstReference->GetPrefixForInspection(),
                    firstReference->GetDomainForInspection(),firstWork,firstReference)&&firstReference))return false;
            FDistanceBudget noFirstWork{0};refusedFirst=firstReference;
            TestTrue(TEXT("budget0 never mints partial first-reference entry"),
                !FNativeRetainedBoxReferenceFirstEntryValues::TryProve(firstReference->GetPrefixForInspection(),
                    firstReference->GetDomainForInspection(),noFirstWork,refusedFirst)&&!refusedFirst);
            FDistanceBudget sampleWork{8192};FNativeFixedBoxAtFloatTimeValuesPtr floatSample;
            const auto firstMotion=firstReference->GetPrefixForInspection()->GetCornerForInspection()->GetCornerReferenceForInspection()->GetMotionForInspection();
            if(!TestTrue(TEXT("native Box values computed at provided .375f without selecting an event"),
                FNativeFixedBoxAtFloatTimeValues::TryBuild(firstMotion,.375f,sampleWork,floatSample)&&floatSample))return false;
            const auto actualCorner=floatSample->GetPhysicalCornersForInspection()[3];
            TestTrue(TEXT("actual corner uses physical extents and original float, keeps common reference provenance only"),
                actualCorner==FVector(-.125,-.25,-.734375)&&floatSample->GetFloatSeconds()==.375f&&
                floatSample->SharesMotionWithReferenceEntry(firstReference)&&
                floatSample->MatchesMotionAndSeconds(firstMotion,policyNative,.375f)&&
                !floatSample->MatchesMotionAndSeconds(firstMotion,changedPolicy,.375f));
            const float nextSample=KinematicCenterDetail::FromBits(KinematicCenterDetail::Bits(.375f)+1);
            TestFalse(TEXT("adjacent float seconds cannot reuse sample bits"),floatSample->MatchesMotionAndSeconds(firstMotion,policyNative,nextSample));
            FNativeFixedBoxAtFloatTimeValuesPtr nextFloatSample;
            if(!TestTrue(TEXT("adjacent original float is evaluated directly, no ratio-and-back retime"),
                FNativeFixedBoxAtFloatTimeValues::TryBuild(firstMotion,nextSample,sampleWork,nextFloatSample)))return false;
            TestTrue(TEXT("actual next-time corner retains distinct native floating location without Quantize"),
                nextFloatSample->GetPhysicalCornersForInspection()[3].Z!=actualCorner.Z&&nextFloatSample->GetFloatSeconds()==nextSample);
            for(float invalid:{KinematicCenterDetail::FromBits(0x80000000u),KinematicCenterDetail::FromBits(1u),
                KinematicCenterDetail::FromBits(0x7fc00000u),KinematicCenterDetail::FromBits(0x7f800000u),
                KinematicCenterDetail::FromBits(KinematicCenterDetail::Bits(policyNative.Horizon)+1)})
            {
                auto rejectedSample=floatSample;
                TestTrue(TEXT("negativezero/subnormal/NaN/infinite/out-of-H time refused before evaluation"),
                    !FNativeFixedBoxAtFloatTimeValues::TryBuild(firstMotion,invalid,sampleWork,rejectedSample)&&!rejectedSample);
            }
            if(!TestTrue(TEXT("sole native-time sample can lend Motion/time across output reset"),
                FNativeFixedBoxAtFloatTimeValues::TryBuild(floatSample->GetMotionForInspection(),floatSample->GetFloatSeconds(),sampleWork,floatSample)&&floatSample))return false;
            FDistanceBudget noSampleWork{0};auto rejectedSample=floatSample;
            TestTrue(TEXT("budget0 cannot publish partial native-time values"),
                !FNativeFixedBoxAtFloatTimeValues::TryBuild(firstMotion,.375f,noSampleWork,rejectedSample)&&!rejectedSample);
            FDistanceBudget rangeWork{262144};FNativeFixedBoxFloatRangeGeometryValuesPtr rangeGeometry;
            if(!TestTrue(TEXT("same PVA Box physical/rounded geometry excludes ALL providers for every original float in[0,1/8]"),
                FNativeFixedBoxFloatRangeGeometryValues::TryProve(quarterCorner->GetGeometryForInspection(),firstMotion,0.f,.125f,rangeWork,rangeGeometry)&&rangeGeometry))return false;
            TestTrue(TEXT("range retains original floats,H,footprints and exact Motion/query captures, not full World chord miss"),
                rangeGeometry->GetFirstFloatSeconds()==0&&rangeGeometry->GetLastFloatSeconds()==.125f&&
                rangeGeometry->GetMotionForInspection()->GetOriginalHorizon()==policyNative.Horizon&&
                rangeGeometry->MatchesInputs(quarterCorner->GetGeometryForInspection(),firstMotion,policyNative,0.f,.125f)&&
                !rangeGeometry->MatchesInputs(quarterCorner->GetGeometryForInspection(),firstMotion,policyNative,0.f,nextSample));
            const uint32 rangeControls=VectorGetControlRegister();
            VectorSetControlRegister(rangeControls|uint32(0x20));
            TestTrue(TEXT("sticky flags do not relabel numeric controls"),rangeGeometry->MatchesInputs(
                quarterCorner->GetGeometryForInspection(),firstMotion,policyNative,0.f,.125f));
            VectorSetControlRegister(rangeControls^uint32(0x8000));
            TestFalse(TEXT("same floats do not silently transfer cached-axis footprints across FTZ controls"),rangeGeometry->MatchesInputs(
                quarterCorner->GetGeometryForInspection(),firstMotion,policyNative,0.f,.125f));
            VectorSetControlRegister(rangeControls);

            TestTrue(TEXT("actual sample now retains the same complete controls as the range proof"),
                floatSample->GetNumericForInspection().Status==EAnalyticNumericStatus::Complete&&
                floatSample->GetNumericForInspection().ControlFingerprint()==rangeGeometry->GetNumericForInspection().ControlFingerprint());
            VectorSetControlRegister(rangeControls|uint32(0x20));
            TestTrue(TEXT("native sample matching ignores only sticky flags"),floatSample->MatchesMotionAndSeconds(firstMotion,policyNative,.375f));
            VectorSetControlRegister(rangeControls^uint32(0x8000));
            TestFalse(TEXT("same Motion/time cannot transfer an actual sample across FTZ controls"),floatSample->MatchesMotionAndSeconds(firstMotion,policyNative,.375f));
            VectorSetControlRegister(rangeControls);

            // The bracket's declared P concerns PHYSICAL corner hulls. Keep
            // old rounded-query/shell Range proof distinct and unchanged.
            FNativeFixedBoxPhysicalFloatRangeClearanceValuesPtr physicalPrefix,physicalLater;
            if(!TestTrue(TEXT("strong physical-prefix type excludes entire union for same8consumed corners"),
                FNativeFixedBoxPhysicalFloatRangeClearanceValues::TryProve(quarterCorner->GetGeometryForInspection(),firstMotion,
                    0.f,.125f,rangeWork,physicalPrefix)&&physicalPrefix&&
                physicalPrefix->MatchesInputs(quarterCorner->GetGeometryForInspection(),firstMotion,policyNative,0.f,.125f)))return false;
            if(!TestTrue(TEXT("physical hull remains strictly clear through5/16 without importing query shells"),
                FNativeFixedBoxPhysicalFloatRangeClearanceValues::TryProve(quarterCorner->GetGeometryForInspection(),firstMotion,
                    0.f,.3125f,rangeWork,physicalLater)&&physicalLater))return false;
            FDistanceBudget widerQueryWork{8192};FNativeFixedBoxFloatRangeGeometryValuesPtr widerQueryRange;
            TestTrue(TEXT("same captured rounded/shell geometry cannot be relabelled clear while physical P remains clear"),
                !FNativeFixedBoxFloatRangeGeometryValues::TryProve(quarterCorner->GetGeometryForInspection(),firstMotion,
                    0.f,.3125f,widerQueryWork,widerQueryRange)&&!widerQueryRange);
            FDistanceBudget touchingPhysicalWork{8192};auto refusedPhysical=physicalPrefix;
            TestTrue(TEXT("physical tangency3/8 remains Unknown without an epsilon or shell allowance"),
                !FNativeFixedBoxPhysicalFloatRangeClearanceValues::TryProve(quarterCorner->GetGeometryForInspection(),firstMotion,
                    0.f,.375f,touchingPhysicalWork,refusedPhysical)&&!refusedPhysical);
            FDistanceBudget physicalNoWork{0};refusedPhysical=physicalPrefix;
            TestTrue(TEXT("physical-prefix budget0 cannot publish partial hull or separation"),
                !FNativeFixedBoxPhysicalFloatRangeClearanceValues::TryProve(quarterCorner->GetGeometryForInspection(),firstMotion,
                    0.f,.125f,physicalNoWork,refusedPhysical)&&!refusedPhysical);
            VectorSetControlRegister(rangeControls^uint32(0x8000));
            TestFalse(TEXT("physical-prefix matcher still binds complete controls"),physicalPrefix->MatchesInputs(
                quarterCorner->GetGeometryForInspection(),firstMotion,policyNative,0.f,.125f));
            VectorSetControlRegister(rangeControls);
            FDistanceBudget bracketWork{32768};FNativeRetainedBoxFloatIntersectionBracketValuesPtr bracket;
            if(!TestTrue(TEXT("closed all-provider prefix plus SAME actual inside sample bounds first admitted geometric intersection"),
                FNativeRetainedBoxFloatIntersectionBracketValues::TryJoin(physicalPrefix,interior,bracketWork,bracket)&&bracket))return false;
            TestTrue(TEXT("bracket keeps originalfloat endpoints and exact compressed-ordinal successor; not nativeTOI"),
                bracket->GetClearPrefixSeconds()==.125f&&bracket->GetInsideSeconds()==.75f&&
                bracket->GetFirstPossibleSeconds()==KinematicCenterDetail::FromBits(KinematicCenterDetail::Bits(.125f)+1)&&
                !bracket->IsSingletonAdmittedTime()&&bracket->MatchesInputs(quarterCorner->GetGeometryForInspection(),firstMotion,policyNative));
            FNativeFixedBoxPhysicalFloatRangeClearanceValuesPtr nonPrefix;
            if(!TestTrue(TEXT("a positive-start clear range is valid geometry but not an initial prefix"),
                FNativeFixedBoxPhysicalFloatRangeClearanceValues::TryProve(quarterCorner->GetGeometryForInspection(),firstMotion,.0625f,.125f,rangeWork,nonPrefix)))return false;
            auto refusedBracket=bracket;
            TestTrue(TEXT("bracket rejects omitted initial interval instead of inferring endpoint monotonicity"),
                !FNativeRetainedBoxFloatIntersectionBracketValues::TryJoin(nonPrefix,interior,bracketWork,refusedBracket)&&!refusedBracket);
            FNativeFixedBoxReferenceMotionValuesPtr separateMotion;FNativeFixedBoxPhysicalFloatRangeClearanceValuesPtr separatePrefix;
            if(!TestTrue(TEXT("equal captures may still belong to distinct immutable Motion objects"),
                FNativeFixedBoxReferenceMotionValues::TryBuild(policyNative,rangeWork,separateMotion)&&
                FNativeFixedBoxPhysicalFloatRangeClearanceValues::TryProve(quarterCorner->GetGeometryForInspection(),separateMotion,0.f,.125f,rangeWork,separatePrefix)))return false;
            refusedBracket=bracket;
            TestTrue(TEXT("bracket cannot splice distinct immutable motions even with equal captured bits"),
                !FNativeRetainedBoxFloatIntersectionBracketValues::TryJoin(separatePrefix,interior,bracketWork,refusedBracket)&&!refusedBracket);
            VectorSetControlRegister(rangeControls^uint32(0x40));refusedBracket=bracket;
            TestTrue(TEXT("bracket and its join refuse changed DAZ before any physical authority"),
                !bracket->MatchesInputs(quarterCorner->GetGeometryForInspection(),firstMotion,policyNative)&&
                !FNativeRetainedBoxFloatIntersectionBracketValues::TryJoin(physicalPrefix,interior,bracketWork,refusedBracket)&&!refusedBracket);
            VectorSetControlRegister(rangeControls);
            FDistanceBudget noBracketWork{0};refusedBracket=bracket;
            TestTrue(TEXT("budget0 cannot mint a temporal bracket from partial work"),
                !FNativeRetainedBoxFloatIntersectionBracketValues::TryJoin(physicalPrefix,interior,noBracketWork,refusedBracket)&&!refusedBracket);
            // true means a still-valid bracket, not mandatory improvement. A
            // one-unit attempt quota cannot close either midpoint proof.
            FDistanceBudget quotaWork{128};FNativeRetainedBoxFloatIntersectionBracketValuesPtr stopped;
            if(!TestTrue(TEXT("unknown midpoint retains the original valid bracket without a partial update"),
                FNativeRetainedBoxFloatIntersectionBracketValues::TryRefine(bracket,1,quotaWork,stopped)&&stopped))return false;
            TestTrue(TEXT("tiny quota changes no endpoint and refunds no mathematical work"),stopped==bracket&&quotaWork.Used>0&&quotaWork.Remaining<128);
            FDistanceBudget refineWork{32768};auto refined=bracket;
            if(!TestTrue(TEXT("ordinal refinement preserves whole-prefix/inside invariants under one charged budget"),
                FNativeRetainedBoxFloatIntersectionBracketValues::TryRefine(refined,4096,refineWork,refined)&&refined))return false;
            TestTrue(TEXT("refinement never widens the first-geometric-intersection bracket or fabricates a scalar contact"),
                refined->GetClearPrefixSeconds()>=bracket->GetClearPrefixSeconds()&&refined->GetInsideSeconds()<=bracket->GetInsideSeconds()&&
                refined->MatchesInputs(quarterCorner->GetGeometryForInspection(),firstMotion,policyNative));
            if(!TestTrue(TEXT("sole bracket can lend both proofs across join output reset"),
                FNativeRetainedBoxFloatIntersectionBracketValues::TryJoin(bracket->GetPrefixForInspection(),
                    bracket->GetInsideForInspection(),bracketWork,bracket)&&bracket))return false;

            // Native provider attribution MUST recompute all distances over
            // the whole float bracket; ReferenceDomain's time cell is different.
            FDistanceBudget nativeDomainWork{262144};FNativeFixedBoxAtFloatTimeValuesPtr halfActual;
            FNativeRetainedBoxAtFloatInteriorValuesPtr halfInside;FNativeRetainedBoxFloatIntersectionBracketValuesPtr tightBracket;
            if(!TestTrue(TEXT("same physical prefix plus actual half-second inside give a tighter native window"),
                FNativeFixedBoxAtFloatTimeValues::TryBuild(firstMotion,.5f,nativeDomainWork,halfActual)&&
                FNativeRetainedBoxAtFloatInteriorValues::TryFind(firstReference,halfActual,nativeDomainWork,halfInside)&&
                FNativeRetainedBoxFloatIntersectionBracketValues::TryJoin(physicalLater,halfInside,nativeDomainWork,tightBracket)))return false;
            FNativeRetainedBoxFloatProviderDomainValuesPtr nativeDomain;
            if(!TestTrue(TEXT("all native-window intersections belong to same provider and openUV cell only"),
                FNativeRetainedBoxFloatProviderDomainValues::TryProve(tightBracket,nativeDomainWork,nativeDomain)&&nativeDomain))return false;
            TestTrue(TEXT("native proof owns successor(C)..U and six newly bounded complement/edge fragments"),
                nativeDomain->GetFirstFloatSeconds()==tightBracket->GetFirstPossibleSeconds()&&nativeDomain->GetLastFloatSeconds()==.5f&&
                nativeDomain->GetFragmentsForInspection().Num()==6&&nativeDomain->GetProviderIndex()==0&&
                nativeDomain->MatchesInputs(tightBracket,quarterCorner->GetGeometryForInspection(),policyNative)&&
                !nativeDomain->MatchesInputs(bracket,quarterCorner->GetGeometryForInspection(),policyNative)&&
                !nativeDomain->GetCornerForInspection()->GetOwnerForInspection().AuthorityEligible);
            bool nativeEdges=true;int32 nativeEdgeCount=0;
            for(const auto& frag:nativeDomain->GetFragmentsForInspection())
            {
                nativeEdges&=frag.ProviderIndex==0&&frag.Distance.Status!=EDistanceBoundStatus::Unknown&&frag.Distance.LowerSquared>0;
                if(frag.Part==EFloatProviderDomainPart::UMin||frag.Part==EFloatProviderDomainPart::UMax)
                {++nativeEdgeCount;nativeEdges&=frag.Tensor.DegreeU==0&&frag.Tensor.DegreeV==1;}
                if(frag.Part==EFloatProviderDomainPart::VMin||frag.Part==EFloatProviderDomainPart::VMax)
                {++nativeEdgeCount;nativeEdges&=frag.Tensor.DegreeU==2&&frag.Tensor.DegreeV==0;}
            }
            TestTrue(TEXT("all four closed tensor boundaries keep actual degree-zero row/column strides"),nativeEdges&&nativeEdgeCount==4);

            // This sample bounds every point/regular chart normal on the
            // native domain, while orientation/response/time remain unproved.
            FDistanceBudget chartWork{16384};FNativeRetainedBoxChartSampleValuesPtr chart;
            if(!TestTrue(TEXT("native provider domain derives original-net/globalUV actual scalar sample and separate errors"),
                FNativeRetainedBoxChartSampleValues::TryBuild(nativeDomain,chartWork,chart)&&chart))return false;
            const auto uv=chart->GetGlobalUVForInspection();const auto& representative=chart->GetRepresentativeOriginalTensorForInspection();
            const auto actualStored=representative.EvaluateNormal(uv.X,uv.Y),actualConsumed=actualStored.GetSafeNormal();
            bool scalarBits=true;
            for(int32 d=0;d<3;++d)scalarBits&=AnalyticCertified::Bits(actualStored[d])==AnalyticCertified::Bits(chart->GetStoredNormalForInspection()[d])&&
                AnalyticCertified::Bits(actualConsumed[d])==AnalyticCertified::Bits(chart->GetConsumedNormalForInspection()[d]);
            TestTrue(TEXT("scalar bits preserve original evaluator and actual second normalization without a contact grant"),
                scalarBits&&uv==FVector2d(.25,.25)&&representative.DegreeU==2&&representative.DegreeV==1&&
                chart->MatchesInputs(nativeDomain,policyNative)&&!chart->GetDomainForInspection()->GetCornerForInspection()->GetOwnerForInspection().AuthorityEligible);
            // A far point in the SAME cell must also be covered; bounding
            // only the reference root or the representative sample is unsafe.
            const auto farPoint=representative.Evaluate(.5,.5);
            const auto farDelta=ContactDetail::Subtract(Detail::Point(farPoint),Detail::Point(chart->GetPointForInspection()));
            const auto farSquared=ContactDetail::SquaredNorm(farDelta);
            TestTrue(TEXT("point error covers the full nativeUV cell and retains the unselected opposite normal"),
                AnalyticCertified::Valid(farSquared)&&chart->GetPointErrorCm()>=FMath::Sqrt(FMath::Max(0.,farSquared.Lower))&&
                chart->GetStoredEitherChartErrorL1()>=1&&chart->GetConsumedEitherChartErrorL1()>=1&&
                chart->GetStoredEitherChartErrorL1()>=chart->GetStoredOrientedChartErrorL1()&&
                chart->GetConsumedEitherChartErrorL1()>=chart->GetConsumedOrientedChartErrorL1());

            FNativeRetainedBoxResponsePointSampleValuesPtr responseSample;
            if(!TestTrue(TEXT("shared response point and arm are sampled only at same model insideU with informative ideal-branch bounds"),
                FNativeRetainedBoxResponsePointSampleValues::TryBuild(chart,chartWork,responseSample)&&responseSample))return false;
            TestTrue(TEXT("response sample retains the provided inside float rather than selecting a collision TOI"),
                responseSample->GetModelFrameAtProvidedInsideFloatForInspection()==halfActual&&halfActual->GetFloatSeconds()==.5f&&
                responseSample->MatchesInputs(chart,policyNative)&&
                responseSample->GetModelCOMForInspection()==halfActual->GetOwnerAtFloatTimeForInspection().Location);
            // P remains an ideal alternative even if the scalar resolver took
            // the face-center branch. This far chart point must not be deleted.
            const auto idealFarArm=farPoint-responseSample->GetModelCOMForInspection();
            const auto pointGap=ContactDetail::SquaredNorm(ContactDetail::Subtract(
                Detail::Point(responseSample->GetScalarForInspection().SolvePoint),Detail::Point(farPoint)));
            const auto armGap=ContactDetail::SquaredNorm(ContactDetail::Subtract(
                Detail::Point(responseSample->GetActualSubtractedArmForInspection()),Detail::Point(idealFarArm)));
            TestTrue(TEXT("separate errors cover far chart point and arm without using the scalar branch to prune ideals"),
                AnalyticCertified::Valid(pointGap)&&AnalyticCertified::Valid(armGap)&&
                responseSample->GetSamplePointErrorCm()>=FMath::Sqrt(FMath::Max(0.,pointGap.Lower))&&
                responseSample->GetSampleArmErrorCm()>=FMath::Sqrt(FMath::Max(0.,armGap.Lower)));
            const auto containsOwnerCOM=[](const FIntervalPoint& bound,const FVector& actual)
            {
                const auto point=Detail::Point(actual);
                if(!Detail::ValidPoint(bound)||!Detail::ValidPoint(point))return false;
                for(int32 d=0;d<3;++d)if(point.Axis[d].Lower<bound.Axis[d].Lower||point.Axis[d].Upper>bound.Axis[d].Upper)return false;
                return true;
            };
            bool windowCOMCovered=true,windowArmsCovered=true;
            for(float seconds:{nativeDomain->GetFirstFloatSeconds(),.375f,nativeDomain->GetLastFloatSeconds()})
            {
                const auto owner=policyNative.OwnerState.Integrate(seconds).Location;
                windowCOMCovered&=containsOwnerCOM(responseSample->GetModelWindowOwnerCOMHullForInspection(),owner);
                const auto armAtOtherCOM=ContactDetail::Subtract(Detail::Point(farPoint),Detail::Point(owner));
                const auto armDifference=ContactDetail::SquaredNorm(ContactDetail::Subtract(
                    Detail::Point(responseSample->GetActualSubtractedArmForInspection()),armAtOtherCOM));
                windowArmsCovered&=AnalyticCertified::Valid(armDifference)&&
                    responseSample->GetSampleToIdealWindowArmErrorCm()>=FMath::Sqrt(FMath::Max(0.,armDifference.Lower));
            }
            TestTrue(TEXT("same domain window ownerCOM hull covers endpoints and interior original float, no selected TOI"),windowCOMCovered);
            TestTrue(TEXT("sample-at-U arm error covers other-window-COM ideal P arms without pretending their resolver DAG ran"),
                windowArmsCovered&&responseSample->GetSampleToIdealWindowPointErrorCm()>=FMath::Sqrt(FMath::Max(0.,pointGap.Lower)));
            FDistanceBudget noResponseWork{0};auto refusedResponse=responseSample;
            TestTrue(TEXT("response sample budget0 refuses and clears retained conditional values"),
                !FNativeRetainedBoxResponsePointSampleValues::TryBuild(chart,noResponseWork,refusedResponse)&&!refusedResponse);
            VectorSetControlRegister(rangeControls^uint32(0x8000));
            TestFalse(TEXT("response sample cannot transfer across changed FTZ"),responseSample->MatchesInputs(chart,policyNative));
            VectorSetControlRegister(rangeControls);
            if(!TestTrue(TEXT("sole response output may lend its chart across reset"),
                FNativeRetainedBoxResponsePointSampleValues::TryBuild(responseSample->GetChartForInspection(),chartWork,responseSample)&&responseSample))return false;
            FDistanceBudget noChartWork{0};auto refusedChart=chart;
            TestTrue(TEXT("chart budget0 refuses without publishing partial errors"),
                !FNativeRetainedBoxChartSampleValues::TryBuild(nativeDomain,noChartWork,refusedChart)&&!refusedChart);
            VectorSetControlRegister(rangeControls^uint32(0x40));
            TestFalse(TEXT("chart sample does not transfer across changed native DAZ controls"),chart->MatchesInputs(nativeDomain,policyNative));
            VectorSetControlRegister(rangeControls);
            if(!TestTrue(TEXT("sole chart output may lend owned domain across output reset"),
                FNativeRetainedBoxChartSampleValues::TryBuild(chart->GetDomainForInspection(),chartWork,chart)&&chart))return false;
            FDistanceBudget wideDomainWork{8192};auto refusedNativeDomain=nativeDomain;
            TestTrue(TEXT("valid reference-domain seal cannot authorize wider native window that meets UV boundaries"),
                !FNativeRetainedBoxFloatProviderDomainValues::TryProve(bracket,wideDomainWork,refusedNativeDomain)&&!refusedNativeDomain);
            FDistanceBudget lateOtherWork{262144};Speed::FPreparedGeometrySelectionPtr lateOtherGeometry;
            auto mixedNative=policyNative;mixedNative.OwnerState.Location.Z=.865625;mixedNative.BodyState=mixedNative.OwnerState;
            mixedNative.Start=mixedNative.Query.Start=mixedNative.BodyState.Location;
            mixedNative.End=mixedNative.Query.End=mixedNative.OwnerState.Integrate(mixedNative.Horizon).Location;
            FNativeFixedBoxReferenceMotionValuesPtr mixedMotion;FNativeRetainedBoxCornerReferenceValuesPtr mixedCorner;
            FReferenceDyadicCell mixedU; mixedU.Depth=2;mixedU.Index=1;
            FReferenceDyadicCell mixedTime; mixedTime.Depth=3;mixedTime.Index=2;
            FNativeRetainedBoxPreCellClearanceValuesPtr mixedReferencePrefix;
            FNativeRetainedBoxRootDomainClearanceValuesPtr mixedReferenceDomain;
            FNativeRetainedBoxReferenceFirstEntryValuesPtr mixedFirst;
            if(!TestTrue(TEXT("later other provider is clear on reference cell[1/4,3/8] around root.3"),
                FContactUnitRetainedProducer::Build(tilted,lateOtherWork,lateOtherGeometry,true,-.9)&&
                FNativeFixedBoxReferenceMotionValues::TryBuild(mixedNative,lateOtherWork,mixedMotion)&&
                FNativeRetainedBoxCornerReferenceValues::TryProve(lateOtherGeometry,mixedMotion,0,mixedU,lower,mixedTime,3,1,lateOtherWork,mixedCorner)&&
                FNativeRetainedBoxPreCellClearanceValues::TryProve(mixedCorner,lateOtherWork,mixedReferencePrefix)&&
                FNativeRetainedBoxRootDomainClearanceValues::TryProve(mixedCorner,lateOtherWork,mixedReferenceDomain)&&
                FNativeRetainedBoxReferenceFirstEntryValues::TryProve(mixedReferencePrefix,mixedReferenceDomain,lateOtherWork,mixedFirst)))return false;
            FNativeFixedBoxPhysicalFloatRangeClearanceValuesPtr mixedPhysicalPrefix;
            FNativeFixedBoxAtFloatTimeValuesPtr mixedActual;FNativeRetainedBoxAtFloatInteriorValuesPtr mixedInside;
            FNativeRetainedBoxFloatIntersectionBracketValuesPtr mixedBracket;
            if(!TestTrue(TEXT("native insideU25/64 is after other-provider entry yet before selectedUV boundary entry"),
                FNativeFixedBoxPhysicalFloatRangeClearanceValues::TryProve(lateOtherGeometry,mixedMotion,0.f,.25f,lateOtherWork,mixedPhysicalPrefix)&&
                FNativeFixedBoxAtFloatTimeValues::TryBuild(mixedMotion,.390625f,lateOtherWork,mixedActual)&&
                FNativeRetainedBoxAtFloatInteriorValues::TryFind(mixedFirst,mixedActual,lateOtherWork,mixedInside)&&
                FNativeRetainedBoxFloatIntersectionBracketValues::TryJoin(mixedPhysicalPrefix,mixedInside,lateOtherWork,mixedBracket)))return false;
            FDistanceBudget otherNativeDomainWork{16384};refusedNativeDomain=nativeDomain;
            TestTrue(TEXT("nonselected provider met after reference cell still prevents native-window attribution"),
                !FNativeRetainedBoxFloatProviderDomainValues::TryProve(mixedBracket,otherNativeDomainWork,refusedNativeDomain)&&!refusedNativeDomain);
            FDistanceBudget zeroNativeDomainWork{0};refusedNativeDomain=nativeDomain;
            TestTrue(TEXT("native-domain budget0 cannot publish partial provider/boundary coverage"),
                !FNativeRetainedBoxFloatProviderDomainValues::TryProve(tightBracket,zeroNativeDomainWork,refusedNativeDomain)&&!refusedNativeDomain);
            VectorSetControlRegister(rangeControls^uint32(0x8000));
            TestFalse(TEXT("native-domain evidence cannot transfer to another FTZ mode"),nativeDomain->MatchesInputs(
                tightBracket,quarterCorner->GetGeometryForInspection(),policyNative));
            VectorSetControlRegister(rangeControls);
            if(!TestTrue(TEXT("sole native-domain output can lend its owned bracket across reset"),
                FNativeRetainedBoxFloatProviderDomainValues::TryProve(nativeDomain->GetBracketForInspection(),nativeDomainWork,nativeDomain)&&nativeDomain))return false;
            // Clear endpoints do not prove a swept range: this coefficient
            // fixture returns to its original Z after an interior excursion.
            auto returningNative=policyNative;returningNative.OwnerState.Location.Z=3;
            returningNative.OwnerState.Velocity.Z=-16;returningNative.OwnerState.Acceleration.Z=32;
            returningNative.BodyState=returningNative.OwnerState;
            returningNative.Start=returningNative.Query.Start=returningNative.BodyState.Location;
            returningNative.End=returningNative.Query.End=returningNative.OwnerState.Integrate(returningNative.Horizon).Location;
            FNativeFixedBoxReferenceMotionValuesPtr returningMotion;
            if(!TestTrue(TEXT("returning accelerated common fixed Box input"),
                FNativeFixedBoxReferenceMotionValues::TryBuild(returningNative,rangeWork,returningMotion)))return false;

            // Only retained geometry and captured request VALUES are supplied;
            // the producer derives SAME Motion, reference root and float bounds.
            FDistanceBudget factoryWork{1048576};FNativeRetainedBoxFloatIntersectionBracketValuesPtr automaticBracket;
            if(!TestTrue(TEXT("automatic Box bracket consumes capture without provider/corner/cell/time endpoint hints"),
                TryFindNativeRetainedBoxFloatBracket(quarterCorner->GetGeometryForInspection(),policyNative,8192,factoryWork,automaticBracket)&&automaticBracket))return false;
            const auto automaticMotion=automaticBracket->GetPrefixForInspection()->GetMotionForInspection();
            TestTrue(TEXT("automatically derived bracket keeps full capture and encloses known geometric reference fixture3/8"),
                automaticMotion->MatchesCapturedValues(policyNative)&&
                automaticBracket->MatchesInputs(quarterCorner->GetGeometryForInspection(),automaticMotion,policyNative)&&
                automaticBracket->GetClearPrefixSeconds()<.375f&&automaticBracket->GetInsideSeconds()>=.375f&&
                automaticBracket->GetInsideSeconds()<=policyNative.Horizon);
            FDistanceBudget returningFactoryWork{1048576};FNativeRetainedBoxFloatIntersectionBracketValuesPtr returningBracket;
            if(!TestTrue(TEXT("clear final endpoint does not prevent automatic search of an intermediate returning excursion"),
                TryFindNativeRetainedBoxFloatBracket(quarterCorner->GetGeometryForInspection(),returningNative,8192,returningFactoryWork,returningBracket)&&returningBracket))return false;
            const auto returningFactoryMotion=returningBracket->GetPrefixForInspection()->GetMotionForInspection();
            TestTrue(TEXT("returning bracket contains first geometric fixture7/32 and inside time is earlier than clearH"),
                returningBracket->MatchesInputs(quarterCorner->GetGeometryForInspection(),returningFactoryMotion,returningNative)&&
                returningBracket->GetClearPrefixSeconds()<.21875f&&returningBracket->GetInsideSeconds()>=.21875f&&
                returningBracket->GetInsideSeconds()<returningNative.Horizon);
            auto refusedAutomatic=automaticBracket;FDistanceBudget noFactoryWork{0};
            TestTrue(TEXT("automatic factory budget0 clears output without a partial first/inside join"),
                !TryFindNativeRetainedBoxFloatBracket(quarterCorner->GetGeometryForInspection(),policyNative,8192,noFactoryWork,refusedAutomatic)&&!refusedAutomatic);
            refusedAutomatic=automaticBracket;FDistanceBudget invalidFactoryWork{128};
            TestTrue(TEXT("nonpositive technical proof quota refuses without spending"),
                !TryFindNativeRetainedBoxFloatBracket(quarterCorner->GetGeometryForInspection(),policyNative,0,invalidFactoryWork,refusedAutomatic)&&
                !refusedAutomatic&&invalidFactoryWork.Used==0&&invalidFactoryWork.Remaining==128);
            refusedAutomatic=automaticBracket;FDistanceBudget excludedFactoryWork{2048};
            TestTrue(TEXT("mismatching Source query cannot be replaced by automatic indices or a miss"),
                !TryFindNativeRetainedBoxFloatBracket(quarterCorner->GetGeometryForInspection(),changedPolicy,128,excludedFactoryWork,refusedAutomatic)&&!refusedAutomatic);
            FDistanceBudget aliasFactoryWork{1048576};
            if(!TestTrue(TEXT("automatic output may lend owned geometry and captured values across reset"),
                TryFindNativeRetainedBoxFloatBracket(automaticBracket->GetPrefixForInspection()->GetGeometryForInspection(),
                    automaticBracket->GetPrefixForInspection()->GetMotionForInspection()->GetCapturedForInspection(),
                    8192,aliasFactoryWork,automaticBracket)&&automaticBracket))return false;
            FNativeFixedBoxFloatRangeGeometryValuesPtr endpoint0,endpoint1;
            if(!TestTrue(TEXT("both returning-motion endpoints geometrically clear with captured query shells"),
                FNativeFixedBoxFloatRangeGeometryValues::TryProve(quarterCorner->GetGeometryForInspection(),returningMotion,0.f,0.f,rangeWork,endpoint0)&&
                FNativeFixedBoxFloatRangeGeometryValues::TryProve(quarterCorner->GetGeometryForInspection(),returningMotion,1.f,1.f,rangeWork,endpoint1)))return false;
            FDistanceBudget excursionWork{8192};auto refusedRange=rangeGeometry;
            TestTrue(TEXT("interior excursion forbids range clearance despite both endpoints clear"),
                !FNativeFixedBoxFloatRangeGeometryValues::TryProve(quarterCorner->GetGeometryForInspection(),returningMotion,0.f,1.f,excursionWork,refusedRange)&&!refusedRange);
            FDistanceBudget interiorRangeWork{8192};refusedRange=rangeGeometry;
            TestTrue(TEXT("interior finite OTHER provider cannot be omitted by corner-only traces"),
                !FNativeFixedBoxFloatRangeGeometryValues::TryProve(blockedCorner->GetGeometryForInspection(),firstMotion,0.f,.125f,interiorRangeWork,refusedRange)&&!refusedRange);
            auto wrongQuery=policyNative;wrongQuery.Query.HalfExtent.X+=1;
            FNativeFixedBoxReferenceMotionValuesPtr wrongQueryMotion;
            if(!TestTrue(TEXT("fixed pose input can be retained while independent literal query mismatch remains visible"),
                FNativeFixedBoxReferenceMotionValues::TryBuild(wrongQuery,rangeWork,wrongQueryMotion)))return false;
            refusedRange=rangeGeometry;
            TestTrue(TEXT("range proof refuses changed query extent instead of forcing it to physical/captured shape"),
                !FNativeFixedBoxFloatRangeGeometryValues::TryProve(quarterCorner->GetGeometryForInspection(),wrongQueryMotion,0.f,.125f,rangeWork,refusedRange)&&!refusedRange);
            refusedRange=rangeGeometry;
            TestTrue(TEXT("unordered float range refused before geometry"),
                !FNativeFixedBoxFloatRangeGeometryValues::TryProve(quarterCorner->GetGeometryForInspection(),firstMotion,.125f,0.f,rangeWork,refusedRange)&&!refusedRange);
            if(!TestTrue(TEXT("sole range result may lend geometry/Motion/original floats across output reset"),
                FNativeFixedBoxFloatRangeGeometryValues::TryProve(rangeGeometry->GetGeometryForInspection(),rangeGeometry->GetMotionForInspection(),
                    rangeGeometry->GetFirstFloatSeconds(),rangeGeometry->GetLastFloatSeconds(),rangeWork,rangeGeometry)&&rangeGeometry))return false;
            FDistanceBudget noRangeWork{0};refusedRange=rangeGeometry;
            TestTrue(TEXT("budget0 never publishes partial all-provider float-range clearance"),
                !FNativeFixedBoxFloatRangeGeometryValues::TryProve(quarterCorner->GetGeometryForInspection(),firstMotion,0.f,.125f,noRangeWork,refusedRange)&&!refusedRange);
            auto aliasTensor=tilted;
            TestTrue(TEXT("tensor restriction preserves input when output aliases it"),
                RestrictReferenceTensorDyadic(aliasTensor,lower,lower,cellWork,aliasTensor));
            FDistanceBudget noCellWork{0};
            TestFalse(TEXT("exhausted restriction budget clears output"),RestrictReferenceTensorDyadic(tilted,lower,lower,noCellWork,aliasTensor));
            auto composite=native;composite.Horizon=.3f;composite.OwnerState.Location=FVector(12,-8,64);
            composite.OwnerState.Velocity=FVector(3,-2,5);composite.OwnerState.Acceleration=FVector(2,4,-7);
            composite.OwnerState.Rotation=FQuat(FVector(0,0,1),.4);composite.LocalRotation=FQuat(FVector(0,1,0),-.2);
            composite.LocalCOM=FVector(2,-3,5);composite.LocalOffset=FVector(-1,4,2);composite.PhysicalHalfExtent=FVector(3,4,2);
            composite.BodyState=composite.OwnerState;composite.BodyState.Rotation=composite.OwnerState.Rotation*composite.LocalRotation;
            composite.BodyState.Location=(composite.OwnerState.Location-composite.OwnerState.Rotation.RotateVector(composite.LocalCOM))+
                composite.OwnerState.Rotation.RotateVector(composite.LocalOffset);
            if(!TestTrue(TEXT("nonzero COM/offset, distinct owner/localQ and accelerated H=.3f reference motion preserved"),
                FNativeFixedBoxReferenceMotionValues::TryBuild(composite,motionWork,motion)&&motion))return false;
            const auto ownerWindow=motion->BoundConsumedOwnerCOMBetweenFloatSeconds(0.f,composite.Horizon);
            bool compositeOwnersCovered=Detail::ValidPoint(ownerWindow);
            for(float tau:{0.f,.25f,.5f,1.f})
                compositeOwnersCovered&=containsOwnerCOM(ownerWindow,composite.OwnerState.Integrate(composite.Horizon*tau).Location);
            const auto ownerAtZero=motion->BoundConsumedOwnerCOMBetweenFloatSeconds(0.f,0.f);
            TestTrue(TEXT("ownerCOM bound uses accelerated PVA directly and stays distinct from offset body origin"),
                compositeOwnersCovered&&containsOwnerCOM(ownerAtZero,composite.OwnerState.Integrate(0.f).Location)&&
                !containsOwnerCOM(ownerAtZero,composite.BodyState.Location));
            TestFalse(TEXT("ownerCOM window rejects reversed original seconds"),
                Detail::ValidPoint(motion->BoundConsumedOwnerCOMBetweenFloatSeconds(composite.Horizon,0.f)));
            for(float invalid:{KinematicCenterDetail::FromBits(0x80000000u),KinematicCenterDetail::FromBits(1u),
                KinematicCenterDetail::FromBits(0x7fc00000u),KinematicCenterDetail::FromBits(0x7f800000u),
                KinematicCenterDetail::FromBits(KinematicCenterDetail::Bits(composite.Horizon)+1)})
            {
                TestFalse(TEXT("ownerCOM window rejects negativezero/subnormal/NaN/infinite/out-of-H first time"),
                    Detail::ValidPoint(motion->BoundConsumedOwnerCOMBetweenFloatSeconds(invalid,composite.Horizon)));
                TestFalse(TEXT("ownerCOM window rejects negativezero/subnormal/NaN/infinite/out-of-H last time"),
                    Detail::ValidPoint(motion->BoundConsumedOwnerCOMBetweenFloatSeconds(0.f,invalid)));
            }
            FFloorNativeSolidModel independent;
            if(!TestTrue(TEXT("independent existing model closes actual fixed inputs"),BuildFloorNativeSolidModel(composite,independent)))return false;
            for(int32 corner=0;corner<8;++corner)for(float tau:{0.f,.25f,.5f,1.f})
            {
                const float seconds=composite.Horizon*tau;const auto reference=independent.BoundPhysicalCorner(corner,seconds,seconds,false);
                const auto polynomial=CenterDetail::Evaluate(motion->GetCornerPathsForInspection()[corner],Interval::Exact(double(tau)));
                bool overlap=Detail::ValidPoint(reference)&&Detail::ValidPoint(polynomial);
                for(int32 d=0;d<3&&overlap;++d)overlap=reference.Axis[d].Lower<=polynomial.Axis[d].Upper&&polynomial.Axis[d].Lower<=reference.Axis[d].Upper;
                TestTrue(TEXT("each physical XYZ corner reference polynomial agrees with the independently bounded reference motion"),overlap);
            }
            FDistanceBudget compositeSampleWork{8192};FNativeFixedBoxAtFloatTimeValuesPtr compositeSample;
            if(!TestTrue(TEXT("actual provided float DAG keeps nonzero COM/offset/distinctQ/acceleration/H=.3f"),
                FNativeFixedBoxAtFloatTimeValues::TryBuild(motion,composite.Horizon,compositeSampleWork,compositeSample)&&compositeSample))return false;
            const float actualSeconds=compositeSample->GetFloatSeconds();const float nativeSquare=actualSeconds*actualSeconds;
            const double referenceSquare=double(actualSeconds)*double(actualSeconds);
            const double nominalReferenceZ=composite.OwnerState.Location.Z+composite.OwnerState.Velocity.Z*double(actualSeconds)+
                .5*composite.OwnerState.Acceleration.Z*referenceSquare;
            TestTrue(TEXT(".3f square is genuinely binary32 consumed, not promoted reference square"),
                double(nativeSquare)!=referenceSquare&&compositeSample->GetOwnerAtFloatTimeForInspection().Location.Z!=nominalReferenceZ);
            bool allSamplesBounded=true;
            for(int32 k=0;k<8;++k)
            {
                const auto point=Detail::Point(compositeSample->GetPhysicalCornersForInspection()[k]);
                const auto bound=compositeSample->GetConsumedAtSameSecondsForInspection()[k];
                for(int32 d=0;d<3;++d)allSamplesBounded&=point.Axis[d].Lower>=bound.Axis[d].Lower&&point.Axis[d].Upper<=bound.Axis[d].Upper;
                allSamplesBounded&=AnalyticCertified::Valid(compositeSample->GetDifferenceSquaredForInspection()[k])&&
                    compositeSample->GetDifferenceSquaredForInspection()[k].Lower>=0;
            }
            TestTrue(TEXT("ALL8 native corners lie in consumed bounds; reference difference is recorded, never an allowance"),allSamplesBounded);
            TestFalse(TEXT("unrelated common Motion cannot be relabelled as earlier first-reference input"),compositeSample->SharesMotionWithReferenceEntry(firstReference));
            auto signedZero=policyNative;signedZero.OwnerState.Location.X=AnalyticCertified::FromBits(AnalyticCertified::Sign);
            signedZero.OwnerState.Velocity.X=0;signedZero.OwnerState.Acceleration.X=0;signedZero.BodyState=signedZero.OwnerState;
            signedZero.BodyState.Location.X=0;
            FNativeFixedBoxReferenceMotionValuesPtr zeroMotion;FNativeFixedBoxAtFloatTimeValuesPtr zeroSample;
            if(!TestTrue(TEXT("+0 time performs Integrate and COM DAG, never captured-pose shortcut"),
                FNativeFixedBoxReferenceMotionValues::TryBuild(signedZero,compositeSampleWork,zeroMotion)&&
                FNativeFixedBoxAtFloatTimeValues::TryBuild(zeroMotion,0.f,compositeSampleWork,zeroSample)&&zeroSample))return false;
            TestTrue(TEXT("real native additions change captured -0 owner X to +0 at provided +0"),
                AnalyticCertified::Bits(signedZero.OwnerState.Location.X)==AnalyticCertified::Sign&&
                AnalyticCertified::Bits(zeroSample->GetOwnerAtFloatTimeForInspection().Location.X)==0);
            if(!TestTrue(TEXT("sole motion output may lend captured inputs across reset"),
                FNativeFixedBoxReferenceMotionValues::TryBuild(motion->GetCapturedForInspection(),motionWork,motion)&&motion))return false;
            auto rotating=composite;rotating.OwnerState.AngularVelocity.Z=1;rotating.BodyState.AngularVelocity.Z=1;
            TestFalse(TEXT("rotating Box cannot be relabelled fixed reference motion"),FNativeFixedBoxReferenceMotionValues::TryBuild(rotating,motionWork,motion));
            TestFalse(TEXT("invalid fixed motion clears previous result"),bool(motion));
            auto frozen=composite;frozen.Frozen=true;
            TestFalse(TEXT("frozen override not admitted by reference model"),FNativeFixedBoxReferenceMotionValues::TryBuild(frozen,motionWork,motion));
            auto wrongBody=composite;wrongBody.BodyState.Location.X+=1;
            TestFalse(TEXT("incoherent origin/COM/offset body capture refused"),FNativeFixedBoxReferenceMotionValues::TryBuild(wrongBody,motionWork,motion));
            FDistanceBudget noMotionWork{0};
            TestFalse(TEXT("exhausted reference motion budget cannot mint inputs"),FNativeFixedBoxReferenceMotionValues::TryBuild(composite,noMotionWork,motion));
            auto stationary=path;stationary.ControlPoints[1]=stationary.ControlPoints[0];
            TestFalse(TEXT("singular point motion cannot acquire transverse root proof"),
                FTensorPointRootBounds::TryProve(curve,stationary,rootWork,root));
            TestFalse(TEXT("failed root producer clears the old result"),bool(root));
            auto edgePath=path;for(auto& cp:edgePath.ControlPoints)cp.Axis[0]=Interval::Exact(-.5);
            TestFalse(TEXT("boundary root is not an interior root grant"),
                FTensorPointRootBounds::TryProve(curve,edgePath,rootWork,root));
            FDistanceBudget noRootWork{0};
            TestFalse(TEXT("exhausted shared budget cannot prove a partial root"),
                FTensorPointRootBounds::TryProve(curve,path,noRootWork,root));
            TestTrue(TEXT("root producer uses the same finite caller budget without credit"),rootWork.Used>0&&rootWork.Used<=4096&&noRootWork.Used==0);
        }
    }
    TArray<FTensorBezierSurface> surfaces={ContactUnitBowl()};FDistanceBudget budget{131072};
    const auto r=BoundTensorSphereFirstContact(MakeArrayView(surfaces),FVector3d(0,0,1),FVector3d(0,0,-1),0.25,budget,1./65536.,1.e-7,256,0.01,0.05);
    if(Speed::Analytic::InspectCurrentAnalyticNumericContext(Speed::Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush).Status==Speed::Analytic::EAnalyticNumericStatus::Complete)
        UE_LOG(LogTemp,Display,TEXT("[CurvedBowlMath48] Status=%u Failure=%u TimeFailure=%u Temporal=%d Consumer=%d Used=%llu Remaining=%d"),
            uint32(r.Status),uint32(r.Failure),uint32(r.TimeFailure),r.TemporalWidthRefinements,r.ConsumerWidthRefinements,
            static_cast<unsigned long long>(budget.Used),budget.Remaining);
    TestTrue(TEXT("real curved polynomial first contact is enclosed"),r.Status==ETensorSphereContactStatus::FirstContactEnclosed);
    if(r.Status==ETensorSphereContactStatus::FirstContactEnclosed)
    {
        TestTrue(TEXT("first time3/8 is contained"),r.LowerTime<=0.375&&r.UpperTime>=0.375);
        for(int32 d=0;d<3;++d)
        {
            const auto p=r.SurfacePoint.Axis[d],n=r.RadialUnitNormal.Axis[d];const double exactNormal=d==2?1.:0.;
            TestTrue(TEXT("actual curved contact at origin is enclosed"),p.Lower<=0.&&p.Upper>=0.&&p.Upper-p.Lower<=0.01);
            TestTrue(TEXT("actual radial unit normal+Z is enclosed"),n.Lower<=exactNormal&&n.Upper>=exactNormal&&n.Upper-n.Lower<=0.05);
        }
    }
    TestTrue(TEXT("one shared budget covers time/providers/spatial/contact work"),budget.Used>256&&budget.Used<=131072);
    TestTrue(TEXT("original floating context restored"),context.Restore.Restore());return !HasAnyErrors();
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAnalyticTensorFiniteEdgeContactTest,
 "IAmSpeed.AnalyticWorld.TensorContact.OriginalFiniteEdges",
 EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FAnalyticTensorFiniteEdgeContactTest::RunTest(const FString&)
{
    FTensorBoundsTestContext context;if(!TestTrue(TEXT("RNE fixture established"),context.Ready))return false;
    using namespace Speed::Analytic;using namespace Speed::Analytic::Certified;
    // A genuine original edge minimum has a NONZERO inward derivative. Treating
    // it as an interior stationary point would discard the actual first contact.
    FTensorBezierSurface plane;plane.DegreeU=plane.DegreeV=1;
    plane.ControlPoints={FVector3d(0,0,0),FVector3d(0,1,0),FVector3d(1,0,0),FVector3d(1,1,0)};
    TArray<FTensorBezierSurface> surfaces={plane};FDistanceBudget budget{131072};
    const auto r=BoundTensorSphereFirstContact(MakeArrayView(surfaces),FVector3d(-1,0.5,0),FVector3d(0.5,0.5,0),0.5,budget,1./65536.,1.e-7,256,0.01,0.05);
    TestTrue(TEXT("finite original edge is not thrown away by interior stationarity"),r.Status==ETensorSphereContactStatus::FirstContactEnclosed);
    if(r.Status==ETensorSphereContactStatus::FirstContactEnclosed)
    {
        const double time=1./3.;TestTrue(TEXT("true edge first time1/3 is contained"),r.LowerTime<=time&&r.UpperTime>=time);
        const double point[3]={0,0.5,0},normal[3]={-1,0,0};
        for(int32 d=0;d<3;++d)
        {
            TestTrue(TEXT("original edge contact point is enclosed"),r.SurfacePoint.Axis[d].Lower<=point[d]&&r.SurfacePoint.Axis[d].Upper>=point[d]);
            TestTrue(TEXT("edge sphere radial normal is enclosed"),r.RadialUnitNormal.Axis[d].Lower<=normal[d]&&r.RadialUnitNormal.Axis[d].Upper>=normal[d]);
        }
    }
    // On artificial split edges all original-interior stationary conditions stay.
    TestTrue(TEXT("positive derivative cannot introduce a fake original lower edge at u=1/2"),!ContactDetail::CanContainMinimum(Interval{1,2,true},0.5,1.));
    TestTrue(TEXT("negative derivative cannot introduce a fake original upper edge at u=1/2"),!ContactDetail::CanContainMinimum(Interval{-2,-1,true},0.,0.5));
    TestTrue(TEXT("original lower KKT is allowed"),ContactDetail::CanContainMinimum(Interval{1,2,true},0.,0.5));
    TestTrue(TEXT("original upper KKT is allowed"),ContactDetail::CanContainMinimum(Interval{-2,-1,true},0.5,1.));
    TestTrue(TEXT("original floating context restored"),context.Restore.Restore());return !HasAnyErrors();
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAnalyticTensorContactAmbiguityTest,
 "IAmSpeed.AnalyticWorld.TensorContact.AmbiguityAndBudget",
 EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FAnalyticTensorContactAmbiguityTest::RunTest(const FString&)
{
    FTensorBoundsTestContext context;if(!TestTrue(TEXT("RNE fixture established"),context.Ready))return false;
    using namespace Speed::Analytic;using namespace Speed::Analytic::Certified;
    FTensorBezierSurface left,right;left.ControlPoints.Add(FVector3d(-0.25,0,0));right.ControlPoints.Add(FVector3d(0.25,0,0));
    TArray<FTensorBezierSurface> surfaces={left,right};FDistanceBudget temporal{8192};
    const auto bracket=BoundTensorSphereSweep(MakeArrayView(surfaces),FVector3d(0,0,1),FVector3d(0,0,-1),0.5,temporal,1./65536.,1.e-7,1);
    TestTrue(TEXT("ambiguity fixture reaches a temporal bracket before contact filtering"),bracket.Status==ETensorSphereTimeStatus::FirstContactBracket&&temporal.Remaining>0);
    FDistanceBudget budget{8192};
    const auto r=BoundTensorSphereFirstContact(MakeArrayView(surfaces),FVector3d(0,0,1),FVector3d(0,0,-1),0.5,budget,1./65536.,1.e-7,1,0.01,0.05);
    TestTrue(TEXT("two separated simultaneous contacts never collapse to one provider"),r.Status==ETensorSphereContactStatus::Unknown&&budget.Remaining>0);
    FDistanceBudget empty{0};const auto stopped=BoundTensorSphereFirstContact(MakeArrayView(surfaces),FVector3d(0,0,1),FVector3d(0,0,-1),0.5,empty,1./65536.,1.e-7,1,0.01,0.05);
    TestTrue(TEXT("shared exhausted budget cannot certify a partial candidate hull"),stopped.Status==ETensorSphereContactStatus::Unknown&&empty.Used==0);
    FDistanceBudget invalid{32};const auto zero=BoundTensorSphereFirstContact(MakeArrayView(surfaces),FVector3d(0,0,1),FVector3d(0,0,-1),0.,invalid,1./65536.,1.e-7,1,0.01,0.05);
    TestTrue(TEXT("zero radius cannot produce a unit radial normal"),zero.Status==ETensorSphereContactStatus::Unknown&&invalid.Used==0);
    TestTrue(TEXT("original floating context restored"),context.Restore.Restore());return !HasAnyErrors();
}
namespace
{
FAutoConsoleCommand TensorContactUnits(TEXT("Speed.Analytic.RunTensorContactUnits"),
    TEXT("Run three bounded real-tensor first-contact enclosure tests, no World admission."),
    FConsoleCommandDelegate::CreateLambda([]()
    {
        static bool Used=false;auto& Framework=FAutomationTestFramework::Get();
        if(!IsRunningCommandlet()||!FParse::Param(FCommandLine::Get(),TEXT("AnalyticTensorContactUnits"))||Used||GIsAutomationTesting||GIsSlowTask||GIsPlayInEditorWorld||Framework.GetCurrentTest()||!Framework.IsLatentCommandQueueEmpty())
        {UE_LOG(LogTemp,Error,TEXT("Analytic tensor contact commandlet refused"));return;}
        Used=true;
        const TCHAR* Classes[]={TEXT("FAnalyticTensorCurvedContactTest"),TEXT("FAnalyticTensorFiniteEdgeContactTest"),TEXT("FAnalyticTensorContactAmbiguityTest")};
        const TCHAR* Names[]={TEXT("IAmSpeed.AnalyticWorld.TensorContact.CurvedPointAndNormal"),TEXT("IAmSpeed.AnalyticWorld.TensorContact.OriginalFiniteEdges"),TEXT("IAmSpeed.AnalyticWorld.TensorContact.AmbiguityAndBudget")};
        for(int32 i=0;i<3;++i)
        {
            if(GIsAutomationTesting||GIsSlowTask||GIsPlayInEditorWorld||Framework.GetCurrentTest()||!Framework.IsLatentCommandQueueEmpty()||!Framework.ContainsTest(Classes[i]))
            {UE_LOG(LogTemp,Error,TEXT("Analytic tensor contact test ownership refused"));return;}
            Framework.SetRequestedTestFilter(EAutomationTestFlags::EngineFilter);Framework.StartTestByName(Classes[i],0,Names[i]);
            if(!GIsAutomationTesting||!Framework.GetCurrentTest()||Framework.GetCurrentTestFullPath()!=Names[i]||!Framework.IsLatentCommandQueueEmpty())
            {UE_LOG(LogTemp,Error,TEXT("Analytic tensor contact synchronous ownership refused"));return;}
            FAutomationTestExecutionInfo info;const bool stopped=Framework.StopTest(info);
            if(GIsAutomationTesting||GIsSlowTask||GIsPlayInEditorWorld||Framework.GetCurrentTest()||!Framework.IsLatentCommandQueueEmpty())
            {UE_LOG(LogTemp,Error,TEXT("Analytic tensor contact final ownership refused"));return;}
            const bool passed=stopped&&info.bSuccessful&&info.GetErrorTotal()==0&&info.GetWarningTotal()==0;
            UE_LOG(LogTemp,Display,TEXT("ANALYTIC_TENSOR_CONTACT_UNIT name=%s passed=%d errors=%d warnings=%d"),Names[i],passed,info.GetErrorTotal(),info.GetWarningTotal());
        }
    }));
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTensorCapturedCeilingProbe1501Test,
    "IAmSpeed.AnalyticWorld.TensorContact.CapturedCeilingProbe1501",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FTensorCapturedCeilingProbe1501Test::RunTest(const FString&)
{
    using namespace Speed::Analytic;using namespace Speed::Analytic::Certified;
    FTensorBoundsTestContext context;if(!TestTrue(TEXT("RNE fixture"),context.Ready))return false;
    // Exact CP intervals from packet090d4115: provider373 then2834..2859,
    // 140CP/6720B concat SHA b599ffdaa436fa492c342e0d5c6cb9ecc2a1fb156a0b7d769808d39124456755.
    // This is a captured27-provider MATH subset, NOT a2992World/cost/owner,
    // normal-quantizer, actual-wheel callback or physical-hit certificate.
    static constexpr uint64 bits[140][6]={
        {0x40AC1FD4DC8E8475ULL,0x40AC1FD4DC8E8476ULL,0x40AB800000000000ULL,0x40AB800000000000ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40AC1FD4DC8E8475ULL,0x40AC1FD4DC8E8476ULL,0x40AB8CCCCCCCCCCCULL,0x40AB8CCCCCCCCCCDULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40AC1FD4DC8E8475ULL,0x40AC1FD4DC8E8476ULL,0x40AB999999999999ULL,0x40AB99999999999AULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40AC1FD4DC8E8475ULL,0x40AC1FD4DC8E8476ULL,0x40ABA66666666666ULL,0x40ABA66666666667ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40AC1FD4DC8E8475ULL,0x40AC1FD4DC8E8476ULL,0x40ABB33333333333ULL,0x40ABB33333333334ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40AC1FD4DC8E8475ULL,0x40AC1FD4DC8E8476ULL,0x40ABC00000000000ULL,0x40ABC00000000000ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40A67FDD7D3ED05EULL,0x40A67FDD7D3ED05FULL,0x40AB800000000000ULL,0x40AB800000000000ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40A67FDD7D3ED05EULL,0x40A67FDD7D3ED05FULL,0x40AB8CCCCCCCCCCCULL,0x40AB8CCCCCCCCCCDULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40A67FDD7D3ED05EULL,0x40A67FDD7D3ED05FULL,0x40AB999999999999ULL,0x40AB99999999999AULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40A67FDD7D3ED05EULL,0x40A67FDD7D3ED05FULL,0x40ABA66666666666ULL,0x40ABA66666666667ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40A67FDD7D3ED05EULL,0x40A67FDD7D3ED05FULL,0x40ABB33333333333ULL,0x40ABB33333333334ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40A67FDD7D3ED05EULL,0x40A67FDD7D3ED05FULL,0x40ABC00000000000ULL,0x40ABC00000000000ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40A0DFE61DEF1C46ULL,0x40A0DFE61DEF1C47ULL,0x40AB800000000000ULL,0x40AB800000000000ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40A0DFE61DEF1C46ULL,0x40A0DFE61DEF1C47ULL,0x40AB8CCCCCCCCCCCULL,0x40AB8CCCCCCCCCCDULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40A0DFE61DEF1C46ULL,0x40A0DFE61DEF1C47ULL,0x40AB999999999999ULL,0x40AB99999999999AULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40A0DFE61DEF1C46ULL,0x40A0DFE61DEF1C47ULL,0x40ABA66666666666ULL,0x40ABA66666666667ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40A0DFE61DEF1C46ULL,0x40A0DFE61DEF1C47ULL,0x40ABB33333333333ULL,0x40ABB33333333334ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40A0DFE61DEF1C46ULL,0x40A0DFE61DEF1C47ULL,0x40ABC00000000000ULL,0x40ABC00000000000ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40967FDD7D3ED05EULL,0x40967FDD7D3ED05FULL,0x40AB800000000000ULL,0x40AB800000000000ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40967FDD7D3ED05EULL,0x40967FDD7D3ED05FULL,0x40AB8CCCCCCCCCCCULL,0x40AB8CCCCCCCCCCDULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40967FDD7D3ED05EULL,0x40967FDD7D3ED05FULL,0x40AB999999999999ULL,0x40AB99999999999AULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40967FDD7D3ED05EULL,0x40967FDD7D3ED05FULL,0x40ABA66666666666ULL,0x40ABA66666666667ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40967FDD7D3ED05EULL,0x40967FDD7D3ED05FULL,0x40ABB33333333333ULL,0x40ABB33333333334ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40967FDD7D3ED05EULL,0x40967FDD7D3ED05FULL,0x40ABC00000000000ULL,0x40ABC00000000000ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40867FDD7D3ED05EULL,0x40867FDD7D3ED05FULL,0x40AB800000000000ULL,0x40AB800000000000ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40867FDD7D3ED05EULL,0x40867FDD7D3ED05FULL,0x40AB8CCCCCCCCCCCULL,0x40AB8CCCCCCCCCCDULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40867FDD7D3ED05EULL,0x40867FDD7D3ED05FULL,0x40AB999999999999ULL,0x40AB99999999999AULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40867FDD7D3ED05EULL,0x40867FDD7D3ED05FULL,0x40ABA66666666666ULL,0x40ABA66666666667ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40867FDD7D3ED05EULL,0x40867FDD7D3ED05FULL,0x40ABB33333333333ULL,0x40ABB33333333334ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40867FDD7D3ED05EULL,0x40867FDD7D3ED05FULL,0x40ABC00000000000ULL,0x40ABC00000000000ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x0000000000000000ULL,0x0000000000000000ULL,0x40AB800000000000ULL,0x40AB800000000000ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x0000000000000000ULL,0x0000000000000000ULL,0x40AB8CCCCCCCCCCCULL,0x40AB8CCCCCCCCCCDULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x0000000000000000ULL,0x0000000000000000ULL,0x40AB999999999999ULL,0x40AB99999999999AULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x0000000000000000ULL,0x0000000000000000ULL,0x40ABA66666666666ULL,0x40ABA66666666667ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x0000000000000000ULL,0x0000000000000000ULL,0x40ABB33333333333ULL,0x40ABB33333333334ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x0000000000000000ULL,0x0000000000000000ULL,0x40ABC00000000000ULL,0x40ABC00000000000ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x0000000000000000ULL,0x0000000000000000ULL,0x40AB800000000000ULL,0x40AB800000000000ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40AC99D0B2A4EC7AULL,0x40AC99D0B2A4EC7BULL,0x40AB73FC539AD08FULL,0x40AB73FC539AD090ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40AC1FD4DC8E8475ULL,0x40AC1FD4DC8E8476ULL,0x40AB800000000000ULL,0x40AB800000000000ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40AC1FD4DC8E8475ULL,0x40AC1FD4DC8E8476ULL,0x40AB800000000000ULL,0x40AB800000000000ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0xC0AC1FD4DC8E8476ULL,0xC0AC1FD4DC8E8475ULL,0x40AB800000000000ULL,0x40AB800000000000ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40AC99D0B2A4EC7AULL,0x40AC99D0B2A4EC7BULL,0x40AB73FC539AD08FULL,0x40AB73FC539AD090ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x0000000000000000ULL,0x0000000000000000ULL,0x40AB800000000000ULL,0x40AB800000000000ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x0000000000000000ULL,0x0000000000000000ULL,0x40AB800000000000ULL,0x40AB800000000000ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0xC0AC1FD4DC8E8476ULL,0xC0AC1FD4DC8E8475ULL,0x40AB800000000000ULL,0x40AB800000000000ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40AD0F1C774373B4ULL,0x40AD0F1C774373B5ULL,0x40AB506780B2D534ULL,0x40AB506780B2D535ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40AC99D0B2A4EC7AULL,0x40AC99D0B2A4EC7BULL,0x40AB73FC539AD08FULL,0x40AB73FC539AD090ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40AC99D0B2A4EC7AULL,0x40AC99D0B2A4EC7BULL,0x40AB73FC539AD08FULL,0x40AB73FC539AD090ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0xC0AC1FD4DC8E8476ULL,0xC0AC1FD4DC8E8475ULL,0x40AB800000000000ULL,0x40AB800000000000ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40AD7B36372425EBULL,0x40AD7B36372425ECULL,0x40AB169F934FBED1ULL,0x40AB169F934FBED2ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40AD0F1C774373B4ULL,0x40AD0F1C774373B5ULL,0x40AB506780B2D534ULL,0x40AB506780B2D535ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40AD0F1C774373B4ULL,0x40AD0F1C774373B5ULL,0x40AB506780B2D534ULL,0x40AB506780B2D535ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0xC0AC1FD4DC8E8476ULL,0xC0AC1FD4DC8E8475ULL,0x40AB800000000000ULL,0x40AB800000000000ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40ADD9F675B04F7CULL,0x40ADD9F675B04F7DULL,0x40AAC8DCFD7BE7B2ULL,0x40AAC8DCFD7BE7B3ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40AD7B36372425EBULL,0x40AD7B36372425ECULL,0x40AB169F934FBED1ULL,0x40AB169F934FBED2ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40AD7B36372425EBULL,0x40AD7B36372425ECULL,0x40AB169F934FBED1ULL,0x40AB169F934FBED2ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0xC0AC1FD4DC8E8476ULL,0xC0AC1FD4DC8E8475ULL,0x40AB800000000000ULL,0x40AB800000000000ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40B163FBB31ED722ULL,0x40B163FBB31ED723ULL,0x40A5DADC0CEE88E9ULL,0x40A5DADC0CEE88EAULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40ADD9F675B04F7CULL,0x40ADD9F675B04F7DULL,0x40AAC8DCFD7BE7B2ULL,0x40AAC8DCFD7BE7B3ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40ADD9F675B04F7CULL,0x40ADD9F675B04F7DULL,0x40AAC8DCFD7BE7B2ULL,0x40AAC8DCFD7BE7B3ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0xC0AC1FD4DC8E8476ULL,0xC0AC1FD4DC8E8475ULL,0x40AB800000000000ULL,0x40AB800000000000ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40B18B0DBC0E347CULL,0x40B18B0DBC0E347DULL,0x40A57BA5055BC39EULL,0x40A57BA5055BC39FULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40B163FBB31ED722ULL,0x40B163FBB31ED723ULL,0x40A5DADC0CEE88E9ULL,0x40A5DADC0CEE88EAULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40B163FBB31ED722ULL,0x40B163FBB31ED723ULL,0x40A5DADC0CEE88E9ULL,0x40A5DADC0CEE88EAULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0xC0AC1FD4DC8E8476ULL,0xC0AC1FD4DC8E8475ULL,0x40AB800000000000ULL,0x40AB800000000000ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40B1A815EAC15A80ULL,0x40B1A815EAC15A81ULL,0x40A50F03C012CDA5ULL,0x40A50F03C012CDA6ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40B18B0DBC0E347CULL,0x40B18B0DBC0E347DULL,0x40A57BA5055BC39EULL,0x40A57BA5055BC39FULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40B18B0DBC0E347CULL,0x40B18B0DBC0E347DULL,0x40A57BA5055BC39EULL,0x40A57BA5055BC39FULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0xC0AC1FD4DC8E8476ULL,0xC0AC1FD4DC8E8475ULL,0x40AB800000000000ULL,0x40AB800000000000ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40B1B9F6A1E1B383ULL,0x40B1B9F6A1E1B384ULL,0x40A49924EEE9E3BAULL,0x40A49924EEE9E3BBULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40B1A815EAC15A80ULL,0x40B1A815EAC15A81ULL,0x40A50F03C012CDA5ULL,0x40A50F03C012CDA6ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40B1A815EAC15A80ULL,0x40B1A815EAC15A81ULL,0x40A50F03C012CDA5ULL,0x40A50F03C012CDA6ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0xC0AC1FD4DC8E8476ULL,0xC0AC1FD4DC8E8475ULL,0x40AB800000000000ULL,0x40AB800000000000ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40B1C00000000000ULL,0x40B1C00000000000ULL,0x40A41E902BCF92F5ULL,0x40A41E902BCF92F6ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40B1B9F6A1E1B383ULL,0x40B1B9F6A1E1B384ULL,0x40A49924EEE9E3BAULL,0x40A49924EEE9E3BBULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40B1B9F6A1E1B383ULL,0x40B1B9F6A1E1B384ULL,0x40A49924EEE9E3BAULL,0x40A49924EEE9E3BBULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0xC0AC1FD4DC8E8476ULL,0xC0AC1FD4DC8E8475ULL,0x40AB800000000000ULL,0x40AB800000000000ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40B1C00000000000ULL,0x40B1C00000000000ULL,0x40A21E902BCF92F5ULL,0x40A21E902BCF92F6ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40B1C00000000000ULL,0x40B1C00000000000ULL,0x40A41E902BCF92F5ULL,0x40A41E902BCF92F6ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40B1C00000000000ULL,0x40B1C00000000000ULL,0x40A41E902BCF92F5ULL,0x40A41E902BCF92F6ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0xC0AC1FD4DC8E8476ULL,0xC0AC1FD4DC8E8475ULL,0x40AB800000000000ULL,0x40AB800000000000ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40B1C00000000000ULL,0x40B1C00000000000ULL,0x407BCC147AE147AEULL,0x407BCC147AE147AFULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40B1C00000000000ULL,0x40B1C00000000000ULL,0x40A21E902BCF92F5ULL,0x40A21E902BCF92F6ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40B1C00000000000ULL,0x40B1C00000000000ULL,0x40A21E902BCF92F5ULL,0x40A21E902BCF92F6ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0xC0AC1FD4DC8E8476ULL,0xC0AC1FD4DC8E8475ULL,0x40AB800000000000ULL,0x40AB800000000000ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40B1C00000000000ULL,0x40B1C00000000000ULL,0x0000000000000000ULL,0x0000000000000000ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40B1C00000000000ULL,0x40B1C00000000000ULL,0x407BCC147AE147AEULL,0x407BCC147AE147AFULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40B1C00000000000ULL,0x40B1C00000000000ULL,0x407BCC147AE147AEULL,0x407BCC147AE147AFULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0xC0AC1FD4DC8E8476ULL,0xC0AC1FD4DC8E8475ULL,0x40AB800000000000ULL,0x40AB800000000000ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40B1C00000000000ULL,0x40B1C00000000000ULL,0xC07BCC147AE147AFULL,0xC07BCC147AE147AEULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40B1C00000000000ULL,0x40B1C00000000000ULL,0x0000000000000000ULL,0x0000000000000000ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40B1C00000000000ULL,0x40B1C00000000000ULL,0x0000000000000000ULL,0x0000000000000000ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0xC0AC1FD4DC8E8476ULL,0xC0AC1FD4DC8E8475ULL,0x40AB800000000000ULL,0x40AB800000000000ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40B1C00000000000ULL,0x40B1C00000000000ULL,0xC0A21E902BCF92F6ULL,0xC0A21E902BCF92F5ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40B1C00000000000ULL,0x40B1C00000000000ULL,0xC07BCC147AE147AFULL,0xC07BCC147AE147AEULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40B1C00000000000ULL,0x40B1C00000000000ULL,0xC07BCC147AE147AFULL,0xC07BCC147AE147AEULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0xC0AC1FD4DC8E8476ULL,0xC0AC1FD4DC8E8475ULL,0x40AB800000000000ULL,0x40AB800000000000ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40B1C00000000000ULL,0x40B1C00000000000ULL,0xC0A41E902BCF92F6ULL,0xC0A41E902BCF92F5ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40B1C00000000000ULL,0x40B1C00000000000ULL,0xC0A21E902BCF92F6ULL,0xC0A21E902BCF92F5ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40B1C00000000000ULL,0x40B1C00000000000ULL,0xC0A21E902BCF92F6ULL,0xC0A21E902BCF92F5ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0xC0AC1FD4DC8E8476ULL,0xC0AC1FD4DC8E8475ULL,0x40AB800000000000ULL,0x40AB800000000000ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40B1B9F6A1E1B383ULL,0x40B1B9F6A1E1B384ULL,0xC0A49924EEE9E3BBULL,0xC0A49924EEE9E3BAULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40B1C00000000000ULL,0x40B1C00000000000ULL,0xC0A41E902BCF92F6ULL,0xC0A41E902BCF92F5ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40B1C00000000000ULL,0x40B1C00000000000ULL,0xC0A41E902BCF92F6ULL,0xC0A41E902BCF92F5ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0xC0AC1FD4DC8E8476ULL,0xC0AC1FD4DC8E8475ULL,0x40AB800000000000ULL,0x40AB800000000000ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40B1A815EAC15A80ULL,0x40B1A815EAC15A81ULL,0xC0A50F03C012CDA6ULL,0xC0A50F03C012CDA5ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40B1B9F6A1E1B383ULL,0x40B1B9F6A1E1B384ULL,0xC0A49924EEE9E3BBULL,0xC0A49924EEE9E3BAULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40B1B9F6A1E1B383ULL,0x40B1B9F6A1E1B384ULL,0xC0A49924EEE9E3BBULL,0xC0A49924EEE9E3BAULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0xC0AC1FD4DC8E8476ULL,0xC0AC1FD4DC8E8475ULL,0x40AB800000000000ULL,0x40AB800000000000ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40B18B0DBC0E347CULL,0x40B18B0DBC0E347DULL,0xC0A57BA5055BC39FULL,0xC0A57BA5055BC39EULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40B1A815EAC15A80ULL,0x40B1A815EAC15A81ULL,0xC0A50F03C012CDA6ULL,0xC0A50F03C012CDA5ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40B1A815EAC15A80ULL,0x40B1A815EAC15A81ULL,0xC0A50F03C012CDA6ULL,0xC0A50F03C012CDA5ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0xC0AC1FD4DC8E8476ULL,0xC0AC1FD4DC8E8475ULL,0x40AB800000000000ULL,0x40AB800000000000ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40B163FBB31ED722ULL,0x40B163FBB31ED723ULL,0xC0A5DADC0CEE88EAULL,0xC0A5DADC0CEE88E9ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40B18B0DBC0E347CULL,0x40B18B0DBC0E347DULL,0xC0A57BA5055BC39FULL,0xC0A57BA5055BC39EULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40B18B0DBC0E347CULL,0x40B18B0DBC0E347DULL,0xC0A57BA5055BC39FULL,0xC0A57BA5055BC39EULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0xC0AC1FD4DC8E8476ULL,0xC0AC1FD4DC8E8475ULL,0x40AB800000000000ULL,0x40AB800000000000ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40ADD9F675B04F7CULL,0x40ADD9F675B04F7DULL,0xC0AAC8DCFD7BE7B3ULL,0xC0AAC8DCFD7BE7B2ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40B163FBB31ED722ULL,0x40B163FBB31ED723ULL,0xC0A5DADC0CEE88EAULL,0xC0A5DADC0CEE88E9ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40B163FBB31ED722ULL,0x40B163FBB31ED723ULL,0xC0A5DADC0CEE88EAULL,0xC0A5DADC0CEE88E9ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0xC0AC1FD4DC8E8476ULL,0xC0AC1FD4DC8E8475ULL,0x40AB800000000000ULL,0x40AB800000000000ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40AD7B36372425EBULL,0x40AD7B36372425ECULL,0xC0AB169F934FBED2ULL,0xC0AB169F934FBED1ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40ADD9F675B04F7CULL,0x40ADD9F675B04F7DULL,0xC0AAC8DCFD7BE7B3ULL,0xC0AAC8DCFD7BE7B2ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40ADD9F675B04F7CULL,0x40ADD9F675B04F7DULL,0xC0AAC8DCFD7BE7B3ULL,0xC0AAC8DCFD7BE7B2ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0xC0AC1FD4DC8E8476ULL,0xC0AC1FD4DC8E8475ULL,0x40AB800000000000ULL,0x40AB800000000000ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40AD0F1C774373B4ULL,0x40AD0F1C774373B5ULL,0xC0AB506780B2D535ULL,0xC0AB506780B2D534ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40AD7B36372425EBULL,0x40AD7B36372425ECULL,0xC0AB169F934FBED2ULL,0xC0AB169F934FBED1ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40AD7B36372425EBULL,0x40AD7B36372425ECULL,0xC0AB169F934FBED2ULL,0xC0AB169F934FBED1ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0xC0AC1FD4DC8E8476ULL,0xC0AC1FD4DC8E8475ULL,0x40AB800000000000ULL,0x40AB800000000000ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40AC99D0B2A4EC7AULL,0x40AC99D0B2A4EC7BULL,0xC0AB73FC539AD090ULL,0xC0AB73FC539AD08FULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40AD0F1C774373B4ULL,0x40AD0F1C774373B5ULL,0xC0AB506780B2D535ULL,0xC0AB506780B2D534ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40AD0F1C774373B4ULL,0x40AD0F1C774373B5ULL,0xC0AB506780B2D535ULL,0xC0AB506780B2D534ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0xC0AC1FD4DC8E8476ULL,0xC0AC1FD4DC8E8475ULL,0x40AB800000000000ULL,0x40AB800000000000ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40AC1FD4DC8E8475ULL,0x40AC1FD4DC8E8476ULL,0xC0AB800000000000ULL,0xC0AB800000000000ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40AC99D0B2A4EC7AULL,0x40AC99D0B2A4EC7BULL,0xC0AB73FC539AD090ULL,0xC0AB73FC539AD08FULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40AC99D0B2A4EC7AULL,0x40AC99D0B2A4EC7BULL,0xC0AB73FC539AD090ULL,0xC0AB73FC539AD08FULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0xC0AC1FD4DC8E8476ULL,0xC0AC1FD4DC8E8475ULL,0x40AB800000000000ULL,0x40AB800000000000ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x0000000000000000ULL,0x0000000000000000ULL,0xC0AB800000000000ULL,0xC0AB800000000000ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40AC1FD4DC8E8475ULL,0x40AC1FD4DC8E8476ULL,0xC0AB800000000000ULL,0xC0AB800000000000ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
        {0x40AC1FD4DC8E8475ULL,0x40AC1FD4DC8E8476ULL,0xC0AB800000000000ULL,0xC0AB800000000000ULL,0x40A0000000000000ULL,0x40A0000000000000ULL},
    };
    TArray<FIntervalTensorSurface> surfaces;int32 cursor=0;
    for(int32 face=0;face<27;++face)
    {
        FIntervalTensorSurface surface;surface.DegreeU=surface.DegreeV=face?1:5;
        for(int32 k=0;k<(face?4:36);++k,++cursor)
        {
            FIntervalPoint point;
            for(int32 d=0;d<3;++d)point.Axis[d]={AnalyticCertified::FromBits(bits[cursor][d*2]),AnalyticCertified::FromBits(bits[cursor][d*2+1]),true};
            if(!TestTrue(TEXT("authentic finite interval CP"),Detail::ValidPoint(point)))return false;
            surface.ControlPoints.Add(point);
        }
        surfaces.Add(MoveTemp(surface));
    }
    if(!TestTrue(TEXT("all27captured providers and140CP retained"),surfaces.Num()==27&&cursor==140))return false;
    struct FCaptured {FVector3d Start,End;double TimeGap;};
    const FCaptured captured[]={
        {FVector3d(81.379153170335854,3519.9917403854815,2023.5473112824643),FVector3d(81.30572442163492,3519.4662731805356,2047.5514489742045),3.1754940982244103e-6},
        {FVector3d(81.379267150310383,3519.991765901394,2023.5473082520084),FVector3d(81.305836567716057,3519.4663006213009,2047.5514459802746),3.1754941052500844e-6}};
    const double radius=17.010000228881836;
    const uint32 original=VectorGetControlRegister();
    for(const auto& capture:captured)for(uint32 flush:{0u,64u,32768u,32832u})
    {
        VectorSetControlRegister((original&~uint32(0x8040))|flush);
        // Same nominal1054631 ceiling as captured query; setup overhead of
        // full2992World is NOT reproduced or silently claimed equivalent.
        FDistanceBudget budget{1054631};
        const uint32 initialMemoFlags=VectorGetControlRegister();
        const auto result=BoundTensorSphereFirstContact(surfaces,capture.Start,capture.End,radius,budget,
            capture.TimeGap,0.,8192,.001250028610229492,.00012500000593718141,24);
        const uint32 expectedMemoFlags=VectorGetControlRegister();
        ContactDetail::FContactCellGeometryMemo memo;FIntervalBezierCenterPath memoPath;
        if(!TestTrue(TEXT("captured fixed path"),CenterDetail::Linear(capture.Start,capture.End,memoPath)))return false;
        for(int32 repeat=0;repeat<2;++repeat)
        {
            VectorSetControlRegister(initialMemoFlags);FDistanceBudget memoBudget{1054631};
            const auto memoResult=BoundTensorSphereFirstContact(surfaces,memoPath,radius,memoBudget,
                capture.TimeGap,0.,8192,.001250028610229492,.00012500000593718141,24,nullptr,&memo);
            const uint32 memoFlags=VectorGetControlRegister();
            TestTrue(TEXT("captured1501 cold/warm full oracle including limits and flags"),ContactMemo71Signature(memoResult)==ContactMemo71Signature(result)&&
                memoBudget.Used==budget.Used&&memoBudget.Remaining==budget.Remaining&&memoFlags==expectedMemoFlags);
        }
        VectorSetControlRegister(expectedMemoFlags);
        AddInfo(FString::Printf(TEXT("CapturedCeiling1501 subset27 FTZ=%u Status=%u Failure=%u TimeFailure=%u Used=%llu Remaining=%d TimeWork=%llu NonTimeWork=%llu Cells=%llu SplitU=%llu SplitV=%llu Site=%s Temporal=%d Consumer=%d WidthDiag=%d WidthSurface=%d WidthUV=[%.17g,%.17g]x[%.17g,%.17g]"),
            flush,uint32(result.Status),uint32(result.Failure),uint32(result.TimeFailure),budget.Used,budget.Remaining,
            result.TimeWorkUsed,budget.Used-result.CallInitialUsed-result.TimeWorkUsed,result.SpatialVisitedCells,result.SpatialSplitsU,result.SpatialSplitsV,result.WorkSite,result.TemporalWidthRefinements,result.ConsumerWidthRefinements,int32(result.HasWidthDiagnostic),result.WidthSurface,result.WidthMinU,result.WidthMaxU,result.WidthMinV,result.WidthMaxV));
        TestTrue(TEXT("captured ceiling math closes under unchanged nominal budget and targets"),
            result.Status==ETensorSphereContactStatus::FirstContactEnclosed&&result.Failure==ETensorSphereContactFailure::None&&!result.Leaves.IsEmpty()&&result.CollisionRadiusBits==0x4031028F60000000ULL);
        TestTrue(TEXT("passive Time accounting stays within same nonrefunded call"),result.HasCallAccounting&&result.CallInitialUsed<=budget.Used&&result.TimeWorkUsed<=budget.Used-result.CallInitialUsed);
    }
    VectorSetControlRegister(original);return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTensorPlanarProjectionDomainsTest,
    "IAmSpeed.AnalyticWorld.TensorContact.PlanarProjectionDomains",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FTensorPlanarProjectionDomainsTest::RunTest(const FString&)
{
    using namespace Speed::Analytic;using namespace Speed::Analytic::Certified;
    FTensorBoundsTestContext context;if(!TestTrue(TEXT("RNE fixture"),context.Ready))return false;
    const uint32 original=VectorGetControlRegister();
    for(uint32 flush:{0u,64u,32768u,32832u})
    {
        VectorSetControlRegister((original&~uint32(0x8040))|flush);
        const auto triangle=[](double shift)
        {
            FIntervalTensorSurface s;s.DegreeU=s.DegreeV=1;
            for(const auto& p:{FVector3d(shift,0,0),FVector3d(shift+3,0,0),FVector3d(shift,3,0),FVector3d(shift,3,0)})s.ControlPoints.Add(Detail::Point(p));
            return s;
        };
        TArray<FIntervalTensorSurface> surfaces{triangle(0),triangle(20)};
        auto center=Detail::Point(FVector3d(1,1,-2));FDistanceBudget budget{8192};ContactDetail::FPlanarProjectionDomains proof;
        if(!TestTrue(TEXT("finite interior witness closes whole supplied planar union"),ContactDetail::ProjectedPlanarDomains(surfaces,center,budget,proof)==ContactDetail::EPlanarProjectionProof::Proved))return false;
        TestTrue(TEXT("actual inverse originalUV retained, other finite triangle excluded"),proof.NormalAxis==2&&proof.Providers.Num()==2&&proof.Providers[0].Bounded&&!proof.Providers[0].Outside&&proof.Providers[1].Outside&&
            proof.Providers[0].U.Lower<=1./3.&&proof.Providers[0].U.Upper>=1./3.&&proof.Providers[0].V.Lower<=.5&&proof.Providers[0].V.Upper>=.5);
        auto small=triangle(0);
        small.ControlPoints[1]=Detail::Point(FVector3d(1.5,0,0));
        small.ControlPoints[2]=small.ControlPoints[3]=Detail::Point(FVector3d(0,1.5,0));
        TArray<FIntervalTensorSurface> overlapping{triangle(0),small};FDistanceBudget overlapBudget{8192};auto overlapProof=proof;
        TestTrue(TEXT("barycentric outside proof is nonvacuous even when tangent bbox contains foot"),
            ContactDetail::ProjectedPlanarDomains(overlapping,center,overlapBudget,overlapProof)==ContactDetail::EPlanarProjectionProof::Proved&&overlapProof.Providers[1].Outside);
        auto edge=triangle(0);edge.ControlPoints[1]=Detail::Point(FVector3d(2,0,0));
        edge.ControlPoints[2]=edge.ControlPoints[3]=Detail::Point(FVector3d(0,2,0));
        TArray<FIntervalTensorSurface> sharedEdge{triangle(0),edge};FDistanceBudget sharedBudget{8192};auto sharedProof=proof;
        TestTrue(TEXT("strict interior witness never drops another provider at closed edge"),
            ContactDetail::ProjectedPlanarDomains(sharedEdge,center,sharedBudget,sharedProof)==ContactDetail::EPlanarProjectionProof::Proved&&!sharedProof.Providers[1].Outside);
        auto apex=triangle(0);apex.ControlPoints[1]=Detail::Point(FVector3d(2,0,0));
        apex.ControlPoints[2]=apex.ControlPoints[3]=Detail::Point(FVector3d(1,1,0));
        TArray<FIntervalTensorSurface> sharedApex{triangle(0),apex};FDistanceBudget apexBudget{8192};auto apexProof=proof;
        TestTrue(TEXT("uncertain inverse at apex keeps finite provider under separate strict witness"),
            ContactDetail::ProjectedPlanarDomains(sharedApex,center,apexBudget,apexProof)==ContactDetail::EPlanarProjectionProof::Proved&&!apexProof.Providers[1].Outside&&!apexProof.Providers[1].Bounded);
        auto boundary=Detail::Point(FVector3d(0,0,-2));FDistanceBudget edgeBudget{8192};auto rejected=proof;
        TestTrue(TEXT("boundary is not a strict union witness"),ContactDetail::ProjectedPlanarDomains(surfaces,boundary,edgeBudget,rejected)==ContactDetail::EPlanarProjectionProof::NotApplicable&&rejected.Providers.IsEmpty());
        auto nonplane=surfaces;nonplane[1].ControlPoints[0].Axis[2]=Interval::Exact(.125);FDistanceBudget nonplaneBudget{8192};rejected=proof;
        TestTrue(TEXT("one provider off common plane defeats certificate, not silently dropped"),ContactDetail::ProjectedPlanarDomains(nonplane,center,nonplaneBudget,rejected)==ContactDetail::EPlanarProjectionProof::NotApplicable&&rejected.Providers.IsEmpty());
        TArray<FIntervalTensorSurface> independent{triangle(0)};
        independent[0].ControlPoints[2].Axis[0]={-.01,.01,true};independent[0].ControlPoints[3]=independent[0].ControlPoints[2];FDistanceBudget independentBudget{8192};rejected=proof;
        TestTrue(TEXT("equal nonsingleton CP boxes are not the same fixed vertex"),ContactDetail::ProjectedPlanarDomains(independent,center,independentBudget,rejected)==ContactDetail::EPlanarProjectionProof::NotApplicable&&rejected.Providers.IsEmpty());
        FDistanceBudget lateDenied{26,5,1};rejected=proof;
        TestTrue(TEXT("late certificate floor failure publishes no partial provider domains"),ContactDetail::ProjectedPlanarDomains(surfaces,center,lateDenied,rejected)==ContactDetail::EPlanarProjectionProof::Work&&lateDenied.Used==30&&lateDenied.Remaining==1&&rejected.Providers.IsEmpty());
        FDistanceBudget denied{3,5,1};rejected=proof;
        TestTrue(TEXT("prepaid same-account floor failure is terminal and clears output"),ContactDetail::ProjectedPlanarDomains(surfaces,center,denied,rejected)==ContactDetail::EPlanarProjectionProof::Work&&denied.Used==7&&denied.Remaining==1&&rejected.Providers.IsEmpty());
    }
    VectorSetControlRegister(original);return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAnalyticContinuousCurveSupport78Test,
    "IAmSpeed.AnalyticWorld.TensorContact.ContinuousCurveSupport",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FAnalyticContinuousCurveSupport78Test::RunTest(const FString&)
{
    using namespace Speed::Analytic;using namespace Speed::Analytic::Certified;
    using Proof=ECurvePathExclusion;using Memo=ContactDetail::FContactCellGeometryMemo;
    FTensorBoundsTestContext context;if(!TestTrue(TEXT("RNE fixture"),context.Ready))return false;
    const uint32 original=VectorGetControlRegister();ON_SCOPE_EXIT{VectorSetControlRegister(original);};
    for(uint32 flush:{0u,64u,32768u,32832u})
    {
        VectorSetControlRegister((original&~uint32(0x807f))|flush|0x3f);
        const auto control=RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush).ControlFingerprint();
        FIntervalTensorSurface diagonal;diagonal.DegreeU=0;diagonal.DegreeV=1;
        diagonal.ControlPoints={Detail::Point(FVector3d(-1,-1,0)),Detail::Point(FVector3d(1,1,0))};
        TArray<FIntervalTensorSurface> surfaces{diagonal};
        FIntervalBezierCenterPath path;
        if(!TestTrue(TEXT("correlated clear path"),CenterDetail::Linear(FVector3d(-.25,-1.75,0),FVector3d(1.75,.25,0),path)))return false;
        Memo memo;
        for(uint32 initialFlags:{0u,63u})
        {
        uint64 expectedUsed=0;uint32 expectedFlags=0;
        for(int32 repeat=0;repeat<3;++repeat)
        {
            VectorSetControlRegister((original&~uint32(0x807f))|flush|initialFlags);
            FContinuousCurvePathSupport support(surfaces,repeat?&memo:nullptr,control);
            FDistanceBudget work{10000,0,0};
            TestTrue(TEXT("full path separated although Cartesian boxes overlap"),support.TryExclude(0,path,Interval::Exact(1),work,8)==Proof::Clear);
            if(!repeat){expectedUsed=work.Used;expectedFlags=VectorGetControlRegister();}
            else TestTrue(TEXT("uncached/cold/warm same debits and flags"),work.Used==expectedUsed&&VectorGetControlRegister()==expectedFlags);
        }
        }
        FContinuousCurvePathSupport support(surfaces,&memo,control);
        FDistanceBudget noWork{0,0,0};
        TestTrue(TEXT("global budget denial is Unknown"),support.TryExclude(0,path,Interval::Exact(1),noWork,8)==Proof::Unknown);
        FDistanceBudget localWork{10000,0,0};
        TestTrue(TEXT("zero local effort proves nothing"),support.TryExclude(0,path,Interval::Exact(1),localWork,0)==Proof::Undecided);
        FDistanceBudget negativeWork{10000,0,0};
        TestTrue(TEXT("negative squared radius cannot exclude"),support.TryExclude(0,path,Interval::Exact(-1),negativeWork,8)==Proof::Unknown);
        FDistanceBudget wrongWork{10000,0,0};
        FContinuousCurvePathSupport wrong(surfaces,&memo,control^(uint64(1)<<40));
        TestTrue(TEXT("numeric identity mismatch is Unknown"),wrong.TryExclude(0,path,Interval::Exact(1),wrongWork,8)==Proof::Unknown);

        FIntervalTensorSurface horizontal;horizontal.DegreeU=0;horizontal.DegreeV=1;
        horizontal.ControlPoints={Detail::Point(FVector3d(-1,0,0)),Detail::Point(FVector3d(1,0,0))};
        TArray<FIntervalTensorSurface> tangentSurfaces{horizontal};FIntervalBezierCenterPath tangent;
        CenterDetail::Linear(FVector3d(-.5,1,0),FVector3d(.5,1,0),tangent);
        FContinuousCurvePathSupport tangentSupport(tangentSurfaces,&memo,control);FDistanceBudget tangentWork{10000,0,0};
        TestTrue(TEXT("exact tangency is not strict separation"),tangentSupport.TryExclude(0,tangent,Interval::Exact(1),tangentWork,8)==Proof::Undecided);

        FIntervalTensorSurface bulge;bulge.DegreeU=0;bulge.DegreeV=2;
        bulge.ControlPoints={Detail::Point(FVector3d(-1,0,0)),Detail::Point(FVector3d(0,2,0)),Detail::Point(FVector3d(1,0,0))};
        Detail::FNode originalBulge,leftBulge,rightBulge;originalBulge.DegreeU=0;originalBulge.DegreeV=2;originalBulge.Net=bulge.ControlPoints;
        TestTrue(TEXT("restrict full curve net"),Detail::Split(originalBulge,false,leftBulge,rightBulge));
        TestTrue(TEXT("interval split really widens an exact-zero control"),leftBulge.Net[1].Axis[2].Lower<0&&leftBulge.Net[1].Axis[2].Upper>0);
        ContactDetail::FCurveSupportBands restrictedBands;
        TestTrue(TEXT("root projection survives widened child zero"),ContactDetail::BuildCurveSupportBands(leftBulge,2,restrictedBands)&&restrictedBands.ZeroAxis==2);
        TestFalse(TEXT("invalid projection axis refused"),ContactDetail::BuildCurveSupportBands(leftBulge,3,restrictedBands));
        TArray<FIntervalTensorSurface> curved{bulge};FIntervalBezierCenterPath throughBulge;
        CenterDetail::Linear(FVector3d(0,1,0),FVector3d(0,1,0),throughBulge);
        FContinuousCurvePathSupport bulgeSupport(curved,&memo,control);FDistanceBudget bulgeWork{10000,0,0};
        TestTrue(TEXT("curve interior cannot be replaced by endpoint chord"),bulgeSupport.TryExclude(0,throughBulge,Interval::Exact(.01),bulgeWork,8)==Proof::Undecided);
        FIntervalBezierCenterPath rightContact;CenterDetail::Linear(FVector3d(.75,0,0),FVector3d(.75,0,0),rightContact);
        FContinuousCurvePathSupport childSupport(tangentSurfaces,&memo,control);FDistanceBudget childWork{10000,0,0};
        TestTrue(TEXT("clear left child cannot discard intersecting right child"),childSupport.TryExclude(0,rightContact,Interval::Exact(.01),childWork,8)==Proof::Undecided);

        auto uncertain=tangent;
        for(auto& cp:uncertain.ControlPoints)cp.Axis[1]={0.,1.,true};
        FDistanceBudget uncertainWork{10000,0,0};
        TestTrue(TEXT("coefficient uncertainty includes intersecting polynomial"),tangentSupport.TryExclude(0,uncertain,Interval::Exact(.01),uncertainWork,8)==Proof::Undecided);
        auto malformed=path;malformed.ControlPoints[0].Axis[0].Known=false;FDistanceBudget malformedWork{10000,0,0};
        TestTrue(TEXT("invalid path is Unknown"),support.TryExclude(0,malformed,Interval::Exact(1),malformedWork,8)==Proof::Unknown);

        auto generic=horizontal;generic.DegreeU=1;generic.ControlPoints.Append(horizontal.ControlPoints);
        TArray<FIntervalTensorSurface> genericSurfaces{generic};FContinuousCurvePathSupport genericSupport(genericSurfaces,&memo,control);FDistanceBudget genericWork{10000,0,0};
        TestTrue(TEXT("unreduced tensor retains exact fallback"),genericSupport.TryExclude(0,path,Interval::Exact(1),genericWork,8)==Proof::Undecided);
        // Same helper observes an illegal changed caller view and cannot use its old root.
        FDistanceBudget prepareChanged{10000,0,0};support.TryExclude(0,path,Interval::Exact(1),prepareChanged,8);
        surfaces[0].ControlPoints[0].Axis[0]=Interval::Exact(-2);FDistanceBudget changedWork{10000,0,0};
        TestTrue(TEXT("changed complete root identity is Unknown"),support.TryExclude(0,path,Interval::Exact(1),changedWork,8)==Proof::Unknown);
    }
    return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAnalyticContinuousCurveTime78Test,
    "IAmSpeed.AnalyticWorld.TensorContact.ContinuousCurveTimeIntegration",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FAnalyticContinuousCurveTime78Test::RunTest(const FString&)
{
    using namespace Speed::Analytic;using namespace Speed::Analytic::Certified;
    FTensorBoundsTestContext context;if(!TestTrue(TEXT("RNE fixture"),context.Ready))return false;
    FIntervalTensorSurface diagonal;diagonal.DegreeU=0;diagonal.DegreeV=1;
    diagonal.ControlPoints={Detail::Point(FVector3d(-1,-1,0)),Detail::Point(FVector3d(1,1,0))};
    TArray<FIntervalTensorSurface> surfaces{diagonal};FIntervalBezierCenterPath path;
    CenterDetail::Linear(FVector3d(-.25,-1.75,0),FVector3d(1.75,.25,0),path);
    FDistanceBudget clearWork{131072,0,0};
    TimeDetail::FRetainedSphereTimeSearch clearSearch(surfaces,path,1.,clearWork,1.e-7,256,nullptr,true);
    const auto clear=clearSearch.Advance(1./4096.);
    TestTrue(TEXT("new filter actually discharges a time cell"),clearSearch.GetCurveSupportAttemptsForInspection()>0&&clearSearch.GetCurveSupportClearedCellsForInspection()>0);
    TestTrue(TEXT("time search closes correlated full path after initial separation"),clear.Status==ETensorSphereTimeStatus::NoIntersectionWithProvidedTensors&&clear.InitialSeparationProved);
    FIntervalTensorSurface blocker;blocker.DegreeU=blocker.DegreeV=0;
    blocker.ControlPoints={Detail::Point(FVector3d(1.75,.25,0))};surfaces.Add(blocker);
    FDistanceBudget unionWork{131072,0,0};
    const auto unionResult=BoundTensorSphereSweep(surfaces,path,1.,unionWork,1./4096.,1.e-7,256);
    TestTrue(TEXT("clear first provider cannot remove second provider contact"),unionResult.Status==ETensorSphereTimeStatus::FirstContactBracket&&unionResult.InitialSeparationProved);
    TArray<FIntervalTensorSurface> interior{blocker};FIntervalBezierCenterPath enterExit;enterExit.Degree=2;
    enterExit.ControlPoints={Detail::Point(FVector3d(1.75,-2.75,0)),Detail::Point(FVector3d(1.75,3.25,0)),Detail::Point(FVector3d(1.75,-2.75,0))};
    FDistanceBudget interiorWork{131072,0,0};
    const auto crossed=BoundTensorSphereSweep(interior,enterExit,.25,interiorWork,1./4096.,1.e-7,256);
    TestTrue(TEXT("two clear endpoints cannot exclude interior enter-exit"),crossed.Status==ETensorSphereTimeStatus::FirstContactBracket&&crossed.UpperTime<.5);
    return !HasAnyErrors();
}

// Include before MinimumMidpointTests.fragment.inl, then call the two helpers
// from its existing two RunTest bodies. These are additional cases, not new
// native registrations and not an execution or source-installation claim.
#include <type_traits>
#include "Misc/ScopeExit.h"
#include "Misc/ScopedCVar.h"
namespace Speed::Analytic::Certified::MinimumMidpointExtra4480
{
namespace A=::AnalyticCertified;
using Memo=Detail::FMinimumMidpointMemo;
static_assert(!std::is_copy_constructible_v<Memo::FCallScope>);
static_assert(!std::is_copy_assignable_v<Memo::FCallScope>);
static_assert(!std::is_move_constructible_v<Memo::FCallScope>);
static_assert(!std::is_move_assignable_v<Memo::FCallScope>);
static_assert(!std::is_constructible_v<Memo::FCallScope,Memo*,FIntervalTensorSurface&&,uint64>);
inline bool SamePoint(const FIntervalPoint& a,const FIntervalPoint& b)
{
    for(int32 d=0;d<3;++d)if(a.Axis[d].Known!=b.Axis[d].Known||
        A::Bits(a.Axis[d].Lower)!=A::Bits(b.Axis[d].Lower)||A::Bits(a.Axis[d].Upper)!=A::Bits(b.Axis[d].Upper))return false;
    return true;
}
inline bool SameBounds(const FTensorDistanceBounds& a,const FTensorDistanceBounds& b)
{
    return a.Status==b.Status&&A::Bits(a.LowerSquared)==A::Bits(b.LowerSquared)&&
        A::Bits(a.UpperSquared)==A::Bits(b.UpperSquared)&&A::Bits(a.SampleUV.X)==A::Bits(b.SampleUV.X)&&
        A::Bits(a.SampleUV.Y)==A::Bits(b.SampleUV.Y)&&SamePoint(a.SamplePoint,b.SamplePoint);
}
inline FIntervalTensorSurface Plane()
{
    FIntervalTensorSurface surface;surface.DegreeU=surface.DegreeV=1;
    for(const FVector3d point:{FVector3d(-4,-4,0),FVector3d(-4,4,0),FVector3d(4,-4,0),FVector3d(4,4,0)})
        surface.ControlPoints.Add(Detail::Point(point));
    return surface;
}
inline Detail::FNode Node(const FIntervalTensorSurface& surface)
{
    Detail::FNode node;node.DegreeU=surface.DegreeU;node.DegreeV=surface.DegreeV;
    for(const auto& p:surface.ControlPoints)node.Net.Add(p);return node;
}
inline bool AdditionalGeometryChecks(uint32 original)
{
    const uint32 saved=VectorGetControlRegister();ON_SCOPE_EXIT{VectorSetControlRegister(saved);};
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})
    {
        const uint32 active=(original&~uint32(0x807f))|flush|0x3f;
        VectorSetControlRegister(active);
        const uint64 control=RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush).ControlFingerprint();
        const auto surface=Plane();const auto root=Node(surface);Memo memo;
        {
            Memo::FCallScope scope(&memo,surface,control);
            const auto first=scope.Midpoint(root),second=scope.Midpoint(root);
            if(!SamePoint(first,second)||memo.GetCountersForInspection().Hits==0)return false;
        }
        const auto miss=[&](const FIntervalTensorSurface& rootSurface,const Detail::FNode& node,bool mayPublish)
        {
            VectorSetControlRegister(active);const auto expected=Detail::Midpoint(node);const uint32 flags=VectorGetControlRegister();
            const auto prior=memo.GetCountersForInspection();const int32 entries=memo.GetEntriesForInspection();
            VectorSetControlRegister(active);FIntervalPoint actual;
            {Memo::FCallScope scope(&memo,rootSurface,control);actual=scope.Midpoint(node);}
            return SamePoint(expected,actual)&&VectorGetControlRegister()==flags&&memo.GetCountersForInspection().Hits==prior.Hits&&
                (mayPublish||(memo.GetCountersForInspection().Builds==prior.Builds&&memo.GetEntriesForInspection()==entries));
        };
        auto signedNode=root;signedNode.Net[0].Axis[2]={-0.,-0.,true};
        if(!miss(surface,signedNode,true))return false; // Cell CP identity keeps signed zero.
        auto signedSurface=surface;signedSurface.ControlPoints[0].Axis[2]={-0.,-0.,true};
        if(!miss(signedSurface,root,true))return false; // Same node still has a different root identity.
        auto domain=root;domain.DepthU=1;domain.MaxU=.5;
        if(!miss(surface,domain,true))return false;
        domain.MinU=.25;domain.MaxU=.75;
        if(!miss(surface,domain,true))return false;
        auto depth=root;depth.DepthV=1;
        if(!miss(surface,depth,true))return false;
        auto unknown=root;unknown.Net[0].Axis[2].Known=false;
        if(!miss(surface,unknown,false)||!miss(surface,unknown,false))return false;
        auto unknownSurface=surface;unknownSurface.ControlPoints[0].Axis[2].Known=false;
        if(!miss(unknownSurface,root,false))return false;
        auto inverted=root;inverted.Net[0].Axis[2]={2.,1.,true};
        if(!miss(surface,inverted,false))return false;
        auto tiny=root;const double denormal=A::FromBits(1);tiny.Net[0].Axis[2]={denormal,denormal,true};
        if(!miss(surface,tiny,false))return false;

        // Keep one old root index live while nested calls saturate the roots
        // directory. It must remain valid until every active scope closes.
        Memo nested;
        {
            Memo::FCallScope outer(&nested,surface,control);
            const auto expected=Detail::Midpoint(root);
            if(!SamePoint(expected,outer.Midpoint(root)))return false;
            for(int32 i=1;i<67;++i)
            {
                auto other=surface;other.ControlPoints[0].Axis[0]=Interval::Exact(-4.-double(i)*.03125);
                const auto node=Node(other);const auto oracle=Detail::Midpoint(node);FIntervalPoint actual;
                {Memo::FCallScope inner(&nested,other,control);actual=inner.Midpoint(node);}
                if(!SamePoint(oracle,actual)||nested.GetActiveCallsForInspection()!=1||nested.GetCountersForInspection().Evictions!=0)return false;
            }
            const uint64 priorHits=nested.GetCountersForInspection().Hits;
            if(!SamePoint(expected,outer.Midpoint(root))||nested.GetCountersForInspection().Hits<=priorHits)return false;
            {
                Memo::FCallScope inner(&nested,surface,control);
                if(nested.GetActiveCallsForInspection()!=2||!SamePoint(expected,inner.Midpoint(root)))return false;
            }
            if(nested.GetRootsForInspection()>64||nested.GetEntriesForInspection()>64||
                nested.GetStoredPointsForInspection()>1024||nested.GetActiveCallsForInspection()!=1)return false;
        }
        if(nested.GetActiveCallsForInspection()!=0)return false;
        {
            Memo::FCallScope after(&nested,surface,control);
            if(!SamePoint(Detail::Midpoint(root),after.Midpoint(root)))return false;
        }
        if(nested.GetCountersForInspection().Evictions==0||nested.GetActiveCallsForInspection()!=0)return false;
    }
    return true;
}
inline bool AdditionalThresholdChecks(uint32 original)
{
    const uint32 saved=VectorGetControlRegister();ON_SCOPE_EXIT{VectorSetControlRegister(saved);};
    const auto surface=Plane();const Interval threshold=Interval::Exact(1.);
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})for(uint32 sticky:{0u,0x3fu})
    {
        const uint32 initial=(original&~uint32(0x807f))|flush|sticky;
        for(const FVector3d position:{FVector3d(0,0,20),FVector3d(-4,-4,.25)})
        {
            const auto center=Detail::Point(position);
            VectorSetControlRegister(initial);FDistanceBudget oracleWork{64,7,0};
            const auto oracle=BoundTensorDistanceSquaredIntervalCenter(surface,center,oracleWork,0.,47,&threshold,nullptr);
            const uint32 oracleFlags=VectorGetControlRegister();
            if(oracle.Status==EDistanceBoundStatus::Unknown||!(oracle.LowerSquared>threshold.Upper||oracle.UpperSquared<threshold.Lower))return false;
            Memo memo;
            for(int32 repeat=0;repeat<2;++repeat)
            {
                VectorSetControlRegister(initial);FDistanceBudget work{64,7,0};
                const auto actual=BoundTensorDistanceSquaredIntervalCenter(surface,center,work,0.,47,&threshold,&memo);
                if(!SameBounds(oracle,actual)||VectorGetControlRegister()!=oracleFlags||work.Used!=oracleWork.Used||
                    work.Remaining!=oracleWork.Remaining||work.ConsumptionFloor!=oracleWork.ConsumptionFloor)return false;
                if(memo.GetCountersForInspection().Requests!=0||memo.GetCountersForInspection().Builds!=0||
                    memo.GetRootsForInspection()!=0||memo.GetEntriesForInspection()!=0)return false;
            }
            VectorSetControlRegister(initial);FDistanceBudget referenceWork{64,7,0};
            FRetainedTensorMinimum reference(surface,center,referenceWork,0.,47,&threshold,nullptr);
            const auto expected=reference.Advance();const uint32 referenceFlags=VectorGetControlRegister();
            if(expected.Status==EDistanceBoundStatus::Unknown)return false;
            for(int32 repeat=0;repeat<2;++repeat)
            {
                VectorSetControlRegister(initial);FDistanceBudget work{64,7,0};
                FRetainedTensorMinimum value(surface,center,work,0.,47,&threshold,&memo);
                const auto actual=value.Advance();
                if(!SameBounds(expected,actual)||VectorGetControlRegister()!=referenceFlags||work.Used!=referenceWork.Used||
                    work.Remaining!=referenceWork.Remaining||work.ConsumptionFloor!=referenceWork.ConsumptionFloor)return false;
                if(memo.GetCountersForInspection().Requests!=0||memo.GetCountersForInspection().Builds!=0||
                    memo.GetRootsForInspection()!=0||memo.GetEntriesForInspection()!=0)return false;
            }
        }
    }
    return true;
}
inline bool SameRetainedCounters(const FRetainedTensorMinimumCounters& a,const FRetainedTensorMinimumCounters& b)
{
    return a.NewRoots==b.NewRoots&&a.ReusedLeaves==b.ReusedLeaves&&a.DirtyBounds==b.DirtyBounds&&
        a.KKTExcluded==b.KKTExcluded&&a.UVSplits==b.UVSplits&&a.LocalStops==b.LocalStops&&a.PeakHeapLeaves==b.PeakHeapLeaves;
}
struct FProjectedObservation
{
    FTensorDistanceBounds Result;
    FRetainedTensorMinimumCounters Counters;
    ERetainedTensorMinimumFailure Failure=ERetainedTensorMinimumFailure::None;
    uint64 Used=0,LocalSpent=0;int32 Remaining=0,Floor=0;uint32 Flags=0;bool Terminal=false;
};
inline bool SameProjected(const FProjectedObservation& a,const FProjectedObservation& b)
{
    return SameBounds(a.Result,b.Result)&&SameRetainedCounters(a.Counters,b.Counters)&&a.Failure==b.Failure&&
        a.Used==b.Used&&a.LocalSpent==b.LocalSpent&&a.Remaining==b.Remaining&&a.Floor==b.Floor&&a.Flags==b.Flags&&a.Terminal==b.Terminal;
}
inline bool AdditionalProjectedRecursionChecks(uint32 original)
{
    const uint32 saved=VectorGetControlRegister();ON_SCOPE_EXIT{VectorSetControlRegister(saved);};
    FIntervalTensorSurface surface;surface.DegreeU=surface.DegreeV=5;
    const double x[]={5.,2.,0.,0.,2.,5.};
    for(int32 u=0;u<6;++u)for(int32 v=0;v<6;++v)
        surface.ControlPoints.Add(Detail::Point(FVector3d(x[v],-5.+2.*v,0)));
    if(!ProjectedDistanceDetail::IsUsefulYZFamily(surface))return false;
    const auto projected=ProjectedDistanceDetail::ProjectYZ(surface);
    if(projected.DegreeU!=0||projected.DegreeV!=5||projected.ControlPoints.Num()!=6)return false;
    const auto center=Detail::Point(FVector3d::ZeroVector);const Interval threshold=Interval::Exact(.25);
    uint64 genuineHits=0;
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})for(uint32 sticky:{0u,0x3fu})for(int32 cap:{4,7,31})
    {
        const uint32 initial=(original&~uint32(0x807f))|flush|sticky;
        const auto oneShot=[&](Memo* memo)
        {
            VectorSetControlRegister(initial);FDistanceBudget work{1000,13,0};FProjectedObservation out;
            out.Result=BoundTensorDistanceSquaredIntervalCenter(surface,center,work,0.,cap,&threshold,memo);
            out.Flags=VectorGetControlRegister();out.Used=work.Used;out.Remaining=work.Remaining;out.Floor=work.ConsumptionFloor;
            return out;
        };
        const auto oracle=oneShot(nullptr);Memo memo;
        const auto cold=oneShot(&memo),warm=oneShot(&memo);
        if(!SameProjected(oracle,cold)||!SameProjected(oracle,warm)||oracle.Result.Status==EDistanceBoundStatus::Unknown)return false;
        // Two DISTINCT memo roots prove that the recursive projected degree0x5
        // solve actually received the memo, not merely the outer degree5x5 call.
        if(memo.GetRootsForInspection()<2||memo.GetCountersForInspection().Requests<2||memo.GetActiveCallsForInspection()!=0)return false;
        if(cap==4&&!(oracle.Used==17&&oracle.Result.Status==EDistanceBoundStatus::Bounded&&
            oracle.Result.UpperSquared>threshold.Upper&&oracle.Result.SamplePoint.Axis[0].Lower>.9))return false;
        genuineHits+=memo.GetCountersForInspection().Hits;
        const auto retained=[&](Memo* value)
        {
            VectorSetControlRegister(initial);FDistanceBudget work{1000,13,0};FProjectedObservation out;
            FRetainedTensorMinimum minimum(surface,center,work,0.,cap,&threshold,value);
            out.Result=minimum.Advance();out.Flags=VectorGetControlRegister();out.Counters=minimum.GetCountersForInspection();
            out.Failure=minimum.GetFailureForInspection();out.Terminal=minimum.IsTerminalForInspection();
            out.LocalSpent=minimum.GetLocalSpentForInspection();out.Used=work.Used;out.Remaining=work.Remaining;out.Floor=work.ConsumptionFloor;
            return out;
        };
        const auto retainedOracle=retained(nullptr);Memo retainedMemo;
        const auto retainedCold=retained(&retainedMemo),retainedWarm=retained(&retainedMemo);
        if(!SameProjected(retainedOracle,retainedCold)||!SameProjected(retainedOracle,retainedWarm)||
            retainedOracle.Result.Status==EDistanceBoundStatus::Unknown)return false;
        if(retainedMemo.GetRootsForInspection()<2||retainedMemo.GetCountersForInspection().Requests<2||retainedMemo.GetActiveCallsForInspection()!=0)return false;
        genuineHits+=retainedMemo.GetCountersForInspection().Hits;
    }
    return genuineHits>0;
}
}

// New synthetic arithmetic/cache tests only. No physical/profile admission.
#include "Misc/ScopeExit.h"
#include "Misc/ScopedCVar.h"
namespace Speed::Analytic::Certified::MinimumMidpointTest4480
{
namespace A=::AnalyticCertified;
using Memo=Detail::FMinimumMidpointMemo;
inline bool SamePoint(const FIntervalPoint& a,const FIntervalPoint& b)
{
    for(int32 d=0;d<3;++d)
        if(a.Axis[d].Known!=b.Axis[d].Known||A::Bits(a.Axis[d].Lower)!=A::Bits(b.Axis[d].Lower)||
            A::Bits(a.Axis[d].Upper)!=A::Bits(b.Axis[d].Upper))return false;
    return true;
}
inline bool SameBounds(const FTensorDistanceBounds& a,const FTensorDistanceBounds& b)
{
    return a.Status==b.Status&&A::Bits(a.LowerSquared)==A::Bits(b.LowerSquared)&&
        A::Bits(a.UpperSquared)==A::Bits(b.UpperSquared)&&A::Bits(a.SampleUV.X)==A::Bits(b.SampleUV.X)&&
        A::Bits(a.SampleUV.Y)==A::Bits(b.SampleUV.Y)&&SamePoint(a.SamplePoint,b.SamplePoint);
}
inline bool SameCounters(const FRetainedTensorMinimumCounters& a,const FRetainedTensorMinimumCounters& b)
{
    return a.NewRoots==b.NewRoots&&a.ReusedLeaves==b.ReusedLeaves&&a.DirtyBounds==b.DirtyBounds&&
        a.KKTExcluded==b.KKTExcluded&&a.UVSplits==b.UVSplits&&a.LocalStops==b.LocalStops&&
        a.PeakHeapLeaves==b.PeakHeapLeaves;
}
inline Detail::FNode Node(const FIntervalTensorSurface& surface)
{
    Detail::FNode node;node.DegreeU=surface.DegreeU;node.DegreeV=surface.DegreeV;
    for(const auto& cp:surface.ControlPoints)node.Net.Add(cp);
    return node;
}
inline FIntervalTensorSurface Plane()
{
    FIntervalTensorSurface surface;surface.DegreeU=surface.DegreeV=1;
    for(const FVector3d point:{FVector3d(-4,-4,0),FVector3d(-4,4,0),FVector3d(4,-4,0),FVector3d(4,4,0)})
        surface.ControlPoints.Add(Detail::Point(point));
    return surface;
}
inline FIntervalTensorSurface DirectFixture(int32 degree)
{
    FIntervalTensorSurface surface;surface.DegreeU=surface.DegreeV=degree;
    for(int32 u=0;u<=degree;++u)for(int32 v=0;v<=degree;++v)
        surface.ControlPoints.Add(Detail::Point(FVector3d(.1*u,.3*v,.01*(u*u+v*v))));
    return surface;
}
struct FObserved
{
    FTensorDistanceBounds First,Second;
    FRetainedTensorMinimumCounters Counters;
    ERetainedTensorMinimumFailure Failure=ERetainedTensorMinimumFailure::None;
    uint64 Used=0,LocalSpent=0;
    int32 Remaining=0,Floor=0;
    uint32 Flags=0;
    bool Terminal=false;
};
inline bool SameObserved(const FObserved& a,const FObserved& b)
{
    return SameBounds(a.First,b.First)&&SameBounds(a.Second,b.Second)&&SameCounters(a.Counters,b.Counters)&&
        a.Failure==b.Failure&&a.Used==b.Used&&a.LocalSpent==b.LocalSpent&&a.Remaining==b.Remaining&&
        a.Floor==b.Floor&&a.Flags==b.Flags&&a.Terminal==b.Terminal;
}
struct FForkObserved
{
    FTensorDistanceBounds Parent,Child,Refused,Restored;
    FRetainedTensorMinimumCounters ParentCounters,ChildCounters;
    ERetainedTensorMinimumFailure BadWhy=ERetainedTensorMinimumFailure::None,FinalFailure=ERetainedTensorMinimumFailure::None;
    uint64 Used=0,ParentSpent=0,ChildSpent=0;
    int32 Remaining=0,Floor=0;
    uint32 Flags=0;
    bool ParentKnown=false,ChildMade=false,BadRefused=false,ParentUnchanged=false,NumericNoDebit=false,StickyTerminal=false;
};
inline bool SameFork(const FForkObserved& a,const FForkObserved& b)
{
    return SameBounds(a.Parent,b.Parent)&&SameBounds(a.Child,b.Child)&&SameBounds(a.Refused,b.Refused)&&
        SameBounds(a.Restored,b.Restored)&&SameCounters(a.ParentCounters,b.ParentCounters)&&
        SameCounters(a.ChildCounters,b.ChildCounters)&&a.BadWhy==b.BadWhy&&a.FinalFailure==b.FinalFailure&&
        a.Used==b.Used&&a.ParentSpent==b.ParentSpent&&a.ChildSpent==b.ChildSpent&&a.Remaining==b.Remaining&&
        a.Floor==b.Floor&&a.Flags==b.Flags&&a.ParentKnown==b.ParentKnown&&a.ChildMade==b.ChildMade&&
        a.BadRefused==b.BadRefused&&a.ParentUnchanged==b.ParentUnchanged&&a.NumericNoDebit==b.NumericNoDebit&&
        a.StickyTerminal==b.StickyTerminal;
}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMinimumMidpointGeometry4480Test,
    "IAmSpeed.AnalyticWorld.TensorDistance.MinimumMidpointGeometry",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FMinimumMidpointGeometry4480Test::RunTest(const FString&)
{
    using namespace Speed::Analytic;using namespace Speed::Analytic::Certified;
    namespace T=MinimumMidpointTest4480;namespace A=::AnalyticCertified;
    using Memo=Detail::FMinimumMidpointMemo;
    FTensorBoundsTestContext context;if(!TestTrue(TEXT("RNE fixture established"),context.Ready))return false;
    const uint32 original=VectorGetControlRegister();ON_SCOPE_EXIT{VectorSetControlRegister(original);};
    uint64 observedRequests=0,observedHits=0,observedBuilds=0;
    TestTrue(TEXT("thread getter is stable on this thread"),&Detail::ThreadMinimumMidpointMemo()==&Detail::ThreadMinimumMidpointMemo());
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})for(uint32 sticky:{0u,0x3fu})
    {
        const uint32 initial=(original&~uint32(0x807f))|flush|sticky;
        VectorSetControlRegister(initial);
        const uint64 control=RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush).ControlFingerprint();
        Memo memo;
        // Degree16 is exercised only as a direct geometric operation.
        for(int32 degree:{0,1,5,16})
        {
            const auto surface=T::DirectFixture(degree);const auto root=T::Node(surface);
            VectorSetControlRegister(initial);const auto expected=Detail::Midpoint(root);const uint32 expectedFlags=VectorGetControlRegister();
            for(int32 repeat=0;repeat<2;++repeat)
            {
                VectorSetControlRegister(initial);
                FIntervalPoint actual;
                {Memo::FCallScope scope(&memo,surface,control);actual=scope.Midpoint(root);}
                const uint32 actualFlags=VectorGetControlRegister();
                if(!TestTrue(TEXT("direct original/cold/warm midpoint bits and complete vector flags match"),
                    T::SamePoint(expected,actual)&&actualFlags==expectedFlags))return false;
            }
            auto changedSurface=surface;changedSurface.ControlPoints[0].Axis[0]=Interval::Exact(7.);
            const auto changedRoot=T::Node(changedSurface);
            VectorSetControlRegister(initial);const auto expectedRoot=Detail::Midpoint(changedRoot);const uint32 rootFlags=VectorGetControlRegister();
            const auto hitsBeforeRoot=memo.GetCountersForInspection().Hits;
            VectorSetControlRegister(initial);FIntervalPoint actualRoot;
            {Memo::FCallScope scope(&memo,changedSurface,control);actualRoot=scope.Midpoint(changedRoot);}
            if(!TestTrue(TEXT("changed complete root cannot recover previous midpoint"),T::SamePoint(expectedRoot,actualRoot)&&
                VectorGetControlRegister()==rootFlags&&memo.GetCountersForInspection().Hits==hitsBeforeRoot))return false;

            auto changedNode=root;changedNode.Net.Last().Axis[1]=Interval::Exact(11.);
            VectorSetControlRegister(initial);const auto expectedNode=Detail::Midpoint(changedNode);const uint32 nodeFlags=VectorGetControlRegister();
            const auto hitsBeforeNode=memo.GetCountersForInspection().Hits;
            VectorSetControlRegister(initial);FIntervalPoint actualNode;
            {Memo::FCallScope scope(&memo,surface,control);actualNode=scope.Midpoint(changedNode);}
            if(!TestTrue(TEXT("same domain but changed restricted CP bits do not alias"),T::SamePoint(expectedNode,actualNode)&&
                VectorGetControlRegister()==nodeFlags&&memo.GetCountersForInspection().Hits==hitsBeforeNode))return false;
            if(degree>0&&degree<=5)
            {
                Detail::FNode u0,u1,uv0,uv1,v0,v1,vu0,vu1;
                if(!TestTrue(TEXT("both ordered dyadic paths are valid"),Detail::Split(root,true,u0,u1)&&
                    Detail::Split(u0,false,uv0,uv1)&&Detail::Split(root,false,v0,v1)&&Detail::Split(v0,true,vu0,vu1)))return false;
                for(const auto* node:{&uv0,&vu0})
                {
                    VectorSetControlRegister(initial);const auto orderedExpected=Detail::Midpoint(*node);const uint32 orderedFlags=VectorGetControlRegister();
                    VectorSetControlRegister(initial);FIntervalPoint orderedActual;
                    {Memo::FCallScope scope(&memo,surface,control);orderedActual=scope.Midpoint(*node);}
                    if(!TestTrue(TEXT("U/V ordering keeps its own actual restricted net"),T::SamePoint(orderedExpected,orderedActual)&&
                        VectorGetControlRegister()==orderedFlags))return false;
                }
            }
        }
        // A foreign requested fingerprint must fall back without a cache hit.
        const auto surface=T::DirectFixture(1);const auto node=T::Node(surface);
        VectorSetControlRegister(initial);const auto foreignExpected=Detail::Midpoint(node);const uint32 foreignFlags=VectorGetControlRegister();
        const auto foreignHits=memo.GetCountersForInspection().Hits;
        VectorSetControlRegister(initial);FIntervalPoint foreignActual;
        {Memo::FCallScope scope(&memo,surface,control^uint64(1));foreignActual=scope.Midpoint(node);}
        TestTrue(TEXT("foreign numeric fingerprint never reuses a cached midpoint"),T::SamePoint(foreignExpected,foreignActual)&&
            VectorGetControlRegister()==foreignFlags&&memo.GetCountersForInspection().Hits==foreignHits);
        const auto counters=memo.GetCountersForInspection();
        observedRequests+=counters.Requests;observedHits+=counters.Hits;observedBuilds+=counters.Builds;
        TestTrue(TEXT("small direct workload remains bounded"),counters.Requests<256&&counters.Hits<=counters.Requests);
    }
    // Deliberately inexact half-sum: the guard must be tested with flags that
    // were ACTUALLY raised, rather than merely requesting a sticky mode.
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})
    {
        const uint32 clean=(original&~uint32(0x807f))|flush;VectorSetControlRegister(clean);
        const uint64 control=RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush).ControlFingerprint();
        FIntervalTensorSurface surface;surface.DegreeU=1;
        surface.ControlPoints={Detail::Point(FVector3d(1.,.3,.7)),Detail::Point(FVector3d(A::FromBits(0x3c90000000000000ULL),.1,.2))};
        const auto node=T::Node(surface);Memo memo;
        VectorSetControlRegister(clean);const auto expected=Detail::Midpoint(node);const uint32 raised=VectorGetControlRegister()&0x3f;
        if(!TestTrue(TEXT("missing-sticky fixture actually raises exception status"),raised!=0))return false;
        VectorSetControlRegister(clean);FIntervalPoint cold;
        {Memo::FCallScope scope(&memo,surface,control);cold=scope.Midpoint(node);}
        if(!TestTrue(TEXT("clean cold midpoint raises the same flags"),T::SamePoint(cold,expected)&&
            (VectorGetControlRegister()&0x3f)==raised))return false;
        const uint64 before=memo.GetCountersForInspection().Hits;
        VectorSetControlRegister(clean);FIntervalPoint missing;
        {Memo::FCallScope scope(&memo,surface,control);missing=scope.Midpoint(node);}
        TestTrue(TEXT("missing previously required flags decline the hit, without synthesizing flags"),
            T::SamePoint(missing,expected)&&(VectorGetControlRegister()&0x3f)==raised&&memo.GetCountersForInspection().Hits==before);
        VectorSetControlRegister(clean|0x3f);FIntervalPoint warm;
        {Memo::FCallScope scope(&memo,surface,control);warm=scope.Midpoint(node);}
        TestTrue(TEXT("available sticky flags enable a genuine warm hit"),T::SamePoint(warm,expected)&&memo.GetCountersForInspection().Hits>before);
    }
    // Roots64 is a documented memory bound. Exit each scope before the next
    // root; root65 may only mark saturation and root66 may evict on entry.
    {
        const uint32 active=(original&~uint32(0x807f))|0x3f;VectorSetControlRegister(active);
        const uint64 control=RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush).ControlFingerprint();
        Memo memo;
        for(int32 i=0;i<67;++i)
        {
            auto surface=T::Plane();surface.ControlPoints[0].Axis[0]=Interval::Exact(-4.-double(i)*.03125);
            const auto node=T::Node(surface);
            const auto expected=Detail::Midpoint(node);FIntervalPoint actual;
            {Memo::FCallScope scope(&memo,surface,control);actual=scope.Midpoint(node);}
            if(!TestTrue(TEXT("bounded eviction preserves the original geometric operation"),T::SamePoint(expected,actual)))return false;
        }
        const auto counters=memo.GetCountersForInspection();
        TestTrue(TEXT("directory eviction is genuinely exercised with bounded work"),counters.Evictions>0&&counters.Requests>=67&&counters.Requests<256);
    }
    TestTrue(TEXT("cache exercise is not vacuous"),observedRequests>0&&observedHits>0&&observedBuilds>0);
    TestTrue(TEXT("additional exact identity, invalid and nested-saturation cases"),MinimumMidpointExtra4480::AdditionalGeometryChecks(original));
    return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMinimumMidpointSolverParity4480Test,
    "IAmSpeed.AnalyticWorld.TensorDistance.MinimumMidpointSolverParity",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FMinimumMidpointSolverParity4480Test::RunTest(const FString&)
{
    using namespace Speed::Analytic;using namespace Speed::Analytic::Certified;
    namespace T=MinimumMidpointTest4480;using Memo=Detail::FMinimumMidpointMemo;
    FTensorBoundsTestContext context;if(!TestTrue(TEXT("RNE fixture established"),context.Ready))return false;
    const uint32 original=VectorGetControlRegister();ON_SCOPE_EXIT{VectorSetControlRegister(original);};
    TArray<FIntervalTensorSurface> surfaces;surfaces.Add(T::Plane());
    FIntervalTensorSurface curve;curve.DegreeV=3;
    for(const FVector3d point:{FVector3d(-2,0,0),FVector3d(-.5,0,1),FVector3d(.5,0,-1),FVector3d(2,0,0)})
        curve.ControlPoints.Add(Detail::Point(point));
    surfaces.Add(curve);
    struct FAllowance {int32 Global,Local,Floor;};
    const FAllowance allowances[]={{0,47,0},{64,0,0},{64,1,0},{64,13,63},{96,13,0},{96,47,0}};
    uint64 requests=0,hits=0,builds=0;
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})for(uint32 sticky:{0u,0x3fu})
    {
        const uint32 initial=(original&~uint32(0x807f))|flush|sticky;
        for(const auto& surface:surfaces)for(int32 centerMode=0;centerMode<2;++centerMode)for(int32 thresholdMode=0;thresholdMode<3;++thresholdMode)
        {
            auto center=Detail::Point(FVector3d(.375,-.125,1.));
            if(centerMode){center.Axis[0]={-.5,.5,true};center.Axis[2]={.75,1.25,true};}
            const Interval threshold=thresholdMode==1?Interval::Exact(1.):Interval{.5,1.5,true};
            const Interval* thresholdPointer=thresholdMode?&threshold:nullptr;
            for(const auto& allowance:allowances)
            {
                const auto oneShot=[&](Memo* memo)
                {
                    VectorSetControlRegister(initial);FDistanceBudget work{allowance.Global,7,allowance.Floor};T::FObserved out;
                    out.First=BoundTensorDistanceSquaredIntervalCenter(surface,center,work,0.,allowance.Local,thresholdPointer,memo);
                    out.Flags=VectorGetControlRegister();out.Used=work.Used;out.Remaining=work.Remaining;out.Floor=work.ConsumptionFloor;
                    return out;
                };
                const auto oracle=oneShot(nullptr);Memo memo;
                const auto cold=oneShot(&memo),warm=oneShot(&memo);
                if(!TestTrue(TEXT("one-shot null/cold/warm preserve every result bit, budget, floor and vector flag"),
                    T::SameObserved(oracle,cold)&&T::SameObserved(oracle,warm)))return false;
                const auto stats=memo.GetCountersForInspection();requests+=stats.Requests;hits+=stats.Hits;builds+=stats.Builds;
                const auto retained=[&](Memo* value)
                {
                    VectorSetControlRegister(initial);FDistanceBudget work{allowance.Global,7,allowance.Floor};T::FObserved out;
                    FRetainedTensorMinimum minimum(surface,center,work,0.,allowance.Local,thresholdPointer,value);
                    out.First=minimum.Advance();out.Second=minimum.Advance();
                    out.Flags=VectorGetControlRegister();out.Counters=minimum.GetCountersForInspection();
                    out.Failure=minimum.GetFailureForInspection();out.Terminal=minimum.IsTerminalForInspection();
                    out.LocalSpent=minimum.GetLocalSpentForInspection();out.Used=work.Used;out.Remaining=work.Remaining;out.Floor=work.ConsumptionFloor;
                    return out;
                };
                const auto retainedOracle=retained(nullptr);Memo retainedMemo;
                const auto retainedCold=retained(&retainedMemo),retainedWarm=retained(&retainedMemo);
                if(!TestTrue(TEXT("retained null/cold/warm preserve witnesses, counters, refusals and allowance across Advance"),
                    T::SameObserved(retainedOracle,retainedCold)&&T::SameObserved(retainedOracle,retainedWarm)))return false;
                const auto retainedStats=retainedMemo.GetCountersForInspection();
                requests+=retainedStats.Requests;hits+=retainedStats.Hits;builds+=retainedStats.Builds;
            }
        }
        const auto forkRun=[&](Memo* memo)
        {
            VectorSetControlRegister(initial);const auto surface=T::Plane();auto box=Detail::Point(FVector3d(0,0,3.5));
            box.Axis[0]={-1,1,true};box.Axis[1]={-1,1,true};box.Axis[2]={3,4,true};
            const Interval threshold=Interval::Exact(9.);FDistanceBudget work{16384,7,0};T::FForkObserved out;
            FRetainedTensorMinimum parent(surface,box,work,0.,256,&threshold,memo);
            out.Parent=parent.Advance();out.ParentKnown=parent.IsKnownSnapshot();out.ParentSpent=parent.GetLocalSpentForInspection();
            const auto& frozen=parent;auto enlarged=box;enlarged.Axis[2]={2,5,true};
            const uint64 beforeBad=work.Used;
            out.BadRefused=!frozen.ForkContainedImmutable(enlarged,out.BadWhy)&&work.Used==beforeBad;
            ERetainedTensorMinimumFailure why=ERetainedTensorMinimumFailure::None;
            auto child=frozen.ForkContainedImmutable(Detail::Point(FVector3d(0,0,3.25)),why);
            out.ChildMade=bool(child)&&why==ERetainedTensorMinimumFailure::None;
            if(child){out.Child=child->Advance();out.ChildCounters=child->GetCountersForInspection();out.ChildSpent=child->GetLocalSpentForInspection();}
            out.ParentUnchanged=parent.IsKnownSnapshot()&&parent.GetLocalSpentForInspection()==out.ParentSpent;
            const uint64 beforeNumeric=work.Used;const uint32 active=VectorGetControlRegister();
            VectorSetControlRegister(active^uint32(0x8000));out.Refused=parent.Advance();
            VectorSetControlRegister(active);out.Restored=parent.Advance();
            out.NumericNoDebit=work.Used==beforeNumeric;
            out.FinalFailure=parent.GetFailureForInspection();
            out.StickyTerminal=parent.IsTerminalForInspection()&&out.Refused.Status==EDistanceBoundStatus::Unknown&&out.Restored.Status==EDistanceBoundStatus::Unknown;
            out.ParentCounters=parent.GetCountersForInspection();out.Used=work.Used;out.Remaining=work.Remaining;out.Floor=work.ConsumptionFloor;out.Flags=VectorGetControlRegister();
            return out;
        };
        const auto forkOracle=forkRun(nullptr);Memo forkMemo;
        const auto forkCold=forkRun(&forkMemo),forkWarm=forkRun(&forkMemo);
        if(!TestTrue(TEXT("fork fixture genuinely materializes a contained child and refuses widened/numeric scopes"),
            forkOracle.ParentKnown&&forkOracle.ChildMade&&forkOracle.BadRefused&&forkOracle.ParentUnchanged&&forkOracle.NumericNoDebit&&
            forkOracle.BadWhy==ERetainedTensorMinimumFailure::Scope&&forkOracle.FinalFailure==ERetainedTensorMinimumFailure::Numeric&&forkOracle.StickyTerminal))return false;
        if(!TestTrue(TEXT("memo never transfers old query bounds or revives a refused retained context"),T::SameFork(forkOracle,forkCold)&&T::SameFork(forkOracle,forkWarm)))return false;
        const auto forkStats=forkMemo.GetCountersForInspection();requests+=forkStats.Requests;hits+=forkStats.Hits;builds+=forkStats.Builds;
    }
    TestTrue(TEXT("solver parity actually used cold and warm midpoint paths"),requests>0&&hits>0&&builds>0);
    TestTrue(TEXT("quick threshold remains lazy"),MinimumMidpointExtra4480::AdditionalThresholdChecks(original));
    TestTrue(TEXT("degree5x5 genuinely traverses projected recursion with exact parity"),MinimumMidpointExtra4480::AdditionalProjectedRecursionChecks(original));
    return !HasAnyErrors();
}

// New private-retained-cell tests. All current 89 tests remain byte-exact above.
namespace Speed::Analytic::Certified::RetainedMiddleTestCandidate1
{
namespace T=MinimumMidpointTest4480;
using Memo=Detail::FMinimumMidpointMemo;
struct FRun
{
    T::FForkObserved Semantic;
    Memo::FCounters BeforeChild,AfterChild;
};
inline FRun Contained(bool cached,uint32 initial,bool clearStickyBeforeFork=false)
{
    VectorSetControlRegister(initial);
    const auto surface=T::Plane();auto box=Detail::Point(FVector3d(0,0,3.5));
    box.Axis[0]={-1,1,true};box.Axis[1]={-1,1,true};box.Axis[2]={3,4,true};
    const Interval threshold=Interval::Exact(10.5625);
    FDistanceBudget work{16384,7,0};Memo memo;FRun result;auto& out=result.Semantic;
    FRetainedTensorMinimum parent(surface,box,work,0.,4,&threshold,cached?&memo:nullptr);
    out.Parent=parent.Advance();out.ParentKnown=parent.IsKnownSnapshot();
    out.ParentSpent=parent.GetLocalSpentForInspection();out.ParentCounters=parent.GetCountersForInspection();
    const auto parentBound=out.Parent;
    result.BeforeChild=memo.GetCountersForInspection();
    if(clearStickyBeforeFork)VectorSetControlRegister(VectorGetControlRegister()&~uint32(0x3f));
    ERetainedTensorMinimumFailure why=ERetainedTensorMinimumFailure::None;
    const FRetainedTensorMinimum& frozen=parent;
    auto child=frozen.ForkContainedImmutable(Detail::Point(FVector3d(0,0,3.25)),why);
    out.ChildMade=bool(child)&&why==ERetainedTensorMinimumFailure::None;
    if(child)
    {
        out.Child=child->Advance();out.ChildCounters=child->GetCountersForInspection();
        out.ChildSpent=child->GetLocalSpentForInspection();
    }
    result.AfterChild=memo.GetCountersForInspection();
    out.ParentUnchanged=parent.IsKnownSnapshot()&&parent.GetLocalSpentForInspection()==out.ParentSpent&&
        T::SameBounds(parentBound,parent.Advance());
    out.ParentCounters=parent.GetCountersForInspection();
    out.FinalFailure=parent.GetFailureForInspection();out.Used=work.Used;
    out.Remaining=work.Remaining;out.Floor=work.ConsumptionFloor;out.Flags=VectorGetControlRegister();
    return result;
}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRetainedCellMiddleReuseCandidate1Test,
    "IAmSpeed.AnalyticWorld.TensorDistance.RetainedCellMiddleReuse",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FRetainedCellMiddleReuseCandidate1Test::RunTest(const FString&)
{
    using namespace Speed::Analytic;using namespace Speed::Analytic::Certified;
    namespace T=MinimumMidpointTest4480;namespace X=RetainedMiddleTestCandidate1;
    FTensorBoundsTestContext context;if(!TestTrue(TEXT("RNE fixture established"),context.Ready))return false;
    const uint32 original=VectorGetControlRegister();ON_SCOPE_EXIT{VectorSetControlRegister(original);};
    uint64 actualLocalHits=0,hashLookupsAvoided=0,actualPublications=0;
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})for(uint32 sticky:{0u,0x3fu})
    {
        const uint32 initial=(original&~uint32(0x807f))|flush|sticky;
        const auto oracle=X::Contained(false,initial),actual=X::Contained(true,initial);
        if(!TestTrue(TEXT("retained private snapshot preserves all bound/witness bits, counters, budgets and vector flags"),
            T::SameFork(oracle.Semantic,actual.Semantic)))return false;
        if(!TestTrue(TEXT("fixture really advances a strict child dirty bound and leaves its frozen parent unchanged"),
            oracle.Semantic.ParentKnown&&oracle.Semantic.ChildMade&&oracle.Semantic.ParentUnchanged&&
            oracle.Semantic.Child.Status!=EDistanceBoundStatus::Unknown&&oracle.Semantic.ChildCounters.DirtyBounds>0))return false;
        const auto& before=actual.BeforeChild;const auto& after=actual.AfterChild;
        if(!TestTrue(TEXT("local snapshot publication is genuine and bounded"),
            before.RetainedMiddlePublications>0&&after.RetainedMiddleRequests<4096&&
            after.RetainedMiddleHits<=after.RetainedMiddleRequests))return false;
        actualPublications+=before.RetainedMiddlePublications;
        actualLocalHits+=after.RetainedMiddleHits-before.RetainedMiddleHits;
        if(sticky==0x3f)
        {
            if(!TestTrue(TEXT("compatible immutable dirty cell avoids every full-net memo lookup in this child"),
                after.RetainedMiddleHits>before.RetainedMiddleHits&&after.Requests==before.Requests))return false;
            ++hashLookupsAvoided;
        }
    }
    TestTrue(TEXT("new local hit path is nonvacuous, without a new winner/bound cache"),
        actualLocalHits>0&&hashLookupsAvoided==4&&actualPublications>0);
    return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRetainedCellMiddleScopeAndQuickCandidate1Test,
    "IAmSpeed.AnalyticWorld.TensorDistance.RetainedCellMiddleScopeAndQuickThreshold",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FRetainedCellMiddleScopeAndQuickCandidate1Test::RunTest(const FString&)
{
    using namespace Speed::Analytic;using namespace Speed::Analytic::Certified;
    namespace T=MinimumMidpointTest4480;namespace X=RetainedMiddleTestCandidate1;
    using Memo=Detail::FMinimumMidpointMemo;
    FTensorBoundsTestContext context;if(!TestTrue(TEXT("RNE fixture established"),context.Ready))return false;
    const uint32 original=VectorGetControlRegister();ON_SCOPE_EXIT{VectorSetControlRegister(original);};
    uint64 genuineFallbacks=0;
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})
    {
        const uint32 flagged=(original&~uint32(0x807f))|flush|0x3f;
        const auto oracle=X::Contained(false,flagged,true),actual=X::Contained(true,flagged,true);
        if(!TestTrue(TEXT("missing sticky status falls back to the original geometry calculation, without synthesizing flags"),
            T::SameFork(oracle.Semantic,actual.Semantic)&&actual.Semantic.ChildMade&&
            actual.Semantic.ChildCounters.DirtyBounds>0&&
            actual.AfterChild.RetainedMiddleHits==actual.BeforeChild.RetainedMiddleHits&&
            actual.AfterChild.Requests>actual.BeforeChild.Requests))return false;
        ++genuineFallbacks;
        const auto surface=T::Plane();const Interval threshold=Interval::Exact(1.);
        for(const FVector3d position:{FVector3d(0,0,20),FVector3d(-4,-4,.25)})
        {
            const auto center=Detail::Point(position);
            VectorSetControlRegister(flagged);FDistanceBudget baselineWork{64,7,0};
            FRetainedTensorMinimum baseline(surface,center,baselineWork,0.,47,&threshold,nullptr);
            const auto expected=baseline.Advance();const uint32 expectedFlags=VectorGetControlRegister();
            VectorSetControlRegister(flagged);FDistanceBudget actualWork{64,7,0};Memo memo;
            FRetainedTensorMinimum value(surface,center,actualWork,0.,47,&threshold,&memo);
            const auto bound=value.Advance();const auto stats=memo.GetCountersForInspection();
            if(!TestTrue(TEXT("quick threshold corner/box witness never publishes a geometric midpoint"),
                expected.Status!=EDistanceBoundStatus::Unknown&&T::SameBounds(expected,bound)&&
                expectedFlags==VectorGetControlRegister()&&baselineWork.Used==actualWork.Used&&
                baselineWork.Remaining==actualWork.Remaining&&stats.Requests==0&&
                stats.RetainedMiddleRequests==0&&stats.RetainedMiddlePublications==0))return false;
        }
        // A wrong numeric fingerprint cannot acquire an owner scope or a hit.
        VectorSetControlRegister(flagged);Memo memo;
        const uint64 fingerprint=RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush).ControlFingerprint();
        {Memo::FCallScope rejected(&memo,surface,fingerprint^uint64(1));
            TestFalse(TEXT("foreign context declines the private retained guard"),rejected.HasRetainedMiddleContext());
            rejected.ObserveRetainedMiddle(true,true);}
        TestTrue(TEXT("unowned guard cannot fabricate an observed hit/publication"),
            memo.GetCountersForInspection().RetainedMiddleHits==0&&memo.GetCountersForInspection().RetainedMiddlePublications==0);
        auto invalidCenter=Detail::Point(FVector3d(0,0,3));invalidCenter.Axis[0].Known=false;
        FDistanceBudget invalidWork{64};FRetainedTensorMinimum invalid(surface,invalidCenter,invalidWork,0.,47,nullptr,&memo);
        TestTrue(TEXT("invalid center is refused before any midpoint or snapshot work"),
            invalid.Advance().Status==EDistanceBoundStatus::Unknown&&invalidWork.Used==0&&
            memo.GetCountersForInspection().Requests==0&&memo.GetCountersForInspection().RetainedMiddleRequests==0);
    }
    TestTrue(TEXT("missing-sticky fallback actually exercised in all four control modes"),genuineFallbacks==4);
    return !HasAnyErrors();
}

// New same-invocation box tests; all previous 91 definitions remain byte-exact.
namespace Speed::Analytic::Certified::SameCallBoxTest4483
{
namespace T=MinimumMidpointTest4480;namespace A=::AnalyticCertified;
using Memo=Detail::FMinimumMidpointMemo;
struct FDirect
{
    bool Known=false,NumericRefused=false;
    RetainedMinimumDetail::FLeafBounds Bound;
    FAnalyticNumericEvidence Numeric;
    Memo::FCounters Stats;
};
inline FDirect Direct(const FIntervalTensorSurface& surface,const FIntervalPoint& center,
    const Interval* threshold,int32 path,bool enabled,uint32 initial,bool nullOwner=false)
{
    VectorSetControlRegister(initial);FDirect result;Memo memo;
    try
    {
        const uint64 control=RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush).ControlFingerprint();
        Memo::FCallScope scope(nullOwner?nullptr:&memo,surface,control,enabled);
        auto node=T::Node(surface);
        if(path==0)
        {
            result.Known=Detail::Bounds(node,center,threshold,&scope);
            if(result.Known)
            {
                result.Bound.LowerSquared=node.LowerSquared;result.Bound.UpperSquared=node.UpperSquared;
                result.Bound.WitnessPoint=node.WitnessPoint;result.Bound.WitnessUV=node.WitnessUV;
            }
        }
        else result.Known=RetainedMinimumDetail::Bounds(surface,node,center,result.Bound,threshold,&scope);
    }
    catch(const FAnalyticNumericUnknown&){result.NumericRefused=true;}
    result.Stats=memo.GetCountersForInspection();
    result.Numeric=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    return result;
}
inline bool Same(const FDirect& a,const FDirect& b)
{
    return a.Known==b.Known&&a.NumericRefused==b.NumericRefused&&a.Numeric.Status==b.Numeric.Status&&
        a.Numeric.Failure==b.Numeric.Failure&&a.Numeric.VectorControl==b.Numeric.VectorControl&&
        a.Numeric.ScalarControl==b.Numeric.ScalarControl&&a.Numeric.SnapshotVectorControl==b.Numeric.SnapshotVectorControl&&
        (!a.Known||(A::Bits(a.Bound.LowerSquared)==A::Bits(b.Bound.LowerSquared)&&
        A::Bits(a.Bound.UpperSquared)==A::Bits(b.Bound.UpperSquared)&&T::SamePoint(a.Bound.WitnessPoint,b.Bound.WitnessPoint)&&
        A::Bits(a.Bound.WitnessUV.X)==A::Bits(b.Bound.WitnessUV.X)&&A::Bits(a.Bound.WitnessUV.Y)==A::Bits(b.Bound.WitnessUV.Y)));
}
struct FWhole
{
    FTensorDistanceBounds Bound;FAnalyticNumericEvidence Numeric;Memo::FCounters Stats;
    uint64 Used=0;int32 Remaining=0,Floor=0;
};
inline FWhole Whole(bool retained,bool enabled,uint32 initial,int32 remaining=64)
{
    VectorSetControlRegister(initial);const auto surface=T::Plane();const auto center=Detail::Point(FVector3d(0,0,3));
    const Interval threshold=Interval::Exact(10.);FDistanceBudget work{remaining,7,0};Memo memo;FWhole out;
    if(retained)
    {FRetainedTensorMinimum value(surface,center,work,0.,16,&threshold,enabled?&memo:nullptr);out.Bound=value.Advance();}
    else out.Bound=BoundTensorDistanceSquaredIntervalCenter(surface,center,work,0.,16,&threshold,enabled?&memo:nullptr);
    out.Stats=memo.GetCountersForInspection();out.Used=work.Used;out.Remaining=work.Remaining;out.Floor=work.ConsumptionFloor;
    out.Numeric=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);return out;
}
inline bool SameWhole(const FWhole& a,const FWhole& b)
{
    return T::SameBounds(a.Bound,b.Bound)&&a.Used==b.Used&&a.Remaining==b.Remaining&&a.Floor==b.Floor&&
        a.Numeric.Status==b.Numeric.Status&&a.Numeric.Failure==b.Numeric.Failure&&
        a.Numeric.VectorControl==b.Numeric.VectorControl&&a.Numeric.ScalarControl==b.Numeric.ScalarControl&&
        a.Numeric.SnapshotVectorControl==b.Numeric.SnapshotVectorControl;
}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSameCallBoxReuse4483Test,
    "IAmSpeed.AnalyticWorld.TensorDistance.SameCallBoxReuse",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FSameCallBoxReuse4483Test::RunTest(const FString&)
{
    using namespace Speed::Analytic;using namespace Speed::Analytic::Certified;
    namespace T=MinimumMidpointTest4480;namespace X=SameCallBoxTest4483;namespace A=::AnalyticCertified;
    FTensorBoundsTestContext context;if(!TestTrue(TEXT("RNE fixture established"),context.Ready))return false;
    const uint32 original=VectorGetControlRegister();ON_SCOPE_EXIT{VectorSetControlRegister(original);};
    uint64 directCases=0,wholeCases=0;
    const Interval threshold{0.,100.,true};const auto center=Detail::Point(FVector3d(0,0,3));
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})for(uint32 sticky:{0u,0x3fu})
    {
        const uint32 initial=(original&~uint32(0x807f))|flush|sticky;
        for(int32 variant=0;variant<3;++variant)
        {
            auto surface=T::Plane();
            if(variant==1){surface.ControlPoints[0].Axis[2]={A::FromBits(A::Sign),0.,true};surface.ControlPoints.Last().Axis[2]={0.,A::FromBits(A::Sign),true};}
            if(variant==2){surface.ControlPoints[0].Axis[2]={-1.,.25,true};surface.ControlPoints.Last().Axis[2]={-.5,1.,true};}
            for(int32 path:{0,1})
            {
                const auto oracle=X::Direct(surface,center,&threshold,path,false,initial);
                const auto actual=X::Direct(surface,center,&threshold,path,true,initial);
                if(!TestTrue(TEXT("same-call endpoints preserve bound/witness bits and complete numeric evidence"),X::Same(oracle,actual)&&actual.Known))return false;
                if(!TestTrue(TEXT("both direct bounds genuinely avoid their second Min/Max pass"),
                    actual.Stats.OrdinaryBoxAvoidedCP==uint64(surface.ControlPoints.Num())&&actual.Stats.OrdinaryBoxRecomputedCP==0&&
                    oracle.Stats.OrdinaryBoxRecomputedCP==uint64(surface.ControlPoints.Num())&&oracle.Stats.OrdinaryBoxAvoidedCP==0))return false;
                ++directCases;
            }
        }
        for(bool retained:{false,true})
        {
            const auto oracle=X::Whole(retained,false,initial),actual=X::Whole(retained,true,initial);
            if(!TestTrue(TEXT("one-shot and private retained-cell bounds preserve results, witnesses, Used and flags"),
                X::SameWhole(oracle,actual)&&actual.Bound.Status!=EDistanceBoundStatus::Unknown&&actual.Stats.OrdinaryBoxAvoidedCP>0&&actual.Used>7))return false;
            const auto stoppedOracle=X::Whole(retained,false,initial,0),stoppedActual=X::Whole(retained,true,initial,0);
            if(!TestTrue(TEXT("budget exhaustion remains terminal without a box reuse"),
                X::SameWhole(stoppedOracle,stoppedActual)&&stoppedActual.Bound.Status==EDistanceBoundStatus::Unknown&&
                stoppedActual.Stats.OrdinaryBoxAvoidedCP==0&&stoppedActual.Used==7))return false;
            ++wholeCases;
        }
    }
    TestTrue(TEXT("all three real ordinary paths and eight numeric modes exercised nonvacuously"),directCases==48&&wholeCases==16);
    return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSameCallBoxScopeAndRefusal4483Test,
    "IAmSpeed.AnalyticWorld.TensorDistance.SameCallBoxScopeAndRefusal",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FSameCallBoxScopeAndRefusal4483Test::RunTest(const FString&)
{
    using namespace Speed::Analytic;using namespace Speed::Analytic::Certified;
    namespace T=MinimumMidpointTest4480;namespace X=SameCallBoxTest4483;namespace A=::AnalyticCertified;
    FTensorBoundsTestContext context;if(!TestTrue(TEXT("RNE fixture established"),context.Ready))return false;
    const uint32 original=VectorGetControlRegister();ON_SCOPE_EXIT{VectorSetControlRegister(original);};
    uint64 checked=0;const Interval threshold{0.,100.,true},farThreshold=Interval::Exact(1.);
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})for(uint32 sticky:{0u,0x3fu})for(int32 path:{0,1})
    {
        const uint32 initial=(original&~uint32(0x807f))|flush|sticky;
        for(int32 variant=0;variant<9;++variant)
        {
            auto surface=T::Plane();auto center=Detail::Point(FVector3d(0,0,3));const Interval* selected=&threshold;
            if(variant==0)selected=nullptr;
            if(variant==1){center=Detail::Point(FVector3d(0,0,20));selected=&farThreshold;}
            if(variant==2)surface.ControlPoints[0].Axis[0].Known=false;
            if(variant==3)surface.ControlPoints[0].Axis[0]={A::FromBits(1),A::FromBits(1),true};
            if(variant==4)surface.ControlPoints[0].Axis[0]={2.,1.,true};
            if(variant==5)surface.ControlPoints[0].Axis[0]={A::FromBits(A::Infinity),A::FromBits(A::Infinity),true};
            if(variant==6)surface.ControlPoints[0].Axis[0]={A::FromBits(A::Infinity|1),A::FromBits(A::Infinity|1),true};
            if(variant==7)center.Axis[2].Known=false;
            if(variant==8){surface.DegreeU=0;surface.ControlPoints.Reset();}
            const auto oracle=X::Direct(surface,center,selected,path,false,initial);
            const auto actual=X::Direct(surface,center,selected,path,true,initial);
            if(!TestTrue(TEXT("NULL, Decided and refused inputs preserve every numeric flag without publishing a box"),
                X::Same(oracle,actual)&&actual.Stats.OrdinaryBoxAvoidedCP==0))return false;
            if(variant==0&&!TestTrue(TEXT("NULL threshold genuinely retains the original full second scan"),actual.Known&&actual.Stats.OrdinaryBoxRecomputedCP==4))return false;
            if(variant==1&&!TestTrue(TEXT("quick-decided remains before all ordinary scans and midpoint requests"),actual.Known&&actual.Stats.OrdinaryBoxRecomputedCP==0&&actual.Stats.Requests==0))return false;
            if(variant>=2&&!TestTrue(TEXT("invalid/refused geometry stays Unknown before reuse"),!actual.Known&&actual.Stats.OrdinaryBoxRecomputedCP==0))return false;
            ++checked;
        }
        const auto surface=T::Plane();const auto center=Detail::Point(FVector3d(0,0,3));
        const auto nullOracle=X::Direct(surface,center,&threshold,path,false,initial,true);
        const auto nullEnabled=X::Direct(surface,center,&threshold,path,true,initial,true);
        if(!TestTrue(TEXT("null memo scope cannot authorize same-call reuse or record counters"),X::Same(nullOracle,nullEnabled)&&nullEnabled.Known&&
            nullEnabled.Stats.OrdinaryBoxAvoidedCP==0&&nullEnabled.Stats.OrdinaryBoxRecomputedCP==0))return false;
        const uint32 wrongRounding=(initial&~uint32(0x6000))|0x2000;
        const auto badOracle=X::Direct(surface,center,&threshold,path,false,wrongRounding);
        const auto badActual=X::Direct(surface,center,&threshold,path,true,wrongRounding);
        if(!TestTrue(TEXT("unsupported numeric context is refused identically and not repaired"),X::Same(badOracle,badActual)&&badActual.NumericRefused&&
            badActual.Stats.OrdinaryBoxAvoidedCP==0))return false;
    }
    TestTrue(TEXT("144 distinct refusal/lazy scenarios actually inspected"),checked==144);
    return !HasAnyErrors();
}

// Source4484 observation-only tests. No physical/world or speed acceptance.
namespace Speed::Analytic::Certified::MinimumHashProfileTest4484
{
namespace T=MinimumMidpointTest4480;
namespace P=Detail::MinimumMidpointProfile84;
struct FObservation
{
    FIntervalPoint Cold,Warm,NextScope;
    P::FCounters Before,After;
    Detail::FMinimumMidpointMemo::FCounters Memo;
    uint32 Flags=0;
};
inline FObservation Observe(bool enabled,uint32 initial)
{
    const auto surface=T::Plane();const auto node=T::Node(surface);
    VectorSetControlRegister(initial);
    const auto control=RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush).ControlFingerprint();
    Detail::FMinimumMidpointMemo memo;FObservation result;result.Before=P::Counters();
    {
        P::FEnabledScope observe(enabled);
        {Detail::FMinimumMidpointMemo::FCallScope scope(&memo,surface,control);result.Cold=scope.Midpoint(node);result.Warm=scope.Midpoint(node);}
        {Detail::FMinimumMidpointMemo::FCallScope scope(&memo,surface,control);result.NextScope=scope.Midpoint(node);}
    }
    result.After=P::Counters();result.Memo=memo.GetCountersForInspection();result.Flags=VectorGetControlRegister();return result;
}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMinimumHashProfileCounts4484Test,
    "IAmSpeed.AnalyticWorld.TensorDistance.MinimumHashProfileCounts",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FMinimumHashProfileCounts4484Test::RunTest(const FString&)
{
    using namespace Speed::Analytic;using namespace Speed::Analytic::Certified;
    namespace X=MinimumHashProfileTest4484;namespace P=Detail::MinimumMidpointProfile84;namespace T=MinimumMidpointTest4480;
    FTensorBoundsTestContext context;if(!TestTrue(TEXT("RNE observation fixture"),context.Ready))return false;
    const uint32 original=VectorGetControlRegister();ON_SCOPE_EXIT{VectorSetControlRegister(original);};
    P::FEnabledScope preserveDisabled(false);uint64 cases=0;
    const uint64 expected[P::StageCount]={2,3,1,2,3};
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})for(uint32 sticky:{0u,0x3fu})
    {
        const uint32 initial=(original&~uint32(0x807f))|flush|sticky;
        const auto oracle=X::Observe(false,initial),observed=X::Observe(true,initial);
        if(!TestTrue(TEXT("profile preserves all midpoint bits and complete vector control/flags"),
            T::SamePoint(oracle.Cold,observed.Cold)&&T::SamePoint(oracle.Warm,observed.Warm)&&T::SamePoint(oracle.NextScope,observed.NextScope)&&oracle.Flags==observed.Flags))return false;
        if(!TestTrue(TEXT("real cold and two warm hits remain nonvacuous"),observed.Memo.Requests==3&&observed.Memo.Hits==2&&observed.Memo.Builds==1&&
            oracle.Memo.Requests==observed.Memo.Requests&&oracle.Memo.Hits==observed.Memo.Hits&&oracle.Memo.Builds==observed.Memo.Builds))return false;
        for(uint32 stage=0;stage<P::StageCount;++stage)
        {
            if(!TestTrue(TEXT("every exact operation counted and closed; tick zero allowed at finite resolution"),
                observed.After.Calls[stage]-observed.Before.Calls[stage]==expected[stage]&&
                observed.After.Finished[stage]-observed.Before.Finished[stage]==expected[stage]&&
                observed.Before.InFlight[stage]==0&&observed.After.InFlight[stage]==0&&
                observed.After.Unwinding[stage]==observed.Before.Unwinding[stage]&&observed.After.Ticks[stage]>=observed.Before.Ticks[stage]))return false;
            if(!TestTrue(TEXT("disabled observation performs no capture or cycle accounting"),
                oracle.After.Calls[stage]==oracle.Before.Calls[stage]&&oracle.After.Finished[stage]==oracle.Before.Finished[stage]&&
                oracle.After.InFlight[stage]==oracle.Before.InFlight[stage]&&oracle.After.Ticks[stage]==oracle.Before.Ticks[stage]&&oracle.After.Unwinding[stage]==oracle.Before.Unwinding[stage]))return false;
        }
        if(!TestTrue(TEXT("snapshot integrity retained"),observed.Before.Integrity&&observed.After.Integrity&&oracle.After.Integrity))return false;
        ++cases;
    }
    TestTrue(TEXT("all eight FTZ/DAZ+sticky contexts exercised"),cases==8);return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMinimumHashProfileDisabledAndNesting4484Test,
    "IAmSpeed.AnalyticWorld.TensorDistance.MinimumHashProfileDisabledAndNesting",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FMinimumHashProfileDisabledAndNesting4484Test::RunTest(const FString&)
{
    using namespace Speed::Analytic;using namespace Speed::Analytic::Certified;
    namespace P=Detail::MinimumMidpointProfile84;
    FTensorBoundsTestContext context;if(!TestTrue(TEXT("RNE nested observation fixture"),context.Ready))return false;
    const uint32 beforeFlags=VectorGetControlRegister();P::FEnabledScope disabled(false);const auto before=P::Counters();
    {
        P::FEnabledScope enabled(true);
        {
            P::FScope parent(P::RootHash);
            {P::FEnabledScope off(false);P::FScope ignored(P::SameRoot);}
            {P::FScope child(P::CellHash);child.Stop();child.Stop();}
            const auto active=P::Counters();
            if(!TestTrue(TEXT("nested scopes expose active parent; child stop idempotent"),P::Enabled()&&active.InFlight[P::RootHash]==1&&
                active.Finished[P::RootHash]==before.Finished[P::RootHash]&&active.Calls[P::CellHash]-before.Calls[P::CellHash]==1&&active.InFlight[P::CellHash]==0))return false;
        }
        try{P::FScope unwinding(P::SameCell);throw 1;}catch(int){}
    }
    const auto after=P::Counters();
    TestTrue(TEXT("enabled state restored, no numeric control or sticky flag mutation"),!P::Enabled()&&VectorGetControlRegister()==beforeFlags);
    TestTrue(TEXT("no double finish, disabled branch unobserved, unwind recorded"),after.Integrity&&
        after.Calls[P::RootHash]-before.Calls[P::RootHash]==1&&after.Finished[P::RootHash]-before.Finished[P::RootHash]==1&&
        after.Calls[P::CellHash]-before.Calls[P::CellHash]==1&&after.Finished[P::CellHash]-before.Finished[P::CellHash]==1&&
        after.Calls[P::SameRoot]==before.Calls[P::SameRoot]&&after.Unwinding[P::SameCell]-before.Unwinding[P::SameCell]==1);
    for(uint32 stage=0;stage<P::StageCount;++stage)TestTrue(TEXT("all final observation scopes closed"),after.InFlight[stage]==0);
    return !HasAnyErrors();
}

// Endpoint reuse changes only repeated time-predicate work. It is not a
// world/physics acceptance fixture and supplies no new contact tolerance.
namespace Speed::Analytic::Certified::TimeDetail
{
struct FTimeEndpointMemoTestAccess
{
    static FUnionBound Endpoint(FRetainedSphereTimeSearch& s,FTimeNode& n,Interval r2)
    {return s.EvaluateEndpointSpatial(n,r2);}
    static bool Share(FRetainedSphereTimeSearch& s,const FTimeNode& n,FTimeNode& right)
    {return s.ShareEndpointRight(n,right);}
};
}
namespace
{
bool SameEndpointTimeResult(const Speed::Analytic::Certified::FTensorSphereTimeBounds& a,
    const Speed::Analytic::Certified::FTensorSphereTimeBounds& b)
{
    namespace C=Speed::Analytic::Certified;namespace A=AnalyticCertified;
    if(a.Status!=b.Status||A::Bits(a.LowerTime)!=A::Bits(b.LowerTime)||A::Bits(a.UpperTime)!=A::Bits(b.UpperTime)||
        a.InsideWitnessSurfaceIndex!=b.InsideWitnessSurfaceIndex||a.InitialSeparationProved!=b.InitialSeparationProved||
        A::Bits(a.InsideSampleUV.X)!=A::Bits(b.InsideSampleUV.X)||A::Bits(a.InsideSampleUV.Y)!=A::Bits(b.InsideSampleUV.Y))return false;
    for(int32 d=0;d<3;++d)if(a.InsideSurfacePoint.Axis[d].Known!=b.InsideSurfacePoint.Axis[d].Known||
        A::Bits(a.InsideSurfacePoint.Axis[d].Lower)!=A::Bits(b.InsideSurfacePoint.Axis[d].Lower)||
        A::Bits(a.InsideSurfacePoint.Axis[d].Upper)!=A::Bits(b.InsideSurfacePoint.Axis[d].Upper))return false;
    return true;
}
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTimeEndpointMemoRightAndRefinementTest,
    "IAmSpeed.AnalyticWorld.TensorContact.TimeEndpointMemo.RightAndRefinement",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FTimeEndpointMemoRightAndRefinementTest::RunTest(const FString&)
{
    using namespace Speed::Analytic;using namespace Speed::Analytic::Certified;
    namespace TD=TimeDetail;
    FTensorBoundsTestContext context;if(!TestTrue(TEXT("RNE endpoint fixture"),context.Ready))return false;
    const uint32 original=VectorGetControlRegister();ON_SCOPE_EXIT{VectorSetControlRegister(original);};
    TArray<FIntervalTensorSurface> surfaces;FIntervalTensorSurface point;point.ControlPoints.Add(Detail::Point(FVector3d::ZeroVector));surfaces.Add(point);
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})for(uint32 sticky:{0u,0x3fu})for(int32 degree:{1,2})
    {
        const uint32 initial=(original&~uint32(0x807f))|flush|sticky;
        FIntervalBezierCenterPath path;path.Degree=degree;path.ControlPoints.Add(Detail::Point(FVector3d(9,0,0)));
        if(degree==2)path.ControlPoints.Add(Detail::Point(FVector3d(4.5,0,0)));
        path.ControlPoints.Add(Detail::Point(FVector3d::ZeroVector));
        FDistanceBudget baselineWork{1000000},memoWork{1000000};
        VectorSetControlRegister(initial);
        TD::FRetainedSphereTimeSearch baseline(surfaces,path,2.,baselineWork,1.e-7,128,nullptr,false,nullptr,false);
        const auto coarse=baseline.Advance(1./64.),fine=baseline.Advance(1./4096.);const uint32 baselineFlags=VectorGetControlRegister();
        VectorSetControlRegister(initial);
        TD::FRetainedSphereTimeSearch cached(surfaces,path,2.,memoWork,1.e-7,128,nullptr,false,nullptr,true);
        const auto cachedCoarse=cached.Advance(1./64.);const auto before=cached.GetEndpointCountersForInspection();
        const auto cachedFine=cached.Advance(1./4096.);const auto after=cached.GetEndpointCountersForInspection();
        if(!TestTrue(TEXT("same exact first-contact brackets, UV witness, endpoint values and numeric flags"),
            coarse.Status==ETensorSphereTimeStatus::FirstContactBracket&&fine.Status==ETensorSphereTimeStatus::FirstContactBracket&&
            SameEndpointTimeResult(coarse,cachedCoarse)&&SameEndpointTimeResult(fine,cachedFine)&&VectorGetControlRegister()==baselineFlags))return false;
        if(!TestTrue(TEXT("real right descendants and deferred refinement reuse completed endpoint proofs"),
            before.Hits>0&&after.Hits>before.Hits&&after.RightInheritances>0&&after.Publications==after.Evaluations&&
            after.Requests==after.Evaluations+after.Hits&&memoWork.Used>0&&memoWork.Remaining==1000000-int32(memoWork.Used)))return false;
        if(!TestTrue(TEXT("oracle flag leaves endpoint memo completely dormant"),baseline.GetEndpointCountersForInspection().Requests==0))return false;
    }
    return !HasAnyErrors();
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTimeEndpointMemoIdentityAndBudgetTest,
    "IAmSpeed.AnalyticWorld.TensorContact.TimeEndpointMemo.IdentityAndBudget",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FTimeEndpointMemoIdentityAndBudgetTest::RunTest(const FString&)
{
    using namespace Speed::Analytic;using namespace Speed::Analytic::Certified;
    namespace TD=TimeDetail;namespace A=AnalyticCertified;
    FTensorBoundsTestContext context;if(!TestTrue(TEXT("RNE identity fixture"),context.Ready))return false;
    const uint32 original=VectorGetControlRegister();ON_SCOPE_EXIT{VectorSetControlRegister(original);};
    TArray<FIntervalTensorSurface> surfaces;FIntervalTensorSurface point;point.ControlPoints.Add(Detail::Point(FVector3d::ZeroVector));surfaces.Add(point);
    FIntervalBezierCenterPath path;if(!CenterDetail::Linear(FVector3d(9,0,0),FVector3d(.25,0,0),path))return false;
    FDistanceBudget work{100000};TD::FRetainedSphereTimeSearch search(surfaces,path,2.,work,1.e-7,128,nullptr,false,nullptr,true);
    TD::FTimeNode node{0.,1.,0,path};const auto r2=A::Square(Interval::Exact(2));
    const auto first=TD::FTimeEndpointMemoTestAccess::Endpoint(search,node,r2);
    if(!TestTrue(TEXT("known endpoint proof published only after real evaluation"),first.Known&&first.AnyInside&&node.Endpoint.IsSet()))return false;
    const uint64 before=work.Used;const auto second=TD::FTimeEndpointMemoTestAccess::Endpoint(search,node,r2);
    if(!TestTrue(TEXT("hit uses one live validation debit and no refund"),second.Known&&second.AnyInside&&work.Used==before+1&&search.GetEndpointCountersForInspection().Hits==1))return false;
    FIntervalBezierCenterPath leftPath,rightPath;if(!CenterDetail::Split(path,leftPath,rightPath))return false;
    TD::FTimeNode right{.5,1.,1,rightPath},left{0.,.5,1,leftPath};
    if(!TestTrue(TEXT("only the exactly copied right endpoint inherits"),TD::FTimeEndpointMemoTestAccess::Share(search,node,right)&&right.Endpoint.IsSet()&&
        TD::FTimeEndpointMemoTestAccess::Share(search,node,left)&&!left.Endpoint.IsSet()))return false;
    auto changed=rightPath;changed.ControlPoints.Last().Axis[0].Upper=A::FromBits(A::Bits(changed.ControlPoints.Last().Axis[0].Upper)+1);
    TD::FTimeNode oneUlp{.5,1.,1,changed};
    if(!TestTrue(TEXT("one-ULP endpoint enclosure difference forbids sharing"),TD::FTimeEndpointMemoTestAccess::Share(search,node,oneUlp)&&!oneUlp.Endpoint.IsSet()))return false;
    const uint64 floorUsed=work.Used;const int32 remaining=work.Remaining;work.ConsumptionFloor=remaining;
    if(!TestTrue(TEXT("floor-denied cache hit fails closed without refund or debit"),!TD::FTimeEndpointMemoTestAccess::Endpoint(search,right,r2).Known&&work.Used==floorUsed&&work.Remaining==remaining))return false;
    work.ConsumptionFloor=0;
    FDistanceBudget otherWork{100000};TD::FRetainedSphereTimeSearch other(surfaces,path,2.,otherWork,1.e-7,128,nullptr,false,nullptr,true);
    if(!TestTrue(TEXT("same geometry in another operation cannot borrow this proof"),TD::FTimeEndpointMemoTestAccess::Endpoint(other,right,r2).Known&&
        other.GetEndpointCountersForInspection().Hits==0&&other.GetEndpointCountersForInspection().Evaluations==1))return false;
    // A Known but undecided endpoint is cached as undecided, never as separation or penetration.
    FIntervalBezierCenterPath uncertainPath;uncertainPath.Degree=0;auto uncertainPoint=Detail::Point(FVector3d(2,0,0));uncertainPoint.Axis[0]={1.,3.,true};uncertainPath.ControlPoints.Add(uncertainPoint);
    FDistanceBudget uncertainWork{100000};TD::FRetainedSphereTimeSearch uncertain(surfaces,uncertainPath,2.,uncertainWork,1.e-7,1,nullptr,false,nullptr,true);
    TD::FTimeNode uncertainNode{0.,1.,0,uncertainPath};const auto u=TD::FTimeEndpointMemoTestAccess::Endpoint(uncertain,uncertainNode,r2);
    const auto uh=TD::FTimeEndpointMemoTestAccess::Endpoint(uncertain,uncertainNode,r2);
    if(!TestTrue(TEXT("bounded undecided remains undecided after memoization"),u.Known&&!u.AnyInside&&!u.AllClear&&uh.Known&&!uh.AnyInside&&!uh.AllClear&&
        uncertain.GetEndpointCountersForInspection().Hits==1))return false;
    FDistanceBudget zero{0};TD::FRetainedSphereTimeSearch failed(surfaces,path,2.,zero,1.e-7,128,nullptr,false,nullptr,true);
    TD::FTimeNode failedNode{0.,1.,0,path};
    if(!TestTrue(TEXT("partial work failure publishes no endpoint proof"),!TD::FTimeEndpointMemoTestAccess::Endpoint(failed,failedNode,r2).Known&&!failedNode.Endpoint.IsSet()))return false;
    const auto count=search.GetEndpointCountersForInspection().Hits;VectorSetControlRegister((VectorGetControlRegister()&~uint32(0x6000))|uint32(0x2000));
    if(!TestTrue(TEXT("changed numeric control cannot reuse a proof"),!TD::FTimeEndpointMemoTestAccess::Endpoint(search,node,r2).Known&&search.GetEndpointCountersForInspection().Hits==count))return false;
    return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAnalyticContactRootLruTest,
 "IAmSpeed.AnalyticWorld.TensorContact.GeometryRootLruEquivalence",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FAnalyticContactRootLruTest::RunTest(const FString&)
{
    using namespace Speed::Analytic;using namespace Certified;using Memo=ContactDetail::FContactCellGeometryMemo;using Net=Memo::ENet;
    FTensorBoundsTestContext context;if(!TestTrue(TEXT("RNE fixture"),context.Ready))return false;
    FScopedCVar<int32> enabled(TEXT("p.IAmSpeed.AnalyticWorld.ContactCellRootLru"),1,FName(TEXT("ContactRootLruEquivalence")));
    const uint32 original=VectorGetControlRegister();ON_SCOPE_EXIT{VectorSetControlRegister(original);};
    const auto pointRoot=[](double x,Detail::FNode& p,Detail::FNode& du,Detail::FNode& dv)
    {
        p={};p.Net.Add(Detail::Point(FVector3d(x,0,0)));
        return ContactDetail::OriginalDerivativeNet(p,true,du)&&ContactDetail::OriginalDerivativeNet(p,false,dv);
    };
    for(uint32 flush:{0u,64u,32768u,32832u})
    {
        VectorSetControlRegister((original&~uint32(0x807f))|flush|0x3f);
        const uint64 control=RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush).ControlFingerprint();
        Memo memo;Memo::FCellPtr pinnedOldest,hot;
        {
            Memo::FCallScope scope(&memo,control);
            for(int32 k=0;k<64;++k)
            {
                Detail::FNode p,du,dv;if(!pointRoot(k,p,du,dv))return false;
                auto cell=scope.FindRoot(p,du,dv);if(!TestTrue(TEXT("publish within unchanged root quota"),bool(cell)))return false;
                if(k==0)hot=cell;else if(k==1)pinnedOldest=cell;
            }
            Detail::FNode p,du,dv;if(!pointRoot(0,p,du,dv))return false;
            TestTrue(TEXT("touching a root preserves exact handle and marks it recent"),scope.FindRoot(p,du,dv)==hot);
            if(!pointRoot(65,p,du,dv))return false;
            TestFalse(TEXT("full active directory refuses new publication"),bool(scope.FindRoot(p,du,dv)));
            {
                Memo::FCallScope nested(&memo,control);
                TestFalse(TEXT("nested call cannot evict outer directory"),bool(nested.FindRoot(p,du,dv)));
                TestTrue(TEXT("same root/cell limits while reentrant"),memo.GetDirectoryRootsForInspection()==64&&
                    memo.GetLiveCellsForInspection()==64&&memo.GetCountersForInspection().GeometryEvictions==0);
            }
        }
        const auto* pinnedAddress=&pinnedOldest->Get(Net::Position);
        {
            Memo::FCallScope scope(&memo,control);
            TestTrue(TEXT("only one old root retired at outer boundary, remaining 63 preserved"),
                memo.GetDirectoryRootsForInspection()==63&&memo.GetCountersForInspection().GeometryEvictions==1);
            TestTrue(TEXT("retained oldest cell and reference survive directory removal and remain charged"),
                memo.GetLiveCellsForInspection()==64 && &pinnedOldest->Get(Net::Position)==pinnedAddress&&
                pinnedAddress->Net[0].Axis[0].Lower==1);
            Detail::FNode p,du,dv;if(!pointRoot(0,p,du,dv))return false;
            TestTrue(TEXT("hot root survives boundary without reconstruction"),scope.FindRoot(p,du,dv)==hot);
            if(!pointRoot(65,p,du,dv))return false;
            TestTrue(TEXT("one new root uses released directory slot without discounting pinned memory"),
                bool(scope.FindRoot(p,du,dv))&&memo.GetDirectoryRootsForInspection()==64&&memo.GetLiveCellsForInspection()==65);
            TestTrue(TEXT("unchanged live-cell and point ceilings"),memo.GetLiveCellsForInspection()<=65536&&memo.GetLivePointsForInspection()<=1048576);
            Memo::FCallScope wrong(&memo,control^(uint64(1)<<40));
            TestFalse(TEXT("incompatible numeric fingerprint cannot use retained directory"),bool(wrong.FindRoot(p,du,dv)));
        }
        pinnedOldest.Reset();hot.Reset();
        TestTrue(TEXT("retired cell is released when its last external handle ends"),memo.GetLiveCellsForInspection()==64&&memo.GetLivePointsForInspection()==192);

        TArray<FTensorBezierSurface> nominal{ContactUnitBowl()};TArray<FIntervalTensorSurface> surfaces;
        if(!Detail::PromoteTensors(nominal,surfaces))return false;
        for(uint32 flags:{0u,63u})for(int32 limit:{32,131072})
        {
            Memo contactMemo;
            {
                Memo::FCallScope scope(&contactMemo,control);
                for(int32 k=0;k<63;++k)
                {Detail::FNode p,du,dv;if(!pointRoot(10000+k,p,du,dv)||!scope.FindRoot(p,du,dv))return false;}
            }
            FIntervalBezierCenterPath path;
            if(!CenterDetail::Linear(FVector3d(0,0,1),FVector3d(0,0,-1),path))return false;
            int32 oracleCalls=0;
            FContactEnclosureFilter oracleFilter=[&](const FTensorSphereContactEnclosure&)
            {++oracleCalls;return oracleCalls==1?EContactEnclosureDecision::Refine:EContactEnclosureDecision::Accept;};
            VectorSetControlRegister((original&~uint32(0x807f))|flush|flags);
            FDistanceBudget referenceBudget{limit,7,0};
            const auto reference=BoundTensorSphereFirstContact(surfaces,path,.25,referenceBudget,1./65536.,1.e-7,256,.01,.05,24,&oracleFilter,nullptr);
            const auto referenceBits=ContactMemo71Signature(reference);const uint32 expectedFlags=VectorGetControlRegister();
            for(int32 repeat=0;repeat<2;++repeat)
            {
                int32 calls=0;FContactEnclosureFilter filter=[&](const FTensorSphereContactEnclosure&)
                {++calls;return calls==1?EContactEnclosureDecision::Refine:EContactEnclosureDecision::Accept;};
                VectorSetControlRegister((original&~uint32(0x807f))|flush|flags);
                FDistanceBudget work{limit,7,0};
                const auto result=BoundTensorSphereFirstContact(surfaces,path,.25,work,1./65536.,1.e-7,256,.01,.05,24,&filter,&contactMemo);
                const uint32 returnedFlags=VectorGetControlRegister();
                TestTrue(TEXT("full contact output, callbacks, work and numeric flags unchanged across LRU boundary"),
                    ContactMemo71Signature(result)==referenceBits&&work.Remaining==referenceBudget.Remaining&&work.Used==referenceBudget.Used&&
                    work.ConsumptionFloor==referenceBudget.ConsumptionFloor&&calls==oracleCalls&&returnedFlags==expectedFlags);
                // Attempt a new root after the real contact populated the
                // last slot. The next outer query must retain its hot root.
                if(repeat==0&&contactMemo.GetDirectoryRootsForInspection()==64)
                {
                    Memo::FCallScope pressure(&contactMemo,control);Detail::FNode p,du,dv;
                    if(!pointRoot(20000,p,du,dv))return false;
                    TestFalse(TEXT("pressure remains a miss inside its own scope"),bool(pressure.FindRoot(p,du,dv)));
                }
            }
            if(limit==131072)TestTrue(TEXT("nonvacuous warm geometry reuse after partial retirement"),
                contactMemo.GetCountersForInspection().GeometryEvictions==1&&contactMemo.GetCountersForInspection().ChildrenHits>0&&
                contactMemo.GetDirectoryRootsForInspection()==63);
        }
    }
    return !HasAnyErrors();
}
#if WITH_DEV_AUTOMATION_TESTS
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMinimumLookupNumericInvocationTest,
    "IAmSpeed.AnalyticWorld.TensorDistance.MinimumLookupNumericInvocation",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FMinimumLookupNumericInvocationTest::RunTest(const FString&)
{
    using namespace Speed::Analytic;using namespace Speed::Analytic::Certified;
    namespace A=::AnalyticCertified;namespace T=MinimumMidpointTest4480;
    using Memo=Detail::FMinimumMidpointMemo;
    FTensorBoundsTestContext context;if(!TestTrue(TEXT("RNE fixture"),context.Ready))return false;
    const uint32 original=VectorGetControlRegister();ON_SCOPE_EXIT{VectorSetControlRegister(original);};
    uint64 actualHits=0,actualDeclines=0;
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})
    {
        const uint32 clean=(original&~uint32(0x807f))|flush;VectorSetControlRegister(clean);
        const uint64 control=RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush).ControlFingerprint();
        FIntervalTensorSurface surface;surface.DegreeU=1;
        surface.ControlPoints={Detail::Point(FVector3d(1.,.3,.7)),Detail::Point(FVector3d(A::FromBits(0x3c90000000000000ULL),.1,.2))};
        const auto node=T::Node(surface);Memo memo;
        Memo::FCallScope scope(&memo,surface,control);
        VectorSetControlRegister(clean);const auto expected=Detail::Midpoint(node);const uint32 required=VectorGetControlRegister()&0x3f;
        if(!TestTrue(TEXT("nonvacuous arithmetic status requirement"),required!=0))return false;
        VectorSetControlRegister(clean);const auto cold=scope.Midpoint(node);
        if(!TestTrue(TEXT("cold geometry still publishes only its real post-arithmetic status"),T::SamePoint(cold,expected)&&
            (VectorGetControlRegister()&0x3f)==required&&memo.GetCountersForInspection().Builds>0))return false;
        // Stay inside the SAME call scope across deliberate mutations. No
        // scope-level evidence or sticky status can satisfy a later invocation.
        for(uint32 requested:{clean|0x3fu,clean,(clean^0x40u)|0x3fu,(clean^0x8000u)|0x3fu,clean|0x2000u|0x3fu,clean|0x3fu})
        {
            VectorSetControlRegister(requested);const auto oracle=Detail::Midpoint(node);const uint32 oracleFlags=VectorGetControlRegister();
            const uint64 priorHits=memo.GetCountersForInspection().Hits;
            VectorSetControlRegister(requested);const auto actual=scope.Midpoint(node);const uint32 actualFlags=VectorGetControlRegister();
            const bool mayHit=(requested&~uint32(0x3f))==(clean&~uint32(0x3f))&&(requested&required)==required;
            const uint64 hitDelta=memo.GetCountersForInspection().Hits-priorHits;
            if(!TestTrue(TEXT("each invocation authenticates fresh FTZ DAZ rounding and sticky status before its integer lookup"),
                T::SamePoint(oracle,actual)&&actualFlags==oracleFlags&&hitDelta==(mayHit?1u:0u)))return false;
            if(mayHit)++actualHits;else ++actualDeclines;
        }
        VectorSetControlRegister(clean|0x3fu);
        {Memo::FCallScope nested(&memo,surface,control);const auto inside=nested.Midpoint(node);
            if(!TestTrue(TEXT("nested scope keeps its own fresh invocation and root binding"),T::SamePoint(inside,expected)&&memo.GetActiveCallsForInspection()==2))return false;}
        if(!TestTrue(TEXT("nested destruction preserves the outer exact geometry only"),memo.GetActiveCallsForInspection()==1))return false;
    }
    TestTrue(TEXT("both lookup reuse and fresh-context refusal were exercised"),actualHits==8&&actualDeclines==16);
    return !HasAnyErrors();
}
#endif
