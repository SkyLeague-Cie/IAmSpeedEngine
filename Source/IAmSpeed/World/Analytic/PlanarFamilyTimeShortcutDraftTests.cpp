#if WITH_DEV_AUTOMATION_TESTS
#include "Misc/AutomationTest.h"
#include "TensorSphereContactBounds.h"
#include "IAmSpeed/World/Simulation/FrameFloatingPointContext.h"

namespace PlanarFamilyDraftTest
{
namespace C=Speed::Analytic::Certified;
namespace A=::AnalyticCertified;
using I=A::Interval;
struct FNumeric
{
    Speed::FFrameFloatingPointContext Original=Speed::CaptureFrameFloatingPointContext();
    Speed::FScopedFrameFloatingPointContext Restore{Original};
    std::fenv_t Held;
    bool Ready=Original.bValid&&Restore.IsApplied()&&std::feholdexcept(&Held)==0&&std::fesetround(FE_TONEAREST)==0;
};
C::FIntervalTensorSurface Triangle(double height,int32 axis=2)
{
    C::FIntervalTensorSurface s;s.DegreeU=s.DegreeV=1;
    const int32 x=(axis+1)%3,y=(axis+2)%3;
    for(const auto& pair:{FVector2d(-8,-8),FVector2d(8,-8),FVector2d(0,8),FVector2d(0,8)})
    {
        FVector3d p=FVector3d::ZeroVector;p[axis]=height;p[x]=pair.X;p[y]=pair.Y;
        s.ControlPoints.Add(C::Detail::Point(p));
    }
    return s;
}
C::FIntervalTensorSurface Curve(double x=0)
{
    // S(u,v)=(u, v, 1+v*v), u in[-2,2],v in[-1,1].
    C::FIntervalTensorSurface s;s.DegreeU=1;s.DegreeV=2;
    for(double u:{-2.,2.})for(const auto& yz:{FVector2d(-1,2),FVector2d(0,0),FVector2d(1,2)})
        s.ControlPoints.Add(C::Detail::Point(FVector3d(u+x,yz.X,yz.Y)));
    return s;
}
bool Path(const FVector3d& a,const FVector3d& b,C::FIntervalBezierCenterPath& path)
{return C::CenterDetail::Linear(a,b,path);}
FVector3d OriginalTriangleAtFixedUV(const C::FIntervalTensorSurface& s,FVector2d uv)
{
    // Independent analytic triangle formula; no production interval evaluator.
    FVector3d a,b,c;
    for(int32 d=0;d<3;++d)
    {a[d]=s.ControlPoints[0].Axis[d].Lower;b[d]=s.ControlPoints[1].Axis[d].Lower;c[d]=s.ControlPoints[2].Axis[d].Lower;}
    return (1.-uv.X)*((1.-uv.Y)*a+uv.Y*b)+uv.X*c;
}
bool Contains(const C::FIntervalPoint& box,const FVector3d& p)
{
    for(int32 d=0;d<3;++d)if(!A::Valid(box.Axis[d])||p[d]<box.Axis[d].Lower||p[d]>box.Axis[d].Upper)return false;
    return true;
}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlanarFamilyDraftViews,
    "IAmSpeed.AnalyticWorld.TensorTime.PlanarFamilyDraft.NoncontiguousViews",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FPlanarFamilyDraftViews::RunTest(const FString&)
{
    using namespace PlanarFamilyDraftTest;FNumeric numeric;
    if(!TestTrue(TEXT("numeric fixture"),numeric.Ready))return false;
    const uint32 controls=VectorGetControlRegister();
    for(uint32 flush:{0u,64u,32768u,32832u})
    {
        VectorSetControlRegister((controls&~uint32(0x8040))|flush);
        TArray<C::FIntervalTensorSurface> surfaces{Triangle(0),Curve(),Triangle(0),Triangle(1),Triangle(0,0)};
        C::FIntervalBezierCenterPath path;
        if(!TestTrue(TEXT("actual fixed linear path"),Path(FVector3d(0,0,3),FVector3d(0,0,-1),path)))return false;
        C::FDistanceBudget work{200000};TUniquePtr<C::FPlanarFamilyTimeShortcutPlan> plan;
        if(!TestTrue(TEXT("mixed source builds a local plane plan"),C::FPlanarFamilyTimeShortcutPlan::Build(surfaces,path,work,plan)==C::EPlanarFamilyBuild::Ready&&plan))return false;
        TestTrue(TEXT("noncontiguous original indices share exactly their plane family"),
            plan->GetFamilyForProviderForInspection(0)==plan->GetFamilyForProviderForInspection(2)&&
            plan->GetFamilyLocalForInspection(0)==0&&plan->GetFamilyLocalForInspection(2)==1);
        TestTrue(TEXT("curve stays generic; distinct heights and normal axes remain distinct"),
            plan->GetFamilyForProviderForInspection(1)==INDEX_NONE&&
            plan->GetFamilyForProviderForInspection(3)!=plan->GetFamilyForProviderForInspection(0)&&
            plan->GetFamilyForProviderForInspection(4)!=plan->GetFamilyForProviderForInspection(0));
        const TArray<int32> members{0,2};
        C::ContactDetail::FPlanarProjectionDomains proof;
        TestTrue(TEXT("indexed certificate covers the original noncontiguous views"),
            C::ContactDetail::ProjectedPlanarDomainsIndexed(surfaces,members,C::CenterDetail::Hull(path),work,proof)==
                C::ContactDetail::EPlanarProjectionProof::Proved&&proof.Providers.Num()==2);
        const TArray<int32> reversed{2,0};
        TestTrue(TEXT("reordered input cannot publish an indexed family certificate"),
            C::ContactDetail::ProjectedPlanarDomainsIndexed(surfaces,reversed,C::CenterDetail::Hull(path),work,proof)==
                C::ContactDetail::EPlanarProjectionProof::Arithmetic&&proof.Providers.IsEmpty());
    }
    TestTrue(TEXT("numeric context restored"),numeric.Restore.Restore());return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlanarFamilyDraftFixedWitness,
    "IAmSpeed.AnalyticWorld.TensorTime.PlanarFamilyDraft.FixedUVWitnessAndFallback",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FPlanarFamilyDraftFixedWitness::RunTest(const FString&)
{
    using namespace PlanarFamilyDraftTest;FNumeric numeric;
    if(!TestTrue(TEXT("numeric fixture"),numeric.Ready))return false;
    const uint32 controls=VectorGetControlRegister();
    for(uint32 flush:{0u,64u,32768u,32832u})
    {
        VectorSetControlRegister((controls&~uint32(0x8040))|flush);
        TArray<C::FIntervalTensorSurface> surfaces{Triangle(0)};
        C::FIntervalBezierCenterPath path;Path(FVector3d(0,0,2),FVector3d(0,0,-1),path);
        C::FDistanceBudget work{200000};TUniquePtr<C::FPlanarFamilyTimeShortcutPlan> plan;
        if(!TestTrue(TEXT("fixed lateral plane family"),C::FPlanarFamilyTimeShortcutPlan::Build(surfaces,path,work,plan)==C::EPlanarFamilyBuild::Ready&&plan))return false;
        C::FTensorDistanceBounds witness;const auto r2=A::Square(I::Exact(.25));
        const FVector3d center(.125,0,.125);
        if(!TestTrue(TEXT("real fixed parameter certifies strict inside"),
            plan->TryProvider(surfaces,0,C::Detail::Point(center),r2,work,witness)==C::EPlanarFamilyShortcut::Inside))return false;
        const FVector3d expected=OriginalTriangleAtFixedUV(surfaces[0],witness.SampleUV);
        TestTrue(TEXT("witness encloses original polynomial at fixed UV"),
            Contains(witness.SamplePoint,expected)&&witness.SampleUV.X>=0&&witness.SampleUV.X<=1&&witness.SampleUV.Y>=0&&witness.SampleUV.Y<=1);
        TestTrue(TEXT("fresh fixed UV tracks the current projected center and is evaluated on the original net"),
            FMath::Abs(expected.X-center.X)<1.e-10&&witness.UpperSquared<r2.Lower&&
            (center-expected).SquaredLength()<=witness.UpperSquared);
        TestTrue(TEXT("strict axial clearance precedes any UV minimum"),
            plan->TryProvider(surfaces,0,C::Detail::Point(FVector3d(0,0,2)),r2,work,witness)==C::EPlanarFamilyShortcut::Clear&&
            witness.Status==C::EDistanceBoundStatus::Unknown);

        // A wide original path must not force a permanently off-center witness.
        C::FIntervalBezierCenterPath lateral;Path(FVector3d(-2,0,2),FVector3d(2,0,-1),lateral);
        TUniquePtr<C::FPlanarFamilyTimeShortcutPlan> broad;
        if(!TestTrue(TEXT("finite family covers all lateral feet"),C::FPlanarFamilyTimeShortcutPlan::Build(surfaces,lateral,work,broad)==C::EPlanarFamilyBuild::Ready&&broad))return false;
        const auto offCenter=C::Detail::Point(FVector3d(1,0,.125));
        TestTrue(TEXT("current-hull inverse avoids the obsolete whole-path witness"),
            broad->TryProvider(surfaces,0,offCenter,r2,work,witness)==C::EPlanarFamilyShortcut::Inside&&
            witness.Status==C::EDistanceBoundStatus::Bounded);
        C::EPlanarFamilyShortcut failure=C::EPlanarFamilyShortcut::NotApplicable;
        const auto generic=C::TimeDetail::Evaluate(surfaces,offCenter,work,r2,0.,8192,true,broad.Get(),&failure);
        TestTrue(TEXT("fresh projection answers inside without a generic minimum"),
            generic.Known&&generic.AnyInside&&generic.SurfaceIndex==0&&failure==C::EPlanarFamilyShortcut::NotApplicable&&
            broad->GetCountersForInspection().GenericAttempts==0&&
            Contains(generic.Witness.SamplePoint,OriginalTriangleAtFixedUV(surfaces[0],generic.Witness.SampleUV)));
        const auto outside=C::Detail::Point(FVector3d(100,0,.125));
        TestTrue(TEXT("uncovered finite plane foot remains undecided"),
            broad->TryProvider(surfaces,0,outside,r2,work,witness)==C::EPlanarFamilyShortcut::Undecided&&
            witness.Status==C::EDistanceBoundStatus::Unknown);

    }
    TestTrue(TEXT("numeric context restored"),numeric.Restore.Restore());return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlanarFamilyDraftFirstContact,
    "IAmSpeed.AnalyticWorld.TensorTime.PlanarFamilyDraft.MixedFirstContact",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FPlanarFamilyDraftFirstContact::RunTest(const FString&)
{
    using namespace PlanarFamilyDraftTest;FNumeric numeric;
    if(!TestTrue(TEXT("numeric fixture"),numeric.Ready))return false;
    const uint32 controls=VectorGetControlRegister();
    for(uint32 flush:{0u,64u,32768u,32832u})
    {
        VectorSetControlRegister((controls&~uint32(0x8040))|flush);
        const auto search=[&](TArray<C::FIntervalTensorSurface> surfaces,FVector3d a,FVector3d b,double radius,double exact,double gap)
        {
            C::FIntervalBezierCenterPath path;Path(a,b,path);C::FDistanceBudget work{500000};
            TUniquePtr<C::FPlanarFamilyTimeShortcutPlan> plan;
            if(!TestTrue(TEXT("real finite plane family is eligible"),
                C::FPlanarFamilyTimeShortcutPlan::Build(surfaces,path,work,plan)==C::EPlanarFamilyBuild::Ready&&plan))return false;
            C::TimeDetail::FRetainedSphereTimeSearch time(surfaces,path,radius,work,0.,8192,nullptr,false,plan.Get());
            const auto result=time.Advance(gap);
            TestTrue(TEXT("unchanged global frontier encloses independent analytic first time"),
                result.Status==C::ETensorSphereTimeStatus::FirstContactBracket&&result.InitialSeparationProved&&
                result.LowerTime<=exact&&result.UpperTime>=exact&&result.UpperTime-result.LowerTime<=gap);
            TestTrue(TEXT("actual early shortcut was taken"),plan->GetCountersForInspection().Clear>0);
            if(surfaces.Num()==2&&surfaces[0].DegreeV==2)
                TestTrue(TEXT("curved provider still takes the original generic distance route"),
                    plan->GetCountersForInspection().GenericAttempts>0);
            return result.Status==C::ETensorSphereTimeStatus::FirstContactBracket;
        };
        // Curve's unique first contact: z=1+r, y=0 => t=3/8.
        search({Triangle(0),Curve()},FVector3d(0,0,2),FVector3d(0,0,0),.25,3./8.,1./65536.);
        search({Curve(),Triangle(0)},FVector3d(0,0,2),FVector3d(0,0,0),.25,3./8.,1./65536.);
        // Perpendicular competitors: z=1+r at 1/4, x=r at 1/2.
        search({Triangle(0,0),Triangle(1)},FVector3d(3,0,3),FVector3d(-1,0,-1),1.,1./4.,1./65536.);
        // Parallel heights are separate exact families, with no epsilon grouping.
        search({Triangle(0),Triangle(1)},FVector3d(0,0,3),FVector3d(0,0,-1),.25,7./16.,1./65536.);
        const double offset=1./1048576.;
        search({Triangle(1),Triangle(1+offset)},FVector3d(0,0,3),FVector3d(0,0,-1),.25,
            7./16.-1./4194304.,1./67108864.);
    }
    TestTrue(TEXT("numeric context restored"),numeric.Restore.Restore());return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlanarFamilyDraftTies,
    "IAmSpeed.AnalyticWorld.TensorContact.PlanarFamilyDraft.TiesKeepAllLeaves",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FPlanarFamilyDraftTies::RunTest(const FString&)
{
    using namespace PlanarFamilyDraftTest;FNumeric numeric;
    if(!TestTrue(TEXT("numeric fixture"),numeric.Ready))return false;
    C::PlanarFamilyTimeShortcutControl::FScopedTestEnable enabled(true);
    TArray<C::FIntervalTensorSurface> surfaces{Triangle(0),Triangle(0)};
    C::FIntervalBezierCenterPath path;Path(FVector3d(0,0,2),FVector3d(0,0,-1),path);
    C::FDistanceBudget work{1000000};int32 callbacks=0;bool both=false,originalOrder=false;
    const auto accept=[&](const C::FTensorSphereContactEnclosure& provisional)
    {
        ++callbacks;bool first=false,second=false;
        for(const auto& leaf:provisional.Leaves){first|=leaf.SurfaceIndex==0;second|=leaf.SurfaceIndex==1;}
        both=first&&second;
        originalOrder=!provisional.Leaves.IsEmpty()&&provisional.Leaves[0].SurfaceIndex==1&&provisional.Leaves.Last().SurfaceIndex==0;
        return both&&originalOrder?C::EContactEnclosureDecision::Accept:C::EContactEnclosureDecision::Refuse;
    };
    const C::FContactEnclosureFilter filter(accept);
    const auto contact=C::BoundTensorSphereFirstContact(surfaces,path,.25,work,1./65536.,0.,8192,.01,.01,24,&filter);
    TestTrue(TEXT("coplanar equal-time providers reach the original complete spatial callback"),
        contact.Status==C::ETensorSphereContactStatus::FirstContactEnclosed&&callbacks>0&&both&&originalOrder&&
        contact.LowerTime<=7./12.&&contact.UpperTime>=7./12.);

    // Equal-time perpendicular contacts have different points/normals.
    surfaces={Triangle(0,0),Triangle(0)};Path(FVector3d(2,0,2),FVector3d(0,0,0),path);
    C::FDistanceBudget ambiguous{1000000};
    const auto tie=C::BoundTensorSphereFirstContact(surfaces,path,.25,ambiguous,1./65536.,0.,8192,.01,.01);
    TestTrue(TEXT("perpendicular tie cannot become a chosen family hit"),
        tie.Status==C::ETensorSphereContactStatus::Unknown&&tie.Leaves.IsEmpty());
    TestTrue(TEXT("numeric context restored"),numeric.Restore.Restore());return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlanarFamilyDraftTerminal,
    "IAmSpeed.AnalyticWorld.TensorTime.PlanarFamilyDraft.TerminalWorkArithmetic",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FPlanarFamilyDraftTerminal::RunTest(const FString&)
{
    using namespace PlanarFamilyDraftTest;FNumeric numeric;
    if(!TestTrue(TEXT("numeric fixture"),numeric.Ready))return false;
    TArray<C::FIntervalTensorSurface> surfaces{Triangle(0)};
    C::FIntervalBezierCenterPath path;Path(FVector3d(0,0,2),FVector3d(0,0,-1),path);
    {
        C::FDistanceBudget work{65536};TUniquePtr<C::FPlanarFamilyTimeShortcutPlan> plan;
        if(!TestTrue(TEXT("work failure fixture builds"),C::FPlanarFamilyTimeShortcutPlan::Build(surfaces,path,work,plan)==C::EPlanarFamilyBuild::Ready&&plan))return false;
        work.ConsumptionFloor=work.Remaining-1;const uint64 before=work.Used;
        C::TimeDetail::FRetainedSphereTimeSearch time(surfaces,path,.25,work,0.,8192,nullptr,false,plan.Get());
        const auto stopped=time.Advance(1./65536.);
        TestTrue(TEXT("shortcut work failure terminates before a generic provider attempt"),
            stopped.Status==C::ETensorSphereTimeStatus::Unknown&&
            time.GetFailureForInspection()==C::TimeDetail::ETimeSearchFailure::PlaneShortcutWork&&
            work.Used==before+1&&work.Remaining==work.ConsumptionFloor&&plan->GetCountersForInspection().Work==1);
        const uint64 consumed=work.Used;
        TestTrue(TEXT("terminal search cannot restart or refill"),
            time.Advance(1./262144.).Status==C::ETensorSphereTimeStatus::Unknown&&work.Used==consumed);
    }
    {
        C::FDistanceBudget work{65536};TUniquePtr<C::FPlanarFamilyTimeShortcutPlan> plan;
        if(!TestTrue(TEXT("arithmetic failure fixture builds"),C::FPlanarFamilyTimeShortcutPlan::Build(surfaces,path,work,plan)==C::EPlanarFamilyBuild::Ready&&plan))return false;
        // Same numbers at a different original surface address are not the retained view.
        const auto copied=surfaces;const uint64 before=work.Used;
        C::TimeDetail::FRetainedSphereTimeSearch time(copied,path,.25,work,0.,8192,nullptr,false,plan.Get());
        const auto stopped=time.Advance(1./65536.);
        TestTrue(TEXT("identity/arithmetic refusal is terminal without generic fallback"),
            stopped.Status==C::ETensorSphereTimeStatus::Unknown&&
            time.GetFailureForInspection()==C::TimeDetail::ETimeSearchFailure::PlaneShortcutArithmetic&&
            work.Used==before+1&&plan->GetCountersForInspection().Arithmetic==1);
    }
    {
        C::FDistanceBudget empty{0};TUniquePtr<C::FPlanarFamilyTimeShortcutPlan> plan;
        TestTrue(TEXT("setup work failure publishes no partial family plan"),
            C::FPlanarFamilyTimeShortcutPlan::Build(surfaces,path,empty,plan)==C::EPlanarFamilyBuild::Work&&!plan&&empty.Used==0);
    }
    TestTrue(TEXT("numeric context restored"),numeric.Restore.Restore());return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlanarFamilyDraftSpatial,
    "IAmSpeed.AnalyticWorld.TensorContact.PlanarFamilyDraft.SpatialScatterAndFallback",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FPlanarFamilyDraftSpatial::RunTest(const FString&)
{
    using namespace PlanarFamilyDraftTest;FNumeric numeric;
    if(!TestTrue(TEXT("numeric fixture"),numeric.Ready))return false;
    const uint32 controls=VectorGetControlRegister();
    for(uint32 flush:{0u,64u,32768u,32832u})
    {
        VectorSetControlRegister((controls&~uint32(0x8040))|flush);
        TArray<C::FIntervalTensorSurface> surfaces{Triangle(0),Curve(),Triangle(0),Triangle(1),Triangle(0,0)};
        C::FIntervalBezierCenterPath path;Path(FVector3d(0,0,3),FVector3d(0,0,-1),path);
        C::FDistanceBudget work{200000};TUniquePtr<C::FPlanarFamilyTimeShortcutPlan> plan;
        if(!TestTrue(TEXT("same noncontiguous family plan"),C::FPlanarFamilyTimeShortcutPlan::Build(surfaces,path,work,plan)==C::EPlanarFamilyBuild::Ready&&plan))return false;
        auto center=C::Detail::Point(FVector3d(0,0,.25));center.Axis[2]={.24,.26,true};
        C::ContactDetail::FPlanarProjectionDomains whole;
        TestTrue(TEXT("original whole-union projection is inapplicable to the mixed union"),
            C::ContactDetail::ProjectedPlanarDomains(surfaces,center,work,whole)==C::ContactDetail::EPlanarProjectionProof::NotApplicable);
        TArray<C::FPlanarFamilySpatialProvider> domains;
        if(!TestTrue(TEXT("family projection succeeds at the common bracket center"),
            plan->ProjectSpatialFamilies(surfaces,center,work,domains)==C::EPlanarFamilySpatialProjection::Complete&&domains.Num()==surfaces.Num()))return false;
        TestTrue(TEXT("scatter preserves noncontiguous original indices and curve generic membership"),
            domains[0].Proved&&!domains[1].Proved&&domains[2].Proved&&
            domains[0].NormalAxis==2&&domains[2].NormalAxis==2&&
            domains[0].Domain.Bounded&&domains[2].Domain.Bounded);
        TestTrue(TEXT("height and axis keep their own foot, never another family's foot"),
            domains[0].Foot.Axis[2].Lower==0&&domains[3].Proved&&domains[3].Foot.Axis[2].Lower==1&&
            domains[4].Proved&&domains[4].NormalAxis==0&&domains[4].Foot.Axis[0].Lower==0);
        TestTrue(TEXT("valid NotApplicable projection leaves only generic provider entries"),
            plan->ProjectSpatialFamilies(surfaces,C::Detail::Point(FVector3d(100,100,.25)),work,domains)==
                C::EPlanarFamilySpatialProjection::Complete&&domains.Num()==surfaces.Num());
        bool any=false;for(const auto& domain:domains)any|=domain.Proved;
        TestTrue(TEXT("no partial local certificate survives the absence of a finite foot witness"),!any);
        const auto copied=surfaces;
        TestTrue(TEXT("wrong original view is terminal and clears the entire scatter"),
            plan->ProjectSpatialFamilies(copied,center,work,domains)==C::EPlanarFamilySpatialProjection::Arithmetic&&domains.IsEmpty());
        work.ConsumptionFloor=work.Remaining;
        TestTrue(TEXT("work failure publishes no partial scatter"),
            plan->ProjectSpatialFamilies(surfaces,center,work,domains)==C::EPlanarFamilySpatialProjection::Work&&domains.IsEmpty());
    }
    {
        C::PlanarFamilyTimeShortcutControl::FScopedTestEnable enabled(true);
        TArray<C::FIntervalTensorSurface> surfaces{Triangle(0),Curve()};
        C::FIntervalBezierCenterPath path;Path(FVector3d(0,0,2),FVector3d(0,0,0),path);
        C::FDistanceBudget work{1000000};
        const auto contact=C::BoundTensorSphereFirstContact(surfaces,path,.25,work,1./65536.,0.,8192,.01,.01);
        bool curve=false;for(const auto& leaf:contact.Leaves)curve|=leaf.SurfaceIndex==1;
        TestTrue(TEXT("family-local spatial pruning retains the genuinely earlier generic curve"),
            contact.Status==C::ETensorSphereContactStatus::FirstContactEnclosed&&curve&&
            contact.LowerTime<=3./8.&&contact.UpperTime>=3./8.);
    }
    TestTrue(TEXT("numeric context restored"),numeric.Restore.Restore());return !HasAnyErrors();
}


namespace CoordinateLineDraftTest
{
using namespace PlanarFamilyDraftTest;
C::FIntervalTensorSurface Line(bool reversed=false)
{
    // Exactly representable Bernstein coefficients of z(v)=5v/16+11v^5/16.
    // It is strongly nonlinear but its image is exactly the finite segment.
    constexpr double z[6]={0.,.0625,.125,.1875,.25,1.};
    C::FIntervalTensorSurface result;result.DegreeV=5;
    for(int32 i=0;i<6;++i)result.ControlPoints.Add(C::Detail::Point(FVector3d(0,4,z[reversed?5-i:i])));
    return result;
}
FVector3d At(double v,bool reversed=false)
{
    const double t=reversed?1-v:v;
    return FVector3d(0,4,.3125*t+.6875*t*t*t*t*t);
}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCoordinateLineFixedBoundsTest,
    "IAmSpeed.AnalyticWorld.TensorTime.CoordinateLine.FixedWitnessAndBounds",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FCoordinateLineFixedBoundsTest::RunTest(const FString&)
{
    using namespace CoordinateLineDraftTest;FNumeric numeric;
    if(!TestTrue(TEXT("numeric fixture"),numeric.Ready))return false;
    const uint32 controls=VectorGetControlRegister();
    for(uint32 flush:{0u,64u,32768u,32832u})for(bool reverse:{false,true})
    {
        VectorSetControlRegister((controls&~uint32(0x8040))|flush);
        TArray<C::FIntervalTensorSurface> surfaces{Line(reverse)};
        C::FIntervalBezierCenterPath path;Path(FVector3d(0,5,.5),FVector3d(0,3,.5),path);
        C::FDistanceBudget work{200000};TUniquePtr<C::FPlanarFamilyTimeShortcutPlan> plan;
        if(!TestTrue(TEXT("strict monotone coordinate-line plan"),
            C::FPlanarFamilyTimeShortcutPlan::Build(surfaces,path,work,plan,true)==C::EPlanarFamilyBuild::Ready&&plan))return false;
        const auto r2=A::Square(I::Exact(.25));C::FTensorDistanceBounds witness;
        const FVector3d center(0,4.125,.5);
        if(!TestTrue(TEXT("original nonlinear parameter supplies strict inside"),
            plan->TryProvider(surfaces,0,C::Detail::Point(center),r2,work,witness)==C::EPlanarFamilyShortcut::Inside))return false;
        const auto expected=At(witness.SampleUV.Y,reverse);
        TestTrue(TEXT("original polynomial is independently enclosed at the proposed UV"),
            witness.SampleUV.X==0&&witness.SampleUV.Y>=0&&witness.SampleUV.Y<=1&&
            Contains(witness.SamplePoint,expected)&&(center-expected).SquaredLength()<=witness.UpperSquared&&
            witness.LowerSquared<=.015625&&witness.UpperSquared<r2.Lower);
        TestTrue(TEXT("nonlinear inverse differs from the erroneous chord parameter"),
            FMath::Abs(witness.SampleUV.Y-.5)>.1);
        TestTrue(TEXT("finite line support proves a strict miss"),
            plan->TryProvider(surfaces,0,C::Detail::Point(FVector3d(0,5,.5)),r2,work,witness)==C::EPlanarFamilyShortcut::Clear&&
            witness.Status==C::EDistanceBoundStatus::Unknown);
        auto hull=C::Detail::Point(FVector3d(0,4.25,.5));hull.Axis[1]={4.1,4.4,true};
        TestTrue(TEXT("threshold-straddling box has a certified bounded result"),
            plan->TryProvider(surfaces,0,hull,r2,work,witness)==C::EPlanarFamilyShortcut::Bounded&&
            witness.Status==C::EDistanceBoundStatus::Bounded&&witness.LowerSquared<=r2.Upper&&witness.UpperSquared>=r2.Lower&&
            Contains(witness.SamplePoint,At(.5,reverse)));
        C::EPlanarFamilyShortcut failure=C::EPlanarFamilyShortcut::NotApplicable;
        const auto bound=C::TimeDetail::Evaluate(surfaces,hull,work,r2,0.,1,false,plan.Get(),&failure);
        TestTrue(TEXT("bounded shortcut does not launch a generic UV minimum"),
            bound.Known&&!bound.AllClear&&!bound.AnyInside&&failure==C::EPlanarFamilyShortcut::NotApplicable&&
            plan->GetCountersForInspection().GenericAttempts==0&&plan->GetCountersForInspection().Bounded>=2);
        TestTrue(TEXT("exact tangency never becomes strict inside or clear"),
            plan->TryProvider(surfaces,0,C::Detail::Point(FVector3d(0,4.25,.5)),r2,work,witness)==C::EPlanarFamilyShortcut::Bounded);
        TestTrue(TEXT("finite endpoint is not relabelled as interior"),
            plan->TryProvider(surfaces,0,C::Detail::Point(FVector3d(0,4.125,0)),r2,work,witness)==C::EPlanarFamilyShortcut::Undecided);
    }
    TestTrue(TEXT("numeric context restored"),numeric.Restore.Restore());return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCoordinateLineAdmissionTest,
    "IAmSpeed.AnalyticWorld.TensorTime.CoordinateLine.AdmissionAndLimits",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FCoordinateLineAdmissionTest::RunTest(const FString&)
{
    using namespace CoordinateLineDraftTest;FNumeric numeric;
    if(!TestTrue(TEXT("numeric fixture"),numeric.Ready))return false;
    C::FIntervalBezierCenterPath path;Path(FVector3d(0,5,.5),FVector3d(0,3,.5),path);
    TArray<C::FIntervalTensorSurface> surfaces{Line()};
    C::FDistanceBudget work{200000};TUniquePtr<C::FPlanarFamilyTimeShortcutPlan> plan;
    TestTrue(TEXT("old explicit Build API remains opt-in"),
        C::FPlanarFamilyTimeShortcutPlan::Build(surfaces,path,work,plan)==C::EPlanarFamilyBuild::NotApplicable&&!plan);
    const auto rejects=[&](C::FIntervalTensorSurface surface)
    {
        TArray<C::FIntervalTensorSurface> input{MoveTemp(surface)};
        C::FDistanceBudget budget{10000};TUniquePtr<C::FPlanarFamilyTimeShortcutPlan> local;
        return C::FPlanarFamilyTimeShortcutPlan::Build(input,path,budget,local,true)==C::EPlanarFamilyBuild::NotApplicable&&!local;
    };
    auto zeroDerivative=Line();zeroDerivative.ControlPoints[1].Axis[2]=zeroDerivative.ControlPoints[0].Axis[2];
    TestTrue(TEXT("non-strict derivative remains generic"),rejects(zeroDerivative));
    auto bent=Line();bent.ControlPoints[1].Axis[1]=I::Exact(4.125);
    TestTrue(TEXT("a genuinely nonrectilinear curve remains generic"),rejects(bent));
    auto independent=Line();for(auto& p:independent.ControlPoints)p.Axis[1]={3.99,4.01,true};
    TestTrue(TEXT("identical non-singleton normal boxes do not prove a constant coordinate"),rejects(independent));
    if(!TestTrue(TEXT("valid plan for failure checks"),
        C::FPlanarFamilyTimeShortcutPlan::Build(surfaces,path,work,plan,true)==C::EPlanarFamilyBuild::Ready&&plan))return false;
    const auto r2=A::Square(I::Exact(.25));C::FTensorDistanceBounds witness;
    C::FDistanceBudget foreign{200000};
    TestTrue(TEXT("foreign account cannot consume a line proof"),
        plan->TryProvider(surfaces,0,C::Detail::Point(FVector3d(0,4.125,.5)),r2,foreign,witness)==C::EPlanarFamilyShortcut::Arithmetic);
    const uint64 before=work.Used;work.ConsumptionFloor=work.Remaining-8;
    TestTrue(TEXT("prepaid line attempt refuses before a partial debit"),
        plan->TryProvider(surfaces,0,C::Detail::Point(FVector3d(0,4.125,.5)),r2,work,witness)==C::EPlanarFamilyShortcut::Work&&
        work.Used==before&&work.Remaining-work.ConsumptionFloor==8&&witness.Status==C::EDistanceBoundStatus::Unknown);
    TestTrue(TEXT("numeric context restored"),numeric.Restore.Restore());return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCoordinateLineFirstContactTest,
    "IAmSpeed.AnalyticWorld.TensorTime.CoordinateLine.FirstContactAndOriginalLeaves",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FCoordinateLineFirstContactTest::RunTest(const FString&)
{
    using namespace CoordinateLineDraftTest;FNumeric numeric;
    if(!TestTrue(TEXT("numeric fixture"),numeric.Ready))return false;
    const uint32 controls=VectorGetControlRegister();
    for(uint32 flush:{0u,64u,32768u,32832u})
    {
        VectorSetControlRegister((controls&~uint32(0x8040))|flush);
        TArray<C::FIntervalTensorSurface> surfaces{Line()};
        C::FIntervalBezierCenterPath path;Path(FVector3d(0,5,.5),FVector3d(0,3,.5),path);
        C::FDistanceBudget work{500000};TUniquePtr<C::FPlanarFamilyTimeShortcutPlan> plan;
        if(!TestTrue(TEXT("line plan"),C::FPlanarFamilyTimeShortcutPlan::Build(surfaces,path,work,plan,true)==C::EPlanarFamilyBuild::Ready&&plan))return false;
        C::TimeDetail::FRetainedSphereTimeSearch search(surfaces,path,.25,work,0.,8192,nullptr,false,plan.Get());
        const auto first=search.Advance(1./65536.);
        TestTrue(TEXT("same chronological search encloses exact first time"),
            first.Status==C::ETensorSphereTimeStatus::FirstContactBracket&&first.InitialSeparationProved&&
            first.LowerTime<=.375&&first.UpperTime>=.375&&first.UpperTime-first.LowerTime<=1./65536.);
        const auto second=search.Advance(1./262144.);
        TestTrue(TEXT("retained chronological frontier narrows without generic UV minima"),
            second.Status==C::ETensorSphereTimeStatus::FirstContactBracket&&second.LowerTime>=first.LowerTime&&second.UpperTime<=first.UpperTime&&
            second.LowerTime<=.375&&second.UpperTime>=.375&&plan->GetCountersForInspection().GenericAttempts==0&&
            plan->GetCountersForInspection().Bounded>0&&search.GetSpatialCountersForInspection().NewRoots==0&&
            work.Used+uint64(work.Remaining)==500000);
        C::PlanarFamilyTimeShortcutControl::FScopedTestEnable enabled(true);
        C::FDistanceBudget contactWork{500000};
        const auto contact=C::BoundTensorSphereFirstContact(surfaces,path,.25,contactWork,1./65536.,0.,8192,.001,.001);
        TestTrue(TEXT("unchanged spatial stage still returns original curve leaves"),
            contact.Status==C::ETensorSphereContactStatus::FirstContactEnclosed&&!contact.Leaves.IsEmpty()&&
            contact.LowerTime<=.375&&contact.UpperTime>=.375);
        for(const auto& leaf:contact.Leaves)
            TestTrue(TEXT("leaf parameter and point belong to the original quintic"),leaf.SurfaceIndex==0&&
                Contains(leaf.FixedParameterPoint,At(leaf.FixedParameter.Y))&&leaf.MinV<=leaf.FixedParameter.Y&&leaf.MaxV>=leaf.FixedParameter.Y);
    }
    TestTrue(TEXT("numeric context restored"),numeric.Restore.Restore());return !HasAnyErrors();
}


IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlanarFamilyExactCenterMemoTest,
    "IAmSpeed.AnalyticWorld.TensorTime.PlanarFamilyDraft.ExactCenterMemo",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FPlanarFamilyExactCenterMemoTest::RunTest(const FString&)
{
    using namespace PlanarFamilyDraftTest;FNumeric numeric;
    if(!TestTrue(TEXT("numeric fixture"),numeric.Ready))return false;
    const uint32 controls=VectorGetControlRegister();
    const auto sameWitness=[](const C::FTensorDistanceBounds& a,const C::FTensorDistanceBounds& b)
    {
        if(a.Status!=b.Status||A::Bits(a.LowerSquared)!=A::Bits(b.LowerSquared)||
            A::Bits(a.UpperSquared)!=A::Bits(b.UpperSquared)||A::Bits(a.SampleUV.X)!=A::Bits(b.SampleUV.X)||
            A::Bits(a.SampleUV.Y)!=A::Bits(b.SampleUV.Y))return false;
        for(int32 d=0;d<3;++d)if(a.SamplePoint.Axis[d].Known!=b.SamplePoint.Axis[d].Known||
            A::Bits(a.SamplePoint.Axis[d].Lower)!=A::Bits(b.SamplePoint.Axis[d].Lower)||
            A::Bits(a.SamplePoint.Axis[d].Upper)!=A::Bits(b.SamplePoint.Axis[d].Upper))return false;
        return true;
    };
    for(uint32 flush:{0u,64u,32768u,32832u})
    {
        VectorSetControlRegister((controls&~uint32(0x8040))|flush);
        TArray<C::FIntervalTensorSurface> surfaces{Triangle(0),Curve(),Triangle(0)};
        C::FIntervalBezierCenterPath path;Path(FVector3d(0,0,2),FVector3d(0,0,-1),path);
        C::FDistanceBudget work{200000};TUniquePtr<C::FPlanarFamilyTimeShortcutPlan> plan;
        if(!TestTrue(TEXT("noncontiguous identical providers form a family"),
            C::FPlanarFamilyTimeShortcutPlan::Build(surfaces,path,work,plan)==C::EPlanarFamilyBuild::Ready&&plan))return false;
        const auto r2=A::Square(I::Exact(.25));
        auto center=C::Detail::Point(FVector3d(0,0,.125));
        C::FTensorDistanceBounds first,second;
        uint64 before=work.Used;
        if(!TestTrue(TEXT("first provider proves current finite foot"),
            plan->TryProvider(surfaces,0,center,r2,work,first)==C::EPlanarFamilyShortcut::Inside))return false;
        const uint64 missCost=work.Used-before;before=work.Used;
        if(!TestTrue(TEXT("second provider reuses the same-family projection"),
            plan->TryProvider(surfaces,2,center,r2,work,second)==C::EPlanarFamilyShortcut::Inside))return false;
        const uint64 hitCost=work.Used-before;
        TestTrue(TEXT("memo avoids projection work and preserves every output bit"),
            hitCost<missCost&&sameWitness(first,second)&&Contains(second.SamplePoint,OriginalTriangleAtFixedUV(surfaces[2],second.SampleUV)));

        const auto oldUV=first.SampleUV;center=C::Detail::Point(FVector3d(1,0,.125));before=work.Used;
        if(!TestTrue(TEXT("different current center recomputes the inverse"),
            plan->TryProvider(surfaces,0,center,r2,work,first)==C::EPlanarFamilyShortcut::Inside))return false;
        TestTrue(TEXT("different center pays a fresh projection and moves its UV"),
            work.Used-before>hitCost&&first.SampleUV.Y!=oldUV.Y&&
            Contains(first.SamplePoint,OriginalTriangleAtFixedUV(surfaces[0],first.SampleUV)));
        before=work.Used;
        TestTrue(TEXT("second provider at changed center reuses only that new inverse"),
            plan->TryProvider(surfaces,2,center,r2,work,second)==C::EPlanarFamilyShortcut::Inside&&
            work.Used-before==hitCost&&sameWitness(first,second));

        // A one-bit change of just one endpoint invalidates the literal key.
        center.Axis[0].Upper=A::FromBits(A::Bits(center.Axis[0].Upper)+1);before=work.Used;
        TestTrue(TEXT("even a one-ULP upper-endpoint change is a memo miss"),
            plan->TryProvider(surfaces,0,center,r2,work,first)==C::EPlanarFamilyShortcut::Inside&&work.Used-before>hitCost);
        before=work.Used;
        TestTrue(TEXT("the exact interval center then hits"),
            plan->TryProvider(surfaces,2,center,r2,work,second)==C::EPlanarFamilyShortcut::Inside&&
            work.Used-before==hitCost&&sameWitness(first,second));

        center=C::Detail::Point(FVector3d(100,0,.125));before=work.Used;
        TestTrue(TEXT("outside finite union remains undecided"),
            plan->TryProvider(surfaces,0,center,r2,work,first)==C::EPlanarFamilyShortcut::Undecided&&
            first.Status==C::EDistanceBoundStatus::Unknown);
        const uint64 negativeMiss=work.Used-before;before=work.Used;
        TestTrue(TEXT("completed negative projection is reusable without a witness"),
            plan->TryProvider(surfaces,2,center,r2,work,second)==C::EPlanarFamilyShortcut::Undecided&&
            work.Used-before<negativeMiss&&second.Status==C::EDistanceBoundStatus::Unknown);
        center=C::Detail::Point(FVector3d(0,0,.125));before=work.Used;
        TestTrue(TEXT("negative memo never survives a different center"),
            plan->TryProvider(surfaces,0,center,r2,work,first)==C::EPlanarFamilyShortcut::Inside&&work.Used-before>hitCost);
        C::FDistanceBudget foreign{200000};const uint64 spent=work.Used;
        TestTrue(TEXT("foreign account is refused before a warm memo"),
            plan->TryProvider(surfaces,2,center,r2,foreign,second)==C::EPlanarFamilyShortcut::Arithmetic&&
            foreign.Used==0&&work.Used==spent);
        const TArray<C::FIntervalTensorSurface> copied=surfaces;
        TestTrue(TEXT("copied source view is refused before a warm memo"),
            plan->TryProvider(copied,2,center,r2,work,second)==C::EPlanarFamilyShortcut::Arithmetic&&work.Used==spent);
        TestTrue(TEXT("all real work stays in the original account"),work.Used+uint64(work.Remaining)==200000);
    }
    TestTrue(TEXT("numeric context restored"),numeric.Restore.Restore());return !HasAnyErrors();
}

#endif

// DRAFT ONLY: compile after integrating the helper into TensorDistanceBounds.h.
#if WITH_DEV_AUTOMATION_TESTS
#include "Misc/AutomationTest.h"
#include "TensorDistanceBounds.h"
#include "IAmSpeed/World/Simulation/FrameFloatingPointContext.h"
namespace StrongConvexCurveDraftTest
{
namespace C=Speed::Analytic::Certified;
namespace Q=C::StrongConvexCurveMinimum;
namespace A=::AnalyticCertified;
using I=A::Interval;
struct FNumeric
{
    Speed::FFrameFloatingPointContext Original=Speed::CaptureFrameFloatingPointContext();
    Speed::FScopedFrameFloatingPointContext Restore{Original};
    std::fenv_t Held;
    bool Ready=Original.bValid&&Restore.IsApplied()&&std::feholdexcept(&Held)==0&&std::fesetround(FE_TONEAREST)==0;
};
C::FIntervalTensorSurface Line(double epsilon=0)
{
    C::FIntervalTensorSurface s;s.DegreeU=0;s.DegreeV=5;
    for(int32 i=0;i<6;++i)
    {auto p=C::Detail::Point(FVector3d(i,0,0));p.Axis[1]={-epsilon,epsilon,true};s.ControlPoints.Add(p);}
    return s;
}
C::FIntervalTensorSurface Parabola()
{
    // Exact integer Bernstein coefficients: C(t)=(20t,20t^2,0).
    C::FIntervalTensorSurface s;s.DegreeU=0;s.DegreeV=5;
    for(int32 i=0;i<6;++i)s.ControlPoints.Add(C::Detail::Point(FVector3d(4*i,i*(i-1),0)));
    return s;
}
bool Encloses(const C::FTensorDistanceBounds& r,double squared)
{return r.Status==C::EDistanceBoundStatus::Bounded&&r.LowerSquared<=squared&&r.UpperSquared>=squared;}
bool Contains(const C::FIntervalPoint& p,FVector3d value)
{
    for(int32 d=0;d<3;++d)if(!A::Valid(p.Axis[d])||p.Axis[d].Lower>value[d]||p.Axis[d].Upper<value[d])return false;
    return true;
}
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FStrongConvexCurveDraft,
    "IAmSpeed.AnalyticWorld.TensorDistance.StrongConvexCurveDraft.Discriminants",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FStrongConvexCurveDraft::RunTest(const FString&)
{
    using namespace StrongConvexCurveDraftTest;
    FNumeric numeric;if(!TestTrue(TEXT("numeric fixture"),numeric.Ready))return false;
    const uint32 controls=VectorGetControlRegister();
    for(uint32 flush:{0u,64u,32768u,32832u})
    {
        VectorSetControlRegister((controls&~uint32(0x8040))|flush);
        auto curve=Parabola();C::FDistanceBudget work{2000};C::FTensorDistanceBounds result;Q::FInspection info;
        auto status=Q::TryBound(curve,C::Detail::Point(FVector3d(10,5,3)),work,128,result,&info);
        TestTrue(TEXT("curved interior minimum is admitted"),status==Q::EResult::Bounded&&info.CurvatureLower>0);
        TestTrue(TEXT("known original parabola minimum squared=9 enclosed"),Encloses(result,9));
        TestTrue(TEXT("strict interior minimizer retained"),info.ParameterLower<=.5&&info.ParameterUpper>=.5&&
            info.ParameterUpper-info.ParameterLower<.01&&!info.PossibleStart&&!info.PossibleEnd);
        const double v=result.SampleUV.Y;
        TestTrue(TEXT("upper witness is an original fixed UV point"),result.SampleUV.X==0&&v>=0&&v<=1&&
            Contains(result.SamplePoint,FVector3d(20*v,20*v*v,0)));
        TestTrue(TEXT("bounded constant work; no fabricated convergence"),work.Used>0&&work.Used<=24&&
            result.Status==C::EDistanceBoundStatus::Bounded&&info.NewtonSteps<=8);

        for(const auto& p:{FVector3d(-2,0,0),FVector3d(7,0,0)})
        {
            auto line=Line();C::FDistanceBudget b{128};
            status=Q::TryBound(line,C::Detail::Point(p),b,128,result,&info);
            TestTrue(TEXT("original endpoint min survives empty interior-root Newton set"),
                status==Q::EResult::Bounded&&Encloses(result,4));
            TestTrue(TEXT("right endpoint provenance"),p.X<0?
                info.PossibleStart&&info.ParameterLower==0:info.PossibleEnd&&info.ParameterUpper==1);
        }

        // Independent CP intervals and centers crossing an original endpoint.
        // Constant translations y=+-epsilon are two genuine represented curves,
        // not a correlated replacement for the entire interval family.
        const double epsilon=1e-8;auto uncertain=Line(epsilon);
        auto centers=C::Detail::Point(FVector3d(0,2,0));centers.Axis[0]={-.5,.5,true};
        C::FDistanceBudget uncertainWork{128};
        status=Q::TryBound(uncertain,centers,uncertainWork,128,result,&info);
        TestTrue(TEXT("uncertain coefficient and center boxes remain admitted"),status==Q::EResult::Bounded);
        TestTrue(TEXT("possible endpoint is unioned with possible interior minima"),
            info.PossibleStart&&info.ParameterLower==0&&info.ParameterUpper>=.1);
        for(double x:{-.5,0.,.5})for(double y:{-epsilon,epsilon})
        {
            const double exact=(2-y)*(2-y)+(x<0?x*x:0.);
            TestTrue(TEXT("all translated-line and center realizations enclosed"),Encloses(result,exact));
        }
        TestTrue(TEXT("original independent point enclosure retained"),
            result.SamplePoint.Axis[1].Lower<=-epsilon&&result.SamplePoint.Axis[1].Upper>=epsilon);

        C::FDistanceBudget nonconvex{128};
        status=Q::TryBound(curve,C::Detail::Point(FVector3d(10,20,0)),nonconvex,128,result,&info);
        TestTrue(TEXT("unproved strong convexity leaves existing minimum applicable"),
            status==Q::EResult::NotApplicable&&result.Status==C::EDistanceBoundStatus::Unknown&&nonconvex.Used>0);
        C::FDistanceBudget empty{0};
        status=Q::TryBound(curve,C::Detail::Point(FVector3d(10,5,3)),empty,128,result);
        TestTrue(TEXT("global exhaustion is terminal Work"),status==Q::EResult::Work&&empty.Used==0);
        C::FDistanceBudget local{128};
        status=Q::TryBound(curve,C::Detail::Point(FVector3d(10,5,3)),local,1,result);
        TestTrue(TEXT("local stop never refunds or overconsumes"),status==Q::EResult::LocalLimit&&local.Used==1&&local.Remaining==127);
        C::FDistanceBudget floor{128};floor.ConsumptionFloor=128;
        status=Q::TryBound(curve,C::Detail::Point(FVector3d(10,5,3)),floor,128,result);
        TestTrue(TEXT("reservation floor honored"),status==Q::EResult::Work&&floor.Used==0);
    }
    return true;
}
#endif


#if WITH_DEV_AUTOMATION_TESTS
#include "Misc/ScopedCVar.h"
namespace Speed::Analytic::Certified::StrongConvexCurveMinimum
{IAMSPEED_API uint64 ObservationCallsForTesting();}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FStrongConvexCurveOneShotRouteTest,
    "IAmSpeed.AnalyticWorld.TensorDistance.StrongConvexCurveDraft.OneShotRoute",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FStrongConvexCurveOneShotRouteTest::RunTest(const FString&)
{
    using namespace StrongConvexCurveDraftTest;
    FNumeric numeric;if(!TestTrue(TEXT("numeric fixture"),numeric.Ready))return false;
    // Engine temporary CVar layers restore the prior value and its provenance.
    FScopedCVar<int32> enabled(TEXT("p.IAmSpeed.AnalyticWorld.PlanarFamilyTimeShortcut"),1,
        FName(TEXT("StrongConvexOneShotRoute")));
    FScopedCVar<int32> profile(TEXT("p.IAmSpeed.AnalyticWorld.PhaseTiming"),1,
        FName(TEXT("StrongConvexOneShotRouteProfile")));
    if(!TestTrue(TEXT("actual module runtime gate enabled"),C::PlanarFamilyTimeShortcutControl::RuntimeEnabled()))return false;
    const uint32 controls=VectorGetControlRegister();
    for(uint32 flush:{0u,64u,32768u,32832u})
    {
        VectorSetControlRegister((controls&~uint32(0x8040))|flush);
        const auto curve=Parabola();const auto center=C::Detail::Point(FVector3d(9,6,3));
        // C(.5)=(10,5,0), C'(.5)=(20,20,0), so the true minimum is11.
        // The root Cartesian hull gives only9; threshold10.5 is unresolved.
        const I threshold=I::Exact(10.5);C::FDistanceBudget work{2048};
        const uint64 before=Q::ObservationCallsForTesting();
        const auto result=C::BoundTensorDistanceSquaredIntervalCenter(curve,center,work,0.,128,&threshold);
        TestTrue(TEXT("one-shot actually invokes the exported strong-convex helper"),
            Q::ObservationCallsForTesting()==before+1);
        TestTrue(TEXT("one-shot proves original curved separation without changing threshold"),
            result.Status!=C::EDistanceBoundStatus::Unknown&&result.LowerSquared>threshold.Upper&&
            result.LowerSquared<=11.&&result.UpperSquared>=11.);
        const double u=result.SampleUV.X,v=result.SampleUV.Y;
        // DegreeU=0 makes every U in [0,1] the same original curve. The
        // existing root-midpoint witness U=.5 remains valid when the helper
        // improves only the lower bound, not the strict best upper bound.
        TestTrue(TEXT("one-shot still carries original UV witness"),
            u>=0&&u<=1&&v>=0&&v<=1&&Contains(result.SamplePoint,FVector3d(20*v,20*v*v,0)));
        TestTrue(TEXT("one-shot keeps local and global accounts"),work.Used<=128&&work.Used+uint64(work.Remaining)==2048);

        C::FDistanceBudget coarseWork{2048};const uint64 coarseBefore=Q::ObservationCallsForTesting();
        const auto coarse=C::BoundTensorDistanceSquaredIntervalCenter(curve,center,coarseWork,0.,1,&threshold);
        TestTrue(TEXT("local1 coarse route never attempts the optional helper"),
            Q::ObservationCallsForTesting()==coarseBefore&&coarseWork.Used==1&&
            coarse.Status!=C::EDistanceBoundStatus::Unknown&&coarse.LowerSquared<=11.&&coarse.UpperSquared>=11.);

        // Compile/link the default arguments on the exported declaration too.
        C::FDistanceBudget defaultsWork{2048};
        const auto defaults=C::BoundTensorDistanceSquaredIntervalCenter(curve,center,defaultsWork,1e9);
        TestTrue(TEXT("four-argument convenience call retains defaults"),
            defaults.Status!=C::EDistanceBoundStatus::Unknown&&defaults.LowerSquared<=11.&&defaults.UpperSquared>=11.);
    }
    TestTrue(TEXT("numeric context restored"),numeric.Restore.Restore());return !HasAnyErrors();
}
#endif
