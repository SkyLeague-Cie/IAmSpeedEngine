#include "PreparedSphereGeneralFirstArrivalRs.h"
#include "PreparedSweepEvidence.h"
#include "SignedAffineSphereNativeClosure.h"
#include "ReferenceSphereC2ContactPlan.h"
#if !UE_BUILD_SHIPPING
#include "ReferenceSphereC2FixedRoundedPrefix.h"
#include "IAmSpeed/World/Analytic/CanonicalAxialPlaneBounds.h"
#endif
#include "ReferenceRepeatedSphereFloorPlan.h"
#include "HAL/IConsoleManager.h"
#include "HAL/PlatformTime.h"
#include "IAmSpeed/World/Analytic/FixedShapeQueryBounds.h"
namespace Speed::Analytic::Prepared
{
namespace
{
TAutoConsoleVariable<int32> CSignedC2SphereTime(TEXT("p.IAmSpeed.AnalyticWorld.SignedC2SphereTime"),0,
    TEXT("Default OFF. Prove an original C2 sphere time prefix and signed endpoint on the complete retained union."));
#if !UE_BUILD_SHIPPING
TAutoConsoleVariable<int32> CSignedC2VariableSphereTime(TEXT("p.IAmSpeed.AnalyticWorld.SignedC2VariableSphereTime"),0,
    TEXT("Default OFF. Prove only strictly clear reference time cells in the authenticated rising sphere corridor."));
#endif
TAutoConsoleVariable<int32> CSignedC2InteriorSphereContraction(TEXT("p.IAmSpeed.AnalyticWorld.SignedC2InteriorSphereContraction"),0,
    TEXT("Default OFF. Bound interior KKT points of the admitted signed C2 sphere spatial leaf search."));
#if !UE_BUILD_SHIPPING
TAutoConsoleVariable<int32> CSignedC2VariableRoundedPrefix(TEXT("p.IAmSpeed.AnalyticWorld.SignedC2VariableRoundedPrefix"),0,
    TEXT("Default OFF. Prove the original binary32-seconds variable-height Rs prefix on the complete retained union."));
TAutoConsoleVariable<int32> CSignedC2VariableFreshPoint(TEXT("p.IAmSpeed.AnalyticWorld.SignedC2VariableFreshPoint"),0,
    TEXT("Default OFF. Independently prove the actual variable-height consumed singleton with its original Rs/Rc geometry."));
TAutoConsoleVariable<int32> CSignedC2FixedRoundedPrefix(TEXT("p.IAmSpeed.AnalyticWorld.SignedC2FixedRoundedPrefix"),0,
    TEXT("Default OFF. Prove the exact signed C2 fixed rounded prefix independently from reference time."));
TAutoConsoleVariable<int32> CSignedC2FixedFreshPoint(TEXT("p.IAmSpeed.AnalyticWorld.SignedC2FixedFreshPoint"),0,
    TEXT("Default OFF. Independently prove the fresh native singleton on the admitted signed fixed chart."));
#endif
TAutoConsoleVariable<int32> CSignedC2SphereFreshPoint(TEXT("p.IAmSpeed.AnalyticWorld.SignedC2SphereFreshPoint"),0,
    TEXT("Default OFF. Prove the mandatory fresh actual rounded singleton with the same C2 partition."));
TAutoConsoleVariable<int32> CSignedAffineSphereResidual(TEXT("p.IAmSpeed.AnalyticWorld.SignedAffineSphereResidual"),0,
    TEXT("Use a signed reference time proof on authenticated affine sphere candidates."));
TAutoConsoleVariable<int32> CSignedAffineSphereRoundedClosure(TEXT("p.IAmSpeed.AnalyticWorld.SignedAffineSphereRoundedClosure"),0,
    TEXT("Close the actual rounded sphere prefix and fresh point with authenticated affine signed separation."));
TAutoConsoleVariable<int32> CSignedAffineReferenceTimeCorrelation(TEXT("p.IAmSpeed.AnalyticWorld.SignedAffineReferenceTimeCorrelation"),0,
    TEXT("Try a reusable signed Bernstein separation on authenticated quadratic reference time cells. Default OFF."));
TAutoConsoleVariable<int32> CSignedAffineNativeWholeRcEarly(TEXT("p.IAmSpeed.AnalyticWorld.SignedAffineNativeWholeRcEarly"),0,
    TEXT("Try full native Rc separation before the reference time search. Default OFF."));
TAutoConsoleVariable<int32> CSignedSphereSpatialWidthRetry(TEXT("p.IAmSpeed.AnalyticWorld.SignedSphereSpatialWidthRetry"),0,
    TEXT("Try one tighter spatial subdivision on the same certified signed Sphere contact time. Default OFF."));

#if !UE_BUILD_SHIPPING
TAutoConsoleVariable<int32> CSignedSphere632799AffinePreclear(
    TEXT("p.IAmSpeed.AnalyticWorld.SignedSphere632799AffinePreclear"),0,
    TEXT("Default OFF. Try one paid all-role affine Rs prefix on the three exact P204 captures."));
#endif
#if !UE_BUILD_SHIPPING
TAutoConsoleVariable<int32> CNativeSphere799TimeMidpoint(
    TEXT("p.IAmSpeed.AnalyticWorld.NativeSphere799TimeMidpoint"),0,
    TEXT("Default OFF. Bounded elapsed-time midpoint proposal for the exact private 799 signed prefix."));
TAutoConsoleVariable<int32> CNativeSphere799ReferenceResidualTiming(
    TEXT("p.IAmSpeed.AnalyticWorld.NativeSphere799ReferenceResidualTiming"),0,
    TEXT("Default OFF. Passive timing for the single exact P202W1 case 799 General request."));
#endif
TAutoConsoleVariable<int32> CSignedSphereWholeGeometryHullMemo(TEXT("p.IAmSpeed.AnalyticWorld.SignedSphereWholeGeometryHullMemo"),0,
    TEXT("Default OFF. Reuse original immutable tensor hulls only in signed retained whole time cells."));
TAutoConsoleVariable<int32> CSignedSphereTimeAttribution(TEXT("p.IAmSpeed.AnalyticWorld.SignedSphereTimeAttribution"),0,
    TEXT("Default OFF. Attribute bounded signed C2 time kernel work without influencing the solver."));
TAutoConsoleVariable<int32> CSignedSphereRetainedEndpointReuse(TEXT("p.IAmSpeed.AnalyticWorld.SignedSphereRetainedEndpointReuse"),0,
    TEXT("Reuse a same-time authenticated signed Sphere inside endpoint on retained right cells. Default OFF."));

TAutoConsoleVariable<int32> CSignedAffineNativeWholeRcWallSlices(TEXT("p.IAmSpeed.AnalyticWorld.SignedAffineNativeWholeRcWallSlices"),0,
    TEXT("Prove full native/reference/literal Rc clearance on two exact wall-oriented time slices. Default OFF."));

TAutoConsoleVariable<int32> CSignedAffineNativeWholeRcSeparatePaths(
    TEXT("p.IAmSpeed.AnalyticWorld.SignedAffineNativeWholeRcSeparatePaths"),0,
    TEXT("After reference no-contact, prove native and literal Rc clearance separately. Default OFF."));

TAutoConsoleVariable<int32> CSignedAffineSphereRoundedTailSingleton(TEXT("p.IAmSpeed.AnalyticWorld.SignedAffineSphereRoundedTailSingleton"),0,
    TEXT("Prove the last binary32 integration time as an exact singleton after an indeterminate signed prefix. Default OFF."));

TAutoConsoleVariable<int32> CSignedAffineSphereRoundedVerticalOrigin(TEXT("p.IAmSpeed.AnalyticWorld.SignedAffineSphereRoundedVerticalOrigin"),0,
    TEXT("Use a common local vertical origin for the signed rounded sphere prefix. Default OFF."));

bool ContainsRoundedCenterHull(const C::FReferenceCandidateSet& partition,const C::FIntervalPoint& hull)
{
    const auto& path=partition.GetPathForFixedSource();
    if(path.Degree!=0||path.ControlPoints.Num()!=1||!C::Detail::ValidPoint(path.ControlPoints[0])||
        !C::Detail::ValidPoint(hull))return false;
    for(int32 axis=0;axis<3;++axis)
        if(hull.Axis[axis].Lower<path.ControlPoints[0].Axis[axis].Lower||
            hull.Axis[axis].Upper>path.ControlPoints[0].Axis[axis].Upper)return false;
    return true;
}


#if !UE_BUILD_SHIPPING
TAutoConsoleVariable<int32> CNativeSphereDiagonal698Prefix(TEXT("p.IAmSpeed.AnalyticWorld.NativeSphereDiagonal698Prefix"),0,
    TEXT("Private exact native Sphere diagonal prefix prototype. Default OFF."));
TAutoConsoleVariable<int32> CNativeSphereDiagonal698SearchTiming(
    TEXT("p.IAmSpeed.AnalyticWorld.NativeSphereDiagonal698SearchTiming"),0,
    TEXT("Time only the authenticated 698 diagonal reference and prefix on frame 394. Default OFF."));

bool IsDiagonal698Step(const PreparedExecution::FNativeSolidStaticValues* native,
    const FStaticSphereInitialValues& values,float horizon)
{
    return CNativeSphereDiagonal698Prefix.GetValueOnAnyThread()==1&&native&&
        native->Shape==PreparedExecution::ENativeSolidQueryShape::Sphere&&
        native->FrameClock.CanonicalFrame==394&&native->FrameClock.OwnerLocalFrame==395&&
        values.State.Location==FVector3d(-4446.6912493009977,-3446.6912493010018,1677.0276423463501)&&
        values.State.Velocity==FVector3d(-1341.1099999999999,-1341.1099999999999,-665.86000000000001)&&
        values.State.Acceleration==FVector3d(41.175230996146318,41.175230996146318,-629.55653204351108)&&
        horizon==0.0033333334140479565f&&values.SupportRadius==93.149999976158142&&
        values.QueryRadius==93.150100708007812f;
}

// Bounds all original U coefficients in every V coefficient of (X+Y,Z).
// Cauchy on (n,n,m) is sufficient for Rs separation, independently of the
// omitted tangential component. No exact extrusion or new hit is inferred.
bool ProveDiagonal698Panel(const C::FIntervalTensorSurface& panel,bool edgeAxis,
    const AnalyticCertified::Interval (&centerS)[3],const AnalyticCertified::Interval (&centerZ)[3],
    AnalyticCertified::Interval radiusSquared,C::FDistanceBudget& budget)
{
    namespace A=::AnalyticCertified;
    using I=A::Interval;
    using C::SignedAffineSphereResidual::Hull;
    struct FPoint {I S,Z;};
    struct FNode {FPoint Net[6];int32 Depth=0;};
    if(!budget.Consume()||panel.DegreeU!=5||panel.DegreeV!=5||panel.ControlPoints.Num()!=36)return false;
    FNode root;
    for(int32 v=0;v<6;++v)for(int32 u=0;u<6;++u)
    {
        if(!budget.Consume())return false;
        const auto& point=panel.ControlPoints[u*6+v];
        if(!C::Detail::ValidPoint(point))return false;
        const I s=A::Add(point.Axis[0],point.Axis[1]);
        if(!A::Valid(s))return false;
        auto& bound=root.Net[v];
        bound=u?FPoint{Hull(bound.S,s),Hull(bound.Z,point.Axis[2])}:FPoint{s,point.Axis[2]};
    }
    const auto project=[](I s,I z,I n,I m){return A::Add(A::Multiply(s,n),A::Multiply(z,m));};
    const auto mid=[](I a){return a.Lower*.5+a.Upper*.5;};
    const auto half=I::Exact(.5);
    const auto average=[&](I a,I b){return A::Add(A::Multiply(a,half),A::Multiply(b,half));};
    TArray<FNode,TInlineAllocator<25>> pending;pending.Add(root);
    while(!pending.IsEmpty())
    {
        if(!budget.Consume())return false;
        FNode node=pending.Pop(EAllowShrinking::No),left,right;
        left.Depth=right.Depth=node.Depth+1;
        FNode layer=node;
        if(!edgeAxis)
        {
            left.Net[0]=layer.Net[0];right.Net[5]=layer.Net[5];
            for(int32 degree=5;degree>0;--degree)
            {
                for(int32 k=0;k<degree;++k)
                {
                    if(!budget.Consume())return false;
                    layer.Net[k]={average(layer.Net[k].S,layer.Net[k+1].S),average(layer.Net[k].Z,layer.Net[k+1].Z)};
                    if(!A::Valid(layer.Net[k].S)||!A::Valid(layer.Net[k].Z))return false;
                }
                left.Net[6-degree]=layer.Net[0];right.Net[degree-1]=layer.Net[degree-1];
            }
        }
        if(!budget.Consume())return false;
        const auto n=I::Exact((mid(layer.Net[0].S)-mid(centerS[2]))*.5);
        const auto m=I::Exact(mid(layer.Net[0].Z)-mid(centerZ[2]));
        const auto norm2=A::Add(A::Multiply(I::Exact(2),A::Square(n)),A::Square(m));
        if(!A::Valid(norm2)||norm2.Lower<=0)return false;
        double lo=TNumericLimits<double>::Max(),hi=-TNumericLimits<double>::Max();
        for(const auto& point:node.Net)
        {
            if(!budget.Consume())return false;
            const auto projection=project(point.S,point.Z,n,m);
            if(!A::Valid(projection))return false;
            lo=FMath::Min(lo,projection.Lower);
        }
        for(int32 k=0;k<3;++k)
        {
            if(!budget.Consume())return false;
            const auto projection=project(centerS[k],centerZ[k],n,m);
            if(!A::Valid(projection))return false;
            hi=FMath::Max(hi,projection.Upper);
        }
        const auto gap=A::Subtract(I::Exact(lo),I::Exact(hi));
        const auto gap2=A::Square(gap),required=A::Multiply(radiusSquared,norm2);
        if(!A::Valid(gap2)||!A::Valid(required))return false;
        if(gap.Lower>0&&gap2.Lower>required.Upper)continue;
        if(edgeAxis||node.Depth>=24)return false;
        pending.Add(MoveTemp(left));pending.Add(MoveTemp(right));
    }
    return true;
}

bool TryDiagonal698Prefix(const FPreparedGeometrySelectionPtr& geometry,
    const C::FZeroOffsetKinematicCenterReference& model,const FStaticSphereInitialValues& values,
    const PreparedExecution::FSelectedWholeHullPartition* fullRc,const C::FReferenceCandidateSetPtr& generic,
    float end,C::FDistanceBudget& budget)
{
    namespace A=::AnalyticCertified;
    namespace Prefix=C::SignedAffineSphereCorrelatedPrefix;
    using I=A::Interval;
    const auto rc=fullRc&&fullRc->Partition?fullRc->Partition->GetCandidateIndices():TConstArrayView<int32>{};
    const auto reference=generic?generic->GetCandidateIndices():TConstArrayView<int32>{};
    const uint32 endBits=C::KinematicCenterDetail::Bits(end);
    const uint64 before=budget.Used;
    const int32 available=budget.Remaining-budget.ConsumptionFloor;
    const auto finish=[&](const TCHAR* stage,bool clear=false)
    {
        UE_LOG(LogTemp,Display,TEXT("[NativeSphereDiagonal698Prefix] Stage=%s EndBits=%08x RsCount=%d RcCount=%d Rc0=%d Rc1=%d Rc2=%d Available=%d Floor=%d Used=%llu"),
            stage,endBits,reference.Num(),rc.Num(),rc.Num()>0?rc[0]:-1,rc.Num()>1?rc[1]:-1,rc.Num()>2?rc[2]:-1,
            available,budget.ConsumptionFloor,budget.Used-before);
        return clear;
    };
    if(endBits!=C::KinematicCenterDetail::Bits(0.0018721388187259436f))return finish(TEXT("end"));
    if(!geometry||geometry->GetModel()!=EPreparedGeometryModel::RetainedReference||!geometry->GetReference()||
        geometry->GetReference()->GetExternalPacketIdentity()!=TEXT("090d41158d13f1127827b720ff467a51e415668aa205209a67ed8afcb6b10dc0"))
        return finish(TEXT("scene"));
    if(!generic||generic->GetScene()!=geometry->GetSelectedUnion()||reference.Num()!=2||reference[0]!=1697||reference[1]!=2360||
        A::Bits(generic->GetRadiusForFixedSource())!=A::Bits(values.SupportRadius))return finish(TEXT("reference_roster"));
    if(!fullRc||!fullRc->UsesGeometry(geometry)||rc.Num()!=2||rc[0]!=1697||rc[1]!=2360||
        A::Bits(fullRc->Partition->GetRadiusForFixedSource())!=A::Bits(double(values.QueryRadius))||
        !ContainsRoundedCenterHull(*fullRc->Partition,C::BoundConsumedCenterBetweenFloatSeconds(model,0.f,end)))
        return finish(TEXT("rc_cover_or_roster"));
    constexpr int32 allowance=1792;
    if(available<allowance)return finish(TEXT("response_reserve"));
    const auto prove=[&]()->const TCHAR*
    {
        C::FScopedDistanceBudgetFloor paid(budget,budget.Remaining-allowance);
        if(!paid.IsValid()||!budget.Consume())return TEXT("proof_account");
        Prefix::FCenterNet x,y;
        const auto& p=values.State;
        if(!Prefix::BuildCenterNet(p.Location,p.Velocity,p.Acceleration,end,values.SupportRadius,1,x)||
            !Prefix::BuildCenterNet(p.Location,p.Velocity,p.Acceleration,end,values.SupportRadius,0,y))return TEXT("center_net");
        I centerS[3],centerZ[3];
        for(int32 k=0;k<3;++k)
        {
            if(!budget.Consume())return TEXT("center_account");
            centerS[k]=A::Add(A::Add(x.Axis[0][k],x.TravelOrigin),A::Add(y.Axis[0][k],y.TravelOrigin));
            centerZ[k]=A::Add(x.Axis[1][k],I::Exact(values.SupportRadius));
            if(!A::Valid(centerS[k])||!A::Valid(centerZ[k]))return TEXT("center_projection");
        }
        const auto panels=fullRc->Partition->GetSurfacesForFixedSource();
        const auto radiusSquared=A::Square(I::Exact(values.SupportRadius));
        if(panels.Num()!=2||!A::Valid(radiusSquared))return TEXT("panel_inputs");
        if(!ProveDiagonal698Panel(panels[0],true,centerS,centerZ,radiusSquared,budget))return TEXT("panel_1697");
        if(!ProveDiagonal698Panel(panels[1],false,centerS,centerZ,radiusSquared,budget))return TEXT("panel_2360");
        return nullptr;
    };
    const TCHAR* refusal=prove();
    return refusal?finish(refusal):finish(TEXT("rs_prefix_closed"),true);
}
TAutoConsoleVariable<int32> CNativeSphereDiagonal698ReferenceClear(
    TEXT("p.IAmSpeed.AnalyticWorld.NativeSphereDiagonal698ReferenceClear"),0,
    TEXT("Prove only the first dyadic half of the original case 698 reference path. Default OFF."));

TAutoConsoleVariable<int32> CNativeSphereDiagonal698ReferenceWindow(
    TEXT("p.IAmSpeed.AnalyticWorld.NativeSphereDiagonal698ReferenceWindow"),0,
    TEXT("Prune only the original dyadic reference frontier using a proven case 698 window. Default OFF."));

constexpr double Diagonal698WindowClear=18403./32768.;
constexpr double Diagonal698WindowInside=18404./32768.;

bool TryDiagonal698ReferenceClear(const FPreparedGeometrySelectionPtr& geometry,
    const FStaticSphereInitialValues& values,const PreparedExecution::FSelectedWholeHullPartition* fullRc,
    const C::FReferenceCandidateSetPtr& reference,C::FDistanceBudget& budget,bool window=false)
{
    namespace A=::AnalyticCertified;
    using I=A::Interval;
    if(!geometry||geometry->GetModel()!=EPreparedGeometryModel::RetainedReference||!geometry->GetReference()||
        geometry->GetReference()->GetExternalPacketIdentity()!=TEXT("090d41158d13f1127827b720ff467a51e415668aa205209a67ed8afcb6b10dc0")||
        !reference||reference->GetScene()!=geometry->GetSelectedUnion()||!fullRc||!fullRc->UsesGeometry(geometry)||
        A::Bits(reference->GetRadiusForFixedSource())!=A::Bits(values.SupportRadius)||
        A::Bits(fullRc->Partition->GetRadiusForFixedSource())!=A::Bits(double(values.QueryRadius)))
        return false;
    const auto rs=reference->GetCandidateIndices(),rc=fullRc->Partition->GetCandidateIndices();
    if(rs.Num()!=2||rs[0]!=1697||rs[1]!=2360||rc.Num()!=2||rc[0]!=rs[0]||rc[1]!=rs[1])
        return false;
    const auto panels=reference->GetSurfacesForFixedSource();
    const auto& path=reference->GetPathForFixedSource();
    constexpr int32 allowance=1792;
    constexpr int32 originalSearchReserve=20000;
    if(panels.Num()!=2||path.Degree!=2||!C::CenterDetail::Valid(path)||
        budget.Remaining-budget.ConsumptionFloor<allowance+originalSearchReserve)
        return false;
    C::FScopedDistanceBudgetFloor paid(budget,budget.Remaining-allowance);
    if(!paid.IsValid()||!budget.Consume())return false;
    const auto r2=A::Square(I::Exact(values.SupportRadius));
    if(!A::Valid(r2))return false;
    C::FIntervalBezierCenterPath left,right;
    if(window)
    {
        if(panels[1].DegreeU!=5||panels[1].DegreeV!=5||panels[1].ControlPoints.Num()!=36)return false;
        // Charge the bounded 5x5 original point evaluation and the degree-2
        // time controls. This UV is only a proposal on the live original net.
        for(int32 i=0;i<120;++i)if(!budget.Consume())return false;
        C::FIntervalPoint witness;
        const FVector2d uv(0.4813379645347595,0.8700420907698572);
        if(!C::ProjectedDistanceDetail::EvaluateFixedUV(panels[1],uv,witness))return false;
        const auto time=I::Exact(Diagonal698WindowInside);
        const auto center=C::CenterDetail::Evaluate(path,time);
        if(!C::Detail::ValidPoint(center))return false;
        I distance2=I::Exact(0.);
        for(int32 d=0;d<3;++d)
            distance2=A::Add(distance2,A::Square(A::Subtract(witness.Axis[d],center.Axis[d])));
        if(!A::Valid(distance2)||!(distance2.Upper<r2.Lower))return false;
        const auto end=I::Exact(Diagonal698WindowClear),other=A::Subtract(I::Exact(1.),end);
        left.Degree=2;left.ControlPoints=path.ControlPoints;
        for(int32 d=0;d<3;++d)
        {
            left.ControlPoints[1].Axis[d]=A::Add(A::Multiply(other,path.ControlPoints[0].Axis[d]),
                A::Multiply(end,path.ControlPoints[1].Axis[d]));
            const auto tail=A::Add(A::Multiply(other,path.ControlPoints[1].Axis[d]),
                A::Multiply(end,path.ControlPoints[2].Axis[d]));
            left.ControlPoints[2].Axis[d]=A::Add(A::Multiply(other,left.ControlPoints[1].Axis[d]),
                A::Multiply(end,tail));
        }
        if(!C::CenterDetail::Valid(left))return false;
    }
    else if(!C::CenterDetail::Split(path,left,right))return false;
    I centerS[3],centerZ[3];
    for(int32 k=0;k<3;++k)
    {
        if(!budget.Consume())return false;
        const auto& point=left.ControlPoints[k];
        centerS[k]=A::Add(point.Axis[0],point.Axis[1]);
        centerZ[k]=point.Axis[2];
        if(!A::Valid(centerS[k]))return false;
    }
    return ProveDiagonal698Panel(panels[0],true,centerS,centerZ,r2,budget)&&
        ProveDiagonal698Panel(panels[1],false,centerS,centerZ,r2,budget);
}

#endif

C::SignedAffineSphereResidual::EDecision TryWholeRcWallSlices(
    const C::AffineReferenceDetail::FProduct& product,const C::FZeroOffsetKinematicCenterReference& model,
    const FWorldQuery& query,const C::FIntervalPoint& completeCenters,const FVector3d& literalLerpError,
    double radius,C::FDistanceBudget& budget)
{
    namespace A=::AnalyticCertified;
    namespace Signed=C::SignedAffineSphereResidual;
    using E=Signed::EDecision;using I=A::Interval;
    if(!Signed::Private::Admits(product)||budget.Remaining-budget.ConsumptionFloor<48)
        return E::NotApplicable;
    const int32 travel=1-product.FreeWorldAxis;
    const auto width=[](I range){return A::Subtract(I::Exact(range.Upper),I::Exact(range.Lower));};
    const auto rise=width(completeCenters.Axis[2]);
    const auto across=A::Multiply(I::Exact(4.),width(completeCenters.Axis[travel]));
    // Ordering hint only: a wall approach can widen global Z far more
    // than the cross-wall axis where the shifted residual is best conditioned.
    if(!A::Valid(rise)||!A::Valid(across)||!(rise.Lower>across.Upper))
        return E::NotApplicable;
    if(!budget.Consume())return E::Work;
    auto rotated=product;
    for(auto& point:rotated.Curve.ControlPoints)Swap(point.Axis[2],point.Axis[travel]);
    for(int32 half=0;half<2;++half)
    {
        if(!budget.Consume())return E::Work;
        const I time{double(half)*.5,double(half+1)*.5,true};
        const auto native=C::BoundConsumedCenterRange(model,time);
        const auto ideal=C::CenterDetail::Evaluate(model.GetReferenceCurve(),time);
        if(!C::Detail::ValidPoint(native)||!C::Detail::ValidPoint(ideal))return E::NotApplicable;
        auto centers=C::TimeDetail::Hull(native,ideal);
        for(int32 axis=0;axis<3;++axis)
        {
            const auto start=I::Exact(query.Start[axis]),end=I::Exact(query.End[axis]);
            const auto chord=A::Add(A::Add(start,A::Multiply(A::Subtract(end,start),time)),
                I{-literalLerpError[axis],literalLerpError[axis],true});
            if(!A::Valid(chord))return E::NotApplicable;
            // Each true center also lies in the validated complete hull.
            // Intersecting removes only interval overhang and retains its
            // already checked complete selected-volume/radius relationship.
            auto& bound=centers.Axis[axis];
            const auto& full=completeCenters.Axis[axis];
            bound.Lower=FMath::Max(full.Lower,FMath::Min(bound.Lower,chord.Lower));
            bound.Upper=FMath::Min(full.Upper,FMath::Max(bound.Upper,chord.Upper));
            if(!A::Valid(bound))return E::NotApplicable;
        }
        // Orthogonal permutation of the original product and center.
        // The free axis stays dropped only for this strict lower bound.
        Swap(centers.Axis[2],centers.Axis[travel]);
        Signed::FCenterRange shifted;
        if(!Signed::BuildRoundedRange(centers,radius,shifted))return E::NotApplicable;
        const auto proof=Signed::Exclude(rotated,shifted,16,budget);
        if(proof!=E::Clear)return proof;
    }
    return E::Clear;
}

#if !UE_BUILD_SHIPPING
struct FNativeBroadC2WholeRc
{
    const FReferenceSphereStrictSeparation& Initial;
    const C::FReferenceCandidateSet& Candidates;
    const PreparedExecution::FSelectedWholeHullPartition& Rounded;
    const VariableC2Time::FNativeProposals& Proposals;
};
#endif

// Native no-contact publication needs Rc, including the literal query chord.

#if !UE_BUILD_SHIPPING
TAutoConsoleVariable<int32> CSignedC2DirectedWholeRc(TEXT("p.IAmSpeed.AnalyticWorld.SignedC2DirectedWholeRc"),0,
    TEXT("Default OFF. Prove an original whole sphere volume by projected signed tensor support near the retained stadium seam."));

TAutoConsoleVariable<int32> CSignedC2HighShortPreclear(TEXT("p.IAmSpeed.AnalyticWorld.SignedC2HighShortPreclear"),0,
    TEXT("Default OFF. Prove four original High quarters and full native Rc before generic time search on a short remainder."));

TAutoConsoleVariable<int32> CSignedC2HighBroadPreclear(TEXT("p.IAmSpeed.AnalyticWorld.SignedC2HighBroadPreclear"),0,
    TEXT("Default OFF. Prove the four original High/Broad cells and complete Rc before generic temporal search."));

bool IsDirectedC2Approach(const C::FIntervalPoint& center)
{
    return C::Detail::ValidPoint(center)&&
        center.Axis[0].Lower>=4977.75&&center.Axis[0].Upper<=4980.5&&
        center.Axis[1].Lower>=-1343.75&&center.Axis[1].Upper<=-1342.5&&
        center.Axis[2].Lower>=95.25&&center.Axis[2].Upper<=96.5;
}

TAutoConsoleVariable<int32> CSignedC2PostSeamThirdPanel(
    TEXT("p.IAmSpeed.AnalyticWorld.SignedC2PostSeamThirdPanel"),0,
    TEXT("Default OFF. Try the authenticated finite 1098 product after the established post-seam arrivals."));

// Work admission only. Every candidate still needs its live directed proof.
bool IsPostSeamDirectedC2(const C::FIntervalPoint& center,float horizon)
{
    if(C::Detail::ValidPoint(center)&&
        center.Axis[0].Lower>=4981.5&&center.Axis[0].Upper<=5024.&&
        center.Axis[1].Lower>=-1360.&&center.Axis[1].Upper<=-1344.&&
        center.Axis[2].Lower>=96.75&&center.Axis[2].Upper<=120.)return true;
    return CSignedC2PostSeamThirdPanel.GetValueOnAnyThread()==1&&
        C::KinematicCenterDetail::Bits(horizon)==C::KinematicCenterDetail::Bits(1.f/300.f)&&
        PostSeamC2::ContainsThirdPanel(center);
}

bool IsShortDirectedC2Remainder(const PreparedExecution::FNativeSolidStaticValues& native)
{
    // This is only work admission. A short horizon cannot replace any of
    // the original numeric, literal chord or complete-volume checks.
    if(!(native.Horizon>0.f&&native.Horizon<=0x1p-10f))return false;
    PreparedExecution::FStepIdentity step;
    return PreparedExecution::ReadCurrentStep(step)&&step.Frame==native.FrameClock.CanonicalFrame&&
        step.TimePassed>0.f&&C::KinematicCenterDetail::Bits(step.Remaining)==C::KinematicCenterDetail::Bits(native.Horizon);
}

bool ProveDirectedC2Tensor(const C::FIntervalTensorSurface& surface,const C::FIntervalPoint (&centers)[3],
    const FVector3d& centerMid,::AnalyticCertified::Interval r2,C::FDistanceBudget& budget)
{
    namespace A=::AnalyticCertified;
    using P=C::FIntervalPoint;
    if(surface.DegreeU!=5||surface.DegreeV!=5)return false;

    C::Detail::FNode root;
    root.DegreeU=surface.DegreeU;root.DegreeV=surface.DegreeV;
    for(const auto& point:surface.ControlPoints)root.Net.Add(point);
    TArray<C::Detail::FNode,TInlineAllocator<16>> pending;
    pending.Add(MoveTemp(root));
    while(!pending.IsEmpty())
    {
        if(!budget.Consume())return false;
        auto node=pending.Pop(EAllowShrinking::No);
        P cell;
        FVector3d mean=FVector3d::ZeroVector;
        for(const auto& point:node.Net)
        {
            if(!C::Detail::ValidPoint(point))return false;
            cell=cell.Axis[0].Known?C::TimeDetail::Hull(cell,point):point;
            for(int32 d=0;d<3;++d)
                mean[d]+=(.5*point.Axis[d].Lower+.5*point.Axis[d].Upper)/36.;
        }
        FVector3d boxAxis;
        for(int32 d=0;d<3;++d)
        {
            const auto& b=cell.Axis[d];
            boxAxis[d]=FMath::Clamp(centerMid[d],b.Lower,b.Upper)-centerMid[d];
        }
        // Both directions only propose separators. The directed proof
        // encloses every center and every point of the entire tensor cell.
        bool separated=false;
        for(const FVector3d& axis:{mean-centerMid,boxAxis})
        {
            if(!budget.Consume())return false;
            if(!C::GroupedBoxDetail::ValidAxis(axis))continue;
            const auto patch=C::GroupedBoxDetail::ProjectHull(node.Net,axis);
            const auto center=C::GroupedBoxDetail::ProjectHull(centers,axis);
            const auto norm=C::GroupedBoxDetail::Project(C::Detail::Point(axis),axis);
            const auto gap=A::Subtract(patch,center);
            const auto squared=A::Subtract(A::Square(gap),A::Multiply(r2,norm));
            if(A::Valid(gap)&&A::Valid(norm)&&A::Valid(squared)&&
                norm.Lower>0.&&gap.Lower>0.&&squared.Lower>0.)
            {
                separated=true;
                break;
            }
        }
        if(separated)continue;
        if(node.DepthU+node.DepthV>=24||!budget.Consume())return false;
        C::Detail::FNode first,second;
        // V resolves the stadium section; every child retains the full U span.
        if(!C::Detail::Split(node,false,first,second))return false;
        pending.Add(MoveTemp(first));pending.Add(MoveTemp(second));
    }
    return true;
}

TAutoConsoleVariable<int32> CSignedC2DirectedAdaptive2375(
    TEXT("p.IAmSpeed.AnalyticWorld.SignedC2DirectedAdaptive2375"),0,
    TEXT("Default OFF. Bounded original 2375 clear proof on the full padded center path."));

bool TryAdaptiveDirected2375(const FReferenceTransport& reference,const C::FIntervalTensorSurface& surface,
    const C::FIntervalPoint (&centers)[3],const C::FIntervalPoint& hull,double radius,
    C::FDistanceBudget& budget,int32& visited)
{
    namespace A=::AnalyticCertified;
    namespace Signed=C::SignedAffineSphereResidual;
    using I=A::Interval;
    using P=C::FIntervalPoint;
    constexpr int32 allowance=96,maximumDepth=4,maximumVisits=15,provider=2375;
    // Complete original V half-Hessian >571 throughout this authenticated
    // panel chart. The existing finite-edge helper uses the lower bound 500.
    if(!C::Detail::ValidPoint(hull)||hull.Axis[0].Lower<4985.5||hull.Axis[0].Upper>5008.||
        hull.Axis[1].Lower< -1354.||hull.Axis[1].Upper> -1344.5||
        hull.Axis[2].Lower<98.25||hull.Axis[2].Upper>113.||
        surface.DegreeU!=5||surface.DegreeV!=5||surface.ControlPoints.Num()!=36||
        budget.Remaining-budget.ConsumptionFloor<allowance)return false;
    C::FScopedDistanceBudgetFloor local(budget,budget.Remaining-allowance);
    if(!local.IsValid()||!budget.Consume())return false;
    const auto identities=reference.GetProofIdentities();
    const auto map=reference.GetProviderFaceIndices();
    const auto faces=reference.GetFaces();
    const auto& expected=VariableC2Time::Faces[3];
    if(expected.Index!=provider||identities.Num()!=10||
        identities[0]!=TEXT("46cd45f7af71cf745f541c92e1a10066c9dba9cb7fcc27d1fbe0d0ecdcaf31a8")||
        map.Num()!=2992||!faces.IsValidIndex(map[provider]))return false;
    const auto& face=faces[map[provider]];
    if(face.FromPlane||face.CanonicalRecordIdentity!=expected.Canonical||face.BoundRecordIdentity!=expected.Bound||
        face.DomainAndChart.Num()!=4)return false;

    const FReferenceAffineExtrusionDescription* proposal=nullptr;
    for(const auto& fact:reference.GetExactAffineExtrusions(provider))
    {
        if(!budget.Consume()||fact.Provider!=provider||fact.SourceFace!=map[provider]||
            fact.CorrelationProofIdentity!=reference.GetExactAffineProofIdentity()||
            fact.CanonicalRecordIdentity!=expected.Canonical||fact.BoundRecordIdentity!=expected.Bound)return false;
        if(fact.Product.FreeWorldAxis!=1||!fact.Product.FreeAlongU)continue;
        if(proposal)return false;
        proposal=&fact;
    }
    if(!proposal)return false;
    for(int32 u=0;u<6;++u)for(int32 v=0;v<6;++v)
    {
        if(!budget.Consume())return false;
        for(int32 d=0;d<3;d+=2)
        {
            const auto q=surface.ControlPoints[6*u+v].Axis[d],row=surface.ControlPoints[v].Axis[d];
            if(!A::Valid(q)||!A::Valid(row)||A::Bits(q.Lower)!=A::Bits(row.Lower)||A::Bits(q.Upper)!=A::Bits(row.Upper))
                return false;
        }
    }

    struct FCell {C::FIntervalBezierCenterPath Path;int32 Depth;};
    C::FIntervalBezierCenterPath original;
    original.Degree=2;original.ControlPoints={centers[0],centers[1],centers[2]};
    TArray<FCell,TInlineAllocator<maximumDepth+1>> pending;
    pending.Add({MoveTemp(original),0});
    while(!pending.IsEmpty())
    {
        if(++visited>maximumVisits||!budget.Consume())return false;
        auto cell=pending.Pop(EAllowShrinking::No);
        const auto middle=C::CenterDetail::Evaluate(cell.Path,I::Exact(.5));
        Signed::FCenterRange shifted;
        double v=0.,lower=0.;
        if(!Signed::BuildRoundedRange(middle,radius,shifted)||!Signed::ProposeCurveV(proposal->Product,shifted,v))
            return false;
        P projected[]={cell.Path.ControlPoints[0],cell.Path.ControlPoints[1],cell.Path.ControlPoints[2]};
        // Dropping Y is a lower bound on the same original finite surface.
        for(auto& point:projected)point.Axis[1]=I::Exact(-1344.);
        if(C::SignedC2FiniteRival::ProveOnCheckedFiniteEdge(surface,projected,v,radius,budget,lower)&&lower>0.)
            continue;
        C::FIntervalBezierCenterPath left,right;
        if(cell.Depth>=maximumDepth||!budget.Consume()||!C::CenterDetail::Split(cell.Path,left,right))return false;
        pending.Add({MoveTemp(right),cell.Depth+1});
        pending.Add({MoveTemp(left),cell.Depth+1});
    }
    return true;
}

bool TryDirectedFinitePanelCover(const FReferenceTransport& reference,int32 provider,const C::FIntervalTensorSurface& patch,
    const C::FIntervalPoint (&centers)[3],const C::FIntervalPoint& hull,double radius,
    const C::FIntervalBezierCenterPath& referencePath,const ::AnalyticCertified::Interval (&errors)[3],
    const FVector3d& literalStart,const FVector3d& literalEnd,const FVector3d& literalLerpError,C::FDistanceBudget& budget)
{
    namespace A=::AnalyticCertified;
    using I=A::Interval;
    const bool descendingNext=provider==2375;
    if((!descendingNext&&provider!=870)||
        !(descendingNext?PostSeamC2::ContainsDescendingNextPanel(hull):PostSeamC2::ContainsFourthWholeVolume(hull))||
        !budget.Consume())return false;
    const auto& expected=descendingNext?VariableC2Time::Faces[3]:VariableC2Time::SeamExtraFaces[2];
    const auto identities=reference.GetProofIdentities();
    const auto map=reference.GetProviderFaceIndices();
    const auto faces=reference.GetFaces();
    if(expected.Index!=provider||identities.Num()!=10||
        identities[0]!=TEXT("46cd45f7af71cf745f541c92e1a10066c9dba9cb7fcc27d1fbe0d0ecdcaf31a8")||
        map.Num()!=2992||faces.Num()!=2654||!faces.IsValidIndex(map[provider]))return false;
    const auto& face=faces[map[provider]];
    if(face.FromPlane||face.CanonicalRecordIdentity!=expected.Canonical||face.BoundRecordIdentity!=expected.Bound||
        face.DomainAndChart.Num()!=4)return false;
    const FReferenceAffineExtrusionDescription* selected=nullptr;
    for(const auto& fact:reference.GetExactAffineExtrusions(provider))
    {
        if(!budget.Consume()||fact.Provider!=provider||fact.SourceFace!=map[provider]||
            fact.CorrelationProofIdentity!=reference.GetExactAffineProofIdentity()||
            fact.CanonicalRecordIdentity!=expected.Canonical||fact.BoundRecordIdentity!=expected.Bound)return false;
        if(fact.Product.FreeWorldAxis!=1)continue;
        if(selected||!fact.Product.FreeAlongU)return false;
        selected=&fact;
    }
    if(!selected||!budget.Consume())return false;
    C::FIntervalBezierCenterPath physical=referencePath,literal=referencePath;
    for(int32 d=0;d<3;++d)
    {
        if(!A::Finite(literalLerpError[d])||literalLerpError[d]<0.)return false;
        const auto a=I::Exact(literalStart[d]),b=I::Exact(literalEnd[d]);
        const I chord[]={a,A::Multiply(I::Exact(.5),A::Add(a,b)),b};
        const I error{-literalLerpError[d],literalLerpError[d],true};
        for(int32 k=0;k<3;++k)
        {
            auto& actual=physical.ControlPoints[k].Axis[d];
            auto& line=literal.ControlPoints[k].Axis[d];
            actual=A::Add(actual,errors[d]);
            line=A::Add(chord[k],error);
            if(!A::Valid(actual)||!A::Valid(line)||
                actual.Lower<hull.Axis[d].Lower||actual.Upper>hull.Axis[d].Upper||
                line.Lower<hull.Axis[d].Lower||line.Upper>hull.Axis[d].Upper)return false;
        }
    }
    return PostSeamC2::FProof::ClearFiniteWholeVolume(patch,centers,hull,radius,physical,literal,
        *selected,budget,descendingNext);
}

TAutoConsoleVariable<int32> CNativeSphereFiniteWallQuadraticRs(
    TEXT("p.IAmSpeed.AnalyticWorld.NativeSphereFiniteWallQuadraticRs"),0,
    TEXT("Use the original finite plane time plan on the authenticated quadratic wall query. Default OFF."));

bool AdmitFiniteWallQuadraticRs(const FPreparedGeometrySelectionPtr& geometry,
    const FStaticSphereInitialValues& values,const C::FZeroOffsetKinematicCenterReference& model,
    const PreparedExecution::FNativeSolidStaticValues& native,const C::FReferenceBoundsCandidateSet& volume,
    const PreparedExecution::FSelectedWholeHullPartition& fullRc,const C::FReferenceCandidateSetPtr& candidates,
    C::FDistanceBudget& budget)
{
    namespace A=::AnalyticCertified;
    using I=A::Interval;
    using namespace C::CanonicalAxialPlaneDetail;
    if(!geometry||!geometry->HasValidAliases()||geometry->GetModel()!=EPreparedGeometryModel::RetainedReference||
        !geometry->GetReference()||!candidates||!fullRc.UsesGeometry(geometry)||!fullRc.Partition||
        !model.IsKnown()||model.GetReferenceCurve().Degree!=2||!C::CenterDetail::Valid(model.GetReferenceCurve()))
        return false;
    const auto& reference=*geometry->GetReference();
    const auto& scene=geometry->GetSelectedUnion();
    const auto& q=native.Query;
    if(reference.GetExternalPacketIdentity()!=TEXT("090d41158d13f1127827b720ff467a51e415668aa205209a67ed8afcb6b10dc0")||
        reference.GetExactAffineProofIdentity()!=TEXT("719562ab664d391eaf4de243d049078909284b5172c997894ae229f3829d0892")||
        candidates->GetScene()!=scene||fullRc.Partition->GetScene()!=scene||volume.GetScene()!=scene||
        native.Shape!=PreparedExecution::ENativeSolidQueryShape::Sphere||native.Frozen||native.Prescribed||
        q.Shape!=EQueryShape::Sphere||q.bObjectQuery||!q.bApplyCollisionFilter||!q.bAuthorityOnly||
        q.bExcludeAuthorityEligible||q.RequiredSourceId||q.RequiredSurfaceId||q.RequiredCanonicalGroupId||
        !A::Finite(q.DomainTolerance)||q.DomainTolerance<0.||q.Start!=values.State.Location||q.Start!=native.Start||
        q.End!=native.End||q.End.X<=q.Start.X||
        A::Bits(q.Radius)!=A::Bits(double(values.QueryRadius))||
        A::Bits(q.Radius)!=A::Bits(fullRc.Partition->GetRadiusForFixedSource())||
        A::Bits(values.SupportRadius)!=A::Bits(candidates->GetRadiusForFixedSource())||
        !(0.<values.SupportRadius&&values.SupportRadius<=q.Radius))return false;

    const auto rs=candidates->GetCandidateIndices(),rc=fullRc.Partition->GetCandidateIndices();
    const auto padded=volume.GetCandidateIndices();
    if(rs.Num()!=2||rs[0]!=2693||rs[1]!=2694||rc.Num()!=2||rc[0]!=rs[0]||rc[1]!=rs[1]||
        !padded.Contains(rs[0])||!padded.Contains(rs[1])||padded.Num()>16||
        rs.Num()+candidates->GetExcludedIndices().Num()!=scene->Num()||
        rc.Num()+fullRc.Partition->GetExcludedIndices().Num()!=scene->Num()||
        padded.Num()+volume.GetExcludedIndices().Num()!=scene->Num()||
        budget.Remaining-budget.ConsumptionFloor<512)return false;
    C::FScopedDistanceBudgetFloor allowance(budget,budget.Remaining-512);
    if(!allowance.IsValid()||!budget.Consume())return false;

    const auto& fixed=candidates->GetPathForFixedSource();
    const auto& ideal=model.GetReferenceCurve();
    if(fixed.Degree!=2||fixed.ControlPoints.Num()!=3)return false;
    for(int32 i=0;i<3;++i)
        if(!SamePoint(fixed.ControlPoints[i],ideal.ControlPoints[i]))return false;
    auto centers=C::TimeDetail::Hull(C::CenterDetail::Hull(ideal),
        C::BoundConsumedCenterBetweenFloatSeconds(model,0.f,native.Horizon));
    const auto radius=A::Add(I::Exact(q.Radius),A::Multiply(I::Exact(3.),I::Exact(q.DomainTolerance)));
    if(!C::Detail::ValidPoint(centers)||!C::Detail::ValidPoint(volume.GetQueryHull())||!A::Valid(radius))return false;
    const auto radius2=A::Square(I::Exact(radius.Upper));
    if(!A::Valid(radius2))return false;
    for(int32 d=0;d<3;++d)
    {
        if(!budget.Consume())return false;
        const auto lerp=A::Add(A::Multiply(I::Exact(0x1p-48),
            A::Add(I::Exact(FMath::Abs(q.Start[d])),I::Exact(FMath::Abs(q.End[d])))),I::Exact(0x1p-1000));
        const auto literal=A::Add(I{FMath::Min(q.Start[d],q.End[d]),FMath::Max(q.Start[d],q.End[d]),true},
            I{-lerp.Upper,lerp.Upper,true});
        if(!A::Valid(lerp)||!A::Valid(literal))return false;
        auto& axis=centers.Axis[d];
        axis={FMath::Min(axis.Lower,literal.Lower),FMath::Max(axis.Upper,literal.Upper),true};
        const auto occupied=A::Add(axis,I{-radius.Upper,radius.Upper,true});
        const auto& cover=volume.GetQueryHull().Axis[d];
        if(!A::Valid(occupied)||occupied.Lower<cover.Lower||occupied.Upper>cover.Upper)return false;
    }

    const auto nominal=geometry->GetNominalScene();
    if(!nominal||centers.Axis[0].Upper>=5120.||centers.Axis[1].Upper>-1344.||centers.Axis[2].Lower<160.)
        return false;
    const FBoundedPlane* plane=nullptr;
    for(const auto& candidate:nominal->GetPlanes())
    {
        if(!budget.Consume())return false;
        if(candidate.PrimitiveId!=0x34d093ea700ae349ULL)continue;
        if(plane)return false;
        plane=&candidate;
    }
    if(!plane||plane->Origin.X!=5120.||plane->Normal!=FVector3d(-1.,0.,0.)||
        plane->SourceId!=1274291482858373289ULL||plane->SurfaceId!=17000356195137514675ULL||
        plane->FeatureId!=3456533302335197110ULL)return false;
    const uint64 count=uint64(plane->DomainVertices.Num());
    if(count<3||count>16||!C::CandidateDetail::ConsumeRepeated(budget,2u+count+count*(count-1u)/2u)||
        !plane->IsValid())return false;
    C::FCanonicalAxialPlaneBounds finite;
    finite.Plane=*plane;finite.NormalAxis=0;finite.UAxis=Axis(plane->AxisU);finite.VAxis=Axis(plane->AxisV);
    if(finite.UAxis<=0||finite.VAxis<=0||finite.UAxis==finite.VAxis)return false;
    auto foot=centers;foot.Axis[0]=I::Exact(5120.);
    const auto owners=nominal->GetOwners();
    const auto surfaces=candidates->GetSurfacesForFixedSource();
    if(surfaces.Num()!=2||!owners.IsValidIndex(rs[0])||!owners.IsValidIndex(rs[1]))return false;
    const uint64 group=owners[rs[0]].CanonicalGroupId;
    if(!OwnerIsPlane(owners[rs[0]],*plane,group)||!OwnerIsPlane(owners[rs[1]],*plane,group))return false;
    for(int32 k=0;k<2;++k)
        if(!budget.Consume()||!ExactPlaneTriangle(finite,surfaces[k]))return false;
    finite.Triangle=surfaces[0];
    if(!TriangleInterior(finite,foot))return false;
    // The companion stays in the unchanged solver. Authenticate a strict
    // exterior half-space here, using the same complete transverse hull.
    const auto& cp=surfaces[1].ControlPoints;
    const auto orientation=Orient(cp[0],cp[2],cp[1],finite.UAxis,finite.VAxis);
    const int32 sign=Positive(orientation,1)?1:Positive(orientation,-1)?-1:0;
    if(!sign)return false;
    const int32 vertices[]={0,2,1};
    bool exterior=false;
    for(int32 i=0;i<3;++i)
    {
        if(!budget.Consume())return false;
        exterior|=Positive(Orient(cp[vertices[i]],cp[vertices[(i+1)%3]],foot,finite.UAxis,finite.VAxis),-sign);
    }
    if(!exterior)return false;
    for(const int32 index:padded)
    {
        if(index==rs[0]||index==rs[1])continue;
        C::FIntervalTensorSurface net;
        C::FIntervalPoint bounds;
        if(!budget.Consume()||!scene->CopySurfaceForQuery(index,budget,net)||
            !C::CandidateDetail::SurfaceHull(net,bounds))return false;
        const auto squared=C::CandidateDetail::BoxDistanceSquared(bounds,centers);
        if(!A::Valid(squared)||!(squared.Lower>radius2.Upper))return false;
    }
    return true;
}

bool ProveFiniteWallConsumedPrefix(const FStaticSphereInitialValues& values,
    const C::FZeroOffsetKinematicCenterReference& model,float end,C::FDistanceBudget& budget,
    int32& tail,double& minimumGap)
{
    namespace A=::AnalyticCertified;
    using I=A::Interval;
    using namespace C::KinematicCenterDetail;
    tail=0;minimumGap=0.;
    const uint32 endBits=Bits(end);
    if(endBits<FloatMinNormal||endBits>=FloatInfinity||!budget.Consume())return false;
    I error[3];
    if(!VariableC2RoundedPrefix::CenterError(values,double(end),error))return false;
    const auto p=I::Exact(values.State.Location.X),v=I::Exact(values.State.Velocity.X);
    const auto a=I::Exact(values.State.Acceleration.X),radius=I::Exact(values.SupportRadius);
    const auto slope=A::Add(v,A::Multiply(a,I{0.,double(end),true}));
    if(!A::Valid(slope)||slope.Lower<=0.)return false;
    const auto gap=[&](I x)
    {return A::Subtract(A::Subtract(I::Exact(5120.),x),radius);};
    const auto bulk=[&](float cap)
    {
        const auto t=I::Exact(double(cap));
        const auto ideal=A::Add(A::Add(p,A::Multiply(v,t)),A::Multiply(A::Multiply(I::Exact(.5),a),A::Square(t)));
        // Positive slope on the whole prefix and a uniform bound of the
        // original consumed DAG give an upper X bound at every earlier ordinal.
        return gap(A::Add(ideal,error[0]));
    };
    auto clear=bulk(end);
    if(A::Valid(clear)&&clear.Lower>0.){minimumGap=clear.Lower;return true;}
    constexpr uint32 maximumTail=16;
    if(endBits<FloatMinNormal+maximumTail||!budget.Consume())return false;
    const uint32 bulkBits=endBits-maximumTail;
    clear=bulk(FromBits(bulkBits));
    if(!A::Valid(clear)||clear.Lower<=0.)return false;
    minimumGap=clear.Lower;
    for(uint32 ordinal=bulkBits+1;ordinal<=endBits;++ordinal)
    {
        if(!budget.Consume())return false;
        const auto point=C::BoundConsumedCenterBetweenFloatSeconds(model,FromBits(ordinal),FromBits(ordinal));
        if(!C::Detail::ValidPoint(point))return false;
        const auto exactGap=gap(point.Axis[0]);
        if(!A::Valid(exactGap)||exactGap.Lower<=0.)return false;
        minimumGap=FMath::Min(minimumGap,exactGap.Lower);
        ++tail;
    }
    return true;
}

bool ProveDirectedC2WholeVolume(const FReferenceTransport& reference,const C::FReferenceBoundsCandidateSet& volume,
    const C::FZeroOffsetKinematicCenterReference& model,const FStaticSphereInitialValues& captured,
    float horizon,const FVector3d& chordError,const FVector3d& literalStart,const FVector3d& literalEnd,
    const FVector3d& literalLerpError,double radius,C::FDistanceBudget& budget,
    bool shortRemainder,uint32 ownerFrame,bool lateFourthVolume,bool descendingThirdVolume,bool descendingNextVolume)
{
    namespace A=::AnalyticCertified;
    using I=A::Interval;
    using P=C::FIntervalPoint;
    const auto& path=model.GetReferenceCurve();
    I errors[3];
    if(path.Degree!=2||path.ControlPoints.Num()!=3||radius>VariableC2Time::BroadNativeRadiusCap||
        !VariableC2RoundedPrefix::CenterError(captured,double(horizon),errors)||!budget.Consume())return false;
    P centers[3];
    FVector3d centerMid=FVector3d::ZeroVector;
    for(int32 d=0;d<3;++d)
    {
        if(!A::Finite(chordError[d])||chordError[d]<0.)return false;
        const auto error=A::Add(errors[d],I{-chordError[d],chordError[d],true});
        for(int32 k=0;k<3;++k)
        {
            auto& value=centers[k].Axis[d];
            value=A::Add(path.ControlPoints[k].Axis[d],error);
            if(!A::Valid(value))return false;
            centerMid[d]+=(.5*value.Lower+.5*value.Upper)/3.;
        }
    }
    const auto hull=C::TimeDetail::Hull(C::TimeDetail::Hull(centers[0],centers[1]),centers[2]);
    if(!C::Detail::ValidPoint(hull)||(!shortRemainder&&!IsDirectedC2Approach(hull)&&
        !IsPostSeamDirectedC2(hull,horizon)&&!(lateFourthVolume&&PostSeamC2::ContainsFourthWholeVolume(hull))&&
        !(descendingThirdVolume&&PostSeamC2::ContainsDescendingThirdPanel(hull))&&
        !(descendingNextVolume&&PostSeamC2::ContainsDescendingNextPanel(hull))))return false;
    const auto r2=A::Square(I::Exact(radius));
    if(!A::Valid(r2))return false;
    // This only chooses a proof subdivision. It preserves the whole original
    // padded center curve, native roster, literal-chord bound and live radius.
    const bool firstDeparture=hull.Axis[0].Lower>=4985.875&&hull.Axis[0].Upper<=4988.&&
        hull.Axis[1].Lower>=-1345.875&&hull.Axis[1].Upper<=-1345.125&&
        hull.Axis[2].Lower>=98.375&&hull.Axis[2].Upper<=99.25;
    const bool nextDeparture=hull.Axis[0].Lower>=4989.25&&hull.Axis[0].Upper<=4991.5&&
        hull.Axis[1].Lower>=-1346.875&&hull.Axis[1].Upper<=-1346.125&&
        hull.Axis[2].Lower>=99.875&&hull.Axis[2].Upper<=101.125;
    const bool splitSibling=!shortRemainder&&C::KinematicCenterDetail::Bits(horizon)==C::KinematicCenterDetail::Bits(1.f/300.f)&&
        (firstDeparture||nextDeparture);
    for(int32 index:volume.GetCandidateIndices())
    {
        C::FIntervalTensorSurface surface;
        P bounds;
        if(!budget.Consume()||!volume.GetScene()->CopySurfaceForQuery(index,budget,surface)||
            !C::CandidateDetail::SurfaceHull(surface,bounds))return false;
        const auto rootGap=A::Subtract(C::CandidateDetail::BoxDistanceSquared(bounds,hull),r2);
        if(!A::Valid(rootGap))return false;
        if(rootGap.Lower>0.)continue;
        if(index==2375&&CSignedC2DirectedAdaptive2375.GetValueOnAnyThread()==1)
        {
            const uint64 before=budget.Used;
            int32 visited=0;
            if(TryAdaptiveDirected2375(reference,surface,centers,hull,radius,budget,visited))
            {
                UE_LOG(LogTemp,Display,TEXT("[NativeSphereC2DirectedAdaptive2375] Frame=%u Provider=2375 H=%.17g Radius=%.17g Visits=%d Used=%llu"),
                    ownerFrame,double(horizon),radius,visited,budget.Used-before);
                continue;
            }
        }
        if(!splitSibling||index!=2375)
        {
            constexpr int32 finiteAllowance=256;
            if(((lateFourthVolume&&index==870)||(descendingNextVolume&&index==2375))&&
                budget.Remaining-budget.ConsumptionFloor>=finiteAllowance)
            {
                const uint64 before=budget.Used;
                bool clear=false;
                {
                    C::FScopedDistanceBudgetFloor optional(budget,budget.Remaining-finiteAllowance);
                    if(optional.IsValid())
                        clear=TryDirectedFinitePanelCover(reference,index,surface,centers,hull,radius,path,errors,
                            literalStart,literalEnd,literalLerpError,budget);
                }
                if(clear)
                {
                    if(index==870)
                    {
                        UE_LOG(LogTemp,Display,TEXT("[NativeSphereC2DirectedFourthCover] Frame=%u Provider=870 Clear=1 Used=%llu"),
                            ownerFrame,budget.Used-before);
                    }
                    else
                    {
                        UE_LOG(LogTemp,Display,TEXT("[NativeSphereC2DirectedNextCover] Frame=%u Provider=2375 Clear=1 Used=%llu"),
                            ownerFrame,budget.Used-before);
                    }
                    continue;
                }
            }
            if(!ProveDirectedC2Tensor(surface,centers,centerMid,r2,budget))return false;
            continue;
        }
        const uint64 before=budget.Used;
        C::FIntervalBezierCenterPath whole,left,right;
        whole.Degree=2;whole.ControlPoints={centers[0],centers[1],centers[2]};
        if(!budget.Consume()||!C::CenterDetail::Split(whole,left,right))return false;
        for(const auto* half:{&left,&right})
        {
            if(!budget.Consume())return false;
            const P controls[]={half->ControlPoints[0],half->ControlPoints[1],half->ControlPoints[2]};
            FVector3d proposal=FVector3d::ZeroVector;
            for(const auto& point:controls)for(int32 d=0;d<3;++d)
                proposal[d]+=(.5*point.Axis[d].Lower+.5*point.Axis[d].Upper)/3.;
            if(!ProveDirectedC2Tensor(surface,controls,proposal,r2,budget))return false;
        }
        UE_LOG(LogTemp,Display,TEXT("[NativeSphereC2DirectedSiblingHalves] Provider=2375 H=%.17g Radius=%.17g Clear=1 Used=%llu"),
            double(horizon),radius,budget.Used-before);
    }
    return true;
}

TAutoConsoleVariable<int32> CSignedC2PostSeamRs(TEXT("p.IAmSpeed.AnalyticWorld.SignedC2PostSeamRs"),0,
    TEXT("Default OFF. Prove the original signed post-seam positive Rs arrivals with full competitor cover."));

TAutoConsoleVariable<int32> CSignedC2NaturalSeamRs(TEXT("p.IAmSpeed.AnalyticWorld.SignedC2NaturalSeamRs"),0,
    TEXT("Default OFF. Prove separate Rs and Rc order on the authenticated 695/1940 seam approach."));

TAutoConsoleVariable<int32> CSignedC2PostSeamNextPanelPrefix(
    TEXT("p.IAmSpeed.AnalyticWorld.SignedC2PostSeamNextPanelPrefix"),0,
    TEXT("Default OFF. Limit the original 2375 rival proof to a strict inside witness when the whole horizon cannot be separated."));

TAutoConsoleVariable<int32> CSignedC2WallLipRs(
    TEXT("p.IAmSpeed.AnalyticWorld.SignedC2WallLipRs"),0,
    TEXT("Default OFF. Prove the original positive Rs contact at the signed lower 870 wall lip; keep the distinct Rc wall overlap."));
TAutoConsoleVariable<int32> CSignedC2PostSeamFourthPanel(
    TEXT("p.IAmSpeed.AnalyticWorld.SignedC2PostSeamFourthPanel"),0,
    TEXT("Try the authenticated original affine 870 panel and retain the entire live competitor volume. Default OFF."));

bool IsExpandedFourthPanelFullVolume(float horizon,const C::FIntervalPoint& rawCenterHull)
{
    return CSignedC2PostSeamFourthPanel.GetValueOnAnyThread()==1&&
        C::KinematicCenterDetail::Bits(horizon)==C::KinematicCenterDetail::Bits(1.f/300.f)&&
        !PostSeamC2::ContainsOriginalFourthPanel(rawCenterHull)&&PostSeamC2::ContainsFourthWholeVolume(rawCenterHull);
}

bool IsLateDirectedC2Remainder(const PreparedExecution::FNativeSolidStaticValues& native,
    const C::FIntervalPoint& rawCenterHull)
{
    // Admit work only on a live remainder in the signed 870 region.
    // The whole-volume producer still tests its padded hull and every role.
    if(CSignedC2PostSeamFourthPanel.GetValueOnAnyThread()!=1||
        !(native.Horizon>0x1p-10f&&native.Horizon<1.f/300.f)||
        !PostSeamC2::ContainsFourthWholeVolume(rawCenterHull))return false;
    PreparedExecution::FStepIdentity step;
    PreparedExecution::FCanonicalPreparedFrameClock clock;
    const auto bits=C::KinematicCenterDetail::Bits;
    return PreparedExecution::ReadCurrentStep(step)&&
        PreparedExecution::JoinCanonicalPreparedFrameClock(step.Frame,native.FrameClock.OwnerLocalFrame,clock)&&
        clock==native.FrameClock&&bits(step.Delta)==bits(1.f/300.f)&&
        step.TimePassed>0.f&&step.TimePassed<step.Delta&&native.Horizon<step.Delta&&
        bits(step.Remaining)==bits(native.Horizon);
}

bool IsDescendingFiniteDirectedVolume(const FStaticSphereInitialValues& captured,
    const PreparedExecution::FNativeSolidStaticValues& native,const C::FIntervalPoint& rawCenterHull,
    TConstArrayView<int32> liveIndices,bool nextPanel)
{
    // This admits only work for the original full native cover. It neither
    // selects a contact nor excludes any other live provider.
    if(!(captured.State.Velocity.Z<0.&&captured.State.Acceleration.Z<0.)||
        !(native.Horizon>0x1p-10f&&native.Horizon<=1.f/300.f))return false;
    if(nextPanel)
    {
        if(CSignedC2PostSeamNextPanelPrefix.GetValueOnAnyThread()!=1||
            !PostSeamC2::ContainsDescendingNextPanel(rawCenterHull)||!liveIndices.Contains(2375))return false;
    }
    else if(CSignedC2PostSeamThirdPanel.GetValueOnAnyThread()!=1||
        !PostSeamC2::ContainsDescendingThirdPanel(rawCenterHull)||!liveIndices.Contains(1098))return false;
    PreparedExecution::FStepIdentity step;
    PreparedExecution::FCanonicalPreparedFrameClock clock;
    const auto bits=C::KinematicCenterDetail::Bits;
    if(!PreparedExecution::ReadCurrentStep(step)||
        !PreparedExecution::JoinCanonicalPreparedFrameClock(step.Frame,native.FrameClock.OwnerLocalFrame,clock)||
        clock!=native.FrameClock||bits(step.Delta)!=bits(1.f/300.f)||bits(step.Remaining)!=bits(native.Horizon))return false;
    return bits(native.Horizon)==bits(step.Delta)?step.TimePassed==0.f:step.TimePassed>0.f&&step.TimePassed<step.Delta;
}

TAutoConsoleVariable<int32> CSignedC2NextPanelTighterCap(
    TEXT("p.IAmSpeed.AnalyticWorld.SignedC2NextPanelTighterCap"),0,
    TEXT("Refine a certified 2375 upper cap and prove original rival 1098 on four common-path intervals. Default OFF."));

TAutoConsoleVariable<int32> CSignedC2ThirdPanelSeparateOrder(
    TEXT("p.IAmSpeed.AnalyticWorld.SignedC2ThirdPanelSeparateOrder"),0,
    TEXT("Prove separate first-provider order on the physical path and literal chord of the signed 1098/870 transition. Default OFF."));

// Private to the producer. A physical Rs cap can be consumed only after
// both paths and every retained competitor pass. LiteralEnd is diagnostic.
struct FThirdPanelSeparateOrder
{
    double PhysicalEnd=0.;
    double LiteralEnd=0.;
};

bool ProveThirdPanelRival870(const FReferenceTransport& reference,const C::FIntervalTensorSurface& surface,
    const C::FIntervalPoint (&physical)[3],const C::FIntervalPoint (&literal)[3],double radius,C::FDistanceBudget& account)
{
    namespace A=::AnalyticCertified;
    using I=A::Interval;
    constexpr int32 provider=870;
    const auto map=reference.GetProviderFaceIndices();
    const auto faces=reference.GetFaces();
    if(!map.IsValidIndex(provider)||!faces.IsValidIndex(map[provider]))return false;
    const auto& face=faces[map[provider]];
    if(face.FromPlane||face.DomainAndChart.Num()!=4)return false;
    const FReferenceAffineExtrusionDescription* product=nullptr;
    for(const auto& fact:reference.GetExactAffineExtrusions(provider))
    {
        if(!account.Consume()||fact.Provider!=provider||
            fact.CorrelationProofIdentity!=reference.GetExactAffineProofIdentity())return false;
        if(fact.Product.FreeWorldAxis!=1)continue;
        if(product||!fact.Product.FreeAlongU||fact.SourceFace!=map[provider]||
            fact.CanonicalRecordIdentity!=face.CanonicalRecordIdentity||
            fact.BoundRecordIdentity!=face.BoundRecordIdentity)return false;
        product=&fact;
    }
    if(!product||!product->StructurallyMatches(surface,account))return false;
    const auto& edge=product->Product.Curve.ControlPoints;
    const auto scaledRadius=C::SignedC2FiniteRival::Scale(6.,A::Square(I::Exact(radius)));
    if(!A::Valid(scaledRadius))return false;
    const auto clear=[&](const C::FIntervalPoint (&centers)[3])
    {
        // Signed 870 has XZ half-Hessian >522 on this full-V chart.
        // Nonnegative V gradient on the full time cell makes V=0 its
        // projected minimum; dropping finite Y only decreases distance.
        if(!account.Consume())return false;
        I gradient[3],norm[5];
        for(auto& value:gradient)value=I::Exact(0.);
        for(auto& value:norm)value=I::Exact(0.);
        for(const auto& p:centers)
            if(!C::Detail::ValidPoint(p)||p.Axis[0].Lower<5020.||p.Axis[0].Upper>5040.||
                p.Axis[1].Lower<-1372.||p.Axis[1].Upper>-1350.||
                p.Axis[2].Lower<131.||p.Axis[2].Upper>148.)return false;
        for(int32 d=0;d<3;d+=2)
        {
            const auto start=edge[0].Axis[d];
            const auto tangent=C::SignedC2FiniteRival::Scale(5.,A::Subtract(edge[1].Axis[d],start));
            I delta[3],squared[5];
            for(int32 k=0;k<3;++k)
            {
                delta[k]=A::Subtract(start,centers[k].Axis[d]);
                gradient[k]=A::Add(gradient[k],A::Multiply(delta[k],tangent));
            }
            C::SignedC2FiniteRival::SquareCoefficients(delta,squared);
            for(int32 k=0;k<5;++k)norm[k]=A::Add(norm[k],squared[k]);
        }
        for(const auto g:gradient)if(!A::Valid(g)||g.Lower<0.)return false;
        for(const auto n:norm)
        {
            const auto gap=A::Subtract(n,scaledRadius);
            if(!A::Valid(gap)||gap.Lower<=0.)return false;
        }
        return true;
    };
    return clear(physical)&&clear(literal);
}

bool BuildThirdPanelSeparateOrder(const FReferenceSphereStrictSeparation& initial,const C::FReferenceCandidateSet& rs,
    const FWorldQuery& query,const PostSeamC2::FProof& proof,const C::Interval (&nativeError)[3],
    const C::Interval (&literalError)[3],C::FDistanceBudget& account,C::FIntervalPoint (&physical)[3],
    C::FIntervalPoint (&literal)[3],FThirdPanelSeparateOrder& order)
{
    namespace A=::AnalyticCertified;
    using I=A::Interval;
    const auto& path=rs.GetPathForFixedSource();
    const auto& values=initial.GetCapturedInput();
    const auto rc=double(values.QueryRadius);
    const auto rsRadius=values.SupportRadius;
    const auto literalAt=[&](double t)
    {
        C::FIntervalPoint center;
        for(int32 d=0;d<3;++d)
        {
            const auto start=I::Exact(query.Start[d]),end=I::Exact(query.End[d]);
            const auto p=A::Add(start,A::Multiply(I::Exact(t),A::Subtract(end,start)));
            center.Axis[d]=A::Add(p,literalError[d]);
        }
        return center;
    };
    const auto inside=[&](const C::FIntervalPoint& center,double radius)
    {
        FVector2d uv;
        C::FIntervalPoint point;
        I gap;
        return account.Consume()&&proof.Inside(center,radius,account,uv,point,gap);
    };
    const auto findCap=[&](auto at,double radius,int32 iterations,double& cap)
    {
        // The lower seed limits proposals only. It need not be clear.
        double lo=.75,hi=1.;
        if(!inside(at(hi),radius))return false;
        for(int32 i=0;i<iterations;++i)
        {
            const double t=.5*(lo+hi);
            // Only positive inside witnesses move the upper end. Failed
            // proposals guide later samples, never count as separation.
            if(inside(at(t),radius))hi=t;
            else lo=t;
        }
        if(!(hi>0.&&hi<1.))return false;
        cap=hi;
        return true;
    };
    const C::FIntervalPoint literalStart=literalAt(0.);
    const C::FIntervalPoint literalStartOnly[]={literalStart,literalStart,literalStart};
    if(!proof.Clear(literalStartOnly,VariableC2Time::BroadNativeRadiusCap,account)||
        !findCap([&](double t){return C::CenterDetail::Evaluate(path,I::Exact(t));},rsRadius,11,order.PhysicalEnd)||
        !findCap(literalAt,rc,8,order.LiteralEnd))return false;
    const auto nativeWitness=C::FloorReferenceTimeToFloatSeconds(initial.GetModel(),order.PhysicalEnd);
    if(!nativeWitness.Known||!(nativeWitness.Seconds>0.f)||
        !inside(C::BoundConsumedCenterAtFloatSeconds(initial.GetModel(),nativeWitness.Seconds),rc)||
        !VariableC2Time::Restrict(path,0.,order.PhysicalEnd,account,physical))return false;
    for(auto& p:physical)for(int32 d=0;d<3;++d)p.Axis[d]=A::Add(p.Axis[d],nativeError[d]);
    for(int32 k=0;k<3;++k)literal[k]=literalAt(.5*double(k)*order.LiteralEnd);
    return true;
}

bool IsWallLipRsScope(const PreparedExecution::FNativeSolidStaticValues& native)
{
    const auto& q=native.Query;
    return CSignedC2WallLipRs.GetValueOnAnyThread()==1&&native.Horizon==1.f/300.f&&
        q.Start.X>=5026.&&q.Start.X<5027.&&q.Start.Z>=160.&&q.Start.Z<=164.&&
        q.Start.Y>=-1550.&&q.Start.Y<=-1500.&&q.End.Z<160.&&q.End.Z>=159.;
}

bool ProveOriginalWallLipRsRivals(const FReferenceSphereStrictSeparation& initial,
    const C::FReferenceCandidateSet& rs,const PreparedExecution::FSelectedWholeHullPartition& rc,
    const C::FReferenceBoundsCandidateSet& volume,const PreparedExecution::FNativeSolidStaticValues& native,
    PostSeamC2::FProof& proof,C::FDistanceBudget& account)
{
    namespace A=::AnalyticCertified;
    using I=A::Interval;
    using namespace C::CanonicalAxialPlaneDetail;
    constexpr int32 allowance=1024;
    const auto geometry=initial.GetGeometry();
    if(!geometry||!geometry->HasValidAliases()||!geometry->GetReference()||!rc.UsesGeometry(geometry)||
        !rc.Partition||!proof.IsFourthPanel()||!IsWallLipRsScope(native)||account.Remaining-account.ConsumptionFloor<allowance)
        return false;
    const auto scene=geometry->GetSelectedUnion();
    const auto nominal=geometry->GetNominalScene();
    const auto& reference=*geometry->GetReference();
    const auto& q=native.Query;
    const auto& state=initial.GetCapturedInput().State;
    const auto& values=initial.GetCapturedInput();
    const auto actualStep=[&]()
    {
        PreparedExecution::FStepIdentity step;
        return PreparedExecution::ReadCurrentStep(step)&&step.Frame==native.FrameClock.CanonicalFrame&&
            step.Iteration==1&&step.QuerySerial!=0&&C::KinematicCenterDetail::Bits(step.TimePassed)==0&&
            C::KinematicCenterDetail::Bits(step.Delta)==C::KinematicCenterDetail::Bits(native.Horizon)&&
            C::KinematicCenterDetail::Bits(step.Remaining)==C::KinematicCenterDetail::Bits(native.Horizon);
    };
    if(!scene||!nominal||reference.GetExactAffineProofIdentity()!=TEXT("719562ab664d391eaf4de243d049078909284b5172c997894ae229f3829d0892")||
        reference.GetExternalPacketIdentity()!=TEXT("090d41158d13f1127827b720ff467a51e415668aa205209a67ed8afcb6b10dc0")||
        scene!=reference.GetCanonicalUnion()||rs.GetScene()!=scene||rc.Partition->GetScene()!=scene||volume.GetScene()!=scene||
        native.Shape!=PreparedExecution::ENativeSolidQueryShape::Sphere||native.Frozen||native.Prescribed||
        q.Shape!=EQueryShape::Sphere||q.bObjectQuery||!q.bApplyCollisionFilter||!q.bAuthorityOnly||q.bIncludeTriangles||
        q.bExcludeAuthorityEligible||q.RequiredSourceId||q.RequiredSurfaceId||q.RequiredCanonicalGroupId||
        q.ReferenceNormal!=FVector3d::ZeroVector||q.MinimumReferenceNormalDot!=-1.||
        q.DomainTolerance!=1.e-6||q.InitialOverlapTolerance!=0.05||
        q.Start!=state.Location||q.Start!=native.Start||q.End!=native.End||
        state.Location!=native.OwnerState.Location||state.Velocity!=native.OwnerState.Velocity||
        state.Acceleration!=native.OwnerState.Acceleration||state.Velocity.X!=0.||state.Acceleration.X!=0.||
        state.Velocity.Z>=0.||state.Acceleration.Z>=0.||q.Start.X!=q.End.X||
        A::Bits(q.Radius)!=A::Bits(double(native.QueryRadius))||A::Bits(q.Radius)!=A::Bits(double(values.QueryRadius))||
        A::Bits(q.Radius)!=A::Bits(rc.Partition->GetRadiusForFixedSource())||
        A::Bits(values.SupportRadius)!=A::Bits(rs.GetRadiusForFixedSource())||
        C::KinematicCenterDetail::Bits(initial.GetHorizon())!=C::KinematicCenterDetail::Bits(native.Horizon)||
        !actualStep())return false;
    const auto padded=volume.GetCandidateIndices(),ri=rs.GetCandidateIndices(),ci=rc.Partition->GetCandidateIndices();
    if(padded.Num()!=4||!padded.Contains(870)||!padded.Contains(1098)||!padded.Contains(2693)||!padded.Contains(2694)||
        ri.Num()!=2||!ri.Contains(870)||!ri.Contains(1098)||
        !rs.GetExcludedIndices().Contains(2693)||!rs.GetExcludedIndices().Contains(2694)||
        !ci.Contains(870)||!ci.Contains(2693)||
        padded.Num()+volume.GetExcludedIndices().Num()!=scene->Num()||
        ri.Num()+rs.GetExcludedIndices().Num()!=scene->Num()||
        ci.Num()+rc.Partition->GetExcludedIndices().Num()!=scene->Num())return false;
    C::FScopedDistanceBudgetFloor bounded(account,account.Remaining-allowance);
    if(!bounded.IsValid()||!account.Consume()||!proof.ThirdPanelRetainsRc(padded,account)||!proof.BindWallLip(account))return false;

    const auto& model=initial.GetModel();
    const auto& path=rs.GetPathForFixedSource();
    if(path.Degree!=2||!C::CenterDetail::Valid(path)||!C::Detail::ValidPoint(volume.GetQueryHull()))return false;
    auto centers=C::TimeDetail::Hull(C::CenterDetail::Hull(path),
        C::BoundConsumedCenterBetweenFloatSeconds(model,0.f,native.Horizon));
    const auto radius=A::Add(I::Exact(q.Radius),A::Multiply(I::Exact(3.),I::Exact(q.DomainTolerance)));
    if(!C::Detail::ValidPoint(centers)||!A::Valid(radius))return false;
    I lerpError[3];
    for(int32 d=0;d<3;++d)
    {
        if(!account.Consume())return false;
        const auto e=A::Add(A::Multiply(I::Exact(0x1p-48),
            A::Add(I::Exact(FMath::Abs(q.Start[d])),I::Exact(FMath::Abs(q.End[d])))),I::Exact(0x1p-1000));
        if(!A::Valid(e))return false;
        lerpError[d]=I{-e.Upper,e.Upper,true};
        const auto literal=A::Add(I{FMath::Min(q.Start[d],q.End[d]),FMath::Max(q.Start[d],q.End[d]),true},lerpError[d]);
        if(!A::Valid(literal))return false;
        auto& axis=centers.Axis[d];
        axis={FMath::Min(axis.Lower,literal.Lower),FMath::Max(axis.Upper,literal.Upper),true};
        const auto occupied=A::Add(axis,I{-radius.Upper,radius.Upper,true});
        const auto& cover=volume.GetQueryHull().Axis[d];
        if(!A::Valid(occupied)||occupied.Lower<cover.Lower||occupied.Upper>cover.Upper)return false;
    }

    // Keep both physical wall exclusions and its distinct true literal
    // capture overlap. This is geometry evidence, never an Rc query result.
    const auto wallDistance=A::Subtract(I::Exact(5120.),I::Exact(q.Start.X));
    const auto wallRs=A::Subtract(A::Subtract(I::Exact(5120.),centers.Axis[0]),I::Exact(values.SupportRadius));
    const auto wallRc=A::Subtract(I::Exact(q.Radius),wallDistance);
    if(!A::Valid(wallRs)||!A::Valid(wallRc)||!(wallRs.Lower>0.)||!(wallRc.Lower>0.))return false;
    const FBoundedPlane* plane=nullptr;
    for(const auto& candidate:nominal->GetPlanes())
    {
        if(!account.Consume())return false;
        if(candidate.PrimitiveId!=0x34d093ea700ae349ULL)continue;
        if(plane)return false;
        plane=&candidate;
    }
    if(!plane||plane->Origin.X!=5120.||plane->Normal!=FVector3d(-1.,0.,0.)||
        plane->SourceId!=1274291482858373289ULL||plane->SurfaceId!=17000356195137514675ULL||
        plane->FeatureId!=3456533302335197110ULL)return false;
    const auto owners=nominal->GetOwners();
    if(!owners.IsValidIndex(2693)||!owners.IsValidIndex(2694)||
        !OwnerIsPlane(owners[2693],*plane,owners[2693].CanonicalGroupId)||
        !OwnerIsPlane(owners[2694],*plane,owners[2693].CanonicalGroupId))return false;
    C::FCanonicalAxialPlaneBounds finite;
    finite.Plane=*plane;finite.NormalAxis=0;finite.UAxis=Axis(plane->AxisU);finite.VAxis=Axis(plane->AxisV);
    if(finite.UAxis<=0||finite.VAxis<=0||finite.UAxis==finite.VAxis)return false;
    const auto count=uint64(plane->DomainVertices.Num());
    if(count<3||count>16||!C::CandidateDetail::ConsumeRepeated(account,2u+count+count*(count-1u)/2u)||!plane->IsValid())
        return false;
    auto foot=C::Detail::Point(q.Start);foot.Axis[0]=I::Exact(5120.);
    for(int32 i:{2693,2694})
    {
        if(!account.Consume()||!scene->CopySurfaceForQuery(i,account,finite.Triangle)||
            !ExactPlaneTriangle(finite,finite.Triangle)||(i==2693&&!TriangleInterior(finite,foot)))return false;
    }

    C::FIntervalTensorSurface rival;
    const auto r2=A::Square(I::Exact(radius.Upper));
    const C::FIntervalPoint whole[3]={centers,centers,centers};
    FVector3d middle;
    for(int32 d=0;d<3;++d)middle[d]=.5*centers.Axis[d].Lower+.5*centers.Axis[d].Upper;
    if(!A::Valid(r2)||!scene->CopySurfaceForQuery(1098,account,rival)||
        !ProveDirectedC2Tensor(rival,whole,middle,r2,account))return false;

    FVector2d uv=FVector2d::ZeroVector;
    C::FIntervalPoint sample;
    I gap;
    const auto refAt=[&](double t){return C::CenterDetail::Evaluate(path,I::Exact(t));};
    if(!proof.Inside(refAt(proof.EndLimit()),values.SupportRadius,account,uv,sample,gap)&&
        (!proof.TryRaiseFourthPanelCap(account)||
            !proof.Inside(refAt(proof.EndLimit()),values.SupportRadius,account,uv,sample,gap)))return false;
    const double limit=proof.EndLimit();
    const auto floor=C::FloorReferenceTimeToFloatSeconds(model,limit);
    C::FIntervalPoint literalAtCap;
    for(int32 d=0;d<3;++d)
    {
        const auto time=I::Exact(limit);
        literalAtCap.Axis[d]=A::Add(A::Add(A::Multiply(I::Exact(q.Start[d]),A::Subtract(I::Exact(1.),time)),
            A::Multiply(I::Exact(q.End[d]),time)),lerpError[d]);
    }
    if(!floor.Known||!C::Detail::ValidPoint(literalAtCap)||
        !proof.Inside(C::BoundConsumedCenterAtFloatSeconds(model,floor.Seconds),q.Radius,account,uv,sample,gap)||
        !proof.Inside(literalAtCap,q.Radius,account,uv,sample,gap,true)||!proof.IsLive()||!actualStep())return false;
    UE_LOG(LogTemp,Display,TEXT("[NativeSphereC2WallLipRsOrder] Frame=%u Provider=870 Wall=2693 WallLiteralT0Overlap=1 WallRsGapLower=%.17g WallRcDepthLower=%.17g Cap=%.17g Rs=2 Padded=4 RoundedRc=%d"),
        native.FrameClock.OwnerLocalFrame,wallRs.Lower,wallRc.Lower,limit,ci.Num());
    return true;
}


TAutoConsoleVariable<int32> CSignedC2DescendingNextPanelPrefix(
    TEXT("p.IAmSpeed.AnalyticWorld.SignedC2DescendingNextPanelPrefix"),0,
    TEXT("Private descending 2375/1098 seam order with one jointly verified prefix on the three original paths."));

TAutoConsoleVariable<int32> CSignedC2FourthPanelSeparateOrder(
    TEXT("p.IAmSpeed.AnalyticWorld.SignedC2FourthPanelSeparateOrder"),0,
    TEXT("Private descending 870/1098 seam order with separate original reference Rs, native Rc and literal Rc proof caps."));

bool ProveFourthPanelRival1098(const FReferenceTransport& reference,const C::FIntervalTensorSurface& surface,
    const C::FIntervalPoint (&controls)[3],double radius,C::FDistanceBudget& account)
{
    namespace A=::AnalyticCertified;
    using I=A::Interval;
    constexpr int32 provider=1098;
    const auto map=reference.GetProviderFaceIndices();
    const auto faces=reference.GetFaces();
    if(!map.IsValidIndex(provider)||!faces.IsValidIndex(map[provider]))return false;
    const auto& face=faces[map[provider]];
    if(face.FromPlane||face.DomainAndChart.Num()!=4)return false;
    const FReferenceAffineExtrusionDescription* product=nullptr;
    for(const auto& fact:reference.GetExactAffineExtrusions(provider))
    {
        if(!account.Consume()||fact.Provider!=provider||
            fact.CorrelationProofIdentity!=reference.GetExactAffineProofIdentity())return false;
        if(fact.Product.FreeWorldAxis!=1)continue;
        if(product||!fact.Product.FreeAlongU||fact.SourceFace!=map[provider]||
            fact.CanonicalRecordIdentity!=face.CanonicalRecordIdentity||
            fact.BoundRecordIdentity!=face.BoundRecordIdentity)return false;
        product=&fact;
    }
    if(!product||!product->StructurallyMatches(surface,account))return false;
    const auto& edge=product->Product.Curve.ControlPoints;
    if(product->Product.Curve.DegreeV!=5)return false;
    const auto centers=C::TimeDetail::Hull(C::TimeDetail::Hull(controls[0],controls[1]),controls[2]);
    if(!C::Detail::ValidPoint(centers))return false;
    I delta[2][6],slope[2][5];
    for(int32 j=0;j<2;++j)
    {
        const int32 d=2*j;
        for(int32 k=0;k<6;++k)
        {
            delta[j][k]=A::Subtract(edge[k].Axis[d],centers.Axis[d]);
            if(k) slope[j][k-1]=C::SignedC2FiniteRival::Scale(5.,A::Subtract(edge[k].Axis[d],edge[k-1].Axis[d]));
        }
    }
    constexpr int32 choose4[]={1,4,6,4,1},choose5[]={1,5,10,10,5,1};
    for(int32 k=0;k<10;++k)
    {
        auto derivative=I::Exact(0.);
        for(int32 i=FMath::Max(0,k-4);i<=FMath::Min(5,k);++i)
        {
            if(!account.Consume())return false;
            const int32 j=k-i;
            derivative=A::Add(derivative,C::SignedC2FiniteRival::Scale(double(choose5[i]*choose4[j]),
                A::Add(A::Multiply(delta[0][i],slope[0][j]),A::Multiply(delta[1][i],slope[1][j]))));
        }
        // The omitted binomial denominator is strictly positive. Every V
        // derivative is negative, so dropping finite Y gives a V=1 lower bound.
        if(!A::Valid(derivative)||!(derivative.Upper<0.))return false;
    }
    I squared[5];
    for(auto& x:squared)x=I::Exact(0.);
    for(int32 d=0;d<3;d+=2)
    {
        I endpoint[3],coefficients[5];
        for(int32 k=0;k<3;++k)endpoint[k]=A::Subtract(edge[5].Axis[d],controls[k].Axis[d]);
        C::SignedC2FiniteRival::SquareCoefficients(endpoint,coefficients);
        for(int32 k=0;k<5;++k)
        {
            if(!account.Consume())return false;
            squared[k]=A::Add(squared[k],coefficients[k]);
        }
    }
    const auto scaledRadius=C::SignedC2FiniteRival::Scale(6.,A::Square(I::Exact(radius)));
    for(const auto value:squared)
    {
        const auto gap=A::Subtract(value,scaledRadius);
        if(!A::Valid(gap)||!(gap.Lower>0.))return false;
    }
    return true;
}

bool IsDescendingFourthPanelSeparateOrderScope(const PreparedExecution::FNativeSolidStaticValues& native)
{
    const auto& q=native.Query;
    return CSignedC2FourthPanelSeparateOrder.GetValueOnAnyThread()==1&&native.Horizon==1.f/300.f&&
        q.Start.X>=5021.&&q.Start.X<=5023.&&q.Start.Y>=-1530.&&q.Start.Y<=-1528.&&q.Start.Z>=134.&&q.Start.Z<=136.&&
        q.End.X>=5021.&&q.End.X<q.Start.X&&q.End.Y>=-1530.&&q.End.Y<q.Start.Y&&q.End.Z>=134.&&q.End.Z<q.Start.Z;
}

bool ProveOriginalFourthPanelSeparateOrder(const FReferenceSphereStrictSeparation& initial,
    const C::FReferenceCandidateSet& rs,const PreparedExecution::FSelectedWholeHullPartition& rc,
    const C::FReferenceBoundsCandidateSet& volume,const PreparedExecution::FNativeSolidStaticValues& native,
    PostSeamC2::FProof& proof,C::FDistanceBudget& account)
{
    namespace A=::AnalyticCertified;
    using I=A::Interval;
    constexpr int32 allowance=1152;
    const uint64 before=account.Used;
    const auto geometry=initial.GetGeometry();
    if(!geometry||!geometry->HasValidAliases()||!geometry->GetReference()||!rc.UsesGeometry(geometry)||
        !rc.Partition||!proof.IsFourthPanel()||!IsDescendingFourthPanelSeparateOrderScope(native)||account.Remaining-account.ConsumptionFloor<allowance)
        return false;
    const auto scene=geometry->GetSelectedUnion();
    const auto& reference=*geometry->GetReference();
    const auto& q=native.Query;
    const auto& state=initial.GetCapturedInput().State;
    const auto& values=initial.GetCapturedInput();
    const auto actualStep=[&]()
    {
        PreparedExecution::FStepIdentity step;
        return PreparedExecution::ReadCurrentStep(step)&&step.Frame==native.FrameClock.CanonicalFrame&&
            step.Iteration==1&&step.QuerySerial!=0&&C::KinematicCenterDetail::Bits(step.TimePassed)==0&&
            C::KinematicCenterDetail::Bits(step.Delta)==C::KinematicCenterDetail::Bits(native.Horizon)&&
            C::KinematicCenterDetail::Bits(step.Remaining)==C::KinematicCenterDetail::Bits(native.Horizon);
    };
    if(!scene||reference.GetExactAffineProofIdentity()!=TEXT("719562ab664d391eaf4de243d049078909284b5172c997894ae229f3829d0892")||
        reference.GetExternalPacketIdentity()!=TEXT("090d41158d13f1127827b720ff467a51e415668aa205209a67ed8afcb6b10dc0")||
        scene!=reference.GetCanonicalUnion()||rs.GetScene()!=scene||rc.Partition->GetScene()!=scene||volume.GetScene()!=scene||
        native.Shape!=PreparedExecution::ENativeSolidQueryShape::Sphere||native.Frozen||native.Prescribed||
        q.Shape!=EQueryShape::Sphere||q.bObjectQuery||!q.bApplyCollisionFilter||!q.bAuthorityOnly||q.bIncludeTriangles||
        q.bExcludeAuthorityEligible||q.RequiredSourceId||q.RequiredSurfaceId||q.RequiredCanonicalGroupId||
        q.ReferenceNormal!=FVector3d::ZeroVector||q.MinimumReferenceNormalDot!=-1.||
        q.DomainTolerance!=1.e-6||q.InitialOverlapTolerance!=0.05||
        q.Start!=state.Location||q.Start!=native.Start||q.End!=native.End||
        state.Location!=native.OwnerState.Location||state.Velocity!=native.OwnerState.Velocity||
        state.Acceleration!=native.OwnerState.Acceleration||state.Velocity.X>=0.||
        state.Velocity.Z>=0.||state.Acceleration.Z>=0.||
        A::Bits(q.Radius)!=A::Bits(double(native.QueryRadius))||A::Bits(q.Radius)!=A::Bits(double(values.QueryRadius))||
        A::Bits(q.Radius)!=A::Bits(rc.Partition->GetRadiusForFixedSource())||
        A::Bits(values.SupportRadius)!=A::Bits(rs.GetRadiusForFixedSource())||
        C::KinematicCenterDetail::Bits(initial.GetHorizon())!=C::KinematicCenterDetail::Bits(native.Horizon)||
        !actualStep())return false;
    const auto padded=volume.GetCandidateIndices(),ri=rs.GetCandidateIndices(),ci=rc.Partition->GetCandidateIndices();
    const auto exactRoster=[](TConstArrayView<int32> indices)
    {
        return indices.Num()==3&&indices.Contains(870)&&indices.Contains(1098)&&indices.Contains(2375);
    };
    if(!exactRoster(padded)||!exactRoster(ri)||!exactRoster(ci)||
        padded.Num()+volume.GetExcludedIndices().Num()!=scene->Num()||
        ri.Num()+rs.GetExcludedIndices().Num()!=scene->Num()||
        ci.Num()+rc.Partition->GetExcludedIndices().Num()!=scene->Num())return false;
    C::FScopedDistanceBudgetFloor bounded(account,account.Remaining-allowance);
    if(!bounded.IsValid()||!account.Consume()||!proof.ThirdPanelRetainsRc(padded,account))return false;

    const auto& model=initial.GetModel();
    const auto& path=rs.GetPathForFixedSource();
    I nativeError[3],literalError[3];
    if(path.Degree!=2||!C::CenterDetail::Valid(path)||
        !VariableC2RoundedPrefix::CenterError(values,double(native.Horizon),nativeError))return false;
    const auto radius=A::Add(I::Exact(q.Radius),A::Multiply(I::Exact(3.),I::Exact(q.DomainTolerance)));
    constexpr double rcCompare=VariableC2Time::BroadNativeRadiusCap;
    if(!A::Valid(radius)||radius.Upper>rcCompare)return false;
    auto whole=C::TimeDetail::Hull(C::CenterDetail::Hull(path),
        C::BoundConsumedCenterBetweenFloatSeconds(model,0.f,native.Horizon));
    if(!C::Detail::ValidPoint(whole))return false;
    for(int32 d=0;d<3;++d)
    {
        if(!account.Consume())return false;
        const auto error=A::Add(A::Multiply(I::Exact(0x1p-48),
            A::Add(I::Exact(FMath::Abs(q.Start[d])),I::Exact(FMath::Abs(q.End[d])))),I::Exact(0x1p-1000));
        if(!A::Valid(error))return false;
        literalError[d]=I{-error.Upper,error.Upper,true};
        const auto literal=A::Add(I{FMath::Min(q.Start[d],q.End[d]),FMath::Max(q.Start[d],q.End[d]),true},literalError[d]);
        if(!A::Valid(literal))return false;
        auto& axis=whole.Axis[d];
        axis={FMath::Min(axis.Lower,literal.Lower),FMath::Max(axis.Upper,literal.Upper),true};
        const auto occupied=A::Add(axis,I{-radius.Upper,radius.Upper,true});
        const auto& cover=volume.GetQueryHull().Axis[d];
        if(!A::Valid(occupied)||!A::Valid(cover)||occupied.Lower<cover.Lower||occupied.Upper>cover.Upper)return false;
    }
    C::FIntervalTensorSurface rival,other;
    if(!scene->CopySurfaceForQuery(1098,account,rival)||!scene->CopySurfaceForQuery(2375,account,other))return false;
    FVector3d middle;
    for(int32 d=0;d<3;++d)middle[d]=.5*whole.Axis[d].Lower+.5*whole.Axis[d].Upper;
    const C::FIntervalPoint allPaths[3]={whole,whole,whole};
    if(!ProveDirectedC2Tensor(other,allPaths,middle,A::Square(I::Exact(rcCompare)),account))return false;

    const auto refAt=[&](double alpha){return C::CenterDetail::Evaluate(path,I::Exact(alpha));};
    const auto nativeAt=[&](double alpha)
    {
        const auto floor=C::FloorReferenceTimeToFloatSeconds(model,alpha);
        return floor.Known&&floor.Seconds>0.f?C::BoundConsumedCenterAtFloatSeconds(model,floor.Seconds):C::FIntervalPoint{};
    };
    const auto literalAt=[&](double alpha)
    {
        C::FIntervalPoint center;
        for(int32 d=0;d<3;++d)
        {
            const auto a=I::Exact(alpha);
            const auto start=I::Exact(q.Start[d]),end=I::Exact(q.End[d]);
            const auto position=A::Add(start,A::Multiply(a,A::Subtract(end,start)));
            center.Axis[d]=A::Add(position,literalError[d]);
        }
        return center;
    };
    const auto inside=[&](const C::FIntervalPoint& center,double r)
    {
        FVector2d uv=FVector2d::ZeroVector;
        C::FIntervalPoint point;
        I gap;
        return C::Detail::ValidPoint(center)&&account.Consume()&&proof.Inside(center,r,account,uv,point,gap);
    };
    const auto cap=[&](auto evaluate,double r,double& limit)
    {
        double lo=15./16.,hi=1.;
        if(!inside(evaluate(hi),r))return false;
        for(int32 step=0;step<11;++step)
        {
            const double t=.5*(lo+hi);
            // A failed proposal is not a clear witness and does not certify a lower endpoint.
            if(inside(evaluate(t),r))hi=t;
            else lo=t;
        }
        if(!(hi>0.&&hi<1.))return false;
        limit=hi;
        return true;
    };
    const auto literalStart=literalAt(0.);
    const C::FIntervalPoint literalStartOnly[3]={literalStart,literalStart,literalStart};
    const auto nativeStart=C::BoundConsumedCenterAtFloatSeconds(model,0.f);
    const C::FIntervalPoint nativeStartOnly[3]={nativeStart,nativeStart,nativeStart};
    if(!proof.Clear(literalStartOnly,rcCompare,account)||!proof.Clear(nativeStartOnly,rcCompare,account))return false;
    double rsCap=0.,nativeCap=0.,literalCap=0.;
    if(!cap(refAt,values.SupportRadius,rsCap)||!cap(nativeAt,q.Radius,nativeCap)||!cap(literalAt,q.Radius,literalCap))return false;

    C::FIntervalPoint rsPrefix[3],nativePrefix[3],literalPrefix[3];
    if(!VariableC2Time::Restrict(path,0.,rsCap,account,rsPrefix)||
        !VariableC2Time::Restrict(path,0.,nativeCap,account,nativePrefix))return false;
    for(int32 k=0;k<3;++k)
    {
        literalPrefix[k]=literalAt(.5*double(k)*literalCap);
        for(int32 d=0;d<3;++d)
        {
            rsPrefix[k].Axis[d]=A::Add(rsPrefix[k].Axis[d],nativeError[d]);
            nativePrefix[k].Axis[d]=A::Add(nativePrefix[k].Axis[d],nativeError[d]);
        }
    }
    for(int32 k=0;k<3;++k)
        if(!proof.ContainsCenter(rsPrefix[k])||!proof.ContainsCenter(nativePrefix[k])||
            !proof.ContainsCenter(literalPrefix[k]))return false;
    if(!ProveFourthPanelRival1098(reference,rival,rsPrefix,values.SupportRadius,account)||
        !ProveFourthPanelRival1098(reference,rival,nativePrefix,rcCompare,account)||
        !ProveFourthPanelRival1098(reference,rival,literalPrefix,rcCompare,account)||
        !actualStep()||!proof.SetFourthPanelPhysicalCap(rsCap,account))return false;
    UE_LOG(LogTemp,Display,TEXT("[NativeSphereC2FourthPanelSeparateOrder] Frame=%u Provider=870 Rival=1098 RsCap=%.17g NativeRcCap=%.17g LiteralRcCap=%.17g Padded=3 Used=%llu"),
        native.FrameClock.OwnerLocalFrame,rsCap,nativeCap,literalCap,account.Used-before);
    return true;
}

bool ProvePostSeamRivals(const FReferenceSphereStrictSeparation& initial,
    const C::FReferenceCandidateSet& rs,const C::FReferenceBoundsCandidateSet& volume,
    const PostSeamC2::FProof& proof,C::FDistanceBudget& account,
    const PreparedExecution::FNativeSolidStaticValues* nativeQuery=nullptr,bool split1098=false,
    FThirdPanelSeparateOrder* separateOrder=nullptr,bool* tryLaterFourthPanelCap=nullptr)
{
    if(tryLaterFourthPanelCap)*tryLaterFourthPanelCap=false;
    namespace A=::AnalyticCertified;
    using I=A::Interval;
    constexpr int32 allowance=768;
    const auto geometry=initial.GetGeometry();
    const auto indices=volume.GetCandidateIndices();
    if(account.Remaining-account.ConsumptionFloor<allowance||!geometry||
        volume.GetScene()!=rs.GetScene()||volume.GetScene()!=geometry->GetSelectedUnion()||
        indices.Num()>16||indices.Num()+volume.GetExcludedIndices().Num()!=volume.GetScene()->Num())return false;
    if(separateOrder)
    {
        *separateOrder={};
        if(!proof.IsThirdPanel()||!indices.Contains(870))return false;
    }
    FThirdPanelSeparateOrder provedOrder;
    C::FIntervalPoint physicalPrefix[3],literalPrefix[3];
    C::FScopedDistanceBudgetFloor bounded(account,account.Remaining-allowance);
    for(int32 index:rs.GetCandidateIndices())
        if(!account.Consume()||!indices.Contains(index))return false;
    C::FIntervalPoint controls[3];
    I error[3];
    const auto& values=initial.GetCapturedInput();
    if(!bounded.IsValid()||!VariableC2Time::Restrict(rs.GetPathForFixedSource(),0.,proof.EndLimit(),account,controls)||
        !VariableC2RoundedPrefix::CenterError(values,double(initial.GetHorizon()),error))return false;
    const bool latePanel=proof.IsThirdPanel()||proof.IsFourthPanel()||proof.IsDescendingNextPanel();
    const bool verifyInsideCap=proof.IsFourthPanel()||proof.IsDescendingThirdPanel()||proof.IsDescendingNextPanel();
    if(latePanel||split1098)
    {
        if(!nativeQuery||!account.Consume()||
            (latePanel&&!proof.ThirdPanelRetainsRc(indices,account)))return false;
        if(split1098&&(proof.GlobalProvider()!=2375||!(proof.EndLimit()>0.&&proof.EndLimit()<1.)))return false;
        const auto& query=nativeQuery->Query;
        const auto& state=values.State;
        const I nativeError[]={error[0],error[1],error[2]};
        I literalError[3];
        const auto same=[](const FVector3d& x,const FVector3d& y)
        {
            for(int32 d=0;d<3;++d)if(A::Bits(x[d])!=A::Bits(y[d]))return false;
            return true;
        };
        if(nativeQuery->Shape!=PreparedExecution::ENativeSolidQueryShape::Sphere||
            nativeQuery->Frozen||nativeQuery->Prescribed||query.Shape!=EQueryShape::Sphere||
            query.bObjectQuery||!query.bApplyCollisionFilter||!query.bAuthorityOnly||
            !same(state.Location,nativeQuery->OwnerState.Location)||
            !same(state.Velocity,nativeQuery->OwnerState.Velocity)||
            !same(state.Acceleration,nativeQuery->OwnerState.Acceleration)||
            !same(query.Start,nativeQuery->Start)||!same(query.End,nativeQuery->End)||
            (separateOrder&&!same(query.Start,state.Location))||
            A::Bits(query.Radius)!=A::Bits(double(values.QueryRadius))||
            A::Bits(query.Radius)!=A::Bits(double(nativeQuery->QueryRadius))||
            C::KinematicCenterDetail::Bits(nativeQuery->Horizon)!=C::KinematicCenterDetail::Bits(initial.GetHorizon())||
            !A::Finite(query.DomainTolerance)||query.DomainTolerance<0.||
            !(proof.EndLimit()>0.&&proof.EndLimit()<=1.)||
            (proof.IsThirdPanel()&&proof.EndLimit()!=1.))return false;
        const auto radius=A::Add(I::Exact(query.Radius),A::Multiply(I::Exact(3.),I::Exact(query.DomainTolerance)));
        if(!A::Valid(radius)||radius.Upper>VariableC2Time::BroadNativeRadiusCap)return false;
        // A strict original-Rc start plus a strict Rs inside witness implies a
        // positive Rc crossing of this same provider. Every rival is proved
        // below through all of H at the stronger radius. Rc never sets Rs time.
        if(latePanel)
        {
            C::FIntervalPoint startOnly[3];
            for(auto& point:startOnly)
                for(int32 d=0;d<3;++d)point.Axis[d]=A::Add(controls[0].Axis[d],error[d]);
            if(!proof.Clear(startOnly,VariableC2Time::BroadNativeRadiusCap,account))return false;
        }
        const auto actual=C::BoundConsumedCenterBetweenFloatSeconds(initial.GetModel(),0.f,initial.GetHorizon());
        const auto ideal=C::CenterDetail::Hull(rs.GetPathForFixedSource());
        if(!C::Detail::ValidPoint(actual)||!C::Detail::ValidPoint(ideal))return false;
        const auto physical=C::TimeDetail::Hull(actual,ideal);
        const auto abs=[](double x){return I::Exact(A::FromBits(A::Bits(x)&A::Magnitude));};
        const auto t2=A::Square(I::Exact(double(initial.GetHorizon())));
        const auto& original=rs.GetPathForFixedSource();
        if((split1098||verifyInsideCap)&&(original.Degree!=2||!C::CenterDetail::Valid(original)))return false;
        C::FIntervalPoint chordAtCap;
        for(int32 d=0;d<3;++d)
        {
            // The literal chord differs from the ideal quadratic by the
            // affine endpoint errors and at most |A| H^2 / 8 in each axis.
            if((split1098||verifyInsideCap)&&!account.Consume())return false;
            // Full-H endpoints remain full-H even when the proof cap is shorter.
            const auto& first=proof.IsThirdPanel()?controls[0]:original.ControlPoints[0];
            const auto& last=proof.IsThirdPanel()?controls[2]:original.ControlPoints[2];
            const auto start=A::Subtract(I::Exact(query.Start[d]),first.Axis[d]);
            const auto end=A::Subtract(I::Exact(query.End[d]),last.Axis[d]);
            const auto lerp=A::Add(A::Multiply(I::Exact(0x1p-48),
                A::Add(abs(query.Start[d]),abs(query.End[d]))),I::Exact(0x1p-1000));
            const auto sag=A::Multiply(I::Exact(.125),A::Multiply(abs(state.Acceleration[d]),t2));
            if(!A::Valid(start)||!A::Valid(end)||!A::Valid(lerp)||!A::Valid(sag))return false;
            const double endpoints=FMath::Max(FMath::Max(abs(start.Lower).Upper,abs(start.Upper).Upper),
                FMath::Max(abs(end.Lower).Upper,abs(end.Upper).Upper));
            const auto chordError=A::Add(A::Add(I::Exact(endpoints),sag),lerp);
            const auto lo=FMath::Min(query.Start[d],query.End[d]);
            const auto hi=FMath::Max(query.Start[d],query.End[d]);
            const auto chord=A::Add(I{lo,hi,true},I{-lerp.Upper,lerp.Upper,true});
            if(!A::Valid(chordError)||!A::Valid(chord))return false;
            if(verifyInsideCap)
            {
                const auto cap=I::Exact(proof.EndLimit());
                const auto literal=A::Add(A::Multiply(I::Exact(query.Start[d]),A::Subtract(I::Exact(1.),cap)),
                    A::Multiply(I::Exact(query.End[d]),cap));
                chordAtCap.Axis[d]=A::Add(literal,I{-lerp.Upper,lerp.Upper,true});
            }
            const auto axis=physical.Axis[d];
            const auto expanded=A::Add(I{FMath::Min(axis.Lower,chord.Lower),FMath::Max(axis.Upper,chord.Upper),true},
                I{-radius.Upper,radius.Upper,true});
            const auto& cover=volume.GetQueryHull().Axis[d];
            if(!A::Valid(expanded)||!A::Valid(cover)||expanded.Lower<cover.Lower||expanded.Upper>cover.Upper)return false;
            literalError[d]={-lerp.Upper,lerp.Upper,true};
            const double pad=FMath::Max(error[d].Upper,chordError.Upper);
            error[d]={-pad,pad,true};
        }
        if(verifyInsideCap)
        {
            // Existence at the proposed upper cap on each original path is
            // separate from first entry; the existing Rs search owns timing.
            const auto floor=C::FloorReferenceTimeToFloatSeconds(initial.GetModel(),proof.EndLimit());
            FVector2d uv;
            C::FIntervalPoint point;
            I gap;
            if(!floor.Known||!C::Detail::ValidPoint(chordAtCap))return false;
            if(!proof.Inside(controls[2],values.SupportRadius,account,uv,point,gap)||
                !proof.Inside(C::BoundConsumedCenterAtFloatSeconds(initial.GetModel(),floor.Seconds),
                    double(values.QueryRadius),account,uv,point,gap)||
                !proof.Inside(chordAtCap,double(values.QueryRadius),account,uv,point,gap))
            {
                if(tryLaterFourthPanelCap)*tryLaterFourthPanelCap=true;
                return false;
            }
        }
        if(separateOrder&&!BuildThirdPanelSeparateOrder(initial,rs,query,proof,nativeError,literalError,account,
            physicalPrefix,literalPrefix,provedOrder))return false;
    }
    FVector3d middle=FVector3d::ZeroVector;
    for(auto& point:controls)
    {
        for(int32 d=0;d<3;++d)
        {
            auto& axis=point.Axis[d];
            axis=A::Add(axis,error[d]);
            if(!A::Valid(axis))return false;
            middle[d]+=(.5*axis.Lower+.5*axis.Upper)/3.;
        }
        if(!proof.ContainsCenter(point))return false;
    }
    const auto hull=C::TimeDetail::Hull(C::TimeDetail::Hull(controls[0],controls[1]),controls[2]);
    const auto r2=A::Square(I::Exact(VariableC2Time::BroadNativeRadiusCap));
    for(int32 index:indices)
    {
        if(!account.Consume())return false;
        if(index==proof.GlobalProvider()||index==proof.SeparateRivalProvider())continue;
        C::FIntervalTensorSurface surface;
        C::FIntervalPoint bounds;
        if(!volume.GetScene()->CopySurfaceForQuery(index,account,surface)||
            !C::CandidateDetail::SurfaceHull(surface,bounds))return false;
        const auto gap=A::Subtract(C::CandidateDetail::BoxDistanceSquared(bounds,hull),r2);
        if(!A::Valid(gap))return false;
        if(separateOrder&&index==870)
        {
            if(!ProveThirdPanelRival870(*geometry->GetReference(),surface,physicalPrefix,literalPrefix,
                VariableC2Time::BroadNativeRadiusCap,account))return false;
            continue;
        }
        if(gap.Lower>0.)continue;
        if(split1098&&index==1098)
        {
            // Keep the same radius, tensor and directed kernel. Only the time
            // controls change, covering the whole shorter prefix end-to-end.
            const double quarter=.25*proof.EndLimit();
            for(int32 side=0;side<4;++side)
            {
                C::FIntervalPoint restricted[3];
                if(!VariableC2Time::Restrict(rs.GetPathForFixedSource(),double(side)*quarter,double(side+1)*quarter,
                    account,restricted))return false;
                FVector3d representative=FVector3d::ZeroVector;
                for(auto& point:restricted)
                {
                    for(int32 d=0;d<3;++d)
                    {
                        auto& axis=point.Axis[d];
                        axis=A::Add(axis,error[d]);
                        if(!A::Valid(axis))return false;
                        representative[d]+=(.5*axis.Lower+.5*axis.Upper)/3.;
                    }
                    if(!proof.ContainsCenter(point))return false;
                }
                if(!ProveDirectedC2Tensor(surface,restricted,representative,r2,account))return false;
            }
            continue;
        }
        if(!ProveDirectedC2Tensor(surface,controls,middle,r2,account))return false;
    }
    if(separateOrder)*separateOrder=provedOrder;
    return true;
}

TAutoConsoleVariable<int32> CSignedC2SeamRs(TEXT("p.IAmSpeed.AnalyticWorld.SignedC2SeamRs"),0,
    TEXT("Default OFF. Prove the original positive Rs seam arrival with separate complete Rc coverage."));

bool ProveSeamRsSides(const FReferenceSphereStrictSeparation& initial,const C::FReferenceCandidateSet& rs,
    C::FDistanceBudget& budget)
{
    namespace A=::AnalyticCertified;
    using I=A::Interval;
    const auto& path=rs.GetPathForFixedSource();
    constexpr int32 allowance=128;
    if(budget.Remaining-budget.ConsumptionFloor<allowance)return false;
    C::FScopedDistanceBudgetFloor bounded(budget,budget.Remaining-allowance);
    I error[3];
    if(!bounded.IsValid()||!budget.Consume()||!VariableC2RoundedPrefix::CenterError(
        initial.GetCapturedInput(),double(initial.GetHorizon()),error))return false;
    C::FIntervalPoint controls[3];
    FVector3d centerMid=FVector3d::ZeroVector;
    for(int32 k=0;k<3;++k)for(int32 d=0;d<3;++d)
    {
        auto& out=controls[k].Axis[d];
        out=A::Add(path.ControlPoints[k].Axis[d],error[d]);
        if(!A::Valid(out))return false;
        centerMid[d]+=(.5*out.Lower+.5*out.Upper)/3.;
    }
    const auto indices=rs.GetCandidateIndices();
    const auto surfaces=rs.GetSurfacesForFixedSource();
    const auto squared=A::Square(I::Exact(93.21875));
    for(int32 k=0;k<indices.Num();++k)
    {
        if(!budget.Consume())return false;
        if(indices[k]==695||indices[k]==1940)continue;
        if(!ProveDirectedC2Tensor(surfaces[k],controls,centerMid,squared,budget))return false;
    }
    return true;
}
#endif

// The already closed complete volume supplies original exclusions only after
// checking it contains every center path expanded by the actual proof radius.
bool ProveNativeWholeRcFromVolume(const FPreparedGeometrySelectionPtr& geometry,
    const C::FZeroOffsetKinematicCenterReference& model,const FStaticSphereInitialValues& captured,
    const PreparedExecution::FNativeSolidStaticValues& native,const C::FReferenceBoundsCandidateSet* volume,
    C::FDistanceBudget& budget,bool early=false,bool separatePaths=false
#if !UE_BUILD_SHIPPING
    ,const FNativeBroadC2WholeRc* broad=nullptr,bool directedC2=false
#endif
    )
{
    namespace A=::AnalyticCertified;
    namespace Joint=C::SignedAffineSphereCorrelatedPrefix;
    namespace Signed=C::SignedAffineSphereResidual;
    using I=A::Interval;
    const auto numeric=RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(!geometry||!geometry->HasValidAliases()||!geometry->GetReference()||!volume||
        volume->GetScene()!=geometry->GetSelectedUnion()||volume->GetCandidateIndices().Num()>16||
        volume->GetCandidateIndices().Num()+volume->GetExcludedIndices().Num()!=volume->GetScene()->Num()||
        !C::Detail::ValidPoint(volume->GetQueryHull())||!model.IsKnown()||!budget.Consume())return false;
    const auto& query=native.Query;
    const auto& p=captured.State.Location;const auto& v=captured.State.Velocity;const auto& a=captured.State.Acceleration;
    const auto same=[](const FVector3d& x,const FVector3d& y)
    {
        for(int32 axis=0;axis<3;++axis)if(A::Bits(x[axis])!=A::Bits(y[axis]))return false;
        return true;
    };
    if(native.Shape!=PreparedExecution::ENativeSolidQueryShape::Sphere||native.Frozen||native.Prescribed||
        query.Shape!=EQueryShape::Sphere||query.bObjectQuery||!query.bApplyCollisionFilter||!query.bAuthorityOnly||
        !same(p,native.OwnerState.Location)||!same(v,native.OwnerState.Velocity)||!same(a,native.OwnerState.Acceleration)||
        !same(query.Start,native.Start)||!same(query.End,native.End)||
        A::Bits(query.Radius)!=A::Bits(double(captured.QueryRadius))||
        A::Bits(query.Radius)!=A::Bits(double(native.QueryRadius))||query.Radius<captured.SupportRadius||
        !A::Finite(query.DomainTolerance)||query.DomainTolerance<0||
        !(native.Horizon>0.f&&native.Horizon<=1.f))return false;
    const auto radius=A::Add(I::Exact(query.Radius),A::Multiply(I::Exact(3.),I::Exact(query.DomainTolerance)));
    if(!A::Valid(radius)||!C::CandidateDetail::PositiveRadius(radius.Upper))return false;
    const auto r2=A::Square(I::Exact(radius.Upper));
    const auto nativeHull=C::BoundConsumedCenterBetweenFloatSeconds(model,0.f,native.Horizon);
    const auto idealHull=C::CenterDetail::Hull(model.GetReferenceCurve());
    const FVector3d endpoint=SBox::AdvancePosition(p,v,a,native.Horizon);
    if(!A::Valid(r2)||!C::Detail::ValidPoint(nativeHull)||!C::Detail::ValidPoint(idealHull)||
        !C::ReferenceSphereDetail::Contains(C::BoundConsumedCenterAtFloatSeconds(model,native.Horizon),endpoint))return false;
    auto centers=C::TimeDetail::Hull(nativeHull,idealHull);
    FVector3d chordError=FVector3d::ZeroVector,literalLerpError=FVector3d::ZeroVector;
    const auto absolute=[](double x){return I::Exact(A::FromBits(A::Bits(x)&A::Magnitude));};
    const auto maxAbs=[&](I x){return FMath::Max(absolute(x.Lower).Upper,absolute(x.Upper).Upper);};
    const auto t=I::Exact(double(native.Horizon)),t2=A::Square(t);
    for(int32 axis=0;axis<3;++axis)
    {
        const auto start=I::Exact(query.Start[axis]),end=I::Exact(query.End[axis]);
        const auto base=I::Exact(p[axis]);
        const auto idealEnd=A::Add(A::Add(base,A::Multiply(I::Exact(v[axis]),t)),
            A::Multiply(A::Multiply(I::Exact(.5),I::Exact(a[axis])),t2));
        const auto deltaStart=A::Subtract(start,base),deltaEnd=A::Subtract(end,idealEnd);
        // Covers either standard double Lerp arrangement and its contractions.
        const auto lerpError=A::Add(A::Multiply(I::Exact(0x1p-48),A::Add(absolute(query.Start[axis]),
            absolute(query.End[axis]))),I::Exact(0x1p-1000));
        const auto sag=A::Multiply(I::Exact(.125),A::Multiply(absolute(a[axis]),t2));
        if(!A::Valid(deltaStart)||!A::Valid(deltaEnd)||!A::Valid(lerpError)||!A::Valid(sag))return false;
        // Exact chord minus quadratic equals interpolation of the endpoint
        // differences plus A*H^2*u*(1-u)/2, for u in [0,1].
        const auto extra=A::Add(A::Add(I::Exact(FMath::Max(maxAbs(deltaStart),maxAbs(deltaEnd))),sag),lerpError);
        const I literal{FMath::Min(start.Lower,end.Lower),FMath::Max(start.Upper,end.Upper),true};
        const auto chord=A::Add(literal,I{-lerpError.Upper,lerpError.Upper,true});
        if(!A::Valid(extra)||!A::Valid(chord)||!(extra.Upper>=0))return false;
        chordError[axis]=extra.Upper;literalLerpError[axis]=lerpError.Upper;
        auto& center=centers.Axis[axis];
        center={FMath::Min(center.Lower,chord.Lower),FMath::Max(center.Upper,chord.Upper),true};
        const auto expanded=A::Add(center,I{-radius.Upper,radius.Upper,true});
        const auto& domain=volume->GetQueryHull().Axis[axis];
        if(!A::Valid(expanded)||expanded.Lower<domain.Lower||expanded.Upper>domain.Upper)return false;
    }
#if !UE_BUILD_SHIPPING
    if(directedC2)
    {
        const auto source=geometry->GetReference();
        if(geometry->GetModel()!=EPreparedGeometryModel::RetainedReference||
            source->GetCanonicalUnion()!=volume->GetScene()||
            source->GetExternalPacketIdentity()!=TEXT("090d41158d13f1127827b720ff467a51e415668aa205209a67ed8afcb6b10dc0"))
            return false;
        return ProveDirectedC2WholeVolume(*source,*volume,model,captured,native.Horizon,chordError,
            query.Start,query.End,literalLerpError,radius.Upper,budget,
            IsShortDirectedC2Remainder(native),native.FrameClock.OwnerLocalFrame,
            IsLateDirectedC2Remainder(native,C::CenterDetail::Hull(model.GetReferenceCurve()))||
            IsExpandedFourthPanelFullVolume(native.Horizon,C::CenterDetail::Hull(model.GetReferenceCurve())),
            IsDescendingFiniteDirectedVolume(captured,native,idealHull,volume->GetCandidateIndices(),false),
            IsDescendingFiniteDirectedVolume(captured,native,idealHull,volume->GetCandidateIndices(),true))&&
            RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush).
                ControlFingerprint()==numeric.ControlFingerprint();
    }
    if(broad)
    {
        // Exact same literal-query admission and original volume checks as
        // above. A separate signed proof now clears all three center paths.
        if(&broad->Initial.GetModel()!=&model||broad->Initial.GetGeometry()!=geometry||
            C::KinematicCenterDetail::Bits(broad->Initial.GetHorizon())!=C::KinematicCenterDetail::Bits(native.Horizon)||
            radius.Upper>VariableC2Time::BroadNativeRadiusCap)return false;
        for(int32 index:volume->GetCandidateIndices())
        {
            if(!budget.Consume())return false;
            switch(index)
            {
            case 135:case 695:case 788:case 1442:case 1940:case 2375:break;
            default:
            {
                // The native AABB volume can retain a corner provider that
                // the radial partition excludes. Prove its live immutable
                // bounds against all original query-center paths at the
                // inflated native radius. The original roster is unchanged.
                C::FIntervalTensorSurface surface;
                C::FIntervalPoint box;
                if(!volume->GetScene()->CopySurfaceForQuery(index,budget,surface)||
                    !C::CandidateDetail::SurfaceHull(surface,box))return false;
                const auto gap=A::Subtract(C::CandidateDetail::BoxDistanceSquared(box,centers),r2);
                const bool clear=A::Valid(gap)&&gap.Lower>0.;
                if(broad->Proposals.HighPrefixOnly)
                {
                    UE_LOG(LogTemp,Display,TEXT("[NativeSphereC2HighWholeRcRival] Provider=%d Clear=%d LowerSquaredGap=%.17g"),
                        index,int32(clear),A::Valid(gap)?gap.Lower:0.);
                }
                else
                {
                    UE_LOG(LogTemp,Display,TEXT("[NativeSphereC2BroadWholeRcRival] Provider=%d Clear=%d LowerSquaredGap=%.17g"),
                        index,int32(clear),A::Valid(gap)?gap.Lower:0.);
                }
                if(!clear)return false;
                break;
            }
            }
        }
        double lower=0.;
        const bool clear=VariableC2RoundedPrefix::Prove(broad->Initial,broad->Candidates,broad->Rounded,
            native.Horizon,budget,lower,nullptr,&broad->Proposals,&chordError,radius.Upper);
        return clear&&RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush).
            ControlFingerprint()==numeric.ControlFingerprint();
    }
#endif
    const auto& source=geometry->GetReference();
    Joint::FCenterNet nets[2],literalNets[2];
    bool built[2]={false,false};
    using FFact=FReferenceAffineExtrusionDescription;
    if(early)
    {
        Signed::FCenterRange endpointRange;
        if(!Signed::BuildRoundedRange(C::Detail::Point(endpoint),radius.Upper,endpointRange))return false;
        for(int32 index:volume->GetCandidateIndices())
        {
            if(!budget.Consume())return false;
            for(const auto& fact:source->GetExactAffineExtrusions(index))
            {
                if(!budget.Consume()||fact.Provider!=index||
                    fact.CorrelationProofIdentity!=source->GetExactAffineProofIdentity())return false;
                if(!Signed::Private::Admits(fact.Product))continue;
                // An actual native inside endpoint prevents a whole miss.
                double vProposal=0;
                if(!Signed::ProposeCurveV(fact.Product,endpointRange,vProposal))continue;
                Signed::FWitness witness;
                const auto status=Signed::ProveInside(fact.Product,endpointRange,vProposal,budget,witness);
                if(status==Signed::EDecision::Inside||status==Signed::EDecision::Work||status==Signed::EDecision::Arithmetic)
                    return false;
            }
        }
    }
    TArray<const FFact*,TInlineAllocator<16>> clearProjected;
    for(int32 index:volume->GetCandidateIndices())
    {
        if(!budget.Consume())return false;
        const auto facts=source->GetExactAffineExtrusions(index);
        if(early&&!clearProjected.IsEmpty())
        {
            bool projectedClear=false;
            for(const auto& fact:facts)
            {
                if(!Signed::Private::Admits(fact.Product)||fact.ExactProjectedCurveIdentity.IsEmpty())continue;
                for(const auto* prior:clearProjected)
                    if(prior->Product.FreeWorldAxis==fact.Product.FreeWorldAxis&&
                        prior->ExactProjectedCurveIdentity==fact.ExactProjectedCurveIdentity)
                    {projectedClear=true;break;}
                if(projectedClear)break;
            }
            // Strict projected clearance discarded the free-axis square, so
            // the same exact projected curve is clear with any axial range.
            if(projectedClear)continue;
        }
        // Cheap original-space rejection also handles finite floor triangles.
        C::FIntervalTensorSurface surface;
        if(!volume->GetScene()->CopySurfaceForQuery(index,budget,surface))return false;
        C::Detail::FNode root;
        root.DegreeU=surface.DegreeU;root.DegreeV=surface.DegreeV;
        for(const auto& point:surface.ControlPoints)root.Net.Add(point);
        if(!C::Detail::Bounds(root,centers,&r2))return false;
        if(root.LowerSquared>r2.Upper)continue;
        bool clear=false;
        for(const auto& fact:facts)
        {
            if(!early&&(!budget.Consume()||fact.Provider!=index||
                fact.CorrelationProofIdentity!=source->GetExactAffineProofIdentity()))return false;
            const auto& product=fact.Product;
            if(!Signed::Private::Admits(product))continue;
            if(early&&CSignedAffineNativeWholeRcWallSlices.GetValueOnAnyThread()==1)
            {
                const auto proof=TryWholeRcWallSlices(product,model,query,centers,literalLerpError,radius.Upper,budget);
                if(proof==Signed::EDecision::Clear)
                {
                    if(!fact.ExactProjectedCurveIdentity.IsEmpty())clearProjected.Add(&fact);
                    clear=true;break;
                }
                if(proof==Signed::EDecision::Work||proof==Signed::EDecision::Arithmetic)return false;
            }
            const int32 axis=product.FreeWorldAxis;
            if(!built[axis])
            {
                if(!Joint::BuildCenterNet(p,v,a,native.Horizon,radius.Upper,axis,nets[axis],
                    separatePaths?nullptr:&chordError))return false;
                if(separatePaths)
                {
                    // Degree-elevate the exact line independently of the native
                    // quadratic. Keep the same origin and radius for both proofs.
                    auto& literal=literalNets[axis];
                    literal=nets[axis];
                    for(int32 d=0;d<2;++d)
                    {
                        const int32 coordinate=d?2:1-axis;
                        const auto origin=d?I::Exact(radius.Upper):literal.TravelOrigin;
                        const auto first=A::Subtract(I::Exact(query.Start[coordinate]),origin);
                        const auto last=A::Subtract(I::Exact(query.End[coordinate]),origin);
                        const auto error=literalLerpError[coordinate];
                        const I pad{-error,error,true};
                        literal.Axis[d][0]=A::Add(first,pad);
                        literal.Axis[d][1]=A::Add(A::Multiply(I::Exact(.5),A::Add(first,last)),pad);
                        literal.Axis[d][2]=A::Add(last,pad);
                        for(const auto& control:literal.Axis[d])if(!A::Valid(control))return false;
                    }
                }
                built[axis]=true;
            }
            // Dropping the free-axis square can only lower distance; no
            // interior assumption or positive owner selection is needed.
            auto proof=Joint::Exclude(product,nets[axis],budget);
            if(separatePaths&&proof==Signed::EDecision::Clear)
                proof=Joint::Exclude(product,literalNets[axis],budget);
            if(proof==Signed::EDecision::Clear)
            {
                if(early&&!fact.ExactProjectedCurveIdentity.IsEmpty())clearProjected.Add(&fact);
                clear=true;break;
            }
            if(proof==Signed::EDecision::Work||proof==Signed::EDecision::Arithmetic)return false;
        }
        if(!clear)return false;
    }
    return RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush).ControlFingerprint()==
        numeric.ControlFingerprint();
}
}
bool FReferenceSphereGeneralFirstArrivalRs::TryEvaluate(FReferenceSphereStrictSeparationPtr initial,float horizon,
    const C::FReferenceSphereOptions& options,C::FDistanceBudget& budget,
    FReferenceSphereGeneralFirstArrivalRsPtr& out,FString& reason,C::FReferenceSphereEvaluation* partial)
{
    return TryEvaluateImpl(MoveTemp(initial),horizon,options,budget,out,reason,partial,nullptr,0,nullptr,nullptr,nullptr);
}
bool FReferenceSphereGeneralFirstArrivalRs::TryEvaluateWithFullRc(FReferenceSphereStrictSeparationPtr initial,float horizon,
    const C::FReferenceSphereOptions& options,const PreparedExecution::FSelectedWholeHullPartition& fullRc,
    int32 temporalReserve,C::FDistanceBudget& budget,FReferenceSphereGeneralFirstArrivalRsPtr& out,FString& reason,
    C::FReferenceSphereEvaluation* partial,const PreparedExecution::FNativeSolidStaticValues* native,
    const C::FReferenceBoundsCandidateSet* nativeVolume,bool* wholeRcClear,bool directedShortOnly)
{
    return TryEvaluateImpl(MoveTemp(initial),horizon,options,budget,out,reason,partial,&fullRc,temporalReserve,
        native,nativeVolume,wholeRcClear,directedShortOnly);
}
bool FReferenceSphereGeneralFirstArrivalRs::TryEvaluateNextEligible(
    const PreparedExecution::FNativeRepeatedSphereFloorExclusion& excluded,const C::FReferenceSphereOptions& options,
    const PreparedExecution::FSelectedWholeHullPartition& fullRc,int32 temporalReserve,
    C::FDistanceBudget& budget,FReferenceSphereGeneralFirstArrivalRsPtr& out,FString& reason)
{
    using E=C::EReferenceRoundedPrefix;
    out.Reset();reason=TEXT("Next eligible Sphere retained proof unavailable");
    const auto request=excluded.GetCurrentRequest();const auto* pairs=excluded.GetCurrentPairs();
    if(!request||!pairs||!excluded.IsCurrent(request,*pairs)||temporalReserve<=0||
        temporalReserve>budget.Remaining-budget.ConsumptionFloor)return false;
    const auto geometry=request->GetSnapshot()->GetGeometrySelection();
    if(!fullRc.UsesGeometry(geometry))return false;
    const auto& v=request->GetValues();const auto& state=v.OwnerState;
    FReferenceRepeatedSphereFloorPlan plan;
    const auto produce=[&](const C::FReferenceCandidateSetPtr& rs,C::FDistanceBudget& paid,C::FTensorSphereContactEnclosure& contact)
    {
        if(!plan.TryBuild(request,*pairs,excluded,rs,fullRc,paid)||!plan.TryReferenceContact(options,paid,contact))contact={};
        return true; // This operation has no alternate producer or inferred miss.
    };
    const auto prefix=[&](float end,C::FDistanceBudget& paid)
    {return plan.ProveConsumed(end,paid)?E::Clear:E::Refused;};
    const C::FReferenceContactProducer::FPrefixCallback prefixCall(prefix);
    const C::FReferenceContactProducer producer(produce,&prefixCall);
    const auto accepted=[](int32){return true;}; // Native admission checked every original row and contract.
    C::FReferenceSphereEvaluation ref;
    {
        C::FScopedDistanceBudgetFloor search(budget,budget.ConsumptionFloor+temporalReserve);
        if(!search.IsValid())return false;
        ref=C::EvaluateReferenceSphere(geometry->GetSelectedUnion(),state.Location,state.Velocity,state.Acceleration,
            v.Horizon,v.SupportRadius,double(v.PhysicalRadius),options,budget,accepted,&producer);
    }
    if(ref.Status!=C::EReferenceSphereStatus::ReferenceContactBounded||!ref.ConsumedPrefixClear||
        !ref.Candidates||ref.Candidates->GetScene()!=geometry->GetSelectedUnion()||ref.Contact.Leaves.IsEmpty()||
        ref.Contact.CollisionRadiusBits!=AnalyticCertified::Bits(v.SupportRadius))
    {
        reason=FString::Printf(TEXT("Next eligible Sphere search/prefix unavailable; Stage=%u Contact=%u TimeFailure=%u"),
            uint32(ref.Stage),uint32(ref.Contact.Status),uint32(ref.Contact.TimeFailure));return false;
    }
    if(!(ref.ConsumedSeconds>0.f&&ref.ConsumedSeconds<v.Horizon))return false;
    const auto fresh=Speed::SBox::AdvancePosition(state.Location,state.Velocity,state.Acceleration,ref.ConsumedSeconds);
    if(!plan.ProveFresh(ref.ConsumedSeconds,fresh,budget)||!excluded.IsCurrent(request,*pairs))
    {reason=TEXT("Next eligible Sphere fresh actual rounded point unavailable");return false;}
    TSharedPtr<FReferenceSphereGeneralFirstArrivalRs,ESPMode::ThreadSafe> value(new FReferenceSphereGeneralFirstArrivalRs);
    FStaticSphereInitialValues input;
    input.State=state;input.PhysicalRadius=v.PhysicalRadius;input.StaticContactOffset=v.StaticContactOffset;
    input.ActualMargin=v.SphereMargin;input.QueryRadius=v.QueryRadius;input.SupportRadius=v.SupportRadius;
    value->Filtered=FFilteredPoint{geometry,input,fresh,excluded.ExcludedOriginalProvider()};
    value->Reference=MoveTemp(ref);value->RcPartition=fullRc.Partition;value->Horizon=v.Horizon;
    out=MoveTemp(value);reason.Reset();return true;
}

bool FReferenceSphereGeneralFirstArrivalRs::TryEvaluateImpl(FReferenceSphereStrictSeparationPtr initial,float horizon,
    const C::FReferenceSphereOptions& options,C::FDistanceBudget& budget,
    FReferenceSphereGeneralFirstArrivalRsPtr& out,FString& reason,C::FReferenceSphereEvaluation* partial,
    const PreparedExecution::FSelectedWholeHullPartition* fullRc,int32 temporalReserve,
    const PreparedExecution::FNativeSolidStaticValues* native,const C::FReferenceBoundsCandidateSet* nativeVolume,bool* wholeRcClear,
    bool directedShortOnly)
{
    out.Reset();reason.Reset();if(partial)*partial={};RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    const auto refuse=[&](const TCHAR* why){reason=why;return false;};
    if(wholeRcClear)*wholeRcClear=false;
    const bool admitWholeRc=native&&nativeVolume&&wholeRcClear&&fullRc;
    if((native||nativeVolume||wholeRcClear)&&(!admitWholeRc||
        C::KinematicCenterDetail::Bits(native->Horizon)!=C::KinematicCenterDetail::Bits(horizon)))
        return refuse(TEXT("Complete native Rc miss input identity unavailable"));
    TOptional<C::FScopedDistanceBudgetFloor> searchFloor;
    if(fullRc)
    {
        if(temporalReserve<=0||temporalReserve>budget.Remaining-budget.ConsumptionFloor)
            return refuse(TEXT("General Sphere response work unavailable"));
        searchFloor.Emplace(budget,budget.ConsumptionFloor+temporalReserve);
        if(!searchFloor->IsValid())return refuse(TEXT("General Sphere response work unavailable"));
    }
    if(!initial||C::KinematicCenterDetail::Bits(initial->GetSeconds())!=0||!budget.Consume())
        return refuse(TEXT("FirstArrivalRs requires a separated retained point, not support/overlap"));
    const auto geometry=initial->GetGeometry();const auto& values=initial->GetCapturedInput();
    if(!geometry||!geometry->HasValidAliases()||C::KinematicCenterDetail::Bits(initial->GetHorizon())!=C::KinematicCenterDetail::Bits(horizon))
        return refuse(TEXT("General FirstArrivalRs captured geometry/horizon differs"));
    const auto& model=initial->GetModel();
    constexpr int32 proofAllowance=768;
    const bool tryWholeRcEarly=admitWholeRc&&CSignedAffineSphereResidual.GetValueOnAnyThread()==1&&
        CSignedAffineSphereRoundedClosure.GetValueOnAnyThread()==1&&CSignedAffineNativeWholeRcEarly.GetValueOnAnyThread()==1;

#if !UE_BUILD_SHIPPING
    const bool admitDirected=admitWholeRc&&CSignedC2DirectedWholeRc.GetValueOnAnyThread()==1&&
        CSignedC2VariableSphereTime.GetValueOnAnyThread()==1&&
        fullRc->UsesGeometry(geometry)&&
        AnalyticCertified::Bits(fullRc->Partition->GetRadiusForFixedSource())==AnalyticCertified::Bits(double(values.QueryRadius))&&
        nativeVolume->GetCandidateIndices().Num()<=16;
    const auto& directedPath=model.GetReferenceCurve();
    const auto directedHull=admitDirected?C::CenterDetail::Hull(directedPath):C::FIntervalPoint{};
    const bool directedApproach=admitDirected&&IsDirectedC2Approach(directedHull);
    const bool directedPostSeam=admitDirected&&IsPostSeamDirectedC2(directedHull,horizon);
    const bool directedShort=admitDirected&&IsShortDirectedC2Remainder(*native);
    const bool directedLate=admitDirected&&!directedApproach&&!directedPostSeam&&!directedShort&&
        IsLateDirectedC2Remainder(*native,directedHull);
    const bool directedFourthFull=admitDirected&&!directedApproach&&!directedPostSeam&&!directedShort&&
        IsExpandedFourthPanelFullVolume(horizon,directedHull);
    const bool directedThirdVolume=admitDirected&&!directedApproach&&!directedPostSeam&&!directedShort&&
        !directedLate&&!directedFourthFull&&
        IsDescendingFiniteDirectedVolume(values,*native,directedHull,nativeVolume->GetCandidateIndices(),false);
    const bool directedNextVolume=admitDirected&&!directedApproach&&!directedPostSeam&&!directedShort&&
        !directedLate&&!directedFourthFull&&!directedThirdVolume&&
        IsDescendingFiniteDirectedVolume(values,*native,directedHull,nativeVolume->GetCandidateIndices(),true);
    // Give the cheaper established High proof first refusal on its own cell.
    const bool deferDirectedShort=directedShort&&!directedApproach&&!directedPostSeam&&
        CSignedC2SphereTime.GetValueOnAnyThread()==1&&CSignedC2VariableRoundedPrefix.GetValueOnAnyThread()==1&&
        directedPath.Degree==2&&directedPath.ControlPoints.Num()==3&&
        C::SignedC2FiniteRival::ContainsPath(directedPath.ControlPoints[0],directedPath.ControlPoints[1],directedPath.ControlPoints[2]);
    const auto tryDirected=[&](C::FDistanceBudget& account)
    {
        if(account.Remaining-account.ConsumptionFloor<proofAllowance)return false;
        const uint64 before=account.Used;
        bool clear=false;
        {
            C::FScopedDistanceBudgetFloor optional(account,account.Remaining-proofAllowance);
            if(optional.IsValid())
                clear=ProveNativeWholeRcFromVolume(geometry,model,values,*native,nativeVolume,account,false,false,nullptr,true);
        }
        UE_LOG(LogTemp,Display,TEXT("[NativeSphereC2DirectedWholeRc] Frame=%u Clear=%d Candidates=%d Used=%llu ShortRemainder=%d PostSeam=%d"),
            native->FrameClock.OwnerLocalFrame,int32(clear),nativeVolume->GetCandidateIndices().Num(),account.Used-before,
            int32(directedShort),int32(directedPostSeam));
        if(directedLate)
        {
            UE_LOG(LogTemp,Display,TEXT("[NativeSphereC2DirectedLateRemainder] Frame=%u H=%.17g Clear=%d Candidates=%d Used=%llu"),
                native->FrameClock.OwnerLocalFrame,double(native->Horizon),int32(clear),
                nativeVolume->GetCandidateIndices().Num(),account.Used-before);
        }
        if(directedFourthFull)
        {
            UE_LOG(LogTemp,Display,TEXT("[NativeSphereC2DirectedFourthFull] Frame=%u Clear=%d Candidates=%d Used=%llu"),
                native->FrameClock.OwnerLocalFrame,int32(clear),nativeVolume->GetCandidateIndices().Num(),account.Used-before);
        }
        if(directedThirdVolume)
        {
            UE_LOG(LogTemp,Display,TEXT("[NativeSphereC2DirectedThirdVolume] Frame=%u H=%.17g Clear=%d Candidates=%d Used=%llu"),
                native->FrameClock.OwnerLocalFrame,double(native->Horizon),int32(clear),
                nativeVolume->GetCandidateIndices().Num(),account.Used-before);
        }
        if(directedNextVolume)
        {
            UE_LOG(LogTemp,Display,TEXT("[NativeSphereC2DirectedNextVolume] Frame=%u H=%.17g Clear=%d Candidates=%d Used=%llu"),
                native->FrameClock.OwnerLocalFrame,double(native->Horizon),int32(clear),
                nativeVolume->GetCandidateIndices().Num(),account.Used-before);
        }
        return clear;
    };
    if(directedShortOnly)
    {
        // Same admitted directed branch, original account and proof.
        // This mode cannot reach the generic search or construct a hit.
        if(!directedShort||deferDirectedShort||!tryDirected(budget))return false;
        *wholeRcClear=true;
        return true;
    }
    if((directedApproach||directedPostSeam||directedLate||directedFourthFull||directedThirdVolume||directedNextVolume||
        (directedShort&&!deferDirectedShort))&&tryDirected(budget))
    {
        searchFloor.Reset();
        *wholeRcClear=true;
        return true;
    }
#endif
    if(directedShortOnly)return false;
    uint64 earlyRcWork=0;
    if(tryWholeRcEarly&&fullRc->UsesGeometry(geometry)&&
        AnalyticCertified::Bits(fullRc->Partition->GetRadiusForFixedSource())==AnalyticCertified::Bits(double(values.QueryRadius))&&
        budget.Remaining-budget.ConsumptionFloor>=proofAllowance)
    {
        const uint64 before=budget.Used;
        bool clear=false;
        {
            C::FScopedDistanceBudgetFloor optional(budget,budget.Remaining-proofAllowance);
            if(optional.IsValid())clear=ProveNativeWholeRcFromVolume(geometry,model,values,*native,nativeVolume,budget,true);
        }
        if(clear)
        {
            searchFloor.Reset();
            *wholeRcClear=true;
            return true; // Stronger full Rc result covers the smaller exact Rs path as well.
        }
        earlyRcWork=budget.Used-before;
    }
    // Complete union, no post-contact leaf deletion and no material/Source
    // claim: all geometry participates in this mathematical computation.
    const auto& source=geometry->GetReference();
    const FCapturedSignedSphereMotion motion(values.State.Location,values.State.Velocity,values.State.Acceleration,horizon);
    bool signedReferenceContact=false,signedReferenceNoContact=false,usedSignedRoundedPrefix=false,provedWholeRc=false;
    uint64 splitRcWork=0;
    bool splitRcAttempted=false;
    FReferenceAffineContactPlanPtr roundedPartition;
    SignedAffineSphereNativeClosure::FPrefixObservation roundedPrefixObservation;
#if !UE_BUILD_SHIPPING
    const bool diagonal698=IsDiagonal698Step(native,values,horizon);
    C::FReferenceCandidateSetPtr generic698;
    uint32 finiteWallSelectedBits=0;
    const bool timeDiagonal698=diagonal698&&CNativeSphereDiagonal698SearchTiming.GetValueOnAnyThread()==1;
    uint64 prefixStart698=0,prefixEnd698=0,prefixWork698=0;
    uint32 prefixCalls698=0;
#endif
    const bool tryAffine=CSignedAffineSphereResidual.GetValueOnAnyThread()==1&&bool(source);
    const bool tryC2=admitWholeRc&&CSignedC2SphereTime.GetValueOnAnyThread()==1;
    FReferenceSphereC2ContactPlan::FObservation c2;
    FReferenceSphereC2Prefix c2Rounded;
    bool c2RoundedTried=false,c2RoundedBuilt=false;
    float c2RoundedEnd=0.f;
#if !UE_BUILD_SHIPPING
    PostSeamC2::FProof postSeamProof;
    uint32 postSeamEndBits=0;
    C::FReferenceCandidateSetPtr fixedRoundedCandidates,variableRoundedCandidates;
    uint32 variableRoundedSelectedBits=0;
    bool variableSignedNoContact=false;
    TOptional<FVector2d> variableRoundedLaterUV;
    TOptional<VariableC2Time::FNativeProposals> variableRoundedBroadUV;
    bool variableWholeRcPreproved=false;
    uint32 fixedRoundedEndBits=0;
#endif
    using PrefixTiming=SignedAffineSphereNativeClosure::FPrefixTiming;
    using PrefixSample=SignedAffineSphereNativeClosure::FScopedPrefixTiming;
    PrefixTiming* residualTiming=nullptr;
    bool timeMidpoint799=false;
#if !UE_BUILD_SHIPPING
    TOptional<PrefixTiming> residual799;
    static thread_local bool observed799=false;
    const bool timing799=!observed799&&CNativeSphere799ReferenceResidualTiming.GetValueOnAnyThread()==1;
    const bool midpoint799=CNativeSphere799TimeMidpoint.GetValueOnAnyThread()==1;
    if((timing799||midpoint799)&&admitWholeRc&&source&&
        source->GetExternalPacketIdentity()==TEXT("090d41158d13f1127827b720ff467a51e415668aa205209a67ed8afcb6b10dc0")&&
        native->FrameClock.CanonicalFrame==5359&&native->FrameClock.OwnerLocalFrame==5360&&
        native->OwnerStableId==10&&native->BodyIndex==0&&C::KinematicCenterDetail::Bits(horizon)==0x3b5a740e&&
        motion.Position==FVector3d(4956.6419417560528,-2091.580078125,93.150000000000006)&&
        motion.Velocity==FVector3d(1352.29,0.,0.)&&motion.Acceleration==FVector3d(-268.068689082351,0.,0.))
    {
        PreparedExecution::FStepIdentity step;
        if(PreparedExecution::ReadCurrentStep(step)&&step.Frame==5359&&step.StepSerial==5360&&
            step.QuerySerial==1&&step.QueryEpoch==2&&step.Iteration==1&&step.NumericControl==0x0000027f00001f80ull)
        {
            timeMidpoint799=midpoint799;
            if(timing799)
            {
                observed799=true;
                residualTiming=&residual799.Emplace();
            }
        }
    }
#endif
    const auto produce=[&](const C::FReferenceCandidateSetPtr& candidates,C::FDistanceBudget& account,
        C::FTensorSphereContactEnclosure& out)
    {
        PrefixSample produceSample(residualTiming,PrefixTiming::Produce,account);
        using E=C::AffineReferenceDetail::EOutcome;
        FReferenceAffineContactPlanPtr plan;
        const auto status=tryAffine?FReferenceAffineContactPlan::Build(source,candidates,account,plan,nullptr,0,false):
            E::NotApplicable;
        if(status==E::NotApplicable)
        {
#if !UE_BUILD_SHIPPING
            if(diagonal698)generic698=candidates;
#endif
            if(tryC2&&candidates&&FReferenceSphereC2ContactPlan::TryReferenceContact(*initial,*candidates,options,account,out,c2))
                return true;
#if !UE_BUILD_SHIPPING
            if(tryC2&&CSignedC2PostSeamRs.GetValueOnAnyThread()==1&&fullRc&&candidates)
            {
                const auto attempt=[&](bool thirdPanel,bool fourthPanel=false)
                {
                    if((thirdPanel||fourthPanel)&&(!native||!nativeVolume))return false;
                    PostSeamC2::FProof candidateProof;
                    const bool nextPrefix=CSignedC2PostSeamNextPanelPrefix.GetValueOnAnyThread()==1;
                    if(!candidateProof.Bind(*initial,*candidates,*fullRc,account,
                        CSignedC2NaturalSeamRs.GetValueOnAnyThread()==1,thirdPanel,fourthPanel,native&&nativeVolume&&nextPrefix))return false;
                    uint32 candidateBits=0;
                    const uint64 before=account.Used;
                    bool tryLaterFourthPanelCap=false;
                    const bool wallLip=fourthPanel&&native&&IsWallLipRsScope(*native);
                    bool rivals=wallLip?ProveOriginalWallLipRsRivals(*initial,*candidates,*fullRc,*nativeVolume,*native,candidateProof,account):
                        ProvePostSeamRivals(*initial,*candidates,*nativeVolume,candidateProof,account,
                            (thirdPanel||fourthPanel||candidateProof.IsDescendingNextPanel())?native:nullptr,
                            false,nullptr,fourthPanel?&tryLaterFourthPanelCap:nullptr);
                    if(!thirdPanel&&!fourthPanel&&!candidateProof.IsDescendingNextPanel()&&!rivals&&
                        nextPrefix&&candidateProof.TryLimitNextPanel(account))
                    {
                        const uint64 shortBefore=account.Used;
                        rivals=ProvePostSeamRivals(*initial,*candidates,*nativeVolume,candidateProof,account);
                        UE_LOG(LogTemp,Display,TEXT("[NativeSphereC2NextPanelPrefix] Frame=%u Provider=%d Limit=%.17g Rivals=%d Used=%llu"),
                            native->FrameClock.OwnerLocalFrame,candidateProof.GlobalProvider(),candidateProof.EndLimit(),
                            int32(rivals),account.Used-shortBefore);
                    }
                    if(!thirdPanel&&!candidateProof.IsDescendingNextPanel()&&!rivals&&
                        candidateProof.GlobalProvider()==2375&&candidateProof.EndLimit()<1.&&
                        CSignedC2NextPanelTighterCap.GetValueOnAnyThread()==1&&
                        nativeVolume->GetCandidateIndices().Contains(1098))
                    {
                        const uint64 shortBefore=account.Used;
                        if(candidateProof.TryTightenNextPanel(account))
                        {
                            rivals=ProvePostSeamRivals(*initial,*candidates,*nativeVolume,candidateProof,account,native,true);
                            UE_LOG(LogTemp,Display,TEXT("[NativeSphereC2NextPanelTighterCap] Frame=%u Provider=2375 Limit=%.17g Rivals=%d Used=%llu CommonChord=%d"),
                                native->FrameClock.OwnerLocalFrame,candidateProof.EndLimit(),int32(rivals),account.Used-shortBefore,int32(rivals));
                        }
                    }
                    if(candidateProof.IsDescendingNextPanel()&&!rivals&&nextPrefix&&
                        CSignedC2DescendingNextPanelPrefix.GetValueOnAnyThread()==1)
                    {
                        const auto pair=[](TConstArrayView<int32> indices)
                        {
                            return indices.Num()==2&&indices.Contains(1098)&&indices.Contains(2375);
                        };
                        const uint64 prefixBefore=account.Used;
                        if(pair(nativeVolume->GetCandidateIndices())&&pair(candidates->GetCandidateIndices())&&
                            pair(fullRc->Partition->GetCandidateIndices())&&candidateProof.TryLimitNextPanel(account))
                        {
                            // The cap comes from strict reference Rs inside.
                            // The original helper independently requires native
                            // and literal Rc inside, and proves the full rival
                            // cover at Rc + 3 Domain before the Rs solver runs.
                            rivals=ProvePostSeamRivals(*initial,*candidates,*nativeVolume,candidateProof,account,native,true);
                            UE_LOG(LogTemp,Display,TEXT("[NativeSphereC2DescendingNextPanelPrefix] Frame=%u Provider=2375 Rival=1098 Cap=%.17g Proved=%d Used=%llu"),
                                native->FrameClock.OwnerLocalFrame,candidateProof.EndLimit(),int32(rivals),account.Used-prefixBefore);
                        }
                    }
                    if(thirdPanel&&!rivals&&!candidateProof.IsDescendingThirdPanel()&&
                        CSignedC2ThirdPanelSeparateOrder.GetValueOnAnyThread()==1&&
                        nativeVolume->GetCandidateIndices().Contains(870))
                    {
                        FThirdPanelSeparateOrder order;
                        const uint64 orderBefore=account.Used;
                        rivals=ProvePostSeamRivals(*initial,*candidates,*nativeVolume,candidateProof,account,native,false,&order)&&
                            candidateProof.SetThirdPanelPhysicalCap(order.PhysicalEnd,account);
                        UE_LOG(LogTemp,Display,TEXT("[NativeSphereC2ThirdPanelSeparateOrder] Frame=%u Provider=1098 Rival=870 PhysicalCap=%.17g LiteralCap=%.17g Proved=%d Used=%llu"),
                            native->FrameClock.OwnerLocalFrame,order.PhysicalEnd,order.LiteralEnd,int32(rivals),account.Used-orderBefore);
                    }
                    if(fourthPanel)
                    {
                        // Keep 7/8 first. Try a later cap only if one of the
                        // original inside witnesses did not close; a refused
                        // rival proof cannot justify moving the cap.
                        for(int32 retry=0;!rivals&&tryLaterFourthPanelCap&&retry<2;++retry)
                        {
                            if(!candidateProof.TryRaiseFourthPanelCap(account))break;
                            rivals=ProvePostSeamRivals(*initial,*candidates,*nativeVolume,candidateProof,account,
                                native,false,nullptr,&tryLaterFourthPanelCap);
                        }
                    }
                    const bool separateFourth=fourthPanel&&!rivals&&!wallLip&&IsDescendingFourthPanelSeparateOrderScope(*native)&&
                        ProveOriginalFourthPanelSeparateOrder(*initial,*candidates,*fullRc,*nativeVolume,*native,candidateProof,account);
                    rivals=rivals||separateFourth;
                    if(fourthPanel&&rivals&&!wallLip&&!separateFourth)
                    {
                        UE_LOG(LogTemp,Display,TEXT("[NativeSphereC2FourthPanelCap] Frame=%u Provider=870 Cap=%.17g RsInside=1 NativeRcInside=1 ChordRcInside=1 Rivals=1 Used=%llu"),
                            native->FrameClock.OwnerLocalFrame,candidateProof.EndLimit(),account.Used-before);
                    }
                    const bool naturalOrder=!candidateProof.HasSeparateOrder()||(rivals&&candidateProof.ProveNaturalOrder(account));
                    C::ContactDetail::FInteriorTensorContraction contractor(candidateProof.LocalProvider());
                    auto* spatial=CSignedC2InteriorSphereContraction.GetValueOnAnyThread()==1?&contractor:nullptr;
                    const bool closed=rivals&&naturalOrder&&FReferenceSphereC2ContactPlan::TryReferencePostSeam(
                        *initial,*candidates,options,account,candidateProof,out,spatial,candidateBits);
                    UE_LOG(LogTemp,Display,TEXT("[NativeSphereC2PostSeamRs] Frame=%u Rivals=%d Closed=%d Status=%u Failure=%u TimeFailure=%u EndBits=%08x Used=%llu Provider=%d"),
                        native->FrameClock.OwnerLocalFrame,int32(rivals),int32(closed),uint32(out.Status),uint32(out.Failure),
                        uint32(out.TimeFailure),candidateBits,account.Used-before,candidateProof.GlobalProvider());
                    if(candidateProof.IsNatural())
                    {
                        UE_LOG(LogTemp,Display,TEXT("[NativeSphereC2NaturalSeamOrder] Frame=%u Order=%d Closed=%d EndBits=%08x"),
                            native->FrameClock.OwnerLocalFrame,int32(naturalOrder),int32(closed),candidateBits);
                    }
                    if(candidateProof.IsSibling1940())
                    {
                        UE_LOG(LogTemp,Display,TEXT("[NativeSphereC2Sibling1940Order] Frame=%u Rival=2375 Order=%d Closed=%d EndBits=%08x"),
                            native->FrameClock.OwnerLocalFrame,int32(naturalOrder),int32(closed),candidateBits);
                    }
                    if(closed)
                    {
                        postSeamProof=MoveTemp(candidateProof);
                        postSeamEndBits=candidateBits;
                        return true;
                    }
                    out={};
                    return false;
                };
                if(attempt(false))return true;
                if(CSignedC2PostSeamThirdPanel.GetValueOnAnyThread()==1&&attempt(true))return true;
                if(CSignedC2PostSeamFourthPanel.GetValueOnAnyThread()==1&&attempt(false,true))return true;
            }
            if(tryC2&&CSignedC2VariableSphereTime.GetValueOnAnyThread()==1&&fullRc&&candidates)
            {
                FReferenceSphereC2ContactPlan::FVariableTimeObservation observed;
                const uint64 before=account.Used;
                const bool prepareNative=CSignedC2VariableRoundedPrefix.GetValueOnAnyThread()==1||
                    CSignedC2VariableFreshPoint.GetValueOnAnyThread()==1;
                // Authenticated variable geometry retains 695 at local slot 1.
                // Only the new signed chart lanes may use this opt-in contractor.
                C::ContactDetail::FInteriorTensorContraction broadSpatial(1);
                auto* spatial=CSignedC2InteriorSphereContraction.GetValueOnAnyThread()==1?&broadSpatial:nullptr;
                const FReferenceSphereC2ContactPlan::FSeamSideProof seamSide(ProveSeamRsSides);
                const bool tryHighShort=deferDirectedShort&&CSignedC2HighShortPreclear.GetValueOnAnyThread()==1;
                PreparedExecution::FStepIdentity wholeStep;
                const bool tryHighBroad=admitDirected&&CSignedC2VariableRoundedPrefix.GetValueOnAnyThread()==1&&
                    CSignedC2HighBroadPreclear.GetValueOnAnyThread()==1&&horizon>0.f&&horizon<=1.f/300.f&&
                    PreparedExecution::ReadCurrentStep(wholeStep)&&wholeStep.Frame==native->FrameClock.CanonicalFrame&&
                    wholeStep.QuerySerial!=0&&C::KinematicCenterDetail::Bits(wholeStep.Remaining)==C::KinematicCenterDetail::Bits(horizon);
                const auto preproveWhole=[&](const VariableC2Time::FNativeProposals& proposals,C::FDistanceBudget& proofBudget)
                {
                    const int32 allowance=proposals.HighPrefixOnly?96:64;
                    if((!tryHighBroad&&!(tryHighShort&&proposals.HighPrefixOnly))||
                        proofBudget.Remaining-proofBudget.ConsumptionFloor<allowance)return false;
                    C::FScopedDistanceBudgetFloor bounded(proofBudget,proofBudget.Remaining-allowance);
                    const FNativeBroadC2WholeRc broad{*initial,*candidates,*fullRc,proposals};
                    return bounded.IsValid()&&ProveNativeWholeRcFromVolume(geometry,model,values,*native,nativeVolume,
                        proofBudget,false,false,&broad);
                };
                const FReferenceSphereC2ContactPlan::FPreprovedWholeRc wholeProof(preproveWhole);
                static thread_local uint32 timingReports=0;
                const bool measure=timingReports<48&&CSignedSphereTimeAttribution.GetValueOnAnyThread()==1;
                C::TimeDetail::FSignedTimeCostObservation timeCosts;
                timeCosts.Account=&account;
                const auto produceVariable=[&]{
                    return FReferenceSphereC2ContactPlan::TryReferenceVariableTime(*initial,*candidates,*fullRc,
                        options,account,out,observed,prepareNative,spatial,
                        CSignedC2SeamRs.GetValueOnAnyThread()==1?&seamSide:nullptr,
                        tryHighShort||tryHighBroad?&wholeProof:nullptr,tryHighBroad,tryHighShort);
                };
                const bool produced=[&]{
                    if(!measure)return produceVariable();
                    C::TimeDetail::FScopedSignedTimeCosts observe(&timeCosts);
                    return produceVariable();
                }();
                if(measure&&timeCosts.HasSamples())
                {
                    ++timingReports;
                    static constexpr const TCHAR* names[]={TEXT("Initial"),TEXT("Coarse"),TEXT("Preclear"),
                        TEXT("Signed"),TEXT("Endpoint"),TEXT("Whole")};
                    static_assert(UE_ARRAY_COUNT(names)==C::TimeDetail::FSignedTimeCostObservation::RowCount);
                    FString rows;
                    for(int32 i=0;i<UE_ARRAY_COUNT(names);++i)
                    {
                        const auto& row=timeCosts.Rows[i];
                        rows+=FString::Printf(TEXT(" %s=%llu/%llu/%llu"),names[i],row.Calls,row.Ticks,row.Work);
                    }
                    UE_LOG(LogTemp,Display,TEXT("[NativeSphereSignedTimeAttribution] Frame=%u H=%.17g Status=%u Later=%d Broad=%d High=%d SecPerCycle=%.17g InsideProposals=%llu InsideReused=%llu CallsTicksWork=1%s WholeHullLookups=%llu WholeHullHits=%llu WholeHullStores=%llu"),
                        native->FrameClock.OwnerLocalFrame,double(horizon),uint32(out.Status),int32(observed.Later),
                        int32(observed.Broad),int32(observed.HighPrefixOnly),FPlatformTime::GetSecondsPerCycle64(),
                        timeCosts.InsidePublished,timeCosts.InsideReused,*rows,
                        timeCosts.WholeHullLookups,timeCosts.WholeHullHits,timeCosts.WholeHullStores);
                }
                if(produced)
                {
                    if(observed.NativeProposals.IsSet())
                    {
                        variableRoundedCandidates=candidates;
                        variableRoundedSelectedBits=observed.NativeEndBits;
                        variableRoundedBroadUV=observed.NativeProposals;
                        variableSignedNoContact=(observed.Broad||observed.HighPrefixOnly)&&
                            out.Status==C::ETensorSphereContactStatus::NoContactWithProvidedTensors;
                        variableWholeRcPreproved=observed.WholeRcPreproved;
                    }
                    else if(!observed.Broad&&!observed.ExtendedEarly&&!observed.HighPrefixOnly&&!observed.SeamRs&&prepareNative&&observed.Clear&&
                        out.Status==C::ETensorSphereContactStatus::FirstContactEnclosed&&
                        !out.Leaves.IsEmpty()&&out.Leaves[0].SurfaceIndex==695&&account.Consume())
                    {
                        const auto selected=C::FloorReferenceTimeToFloatSeconds(initial->GetModel(),out.LowerTime);
                        if(selected.Known&&selected.Seconds>0.f)
                        {
                            variableRoundedCandidates=candidates;
                            variableRoundedSelectedBits=C::KinematicCenterDetail::Bits(selected.Seconds);
                            if(observed.Later)variableRoundedLaterUV=out.Leaves[0].FixedParameter;
                        }
                    }
                    UE_LOG(LogTemp,Display,TEXT("[NativeSphereC2VariableTime] Frame=%u Attempts=%u Clear=%u LastClear=[%.17g,%.17g] ScaledLower=%.17g Status=%u Failure=%u Used=%llu Broad=%d BroadEndBits=%08x Early=%d EarlyEndBits=%08x HighPrefixOnly=%d HighEndBits=%08x HighPreClear=%u HighPreClearUsed=%llu BroadPreClear=%u BroadPreClearUsed=%llu Later=%d LaterPreClear=%u LaterPreClearUsed=%llu"),
                        native->FrameClock.OwnerLocalFrame,observed.Attempts,observed.Clear,observed.First,observed.Last,
                        observed.ScaledLower,uint32(out.Status),uint32(out.Failure),account.Used-before,
                        int32(observed.Broad),observed.Broad?observed.NativeEndBits:0u,
                        int32(observed.ExtendedEarly),observed.ExtendedEarly?observed.NativeEndBits:0u,
                        int32(observed.HighPrefixOnly),observed.HighPrefixOnly?observed.NativeEndBits:0u,
                        observed.HighPreClear,observed.HighPreClearUsed,observed.BroadPreClear,observed.BroadPreClearUsed,
                        int32(observed.Later),observed.LaterPreClear,observed.LaterPreClearUsed);
                    if(observed.SeamRs)
                        UE_LOG(LogTemp,Display,TEXT("[NativeSphereC2SeamRs] Frame=%u Status=%u Failure=%u EndBits=%08x PreClear=%u PreClearUsed=%llu"),
                            native->FrameClock.OwnerLocalFrame,uint32(out.Status),uint32(out.Failure),observed.NativeEndBits,
                            observed.SeamPreClear,observed.SeamPreClearUsed);
                    if((observed.Later||observed.Broad||observed.HighPrefixOnly||observed.SeamRs)&&spatial&&broadSpatial.Attempts)
                    {
                        UE_LOG(LogTemp,Display,TEXT("[NativeSphereC2BroadSpatial] Frame=%u Provider=695 Attempts=%u Excluded=%u PrunedChildren=%u Status=%u Failure=%u SpatialCells=%u Later=%d"),
                            native->FrameClock.OwnerLocalFrame,broadSpatial.Attempts,broadSpatial.Excluded,
                            broadSpatial.PrunedChildren,uint32(out.Status),uint32(out.Failure),out.SpatialVisitedCells,int32(observed.Later));
                    }
                    return true;
                }
            }
            if(CSignedC2InteriorSphereContraction.GetValueOnAnyThread()==1&&admitWholeRc&&fullRc&&
                geometry->GetModel()==EPreparedGeometryModel::RetainedReference&&
                fullRc->UsesGeometry(geometry)&&source&&candidates&&candidates->GetScene()==geometry->GetSelectedUnion()&&
                source->GetExternalPacketIdentity()==TEXT("090d41158d13f1127827b720ff467a51e415668aa205209a67ed8afcb6b10dc0"))
            {
                const auto rs=candidates->GetCandidateIndices(),rc=fullRc->Partition->GetCandidateIndices();
                const auto& p=values.State.Location;const auto& v=values.State.Velocity;const auto& a=values.State.Acceleration;
                // The spatial proof is geometric. This initial scope is the
                // signed backwall chart and the observed rising-ball corridor.
                if(rs.Num()==4&&rs[0]==135&&rs[1]==695&&rs[2]==1940&&rs[3]==2375&&
                    rc.Num()==4&&rc[0]==135&&rc[1]==695&&rc[2]==1940&&rc[3]==2375&&
                    p.X>4963.5&&p.X<4964.&&p.Y>-1339.25&&p.Y<-1338.75&&p.Z>93.17&&p.Z<93.18&&
                    v.X>660.&&v.X<675.&&v.Y>-185.&&v.Y<-175.&&v.Z>5.&&v.Z<6.&&
                    a.X>-25.&&a.X<-15.&&a.Y>3.&&a.Y<8.&&a.Z>-655.&&a.Z<-645.&&
                    C::KinematicCenterDetail::Bits(horizon)==C::KinematicCenterDetail::Bits(1.f/300.f)&&
                    AnalyticCertified::Bits(candidates->GetRadiusForFixedSource())==AnalyticCertified::Bits(values.SupportRadius)&&
                    AnalyticCertified::Bits(fullRc->Partition->GetRadiusForFixedSource())==AnalyticCertified::Bits(double(values.QueryRadius)))
                {
                    C::ContactDetail::FInteriorTensorContraction proof(1);
                    const uint64 before=account.Used;
                    out=candidates->Contact(account,options.TimeGap,options.SpatialGap,options.SpatialNodesPerAttempt,
                        options.PointWidth,options.NormalWidth,24,nullptr,nullptr,&proof);
                    if(out.Status==C::ETensorSphereContactStatus::FirstContactEnclosed&&proof.Attempts&&proof.PrunedChildren)
                        fixedRoundedCandidates=candidates;
                    UE_LOG(LogTemp,Display,TEXT("[NativeSphereC2InteriorContraction] Provider=695 Attempts=%u Excluded=%u PrunedChildren=%u Status=%u Failure=%u Used=%llu"),
                        proof.Attempts,proof.Excluded,proof.PrunedChildren,uint32(out.Status),uint32(out.Failure),account.Used-before);
                    return true;
                }
            }
            if(admitWholeRc&&CNativeSphereFiniteWallQuadraticRs.GetValueOnAnyThread()==1&&
                AdmitFiniteWallQuadraticRs(geometry,values,model,*native,*nativeVolume,*fullRc,candidates,account))
            {
                // The existing solver, retained dyadic frontier and point/
                // normal materializer own the result. The proved roster can
                // only support a separately proved actual rounded prefix.
                const auto originalSign=[](double,double,C::FDistanceBudget&)
                    {return C::TimeDetail::FSignedTimeInterval{};};
                const C::TimeDetail::FSignedTimeQuery query(originalSign,nullptr,true);
                out=candidates->Contact(account,options.TimeGap,options.SpatialGap,options.SpatialNodesPerAttempt,
                    options.PointWidth,options.NormalWidth,24,nullptr,nullptr,nullptr,&query);
                if(out.Status==C::ETensorSphereContactStatus::FirstContactEnclosed)
                {
                    const auto floor=C::FloorReferenceTimeToFloatSeconds(model,out.LowerTime);
                    if(floor.Known)finiteWallSelectedBits=C::KinematicCenterDetail::Bits(floor.Seconds);
                }
                UE_LOG(LogTemp,Display,TEXT("[NativeSphereFiniteWallQuadraticRs] Frame=%u Provider=2693 Status=%u Failure=%u TimeFailure=%u Lower=%.17g Upper=%.17g TimeWork=%llu Spatial=%llu"),
                    native->FrameClock.OwnerLocalFrame,uint32(out.Status),uint32(out.Failure),uint32(out.TimeFailure),
                    out.LowerTime,out.UpperTime,out.TimeWorkUsed,out.SpatialVisitedCells);
                return true;
            }
            if(diagonal698&&native&&native->BodyIndex==0&&
                native->Query.DomainTolerance==1.e-6&&native->Query.InitialOverlapTolerance==.05&&
                CNativeSphereDiagonal698ReferenceWindow.GetValueOnAnyThread()==1)
            {
                using FQuery=C::TimeDetail::FSignedTimeQuery;
                FQuery::FDyadicHintStats stats;
                bool attempted=false,proved=false;
                uint64 proofWork=0;
                const auto originalSign=[](double,double,C::FDistanceBudget&)
                    {return C::TimeDetail::FSignedTimeInterval{};};
                const auto proveWindow=[&](double& lower,double& upper,C::FDistanceBudget& current)
                {
                    if(&current!=&account)return false;
                    attempted=true;
                    const uint64 before=current.Used;
                    proved=TryDiagonal698ReferenceClear(geometry,values,fullRc,candidates,current,true);
                    proofWork=current.Used-before;
                    if(!proved)return false;
                    lower=Diagonal698WindowClear;upper=Diagonal698WindowInside;
                    return true;
                };
                const FQuery::FDyadicHintCallback hint(proveWindow);
                const FQuery query(originalSign,nullptr,false,&hint,&stats);
                out=candidates->Contact(account,options.TimeGap,options.SpatialGap,options.SpatialNodesPerAttempt,
                    options.PointWidth,options.NormalWidth,24,nullptr,nullptr,nullptr,&query);
                if(attempted)
                    UE_LOG(LogTemp,Display,TEXT("[NativeSphereDiagonal698ReferenceWindow] Frame=%u Proved=%d Lower=%.17g Upper=%.17g Used=%llu PrefixSkips=%llu SplitSkips=%llu Status=%u TimeWork=%llu"),
                        native->FrameClock.OwnerLocalFrame,int32(proved),Diagonal698WindowClear,Diagonal698WindowInside,
                        proofWork,stats.PrefixSkips,stats.SplitSkips,uint32(out.Status),out.TimeWorkUsed);
                return true;
            }
            if(diagonal698&&CNativeSphereDiagonal698ReferenceClear.GetValueOnAnyThread()==1)
            {
                bool attempted=false,proved=false;
                uint32 skips=0;
                uint64 proofWork=0;
                const auto originalSign=[](double,double,C::FDistanceBudget&)
                    {return C::TimeDetail::FSignedTimeInterval{};};
                const auto clearLeft=[&](double first,double last,C::FDistanceBudget& current)
                {
                    if(&current!=&account||!(0.<=first&&first<last&&last<=.5))return false;
                    if(!attempted)
                    {
                        // The original direct dispatch already declined.
                        // A paid proof is needed only on a real eligible generic cell.
                        attempted=true;
                        const uint64 before=current.Used;
                        proved=TryDiagonal698ReferenceClear(geometry,values,fullRc,candidates,current);
                        proofWork=current.Used-before;
                    }
                    if(!proved||!current.Consume())return false;
                    ++skips;
                    return true;
                };
                const C::TimeDetail::FSignedTimeQuery::FClearCallback beforeCoarse(clearLeft);
                const C::TimeDetail::FSignedTimeQuery query(originalSign,&beforeCoarse);
                out=candidates->Contact(account,options.TimeGap,options.SpatialGap,options.SpatialNodesPerAttempt,
                    options.PointWidth,options.NormalWidth,24,nullptr,nullptr,nullptr,&query);
                if(proved)
                {
                    UE_LOG(LogTemp,Display,TEXT("[NativeSphereDiagonal698ReferenceClear] Frame=%u Clear=1 Lower=0 Upper=0.5 Used=%llu Skips=%u Status=%u TimeWork=%llu"),
                        native->FrameClock.OwnerLocalFrame,proofWork,skips,uint32(out.Status),out.TimeWorkUsed);
                }
                else if(attempted)
                {
                    UE_LOG(LogTemp,Display,TEXT("[NativeSphereDiagonal698ReferenceClear] Frame=%u Clear=0 Used=%llu"),
                        native->FrameClock.OwnerLocalFrame,proofWork);
                }
                return true;
            }
#endif
            return false;
        }
        if(status!=E::Complete||!plan)
        {
            out.Failure=account.Remaining<=account.ConsumptionFloor?
                C::ETensorSphereContactFailure::Work:C::ETensorSphereContactFailure::Input;
            out.WorkSite=TEXT("signed_affine_admission");
            return true;
        }
        const auto accept=[](const C::FTensorSphereContactEnclosure&){return C::EContactEnclosureDecision::Accept;};
        const C::FContactEnclosureFilter filter(accept);
        FReferenceAffineContactPlan::FScopedReferencePreclear* preclear=nullptr;
#if !UE_BUILD_SHIPPING
        FReferenceAffineContactPlan::FScopedReferencePreclear scopedPreclear;
        int32 capturedCase=0,capturedOrdinal=0;
        if(CSignedSphere632799AffinePreclear.GetValueOnAnyThread()==1&&admitWholeRc&&source&&candidates&&
            source->GetExternalPacketIdentity()==TEXT("090d41158d13f1127827b720ff467a51e415668aa205209a67ed8afcb6b10dc0")&&
            candidates->GetScene()==geometry->GetSelectedUnion()&&fullRc->UsesGeometry(geometry)&&
            nativeVolume->GetScene()==geometry->GetSelectedUnion()&&native->BodyIndex==0&&
            values.SupportRadius==93.149999976158142&&double(values.QueryRadius)==93.150100708007812&&
            native->Query.DomainTolerance==1.e-6&&native->Query.InitialOverlapTolerance==.05)
        {
            const auto rs=candidates->GetCandidateIndices(),rc=fullRc->Partition->GetCandidateIndices();
            const auto matchesRoster=[](TConstArrayView<int32> live,TConstArrayView<int32> expected)
            {
                return live.Num()==expected.Num()&&
                    FMemory::Memcmp(live.GetData(),expected.GetData(),SIZE_T(live.Num())*sizeof(int32))==0;
            };
            const auto sameVector=[](const FVector3d& a,const FVector3d& b)
            {
                using ::AnalyticCertified::Bits;
                return Bits(a.X)==Bits(b.X)&&Bits(a.Y)==Bits(b.Y)&&Bits(a.Z)==Bits(b.Z);
            };
            const auto matchesMotion=[&](uint32 bits,const FVector3d& p,const FVector3d& v,const FVector3d& a)
            {
                return C::KinematicCenterDetail::Bits(horizon)==bits&&sameVector(motion.Position,p)&&
                    sameVector(motion.Velocity,v)&&sameVector(motion.Acceleration,a);
            };
            static constexpr int32 rs632First[]={118,726,1931,2191},rs632Second[]={118,726,1931};
            static constexpr int32 rs799[]={1940,2375},rc799[]={1914,1940,2375};
            PreparedExecution::FStepIdentity step;
            if(PreparedExecution::ReadCurrentStep(step)&&step.Frame==native->FrameClock.CanonicalFrame&&
                step.StepSerial==native->FrameClock.OwnerLocalFrame&&step.NumericControl==0x0000027f00001f80ull&&
                C::KinematicCenterDetail::Bits(step.Remaining)==C::KinematicCenterDetail::Bits(horizon))
            {
                if(step.Frame==4395&&native->OwnerStableId==8&&step.QuerySerial==3&&step.QueryEpoch==4&&step.Iteration==2&&
                    matchesRoster(rs,MakeArrayView(rs632First))&&matchesRoster(rc,MakeArrayView(rs632First))&&
                    matchesMotion(0x3b59e45c,{14.944408288634424,3839.1291550372389,93.149999976158313},
                        {148.8199608654991,2381.5893737243919,-1.2005330063402653e-10},
                        {-4.5691239919549549,-73.120413976577851,0.}))
                {capturedCase=632;capturedOrdinal=1;scopedPreclear.Cap=7./64.;scopedPreclear.FreeAxis=0;}
                else if(step.Frame==4395&&native->OwnerStableId==8&&step.QuerySerial==5&&step.QueryEpoch==6&&step.Iteration==3&&
                    matchesRoster(rs,MakeArrayView(rs632Second))&&matchesRoster(rc,MakeArrayView(rs632Second))&&
                    matchesMotion(0x3b41e47f,{14.998906262575835,3840.0012947345808,93.165100915357598},
                        {148.81828763968556,2381.5625968270306,6.9809800606890349e-07},
                        {-4.5691239919549549,-73.120413976577851,2.9487038588627001e-08}))
                {capturedCase=632;capturedOrdinal=2;scopedPreclear.Cap=.75;scopedPreclear.FreeAxis=0;}
                else if(step.Frame==5359&&native->OwnerStableId==10&&step.QuerySerial==1&&step.QueryEpoch==2&&step.Iteration==1&&
                    matchesRoster(rs,MakeArrayView(rs799))&&matchesRoster(rc,MakeArrayView(rc799))&&
                    matchesMotion(0x3b5a740e,{4956.6419417560528,-2091.580078125,93.150000000000006},
                        {1352.29,0.,0.},{-268.068689082351,0.,0.}))
                {capturedCase=799;capturedOrdinal=1;scopedPreclear.Cap=.75;scopedPreclear.FreeAxis=1;}
            }
            if(capturedCase)preclear=&scopedPreclear;
        }
#endif
        const auto fixed=plan->Contact(account,options.TimeGap,options.SpatialGap,options.SpatialNodesPerAttempt,
            options.PointWidth,options.NormalWidth,filter,&motion,CSignedAffineReferenceTimeCorrelation.GetValueOnAnyThread()==1,preclear);
        const auto* contact=fixed?fixed->InspectForSameFixedSource(source,candidates):nullptr;
#if !UE_BUILD_SHIPPING
        if(preclear)
        {
            UE_LOG(LogTemp,Display,TEXT("[NativeSphere632799AffinePreclear] Case=%d Capture=%d Frame=%u HBits=%08x Cap=%.17g Attempted=%d Certified=%d Work=%llu AcceptedCells=%u ReusedCells=%u Status=%u Lower=%.17g Upper=%.17g TimeWork=%llu"),
                capturedCase,capturedOrdinal,native->FrameClock.OwnerLocalFrame,C::KinematicCenterDetail::Bits(horizon),
                scopedPreclear.Cap,int32(scopedPreclear.Attempted),int32(scopedPreclear.Certified),scopedPreclear.CertificateWork,
                scopedPreclear.AcceptedCells,scopedPreclear.ReusedCells,contact?uint32(contact->Status):0u,
                contact?contact->LowerTime:0.,contact?contact->UpperTime:0.,contact?contact->TimeWorkUsed:0ull);
        }
#endif
        if(contact)
        {
            out=*contact;
            signedReferenceContact=out.Status==C::ETensorSphereContactStatus::FirstContactEnclosed;
            signedReferenceNoContact=out.Status==C::ETensorSphereContactStatus::NoContactWithProvidedTensors;
        }
        else
        {
            out.Failure=C::ETensorSphereContactFailure::Input;
            out.WorkSite=TEXT("signed_affine_transport");
        }
        return true;
    };
    const auto proveRounded=[&](float end,C::FDistanceBudget& account)->C::EReferenceRoundedPrefix
    {
        PrefixSample prefixSample(residualTiming,PrefixTiming::Prefix,account);
        using E=SignedAffineSphereNativeClosure::EClose;
        using P=C::EReferenceRoundedPrefix;
#if !UE_BUILD_SHIPPING
        // The generic contact and its selected float endpoint stay authoritative.
        // This only closes Rs on that prefix; the fresh Rs point still runs.
        if(diagonal698)
        {
            uint64 beganWork=0;
            if(timeDiagonal698&&++prefixCalls698==1)
            {
                beganWork=account.Used;
                prefixStart698=FPlatformTime::Cycles64();
            }
            const bool clear=TryDiagonal698Prefix(geometry,model,values,fullRc,generic698,end,account);
            if(timeDiagonal698&&prefixCalls698==1)
            {
                prefixEnd698=FPlatformTime::Cycles64();
                prefixWork698=account.Used-beganWork;
            }
            if(clear)return P::Clear;
        }
#endif
#if !UE_BUILD_SHIPPING
        if(finiteWallSelectedBits&&C::KinematicCenterDetail::Bits(end)==finiteWallSelectedBits)
        {
            int32 tail=0;
            double minimumGap=0.;
            const uint64 before=account.Used;
            const bool clear=ProveFiniteWallConsumedPrefix(values,model,end,account,tail,minimumGap);
            UE_LOG(LogTemp,Display,TEXT("[NativeSphereFiniteWallQuadraticRsRounded] Frame=%u EndBits=%08x Clear=%d Tail=%d GapLower=%.17g Used=%llu"),
                native->FrameClock.OwnerLocalFrame,finiteWallSelectedBits,int32(clear),tail,minimumGap,account.Used-before);
            if(clear)return P::Clear;
        }
#endif
        if(c2.Admitted&&fullRc)
        {
            if(!c2RoundedTried)
            {
                c2RoundedTried=true;
                c2RoundedBuilt=FReferenceSphereC2Prefix::BuildConsumed(*initial,*fullRc,account,c2Rounded);
            }
            if(c2RoundedBuilt&&c2Rounded.ProveConsumed(end,account))
            {
                c2.RoundedClear=true;
                c2RoundedEnd=end;
                return P::Clear;
            }
        }
#if !UE_BUILD_SHIPPING
        if(postSeamEndBits!=0&&C::KinematicCenterDetail::Bits(end)==postSeamEndBits)
        {
            const uint64 before=account.Used;
            const bool clear=postSeamProof.Rounded(end,account);
            UE_LOG(LogTemp,Display,TEXT("[NativeSphereC2PostSeamRoundedPrefix] Frame=%u EndBits=%08x Clear=%d Used=%llu"),
                native->FrameClock.OwnerLocalFrame,postSeamEndBits,int32(clear),account.Used-before);
            if(clear)return P::Clear;
        }
        if(variableWholeRcPreproved&&variableSignedNoContact&&admitWholeRc&&variableRoundedCandidates&&
            C::KinematicCenterDetail::Bits(end)==variableRoundedSelectedBits&&
            C::KinematicCenterDetail::Bits(end)==C::KinematicCenterDetail::Bits(horizon))
        {
            provedWholeRc=true;
            return P::Clear;
        }
        if(CSignedC2VariableRoundedPrefix.GetValueOnAnyThread()==1&&admitWholeRc&&variableRoundedCandidates&&fullRc&&
            variableRoundedSelectedBits==C::KinematicCenterDetail::Bits(end))
        {
            double scaledLower=0.;
            const uint64 before=account.Used;
            const int32 available=account.Remaining-account.ConsumptionFloor;
            // The whole native miss is a separate proof. High also pays the
            // finite 1940 rival on both spans at the literal inflated radius.
            const bool highWhole=variableRoundedBroadUV.IsSet()&&variableRoundedBroadUV->HighPrefixOnly;
            // 44 paid native admission/roster units in the reviewed corner
            // volume, plus at most 52 for two failed halves and four quarters.
            constexpr int32 highQuarterAllowance=96;
            const bool highQuarters=highWhole&&variableRoundedBroadUV->HasWholeQuarterProposals&&available>=highQuarterAllowance;
            const int32 wholeNativeAllowance=highQuarters?highQuarterAllowance:(highWhole?72:64);
            if(variableSignedNoContact&&variableRoundedBroadUV.IsSet()&&
                C::KinematicCenterDetail::Bits(end)==C::KinematicCenterDetail::Bits(horizon)&&available>=wholeNativeAllowance)
            {
                const FNativeBroadC2WholeRc broad{*initial,*variableRoundedCandidates,*fullRc,variableRoundedBroadUV.GetValue()};
                C::FScopedDistanceBudgetFloor optional(account,account.Remaining-wholeNativeAllowance);
                if(optional.IsValid())
                    provedWholeRc=ProveNativeWholeRcFromVolume(geometry,model,values,*native,nativeVolume,account,
                        false,false,&broad);
                if(highWhole)
                {
                    UE_LOG(LogTemp,Display,TEXT("[NativeSphereC2HighWholeRc] Frame=%u EndBits=%08x Clear=%d Used=%llu"),
                        native->FrameClock.OwnerLocalFrame,variableRoundedSelectedBits,int32(provedWholeRc),account.Used-before);
                }
                else
                {
                    UE_LOG(LogTemp,Display,TEXT("[NativeSphereC2BroadWholeRc] Frame=%u EndBits=%08x Clear=%d Used=%llu"),
                        native->FrameClock.OwnerLocalFrame,variableRoundedSelectedBits,int32(provedWholeRc),account.Used-before);
                }
                if(provedWholeRc)return P::Clear;
            }
            if(deferDirectedShort&&variableSignedNoContact&&
                C::KinematicCenterDetail::Bits(end)==C::KinematicCenterDetail::Bits(horizon)&&tryDirected(account))
            {
                provedWholeRc=true;
                return P::Clear;
            }
            const uint64 prefixBefore=account.Used;
            const int32 prefixAvailable=account.Remaining-account.ConsumptionFloor;
            const bool clear=VariableC2RoundedPrefix::Prove(*initial,*variableRoundedCandidates,*fullRc,end,account,scaledLower,
                variableRoundedLaterUV.IsSet()?&variableRoundedLaterUV.GetValue():nullptr,
                variableRoundedBroadUV.IsSet()?&variableRoundedBroadUV.GetValue():nullptr);
            UE_LOG(LogTemp,Display,TEXT("[NativeSphereC2VariableRoundedPrefix] Frame=%u EndBits=%08x Clear=%d ScaledLower=%.17g Available=%d Used=%llu"),
                native->FrameClock.OwnerLocalFrame,variableRoundedSelectedBits,int32(clear),scaledLower,prefixAvailable,account.Used-prefixBefore);
            if(clear)return P::Clear;
        }
        if(CSignedC2FixedRoundedPrefix.GetValueOnAnyThread()==1&&admitWholeRc&&fixedRoundedCandidates&&fullRc)
        {
            double scaledLower=0.;
            const uint64 before=account.Used;
            const int32 available=account.Remaining-account.ConsumptionFloor;
            const bool clear=FixedC2RoundedPrefix::Prove(*initial,*fixedRoundedCandidates,*fullRc,end,account,scaledLower);
            UE_LOG(LogTemp,Display,TEXT("[NativeSphereC2FixedRoundedPrefix] Frame=%u EndBits=%08x Clear=%d ScaledLower=%.17g Available=%d Used=%llu"),
                native->FrameClock.OwnerLocalFrame,C::KinematicCenterDetail::Bits(end),int32(clear),scaledLower,available,account.Used-before);
            if(clear){fixedRoundedEndBits=C::KinematicCenterDetail::Bits(end);return P::Clear;}
        }
#endif
        if(CSignedAffineSphereRoundedClosure.GetValueOnAnyThread()!=1)return P::NotApplicable;
        const bool separatePaths=CSignedAffineNativeWholeRcSeparatePaths.GetValueOnAnyThread()==1;
        if((!tryWholeRcEarly||separatePaths)&&admitWholeRc&&signedReferenceNoContact&&
            C::KinematicCenterDetail::Bits(end)==C::KinematicCenterDetail::Bits(horizon))
        {
            // The Rs result selects this proof only. Publication requires
            // independent native/reference/literal separation at full Rc.
            const int32 allowance=separatePaths?2*proofAllowance:proofAllowance;
            if(account.Remaining-account.ConsumptionFloor<allowance)return P::NotApplicable;
            {
                C::FScopedDistanceBudgetFloor optional(account,account.Remaining-allowance);
                if(!optional.IsValid())return P::NotApplicable;
                const uint64 before=account.Used;
                provedWholeRc=ProveNativeWholeRcFromVolume(geometry,model,values,*native,nativeVolume,
                    account,false,separatePaths);
                if(separatePaths)
                {
                    splitRcAttempted=true;
                    splitRcWork=account.Used-before;
                }
            }
            // Keep the original generic prefix available with its own floor.
            // The bounded optional work remains charged on a refusal.
            return provedWholeRc?P::Clear:P::NotApplicable;
        }
        if(!signedReferenceContact)return P::NotApplicable;
        C::FReferenceCandidateSetPtr roundedCover;
        if(fullRc&&fullRc->UsesGeometry(geometry)&&
            AnalyticCertified::Bits(fullRc->Partition->GetRadiusForFixedSource())==AnalyticCertified::Bits(double(values.QueryRadius)))
            roundedCover=fullRc->Partition;
        const auto close=SignedAffineSphereNativeClosure::TryPrefix(*initial,motion,end,options.PrefixRangeAttempts,
            account,roundedPartition,roundedCover,roundedPrefixObservation,
            CSignedAffineSphereRoundedTailSingleton.GetValueOnAnyThread()==1,
            CSignedAffineSphereRoundedVerticalOrigin.GetValueOnAnyThread()==1,residualTiming,timeMidpoint799);
        if(close==E::NotApplicable)return P::NotApplicable;
        if(close!=E::Clear)return P::Refused;
        usedSignedRoundedPrefix=true;
        return P::Clear;
    };
    const C::FReferenceContactProducer::FPrefixCallback prefix(proveRounded);
    const C::FReferenceContactProducer producer(produce,&prefix);
#if !UE_BUILD_SHIPPING
    const uint64 referenceStart698=timeDiagonal698?FPlatformTime::Cycles64():0;
#endif
    if(residualTiming)
    {
        residualTiming->EvaluateUsed=budget.Used;
        residualTiming->EvaluateStart=FPlatformTime::Cycles64();
    }
    auto reference=C::EvaluateReferenceSphere(geometry->GetSelectedUnion(),values.State.Location,values.State.Velocity,
        values.State.Acceleration,horizon,values.SupportRadius,double(values.PhysicalRadius),options,budget,
        [](int32){return true;},(tryAffine||tryC2)?&producer:nullptr);
    if(residualTiming)residualTiming->EvaluateEnd=FPlatformTime::Cycles64();
    if(c2.Admitted&&native)
        UE_LOG(LogTemp,Display,TEXT("[NativeSphereSignedC2] Frame=%u Admitted=1 TimeStatus=%u Failure=%u TimeFailure=%u ClearCalls=%u InsideCalls=%u RoundedClear=%d"),
            native->FrameClock.OwnerLocalFrame,uint32(reference.Contact.Status),uint32(reference.Contact.Failure),
            uint32(reference.Contact.TimeFailure),c2.ClearCalls,c2.InsideCalls,int32(c2.RoundedClear));
#if !UE_BUILD_SHIPPING
    if(timeDiagonal698)
    {
        const uint64 referenceEnd=FPlatformTime::Cycles64();
        const bool valid=prefixCalls698==1&&referenceStart698<=prefixStart698&&
            prefixStart698<=prefixEnd698&&prefixEnd698<=referenceEnd;
        UE_LOG(LogTemp,Display,TEXT("[NativeSphereDiagonal698SearchTiming] Frame=%llu OwnerFrame=%u Valid=%d PrefixCalls=%u SecondsPerCycle=%.17g ReferenceTicks=%llu BeforePrefixTicks=%llu PrefixTicks=%llu AfterPrefixTicks=%llu PrefixWork=%llu GenericCandidates=%d ContactStatus=%u ContactTimeWork=%llu SpatialCells=%llu"),
            native->FrameClock.CanonicalFrame,native->FrameClock.OwnerLocalFrame,int32(valid),prefixCalls698,
            FPlatformTime::GetSecondsPerCycle64(),referenceStart698<=referenceEnd?referenceEnd-referenceStart698:0,
            valid?prefixStart698-referenceStart698:0,valid?prefixEnd698-prefixStart698:0,
            valid?referenceEnd-prefixEnd698:0,prefixWork698,generic698?generic698->GetCandidateIndices().Num():0,
            uint32(reference.Contact.Status),uint64(reference.Contact.TimeWorkUsed),uint64(reference.Contact.SpatialVisitedCells));
    }
#endif
    searchFloor.Reset(); // Post-search work uses the same account with its original floor.
    // There is one owning reference buffer. Refusals move it into the optional
    // diagnostic; success moves it into the immutable arrival. No leaf-array clone.
    const auto refuseReference=[&](const TCHAR* why)
    {reason=why;if(partial)*partial=MoveTemp(reference);return false;};
    if((reference.Status!=C::EReferenceSphereStatus::ReferenceNoContact&&reference.Status!=C::EReferenceSphereStatus::ReferenceContactBounded)||
        !reference.ConsumedPrefixClear||!reference.Candidates||reference.Candidates->GetScene()!=geometry->GetSelectedUnion())
        return refuseReference(*FString::Printf(TEXT("FirstArrivalRs reference/strict consumed prefix unavailable; stage=%u contact_status=%u budget_remaining=%d rounded_admission=%llu rounded_separation=%llu rounded_cover=%d early_rc_work=%llu split_rc_attempted=%d split_rc_work=%llu; Unknown retained"),
            uint32(reference.Stage),uint32(reference.Contact.Status),budget.Remaining,roundedPrefixObservation.AdmissionWork,
            roundedPrefixObservation.SeparationWork,int32(roundedPrefixObservation.ReusedCover),earlyRcWork,int32(splitRcAttempted),splitRcWork));
    if(provedWholeRc)
    {
        if(!admitWholeRc||!fullRc->UsesGeometry(geometry)||
            AnalyticCertified::Bits(fullRc->Partition->GetRadiusForFixedSource())!=AnalyticCertified::Bits(double(values.QueryRadius))||
            reference.Status!=C::EReferenceSphereStatus::ReferenceNoContact||
            C::KinematicCenterDetail::Bits(reference.ConsumedSeconds)!=C::KinematicCenterDetail::Bits(horizon))
            return refuseReference(TEXT("Independent whole-H native Rc clearance identity differs"));
        *wholeRcClear=true;return true; // Distinct clearance; no arrival or hit is constructed.
    }
    if(reference.Status==C::EReferenceSphereStatus::ReferenceContactBounded&&
        (reference.Contact.CollisionRadiusBits!=AnalyticCertified::Bits(values.SupportRadius)||reference.Contact.Leaves.IsEmpty()))
        return refuseReference(TEXT("FirstArrivalRs enclosure radius/leaves differ"));
    // This is a mathematical evaluation of the SAME native DAG, not an
    // applied state. Reclassify its actual rounded center against Rs; never
    // describe the conservative consumed time as exact physical tangency.
    uint32 consumedBits=0;FMemory::Memcpy(&consumedBits,&reference.ConsumedSeconds,4);
    if((consumedBits&0x80000000u)||consumedBits>=0x7f800000u||(consumedBits&&consumedBits<0x00800000u))
        return refuseReference(TEXT("Consumed native seconds are not supported positive normal or exact +0"));
    C::FReferenceCandidateSetPtr rc;
    TOptional<C::FScopedDistanceBudgetFloor> publicationFloor;
    if(fullRc)
    {
        if(!fullRc->UsesGeometry(geometry)||
            AnalyticCertified::Bits(fullRc->Partition->GetRadiusForFixedSource())!=AnalyticCertified::Bits(double(values.QueryRadius))||
            reference.ConsumedSeconds>horizon||
            !ContainsRoundedCenterHull(*fullRc->Partition,C::BoundConsumedCenterBetweenFloatSeconds(model,0.f,horizon))||
            !ContainsRoundedCenterHull(*fullRc->Partition,C::BoundConsumedCenterBetweenFloatSeconds(model,0.f,reference.ConsumedSeconds)))
            return refuseReference(TEXT("Retained full-H Rc partition does not cover the exact consumed prefix"));
        rc=fullRc->Partition; // Complete conservative superset. No candidate array copy or second selection.
        // Response quality charges one entry, both complete candidate lists,
        // and one charge per contact leaf in each of quality and named hit.
        // The full producer policy was already sealed before the search.
        const int64 publication=1ll+reference.Candidates->GetCandidateIndices().Num()+rc->GetCandidateIndices().Num()+
            2ll*reference.Contact.Leaves.Num();
        if(publication>int64(budget.Remaining)-budget.ConsumptionFloor)
            return refuseReference(TEXT("General Sphere named response work unavailable after the temporal search"));
        publicationFloor.Emplace(budget,budget.ConsumptionFloor+int32(publication));
        if(!publicationFloor->IsValid())return refuseReference(TEXT("General Sphere named response work unavailable"));
    }
    else
    {
        const auto cover=PreparedExecution::SelectSelectedWholeConsumedSphereHull(geometry,model,reference.ConsumedSeconds,
            double(values.QueryRadius),budget);
        if(!cover.UsesGeometry(geometry))return refuseReference(TEXT("Complete general rounded Rc hull unavailable"));
        rc=cover.Partition;
    }
    FReferenceSphereStrictSeparationPtr consumed;
    bool pointClosed=false,c2FreshClosed=false;
#if !UE_BUILD_SHIPPING
    const uint64 freshStart=residualTiming?FPlatformTime::Cycles64():0;
    const uint64 freshBefore=residualTiming?budget.Used:0;
#endif
    if(usedSignedRoundedPrefix)
        pointClosed=FReferenceSphereStrictSeparation::TryProveAtSignedAffine(geometry,values,initial->GetLocalOffset(),initial->GetLocalCOM(),
            horizon,reference.ConsumedSeconds,options,budget,consumed,reason,roundedPartition.Get());
    else if(c2.RoundedClear&&C::KinematicCenterDetail::Bits(c2RoundedEnd)==consumedBits&&CSignedC2SphereFreshPoint.GetValueOnAnyThread()==1)
    {
        pointClosed=FReferenceSphereStrictSeparation::TryProveAtSignedC2(geometry,values,initial->GetLocalOffset(),initial->GetLocalCOM(),
            horizon,reference.ConsumedSeconds,options,budget,consumed,reason,c2Rounded,c2FreshClosed);
        UE_LOG(LogTemp,Display,TEXT("[NativeSphereSignedC2FreshPoint] Frame=%u SecondsBits=%08x Proved=%d PointClosed=%d"),
            native->FrameClock.OwnerLocalFrame,consumedBits,int32(c2FreshClosed),int32(pointClosed));
    }
#if !UE_BUILD_SHIPPING
    else if(postSeamEndBits!=0&&postSeamEndBits==consumedBits)
    {
        FReferenceSphereC2Prefix fresh;
        const uint64 before=budget.Used;
        const bool bound=FReferenceSphereC2Prefix::BindPostSeamFresh(postSeamProof,reference.ConsumedSeconds,fresh);
        if(bound)
            pointClosed=FReferenceSphereStrictSeparation::TryProveAtSignedC2(geometry,values,initial->GetLocalOffset(),initial->GetLocalCOM(),
                horizon,reference.ConsumedSeconds,options,budget,consumed,reason,fresh,c2FreshClosed);
        else
            pointClosed=FReferenceSphereStrictSeparation::TryProveAt(geometry,values,initial->GetLocalOffset(),initial->GetLocalCOM(),
                horizon,reference.ConsumedSeconds,options,budget,consumed,reason);
        UE_LOG(LogTemp,Display,TEXT("[NativeSphereC2PostSeamFreshPoint] Frame=%u EndBits=%08x Bound=%d Proved=%d PointClosed=%d Used=%llu"),
            native->FrameClock.OwnerLocalFrame,consumedBits,int32(bound),int32(c2FreshClosed),int32(pointClosed),budget.Used-before);
    }
    else if(CSignedC2VariableFreshPoint.GetValueOnAnyThread()==1&&admitWholeRc&&variableRoundedCandidates&&fullRc&&
        variableRoundedSelectedBits!=0&&variableRoundedSelectedBits==consumedBits)
    {
        FReferenceSphereC2Prefix variableFresh;
        const bool bound=FReferenceSphereC2Prefix::BindVariableFresh(*initial,*variableRoundedCandidates,*fullRc,
            reference.ConsumedSeconds,variableRoundedLaterUV.IsSet()?&variableRoundedLaterUV.GetValue():nullptr,
            variableRoundedBroadUV.IsSet()?&variableRoundedBroadUV.GetValue():nullptr,variableFresh);
        const uint64 before=budget.Used;
        if(bound)
            pointClosed=FReferenceSphereStrictSeparation::TryProveAtSignedC2(geometry,values,initial->GetLocalOffset(),initial->GetLocalCOM(),
                horizon,reference.ConsumedSeconds,options,budget,consumed,reason,variableFresh,c2FreshClosed);
        else
            pointClosed=FReferenceSphereStrictSeparation::TryProveAt(geometry,values,initial->GetLocalOffset(),initial->GetLocalCOM(),
                horizon,reference.ConsumedSeconds,options,budget,consumed,reason);
        UE_LOG(LogTemp,Display,TEXT("[NativeSphereC2VariableFreshPoint] Frame=%u SecondsBits=%08x Bound=%d Proved=%d PointClosed=%d Used=%llu"),
            native->FrameClock.OwnerLocalFrame,consumedBits,int32(bound),int32(c2FreshClosed),int32(pointClosed),budget.Used-before);
    }
    else if(CSignedC2FixedFreshPoint.GetValueOnAnyThread()==1&&admitWholeRc&&fixedRoundedCandidates&&fullRc&&
        fixedRoundedEndBits!=0&&fixedRoundedEndBits==consumedBits)
    {
        FReferenceSphereC2Prefix fixedFresh;
        const bool bound=FReferenceSphereC2Prefix::BindFixedFresh(*initial,*fixedRoundedCandidates,*fullRc,
            reference.ConsumedSeconds,fixedFresh);
        const uint64 before=budget.Used;
        if(bound)
            pointClosed=FReferenceSphereStrictSeparation::TryProveAtSignedC2(geometry,values,initial->GetLocalOffset(),initial->GetLocalCOM(),
                horizon,reference.ConsumedSeconds,options,budget,consumed,reason,fixedFresh,c2FreshClosed);
        else
            pointClosed=FReferenceSphereStrictSeparation::TryProveAt(geometry,values,initial->GetLocalOffset(),initial->GetLocalCOM(),
                horizon,reference.ConsumedSeconds,options,budget,consumed,reason);
        UE_LOG(LogTemp,Display,TEXT("[NativeSphereC2FixedFreshPoint] Frame=%u SecondsBits=%08x Bound=%d Proved=%d PointClosed=%d Used=%llu"),
            native->FrameClock.OwnerLocalFrame,consumedBits,int32(bound),int32(c2FreshClosed),int32(pointClosed),budget.Used-before);
    }
#endif
    else
        pointClosed=FReferenceSphereStrictSeparation::TryProveAt(geometry,values,initial->GetLocalOffset(),initial->GetLocalCOM(),
            horizon,reference.ConsumedSeconds,options,budget,consumed,reason);
#if !UE_BUILD_SHIPPING
    if(residualTiming&&pointClosed&&consumed&&reference.Status==C::EReferenceSphereStatus::ReferenceContactBounded)
    {
        const uint64 now=FPlatformTime::Cycles64();
        auto& t=*residualTiming;
        auto& fresh=t.Rows[PrefixTiming::Fresh];
        fresh.Calls=1;
        t.Integrity&=now>=freshStart&&budget.Used>=freshBefore;
        if(now>=freshStart)fresh.Ticks=now-freshStart;
        if(budget.Used>=freshBefore)fresh.Work=budget.Used-freshBefore;
        const bool ordered=t.EvaluateStart<=t.ProduceFirst&&t.ProduceFirst<=t.EvaluateEnd;
        const uint64 total=ordered?t.EvaluateEnd-t.EvaluateStart:0;
        const uint64 selection=ordered?t.ProduceFirst-t.EvaluateStart:0;
        const uint64 nested=selection+t.Rows[PrefixTiming::Produce].Ticks+t.Rows[PrefixTiming::Prefix].Ticks;
        t.Integrity&=ordered&&nested<=total;
        const uint64 other=nested<=total?total-nested:0;
        static constexpr const TCHAR* labels[]={TEXT("Produce"),TEXT("Prefix"),TEXT("Admission"),TEXT("WholeBox"),
            TEXT("Joint"),TEXT("Tail"),TEXT("Predecessor"),TEXT("RangeRetry"),TEXT("Fresh")};
        static_assert(UE_ARRAY_COUNT(labels)==PrefixTiming::Count);
        FString rows;
        for(int32 i=0;i<PrefixTiming::Count;++i)
        {
            const auto& row=t.Rows[i];
            rows+=FString::Printf(TEXT(" %s=%llu/%llu/%llu"),labels[i],row.Calls,row.Ticks,row.Work);
        }
        UE_LOG(LogTemp,Display,TEXT("[NativeSphere799ReferenceResidualTiming] Frame=5360 Serial=1 HBits=3b5a740e EndBits=%08x Integrity=%d SignedPrefix=%d CoverAttempts=%u ReselectAttempts=%u ReusedCover=%d SecPerCycle=%.17g EvaluateTicks=%llu SelectionTicks=%llu SelectionWork=%llu OtherConsumerTicks=%llu CallsTicksWork=1%s"),
            consumedBits,int32(t.Integrity),int32(usedSignedRoundedPrefix),t.CoverAttempts,t.ReselectAttempts,
            int32(roundedPrefixObservation.ReusedCover),FPlatformTime::GetSecondsPerCycle64(),total,selection,t.SelectWork,other,*rows);
    }
#endif
    if(!pointClosed||!consumed)
    {
        const FString why=FString::Printf(TEXT("Fresh general rounded consumed Rs separation unavailable; %s"),*reason);
        return refuseReference(*why);
    }
    TSharedPtr<FReferenceSphereGeneralFirstArrivalRs,ESPMode::ThreadSafe> result(new FReferenceSphereGeneralFirstArrivalRs);
    result->Initial=MoveTemp(initial);result->ConsumedPoint=MoveTemp(consumed);result->Reference=MoveTemp(reference);
    result->RcPartition=MoveTemp(rc);result->Horizon=horizon;
    RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);out=result;return true;
}
}
