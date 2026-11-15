#pragma once

// Independent reference-guided proof of the original binary32-seconds
// consumed path and actual singleton. Neither path selects an event or time.
namespace Speed::Analytic::Prepared::VariableC2RoundedPrefix
{
namespace A=::AnalyticCertified;
using I=A::Interval;
using P=C::FIntervalPoint;

inline bool InAuditedTube(const P& point)
{
    // Same inward dyadic subset as the authenticated variable-time theorem.
    return C::Detail::ValidPoint(point)&&
        point.Axis[0].Lower>=4963.93408203125&&point.Axis[0].Upper<=4966.1826171875&&
        point.Axis[1].Lower>=-1339.6376953125&&point.Axis[1].Upper<=-1339.0283203125&&
        point.Axis[2].Lower>=93.176025390625&&point.Axis[2].Upper<=93.1939697265625;
}

inline bool Binds(const FReferenceSphereStrictSeparation& initial,const C::FReferenceCandidateSet& rs,
    const PreparedExecution::FSelectedWholeHullPartition& rc,float end,bool later,
    const VariableC2Time::FNativeProposals* dynamicUV)
{
    if(!A::Finite(double(end))||end<=0.f||end>initial.GetHorizon())return false;
    const bool admitted=dynamicUV?
        (dynamicUV->SeamSideFor?dynamicUV->SeamSideFor==&rs&&
            dynamicUV->SeamNumericControl==RequireCurrentAnalyticNumericContext(
                EAnalyticNumericContract::Binary64NearestAllowFlush).ControlFingerprint()&&
            VariableC2Time::AdmitsSeamRs(initial,rs,rc):
            dynamicUV->HighPrefixOnly?VariableC2Time::AdmitsHighPrefixOnly(initial,rs,rc):
            dynamicUV->ExtendedEarly?VariableC2Time::AdmitsExtendedEarly(initial,rs,rc):BroadC2Tube::Admits(initial,rs,rc)):
        (later?LaterC2Tube::Admits(initial,rs,rc):VariableC2Time::Admits(initial,rs,rc));
    if(!admitted)
        return false;
    const auto& hull=rc.Partition->GetPathForFixedSource();
    return hull.Degree==0&&hull.ControlPoints.Num()==1&&
        C::GroupedBoxDetail::SamePoint(hull.ControlPoints[0],
            C::BoundConsumedCenterBetweenFloatSeconds(initial.GetModel(),0.f,initial.GetHorizon()));
}

inline bool ContainsNativeCenter(const P& point,const FVector2d* laterUV,
    const VariableC2Time::FNativeProposals* dynamicUV)
{
    if(dynamicUV)
        return dynamicUV->SeamSideFor?VariableC2Time::ContainsSeamRsPrefix(point):
            dynamicUV->HighPrefixOnly?C::SignedC2FiniteRival::Contains(point):
            dynamicUV->ExtendedEarly?VariableC2Time::InExtendedEarlyTube(point):BroadC2Tube::Contains(point);
    return laterUV?LaterC2Tube::Contains(point):InAuditedTube(point);
}

// For each actual binary32 t in [0,end], t*t has absolute binary32 rounding
// error <= 2^-41. The real magnitudes below bound all binary64 operands and
// intermediates in Pos+Vel*t+0.5*Accel*(t*t), whether fused or separate.
inline bool CenterError(const FStaticSphereInitialValues& values,double end,I (&out)[3])
{
    const auto t=I::Exact(end),t2=A::Square(t);
    if(!A::Valid(t2)||t2.Upper>=0x1p-16)return false;
    const auto roundedSquare=A::Add(t2,I::Exact(0x1p-41));
    for(int32 d=0;d<3;++d)
    {
        const double p=FMath::Abs(values.State.Location[d]),v=FMath::Abs(values.State.Velocity[d]);
        const double a=FMath::Abs(values.State.Acceleration[d]);
        if(!(p<8191.&&v<8191.&&a<8191.))return false;
        const auto half=A::Multiply(I::Exact(.5),I::Exact(a));
        const auto size=A::Add(A::Add(I::Exact(p),A::Multiply(I::Exact(v),t)),A::Multiply(half,roundedSquare));
        if(!A::Valid(size)||size.Upper>=8191.)return false;
        // Five binary64 operations below 8192, including any underflow or
        // rounding of a half-acceleration, fit in the fixed 2^-37 cm term.
        const auto e=A::Add(A::Multiply(half,I::Exact(0x1p-41)),I::Exact(0x1p-37));
        if(!A::Valid(e)||e.Upper<=0.)return false;
        out[d]={-e.Upper,e.Upper,true};
    }
    return true;
}

inline constexpr double ProposalU[2]={0x1.da542993c3858p-1,0x1.dc60d11d16a44p-1};
inline constexpr double ProposalV[2]={0x1.9b3c48cae2619p-4,0x1.0765c5ecd4832p-3};

inline bool Prove(const FReferenceSphereStrictSeparation& initial,const C::FReferenceCandidateSet& rs,
    const PreparedExecution::FSelectedWholeHullPartition& rc,float end,C::FDistanceBudget& budget,double& lower,const FVector2d* laterUV,
    const VariableC2Time::FNativeProposals* broadUV,const FVector3d* literalChordError=nullptr,
    double wholeNativeRadius=0.)
{
    lower=0.;
    const bool high=broadUV&&broadUV->HighPrefixOnly;
    const bool seam=broadUV&&broadUV->SeamSideFor;
    constexpr int32 quarterAllowance=52;
    const bool refineHighWhole=high&&literalChordError&&broadUV->HasWholeQuarterProposals&&
        budget.Remaining-budget.ConsumptionFloor>=quarterAllowance;
    const auto& path=rs.GetPathForFixedSource();
    const bool refineApproachPrefix=high&&!literalChordError&&end<initial.GetHorizon()&&
        budget.Remaining-budget.ConsumptionFloor>=quarterAllowance&&path.Degree==2&&path.ControlPoints.Num()==3&&
        C::SignedC2FiniteRival::ContainsApproach(path.ControlPoints[0])&&
        C::SignedC2FiniteRival::ContainsApproach(path.ControlPoints[1])&&
        C::SignedC2FiniteRival::ContainsApproach(path.ControlPoints[2]);
    const bool refineSeam=seam&&!literalChordError&&end<initial.GetHorizon();
    if(seam&&!refineSeam)return false;
    const int32 allowance=(refineHighWhole||refineApproachPrefix||refineSeam)?quarterAllowance:(high?22:16);
    if(budget.Remaining-budget.ConsumptionFloor<allowance)return false;
    C::FScopedDistanceBudgetFloor bounded(budget,budget.Remaining-allowance);
    if(!bounded.IsValid()||!budget.Consume()||!Binds(initial,rs,rc,end,laterUV!=nullptr,broadUV))return false;
    const auto& values=initial.GetCapturedInput();
    const double seconds=double(end),times[3]={0.,.5*seconds,seconds};
    I error[3];
    VariableC2Time::FChart chart;
    if(!CenterError(values,seconds,error)||
        !FixedC2RoundedPrefix::BuildPositionAndDerivatives(rs.GetSurfacesForFixedSource()[1],budget,chart))return false;
    double radius=values.SupportRadius;
    if(literalChordError)
    {
        if(!broadUV||broadUV->ExtendedEarly||!(wholeNativeRadius>=double(values.QueryRadius))||
            wholeNativeRadius>VariableC2Time::BroadNativeRadiusCap)return false;
        radius=wholeNativeRadius;
        for(int32 d=0;d<3;++d)
        {
            const double extra=(*literalChordError)[d];
            if(!A::Finite(extra)||extra<0.)return false;
            error[d]=A::Add(error[d],I{-extra,extra,true});
            if(!A::Valid(error[d]))return false;
        }
    }
    chart.Later=laterUV!=nullptr;chart.Broad=broadUV&&!broadUV->ExtendedEarly;
    chart.SeamRs=seam;
    const auto proveSpan=[&](int32 cell,double a,double b,const FVector2d* quarterUV,double& margin)
    {
        if(!budget.Consume())return false;
        const double ends[3][2]={{a,a},{a,b},{b,b}};
        P centers[3];
        for(int32 k=0;k<3;++k)
        {
            const auto s0=I::Exact(ends[k][0]),s1=I::Exact(ends[k][1]);
            for(int32 d=0;d<3;++d)
            {
                const auto vel=A::Multiply(I::Exact(.5),I::Exact(values.State.Velocity[d]));
                const auto accel=A::Multiply(I::Exact(.5),I::Exact(values.State.Acceleration[d]));
                const auto linear=A::Multiply(vel,A::Add(s0,s1));
                const auto quadratic=A::Multiply(accel,A::Multiply(s0,s1));
                centers[k].Axis[d]=A::Add(A::Add(A::Add(I::Exact(values.State.Location[d]),linear),quadratic),error[d]);
            }
            if(!high&&!ContainsNativeCenter(centers[k],laterUV,broadUV))return false;
        }
        if(high&&!C::SignedC2FiniteRival::ContainsPath(centers[0],centers[1],centers[2]))return false;
        // UV is only a proposal. The current directed Bernstein residual,
        // signed patch and complete geometry decide every admitted motion.
        FVector2d proposal(ProposalU[cell],ProposalV[cell]);
        if(quarterUV)proposal=*quarterUV;
        else if(broadUV)proposal=cell==0?broadUV->FirstHalf:broadUV->Tail;
        else if(laterUV)
        {
            const auto y=centers[2].Axis[1];
            proposal=cell==1?*laterUV:FVector2d((-1280.-(.5*y.Lower+.5*y.Upper))/64.,
                .5*(laterUV->Y+(values.State.Location.X-4960.)/63.));
        }
        if(!VariableC2Time::ProveQuadraticAtUV(chart,centers,proposal,radius,budget,margin))return false;
        if(seam)
        {
            double rivalLower=0.;
            if(!VariableC2Time::ProveSeamRsRival(rs,centers,proposal.Y,budget,rivalLower))return false;
            margin=FMath::Min(margin,rivalLower);
        }
        else if(high)
        {
            double rivalLower=0.;
            const double rivalRadius=literalChordError?radius:double(values.QueryRadius);
            if(!VariableC2Time::ProveHighRival(rs,centers,proposal.Y,rivalRadius,budget,rivalLower))
                return false;
            margin=FMath::Min(margin,rivalLower);
        }
        return margin>0.;
    };
    double margins[2]{};
    for(int32 cell=0;cell<2;++cell)
    {
        const double a=times[cell],b=times[cell+1];
        if(proveSpan(cell,a,b,nullptr,margins[cell]))continue;
        if(!refineHighWhole&&!refineApproachPrefix&&!refineSeam)return false;
        // Keep a successful half. A refused half is replaced by two fresh
        // directed proofs, with the original errors, radii and every rival.
        const double mid=.5*a+.5*b;
        const FVector2d quarterUV=refineHighWhole?(cell==0?broadUV->FirstQuarter:broadUV->ThirdQuarter):
            (cell==0?broadUV->FirstHalf*1.5-broadUV->Tail*.5:(broadUV->FirstHalf+broadUV->Tail)*.5);
        double first=0.,last=0.;
        if(!proveSpan(cell,a,mid,&quarterUV,first)||!proveSpan(cell,mid,b,nullptr,last))return false;
        margins[cell]=FMath::Min(first,last);
    }
    lower=FMath::Min(margins[0],margins[1]);return lower>0.;
}

inline bool ProveFresh(const FReferenceSphereStrictSeparation& initial,const C::FReferenceCandidateSet& rs,
    const PreparedExecution::FSelectedWholeHullPartition& rc,const FPreparedGeometrySelectionPtr& geometry,
    const FStaticSphereInitialValues& captured,FVector3d localOffset,FVector3d localCOM,float horizon,float end,
    const FVector3d& actual,const P& originalBound,C::FDistanceBudget& budget,const FVector2d* laterUV,
    const VariableC2Time::FNativeProposals* broadUV)
{
    const auto bits=C::KinematicCenterDetail::Bits;
    const auto& expected=initial.GetCapturedInput();
    const auto point=C::Detail::Point(actual);
    const auto same=FixedC2RoundedPrefix::SameBits;
    if(initial.GetGeometry()!=geometry||bits(horizon)!=bits(initial.GetHorizon())||
        !same(localOffset,initial.GetLocalOffset())||!same(localCOM,initial.GetLocalCOM())||
        !same(captured.State.Location,expected.State.Location)||!same(captured.State.Velocity,expected.State.Velocity)||
        !same(captured.State.Acceleration,expected.State.Acceleration)||
        A::Bits(captured.SupportRadius)!=A::Bits(expected.SupportRadius)||bits(captured.QueryRadius)!=bits(expected.QueryRadius)||
        bits(captured.PhysicalRadius)!=bits(expected.PhysicalRadius)||bits(captured.StaticContactOffset)!=bits(expected.StaticContactOffset)||
        bits(captured.ActualMargin)!=bits(expected.ActualMargin)||!ContainsNativeCenter(point,laterUV,broadUV)||
        !C::ReferenceSphereDetail::Contains(originalBound,actual))return false;
    const bool high=broadUV&&broadUV->HighPrefixOnly;
    const bool seam=broadUV&&broadUV->SeamSideFor;
    const int32 allowance=(high||seam)?13:10;
    if(budget.Remaining-budget.ConsumptionFloor<allowance)return false;
    C::FScopedDistanceBudgetFloor bounded(budget,budget.Remaining-allowance);
    if(!bounded.IsValid()||!budget.Consume()||!Binds(initial,rs,rc,end,laterUV!=nullptr,broadUV)||
        !C::GroupedBoxDetail::SamePoint(originalBound,C::BoundConsumedCenterAtFloatSeconds(initial.GetModel(),end))||
        !C::ReferenceSphereDetail::Contains(rc.Partition->GetPathForFixedSource().ControlPoints[0],actual))return false;
    VariableC2Time::FChart chart;
    if(!FixedC2RoundedPrefix::BuildPositionAndDerivatives(rs.GetSurfacesForFixedSource()[1],budget,chart))return false;
    chart.Later=laterUV!=nullptr;chart.Broad=broadUV&&!broadUV->ExtendedEarly;
    chart.SeamRs=seam;
    const P centers[3]={point,point,point};
    double lower=0.;
    const auto proposal=broadUV?broadUV->Tail:laterUV?*laterUV:FVector2d(ProposalU[1],ProposalV[1]);
    if(!VariableC2Time::ProveQuadraticAtUV(chart,centers,proposal,captured.SupportRadius,budget,lower)||lower<=0.)
        return false;
    if(seam)return VariableC2Time::ProveSeamRsRival(rs,centers,proposal.Y,budget,lower);
    return !high||VariableC2Time::ProveHighRival(rs,centers,proposal.Y,double(captured.QueryRadius),budget,lower);
}
}
