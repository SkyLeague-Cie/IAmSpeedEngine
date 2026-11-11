#pragma once
#include "../Analytic/SignedC2FiniteRival.h"

// Private proof for one authenticated variable-height reference path.
// All arithmetic publishes only a strict clear decision on the current cell.
namespace Speed::Analytic::Prepared::VariableC2Time
{
using I=C::Interval;
using P=C::FIntervalPoint;
using T=C::FIntervalTensorSurface;
namespace A=::AnalyticCertified;

struct FFace
{
    int32 Index;
    const TCHAR* Canonical;
    const TCHAR* Bound;
};

inline constexpr FFace Faces[] = {
    {135, TEXT("7ad93ffbf6662f147b4acbf2c89a829c6a827e2d36a0d0640af083535aa5cefd"), TEXT("ff6e47ecb3ef0b92ad88176117b58f8e8a8c8cac486cd92c928c9a11b509f1a4")},
    {695, TEXT("265fd94dbe81cafce7661e55b8c9cc08652ecd9315f9b2737340802579f1c656"), TEXT("116ec11ca74bd9f56a14a68f9465d289a7e28b5cfcb1525b9f152c915ae1bfed")},
    {1940, TEXT("963b7f366903d0757dc1d56f837c85b91ce8a6cd4e2cb26bfdc49a83658a6399"), TEXT("5301f5523033d9c5af71b8b90438813736ab0e8ed82bb19fae81f9f2459df3ed")},
    {2375, TEXT("19f2338a81e1a0c4f04127181ef74ddd52bda147c630c2089551abf0fd185aec"), TEXT("b5a5a966d5e9024fae893c21c511c2059d36dbbb2813533c41272b23b9daa515")},
};

inline constexpr double BroadNativeRadiusCap=93.150108337402344;

inline constexpr FFace BroadOptionalFaces[] = {
    {788, TEXT("810b2ebc22bd021e74b5d383a87898488a0cca88bf6c1244b6f1bfb27ae25dbd"), TEXT("55d50b6958c748ae0352d684bdd05dccbdd2f7c69fa41735e2f6e91dcb932e3e")},
    {1442, TEXT("a4427821429e755caabad1bc2f44d2b157dff56acf125745fe8b3ffed0857fa1"), TEXT("fa3c2bda7893f04d65c1084ad9a62e10a6a360db5eb21ed69543a2be6bce28ec")},
};

inline constexpr FFace SeamExtraFaces[]={
    {1098,TEXT("79c3ea62fe3d1b41b03c0862cb090367d49ef20909044e8840ed33fe0022a870"),
          TEXT("698687b9c7662ed806db00f0232f87f3d3c34eab67fa4a20c39dc02c8b925a05")},
    {1490,TEXT("ebddd5266d2c3db0aee54441bed1a2a0b7e310b0e3c8187c2b067f1504070e23"),
          TEXT("a91c929f1cc3cdcb44916d1f840728efba397cd9797492d5a06e08bd88b3ab5d")},
    {870,TEXT("6eff73b374ed92ffd7a0f6a26f07b46c25c2e5e69a1aaf015d9f607224f21775"),
         TEXT("7308334b2fdd15db573523fa930a9562c8127e170d9eb9c6f6921ae4f7d46ee2")},
};

struct FNativeProposals
{
    FVector2d FirstHalf, Tail;
    FVector2d FirstQuarter=FVector2d::ZeroVector,ThirdQuarter=FVector2d::ZeroVector;
    bool ExtendedEarly=false,HighPrefixOnly=false,HasWholeQuarterProposals=false;
    const C::FReferenceCandidateSet* SeamSideFor=nullptr;
    uint64 SeamNumericControl=0;
};

inline bool AdmitsGeometry(const FReferenceSphereStrictSeparation& initial,const C::FReferenceCandidateSet& rs,
    const PreparedExecution::FSelectedWholeHullPartition& rc,P& tube,bool broad=false,bool seam=false,bool allowNextPanelSubset=false,
    int32 requiredNextPanel=2375,bool descendingPanel=false)
{
    if(descendingPanel&&(!broad||!seam||!allowNextPanelSubset||
        (requiredNextPanel!=870&&requiredNextPanel!=1098&&requiredNextPanel!=2375)))return false;
    const auto& geometry=initial.GetGeometry();
    const auto& v=initial.GetCapturedInput();
    if(!geometry||!geometry->HasValidAliases()||geometry->GetModel()!=EPreparedGeometryModel::RetainedReference||
        !rc.UsesGeometry(geometry)||!rc.Partition||!initial.GetModel().IsKnown()||
        C::KinematicCenterDetail::Bits(initial.GetSeconds())!=0||
        (broad?!(initial.GetHorizon()>0.f&&initial.GetHorizon()<=1.f/300.f):
            C::KinematicCenterDetail::Bits(initial.GetHorizon())!=C::KinematicCenterDetail::Bits(1.f/300.f))||
        !((descendingPanel?v.State.Velocity.Z<0.:v.State.Velocity.Z>0.)&&v.State.Acceleration.Z<0.)||
        !C::CandidateDetail::PositiveRadius(v.SupportRadius)||double(v.QueryRadius)<v.SupportRadius||
        double(v.QueryRadius)>93.150100708007812)return false;
    const auto& reference=geometry->GetReference();
    const auto scene=geometry->GetSelectedUnion();
    if(!reference||!scene||scene!=reference->GetCanonicalUnion()||rs.GetScene()!=scene||
        rc.Partition->GetScene()!=scene||A::Bits(rs.GetRadiusForFixedSource())!=A::Bits(v.SupportRadius)||
        A::Bits(rc.Partition->GetRadiusForFixedSource())!=A::Bits(double(v.QueryRadius))||
        reference->GetExternalPacketIdentity()!=TEXT("090d41158d13f1127827b720ff467a51e415668aa205209a67ed8afcb6b10dc0"))
        return false;
    const auto identities=reference->GetProofIdentities();
    const auto map=reference->GetProviderFaceIndices();
    const auto faces=reference->GetFaces();
    const auto owners=geometry->GetNominalScene()->GetOwners();
    if(identities.Num()!=10||identities[0]!=TEXT("46cd45f7af71cf745f541c92e1a10066c9dba9cb7fcc27d1fbe0d0ecdcaf31a8")||
        map.Num()!=2992||owners.Num()!=2992||faces.Num()!=2654)return false;
    const auto matchesFace=[&](const FFace& expected)
    {
        if(!faces.IsValidIndex(map[expected.Index])||owners[expected.Index].SourceId!=1274291482858373289ull)return false;
        const auto& face=faces[map[expected.Index]];
        return !face.FromPlane&&face.CanonicalRecordIdentity==expected.Canonical&&face.BoundRecordIdentity==expected.Bound&&
            face.DomainAndChart.Num()==4;
    };
    const bool liveLatePanel=seam&&allowNextPanelSubset&&
        (requiredNextPanel==1098||requiredNextPanel==870||(requiredNextPanel==2375&&descendingPanel));
    if(liveLatePanel&&!matchesFace(requiredNextPanel==2375?Faces[3]:SeamExtraFaces[requiredNextPanel==870?2:0]))return false;
    const auto matchesPartition=[&](const C::FReferenceCandidateSet& partition)
    {
        const auto indices=partition.GetCandidateIndices();
        const auto excluded=partition.GetExcludedIndices();
        const bool subset=seam&&allowNextPanelSubset;
        if(subset&&requiredNextPanel!=2375&&!liveLatePanel)return false;
        if(indices.Num()<(subset?1:4)||indices.Num()>(liveLatePanel?16:seam?7:broad?6:4)||
            excluded.Num()+indices.Num()!=scene->Num()||
            partition.GetSurfacesForFixedSource().Num()!=indices.Num())return false;
        // Both objects are complete original selections from this immutable
        // scene and radius. The candidate lists are left intact. The native
        // volume must retain both and strictly prove every other live rival.
        if(liveLatePanel)return indices.Contains(requiredNextPanel);
        if(seam)
        {
            const FFace expected[]={Faces[0],Faces[1],BroadOptionalFaces[0],SeamExtraFaces[0],SeamExtraFaces[1],Faces[2],Faces[3]};
            if(!subset&&indices.Num()!=UE_ARRAY_COUNT(expected))return false;
            int32 next=0;
            for(const auto& face:expected)
            {
                if(indices.IsValidIndex(next)&&indices[next]==face.Index)
                {
                    if(!matchesFace(face))return false;
                    ++next;
                    continue;
                }
                // Only the finite-panel lanes can admit a subset. Each
                // omitted role must be excluded by this original radius
                // partition. The complete native volume remains separate.
                if(!subset||face.Index==requiredNextPanel||!excluded.Contains(face.Index))return false;
            }
            return next==indices.Num();
        }
        int32 next=0;
        for(const auto& expected:Faces)
        {
            if(broad&&expected.Index==1940)
                for(const auto& optional:BroadOptionalFaces)
                    if(indices.IsValidIndex(next)&&indices[next]==optional.Index)
                    {
                        if(!matchesFace(optional))return false;
                        ++next;
                    }
            if(!indices.IsValidIndex(next)||indices[next++]!=expected.Index||!matchesFace(expected))return false;
        }
        return next==indices.Num();
    };
    if(!matchesPartition(rs)||!matchesPartition(*rc.Partition))return false;
    // The whole native query's conservative volume can retain an optional
    // role absent from the reference/rounded partitions. Authenticate it too.
    if(broad&&!liveLatePanel)
        for(const auto& optional:BroadOptionalFaces)
            if(!matchesFace(optional))return false;
    const auto& path=rs.GetPathForFixedSource();
    const auto& original=initial.GetModel().GetReferenceCurve();
    if(!C::CenterDetail::Valid(path)||path.Degree!=2||path.ControlPoints.Num()!=3||
        original.Degree!=2||original.ControlPoints.Num()!=3)return false;
    for(int32 i=0;i<3;++i)
        if(!C::GroupedBoxDetail::SamePoint(path.ControlPoints[i],original.ControlPoints[i]))return false;
    tube=C::CenterDetail::Hull(path);
    return C::Detail::ValidPoint(tube);
}


inline bool ContainsSeamRsPrefix(const P& point)
{
    return C::Detail::ValidPoint(point)&&
        point.Axis[0].Lower>=4980.099609375&&point.Axis[0].Upper<=4981.6875&&
        point.Axis[1].Lower>=-1343.953125&&point.Axis[1].Upper<=-1343.4921875&&
        point.Axis[2].Lower>=96.1875&&point.Axis[2].Upper<=96.75;
}

inline bool AdmitsSeamRs(const FReferenceSphereStrictSeparation& initial,
    const C::FReferenceCandidateSet& rs,const PreparedExecution::FSelectedWholeHullPartition& rc)
{
    P tube;
    if(!AdmitsGeometry(initial,rs,rc,tube,true,true)||
        C::KinematicCenterDetail::Bits(initial.GetHorizon())!=C::KinematicCenterDetail::Bits(1.f/300.f))return false;
    const auto& controls=rs.GetPathForFixedSource().ControlPoints;
    return ContainsSeamRsPrefix(controls[0])&&
        tube.Axis[0].Upper<=4982.125&&tube.Axis[1].Lower>=-1344.125&&tube.Axis[2].Upper<=97.;
}

inline bool Admits(const FReferenceSphereStrictSeparation& initial,const C::FReferenceCandidateSet& rs,
    const PreparedExecution::FSelectedWholeHullPartition& rc)
{
    P tube;
    if(!AdmitsGeometry(initial,rs,rc,tube))return false;
    // Inward dyadic subset of the audited center box. The complete three
    // rivals and all 695 outside the interior chart clear the larger Rc.
    return C::Detail::ValidPoint(tube)&&
        tube.Axis[0].Lower>=4963.93408203125&&tube.Axis[0].Upper<=4966.1826171875&&
        tube.Axis[1].Lower>=-1339.6376953125&&tube.Axis[1].Upper<=-1339.0283203125&&
        tube.Axis[2].Lower>=93.176025390625&&tube.Axis[2].Upper<=93.1939697265625;
}

// Closed original phase-5 chart, with the same Hessian bound >=1000.
// The signed packet leaves only 135,695,1940,2375 here. The three rivals
// clear Rc throughout this rectangle, independent of the sampled motion.
inline bool InExtendedEarlyTube(const P& point)
{
    return C::Detail::ValidPoint(point)&&
        point.Axis[0].Lower>=4963.5&&point.Axis[0].Upper<=4966.375&&
        point.Axis[1].Lower>=-1339.75&&point.Axis[1].Upper<=-1338.75&&
        point.Axis[2].Lower>=93.169921875&&point.Axis[2].Upper<=93.203125;
}

inline bool AdmitsExtendedEarly(const FReferenceSphereStrictSeparation& initial,const C::FReferenceCandidateSet& rs,
    const PreparedExecution::FSelectedWholeHullPartition& rc)
{
    P tube;
    return AdmitsGeometry(initial,rs,rc,tube)&&InExtendedEarlyTube(tube);
}

inline I Scale(double scale,I x){return A::Multiply(I::Exact(scale),x);}
inline I Dot(const P& a,const P& b)
{
    I out=I::Exact(0.);
    for(int32 d=0;d<3;++d)out=A::Add(out,A::Multiply(a.Axis[d],b.Axis[d]));
    return out;
}
inline P Subtract(const P& a,const P& b)
{
    P out;
    for(int32 d=0;d<3;++d)out.Axis[d]=A::Subtract(a.Axis[d],b.Axis[d]);
    return out;
}
inline bool Derivative(const T& source,bool alongU,C::FDistanceBudget& budget,T& out)
{
    out.DegreeU=source.DegreeU-int32(alongU);out.DegreeV=source.DegreeV-int32(!alongU);
    if(out.DegreeU<0||out.DegreeV<0)return false;
    const int32 stride=source.DegreeV+1,step=alongU?stride:1;
    const double degree=double(alongU?source.DegreeU:source.DegreeV);
    for(int32 i=0;i<=out.DegreeU;++i)for(int32 j=0;j<=out.DegreeV;++j)
    {
        if(!budget.Consume())return false;
        const int32 k=i*stride+j;
        auto row=Subtract(source.ControlPoints[k+step],source.ControlPoints[k]);
        for(auto& axis:row.Axis)axis=Scale(degree,axis);
        if(!C::Detail::ValidPoint(row))return false;
        out.ControlPoints.Add(row);
    }
    return true;
}

struct FChart
{
    const T* Value=nullptr;
    bool Later=false,Broad=false,SeamRs=false;
    T U,V,UU,UV,VV;
    bool Build(const T& source,C::FDistanceBudget& budget)
    {
        if(source.DegreeU!=5||source.DegreeV!=5||source.ControlPoints.Num()!=36)return false;
        if(!Derivative(source,true,budget,U)||!Derivative(source,false,budget,V)||
            !Derivative(U,true,budget,UU)||!Derivative(U,false,budget,UV)||!Derivative(V,false,budget,VV))return false;
        Value=&source;return true;
    }
};

inline bool Inside(const FChart& chart,FVector2d uv)
{return uv.X>=.875&&uv.X<=(chart.SeamRs?1.:chart.Broad?127./128.:.96875)&&uv.Y>=.03125&&
    uv.Y<=(chart.Broad?7./8.:chart.Later?17./64.:.21875);}

inline bool Sample(const T& surface,FVector2d uv,C::FDistanceBudget& budget,P& out)
{return budget.Consume()&&C::ProjectedDistanceDetail::EvaluateFixedUV(surface,uv,out);}

inline bool Propose(const FChart& chart,const P& center,C::FDistanceBudget& budget,FVector2d& uv)
{
    const auto middle=[](I a){return .5*a.Lower+.5*a.Upper;};
    uv=FVector2d((-1280.-middle(center.Axis[1]))/64.,chart.SeamRs?27./32.:(middle(center.Axis[0])-4960.)/63.);
    for(int32 i=0;i<5;++i)
    {
        if(!Inside(chart,uv))return false;
        P p,u,v,uu,uvv,vv;
        if(!Sample(*chart.Value,uv,budget,p)||!Sample(chart.U,uv,budget,u)||!Sample(chart.V,uv,budget,v)||
            !Sample(chart.UU,uv,budget,uu)||!Sample(chart.UV,uv,budget,uvv)||!Sample(chart.VV,uv,budget,vv))return false;
        const auto d=Subtract(p,center);
        const double gu=middle(Dot(d,u)),gv=middle(Dot(d,v));
        const double a=middle(A::Add(Dot(u,u),Dot(d,uu))),b=middle(A::Add(Dot(u,v),Dot(d,uvv)));
        const double c=middle(A::Add(Dot(v,v),Dot(d,vv))),det=a*c-b*b;
        if(!(det>0.)||!A::Finite(det))return false;
        uv.X-=(c*gu-b*gv)/det;uv.Y-=(a*gv-b*gu)/det;
        // The original seed can overshoot V on the signed approach cell.
        // Recover the proposal within the same five paid iterations. All
        // original directed sample, interval and rival tests still decide.
        if(i==0&&chart.Broad&&uv.X>=7./8.&&uv.X<=127./128.&&
            A::Finite(uv.Y)&&uv.Y>7./8.&&C::SignedC2FiniteRival::ContainsApproach(center))
            uv.Y=3./4.;
    }
    // This is only a proposal. Every accepted interval below is evaluated
    // at the resulting fixed binary64 coordinates with directed arithmetic.
    return Inside(chart,uv);
}

using C::SignedC2FiniteRival::SquareCoefficients;

inline bool ProveQuadraticAtUV(const FChart& chart,const P (&centers)[3],FVector2d uv,double radius,
    C::FDistanceBudget& budget,double& lower)
{
    if(!chart.Value||!Inside(chart,uv)||!C::CandidateDetail::PositiveRadius(radius))return false;
    for(const auto& center:centers)if(!C::Detail::ValidPoint(center))return false;
    P p,u,v;
    if(!Sample(*chart.Value,uv,budget,p)||!Sample(chart.U,uv,budget,u)||!Sample(chart.V,uv,budget,v)||
        !budget.Consume())return false;
    P diff[3];
    I gu[3],gv[3],gu2[5],gv2[5],norm[5]{};
    for(int32 i=0;i<3;++i)
    {
        diff[i]=Subtract(p,centers[i]);gu[i]=Dot(diff[i],u);gv[i]=Dot(diff[i],v);
    }
    for(auto& q:norm)q=I::Exact(0.);
    for(int32 d=0;d<3;++d)
    {
        const I axis[3]={diff[0].Axis[d],diff[1].Axis[d],diff[2].Axis[d]};
        I squared[5];SquareCoefficients(axis,squared);
        for(int32 i=0;i<5;++i)norm[i]=A::Add(norm[i],squared[i]);
    }
    SquareCoefficients(gu,gu2);SquareCoefficients(gv,gv2);
    const auto r2=Scale(6.,A::Square(I::Exact(radius)));
    lower=TNumericLimits<double>::Max();
    const double hessian=chart.Broad?500.:chart.Later?800.:1000.;
    for(int32 i=0;i<5;++i)
    {
        // The authenticated chart supplies the lower Hessian bound.
        // Normalize all charts to the existing 6000*squared-gap readback.
        const auto signedValue=A::Subtract(Scale(hessian,A::Subtract(norm[i],r2)),A::Add(gu2[i],gv2[i]));
        const auto scaled=chart.Broad?Scale(2.,signedValue):chart.Later?Scale(1.25,signedValue):signedValue;
        if(!A::Valid(scaled)||scaled.Lower<=0.)return false;
        lower=FMath::Min(lower,scaled.Lower);
    }
    return true;
}
inline bool ProveQuadratic(const FChart& chart,const P (&centers)[3],const P& proposalCenter,double radius,
    C::FDistanceBudget& budget,double& lower)
{
    if(!chart.Value||!C::Detail::ValidPoint(proposalCenter)||!C::CandidateDetail::PositiveRadius(radius))return false;
    for(const auto& center:centers)if(!C::Detail::ValidPoint(center))return false;
    FVector2d uv;
    return Propose(chart,proposalCenter,budget,uv)&&ProveQuadraticAtUV(chart,centers,uv,radius,budget,lower);
}

inline bool Restrict(const C::FIntervalBezierCenterPath& path,double first,double last,
    C::FDistanceBudget& budget,P (&centers)[3])
{
    if(!budget.Consume())return false;
    const auto a=I::Exact(first);
    centers[0]=C::CenterDetail::Evaluate(path,a);
    centers[2]=C::CenterDetail::Evaluate(path,I::Exact(last));
    if(!C::Detail::ValidPoint(centers[0])||!C::Detail::ValidPoint(centers[2]))return false;
    // Exact reparameterized quadratic control 1 = C(first)+(last-first)*C'(first)/2.
    const auto delta=A::Subtract(I::Exact(last),a);
    const auto oneMinus=A::Subtract(I::Exact(1.),a);
    for(int32 d=0;d<3;++d)
    {
        const auto x=path.ControlPoints[0].Axis[d],y=path.ControlPoints[1].Axis[d],z=path.ControlPoints[2].Axis[d];
        const auto tangent=A::Add(A::Multiply(oneMinus,A::Subtract(y,x)),A::Multiply(a,A::Subtract(z,y)));
        centers[1].Axis[d]=A::Add(centers[0].Axis[d],A::Multiply(delta,tangent));
    }
    return C::Detail::ValidPoint(centers[1]);
}

inline bool AdmitsHighPrefixOnly(const FReferenceSphereStrictSeparation& initial,const C::FReferenceCandidateSet& rs,
    const PreparedExecution::FSelectedWholeHullPartition& rc)
{
    P hull;
    if(!AdmitsGeometry(initial,rs,rc,hull,true))return false;
    const auto& centers=rs.GetPathForFixedSource().ControlPoints;
    return C::SignedC2FiniteRival::ContainsPath(centers[0],centers[1],centers[2]);
}


inline bool ProveSeamRsRival(const C::FReferenceCandidateSet& rs,const P (&centers)[3],double proposalV,
    C::FDistanceBudget& budget,double& lower)
{
    for(const auto& point:centers)if(!ContainsSeamRsPrefix(point))return false;
    const auto surfaces=rs.GetSurfacesForFixedSource();
    const int32 local=rs.GetCandidateIndices().Find(1940);
    return surfaces.IsValidIndex(local)&&C::SignedC2FiniteRival::ProveOnCheckedFiniteEdge(
        surfaces[local],centers,proposalV,rs.GetRadiusForFixedSource(),budget,lower);
}

inline bool ProveHighRival(const C::FReferenceCandidateSet& rs,const P (&centers)[3],double proposalV,double radius,
    C::FDistanceBudget& budget,double& lower)
{
    const auto surfaces=rs.GetSurfacesForFixedSource();
    const int32 local=rs.GetCandidateIndices().Find(1940);
    return surfaces.IsValidIndex(local)&&C::SignedC2FiniteRival::Prove(surfaces[local],centers,proposalV,radius,budget,lower);
}

}
