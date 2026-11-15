#pragma once

// The authenticated 1940 and 2375 patches are exact finite Y extrusions.
// The signed XZ half-Hessians on their respective boxes exceed 570 and 642,
// over the entire original V domain. Both reuse the conservative 500 bound.
namespace Speed::Analytic::Prepared::PostSeamC2
{
namespace A=::AnalyticCertified;
using I=C::Interval;
using P=C::FIntervalPoint;
using T=C::FIntervalTensorSurface;
inline bool Contains(const P& p,bool nextPanel)
{
    if(!C::Detail::ValidPoint(p))return false;
    if(nextPanel)
        return p.Axis[0].Lower>=4985.5&&p.Axis[0].Upper<=5024.&&
            p.Axis[1].Lower>=-1362.&&p.Axis[1].Upper<=-1344.5&&
            p.Axis[2].Lower>=98.25&&p.Axis[2].Upper<=128.;
    return p.Axis[0].Lower>=4983.75&&p.Axis[0].Upper<=4986.&&
        p.Axis[1].Lower>=-1345.25&&p.Axis[1].Upper<=-1344.5&&
        p.Axis[2].Lower>=97.5&&p.Axis[2].Upper<=98.25;
}

// Signed 2375 free-Y product, full original V: XZ half-Hessian exceeds
// 970 throughout this axial extension of its original XZ box.
inline bool ContainsDescendingNextPanel(const P& p)
{
    return C::Detail::ValidPoint(p)&&p.Axis[0].Lower>=4985.5&&p.Axis[0].Upper<=5024.&&
        p.Axis[1].Lower>=-2319.25&&p.Axis[1].Upper<=-1344.5&&
        p.Axis[2].Lower>=98.25&&p.Axis[2].Upper<=128.;
}

// Authenticated 1098 product, full original V: directed XZ half-Hessian
// exceeds 544 throughout this box. The proof uses the existing bound 500.
inline bool ContainsThirdPanel(const P& p)
{
    return C::Detail::ValidPoint(p)&&p.Axis[0].Lower>=5004.&&p.Axis[0].Upper<=5040.&&
        p.Axis[1].Lower>=-1372.&&p.Axis[1].Upper<=-1350.&&
        p.Axis[2].Lower>=109.&&p.Axis[2].Upper<=148.;
}

// The authenticated 1098 is the same finite free-Y product throughout
// this axial interval. Its signed full-V XZ half-Hessian exceeds 977 here.
// Y neither enters that derivative nor lowers the projected distance bound.
inline bool ContainsDescendingThirdPanel(const P& p)
{
    return C::Detail::ValidPoint(p)&&p.Axis[0].Lower>=5004.&&p.Axis[0].Upper<=5040.&&
        p.Axis[1].Lower>=-2319.25&&p.Axis[1].Upper<=-1350.&&
        p.Axis[2].Lower>=109.&&p.Axis[2].Upper<=148.;
}

// Signed 870 free-Y product, full V XZ half-Hessian >522 on this box.
inline bool ContainsOriginalFourthPanel(const P& p)
{
    return C::Detail::ValidPoint(p)&&p.Axis[0].Lower>=5020.&&p.Axis[0].Upper<=5040.&&
        p.Axis[1].Lower>=-1372.&&p.Axis[1].Upper<=-1350.&&
        p.Axis[2].Lower>=131.&&p.Axis[2].Upper<=148.;
}

// Work domain. The spatial proof outside the original box binds a
// separate convexity certificate to the actual expanded center hull.
inline bool ContainsFourthPanel(const P& p)
{
    return C::Detail::ValidPoint(p)&&p.Axis[0].Lower>=5020.&&p.Axis[0].Upper<=5060.&&
        p.Axis[1].Lower>=-1400.&&p.Axis[1].Upper<=-1350.&&
        p.Axis[2].Lower>=131.&&p.Axis[2].Upper<=180.;
}

// Whole-volume work only: provider 870 is signed as a free-Y extrusion
// from -1344 to below -2319.25. This does not extend positive contact
// admission. The live XZ half-Hessian and complete volume still must pass.
inline bool ContainsFourthWholeVolume(const P& p)
{
    return C::Detail::ValidPoint(p)&&p.Axis[0].Lower>=5020.&&p.Axis[0].Upper<=5060.&&
        p.Axis[1].Lower>=-2319.25&&p.Axis[1].Upper<=-1350.&&
        p.Axis[2].Lower>=131.&&p.Axis[2].Upper<=180.;
}

inline bool ContainsNaturalPrefix(const P& p)
{
    return C::Detail::ValidPoint(p)&&p.Axis[0].Lower>=4980.375&&p.Axis[0].Upper<=4982.&&
        p.Axis[1].Lower>=-1344.0625&&p.Axis[1].Upper<=-1343.5625&&
        p.Axis[2].Lower>=96.25&&p.Axis[2].Upper<=96.875;
}

// Authenticated 695: outside this localized chart no point can reach either
// original radius in the prefix box. Inside, the half-Hessian exceeds 500.
struct FNaturalRival
{
    VariableC2Time::FChart Chart;
    bool Clear(const P (&centers)[3],double seedV,double radius,C::FDistanceBudget& account) const
    {
        namespace V=VariableC2Time;
        for(const auto& center:centers)if(!ContainsNaturalPrefix(center))return false;
        const auto middle=[](I a){return .5*a.Lower+.5*a.Upper;};
        const auto valid=[](FVector2d uv){return uv.X>=7./8.&&uv.X<=1.&&uv.Y>=1./32.&&uv.Y<=29./32.;};
        const auto& center=centers[2];
        FVector2d uv(FMath::Min(1.,(-1280.-middle(center.Axis[1]))/64.),seedV);
        for(int32 i=0;i<5;++i)
        {
            P p,u,v,uu,duv,vv;
            if(!valid(uv)||!V::Sample(*Chart.Value,uv,account,p)||!V::Sample(Chart.U,uv,account,u)||
                !V::Sample(Chart.V,uv,account,v)||!V::Sample(Chart.UU,uv,account,uu)||
                !V::Sample(Chart.UV,uv,account,duv)||!V::Sample(Chart.VV,uv,account,vv))return false;
            const auto d=V::Subtract(p,center);
            const double gu=middle(V::Dot(d,u)),gv=middle(V::Dot(d,v));
            const double a=middle(A::Add(V::Dot(u,u),V::Dot(d,uu)));
            const double b=middle(A::Add(V::Dot(u,v),V::Dot(d,duv)));
            const double c=middle(A::Add(V::Dot(v,v),V::Dot(d,vv))),det=a*c-b*b;
            if(!(det>0.&&c>0.)||!A::Finite(det))return false;
            if(uv.X==1.&&gu<0.)uv.Y-=gv/c;
            else
            {
                uv.X=FMath::Min(1.,uv.X-(c*gu-b*gv)/det);
                uv.Y-=(a*gv-b*gu)/det;
            }
        }
        P point,du,dv;
        if(!valid(uv)||!V::Sample(*Chart.Value,uv,account,point)||!V::Sample(Chart.U,uv,account,du)||
            !V::Sample(Chart.V,uv,account,dv)||!account.Consume())return false;
        P delta[3];
        I gu[3],gv[3],gu2[5],gv2[5],norm[5];
        bool edge=uv.X==1.;
        for(int32 i=0;i<3;++i)
        {
            delta[i]=V::Subtract(point,centers[i]);
            gu[i]=V::Dot(delta[i],du);gv[i]=V::Dot(delta[i],dv);
            edge=edge&&A::Valid(gu[i])&&gu[i].Upper<=0.;
        }
        // Feasible movement from U=1 has dU<=0. Only a directed
        // nonpositive gradient over the whole cell eliminates its residual.
        if(!edge)V::SquareCoefficients(gu,gu2);
        V::SquareCoefficients(gv,gv2);
        for(auto& value:norm)value=I::Exact(0.);
        for(int32 d=0;d<3;++d)
        {
            const I lane[]={delta[0].Axis[d],delta[1].Axis[d],delta[2].Axis[d]};
            I squared[5];V::SquareCoefficients(lane,squared);
            for(int32 i=0;i<5;++i)norm[i]=A::Add(norm[i],squared[i]);
        }
        const auto r2=V::Scale(6.,A::Square(I::Exact(radius)));
        for(int32 i=0;i<5;++i)
        {
            const auto residual=edge?gv2[i]:A::Add(gu2[i],gv2[i]);
            const auto gap=A::Subtract(V::Scale(500.,A::Subtract(norm[i],r2)),residual);
            if(!A::Valid(gap)||gap.Lower<=0.)return false;
        }
        return true;
    }
};

class FProof final
{
    const FReferenceSphereStrictSeparation* Initial=nullptr;
    const C::FReferenceCandidateSet* Rs=nullptr;
    const PreparedExecution::FSelectedWholeHullPartition* Rc=nullptr;
    const T* Surface=nullptr;
    int32 Local=INDEX_NONE;
    bool NextPanel=false,Natural=false,Sibling1940=false,ThirdPanel=false,FourthPanel=false;
    bool DescendingThirdPanel=false,DescendingNextPanel=false,WallLip=false;
    FNaturalRival NaturalRival;
    P FourthPanelConvexDomain;
    P SiblingPoint,SiblingTangent;
    double NextPanelEnd=1.;
    double NaturalClearEnd=0.,NaturalInsideEnd=0.;
    double NaturalRcClearEnd=0.,NaturalRcInsideEnd=0.;
    uint64 Numeric=0;
    I XZ[2][6],Slope[2][5],Curvature[2][4],Y[6],YSlope[5];
    I WallLipOffset[2][6];
    double WallLipX=0.;

    static double Middle(I a){return .5*a.Lower+.5*a.Upper;}
    static I Scale(double x,I v){return A::Multiply(I::Exact(x),v);}
    template<int32 N> static I Eval(const I (&net)[N],double u)
    {
        I work[N];
        for(int32 i=0;i<N;++i)work[i]=net[i];
        return C::SignedC2FiniteRival::Evaluate(work,N,I::Exact(u));
    }
    static bool Equal(I a,I b)
    {
        return A::Valid(a)&&A::Valid(b)&&A::Bits(a.Lower)==A::Bits(b.Lower)&&A::Bits(a.Upper)==A::Bits(b.Upper);
    }
    bool ProveSiblingEndpoint(const P (&centers)[3],double radius,C::FDistanceBudget& account) const
    {
        namespace V=VariableC2Time;
        if(!account.Consume()||!C::CandidateDetail::PositiveRadius(radius))return false;
        I norm[5];
        P delta[3];
        for(auto& value:norm)value=I::Exact(0.);
        for(int32 k=0;k<3;++k)
        {
            if(!Contains(centers[k],false))return false;
            delta[k]=V::Subtract(SiblingPoint,centers[k]);
            delta[k].Axis[1]=I::Exact(0.);
            const auto halfGradient=V::Dot(delta[k],SiblingTangent);
            if(!A::Valid(halfGradient)||halfGradient.Lower<=0.)return false;
        }
        // Signed complete original 2375 V half-Hessian >821 in this box.
        // Positive V derivative at the finite left endpoint makes it the
        // exact XZ minimum. Dropping Y only lowers the original distance.
        for(int32 d=0;d<3;d+=2)
        {
            const I lane[]={delta[0].Axis[d],delta[1].Axis[d],delta[2].Axis[d]};
            I squared[5];V::SquareCoefficients(lane,squared);
            for(int32 k=0;k<5;++k)norm[k]=A::Add(norm[k],squared[k]);
        }
        const auto r2=Scale(6.,A::Square(I::Exact(radius)));
        for(const auto& value:norm)
        {
            const auto gap=A::Subtract(value,r2);
            if(!A::Valid(gap)||gap.Lower<=0.)return false;
        }
        return true;
    }
    bool ProveSeparateRival(const P (&centers)[3],double radius,C::FDistanceBudget& account) const
    {
        if(Sibling1940)return ProveSiblingEndpoint(centers,radius,account);
        double v=0.;
        return Natural&&ProposeV(centers[2],account,v)&&NaturalRival.Clear(centers,v,radius,account);
    }
    bool ProveFourthPanelConvexity(const P& centers,C::FDistanceBudget& account,int32 allowance=256,
        bool wholeVolume=false) const
    {
        constexpr int32 degree=8;
        if(!(wholeVolume?ContainsFourthWholeVolume(centers):ContainsFourthPanel(centers))||
            account.Remaining-account.ConsumptionFloor<allowance)return false;
        C::FScopedDistanceBudgetFloor limit(account,account.Remaining-allowance);
        if(!limit.IsValid())return false;
        constexpr int32 choose3[]={1,3,3,1},choose4[]={1,4,6,4,1},choose5[]={1,5,10,10,5,1};
        constexpr int32 choose8[]={1,8,28,56,70,56,28,8,1};
        struct FCell
        {
            I Net[degree+1];
            int32 Depth=0;
        };
        FCell root;
        for(int32 k=0;k<=degree;++k)
        {
            if(!account.Consume())return false;
            auto sum=I::Exact(0.);
            // Products of degree 4x4 and 5x3 give the full degree-8
            // coefficients of |q'|^2 + (q-center) dot q''.
            for(int32 i=FMath::Max(0,k-4);i<=FMath::Min(4,k);++i)
            {
                const int32 j=k-i;
                const auto product=A::Add(A::Multiply(Slope[0][i],Slope[0][j]),
                    A::Multiply(Slope[1][i],Slope[1][j]));
                if(!account.Consume())return false;
                sum=A::Add(sum,Scale(double(choose4[i]*choose4[j]),product));
            }
            for(int32 i=FMath::Max(0,k-3);i<=FMath::Min(5,k);++i)
            {
                const int32 j=k-i;
                const auto product=A::Add(
                    A::Multiply(A::Subtract(XZ[0][i],centers.Axis[0]),Curvature[0][j]),
                    A::Multiply(A::Subtract(XZ[1][i],centers.Axis[2]),Curvature[1][j]));
                if(!account.Consume())return false;
                sum=A::Add(sum,Scale(double(choose5[i]*choose3[j]),product));
            }
            if(!C::ContactDetail::ProjectionDividePositive(sum,I::Exact(double(choose8[k])),root.Net[k]))return false;
        }
        TArray<FCell,TInlineAllocator<6>> pending;
        pending.Add(root);
        while(!pending.IsEmpty())
        {
            if(!account.Consume())return false;
            auto cell=pending.Pop(EAllowShrinking::No);
            bool positive=true;
            for(const auto value:cell.Net)
            {
                if(!A::Valid(value))return false;
                positive=positive&&value.Lower>500.;
            }
            if(positive)continue;
            if(cell.Depth==4)return false;
            FCell left,right;
            left.Depth=right.Depth=cell.Depth+1;
            left.Net[0]=cell.Net[0];
            right.Net[degree]=cell.Net[degree];
            for(int32 level=1;level<=degree;++level)
            {
                if(!account.Consume())return false;
                for(int32 j=0;j<=degree-level;++j)
                    cell.Net[j]=Scale(.5,A::Add(cell.Net[j],cell.Net[j+1]));
                left.Net[level]=cell.Net[0];
                right.Net[degree-level]=cell.Net[degree-level];
            }
            pending.Add(right);
            pending.Add(left);
        }
        return true;
    }
    void LoadProjectedCurve(const T& patch)
    {
        for(int32 j=0;j<2;++j)
        {
            for(int32 k=0;k<6;++k)XZ[j][k]=patch.ControlPoints[k].Axis[2*j];
            for(int32 k=0;k<5;++k)Slope[j][k]=Scale(5.,A::Subtract(XZ[j][k+1],XZ[j][k]));
            for(int32 k=0;k<4;++k)Curvature[j][k]=Scale(4.,A::Subtract(Slope[j][k+1],Slope[j][k]));
        }
    }

    bool ProposeCurveV(const P& center,C::FDistanceBudget& account,double& v,bool finiteExtrusion) const
    {
        const bool allowEndpoints=NextPanel||finiteExtrusion;
        v=WallLip?FMath::Clamp(1.+(Middle(center.Axis[2])-160.)/64.,0.,1.):
            Natural?.86:allowEndpoints?.055:.96;
        for(int32 i=0;i<5;++i)
        {
            if(!account.Consume()||!A::Finite(v)||v<0.||v>1.||(!allowEndpoints&&(v==0.||v==1.)))return false;
            double gradient=0.,second=0.;
            for(int32 j=0;j<2;++j)
            {
                const double delta=Middle(Eval(XZ[j],v))-Middle(center.Axis[2*j]);
                const double slope=Middle(Eval(Slope[j],v));
                gradient+=delta*slope;
                second+=slope*slope+delta*Middle(Eval(Curvature[j],v));
            }
            if(!A::Finite(gradient)||!A::Finite(second)||second<=0.)return false;
            v-=gradient/second;
            if(allowEndpoints)
            {
                if(!A::Finite(v))return false;
                v=FMath::Clamp(v,0.,1.);
            }
        }
        return A::Finite(v)&&(allowEndpoints?v>=0.&&v<=1.:v>0.&&v<1.);
    }
    bool ProposeV(const P& center,C::FDistanceBudget& account,double& v) const
    {
        return ContainsCenter(center)&&ProposeCurveV(center,account,v,false);
    }

    bool ClearProjectedCell(const T& patch,const P (&centers)[3],double radius,C::FDistanceBudget& account,
        bool finiteExtrusion=false) const
    {
        P projected[]={centers[0],centers[1],centers[2]};
        // Dropping Y only lowers distance to the original finite extrusion.
        for(auto& center:projected)center.Axis[1]=I::Exact(-1344.);
        double v=0.,lower=0.;
        return ProposeCurveV(centers[2],account,v,finiteExtrusion)&&C::SignedC2FiniteRival::ProveOnCheckedFiniteEdge(
            patch,projected,v,radius,account,lower,(DescendingThirdPanel||DescendingNextPanel)?900.:500.)&&lower>0.;
    }

    I WallLipDistance(const P& center,double v,double radius,I* gradient=nullptr) const
    {
        if(!WallLip||!C::Detail::ValidPoint(center)||center.Axis[0].Lower>WallLipX||
            center.Axis[0].Upper<WallLipX||!(radius>64.&&radius<128.)||!(v>=0.&&v<=1.))
            return {};
        // On this authenticated zero-X-motion request, Sterbenz gives
        // both 5120-X and (5120-X)-radius exactly. Avoid subtracting two
        // roughly 8700 cm^2 quantities at the signed near-tangent root.
        const double d=5120.-WallLipX;
        if(!(d>=64.&&d<=128.&&radius*.5<=d&&d<=radius*2.))return {};
        const auto dx=Eval(WallLipOffset[0],v);
        const auto dz=A::Add(A::Subtract(center.Axis[2],I::Exact(160.)),Eval(WallLipOffset[1],v));
        auto gap=A::Add(A::Multiply(I::Exact(d-radius),A::Add(I::Exact(d),I::Exact(radius))),
            A::Add(A::Multiply(I::Exact(-2.*d),dx),A::Add(A::Square(dx),A::Square(dz))));
        if(gradient)
            *gradient=A::Subtract(A::Multiply(A::Subtract(I::Exact(d),dx),Eval(Slope[0],v)),
                A::Multiply(dz,Eval(Slope[1],v)));
        return gap;
    }
    bool ClearWallLip(const P (&centers)[3],double radius,C::FDistanceBudget& account) const
    {
        auto end=centers[2];
        double lowest=end.Axis[2].Lower;
        for(const auto& center:centers)
        {
            if(center.Axis[0].Lower>WallLipX||center.Axis[0].Upper<WallLipX)return false;
            lowest=FMath::Min(lowest,center.Axis[2].Lower);
        }
        // All XZ controls are monotone, X(surface)>X(center), and Z(0)
        // is below this entire chart. At a minimizing interior V,
        // (centerZ-Z)*Z'=(X-centerX)*X'>=0. At either finite endpoint
        // the same centerZ>=Z follows from the signed endpoint KKT
        // (at V=1, X'=0). Thus the minimum distance is monotone in
        // centerZ and the lowest certified Bernstein control suffices.
        end.Axis[0]=I::Exact(WallLipX);
        end.Axis[2]=I::Exact(lowest);
        double v=1.;
        if(lowest<160.&&!ProposeV(end,account,v))return false;
        I gradient;
        if(!account.Consume())return false;
        const auto gap=WallLipDistance(end,v,radius,&gradient);
        if(!A::Valid(gap)||!A::Valid(gradient))return false;
        if(lowest>=160.)return gap.Lower>0.;
        // The live signed half-Hessian exceeds 500. Using 256 in the
        // convex lower bound is conservative and keeps the scale dyadic.
        const auto bound=A::Subtract(gap,Scale(1./256.,A::Square(gradient)));
        return A::Valid(bound)&&bound.Lower>0.;
    }

public:
    static bool ClearFiniteWholeVolume(const T& patch,const P (&centers)[3],const P& hull,double radius,
        const C::FIntervalBezierCenterPath& physical,const C::FIntervalBezierCenterPath& literal,
        const FReferenceAffineExtrusionDescription& product,C::FDistanceBudget& account,bool descendingNext)
    {
        if(!(descendingNext?ContainsDescendingNextPanel(hull):ContainsFourthWholeVolume(hull))||
            patch.DegreeU!=5||patch.DegreeV!=5||patch.ControlPoints.Num()!=36||
            !account.Consume())return false;
        for(const auto& center:centers)
        {
            if(!C::Detail::ValidPoint(center))return false;
            for(int32 d=0;d<3;++d)
                if(center.Axis[d].Lower<hull.Axis[d].Lower||center.Axis[d].Upper>hull.Axis[d].Upper)return false;
        }

        FProof projected;
        projected.LoadProjectedCurve(patch);
        projected.DescendingNextPanel=descendingNext;
        // 2375 uses its source-bound full-V XZ curvature proof. The
        // 870 lane still recomputes and pays the live curvature certificate.
        if(!descendingNext&&!projected.ProveFourthPanelConvexity(hull,account,128,true))return false;
        // Check a bounded partition of the same input path. The full-V
        // Hessian and source product must both hold before accepting any cover.
        struct FCell {C::FIntervalBezierCenterPath Path;int32 Depth,Offset;};
        C::FIntervalBezierCenterPath whole;
        whole.Degree=2;whole.ControlPoints={centers[0],centers[1],centers[2]};
        TArray<FCell,TInlineAllocator<3>> pending;
        pending.Add({MoveTemp(whole),0,0});
        while(!pending.IsEmpty())
        {
            if(!account.Consume())return false;
            auto cell=pending.Pop(EAllowShrinking::No);
            const P control[]={cell.Path.ControlPoints[0],cell.Path.ControlPoints[1],cell.Path.ControlPoints[2]};
            if(projected.ClearProjectedCell(patch,control,radius,account,true))continue;
            if(cell.Depth>0)
            {
                // Keep the original common cover first. Where the artificial
                // chord inflation is too wide, prove both original paths
                // separately over the same dyadic cell and live Hessian.
                const double count=double(1<<cell.Depth);
                const double first=double(cell.Offset)/count,last=double(cell.Offset+1)/count;
                P actual[3],chord[3];
                if(VariableC2Time::Restrict(physical,first,last,account,actual)&&
                    projected.ClearProjectedCell(patch,actual,radius,account,true)&&
                    VariableC2Time::Restrict(literal,first,last,account,chord)&&
                    projected.ClearProjectedCell(patch,chord,radius,account,true))continue;
            }
            C::FIntervalBezierCenterPath left,right;
            if(cell.Depth==2||!account.Consume()||!C::CenterDetail::Split(cell.Path,left,right))return false;
            pending.Add({MoveTemp(right),cell.Depth+1,2*cell.Offset+1});
            pending.Add({MoveTemp(left),cell.Depth+1,2*cell.Offset});
        }
        return product.StructurallyMatches(patch,account);
    }

    bool Bind(const FReferenceSphereStrictSeparation& initial,const C::FReferenceCandidateSet& rs,
        const PreparedExecution::FSelectedWholeHullPartition& rc,C::FDistanceBudget& account,
        bool allowNatural=false,bool thirdPanel=false,bool fourthPanel=false,bool allowDescendingNext=false)
    {
        if(thirdPanel&&fourthPanel)return false;
        const bool descending=initial.GetCapturedInput().State.Velocity.Z<0.;
        const bool descendingNext=allowDescendingNext&&!thirdPanel&&!fourthPanel&&descending;
        const bool latePanel=thirdPanel||fourthPanel||descendingNext;
        const bool descendingFourth=fourthPanel&&descending;
        const bool descendingThird=thirdPanel&&descending;
        P box;
        if(C::KinematicCenterDetail::Bits(initial.GetHorizon())!=C::KinematicCenterDetail::Bits(1.f/300.f))
            return false;
        const bool originalRoster=!latePanel&&VariableC2Time::AdmitsGeometry(initial,rs,rc,box,true,true);
        if(latePanel)
        {
            const int32 selected=descendingNext?2375:fourthPanel?870:1098;
            if(!VariableC2Time::AdmitsGeometry(initial,rs,rc,box,true,true,true,selected,descending)||
                (descendingNext?!ContainsDescendingNextPanel(box):
                    fourthPanel?!(descendingFourth?ContainsFourthWholeVolume(box):ContainsFourthPanel(box)):
                    !(descendingThird?ContainsDescendingThirdPanel(box):ContainsThirdPanel(box))))
                return false;
        }
        else if(!originalRoster&&(!VariableC2Time::AdmitsGeometry(initial,rs,rc,box,true,true,true)||!Contains(box,true)))
            return false;
        const bool natural=!latePanel&&allowNatural&&ContainsNaturalPrefix(rs.GetPathForFixedSource().ControlPoints[0])&&
            box.Axis[0].Upper<=4982.3125&&box.Axis[1].Lower>=-1344.1875&&box.Axis[2].Upper<=97.;
        // The later 1940 branch still uses the original 1940 box for all
        // proved cells. Only the captured full H may extend beyond its top.
        const bool ordinary1940=!latePanel&&Contains(box,false);
        P clipped=box;
        clipped.Axis[2].Upper=FMath::Min(clipped.Axis[2].Upper,98.25);
        const bool sibling=!latePanel&&allowNatural&&!natural&&!ordinary1940&&box.Axis[2].Upper<=98.4375&&Contains(clipped,false);
        const bool nextPanel=!natural&&!sibling&&!ordinary1940;
        if(!latePanel&&((!originalRoster&&!nextPanel)||(nextPanel&&!Contains(box,true))))return false;
        const int32 provider=fourthPanel?870:thirdPanel?1098:nextPanel?2375:1940;
        const auto& path=rc.Partition->GetPathForFixedSource();
        if(path.Degree!=0||path.ControlPoints.Num()!=1||
            !C::GroupedBoxDetail::SamePoint(path.ControlPoints[0],
                C::BoundConsumedCenterBetweenFloatSeconds(initial.GetModel(),0.f,initial.GetHorizon())))return false;
        const auto indices=rs.GetCandidateIndices();
        int32 local=INDEX_NONE;
        for(int32 i=0;i<indices.Num();++i)if(indices[i]==provider)local=i;
        if(local==INDEX_NONE)return false;
        const auto& patch=rs.GetSurfacesForFixedSource()[local];
        if(patch.DegreeU!=5||patch.DegreeV!=5||patch.ControlPoints.Num()!=36)return false;
        const auto numeric=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
        if(numeric.Status!=EAnalyticNumericStatus::Complete)return false;
        if(latePanel)
        {
            const auto& reference=*initial.GetGeometry()->GetReference();
            const auto& expected=descendingNext?VariableC2Time::Faces[3]:VariableC2Time::SeamExtraFaces[fourthPanel?2:0];
            const FReferenceAffineExtrusionDescription* fact=nullptr;
            for(const auto& candidate:reference.GetExactAffineExtrusions(provider))
            {
                if(!account.Consume()||candidate.Provider!=provider||
                    candidate.CorrelationProofIdentity!=reference.GetExactAffineProofIdentity())return false;
                if(candidate.Product.FreeWorldAxis!=1)continue;
                if(fact||!candidate.Product.FreeAlongU||candidate.CanonicalRecordIdentity!=expected.Canonical||
                    candidate.BoundRecordIdentity!=expected.Bound)return false;
                fact=&candidate;
            }
            if(!fact||!fact->StructurallyMatches(patch,account))return false;
        }
        else
        {
            for(int32 u=0;u<6;++u)for(int32 v=0;v<6;++v)
            {
                if(!account.Consume())return false;
                const auto& q=patch.ControlPoints[6*u+v];
                if(!Equal(q.Axis[0],patch.ControlPoints[v].Axis[0])||
                    !Equal(q.Axis[1],patch.ControlPoints[6*u].Axis[1])||
                    !Equal(q.Axis[2],patch.ControlPoints[v].Axis[2]))return false;
            }
        }
        LoadProjectedCurve(patch);
        if(fourthPanel&&(descendingFourth||!ContainsOriginalFourthPanel(box)))
        {
            auto domain=C::TimeDetail::Hull(box,path.ControlPoints[0]);
            // Provide headroom for directed center/chord padding. Every
            // actual proof center is independently tested against this hull.
            constexpr double padding=0x1p-7;
            for(auto& axis:domain.Axis)axis=A::Add(axis,I{-padding,padding,true});
            if(!ProveFourthPanelConvexity(domain,account,256,descendingFourth))return false;
            FourthPanelConvexDomain=domain;
        }
        for(int32 k=0;k<6;++k)Y[k]=patch.ControlPoints[6*k].Axis[1];
        for(int32 k=0;k<5;++k)
        {
            YSlope[k]=Scale(5.,A::Subtract(Y[k+1],Y[k]));
            if(!A::Valid(YSlope[k])||YSlope[k].Upper>=0.)return false;
        }
        if(natural)
        {
            const int32 rival=indices.Find(695);
            if(!rs.GetSurfacesForFixedSource().IsValidIndex(rival))return false;
            const auto& wall=rs.GetSurfacesForFixedSource()[rival];
            if(!NaturalRival.Chart.Build(wall,account))return false;
            for(int32 v=0;v<6;++v)
            {
                if(!account.Consume())return false;
                for(int32 d=0;d<3;++d)
                    if(!Equal(wall.ControlPoints[30+v].Axis[d],patch.ControlPoints[v].Axis[d]))return false;
            }
        }
        if(sibling)
        {
            const int32 rival=indices.Find(2375);
            if(!rs.GetSurfacesForFixedSource().IsValidIndex(rival))return false;
            const auto& wall=rs.GetSurfacesForFixedSource()[rival];
            if(wall.DegreeU!=5||wall.DegreeV!=5||wall.ControlPoints.Num()!=36)return false;
            for(int32 u=0;u<6;++u)
            {
                for(int32 d=0;d<3;++d)
                    if(!Equal(wall.ControlPoints[6*u].Axis[d],patch.ControlPoints[6*u+5].Axis[d]))return false;
                for(int32 v=0;v<6;++v)
                {
                    if(!account.Consume())return false;
                    const auto& q=wall.ControlPoints[6*u+v];
                    if(!Equal(q.Axis[0],wall.ControlPoints[v].Axis[0])||
                        !Equal(q.Axis[1],wall.ControlPoints[6*u].Axis[1])||
                        !Equal(q.Axis[2],wall.ControlPoints[v].Axis[2]))return false;
                }
            }
            SiblingPoint=wall.ControlPoints[0];SiblingPoint.Axis[1]=I::Exact(0.);
            SiblingTangent.Axis[1]=I::Exact(0.);
            for(int32 d=0;d<3;d+=2)
                SiblingTangent.Axis[d]=Scale(5.,A::Subtract(wall.ControlPoints[1].Axis[d],wall.ControlPoints[0].Axis[d]));
            if(!C::Detail::ValidPoint(SiblingTangent))return false;
        }
        Initial=&initial;Rs=&rs;Rc=&rc;Surface=&patch;Local=local;NextPanel=nextPanel;Natural=natural;
        Sibling1940=sibling;ThirdPanel=thirdPanel;FourthPanel=fourthPanel;
        DescendingThirdPanel=descendingThird;DescendingNextPanel=descendingNext;
        if(fourthPanel)NextPanelEnd=7./8.;
        Numeric=numeric.ControlFingerprint();
        return IsLive();
    }
    bool IsLive() const
    {
        const auto numeric=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
        return Initial&&Rs&&Rc&&Surface&&Local!=INDEX_NONE&&numeric.Status==EAnalyticNumericStatus::Complete&&
            numeric.ControlFingerprint()==Numeric;
    }
    int32 LocalProvider() const{return Local;}
    int32 GlobalProvider() const{return FourthPanel?870:ThirdPanel?1098:NextPanel?2375:1940;}
    bool IsThirdPanel() const{return ThirdPanel;}
    bool IsDescendingThirdPanel() const{return DescendingThirdPanel;}
    bool IsDescendingNextPanel() const{return DescendingNextPanel;}
    bool IsFourthPanel() const{return FourthPanel;}
    bool BindWallLip(C::FDistanceBudget& account)
    {
        if(!FourthPanel||!IsLive()||!C::Detail::ValidPoint(FourthPanelConvexDomain))return false;
        const auto& state=Initial->GetCapturedInput().State;
        const auto& domain=FourthPanelConvexDomain;
        if(state.Velocity.X!=0.||state.Acceleration.X!=0.||state.Location.X<5026.||state.Location.X>=5027.||
            domain.Axis[2].Lower<159.||domain.Axis[2].Upper>164.||domain.Axis[2].Lower>=160.||
            domain.Axis[1].Lower<-1550.||domain.Axis[1].Upper>-1500.||XZ[0][0].Lower<=state.Location.X||
            XZ[1][0].Upper>=domain.Axis[2].Lower)return false;
        for(int32 k=0;k<6;++k)
        {
            if(!account.Consume()||!A::Valid(XZ[0][k])||!A::Valid(XZ[1][k]))return false;
            if(k&&((XZ[0][k-1].Upper>XZ[0][k].Lower)||!(XZ[1][k-1].Upper<XZ[1][k].Lower)))return false;
            if((k>=3&&!Equal(XZ[0][k],I::Exact(5120.)))||(k==5&&!Equal(XZ[1][k],I::Exact(160.))))return false;
            WallLipOffset[0][k]=k>=3?I::Exact(0.):A::Subtract(I::Exact(5120.),XZ[0][k]);
            WallLipOffset[1][k]=k==5?I::Exact(0.):A::Subtract(I::Exact(160.),XZ[1][k]);
        }
        WallLipX=state.Location.X;
        WallLip=true;
        return true;
    }
    bool ThirdPanelRetainsRc(TConstArrayView<int32> nativeIndices,C::FDistanceBudget& account) const
    {
        if((!ThirdPanel&&!FourthPanel&&!DescendingNextPanel)||!IsLive()||!Rc->UsesGeometry(Initial->GetGeometry()))return false;
        for(const int32 index:Rc->Partition->GetCandidateIndices())
            if(!account.Consume()||!nativeIndices.Contains(index))return false;
        return true;
    }
    bool IsNatural() const{return Natural;}
    bool IsSibling1940() const{return Sibling1940;}
    bool HasSeparateOrder() const{return Natural||Sibling1940;}
    int32 SeparateRivalProvider() const{return Sibling1940?2375:Natural?695:INDEX_NONE;}
    double EndLimit() const
    {
        if(Sibling1940)return 3./4.;
        return Natural?425./512.:NextPanel?NextPanelEnd:15./32.;
    }
    bool TryLimitNextPanel(C::FDistanceBudget& account)
    {
        constexpr int32 allowance=96;
        if(!NextPanel||NextPanelEnd!=1.||!IsLive()||
            account.Remaining-account.ConsumptionFloor<allowance)return false;
        C::FScopedDistanceBudgetFloor bounded(account,account.Remaining-allowance);
        if(!bounded.IsValid())return false;
        const auto& path=Rs->GetPathForFixedSource();
        for(int32 step=1;step<8;++step)
        {
            if(!account.Consume())return false;
            const double t=double(step)/8.;
            FVector2d uv;
            P point;
            I gap;
            // This is only an upper proof limit, supported by a strict
            // original-Rs inside witness. The original solver still finds
            // the first arrival and supplies its point/normal.
            if(Inside(C::CenterDetail::Evaluate(path,I::Exact(t)),Rs->GetRadiusForFixedSource(),account,uv,point,gap))
            {
                NextPanelEnd=t;
                return true;
            }
        }
        return false;
    }
    bool TryRaiseFourthPanelCap(C::FDistanceBudget& account)
    {
        constexpr int32 allowance=48;
        if(!FourthPanel||!IsLive()||account.Remaining-account.ConsumptionFloor<allowance)return false;
        C::FScopedDistanceBudgetFloor bounded(account,account.Remaining-allowance);
        if(!bounded.IsValid())return false;
        const auto& path=Rs->GetPathForFixedSource();
        constexpr double proposals[]={15./16.,1.};
        for(const double cap:proposals)
        {
            if(cap<=NextPanelEnd)continue;
            if(!account.Consume())return false;
            FVector2d uv;
            P point;
            I gap;
            // A new upper cap is merely a live reference inside proposal.
            // The original native/chord, rival and contact checks follow.
            if(Inside(C::CenterDetail::Evaluate(path,I::Exact(cap)),Rs->GetRadiusForFixedSource(),account,uv,point,gap))
            {
                NextPanelEnd=cap;
                return true;
            }
        }
        return false;
    }
    bool TryTightenNextPanel(C::FDistanceBudget& account)
    {
        constexpr int32 allowance=96;
        if(!NextPanel||!(NextPanelEnd>0.&&NextPanelEnd<1.)||!IsLive()||
            account.Remaining-account.ConsumptionFloor<allowance)return false;
        C::FScopedDistanceBudgetFloor bounded(account,account.Remaining-allowance);
        if(!bounded.IsValid())return false;
        const auto& path=Rs->GetPathForFixedSource();
        const double original=NextPanelEnd;
        // The previous eighth-grid sample was not proved inside. This lower
        // endpoint only guides proposals; it supplies no separation evidence.
        double lower=original-1./8.,upper=original;
        for(int32 step=0;step<3;++step)
        {
            if(!account.Consume())return false;
            const double t=.5*(lower+upper);
            FVector2d uv;
            P point;
            I gap;
            if(Inside(C::CenterDetail::Evaluate(path,I::Exact(t)),Rs->GetRadiusForFixedSource(),account,uv,point,gap))
                upper=t;
            else
                lower=t;
        }
        if(!(upper<original))return false;
        NextPanelEnd=upper;
        return true;
    }
    bool ContainsCenter(const P& center) const
    {
        if(FourthPanel)
        {
            if(!FourthPanelConvexDomain.Axis[0].Known)return ContainsOriginalFourthPanel(center);
            if(!C::Detail::ValidPoint(center))return false;
            for(int32 d=0;d<3;++d)
                if(center.Axis[d].Lower<FourthPanelConvexDomain.Axis[d].Lower||
                    center.Axis[d].Upper>FourthPanelConvexDomain.Axis[d].Upper)return false;
            return true;
        }
        if(DescendingNextPanel)return ContainsDescendingNextPanel(center);
        if(ThirdPanel)return DescendingThirdPanel?ContainsDescendingThirdPanel(center):ContainsThirdPanel(center);
        return Natural?ContainsNaturalPrefix(center):Contains(center,NextPanel);
    }
    double NaturalTimeGap() const{return A::Subtract(I::Exact(NaturalInsideEnd),I::Exact(NaturalClearEnd)).Upper;}
    bool NaturalReferenceClear(double end) const{return IsLive()&&NaturalClearEnd>0.&&end<=NaturalClearEnd;}
    bool NaturalReferenceBracket(double lower,double upper) const
    {
        return NaturalReferenceClear(lower)&&NaturalRcClearEnd>0.&&NaturalRcClearEnd<NaturalRcInsideEnd&&
            lower<upper&&NaturalClearEnd<upper&&upper<=NaturalInsideEnd;
    }
    bool SetThirdPanelPhysicalCap(double cap,C::FDistanceBudget& account)
    {
        if(!ThirdPanel||NextPanelEnd!=1.||!(cap>0.&&cap<1.)||!IsLive())return false;
        FVector2d uv;
        P point;
        I gap;
        const auto& path=Rs->GetPathForFixedSource();
        if(!account.Consume()||
            !Inside(C::CenterDetail::Evaluate(path,I::Exact(cap)),Rs->GetRadiusForFixedSource(),account,uv,point,gap))
            return false;
        NextPanelEnd=cap;
        return true;
    }
    bool SetFourthPanelPhysicalCap(double cap,C::FDistanceBudget& account)
    {
        if(!FourthPanel||!(cap>0.&&cap<1.)||!IsLive())return false;
        FVector2d uv;
        P point;
        I gap;
        const auto& path=Rs->GetPathForFixedSource();
        if(!account.Consume()||
            !Inside(C::CenterDetail::Evaluate(path,I::Exact(cap)),Rs->GetRadiusForFixedSource(),account,uv,point,gap))
            return false;
        NextPanelEnd=cap;
        return true;
    }
    bool TimeWithinCover(float end) const
    {
        if(!IsLive()||!FMath::IsFinite(end)||end<=0.f)return false;
        const auto max=A::Multiply(I::Exact(EndLimit()),I::Exact(double(Initial->GetHorizon())));
        return A::Valid(max)&&double(end)<=max.Lower;
    }
    bool Clear(const P (&centers)[3],double radius,C::FDistanceBudget& account) const
    {
        if(!IsLive())return false;
        for(const auto& center:centers)if(!ContainsCenter(center))return false;
        return (WallLip?ClearWallLip(centers,radius,account):ClearProjectedCell(*Surface,centers,radius,account))&&IsLive();
    }
    bool Inside(const P& center,double radius,C::FDistanceBudget& account,FVector2d& uv,P& point,I& gap,
        bool useOriginalLiteralIntervals=false) const
    {
        if(!IsLive()||!ProposeV(center,account,uv.Y))return false;
        const double y=Middle(center.Axis[1]);
        const double slope=Middle(YSlope[0]);
        if(!A::Finite(slope)||!(slope<0.))return false;
        uv.X=(y-Middle(Y[0]))/slope;
        for(int32 i=0;i<4;++i)
        {
            if(!account.Consume()||!A::Finite(uv.X)||uv.X<=0.||uv.X>=1.)return false;
            const double d=Middle(Eval(YSlope,uv.X));
            if(!A::Finite(d)||d>=0.)return false;
            uv.X-=(Middle(Eval(Y,uv.X))-y)/d;
        }
        if(!A::Finite(uv.X)||uv.X<=0.||uv.X>=1.||
            !VariableC2Time::Sample(*Surface,uv,account,point)||!account.Consume())return false;
        if(WallLip&&!useOriginalLiteralIntervals)
            gap=A::Add(WallLipDistance(center,uv.Y,radius),A::Square(A::Subtract(point.Axis[1],center.Axis[1])));
        else
        {
            gap=I::Exact(0.);
            for(int32 d=0;d<3;++d)gap=A::Add(gap,A::Square(A::Subtract(point.Axis[d],center.Axis[d])));
            gap=A::Subtract(gap,A::Square(I::Exact(radius)));
        }
        return A::Valid(gap)&&gap.Upper<0.&&IsLive();
    }
    bool CoverNatural(double end,double radius,const I* error,bool winner,C::FDistanceBudget& account) const
    {
        if(!IsLive()||!HasSeparateOrder()||!(end>=0.&&end<=EndLimit()))return false;
        struct FCell {double A,B;int32 Depth;};
        TArray<FCell,TInlineAllocator<32>> cells;
        cells.Add({0.,end,0});
        const auto& path=Rs->GetPathForFixedSource();
        int32 visits=0;
        while(!cells.IsEmpty())
        {
            if(++visits>64)return false;
            const auto cell=cells.Pop(EAllowShrinking::No);
            P centers[3];
            if(!VariableC2Time::Restrict(path,cell.A,cell.B,account,centers))return false;
            if(error)
                for(auto& center:centers)for(int32 d=0;d<3;++d)center.Axis[d]=A::Add(center.Axis[d],error[d]);
            if(winner?Clear(centers,radius,account):ProveSeparateRival(centers,radius,account))continue;
            const double middle=.5*(cell.A+cell.B);
            if(cell.Depth>=28||!(cell.A<middle&&middle<cell.B))return false;
            cells.Add({middle,cell.B,cell.Depth+1});
            cells.Add({cell.A,middle,cell.Depth+1});
        }
        return IsLive();
    }
    bool ProveNaturalOrder(C::FDistanceBudget& account)
    {
        constexpr int32 scale=1<<22;
        const int32 allowance=Sibling1940?896:2304;
        if(!HasSeparateOrder()||!IsLive()||account.Remaining-account.ConsumptionFloor<allowance)return false;
        C::FScopedDistanceBudgetFloor bounded(account,account.Remaining-allowance);
        const auto& values=Initial->GetCapturedInput();
        const auto& path=Rs->GetPathForFixedSource();
        const double radii[]={values.SupportRadius,double(values.QueryRadius)};
        I error[3];
        if(!bounded.IsValid()||!account.Consume()||
            !VariableC2RoundedPrefix::CenterError(values,double(Initial->GetHorizon()),error))return false;
        double lows[2],highs[2];
        for(int32 i=0;i<2;++i)
        {
            const auto inside=[&](int32 at)
            {
                FVector2d uv;
                P point;
                I gap;
                return Inside(C::CenterDetail::Evaluate(path,I::Exact(double(at)/scale)),radii[i],account,uv,point,gap);
            };
            int32 lo=Sibling1940?5*scale/8:3*scale/4,hi=Sibling1940?3*scale/4:425*scale/512;
            if(!inside(hi))return false;
            while(hi-lo>1)
            {
                const int32 middle=lo+(hi-lo)/2;
                if(inside(middle))hi=middle;else lo=middle;
            }
            lows[i]=double(lo)/scale;highs[i]=double(hi)/scale;
            const auto floor=C::FloorReferenceTimeToFloatSeconds(Initial->GetModel(),lows[i]);
            if(!floor.Known||floor.Seconds<=0.f||
                !CoverNatural(lows[i],radii[i],nullptr,true,account)||
                !CoverNatural(highs[i],radii[i],nullptr,false,account))return false;
            if(i==1)
            {
                I consumed;
                if(!C::ContactDetail::ProjectionDividePositive(I::Exact(double(floor.Seconds)),
                    I::Exact(double(Initial->GetHorizon())),consumed)||
                    !CoverNatural(consumed.Upper,radii[i],error,true,account)||
                    !CoverNatural(consumed.Upper,radii[i],error,false,account))return false;
            }
        }
        NaturalClearEnd=lows[0];NaturalInsideEnd=highs[0];
        NaturalRcClearEnd=lows[1];NaturalRcInsideEnd=highs[1];
        return IsLive();
    }
    bool Rounded(float end,C::FDistanceBudget& account) const
    {
        const int32 allowance=Sibling1940?192:Natural?768:NextPanel?40:24;
        if(!TimeWithinCover(end)||account.Remaining-account.ConsumptionFloor<allowance)return false;
        C::FScopedDistanceBudgetFloor bounded(account,account.Remaining-allowance);
        const auto& values=Initial->GetCapturedInput();
        I error[3];
        if(!bounded.IsValid()||!account.Consume()||!VariableC2RoundedPrefix::CenterError(values,double(end),error))return false;
        const double seconds=double(end);
        if(HasSeparateOrder())
        {
            I normalized;
            return C::ContactDetail::ProjectionDividePositive(I::Exact(seconds),I::Exact(double(Initial->GetHorizon())),normalized)&&
                CoverNatural(normalized.Upper,values.SupportRadius,error,true,account)&&
                CoverNatural(normalized.Upper,values.SupportRadius,error,false,account);
        }
        const int32 count=NextPanel?4:2;
        for(int32 cell=0;cell<count;++cell)
        {
            const double a=seconds*(double(cell)/count),b=seconds*(double(cell+1)/count);
            const double ends[3][2]={{a,a},{a,b},{b,b}};
            P centers[3];
            for(int32 k=0;k<3;++k)for(int32 d=0;d<3;++d)
            {
                const auto s0=I::Exact(ends[k][0]),s1=I::Exact(ends[k][1]);
                const auto linear=A::Multiply(Scale(.5,I::Exact(values.State.Velocity[d])),A::Add(s0,s1));
                const auto quadratic=A::Multiply(Scale(.5,I::Exact(values.State.Acceleration[d])),A::Multiply(s0,s1));
                centers[k].Axis[d]=A::Add(A::Add(A::Add(I::Exact(values.State.Location[d]),linear),quadratic),error[d]);
            }
            if(!Clear(centers,values.SupportRadius,account))return false;
        }
        return IsLive();
    }
    bool Fresh(const FPreparedGeometrySelectionPtr& geometry,const FStaticSphereInitialValues& captured,
        FVector3d offset,FVector3d com,float horizon,float end,const FVector3d& actual,const P& original,
        C::FDistanceBudget& account) const
    {
        const int32 allowance=Sibling1940?16:Natural?64:12;
        if(!TimeWithinCover(end)||geometry!=Initial->GetGeometry()||!Rc->UsesGeometry(geometry)||
            account.Remaining-account.ConsumptionFloor<allowance)return false;
        const auto& values=Initial->GetCapturedInput();
        const auto same=[](const FVector3d& a,const FVector3d& b)
        {
            for(int32 d=0;d<3;++d)if(A::Bits(a[d])!=A::Bits(b[d]))return false;
            return true;
        };
        const auto bits=C::KinematicCenterDetail::Bits;
        if(bits(horizon)!=bits(Initial->GetHorizon())||!same(offset,Initial->GetLocalOffset())||
            !same(com,Initial->GetLocalCOM())||!same(captured.State.Location,values.State.Location)||
            !same(captured.State.Velocity,values.State.Velocity)||!same(captured.State.Acceleration,values.State.Acceleration)||
            A::Bits(captured.SupportRadius)!=A::Bits(values.SupportRadius)||bits(captured.QueryRadius)!=bits(values.QueryRadius)||
            bits(captured.PhysicalRadius)!=bits(values.PhysicalRadius)||bits(captured.StaticContactOffset)!=bits(values.StaticContactOffset)||
            bits(captured.ActualMargin)!=bits(values.ActualMargin)||
            !C::GroupedBoxDetail::SamePoint(original,C::BoundConsumedCenterAtFloatSeconds(Initial->GetModel(),end))||
            !C::ReferenceSphereDetail::Contains(original,actual)||
            !C::ReferenceSphereDetail::Contains(Rc->Partition->GetPathForFixedSource().ControlPoints[0],actual))return false;
        C::FScopedDistanceBudgetFloor bounded(account,account.Remaining-allowance);
        const auto p=C::Detail::Point(actual);
        const P centers[3]={p,p,p};
        if(!bounded.IsValid()||!account.Consume()||!Clear(centers,values.SupportRadius,account))return false;
        return !HasSeparateOrder()||ProveSeparateRival(centers,values.SupportRadius,account);
    }
};
}
