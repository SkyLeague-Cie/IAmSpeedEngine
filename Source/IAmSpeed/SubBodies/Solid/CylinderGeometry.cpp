#include "CylinderGeometry.h"
#include <limits>
namespace Speed::Geometry
{
bool FCylinderGeometry::IsValid() const
{return FMath::IsFinite(Radius)&&FMath::IsFinite(HalfHeight)&&Radius>0.&&HalfHeight>0.;}
FVector3d FCylinderGeometry::Support(const FVector3d& D) const
{
    const double R=FMath::Sqrt(D.X*D.X+D.Y*D.Y);
    return FVector3d(R>1.e-15 ? Radius*D.X/R : 0.,R>1.e-15 ? Radius*D.Y/R : 0.,D.Z>0. ? HalfHeight : (D.Z<0. ? -HalfHeight : 0.));
}
bool TryCylinderSphereContact(const FCylinderGeometry& C,const FVector3d& P,double R,FCylinderContact& Out)
{
    Out={};if(!C.IsValid()||P.ContainsNaN()||!FMath::IsFinite(R)||R<0.)return false;
    const double Radial=FMath::Sqrt(P.X*P.X+P.Y*P.Y),Side=Radial-C.Radius,Cap=FMath::Abs(P.Z)-C.HalfHeight;
    const FVector3d RadialNormal=Radial>1.e-15 ? FVector3d(P.X/Radial,P.Y/Radial,0.) : FVector3d(1.,0.,0.);
    const double Sign=P.Z>=0. ? 1. : -1.;double D=0.;
    if(Side>0. || Cap>0.)
    {
        Out.CylinderPoint=FVector3d(Radial>C.Radius ? RadialNormal.X*C.Radius : P.X,Radial>C.Radius ? RadialNormal.Y*C.Radius : P.Y,FMath::Clamp(P.Z,-C.HalfHeight,C.HalfHeight));
        const FVector3d V=P-Out.CylinderPoint;D=V.Size();Out.Normal=V.GetSafeNormal(1.e-15);
    }
    else if(Side>=Cap) {D=Side;Out.Normal=RadialNormal;Out.CylinderPoint=RadialNormal*C.Radius+FVector3d(0.,0.,P.Z);}
    else {D=Cap;Out.Normal=FVector3d(0.,0.,Sign);Out.CylinderPoint=FVector3d(P.X,P.Y,Sign*C.HalfHeight);}
    Out.ShapePoint=P-Out.Normal*R;Out.LowerSeparation=Out.UpperSeparation=D-R;Out.bConverged=true;return true;
}
namespace
{
// Cartesian product of an exact disk and axial interval against a box
// whose axis is parallel to the cylinder. This also handles symmetric deep
// overlaps without forcing EPA to approximate a circular continuum of faces.
void CylinderAlignedBoxContact(const FCylinderGeometry& C,const FVector3d& P,const FVector3d& E,FCylinderContact& Out)
{
    const double SX=P.X>=0.?1.:-1.,SY=P.Y>=0.?1.:-1.,SZ=P.Z>=0.?1.:-1.;
    const double X=FMath::Abs(P.X)-E.X,Y=FMath::Abs(P.Y)-E.Y;
    FVector3d Axy(SX*FMath::Max(X,0.),SY*FMath::Max(Y,0.),0.),N;
    double Radial;
    if(X>0.||Y>0.) {const double L=Axy.Size();N=Axy/L;Radial=L-C.Radius;}
    else if(X>=Y) {N=FVector3d(SX,0.,0.);Axy=FVector3d(P.X-SX*E.X,0.,0.);Radial=X-C.Radius;}
    else {N=FVector3d(0.,SY,0.);Axy=FVector3d(0.,P.Y-SY*E.Y,0.);Radial=Y-C.Radius;}
    const double Axial=FMath::Abs(P.Z)-E.Z-C.HalfHeight;
    if(Radial>0.&&Axial>0.)
    {
        const double D=FMath::Sqrt(Radial*Radial+Axial*Axial);
        Out.Normal=(N*Radial+FVector3d(0.,0.,SZ*Axial))/D;
        Out.CylinderPoint=N*C.Radius+FVector3d(0.,0.,SZ*C.HalfHeight);
        Out.ShapePoint=Axy+FVector3d(0.,0.,P.Z-SZ*E.Z);
        Out.LowerSeparation=Out.UpperSeparation=D;
    }
    else if(Radial>=Axial)
    {
        const double Z=FMath::Clamp(P.Z,-C.HalfHeight,C.HalfHeight);
        Out.Normal=N;Out.CylinderPoint=N*C.Radius+FVector3d(0.,0.,Z);Out.ShapePoint=Axy+FVector3d(0.,0.,Z);
        Out.LowerSeparation=Out.UpperSeparation=Radial;
    }
    else
    {
        // Closest rectangle point to the axis is inside the disk here.
        const FVector3d Common(SX*FMath::Max(X,0.),SY*FMath::Max(Y,0.),0.);
        Out.Normal=FVector3d(0.,0.,SZ);Out.CylinderPoint=Common+FVector3d(0.,0.,SZ*C.HalfHeight);
        Out.ShapePoint=Common+FVector3d(0.,0.,P.Z-SZ*E.Z);
        Out.LowerSeparation=Out.UpperSeparation=Axial;
    }
    Out.bConverged=true;
}
struct FCylinderSupport {FVector3d V,A,B;};
struct FCylinderClosest {FVector3d V=FVector3d::ZeroVector;double Weights[4]={};uint8 Mask=0;double DistanceSquared=std::numeric_limits<double>::max();};
// Enumerate every independent simplex face; no iteration-order projection or
// sampling oracle. At most15 candidates, with an explicit rank check.
FCylinderClosest CylinderClosest(TConstArrayView<FCylinderSupport> S)
{
    FCylinderClosest Best;
    for(uint32 Mask=1;Mask<(1u<<S.Num());++Mask)
    {
        int32 Id[4]={},N=0;for(int32 I=0;I<S.Num();++I)if(Mask&(1u<<I))Id[N++]=I;
        double W[4]={};W[0]=1.;const auto A=S[Id[0]].V;
        if(N==2)
        {const auto U=S[Id[1]].V-A;const double Den=U.SizeSquared();if(Den<1.e-24)continue;W[1]=-FVector3d::DotProduct(A,U)/Den;W[0]=1.-W[1];}
        else if(N==3)
        {
            const auto U=S[Id[1]].V-A,V=S[Id[2]].V-A;
            // Cross-product barycentrics avoid subtracting nearly equal
            // Gram products at a true tangent. Any accepted nonnegative
            // weights still give a genuine convex witness, independently
            // certified by the support-plane lower separation.
            const auto Normal=FVector3d::CrossProduct(U,V);const double Den=Normal.SizeSquared();
            if(Den<=1.e-28*U.SizeSquared()*V.SizeSquared())continue;
            W[1]=FVector3d::DotProduct(FVector3d::CrossProduct(-A,V),Normal)/Den;
            W[2]=FVector3d::DotProduct(FVector3d::CrossProduct(U,-A),Normal)/Den;W[0]=1.-W[1]-W[2];
        }
        else if(N==4)
        {
            const auto U=S[Id[1]].V-A,V=S[Id[2]].V-A,T=S[Id[3]].V-A;
            const double Den=FVector3d::DotProduct(U,FVector3d::CrossProduct(V,T));
            if(FMath::Abs(Den)<=1.e-12*FMath::Sqrt(U.SizeSquared()*V.SizeSquared()*T.SizeSquared()))continue;
            W[1]=FVector3d::DotProduct(-A,FVector3d::CrossProduct(V,T))/Den;
            W[2]=FVector3d::DotProduct(U,FVector3d::CrossProduct(-A,T))/Den;
            W[3]=FVector3d::DotProduct(U,FVector3d::CrossProduct(V,-A))/Den;W[0]=1.-W[1]-W[2]-W[3];
        }
        bool Valid=true;FVector3d P=FVector3d::ZeroVector;double Total=0.;
        for(int32 I=0;I<N;++I){Valid&=FMath::IsFinite(W[I])&&W[I]>=-1.e-10;W[I]=FMath::Max(0.,W[I]);Total+=W[I];}
        if(!Valid||Total<=0.)continue;
        for(int32 I=0;I<N;++I){W[I]/=Total;P+=S[Id[I]].V*W[I];}
        const double D=P.SizeSquared();if(D>=Best.DistanceSquared)continue;
        Best={};Best.V=P;Best.Mask=uint8(Mask);Best.DistanceSquared=D;
        for(int32 I=0;I<N;++I)Best.Weights[Id[I]]=W[I];
    }
    return Best;
}
void CylinderWitnesses(TConstArrayView<FCylinderSupport> S,const FCylinderClosest& K,FCylinderContact& Out)
{Out.ShapePoint=Out.CylinderPoint=FVector3d::ZeroVector;for(int32 I=0;I<S.Num();++I){Out.ShapePoint+=S[I].A*K.Weights[I];Out.CylinderPoint+=S[I].B*K.Weights[I];}}
}
bool TryCylinderBoxContact(const FCylinderGeometry& C,const FVector3d& P,const FQuat4d& Q,const FVector3d& E,
    FCylinderContact& Out,double Tol,int32 MaxIterations)
{
    Out={};if(!C.IsValid()||P.ContainsNaN()||Q.ContainsNaN()||!Q.IsNormalized()||E.ContainsNaN()||E.GetMin()<=0.||!FMath::IsFinite(Tol)||Tol<=0.||MaxIterations<4||MaxIterations>256)return false;
    // Exact test, not an angular epsilon: no tilted OBB is snapped to axial.
    if(Q.X==0.&&Q.Y==0.)
    {
        CylinderAlignedBoxContact(C,Q.UnrotateVector(P),E,Out);
        Out.Normal=Q.RotateVector(Out.Normal);Out.CylinderPoint=Q.RotateVector(Out.CylinderPoint);Out.ShapePoint=Q.RotateVector(Out.ShapePoint);return true;
    }
    auto Support=[&](const FVector3d& D)
    {
        const auto L=Q.UnrotateVector(D);const FVector3d Local(L.X>0.?E.X:(L.X<0.?-E.X:0.),L.Y>0.?E.Y:(L.Y<0.?-E.Y:0.),L.Z>0.?E.Z:(L.Z<0.?-E.Z:0.));
        FCylinderSupport S;S.A=P+Q.RotateVector(Local);S.B=C.Support(-D);S.V=S.A-S.B;return S;
    };
    TArray<FCylinderSupport> Simplex;Simplex.Reserve(4);Simplex.Add(Support(P.IsNearlyZero(1.e-12)?FVector3d(1.,0.,0.):-P));FCylinderClosest Near;
    bool Intersect=false;
    for(int32 Iter=0;Iter<MaxIterations;++Iter)
    {
        ++Out.Iterations;Near=CylinderClosest(Simplex);if(!Near.Mask)return true;
        const double Upper=FMath::Sqrt(Near.DistanceSquared);
        if(Upper<=1.e-12){Intersect=true;break;}
        const auto Next=Support(-Near.V);const double Lower=FVector3d::DotProduct(Near.V,Next.V)/Upper;
        if(Upper-Lower<=Tol)
        {CylinderWitnesses(Simplex,Near,Out);Out.Normal=Near.V/Upper;Out.LowerSeparation=Lower;Out.UpperSeparation=Upper;Out.bConverged=true;return true;}
        TArray<FCylinderSupport> Reduced;Reduced.Reserve(4);for(int32 I=0;I<Simplex.Num();++I)if(Near.Weights[I]>0.)Reduced.Add(Simplex[I]);
        bool Duplicate=false;for(const auto& S:Reduced)Duplicate|=(S.V-Next.V).SizeSquared()<1.e-24;
        if(Duplicate||Reduced.Num()>=4)return true;
        Reduced.Add(Next);Simplex=MoveTemp(Reduced);
    }
    if(!Intersect)return true;
    // EPA starts from an origin-containing tetrahedron, including boundary
    // contact. Extra directions repair a line/triangle GJK simplex; finite cap.
    TArray<FCylinderSupport> Seeds=Simplex;
    auto AddSeed=[&](FVector3d D){const auto S=Support(D);for(const auto& X:Seeds)if((X.V-S.V).SizeSquared()<1.e-20)return;Seeds.Add(S);};
    for(int Axis=0;Axis<3;++Axis)for(double Sign:{-1.,1.}){FVector3d D=FVector3d::ZeroVector;D[Axis]=Sign;AddSeed(D);}
    for(double X:{-1.,1.})for(double Y:{-1.,1.})for(double Z:{-1.,1.})AddSeed(FVector3d(X,Y,Z));
    TArray<FCylinderSupport> Vertices;bool Found=false;double BestMinimum=-1.;
    for(int32 A=0;A<Seeds.Num();++A)for(int32 B=A+1;B<Seeds.Num();++B)for(int32 D=B+1;D<Seeds.Num();++D)for(int32 F=D+1;F<Seeds.Num();++F)
    {
        const FCylinderSupport T[4]={Seeds[A],Seeds[B],Seeds[D],Seeds[F]};
        const auto O=T[0].V,U=T[1].V-O,V=T[2].V-O,W=T[3].V-O;
        const double Det=FVector3d::DotProduct(U,FVector3d::CrossProduct(V,W));
        if(FMath::Abs(Det)<=1.e-12*FMath::Sqrt(U.SizeSquared()*V.SizeSquared()*W.SizeSquared()))continue;
        const double B1=FVector3d::DotProduct(-O,FVector3d::CrossProduct(V,W))/Det,
            B2=FVector3d::DotProduct(U,FVector3d::CrossProduct(-O,W))/Det,
            B3=FVector3d::DotProduct(U,FVector3d::CrossProduct(V,-O))/Det,B0=1.-B1-B2-B3;
        const double Minimum=FMath::Min(FMath::Min(B0,B1),FMath::Min(B2,B3));
        if(Minimum<-1.e-12)continue;
        if(Minimum<=BestMinimum)continue;BestMinimum=Minimum;Vertices.Reset();Vertices.Append(T,4);Found=true;
        if(Minimum>1.e-4)goto TetraFound;
    }
TetraFound:
    if(!Found)return true;
    const FVector3d Interior=(Vertices[0].V+Vertices[1].V+Vertices[2].V+Vertices[3].V)*.25;
    struct FFace {int32 A,B,C;FVector3d N;double D;};TArray<FFace> Faces;
    auto Face=[&](int32 A,int32 B,int32 D)
    {
        auto N=FVector3d::CrossProduct(Vertices[B].V-Vertices[A].V,Vertices[D].V-Vertices[A].V).GetSafeNormal(1.e-15);
        if(N.IsZero()||Faces.Num()>=256)return false;if(FVector3d::DotProduct(N,Vertices[A].V-Interior)<0.){Swap(B,D);N=-N;}
        const double Distance=FVector3d::DotProduct(N,Vertices[A].V);if(Distance<-Tol)return false;
        Faces.Add({A,B,D,N,FMath::Max(0.,Distance)});return true;
    };
    if(!Face(0,1,2)||!Face(0,3,1)||!Face(0,2,3)||!Face(1,3,2))return true;
    for(int32 Iter=0;Iter<MaxIterations;++Iter)
    {
        ++Out.Iterations;int32 Best=0;for(int32 I=1;I<Faces.Num();++I)if(Faces[I].D<Faces[Best].D)Best=I;
        const auto F=Faces[Best];const auto S=Support(F.N);const double Upper=FVector3d::DotProduct(S.V,F.N);
        if(Upper-F.D<=Tol)
        {
            const FCylinderSupport T[3]={Vertices[F.A],Vertices[F.B],Vertices[F.C]};const auto K=CylinderClosest(MakeArrayView(T));if(!K.Mask)return true;
            CylinderWitnesses(MakeArrayView(T),K,Out);Out.Normal=-F.N;Out.LowerSeparation=-Upper;Out.UpperSeparation=-F.D;Out.bConverged=true;return true;
        }
        if(Vertices.Num()>=128||Faces.Num()>=256)return true;
        struct FEdge{int32 A,B;};TArray<FEdge> Boundary;
        auto Edge=[&](int32 A,int32 B){for(int32 I=0;I<Boundary.Num();++I)if(Boundary[I].A==B&&Boundary[I].B==A){Boundary.RemoveAt(I);return;}Boundary.Add({A,B});};
        for(int32 I=Faces.Num()-1;I>=0;--I)if(FVector3d::DotProduct(Faces[I].N,S.V)-Faces[I].D>Tol*.01)
        {const auto X=Faces[I];Edge(X.A,X.B);Edge(X.B,X.C);Edge(X.C,X.A);Faces.RemoveAt(I);}
        if(Boundary.IsEmpty())return true;const int32 Index=Vertices.Add(S);
        for(const auto& E0:Boundary)if(!Face(E0.A,E0.B,Index))return true;
    }
    return true;
}
}
