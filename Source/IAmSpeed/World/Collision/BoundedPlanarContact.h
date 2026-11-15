#pragma once
#include "PlanarContactImpulse.h"

namespace Speed {
/** Static planar contact with accumulated Coulomb impulses AND speed-limit
 * reactions in the same iteration. Limit reactions are external game
 * constraints; they are exported separately from contact impulses.
 * ComplianceSeconds is a material response horizon, never the frame dt.
 * Effective k=m_eff/T^2,c=2*m_eff/T give CFM=Knormal/3 at horizon T.
 * A nonpenetration floor remains active at actual CCD contact. Separated
 * candidates obey vn>=-gap/T and therefore cannot be labelled touching.
 */
struct FBoundedPlanarContact {
 FPlanarNormalImpulse Contact;
 FVector LimitDeltaVelocity=FVector::ZeroVector;
 FVector LimitDeltaAngularVelocity=FVector::ZeroVector;
 double MaximumResidual=0;
};
namespace BoundedContact {
inline bool Linear3(const double A[3][3],const FVector& B,FVector& X){
 double M[3][4]={{A[0][0],A[0][1],A[0][2],B.X},{A[1][0],A[1][1],A[1][2],B.Y},{A[2][0],A[2][1],A[2][2],B.Z}};
 for(int C=0;C<3;++C){int P=C;for(int R=C+1;R<3;++R)if(FMath::Abs(M[R][C])>FMath::Abs(M[P][C]))P=R;if(!FMath::IsFinite(M[P][C])||FMath::Abs(M[P][C])<1e-30)return false;for(int J=C;J<4;++J)Swap(M[P][J],M[C][J]);double D=M[C][C];for(int J=C;J<4;++J)M[C][J]/=D;for(int R=0;R<3;++R)if(R!=C){double F=M[R][C];for(int J=C;J<4;++J)M[R][J]-=F*M[C][J];}}
 X=FVector(M[0][3],M[1][3],M[2][3]);return !X.ContainsNaN();
}
inline bool ProjectAngular(const FVector& W,const FMatrix& InvI,double Cap,FVector& Out){
 if(W.SizeSquared()<=Cap*Cap){Out=W;return true;}
 auto Eval=[&](double Lambda,FVector& V){double A[3][3];for(int R=0;R<3;++R)for(int C=0;C<3;++C)A[R][C]=(R==C?1.:0.)+Lambda*InvI.M[C][R];return Linear3(A,W,V);};
 double Lo=0,Hi=1/FMath::Max(1e-30,FMath::Max(InvI.M[0][0],FMath::Max(InvI.M[1][1],InvI.M[2][2])));FVector V;
 int K=0;for(;K<64;++K){if(!Eval(Hi,V))return false;if(V.SizeSquared()<=Cap*Cap)break;Hi*=2;}
 if(K==64)return false;
 for(K=0;K<64;++K){double M=(Lo+Hi)*.5;if(!Eval(M,V))return false;if(V.SizeSquared()>Cap*Cap)Lo=M;else Hi=M;}
 if(!Eval(Hi,Out))return false;
 // Only remove floating roundoff at the numerical boundary of the ball.
 if(Out.SizeSquared()>Cap*Cap)Out*=Cap/Out.Size();
 return true;
}
}
inline FBoundedPlanarContact SolveBoundedPlanarContact(
 TConstArrayView<FVector> Points,TConstArrayView<double> Gaps,const FVector& N,
 const FVector& COM,const FVector& V0,const FVector& W0,double InvMass,
 const FMatrix& InvI,double Restitution,double Friction,double LinearCap,double AngularCap,
 double ComplianceSeconds)
{
 FBoundedPlanarContact Out;const int Count=Points.Num();
 if(Count<1||Count>4||Gaps.Num()!=Count||!FMath::IsFinite(InvMass)||!FMath::IsFinite(LinearCap)||!FMath::IsFinite(AngularCap)||!FMath::IsFinite(ComplianceSeconds)||!FMath::IsFinite(Restitution)||!FMath::IsFinite(Friction)||InvMass<=0||LinearCap<=0||AngularCap<=0||ComplianceSeconds<0||Restitution<0||Restitution>1||Friction<0||V0.ContainsNaN()||W0.ContainsNaN()||N.ContainsNaN()||COM.ContainsNaN()||FMath::Abs(N.SizeSquared()-1)>1e-10)return Out;
 for(int A=0;A<3;++A)for(int B=0;B<3;++B)if(!FMath::IsFinite(InvI.M[A][B])||FMath::Abs(InvI.M[A][B]-InvI.M[B][A])>1e-12*FMath::Max(1.,FMath::Abs(InvI.M[A][B])))return Out;
 const double D2=InvI.M[0][0]*InvI.M[1][1]-InvI.M[0][1]*InvI.M[1][0];
 const double D3=InvI.M[0][0]*(InvI.M[1][1]*InvI.M[2][2]-InvI.M[1][2]*InvI.M[2][1])-InvI.M[0][1]*(InvI.M[1][0]*InvI.M[2][2]-InvI.M[1][2]*InvI.M[2][0])+InvI.M[0][2]*(InvI.M[1][0]*InvI.M[2][1]-InvI.M[1][1]*InvI.M[2][0]);
 if(InvI.M[0][0]<=0||D2<=0||D3<=0)return Out;
 for(int A=0;A<Count;++A)if(Points[A].ContainsNaN()||!FMath::IsFinite(Gaps[A]))return Out;
 FVector U,T;N.FindBestAxisVectors(U,T);FVector R[4],RN[4],AN[4],JT[4]={};double KN[4],K00[4],K01[4],K11[4],Det[4],CFM[4],Floor[4],Target[4],JN[4]={};
 const double Horizon=ComplianceSeconds>0?ComplianceSeconds:1.;
 for(int I=0;I<Count;++I){R[I]=Points[I]-COM;RN[I]=FVector::CrossProduct(R[I],N);AN[I]=InvI.TransformVector(RN[I]);KN[I]=InvMass+FVector::DotProduct(RN[I],AN[I]);FVector RU=FVector::CrossProduct(R[I],U),RT=FVector::CrossProduct(R[I],T);K00[I]=InvMass+FVector::DotProduct(RU,InvI.TransformVector(RU));K01[I]=FVector::DotProduct(RU,InvI.TransformVector(RT));K11[I]=InvMass+FVector::DotProduct(RT,InvI.TransformVector(RT));Det[I]=K00[I]*K11[I]-K01[I]*K01[I];if(KN[I]<=0||Det[I]<=0||!FMath::IsFinite(Det[I]))return Out;CFM[I]=ComplianceSeconds>0?KN[I]/3.:0.;Floor[I]=-FMath::Max(0.,Gaps[I])/Horizon;Target[I]=Floor[I]-Restitution*FMath::Min(0.,FVector::DotProduct(V0+FVector::CrossProduct(W0,R[I]),N));}
 // Preserve exact plastic edge/face solutions when they satisfy this SAME
 // contact problem and inactive speed bounds. PGS convergence alone cannot
 // certify exact zero modes; its finite residual otherwise sustains rocking.
 // Separated candidate rows may carry no impulse and must remain feasible.
 if(ComplianceSeconds==0 || Restitution==0){
  for(int Pass=0;Pass<2;++Pass){
   FVector ExactPoints[4];int Map[4],Num=0;
   for(int I=0;I<Count;++I)if(Pass==0 || Gaps[I]<=0){Map[Num]=I;ExactPoints[Num++]=Points[I];}
   if(Num==0)continue;
   const auto Exact=SolvePlanarCoulombImpulse(MakeArrayView(ExactPoints,Num),N,COM,V0,W0,InvMass,InvI,Restitution,Friction);
   if(!Exact.bSolved)continue;
   const FVector EV=V0+Exact.DeltaVelocity,EW=W0+Exact.DeltaAngularVelocity;
   if(EV.SizeSquared()>LinearCap*LinearCap || EW.SizeSquared()>AngularCap*AngularCap)continue;
   double J[4]={};FVector Tangent[4]={};for(int I=0;I<Num;++I){J[Map[I]]=Exact.Impulses[I];Tangent[Map[I]]=Exact.TangentImpulses[I];}
   const double Eps=4096*DBL_EPSILON*FMath::Max(1.,V0.Size()+W0.Size()*R[0].Size());bool Valid=true;
   for(int I=0;I<Count;++I){
    const double VN=FVector::DotProduct(EV+FVector::CrossProduct(EW,R[I]),N);
    const double Desired=FMath::Max(Floor[I],Target[I]-CFM[I]*J[I]);
    Valid &= J[I]>0 ? FMath::Abs(VN-Desired)<=Eps : VN>=Desired-Eps;
   }
   if(!Valid)continue;
   Out.Contact=Exact;for(int I=0;I<Count;++I){Out.Contact.Impulses[I]=J[I];Out.Contact.TangentImpulses[I]=Tangent[I];}
   return Out;
  }
 }
 FVector V=V0,W=W0,CapV=FVector::ZeroVector,CapW=FVector::ZeroVector;
 const double Tol=2e-11*FMath::Max(1.,V0.Size()+W0.Size()*R[0].Size());
 for(int Iter=0;Iter<512;++Iter){const FVector OldV=V,OldW=W;
  for(int I=0;I<Count;++I){double VN=FVector::DotProduct(V+FVector::CrossProduct(W,R[I]),N);double Next=FMath::Max(0.,FMath::Max(JN[I]+(Target[I]-VN-CFM[I]*JN[I])/(KN[I]+CFM[I]),JN[I]+(Floor[I]-VN)/KN[I]));double D=Next-JN[I];JN[I]=Next;V+=N*(InvMass*D);W+=AN[I]*D;}
  for(int I=0;I<Count;++I){FVector VP=V+FVector::CrossProduct(W,R[I]);double A=FVector::DotProduct(VP,U),B=FVector::DotProduct(VP,T);FVector Next=JT[I]-U*((K11[I]*A-K01[I]*B)/Det[I])-T*((K00[I]*B-K01[I]*A)/Det[I]);double Max=Friction*JN[I],Size=Next.Size();if(Size>Max)Next*=Max/Size;FVector D=Next-JT[I];JT[I]=Next;V+=D*InvMass;W+=InvI.TransformVector(FVector::CrossProduct(R[I],D));}
  const FVector FreeV=V-CapV;V=FreeV.GetClampedToMaxSize(LinearCap);CapV=V-FreeV;
  const FVector FreeW=W-CapW;if(!BoundedContact::ProjectAngular(FreeW,InvI,AngularCap,W))return Out;CapW=W-FreeW;
  const double Change=FMath::Max((V-OldV).Size(),(W-OldW).Size()*FMath::Max(1.,R[0].Size()));
  if(V.ContainsNaN()||W.ContainsNaN())return Out;
  if(Change>Tol)continue;
  double Residual=0;
  for(int I=0;I<Count;++I){const FVector VP=V+FVector::CrossProduct(W,R[I]);double VN=FVector::DotProduct(VP,N),Desired=FMath::Max(Floor[I],Target[I]-CFM[I]*JN[I]);Residual=FMath::Max(Residual,JN[I]>Tol/InvMass?FMath::Abs(VN-Desired):FMath::Max(0.,Desired-VN));Residual=FMath::Max(Residual,FMath::Max(0.,JT[I].Size()-Friction*JN[I])*InvMass);const FVector Slip=VP-N*VN;if(Slip.Size()>Tol && JT[I].Size()>Tol/InvMass){const double Dot=FVector::DotProduct(JT[I],Slip);Residual=FMath::Max(Residual,FMath::Max(0.,Dot/(1.+JT[I].Size())));}}
  Residual=FMath::Max(Residual,FMath::Max(0.,V.Size()-LinearCap));Residual=FMath::Max(Residual,FMath::Max(0.,W.Size()-AngularCap)*FMath::Max(1.,R[0].Size()));
  if(Residual>32*Tol)continue;
  Out.Contact.DeltaVelocity=V-V0;Out.Contact.DeltaAngularVelocity=W-W0;Out.Contact.IterationCount=Iter+1;Out.Contact.bSolved=true;Out.MaximumResidual=Residual;Out.LimitDeltaVelocity=CapV;Out.LimitDeltaAngularVelocity=CapW;
  for(int I=0;I<Count;++I){Out.Contact.Impulses[I]=JN[I];Out.Contact.TangentImpulses[I]=JT[I];}return Out;
 }
 return Out;
}
}
