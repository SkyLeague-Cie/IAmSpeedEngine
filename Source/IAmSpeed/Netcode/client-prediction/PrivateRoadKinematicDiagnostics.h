#pragma once
#include "IAmSpeed/Base/Kinematic.h"
#include <cmath>

namespace Speed::Netcode::Prediction
{
struct FPrivateRoadKinematicDiagnostics
{
    bool Valid=false,Different=false;
    double Position=0,Velocity=0,Acceleration=0,RotationRadians=0,AngularVelocity=0,AngularAcceleration=0;
};
// Numerical diagnostics on validated copies only. Exact field equality is an
// independent verdict: a rounded/underflowed diagnostic must NEVER erase a
// real kinematic difference. This does not compare hidden causal world state.
inline FPrivateRoadKinematicDiagnostics MeasurePrivateRoadKinematics(const SKinematic& A,const SKinematic& B) noexcept
{
    FPrivateRoadKinematicDiagnostics Out;
    const auto Finite=[](const SKinematic& K)
    {return !K.Location.ContainsNaN()&&!K.Velocity.ContainsNaN()&&!K.Acceleration.ContainsNaN()
        &&!K.AngularVelocity.ContainsNaN()&&!K.AngularAcceleration.ContainsNaN()&&K.Rotation.IsNormalized();};
    if(!Finite(A)||!Finite(B))return Out;
    const auto& Q=A.Rotation;const auto& R=B.Rotation;
    const bool SameRotation=Q==R||(Q.X==-R.X&&Q.Y==-R.Y&&Q.Z==-R.Z&&Q.W==-R.W);
    Out.Different=!(A.Location==B.Location&&A.Velocity==B.Velocity&&A.Acceleration==B.Acceleration
        &&A.AngularVelocity==B.AngularVelocity&&A.AngularAcceleration==B.AngularAcceleration&&SameRotation);
    const auto Distance=[](const FVector& X,const FVector& Y)
    {return std::hypot(double(X.X)-double(Y.X),double(X.Y)-double(Y.Y),double(X.Z)-double(Y.Z));};
    Out.Position=Distance(A.Location,B.Location);Out.Velocity=Distance(A.Velocity,B.Velocity);
    Out.Acceleration=Distance(A.Acceleration,B.Acceleration);Out.AngularVelocity=Distance(A.AngularVelocity,B.AngularVelocity);
    Out.AngularAcceleration=Distance(A.AngularAcceleration,B.AngularAcceleration);
    if(!SameRotation)
    {
        const auto Norm4=[](double X,double Y,double Z,double W){return std::hypot(std::hypot(X,Y),std::hypot(Z,W));};
        const double QNorm=Norm4(Q.X,Q.Y,Q.Z,Q.W),RNorm=Norm4(R.X,R.Y,R.Z,R.W);
        if(!(QNorm>0&&RNorm>0))return Out;
        const double QX=Q.X/QNorm,QY=Q.Y/QNorm,QZ=Q.Z/QNorm,QW=Q.W/QNorm;
        double RX=R.X/RNorm,RY=R.Y/RNorm,RZ=R.Z/RNorm,RW=R.W/RNorm;
        if(QX*RX+QY*RY+QZ*RZ+QW*RW<0){RX=-RX;RY=-RY;RZ=-RZ;RW=-RW;}
        // For unit quaternions, chord difference/sum yields theta/4. atan2
        // retains tiny nonzero rotations when the dot product rounds to1.
        Out.RotationRadians=4.*std::atan2(Norm4(QX-RX,QY-RY,QZ-RZ,QW-RW),Norm4(QX+RX,QY+RY,QZ+RZ,QW+RW));
    }
    Out.Valid=std::isfinite(Out.Position)&&std::isfinite(Out.Velocity)&&std::isfinite(Out.Acceleration)
        &&std::isfinite(Out.RotationRadians)&&std::isfinite(Out.AngularVelocity)&&std::isfinite(Out.AngularAcceleration);
    return Out;
}
}
