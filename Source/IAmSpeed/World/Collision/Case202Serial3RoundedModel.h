#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

namespace Case202Serial3
{
struct Range
{
    double Lo=0., Hi=0.;
    bool Valid() const { return std::isfinite(Lo)&&std::isfinite(Hi)&&Lo<=Hi; }
    bool Exactly(double value) const { return Lo==value&&Hi==value; }
};
using V3=std::array<Range,3>;
using Q4=std::array<Range,4>;

struct Input
{
    std::array<double,3> Position,Velocity,Acceleration,Omega,Alpha,COM,Offset,HalfExtent;
    std::array<double,4> Q;
};

struct BoundedPose
{
    V3 Center;
    Q4 Rotation;
    std::array<V3,8> Corners;
};

// These are monotone endpoint operations of the reviewed native rounded DAG,
// unlike real-arithmetic intervals widened away from zero at every node.
inline double RNAdd(double a,double b) { volatile double out=a+b;return out; }
inline double RNMultiply(double a,double b) { volatile double out=a*b;return out; }
inline double RNDivide(double a,double b) { volatile double out=a/b;return out; }
inline Range Exact(double x) {return {x,x};}
inline Range Add(Range a,Range b) {return {RNAdd(a.Lo,b.Lo),RNAdd(a.Hi,b.Hi)};}
inline Range Negate(Range a) {return {-a.Hi,-a.Lo};}
inline Range Subtract(Range a,Range b) {return Add(a,Negate(b));}
inline Range Multiply(Range a,Range b)
{
    const double values[]={RNMultiply(a.Lo,b.Lo),RNMultiply(a.Lo,b.Hi),
                           RNMultiply(a.Hi,b.Lo),RNMultiply(a.Hi,b.Hi)};
    return {*std::min_element(values,values+4),*std::max_element(values,values+4)};
}
inline Range Square(Range a)
{
    const double x=RNMultiply(a.Lo,a.Lo),y=RNMultiply(a.Hi,a.Hi);
    return {a.Lo<=0.&&0.<=a.Hi?0.:(std::min)(x,y),(std::max)(x,y)};
}
inline Range DividePositive(Range a,double b) {return {RNDivide(a.Lo,b),RNDivide(a.Hi,b)};}

inline V3 Cross(const V3& a,const V3& b)
{
    V3 out;
    for(int i=0;i<3;++i)
    {
        const int j=(i+1)%3,k=(i+2)%3;
        out[i]=Subtract(Multiply(a[j],b[k]),Multiply(a[k],b[j]));
    }
    return out;
}

inline V3 Rotate(const Q4& q,const V3& v)
{
    const V3 axis={q[0],q[1],q[2]};
    auto t=Cross(axis,v);
    for(auto& x:t)x=Multiply(Exact(2.),x);
    const auto u=Cross(axis,t);
    V3 out;
    for(int i=0;i<3;++i)out[i]=Add(Add(v[i],Multiply(q[3],t[i])),u[i]);
    return out;
}

inline Q4 Hamilton(const Q4& a,const Q4& b)
{
    constexpr int swizzle[3][4]={{3,2,1,0},{2,3,0,1},{1,0,3,2}};
    constexpr int signs[3][4]={{1,-1,1,-1},{1,1,-1,-1},{-1,1,1,-1}};
    Q4 out;
    for(int i=0;i<4;++i)out[i]=Multiply(a[3],b[i]);
    for(int k=0;k<3;++k)for(int i=0;i<4;++i)
    {
        auto term=Multiply(a[k],b[swizzle[k][i]]);
        if(signs[k][i]<0)term=Negate(term);
        out[i]=Add(term,out[i]);
    }
    return out;
}

inline float RoundFloatSquare(float x) { volatile float out=x*x;return out; }

inline bool BoundReviewedCell(const Input& input,float first,float last,bool alternateDot,
    double verifiedCosineThreshold,BoundedPose& out)
{
    if(!std::isfinite(first)||!std::isfinite(last)||first<0.f||first>last||
       !std::isfinite(verifiedCosineThreshold)||verifiedCosineThreshold<=0.||
       !std::all_of(input.Alpha.begin(),input.Alpha.end(),[](double a){return a==0.;}))return false;
    const Range seconds{double(first),double(last)};
    const Range time2{double(RoundFloatSquare(first)),double(RoundFloatSquare(last))};
    V3 theta;
    for(int i=0;i<3;++i)theta[i]=Multiply(Exact(input.Omega[i]),seconds);
    const auto norm2=Add(Add(Square(theta[0]),Square(theta[1])),Square(theta[2]));
    if(!norm2.Valid()||norm2.Lo<0.||norm2.Hi>=1.e-8)return false;
    volatile double angle=std::sqrt(norm2.Hi);
    if(!(RNMultiply(angle,.5)<verifiedCosineThreshold))return false;
    const auto scale=Add(Subtract(Exact(.5),DividePositive(norm2,48.)),DividePositive(Square(norm2),3840.));
    if(!scale.Exactly(.5))return false;
    Q4 dq={Multiply(theta[0],scale),Multiply(theta[1],scale),Multiply(theta[2],scale),Exact(1.)};
    Q4 q0;
    for(int i=0;i<4;++i)q0[i]=Exact(input.Q[i]);
    const auto raw=Hamilton(dq,q0);
    const auto x2=Square(raw[0]),y2=Square(raw[1]),z2=Square(raw[2]),w2=Square(raw[3]);
    const auto norm=alternateDot?Add(Add(x2,z2),Add(y2,w2)):Add(Add(x2,y2),Add(z2,w2));
    if(!norm.Exactly(1.))return false;
    // At zero theta the native early branch returns q0. The interval also
    // contains q0; multiplying by the proved exact reciprocal 1 is identical.
    out.Rotation=raw;

    V3 com,offset,position;
    for(int i=0;i<3;++i)
    {
        com[i]=Exact(input.COM[i]);offset[i]=Exact(input.Offset[i]);
        position[i]=Add(Add(Exact(input.Position[i]),Multiply(Exact(input.Velocity[i]),seconds)),
                        Multiply(Multiply(Exact(.5),Exact(input.Acceleration[i])),time2));
    }
    const auto rotatedCOM=Rotate(raw,com),rotatedOffset=Rotate(raw,offset);
    for(int i=0;i<3;++i)out.Center[i]=Add(Subtract(position[i],rotatedCOM[i]),rotatedOffset[i]);
    for(int k=0;k<8;++k)
    {
        V3 local;
        for(int i=0;i<3;++i)local[i]=Exact((k&(1<<i))?input.HalfExtent[i]:-input.HalfExtent[i]);
        const auto rotated=Rotate(raw,local);
        for(int i=0;i<3;++i)
        {
            out.Corners[k][i]=Add(out.Center[i],rotated[i]);
            if(!out.Corners[k][i].Valid())return false;
        }
    }
    return true;
}
}
