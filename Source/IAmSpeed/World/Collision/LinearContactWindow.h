#pragma once
#include <algorithm>
#include <cmath>
namespace Speed::Collision
{
// Intersection of fixed-axis linear projection intervals, in seconds.
// Valid only for fixed orientations and zero relative acceleration.
struct FLinearContactWindow
{
    double Enter=0, Exit;
    bool Possible;
    explicit FLinearContactWindow(double Duration):Exit(Duration),Possible(std::isfinite(Duration)&&Duration>=0) {}
    bool Clip(double Separation,double RelativeSpeed,double Radius)
    {
        if(!Possible) return false;
        if(!std::isfinite(Separation)||!std::isfinite(RelativeSpeed)||!std::isfinite(Radius)||Radius<0)
            return Possible=false;
        if(RelativeSpeed==0) return Possible=std::abs(Separation)<=Radius;
        double A=(-Radius-Separation)/RelativeSpeed, B=(Radius-Separation)/RelativeSpeed;
        if(A>B) std::swap(A,B);
        Enter=std::max(Enter,A); Exit=std::min(Exit,B);
        return Possible=Enter<=Exit;
    }
};
}
