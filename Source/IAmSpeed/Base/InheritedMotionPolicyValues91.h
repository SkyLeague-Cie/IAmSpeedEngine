#pragma once
#include "CoreTypes.h"
#include <cstdint>
#include <cstring>
namespace Speed
{
// Raw policy captured ONLY by Initialize. It is not a verified motion bound,
// current-mode promise, geometry, admission or no-contact certificate.
struct FInheritedMotionPolicyValues91
{
    float ParentMaxV=0,ParentMaxOmega=0;
    bool Captured=false;
};
inline bool SameInheritedMotionPolicy91(const FInheritedMotionPolicyValues91& a,const FInheritedMotionPolicyValues91& b)
{
    std::uint32_t av=0,aw=0,bv=0,bw=0;
    std::memcpy(&av,&a.ParentMaxV,4);std::memcpy(&aw,&a.ParentMaxOmega,4);
    std::memcpy(&bv,&b.ParentMaxV,4);std::memcpy(&bw,&b.ParentMaxOmega,4);
    return a.Captured==b.Captured&&av==bv&&aw==bw;
}
// Private observation only. One definition in the existing IAS profiling TU.
IAMSPEED_API void ObserveInheritedMotionInitialization91(bool captured);
}
