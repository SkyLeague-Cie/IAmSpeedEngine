#pragma once
#include <cstdint>
namespace Speed::Collision
{
template<class Target,class CandidateTarget>
bool IsCarBumpInCooldown(const Target& Last,const CandidateTarget& Candidate,std::uint16_t Frames)
{
    return Frames>0 && Last==Candidate;
}
template<class Target>
void AdvanceCarBumpCooldown(Target& Last,std::uint16_t& Frames)
{
    if(Frames>0) --Frames;
    if(Frames==0) Last=Target{};
}
}
