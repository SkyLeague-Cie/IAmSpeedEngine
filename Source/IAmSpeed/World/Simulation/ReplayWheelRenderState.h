#pragma once
#include "ReplayFieldArchive.h"
#include <array>
namespace Speed::Replay
{
// Values copied solely from physical-owner SWheelRenderData. Independent of
// live render ticks, Chaos visual output, UObject pointers and lossy packing.
struct FReplayWheelRenderState
{
    std::array<double,4> Roll{0,0,0,1},Steer{0,0,0,1};
    std::array<double,3> Position{},Right{0,1,0},Up{0,0,1},Forward{1,0,0};
    float SpringOffset=0;
};
template<class Archive,class State> inline void WheelRenderFields(Archive& A,State& S)
{
    for(auto& V:S.Roll)A.Scalar(V);for(auto& V:S.Steer)A.Scalar(V);
    for(auto& V:S.Position)A.Scalar(V);for(auto& V:S.Right)A.Scalar(V);
    for(auto& V:S.Up)A.Scalar(V);for(auto& V:S.Forward)A.Scalar(V);A.Scalar(S.SpringOffset);
}
inline void WriteWheelRenderState(FFieldWriter& W,const std::array<FReplayWheelRenderState,4>& Wheels)
{W.Scalar(uint8_t(1));for(const auto& S:Wheels)WheelRenderFields(W,S);}
inline void ReadWheelRenderState(FFieldReader& R,std::array<FReplayWheelRenderState,4>& Wheels)
{uint8_t Version=0;R.Scalar(Version);if(Version!=1)throw FFieldError("unknown wheel render schema");for(auto& S:Wheels)WheelRenderFields(R,S);}
}
