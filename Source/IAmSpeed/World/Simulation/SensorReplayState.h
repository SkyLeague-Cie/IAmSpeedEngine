#pragma once
#include <array>
#include <algorithm>
#include <cstdint>
#include <vector>
#include "ReplayFieldArchive.h"
namespace Speed::Replay
{
struct FSensorReplayReference
{
    uint64_t Body=0;uint16_t SubBody=0;
    bool operator==(const FSensorReplayReference&)const=default;
    bool operator<(const FSensorReplayReference& B)const
    {return Body<B.Body || (Body==B.Body && SubBody<B.SubBody);}
};
struct FSensorReplayState
{
    uint8_t Shape=0,CollisionChannel=0;
    std::array<double,19> Kinematic{};
    std::array<double,3> LocalPosition{},LocalScale{},Dimensions{};
    std::array<double,4> LocalRotation{};
    std::array<uint8_t,64> Responses{};
    std::vector<FSensorReplayReference> Ordered,Present,Touched;
};
inline void ValidateSensorReferences(const std::vector<FSensorReplayReference>& R,bool Ordered)
{
    if(R.size()>256)throw FFieldError("sensor reference bound");
    for(size_t I=0;I<R.size();++I)
    {
        if(!R[I].Body)throw FFieldError("sensor reference has no stable body");
        if(Ordered){for(size_t J=0;J<I;++J)if(R[J]==R[I])throw FFieldError("duplicate ordered sensor reference");}
        else if(I && !(R[I-1]<R[I]))throw FFieldError("sensor reference set order");
    }
}
inline void ValidateSensorState(const FSensorReplayState& S)
{
    if((S.Shape!=1 && S.Shape!=2) || S.CollisionChannel>=S.Responses.size())throw FFieldError("unsupported sensor shape/channel");
    for(auto R:S.Responses)if(R>2)throw FFieldError("sensor collision response");
    ValidateSensorReferences(S.Ordered,true);ValidateSensorReferences(S.Present,false);ValidateSensorReferences(S.Touched,false);
    auto Sorted=S.Ordered;std::sort(Sorted.begin(),Sorted.end());
    if(Sorted!=S.Present)throw FFieldError("sensor ordered/set disagreement");
}
inline void WriteSensorState(FFieldWriter& W,const FSensorReplayState& S)
{
    ValidateSensorState(S);W.Scalar(uint8_t(2));W.Scalar(S.Shape);W.Scalar(S.CollisionChannel);
    for(auto V:S.Kinematic)W.Scalar(V);
    for(auto V:S.LocalPosition)W.Scalar(V);for(auto V:S.LocalRotation)W.Scalar(V);for(auto V:S.LocalScale)W.Scalar(V);
    for(auto V:S.Dimensions)W.Scalar(V);for(auto V:S.Responses)W.Scalar(V);
    for(const auto* R:{&S.Ordered,&S.Present,&S.Touched})
    {W.Scalar(uint16_t(R->size()));for(const auto& V:*R){W.Scalar(V.Body);W.Scalar(V.SubBody);}}
}
inline void ReadSensorState(FFieldReader& R,FSensorReplayState& S)
{
    uint8_t Version=0;R.Scalar(Version);if(Version!=2)throw FFieldError("unsupported sensor field version");
    R.Scalar(S.Shape);R.Scalar(S.CollisionChannel);
    for(auto& V:S.Kinematic)R.Scalar(V);
    for(auto& V:S.LocalPosition)R.Scalar(V);for(auto& V:S.LocalRotation)R.Scalar(V);for(auto& V:S.LocalScale)R.Scalar(V);
    for(auto& V:S.Dimensions)R.Scalar(V);for(auto& V:S.Responses)R.Scalar(V);
    for(auto* Refs:{&S.Ordered,&S.Present,&S.Touched})
    {uint16_t N=0;R.Scalar(N);if(N>256)throw FFieldError("sensor reference bound");Refs->resize(N);for(auto& V:*Refs){R.Scalar(V.Body);R.Scalar(V.SubBody);}}
    ValidateSensorState(S);
}
}
