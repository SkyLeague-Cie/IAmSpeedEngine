#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
namespace Speed::Collision
{
constexpr std::size_t CarBumpSnapshotSize=1+sizeof(std::uint16_t)+sizeof(std::uint64_t);
inline std::array<std::uint8_t,CarBumpSnapshotSize> EncodeCarBumpSnapshot(
    std::uint16_t Frames,std::uint64_t VictimId)
{
    std::array<std::uint8_t,CarBumpSnapshotSize> Bytes{};
    Bytes[0]=1;
    std::memcpy(Bytes.data()+1,&Frames,sizeof(Frames));
    std::memcpy(Bytes.data()+1+sizeof(Frames),&VictimId,sizeof(VictimId));
    return Bytes;
}
inline bool DecodeCarBumpSnapshot(const std::uint8_t* Bytes,std::size_t Size,
    std::uint16_t& OutFrames,std::uint64_t& OutVictimId)
{
    if(!Bytes || Size!=CarBumpSnapshotSize || Bytes[0]!=1) return false;
    std::uint16_t Frames=0;std::uint64_t VictimId=0;
    std::memcpy(&Frames,Bytes+1,sizeof(Frames));
    std::memcpy(&VictimId,Bytes+1+sizeof(Frames),sizeof(VictimId));
    if((Frames==0)!=(VictimId==0)) return false;
    OutFrames=Frames;OutVictimId=VictimId;
    return true;
}
}
