#pragma once
#include "NetInputTransport.h"
#include <algorithm>
namespace Speed::Netcode
{
// FReliableOutbox::Retry walks reusable storage slots, not sequence order.
// Only a caller-owned COPY is sorted before selecting a bounded retry prefix.
// False keeps the exact previous unbounded iteration order. No record/ACK is
// changed and this function grants no source or physical-consumption evidence.
inline void OrderOriginalRetryCopy(std::vector<FPacket>& Records,bool Bounded)
{
    if(!Bounded)return;
    std::sort(Records.begin(),Records.end(),[](const FPacket& A,const FPacket& B)
    {return A.Sequence<B.Sequence||(A.Sequence==B.Sequence&&A.Frame<B.Frame);});
}
}
