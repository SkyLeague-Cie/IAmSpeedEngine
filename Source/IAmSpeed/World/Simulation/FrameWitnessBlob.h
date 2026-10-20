#pragma once
#include "CoreMinimal.h"
#if !UE_BUILD_SHIPPING
#include "Misc/Compression.h"
#include "Misc/SecureHash.h"
namespace Speed::FrameWitnessBlob
{
// Lossless correctness-only transport. Original physical byte values, order and
// pointer sidecars are unchanged. Timing controls keep their recorder disabled.
inline constexpr int32 HeaderBytes=44,MaxRawBytes=16*1024*1024,MaxPackedBytes=17*1024*1024;
inline constexpr uint8 Magic[8]={'I','A','S','Z','L','I','B','1'};
inline void Put64(uint8* Out,uint64 Value){for(int32 I=0;I<8;++I)Out[I]=uint8(Value>>(8*I));}
inline uint64 Get64(const uint8* In){uint64 V=0;for(int32 I=0;I<8;++I)V|=uint64(In[I])<<(8*I);return V;}
inline bool Encode(const TArray<uint8>& Raw,TArray<uint8>& Encoded)
{
 if(Raw.Num()>MaxRawBytes)return false;
 int64 Bound=0;
 if(!Raw.IsEmpty() && !FCompression::CompressMemoryBound(NAME_Zlib,Bound,int64(Raw.Num())))return false;
 if(Bound<0 || Bound>MaxPackedBytes-HeaderBytes)return false;
 TArray<uint8> Work;Work.SetNumUninitialized(HeaderBytes+int32(Bound));
 FMemory::Memcpy(Work.GetData(),Magic,8);Put64(Work.GetData()+8,uint64(Raw.Num()));
 int64 Packed=Bound;
 if(!Raw.IsEmpty() && !FCompression::CompressMemory(NAME_Zlib,Work.GetData()+HeaderBytes,Packed,Raw.GetData(),int64(Raw.Num()),COMPRESS_BiasSpeed))return false;
 if(Packed<0 || Packed>Bound)return false;
 Put64(Work.GetData()+16,uint64(Packed));const uint8 Empty=0;
 FSHA1::HashBuffer(Raw.IsEmpty()?&Empty:Raw.GetData(),uint64(Raw.Num()),Work.GetData()+24);
 Work.SetNum(HeaderBytes+int32(Packed),EAllowShrinking::No);Encoded=MoveTemp(Work);return true;
}
// Used by native interoperability tests. The Python reader also checks strict
// zlib stream exhaustion, artifact SHA256 and exact decompressed byte SHA256.
inline bool Decode(const TArray<uint8>& Encoded,TArray<uint8>& Raw)
{
 if(Encoded.Num()<HeaderBytes || Encoded.Num()>MaxPackedBytes || FMemory::Memcmp(Encoded.GetData(),Magic,8))return false;
 const uint64 Size=Get64(Encoded.GetData()+8),Packed=Get64(Encoded.GetData()+16);
 if(Size>MaxRawBytes || Packed!=uint64(Encoded.Num()-HeaderBytes) || (!Size && Packed))return false;
 TArray<uint8> Work;Work.SetNumUninitialized(int32(Size));
 if(Size && !FCompression::UncompressMemory(NAME_Zlib,Work.GetData(),int64(Size),Encoded.GetData()+HeaderBytes,int64(Packed)))return false;
 uint8 Hash[20];const uint8 Empty=0;FSHA1::HashBuffer(Work.IsEmpty()?&Empty:Work.GetData(),Size,Hash);
 if(FMemory::Memcmp(Hash,Encoded.GetData()+24,20))return false;
 Raw=MoveTemp(Work);return true;
}
}
#endif
