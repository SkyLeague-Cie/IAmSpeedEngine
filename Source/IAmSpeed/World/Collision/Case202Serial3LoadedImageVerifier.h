#pragma once
#include "Case202Serial3GraphEvidence.h"
#include <cstring>
#include <memory>
#include <iterator>
#include <vector>

#if defined(_WIN32) && defined(_M_X64)
#pragma push_macro("far")
#pragma push_macro("near")
#if defined(CASE202_REVIEW_STANDALONE)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>
#else
#include "Windows/WindowsHWrapper.h"
#endif
#include <bcrypt.h>
#include <intrin.h>
#pragma pop_macro("near")
#pragma pop_macro("far")
#pragma comment(lib,"bcrypt.lib")

namespace Case202Serial3
{
using Digest=std::array<uint8_t,32>;
constexpr uint32_t MaxIASDataSpans=32,MaxCRTSpans=8;
constexpr uint32_t MaxCodeBytes=16*1024*1024,MaxDataBytes=1024*1024;
constexpr uint32_t FmaExportRVA=0x93ef0,FmaFlagRVA=0x139e60,FmaCodeBytes=15;
constexpr std::array<uint32_t,2> ClassifyRVA{0x688f0,0x5f410};
constexpr std::array<uint32_t,2> ClassifyBytes{0x5c,0x41};

// Fixed little-endian data format; every field preceding Signature is signed.
// Ranges are linked x64 image bytes audited post-link, without ASLR-varying data.
#pragma pack(push,1)
struct AuditedSpan
{
    uint32_t RVA,Bytes;
    Digest DigestOfLoadedBytes;
};
struct SignedImageSeal
{
    char Magic[8];
    uint32_t Version;
    uint32_t IntegrateRVA,RotationRVA,GraphRVA,CosIATRVA,CosRVA;
    uint32_t IntegrateCallRVA,CosCallRVA,CosThunkRVA;
    uint32_t FmaIATRVA,FmaThunkRVA;
    uint32_t ClassifyIATRVA[2],ClassifyThunkRVA[2];
    Digest IAS,CRT,CanonicalDAG;
    uint32_t IASDataCount,CRTCount;
    AuditedSpan IASCode,IASData[MaxIASDataSpans],CRTSpans[MaxCRTSpans];
    uint8_t Signature[256];
};
#pragma pack(pop)
static_assert(sizeof(AuditedSpan)==40&&offsetof(SignedImageSeal,Signature)==1812,
    "wire layout must match the audited packer");

namespace SealDetail
{
struct FileHandle
{
    HANDLE Handle=INVALID_HANDLE_VALUE;
    ~FileHandle(){if(Handle!=INVALID_HANDLE_VALUE)CloseHandle(Handle);}
};
struct Algorithm
{
    BCRYPT_ALG_HANDLE Handle=nullptr;
    ~Algorithm(){if(Handle)BCryptCloseAlgorithmProvider(Handle,0);}
};
struct HashHandle
{
    BCRYPT_HASH_HANDLE Handle=nullptr;
    ~HashHandle(){if(Handle)BCryptDestroyHash(Handle);}
};
struct PublicKey
{
    BCRYPT_KEY_HANDLE Handle=nullptr;
    ~PublicKey(){if(Handle)BCryptDestroyKey(Handle);}
};
inline bool Ok(NTSTATUS result){return result>=0;}

class SHA256
{
    Algorithm Provider;
    // The CNG state storage must outlive its owning hash.
    std::vector<uint8_t> Object;
    HashHandle State;
public:
    bool Init()
    {
        if(!Ok(BCryptOpenAlgorithmProvider(&Provider.Handle,BCRYPT_SHA256_ALGORITHM,nullptr,0)))return false;
        ULONG bytes=0,returned=0;
        if(!Ok(BCryptGetProperty(Provider.Handle,BCRYPT_OBJECT_LENGTH,reinterpret_cast<PUCHAR>(&bytes),
            sizeof(bytes),&returned,0))||returned!=sizeof(bytes)||!bytes||bytes>65536)return false;
        Object.resize(bytes);
        return Ok(BCryptCreateHash(Provider.Handle,&State.Handle,Object.data(),bytes,nullptr,0,0));
    }
    bool Add(const void* bytes,ULONG count)
    {return Ok(BCryptHashData(State.Handle,const_cast<PUCHAR>(static_cast<const uint8_t*>(bytes)),count,0));}
    bool Finish(Digest& out)
    {return Ok(BCryptFinishHash(State.Handle,out.data(),ULONG(out.size()),0));}
};

inline bool HashMemory(const void* bytes,ULONG count,Digest& out)
{
    SHA256 hash;
    return hash.Init()&&hash.Add(bytes,count)&&hash.Finish(out);
}

inline bool HashModuleFile(HMODULE module,Digest& out)
{
    wchar_t path[32768];
    const DWORD n=GetModuleFileNameW(module,path,DWORD(std::size(path)));
    if(!n||n>=std::size(path))return false;
    FileHandle file{CreateFileW(path,GENERIC_READ,FILE_SHARE_READ,nullptr,OPEN_EXISTING,FILE_ATTRIBUTE_NORMAL,nullptr)};
    LARGE_INTEGER size;
    if(file.Handle==INVALID_HANDLE_VALUE||!GetFileSizeEx(file.Handle,&size)||size.QuadPart<=0||
        size.QuadPart>256ll*1024*1024)return false;
    SHA256 hash;
    if(!hash.Init())return false;
    std::vector<uint8_t> buffer(65536);
    for(LONGLONG left=size.QuadPart;left;)
    {
        const DWORD want=DWORD((std::min)(left,LONGLONG(buffer.size())));
        DWORD read=0;
        if(!ReadFile(file.Handle,buffer.data(),want,&read,nullptr)||read!=want||!hash.Add(buffer.data(),read))return false;
        left-=read;
    }
    return hash.Finish(out);
}

inline bool ReadSeal(const wchar_t* path,SignedImageSeal& seal)
{
    if(!path||!*path)return false;
    FileHandle file{CreateFileW(path,GENERIC_READ,FILE_SHARE_READ,nullptr,OPEN_EXISTING,FILE_ATTRIBUTE_NORMAL,nullptr)};
    LARGE_INTEGER size;
    DWORD count=0;
    return file.Handle!=INVALID_HANDLE_VALUE&&GetFileSizeEx(file.Handle,&size)&&size.QuadPart==sizeof(seal)&&
        ReadFile(file.Handle,&seal,DWORD(sizeof(seal)),&count,nullptr)&&count==sizeof(seal);
}

inline bool VerifySignature(const SignedImageSeal& seal,const uint8_t* blob,ULONG length)
{
    Algorithm rsa;
    PublicKey key;
    Digest digest;
    if(!blob||length<sizeof(BCRYPT_RSAKEY_BLOB)||
        !HashMemory(&seal,ULONG(offsetof(SignedImageSeal,Signature)),digest)||
        !Ok(BCryptOpenAlgorithmProvider(&rsa.Handle,BCRYPT_RSA_ALGORITHM,nullptr,0))||
        !Ok(BCryptImportKeyPair(rsa.Handle,nullptr,BCRYPT_RSAPUBLIC_BLOB,&key.Handle,
            const_cast<PUCHAR>(blob),length,0)))return false;
    BCRYPT_PKCS1_PADDING_INFO padding{BCRYPT_SHA256_ALGORITHM};
    return Ok(BCryptVerifySignature(key.Handle,&padding,digest.data(),ULONG(digest.size()),
        const_cast<PUCHAR>(seal.Signature),ULONG(sizeof(seal.Signature)),BCRYPT_PAD_PKCS1));
}

inline HMODULE ModuleOf(const void* p)
{
    HMODULE module=nullptr;
    return p&&GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS|GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
        static_cast<LPCWSTR>(p),&module)?module:nullptr;
}

inline bool ReadableIn(HMODULE module,const void* pointer,size_t bytes,bool immutable=false)
{
    auto address=reinterpret_cast<uintptr_t>(pointer);
    if(!module||!address||!bytes||address+bytes<address)return false;
    const auto end=address+bytes;
    while(address<end)
    {
        MEMORY_BASIC_INFORMATION info{};
        if(VirtualQuery(reinterpret_cast<const void*>(address),&info,sizeof(info))!=sizeof(info)||
            info.AllocationBase!=module||info.State!=MEM_COMMIT||info.Type!=MEM_IMAGE||
            (info.Protect&(PAGE_GUARD|PAGE_NOACCESS))!=0||
            (info.Protect&0xffu)==PAGE_EXECUTE)return false;
        const DWORD access=info.Protect&0xffu;
        if(immutable&&access!=PAGE_READONLY&&access!=PAGE_EXECUTE_READ)return false;
        const auto next=reinterpret_cast<uintptr_t>(info.BaseAddress)+info.RegionSize;
        if(next<=address)return false;
        address=next;
    }
    return true;
}

inline const uint8_t* RVA(HMODULE module,uint32_t rva,size_t size)
{
    const auto base=reinterpret_cast<uintptr_t>(module);
    if(!rva||base+rva<base)return nullptr;
    const auto* point=reinterpret_cast<const uint8_t*>(base+rva);
    return ReadableIn(module,point,size)?point:nullptr;
}

inline bool RelativeTarget(HMODULE module,uint32_t from,uint32_t to,bool indirectJump)
{
    const uint32_t length=indirectJump?6:5;
    const auto* code=RVA(module,from,length);
    if(!code||(indirectJump?(code[0]!=0xff||code[1]!=0x25):code[0]!=0xe8))return false;
    int32_t displacement;
    std::memcpy(&displacement,code+length-4,sizeof(displacement));
    return int64_t(from)+length+displacement==int64_t(to);
}

inline bool CheckSpan(HMODULE module,const AuditedSpan& span,uint32_t maximum)
{
    Digest got;
    const auto* ptr=RVA(module,span.RVA,span.Bytes);
    return ptr&&span.Bytes<=maximum&&ReadableIn(module,ptr,span.Bytes,true)&&
        HashMemory(ptr,span.Bytes,got)&&got==span.DigestOfLoadedBytes;
}

struct MappedSection
{
    uint32_t Start=0,Bytes=0;
};

inline bool FindMappedSection(HMODULE module,const char (&name)[9],uint32_t exactAccess,MappedSection& out)
{
    IMAGE_DOS_HEADER dos{};
    if(!ReadableIn(module,module,sizeof(dos),true))return false;
    std::memcpy(&dos,module,sizeof(dos));
    if(dos.e_magic!=IMAGE_DOS_SIGNATURE||dos.e_lfanew<=0||dos.e_lfanew>0x10000)return false;
    IMAGE_NT_HEADERS64 nt{};
    const auto* ntBytes=RVA(module,uint32_t(dos.e_lfanew),sizeof(nt));
    if(!ntBytes||!ReadableIn(module,ntBytes,sizeof(nt),true))return false;
    std::memcpy(&nt,ntBytes,sizeof(nt));
    if(nt.Signature!=IMAGE_NT_SIGNATURE||nt.FileHeader.Machine!=IMAGE_FILE_MACHINE_AMD64||
        nt.OptionalHeader.Magic!=IMAGE_NT_OPTIONAL_HDR64_MAGIC||
        nt.FileHeader.SizeOfOptionalHeader!=sizeof(nt.OptionalHeader)||
        !nt.FileHeader.NumberOfSections||nt.FileHeader.NumberOfSections>96)return false;
    const uint32_t sections=uint32_t(dos.e_lfanew)+sizeof(nt);
    for(uint32_t i=0;i<nt.FileHeader.NumberOfSections;++i)
    {
        IMAGE_SECTION_HEADER section{};
        const auto* bytes=RVA(module,sections+i*sizeof(section),sizeof(section));
        if(!bytes||!ReadableIn(module,bytes,sizeof(section),true))return false;
        std::memcpy(&section,bytes,sizeof(section));
        if(std::memcmp(section.Name,name,8)!=0)continue;
        const uint32_t access=section.Characteristics&(IMAGE_SCN_MEM_READ|IMAGE_SCN_MEM_WRITE|IMAGE_SCN_MEM_EXECUTE);
        if(out.Bytes||access!=exactAccess||!section.VirtualAddress||!section.Misc.VirtualSize||
            section.Misc.VirtualSize>section.SizeOfRawData||
            uint64_t(section.VirtualAddress)+section.Misc.VirtualSize>nt.OptionalHeader.SizeOfImage)return false;
        out={section.VirtualAddress,section.Misc.VirtualSize};
    }
    return out.Bytes!=0;
}

inline bool CheckIASSpans(HMODULE module,const SignedImageSeal& seal)
{
    MappedSection text,rdata;
    if(!seal.IASDataCount||seal.IASDataCount>MaxIASDataSpans||
        !FindMappedSection(module,".text\0\0\0",IMAGE_SCN_MEM_READ|IMAGE_SCN_MEM_EXECUTE,text)||
        !FindMappedSection(module,".rdata\0\0",IMAGE_SCN_MEM_READ,rdata)||
        seal.IASCode.RVA!=text.Start||seal.IASCode.Bytes!=text.Bytes||
        !CheckSpan(module,seal.IASCode,MaxCodeBytes))return false;
    uint64_t previousEnd=rdata.Start;
    for(uint32_t i=0;i<seal.IASDataCount;++i)
    {
        const auto& span=seal.IASData[i];
        const uint64_t end=uint64_t(span.RVA)+span.Bytes;
        if(span.RVA<previousEnd||end>uint64_t(rdata.Start)+rdata.Bytes||
            !CheckSpan(module,span,MaxDataBytes))return false;
        previousEnd=end;
    }
    return true;
}

inline bool CheckCRTSpans(HMODULE module,const AuditedSpan* spans,uint32_t count)
{
    if(!count||count>MaxCRTSpans)return false;
    for(uint32_t i=0;i<count;++i)if(!CheckSpan(module,spans[i],MaxDataBytes))return false;
    return true;
}

inline bool SpanContains(const AuditedSpan* spans,uint32_t count,uint32_t rva,uint32_t length=1)
{
    if(!rva||!length||count>MaxCRTSpans)return false;
    for(uint32_t i=0;i<count;++i)
        if(rva>=spans[i].RVA&&uint64_t(rva)+length<=uint64_t(spans[i].RVA)+spans[i].Bytes)return true;
    return false;
}

inline bool ReadImport(HMODULE module,const void* slot,const void*& target)
{
    if(!ReadableIn(module,slot,sizeof(target)))return false;
    std::memcpy(&target,slot,sizeof(target));
    return target!=nullptr;
}

inline bool HardwareFmaIsAvailable()
{
    int registers[4]{};
    __cpuid(registers,1);
    constexpr uint32_t flags=(1u<<12)|(1u<<27)|(1u<<28); // FMA, OSXSAVE, AVX
    return (uint32_t(registers[2])&flags)==flags&&(_xgetbv(0)&6u)==6u;
}

inline bool StillLoaded(const NativeModelIdentity& model)
{
    const auto ias=ModuleOf(model.Integrate),crt=ModuleOf(model.ResolvedCos);
    const void *cos=nullptr,*fma=nullptr;
    if(!ias||!crt||ias!=model.IASModule||crt!=model.CRTModule||
        !ReadImport(ias,model.IATSlot,cos)||cos!=model.ResolvedCos||
        !ReadImport(ias,model.FmaIATSlot,fma)||fma!=model.ResolvedFma||
        RVA(crt,FmaExportRVA,FmaCodeBytes)!=fma||
        RVA(crt,FmaFlagRVA,sizeof(uint32_t))!=model.FmaHardwareFlag||
        !HardwareFmaIsAvailable())return false;
    for(size_t i=0;i<ClassifyRVA.size();++i)
    {
        const void* classify=nullptr;
        if(!ReadImport(ias,model.ClassifyIATSlots[i],classify)||
            RVA(crt,ClassifyRVA[i],ClassifyBytes[i])!=classify)return false;
    }
    const auto* flag=static_cast<const volatile uint32_t*>(model.FmaHardwareFlag);
    return *flag!=0;
}
}

class ReviewedLoadedImageVerifier final
{
public:
    static std::unique_ptr<const LoadedModelAttestation> TrySeal(const wchar_t* path,
        const uint8_t* reviewerPublicKey,ULONG keyBytes,const void* integrate,const void* rotation,const void* graph,
        uint64_t numeric,NativeModelIdentity& live)
    {
        using namespace SealDetail;
        live={};
        SignedImageSeal seal{};
        if(numeric!=0x0000027f00001f80ull||!ReadSeal(path,seal)||std::memcmp(seal.Magic,"SL202PE2",8)||
            seal.Version!=2||!VerifySignature(seal,reviewerPublicKey,keyBytes))return {};
        // The UCRT proof is limited to the one independently reviewed image.
        constexpr Digest reviewedCRT={0xde,0xdb,0x76,0x11,0xe1,0xab,0xc1,0xff,0x38,0x52,0xdc,0xac,0x3d,0x4c,0x93,0xa0,
                                      0x48,0xd7,0x29,0xe2,0x5b,0x06,0x19,0x73,0xb9,0x96,0x9f,0x22,0x69,0x8f,0x14,0xca};
        if(seal.CRT!=reviewedCRT||seal.CosRVA!=0xa7730||
            !SpanContains(&seal.IASCode,1,seal.IntegrateRVA)||
            !SpanContains(&seal.IASCode,1,seal.RotationRVA)||
            !SpanContains(&seal.IASCode,1,seal.GraphRVA)||
            !SpanContains(&seal.IASCode,1,seal.IntegrateCallRVA,5)||
            !SpanContains(&seal.IASCode,1,seal.CosCallRVA,5)||
            !SpanContains(&seal.IASCode,1,seal.CosThunkRVA,6)||
            !SpanContains(&seal.IASCode,1,seal.FmaThunkRVA,6)||
            !SpanContains(seal.CRTSpans,seal.CRTCount,0xa7730,0x3d7)||
            !SpanContains(seal.CRTSpans,seal.CRTCount,0x106f40,0x98)||
            !SpanContains(seal.CRTSpans,seal.CRTCount,FmaExportRVA,FmaCodeBytes))return {};
        const auto ias=ModuleOf(integrate);
        const auto* slot=RVA(ias,seal.CosIATRVA,sizeof(void*));
        const auto* fmaSlot=RVA(ias,seal.FmaIATRVA,sizeof(void*));
        if(!ias||ModuleOf(rotation)!=ias||ModuleOf(graph)!=ias||!slot||!fmaSlot||slot==fmaSlot||
            RVA(ias,seal.IntegrateRVA,1)!=integrate||RVA(ias,seal.RotationRVA,1)!=rotation||
            RVA(ias,seal.GraphRVA,1)!=graph||
            !RelativeTarget(ias,seal.IntegrateCallRVA,seal.RotationRVA,false)||
            !RelativeTarget(ias,seal.CosCallRVA,seal.CosThunkRVA,false)||
            !RelativeTarget(ias,seal.CosThunkRVA,seal.CosIATRVA,true)||
            !RelativeTarget(ias,seal.FmaThunkRVA,seal.FmaIATRVA,true))return {};
        const void *cos=nullptr,*fma=nullptr;
        if(!ReadImport(ias,slot,cos)||!ReadImport(ias,fmaSlot,fma))return {};
        const auto crt=ModuleOf(cos);
        if(!crt||RVA(crt,seal.CosRVA,1)!=cos||RVA(crt,FmaExportRVA,FmaCodeBytes)!=fma)return {};
        std::array<const void*,2> classifySlots{};
        for(size_t i=0;i<ClassifyRVA.size();++i)
        {
            classifySlots[i]=RVA(ias,seal.ClassifyIATRVA[i],sizeof(void*));
            if(!classifySlots[i]||!SpanContains(&seal.IASCode,1,seal.ClassifyThunkRVA[i],6)||
                !RelativeTarget(ias,seal.ClassifyThunkRVA[i],seal.ClassifyIATRVA[i],true)||
                !SpanContains(seal.CRTSpans,seal.CRTCount,ClassifyRVA[i],ClassifyBytes[i]))return {};
        }
        Digest iasHash,crtHash;
        if(!HashModuleFile(ias,iasHash)||!HashModuleFile(crt,crtHash)||iasHash!=seal.IAS||crtHash!=seal.CRT||
            !CheckIASSpans(ias,seal)||!CheckCRTSpans(crt,seal.CRTSpans,seal.CRTCount))return {};
        live.Image=iasHash;live.CRT=crtHash;live.RoundedDAG=seal.CanonicalDAG;live.NumericControl=numeric;
        live.Integrate=integrate;live.IATSlot=slot;live.ResolvedCos=cos;
        live.IASModule=ias;live.CRTModule=crt;live.CosineReturnOneBelow=0x1p-27;
        live.FmaIATSlot=fmaSlot;live.ResolvedFma=fma;live.FmaHardwareFlag=RVA(crt,FmaFlagRVA,sizeof(uint32_t));
        live.ClassifyIATSlots=classifySlots;
        if(!live.FmaHardwareFlag){live={};return {};}
        if(!live.Matches(live)||!StillLoaded(live)){live={};return {};}
        return std::unique_ptr<const LoadedModelAttestation>(new LoadedModelAttestation(live,&StillLoaded));
    }
};
}
#endif
