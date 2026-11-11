#pragma once
#include "NativePrepareGTInspection.h"

namespace Speed::PrepareFieldPrediction
{
// Pure named-value operations. Neither reading a field nor predicting it grants
// a native invocation. Caller still needs the private Source/step capability.
inline bool Get(const FNativePrepareGTFields& f,const FString& key,uint64& value)
{
    if(f.Names.Num()!=f.Words.Num())return false;
    const int32 i=f.Names.IndexOfByKey(key);if(i==INDEX_NONE)return false;
    value=f.Words[i];return true;
}
inline bool Set(FNativePrepareGTFields& f,const FString& key,uint64 value)
{
    if(f.Names.Num()!=f.Words.Num())return false;
    const int32 i=f.Names.IndexOfByKey(key);if(i==INDEX_NONE)return false;
    f.Words[i]=value;return true;
}
inline bool Double(const FNativePrepareGTFields& f,const FString& key,double& value)
{
    uint64 bits=0;if(!Get(f,key,bits))return false;
    FMemory::Memcpy(&value,&bits,sizeof(value));return FMath::IsFinite(value);
}
inline bool Vector(const FNativePrepareGTFields& f,const FString& key,FVector& value)
{
    return Double(f,key+TEXT(".x"),value.X)&&Double(f,key+TEXT(".y"),value.Y)&&Double(f,key+TEXT(".z"),value.Z);
}
inline bool Clone(const FNativePrepareGTFields& before,FNativePrepareGTFields& out,
    Analytic::Certified::FDistanceBudget& budget)
{
    if(&before==&out)return false;
    out={};
    if(before.Names.Num()!=before.Words.Num()||before.Names.Num()>4096||
        before.WeakIdentityNames.Num()!=before.WeakIdentities.Num()||before.WeakIdentities.Num()>4096||
        before.SharedIdentityNames.Num()!=before.SharedIdentities.Num()||before.SharedIdentities.Num()>4096||
        before.Sections.Num()>8||before.RetainedValues.Num()>8||before.RetainedCameraEndpoints.Num()>8)return false;
    int64 units=int64(before.Names.Num())+before.WeakIdentities.Num()+before.SharedIdentities.Num()+
        before.RetainedValues.Num()+before.RetainedCameraEndpoints.Num()+1;
    for(const auto& name:before.Names)if(name.IsEmpty()||name.Len()>128)return false;
    for(int32 i=0;i<before.WeakIdentityNames.Num();++i)
    {
        const auto& name=before.WeakIdentityNames[i];
        if(name.IsEmpty()||name.Len()>128||before.WeakIdentityNames.Find(name)!=i)return false;
    }
    for(int32 i=0;i<before.SharedIdentityNames.Num();++i)
    {
        const auto& name=before.SharedIdentityNames[i];
        if(name.IsEmpty()||name.Len()>128||before.SharedIdentityNames.Find(name)!=i)return false;
    }
    for(const auto& section:before.Sections)
    {
        if(section.Name.IsEmpty()||section.Name.Len()>128||section.Bytes.Num()>65536)return false;
        units+=section.Bytes.Num()+1;
    }
    if(budget.ConsumptionFloor<0||units>int64(budget.Remaining)-budget.ConsumptionFloor||
        budget.Used>MAX_uint64-uint64(units))return false;
    for(int64 i=0;i<units;++i)if(!budget.Consume())return false;
    out=before;return true;
}
inline bool ReplaceNamedHit(FNativePrepareGTFields& out,const FString& prefix,const SHitResult& hit,
    Analytic::Certified::FDistanceBudget& budget)
{
    FNativePrepareGTFields values;if(!values.Hit(prefix,hit,budget))return false;
    for(int32 i=0;i<values.Names.Num();++i)if(!Set(out,values.Names[i],values.Words[i]))return false;
    // Hit identities are values too. Retain the exact alias, never just a
    // presence bit or a raw pointer vulnerable to reuse.
    for(int32 i=0;i<values.WeakIdentityNames.Num();++i)
    {
        const int32 j=out.WeakIdentityNames.IndexOfByKey(values.WeakIdentityNames[i]);
        if(j==INDEX_NONE||!out.WeakIdentities.IsValidIndex(j))return false;
        out.WeakIdentities[j]=values.WeakIdentities[i];
    }
    for(int32 i=0;i<values.SharedIdentityNames.Num();++i)
    {
        const int32 j=out.SharedIdentityNames.IndexOfByKey(values.SharedIdentityNames[i]);
        if(j==INDEX_NONE||!out.SharedIdentities.IsValidIndex(j))return false;
        out.SharedIdentities[j]=values.SharedIdentities[i];
    }
    return true;
}
}
