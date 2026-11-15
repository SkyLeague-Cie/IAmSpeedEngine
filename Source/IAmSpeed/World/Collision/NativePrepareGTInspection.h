#pragma once
#include "CoreMinimal.h"
#include <memory>
#include <type_traits>
#include "Components/ActorComponent.h"
#include "NativeInitialHitCache.h"
#include "PreparedFirstRsResponseData.h"
#include "NativePreparationObservationAccess.h"
class FSpeedCarCameraEndpoint;
#include "PreparedNativeOwnerBindings.h"
#include "IAmSpeed/World/Analytic/TensorDistanceBounds.h"
#include "IAmSpeed/Base/Kinematic.h"
namespace Speed
{
namespace Input::V2 {struct FOwnerInputSnapshot;}
struct FNativePrepareGTByteSection
{
    FString Name;TArray<uint8> Bytes;
    bool operator==(const FNativePrepareGTByteSection& x) const {return Name==x.Name&&Bytes==x.Bytes;}
};
// Named, padding-free diagnostic values. No weak reference is resolved and no
// preparation/input/force/advance capability follows. Counts are technical IO
// bounds only. Caller must retain the authentic parked GT lease and all pins.
struct FNativePrepareGTFields
{
    std::shared_ptr<const Input::V2::FOwnerInputSnapshot> RetainedInput; // immutable non-UObject identity; prevents ABA between reads
    // Immutable shared values keep exact input/publication/endpoint instances
    // alive across native reads. GT source state retains their initial handles.
    TArray<std::shared_ptr<const void>> RetainedValues;
    TArray<TSharedPtr<const FSpeedCarCameraEndpoint,ESPMode::ThreadSafe>> RetainedCameraEndpoints;
    // Retain actual weak identities and thread-safe liveness owners without
    // resolving UObjects. Their values are compared separately from IO words.
    TArray<FString> WeakIdentityNames,SharedIdentityNames;
    TArray<TWeakObjectPtr<const UObject>> WeakIdentities;
    TArray<TSharedPtr<const void,ESPMode::ThreadSafe>> SharedIdentities;
    bool WeakIdentity(const FString& name,TWeakObjectPtr<const UObject> value,Analytic::Certified::FDistanceBudget& budget)
    {
        if(WeakIdentities.Num()>=4096||WeakIdentityNames.Contains(name)||
            !Add(name+TEXT(".explicit_null"),value.IsExplicitlyNull(),budget))return false;
        WeakIdentityNames.Add(name);WeakIdentities.Add(value);return true;
    }
    bool SharedIdentity(const FString& name,TSharedPtr<const void,ESPMode::ThreadSafe> value,Analytic::Certified::FDistanceBudget& budget)
    {
        if(SharedIdentities.Num()>=4096||SharedIdentityNames.Contains(name)||
            !Add(name+TEXT(".pointer"),reinterpret_cast<UPTRINT>(value.Get()),budget))return false;
        SharedIdentityNames.Add(name);SharedIdentities.Add(MoveTemp(value));return true;
    }
    TArray<FString> Names;TArray<uint64> Words;TArray<FNativePrepareGTByteSection> Sections;
    bool Add(const FString& name,uint64 value,Analytic::Certified::FDistanceBudget& budget)
    {
        if(name.IsEmpty()||name.Len()>128||Names.Num()>=4096||Names.Contains(name)||!budget.Consume())return false;
        Names.Add(name);Words.Add(value);return true;
    }
    template<class T> bool Scalar(const FString& name,T value,Analytic::Certified::FDistanceBudget& budget)
    {
        if constexpr(std::is_same_v<T,float>)return Float(name,value,budget);
        else if constexpr(std::is_same_v<T,double>)return Double(name,value,budget);
        else if constexpr(std::is_enum_v<T>)return Scalar(name,static_cast<std::underlying_type_t<T>>(value),budget);
        else {static_assert(std::is_integral_v<T>);return Add(name,uint64(value),budget);}
    }
    bool Quat(const FString& name,const FQuat& q,Analytic::Certified::FDistanceBudget& budget)
    {return Double(name+TEXT(".x"),q.X,budget)&&Double(name+TEXT(".y"),q.Y,budget)&&Double(name+TEXT(".z"),q.Z,budget)&&Double(name+TEXT(".w"),q.W,budget);}
    bool Double(const FString& name,double value,Analytic::Certified::FDistanceBudget& budget)
    {uint64 bits=0;FMemory::Memcpy(&bits,&value,8);return Add(name,bits,budget);}
    bool Float(const FString& name,float value,Analytic::Certified::FDistanceBudget& budget)
    {uint32 bits=0;FMemory::Memcpy(&bits,&value,4);return Add(name,bits,budget);}
    bool Hit(const FString& name,const SHitResult& h,Analytic::Certified::FDistanceBudget& budget)
    {
        bool ok=true;const auto add=[&](const TCHAR* k,uint64 v){ok=Add(name+k,v,budget)&&ok;};
        add(TEXT(".empty"),IsEmptyNativeInitialHit(h));add(TEXT(".hit"),h.bHit);add(TEXT(".blocking"),h.bBlockingHit);add(TEXT(".penetrating"),h.bStartPenetrating);add(TEXT(".varying_normal"),h.bSurfaceNormalMayVary);
        add(TEXT(".component_explicit_null"),h.Component.IsExplicitlyNull());add(TEXT(".body_explicit_null"),h.SubBody.IsExplicitlyNull());add(TEXT(".prepared_evidence_present"),bool(h.PreparedGroundEvidence));add(TEXT(".support_identity_resolved"),false);
        ok=Vector(name+TEXT(".location"),h.Location,budget)&&ok;ok=Vector(name+TEXT(".point"),h.ImpactPoint,budget)&&ok;ok=Vector(name+TEXT(".normal"),h.ImpactNormal,budget)&&ok;
        ok=Vector(name+TEXT(".arm_this"),h.ContactPointThis,budget)&&ok;ok=Vector(name+TEXT(".arm_other"),h.ContactPointOther,budget)&&ok;
        ok=Float(name+TEXT(".toi_bits"),h.TOI,budget)&&ok;ok=Float(name+TEXT(".depth_bits"),h.PenetrationDepth,budget)&&ok;ok=Float(name+TEXT(".error_bits"),h.GeometricErrorBoundCm,budget)&&ok;
        add(TEXT(".face"),uint64(int64(h.FaceIndex)));add(TEXT(".feature_this"),uint64(h.ContactFeatureThis));add(TEXT(".feature_other"),uint64(h.ContactFeatureOther));add(TEXT(".feature_index_this"),uint64(int64(h.ContactFeatureIndexThis)));add(TEXT(".feature_index_other"),uint64(int64(h.ContactFeatureIndexOther)));
        add(TEXT(".source_id"),h.SourceId);add(TEXT(".surface_id"),h.SurfaceId);add(TEXT(".feature_id"),h.FeatureId);add(TEXT(".primitive_id"),h.PrimitiveId);add(TEXT(".group_id"),h.CanonicalGroupId);add(TEXT(".material_id"),h.MaterialId);add(TEXT(".frame_tag"),h.FrameTag);return PreparedExecution::AppendNativeHitIdentityFields(*this,name,h,budget)&&ok;
    }
    bool Vector(const FString& name,const FVector& v,Analytic::Certified::FDistanceBudget& budget)
    {return Double(name+TEXT(".x"),v.X,budget)&&Double(name+TEXT(".y"),v.Y,budget)&&Double(name+TEXT(".z"),v.Z,budget);}
    bool Kinematic(const FString& name,const FKinematicState& v,Analytic::Certified::FDistanceBudget& budget)
    {
        return Vector(name+TEXT(".P"),v.Location,budget)&&Vector(name+TEXT(".V"),v.Velocity,budget)&&Vector(name+TEXT(".A"),v.Acceleration,budget)&&
            Double(name+TEXT(".Q.x"),v.Rotation.X,budget)&&Double(name+TEXT(".Q.y"),v.Rotation.Y,budget)&&Double(name+TEXT(".Q.z"),v.Rotation.Z,budget)&&Double(name+TEXT(".Q.w"),v.Rotation.W,budget)&&
            Vector(name+TEXT(".W"),v.AngularVelocity,budget)&&Vector(name+TEXT(".Alpha"),v.AngularAcceleration,budget);
    }
    bool Bytes(const FString& name,const uint8* data,int32 count,Analytic::Certified::FDistanceBudget& budget)
    {
        if(name.IsEmpty()||name.Len()>128||count<0||count>65536||Sections.Num()>=8||(count&&!data))return false;
        for(const auto& s:Sections)if(s.Name==name)return false;
        // The actual shared budget exposes unit consumption only; charge each
        // byte before allocation/copy, retaining its overflow/exhaustion guards.
        for(int32 i=0;i<=count;++i)if(!budget.Consume())return false;
        FNativePrepareGTByteSection s;s.Name=name;if(count)s.Bytes.Append(data,count);Sections.Add(MoveTemp(s));return true;
    }
    bool Append(const FNativePrepareGTFields& x,Analytic::Certified::FDistanceBudget& budget)
    {
        if(x.Names.Num()!=x.Words.Num()||x.WeakIdentityNames.Num()!=x.WeakIdentities.Num()||
            x.SharedIdentityNames.Num()!=x.SharedIdentities.Num()||WeakIdentities.Num()+x.WeakIdentities.Num()>4096||
            SharedIdentities.Num()+x.SharedIdentities.Num()>4096)return false;
        for(const auto& name:x.WeakIdentityNames)if(WeakIdentityNames.Contains(name))return false;
        for(const auto& name:x.SharedIdentityNames)if(SharedIdentityNames.Contains(name))return false;
        WeakIdentityNames.Append(x.WeakIdentityNames);WeakIdentities.Append(x.WeakIdentities);
        SharedIdentityNames.Append(x.SharedIdentityNames);SharedIdentities.Append(x.SharedIdentities);
        if(x.RetainedInput){if(RetainedInput&&(RetainedInput!=x.RetainedInput||RetainedInput.owner_before(x.RetainedInput)||x.RetainedInput.owner_before(RetainedInput)))return false;RetainedInput=x.RetainedInput;}
        if(RetainedValues.Num()+x.RetainedValues.Num()>8||RetainedCameraEndpoints.Num()+x.RetainedCameraEndpoints.Num()>8)return false;
        RetainedValues.Append(x.RetainedValues);RetainedCameraEndpoints.Append(x.RetainedCameraEndpoints);
        for(int32 i=0;i<x.Names.Num();++i)if(!Add(x.Names[i],x.Words[i],budget))return false;
        for(const auto& s:x.Sections)if(!Bytes(s.Name,s.Bytes.GetData(),s.Bytes.Num(),budget))return false;
        return true;
    }
    bool AppendPrefixed(const FString& prefix,const FNativePrepareGTFields& x,Analytic::Certified::FDistanceBudget& budget)
    {
        FNativePrepareGTFields named=x;
        for(auto& name:named.Names)name=prefix+name;
        for(auto& section:named.Sections)section.Name=prefix+section.Name;
        for(auto& name:named.WeakIdentityNames)name=prefix+name;
        for(auto& name:named.SharedIdentityNames)name=prefix+name;
        return Append(named,budget);
    }
    bool Same(const FNativePrepareGTFields& x) const
    {
        if(Names.Num()!=Words.Num()||x.Names.Num()!=x.Words.Num()||
            WeakIdentityNames.Num()!=WeakIdentities.Num()||x.WeakIdentityNames.Num()!=x.WeakIdentities.Num()||
            SharedIdentityNames.Num()!=SharedIdentities.Num()||x.SharedIdentityNames.Num()!=x.SharedIdentities.Num())return false;
        if(WeakIdentityNames!=x.WeakIdentityNames||SharedIdentityNames!=x.SharedIdentityNames||
            WeakIdentities.Num()!=x.WeakIdentities.Num()||SharedIdentities!=x.SharedIdentities)return false;
        for(int32 i=0;i<WeakIdentities.Num();++i)
            if(!WeakIdentities[i].HasSameIndexAndSerialNumber(x.WeakIdentities[i]))return false;
        if(Names!=x.Names||Words!=x.Words||Sections!=x.Sections||RetainedInput!=x.RetainedInput||
            RetainedInput.owner_before(x.RetainedInput)||x.RetainedInput.owner_before(RetainedInput)||
            RetainedValues.Num()!=x.RetainedValues.Num()||RetainedCameraEndpoints!=x.RetainedCameraEndpoints)return false;
        for(int32 i=0;i<RetainedValues.Num();++i)
            if(RetainedValues[i]!=x.RetainedValues[i]||RetainedValues[i].owner_before(x.RetainedValues[i])||x.RetainedValues[i].owner_before(RetainedValues[i]))return false;
        return true;
    }
};
inline bool MatchesNativePrepareGTBinding(const UActorComponent& object,const ISpeedComponent& adapter,const FPreparedNativeOwnerBinding& b)
{
    if(!IsInGameThread())return PreparationPrefix::IsCurrentPrivateSourceObservation(&object,&adapter,b);
    return b.BodyCensusCaptured&&b.NativeObject==&object&&b.Adapter==&adapter&&b.Class==object.GetClass()&&
        b.Object.HasSameIndexAndSerialNumber(FWeakObjectPtr(&object));
}
}
