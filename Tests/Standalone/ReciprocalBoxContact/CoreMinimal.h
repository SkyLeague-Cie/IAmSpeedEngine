#pragma once
#include <cstdint>
using uint8=std::uint8_t; using int8=std::int8_t; using uint32=std::uint32_t;
using int32=std::int32_t; using uint64=std::uint64_t;
constexpr int INDEX_NONE=-1;
class UPrimitiveComponent {};
class USSubBody : public UPrimitiveComponent {};
struct FVector {
    double X=0,Y=0,Z=0;
    FVector()=default;
    FVector(double x,double y,double z):X(x),Y(y),Z(z){}
    FVector operator-() const {return {-X,-Y,-Z};}
    bool operator==(const FVector& b)const{return X==b.X&&Y==b.Y&&Z==b.Z;}
    static const FVector ZeroVector;
};
inline const FVector FVector::ZeroVector{};
template<class T> class TWeakObjectPtr {
    T* Value=nullptr;
public:
    TWeakObjectPtr(T* p=nullptr):Value(p){}
    TWeakObjectPtr& operator=(T* p){Value=p;return *this;}
    T* Get()const{return Value;}
};
struct FHitResult {
    bool bBlockingHit=false,bStartPenetrating=false;
    FVector ImpactPoint,ImpactNormal,Location;
    float Time=0,PenetrationDepth=0;
    int32 FaceIndex=INDEX_NONE;
    TWeakObjectPtr<UPrimitiveComponent> Component;
};
