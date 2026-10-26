#pragma once
// Scalar-only little-endian archive. No padding, UObject, pointers or quantization.
#include <cstdint>
#include <cstddef>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <type_traits>
#include <vector>
namespace Speed::Replay
{
using FBytes=std::vector<uint8_t>;
struct FFieldError:std::runtime_error { using std::runtime_error::runtime_error; };
class FFieldWriter
{
    FBytes Data;
public:
    static constexpr size_t MaxBytes=1024*1024;
    template<class T> void Scalar(const T& Value)
    {
        if constexpr(std::is_enum_v<T>) Scalar(static_cast<std::underlying_type_t<T>>(Value));
        else if constexpr(std::is_same_v<T,bool>) Scalar(uint8_t(Value?1:0));
        else
        {
            static_assert(std::is_integral_v<T> || std::is_floating_point_v<T>);
            static_assert(sizeof(T)==1 || sizeof(T)==2 || sizeof(T)==4 || sizeof(T)==8);
            static_assert(!std::is_floating_point_v<T> || std::numeric_limits<T>::is_iec559);
            if(Data.size()>MaxBytes-sizeof(T)) throw FFieldError("state size limit");
            using U=std::conditional_t<sizeof(T)==1,uint8_t,std::conditional_t<sizeof(T)==2,uint16_t,std::conditional_t<sizeof(T)==4,uint32_t,uint64_t>>>;
            U Bits=0; std::memcpy(&Bits,&Value,sizeof(T));
            for(size_t I=0;I<sizeof(T);++I) Data.push_back(uint8_t(Bits>>(8*I)));
        }
    }
    template<class V> void Vector(const V& Value) { Scalar(Value.X);Scalar(Value.Y);Scalar(Value.Z); }
    template<class Q> void Quat(const Q& Value) { Vector(Value);Scalar(Value.W); }
    template<class T,size_t N> void Array(const T (&Values)[N]) { for(const auto& Value:Values) Scalar(Value); }
    const FBytes& Bytes() const { return Data; }
};
class FFieldReader
{
    const FBytes& Data;size_t At=0;
public:
    explicit FFieldReader(const FBytes& Bytes):Data(Bytes)
    { if(Data.size()>FFieldWriter::MaxBytes) throw FFieldError("state size limit"); }
    template<class T> void Scalar(T& Value)
    {
        if constexpr(std::is_enum_v<T>) {std::underlying_type_t<T> V{};Scalar(V);Value=static_cast<T>(V);}
        else if constexpr(std::is_same_v<T,bool>) {uint8_t V=0;Scalar(V);if(V>1) throw FFieldError("invalid boolean");Value=V!=0;}
        else
        {
            static_assert(std::is_integral_v<T> || std::is_floating_point_v<T>);
            static_assert(sizeof(T)==1 || sizeof(T)==2 || sizeof(T)==4 || sizeof(T)==8);
            static_assert(!std::is_floating_point_v<T> || std::numeric_limits<T>::is_iec559);
            if(At>Data.size() || sizeof(T)>Data.size()-At) throw FFieldError("truncated state");
            using U=std::conditional_t<sizeof(T)==1,uint8_t,std::conditional_t<sizeof(T)==2,uint16_t,std::conditional_t<sizeof(T)==4,uint32_t,uint64_t>>>;
            U Bits=0;for(size_t I=0;I<sizeof(T);++I) Bits=U(Bits|U(uint64_t(Data[At++])<<(8*I)));
            std::memcpy(&Value,&Bits,sizeof(T));
        }
    }
    template<class V> void Vector(V& Value) { Scalar(Value.X);Scalar(Value.Y);Scalar(Value.Z); }
    template<class Q> void Quat(Q& Value) { Vector(Value);Scalar(Value.W); }
    template<class T,size_t N> void Array(T (&Values)[N]) { for(auto& Value:Values) Scalar(Value); }
    bool AtEnd()const{return At==Data.size();}
};
}
