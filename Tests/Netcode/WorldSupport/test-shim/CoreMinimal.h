#pragma once
// ORDINARY CONTROL-FLOW TEST SHIM ONLY. Never add this directory to UE module
// include paths or treat its containers/UObject-shaped values as native UE.
#include <cstdint>
#include <map>
#include <memory>
#include <set>
#include <vector>
using uint8 = std::uint8_t; using uint32 = std::uint32_t; using uint64 = std::uint64_t; using int32 = std::int32_t;
struct FVector { double X = 0, Y = 0, Z = 0; FVector() = default; FVector(double A, double B, double C) : X(A), Y(B), Z(C) {} };
struct FQuat { double X = 0, Y = 0, Z = 0, W = 1; FQuat() = default; FQuat(double A, double B, double C, double D) : X(A), Y(B), Z(C), W(D) {} };
template<class T> class TConstArrayView
{
    const T* Ptr; int32 Count;
public:
    TConstArrayView(const T* P, int32 N) noexcept : Ptr(P), Count(N) {}
    const T* GetData() const noexcept { return Ptr; } int32 Num() const noexcept { return Count; }
};
template<class T> class TArray : public std::vector<T>
{
public:
    int32 Num() const noexcept { return static_cast<int32>(this->size()); }
    bool IsEmpty() const noexcept { return this->empty(); }
    const T* GetData() const noexcept { return this->data(); }
    void Reserve(int32 N) { this->reserve(static_cast<std::size_t>(N)); }
    int32 Add(T V) { this->push_back(std::move(V)); return Num() - 1; }
};
template<class K, class V> class TMap : public std::map<K, V>
{
public:
    int32 Num() const noexcept { return static_cast<int32>(this->size()); }
    V FindRef(const K& Key) const { const auto I = this->find(Key); return I == this->end() ? V{} : I->second; }
    V& Add(K Key) { return (*this)[Key]; }
    V& Add(K Key, V Value) { return this->insert_or_assign(std::move(Key), std::move(Value)).first->second; }
};
template<class T> class TSet : public std::set<T>
{
public:
    void Add(T V) { this->insert(std::move(V)); }
};
