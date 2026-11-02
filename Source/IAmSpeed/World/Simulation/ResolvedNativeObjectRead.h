#pragma once
#include "CoreMinimal.h"
#include "UObject/ObjectPtr.h"
#include <stdexcept>

namespace Speed::NativeObjectRead
{
// The owning host must hold original birth pins and its short GC exclusion.
// This read never resolves/loads an object or invokes ObjectHandleRead. It is
// NOT a lifetime/module/escaped-view lease and cannot admit restoration.
class FUnresolvedOriginal final : public std::runtime_error
{
public: FUnresolvedOriginal():std::runtime_error("unresolved original native object handle"){}
};
template<class T> T* ResolvedOriginal(const TObjectPtr<T>& Ptr)
{
    if(!::IsObjectHandleResolved(Ptr.GetHandle()))
        throw FUnresolvedOriginal();
    return ObjectPtr_Private::Friend::NoAccessTrackingGetNoResolve(Ptr);
}
template<class T> bool MatchesOriginalBirth(const TObjectPtr<T>& Ptr,const TWeakObjectPtr<T>& Birth)
{
    auto* Original=Birth.Get();
    return Original&&::IsObjectHandleResolved(Ptr.GetHandle())
        &&ObjectPtr_Private::Friend::NoAccessTrackingGetNoResolve(Ptr)==Original;
}
template<class T> class TRead final
{
    const TObjectPtr<T>& Ptr; bool Original;
public:
    TRead(const TObjectPtr<T>& P,bool O):Ptr(P),Original(O){}
    T* Get()const{return Original?ResolvedOriginal(Ptr):Ptr.Get();}
    operator T*()const{return Get();}
    T* operator->()const{return Get();}
    explicit operator bool()const{return Ptr!=nullptr;}
};
template<class T> class TView final
{
    const TArray<TObjectPtr<T>>& Values; bool Original;
public:
    TView(const TArray<TObjectPtr<T>>& V,bool O):Values(V),Original(O){}
    int32 Num()const{return Values.Num();}
    bool IsValidIndex(int32 I)const{return Values.IsValidIndex(I);}
    TRead<T> operator[](int32 I)const{return {Values[I],Original};}
    struct FIterator
    {
        const TView* View; int32 Index;
        TRead<T> operator*()const{return (*View)[Index];}
        FIterator& operator++(){++Index;return *this;}
        bool operator!=(const FIterator& R)const{return Index!=R.Index;}
    };
    FIterator begin()const{return {this,0};}
    FIterator end()const{return {this,Num()};}
};
}
