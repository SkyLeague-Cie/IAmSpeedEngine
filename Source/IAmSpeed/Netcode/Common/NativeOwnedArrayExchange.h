#pragma once
#include "CoreMinimal.h"
#include "Templates/UnrealTemplate.h"
#include <type_traits>

namespace Speed::NativeState
{
// Exact UE default-heap TArray ownership exchange. The UE Swap specialization
// selected by this actual trait copies only the aligned array header bytes;
// it does not call an element constructor/destructor, allocator or callback.
// Do not generalize to another allocator or fall back to move assignment.
// This utility grants NO world/source/lifetime lease or restore admission.
template<class Element>
void ExchangeOwnedDefaultArrays(TArray<Element,FDefaultAllocator>& A,TArray<Element,FDefaultAllocator>& B) noexcept
{
    using FArray=TArray<Element,FDefaultAllocator>;
    static_assert(TUseBitwiseSwap<FArray>::Value,"Native finalization requires UE header-only swap");
    static_assert(std::is_move_constructible_v<FArray>,"UE Swap requires a movable default array");
    ::Swap(A,B);
}
}
