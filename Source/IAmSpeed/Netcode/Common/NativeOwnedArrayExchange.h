#pragma once

#include "CoreMinimal.h"
#include "Templates/UnrealTemplate.h"
#include <type_traits>

namespace Speed::NativeState
{
// The generic Unreal Swap takes the bitwise branch for this exact allocator.
// The plan keeps the original allocation alive through common publication.
template<class Element>
void ExchangeOwnedDefaultArrays(TArray<Element, FDefaultAllocator>& Target,
    TArray<Element, FDefaultAllocator>& Staged) noexcept
{
    using FArray = TArray<Element, FDefaultAllocator>;
    static_assert(TUseBitwiseSwap<FArray>::Value);
    static_assert(std::is_move_constructible_v<FArray>);
    ::Swap<FArray>(Target, Staged);
}
}
