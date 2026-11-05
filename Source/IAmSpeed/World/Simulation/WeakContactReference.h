#pragma once
#include "CoreMinimal.h"
#include "UObject/WeakObjectPtrTemplates.h"
namespace Speed
{
// Valid weak identities compare directly to a raw ignored-component list.
// Preserve the ordinary weak comparison for null/stale/pending-garbage cases:
// native FWeakObjectPtr equality treats two invalid references as equal.
template<class T> bool ContainsWeakContactReference(const TArray<T*>& Values,const TWeakObjectPtr<T>& Reference)
{
    return Reference.IsValid()?Values.Contains(Reference.Get()):Values.Contains(Reference);
}
}
