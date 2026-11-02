#pragma once
#include <memory>
namespace Speed::NativeEpoch
{
// Native storage selection only, NOT a restore/lifecycle/configuration lease.
// The original epoch factory binds this through private native entries. There
// is no public caller boolean, mutable state view or install/revive operation.
class IOriginalStorageRoute
{
public:
    virtual ~IOriginalStorageRoute()=default;
    virtual bool SelectsOriginalStorage() const noexcept=0;
};
}
