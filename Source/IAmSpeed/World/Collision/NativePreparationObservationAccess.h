#pragma once
class UActorComponent;
class ISpeedComponent;
namespace Speed
{
struct FPreparedNativeOwnerBinding;
namespace PreparationPrefix
{
// True only inside the private Source observer, or the native one-use position
// dispatch, of the current cursor. It grants retained-row read access, never phase/physics
// admission. No weak resolution or UObject lookup is performed on the worker.
IAMSPEED_API const FPreparedNativeOwnerBinding* FindCurrentPrivateSourceObservation(const ISpeedComponent*) noexcept;
IAMSPEED_API bool IsCurrentPrivateSourceObservation(const UActorComponent*,
    const ISpeedComponent*,const FPreparedNativeOwnerBinding&) noexcept;
}
}
