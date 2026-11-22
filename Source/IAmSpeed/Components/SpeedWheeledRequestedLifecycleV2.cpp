#include "SpeedWheeledComponent.h"
#include "IAmSpeed/Input/NativeRequestedLifecycleV2.h"
#include "Misc/ScopeLock.h"
#include "Misc/ScopeTryLock.h"

bool USpeedWheeledComponent::ClaimCanonicalRequestedInputV2(std::shared_ptr<const Speed::Input::V2::FNativeRequestedBindingV2> Binding)
{
    if(!Binding||!Binding->OnOwner()||!Binding->Source()->IsOpen()||CanonicalRequestedBindingV2||bCanonicalInputPending||CanonicalRequestedInput
        ||!CanAdmitCanonicalRequestedInput()||Binding->Mapping().LocalComponent!=GetPublishedSimulationStableId()
        ||Binding->Mapping().LocalBodyGeneration!=GetCanonicalRequestedBodyGeneration())return false;
    FScopeTryLock Lock(&FrameInputProducerMutex);
    if(!Lock.IsLocked()||InputReservationV2||FrameInputStream||FrameInputStreamV2||HasLegacyRemoteInputAuthority()||IsTestInputOverrideEnabled())return false;
    CanonicalRequestedBindingV2=std::move(Binding);bCanonicalRequestedV2Retired=false;
    bCanonicalRequestedAuthority.store(true,std::memory_order_release);bProducedInputAuthority.store(true,std::memory_order_release);
    return true;
}
bool USpeedWheeledComponent::OwnsCanonicalRequestedInputV2(const std::shared_ptr<const Speed::Input::V2::FNativeRequestedBindingV2>& Binding)const
{return Binding&&Binding==CanonicalRequestedBindingV2&&Binding->OnOwner();}
bool USpeedWheeledComponent::RetireCanonicalRequestedInputV2(const std::shared_ptr<const Speed::Input::V2::FNativeRequestedBindingV2>& Binding)
{
    if(!Binding||!Binding->OnOwner())return false;
    if(!CanonicalRequestedBindingV2||CanonicalRequestedBindingV2!=Binding)return true;
    if(!NeutralizeCanonicalInputAtBoundary())return false;
    CanonicalRequestedBindingV2.reset();bCanonicalRequestedV2Retired=true;bInputFrameRejectedV2=false;
    // Keep canonical requested authority latched. No legacy/GT reacquisition,
    // body removal, movement release or fabricated digital edge follows.
    return true;
}
bool USpeedWheeledComponent::InstallCanonicalRequestedInputV2(uint64 Frame,
    std::shared_ptr<const Speed::Input::V2::FRequestedInputConsumption> Input,
    const std::shared_ptr<const Speed::Input::V2::FNativeRequestedBindingV2>& Binding)
{
    if(!OwnsCanonicalRequestedInputV2(Binding)||!Input||!Input->Request
        ||Input->Request->GetState().Frame!=Frame||bCanonicalInputPending||CanonicalInputSnapshot)return false;
    // Do not re-read a socket/host fence here: N already has its native token.
    CanonicalRequestedInput=std::move(Input);bCanonicalInputPending=true;bInputFrameRejectedV2=false;return true;
}
