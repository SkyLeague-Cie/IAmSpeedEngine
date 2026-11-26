#include "NativeRequestedPreparationFixture.h"
#include "NativeRequestedPreparationV2.h"
#include "HAL/PlatformTLS.h"

ANativeRequestedPreparationCar::ANativeRequestedPreparationCar(const FObjectInitializer& Initializer)
    :Super(Initializer.SetDefaultSubobjectClass<UNativeRequestedPreparationProbe>(VehicleMovementComponentName)){}

bool UNativeRequestedPreparationProbe::CompareCanonicalRequestedPreparationV2(
    const Speed::FNativeRequestedPreparationContextV2& C) const
{
#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
    ++ComparisonCalls;ComparisonThread=FPlatformTLS::GetCurrentThreadId();
    if(ThrowComparison)throw std::runtime_error("intentional native comparison probe exception");
    if(IsInGameThread()||RejectComparison)return false;
    using namespace Speed::Netcode::Agreement;
    const auto& O=C.ExpectedOffer();const auto& R=C.ReceivedReply();const auto& B=C.Boundary();
    const FBytes ComparisonProbeOnly={'C','T','X',1};
    // Opaque fixture bytes demonstrate a real game callback, not completeness
    // of any gameplay/physics policy. SupportsCanonicalRequestedInputsV2 stays false.
    return O.GameCodec==0x43545801&&O.GameCodecVersion==1&&O.Profiles.size()==1
        &&O.Profiles[0].Route==EProfileRoute::ClientInputsAtServer&&O.Profiles[0].Bytes==ComparisonProbeOnly
        &&R.Profiles.size()==1&&R.Profiles[0].Route==O.Profiles[0].Route&&R.Profiles[0].Bytes==ComparisonProbeOnly
        &&C.EffectiveContract()&&C.Command().Binding.Contract
        &&C.EffectiveContract()->GetFingerprint()==C.Command().Binding.Contract->GetFingerprint()
        &&O.WireContract==std::array<std::uint8_t,32>{0x43,0x54,0x58,1}
        &&B.LocalComponent==GetPublishedSimulationStableId()
        &&B.BodyGeneration==GetCanonicalRequestedBodyGeneration()
        &&B.CanonicalFrame==0&&B.LogicalRateNumerator==300&&B.LogicalRateDenominator==1;
#else
    return false;
#endif
}
