#pragma once
#include "InputFrameV2.h"

namespace Speed::Input::V2
{
// An owner ordinal orders local boundary operations. It is never a device
// source sequence, network acknowledgement, or physical-completion serial.
struct FLocalInputPollOrdinal { std::uint64_t Value=0; };
enum class ERequestedInputOrigin : std::uint8_t
{ OriginalProducer, ConfirmedNetwork, LocalPrediction, LocalFallback };
enum class ERequestedAuthorityCause : std::uint8_t { Original, Prediction, Fallback };
struct FRequestedInputState
{
    FFrameNumber Frame=0;
    FActionValues Values{};
    std::uint32_t ActiveMask=0;
    bool Reset=false;
};

// Versioned, immutable requested input. The optional original is genuine V2
// evidence. There is deliberately no conversion from a forecast to FInputFrame
// and no source-sequence/generation fields in FRequestedInputState.
class FRequestedInputFrame final
{
public:
    static constexpr std::uint16_t Version=1;
    static std::optional<FRequestedInputFrame> FromOriginal(const FInputFrame& Original)
    {
        const auto& C=Original.GetContract();
        if(!C||!Original.IsValidFor(*C))return {};
        const auto& D=Original.GetData();
        return FRequestedInputFrame(C,{D.ConsumptionFrame,D.Values,D.ActiveMask,D.Reset},
            ERequestedInputOrigin::OriginalProducer,{},Original);
    }
    static std::optional<FRequestedInputFrame> FromNetwork(
        std::shared_ptr<const FInputActionContract> Contract,FRequestedInputState State,
        ERequestedInputOrigin Origin,std::optional<ERequestedAuthorityCause> AuthorityCause,
        std::optional<FInputFrame> Original={})
    {
        if(!Contract||Origin==ERequestedInputOrigin::OriginalProducer||Origin>ERequestedInputOrigin::LocalFallback
            ||(Origin==ERequestedInputOrigin::ConfirmedNetwork)!=AuthorityCause.has_value()
            ||(AuthorityCause&&*AuthorityCause>ERequestedAuthorityCause::Fallback)
            ||Original.has_value()!=(AuthorityCause&&*AuthorityCause==ERequestedAuthorityCause::Original)
            ||(Origin!=ERequestedInputOrigin::ConfirmedNetwork&&State.Reset))return {};
        for(std::size_t I=0;I<ActionCount;++I)
        {
            const auto* A=Contract->Find(static_cast<FActionId>(I));
            if(!A||A->Wiring==EActionWiring::Unwired)
            { if(State.Values[I]||(State.ActiveMask&(std::uint32_t{1}<<I)))return {}; }
            else if(!A->Accepts(State.Values[I]))return {};
        }
        if(Original)
        {
            const auto& D=Original->GetData();
            if(!Original->IsValidFor(*Contract)||D.ConsumptionFrame!=State.Frame||D.Values!=State.Values
                ||D.ActiveMask!=State.ActiveMask||D.Reset!=State.Reset)return {};
        }
        return FRequestedInputFrame(std::move(Contract),State,Origin,AuthorityCause,std::move(Original));
    }
    const FRequestedInputState& GetState() const noexcept { return State; }
    const std::shared_ptr<const FInputActionContract>& GetContract() const noexcept { return Contract; }
    ERequestedInputOrigin GetOrigin() const noexcept { return Origin; }
    std::optional<ERequestedAuthorityCause> GetAuthorityCause() const noexcept { return AuthorityCause; }
    const std::optional<FInputFrame>& GetOriginal() const noexcept { return Original; }
private:
    FRequestedInputFrame(std::shared_ptr<const FInputActionContract> C,FRequestedInputState S,
        ERequestedInputOrigin O,std::optional<ERequestedAuthorityCause> Cause,std::optional<FInputFrame> F)
        :Contract(std::move(C)),State(S),Origin(O),AuthorityCause(Cause),Original(std::move(F)){}
    std::shared_ptr<const FInputActionContract> Contract;
    FRequestedInputState State;
    ERequestedInputOrigin Origin;
    std::optional<ERequestedAuthorityCause> AuthorityCause;
    std::optional<FInputFrame> Original;
};
}
