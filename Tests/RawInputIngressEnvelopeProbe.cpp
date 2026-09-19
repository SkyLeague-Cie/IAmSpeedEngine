#include "../Source/IAmSpeed/Input/RawInputIngressEnvelope.h"

#include <cstdlib>
#include <cstring>
#include <iostream>
#include <limits>
#include <type_traits>
#include <utility>

using namespace Speed::Input::V2;
using namespace Speed::Lifecycle;

static unsigned Checks = 0;
static void Check(bool Value, const char* Message)
{
    ++Checks;
    if (!Value) { std::cerr << "FAIL " << Message << '\n'; std::exit(1); }
}

static bool SameFloat(const float& A, const float& B)
{ return std::memcmp(&A, &B, sizeof(float)) == 0; }

static bool SameValue(const FRawValue& A, const FRawValue& B)
{ return A.Control == B.Control && SameFloat(A.Value, B.Value); }

static bool SameSample(const FRawInputSample& A, const FRawInputSample& B)
{
    if (A.DeviceId != B.DeviceId || A.Generation.Value != B.Generation.Value ||
        A.Kind != B.Kind || A.Sequence != B.Sequence || A.Status != B.Status ||
        A.FinalState.size() != B.FinalState.size() || A.Changes.size() != B.Changes.size())
        return false;
    for (std::size_t I = 0; I < A.FinalState.size(); ++I)
        if (!SameValue(A.FinalState[I], B.FinalState[I])) return false;
    for (std::size_t I = 0; I < A.Changes.size(); ++I)
        if (!SameValue(A.Changes[I].State, B.Changes[I].State) ||
            A.Changes[I].Order.Sequence != B.Changes[I].Order.Sequence ||
            A.Changes[I].Order.WithinSequence != B.Changes[I].Order.WithinSequence)
            return false;
    return true;
}

static FRawInputSample Keyboard()
{
    FRawInputSample S;
    S.DeviceId = 11; S.Generation.Value = 12; S.Sequence = 3;
    S.Kind = ERawDeviceKind::Keyboard; S.Status = ERawSampleStatus::Valid;
    S.FinalState = {{{ERawControlKind::KeyboardUsage, 4}, 0.0f},
                    {{ERawControlKind::KeyboardUsage, 5}, 1.0f}};
    S.Changes = {{S.FinalState[1], {2, 7}}, {S.FinalState[0], {3, 9}}};
    return S;
}

static FRawInputSample Gamepad()
{
    FRawInputSample S;
    S.DeviceId = 21; S.Generation.Value = 22; S.Sequence = 23;
    S.Kind = ERawDeviceKind::Gamepad; S.Status = ERawSampleStatus::Resync;
    S.FinalState = {
        {{ERawControlKind::PadButton, static_cast<std::uint16_t>(EPadButton::South)}, 1.0f},
        {{ERawControlKind::PadButton, static_cast<std::uint16_t>(EPadButton::Menu)}, 0.0f},
        {{ERawControlKind::PadAxis, static_cast<std::uint16_t>(EPadAxis::LeftX)}, 0.17f},
        {{ERawControlKind::PadAxis, static_cast<std::uint16_t>(EPadAxis::LeftY)}, -0.0f},
        {{ERawControlKind::PadAxis, static_cast<std::uint16_t>(EPadAxis::RightX)}, -0.61f},
        {{ERawControlKind::PadAxis, static_cast<std::uint16_t>(EPadAxis::LeftTrigger)}, 0.66f}};
    return S;
}

static void RejectedWithoutMutation(const FParticipantHandle& Target,
    const FRawInputSample& Sample, const char* Message)
{
    const auto Before = Sample;
    Check(!FRawInputIngressEnvelope::CreateCopied(Target, {41}, {42}, 0, Sample), Message);
    Check(SameSample(Sample, Before), "rejection preserves caller values and ordering");
}

int main()
{
    static_assert(std::is_same_v<decltype(std::declval<FRawInputIngressEnvelope&>().GetSample()),
        const FRawInputSample&>, "sample access is const even on a mutable envelope");
    static_assert(std::is_same_v<decltype(std::declval<FRawInputIngressEnvelope&>().GetTarget()),
        const FParticipantHandle&>, "destination access is const");
    static_assert(!std::is_polymorphic_v<FRawInputIngressEnvelope>, "no virtual dispatch");
    static_assert(!std::is_default_constructible_v<FRawInputIngressEnvelope>, "checked construction");

    // Only Configured protocols are created. These handles are deliberately
    // synthetic/unregistered; structural acceptance must not imply admission.
    const auto Code = std::make_shared<const FCodeLease>(FCodeLease{23});
    TSessionLifecycle<2> A(31, Code);
    TSessionLifecycle<2> B(31, Code);
    const FParticipantHandle Target{A.GetKey(), 1, 37};
    const FParticipantHandle Other{B.GetKey(), 1, 37};
    const auto K = Keyboard();
    const auto G = Gamepad();
    Check(K.IsValid() && G.IsValid(), "authored raw fixtures meet existing contract");

    for (const auto Frame : {std::uint64_t{0}, std::numeric_limits<std::uint64_t>::max()})
    {
        for (const auto& Sample : {K, G})
        {
            const auto E = FRawInputIngressEnvelope::CreateCopied(Target, {41}, {42}, Frame, Sample);
            Check(bool(E), "valid sample is copied at either frame address boundary");
            Check(E->GetTarget().Session.GetEpoch() == 31 && E->GetTarget().Slot == 1 &&
                E->GetTarget().Generation == 37, "destination metadata retained");
            Check(E->GetStreamEpoch().Value == 41 && E->GetMappingRevision().Value == 42 &&
                E->GetConsumptionFrame() == Frame, "distinct addressing fields retained");
            Check(SameSample(E->GetSample(), Sample), "all raw fields retained without transforms");
            Check(E->GetSample().FinalState.data() != Sample.FinalState.data(), "baseline storage independent");
            Check(Sample.Changes.empty() || E->GetSample().Changes.data() != Sample.Changes.data(),
                "change storage independent");
        }
    }

    std::optional<FRawInputIngressEnvelope> Retained;
    std::weak_ptr<const FCodeLease> CodeWitness;
    {
        auto TemporaryCode = std::make_shared<const FCodeLease>(FCodeLease{53});
        CodeWitness = TemporaryCode;
        TSessionLifecycle<2> TemporarySession(51, TemporaryCode);
        FParticipantHandle TemporaryTarget{TemporarySession.GetKey(), 0, 52};
        auto TemporarySample = K;
        Retained = FRawInputIngressEnvelope::CreateCopied(TemporaryTarget, {41}, {42}, 7, TemporarySample);
        Check(bool(Retained), "copy transient caller-owned data");
        TemporaryTarget = {};
        TemporarySample.FinalState[0].Value = 1.0f;
        TemporarySample.Changes[0].Order = {999, 999};
        TemporarySample.FinalState.clear(); TemporarySample.Changes.clear();
        Check(SameSample(Retained->GetSample(), K), "caller mutation cannot alter copy");
    }
    Check(CodeWitness.expired(), "envelope does not retain a code lease or configured session");
    Check(Retained->GetTarget().Session.GetEpoch() == 51 && SameSample(Retained->GetSample(), K),
        "address and raw copy survive caller destruction");

    Check(bool(FRawInputIngressEnvelope::CreateCopied(Other, {41}, {42}, 7, K)),
        "equal-epoch foreign key is structurally accepted, never authenticated here");
    auto Unregistered = Target;
    Unregistered.Slot = std::numeric_limits<std::size_t>::max();
    Unregistered.Generation = std::numeric_limits<std::uint64_t>::max();
    const auto UnregisteredEnvelope = FRawInputIngressEnvelope::CreateCopied(Unregistered, {41}, {42}, 7, K);
    Check(bool(UnregisteredEnvelope) && UnregisteredEnvelope->GetTarget().Slot == Unregistered.Slot &&
        UnregisteredEnvelope->GetTarget().Generation == Unregistered.Generation,
        "no capacity-specific authorization or integer truncation");
    auto InvalidTarget = Target; InvalidTarget.Session = {};
    RejectedWithoutMutation(InvalidTarget, K, "default session key rejected");
    InvalidTarget = Target; InvalidTarget.Generation = 0;
    RejectedWithoutMutation(InvalidTarget, K, "zero participant generation rejected");
    Check(!FRawInputIngressEnvelope::CreateCopied(Target, {}, {42}, 0, K), "zero stream epoch rejected");
    Check(!FRawInputIngressEnvelope::CreateCopied(Target, {41}, {}, 0, K), "zero mapping revision rejected");

    for (unsigned Case = 0; Case != 17; ++Case)
    {
        auto Invalid = K;
        switch (Case)
        {
        case 0: Invalid.DeviceId = 0; break;
        case 1: Invalid.Generation.Value = 0; break;
        case 2: Invalid.Sequence = 0; break;
        case 3: Invalid.Status = ERawSampleStatus::Unsupported; break;
        case 4: Invalid.Status = ERawSampleStatus::Overflow; break;
        case 5: Invalid.Kind = static_cast<ERawDeviceKind>(255); break;
        case 6: Invalid.FinalState.clear(); break;
        case 7: Invalid.FinalState[0].Value = 0.5f; break;
        case 8: Invalid.FinalState[0].Value = std::numeric_limits<float>::quiet_NaN(); break;
        case 9: Invalid.FinalState[0].Value = std::numeric_limits<float>::infinity(); break;
        case 10: std::swap(Invalid.FinalState[0], Invalid.FinalState[1]); break;
        case 11: Invalid.FinalState[1].Control = Invalid.FinalState[0].Control; break;
        case 12: Invalid.Changes[1].Order = Invalid.Changes[0].Order; break;
        case 13: Invalid.Changes[0].State.Control.Code = 6; break;
        case 14: Invalid.Changes[1].State.Value = 1.0f; break;
        case 15: Invalid.Status = ERawSampleStatus::Resync; break;
        case 16: Invalid.FinalState.resize(FRawInputSample::MaxControls + 1); break;
        }
        RejectedWithoutMutation(Target, Invalid, "invalid raw sample rejected without repair");
    }
    auto InvalidAxis = G;
    InvalidAxis.FinalState[2].Value = -1.01f;
    RejectedWithoutMutation(Target, InvalidAxis, "stick outside raw domain rejected");
    InvalidAxis = G; InvalidAxis.FinalState.back().Value = -0.01f;
    RejectedWithoutMutation(Target, InvalidAxis, "negative trigger rejected");

    auto FullChanges = K;
    FullChanges.FinalState.resize(1); FullChanges.Changes.clear();
    for (std::size_t I = 0; I < FRawInputSample::MaxChanges; ++I)
    {
        const float Value = I % 2 == 0 ? 1.0f : 0.0f;
        FullChanges.Changes.push_back({{FullChanges.FinalState[0].Control, Value},
            {static_cast<std::uint64_t>(I) + 1, 0}});
    }
    FullChanges.Sequence = static_cast<std::uint64_t>(FRawInputSample::MaxChanges);
    FullChanges.FinalState[0].Value = FullChanges.Changes.back().State.Value;
    const auto FullEnvelope = FRawInputIngressEnvelope::CreateCopied(Target, {41}, {42}, 7, FullChanges);
    Check(bool(FullEnvelope) && SameSample(FullEnvelope->GetSample(), FullChanges),
        "all256 changes preserved without sorting or edge loss");
    ++FullChanges.Sequence;
    FullChanges.FinalState[0].Value = 1.0f;
    FullChanges.Changes.push_back({FullChanges.FinalState[0], {FullChanges.Sequence, 0}});
    RejectedWithoutMutation(Target, FullChanges, "257changes rejected, not truncated");

    Check(A.GetState() == ESessionState::Configured && B.GetState() == ESessionState::Configured,
        "fixtures remain configured, no runtime lifecycle operation");
    Check(!A.ParticipantState(Target) && !B.ParticipantState(Other),
        "structurally accepted handles are still unregistered");
    std::cout << "Raw input ingress envelope checks=" << Checks << " PASS\n";
}
