#pragma once
#include "IAmSpeed/Input/InputProcessingPolicy.h"
#include <optional>

namespace Speed::Netcode::ContractOffer
{
namespace V = Input::V2;
// Control-plane description only. A decoded offer is UNTRUSTED data, not a
// binding, authority, session, producer, device, identity or permission receipt.
// The authenticated host selects/approves the entire contract/policy/mode before
// assigning identities and binding. NetInputTransport remains unchanged.
constexpr std::size_t MaxEnvelopeBytes = 128 * 1024;
constexpr std::uint16_t WireVersion = 1;
struct FOffer
{
    std::shared_ptr<const V::FInputActionContract> Contract;
    V::FInputProcessingPolicy Processing;
    V::EExactRemoteAdmission Mode = V::EExactRemoteAdmission::Closed;
};
namespace Detail
{
constexpr std::uint32_t Magic = 0x31434149; // "IAC1", explicit little endian.
static_assert(Speed::Input::ActionCount == 32, "Version1 has exactly32 policy axes");
static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559, "IEEE binary32 required");
struct FWriter
{
    std::vector<std::uint8_t> Bytes;
    bool Good = true;
    void U(std::uint64_t Value, unsigned Width)
    {
        if (!Width || Width > 8 || Bytes.size() > MaxEnvelopeBytes - Width) { Good = false; return; }
        for (unsigned I = 0; I < Width; ++I) Bytes.push_back(std::uint8_t(Value >> (8 * I)));
    }
    void Float(float Value)
    {
        if (!std::isfinite(Value)) { Good = false; return; }
        if (Value == 0) Value = 0; // Same positive-zero normalization as Create.
        std::uint32_t Bits = 0; std::memcpy(&Bits, &Value, sizeof(Bits)); U(Bits, 4);
    }
    void Text(const std::string& Value)
    {
        if (Value.empty() || Value.size() > 64 || Bytes.size() > MaxEnvelopeBytes - 1 - Value.size()) { Good = false; return; }
        U(Value.size(), 1); Bytes.insert(Bytes.end(), Value.begin(), Value.end());
    }
    void Modifiers(const std::vector<V::FScalarModifier>& Values)
    {
        if (Values.size() > 16) { Good = false; return; }
        U(Values.size(), 1);
        for (const auto& M : Values) { U(static_cast<std::uint8_t>(M.Kind), 1); Float(M.A); Float(M.B); }
    }
};
struct FReader
{
    const std::uint8_t* Data;
    std::size_t Size, Position = 0;
    bool Good = true;
    std::uint64_t U(unsigned Width) noexcept
    {
        if (!Width || Width > 8 || Width > Size - Position) { Good = false; return 0; }
        std::uint64_t Value = 0;
        for (unsigned I = 0; I < Width; ++I) Value |= std::uint64_t(Data[Position++]) << (8 * I);
        return Value;
    }
    float Float() noexcept
    {
        const auto Bits = std::uint32_t(U(4)); float Value = 0; std::memcpy(&Value, &Bits, sizeof(Value));
        if (!std::isfinite(Value)) Good = false;
        return Value;
    }
    std::string Text()
    {
        const auto N = std::size_t(U(1));
        if (!N || N > 64 || N > Size - Position) { Good = false; return {}; }
        std::string Value(reinterpret_cast<const char*>(Data + Position), N); Position += N; return Value;
    }
    bool Modifiers(std::vector<V::FScalarModifier>& Values)
    {
        const auto N = U(1); if (!Good || N > 16) return false;
        for (std::uint64_t I = 0; I < N; ++I)
        {
            const auto Kind = U(1); if (Kind > static_cast<std::uint8_t>(V::EScalarModifier::Clamp)) return false;
            V::FScalarModifier M; M.Kind = static_cast<V::EScalarModifier>(Kind); M.A = Float(); M.B = Float();
            if (!Good) return false; Values.push_back(M);
        }
        return Good;
    }
};
}
// Encoding normalizes only the order which Create itself declares immaterial:
// aliases and physical destinations. Mapping contributions/modifiers keep order.
inline std::optional<std::vector<std::uint8_t>> Encode(const V::FInputActionContractDescription& Description,
    const V::FInputProcessingPolicy& Processing, V::EExactRemoteAdmission Mode) noexcept
{
    try
    {
        if (!V::AdmitsOriginalDeviceRemote(Mode) || Description.Actions.size() < 3 || Description.Actions.size() > Speed::Input::ActionCount
            || Description.Mapping.size() > 256 || Description.Physical.size() != 3) return {};
        // Bound caller-owned descriptions before Create copies any vectors.
        for (const auto& A : Description.Actions)
        {
            if (A.Owner.empty() || A.Owner.size() > 64 || A.Name.empty() || A.Name.size() > 64
                || A.Aliases.size() > 16 || A.Modifiers.size() > 16) return {};
            for (const auto& Alias : A.Aliases) if (Alias.empty() || Alias.size() > 64) return {};
        }
        for (const auto& B : Description.Mapping) if (B.Modifiers.size() > 16) return {};
        const auto C = V::FInputActionContract::Create(Description);
        if (!C || !V::ValidInputProcessingPolicy(V::EProducerContract::ExactRemote, Processing, *C, Mode)) return {};
        const auto& D = C->GetDescription(); Detail::FWriter W;
        W.U(Detail::Magic, 4); W.U(WireVersion, 2); W.U(0, 2); W.U(0, 4); W.U(static_cast<std::uint8_t>(Mode), 1);
        W.U(D.Revision.Value, 8); W.U(D.Actions.size(), 1);
        for (const auto& A : D.Actions)
        {
            W.U(A.Id, 1); W.Text(A.Owner); W.Text(A.Name); W.U(A.Version, 4); W.U(A.Aliases.size(), 1);
            for (const auto& Alias : A.Aliases) W.Text(Alias);
            W.U(static_cast<std::uint8_t>(A.Type), 1); W.U(static_cast<std::uint8_t>(A.Wiring), 1); W.U(A.Signed ? 1 : 0, 1);
            W.U(static_cast<std::uint16_t>(A.Quantization), 2);
            W.Float(A.ActivateAbove); W.Float(A.DeactivateAtOrBelow); W.Float(A.Deadzone); W.Float(A.Exponent); W.Float(A.Sensitivity);
            W.U(static_cast<std::uint8_t>(A.Accumulation), 1); W.Modifiers(A.Modifiers);
        }
        W.U(D.Mapping.size(), 2);
        for (const auto& B : D.Mapping)
        {
            W.U(static_cast<std::uint8_t>(B.Control.Kind), 1); W.U(B.Control.Code, 2); W.U(B.Action, 1);
            W.Float(B.Scale); W.Modifiers(B.Modifiers);
        }
        W.U(D.Physical.size(), 1);
        for (const auto& B : D.Physical) { W.U(B.Action, 1); W.U(static_cast<std::uint8_t>(B.Destination), 1); }
        W.U(Speed::Input::ActionCount, 1); for (const auto Step : Processing.Step) W.U(Step, 2);
        if (!W.Good) return {};
        for (unsigned I = 0; I < 4; ++I) W.Bytes[8 + I] = std::uint8_t(W.Bytes.size() >> (8 * I));
        return std::move(W.Bytes);
    }
    catch (...) { return {}; }
}
inline std::optional<FOffer> Decode(const std::uint8_t* Data, std::size_t Size) noexcept
{
    try
    {
        if (!Data || Size < 22 || Size > MaxEnvelopeBytes) return {};
        Detail::FReader R{Data, Size};
        if (R.U(4) != Detail::Magic || R.U(2) != WireVersion || R.U(2) != 0 || R.U(4) != Size) return {};
        const auto Mode = static_cast<V::EExactRemoteAdmission>(R.U(1));
        if (!V::AdmitsOriginalDeviceRemote(Mode)) return {};
        V::FInputActionContractDescription D; D.Revision.Value = R.U(8);
        const auto Count = R.U(1); if (!R.Good || Count < 3 || Count > Speed::Input::ActionCount) return {};
        for (std::uint64_t I = 0; I < Count; ++I)
        {
            V::FActionDefinition A; A.Id = static_cast<Speed::Input::FActionId>(R.U(1)); A.Owner = R.Text(); A.Name = R.Text();
            A.Version = std::uint32_t(R.U(4)); const auto Aliases = R.U(1); if (!R.Good || Aliases > 16) return {};
            for (std::uint64_t J = 0; J < Aliases; ++J) A.Aliases.push_back(R.Text());
            const auto Type = R.U(1), Wiring = R.U(1), Signed = R.U(1), Quantization = R.U(2);
            if (Type > 1 || Wiring > 1 || Signed > 1 || Quantization > 32767) return {};
            A.Type = static_cast<V::EActionType>(Type); A.Wiring = static_cast<V::EActionWiring>(Wiring); A.Signed = Signed == 1; A.Quantization = std::int16_t(Quantization);
            A.ActivateAbove = R.Float(); A.DeactivateAtOrBelow = R.Float(); A.Deadzone = R.Float(); A.Exponent = R.Float(); A.Sensitivity = R.Float();
            const auto Accumulation = R.U(1); if (Accumulation > 1) return {};
            A.Accumulation = static_cast<V::EActionAccumulation>(Accumulation);
            if (!R.Modifiers(A.Modifiers) || !R.Good) return {}; D.Actions.push_back(std::move(A));
        }
        const auto Mappings = R.U(2); if (!R.Good || Mappings > 256) return {};
        for (std::uint64_t I = 0; I < Mappings; ++I)
        {
            V::FRawActionBinding B; const auto Kind = R.U(1); if (Kind > static_cast<std::uint8_t>(V::ERawControlKind::MouseButton)) return {};
            B.Control.Kind = static_cast<V::ERawControlKind>(Kind); B.Control.Code = std::uint16_t(R.U(2)); B.Action = static_cast<Speed::Input::FActionId>(R.U(1));
            B.Scale = R.Float(); if (!R.Modifiers(B.Modifiers) || !R.Good) return {}; D.Mapping.push_back(std::move(B));
        }
        const auto Physical = R.U(1); if (!R.Good || Physical != 3) return {};
        for (unsigned I = 0; I < 3; ++I)
        {
            V::FPhysicalBindingDescriptor B; B.Action = static_cast<Speed::Input::FActionId>(R.U(1)); const auto Destination = R.U(1); if (Destination > 2) return {};
            B.Destination = static_cast<V::EPhysicalDestination>(Destination); D.Physical.push_back(B);
        }
        if (R.U(1) != Speed::Input::ActionCount) return {};
        V::FInputProcessingPolicy Processing; for (auto& Step : Processing.Step) Step = std::uint16_t(R.U(2));
        if (!R.Good || R.Position != Size) return {};
        const auto C = V::FInputActionContract::Create(std::move(D));
        if (!C || !V::ValidInputProcessingPolicy(V::EProducerContract::ExactRemote, Processing, *C, Mode)) return {};
        // Reject alternate ordering, negative-zero aliases and every other
        // noncanonical encoding. Fingerprint bytes are never decoded as wire.
        const auto Canonical = Encode(C->GetDescription(), Processing, Mode);
        if (!Canonical || Canonical->size() != Size || !std::equal(Canonical->begin(), Canonical->end(), Data)) return {};
        return FOffer{C, Processing, Mode};
    }
    catch (...) { return {}; }
}
}
