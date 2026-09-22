#pragma once

#include "InputFrame.h"
#include "RawInput.h"
#include <algorithm>
#include <cstring>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace Speed::Input::V2
{
enum class EActionType : std::uint8_t { Bool, Axis1D };
enum class EActionWiring : std::uint8_t { Unwired, Wired };
enum class EStateAction : std::uint8_t { Started, Triggered, Completed };
enum class EPhysicalDestination : std::uint8_t { Throttle, Brake, Steering };
enum class EActionAccumulation : std::uint8_t { Sum, HighestAbsolute };
enum class EScalarModifier : std::uint8_t { Scale, Deadzone, Exponent, Clamp };
// Ordered, frame-independent transforms. Unsupported UE transforms are refused
// by the importer rather than silently dropped or evaluated on the game thread.
struct FScalarModifier
{
	EScalarModifier Kind = EScalarModifier::Scale;
	float A = 1;
	float B = 0;
};
struct FActionDefinition
{
	FActionId Id = 0;
	std::string Owner;
	std::string Name;
	std::uint32_t Version = 1;
	std::vector<std::string> Aliases;
	EActionType Type = EActionType::Bool;
	EActionWiring Wiring = EActionWiring::Unwired;
	bool Signed = false;
	std::int16_t Quantization = 1;
	// Activation uses absolute normalized magnitude: inactive -> active when
	// > ActivateAbove; active -> inactive when <= DeactivateAtOrBelow.
	float ActivateAbove = 0;
	float DeactivateAtOrBelow = 0;
	float Deadzone = 0;
	float Exponent = 1;
	float Sensitivity = 1;
	EActionAccumulation Accumulation = EActionAccumulation::Sum;
	std::vector<FScalarModifier> Modifiers;
	bool Accepts(std::int16_t Value) const
	{ return Value >= (Signed ? -Quantization : 0) && Value <= Quantization; }
};
struct FRawActionBinding
{
	FRawControl Control; FActionId Action = 0; float Scale = 1;
	std::vector<FScalarModifier> Modifiers;
};
struct FPhysicalBindingDescriptor { FActionId Action = 0; EPhysicalDestination Destination = EPhysicalDestination::Throttle; };
struct FInputActionContractDescription
{
	FMappingRevision Revision;
	std::vector<FActionDefinition> Actions; // Strict ID order, stable across games/saves.
	std::vector<FRawActionBinding> Mapping; // Contribution order is significant.
	std::vector<FPhysicalBindingDescriptor> Physical;
};

// Exact canonical bytes, NOT std::hash or object memory. Equality is collision
// free; this is an in-process contract fingerprint, not a network wire format.
using FContractFingerprint = std::vector<std::uint8_t>;
class FInputActionContract final
{
public:
	static std::shared_ptr<const FInputActionContract> Create(FInputActionContractDescription Description)
	{
		if (!Validate(Description)) return {};
		// Alias and destination enumeration order has no runtime meaning.
		// Mapping contribution order does, and is deliberately preserved.
		for (auto& A : Description.Actions) std::sort(A.Aliases.begin(), A.Aliases.end());
		std::sort(Description.Physical.begin(), Description.Physical.end(),
			[](const FPhysicalBindingDescriptor& A, const FPhysicalBindingDescriptor& B)
			{ return A.Destination < B.Destination; });
		return std::shared_ptr<const FInputActionContract>(new FInputActionContract(std::move(Description)));
	}
	const FInputActionContractDescription& GetDescription() const { return Description; }
	const FContractFingerprint& GetFingerprint() const { return Fingerprint; }
	const FActionDefinition* Find(FActionId Id) const
	{
		for (const auto& A : Description.Actions) if (A.Id == Id) return &A;
		return nullptr;
	}
	static std::vector<FActionDefinition> BaseActions()
	{
		std::vector<FActionDefinition> Result;
		for (FActionId Id = Throttle; Id <= Steering; ++Id)
		{
			FActionDefinition A;
			A.Id = Id; A.Owner = "IAmSpeed";
			A.Name = Id == Throttle ? "Throttle" : (Id == Brake ? "Brake" : "Steering");
			A.Type = EActionType::Axis1D; A.Wiring = EActionWiring::Wired;
			A.Signed = Id == Steering; A.Quantization = A.Signed ? 127 : 255;
			Result.push_back(A);
		}
		return Result;
	}
private:
	static bool ValidModifiers(const std::vector<FScalarModifier>& Modifiers)
	{
		if (Modifiers.size() > 16) return false;
		for (const auto& M : Modifiers)
		{
			if (!std::isfinite(M.A) || !std::isfinite(M.B)) return false;
			switch (M.Kind)
			{
			case EScalarModifier::Scale: if (M.B != 0) return false; break;
			case EScalarModifier::Deadzone: if (M.A < 0 || M.A >= M.B || M.B > 1) return false; break;
			case EScalarModifier::Exponent: if ((M.A != 1 && M.A != 2) || M.B != 0) return false; break;
			case EScalarModifier::Clamp: if (M.A < -1 || M.B > 1 || M.A > M.B) return false; break;
			default: return false;
			}
		}
		return true;
	}
	static bool Token(const std::string& S)
	{
		if (S.empty() || S.size() > 64) return false;
		for (const char C : S) if (!((C >= 'A' && C <= 'Z') || (C >= 'a' && C <= 'z')
			|| (C >= '0' && C <= '9') || C == '_' || C == '.')) return false;
		return true;
	}
	static bool Validate(const FInputActionContractDescription& D)
	{
		if (!D.Revision.Value || D.Actions.size() < 3 || D.Actions.size() > ActionCount
			|| D.Mapping.size() > 256 || D.Physical.size() != 3) return false;
		const auto Base = BaseActions();
		std::vector<std::string> Names;
		for (std::size_t I = 0; I < D.Actions.size(); ++I)
		{
			const auto& A = D.Actions[I];
			if (A.Id >= ActionCount || (I && A.Id <= D.Actions[I - 1].Id) || !Token(A.Owner)
				|| !Token(A.Name) || !A.Version || A.Aliases.size() > 16 || A.Type > EActionType::Axis1D
				|| A.Wiring > EActionWiring::Wired || A.Quantization <= 0
				|| A.Accumulation > EActionAccumulation::HighestAbsolute || !ValidModifiers(A.Modifiers)
				|| !std::isfinite(A.ActivateAbove) || !std::isfinite(A.DeactivateAtOrBelow)
				|| A.DeactivateAtOrBelow < 0 || A.ActivateAbove < A.DeactivateAtOrBelow || A.ActivateAbove >= 1
				|| !std::isfinite(A.Deadzone) || A.Deadzone < 0 || A.Deadzone >= 1
				|| !std::isfinite(A.Exponent) || A.Exponent <= 0
				|| !std::isfinite(A.Sensitivity) || A.Sensitivity <= 0) return false;
			if (A.Type == EActionType::Bool && (A.Signed || A.Quantization != 1
				|| A.Deadzone != 0 || A.Exponent != 1 || A.Sensitivity != 1 || !A.Modifiers.empty())) return false;
			if (I < 3 && (A.Id != Base[I].Id || A.Owner != Base[I].Owner || A.Name != Base[I].Name
				|| A.Type != Base[I].Type || A.Signed != Base[I].Signed || A.Quantization != Base[I].Quantization
				|| A.Wiring != EActionWiring::Wired)) return false;
			auto AddName = [&](const std::string& Name)
			{
				if (!Token(Name)) return false;
				const auto Qualified = A.Owner + ":" + Name;
				for (const auto& Old : Names) if (Old == Qualified) return false;
				Names.push_back(Qualified); return true;
			};
			if (!AddName(A.Name)) return false;
			for (const auto& Alias : A.Aliases) if (!AddName(Alias)) return false;
		}
		auto Find = [&](FActionId Id) -> const FActionDefinition*
		{ for (const auto& A : D.Actions) if (A.Id == Id) return &A; return nullptr; };
		for (const auto& B : D.Mapping)
		{
			const auto* A = Find(B.Action);
			if (!A || A->Wiring != EActionWiring::Wired || !B.Control.IsValid()
				|| !std::isfinite(B.Scale) || B.Scale == 0 || !ValidModifiers(B.Modifiers)
				|| (A->Type == EActionType::Bool && (B.Scale != 1 || !B.Modifiers.empty()))) return false;
		}
		std::array<bool, 3> Destinations{};
		for (const auto& B : D.Physical)
		{
			const auto N = static_cast<std::size_t>(B.Destination);
			const auto* A = Find(B.Action);
			if (N >= Destinations.size() || Destinations[N] || !A || A->Wiring != EActionWiring::Wired
				|| A->Type != EActionType::Axis1D || A->Signed != (N == 2)
				|| A->Quantization != (N == 2 ? 127 : 255)) return false;
			Destinations[N] = true;
		}
		return true;
	}
	void Add(std::uint64_t Value)
	{ for (unsigned I = 0; I < 8; ++I) { Fingerprint.push_back(static_cast<std::uint8_t>(Value & 255)); Value >>= 8; } }
	void AddText(const std::string& Text)
	{ Add(Text.size()); for (unsigned char C : Text) Fingerprint.push_back(C); }
	void AddFloat(float Value)
	{
		static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559, "IEEE binary32 required");
		if (Value == 0) Value = 0; // Canonical positive zero.
		std::uint32_t Bits = 0; std::memcpy(&Bits, &Value, sizeof(Bits)); Add(Bits);
	}
	void AddModifiers(const std::vector<FScalarModifier>& Modifiers)
	{
		Add(Modifiers.size());
		for (const auto& M : Modifiers) { Add(static_cast<unsigned>(M.Kind)); AddFloat(M.A); AddFloat(M.B); }
	}
	explicit FInputActionContract(FInputActionContractDescription D) : Description(std::move(D))
	{
		Add(3); Add(Description.Revision.Value); Add(Description.Actions.size());
		for (const auto& A : Description.Actions)
		{
			Add(A.Id); AddText(A.Owner); AddText(A.Name); Add(A.Version); Add(A.Aliases.size());
			for (const auto& Alias : A.Aliases) AddText(Alias);
			Add(static_cast<unsigned>(A.Type)); Add(static_cast<unsigned>(A.Wiring)); Add(A.Signed);
			Add(static_cast<std::uint16_t>(A.Quantization)); AddFloat(A.ActivateAbove); AddFloat(A.DeactivateAtOrBelow);
			AddFloat(A.Deadzone); AddFloat(A.Exponent); AddFloat(A.Sensitivity);
			Add(static_cast<unsigned>(A.Accumulation)); AddModifiers(A.Modifiers);
		}
		Add(Description.Mapping.size());
		for (const auto& B : Description.Mapping)
		{ Add(static_cast<unsigned>(B.Control.Kind)); Add(B.Control.Code); Add(B.Action); AddFloat(B.Scale); AddModifiers(B.Modifiers); }
		Add(Description.Physical.size());
		for (const auto& B : Description.Physical) { Add(B.Action); Add(static_cast<unsigned>(B.Destination)); }
	}
	const FInputActionContractDescription Description;
	FContractFingerprint Fingerprint;
};
}
