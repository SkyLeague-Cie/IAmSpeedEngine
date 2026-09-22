#include "InputMappingImport.h"
#include "InputMappingContext.h"
#include "InputModifiers.h"
#include "InputAction.h"
#include "InputCoreTypes.h"

#if PLATFORM_WINDOWS && !UE_SERVER
#include "Windows/AllowWindowsPlatformTypes.h"
#include "Windows/PreWindowsApi.h"
THIRD_PARTY_INCLUDES_START
#include "Windows/GameInputCanonicalControls.h"
THIRD_PARTY_INCLUDES_END
#include "Windows/PostWindowsApi.h"
#include "Windows/HideWindowsPlatformTypes.h"
#endif

namespace Speed::Input::V2
{
namespace
{
bool ImportModifiers(const TArray<TObjectPtr<UInputModifier>>& In, std::vector<FScalarModifier>& Out)
{
	for (const UInputModifier* Modifier : In)
	{
		// Exact classes only: a Blueprint/subclass may override the transform.
		if (!Modifier) return false;
		if (Modifier->GetClass() == UInputModifierNegate::StaticClass())
			Out.push_back({EScalarModifier::Scale, CastChecked<UInputModifierNegate>(Modifier)->bX ? -1.0f : 1.0f, 0});
		else if (Modifier->GetClass() == UInputModifierScalar::StaticClass())
			Out.push_back({EScalarModifier::Scale, float(CastChecked<UInputModifierScalar>(Modifier)->Scalar.X), 0});
		else if (Modifier->GetClass() == UInputModifierDeadZone::StaticClass())
		{
			const auto* Deadzone = CastChecked<UInputModifierDeadZone>(Modifier);
			if (Deadzone->Type != EDeadZoneType::Axial && Deadzone->Type != EDeadZoneType::Radial) return false;
			Out.push_back({EScalarModifier::Deadzone, Deadzone->LowerThreshold, Deadzone->UpperThreshold});
		}
		else if (Modifier->GetClass() == UInputModifierResponseCurveExponential::StaticClass())
			Out.push_back({EScalarModifier::Exponent, float(CastChecked<UInputModifierResponseCurveExponential>(Modifier)->CurveExponent.X), 0});
		else return false;
	}
	return true;
}
bool PadControl(const FKey& Key, FRawControl& Out)
{
	struct FButton { FKey Key; EPadButton Button; };
	const FButton Buttons[] = {
		{EKeys::Gamepad_FaceButton_Bottom, EPadButton::South}, {EKeys::Gamepad_FaceButton_Right, EPadButton::East},
		{EKeys::Gamepad_FaceButton_Left, EPadButton::West}, {EKeys::Gamepad_FaceButton_Top, EPadButton::North},
		{EKeys::Gamepad_DPad_Up, EPadButton::DPadUp}, {EKeys::Gamepad_DPad_Down, EPadButton::DPadDown},
		{EKeys::Gamepad_DPad_Left, EPadButton::DPadLeft}, {EKeys::Gamepad_DPad_Right, EPadButton::DPadRight},
		{EKeys::Gamepad_LeftShoulder, EPadButton::LeftShoulder}, {EKeys::Gamepad_RightShoulder, EPadButton::RightShoulder},
		{EKeys::Gamepad_LeftThumbstick, EPadButton::LeftStick}, {EKeys::Gamepad_RightThumbstick, EPadButton::RightStick},
		{EKeys::Gamepad_Special_Right, EPadButton::Menu}, {EKeys::Gamepad_Special_Left, EPadButton::View}};
	for (const auto& B : Buttons) if (Key == B.Key) { Out = {ERawControlKind::PadButton, uint16(B.Button)}; return true; }
	struct FAxis { FKey Key; EPadAxis Axis; };
	const FAxis Axes[] = {
		{EKeys::Gamepad_LeftX, EPadAxis::LeftX}, {EKeys::Gamepad_LeftY, EPadAxis::LeftY},
		{EKeys::Gamepad_RightX, EPadAxis::RightX}, {EKeys::Gamepad_RightY, EPadAxis::RightY},
		{EKeys::Gamepad_LeftTriggerAxis, EPadAxis::LeftTrigger}, {EKeys::Gamepad_RightTriggerAxis, EPadAxis::RightTrigger}};
	for (const auto& A : Axes) if (Key == A.Key) { Out = {ERawControlKind::PadAxis, uint16(A.Axis)}; return true; }
	return false; // Virtual directions, gyro, touch and digital trigger thresholds need explicit support.
}
}

bool ImportInputMappings(const UInputMappingContext* Context, const TMap<const UInputAction*, FActionId>& Actions,
	FInputActionContractDescription& Description, FString& Error)
{
	check(IsInGameThread());
	auto Reject = [&](const TCHAR* Reason) { Error = Reason; return false; };
	if (!Context || !Description.Mapping.empty()) return Reject(TEXT("Missing context or nonempty import destination"));
	auto Result = Description;
	TSet<FActionId> Seen;
#if PLATFORM_WINDOWS && !UE_SERVER
	const HKL Layout = GetKeyboardLayout(0); // Frozen for this session; never queried by physical polling.
#endif
	for (const auto& Mapping : Context->GetMappings())
	{
		if (!Mapping.Key.IsValid()) continue; // Saved remapping disables conflicting secondary entries this way.
		const UInputAction* Action = Mapping.Action;
		const FActionId* Id = Actions.Find(Action);
		if (!Action || !Id || *Id >= Result.Actions.size()) return Reject(TEXT("Unidentified action asset"));
		auto& Definition = Result.Actions[*Id];
		const bool Boolean = Definition.Type == EActionType::Bool;
		if (Definition.Id != *Id || Action->ValueType != (Boolean ? EInputActionValueType::Boolean : EInputActionValueType::Axis1D)
			|| !Action->Triggers.IsEmpty() || !Mapping.Triggers.IsEmpty())
			return Reject(TEXT("Unsupported action type or trigger: time/chord/custom triggers are not silently imported"));
		if (!Seen.Contains(*Id))
		{
			if (Boolean && !Action->Modifiers.IsEmpty()) return Reject(TEXT("Boolean action modifiers unsupported"));
			std::vector<FScalarModifier> ActionModifiers;
			if (!ImportModifiers(Action->Modifiers, ActionModifiers)) return Reject(TEXT("Unsupported action modifier"));
			ActionModifiers.insert(ActionModifiers.end(), Definition.Modifiers.begin(), Definition.Modifiers.end());
			Definition.Modifiers = std::move(ActionModifiers);
			if (Action->AccumulationBehavior == EInputActionAccumulationBehavior::TakeHighestAbsoluteValue)
				Definition.Accumulation = EActionAccumulation::HighestAbsolute;
			else if (Action->AccumulationBehavior == EInputActionAccumulationBehavior::Cumulative)
				Definition.Accumulation = EActionAccumulation::Sum;
			else return Reject(TEXT("Unsupported action accumulation"));
			Seen.Add(*Id);
		}
		FRawActionBinding Binding; Binding.Action = *Id;
		if (Boolean && !Mapping.Modifiers.IsEmpty()) return Reject(TEXT("Boolean mapping modifiers unsupported"));
		if (!ImportModifiers(Mapping.Modifiers, Binding.Modifiers)) return Reject(TEXT("Unsupported mapping modifier"));
		if (Mapping.Key.IsGamepadKey())
		{
			if (!PadControl(Mapping.Key, Binding.Control)) return Reject(TEXT("Unsupported canonical gamepad control"));
		}
		else
		{
#if PLATFORM_WINDOWS && !UE_SERVER
			const uint32* VirtualKey = nullptr; const uint32* Character = nullptr;
			FInputKeyManager::Get().GetCodesFromKey(Mapping.Key, VirtualKey, Character);
			uint32 VK = VirtualKey ? *VirtualKey : 0;
			if (!VK && Character)
			{
				const TCHAR Glyph = FChar::ToLower(TCHAR(*Character));
				const SHORT Translation = VkKeyScanExW(Glyph, Layout);
				if (Translation == -1 || (Translation & 0xff00)) return Reject(TEXT("Keyboard glyph requires an unsupported modifier chord"));
				VK = uint32(Translation & 0xff);
			}
			const uint32 Scan = VK ? MapVirtualKeyExW(VK, MAPVK_VK_TO_VSC_EX, Layout) : 0;
			Binding.Control = {ERawControlKind::KeyboardUsage, Windows::KeyboardUsage(Scan)};
			// These VKs share historical scan bytes; retain their distinct HID usages.
			if (VK == VK_PAUSE) Binding.Control.Code = 0x48;
			if (VK == VK_NUMLOCK) Binding.Control.Code = 0x53;
			if (!Binding.Control.IsValid()) return Reject(TEXT("Unsupported keyboard key/layout translation"));
#else
			return Reject(TEXT("No keyboard mapping adapter for this platform"));
#endif
		}
		Result.Mapping.push_back(std::move(Binding));
	}
	for (const auto& Definition : Result.Actions)
		if (Definition.Wiring == EActionWiring::Wired && !Seen.Contains(Definition.Id))
			return Reject(TEXT("A required game action has no binding"));
	if (!FInputActionContract::Create(Result)) return Reject(TEXT("Imported mapping exceeds supported response or capacity"));
	Description = std::move(Result); Error.Reset(); return true;
}
}
