#pragma once

#include "CoreMinimal.h"
#include "GameFramework/PlayerController.h"
#include "IAmSpeed/Input/InputStream.h"
#include "IAmSpeed/Input/InputHostSession.h"
#include "IAmSpeed/Input/ControlApplicationJournal.h"
#include <memory>
#include "SpeedController.generated.h"

class ASpeedCar;
enum class ESimulationQuiescence : uint8;
class UEnhancedInputComponent;
class UInputAction;
class UInputMappingContext;
struct FInputActionValue;

/**
 * Generic player controller for an IAmSpeed car.
 *
 * The base class owns the common ground-driving inputs. Games can specialize
 * presentation and input filtering through the protected virtual hooks.
 */
UCLASS()
class IAMSPEED_API ASpeedController : public APlayerController
{
	GENERATED_BODY()

public:
	ASpeedController();
	/** Pause, retire the old epoch and queue a fresh immutable mapping session. */
	bool RestartInputSessionAtBoundary();
#if !UE_BUILD_SHIPPING
	/** Explicit test harness takes over through a separately bound exact producer. */
	bool UseScenarioInputAuthorityAtBoundary();
#endif

	bool ConfigureInputSessionV2(std::shared_ptr<Speed::Input::V2::FInputHostSession> Session);
	Speed::Input::V2::FControlApplicationBatch ReadControlReceipts(uint64 Cursor) const
	{ check(IsInGameThread()); return ControlReceiptsV2.Read(Cursor); }
	/** Register before possession. UObject lifetime remains owned by Unreal. */
	template<class T>
	bool BindAction(const FString& Name, Speed::Input::FActionId Action,
		Speed::Input::V2::EStateAction State, T* Receiver,
		void (T::*Method)(const Speed::Input::V2::FActionEvent&))
	{
		if (!IsInGameThread() || !InputSessionV2 || !Receiver || !Method) return false;
		struct FReceiver
		{
			TWeakObjectPtr<T> Target;
			void (T::*Callback)(const Speed::Input::V2::FActionEvent&);
			void Dispatch(const Speed::Input::V2::FActionEvent& Event)
			{ if (T* Object = Target.Get()) (Object->*Callback)(Event); }
		};
		auto Adapter = std::make_shared<FReceiver>(FReceiver{Receiver, Method});
		if (!InputSessionV2->Presentation->BindAction(TCHAR_TO_UTF8(*Name), Action, State,
			std::weak_ptr<FReceiver>(Adapter), &FReceiver::Dispatch)) return false;
		InputReceiversV2.push_back(std::move(Adapter)); return true;
	}
	void Tick(float DeltaSeconds) override;
	void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	/** Presentation only: callback receives const values, never a physical writer.
	 * Name is explicitly associated with a slot; unknown slots/duplicate names fail. */
	bool BindAction(const FString& Name, Speed::Input::FActionId Action,
		Speed::Input::FInputPresentationBindings::FCallback Callback);
	/** Optional values-only source installation before possession. This is a
	 * lifecycle operation, never device acquisition or a GameThread input feed.
	 * No source is installed by default; the real device backend is unresolved. */
	bool ConfigureInputProducer(std::shared_ptr<Speed::Input::IInputProducer> Producer);
	/** Binds the common IAmSpeed driving actions to this controller. */
	virtual void SetupEnhancedInputComponent(UEnhancedInputComponent* EnhancedInputComponent);

	/** Applies forward throttle in the normalized [0, 1] range. */
	void Throttle(const FInputActionValue& Value);
	/** Announces the beginning of braking to presentation specializations. */
	void StartBrake(const FInputActionValue& Value);
	/** Applies braking/reverse input in the normalized [0, 1] range. */
	void Brake(const FInputActionValue& Value);
	/** Clears braking and announces its end to presentation specializations. */
	void StopBrake(const FInputActionValue& Value);
	/** Applies signed steering input in the normalized [-1, 1] range. */
	void Steering(const FInputActionValue& Value);
	/** Handles the bound pause action; games may specialize their pause presentation. */
	virtual void PauseInput(const FInputActionValue& Value);
	virtual void StartBackCamera(const FInputActionValue& Value);
	virtual void CompleteBackCamera(const FInputActionValue& Value);
	virtual void CamYaw(const FInputActionValue& Value);
	virtual void CamPitch(const FInputActionValue& Value);
	void CompleteCamYaw(const FInputActionValue& Value);
	void CompleteCamPitch(const FInputActionValue& Value);

	/**
	 * Toggles Unreal pause normally. In standalone mode, SetPause also suspends
	 * or resumes the separately-owned IAmSpeed simulation worker.
	 */
	void Pause() override;
	bool SetPause(bool bPause, FCanUnpause CanUnpauseDelegate = FCanUnpause()) override;

protected:
	virtual bool RequiresInputSessionV2() const { return false; }
	virtual std::shared_ptr<Speed::Input::V2::FInputHostSession> CreateInputSessionV2(uint64 FirstFrame) { return {}; }
	virtual bool BindInputPresentationV2() { return true; }
	bool RefreshInputSessionV2();
	bool HasInputSessionV2() const { return InputSessionV2 != nullptr; }
	virtual Speed::Input::V2::EControlApplication ExecuteInputControlV2(const Speed::Input::V2::FControlRequest& Request);
	virtual void HandleInputs();
	void HandleInputs(const Speed::Input::FPublishedInputFrame& Snapshot);
	void SetupInputComponent() override;
	void OnPossess(APawn* InPawn) override;
	void OnUnPossess() override;

	/** Lets games apply their user-configured steering response or deadzone. */
	virtual float FilterSteeringInput(float SteeringInput) const;
	/** Lets games mirror brake state into presentation such as brake lights. */
	virtual void OnBrakeInputChanged(bool bBraking);
	/** Presentation hook called after Unreal and the owned simulation agree on pause. */
	virtual void OnPauseStateChanged(bool bPaused) {}
	/** Aligns the owned standalone simulation with Unreal's current pause state. */
	void SynchronizeOwnedSimulationPauseWithWorld();

	/** Input mapping context shared by the generic and game-specific actions. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = Input)
	TObjectPtr<UInputMappingContext> InputMappingContext = nullptr;

	/** IAmSpeed car currently controlled by this player controller. */
	UPROPERTY(VisibleInstanceOnly, BlueprintReadOnly, Category = Input)
	TObjectPtr<ASpeedCar> SpeedCar = nullptr;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = Input)
	TObjectPtr<UInputAction> SteeringAction = nullptr;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = Input)
	TObjectPtr<UInputAction> ThrottleAction = nullptr;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = Input)
	TObjectPtr<UInputAction> BrakeAction = nullptr;

	/** Exposed here so derived controllers can bind their own pause UI action. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = Input)
	TObjectPtr<UInputAction> PauseAction = nullptr;

	// Keep these reflected names when moving saved game-specific action assets
	// to the common controller; the six bindings exist only in this base.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = Input)
	UInputAction* StartBackCameraAction = nullptr;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = Input)
	UInputAction* CamYawAction = nullptr;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = Input)
	UInputAction* CamPitchAction = nullptr;

private:
	std::shared_ptr<Speed::Input::V2::FInputHostSession> InputSessionV2;
	std::vector<std::shared_ptr<void>> InputReceiversV2;
	uint64 LastInputSessionV2 = 0;
	Speed::Input::V2::FControlApplicationJournal ControlReceiptsV2;
	bool BeginRegistryInputSession();
	bool QueueRegistryInputCommand(Speed::Input::V2::EBoundaryOperation Operation);
	bool ServiceRegistryInputSession();
	bool ReleaseRegistryInputSession();
	bool ServiceInputSessionV2();
	bool ReleaseInputSessionV2();
	bool SetInputPauseV2(bool bPause, FCanUnpause CanUnpauseDelegate);
#if defined(WITH_DEV_AUTOMATION_TESTS) && WITH_DEV_AUTOMATION_TESTS
	friend struct Speed::Input::FControllerInputTestAccess;
	friend class FIAmSpeedControllerInputLifecycleTest;
#endif
	std::shared_ptr<Speed::Input::IInputProducer> InputProducer = nullptr;
	std::shared_ptr<Speed::Input::FInputStream> InputSnapshots;
	Speed::Input::FInputPresentationBindings PresentationBindings;
	/** Updates the worker owned by the authoritative IAmSpeed game mode. */
	void SetStandaloneSimulationPaused(bool bPaused);
	ESimulationQuiescence QuiesceStandaloneInputOwner();
	bool ApplyInputLifecyclePause(bool bPaused);
	bool ReleaseInputLifecycle();
	bool bScenarioOwnsInputAuthority = false;
	bool bInputSessionRequiredV2 = false;
	bool bInputSessionPendingV2 = false;
	bool bInputLifecycleFault = false;
};
