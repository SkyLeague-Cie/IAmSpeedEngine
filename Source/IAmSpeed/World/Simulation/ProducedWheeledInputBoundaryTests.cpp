#if WITH_DEV_AUTOMATION_TESTS

#include "IAmSpeed/Components/SpeedWheeledComponent.h"
#include "IAmSpeed/Input/InputStream.h"
#include "IAmSpeed/Input/Testing/WheeledTestProfile.h"
#include "Misc/AutomationTest.h"
#include "UObject/StrongObjectPtr.h"

namespace
{
// Instrumentation forwards unchanged to the real sealed producer. It neither
// manufactures input nor substitutes for the component's physical consumer.
class FCountedWheeledProfileProducer final : public Speed::Input::IInputProducer
{
public:
	explicit FCountedWheeledProfileProducer(std::unique_ptr<Speed::Input::FTestInputProducer> InSource)
		: Source(std::move(InSource)) {}
	std::optional<Speed::Input::FInputFrame> Produce(Speed::Input::FFrameNumber Frame) override
	{
		++Calls;
		return Source->Produce(Frame);
	}
	uint32 Calls = 0;
private:
	std::unique_ptr<Speed::Input::FTestInputProducer> Source;
};

struct FExpectedWheeledAxes
{
	int32 Throttle, Brake, Steering;
};
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FIAmSpeedProducedWheeledInputBoundaryTest,
	"IAmSpeed.Simulation.ProducedWheeledInputBoundary",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedProducedWheeledInputBoundaryTest::RunTest(const FString& Parameters)
{
	using namespace Speed::Input;
	using namespace Speed::Input::Testing;
	// Independent literal oracles: scenario keys 0/2/4/6 affect C1/3/5/7.
	// Local component frame is C+1; C0 is the neutral attachment/reset frame.
	const std::array<FExpectedWheeledAxes, 11> Targets = {{
		{0, 0, 0}, {255, 0, -127}, {255, 0, -127}, {128, 128, -63},
		{128, 128, -63}, {0, 255, 64}, {0, 255, 64}, {0, 0, 0},
		{0, 0, 0}, {0, 0, 0}, {0, 0, 0}
	}};
	const auto Physical = Targets; // Exact Test fixture: no implicit component slew.

	for (const uint32 ObservationStride : {1u, 3u, 7u})
	{
		FWheeledTestProfile Profile;
		Profile.Identity = {EProducerKind::Device, 71};
		Profile.FrameCount = Targets.size();
		Profile.Keys = {{0, 1.f, 0.f, -1.f}, {2, .5f, .5f, -.5f},
			{4, 0.f, 1.f, .5f}, {6, 0.f, 0.f, 0.f}};
		auto Compiled = CompileWheeledTestProfile(Profile);
		if (!TestTrue(TEXT("W1 profile compiles"), Compiled.Error == EWheeledProfileError::None && bool(Compiled.Producer)))
			return false;
		auto Counted = std::make_shared<FCountedWheeledProfileProducer>(std::move(Compiled.Producer));
		auto Stream = std::make_shared<FInputStream>(Counted);
		TStrongObjectPtr<USpeedWheeledComponent> Owner(NewObject<USpeedWheeledComponent>());
		USpeedWheeledComponent* Component = Owner.Get();
		TestFalse(TEXT("legacy scripted override stays disabled"), Component->IsTestInputOverrideEnabled());
		Component->SetFrameInputStream(Stream);
		TestFalse(TEXT("attachment does not publish"), Stream->ReadLatest().has_value());

		auto CheckAxes = [this](const FString& Label, const FWheeledInputState& Actual,
			const FExpectedWheeledAxes& Expected)
		{
			TestEqual(Label + TEXT(" throttle"), int32(Actual.Throttle), Expected.Throttle);
			TestEqual(Label + TEXT(" brake"), int32(Actual.Brake), Expected.Brake);
			TestEqual(Label + TEXT(" steering"), int32(Actual.Steer), Expected.Steering);
		};
		for (uint32 C = 0; C < Targets.size(); ++C)
		{
			Component->BaseGameState.NumFrame = C + 1;
			// A competing remote packet must never displace the sealed Test owner.
			if (C == 1)
			{
				FWheeledInputState Poison;
				Poison.Throttle = 211; Poison.Brake = 213; Poison.Steer = 101;
				TestTrue(TEXT("remote poison rejected while Test producer owns input"),
					Component->SubmitLegacyWheeledInput(2,2,Poison) != ELegacyRemoteAdmission::Accepted);
			}
			Component->UpdateInputs();
			const FString Label = FString::Printf(TEXT("stride=%u C=%u"), ObservationStride, C);
			CheckAxes(Label + TEXT(" exact Test input"), Component->WheeledPhysicalInput, Targets[C]);
			CheckAxes(Label + TEXT(" physical"), Component->WheeledPhysicalInput, Physical[C]);
			TestEqual(Label + TEXT(" one producer call"), Counted->Calls, C + 1);
			TestTrue(Label + TEXT(" consumed stream latched"), Component->ConsumedFrameInputStream == Stream);
			const auto BeforePublication = Stream->ReadLatest();
			TestTrue(Label + TEXT(" consumption is not publication"), C == 0
				? !BeforePublication : BeforePublication && BeforePublication->Frame.GetConsumptionFrame() == C - 1);

			Component->UpdateInputs();
			CheckAxes(Label + TEXT(" repeat exact Test input"), Component->WheeledPhysicalInput, Targets[C]);
			CheckAxes(Label + TEXT(" repeat physical"), Component->WheeledPhysicalInput, Physical[C]);
			TestEqual(Label + TEXT(" repeat uses history"), Counted->Calls, C + 1);
			const auto Recorded = Stream->ReadRecorded(C);
			TestTrue(Label + TEXT(" immutable frame recorded"), Recorded.has_value());
			if (Recorded)
			{
				TestTrue(Label + TEXT(" identity and clocks"), Recorded->GetProducer().Id == 71
					&& Recorded->GetProducer().Kind == EProducerKind::Device
					&& Recorded->GetSourceFrame() == C && Recorded->GetConsumptionFrame() == C);
				TestTrue(Label + TEXT(" reset only C0"), Recorded->RequiresReset() == (C == 0));
				TestEqual(Label + TEXT(" no synthetic action edges"), int32(Recorded->GetEdgeCount()), 0);
				TestEqual(Label + TEXT(" recorded throttle"), int32(Recorded->GetActions()[Throttle]), Targets[C].Throttle);
				TestEqual(Label + TEXT(" recorded brake"), int32(Recorded->GetActions()[Brake]), Targets[C].Brake);
				TestEqual(Label + TEXT(" recorded steering"), int32(Recorded->GetActions()[Steering]), Targets[C].Steering);
			}
			// Invoke the real publication callback after this boundary step. This
			// is not a claim that an entire simulation transaction has completed.
			Component->OnCanonicalFramePublished(C);
			Component->OnCanonicalFramePublished(C);
			const auto Published = Stream->ReadLatest();
			TestTrue(Label + TEXT(" completed frame published once"), Published
				&& Published->Frame.GetConsumptionFrame() == C && Published->Serial == C + 1);
			if (C % ObservationStride == 0)
			{
				for (uint32 Read = 0; Read < 3; ++Read)
				{
					FPresentationInputScope PresentationScope;
					const auto Latest = Stream->ReadLatest();
					TestTrue(Label + TEXT(" presentation only observes"), Latest && Recorded
						&& Latest->Serial == C + 1 && Latest->Frame.GetActions() == Recorded->GetActions());
				}
				CheckAxes(Label + TEXT(" observation leaves physics unchanged"), Component->WheeledPhysicalInput, Physical[C]);
				TestEqual(Label + TEXT(" observation never polls producer"), Counted->Calls, C + 1);
			}
		}
		const auto ResetHistory = Stream->ReadRecorded(0);
		TestTrue(TEXT("later frames preserve neutral C0 history"), ResetHistory
			&& ResetHistory->RequiresReset() && ResetHistory->GetActions()[Throttle] == 0);
		Component->SetFrameInputStream(nullptr);
		TestFalse(TEXT("detached stream cannot consume even recorded frames"), Stream->Consume(10).has_value());
		TestFalse(TEXT("detached stream cannot publish"), Stream->PublishCompleted(10));
		TestTrue(TEXT("component stream detached"), !Component->FrameInputStream);
	}
	return true;
}

#endif
