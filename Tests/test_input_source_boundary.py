"""Source guards complement the native core probe; they do not compile Unreal."""
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1] / "Source" / "IAmSpeed"


def body(path, signature):
    source = (ROOT / path).read_text(encoding="utf-8-sig")
    start = source.index("{", source.index(signature))
    depth = 1
    end = start + 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start:end]


class InputSourceBoundary(unittest.TestCase):
    def test_consumer_has_no_unreal_action_or_controller_access(self):
        consumer = body("Components/SpeedWheeledComponent.cpp", "bool USpeedWheeledComponent::ConsumeProducedWheeledInputs")
        for forbidden in ("FInputActionValue", "GetActionValue", "GetController", "GFrameCounter", "GetWorld", "Value.Get<"):
            self.assertNotIn(forbidden, consumer)
        self.assertIn("Stream->Consume(CanonicalFrame)", consumer)

    def test_legacy_conversion_and_exclusive_route(self):
        update = body("Components/SpeedWheeledComponent.cpp", "void USpeedWheeledComponent::UpdateInputs()")
        self.assertIn("FromLegacyLocalFrame(NumFrame())", update)
        self.assertIn("if (!bProduced)", update)
        self.assertIn("if (!bProduced || IsTestInputOverrideEnabled())", update)
        header = (ROOT / "Controllers/SpeedController.h").read_text()
        self.assertIn("InputProducer = nullptr;", header)
        configure = body("Controllers/SpeedController.cpp", "bool ASpeedController::ConfigureInputProducer(")
        self.assertIn("if (SpeedCar ||", configure)
        self.assertIn("FPresentationInputScope::IsActive()", configure)

    def test_no_gt_acquisition_for_new_device_producer(self):
        controller = (ROOT / "Controllers/SpeedController.cpp").read_text()
        for forbidden in ("FDeviceInputProducer", "PublishDevice", "GFrameCounter", "SetAction(", "->Produce("):
            self.assertNotIn(forbidden, controller)
        for name in ("Throttle", "StartBrake", "Brake", "StopBrake", "Steering"):
            handler = body("Controllers/SpeedController.cpp", f"void ASpeedController::{name}(")
            self.assertIn("if (InputSnapshots) return;", handler)
            for forbidden in ("InputProducer", "Publish", "SetAction", "GFrameCounter"):
                self.assertNotIn(forbidden, handler)
        for path in ("Input/InputFrame.h", "Input/InputProducer.h", "Input/DeviceInputSession.h"):
            core = (ROOT / path).read_text()
            for forbidden in ("FInputActionValue", "EnhancedInput", "PlayerController", "GFrameCounter", "IsInGameThread", "CoreMinimal.h"):
                self.assertNotIn(forbidden, core)

    def test_gt_observes_latest_only(self):
        tick = body("Controllers/SpeedController.cpp", "void ASpeedController::Tick(")
        self.assertIn("ReadLatest()", tick)
        for forbidden in ("Consume(", "Produce(", "ReadRecorded(", "SetPhys", "PublishDevice", "while", "for ("):
            self.assertNotIn(forbidden, tick)
        handler = body("Controllers/SpeedController.cpp", "void ASpeedController::HandleInputs(")
        self.assertIn("PresentationBindings.HandleInputs(Snapshot)", handler)
        self.assertNotIn("SpeedCar", handler)

    def test_publication_follows_successful_physical_snapshot(self):
        source = (ROOT / "World/Simulation/SpeedSimulation.cpp").read_text()
        publish = source.index("if (!SnapshotBuffer.Publish(Snapshot))")
        notify = source.index("NotifyCanonicalFramePublished(Context.NumFrame)")
        self.assertLess(publish, notify)
        self.assertIn("return false;", source[publish:notify])
        consume = body("Input/InputStream.h", "std::optional<FInputFrame> Consume(")
        self.assertNotIn("Latest =", consume)

    def test_component_writer_guards_precede_mutation(self):
        for name in ("SetPhysThrottleInput", "SetPhysBrakeInput", "SetPhysSteeringInput", "QueueWheeledInputCommand", "SetTestInputOverrideEnabled"):
            writer = body("Components/SpeedWheeledComponent.cpp", f"void USpeedWheeledComponent::{name}(")
            self.assertIn("if (Speed::Input::FPresentationInputScope::IsActive()) return;", writer)
            prefix = writer[:writer.index("FPresentationInputScope::IsActive")]
            self.assertNotIn(".store(", prefix)
            self.assertNotIn("FScopeLock", prefix)

    def test_teardown_bypasses_presentation_guard_and_disables_old_stream(self):
        setter = body("Components/SpeedWheeledComponent.cpp", "void USpeedWheeledComponent::SetFrameInputStream(")
        self.assertIn("if (Stream && Speed::Input::FPresentationInputScope::IsActive()) return;", setter)
        self.assertLess(setter.index("FrameInputStream->Deactivate()"), setter.index("FrameInputStream = MoveTemp(Stream)"))
        detach = body("Controllers/SpeedController.cpp", "void ASpeedController::OnUnPossess(")
        self.assertLess(detach.index("SetFrameInputStream(nullptr)"), detach.index("InputSnapshots.reset()"))

    def test_override_observes_skip_failure(self):
        consumer = body("Components/SpeedWheeledComponent.cpp", "bool USpeedWheeledComponent::ConsumeProducedWheeledInputs")
        self.assertIn("const bool bSkipped = Stream->Skip(CanonicalFrame)", consumer)
        self.assertIn("ensureMsgf(bSkipped", consumer)

    def test_worker_owns_no_controller_reference(self):
        stream = (ROOT / "Input/InputStream.h").read_text()
        self.assertNotIn("APlayerController", stream)
        self.assertNotIn("UObject*", stream)
        self.assertIn("std::shared_ptr<IInputProducer>", stream)
        self.assertIn("std::lock_guard<std::mutex>", stream)
        controller = (ROOT / "Controllers/SpeedController.cpp").read_text()
        self.assertIn("SpeedCar->SetFrameInputStream(nullptr)", controller)
        self.assertIn("InputSnapshots.reset()", controller)


if __name__ == "__main__":
    unittest.main()
