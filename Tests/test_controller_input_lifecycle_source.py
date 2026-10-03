"""L2 source guardrails only; no compilation/runtime claim."""
from pathlib import Path
import re
import unittest

ROOT = Path(__file__).parents[1] / 'Source/IAmSpeed'
CONTROLLER = (ROOT / 'Controllers/SpeedController.cpp').read_text()
STREAM = (ROOT / 'Input/InputStream.h').read_text()
OWNER = (ROOT / 'World/Simulation/SpeedSimulation.cpp').read_text()
TEST = (ROOT / 'World/Simulation/ControllerInputLifecycleTests.cpp').read_text()


def validate(controller, stream, owner, test):
    pause = controller.split('bool ASpeedController::SetPause(', 1)[1].split('void ASpeedController::SetStandaloneSimulationPaused', 1)[0]
    if 'if (bPause || bInputLifecycleFault)' not in pause:
        raise ValueError('Fault recovery requires renewed owner acknowledgement')
    owned = pause.split('if (bPause || bInputLifecycleFault)', 1)[1]
    if not owned.index('return false; // Pending pause remains') < owned.index('ApplyInputLifecyclePause(true)') < owned.index('Super::SetPause'):
        raise ValueError('Quiescence and source pause must precede Unreal pause')
    if 'if (bPause || bInputLifecycleFault)' not in pause or '!bChanged && bPause && !IsPaused()' not in owned:
        raise ValueError('Fault recovery requires a new acknowledgement and cannot resume an already paused world')
    resume = owned.split('if ((bChanged && !bPause)', 1)[1]
    if resume.index('ApplyInputLifecyclePause(false)') > resume.index('SetStandaloneSimulationPaused(false)'):
        raise ValueError('Source must resume before the worker')
    lifecycle = controller.split('bool ASpeedController::ApplyInputLifecyclePause', 1)[1]
    if re.search(r'(WheeledUserInput|->(?:Produce|SetThrottleInput|SetBrakeInput|SetSteeringInput)\s*\()', lifecycle):
        raise ValueError('GT lifecycle cannot acquire or write physical input')
    if 'if (!Active || LifecyclePaused) return std::nullopt;' not in stream or 'HistoryEpoch[Frame % HistoryCapacity] != ControlEpoch' not in stream:
        raise ValueError('Paused acquisition and obsolete publication must be gated')
    boundary = owner.split('ESimulationQuiescence ASpeedSimulation::TryPauseOwnedSimulation', 1)[1].split('void ASpeedSimulation::PauseOwnedSimulation', 1)[0]
    if 'SimulationWorker->TryPause(TimeoutMilliseconds)' not in boundary or 'ESimulationQuiescence::AlreadyStopped' not in boundary:
        raise ValueError('Explicit quiescence results required')
    for witness in ('Controller->SetPause(true)', 'Controller->SetPause(false)', 'Controller->UnPossess()', 'Controller->EndPlay(', 'BlockEntered->Wait(1000)', 'Source->RejectNextControl = true;'):
        if witness not in test:
            raise ValueError('Missing controller lifecycle scenario')
    if 'TestInputProducer' in test:
        raise ValueError('Real device session required')


class ControllerLifecycleGuards(unittest.TestCase):
    def test_structure(self):
        validate(CONTROLLER, STREAM, OWNER, TEST)

    def test_reject_gt_poll(self):
        with self.assertRaises(ValueError):
            validate(CONTROLLER + '\nSource->Produce(0);', STREAM, OWNER, TEST)

    def test_reject_physical_writer(self):
        with self.assertRaises(ValueError):
            validate(CONTROLLER + '\nComponent->SetThrottleInput(0);', STREAM, OWNER, TEST)

    def test_reject_unbounded_pause(self):
        with self.assertRaises(ValueError):
            validate(CONTROLLER, STREAM, OWNER.replace('SimulationWorker->TryPause(TimeoutMilliseconds)', 'SimulationWorker->Pause()'), TEST)

    def test_reject_stale_publication(self):
        with self.assertRaises(ValueError):
            validate(CONTROLLER, STREAM.replace('HistoryEpoch[Frame % HistoryCapacity] != ControlEpoch', 'false'), OWNER, TEST)

    def test_reject_missing_timeout_scenario(self):
        with self.assertRaises(ValueError):
            validate(CONTROLLER, STREAM, OWNER, TEST.replace('BlockEntered->Wait(1000)', 'true'))

    def test_reject_resume_without_new_ack(self):
        with self.assertRaises(ValueError):
            validate(CONTROLLER.replace('if (bPause || bInputLifecycleFault)', 'if (bPause)'), STREAM, OWNER, TEST)

    def test_reject_missing_teardown(self):
        with self.assertRaises(ValueError):
            validate(CONTROLLER, STREAM, OWNER, TEST.replace('Controller->EndPlay(', 'Unused('))


if __name__ == '__main__':
    unittest.main()
