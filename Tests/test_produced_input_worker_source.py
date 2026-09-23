"""Structural guardrails only: these do not compile or execute the UE fixture."""
from pathlib import Path
import re
import unittest

SOURCE = (Path(__file__).parents[1] / 'Source/IAmSpeed/World/Simulation/ProducedInputWorkerOrderTests.cpp').read_text()


def validate(source):
    forbidden = r'->(?:UpdateInputs|OnCanonicalFramePublished|PublishCompleted|SetThrottleInput|SetBrakeInput|SetSteeringInput|SetImmediateMovementForTesting)\s*\('
    if re.search(forbidden, source):
        raise ValueError('Manual input consumption/publication/setter bypass')
    worker = source.split('FSimulationWorker Worker([&]', 1)[1].split('}, [](FSimulationWorkerWaitContext&)', 1)[0]
    if worker.count('Driver->RunCanonicalFrames(1)') != 1:
        raise ValueError('Must use the production canonical driver')
    if re.search(r'\bTest(?:True|False|Equal|NotNull)\s*\(', worker):
        raise ValueError('Automation assertions must not run on worker')
    acquisition = source.split('auto Counted =', 1)[1].split('Stream = std::make_shared<FInputStream>', 1)[0]
    if 'Stream->' in acquisition:
        raise ValueError('Producer observer must not re-enter locked stream')
    if source.index('Driver->RegisterPresentationProducer') > source.index('Worker.Start()'):
        raise ValueError('Register observer before worker start')
    if source.index('Worker.StopAndJoin(); // No test assertion') > source.index('TestTrue(TEXT("finite run completed'):
        raise ValueError('Join before inspecting worker witnesses')
    capture = 'const uint16 InitialCountdown = Component->GetMinNbFramesBeforeCanMove();'
    if capture not in source or source.index(capture) > source.index('Worker.Start()'):
        raise ValueError('Capture configured countdown before worker')
    if 'InitialCountdown > 0' not in source:
        raise ValueError('Configured countdown must be positive')
    for stage in ('Before', 'After'):
        if f'int32(Result.Countdown{stage}), int32(InitialCountdown)' not in source:
            raise ValueError('Countdown must retain captured configured value')
    if re.search(r'WheeledPhysicsState\.nbFramesbeforeCanMove\s*=', source):
        raise ValueError('Do not force countdown state')


class SourceGuardTests(unittest.TestCase):
    def test_real_fixture_structure(self):
        validate(SOURCE)

    def test_reject_direct_consumption(self):
        with self.assertRaises(ValueError): validate(SOURCE + '\nComponent->UpdateInputs();')

    def test_reject_manual_publication(self):
        with self.assertRaises(ValueError): validate(SOURCE + '\nComponent->OnCanonicalFramePublished(0);')

    def test_reject_axis_setter(self):
        with self.assertRaises(ValueError): validate(SOURCE + '\nComponent->SetThrottleInput(1);')

    def test_reject_reentrant_observer(self):
        with self.assertRaises(ValueError):
            validate(SOURCE.replace('Witnesses.Add({EInputWitness::Produce', 'Stream->ReadLatest(); Witnesses.Add({EInputWitness::Produce'))

    def test_reject_worker_assertion(self):
        with self.assertRaises(ValueError):
            validate(SOURCE.replace('const uint64 C = Driver->CanonicalNumFrame;', 'TestTrue(TEXT("bad"), true); const uint64 C = Driver->CanonicalNumFrame;'))

    def test_reject_struct_default_countdown(self):
        with self.assertRaises(ValueError):
            validate(SOURCE.replace('Component->GetMinNbFramesBeforeCanMove()', 'uint16(1)'))

    def test_reject_forced_countdown(self):
        with self.assertRaises(ValueError):
            validate(SOURCE + '\nComponent->WheeledPhysicsState.nbFramesbeforeCanMove = 1;')

    def test_reject_changed_countdown_oracle(self):
        with self.assertRaises(ValueError):
            validate(SOURCE.replace('int32(Result.CountdownAfter), int32(InitialCountdown)', 'int32(Result.CountdownAfter), int32(1)'))


if __name__ == '__main__':
    unittest.main()
