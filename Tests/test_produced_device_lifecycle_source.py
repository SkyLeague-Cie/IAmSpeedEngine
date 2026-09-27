"""Structural L1 guardrails; not compilation or runtime evidence."""
from pathlib import Path
import re
import unittest

SOURCE = (Path(__file__).parents[1] / 'Source/IAmSpeed/World/Simulation/ProducedDeviceLifecycleTests.cpp').read_text()


def validate(source):
    if 'TestInputProducer' in source or 'CompileWheeledTestProfile' in source:
        raise ValueError('Must use the real device session')
    if 'std::make_shared<FDeviceInputSession>(74, Digital)' not in source:
        raise ValueError('Missing real session')
    if re.search(r'->(?:UpdateInputs|OnCanonicalFramePublished|SetThrottleInput|SetBrakeInput|SetSteeringInput|SetImmediateMovementForTesting)\s*\(', source):
        raise ValueError('Physical bypass')
    worker = source.split('FSimulationWorker Worker([&]', 1)[1].split('}, [](FSimulationWorkerWaitContext&)', 1)[0]
    if worker.count('Driver->RunCanonicalFrames(1)') != 1 or re.search(r'\bTest(?:True|False|Equal)\s*\(', worker):
        raise ValueError('Real driver and post-join assertions required')
    observer = source.split('auto Counted =', 1)[1].split('Stream = std::make_shared<FInputStream>', 1)[0]
    if 'Stream->' in observer:
        raise ValueError('No stream reentry while Produce holds its mutex')
    join = source.index('Worker.StopAndJoin(); // No test assertion')
    detach = source.index('Component->SetFrameInputStream(nullptr); // Worker already joined')
    consume = source.index('Stream->Consume(FrameCount).has_value()')
    publish = source.index('Stream->PublishCompleted(FrameCount - 1)')
    count = source.index('TEXT("stop and detach cause no producer polling")')
    if not join < detach < consume < publish < count:
        raise ValueError('Retained stream attempts must be checked after join and detach')


class LifecycleSourceGuards(unittest.TestCase):
    def test_structure(self):
        validate(SOURCE)

    def test_reject_test_producer(self):
        with self.assertRaises(ValueError):
            validate(SOURCE.replace('FDeviceInputSession', 'FTestInputProducer'))

    def test_reject_direct_setter(self):
        with self.assertRaises(ValueError):
            validate(SOURCE + '\nComponent->SetThrottleInput(1);')

    def test_reject_stream_reentry(self):
        with self.assertRaises(ValueError):
            validate(SOURCE.replace('Witnesses.Add({EDeviceLifecycleWitness::Produce', 'Stream->ReadLatest(); Witnesses.Add({EDeviceLifecycleWitness::Produce'))

    def test_reject_worker_assertion(self):
        with self.assertRaises(ValueError):
            validate(SOURCE.replace('bool LifecycleValid = true;', 'TestTrue(TEXT("bad"), true); bool LifecycleValid = true;'))

    def test_reject_missing_detach(self):
        with self.assertRaises(ValueError):
            validate(SOURCE.replace('Component->SetFrameInputStream(nullptr); // Worker already joined', '// Worker already joined'))


if __name__ == '__main__':
    unittest.main()
