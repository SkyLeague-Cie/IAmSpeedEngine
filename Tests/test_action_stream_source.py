"""Common portable consumption and one-way publication/dispatch boundaries."""
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1] / "Source/IAmSpeed/Input"


class StreamSource(unittest.TestCase):
    def test_common_source_and_sink(self):
        stream = (ROOT / "InputStreamV2.h").read_text(encoding="utf-8-sig")
        self.assertEqual(stream.count("AssembleDrivingTargets("), 1)
        self.assertIn("Source->Produce(Frame)", stream)
        self.assertIn("!Slot->Committed", stream)
        self.assertIn("EConsumeStatus::AlreadyPending, {}, {}", stream)
        self.assertIn("EConsumeStatus::PublishedReplay", stream)
        self.assertIn("try { Input = Source->Produce(Frame); }", stream)
        self.assertIn("catch (...) { return Fault(); }", stream)
        self.assertIn("FTestInputProducer final : public IInputProducer",
                      (ROOT / "Testing/TestInputProducerV2.h").read_text())
        self.assertIn("FDeviceInputProducer final : public IInputProducer",
                      (ROOT / "InputProducerV2.h").read_text())

    def test_no_presentation_receiver_on_worker(self):
        for name in ("InputProducerV2.h", "InputStreamV2.h", "PhysicalActionSink.h"):
            text = (ROOT / name).read_text(encoding="utf-8-sig")
            for forbidden in ("std::function", "UObject", "FKey", "EnhancedInput", "SpeedController", "ActionDispatch.h"):
                self.assertNotIn(forbidden, text)

    def test_presentation_reads_only_published_batches(self):
        text = (ROOT / "ActionDispatch.h").read_text(encoding="utf-8-sig")
        self.assertIn("ReadPublishedSince(Cursor)", text)
        self.assertNotIn("ReadRecorded(", text)
        self.assertNotIn("->Consume(", text)
        self.assertIn("const auto Snapshot = Bindings", text)
        self.assertIn("FPresentationInputScope ReadOnly", text)
        self.assertIn("return BindSnapshotAction(std::move(Name), Action, std::move(Callback))", text)


if __name__ == "__main__":
    unittest.main()
