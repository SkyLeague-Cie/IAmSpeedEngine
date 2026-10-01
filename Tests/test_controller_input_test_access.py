"""Read-only test access source contract. Unreal compilation is a separate gate."""
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1] / "Source/IAmSpeed"


class AccessBoundary(unittest.TestCase):
    def test_only_automation_and_read_only(self):
        text = (ROOT / "Input/Testing/ControllerInputTestAccess.h").read_text()
        self.assertLess(text.index("#if defined(WITH_DEV_AUTOMATION_TESTS) && WITH_DEV_AUTOMATION_TESTS"), text.index("#include"))
        self.assertIn("std::weak_ptr<const FInputStream>", text)
        self.assertIn("std::lock_guard<std::mutex>", text)
        for forbidden in ("Consume(", "PublishCompleted(", "Skip(", "Deactivate(",
                          "shared_ptr<FInputStream>", "Stream->Active ="):
            self.assertNotIn(forbidden, text)

    def test_no_component_friend(self):
        self.assertNotIn("FControllerInputTestAccess",
                         (ROOT / "Components/SpeedWheeledComponent.h").read_text())
        for path in ("Controllers/SpeedController.h", "Input/InputStream.h"):
            text = (ROOT / path).read_text()
            self.assertIn("#if defined(WITH_DEV_AUTOMATION_TESTS) && WITH_DEV_AUTOMATION_TESTS\n", text)
            self.assertIn("friend struct", text)


if __name__ == "__main__":
    unittest.main()
