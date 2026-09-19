"""Portable mapping boundaries; no runtime/platform qualification."""
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1] / "Source/IAmSpeed/Input"


class MappingSource(unittest.TestCase):
    def test_response_uses_explicit_supported_operations(self):
        text = (ROOT / "ActionMapping.h").read_text(encoding="utf-8-sig")
        self.assertNotIn("std::pow", text)
        self.assertIn("A.Exponent != 1.0f && A.Exponent != 2.0f", text)
        self.assertIn("Remapped * Remapped", text)
        self.assertIn("!SupportsContract(*Candidate)", text)


if __name__ == "__main__":
    unittest.main()
