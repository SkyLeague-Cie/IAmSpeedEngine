"""Source guards only; native impact precision/repeatability require runtime validation."""
from pathlib import Path
import re
import unittest

ROOT = Path(__file__).resolve().parents[1] / "Source/IAmSpeed/SubBodies/Common"


def helper(filename, pair):
    source = (ROOT / filename).read_text(encoding="utf-8-sig")
    source = re.sub(r"//[^\n]*|/\*.*?\*/", "", source, flags=re.S)
    match = re.search(r"void Quantize" + pair + r"ContactHit\(SHitResult& Hit\)\s*\{([^}]+)\}", source)
    if match is None:
        raise AssertionError("Contact helper missing")
    return source, " ".join(match.group(1).split())


class ContactPointPrecision(unittest.TestCase):
    def setUp(self):
        self.helpers = [helper("ISphereSweeper.cpp", "SphereBox"), helper("IBoxSweeper.cpp", "BoxSphere")]

    def test_no_active_point_quantization_in_either_sweeper(self):
        for source, body in self.helpers:
            self.assertNotIn("ContactPointQuantizationCm", source)
            self.assertNotIn("QuantizeVectorCm", source)
            self.assertNotRegex(body, r"ImpactPoint\s*=")

    def test_normals_keep_existing_quantization_and_points_are_untouched(self):
        for _, body in self.helpers:
            self.assertEqual(body, "Hit.ImpactNormal = Speed::QuantizeUnitNormal(Hit.ImpactNormal);")

    def test_sphere_box_helpers_have_identical_operations(self):
        self.assertEqual(self.helpers[0][1], self.helpers[1][1])


if __name__ == "__main__":
    unittest.main()