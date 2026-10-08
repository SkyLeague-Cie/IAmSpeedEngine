"""Source guards only; native impact precision/repeatability require runtime validation."""
from pathlib import Path
import re
import unittest

ROOT = Path(__file__).resolve().parents[1] / "Source/IAmSpeed/SubBodies/Common"


def helper(filename, pair):
    source = (ROOT / filename).read_text(encoding="utf-8-sig")
    source = re.sub(r"//[^\n]*|/\*.*?\*/", "", source, flags=re.S)
    match = re.search(r"void Quantize" + pair + r"ContactHit\(SHitResult& Hit\)\s*\{([^}]*)\}", source)
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

    def test_contact_points_and_normals_are_untouched(self):
        for _, body in self.helpers:
            self.assertEqual(body, "")

    def test_sphere_box_helpers_have_identical_operations(self):
        self.assertEqual(self.helpers[0][1], self.helpers[1][1])


    def test_persistent_sphere_contact_preserves_point_and_normal(self):
        source = (ROOT.parent / "Solid/BoxSubBody.cpp").read_text(encoding="utf-8-sig")
        source = re.sub(r"//[^\n]*|/\*.*?\*/", "", source, flags=re.S)
        body = source.split("bool UBoxSubBody::TryBuildPersistentSphereContact(", 1)[1]
        body = body.split("bool UBoxSubBody::SweepTOI(", 1)[0]
        self.assertNotIn("Quantize", body)
        self.assertRegex(body, r"OutHit\s*=\s*SHitResult\(\s*true,\s*ContactPoint,\s*N,\s*0\.0f\)")
        self.assertIn(".GetSafeNormal()", body)
        self.assertIn("N.IsNearlyZero()", body)

    def test_disabled_quantization_has_searchable_tag(self):
        tag = "#TODO see if it is still useful for netcode with new architecture"
        for relative in ("Common/ISphereSweeper.cpp", "Common/IBoxSweeper.cpp", "Solid/BoxSubBody.cpp"):
            self.assertIn(tag, (ROOT.parent / relative).read_text(encoding="utf-8-sig"))

if __name__ == "__main__":
    unittest.main()