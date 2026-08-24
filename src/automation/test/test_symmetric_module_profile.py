from pathlib import Path
import unittest

import yaml


class TestSymmetricModuleProfile(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        path = Path(__file__).parents[1] / "config" / "catheter_limits.yaml"
        cls.profiles = yaml.safe_load(path.read_text(encoding="utf-8"))["catheters"]

    def test_each_profile_has_six_joint_values(self):
        for profile_name, profile in self.profiles.items():
            with self.subTest(profile=profile_name):
                for field in ("pos_lower", "pos_upper", "vel_min", "vel_max"):
                    values = profile[field]
                    self.assertEqual(len(values), 6)

    def test_bending_is_rotary_and_bidirectional(self):
        for profile_name, profile in self.profiles.items():
            with self.subTest(profile=profile_name):
                self.assertLess(profile["pos_lower"][2], 0.0)
                self.assertGreater(profile["pos_upper"][2], 0.0)
                self.assertEqual(profile["pos_lower"][2], profile["pos_lower"][5])
                self.assertEqual(profile["pos_upper"][2], profile["pos_upper"][5])
                self.assertEqual(profile["vel_max"][2], profile["vel_max"][5])
                self.assertEqual(profile["vel_max"][1], profile["vel_max"][4])


if __name__ == "__main__":
    unittest.main()
