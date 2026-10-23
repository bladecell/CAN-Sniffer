"""Source-level safety/wiring checks, not target execution."""
import pathlib
import subprocess
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]


class MigrationTests(unittest.TestCase):
    def test_legacy_regression_bodies_are_preserved(self):
        old = subprocess.check_output([
            "git", "show", "7d365fb:sw/firmware/tests/on_target/main/lifecycle_tests.cpp"
        ], text=True)
        current = (ROOT / "main/lifecycle_tests.cpp").read_text()
        cases = old.split("TEST_CASE(")[1:6]
        self.assertEqual(len(cases), 5)
        for case in cases:
            # The last legacy case ends immediately before the first P2.8 case.
            self.assertIn("TEST_CASE(" + case.strip(), current)

    def test_no_implicit_erase_restart_or_duplicate_p28_cases(self):
        text = (ROOT / "main/lifecycle_tests.cpp").read_text()
        self.assertNotIn("Settings::getInstance().init()", text)
        self.assertNotIn("esp_restart()", text)
        self.assertNotIn('"[lifecycle][p2.8]"', text)
        self.assertIn("bool legacyTaskCreationShouldFail()", text)


if __name__ == "__main__":
    unittest.main()
