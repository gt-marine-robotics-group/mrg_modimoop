"""Checks of scenario validation and angular discontinuity handling."""
import math
import unittest
from pathlib import Path

from mrg_modimoop_test.metrics import wrap
from mrg_modimoop_test.scenario import ScenarioError, load


SCENARIOS = Path(__file__).resolve().parents[1] / 'scenarios'


class ScenarioMetricsTest(unittest.TestCase):
    def test_installed_controller_scenarios_are_valid(self):
        for path in SCENARIOS.glob('*.yaml'):
            with self.subTest(path=path.name):
                self.assertIn(load(path)['mode'], ('isolated', 'simulation'))

    def test_unknown_threshold_is_rejected(self):
        path = SCENARIOS / 'isolated_wrap.yaml'
        with self.assertRaisesRegex(ScenarioError, 'unknown key'):
            load(path, ['thresholds.no_such_check=1'])

    def test_heading_wrap_uses_short_arc(self):
        error = wrap(math.radians(-179) - math.radians(179))
        self.assertAlmostEqual(math.degrees(error), 2.0)


if __name__ == '__main__':
    unittest.main()
