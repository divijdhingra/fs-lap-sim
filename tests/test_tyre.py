"""Checks on the Pacejka Magic Formula tyre curves."""
import math, os, sys, unittest
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from fslap import Vehicle, default_car, default_conditions
from fslap.tyre import MagicCurve, C_LAT, E_LAT


class Tyre(unittest.TestCase):
    def setUp(self):
        self.c = MagicCurve(C_LAT, E_LAT, math.radians(8))

    def test_peak_is_at_the_chosen_slip(self):
        self.assertAlmostEqual(self.c.force_ratio(math.radians(8)), 1.0, places=6)
        self.assertLess(self.c.force_ratio(math.radians(7)), 1.0)
        self.assertLess(self.c.force_ratio(math.radians(12)), 1.0)      # falls away past the peak

    def test_slip_for_inverts_the_curve(self):
        for u in (0.1, 0.35, 0.6, 0.85, 0.97):
            self.assertAlmostEqual(self.c.force_ratio(self.c.slip_for(u)), u, delta=0.004)

    def test_slip_angle_and_steer_at_a_corner(self):
        veh = Vehicle(default_car(), default_conditions())
        v = veh.corner_speed(1 / 15.0)
        st = veh.point_state(v, 1 / 15.0, 0.0, None)
        self.assertTrue(0 < st["slip_f"] <= 8.001 and 0 < st["slip_r"] <= 8.001)
        geometric = math.degrees(veh.L / 15.0)                           # steer needed with no slip
        self.assertAlmostEqual(st["steer"], geometric + st["slip_f"] - st["slip_r"], places=6)

    def test_more_slip_means_more_cornering_drag(self):
        speeds = []
        for peak in (4, 12):
            car = default_car(); car["peak_slip_angle"] = peak
            veh = Vehicle(car, default_conditions())
            speeds.append(veh.corner_speed(1 / 15.0))
        self.assertGreater(speeds[0], speeds[1] - 1e-9)


if __name__ == "__main__":
    unittest.main()
