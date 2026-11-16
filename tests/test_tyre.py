"""Checks on the Pacejka Magic Formula tyre curves."""
import json, math, os, sys, unittest
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from fslap import Vehicle, default_car, default_conditions
from fslap import api
from fslap.tyre import MagicCurve, C_LAT, E_LAT, temp_grip, temp_step, COOL_0, COOL_V, HEAT_SHARE, TEMP_FLOOR

api.load_tracks(open(os.path.join(ROOT, "data", "tracks.json")).read())


def run(track, cond=None, car=None):
    return json.loads(api.run(json.dumps(dict(track=track, car=car or {}, cond=cond or {}))))


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


class Temperature(unittest.TestCase):
    def test_grip_is_best_at_the_optimum(self):
        self.assertEqual(temp_grip(70, 70), 1.0)
        self.assertAlmostEqual(temp_grip(30, 70), 0.9)                 # 10 % lost 40 C away
        self.assertAlmostEqual(temp_grip(110, 70), temp_grip(30, 70))
        self.assertEqual(temp_grip(-200, 70), TEMP_FLOOR)

    def test_temperature_settles_where_heating_equals_cooling(self):
        t, v, heat = 20.0, 15.0, 2000.0
        for _ in range(4000):
            t = temp_step(t, heat, v, 20.0, 1.0)
        self.assertAlmostEqual(t, 20.0 + HEAT_SHARE * heat / (COOL_0 + COOL_V * v), places=3)
        # one very long step lands on the same value and does not overshoot
        self.assertAlmostEqual(temp_step(20.0, heat, v, 20.0, 1e6), t, places=3)

    def test_cold_tyres_are_slower(self):
        warm, cold = run("fsuk_autocross"), run("fsuk_autocross", {"tyre_temp": 20})
        self.assertGreater(cold["time"], warm["time"] + 1.0)
        self.assertGreater(cold["tyres"]["temp_r_end"], 20)             # they warm up as the lap goes on

    def test_endurance_temperature_and_wear(self):
        r = run("fsuk_endurance", {"power_cap": 30})
        rows = r["lap_rows"]
        self.assertTrue(r["finished"])
        self.assertGreater(rows[-1]["temp_r"], rows[-1]["temp_f"] + 5)   # driven rear tyres run hotter
        self.assertLess(abs(rows[-1]["temp_r"] - rows[-2]["temp_r"]), 1.0)   # settled by the end
        self.assertTrue(40 < rows[-1]["temp_f"] < 100 and 40 < rows[-1]["temp_r"] < 110)
        self.assertGreater(r["tyres"]["wear_r"], r["tyres"]["wear_f"])   # and wear faster
        km = 27 * 0.81
        mean = 0.5 * (r["tyres"]["wear_f"] + r["tyres"]["wear_r"])
        self.assertTrue(0.5 * 0.5 * km < mean < 2 * 0.5 * km)            # near the entered 0.5 %/km
        self.assertEqual(run("fsuk_endurance", {"power_cap": 30, "wear_rate": 0})["tyres"]["wear_r"], 0)


if __name__ == "__main__":
    unittest.main()
