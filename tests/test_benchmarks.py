"""Sanity checks against known Formula Student numbers and basic physics.

Run with:  python3 -m unittest discover tests
"""
import json, os, sys, unittest
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from fslap import api, default_car, default_conditions, Vehicle

api.load_tracks(open(os.path.join(ROOT, "data", "tracks.json")).read())

def run(track, car=None, cond=None):
    return json.loads(api.run(json.dumps(dict(track=track, car=car or {}, cond=cond or {}))))


class Benchmarks(unittest.TestCase):
    def test_acceleration_in_fs_range(self):
        # Competitive electric FS cars cover 75 m in roughly 3.3 s (best 4WD) to 4.5 s (RWD).
        self.assertTrue(3.6 < run("accel")["time"] < 4.6)

    def test_awd_is_quicker_off_the_line(self):
        self.assertLess(run("accel", {"drive": "AWD"})["time"], run("accel")["time"] - 0.3)

    def test_skidpad_in_fs_range(self):
        # Good skidpad laps with wings are about 4.7 to 5.3 s.
        self.assertTrue(4.7 < run("skidpad")["time"] < 5.4)

    def test_skidpad_matches_hand_calculation(self):
        # No wings, no load sensitivity: speed = sqrt(mu * g * R), a textbook result.
        r = run("skidpad", {"cl_a": 0, "load_sens": 0, "mu_y": 1.4, "cd_a": 0.01, "rolling_res": 0.001})
        expected = 2 * 3.14159265 * 9.125 / (1.4 * 9.81 * 9.125) ** 0.5
        self.assertAlmostEqual(r["time"], expected, delta=0.06)

    def test_power_limit_respected(self):
        for t in ("accel", "fsuk_autocross"):
            self.assertLessEqual(run(t)["p_max"], 80.05)

    def test_endurance_energy(self):
        # Flat out at 80 kW a 6.5 kWh pack should NOT last 22 km...
        self.assertFalse(run("fsuk_endurance")["finished"])
        # ...but with the power turned down it should.
        r = run("fsuk_endurance", cond={"power_cap": 25})
        self.assertTrue(r["finished"])
        self.assertEqual(r["laps_done"], 27)

    def test_sensible_directions(self):
        base = run("fsuk_autocross")["time"]
        self.assertGreater(run("fsuk_autocross", {"mass_car": 280})["time"], base)       # heavier = slower
        self.assertGreater(run("fsuk_autocross", cond={"track_grip": 70})["time"], base)  # wet = slower
        self.assertGreater(run("fsuk_autocross", cond={"tyre_wear": 100})["time"], base)  # worn = slower
        self.assertGreater(run("fsuk_autocross", cond={"soc_start": 8})["time"], base)    # flat battery = slower

    def test_wheel_loads_add_up(self):
        v = Vehicle(default_car(), default_conditions())
        for speed, ax, ay in ((0, 0, 0), (20, 3, 10), (25, -12, -8)):
            total = sum(v.wheel_loads(speed, ax, ay))
            self.assertAlmostEqual(total, v.m * 9.81 + v.q_cl * speed ** 2, delta=1.0)


if __name__ == "__main__":
    unittest.main()
