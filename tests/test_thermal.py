"""Checks on battery and motor heating and the cut-back when hot."""
import json, os, sys, unittest
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from fslap import api
from fslap.thermal import batt_derate, motor_derate, heat_step, batt_thermal_mass

api.load_tracks(open(os.path.join(ROOT, "data", "tracks.json")).read())


def run(track, cond=None, car=None):
    return json.loads(api.run(json.dumps(dict(track=track, car=car or {}, cond=cond or {}))))


class Thermal(unittest.TestCase):
    def test_cut_back_curves(self):
        self.assertEqual(batt_derate(40), 1.0)
        self.assertAlmostEqual(batt_derate(55), 0.5)
        self.assertEqual(batt_derate(60), 0.0)                 # rules limit
        self.assertEqual(motor_derate(90), 1.0)
        self.assertAlmostEqual(motor_derate(120), 0.3)

    def test_sealed_pack_heats_by_energy_over_heat_capacity(self):
        # No cooling: temperature rise = heat / heat capacity, a hand calculation.
        mass = batt_thermal_mass(6.5)
        self.assertAlmostEqual(heat_step(25.0, 1000.0, 0.0, mass, 20.0, 600.0), 25.0 + 1000.0 * 600.0 / mass, places=6)
        # With cooling it settles where heating equals cooling.
        self.assertAlmostEqual(heat_step(25.0, 1000.0, 20.0, mass, 20.0, 1e7), 20.0 + 1000.0 / 20.0, places=3)

    def test_endurance_heats_the_pack_and_motor(self):
        r = run("fsuk_endurance", {"lift_coast": 15})
        self.assertTrue(r["finished"])
        h = r["heat"]
        self.assertTrue(35 < h["batt_end"] < 60)               # typical: packs finish endurance warm
        self.assertTrue(55 < h["motor_peak"] < 100)
        rows = r["lap_rows"]
        self.assertGreater(rows[-1]["batt_temp"], rows[0]["batt_temp"] + 10)

    def test_hot_day_poor_cooling_cuts_power_and_costs_time(self):
        base = run("fsuk_endurance", {"lift_coast": 15})
        hot = run("fsuk_endurance", {"lift_coast": 15, "air_temp": 30, "batt_temp": 35}, {"batt_cool": 0, "motor_cool": 10})
        self.assertLess(hot["heat"]["batt_cut_min"], 100)
        self.assertLess(hot["heat"]["motor_cut_min"], 100)
        self.assertLess(hot["heat"]["batt_peak"], 60.5)        # the cut-back keeps the pack at or under the rules limit
        self.assertGreater(hot["lap_rows"][-1]["time"], base["lap_rows"][-1]["time"] + 1.0)

    def test_hot_motor_is_slower_off_the_line(self):
        self.assertGreater(run("accel", {"motor_temp": 115})["time"], run("accel")["time"] + 0.2)


if __name__ == "__main__":
    unittest.main()
