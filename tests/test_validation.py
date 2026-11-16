"""The preset fitted to the logged FS UK stint must stay close to the log."""
import json, math, os, sys, unittest
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from fslap import api, PRESETS

api.load_tracks(open(os.path.join(ROOT, "data", "tracks.json")).read())
LOG = json.load(open(os.path.join(ROOT, "data", "validation.json")))


class Validation(unittest.TestCase):
    def test_fitted_preset_matches_log(self):
        p = [q for q in PRESETS if q["id"] == "logged"][0]
        r = json.loads(api.run(json.dumps(dict(track="fsuk_endurance", car=p["car"], cond=p["cond"]))))
        self.assertAlmostEqual(r["lap_time"] / LOG["best_time"], 1.0, delta=0.02)          # fitted
        e_log = LOG["lap_energy_kwh"][LOG["best_lap"] - 1]
        self.assertAlmostEqual(r["lap_rows"][1]["e_kwh"] / e_log, 1.0, delta=0.05)          # not fitted
        sv, lv = r["ch"]["v"], LOG["v"]
        rms = math.sqrt(sum((a - b) ** 2 for a, b in zip(sv, lv)) / len(sv)) * 3.6
        self.assertLess(rms, 5.5)                                                           # km/h, not fitted


if __name__ == "__main__":
    unittest.main()
