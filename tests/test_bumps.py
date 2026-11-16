"""Checks for the road bump (quarter-car) model."""
import json, math, os, sys, unittest
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from fslap import api
from fslap.bumps import tyre_load_gain, tyre_load_rms

api.load_tracks(open(os.path.join(ROOT, "data", "tracks.json")).read())
MS, MU, K, C, KT = 60.0, 10.0, 35000.0, 2000.0, 110000.0


def run(track, car=None, cond=None):
    return json.loads(api.run(json.dumps(dict(track=track, car=car or {}, cond=cond or {}))))


class Bumps(unittest.TestCase):
    def test_slow_bumps_just_lift_the_whole_corner(self):
        # Far below any resonance the corner moves as one lump, so the tyre load
        # change is simply mass x acceleration = (ms + mu) * (2 pi f)^2 per metre.
        f = 0.2
        self.assertAlmostEqual(tyre_load_gain(MS, MU, K, C, KT, f) / ((MS + MU) * (2 * math.pi * f) ** 2), 1.0, delta=0.02)

    def test_gain_matches_step_by_step_simulation(self):
        # Independent check: drive the two-mass model over a sine-wave road in
        # small time steps and measure the tyre load swing directly.
        f, amp, h = 8.0, 0.001, 0.0002
        zs = zu = vs = vu = 0.0
        peak = 0.0
        for i in range(int(4.0 / h)):
            t = i * h
            zr = amp * math.sin(2 * math.pi * f * t)
            fs = K * (zs - zu) + C * (vs - vu)
            ft = KT * (zr - zu)
            vs += -fs / MS * h; vu += (fs + ft) / MU * h
            zs += vs * h; zu += vu * h
            if t > 3.0:
                peak = max(peak, abs(ft))
        self.assertAlmostEqual(peak / amp / tyre_load_gain(MS, MU, K, C, KT, f), 1.0, delta=0.03)

    def test_flutter_grows_with_square_root_of_speed(self):
        a = tyre_load_rms(MS, MU, K, C, KT, 10.0, 16e-6)
        b = tyre_load_rms(MS, MU, K, C, KT, 40.0, 16e-6)
        self.assertAlmostEqual(b / a, 2.0, delta=0.01)

    def test_there_is_a_best_damper_setting(self):
        soft, mid, stiff = (tyre_load_rms(MS, MU, K, c, KT, 15.0, 16e-6) for c in (200, 1100, 8000))
        self.assertLess(mid, soft)
        self.assertLess(mid, stiff)

    def test_rougher_track_is_slower_and_smooth_track_costs_nothing(self):
        smooth = run("fsuk_autocross", cond={"roughness": 0})["time"]
        mid = run("fsuk_autocross", cond={"roughness": 16})["time"]
        rough = run("fsuk_autocross", cond={"roughness": 64})["time"]
        self.assertTrue(smooth < mid < rough)
        no_sens = run("fsuk_autocross", {"bump_sens": 0}, {"roughness": 64})["time"]
        self.assertAlmostEqual(no_sens, smooth, delta=0.001)

    def test_dampers_now_have_a_sweet_spot_in_lap_time(self):
        t = {c: run("fsuk_autocross", {"damper_f": c, "damper_r": c}, {"roughness": 16})["time"] for c in (200, 1200, 8000)}
        self.assertLess(t[1200], t[200] - 0.2)
        self.assertLess(t[1200], t[8000] - 0.2)


if __name__ == "__main__":
    unittest.main()
