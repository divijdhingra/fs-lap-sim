"""Checks for the spring and damper replay against hand calculations."""
import json, math, os, sys, unittest
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from fslap import api, default_car, default_conditions, Vehicle
from fslap.ride import ride_replay, ride_metrics

api.load_tracks(open(os.path.join(ROOT, "data", "tracks.json")).read())


def step(car, ay=0.0, ax=0.0, secs=3.0):
    """A sudden, held input: the classic test of a spring and damper system."""
    veh = Vehicle(car, default_conditions())
    n = int(secs / 0.005)
    t = [i * 0.005 for i in range(n)]
    g = [0.0] * 5 + [1.0] * (n - 5)
    ch, summ = ride_replay(veh, t, [0.0] * n, [ax * x for x in g], [ay * x for x in g], lag=0.0)
    return veh, ch, summ


class Ride(unittest.TestCase):
    def test_settled_loads_match_lap_solver(self):
        # On the steady skidpad circle the replay must agree with the steady model.
        c = json.loads(api.run(json.dumps(dict(track="skidpad"))))["ch"]
        for i in (450, 500, 550):
            for w in ("fl", "fr", "rl", "rr"):
                self.assertAlmostEqual(c["fz_" + w][i], c["fzd_" + w][i], delta=5)
            self.assertAlmostEqual(c["roll"][i], c["roll_dyn"][i], delta=0.02)

    def test_roll_settles_at_hand_calculated_angle(self):
        veh, ch, _ = step(default_car(), ay=1.0)
        # only the CoG height above the roll axis rolls the body on its springs
        h_axis = veh.wf * veh.rc_f + (1 - veh.wf) * veh.rc_r
        expected = math.degrees(veh.m * 9.81 * (veh.h - h_axis) / (veh.kroll_f + veh.kroll_r))
        self.assertAlmostEqual(ch["roll_dyn"][-1], expected, delta=0.01)

    def test_roll_centres_keep_total_load_transfer(self):
        # Raising the roll centres cuts roll but the total load moved across the car is unchanged.
        lo, hi = dict(default_car(), rc_f=0.0, rc_r=0.0), dict(default_car(), rc_f=0.10, rc_r=0.10)
        (va, ca, _), (vb, cb, _) = step(lo, ay=1.0), step(hi, ay=1.0)
        self.assertLess(cb["roll_dyn"][-1], 0.75 * ca["roll_dyn"][-1])
        for ch, veh in ((ca, va), (cb, vb)):
            moved = (ch["fzd_fr"][-1] - ch["fzd_fl"][-1]) / 2 * veh.tf + (ch["fzd_rr"][-1] - ch["fzd_rl"][-1]) / 2 * veh.tr
            self.assertAlmostEqual(moved, veh.m * 9.81 * veh.h, delta=6)
        # with roll centres, part of the load arrives at once through the links; shown
        # with soft dampers, which otherwise pass load on only as the body rolls
        soft = dict(damper_f=300, damper_r=300)
        (_, ca, _), (_, cb, _) = step(dict(lo, **soft), ay=1.0), step(dict(hi, **soft), ay=1.0)
        self.assertGreater(cb["fzd_fr"][5] - cb["fzd_fl"][5], ca["fzd_fr"][5] - ca["fzd_fl"][5] + 50)

    def test_anti_dive_cuts_pitch_not_load_transfer(self):
        (va, ca, _), (vb, cb, _) = step(dict(default_car(), anti_dive=0), ax=-1.0), step(dict(default_car(), anti_dive=60), ax=-1.0)
        self.assertLess(cb["pitch"][-1], 0.8 * ca["pitch"][-1])
        self.assertAlmostEqual(cb["fzd_fl"][-1] + cb["fzd_fr"][-1], ca["fzd_fl"][-1] + ca["fzd_fr"][-1], delta=3)
        (_, sa, _), (_, sb, _) = step(dict(default_car(), anti_squat=0), ax=0.8), step(dict(default_car(), anti_squat=60), ax=0.8)
        self.assertLess(abs(sb["pitch"][-1]), 0.8 * abs(sa["pitch"][-1]))

    def test_soft_dampers_overshoot_by_textbook_amount(self):
        # For a spring-mass-damper with damping ratio z < 1, a step input
        # overshoots by exp(-pi z / sqrt(1 - z^2)).
        car = dict(default_car(), damper_f=600, damper_r=600)
        veh, ch, _ = step(car, ay=1.0)
        z = ride_metrics(veh)["zeta_roll"]
        self.assertLess(z, 1.0)
        expected = math.exp(-math.pi * z / math.sqrt(1 - z * z))
        overshoot = max(ch["roll_dyn"]) / ch["roll_dyn"][-1] - 1
        self.assertAlmostEqual(overshoot, expected, delta=0.02)

    def test_stiff_dampers_do_not_overshoot(self):
        _, ch, _ = step(default_car(), ay=1.0)
        self.assertLess(max(ch["roll_dyn"]) / ch["roll_dyn"][-1], 1.005)

    def test_braking_dive_settles_with_correct_load_transfer(self):
        veh, ch, _ = step(default_car(), ax=-1.0)
        self.assertGreater(ch["pitch"][-1], 0)            # nose down
        front = ch["fzd_fl"][-1] + ch["fzd_fr"][-1]
        expected = veh.m * 9.81 * veh.wf + veh.m * 9.81 * veh.h / veh.L
        self.assertAlmostEqual(front, expected, delta=3)

    def test_ride_frequency_formula(self):
        veh = Vehicle(default_car(), default_conditions())
        m = ride_metrics(veh)
        corner = veh.m * veh.wf / 2 - veh.m_unsprung
        self.assertAlmostEqual(m["freq_f"], math.sqrt(35000 / corner) / (2 * math.pi), places=3)
        self.assertTrue(2.5 < m["freq_f"] < 4.5)          # usual FS range with wings

    def test_feedback_settles(self):
        # Re-solving the lap with updated tyre loads should home in on one answer.
        r = json.loads(api.run(json.dumps(dict(track="fsuk_autocross"))))["ride"]
        t = r["feedback_times"]
        self.assertLess(abs(t[-1] - t[-2]), 0.06)
        self.assertGreater(r["time_cost"], 0)             # body movement costs time, never gains it

    def test_badly_underdamped_car_is_slower(self):
        base = json.loads(api.run(json.dumps(dict(track="fsuk_autocross"))))["time"]
        soft = json.loads(api.run(json.dumps(dict(track="fsuk_autocross", car={"damper_f": 200, "damper_r": 200}))))["time"]
        self.assertGreater(soft, base + 0.05)

    def test_steady_cornering_unaffected_by_dampers(self):
        # On a constant circle the body is settled, so dampers should not matter.
        a = json.loads(api.run(json.dumps(dict(track="skidpad"))))["time"]
        b = json.loads(api.run(json.dumps(dict(track="skidpad", car={"damper_f": 400, "damper_r": 400}))))["time"]
        self.assertAlmostEqual(a, b, delta=0.02)


if __name__ == "__main__":
    unittest.main()
