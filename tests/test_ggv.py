"""The grip envelope (g-g-v diagram) must agree with the solver's own limits."""
import json, os, unittest
from fslap import api
from fslap.ggv import ggv, envelope
from fslap.params import default_car, default_conditions
from fslap.vehicle import Vehicle

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


class Ggv(unittest.TestCase):
    def setUp(self):
        self.car, self.cond = default_car(), default_conditions()

    def test_shape(self):
        g = ggv(self.car, self.cond)
        self.assertEqual(len(g["curves"]), 5)
        for c in g["curves"]:
            self.assertEqual(c["ay"][0], 0.0)
            self.assertTrue(all(b > a for a, b in zip(c["ay"], c["ay"][1:])))          # cornering rises along the outline
            self.assertGreater(c["acc"][0], 0.3); self.assertLess(c["brake"][0], -1.0)  # straight-line ends
            self.assertLess(c["acc"][-1], c["acc"][0])                                  # cornering uses up traction
            self.assertGreater(c["brake"][-1], c["brake"][0])                           # and braking grip
            self.assertTrue(all(a >= b for a, b in zip(c["acc"], c["brake"])))

    def test_downforce_widens_it_with_speed(self):
        g = ggv(self.car, self.cond)["curves"]
        self.assertGreater(g[-1]["ay"][-1], g[0]["ay"][-1] + 0.2)
        self.assertLess(g[-1]["brake"][0], g[0]["brake"][0] - 0.3)
        self.assertLess(g[-1]["acc"][0], g[0]["acc"][0])            # power limited at speed
        flat = dict(self.car, cl_a=0.0)
        f = ggv(flat, self.cond)["curves"]
        self.assertLess(f[-1]["ay"][-1], f[0]["ay"][-1] + 0.02)       # no wings: no gain with speed (drag even costs a little)

    def test_matches_skidpad_and_corner_speed(self):
        # The skidpad (9.125 m radius) is run at the cornering limit, so its speed must sit on the outline.
        veh = Vehicle(self.car, self.cond)
        v = veh.corner_speed(1 / 9.125)
        lim = envelope(veh, v)["ay"][-1]
        self.assertAlmostEqual(v * v / 9.125 / 9.81, lim, delta=0.06)

    def test_lap_stays_inside(self):
        api.load_tracks(open(os.path.join(ROOT, "data", "tracks.json")).read())
        r = json.loads(api.run(json.dumps(dict(track="fsuk_autocross"))))
        g = r["ggv"]["curves"]
        out = 0
        for v, ax, ay in zip(r["ch"]["v"], r["ch"]["ax"], r["ch"]["ay"]):
            c = min(g, key=lambda q: abs(q["v"] - v))
            if abs(v - c["v"]) > 1.0:
                continue
            if abs(ay) > c["ay"][-1] + 0.12 or ax > c["acc"][0] + 0.12 or ax < c["brake"][0] - 0.12:
                out += 1
        self.assertLess(out, 0.01 * len(r["ch"]["v"]))      # a handful sit just outside: body movement and tyre warm-up

    def test_driver_share_shrinks_it(self):
        a = ggv(self.car, self.cond)["curves"][1]
        b = ggv(self.car, dict(self.cond, driver=80, driver_brake=80))["curves"][1]
        self.assertLess(b["ay"][-1], a["ay"][-1] - 0.1)
        self.assertGreater(b["brake"][0], a["brake"][0] + 0.1)


if __name__ == "__main__":
    unittest.main()
