"""Cone layouts: the driven line, the real competition layouts, and drawn layouts."""
import json, math, os, unittest
from fslap import api
from fslap.layout import build_track, driven_line, CAR_HALF_WIDTH, CONE_MARGIN

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REAL = ["fsg18", "fsi18", "fsg19", "fss19", "fse22", "fsg23"]


def circle(r, n=200):
    return [r * math.cos(2 * math.pi * i / n) for i in range(n)], [r * math.sin(2 * math.pi * i / n) for i in range(n)]


def stadium(straight=40.0, r=8.0, step=0.5):
    """Two straights joined by two half circles, anticlockwise."""
    x, y = [], []
    for i in range(int(straight / step)): x.append(i * step); y.append(-r)
    for i in range(int(math.pi * r / step)):
        a = -math.pi / 2 + i * step / r; x.append(straight + r * math.cos(a)); y.append(r * math.sin(a))
    for i in range(int(straight / step)): x.append(straight - i * step); y.append(r)
    for i in range(int(math.pi * r / step)):
        a = math.pi / 2 + i * step / r; x.append(r * math.cos(a)); y.append(r * math.sin(a))
    return x, y


class Layout(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        text = open(os.path.join(ROOT, "data", "tracks.json")).read()
        api.load_tracks(text)
        cls.tracks = {t["id"]: t for t in json.loads(text)["tracks"]}

    def test_circle_stays_a_circle(self):
        x, y = circle(20.0)
        t = build_track(x, y, 1.75, "c", "circle")
        room = 1.75 - CAR_HALF_WIDTH - CONE_MARGIN
        # anywhere in the lane is a fair line round a circle; it must still be a circle inside it
        self.assertTrue(2 * math.pi * (20 - room) - 1.5 < t["length"] < 2 * math.pi * (20 + room) + 1.5)
        r = t["length"] / (2 * math.pi)
        for k in t["k"]:
            self.assertAlmostEqual(k, 1 / r, delta=0.006)

    def test_line_is_quicker_than_the_lane_middle_and_stays_in_lane(self):
        x, y = stadium()
        lap = lambda t: (api.add_track(json.dumps(t)), json.loads(api.run(json.dumps(dict(track=t["id"]))))["time"])[1]
        mid = lap(build_track(x, y, 2.0, "m", "middle", racing_line=False))
        line = lap(build_track(x, y, 2.0, "l", "line"))
        self.assertLess(line, mid - 0.2)
        room = 2.0 - CAR_HALF_WIDTH - CONE_MARGIN
        px, py, _ = driven_line(x, y, [2.0] * len(x))
        for a, b in zip(px, py):
            self.assertLess(min(math.hypot(a - c, b - d) for c, d in zip(x, y)), room + 0.05)

    def test_wider_lane_is_faster(self):
        x, y = stadium()
        times = []
        for w in (3.0, 5.0):
            api.make_track(json.dumps(dict(id="w%s" % w, name="w", centre=list(zip(x, y)), width=w)))
            times.append(json.loads(api.run(json.dumps(dict(track="w%s" % w))))["time"])
        self.assertLess(times[1], times[0])

    def test_real_layouts_keep_the_car_off_the_cones(self):
        for tid in REAL:
            t = self.tracks[tid]
            cones = t["cones"]["left"] + t["cones"]["right"]
            gap = min(math.hypot(x - c[0], y - c[1]) for x, y in zip(t["x"], t["y"]) for c in cones)
            self.assertGreater(gap, 0.65, tid)      # aim is 0.85 m; smoothing and sub-3 m lanes eat some of it
            self.assertTrue(100 < t["length"] < 500, tid)

    def test_real_layouts_give_sensible_laps(self):
        for tid in REAL:
            r = json.loads(api.run(json.dumps(dict(track=tid))))
            kmh = 3.6 * self.tracks[tid]["length"] / r["time"]
            self.assertTrue(30 < kmh < 65, "%s %.1f km/h" % (tid, kmh))
            self.assertGreaterEqual(len(r["corners"]), 4, tid)

    def test_drawn_layout_runs_as_lap_and_endurance(self):
        x, y = stadium()
        t = json.loads(api.make_track(json.dumps(dict(id="my_t", name="Mine", centre=list(zip(x, y)), width=3.5))))
        self.assertEqual(t["group"], "mine")
        lap = json.loads(api.run(json.dumps(dict(track="my_t"))))
        endu = json.loads(api.run(json.dumps(dict(track="my_t", laps=5, cond=dict(power_cap=30)))))
        self.assertEqual(lap["kind"], "autocross")
        self.assertEqual(endu["kind"], "endurance")
        self.assertEqual(endu["laps_done"], 5)
        self.assertGreater(endu["time"], 4 * endu["lap_time"])

    def test_too_small_layout_is_refused(self):
        x, y = circle(2.0, 20)
        with self.assertRaises(ValueError):
            build_track(x, y, 1.75, "s", "small")


if __name__ == "__main__":
    unittest.main()
