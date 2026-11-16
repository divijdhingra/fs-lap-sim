"""Thin JSON wrapper used by the web page (and handy from the command line)."""
import json
from .params import PARAM_GROUPS, PRESETS, default_car, default_conditions
from .solver import simulate
from .layout import build_track

_TRACKS = {}

def load_tracks(text):
    _TRACKS.clear()
    for t in json.loads(text)["tracks"]:
        _TRACKS[t["id"]] = t
    return json.dumps([dict(id=t["id"], name=t["name"], kind=t["kind"], length=t["length"],
                            laps=t.get("laps", 1), source=t.get("source", "")) for t in _TRACKS.values()])

def schema():
    return json.dumps(dict(groups=PARAM_GROUPS, car=default_car(), cond=default_conditions(), presets=PRESETS))

def run(text):
    req = json.loads(text)
    car = dict(default_car()); car.update(req.get("car") or {})
    cond = dict(default_conditions()); cond.update(req.get("cond") or {})
    track = _TRACKS[req["track"]]
    if req.get("laps"):                     # run a one-lap cone layout as a multi-lap endurance
        track = dict(track, laps=max(1, int(req["laps"])), kind="endurance" if int(req["laps"]) > 1 else track["kind"])
    return json.dumps(simulate(car, cond, track), separators=(",", ":"))

def add_track(text):
    """Register a finished track (used to hand a drawn layout to the second engine)."""
    t = json.loads(text)
    _TRACKS[t["id"]] = t

def make_track(text):
    """Build a track from a layout drawn in the browser: the middle of the lane
    as points [[x, y], ...] in metres, in driving order, and the lane width."""
    q = json.loads(text)
    pts = q["centre"]
    width = min(8.0, max(2.5, float(q["width"])))
    t = build_track([p[0] for p in pts], [p[1] for p in pts], width / 2, q["id"], q["name"],
                    source=q.get("source", ""), cones=q.get("cones"))
    t["group"] = "mine"; t["lane_width"] = width
    _TRACKS[t["id"]] = t
    return json.dumps(t, separators=(",", ":"))
