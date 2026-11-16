"""Thin JSON wrapper used by the web page (and handy from the command line)."""
import json
from .params import PARAM_GROUPS, PRESETS, default_car, default_conditions
from .solver import simulate

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
    return json.dumps(simulate(car, cond, _TRACKS[req["track"]]), separators=(",", ":"))
