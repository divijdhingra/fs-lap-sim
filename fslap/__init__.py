"""fslap: a quasi-steady-state lap time simulator for electric Formula Student cars.

Pure Python (standard library only) so the same code runs on a laptop and,
through Pyodide, directly inside a web browser.
"""
from .params import PARAM_GROUPS, PRESETS, default_car, default_conditions
from .vehicle import Vehicle
from .solver import simulate
