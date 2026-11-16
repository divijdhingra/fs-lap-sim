"""Battery and motor heating, and the power cut-back that protects them.

Battery: every amp flowing through the pack's internal resistance makes heat
(current squared x resistance). The cells soak it up and shed a little to the
cooling air. Formula Student rules stop the car if any cell passes 60 C, so
the controller cuts the current limit as the pack nears that.

Motor and inverter: the share of electrical power that does not reach the
wheels is lost as heat, most of it in the motor windings and inverter. A
liquid cooling loop carries it to a radiator. The controller cuts torque as
the motor nears its temperature limit.

Assumptions
- The pack is one lump at one temperature (no hot cells), with a heat capacity
  worked out from its energy: 180 W·h per kg of cells and 900 J/(kg·K).
- The motor, inverter and their coolant are one lump of 10 kJ/K.
- Cooling is proportional to the temperature above the air, at a fixed rate
  (no change with car speed).
- MOTOR_HEAT_SHARE of the drivetrain losses heat the motor and inverter; the
  rest is lost in the gears, chain and bearings.
- Cut-back is a straight line between the start and limit temperatures.
- Within one lap the cut-back is fixed at its start-of-lap value.
"""
import math

CELL_WH_PER_KG = 180.0
CELL_CP = 900.0            # J/(kg K)
BATT_DERATE_START, BATT_LIMIT = 50.0, 60.0      # C: current limit falls from 100 % to 0
MOTOR_THERMAL_MASS = 10000.0                    # J/K
MOTOR_HEAT_SHARE = 0.7
MOTOR_DERATE_START, MOTOR_LIMIT = 100.0, 120.0  # C: torque falls from 100 % to MOTOR_FLOOR
MOTOR_FLOOR = 0.3


def batt_thermal_mass(kwh):
    return kwh * 1000.0 / CELL_WH_PER_KG * CELL_CP


def batt_derate(temp):
    """Share of the battery current limit allowed at this pack temperature."""
    return min(1.0, max(0.0, (BATT_LIMIT - temp) / (BATT_LIMIT - BATT_DERATE_START)))


def motor_derate(temp):
    """Share of peak motor torque allowed at this motor temperature."""
    x = min(1.0, max(0.0, (MOTOR_LIMIT - temp) / (MOTOR_LIMIT - MOTOR_DERATE_START)))
    return MOTOR_FLOOR + (1.0 - MOTOR_FLOOR) * x


def heat_step(temp, heat_w, cool_w_per_k, mass_j_per_k, air, dt):
    """New temperature of a lump after dt seconds of steady heating and cooling."""
    if cool_w_per_k <= 0:
        return temp + heat_w * dt / mass_j_per_k
    target = air + heat_w / cool_w_per_k
    return target + (temp - target) * math.exp(-cool_w_per_k * dt / mass_j_per_k)
