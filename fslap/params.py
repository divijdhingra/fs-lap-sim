"""Every number the user can change, with its unit, default and a plain-English
explanation. The web page builds its input form from this list, so the form
and the physics can never disagree about what a parameter means.

All units are SI unless the label says otherwise (kW, kWh, rpm, deg and %
are used where they are the normal engineering way to quote the figure).
Defaults describe a *typical* mid-field electric Formula Student car; they are
not any specific team's car.
"""

def P(key, label, unit, default, lo, hi, step, help):
    return dict(key=key, label=label, unit=unit, default=default, min=lo, max=hi, step=step, help=help)

CAR_GROUPS = [
    ("Mass and geometry", [
        P("mass_car", "Car mass (no driver)", "kg", 230, 120, 400, 1, "Mass of the car ready to run, without the driver."),
        P("mass_driver", "Driver mass", "kg", 70, 40, 110, 1, "Driver with kit. Added to the car mass."),
        P("wheelbase", "Wheelbase", "m", 1.55, 1.4, 1.9, 0.01, "Distance between front and rear axles. The rules require at least 1.525 m."),
        P("track_f", "Front track width", "m", 1.22, 1.0, 1.6, 0.01, "Distance between the centres of the two front tyres."),
        P("track_r", "Rear track width", "m", 1.18, 1.0, 1.6, 0.01, "Distance between the centres of the two rear tyres."),
        P("cg_height", "Centre of gravity height", "m", 0.30, 0.15, 0.5, 0.005, "Height of the balance point of car plus driver. Higher means more weight shifts between tyres under braking, acceleration and cornering (load transfer)."),
        P("weight_front", "Static weight on front axle", "%", 47, 35, 65, 0.5, "Share of the total weight resting on the front tyres when the car is standing still."),
    ]),
    ("Tyres", [
        P("mu_x", "Longitudinal grip coefficient", "-", 1.45, 0.8, 2.2, 0.01, "Peak grip for accelerating and braking, as force divided by the load on the tyre, at the reference load."),
        P("mu_y", "Lateral grip coefficient", "-", 1.50, 0.8, 2.2, 0.01, "Peak grip for cornering at the reference load."),
        P("tyre_ref_load", "Reference tyre load", "N", 750, 300, 1500, 10, "The tyre load at which the grip coefficients above apply. Roughly one corner's share of the car's weight."),
        P("load_sens", "Load sensitivity", "-", 0.15, 0, 0.4, 0.01, "How much grip coefficient is lost when tyre load doubles from the reference load. 0.15 means 15% less. This is why load transfer costs grip: the loaded tyre gains less than the unloaded tyre loses."),
        P("tyre_radius", "Tyre rolling radius", "m", 0.203, 0.18, 0.28, 0.001, "Radius of the loaded tyre. 0.203 m suits a 16 inch diameter tyre on a 10 inch wheel."),
        P("rolling_res", "Rolling resistance coefficient", "-", 0.02, 0.005, 0.05, 0.001, "Drag from the tyres deforming as they roll, as a fraction of the car's weight."),
    ]),
    ("Aerodynamics", [
        P("cl_a", "Downforce area ClA", "m²", 3.2, 0, 7, 0.05, "Downforce coefficient times frontal area. Downforce = 0.5 x air density x ClA x speed². Set to 0 for a car with no wings."),
        P("cd_a", "Drag area CdA", "m²", 1.35, 0.3, 3, 0.01, "Drag coefficient times frontal area. Drag = 0.5 x air density x CdA x speed²."),
        P("aero_balance", "Aero balance (front)", "%", 45, 20, 70, 0.5, "Share of the downforce that pushes on the front axle."),
    ]),
    ("Motor and drivetrain", [
        P("drive", "Driven wheels", "", "RWD", None, None, None, "RWD = rear wheels driven, AWD = all four (for example hub motors), FWD = front wheels."),
        P("motor_torque", "Peak motor torque (total)", "N·m", 230, 20, 600, 1, "Total peak torque at the motor shaft(s). For several motors, add them up."),
        P("motor_power", "Peak motor power (total)", "kW", 100, 10, 200, 1, "Total mechanical power the motor(s) can deliver. The 80 kW rules limit usually bites first."),
        P("motor_rpm", "Motor top speed at nominal voltage", "rpm", 6000, 2000, 22000, 50, "Highest motor speed when the battery is at its nominal voltage. It scales with battery voltage, so top speed falls as the battery empties."),
        P("gear_ratio", "Gear ratio", "-", 3.8, 1, 16, 0.05, "Motor turns per wheel turn. Higher gives more push at the wheels but a lower top speed."),
        P("drive_eff", "Drivetrain efficiency", "%", 88, 60, 99, 0.5, "Share of battery electrical power that reaches the road as push, after inverter, motor and transmission losses."),
    ]),
    ("Battery", [
        P("batt_kwh", "Battery capacity", "kWh", 6.5, 2, 12, 0.1, "Usable energy stored in the accumulator when full."),
        P("cells_series", "Cells in series", "-", 140, 30, 144, 1, "Sets pack voltage: each lithium-ion cell runs from 4.2 V (full) to 3.0 V (empty). The rules cap pack voltage at 600 V."),
        P("batt_res", "Pack internal resistance", "Ω", 0.20, 0.02, 1, 0.01, "Resistance inside the battery. It makes voltage sag under load and turns some energy into heat."),
        P("batt_imax", "Maximum discharge current", "A", 170, 50, 500, 5, "Most current the pack is allowed to supply. With low voltage (low charge) this limits power below 80 kW."),
        P("regen_kw", "Maximum regen power", "kW", 20, 0, 80, 1, "Most electrical power recovered into the battery while braking (regenerative braking). 0 switches regen off."),
    ]),
    ("Brakes", [
        P("brake_bias", "Brake bias (front)", "%", 65, 40, 85, 0.5, "Share of the total braking force sent to the front tyres. The best value is where front and rear tyres reach their grip limit together."),
    ]),
    ("Suspension", [
        P("wheel_rate_f", "Front wheel rate", "N/mm", 35, 5, 150, 1, "Spring stiffness measured at the wheel: the force needed to push one front wheel up by 1 mm. It is the spring rate multiplied by the square of the motion ratio (how far the spring moves per mm of wheel travel)."),
        P("wheel_rate_r", "Rear wheel rate", "N/mm", 35, 5, 150, 1, "Spring stiffness measured at one rear wheel."),
        P("arb_f", "Front anti-roll bar", "N·m/deg", 0, 0, 1500, 10, "Extra resistance to body roll from a front anti-roll bar. It stiffens roll without stiffening the ride. More at the front shifts cornering load transfer forward, which adds understeer."),
        P("arb_r", "Rear anti-roll bar", "N·m/deg", 0, 0, 1500, 10, "Extra roll resistance from a rear anti-roll bar. More at the rear adds oversteer."),
        P("damper_f", "Front damper rate", "N·s/m", 2000, 100, 12000, 50, "Damper force at the wheel for each metre per second of suspension movement. Dampers do not change how far the body finally moves, only how quickly it gets there and whether it overshoots."),
        P("damper_r", "Rear damper rate", "N·s/m", 2000, 100, 12000, 50, "Damper force per metre per second at one rear wheel."),
        P("unsprung", "Unsprung mass per corner", "kg", 10, 3, 25, 0.5, "Mass of one wheel, tyre, upright and brake: the parts that are not carried by the springs."),
    ]),
]

COND_GROUP = ("Conditions", [
    P("soc_start", "Battery charge at start", "%", 100, 5, 100, 1, "State of charge when the run starts. Lower charge means lower voltage, less power and a lower top speed."),
    P("power_cap", "Power limit", "kW", 80, 5, 80, 1, "Electrical power limit at the battery. The rules cap this at 80 kW. Teams turn it down in endurance to make the battery last."),
    P("tyre_wear", "Tyre wear at start", "%", 0, 0, 100, 1, "0 = new tyres, 100 = fully worn. Assumed to cut grip in a straight line down to 12% less at fully worn."),
    P("wear_rate", "Tyre wear rate", "%/km", 0.5, 0, 5, 0.1, "How quickly the tyres wear during endurance."),
    P("track_grip", "Track grip", "%", 100, 40, 110, 1, "Scales all tyre grip. 100 = warm, dry tarmac. Roughly 80 = damp, 60 = wet."),
    P("air_temp", "Air temperature", "°C", 20, -5, 45, 1, "Warmer air is thinner, which reduces both downforce and drag."),
])

PARAM_GROUPS = dict(
    car=[dict(name=n, params=ps) for n, ps in CAR_GROUPS],
    cond=dict(name=COND_GROUP[0], params=COND_GROUP[1]),
)

def default_car():
    return {p["key"]: p["default"] for _, ps in CAR_GROUPS for p in ps}

def default_conditions():
    return {p["key"]: p["default"] for p in COND_GROUP[1]}
