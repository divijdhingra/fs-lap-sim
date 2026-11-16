"""Every number the user can change, with its unit, default and a plain-English
explanation. The web page builds its input form from this list, so the form
and the physics can never disagree about what a parameter means.

All units are SI unless the label says otherwise (kW, kWh, rpm, deg and %
are used where they are the normal engineering way to quote the figure).
Defaults describe a *typical* mid-field electric Formula Student car; they are
not any specific team's car.
"""

def P(key, label, unit, default, lo, hi, step, help, options=None):
    d = dict(key=key, label=label, unit=unit, default=default, min=lo, max=hi, step=step, help=help)
    if options:
        d["options"] = options          # a pick-list instead of a number
    return d

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
        P("peak_slip_angle", "Slip angle at peak cornering grip", "deg", 8, 3, 15, 0.5, "A tyre only makes cornering force when it points slightly away from the direction it is travelling. This is the angle at which it grips hardest. Larger angles mean more drag while cornering and more tyre heating."),
        P("peak_slip_ratio", "Wheel slip at peak drive or brake grip", "%", 10, 3, 25, 0.5, "A tyre only makes drive or brake force when it turns slightly faster or slower than the road passing under it. This is the amount of slip at which it grips hardest."),
        P("tyre_temp_opt", "Best tyre temperature", "°C", 70, 40, 110, 1, "The tread temperature at which the tyres grip hardest. Grip falls away either side of it: about 10% is lost 40 °C away, and the loss grows with the square of the distance."),
        P("tyre_radius", "Tyre rolling radius", "m", 0.203, 0.18, 0.28, 0.001, "Radius of the loaded tyre. 0.203 m suits a 16 inch diameter tyre on a 10 inch wheel."),
        P("tyre_kv", "Tyre vertical stiffness", "N/mm", 110, 40, 300, 5, "How stiff the tyre is as a spring when squashed vertically. It sets how the wheel bounces over bumps. It depends mainly on tyre pressure."),
        P("bump_sens", "Grip lost to load flutter", "-", 0.3, 0, 1, 0.05, "When a tyre's load flutters over bumps it loses grip, because a tyre sheds force at once when load drops but needs time to build it back. This is the share of the load flutter that is lost as grip: 0.3 means a tyre whose load varies by 10% loses 3% of its grip. It is an assumed figure; real values come from tyre testing."),
        P("rolling_res", "Rolling resistance coefficient", "-", 0.02, 0.005, 0.05, 0.001, "Drag from the tyres deforming as they roll, as a fraction of the car's weight."),
    ]),
    ("Aerodynamics", [
        P("cl_a", "Downforce area ClA", "m²", 3.2, 0, 7, 0.05, "Downforce coefficient times frontal area. Downforce = 0.5 x air density x ClA x speed². Set to 0 for a car with no wings."),
        P("cd_a", "Drag area CdA", "m²", 1.35, 0.3, 3, 0.01, "Drag coefficient times frontal area. Drag = 0.5 x air density x CdA x speed²."),
        P("aero_balance", "Aero balance (front)", "%", 45, 20, 70, 0.5, "Share of the downforce that pushes on the front axle."),
    ]),
    ("Motor and drivetrain", [
        P("drive", "Driven wheels", "", "RWD", None, None, None, "RWD = rear wheels driven, AWD = all four (for example hub motors), FWD = front wheels.", ["RWD", "AWD", "FWD"]),
        P("diff", "Left to right torque split", "", "Limited slip", None, None, None, "How drive torque is shared between the left and right wheels of a driven axle. Open = an open differential: both wheels get the same torque, so the lightly loaded inside wheel sets the limit and spins first. Limited slip = each wheel gets what its tyre can use. Torque vectoring = one motor per wheel, which also sends more torque to the outside wheel to help turn the car.", ["Limited slip", "Open", "Torque vectoring"]),
        P("tv_moment", "Torque vectoring turning moment", "N·m", 400, 0, 2000, 10, "The largest turning moment (yaw moment) the controller applies by driving the outside wheel harder than the inside one. It moves cornering work from the axle that is nearer its grip limit to the other. Only used when the torque split is set to torque vectoring."),
        P("motor_torque", "Peak motor torque (total)", "N·m", 230, 20, 600, 1, "Total peak torque at the motor shaft(s). For several motors, add them up."),
        P("motor_power", "Peak motor power (total)", "kW", 100, 10, 200, 1, "Total mechanical power the motor(s) can deliver. The 80 kW rules limit usually bites first."),
        P("motor_rpm", "Motor top speed at nominal voltage", "rpm", 6000, 2000, 22000, 50, "Highest motor speed when the battery is at its nominal voltage. It scales with battery voltage, so top speed falls as the battery empties."),
        P("gear_ratio", "Gear ratio", "-", 3.8, 1, 16, 0.05, "Motor turns per wheel turn. Higher gives more push at the wheels but a lower top speed."),
        P("drive_eff", "Drivetrain efficiency", "%", 88, 60, 99, 0.5, "Share of battery electrical power that reaches the road as push, after inverter, motor and transmission losses."),
        P("motor_cool", "Motor and inverter cooling", "W/K", 30, 0, 300, 1, "Heat the motor and inverter cooling loop sheds for each degree above the air temperature. A bigger radiator or more airflow raises it."),
    ]),
    ("Battery", [
        P("batt_kwh", "Battery capacity", "kWh", 6.5, 2, 12, 0.1, "Usable energy stored in the accumulator when full."),
        P("cells_series", "Cells in series", "-", 140, 30, 144, 1, "Sets pack voltage: each lithium-ion cell runs from 4.2 V (full) to 3.0 V (empty). The rules cap pack voltage at 600 V."),
        P("batt_res", "Pack internal resistance", "Ω", 0.20, 0.02, 1, 0.01, "Resistance inside the battery. It makes voltage sag under load and turns some energy into heat."),
        P("batt_imax", "Maximum discharge current", "A", 170, 50, 500, 5, "Most current the pack is allowed to supply. With low voltage (low charge) this limits power below 80 kW."),
        P("regen_kw", "Maximum regen power", "kW", 20, 0, 80, 1, "Most electrical power recovered into the battery while braking (regenerative braking). 0 switches regen off."),
        P("batt_cool", "Battery cooling", "W/K", 15, 0, 200, 1, "Heat the pack sheds for each degree it is above the air temperature. About 15 for a fan-cooled pack, more with ducted air or liquid cooling, 0 for a sealed box."),
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
        P("rc_f", "Front roll centre height", "m", 0.03, 0, 0.15, 0.005, "The roll centre is the point the body rolls about at that axle, set by the angles of the suspension arms. The higher it is, the more of the cornering load goes straight through the arms into the tyres instead of through the springs, so the body rolls less and the load reaches the tyres sooner."),
        P("rc_r", "Rear roll centre height", "m", 0.05, 0, 0.15, 0.005, "Roll centre height at the rear axle. Raising one end's roll centre moves load transfer to that end, like stiffening its anti-roll bar: higher at the rear adds oversteer."),
        P("anti_dive", "Anti-dive (front)", "%", 20, 0, 100, 5, "Share of the braking load transfer that the front suspension arms carry directly instead of the springs. 100 means the nose does not dive at all under braking. It changes how far and how fast the body pitches, not the settled tyre loads."),
        P("anti_squat", "Anti-squat (rear)", "%", 20, 0, 100, 5, "Share of the acceleration load transfer that the rear suspension arms carry directly, which stops the tail squatting under power."),
        P("camber_f", "Front static camber", "deg", -1.5, -5, 1, 0.1, "How far the front tyres lean when the car is standing still. Negative means the tops lean in towards the car. In a corner the body rolls and tips the outside tyre upright, so some negative camber keeps it gripping. Too much hurts braking and traction, where an upright tyre is best."),
        P("camber_r", "Rear static camber", "deg", -1.0, -5, 1, 0.1, "Static lean of the rear tyres. Negative means the tops lean in."),
        P("camber_gain", "Camber recovered in roll", "deg/deg", 0.5, 0, 1, 0.05, "How much of each degree of body roll the suspension arms cancel out by tilting the wheel back. 0 = the tyres tip over fully with the body, 1 = the tyres stay at their static camber however much the body rolls."),
        P("unsprung", "Unsprung mass per corner", "kg", 10, 3, 25, 0.5, "Mass of one wheel, tyre, upright and brake: the parts that are not carried by the springs."),
    ]),
]

COND_GROUP = ("Conditions", [
    P("driver", "Driver: cornering grip used", "%", 100, 50, 100, 1, "How much of the tyres' grip the driver uses in corners. 100 = a perfect driver on the limit in every corner. A good student driver is typically around 85 to 95."),
    P("driver_brake", "Driver: braking grip used", "%", 100, 30, 100, 1, "How hard the driver brakes, as a share of what the tyres could take. Most drivers are further from the limit under braking than in corners, because locking a wheel is easy and costly. A lower figure means earlier, gentler braking."),
    P("driver_power", "Driver: traction used on corner exit", "%", 100, 30, 100, 1, "How much of the available traction the driver uses when putting power down. On a straight, where grip is not the limit, the driver still uses full power."),
    P("traction_control", "Traction control", "", "Off", None, None, None, "On = the controller trims motor torque to hold the driven tyres at their best slip, so 97% of the available traction is used however cautious the driver is on the throttle. It makes no difference to a perfect driver.", ["Off", "On"]),
    P("lift_coast", "Lift and coast before braking", "m", 0, 0, 40, 1, "The driver lifts off the throttle this many metres before each braking point and lets the car roll. Costs a little lap time and saves energy, which is how teams make the battery last in endurance."),
    P("soc_start", "Battery charge at start", "%", 100, 5, 100, 1, "State of charge when the run starts. Lower charge means lower voltage, less power and a lower top speed."),
    P("power_cap", "Power limit", "kW", 80, 5, 80, 1, "Electrical power limit at the battery. The rules cap this at 80 kW. Teams turn it down in endurance to make the battery last."),
    P("batt_temp", "Battery temperature at start", "°C", 25, 0, 60, 1, "Pack temperature when the run starts. The rules stop the car at 60 °C, so the controller cuts the current limit in a straight line from full at 50 °C to nothing at 60 °C."),
    P("motor_temp", "Motor temperature at start", "°C", 40, 0, 120, 1, "Motor and inverter temperature when the run starts. Torque is cut in a straight line from full at 100 °C to 30% at 120 °C."),
    P("tyre_wear", "Tyre wear at start", "%", 0, 0, 100, 1, "0 = new tyres, 100 = fully worn. Assumed to cut grip in a straight line down to 12% less at fully worn."),
    P("tyre_temp", "Tyre temperature at start", "°C", 60, 0, 120, 1, "Tread temperature when the run starts. Formula Student bans tyre warmers, so a car straight out of the pits is near air temperature; 60 means the tyres have already done a warm-up lap or two."),
    P("wear_rate", "Tyre wear rate", "%/km", 0.5, 0, 5, 0.1, "How quickly the tyres wear during endurance when worked as hard as the default car on the FS UK layout. Tyres that slide more (more slip angle, more wheelspin, harder braking) wear faster in proportion to the energy lost in sliding, and faster again when hotter than their best temperature. Front and rear wear separately."),
    P("roughness", "Track roughness", "10⁻⁶ m³", 4, 0, 256, 1, "How bumpy the surface is, on the ISO 8608 road scale. 0 = perfectly smooth, 4 = race circuit, 16 = good tarmac (car park or airfield in good condition), 64 = average public road, 256 = poor road."),
    P("track_grip", "Track grip", "%", 100, 40, 110, 1, "Scales all tyre grip. 100 = warm, dry tarmac. Roughly 80 = damp, 60 = wet."),
    P("air_temp", "Air temperature", "°C", 20, -5, 45, 1, "Warmer air is thinner, which reduces both downforce and drag."),
])

# Ready-made starting points. Each lists only what differs from the defaults.
PRESETS = [
    dict(id="typical", name="Typical FS electric car", car={}, cond={},
         note="A generic mid-field electric Formula Student car. Not any specific team's car."),
    dict(id="logged", name="Fitted to the logged FS UK stint",
         car=dict(motor_torque=80, drive_eff=74, regen_kw=0, mu_x=1.32, mu_y=1.32),
         cond=dict(power_cap=46, driver=79, driver_brake=48, driver_power=79),
         note="Read from the log: 46 kW peak power, no regen, and a constant push of about 2000 N at full "
              "throttle (so the car was torque limited, not power limited). "
              "Fitted: tyre grip 1.32 with a driver using 79% of it in corners and on corner exit but only "
              "48% under braking, which reproduces the best lap time and the logged braking of 0.75 g; "
              "74% drivetrain efficiency, which reproduces the energy used per lap. "
              "Assumed: everything else, including mass (300 kg with driver). The true mass would change "
              "the fitted figures."),
]

PARAM_GROUPS = dict(
    car=[dict(name=n, params=ps) for n, ps in CAR_GROUPS],
    cond=dict(name=COND_GROUP[0], params=COND_GROUP[1]),
)

def default_car():
    return {p["key"]: p["default"] for _, ps in CAR_GROUPS for p in ps}

def default_conditions():
    return {p["key"]: p["default"] for p in COND_GROUP[1]}
