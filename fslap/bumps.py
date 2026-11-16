"""Road bumps: how much the load on a tyre flutters on a rough surface.

Uses the classic "quarter-car" model: one corner of the car as two masses
stacked on springs.

        sprung mass     (that corner's share of the body)
        spring + damper (the suspension)
        unsprung mass   (wheel, tyre, upright, brake)
        tyre spring     (the tyre squashing vertically)
        road            (moves up and down as the car drives over it)

For a road that moves up and down at a given frequency, the model gives how
much the tyre's load changes per metre of road movement (the "gain"). A real
road is a mix of all frequencies; ISO 8608 describes that mix with one
roughness number. Adding up the gain over all frequencies, weighted by that
mix, gives the typical (RMS) tyre load variation.

Assumptions
- Each corner is treated on its own (no anti-roll bar or body roll effect on bumps).
- Linear spring, damper and tyre; the tyre never leaves the ground.
- Roughness is the same everywhere on the track and for all four wheels.
- Road spectrum follows ISO 8608 (roughness falls with the square of spatial
  frequency), counted from 0.3 to 60 Hz.
"""
import cmath, math

N0 = 0.1                                   # reference spatial frequency, cycles per metre (ISO 8608)
_FREQS = [0.3 * (60 / 0.3) ** (i / 159.0) for i in range(160)]


def tyre_load_gain(ms, mu, k, c, kt, f):
    """Tyre load change per metre of road movement at frequency f (N/m)."""
    s = 2j * math.pi * f
    d = k + c * s                                           # suspension spring + damper
    zu = kt / (mu * s * s + kt + d * ms * s * s / (ms * s * s + d))   # wheel movement per road movement
    return abs(kt * (1 - zu))


def tyre_load_rms(ms, mu, k, c, kt, v, gd0):
    """Typical tyre load variation (N, RMS) at speed v (m/s) on a road of ISO
    roughness gd0 (m³ at 0.1 cycles/m)."""
    if gd0 <= 0 or v <= 0:
        return 0.0
    total = 0.0
    prev_f = prev_y = None
    for f in _FREQS:
        y = tyre_load_gain(ms, mu, k, c, kt, f) ** 2 * gd0 * N0 * N0 * v / (f * f)   # gain² x road spectrum
        if prev_f is not None:
            total += 0.5 * (y + prev_y) * (f - prev_f)
        prev_f, prev_y = f, y
    return math.sqrt(total)


def hop_frequency(mu, k, kt):
    """Frequency at which the wheel bounces between tyre and suspension springs (Hz)."""
    return math.sqrt((k + kt) / mu) / (2 * math.pi)
