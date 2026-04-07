#!/usr/bin/env python3
"""
Calculate g1/g2/g3 for the FOC shaft-speed observer.

The equations follow the pole-placement procedure from:

    A. Bellini, S. Bifaretti, S. Costantini,
    "A Digital Speed Filter for Motion Control Drives with a Low Resolution
    Position Encoder", Automatika 44(1-2), 2003.

The firmware uses these values in FOC::apply_kalman():

    Th_hat = nTh + g1 * angle_residual
    W_hat  = nW  + g2 * angle_residual
    E_hat  = nE  + g3 * angle_residual

Use lower bandwidth for noisy encoders/direct-drive axes. Use higher bandwidth
only when angle lag starts hurting current control or dynamic response.
"""

from __future__ import annotations

import argparse
import math
import sys


DEFAULT_SAMPLE_TIME_S = 25e-6
DEFAULT_BANDWIDTH_RAD_S = 250.0
DEFAULT_DAMPING_ANGLE_DEG = 40.0

FIRMWARE_DEFAULT_GAINS = (
    0.015700989410003974,
    3.925227776360174,
    387.54711795263574,
)

PAPER_EXAMPLE_GAINS = (
    # Bellini et al. example: T=150 us, p0=w=1000 rad/s, theta=40 deg.
    0.31601092447081625,
    315.10620023445256,
    124212.25952279102,
)


def calculate_observer_gains(
    sample_time_s: float = DEFAULT_SAMPLE_TIME_S,
    bandwidth_rad_s: float = DEFAULT_BANDWIDTH_RAD_S,
    damping_angle_deg: float = DEFAULT_DAMPING_ANGLE_DEG,
    real_pole_rad_s: float | None = None,
) -> tuple[float, float, float]:
    """Return (g1, g2, g3) for the static Kalman-style speed filter.

    Args:
        sample_time_s: Observer/control-loop period in seconds. This must match
            the firmware's `T`; VBDrive currently uses 25e-6 s.
        bandwidth_rad_s: Observer response-speed knob `w`, in rad/s. In pole
            terms it is the distance of the complex pole pair from the origin:
            s = -w*cos(theta) +- j*w*sin(theta). In tuning terms, lower values
            make the velocity estimate smoother and laggier; higher values make
            it follow faster but pass more encoder noise into velocity and Iq.
            The decay time constant is about 1 / (w * cos(theta)).
        damping_angle_deg: Pole-pair angle `theta`, in degrees, measured away
            from the negative real axis. Small angles put poles closer to the
            real axis, making the observer more damped/less ringy. Large angles
            put poles closer to the imaginary axis, making it oscillate more and
            reject noise less. The paper suggests roughly 40..50 deg.
        real_pole_rad_s: Continuous-time real pole `p0`. If omitted, this uses
            `bandwidth_rad_s`, matching the old script. Keep `p0 ~= w` while
            doing first-pass tuning; move it only if the acceleration estimate
            is specifically too slow or too noisy.

    Tuning notes:
        - Direct-drive gear_ratio=1 exposes rotor velocity noise directly, so it
          usually needs lower `bandwidth_rad_s` than geared axes.
        - `g3` grows very quickly with bandwidth; if velocity feedback is noisy,
          reducing bandwidth is usually safer than changing only velocity_kp.
        - Recalculate gains whenever `sample_time_s` changes.
    """
    if sample_time_s <= 0.0:
        raise ValueError("sample_time_s must be positive")
    if bandwidth_rad_s <= 0.0:
        raise ValueError("bandwidth_rad_s must be positive")
    if not 0.0 < damping_angle_deg < 90.0:
        raise ValueError("damping_angle_deg must be in the open interval (0, 90)")

    if real_pole_rad_s is None:
        real_pole_rad_s = bandwidth_rad_s
    if real_pole_rad_s <= 0.0:
        raise ValueError("real_pole_rad_s must be positive")

    theta_rad = math.radians(damping_angle_deg)

    # Map desired continuous-time observer poles to the z-plane.
    #
    #   real pole:        s = -p0
    #   complex pole pair s = -w*cos(theta) +- j*w*sin(theta)
    #
    # The resulting discrete characteristic polynomial is:
    #
    #   (z - ro0) * (z^2 - 2*ro1*cos(phi)*z + ro1^2)
    #     = z^3 + c2*z^2 + c1*z + c0
    ro0 = math.exp(-real_pole_rad_s * sample_time_s)
    ro1 = math.exp(-bandwidth_rad_s * sample_time_s * math.cos(theta_rad))
    phi = bandwidth_rad_s * sample_time_s * math.sin(theta_rad)

    c2 = -2.0 * ro1 * math.cos(phi) - ro0
    c1 = ro1 * (2.0 * ro0 * math.cos(phi) + ro1)
    c0 = -ro0 * ro1 * ro1

    # Paper gain form. l1/l2/l3 correct angle, speed, and acceleration.
    l1 = c2 + 3.0
    l2 = (c1 - c0 - 4.0 + 3.0 * l1) / (2.0 * sample_time_s)
    l3 = (c1 + c0 - 2.0 + l1) / (sample_time_s * sample_time_s)

    # Firmware gain form. These match the integration order used in
    # FOC::apply_kalman(), where the residual is injected after prediction.
    g3 = l3
    g2 = l2 - sample_time_s * g3
    g1 = l1 - sample_time_s * g2 - g3 * sample_time_s * sample_time_s / 2.0

    return g1, g2, g3


parser = argparse.ArgumentParser(
    description="Calculate g1/g2/g3 observer gains for VBDrive FOC.",
    formatter_class=argparse.ArgumentDefaultsHelpFormatter,
)
parser.add_argument(
    "--sample-time-us",
    type=float,
    default=DEFAULT_SAMPLE_TIME_S * 1e6,
    help="FOC observer sample time in microseconds. Must match firmware T.",
)
parser.add_argument(
    "--bandwidth",
    type=float,
    default=DEFAULT_BANDWIDTH_RAD_S,
    help=(
        "Observer response speed w in rad/s. Lower is smoother/laggier; "
        "higher is faster/noisier."
    ),
)
parser.add_argument(
    "--damping-angle",
    type=float,
    default=DEFAULT_DAMPING_ANGLE_DEG,
    help=(
        "Complex-pole angle theta in degrees. Lower is more damped; higher is "
        "more ringy/noise-sensitive."
    ),
)
parser.add_argument(
    "--real-pole",
    type=float,
    default=None,
    help="Observer real pole p0 in rad/s. Defaults to the same value as w.",
)
parser.add_argument(
    "--check",
    action="store_true",
    help="Verify the firmware defaults and the Bellini paper example.",
)

CALCULATION_FLAGS = {
    "--sample-time-us",
    "--bandwidth",
    "--damping-angle",
    "--real-pole",
}


if __name__ == "__main__":
    if len(sys.argv) == 1:
        parser.print_help()
        raise SystemExit(0)

    args = parser.parse_args()

    if args.check:
        checks = (
            (
                "firmware defaults",
                calculate_observer_gains(
                    sample_time_s=25e-6,
                    bandwidth_rad_s=250.0,
                    damping_angle_deg=40.0,
                    real_pole_rad_s=250.0,
                ),
                FIRMWARE_DEFAULT_GAINS,
            ),
            (
                "paper example",
                calculate_observer_gains(
                    sample_time_s=150e-6,
                    bandwidth_rad_s=1000.0,
                    damping_angle_deg=40.0,
                    real_pole_rad_s=1000.0,
                ),
                PAPER_EXAMPLE_GAINS,
            ),
        )
        for name, calculated, expected in checks:
            for calculated_gain, expected_gain in zip(calculated, expected):
                if not math.isclose(calculated_gain, expected_gain, rel_tol=1e-12, abs_tol=1e-12):
                    raise AssertionError(
                        f"{name} mismatch: calculated {calculated}, expected {expected}"
                    )
        print("Reference checks passed.")
        raise SystemExit(0)

    if not any(arg.split("=", 1)[0] in CALCULATION_FLAGS for arg in sys.argv[1:]):
        parser.print_help()
        raise SystemExit(0)

    gains = calculate_observer_gains(
        sample_time_s=args.sample_time_us * 1e-6,
        bandwidth_rad_s=args.bandwidth,
        damping_angle_deg=args.damping_angle,
        real_pole_rad_s=args.real_pole,
    )

    print(f"g1 = {gains[0]:.17g}")
    print(f"g2 = {gains[1]:.17g}")
    print(f"g3 = {gains[2]:.17g}")
