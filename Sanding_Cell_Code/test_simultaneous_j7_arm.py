"""Simultaneity demo: 7th axis (J7) and robot arm covering the Table B XY plane.

Runs on Table B (UCS Plane_2) with the tool 3 Plane 2 TCP so the demo can be done
with the arm holding tool 3.

MOTION ONLY - no force control, no vibration. Everything happens at a safe
standoff height above the table, so the tool never touches the surface.

The coordinate idea this demo is built on
-----------------------------------------
Table B is ~2000 mm in X and ~900 mm in Y, but the arm alone can only reach a
window of roughly 700 mm in X. The 7th axis carries the robot BASE along X, and
the base is the origin the arm's points are measured from - so when J7 moves, the
whole reachable envelope SLIDES with it.

That means a target written in fixed, global table coordinates has to have the
7th-axis position SUBTRACTED out before it is sent to the arm:

    local_x = global_x - j7        (Y is untouched - the rail only translates X)

This is the same conversion the production planner does, see
`_robot_base_points` in table_b_dxf/executor.py.

That subtraction is what makes the two axes genuinely coordinated rather than
merely moving at the same time. Every waypoint here is authored in GLOBAL table
coordinates - a real position on the 2000 x 900 table. For each one the demo picks
a legal J7 station, converts the point into the arm's local frame, and then drives
BOTH at once. The tool tracks a straight global line across the whole table while
the rail slides underneath it and the arm continuously re-aims to compensate.

The visible result: the tool traces one long, straight, continuous path across the
full 2 m table - something neither axis could do alone.

How the two axes overlap
------------------------
`communicate` with wait=False dispatches J7 and returns immediately, so the arm
MoveL that follows travels at the same time. The async J7 sets `_j7_async_pending`,
and the next waited point move blocks on `waitForSeventhAxisIdle`, so the demo
never advances to the next waypoint until BOTH axes have settled. Same proven path
as table_b_dxf/execution/xy_force_runner.py.

Every waypoint is validated against the REAL tool 3 reach envelope
(table_b_dxf/tool_reach.py) before anything moves, so an out-of-reach point is
reported up front instead of faulting mid-demo.

Usage:
    python test_simultaneous_j7_arm.py                  # the show: one simultaneous run
    python test_simultaneous_j7_arm.py --dry-run        # print plan + reach check, no motion
    python test_simultaneous_j7_arm.py --loop 3         # repeat for longer footage
    python test_simultaneous_j7_arm.py --countdown 10   # more time to start recording
    python test_simultaneous_j7_arm.py --mode both      # + sequential baseline, for timings
"""

import argparse
import os
import sys
import time

import yaml

sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from modules.CPS import CPSClient
from Server_Better_V2 import (
    clear_stop,
    communicate,
    setup_logger,
    stop_requested,
    waitForBlending,
)
from table_b_dxf.tool_reach import (
    AXIS7_MAX_MM,
    AXIS7_MIN_MM,
    reach_span_at_y,
    station_window_for_point,
)

# Tool 3 is what the demo is filmed with; its reach envelope gates every waypoint.
REACH_TOOL = "tool_3"

# Table B pose frame, matching table_b_dxf/execution/motion_config.py.
ORIENTATION = [180.0, 0.0, 0.0]
# Safe standoff height above the Table B surface. The DXF runner works at -17 mm
# pre-height, so -60 keeps the tool clearly in the air for this motion-only demo.
DEMO_Z_MM = -60.0

# Usable extent of the Table B XY plane for this demo, in GLOBAL table coordinates.
TABLE_X_MM = 2000.0
TABLE_Y_MM = 900.0


def _sweep(y, x_start, x_end, count):
    """Waypoints along one straight global line in X - the full-table traverse."""
    if count < 2:
        return [[float(x_start), float(y)]]
    step = (float(x_end) - float(x_start)) / (count - 1)
    return [[float(x_start) + step * i, float(y)] for i in range(count)]


# The demo route, in GLOBAL Table B coordinates (X across the 2 m length, Y across
# the 900 mm width). No station numbers here on purpose: the J7 position for each
# waypoint is COMPUTED from the geometry, which is the whole point of the demo.
#
# Legs 1 and 3 are long full-table sweeps - these are the money shots, where the
# rail and the arm visibly move together to hold a straight line. Leg 2 is a short
# Y step at the far end, and leg 4 a mid-table zigzag, to break up the motion.
DEMO_LEGS = [
    {
        "name": "Sweep out (low Y)",
        "points": _sweep(y=250.0, x_start=200.0, x_end=1800.0, count=9),
    },
    {
        "name": "Cross over (far end)",
        "points": [[1800.0, 250.0], [1800.0, 450.0], [1650.0, 450.0]],
    },
    {
        "name": "Sweep back (mid Y)",
        "points": _sweep(y=450.0, x_start=1650.0, x_end=250.0, count=9),
    },
    {
        "name": "Zigzag (mid table)",
        "points": [
            [250.0, 450.0],
            [500.0, 620.0],
            [800.0, 450.0],
            [1100.0, 620.0],
            [1400.0, 450.0],
        ],
    },
]


def load_config():
    """Loads configuration from config.yaml."""
    config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "configs", "config.yaml")
    with open(config_path, "r") as file:
        return yaml.safe_load(file)


def station_for_point(global_x, global_y, bias):
    """Pick a legal 7th-axis position for a global table point.

    `station_window_for_point` gives the full range of J7 positions from which the
    tool can reach this point. `bias` selects where inside that window to sit:
    0 = as far left as legal, 1 = as far right, 0.5 = centred.

    Returns (station_mm, local_x_mm) or None when the point is unreachable.
    """
    window = station_window_for_point(REACH_TOOL, float(global_x), float(global_y))
    if window is None:
        return None
    low, high = window
    # Constrain to the rail's real travel before choosing inside the window.
    low = max(low, AXIS7_MIN_MM)
    high = min(high, AXIS7_MAX_MM)
    if low > high:
        return None
    bias = min(1.0, max(0.0, float(bias)))
    station = low + (high - low) * bias
    return station, float(global_x) - station


def validate_route(legs, base_bias=0.5, swing=0.10):
    """Resolve every waypoint to (station, local_x) and collect any unreachable ones.

    Beyond just finding a legal station, this deliberately makes the ARM move too.
    Centring in the reach window (a constant bias) is legal but dull: over a long
    straight sweep the window slides at the same rate as the target, so local_x
    goes CONSTANT and the rail ends up doing all the visible work.

    To avoid that, the bias is swung back and forth between waypoints. The global
    point is unchanged - it is still the exact same place on the table - but the
    work is re-divided between the two axes each step, so the arm reaches out and
    draws back while the rail advances. Both axes are then plainly in motion, which
    is the whole point of the footage.

    Returns (resolved_legs, problems). Doing this before connecting means a bad
    point is a printed message, never a fault partway through a take.
    """
    resolved = []
    problems = []
    for leg in legs:
        entries = []
        for index, (global_x, global_y) in enumerate(leg["points"]):
            if not (0.0 <= global_x <= TABLE_X_MM and 0.0 <= global_y <= TABLE_Y_MM):
                problems.append(
                    f"{leg['name']}: ({global_x:.0f}, {global_y:.0f}) is outside the "
                    f"{TABLE_X_MM:.0f} x {TABLE_Y_MM:.0f} demo area"
                )
                continue
            # Alternate the split so the arm visibly extends and retracts as J7 advances.
            bias = base_bias + (swing if index % 2 == 0 else -swing)
            solution = station_for_point(global_x, global_y, bias)
            if solution is None:
                span = reach_span_at_y(REACH_TOOL, global_y)
                detail = f"local span at Y={global_y:.0f} is {span}" if span else f"Y={global_y:.0f} outside envelope"
                problems.append(
                    f"{leg['name']}: ({global_x:.0f}, {global_y:.0f}) unreachable for {REACH_TOOL} ({detail})"
                )
                continue
            station, local_x = solution
            entries.append(
                {
                    "global": [global_x, global_y],
                    "station": station,
                    "local_x": local_x,
                    "y": global_y,
                }
            )
        resolved.append({"name": leg["name"], "entries": entries})
    return resolved, problems


def pose_from_entry(entry, z):
    """Arm pose in the base-local frame: X has the 7th-axis position subtracted out."""
    return [entry["local_x"], entry["y"], float(z), ORIENTATION[0], ORIENTATION[1], ORIENTATION[2]]


def read_j7_position(cps):
    """Best-effort J7 position read, for demo logging only."""
    state = []
    try:
        nret = cps.HRIF_HRApp(0, "HR_Motor", "MotorGetState", ["J7"], state)
    except Exception:
        return None
    if nret not in (0, None):
        return None
    for value in state:
        try:
            number = float(value)
        except (TypeError, ValueError):
            continue
        # Skip the status flags; a plausible travel coordinate is what we want.
        if abs(number) > 5.0:
            return number
    return None


def abort_if_stopped(config, where):
    if stop_requested():
        config["logger"].warning("[simul-demo] Emergency stop requested during %s.", where)
        raise RuntimeError("Simultaneity demo stopped by emergency stop")


def dwell(config, seconds, where):
    """Hold position briefly so the demo is easy to follow.

    Sleeps in slices so an emergency stop is noticed promptly.
    """
    if seconds <= 0:
        return
    deadline = time.time() + seconds
    while time.time() < deadline:
        abort_if_stopped(config, where)
        time.sleep(min(0.1, max(0.0, deadline - time.time())))


def move_j7(cps, config, target, tcp, ucs, speed, wait):
    """Command the 7th axis. wait=False returns while J7 is still travelling."""
    result = communicate(
        cps=cps,
        config=config,
        point=None,
        tcp=tcp,
        ucs=ucs,
        seventh=target,
        speed=speed,
        velocity_profile="robotspeed",
        wait=wait,
        require_seventh_ok=True,
    )
    if result is None:
        raise RuntimeError(f"7th-axis move to {target:.1f} failed (require_seventh_ok).")


def move_arm(cps, config, point, tcp, ucs, speed, profile="robotspeed"):
    """Command an arm-only MoveL (seventh=-1) and wait for it.

    When a preceding J7 move was started async, `communicate` also blocks here
    until J7 reports idle, so a waypoint is not done until BOTH axes are.
    """
    communicate(
        cps=cps,
        config=config,
        point=point,
        tcp=tcp,
        ucs=ucs,
        seventh=-1,
        speed=speed,
        velocity_profile=profile,
        wait=True,
        require_seventh_ok=True,
    )


def go_to_waypoint(cps, config, entry, *, simultaneous, tcp, ucs, speed, z, label, dwell_s):
    """Drive J7 and the arm to one GLOBAL table point.

    The global point is held constant; what differs between modes is only whether
    the rail and the arm travel together or one after the other.
    """
    abort_if_stopped(config, label)

    station = entry["station"]
    arm_point = pose_from_entry(entry, z)

    config["logger"].info(
        "[simul-demo] %s | global=(%.0f, %.0f) -> j7=%.1f + local_x=%.1f",
        label,
        entry["global"][0],
        entry["global"][1],
        station,
        entry["local_x"],
    )

    start = time.time()
    if simultaneous:
        # Rail and arm travel together; the arm re-aims as the base slides under it.
        move_j7(cps, config, station, tcp, ucs, speed, wait=False)
        move_arm(cps, config, arm_point, tcp, ucs, speed, profile="sandingspeed")
    else:
        # Baseline: rail finishes first, then the arm moves. Visibly stop-start.
        move_j7(cps, config, station, tcp, ucs, speed, wait=True)
        move_arm(cps, config, arm_point, tcp, ucs, speed, profile="sandingspeed")

    elapsed = time.time() - start
    dwell(config, dwell_s, f"{label} dwell")
    return elapsed


def run_mode(cps, config, *, simultaneous, resolved, tcp, ucs, speed, z, dwell_s, passes):
    """Trace the whole global route, leg by leg."""
    mode_name = "SIMULTANEOUS" if simultaneous else "SEQUENTIAL"
    print(f"\n=== {mode_name} ===")

    total = 0.0
    count = 0
    for pass_index in range(passes):
        if passes > 1:
            print(f"-- pass {pass_index + 1}/{passes} --")
        for leg_index, leg in enumerate(resolved, start=1):
            entries = leg["entries"]
            if not entries:
                continue
            j7_span = (
                min(entry["station"] for entry in entries),
                max(entry["station"] for entry in entries),
            )
            print(
                f"  Leg {leg_index}: {leg['name']:<22} "
                f"{len(entries)} pts, j7 {j7_span[0]:.0f} -> {j7_span[1]:.0f} mm"
            )
            leg_start = time.time()
            for point_index, entry in enumerate(entries, start=1):
                total += go_to_waypoint(
                    cps, config, entry,
                    simultaneous=simultaneous,
                    tcp=tcp, ucs=ucs, speed=speed, z=z,
                    dwell_s=dwell_s,
                    label=f"leg {leg_index} pt {point_index}/{len(entries)}",
                )
                count += 1
            print(f"     leg took {time.time() - leg_start:.1f}s")
            waitForBlending(cps=cps, config=config)

    average = total / count if count else 0.0
    print(f"{mode_name}: {count} waypoints, {average:.3f}s average move time")
    return average


def parse_args():
    parser = argparse.ArgumentParser(
        description="7th axis + arm simultaneity demo over the Table B XY plane (Plane_2 / tool3plane2). Motion only.",
    )
    parser.add_argument(
        "--mode",
        choices=("sim", "seq", "both"),
        default="sim",
        help=(
            "sim = one continuous simultaneous run, best for filming (default). "
            "seq = sequential only. both = sequential baseline then simultaneous."
        ),
    )
    parser.add_argument("--dwell", type=float, default=0.3, help="Seconds to hold at each waypoint (default 0.3).")
    parser.add_argument("--passes", type=int, default=1, help="Times to trace the route per run (default 1).")
    parser.add_argument("--speed", type=float, default=None, help="Speed ratio override (default: UI robotSpeed).")
    parser.add_argument("--z", type=float, default=DEMO_Z_MM, help=f"Demo Z height in Plane_2 (default {DEMO_Z_MM}).")
    parser.add_argument(
        "--station-bias",
        type=float,
        default=0.5,
        help=(
            "Centre of the J7 window to work from: 0 = as far left as legal, "
            "1 = as far right, 0.5 = centred (default)."
        ),
    )
    parser.add_argument(
        "--swing",
        type=float,
        default=0.10,
        help=(
            "How much to alternate the work split between rail and arm at each waypoint. "
            "0 = none (arm goes still on long sweeps), 0.10 = balanced (default). "
            "Much above ~0.2 the rail starts thrashing back and forth between points."
        ),
    )
    parser.add_argument(
        "--countdown",
        type=float,
        default=5.0,
        help="Seconds to wait before the first move, so you can start recording (default 5).",
    )
    parser.add_argument(
        "--loop",
        type=int,
        default=1,
        help="Run the whole sequence this many times back-to-back, for longer footage (default 1).",
    )
    parser.add_argument("--dry-run", action="store_true", help="Print the plan and reach check, then exit without moving.")
    return parser.parse_args()


def countdown(config, seconds):
    """Give the camera operator time to start recording before anything moves."""
    if seconds <= 0:
        return
    remaining = int(seconds)
    while remaining > 0:
        abort_if_stopped(config, "countdown")
        print(f"  starting in {remaining}...", flush=True)
        time.sleep(1.0)
        remaining -= 1
    leftover = seconds - int(seconds)
    if leftover > 0:
        time.sleep(leftover)
    print("  GO\n", flush=True)


def print_plan(resolved, z):
    print("\nRoute (global table coords -> j7 + arm-local X):")
    for leg_index, leg in enumerate(resolved, start=1):
        print(f"  Leg {leg_index}: {leg['name']}")
        for entry in leg["entries"]:
            gx, gy = entry["global"]
            print(
                f"      global ({gx:7.1f}, {gy:6.1f})   j7 {entry['station']:7.1f}"
                f"   local_x {entry['local_x']:7.1f}"
            )
    print(f"  (all points at Z={z} with orientation {ORIENTATION})")


def main():
    args = parse_args()
    config = load_config()
    config["logger"] = setup_logger(config["settings"]["debug"])

    tcp = config["coords"]["tcptool3plane2"]
    ucs = config["coords"]["ucsTable2"]
    speed = args.speed if args.speed is not None else float(config["UI"]["robotSpeed"])

    print("=== Table B XY Simultaneity Demo (motion only) ===")
    print(f"TCP           : {tcp}")
    print(f"UCS           : {ucs}")
    print(f"Speed ratio   : {speed}")
    print(f"Table area    : {TABLE_X_MM:.0f} x {TABLE_Y_MM:.0f} mm   reach model: {REACH_TOOL}")
    print(f"Mode          : {args.mode}   dwell={args.dwell}s   passes={args.passes}   loop={args.loop}")
    print("Force control : DISABLED    Vibration: DISABLED")

    resolved, problems = validate_route(DEMO_LEGS, args.station_bias, args.swing)
    print_plan(resolved, args.z)

    if problems:
        print(f"\n{len(problems)} unreachable/out-of-area waypoint(s):")
        for problem in problems:
            print(f"  - {problem}")
        print("Fix DEMO_LEGS (or adjust --station-bias) before running on the robot.")
        return 2

    total_points = sum(len(leg["entries"]) for leg in resolved)
    stations = [entry["station"] for leg in resolved for entry in leg["entries"]]
    locals_x = [entry["local_x"] for leg in resolved for entry in leg["entries"]]
    # Sum of per-step motion on each axis. Both must be non-trivial, otherwise one
    # axis is coasting and the footage will not show simultaneity.
    rail_travel = sum(abs(b - a) for a, b in zip(stations, stations[1:]))
    arm_travel = sum(abs(b - a) for a, b in zip(locals_x, locals_x[1:]))
    print(
        f"\nReach check OK: {total_points} waypoints, "
        f"j7 range {min(stations):.0f} - {max(stations):.0f} mm (limit {AXIS7_MIN_MM:.0f} - {AXIS7_MAX_MM:.0f})."
    )
    print(
        f"Axis motion    : rail {rail_travel:.0f} mm total, arm local X {arm_travel:.0f} mm total "
        f"(both should be large - raise --swing if the arm figure is small)."
    )

    if args.dry_run:
        print("Dry run - no connection made, no motion commanded.")
        return 0

    # The demo moves the arm in free air; make sure nothing is left latched.
    clear_stop()

    cps = CPSClient()
    ip = config["server"]["cpip"]
    port = config["server"]["cps"]
    ret = cps.HRIF_Connect(0, ip, port)
    print(f"\nHRIF_Connect({ip}:{port}) -> ret={ret}")
    if ret != 0:
        print("Could not connect to the robot controller.")
        return 1

    sequential_avg = None
    simultaneous_avg = None
    try:
        print("\nReady. Start recording now.")
        countdown(config, args.countdown)

        for run_index in range(max(1, args.loop)):
            if args.loop > 1:
                print(f"\n########## RUN {run_index + 1}/{args.loop} ##########")

            if args.mode in ("both", "seq"):
                sequential_avg = run_mode(
                    cps, config,
                    simultaneous=False,
                    resolved=resolved,
                    tcp=tcp, ucs=ucs, speed=speed, z=args.z,
                    dwell_s=args.dwell, passes=args.passes,
                )
            if args.mode in ("both", "sim"):
                simultaneous_avg = run_mode(
                    cps, config,
                    simultaneous=True,
                    resolved=resolved,
                    tcp=tcp, ucs=ucs, speed=speed, z=args.z,
                    dwell_s=args.dwell, passes=args.passes,
                )

        print("\n=== Sequence complete - you can stop recording ===")
        if sequential_avg is not None:
            print(f"Sequential   : {sequential_avg:.3f}s per waypoint")
        if simultaneous_avg is not None:
            print(f"Simultaneous : {simultaneous_avg:.3f}s per waypoint")
        if sequential_avg and simultaneous_avg:
            saved = sequential_avg - simultaneous_avg
            percent = (saved / sequential_avg * 100.0) if sequential_avg else 0.0
            print(f"Saved        : {saved:.3f}s per waypoint ({percent:.1f}% faster)")
        return 0
    except RuntimeError as error:
        print(f"\nDemo aborted: {error}")
        return 1
    except KeyboardInterrupt:
        print("\nDemo interrupted.")
        return 1
    finally:
        cps.HRIF_DisConnect(0)
        print("Disconnected.")


if __name__ == "__main__":
    raise SystemExit(main())
