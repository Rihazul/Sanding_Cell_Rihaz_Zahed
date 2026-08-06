"""Simultaneity demo: 7th axis (J7) and robot arm moving at the same time.

Runs on Table B (UCS Plane_2) with the tool 3 Plane 2 TCP so the demo can be done
with the arm holding tool 3.

MOTION ONLY - no force control, no vibration. Everything happens at a safe
standoff height above the table, so the tool never touches the surface.

Shape of the demo
-----------------
The work is laid out as a list of STATIONS. Each station is one 7th-axis position
paired with its own set of Plane_2 points, and the arm traces those points like a
short sanding pass (small hops with a brief dwell at each point) so an observer can
see the arm working.

The interesting part is the TRANSIT between stations. Moving from one station to
the next means J7 must travel AND the arm must reposition to the next point set:

  SEQUENTIAL    J7 moves with wait=True, then the arm moves. Times add up.
  SIMULTANEOUS  J7 moves with wait=False (returns as soon as the move is accepted),
                then the arm moves immediately. Both travel together, so the transit
                costs roughly the slower axis instead of the sum.

`communicate` provides the interlock for the simultaneous case: an async J7 move
sets `_j7_async_pending`, and the next waited point move blocks on
`waitForSeventhAxisIdle` before returning. The arm is free to fly during J7 travel,
but the demo never starts a station's pass until J7 has actually settled. This
mirrors the proven path in table_b_dxf/execution/xy_force_runner.py.

By default the script just runs the SIMULTANEOUS sequence once, straight through -
that is the version worth filming. Pass --mode both if you also want the sequential
baseline for a timing comparison.

Usage:
    python test_simultaneous_j7_arm.py                  # the show: one simultaneous run
    python test_simultaneous_j7_arm.py --loop 3         # repeat for longer footage
    python test_simultaneous_j7_arm.py --countdown 10   # more time to start recording
    python test_simultaneous_j7_arm.py --dwell 1.5      # longer pause at each point
    python test_simultaneous_j7_arm.py --passes 2       # repeat each station's pass
    python test_simultaneous_j7_arm.py --mode both      # + sequential baseline, for timings
    python test_simultaneous_j7_arm.py --dry-run        # print the plan, no motion
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

# Table B pose frame, matching table_b_dxf/execution/motion_config.py.
ORIENTATION = [180.0, 0.0, 0.0]
# Safe standoff height above the Table B surface. The DXF runner works at -17 mm
# pre-height, so -60 keeps the tool clearly in the air for this motion-only demo.
DEMO_Z_MM = -60.0

# The demo sequence: each station is one 7th-axis position plus the Plane_2 points
# the arm traces there. The points are small XY hops (a few cm) so each station
# reads as a short "sanding pass" rather than a big traverse - the big motion is
# meant to be the J7 transit between stations.
#
# XY values are in the Plane_2 frame; Z and orientation are held constant.
STATIONS = [
    {
        "name": "Station 1 (near)",
        "j7": 300.0,
        "points": [
            [150.0, 150.0],
            [260.0, 150.0],
            [260.0, 230.0],
            [150.0, 230.0],
        ],
    },
    {
        "name": "Station 2 (mid)",
        "j7": 750.0,
        "points": [
            [200.0, 260.0],
            [320.0, 260.0],
            [320.0, 340.0],
            [200.0, 340.0],
        ],
    },
    {
        "name": "Station 3 (far)",
        "j7": 1200.0,
        "points": [
            [260.0, 180.0],
            [380.0, 180.0],
            [380.0, 260.0],
            [260.0, 260.0],
        ],
    },
    {
        "name": "Station 4 (return)",
        "j7": 550.0,
        "points": [
            [180.0, 300.0],
            [300.0, 300.0],
            [300.0, 380.0],
            [180.0, 380.0],
        ],
    },
]


def load_config():
    """Loads configuration from config.yaml."""
    config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "configs", "config.yaml")
    with open(config_path, "r") as file:
        return yaml.safe_load(file)


def pose(xy, z, rxyz):
    return [float(xy[0]), float(xy[1]), float(z), rxyz[0], rxyz[1], rxyz[2]]


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
    """Hold position for a moment so the demo is easy to follow.

    Sleeps in slices so an emergency stop is noticed promptly instead of only
    after the full dwell has elapsed.
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
        raise RuntimeError(f"7th-axis move to {target} failed (require_seventh_ok).")


def move_arm(cps, config, point, tcp, ucs, speed, profile="robotspeed"):
    """Command an arm-only MoveL (seventh=-1) and wait for it.

    When a preceding J7 move was started async, `communicate` also blocks here
    until J7 reports idle, so a transit is not done until BOTH axes are.
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


def run_station_pass(cps, config, station, *, tcp, ucs, speed, z, dwell_s, passes):
    """Trace this station's point set like a short sanding pass (no force, no vibration)."""
    points = station["points"]
    for pass_index in range(passes):
        # Alternate direction on repeat passes so it looks like back-and-forth work.
        ordered = points if pass_index % 2 == 0 else list(reversed(points))
        for point_index, xy in enumerate(ordered, start=1):
            abort_if_stopped(config, f"{station['name']} pass")
            config["logger"].info(
                "[simul-demo] %s | pass %s/%s point %s/%s -> %s",
                station["name"],
                pass_index + 1,
                passes,
                point_index,
                len(ordered),
                xy,
            )
            move_arm(cps, config, pose(xy, z, ORIENTATION), tcp, ucs, speed, profile="sandingspeed")
            dwell(config, dwell_s, f"{station['name']} dwell")
    waitForBlending(cps=cps, config=config)


def run_transit(cps, config, station, *, simultaneous, tcp, ucs, speed, z, label):
    """Move J7 to this station AND the arm to its first point.

    Returns elapsed wall-clock seconds, which is what the two modes are compared on.
    """
    abort_if_stopped(config, f"{label} start")

    j7_target = station["j7"]
    first_point = pose(station["points"][0], z, ORIENTATION)
    j7_before = read_j7_position(cps)
    config["logger"].info(
        "[simul-demo] %s | mode=%s j7: %s -> %s | arm -> %s",
        label,
        "SIMULTANEOUS" if simultaneous else "SEQUENTIAL",
        f"{j7_before:.1f}" if j7_before is not None else "?",
        j7_target,
        station["points"][0],
    )

    start = time.time()
    if simultaneous:
        # Fire J7 and do NOT wait, so the arm move below overlaps its travel.
        move_j7(cps, config, j7_target, tcp, ucs, speed, wait=False)
        dispatch_s = time.time() - start
        move_arm(cps, config, first_point, tcp, ucs, speed)
        config["logger"].info(
            "[simul-demo] %s | J7 dispatch returned in %.3fs (arm moved during J7 travel)",
            label,
            dispatch_s,
        )
    else:
        # Baseline: J7 fully completes before the arm starts.
        move_j7(cps, config, j7_target, tcp, ucs, speed, wait=True)
        j7_done_s = time.time() - start
        move_arm(cps, config, first_point, tcp, ucs, speed)
        config["logger"].info(
            "[simul-demo] %s | J7 finished alone in %.3fs, then the arm moved",
            label,
            j7_done_s,
        )

    elapsed = time.time() - start
    abort_if_stopped(config, f"{label} end")
    config["logger"].info("[simul-demo] %s | transit done in %.3fs", label, elapsed)
    return elapsed


def run_mode(cps, config, *, simultaneous, tcp, ucs, speed, z, dwell_s, passes, stations):
    """Walk every station: transit to it (timed), then trace its point set."""
    mode_name = "SIMULTANEOUS" if simultaneous else "SEQUENTIAL"
    print(f"\n=== {mode_name} : {len(stations)} stations ===")

    transit_times = []
    for index, station in enumerate(stations, start=1):
        label = f"transit {index}/{len(stations)} -> {station['name']}"
        elapsed = run_transit(
            cps, config, station,
            simultaneous=simultaneous,
            tcp=tcp, ucs=ucs, speed=speed, z=z,
            label=label,
        )
        transit_times.append(elapsed)
        print(f"  {station['name']:<20} j7={station['j7']:>7.1f}  transit {elapsed:.3f}s")

        run_station_pass(
            cps, config, station,
            tcp=tcp, ucs=ucs, speed=speed, z=z,
            dwell_s=dwell_s, passes=passes,
        )

    average = sum(transit_times) / len(transit_times) if transit_times else 0.0
    print(f"{mode_name}: {len(transit_times)} transits, average {average:.3f}s per transit")
    return average


def parse_args():
    parser = argparse.ArgumentParser(
        description="7th axis + arm simultaneity demo (Plane_2 / tool3plane2). Motion only - no force, no vibration.",
    )
    parser.add_argument(
        "--mode",
        choices=("sim", "seq", "both"),
        default="sim",
        help=(
            "sim = one continuous simultaneous run, best for filming (default). "
            "seq = sequential only. both = sequential baseline then simultaneous, for timing comparison."
        ),
    )
    parser.add_argument("--dwell", type=float, default=1.0, help="Seconds to hold at each point (default 1.0).")
    parser.add_argument("--passes", type=int, default=1, help="Passes over each station's point set (default 1).")
    parser.add_argument("--speed", type=float, default=None, help="Speed ratio override (default: UI robotSpeed).")
    parser.add_argument("--z", type=float, default=DEMO_Z_MM, help=f"Demo Z height in Plane_2 (default {DEMO_Z_MM}).")
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
    parser.add_argument("--dry-run", action="store_true", help="Print the plan and exit without connecting or moving.")
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


def print_plan(stations, z):
    print("\nDemo sequence:")
    for index, station in enumerate(stations, start=1):
        points = ", ".join(f"({xy[0]:.0f},{xy[1]:.0f})" for xy in station["points"])
        print(f"  {index}. {station['name']:<20} j7={station['j7']:>7.1f}  points: {points}")
    print(f"  (all points at Z={z} with orientation {ORIENTATION})")


def main():
    args = parse_args()
    config = load_config()
    config["logger"] = setup_logger(config["settings"]["debug"])

    tcp = config["coords"]["tcptool3plane2"]
    ucs = config["coords"]["ucsTable2"]
    speed = args.speed if args.speed is not None else float(config["UI"]["robotSpeed"])

    print("=== 7th Axis + Arm Simultaneity Demo (motion only) ===")
    print(f"TCP           : {tcp}")
    print(f"UCS           : {ucs}")
    print(f"Speed ratio   : {speed}")
    print(f"Mode          : {args.mode}   dwell={args.dwell}s   passes={args.passes}   loop={args.loop}")
    print("Force control : DISABLED    Vibration: DISABLED")
    print_plan(STATIONS, args.z)

    if args.dry_run:
        print("\nDry run - no connection made, no motion commanded.")
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
                    tcp=tcp, ucs=ucs, speed=speed, z=args.z,
                    dwell_s=args.dwell, passes=args.passes, stations=STATIONS,
                )
            if args.mode in ("both", "sim"):
                simultaneous_avg = run_mode(
                    cps, config,
                    simultaneous=True,
                    tcp=tcp, ucs=ucs, speed=speed, z=args.z,
                    dwell_s=args.dwell, passes=args.passes, stations=STATIONS,
                )

        print("\n=== Sequence complete - you can stop recording ===")
        if sequential_avg is not None:
            print(f"Sequential   : {sequential_avg:.3f}s per transit")
        if simultaneous_avg is not None:
            print(f"Simultaneous : {simultaneous_avg:.3f}s per transit")
        if sequential_avg and simultaneous_avg:
            saved = sequential_avg - simultaneous_avg
            percent = (saved / sequential_avg * 100.0) if sequential_avg else 0.0
            print(f"Saved        : {saved:.3f}s per transit ({percent:.1f}% faster)")
            print("A clear saving means J7 and the arm really did travel together.")
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
