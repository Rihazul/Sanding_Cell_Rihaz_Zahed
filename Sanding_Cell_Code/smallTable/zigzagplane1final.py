# testcommu.py

import sys
import math
import os
import time
import json
import uuid
from typing import Optional

# Add parent directory to path so Python can find the modules
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import yaml
import threading
from Components.RobotState import RobotState
from Server_Better_V2 import (
    communicate,
    setup_logger,
    waitForBlending,
    setSpeed,
    turn_vibration_on,
    turn_vibration_off,
    putForce,
    releaseForce,
    putForceZplus,
    putForceZminus,
    stop_requested,
)
from smallTable.waypoint2 import (
    Waypoint2Config,
    execute_waypoint2_path,
    generate_spiral_points_between,
    generate_arc_line_segments_between,
)
from modules.CPS import CPSClient  # Ensure CPSClient is properly defined
from smallTable.scancord import (
    read_scan_results,
    get_door_position,
    get_frame_point,
    get_inner_corner_point,
    get_outer_corner_point,
    get_pocket_point,
)


def compute_timeout(
    *,
    total_points: Optional[int] = None,
    cap: float = 180.0,
    velocity: float = 300.0,
) -> float:
    """
    Estimate a completion timeout based on total points.

    - Uses a simple model: time = (points / vmax) + overhead
    - Caps the timeout to avoid excessive waits.
    """
    if total_points is None:
        return cap

    distance_per_point = 4.64
    # vmax = 300.0
    # time_factor_max = float(total_points / vmax)
    time_factor = float(total_points * distance_per_point / (velocity))
    # Keep timeout aligned to the estimated runtime from points/speed.
    timeout = min(cap, max(1.0, time_factor))
    print(
        f"[Timeout] total_points={total_points}, est={time_factor:.1f}s timeout={timeout:.1f}s"
    )
    return timeout


DEFAULT_ORIENTATION_TOL = 1.0
# Optional forced orientation (Rx, Ry, Rz in degrees) to keep tool normal
# perpendicular to the sanding plane. When None, we lock to the actual
# orientation measured under force control.
FORCE_SPIRAL_ORIENT = None


def _read_act_coord_pose(cps, tcp: str, ucs: str) -> Optional[list]:
    result = []
    ret = cps.HRIF_ReadActCoord(0, 0, tcp, ucs, result)
    if ret == 0 and len(result) >= 6:
        try:
            return [float(v) for v in result[:6]]
        except (TypeError, ValueError):
            pass
    result = []
    ret = cps.HRIF_ReadActPos(0, 0, result)
    if ret != 0 or len(result) < 12:
        return None
    try:
        return [float(v) for v in result[6:12]]
    except (TypeError, ValueError):
        return None


def _capture_locked_orient(
    cps,
    config,
    *,
    tcp: str,
    ucs: str,
    fallback_orient,
    settle_s: float = 0.05,
):
    time.sleep(max(0.0, float(settle_s)))
    actual = _read_act_coord_pose(cps, tcp, ucs)
    logger = config.get("logger") if config else None
    if actual and len(actual) >= 6:
        orient = list(actual[3:6])
        msg = f"[Orientation] locked to actual under force: {orient}"
        if logger:
            logger.info(msg)
        else:
            print(msg)
        return orient
    msg = f"[Orientation] failed to read actual; using fallback {fallback_orient}"
    if logger:
        logger.warning(msg)
    else:
        print(msg)
    return list(fallback_orient)

DEFAULT_SPIRAL_LINEAR_SPEED = 200.0
DEFAULT_SPIRAL_RADIUS = 12.0
DEFAULT_MOVEJS_JERK_RATIO = 100
DEFAULT_MOVEJS_TRANSITION_DEG = 25
MOVEJS_MAX_POINTS = 50



def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def _linear_speed_to_turns(speed_mm_s: float) -> int:
    """Map 100–300 mm/s to 20–10 turns (faster speed -> fewer turns)."""
    speed = _clamp(speed_mm_s, 100.0, 300.0)
    turns = 20.0 - (speed - 100.0) * (10.0 / 200.0)
    print(turns)
    return int(round(_clamp(turns, 10.0, 20.0)))


def apply_spiral_settings(settings: Optional[dict]) -> None:
    """Apply incoming spiral settings to defaults used during spiral generation."""
    global DEFAULT_SPIRAL_LINEAR_SPEED, DEFAULT_SPIRAL_RADIUS
    if not settings:
        return
    # Handle dataclass or plain dict
    radius_val = (
        getattr(settings, "radius_mm", None)
        if not isinstance(settings, dict)
        else settings.get("radius_mm")
    )
    linear_val = (
        getattr(settings, "linear_speed_mm_s", None)
        if not isinstance(settings, dict)
        else settings.get("linear_speed_mm_s")
    )
    DEFAULT_SPIRAL_LINEAR_SPEED = _clamp(
        float(linear_val if linear_val is not None else DEFAULT_SPIRAL_LINEAR_SPEED),
        100.0,
        300.0,
    )
    DEFAULT_SPIRAL_RADIUS = _clamp(
        float(radius_val if radius_val is not None else DEFAULT_SPIRAL_RADIUS),
        10.0,
        15.0,
    )


def generate_spiral_between_points(
    start_pose,
    end_pose,
    turns: Optional[int] = 12,
    radius: Optional[float] = None,
    angle_step_deg: float = 120.0,
    max_points: Optional[int] = None,
    orientation: str = "horizontal",
):
    """
    Build a spiral path between two cartesian poses (X, Y, Z, Rx, Ry, Rz).
    Keeps orientation from start_pose, interpolates center XY between poses,
    and adds radial offsets for each step.
    """
    x0, y0, z0, rx, ry, rz = start_pose[:6]
    x1, y1, _, _, _, _ = end_pose[:6]
    if radius is None:
        radius = DEFAULT_SPIRAL_RADIUS
    if y0 == y1 and orientation == "vertical":
        turns = 1
    elif x0 == x1 and orientation == "horizontal":
        turns = 1
    else:
        dist = max(abs(x1 - x0), abs(y1 - y0))
        # factor = 1.0
        # if orientation == "vertical":
        #     factor= 1.3
        # turns = int((4.0/260.0) * float(dist) * float(factor))
        turns = math.ceil(dist/(radius*2.0))

    # Always map speed -> turns so higher speed yields fewer turns

    total_steps = int(turns * (360.0 / angle_step_deg))
    if max_points is not None:
        total_steps = max(1, min(total_steps, int(max_points)))
    else:
        total_steps = max(1, total_steps)
    points = list(start_pose[:6])  # start point

    for step in range(1, total_steps + 1):
        t = step / total_steps
        cx = x0 + (x1 - x0) * t
        cy = y0 + (y1 - y0) * t
        theta_deg = step * angle_step_deg
        theta = math.radians(theta_deg)

        px = cx + radius * math.cos(theta)
        py = cy + radius * math.sin(theta)
        pz = z0  # hold Z while spiraling along Y

        points.extend([px, py, pz, rx, ry, rz])

    return points


def run_spiral_between_points(
    cps: CPSClient,
    config: dict,
    start_pose,
    end_pose,
    *,
    turns: Optional[int] = 12,
    radius: Optional[float] = None,
    angle_step_deg: float = 120.0,
    max_points: Optional[int] = None,
    tcp: str = None,
    ucs: str = None,
    track_name: str = None,
    velocity: float = 300.0,
    accel: float = 500.0,
    jerk: float = 10000.0,
    box_id: int = 0,
    robot_id: int = 0,
    init_path: bool = True,
    orientation: str = "horizontal",
):
    """Push a spiral MovePathL between two poses.

    By default (`finalize=False`), the path is only pushed (optionally initialized
    with `init_path`) and execution must be triggered separately via
    `finalize_spiral_path`. Set `finalize=True` to push and execute immediately.
    """
    track_name = track_name
    print(f"[Spiral] Using track name: {track_name}")
    tcp_name = tcp or config["coords"].get("tcptool3plane1")
    ucs_name = ucs or config["coords"].get("ucsTable1")

    all_points = generate_spiral_between_points(
        start_pose=start_pose,
        end_pose=end_pose,
        turns=turns,
        radius=DEFAULT_SPIRAL_RADIUS if radius is None else radius,
        angle_step_deg=angle_step_deg,
        max_points=max_points,
        orientation=orientation,
    )
    xs = all_points[0::6]
    ys = all_points[1::6]
    print("spiral x range:", min(xs), max(xs), " y range:", min(ys), max(ys))

    if len(all_points) % 6 != 0:
        print("[Spiral] Point list misaligned (not divisible by 6).")
        return False, None

    count = len(all_points) // 6
    print(f"[Spiral] Total points = {count}")

    if init_path:
        # Raw data type = 1 (PCS), speed ratio = 1.0, blending radius = 0.0
        ret = cps.HRIF_InitPath(
            box_id,
            robot_id,
            1,
            track_name,
            0.50,
            1.00,
            velocity,
            accel,
            jerk,
            ucs_name,
            tcp_name
        )
        print("[Spiral][Init] ret =", ret)
        if ret != 0:
            return False, None

    ret = cps.HRIF_PushPathPoints(box_id, robot_id, track_name, all_points)
    print("[Spiral][Push] ret =", ret)
    if ret != 0:
        return False, None

    return True, count


def finalize_spiral_path(
    cps: CPSClient,
    track_name: str,
    *,
    box_id: int = 0,
    robot_id: int = 0,
    completion_timeout=76.0,
    min_runtime_s: float = 0.0,
    force: Optional[float] = None,
    config: Optional[dict] = None,
    force_settle_s: float = 0.2,
    manage_force_cycle: bool = True,
    expected_start_pose: Optional[list] = None,
    start_pos_tol_mm: float = 0.8,
    start_ori_tol_deg: float = 1.2,
    delete_path_after: bool = True,
) -> bool:
    """
    3.19 flow:
      EndPushPathPoints -> ReadPathState(ready=3) -> MovePathL -> monitor with
      ReadTrackProcess + IsMotionDone (+ flags as fallback).
    """
    ret = cps.HRIF_EndPushPathPoints(box_id, robot_id, track_name)
    print("[Spiral][EndPush] ret =", ret)
    if ret != 0:
        return False

    robot_state = RobotState(config=config, cps_client=cps)

    # Optional telemetry from 3.19 interface to help diagnose state transitions.
    path_info = []
    info_ret = cps.HRIF_ReadPathInfo(box_id, robot_id, track_name, path_info)

    def _try_parse_pose(values) -> Optional[list]:
        if values is None:
            return None
        if isinstance(values, (list, tuple)):
            if len(values) < 6:
                return None
            try:
                return [float(v) for v in values[:6]]
            except (TypeError, ValueError):
                return None
        if isinstance(values, str):
            normalized = values
            for sep in ("[", "]", "(", ")", ";", "/", "|"):
                normalized = normalized.replace(sep, ",")
            parts = [p.strip() for p in normalized.split(",") if p.strip()]
            if len(parts) < 6:
                return None
            try:
                return [float(v) for v in parts[:6]]
            except (TypeError, ValueError):
                return None
        return None

    def _parse_path_info(info: list) -> dict:
        """
        Parse HRIF_ReadPathInfo payload according manufacturer mapping.
        Supports both:
        - compact field mode (coords packed as strings)
        - comma-expanded mode caused by SDK split(',') behavior
        """
        parsed = {
            "raw_type": None,
            "status_l": None,
            "error_l": None,
            "point_count": None,
            "first_pose": None,
            "mode": "unknown",
        }
        if not info:
            return parsed

        # Compact form from docs:
        # [0..9] scalars, [10] ucs, [11] tcp, [12] points, [13] first point
        if len(info) >= 14:
            parsed["raw_type"] = str(info[0]).strip()
            parsed["status_l"] = str(info[3]).strip() if len(info) > 3 else None
            parsed["error_l"] = str(info[4]).strip() if len(info) > 4 else None
            try:
                parsed["point_count"] = int(float(str(info[12]).strip()))
            except (TypeError, ValueError):
                parsed["point_count"] = None
            parsed["first_pose"] = _try_parse_pose(info[13] if len(info) > 13 else None)
            parsed["mode"] = "compact"

        # Expanded mode:
        # [0..9] scalars + 6 ucs + 6 tcp + [22] points + [23..28] first pose
        if len(info) >= 29:
            parsed["raw_type"] = str(info[0]).strip()
            parsed["status_l"] = str(info[3]).strip() if len(info) > 3 else None
            parsed["error_l"] = str(info[4]).strip() if len(info) > 4 else None
            try:
                parsed["point_count"] = int(float(str(info[22]).strip()))
            except (TypeError, ValueError):
                parsed["point_count"] = None
            parsed["first_pose"] = _try_parse_pose(info[23:29])
            parsed["mode"] = "expanded"

        return parsed

    path_info_parsed = _parse_path_info(path_info)
    if info_ret == 0:
        print(
            "[Spiral][PathInfo] mode={} len={} raw_type={} stateL={} errL={} points={}".format(
                path_info_parsed.get("mode"),
                len(path_info),
                path_info_parsed.get("raw_type"),
                path_info_parsed.get("status_l"),
                path_info_parsed.get("error_l"),
                path_info_parsed.get("point_count"),
            )
        )

    def _read_path_l_state():
        st = []
        ret_local = cps.HRIF_ReadPathState(box_id, robot_id, track_name, st)
        l_state = str(st[2]).strip() if len(st) > 2 else None
        l_err = str(st[3]).strip() if len(st) > 3 else None
        return ret_local, st, l_state, l_err

    def _as_motion_done(value) -> bool:
        return (isinstance(value, bool) and value) or str(value).strip().lower() in (
            "1",
            "true",
            "ok",
        )

    # Wait until path is ready (state 3, error 0).
    ready_start = time.time()
    while True:
        if stop_requested():
            print("[Spiral] Stop requested while waiting for PATH_READY; aborting segment.")
            return False
        ret, st, l_state, l_err = _read_path_l_state()
        if ret == 0 and l_err == "0" and l_state == "3":
            break
        elapsed = time.time() - ready_start
        if elapsed > 120.0:
            print(f"[Spiral] Timeout waiting for PATH_READY after {elapsed:.1f}s:", st)
            return False
        time.sleep(0.05)

    # Move to trajectory start position before execution (manufacturer flow).
    first_pose = path_info_parsed.get("first_pose")
    start_pose_target = None
    if expected_start_pose is not None and len(expected_start_pose) >= 6:
        start_pose_target = list(expected_start_pose[:6])
        if first_pose is not None and len(first_pose) >= 6:
            try:
                ref_pos_err = max(abs(a - b) for a, b in zip(first_pose[:3], start_pose_target[:3]))
                ref_ori_err = max(abs(a - b) for a, b in zip(first_pose[3:6], start_pose_target[3:6]))
                if ref_pos_err > 50.0 or ref_ori_err > 20.0:
                    print(
                        "[Spiral] Ignoring ReadPathInfo first-point due mismatch with expected start "
                        f"(pos_err={ref_pos_err:.3f}mm ori_err={ref_ori_err:.3f}deg)."
                    )
            except Exception:
                pass
    elif first_pose is not None and len(first_pose) >= 6:
        start_pose_target = list(first_pose[:6])
    if start_pose_target is not None and len(start_pose_target) >= 6 and config is not None:
        tcp_name = config["coords"].get("tcptool3plane1")
        ucs_name = config["coords"].get("ucsTable1")
        actual_pose = _read_act_coord_pose(cps, tcp_name, ucs_name)
        if actual_pose is not None and len(actual_pose) >= 6:
            pos_err = max(abs(a - b) for a, b in zip(actual_pose[:3], start_pose_target[:3]))
            ori_err = max(abs(a - b) for a, b in zip(actual_pose[3:6], start_pose_target[3:6]))
            # In practice, orientation units/representation may differ between path points
            # and readback pose on some controller builds; use position as the hard gate.
            need_align = pos_err > float(start_pos_tol_mm)
            if need_align:
                if not manage_force_cycle:
                    # For segmented A->B MovePathL with force managed outside this function,
                    # do not force or abort start re-alignment; continue to preserve flow.
                    print(
                        "[Spiral] Start offset detected while force cycle is externally managed; "
                        f"continuing without enforced alignment (pos_err={pos_err:.3f}mm ori_err={ori_err:.3f}deg)."
                    )
                    need_align = False
                if need_align:
                    print(
                        "[Spiral] Aligning to trajectory start before MovePathL "
                        f"(pos_err={pos_err:.3f}mm ori_err={ori_err:.3f}deg)."
                    )
                    communicate(
                        cps=cps,
                        config=config,
                        point=list(start_pose_target[:6]),
                        tcp=tcp_name,
                        ucs=ucs_name,
                        seventh=-1,
                        speed=0.8,
                        wait=True,
                        speed_mode="linear",
                    )

    vibration_on = False
    force_applied = False
    ok = False

    try:
        if manage_force_cycle and force is not None and config is not None:
            putForceZminus(
                cps=cps,
                force=force,
                tcp=config["coords"]["tcptool3plane1"],
                ucs=config["coords"]["ucsTable1"],
                config=config,
            )
            force_applied = True
            time.sleep(force_settle_s)

        ret, st, _, l_err = _read_path_l_state()
        if ret != 0:
            print(f"[Spiral] Failed to read path state before MovePathL: ret={ret}")
            return False
        if l_err != "0":
            print(f"[Spiral] Path reported error before MovePathL: {st}")
            return False

        # Execute path. 40093/20041 are treated as retryable timing/busy states.
        move_started = False
        move_started_by_ret0 = False
        last_move_ret = None
        max_move_attempts = 120
        for attempt in range(max_move_attempts):
            if stop_requested():
                print("[Spiral] Stop requested before MovePathL; aborting segment.")
                return False

            ret = cps.HRIF_MovePathL(box_id, robot_id, track_name)
            last_move_ret = ret
            print(
                f"[Spiral] Robot returned {ret} after MovePathL "
                f"(attempt {attempt + 1}/{max_move_attempts})"
            )
            if ret == 0:
                move_started = True
                move_started_by_ret0 = True
                break

            _, st_after, _, l_err_after = _read_path_l_state()
            if ret in (40093, 20041) and l_err_after == "0":
                # 40093 often means another motion is still winding down.
                # Back off longer to avoid flooding MovePathL retries.
                time.sleep(0.20 if ret == 40093 else 0.08)
                continue

            print(
                f"[Spiral] MovePathL failed (ret={ret}, state={st_after}, path_err={l_err_after})"
            )
            return False

        if not move_started:
            print(f"[Spiral] MovePathL failed after retries (last_ret={last_move_ret}).")
            return False

        if manage_force_cycle:
            turn_vibration_on(cps)
            vibration_on = True

        # Monitor execution with controller-native signals.
        ok = True
        move_start = time.time()
        start = move_start
        estimate_timeout = float(completion_timeout)
        safety_timeout = estimate_timeout + max(8.0, min(30.0, estimate_timeout * 0.3))
        timeout_notice_logged = False
        done_stable_s = 0.12
        done_stable_start = None
        last_done_reason = None

        while True:
            if stop_requested():
                print("[Spiral] Stop requested during MovePathL wait; aborting segment.")
                ok = False
                break

            pstate = []
            pret = cps.HRIF_ReadPathState(box_id, robot_id, track_name, pstate)
            if pret != 0:
                print("HRIF_ReadPathState failed:", pret)
                ok = False
                break
            if len(pstate) > 3 and pstate[3] != "0":
                print(f"[Spiral] Path reported error: {pstate}")
                ok = False
                break

            elapsed_move = time.time() - move_start
            min_runtime_reached = elapsed_move >= max(0.0, float(min_runtime_s))

            # Read controller progress (0..1).
            track_result = []
            track_progress = None
            tre = cps.HRIF_ReadTrackProcess(box_id, robot_id, track_result)
            if tre == 0 and len(track_result) > 0:
                try:
                    track_progress = float(track_result[0])
                except (TypeError, ValueError):
                    track_progress = None
            # Read IsMotionDone.
            motion_done = []
            motion_done_api = False
            mret = cps.HRIF_IsMotionDone(box_id, robot_id, motion_done)
            if mret == 0 and motion_done:
                motion_done_api = _as_motion_done(motion_done[-1])

            # Read flags.
            flags = {}
            in_motion_now = None
            point_motion_done = False
            in_position = False
            if robot_state.fetch_and_update_flags():
                flags = robot_state.state.get("flags", {})
                in_motion_now = bool(flags.get("in_motion"))
                point_motion_done = bool(flags.get("point_motion_done"))
                in_position = bool(flags.get("in_position"))

            done_by_flags = (
                point_motion_done
                and in_position
                and (in_motion_now is False)
            )
            done_by_is_motion_done = (
                move_started_by_ret0
                and motion_done_api
                and (in_motion_now is False or in_motion_now is None)
            )
            done_by_track_raw = (track_progress is not None) and (track_progress >= 0.999999)
            # Guard against false-positive track completion while controller still
            # reports active motion.
            done_by_track = done_by_track_raw and (
                done_by_is_motion_done
                or done_by_flags
                or (in_motion_now is False)
            )

            completion_signal = done_by_track or done_by_is_motion_done or done_by_flags
            if min_runtime_reached and completion_signal:
                if done_stable_start is None:
                    done_stable_start = time.time()
                    if done_by_track:
                        if in_motion_now is True and point_motion_done and in_position:
                            last_done_reason = "track_process(stale_in_motion_flag)"
                        else:
                            last_done_reason = "track_process"
                    elif done_by_is_motion_done:
                        last_done_reason = "is_motion_done"
                    else:
                        last_done_reason = "flags_done+in_position+in_motion_false"
                elif (time.time() - done_stable_start) >= done_stable_s:
                    print(
                        f"[Spiral] Completion confirmed by {last_done_reason} "
                        f"for {done_stable_s:.2f}s; exiting wait loop."
                    )
                    break
            else:
                done_stable_start = None
                last_done_reason = None

            elapsed = time.time() - start
            if elapsed > estimate_timeout and not timeout_notice_logged:
                print(
                    f"[Spiral] Estimated runtime reached: elapsed={elapsed:.1f}s, "
                    f"estimate={estimate_timeout:.1f}s; waiting for true completion."
                )
                timeout_notice_logged = True

            if elapsed > safety_timeout:
                # Final guard: accept only if a controller-backed completion signal exists.
                safe_done = done_by_track or done_by_is_motion_done or done_by_flags
                if move_started_by_ret0 and safe_done:
                    print(
                        "[Spiral] Safety timeout reached with confirmed completion signal; "
                        "treating as complete."
                    )
                    break
                print(
                    f"Timeout waiting for idle. elapsed={elapsed:.1f}s, "
                    f"estimate={estimate_timeout:.1f}s, safety={safety_timeout:.1f}s, "
                    f"path_state={pstate}, track={track_result}, flags={flags}, "
                    f"is_motion_done={motion_done_api}"
                )
                ok = False
                break

            time.sleep(0.02)

    finally:
        if manage_force_cycle and vibration_on:
            turn_vibration_off(cps)
        if manage_force_cycle and force_applied:
            releaseForce(cps=cps, config=config, wait_for_blending=False)
        if delete_path_after:
            del_ret = cps.HRIF_DelPath(box_id, robot_id, track_name)
            if del_ret != 0:
                print(f"[Spiral] HRIF_DelPath failed for {track_name}: ret={del_ret}")

    return ok


def generate_zigzag_path(
    x_coords,
    y_coords,
    z_coords,
    innerOffset,
    innerOffsetX,
    x_min_offset: float = 0.0,
    x_max_offset: float = 0.0,
    orientation="horizontal",
    movement="zigzag",
    innerSandingOffset=50,
    edge_coverage=False
):
    """
    Generate a zigzag/spiral path for sanding.

    Args:
        edge_coverage: If True, prepend a rectangular edge coverage path
                       (P1→P2→P3→P4→P1) before the zigzag pattern. The zigzag
                       then starts immediately from P1 after edge coverage completes.
    """
    prepoint = None
    zigzag_coords = []

    # Parameters (adjust as needed)
    tool3y = 50.8  # Tool offset in Y
    tool3x = 38.1  # Tool offset in X
    xframe_1 = 0
    xframe_2 = 0

    # 1) Collect boundary coordinates as [x, y, z]
    boundary_coords = []
    for i in range(len(x_coords)):
        boundary_coords.append([x_coords[i], y_coords[i], z_coords[i]])

    # Close the loop by duplicating the first point at the end
    if boundary_coords:
        boundary_coords.append(boundary_coords[0][:])  # copy for safety

    # 2) Compute the zigzag path (offset corners + zigzag)
    zigzag_coords = []

    # Ensure we have valid coordinates
    if x_coords and y_coords and z_coords:
        # We'll assume the pocket's Z-level is the same as the first boundary point
        z_zigzag = boundary_coords[0][2]

        # For Pocket4, corners (P13, P14, P15, P16):
        modified_Point2 = [
            (x_coords[1]) / 1 + tool3x + innerOffsetX,
            y_coords[1] - tool3y - (innerOffset),
        ]
        modified_Point3 = [
            x_coords[2] - tool3x - innerOffset,
            y_coords[2] - tool3y - innerOffset,
        ]
        modified_Point1 = [
            (x_coords[0]) / 1 + tool3x + innerOffsetX,
            y_coords[0] + tool3y + innerOffset,
        ]
        modified_Point4 = [
            x_coords[3] - tool3x - innerOffset,
            y_coords[3] + tool3y + innerOffset,
        ]
        print("modified_Point1:", modified_Point1)
        print("modified_Point2:", modified_Point2)
        print("modified_Point3:", modified_Point3)
        print("modified_Point4:", modified_Point4)

        # Bounding box for alternate orientations
        x_min = min(
            modified_Point1[0],
            modified_Point2[0],
            modified_Point3[0],
            modified_Point4[0],
        )
        x_max = max(
            modified_Point1[0],
            modified_Point2[0],
            modified_Point3[0],
            modified_Point4[0],
        )
        y_min = min(
            modified_Point1[1],
            modified_Point2[1],
            modified_Point3[1],
            modified_Point4[1],
        )
        y_max = max(
            modified_Point1[1],
            modified_Point2[1],
            modified_Point3[1],
            modified_Point4[1],
        )

        # Optional x-bound expansion/contraction (used for split overlap in horizontal zigzag)
        x_min += float(x_min_offset)
        x_max += float(x_max_offset)

        # Calculate available horizontal dimension
        xinner = abs(x_max - x_min)
        print("xinner=", xinner)

        orientation_mode = (orientation or "vertical").lower()
        movement_mode = (movement or "zigzag").lower()

        # Modified Point Layout:
        # modified_Point1 = Bottom-right
        # modified_Point2 = Top-right
        # modified_Point3 = Top-left
        # modified_Point4 = Bottom-left

        rx_sanding = -0.034
        ry_sanding = 0.556
        rz_sanding = 0.251

        # Generate edge coverage path if enabled (rectangular path around the boundary)
        # The edge coverage must END at the point where zigzag/spiral will START
        # Edge coverage uses ORIGINAL boundary points (without innerOffsetX/innerOffset applied)
        edge_coverage_coords = []
        if edge_coverage:
            # Remove the offsets from modified points to get original boundary positions
            # Modified points already have tool3x, tool3y, innerOffsetX, and innerOffset applied
            # For edge coverage, we only want tool3x and tool3y, not the inner offsets
            edge_Point1 = [
                x_coords[0] + tool3x + 1.75,
                y_coords[0] + tool3y + 1.75,
                z_zigzag,
            ]
            edge_Point2 = [
                x_coords[1] + tool3x + 1.75,
                y_coords[1] - tool3y - 1.75,
                z_zigzag,
            ]
            edge_Point3 = [
                x_coords[2] - tool3x - 1.75,
                y_coords[2] - tool3y - 1.75,
                z_zigzag,
            ]
            edge_Point4 = [
                x_coords[3] - tool3x - 1.75,
                y_coords[3] + tool3y + 1.75,
                z_zigzag,
            ]

            # Edge coverage always uses the same path regardless of orientation
            # Edge coverage: P2 → P3 → P4 → P1 → P2 (starts at top-left like horizontal)
            if orientation_mode == "horizontal":
                # Horizontal zigzag starts at top-left (P2)
                # Edge coverage: P2 → P3 → P4 → P1 → P2 (ends at top-left)
                edge_coverage_coords = [
                    [
                        edge_Point2[0],
                        edge_Point2[1],
                        edge_Point2[2],
                        rx_sanding,
                        ry_sanding,
                        rz_sanding,
                    ],  # Start P2 (top-left)
                    [
                        edge_Point3[0],
                        edge_Point3[1],
                        edge_Point3[2],
                        rx_sanding,
                        ry_sanding,
                        rz_sanding,
                    ],  # P3 (top-right)
                    [
                        edge_Point4[0],
                        edge_Point4[1],
                        edge_Point4[2],
                        rx_sanding,
                        ry_sanding,
                        rz_sanding,
                    ],  # P4 (bottom-right)
                    [
                        edge_Point1[0],
                        edge_Point1[1],
                        edge_Point1[2],
                        rx_sanding,
                        ry_sanding,
                        rz_sanding,
                    ],  # P1 (bottom-left)
                    [
                        edge_Point2[0],
                        edge_Point2[1],
                        edge_Point2[2],
                        rx_sanding,
                        ry_sanding,
                        rz_sanding,
                    ],  # End P2 (top-left) - zigzag start
                ]
                print(
                    "Edge coverage (horizontal): P2 → P3 → P4 → P1 → P2 (ends at top-left for zigzag start)"
                )
            else:
                # Vertical zigzag starts at bottom-right (P4)
                # Edge coverage: P4 → P1 → P2 → P3 → P4 (ends at bottom-right)
                edge_coverage_coords = [
                    [
                        edge_Point4[0],
                        edge_Point4[1],
                        edge_Point4[2],
                        rx_sanding,
                        ry_sanding,
                        rz_sanding,
                    ],  # Start P4 (bottom-right)
                    [
                        edge_Point1[0],
                        edge_Point1[1],
                        edge_Point1[2],
                        rx_sanding,
                        ry_sanding,
                        rz_sanding,
                    ],  # P1 (bottom-left)
                    [
                        edge_Point2[0],
                        edge_Point2[1],
                        edge_Point2[2],
                        rx_sanding,
                        ry_sanding,
                        rz_sanding,
                    ],  # P2 (top-left)
                    [
                        edge_Point3[0],
                        edge_Point3[1],
                        edge_Point3[2],
                        rx_sanding,
                        ry_sanding,
                        rz_sanding,
                    ],  # P3 (top-right)
                    [
                        edge_Point4[0],
                        edge_Point4[1],
                        edge_Point4[2],
                        rx_sanding,
                        ry_sanding,
                        rz_sanding,
                    ],  # End P4 (bottom-right) - zigzag start
                ]
                print(
                    "Edge coverage (vertical): P4 → P1 → P2 → P3 → P4 (ends at bottom-right for zigzag start)"
                )

        if movement_mode == "rect":
            zigzag_coords = [
                [
                    modified_Point1[0],
                    modified_Point1[1],
                    z_zigzag,
                    rx_sanding,
                    ry_sanding,
                    rz_sanding,
                ],
                [
                    modified_Point2[0],
                    modified_Point2[1],
                    z_zigzag,
                    rx_sanding,
                    ry_sanding,
                    rz_sanding,
                ],
                [
                    modified_Point3[0],
                    modified_Point3[1],
                    z_zigzag,
                    rx_sanding,
                    ry_sanding,
                    rz_sanding,
                ],
                [
                    modified_Point4[0],
                    modified_Point4[1],
                    z_zigzag,
                    rx_sanding,
                    ry_sanding,
                    rz_sanding,
                ],
                [
                    modified_Point1[0],
                    modified_Point1[1],
                    z_zigzag,
                    rx_sanding,
                    ry_sanding,
                    rz_sanding,
                ],
            ]
        elif orientation_mode == "horizontal":
            yinner = abs(y_max - y_min)
            if yinner > 0:
                num_steps = math.floor(yinner / innerSandingOffset)
                if num_steps == 0:
                    num_steps = 1  # Prevent division by zero, ensure at least 1 row
                adjusted_step = yinner / num_steps

                offset = 0.0
                toggle = 0

                # Build zigzag rows from top (max y) to bottom (min y)
                # Starts at top-left (x_min, y_max)
                while offset <= yinner + 1e-9:
                    current_y = y_max - offset
                    row_points = [
                        [
                            x_min,
                            current_y,
                            z_zigzag,
                            rx_sanding,
                            ry_sanding,
                            rz_sanding,
                        ],  # Start at x_min (left side)
                        [
                            x_max,
                            current_y,
                            z_zigzag,
                            rx_sanding,
                            ry_sanding,
                            rz_sanding,
                        ],  # Go to x_max (right side)
                    ]
                    if toggle:
                        row_points.reverse()

                    zigzag_coords.extend(row_points)
                    offset += adjusted_step
                    toggle = 1 - toggle
            else:
                # Fallback: single row when height is too small
                zigzag_coords.extend(
                    [
                        [x_min, y_max, z_zigzag, rx_sanding, ry_sanding, rz_sanding],
                        [x_max, y_max, z_zigzag, rx_sanding, ry_sanding, rz_sanding],
                    ]
                )
        else:
            # Vertical orientation
            # In real robot, P1/P2 (X = -212.56) are physically RIGHT, P3/P4 (X = -55.1) are physically LEFT
            # Start from RIGHT side (P1/P2) and subtract offset to move toward LEFT (P3/P4)
            if xinner > 0:
                # Determine how many "columns" in the zigzag
                num_steps = math.ceil(xinner / innerSandingOffset)
                if num_steps == 0:
                    num_steps = 1  # Prevent division by zero
                adjusted_step = xinner / num_steps
                print(
                    f"[Vertical] xinner={xinner}, num_steps={num_steps}, adjusted_step={adjusted_step}"
                )

                offset = 0.0
                toggle = 0

                # Build zigzag path from right (P1/P2) to left (P3/P4)
                # Subtract offset from P1/P2 X values to move toward more negative X
                while offset <= xinner + 1e-9:  # small floating-point tolerance
                    row_points = [
                        [
                            modified_Point4[0] - offset,
                            modified_Point4[1],
                            z_zigzag,
                            rx_sanding,
                            ry_sanding,
                            rz_sanding,
                        ],  # P1 side (bottom-right, moving left)
                        [
                            modified_Point3[0] - offset,
                            modified_Point3[1],
                            z_zigzag,
                            rx_sanding,
                            ry_sanding,
                            rz_sanding,
                        ],  # P2 side (top-right, moving left)
                    ]
                    # Reverse every other column to create a zigzag
                    if toggle:
                        row_points.reverse()

                    zigzag_coords.extend(row_points)
                    offset += adjusted_step
                    toggle = 1 - toggle
            else:
                # Fallback: single column when width is too small
                zigzag_coords.extend(
                    [
                        [
                            modified_Point4[0],
                            modified_Point4[1],
                            z_zigzag,
                            rx_sanding,
                            ry_sanding,
                            rz_sanding,
                        ],
                        [
                            modified_Point3[0],
                            modified_Point3[1],
                            z_zigzag,
                            rx_sanding,
                            ry_sanding,
                            rz_sanding,
                        ],
                    ]
                )

        # Update coordinates to absolute values for edge coverage
        for point in edge_coverage_coords:
            point[1] = abs(point[1])
            point[0] = abs(point[0])

        # Update only the y coordinate to its absolute value for zigzag
        for point in zigzag_coords:
            point[1] = abs(point[1])
            point[0] = abs(point[0])

        if movement_mode == "rect":
            prepoint = [
                abs(zigzag_coords[0][0]) + 0.5,
                zigzag_coords[0][1],
                z_zigzag,
                rx_sanding,
                ry_sanding,
                rz_sanding,
            ]
        elif orientation_mode == "horizontal":
            # Horizontal: edge coverage starts at P2 (top-left), zigzag starts at top-left
            if edge_coverage:
                prepoint = [
                    abs(modified_Point2[0]) + 0.5,
                    abs(modified_Point2[1]),
                    z_zigzag,
                    rx_sanding,
                    ry_sanding,
                    rz_sanding,
                ]
            else:
                prepoint = [
                    abs(x_min) + 0.5,
                    y_max,
                    z_zigzag,
                    rx_sanding,
                    ry_sanding,
                    rz_sanding,
                ]
        else:
            # Vertical: edge coverage starts at same position as horizontal (P2/top-left)
            if edge_coverage:
                prepoint = [
                    abs(modified_Point4[0]) + 0.5,
                    abs(modified_Point4[1]),
                    z_zigzag,
                    rx_sanding,
                    ry_sanding,
                    rz_sanding,
                ]
            else:
                prepoint = [
                    abs(modified_Point4[0]) + 0.5,
                    abs(modified_Point4[1]),
                    z_zigzag,
                    rx_sanding,
                    ry_sanding,
                    rz_sanding,
                ]

        if edge_coverage:
            print(
                f"Edge coverage: {len(edge_coverage_coords)} points (MoveL), Zigzag: {len(zigzag_coords)} points (spiral)"
            )

    return edge_coverage_coords, zigzag_coords, prepoint


def load_config():
    """Loads configuration from config.yaml."""
    with open("./configs/config.yaml", "r") as file:
        config = yaml.safe_load(file)
    return config


def load_json_config():
    """Loads configuration from config.json."""
    with open("./configs/cycleData.json", "r") as file:
        config = json.load(file)
    return config


def get_pocket_size(door_number, default_on_error=True):
    """Return pocket size (xlen, ylen) using pocket corner points."""
    points = []
    for corner in range(4):
        point = get_pocket_point(
            door_number, corner, default_on_error=default_on_error
        )
        if not point or len(point) < 2:
            return {"xlen": "null", "ylen": "null"}
        try:
            x_val = float(point[0])
            y_val = float(point[1])
        except (TypeError, ValueError):
            return {"xlen": "null", "ylen": "null"}
        points.append((x_val, y_val))

    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    return {"xlen": max(xs) - min(xs), "ylen": max(ys) - min(ys)}


def smalldoor1zizag(
    force, z, cps, orientation="horizontal", movement="zigzag", spiral_settings=None
):
    apply_spiral_settings(spiral_settings)

    def smalldoor1zizagsmall(force, z, cps, split=False):
        # Load configuration from YAML
        config = load_config()
        json_config = load_json_config()

        # Set up logger
        config["logger"] = setup_logger(config["settings"]["debug"])

        # Establish connection with robot
        # cps = CPSClient()
        # IP = config['server']['cpip']
        # port = config['server']['cps']
        # ret = cps.HRIF_Connect(0, IP, port)

        # Main Points
        p8 = get_inner_corner_point(1, 0)
        print("p8:", p8)
        p7 = get_inner_corner_point(1, 1)
        print("p7:", p7)
        p6 = get_inner_corner_point(1, 2)
        print("p6:", p6)
        p5 = get_inner_corner_point(1, 3)
        print("p5:", p5)
        # 7th axis postion
        x1 = p8[0] + get_door_position(1)
        print("x1:", x1)
        tcx0 = x1
        tcx1 = tcx0 + ((p6[0] - p7[0]) / 2)
        seventh_positions = [tcx0, tcx1] if split else [x1]
        # prehoming
        prehoming = [0, 200, 50, 0, 0, 0]
        # #Depth of the poocket z
        # z=-6.5
        # zigzag points

        distance = p6[0] - p8[0]
        print("distance:", distance)
        # Points calculation
        point5 = [-p5[0], p5[1], z, -0.034, 0.556, 0.251]
        print("point5:", point5)
        point6 = [-p6[0], p6[1], z, -0.034, 0.556, 0.251]
        print("point6:", point6)
        point7 = [-p7[0], p7[1], z, -0.034, 0.556, 0.251]
        print("point7:", point7)
        point8 = [-p8[0], p8[1], z, -0.034, 0.556, 0.251]
        print("point8:", point8)

        # Final Points
        point5u = [-distance, point5[1], point5[2], point5[3], point5[4], point5[5]]
        print("point5u:", point5u)
        point6u = [-distance, point6[1], point6[2], point6[3], point6[4], point6[5]]
        print("point6u:", point6u)
        point7u = [0, point7[1], point7[2], point7[3], point7[4], point7[5]]
        print("point7u:", point7u)
        point8u = [0, point8[1], point8[2], point8[3], point8[4], point8[5]]
        print("point8u:", point8u)

        # 1) Collect boundary coordinates as [x, y, z]
        x_coords1 = [point5u[0], point6u[0], point7u[0], point8u[0]]
        y_coords1 = [point5u[1], point6u[1], point7u[1], point8u[1]]
        z_coords1 = [point5u[2], point6u[2], point7u[2], point8u[2]]

        print("x_coords1 ", x_coords1)
        print("y_coords2 ", y_coords1)
        print("z_coords3", z_coords1)

        # Global variables
        # prepoint = None
        # zigzag_coords = []

        # Second Pocket 1st Cycle
        x_coords_path = x_coords1
        if split:
            x_coords_path = [coord / 2 for coord in x_coords1]

        orientation_mode = (orientation or "horizontal").lower()
        inner_sanding_offset = 50.0
        middle_overlap = inner_sanding_offset * 0.5

        edge_coverage_pathp1, _, edge_prepointp1 = generate_zigzag_path(
            x_coords=x_coords1,
            y_coords=y_coords1,
            z_coords=z_coords1,
            innerOffset=17,
            innerOffsetX=17,
            orientation=orientation,
            movement=movement,
            innerSandingOffset=50,
            edge_coverage=True  # Enable edge coverage with MoveL before spiral
        )
        _, zigzag_pathp1, prepointp1 = generate_zigzag_path(
            x_coords=x_coords_path,
            y_coords=y_coords1,
            z_coords=z_coords1,
            innerOffset=17,
            innerOffsetX=17,
            orientation=orientation,
            movement=movement,
            innerSandingOffset=50,
            edge_coverage=False
        )
        zigzag_pathp1_left = zigzag_pathp1
        zigzag_pathp1_right = zigzag_pathp1
        if split:
            _, zigzag_pathp1_left, _ = generate_zigzag_path(
                x_coords=x_coords_path,
                y_coords=y_coords1,
                z_coords=z_coords1,
                innerOffset=17,
                innerOffsetX=17,
                x_max_offset=middle_overlap,
                orientation=orientation,
                movement=movement,
                innerSandingOffset=50,
                edge_coverage=False,
            )
            _, zigzag_pathp1_right, _ = generate_zigzag_path(
                x_coords=x_coords_path,
                y_coords=y_coords1,
                z_coords=z_coords1,
                innerOffset=17,
                innerOffsetX=17,
                x_min_offset=-middle_overlap,
                orientation=orientation,
                movement=movement,
                innerSandingOffset=50,
                edge_coverage=False,
            )
        print("edge_coverage_pathp1=", edge_coverage_pathp1)
        print("zigzag_pathp=", zigzag_pathp1)
        print("prepointp:", prepointp1)

        def perform_process_top(
            cps, config, edge_points, zigzag_points, force, run_edge_coverage=True
        ):
            force_seek_linear = 5.0
            force_blending_timeout = 7.0
            if split:
                force_blending_timeout = 0.4
            has_edge = run_edge_coverage and edge_points and len(edge_points) > 0
            if has_edge:
                force_seek_linear = 5.0
                force_blending_timeout = 7.0
                if split:
                    force_blending_timeout = 0.4
                putForceZminus(
                    cps=cps,
                    force=force,
                    tcp=config["coords"]["tcptool3plane1"],
                    ucs=config["coords"]["ucsTable1"],
                    config=config,
                    search_linear_velocity=force_seek_linear,
                    blending_timeout_s=force_blending_timeout,
                )
                print("Turned Vibration On")

            # Step 1: Edge coverage with MoveL (linear path between modified points)
            if has_edge:
                print(
                    f"[Edge Coverage] Starting linear MoveL for {len(edge_points)} edge points"
                )
                edge_speed = float(json_config["sandingSpeed"]) * 0.3
                for point in edge_points:
                    communicate(
                        cps=cps,
                        config=config,
                        point=point,
                        tcp=config["coords"]["tcptool3plane1"],
                        ucs=config["coords"]["ucsTable1"],
                        seventh=-1,
                        speed=edge_speed,
                        speed_mode="linear",
                        wait=False
                    )
                    if point == edge_points[-1]:
                        communicate(
                            cps=cps,
                            config=config,
                            point=point,
                            tcp=config["coords"]["tcptool3plane1"],
                            ucs=config["coords"]["ucsTable1"],
                            seventh=-1,
                            speed=edge_speed,
                            speed_mode="linear",
                            wait=True
                        )
                    turn_vibration_on(cps)
                # Wait for edge coverage to complete before starting spiral
                waitForBlending(cps=cps, config=config)
                print("[Edge Coverage] Completed linear edge path")
                releaseForce(cps=cps, config=config)
                turn_vibration_off(cps)

            # Step 2: Zigzag/Spiral motion
            if zigzag_points and len(zigzag_points) > 0:
                # Move to first zigzag point to ensure proper transition from edge coverage
                print("[Spiral] Moving to first zigzag point:", zigzag_points[0])
                communicate(
                    cps=cps,
                    config=config,
                    point=zigzag_points[0],
                    tcp=config["coords"]["tcptool3plane1"],
                    ucs=config["coords"]["ucsTable1"],
                    seventh=-1,
                    speed=float(json_config["sandingSpeed"]),
                    wait=True
                )
                # locked_orient = list(zigzag_points[0][3:6])
                # if FORCE_SPIRAL_ORIENT:
                #     locked_orient = list(FORCE_SPIRAL_ORIENT)
                putForceZminus(
                    cps=cps,
                    force=force,
                    tcp=config["coords"]["tcptool3plane1"],
                    ucs=config["coords"]["ucsTable1"],
                    config=config,
                    search_linear_velocity=force_seek_linear,
                    blending_timeout_s=force_blending_timeout,
                )
                # if FORCE_SPIRAL_ORIENT is None:
                #     locked_orient = _capture_locked_orient(
                #         cps,
                #         config,
                #         tcp=config["coords"]["tcptool3plane1"],
                #         ucs=config["coords"]["ucsTable1"],
                #         fallback_orient=locked_orient,
                #         settle_s=0.05,
                #     )
                # else:
                #     if config.get("logger"):
                #         config["logger"].info(f"[Orientation] forced for spiral: {locked_orient}")
                #     else:
                #         print(f"[Orientation] forced for spiral: {locked_orient}")
                turn_vibration_on(cps)
                # use_waypoint2 = True
                # wp2_cfg = Waypoint2Config(
                #     speed=150.0,
                #     accel=300.0,
                #     radius=8.0,
                #     min_seg_len=5.0,
                #     min_angle_deg=12.0,
                #     max_angle_deg=170.0,
                #     use_arc=True,
                #     use_wp2_for_line=True,
                #     enforce_orientation="start",
                #     wait_timeout_s=20.0,
                #     cmd_id_prefix="zig",
                #     line_cmd_id_prefix="zigL",
                # )
                # throttle_every = 8
                # throttle_sleep_s = 0.01
                # move_kwargs = {
                #     "cps": cps,
                #     "config": config,
                #     "tcp": config["coords"]["tcptool3plane1"],
                #     "ucs": config["coords"]["ucsTable1"],
                #     "seventh": -1,
                #     "speed": float(json_config["sandingSpeed"]),
                #     "speed_mode": "linear",
                #     "wait": True,
                # }
                # zigzag_points = [
                #     [p[0], p[1], p[2], locked_orient[0], locked_orient[1], locked_orient[2]]
                #     for p in zigzag_points
                # ]
                # bounds = None
                # bounds_points = edge_points if edge_points else zigzag_points
                # if bounds_points:
                #     xs = [p[0] for p in bounds_points]
                #     ys = [p[1] for p in bounds_points]
                #     bounds = (min(xs), max(xs), min(ys), max(ys))
                if len(zigzag_points) < 2:
                    print("[Spiral] Not enough zigzag points for path execution.")
                else:
                    full_track_name = f"small_{uuid.uuid4().hex[:6]}"
                    total_points = 0
                    for index in range(len(zigzag_points) - 1):
                        point_A = zigzag_points[index]
                        point_B = zigzag_points[index + 1]

                    # if use_waypoint2:
                    #     wp2_segments = generate_arc_line_segments_between(
                    #         point_A,
                    #         point_B,
                    #         radius=12.0,
                    #         arc_step_deg=120.0,
                    #         pitch=12.0,
                    #         clockwise=True,
                    #         bounds=bounds,
                    #         safety_margin=5.0,
                    #         min_radius=1.0,
                    #     )
                    #     last_pair = index == (len(zigzag_points) - 2)
                    #     for seg_idx, seg_points in enumerate(wp2_segments):
                    #         wait_end = last_pair and seg_idx == (len(wp2_segments) - 1)
                    #         wp2_result = execute_waypoint2_path(
                    #             cps,
                    #             seg_points,
                    #             tcp=config["coords"]["tcptool3plane1"],
                    #             ucs=config["coords"]["ucsTable1"],
                    #             cfg=wp2_cfg,
                    #             wait_each=False,
                    #             wait_end=wait_end,
                    #             throttle_every=throttle_every,
                    #             throttle_sleep_s=throttle_sleep_s,
                    #             move_l_fn=communicate,
                    #             move_l_kwargs=move_kwargs,
                    #             logger=config.get("logger"),
                    #         )
                    #         if not wp2_result.get("ok", False):
                    #             raise RuntimeError("[WayPoint2] Segment execution failed.")
                    #     continue
                    
                # #motion done for zigzag, now turn off vibration and release force before next steps        
                # turn_vibration_off(cps)
                # releaseForce(cps=cps, config=config)
                
                        print("Spiral move from A to B:", point_A, "->", point_B)
                        success, count = run_spiral_between_points(
                            cps=cps,
                            config=config,
                            start_pose=point_A,
                            end_pose=point_B,
                            radius=12.0,
                            angle_step_deg=60.0,
                            track_name=full_track_name,
                            velocity=110.0,
                            accel=200.0,
                            jerk=2800.0,
                            init_path=(index == 0),
                            orientation=orientation
                        )
                        if not success:
                            raise RuntimeError("[Spiral] run_spiral_between_points failed for door 1.")
                        total_points += int(count or 0)

                    timeout = compute_timeout(
                        total_points=total_points, velocity=300.0 * 10.0 / 45.0
                    )
                    # Keep a short anti-false-positive runtime guard without
                    # adding visible delay at the final point.
                    min_runtime_s = max(0.15, min(0.6, timeout * 0.05))
                    finalized = finalize_spiral_path(
                        cps,
                        full_track_name,
                        box_id=0,
                        robot_id=0,
                        completion_timeout=timeout,
                        min_runtime_s=min_runtime_s,
                        force=force,
                        config=config,
                        manage_force_cycle=False,
                        expected_start_pose=zigzag_points[0],
                    )
                    if not finalized:
                        raise RuntimeError("[Spiral] finalize_spiral_path failed for door 1.")
                    
            # Wait for blending and turn off vibration
            waitForBlending(cps=cps, config=config)
            turn_vibration_off(cps)
            # Release force
            releaseForce(cps=cps, config=config)

        if split and edge_coverage_pathp1:
            edge_start = edge_coverage_pathp1[0]
        else:
            edge_start = prepointp1

        if split:
            communicate(
                cps=cps,
                config=config,
                seventh=tcx0,
                tcp=config["coords"]["tcptool3plane1"],
                ucs=config["coords"]["ucsTable1"],
                speed=0.8,
                wait=True
            )
            if edge_prepointp1:
                communicate(
                    cps=cps,
                    config=config,
                    point=edge_prepointp1,
                    tcp=config["coords"]["tcptool3plane1"],
                    ucs=config["coords"]["ucsTable1"],
                    seventh=-1,
                    speed=1.0,
                    wait=True
                )
            communicate(
                cps=cps,
                config=config,
                point=edge_start,
                tcp=config["coords"]["tcptool3plane1"],
                ucs=config["coords"]["ucsTable1"],
                seventh=-1,
                speed=1.0,
                wait=True
            )
            perform_process_top(
                cps,
                config,
                edge_points=edge_coverage_pathp1,
                zigzag_points=[],
                force=force,
                run_edge_coverage=True
            )
        
        for idx, seventh_pos in enumerate(seventh_positions):
            first_split = split and idx == 0
            current_zigzag = zigzag_pathp1
            if split:
                current_zigzag = zigzag_pathp1_left if idx == 0 else zigzag_pathp1_right
            if not first_split:
                communicate(
                    cps=cps,
                    config=config,
                    seventh=seventh_pos,
                    tcp=config["coords"]["tcptool3plane1"],
                    ucs=config["coords"]["ucsTable1"],
                    speed=0.8,
                    wait=True
                )
                if not split:
                    communicate(
                        cps=cps,
                        config=config,
                        point=prepointp1,  # Dynamic prepoint
                        tcp=config["coords"]["tcptool3plane1"],
                        ucs=config["coords"]["ucsTable1"],
                        seventh=-1,
                        speed=1.0,
                        wait=True
                    )
            # # turn_vibration_on(cps)
            perform_process_top(
                cps,
                config,
                edge_points=edge_coverage_pathp1 if not split else [],
                zigzag_points=current_zigzag,
                force=force,
                run_edge_coverage=not split
            )  # Edge coverage + zigzag path
            # # turn_vibration_off(cps)
            communicate(
                cps=cps,
                config=config,
                point=prehoming,
                tcp=config["coords"]["tcptool3plane1"],
                ucs=config["coords"]["ucsTable1"],
                seventh=-1,
                speed=0.8,
                wait=True
            )

    pocket_size = get_pocket_size(1, default_on_error=True)
    pocket_y = pocket_size["ylen"]
    pocket_x = pocket_size["xlen"]

    print("pocket_y:", pocket_y)
    print("pocket_x:", pocket_x)

    # Check conditions
    if pocket_y == "null" or pocket_x == "null":
        print("No door data available - skipping operations")
    elif isinstance(pocket_y, (int, float)) and isinstance(pocket_x, (int, float)):
        smalldoor1zizagsmall(force, z, cps, split=pocket_y > 600 and pocket_x > 290)
    elif isinstance(pocket_y, (int, float)) or isinstance(pocket_x, (int, float)):
        smalldoor1zizagsmall(force, z, cps, split=False)
    else:
        print(
            f"Invalid pocket size type: {type(pocket_x)}, {type(pocket_y)} - expected numbers or 'null'"
        )


def smalldoor2zizag(
    force, z, cps, orientation="horizontal", movement="zigzag", spiral_settings=None
):
    apply_spiral_settings(spiral_settings)

    def smalldoor2zizagsmall(force, z, cps, split=False):
        # Load configuration from YAML
        config = load_config()
        json_config = load_json_config()

        # Set up logger
        config["logger"] = setup_logger(config["settings"]["debug"])

        # Establish connection with robot
        # cps = CPSClient()
        # IP = config['server']['cpip']
        # port = config['server']['cps']
        # ret = cps.HRIF_Connect(0, IP, port)

        # Main Points
        p8 = get_inner_corner_point(2, 0)
        print("p8:", p8)
        p7 = get_inner_corner_point(2, 1)
        print("p7:", p7)
        p6 = get_inner_corner_point(2, 2)
        print("p6:", p6)
        p5 = get_inner_corner_point(2, 3)
        print("p5:", p5)
        # 7th axis postion
        x1 = p8[0] + get_door_position(2)
        print("x1:", x1)
        tcx0 = x1
        tcx1 = tcx0 + ((p6[0] - p7[0]) / 2)
        seventh_positions = [tcx0, tcx1] if split else [x1]
        # prehoming
        prehoming = [0, 200, 50, 0, 0, 0]
        # #Depth of the poocket z
        # z=-6.5
        # zigzag points
        distance = p6[0] - p8[0]
        print("distance:", distance)
        # Points calculation
        point5 = [-p5[0], p5[1], z, -0.034, 0.556, 0.251]
        print("point5:", point5)
        point6 = [-p6[0], p6[1], z, -0.034, 0.556, 0.251]
        print("point6:", point6)
        point7 = [-p7[0], p7[1], z, -0.034, 0.556, 0.251]
        print("point7:", point7)
        point8 = [-p8[0], p8[1], z, -0.034, 0.556, 0.251]
        print("point8:", point8)

        # Final Points
        point5u = [-distance, point5[1], point5[2], point5[3], point5[4], point5[5]]
        print("point5u:", point5u)
        point6u = [-distance, point6[1], point6[2], point6[3], point6[4], point6[5]]
        print("point6u:", point6u)
        point7u = [0, point7[1], point7[2], point7[3], point7[4], point7[5]]
        print("point7u:", point7u)
        point8u = [0, point8[1], point8[2], point8[3], point8[4], point8[5]]
        print("point8u:", point8u)

        # 1) Collect boundary coordinates as [x, y, z]
        x_coords1 = [point5u[0], point6u[0], point7u[0], point8u[0]]
        y_coords1 = [point5u[1], point6u[1], point7u[1], point8u[1]]
        z_coords1 = [point5u[2], point6u[2], point7u[2], point8u[2]]

        print("x_coords1 ", x_coords1)
        print("y_coords2 ", y_coords1)
        print("z_coords3", z_coords1)

        # Global variables
        # prepoint = None
        # zigzag_coords = []

        # Second Pocket 1st Cycle
        x_coords_path = x_coords1
        if split:
            x_coords_path = [coord / 2 for coord in x_coords1]

        orientation_mode = (orientation or "horizontal").lower()
        inner_sanding_offset = 50.0
        middle_overlap = inner_sanding_offset * 0.5

        edge_coverage_pathp1, _, edge_prepointp1 = generate_zigzag_path(
            x_coords=x_coords1,
            y_coords=y_coords1,
            z_coords=z_coords1,
            innerOffset=17,
            innerOffsetX=17,
            orientation=orientation,
            movement=movement,
            innerSandingOffset=50,
            edge_coverage=True  # Enable edge coverage with MoveL before spiral
        )
        _, zigzag_pathp1, prepointp1 = generate_zigzag_path(
            x_coords=x_coords_path,
            y_coords=y_coords1,
            z_coords=z_coords1,
            innerOffset=17,
            innerOffsetX=17,
            orientation=orientation,
            movement=movement,
            innerSandingOffset=50,
            edge_coverage=False
        )
        zigzag_pathp1_left = zigzag_pathp1
        zigzag_pathp1_right = zigzag_pathp1
        if split:
            _, zigzag_pathp1_left, _ = generate_zigzag_path(
                x_coords=x_coords_path,
                y_coords=y_coords1,
                z_coords=z_coords1,
                innerOffset=17,
                innerOffsetX=17,
                x_max_offset=middle_overlap,
                orientation=orientation,
                movement=movement,
                innerSandingOffset=50,
                edge_coverage=False,
            )
            _, zigzag_pathp1_right, _ = generate_zigzag_path(
                x_coords=x_coords_path,
                y_coords=y_coords1,
                z_coords=z_coords1,
                innerOffset=17,
                innerOffsetX=17,
                x_min_offset=-middle_overlap,
                orientation=orientation,
                movement=movement,
                innerSandingOffset=50,
                edge_coverage=False,
            )
        print("edge_coverage_pathp1=", edge_coverage_pathp1)
        print("zigzag_pathp=", zigzag_pathp1)
        print("prepointp:", prepointp1)

        def perform_process_top(
            cps, config, edge_points, zigzag_points, force, run_edge_coverage=True
        ):
            force_seek_linear = 5.0
            force_blending_timeout = 7.0
            if split:
                force_blending_timeout = 0.4
            has_edge = run_edge_coverage and edge_points and len(edge_points) > 0
            if has_edge:
                force_seek_linear = 5.0
                force_blending_timeout = 7.0
                if split:
                    force_blending_timeout = 0.4
                putForceZminus(
                    cps=cps,
                    force=force,
                    tcp=config["coords"]["tcptool3plane1"],
                    ucs=config["coords"]["ucsTable1"],
                    config=config,
                    search_linear_velocity=force_seek_linear,
                    blending_timeout_s=force_blending_timeout,
                )

            # Step 1: Edge coverage with MoveL (linear path between modified points)
            if has_edge:
                print(
                    f"[Edge Coverage] Starting linear MoveL for {len(edge_points)} edge points"
                )
                edge_speed = float(json_config["sandingSpeed"]) * 0.3
                for point in edge_points:
                    communicate(
                        cps=cps,
                        config=config,
                        point=point,
                        tcp=config["coords"]["tcptool3plane1"],
                        ucs=config["coords"]["ucsTable1"],
                        seventh=-1,
                        speed=edge_speed,
                        speed_mode="linear",
                        wait=False
                    )
                    if point == edge_points[-1]:
                        communicate(
                            cps=cps,
                            config=config,
                            point=point,
                            tcp=config["coords"]["tcptool3plane1"],
                            ucs=config["coords"]["ucsTable1"],
                            seventh=-1,
                            speed=edge_speed,
                            speed_mode="linear",
                            wait=True
                        )
                    turn_vibration_on(cps)
                # Wait for edge coverage to complete before starting spiral
                waitForBlending(cps=cps, config=config)
                print("[Edge Coverage] Completed linear edge path")
                releaseForce(cps=cps, config=config)
                turn_vibration_off(cps)

            # Step 2: Zigzag/Spiral motion
            if zigzag_points and len(zigzag_points) > 0:
                spiral_track_name = f"small_{uuid.uuid4().hex[:6]}"
                path_initialized = False
                push_failed = False
                total_count = 0
                
                # Move to first zigzag point to ensure proper transition from edge coverage
                print("[Spiral] Moving to first zigzag point:", zigzag_points[0])
                communicate(
                    cps=cps,
                    config=config,
                    point=zigzag_points[0],
                    tcp=config["coords"]["tcptool3plane1"],
                    ucs=config["coords"]["ucsTable1"],
                    seventh=-1,
                    speed=float(json_config["sandingSpeed"]),
                    wait=True
                )
                locked_orient = list(zigzag_points[0][3:6])
                if FORCE_SPIRAL_ORIENT:
                    locked_orient = list(FORCE_SPIRAL_ORIENT)
                putForceZminus(
                    cps=cps,
                    force=force,
                    tcp=config["coords"]["tcptool3plane1"],
                    ucs=config["coords"]["ucsTable1"],
                    config=config,
                    search_linear_velocity=force_seek_linear,
                    blending_timeout_s=force_blending_timeout,
                )
                if FORCE_SPIRAL_ORIENT is None:
                    locked_orient = _capture_locked_orient(
                        cps,
                        config,
                        tcp=config["coords"]["tcptool3plane1"],
                        ucs=config["coords"]["ucsTable1"],
                        fallback_orient=locked_orient,
                        settle_s=0.05,
                    )
                else:
                    if config.get("logger"):
                        config["logger"].info(f"[Orientation] forced for spiral: {locked_orient}")
                    else:
                        print(f"[Orientation] forced for spiral: {locked_orient}")
                turn_vibration_on(cps)
                use_waypoint2 = True
                wp2_cfg = Waypoint2Config(
                    speed=150.0,
                    accel=300.0,
                    radius=8.0,
                    min_seg_len=5.0,
                    min_angle_deg=12.0,
                    max_angle_deg=170.0,
                    use_arc=True,
                    use_wp2_for_line=True,
                    enforce_orientation="start",
                    wait_timeout_s=20.0,
                    cmd_id_prefix="zig",
                    line_cmd_id_prefix="zigL",
                )
                throttle_every = 8
                throttle_sleep_s = 0.01
                move_kwargs = {
                    "cps": cps,
                    "config": config,
                    "tcp": config["coords"]["tcptool3plane1"],
                    "ucs": config["coords"]["ucsTable1"],
                    "seventh": -1,
                    "speed": float(json_config["sandingSpeed"]),
                    "speed_mode": "linear",
                    "wait": True,
                }
                zigzag_points = [
                    [p[0], p[1], p[2], locked_orient[0], locked_orient[1], locked_orient[2]]
                    for p in zigzag_points
                ]
                bounds = None
                bounds_points = edge_points if edge_points else zigzag_points
                if bounds_points:
                    xs = [p[0] for p in bounds_points]
                    ys = [p[1] for p in bounds_points]
                    bounds = (min(xs), max(xs), min(ys), max(ys))
                    
                for index, _ in enumerate(zigzag_points):
                    point_A = zigzag_points[index]
                    if index + 1 >= len(zigzag_points):
                        break
                    point_B = zigzag_points[index + 1]

                    if use_waypoint2:
                        wp2_segments = generate_arc_line_segments_between(
                            point_A,
                            point_B,
                            radius=12.0,
                            arc_step_deg=120.0,
                            pitch=12.0,
                            clockwise=True,
                            bounds=bounds,
                            safety_margin=5.0,
                            min_radius=1.0,
                        )
                        last_pair = index == (len(zigzag_points) - 2)
                        for seg_idx, seg_points in enumerate(wp2_segments):
                            wait_end = last_pair and seg_idx == (len(wp2_segments) - 1)
                            wp2_result = execute_waypoint2_path(
                                cps,
                                seg_points,
                                tcp=config["coords"]["tcptool3plane1"],
                                ucs=config["coords"]["ucsTable1"],
                                cfg=wp2_cfg,
                                wait_each=False,
                                wait_end=wait_end,
                                throttle_every=throttle_every,
                                throttle_sleep_s=throttle_sleep_s,
                                move_l_fn=communicate,
                                move_l_kwargs=move_kwargs,
                                logger=config.get("logger"),
                            )
                            if not wp2_result.get("ok", False):
                                raise RuntimeError("[WayPoint2] Segment execution failed.")
                        continue
                    
                #motion done for zigzag, now turn off vibration and release force before next steps        
                turn_vibration_off(cps)
                releaseForce(cps=cps, config=config)
                    
                #     print("Spiral move from A to B:", point_A, "->", point_B)
                #     success, count = run_spiral_between_points(
                #         cps=cps,
                #         config=config,
                #         start_pose=point_A,
                #         end_pose=point_B,
                #         radius=12.0,
                #         angle_step_deg=45.0,
                #         track_name=spiral_track_name,
                #         velocity=150.0,
                #         accel=300.0,
                #         jerk=3000.0,
                #         init_path=not path_initialized,
                #         orientation=orientation
                #     )
                #     if not success:
                #         push_failed = True
                #         break

                #     path_initialized = True
                #     total_count += count

                # if path_initialized and not push_failed:
                #     timeout = compute_timeout(
                #         total_points=total_count, velocity=300.0 * 10.0 / 45.0
                #     )
                #     # Keep a short anti-false-positive runtime guard without
                #     # adding visible delay at the final point.
                #     min_runtime_s = max(0.15, min(0.6, timeout * 0.05))
                #     finalized = finalize_spiral_path(
                #         cps,
                #         spiral_track_name,
                #         box_id=0,
                #         robot_id=0,
                #         completion_timeout=timeout,
                #         min_runtime_s=min_runtime_s,
                #         force=force,
                #         config=config
                #     )
                #     if not finalized:
                #         raise RuntimeError("[Spiral] finalize_spiral_path failed for door 2.")

            # Wait for blending and turn off vibration
            # waitForBlending(cps=cps, config=config)
            #turn_vibration_off(cps)
            # Release force
            #releaseForce(cps=cps, config=config)

        if split and edge_coverage_pathp1:
            edge_start = edge_coverage_pathp1[0]
        else:
            edge_start = prepointp1

        if split:
            communicate(
                cps=cps,
                config=config,
                seventh=tcx0,
                tcp=config["coords"]["tcptool3plane1"],
                ucs=config["coords"]["ucsTable1"],
                speed=0.8,
                wait=True
            )
            if edge_prepointp1:
                communicate(
                    cps=cps,
                    config=config,
                    point=edge_prepointp1,
                    tcp=config["coords"]["tcptool3plane1"],
                    ucs=config["coords"]["ucsTable1"],
                    seventh=-1,
                    speed = 1.0,
                    wait=True
                )
            communicate(
                cps=cps,
                config=config,
                point=edge_start,
                tcp=config["coords"]["tcptool3plane1"],
                ucs=config["coords"]["ucsTable1"],
                seventh=-1,
                speed= 1.0,
                wait=True
            )
            perform_process_top(
                cps,
                config,
                edge_points=edge_coverage_pathp1,
                zigzag_points=[],
                force=force,
                run_edge_coverage=True
            )

        for idx, seventh_pos in enumerate(seventh_positions):
            first_split = split and idx == 0
            current_zigzag = zigzag_pathp1
            if split:
                current_zigzag = zigzag_pathp1_left if idx == 0 else zigzag_pathp1_right
            if not first_split:
                communicate(
                    cps=cps,
                    config=config,
                    seventh=seventh_pos,
                    tcp=config["coords"]["tcptool3plane1"],
                    ucs=config["coords"]["ucsTable1"],
                    speed=0.8,
                    wait=True
                )
                if not split:
                    communicate(
                        cps=cps,
                        config=config,
                        point=prepointp1,  # Dynamic prepoint
                        tcp=config["coords"]["tcptool3plane1"],
                        ucs=config["coords"]["ucsTable1"],
                        seventh=-1,
                        speed=1.0,
                        wait=True
                    )
            # turn_vibration_on(cps)
            perform_process_top(
                cps,
                config,
                edge_points=edge_coverage_pathp1 if not split else [],
                zigzag_points=current_zigzag,
                force=force,
                run_edge_coverage=not split
            )
            # Edge coverage + zigzag path
            # turn_vibration_off(cps)
            communicate(
                cps=cps,
                config=config,
                point=prehoming,
                tcp=config["coords"]["tcptool3plane1"],
                ucs=config["coords"]["ucsTable1"],
                seventh=-1,
                speed=0.8,
                wait=True
            )

    pocket_size = get_pocket_size(2, default_on_error=True)
    pocket_y = pocket_size["ylen"]
    pocket_x = pocket_size["xlen"]

    print("pocket_y:", pocket_y)
    print("pocket_x:", pocket_x)

    # Check conditions
    if pocket_y == "null" or pocket_x == "null":
        print("No door data available - skipping operations")
    elif isinstance(pocket_y, (int, float)) and isinstance(pocket_x, (int, float)):
        smalldoor2zizagsmall(force, z, cps, split=pocket_y > 600 and pocket_x > 290)
    elif isinstance(pocket_y, (int, float)) or isinstance(pocket_x, (int, float)):
        smalldoor2zizagsmall(force, z, cps, split=False)
    else:
        print(
            f"Invalid pocket size type: {type(pocket_x)}, {type(pocket_y)} - expected numbers or 'null'"
        )


def smalldoor3zizag(
    force, z, cps, orientation="vertical", movement="zigzag", spiral_settings=None
):
    apply_spiral_settings(spiral_settings)

    def smalldoor3zizagsmall(force, z, cps, split=False):
        # Load configuration from YAML
        config = load_config()
        json_config = load_json_config()

        # Set up logger
        config["logger"] = setup_logger(config["settings"]["debug"])

        # Establish connection with robot
        # cps = CPSClient()
        # IP = config['server']['cpip']
        # port = config['server']['cps']
        # ret = cps.HRIF_Connect(0, IP, port)

        # Main Points
        p8 = get_inner_corner_point(3, 0)
        print("p8:", p8)
        p7 = get_inner_corner_point(3, 1)
        print("p7:", p7)
        p6 = get_inner_corner_point(3, 2)
        print("p6:", p6)
        p5 = get_inner_corner_point(3, 3)
        print("p5:", p5)
        # 7th axis postion
        x1 = p8[0] + get_door_position(3)
        print("x1:", x1)
        tcx0 = x1
        tcx1 = tcx0 + ((p6[0] - p7[0]) / 2)
        seventh_positions = [tcx0, tcx1] if split else [x1]
        # prehoming
        prehoming = [0, 200, 50, 0, 0, 0]
        # #Depth of the poocket z
        # z=-6.5
        # zigzag points
        distance = p6[0] - p8[0]
        print("distance:", distance)
        # Points calculation
        point5 = [-p5[0], p5[1], z, -0.034, 0.556, 0.251]
        print("point5:", point5)
        point6 = [-p6[0], p6[1], z, -0.034, 0.556, 0.251]
        print("point6:", point6)
        point7 = [-p7[0], p7[1], z, -0.034, 0.556, 0.251]
        print("point7:", point7)
        point8 = [-p8[0], p8[1], z, -0.034, 0.556, 0.251]
        print("point8:", point8)

        # Final Points
        point5u = [-distance, point5[1], point5[2], point5[3], point5[4], point5[5]]
        print("point5u:", point5u)
        point6u = [-distance, point6[1], point6[2], point6[3], point6[4], point6[5]]
        print("point6u:", point6u)
        point7u = [0, point7[1], point7[2], point7[3], point7[4], point7[5]]
        print("point7u:", point7u)
        point8u = [0, point8[1], point8[2], point8[3], point8[4], point8[5]]
        print("point8u:", point8u)

        # 1) Collect boundary coordinates as [x, y, z]
        x_coords1 = [point5u[0], point6u[0], point7u[0], point8u[0]]
        y_coords1 = [point5u[1], point6u[1], point7u[1], point8u[1]]
        z_coords1 = [point5u[2], point6u[2], point7u[2], point8u[2]]

        print("x_coords1 ", x_coords1)
        print("y_coords2 ", y_coords1)
        print("z_coords3", z_coords1)

        # Global variables
        # prepoint = None
        # zigzag_coords = []

        # Second Pocket 1st Cycle
        x_coords_path = x_coords1
        if split:
            x_coords_path = [coord / 2 for coord in x_coords1]

        orientation_mode = (orientation or "horizontal").lower()
        inner_sanding_offset = 50.0
        middle_overlap = inner_sanding_offset * 0.5

        edge_coverage_pathp1, _, edge_prepointp1 = generate_zigzag_path(
            x_coords=x_coords1,
            y_coords=y_coords1,
            z_coords=z_coords1,
            innerOffset=17,
            innerOffsetX=17,
            orientation=orientation,
            movement=movement,
            innerSandingOffset=50,
            edge_coverage=True  # Enable edge coverage with MoveL before spiral
        )
        _, zigzag_pathp1, prepointp1 = generate_zigzag_path(
            x_coords=x_coords_path,
            y_coords=y_coords1,
            z_coords=z_coords1,
            innerOffset=17,
            innerOffsetX=17,
            orientation=orientation,
            movement=movement,
            innerSandingOffset=50,
            edge_coverage=False
        )
        zigzag_pathp1_left = zigzag_pathp1
        zigzag_pathp1_right = zigzag_pathp1
        if split:
            _, zigzag_pathp1_left, _ = generate_zigzag_path(
                x_coords=x_coords_path,
                y_coords=y_coords1,
                z_coords=z_coords1,
                innerOffset=17,
                innerOffsetX=17,
                x_max_offset=middle_overlap,
                orientation=orientation,
                movement=movement,
                innerSandingOffset=50,
                edge_coverage=False,
            )
            _, zigzag_pathp1_right, _ = generate_zigzag_path(
                x_coords=x_coords_path,
                y_coords=y_coords1,
                z_coords=z_coords1,
                innerOffset=17,
                innerOffsetX=17,
                x_min_offset=-middle_overlap,
                orientation=orientation,
                movement=movement,
                innerSandingOffset=50,
                edge_coverage=False,
            )
        print("edge_coverage_pathp1=", edge_coverage_pathp1)
        print("zigzag_pathp=", zigzag_pathp1)
        print("prepointp:", prepointp1)

        def perform_process_top(
            cps, config, edge_points, zigzag_points, force, run_edge_coverage=True
        ):
            force_seek_linear = 5.0
            force_blending_timeout = 7.0
            if split:
                force_blending_timeout = 0.4
            has_edge = run_edge_coverage and edge_points and len(edge_points) > 0
            if has_edge:
                force_seek_linear = 5.0
                force_blending_timeout = 7.0
                if split:
                    force_blending_timeout = 0.4
                putForceZminus(
                    cps=cps,
                    force=force,
                    tcp=config["coords"]["tcptool3plane1"],
                    ucs=config["coords"]["ucsTable1"],
                    config=config,
                    search_linear_velocity=force_seek_linear,
                    blending_timeout_s=force_blending_timeout,
                )

            # Step 1: Edge coverage with MoveL (linear path between modified points)
            if has_edge:
                print(
                    f"[Edge Coverage] Starting linear MoveL for {len(edge_points)} edge points"
                )
                edge_speed = float(json_config["sandingSpeed"]) * 0.3
                for point in edge_points:
                    communicate(
                        cps=cps,
                        config=config,
                        point=point,
                        tcp=config["coords"]["tcptool3plane1"],
                        ucs=config["coords"]["ucsTable1"],
                        seventh=-1,
                        speed=edge_speed,
                        speed_mode="linear",
                        wait=False
                    )
                    if point == edge_points[-1]:
                        communicate(
                            cps=cps,
                            config=config,
                            point=point,
                            tcp=config["coords"]["tcptool3plane1"],
                            ucs=config["coords"]["ucsTable1"],
                            seventh=-1,
                            speed=edge_speed,
                            speed_mode="linear",
                            wait=True
                        )
                    turn_vibration_on(cps)
                # Wait for edge coverage to complete before starting spiral
                waitForBlending(cps=cps, config=config)
                print("[Edge Coverage] Completed linear edge path")
                releaseForce(cps=cps, config=config)
                turn_vibration_off(cps)

            # Step 2: Zigzag/Spiral motion
            if zigzag_points and len(zigzag_points) > 0:
                spiral_track_name = f"small_{uuid.uuid4().hex[:6]}"
                path_initialized = False
                push_failed = False
                total_count = 0
                
                # Move to first zigzag point to ensure proper transition from edge coverage
                print("[Spiral] Moving to first zigzag point:", zigzag_points[0])
                communicate(
                    cps=cps,
                    config=config,
                    point=zigzag_points[0],
                    tcp=config["coords"]["tcptool3plane1"],
                    ucs=config["coords"]["ucsTable1"],
                    seventh=-1,
                    speed=float(json_config["sandingSpeed"]),
                    wait=True
                )
                locked_orient = list(zigzag_points[0][3:6])
                if FORCE_SPIRAL_ORIENT:
                    locked_orient = list(FORCE_SPIRAL_ORIENT)
                putForceZminus(
                    cps=cps,
                    force=force,
                    tcp=config["coords"]["tcptool3plane1"],
                    ucs=config["coords"]["ucsTable1"],
                    config=config,
                    search_linear_velocity=force_seek_linear,
                    blending_timeout_s=force_blending_timeout,
                )
                if FORCE_SPIRAL_ORIENT is None:
                    locked_orient = _capture_locked_orient(
                        cps,
                        config,
                        tcp=config["coords"]["tcptool3plane1"],
                        ucs=config["coords"]["ucsTable1"],
                        fallback_orient=locked_orient,
                        settle_s=0.05,
                    )
                else:
                    if config.get("logger"):
                        config["logger"].info(f"[Orientation] forced for spiral: {locked_orient}")
                    else:
                        print(f"[Orientation] forced for spiral: {locked_orient}")
                turn_vibration_on(cps)
                use_waypoint2 = True
                wp2_cfg = Waypoint2Config(
                    speed=150.0,
                    accel=300.0,
                    radius=8.0,
                    min_seg_len=5.0,
                    min_angle_deg=12.0,
                    max_angle_deg=170.0,
                    use_arc=True,
                    use_wp2_for_line=True,
                    enforce_orientation="start",
                    wait_timeout_s=20.0,
                    cmd_id_prefix="zig",
                    line_cmd_id_prefix="zigL",
                )
                throttle_every = 8
                throttle_sleep_s = 0.01
                move_kwargs = {
                    "cps": cps,
                    "config": config,
                    "tcp": config["coords"]["tcptool3plane1"],
                    "ucs": config["coords"]["ucsTable1"],
                    "seventh": -1,
                    "speed": float(json_config["sandingSpeed"]),
                    "speed_mode": "linear",
                    "wait": True,
                }
                zigzag_points = [
                    [p[0], p[1], p[2], locked_orient[0], locked_orient[1], locked_orient[2]]
                    for p in zigzag_points
                ]
                bounds = None
                bounds_points = edge_points if edge_points else zigzag_points
                if bounds_points:
                    xs = [p[0] for p in bounds_points]
                    ys = [p[1] for p in bounds_points]
                    bounds = (min(xs), max(xs), min(ys), max(ys))
                
                for index, _ in enumerate(zigzag_points):
                    point_A = zigzag_points[index]
                    if index + 1 >= len(zigzag_points):
                        break
                    point_B = zigzag_points[index + 1]
                    
                    if use_waypoint2:
                        wp2_segments = generate_arc_line_segments_between(
                            point_A,
                            point_B,
                            radius=12.0,
                            arc_step_deg=120.0,
                            pitch=12.0,
                            clockwise=True,
                            bounds=bounds,
                            safety_margin=5.0,
                            min_radius=1.0,
                        )
                        last_pair = index == (len(zigzag_points) - 2)
                        for seg_idx, seg_points in enumerate(wp2_segments):
                            wait_end = last_pair and seg_idx == (len(wp2_segments) - 1)
                            wp2_result = execute_waypoint2_path(
                                cps,
                                seg_points,
                                tcp=config["coords"]["tcptool3plane1"],
                                ucs=config["coords"]["ucsTable1"],
                                cfg=wp2_cfg,
                                wait_each=False,
                                wait_end=wait_end,
                                throttle_every=throttle_every,
                                throttle_sleep_s=throttle_sleep_s,
                                move_l_fn=communicate,
                                move_l_kwargs=move_kwargs,
                                logger=config.get("logger"),
                            )
                            if not wp2_result.get("ok", False):
                                raise RuntimeError("[WayPoint2] Segment execution failed.")
                        continue
                    
                #motion done for zigzag, now turn off vibration and release force before next steps        
                turn_vibration_off(cps)
                releaseForce(cps=cps, config=config)

                #     print("Spiral move from A to B:", point_A, "->", point_B)
                #     success, count = run_spiral_between_points(
                #         cps=cps,
                #         config=config,
                #         start_pose=point_A,
                #         end_pose=point_B,
                #         radius=12.0,
                #         angle_step_deg=45.0,
                #         track_name=spiral_track_name,
                #         velocity=150.0,
                #         accel=300.0,
                #         jerk=3000.0,
                #         init_path=not path_initialized,
                #         orientation=orientation
                #     )
                #     if not success:
                #         push_failed = True
                #         break

                #     path_initialized = True
                #     total_count += count

                # if path_initialized and not push_failed:
                #     timeout = compute_timeout(
                #         total_points=total_count, velocity=300.0 * 10.0 / 45.0
                #     )
                #     # Keep a short anti-false-positive runtime guard without
                #     # adding visible delay at the final point.
                #     min_runtime_s = max(0.15, min(0.6, timeout * 0.05))
                #     finalized = finalize_spiral_path(
                #         cps,
                #         spiral_track_name,
                #         box_id=0,
                #         robot_id=0,
                #         completion_timeout=timeout,
                #         min_runtime_s=min_runtime_s,
                #         force = force,
                #         config= config
                #     )
                #     if not finalized:
                #         raise RuntimeError("[Spiral] finalize_spiral_path failed for door 3.")

            # Wait for blending and turn off vibration
            #waitForBlending(cps=cps, config=config)
            #turn_vibration_off(cps)
            # Release force
            #releaseForce(cps=cps, config=config)

        if split and edge_coverage_pathp1:
            edge_start = edge_coverage_pathp1[0]
        else:
            edge_start = prepointp1

        if split:
            communicate(
                cps=cps,
                config=config,
                seventh=tcx0,
                tcp=config["coords"]["tcptool3plane1"],
                ucs=config["coords"]["ucsTable1"],
                speed=0.8,
                wait=True
            )
            if edge_prepointp1:
                communicate(
                    cps=cps,
                    config=config,
                    point=edge_prepointp1,
                    tcp=config["coords"]["tcptool3plane1"],
                    ucs=config["coords"]["ucsTable1"],
                    seventh=-1,
                    speed=1.0,
                    wait=True
                )
            communicate(
                cps=cps,
                config=config,
                point=edge_start,
                tcp=config["coords"]["tcptool3plane1"],
                ucs=config["coords"]["ucsTable1"],
                seventh=-1,
                speed=1.0,
                wait=True
            )
            perform_process_top(
                cps,
                config,
                edge_points=edge_coverage_pathp1,
                zigzag_points=[],
                force=force,
                run_edge_coverage=True
            )

        for idx, seventh_pos in enumerate(seventh_positions):
            first_split = split and idx == 0
            current_zigzag = zigzag_pathp1
            if split:
                current_zigzag = zigzag_pathp1_left if idx == 0 else zigzag_pathp1_right
            if not first_split:
                communicate(
                    cps=cps,
                    config=config,
                    seventh=seventh_pos,
                    tcp=config["coords"]["tcptool3plane1"],
                    ucs=config["coords"]["ucsTable1"],
                    speed=0.8,
                    wait=True
                )
                if not split:
                    communicate(
                        cps=cps,
                        config=config,
                        point=prepointp1,  # Dynamic prepoint
                        tcp=config["coords"]["tcptool3plane1"],
                        ucs=config["coords"]["ucsTable1"],
                        seventh=-1,
                        speed=1.0,
                        wait=True
                    )
            # # turn_vibration_on(cps)
            perform_process_top(
                cps,
                config,
                edge_points=edge_coverage_pathp1 if not split else [],
                zigzag_points=current_zigzag,
                force=force,
                run_edge_coverage=not split
            )  # Edge coverage + zigzag path
            # # turn_vibration_off(cps)
            communicate(
                cps=cps,
                config=config,
                point=prehoming,
                tcp=config["coords"]["tcptool3plane1"],
                ucs=config["coords"]["ucsTable1"],
                seventh=-1,
                speed=0.8,
                wait=True
            )

    pocket_size = get_pocket_size(3, default_on_error=True)
    pocket_y = pocket_size["ylen"]
    pocket_x = pocket_size["xlen"]

    print("pocket_y:", pocket_y)
    print("pocket_x:", pocket_x)

    # Check conditions
    if pocket_y == "null" or pocket_x == "null":
        print("No door data available - skipping operations")
    elif isinstance(pocket_y, (int, float)) and isinstance(pocket_x, (int, float)):
        smalldoor3zizagsmall(force, z, cps, split=pocket_y > 600 and pocket_x > 290)
    elif isinstance(pocket_y, (int, float)) or isinstance(pocket_x, (int, float)):
        smalldoor3zizagsmall(force, z, cps, split=False)
    else:
        print(
            f"Invalid pocket size type: {type(pocket_x)}, {type(pocket_y)} - expected numbers or 'null'"
        )


def smalldoor4zizag(
    force, z, cps, orientation="vertical", movement="zigzag", spiral_settings=None
):
    apply_spiral_settings(spiral_settings)

    def smalldoor4zizagsmall(force, z, cps, split=False):
        # Load configuration from YAML
        config = load_config()
        json_config = load_json_config()

        # Set up logger
        config["logger"] = setup_logger(config["settings"]["debug"])

        # Establish connection with robot
        # cps = CPSClient()
        # IP = config['server']['cpip']
        # port = config['server']['cps']
        # ret = cps.HRIF_Connect(0, IP, port)

        # Main Points
        p8 = get_inner_corner_point(4, 0)
        print("p8:", p8)
        p7 = get_inner_corner_point(4, 1)
        print("p7:", p7)
        p6 = get_inner_corner_point(4, 2)
        print("p6:", p6)
        p5 = get_inner_corner_point(4, 3)
        print("p5:", p5)
        # 7th axis postion
        x1 = p8[0] + get_door_position(4)
        print("x1:", x1)
        tcx0 = x1
        tcx1 = tcx0 + ((p6[0] - p7[0]) / 2)
        seventh_positions = [tcx0, tcx1] if split else [x1]
        # prehoming
        prehoming = [0, 200, 50, 0, 0, 0]
        # #Depth of the poocket z
        # z=-6.5
        # zigzag points
        distance = p6[0] - p8[0]
        print("distance:", distance)
        # Points calculation
        point5 = [-p5[0], p5[1], z, -0.034, 0.556, 0.251]
        print("point5:", point5)
        point6 = [-p6[0], p6[1], z, -0.034, 0.556, 0.251]
        print("point6:", point6)
        point7 = [-p7[0], p7[1], z, -0.034, 0.556, 0.251]
        print("point7:", point7)
        point8 = [-p8[0], p8[1], z, -0.034, 0.556, 0.251]
        print("point8:", point8)

        # Final Points
        point5u = [-distance, point5[1], point5[2], point5[3], point5[4], point5[5]]
        print("point5u:", point5u)
        point6u = [-distance, point6[1], point6[2], point6[3], point6[4], point6[5]]
        print("point6u:", point6u)
        point7u = [0, point7[1], point7[2], point7[3], point7[4], point7[5]]
        print("point7u:", point7u)
        point8u = [0, point8[1], point8[2], point8[3], point8[4], point8[5]]
        print("point8u:", point8u)

        # 1) Collect boundary coordinates as [x, y, z]
        x_coords1 = [point5u[0], point6u[0], point7u[0], point8u[0]]
        y_coords1 = [point5u[1], point6u[1], point7u[1], point8u[1]]
        z_coords1 = [point5u[2], point6u[2], point7u[2], point8u[2]]

        print("x_coords1 ", x_coords1)
        print("y_coords2 ", y_coords1)
        print("z_coords3", z_coords1)

        # Global variables
        # prepoint = None
        # zigzag_coords = []

        # Second Pocket 1st Cycle
        x_coords_path = x_coords1
        if split:
            x_coords_path = [coord / 2 for coord in x_coords1]

        orientation_mode = (orientation or "horizontal").lower()
        inner_sanding_offset = 50.0
        middle_overlap = inner_sanding_offset * 0.5

        edge_coverage_pathp1, _, edge_prepointp1 = generate_zigzag_path(
            x_coords=x_coords1,
            y_coords=y_coords1,
            z_coords=z_coords1,
            innerOffset=17,
            innerOffsetX=17,
            orientation=orientation,
            movement=movement,
            innerSandingOffset=50,
            edge_coverage=True  # Enable edge coverage with MoveL before spiral
        )
        _, zigzag_pathp1, prepointp1 = generate_zigzag_path(
            x_coords=x_coords_path,
            y_coords=y_coords1,
            z_coords=z_coords1,
            innerOffset=17,
            innerOffsetX=17,
            orientation=orientation,
            movement=movement,
            innerSandingOffset=50,
            edge_coverage=False
        )
        zigzag_pathp1_left = zigzag_pathp1
        zigzag_pathp1_right = zigzag_pathp1
        if split:
            _, zigzag_pathp1_left, _ = generate_zigzag_path(
                x_coords=x_coords_path,
                y_coords=y_coords1,
                z_coords=z_coords1,
                innerOffset=17,
                innerOffsetX=17,
                x_max_offset=middle_overlap,
                orientation=orientation,
                movement=movement,
                innerSandingOffset=50,
                edge_coverage=False,
            )
            _, zigzag_pathp1_right, _ = generate_zigzag_path(
                x_coords=x_coords_path,
                y_coords=y_coords1,
                z_coords=z_coords1,
                innerOffset=17,
                innerOffsetX=17,
                x_min_offset=-middle_overlap,
                orientation=orientation,
                movement=movement,
                innerSandingOffset=50,
                edge_coverage=False,
            )
        print("edge_coverage_pathp1=", edge_coverage_pathp1)
        print("zigzag_pathp=", zigzag_pathp1)
        print("prepointp:", prepointp1)

        def perform_process_top(
            cps, config, edge_points, zigzag_points, force, run_edge_coverage=True
        ):
            force_seek_linear = 5.0
            force_blending_timeout = 7.0
            if split:
                force_blending_timeout = 0.4
            has_edge = run_edge_coverage and edge_points and len(edge_points) > 0
            if has_edge:
                force_seek_linear = 5.0
                force_blending_timeout = 7.0
                if split:
                    force_blending_timeout = 0.4
                putForceZminus(
                    cps=cps,
                    force=force,
                    tcp=config["coords"]["tcptool3plane1"],
                    ucs=config["coords"]["ucsTable1"],
                    config=config,
                    search_linear_velocity=force_seek_linear,
                    blending_timeout_s=force_blending_timeout,
                )

            # Step 1: Edge coverage with MoveL (linear path between modified points)
            if has_edge:
                print(
                    f"[Edge Coverage] Starting linear MoveL for {len(edge_points)} edge points"
                )
                edge_speed = float(json_config["sandingSpeed"]) * 0.3
                for point in edge_points:
                    communicate(
                        cps=cps,
                        config=config,
                        point=point,
                        tcp=config["coords"]["tcptool3plane1"],
                        ucs=config["coords"]["ucsTable1"],
                        seventh=-1,
                        speed=edge_speed,
                        speed_mode="linear",
                        wait=False
                    )
                    if point == edge_points[-1]:
                        communicate(
                            cps=cps,
                            config=config,
                            point=point,
                            tcp=config["coords"]["tcptool3plane1"],
                            ucs=config["coords"]["ucsTable1"],
                            seventh=-1,
                            speed=edge_speed,
                            speed_mode="linear",
                            wait=True
                        )
                    turn_vibration_on(cps)
                # Wait for edge coverage to complete before starting spiral
                waitForBlending(cps=cps, config=config)
                print("[Edge Coverage] Completed linear edge path")
                releaseForce(cps=cps, config=config)
                turn_vibration_off(cps)

            # Step 2: Zigzag/Spiral motion
            if zigzag_points and len(zigzag_points) > 0:
                spiral_track_name = f"small_{uuid.uuid4().hex[:6]}"
                path_initialized = False
                push_failed = False
                total_count = 0
                
                # Move to first zigzag point to ensure proper transition from edge coverage
                print("[Spiral] Moving to first zigzag point:", zigzag_points[0])
                communicate(
                    cps=cps,
                    config=config,
                    point=zigzag_points[0],
                    tcp=config["coords"]["tcptool3plane1"],
                    ucs=config["coords"]["ucsTable1"],
                    seventh=-1,
                    speed=float(json_config["sandingSpeed"]),
                    wait=True
                )
                locked_orient = list(zigzag_points[0][3:6])
                if FORCE_SPIRAL_ORIENT:
                    locked_orient = list(FORCE_SPIRAL_ORIENT)

                putForceZminus(
                    cps=cps,
                    force=force,
                    tcp=config["coords"]["tcptool3plane1"],
                    ucs=config["coords"]["ucsTable1"],
                    config=config,
                    search_linear_velocity=force_seek_linear,
                    blending_timeout_s=force_blending_timeout,
                )
                if FORCE_SPIRAL_ORIENT is None:
                    locked_orient = _capture_locked_orient(
                        cps,
                        config,
                        tcp=config["coords"]["tcptool3plane1"],
                        ucs=config["coords"]["ucsTable1"],
                        fallback_orient=locked_orient,
                        settle_s=0.05,
                    )
                else:
                    if config.get("logger"):
                        config["logger"].info(f"[Orientation] forced for spiral: {locked_orient}")
                    else:
                        print(f"[Orientation] forced for spiral: {locked_orient}")
                turn_vibration_on(cps)
                use_waypoint2 = True
                wp2_cfg = Waypoint2Config(
                    speed=150.0,
                    accel=300.0,
                    radius=8.0,
                    min_seg_len=5.0,
                    min_angle_deg=12.0,
                    max_angle_deg=170.0,
                    use_arc=True,
                    use_wp2_for_line=True,
                    enforce_orientation="start",
                    wait_timeout_s=20.0,
                    cmd_id_prefix="zig",
                    line_cmd_id_prefix="zigL",
                )
                throttle_every = 8
                throttle_sleep_s = 0.01
                move_kwargs = {
                    "cps": cps,
                    "config": config,
                    "tcp": config["coords"]["tcptool3plane1"],
                    "ucs": config["coords"]["ucsTable1"],
                    "seventh": -1,
                    "speed": float(json_config["sandingSpeed"]),
                    "speed_mode": "linear",
                    "wait": True,
                }
                zigzag_points = [
                    [p[0], p[1], p[2], locked_orient[0], locked_orient[1], locked_orient[2]]
                    for p in zigzag_points
                ]
                bounds = None
                bounds_points = edge_points if edge_points else zigzag_points
                if bounds_points:
                    xs = [p[0] for p in bounds_points]
                    ys = [p[1] for p in bounds_points]
                    bounds = (min(xs), max(xs), min(ys), max(ys))
                    
                for index, _ in enumerate(zigzag_points):
                    point_A = zigzag_points[index]
                    if index + 1 >= len(zigzag_points):
                        break
                    point_B = zigzag_points[index + 1]

                    if use_waypoint2:
                        wp2_segments = generate_arc_line_segments_between(
                            point_A,
                            point_B,
                            radius=12.0,
                            arc_step_deg=120.0,
                            pitch=12.0,
                            clockwise=True,
                            bounds=bounds,
                            safety_margin=5.0,
                            min_radius=1.0,
                        )
                        last_pair = index == (len(zigzag_points) - 2)
                        for seg_idx, seg_points in enumerate(wp2_segments):
                            wait_end = last_pair and seg_idx == (len(wp2_segments) - 1)
                            wp2_result = execute_waypoint2_path(
                                cps,
                                seg_points,
                                tcp=config["coords"]["tcptool3plane1"],
                                ucs=config["coords"]["ucsTable1"],
                                cfg=wp2_cfg,
                                wait_each=False,
                                wait_end=wait_end,
                                throttle_every=throttle_every,
                                throttle_sleep_s=throttle_sleep_s,
                                move_l_fn=communicate,
                                move_l_kwargs=move_kwargs,
                                logger=config.get("logger"),
                            )
                            if not wp2_result.get("ok", False):
                                raise RuntimeError("[WayPoint2] Segment execution failed.")
                        continue
                    
                #motion done for zigzag, now turn off vibration and release force before next steps        
                turn_vibration_off(cps)
                releaseForce(cps=cps, config=config)    
                
                #     print("Spiral move from A to B:", point_A, "->", point_B)
                #     success, count = run_spiral_between_points(
                #         cps=cps,
                #         config=config,
                #         start_pose=point_A,
                #         end_pose=point_B,
                #         radius=12.0,
                #         angle_step_deg=45.0,
                #         track_name=spiral_track_name,
                #         velocity=150.0,
                #         accel=300.0,
                #         jerk=3000.0,
                #         init_path=not path_initialized,
                #         orientation=orientation
                #     )
                #     if not success:
                #         push_failed = True
                #         break

                #     path_initialized = True
                #     total_count += count

                # if path_initialized and not push_failed:
                #     timeout = compute_timeout(
                #         total_points=total_count, velocity=300.0 * 10.0 / 45.0
                #     )
                #     # Keep a short anti-false-positive runtime guard without
                #     # adding visible delay at the final point.
                #     min_runtime_s = max(0.15, min(0.6, timeout * 0.05))
                #     finalized = finalize_spiral_path(
                #         cps,
                #         spiral_track_name,
                #         box_id=0,
                #         robot_id=0,
                #         completion_timeout=timeout,
                #         min_runtime_s=min_runtime_s,
                #         force=force,
                #         config=config
                #     )
                #     if not finalized:
                #         raise RuntimeError("[Spiral] finalize_spiral_path failed for door 4.")

            # Wait for blending and turn off vibration
            #waitForBlending(cps=cps, config=config)
            #turn_vibration_off(cps)
            # Release force
            #releaseForce(cps=cps, config=config)
        
        if split and edge_coverage_pathp1:
            edge_start = edge_coverage_pathp1[0]
        else:
            edge_start = prepointp1

        if split:
            communicate(
                cps=cps,
                config=config,
                seventh=tcx0,
                tcp=config["coords"]["tcptool3plane1"],
                ucs=config["coords"]["ucsTable1"],
                speed=0.8,
                wait=True
            )
            if edge_prepointp1:
                communicate(
                    cps=cps,
                    config=config,
                    point=edge_prepointp1,
                    tcp=config["coords"]["tcptool3plane1"],
                    ucs=config["coords"]["ucsTable1"],
                    seventh=-1,
                    speed=1.0,
                    wait=True
                )
            communicate(
                cps=cps,
                config=config,
                point=edge_start,
                tcp=config["coords"]["tcptool3plane1"],
                ucs=config["coords"]["ucsTable1"],
                seventh=-1,
                speed=1.0,
                wait=True
            )
            perform_process_top(
                cps,
                config,
                edge_points=edge_coverage_pathp1,
                zigzag_points=[],
                force=force,
                run_edge_coverage=True
            )

        for idx, seventh_pos in enumerate(seventh_positions):
            first_split = split and idx == 0
            current_zigzag = zigzag_pathp1
            if split:
                current_zigzag = zigzag_pathp1_left if idx == 0 else zigzag_pathp1_right
            if not first_split:
                communicate(
                    cps=cps,
                    config=config,
                    seventh=seventh_pos,
                    tcp=config["coords"]["tcptool3plane1"],
                    ucs=config["coords"]["ucsTable1"],
                    speed=0.8,
                    wait=True
                )
                if not split:
                    communicate(
                        cps=cps,
                        config=config,
                        point=prepointp1,  # Dynamic prepoint
                        tcp=config["coords"]["tcptool3plane1"],
                        ucs=config["coords"]["ucsTable1"],
                        seventh=-1,
                        speed=1.0,
                        wait=True
                    )
            # # turn_vibration_on(cps)
            perform_process_top(
                cps,
                config,
                edge_points=edge_coverage_pathp1 if not split else [],
                zigzag_points=current_zigzag,
                force=force,
                run_edge_coverage=not split,
            )  # Edge coverage + zigzag path
            # # turn_vibration_off(cps)
            communicate(
                cps=cps,
                config=config,
                point=prehoming,
                tcp=config["coords"]["tcptool3plane1"],
                ucs=config["coords"]["ucsTable1"],
                seventh=-1,
                speed=0.8,
                wait=True
            )

    pocket_size = get_pocket_size(4, default_on_error=True)
    pocket_y = pocket_size["ylen"]
    pocket_x = pocket_size["xlen"]

    print("pocket_y:", pocket_y)
    print("pocket_x:", pocket_x)

    # Check conditions
    if pocket_y == "null" or pocket_x == "null":
        print("No door data available - skipping operations")
    elif isinstance(pocket_y, (int, float)) and isinstance(pocket_x, (int, float)):
        smalldoor4zizagsmall(force, z, cps, split=pocket_y > 600 and pocket_x > 290)
    elif isinstance(pocket_y, (int, float)) or isinstance(pocket_x, (int, float)):
        smalldoor4zizagsmall(force, z, cps, split=False)
    else:
        print(
            f"Invalid pocket size type: {type(pocket_x)}, {type(pocket_y)} - expected numbers or 'null'"
        )


if __name__ == "__main__":
    smalldoor1zizag(force=5, z=-6.5)
    smalldoor2zizag(force=5, z=-6.5)
    smalldoor3zizag(force=5, z=-6.5)
    # smalldoor4zizag(force=5,z=-6.5)# Uncommented to call smalldoor4zizag
