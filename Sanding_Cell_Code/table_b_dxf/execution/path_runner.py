from __future__ import annotations

from typing import Any

from Server_Better_V2 import (
    _get_tool_in_hand,
    _get_tool_in_hand_for_drop,
    communicate,
    getTool11,
    keepTool11,
    waitForBlending,
)

from .common import (
    TableBDxfRobotExecutionError,
    TableBDxfRobotStopRequested,
    log as _log,
    raise_if_stop_requested as _raise_if_stop_requested,
)
from .joint_safety import JointSafetyLimitError, guarded_move_j6_to_absolute, guarded_move_only_j6r
from .motion_config import robot_speed, table_b_ucs, tcp_for_physical_tool
from .tool2_recovery import (
    TOOL2_HOMING_J6_DEG,
    clear_tool2_recovery_state,
    recover_tool2_before_homing_if_needed,
    tool2_recovery_pending,
)
from .tool2_side_runner import run_tool2_side_batch
from .xy_force_runner import run_force_xy_path


TOOL_1_3_JOINT_CORRECTIONS: dict[tuple[int, int], dict[str, float]] = {
    (1, 3): {"J1": 30.0, "J6": -90.0},
    (3, 1): {"J1": -30.0, "J6": 90.0},
}


def _table_b_execution_value(config: dict[str, Any], key: str, default: float) -> float:
    value = None
    if isinstance(config, dict):
        motion_cfg = config.get("tableBDxfMotion")
        if isinstance(motion_cfg, dict):
            value = motion_cfg.get(key)
        if value is None:
            value = (config.get("door") or {}).get(key)
        if value is None:
            value = (config.get("settings") or {}).get(key)
    try:
        return float(value) if value is not None else float(default)
    except (TypeError, ValueError):
        return float(default)


def _xy_operation_family(step: dict[str, Any]) -> str:
    text = " ".join(
        str(step.get(key) or "")
        for key in ("tool", "operation", "path_id", "split_from_path_id")
    ).lower()
    if "frame" in text:
        return "frame"
    if "zigzag" in text or "pocketzigzag" in text or str(step.get("tool") or "") == "tool_4":
        return "pocket_zigzag"
    if "3d" in text:
        return "3d"
    if "edge" in text or str(step.get("tool") or "") == "tool_3":
        return "pocket_edge"
    return text or "unknown"


def _settle_between_xy_operation_families(
    cps: Any,
    config: dict[str, Any],
    previous_step: dict[str, Any],
    next_step: dict[str, Any],
) -> None:
    previous_family = _xy_operation_family(previous_step)
    next_family = _xy_operation_family(next_step)
    if previous_family == next_family:
        return
    timeout_s = max(
        0.0,
        _table_b_execution_value(config, "operationBoundaryBlendTimeoutSec", 0.8),
    )
    if timeout_s <= 0.0:
        return
    _log(
        config,
        "[TableB DXF Robot] operation boundary settle: %s -> %s before path %s",
        previous_family,
        next_family,
        next_step.get("path_id"),
    )
    waitForBlending(cps=cps, config=config, timeout_s=timeout_s)


def _normalize_tool_in_hand(raw_tool: Any) -> int | None:
    if raw_tool == 0:
        return None
    try:
        tool = int(raw_tool)
    except (TypeError, ValueError) as error:
        raise TableBDxfRobotExecutionError(
            f"Unable to read mounted tool state: {raw_tool!r}."
        ) from error
    if tool in (1, 2, 3, 4):
        return tool
    raise TableBDxfRobotExecutionError(
        f"Unknown mounted tool state from CI sensors: {raw_tool!r}."
    )


def _read_tool_in_hand(cps: Any) -> int | None:
    return _normalize_tool_in_hand(_get_tool_in_hand(cps))


def _read_tool_in_hand_for_drop(cps: Any) -> int | None:
    raw_tool = _get_tool_in_hand_for_drop(cps)
    if raw_tool is None:
        return None
    return _normalize_tool_in_hand(raw_tool)


def _j7_reference_tool(tool: int | None, fallback_tool: int = 3) -> int:
    # J7-only commands still require a TCP/UCS argument. Tool 2 side execution is
    # separate and may not share the XY force-runner TCP, so use a supported TCP as
    # the reference for rail-only moves.
    if tool in (1, 3, 4):
        return int(tool)
    return int(fallback_tool if fallback_tool in (1, 3, 4) else 3)


def _apply_tool_transition_correction(
    cps: Any, config: dict[str, Any], current_tool: int, requested_tool: int
) -> None:
    correction = TOOL_1_3_JOINT_CORRECTIONS.get((current_tool, requested_tool))
    if not correction:
        return
    _log(
        config,
        "[TableB DXF Robot] Tool %s -> Tool %s joint correction J1=%+.1f J6=%+.1f",
        current_tool,
        requested_tool,
        correction["J1"],
        correction["J6"],
    )
    guarded_move_only_j6r(
        cps,
        correction["J6"],
        config,
        J1=correction["J1"],
        wait=True,
        context=f"Tool {current_tool} -> Tool {requested_tool} transition correction",
    )


def _ensure_tool_in_hand(
    cps: Any, config: dict[str, Any], requested_tool: int, mounted_tool: int | None
) -> int:
    _raise_if_stop_requested(cps, config, "before tool-state read")
    di_fallback_tool = None
    try:
        sensor_tool = _read_tool_in_hand(cps)
    except TableBDxfRobotExecutionError as error:
        di_fallback_tool = _read_tool_in_hand_for_drop(cps)
        if di_fallback_tool is None:
            raise
        _log(
            config,
            "[TableB DXF Robot] CI could not identify mounted tool (%s); DI reports Tool %s off-station for recovery",
            error,
            di_fallback_tool,
        )
        sensor_tool = di_fallback_tool
    if mounted_tool is not None and sensor_tool != mounted_tool:
        _log(
            config,
            "[TableB DXF Robot] mounted-tool memory %s differs from CI sensor %s; using sensor",
            mounted_tool,
            sensor_tool,
        )
    current_tool = sensor_tool
    if current_tool == requested_tool:
        if di_fallback_tool == requested_tool:
            _return_j7_to_zero(cps, config, _j7_reference_tool(current_tool, requested_tool))
            keepTool11(
                cps,
                toolNumber=current_tool,
                config=config,
                goToSafe=True,
                startFromSafe=True,
                verify_release=False,
            )
            raise TableBDxfRobotExecutionError(
                f"Tool {requested_tool} is off-station but CI could not confirm it. "
                f"Tool {requested_tool} was returned; check Tool {requested_tool} sensor before retrying."
            )
        _log(config, "[TableB DXF Robot] Tool %s already mounted", requested_tool)
        return requested_tool
    _raise_if_stop_requested(cps, config, "before tool change")

    # All tool changes start with the rail at the calibrated tool station.
    _return_j7_to_zero(cps, config, _j7_reference_tool(current_tool, requested_tool))

    if current_tool is not None:
        _log(config, "[TableB DXF Robot] dropping Tool %s before picking Tool %s", current_tool, requested_tool)
        keepTool11(
            cps,
            toolNumber=current_tool,
            config=config,
            goToSafe=False,
            startFromSafe=True,
            verify_release=(di_fallback_tool != current_tool),
        )
        if di_fallback_tool != current_tool:
            dropped_state = _read_tool_in_hand(cps)
            if dropped_state is not None:
                raise TableBDxfRobotExecutionError(
                    f"Tool {current_tool} drop was not confirmed; CI still reports Tool {dropped_state}."
                )
        _apply_tool_transition_correction(cps, config, current_tool, requested_tool)
        start_from_safe = False
    else:
        _log(config, "[TableB DXF Robot] no tool mounted; picking Tool %s", requested_tool)
        start_from_safe = True

    success = getTool11(
        cps,
        toolNumber=requested_tool,
        config=config,
        startFromSafe=start_from_safe,
        exitToSafe=True,
    )
    if not success:
        raise TableBDxfRobotExecutionError(f"Tool {requested_tool} pick failed.")
    picked_state = _read_tool_in_hand(cps)
    if picked_state != requested_tool:
        raise TableBDxfRobotExecutionError(
            f"Tool {requested_tool} pick was not confirmed; CI reports {picked_state}."
        )
    return requested_tool


def _return_j7_to_zero(cps: Any, config: dict[str, Any], physical_tool: int) -> None:
    _raise_if_stop_requested(cps, config, "before J7 return-to-zero")
    reference_tool = _j7_reference_tool(physical_tool)
    tcp = tcp_for_physical_tool(config, reference_tool)
    ucs = table_b_ucs(config)
    _log(config, "[TableB DXF Robot] returning J7 to 0 mm after tool %s batch", physical_tool)
    result = communicate(
        cps=cps,
        config=config,
        point=None,
        tcp=tcp,
        ucs=ucs,
        seventh=0,
        speed=robot_speed(config),
        velocity_profile="robotspeed",
        wait=True,
        require_seventh_ok=True,
    )
    if result is None:
        raise TableBDxfRobotExecutionError(
            f"Failed to return J7 to 0 mm after tool {physical_tool} batch."
        )


def _move_arm_to_task_home(cps: Any, config: dict[str, Any], reason: str) -> None:
    _raise_if_stop_requested(cps, config, f"before arm home move ({reason})")
    _log(config, "[TableB DXF Robot] moving 6-axis arm directly to safePoint: %s", reason)
    result = communicate(
        cps=cps,
        config=config,
        point=config["point"]["safePoint"],
        tcp=config["coords"]["tcpDefault"],
        ucs=config["coords"]["ucsDefault"],
        seventh=-1,
        speed=robot_speed(config),
        velocity_profile="robotspeed",
        wait=True,
    )
    if result is None:
        raise TableBDxfRobotExecutionError("Failed to move 6-axis arm to safePoint.")


def _return_to_home_with_tool(cps: Any, config: dict[str, Any], physical_tool: int) -> None:
    """Return the rail and arm to home/safe while keeping the mounted tool."""
    if physical_tool == 2 and tool2_recovery_pending():
        _log(config, "[TableB DXF Robot] Tool 2 side-aware return: recover arm before J7/home.")
        recovered = recover_tool2_before_homing_if_needed(cps, config, clear_state=False)
        if not recovered:
            raise TableBDxfRobotExecutionError("Failed to recover Tool 2 before J7 return.")
        _move_arm_to_task_home(cps, config, "after Tool 2 side-aware recovery before J7 return")
        _return_j7_to_zero(cps, config, physical_tool)
        _move_arm_to_task_home(cps, config, "after Tool 2 side-aware J7 return")
        clear_tool2_recovery_state()
        return
    _move_arm_to_task_home(cps, config, "before J7 return")
    _return_j7_to_zero(cps, config, physical_tool)
    _move_arm_to_task_home(cps, config, "after J7 return")


def _recover_tool2_before_new_task_if_needed(cps: Any, config: dict[str, Any]) -> int | None:
    """Normalize a stopped Tool 2 pose before a new Table B task starts.

    Homing already uses this recovery. Start Task needs the same protection
    because an operator can press Stop, skip Homing, and immediately retry the
    task while Tool 2 is still near the door side/edge.
    """
    if not tool2_recovery_pending():
        return None

    _raise_if_stop_requested(cps, config, "before Tool 2 start-task recovery")
    try:
        mounted_tool = _read_tool_in_hand(cps)
    except TableBDxfRobotExecutionError as error:
        fallback_tool = _read_tool_in_hand_for_drop(cps)
        if fallback_tool != 2:
            raise TableBDxfRobotExecutionError(
                "Tool 2 recovery is pending from a stopped task, but the mounted tool "
                "cannot be confirmed as Tool 2. Run homing or check the tool sensor before starting."
            ) from error
        mounted_tool = 2

    if mounted_tool != 2:
        _log(
            config,
            "[TableB DXF Robot] clearing stale Tool 2 recovery state before start; mounted tool is %s",
            mounted_tool,
        )
        clear_tool2_recovery_state()
        return mounted_tool

    _log(config, "[TableB DXF Robot] Tool 2 recovery pending before Start Task; recovering before plan execution.")
    recovered = recover_tool2_before_homing_if_needed(cps, config)
    if not recovered:
        raise TableBDxfRobotExecutionError(
            "Tool 2 recovery is pending, but the safe recovery move could not be completed. "
            "Run homing before starting the task."
        )
    _return_to_home_with_tool(cps, config, 2)
    return 2


def run_robot_plan(cps: Any, config: dict[str, Any], plan: dict[str, Any]) -> dict[str, Any]:
    """Execute the resolved Table B DXF run plan, including tool changes."""
    if cps is None:
        raise TableBDxfRobotExecutionError("A connected CPS instance is required for real execution.")

    executed_steps = 0
    mounted_tool: int | None = _recover_tool2_before_new_task_if_needed(cps, config)
    for batch in plan.get("batches") or []:
        _raise_if_stop_requested(cps, config, "before tool batch")
        physical_tool = int(batch.get("physical_tool") or 0)
        if physical_tool not in (1, 2, 3, 4):
            raise TableBDxfRobotExecutionError(f"Unsupported physical tool batch: {physical_tool}")

        _log(config, "[TableB DXF Robot] === TOOL %s BATCH START ===", physical_tool)
        mounted_tool = _ensure_tool_in_hand(cps, config, physical_tool, mounted_tool)
        # Tool 2 opens with a -180 wrist flip. Do that from the arm's home pose,
        # not wherever the tool change left it: flipping at the tool-safe point
        # swings the tool through a pose that was never validated for it.
        _move_arm_to_task_home(cps, config, f"before tool {physical_tool} operation")

        steps = list(batch.get("steps") or [])
        try:
            if physical_tool == 2:
                executed_steps += run_tool2_side_batch(cps, config, steps)
            else:
                previous_xy_step: dict[str, Any] | None = None
                for step in steps:
                    _raise_if_stop_requested(cps, config, "before next path")
                    if previous_xy_step is not None:
                        _settle_between_xy_operation_families(
                            cps, config, previous_xy_step, step
                        )
                    run_force_xy_path(cps, config, step)
                    previous_xy_step = step
                    executed_steps += 1
        except JointSafetyLimitError:
            if physical_tool == 2:
                _log(
                    config,
                    "[TableB DXF Robot] Tool 2 joint safety limit reached; aborting task and stabilizing to homing.",
                )
                recovered = recover_tool2_before_homing_if_needed(cps, config)
                if not recovered:
                    guarded_move_j6_to_absolute(
                        cps,
                        TOOL2_HOMING_J6_DEG,
                        config,
                        wait=True,
                        context="Tool 2 joint-limit fallback absolute J6 homing",
                    )
                _return_to_home_with_tool(cps, config, physical_tool)
            raise

        _return_to_home_with_tool(cps, config, physical_tool)
        _log(config, "[TableB DXF Robot] === TOOL %s BATCH COMPLETE ===", physical_tool)

    _log(config, "[TableB DXF Robot] run complete; keeping mounted tool %s at J7=0", mounted_tool)
    return {"executed_steps": executed_steps, "tool_kept_in_hand": mounted_tool}
