from __future__ import annotations

from typing import Any

from Server_Better_V2 import moveOnlyJ6r

from .common import TableBDxfRobotExecutionError, log as _log

JOINT_LIMIT_MARGIN_DEG = 10.0
SAFE_J3_LIMIT_DEG = (-151.0, 151.0)

# Physical limits supplied for the robot. The safety window keeps 10 degrees
# away from each end so repeated relative wrist moves do not accumulate into
# a joint-limit crash.
JOINT_LIMITS_DEG: tuple[tuple[float, float], ...] = (
    (-360.0, 360.0),
    (-135.0, 135.0),
    (-153.0, 153.0),
    (-360.0, 360.0),
    (-180.0, 180.0),
    (-360.0, 360.0),
)

SAFE_JOINT_LIMITS_DEG: tuple[tuple[float, float], ...] = tuple(
    SAFE_J3_LIMIT_DEG if index == 2 else (low + JOINT_LIMIT_MARGIN_DEG, high - JOINT_LIMIT_MARGIN_DEG)
    for index, (low, high) in enumerate(JOINT_LIMITS_DEG)
)


class JointSafetyLimitError(TableBDxfRobotExecutionError):
    """Raised when a requested joint move would leave the safe joint window."""


def read_current_joints(cps: Any) -> list[float]:
    result: list[Any] = []
    try:
        ret = cps.HRIF_ReadActJointPos(0, 0, result)
    except Exception as error:
        raise TableBDxfRobotExecutionError("Unable to read robot joint position before joint safety check.") from error
    if ret != 0 or len(result) < 6:
        raise TableBDxfRobotExecutionError(
            f"Unable to read robot joint position before joint safety check: ret={ret} result={result!r}."
        )
    try:
        return [float(result[index]) for index in range(6)]
    except (TypeError, ValueError) as error:
        raise TableBDxfRobotExecutionError(
            f"Invalid robot joint values before joint safety check: {result!r}."
        ) from error


def _joint_name(index: int) -> str:
    return f"J{index + 1}"


def _joint_limit_violations(joints: list[float]) -> list[str]:
    violations: list[str] = []
    for index, value in enumerate(joints[:6]):
        low, high = SAFE_JOINT_LIMITS_DEG[index]
        if value < low or value > high:
            violations.append(f"{_joint_name(index)}={value:.3f} outside safe [{low:.1f}, {high:.1f}]")
    return violations


def _assert_joints_inside_safe_window(joints: list[float], *, context: str) -> None:
    violations = _joint_limit_violations(joints)
    if violations:
        raise JointSafetyLimitError(
            f"Joint safety blocked {context}: " + "; ".join(violations)
        )


def guarded_move_only_j6r(
    cps: Any,
    j6_delta: float,
    config: dict[str, Any],
    *,
    J1: float = 0.0,
    wait: bool = True,
    context: str = "relative J6 move",
) -> None:
    current = read_current_joints(cps)
    _assert_joints_inside_safe_window(current, context=f"{context} start")

    target = list(current)
    target[0] += float(J1)
    target[5] += float(j6_delta)
    _assert_joints_inside_safe_window(target, context=f"{context} target")

    _log(
        config,
        "[TableB DXF JointSafety] %s current=%s target=%s deltaJ1=%.3f deltaJ6=%.3f",
        context,
        [round(value, 3) for value in current],
        [round(value, 3) for value in target],
        float(J1),
        float(j6_delta),
    )
    moveOnlyJ6r(cps, j6_delta, config, J1=J1, wait=wait)


def _shortest_safe_j6_delta(current_j6: float, target_j6: float, *, context: str) -> float:
    """Pick the rotation direction that reaches the target pose with least travel.

    J6 spans [-360, 360], so a wrist pose is reachable at several joint values
    (target, target +- 360, ...). Taking the raw difference can sweep the long
    way round -- from J6=-270 to homing at +90 that is a full +360 turn when -0
    would do. Prefer the smallest move whose landing point is inside the safe
    window; fall back to any safe candidate if the smallest is not.
    """
    low, high = SAFE_JOINT_LIMITS_DEG[5]
    # Land on the target's own revolution when that is safe: homing expects the
    # wrist near TOOL2_HOMING_J6_DEG, and finishing a revolution away would eat
    # the headroom the next cycle's relative moves rely on. Only when that is
    # unreachable do we accept an equivalent pose one revolution out.
    preferred = target_j6 - current_j6
    if low <= target_j6 <= high and abs(preferred) <= 360.0:
        return preferred
    candidates = sorted(
        (target_j6 + turn * 360.0 - current_j6 for turn in (-1, 0, 1)),
        key=abs,
    )
    for delta in candidates:
        landing = current_j6 + delta
        if low <= landing <= high:
            return delta
    raise JointSafetyLimitError(
        f"Joint safety blocked {context}: no J6 rotation from {current_j6:.3f} reaches "
        f"{target_j6:.3f} inside safe [{low:.1f}, {high:.1f}]."
    )


def guarded_move_j6_to_absolute(
    cps: Any,
    target_j6: float,
    config: dict[str, Any],
    *,
    wait: bool = True,
    context: str = "absolute J6 stabilization",
) -> None:
    """Move J6 to a known safe absolute angle even if current J6 is near a limit."""
    current = read_current_joints(cps)
    target = list(current)
    target[5] = float(target_j6)
    _assert_joints_inside_safe_window(target, context=f"{context} target")

    j6_delta = _shortest_safe_j6_delta(current[5], float(target_j6), context=context)
    target[5] = current[5] + j6_delta
    _log(
        config,
        "[TableB DXF JointSafety] %s current=%s target=%s deltaJ6=%.3f",
        context,
        [round(value, 3) for value in current],
        [round(value, 3) for value in target],
        j6_delta,
    )
    moveOnlyJ6r(cps, j6_delta, config, wait=wait)
