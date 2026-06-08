# cc_client.socket_send_string('Hi I am the cobot', name)

from modules.final_async_reader import getRawHeight, getInstrument, scale_value
import copy
import math, csv
import yaml
import math, time
from modules.CPS import CPSClient
import numpy as np
import logging
from logging.handlers import TimedRotatingFileHandler
import os
import requests
import json
# from smallTable.scansmalltable import scanTableA


def waitForBlending(cps, config, timeout_s=None):
    door_cfg = config.get("door", {}) if isinstance(config, dict) else {}
    if timeout_s is None:
        # Default was 7s; reduce to 50% to shorten J7<->arm transition stalls.
        timeout_s = float(door_cfg.get("blendTimeoutSeconds", 3.5))
    timeout_s = max(0.2, float(timeout_s))
    settle_s = max(0.005, float(door_cfg.get("blendSettleDelaySec", 0.01)))
    poll_s = max(0.005, float(door_cfg.get("blendPollIntervalSec", 0.01)))
    start_time = time.time()
    status_ok = True
    while True:
        if stop_requested():
            return False
        result = []
        nret = cps.HRIF_IsBlendingDone(0, 0, result)
        done = False
        if nret != 0:
            # Some SDKs return 20018 when blending status is not available in
            # the current robot state. Fall back to robot-state completion
            # check instead of immediately continuing.
            if nret == 20018:
                robot_state = []
                nret_robot = cps.HRIF_ReadRobotState(0, 0, robot_state)
                robot_idle = (
                    nret_robot == 0
                    and isinstance(robot_state, (list, tuple))
                    and len(robot_state) > 11
                    and str(robot_state[11]).strip() == "1"
                )
                if robot_idle:
                    break
                if time.time() - start_time >= timeout_s:
                    if isinstance(config, dict) and config.get("logger"):
                        config["logger"].warning(
                            f"[waitForBlending] Timed out after {timeout_s}s (ret=20018, robot moving); continuing."
                        )
                    status_ok = False
                    break
                time.sleep(poll_s)
                continue
            if isinstance(config, dict) and config.get("logger"):
                config["logger"].warning(
                    f"[waitForBlending] HRIF_IsBlendingDone error (ret={nret}); continuing."
                )
                status_ok = False
            break
        if result:
            last = result[-1]
            if isinstance(last, bool):
                done = last
            else:
                done = str(last).strip().lower() in ("1", "true", "ok")
        if done:
            break
        if time.time() - start_time >= timeout_s:
            # Avoid blocking forever if the controller never reports done.
            if isinstance(config, dict) and config.get("logger"):
                config["logger"].warning(
                    f"[waitForBlending] Timed out after {timeout_s}s; continuing."
                )
            status_ok = False
            break
        time.sleep(poll_s)
    # Keep a minimal settle delay; larger fixed sleeps add visible latency
    # between chained point-to-point moves.
    time.sleep(settle_s)
    return status_ok
    # result = [False]
    # start_time = time.time()
    #
    # while True:
    #     nret = cps.HRIF_IsBlendingDone(0,0, result)
    #     print(f"Blending results : {result}", end='\r')
    #     if result[0]:
    #         break
    #     if time.time() - start_time >= 7:
    #         # Avoid blocking forever if the controller never reports done.
    #         if isinstance(config, dict) and config.get('logger'):
    #             config['logger'].warning("[waitForBlending] Timed out after 7s; continuing.")
    #         break
    # print("\n")
    # time.sleep(0.3)
    # return


def waitForMotion(cps, config):
    result = [False]

    while result[0] == False:
        nRet = cps.HRIF_IsMotionDone(0, 0, result)
        print(f"Is motion done: {result}", end="\r")
    print("\n")
    time.sleep(0.3)
    return


def load_json_config():
    """Loads configuration from config.json."""
    with open("./configs/cycleData.json", "r") as file:
        config = json.load(file)
    return config


def _as_float(value, fallback=None):
    try:
        return float(value)
    except (TypeError, ValueError):
        return fallback


def _resolve_ui_ratio(config, key, fallback=None):
    """
    Resolve speed ratio from cycleData.json first, then config['UI'], then fallback.
    Clamps to [0.0, 1.0].
    """
    value = None
    try:
        cycle_cfg = load_json_config()
        value = _as_float(cycle_cfg.get(key), None)
    except Exception:
        value = None

    if value is None and isinstance(config, dict):
        value = _as_float(config.get("UI", {}).get(key), None)

    if value is None:
        value = _as_float(fallback, 0.0)

    if value < 0.0:
        value = 0.0
    elif value > 1.0:
        value = 1.0
    return value


def msg_to_frontend(api_url, message, timeout_s=0.8):
    try:
        response = requests.post(api_url, json={"message": message}, timeout=timeout_s)
        response.raise_for_status()  # Raise exception for HTTP errors
        return response.json()
    except requests.exceptions.RequestException as e:
        print(f"Failed to send message to trigger API: {e}")
        return None


# Global stop flag shared by long-running operations.
STOP_REQUESTED = False
STOP_FLAG_PATH = os.path.join(os.path.dirname(__file__), '.stop_requested')


def request_stop():
    global STOP_REQUESTED
    STOP_REQUESTED = True
    try:
        with open(STOP_FLAG_PATH, 'w') as f:
            f.write('1')
    except Exception:
        pass


def clear_stop():
    global STOP_REQUESTED
    STOP_REQUESTED = False
    try:
        if os.path.exists(STOP_FLAG_PATH):
            os.remove(STOP_FLAG_PATH)
    except Exception:
        pass


def stop_requested() -> bool:
    if STOP_REQUESTED:
        return True
    try:
        return os.path.exists(STOP_FLAG_PATH)
    except Exception:
        return STOP_REQUESTED


def _stop_diagnostics():
    """
    Debug helper for stop-related behavior.
    Returns current stop flag states and runtime file path.
    """
    exists = False
    mtime = None
    try:
        exists = os.path.exists(STOP_FLAG_PATH)
        if exists:
            mtime = os.path.getmtime(STOP_FLAG_PATH)
    except Exception:
        exists = False
        mtime = None

    return {
        "stop_requested_mem": bool(STOP_REQUESTED),
        "stop_flag_path": STOP_FLAG_PATH,
        "stop_flag_exists": bool(exists),
        "stop_flag_mtime": mtime,
        "runtime_file": os.path.abspath(__file__),
    }


def _read_box_bit(cps, reader, bit_index):
    result = []
    nRet = reader(0, bit_index, result)
    if nRet != 0 or not result:
        return None
    try:
        return int(result[0])
    except (TypeError, ValueError):
        return None


def _read_tool_sensors(cps):
    return {
        "ci0": _read_box_bit(cps, cps.HRIF_ReadBoxCI, 0),
        "ci1": _read_box_bit(cps, cps.HRIF_ReadBoxCI, 1),
        "ci2": _read_box_bit(cps, cps.HRIF_ReadBoxCI, 2),
        "di4": _read_box_bit(cps, cps.HRIF_ReadBoxDI, 4),
        "di5": _read_box_bit(cps, cps.HRIF_ReadBoxDI, 5),
        "di6": _read_box_bit(cps, cps.HRIF_ReadBoxDI, 6),
        "di7": _read_box_bit(cps, cps.HRIF_ReadBoxDI, 7),
    }


def _decode_tool_from_ci(ci0, ci1, ci2):
    """Decode attached tool from CI bits only."""
    if any(val is None for val in (ci0, ci1, ci2)):
        return None
    if ci0 == 0 and ci1 == 0 and ci2 == 0:
        return 0
    if ci0 == 0 and ci1 == 1 and ci2 == 1:
        return 1
    if ci0 == 0 and ci1 == 1 and ci2 == 0:
        return 2
    if ci0 == 1 and ci1 == 0 and ci2 == 0:
        return 3
    if ci0 == 0 and ci1 == 0 and ci2 == 1:
        return 4
    return -1


def _get_tool_in_hand(cps):
    sensors = _read_tool_sensors(cps)
    ci0 = sensors["ci0"]
    ci1 = sensors["ci1"]
    ci2 = sensors["ci2"]
    di4 = sensors["di4"]
    di5 = sensors["di5"]
    di6 = sensors["di6"]
    di7 = sensors["di7"]

    # CI bits are the primary and sufficient source for tool decode.
    # Do not fail decode just because one DI read is temporarily missing.
    if any(val is None for val in (ci0, ci1, ci2)):
        return None

    # First, decode by CI bits.
    ci_decoded = _decode_tool_from_ci(ci0, ci1, ci2)
    if ci_decoded in (1, 2, 3, 4):
        return ci_decoded

    # DI fallback only when CI is ambiguous/invalid (not when CI says empty).
    if ci_decoded == -1 and all(val is not None for val in (di4, di5, di6, di7)):
        low_bits = [idx for idx, val in enumerate((di4, di5, di6, di7), start=4) if val == 0]
        if len(low_bits) == 1:
            if low_bits[0] == 7:
                return 1
            if low_bits[0] == 5:
                return 2
            if low_bits[0] == 6:
                return 3
            if low_bits[0] == 4:
                return 4

    if ci_decoded == 0:
        return 0

    # Fallback to CI decode if DI values are inconsistent/lagging.
    return ci_decoded


def _verify_tool_attached(cps, tool_number, config):
    if tool_number not in (1, 2, 3, 4):
        return True

    timeout_s = None
    poll_s = None
    try:
        timeout_s = float(config.get("tool", {}).get("signalWaitTimeoutSeconds"))
    except (TypeError, ValueError):
        timeout_s = None
    # Tool 2 often has a slower CI stabilization right after lift.
    if tool_number == 2:
        try:
            t2_timeout = float(config.get("tool", {}).get("signalWaitTimeoutTool2Seconds", 4.0))
            if timeout_s is None or t2_timeout > timeout_s:
                timeout_s = t2_timeout
        except (TypeError, ValueError):
            pass
    try:
        poll_s = float(config.get("tool", {}).get("signalPollIntervalSeconds"))
    except (TypeError, ValueError):
        poll_s = None

    # Backward-compatible fallback to retry settings.
    if timeout_s is None or poll_s is None:
        attempts = 3
        delay_s = 0.2
        try:
            attempts = int(config.get("tool", {}).get("sensorRetryAttempts", attempts))
        except (TypeError, ValueError):
            attempts = 3
        try:
            delay_s = float(config.get("tool", {}).get("sensorRetryDelay", delay_s))
        except (TypeError, ValueError):
            delay_s = 0.2
        poll_s = delay_s
        timeout_s = max(0.0, (attempts - 1) * delay_s)

    if poll_s is None or poll_s <= 0:
        poll_s = 0.05

    start = time.monotonic()
    detected = None
    while True:
        detected = _get_tool_in_hand(cps)
        if detected == tool_number:
            return True
        if timeout_s is not None and timeout_s >= 0:
            if time.monotonic() - start >= timeout_s:
                break
        else:
            break
        time.sleep(poll_s)

    if detected is None:
        msg_to_frontend(
            api_url=config["server"]["frontEnd_messaging_url"],
            message="Failed to read tool sensor inputs (CI/DI).",
        )
        return False

    sensors = _read_tool_sensors(cps)
    sensor_msg = (
        f"CI0={sensors['ci0']} CI1={sensors['ci1']} CI2={sensors['ci2']} "
        f"DI4={sensors['di4']} DI5={sensors['di5']} DI6={sensors['di6']} DI7={sensors['di7']}"
    )
    detected_label = "none" if detected == 0 else f"tool {detected}"
    msg_to_frontend(
        api_url=config["server"]["frontEnd_messaging_url"],
        message=(
            f"Tool {tool_number} not detected after pick (detected {detected_label}). "
            f"{sensor_msg}. Check the tool station or sensors."
        ),
    )
    return False


def _verify_tool_released(cps, config, expected_tool_number=None):
    timeout_s = None
    poll_s = None
    try:
        timeout_s = float(config.get("tool", {}).get("signalWaitTimeoutSeconds"))
    except (TypeError, ValueError):
        timeout_s = None
    try:
        poll_s = float(config.get("tool", {}).get("signalPollIntervalSeconds"))
    except (TypeError, ValueError):
        poll_s = None

    if timeout_s is None:
        timeout_s = 2.0
    if poll_s is None or poll_s <= 0:
        poll_s = 0.05

    start = time.monotonic()
    detected = None
    while True:
        detected = _get_tool_in_hand(cps)
        if detected == 0:
            return True
        if time.monotonic() - start >= timeout_s:
            break
        time.sleep(poll_s)

    sensors = _read_tool_sensors(cps)
    sensor_msg = (
        f"CI0={sensors['ci0']} CI1={sensors['ci1']} CI2={sensors['ci2']} "
        f"DI4={sensors['di4']} DI5={sensors['di5']} DI6={sensors['di6']} DI7={sensors['di7']}"
    )
    if detected is None:
        detected_label = "unknown (sensor read failed)"
    elif detected == 0:
        detected_label = "none"
    else:
        detected_label = f"tool {detected}"

    expected_msg = (
        f" (expected release of tool {expected_tool_number})"
        if expected_tool_number is not None
        else ""
    )
    msg_to_frontend(
        api_url=config["server"]["frontEnd_messaging_url"],
        message=(
            f"Tool release not confirmed{expected_msg}. "
            f"Detected {detected_label}. {sensor_msg}"
        ),
    )
    return False


def _tool_speed(config, key, default):
    try:
        return float(config.get("tool", {}).get(key, default))
    except (TypeError, ValueError):
        return default


def _tool_valve_delay(config, key, default=0.15):
    try:
        delay = float(config.get("tool", {}).get(key, default))
    except (TypeError, ValueError):
        delay = float(default)
    if delay < 0:
        delay = 0.0
    return delay



def setup_logger(
    enable_console_logging: bool = False,
    enable_file_logging: bool = True,
    log_file_name: str = "app.log",
    logger_name: str = "DualOutputLogger",
):
    """Configure a shared logger with optional console and rotating file outputs.

    Adds handlers only once per logger to avoid duplicate/locked file handles when
    multiple modules import this helper. If the primary log file cannot be opened
    (e.g., already locked by another process), it falls back to a per-process file.
    """
    logger = logging.getLogger(logger_name)
    logger.setLevel(logging.DEBUG)
    logger.propagate = False

    # Avoid attaching duplicate handlers if already configured in this process.
    if logger.handlers:
        return logger

    log_dir = "logs"
    os.makedirs(log_dir, exist_ok=True)
    log_file = os.path.join(log_dir, log_file_name)

    file_handler = None
    if enable_file_logging:
        try:
            file_handler = TimedRotatingFileHandler(
                filename=log_file,
                when="H",
                interval=1,
                backupCount=120,
                encoding="utf-8",
                delay=True,  # Open file lazily to reduce lock contention
            )
        except OSError as exc:
            base, ext = os.path.splitext(log_file_name)
            fallback_name = f"{base}_{os.getpid()}{ext or '.log'}"
            fallback_path = os.path.join(log_dir, fallback_name)
            try:
                file_handler = TimedRotatingFileHandler(
                    filename=fallback_path,
                    when="H",
                    interval=1,
                    backupCount=120,
                    encoding="utf-8",
                    delay=True,
                )
                print(
                    f"[setup_logger] {log_file} unavailable ({exc}); using {fallback_path} instead."
                )
            except OSError as exc2:
                print(
                    f"[setup_logger] Failed to open fallback log file {fallback_path}: {exc2}. File logging disabled."
                )
                file_handler = None

        if file_handler:
            file_handler.setLevel(logging.DEBUG)
            file_formatter = logging.Formatter(
                "%(asctime)s - %(levelname)s - [Line: %(lineno)d] - %(message)s"
            )
            file_handler.setFormatter(file_formatter)
            logger.addHandler(file_handler)

    if enable_console_logging:
        console_handler = logging.StreamHandler()
        console_handler.setLevel(logging.INFO)
        console_formatter = logging.Formatter(
            "%(asctime)s - %(levelname)s - [Line: %(lineno)d] - %(message)s"
        )
        console_handler.setFormatter(console_formatter)
        logger.addHandler(console_handler)

    return logger


def adjust_heights(data):
    """Balance out the heights to remove any angular tilt throughout the values
    ***Important*** removes all NaN entries.

    Args:
        data (list of dictionaries): The data (contains the dictionary format {'height': float, 'dist': float} in a list)

    Returns:
        list of dictionaries: the data with the tilt removed
    """
    # Step 1: Filter out dictionaries with non-NaN heights
    # non_nan_data = [d for d in data if not np.isnan(d['height'])]
    non_nan_heights = [d["height"] for d in data if not np.isnan(d["height"])]

    # Get the first five and last five non-NaN heights (why 5? This is experience, saw that 5 values were a good number)
    first_few_heights = non_nan_heights[:5]
    last_few_heights = non_nan_heights[
        -7:-3
    ]  # started from -3 because last few values from the laser are almost always outliers

    # Step 2: Calculate averages
    first_avg = np.mean(first_few_heights)
    last_avg = np.mean(last_few_heights)

    count = 0
    for i in range(-1, -len(data) - 1, -1):
        if not np.isnan(data[i]["height"]):
            data[i]["height"] = last_avg
            count += 1
        if count == 3:
            break

    # Step 3: Calculate the difference in averages
    avg_diff = last_avg - first_avg

    # Calculate the number of steps (dicts) between the first and last occurrences
    steps = len(non_nan_heights)

    # Step 5: Incrementally adjust heights
    adjusted_data = []
    cnt = 0
    for i, d in enumerate(data):
        if not np.isnan(d["height"]):
            cnt += 1
            # Calculate the incremental adjustment factor
            factor = (cnt / steps) if steps != 0 else 0
            # Adjust the height
            adjusted_height = d["height"] - factor * avg_diff
        else:
            adjusted_height = d["height"]  # Keep NaN as is
        adjusted_data.append({"dist": d["dist"], "height": adjusted_height})

    return adjusted_data


def setSpeed(cps, speed, config, wait_for_blending=False):
    """
    Sets the speed of the cobot.

    Args:
        cps: CPSClient instance
        speed (float): desired override speed, in [0,1]
        config (dict): must contain a 'logger' key
        wait_for_blending (bool): wait before applying override
    """
    if speed is None:
        return

    # Prepare an empty list to receive the current override speed
    currSpeed = []

    # 1) Read the current override; nRet == 0 means “success.”
    nRet = cps.HRIF_ReadOverride(0, 0, currSpeed)

    # ← added: if the read call failed, currSpeed may still be empty.
    if nRet != 0:
        config["logger"].error(
            f"[setSpeed] HRIF_ReadOverride failed (ret={nRet}), currSpeed={currSpeed!r}"
        )
        return

    # ← added: guard against an empty list before indexing currSpeed[0]
    if not currSpeed:
        config["logger"].error(
            "[setSpeed] HRIF_ReadOverride returned an empty currSpeed list."
        )
        return

    # ← added: wrap in try/except in case currSpeed[0] isn’t parseable
    try:
        current = float(currSpeed[0])
    except (ValueError, TypeError) as e:
        config["logger"].error(
            f"[setSpeed] couldn’t parse currSpeed[0]={currSpeed[0]!r} as float: {e}"
        )
        return

    # Detect override scale (older SDKs report 0-1, newer ones may report 0-100).
    override_scale = 100.0 if current > 1.5 else 1.0
    target_speed = speed * override_scale

    # If it’s already the requested speed, do nothing
    if abs(current - target_speed) < 1e-6:
        return

    # Otherwise, optionally wait for blending and set the new override
    if wait_for_blending:
        waitForBlending(cps=cps, config=config)
    # waitForMotion(cps=cps, config=config)

    nRet = cps.HRIF_SetOverride(0, 0, target_speed)
    if nRet == 0:
        config["logger"].info(
            f"[setSpeed] Could set speed to {target_speed:.1f} (scale={override_scale})"
        )
    else:
        if nRet == 20018:
            # Command prohibited in current state (often while moving).
            config["logger"].warning(
                f"[setSpeed] Override not allowed in current state (ret={nRet}); continuing."
            )
            return

        # Fallback: some SDKs expect 0-100 instead of 0-1 (or vice-versa).
        alt_speed = None
        if override_scale == 1.0 and speed <= 1.0:
            alt_speed = speed * 100.0
        elif override_scale == 100.0:
            alt_speed = speed

        if alt_speed is not None:
            # Clamp to a sane range to avoid invalid-argument errors.
            if alt_speed > 100.0:
                alt_speed = 100.0
            if 0 < alt_speed < 1.0:
                alt_speed = 1.0
            nRet2 = cps.HRIF_SetOverride(0, 0, alt_speed)
            if nRet2 == 0:
                config["logger"].info(
                    f"[setSpeed] Fallback override set to {alt_speed:.1f} (ret={nRet})"
                )
                return

        config["logger"].error(
            f"[setSpeed] Couldn't set speed to {target_speed:.1f} (scale={override_scale}, ret={nRet})"
        )
        msg_to_frontend(
            api_url=config["server"]["frontEnd_messaging_url"],
            message="Error With Robot Settings. Please Verify The Robot Settings and Try Again. Terminating Process...",
        )
        # exit(-1)

    return


def toggle_stopper_status(cps, digital_number=2):
    stopper_status = []
    nRet = cps.HRIF_ReadBoxDO(0, digital_number, stopper_status)

    # Check current status and toggle it
    if stopper_status[0] == "1":
        nRet = cps.HRIF_SetBoxDO(0, digital_number, 0)
    else:
        nRet = cps.HRIF_SetBoxDO(0, digital_number, 1)



def putForceYplus1edge(cps, force, tcp, ucs, config, goal=[0, 1, 0]):
    # Initialize parameters
    boxID = 0  # Control box ID
    rbtID = 0  # Robot ID

    result = []
    nret = 0

    waitForBlending(cps, config)
    setUCS_TCP(cps=cps, tcp=tcp, ucs=ucs, config=config)
    setSpeed(cps, speed=config["UI"]["sandSpeed"], config=config)

    nRet = cps.HRIF_SetForceZero(0, 0)
    if nRet != 0:
        config["logger"].error(f"Failed to set force zero: {nRet}")
        return

    # Set tool coordinate system mode for force control
    nret = cps.HRIF_SetForceToolCoordinateMotion(boxID, rbtID, 0)
    time.sleep(0.0001)
    config["logger"].info(f"forcetoolcoordinate: {nret}, result: {result}")
    if nret != 0:
        config["logger"].error(f"Failed to set force tool coordinate motion: {nret}")
        return

    # Set the force control strategy to constant force mode
    nret = cps.HRIF_SetForceControlStrategy(boxID, rbtID, 0)
    time.sleep(0.0001)
    config["logger"].info(f"force strategy: {nret}")
    if nret != 0:
        config["logger"].error(f"Failed to set force control strategy: {nret}")
        return

    # Define the target force control values (e.g., maintain fixed force in y and z axis)
    freedom = goal + [0, 0, 0]
    time.sleep(0.0001)
    cps.HRIF_SetControlFreedom(0, 0, freedom)  # force control degree of freedom

    # Set maximum search velocities for force control
    linear_velocity = 5  # 100 mm/s
    angular_velocity = 1  # 10 °/s
    nret = cps.HRIF_SetMaxSearchVelocities(
        boxID, rbtID, linear_velocity, angular_velocity
    )
    time.sleep(0.0001)
    config["logger"].info(f"search velocities: {nret}")
    if nret != 0:
        config["logger"].error(f"Failed to set max search velocities: {nret}")
        return

    # Set PID parameters to ensure stability in force control
    dFp = 0.8
    dFi = 0.001
    dFd = 0.02
    dTp = 0.8
    dTi = 0.001
    dTd = 0.02

    nRet = cps.HRIF_SetPIDControlParams(0, 0, dFp, dFi, dFd, dTp, dTi, dTd)
    time.sleep(0.0001)
    if nRet != 0:
        config["logger"].error(f"Failed to set PID control params: {nRet}")
        return

    # Set the Mass parameter
    # Mass = [80, 80, 80, 10, 10, 10]
    # nRet = cps.HRIF_SetMassParams(0, 0, Mass)
    # time.sleep(0.0001)
    # if nRet != 0:
    # config['logger'].error(f"Failed to set mass params: {nRet}")
    # return

    # Stiffness
    # Stiff = [1500, 1500, 1500, 100, 100, 100]
    # nRet = cps.HRIF_SetStiffParams(0,0,Stiff)
    # time.sleep(0.0001)
    # if nRet != 0:
    # config['logger'].error(f"Failed to set PID control params: {nRet}")
    # return

    # # Set the damp parameter
    # damp = [1600, 1600, 1600, 40, 40, 40]
    # nRet = cps.HRIF_SetDampParams(0, 0, damp)
    # time.sleep(0.0001)
    # if nRet != 0:
    #     config['logger'].error(f"Failed to set damp params: {nRet}")
    #     return

    force_goal = [
        force * goal[0],
        force * goal[1],
        force * goal[2],
        0,
        0,
        0,
        0,
    ]  # Target force: [X, Y, Z, Rx, Ry, Rz]
    nret = cps.HRIF_SetForceControlGoal(boxID, rbtID, force_goal)
    time.sleep(0.0001)
    config["logger"].info(f"[forceControl] force control goal: {nret}")
    if nret != 0:
        config["logger"].error(f"Failed to set force control goal: {nret}")
        return

    # Enable force control
    cps.HRIF_SetForceControlState(boxID, rbtID, 1)
    time.sleep(0.0001)

    notFound = True
    while notFound:
        if stop_requested():
            try:
                cps.HRIF_SetForceControlState(boxID, rbtID, 0)
            except Exception:
                pass
            config["logger"].info("[forceControl] Stop requested during force seek; aborting.")
            return
        result = []
        nRet = cps.HRIF_ReadFTCabData(0, 0, result)
        config["logger"].info(f"[forceControl] Force that is coming is: {result}")

        for i, val in enumerate(goal):
            if val and abs(float(result[i])) > abs(force):
                config["logger"].info(
                    f"[forceControl] Force condition met: Axis {i}, Force {result[i]}"
                )
                time.sleep(0.0001)
                notFound = False
                break

        time.sleep(0.0001)

    config["logger"].info(f"[forceControl] applying force: {force}N")
    # time.sleep(0.1)
    config["logger"].info(f"[forceControl] Turned on vibration")


def putForce(cps, force, tcp, ucs, config, goal=[0, 0, 1]):
    # Initialize parameters
    boxID = 0  # Control box ID
    rbtID = 0  # Robot ID

    result = []
    nret = 0

    waitForBlending(cps, config)
    setUCS_TCP(cps=cps, tcp=tcp, ucs=ucs, config=config)
    # waitForBlending(cps, config)
    setSpeed(cps, speed=config["UI"]["sandSpeed"], config=config)
    # msg_to_frontend(api_url=config['server']['frontEnd_messaging_url'], message=f"Turned on Force Control, Searching for Surface To Touch With {force}N")
    nRet = cps.HRIF_SetForceZero(0, 0)

    # Set tool coordinate system mode for force control
    nret = cps.HRIF_SetForceToolCoordinateMotion(boxID, rbtID, 0)
    time.sleep(0.0001)
    config["logger"].info(f"forcetoolcoordinate: {nret}, result: {result}")
    # Set the force control strategy to constant force mode
    nret = cps.HRIF_SetForceControlStrategy(boxID, rbtID, 0)
    time.sleep(0.0001)
    config["logger"].info(f"force strategy: {nret}")

    # Define the target force control values (e.g., maintain fixed force in y and z axis)
    freedom = goal + [0, 0, 0]
    time.sleep(0.0001)
    cps.HRIF_SetControlFreedom(0, 0, freedom)  # force control degree of freedom
    # Set maximum search velocities for force control
    linear_velocity = 5  # 100 mm/s
    angular_velocity = 1  # 10 °/s
    nret = cps.HRIF_SetMaxSearchVelocities(
        boxID, rbtID, linear_velocity, angular_velocity
    )
    time.sleep(0.0001)
    config["logger"].info(f"search velocities: {nret}")

    # SetthePIDparameter
    # Set PID parameters to ensure stability in force control
    dFp = 0.8
    dFi = 0.001
    dFd = 0.02
    dTp = 0.8
    dTi = 0.001
    dTd = 0.02

    # dFp=0.5
    # dFi=0.1
    # dFd=0
    # dTp=0.5
    # dTi=0.1
    # dTd=0
    # SetthePIDparameter
    nRet = cps.HRIF_SetPIDControlParams(0, 0, dFp, dFi, dFd, dTp, dTi, dTd)
    time.sleep(0.0001)
    damp = [2500, 2500, 2500, 40, 40, 40]
    nRet = cps.HRIF_SetDampParams(0, 0, damp)
    time.sleep(0.0001)

    force_goal = [
        force * goal[0],
        force * goal[1],
        force * goal[2],
        0,
        0,
        0,
        0,
    ]  # Target force: [X, Y, Z, Rx, Ry, Rz] changed by rafat for z minus is removed

    nret = cps.HRIF_SetForceControlGoal(boxID, rbtID, force_goal)
    time.sleep(0.0001)
    config["logger"].info(f"[forceControl] force control goal: {nret}")
    # Enable force control

    cps.HRIF_SetForceControlState(boxID, rbtID, 1)
    time.sleep(0.0001)

    notFound = True
    while notFound:
        if stop_requested():
            try:
                cps.HRIF_SetForceControlState(boxID, rbtID, 0)
            except Exception:
                pass
            config["logger"].info("[forceControl] Stop requested during force seek; aborting.")
            return
        result = []
        nRet = cps.HRIF_ReadFTCabData(0, 0, result)
        # config['logger'].info(f"[forceControl] Force that is coming is: {result}")
        # nRet = cps.HRIF_ReadForceControlState(0,0,result)
        # config['logger'].info(f"result_1 that is coming is: {result}")

        for i, val in enumerate(goal):
            if val and abs(float(result[i])) > abs(force):
                time.sleep(0.0001)
                notFound = False
                break

        time.sleep(0.0001)

    config["logger"].info(f"[forceControl] applying force: {force}N")
    # toggle_stopper_status(cps, digital_number=4)
    time.sleep(0.0001)
    config["logger"].info(f"[forceControl] Turned on vibration")
    # input("Proceed with force?")


def putForceZplus(cps, force, tcp, ucs, config, goal=[0, 0, 1]):
    # Initialize parameters
    boxID = 0  # Control box ID
    rbtID = 0  # Robot ID

    result = []
    nret = 0

    waitForBlending(cps, config)
    setUCS_TCP(cps=cps, tcp=tcp, ucs=ucs, config=config)
    setSpeed(cps, speed=config["UI"]["sandSpeed"], config=config)

    nRet = cps.HRIF_SetForceZero(0, 0)
    if nRet != 0:
        config["logger"].error(f"Failed to set force zero: {nRet}")
        return

    # Set tool coordinate system mode for force control
    nret = cps.HRIF_SetForceToolCoordinateMotion(boxID, rbtID, 0)
    time.sleep(0.0001)
    config["logger"].info(f"forcetoolcoordinate: {nret}, result: {result}")
    if nret != 0:
        config["logger"].error(f"Failed to set force tool coordinate motion: {nret}")
        return

    # Set the force control strategy to constant force mode
    nret = cps.HRIF_SetForceControlStrategy(boxID, rbtID, 0)
    time.sleep(0.0001)
    config["logger"].info(f"force strategy: {nret}")
    if nret != 0:
        config["logger"].error(f"Failed to set force control strategy: {nret}")
        return

    # Define the target force control values (e.g., maintain fixed force in y and z axis)
    freedom = goal + [0, 0, 0]
    time.sleep(0.0001)
    cps.HRIF_SetControlFreedom(0, 0, freedom)  # force control degree of freedom

    # Set maximum search velocities for force control
    linear_velocity = 5  # 100 mm/s
    angular_velocity = 1  # 10 °/s
    nret = cps.HRIF_SetMaxSearchVelocities(
        boxID, rbtID, linear_velocity, angular_velocity
    )
    time.sleep(0.0001)
    config["logger"].info(f"search velocities: {nret}")
    if nret != 0:
        config["logger"].error(f"Failed to set max search velocities: {nret}")
        return

    # Set PID parameters to ensure stability in force control
    dFp = 0.8
    dFi = 0.001
    dFd = 0.02
    dTp = 0.8
    dTi = 0.001
    dTd = 0.02

    # nRet = cps.HRIF_SetPIDControlParams(0, 0, dFp, dFi, dFd, dTp, dTi, dTd)
    # time.sleep(0.1)
    # if nRet != 0:
    # config['logger'].error(f"Failed to set PID control params: {nRet}")
    # return

    # # Set the Mass parameter
    # Mass = [80, 80, 80, 10, 10, 10]
    # nRet = cps.HRIF_SetMassParams(0, 0, Mass)
    # time.sleep(0.0001)
    # #if nRet != 0:
    #     #config['logger'].error(f"Failed to set mass params: {nRet}")
    #     #return

    # #Stiffness
    # Stiff = [1500, 1500, 1500, 100, 100, 100]
    # nRet = cps.HRIF_SetStiffParams(0,0,Stiff)
    # time.sleep(0.0001)
    # if nRet != 0:
    #     config['logger'].error(f"Failed to set PID control params: {nRet}")
    #     return

    # Set the damp parameter
    damp = [4000, 4000, 4000, 40, 40, 40]
    nRet = cps.HRIF_SetDampParams(0, 0, damp)
    time.sleep(0.0001)
    if nRet != 0:
        config["logger"].error(f"Failed to set damp params: {nRet}")
        return

    force_goal = [
        force * goal[0],
        force * goal[1],
        force * goal[2],
        0,
        0,
        0,
        0,
    ]  # Target force: [X, Y, Z, Rx, Ry, Rz]
    nret = cps.HRIF_SetForceControlGoal(boxID, rbtID, force_goal)
    time.sleep(0.0001)
    config["logger"].info(f"[forceControl] force control goal: {nret}")
    if nret != 0:
        config["logger"].error(f"Failed to set force control goal: {nret}")
        return

    # Enable force control
    cps.HRIF_SetForceControlState(boxID, rbtID, 1)
    time.sleep(0.0001)

    notFound = True
    while notFound:
        if stop_requested():
            try:
                cps.HRIF_SetForceControlState(boxID, rbtID, 0)
            except Exception:
                pass
            config["logger"].info("[forceControl] Stop requested during force seek; aborting.")
            return
        result = []
        nRet = cps.HRIF_ReadFTCabData(0, 0, result)
        config["logger"].info(f"[forceControl] Force that is coming is: {result}")

        for i, val in enumerate(goal):
            if val and abs(float(result[i])) > abs(force):
                config["logger"].info(
                    f"[forceControl] Force condition met: Axis {i}, Force {result[i]}"
                )
                time.sleep(0.0001)
                notFound = False
                break

        time.sleep(0.0001)

    config["logger"].info(f"[forceControl] applying force: {force}N")
    # time.sleep(0.1)
    config["logger"].info(f"[forceControl] Turned on vibration")


def putForceZminus(
    cps,
    force,
    tcp,
    ucs,
    config,
    goal=[0, 0, 1],
    search_linear_velocity=5.0,
    search_angular_velocity=1.0,
    blending_timeout_s=7.0,
    force_reach_ratio=0.9,
    max_seek_seconds=10.0,
):
    # Initialize parameters
    boxID = 0  # Control box ID
    rbtID = 0  # Robot ID

    result = []
    nret = 0

    waitForBlending(cps, config, timeout_s=blending_timeout_s)
    setUCS_TCP(cps=cps, tcp=tcp, ucs=ucs, config=config)
    json_config = load_json_config()
    setSpeed(cps, speed=float(json_config["sandingSpeed"]), config=config)

    nRet = cps.HRIF_SetForceZero(0, 0)
    if nRet != 0:
        config["logger"].error(f"Failed to set force zero: {nRet}")
        return False

    # Set tool coordinate system mode for force control
    nret = cps.HRIF_SetForceToolCoordinateMotion(boxID, rbtID, 0)
    time.sleep(0.0001)
    config["logger"].info(f"forcetoolcoordinate: {nret}, result: {result}")
    if nret != 0:
        config["logger"].error(f"Failed to set force tool coordinate motion: {nret}")
        return False

    # Set the force control strategy to constant force mode
    nret = cps.HRIF_SetForceControlStrategy(boxID, rbtID, 0)
    time.sleep(0.0001)
    config["logger"].info(f"force strategy: {nret}")
    if nret != 0:
        config["logger"].error(f"Failed to set force control strategy: {nret}")
        return False

    # Define the target force control values (e.g., maintain fixed force in y and z axis)
    freedom = goal + [0, 0, 0]
    time.sleep(0.0001)
    cps.HRIF_SetControlFreedom(0, 0, freedom)  # force control degree of freedom

    # Set maximum search velocities for force control
    linear_velocity = max(1.0, float(search_linear_velocity))
    angular_velocity = max(0.1, float(search_angular_velocity))
    nret = cps.HRIF_SetMaxSearchVelocities(
        boxID, rbtID, linear_velocity, angular_velocity
    )
    time.sleep(0.0001)
    config["logger"].info(
        f"search velocities: {nret} (linear={linear_velocity}, angular={angular_velocity})"
    )
    if nret != 0:
        config["logger"].error(f"Failed to set max search velocities: {nret}")
        return False

    # Set PID parameters to ensure stability in force control
    dFp = 0.8
    dFi = 0.001
    dFd = 0.02
    dTp = 0.8
    dTi = 0.001
    dTd = 0.02

    # nRet = cps.HRIF_SetPIDControlParams(0, 0, dFp, dFi, dFd, dTp, dTi, dTd)
    # time.sleep(0.1)
    # if nRet != 0:
    # config['logger'].error(f"Failed to set PID control params: {nRet}")
    # return

    # # Set the Mass parameter
    # Mass = [80, 80, 80, 10, 10, 10]
    # nRet = cps.HRIF_SetMassParams(0, 0, Mass)
    # time.sleep(0.0001)
    # #if nRet != 0:
    #     #config['logger'].error(f"Failed to set mass params: {nRet}")
    #     #return

    # #Stiffness
    # Stiff = [1500, 1500, 1500, 100, 100, 100]
    # nRet = cps.HRIF_SetStiffParams(0,0,Stiff)
    # time.sleep(0.0001)
    # if nRet != 0:
    #     config['logger'].error(f"Failed to set PID control params: {nRet}")
    #     return

    # Set the damp parameter
    damp = [8000, 8000, 8000, 40, 40, 40]
    nRet = cps.HRIF_SetDampParams(0, 0, damp)
    time.sleep(0.0001)
    if nRet != 0:
        config["logger"].error(f"Failed to set damp params: {nRet}")
        return False

    force_goal = [
        force * goal[0],
        force * goal[1],
        -force * goal[2],
        0,
        0,
        0,
        0,
    ]  # Target force: [X, Y, Z, Rx, Ry, Rz]
    nret = cps.HRIF_SetForceControlGoal(boxID, rbtID, force_goal)
    time.sleep(0.0001)
    config["logger"].info(f"[forceControl] force control goal: {nret}")
    if nret != 0:
        config["logger"].error(f"Failed to set force control goal: {nret}")
        return False

    # Enable force control
    cps.HRIF_SetForceControlState(boxID, rbtID, 1)
    time.sleep(0.0001)

    target_force = abs(float(force))
    reach_ratio = max(0.1, min(1.0, float(force_reach_ratio)))
    required_force = target_force * reach_ratio
    seek_start_t = time.time()

    notFound = True
    while notFound:
        if stop_requested():
            try:
                cps.HRIF_SetForceControlState(boxID, rbtID, 0)
            except Exception:
                pass
            config["logger"].info("[forceControl] Stop requested during force seek; aborting.")
            return False
        if (time.time() - seek_start_t) >= float(max_seek_seconds):
            try:
                cps.HRIF_SetForceControlState(boxID, rbtID, 0)
            except Exception:
                pass
            config["logger"].error(
                "[forceControl] Timeout during force seek after %.2fs (required=%.3fN).",
                float(max_seek_seconds),
                required_force,
            )
            return False

        result = []
        nRet = cps.HRIF_ReadFTCabData(0, 0, result)
        if nRet != 0 or len(result) < 3:
            time.sleep(0.001)
            continue
        config["logger"].info(f"[forceControl] Force that is coming is: {result}")

        for i, val in enumerate(goal):
            if not val:
                continue
            try:
                measured = abs(float(result[i]))
            except (TypeError, ValueError, IndexError):
                continue
            if measured >= required_force:
                config["logger"].info(
                    "[forceControl] Force condition met: Axis %s, Force %.3f (required %.3f, ratio %.2f)",
                    i,
                    measured,
                    required_force,
                    reach_ratio,
                )
                time.sleep(0.0001)
                notFound = False
                break

        time.sleep(0.0001)

    config["logger"].info(f"[forceControl] applying force: {force}N")
    # time.sleep(0.1)
    config["logger"].info(f"[forceControl] Turned on vibration")
    return True


def putForce3Direction(cps, force, tcp, ucs, config, goal=[1, 1, 1]):
    # Initialize parameters
    boxID = 0  # Control box ID
    rbtID = 0  # Robot ID

    result = []
    nret = 0

    waitForBlending(cps, config)
    setUCS_TCP(cps=cps, tcp=tcp, ucs=ucs, config=config)
    setSpeed(cps, speed=config["UI"]["sandSpeed"], config=config)

    nRet = cps.HRIF_SetForceZero(0, 0)
    if nRet != 0:
        config["logger"].error(f"Failed to set force zero: {nRet}")
        return

    # Set tool coordinate system mode for force control
    nret = cps.HRIF_SetForceToolCoordinateMotion(boxID, rbtID, 0)
    time.sleep(0.0001)
    config["logger"].info(f"forcetoolcoordinate: {nret}, result: {result}")
    if nret != 0:
        config["logger"].error(f"Failed to set force tool coordinate motion: {nret}")
        return

    # Set the force control strategy to constant force mode
    nret = cps.HRIF_SetForceControlStrategy(boxID, rbtID, 0)
    time.sleep(0.0001)
    config["logger"].info(f"force strategy: {nret}")
    if nret != 0:
        config["logger"].error(f"Failed to set force control strategy: {nret}")
        return

    # Define the target force control values (e.g., maintain fixed force in y and z axis)
    freedom = goal + [0, 0, 0]
    time.sleep(0.0001)
    cps.HRIF_SetControlFreedom(0, 0, freedom)  # force control degree of freedom

    # Set maximum search velocities for force control
    linear_velocity = 5  # 100 mm/s
    angular_velocity = 1  # 10 °/s
    nret = cps.HRIF_SetMaxSearchVelocities(
        boxID, rbtID, linear_velocity, angular_velocity
    )
    time.sleep(0.0001)
    config["logger"].info(f"search velocities: {nret}")
    if nret != 0:
        config["logger"].error(f"Failed to set max search velocities: {nret}")
        return

    # Set PID parameters to ensure stability in force control
    dFp = 0.8
    dFi = 0.001
    dFd = 0.02
    dTp = 0.8
    dTi = 0.001
    dTd = 0.02

    # nRet = cps.HRIF_SetPIDControlParams(0, 0, dFp, dFi, dFd, dTp, dTi, dTd)
    # time.sleep(0.0001)
    # if nRet != 0:
    # config['logger'].error(f"Failed to set PID control params: {nRet}")
    # return

    # Set the Mass parameter
    Mass = [80, 80, 80, 10, 10, 10]
    nRet = cps.HRIF_SetMassParams(0, 0, Mass)
    time.sleep(0.0001)
    if nRet != 0:
        config["logger"].error(f"Failed to set mass params: {nRet}")
        return

    # Stiffness
    Stiff = [1500, 1500, 1500, 100, 100, 100]
    nRet = cps.HRIF_SetStiffParams(0, 0, Stiff)
    time.sleep(0.0001)
    if nRet != 0:
        config["logger"].error(f"Failed to set PID control params: {nRet}")
        return

    # Set the damp parameter
    damp = [3500, 3500, 3500, 40, 40, 40]
    nRet = cps.HRIF_SetDampParams(0, 0, damp)
    time.sleep(0.0001)
    if nRet != 0:
        config["logger"].error(f"Failed to set damp params: {nRet}")
        return

    force_goal = [
        force[0] * goal[0],
        force[1] * goal[1],
        force[2] * goal[2],
        0,
        0,
        0,
        0,
    ]  # Target force: [X, Y, Z, Rx, Ry, Rz]
    nret = cps.HRIF_SetForceControlGoal(boxID, rbtID, force_goal)
    time.sleep(0.0001)
    config["logger"].info(f"[forceControl] force control goal: {nret}")
    if nret != 0:
        config["logger"].error(f"Failed to set force control goal: {nret}")
        return

    # Enable force control
    cps.HRIF_SetForceControlState(boxID, rbtID, 1)
    time.sleep(0.0001)

    notFound = True
    while notFound:
        if stop_requested():
            try:
                cps.HRIF_SetForceControlState(boxID, rbtID, 0)
            except Exception:
                pass
            config["logger"].info("[forceControl] Stop requested during force seek; aborting.")
            return
        result = []
        nRet = cps.HRIF_ReadFTCabData(0, 0, result)
        config["logger"].info(f"[forceControl] Force that is coming is: {result}")

        for i, val in enumerate(goal):
            if val and abs(float(result[i])) > abs(force):
                config["logger"].info(
                    f"[forceControl] Force condition met: Axis {i}, Force {result[i]}"
                )
                time.sleep(0.0001)
                notFound = False
                break

        time.sleep(0.0001)

    config["logger"].info(f"[forceControl] applying force: {force}N")
    # time.sleep(0.1)
    config["logger"].info(f"[forceControl] Turned on vibration")


def putForce3Directionup(cps, force, tcp, ucs, config, goal=[1, 1, 1]):
    # Initialize parameters
    boxID = 0  # Control box ID
    rbtID = 0  # Robot ID
    timeout = 0.0001  # Timeout in seconds
    start_time = time.time()
    success = False

    try:
        waitForBlending(cps, config)
        setUCS_TCP(cps=cps, tcp=tcp, ucs=ucs, config=config)
        setSpeed(cps, speed=config["UI"]["sandSpeed"], config=config)

        nRet = cps.HRIF_SetForceZero(boxID, rbtID)
        if nRet != 0:
            config["logger"].error(f"Failed to set force zero: {nRet}")
            return False

        # Set force control parameters
        nret = cps.HRIF_SetForceToolCoordinateMotion(boxID, rbtID, 0)
        if nret != 0:
            config["logger"].error(
                f"Failed to set force tool coordinate motion: {nret}"
            )
            return False

        nret = cps.HRIF_SetForceControlStrategy(boxID, rbtID, 0)
        if nret != 0:
            config["logger"].error(f"Failed to set force control strategy: {nret}")
            return False

        freedom = goal + [0, 0, 0]
        cps.HRIF_SetControlFreedom(boxID, rbtID, freedom)

        # Set velocities
        nret = cps.HRIF_SetMaxSearchVelocities(boxID, rbtID, 5, 1)
        if nret != 0:
            config["logger"].error(f"Failed to set max search velocities: {nret}")
            return False

        # Set dynamic parameters
        params = [
            ("HRIF_SetMassParams", [80, 80, 80, 10, 10, 10]),
            ("HRIF_SetStiffParams", [1500, 1500, 1500, 100, 100, 100]),
            ("HRIF_SetDampParams", [3500, 3500, 3500, 40, 40, 40]),
        ]

        for func, values in params:
            nRet = getattr(cps, func)(boxID, rbtID, values)
            if nRet != 0:
                config["logger"].error(f"Failed {func}: {nRet}")
                return False

        # Set force goal
        force_goal = [force[i] * goal[i] for i in range(3)] + [0, 0, 0, 0]
        nret = cps.HRIF_SetForceControlGoal(boxID, rbtID, force_goal)
        if nret != 0:
            config["logger"].error(f"Failed to set force control goal: {nret}")
            return False

        # Enable force control
        cps.HRIF_SetForceControlState(boxID, rbtID, 1)
        config["logger"].info(f"[forceControl] applying force: {force}N")

        # Monitor force
        while time.time() - start_time < timeout:
            result = []
            nRet = cps.HRIF_ReadFTCabData(boxID, rbtID, result)
            if nRet != 0:
                config["logger"].error(f"Failed to read force data: {nRet}")
                break

            config["logger"].info(f"[forceControl] Force that is coming is: {result}")

            # Check all enabled axes
            success = True
            for i, enabled in enumerate(goal[:3]):
                if enabled and abs(float(result[i])) < abs(force[i]):
                    config["logger"].info(
                        f"[forceControl] Force condition met: Axis {i}, Force {result[i]}"
                    )
                    success = False
                    break

            if success:
                config["logger"].info("[forceControl] Target force reached")
                break

            time.sleep(0.0001)  # Reduce CPU usage

        return success

    except Exception as e:
        config["logger"].error(f"Error in force control: {str(e)}")
        return False


def putForceYplus(cps, force, tcp, ucs, config, goal=[0, 1, 0]):
    # Initialize parameters
    boxID = 0  # Control box ID
    rbtID = 0  # Robot ID

    result = []
    nret = 0

    waitForBlending(cps, config)
    setUCS_TCP(cps=cps, tcp=tcp, ucs=ucs, config=config)
    # waitForBlending(cps, config)
    setSpeed(cps, speed=config["UI"]["sandSpeed"], config=config)
    # msg_to_frontend(api_url=config['server']['frontEnd_messaging_url'], message=f"Turned on Force Control, Searching for Surface To Touch With {force}N")
    nRet = cps.HRIF_SetForceZero(0, 0)

    # Set tool coordinate system mode for force control
    nret = cps.HRIF_SetForceToolCoordinateMotion(boxID, rbtID, 0)
    time.sleep(0.1)
    config["logger"].info(f"forcetoolcoordinate: {nret}, result: {result}")
    # Set the force control strategy to constant force mode
    nret = cps.HRIF_SetForceControlStrategy(boxID, rbtID, 0)
    time.sleep(0.1)
    config["logger"].info(f"force strategy: {nret}")

    # Define the target force control values (e.g., maintain fixed force in y and z axis)
    freedom = goal + [0, 0, 0]
    time.sleep(0.1)
    cps.HRIF_SetControlFreedom(0, 0, freedom)  # force control degree of freedom
    # Set maximum search velocities for force control
    linear_velocity = 100  # 100 mm/s
    angular_velocity = 50  # 10 °/s
    nret = cps.HRIF_SetMaxSearchVelocities(
        boxID, rbtID, linear_velocity, angular_velocity
    )
    time.sleep(0.1)
    config["logger"].info(f"search velocities: {nret}")

    # SetthePIDparameter
    # Set PID parameters to ensure stability in force control
    dFp = 1.00
    dFi = 0.100
    dFd = 0.000
    dTp = 1.00
    dTi = 0.100
    dTd = 0.000

    # dFp=0.5
    # dFi=0.1
    # dFd=0
    # dTp=0.5
    # dTi=0.1
    # dTd=0
    # SetthePIDparameter
    nRet = cps.HRIF_SetPIDControlParams(0, 0, dFp, dFi, dFd, dTp, dTi, dTd)
    time.sleep(0.1)
    damp = [1600, 1600, 1600, 40, 40, 40]
    nRet = cps.HRIF_SetDampParams(0, 0, damp)
    time.sleep(0.1)

    force_goal = [
        force * goal[0],
        force * goal[1],
        force * goal[2],
        0,
        0,
        0,
        0,
    ]  # Target force: [X, Y, Z, Rx, Ry, Rz] changed by rafat for z minus is removed

    nret = cps.HRIF_SetForceControlGoal(boxID, rbtID, force_goal)
    time.sleep(0.1)
    config["logger"].info(f"[forceControl] force control goal: {nret}")
    # Enable force control

    cps.HRIF_SetForceControlState(boxID, rbtID, 1)
    time.sleep(0.1)

    notFound = True
    while notFound:
        if stop_requested():
            try:
                cps.HRIF_SetForceControlState(boxID, rbtID, 0)
            except Exception:
                pass
            config["logger"].info("[forceControl] Stop requested during force seek; aborting.")
            return
        result = []
        nRet = cps.HRIF_ReadFTCabData(0, 0, result)
        # config['logger'].info(f"[forceControl] Force that is coming is: {result}")
        # nRet = cps.HRIF_ReadForceControlState(0,0,result)
        # config['logger'].info(f"result_1 that is coming is: {result}")

        for i, val in enumerate(goal):
            if val and abs(float(result[i])) > abs(force):
                time.sleep(0.2)
                notFound = False
                break

        time.sleep(0.1)

    config["logger"].info(f"[forceControl] applying force: {force}N")
    # toggle_stopper_status(cps, digital_number=4)
    time.sleep(0.1)
    config["logger"].info(f"[forceControl] Turned on vibration")
    # input("Proceed with force?")


def putForceYplus1(cps, force, tcp, ucs, config, goal=[0, 1, 0]):
    # Initialize parameters
    boxID = 0  # Control box ID
    rbtID = 0  # Robot ID

    result = []
    nret = 0

    waitForBlending(cps, config)
    setUCS_TCP(cps=cps, tcp=tcp, ucs=ucs, config=config)
    setSpeed(cps, speed=config["UI"]["sandSpeed"], config=config)

    nRet = cps.HRIF_SetForceZero(0, 0)
    if nRet != 0:
        config["logger"].error(f"Failed to set force zero: {nRet}")
        return

    # Set tool coordinate system mode for force control
    nret = cps.HRIF_SetForceToolCoordinateMotion(boxID, rbtID, 0)
    time.sleep(0.0001)
    config["logger"].info(f"forcetoolcoordinate: {nret}, result: {result}")
    if nret != 0:
        config["logger"].error(f"Failed to set force tool coordinate motion: {nret}")
        return

    # Set the force control strategy to constant force mode
    nret = cps.HRIF_SetForceControlStrategy(boxID, rbtID, 0)
    time.sleep(0.0001)
    config["logger"].info(f"force strategy: {nret}")
    if nret != 0:
        config["logger"].error(f"Failed to set force control strategy: {nret}")
        return

    # Define the target force control values (e.g., maintain fixed force in y and z axis)
    freedom = goal + [0, 0, 0]
    time.sleep(0.0001)
    cps.HRIF_SetControlFreedom(0, 0, freedom)  # force control degree of freedom

    # Set maximum search velocities for force control
    linear_velocity = 5  # 100 mm/s
    angular_velocity = 1  # 10 °/s
    nret = cps.HRIF_SetMaxSearchVelocities(
        boxID, rbtID, linear_velocity, angular_velocity
    )
    time.sleep(0.0001)
    config["logger"].info(f"search velocities: {nret}")
    if nret != 0:
        config["logger"].error(f"Failed to set max search velocities: {nret}")
        return

    # Set PID parameters to ensure stability in force control
    dFp = 0.8
    dFi = 0.001
    dFd = 0.02
    dTp = 0.8
    dTi = 0.001
    dTd = 0.02

    # nRet = cps.HRIF_SetPIDControlParams(0, 0, dFp, dFi, dFd, dTp, dTi, dTd)
    # time.sleep(0.1)
    # if nRet != 0:
    # config['logger'].error(f"Failed to set PID control params: {nRet}")
    # return

    # Set the Mass parameter
    Mass = [80, 80, 80, 10, 10, 10]
    nRet = cps.HRIF_SetMassParams(0, 0, Mass)
    time.sleep(0.0001)
    # if nRet != 0:
    # config['logger'].error(f"Failed to set mass params: {nRet}")
    # return

    # Stiffness
    Stiff = [1500, 1500, 1500, 100, 100, 100]
    nRet = cps.HRIF_SetStiffParams(0, 0, Stiff)
    time.sleep(0.0001)
    if nRet != 0:
        config["logger"].error(f"Failed to set PID control params: {nRet}")
        return

    # Set the damp parameter
    damp = [2500, 2500, 2500, 40, 40, 40]
    nRet = cps.HRIF_SetDampParams(0, 0, damp)
    time.sleep(0.0001)
    if nRet != 0:
        config["logger"].error(f"Failed to set damp params: {nRet}")
        return

    force_goal = [
        force * goal[0],
        force * goal[1],
        force * goal[2],
        0,
        0,
        0,
        0,
    ]  # Target force: [X, Y, Z, Rx, Ry, Rz]
    nret = cps.HRIF_SetForceControlGoal(boxID, rbtID, force_goal)
    time.sleep(0.0001)
    config["logger"].info(f"[forceControl] force control goal: {nret}")
    if nret != 0:
        config["logger"].error(f"Failed to set force control goal: {nret}")
        return

    # Enable force control
    cps.HRIF_SetForceControlState(boxID, rbtID, 1)
    time.sleep(0.0001)

    notFound = True
    while notFound:
        if stop_requested():
            try:
                cps.HRIF_SetForceControlState(boxID, rbtID, 0)
            except Exception:
                pass
            config["logger"].info("[forceControl] Stop requested during force seek; aborting.")
            return
        result = []
        nRet = cps.HRIF_ReadFTCabData(0, 0, result)
        config["logger"].info(f"[forceControl] Force that is coming is: {result}")

        for i, val in enumerate(goal):
            if val and abs(float(result[i])) > abs(force):
                config["logger"].info(
                    f"[forceControl] Force condition met: Axis {i}, Force {result[i]}"
                )
                time.sleep(0.0001)
                notFound = False
                break

        time.sleep(0.0001)

    config["logger"].info(f"[forceControl] applying force: {force}N")
    # time.sleep(0.1)
    config["logger"].info(f"[forceControl] Turned on vibration")


def putForceXplus(cps, force, tcp, ucs, config, goal=[1, 0, 0]):
    # Initialize parameters
    boxID = 0  # Control box ID
    rbtID = 0  # Robot ID

    result = []
    nret = 0

    waitForBlending(cps, config)
    setUCS_TCP(cps=cps, tcp=tcp, ucs=ucs, config=config)
    setSpeed(cps, speed=config["UI"]["sandSpeed"], config=config)

    nRet = cps.HRIF_SetForceZero(0, 0)
    if nRet != 0:
        config["logger"].error(f"Failed to set force zero: {nRet}")
        return

    # Set tool coordinate system mode for force control
    nret = cps.HRIF_SetForceToolCoordinateMotion(boxID, rbtID, 0)
    time.sleep(0.0001)
    config["logger"].info(f"forcetoolcoordinate: {nret}, result: {result}")
    if nret != 0:
        config["logger"].error(f"Failed to set force tool coordinate motion: {nret}")
        return

    # Set the force control strategy to constant force mode
    nret = cps.HRIF_SetForceControlStrategy(boxID, rbtID, 0)
    time.sleep(0.0001)
    config["logger"].info(f"force strategy: {nret}")
    if nret != 0:
        config["logger"].error(f"Failed to set force control strategy: {nret}")
        return

    # Define the target force control values (e.g., maintain fixed force in y and z axis)
    freedom = goal + [0, 0, 0]
    time.sleep(0.0001)
    cps.HRIF_SetControlFreedom(0, 0, freedom)  # force control degree of freedom

    # Set maximum search velocities for force control
    linear_velocity = 5  # 100 mm/s
    angular_velocity = 1  # 10 °/s
    nret = cps.HRIF_SetMaxSearchVelocities(
        boxID, rbtID, linear_velocity, angular_velocity
    )
    time.sleep(0.0001)
    config["logger"].info(f"search velocities: {nret}")
    if nret != 0:
        config["logger"].error(f"Failed to set max search velocities: {nret}")
        return

    # Set PID parameters to ensure stability in force control
    dFp = 0.8
    dFi = 0.001
    dFd = 0.02
    dTp = 0.8
    dTi = 0.001
    dTd = 0.02

    # nRet = cps.HRIF_SetPIDControlParams(0, 0, dFp, dFi, dFd, dTp, dTi, dTd)
    # time.sleep(0.1)
    # if nRet != 0:
    # config['logger'].error(f"Failed to set PID control params: {nRet}")
    # return

    # Set the Mass parameter
    Mass = [80, 80, 80, 10, 10, 10]
    nRet = cps.HRIF_SetMassParams(0, 0, Mass)
    time.sleep(0.0001)
    if nRet != 0:
        config["logger"].error(f"Failed to set mass params: {nRet}")
        return

    # Stiffness
    # Keep X engagement softer to avoid pushing/shifting the door on angled edge contact.
    Stiff = [1000, 1000, 1000, 100, 100, 100]
    nRet = cps.HRIF_SetStiffParams(0, 0, Stiff)
    time.sleep(0.0001)
    if nRet != 0:
        config["logger"].error(f"Failed to set PID control params: {nRet}")
        return

    # Set the damp parameter
    damp = [2500, 2500, 2500, 40, 40, 40]
    nRet = cps.HRIF_SetDampParams(0, 0, damp)
    time.sleep(0.0001)
    if nRet != 0:
        config["logger"].error(f"Failed to set damp params: {nRet}")
        return

    force_goal = [
        force * goal[0],
        force * goal[1],
        force * goal[2],
        0,
        0,
        0,
        0,
    ]  # Target force: [X, Y, Z, Rx, Ry, Rz]
    nret = cps.HRIF_SetForceControlGoal(boxID, rbtID, force_goal)
    time.sleep(0.0001)
    config["logger"].info(f"[forceControl] force control goal: {nret}")
    if nret != 0:
        config["logger"].error(f"Failed to set force control goal: {nret}")
        return

    # Enable force control
    cps.HRIF_SetForceControlState(boxID, rbtID, 1)
    time.sleep(0.0001)

    notFound = True
    while notFound:
        if stop_requested():
            try:
                cps.HRIF_SetForceControlState(boxID, rbtID, 0)
            except Exception:
                pass
            config["logger"].info("[forceControl] Stop requested during force seek; aborting.")
            return
        result = []
        nRet = cps.HRIF_ReadFTCabData(0, 0, result)
        config["logger"].info(f"[forceControl] Force that is coming is: {result}")

        for i, val in enumerate(goal):
            if val and abs(float(result[i])) > abs(force):
                config["logger"].info(
                    f"[forceControl] Force condition met: Axis {i}, Force {result[i]}"
                )
                time.sleep(0.0001)
                notFound = False
                break

        time.sleep(0.0001)

    config["logger"].info(f"[forceControl] applying force: {force}N")
    # time.sleep(0.1)
    config["logger"].info(f"[forceControl] Turned on vibration")


def putForceXminus(cps, force, tcp, ucs, config, goal=[1, 0, 0]):
    # Initialize parameters
    boxID = 0  # Control box ID
    rbtID = 0  # Robot ID

    result = []
    nret = 0

    waitForBlending(cps, config)
    setUCS_TCP(cps=cps, tcp=tcp, ucs=ucs, config=config)
    setSpeed(cps, speed=config["UI"]["sandSpeed"], config=config)

    nRet = cps.HRIF_SetForceZero(0, 0)
    if nRet != 0:
        config["logger"].error(f"Failed to set force zero: {nRet}")
        return

    # Set tool coordinate system mode for force control
    nret = cps.HRIF_SetForceToolCoordinateMotion(boxID, rbtID, 0)
    time.sleep(0.0001)
    config["logger"].info(f"forcetoolcoordinate: {nret}, result: {result}")
    if nret != 0:
        config["logger"].error(f"Failed to set force tool coordinate motion: {nret}")
        return

    # Set the force control strategy to constant force mode
    nret = cps.HRIF_SetForceControlStrategy(boxID, rbtID, 0)
    time.sleep(0.0001)
    config["logger"].info(f"force strategy: {nret}")
    if nret != 0:
        config["logger"].error(f"Failed to set force control strategy: {nret}")
        return

    # Define the target force control values (e.g., maintain fixed force in y and z axis)
    freedom = goal + [0, 0, 0]
    time.sleep(0.0001)
    cps.HRIF_SetControlFreedom(0, 0, freedom)  # force control degree of freedom

    # Set maximum search velocities for force control
    # Use same default seek speed as X+ for a gentler, symmetric contact behavior.
    force_cfg = config.get("forceControl", {}) if isinstance(config, dict) else {}
    try:
        linear_velocity = float(force_cfg.get("xminusLinearVelocity", 5))
    except (TypeError, ValueError):
        linear_velocity = 7
    try:
        angular_velocity = float(force_cfg.get("xminusAngularVelocity", 1))
    except (TypeError, ValueError):
        angular_velocity = 1
    nret = cps.HRIF_SetMaxSearchVelocities(
        boxID, rbtID, linear_velocity, angular_velocity
    )
    time.sleep(0.0001)
    config["logger"].info(f"search velocities: {nret}")
    if nret != 0:
        config["logger"].error(f"Failed to set max search velocities: {nret}")
        return

    # Set PID parameters to ensure stability in force control
    dFp = 0.8
    dFi = 0.001
    dFd = 0.02
    dTp = 0.8
    dTi = 0.001
    dTd = 0.02

    # nRet = cps.HRIF_SetPIDControlParams(0, 0, dFp, dFi, dFd, dTp, dTi, dTd)
    # time.sleep(0.1)
    # if nRet != 0:
    # config['logger'].error(f"Failed to set PID control params: {nRet}")
    # return

    # Set the Mass parameter
    Mass = [80, 80, 80, 10, 10, 10]
    nRet = cps.HRIF_SetMassParams(0, 0, Mass)
    time.sleep(0.0001)
    if nRet != 0:
        config["logger"].error(f"Failed to set mass params: {nRet}")
        return

    # Stiffness
    # Keep X engagement softer to avoid pushing/shifting the door on angled edge contact.
    Stiff = [1000, 1000, 1000, 100, 100, 100]
    nRet = cps.HRIF_SetStiffParams(0, 0, Stiff)
    time.sleep(0.0001)
    if nRet != 0:
        config["logger"].error(f"Failed to set PID control params: {nRet}")
        return

    # Set the damp parameter
    damp = [2500, 2500, 2500, 40, 40, 40]
    nRet = cps.HRIF_SetDampParams(0, 0, damp)
    time.sleep(0.0001)
    if nRet != 0:
        config["logger"].error(f"Failed to set damp params: {nRet}")
        return

    force_goal = [
        -force * goal[0],
        force * goal[1],
        force * goal[2],
        0,
        0,
        0,
        0,
    ]  # Target force: [X, Y, Z, Rx, Ry, Rz]
    nret = cps.HRIF_SetForceControlGoal(boxID, rbtID, force_goal)
    time.sleep(0.0001)
    config["logger"].info(f"[forceControl] force control goal: {nret}")
    if nret != 0:
        config["logger"].error(f"Failed to set force control goal: {nret}")
        return

    # Enable force control
    cps.HRIF_SetForceControlState(boxID, rbtID, 1)
    time.sleep(0.0001)

    notFound = True
    while notFound:
        if stop_requested():
            try:
                cps.HRIF_SetForceControlState(boxID, rbtID, 0)
            except Exception:
                pass
            config["logger"].info("[forceControl] Stop requested during force seek; aborting.")
            return
        result = []
        nRet = cps.HRIF_ReadFTCabData(0, 0, result)
        config["logger"].info(f"[forceControl] Force that is coming is: {result}")

        for i, val in enumerate(goal):
            if val and abs(float(result[i])) > abs(force):
                config["logger"].info(
                    f"[forceControl] Force condition met: Axis {i}, Force {result[i]}"
                )
                time.sleep(0.0001)
                notFound = False
                break

        time.sleep(0.0001)

    config["logger"].info(f"[forceControl] applying force: {force}N")
    # time.sleep(0.1)
    config["logger"].info(f"[forceControl] Turned on vibration")


def putForceYminus1(cps, force, tcp, ucs, config, goal=[0, 1, 0]):
    # Initialize parameters
    boxID = 0  # Control box ID
    rbtID = 0  # Robot ID

    result = []
    nret = 0

    waitForBlending(cps, config)
    setUCS_TCP(cps=cps, tcp=tcp, ucs=ucs, config=config)
    setSpeed(cps, speed=config["UI"]["sandSpeed"], config=config)

    nRet = cps.HRIF_SetForceZero(0, 0)
    if nRet != 0:
        config["logger"].error(f"Failed to set force zero: {nRet}")
        return

    # Set tool coordinate system mode for force control
    nret = cps.HRIF_SetForceToolCoordinateMotion(boxID, rbtID, 0)
    time.sleep(0.0001)
    config["logger"].info(f"forcetoolcoordinate: {nret}, result: {result}")
    if nret != 0:
        config["logger"].error(f"Failed to set force tool coordinate motion: {nret}")
        return

    # Set the force control strategy to constant force mode
    nret = cps.HRIF_SetForceControlStrategy(boxID, rbtID, 0)
    time.sleep(0.0001)
    config["logger"].info(f"force strategy: {nret}")
    if nret != 0:
        config["logger"].error(f"Failed to set force control strategy: {nret}")
        return

    # Define the target force control values (e.g., maintain fixed force in y and z axis)
    freedom = goal + [0, 0, 0]
    time.sleep(0.0001)
    cps.HRIF_SetControlFreedom(0, 0, freedom)  # force control degree of freedom

    # Set maximum search velocities for force control
    linear_velocity = 5  # 100 mm/s
    angular_velocity = 1  # 10 °/s
    nret = cps.HRIF_SetMaxSearchVelocities(
        boxID, rbtID, linear_velocity, angular_velocity
    )
    time.sleep(0.0001)
    config["logger"].info(f"search velocities: {nret}")
    if nret != 0:
        config["logger"].error(f"Failed to set max search velocities: {nret}")
        return

    # Set PID parameters to ensure stability in force control
    dFp = 0.8
    dFi = 0.001
    dFd = 0.02
    dTp = 0.8
    dTi = 0.001
    dTd = 0.02

    # nRet = cps.HRIF_SetPIDControlParams(0, 0, dFp, dFi, dFd, dTp, dTi, dTd)
    # time.sleep(0.1)
    # if nRet != 0:
    # config['logger'].error(f"Failed to set PID control params: {nRet}")
    # return

    # Set the Mass parameter
    Mass = [80, 80, 80, 10, 10, 10]
    nRet = cps.HRIF_SetMassParams(0, 0, Mass)
    time.sleep(0.0001)
    if nRet != 0:
        config["logger"].error(f"Failed to set mass params: {nRet}")
        return

    # Stiffness
    Stiff = [1000, 1000, 1000, 100, 100, 100]
    nRet = cps.HRIF_SetStiffParams(0, 0, Stiff)
    time.sleep(0.0001)
    if nRet != 0:
        config["logger"].error(f"Failed to set PID control params: {nRet}")
        return

    # Set the damp parameter
    damp = [2500, 2500, 2500, 40, 40, 40]
    nRet = cps.HRIF_SetDampParams(0, 0, damp)
    time.sleep(0.0001)
    if nRet != 0:
        config["logger"].error(f"Failed to set damp params: {nRet}")
        return

    force_goal = [
        force * goal[0],
        -force * goal[1],
        force * goal[2],
        0,
        0,
        0,
        0,
    ]  # Target force: [X, Y, Z, Rx, Ry, Rz]
    nret = cps.HRIF_SetForceControlGoal(boxID, rbtID, force_goal)
    time.sleep(0.0001)
    config["logger"].info(f"[forceControl] force control goal: {nret}")
    if nret != 0:
        config["logger"].error(f"Failed to set force control goal: {nret}")
        return

    # Enable force control
    cps.HRIF_SetForceControlState(boxID, rbtID, 1)
    time.sleep(0.0001)

    notFound = True
    while notFound:
        if stop_requested():
            try:
                cps.HRIF_SetForceControlState(boxID, rbtID, 0)
            except Exception:
                pass
            config["logger"].info("[forceControl] Stop requested during force seek; aborting.")
            return
        result = []
        nRet = cps.HRIF_ReadFTCabData(0, 0, result)
        config["logger"].info(f"[forceControl] Force that is coming is: {result}")

        for i, val in enumerate(goal):
            if val and abs(float(result[i])) > abs(force):
                config["logger"].info(
                    f"[forceControl] Force condition met: Axis {i}, Force {result[i]}"
                )
                time.sleep(0.0001)
                notFound = False
                break

        time.sleep(0.0001)

    config["logger"].info(f"[forceControl] applying force: {force}N")
    # time.sleep(0.1)
    config["logger"].info(f"[forceControl] Turned on vibration")


def putForceYminus1edge(cps, force, tcp, ucs, config, goal=[0, 1, 0]):
    # Initialize parameters
    boxID = 0  # Control box ID
    rbtID = 0  # Robot ID

    result = []
    nret = 0

    waitForBlending(cps, config)
    setUCS_TCP(cps=cps, tcp=tcp, ucs=ucs, config=config)
    setSpeed(cps, speed=config["UI"]["sandSpeed"], config=config)

    nRet = cps.HRIF_SetForceZero(0, 0)
    if nRet != 0:
        config["logger"].error(f"Failed to set force zero: {nRet}")
        return

    # Set tool coordinate system mode for force control
    nret = cps.HRIF_SetForceToolCoordinateMotion(boxID, rbtID, 0)
    time.sleep(0.0001)
    config["logger"].info(f"forcetoolcoordinate: {nret}, result: {result}")
    if nret != 0:
        config["logger"].error(f"Failed to set force tool coordinate motion: {nret}")
        return

    # Set the force control strategy to constant force mode
    nret = cps.HRIF_SetForceControlStrategy(boxID, rbtID, 0)
    time.sleep(0.0001)
    config["logger"].info(f"force strategy: {nret}")
    if nret != 0:
        config["logger"].error(f"Failed to set force control strategy: {nret}")
        return

    # Define the target force control values (e.g., maintain fixed force in y and z axis)
    freedom = goal + [0, 0, 0]
    time.sleep(0.0001)
    cps.HRIF_SetControlFreedom(0, 0, freedom)  # force control degree of freedom

    # Set maximum search velocities for force control
    linear_velocity = 5  # 100 mm/s
    angular_velocity = 1  # 10 °/s
    nret = cps.HRIF_SetMaxSearchVelocities(
        boxID, rbtID, linear_velocity, angular_velocity
    )
    time.sleep(0.0001)
    config["logger"].info(f"search velocities: {nret}")
    if nret != 0:
        config["logger"].error(f"Failed to set max search velocities: {nret}")
        return

    # Set PID parameters to ensure stability in force control
    dFp = 0.8
    dFi = 0.001
    dFd = 0.02
    dTp = 0.8
    dTi = 0.001
    dTd = 0.02

    nRet = cps.HRIF_SetPIDControlParams(0, 0, dFp, dFi, dFd, dTp, dTi, dTd)
    time.sleep(0.0001)
    if nRet != 0:
        config["logger"].error(f"Failed to set PID control params: {nRet}")
        return

    # Set the Mass parameter
    # Mass = [80, 80, 80, 10, 10, 10]
    # nRet = cps.HRIF_SetMassParams(0, 0, Mass)
    # time.sleep(0.0001)
    # if nRet != 0:
    #     config['logger'].error(f"Failed to set mass params: {nRet}")
    #     return

    # Stiffness
    # Stiff = [1000, 1000, 1000, 100, 100, 100]
    # nRet = cps.HRIF_SetStiffParams(0,0,Stiff)
    # time.sleep(0.0001)
    # if nRet != 0:
    #     config['logger'].error(f"Failed to set PID control params: {nRet}")
    #     return

    # Set the damp parameter
    # damp = [2500, 2500, 2500, 40, 40, 40]
    # nRet = cps.HRIF_SetDampParams(0, 0, damp)
    # time.sleep(0.0001)
    # if nRet != 0:
    #     config['logger'].error(f"Failed to set damp params: {nRet}")
    #     return

    force_goal = [
        force * goal[0],
        -force * goal[1],
        force * goal[2],
        0,
        0,
        0,
        0,
    ]  # Target force: [X, Y, Z, Rx, Ry, Rz]
    nret = cps.HRIF_SetForceControlGoal(boxID, rbtID, force_goal)
    time.sleep(0.0001)
    config["logger"].info(f"[forceControl] force control goal: {nret}")
    if nret != 0:
        config["logger"].error(f"Failed to set force control goal: {nret}")
        return

    # Enable force control
    cps.HRIF_SetForceControlState(boxID, rbtID, 1)
    time.sleep(0.0001)

    notFound = True
    while notFound:
        if stop_requested():
            try:
                cps.HRIF_SetForceControlState(boxID, rbtID, 0)
            except Exception:
                pass
            config["logger"].info("[forceControl] Stop requested during force seek; aborting.")
            return
        result = []
        nRet = cps.HRIF_ReadFTCabData(0, 0, result)
        config["logger"].info(f"[forceControl] Force that is coming is: {result}")

        for i, val in enumerate(goal):
            if val and abs(float(result[i])) > abs(force):
                config["logger"].info(
                    f"[forceControl] Force condition met: Axis {i}, Force {result[i]}"
                )
                time.sleep(0.0001)
                notFound = False
                break

        time.sleep(0.0001)

    config["logger"].info(f"[forceControl] applying force: {force}N")
    # time.sleep(0.1)
    config["logger"].info(f"[forceControl] Turned on vibration")


def releaseForce(cps, config, wait_for_blending=True):
    # Initialize parameters
    boxID = 0  # Control box ID
    rbtID = 0  # Robot ID
    # Disable force control after the movement is completed
    # toggle_stopper_status(cps, digital_number=4)
    config["logger"].info(f"[forceControl] Turned off vibration")
    cps.HRIF_SetForceControlState(boxID, rbtID, 0)
    config["logger"].info(f"[forceControl] releasing force: 0N")
    if wait_for_blending:
        waitForBlending(cps, config)
    setSpeed(cps, speed=config["UI"]["robotSpeed"], config=config)


def setUCS_TCP(cps, tcp, ucs, config):
    door_cfg = config.get("door", {}) if isinstance(config, dict) else {}
    coord_blend_timeout_s = max(
        0.05, float(door_cfg.get("coordBlendTimeoutSeconds", 0.2))
    )

    def coord_matches(current, by_name, target_name):
        """Compare coordinate identities robustly across SDK response formats."""
        if not isinstance(current, (list, tuple)) or len(current) == 0:
            return False

        target = str(target_name).strip().lower()

        # Some SDK variants return the current coordinate name directly.
        first = str(current[0]).strip().lower()
        if first == target:
            return True

        # Other variants include the coordinate name in another field.
        for item in current:
            if str(item).strip().lower() == target:
                return True

        # If both APIs return vectors/records, compare element-wise with tolerance.
        if isinstance(by_name, (list, tuple)) and len(by_name) == len(current) and len(by_name) > 0:
            for a, b in zip(current, by_name):
                try:
                    if abs(float(a) - float(b)) > 1e-4:
                        return False
                except Exception:
                    if str(a).strip().lower() != str(b).strip().lower():
                        return False
            return True
        return False

    def wait_coord_applied(read_fn, expected, label, target_name, timeout_s=0.12, poll_s=0.01):
        """Poll current coordinate name briefly instead of fixed long sleeps."""
        end_t = time.monotonic() + timeout_s
        while time.monotonic() < end_t:
            current = []
            nret_local = read_fn(0, 0, current)
            if nret_local == 0 and coord_matches(current, expected, target_name):
                return True
            time.sleep(poll_s)
        if isinstance(config, dict) and config.get("logger"):
            config["logger"].warning(
                f"[setUCS_TCP] Timed out waiting for {label} apply; continuing."
            )
        return False

    def set_coord_with_retry(setter_fn, reader_fn, by_name, label, target):
        """Retry set when controller returns transient state errors like 20018."""
        last_ret = None
        for attempt in range(3):
            if stop_requested():
                return False
            last_ret = setter_fn(0, 0, target)
            if last_ret == 0:
                wait_coord_applied(reader_fn, by_name, label, target)
                config["logger"].info(f"[setUCS_TCP] Success in setting {label} to: {target}")
                return True
            if last_ret == 20018:
                try:
                    cps.HRIF_GrpStop(0, 0)
                except Exception:
                    pass
                time.sleep(0.05)
                continue
            break

        config["logger"].error(
            f"[setUCS_TCP] Failure in setting {label} to: {target}, nret = {last_ret}"
        )
        msg_to_frontend(
            api_url=config["server"]["frontEnd_messaging_url"],
            message="Error With Robot Settings. Please Verify The Robot Settings and Try Again. Terminating Process...",
        )
        return False

    # Step 1: Set the UCS
    ucsByCurrent = []  # Read User coordinates
    nRet = cps.HRIF_ReadCurUCS(0, 0, ucsByCurrent)

    if not coord_matches(ucsByCurrent, None, ucs):
        ucsByName = []
        nRet = cps.HRIF_ReadUCSByName(0, 0, ucs, ucsByName)
        if not coord_matches(ucsByCurrent, ucsByName, ucs):
            waitForBlending(cps, config, timeout_s=coord_blend_timeout_s)
            if not set_coord_with_retry(
                cps.HRIF_SetUCSByName, cps.HRIF_ReadCurUCS, ucsByName, "UCS", ucs
            ):
                return False
    # else:
    #     config['logger'].info(f'[setUCS_TCP] Used previous UCS: {ucs}')

    # Step 2: Set the TCP
    tcpByCurrent = []  # Read User coordinates
    nRet = cps.HRIF_ReadCurTCP(0, 0, tcpByCurrent)

    if not coord_matches(tcpByCurrent, None, tcp):
        tcpByName = []
        nRet = cps.HRIF_ReadTCPByName(0, 0, tcp, tcpByName)
        if not coord_matches(tcpByCurrent, tcpByName, tcp):
            waitForBlending(cps, config, timeout_s=coord_blend_timeout_s)
            if not set_coord_with_retry(
                cps.HRIF_SetTCPByName, cps.HRIF_ReadCurTCP, tcpByName, "TCP", tcp
            ):
                return False
    # else:
    #     config['logger'].info(f'[setUCS_TCP] Used previous TCP: {tcp}')

    return True


def handle_client(config, homingState=False, startSanding=True, scan=False, cps=None):
    # Establish connection with robot (unless caller provides an existing CPS client)
    if cps is None:
        cps = CPSClient()
        IP = config["server"]["cpip"]
        port = config["server"]["cps"]
        ret = cps.HRIF_Connect(0, IP, port)
    if "logger" not in config:
        config["logger"] = setup_logger(config["settings"]["debug"])

    def homingFunction(cps, config):
        result = [-1, -1, -1, -1]
        door_cfg = config.get("door", {}) if isinstance(config, dict) else {}
        homing_blend_timeout_s = max(
            0.2, float(door_cfg.get("homingBlendTimeoutSeconds", 1.0))
        )
        motor_connect_settle_s = max(
            0.01, float(door_cfg.get("homingMotorConnectDelaySec", 0.04))
        )
        motor_stop_settle_s = max(
            0.01, float(door_cfg.get("homingMotorStopDelaySec", 0.05))
        )
        origin_command_settle_s = max(
            0.01, float(door_cfg.get("homingOriginCommandDelaySec", 0.125))
        )
        origin_poll_s = max(
            0.02, float(door_cfg.get("homingOriginPollIntervalSec", 0.05))
        )
        origin_wait_timeout_s = max(
            5.0, float(door_cfg.get("homingOriginWaitTimeoutSec", 60.0))
        )

        def origin_done(state):
            """Robust completion check for MotorGetState response."""
            if not isinstance(state, (list, tuple)) or len(state) < 3:
                return False
            raw_s = str(state[2]).strip().lower()
            # Common SDK variants for idle/done.
            if raw_s in ("idle", "done", "false"):
                return True
            try:
                return float(raw_s) == 0.0
            except (TypeError, ValueError):
                return False

        def motor_cmd_ok(nret, state):
            if nret is None:
                return bool(state) and str(state[0]) == "0"
            return nret == 0 or (bool(state) and str(state[0]) == "0")

        if not setUCS_TCP(
            cps,
            tcp=config["coords"]["tcpDefault"],
            ucs=config["coords"]["ucsDefault"],
            config=config,
        ):
            msg_to_frontend(
                api_url=config["server"]["frontEnd_messaging_url"],
                message="Homing failed: unable to set UCS/TCP.",
            )
            return
        if stop_requested():
            msg_to_frontend(
                api_url=config["server"]["frontEnd_messaging_url"],
                message="Homing stopped by user.",
            )
            return
        # result = [ ] # Read the current actual location information
        nRet = cps.HRIF_ReadActPos(0, 0, result)  # Read the joint position variable

        # ---- SAFETY CHECK / RETRY ----
        if result is None or not isinstance(result, (list, tuple)):
            config["logger"].error(f"[HomingFunc] Invalid result type: {result}")
            msg_to_frontend(
                api_url=config["server"]["frontEnd_messaging_url"],
                message="Homing failed: controller returned invalid position data.",
            )
            return
        if nRet != 0 or len(result) < 7:
            config["logger"].error(
                f"[HomingFunc] HRIF_ReadActPos failed ret={nRet}, len={len(result)} data={result}"
            )
            # Try one reconnect + retry before aborting
            try:
                cps.HRIF_DisConnect(0)
                time.sleep(0.2)
                cps.HRIF_Connect(0, config["server"]["cpip"], config["server"]["cps"])
                result = [-1, -1, -1, -1]
                nRet = cps.HRIF_ReadActPos(0, 0, result)
            except Exception as exc:
                config["logger"].error(f"[HomingFunc] Reconnect failed: {exc}")

            if nRet != 0 or len(result) < 7:
                msg_to_frontend(
                    api_url=config["server"]["frontEnd_messaging_url"],
                    message="Homing failed: controller did not return full position data.",
                )
                return
        
        dX = float(result[6])
        # config['logger'].info(f"[homing] current position: {result}")
        msg_to_frontend(
            api_url=config["server"]["frontEnd_messaging_url"],
            message="Homing Started...",
        )
        if dX > config["point"]["safePointTool"][0]:
            communicate(
                cps=cps,
                point=config["point"]["safePointTool"],
                tcp=config["coords"]["tcpDefault"],
                ucs=config["coords"]["ucsDefault"],
                seventh=-1,
                config=config,
                speed=config["door"]["homingSpeed"],
                wait=False,
            )
        if stop_requested():
            msg_to_frontend(
                api_url=config["server"]["frontEnd_messaging_url"],
                message="Homing stopped by user.",
            )
            return
        communicate(
            cps=cps,
            point=config["point"]["safePoint"],
            tcp=config["coords"]["tcpDefault"],
            ucs=config["coords"]["ucsDefault"],
            seventh=-1,
            config=config,
            speed=config["door"]["homingSpeed"],
            wait=False,
        )
        waitForBlending(cps=cps, config=config, timeout_s=homing_blend_timeout_s)
        if stop_requested():
            msg_to_frontend(
                api_url=config["server"]["frontEnd_messaging_url"],
                message="Homing stopped by user.",
            )
            return
        # connect the 7th axis motor
        nret = cps.HRIF_HRApp(0, "HR_Motor", "MotorConnect", ["J7"], result)
        time.sleep(motor_connect_settle_s)
        if result[1] != "OK":
            config["logger"].error(
                "[HomingFunc] Could not connect to the motor. Exiting..."
            )
            msg_to_frontend(
                api_url=config["server"]["frontEnd_messaging_url"],
                message="7th Axis Connection Error. Please Check if It's Working and Try Again! Terminating Cycle...",
            )
            exit(-1)
        # Step 2: go to your homing position (for 7th axis)
        config["logger"].info("[HomingFunc] step 2: Go to the homing switch")
        # communicate(cps=cps, tcp=config['coords']['tcpDefault'], ucs=config['coords']['ucsDefault'], seventh=0, config=config, speed=config['door']['homingSpeed'])
        max_origin_cmd_retries = int(door_cfg.get("homingOriginCommandRetries", 3) or 3)
        if max_origin_cmd_retries < 1:
            max_origin_cmd_retries = 1
        origin_started = False
        for attempt in range(1, max_origin_cmd_retries + 1):
            result = []
            nret = cps.HRIF_HRApp(0, "HR_Motor", "MotorStop", ["J7"], result)
            time.sleep(motor_stop_settle_s)
            print(f"****** motor stop nret: {nret}; result: {result}")

            result = []
            nret = cps.HRIF_HRApp(0, "HR_Motor", "MotorMoveOrigin", ["J7"], result)
            # nret = cps.HRIF_HRApp(0, 'HR_Motor','MotorMovePosition', ["J7", "-22.0"], result)
            # seventhGoToPos(cpsclient, position=0, speed=config['7thAxis']['speed'] * config['cobot']['speed'], config=config)
            time.sleep(origin_command_settle_s)
            print(f"****** move origin nret: {nret}; result: {result}")

            if motor_cmd_ok(nret, result):
                origin_started = True
                break

            config["logger"].warning(
                f"[HomingFunc] MotorMoveOrigin not accepted on attempt "
                f"{attempt}/{max_origin_cmd_retries} (ret={nret}, res={result}). Retrying..."
            )
            cps.HRIF_HRApp(0, "HR_Motor", "MotorConnect", ["J7"], [])
            time.sleep(0.05 * attempt)

        if not origin_started:
            config["logger"].error(
                f"[HomingFunc] MotorMoveOrigin failed after {max_origin_cmd_retries} attempts."
            )
            msg_to_frontend(
                api_url=config["server"]["frontEnd_messaging_url"],
                message="Homing failed: J7 origin command was not accepted.",
            )
            return

        result = [-1, -1, "-1"]
        origin_wait_start = time.time()
        # Wait until the motor reports origin complete.
        while not origin_done(result):
            if stop_requested():
                try:
                    cps.HRIF_HRApp(0, "HR_Motor", "MotorStop", ["J7"], [])
                except Exception:
                    pass
                msg_to_frontend(
                    api_url=config["server"]["frontEnd_messaging_url"],
                    message="Homing stopped by user.",
                )
                return
            nret = cps.HRIF_HRApp(0, "HR_Motor", "MotorGetState", ["J7"], result)
            if nret not in (0, None):
                config["logger"].warning(
                    f"[HomingFunc] MotorGetState returned ret={nret}; res={result}"
                )
            print(f"****** result: {result}")
            if (time.time() - origin_wait_start) >= origin_wait_timeout_s:
                try:
                    cps.HRIF_HRApp(0, "HR_Motor", "MotorStop", ["J7"], [])
                except Exception:
                    pass
                config["logger"].error(
                    f"[HomingFunc] Timeout waiting for J7 origin complete after "
                    f"{origin_wait_timeout_s:.1f}s. Last state={result}"
                )
                msg_to_frontend(
                    api_url=config["server"]["frontEnd_messaging_url"],
                    message="Homing failed: timeout while waiting for J7 origin.",
                )
                return
            time.sleep(origin_poll_s)
        # Move to the configured J7 home position after origin.
        if stop_requested():
            msg_to_frontend(
                api_url=config["server"]["frontEnd_messaging_url"],
                message="Homing stopped by user.",
            )
            return
        home_default = float(config.get("seventhAxis", {}).get("homePosition", -65))
        home_j7 = float(config.get("UI", {}).get("seventhAxisHome", home_default))
        home_abs_limit = float(config.get("seventhAxis", {}).get("homeAbsLimit", 2000.0))
        if abs(home_j7) > home_abs_limit:
            config["logger"].error(
                f"[HomingFunc] Invalid seventhAxisHome={home_j7}. "
                f"Allowed absolute limit={home_abs_limit}."
            )
            msg_to_frontend(
                api_url=config["server"]["frontEnd_messaging_url"],
                message="Homing failed: invalid J7 home target value in settings.",
            )
            return
        config["logger"].info(
            f"[HomingFunc] Reached J7 origin; moving to home position {home_j7}mm"
        )
        ok = communicate(
            cps=cps,
            config=config,
            seventh=home_j7,
            tcp=config["coords"]["tcpDefault"],
            ucs=config["coords"]["ucsDefault"],
            speed=config["door"]["homingSpeed"],
            wait=True,
            require_seventh_ok=True,
        )
        if ok is None:
            msg_to_frontend(
                api_url=config["server"]["frontEnd_messaging_url"],
                message="Homing failed: could not move J7 to home position.",
            )
            return
        config["logger"].info("[HomingFunc] DONE! Success to reach J7 home position")
        msg_to_frontend(
            api_url=config["server"]["frontEnd_messaging_url"],
            message="Homing Completed Successfully!",
        )


    def mm_to_inches(mm):
        inches = mm / 25.4
        return inches

    def inches_to_mm(inches):
        mm = inches * 25.4
        return mm

    def addXVal(point, val):
        retPoint = copy.deepcopy(point)
        # config['logger'].info(f"val: (before) {point}")
        retPoint[0] += val
        # config['logger'].info(f"val: (after) {retPoint}")
        return retPoint

    def addYVal(point, val):
        retPoint = copy.deepcopy(point)
        # config['logger'].info(f"val: (before) {point}")
        yVal = math.cos(math.radians(config["table"]["angle"])) * val
        zVal = math.sin(math.radians(config["table"]["angle"])) * val
        # config['logger'].info(f"yval: {yVal}")
        # config['logger'].info(f"zval: {zVal}")
        retPoint[1] += yVal
        retPoint[2] += zVal
        # config['logger'].info(f"val: (after) {retPoint}")
        return retPoint

    def addZVal(point, val):
        retPoint = copy.deepcopy(point)
        # config['logger'].info(f"val: (before) {point}")
        yVal = math.sin(math.radians(config["table"]["angle"])) * val
        zVal = math.cos(math.radians(config["table"]["angle"])) * val
        # config['logger'].info(f"yval: {yVal}")
        # config['logger'].info(f"zval: {zVal}")
        retPoint[1] += yVal
        retPoint[2] += zVal
        # config['logger'].info(f"val: (after) {retPoint}")
        return retPoint

    def getTool(cps, toolNumber, config, startFromSafe=True):
        """_summary_

        Args:
            cps (CPSClient): cobot client
            toolNumber (int): The position from which to pick tool (out of 1-4)
            config (config): configuration info
            startFromSafe (bool, optional): If should go to the safe tool picking position or not. Make False if doing tool drop and pick one after another. Defaults to True.
        """
        if not config["settings"]["useTool"]:
            return False
        # Force fast tool travel at full robot ratio for responsive pick/drop.
        pick_fast = 1.0
        pick_slow = _tool_speed(
            config, "autoPickSlowSpeed", config.get("UI", {}).get("sandSpeed")
        )
        # Keep tool fast/slow aligned with cycle/UI ratios used by scan/sanding flows.
        pick_slow = _resolve_ui_ratio(config, "sandSpeed", pick_slow)
        config["logger"].info(
            "[toolMotion][autoPick] ratios: fast(robot)=%.3f slow(sanding)=%.3f",
            float(pick_fast),
            float(pick_slow),
        )
        current_tool = _get_tool_in_hand(cps)
        if current_tool and current_tool > 0:
            msg_to_frontend(
                api_url=config["server"]["frontEnd_messaging_url"],
                message=f"Tool {current_tool} already in hand. Drop it before picking Tool {toolNumber}.",
            )
            return False
        if stop_requested():
            msg_to_frontend(
                api_url=config["server"]["frontEnd_messaging_url"],
                message="Tool collection stopped by user.",
            )
            return False
        # Ensure prior blended motion is fully settled before speed/approach changes.
        waitForBlending(cps=cps, config=config)
        # Keep override neutral so linear profile commands are not flattened.
        setSpeed(cps, 1.0, config=config)
        msg_to_frontend(
            api_url=config["server"]["frontEnd_messaging_url"],
            message=f"Tool {toolNumber} Collection Started...",
        )
        # if didn't drop another tool just before picking this one, then come to safe picking position
        if startFromSafe:
            communicate(
                cps=cps,
                point=config["point"]["safePointTool"],
                tcp=config["coords"]["tcpDefault"],
                ucs=config["coords"]["ucsDefault"],
                seventh=-1,
                config=config,
                speed=pick_fast,
                velocity_profile="robotspeed",
                speed_mode="linear",
                wait=False,
            )
            if stop_requested():
                msg_to_frontend(
                    api_url=config["server"]["frontEnd_messaging_url"],
                    message="Tool collection stopped by user.",
                )
                return False

        # go to that tool's home position (right above the tool)
        communicate(
            cps=cps,
            point=config["point"][f"tool{toolNumber}home"],
            tcp=config["coords"]["tcpDefault"],
            ucs=config["coords"]["ucsDefault"],
            seventh=-1,
            config=config,
            speed=pick_fast,
            velocity_profile="robotspeed",
            speed_mode="linear",
            wait=True,
        )
        if stop_requested():
            msg_to_frontend(
                api_url=config["server"]["frontEnd_messaging_url"],
                message="Tool collection stopped by user.",
            )
            return False
        # drop (for safety, to open the valve)
        toolValve(cps, valveState="drop", config=config)
        if stop_requested():
            msg_to_frontend(
                api_url=config["server"]["frontEnd_messaging_url"],
                message="Tool collection stopped by user.",
            )
            return False
        # touch the tool (slowly)
        communicate(
            cps=cps,
            point=config["point"][f"tool{toolNumber}"],
            tcp=config["coords"]["tcpDefault"],
            ucs=config["coords"]["ucsDefault"],
            seventh=-1,
            config=config,
            speed=pick_slow,
            velocity_profile="sandingspeed",
            speed_mode="linear",
            wait=True,
        )
        if stop_requested():
            msg_to_frontend(
                api_url=config["server"]["frontEnd_messaging_url"],
                message="Tool collection stopped by user.",
            )
            return False
        # pick the tool
        toolValve(cps, valveState="pick", config=config)
        if stop_requested():
            msg_to_frontend(
                api_url=config["server"]["frontEnd_messaging_url"],
                message="Tool collection stopped by user.",
            )
            return False
        # come back to tool's home position (slow retract)
        communicate(
            cps=cps,
            point=config["point"][f"tool{toolNumber}home"],
            tcp=config["coords"]["tcpDefault"],
            ucs=config["coords"]["ucsDefault"],
            seventh=-1,
            config=config,
            speed=pick_slow,
            velocity_profile="sandingspeed",
            speed_mode="linear",
            wait=True,
        )
        if stop_requested():
            msg_to_frontend(
                api_url=config["server"]["frontEnd_messaging_url"],
                message="Tool collection stopped by user.",
            )
            return False
        # come back to safe tool picking position
        msg_to_frontend(
            api_url=config["server"]["frontEnd_messaging_url"],
            message=f"Tool {toolNumber} Collection Successful!",
        )
        communicate(
            cps=cps,
            point=config["point"]["safePointTool"],
            tcp=config["coords"]["tcpDefault"],
            ucs=config["coords"]["ucsDefault"],
            seventh=-1,
            config=config,
            speed=pick_fast,
            velocity_profile="robotspeed",
            speed_mode="linear",
            wait=True,
        )
        return True

    def keepTool(cps, toolNumber, config, goToSafe=True):
        if not config["settings"]["useTool"]:
            return
        # Force fast tool travel at full robot ratio for responsive pick/drop.
        drop_fast = 1.0
        drop_slow = _tool_speed(
            config, "autoDropSlowSpeed", config.get("UI", {}).get("sandSpeed")
        )
        drop_slow = _resolve_ui_ratio(config, "sandSpeed", drop_slow)
        config["logger"].info(
            "[toolMotion][autoDrop] ratios: fast(robot)=%.3f slow(sanding)=%.3f",
            float(drop_fast),
            float(drop_slow),
        )
        if stop_requested():
            msg_to_frontend(
                api_url=config["server"]["frontEnd_messaging_url"],
                message="Tool keeping stopped by user.",
            )
            return
        # Wait for previous trajectory to settle so override/speed is applied reliably.
        waitForBlending(cps=cps, config=config)
        # Keep override neutral so linear profile commands are not flattened.
        setSpeed(cps, 1.0, config=config)
        msg_to_frontend(
            api_url=config["server"]["frontEnd_messaging_url"],
            message=f"Tool {toolNumber} Keeping Started...",
        )
        # come to safe tool picking position
        communicate(
            cps=cps,
            point=config["point"]["safePointTool"],
            tcp=config["coords"]["tcpDefault"],
            ucs=config["coords"]["ucsDefault"],
            seventh=-1,
            config=config,
            speed=drop_fast,
            velocity_profile="robotspeed",
            speed_mode="linear",
            wait=True,
        )
        if stop_requested():
            msg_to_frontend(
                api_url=config["server"]["frontEnd_messaging_url"],
                message="Tool keeping stopped by user.",
            )
            return
        # go to tool's home
        communicate(
            cps=cps,
            point=config["point"][f"tool{toolNumber}home"],
            tcp=config["coords"]["tcpDefault"],
            ucs=config["coords"]["ucsDefault"],
            seventh=-1,
            config=config,
            speed=drop_fast,
            velocity_profile="robotspeed",
            speed_mode="linear",
            wait=True,
        )
        if stop_requested():
            msg_to_frontend(
                api_url=config["server"]["frontEnd_messaging_url"],
                message="Tool keeping stopped by user.",
            )
            return
        # touch the tool (slowly)
        communicate(
            cps=cps,
            point=config["point"][f"tool{toolNumber}"],
            tcp=config["coords"]["tcpDefault"],
            ucs=config["coords"]["ucsDefault"],
            seventh=-1,
            config=config,
            speed=drop_slow,
            velocity_profile="sandingspeed",
            speed_mode="linear",
            wait=True,
        )
        if stop_requested():
            msg_to_frontend(
                api_url=config["server"]["frontEnd_messaging_url"],
                message="Tool keeping stopped by user.",
            )
            return
        # drop the tool
        toolValve(cps, valveState="drop", config=config)
        if stop_requested():
            msg_to_frontend(
                api_url=config["server"]["frontEnd_messaging_url"],
                message="Tool keeping stopped by user.",
            )
            return
        # come back to tool's home (slow retract)
        communicate(
            cps=cps,
            point=config["point"][f"tool{toolNumber}home"],
            tcp=config["coords"]["tcpDefault"],
            ucs=config["coords"]["ucsDefault"],
            seventh=-1,
            config=config,
            speed=drop_slow,
            velocity_profile="sandingspeed",
            speed_mode="linear",
            wait=True,
        )
        if stop_requested():
            msg_to_frontend(
                api_url=config["server"]["frontEnd_messaging_url"],
                message="Tool keeping stopped by user.",
            )
            return
        # if don't need to pick another tool just after dropping this one, then come to safe picking position
        msg_to_frontend(
            api_url=config["server"]["frontEnd_messaging_url"],
            message=f"Tool {toolNumber} Kept Successfully",
        )
        if goToSafe:
            communicate(
                cps=cps,
                point=config["point"]["safePointTool"],
                tcp=config["coords"]["tcpDefault"],
                ucs=config["coords"]["ucsDefault"],
                seventh=-1,
                config=config,
                speed=drop_fast,
                velocity_profile="robotspeed",
                speed_mode="linear",
                wait=False,
            )

        time.sleep(2)

    def toolValve(cps, valveState: str, config):
        """Used to "pick" or "drop" the tool.
        Helpful: When DO5 is 1: the pneumatic is loose. When D05 is 0, pneumatic is tight.

        Args:
            cps (CPSClient): cps
            valveState (str): "pick" to grab the tool, "drop" to let the tool go.
            config (config): configuration file
        """
        if stop_requested():
            config["logger"].warning(
                f"[toolValve] Ignored valve '{valveState}' due to stop request."
            )
            return
        pre_delay = _tool_valve_delay(config, "toolValvePreDelaySec", 0.15)
        post_delay = _tool_valve_delay(config, "toolValvePostDelaySec", 0.15)
        # Required dwell for pneumatic stability.
        time.sleep(pre_delay)
        if valveState == "drop":
            status = 1
            digOutput = 5  # DOnumber=0,1,2,3,4

            # while True:
            #     confirmation = input("Are you sure you want to DROP the tool? (yes/no): ").strip().lower()
            #     if confirmation == 'yes':
            #         break

            #     time.sleep(0.1)
            nRet = cps.HRIF_SetBoxDO(0, digOutput, status)
            if config["settings"]["debug"]:
                config["logger"].info(
                    f"[toolValve] Tool is dropped! Success: {nRet} (0 means successful)"
                )

        elif valveState == "pick":
            status = 0
            digOutput = 5  # DOnumber=0,1,2,3,4
            nRet = cps.HRIF_SetBoxDO(0, digOutput, status)
            if config["settings"]["debug"]:
                config["logger"].info(
                    f"[toolValve] Tool is dropped! Success: {nRet} (0 means successful)"
                )
        # required sleep (for proper functioning)
        msg_to_frontend(
            api_url=config["server"]["frontEnd_messaging_url"],
            message=f"Tool Set to '{valveState}'",
        )
        time.sleep(post_delay)

    def saveAsCSV(fileName, measurements):
        # Keep the historical columns first, but tolerate extra diagnostic fields.
        base_fieldnames = ["dist", "height"]
        extra_fieldnames = []
        for measurement in measurements or []:
            if not isinstance(measurement, dict):
                continue
            for key in measurement.keys():
                if key not in base_fieldnames and key not in extra_fieldnames:
                    extra_fieldnames.append(key)
        fieldnames = base_fieldnames + extra_fieldnames

        # Writing to CSV
        with open(fileName, mode="w", newline="") as file:
            writer = csv.DictWriter(file, fieldnames=fieldnames)
            # Write the header
            writer.writeheader()
            # Write the data
            writer.writerows(measurements)

    def csv_to_dict_list(file_path):
        with open(file_path, mode="r") as file:
            reader = csv.reader(file)
            headers = next(reader)  # Read the headers

            # Convert each row's values to float where possible
            def convert_to_float(value):
                try:
                    return float(value)
                except ValueError:
                    return value  # Keep the original value if conversion fails

            # Build the list of dictionaries with converted values
            return [
                {header: convert_to_float(value) for header, value in zip(headers, row)}
                for row in reader
            ]

    # Step 2: Function to calculate frames and pockets for each chunk using different thresholds
    def identify_gradient_change_points_dynamic(
        chunks, threshold, min_stable_distance, three_d_compensation, config
    ):
        """
        Identifies stable regions where the change in height is below a specified threshold for a minimum distance.

        Args:
        - chunks (list of dicts): Each dict contains 'dist' and 'height'.
        - threshold (float): The threshold for detecting stable regions based on height change.
        - min_stable_distance (float): Minimum distance required for a region to be considered stable.
        - three_d_compensation (float): fixed 3D compensation for scan geometry.
        - config (dict): configuration file
        Returns:
        - stable_regions (list of dicts): Each dict contains the start and end distances of a stable region.
        """
        # Filter out NaN values in height data
        chunks = [item for item in chunks if not math.isnan(item["height"])]
        compensation = 0
        if len(chunks) < 2:
            raise ValueError(
                "Insufficient scan points to classify depth profile (need at least 2 points)."
            )

        # Extract distances and heights
        distances = [item["dist"] for item in chunks]
        heights = [item["height"] for item in chunks]
        stable_start_idx = None  # Track start of stable region
        stable_regions = []
        scan_settings = config.get("settings", {}) if isinstance(config, dict) else {}
        min_frame_region_mm = float(
            scan_settings.get("scanMinFrameRegionMm", max(30.0, float(min_stable_distance)))
        )

        for i in range(1, len(heights)):
            # Calculate change in height from the previous point
            height_diff = abs(heights[i] - heights[i - 1])

            # Check if the change in height is below the threshold
            if height_diff < threshold:
                if stable_start_idx is None:
                    stable_start_idx = i - 1  # Mark the start of a stable region
            else:
                # If height change exceeds the threshold and we were in a stable region
                if stable_start_idx is not None:
                    # Check if stable region meets the minimum distance requirement
                    start_dist = distances[stable_start_idx]
                    end_dist = distances[i - 1]

                    if end_dist - start_dist >= min_stable_distance:
                        stable_regions.append(
                            {"start_distance": start_dist, "end_distance": end_dist}
                        )
                    stable_start_idx = None  # Reset stable start index

        # Check the final region if the data ends in a stable region
        if stable_start_idx is not None:
            start_dist = distances[stable_start_idx]
            end_dist = distances[-1]
            if end_dist - start_dist >= min_stable_distance:
                stable_regions.append(
                    {"start_distance": start_dist, "end_distance": end_dist}
                )
        config["logger"].info("[scan-classify] raw stable regions: %s", stable_regions)

        total_length = max(float(chunks[-1]["dist"]) - float(chunks[0]["dist"]), 0.0)
        height_range = max(heights) - min(heights)

        def _samples_in_region(start_dist, end_dist):
            return [
                float(item["height"])
                for item in chunks
                if float(start_dist) <= float(item["dist"]) <= float(end_dist)
            ]

        def _mean_or_nan(values):
            return float(np.mean(values)) if values else float("nan")

        def uniform_depth_result(reason):
            return {
                "frame_1": 0.0,
                "threeD_1": 0.0,
                "pocket": total_length,
                "threeD_2": 0.0,
                "frame_2": 0.0,
                "profileType": "uniform_depth_no_pocket",
                "pocketDetected": False,
                "stableRegionCount": len(stable_regions),
                "heightRange": height_range,
                "depthMm": 0.0,
                "frameAvgHeight": float("nan"),
                "pocketAvgHeight": float("nan"),
                "reason": reason,
            }

        # If we cannot find two stable bands, we treat the door as uniform depth.
        if len(stable_regions) < 2:
            return uniform_depth_result("stable_regions_lt_2")

        filtered_stable_regions = [
            region
            for region in stable_regions
            if (float(region["end_distance"]) - float(region["start_distance"]))
            >= float(min_frame_region_mm)
        ]
        if len(filtered_stable_regions) >= 2:
            selected_stable_regions = filtered_stable_regions
            selection_reason = "filtered_stable_regions"
        else:
            selected_stable_regions = stable_regions
            selection_reason = "raw_stable_regions_fallback"

        config["logger"].info(
            "[scan-classify] selected stable regions (%s, min_frame_region_mm=%.3f): %s",
            selection_reason,
            float(min_frame_region_mm),
            selected_stable_regions,
        )

        if selection_reason == "filtered_stable_regions":
            frame1_point1 = float(selected_stable_regions[0]["start_distance"])
        else:
            frame1_point1 = float(chunks[0]["dist"])
        frame1_point2 = selected_stable_regions[0]["end_distance"]
        pocket_point1 = selected_stable_regions[0]["end_distance"]
        pocket_point2 = selected_stable_regions[-1]["start_distance"]
        frame2_point1 = selected_stable_regions[-1]["start_distance"]
        frame2_point2 = float(chunks[-1]["dist"])

        config["logger"].info("values: %s, %s, %s, %s, %s, %s",
            frame1_point1,
            frame1_point2,
            pocket_point1,
            pocket_point2,
            frame2_point1,
            frame2_point2,
        )
        # input("aaa")

        frame1 = frame1_point2 - frame1_point1 + compensation
        pocket = pocket_point2 - pocket_point1
        threeD1 = pocket_point1 - frame1_point2
        threeD2 = frame2_point1 - pocket_point2
        frame2 = frame2_point2 - frame2_point1 + compensation

        if frame1 < 0 or frame2 < 0 or pocket <= 0:
            return uniform_depth_result("non_positive_segment")

        frame1_heights = _samples_in_region(frame1_point1, frame1_point2)
        frame2_heights = _samples_in_region(frame2_point1, frame2_point2)
        if len(selected_stable_regions) > 2:
            pocket_heights = []
            for region in selected_stable_regions[1:-1]:
                pocket_heights.extend(
                    _samples_in_region(region["start_distance"], region["end_distance"])
                )
        else:
            pocket_heights = _samples_in_region(pocket_point1, pocket_point2)

        frame_avg_height = _mean_or_nan(frame1_heights + frame2_heights)
        pocket_avg_height = _mean_or_nan(pocket_heights)
        if math.isnan(frame_avg_height) or math.isnan(pocket_avg_height):
            depth_mm = 0.0
        else:
            depth_mm = abs(frame_avg_height - pocket_avg_height)

        result = {
            "frame_1": frame1,
            "threeD_1": three_d_compensation,
            "pocket": pocket,
            "threeD_2": three_d_compensation,
            "frame_2": frame2,
            "profileType": "frame_and_pocket",
            "pocketDetected": True,
            "stableRegionCount": len(stable_regions),
            "selectedStableRegionCount": len(selected_stable_regions),
            "heightRange": height_range,
            "depthMm": depth_mm,
            "frameAvgHeight": frame_avg_height,
            "pocketAvgHeight": pocket_avg_height,
            "reason": selection_reason,
        }

        return result

    # Paste this function in Server_Better_V2.py files

    def scan_table(cps, config):
        def find_x_groups(data, n):
            """
            This will return the groups in which valid heights are not NaN.
            A new group is started if n consecutive NaN values are encountered.

            Args:
                data (list): A list of dictionaries containing 'height' values.
                n (int): The number of consecutive NaN values required to start a new group.

            Returns:
                list: A list of groups (each group is a list of dictionaries).
            """
            grouped_valids = []  # To store lists of valid dictionaries
            current_group = []  # To accumulate valid dictionaries in the current group
            nan_count = 0  # To track consecutive NaN values

            for item in data:
                height_value = item.get(
                    "height", float("nan")
                )  # Get the 'height' value, default to NaN if not found

                if math.isnan(float(height_value)):
                    nan_count += 1  # Increment the NaN counter
                else:
                    nan_count = 0  # Reset the NaN counter if a valid value is found
                    current_group.append(item)  # Add valid items to the current group

                # If we encounter n consecutive NaN values, we close the current group
                if nan_count == n:
                    if current_group:
                        grouped_valids.append(current_group)
                        current_group = []  # Reset the current group for the next batch
                    nan_count = 0  # Reset the NaN counter after splitting

            # If there are any valid entries left in the last group, add them too
            if current_group:
                grouped_valids.append(current_group)

            return grouped_valids

        def _non_nan_samples(group):
            return [
                item
                for item in group
                if not math.isnan(float(item.get("height", float("nan"))))
            ]

        def _group_span_mm(group):
            samples = _non_nan_samples(group)
            if len(samples) < 2:
                return 0.0
            dists = [float(item.get("dist", 0.0)) for item in samples]
            return max(dists) - min(dists)

        def connect_j7():
            result = []
            cps.HRIF_HRApp(0, "HR_Motor", "GetMotorList", [], result)
            cps.HRIF_HRApp(0, "HR_Motor", "MotorConnect", ["J7"], result)

        def get_j7_state():
            result = []
            nret = cps.HRIF_HRApp(0, "HR_Motor", "MotorGetState", ["J7"], result)
            if (nret == 0 or nret is None) and len(result) > 2 and str(result[0]) == "0":
                return result[2]
            return None

        def wait_for_j7_idle(timeout_s=45.0, poll_s=0.02):
            start_time = time.time()
            while True:
                if stop_requested():
                    return False
                state = get_j7_state()
                if state is not None and str(state).strip() == "0":
                    return True
                if (time.time() - start_time) >= float(timeout_s):
                    return False
                time.sleep(max(0.005, float(poll_s)))

        # find the 7th axis positions
        robo7thPos = [
            config["table"][f"lengthx{i}"] for i in range(config["table"]["count"])
        ]
        # Optional offset to map homing position to origin when J7 state is unknown.
        j7_zero_offset = float(
            config.get("offset", {}).get(
                "seventhAxisHomeToOrigin",
                config.get("offset", {}).get("seventhAxis", 0),
            )
            or 0
        )
        apply_j7_offset = False
        config["logger"].info("[scan] 7th axis positions robot will move: %s", robo7thPos)

        framePoints = []
        pocketPoints = []
        outerCornerPoints = []
        innerCornerPoints = []
        own7thpos = []
        xVals = []
        yVals = []
        allXMeasurements = []
        scan_threshold = float(
            config.get("scanThresholdDefault", config.get("scanThreshold", {}).get("T1", 1))
        )
        scan_min_stable_distance = float(
            config.get(
                "scanThresholdMinDDefault",
                config.get("scanThresholdMinD", {}).get("T1", 0),
            )
        )
        if scan_min_stable_distance <= 0.0:
            scan_min_stable_distance = 5.0
            config["logger"].warning(
                "[scan] scan_min_stable_distance was <= 0. Using fallback %.1fmm.",
                scan_min_stable_distance,
            )
        scan_three_d_compensation = float(
            config.get("scanThreeDDefault", config.get("model3D", {}).get("1", 0))
        )
        scan_robot_speed = _resolve_ui_ratio(
            config, "robotSpeed", config.get("UI", {}).get("robotSpeed")
        )
        scan_lift_z_mm = float(
            config.get("offset", {}).get("scanLiftZMm", 15.0)
        )
        scan_min_group_points = int(
            config.get("settings", {}).get("scanMinGroupPoints", 20)
        )
        scan_min_group_span_mm = float(
            config.get("settings", {}).get("scanMinDoorSpanMm", 120.0)
        )
        door_cfg = config.get("door", {}) if isinstance(config, dict) else {}
        scan_blend_timeout_s = max(
            0.5, float(door_cfg.get("scanBlendTimeoutSeconds", 7.0))
        )
        config["logger"].info(
            "[scan] Using model-agnostic scan profile: threshold=%s, min_stable_distance=%s, three_d=%s, blend_timeout=%ss, lift_z=%smm, min_group_points=%s, min_group_span=%smm",
            scan_threshold,
            scan_min_stable_distance,
            scan_three_d_compensation,
            scan_blend_timeout_s,
            scan_lift_z_mm,
            scan_min_group_points,
            scan_min_group_span_mm,
        )

        msg_to_frontend(
            api_url=config["server"]["frontEnd_messaging_url"],
            message=f"Table Scanning Started...",
        )
        config["_scan_last_error"] = None

        # Scan is executed on small-door Table A.
        # Legacy non-blocking behavior:
        # drive only Table A down (45°), leave Table B unchanged, then proceed.
        active_table_result = set_table_state(cps, "tableAOpenClose", "Close")
        parked_table_result = {"success": True, "newState": "Unchanged", "message": "Table B left unchanged for scan"}
        config["logger"].info(
            "[scan][INTERLOCK_LEGACY] runtime=%s active(tableA->45deg_cmd)=%s parked(tableB->unchanged)=%s",
            os.path.abspath(__file__),
            active_table_result,
            parked_table_result,
        )

        if config["settings"]["actualScan"]:
            connect_j7()
            state = get_j7_state()
            if state is None or state == "-1":
                apply_j7_offset = True
                config["logger"].warning(
                    "[scan] J7 state unknown; assuming current position is homing. Applying offset if configured."
                )
                if not j7_zero_offset:
                    config["logger"].warning(
                        "[scan] J7 offset not configured; scan will use current J7 position as origin."
                    )
                msg_to_frontend(
                    api_url=config["server"]["frontEnd_messaging_url"],
                    message="Scan safety check warning: J7 state unknown. Assuming homing position and applying offset.",
                )

            if state == "1":
                config["logger"].info("[scan] J7 moving; stopping before scan.")
                stop_result = []
                cps.HRIF_HRApp(0, "HR_Motor", "MotorStop", ["J7"], stop_result)
                for _ in range(50):
                    time.sleep(0.1)
                    state = get_j7_state()
                    if state == "0":
                        break
                if state != "0":
                    config["logger"].error(
                        "[scan] J7 did not stop; aborting scan for safety."
                    )
                    msg_to_frontend(
                        api_url=config["server"]["frontEnd_messaging_url"],
                        message="Scan safety check failed: J7 did not stop. Please verify J7 and try again.",
                    )
                    config["_scan_last_error"] = "Scan safety check failed: J7 did not stop."
                    exit(-1)

            if apply_j7_offset and j7_zero_offset:
                robo7thPos = [pos + j7_zero_offset for pos in robo7thPos]
                config["logger"].info(
                    "[scan] J7 offset applied (%s mm). New positions: %s",
                    j7_zero_offset,
                    robo7thPos,
                )

            if robo7thPos:
                if stop_requested():
                    msg_to_frontend(
                        api_url=config["server"]["frontEnd_messaging_url"],
                        message="Scan stopped by user.",
                    )
                    return ([], [], [], [], [], [], [])
                start_pos = robo7thPos[0]
                config["logger"].info(
                    f"[scan] Pre-scan move to lengthx0: {start_pos}mm"
                )
                pre_scan_ok = communicate(
                    cps=cps,
                    seventh=start_pos,
                    tcp=config["coords"]["tcpLaserPlane1"],
                    ucs=config["coords"]["ucsTable1"],
                    config=config,
                    speed=scan_robot_speed,
                    velocity_profile="robotspeed",
                    wait=True,
                    require_seventh_ok=True,
                )
                if pre_scan_ok is None:
                    msg_to_frontend(
                        api_url=config["server"]["frontEnd_messaging_url"],
                        message="Scan aborted: failed to move J7 to starting position.",
                    )
                    config["_scan_last_error"] = "Scan aborted: failed to move J7 to starting position."
                    return ([], [], [], [], [], [], [])

            for tblCnt, roboPos in enumerate(robo7thPos):
                if stop_requested():
                    msg_to_frontend(
                        api_url=config["server"]["frontEnd_messaging_url"],
                        message="Scan stopped by user.",
                    )
                    return ([], [], [], [], [], [], [])
                ###########################
                ####### x axis moving #####
                ###########################
                xStart = addXVal(
                    addYVal(
                        config["point"]["table1Origin"],
                        config["door"]["frame"] + config["offset"]["doorYOffset"],
                    ),
                    -config["offset"]["scannerOffsetInLeft"],
                )
                xEnd = addXVal(
                    addYVal(
                        config["point"]["table1Origin"],
                        config["door"]["frame"] + config["offset"]["doorYOffset"],
                    ),
                    config["table"][f"length{tblCnt}"]
                    - config["offset"]["scannerOffsetInLeft"],
                )
                xStart = addZVal(xStart, scan_lift_z_mm)
                xEnd = addZVal(xEnd, scan_lift_z_mm)
                config["logger"].info(
                    f"[scan-x] points to move: <start>: {xStart} >>> <end>:{xEnd}"
                )
                x_prestart_offset_mm = 11.0
                xLeadIn = addXVal(xStart, -x_prestart_offset_mm)
                config["logger"].info(
                    f"[scan-x] lead-in point: {xLeadIn} (offset {x_prestart_offset_mm}mm before start)"
                )

                msg_to_frontend(
                    api_url=config["server"]["frontEnd_messaging_url"],
                    message=f"Moving to Section {tblCnt + 1} of Table to Scan Horizontal...",
                )

                if tblCnt > 0:
                    move_ok = communicate(
                        cps=cps,
                        seventh=roboPos,
                        tcp=config["coords"]["tcpLaserPlane1"],
                        ucs=config["coords"]["ucsTable1"],
                        config=config,
                        speed=scan_robot_speed,
                        velocity_profile="robotspeed",
                        wait=False,
                        require_seventh_ok=True,
                    )
                    if move_ok is None:
                        msg_to_frontend(
                            api_url=config["server"]["frontEnd_messaging_url"],
                            message=f"Scan aborted: failed to move J7 to table section {tblCnt + 1}.",
                        )
                        config["_scan_last_error"] = (
                            f"Scan aborted: failed to move J7 to table section {tblCnt + 1}."
                        )
                        return ([], [], [], [], [], [], [])
                    communicate(
                        cps=cps,
                        point=xLeadIn,
                        tcp=config["coords"]["tcpLaserPlane1"],
                        ucs=config["coords"]["ucsTable1"],
                        config=config,
                        speed=scan_robot_speed,
                        velocity_profile="robotspeed",
                        wait=True,
                    )
                    if not wait_for_j7_idle(timeout_s=45.0, poll_s=0.02):
                        msg_to_frontend(
                            api_url=config["server"]["frontEnd_messaging_url"],
                            message=f"Scan aborted: J7 timeout while moving to section {tblCnt + 1}.",
                        )
                        config["_scan_last_error"] = (
                            f"Scan aborted: J7 timeout while moving to section {tblCnt + 1}."
                        )
                        return ([], [], [], [], [], [], [])
                    communicate(
                        cps=cps,
                        point=xStart,
                        tcp=config["coords"]["tcpLaserPlane1"],
                        ucs=config["coords"]["ucsTable1"],
                        config=config,
                        speed=scan_robot_speed,
                        velocity_profile="robotspeed",
                        wait=True,
                    )
                else:
                    move_ok = communicate(
                        cps=cps,
                        seventh=roboPos,
                        tcp=config["coords"]["tcpLaserPlane1"],
                        ucs=config["coords"]["ucsTable1"],
                        config=config,
                        speed=scan_robot_speed,
                        velocity_profile="robotspeed",
                        require_seventh_ok=True,
                    )
                    if move_ok is None:
                        msg_to_frontend(
                            api_url=config["server"]["frontEnd_messaging_url"],
                            message=f"Scan aborted: failed to move J7 to table section {tblCnt + 1}.",
                        )
                        config["_scan_last_error"] = (
                            f"Scan aborted: failed to move J7 to table section {tblCnt + 1}."
                        )
                        return ([], [], [], [], [], [], [])
                    communicate(
                        cps=cps,
                        point=xLeadIn,
                        tcp=config["coords"]["tcpLaserPlane1"],
                        ucs=config["coords"]["ucsTable1"],
                        config=config,
                        speed=scan_robot_speed,
                        velocity_profile="robotspeed",
                        wait=True,
                    )
                    communicate(
                        cps=cps,
                        point=xStart,
                        tcp=config["coords"]["tcpLaserPlane1"],
                        ucs=config["coords"]["ucsTable1"],
                        config=config,
                        speed=scan_robot_speed,
                        velocity_profile="robotspeed",
                        wait=True,
                    )

                config["logger"].info(
                    f"[scan-x] success to move 7th to go to position for table: {tblCnt + 1}, position: {roboPos}"
                )
                config["logger"].info(f"[scan-x] scan start point reached: {xStart}")

                waitForBlending(cps=cps, config=config, timeout_s=scan_blend_timeout_s)
                xmeasurements = communicate(
                    cps=cps,
                    point=xEnd,
                    tcp=config["coords"]["tcpLaserPlane1"],
                    ucs=config["coords"]["ucsTable1"],
                    config=config,
                    doMeasure=1,
                    speed=config["UI"]["scanSpeed"],
                    velocity_profile="sandingspeed",
                )
                print(f"scanSpeed: {config['UI']['scanSpeed']}")
                config["logger"].info(f"[scan-x] end point reached: {xEnd}")

                # saving the initial values (raw scan data)
                saveAsCSV(f"./static/prev_xm{tblCnt}.csv", xmeasurements)
                # add sufficient x distance to the points
                for xmeasurement in xmeasurements:
                    xmeasurement["dist"] += roboPos
                # performing an height adjustment here for better results
                xmeasurements = adjust_heights(xmeasurements)
                saveAsCSV(f"./static/xm{tblCnt}.csv", xmeasurements)
                allXMeasurements += xmeasurements
                config["logger"].info(
                    "[scan-x] X-start return is disabled; staying at xEnd after section scan."
                )

                # Saves the x scan values first
                saveAsCSV("./static/xmeasures.csv", allXMeasurements)
                config["logger"].info("[scan] x scan values saved in xmeasures.csv")
                msg_to_frontend(
                    api_url=config["server"]["frontEnd_messaging_url"],
                    message=f"Section {tblCnt + 1}'s Horizontal Scan Completed!",
                )
        else:
            # load from the saved value
            allXMeasurements = csv_to_dict_list("./static/xmeasures.csv")
            # config['logger'].info("Line 902 AllXMeasurements: ", allXMeasurements)

        ###############################
        ####### group calcul x    #####
        ###############################

        allXMeasurements = find_x_groups(allXMeasurements, 4)
        if not allXMeasurements:
            msg_to_frontend(
                api_url=config["server"]["frontEnd_messaging_url"],
                message="Scan aborted: no valid horizontal scan groups found.",
            )
            config["_scan_last_error"] = "Scan aborted: no valid horizontal scan groups found."
            return ([], [], [], [], [], [], [])

        expected_doors = int(config["table"]["count"])
        if len(allXMeasurements) > expected_doors:
            config["logger"].warning(
                "[scan] Detected %s scan groups, but table count is %s. Trimming likely noise groups.",
                len(allXMeasurements),
                expected_doors,
            )
            # Keep the largest groups (most valid samples), then restore scan order.
            selected_indexes = sorted(
                sorted(
                    range(len(allXMeasurements)),
                    key=lambda idx: len(allXMeasurements[idx]),
                    reverse=True,
                )[:expected_doors]
            )
            allXMeasurements = [allXMeasurements[idx] for idx in selected_indexes]
        elif len(allXMeasurements) < expected_doors:
            config["logger"].warning(
                "[scan] Detected only %s scan groups for table count %s.",
                len(allXMeasurements),
                expected_doors,
            )

        # Filter noisy/false X groups so Y scan runs only on valid detected doors.
        pre_filter_count = len(allXMeasurements)
        filtered_groups = []
        for idx, group in enumerate(allXMeasurements):
            valid_samples = _non_nan_samples(group)
            span_mm = _group_span_mm(group)
            if len(valid_samples) < scan_min_group_points or span_mm < scan_min_group_span_mm:
                config["logger"].warning(
                    "[scan] Skipping X group %s as invalid door candidate (samples=%s span=%.2fmm).",
                    idx + 1,
                    len(valid_samples),
                    span_mm,
                )
                continue
            filtered_groups.append(group)
        allXMeasurements = filtered_groups
        config["logger"].info(
            "[scan] Valid X groups after filtering: %s/%s",
            len(allXMeasurements),
            pre_filter_count,
        )
        if not allXMeasurements:
            msg_to_frontend(
                api_url=config["server"]["frontEnd_messaging_url"],
                message="Scan aborted: no valid doors detected from horizontal scan.",
            )
            config["_scan_last_error"] = "Scan aborted: no valid doors detected from horizontal scan."
            return ([], [], [], [], [], [], [])

        # reversing to start from the last to first (to save time)
        beginning_non_nan = next(
            item for item in allXMeasurements[0] if not np.isnan(item["height"])
        )
        msg_to_frontend(
            api_url=config["server"]["frontEnd_messaging_url"],
            message=f"Horizontal Scan Completed! Doors found: {len(allXMeasurements)}...",
        )
        allXMeasurements.reverse()

        # Define thresholds for each chunk
        total_doors = len(allXMeasurements)
        for xcnt, xmeasurements in enumerate(allXMeasurements):
            if stop_requested():
                msg_to_frontend(
                        api_url=config["server"]["frontEnd_messaging_url"],
                        message="Scan stopped by user.",
                    )    
                return ([], [], [], [], [], [], [])
                ###############################
                ####### point calcu x     #####
                ###############################        

            first_non_nan = next(
                item for item in xmeasurements if not np.isnan(item["height"])
            )
            config["logger"].info("First non nan: ", first_non_nan)

            # xpos = mm_to_inches(first_non_nan['dist'] - config['offset']['scannerOffsetInLeft'] / 2) # this is position of 7th axis
            xpos = (
                first_non_nan["dist"] - beginning_non_nan["dist"]
            )  # this is position of 7th axis

            # xlen, xframe_1, xframe_2 = find_constant_height_periods(xmeasurements, threshold=2)
            # results = calculate_for_each_chunk([xmeasurements], thresholds=[xthresholds[xcnt]])
            # results = identify_gradient_change_points_dynamic(xmeasurements, thresholds=[0.15, 0.1])
            results = identify_gradient_change_points_dynamic(
                xmeasurements,
                threshold=scan_threshold,
                min_stable_distance=scan_min_stable_distance,
                three_d_compensation=scan_three_d_compensation,
                config=config,
            )

            config["logger"].info("[scan] calculated x values for door: ", results)

            # appropriate x values that we shall use
            xframe_1, x_td1, x_pocket, x_td2, xframe_2 = (
                results["frame_1"],
                results["threeD_1"],
                results["pocket"],
                results["threeD_2"],
                results["frame_2"],
            )
            xlen = xframe_1 + x_pocket + xframe_2
            xframe_1 = xframe_1 + x_td1
            xframe_2 = xframe_2 + x_td2
            x_profile_type = results.get("profileType", "unknown")
            x_pocket_detected = bool(results.get("pocketDetected", False))
            xVals.append(
                {
                    "xlen": xlen,
                    "xframe_1": xframe_1,
                    "xframe_2": xframe_2,
                    "xDepthMm": results.get("depthMm", 0),
                    "xProfileType": x_profile_type,
                    "xPocketDetected": x_pocket_detected,
                    "xHeightRange": results.get("heightRange", 0),
                    "xClassificationReason": results.get("reason", ""),
                }
            )

            own7thpos.append(xpos)
            ymeasurements = []
            stop_scan_after_this_door = False

            if config["settings"]["actualScan"]:
                if stop_requested():
                    msg_to_frontend(
                        api_url=config["server"]["frontEnd_messaging_url"],
                        message="Scan stopped by user.",
                    )
                    return ([], [], [], [], [], [], [])
                door_number = total_doors - xcnt
                is_final_scan_iteration = xcnt == (total_doors - 1)
                msg_to_frontend(
                    api_url=config["server"]["frontEnd_messaging_url"],
                    message=f"Moving to Door {door_number} to Scan Vertically...",
                )
                # It gets stuck here and returns code 20018
                # Adjust timer in IsBlendingDone at the top
                ###########################
                ####### y axis moving #####
                ###########################

                # Anchor Y scan on the detected door center so each Y run matches the
                # actual door found in X-scan and avoids tiny/noise segments.
                x_non_nan = _non_nan_samples(xmeasurements)
                x_min_dist = min(float(item["dist"]) for item in x_non_nan)
                x_max_dist = max(float(item["dist"]) for item in x_non_nan)
                x_center_dist = (x_min_dist + x_max_dist) / 2.0
                y_scan_anchor_x = max(0.0, x_center_dist - float(xpos))

                yStart = addYVal(
                    addXVal(config["point"]["table1Origin"], y_scan_anchor_x),
                    -config["offset"]["scannerOffsetInBottom"],
                )
                yLeadIn = addYVal(yStart, -5.0)
                yEnd = addYVal(
                    addYVal(
                        addXVal(config["point"]["table1Origin"], y_scan_anchor_x),
                        config["table"]["width"],
                    ),
                    -config["offset"]["scannerOffsetInBottom"],
                )
                yStart = addZVal(yStart, scan_lift_z_mm)
                yEnd = addZVal(yEnd, scan_lift_z_mm)
                config["logger"].info(
                    f"[scan-y] points to move: <start>: {yStart} >>> <end>:{yEnd}"
                )
                config["logger"].info(
                    "[scan-y] iteration=%s/%s door_label=%s seventh=%.3f",
                    xcnt + 1,
                    total_doors,
                    door_number,
                    float(xpos),
                )
                # Restore simultaneous J7 + move-to-yStart for all doors, including Door 1.
                move_ok = communicate(
                    cps=cps,
                    seventh=xpos,
                    tcp=config["coords"]["tcpLaserPlane1"],
                    ucs=config["coords"]["ucsDefault"],
                    config=config,
                    speed=scan_robot_speed,
                    velocity_profile="robotspeed",
                    wait=False,
                    require_seventh_ok=True,
                )
                if move_ok is None:
                    msg_to_frontend(
                        api_url=config["server"]["frontEnd_messaging_url"],
                        message=f"Scan aborted: failed to move J7 to door position {door_number}.",
                    )
                    return ([], [], [], [], [], [], [])
                communicate(
                    cps=cps,
                    point=yLeadIn,
                    tcp=config["coords"]["tcpLaserPlane1"],
                    ucs=config["coords"]["ucsTable1"],
                    config=config,
                    speed=scan_robot_speed,
                    velocity_profile="robotspeed",
                    wait=True,
                )
                if not wait_for_j7_idle(timeout_s=45.0, poll_s=0.02):
                    msg_to_frontend(
                        api_url=config["server"]["frontEnd_messaging_url"],
                        message=f"Scan aborted: J7 timeout while moving to Door {door_number}.",
                    )
                    return ([], [], [], [], [], [], [])
                communicate(
                    cps=cps,
                    point=yStart,
                    tcp=config["coords"]["tcpLaserPlane1"],
                    ucs=config["coords"]["ucsTable1"],
                    config=config,
                    speed=scan_robot_speed,
                    velocity_profile="robotspeed",
                    wait=True,
                )

                config["logger"].info(
                    f"[scan-y] success to move 7th to go to door position {door_number}, seventh: {xpos}"
                )
                config["logger"].info(f"[scan-y] start point reached: {yStart}")

                waitForBlending(cps=cps, config=config, timeout_s=scan_blend_timeout_s)

                ymeasurements = communicate(
                    cps=cps,
                    point=yEnd,
                    tcp=config["coords"]["tcpLaserPlane1"],
                    ucs=config["coords"]["ucsTable1"],
                    config=config,
                    doMeasure=1,
                    speed=config["UI"]["scanSpeed"],
                    velocity_profile="sandingspeed",
                    # Stop on sustained NaN only after valid data tail (handled in data_collection).
                    stopWhenNan=True,
                )
                config["logger"].info(
                    "[scan-y] collected %s samples for door %s",
                    0 if ymeasurements is None else len(ymeasurements),
                    door_number,
                )
                config["logger"].info(f"[scan-y] end point reached: {yEnd}")
                print(f"scanSpeed: {config['UI']['scanSpeed']}")

                saveAsCSV(f"./static/prev_ym{xcnt}.csv", ymeasurements)
                ymeasurements = adjust_heights(ymeasurements)
                saveAsCSV(f"./static/ym{xcnt}.csv", ymeasurements)

                # communicate(
                #     cps=cps,
                #     point=yStart,
                #     tcp=config["coords"]["tcpLaserPlane1"],
                #     ucs=config["coords"]["ucsTable1"],
                #     seventh=-1,
                #     config=config,
                #     speed=scan_robot_speed,
                #     velocity_profile="robotspeed",
                #     wait=False,
                # )

                msg_to_frontend(
                    api_url=config["server"]["frontEnd_messaging_url"],
                    message=f"Vertical Scanning of Door {door_number} Completed!",
                )
                if door_number == 1 or is_final_scan_iteration:
                    config["logger"].info(
                        "[scan-y] Door 1/final Y-scan complete; moving directly to safe home point."
                    )
                    msg_to_frontend(
                        api_url=config["server"]["frontEnd_messaging_url"],
                        message="Door 1 Y scan complete. Moving directly to homing safe point...",
                    )
                    communicate(
                        cps=cps,
                        point=config["point"]["safePoint"],
                        tcp=config["coords"]["tcpDefault"],
                        ucs=config["coords"]["ucsDefault"],
                        seventh=-1,
                        config=config,
                        speed=scan_robot_speed,
                        velocity_profile="robotspeed",
                        wait=True,
                    )
                    stop_scan_after_this_door = True
            else:
                ymeasurements = csv_to_dict_list(f"./static/ym{xcnt}.csv")

            ###############################
            ####### point calcu y     #####
            ###############################
            msg_to_frontend(
                api_url=config["server"]["frontEnd_messaging_url"],
                message=f"Calculating the Points for All Doors...⚙️",
            )

            # ylen, yframe_1, yframe_2 = find_constant_height_periods(ymeasurements, threshold=2)
            results = identify_gradient_change_points_dynamic(
                ymeasurements,
                threshold=scan_threshold,
                min_stable_distance=scan_min_stable_distance,
                three_d_compensation=scan_three_d_compensation,
                config=config,
            )
            # results = identify_gradient_change_points_dynamic(ymeasurements, thresholds=[0.2, 0.1])
            config["logger"].info("[scan] calculated y values for door: ", results)
            yframe_1, y_td1, y_pocket, y_td2, yframe_2 = (
                results["frame_1"],
                results["threeD_1"],
                results["pocket"],
                results["threeD_2"],
                results["frame_2"],
            )
            ylen = yframe_1 + y_pocket + yframe_2
            yframe_1 = yframe_1 + y_td1
            yframe_2 = yframe_2 + y_td2
            y_profile_type = results.get("profileType", "unknown")
            y_pocket_detected = bool(results.get("pocketDetected", False))
            if (
                x_profile_type == "uniform_depth_no_pocket"
                and y_profile_type == "uniform_depth_no_pocket"
            ):
                door_profile = "uniform_depth_no_pocket"
            elif (
                x_profile_type == "frame_and_pocket"
                and y_profile_type == "frame_and_pocket"
            ):
                door_profile = "frame_and_pocket"
            else:
                door_profile = "mixed_or_uncertain"

            xVals[-1]["doorProfile"] = door_profile
            yVals.append(
                {
                    "ylen": ylen,
                    "yframe_1": yframe_1,
                    "yframe_2": yframe_2,
                    "yDepthMm": results.get("depthMm", 0),
                    "yProfileType": y_profile_type,
                    "yPocketDetected": y_pocket_detected,
                    "yHeightRange": results.get("heightRange", 0),
                    "yClassificationReason": results.get("reason", ""),
                    "doorProfile": door_profile,
                }
            )
            config["logger"].info(
                f"[scan] x stuffs: <total length>: {xlen} mm, <left frame>: {xframe_1} mm, <right frame>: {xframe_2} mm, <depth>: {xVals[-1]['xDepthMm']} mm"
            )
            config["logger"].info(
                f"[scan] y stuffs: <total length>: {ylen} mm, <left frame>: {yframe_1} mm, <right frame>: {yframe_2} mm, <depth>: {yVals[-1]['yDepthMm']} mm"
            )

            # framePoints: points that connect the center of the frames
            framePoints.append(
                {
                    0: addZVal(
                        addXVal(
                            addYVal(config["point"]["table1Origin"], yframe_1 / 2),
                            xframe_1 / 2,
                        ),
                        0,
                    ),
                    1: addZVal(
                        addXVal(
                            addYVal(
                                config["point"]["table1Origin"], ylen - yframe_2 / 2
                            ),
                            xframe_1 / 2,
                        ),
                        0,
                    ),
                    2: addZVal(
                        addXVal(
                            addYVal(
                                config["point"]["table1Origin"], ylen - yframe_2 / 2
                            ),
                            xlen - xframe_2 / 2,
                        ),
                        0,
                    ),
                    3: addZVal(
                        addXVal(
                            addYVal(config["point"]["table1Origin"], yframe_1 / 2),
                            xlen - xframe_2 / 2,
                        ),
                        0,
                    ),
                }
            )
            # innerCornerPoints: points that connect the inner 4 corners (excluding the 3d)
            # the reason I added 2 is because I wanted to add an offset for sanding
            innerCornerPoints.append(
                {
                    0: addZVal(
                        addXVal(
                            addYVal(
                                config["point"]["table1Origin"],
                                yframe_1 - config["offset"]["toolin"],
                            ),
                            +xframe_1 - config["offset"]["toolin"],
                        ),
                        10,
                    ),
                    1: addZVal(
                        addXVal(
                            addYVal(
                                config["point"]["table1Origin"],
                                ylen - yframe_2 + config["offset"]["toolin"],
                            ),
                            +xframe_1 - config["offset"]["toolin"],
                        ),
                        10,
                    ),
                    2: addZVal(
                        addXVal(
                            addYVal(
                                config["point"]["table1Origin"],
                                ylen - yframe_2 + config["offset"]["toolin"],
                            ),
                            +xlen - xframe_2 + config["offset"]["toolin"],
                        ),
                        10,
                    ),
                    3: addZVal(
                        addXVal(
                            addYVal(
                                config["point"]["table1Origin"],
                                yframe_1 - config["offset"]["toolin"],
                            ),
                            +xlen - xframe_2 + config["offset"]["toolin"],
                        ),
                        10,
                    ),
                }
            )
            # outerCornerPoints: the 4 corner points of the door
            outerCornerPoints.append(
                {
                    0: addZVal(
                        addXVal(
                            addYVal(
                                config["point"]["table1Origin"],
                                -config["offset"]["edgeOffset"],
                            ),
                            -config["offset"]["edgeOffset"],
                        ),
                        -55,
                    ),
                    1: addZVal(
                        addXVal(
                            addYVal(
                                config["point"]["table1Origin"],
                                +config["offset"]["edgeOffset"] + ylen,
                            ),
                            -config["offset"]["edgeOffset"],
                        ),
                        -55,
                    ),
                    2: addZVal(
                        addXVal(
                            addYVal(
                                config["point"]["table1Origin"],
                                +config["offset"]["edgeOffset"] + ylen,
                            ),
                            config["offset"]["edgeOffset"] + xlen,
                        ),
                        -55,
                    ),
                    3: addZVal(
                        addXVal(
                            addYVal(
                                config["point"]["table1Origin"],
                                -config["offset"]["edgeOffset"],
                            ),
                            config["offset"]["edgeOffset"] + xlen,
                        ),
                        -55,
                    ),
                }
            )
            # pocketPoints: The inner pocket 4 corner points (considering the offsets of the square tool (called tool3 for some reason))
            pocketPoints.append(
                {
                    0: addZVal(
                        addXVal(
                            addYVal(
                                config["point"]["table1Origin"],
                                +config["offset"]["innerOffset"]
                                + yframe_1
                                + config["offset"]["tool3y"] / 2,
                            ),
                            +config["offset"]["innerOffset"]
                            + xframe_1
                            + config["offset"]["tool3x"] / 2,
                        ),
                        -10,
                    ),
                    1: addZVal(
                        addXVal(
                            addYVal(
                                config["point"]["table1Origin"],
                                -config["offset"]["innerOffset"]
                                + ylen
                                - yframe_2
                                - config["offset"]["tool3y"] / 2,
                            ),
                            +config["offset"]["innerOffset"]
                            + xframe_1
                            + config["offset"]["tool3x"] / 2,
                        ),
                        -10,
                    ),
                    2: addZVal(
                        addXVal(
                            addYVal(
                                config["point"]["table1Origin"],
                                -config["offset"]["innerOffset"]
                                + ylen
                                - yframe_2
                                - config["offset"]["tool3y"] / 2,
                            ),
                            -config["offset"]["innerOffset"]
                            + xlen
                            - xframe_2
                            - config["offset"]["tool3x"] / 2,
                        ),
                        -10,
                    ),
                    3: addZVal(
                        addXVal(
                            addYVal(
                                config["point"]["table1Origin"],
                                +config["offset"]["innerOffset"]
                                + yframe_1
                                + config["offset"]["tool3y"] / 2,
                            ),
                            -config["offset"]["innerOffset"]
                            + xlen
                            - xframe_2
                            - config["offset"]["tool3x"] / 2,
                        ),
                        -10,
                    ),
                }
            )

            # Keep Door 1 direct-homing behavior, but only after all point sets
            # for this door are fully appended. Breaking earlier caused
            # xVals/yVals and point arrays to get out of sync.
            if stop_scan_after_this_door:
                config["logger"].info(
                    "[scan-y] Stopping additional Y-scan iterations after Door 1 completion."
                )
                break

            config["logger"].info(
                f"[scan-calculation] outer sanding points: {framePoints}"
            )
            config["logger"].info(
                f"[scan-calculation] edge  sanding points: {framePoints}"
            )
            config["logger"].info(
                f"[scan-calculation] inner sanding points: {pocketPoints}"
            )
            # if config["settings"]["actualScan"]:
            #     # communicate(cps=cps, point=config['point']['safePoint'], tcp=config['coords']['tcpDefault'], ucs=config['coords']['ucsDefault'], seventh=-1, config=config, speed=config['UI']['robotSpeed'])
            #     communicate(
            #         cps=cps,
            #         point=config["point"]["safepointprehoming"],
            #         tcp=config["coords"]["tcpLaserPlane1"],
            #         ucs=config["coords"]["ucsTable1"],
            #         seventh=-1,
            #         config=config,
            #         speed=scan_robot_speed,
            #         velocity_profile="robotspeed",
            #     )
            #     config["logger"].info("[scan] moved successfully to safepoint")
            msg_to_frontend(
                api_url=config["server"]["frontEnd_messaging_url"],
                message=f"Point Calculated for All Doors. Scanning Completed for the Table!",
            )

        own7thpos.reverse()
        own7thpos = own7thpos[: config["table"]["count"]]
        framePoints.reverse()
        outerCornerPoints.reverse()
        innerCornerPoints.reverse()
        pocketPoints.reverse()
        xVals.reverse()
        yVals.reverse()

        return (
            own7thpos,
            framePoints,
            pocketPoints,
            outerCornerPoints,
            innerCornerPoints,
            xVals,
            yVals,
        )

    def sand_table(
        cps,
        config,
        robo7thPos,
        framePoints,
        pocketPoints,
        outerCornerPoints,
        innerCornerPoints,
        xVals,
        yVals,
    ):
        def moveOnlyJ6(cps, J6, config, wait=True):
            """Only increase the J6 angle to the certain value given. handles both positive and negative values

            Args:
                cps (cps): cobot client
                J6 (float): angle value (in degrees)
                config (config): our config value
            """
            result = []  # Read the current actual location information
            nret = cps.HRIF_ReadActPos(0, 0, result)  # Read the joint position variable
            dJ1 = float(result[0])
            dJ2 = float(result[1])
            dJ3 = float(result[2])
            dJ4 = float(result[3])
            dJ5 = float(result[4])
            dJ6 = float(result[5]) + J6  # Read the spatial position variable

            waitForBlending(cps, config)
            # setSpeed(cps, 0.6, config=config)

            sTcpName = config["coords"][
                "tcpDefault"
            ]  # Definetheusercoordinatesvariable
            sUcsName = config["coords"]["ucsDefault"]  # Definemovementspeed
            dVelocity = 100  # Define the movement acceleration
            dAcc = 150  # Define the transition radius
            dRadius = config["coords"][
                "transitionRadius"
            ]  # Deine whether the joint angle is used
            nIsUseJoint = 1  # Define whether to stop using the test DI
            nIsSeek = 0  # Define The DI index of the detection
            nIOBit = 0  # Define The DI status of the detected state
            nIOState = 0  # Defining Road Point ID
            stdCmdID = "0"  # Perform road point movement

            nret = cps.HRIF_MoveJ(
                0,
                0,
                config["point"]["table1Origin"],
                [dJ1, dJ2, dJ3, dJ4, dJ5, dJ6],
                sTcpName,
                sUcsName,
                dVelocity,
                dAcc,
                dRadius,
                nIsUseJoint,
                nIsSeek,
                nIOBit,
                nIOState,
                stdCmdID,
            )
            robotRes = []
            # time.sleep(0.2)
            if nret != 0:
                config["logger"].error(
                    f"[moveOnlyJ6] Failure in moving to joint pos {[dJ1, dJ2, dJ3, dJ4, dJ5, dJ6]}"
                )
                msg_to_frontend(
                    api_url=config["server"]["frontEnd_messaging_url"],
                    message=f"Cobot Reached Joint Position Limit. Perform Homing and Try Again. Exiting Cycle...",
                )
                exit(-1)
            nret = -1
            while wait:
                nret = cps.HRIF_ReadRobotState(0, 0, robotRes)
                config["logger"].info(robotRes)
                if robotRes[11] == "1":
                    config["logger"].info(
                        f"[moveOnlyJ6] Success in moving to joint pos {[dJ1, dJ2, dJ3, dJ4, dJ5, dJ6]}"
                    )
                    return
                config["logger"].info(
                    f"[moveOnlyJ6] Trying to move joints to {[dJ1, dJ2, dJ3, dJ4, dJ5, dJ6]}"
                )
                # time.sleep(0.2)

        def goToSafeJoints(cps, config):
            sTcpName = config["coords"][
                "tcpDefault"
            ]  # Definetheusercoordinatesvariable
            sUcsName = config["coords"]["ucsDefault"]  # Definemovementspeed
            dVelocity = 100  # Define the movement acceleration
            dAcc = 150  # Define the transition radius
            dRadius = config["coords"][
                "transitionRadius"
            ]  # Deine whether the joint angle is used
            nIsUseJoint = 1  # Define whether to stop using the test DI
            nIsSeek = 0  # Define The DI index of the detection
            nIOBit = 0  # Define The DI status of the detected state
            nIOState = 0  # Defining Road Point ID
            stdCmdID = "0"  # Perform road point movement

            nret = cps.HRIF_MoveJ(
                0,
                0,
                config["point"]["table1Origin"],
                config["point"]["table1OriginAngle"],
                sTcpName,
                sUcsName,
                dVelocity,
                dAcc,
                dRadius,
                nIsUseJoint,
                nIsSeek,
                nIOBit,
                nIOState,
                stdCmdID,
            )
            robotRes = []
            if nret != 0:
                config["logger"].error(
                    f"[goToSafeJoints] Failure in moving to joint pos {[dJ1, dJ2, dJ3, dJ4, dJ5, dJ6]}"
                )
                msg_to_frontend(
                    api_url=config["server"]["frontEnd_messaging_url"],
                    message="Error With Robot Joint Position. Please Verify The Door Size and Try Again. Terminating Process...",
                )
                exit(-1)
            while nret != 0:
                nret = cps.HRIF_ReadRobotState(0, 0, robotRes)
                if robotRes[11] == "1":
                    config["logger"].info(
                        f"[goToSafeJoints] Success in moving to joint pos {config['point']['table1OriginAngle']}"
                    )
                    return
                config["logger"].info(
                    f"[goToSafeJoints] Trying to move joints to {config['point']['table1OriginAngle']}"
                )
                time.sleep(0.2)

        def frameCycle(cps, framePoints, config):
            # Frame Inner cycle
            for frameSandCnt in range(config["UI"]["frameSandCount"]):
                msg_to_frontend(
                    api_url=config["server"]["frontEnd_messaging_url"],
                    message=f"Frame Cycle {frameSandCnt + 1}/{config['UI']['frameSandCount']} Started...",
                )
                for i, point in enumerate([0, 1, 2, 3, 0]):
                    # data = formattedInstruction(outerSandingPoints[tblCnt][point], debug=config['settings']['debug'])
                    config["logger"].info(
                        f"[frameCycle] going to point: {point} of table: {tblCnt}"
                    )
                    if frameSandCnt == 0 and i == 0:
                        communicate(
                            cps=cps,
                            point=framePoints[tblCnt][point],
                            tcp=config["coords"]["tcpFrameTool"],
                            ucs=config["coords"]["ucsTable1"],
                            config=config,
                            speed=config["UI"]["robotSpeed"],
                            wait=True,
                        )
                    else:
                        if (
                            frameSandCnt == 0
                            and i == 1
                            and config["UI"]["frameForce"] != 0
                        ):
                            putForce(
                                cps=cps,
                                force=config["UI"]["frameForce"],
                                tcp=config["coords"]["tcpFrameTool"],
                                ucs=config["coords"]["ucsTable1"],
                                config=config,
                            )
                        communicate(
                            cps=cps,
                            point=framePoints[tblCnt][point],
                            tcp=config["coords"]["tcpFrameTool"],
                            ucs=config["coords"]["ucsTable1"],
                            config=config,
                            wait=False,
                        )
                    config["logger"].info(
                        f"[frameCycle] reached  point: {point} of table: {tblCnt}"
                    )

                msg_to_frontend(
                    api_url=config["server"]["frontEnd_messaging_url"],
                    message=f"Frame Cycle {frameSandCnt + 1}/{config['UI']['frameSandCount']} Completed!",
                )
            waitForBlending(cps, config)
            if config["UI"]["frameForce"] != 0:
                releaseForce(cps=cps, config=config)
            communicate(
                cps=cps,
                point=framePoints[tblCnt][0],
                tcp=config["coords"]["tcpFrameTool"],
                ucs=config["coords"]["ucsTable1"],
                config=config,
                speed=config["UI"]["robotSpeed"],
                wait=False,
            )

        def pocketSQCycle(cps, pocketPoints, config):
            # Frame Inner cycle
            for pocketSandCnt in range(config["UI"]["pocketSQSandCnt"]):
                msg_to_frontend(
                    api_url=config["server"]["frontEnd_messaging_url"],
                    message=f"Pocket Square Cycle {pocketSandCnt + 1}/{config['UI']['pocketSQSandCnt']} Started...",
                )
                for i, point in enumerate([0, 1, 2, 3, 0]):
                    # data = formattedInstruction(outerSandingPoints[tblCnt][point], debug=config['settings']['debug'])
                    config["logger"].info(
                        f"[pocketSQCycle] going to point: {point} of table: {tblCnt}"
                    )
                    if pocketSandCnt == 0 and i == 0:
                        communicate(
                            cps=cps,
                            point=pocketPoints[tblCnt][point],
                            tcp=config["coords"]["tcpFrameTool"],
                            ucs=config["coords"]["ucsTable1"],
                            config=config,
                            speed=config["UI"]["robotSpeed"],
                            wait=True,
                        )
                    else:
                        if (
                            pocketSandCnt == 0
                            and i == 1
                            and config["UI"]["pocketSQForce"] != 0
                        ):
                            putForce(
                                cps=cps,
                                force=config["UI"]["pocketSQForce"],
                                tcp=config["coords"]["tcpFrameTool"],
                                ucs=config["coords"]["ucsTable1"],
                                config=config,
                            )
                        communicate(
                            cps=cps,
                            point=pocketPoints[tblCnt][point],
                            tcp=config["coords"]["tcpFrameTool"],
                            ucs=config["coords"]["ucsTable1"],
                            config=config,
                            speed=config["UI"]["sandSpeed"],
                            wait=False,
                        )
                    # cps.waitBlendingDone(0,0)

                    config["logger"].info(
                        f"[pocketSQCycle] reached  point: {point} of table: {tblCnt}"
                    )
                msg_to_frontend(
                    api_url=config["server"]["frontEnd_messaging_url"],
                    message=f"Pocket Square Cycle {pocketSandCnt + 1}/{config['UI']['pocketSQSandCnt']} Completed!",
                )
            waitForBlending(cps, config)
            if config["UI"]["pocketSQForce"] != 0:
                releaseForce(cps=cps, config=config)
            communicate(
                cps=cps,
                point=pocketPoints[tblCnt][0],
                tcp=config["coords"]["tcpFrameTool"],
                ucs=config["coords"]["ucsTable1"],
                config=config,
                speed=config["UI"]["robotSpeed"],
            )

        def threeDCycle(cps, pocketPoints, config):
            # 3D Sanding Cycle
            for threeDSandCnt in range(config["UI"]["threeDSandCount"]):
                msg_to_frontend(
                    api_url=config["server"]["frontEnd_messaging_url"],
                    message=f"3D Sanding Cycle {threeDSandCnt + 1}/{config['UI']['threeDSandCount']} Started...",
                )
                for i, point in enumerate([0, 1, 2, 3, 0]):
                    # data = formattedInstruction(outerSandingPoints[tblCnt][point], debug=config['settings']['debug'])
                    config["logger"].info(
                        f"[innerCycle - 3D] going to point: {point} of table: {tblCnt}"
                    )
                    if threeDSandCnt == 0 and i == 0:
                        communicate(
                            cps=cps,
                            point=pocketPoints[tblCnt][point],
                            tcp=config["coords"]["tcpFrameTool"],
                            ucs=config["coords"]["ucsTable1"],
                            config=config,
                            speed=config["UI"]["robotSpeed"],
                            wait=True,
                        )
                    else:
                        if (
                            threeDCycle == 0
                            and i == 1
                            and config["UI"]["threeDForce"] != 0
                        ):
                            putForce(
                                cps=cps,
                                force=config["UI"]["threeDForce"],
                                tcp=config["coords"]["tcp3DTool"],
                                ucs=config["coords"]["ucsTable1"],
                                config=config,
                            )
                        communicate(
                            cps=cps,
                            point=pocketPoints[tblCnt][point],
                            tcp=config["coords"]["tcp3DTool"],
                            ucs=config["coords"]["ucsTable1"],
                            config=config,
                            speed=config["UI"]["sandSpeed"],
                            wait=False,
                        )

                    config["logger"].info(
                        f"[innerCycle - 3D] reached  point: {point} of table: {tblCnt}"
                    )
                msg_to_frontend(
                    api_url=config["server"]["frontEnd_messaging_url"],
                    message=f"3D Sanding Cycle {threeDSandCnt + 1}/{config['UI']['threeDSandCount']} Completed!",
                )
            waitForBlending(cps, config)
            if config["UI"]["threeDForce"] != 0:
                releaseForce(cps=cps, config=config)
            communicate(
                cps=cps,
                point=pocketPoints[tblCnt][0],
                tcp=config["coords"]["tcpFrameTool"],
                ucs=config["coords"]["ucsTable1"],
                config=config,
                speed=config["UI"]["robotSpeed"],
                wait=True,
            )

        def pocketZigZagCycle(cps, yVal, pocketPoints, config):
            # pocket cycle
            for pocketSandCnt in range(config["UI"]["pocketZigSandCnt"]):
                msg_to_frontend(
                    api_url=config["server"]["frontEnd_messaging_url"],
                    message=f"Pocket ZigZag Cycle {pocketSandCnt + 1}/{config['UI']['pocketZigSandCnt']} Started...",
                )
                point = []
                yinner = (
                    yVal["ylen"]
                    - yVal["yframe_1"]
                    - yVal["yframe_2"]
                    - config["offset"]["tool3y"]
                    - 2 * config["offset"]["innerOffset"]
                )
                offset = 0
                toggle = 0
                # this was added to do one extra cycle
                while offset <= yinner:
                    points = [
                        addYVal(pocketPoints[1], -offset),
                        addYVal(pocketPoints[2], -offset),
                    ]

                    if toggle:
                        points.reverse()

                    for i, point in enumerate(points):
                        config["logger"].info(
                            f"[pocketCycle] going to point: {point} of table: {tblCnt}"
                        )
                        if i != 0:
                            if offset == 0 and config["UI"]["pocketZigForce"] != 0:
                                putForce(
                                    cps=cps,
                                    force=config["UI"]["pocketZigForce"],
                                    tcp=config["coords"]["tcpPocketTool"],
                                    ucs=config["coords"]["ucsTable1"],
                                    config=config,
                                )
                            communicate(
                                cps=cps,
                                point=point,
                                tcp=config["coords"]["tcpPocketTool"],
                                ucs=config["coords"]["ucsTable1"],
                                config=config,
                                speed=config["UI"]["sandSpeed"],
                                wait=False,
                            )
                        else:
                            if offset == 0:
                                communicate(
                                    cps=cps,
                                    point=point,
                                    tcp=config["coords"]["tcpPocketTool"],
                                    ucs=config["coords"]["ucsTable1"],
                                    config=config,
                                    speed=config["UI"]["robotSpeed"],
                                    wait=True,
                                )
                            else:
                                communicate(
                                    cps=cps,
                                    point=point,
                                    tcp=config["coords"]["tcpPocketTool"],
                                    ucs=config["coords"]["ucsTable1"],
                                    config=config,
                                    speed=config["UI"]["sandSpeed"],
                                    wait=False,
                                )
                        # if i != 0:
                        #     if config['UI']['controlPressure'] != 0: releaseForce(cps=cps, config=config)
                        config["logger"].info(
                            f"[pocketCycle] reached  point: {point} of table: {tblCnt}"
                        )

                    offset += config["offset"]["innerSandingOffset"]
                    toggle = 1 - toggle

                    if (
                        offset > yinner
                        and offset < yinner + config["offset"]["innerSandingOffset"]
                    ):
                        offset = yinner
                waitForBlending(cps, config)
                if config["UI"]["pocketZigForce"] != 0:
                    releaseForce(cps=cps, config=config)
                    communicate(
                        cps=cps,
                        point=point,
                        tcp=config["coords"]["tcpPocketTool"],
                        ucs=config["coords"]["ucsTable1"],
                        config=config,
                        speed=config["UI"]["robotSpeed"],
                    )
                # keepTool(cps, toolNumber=2, config=config)
                msg_to_frontend(
                    api_url=config["server"]["frontEnd_messaging_url"],
                    message=f"Pocket ZigZag Cycle {pocketSandCnt + 1}/{config['UI']['pocketZigSandCnt']} Completed!",
                )

        def sideCycle(cps, outerCornerPoints, config):
            # outer sanding cycle
            angles = [-90, 90, 90, 90]
            rx = [
                config["point"]["weirdToolTable1Origin"][3],
                config["point"]["weirdToolTable1Origin"][3],
                config["point"]["weirdToolTable1Origin"][3],
                config["point"]["weirdToolTable1Origin"][3],
                config["point"]["weirdToolTable1Origin"][3],
            ]
            ry = [
                config["point"]["weirdToolTable1Origin"][4],
                config["point"]["weirdToolTable1Origin"][4],
                config["point"]["weirdToolTable1Origin"][4],
                config["point"]["weirdToolTable1Origin"][4],
                config["point"]["weirdToolTable1Origin"][4],
            ]
            rz = [90, 0, -90, 180, 180]

            goal = [[1, 0, 0], [0, 1, 0], [1, 0, 0], [0, 1, 0]]

            forceDirection = [1, -1, -1, 1]

            for sideSandCnt in range(config["UI"]["sideSandCount"]):
                msg_to_frontend(
                    api_url=config["server"]["frontEnd_messaging_url"],
                    message=f"Side Sanding Cycle {sideSandCnt + 1}/{config['UI']['sideSandCount']} Started...",
                )
                forwardOrBackward = 0
                pointList = []

                if sideSandCnt % 2:
                    # forwardOrBackward = -1
                    # pointList = [3, 2, 1, 0]
                    forwardOrBackward = 1
                    pointList = [0, 1, 2, 3]
                else:
                    forwardOrBackward = 1
                    pointList = [0, 1, 2, 3]

                for idxm, point in enumerate(pointList):
                    # data = formattedInstruction(outerSandingPoints[tblCnt][point], debug=config['settings']['debug'])
                    # waitForBlending(cps, config)
                    moveOnlyJ6(cps, angles[idxm], config, wait=True)
                    piont1 = [
                        outerCornerPoints[tblCnt][point][0],
                        outerCornerPoints[tblCnt][point][1],
                        outerCornerPoints[tblCnt][point][2],
                        rx[idxm],
                        ry[idxm],
                        rz[idxm],
                    ]
                    piont2 = [
                        outerCornerPoints[tblCnt][(point + forwardOrBackward) % 4][0],
                        outerCornerPoints[tblCnt][(point + forwardOrBackward) % 4][1],
                        outerCornerPoints[tblCnt][(point + forwardOrBackward) % 4][2],
                        rx[idxm],
                        ry[idxm],
                        rz[idxm],
                    ]

                    if idxm == 0:
                        pointInit = addZVal(piont1, +55)
                        config["logger"].info(
                            f"[sideCycle] going to init point: {pointInit} of table: {tblCnt}"
                        )
                        communicate(
                            cps=cps,
                            point=pointInit,
                            config=config,
                            tcp=config["coords"]["tcpSideTool"],
                            ucs=config["coords"]["ucsTable1"],
                            speed=config["UI"]["robotSpeed"],
                            wait=False,
                        )
                        config["logger"].info(
                            f"[sideCycle] reached init point: {pointInit} of table: {tblCnt}"
                        )

                    config["logger"].info(
                        f"[sideCycle] going to point: {piont1} of table: {tblCnt}"
                    )
                    communicate(
                        cps=cps,
                        point=piont1,
                        config=config,
                        tcp=config["coords"]["tcpSideTool"],
                        ucs=config["coords"]["ucsTable1"],
                        speed=config["UI"]["robotSpeed"],
                        wait=False,
                    )
                    config["logger"].info(
                        f"[sideCycle] reached  point: {piont1} of table: {tblCnt}"
                    )

                    config["logger"].info(
                        f"[sideCycle] going to point: {piont2} of table: {tblCnt}"
                    )
                    waitForBlending(cps, config)
                    if config["UI"]["sideForce"] != 0:
                        putForce(
                            cps=cps,
                            force=forceDirection[idxm] * config["UI"]["sideForce"],
                            goal=goal[idxm],
                            tcp=config["coords"]["tcpSideTool"],
                            ucs=config["coords"]["ucsTable1"],
                            config=config,
                        )
                    communicate(
                        cps=cps,
                        point=piont2,
                        tcp=config["coords"]["tcpSideTool"],
                        ucs=config["coords"]["ucsTable1"],
                        config=config,
                        speed=config["UI"]["sandSpeed"],
                        wait=False,
                    )
                    waitForBlending(cps, config)
                    if config["UI"]["sideForce"] != 0:
                        releaseForce(cps=cps, config=config)
                    communicate(
                        cps=cps,
                        point=piont2,
                        tcp=config["coords"]["tcpSideTool"],
                        ucs=config["coords"]["ucsTable1"],
                        config=config,
                        speed=config["UI"]["robotSpeed"],
                        wait=False,
                    )
                    config["logger"].info(
                        f"[sideCycle] reached  point: {piont2} of table: {tblCnt}"
                    )

                    if idxm == len(pointList) - 1:
                        pointInit = addZVal(piont2, +55)
                        config["logger"].info(
                            f"[sideCycle] going to init point: {pointInit} of table: {tblCnt}"
                        )
                        communicate(
                            cps=cps,
                            point=pointInit,
                            tcp=config["coords"]["tcpSideTool"],
                            ucs=config["coords"]["ucsTable1"],
                            config=config,
                            speed=config["UI"]["robotSpeed"],
                        )
                        config["logger"].info(
                            f"[sideCycle] reached init point: {pointInit} of table: {tblCnt}"
                        )
                        # waitForBlending(cps, config)
                        moveOnlyJ6(cps, -180, config, wait=True)
                msg_to_frontend(
                    api_url=config["server"]["frontEnd_messaging_url"],
                    message=f"Side Sanding Cycle {sideSandCnt + 1}/{config['UI']['sideSandCount']} Completed!",
                )

        def outEdgeCycle(cps, pocketPoints, config):
            # outer sanding cycle
            angles = [-90, 90, 90, 90]
            rx = [30, 30, 30, 30]
            ry = [
                config["point"]["weirdToolTable1Origin"][4],
                config["point"]["weirdToolTable1Origin"][4],
                config["point"]["weirdToolTable1Origin"][4],
                config["point"]["weirdToolTable1Origin"][4],
                config["point"]["weirdToolTable1Origin"][4],
            ]
            rz = [90, 0, -90, 180, 180]

            goal = [[1, 0, 0], [0, 1, 0], [1, 0, 0], [0, 1, 0]]

            forceDirection = [1, -1, -1, 1]

            for edgeSandCnt in range(config["UI"]["outedgeSandCount"]):
                msg_to_frontend(
                    api_url=config["server"]["frontEnd_messaging_url"],
                    message=f"Outer Edge Sanding Cycle {edgeSandCnt + 1}/{config['UI']['outedgeSandCount']} Started...",
                )
                forwardOrBackward = 0
                pointList = []

                if edgeSandCnt % 2:
                    # forwardOrBackward = -1
                    # pointList = [3, 2, 1, 0]
                    forwardOrBackward = 1
                    pointList = [0, 1, 2, 3]
                else:
                    forwardOrBackward = 1
                    pointList = [0, 1, 2, 3]

                for idxm, point in enumerate(pointList):
                    # data = formattedInstruction(outerSandingPoints[tblCnt][point], debug=config['settings']['debug'])
                    # waitForBlending(cps, config)
                    moveOnlyJ6(cps, angles[idxm], config, wait=True)
                    piont1 = [
                        pocketPoints[tblCnt][point][0],
                        pocketPoints[tblCnt][point][1],
                        pocketPoints[tblCnt][point][2],
                        rx[idxm],
                        ry[idxm],
                        rz[idxm],
                    ]
                    piont2 = [
                        pocketPoints[tblCnt][(point + forwardOrBackward) % 4][0],
                        pocketPoints[tblCnt][(point + forwardOrBackward) % 4][1],
                        pocketPoints[tblCnt][(point + forwardOrBackward) % 4][2],
                        rx[idxm],
                        ry[idxm],
                        rz[idxm],
                    ]

                    if idxm == 0:
                        pointInit = addZVal(piont1, +100)
                        config["logger"].info(
                            f"[outedgeCycle] going to init point: {pointInit} of table: {tblCnt}"
                        )
                        communicate(
                            cps=cps,
                            point=pointInit,
                            tcp=config["coords"]["tcpEdgeTool"],
                            ucs=config["coords"]["ucsTable1"],
                            config=config,
                            speed=config["UI"]["robotSpeed"],
                            wait=False,
                        )
                        config["logger"].info(
                            f"[outedgeCycle] reached init point: {pointInit} of table: {tblCnt}"
                        )

                    config["logger"].info(
                        f"[outedgeCycle] going to point: {piont1} of table: {tblCnt}"
                    )
                    communicate(
                        cps=cps,
                        point=piont1,
                        tcp=config["coords"]["tcpEdgeTool"],
                        ucs=config["coords"]["ucsTable1"],
                        config=config,
                        speed=config["UI"]["robotSpeed"],
                        wait=False,
                    )
                    config["logger"].info(
                        f"[outedgeCycle] reached  point: {piont1} of table: {tblCnt}"
                    )

                    config["logger"].info(
                        f"[outedgeCycle] going to point: {piont2} of table: {tblCnt}"
                    )
                    waitForBlending(cps, config)
                    if config["UI"]["outedgeForce"] != 0:
                        putForce(
                            cps=cps,
                            force=forceDirection[idxm] * config["UI"]["outedgeForce"],
                            goal=goal[idxm],
                            tcp=config["coords"]["tcpSideTool"],
                            ucs=config["coords"]["ucsTable1"],
                            config=config,
                        )
                    communicate(
                        cps=cps,
                        point=piont2,
                        tcp=config["coords"]["tcpEdgeTool"],
                        ucs=config["coords"]["ucsTable1"],
                        config=config,
                        speed=config["UI"]["sandSpeed"],
                        wait=False,
                    )
                    waitForBlending(cps, config)
                    if config["UI"]["outedgeForce"] != 0:
                        releaseForce(cps=cps, config=config)
                    config["logger"].info(
                        f"[outedgeCycle] reached  point: {piont2} of table: {tblCnt}"
                    )

                    if idxm == len(pointList) - 1:
                        pointInit = addZVal(piont2, +100)
                        config["logger"].info(
                            f"[outedgeCycle] going to init point: {pointInit} of table: {tblCnt}"
                        )
                        communicate(
                            cps=cps,
                            point=pointInit,
                            tcp=config["coords"]["tcpEdgeTool"],
                            ucs=config["coords"]["ucsTable1"],
                            config=config,
                            speed=config["UI"]["robotSpeed"],
                        )
                        config["logger"].info(
                            f"[outedgeCycle] reached init point: {pointInit} of table: {tblCnt}"
                        )
                        # waitForBlending(cps, config)
                        moveOnlyJ6(cps, -180, config, wait=True)
                msg_to_frontend(
                    api_url=config["server"]["frontEnd_messaging_url"],
                    message=f"Outer Edge Sanding Cycle {edgeSandCnt + 1}/{config['UI']['outedgeSandCount']} Completed!",
                )

        def inEdgeCycle(cps, pocketPoints, config):
            # inner sanding cycle
            offsetVal = 5
            angle = 12.5
            offsetX = [
                [offsetVal, offsetVal],
                [
                    offsetVal + config["offset"]["tool3x"],
                    -offsetVal - config["offset"]["tool3x"],
                ],
                [-10, -10],
                [
                    -10 - config["offset"]["tool3x"],
                    offsetVal + config["offset"]["tool3x"],
                ],
            ]
            offsetY = [
                [
                    offsetVal + config["offset"]["tool3y"],
                    -offsetVal - config["offset"]["tool3y"],
                ],
                [-offsetVal, -offsetVal],
                [
                    -offsetVal - config["offset"]["tool3y"],
                    offsetVal + config["offset"]["tool3y"],
                ],
                [offsetVal, offsetVal],
            ]
            rxy = [
                [config["point"]["table1Origin"][3], angle],
                [angle, config["point"]["table1Origin"][4]],
                [config["point"]["table1Origin"][3], -angle],
                [-angle, config["point"]["table1Origin"][4]],
            ]

            rz = [
                config["point"]["table1Origin"][5],
                config["point"]["table1Origin"][5],
                config["point"]["table1Origin"][5],
                config["point"]["table1Origin"][5],
                config["point"]["table1Origin"][5],
            ]

            goal = [[0, 0, 1], [0, 0, 1], [0, 0, 1], [0, 0, 1]]

            forceDirection = [1, 1, 1, 1]

            for edgeSandCnt in range(config["UI"]["inedgeSandCount"]):
                msg_to_frontend(
                    api_url=config["server"]["frontEnd_messaging_url"],
                    message=f"Inner Edge Sanding Cycle {edgeSandCnt + 1}/{config['UI']['inedgeSandCount']} Started...",
                )
                forwardOrBackward = 0
                pointList = []

                if edgeSandCnt % 2:
                    # forwardOrBackward = -1
                    # pointList = [3, 2, 1, 0]
                    forwardOrBackward = 1
                    pointList = [0, 1, 2, 3]
                else:
                    forwardOrBackward = 1
                    pointList = [0, 1, 2, 3]

                for idxm, point in enumerate(pointList):
                    # data = formattedInstruction(outerSandingPoints[tblCnt][point], debug=config['settings']['debug'])
                    piont1 = [
                        pocketPoints[tblCnt][point][0] + offsetX[idxm][0] + 2,
                        pocketPoints[tblCnt][point][1] + offsetY[idxm][0] + 2,
                        pocketPoints[tblCnt][point][2],
                        rxy[idxm][0],
                        rxy[idxm][1],
                        rz[idxm],
                    ]
                    piont2 = [
                        pocketPoints[tblCnt][(point + forwardOrBackward) % 4][0]
                        + offsetX[idxm][1]
                        + 2,
                        pocketPoints[tblCnt][(point + forwardOrBackward) % 4][1]
                        + offsetY[idxm][1]
                        + 2,
                        pocketPoints[tblCnt][(point + forwardOrBackward) % 4][2],
                        rxy[idxm][0],
                        rxy[idxm][1],
                        rz[idxm],
                    ]

                    config["logger"].info(
                        f"[inedgeCycle] going to point: {piont1} of table: {tblCnt}"
                    )
                    communicate(
                        cps=cps,
                        point=piont1,
                        tcp=config["coords"]["tcpEdgeToolIn"],
                        ucs=config["coords"]["ucsTable1"],
                        config=config,
                        speed=config["UI"]["robotSpeed"],
                        wait=False,
                    )
                    config["logger"].info(
                        f"[inedgeCycle] reached  point: {piont1} of table: {tblCnt}"
                    )

                    config["logger"].info(
                        f"[inedgeCycle] going to point: {piont2} of table: {tblCnt}"
                    )
                    waitForBlending(cps, config)
                    if config["UI"]["inedgeForce"] != 0:
                        putForce(
                            cps=cps,
                            force=forceDirection[idxm] * config["UI"]["inedgeForce"],
                            goal=goal[idxm],
                            tcp=config["coords"]["tcpSideTool"],
                            ucs=config["coords"]["ucsTable1"],
                            config=config,
                        )
                    communicate(
                        cps=cps,
                        point=piont2,
                        tcp=config["coords"]["tcpEdgeToolIn"],
                        ucs=config["coords"]["ucsTable1"],
                        config=config,
                        speed=config["UI"]["sandSpeed"],
                        wait=False,
                    )
                    waitForBlending(cps, config)
                    if config["UI"]["inedgeForce"] != 0:
                        releaseForce(cps=cps, config=config)
                    communicate(
                        cps=cps,
                        point=piont2,
                        tcp=config["coords"]["tcpEdgeToolIn"],
                        ucs=config["coords"]["ucsTable1"],
                        config=config,
                        speed=config["UI"]["sandSpeed"],
                        wait=False,
                    )
                    config["logger"].info(
                        f"[inedgeCycle] reached  point: {piont2} of table: {tblCnt}"
                    )

                msg_to_frontend(
                    api_url=config["server"]["frontEnd_messaging_url"],
                    message=f"Inner Edge Sanding Cycle {edgeSandCnt + 1}/{config['UI']['inedgeSandCount']} Completed!",
                )

        doSideCycle = (
            config["toolForModels"][f"model{config['UI']['model']}"][0] == 1
            and config["UI"]["sideSandCount"] > 0
        )
        doFrameCycle = (
            config["toolForModels"][f"model{config['UI']['model']}"][1] == 1
            and config["UI"]["frameSandCount"] > 0
        )
        doOutEdgeCycle = (
            config["toolForModels"][f"model{config['UI']['model']}"][2] == 1
            and config["UI"]["outedgeSandCount"] > 0
        )
        doInEdgeCycle = (
            config["toolForModels"][f"model{config['UI']['model']}"][3] == 1
            and config["UI"]["inedgeSandCount"] > 0
        )
        doPocketZigCycle = (
            config["toolForModels"][f"model{config['UI']['model']}"][4] == 1
            and config["UI"]["pocketZigSandCnt"] > 0
        )
        doPocketSqCycle = (
            config["toolForModels"][f"model{config['UI']['model']}"][5] == 1
            and config["UI"]["pocketSQSandCnt"] > 0
        )
        do3DCycle = (
            config["toolForModels"][f"model{config['UI']['model']}"][6] == 1
            and config["UI"]["threeDSandCount"] > 0
        )

        # Which tools are currently picked
        # Positions are 1, 2, 3, 4 (left top == 1, and the rest are counter clockwise)
        # toolsPicked = {1: False,
        #                2: False,
        #                3: False,
        #                4: False}

        #########################################################
        ########### Tool Pick # 1: Pick the Side tool ###########
        msg_to_frontend(
            api_url=config["server"]["frontEnd_messaging_url"],
            message=f"Sanding Started For the Scanned Doors Using the Chosen Options...",
        )
        enum7thPos = list(reversed(list(enumerate(robo7thPos))))

        if doSideCycle or doOutEdgeCycle:
            # toolsPicked[config['tool']['sideToolPos']] = True
            if not getTool(cps, toolNumber=config["tool"]["sideToolPos"], config=config):
                return

        if doSideCycle:
            enum7thPos = list(reversed(enum7thPos))
            origin_point = config["point"]["weirdToolTable1Origin"]
            origin_ucs = config["coords"]["ucsTable1"]
            origin_tcp = config["coords"]["tcpSideTool"]

            for tblCnt, roboPos in enum7thPos:
                msg_to_frontend(
                    api_url=config["server"]["frontEnd_messaging_url"],
                    message=f"Side Cycle Started for Door {tblCnt + 1}/{len(enum7thPos)}...",
                )
                communicate(
                    cps=cps,
                    point=origin_point,
                    tcp=origin_tcp,
                    ucs=origin_ucs,
                    seventh=-1,
                    config=config,
                    speed=config["UI"]["robotSpeed"],
                    wait=False,
                )
                waitForBlending(cps=cps, config=config)
                communicate(
                    cps=cps,
                    tcp=config["coords"]["tcpDefault"],
                    ucs=config["coords"]["ucsDefault"],
                    seventh=roboPos,
                    config=config,
                    speed=config["UI"]["robotSpeed"],
                )
                sideCycle(cps=cps, outerCornerPoints=outerCornerPoints, config=config)
                waitForBlending(cps=cps, config=config)
                communicate(
                    cps=cps,
                    point=origin_point,
                    tcp=origin_tcp,
                    ucs=origin_ucs,
                    seventh=-1,
                    config=config,
                    speed=config["UI"]["robotSpeed"],
                    wait=False,
                )
                msg_to_frontend(
                    api_url=config["server"]["frontEnd_messaging_url"],
                    message=f"Side Cycle Completed for Door {tblCnt + 1}/{len(enum7thPos)}!",
                )

        if doOutEdgeCycle:
            enum7thPos = list(reversed(enum7thPos))
            origin_point = config["point"]["weirdToolTable1Origin"]
            origin_ucs = config["coords"]["ucsTable1"]
            origin_tcp = config["coords"]["tcpEdgeTool"]

            for tblCnt, roboPos in enum7thPos:
                msg_to_frontend(
                    api_url=config["server"]["frontEnd_messaging_url"],
                    message=f"Out Edge Cycle Started for Door {tblCnt + 1}/{len(enum7thPos)}...",
                )
                communicate(
                    cps=cps,
                    point=origin_point,
                    tcp=origin_tcp,
                    ucs=origin_ucs,
                    seventh=-1,
                    config=config,
                    speed=config["UI"]["robotSpeed"],
                    wait=False,
                )

                waitForBlending(cps=cps, config=config)
                communicate(
                    cps=cps,
                    tcp=config["coords"]["tcpDefault"],
                    ucs=config["coords"]["ucsDefault"],
                    seventh=roboPos,
                    config=config,
                    speed=config["UI"]["robotSpeed"],
                )
                outEdgeCycle(cps=cps, pocketPoints=outerCornerPoints, config=config)
                communicate(
                    cps=cps,
                    point=origin_point,
                    tcp=origin_tcp,
                    ucs=origin_ucs,
                    seventh=-1,
                    config=config,
                    speed=config["UI"]["robotSpeed"],
                    wait=False,
                )
                msg_to_frontend(
                    api_url=config["server"]["frontEnd_messaging_url"],
                    message=f"Out Edge Cycle Completed for Door {tblCnt + 1}/{len(enum7thPos)}!",
                )

        # if side tool is picked, keep the tool (since it will not be used after this)
        if doSideCycle or doOutEdgeCycle:
            keepTool(
                cps,
                toolNumber=config["tool"]["sideToolPos"],
                config=config,
                goToSafe=not (
                    doFrameCycle
                    or doInEdgeCycle
                    or doPocketSqCycle
                    or doPocketZigCycle
                    or do3DCycle
                ),
            )

        #########################################################
        ########### Tool Pick # 2: Pick the flat tool ###########
        if doFrameCycle or doInEdgeCycle or doPocketSqCycle or doPocketZigCycle:
            if not getTool(
                cps,
                toolNumber=config["tool"]["frameToolPos"],
                config=config,
                startFromSafe=not (doSideCycle or doOutEdgeCycle),
            ):
                return

        if doFrameCycle:
            enum7thPos = list(reversed(enum7thPos))
            origin_point = config["point"]["table1Origin"]
            origin_ucs = config["coords"]["ucsTable1"]
            origin_tcp = config["coords"]["tcpFrameTool"]

            for tblCnt, roboPos in enum7thPos:
                msg_to_frontend(
                    api_url=config["server"]["frontEnd_messaging_url"],
                    message=f"Frame Cycle Started for Door {tblCnt + 1}/{len(enum7thPos)}...",
                )
                communicate(
                    cps=cps,
                    point=origin_point,
                    tcp=origin_tcp,
                    ucs=origin_ucs,
                    seventh=-1,
                    config=config,
                    speed=config["UI"]["robotSpeed"],
                    wait=False,
                )

                waitForBlending(cps=cps, config=config)
                communicate(
                    cps=cps,
                    tcp=config["coords"]["tcpDefault"],
                    ucs=config["coords"]["ucsDefault"],
                    seventh=roboPos,
                    config=config,
                    speed=config["UI"]["robotSpeed"],
                )
                frameCycle(cps=cps, framePoints=framePoints, config=config)
                communicate(
                    cps=cps,
                    point=origin_point,
                    tcp=origin_tcp,
                    ucs=origin_ucs,
                    seventh=-1,
                    config=config,
                    speed=config["UI"]["robotSpeed"],
                    wait=False,
                )
                msg_to_frontend(
                    api_url=config["server"]["frontEnd_messaging_url"],
                    message=f"Frame Cycle Completed for Door {tblCnt + 1}/{len(enum7thPos)}!",
                )

        if doInEdgeCycle:
            enum7thPos = list(reversed(enum7thPos))
            origin_point = config["point"]["weirdToolTable1Origin"]
            origin_ucs = config["coords"]["ucsTable1"]
            origin_tcp = config["coords"]["tcpEdgeToolIn"]

            for tblCnt, roboPos in enum7thPos:
                msg_to_frontend(
                    api_url=config["server"]["frontEnd_messaging_url"],
                    message=f"In Edge Cycle Started for Door {tblCnt + 1}/{len(enum7thPos)}...",
                )
                communicate(
                    cps=cps,
                    point=origin_point,
                    tcp=origin_tcp,
                    ucs=origin_ucs,
                    seventh=-1,
                    config=config,
                    speed=config["UI"]["robotSpeed"],
                    wait=False,
                )

                waitForBlending(cps=cps, config=config)
                communicate(
                    cps=cps,
                    tcp=config["coords"]["tcpDefault"],
                    ucs=config["coords"]["ucsDefault"],
                    seventh=roboPos,
                    config=config,
                    speed=config["UI"]["robotSpeed"],
                )
                inEdgeCycle(cps=cps, pocketPoints=innerCornerPoints, config=config)
                communicate(
                    cps=cps,
                    point=origin_point,
                    tcp=origin_tcp,
                    ucs=origin_ucs,
                    seventh=-1,
                    config=config,
                    speed=config["UI"]["robotSpeed"],
                    wait=False,
                )
                msg_to_frontend(
                    api_url=config["server"]["frontEnd_messaging_url"],
                    message=f"In Edge Cycle Completed for Door {tblCnt + 1}/{len(enum7thPos)}!",
                )

        if doPocketZigCycle:
            enum7thPos = list(reversed(enum7thPos))
            origin_point = config["point"]["table1Origin"]
            origin_ucs = config["coords"]["ucsTable1"]
            origin_tcp = config["coords"]["tcpPocketTool"]

            for tblCnt, roboPos in enum7thPos:
                msg_to_frontend(
                    api_url=config["server"]["frontEnd_messaging_url"],
                    message=f"Pocket Zig-Zag Cycle Started for Door {tblCnt + 1}/{len(enum7thPos)}...",
                )
                communicate(
                    cps=cps,
                    point=origin_point,
                    tcp=origin_tcp,
                    ucs=origin_ucs,
                    seventh=-1,
                    config=config,
                    speed=config["UI"]["robotSpeed"],
                    wait=False,
                )

                waitForBlending(cps=cps, config=config)
                communicate(
                    cps=cps,
                    tcp=config["coords"]["tcpDefault"],
                    ucs=config["coords"]["ucsDefault"],
                    seventh=roboPos,
                    config=config,
                    speed=config["UI"]["robotSpeed"],
                )
                pocketZigZagCycle(
                    cps=cps,
                    yVal=yVals[tblCnt],
                    pocketPoints=pocketPoints[tblCnt],
                    config=config,
                )
                communicate(
                    cps=cps,
                    point=origin_point,
                    tcp=origin_tcp,
                    ucs=origin_ucs,
                    seventh=-1,
                    config=config,
                    speed=config["UI"]["robotSpeed"],
                    wait=False,
                )
                msg_to_frontend(
                    api_url=config["server"]["frontEnd_messaging_url"],
                    message=f"Pocket Zig-Zag Cycle Completed for Door {tblCnt + 1}/{len(enum7thPos)}!",
                )

        if doPocketSqCycle:
            enum7thPos = list(reversed(enum7thPos))
            origin_point = config["point"]["table1Origin"]
            origin_ucs = config["coords"]["ucsTable1"]
            origin_tcp = config["coords"]["tcpPocketTool"]

            for tblCnt, roboPos in enum7thPos:
                msg_to_frontend(
                    api_url=config["server"]["frontEnd_messaging_url"],
                    message=f"Pocket Square Cycle Started for Door {tblCnt + 1}/{len(enum7thPos)}...",
                )
                communicate(
                    cps=cps,
                    point=origin_point,
                    tcp=origin_tcp,
                    ucs=origin_ucs,
                    seventh=-1,
                    config=config,
                    speed=config["UI"]["robotSpeed"],
                    wait=False,
                )

                waitForBlending(cps=cps, config=config)
                communicate(
                    cps=cps,
                    tcp=config["coords"]["tcpDefault"],
                    ucs=config["coords"]["ucsDefault"],
                    seventh=roboPos,
                    config=config,
                    speed=config["UI"]["robotSpeed"],
                )
                pocketSQCycle(cps=cps, pocketPoints=pocketPoints, config=config)
                communicate(
                    cps=cps,
                    point=origin_point,
                    tcp=origin_tcp,
                    ucs=origin_ucs,
                    seventh=-1,
                    config=config,
                    speed=config["UI"]["robotSpeed"],
                    wait=False,
                )
                msg_to_frontend(
                    api_url=config["server"]["frontEnd_messaging_url"],
                    message=f"Pocket Square Cycle Completed for Door {tblCnt + 1}/{len(enum7thPos)}!",
                )

        # if square tool is picked, keep the tool (since it will not be used after this)
        if (
            doFrameCycle
            or doOutEdgeCycle
            or doInEdgeCycle
            or doPocketSqCycle
            or doPocketZigCycle
        ):
            keepTool(
                cps,
                toolNumber=config["tool"]["frameToolPos"],
                config=config,
                goToSafe=not do3DCycle,
            )

        #########################################################
        ########### Tool Pick # 3: Pick the 3D tool ###########
        if do3DCycle:
            if not getTool(
                cps,
                toolNumber=config["tool"]["3dToolPos"],
                config=config,
                startFromSafe=not (
                    doFrameCycle
                    or doOutEdgeCycle
                    or doInEdgeCycle
                    or doPocketSqCycle
                    or doPocketZigCycle
                    or doSideCycle
                    or doOutEdgeCycle
                ),
            ):
                return

        if do3DCycle:
            enum7thPos = list(reversed(enum7thPos))
            origin_point = config["point"]["table1Origin"]
            origin_ucs = config["coords"]["ucsTable1"]
            origin_tcp = config["coords"]["tcpPocketTool"]

            for tblCnt, roboPos in enum7thPos:
                msg_to_frontend(
                    api_url=config["server"]["frontEnd_messaging_url"],
                    message=f"3D Cycle Started for Door {tblCnt + 1}/{len(enum7thPos)}...",
                )
                communicate(
                    cps=cps,
                    point=origin_point,
                    tcp=origin_tcp,
                    ucs=origin_ucs,
                    seventh=-1,
                    config=config,
                    speed=config["UI"]["robotSpeed"],
                    wait=False,
                )

                waitForBlending(cps=cps, config=config)
                communicate(
                    cps=cps,
                    tcp=config["coords"]["tcpDefault"],
                    ucs=config["coords"]["ucsDefault"],
                    seventh=roboPos,
                    config=config,
                    speed=config["UI"]["robotSpeed"],
                )
                threeDCycle(cps=cps, pocketPoints=pocketPoints, config=config)
                communicate(
                    cps=cps,
                    point=origin_point,
                    tcp=origin_tcp,
                    ucs=origin_ucs,
                    seventh=-1,
                    config=config,
                    speed=config["UI"]["robotSpeed"],
                    wait=False,
                )
                msg_to_frontend(
                    api_url=config["server"]["frontEnd_messaging_url"],
                    message=f"3D Cycle Completed for Door {tblCnt + 1}/{len(enum7thPos)}!",
                )

        if do3DCycle:
            keepTool(cps, toolNumber=config["tool"]["3dToolPos"], config=config)

        communicate(
            cps=cps,
            point=config["point"]["safePoint"],
            tcp=config["coords"]["tcpDefault"],
            ucs=config["coords"]["ucsDefault"],
            seventh=-1,
            config=config,
            speed=config["UI"]["robotSpeed"],
        )

        # break

    ##########################################
    ######At the beginning, do homing!!#######
    ##########################################
    scan_results = ()
    if homingState:
        homingFunction(cps=cps, config=config)
    if scan:
        scan_results = scan_table(cps=cps, config=config)
    if startSanding:
        (
            own7thpos,
            framePoints,
            pocketPoints,
            outerCornerPoints,
            innerCornerPoints,
            xVals,
            yVals,
        ) = scan_results
        sand_table(
            cps,
            config,
            own7thpos,
            framePoints,
            pocketPoints,
            outerCornerPoints,
            innerCornerPoints,
            xVals=xVals,
            yVals=yVals,
        )
    return scan_results

    # if startSanding:
    #     # homingFunction(cps=cps, config=config)
    #     # own7thpos, framePoints, pocketPoints, outerCornerPoints, innerCornerPoints, xVals, yVals
    #     robo7thPos, framePoints, pocketPoints, outerCornerPoints, innerCornerPoints, xVals, yVals = scan_table(cps=cps, config=config)
    #     sand_table(cps, config, robo7thPos, framePoints, pocketPoints, outerCornerPoints, innerCornerPoints, xVals=xVals, yVals=yVals)

    #     # communicate(cps=cps, point=config['point']['safePointTool'], tcp=config['coords']['tcpDefault'], ucs=config['coords']['ucsDefault'], seventh=0, config=config, speed=config['UI']['robotSpeed'])
    #     homingFunction(cps=cps, config=config)
    #     # control_table(cps, tableState="close")


def communicate(
    cps,
    config,
    tcp,
    ucs,
    point=None,
    seventh=-1,
    doMeasure=0,
    speed=None,
    stopWhenNan=False,
    stopWhenNanMinDist=None,
    wait=None,
    speed_mode="override",
    require_seventh_ok=False,
    velocity_profile=None,
):
    if stop_requested():
        return []
    # Cache active TCP/UCS inside the process config to avoid repeated
    # coordinate re-application on every nearby point transition.
    coord_cache = None
    if isinstance(config, dict):
        coord_cache = config.setdefault("_coord_cache", {})

    def euclidean_distance(point1, point2):
        """
        Calculate the Euclidean distance between two points in 3D space.

        Args:
            point1 (list or tuple): Coordinates of the first point [x1, y1, z1].
            point2 (list or tuple): Coordinates of the second point [x2, y2, z2].

        Returns:
            float: Euclidean distance between the two points.
        """
        if len(point1) != 3 or len(point2) != 3:
            raise ValueError(
                "Both points must have exactly three coordinates (x, y, z)."
            )

        distance = math.sqrt(
            (point2[0] - point1[0]) ** 2
            + (point2[1] - point1[1]) ** 2
            + (point2[2] - point1[2]) ** 2
        )

        return distance

    def resolve_velocity_profile(profile_hint):
        if not isinstance(profile_hint, str):
            return None
        normalized = profile_hint.strip().lower().replace("_", "").replace(" ", "")
        if normalized in (
            "sanding",
            "sand",
            "sandingvelocity",
            "sandvelocity",
            "sandingspeed",
        ):
            return "sanding"
        if normalized in (
            "robot",
            "robo",
            "robotvelocity",
            "robovelocity",
            "robotspeed",
        ):
            return "robot"
        return None

    def customMoveL(
        cps,
        point,
        tcp,
        ucs,
        config,
        speed,
        profile="robot",
        wait=True,
        ratio_for_log=None,
    ):
        RawACSpoints = [0, 0, 0, 0, 0, 0]  # Define the tool coordinate variable

        nIsSeek = 0  # Define The DI index of the detection
        nIOBit = 0  # Define The DI status of the detected state www.hansrobot.com 164Interface Description
        nIOState = 0  # Defining Road Point ID
        stdCmdID = "0"  # Perform road point movement

        need_apply_coords = True
        if isinstance(coord_cache, dict):
            same_coords = (
                coord_cache.get("tcp") == tcp and coord_cache.get("ucs") == ucs
            )
            verify_budget = int(coord_cache.get("verify_budget", 0) or 0)
            # Re-verify occasionally for safety, but skip repeated checks most of the time.
            if same_coords and verify_budget > 0:
                need_apply_coords = False
                coord_cache["verify_budget"] = verify_budget - 1

        if need_apply_coords:
            if not setUCS_TCP(cps=cps, tcp=tcp, ucs=ucs, config=config):
                return
            if isinstance(coord_cache, dict):
                coord_cache["tcp"] = tcp
                coord_cache["ucs"] = ucs
                coord_cache["verify_budget"] = 25

        # input("Is this fine?")

        robot_velocity = config["coords"]["roboVelocity"]
        robot_acceleration = config["coords"]["roboAcceleration"]
        sanding_velocity = config["coords"]["sandingVelocity"]
        sanding_acceleration = config["coords"]["sandingAcceleration"]
        
        if profile == "sanding":
            base_velocity = sanding_velocity
            base_acceleration = sanding_acceleration
        else:
            base_velocity = robot_velocity
            base_acceleration = robot_acceleration

        velocity = base_velocity
        acceleration = base_acceleration

        if speed is not None:
            try:
                speed_value = float(speed)
            except (TypeError, ValueError):
                speed_value = None

            if speed_value is not None:
                # Ratio model: [0..1], where 1.0 means 100% of profile max.
                if speed_value < 0.0:
                    speed_value = 0.0
                elif speed_value > 1.0:
                    speed_value = 1.0

                if speed_value <= 0.0:
                    config["logger"].warning(
                        "[customMoveL] speed ratio is 0.0; skipping MoveL to %s", point
                    )
                    return

                velocity = speed_value * base_velocity
                # Keep acceleration fixed per profile; only velocity scales with ratio.
                acceleration = base_acceleration

        if stop_requested():
            return
        config["logger"].info(
            "[customMoveL] profile=%s ratio=%s base_v=%.3f base_a=%.3f cmd_v=%.3f cmd_a=%.3f",
            profile,
            "None" if ratio_for_log is None else f"{float(ratio_for_log):.3f}",
            float(base_velocity),
            float(base_acceleration),
            float(velocity),
            float(acceleration),
        )
        planned_distance_mm = None
        try:
            act_pos = []
            nret_pos = cps.HRIF_ReadActPos(0, 0, act_pos)
            if (
                nret_pos == 0
                and isinstance(act_pos, (list, tuple))
                and len(act_pos) > 8
                and isinstance(point, (list, tuple))
                and len(point) > 2
            ):
                start_xyz = [float(act_pos[6]), float(act_pos[7]), float(act_pos[8])]
                target_xyz = [float(point[0]), float(point[1]), float(point[2])]
                planned_distance_mm = euclidean_distance(start_xyz, target_xyz)
        except Exception:
            planned_distance_mm = None
        nRet = cps.HRIF_MoveL(
            0,
            0,
            point,
            RawACSpoints,
            tcp,
            ucs,
            velocity,
            acceleration,
            config["coords"]["transitionRadius"],
            nIsSeek,
            nIOBit,
            nIOState,
            stdCmdID,
        )
        # time.sleep(0.2)
        print(nRet)
        # config['logger'].info(f"[moveL] nret for moving: {nRet}")
        if nRet == 0:
            robotRes = []
            move_wait_timeout_s = 12.0
            try:
                base_timeout_s = max(
                    1.0,
                    float(
                        config.get("door", {}).get(
                            "moveLWaitTimeoutSec", move_wait_timeout_s
                        )
                    ),
                )
                timeout_scale = float(
                    config.get("door", {}).get("moveLTimeoutDistanceScale", 1.8)
                )
                timeout_overhead_s = float(
                    config.get("door", {}).get("moveLTimeoutOverheadSec", 2.0)
                )
                if timeout_scale < 1.0:
                    timeout_scale = 1.0
                if timeout_overhead_s < 0.0:
                    timeout_overhead_s = 0.0

                dynamic_timeout_s = base_timeout_s
                if (
                    planned_distance_mm is not None
                    and planned_distance_mm > 0.0
                    and velocity is not None
                    and float(velocity) > 0.0
                ):
                    est_travel_s = planned_distance_mm / float(velocity)
                    dynamic_timeout_s = (est_travel_s * timeout_scale) + timeout_overhead_s

                # Sanding paths are long and should not fall through early.
                sanding_min_timeout_s = float(
                    config.get("door", {}).get("sandingMoveLMinTimeoutSec", 60.0)
                )
                if profile == "sanding":
                    move_wait_timeout_s = max(
                        base_timeout_s, dynamic_timeout_s, sanding_min_timeout_s
                    )
                else:
                    move_wait_timeout_s = max(base_timeout_s, dynamic_timeout_s)

                config["logger"].info(
                    "[customMoveL] wait timeout set to %.1fs (profile=%s, dist_mm=%s, cmd_v=%.3f)",
                    move_wait_timeout_s,
                    profile,
                    "None"
                    if planned_distance_mm is None
                    else f"{float(planned_distance_mm):.1f}",
                    float(velocity),
                )
            except (TypeError, ValueError, AttributeError):
                move_wait_timeout_s = 12.0
            wait_start = time.time()
            while wait:
                nret = cps.HRIF_ReadRobotState(0, 0, robotRes)
                if stop_requested():
                    break
                # config['logger'].info(robotRes)

                #---- SAFETY CHECK ----
                if robotRes is None:
                    raise ValueError("customMoveL: robotRes is None")
                if not isinstance(robotRes, (list, tuple)):
                    raise TypeError(f"customMoveL: robotRes is not list/tuple: {robotRes}")
                if len(robotRes)<= 11:
                    raise IndexError(f"customMoveL: robotRes too short ({len(robotRes)}): {robotRes}")
                
                if robotRes[11] == "1":
                    break
                if (time.time() - wait_start) >= move_wait_timeout_s:
                    config["logger"].warning(
                        "[customMoveL] wait timeout after %.1fs at point %s; continuing.",
                        move_wait_timeout_s,
                        point,
                    )
                    break
                # time.sleep(0.2)
            config["logger"].info(f"\n[customMoveL] Could move to {point}")
        else:
            config["logger"].error(f"\n[customMoveL] Couldn't move to {point}")
            msg_to_frontend(
                api_url=config["server"]["frontEnd_messaging_url"],
                message="Error In Moving! Please Check Door Size/Robot Settings and Try Again. Terminating Process...",
            )
            exit(-1)

    def data_collection(cps, config, stopWhenNan=False, stopWhenNanMinDist=None):
        measurements = []
        instrument = getInstrument()
        result = []
        nRet = cps.HRIF_ReadActPos(0, 0, result)
        initPos = [float(result[6]), float(result[7]), float(result[8])]
        keepgoing = True
        nan_count = 0
        # NaN stop tuning: stop only after we already saw valid data,
        # then encountered a sustained NaN tail (end-of-door).
        scan_stop_cfg = config.get("settings", {}) if isinstance(config, dict) else {}
        consecutive_nan_threshold = int(scan_stop_cfg.get("scanStopConsecutiveNan", 5))
        min_valid_samples_before_nan_stop = int(
            scan_stop_cfg.get("scanStopMinValidSamples", 10)
        )
        nan_tail_travel_mm = float(scan_stop_cfg.get("scanStopNanTailMm", 10.0))
        startStopNan = False
        valid_sample_count = 0
        last_valid_raw_dist = None
        first_valid_raw_dist = None
        msg_to_frontend(
            api_url=config["server"]["frontEnd_messaging_url"],
            message=f"Scanning Door Using the Laser Sensor...⚙️",
        )
        read_error_count = 0
        max_read_errors_before_abort = 80
        try:
            while keepgoing:
                try:
                    height = getRawHeight(instrument)
                    read_error_count = 0
                except Exception as sensor_exc:
                    # Transient serial/CRC issues can appear during repeated scans.
                    # Keep scanning and treat the sample as NaN unless errors persist.
                    read_error_count += 1
                    config["logger"].warning(
                        "[scan-x] Laser read failed (%s/%s): %s",
                        read_error_count,
                        max_read_errors_before_abort,
                        sensor_exc,
                    )
                    height = float("nan")
                    try:
                        if hasattr(instrument, "serial") and instrument.serial:
                            instrument.serial.reset_input_buffer()
                            instrument.serial.reset_output_buffer()
                    except Exception:
                        pass
                    if read_error_count >= max_read_errors_before_abort:
                        raise RuntimeError(
                            "Laser sensor read failed repeatedly; aborting scan."
                        ) from sensor_exc

                # Read position and compute distance
                cps.HRIF_ReadActPos(0, 0, result)
                pos = [float(result[6]), float(result[7]), float(result[8])]
                # if config['settings']['debug']:
                #     config['logger'].info(f"pos: {pos}, height: {height}                       ")

                if not math.isnan(height):
                    startStopNan = True
                    valid_sample_count += 1
                raw_dist = euclidean_distance(point1=initPos, point2=pos)
                if not math.isnan(height) and first_valid_raw_dist is None:
                    first_valid_raw_dist = float(raw_dist)
                # Calculate the distance from initial position
                dist = raw_dist

                # Append measurement
                measurements.append(
                    {"height": scale_value(height), "dist": dist, "raw_dist": raw_dist}
                )

                nan_stop_gate_ok = True
                if stopWhenNanMinDist is not None:
                    try:
                        nan_stop_gate_ok = float(raw_dist) >= float(stopWhenNanMinDist)
                    except (TypeError, ValueError):
                        nan_stop_gate_ok = True

                if stopWhenNan and startStopNan and nan_stop_gate_ok:
                    # Check the height for NaN and update nan_count
                    if math.isnan(height):
                        nan_count += 1  # Increment the NaN counter
                    else:
                        nan_count = 0  # Reset the NaN counter if height is valid
                        last_valid_raw_dist = float(raw_dist)

                    # Check if NaN count reaches the threshold
                    enough_valid_samples = (
                        valid_sample_count >= min_valid_samples_before_nan_stop
                    )
                    nan_tail_ok = False
                    if last_valid_raw_dist is not None:
                        nan_tail_ok = (float(raw_dist) - float(last_valid_raw_dist)) >= float(
                            nan_tail_travel_mm
                        )
                    if nan_count >= consecutive_nan_threshold and enough_valid_samples and nan_tail_ok:
                        config["logger"].info(
                            "[scan-stop] stopWhenNan triggered after valid data tail (valid_samples=%s, nan_count=%s, tail_mm=%.2f)",
                            valid_sample_count,
                            nan_count,
                            float(raw_dist) - float(last_valid_raw_dist)
                            if last_valid_raw_dist is not None
                            else 0.0,
                        )
                        keepgoing = False
                        nRet = cps.HRIF_GrpStop(0, 0)

                # Read robot state and stop only when motion is actually done.
                cps.HRIF_ReadRobotState(0, 0, result)
                if isinstance(result, (list, tuple)) and len(result) > 11:
                    if str(result[11]).strip() == "1":
                        keepgoing = False
                elif isinstance(result, (list, tuple)) and len(result) > 0:
                    # Fallback for unexpected payload shape.
                    if str(result[0]).strip() == "0":
                        keepgoing = False
        finally:
            try:
                if hasattr(instrument, "serial") and instrument.serial:
                    instrument.serial.close()
            except Exception:
                pass

        if first_valid_raw_dist is not None:
            for measurement in measurements:
                raw_dist = float(measurement.get("raw_dist", measurement.get("dist", 0.0)))
                measurement["dist"] = raw_dist - float(first_valid_raw_dist)
            config["logger"].info(
                "[scan-data] Normalized scan distances to first valid surface hit: origin_raw_dist=%.3fmm samples=%s",
                float(first_valid_raw_dist),
                len(measurements),
            )

        # config['logger'].info("\n\n")

        return measurements

    def waitWhileMovingRobot(cps, keepgoing=True):
        # keep going?
        # keepgoing = True
        result = []
        while keepgoing:
            cps.HRIF_ReadRobotState(0, 0, result)
            # config['logger'].info(f"Here! waiting now {result}")
            if result[11] == "1":
                keepgoing = False

    def seventhGoToPos(cps, position, speed, config, wait=True):
        def move_ok(nret, result):
            if nret is None:
                return bool(result) and str(result[0]) == "0"
            return nret == 0

        def read_motion_state(state):
            if not isinstance(state, (list, tuple)) or len(state) < 3:
                return None
            return str(state[2]).strip().lower()

        def state_readable(state):
            return read_motion_state(state) is not None

        def state_done(state):
            motion = read_motion_state(state)
            if motion is None:
                return False
            if motion in ("idle", "done", "false", "stop", "stopped"):
                return True
            try:
                return float(motion) == 0.0
            except (TypeError, ValueError):
                return False

        def _safe_float(value):
            try:
                return float(value)
            except (TypeError, ValueError):
                return None

        def _extract_pos_from_state(state):
            """
            Best-effort J7 position extraction from MotorGetState payload.
            Different firmware variants return position in different slots.
            """
            if not isinstance(state, (list, tuple)):
                return None
            # Common compact response is ['0', 'OK', <motion>, <motion_detail>]
            # and does not include physical position. Treat those as unavailable.
            if len(state) <= 4:
                return None
            # Only inspect fields beyond index 3 to avoid confusing motion-state
            # flags (often '0'/'1') with a position value.
            candidates = list(reversed(state[4:]))
            for candidate in candidates:
                pos = _safe_float(candidate)
                if pos is not None:
                    return pos
            return None

        def stop_j7():
            try:
                cps.HRIF_HRApp(0, "HR_Motor", "MotorStop", ["J7"], [])
            except Exception:
                pass

        j7_trace = []
        j7_last_logged_motion = None
        j7_trace_enabled = bool(
            isinstance(config, dict)
            and (
                config.get("settings", {}).get("debug", False)
                or config.get("door", {}).get("traceSeventhAxis", False)
            )
        )

        def trace_event(label, state=None, **extra):
            if not j7_trace_enabled:
                return
            entry = {"t": round(time.time(), 3), "label": label}
            if state is not None:
                entry["state"] = list(state) if isinstance(state, (list, tuple)) else state
            for key, value in extra.items():
                entry[key] = value
            if len(j7_trace) >= 120:
                j7_trace.pop(0)
            j7_trace.append(entry)

        def fail(log_message, frontend_message, stop_motor=False):
            if stop_motor:
                stop_j7()
            if j7_trace_enabled:
                log_message = f"{log_message} trace={j7_trace}"
            config["logger"].error(log_message)
            msg_to_frontend(
                api_url=config["server"]["frontEnd_messaging_url"],
                message=frontend_message,
            )
            return False

        pluginRes = []
        result = []
        door_cfg = config.get("door", {}) if isinstance(config, dict) else {}
        move_poll_s = max(0.005, float(door_cfg.get("seventhAxisPollIntervalSec", 0.02)))
        move_wait_timeout_s = max(
            5.0, float(door_cfg.get("seventhAxisMoveTimeoutSec", 45.0))
        )
        # Since MotorGetState only reports -1/0/1 (unknown/stopped/running),
        # we need a motion-state transition check to avoid false "arrived".
        require_run_transition = bool(
            door_cfg.get("seventhAxisRequireRunTransition", True)
        )
        run_transition_grace_s = max(
            0.05, float(door_cfg.get("seventhAxisRunTransitionGraceSec", 0.35))
        )
        min_delta_for_transition_mm = max(
            0.0, float(door_cfg.get("seventhAxisTransitionDeltaMm", 20.0))
        )
        # Require a stable stopped-state window before accepting move completion.
        settle_window_s = max(
            0.05, float(door_cfg.get("seventhAxisSettleWindowSec", 0.8))
        )
        settle_stable_samples = max(
            2, int(door_cfg.get("seventhAxisSettleStableSamples", 6))
        )

        # Read state first; only reconnect J7 if state read is unavailable.
        nret = cps.HRIF_HRApp(0, "HR_Motor", "MotorGetState", ["J7"], pluginRes)
        trace_event("initial_state_read", pluginRes, nret=nret, target=position, wait=wait)
        if (nret not in (0, None)) or not state_readable(pluginRes):
            result = []
            nret_conn = cps.HRIF_HRApp(0, "HR_Motor", "MotorConnect", ["J7"], result)
            if not move_ok(nret_conn, result):
                return fail(
                    f"[7thAxisMove] MotorConnect failed (ret={nret_conn}, res={result}).",
                    "7th axis connection failed. Please verify J7 and try again.",
                )
            # Short readiness poll (faster than fixed 0.2s x N sleeps).
            ready = False
            for _ in range(8):
                pluginRes = []
                nret = cps.HRIF_HRApp(0, "HR_Motor", "MotorGetState", ["J7"], pluginRes)
                if (nret in (0, None)) and state_readable(pluginRes):
                    ready = True
                    break
                time.sleep(0.02)
            if not ready:
                return fail(
                    f"[7thAxisMove] Failed to read J7 state before move (ret={nret}, res={pluginRes}).",
                    "7th axis state read failed. Please verify J7 connection and try again.",
                )

        if not state_readable(pluginRes):
            return fail(
                f"[7thAxisMove] Failed to read J7 state before move (ret={nret}, res={pluginRes}).",
                "7th axis state read failed. Please verify J7 connection and try again.",
            )

        max_move_retries = 8
        nret = None
        result = []
        for move_attempt in range(1, max_move_retries + 1):
            result = []
            nret = cps.HRIF_HRApp(
                0, "HR_Motor", "MotorMovePositionSpeed", ["J7", position, speed], result
            )
            if move_ok(nret, result):
                break

            result_text = " ".join(str(item) for item in result).lower()
            motor_not_ready = str(result[0]) == "60006" if result else False
            motor_not_ready = motor_not_ready or ("not ready" in result_text)
            if not motor_not_ready:
                break

            config["logger"].warning(
                "[7thAxisMove] J7 not ready on move attempt %s/%s (ret=%s, res=%s). Retrying...",
                move_attempt,
                max_move_retries,
                nret,
                result,
            )
            cps.HRIF_HRApp(0, "HR_Motor", "MotorConnect", ["J7"], [])
            time.sleep(0.08 * move_attempt)

        if not move_ok(nret, result):
            return fail(
                f"[7thAxisMove] MotorMovePositionSpeed failed (ret={nret}, res={result}).",
                "7th axis move failed. Please verify J7 and try again.",
            )

        config["logger"].info(
            f"[7thAxisMove] Going to position: {position}mm with speed: {speed}mm/s"
        )
        trace_event("move_command_accepted", result, target=position, speed=speed)
        if isinstance(config, dict):
            # Track the last accepted command target. This is not a proof of arrival.
            config["_last_j7_command"] = float(position)

        if not wait:
            config["logger"].info(
                "[7thAxisMove] Command accepted in async mode: target=%.3fmm",
                float(position),
            )
            return True

        wait_start = time.time()
        consecutive_state_errors = 0
        saw_running = False
        while True:
            if stop_requested():
                stop_j7()
                if isinstance(config, dict) and config.get("logger"):
                    config["logger"].warning(
                        "[7thAxisMove] Stop requested while waiting for J7 move. diag=%s",
                        _stop_diagnostics(),
                    )
                return False

            pluginRes = []
            nret = cps.HRIF_HRApp(0, "HR_Motor", "MotorGetState", ["J7"], pluginRes)
            if (nret not in (0, None)) or not state_readable(pluginRes):
                consecutive_state_errors += 1
                trace_event(
                    "state_read_error",
                    pluginRes,
                    nret=nret,
                    consecutive_errors=consecutive_state_errors,
                )
                if consecutive_state_errors >= 5:
                    return fail(
                        f"[7thAxisMove] MotorGetState failed during move (ret={nret}, res={pluginRes}).",
                        "7th axis state read failed during move. Movement stopped for safety.",
                        stop_motor=True,
                    )
            else:
                consecutive_state_errors = 0
                motion_state = read_motion_state(pluginRes)
                if motion_state != j7_last_logged_motion:
                    trace_event(
                        "motion_state_change",
                        pluginRes,
                        motion_state=motion_state,
                        elapsed=round(time.time() - wait_start, 3),
                    )
                    j7_last_logged_motion = motion_state
                if motion_state in ("1", "true", "running"):
                    saw_running = True

                if state_done(pluginRes):
                    trace_event(
                        "state_done_seen",
                        pluginRes,
                        elapsed=round(time.time() - wait_start, 3),
                        saw_running=saw_running,
                    )
                    if require_run_transition and (not saw_running):
                        elapsed = time.time() - wait_start
                        if elapsed < run_transition_grace_s:
                            time.sleep(move_poll_s)
                            continue
                        config["logger"].warning(
                            "[7thAxisMove] Move reported done without RUNNING transition; "
                            "accepting stopped state after grace window (target=%.3f, state=%s).",
                            float(position),
                            pluginRes,
                        )

                    stable_done = True
                    stable_samples = 0
                    settle_deadline = time.time() + settle_window_s
                    trace_event(
                        "settle_window_enter",
                        pluginRes,
                        settle_window_s=settle_window_s,
                        settle_stable_samples=settle_stable_samples,
                    )
                    while time.time() < settle_deadline:
                        probe = []
                        nret_probe = cps.HRIF_HRApp(
                            0, "HR_Motor", "MotorGetState", ["J7"], probe
                        )
                        if (nret_probe not in (0, None)) or not state_readable(probe):
                            stable_done = False
                            trace_event(
                                "settle_probe_error",
                                probe,
                                nret=nret_probe,
                                stable_samples=stable_samples,
                            )
                            break

                        probe_motion = read_motion_state(probe)
                        if probe_motion in ("1", "true", "running"):
                            stable_done = False
                            trace_event(
                                "settle_probe_running",
                                probe,
                                stable_samples=stable_samples,
                            )
                            config["logger"].warning(
                                "[7thAxisMove] J7 left STOPPED state during settle window; continuing wait. probe=%s",
                                probe,
                            )
                            break

                        if state_done(probe):
                            stable_samples += 1
                            trace_event(
                                "settle_probe_done",
                                probe,
                                stable_samples=stable_samples,
                            )
                            if stable_samples >= settle_stable_samples:
                                break
                        time.sleep(move_poll_s)

                    if stable_done and stable_samples >= settle_stable_samples:
                        trace_event(
                            "settle_accept_done",
                            pluginRes,
                            stable_samples=stable_samples,
                            elapsed=round(time.time() - wait_start, 3),
                        )
                        break

            if (time.time() - wait_start) >= move_wait_timeout_s:
                return fail(
                    f"[7thAxisMove] Timeout waiting for J7 move to complete after "
                    f"{move_wait_timeout_s:.1f}s (target={position}, last_state={pluginRes}).",
                    "7th axis move timeout. Movement stopped for safety.",
                    stop_motor=True,
                )

            time.sleep(move_poll_s)

        # Optional post-move position verification if controller reports a
        # usable position value in MotorGetState payload.
        verify_cfg = config.get("door", {}) if isinstance(config, dict) else {}
        verify_tol_mm = _safe_float(verify_cfg.get("seventhAxisPosToleranceMm"))
        if verify_tol_mm is None or verify_tol_mm <= 0.0:
            verify_tol_mm = 8.0
        # Default OFF: many firmwares do not expose J7 physical position in
        # MotorGetState, which can cause false mismatches.
        verify_enabled = bool(verify_cfg.get("seventhAxisVerifyPosition", False))

        actual_pos = _extract_pos_from_state(pluginRes)
        if verify_enabled and actual_pos is not None:
            pos_err = abs(float(actual_pos) - float(position))
            if pos_err > float(verify_tol_mm):
                return fail(
                    "[7thAxisMove] Target reached-state but position mismatch: "
                    f"target={float(position):.3f}mm actual={float(actual_pos):.3f}mm "
                    f"err={float(pos_err):.3f}mm tol={float(verify_tol_mm):.3f}mm state={pluginRes}",
                    "7th axis reported done but position mismatch was detected. "
                    "Movement stopped for safety.",
                    stop_motor=True,
                )
            config["logger"].info(
                "[7thAxisMove] Reached position: target=%.3fmm actual=%.3fmm "
                "(err=%.3fmm, tol=%.3fmm)",
                float(position),
                float(actual_pos),
                float(pos_err),
                float(verify_tol_mm),
            )
        else:
            config["logger"].info(
                "[7thAxisMove] Reached position command: target=%.3fmm "
                "(actual unavailable; state=%s)",
                float(position),
                pluginRes,
            )
        if j7_trace_enabled:
            config["logger"].info(
                "[7thAxisMove] completion trace target=%.3f wait=%s trace=%s",
                float(position),
                wait,
                j7_trace,
            )
        return True

    time.sleep(0.0001)

    measurements = []
    # speed_mode="override": <=1 uses override, >1 scales MoveL velocity.
    # speed_mode="linear": always scales MoveL velocity, no override changes.
    selected_profile = resolve_velocity_profile(velocity_profile) or resolve_velocity_profile(
        speed
    )
    if selected_profile is None:
        selected_profile = "robot"

    parsed_speed = speed
    if isinstance(speed, str) and resolve_velocity_profile(speed) is not None:
        if selected_profile == "sanding":
            parsed_speed = config.get("UI", {}).get("sandSpeed")
        else:
            parsed_speed = config.get("UI", {}).get("robotSpeed")

    override_speed = None
    linear_speed = None
    if parsed_speed is not None:
        try:
            speed_value = float(parsed_speed)
        except (TypeError, ValueError):
            speed_value = None
        if speed_value is not None:
            # Ratio model: clamp to [0..1] so 1.0 equals profile max.
            if speed_value < 0.0:
                speed_value = 0.0
            elif speed_value > 1.0:
                speed_value = 1.0

            if speed_mode == "override":
                override_speed = speed_value
            else:
                linear_speed = speed_value

    # setting the speed of the client (ratio override only)
    # Avoid extra override calls when doing seventh-axis-only repositioning.
    if not (point is None and seventh != -1):
        if override_speed is not None:
            setSpeed(cps, override_speed, config=config)
        elif linear_speed is not None and speed_mode == "override":
            # If previous moves set a low override (e.g., 0.3), fast linear
            # moves (>1.0) can still feel slow unless we restore neutral override.
            setSpeed(cps, 1.0, config=config)
    # time.sleep(0.1)

    if seventh != -1:
        if stop_requested():
            return measurements
        if isinstance(config, dict) and config.get("logger"):
            try:
                config["logger"].info(
                    "[J7 Target] target=%.3f wait=%s require_ok=%s profile=%s speed_arg=%s tcp=%s ucs=%s",
                    float(seventh),
                    wait if wait is not None else bool(1 - doMeasure),
                    bool(require_seventh_ok),
                    selected_profile,
                    speed,
                    tcp,
                    ucs,
                )
            except Exception:
                config["logger"].info(
                    "[J7 Target] target=%s wait=%s require_ok=%s profile=%s speed_arg=%s tcp=%s ucs=%s",
                    seventh,
                    wait if wait is not None else bool(1 - doMeasure),
                    bool(require_seventh_ok),
                    selected_profile,
                    speed,
                    tcp,
                    ucs,
                )
        ok = seventhGoToPos(
            cps=cps,
            position=seventh,
            speed=config["UI"]["seventhAxisSpeed"]
            / 100
            * config["seventhAxis"]["maxSpeed"],
            config=config,
            wait=wait if wait is not None else bool(1 - doMeasure),
        )
        if not ok:
            return None if require_seventh_ok else measurements

    if point:
        if stop_requested():
            return measurements
        motion_ratio_log = (
            override_speed if override_speed is not None else linear_speed
        )
        customMoveL(
            cps,
            point=point,
            tcp=tcp,
            ucs=ucs,
            speed=linear_speed,
            profile=selected_profile,
            config=config,
            wait=wait if wait is not None else bool(1 - doMeasure),
            ratio_for_log=motion_ratio_log,
        )

    # the list to collect values in
    if doMeasure:
        measurements = data_collection(cps, config, stopWhenNan, stopWhenNanMinDist)
        # Ensure the robot is fully stopped before issuing the next command.
        waitWhileMovingRobot(cps)
    # else:
    #     waitWhileMovingRobot(cps)

    # pause script
    # time.sleep(1)
    # try:
    #     input("proceed to the next one?")
    # except EOFError:
    #     config['logger'].info("No input available, proceeding without user confirmation.")

    return measurements


# New integration by rafat dated on 28_12_2024
def toolValve1(cps, valveState: str, config):  # Tool valve for grabbing or throwing
    """Used to "pick" or "drop" the tool.
    Helpful: When DO5 is 1: the pneumatic is loose. When D05 is 0, pneumatic is tight.

    Args:
        cps (CPSClient): cps
        valveState (str): "pick" to grab the tool, "drop" to let the tool go.
        config (config): configuration file
    """
    if stop_requested():
        return
    pre_delay = _tool_valve_delay(config, "toolValvePreDelaySec", 0.15)
    post_delay = _tool_valve_delay(config, "toolValvePostDelaySec", 0.15)
    # Required dwell for pneumatic stability.
    time.sleep(pre_delay)
    if valveState == "drop":
        status = 1
        digOutput = 5  # DOnumber=0,1,2,3,4

        # while True:
        #     confirmation = input("Are you sure you want to DROP the tool? (yes/no): ").strip().lower()
        #     if confirmation == 'yes':
        #         break

        #     time.sleep(0.1)

        nRet = cps.HRIF_SetBoxDO(0, digOutput, status)
        if config["settings"]["debug"]:
            config["logger"].info(
                f"[toolValve] Tool is dropped! Success: {nRet} (0 means successful)"
            )

    elif valveState == "pick":
        status = 0
        digOutput = 5  # DOnumber=0,1,2,3,4
        nRet = cps.HRIF_SetBoxDO(0, digOutput, status)
        if config["settings"]["debug"]:
            config["logger"].info(
                f"[toolValve] Tool is dropped! Success: {nRet} (0 means successful)"
            )
    # Required dwell for pneumatic stability.
    msg_to_frontend(
        api_url=config["server"]["frontEnd_messaging_url"],
        message=f"Tool Set to '{valveState}'",
    )
    time.sleep(post_delay)


def getTool11(
    cps, toolNumber, config, startFromSafe=True, exitToSafe=True
):  # Tool postion dile nibe
    """Manual tool pick flow."""
    if not config["settings"]["useTool"]:
        return False

    pick_fast = 1.0
    pick_slow = _tool_speed(
        config, "manualPickSlowSpeed", config.get("UI", {}).get("sandSpeed")
    )
    pick_slow = _resolve_ui_ratio(config, "sandSpeed", pick_slow)
    config["logger"].info(
        "[toolMotion][manualPick] ratios: fast(robot)=%.3f slow(sanding)=%.3f",
        float(pick_fast),
        float(pick_slow),
    )

    # Keep override neutral so linear profile commands are not flattened.
    setSpeed(cps, 1.0, config=config)

    current_tool = _get_tool_in_hand(cps)
    if current_tool and current_tool > 0:
        msg_to_frontend(
            api_url=config["server"]["frontEnd_messaging_url"],
            message=f"Tool {current_tool} already in hand. Drop it before picking Tool {toolNumber}.",
        )
        return False

    msg_to_frontend(
        api_url=config["server"]["frontEnd_messaging_url"],
        message=f"Tool {toolNumber} Collection Started...",
    )

    if startFromSafe:
        config["logger"].info(
            "[toolMotion][manualPick] phase=pre_safe profile=robot ratio=%.3f",
            float(pick_fast),
        )
        communicate(
            cps=cps,
            point=config["point"]["safePointTool"],
            tcp=config["coords"]["tcpDefault"],
            ucs=config["coords"]["ucsDefault"],
            seventh=-1,
            config=config,
            speed=pick_fast,
            velocity_profile="robotspeed",
            speed_mode="linear",
            wait=False,
        )

    config["logger"].info(
        "[toolMotion][manualPick] phase=approach_home profile=robot ratio=%.3f",
        float(pick_fast),
    )
    communicate(
        cps=cps,
        point=config["point"][f"tool{toolNumber}home"],
        tcp=config["coords"]["tcpDefault"],
        ucs=config["coords"]["ucsDefault"],
        seventh=-1,
        config=config,
        speed=pick_fast,
        velocity_profile="robotspeed",
        speed_mode="linear",
        wait=True,
    )

    if not waitForBlending(cps=cps, config=config):
        msg_to_frontend(
            api_url=config["server"]["frontEnd_messaging_url"],
            message="Aborting pick: robot blending did not complete before valve open.",
        )
        raise RuntimeError("Blending timeout/error before tool pick pre-drop.")
    toolValve1(cps, valveState="drop", config=config)

    config["logger"].info(
        "[toolMotion][manualPick] phase=touch profile=sanding ratio=%.3f",
        float(pick_slow),
    )
    communicate(
        cps=cps,
        point=config["point"][f"tool{toolNumber}"],
        tcp=config["coords"]["tcpDefault"],
        ucs=config["coords"]["ucsDefault"],
        seventh=-1,
        config=config,
        speed=pick_slow,
        velocity_profile="sandingspeed",
        speed_mode="linear",
        wait=True,
    )

    if not waitForBlending(cps=cps, config=config):
        msg_to_frontend(
            api_url=config["server"]["frontEnd_messaging_url"],
            message="Aborting pick: robot blending did not complete before valve close.",
        )
        raise RuntimeError("Blending timeout/error before tool pick.")
    toolValve1(cps, valveState="pick", config=config)

    config["logger"].info(
        "[toolMotion][manualPick] phase=retract_home profile=sanding ratio=%.3f",
        float(pick_slow),
    )
    communicate(
        cps=cps,
        point=config["point"][f"tool{toolNumber}home"],
        tcp=config["coords"]["tcpDefault"],
        ucs=config["coords"]["ucsDefault"],
        seventh=-1,
        config=config,
        speed=pick_slow,
        velocity_profile="sandingspeed",
        speed_mode="linear",
        wait=True,
    )

    if exitToSafe:
        config["logger"].info(
            "[toolMotion][manualPick] phase=exit_safe profile=robot ratio=%.3f",
            float(pick_fast),
        )
        communicate(
            cps=cps,
            point=config["point"]["safePointTool"],
            tcp=config["coords"]["tcpDefault"],
            ucs=config["coords"]["ucsDefault"],
            seventh=-1,
            config=config,
            speed=pick_fast,
            velocity_profile="robotspeed",
            speed_mode="linear",
            wait=True,
        )

    # Verify attachment after retract path. Tool 2 can report CI transition late
    # right at tool home; re-check after safe exit before failing.
    if not _verify_tool_attached(cps, toolNumber, config):
        settle_s = float(config.get("tool", {}).get("sensorSettleSeconds", 0.4) or 0.4)
        if settle_s > 0:
            time.sleep(settle_s)
        if not _verify_tool_attached(cps, toolNumber, config):
            return False

    msg_to_frontend(
        api_url=config["server"]["frontEnd_messaging_url"],
        message=f"Tool {toolNumber} Collection Successful!",
    )
    return True


def keepTool11(
    cps, toolNumber, config, goToSafe=True, startFromSafe=True
):  # Tool Postion a rekhe dibe
    if not config["settings"]["useTool"]:
        return

    drop_fast = 1.0
    drop_slow = _tool_speed(
        config, "manualDropSlowSpeed", config.get("UI", {}).get("sandSpeed")
    )
    drop_slow = _resolve_ui_ratio(config, "sandSpeed", drop_slow)
    config["logger"].info(
        "[toolMotion][manualDrop] ratios: fast(robot)=%.3f slow(sanding)=%.3f",
        float(drop_fast),
        float(drop_slow),
    )

    # Keep override neutral so linear profile commands are not flattened.
    setSpeed(cps, 1.0, config=config)

    msg_to_frontend(
        api_url=config["server"]["frontEnd_messaging_url"],
        message=f"Tool {toolNumber} Keeping Started...",
    )

    if startFromSafe:
        config["logger"].info(
            "[toolMotion][manualDrop] phase=pre_safe profile=robot ratio=%.3f",
            float(drop_fast),
        )
        communicate(
            cps=cps,
            point=config["point"]["safePointTool"],
            tcp=config["coords"]["tcpDefault"],
            ucs=config["coords"]["ucsDefault"],
            seventh=-1,
            config=config,
            speed=drop_fast,
            velocity_profile="robotspeed",
            speed_mode="linear",
            wait=False,
        )

    config["logger"].info(
        "[toolMotion][manualDrop] phase=approach_home profile=robot ratio=%.3f",
        float(drop_fast),
    )
    communicate(
        cps=cps,
        point=config["point"][f"tool{toolNumber}home"],
        tcp=config["coords"]["tcpDefault"],
        ucs=config["coords"]["ucsDefault"],
        seventh=-1,
        config=config,
        speed=drop_fast,
        velocity_profile="robotspeed",
        speed_mode="linear",
        wait=True,
    )

    if not waitForBlending(cps=cps, config=config):
        # Same safety pattern used later in this drop flow:
        # if blending status is stale but robot is already idle, proceed.
        robot_state = []
        nret_state = cps.HRIF_ReadRobotState(0, 0, robot_state)
        robot_idle = (
            nret_state == 0
            and isinstance(robot_state, (list, tuple))
            and len(robot_state) > 11
            and str(robot_state[11]).strip() == "1"
        )
        if not robot_idle:
            msg_to_frontend(
                api_url=config["server"]["frontEnd_messaging_url"],
                message="Aborting drop: robot blending did not complete before touch move.",
            )
            raise RuntimeError("Blending timeout/error before tool drop touch move.")
        config["logger"].warning(
            "[toolMotion][manualDrop] Blending timeout before touch move, but robot is idle; proceeding."
        )

    config["logger"].info(
        "[toolMotion][manualDrop] phase=touch profile=sanding ratio=%.3f",
        float(drop_slow),
    )
    communicate(
        cps=cps,
        point=config["point"][f"tool{toolNumber}"],
        tcp=config["coords"]["tcpDefault"],
        ucs=config["coords"]["ucsDefault"],
        seventh=-1,
        config=config,
        speed=drop_slow,
        velocity_profile="sandingspeed",
        speed_mode="linear",
        wait=True,
    )

    if not waitForBlending(cps=cps, config=config):
        # Blending status can be stale on some controller states. If robot is
        # already idle, proceed with drop rather than hard-failing manual flow.
        robot_state = []
        nret_state = cps.HRIF_ReadRobotState(0, 0, robot_state)
        robot_idle = (
            nret_state == 0
            and isinstance(robot_state, (list, tuple))
            and len(robot_state) > 11
            and str(robot_state[11]).strip() == "1"
        )
        if not robot_idle:
            msg_to_frontend(
                api_url=config["server"]["frontEnd_messaging_url"],
                message="Aborting drop: robot blending did not complete before valve open.",
            )
            raise RuntimeError("Blending timeout/error before tool drop.")
        config["logger"].warning(
            "[toolMotion][manualDrop] Blending timeout before valve open, but robot is idle; proceeding with drop."
        )
    toolValve1(cps, valveState="drop", config=config)

    # Retract first, then verify release. After tool-point edits, checking
    # release exactly at touch depth can produce false negatives.
    config["logger"].info(
        "[toolMotion][manualDrop] phase=retract_home profile=sanding ratio=%.3f",
        float(drop_slow),
    )
    communicate(
        cps=cps,
        point=config["point"][f"tool{toolNumber}home"],
        tcp=config["coords"]["tcpDefault"],
        ucs=config["coords"]["ucsDefault"],
        seventh=-1,
        config=config,
        speed=drop_slow,
        velocity_profile="sandingspeed",
        speed_mode="linear",
        wait=True,
    )

    # After drop, tool must not remain attached.
    if _verify_tool_attached(cps, toolNumber, config):
        raise RuntimeError("Tool still detected after drop command.")
    if not _verify_tool_released(
        cps=cps, config=config, expected_tool_number=toolNumber
    ):
        # One short settle retry helps when CI/DI transitions lag slightly.
        time.sleep(0.25)
        if not _verify_tool_released(
            cps=cps, config=config, expected_tool_number=toolNumber
        ):
            raise RuntimeError("Tool release not confirmed after drop command.")

    msg_to_frontend(
        api_url=config["server"]["frontEnd_messaging_url"],
        message=f"Tool {toolNumber} Kept Successfully",
    )
    if goToSafe:
        config["logger"].info(
            "[toolMotion][manualDrop] phase=exit_safe profile=robot ratio=%.3f",
            float(drop_fast),
        )
        communicate(
            cps=cps,
            point=config["point"]["safePointTool"],
            tcp=config["coords"]["tcpDefault"],
            ucs=config["coords"]["ucsDefault"],
            seventh=-1,
            config=config,
            speed=drop_fast,
            velocity_profile="robotspeed",
            speed_mode="linear",
            wait=False,
        )


def keepToolupdated(cps, toolNumber, config):
    # if (not config['settings']['useTool']):
    #     return
    msg_to_frontend(
        api_url=config["server"]["frontEnd_messaging_url"],
        message=f"Tool {toolNumber} Keeping Started...",
    )
    # come to safe tool picking position
    communicate(
        cps=cps,
        point=config["point"]["safePointTool"],
        tcp=config["coords"]["tcpDefault"],
        ucs=config["coords"]["ucsDefault"],
        seventh=-1,
        config=config,
        speed=0.9,
        velocity_profile="robotspeed",
        wait=True,
    )
    # go to tool's home
    communicate(
        cps=cps,
        point=config["point"][f"tool{toolNumber}home"],
        tcp=config["coords"]["tcpDefault"],
        ucs=config["coords"]["ucsDefault"],
        seventh=-1,
        config=config,
        speed=0.9,
        velocity_profile="robotspeed",
        wait=True,
    )
    # touch the tool (slowly)
    communicate(
        cps=cps,
        point=config["point"][f"tool{toolNumber}"],
        tcp=config["coords"]["tcpDefault"],
        ucs=config["coords"]["ucsDefault"],
        seventh=-1,
        config=config,
        speed=0.1,
        velocity_profile="sandingspeed",
        wait=True,
    )
    # drop the tool
    waitForBlending(cps=cps, config=config)
    toolValve1(cps, valveState="drop", config=config)
    # come back to tool's home
    communicate(
        cps=cps,
        point=config["point"][f"tool{toolNumber}home"],
        tcp=config["coords"]["tcpDefault"],
        ucs=config["coords"]["ucsDefault"],
        seventh=-1,
        config=config,
        speed=0.1,
        velocity_profile="sandingspeed",
        wait=True,
    )
    # if don't need to pick another tool just after dropping this one, then come to safe picking position
    # msg_to_frontend(api_url=config['server']['frontEnd_messaging_url'], message=f"Tool {toolNumber} Kept Successfully")
    # if goToSafe:
    #     communicate(cps=cps, point=config['point']['safePointTool'],tcp=config['coords']['tcpDefault'], ucs=config['coords']['ucsDefault'], seventh=-1, config=config, speed=0.9, wait=True)


def getToolUpdated(cps, toolNumber, config, startFromSafe=True):
    """_summary_

    Args:
        cps (CPSClient): cobot client
        toolNumber (int): The position from which to pick tool (out of 1-4)
        config (config): configuration info
        startFromSafe (bool, optional): If should go to the safe tool picking position or not. Make False if doing tool drop and pick one after another. Defaults to True.
    """
    # if (not config['settings']['useTool']):
    #     return
    # msg_to_frontend(api_url=config['server']['frontEnd_messaging_url'], message=f"Tool {toolNumber} Collection Started...")
    # # if didn't drop another tool just before picking this one, then come to safe picking position
    # if startFromSafe:
    #     communicate(cps=cps, point=config['point']['safePointTool'],tcp=config['coords']['tcpDefault'], ucs=config['coords']['ucsDefault'], seventh=-1, config=config, speed=0.9, wait=False)

    # go to that tool's home position (right above the tool)
    communicate(
        cps=cps,
        point=config["point"][f"tool{toolNumber}home"],
        tcp=config["coords"]["tcpDefault"],
        ucs=config["coords"]["ucsDefault"],
        seventh=-1,
        config=config,
        speed=0.9,
        velocity_profile="robotspeed",
        wait=True,
    )
    # drop (for safety, to open the valve)
    waitForBlending(cps=cps, config=config)
    toolValve1(cps, valveState="drop", config=config)
    # touch the tool (slowly)
    communicate(
        cps=cps,
        point=config["point"][f"tool{toolNumber}"],
        tcp=config["coords"]["tcpDefault"],
        ucs=config["coords"]["ucsDefault"],
        seventh=-1,
        config=config,
        speed=0.1,
        velocity_profile="sandingspeed",
        wait=True,
    )
    # pick the tool
    waitForBlending(cps=cps, config=config)
    toolValve1(cps, valveState="pick", config=config)
    # come back to tool's home position
    communicate(
        cps=cps,
        point=config["point"][f"tool{toolNumber}home"],
        tcp=config["coords"]["tcpDefault"],
        ucs=config["coords"]["ucsDefault"],
        seventh=-1,
        config=config,
        speed=0.1,
        velocity_profile="sandingspeed",
        wait=True,
    )
    # come back to safe tool picking position
    msg_to_frontend(
        api_url=config["server"]["frontEnd_messaging_url"],
        message=f"Tool {toolNumber} Collection Successful!",
    )
    communicate(
        cps=cps,
        point=config["point"]["safePointTool"],
        tcp=config["coords"]["tcpDefault"],
        ucs=config["coords"]["ucsDefault"],
        seventh=-1,
        config=config,
        speed=0.9,
        velocity_profile="robotspeed",
        wait=True,
    )

def turn_tool_spin_on(cps, debug=False):
    """
    Turns the tool spin on by setting nBit=5 to nVal=0.
    """
    boxID = 0  # Default box ID
    nBit = 7  # Bit controlling tool spin
    nVal = 1  # 0 = On (as per your request)
    nRet = -1
    if not debug:
        nRet = cps.HRIF_SetBoxDO(boxID, nBit, nVal)

    if nRet == 0:
        print("Tool spin turned ON successfully.")
    else:
        print(f"Error turning ON tool spin. Error code: {nRet}")
        
def turn_tool_spin_off(cps, debug=False):
    """
    Turns the tool spin off by setting nBit=5 to nVal=1.
    """
    boxID = 0  # Default box ID
    nBit = 7  # Bit controlling tool spin
    nVal = 0  # 1 = Off (as per your request)
    nRet = -1
    if not debug:
        nRet = cps.HRIF_SetBoxDO(boxID, nBit, nVal)

    if nRet == 0:
        print("Tool spin turned OFF successfully.")
    else:
        print(f"Error turning OFF tool spin. Error code: {nRet}")

def turn_vibration_on(cps, debug=False):
    """
    Turns the vibration on by setting nBit=4 to nVal=0.
    """
    boxID = 0  # Default box ID
    nBit = 4  # Bit controlling vibration
    nVal = 1  # 0 = On (as per your request)
    nRet = -1
    if not debug:
        nRet = cps.HRIF_SetBoxDO(boxID, nBit, nVal)

    if nRet == 0:
        print("Vibration turned ON successfully.")
    else:
        print(f"Error turning ON vibration. Error code: {nRet}")


def turn_vibration_off(cps, debug=False):
    """
    Turns the vibration off by setting nBit=4 to nVal=1.
    """
    boxID = 0  # Default box ID
    nBit = 4  # Bit controlling vibration
    nVal = 0  # 1 = Off (as per your request)
    nRet = -1
    if not debug:
        nRet = cps.HRIF_SetBoxDO(boxID, nBit, nVal)

    if nRet == 0:
        print("Vibration turned OFF successfully.")
    else:
        print(f"Error turning OFF vibration. Error code: {nRet}")


def moveOnlyJ6r(cps, J6, config, wait=True):
    """Only increase the J6 angle to the certain value given. handles both positive and negative values

    Args:
        cps (cps): cobot client
        J6 (float): angle value (in degrees)
        config (config): our config value
    """
    result = []  # Read the current actual location information
    nret = cps.HRIF_ReadActPos(0, 0, result)  # Read the joint position variable
    dJ1 = float(result[0])
    dJ2 = float(result[1])
    dJ3 = float(result[2])
    dJ4 = float(result[3])
    dJ5 = float(result[4])
    dJ6 = float(result[5]) + J6  # Read the spatial position variable

    waitForBlending(cps, config)
    # setSpeed(cps, 0.6, config=config)

    sTcpName = config["coords"]["tcpDefault"]  # Definetheusercoordinatesvariable
    sUcsName = config["coords"]["ucsDefault"]  # Definemovementspeed
    dVelocity = 100  # Define the movement acceleration
    dAcc = 150  # Define the transition radius
    dRadius = config["coords"][
        "transitionRadius"
    ]  # Deine whether the joint angle is used
    nIsUseJoint = 1  # Define whether to stop using the test DI
    nIsSeek = 0  # Define The DI index of the detection
    nIOBit = 0  # Define The DI status of the detected state
    nIOState = 0  # Defining Road Point ID
    stdCmdID = "0"  # Perform road point movement

    nret = cps.HRIF_MoveJ(
        0,
        0,
        config["point"]["table1Origin"],
        [dJ1, dJ2, dJ3, dJ4, dJ5, dJ6],
        sTcpName,
        sUcsName,
        dVelocity,
        dAcc,
        dRadius,
        nIsUseJoint,
        nIsSeek,
        nIOBit,
        nIOState,
        stdCmdID,
    )
    robotRes = []
    # time.sleep(0.2)
    if nret != 0:
        config["logger"].error(
            f"[moveOnlyJ6] Failure in moving to joint pos {[dJ1, dJ2, dJ3, dJ4, dJ5, dJ6]}"
        )
        msg_to_frontend(
            api_url=config["server"]["frontEnd_messaging_url"],
            message=f"Cobot Reached Joint Position Limit. Perform Homing and Try Again. Exiting Cycle...",
        )
        exit(-1)
    nret = -1
    while wait:
        nret = cps.HRIF_ReadRobotState(0, 0, robotRes)
        config["logger"].info(robotRes)
        if robotRes[11] == "1":
            config["logger"].info(
                f"[moveOnlyJ6] Success in moving to joint pos {[dJ1, dJ2, dJ3, dJ4, dJ5, dJ6]}"
            )
            return
        config["logger"].info(
            f"[moveOnlyJ6] Trying to move joints to {[dJ1, dJ2, dJ3, dJ4, dJ5, dJ6]}"
        )
        # time.sleep(0.2)


def homingFunction1(cps, config):
    result = [-1, -1, -1, -1]
    door_cfg = config.get("door", {}) if isinstance(config, dict) else {}
    homing_blend_timeout_s = max(
        0.2, float(door_cfg.get("homingBlendTimeoutSeconds", 1.0))
    )
    motor_connect_settle_s = max(
        0.01, float(door_cfg.get("homingMotorConnectDelaySec", 0.04))
    )
    motor_stop_settle_s = max(
        0.01, float(door_cfg.get("homingMotorStopDelaySec", 0.05))
    )
    origin_command_settle_s = max(
        0.01, float(door_cfg.get("homingOriginCommandDelaySec", 0.125))
    )
    origin_poll_s = max(
        0.02, float(door_cfg.get("homingOriginPollIntervalSec", 0.05))
    )
    origin_wait_timeout_s = max(
        5.0, float(door_cfg.get("homingOriginWaitTimeoutSec", 60.0))
    )

    def origin_done(state):
        if not isinstance(state, (list, tuple)) or len(state) < 3:
            return False
        raw_s = str(state[2]).strip().lower()
        if raw_s in ("idle", "done", "false"):
            return True
        try:
            return float(raw_s) == 0.0
        except (TypeError, ValueError):
            return False

    def motor_cmd_ok(nret, state):
        if nret is None:
            return bool(state) and str(state[0]) == "0"
        return nret == 0 or (bool(state) and str(state[0]) == "0")

    setUCS_TCP(
        cps,
        tcp=config["coords"]["tcpDefault"],
        ucs=config["coords"]["ucsDefault"],
        config=config,
    )
    # result = [ ] # Read the current actual location information
    nRet = cps.HRIF_ReadActPos(0, 0, result)  # Read the joint position variable
    dX = float(result[6])
    # config['logger'].info(f"[homing] current position: {result}")
    msg_to_frontend(
        api_url=config["server"]["frontEnd_messaging_url"], message="Homing Started..."
    )
    if dX > config["point"]["safePointTool"][0]:
        communicate(
            cps=cps,
            point=config["point"]["safePointTool"],
            tcp=config["coords"]["tcpDefault"],
            ucs=config["coords"]["ucsDefault"],
            seventh=-1,
            config=config,
            speed=config["door"]["homingSpeed"],
            wait=False,
        )
    communicate(
        cps=cps,
        point=config["point"]["safePoint"],
        tcp=config["coords"]["tcpDefault"],
        ucs=config["coords"]["ucsDefault"],
        seventh=-1,
        config=config,
        speed=config["door"]["homingSpeed"],
        wait=False,
    )
    waitForBlending(cps=cps, config=config, timeout_s=homing_blend_timeout_s)
    # connect the 7th axis motor
    nret = cps.HRIF_HRApp(0, "HR_Motor", "MotorConnect", ["J7"], result)
    time.sleep(motor_connect_settle_s)
    if result[1] != "OK":
        config["logger"].error(
            "[HomingFunc] Could not connect to the motor. Exiting..."
        )
        msg_to_frontend(
            api_url=config["server"]["frontEnd_messaging_url"],
            message="7th Axis Connection Error. Please Check if It's Working and Try Again! Terminating Cycle...",
        )
        exit(-1)
    # Step 2: go to your homing position (for 7th axis)
    config["logger"].info("[HomingFunc] step 2: Go to the homing switch")
    # communicate(cps=cps, tcp=config['coords']['tcpDefault'], ucs=config['coords']['ucsDefault'], seventh=0, config=config, speed=config['door']['homingSpeed'])
    max_origin_cmd_retries = int(door_cfg.get("homingOriginCommandRetries", 3) or 3)
    if max_origin_cmd_retries < 1:
        max_origin_cmd_retries = 1
    origin_started = False
    for attempt in range(1, max_origin_cmd_retries + 1):
        result = []
        nret = cps.HRIF_HRApp(0, "HR_Motor", "MotorStop", ["J7"], result)
        time.sleep(motor_stop_settle_s)
        print(f"****** motor stop nret: {nret}; result: {result}")
        result = []
        nret = cps.HRIF_HRApp(0, "HR_Motor", "MotorMoveOrigin", ["J7"], result)
        # seventhGoToPos(cpsclient, position=0, speed=config['7thAxis']['speed'] * config['cobot']['speed'], config=config)
        time.sleep(origin_command_settle_s)
        print(f"****** move origin nret: {nret}; result: {result}")
        if motor_cmd_ok(nret, result):
            origin_started = True
            break
        config["logger"].warning(
            f"[HomingFunc] MotorMoveOrigin not accepted on attempt "
            f"{attempt}/{max_origin_cmd_retries} (ret={nret}, res={result}). Retrying..."
        )
        cps.HRIF_HRApp(0, "HR_Motor", "MotorConnect", ["J7"], [])
        time.sleep(0.05 * attempt)
    if not origin_started:
        config["logger"].error(
            f"[HomingFunc] MotorMoveOrigin failed after {max_origin_cmd_retries} attempts."
        )
        msg_to_frontend(
            api_url=config["server"]["frontEnd_messaging_url"],
            message="Homing failed: J7 origin command was not accepted.",
        )
        return

    result = [-1, -1, "-1"]
    # for waiting till the motor moves to the position
    origin_wait_start = time.time()
    while not origin_done(result):
        nret = cps.HRIF_HRApp(0, "HR_Motor", "MotorGetState", ["J7"], result)
        if nret not in (0, None):
            config["logger"].warning(
                f"[HomingFunc] MotorGetState returned ret={nret}; res={result}"
            )
        print(f"****** result: {result}")
        if (time.time() - origin_wait_start) >= origin_wait_timeout_s:
            try:
                cps.HRIF_HRApp(0, "HR_Motor", "MotorStop", ["J7"], [])
            except Exception:
                pass
            config["logger"].error(
                f"[HomingFunc] Timeout waiting for J7 origin complete after "
                f"{origin_wait_timeout_s:.1f}s. Last state={result}"
            )
            msg_to_frontend(
                api_url=config["server"]["frontEnd_messaging_url"],
                message="Homing failed: timeout while waiting for J7 origin.",
            )
            return
        time.sleep(origin_poll_s)
    # seventhGoToPos(cps, position=0, speed=UISettings['control']['linearAxisSpeed'] * config['7thAxis']['speed'], config=config)
    config["logger"].info("[HomingFunc] DONE! Success to reach 0th position")
    msg_to_frontend(
        api_url=config["server"]["frontEnd_messaging_url"],
        message="Homing Completed Successfully!",
    )


def laser(cps, valveState: str, config):  # Tool valve for grabbing or throwing
    """Used to "pick" or "drop" the tool.
    Helpful: When DO5 is 1: the pneumatic is loose. When D05 is 0, pneumatic is tight.

    Args:
        cps (CPSClient): cps
        valveState (str): "pick" to grab the tool, "drop" to let the tool go.
        config (config): configuration file
    """
    # required sleep (for proper functioning)
    time.sleep(0.5)
    if valveState == "on":
        status = 1
        digOutput = 3  # DOnumber=0,1,2,3,4
        nRet = cps.HRIF_SetBoxDO(0, digOutput, status)
        if config["settings"]["debug"]:
            config["logger"].info(f"Laser is on: {nRet} (1 means successful)")

    elif valveState == "off":
        status = 0
        digOutput = 3  # DOnumber=0,1,2,3,4
        nRet = cps.HRIF_SetBoxDO(0, digOutput, status)
        if config["settings"]["debug"]:
            config["logger"].info(f"Laser is off: {nRet} (0 means successful)")
    # required sleep (for proper functioning)
    msg_to_frontend(
        api_url=config["server"]["frontEnd_messaging_url"],
        message=f"Tool Set to '{valveState}'",
    )
    time.sleep(0.5)


def stopper_statusmod(cps, digital_number=2, state=None):
    """
    Control stopper status
    - state="up": sets DO to 0
    - state="down": sets DO to 1
    - state=None: toggles current status
    """
    stopper_status = []
    nRet = cps.HRIF_ReadBoxDO(0, digital_number, stopper_status)

    if state is None:
        # Toggle mode - original functionality
        if stopper_status[0] == "1":  # Currently down
            nRet = cps.HRIF_SetBoxDO(0, digital_number, 0)  # Set to up
            return "up"
        else:  # Currently up
            nRet = cps.HRIF_SetBoxDO(0, digital_number, 1)  # Set to down
            return "down"
    else:
        # Set to specific state
        if state.lower() == "up":
            nRet = cps.HRIF_SetBoxDO(0, digital_number, 0)
            return "up"
        elif state.lower() == "down":
            nRet = cps.HRIF_SetBoxDO(0, digital_number, 1)
            return "down"
        else:
            raise ValueError("State must be 'up', 'down', or None")


def set_table_state(CPS, table_id, desired_state):
    """
    Set the table to a specific desired state
    """
    timeout_s = 6.0
    poll_s = 0.05

    def _wait_until(state_reader, expected_fn, timeout=timeout_s, poll=poll_s):
        end_t = time.monotonic() + max(0.2, float(timeout))
        last_value = None
        while time.monotonic() < end_t:
            if stop_requested():
                return False, "Stop requested", last_value
            ok, value = state_reader()
            last_value = value
            if ok and expected_fn(value):
                return True, "confirmed", value
            time.sleep(max(0.01, float(poll)))
        return False, "timeout", last_value

    def _read_table_a_di():
        di_state_0 = []
        di_state_1 = []
        nRet0 = CPS.HRIF_ReadBoxDI(0, 0, di_state_0)
        nRet1 = CPS.HRIF_ReadBoxDI(0, 1, di_state_1)
        ok = (
            nRet0 == 0
            and nRet1 == 0
            and len(di_state_0) > 0
            and len(di_state_1) > 0
        )
        value = (
            str(di_state_0[0]) if len(di_state_0) > 0 else None,
            str(di_state_1[0]) if len(di_state_1) > 0 else None,
        )
        return ok, value

    def _read_table_b_co():
        robot_state = []
        nRet = CPS.HRIF_ReadBoxCO(0, 1, robot_state)
        ok = nRet == 0 and len(robot_state) > 0
        value = str(robot_state[0]) if len(robot_state) > 0 else None
        return ok, value

    if table_id == "tableAOpenClose":
        if desired_state == "Open":
            # Set Table A to Open position (horizontal)
            nRet = CPS.HRIF_SetBoxDO(0, 1, 0)
            nRet = CPS.HRIF_SetBoxDO(0, 0, 1)
            confirmed, reason, last_value = _wait_until(
                _read_table_a_di, lambda v: v == ("1", "0")
            )
            if confirmed:
                return {
                    "success": True,
                    "newState": "Open",
                    "message": "Table A set to horizontal position",
                }
            else:
                return {
                    "success": False,
                    "newState": "Error",
                    "message": f"Failed to set Table A to open position ({reason}, sensor={last_value})",
                }

        elif desired_state == "Close":
            # Set Table A to Close position (45 degrees)
            nRet = CPS.HRIF_SetBoxDO(0, 0, 0)
            nRet = CPS.HRIF_SetBoxDO(0, 1, 1)
            confirmed, reason, last_value = _wait_until(
                _read_table_a_di, lambda v: v == ("0", "1")
            )
            if confirmed:
                return {
                    "success": True,
                    "newState": "Close",
                    "message": "Table A set to 45 degree position - WARNING: Be careful when manually moving robot",
                }
            else:
                return {
                    "success": False,
                    "newState": "Error",
                    "message": f"Failed to set Table A to close position ({reason}, sensor={last_value})",
                }
        else:
            return {
                "success": False,
                "newState": "Error",
                "message": "Invalid desired state for Table A",
            }

    elif table_id == "tableBOpenClose":
        if desired_state == "Open":
            # Set Table B to Open position (horizontal)
            nRet = CPS.HRIF_SetBoxCO(0, 1, 0)
            nRet = CPS.HRIF_SetBoxCO(0, 0, 1)
            confirmed, reason, last_value = _wait_until(
                _read_table_b_co, lambda v: v == "0"
            )
            if confirmed:  # Assuming '0' means Open for Table B
                return {
                    "success": True,
                    "newState": "Open",
                    "message": "Table B set to horizontal position",
                }
            else:
                return {
                    "success": False,
                    "newState": "Error",
                    "message": f"Failed to set Table B to open position ({reason}, state={last_value})",
                }

        elif desired_state == "Close":
            # Set Table B to Close position (45 degrees)
            nRet = CPS.HRIF_SetBoxCO(0, 0, 0)
            nRet = CPS.HRIF_SetBoxCO(0, 1, 1)
            confirmed, reason, last_value = _wait_until(
                _read_table_b_co, lambda v: v == "1"
            )
            if confirmed:  # Assuming '1' means Close for Table B
                return {
                    "success": True,
                    "newState": "Close",
                    "message": "Table B set to 45 degree position - WARNING: Be careful when manually moving robot",
                }
            else:
                return {
                    "success": False,
                    "newState": "Error",
                    "message": f"Failed to set Table B to close position ({reason}, state={last_value})",
                }
        else:
            return {
                "success": False,
                "newState": "Error",
                "message": "Invalid desired state for Table B",
            }

    else:
        return {"success": False, "newState": "Error", "message": "Invalid table ID"}


if __name__ == "__main__":
    # Read the YAML configuration file
    with open("./configs/config.yaml", "r") as file:
        config = yaml.safe_load(file)

    handle_client(config)


