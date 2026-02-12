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


def waitForBlending(cps, config, timeout_s=7):
    start_time = time.time()
    while True:
        if stop_requested():
            return
        result = []
        nret = cps.HRIF_IsBlendingDone(0, 0, result)
        done = False
        if nret != 0:
            # Some SDKs return 20018 when blending status is not available in
            # the current robot state; treat it as "no wait needed".
            if nret != 20018 and isinstance(config, dict) and config.get("logger"):
                config["logger"].warning(
                    f"[waitForBlending] HRIF_IsBlendingDone error (ret={nret}); continuing."
                )
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
            break
    # Keep a minimal settle delay; larger fixed sleeps add visible latency
    # between chained point-to-point moves.
    time.sleep(0.02)
    return
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


def msg_to_frontend(api_url, message):
    try:
        response = requests.post(api_url, json={"message": message})
        response.raise_for_status()  # Raise exception for HTTP errors
        return response.json()
    except requests.exceptions.RequestException as e:
        print(f"Failed to send message to trigger API: {e}")
        return None


# Global stop flag shared by long-running operations.
STOP_REQUESTED = False


def request_stop():
    global STOP_REQUESTED
    STOP_REQUESTED = True


def clear_stop():
    global STOP_REQUESTED
    STOP_REQUESTED = False


def stop_requested() -> bool:
    return STOP_REQUESTED




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


# def setSpeed(cps, speed, config):
#         """Sets the speed of the cobot

#         Args:
#             cps (cps): cps
#             speed (float): expects speed to be in [0,1] range
#         """
#         if speed is None: return
#         currSpeed = [ ] # Read the maximum joint speed
#         nRet = cps.HRIF_ReadOverride(0,0, currSpeed)
#         if float(currSpeed[0]) != speed:
#             if speed is not None:
#                 waitForBlending(cps=cps, config=config)
#                 nRet = cps.HRIF_SetOverride(0,0, speed)
#                 # time.sleep(0.2)

#                 if nRet == 0:
#                     config['logger'].info(f'[setSpeed] Could set speed to {speed * 100}%')
#                 else:
#                     config['logger'].error(f"[setSpeed] Couldn't set speed to {speed * 100}%, ret: {nRet}")
#                     msg_to_frontend(api_url=config['server']['frontEnd_messaging_url'], message="Error With Robot Settings. Please Verify The Robot Settings and Try Again. Terminating Process...")
#                     exit(-1)
#         return


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


# def control_table_status(cps):
#     openTableStatus = []
#     nRet = cps.HRIF_ReadBoxDO(0, 1, openTableStatus)
#     if openTableStatus[0] == '1':
#         nRet = cps.HRIF_SetBoxDO(0, 1, 0) # (0, digital output number, states)
#         nRet = cps.HRIF_SetBoxDO(0, 0, 1)
#     else:
#         nRet = cps.HRIF_SetBoxDO(0, 0, 0)
#         nRet = cps.HRIF_SetBoxDO(0, 1, 1)


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
    damp = [8000, 8000, 8000, 40, 40, 40]
    nRet = cps.HRIF_SetDampParams(0, 0, damp)
    time.sleep(0.0001)
    if nRet != 0:
        config["logger"].error(f"Failed to set damp params: {nRet}")
        return

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
            waitForBlending(cps, config, timeout_s=0.4)
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
            waitForBlending(cps, config, timeout_s=0.4)
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
        waitForBlending(cps=cps, config=config)
        if stop_requested():
            msg_to_frontend(
                api_url=config["server"]["frontEnd_messaging_url"],
                message="Homing stopped by user.",
            )
            return
        # connect the 7th axis motor
        nret = cps.HRIF_HRApp(0, "HR_Motor", "MotorConnect", ["J7"], result)
        time.sleep(0.2)
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
        nret = cps.HRIF_HRApp(0, "HR_Motor", "MotorStop", ["J7"], result)
        time.sleep(0.3)
        print(f"****** motor stop nret: {nret}; result: {result}")
        nret = cps.HRIF_HRApp(0, "HR_Motor", "MotorMoveOrigin", ["J7"], result)
        # nret = cps.HRIF_HRApp(0, 'HR_Motor','MotorMovePosition', ["J7", "-22.0"], result)
        # seventhGoToPos(cpsclient, position=0, speed=config['7thAxis']['speed'] * config['cobot']['speed'], config=config)
        time.sleep(1)
        print(f"****** move origin nret: {nret}; result: {result}")
        result = [-1, -1, "-1"]
        # for waiting till the motor moves to the position
        while result[2] != "0":
            if stop_requested():
                msg_to_frontend(
                    api_url=config["server"]["frontEnd_messaging_url"],
                    message="Homing stopped by user.",
                )
                return
            nret = cps.HRIF_HRApp(0, "HR_Motor", "MotorGetState", ["J7"], result)
            print(f"****** result: {result}")
            time.sleep(0.5)
        # Move to the configured J7 home position after origin.
        if stop_requested():
            msg_to_frontend(
                api_url=config["server"]["frontEnd_messaging_url"],
                message="Homing stopped by user.",
            )
            return
        home_j7 = float(config.get("UI", {}).get("seventhAxisHome", -65))
        config["logger"].info(
            f"[HomingFunc] Reached J7 origin; moving to home position {home_j7}mm"
        )
        ok = seventhGoToPos(
            cps=cps,
            position=home_j7,
            speed=0.5,
            config=config,
            wait=True,
        )
        if not ok:
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

    def homingFunction_old(cps, config):
        def check_return_value(val):
            if val != 0:
                config["logger"].info("error code: ", val)

        # config['logger'].info(config)
        result = []
        app_name = "MT_Kinco"
        setUCS_TCP(
            cps=cps,
            tcp=config["coords"]["tcpDefault"],
            ucs=config["coords"]["ucsDefault"],
            config=config,
        )
        config["logger"].info("[homing] Going back to safe positions for J1-J6...")
        #### check if it is behind the 0 line or not (so, the position will be X position of the tool Center), if yes, then only do safePointTool
        result = []  # Read the current actual location information
        nRet = cps.HRIF_ReadActPos(0, 0, result)  # Read the joint position variable
        dX = float(result[6])
        # config['logger'].info(f"[homing] current position: {result}")
        if dX > config["point"]["safePointTool"][0]:
            communicate(
                cps=cps,
                point=config["point"]["safePointTool"],
                tcp=config["coords"]["tcpDefault"],
                ucs=config["coords"]["ucsDefault"],
                seventh=-1,
                config=config,
                speed=config["UI"]["robotSpeed"],
            )
        communicate(
            cps=cps,
            point=config["point"]["safePoint"],
            tcp=config["coords"]["tcpDefault"],
            ucs=config["coords"]["ucsDefault"],
            seventh=-1,
            config=config,
            speed=config["UI"]["robotSpeed"],
        )

        #### Perform homing #####
        power_on = "MotorPowerOn"
        find_origin_params = ["6"]  # aims to 'Find Origin'
        ret = cps.HRIF_HRApp(
            0, app_name, power_on, find_origin_params, result
        )  # start to find origin
        check_return_value(ret)

        config["logger"].info("start to find origin")
        while True:  # wait for motor move done
            time.sleep(1)
            ret = cps.HRIF_HRApp(
                0, app_name, "MotorReadCurFSM", [], result
            )  # start to find origin
            # config['logger'].info(f"Homing thing: {result}")
            # robotRes = []
            # nret = cps.HRIF_ReadRobotState(0, 0, robotRes)
            # config['logger'].info(f"Robot thing: {robotRes}")
            time.sleep(0.2)
            check_return_value(ret)
            if stop_requested():
                msg_to_frontend(
                    api_url=config["server"]["frontEnd_messaging_url"],
                    message="Homing stopped by user.",
                )
                return
            if result[0] == "5" or result[0] == "6":
                config["logger"].info(result)
                break

        config["logger"].info("[homing] Success to find origin!")

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
            return
        if stop_requested():
            msg_to_frontend(
                api_url=config["server"]["frontEnd_messaging_url"],
                message="Tool collection stopped by user.",
            )
            return
        # Ensure prior blended motion is fully settled before speed/approach changes.
        waitForBlending(cps=cps, config=config)
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
                speed=config["UI"]["robotSpeed"],
                wait=True,
            )
            if stop_requested():
                msg_to_frontend(
                    api_url=config["server"]["frontEnd_messaging_url"],
                    message="Tool collection stopped by user.",
                )
                return

        # go to that tool's home position (right above the tool)
        communicate(
            cps=cps,
            point=config["point"][f"tool{toolNumber}home"],
            tcp=config["coords"]["tcpDefault"],
            ucs=config["coords"]["ucsDefault"],
            seventh=-1,
            config=config,
            speed=config["UI"]["robotSpeed"],
            wait=True,
        )
        if stop_requested():
            msg_to_frontend(
                api_url=config["server"]["frontEnd_messaging_url"],
                message="Tool collection stopped by user.",
            )
            return
        # drop (for safety, to open the valve)
        toolValve(cps, valveState="drop", config=config)
        if stop_requested():
            msg_to_frontend(
                api_url=config["server"]["frontEnd_messaging_url"],
                message="Tool collection stopped by user.",
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
            speed=0.15,
            wait=True,
        )
        if stop_requested():
            msg_to_frontend(
                api_url=config["server"]["frontEnd_messaging_url"],
                message="Tool collection stopped by user.",
            )
            return
        # pick the tool
        toolValve(cps, valveState="pick", config=config)
        if stop_requested():
            msg_to_frontend(
                api_url=config["server"]["frontEnd_messaging_url"],
                message="Tool collection stopped by user.",
            )
            return
        # come back to tool's home position
        communicate(
            cps=cps,
            point=config["point"][f"tool{toolNumber}home"],
            tcp=config["coords"]["tcpDefault"],
            ucs=config["coords"]["ucsDefault"],
            seventh=-1,
            config=config,
            speed=0.15,
            wait=True,
        )
        if stop_requested():
            msg_to_frontend(
                api_url=config["server"]["frontEnd_messaging_url"],
                message="Tool collection stopped by user.",
            )
            return
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
            speed=config["UI"]["robotSpeed"],
            wait=True,
        )

    def keepTool(cps, toolNumber, config, goToSafe=True):
        if not config["settings"]["useTool"]:
            return
        if stop_requested():
            msg_to_frontend(
                api_url=config["server"]["frontEnd_messaging_url"],
                message="Tool keeping stopped by user.",
            )
            return
        # Wait for previous trajectory to settle so override/speed is applied reliably.
        waitForBlending(cps=cps, config=config)
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
            speed=config["UI"]["robotSpeed"],
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
            speed=config["UI"]["robotSpeed"],
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
            speed=0.15,
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
        # come back to tool's home
        communicate(
            cps=cps,
            point=config["point"][f"tool{toolNumber}home"],
            tcp=config["coords"]["tcpDefault"],
            ucs=config["coords"]["ucsDefault"],
            seventh=-1,
            config=config,
            speed=0.15,
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
                speed=config["UI"]["robotSpeed"],
                wait=True,
            )

    def control_table(cps, tableState):
        config["logger"].info("[controlTable] Reached! Table State: %s", tableState)
        if tableState == "close":
            config["logger"].info("[controlTable] table closed")
            nRet = cps.HRIF_SetBoxDO(0, 1, 0)  # (0, digital output number, states)
            time.sleep(1)
            nRet = cps.HRIF_SetBoxDO(0, 0, 1)
            time.sleep(1)
        elif tableState == "open":
            config["logger"].info("[controlTable] table opened")
            nRet = cps.HRIF_SetBoxDO(0, 0, 0)
            time.sleep(1)
            nRet = cps.HRIF_SetBoxDO(0, 1, 1)
            time.sleep(1)
        msg_to_frontend(
            api_url=config["server"]["frontEnd_messaging_url"],
            message=f"Table Set To {tableState} (⚠️Please wait till the table fully opens/closes)...",
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
        # required sleep (for proper functioning)
        time.sleep(0.5)
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
        time.sleep(0.5)

    def formattedInstruction(point, seventhAxis=-1, debug=False):
        do7th = 0
        if seventhAxis != -1:
            do7th = 1
        all = point + [do7th, seventhAxis]
        # input(f"points: {all}, {type(all)}")
        # input(f"th: {seventhAxis}")
        point_str = ",".join(str(x) for x in all)
        if debug:
            config["logger"].info(f"Points_str: {point_str}")
        return point_str

    def saveAsCSV(fileName, measurements):
        # Specify the order of fieldnames
        ## here ct means 'current time'
        fieldnames = ["dist", "height"]

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
    def calculate_for_each_chunk(chunks, thresholds, default_threshold=(5, 5)):
        """
        Apply the frames and pocket calculation for each chunk with user-defined thresholds.
        If a chunk doesn't have a corresponding threshold, use the default threshold.
        Returns a list of dictionaries containing chunk index and results.
        """

        # Step 3: Function to calculate frames and pockets (the one you provided)
        # def calculate_frames_and_pockets(df, threshold_increase=5, threshold_decrease=5):
        def calculate_frames_and_pockets(
            data, threshold_increase=5, threshold_decrease=5
        ):
            """
            Identify the significant increase (initial point) and significant decrease (second point)
            and return the frames and pocket based on the thresholds.

            Args:
                data (list): List of dictionaries with 'dist' and 'height' as keys.
                threshold_increase (int): The height increase threshold to identify the start of the pocket.
                threshold_decrease (int): The height decrease threshold to identify the end of the pocket.

            Returns:
                dict: A dictionary containing information about 'frame_1', 'pocket', and 'frame_2' or None.
            """

            # Filter out NaN values from the 'height' key
            data = [item for item in data if not math.isnan(item["height"])]

            # Ensure there's enough data to proceed
            if len(data) == 0:
                config["logger"].error("No valid data after removing NaN values.")
                msg_to_frontend(
                    api_url=config["server"]["frontEnd_messaging_url"],
                    message="Error With Laser! Please Verify it is Working and Try Again. Terminating Process...",
                )
                exit(-1)

            # Get the first and last points in the dataset
            first_point = data[0]
            last_point = data[-1]

            # Find the first point where the height increases by 'threshold_increase' units from the first point
            pocket_start_idx = None
            for i in range(1, len(data)):
                if (data[i]["height"] - first_point["height"]) >= threshold_increase:
                    pocket_start_idx = i
                    pocket_start = data[pocket_start_idx]
                    break

            if pocket_start_idx is None:
                config["logger"].info("No significant increase found.")
                return None

            # Find the first point after the pocket start where the height decreases by 'threshold_decrease' units
            pocket_end_idx = None
            for i in range(pocket_start_idx + 1, len(data)):
                if (pocket_start["height"] - data[i]["height"]) >= threshold_decrease:
                    pocket_end_idx = i
                    pocket_end = data[pocket_end_idx]
                    break

            if pocket_end_idx is None:
                config["logger"].info(
                    "No significant decrease found after the initial increase."
                )
                return None

            # Calculate the 3 sets:
            # Frame 1: From the first point to the initial point (pocket start)
            distance_frame1 = pocket_start["dist"] - first_point["dist"]

            # Pocket: From the initial point (pocket start) to the second point (pocket end)
            distance_pocket = pocket_end["dist"] - pocket_start["dist"]

            # Frame 2: From the second point (pocket end) to the last point
            distance_frame2 = last_point["dist"] - pocket_end["dist"]

            # Store the results in a dictionary
            result = {
                "frame_1": {
                    "first_point": first_point["dist"],
                    "second_point": pocket_start["dist"],
                    "distance": distance_frame1,
                },
                "pocket": {
                    "first_point": pocket_start["dist"],
                    "second_point": pocket_end["dist"],
                    "distance": distance_pocket,
                },
                "frame_2": {
                    "first_point": pocket_end["dist"],
                    "second_point": last_point["dist"],
                    "distance": distance_frame2,
                },
            }

            return result

        results = []

        for i, chunk in enumerate(chunks):
            # Use provided thresholds if available, otherwise fallback to the default
            if i < len(thresholds):
                threshold_increase, threshold_decrease = thresholds[i]
            else:
                threshold_increase, threshold_decrease = default_threshold
                config["logger"].info(
                    f"\nChunk {i + 1}: No specific threshold provided. Using default thresholds: (increase={threshold_increase}, decrease={threshold_decrease})"
                )

            # Apply the calculation function for each chunk
            config["logger"].info(
                f"\nProcessing chunk {i + 1} with threshold (increase={threshold_increase}, decrease={threshold_decrease})"
            )
            # chunk_df = pd.Dataframe.from_dict(chunk)
            result = calculate_frames_and_pockets(
                chunk,
                threshold_increase=threshold_increase,
                threshold_decrease=threshold_decrease,
            )

            if result:
                # Store the result with the chunk number
                chunk_result = {
                    "chunk_index": i + 1,
                    "frame_1": result["frame_1"]["distance"],
                    "pocket": result["pocket"]["distance"],
                    "frame_2": result["frame_2"]["distance"],
                }
                results.append(chunk_result)
            else:
                config["logger"].info(f"No valid results for chunk {i + 1}")

        return results

    def identify_gradient_change_points_dynamic_old(chunks, thresholds):
        """
        Identifies the exact points where the gradient starts and stops using start and stop thresholds.

        Args:
        - distances (list): List of distances.
        - heights (list): List of corresponding heights.
        - threshold (float): The threshold for detecting the start and stop of a gradient.
        - min_distance (float): Minimum distance after which to detect the first gradient change.

        Returns:
        - gradient_changes (list of dicts): List of points where gradients start and stop with the distance.
        """
        chunks = [item for item in chunks if not math.isnan(item["height"])]

        distances = [item["dist"] for item in chunks]
        heights = [item["height"] for item in chunks]
        start_threshold = thresholds[
            0
        ]  # Threshold for detecting the start and stop of a gradient
        stop_threshold = thresholds[1]
        min_distance = 15

        gradient_changes = []
        gradients = np.diff(heights) / np.diff(distances)

        positive_start_idx = None
        negative_start_idx = None

        # Loop through gradients, starting only after the specified minimum distance
        for i in range(1, len(gradients)):
            if distances[i] < min_distance:
                continue  # Skip points until the distance is at least 10mm

            # Detect start of positive gradient
            if abs(gradients[i]) >= start_threshold and positive_start_idx is None:
                positive_start_idx = i

            # Detect end of positive gradient
            elif positive_start_idx is not None and abs(gradients[i]) < stop_threshold:
                gradient_changes.append(
                    {
                        "type": "positive_start",
                        "distance": distances[positive_start_idx],
                        "gradient": gradients[positive_start_idx],
                    }
                )
                gradient_changes.append(
                    {
                        "type": "positive_stop",
                        "distance": distances[i],
                        "gradient": gradients[i],
                    }
                )
                positive_start_idx = None  # Reset

            # Detect start of negative gradient
            # if gradients[i] <= -start_threshold and negative_start_idx is None:
            #     negative_start_idx = i

            # # Detect end of negative gradient
            # elif negative_start_idx is not None and gradients[i] > -stop_threshold:
            #     gradient_changes.append({
            #         'type': 'negative_start',
            #         'distance': distances[negative_start_idx],
            #         'gradient': gradients[negative_start_idx]
            #     })
            #     gradient_changes.append({
            #         'type': 'negative_stop',
            #         'distance': distances[i],
            #         'gradient': gradients[i]
            #     })
            #     negative_start_idx = None  # Reset

        frame1_point1 = float(chunks[0]["dist"])
        frame1_point2 = gradient_changes[0]["distance"]
        pocket_point1 = gradient_changes[1]["distance"]
        pocket_point2 = gradient_changes[2]["distance"]
        frame2_point1 = gradient_changes[3]["distance"]
        frame2_point2 = float(chunks[-1]["dist"])
        # config['logger'].info("frame2_point2: ", frame2_point2)
        # config['logger'].info("frame2_point1: ", frame2_point1)

        frame1 = frame1_point2 - frame1_point1
        pocket = pocket_point2 - pocket_point1
        threeD1 = pocket_point1 - frame1_point2
        threeD2 = frame2_point1 - pocket_point2
        frame2 = frame2_point2 - frame2_point1

        result = {
            "frame_1": frame1,
            "threeD_1": threeD1,
            "pocket": pocket,
            "threeD_2": threeD2,
            "frame_2": frame2,
        }
        # config['logger'].info ("Result: ", result)
        return result

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
        config["logger"].info("Debug: %s", stable_regions)

        total_length = max(float(chunks[-1]["dist"]) - float(chunks[0]["dist"]), 0.0)
        height_range = max(heights) - min(heights)

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
                "reason": reason,
            }

        # If we cannot find two stable bands, we treat the door as uniform depth.
        if len(stable_regions) < 2:
            return uniform_depth_result("stable_regions_lt_2")

        frame1_point1 = float(chunks[0]["dist"])
        frame1_point2 = stable_regions[0]["end_distance"]
        pocket_point1 = stable_regions[0]["end_distance"]
        pocket_point2 = stable_regions[-1]["start_distance"]
        frame2_point1 = stable_regions[-1]["start_distance"]
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

        result = {
            "frame_1": frame1,
            "threeD_1": three_d_compensation,
            "pocket": pocket,
            "threeD_2": three_d_compensation,
            "frame_2": frame2,
            "profileType": "frame_and_pocket",
            "pocketDetected": True,
            "stableRegionCount": len(stable_regions),
            "heightRange": height_range,
            "reason": "stable_regions_detected",
        }

        return result

    def find_constant_height_periods(measurements, threshold):
        def entireFrame(measurements):
            """Find total frame dist."""
            res = 0
            for i, entry in enumerate((measurements)):
                if entry["height"] > 100:
                    break
                res += entry["length"]
            return res

        def flatFrame(measurements):
            """Find total frame dist."""
            res = 0
            for i, entry in enumerate(reversed(measurements)):
                if entry["height"] > 92.5:
                    break
                res += entry["length"]
            return res

        periods = []
        current_height = None
        start_dist = None

        # Iterate through the dictionary and remove entries with NaN height
        # measurements_filtered = [measurement for measurement in measurements if not  math.isnan(measurement.get("height", float('inf')))]
        measureNonNan = []
        for item in measurements:
            if math.isnan(item["height"]):
                continue
            measureNonNan.append(item)

        for idx, measurement in enumerate(measureNonNan):
            dist = measurement["dist"]
            height = measurement["height"]
            if math.isnan(height):
                continue
            if current_height is None:
                # Initialize for the first valid measurement
                current_height = height
                start_dist = dist
            elif abs(height - current_height) > threshold:
                # Height changed
                length = dist - start_dist
                periods.append({"height": current_height, "length": length})
                # Reset for new height
                current_height = height
                start_dist = dist

            # Handle the last period
            elif idx + 1 == len(measureNonNan):
                length = dist - start_dist
                periods.append({"height": current_height, "length": length})

        sumLen = sum(entry["length"] for entry in periods)
        frameFlat = flatFrame(periods)
        frameTot = entireFrame(periods)

        return sumLen, frameFlat, frameTot

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
        scan_three_d_compensation = float(
            config.get("scanThreeDDefault", config.get("model3D", {}).get("1", 0))
        )
        config["logger"].info(
            "[scan] Using model-agnostic scan profile: threshold=%s, min_stable_distance=%s, three_d=%s",
            scan_threshold,
            scan_min_stable_distance,
            scan_three_d_compensation,
        )

        msg_to_frontend(
            api_url=config["server"]["frontEnd_messaging_url"],
            message=f"Table Scanning Started...",
        )

        # tableState = "open"
        control_table(cps, tableState="open")

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
                    speed=config["UI"]["robotSpeed"],
                    wait=True,
                    require_seventh_ok=True,
                )
                if pre_scan_ok is None:
                    msg_to_frontend(
                        api_url=config["server"]["frontEnd_messaging_url"],
                        message="Scan aborted: failed to move J7 to starting position.",
                    )
                    return ([], [], [], [], [], [], [])

            for tblCnt, roboPos in enumerate(robo7thPos):
                if stop_requested():
                    msg_to_frontend(
                        api_url=config["server"]["frontEnd_messaging_url"],
                        message="Scan stopped by user.",
                    )
                    return ([], [], [], [], [], [], [])
                move_ok = communicate(
                    cps=cps,
                    seventh=roboPos,
                    tcp=config["coords"]["tcpLaserPlane1"],
                    ucs=config["coords"]["ucsTable1"],
                    config=config,
                    speed=config["UI"]["robotSpeed"],
                    require_seventh_ok=True,
                )
                if move_ok is None:
                    msg_to_frontend(
                        api_url=config["server"]["frontEnd_messaging_url"],
                        message=f"Scan aborted: failed to move J7 to table section {tblCnt + 1}.",
                    )
                    return ([], [], [], [], [], [], [])
                config["logger"].info(
                    f"[scan-x] success to move 7th to go to position for table: {tblCnt + 1}, position: {roboPos}"
                )
                msg_to_frontend(
                    api_url=config["server"]["frontEnd_messaging_url"],
                    message=f"Moving to Section {tblCnt + 1} of Table to Scan Horizontal...",
                )

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
                config["logger"].info(
                    f"[scan-x] points to move: <start>: {xStart} >>> <end>:{xEnd}"
                )

                communicate(
                    cps=cps,
                    point=xStart,
                    tcp=config["coords"]["tcpLaserPlane1"],
                    ucs=config["coords"]["ucsTable1"],
                    config=config,
                    speed=config["UI"]["robotSpeed"],
                    wait=False,
                )
                config["logger"].info(f"[scan-x] start point reached: {xStart}")

                waitForBlending(cps=cps, config=config)
                xmeasurements = communicate(
                    cps=cps,
                    point=xEnd,
                    tcp=config["coords"]["tcpLaserPlane1"],
                    ucs=config["coords"]["ucsTable1"],
                    config=config,
                    doMeasure=1,
                    speed=config["UI"]["scanSpeed"],
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
                communicate(
                    cps=cps,
                    point=xStart,
                    tcp=config["coords"]["tcpLaserPlane1"],
                    ucs=config["coords"]["ucsTable1"],
                    seventh=-1,
                    config=config,
                    speed=config["UI"]["robotSpeed"],
                    wait=False,
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
                    "xProfileType": x_profile_type,
                    "xPocketDetected": x_pocket_detected,
                    "xHeightRange": results.get("heightRange", 0),
                    "xClassificationReason": results.get("reason", ""),
                }
            )

            own7thpos.append(xpos)
            ymeasurements = []

            if config["settings"]["actualScan"]:
                if stop_requested():
                    msg_to_frontend(
                        api_url=config["server"]["frontEnd_messaging_url"],
                        message="Scan stopped by user.",
                    )
                    return ([], [], [], [], [], [], [])
                msg_to_frontend(
                    api_url=config["server"]["frontEnd_messaging_url"],
                    message=f"Moving to Door {len(allXMeasurements) - xcnt} to Scan Vertically...",
                )
                # It gets stuck here and returns code 20018
                # Adjust timer in IsBlendingDone at the top
                move_ok = communicate(
                    cps=cps,
                    seventh=xpos,
                    tcp=config["coords"]["tcpLaserPlane1"],
                    ucs=config["coords"]["ucsDefault"],
                    config=config,
                    speed=config["UI"]["robotSpeed"],
                    require_seventh_ok=True,
                )
                if move_ok is None:
                    msg_to_frontend(
                        api_url=config["server"]["frontEnd_messaging_url"],
                        message=f"Scan aborted: failed to move J7 to door position {len(allXMeasurements) - xcnt}.",
                    )
                    return ([], [], [], [], [], [], [])
                config["logger"].info(
                    f"[scan-y] success to move 7th to go to position for table: {tblCnt + 1}, position: {xpos}"
                )

                ###########################
                ####### y axis moving #####
                ###########################

                yStart = addYVal(
                    addXVal(config["point"]["table1Origin"], xlen / 4),
                    -config["offset"]["scannerOffsetInBottom"],
                )
                yEnd = addYVal(
                    addYVal(
                        addXVal(config["point"]["table1Origin"], xlen / 3),
                        config["table"]["width"],
                    ),
                    -config["offset"]["scannerOffsetInBottom"],
                )
                config["logger"].info(
                    f"[scan-y] points to move: <start>: {yStart} >>> <end>:{yEnd}"
                )

                communicate(
                    cps=cps,
                    point=yStart,
                    tcp=config["coords"]["tcpLaserPlane1"],
                    ucs=config["coords"]["ucsTable1"],
                    config=config,
                    speed=config["UI"]["robotSpeed"],
                    wait=False,
                )
                config["logger"].info(f"[scan-y] start point reached: {yStart}")

                waitForBlending(cps=cps, config=config)

                ymeasurements = communicate(
                    cps=cps,
                    point=yEnd,
                    tcp=config["coords"]["tcpLaserPlane1"],
                    ucs=config["coords"]["ucsTable1"],
                    config=config,
                    doMeasure=1,
                    speed=config["UI"]["scanSpeed"],
                    stopWhenNan=True,
                )
                config["logger"].info(f"[scan-y] end point reached: {yEnd}")
                print(f"scanSpeed: {config['UI']['scanSpeed']}")

                saveAsCSV(f"./static/prev_ym{xcnt}.csv", ymeasurements)
                ymeasurements = adjust_heights(ymeasurements)
                saveAsCSV(f"./static/ym{xcnt}.csv", ymeasurements)

                communicate(
                    cps=cps,
                    point=yStart,
                    tcp=config["coords"]["tcpLaserPlane1"],
                    ucs=config["coords"]["ucsTable1"],
                    seventh=-1,
                    config=config,
                    speed=config["UI"]["robotSpeed"],
                    wait=False,
                )

                msg_to_frontend(
                    api_url=config["server"]["frontEnd_messaging_url"],
                    message=f"Vertical Scanning of Door {len(allXMeasurements) - xcnt} Completed!",
                )
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
                min_stable_distance=0,
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
                    "yProfileType": y_profile_type,
                    "yPocketDetected": y_pocket_detected,
                    "yHeightRange": results.get("heightRange", 0),
                    "yClassificationReason": results.get("reason", ""),
                    "doorProfile": door_profile,
                }
            )

            config["logger"].info(
                f"[scan] x stuffs: <total length>: {xlen} mm, <left frame>: {xframe_1} mm, <right frame>: {xframe_2} mm"
            )
            config["logger"].info(
                f"[scan] y stuffs: <total length>: {ylen} mm, <left frame>: {yframe_1} mm, <right frame>: {yframe_2} mm"
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

            config["logger"].info(
                f"[scan-calculation] outer sanding points: {framePoints}"
            )
            config["logger"].info(
                f"[scan-calculation] edge  sanding points: {framePoints}"
            )
            config["logger"].info(
                f"[scan-calculation] inner sanding points: {pocketPoints}"
            )
            if config["settings"]["actualScan"]:
                # communicate(cps=cps, point=config['point']['safePoint'], tcp=config['coords']['tcpDefault'], ucs=config['coords']['ucsDefault'], seventh=-1, config=config, speed=config['UI']['robotSpeed'])
                communicate(
                    cps=cps,
                    point=config["point"]["safepointprehoming"],
                    tcp=config["coords"]["tcpLaserPlane1"],
                    ucs=config["coords"]["ucsTable1"],
                    seventh=-1,
                    config=config,
                    speed=config["UI"]["robotSpeed"],
                )
                config["logger"].info("[scan] moved successfully to safepoint")
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
            getTool(cps, toolNumber=config["tool"]["sideToolPos"], config=config)

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
            getTool(
                cps,
                toolNumber=config["tool"]["frameToolPos"],
                config=config,
                startFromSafe=not (doSideCycle or doOutEdgeCycle),
            )

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
            getTool(
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
            )

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
    wait=None,
    speed_mode="override",
    require_seventh_ok=False,
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

    def customMoveL(cps, point, tcp, ucs, config, speed, wait=True):
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

        velocity = config["coords"]["roboVelocity"]
        acceleration = config["coords"]["roboAcceleration"]
        if speed is not None:
            try:
                speed_value = float(speed)
            except (TypeError, ValueError):
                speed_value = None
            if speed_value is not None and speed_value > 0:
                velocity = speed_value * velocity
                acceleration = (
                    config["coords"]["roboAcceleration"]
                    * speed_value
                )

        if stop_requested():
            return
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
                # time.sleep(0.2)
            config["logger"].info(f"\n[customMoveL] Could move to {point}")
        else:
            config["logger"].error(f"\n[customMoveL] Couldn't move to {point}")
            msg_to_frontend(
                api_url=config["server"]["frontEnd_messaging_url"],
                message="Error In Moving! Please Check Door Size/Robot Settings and Try Again. Terminating Process...",
            )
            exit(-1)

    def data_collection(cps, config, stopWhenNan=False):
        measurements = []
        instrument = getInstrument()
        result = []
        nRet = cps.HRIF_ReadActPos(0, 0, result)
        initPos = [float(result[6]), float(result[7]), float(result[8])]
        keepgoing = True
        nan_count = 0
        # Define the consecutive NaN threshold
        consecutive_nan_threshold = 5
        startStopNan = False
        msg_to_frontend(
            api_url=config["server"]["frontEnd_messaging_url"],
            message=f"Scanning Door Using the Laser Sensor...⚙️",
        )

        while keepgoing:
            height = getRawHeight(instrument)

            # Read position and compute distance
            cps.HRIF_ReadActPos(0, 0, result)
            pos = [float(result[6]), float(result[7]), float(result[8])]
            # if config['settings']['debug']:
            #     config['logger'].info(f"pos: {pos}, height: {height}                       ")

            if not math.isnan(height):
                startStopNan = True
            # Calculate the distance from initial position
            dist = euclidean_distance(point1=initPos, point2=pos)

            # Append measurement
            measurements.append({"height": scale_value(height), "dist": dist})

            if stopWhenNan and startStopNan:
                # Check the height for NaN and update nan_count
                if math.isnan(height):
                    nan_count += 1  # Increment the NaN counter
                else:
                    nan_count = 0  # Reset the NaN counter if height is valid

                # Check if NaN count reaches the threshold
                if nan_count >= consecutive_nan_threshold:
                    keepgoing = False
                    nRet = cps.HRIF_GrpStop(0, 0)

            # Read robot state and check for another stop condition
            cps.HRIF_ReadRobotState(0, 0, result)
            if result[0] == "0":
                keepgoing = False

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

        def state_ok(state):
            return (
                isinstance(state, (list, tuple))
                and len(state) >= 3
                and str(state[0]) == "0"
            )

        pluginRes = []
        result = []

        # Read state first; only reconnect J7 if state read is unavailable.
        nret = cps.HRIF_HRApp(0, "HR_Motor", "MotorGetState", ["J7"], pluginRes)
        if (nret not in (0, None)) or not state_ok(pluginRes):
            result = []
            nret_conn = cps.HRIF_HRApp(0, "HR_Motor", "MotorConnect", ["J7"], result)
            if not move_ok(nret_conn, result):
                config["logger"].error(
                    f"[7thAxisMove] MotorConnect failed (ret={nret_conn}, res={result})."
                )
                msg_to_frontend(
                    api_url=config["server"]["frontEnd_messaging_url"],
                    message="7th axis connection failed. Please verify J7 and try again.",
                )
                return False
            # Short readiness poll (faster than fixed 0.2s x N sleeps).
            ready = False
            for _ in range(8):
                pluginRes = []
                nret = cps.HRIF_HRApp(0, "HR_Motor", "MotorGetState", ["J7"], pluginRes)
                if (nret in (0, None)) and state_ok(pluginRes):
                    ready = True
                    break
                time.sleep(0.02)
            if not ready:
                config["logger"].error(
                    f"[7thAxisMove] Failed to read J7 state before move (ret={nret}, res={pluginRes})."
                )
                msg_to_frontend(
                    api_url=config["server"]["frontEnd_messaging_url"],
                    message="7th axis state read failed. Please verify J7 connection and try again.",
                )
                return False

        if not state_ok(pluginRes):
            config["logger"].error(
                f"[7thAxisMove] Failed to read J7 state before move (ret={nret}, res={pluginRes})."
            )
            msg_to_frontend(
                api_url=config["server"]["frontEnd_messaging_url"],
                message="7th axis state read failed. Please verify J7 connection and try again.",
            )
            return False

        # if(config['settings']['debug']):
        #     config['logger'].info(f'motorList: {PosRes}')
        #     config['logger'].info(f'motorConnect: {result}')
        #     config['logger'].info(f'pluginRes: {pluginRes}')
        #     config['logger'].info(f'robotRes: {robotRes}')

        # input("ok?")

        max_move_retries = 8
        nret = None
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

            # Transient J7 readiness race: brief reconnect/readback and retry.
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
            config["logger"].error(
                f"[7thAxisMove] MotorMovePositionSpeed failed (ret={nret}, res={result})."
            )
            msg_to_frontend(
                api_url=config["server"]["frontEnd_messaging_url"],
                message="7th axis move failed. Please verify J7 and try again.",
            )
            return False
        config["logger"].info(
            f"[7thAxisMove] Going to position: {position}mm with speed: {speed}mm/s"
        )
        time.sleep(0.0001)

        while wait:
            if stop_requested():
                try:
                    cps.HRIF_HRApp(0, "HR_Motor", "MotorStop", ["J7"], [])
                except Exception:
                    pass
                return False
            pluginRes = []
            nret = cps.HRIF_HRApp(0, "HR_Motor", "MotorGetState", ["J7"], pluginRes)
            if (nret not in (0, None)) or len(pluginRes) < 3 or str(pluginRes[0]) != "0":
                config["logger"].error(
                    f"[7thAxisMove] MotorGetState failed (ret={nret}, res={pluginRes})."
                )
                msg_to_frontend(
                    api_url=config["server"]["frontEnd_messaging_url"],
                    message="7th axis state read failed during move. Please verify J7.",
                )
                return False

            # Faster state polling reduces visible handoff latency.
            time.sleep(0.005)
            if pluginRes[2] == "0":
                # means that the robot has reached the position
                break

        config["logger"].info(f"[7thAxisMove] Reached position: {position}mm")
        return True

    time.sleep(0.0001)

    measurements = []
    # speed_mode="override": <=1 uses override, >1 scales MoveL velocity.
    # speed_mode="linear": always scales MoveL velocity, no override changes.
    override_speed = None
    linear_speed = None
    if speed is not None:
        try:
            speed_value = float(speed)
        except (TypeError, ValueError):
            speed_value = None
        if speed_value is not None and speed_value > 0:
            if speed_mode == "override":
                if speed_value <= 1:
                    override_speed = speed_value
                else:
                    linear_speed = speed_value
            else:
                linear_speed = speed_value

    # setting the speed of the client (ratio override only)
    # Avoid extra override calls when doing seventh-axis-only repositioning.
    if override_speed is not None and not (point is None and seventh != -1):
        setSpeed(cps, override_speed, config=config)
    # time.sleep(0.1)

    if seventh != -1:
        if stop_requested():
            return measurements
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
        customMoveL(
            cps,
            point=point,
            tcp=tcp,
            ucs=ucs,
            speed=linear_speed,
            config=config,
            wait=wait if wait is not None else bool(1 - doMeasure),
        )

    # the list to collect values in
    if doMeasure:
        measurements = data_collection(cps, config, stopWhenNan)
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
    # required sleep (for proper functioning)
    time.sleep(0.5)
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
    time.sleep(0.5)


def getTool11(cps, toolNumber, config, startFromSafe=True):  # Tool postion dile nibe
    """_summary_

    Args:
        cps (CPSClient): cobot client
        toolNumber (int): The position from which to pick tool (out of 1-4)
        config (config): configuration info
        startFromSafe (bool, optional): If should go to the safe tool picking position or not. Make False if doing tool drop and pick one after another. Defaults to True.
    """
    if not config["settings"]["useTool"]:
        return
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
            speed=0.9,
            wait=True,
        )

    # go to that tool's home position (right above the tool)
    communicate(
        cps=cps,
        point=config["point"][f"tool{toolNumber}home"],
        tcp=config["coords"]["tcpDefault"],
        ucs=config["coords"]["ucsDefault"],
        seventh=-1,
        config=config,
        speed=0.9,
        wait=True,
    )
    # drop (for safety, to open the valve)
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
        wait=True,
    )
    # pick the tool
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
        wait=True,
    )


def keepTool11(cps, toolNumber, config, goToSafe=True):  # Tool Postion a rekhe dibe
    if not config["settings"]["useTool"]:
        return
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
        wait=True,
    )
    # drop the tool
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
        wait=True,
    )
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
            speed=0.9,
            wait=True,
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
        wait=False,
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
        wait=False,
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
        wait=False,
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
        wait=False,
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
        wait=False,
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
        wait=True,
    )


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
    waitForBlending(cps=cps, config=config)
    # connect the 7th axis motor
    nret = cps.HRIF_HRApp(0, "HR_Motor", "MotorConnect", ["J7"], result)
    time.sleep(0.2)
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
    nret = cps.HRIF_HRApp(0, "HR_Motor", "MotorStop", ["J7"], result)
    time.sleep(0.3)
    print(f"****** motor stop nret: {nret}; result: {result}")
    nret = cps.HRIF_HRApp(0, "HR_Motor", "MotorMoveOrigin", ["J7"], result)
    # seventhGoToPos(cpsclient, position=0, speed=config['7thAxis']['speed'] * config['cobot']['speed'], config=config)
    time.sleep(1)
    print(f"****** move origin nret: {nret}; result: {result}")
    result = [-1, -1, "-1"]
    # for waiting till the motor moves to the position
    while result[2] != "0":
        nret = cps.HRIF_HRApp(0, "HR_Motor", "MotorGetState", ["J7"], result)
        print(f"****** result: {result}")
        time.sleep(0.5)
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
    if table_id == "tableAOpenClose":
        if desired_state == "Open":
            # Set Table A to Open position (horizontal)
            nRet = CPS.HRIF_SetBoxDO(0, 1, 0)
            nRet = CPS.HRIF_SetBoxDO(0, 0, 1)

            # Verify the state with sensors
            di_state_0 = []
            di_state_1 = []
            nRet = CPS.HRIF_ReadBoxDI(0, 0, di_state_0)
            nRet = CPS.HRIF_ReadBoxDI(0, 1, di_state_1)

            if di_state_0[0] == "1" and di_state_1[0] == "0":
                return {
                    "success": True,
                    "newState": "Open",
                    "message": "Table A set to horizontal position",
                }
            else:
                return {
                    "success": False,
                    "newState": "Error",
                    "message": "Failed to set Table A to open position",
                }

        elif desired_state == "Close":
            # Set Table A to Close position (45 degrees)
            nRet = CPS.HRIF_SetBoxDO(0, 0, 0)
            nRet = CPS.HRIF_SetBoxDO(0, 1, 1)

            # Verify the state with sensors
            di_state_0 = []
            di_state_1 = []
            nRet = CPS.HRIF_ReadBoxDI(0, 0, di_state_0)
            nRet = CPS.HRIF_ReadBoxDI(0, 1, di_state_1)

            if di_state_0[0] == "0" and di_state_1[0] == "1":
                return {
                    "success": True,
                    "newState": "Close",
                    "message": "Table A set to 45 degree position - WARNING: Be careful when manually moving robot",
                }
            else:
                return {
                    "success": False,
                    "newState": "Error",
                    "message": "Failed to set Table A to close position",
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

            # Verify the state
            robot_state = []
            nRet = CPS.HRIF_ReadBoxCO(0, 1, robot_state)

            if robot_state[0] == "0":  # Assuming '0' means Open for Table B
                return {
                    "success": True,
                    "newState": "Open",
                    "message": "Table B set to horizontal position",
                }
            else:
                return {
                    "success": False,
                    "newState": "Error",
                    "message": "Failed to set Table B to open position",
                }

        elif desired_state == "Close":
            # Set Table B to Close position (45 degrees)
            nRet = CPS.HRIF_SetBoxCO(0, 0, 0)
            nRet = CPS.HRIF_SetBoxCO(0, 1, 1)

            # Verify the state
            robot_state = []
            nRet = CPS.HRIF_ReadBoxCO(0, 1, robot_state)

            if robot_state[0] == "1":  # Assuming '1' means Close for Table B
                return {
                    "success": True,
                    "newState": "Close",
                    "message": "Table B set to 45 degree position - WARNING: Be careful when manually moving robot",
                }
            else:
                return {
                    "success": False,
                    "newState": "Error",
                    "message": "Failed to set Table B to close position",
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

