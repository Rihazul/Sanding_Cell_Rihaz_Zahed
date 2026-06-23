from flask import Flask, jsonify, request, send_from_directory
from flask_cors import CORS
from threading import Thread
from Server_Better_V2 import handle_client, request_stop, clear_stop, stop_requested
from Server_Better_V2 import getTool11, keepTool11,communicate,laser,stopper_statusmod,set_table_state
from Server_Better_V2 import _get_tool_in_hand, _read_tool_sensors
from Server_Better_V2 import setup_logger
import yaml, requests
from modules.CPS import CPSClient, RbtClient, PluginClient
from multiprocessing import Process, Event, Pipe
import time
from flask_socketio import SocketIO
# Get the absolute path to flask_app.py (important when running as an executable)
import os
import sys
from werkzeug.utils import secure_filename
import logging
import threading
import importlib
from contextlib import contextmanager
import re
from collections import deque
from datetime import datetime

# Reduce noisy werkzeug request logs.
logging.getLogger("werkzeug").setLevel(logging.ERROR)

# for right table model plotting and sanding
from FileUtils.upload import upload3DModel
import json
import copy

# for left table homing and sanding total
# Heavy scan/homing modules are imported lazily so the desktop app can open faster.
from cycle_data_utils import overlap_mm_to_step
from tablea_task_worker import run_tablea_task_child


def _get_scan_table_a():
    from smallTable.scansmalltable import scanTableA
    return scanTableA


def _get_homingtotal():
    from smallTable.homingtotal import homingtotal
    return homingtotal

UPLOAD_FOLDER = './3DModels'
ALLOWED_EXTENSIONS = {'stp'}
os.makedirs(UPLOAD_FOLDER,exist_ok=True)

FLASK_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(FLASK_DIR, os.pardir))
REACT_BUILD_DIR = os.path.join(PROJECT_ROOT, "Create_Login_Dashboard_Analytics", "build")
REACT_ASSETS_DIR = os.path.join(REACT_BUILD_DIR, "assets")


#RightTable
modelMethodmap = {
    "modelA": ("model1cycle.mainmodel1", "startingRobotToSandmodel1"),
    "modelB": ("model2cycle.mainmodel2", "startingRobotToSandmodel2"),
    "modelC": ("model3cycle.mainmodel3", "startingRobotToSandmodel3"),
    "modelD": ("model4cycle.mainmodel4", "startingRobotToSandmodel4"),
    "modelE": ("model5cycle.mainmodel5", "startingRobotToSandmodel5"),
}
#RightTable
modelMap = {
    "modelA": ("Table1Model.plotexp", "plot_data"),
    "modelB": ("Table2Model.plotexp", "plot_data"),
    "modelC": ("Table3Model.plotexp", "plot_data"),
    "modelD": ("Table4Model.plotexp", "plot_data"),
    "modelE": ("Table5Model.plotexp", "plot_data"),
}


def _load_function(module_name, function_name):
    module = importlib.import_module(module_name)
    return getattr(module, function_name)


def _run_tableb_plot_dialog(model_key, inverse_overlapping, conn):
    """
    Run the matplotlib Start/Cancel dialog in a dedicated process so GUI code
    executes on that process's main thread (avoids tkinter thread errors).
    """
    try:
        module_name, function_name = modelMap[model_key]
        plot_fn = _load_function(module_name, function_name)
        action = plot_fn(inverseOverlapping=inverse_overlapping)
        conn.send({"ok": True, "action": action})
    except Exception as e:
        conn.send({"ok": False, "error": str(e)})
    finally:
        try:
            conn.close()
        except Exception:
            pass


def _run_tableb_task_child(model_key):
    module_name, function_name = modelMethodmap[model_key]
    task_fn = _load_function(module_name, function_name)
    task_fn()

def allowed_file(filename):
    return '.' in filename and filename.rsplit('.', 1)[1].lower() in ALLOWED_EXTENSIONS


def _scan_indicates_model_f():
    scan_path = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "smallTable",
        "static",
        "scan_results.json",
    )
    try:
        with open(scan_path, "r") as file:
            data = json.load(file)
    except Exception:
        return False

    door_profiles = data.get("doorProfiles")
    if not isinstance(door_profiles, list) or not door_profiles:
        return False

    considered = 0
    for entry in door_profiles:
        if not isinstance(entry, dict):
            continue
        profile = entry.get("profile")
        if profile is None:
            continue
        considered += 1
        if profile != "uniform_depth_no_pocket":
            return False

    if considered == 0:
        return False
    return True


def _build_tablea_signature(table_data):
    if not isinstance(table_data, dict):
        return ""
    base_model = str(table_data.get("model", "") or "").strip()
    door_parts = []
    doors = table_data.get("doors")
    if isinstance(doors, list):
        for door in doors:
            if not isinstance(door, dict):
                continue
            door_num = door.get("doorNumber")
            door_model = str(door.get("model", "") or "").strip()
            door_parts.append(f"{door_num}:{door_model}")
    door_sig = "|".join(sorted(door_parts))
    return f"{base_model}::{door_sig}"


def _zero_task_payload(task_cfg):
    """Normalize a task payload as disabled."""
    if not isinstance(task_cfg, dict):
        return {"cycle": 0, "force": 0, "doors": []}
    task_cfg["cycle"] = 0
    task_cfg["force"] = 0
    if "doors" in task_cfg:
        task_cfg["doors"] = []
    return task_cfg


def _enforce_model_f_tablea_payload(table_data):
    """
    Backend safety gate for Model F:
    - only pocketzigzag is allowed
    - all other tasks are disabled server-side
    - per-door model is forced to modelF
    """
    if not isinstance(table_data, dict):
        return

    table_data["model"] = "modelF"

    disallowed_legacy_tasks = ["frame", "pocketsquare", "3D", "edgeInside", "edgeOutside", "side"]
    for task_key in disallowed_legacy_tasks:
        table_data[task_key] = _zero_task_payload(table_data.get(task_key))

    pocket_cfg = table_data.get("pocketzigzag")
    if not isinstance(pocket_cfg, dict):
        table_data["pocketzigzag"] = {"cycle": 0, "force": 0, "doors": []}

    doors = table_data.get("doors")
    if not isinstance(doors, list):
        return

    for door_entry in doors:
        if not isinstance(door_entry, dict):
            continue
        door_entry["model"] = "modelF"
        tasks = door_entry.get("tasks")
        if not isinstance(tasks, dict):
            continue
        for key in ["frame", "pocketsquare", "3D", "edgeInside", "edgeOutside", "side"]:
            tasks[key] = _zero_task_payload(tasks.get(key))


LOG_LINE_RE = re.compile(
    r"^(?P<ts>\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2},\d{3}) - (?P<level>[A-Z]+) - .* - (?P<msg>.*)$"
)


def _read_last_lines(path, max_lines=2000):
    try:
        with open(path, "r", encoding="utf-8", errors="replace") as f:
            return list(deque(f, maxlen=max_lines))
    except Exception:
        return []


def _level_to_type(level: str) -> str:
    lvl = (level or "").upper()
    if lvl in {"ERROR", "CRITICAL"}:
        return "error"
    if lvl in {"WARNING", "WARN"}:
        return "warning"
    if lvl in {"INFO"}:
        return "success"
    return "info"


def _format_display_date(date_key: str) -> str:
    try:
        dt = datetime.strptime(date_key, "%Y-%m-%d")
        return f"{dt.strftime('%B')} {dt.day}, {dt.year}"
    except Exception:
        return date_key

############################################################################################
client_process = None
client_thread = None
app = Flask(__name__)
CORS(app)
socketio = SocketIO(app, async_mode='threading')

######################################################f######################################
default_data = {
    "tableA": {
        "UI": {
            "frameSandCount": 0.0,
            "pocketZigSandCnt": 0.0,
            "pocketSQSandCnt": 0.0,
            "threeDSandCount": 0.0,
            "inedgeSandCount": 0.0,
            "outedgeSandCount": 0.0,
            "sideSandCount": 0.0,
            "robotSpeed": 0.90,
            "scanSpeed": 0.15, # Increased from 0.15 to 0.25 by rafat
            "seventhAxisSpeed": 50,
            "seventhAxisVelocity": 20,
            "model": "1",  # Assuming default model is "1"
            "controlPressure": 0.0,
            "frameForce": 1,
            "pocketZigForce": 1,
            "pocketSQForce": 1,
            "threeDForce": 1,
            "inedgeForce": 1,
            "outedgeForce": 1,
            "sideForce": 1,
        }
    }
}

# Duplicate tableA data in tableB since these are the default data
default_data["tableB"] = default_data["tableA"]

############################################################################################
CPS = CPSClient()
IP = "192.168.0.10"
port = 10003
# Lazy-connect CPS from endpoints/actions instead of blocking Flask startup.
# Child workers create/connect their own local CPS clients.
ret = None
Tpos = 0
velocity = 0.1
robot_lock = threading.Lock()
CPS_RECONNECT_GRACE_SECONDS = 1.5
CPS_RECONNECT_MIN_INTERVAL_SECONDS = 1.0
STOP_JOIN_TIMEOUT_SECONDS = 0.8
STOP_KILL_JOIN_TIMEOUT_SECONDS = 0.4
POST_STOP_GRACE_SECONDS = 1.0
STOP_TO_HOMING_MIN_GAP_SECONDS = 0.15
STOP_TO_HOMING_PROBE_TIMEOUT_SECONDS = 0.6
_cps_reconnect_grace_until = 0.0
_cps_last_connect_attempt = 0.0
_last_stop_ts = 0.0
_inline_scan_active = threading.Event()
_last_child_exit_ts = 0.0
CHILD_EXIT_SETTLE_SECONDS = 0.6
J7_HOME_STATE_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), ".j7_home_state.json")
TABLEA_SCAN_RESULTS_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "smallTable",
    "static",
    "scan_results.json",
)
TABLEA_SCAN_META_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    ".tablea_scan_meta.json",
)


def _load_j7_home_state() -> bool:
    try:
        if not os.path.exists(J7_HOME_STATE_PATH):
            return False
        with open(J7_HOME_STATE_PATH, "r", encoding="utf-8") as f:
            payload = json.load(f)
        return bool(payload.get("homed", False))
    except Exception:
        return False


def _persist_j7_home_state(is_homed: bool) -> None:
    try:
        payload = {
            "homed": bool(is_homed),
            "updatedAt": datetime.now().isoformat(timespec="seconds"),
        }
        with open(J7_HOME_STATE_PATH, "w", encoding="utf-8") as f:
            json.dump(payload, f)
    except Exception:
        pass


def _load_tablea_scan_meta() -> dict:
    try:
        if not os.path.exists(TABLEA_SCAN_META_PATH):
            return {}
        with open(TABLEA_SCAN_META_PATH, "r", encoding="utf-8") as f:
            payload = json.load(f)
        return payload if isinstance(payload, dict) else {}
    except Exception:
        return {}


def _save_tablea_scan_meta(payload: dict) -> None:
    try:
        with open(TABLEA_SCAN_META_PATH, "w", encoding="utf-8") as f:
            json.dump(payload, f)
    except Exception:
        pass


def _set_tablea_scan_meta(signature=None, model=None, door_models=None):
    payload = {
        "hasScan": True,
        "scannedAt": datetime.now().isoformat(timespec="seconds"),
        "signature": signature or "",
        "model": model or "",
        "doorModels": door_models or [],
    }
    _save_tablea_scan_meta(payload)


def _get_detected_tablea_door_numbers():
    """Return physical door slots explicitly reported by the latest scan."""
    try:
        with open(TABLEA_SCAN_RESULTS_PATH, "r", encoding="utf-8") as f:
            scan_data = json.load(f)
    except Exception:
        return None

    physical_numbers = scan_data.get("physicalDoorNumbers")
    if isinstance(physical_numbers, list):
        detected = []
        for value in physical_numbers:
            try:
                door_number = int(value)
            except (TypeError, ValueError):
                continue
            if 1 <= door_number <= 4:
                detected.append(door_number)
        if detected:
            return sorted(set(detected))

    door_profiles = scan_data.get("doorProfiles")
    if isinstance(door_profiles, list):
        detected = []
        for profile in door_profiles:
            if not isinstance(profile, dict):
                continue
            try:
                door_number = int(profile.get("doorNumber"))
            except (TypeError, ValueError):
                continue
            if 1 <= door_number <= 4:
                detected.append(door_number)
        if detected:
            return sorted(set(detected))

    if isinstance(physical_numbers, list):
        return []

    return None


def _get_requested_tablea_task_doors(table_data):
    """Return door slots with at least one requested task cycle."""
    requested = set()
    doors = table_data.get("doors") if isinstance(table_data, dict) else None
    if not isinstance(doors, list):
        return requested

    for door_entry in doors:
        if not isinstance(door_entry, dict):
            continue
        try:
            door_number = int(door_entry.get("doorNumber"))
        except (TypeError, ValueError):
            continue
        tasks = door_entry.get("tasks")
        if not isinstance(tasks, dict):
            continue
        for task in tasks.values():
            if not isinstance(task, dict):
                continue
            try:
                cycle = int(task.get("cycle", 0) or 0)
            except (TypeError, ValueError):
                cycle = 0
            if cycle > 0:
                requested.add(door_number)
                break

    return requested


def _get_tablea_scan_status():
    meta = _load_tablea_scan_meta()
    has_scan_file = os.path.exists(TABLEA_SCAN_RESULTS_PATH)
    has_scan = bool(has_scan_file or meta.get("hasScan"))
    scan_revision = None
    if has_scan_file:
        try:
            scan_revision = os.path.getmtime(TABLEA_SCAN_RESULTS_PATH)
        except Exception:
            scan_revision = None
    if not has_scan:
        return {
            "hasScan": False,
            "scannedAt": None,
            "signature": "",
            "model": "",
            "doorModels": [],
            "detectedDoorNumbers": [],
            "doorDetectionAvailable": False,
            "isScanning": _inline_scan_active.is_set(),
            "scanRevision": scan_revision,
        }
    if not meta.get("scannedAt"):
        try:
            meta["scannedAt"] = datetime.fromtimestamp(scan_revision).isoformat(timespec="seconds")
        except Exception:
            meta["scannedAt"] = None
    detected_door_numbers = _get_detected_tablea_door_numbers()
    return {
        "hasScan": True,
        "scannedAt": meta.get("scannedAt"),
        "signature": meta.get("signature", ""),
        "model": meta.get("model", ""),
        "doorModels": meta.get("doorModels", []),
        "detectedDoorNumbers": detected_door_numbers or [],
        "doorDetectionAvailable": detected_door_numbers is not None,
        "isScanning": _inline_scan_active.is_set(),
        "scanRevision": scan_revision,
    }


def _tablea_scan_recent(scan_status, max_age_seconds=180):
    scanned_at = scan_status.get("scannedAt") if isinstance(scan_status, dict) else None
    if not scanned_at:
        return False
    try:
        scanned_dt = datetime.fromisoformat(str(scanned_at))
    except Exception:
        return False
    return (datetime.now() - scanned_dt).total_seconds() <= max_age_seconds


def _sanitize_cps_runtime():
    """Keep CPS runtime internals coherent when transport sockets are stale."""
    try:
        client = CPS.g_clients[0]
    except Exception:
        return

    # If fast socket is gone but fast command cache remains, force standard socket path.
    try:
        if getattr(client, "tcp_fast", None) is None and isinstance(client.fast_cmd_list, list):
            client.fast_cmd_list.clear()
    except Exception:
        pass


def _hard_reset_cps_runtime():
    """Reset CPS runtime objects without changing CPS.py implementation."""
    global CPS, _cps_last_connect_attempt
    try:
        CPS.HRIF_DisConnect(0)
    except Exception:
        pass
    try:
        CPS.g_clients[0] = RbtClient()
        CPS.g_plugin_clients[0] = PluginClient()
        CPS.g_client_state[0] = False
        CPS.g_plugin_client_state[0] = False
    except Exception:
        # Fallback: rebuild wrapper object if direct reset is unavailable.
        CPS = CPSClient()
        try:
            CPS.g_clients[0] = RbtClient()
            CPS.g_plugin_clients[0] = PluginClient()
            CPS.g_client_state[0] = False
            CPS.g_plugin_client_state[0] = False
        except Exception:
            pass
    _cps_last_connect_attempt = 0.0


def ensure_cps_connected(force=False):
    """Ensure the global CPS client is connected before issuing commands."""
    global _cps_last_connect_attempt
    now = time.monotonic()
    if not force:
        if now < _cps_reconnect_grace_until:
            return None
        if (now - _cps_last_connect_attempt) < CPS_RECONNECT_MIN_INTERVAL_SECONDS:
            return None
    _cps_last_connect_attempt = now
    _sanitize_cps_runtime()
    try:
        if CPS.HRIF_IsConnected(0):
            client = CPS.g_clients[0]
            if getattr(client, "tcp", None) is not None:
                # Probe the connection with a lightweight read to avoid stale sockets.
                probe = []
                ret = CPS.HRIF_ReadRobotState(0, 0, probe)
                if ret == 0 and isinstance(probe, (list, tuple)) and len(probe) > 0:
                    _cps_last_connect_attempt = 0.0
                    return 0
    except Exception:
        pass
    ret = CPS.HRIF_Connect(0, IP, port)
    if ret != 0:
        return ret
    _sanitize_cps_runtime()
    # Verify that a fresh connect is actually usable before reporting success.
    probe = []
    ret = CPS.HRIF_ReadRobotState(0, 0, probe)
    if ret == 0 and isinstance(probe, (list, tuple)) and len(probe) > 0:
        _cps_last_connect_attempt = 0.0
        return 0
    # Treat empty success payloads as transient/unhealthy and retry at caller level.
    if ret == 0:
        return None
    return ret


def ensure_cps_connected_retry(max_attempts=6, retry_delay_seconds=0.5, force=True):
    """Retry CPS reconnection and return the latest robot error code."""
    last_ret = None
    last_robot_error = None
    hard_reset_done = False
    for _ in range(max_attempts):
        last_ret = ensure_cps_connected(force=force)
        if last_ret == 0:
            return 0
        if last_ret is not None:
            last_robot_error = last_ret
        if force and not hard_reset_done:
            _hard_reset_cps_runtime()
            hard_reset_done = True
        time.sleep(retry_delay_seconds)
    return last_robot_error if last_robot_error is not None else last_ret


@contextmanager
def locked_cps(timeout=1.0, allow_when_busy=False):
    """Lock CPS access and ensure connection; yields True on success."""
    if not allow_when_busy and process_state.get("status") == "in_progress":
        yield False
        return
    acquired = robot_lock.acquire(timeout=timeout)
    if not acquired:
        yield False
        return
    ok = False
    try:
        ret = ensure_cps_connected()
        ok = ret == 0
        yield ok
    finally:
        if acquired:
            robot_lock.release()

############################################################################################
# Store modal data and process state globally
modal_data_store = {}
process_state = {
    'status': 'completed',  # Can be 'in_progress' or 'completed'
    'last_action': None,
}
tool_override_state = {1: False, 2: False, 3: False, 4: False}
# Require homing after every app/server restart.
# Do not trust persisted state across process restarts.
j7_home_confirmed = False
_persist_j7_home_state(False)


def _set_j7_home_confirmed(value: bool) -> None:
    global j7_home_confirmed
    j7_home_confirmed = bool(value)
    _persist_j7_home_state(j7_home_confirmed)


def _probe_j7_state():
    """
    Return J7 motion state string from plugin, or None when unavailable.
    Common values:
    - "0": idle
    - "1": moving
    - "-1": disconnected/uninitialized
    """
    # Never send J7 plugin commands from the Flask parent while a child
    # operation owns the robot. MotorConnect/GetState from this process can
    # interfere with an active MotorMovePositionSpeed command in the child.
    if process_state.get("status") == "in_progress":
        return None

    with robot_lock:
        ret = ensure_cps_connected(force=False)
        if ret != 0:
            return None
        result = []
        nret = CPS.HRIF_HRApp(0, "HR_Motor", "MotorGetState", ["J7"], result)
        if (nret not in (0, None)) or not isinstance(result, (list, tuple)) or len(result) < 3:
            return None
        if str(result[0]).strip() != "0":
            return None
        return str(result[2]).strip()



def _track_process(proc: Process) -> None:
    """Background watcher to clear state when a spawned process finishes."""
    try:
        proc.join()
    finally:
        global client_process, _cps_reconnect_grace_until, _last_child_exit_ts, j7_home_confirmed
        now = time.monotonic()
        _last_child_exit_ts = now
        # Brief cooldown after child process exit to avoid hitting stale CPS sockets.
        _cps_reconnect_grace_until = now + max(CPS_RECONNECT_GRACE_SECONDS, POST_STOP_GRACE_SECONDS)

        # Only clear process_state/client_process if this watcher belongs to the current process slot.
        if client_process is proc:
            exit_code = proc.exitcode
            process_state['status'] = 'completed' if exit_code == 0 else 'failed'
            if exit_code == 0 and process_state.get('last_action') == 'homing':
                _set_j7_home_confirmed(True)
            process_state['last_action'] = None
            client_process = None
            if exit_code == 0:
                socketio.emit('flash_message', {"message": "Process finished"})
            else:
                socketio.emit('flash_message', {"message": f"Process failed (exit={exit_code})"})

        # Parent keeps a global CPS wrapper; reset it after child exits to avoid stale handles.
        with robot_lock:
            _hard_reset_cps_runtime()


def _run_homing_child(config_data_UI):
    """Run homing in a fresh child process to avoid stale CPS sockets."""
    cps = None
    try:
        if "logger" not in config_data_UI:
            config_data_UI["logger"] = setup_logger(config_data_UI["settings"]["debug"])
        cps = CPSClient()
        ret = cps.HRIF_Connect(0, config_data_UI["server"]["cpip"], config_data_UI["server"]["cps"])
        if ret != 0:
            config_data_UI["logger"].error(f"[homing child] HRIF_Connect failed (ret={ret})")
            raise RuntimeError(f"Homing connection failed (ret={ret})")
        homing_ok = handle_client(config_data_UI, homingState=True, startSanding=False, cps=cps)
        if not homing_ok:
            raise RuntimeError("Homing sequence did not complete successfully")
        # Optional legacy follow-up; disabled by default because handle_client(homingState=True)
        # already performs homing, and the extra move can cause unsafe transitions.
        if bool(config_data_UI.get("settings", {}).get("runHomingTotalAfterHandleClient", False)):
            _get_homingtotal()(cps=cps, config=config_data_UI)
    except Exception as e:
        try:
            config_data_UI["logger"].error(f"[homing child] Exception: {e}")
        except Exception:
            print(f"[homing child] Exception: {e}")
        raise
    finally:
        if cps is not None:
            try:
                cps.HRIF_DisConnect(0)
            except Exception:
                pass


def _run_homing_inline(config_data_UI):
    """Run homing in-process on the parent CPS connection (fast path)."""
    global client_thread, j7_home_confirmed
    homing_ok = False
    try:
        if "logger" not in config_data_UI:
            config_data_UI["logger"] = setup_logger(config_data_UI["settings"]["debug"])

        with robot_lock:
            ret = ensure_cps_connected(force=True)
            if ret != 0:
                config_data_UI["logger"].error(f"[homing inline] CPS not ready (ret={ret})")
                return
            homing_ok = bool(
                handle_client(config_data_UI, homingState=True, startSanding=False, cps=CPS)
            )
            if not homing_ok:
                raise RuntimeError("Homing sequence did not complete successfully")

            # Optional legacy follow-up; preserve behavior parity with child path.
            if bool(config_data_UI.get("settings", {}).get("runHomingTotalAfterHandleClient", False)):
                _get_homingtotal()(cps=CPS, config=config_data_UI)
    except Exception as e:
        try:
            config_data_UI["logger"].error(f"[homing inline] Exception: {e}")
        except Exception:
            print(f"[homing inline] Exception: {e}")
    finally:
        if homing_ok and process_state.get("last_action") == "homing":
            _set_j7_home_confirmed(True)
        process_state["status"] = "completed" if homing_ok else "failed"
        process_state["last_action"] = None
        client_thread = None
        socketio.emit(
            'flash_message',
            {"message": "Homing completed" if homing_ok else "Homing failed"},
        )


def _parent_cps_healthy_for_homing():
    """Fast health probe to decide if inline homing can safely reuse parent CPS."""
    with robot_lock:
        ret = ensure_cps_connected(force=False)
        if ret != 0:
            return False
        state = []
        nret = CPS.HRIF_ReadRobotState(0, 0, state)
        if nret != 0 or not isinstance(state, (list, tuple)) or len(state) < 13:
            return False
        # Avoid inline path when robot reports error / emergency-stop.
        if state[2] == "1" or state[7] == "1":
            return False
    return True

############################################################################################
# Home route to render the Vite/React frontend.
@app.route('/')
def interface():
    return send_from_directory(REACT_BUILD_DIR, 'index.html')


@app.route('/assets/<path:filename>')
def react_assets(filename):
    return send_from_directory(REACT_ASSETS_DIR, filename)


@app.route('/table_1/<path:filename>')
def react_table_1_images(filename):
    return send_from_directory(os.path.join(REACT_BUILD_DIR, 'table_1'), filename)


@app.route('/table_2/<path:filename>')
def react_table_2_images(filename):
    return send_from_directory(os.path.join(REACT_BUILD_DIR, 'table_2'), filename)

############################################################################################
# Display the sanding state in the frontend.
@app.route('/trigger', methods=['POST'])
def trigger():
    # Parse the JSON payload
    data = request.json
    if not data:
        return jsonify({"error": "Invalid request, no JSON data found"}), 400

    # Extract the message
    message = data.get('message', 'No message provided')

    # Send the message to all connected WebSocket clients
    socketio.emit('flash_message', {"message": message})

    return jsonify({"status": "Message sent"}), 200

############################################################################################
# Save modal data
@app.route('/save_modal_data', methods=['POST'])
def save_modal_data():
    global modal_data_store
    try:
        modal_data = request.json  # Get modal data from frontend
        modal_data_store = modal_data  # Store it in memory
        return jsonify({'status': 'success', 'message': 'Modal data saved successfully'})
    except Exception as e:
        return jsonify({'status': 'error', 'message': f'Error saving modal data: {str(e)}'})

############################################################################################
# Get modal data
@app.route('/get_modal_data', methods=['GET'])
def get_modal_data():
    if modal_data_store:
        return jsonify(modal_data_store)
    else:
        return jsonify(default_data)


@app.route('/logs/history', methods=['GET'])
def get_logs_history():
    """
    Returns historical log entries grouped by date from ./logs/app.log* on the server machine.
    Query params:
      - days (default 14, max 60) [ignored when all=true]
      - per_file_lines (default 2000, max 200000)
      - all (default false): when true, include all days/files in folder
    """
    try:
        days = int(request.args.get('days', 14))
    except Exception:
        days = 14
    days = max(1, min(days, 60))

    try:
        per_file_lines = int(request.args.get('per_file_lines', 2000))
    except Exception:
        per_file_lines = 2000
    per_file_lines = max(200, min(per_file_lines, 200000))

    include_all = str(request.args.get('all', 'false')).strip().lower() in {'1', 'true', 'yes', 'y', 'on'}

    logs_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")
    if not os.path.isdir(logs_dir):
        return jsonify({"logs": [], "source": logs_dir, "message": "logs directory not found"})

    files = []
    for name in os.listdir(logs_dir):
        if name.startswith("app.log"):
            full_path = os.path.join(logs_dir, name)
            if os.path.isfile(full_path):
                files.append(full_path)
    files.sort(key=lambda p: os.path.getmtime(p), reverse=True)

    grouped = {}
    entry_id = 1
    cutoff_date = datetime.now().date()

    for path in files:
        for line in _read_last_lines(path, per_file_lines):
            m = LOG_LINE_RE.match(line.strip())
            if not m:
                continue
            try:
                ts = datetime.strptime(m.group("ts"), "%Y-%m-%d %H:%M:%S,%f")
            except Exception:
                continue
            age_days = (cutoff_date - ts.date()).days
            if not include_all and (age_days < 0 or age_days >= days):
                continue

            date_key = ts.strftime("%Y-%m-%d")
            grouped.setdefault(date_key, [])
            grouped[date_key].append({
                "id": entry_id,
                "timestamp": ts.strftime("%H:%M:%S"),
                "message": m.group("msg"),
                "type": _level_to_type(m.group("level")),
            })
            entry_id += 1

    daily_logs = []
    for date_key, entries in grouped.items():
        entries.sort(key=lambda e: e["timestamp"], reverse=False)
        daily_logs.append({
            "date": date_key,
            "displayDate": _format_display_date(date_key),
            "entries": entries,
        })

    daily_logs.sort(key=lambda d: d["date"], reverse=True)
    return jsonify({
        "logs": daily_logs,
        "source": logs_dir,
        "includeAll": include_all,
        "days": days,
        "filesScanned": len(files),
    })

############################################################################################

@app.route('/start_TableB_process', methods=['POST'])
def start_TableB_process():
    global client_process
    global j7_home_confirmed
    data = request.json

    if (client_process and client_process.is_alive()) or (client_thread and client_thread.is_alive()):
        return jsonify({'status': 'error', 'message': 'Process already running'})

    if not data:
        return jsonify({"error": "Invalid request, no JSON data found"}), 400

    if 'TableB' not in data:
        return jsonify({"error": "Invalid request, no JSON data found"}), 400

    tableData = data['TableB']
    selected_model = tableData.get('model')

    if not selected_model:
        return jsonify({"error": "Invalid request, model is required for TableB"}), 400
    if selected_model not in modelMap or selected_model not in modelMethodmap:
        valid_models = sorted(set(modelMap.keys()).intersection(set(modelMethodmap.keys())))
        return jsonify({
            "error": f"Invalid model '{selected_model}' for TableB",
            "validModels": valid_models
        }), 400

    inverse_overlapping = data.get('inverseOverlapping', 0)
    inverse_overlapping_step = overlap_mm_to_step(inverse_overlapping)
    parent_conn, child_conn = Pipe(duplex=False)
    plot_process = Process(
        target=_run_tableb_plot_dialog,
        args=(selected_model, inverse_overlapping_step, child_conn),
    )
    plot_process.start()
    child_conn.close()
    plot_process.join()

    if not parent_conn.poll():
        return jsonify({"error": "Plot dialog did not return an action"}), 500

    plot_result = parent_conn.recv()
    if not plot_result.get("ok", False):
        return jsonify({"error": f"Failed to run plot dialog: {plot_result.get('error', 'unknown error')}"}), 500

    buttonClicked = plot_result.get("action", "cancel")
    with open('./configs/cycleData.json', 'w') as f:
        json.dump(data, f)

    if buttonClicked == "cancel":
        return jsonify({
            'success': False,
            'process' : "cancelled",
            "status": "Process is cancelled"
        })

    clear_stop()
    with robot_lock:
        conn_ret = ensure_cps_connected(force=True)
        if conn_ret != 0:
            return jsonify({"error": f"Failed to connect to CPS client (ret={conn_ret})"}), 500
        # Operation routing uses swapped IO table IDs on this cell:
        # selecting Table B task should physically open Table B and close Table A.
        set_table_state(CPS, "tableAOpenClose", "Open")
        set_table_state(CPS, "tableBOpenClose", "Close")
        socketio.emit('flash_message', {"message": "Operation table mode: Table B Open, Table A Close"})
        nRet = CPS.HRIF_SetBoxCO(0, 2, 0)
    print("stopper Test")

    # Disconnect/reset parent CPS before child opens its own CPS session.
    _disconnect_global_cps_for_child_start()
    client_process = Process(target=_run_tableb_task_child, args=(selected_model,))
    process_state['status'] = 'in_progress'
    client_process.start()
    Thread(target=_track_process, args=(client_process,), daemon=True).start()

    # startingRobotToSand()
    return jsonify({
        'success': True,
        'process' : "started",
        "status": "Process started"
    })

############################################################################################

@app.route('/start_TableA_process', methods=['POST'])
def start_TableA_process():
    global client_process, client_thread
    global j7_home_confirmed
    data = request.json
    route_start = time.monotonic()
    route_timings = {}
    print("Received data in start_TableA_process:", data)

    if (client_process and client_process.is_alive()) or (client_thread and client_thread.is_alive()):
        return jsonify({'status': 'error', 'message': 'Process already running'})

    if not data:
        return jsonify({"error": "Invalid request, no JSON data found"}), 400

    if 'TableA' not in data:
        return jsonify({"error": "Invalid request, no JSON data found"}), 400

    tableData = data['TableA']
    step_start = time.monotonic()
    scan_status = _get_tablea_scan_status()
    route_timings["scanStatusSeconds"] = time.monotonic() - step_start
    if not scan_status.get("hasScan"):
        return jsonify(
            {
                "error": "Scan required before starting Table A task.",
                "code": "scan_required",
            }
        ), 409

    if scan_status.get("doorDetectionAvailable"):
        detected_doors = set(scan_status.get("detectedDoorNumbers") or [])
        requested_doors = _get_requested_tablea_task_doors(tableData)
        unavailable_doors = sorted(requested_doors - detected_doors)
        if unavailable_doors:
            return jsonify(
                {
                    "error": (
                        "Task includes doors not detected in the latest scan: "
                        + ", ".join(str(door) for door in unavailable_doors)
                    ),
                    "code": "undetected_doors_selected",
                    "detectedDoorNumbers": sorted(detected_doors),
                    "invalidDoorNumbers": unavailable_doors,
                }
            ), 409

    # Support both payload formats:
    # 1) Legacy: TableA.model + task objects
    # 2) Door-based: TableA.doors[] where each door can include model + task objects
    selected_model = tableData.get('model')
    if not selected_model or selected_model == 'None':
        doors = tableData.get('doors')
        if isinstance(doors, list) and doors:
            # Infer the model from the first door that has one.
            selected_model = next((d.get('model') for d in doors if isinstance(d, dict) and d.get('model')), None)
            # If the frontend sends model keys like modelA/modelB..., use that.
        if not selected_model or selected_model == 'None':
            if _scan_indicates_model_f():
                selected_model = "modelF"
            else:
                return jsonify({"error": "Invalid request, no model found for TableA"}), 400
        # Make it available to any downstream code that still expects TableA.model
        tableData['model'] = selected_model

    current_signature = _build_tablea_signature(tableData)
    previous_signature = str(scan_status.get("signature") or "").strip()
    if previous_signature and current_signature and previous_signature != current_signature:
        return jsonify(
            {
                "error": "Scan no longer matches current Table A model/door selection. Please scan again.",
                "code": "scan_mismatch",
                "scanSignature": previous_signature,
                "currentSignature": current_signature,
            }
        ), 409
    recent_matching_scan = bool(
        previous_signature
        and current_signature
        and previous_signature == current_signature
        and _tablea_scan_recent(scan_status)
    )

    step_start = time.monotonic()
    runtime_config = load_config()
    route_timings["loadConfigSeconds"] = time.monotonic() - step_start
    auto_model_f_from_scan = bool(
        runtime_config.get("settings", {}).get("autoModelFFromScan", False)
    )
    if auto_model_f_from_scan and selected_model != "modelF" and _scan_indicates_model_f():
        print("Scan indicates no pocket; switching to modelF (autoModelFFromScan=True).")
        selected_model = "modelF"
        tableData["model"] = selected_model

    if selected_model == "modelF":
        print("Applying backend Model F guard: only pocketzigzag is allowed; forcing Tool 4 workflow.")
        _enforce_model_f_tablea_payload(tableData)

    step_start = time.monotonic()
    with open('./configs/cycleData.json', 'w') as f:
        json.dump(data, f)
    route_timings["cycleDataWriteSeconds"] = time.monotonic() - step_start

    step_start = time.monotonic()
    clear_stop()
    route_timings["clearStopSeconds"] = time.monotonic() - step_start
    launch_start = time.monotonic()
    thread_start = time.monotonic()
    process_state['status'] = 'in_progress'
    process_state['last_action'] = 'tableA_task'
    client_thread = Thread(
        target=_run_tablea_task_thread,
        args=(tableData['model'], recent_matching_scan),
        daemon=True,
    )
    client_thread.start()
    thread_elapsed = time.monotonic() - thread_start
    total_elapsed = time.monotonic() - launch_start
    route_timings["workerStartSeconds"] = thread_elapsed
    route_timings["launchSeconds"] = total_elapsed
    route_timings["routeTotalSeconds"] = time.monotonic() - route_start
    app.logger.info(
        "[TableA Start] response ready in %.3fs; timings=%s",
        route_timings["routeTotalSeconds"],
        route_timings,
    )

    return jsonify({
        'success': True,
        'process' : "started",
        "status": "Process started",
        "timings": route_timings,
    })


############################################################################################



@app.route('/upload_3d_file', methods=['POST'])
def upload_3d_file():
    if 'file' not in request.files:
        return jsonify({'success': False, 'message': 'No file part'})

    file = request.files['file']
    if file.filename == '':
        return jsonify({'success': False, 'message': 'No selected file'})


    if file and allowed_file(file.filename):
        try:
            # Secure the filename and save the file
            filename = secure_filename(file.filename)
            filepath = os.path.join(UPLOAD_FOLDER, filename)
            file.save(filepath)

            upload3DModel(filepath)

            return jsonify({
                'success': True,
                'message': 'File uploaded successfully',
                'filename': filename
            })
        except Exception as e:
            return jsonify({
                'success': False,
                'message': f'Error processing file: {str(e)}'
            })

    return jsonify({
        'success': False,
        'message': 'Invalid file type'
    })


############################################################################################

# Combines the data from the config file and the user inputted data and return as combined configurations data.
def fetch_and_combine_data():
    try:
        api_response = requests.get("http://localhost:5100/get_modal_data")
        api_data = api_response.json()

        if api_data.get("message") == "No modal data.":
            return {"status": "incomplete", "message": "Settings are still remaining to do."}
    except Exception as e:
        return {"status": "error", "message": f"API error: {e}"}

    try:
        with open('./configs/config.yaml', 'r') as file:
            config_data = yaml.safe_load(file)
        print("Config Data: ", config_data)
    except Exception as e:
        return {"status": "error", "message": f"YAML file error: {e}"}

    combined_data = {**config_data, **api_data['tableA']}  # YAML data is combined with API data
    return combined_data


def _halt_robot_motion_best_effort():
    """Send physical stop commands via a fresh CPS client after hard-stop."""
    try:
        config = load_config()
        config["logger"] = setup_logger(config["settings"]["debug"])
    except Exception:
        return

    cps_tmp = CPSClient()
    try:
        ret = cps_tmp.HRIF_Connect(0, config["server"]["cpip"], config["server"]["cps"])
        if ret != 0:
            return

        # Stop robot/axis motion and disable force mode if active.
        try:
            cps_tmp.HRIF_GrpStop(0, 0)
        except Exception:
            pass
        try:
            result = []
            cps_tmp.HRIF_HRApp(0, "HR_Motor", "MotorStop", ["J7"], result)
        except Exception:
            pass
        try:
            cps_tmp.HRIF_SetForceControlState(0, 0, 0)
        except Exception:
            pass
        try:
            cps_tmp.HRIF_SetBoxDO(0, 4, 0)  # vibration off
        except Exception:
            pass
        try:
            cps_tmp.HRIF_SetBoxDO(0, 7, 0)  # tool spin off
        except Exception:
            pass
        try:
            laser(cps_tmp, "off", config=config)
        except Exception:
            pass
    finally:
        try:
            cps_tmp.HRIF_DisConnect(0)
        except Exception:
            pass


def _read_robot_enabled_flag():
    """Best-effort read of robot enabled flag. Returns True/False/None."""
    result = []
    with locked_cps(allow_when_busy=True) as ok:
        if not ok:
            return None
        nRet = CPS.HRIF_ReadRobotState(0, 0, result)
    if nRet != 0 or len(result) < 2:
        return None
    return result[1] == "1"


def _wait_cps_ready_after_stop(config, max_wait_s=STOP_TO_HOMING_PROBE_TIMEOUT_SECONDS, poll_s=0.08):
    """Wait until CPS accepts a fresh connect+state-read after a hard stop."""
    if not isinstance(config, dict):
        return False
    server = config.get("server", {})
    ip = server.get("cpip")
    port = server.get("cps")
    if not ip or not port:
        return False

    deadline = time.monotonic() + max_wait_s
    while time.monotonic() < deadline:
        probe = CPSClient()
        try:
            ret = probe.HRIF_Connect(0, ip, port)
            if ret == 0:
                state = []
                read_ret = probe.HRIF_ReadRobotState(0, 0, state)
                if read_ret == 0 and isinstance(state, (list, tuple)) and len(state) > 0:
                    return True
        except Exception:
            pass
        finally:
            try:
                probe.HRIF_DisConnect(0)
            except Exception:
                pass
        time.sleep(poll_s)
    return False


def _get_tool_in_hand_stable(cps, attempts=4, delay_s=0.08):
    """
    Read tool-in-hand with short retries to reduce transient CI/DI flicker.
    Returns first stable valid decode in 0..4 when possible; otherwise last read.
    """
    last = None
    stable_hits = 0
    prev_valid = None
    for idx in range(max(1, int(attempts))):
        detected = _get_tool_in_hand(cps)
        last = detected
        if detected in (0, 1, 2, 3, 4):
            if detected == prev_valid:
                stable_hits += 1
            else:
                stable_hits = 1
                prev_valid = detected
            if stable_hits >= 2:
                return detected
        if idx < attempts - 1:
            time.sleep(max(0.0, float(delay_s)))
    return last


def _disconnect_global_cps_for_child_start():
    """Disconnect and reset CPS once before starting a child."""
    with robot_lock:
        _hard_reset_cps_runtime()


def _fast_detach_global_cps_for_child_start():
    """Fast parent CPS detach before a Table A worker owns robot access.

    This avoids blocking the start-task HTTP response on SDK disconnect
    timeouts while still closing parent sockets and resetting the wrapper.
    """
    global CPS, _cps_last_connect_attempt
    with robot_lock:
        try:
            client = CPS.g_clients[0]
            for attr in ("tcp", "tcp_fast"):
                sock = getattr(client, attr, None)
                if sock is not None:
                    try:
                        sock.close()
                    except Exception:
                        pass
                    try:
                        setattr(client, attr, None)
                    except Exception:
                        pass
        except Exception:
            pass
        try:
            plugin_client = CPS.g_plugin_clients[0]
            sock = getattr(plugin_client, "tcp", None)
            if sock is not None:
                try:
                    sock.close()
                except Exception:
                    pass
                try:
                    plugin_client.tcp = None
                except Exception:
                    pass
        except Exception:
            pass
        try:
            CPS.g_clients[0] = RbtClient()
            CPS.g_plugin_clients[0] = PluginClient()
            CPS.g_client_state[0] = False
            CPS.g_plugin_client_state[0] = False
        except Exception:
            CPS = CPSClient()
        _cps_last_connect_attempt = 0.0


def _post_scan_prepare_for_tablea_start():
    """Detach the scan CPS session after scan so first task start is not delayed."""
    try:
        # Let /action return and release robot_lock before cleanup starts.
        time.sleep(0.05)
        start = time.monotonic()
        _fast_detach_global_cps_for_child_start()
        app.logger.info(
            "[scan] Post-scan CPS detach complete in %.3fs; Table A start is prepped.",
            time.monotonic() - start,
        )
    except Exception as exc:
        try:
            app.logger.warning("[scan] Post-scan CPS detach skipped/failed: %s", exc)
        except Exception:
            pass


def _run_tablea_task_thread(model_key, recent_matching_scan=False):
    """Run Table A task in a background thread to avoid Windows process spawn delay."""
    global client_thread, _last_child_exit_ts, _cps_reconnect_grace_until
    ok = False
    try:
        detach_start = time.monotonic()
        _fast_detach_global_cps_for_child_start()
        app.logger.info(
            "[TableA Thread] parent CPS detached in %.3fs before worker connect.",
            time.monotonic() - detach_start,
        )
        run_tablea_task_child(model_key, recent_matching_scan)
        ok = True
    except Exception as exc:
        try:
            app.logger.exception("[TableA Thread] task failed: %s", exc)
        except Exception:
            print(f"[TableA Thread] task failed: {exc}")
    finally:
        _last_child_exit_ts = time.monotonic()
        _cps_reconnect_grace_until = _last_child_exit_ts + max(CPS_RECONNECT_GRACE_SECONDS, POST_STOP_GRACE_SECONDS)
        process_state['status'] = 'completed' if ok else 'failed'
        process_state['last_action'] = None
        client_thread = None
        socketio.emit(
            'flash_message',
            {"message": "Process finished" if ok else "Process failed"},
        )
        _fast_detach_global_cps_for_child_start()

############################################################################################
# Start the handle_client process threads
# def start_process(config):
#     global client_process
#     if client_process and client_process.is_alive():
#         return jsonify({'status': 'error', 'message': 'Process already running'})

#     client_process = Process(target=handle_client, args=(config,))
#     client_process.start()
#     socketio.emit('flash_message', {"message": f"Scanning and Sanding Started..."})
#     return jsonify({'status': 'success', 'message': 'Process started successfully'})

############################################################################################
# Stops the handle_client process threads if stop button is pressed!

@app.route("/stop", methods=["POST"])
def stop_process():
    global _cps_reconnect_grace_until, _last_stop_ts, client_process, client_thread
    had_process = bool(client_process and client_process.is_alive())
    had_thread = bool(client_thread and client_thread.is_alive())
    scan_active = _inline_scan_active.is_set()
    if not had_process and not had_thread and not scan_active:
        enabled = _read_robot_enabled_flag()
        if enabled is False or enabled is None:
            msg = "Robot is not enabled or not connected. Stop is unavailable."
            socketio.emit('flash_message', {"message": msg})
            return jsonify({'error': msg}), 200
    if had_process:
        proc = client_process
        request_stop()
        _halt_robot_motion_best_effort()
        # Give the worker a brief chance to exit cooperatively on stop flag.
        proc.join(timeout=STOP_JOIN_TIMEOUT_SECONDS)
        if proc.is_alive():
            try:
                proc.terminate()
            except Exception:
                pass
            proc.join(timeout=STOP_KILL_JOIN_TIMEOUT_SECONDS)
        if proc.is_alive():
            try:
                proc.kill()
            except Exception:
                pass
            proc.join(timeout=STOP_KILL_JOIN_TIMEOUT_SECONDS)
        if proc.is_alive():
            print("Client process did not exit cleanly after terminate/kill.")
        else:
            print("Client process terminated.")
        # Second hard-off pass after worker exit to guarantee outputs are off.
        _halt_robot_motion_best_effort()
        process_state['status'] = 'completed'
        client_process = None
        _cps_reconnect_grace_until = time.monotonic() + CPS_RECONNECT_GRACE_SECONDS
    elif had_thread:
        th = client_thread
        request_stop()
        _halt_robot_motion_best_effort()
        # Thread cannot be force-killed; rely on cooperative stop + motor stop.
        th.join(timeout=STOP_JOIN_TIMEOUT_SECONDS + STOP_KILL_JOIN_TIMEOUT_SECONDS)
        if th.is_alive():
            print("Inline homing thread is still exiting after stop request.")
        else:
            print("Inline homing thread stopped.")
        # Second hard-off pass after worker exit to guarantee outputs are off.
        _halt_robot_motion_best_effort()
        process_state['status'] = 'completed'
        process_state['last_action'] = None
        client_thread = None
        _cps_reconnect_grace_until = time.monotonic() + CPS_RECONNECT_GRACE_SECONDS
    elif scan_active:
        # Scan runs inline in /action and shares the global CPS socket.
        # The scan thread stops motion through its existing CPS socket. Keep a
        # second-connection stop as a non-blocking safety backup only.
        print("Inline scan is active; requesting cooperative stop.")
        request_stop()
        Thread(target=_halt_robot_motion_best_effort, daemon=True).start()
        _last_stop_ts = time.monotonic()
        _cps_reconnect_grace_until = time.monotonic() + max(
            CPS_RECONNECT_GRACE_SECONDS, POST_STOP_GRACE_SECONDS
        )
        socketio.emit('flash_message', {"message": "Stop requested for active scan"})
        return jsonify({'status': 'success', 'message': 'Stop requested for active scan'})
    else:
        print("No active client process to terminate.")
        _halt_robot_motion_best_effort()

    _last_stop_ts = time.monotonic()
    # After force-stop, avoid immediate CPS traffic and reset runtime handles.
    with robot_lock:
        _hard_reset_cps_runtime()
        _cps_reconnect_grace_until = time.monotonic() + max(CPS_RECONNECT_GRACE_SECONDS, POST_STOP_GRACE_SECONDS)

    if had_process or had_thread:
        socketio.emit('flash_message', {"message": f"Stopped All Running Process!"})
        return jsonify({'status': 'success', 'message': 'Process terminated successfully'})

    socketio.emit('flash_message', {"message": f"No Process to Stop"})
    return jsonify({'status': 'error', 'message': 'No active process to terminate'})

############################################################################################
#Tool3_pick_and_drop integration by rafat
@app.route("/tool_toggle", methods=["POST"])
@app.route("/tool_toggle4", methods=["POST"])
def tool_toggle():
    """
    Expects a JSON payload like:
    {
      "toolNumber": 1,
      "action": "pick"  (or "keep")
    }
    """
    data = request.json
    if not data:
        return jsonify({"error": "No JSON data provided"}), 400

    tool_num = data.get("toolNumber")
    action = data.get("action")

    if not tool_num or not action:
        return jsonify({"error": "toolNumber or action missing"}), 400

    # 1) Get or build your config from file + modal data
    config_data_UI = fetch_and_combine_data()

    # Manually add a logger instance to config
    config_data_UI['logger'] = setup_logger(config_data_UI['settings']['debug'])

    def check_conditions(condition_list):
        """Helper to check CPS conditions."""
        for func, box_id, nBit, expected in condition_list:
            print(func, box_id, nBit, expected)
            result = []
            nRet = func(box_id, nBit, result)
            print('output', nRet, result)
            if nRet != 0 or result[0] != str(expected):
                return False
        return True

    # 3) Depending on the action, call getTool or keepTool
    if action == "pick":
        with locked_cps() as ok:
            if not ok:
                return jsonify({"error": "Failed to connect to CPS client"}), 500
            cps = CPS
            # Validate pick state by decoded tool-in-hand status so adding Tool 4
            # does not break hard-coded CI/DI expectations.
            detected_tool = _get_tool_in_hand_stable(cps)
            if detected_tool == 0:
                # Pick the tool
                try:
                    success = getTool11(cps, toolNumber=tool_num, config=config_data_UI)
                except RuntimeError as exc:
                    socketio.emit('flash_message', {"message": f"Tool {tool_num} pick failed: {exc}"})
                    return jsonify({"error": str(exc)}), 409
                if not success:
                    return jsonify({"error": f"Tool {tool_num} not detected after pick"}), 409
                socketio.emit('flash_message', {"message": f"Picked Tool {tool_num}"})
                if tool_num == 3:
                    tool_override_state[3] = True
                # cps.HRIF_DisConnect(0)
                return jsonify({"status": "success", "message": f"Tool {tool_num} picked successfully"})
            elif detected_tool in (-1, None):
                sensors = _read_tool_sensors(cps)
                sensor_msg = (
                    f"CI0={sensors.get('ci0')} CI1={sensors.get('ci1')} CI2={sensors.get('ci2')} "
                    f"DI4={sensors.get('di4')} DI5={sensors.get('di5')} DI6={sensors.get('di6')} DI7={sensors.get('di7')}"
                )
                socketio.emit(
                    'flash_message',
                    {
                        "message": (
                            f"Tool state uncertain (detected={detected_tool}); blocking pick for safety. "
                            f"{sensor_msg}"
                        )
                    },
                )
                return jsonify({"error": "Tool state uncertain; cannot pick"}), 409
            else:
                socketio.emit(
                    'flash_message',
                    {"message": f"Already holding Tool {detected_tool}. Can't pick Tool {tool_num}"}
                )
                # cps.HRIF_DisConnect(0)
                return jsonify({"error": "Pick conditions not met"}), 400

    elif action == "keep":
        with locked_cps() as ok:
            if tool_num == 3 and tool_override_state.get(3):
                if not ok:
                    return jsonify({"error": "Failed to connect to CPS client"}), 500
                cps = CPS
                try:
                    keepTool11(cps, toolNumber=tool_num, config=config_data_UI)
                except RuntimeError as exc:
                    socketio.emit('flash_message', {"message": f"Tool {tool_num} drop failed: {exc}"})
                    return jsonify({"error": str(exc)}), 409
                tool_override_state[3] = False
                socketio.emit('flash_message', {"message": f"Kept Tool {tool_num}"})
                return jsonify({"status": "success", "message": f"Tool {tool_num} kept successfully"})
            if not ok:
                return jsonify({"error": "Failed to connect to CPS client"}), 500
            cps = CPS
            force_keep = bool(data.get("forceKeep")) or bool(data.get("force"))
            detected_tool = _get_tool_in_hand_stable(cps)
            if detected_tool == tool_num:
                # Keep the tool
                try:
                    keepTool11(cps, toolNumber=tool_num, config=config_data_UI)
                except RuntimeError as exc:
                    socketio.emit('flash_message', {"message": f"Tool {tool_num} drop failed: {exc}"})
                    return jsonify({"error": str(exc)}), 409
                socketio.emit('flash_message', {"message": f"Kept Tool {tool_num}"})
                if tool_num == 3:
                    tool_override_state[3] = False
                # cps.HRIF_DisConnect(0)
                return jsonify({"status": "success", "message": f"Tool {tool_num} kept successfully"})
            else:
                # Recovery path for post-crash or sensor decode uncertainty.
                if force_keep and detected_tool in (-1, None):
                    try:
                        keepTool11(cps, toolNumber=tool_num, config=config_data_UI)
                    except RuntimeError as exc:
                        socketio.emit('flash_message', {"message": f"Tool {tool_num} forced drop failed: {exc}"})
                        return jsonify({"error": str(exc)}), 409
                    if tool_num == 3:
                        tool_override_state[3] = False
                    socketio.emit('flash_message', {
                        "message": f"Kept Tool {tool_num} via forced recovery path"
                    })
                    return jsonify({
                        "status": "success",
                        "message": f"Tool {tool_num} kept via forced recovery path"
                    })
                sensors = _read_tool_sensors(cps)
                sensor_msg = (
                    f"CI0={sensors.get('ci0')} CI1={sensors.get('ci1')} CI2={sensors.get('ci2')} "
                    f"DI4={sensors.get('di4')} DI5={sensors.get('di5')} DI6={sensors.get('di6')} DI7={sensors.get('di7')}"
                )
                socketio.emit(
                    'flash_message',
                    {
                        "message": (
                            f"Not holding Tool {tool_num} to drop "
                            f"(detected={detected_tool}). {sensor_msg}"
                        )
                    },
                )
                # cps.HRIF_DisConnect(0)
                return jsonify({"error": "Drop conditions not met"}), 400

    else:
        return jsonify({"error": "Invalid action. Must be 'pick' or 'keep'."}), 400


############################################################################################
#Tool2_pick_and_drop integration by rafat
@app.route("/tool_toggle2", methods=["POST"])
def tool_toggle2():
    """
    Expects a JSON payload like:
    {
      "toolNumber": 2,
      "action": "pick"  (or "keep")
    }
    """
    data = request.json
    if not data:
        return jsonify({"error": "No JSON data provided"}), 400

    tool_num = data.get("toolNumber")
    action = data.get("action")
    print(f'tool num: {tool_num}, action: {action}')
    if not tool_num or not action:
        return jsonify({"error": "toolNumber or action missing"}), 400

    # 1) Get or build your config from file + modal data
    config_data_UI = fetch_and_combine_data()

    # Manually add a logger instance to config
    config_data_UI['logger'] = setup_logger(config_data_UI['settings']['debug'])

    # 3) Depending on the action, call getTool or keepTool
    if action == "pick":
        with locked_cps() as ok:
            if not ok:
                return jsonify({"error": "Failed to connect to CPS client"}), 500
            cps = CPS
            detected_tool = _get_tool_in_hand_stable(cps)
            if detected_tool == 0:
                try:
                    success = getTool11(cps, toolNumber=tool_num, config=config_data_UI)
                except RuntimeError as exc:
                    socketio.emit('flash_message', {"message": f"Tool {tool_num} pick failed: {exc}"})
                    return jsonify({"error": str(exc)}), 409
                if not success:
                    # One extra settle-check handles slow CI/DI transitions after valve close.
                    settle_s = float(config_data_UI.get("tool", {}).get("sensorSettleSeconds", 0.4) or 0.4)
                    if settle_s > 0:
                        time.sleep(settle_s)
                    # Tool 2 can take longer for CI/DI to settle after lift.
                    confirmed_tool = _get_tool_in_hand_stable(cps, attempts=20, delay_s=0.1)
                    if confirmed_tool != tool_num:
                        return jsonify({"error": f"Tool {tool_num} not detected after pick"}), 409
                socketio.emit('flash_message', {"message": f"Picked Tool {tool_num}"})
                return jsonify({"status": "success", "message": f"Tool {tool_num} picked successfully"})
            elif detected_tool in (-1, None):
                sensors = _read_tool_sensors(cps)
                sensor_msg = (
                    f"CI0={sensors.get('ci0')} CI1={sensors.get('ci1')} CI2={sensors.get('ci2')} "
                    f"DI4={sensors.get('di4')} DI5={sensors.get('di5')} DI6={sensors.get('di6')} DI7={sensors.get('di7')}"
                )
                socketio.emit(
                    'flash_message',
                    {
                        "message": (
                            f"Tool state uncertain (detected={detected_tool}); blocking pick for safety. "
                            f"{sensor_msg}"
                        )
                    },
                )
                return jsonify({"error": "Tool state uncertain; cannot pick"}), 409
            else:
                socketio.emit(
                    'flash_message',
                    {"message": f"Already holding Tool {detected_tool}. Can't pick Tool {tool_num}"}
                )
                return jsonify({"error": "Pick conditions not met"}), 400

    elif action == "keep":
        with locked_cps() as ok:
            if not ok:
                return jsonify({"error": "Failed to connect to CPS client"}), 500
            cps = CPS
            force_keep = bool(data.get("forceKeep")) or bool(data.get("force"))
            detected_tool = _get_tool_in_hand_stable(cps)
            if detected_tool == tool_num:
                try:
                    keepTool11(cps, toolNumber=tool_num, config=config_data_UI)
                except RuntimeError as exc:
                    socketio.emit('flash_message', {"message": f"Tool {tool_num} drop failed: {exc}"})
                    return jsonify({"error": str(exc)}), 409
                socketio.emit('flash_message', {"message": f"Kept Tool {tool_num}"})
                return jsonify({"status": "success", "message": f"Tool {tool_num} kept successfully"})
            else:
                # Recovery path for uncertain decode after abrupt stop/restart.
                if force_keep and detected_tool in (-1, None):
                    try:
                        keepTool11(cps, toolNumber=tool_num, config=config_data_UI)
                    except RuntimeError as exc:
                        socketio.emit('flash_message', {"message": f"Tool {tool_num} forced drop failed: {exc}"})
                        return jsonify({"error": str(exc)}), 409
                    socketio.emit('flash_message', {"message": f"Kept Tool {tool_num} via forced recovery path"})
                    return jsonify(
                        {
                            "status": "success",
                            "message": f"Tool {tool_num} kept via forced recovery path",
                        }
                    )

                sensors = _read_tool_sensors(cps)
                sensor_msg = (
                    f"CI0={sensors.get('ci0')} CI1={sensors.get('ci1')} CI2={sensors.get('ci2')} "
                    f"DI4={sensors.get('di4')} DI5={sensors.get('di5')} DI6={sensors.get('di6')} DI7={sensors.get('di7')}"
                )
                socketio.emit(
                    'flash_message',
                    {
                        "message": (
                            f"Not holding Tool {tool_num} to drop "
                            f"(detected={detected_tool}). {sensor_msg}"
                        )
                    },
                )
                return jsonify({"error": "Drop conditions not met"}), 400
    else:
        return jsonify({"error": "Invalid action. Must be 'pick' or 'keep'."}), 400


###########################################################################################
############################################################################################
#Tool1_pick_and_drop integration by rafat
@app.route("/tool_toggle1", methods=["POST"])
def tool_toggle1():
    """
    Expects a JSON payload like:
    {
      "toolNumber": 1,
      "action": "pick"  (or "keep")
    }
    """
    data = request.json
    if not data:
        return jsonify({"error": "No JSON data provided"}), 400

    tool_num = data.get("toolNumber")
    action = data.get("action")

    if not tool_num or not action:
        return jsonify({"error": "toolNumber or action missing"}), 400

    # 1) Get or build your config from file + modal data
    config_data_UI = fetch_and_combine_data()

    # Manually add a logger instance to config
    config_data_UI['logger'] = setup_logger(config_data_UI['settings']['debug'])

    # 3) Depending on the action, call getTool or keepTool
    if action == "pick":
        with locked_cps() as ok:
            if not ok:
                return jsonify({"error": "Failed to connect to CPS client"}), 500
            cps = CPS
            detected_tool = _get_tool_in_hand_stable(cps)
            if detected_tool == 0:
                try:
                    success = getTool11(cps, toolNumber=tool_num, config=config_data_UI)
                except RuntimeError as exc:
                    socketio.emit('flash_message', {"message": f"Tool {tool_num} pick failed: {exc}"})
                    return jsonify({"error": str(exc)}), 409
                if not success:
                    settle_s = float(config_data_UI.get("tool", {}).get("sensorSettleSeconds", 0.4) or 0.4)
                    if settle_s > 0:
                        time.sleep(settle_s)
                    confirmed_tool = _get_tool_in_hand_stable(cps)
                    if confirmed_tool != tool_num:
                        return jsonify({"error": f"Tool {tool_num} not detected after pick"}), 409
                socketio.emit('flash_message', {"message": f"Picked Tool {tool_num}"})
                return jsonify({"status": "success", "message": f"Tool {tool_num} picked successfully"})
            elif detected_tool in (-1, None):
                sensors = _read_tool_sensors(cps)
                sensor_msg = (
                    f"CI0={sensors.get('ci0')} CI1={sensors.get('ci1')} CI2={sensors.get('ci2')} "
                    f"DI4={sensors.get('di4')} DI5={sensors.get('di5')} DI6={sensors.get('di6')} DI7={sensors.get('di7')}"
                )
                socketio.emit(
                    'flash_message',
                    {
                        "message": (
                            f"Tool state uncertain (detected={detected_tool}); blocking pick for safety. "
                            f"{sensor_msg}"
                        )
                    },
                )
                return jsonify({"error": "Tool state uncertain; cannot pick"}), 409
            else:
                socketio.emit(
                    'flash_message',
                    {"message": f"Already holding Tool {detected_tool}. Can't pick Tool {tool_num}"}
                )
                return jsonify({"error": "Pick conditions not met"}), 400

    elif action == "keep":
        with locked_cps() as ok:
            if not ok:
                return jsonify({"error": "Failed to connect to CPS client"}), 500
            cps = CPS
            force_keep = bool(data.get("forceKeep")) or bool(data.get("force"))
            detected_tool = _get_tool_in_hand_stable(cps)
            if detected_tool == tool_num:
                try:
                    keepTool11(cps, toolNumber=tool_num, config=config_data_UI)
                except RuntimeError as exc:
                    socketio.emit('flash_message', {"message": f"Tool {tool_num} drop failed: {exc}"})
                    return jsonify({"error": str(exc)}), 409
                socketio.emit('flash_message', {"message": f"Kept Tool {tool_num}"})
                return jsonify({"status": "success", "message": f"Tool {tool_num} kept successfully"})
            else:
                if force_keep and detected_tool in (-1, None):
                    try:
                        keepTool11(cps, toolNumber=tool_num, config=config_data_UI)
                    except RuntimeError as exc:
                        socketio.emit('flash_message', {"message": f"Tool {tool_num} forced drop failed: {exc}"})
                        return jsonify({"error": str(exc)}), 409
                    socketio.emit('flash_message', {"message": f"Kept Tool {tool_num} via forced recovery path"})
                    return jsonify(
                        {
                            "status": "success",
                            "message": f"Tool {tool_num} kept via forced recovery path",
                        }
                    )

                sensors = _read_tool_sensors(cps)
                sensor_msg = (
                    f"CI0={sensors.get('ci0')} CI1={sensors.get('ci1')} CI2={sensors.get('ci2')} "
                    f"DI4={sensors.get('di4')} DI5={sensors.get('di5')} DI6={sensors.get('di6')} DI7={sensors.get('di7')}"
                )
                socketio.emit(
                    'flash_message',
                    {
                        "message": (
                            f"Not holding Tool {tool_num} to drop "
                            f"(detected={detected_tool}). {sensor_msg}"
                        )
                    },
                )
                return jsonify({"error": "Drop conditions not met"}), 400
    else:
        return jsonify({"error": "Invalid action. Must be 'pick' or 'keep'."}), 400

############################################################################################
def check_tool1_attachment_condition(cps):
    detected = _get_tool_in_hand_stable(cps)
    all_met = detected == 1
    if all_met:
        socketio.emit('blink_circle_button', {'shouldBlink': True})
    else:
        socketio.emit('blink_circle_button', {'shouldBlink': False})

    return all_met
############################################################################################
@app.route('/check_tool1_status', methods=['GET'])
def check_tool1_status():
    """
    A route that calls our check_tool1_attachment_condition function
    and returns a simple JSON response.
    By calling this route, we ensure the server checks the condition
    and emits the blink_circle_button event if needed.
    """
    with locked_cps() as ok:
        if not ok:
            return jsonify({"status": "busy", "shouldBlink": False})
        should_blink = check_tool1_attachment_condition(CPS)  # Emits if any WS clients exist.
    return jsonify({"status": "OK", "shouldBlink": bool(should_blink)})

############################################################################################
# For Tool 2 Status function
def check_tool2_attachment_condition(cps):
    detected = _get_tool_in_hand_stable(cps)
    all_met = detected == 2
    if all_met:
        socketio.emit('blink_circle_button2', {'shouldBlink': True})
    else:
        socketio.emit('blink_circle_button2', {'shouldBlink': False})

    return all_met
############################################################################################
@app.route('/check_tool2_status', methods=['GET'])
def check_tool2_status():
    """
    A route that calls check_tool2_attachment_condition.
    By calling this route, we check the lines for tool2
    and emit a 'blink_circle_button2' event with True or False.
    """
    with locked_cps() as ok:
        if not ok:
            return jsonify({"status": "busy", "shouldBlink": False})
        should_blink = check_tool2_attachment_condition(CPS)
    return jsonify({"status": "OK", "shouldBlink": bool(should_blink)})

############################################################################################
def check_tool3_attachment_condition(cps):
    if tool_override_state.get(3):
        socketio.emit('blink_circle_button3', {'shouldBlink': True})
        return True
    detected = _get_tool_in_hand_stable(cps)
    all_met = detected == 3
    if all_met:
        socketio.emit('blink_circle_button3', {'shouldBlink': True})
    else:
        socketio.emit('blink_circle_button3', {'shouldBlink': False})

    return all_met
############################################################################################
@app.route('/check_tool3_status', methods=['GET'])
def check_tool3_status():
    """
    A route that calls check_tool3_attachment_condition.
    By calling this route, we check the lines for tool3
    and emit a 'blink_circle_button3' event with True or False.
    """
    with locked_cps() as ok:
        if not ok:
            return jsonify({"status": "busy", "shouldBlink": False})
        should_blink = check_tool3_attachment_condition(CPS)
    return jsonify({"status": "OK", "shouldBlink": bool(should_blink)})

############################################################################################
def check_tool4_attachment_condition(cps):
    detected = _get_tool_in_hand_stable(cps)
    all_met = detected == 4
    if all_met:
        socketio.emit('blink_circle_button4', {'shouldBlink': True})
    else:
        socketio.emit('blink_circle_button4', {'shouldBlink': False})

    return all_met

############################################################################################
@app.route('/check_tool4_status', methods=['GET'])
def check_tool4_status():
    """
    A route that calls check_tool4_attachment_condition.
    By calling this route, we check the lines for tool4
    and emit a 'blink_circle_button4' event with True or False.
    """
    with locked_cps() as ok:
        if not ok:
            return jsonify({"status": "busy", "shouldBlink": False})
        should_blink = check_tool4_attachment_condition(CPS)
    return jsonify({"status": "OK", "shouldBlink": bool(should_blink)})
def _table_name(table_id):
    if table_id == "tableAOpenClose":
        return "Table A"
    if table_id == "tableBOpenClose":
        return "Table B"
    return "Table"


def _table_target_name(desired_state):
    if desired_state == "Open":
        return "horizontal"
    if desired_state == "Close":
        return "45 degree"
    return "unknown"


def _cps_ret_ok(ret):
    return ret in (0, None)


def _command_table_state_direct(cps, table_id, desired_state):
    """
    Send the table output command immediately.
    Physical confirmation is handled by /table_state polling, not by this button route.
    """
    if desired_state not in ("Open", "Close"):
        return {
            "success": False,
            "newState": "Error",
            "message": f"Invalid desired table state: {desired_state}",
        }

    table_name = _table_name(table_id)
    target_name = _table_target_name(desired_state)

    if table_id == "tableAOpenClose":
        if desired_state == "Open":
            # Table A horizontal: DO1 release, DO0 drive.
            release_ret = cps.HRIF_SetBoxDO(0, 1, 0)
            drive_ret = cps.HRIF_SetBoxDO(0, 0, 1)
        else:
            # Table A 45 degree: DO0 release, DO1 drive.
            release_ret = cps.HRIF_SetBoxDO(0, 0, 0)
            drive_ret = cps.HRIF_SetBoxDO(0, 1, 1)
    elif table_id == "tableBOpenClose":
        if desired_state == "Open":
            # Table B horizontal: CO1 release, CO0 drive.
            release_ret = cps.HRIF_SetBoxCO(0, 1, 0)
            drive_ret = cps.HRIF_SetBoxCO(0, 0, 1)
        else:
            # Table B 45 degree: CO0 release, CO1 drive.
            release_ret = cps.HRIF_SetBoxCO(0, 0, 0)
            drive_ret = cps.HRIF_SetBoxCO(0, 1, 1)
    else:
        return {
            "success": False,
            "newState": "Error",
            "message": f"Invalid table ID: {table_id}",
        }

    print(
        f"[table-toggle] command table={table_id} target={desired_state} "
        f"release_ret={release_ret} drive_ret={drive_ret}"
    )

    if not (_cps_ret_ok(release_ret) and _cps_ret_ok(drive_ret)):
        return {
            "success": False,
            "newState": "Error",
            "message": (
                f"{table_name} {target_name} command failed "
                f"(release_ret={release_ret}, drive_ret={drive_ret})"
            ),
        }

    return {
        "success": True,
        "newState": desired_state,
        "commandSent": True,
        "message": f"{table_name} {target_name} command sent",
    }


def _read_current_table_state_for_toggle(cps, table_id):
    if table_id == "tableAOpenClose":
        di_state_0 = []
        di_state_1 = []
        nRet0 = cps.HRIF_ReadBoxDI(0, 0, di_state_0)
        nRet1 = cps.HRIF_ReadBoxDI(0, 1, di_state_1)
        if nRet0 == 0 and nRet1 == 0 and di_state_0 and di_state_1:
            sensor_state = (str(di_state_0[0]), str(di_state_1[0]))
            if sensor_state == ("1", "0"):
                return "Close", None
            if sensor_state == ("0", "1"):
                return "Open", None
            return None, f"Table A sensor state is invalid: {sensor_state}"
        return None, "Unable to read Table A position sensors."

    if table_id == "tableBOpenClose":
        robot_state = []
        nRet = cps.HRIF_ReadBoxCO(0, 1, robot_state)
        if nRet == 0 and robot_state:
            return ("Close" if robot_state[0] == '1' else "Open"), None
        return None, "Unable to read Table B position output."

    return None, "Invalid input. No state change."


############################################################################################
# Toggle the Table A/B Open/Close button text depending upon table current state.
@app.route('/toggle_state/<table_id>', methods=['GET'])
def toggle_state(table_id):
    requested_state = request.args.get("desired")
    print(
        f"[table-toggle] request table={table_id} desired={requested_state} "
        f"j7_home_confirmed={j7_home_confirmed}"
    )

    with locked_cps() as ok:
        if not ok:
            print(f"[table-toggle] busy table={table_id} desired={requested_state}")
            return jsonify({'newState': "Busy"}), 200

        # Ensure 7th axis is at home before allowing table open/close.
        # J7 position cannot be read, so we rely on a software flag set by homing.
        if not j7_home_confirmed:
            msg = "Please home the robot (7th axis) before opening or closing the table."
            print(f"[table-toggle] blocked table={table_id}: {msg}")
            socketio.emit('flash_message', {"message": msg, "type": "warning"})
            return jsonify({"error": msg, "newState": "Blocked"}), 200

        if requested_state not in ("Open", "Close"):
            current_state, error = _read_current_table_state_for_toggle(CPS, table_id)
            if error:
                print(f"[table-toggle] state read failed table={table_id}: {error}")
                socketio.emit('flash_message', {"message": error, "type": "warning"})
                return jsonify({"error": error, "newState": "Unknown"}), 500
            requested_state = "Open" if current_state == "Close" else "Close"

        result = _command_table_state_direct(CPS, table_id, requested_state)

    if not result.get("success", False):
        socketio.emit(
            'flash_message',
            {"message": result.get("message", "Table movement command failed."), "type": "warning"},
        )
        return jsonify(result), 500

    socketio.emit('flash_message', {"message": result["message"]})
    return jsonify(result)

############################################################################################
# Send the Table A/B actual state to the frontend.

@app.route('/get_state/<table_id>', methods=['GET'])
def get_state(table_id):
    robot_state = []
    with locked_cps() as ok:
        if not ok:
            return jsonify({'newState': 'Busy'}), 200

        if table_id == "tableAOpenClose":
            di_state_0 = []
            di_state_1 = []
            nRet0 = CPS.HRIF_ReadBoxDI(0, 0, di_state_0)
            nRet1 = CPS.HRIF_ReadBoxDI(0, 1, di_state_1)
            if nRet0 != 0 or nRet1 != 0 or not di_state_0 or not di_state_1:
                return jsonify({'newState': 'Unknown'}), 200
            sensor_state = (str(di_state_0[0]), str(di_state_1[0]))
            if sensor_state == ("1", "0"):
                new_state = "Close"
            elif sensor_state == ("0", "1"):
                new_state = "Open"
            else:
                new_state = "Unknown"

        elif table_id == "tableBOpenClose":
            nRet = CPS.HRIF_ReadBoxCO(0, 1, robot_state)
            socketio.emit('flash_message', {"message": f"Table B is Set To {'Close' if robot_state[0] == '1' else 'Open'}. Please Wait Till The Process Finishes..."})
            if robot_state[0] == '1':
                new_state = "Close"
            else:
                new_state = "Open"
        else:
            return jsonify({'newState': 'Invalid input. No state change.'})


    return jsonify({'newState': f"{new_state}"})

############################################################################################
# Send the Table A/B actual state to the frontend without emitting flash messages.
@app.route('/table_state/<table_id>', methods=['GET'])
def table_state(table_id):
    with locked_cps() as ok:
        if not ok:
            return jsonify({'state': 'Unknown'})
        if table_id == "tableAOpenClose":
            di_state_0 = []
            di_state_1 = []
            nRet0 = CPS.HRIF_ReadBoxDI(0, 0, di_state_0)
            nRet1 = CPS.HRIF_ReadBoxDI(0, 1, di_state_1)
            if nRet0 == 0 and nRet1 == 0 and di_state_0 and di_state_1:
                if di_state_0[0] == '1' and di_state_1[0] == '0':
                    return jsonify({'state': 'Close'})
                if di_state_0[0] == '0' and di_state_1[0] == '1':
                    return jsonify({'state': 'Open'})
            return jsonify({'state': 'Unknown'})

        if table_id == "tableBOpenClose":
            robot_state = []
            nRet = CPS.HRIF_ReadBoxCO(0, 1, robot_state)
            if nRet == 0 and robot_state:
                return jsonify({'state': 'Close' if robot_state[0] == '1' else 'Open'})
            return jsonify({'state': 'Unknown'})

        return jsonify({'state': 'Invalid'})

############################################################################################
# Send the Stopper A/B actual state to the frontend.
@app.route('/stopper_state/<stopper_id>', methods=['GET'])
def stopper_state(stopper_id):
    with locked_cps() as ok:
        if not ok:
            return jsonify({'state': 'Unknown'})
        if stopper_id == "A":
            robot_state = []
            nRet = CPS.HRIF_ReadBoxDO(0, 2, robot_state)
            if nRet == 0 and robot_state:
                return jsonify({'state': 'Up' if robot_state[0] == '1' else 'Down'})
            return jsonify({'state': 'Unknown'})

        if stopper_id == "B":
            robot_state = []
            nRet = CPS.HRIF_ReadBoxCO(0, 2, robot_state)
            if nRet == 0 and robot_state:
                return jsonify({'state': 'Up' if robot_state[0] == '1' else 'Down'})
            return jsonify({'state': 'Unknown'})

        return jsonify({'state': 'Invalid'})


def _ensure_stopper_a_down(cps, timeout_s=2.0, poll_s=0.05):
    """Command Stopper A down and verify its output state before scanning."""
    stopper_state = []
    read_ret = cps.HRIF_ReadBoxDO(0, 2, stopper_state)
    if read_ret == 0 and stopper_state and stopper_state[0] == '0':
        return {
            "success": True,
            "alreadyInState": True,
            "message": "Stopper A already confirmed down",
        }

    set_ret = cps.HRIF_SetBoxDO(0, 2, 0)
    if set_ret not in (0, None):
        return {
            "success": False,
            "alreadyInState": False,
            "message": f"Failed to command Stopper A down (ret={set_ret})",
        }

    deadline = time.monotonic() + max(0.2, float(timeout_s))
    last_state = None
    while time.monotonic() < deadline:
        stopper_state = []
        read_ret = cps.HRIF_ReadBoxDO(0, 2, stopper_state)
        if read_ret == 0 and stopper_state:
            last_state = stopper_state[0]
            if last_state == '0':
                return {
                    "success": True,
                    "alreadyInState": False,
                    "message": "Stopper A confirmed down",
                }
        time.sleep(poll_s)

    return {
        "success": False,
        "alreadyInState": False,
        "message": f"Stopper A was not confirmed down before scan (state={last_state})",
    }


############################################################################################
# Send robot enable/power status to the frontend.
@app.route('/robot_status', methods=['GET'])
def robot_status():
    result = []
    with locked_cps() as ok:
        if not ok:
            return jsonify({'status': 'busy', 'flags': {}})
        if ok:
            nRet = CPS.HRIF_ReadRobotState(0, 0, result)
    if nRet != 0 or len(result) < 13:
        return jsonify({'status': 'error', 'message': 'Failed to read robot state'}), 500

    flags = {
        'in_motion': result[0] == "1",
        'enabled': result[1] == "1",
        'has_error': result[2] == "1",
        'error_code': result[3],
        'error_axis': result[4],
        'lock_state': result[5] == "1",
        'paused': result[6] == "1",
        'emergency_stop': result[7] == "1",
        'light_screen': result[8] == "1",
        'powered_on': result[9] == "1",
        'ebox_connected': result[10] == "1",
        'point_motion_done': result[11] == "1",
        'in_position': result[12] == "1",
    }
    return jsonify({'status': 'ok', 'flags': flags})

############################################################################################
# Process status (homing / sanding / scan child process)
@app.route('/process_status', methods=['GET'])
def process_status():
    required, reason = _compute_homing_requirement()
    return jsonify(
        {
            'status': process_state.get('status', 'completed'),
            'homingRequired': required,
            'homingReason': reason,
        }
    )


@app.route('/homing_status', methods=['GET'])
def homing_status():
    required, reason = _compute_homing_requirement()
    return jsonify({'required': required, 'reason': reason})


@app.route('/scan_status', methods=['GET'])
def scan_status():
    response = jsonify(_get_tablea_scan_status())
    response.headers["Cache-Control"] = "no-store, no-cache, must-revalidate, max-age=0"
    response.headers["Pragma"] = "no-cache"
    response.headers["Expires"] = "0"
    return response

def load_config():
    """Loads configuration from config.yaml."""
    with open('./configs/config.yaml', 'r') as file:
        config = yaml.safe_load(file)
    return config


def _compute_homing_requirement():
    """
    Return (required: bool, reason: str) for whether homing must be run now.
    """
    required = not bool(j7_home_confirmed)
    reason = "not_homed"
    if not required:
        j7_state = _probe_j7_state()
        # Do not clear homing on transient read failures (None), otherwise
        # users can get re-blocked right after a valid homing.
        if j7_state == "-1":
            # Homing intentionally disconnects J7 after reaching its configured
            # home position to release holding torque. Keep the software homing
            # confirmation; the next J7 movement reconnects the motor.
            required = False
            reason = "j7_disconnected_keep_homed"
        elif j7_state is None:
            required = False
            reason = "j7_probe_unavailable_keep_homed"
        else:
            reason = "ok"
    return required, reason

############################################################################################
# Receives all the button actions and send to the respective functions or trigger the functions in the SDK.
@app.route('/action', methods=['POST'])
def handle_action():

    # Load configuration from YAML
    config = load_config()

    # Set up logger
    config['logger'] = setup_logger(config['settings']['debug'])

    request_data = request.json or {}
    action = request_data.get('action')  # Get action from frontend
    global j7_home_confirmed
    print('the action is :',action)
    result = []  # Define the return value as an empty list

    if action == "stopperUp":
        with locked_cps() as ok:
            if not ok:
                return jsonify({"error": "Failed to connect to CPS client"}), 500
            CPS.HRIF_SetBoxDO(0, 2, 1) #Table Digital Output is 2
        socketio.emit('flash_message', {"message": f"StopperA Put Up"})
        return jsonify({'status': 'success', 'message': 'Action stopper received and executed'})

    if action == "stopperDown":
        with locked_cps() as ok:
            if not ok:
                return jsonify({"error": "Failed to connect to CPS client"}), 500
            CPS.HRIF_SetBoxDO(0, 2, 0) #Table Digital Output is 2
        socketio.emit('flash_message', {"message": f"StopperA Pulled Down"})
        return jsonify({'status': 'success', 'message': 'Action stopper received and executed'})

    elif action == "stopperUpB":
        with locked_cps() as ok:
            if not ok:
                return jsonify({"error": "Failed to connect to CPS client"}), 500
            CPS.HRIF_SetBoxCO(0, 2, 1) #Table Digital Output is 2
        socketio.emit('flash_message', {"message": f"StopperB Put Up"})
        return jsonify({'status': 'success', 'message': 'Action stopper received and executed'})

    elif action == "stopperDownB":
        with locked_cps() as ok:
            if not ok:
                return jsonify({"error": "Failed to connect to CPS client"}), 500
            CPS.HRIF_SetBoxCO(0, 2, 0) #Table Digital Output is 2
        socketio.emit('flash_message', {"message": f"StopperBS Pulled Down"})
        return jsonify({'status': 'success', 'message': 'Action stopper received and executed'})

    elif action == "toolLift":
        with locked_cps() as ok:
            if not ok:
                return jsonify({"error": "Failed to connect to CPS client"}), 500
            CPS.HRIF_SetBoxDO(0, 5, 0)
        socketio.emit('flash_message', {"message": f"Grabbing the Tool In Hand"})
        return jsonify({'status': 'success', 'message': 'Action tool received and executed'})

    elif action == "toolDrop":
        with locked_cps() as ok:
            if not ok:
                return jsonify({"error": "Failed to connect to CPS client"}), 500
            CPS.HRIF_SetBoxDO(0, 5, 1)
        socketio.emit('flash_message', {"message": f"Dropping the Tool In Hand"})
        return jsonify({'status': 'success', 'message': 'Action tool received and executed'})

    elif action == "laserOn":
        with locked_cps() as ok:
            if ok:
                laser(CPS, "on", config=config)
                socketio.emit('flash_message', {"message": "Laser turned ON"})
                return jsonify({'status': 'success', 'message': 'Laser ON executed'})
        # Fallback: try a temporary CPS connection if the shared lock is busy
        cps_tmp = CPSClient()
        ret = cps_tmp.HRIF_Connect(0, config["server"]["cpip"], config["server"]["cps"])
        if ret != 0:
            return jsonify({"error": f"Failed to connect to CPS client (ret={ret})"}), 500
        try:
            laser(cps_tmp, "on", config=config)
        finally:
            cps_tmp.HRIF_DisConnect(0)
        socketio.emit('flash_message', {"message": "Laser turned ON"})
        return jsonify({'status': 'success', 'message': 'Laser ON executed'})

    elif action == "laserOff":
        with locked_cps() as ok:
            if ok:
                laser(CPS, "off", config=config)
                socketio.emit('flash_message', {"message": "Laser turned OFF"})
                return jsonify({'status': 'success', 'message': 'Laser OFF executed'})
        # Fallback: try a temporary CPS connection if the shared lock is busy
        cps_tmp = CPSClient()
        ret = cps_tmp.HRIF_Connect(0, config["server"]["cpip"], config["server"]["cps"])
        if ret != 0:
            return jsonify({"error": f"Failed to connect to CPS client (ret={ret})"}), 500
        try:
            laser(cps_tmp, "off", config=config)
        finally:
            cps_tmp.HRIF_DisConnect(0)
        socketio.emit('flash_message', {"message": "Laser turned OFF"})
        return jsonify({'status': 'success', 'message': 'Laser OFF executed'})

    elif action == "stop":
        request_stop()
        return stop_process()

    elif action == "homing":
        global client_process, client_thread, _last_child_exit_ts
        process_running = bool(client_process and client_process.is_alive())
        thread_running = bool(client_thread and client_thread.is_alive())
        if process_running or thread_running:
            return jsonify({"error": "Process already running"}), 409
        clear_stop()
        config_data_UI = fetch_and_combine_data()
        if not isinstance(config_data_UI, dict):
            return jsonify({"error": "Invalid homing config payload"}), 500
        if config_data_UI.get("status") in ("incomplete", "error"):
            return jsonify({"error": config_data_UI.get("message", "Homing config is incomplete")}), 400

        # After any child-process completion, give CPS transport a short release window.
        since_child_exit = time.monotonic() - _last_child_exit_ts
        if 0 <= since_child_exit < CHILD_EXIT_SETTLE_SECONDS:
            time.sleep(CHILD_EXIT_SETTLE_SECONDS - since_child_exit)

        # Recent hard-stop path: short minimum gap + readiness probe, no fixed 2s sleep.
        elapsed_since_stop = time.monotonic() - _last_stop_ts
        if elapsed_since_stop < 5.0:
            min_gap_wait = STOP_TO_HOMING_MIN_GAP_SECONDS - elapsed_since_stop
            if min_gap_wait > 0:
                time.sleep(min_gap_wait)

        recent_stop_or_child = (
            elapsed_since_stop < 5.0 or since_child_exit < CHILD_EXIT_SETTLE_SECONDS
        )

        # Fast path: reuse parent CPS connection when transport is healthy and stable.
        # Safety: first homing after server start must calibrate J7 origin via fresh transport.
        can_inline = (
            j7_home_confirmed
            and (not recent_stop_or_child)
            and _parent_cps_healthy_for_homing()
        )

        process_state['status'] = 'in_progress'
        process_state['last_action'] = 'homing'
        _set_j7_home_confirmed(False)
        socketio.emit('flash_message', {"message": f"Homing Process Started"})

        if can_inline:
            client_thread = Thread(target=_run_homing_inline, args=(config_data_UI,), daemon=True)
            client_thread.start()
            return jsonify({'status': 'success', 'message': 'Action homing started (inline fast path)'})

        # Safe fallback: force fresh CPS transport via child process.
        with robot_lock:
            _hard_reset_cps_runtime()
            _cps_reconnect_grace_until = time.monotonic() + max(CPS_RECONNECT_GRACE_SECONDS, POST_STOP_GRACE_SECONDS)

        if recent_stop_or_child and not _wait_cps_ready_after_stop(config_data_UI):
            process_state['status'] = 'completed'
            process_state['last_action'] = None
            return jsonify({
                "error": "Robot controller is still releasing previous CPS session after stop. Try homing again."
            }), 503

        client_process = Process(target=_run_homing_child, args=(config_data_UI,))
        client_process.start()
        Thread(target=_track_process, args=(client_process,), daemon=True).start()
        return jsonify({'status': 'success', 'message': 'Action homing started (reconnect fallback)'})

    elif action == "enable":
        with locked_cps(allow_when_busy=True) as ok:
            if not ok:
                return jsonify({"error": "Failed to connect to CPS client"}), 500
            CPS.HRIF_GrpReset(0, 0)
            CPS.HRIF_GrpEnable(0, 0)
        socketio.emit('flash_message', {"message": f"Cobot Enabled"})
        return jsonify({'status': 'success', 'message': 'Action enable received and executed'})

    elif action == "disable":
        with locked_cps(allow_when_busy=True) as ok:
            if not ok:
                return jsonify({"error": "Failed to connect to CPS client"}), 500
            CPS.HRIF_GrpDisable(0, 0)
        socketio.emit('flash_message', {"message": f"Cobot Disabled"})
        return jsonify({'status': 'success', 'message': 'Action enable received and executed'})

    # elif action == "confirm":
    #     config_data_UI = fetch_and_combine_data()
    #     return start_process(config_data_UI)

    elif action == "scan":
        required, reason = _compute_homing_requirement()
        if required:
            msg = "Please run Homing before Scan. 7th axis calibration is required after app/server restart."
            socketio.emit('flash_message', {"message": msg})
            return jsonify(
                {
                    'status': 'error',
                    'code': 'homing_required',
                    'reason': reason,
                    'message': msg,
                }
            ), 409
        clear_stop()
        if _inline_scan_active.is_set():
            return jsonify({'status': 'error', 'message': 'Scan is already running'}), 409
        _inline_scan_active.set()
        scan_x_start_offset = float(config.get("offset", {}).get("scannerOffsetInLeft", 0.0))
        scan_y_start_offset = float(config.get("offset", {}).get("scannerOffsetInBottom", 0.0))
        config['logger'].info(
            "[scan] Runtime source=%s | scan start offsets: X=-%.3fmm Y=-%.3fmm, normal endpoints, no X-start return, home after Door1 Y-end",
            os.path.abspath(__file__),
            scan_x_start_offset,
            scan_y_start_offset,
        )
        socketio.emit(
            'flash_message',
            {
                "message": (
                    f"Scan mode active: X starts {scan_x_start_offset:g}mm and "
                    f"Y starts {scan_y_start_offset:g}mm before reference."
                )
            },
        )
        with robot_lock:
            ret = ensure_cps_connected()
            if ret != 0:
                print(f"Failed to connect, error code: {ret}")
                _inline_scan_active.clear()
                return jsonify({"error": f"Failed to connect to CPS client (ret={ret})"}), 500
            cps = CPS

        def read_ci_bit(cps, bit_index):
            """Reads CI bit and returns its value (0 or 1)."""
            result = []
            nRet = cps.HRIF_ReadBoxCI(0, bit_index, result)
            if nRet != 0:
                print(f"Error reading CI[{bit_index}], error code: {nRet}")
                return None
            return int(result[0])

        try:
            with robot_lock:
                # Read CI values
                ci0 = read_ci_bit(cps, 0)
                ci1 = read_ci_bit(cps, 1)
                ci2 = read_ci_bit(cps, 2)
                if ci0 is None or ci1 is None or ci2 is None:
                    return jsonify({'status': 'error', 'message': 'Failed to read tool sensor inputs (CI).'}), 500

                if ci0 == 0 and ci1 == 0 and ci2 == 0:
                    print("No tool in hand")
                elif ci0 == 1 and ci1 == 0 and ci2 == 0:
                    print("Tool 3 detected -> executing keepTool11()")
                    keepTool11(cps, toolNumber=3, config=config)
                    communicate(cps=cps, point=config['point']['safePoint'], tcp=config['coords']['tcpDefault'], ucs=config['coords']['ucsDefault'], seventh=-1, config=config, speed=0.9, wait=True)
                elif ci0 == 0 and ci1 == 0 and ci2 == 1:
                    print("Tool 4 detected -> executing keepTool11()")
                    keepTool11(cps, toolNumber=4, config=config)
                    communicate(cps=cps, point=config['point']['safePoint'], tcp=config['coords']['tcpDefault'], ucs=config['coords']['ucsDefault'], seventh=-1, config=config, speed=0.9, wait=True)
                elif ci0 == 0 and ci1 == 1 and ci2 == 0:
                    print("Tool 2 detected -> executing keepTool11()")
                    keepTool11(cps, toolNumber=2, config=config)
                    communicate(cps=cps, point=config['point']['safePoint'], tcp=config['coords']['tcpDefault'], ucs=config['coords']['ucsDefault'], seventh=-1, config=config, speed=0.9, wait=True)
                elif ci0 == 0 and ci1 == 1 and ci2 == 1:
                    print("Tool 1 detected -> executing keepTool11()")
                    keepTool11(cps, toolNumber=1, config=config)
                    communicate(cps=cps, point=config['point']['safePoint'], tcp=config['coords']['tcpDefault'], ucs=config['coords']['ucsDefault'], seventh=-1, config=config, speed=0.9, wait=True)
                else:
                    return jsonify({'status': 'error', 'message': f'Unrecognized CI combination: CI0={ci0}, CI1={ci1}, CI2={ci2}'}), 400

                stopper_result = _ensure_stopper_a_down(cps)
                if not stopper_result.get("success", False):
                    config["logger"].error("[scan] %s", stopper_result["message"])
                    return jsonify(
                        {
                            "status": "error",
                            "code": "stopper_a_interlock_failed",
                            "message": stopper_result["message"],
                        }
                    ), 500
                config["logger"].info(
                    "[scan] Stopper A down interlock confirmed (already=%s).",
                    bool(stopper_result.get("alreadyInState", False)),
                )
                socketio.emit(
                    'flash_message',
                    {"message": "Scan interlock confirmed: Stopper A down."},
                )
                laser(cps, "on", config=config)
                try:
                    scan_signature = str(request_data.get("tableAScanSignature") or "").strip()
                    scan_model = str(request_data.get("tableAModel") or "").strip()
                    scan_door_models = request_data.get("tableADoorModels") or []
                    if not isinstance(scan_door_models, list):
                        scan_door_models = []

                    configured_door_numbers = []
                    for door_entry in scan_door_models:
                        if not isinstance(door_entry, dict):
                            continue
                        model_name = str(door_entry.get("model") or "").strip()
                        if not model_name:
                            continue
                        door_num = door_entry.get("doorNumber")
                        try:
                            door_num = int(door_num)
                        except (TypeError, ValueError):
                            continue
                        if door_num > 0:
                            configured_door_numbers.append(door_num)
                    configured_door_numbers = sorted(set(configured_door_numbers))

                    runtime_scan_config = copy.deepcopy(config)
                    table_cfg = runtime_scan_config.get("table", {})
                    configured_door_count = len(configured_door_numbers)
                    if configured_door_count > 0:
                        base_count = int(table_cfg.get("count", configured_door_count) or configured_door_count)
                        # Never exceed configured table sections. This keeps length/lengthx indexing safe.
                        runtime_count = max(1, min(configured_door_count, base_count))
                        table_cfg["count"] = runtime_count
                        runtime_scan_config["table"] = table_cfg
                        config["logger"].info(
                            "[scan] Runtime door count set from selected models: %s (configured doors: %s)",
                            runtime_count,
                            configured_door_numbers,
                        )
                        socketio.emit(
                            'flash_message',
                            {"message": f"Scan runtime: using {runtime_count} configured door(s) for Table A."},
                        )

                    # scan_table owns the Table A 45-degree interlock. Do not
                    # issue the same table command twice from this route.
                    config["logger"].info(
                        "[scan] Delegating Table A 45-degree interlock to scan_table."
                    )
                    socketio.emit(
                        'flash_message',
                        {
                            "message": (
                                "Scan interlock mode: Table A command -> 45deg, "
                                "Table B command -> horizontal."
                            )
                        },
                    )
                    socketio.emit('flash_message', {"message": "Operation table mode: Table A 45°, Table B horizontal"})
                    _get_scan_table_a()(cps=cps, config=runtime_scan_config)
                finally:
                    laser(cps, "off", config=config)

                # Scan path now performs direct homing from scan_table after Door 1 Y-end.
                # Do not run additional scan homing here.
                config["logger"].info(
                    "[scan] Post-scan scanhoming is disabled; using direct homing from scan_table."
                )
                _set_tablea_scan_meta(
                    signature=scan_signature,
                    model=scan_model,
                    door_models=scan_door_models,
                )
                Thread(target=_post_scan_prepare_for_tablea_start, daemon=True).start()
                return jsonify(
                    {
                        'status': 'success',
                        'message': 'Table scan completed',
                        'scanStatus': _get_tablea_scan_status(),
                    }
                )
        except Exception as exc:
            if stop_requested():
                config['logger'].info("[scan] Scan cancelled by stop request: %s", exc)
                return jsonify({'status': 'cancelled', 'message': 'Scan stopped'}), 200
            config['logger'].exception("[scan] Scan action failed: %s", exc)
            return jsonify({'status': 'error', 'message': f'Scan failed: {exc}'}), 500
        finally:
            _inline_scan_active.clear()

    return jsonify({'status': 'error', 'message': 'Invalid action provided'}), 400

############################################################################################
if __name__ == '__main__':
    # Desktop/runtime mode should avoid Flask reloader duplicates.
    app.run(host='0.0.0.0', port=5100, debug=False, use_reloader=False)
