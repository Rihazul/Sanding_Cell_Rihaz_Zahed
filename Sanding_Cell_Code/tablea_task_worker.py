import importlib
import time
import yaml

from modules.CPS import CPSClient
from Server_Better_V2 import move_to_robot_homing_point, set_table_state, setup_logger, stopper_statusmod


MODEL_METHOD_MAP_TABLE_A = {
    "modelA": ("smallTable.smallmodelfinal", "sandingModelATableA"),
    "modelC": ("smallTable.smallmodelfinal3", "sandingModelCTableA"),
    "modelD": ("smallTable.smallmodelfinal4", "sandingModelDETableA"),
    "modelE": ("smallTable.smallmodelfinal5", "sandingModelETableA"),
    "modelF": ("smallTable.smallmodelfinalF", "sandingModelFTableA"),
}


def load_config():
    """Loads configuration from config.yaml."""
    with open("./configs/config.yaml", "r") as file:
        return yaml.safe_load(file)



def normalize_tablea_model_key(model_key):
    return "modelA" if model_key == "modelB" else model_key


def get_model_fn(model_key, logger=None):
    model_key = normalize_tablea_model_key(model_key)
    model_entry = MODEL_METHOD_MAP_TABLE_A.get(model_key)
    if model_entry is None:
        raise RuntimeError(f"Invalid Table A model key in child: {model_key}")
    module_name, function_name = model_entry
    import_start = time.monotonic()
    module = importlib.import_module(module_name)
    model_fn = getattr(module, function_name)
    if logger is not None:
        logger.info(
            "[TableA Child] model import complete in %.3fs (%s.%s)",
            time.monotonic() - import_start,
            module_name,
            function_name,
        )
    return model_fn


def run_tablea_task_child(model_key, recent_matching_scan=False):
    model_key = normalize_tablea_model_key(model_key)
    """
    Own CPS preparation inside the task child so Flask does not block the
    start-task request with parent CPS connect/table-state handoff work.
    """
    child_start = time.monotonic()
    print(
        f"[TableA Child] entered model={model_key} "
        f"recent_matching_scan={bool(recent_matching_scan)}"
    )
    config_start = time.monotonic()
    config = load_config()
    config_elapsed = time.monotonic() - config_start
    logger_start = time.monotonic()
    config["logger"] = setup_logger(config["settings"]["debug"])
    logger = config["logger"]
    logger_elapsed = time.monotonic() - logger_start

    logger.info(
        "[TableA Child] started model=%s recent_matching_scan=%s configLoad=%.3fs loggerSetup=%.3fs",
        model_key,
        bool(recent_matching_scan),
        config_elapsed,
        logger_elapsed,
    )
    model_fn = get_model_fn(model_key, logger=logger)

    cps = CPSClient()
    try:
        prep_start = time.monotonic()
        connect_start = time.monotonic()
        ret = cps.HRIF_Connect(0, config["server"]["cpip"], config["server"]["cps"])
        connect_elapsed = time.monotonic() - connect_start
        logger.info("[TableA Child] CPS connect finished in %.3fs ret=%s", connect_elapsed, ret)
        if ret != 0:
            raise RuntimeError(f"Table A child CPS connect failed (ret={ret})")

        # If a previous run was stopped mid Tool 2 side/edge, recover Tool 2 first, then move
        # the robot arm to homing before any table/stopper motion or sanding restart.
        try:
            from smallTable.tool2_stop_backoff import recover_tool2_after_stop_if_needed

            if recover_tool2_after_stop_if_needed(cps, config):
                logger.info("[TableA Child] Tool 2 stop recovery completed before task start")
                robot_speed = float(config.get("UI", {}).get("robotSpeed", 0.7))
                move_to_robot_homing_point(
                    cps,
                    config,
                    robot_speed,
                    velocity_profile="robotspeed",
                )
                logger.info("[TableA Child] Robot homing completed after Tool 2 recovery")
        except Exception as recovery_exc:
            logger.warning("[TableA Child] Tool 2 stop recovery skipped: %s", recovery_exc)

        # Operation routing uses swapped IO table IDs on this cell:
        # Table A operation requires Table A at 45 degrees and Table B horizontal.
        if recent_matching_scan:
            table_b_elapsed = 0.0
            table_b_result = {
                "success": True,
                "newState": "Open",
                "alreadyInState": True,
                "message": "Table B preparation reused from recent matching scan",
            }
            table_a_elapsed = 0.0
            table_a_already = True
            logger.info(
                "[TableA Child] Skipping Table A/Table B re-confirmation; "
                "recent matching scan already prepared operation table mode."
            )
        else:
            table_b_start = time.monotonic()
            table_b_result = set_table_state(cps, "tableBOpenClose", "Open")
            table_b_elapsed = time.monotonic() - table_b_start
            if not table_b_result.get("success", False):
                raise RuntimeError(
                    f"Table B preparation failed after {table_b_elapsed:.3f}s: "
                    f"{table_b_result.get('message', table_b_result)}"
                )

            table_a_start = time.monotonic()
            table_a_result = set_table_state(cps, "tableAOpenClose", "Close")
            table_a_elapsed = time.monotonic() - table_a_start
            table_a_already = bool(table_a_result.get("alreadyInState", False))
            if not table_a_result.get("success", False):
                raise RuntimeError(
                    f"Table A preparation failed after {table_a_elapsed:.3f}s: "
                    f"{table_a_result.get('message', table_a_result)}"
                )

        stopper_start = time.monotonic()
        stopper_statusmod(cps, state="up")
        stopper_elapsed = time.monotonic() - stopper_start
        logger.info(
            "[TableA Child] preparation complete in %.3fs "
            "(connect=%.3fs, tableB=%.3fs already=%s, tableA=%.3fs already=%s, stopper=%.3fs)",
            time.monotonic() - prep_start,
            connect_elapsed,
            table_b_elapsed,
            bool(table_b_result.get("alreadyInState", False)),
            table_a_elapsed,
            table_a_already,
            stopper_elapsed,
        )

        logger.info(
            "[TableA Child] Starting model task with shared CPS: %s "
            "(child elapsed before model call=%.3fs)",
            model_key,
            time.monotonic() - child_start,
        )
        model_fn(cps=cps)
    finally:
        disconnect_start = time.monotonic()
        try:
            cps.HRIF_DisConnect(0)
        except Exception:
            pass
        try:
            logger.info(
                "[TableA Child] CPS disconnect/final cleanup done in %.3fs totalChildSeconds=%.3fs",
                time.monotonic() - disconnect_start,
                time.monotonic() - child_start,
            )
        except Exception:
            pass
