import importlib
import time
import yaml

from modules.CPS import CPSClient
from Server_Better_V2 import set_table_state, setup_logger, stopper_statusmod


MODEL_METHOD_MAP_TABLE_A = {
    "modelA": ("smallTable.smallmodelfinal", "sandingModelATableA"),
    "modelB": ("smallTable.smallmodelfinal2", "sandingModelBTableA"),
    "modelC": ("smallTable.smallmodelfinal3", "sandingModelCTableA"),
    "modelD": ("smallTable.smallmodelfinal4", "sandingModelDETableA"),
    "modelE": ("smallTable.smallmodelfinal5", "sandingModelETableA"),
    "modelF": ("smallTable.smallmodelfinalF", "sandingModelFTableA"),
}


def load_config():
    """Loads configuration from config.yaml."""
    with open("./configs/config.yaml", "r") as file:
        return yaml.safe_load(file)


def get_model_fn(model_key, logger=None):
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
    """
    Own CPS preparation inside the task child so Flask does not block the
    start-task request with parent CPS connect/table-state handoff work.
    """
    config = load_config()
    config["logger"] = setup_logger(config["settings"]["debug"])
    logger = config["logger"]
    child_start = time.monotonic()

    logger.info(
        "[TableA Child] started model=%s recent_matching_scan=%s",
        model_key,
        bool(recent_matching_scan),
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
        try:
            cps.HRIF_DisConnect(0)
        except Exception:
            pass
