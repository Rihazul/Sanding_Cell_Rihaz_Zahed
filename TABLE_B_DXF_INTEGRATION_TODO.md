# Table B DXF — Stage 4 Follow-up: Robot Execution Integration

> **Status:** NOT DONE. Deferred to a separate focused session, to be tackled only
> after the DXF viewer + toolpath extraction is verified working end-to-end.
>
> Stages 2 (backend module + blueprint) and 3 (frontend viewer) make the DXF
> **viewer and coordinate extraction** work. They do **not** make the robot
> actually sand the selected DXF regions. This file records exactly what is still
> missing so it isn't lost after the merge session.

## The gap in one sentence

The DXF frontend can upload a DXF, select regions, extract a toolpath, and **save
an "approved toolpath" JSON on the backend** — but nothing connects that saved
JSON to the real robot execution path, so pressing "Start Task" for a DXF job does
not move the robot.

## What already exists after Stages 2–3

- **Frontend call:** `saveTableBDxfApprovedToolpath(jobId, payload)` in
  `Create_Login_Dashboard_Analytics/src/services/api.ts`
  → `POST /api/table-b-dxf/approved-toolpath/<job_id>`.
- **Backend endpoint:** `save_table_b_dxf_approved_toolpath` in
  `Sanding_Cell_Code/table_b_dxf/routes.py`, which calls
  `save_approved_toolpath(job_id, payload)` in
  `Sanding_Cell_Code/table_b_dxf/jobs.py`.
- **Persisted artifact:** `approved_toolpath.json` written under the job's
  `toolpaths/<job_id>/` folder (see `get_table_b_dxf_job_paths`). Contains the
  operator-approved toolpath segments + region corner points + `approved_at`.

So today the data flow **stops at a JSON file on disk**. No robot code reads it.

## What Stage 4 needs to build

### 1. A new execution endpoint (do NOT overload the legacy one)
- **Legacy path (leave intact):** `POST /start_TableB_process` in
  `Sanding_Cell_Code/flask_app.py` (~line 1070). It is **model-preset driven** —
  it looks the selected model up in `modelMap` / `modelMethodmap`, spawns a plot
  dialog process, then runs the robot. It has no concept of an arbitrary
  DXF-derived toolpath.
- **New path (to add):** a route such as `POST /api/table-b-dxf/start/<job_id>`
  (or a `start_TableB_dxf_process`) that:
  1. Loads the approved toolpath via `load_approved_toolpath(job_id)` (already
     exists in `jobs.py`).
  2. Validates a job is not already running (mirror the
     `client_process.is_alive()` / `client_thread.is_alive()` guard used by
     `start_TableB_process`).
  3. Translates the DXF toolpath segments (2D DXF coordinates + per-region
     tool/force/cycle) into the robot motion command format the existing worker
     consumes.
  4. Dispatches to the robot the same way the legacy Table B path does
     (`Process(...)` / the client process/thread machinery in `flask_app.py`).

### 2. The worker / process it must hook into
- Legacy Table B execution is driven from `flask_app.py` via the
  `client_process` / `client_thread` globals and a spawned `Process`
  (`_run_tableb_plot_dialog` and the model-method machinery).
- Table A's equivalent worker is `Sanding_Cell_Code/tablea_task_worker.py` — a
  useful reference for how a task worker is structured, but Table B currently
  runs through its own `flask_app.py` path, not a standalone worker file.
- **Decision for Stage 4:** either (a) extend the existing Table B process path to
  accept a toolpath payload instead of a model preset, or (b) add a small
  dedicated Table B DXF worker mirroring `tablea_task_worker.py`. Prefer whichever
  keeps the legacy model-preset flow untouched.

### 3. Coordinate / frame transform (the real work)
- The DXF toolpath is in **DXF/CAD 2D coordinates**. The robot needs **table/world
  coordinates** (plus Z, tool orientation, approach/retract).
- Stage 4 must define and apply the DXF-plane → Table-B-robot-frame transform
  (origin, scale/units, rotation, Z sanding height, retract height). This is the
  single most important correctness item and needs on-machine calibration.

## Data flow to wire (summary)

```
[React viewer]
  select regions -> extract toolpath -> approve
        |
        v
POST /api/table-b-dxf/approved-toolpath/<job_id>     (EXISTS)
        |
        v
toolpaths/<job_id>/approved_toolpath.json            (EXISTS on disk)
        |
        X  <-- MISSING LINK (Stage 4)
        v
POST /api/table-b-dxf/start/<job_id>                  (TO BUILD)
  -> load_approved_toolpath(job_id)                   (EXISTS in jobs.py)
  -> transform DXF coords -> robot frame              (TO BUILD)
  -> dispatch via flask_app.py Table B process path   (hook into EXISTING machinery)
        |
        v
     Robot moves / sands selected DXF regions
```

## Payload shape reference
The exact `approved_toolpath.json` schema is whatever the frontend sends to
`saveTableBDxfApprovedToolpath`. Before Stage 4, capture one real approved job's
JSON from `Sanding_Cell_Code/table_b_dxf/toolpaths/<job_id>/approved_toolpath.json`
and use it as the contract for the transform + dispatch code.

## Explicitly out of scope for the current merge
- No changes to `/start_TableB_process` or the legacy model-preset flow.
- No robot motion code added in Stages 2–3.
- No coordinate transform implemented yet.
