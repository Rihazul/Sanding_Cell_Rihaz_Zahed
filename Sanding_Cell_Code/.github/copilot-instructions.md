<!-- Copilot/AI instructions for contributors and AI assistants -->
# Sanding Cell — AI Assistant Instructions

This file contains concise, project-specific guidance for AI coding assistants working in this repository. Focus on discoverable patterns and concrete commands.

1) Big picture
- **Architecture:** The app is a local Flask backend (`flask_app.py`) + front-end templates served on port `5100`. `app.py` wraps the Flask server using PyWebView for a desktop UI.
- **Hardware integration:** Robot/IO is accessed through a CPS client (see `modules/CPS.py`) and helper functions in `Server_Better_V2.py` (e.g. `getTool11`, `keepTool11`, `handle_client`). Network/IP settings live in `configs/config.yaml` (`server.cpip`, `server.cps`).
- **Long-running tasks:** Sanding/scan jobs spawn separate processes (multiprocessing `Process`) located in `model1cycle/`..`model5cycle/` and `smallTable/`. The Flask routes start these processes and guard them with `client_process` checks.

2) Typical developer workflows
- **Run locally (development):** Start the backend directly `python flask_app.py` (the UI expects `http://0.0.0.0:5100`) or launch the desktop wrapper `python app.py` (it starts `flask_app.py` via subprocess and opens PyWebView).
- **Build / packaging hints:** There are PyInstaller spec files: `app.spec` and `flask_app.spec`. There are helper batch scripts `bat2exeConverter.bat` and `sandingCellv2.bat` used by maintainers to produce Windows executables.
- **Generate function map:** Use the included scanner `map_functions.py` to generate `function_map.txt` for call-graph insight: `python map_functions.py`.

3) Project-specific conventions & gotchas
- **Config-first runtime:** Runtime values (robot TCPs, speeds, tool mappings) are in `configs/config.yaml`. Code often reads and merges YAML settings with UI modal data (`fetch_and_combine_data()` in `flask_app.py`).
- **Model entrypoints:** Each `modelNcycle` module exposes a start function named like `startingRobotToSandmodelN`. Use these names when tracing behavior. `flask_app.py` uses `modelMethodmap` and `modelMap` to dispatch.
- **IPC / hardware safety:** Before commanding hardware, routes create/inspect a CPS client and check sensor conditions. Avoid changing CPS calls without checking existing condition lists in `flask_app.py` (functions like `check_tool*_attachment_condition`).
- **File uploads:** 3D model uploads are saved to `3DModels/` and processed via `FileUtils/upload.py` (route: `/upload_3d_file`). Allowed extension: `.stp`.

4) Integration points to pay attention to
- `Server_Better_V2.py`: central for CPS helper functions and logging. Modifying call signatures here affects many routes.
- `configs/cycleData.json`: written by start routes and used by model workers. Treat it as a transient process handoff file.
- `templates/` and SocketIO events: UI uses SocketIO (`flask_socketio`) for realtime messages — changes to event names must be reflected in the frontend templates.

5) What to change and how to verify
- Prefer small, local changes: update YAML defaults in `configs/config.yaml` for tuning. To verify, run `python flask_app.py` and exercise the matching UI flows.
- When editing long-running process logic, ensure guard checks (`client_process and client_process.is_alive()`) remain in place to avoid starting duplicates.

6) Files to read first (quick tour)
- `flask_app.py` — primary backend, routes, process orchestration
- `app.py` — desktop wrapper that launches the server
- `Server_Better_V2.py` — CPS/hardware helpers
- `configs/config.yaml` — runtime settings and robot/tcp coordinates
- `model*cycle/` and `smallTable/` — concrete sanding/scan implementations
- `FileUtils/upload.py` — 3D model handling

If anything here is unclear or you'd like more detail on a specific area (packaging, a specific model cycle, or CPS commands), tell me which part to expand and I will iterate.
