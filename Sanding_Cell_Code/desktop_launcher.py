import os
import signal
import subprocess
import sys
import time
import urllib.request
from contextlib import suppress

import webview


ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BACKEND_DIR = os.path.join(ROOT, "Sanding_Cell_Code")
FRONTEND_DIR = os.path.join(ROOT, "Create_Login_Dashboard_Analytics")
UI_URL = "http://localhost:3000"
BACKEND_URL = "http://127.0.0.1:5100"


def _is_ready(url: str, timeout_s: float = 2.0) -> bool:
    try:
        with urllib.request.urlopen(url, timeout=timeout_s) as resp:
            return 200 <= int(resp.status) < 500
    except Exception:
        return False


def _wait_ready(url: str, total_timeout_s: float) -> bool:
    start = time.time()
    while (time.time() - start) < total_timeout_s:
        if _is_ready(url):
            return True
        time.sleep(0.8)
    return False


def _taskkill_tree(pid: int) -> None:
    if pid <= 0:
        return
    with suppress(Exception):
        subprocess.run(
            ["taskkill", "/PID", str(pid), "/T", "/F"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )


def _stop_process(proc: subprocess.Popen) -> None:
    if proc is None:
        return
    with suppress(Exception):
        _taskkill_tree(proc.pid)
    with suppress(Exception):
        proc.wait(timeout=2.0)


def _kill_stale_servers() -> None:
    """Best-effort cleanup in case previous app session left children running."""
    cleanup_script = rf"""
$backendMarker = "Sanding_Cell_Code"
$frontendMarker = "Create_Login_Dashboard_Analytics"
$targets = Get-CimInstance Win32_Process | Where-Object {{
  ($_.CommandLine -and (($_.CommandLine -like "*flask_app.py*") -and ($_.CommandLine -like "*$backendMarker*"))) -or
  ($_.CommandLine -and (($_.CommandLine -like "*npm run dev*") -and ($_.CommandLine -like "*$frontendMarker*"))) -or
  ($_.Name -eq "node.exe" -and $_.CommandLine -and ($_.CommandLine -like "*$frontendMarker*"))
}}
foreach ($p in $targets) {{
  try {{ Stop-Process -Id $p.ProcessId -Force -ErrorAction Stop }} catch {{}}
}}
"""
    with suppress(Exception):
        subprocess.run(
            [
                "powershell",
                "-NoProfile",
                "-ExecutionPolicy",
                "Bypass",
                "-Command",
                cleanup_script,
            ],
            check=False,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )


def main() -> int:
    if not os.path.exists(os.path.join(BACKEND_DIR, "flask_app.py")):
        print(f"[launcher] Backend not found: {BACKEND_DIR}\\flask_app.py")
        return 1
    if not os.path.exists(os.path.join(FRONTEND_DIR, "package.json")):
        print(f"[launcher] Frontend not found: {FRONTEND_DIR}\\package.json")
        return 1

    backend_proc = None
    frontend_proc = None
    _kill_stale_servers()
    try:
        backend_proc = subprocess.Popen(
            [sys.executable, "flask_app.py"],
            cwd=BACKEND_DIR,
            creationflags=subprocess.CREATE_NEW_PROCESS_GROUP,
        )
        frontend_cmd = ["npm.cmd", "run", "dev"] if os.name == "nt" else ["npm", "run", "dev"]
        frontend_proc = subprocess.Popen(
            frontend_cmd,
            cwd=FRONTEND_DIR,
            creationflags=subprocess.CREATE_NEW_PROCESS_GROUP,
        )

        # Allow startup in background.
        _wait_ready(BACKEND_URL, total_timeout_s=30.0)
        if not _wait_ready(UI_URL, total_timeout_s=60.0):
            print("[launcher] Frontend did not become ready in time.")
            return 2

        webview.create_window("Sanding Cell", UI_URL, width=1400, height=900)
        webview.start()
        return 0
    finally:
        _stop_process(frontend_proc)
        _stop_process(backend_proc)


if __name__ == "__main__":
    sys.exit(main())
