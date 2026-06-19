import os
import socket
import subprocess
import sys
import time
import urllib.request
from contextlib import suppress


ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BACKEND_DIR = os.path.join(ROOT, "Sanding_Cell_Code")
FRONTEND_DIR = os.path.join(ROOT, "Create_Login_Dashboard_Analytics")
FRONTEND_BUILD = os.path.join(FRONTEND_DIR, "build", "index.html")
UI_URL = "http://localhost:5100"


def _is_ready(url: str, timeout_s: float = 0.5) -> bool:
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
        time.sleep(0.2)
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


def _port_open(port: int, timeout_s: float = 0.1) -> bool:
    try:
        with socket.create_connection(("127.0.0.1", int(port)), timeout=timeout_s):
            return True
    except OSError:
        return False


def _kill_stale_servers() -> None:
    """Best-effort cleanup in case previous app session left children running."""
    if not (_port_open(5100) or _port_open(3000)):
        return
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
    if not os.path.exists(FRONTEND_BUILD):
        print(f"[launcher] Compiled frontend not found: {FRONTEND_BUILD}")
        print("[launcher] Run npm run build in Create_Login_Dashboard_Analytics.")
        return 1

    backend_proc = None
    _kill_stale_servers()
    try:
        backend_proc = subprocess.Popen(
            [sys.executable, "flask_app.py"],
            cwd=BACKEND_DIR,
            creationflags=subprocess.CREATE_NEW_PROCESS_GROUP,
        )
        # Importing PyWebView can take several seconds. Do it while Flask starts.
        import webview

        # Flask serves the compiled React build. Avoid starting Vite/Node in
        # production because its cold start is expensive on Windows/OneDrive.
        if not _wait_ready(UI_URL, total_timeout_s=60.0):
            print("[launcher] Flask UI did not become ready in time.")
            return 2

        webview.create_window("Sanding Cell", UI_URL, width=1400, height=900)
        webview.start()
        return 0
    finally:
        _stop_process(backend_proc)


if __name__ == "__main__":
    sys.exit(main())
