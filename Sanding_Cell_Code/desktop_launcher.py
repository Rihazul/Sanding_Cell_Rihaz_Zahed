import os
import socket
import subprocess
import sys
import threading
import time
import urllib.request
from contextlib import suppress


ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BACKEND_DIR = os.path.join(ROOT, "Sanding_Cell_Code")
FRONTEND_DIR = os.path.join(ROOT, "Create_Login_Dashboard_Analytics")
FRONTEND_BUILD = os.path.join(FRONTEND_DIR, "build", "index.html")
UI_URL = "http://localhost:5100"
LAUNCHER_LOG = os.path.join(BACKEND_DIR, "launcher_startup.log")


def _log(message: str) -> None:
    try:
        stamp = time.strftime("%Y-%m-%d %H:%M:%S")
        with open(LAUNCHER_LOG, "a", encoding="utf-8") as f:
            f.write(f"{stamp} {message}\n")
    except Exception:
        pass


def _loading_html() -> str:
    return """
<!doctype html>
<html>
<head>
  <meta charset="utf-8" />
  <style>
    html, body { height: 100%; margin: 0; }
    body { display: flex; align-items: center; justify-content: center; background: linear-gradient(135deg, #ecfeff 0%, #dbeafe 48%, #eef2ff 100%); font-family: Segoe UI, Tahoma, sans-serif; color: #0f172a; }
    .card { width: min(460px, calc(100vw - 48px)); border-radius: 28px; background: rgba(255,255,255,.86); box-shadow: 0 30px 90px rgba(15,23,42,.20); border: 1px solid rgba(255,255,255,.85); padding: 38px 34px; text-align: center; backdrop-filter: blur(10px); }
    .mark { width: 70px; height: 70px; margin: 0 auto 18px; border-radius: 22px; display: grid; place-items: center; color: white; font-size: 30px; font-weight: 900; background: linear-gradient(135deg, #0891b2, #2563eb); box-shadow: 0 14px 35px rgba(37,99,235,.28); }
    .title { font-size: 26px; font-weight: 900; letter-spacing: -.03em; }
    .sub { margin-top: 10px; color: #475569; font-size: 15px; }
    .bar { margin: 26px auto 0; width: 220px; height: 8px; border-radius: 999px; overflow: hidden; background: #dbeafe; }
    .bar span { display: block; width: 44%; height: 100%; border-radius: 999px; background: linear-gradient(90deg, #06b6d4, #2563eb); animation: slide 1.05s infinite ease-in-out; }
    @keyframes slide { 0% { transform: translateX(-110%); } 100% { transform: translateX(260%); } }
  </style>
</head>
<body><div class="card"><div class="mark">S</div><div class="title">Starting Sanding Cell</div><div class="sub">Preparing robot dashboard and local server...</div><div class="bar"><span></span></div></div></body>
</html>
"""


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


def _load_when_ready(window) -> None:
    wait_start = time.time()
    _log("wait ready begin")
    if _wait_ready(UI_URL, total_timeout_s=60.0):
        _log(f"wait ready done elapsed={time.time() - wait_start:.3f}s")
        with suppress(Exception):
            window.load_url(UI_URL)
    else:
        _log(f"wait ready timeout elapsed={time.time() - wait_start:.3f}s")
        with suppress(Exception):
            window.load_html(
                "<h2 style='font-family:Segoe UI;padding:32px'>Sanding Cell server did not become ready.</h2>"
            )


def main() -> int:
    main_start = time.time()
    _log("launcher main start")
    if not os.path.exists(os.path.join(BACKEND_DIR, "flask_app.py")):
        print(f"[launcher] Backend not found: {os.path.join(BACKEND_DIR, 'flask_app.py')}")
        _log("backend missing")
        return 1
    if not os.path.exists(FRONTEND_BUILD):
        print(f"[launcher] Compiled frontend not found: {FRONTEND_BUILD}")
        print("[launcher] Run npm run build in Create_Login_Dashboard_Analytics.")
        _log("frontend build missing")
        return 1

    backend_proc = None
    cleanup_start = time.time()
    _kill_stale_servers()
    _log(f"stale cleanup elapsed={time.time() - cleanup_start:.3f}s")
    try:
        backend_start = time.time()
        backend_proc = subprocess.Popen(
            [sys.executable, "flask_app.py"],
            cwd=BACKEND_DIR,
            creationflags=subprocess.CREATE_NEW_PROCESS_GROUP,
        )
        _log(f"backend popen elapsed={time.time() - backend_start:.3f}s pid={backend_proc.pid}")

        webview_import_start = time.time()
        import webview
        _log(f"webview import elapsed={time.time() - webview_import_start:.3f}s")

        window_start = time.time()
        window = webview.create_window("Sanding Cell", html=_loading_html(), width=1400, height=900)
        _log(f"webview create_window elapsed={time.time() - window_start:.3f}s")
        _log(f"webview start begin totalBeforeStart={time.time() - main_start:.3f}s")
        webview.start(
            lambda: threading.Thread(
                target=_load_when_ready,
                args=(window,),
                daemon=True,
            ).start()
        )
        _log(f"webview closed total={time.time() - main_start:.3f}s")
        return 0
    finally:
        stop_start = time.time()
        _stop_process(backend_proc)
        _log(f"backend stop elapsed={time.time() - stop_start:.3f}s")


if __name__ == "__main__":
    sys.exit(main())
