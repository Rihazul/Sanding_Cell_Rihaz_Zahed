$ErrorActionPreference = "Stop"
Add-Type -AssemblyName System.Windows.Forms

$root = Split-Path -Parent $MyInvocation.MyCommand.Path
$backendDir = Join-Path $root "Sanding_Cell_Code"
$psLog = Join-Path $backendDir "launcher_ps.log"

function Write-LauncherLog($message) {
    try {
        $timestamp = Get-Date -Format "yyyy-MM-dd HH:mm:ss.fff"
        Add-Content -Path $psLog -Value "$timestamp $message"
    } catch {
        # Do not block app startup if logging fails.
    }
}

Write-LauncherLog "ps start root=$root backendDir=$backendDir"

if (-not (Test-Path (Join-Path $backendDir "desktop_launcher.py"))) {
    Write-LauncherLog "desktop_launcher.py missing"
    [System.Windows.Forms.MessageBox]::Show(
        "desktop_launcher.py not found in Sanding_Cell_Code.",
        "Sanding Cell App",
        [System.Windows.Forms.MessageBoxButtons]::OK,
        [System.Windows.Forms.MessageBoxIcon]::Error
    ) | Out-Null
    exit 1
}

$uvCmd = Get-Command uv -ErrorAction SilentlyContinue
if ($null -eq $uvCmd) {
    Write-LauncherLog "uv not found on PATH"
    [System.Windows.Forms.MessageBox]::Show(
        "uv was not found on PATH. Cannot start Sanding Cell app.",
        "Sanding Cell App",
        [System.Windows.Forms.MessageBoxButtons]::OK,
        [System.Windows.Forms.MessageBoxIcon]::Error
    ) | Out-Null
    exit 1
}
Write-LauncherLog "uv path=$($uvCmd.Source)"

Set-Location $backendDir
Write-LauncherLog "running uv run desktop_launcher.py"
try {
    uv run desktop_launcher.py
    Write-LauncherLog "uv run desktop_launcher.py exited normally"
} catch {
    Write-LauncherLog "uv run desktop_launcher.py failed: $($_.Exception.Message)"
    throw
}
