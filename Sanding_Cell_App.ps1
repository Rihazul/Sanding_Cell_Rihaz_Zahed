$ErrorActionPreference = "Stop"
Add-Type -AssemblyName System.Windows.Forms

$root = Split-Path -Parent $MyInvocation.MyCommand.Path
$backendDir = Join-Path $root "Sanding_Cell_Code"

if (-not (Test-Path (Join-Path $backendDir "desktop_launcher.py"))) {
    [System.Windows.Forms.MessageBox]::Show(
        "desktop_launcher.py not found in Sanding_Cell_Code.",
        "Sanding Cell App",
        [System.Windows.Forms.MessageBoxButtons]::OK,
        [System.Windows.Forms.MessageBoxIcon]::Error
    ) | Out-Null
    exit 1
}

Set-Location $backendDir
uv run --no-sync desktop_launcher.py
