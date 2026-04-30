$ErrorActionPreference = "Stop"
Add-Type -AssemblyName System.Windows.Forms

$root = Split-Path -Parent $MyInvocation.MyCommand.Path
$backendDir = Join-Path $root "Sanding_Cell_Code"
$frontendDir = Join-Path $root "Create_Login_Dashboard_Analytics"
$uiUrl = "http://localhost:5173"

function Stop-ProcessTree {
    param([int]$Pid)
    if ($Pid -le 0) { return }
    try {
        $proc = Get-Process -Id $Pid -ErrorAction SilentlyContinue
        if ($null -ne $proc) {
            & taskkill /PID $Pid /T /F | Out-Null
        }
    } catch {
        # best effort
    }
}

function Start-ManagedProcess {
    param(
        [string]$FilePath,
        [string[]]$ArgumentList,
        [string]$WorkingDirectory
    )
    return Start-Process `
        -FilePath $FilePath `
        -ArgumentList $ArgumentList `
        -WorkingDirectory $WorkingDirectory `
        -WindowStyle Hidden `
        -PassThru
}

function Wait-UrlReady {
    param(
        [string]$Url,
        [int]$TimeoutSec = 60
    )
    $deadline = (Get-Date).AddSeconds($TimeoutSec)
    while ((Get-Date) -lt $deadline) {
        try {
            $res = Invoke-WebRequest -Uri $Url -Method Get -UseBasicParsing -TimeoutSec 2
            if ($res.StatusCode -ge 200) { return $true }
        } catch {
            # keep waiting
        }
        Start-Sleep -Milliseconds 800
    }
    return $false
}

$backendProc = $null
$frontendProc = $null
$appProc = $null

try {
    if (-not (Test-Path (Join-Path $backendDir "flask_app.py"))) {
        throw "Backend file not found: $backendDir\flask_app.py"
    }
    if (-not (Test-Path (Join-Path $frontendDir "package.json"))) {
        throw "Frontend file not found: $frontendDir\package.json"
    }

    $backendProc = Start-ManagedProcess -FilePath "uv" -ArgumentList @("run", "flask_app.py") -WorkingDirectory $backendDir
    $frontendProc = Start-ManagedProcess -FilePath "npm.cmd" -ArgumentList @("run", "dev") -WorkingDirectory $frontendDir

    [void](Wait-UrlReady -Url $uiUrl -TimeoutSec 60)

    $edgeX86 = Join-Path ${env:ProgramFiles(x86)} "Microsoft\Edge\Application\msedge.exe"
    $edgeX64 = Join-Path $env:ProgramFiles "Microsoft\Edge\Application\msedge.exe"
    $chrome = Join-Path $env:ProgramFiles "Google\Chrome\Application\chrome.exe"

    if (Test-Path $edgeX86) {
        $appProc = Start-Process -FilePath $edgeX86 -ArgumentList @("--app=$uiUrl", "--new-window") -PassThru
    } elseif (Test-Path $edgeX64) {
        $appProc = Start-Process -FilePath $edgeX64 -ArgumentList @("--app=$uiUrl", "--new-window") -PassThru
    } elseif (Test-Path $chrome) {
        $appProc = Start-Process -FilePath $chrome -ArgumentList @("--app=$uiUrl", "--new-window") -PassThru
    } else {
        throw "Neither Microsoft Edge nor Google Chrome was found. Install one to run app mode."
    }

    Wait-Process -Id $appProc.Id
}
catch {
    [System.Windows.Forms.MessageBox]::Show(
        "Sanding Cell launcher failed:`n$($_.Exception.Message)",
        "Sanding Cell App",
        [System.Windows.Forms.MessageBoxButtons]::OK,
        [System.Windows.Forms.MessageBoxIcon]::Error
    ) | Out-Null
}
finally {
    if ($null -ne $frontendProc) { Stop-ProcessTree -Pid $frontendProc.Id }
    if ($null -ne $backendProc) { Stop-ProcessTree -Pid $backendProc.Id }
}
