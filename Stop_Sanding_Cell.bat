@echo off
setlocal

set "ROOT=%~dp0"
set "BACKEND_MARKER=Sanding_Cell_Code"
set "FRONTEND_MARKER=Create_Login_Dashboard_Analytics"

echo Stopping Sanding Cell backend/frontend processes...

powershell -NoProfile -ExecutionPolicy Bypass -Command ^
  "$targets = Get-CimInstance Win32_Process | Where-Object { " ^
  "  ($_.CommandLine -and (($_.CommandLine -like '*uv run flask_app.py*') -or ($_.CommandLine -like '*flask_app.py*' -and $_.CommandLine -like '*%BACKEND_MARKER%*'))) -or " ^
  "  ($_.CommandLine -and (($_.CommandLine -like '*npm run dev*' -and $_.CommandLine -like '*%FRONTEND_MARKER%*') -or ($_.Name -eq 'node.exe' -and $_.CommandLine -like '*%FRONTEND_MARKER%*'))) " ^
  "}; " ^
  "if (-not $targets) { Write-Host 'No matching server processes found.'; exit 0 }; " ^
  "$targets | ForEach-Object { " ^
  "  try { Stop-Process -Id $_.ProcessId -Force -ErrorAction Stop; Write-Host ('Stopped PID ' + $_.ProcessId + ' : ' + $_.Name) } " ^
  "  catch { Write-Host ('Failed to stop PID ' + $_.ProcessId + ' : ' + $_.Name + ' -> ' + $_.Exception.Message) } " ^
  "}"

echo Done.
exit /b 0

