@echo off
setlocal

set "ROOT=%~dp0"
set "BACKEND=%ROOT%Sanding_Cell_Code"
set "FRONTEND=%ROOT%Create_Login_Dashboard_Analytics"
set "UI_URL=http://localhost:3000"

if not exist "%BACKEND%\flask_app.py" (
  echo [ERROR] Backend not found: %BACKEND%\flask_app.py
  pause
  exit /b 1
)

if not exist "%FRONTEND%\package.json" (
  echo [ERROR] Frontend not found: %FRONTEND%\package.json
  pause
  exit /b 1
)

echo Starting Sanding backend...
start "Sanding Backend" powershell -NoProfile -ExecutionPolicy Bypass -Command "Set-Location '%BACKEND%'; uv run flask_app.py"

echo Starting Sanding frontend...
start "Sanding Frontend" powershell -NoProfile -ExecutionPolicy Bypass -Command "Set-Location '%FRONTEND%'; npm run dev"

echo Waiting for frontend server...
timeout /t 10 /nobreak >nul

where msedge >nul 2>nul
if %errorlevel%==0 (
  start "" msedge --app=%UI_URL%
  goto :done
)

where chrome >nul 2>nul
if %errorlevel%==0 (
  start "" chrome --app=%UI_URL%
  goto :done
)

start "" %UI_URL%

:done
echo Launched.
exit /b 0
