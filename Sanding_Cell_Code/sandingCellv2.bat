@echo off
REM Activate the virtual environment if you are using one
REM call "C:\Sanding Cell\SandingCell\venv\Scripts\activate"

REM Navigate to the directory where app.py is located
cd "C:\experimental\v03052025\Sanding with UI - Copy\Sanding_Cell_Technoaccord"

REM Run app.py first
echo Starting app.py...
start "" "C:/Users/Technoaccord Inc/AppData/Local/Programs/Python/Python38/python.exe" "C:/experimental/v03052025/Sanding with UI - Copy/Sanding_Cell_Technoaccord/app.py"

REM Wait a moment to ensure app.py has started
timeout /t 3

REM Run flask_app.py next
echo Starting flask_app.py...
"C:/Users/Technoaccord Inc/AppData/Local/Programs/Python/Python38/python.exe" "C:/experimental/v03052025/Sanding with UI - Copy/Sanding_Cell_Technoaccord/flask_app.py"
