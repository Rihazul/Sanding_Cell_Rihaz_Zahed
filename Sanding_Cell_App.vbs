Set WshShell = CreateObject("WScript.Shell")
scriptPath = Replace(WScript.ScriptFullName, "Sanding_Cell_App.vbs", "Sanding_Cell_App.ps1")
WshShell.Run "powershell -NoProfile -ExecutionPolicy Bypass -File """ & scriptPath & """", 0
Set WshShell = Nothing

