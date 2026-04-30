Set WshShell = CreateObject("WScript.Shell")
WshShell.Run Chr(34) & Replace(WScript.ScriptFullName, "Stop_Sanding_Cell_Silent.vbs", "Stop_Sanding_Cell.bat") & Chr(34), 0
Set WshShell = Nothing

