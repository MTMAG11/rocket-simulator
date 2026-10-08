@echo off
rem Double-click to start the Autonomous Rocket Simulator (from source).
rem Uses the project's virtual environment (.venv) next to this file. Errors are shown, never hidden.
setlocal
cd /d "%~dp0"

if not exist ".venv\Scripts\python.exe" (
    echo.
    echo The virtual environment was not found:
    echo     %~dp0.venv
    echo.
    echo Create it once, in this folder, then double-click this file again:
    echo     py -3.11 -m venv .venv
    echo     .venv\Scripts\activate
    echo     pip install -e .
    echo.
    echo See README.md, "Quick Start".
    pause
    exit /b 1
)

".venv\Scripts\python.exe" -c "import rocket_sim, PySide6" >nul 2>nul
if errorlevel 1 (
    echo.
    echo The simulator is not installed in .venv yet. Install it once with:
    echo     .venv\Scripts\activate
    echo     pip install -e .
    echo.
    pause
    exit /b 1
)

".venv\Scripts\python.exe" -m rocket_sim %*
if errorlevel 1 (
    echo.
    echo The simulator stopped with an error ^(see the message above^).
    pause
    exit /b 1
)
endlocal
