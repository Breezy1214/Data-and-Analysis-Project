@echo off
REM Shortest Path Project -- Windows launcher.
REM Detects Python, installs networkx + matplotlib if missing, runs the program.
setlocal
cd /d "%~dp0"

set "PY="
where python >nul 2>nul && set "PY=python"
if not defined PY where py >nul 2>nul && set "PY=py"

if not defined PY (
    echo Error: Python is not installed or not on PATH.
    echo Please install Python 3.9 or later from https://www.python.org/downloads/
    echo and make sure to check "Add Python to PATH" during installation.
    pause
    exit /b 1
)

%PY% -c "import networkx, matplotlib" >nul 2>nul
if errorlevel 1 (
    echo Installing required packages ^(networkx, matplotlib^) -- one-time setup...
    %PY% -m pip install --user --quiet networkx matplotlib
    if errorlevel 1 (
        echo.
        echo Failed to install dependencies automatically.
        echo Try manually:  pip install --user networkx matplotlib
        pause
        exit /b 1
    )
)

%PY% "%~dp0shortest_path.py"
pause
endlocal
