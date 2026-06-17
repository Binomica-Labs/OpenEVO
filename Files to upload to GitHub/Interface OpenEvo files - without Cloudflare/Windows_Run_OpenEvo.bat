@echo off

setlocal EnableDelayedExpansion

for /F %%a in ('echo prompt $E ^| cmd') do set "ESC=%%a"

cd /d "%~dp0"



echo !ESC![96m============================================!ESC![0m

echo !ESC![96m  Starting OpenEvo Interface (V1 - June 11, 2026)!ESC![0m

echo !ESC![96m============================================!ESC![0m

echo.

echo !ESC![97mBrowser will open at !ESC![93mhttp://localhost:8080!ESC![97m!ESC![0m

echo !ESC![90mPress Ctrl+C to stop.!ESC![0m

echo.



:: Auto-open the browser after 4 seconds (in background)

start "" /b cmd /c "timeout /t 4 /nobreak >nul && start http://localhost:8080"



python --version

if errorlevel 1 (

    echo.

    echo !ESC![91mERROR: Python not found. Run Windows_Install_OpenEvo.bat first.!ESC![0m

    echo.

    pause

    exit /b 1

)



python 2026-06-11_OpenEvo_Interface_V1.py

pause

