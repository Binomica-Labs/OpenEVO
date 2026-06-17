@echo off

setlocal EnableDelayedExpansion

for /F %%a in ('echo prompt $E ^| cmd') do set "ESC=%%a"



echo !ESC![96m============================================!ESC![0m

echo !ESC![96m  OpenEvo Installer (May 29, 2026 - V1.1)!ESC![0m

echo !ESC![96m============================================!ESC![0m

echo.

echo !ESC![97mUses your default Python (any Python 3.x).!ESC![0m

echo.



python --version

if errorlevel 1 (

    echo.

    echo !ESC![91mERROR: Python not found.!ESC![0m

    echo !ESC![93mInstall Python 3 from: https://www.python.org/downloads/!ESC![0m

    echo !ESC![93mTick "Add python.exe to PATH" during install, then re-run this file.!ESC![0m

    echo.

    pause

    exit /b 1

)



echo.

echo !ESC![94mInstalling packages...!ESC![0m

python -m pip install --upgrade pip

python -m pip install nicegui==3.4.1 pyserial



echo.

echo !ESC![92m============================================!ESC![0m

echo !ESC![92m  Installation Complete!!ESC![0m

echo !ESC![92m============================================!ESC![0m

echo.

echo !ESC![97mNext steps:!ESC![0m

echo   !ESC![96m1.!ESC![0m Upload firmware to Arduino

echo   !ESC![96m2.!ESC![0m Run Windows_Run_OpenEvo.bat

echo.

pause

