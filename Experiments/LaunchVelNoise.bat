@echo off
REM Activate the conda environment
call conda activate griduav

REM Check if Webots is already running
tasklist /FI "IMAGENAME eq webots.exe" 2>NUL | find /I "webots.exe" >NUL

if "%ERRORLEVEL%"=="0" (
    echo Webots is already running. Skipping launch.
) else (
    echo Launching Webots simulation
    start "Webots Simulator" cmd /c "echo This window runs the Webots simulator (SquareBox.wbt). Leave it open while the experiment runs. && "%WEBOTS%\webots.exe" "%GRIDUAV%\Webots\SquareBox.wbt""

    REM Give Webots a moment to fully open before attaching the controller
    timeout /t 10 /nobreak >nul
)

echo Launching VelNoise Experiment
"%WEBOTS%\webots-controller.exe" "%GRIDUAV%\Experiments\VelNoise.py"