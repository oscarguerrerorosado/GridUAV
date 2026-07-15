@echo off

REM Activate the conda environment
call conda activate griduav

echo Launching Webots simulation
start "Webots Simulator" cmd /k "echo This window runs the Webots simulator (SquareBox.wbt). Leave it open while the experiment runs. && "%WEBOTS%\webots.exe" "%GRIDUAV%\Webots\SquareBox.wbt""
 
REM Give Webots a moment to fully open before attaching the controller
timeout /t 10 /nobreak >nul
 
echo Launching controller
"%WEBOTS%\webots-controller.exe" "%GRIDUAV%\Experiments\Experiment1.py"
 