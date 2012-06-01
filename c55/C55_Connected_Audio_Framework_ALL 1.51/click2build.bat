@echo off

setlocal

echo Rebuilding all projects in c55-connected-audio-fraemwork.....
echo CCS4 version: 
echo    4.2.4          (tested and supported)
echo.
echo Codgen tool version selection priorirty:
echo    1.    4.3.6    (tested and supported)
echo    2.    4.3.9    (tested and supported)
echo    3.    The default codegen tool in your CCS4 setup
echo.
echo DSP/BIOS version: 
echo    5.41.10.36     (tested and supprted)
echo.

pause

for /f %%i in ('cd') do set RET_CLICK_BUILD=%%i
cd c55-connected-audio-framework\build_script
call c55-connected-audio-framework_quick_build

echo Copying .out files to c55-connected-audio-framework_binaries directory...

cd /d %RET_CLICK_BUILD%
copy /y c55-connected-audio-framework\build\CSL_USB_IsoFullSpeedExample_ezdsp_Out\Release\*.out   c55-connected-audio-framework_binaries 

echo Creating boot images in c55-connected-audio-framework_binaries directory...
echo.

set HEX55=
if exist "C:\Program Files\Texas Instruments\ccsv4\tools\compiler\c5500\bin\hex55.exe" set HEX55="C:\Program Files\Texas Instruments\ccsv4\tools\compiler\c5500\bin\hex55.exe"
if exist "C:\Program Files (x86)\Texas Instruments\ccsv4\tools\compiler\c5500\bin\hex55.exe" set HEX55="C:\Program Files (x86)\ccsv4\tools\compiler\c5500\bin\hex55.exe"
if not defined HEX55 (
echo Error: hex55.exe not found!
echo.
goto end
)

for /r c55-connected-audio-framework_binaries %%i in (*.out) do (
if not exist %%~dpni_bootimg mkdir %%~dpni_bootimg
%HEX55% -boot -v5505 -serial8 -b -o %%~dpni_bootimg\bootimg.bin %%i
)

:end
echo If automatic build fails, please refer to readme.txt for detail instructions on how to build manually using CCS4.
echo.
pause

endlocal


