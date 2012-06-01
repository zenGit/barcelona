@echo off

setlocal

echo Rebuilding all projects in c55-connected-audio-fraemwork.....
echo CCS4 version: 
echo    4.2.4           (tested and supported)
echo.
echo Codgen tool version:
echo    4.3.9           (tested and supported)
echo.
echo DSP/BIOS version: 
echo    5.41.10.36      (tested and supprted)
echo.

pause

for /f %%i in ('cd') do set RET_CLICK_BUILD=%%i
cd c55_caf\build_script
call c55_caf_quick_build

echo Copying .out files to c55_caf_binaries directory...

cd /d %RET_CLICK_BUILD%
if exist c55_caf\build\CSL_USB_IsoFullSpeedExample_ezdsp_Out\Release\ (
copy /y c55_caf\build\CSL_USB_IsoFullSpeedExample_ezdsp_Out\Release\*.out   c55_caf_binaries 
)

if exist c55_caf\build\CSL_USB_IsoFullSpeedExample_Out\Release\ (
copy /y c55_caf\build\CSL_USB_IsoFullSpeedExample_Out\Release\*.out   c55_caf_binaries 
)


echo Creating boot images in c55_caf_binaries directory...
echo.

set HEX55=
if exist "C:\Program Files\Texas Instruments\ccsv4\tools\compiler\c5500\bin\hex55.exe" set HEX55="C:\Program Files\Texas Instruments\ccsv4\tools\compiler\c5500\bin\hex55.exe"
if exist "C:\Program Files (x86)\Texas Instruments\ccsv4\tools\compiler\c5500\bin\hex55.exe" set HEX55="C:\Program Files (x86)\ccsv4\tools\compiler\c5500\bin\hex55.exe"
if not defined HEX55 (
echo Error: hex55.exe not found!
echo.
goto end
)

for /r c55_caf_binaries %%i in (*.out) do (
if not exist %%~dpni_bootimg mkdir %%~dpni_bootimg
%HEX55% -boot -v5505 -serial8 -b -o %%~dpni_bootimg\bootimg.bin %%i
)

:end
echo If automatic build fails, please refer to readme.txt for detail instructions on how to build manually using CCS4.
echo.
pause

endlocal


