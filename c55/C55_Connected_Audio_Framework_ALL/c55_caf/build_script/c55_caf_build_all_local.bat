@echo off
rem  Global environment variables: None
rem  Argument
rem    %1		: root directory for c55-connected-audio-framework project
rem    %2               : product               
rem    %3               : clean   
rem    %4               : codegen tool version
rem    %5               : DSP/BIOS version
rem  Usage:
rem    call c55-connected-audio-framework_build_all_local <root directory for c55-connected-audio-framework project> ......

setlocal

for /f %%i in ('cd') do set RET_DIRUSB=%%i

set CCS4_DIR=
if exist "C:\Program Files\Texas Instruments\ccsv4" set CCS4_DIR="C:\Program Files\Texas Instruments\ccsv4" 
if exist "C:\Program Files (x86)\Texas Instruments\ccsv4" set CCS4_DIR="C:\Program Files (x86)\Texas Instruments\ccsv4" 
if not defined CCS4_DIR (
echo Error: CCS4 not found!
goto end
)

rem set /p #=Use CCS4 to select workspace, codegen tool version, DSP/BIOS version and save. Then press any key.

if not exist %1\.metadata\ (
mkdir %1\.metadata
) else (
rmdir /s /q %1\.metadata
mkdir %1\.metadata
)

rem set #=%1

rem for %%i in (%#%) do set #=%%~nxi

rem echo %#%\.metadata %1\.metadata

rem pause

rem xcopy %#%\.metadata %1\.metadata /e /q /y
xcopy .metadata %1\.metadata /e /q /y

rem pause

rem ***** Import projects*****

SET CCS_WORKSPACE=%1

SET USB_AC_DIR=%1

cd /d %CCS4_DIR%\eclipse

jre\bin\java -jar startup.jar -data "%CCS_WORKSPACE%" -application com.ti.ccstudio.apps.projectImport -ccs.location "%USB_AC_DIR%\c55xx_csl\build\cslVC5505"

jre\bin\java -jar startup.jar -data "%CCS_WORKSPACE%" -application com.ti.ccstudio.apps.projectImport -ccs.location "%USB_AC_DIR%\c55x5_drivers\pal_os\build\palos_bios_lib"

jre\bin\java -jar startup.jar -data "%CCS_WORKSPACE%" -application com.ti.ccstudio.apps.projectImport -ccs.location "%USB_AC_DIR%\c55x5_drivers\pal_sys\build\pal_sys_bios_lib"

jre\bin\java -jar startup.jar -data "%CCS_WORKSPACE%" -application com.ti.ccstudio.apps.projectImport -ccs.location "%USB_AC_DIR%\c55x5_drivers\dma\build\dma_bios_drv_lib"

jre\bin\java -jar startup.jar -data "%CCS_WORKSPACE%" -application com.ti.ccstudio.apps.projectImport -ccs.location "%USB_AC_DIR%\c55x5_drivers\i2c\build\i2c_bios_drv_lib"

jre\bin\java -jar startup.jar -data "%CCS_WORKSPACE%" -application com.ti.ccstudio.apps.projectImport -ccs.location "%USB_AC_DIR%\c55x5_drivers\i2s\build\i2s_bios_drv_lib"

if exist "%USB_AC_DIR%\build\CSL_USB_IsoFullSpeedExample_Out" jre\bin\java -jar startup.jar -data "%CCS_WORKSPACE%" -application com.ti.ccstudio.apps.projectImport -ccs.location "%USB_AC_DIR%\build\CSL_USB_IsoFullSpeedExample_Out"

if exist "%USB_AC_DIR%\build\CSL_USB_IsoFullSpeedExample_ezdsp_Out" jre\bin\java -jar startup.jar -data "%CCS_WORKSPACE%" -application com.ti.ccstudio.apps.projectImport -ccs.location "%USB_AC_DIR%\build\CSL_USB_IsoFullSpeedExample_ezdsp_Out"

rem ***** Rebuild projects *****

SET CCS_WORKSPACE=%1

cd /d %CCS4_DIR%\eclipse

jre\bin\java -jar startup.jar -data "%CCS_WORKSPACE%" -application com.ti.ccstudio.apps.projectBuild -ccs.projects cslVC5505 -ccs.configuration Release

jre\bin\java -jar startup.jar -data "%CCS_WORKSPACE%" -application com.ti.ccstudio.apps.projectBuild -ccs.projects palos_bios_lib -ccs.configuration Release

jre\bin\java -jar startup.jar -data "%CCS_WORKSPACE%" -application com.ti.ccstudio.apps.projectBuild -ccs.projects pal_sys_bios_lib -ccs.configuration Release

jre\bin\java -jar startup.jar -data "%CCS_WORKSPACE%" -application com.ti.ccstudio.apps.projectBuild -ccs.projects dma_bios_drv_lib -ccs.configuration Release

jre\bin\java -jar startup.jar -data "%CCS_WORKSPACE%" -application com.ti.ccstudio.apps.projectBuild -ccs.projects i2c_bios_drv_lib -ccs.configuration Release

jre\bin\java -jar startup.jar -data "%CCS_WORKSPACE%" -application com.ti.ccstudio.apps.projectBuild -ccs.projects i2s_bios_drv_lib -ccs.configuration Release

if exist "%USB_AC_DIR%\build\CSL_USB_IsoFullSpeedExample_Out" jre\bin\java -jar startup.jar -data "%CCS_WORKSPACE%" -application com.ti.ccstudio.apps.projectBuild -ccs.projects CSL_USB_IsoFullSpeedExample_Out -ccs.configuration Release

if exist "%USB_AC_DIR%\build\CSL_USB_IsoFullSpeedExample_ezdsp_Out" jre\bin\java -jar startup.jar -data "%CCS_WORKSPACE%" -application com.ti.ccstudio.apps.projectBuild -ccs.projects CSL_USB_IsoFullSpeedExample_ezdsp_Out -ccs.configuration Release

cd /d %RET_DIRUSB%

if /i not "%3"=="clean" goto end

rem ***** Clean *****

call c55-connected-audio-framework_cleanall %1

:end

cd /d %RET_DIRUSB%

rem pause

endlocal





