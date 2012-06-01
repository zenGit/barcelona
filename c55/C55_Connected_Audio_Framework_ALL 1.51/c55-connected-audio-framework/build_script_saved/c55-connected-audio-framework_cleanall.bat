rem Global environment variables:
rem  Argument
rem    %1		: root directory for usb audio class project
rem  Usage:
rem    call c55_connected-audio-framework_cleanall <root directory for usb audio class project>

setlocal

for /f %%i in ('cd') do set RET_DIRUSBCL=%%i

set CCS4_DIR="C:\Program Files\Texas Instruments\ccsv4"

SET CCS_WORKSPACE=%1

set USB_AC_DIR=%1

cd /d %CCS4_DIR%\eclipse

jre\bin\java -jar startup.jar -data "%CCS_WORKSPACE%" -application com.ti.ccstudio.apps.projectBuild -ccs.projects cslVC5505 -ccs.configuration Release -ccs.clean

jre\bin\java -jar startup.jar -data "%CCS_WORKSPACE%" -application com.ti.ccstudio.apps.projectBuild -ccs.projects palos_bios_lib -ccs.configuration Release -ccs.clean

jre\bin\java -jar startup.jar -data "%CCS_WORKSPACE%" -application com.ti.ccstudio.apps.projectBuild -ccs.projects pal_sys_bios_lib -ccs.configuration Release -ccs.clean

jre\bin\java -jar startup.jar -data "%CCS_WORKSPACE%" -application com.ti.ccstudio.apps.projectBuild -ccs.projects dma_bios_drv_lib -ccs.configuration Release -ccs.clean

jre\bin\java -jar startup.jar -data "%CCS_WORKSPACE%" -application com.ti.ccstudio.apps.projectBuild -ccs.projects i2c_bios_drv_lib -ccs.configuration Release -ccs.clean

jre\bin\java -jar startup.jar -data "%CCS_WORKSPACE%" -application com.ti.ccstudio.apps.projectBuild -ccs.projects i2s_bios_drv_lib -ccs.configuration Release -ccs.clean

if exist "%USB_AC_DIR%\build\CSL_USB_IsoFullSpeedExample_Out" jre\bin\java -jar startup.jar -data "%CCS_WORKSPACE%" -application com.ti.ccstudio.apps.projectBuild -ccs.projects CSL_USB_IsoFullSpeedExample_Out -ccs.configuration Release -ccs.clean

if exist "%USB_AC_DIR%\build\CSL_USB_IsoFullSpeedExample_ezdsp_Out" jre\bin\java -jar startup.jar -data "%CCS_WORKSPACE%" -application com.ti.ccstudio.apps.projectBuild -ccs.projects CSL_USB_IsoFullSpeedExample_ezdsp_Out -ccs.configuration Release -ccs.clean

cd /d %RET_DIRUSBCL%

endlocal

