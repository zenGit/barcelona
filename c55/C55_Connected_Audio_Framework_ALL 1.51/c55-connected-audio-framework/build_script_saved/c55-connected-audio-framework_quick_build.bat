@echo off

setlocal

for %%i in (..) do set #=%%~dpnxi
echo %#%
call c55-connected-audio-framework_build_all_local %#%

endlocal
