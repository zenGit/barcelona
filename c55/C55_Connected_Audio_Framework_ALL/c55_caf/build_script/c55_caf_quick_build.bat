@echo off

setlocal

for %%i in (..) do set #=%%~dpnxi
echo %#%
call c55_caf_build_all_local %#%

endlocal
