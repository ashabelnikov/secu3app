@echo off
rem Batch file for generating check sum for firmware of SECU-3 project
rem Created by Alexey A. Shabelnikov, Kiev 26 September 2009. 

set LOGFILE=crclog.txt
set HEXTOBIN=hextobin.exe
set CODECRC=codecrc.exe
set USAGE=Supported options: M16,M32,M64
set FW_SIZE=Undefined
set CRC_ADDR=Undefined

IF "%1" == "" (
echo Command line option required.
echo %USAGE%
exit 1
)

rem Check validity of command line option and set corresponding parameters
IF %1 == M16 ( 
set FW_SIZE=15870
set CRC_ADDR=3DFE
GOTO dowork
)

IF %1 == M32 ( 
set FW_SIZE=31742
set CRC_ADDR=7BFE
GOTO dowork
)

IF %1 == M64 ( 
set FW_SIZE=63486
set CRC_ADDR=F7FE
GOTO dowork
)

echo Invalid platform! 
echo %USAGE%
exit 1

:dowork
echo See %LOGFILE% for detailed information.
IF EXIST %LOGFILE% del %LOGFILE%

echo EXECUTING BATCH... >> %LOGFILE%
echo --------------------------------------------- >> %LOGFILE%

rem Конвертируем HEX файл созданный компилятором в бинарный файл
IF NOT EXIST %HEXTOBIN% (
 echo ERROR: Can not find file "%HEXTOBIN%" >> %LOGFILE%
 goto error
)
%HEXTOBIN% secu-3_app.a90 secu-3_app.bin >> %LOGFILE%
IF ERRORLEVEL 1 GOTO error

rem Делаем копию файла без контрольной суммы
copy secu-3_app.bin secu-3_app0000.bin
copy secu-3_app.a90 secu-3_app0000.a90

rem Считаем и записываем контрольную сумму в бинарный файл
IF NOT EXIST %CODECRC% (
echo ERROR: Can not find file "%CODECRC%" >> %LOGFILE%
goto error
)
%CODECRC% secu-3_app.bin secu-3_app.a90  0  %FW_SIZE%  %CRC_ADDR% -h >> %LOGFILE%
IF ERRORLEVEL 1 GOTO error
%CODECRC% secu-3_app.bin secu-3_app.bin  0  %FW_SIZE%  %CRC_ADDR% -b >> %LOGFILE%
IF ERRORLEVEL 1 GOTO error

echo --------------------------------------------- >> %LOGFILE%
echo ALL OPERATIONS WERE COMPLETED SUCCESSFULLY! >> %LOGFILE%
GOTO exit

:error
echo --------------------------------------------- >> %LOGFILE%
echo WARNING! THERE ARE SOME ERRORS IN EXECUTING BATCH. >> %LOGFILE%

:exit