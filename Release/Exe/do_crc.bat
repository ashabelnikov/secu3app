@echo off
rem Batch file for generating check sum for firmware of SECU-3 project
rem Created by Alexey A. Shabelnikov, Kiev 26 September 2009. 

set LOGFILE=crclog.txt
set HEXTOBIN=hextobin.exe
set CODECRC=codecrc.exe

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

rem Считаем и записываем контрольную сумму в бинарный файл
IF NOT EXIST %CODECRC% (
echo ERROR: Can not find file "%CODECRC%" >> %LOGFILE%
goto error
)
%CODECRC% secu-3_app.bin secu-3_app.bin  0  15870  3DFE -b >> %LOGFILE%
IF ERRORLEVEL 1 GOTO error

echo --------------------------------------------- >> %LOGFILE%
echo ALL OPERATIONS WERE COMPLETED SUCCESSFULLY! >> %LOGFILE%
GOTO exit

:error
echo --------------------------------------------- >> %LOGFILE%
echo WARNING! THERE ARE SOME ERRORS IN EXECUTING BATCH. >> %LOGFILE%

:exit