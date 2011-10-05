@echo off
rem Batch file for generating check sum for firmware of SECU-3 project
rem Created by Alexey A. Shabelnikov, Kiev 26 September 2009. 

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
rem Конвертируем HEX файл созданный компилятором в бинарный файл
%HEXTOBIN% secu-3_app.a90 secu-3_app.bin
IF ERRORLEVEL 1 GOTO error

rem Делаем копию файла без контрольной суммы
rem copy secu-3_app.bin secu-3_app0000.bin
rem copy secu-3_app.a90 secu-3_app0000.a90

rem Считаем и записываем контрольную сумму в бинарный файл
%CODECRC% secu-3_app.bin secu-3_app.a90  0  %FW_SIZE%  %CRC_ADDR% -h
IF ERRORLEVEL 1 GOTO error
%CODECRC% secu-3_app.bin secu-3_app.bin  0  %FW_SIZE%  %CRC_ADDR% -b
IF ERRORLEVEL 1 GOTO error

echo ALL OPERATIONS WERE COMPLETED SUCCESSFULLY!
exit 0

:error
exit 1
