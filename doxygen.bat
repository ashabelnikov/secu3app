@echo off
rem SECU-3  - An open source, free engine control unit
rem Copyright (C) 2007 Alexey A. Shabelnikov. Ukraine, Kiev
rem 
rem Batch file for generating of doxygen documentation on firmware of SECU-3 project
rem Created by Alexey A. Shabelnikov, Kiev 29 October 2010. 

set DOXYGEN=doxygen.exe

rem Delete previous content
for /d %%i in ("doc\*") do rmdir /s /q "%%i"

rem Generate documentation
for %%X in (%DOXYGEN%) do (set FOUND_OX=%%~$PATH:X)
if not defined FOUND_OX (
 echo ERROR: Can not find file "%DOXYGEN%"
 goto error
)

%DOXYGEN% doxyconf
exit 0

:error
exit 1
