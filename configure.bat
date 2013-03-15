@echo off
rem Batch file for build firmware of SECU-3 project under MS Windows. This script will
rem configure Makefile for you and build project.
rem Created by Alexey A. Shabelnikov, Kiev 17 July 2011.
rem Note: It requires IAR compiler and linker or WinAvr

set USAGE=Supported platforms: M16,M32,M64  Supported toolchains: IAR,GCC
set PLATFORM=Undefined
set CFGFILE=platform_cfg
set MAKEFILE=Undefined

set CFG_MCU=Undefined
set CFG_BL_START=Undefined
set CFG_FWD_START=Undefined
set CFG_EE_SIZE=Undefined
set CFG_LNKXCL=Undefined

IF "%1" == "" (
echo Command line option required.
echo %USAGE%
exit 1
)

IF "%2" == "" (
echo Command line option required.
echo %USAGE%
exit 1
)

rem Check validity of command line option
IF %1 == M16 (
set CFG_BL_START=3E00
set CFG_FWD_START=3134
set CFG_EE_SIZE=512
set CFG_LNKXCL=lnkm16s.xcl
) else IF %1 == M32 (
set CFG_MCU=m32
set CFG_BL_START=7C00
set CFG_FWD_START=6F34
set CFG_EE_SIZE=1024
set CFG_LNKXCL=lnkm32s.xcl
) else IF %1 == M64 (
set CFG_MCU=m64
set CFG_BL_START=F800
set CFG_FWD_START=EB34
set CFG_EE_SIZE=2048
set CFG_LNKXCL=lnkm64s.xcl
) else (
echo Invalid platform!
echo %USAGE%
exit 1
)

IF %2 == IAR (
IF %1 == M16 (
set CFG_MCU=m16
) else IF %1 == M32 (
set CFG_MCU=m32
) else IF %1 == M64 (
set CFG_MCU=m64
)
set MAKEFILE=Makefile_iar
GOTO build
) else IF %2 == GCC (
IF %1 == M16 (
set CFG_MCU=atmega16
) else IF %1 == M32 (
set CFG_MCU=atmega32
) else IF %1 == M64 (
set CFG_MCU=atmega64
)
set MAKEFILE=Makefile_gcc
GOTO build
) else (
echo Invalid toolchain!
echo %USAGE%
exit 1
)

:build
rem Generate configuration file
IF EXIST %CFGFILE% del %CFGFILE%
echo MCU=%CFG_MCU%>> %CFGFILE%
echo BL_START=%CFG_BL_START%>> %CFGFILE%
echo FWD_START=%CFG_FWD_START%>> %CFGFILE%
echo EE_SIZE=%CFG_EE_SIZE%>> %CFGFILE%
echo LNKXCL=%CFG_LNKXCL%>> %CFGFILE%

make clean
copy %MAKEFILE% Makefile

IF NOT EXIST %MAKEFILE% (
 echo ERROR: Can not find file "%MAKEFILE%"
 goto error
)

make -f %MAKEFILE%
IF ERRORLEVEL 1 GOTO error

:error
exit 1
