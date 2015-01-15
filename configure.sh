#!/bin/sh

# Script for building of the SECU-3 project firmware under Linux. This script will
# configure Makefile for you and build project.
# Created by Alexey A. Shabelnikov, Kiev 24 July 2011.
# Note: It requires AVR-GCC toolchain

USAGE="USAGE: configure.sh <platform> <toolchain> \n Supported platforms: M32,M64,M644, Supported toolchains: IAR, GCC"
PLATFORM=Undefined
CFGFILE="platform_cfg"
MAKEFILE="Makefile_gcc"

CFG_MCU=Undefined
CFG_BL_START=Undefined
CFG_FWD_START=Undefined
CFG_EE_SIZE=Undefined
CFG_LNKXCL=Undefined
TC=Undefined

#check number of command line arguments
if [ $# -eq 1 ]
then
 TC="GCC"
elif [ $# -eq 2 ]
then
 TC=$2
else
 echo "Invalid command line arguments."
 echo " "$USAGE
 exit 1  #error
fi

# Check validity of command line option
if [ $1 = "M32" ]
then
 CFG_BL_START="7C00"
 CFG_FWD_START="5404"
 CFG_EE_SIZE="1024"
 CFG_LNKXCL="lnkm32s.xcl"
elif [ $1 = "M64" ]
then
 CFG_BL_START="F800"
 CFG_FWD_START="D004"
 CFG_EE_SIZE="2048"
 CFG_LNKXCL="lnkm64s.xcl"
elif [ $1 = "M644" ]
then
 CFG_BL_START="F800"
 CFG_FWD_START="D004"
 CFG_EE_SIZE="2048"
 CFG_LNKXCL="lnkm644s.xcl"
else
 echo "Invalid platform!"
 echo " "$USAGE
 exit 1  #error
fi

if [ $TC = "IAR" ]
then
 if [ $1 = "M32" ]
 then
  CFG_MCU="m32"
 elif [ $1 = "M64" ]
 then
  CFG_MCU="m64"
 elif [ $1 = "M644" ]
 then
  CFG_MCU="m644"
 fi
 MAKEFILE="Makefile_iar"
elif [ $TC = "GCC" ]
 then
 if [ $1 = "M32" ]
 then
  CFG_MCU="atmega32"
 elif [ $1 = "M64" ]
 then
  CFG_MCU="atmega64"
 elif [ $1 = "M644" ]
 then
  CFG_MCU="atmega644"
 fi
 MAKEFILE="Makefile_gcc"
else
 echo "Invalid toolchain!"
 echo " "$USAGE
 exit 1  #error
fi

# Generate configuration file
echo "MCU="$CFG_MCU > $CFGFILE
echo "BL_START="$CFG_BL_START >> $CFGFILE
echo "FWD_START="$CFG_FWD_START >> $CFGFILE
echo "EE_SIZE="$CFG_EE_SIZE >> $CFGFILE
echo "LNKXCL="$CFG_LNKXCL >> $CFGFILE

#check presence of soure Makefile
if [ ! -e "$MAKEFILE" ]
then
 echo "ERROR: Can not find file "$MAKEFILE
 exit 1  #error
fi
make clean
cp $MAKEFILE Makefile

#start build and check for errors
make -f $MAKEFILE
if [ $? -ne 0 ]
then
 exit 1  #error
fi

#OK. All operations were completed successfully
exit 0
