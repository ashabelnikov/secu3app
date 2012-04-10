#!/bin/sh

# Script for build firmware of SECU-3 project under Linux. This script will
# configure Makefile for you and build project.
# Created by Alexey A. Shabelnikov, Kiev 24 July 2011.
# Note: It requires AVR-GCC toolchain

USAGE="USAGE: configure.sh <platform>  \n Supported platforms: M16,M32,M64"
PLATFORM=Undefined
CFGFILE="platform_cfg"
MAKEFILE="Makefile_gcc"

CFG_MCU=Undefined
CFG_BL_START=Undefined
CFG_FWD_START=Undefined
CFG_EE_SIZE=Undefined
CFG_LNKXCL=Undefined

#check number of command line arguments
if [ $# -ne 1 ]
then
 echo "Invalid command line arguments."
 echo " "$USAGE
 exit 1  #error
fi

# Check validity of command line option
if [ $1 = "M16" ]
then
 CFG_MCU="atmega16"
 CFG_BL_START="3E00"
 CFG_FWD_START="3220"
 CFG_EE_SIZE="512"
elif [ $1 = "M32" ]
then
 CFG_MCU="atmega32"
 CFG_BL_START="7C00"
 CFG_FWD_START="7020"
 CFG_EE_SIZE="1024"
elif [ $1 = "M64" ]
then
 CFG_MCU="atmega64"
 CFG_BL_START="F800"
 CFG_FWD_START="EC20"
 CFG_EE_SIZE="2048"
else
 echo "Invalid platform!"
 echo " "$USAGE
 exit 1  #error
fi

# Generate configuration file
echo "MCU="$CFG_MCU > $CFGFILE
echo "BL_START="$CFG_BL_START >> $CFGFILE
echo "FWD_START="$CFG_FWD_START >> $CFGFILE
echo "EE_SIZE="$CFG_EE_SIZE >> $CFGFILE

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
