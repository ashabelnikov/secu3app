#!/bin/sh
# Script file for setting up of microcontroller's fuse bits for SECU-3 project
# Created by Alexey A. Shabelnikov, Kiev 04 December 2013.
# Updated 22 Jan 2018

PROGRAMMER=avrdude
LFUSE=0xD7
HFUSE=0xD4
EFUSE=0xFC
LOCK=0xE0

USAGE="Supported options: M644, M1284"
MCU=Undefined

if [ $# -eq 0 ]
then
 echo "Command line option required."
 echo $USAGE
 exit 1
elif [ $# -ne 1 ]
then
 echo "Wring number of command line options."
 exit 1
fi

# Check validity of command line option and set corresponding parameters
if [ $1 = "M644" ]
then
 MCU=ATMEGA644P
 LFUSE=0xD7
 HFUSE=0xD4
 EFUSE=0xFC
elif [ $1 = "M1284" ]
then
 MCU=ATMEGA1284P
 LFUSE=0xD7
 HFUSE=0xD4
 EFUSE=0xFC
else
 echo "Invalid platform!"
 echo $USAGE
 exit 1
fi

echo "EXECUTING BATCH..."
echo "---------------------------------------------"

#test if programmer exists
$PROGRAMMER >> /dev/null
if [ $? -ne 0 ]
then
 echo "ERROR: Can not execute file "$PROGRAMMER
 PrintError
 exit 1
fi

#ATMega644 and ATMega1284 have Extended Fuse Byte 
FUSEOPT="-U lfuse:w:$LFUSE:m -U hfuse:w:$HFUSE:m -U efuse:w:$EFUSE:m"

#Run programmer
$PROGRAMMER -p $MCU -c avr910 -P /dev/ttyACM0 -F $FUSEOPT -U lock:w:$LOCK:m
if [ $? -ne 0 ]
then
 PrintError
 exit 1
fi

echo "---------------------------------------------"
echo "ALL OPERATIONS WERE COMPLETED SUCCESSFULLY!"
exit 0

PrintError() {
echo "--------------------------------------------"
echo "WARNING! THERE ARE SOME ERRORS IN EXECUTING BATCH."
}
