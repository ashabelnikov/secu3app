#!/bin/sh
# Batch file for programming of firmware code into microcontroller for SECU-3 project
# Created by Alexey A. Shabelnikov, Kiev 07 October 2009.
# Updated 22 Jan 2018

PROGRAMMER=avrdude
CODE=secu-3_app.a90
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
elif [ $1 = "M1284" ]
then
 MCU=ATMEGA1284P
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

#Run programmer
$PROGRAMMER -p $MCU -c avr910 -P /dev/ttyACM0 -U flash:w:$CODE
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
