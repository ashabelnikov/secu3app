/* SECU-3  - An open source, free engine control unit
   Copyright (C) 2007 Alexey A. Shabelnikov. Ukraine, Kiev

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.

   contacts:
              http://secu-3.org
              email: shabelnikov@secu-3.org
*/

/** \file adc.c
 * \author Alexey A. Shabelnikov
 * Implementation of ADC related functions (API).
 * Functions for read values from ADC, perform conversion to phisical values etc
 */

#include "port/avrio.h"
#include "port/interrupt.h"
#include "port/intrinsic.h"
#include "port/port.h"
#include <stdlib.h>
#include "adc.h"
#include "bitmask.h"
#include "magnitude.h"
#include "mathemat.h"
#include "ioconfig.h"

/**number of ADC channel used for MAP */
#define ADCI_MAP                2
/**number of ADC channel used for board voltage */
#define ADCI_UBAT               1
/**number of ADC channel used for CLT */
#define ADCI_TEMP               0
/*ADC channel number for ADD_IO1 */
#define ADCI_ADD_I1             6
/*ADC channel number for ADD_IO2 */
#define ADCI_ADD_I2             5

#if !defined(SECU3T)
/*ADC channel number for ADD_I3 */
#define ADCI_ADD_I3             4
#endif

/*ADC channel number for TPS */
#define ADCI_TPS                7
/**number of ADC channel used for knock */
#define ADCI_KNOCK              3

/**Tics of TCNT1 timer per 1 second */
#define TMR_TICKS_PER_SEC 312500L

/**ADC input connected to GND*/
#define ADCI_GND                0x1F

#if defined(FUEL_INJECT) || defined(GD_CONTROL)

/**Used for TPSdot and MAPdot calculations*/
typedef struct
{
 int16_t volt;              //!< Voltage
 uint16_t tmr;              //!< Timer value
}dotval_t;
#endif

/** Data structure of the ADC state variables */
typedef struct
{
 volatile uint16_t map_value;    //!< last measured value of the manifold absolute pressure
 volatile uint16_t ubat_value;   //!< last measured value of the board voltage
 volatile uint16_t temp_value;   //!< last measured value from the coolant temperature sensor (CLT)
 volatile uint16_t knock_value;  //!< last measured value from knock sensor(s)
 volatile uint16_t add_i1_value; //!< last measured value od ADD_I1
 volatile uint16_t add_i2_value; //!< last measured value of ADD_I2
#if !defined(SECU3T)
 volatile uint16_t add_i3_value; //!< last measured value of ADD_I3
#endif
 volatile uint16_t tps_value;    //!< last measured value of TPS
#if defined(FUEL_INJECT) || defined(GD_CONTROL)
 volatile dotval_t tpsdot[2];    //!< two value pairs used for TPSdot calculations
 volatile dotval_t mapdot[2];    //!< two value pairs used for MAPdot calculations
 volatile uint16_t tpsdot_mindt; //!< minimum time diffrencial used in calculation of d%/dt
 volatile uint16_t mapdot_mindt; //!< minimum time diffrencial used in calculation of dP/dt
#endif
#ifndef TPIC8101
 uint8_t waste_meas;             //!< if 1, then waste measurement will be performed for knock
 uint8_t knock_msm;              //!< State for INTOUT (HIP9011) sampling. Not necessary for TPIC8101
#endif
 uint8_t map_msm;                //!< State for MAP sampling
 uint8_t sens_msm;               //!< State for other sensors sampling
 uint8_t map_to_ckp;             //!< MAP sampling mode
 uint8_t mux_start;              //!< Start value for ADMUX depending on the MAP sampling mode
}adcstate_t;

/** ADC state variables */
adcstate_t adc;

uint16_t adc_get_map_value(void)
{
 uint16_t value;
 _BEGIN_ATOMIC_BLOCK();
 value = adc.map_value;
 _END_ATOMIC_BLOCK();
 return value;
}

uint16_t adc_get_ubat_value(void)
{
 uint16_t value;
 _BEGIN_ATOMIC_BLOCK();
 value = adc.ubat_value;
 _END_ATOMIC_BLOCK();
 return value;
}

uint16_t adc_get_temp_value(void)
{
 uint16_t value;
 _BEGIN_ATOMIC_BLOCK();
 value = adc.temp_value;
 _END_ATOMIC_BLOCK();
 return value;
}

uint16_t adc_get_add_i1_value(void)
{
 uint16_t value;
 _BEGIN_ATOMIC_BLOCK();
 value = adc.add_i1_value;
 _END_ATOMIC_BLOCK();
 return value;
}

uint16_t adc_get_add_i2_value(void)
{
 uint16_t value;
 _BEGIN_ATOMIC_BLOCK();
 value = adc.add_i2_value;
 _END_ATOMIC_BLOCK();
 return value;
}

#if !defined(SECU3T)
uint16_t adc_get_add_i3_value(void)
{
 uint16_t value;
 _BEGIN_ATOMIC_BLOCK();
 value = adc.add_i3_value;
 _END_ATOMIC_BLOCK();
 return value;
}
#endif

uint16_t adc_get_tps_value(void)
{
 uint16_t value;
 _BEGIN_ATOMIC_BLOCK();
 value = adc.tps_value;
 _END_ATOMIC_BLOCK();
 return value;
}
#if defined(FUEL_INJECT) || defined(GD_CONTROL)
int16_t adc_get_tpsdot_value(void)
{
 int16_t dv; uint16_t dt;
 dotval_t tpsval[2];
 _BEGIN_ATOMIC_BLOCK();
 tpsval[0] = adc.tpsdot[0];
 tpsval[1] = adc.tpsdot[1];
 _END_ATOMIC_BLOCK();

 dv = tpsval[0].volt - tpsval[1].volt;  //calculate voltage change in ADC discretes
 dt = (tpsval[0].tmr - tpsval[1].tmr);  //calculate time change in ticks of timer
 if (abs(dv) > 512) dv = (dv < 0) ? -512 : 512; //limit voltage change to the half of ADC range
 if (dt < TMR_TICKS_PER_SEC/1000) return 0;     //avoid overflow, limit time change to minimum 1ms

 return (((int32_t)dv) * TMR_TICKS_PER_SEC) / dt; //calculate 1-st derivative, num of ADC discr / sec
}

int16_t adc_get_mapdot_value(void)
{
 int16_t dv; uint16_t dt;
 dotval_t mapval[2];
 _BEGIN_ATOMIC_BLOCK();
 mapval[0] = adc.mapdot[0];
 mapval[1] = adc.mapdot[1];
 _END_ATOMIC_BLOCK();

 dv = mapval[0].volt - mapval[1].volt;  //calculate voltage change in ADC discretes
 dt = (mapval[0].tmr - mapval[1].tmr);  //calculate time change in ticks of timer
 if (abs(dv) > 512) dv = (dv < 0) ? -512 : 512; //limit voltage change to the half of ADC range
 if (dt < TMR_TICKS_PER_SEC/1000) return 0;     //avoid overflow, limit time change to minimum 1ms

 return (((int32_t)dv) * TMR_TICKS_PER_SEC) / dt; //calculate 1-st derivative, num of ADC discr / sec
}

#endif

uint16_t adc_get_knock_value(void)
{
 uint16_t value;
 _BEGIN_ATOMIC_BLOCK();
 value = adc.knock_value;
 _END_ATOMIC_BLOCK();
 return value;
}

void adc_begin_measure(void)
{
 if (adc.sens_msm != ADCI_GND)
  return; //We can't start new measurement while previous one is not finished yet

 if (adc.map_msm == ADCI_GND
#ifndef TPIC8101
 && adc.knock_msm == ADCI_GND
#endif
 )
 {
  adc.sens_msm = ADMUX = adc.mux_start;
  SETBIT(ADCSRA, ADSC);     //start ADC
 }
 else
 { //just set request and do nothing with ADC
  adc.sens_msm = adc.mux_start;
 }
}

#ifndef TPIC8101
//This function is used for HIP9011 only, it is not used for TPIC8101
void adc_begin_measure_knock(void)
{
 if (adc.knock_msm != ADCI_GND)
  return; //We can't start new measurement while previous one is not finished yet

 if (adc.map_msm == ADCI_GND && adc.sens_msm == ADCI_GND)
 {
  adc.knock_msm = ADMUX = ADC_VREF_TYPE | ADCI_KNOCK;
  adc.waste_meas = 1;     //<--one measurement delay will be used
  SETBIT(ADCSRA, ADSC);   //start ADC
 }
 else
 { //ADC is busy by other task, just set request and do nothing with ADC here
  adc.knock_msm = ADC_VREF_TYPE | ADCI_KNOCK;
  adc.waste_meas = 1;    //<--one measurement delay will be used
 }
}
#endif

void adc_begin_measure_map(void)
{
 if (0==adc.map_to_ckp || adc.map_msm != ADCI_GND)
  return; //We can't start new measurement while previous one is not finished yet

 if (adc.sens_msm == ADCI_GND
#ifndef TPIC8101
 && adc.knock_msm == ADCI_GND
#endif
 )
 {
  adc.map_msm = ADMUX = ADC_VREF_TYPE | ADCI_MAP;
  SETBIT(ADCSRA, ADSC);     //start ADC
 }
 else
 { //ADC is busy by other task, just set request and do nothing with ADC here
  adc.map_msm = ADC_VREF_TYPE | ADCI_MAP;
 }
}

uint8_t adc_is_measure_ready(uint8_t what)
{
 if (what == ADCRDY_MAP)
  return (adc.map_msm == ADCI_GND);
#ifndef TPIC8101
 if (what == ADCRDY_KNOCK)
  return (adc.knock_msm == ADCI_GND);
#endif
 if (what == ADCRDY_SENS)
  return (adc.sens_msm == ADCI_GND);
 return 0;
}

void adc_set_map_to_ckp(uint8_t mtckp)
{
 //skip MAP, start from board voltage OR start from MAP
 uint8_t mux_start = (mtckp) ? (ADC_VREF_TYPE | ADCI_UBAT) : (ADC_VREF_TYPE | ADCI_MAP);
 _BEGIN_ATOMIC_BLOCK();
 adc.mux_start = mux_start;
 adc.map_to_ckp = mtckp;
 _END_ATOMIC_BLOCK();
}

void adc_init(void)
{
 //ADC is ready for new measurement:
 adc.map_msm = ADCI_GND;  //no standalone MAP sampling at the moment
#ifndef TPIC8101
 adc.knock_msm = ADCI_GND;  //no INTOUT sampling at the moment
 adc.waste_meas = 0;
#endif
 adc.sens_msm = ADCI_GND;  //no other sensors sampling at the moment
 adc.map_to_ckp = 0;
 adc.mux_start = ADC_VREF_TYPE | ADCI_MAP;

 adc.knock_value = 0;

 //initialization of ADC, f = 156.25 kHz,
 //internal or external reference voltage source will be used depends on VREF_5V option, interrupt enabled
 ADMUX=ADC_VREF_TYPE;
 ADCSRA=_BV(ADEN)|_BV(ADIE)|_BV(ADPS2)|_BV(ADPS1)|_BV(ADPS0);

 //disable comparator - it is not needed
 ACSR=_BV(ACD);
}

#ifndef TPIC8101

/**For HIP9011 we need to measure INTOUT*/
#define RELAY_INPUT(muxid) \
  if (adc.knock_msm!=ADCI_GND) \
  { \
   ADMUX = adc.knock_msm; \
   adc.sens_msm = ADC_VREF_TYPE | (muxid); \
  } \
  else if (adc.map_msm!=ADCI_GND) \
  { \
   ADMUX = adc.map_msm; \
   adc.sens_msm = ADC_VREF_TYPE | (muxid); \
  } \
  else \
  { \
   ADMUX = ADC_VREF_TYPE | (muxid); \
  }
#else

/**For TPIC8101 we don't need to measure INTOUT*/
#define RELAY_INPUT(muxid) \
  if (adc.map_msm!=ADCI_GND) \
  { \
   ADMUX = adc.map_msm; \
   adc.sens_msm = ADC_VREF_TYPE | (muxid); \
  } \
  else \
  { \
   ADMUX = ADC_VREF_TYPE | (muxid); \
  }
#endif

/** Samples MAPdot */
#define SAMPLE_MAPDOT() \
    if ((TCNT1 - adc.mapdot[0].tmr) >= adc.mapdot_mindt) \
    { \
     /*save values for MAPdot calculations */ \
     adc.mapdot[1] = adc.mapdot[0];          /*previous = current */ \
     adc.mapdot[0].volt = adc.map_value;     /*save voltage */ \
     adc.mapdot[0].tmr = TCNT1;              /*save timer's value */\
    }

/**Samples TPSdot */
#define SAMPLE_TPSDOT() \
   if ((TCNT1 - adc.tpsdot[0].tmr) >= adc.tpsdot_mindt) \
   { \
    /*save values for TPSdot calculations */ \
    adc.tpsdot[1] = adc.tpsdot[0];       /*previous = current */ \
    adc.tpsdot[0].volt = adc.tps_value;  /*save voltage */ \
    adc.tpsdot[0].tmr = TCNT1;           /*save timer value */ \
   }

/**Interrupt for completion of ADC conversion. Measurement of values of all analog sensors.
 * After starting measurements this interrupt routine will be called for each input untill all inputs will be processed.
 */
ISR(ADC_vect)
{
 _ENABLE_INTERRUPT();

 switch(ADMUX & 0x07)
 {
  case ADCI_MAP: //Measurement of MAP completed
   adc.map_value = ADC;

   if (adc.map_to_ckp)
   {
    _DISABLE_INTERRUPT();  //disable interrupts to prevent nested ADC interrupts and other effects
    //MAP is standalone sensor, measured synchronously with CKP
#ifndef TPIC8101
    if (adc.knock_msm!=ADCI_GND)
    {
     ADMUX = adc.knock_msm;
    }
    else
#endif
    if (adc.sens_msm!=ADCI_GND)
    {  
     ADMUX = adc.sens_msm;
    }
    else
    {
     adc.map_msm = ADCI_GND; //MAP sampling has been finished
    }

#if defined(FUEL_INJECT) || defined(GD_CONTROL)
    _ENABLE_INTERRUPT();
    SAMPLE_MAPDOT();
#endif

    if (adc.map_msm == ADCI_GND)
     return;                                 //just exit, don't start ADC
    adc.map_msm = ADCI_GND;
   }
   else
   {
    //MAP is a part of block of other sensors
     _DISABLE_INTERRUPT();
#ifndef TPIC8101
    if (adc.knock_msm!=ADCI_GND)
    {
     ADMUX = adc.knock_msm;
     adc.sens_msm = ADC_VREF_TYPE | ADCI_UBAT;
    }
    else
#endif
    {
     //continue processing other sensors
     ADMUX = ADC_VREF_TYPE | ADCI_UBAT;
    }
    
#if defined(FUEL_INJECT) || defined(GD_CONTROL)
    _ENABLE_INTERRUPT();
    SAMPLE_MAPDOT();
#endif
   }
   break;
   
  case ADCI_UBAT: //Measurement of board voltage completed
   adc.ubat_value = ADC;
   _DISABLE_INTERRUPT();
   RELAY_INPUT(ADCI_TEMP);
   break;

  case ADCI_TEMP: //Measurement of CLT completed
   adc.temp_value = ADC;
   _DISABLE_INTERRUPT();
   RELAY_INPUT(ADCI_TPS);
   break;

  case ADCI_TPS: //Measurement of TPS completed
   adc.tps_value = ADC;
   _DISABLE_INTERRUPT();
   RELAY_INPUT(ADCI_ADD_I1);
#if defined(FUEL_INJECT) || defined(GD_CONTROL)
   _ENABLE_INTERRUPT();
   SAMPLE_TPSDOT();
#endif
   break;

  case ADCI_ADD_I1: //Measurement of ADD_I1 completed
   adc.add_i1_value = ADC;
   _DISABLE_INTERRUPT();
   RELAY_INPUT(ADCI_ADD_I2);
   break;

  case ADCI_ADD_I2: //Measurement of ADD_I2 completed
   adc.add_i2_value = ADC;
   _DISABLE_INTERRUPT();
#if !defined(SECU3T)
   RELAY_INPUT(ADCI_ADD_I3);
   break;

  case ADCI_ADD_I3: //Measurement of ADD_I3 completed
   adc.add_i3_value = ADC;
   _DISABLE_INTERRUPT();        //prevent nested ADC interrupts and other effects
#endif

#ifndef TPIC8101
   if (adc.knock_msm!=ADCI_GND)
   {
    ADMUX = adc.knock_msm;
   }
   else if (adc.map_msm!=ADCI_GND)
   {
    ADMUX = adc.map_msm;
   }
   else
   {
    adc.sens_msm = ADCI_GND; //finished
    return;
   }
   adc.sens_msm = ADCI_GND; //finished
   break;
#else
  _DISABLE_INTERRUPT();
  if (adc.map_msm!=ADCI_GND)
  {
   ADMUX = adc.map_msm;
   adc.sens_msm = ADC_VREF_TYPE | ADCI_KNOCK;
  }
  else
  {
   ADMUX = ADC_VREF_TYPE | ADCI_KNOCK;
  }
  break;
#endif

  //note: we can fall here either from adc_beign_measure_knock() or previous state
  //if defined TPIC8101, when ADCI_KNOCK used for ADD_I4
  case ADCI_KNOCK:         //measurement of the int. output voltage finished
#ifndef TPIC8101
   if (adc.waste_meas)
   {                       //waste measurement is required (for delay)
    adc.waste_meas = 0;
    _DISABLE_INTERRUPT();
    break;                 //change nothing and start ADC again
   }

   //INTOUT measured by ADC
   adc.knock_value = ADC;  //save INTOUT value

   _DISABLE_INTERRUPT();  //disable interrupts to prevent nested ADC interrupts and other effects

   if (adc.map_msm!=ADCI_GND)
   {
    ADMUX = adc.map_msm;
   }
   else if (adc.sens_msm!=ADCI_GND)
   {  
    ADMUX = adc.sens_msm;
   }
   else
   {
    adc.knock_msm = ADCI_GND; //finished (no other tasks)
    return;
   }
   adc.knock_msm = ADCI_GND; //knock finished
#else
   //INTOUT read via SPI, so here we measure ADD_I4
   adc.knock_value = ADC;
   _DISABLE_INTERRUPT();
   if (adc.map_msm!=ADCI_GND)
   {
    ADMUX = adc.map_msm;
    adc.sens_msm = ADCI_GND; //sensors finished
   }
   else
   {
    adc.sens_msm = ADCI_GND; //sensors finished (no other tasks)
    return;
   }
#endif
   break;
 }
 //start ADC
 SETBIT(ADCSRA, ADSC);
}

int16_t adc_compensate(int16_t adcvalue, uint16_t factor, int32_t correction)
{
 return (((((int32_t)adcvalue*factor)+correction)<<2)>>16);
}

uint16_t map_adc_to_kpa(int16_t adcvalue, int16_t offset, int16_t gradient)
{
 int16_t t;
 //Our ADC doesn't measure negative values, but negative value may appear after compensation
 if (adcvalue < 0)
  adcvalue = 0;

 //Expression looks like this: ((adcvalue + offset) * gradient ) / 128, where offset,gradient - constants.
 t = adcvalue + offset;
 if (gradient > 0)
 {
  if (t < 0)
   t = 0;    //restrict value
 }
 else
 {
  if (t > 0)
   t = 0;    //restrict value
 }
 return ( ((int32_t)t) * gradient ) >> 7;
}

uint16_t ubat_adc_to_v(int16_t adcvalue)
{
 //Our ADC doesn't measure negative values, but negative value may appear after compensation
 if (adcvalue < 0)
  adcvalue = 0;
 return adcvalue;
}

//Coolant sensor has linear output. 10mV per C (e.g. LM235)
int16_t temp_adc_to_c(int16_t adcvalue)
{
 //Our ADC doesn't measure negative values, but negative value may appear after compensation
 if (adcvalue < 0)
  adcvalue = 0;
 return (adcvalue - ((int16_t)((TSENS_ZERO_POINT / ADC_DISCRETE)+0.5)) );
}

uint16_t tps_adc_to_pc(int16_t adcvalue, int16_t offset, int16_t gradient)
{
 int16_t t;
 //Our ADC doesn't measure negative values, but negative value may appear after compensation
 if (adcvalue < 0)
  adcvalue = 0;
 t = adcvalue + offset;
 if (gradient > 0)
 {
  if (t < 0)
   t = 0;
 }
 else
 {
  if (t > 0)
   t = 0;
 }

 t = (((int32_t)t) * gradient) >> (7+1);

 restrict_value_to(&t, TPS_MAGNITUDE(0), TPS_MAGNITUDE(110)); //restrict to 110%

 return t;
}

#if defined(FUEL_INJECT) || defined(GD_CONTROL)
int16_t tpsdot_adc_to_pc(int16_t adcvalue, int16_t gradient)
{
 return (((int32_t)adcvalue) * gradient) >> (7+6+1);
}

int16_t mapdot_adc_to_kpa(int16_t adcvalue, int16_t gradient)
{
 return (((int32_t)adcvalue) * gradient) >> (7+6);
}
#endif

void adc_measure_voltage(void)
{
 ADMUX = ADC_VREF_TYPE | ADCI_UBAT; //select voltage input
 SETBIT(ADCSRA, ADSC);              //start measurement
 while(CHECKBIT(ADCSRA, ADSC));     //wait for completion of measurement
 adc.ubat_value = ADC;
}

#if !defined(SECU3T) && defined(MCP3204)
uint16_t adc_get_add_i5_value(void)
{
 uint16_t value;
 _BEGIN_ATOMIC_BLOCKN();
 value = spiadc_chan[0] & 0xFFF;
 _END_ATOMIC_BLOCKN();
 return value;
}

uint16_t adc_get_add_i6_value(void)
{
 uint16_t value;
 _BEGIN_ATOMIC_BLOCKN();
 value = spiadc_chan[1] & 0xFFF;
 _END_ATOMIC_BLOCKN();
 return value;
}

uint16_t adc_get_add_i7_value(void)
{
 uint16_t value;
 _BEGIN_ATOMIC_BLOCKN();
 value = spiadc_chan[2] & 0xFFF;
 _END_ATOMIC_BLOCKN();
 return value;
}

uint16_t adc_get_add_i8_value(void)
{
 uint16_t value;
 _BEGIN_ATOMIC_BLOCKN();
 value = spiadc_chan[3] & 0xFFF;
 _END_ATOMIC_BLOCKN();
 return value;
}

#endif

#if defined(FUEL_INJECT) || defined(GD_CONTROL)
void adc_set_tpsdot_mindt(uint16_t mindt)
{
 adc.tpsdot_mindt = mindt;
}

void adc_set_mapdot_mindt(uint16_t mindt)
{
 adc.mapdot_mindt = mindt;
}
#endif

