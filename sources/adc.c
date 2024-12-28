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

/*ADC channel number for CARB */
#define ADCI_CARB               7
/**number of ADC channel used for knock */
#define ADCI_KNOCK              3

/**Tics of TCNT1 timer per 1 second */
#define TMR_TICKS_PER_SEC 312500L

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
 volatile uint16_t carb_value;   //!< last measured value of TPS
#if defined(FUEL_INJECT) || defined(GD_CONTROL)
 volatile dotval_t tpsdot[2];    //!< two value pairs used for TPSdot calculations
 volatile dotval_t mapdot[2];    //!< two value pairs used for MAPdot calculations
 volatile uint16_t tpsdot_mindt; //!< minimum time diffrencial used in calculation of d%/dt
 volatile uint16_t mapdot_mindt; //!< minimum time diffrencial used in calculation of dP/dt
#endif
 volatile uint8_t sensors_ready; //!< all sensors have been processed and their values are ready for reading
#ifndef TPIC8101
 uint8_t  waste_meas;            //!< if 1, then waste measurement will be performed for knock
#endif
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

uint16_t adc_get_carb_value(void)
{
 uint16_t value;
 _BEGIN_ATOMIC_BLOCK();
 value = adc.carb_value;
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

void adc_begin_measure(uint8_t speed2x)
{
 if (!adc.sensors_ready)
  return; //We can't start new measurement while previous one is not finished yet

 adc.sensors_ready = 0;
 ADMUX = ADCI_MAP|ADC_VREF_TYPE;
 if (speed2x)
  CLEARBIT(ADCSRA, ADPS0); //250kHz
 else
  SETBIT(ADCSRA, ADPS0);   //125kHz
 SETBIT(ADCSRA, ADSC);
}

#ifndef TPIC8101
//This function is used for HIP9011 only, it is not used for TPIC8101
void adc_begin_measure_knock(uint8_t speed2x)
{
 if (!adc.sensors_ready)
  return; //We can't start new measurement while previous one is not finished yet

 adc.sensors_ready = 0;
 adc.waste_meas = 1;   //<--one measurement delay will be used
 ADMUX = ADCI_KNOCK|ADC_VREF_TYPE;
 if (speed2x)
  CLEARBIT(ADCSRA, ADPS0); //250kHz
 else
  SETBIT(ADCSRA, ADPS0);   //125kHz
 SETBIT(ADCSRA, ADSC);
}
#endif

uint8_t adc_is_measure_ready(void)
{
 return adc.sensors_ready;
}

void adc_init(void)
{
 adc.knock_value = 0;
#ifndef TPIC8101
 adc.waste_meas = 0;
#endif

 //initialization of ADC, f = 125.000 kHz,
 //internal or external reference voltage source will be used depends on VREF_5V option, interrupt enabled
 ADMUX=ADC_VREF_TYPE;
 ADCSRA=_BV(ADEN)|_BV(ADIE)|_BV(ADPS2)|_BV(ADPS1)|_BV(ADPS0);

 //ADC is ready for new measurement
 adc.sensors_ready = 1;

 //disable comparator - it is not needed
 ACSR=_BV(ACD);
}

#if defined(FUEL_INJECT) && defined(XTAU_CORR)
volatile uint8_t xtau_str_cnt = 0;
volatile uint32_t xtau_str_int = 0;
static volatile uint16_t xtau_tcnt = 0;
#endif

/**Interrupt for completion of ADC conversion. Measurement of values of all analog sensors.
 * After starting measurements this interrupt routine will be called for each input untill all inputs will be processed.
 */
ISR(ADC_vect)
{
 _ENABLE_INTERRUPT();

 switch(ADMUX&0x07)
 {
  case ADCI_MAP: //Measurement of MAP completed
   adc.map_value = ADC;
   ADMUX = ADCI_UBAT|ADC_VREF_TYPE;
   _DISABLE_INTERRUPT();  //disable interrupts to prevent nested ADC interrupts
   SETBIT(ADCSRA,ADSC);

#if defined(FUEL_INJECT) || defined(GD_CONTROL)
   if ((TCNT1 - adc.mapdot[0].tmr) >= adc.mapdot_mindt)
   {
    //save values for MAPdot calculations
    adc.mapdot[1] = adc.mapdot[0];          //previous = current
    adc.mapdot[0].volt = adc.map_value;     //save voltage
    adc.mapdot[0].tmr = TCNT1;              //save timer's value
   }
#endif

#if defined(FUEL_INJECT) && defined(XTAU_CORR)
   //TODO: calling this code here does not guarantee that it will be called for every engine stroke. So, rewrite it in the future
   xtau_str_int+=(TCNT1 - xtau_tcnt);       //measure time between events and add it to the sum
   xtau_tcnt = TCNT1;
   ++xtau_str_cnt;                          //update counter of events
#endif

   break;

  case ADCI_UBAT: //Measurement of board voltage completed
   adc.ubat_value = ADC;
   ADMUX = ADCI_TEMP|ADC_VREF_TYPE;
   _DISABLE_INTERRUPT();
   SETBIT(ADCSRA,ADSC);
   break;

  case ADCI_TEMP: //Measurement of CLT completed
   adc.temp_value = ADC;
   ADMUX = ADCI_CARB|ADC_VREF_TYPE;
   _DISABLE_INTERRUPT();
   SETBIT(ADCSRA,ADSC);
   break;

  case ADCI_CARB: //Measurement of TPS completed
   adc.carb_value = ADC;
   ADMUX = ADCI_ADD_I1|ADC_VREF_TYPE;
   _DISABLE_INTERRUPT();  //disable interrupts to prevent nested ADC interrupts
   SETBIT(ADCSRA,ADSC);

#if defined(FUEL_INJECT) || defined(GD_CONTROL)
   if ((TCNT1 - adc.tpsdot[0].tmr) >= adc.tpsdot_mindt)
   {
    //save values for TPSdot calculations
    adc.tpsdot[1] = adc.tpsdot[0];          //previous = current
    adc.tpsdot[0].volt = adc.carb_value;//save voltage
    adc.tpsdot[0].tmr = TCNT1;          //save timer value
   }
#endif
   break;

  case ADCI_ADD_I1: //Measurement of ADD_I1 completed
   adc.add_i1_value = ADC;
   ADMUX = ADCI_ADD_I2|ADC_VREF_TYPE;
   _DISABLE_INTERRUPT();
   SETBIT(ADCSRA,ADSC);
   break;

  case ADCI_ADD_I2: //Measurement of ADD_I2 completed
   adc.add_i2_value = ADC;

#if !defined(SECU3T)
   ADMUX = ADCI_ADD_I3|ADC_VREF_TYPE;
   _DISABLE_INTERRUPT();
   SETBIT(ADCSRA,ADSC);
   break;

  case ADCI_ADD_I3: //Measurement of ADD_I3 completed
   adc.add_i3_value = ADC;
#endif

#ifndef TPIC8101
   ADMUX = ADCI_MAP|ADC_VREF_TYPE;
   adc.sensors_ready = 1; //finished
#else
   //continue (as additional analog input)
   ADMUX = ADCI_KNOCK|ADC_VREF_TYPE;
   _DISABLE_INTERRUPT();
   SETBIT(ADCSRA, ADSC);
#endif
   break;

  //note: we can fall here either from adc_beign_measure_knock() or previous state
  //if defined TPIC8101, then ADCI_KNOCK used for ADD_I4
  case ADCI_KNOCK:  //measurement of the int. output voltage finished
#ifndef TPIC8101
   if (adc.waste_meas)
   {                       //waste measurement is required (for delay)
    adc.waste_meas = 0;
   _DISABLE_INTERRUPT();
    SETBIT(ADCSRA, ADSC);  //change nothing and start ADC again
    break;
   }
#endif
   adc.knock_value = ADC;
   adc.sensors_ready = 1;
   break;
 }
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

 restrict_value_to(&t, TPS_MAGNITUDE(0), TPS_MAGNITUDE(100)); //restrict to 100%

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
 ADMUX = ADCI_UBAT|ADC_VREF_TYPE; //select volage input
 SETBIT(ADCSRA, ADSC);            //start measurement
 while(CHECKBIT(ADCSRA, ADSC));   //wait for completion of measurement
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

