/* SECU-3  - An open source, free engine control unit
   Copyright (C) 2007 Alexey A. Shabelnikov. Ukraine, Gorlovka

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

/** \file diagnost.c
 * Implementation of hardware diagnostics
 * (Реализация диагностики аппаратной части)
 */

#ifdef DIAGNOSTICS

#include "port/avrio.h"
#include "port/intrinsic.h"
#include "adc.h"
#include "bitmask.h"
#include "diagnost.h"
#include "knock.h"
#include "procuart.h"
#include "secu3.h"
#include "suspendop.h"
#include "uart.h"
#include "ufcodes.h"
#include "wdt.h"

/**Helpful macro used for generation of bit mask for outputs*/
#define _OBV(b) (((uint16_t)1) << b)

/**Helpful macro used for generation of bit mask for inputs*/
#define _IBV(v,b) (((uint8_t)v) << b)

/**Describes state variables data*/
typedef struct
{
 uint8_t diag_started;   //!< flags indicates that diagnostic mode is started
 uint8_t fsm_state;      //!< finite state machine state
 uint8_t ksp_channel;    //!< number of current knock channel
 uint16_t knock_value[2];//!< latest measurements of two knock sensors
 uint8_t skip_loops;     //!< counter used for skipping of several measurement loop after entering diag. mode
}diag_state_t;

/**Instance of diagnostics state variables */
diag_state_t diag;

void diagnost_start(void)
{
 diag.diag_started = 1;
 diag.fsm_state = 0;
 diag.ksp_channel = KSP_CHANNEL_0;
 diag.skip_loops = 5;
 uart_set_send_mode(DIAGINP_DAT);
}

void diagnost_stop(void)
{
 sop_set_operation(SOP_SEND_NC_LEAVE_DIAG);
}

/**Initialization of outputs in diagnostic mode. All outputs are OFF*/
void init_digital_outputs(void)
{
 //IGN_OU1, IGN_OUT2, IGN_OUT3, IGN_OUT4
 PORTD&= ~(_BV(PD5)|_BV(PD4));
 PORTC&= ~(_BV(PC1)|_BV(PC0));
 DDRD|= (_BV(DDD5)|_BV(DDD4));
 DDRC|= (_BV(DDC1)|_BV(DDC0));
#ifdef SECU3T
 //ADD_IO1, ADD_IO2
 PORTC&= ~(_BV(PC5));
 PORTA&= ~(_BV(PA4));
 DDRC|= (_BV(DDC5));
 DDRA|= (_BV(DDA4));
#endif
 //IE
 PORTB&= ~(_BV(PB0));
 DDRB|= _BV(DDB0);
 //FE
 PORTC&= ~_BV(PC7);
 DDRC|= _BV(DDC7);
 //ECF
#ifdef SECU3T /*SECU-3T*/
 PORTD|= _BV(PD7);
 DDRD |= _BV(DDD7);
#else         /*SECU-3*/
 PORTB&= ~_BV(PB1);
 DDRB |= _BV(DDB1);
#endif
 //CE
#ifdef SECU3T /*SECU-3T*/
 PORTB|= _BV(PB2);
 DDRB |= _BV(DDB2);
#else         /*SECU-3*/
 PORTB&= ~(_BV(PB2));
 DDRB |= _BV(DDB2);
#endif
 //ST_BLOCK
#ifdef SECU3T /*SECU-3T*/
 PORTB|= _BV(PB1);
 DDRB |= _BV(DDB1);
#else         /*SECU-3*/
 PORTD|= _BV(PD7);
 DDRD |= _BV(DDD7);
#endif
}

/**Set states of outputs
 * \param o Bits containing output states
 */
void set_outputs(uint16_t o)
{
 PORTD_Bit4 = o & _OBV(0);   //IGN_OUT1
 PORTD_Bit5 = o & _OBV(1);   //IGN_OUT2
 PORTC_Bit0 = o & _OBV(2);   //IGN_OUT3
 PORTC_Bit1 = o & _OBV(3);   //IGN_OUT4
#ifdef SECU3T
 PORTC_Bit5 = o & _OBV(4);   //ADD_IO1
 PORTA_Bit4 = o & _OBV(5);   //ADD_IO2
#endif
 PORTB_Bit0 = o & _OBV(6);   //IE
 PORTC_Bit7 = o & _OBV(7);   //FE

#ifdef SECU3T /*SECU-3T*/
 PORTD_Bit7 = !(o & _OBV(8));//ECF
#else         /*SECU-3*/
 PORTB_Bit1 = o & _OBV(8);
#endif

#ifdef SECU3T /*SECU-3T*/
 PORTB_Bit2 = !(o & _OBV(9));//CE
#else         /*SECU-3*/
 PORTB_Bit2 = o & _OBV(9);
#endif

 //ST_BLOCK
#ifdef SECU3T /*SECU-3T*/
 PORTB_Bit1 = !(o & _OBV(10));//ST_BLOCK
#else         /*SECU-3*/
 PORTD_Bit7 = !(o & _OBV(10));
#endif
}

/**Initialization of digital inputs in diagnostic mode*/
void init_digital_inputs(void)
{
 //GAS_V, CKPS, REF_S, PS
 DDRC&=~(_BV(DDC6));
 DDRD&=~(_BV(DDD6)|_BV(DDD2));
#ifdef SECU3T
 DDRD&=~(_BV(DDB3));
#else /*SECU-3*/
 DDRC&=~(_BV(DDC4));
#endif
}

/**Get states of inputs
 * \return bits containing state of digital inputs
 */
uint8_t get_inputs(void)
{
 //GAS_V, CKPS, REF_S, PS
 uint8_t i = _IBV(PINC_Bit6, 0) | _IBV(PIND_Bit6, 1) | _IBV(PIND_Bit2, 2) |
#ifdef SECU3T
             _IBV(PIND_Bit3, 3);
#else  /*SECU-3*/
             _IBV(PINC_Bit4, 3);
#endif
 return i; 
}

void diagnost_process(struct ecudata_t* d)
{
 if (0==diag.diag_started)
  return; //normal mode

 //We are in diagnostic mode
 sop_set_operation(SOP_SEND_NC_ENTER_DIAG);

 //perform initialization of digital outputs
 init_digital_outputs();

 //perform initialization of digital inputs
 init_digital_inputs();

 //Diasable unneeded interrupts
 TIMSK&=~(_BV(OCIE2)|_BV(TICIE1)|_BV(OCIE1A)|_BV(OCIE1B)|_BV(TOIE1)|_BV(OCIE0)|_BV(TOIE0));

 //local loop
 while(1)
 {
  //check & execute suspended operations
  sop_execute_operations(d);
  //process data being received and sent via serial port
  process_uart_interface(d);

  switch(diag.fsm_state)
  {
   //start measurements (sensors), start KSP settings latching
   case 0:
    //select next channel
    knock_set_channel(diag.ksp_channel++);
    if (diag.ksp_channel > KSP_CHANNEL_1)
     diag.ksp_channel = KSP_CHANNEL_0;
    //start the process of downloading the settings into the HIP9011 (запускаем процесс загрузки настроек в HIP)
    knock_start_settings_latching();
    //start the process of measuring analog input values (запуск процесса измерения значений аналоговых входов)
    adc_begin_measure(0); //<--normal speed
    diag.fsm_state = 1;
    break;

   //wait for completion of measurements, start integration of current knock channel's signal
   case 1:
    if (adc_is_measure_ready())
    {
     knock_set_integration_mode(KNOCK_INTMODE_INT);
     diag.fsm_state = 2;
    }
    break;

   //start measurements (knock signal)
   case 2:
    knock_set_integration_mode(KNOCK_INTMODE_HOLD);
    adc_begin_measure_knock(0);
    diag.fsm_state = 3;
    break;

   //wait for completion of measurements, and reinitialize state machine
   case 3:
    if (adc_is_measure_ready())
    {
     diag.knock_value[diag.ksp_channel] = adc_get_knock_value();
     _DELAY_CYCLES(1600);
     diag.fsm_state = 0;
    }
    break;
  };

  if (diag.skip_loops == 0)
  {
   //analog inputs
   d->diag_inp.voltage = adc_get_ubat_value();
   d->diag_inp.map = adc_get_map_value();
   d->diag_inp.temp = adc_get_temp_value();
   d->diag_inp.ks_1 = diag.knock_value[0];
#ifdef SECU3T
   d->diag_inp.add_io1 = adc_get_add_io1_value();
   d->diag_inp.add_io2 = adc_get_add_io2_value();
   d->diag_inp.carb = adc_get_carb_value();
   d->diag_inp.ks_2 = diag.knock_value[1];
#else /*SECU-3*/
   d->diag_inp.add_io1 = 0;      //not supported in SECU-3
   d->diag_inp.add_io2 = 0;      //not supported in SECU-3
   d->diag_inp.carb = PINC_Bit5; //in SECU-3 it is digital input
   d->diag_inp.ks_2 = 0;         //not supported in SECU-3
#endif

   //digital inputs
   d->diag_inp.bits = get_inputs();

   //outputs
   set_outputs(d->diag_out);
  }
  else
   --diag.skip_loops;

  wdt_reset_timer();
 }
}

#endif //DIAGNOSTICS
