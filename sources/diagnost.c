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

void init_digital_outputs(void)
{

}

void init_digital_inputs(void)
{

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
   d->diag_inp.add_io1 = 0; //not supported in SECU-3
   d->diag_inp.add_io2 = 0; //not supported in SECU-3
   d->diag_inp.carb = 0;    //not supported in SECU-3
   d->diag_inp.ks_2 = 0;    //not supported in SECU-3
#endif

   //digital inputs
//   d->diag_inp.bits = 0; //todo!!!

   //outputs
//   d->diag_out; //todo!!!
  }
  else
   --diag.skip_loops;

  wdt_reset_timer();
 }
}

#endif //DIAGNOSTICS
