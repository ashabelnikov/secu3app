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

/** \file diagnost.c
 * \author Alexey A. Shabelnikov
 * Implementation of hardware diagnostics
 */

#ifdef DIAGNOSTICS

#include "port/avrio.h"
#include "port/intrinsic.h"
#include "bitmask.h"
#include "diagnost.h"
#include "ecudata.h"
#include "knock.h"
#include "magnitude.h"
#include "procuart.h"
#include "suspendop.h"
#include "uart.h"
#include "ufcodes.h"
#include "wdt.h"

/**Helpful macro used for generation of bit mask for outputs*/
#define _OBV(b) (((uint16_t)1) << b)

/**Helpful macro used for generation of bit mask for inputs*/
#define _IBV(v,b) (((uint8_t)v) << b)

/**Helpful macro for ADC compensation, f,c - must be floationg point constants*/
#define _ADC_COMPENSATE(v, f, c) adc_compensate(v, ADC_COMP_FACTOR(f), ADC_COMP_CORR(f, c))

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
 if (diag.diag_started)
 {
  sop_set_operation(SOP_SEND_NC_LEAVE_DIAG);
  diag.diag_started = 0;
 }
}

/**Initialization of outputs in diagnostic mode. All outputs are OFF*/
void init_digital_outputs(void)
{
 //IGN_OU1, IGN_OUT2, IGN_OUT3, IGN_OUT4
 PORTD&= ~(_BV(PD5)|_BV(PD4));
 PORTC&= ~(_BV(PC1)|_BV(PC0));
 DDRD|= (_BV(DDD5)|_BV(DDD4));
 DDRC|= (_BV(DDC1)|_BV(DDC0));
 //ADD_IO1, ADD_IO2
 PORTC&= ~(_BV(PC5));
 PORTA&= ~(_BV(PA4));
 DDRC|= (_BV(DDC5));
 DDRA|= (_BV(DDA4));
 //IE
 PORTB&= ~(_BV(PB0));
 DDRB|= _BV(DDB0);
 //FE
 PORTC&= ~_BV(PC7);
 DDRC|= _BV(DDC7);
 //ECF
#ifdef REV9_BOARD
 PORTD&= ~(_BV(PD7));
#else
 PORTD|= _BV(PD7);
#endif
 DDRD |= _BV(DDD7);
 //CE
#ifdef REV9_BOARD
 PORTB&= ~(_BV(PB2));
#else
 PORTB|= _BV(PB2);
#endif
 DDRB |= _BV(DDB2);
 //ST_BLOCK
#ifdef REV9_BOARD
 PORTB&= ~(_BV(PB1));
#else
 PORTB|= _BV(PB1);
#endif
 DDRB |= _BV(DDB1);
}

/**Set states of outputs
 * \param o Bits containing output states
 */
void set_outputs(uint16_t o)
{
 WRITEBIT(PORTD, PD4, (o & _OBV(0)));  //IGN_OUT1
 WRITEBIT(PORTD, PD5, (o & _OBV(1)));  //IGN_OUT2
 WRITEBIT(PORTC, PC0, (o & _OBV(2)));  //IGN_OUT3
 WRITEBIT(PORTC, PC1, (o & _OBV(3)));  //IGN_OUT4
 WRITEBIT(PORTC, PC5, (o & _OBV(4)));  //ADD_IO1
 WRITEBIT(PORTA, PA4, (o & _OBV(5)));  //ADD_IO2
 WRITEBIT(PORTB, PB0, (o & _OBV(6)));  //IE
 WRITEBIT(PORTC, PC7, (o & _OBV(7)));  //FE

#ifdef REV9_BOARD
 WRITEBIT(PORTD, PD7, (o & _OBV(8)));  //ECF
#else
 WRITEBIT(PORTD, PD7,!(o & _OBV(8)));
#endif

#ifdef REV9_BOARD
 WRITEBIT(PORTB, PB2, (o & _OBV(9)));  //CE
#else
 WRITEBIT(PORTB, PB2,!(o & _OBV(9)));
#endif

 //ST_BLOCK
#ifdef REV9_BOARD
 WRITEBIT(PORTB, PB1, (o & _OBV(10))); //ST_BLOCK
#else
 WRITEBIT(PORTB, PB1,!(o & _OBV(10)));
#endif

 //BL
 if (o & _OBV(12)) {
  WRITEBIT(PORTC, PC3, (o & _OBV(11))); }
 else {
  WRITEBIT(PORTC, PC3, 1); }             //pull up input
 WRITEBIT(DDRC, DDC3, (o & _OBV(12)));   //select mode: input/output

 //DE
 if (o & _OBV(14)) {
  WRITEBIT(PORTC, PC2, (o & _OBV(13))); }
 else {
  WRITEBIT(PORTC, PC2, 1); }             //pull up input
 WRITEBIT(DDRC, DDC2, (o & _OBV(14)));   //select mode: input/output
}

/**Initialization of digital inputs in diagnostic mode*/
void init_digital_inputs(void)
{
 //GAS_V, CKPS, REF_S, PS
 DDRC&=~(_BV(DDC6));
 DDRD&=~(_BV(DDD6)|_BV(DDD2));
 DDRD&=~(_BV(DDB3));
 //BL jmp, DE jmp
 DDRC &= ~(_BV(DDC3)|_BV(DDC2)); //inputs (входы)
 PORTC|= _BV(PC3)|_BV(PC2);
}

/**Get states of inputs
 * \return bits containing state of digital inputs
 */
uint8_t get_inputs(void)
{
 //GAS_V, CKPS, REF_S, PS
 uint8_t i = _IBV((!!CHECKBIT(PINC, PINC6)), 0) | _IBV((!!CHECKBIT(PIND, PIND6)), 1) | _IBV((!!CHECKBIT(PIND, PIND2)), 2) |
             _IBV((!!CHECKBIT(PIND, PIND3)), 3) |
             _IBV((!!CHECKBIT(PINC, PINC3)), 4) | _IBV((!!CHECKBIT(PINC, PINC2)), 5);  //BL jmp, DE jmp
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
 TIMSK0&=~(_BV(OCIE0A)|_BV(OCIE0B)|_BV(TOIE0));
 TIMSK1&=~(_BV(ICIE1)|_BV(OCIE1A)|_BV(OCIE1B)|_BV(TOIE1));
 TIMSK2&=~(_BV(OCIE2A)|_BV(OCIE2B));

 //Disable external interrupts
 EIMSK&=  ~(_BV(INT0) | _BV(INT1) | _BV(INT2));

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
    if (diag.ksp_channel > KSP_CHANNEL_1)
     diag.ksp_channel = KSP_CHANNEL_0;
    knock_set_channel(diag.ksp_channel);
    //start the process of downloading the settings into the HIP9011
    knock_start_settings_latching();
    //start the process of measuring analog input values
    adc_begin_measure(0); //<--normal speed
    diag.fsm_state = 1;
    break;

   //wait for completion of measurements, start integration of current knock channel's signal
   case 1:
    if (adc_is_measure_ready())
    {
     knock_set_integration_mode(KNOCK_INTMODE_INT);
     _DELAY_US(1000);   //1ms
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
     ++diag.ksp_channel;  //select next channel
     _DELAY_US(100);
     diag.fsm_state = 0;
    }
    break;
  };

  if (diag.skip_loops == 0)
  {
   //analog inputs
   d->diag_inp.voltage = _ADC_COMPENSATE(adc_get_ubat_value(), ADC_VREF_FACTOR, 0.0);
   d->diag_inp.map = _ADC_COMPENSATE(adc_get_map_value(), ADC_VREF_FACTOR, 0.0);
   d->diag_inp.temp = _ADC_COMPENSATE(adc_get_temp_value(), ADC_VREF_FACTOR, 0.0);
   d->diag_inp.ks_1 = _ADC_COMPENSATE(diag.knock_value[0], ADC_VREF_FACTOR, 0.0);
   d->diag_inp.add_io1 = _ADC_COMPENSATE(adc_get_add_io1_value(), ADC_VREF_FACTOR, 0.0);
   d->diag_inp.add_io2 = _ADC_COMPENSATE(adc_get_add_io2_value(), ADC_VREF_FACTOR, 0.0);
   d->diag_inp.carb = _ADC_COMPENSATE(adc_get_carb_value(), ADC_VREF_FACTOR, 0.0);
   d->diag_inp.ks_2 = _ADC_COMPENSATE(diag.knock_value[1], ADC_VREF_FACTOR, 0.0);

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
