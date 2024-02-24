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

/** \file hall.c
 * \author Alexey A. Shabelnikov
 * Implementation of Hall sensor's synchronization processing.
 */

#ifdef HALL_SYNC
#if defined(CKPS_2CHIGN)
 #error "You can not use CKPS_2CHIGN option together with HALL_SYNC!"
#endif
#if defined(CKPS_NPLUS1)
 #error "You can not use CKPS_NPLUS1 option together with HALL_SYNC!"
#endif
#if defined(CAM_SYNC)
 #error "You can not use CAM_SYNC option together with HALL_SYNC!"
#endif
#if defined(SPLIT_ANGLE)
 #error "You can not use SPLIT_ANGLE option together with HALL_SYNC!"
#endif
#if defined(ODDFIRE_ALGO)
 #error "You can not use ODDFIRE_ALGO option together with HALL_SYNC!"
#endif

#include <stdlib.h>
#include "port/avrio.h"
#include "port/interrupt.h"
#include "port/intrinsic.h"
#include "port/port.h"
#include "bitmask.h"
#include "ckps.h"
#include "injector.h"   //inject_start_inj()
#include "ioconfig.h"
#include "magnitude.h"
#include "tables.h"     //fnptr_t

#include "knock.h"

//Phase sensor and phased ignition are not supported when hall sensor is used for synchronization
#if defined(PHASE_SENSOR) || defined(PHASED_IGNITION)
 #error "You can not use phase sensor and phased ignition when Hall sensor is used for synchronization"
#endif

/**Maximum number of ignition channels (cylinders) */
#define IGN_CHANNELS_MAX      8

//Define values for controlling of outputs
#define IGN_OUTPUTS_INIT_VAL 1        //!< value used for initialization
#define IGN_OUTPUTS_ON_VAL   1        //!< value used to turn on ignition channel
#define IGN_OUTPUTS_OFF_VAL  0        //!< value used to turn off ignition channel

#ifdef DWELL_CONTROL
#define IGNOUTCB_ON_VAL (hall.ignout_on_val)
#define IGNOUTCB_OFF_VAL (hall.ignout_off_val)
#else
#define IGNOUTCB_ON_VAL IGN_OUTPUTS_ON_VAL
#define IGNOUTCB_OFF_VAL IGN_OUTPUTS_OFF_VAL
#endif

/**Calibration constant used to compensate delay in interrupts (ticks of timer 1) */
#define CALIBRATION_DELAY    2

//Flags (see flags variable)
#define F_ERROR     0                 //!< Hall sensor error flag, set in the Hall sensor interrupt, reset after processing
#define F_SELFRONT  1                 //!< Determines which fron will be used as starting point for advance angle timer
#define F_VHTPER    2                 //!< used to indicate that measured period is valid (actually measured)
#define F_STROKE    3                 //!< flag for synchronization with rotation
#define F_USEKNK    4                 //!< flag which indicates using of knock channel
#define F_HALLEV    5                 //!< flag indicates presence of Hall sensor event
#define F_IGNIEN    6                 //!< Ignition enabled/disabled
#define F_SPSIGN    7                 //!< Sign of the measured stroke period (time between TDCs)

//Additional flags (see flags2 variable)
#define F_SHUTTER   0                 //!< indicates using of shutter entering for spark generation (used at startup)
#define F_SHUTTER_S 1                 //!< synchronized value of F_SHUTTER
#define F_SELEDGE   2                 //!< indicates selected edge type, falling edge is default (see also same definition in camsens.c)
#define F_VDELTAP   3                 //!< indicates validity of stroke_period_prev

/** State variables */
typedef struct
{
 uint16_t measure_start_value;        //!< previous value if timer 1 used for calculation of stroke period
 volatile uint16_t stroke_period;     //!< stores the last measurement of 1 stoke
 volatile uint8_t chan_number;        //!< number of ignition channels
 uint32_t frq_calc_dividend;          //!< divident for calculating of RPM
 volatile int16_t  advance_angle;     //!< required adv.angle * ANGLE_MULTIPLIER
 volatile uint8_t t1oc;               //!< Timer 1 overflow counter
 volatile uint8_t t1oc_s;             //!< Contains value of t1oc synchronized with stroke_period value
 volatile fnptr_t io_callback;        //!< Callback used to set state of ignition channel (we use single channel)
 volatile uint16_t degrees_per_stroke;//!< Number of degrees which corresponds to the 1 stroke
#ifdef STROBOSCOPE
 uint8_t strobe;                      //!< Flag indicates that strobe pulse must be output on pending ignition stroke
#endif
 volatile uint8_t knkwnd_mode;        //!< used to indicate that knock measuring window is opened
 volatile int16_t knock_wnd_begin;    //!< begin of the phase selection window of detonation in degrees * ANGLE_MULTIPLIER, relatively to TDC
 volatile int16_t knock_wnd_end;      //!< width of the phase selection window of detonation in degrees * ANGLE_MULTIPLIER
 int16_t shutter_wnd_width;           //!< Window width (in degrees of cranckshaft) in trigger shutter
 int16_t degrees_btdc;                //!< Degrees before TDC (value * ANGLE_MULTIPLIER)
 int16_t knock_wnd_begin_v;           //!< cached value of the beginning of phase selection window of detonation
#ifdef DWELL_CONTROL
 volatile uint16_t cr_acc_time;       //!< accumulation time for dwell control (timer's ticks)
 volatile uint8_t rising_edge_spark;  //!< flag, indicates that rising edge of ignition pulse will be generated at the moment of spark
 volatile uint8_t ignout_on_val;
 volatile uint8_t ignout_off_val;
#endif
 volatile uint16_t delay;             //!<
#ifdef FUEL_INJECT
 volatile uint8_t cur_chan;           //!< current number of channel for fuel injection
#endif
 uint8_t ckps_inpalt;                 //!< indicates that CKPS is not remapped

 volatile uint8_t TCNT0_H;            //!< For supplementing timer/counter 0 up to 16 bits

 volatile uint16_t stroke_period_prev;//!< previous value of the stroke_period variable, used for calculation of the 1st derivative
 volatile uint16_t stroke_period_meas;//!< stored measured value of stroke period (not corrected)
}hallstate_t;

hallstate_t hall;                     //!< instance of state variables

/** Arrange flags in the free I/O register
 *  note: may be not effective on other MCUs or even cause bugs! Be aware.
 */
#define flags  GPIOR0                 //ATmega644 has one general purpose I/O register
#define flags2 TWBR

/**Table srtores dividends for calculating of RPM */
#define FRQ_CALC_DIVIDEND(channum) PGM_GET_DWORD(&frq_calc_dividend[channum])
PGM_DECLARE(uint32_t frq_calc_dividend[1+IGN_CHANNELS_MAX]) =
 //     1          2          3          4         5         6         7         8
 {0, 37500000L, 18750000L, 12500000L, 9375000L, 7500000L, 6250000L, 5357143L, 4687500L};


void ckps_init_state_variables(void)
{
 _BEGIN_ATOMIC_BLOCK();

 hall.stroke_period = 0xFFFF;
 hall.stroke_period_prev = 0xFFFF;
 hall.stroke_period_meas = 0xFFFF;
 hall.advance_angle = hall.degrees_btdc; //=0

 CLEARBIT(flags, F_STROKE);
 CLEARBIT(flags, F_VHTPER);
 CLEARBIT(flags, F_HALLEV);
 CLEARBIT(flags, F_SPSIGN);
 SETBIT(flags, F_IGNIEN);
 SETBIT(flags2, F_SHUTTER);
 SETBIT(flags2, F_SHUTTER_S);
 CLEARBIT(flags2, F_VDELTAP);

 TIMSK1|=_BV(TOIE1);                  //enable Timer 1 overflow interrupt. Used for correct calculation of very low RPM

 hall.t1oc = 0;                       //reset overflow counter
 hall.t1oc_s = 255;                   //RPM is very low

#ifdef STROBOSCOPE
 hall.strobe = 0;
#endif

 hall.knkwnd_mode = 0;
#ifdef DWELL_CONTROL
 hall.cr_acc_time = 0;
#endif

#ifdef FUEL_INJECT
 hall.cur_chan = 0;
#endif
 _END_ATOMIC_BLOCK();
}

void ckps_init_state(void)
{
 _BEGIN_ATOMIC_BLOCK();

 if (IOCFG_CMP(IOP_CKPS, IOP_CKPS))
 {
  CLEARBIT(flags2, F_SELEDGE); //falling edge
  hall.ckps_inpalt = 1; //not remapped
 }
 else
  hall.ckps_inpalt = 0; //CKPS mapped on other input

 ckps_init_state_variables();
 CLEARBIT(flags, F_ERROR);

 //Compare channels do not connected to lines of ports (normal port mode)
 TCCR1A = 0;

 TCCR1B = _BV(CS11)|_BV(CS10);  //Tune timer 1 (clock = 312.5 kHz)
 TCCR0B = _BV(CS01)|_BV(CS00);  //Tune timer 0 (clock = 312.5 kHz)

 if (hall.ckps_inpalt)
  TIMSK1|=_BV(ICIE1);    //enable input capture interrupt only if CKPS is not remapped
 _END_ATOMIC_BLOCK();
}

void ckps_set_advance_angle(int16_t angle)
{
 int16_t aad = (hall.degrees_btdc - angle);
 _BEGIN_ATOMIC_BLOCK();
 hall.advance_angle = aad;
 _END_ATOMIC_BLOCK();
}

void ckps_init_ports(void)
{
 IOCFG_INIT(IOP_CKPS, 1); // pullup for ICP1

 //after ignition is on, igniters must not be in the accumulation mode,
 //therefore set low level on their inputs
 IOCFG_INIT(IOP_IGN_OUT1, IGN_OUTPUTS_INIT_VAL);        //init 1-st (can be remapped)
 IOCFG_INIT(IOP_IGN_OUT2, IGN_OUTPUTS_INIT_VAL);        //init 2-nd (can be remapped)
 IOCFG_INIT(IOP_IGN_OUT3, IGN_OUTPUTS_INIT_VAL);        //init 3-rd (can be remapped)
 IOCFG_INIT(IOP_IGN_OUT4, IGN_OUTPUTS_INIT_VAL);        //init 4-th (can be remapped)
 IOCFG_INIT(IOP_IGN_OUT5, IGN_OUTPUTS_INIT_VAL);        //init 5-th (can be remapped)
 IOCFG_INIT(IOP_IGN_OUT6, IGN_OUTPUTS_INIT_VAL);        //init 6-th (can be remapped)
 IOCFG_INIT(IOP_IGN_OUT7, IGN_OUTPUTS_INIT_VAL);        //init 7-th (for maniacs)
 IOCFG_INIT(IOP_IGN_OUT8, IGN_OUTPUTS_INIT_VAL);        //init 8-th (for maniacs)

 //init I/O for Hall output if it is enabled
#ifdef HALL_OUTPUT
 IOCFG_INIT(IOP_HALL_OUT, 1);
#endif

 //init I/O for stroboscope
#ifdef STROBOSCOPE
 IOCFG_INIT(IOP_STROBE, 0);
#endif
}

/** Get a 32-bit value of period, taking into accound overflows
 */
#define GET_OVF_AWARE_PERIOD(sign, ovfcnt, period) \
 (((sign) && (ovfcnt) > 0) ? ((((int32_t)ovfcnt) * 65536) - (65536-(period))) : ((((int32_t)ovfcnt) * 65536) + (period)))


//Instantaneous frequency calculation of crankshaft rotation from the measured period between the engine strokes
//(for example for 4-cylinder, 4-stroke it is 180�)
//Period measured in the discretes of timer (one discrete = 3.2us), one minute = 60 seconds, one second has 1,000,000 us.
uint16_t ckps_calculate_instant_freq(void)
{
 uint16_t period; uint8_t ovfcnt, sign;
 //ensure atomic acces to variable
 _DISABLE_INTERRUPT();
 period = hall.stroke_period;        //stroke period
 ovfcnt = hall.t1oc_s;               //number of timer overflows
 sign = CHECKBIT(flags, F_SPSIGN);   //sign of stroke period
 _ENABLE_INTERRUPT();

 if (0xFF == ovfcnt)
  return 0; //engine is stopped

 //We know period and number of timer overflows, so we can calculate correct value of RPM even if RPM is very low
 return hall.frq_calc_dividend / GET_OVF_AWARE_PERIOD(sign, ovfcnt, period);
}

uint16_t ckps_get_stroke_period(void)
{
 uint16_t period;
 _DISABLE_INTERRUPT();
 period = hall.stroke_period;        //stroke period
 _ENABLE_INTERRUPT();
 return period;
}

void ckps_set_edge_type(uint8_t edge_type)
{
 //Set CKPS input edge only if it is not remapped
 if (hall.ckps_inpalt)
 {
  WRITEBIT(flags2, F_SELEDGE, edge_type); //save selected edge type
  _BEGIN_ATOMIC_BLOCK();
  if (edge_type)
   TCCR1B|= _BV(ICES1);              //rising
  else
   TCCR1B&=~_BV(ICES1);              //falling
  _END_ATOMIC_BLOCK();
 }
}

void ckps_set_cogs_btdc(uint8_t cogs_btdc)
{
 //not supported by Hall sensor
}

#ifndef DWELL_CONTROL
void ckps_set_ignition_cogs(uint8_t cogs)
{
 //not supported by Hall sensor
}
#else
void ckps_set_acc_time(uint16_t i_acc_time)
{
 _BEGIN_ATOMIC_BLOCK();
 hall.cr_acc_time = i_acc_time;
 _END_ATOMIC_BLOCK();
}

void ckps_set_rising_spark(uint8_t rising_edge)
{
 _BEGIN_ATOMIC_BLOCK();
 hall.rising_edge_spark = rising_edge;
 if (rising_edge) { //spark on rising edge
  hall.ignout_on_val = IGN_OUTPUTS_OFF_VAL;
  hall.ignout_off_val = IGN_OUTPUTS_ON_VAL;
 }
 else { //spark on falling edge
  hall.ignout_on_val = IGN_OUTPUTS_ON_VAL;
  hall.ignout_off_val = IGN_OUTPUTS_OFF_VAL;
 }
 _END_ATOMIC_BLOCK();
}
#endif

uint8_t ckps_is_error(void)
{
 return CHECKBIT(flags, F_ERROR) > 0;
}

void ckps_reset_error(void)
{
 CLEARBIT(flags, F_ERROR);
}

void ckps_use_knock_channel(uint8_t use_knock_channel)
{
 WRITEBIT(flags, F_USEKNK, use_knock_channel);
}

uint8_t ckps_is_stroke_event_r()
{
 uint8_t result;
 _BEGIN_ATOMIC_BLOCK();
 result = CHECKBIT(flags, F_STROKE) > 0;
 CLEARBIT(flags, F_STROKE);
 _END_ATOMIC_BLOCK();
 return result;
}

uint8_t ckps_is_cog_changed(void)
{
 uint8_t result;
 _BEGIN_ATOMIC_BLOCK();
 result = CHECKBIT(flags, F_HALLEV) > 0;
 CLEARBIT(flags, F_HALLEV);
 _END_ATOMIC_BLOCK();
 return result;
}

void ckps_set_cyl_number(uint8_t i_cyl_number)
{
 uint16_t degrees_per_stroke;
 uint8_t _t;
 _t = _SAVE_INTERRUPT();
 _DISABLE_INTERRUPT();
 hall.chan_number = i_cyl_number; //set new value
 _RESTORE_INTERRUPT(_t);

 hall.frq_calc_dividend = FRQ_CALC_DIVIDEND(i_cyl_number);

 //precalculate value of degrees per 1 engine stroke (value * ANGLE_MULTIPLIER)
 degrees_per_stroke = (720 * ANGLE_MULTIPLIER) / i_cyl_number;

 _t = _SAVE_INTERRUPT();
 _DISABLE_INTERRUPT();
 hall.io_callback = IOCFG_CB(IOP_IGN_OUT1); //use single output
 hall.degrees_per_stroke = degrees_per_stroke;
 _RESTORE_INTERRUPT(_t);
}

void ckps_set_knock_window(int16_t begin, int16_t end)
{
 int16_t begin_d = (hall.degrees_btdc + begin); //start of window
 int16_t end_d = (end - begin); //width of window
 hall.knock_wnd_begin_v = begin; //save begin value to use in other setters
 _BEGIN_ATOMIC_BLOCK();
 hall.knock_wnd_begin = begin_d;
 hall.knock_wnd_end = end_d;
 _END_ATOMIC_BLOCK();
}

void ckps_enable_ignition(uint8_t i_cutoff)
{
 WRITEBIT(flags, F_IGNIEN, i_cutoff); //enable/disable ignition
}

void ckps_set_merge_outs(uint8_t i_merge)
{
 //not suitable when Hall sensor synchronization is used
}

#ifdef HALL_OUTPUT
void ckps_set_hall_pulse(int16_t i_offset, uint16_t i_duration)
{
 //not supported by Hall sensor
}
#endif

void ckps_set_cogs_num(uint8_t norm_num, uint8_t miss_num)
{
 //not supported by Hall sensor
}

#ifdef FUEL_INJECT
void ckps_set_inj_timing(int16_t phase, uint16_t pw, uint8_t mode)
{
 //not supported in this implementation
}
#endif

void ckps_set_shutter_spark(uint8_t i_shutter)
{
 _BEGIN_ATOMIC_BLOCK(); //we need this because in ATmega644 this flag is not in I/O register
 WRITEBIT(flags2, F_SHUTTER, i_shutter);
 _END_ATOMIC_BLOCK();
}

void ckps_set_shutter_wnd_width(int16_t width)
{
 hall.shutter_wnd_width = width; //save it to use in other setters
}

void ckps_set_degrees_btdc(int16_t degrees_btdc)
{
 int16_t begin_d = (degrees_btdc + hall.knock_wnd_begin_v); //start of window
 hall.degrees_btdc = degrees_btdc; //save it to use in other setters
 _BEGIN_ATOMIC_BLOCK();
 hall.knock_wnd_begin = begin_d;
 _END_ATOMIC_BLOCK();
}

/** Turn OFF specified ignition channel
 * \param i_channel number of ignition channel to turn off
 */
static inline void turn_off_ignition_channel(void)
{
 if (!CHECKBIT(flags, F_IGNIEN))
  return; //ignition disabled
 //Completion of igniter's ignition drive pulse, transfer line of port into a low level - makes
 //the igniter go to the regime of energy accumulation
 ((iocfg_pfn_set)hall.io_callback)(IGNOUTCB_OFF_VAL);
}

/**Check for timer1 overflow during measuring of period*/
#define CHECK_TIM1_OVF() ((CHECKBIT(flags, F_SPSIGN) && hall.t1oc_s < 2) || (!CHECKBIT(flags, F_SPSIGN) && !hall.t1oc_s))

/**Interrupt handler for Compare/Match channel A of timer T1
 */
ISR(TIMER1_COMPA_vect)
{
 uint16_t tmr = TCNT1;
 ((iocfg_pfn_set)hall.io_callback)(IGNOUTCB_ON_VAL);
 TIMSK1&= ~_BV(OCIE1A);//disable interrupt

 //-----------------------------------------------------
 //Software PWM is very sensitive even to small delays. So, we need to allow OCF2 and TOV2
 //interrupts occur during processing of this handler.
#ifdef COOLINGFAN_PWM
 _ENABLE_INTERRUPT();
#endif
 //-----------------------------------------------------

#ifndef DWELL_CONTROL
 //set timer for pulse completion, use fast division by 3
 if (CHECK_TIM1_OVF())
  OCR1B = tmr + (((uint32_t)hall.stroke_period * 21845) >> 16); //pulse width = 1/3, 21845 = 1/3 * 65536
 else
  OCR1B = tmr + 21845;  //pulse width is limited to 87.38ms
#else
 if (hall.rising_edge_spark)
 {
#ifdef COOLINGFAN_PWM
 _DISABLE_INTERRUPT();
#endif
  int32_t period32 = GET_OVF_AWARE_PERIOD(CHECKBIT(flags, F_SPSIGN), hall.t1oc_s, hall.stroke_period);
  if (period32 < 32768) { //prevent error caused by overflow
   if (hall.cr_acc_time > hall.stroke_period-120)
    hall.cr_acc_time = hall.stroke_period-120;  //restrict accumulation time. Dead band = 500us 
  }
  OCR1B = tmr + hall.cr_acc_time;
#ifdef COOLINGFAN_PWM
 _ENABLE_INTERRUPT();
#endif
 }
 else
 { //use dwell control
  if (!CHECKBIT(flags2, F_SHUTTER_S)) //do not set dwell timer here in the shutter-spark mode, because in this mode we set it in the FallingEdge function
  {
   if (CHECK_TIM1_OVF())
   {
    if (hall.cr_acc_time > hall.stroke_period-120)
     hall.cr_acc_time = hall.stroke_period-120;  //restrict accumulation time. Dead band = 500us 
    OCR1B  = tmr + hall.stroke_period - hall.cr_acc_time;
   }
   else
    OCR1B = tmr + 60000;  //pulse width is limited to 192 ms
  }
 }
#endif

#ifdef COOLINGFAN_PWM
 _DISABLE_INTERRUPT();
#endif

#ifdef DWELL_CONTROL
 if (!CHECKBIT(flags2, F_SHUTTER_S) || hall.rising_edge_spark)
 {
#endif
  TIFR1 = _BV(OCF1B);
  TIMSK1|= _BV(OCIE1B);
#ifdef DWELL_CONTROL
 }
#endif

#ifdef HALL_OUTPUT
 IOCFG_SET(IOP_HALL_OUT, 1);//begin of pulse
#endif

#ifdef STROBOSCOPE
 if (1==hall.strobe)
 {
  IOCFG_SET(IOP_STROBE, 1); //start pulse
  hall.strobe = 2;          //and set flag to next state
  OCR1A = TCNT1 + 31;       //We will generate 100uS pulse
  TIMSK1|= _BV(OCIE1A);     //pulse will be ended in the next interrupt
 }
 else if (2==hall.strobe)
 {
  IOCFG_SET(IOP_STROBE, 0); //end pulse
  hall.strobe = 0;          //and reset flag
  return;
 }
#endif
}

/**Interrupt handler for Compare/Match channel B of timer T1.
 */
ISR(TIMER1_COMPB_vect)
{
 turn_off_ignition_channel();//finish ignition pulse
 TIMSK1&= ~_BV(OCIE1B);     //disable interrupt

#ifdef HALL_OUTPUT
 IOCFG_SET(IOP_HALL_OUT, 0);//end of pulse
#endif
}


/**Initialize timer 0 using specified value and start it, clock = 312.5kHz
 * It is assumed that this function called when all interrupts are disabled
 * \param value Value to set timer for, 1 tick = 3.2uS
 */
static inline void set_timer0(uint16_t value)
{
 uint8_t TCNT0_L = _AB(value, 0);
 if (!TCNT0_L)
  TCNT0_L++;
 OCR0A = TCNT0 + TCNT0_L;
 SETBIT(TIFR0, OCF0A);
 hall.TCNT0_H = _AB(value, 1);
 SETBIT(TIMSK0, OCIE0A);
}

/** Special function for processing of falling edge,
 * must be called from ISR
 * \param tmr Timer value at the moment of falling edge
 */
static inline void ProcessFallingEdge(uint16_t tmr)
{
 //save period value if it is correct. We need to do it first of all to have fresh stroke_period value
 if (CHECKBIT(flags, F_VHTPER))
 {
  //calculate stroke period
  hall.stroke_period_meas = tmr - hall.measure_start_value;
  WRITEBIT(flags, F_SPSIGN, tmr < hall.measure_start_value); //save sign
  hall.t1oc_s = hall.t1oc, hall.t1oc = 0; //save value and reset counter
  ///prediction/////////////////////////////////////
  if ((hall.t1oc_s == 0 && CHECKBIT(flags2, F_VDELTAP)) && PGM_GET_BYTE(&fw_data.exdata.hall_predict))
  { //with prediction (only if there were no overflows and stroke_period_prev is valid)
   int32_t newperiod = ((int32_t)hall.stroke_period_meas) + (((int32_t)hall.stroke_period_meas) - hall.stroke_period_prev); //1st derivative is used for prediction
   if (newperiod > 65500)
    newperiod = 65500;
   hall.stroke_period = newperiod;
  }
  else //without prediction (use last interval)
   hall.stroke_period = hall.stroke_period_meas;
  hall.stroke_period_prev = hall.stroke_period_meas;
  SETBIT(flags2, F_VDELTAP);
  ///prediction/////////////////////////////////////
 }
 SETBIT(flags, F_VHTPER);
 SETBIT(flags, F_STROKE); //set the stroke-synchronization event
 hall.measure_start_value = tmr;

 if (!CHECKBIT(flags2, F_SHUTTER_S))
 {
#ifdef STROBOSCOPE
  hall.strobe = 1; //strobe!
#endif

  //-----------------------------------------------------
  //Software PWM is very sensitive even to small delays. So, we need to allow OCF2 and TOV2
  //interrupts occur during processing of this handler.
#ifdef COOLINGFAN_PWM
  _ENABLE_INTERRUPT();
#endif
  //-----------------------------------------------------

  //get 32 bit, overflow aware period value
  int32_t period32 = GET_OVF_AWARE_PERIOD(CHECKBIT(flags, F_SPSIGN), hall.t1oc_s, hall.stroke_period);

  // if number of degrees before a spark is less or equal to window width in interrupter, then use falling edge
  // as a reference for advance angle timer, otherwise use rising edge (that will increase accuracy)
  if (hall.advance_angle <= hall.shutter_wnd_width || hall.shutter_wnd_width == 0)
  {
   CLEARBIT(flags, F_SELFRONT);// use falling edge

   //start timer for counting out of advance angle (spark)
   hall.delay = (((uint32_t)hall.advance_angle * period32) / hall.degrees_per_stroke);
#ifdef COOLINGFAN_PWM
   _DISABLE_INTERRUPT();
#endif

   OCR1A = tmr + ((hall.delay < 15) ? 15 : hall.delay) - CALIBRATION_DELAY; //set compare channel, additionally prevent spark missing when advance angle is near to 60�
   TIFR1 = _BV(OCF1A);
   TIMSK1|= _BV(OCIE1A);
  }
  else
  { //we profit! (accuracy will be better)
   SETBIT(flags, F_SELFRONT);// use rising edge
   hall.delay = ((uint32_t)(hall.advance_angle-hall.shutter_wnd_width) * period32) / hall.degrees_per_stroke;
#ifdef COOLINGFAN_PWM
   _DISABLE_INTERRUPT();
#endif
  }

  //start timer for countiong out of knock window opening
  if (CHECKBIT(flags, F_USEKNK))
  {
   uint16_t delay;
#ifdef COOLINGFAN_PWM
   _ENABLE_INTERRUPT();
#endif
   delay = ((uint32_t)hall.knock_wnd_begin * hall.stroke_period) / hall.degrees_per_stroke;
#ifdef COOLINGFAN_PWM
   _DISABLE_INTERRUPT();
#endif
   set_timer0(delay);
   hall.knkwnd_mode = 0;
  }

  adc_begin_measure(_AB(hall.stroke_period, 1) < 4);//start the process of measuring analog input values
 }
#ifdef DWELL_CONTROL
 else if (!hall.rising_edge_spark)
 {
  CLEARBIT(flags, F_SELFRONT);
  //in shutter-spark mode we start dwell timer here. This will ensure good accuracy at very low RPMs
  int32_t period32 = GET_OVF_AWARE_PERIOD(CHECKBIT(flags, F_SPSIGN), hall.t1oc_s, hall.stroke_period);
#ifdef COOLINGFAN_PWM
  _ENABLE_INTERRUPT();
#endif
  int32_t delay = ((uint32_t)hall.shutter_wnd_width * period32) / hall.degrees_per_stroke;
#ifdef COOLINGFAN_PWM
  _DISABLE_INTERRUPT();
#endif
  if (delay < 65536)
  {
   if (hall.cr_acc_time > delay-120)
    hall.cr_acc_time = delay-120;  //restrict accumulation time. Dead band = 500us 
   OCR1B  = tmr + delay - hall.cr_acc_time;
  }
  else
   OCR1B  = tmr + 60000; //time is limited to 192 ms

  TIFR1 = _BV(OCF1B);
  TIMSK1|= _BV(OCIE1B);
 }
#endif
}

/** Special function for processing rising edge, must be called from ISR
 */
static inline void ProcessRisingEdge(uint16_t tmr)
{
 //spark on rising at the startup, force COMPA interrupt
 if (CHECKBIT(flags2, F_SHUTTER_S))
 {
#ifdef STROBOSCOPE
   hall.strobe = 1; //strobe!
#endif

  OCR1A = TCNT1 + 2;
  TIFR1 = _BV(OCF1A);
  TIMSK1|= _BV(OCIE1A);
 }
 else if (CHECKBIT(flags, F_SELFRONT))
 { // use rising edge as reference point
  OCR1A = tmr + ((hall.delay < 15) ? 15 : hall.delay) - CALIBRATION_DELAY;
  TIFR1 = _BV(OCF1A);
  TIMSK1|= _BV(OCIE1A);
 }

#ifdef FUEL_INJECT
 inject_start_inj(hall.cur_chan);     //start fuel injection
 if (++hall.cur_chan >= hall.chan_number)
  hall.cur_chan = 0;
#endif
}

/**Input capture interrupt of timer 1 */
ISR(TIMER1_CAPT_vect)
{
 //toggle edge
 if (!(TCCR1B & _BV(ICES1)))
 { //falling
  if (CHECKBIT(flags2, F_SELEDGE))
   ProcessRisingEdge(ICR1);
  else
   ProcessFallingEdge(ICR1);
  TCCR1B|= _BV(ICES1);  //next edge will be rising
 }
 else
 { //rising
  if (CHECKBIT(flags2, F_SELEDGE))
   ProcessFallingEdge(ICR1);
  else
   ProcessRisingEdge(ICR1);
  TCCR1B&= ~_BV(ICES1); //next will be falling
 }

 WRITEBIT(flags2, F_SHUTTER_S, CHECKBIT(flags2, F_SHUTTER)); //synchronize
 SETBIT(flags, F_HALLEV); //set event flag
}

/**INT1 handler function (Interrupt from a Hall sensor (external))
 * Called from camsens.c
 */
void ProcessInterrupt1(void) //see also prototype of this function in camsens.c
{
 uint16_t tmr = TCNT1;
 //toggle edge
 if (EICRA & _BV(ISC10))
 { //falling
  if (CHECKBIT(flags2, F_SELEDGE))
   ProcessRisingEdge(tmr);
  else
   ProcessFallingEdge(tmr);
  EICRA&= ~(_BV(ISC10));  //next edge will be rising
 }
 else
 { //rising
  if (CHECKBIT(flags2, F_SELEDGE))
   ProcessFallingEdge(tmr);
  else
   ProcessRisingEdge(tmr);
  EICRA|=_BV(ISC10); //next will be falling
 }

 WRITEBIT(flags2, F_SHUTTER_S, CHECKBIT(flags2, F_SHUTTER)); //synchronize
 SETBIT(flags, F_HALLEV); //set event flag
}

/**INT0 handler function (Interrupt from a Hall sensor (external))
 * Called from camsens.c
 */
void ProcessInterrupt0(void) //see also prototype of this function in camsens.c
{
 uint16_t tmr = TCNT1;
 //toggle edge
 if (EICRA & _BV(ISC00))
 { //falling
  if (CHECKBIT(flags2, F_SELEDGE))
   ProcessRisingEdge(tmr);
  else
   ProcessFallingEdge(tmr);
  EICRA&= ~(_BV(ISC00));  //next edge will be rising
 }
 else
 { //rising
  if (CHECKBIT(flags2, F_SELEDGE))
   ProcessFallingEdge(tmr);
  else
   ProcessRisingEdge(tmr);
  EICRA|=_BV(ISC00); //next will be falling
 }

 WRITEBIT(flags2, F_SHUTTER_S, CHECKBIT(flags2, F_SHUTTER)); //synchronize
 SETBIT(flags, F_HALLEV); //set event flag
}


/**Purpose of this interrupt handler is to supplement timer up to 16 bits and call procedures
 * for opening and closing knock measuring window
 */
ISR(TIMER0_COMPA_vect)
{
 if (hall.TCNT0_H!=0)  //Did high byte exhaust ?
 {
  TCNT0 = 0;
  --hall.TCNT0_H;
 }
 else
 {//the countdown is over
  CLEARBIT(TIMSK0, OCIE0A);    //disable this interrupt

  if (!hall.knkwnd_mode)
  {//start listening detonation (opening the window)
   uint16_t delay;
   knock_set_integration_mode(KNOCK_INTMODE_INT);
   ++hall.knkwnd_mode;
   //-----------------------------------------------------
   //Software PWM is very sensitive even to small delays. So, we need to allow OCF2 and TOV2
   //interrupts occur during processing of this handler.
#ifdef COOLINGFAN_PWM
   _ENABLE_INTERRUPT();
#endif
   //-----------------------------------------------------
   delay = ((uint32_t)hall.knock_wnd_end * hall.stroke_period) / hall.degrees_per_stroke;
#ifdef COOLINGFAN_PWM
   _DISABLE_INTERRUPT();
#endif
   set_timer0(delay);
  }
  else
  {//finish listening a detonation (closing the window) and start the process of measuring integrated value
   knock_set_integration_mode(KNOCK_INTMODE_HOLD);
   knock_start_settings_latching();//start the process of downloading the settings into the HIP9011 (and getting ADC result from TPIC8101)
#ifndef TPIC8101
   adc_begin_measure_knock(_AB(hall.stroke_period, 1) < 4);
#endif
   hall.knkwnd_mode = 0;
  }
 }
}

/** Timer 1 overflow interrupt.
 * Used to count timer 1 overflows to obtain correct revolution period at very low RPMs (10...400 min-1)
 */
ISR(TIMER1_OVF_vect)
{
 ++hall.t1oc;
}

void ckps_set_knock_chanmap(uint8_t chanmap)
{
 //not implemented
}

#endif //HALL_SYNC
