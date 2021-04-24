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

/** \file ckpsn+1.c
 * \author Alexey A. Shabelnikov
 * Implementation of synchronization processing for Kawasaki ZX6R.
 */

#ifdef CKPS_NPLUS1
#if defined(CKPS_2CHIGN)
 #error "You can not use CKPS_2CHIGN option together with CKPS_NPLUS1"
#endif
#if defined(HALL_SYNC)
 #error "You can not use HALL_SYNC option together with CKPS_NPLUS1"
#endif
#if defined(CAM_SYNC)
 #error "You can not use CAM_SYNC option together with CKPS_NPLUS1"
#endif
#if defined(SPLIT_ANGLE)
 #error "You can not use SPLIT_ANGLE option together with CKPS_NPLUS1"
#endif
#if defined(ODDFIRE_ALGO)
 #error "You can not use ODDFIRE_ALGO option together with CKPS_NPLUS1"
#endif

#include <stdlib.h>
#include "port/avrio.h"
#include "port/interrupt.h"
#include "port/intrinsic.h"
#include "port/port.h"
#include "bitmask.h"
#include "ckps.h"
#include "ioconfig.h"
#include "magnitude.h"
#include "tables.h"     //fnptr_t

#include "knock.h"

//Phase sensor and phased ignition are not supported when this method is used for synchronization
#if defined(PHASE_SENSOR) || defined(PHASED_IGNITION)
 #error "You can not use phase sensor and phased ignition when N+1 method is used for synchronization!"
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

/**How many cogs must be skipped during start up*/
#define HALL_ON_START_SKIP_COGS 2

/**Synchronization contition. Polinomial is equal to v / 2.66 */
#define HALL_SYNCCOND(v) (((v) >> 2) + ((v) >> 3))

/**Number of cogs including small one, so 2+1 */
#define HALL_COGS_NUM 3

//Flags (see flags variable)
#define F_ERROR     0                 //!< Hall sensor error flag, set in the Hall sensor interrupt, reset after processing
#define F_VHTPER    2                 //!< used to indicate that measured period is valid (actually measured)
#define F_STROKE    3                 //!< flag for synchronization with rotation
#define F_USEKNK    4                 //!< flag which indicates using of knock channel
#define F_HALLEV    5                 //!< flag indicates presence of Hall sensor event
#define F_IGNIEN    6                 //!< Ignition enabled/disabled
#define F_SPSIGN    7                 //!< Sign of the measured stroke period (time between TDCs)

//Additional flags (see flags2 variable)
#define F_STRMOD    0                 //!< used for synchronization during cranking
#define F_ISSYNC    1                 //!< used to indicate that system has already synchronized
#define F_SELEDGE   2                 //!< indicates selected edge type, falling edge is default

/** State variables */
typedef struct
{
 uint16_t measure_start_value;        //!< previous value if timer 1 used for calculation of stroke period
 uint16_t tmr_prev;                   //!< same as measure_start_value but used for determining cog period
 volatile uint16_t stroke_period;     //!< stores the last measurement of 1 stoke
 uint8_t cog;                         //!< current cog
 uint16_t cog_period;                 //!< same as stroke period, but may also contain period value of small tooth
 uint16_t cog_period_prev;            //!< previous value of cog_period
 volatile uint8_t chan_number;        //!< number of ignition channels
 uint32_t frq_calc_dividend;          //!< divident for calculating of RPM
 volatile int16_t  advance_angle;     //!< required adv.angle * ANGLE_MULTIPLIER
 volatile uint8_t t1oc;               //!< Timer 1 overflow counter
 volatile uint8_t t1oc_s;             //!< Contains value of t1oc synchronized with stroke_period value
 volatile fnptr_t io_callback[HALL_COGS_NUM-1]; //!< Callbacks used to set state of corresponding ignition channel
 uint8_t channel_mode_b;              //!< channel index for use in COMPB interrupt
 uint8_t channel_mode_a;              //!< channel index for use in COMPA interrupt
 volatile uint16_t degrees_per_stroke;//!< Number of degrees which corresponds to the 1 stroke
#ifdef STROBOSCOPE
 uint8_t strobe;                      //!< Flag indicates that strobe pulse must be output on pending ignition stroke
#endif
 volatile uint8_t knkwnd_mode;        //!< used to indicate that knock measuring window is opened
 volatile int16_t knock_wnd_begin;    //!< begin of the phase selection window of detonation in degrees * ANGLE_MULTIPLIER, relatively to TDC
 volatile int16_t knock_wnd_end;      //!< width of the phase selection window of detonation in degrees * ANGLE_MULTIPLIER
 int16_t shutter_wnd_width;           //!< Window width (in degrees of cranckshaft) in trigger shutter
 int16_t knock_wnd_begin_v;           //!< cached value of the beginning of phase selection window of detonation
#ifdef DWELL_CONTROL
 volatile uint16_t cr_acc_time;       //!< accumulation time for dwell control (timer's ticks)
 volatile uint8_t rising_edge_spark;  //!< flag, indicates that rising edge of ignition pulse will be generated at the moment of spark
 volatile uint8_t ignout_on_val;
 volatile uint8_t ignout_off_val;
#endif
 uint8_t ckps_inpalt;                 //!< indicates that CKPS is not remapped

 volatile uint8_t TCNT0_H;            //!< For supplementing timer/counter 0 up to 16 bits
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
 hall.advance_angle = hall.shutter_wnd_width; //=0
 hall.cog = 1;

 CLEARBIT(flags, F_STROKE);
 CLEARBIT(flags, F_VHTPER);
 CLEARBIT(flags, F_HALLEV);
 CLEARBIT(flags, F_SPSIGN);
 CLEARBIT(flags2, F_STRMOD);
 CLEARBIT(flags2, F_ISSYNC);
 SETBIT(flags, F_IGNIEN);

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
 _END_ATOMIC_BLOCK();
}

void ckps_init_state(void)
{
 _BEGIN_ATOMIC_BLOCK();
 if ((IOCFG_CB(IOP_CKPS) == (fnptr_t)iocfg_g_ckps) || (IOCFG_CB(IOP_CKPS) == (fnptr_t)iocfg_g_ckpsi))
 {
  CLEARBIT(flags2, F_SELEDGE);        //falling edge
  hall.ckps_inpalt = 1;               //not remapped
 }
 else
  hall.ckps_inpalt = 0;               //CKPS mapped on other input

 ckps_init_state_variables();
 CLEARBIT(flags, F_ERROR);

 //Compare channels not connected to lines of ports (normal port mode)
 TCCR1A = 0;

 TCCR1B = _BV(CS11)|_BV(CS10); //Tune timer 1 (clock = 312.5 kHz)
 TCCR0B = _BV(CS01)|_BV(CS00); //tune timer 0 (clock = 312.5 kHz)

 if (hall.ckps_inpalt)
  TIMSK1|=_BV(ICIE1);    //enable input capture interrupt only if CKPS is not remapped
 _END_ATOMIC_BLOCK();
}

void ckps_set_advance_angle(int16_t angle)
{
 int16_t aad = (hall.shutter_wnd_width - angle);
 _BEGIN_ATOMIC_BLOCK();
 hall.advance_angle = aad;
 _END_ATOMIC_BLOCK();
}

void ckps_init_ports(void)
{
 PORTD|= _BV(PD6); // pullup for ICP1

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

//Instantaneous frequency calculation of crankshaft rotation from the measured period between the engine strokes
//(for example for 4-cylinder, 4-stroke it is 180�)
//Period measured in the discretes of timer (one discrete = 4us), one minute = 60 seconds, one second has 1,000,000 us.
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
 if (sign && ovfcnt > 0)
  return hall.frq_calc_dividend / ((((int32_t)ovfcnt) * 65536) - (65536-period));
 else
  return hall.frq_calc_dividend / ((((int32_t)ovfcnt) * 65536) + period);
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
 //Set edge of CKPS input only if it is not remapped
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

/**Tune channels' I/O for semi-sequential ignition mode (wasted spark) */
static void set_channels_ss(void)
{
 uint8_t _t, i = 0, chan = hall.chan_number / 2;
 for(; i < chan; ++i)
 {
  fnptr_t value = IOCFG_CB(i);
  _t=_SAVE_INTERRUPT();
  _DISABLE_INTERRUPT();
  hall.io_callback[i] = value;
  _RESTORE_INTERRUPT(_t);
 }
}

void ckps_set_cyl_number(uint8_t i_cyl_number)
{
 uint16_t degrees_per_stroke;
 uint8_t _t;

 hall.frq_calc_dividend = FRQ_CALC_DIVIDEND(i_cyl_number);
 //precalculate value of degrees per 1 engine stroke (value * ANGLE_MULTIPLIER)
 degrees_per_stroke = (720 * ANGLE_MULTIPLIER) / i_cyl_number;

 _t = _SAVE_INTERRUPT();
 _DISABLE_INTERRUPT();
 hall.chan_number = i_cyl_number; //set new value
 hall.degrees_per_stroke = degrees_per_stroke;
 _RESTORE_INTERRUPT(_t);

 set_channels_ss();
}

void ckps_set_knock_window(int16_t begin, int16_t end)
{
 int16_t begin_d = (hall.shutter_wnd_width + begin); //start of window
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
 //not supported in this implementation, because number of cogs must be equal to number of cylinders
}

#ifdef FUEL_INJECT
void ckps_set_inj_timing(int16_t phase, uint16_t pw, uint8_t mode)
{
 //not supported in this implementation
}
#endif

void ckps_set_shutter_spark(uint8_t i_shutter)
{
 //not supported in this implementation
}

void ckps_set_shutter_wnd_width(int16_t width)
{
 int16_t begin_d = (width + hall.knock_wnd_begin_v); //start of window
 hall.shutter_wnd_width = width; //save it to use in other setters
 _BEGIN_ATOMIC_BLOCK();
 hall.knock_wnd_begin = begin_d;
 _END_ATOMIC_BLOCK();
}

void ckps_set_degrees_btdc(int16_t degrees_btdc)
{
 //not supported
}

/** Turn OFF specified ignition channel
 * \param i_channel number of ignition channel to turn off
 */
static inline void turn_off_ignition_channel(uint8_t i_channel)
{
 if (!CHECKBIT(flags, F_IGNIEN))
  return; //ignition disabled
 //Completion of igniter's ignition drive pulse, transfer line of port into a low level - makes
 //the igniter go to the regime of energy accumulation
 ((iocfg_pfn_set)hall.io_callback[i_channel])(IGNOUTCB_OFF_VAL);
}

/**Interrupt handler for Compare/Match channel A of timer T1
 */
ISR(TIMER1_COMPA_vect)
{
 uint16_t tmr = TCNT1;
 ((iocfg_pfn_set)hall.io_callback[hall.channel_mode_a])(IGNOUTCB_ON_VAL);
 TIMSK1&= ~_BV(OCIE1A); //disable interrupt

 //-----------------------------------------------------
 //Software PWM is very sensitive even to small delays. So, we need to allow OCF2 and TOV2
 //interrupts occur during processing of this handler.
#ifdef COOLINGFAN_PWM
 _ENABLE_INTERRUPT();
#endif
 //-----------------------------------------------------

#ifndef DWELL_CONTROL
 hall.channel_mode_b = hall.channel_mode_a;
 //set timer for pulse completion, use fast division by 3
 if ((CHECKBIT(flags, F_SPSIGN) && hall.t1oc_s < 2) || (!CHECKBIT(flags, F_SPSIGN) && !hall.t1oc_s))
//OCR1B = tmr + (((uint32_t)hall.stroke_period * 0xAAAB) >> 17); //pulse width = 1/3
  OCR1B = tmr + hall.stroke_period / 3;
 else
  OCR1B = tmr + 21845;  //pulse width is limited to 87.38ms
#else
 if (hall.rising_edge_spark)
 {
  hall.channel_mode_b = hall.channel_mode_a;
  if (hall.cr_acc_time > hall.stroke_period-120)
   hall.cr_acc_time = hall.stroke_period-120;  //restrict accumulation time. Dead band = 500us 
  OCR1B = tmr + hall.cr_acc_time;
 }
 else
 {
  hall.channel_mode_b = (hall.channel_mode_a < (hall.chan_number>>1)-1) ? hall.channel_mode_a + 1 : 0;
  if ((CHECKBIT(flags, F_SPSIGN) && hall.t1oc_s < 2) || (!CHECKBIT(flags, F_SPSIGN) && !hall.t1oc_s))
  {
   if (hall.cr_acc_time > hall.stroke_period-120)
    hall.cr_acc_time = hall.stroke_period-120;  //restrict accumulation time. Dead band = 500us 
   OCR1B  = tmr + hall.stroke_period - hall.cr_acc_time;
  }
  else
   OCR1B = tmr + 21845;  //pulse width is limited to 87.38ms
 }
#endif

#ifdef COOLINGFAN_PWM
 _DISABLE_INTERRUPT();
#endif

 TIFR1 = _BV(OCF1B);
 TIMSK1|= _BV(OCIE1B);

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
 turn_off_ignition_channel(hall.channel_mode_b);//finish ignition pulse
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

/** Special function for processing falling edge, must be called from ISR
 * \param tmr Timer value at the moment of falling edge
 */
static inline void ProcessCogEdge(uint16_t tmr)
{
 //calculate stroke period
 hall.cog_period = tmr - hall.tmr_prev;

 //Do additional steps during cranking
 if (!CHECKBIT(flags2, F_ISSYNC))
 {
  if (!CHECKBIT(flags2, F_STRMOD))
  {
   if (hall.cog >= HALL_ON_START_SKIP_COGS)
    SETBIT(flags2, F_STRMOD);
   goto sync_skip;
  }
  else
  {
   if (HALL_SYNCCOND(hall.cog_period_prev) > hall.cog_period)
   {
    SETBIT(flags2, F_ISSYNC);                //synchronized!
    hall.cog = 1;
    goto sync_enter;
   }
   goto sync_skip; //skip all if we can't synchronize
  }
 }

 //Check synchronization
 if (HALL_SYNCCOND(hall.cog_period_prev) > hall.cog_period)
 {
  if (hall.cog != HALL_COGS_NUM+1) //check for error
   SETBIT(flags, F_ERROR);
  hall.cog = 1;
 }
 else if (hall.cog >= HALL_COGS_NUM+1)  //cog must not exceed HALL_COGS_NUM
 {
  SETBIT(flags, F_ERROR);
  hall.cog = 1;                         //This this a good idea to set here cog=1, because if misssync occurs, system can continue to work properly (only indicating error by CE)
 }

 //skip small tooth
 if (hall.cog == HALL_COGS_NUM)
  goto sync_skip;

sync_enter:

 //save period value if it is correct. We need to do it first of all to have fresh stroke_period value
 if (CHECKBIT(flags, F_VHTPER))
 {
  //calculate stroke period
  hall.stroke_period = tmr - hall.measure_start_value;
  WRITEBIT(flags, F_SPSIGN, tmr < hall.measure_start_value); //save sign
  hall.t1oc_s = hall.t1oc, hall.t1oc = 0; //save value and reset counter
 }
 SETBIT(flags, F_VHTPER);
 SETBIT(flags, F_STROKE); //set the stroke-synchronization event
 hall.measure_start_value = tmr;

 uint16_t delay;
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

 //start timer for counting out of advance angle (spark)
 delay = (((uint32_t)hall.advance_angle * hall.stroke_period) / hall.degrees_per_stroke);
#ifdef COOLINGFAN_PWM
 _DISABLE_INTERRUPT();
#endif

 hall.channel_mode_a = hall.cog-1;

 OCR1A = tmr + ((delay < 15) ? 15 : delay) - CALIBRATION_DELAY; //set compare channel, additionally prevent spark missing when advance angle is near to 60�
 TIFR1 = _BV(OCF1A);
 TIMSK1|= _BV(OCIE1A);

 //start timer for countiong out of knock window opening
 if (CHECKBIT(flags, F_USEKNK))
 {
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

sync_skip:
 ++hall.cog;
 hall.tmr_prev = tmr;
 hall.cog_period_prev = hall.cog_period;
}

/**Input capture interrupt of timer 1 */
ISR(TIMER1_CAPT_vect)
{
 ProcessCogEdge(ICR1);
 SETBIT(flags, F_HALLEV); //set event flag
}

/**INT1 handler function (Interrupt from a Hall sensor (external)).
 * See also prototype of this function in camsens.c
 */
void ProcessInterrupt1(void)
{
 ProcessCogEdge(TCNT1);
 SETBIT(flags, F_HALLEV); //set event flag
}

/**INT0 handler function (Interrupt from a Hall sensor (external)).
 * See also prototype of this function in camsens.c
 */
void ProcessInterrupt0(void)
{
 ProcessCogEdge(TCNT1);
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
   knock_start_settings_latching();//start the process of downloading the settings into the HIP9011 (and getting of ADC result for TPIC8101)
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

#endif //CKPS_NPLUS1
