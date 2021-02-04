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

/** \file ckps-cs.c
 * \author Alexey A. Shabelnikov
 * Implementation of crankshaft position sensor's processing (camshaft version).
 */

#if defined(CAM_SYNC) && !defined(ODDFIRE_ALGO)

#include <stdlib.h>
#include "port/avrio.h"
#include "port/interrupt.h"
#include "port/intrinsic.h"
#include "port/port.h"
#include "bitmask.h"
#include "camsens.h"
#include "ckps.h"
#include "ioconfig.h"
#include "injector.h"   //inject_start_inj()
#include "magnitude.h"
#include "tables.h"     //fnptr_t

#include "knock.h"

#ifdef STROBOSCOPE
#define STROBE_PW 31    //!< Strobe pulse width (100uS), value in tics ot timer, 1 tick = 3.2uS
#endif

//PHASED_IGNITION can't be used without PHASE_SENSOR
#if defined(PHASED_IGNITION) && !defined(PHASE_SENSOR)
 #error "You can not use phased ignition without phase sensor. Define PHASE_SENSOR if it is present in the system or do not use phased ignition!"
#endif

/**Maximum number of ignition channels */
#define IGN_CHANNELS_MAX      8

#ifdef SPLIT_ANGLE
/**Offset for splitting of channels*/
#define SPLIT_OFFSET 4
#endif

/** Barrier threshold for detecting of missing teeth
 * e.g. for 60-2 crank wheel, p * 2.5
 *      for 36-1 crank wheel, p * 1.5
 */
#define CKPS_GAP_BARRIER(p) ( ((p) << (ckps.miss_cogs_num==2)) + ((p) >> 1) )

/** number of teeth that will be skipped at the start of cranking before synchronization */
#define CKPS_ON_START_SKIP_COGS      5

//Define values for controlling of outputs
#define IGN_OUTPUTS_INIT_VAL 1        //!< value used for initialization
#define IGN_OUTPUTS_ON_VAL   1        //!< value used to turn on ignition channel
#define IGN_OUTPUTS_OFF_VAL  0        //!< value used to turn off ignition channel

#ifdef DWELL_CONTROL
#define IGNOUTCB_ON_VAL (ckps.ignout_on_val)
#define IGNOUTCB_OFF_VAL (ckps.ignout_off_val)
#else
#define IGNOUTCB_ON_VAL IGN_OUTPUTS_ON_VAL
#define IGNOUTCB_OFF_VAL IGN_OUTPUTS_OFF_VAL
#endif

/**Delay of entering COMPA interrupt and setting required level on the corresponding output.
 * Used for compensation of time for increasing of accuracy
 */
#define COMPA_VECT_DELAY 2

#ifdef FUEL_INJECT
/**Delay of entering COMPB interrupt and setting required level on the corresponding output.
 * Used for compensation of time for increasing of accuracy
 */
#define COMPB_VECT_DELAY 2
#endif

// Flags (see flags variable)
#define F_ERROR     0                 //!< CKP error flag, set in the CKP's interrupt, reset after processing
#define F_VHTPER    1                 //!< used to indicate that measured period is valid (actually measured)
#define F_ISSYNC    2                 //!< indicates that synchronization has been completed (missing teeth found)
#define F_STROKE    3                 //!< flag for synchronization with rotation
#define F_USEKNK    4                 //!< flag which indicates using of knock channel
#define F_PNDSPK    5                 //!< indicates that it is necessary to set compare channel for spark (finish of dwell)
#ifdef DWELL_CONTROL
 #define F_PNDDWL   6                 //!< Indicates that it is necessary to set compare channel for dwell (start of dwell)
#endif
#define F_IGNIEN    7                 //!< Ignition enabled/disabled

//Additional flags (see flags2 variable)
#if defined(PHASED_IGNITION) || (defined(PHASE_SENSOR) && defined(FUEL_INJECT))
 #define F_CAMISS    1                //!< Indicates that system has already obtained event from a cam sensor
#endif
#define F_SPSIGN     2                //!< Sign of the measured stroke period (time between TDCs)
#define F_SINGCH     3                //!< indicates that single ignition channel is used
#ifdef PHASE_SENSOR
#define F_CAMREF     4                //!< Specifies to use camshaft sensor as reference
#endif
#ifdef SPLIT_ANGLE
#define F_PNDSPK1    5                //!< indicates that it is necessary to set compare channel for spark (finish of dwell)
#ifdef DWELL_CONTROL
 #define F_PNDDWL1   6                //!< Indicates that it is necessary to set compare channel for dwell (start of dwell)
#endif
#endif

/** State variables */
typedef struct
{
 uint16_t icr_prev;                   //!< previous value if Input Capture Register
 volatile uint16_t period_curr;       //!< last measured inter-tooth period
 uint16_t period_prev;                //!< previous value of inter-tooth period
 volatile uint16_t cog;               //!< counts teeth starting from missing teeth (2 revolutions), begins from 1
 uint16_t measure_start_value;        //!< remembers the value of the capture register to measure the half-turn
 uint16_t current_angle;              //!< counts out given advance angle during the passage of each tooth
 volatile uint16_t stroke_period;     //!< stores the last measurement of the passage of teeth n
 int16_t  advance_angle;              //!< required adv.angle * ANGLE_MULTIPLIER (требуемый УОЗ * ANGLE_MULTIPLIER)
 volatile int16_t advance_angle_buffered;//!< buffered value of advance angle (to ensure correct latching)
 uint8_t  ignition_cogs;              //!< number of teeth determining the duration of ignition drive pulse
 uint8_t  starting_mode;              //!< state of state machine processing of teeth at the startup
 uint8_t  channel_mode;               //!< determines which channel of the ignition to run at the moment
 volatile uint8_t cogs_btdc;          //!< number of teeth from missing teeth to TDC of the first cylinder
 int8_t   knock_wnd_begin_abs;        //!< begin of the phase selection window of detonation in the teeth of wheel, relatively to TDC
 int8_t   knock_wnd_end_abs;          //!< end of the phase selection window of detonation in the teeth of wheel, relatively to TDC
 volatile uint8_t chan_number;        //!< number of ignition channels
 uint32_t frq_calc_dividend;          //!< divident for calculating RPM
#ifdef DWELL_CONTROL
 volatile uint16_t cr_acc_time;       //!< accumulation time for dwell control (timer's ticks)
 uint8_t  channel_mode_b;             //!< determines which channel of the ignition to start accumulate at the moment
 volatile uint8_t rising_edge_spark;  //!< flag, indicates that rising edge of ignition pulse will be generated at the moment of spark
 volatile uint8_t ignout_on_val;      //!< value used to turn on ignition channel
 volatile uint8_t ignout_off_val;     //!< value used to turn off ignition channel
 volatile uint16_t degrees_per_chan;  //!< number of degrees per one channel (degrees between two spark events)
 volatile uint8_t eq_tail1;           //!< event queue tail (index in a static array)
 volatile uint8_t eq_head1;           //!< event queue head (index), queue is empty if head = tail
#endif
#ifdef HALL_OUTPUT
 int8_t   hop_offset;                 //!< Hall output: start of pulse in teeth of wheel relatively to TDC
 uint8_t  hop_duration;               //!< Hall output: duration of pulse in teeth of wheel
#endif
#ifdef FUEL_INJECT
 int16_t  inj_phase;                  //!< Injection timing: start of pulse in crankshaft degrees relatively BTDC, value * ANGLE_MULTIPLIER
 uint8_t  inj_chidx;                  //!< index of channel to fire
#endif
 volatile uint8_t wheel_cogs_num;     //!< Number of teeth, including missing
 volatile uint8_t wheel_cogs_nump1;   //!< wheel_cogs_num + 1
 volatile uint8_t wheel_cogs_numm1;   //!< wheel_cogs_num - 1
 volatile uint8_t miss_cogs_num;      //!< Count of crank wheel's missing teeth
 volatile uint8_t wheel_last_cog;     //!< Number of last(present) tooth, numeration begins from 1!
 /**Number of teeth before TDC which determines moment of advance angle latching, start of measurements from sensors,
  * latching of settings into HIP9011
  */
 volatile uint8_t  wheel_latch_btdc;
 volatile uint16_t degrees_per_cog;   //!< Number of degrees which corresponds to the 1 tooth
 volatile uint16_t degrees_per_cog_r; //!< Reciprocal of the degrees_per_cog, value * 65536
 volatile uint16_t cogs_per_chan;     //!< Number of teeth per 1 ignition channel (it is fractional number * 256)
 volatile int16_t start_angle;        //!< Precalculated value of the advance angle at 66° (at least) BTDC
#ifdef STROBOSCOPE
 uint8_t strobe;                      //!< Flag indicates that strobe pulse must be output on pending ignition stroke
#endif

 volatile uint8_t TCNT0_H;            //!< For supplementing timer/counter 0 up to 16 bits

 volatile uint8_t t1oc;               //!< Timer 1 overflow counter
 volatile uint8_t t1oc_s;             //!< Contains value of t1oc synchronized with stroke_period value

#ifdef SPLIT_ANGLE
 volatile uint8_t chan_number_split;  //!< number of ignition channels with splitting
 uint8_t  channel_mode1;              //!< determines which channel of the ignition to run at the moment (second group)
 uint8_t  channel_mode_b1;            //!< determines which channel of the ignition to start accumulate at the moment (second group)
 int16_t  advance_angle1;             //!< required adv.angle * ANGLE_MULTIPLIER (требуемый УОЗ * ANGLE_MULTIPLIER)
 volatile int16_t advance_angle_buffered1;//!< buffered value of advance angle (to ensure correct latching)
 volatile uint8_t eq_tail2;           //!< event queue tail (index in a static array)
 volatile uint8_t eq_head2;           //!< event queue head (index), queue is empty if head = tail
#endif
}ckpsstate_t;

/**Precalculated data (reference points) and state data for a single channel plug
 */
typedef struct
{
#ifndef DWELL_CONTROL
 /** Counts out teeth for ignition pulse */
 volatile uint8_t ignition_pulse_cogs;
#endif

 /**Address of callback which will be used for settiong of I/O */
 volatile fnptr_t io_callback1;
#ifdef PHASED_IGNITION
 /**Second callback used only in semi-sequential ignition mode */
 volatile fnptr_t io_callback2;
#endif

#ifdef HALL_OUTPUT
 volatile uint16_t hop_begin_cog;      //!< Hall output: tooth number that corresponds to the beginning of pulse
 volatile uint16_t hop_end_cog;        //!< Hall output: tooth number that corresponds to the end of pulse
#endif

#ifdef FUEL_INJECT
 volatile uint16_t inj_angle;          //!< Injection timing
 volatile uint16_t inj_angle_safe;     //!< Injection timing, safe (synchronized)
 volatile uint8_t inj_skipth;          //!< Number of teeth to skip after setting of COMPB
#endif

 /** Determines number of tooth (relatively to TDC) at which "latching" of data is performed */
 volatile uint16_t cogs_latch;
 /** Determines number of tooth at which measurement of rotation period is performed */
 volatile uint16_t cogs_btdc;
 /** Determines number of tooth at which phase selection window for knock detection is opened */
 volatile uint16_t knock_wnd_begin;
 /** Determines number of tooth at which phase selection window for knock detection is closed */
 volatile uint16_t knock_wnd_end;
}chanstate_t;

ckpsstate_t ckps;                         //!< instance of state variables
chanstate_t chanstate[IGN_CHANNELS_MAX];  //!< instance of array of channel's state variables

// Arrange flags in the free I/O register
//  note: may be not effective on other MCUs or even cause bugs! Be aware.
#define flags  GPIOR0                 //!< ATmega644 has one general purpose I/O register and we use it for first flags variable
#define flags2 TWBR                   //!< Second flags variable in I/O register

/**Accessor macro for RPM dividents table*/
#define FRQ_CALC_DIVIDEND(channum) PGM_GET_DWORD(&frq_calc_dividend[channum])
/**Table srtores dividends for calculating of RPM */
PGM_DECLARE(uint32_t frq_calc_dividend[1+IGN_CHANNELS_MAX]) =
 //     1          2          3          4         5         6         7         8
 {0, 37500000L, 18750000L, 12500000L, 9375000L, 7500000L, 6250000L, 5357143L, 4687500L};

#ifdef DWELL_CONTROL

#define DWL_DEAD_TIME 156        //!< 500uS dead time

/**Maximum queue size for ignition events, MUST BE power of two (2,4,8 etc) */
#define IGN_QUEUE_SIZE 4

#define QID_DWELL  0             //!< start dwell
#define QID_SPARK  1             //!< finish dwell (spark)
#define QID_STROBE 2             //!< finish strobe pulse

/**Describes event queue entry for T1 COMPA channel*/
typedef struct
{
 uint16_t end_time;              //!< End time of event in ticks of free running timer 1
 uint8_t  id;                    //!< pending action Id
}ign_queue_t;

/**Event queue for scheduling of ignition spark events (T1 COMPA) */
ign_queue_t ign_eq1[IGN_QUEUE_SIZE];

#ifdef SPLIT_ANGLE
/**Event queue for scheduling of ignition spark events (T3 COMPA) */
ign_queue_t ign_eq2[IGN_QUEUE_SIZE];
#endif

/** Reset specified queue (makes it empty) */
#define QUEUE_RESET(q)  ckps.eq_tail##q = ckps.eq_head##q = 0;

/** Add event into the queue (add to head)
 * r Timer's register (TCNT1 or ICR1)
 * time Time in tics of timer 1 after which event should fire
 * aid ID of pending action, which shold be performed
 */
#define QUEUE_ADD(q, r, time, aid) \
    ign_eq##q[ckps.eq_head##q].end_time = (r) + (time); \
    ign_eq##q[ckps.eq_head##q].id = (aid); \
    ckps.eq_head##q = (ckps.eq_head##q + 1) & (IGN_QUEUE_SIZE-1);

/** Remove event from the queue (remove from tail)*/
#define QUEUE_REMOVE(q) ckps.eq_tail##q = (ckps.eq_tail##q + 1) & (IGN_QUEUE_SIZE-1)

/** Get tail item value from queue */
#define QUEUE_TAIL(q) ign_eq##q[ckps.eq_tail##q]

/** Test is queue empty */
#define QUEUE_IS_EMPTY(q) (ckps.eq_head##q==ckps.eq_tail##q)

#endif //DWELL_CONTROL

/**Set T1 COMPA channel of timer
 * r Timer's register (TCNT1 or ICR1)
 * v Time in tics of timer 1 after which event should fire
 */
#define SET_T1COMPA(r, v) \
     OCR1A = (r) + (v); \
     TIFR1 = _BV(OCF1A); \
     SETBIT(TIMSK1, OCIE1A);

#ifdef FUEL_INJECT
/**Set T1 COMPB channel of timer
 * r Timer's register (TCNT1 or ICR1)
 * v Time in tics of timer 1 after which event should fire
 */
#define SET_T1COMPB(r, v) \
     OCR1B = (r) + (v); \
     TIFR1 = _BV(OCF1B); \
     SETBIT(TIMSK1, OCIE1B);
#endif

#ifdef SPLIT_ANGLE
/**Set T3 COMPA channel of timer. Note: we rely that timers 1 and 3 are synchronized!
 * r Timer's register (TCNT3, TCNT1 or ICR1).
 * v Time in tics of timer 1 after which event should fire
 */
#define SET_T3COMPA(r, v) \
     OCR3A = (r) + (v); \
     TIFR3 = _BV(OCF3A); \
     SETBIT(TIMSK3, OCIE3A);
#endif

void ckps_init_state_variables(void)
{
#ifndef DWELL_CONTROL
 uint8_t i;
 _BEGIN_ATOMIC_BLOCK();
 //Set to value that means to do nothing with outputs
 for(i = 0; i < IGN_CHANNELS_MAX; ++i)
  chanstate[i].ignition_pulse_cogs = 255;
#else
 _BEGIN_ATOMIC_BLOCK();
 ckps.cr_acc_time = 0;
 ckps.channel_mode_b = 0; //prevent missing spark on first ignition event
#ifdef SPLIT_ANGLE
 ckps.channel_mode_b1 = 0; //prevent missing spark on first ignition event
#endif
 CLEARBIT(flags, F_PNDDWL);
#ifdef SPLIT_ANGLE
 CLEARBIT(flags2, F_PNDDWL1);
#endif
 QUEUE_RESET(1);
#ifdef SPLIT_ANGLE
 QUEUE_RESET(2);
#endif
#endif

 ckps.cog = 0;
 ckps.stroke_period = 0xFFFF;
 ckps.advance_angle = ckps.advance_angle_buffered = 0;
#ifdef SPLIT_ANGLE
 ckps.advance_angle1 = ckps.advance_angle_buffered1 = 0;
#endif
 ckps.starting_mode = 0;
 ckps.channel_mode = 0;
#ifdef SPLIT_ANGLE
 ckps.channel_mode1 = 0;
#endif
#if defined(PHASED_IGNITION) || (defined(PHASE_SENSOR) && defined(FUEL_INJECT))
 CLEARBIT(flags2, F_CAMISS);
#endif
 CLEARBIT(flags, F_PNDSPK);
#ifdef SPLIT_ANGLE
 CLEARBIT(flags2, F_PNDSPK1);
#endif
 CLEARBIT(flags, F_STROKE);
 CLEARBIT(flags, F_ISSYNC);
 SETBIT(flags, F_IGNIEN);
 CLEARBIT(flags2, F_SPSIGN);
#ifdef FUEL_INJECT
 ckps.inj_chidx = 0;
 {
 uint8_t i;
 for(i = 0; i < IGN_CHANNELS_MAX; ++i)
  chanstate[i].inj_skipth = 0;
 }
#endif

 TIMSK1|=_BV(TOIE1);                  //enable Timer 1 overflow interrupt. Used for correct calculation of very low RPM

#ifdef STROBOSCOPE
 ckps.strobe = 0;
#endif

 ckps.t1oc = 0;                       //reset overflow counter
 ckps.t1oc_s = 255;                   //RPM is very low
 _END_ATOMIC_BLOCK();
}

void ckps_init_state(void)
{
 _BEGIN_ATOMIC_BLOCK();
 ckps_init_state_variables();
 CLEARBIT(flags, F_ERROR);

 //Compare channels do not connected to lines of ports (normal port mode)
 TCCR1A = 0;

 //Noise reduction, rising edge of capture, clock = 312.5 kHz
 TCCR1B = _BV(ICNC1)|_BV(ICES1)|_BV(CS11)|_BV(CS10);
 TCCR0B = _BV(CS01)|_BV(CS00); //clock = 312.5 kHz

 //enable input capture interrupt of timer 1
 TIMSK1|= _BV(ICIE1);

#ifdef SPLIT_ANGLE
 TCCR3A = 0; //Normal port operation, OC3A/OC3B disconnected.

 //note: it is also started in pwm2.c module
 TCCR3B = _BV(CS31) | _BV(CS30); //start timer, clock  = 312.5 kHz

 CLEARBIT(TIMSK3, OCIE3A); //compare interrupt A is disabled
#endif

 _END_ATOMIC_BLOCK();
}

void ckps_set_advance_angle(int16_t angle)
{
 _BEGIN_ATOMIC_BLOCK();
 ckps.advance_angle_buffered = angle;
 _END_ATOMIC_BLOCK();
}

#ifdef SPLIT_ANGLE
void ckps_set_advance_angle1(int16_t angle)
{
 _BEGIN_ATOMIC_BLOCK();
 ckps.advance_angle_buffered1 = angle;
 _END_ATOMIC_BLOCK();
}
#endif

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

//Instantaneous frequency calculation of crankshaft rotation from the measured period between the engine strokes
//(for example for 4-cylinder, 4-stroke it is 180°)
//Period measured in the discretes of timer (one discrete = 3.2us), one minute = 60 seconds, one second has 1,000,000 us.
uint16_t ckps_calculate_instant_freq(void)
{
 uint16_t period; uint8_t ovfcnt, sign;
 //ensure atomic acces to variable
 _DISABLE_INTERRUPT();
 period = ckps.stroke_period;        //stroke period
 ovfcnt = ckps.t1oc_s;               //number of timer overflows
 sign = CHECKBIT(flags2, F_SPSIGN);  //sign of stroke period
 _ENABLE_INTERRUPT();

 if (0xFF == ovfcnt)
  return 0; //engine is stopped

 //We know period and number of timer overflows, so we can calculate correct value of RPM even if RPM is very low
 if (sign && ovfcnt > 0)
  return ckps.frq_calc_dividend / ((((int32_t)ovfcnt) * 65536) - (65536-period));
 else
  return ckps.frq_calc_dividend / ((((int32_t)ovfcnt) * 65536) + period);
}

uint16_t ckps_get_stroke_period(void)
{
 uint16_t period;
 _DISABLE_INTERRUPT();
 period = ckps.stroke_period;        //stroke period
 _ENABLE_INTERRUPT();
 return period;
}

void ckps_set_edge_type(uint8_t edge_type)
{
 _BEGIN_ATOMIC_BLOCK();
 if (edge_type)
  TCCR1B|= _BV(ICES1);
 else
  TCCR1B&=~_BV(ICES1);
 _END_ATOMIC_BLOCK();
}

/**
 * Ensures that tooth number will be in the allowed range.
 * Tooth number should not be greater than cogs number * 2 or less than zero
 */
static uint16_t _normalize_tn(int16_t i_tn)
{
 if (i_tn > (int16_t)ckps.wheel_cogs_num)
  return i_tn - (int16_t)ckps.wheel_cogs_num;
 if (i_tn <= 0)
  return i_tn + ckps.wheel_cogs_num;
 return i_tn;
}

#ifdef FUEL_INJECT
/** Ensures that angle will be in the allowed range (0...720)
 * \param angle Angle to be normalized, may be negative
 * \return Normalized and correct value of angle
 */
static uint16_t _normalize_angle(int16_t angle)
{
 if (angle > ANGLE_MAGNITUDE(720))
  return angle - ANGLE_MAGNITUDE(720);
 if (angle < 0)
  return ANGLE_MAGNITUDE(720) + angle;
 return angle;
}

/** Synchronize injection angle values */
static void sync_inj_angle(void)
{
 uint8_t i;
 _BEGIN_ATOMIC_BLOCK();
 for(i = 0; i < ckps.chan_number; ++i)
  chanstate[i].inj_angle_safe = chanstate[i].inj_angle;
 _END_ATOMIC_BLOCK();
}
#endif

void ckps_set_cogs_btdc(uint8_t cogs_btdc)
{
 uint8_t _t, i;
 _t=_SAVE_INTERRUPT();
 _DISABLE_INTERRUPT();
 for(i = 0; i < ckps.chan_number; ++i)
 {
  uint16_t tdc = (((uint16_t)cogs_btdc) + ((i * ckps.cogs_per_chan) >> 8));
  chanstate[i].cogs_btdc = _normalize_tn(tdc);
#ifdef SPLIT_ANGLE
  chanstate[i+SPLIT_OFFSET].cogs_btdc = _normalize_tn(tdc);
#endif
  chanstate[i].cogs_latch = _normalize_tn(tdc - ckps.wheel_latch_btdc);
  chanstate[i].knock_wnd_begin = _normalize_tn(tdc + ckps.knock_wnd_begin_abs);
  chanstate[i].knock_wnd_end = _normalize_tn(tdc + ckps.knock_wnd_end_abs);
#ifdef HALL_OUTPUT
  //update Hall output pulse parameters because they depend on ckps.cogs_btdc parameter
  chanstate[i].hop_begin_cog = _normalize_tn(tdc - ckps.hop_offset);
  chanstate[i].hop_end_cog = _normalize_tn(chanstate[i].hop_begin_cog + ckps.hop_duration);
#endif
#ifdef FUEL_INJECT
  uint16_t angle = (tdc * ckps.degrees_per_cog) + ckps.inj_phase;
  if (angle > ANGLE_MAGNITUDE(720))
   angle-= ANGLE_MAGNITUDE(720);    //phase is periodical
  chanstate[i].inj_angle = angle;
#endif
 }
 ckps.cogs_btdc = cogs_btdc;
 _RESTORE_INTERRUPT(_t);
}

#ifndef DWELL_CONTROL
void ckps_set_ignition_cogs(uint8_t cogs)
{
 _BEGIN_ATOMIC_BLOCK();
 ckps.ignition_cogs = cogs;
 _END_ATOMIC_BLOCK();
}
#else
void ckps_set_acc_time(uint16_t i_acc_time)
{
 _BEGIN_ATOMIC_BLOCK();
 ckps.cr_acc_time = i_acc_time;
 _END_ATOMIC_BLOCK();
}
void ckps_set_rising_spark(uint8_t rising_edge)
{
 _BEGIN_ATOMIC_BLOCK();
 ckps.rising_edge_spark = rising_edge;
 if (rising_edge) { //spark on rising edge
  ckps.ignout_on_val = IGN_OUTPUTS_OFF_VAL;
  ckps.ignout_off_val = IGN_OUTPUTS_ON_VAL;
 }
 else { //spark on falling edge
  ckps.ignout_on_val = IGN_OUTPUTS_ON_VAL;
  ckps.ignout_off_val = IGN_OUTPUTS_OFF_VAL;
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
 static uint8_t prev_cog = 0;
 uint8_t value = ckps.cog;
 if (prev_cog != value)
 {
  prev_cog = value;
  return 1;
 }
 return 0;
}

/** Get value of I/O callback by index. This function is necessary for supporting of 7,8 ign. channels
 * \param index Index of callback */
static inline fnptr_t get_callback_ign(uint8_t index)
{
 return (index < IOP_ECF) ? IOCFG_CB(index) : IOCFG_CB(index + IOP_IGNPLG_OFF);
}

/** Tune channels for single output mode
 */
static void set_channels_sc(void)
{
 uint8_t i = 0;
 fnptr_t value = IOCFG_CB(0); //use only 1-st channel
 for(; i < ckps.chan_number; ++i)
 {
  _BEGIN_ATOMIC_BLOCK();
  chanstate[i].io_callback1 = value;
  ((iocfg_pfn_set)get_callback_ign(i))(IGN_OUTPUTS_ON_VAL); //turn off other channels
  _END_ATOMIC_BLOCK();
 }
#ifdef SPLIT_ANGLE
 value = get_callback_ign(SPLIT_OFFSET); //use only 1-st channel
 for(i = 0; i < ckps.chan_number; ++i)
 {
  _BEGIN_ATOMIC_BLOCK();
  chanstate[i + SPLIT_OFFSET].io_callback1 = value;
  ((iocfg_pfn_set)get_callback_ign(i))(IGN_OUTPUTS_ON_VAL); //turn off other channels
  _END_ATOMIC_BLOCK();
 }
#endif
}

#ifndef PHASED_IGNITION
/**Tune channels' I/O for semi-sequential ignition mode (wasted spark) */
static void set_channels_ss(void)
{
 uint8_t _t, i = 0, chan = ckps.chan_number / 2;
 for(; i < chan; ++i)
 {
  fnptr_t value = IOCFG_CB(i);
  _t=_SAVE_INTERRUPT();
  _DISABLE_INTERRUPT();
  chanstate[i].io_callback1 = value;
  chanstate[i + chan].io_callback1 = value;
  _RESTORE_INTERRUPT(_t);
 }
#ifdef SPLIT_ANGLE
 for(i = 0; i < chan; ++i)
 {
  fnptr_t value = get_callback_ign(i + SPLIT_OFFSET);
  _t=_SAVE_INTERRUPT();
  _DISABLE_INTERRUPT();
  chanstate[i + SPLIT_OFFSET].io_callback1 = value;
  chanstate[i + chan + SPLIT_OFFSET].io_callback1 = value;
  _RESTORE_INTERRUPT(_t);
 }
#endif
}

#else
/**Tune channels' I/O for full sequential ignition mode */
static void set_channels_fs(uint8_t fs_mode)
{
 uint8_t _t, i = 0, ch2 = fs_mode ? 0 : ckps.chan_number / 2, iss;
 for(; i < ckps.chan_number; ++i)
 {
  iss = (i + ch2);
  if (iss >= ckps.chan_number)
   iss-=ckps.chan_number;

  _t=_SAVE_INTERRUPT();
  _DISABLE_INTERRUPT();
  chanstate[i].io_callback1 = get_callback_ign(i);
  chanstate[i].io_callback2 = get_callback_ign(iss);
  _RESTORE_INTERRUPT(_t);
 }
#ifdef SPLIT_ANGLE
 for(i = 0; i < ckps.chan_number; ++i)
 {
  iss = (i + ch2);
  if (iss >= ckps.chan_number)
   iss-=ckps.chan_number;

  _t=_SAVE_INTERRUPT();
  _DISABLE_INTERRUPT();
  chanstate[i+SPLIT_OFFSET].io_callback1 = get_callback_ign(i+SPLIT_OFFSET);
  chanstate[i+SPLIT_OFFSET].io_callback2 = get_callback_ign(iss+SPLIT_OFFSET);
  _RESTORE_INTERRUPT(_t);
 }
#endif
}
#endif

void ckps_set_cyl_number(uint8_t i_cyl_number)
{
 uint8_t i = ckps.chan_number;
 _BEGIN_ATOMIC_BLOCK();
 ckps.chan_number = i_cyl_number;
#ifdef SPLIT_ANGLE
 ckps.chan_number_split = i_cyl_number + SPLIT_OFFSET;
#endif
 _END_ATOMIC_BLOCK();

 ckps.frq_calc_dividend = FRQ_CALC_DIVIDEND(i_cyl_number);

 if (CHECKBIT(flags2, F_SINGCH))
 { //single channel
  set_channels_sc();
 }
 else
 {
  //We have to retune I/O configuration after changing of cylinder number
#ifndef PHASED_IGNITION
  set_channels_ss();  // Tune for semi-sequential mode
#else //phased ignition
  //Tune for full sequential mode
  set_channels_fs(1);
#endif
 }

 //unused channels must be turned off
 if (i > i_cyl_number)
 {
#ifdef SPLIT_ANGLE
  for(i = i_cyl_number; i < SPLIT_OFFSET; ++i) //turn off channels in primiry group
   ((iocfg_pfn_set)get_callback_ign(i))(IGN_OUTPUTS_ON_VAL);
  for(i = i_cyl_number + SPLIT_OFFSET; i < IGN_CHANNELS_MAX; ++i) //turn off channels on secondary group
   ((iocfg_pfn_set)get_callback_ign(i))(IGN_OUTPUTS_ON_VAL);
#else //regular mode (single channel per cylinder)
  for(i = i_cyl_number; i < IGN_CHANNELS_MAX; ++i)
   ((iocfg_pfn_set)get_callback_ign(i))(IGN_OUTPUTS_ON_VAL);
#endif
 }

 //TODO: calculations previosly made by ckps_set_cogs_btdc()|ckps_set_knock_window()|ckps_set_hall_pulse() becomes invalid!
 //So, ckps_set_cogs_btdc() must be called again. Do it here or in place where this function called.
}

void ckps_set_knock_window(int16_t begin, int16_t end)
{
 uint8_t _t, i;
 //translate from degrees to teeth (переводим из градусов в зубья)
 ckps.knock_wnd_begin_abs = begin / ((int16_t)ckps.degrees_per_cog);
 ckps.knock_wnd_end_abs = end / ((int16_t)ckps.degrees_per_cog);

 _t=_SAVE_INTERRUPT();
 _DISABLE_INTERRUPT();
 for(i = 0; i < ckps.chan_number; ++i)
 {
  uint16_t tdc = (((uint16_t)ckps.cogs_btdc) + ((i * ckps.cogs_per_chan) >> 8));
  chanstate[i].knock_wnd_begin = _normalize_tn(tdc + ckps.knock_wnd_begin_abs);
  chanstate[i].knock_wnd_end = _normalize_tn(tdc + ckps.knock_wnd_end_abs);
 }
 _RESTORE_INTERRUPT(_t);
}

void ckps_enable_ignition(uint8_t i_cutoff)
{
 WRITEBIT(flags, F_IGNIEN, i_cutoff);
}

void ckps_set_merge_outs(uint8_t i_merge)
{
 WRITEBIT(flags2, F_SINGCH, i_merge);
 if (CHECKBIT(flags2, F_SINGCH))
 { //single channel
  set_channels_sc();
 }
 else
 {
#ifndef PHASED_IGNITION
  set_channels_ss();  // Tune for semi-sequential mode
#else //phased ignition
  set_channels_fs(1); //Tune for full sequential mode if cam sensor works, otherwise tune for semi-sequential mode
#endif
 }
}

#ifdef HALL_OUTPUT
void ckps_set_hall_pulse(int16_t i_offset, uint16_t i_duration)
{
 uint8_t _t, i;
 //save values because we will access them from other function
 ckps.hop_offset = i_offset / ((int16_t)ckps.degrees_per_cog);
 ckps.hop_duration = i_duration / ((int16_t)ckps.degrees_per_cog);

 _t=_SAVE_INTERRUPT();
 _DISABLE_INTERRUPT();
 for(i = 0; i < ckps.chan_number; ++i)
 {
  uint16_t tdc = (((uint16_t)ckps.cogs_btdc) + ((i * ckps.cogs_per_chan) >> 8));
  chanstate[i].hop_begin_cog = _normalize_tn(tdc - ckps.hop_offset);
  chanstate[i].hop_end_cog = _normalize_tn(chanstate[i].hop_begin_cog + ckps.hop_duration);
 }
 _RESTORE_INTERRUPT(_t);
}
#endif

void ckps_set_cogs_num(uint8_t norm_num, uint8_t miss_num)
{
 div_t dr; uint8_t _t;
#ifdef PHASE_SENSOR
 uint16_t err_thrd = norm_num + (norm_num >> 3); //+ 12.5%
#endif
 uint16_t cogs_per_chan, degrees_per_cog, degrees_per_cog_r;

 //precalculate number of cogs per 1 ignition channel, it is fractional number multiplied by 256
 cogs_per_chan = (((uint32_t)norm_num) << 8) / ckps.chan_number;

 //precalculate value of degrees per 1 cog, it is fractional number multiplied by ANGLE_MULTIPLIER
 degrees_per_cog = (((((uint32_t)720) << 8) / norm_num) * ANGLE_MULTIPLIER) >> 8;

 //precalculate value of 1 / degrees_per_cog (reciprocal), result value multiplied by ~65536
 degrees_per_cog_r = (1*65535) / degrees_per_cog;

#ifdef DWELL_CONTROL
 //precalculate value of degrees per channel, value * ANGLE_MULTIPLIER
 uint16_t degrees_per_chan = ((uint32_t)cogs_per_chan * degrees_per_cog) >> 8;
#endif

 //precalculate value and round it always to the upper bound,
 //e.g. for 60-2 crank wheel result = 11 (66°), for 36-1 crank wheel result = 7 (70°)
 dr = div(ANGLE_MAGNITUDE(66), degrees_per_cog);

 _t=_SAVE_INTERRUPT();
 _DISABLE_INTERRUPT();
 //calculate number of last cog
 ckps.wheel_last_cog = norm_num - miss_num;
 //set number of teeth (normal and missing)
 ckps.wheel_cogs_num = norm_num;
 ckps.wheel_cogs_nump1 = norm_num + 1;
 ckps.wheel_cogs_numm1 = norm_num - 1;
 ckps.miss_cogs_num = miss_num;
 //set other precalculated values
 ckps.wheel_latch_btdc = dr.quot + (dr.rem > 0);
 ckps.degrees_per_cog = degrees_per_cog;
 ckps.degrees_per_cog_r = degrees_per_cog_r; //reciprocal of the degrees_per_cog
#ifdef DWELL_CONTROL
 ckps.degrees_per_chan = degrees_per_chan;
#endif
 ckps.cogs_per_chan = cogs_per_chan;
 ckps.start_angle = ckps.degrees_per_cog * ckps.wheel_latch_btdc;
#ifdef PHASE_SENSOR
 cams_set_error_threshold(err_thrd);
#endif
 _RESTORE_INTERRUPT(_t);
}

#ifdef FUEL_INJECT
void ckps_set_inj_timing(int16_t phase, uint16_t pw, uint8_t mode)
{
 uint8_t _t, i;
 //TODO: We can do some optimization in the future - set timing only if it is not equal to current (already set one)

 phase = ANGLE_MAGNITUDE(720.0) - phase;

 //Apply selected injection pulse option: begin of squirt, middle of squirt or end of squirt
 if (mode > INJANGLESPEC_BEGIN)
 {
  //convert delay to angle (value * ANGLE_MULTIPLIER). TODO: how to escape from slow division?
  uint16_t pw_angle = (((uint32_t)pw) * ckps.degrees_per_cog) / ckps.period_curr;
  if (mode == INJANGLESPEC_MIDDLE)
   pw_angle>>= 1;
  //apply, rotate angle if need
  phase-=pw_angle;
  if (phase < 0)
   phase+=ANGLE_MAGNITUDE(720.0);
 }
 //---------------------------------------------------------

 ckps.inj_phase = phase;

 for(i = 0; i < ckps.chan_number; ++i)
 {
  uint16_t tdc = (((uint16_t)ckps.cogs_btdc) + ((i * ckps.cogs_per_chan) >> 8));
  uint16_t angle = (tdc * ckps.degrees_per_cog) +  ckps.inj_phase;
  if (angle > ANGLE_MAGNITUDE(720))
   angle-= ANGLE_MAGNITUDE(720);    //phase is periodical
 _t=_SAVE_INTERRUPT();
 _DISABLE_INTERRUPT();
  chanstate[i].inj_angle = angle;
 _RESTORE_INTERRUPT(_t);
 }

}
#endif

#ifdef PHASE_SENSOR
void ckps_use_cam_ref_s(uint8_t i_camref)
{
 WRITEBIT(flags2, F_CAMREF, i_camref);
}
#endif

/** Turn OFF specified ignition channel
 * \param i_channel number of ignition channel to turn off
 */
static inline void turn_off_ignition_channel(uint8_t i_channel)
{
 if (!CHECKBIT(flags, F_IGNIEN))
  return; //ignition disabled
 //Completion of igniter's ignition drive pulse, transfer line of port into a low level - makes 
 //the igniter go to the regime of energy accumulation
 ((iocfg_pfn_set)chanstate[i_channel].io_callback1)(IGNOUTCB_OFF_VAL);
#ifdef PHASED_IGNITION
 ((iocfg_pfn_set)chanstate[i_channel].io_callback2)(IGNOUTCB_OFF_VAL);
#endif
}

/**Interrupt handler for Compare/Match channel A of timer T1. Used for spark output, strobe and dwell time control
 */
ISR(TIMER1_COMPA_vect)
{
 TIMSK1&= ~_BV(OCIE1A); //disable this interrupt

#ifdef DWELL_CONTROL
 switch(QUEUE_TAIL(1).id) //what exactly happen?
 {
  case QID_DWELL: //start accumulation
   //line of port is high level, now we set it into a low level
   turn_off_ignition_channel(ckps.channel_mode_b);
   break;

  case QID_SPARK:
  {
   //line of port in the low level, now set it into a high level - makes the transistor to close and coil to stop 
   //the accumulation of energy (spark)
   ((iocfg_pfn_set)chanstate[ckps.channel_mode].io_callback1)(IGNOUTCB_ON_VAL);
#ifdef PHASED_IGNITION
   ((iocfg_pfn_set)chanstate[ckps.channel_mode].io_callback2)(IGNOUTCB_ON_VAL);
#endif

#ifdef STROBOSCOPE
   if (1==ckps.strobe)
   {
    IOCFG_SET(IOP_STROBE, 1);  //start pulse
    if (QUEUE_IS_EMPTY(1))
    {
     SET_T1COMPA(TCNT1, STROBE_PW); //strobe pulse is 100uS by default
    }
    QUEUE_ADD(1, TCNT1, STROBE_PW, QID_STROBE);
    ckps.strobe = 0;           //and reset flag
   }
#endif
   int32_t acc_delay = (((uint32_t)ckps.period_curr) * ckps.cogs_per_chan) >> 8;

   if (ckps.rising_edge_spark)
   { //CDI mode
    ckps.channel_mode_b = ckps.channel_mode;
    uint16_t acc_time = (ckps.cr_acc_time > acc_delay-DWL_DEAD_TIME) ? ((uint16_t)acc_delay-DWL_DEAD_TIME) : ckps.cr_acc_time;
    if (QUEUE_IS_EMPTY(1))
    {
     SET_T1COMPA(TCNT1, acc_time);
    }
    QUEUE_ADD(1, TCNT1, acc_time, QID_DWELL);
    CLEARBIT(flags, F_PNDDWL);
   }
   else
   { //IDI mode
    acc_delay-= ckps.cr_acc_time;    //apply dwell time
    if (acc_delay < DWL_DEAD_TIME)
     acc_delay = DWL_DEAD_TIME;

    ckps.channel_mode_b = (ckps.channel_mode < ckps.chan_number-1) ? ckps.channel_mode + 1 : 0;
    SETBIT(flags, F_PNDDWL);

    //if less than 1 teeth remains to the accumulation beginning we have to program compare channel
    if (acc_delay < (ckps.period_curr << 1))
    {
     if (QUEUE_IS_EMPTY(1))
     {
      SET_T1COMPA(TCNT1, (uint16_t)acc_delay);
     }
     QUEUE_ADD(1, TCNT1, (uint16_t)acc_delay, QID_DWELL);
     CLEARBIT(flags, F_PNDDWL);
    }
   }
   break;
  }
#ifdef STROBOSCOPE
  case QID_STROBE:
   IOCFG_SET(IOP_STROBE, 0);  //end pulse
   break;
#endif
 }

 //Remove already processed event from queue. After that, is queue is not empty, then start
 //next event in chain. Also, prevent effects when event already expired.
 QUEUE_REMOVE(1);
 if (!QUEUE_IS_EMPTY(1))
 {
  uint16_t t = (QUEUE_TAIL(1).end_time-(uint16_t)2) - TCNT1;
  if (t > 65520)             //end_time < TCNT1, so, it is expired (forbidden range is 65520...65535)
   t = 2;
  SET_T1COMPA(TCNT1, t);
 }

#else //just use trigger whell teeth instead of dwell control

#ifdef STROBOSCOPE
 if (1==ckps.strobe)
 {
  IOCFG_SET(IOP_STROBE, 1);      //start pulse
  ++ckps.strobe;                 //and set flag to next state
  SET_T1COMPA(TCNT1, STROBE_PW); //We will generate 100uS pulse, pulse will be ended in the next interrupt
 }
 else if (2==ckps.strobe)
 {
  IOCFG_SET(IOP_STROBE, 0);      //end pulse
  ckps.strobe = 0;               //and reset flag
  return;
 }
#endif

 //line of port in the low level, now set it into a high level - makes the igniter to stop 
 //the accumulation of energy and close the transistor (spark)
 ((iocfg_pfn_set)chanstate[ckps.channel_mode].io_callback1)(IGNOUTCB_ON_VAL);
#ifdef PHASED_IGNITION
 ((iocfg_pfn_set)chanstate[ckps.channel_mode].io_callback2)(IGNOUTCB_ON_VAL);
#endif

 chanstate[ckps.channel_mode].ignition_pulse_cogs = 0; //start counting the duration of pulse in the teeth
#endif
}

#ifdef SPLIT_ANGLE
/** Timer 3 compare interrupt A - used for second ignition channels (angle splitting for rotary engines)*/
ISR(TIMER3_COMPA_vect)
{
 TIMSK3&= ~_BV(OCIE3A); //disable this interrupt

#ifdef DWELL_CONTROL
 switch(QUEUE_TAIL(2).id) //what exactly happen?
 {
  case QID_DWELL: //start accumulation
   //line of port is high level, now we set it into a low level
   turn_off_ignition_channel(ckps.channel_mode_b1);
   break;

  case QID_SPARK:
  {
   //line of port in the low level, now set it into a high level - makes the transistor to close and coil to stop 
   //the accumulation of energy (spark)
   ((iocfg_pfn_set)chanstate[ckps.channel_mode1].io_callback1)(IGNOUTCB_ON_VAL);
#ifdef PHASED_IGNITION
   ((iocfg_pfn_set)chanstate[ckps.channel_mode1].io_callback2)(IGNOUTCB_ON_VAL);
#endif

   int32_t acc_delay = (((uint32_t)ckps.period_curr) * ckps.cogs_per_chan) >> 8;

   if (ckps.rising_edge_spark)
   { //CDI mode
    ckps.channel_mode_b1 = ckps.channel_mode1;
    uint16_t acc_time = (ckps.cr_acc_time > acc_delay-DWL_DEAD_TIME) ? ((uint16_t)acc_delay-DWL_DEAD_TIME) : ckps.cr_acc_time;
    if (QUEUE_IS_EMPTY(2))
    {
     SET_T3COMPA(TCNT3, acc_time);
    }
    QUEUE_ADD(2, TCNT1, acc_time, QID_DWELL);
    CLEARBIT(flags2, F_PNDDWL1);
   }
   else
   { //IDI mode
    acc_delay-= ckps.cr_acc_time;    //apply dwell time
    if (acc_delay < DWL_DEAD_TIME)
     acc_delay = DWL_DEAD_TIME;

    ckps.channel_mode_b1 = (ckps.channel_mode1 < ckps.chan_number_split-1) ? ckps.channel_mode1 + 1 : SPLIT_OFFSET;
    SETBIT(flags2, F_PNDDWL1);

    //if less than 1 teeth remains to the accumulation beginning we have to program compare channel
    if (acc_delay < (ckps.period_curr << 1))
    {
     if (QUEUE_IS_EMPTY(2))
     {
      SET_T3COMPA(TCNT3, (uint16_t)acc_delay);
     }
     QUEUE_ADD(2, TCNT1, (uint16_t)acc_delay, QID_DWELL);
     CLEARBIT(flags2, F_PNDDWL1);
    }
   }
   break;
  }
 }

 //Remove already processed event from queue. After that, if queue is not empty, then start
 //next event in chain. Also, prevent effects when event already expired.
 QUEUE_REMOVE(2);
 if (!QUEUE_IS_EMPTY(2))
 {
  uint16_t t = (QUEUE_TAIL(2).end_time-(uint16_t)2) - TCNT1;
  if (t > 65520)             //end_time < TCNT3, so, it is expired (forbidden range is 65520...65535)
   t = 2;
  SET_T3COMPA(TCNT3, t);
 }

#else //just use trigger wheel teeth instead of dwell control

 //line of port in the low level, now set it into a high level - makes the igniter to stop 
 //the accumulation of energy and close the transistor (spark)
 ((iocfg_pfn_set)chanstate[ckps.channel_mode1].io_callback1)(IGNOUTCB_ON_VAL);
#ifdef PHASED_IGNITION
 ((iocfg_pfn_set)chanstate[ckps.channel_mode1].io_callback2)(IGNOUTCB_ON_VAL);
#endif

 chanstate[ckps.channel_mode1].ignition_pulse_cogs = 0; //start counting the duration of pulse in the teeth
#endif

}
#endif

#ifdef FUEL_INJECT
/**Interrupt handler for Compare/Match channel B of timer T1. Used for injection timing
 */
ISR(TIMER1_COMPB_vect)
{
 TIMSK1&= ~_BV(OCIE1B);            //disable interrupt
 inject_start_inj(ckps.inj_chidx); //start fuel injection
}
#endif

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
 ckps.TCNT0_H = _AB(value, 1);
 SETBIT(TIMSK0, OCIE0A);
}

/**Helpful function, used at the startup of engine
 * \return 1 when synchronization is finished, othrewise 0
 */
static uint8_t sync_at_startup(void)
{
 switch(ckps.starting_mode)
 {
  case 0: //skip certain number of teeth
   CLEARBIT(flags, F_VHTPER);
   if (ckps.cog >= CKPS_ON_START_SKIP_COGS)
   {
#ifdef PHASE_SENSOR
    if (CHECKBIT(flags2, F_CAMREF))
     ckps.starting_mode = 1; //switch to this mode if cam reference was enabled
    else
#endif
    ckps.starting_mode = 2; //missing tooth wheel
    //note: for full sequential injection (even for odd cyl. number engines) we don't need to wait for cam sensor pulse, because we start with simultateous injection mode.
   }
   break;

#ifdef PHASE_SENSOR
  case 1: //we fall into this state only if cam sensor was selected as reference
   cams_detect_edge();
   if (cams_is_event_r())
   {
#ifdef PHASED_IGNITION
    if (CHECKBIT(flags2, F_SINGCH))
     set_channels_sc(); //single channel mode
    else
     set_channels_fs(1);     //set full sequential mode
    SETBIT(flags2, F_CAMISS);
#endif
#ifdef FUEL_INJECT
    inject_set_fullsequential(1); //set full sequential mode (if selected) here, because we already obtained sync.pulse from a cam sensor
#endif
    SETBIT(flags, F_ISSYNC);
    ckps.cog = 1; //first tooth
    return 1; //finish
   }
   break;
#endif

  case 2: //find out missing teeth
   //if missing teeth = 0, then reference will be identified by additional VR sensor (REF_S input)
   if ((0==ckps.miss_cogs_num) ? cams_vr_is_event_r() : (ckps.period_curr > CKPS_GAP_BARRIER(ckps.period_prev)))
   {
#ifdef PHASED_IGNITION
    if (CHECKBIT(flags2, F_SINGCH))
     set_channels_sc(); //single channel mode
    else
     set_channels_fs(1);
#endif
#ifdef FUEL_INJECT
    inject_set_fullsequential(1);
#endif
    SETBIT(flags, F_ISSYNC);
    ckps.period_curr = ckps.period_prev;  //exclude value of missing teeth's period
    ckps.cog = 1; //first tooth
    return 1; //finish process of synchronization
   }
   break;
 }
 ckps.icr_prev = ICR1;
 ckps.period_prev = ckps.period_curr;
 ++ckps.cog;
 return 0; //continue process of synchronization
}

/**This procedure called for all teeth (including recovered teeth)
 */
static void process_ckps_cogs(void)
{
 uint8_t i;

#ifdef DWELL_CONTROL
 if (CHECKBIT(flags, F_PNDDWL) && !ckps.rising_edge_spark)
 {
  //calculate delay between current tooth and next spark
  int32_t angle_to_spark = (((int32_t)_normalize_tn(chanstate[ckps.channel_mode_b].cogs_btdc - ckps.cog)) * ckps.degrees_per_cog) - ckps.advance_angle;
  int32_t delay =  (angle_to_spark * (((int32_t)ckps.period_curr * ckps.degrees_per_cog_r) >> 10)) >> 6; //convert angle to delay
  delay-= ckps.cr_acc_time;    //apply dwell time

  if (delay < (ckps.period_curr<<1))
  {
   if (delay < DWL_DEAD_TIME)
    delay = DWL_DEAD_TIME; //restrict accumulation time.

   if (QUEUE_IS_EMPTY(1))
   {
    SET_T1COMPA(ICR1, (uint16_t)delay);
   }
   QUEUE_ADD(1, ICR1, (uint16_t)delay, QID_DWELL);
   CLEARBIT(flags, F_PNDDWL);  // To avoid entering into setup mode
  }
 }
#ifdef SPLIT_ANGLE
 if (CHECKBIT(flags2, F_PNDDWL1) && !ckps.rising_edge_spark)
 {
  //calculate delay between current tooth and next spark
  int32_t angle_to_spark = (((int32_t)_normalize_tn(chanstate[ckps.channel_mode_b1].cogs_btdc - ckps.cog)) * ckps.degrees_per_cog) - ckps.advance_angle1;
  int32_t delay =  (angle_to_spark * (((int32_t)ckps.period_curr * ckps.degrees_per_cog_r) >> 10)) >> 6; //convert angle to delay
  delay-= ckps.cr_acc_time;    //apply dwell time

  if (delay < (ckps.period_curr<<1))
  {
   if (delay < DWL_DEAD_TIME)
    delay = DWL_DEAD_TIME; //restrict accumulation time.

   if (QUEUE_IS_EMPTY(2))
   {
    SET_T3COMPA(ICR1, (uint16_t)delay);  //note: we rely that timers 1 and 3 are synchronized!
   }
   QUEUE_ADD(2, ICR1, (uint16_t)delay, QID_DWELL);
   CLEARBIT(flags2, F_PNDDWL1);  // To avoid entering into setup mode
  }
 }
#endif
#endif

 for(i = 0; i < ckps.chan_number; ++i)
 {
  if (CHECKBIT(flags, F_USEKNK))
  {
   //start listening a detonation (opening the window)
   //начинаем слушать детонацию
   if (ckps.cog == chanstate[i].knock_wnd_begin)
    knock_set_integration_mode(KNOCK_INTMODE_INT);

   //finish listening a detonation (closing the window) and start the process of measuring integrated value
   if (ckps.cog == chanstate[i].knock_wnd_end)
   {
    knock_set_integration_mode(KNOCK_INTMODE_HOLD);
    if (CHECKBIT(flags, F_USEKNK))
     knock_start_settings_latching();//start the process of downloading the settings into the HIP9011 (and getting ADC result for TPIC8101)
#ifndef TPIC8101
    adc_begin_measure_knock(_AB(ckps.stroke_period, 1) < 4);
#endif
   }
  }

  //for 66° before TDC (before working stroke) establish new advance angle to be actuated,
  //before this moment value was stored in a temporary buffer.
  if (ckps.cog == chanstate[i].cogs_latch)
  {
   ckps.channel_mode = i;                    //remember number of channel
#ifdef SPLIT_ANGLE
   ckps.channel_mode1 = i + SPLIT_OFFSET;
   SETBIT(flags2, F_PNDSPK1);                //establish an indication that it is need to count advance angle
#endif
   SETBIT(flags, F_PNDSPK);                  //establish an indication that it is need to count advance angle
   //start counting of advance angle
   ckps.current_angle = ckps.start_angle; // those same 66°
   ckps.advance_angle = ckps.advance_angle_buffered; //advance angle with all the adjustments (say, 15°)
#ifdef SPLIT_ANGLE
   ckps.advance_angle1 = ckps.advance_angle_buffered1; //advance angle with all the adjustments (say, 15°)
#endif
   adc_begin_measure(_AB(ckps.stroke_period, 1) < 4);//start the process of measuring analog input values
#ifdef STROBOSCOPE
   if (0==i)
    ckps.strobe = 1; //strobe!
#endif
  }

  //teeth of end/beginning of the measurement of rotation period - TDC Read and save the measured period,
  //then remember current value of count for the next measurement
  if (ckps.cog==chanstate[i].cogs_btdc)
  {
   //save period value if it is correct
   if (CHECKBIT(flags, F_VHTPER))
   {
    ckps.stroke_period = (ICR1 - ckps.measure_start_value);
    WRITEBIT(flags2, F_SPSIGN, ICR1 < ckps.measure_start_value); //save sign
    ckps.t1oc_s = ckps.t1oc, ckps.t1oc = 0; //save value and reset counter
   }

   ckps.measure_start_value = ICR1;
   SETBIT(flags, F_VHTPER);
   SETBIT(flags, F_STROKE); //set the stroke-synchronozation event
  }

#ifdef HALL_OUTPUT
  if (ckps.cog == chanstate[i].hop_begin_cog)
   IOCFG_SET(IOP_HALL_OUT, 1);
  if (ckps.cog == chanstate[i].hop_end_cog)
   IOCFG_SET(IOP_HALL_OUT, 0);
#endif

#ifdef FUEL_INJECT
   //control injection timing using teeth and COMPB timer channel
   if (!chanstate[i].inj_skipth)
   {
    uint16_t diff = _normalize_angle(((int16_t)chanstate[i].inj_angle_safe) - ((int16_t)(ckps.cog * ckps.degrees_per_cog)));
    if (diff <= (ckps.degrees_per_cog << 1))
    {
     ckps.inj_chidx = i;  //remember number of channel to be fired
     uint16_t delay = ((((uint32_t)diff * (ckps.period_curr)) * ckps.degrees_per_cog_r) >> 16) - COMPB_VECT_DELAY;
     SET_T1COMPB(ICR1, delay);
     sync_inj_angle();
     chanstate[i].inj_skipth = 4;  //skip 4 teeth
    }
   }
   else
    --(chanstate[i].inj_skipth);
#endif
 }

 //Preparing to start the ignition for the current channel (if the right moment became)
 if (CHECKBIT(flags, F_PNDSPK))
 {
  uint16_t diff = ckps.current_angle - ckps.advance_angle;
  if (diff <= (ckps.degrees_per_cog << 1))
  {
   uint16_t delay = ((((uint32_t)diff * (ckps.period_curr)) * ckps.degrees_per_cog_r) >> 16) - COMPA_VECT_DELAY;
#ifdef DWELL_CONTROL
   //before starting the ignition it is left to count less than 2 teeth. It is necessary to prepare the compare module
   if (QUEUE_IS_EMPTY(1))
   {
    SET_T1COMPA(ICR1, delay);
   }
   QUEUE_ADD(1, ICR1, (uint16_t)delay, QID_SPARK);
#else
   SET_T1COMPA(ICR1, delay);
#endif
   CLEARBIT(flags, F_PNDSPK); // For avoiding to enter into setup mode
  }
 }

#ifdef SPLIT_ANGLE
 //Preparing to start the ignition for the current channel (if the right moment became)
 if (CHECKBIT(flags2, F_PNDSPK1))
 {
  uint16_t diff = ckps.current_angle - ckps.advance_angle1;
  if (diff <= (ckps.degrees_per_cog << 1))
  {
   uint16_t delay = ((((uint32_t)diff * (ckps.period_curr)) * ckps.degrees_per_cog_r) >> 16) - COMPA_VECT_DELAY;
#ifdef DWELL_CONTROL
   //before starting the ignition it is left to count less than 2 teeth. It is necessary to prepare the compare module
   if (QUEUE_IS_EMPTY(2))
   {
    SET_T3COMPA(ICR1, delay); //note: we rely that timers 1 and 3 are synchronized!
   }
   QUEUE_ADD(2, ICR1, (uint16_t)delay, QID_SPARK);
#else
   SET_T3COMPA(ICR1, delay); //note: we rely that timers 1 and 3 are synchronized!
#endif
   CLEARBIT(flags2, F_PNDSPK1); // For avoiding to enter into setup mode
  }
 }
#endif


#ifndef DWELL_CONTROL
 //finish the ignition trigger pulses for igniter(s) and immediately increase the number of tooth for processed channel
#ifdef SPLIT_ANGLE
 for(i = 0; i < ckps.chan_number + SPLIT_OFFSET; ++i)
#else
 for(i = 0; i < ckps.chan_number; ++i)
#endif
 {
  if (chanstate[i].ignition_pulse_cogs == 255)
   continue;

  if (chanstate[i].ignition_pulse_cogs >= ckps.ignition_cogs)
  {
   turn_off_ignition_channel(i);
   chanstate[i].ignition_pulse_cogs = 255; //set indication that channel has finished to work
  }
  else
   ++(chanstate[i].ignition_pulse_cogs);
 }
#endif

 //tooth passed - angle before TDC decriased (e.g 6° per tooth for 60-2).
 ckps.current_angle-= ckps.degrees_per_cog;

#ifdef PHASE_SENSOR
 //search for level's toggle from camshaft sensor on each cog
 if (CHECKBIT(flags2, F_CAMREF))
  cams_detect_edge();
#endif
}

/**Input capture interrupt of timer 1 (called at passage of each tooth)
 */
ISR(TIMER1_CAPT_vect)
{
 ckps.period_curr = ICR1 - ckps.icr_prev;

 //At the start of engine, skipping a certain number of teeth for initializing
 //the memory of previous periods. Then look for missing teeth.
 if (!CHECKBIT(flags, F_ISSYNC))
 {
  if (sync_at_startup())
  {
#ifdef DWELL_CONTROL
   SETBIT(flags, F_PNDDWL); //it is need to set compare channel for dwell start
#ifdef SPLIT_ANGLE
   SETBIT(flags2, F_PNDDWL1);
#endif
#endif
#ifdef FUEL_INJECT
   sync_inj_angle();
#endif
   goto sync_enter;
  }
  return;
 }

#ifdef PHASE_SENSOR
 if (CHECKBIT(flags2, F_CAMREF))
 {
  if (cams_is_event_r())
  {
   if (ckps.cog != ckps.wheel_cogs_nump1) //check if sync is correct
    SETBIT(flags, F_ERROR); //ERROR
   ckps.cog = 1; //each 720°
  }
 }
 else
#endif
 {
  //if missing teeth = 0, then reference will be identified by additional VR sensor (REF_S input),
  //Otherwise:
  //Each period, check for missing teeth, and if, after discovering of missing teeth
  //count of teeth being found incorrect, then set error flag.
  if ((0==ckps.miss_cogs_num) ? cams_vr_is_event_r() : (ckps.period_curr > CKPS_GAP_BARRIER(ckps.period_prev)))
  {
   if ((ckps.cog != ckps.wheel_cogs_nump1)) //also taking into account recovered teeth
    SETBIT(flags, F_ERROR); //ERROR
   ckps.cog = 1;
   ckps.period_curr = ckps.period_prev;  //exclude value of missing teeth's period (for missing teeth only)
  }
 }

sync_enter:
#ifdef PHASE_SENSOR
 if (!CHECKBIT(flags2, F_CAMREF))
#endif
 {
  //If the last tooth before missing teeth, we begin the countdown for
  //the restoration of missing teeth, as the initial data using the last
  //value of inter-teeth period.
  if (ckps.miss_cogs_num && ckps.cog == ckps.wheel_last_cog)
   set_timer0(ckps.period_curr);
 }

 //call handler for normal teeth
 process_ckps_cogs();
 ++ckps.cog;

 ckps.icr_prev = ICR1;
 ckps.period_prev = ckps.period_curr;
}

/**Purpose of this interrupt handler is to supplement timer up to 16 bits and call procedure
 * for processing teeth when set 16 bit timer expires
 */
ISR(TIMER0_COMPA_vect)
{
 if (ckps.TCNT0_H)  //Did high byte exhaust ?
 {
  --ckps.TCNT0_H;
 }
 else
 {//the countdown is over
  ICR1 = TCNT1;  //simulate input capture
  CLEARBIT(TIMSK0, OCIE0A); //disable this interrupt

  if (ckps.miss_cogs_num > 1)
  {
   //start timer to recover 60th tooth
   if (ckps.cog == ckps.wheel_cogs_numm1)
    set_timer0(ckps.period_curr);
  }

  //Call handler for missing teeth
  process_ckps_cogs();
  ++ckps.cog;
 }
}

/** Timer 1 overflow interrupt.
 * Used to count timer 1 overflows to obtain correct revolution period at very low RPMs (10...400 min-1)
 */
ISR(TIMER1_OVF_vect)
{
 ++ckps.t1oc;
}

#endif //CAM_SYNC
