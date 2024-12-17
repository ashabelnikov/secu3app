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

/** \file ckps-odd.c
 * \author Alexey A. Shabelnikov
 * Implementation of crankshaft position sensor's processing for odd fire engines.
 */

#if defined(ODDFIRE_ALGO)

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

/**Threshold value used for detection of expiration of events in timer's queue*/
#define EXPEVENT_THRD 65400

/**Maximim number of teeth */
#define TEETH_MAX 200

//PHASED_IGNITION can't be used without PHASE_SENSOR
#if defined(PHASED_IGNITION) && !defined(PHASE_SENSOR)
 #error "You can not use phased ignition without phase sensor. Define PHASE_SENSOR if it is present in the system or do not use phased ignition!"
#endif

#if !defined(DWELL_CONTROL)
 #error "DWELL_CONTROL option is required"
#endif

#if defined(CKPS_2CHIGN)
 #error "CKPS_2CHIGN option is not compatible with ODDFIRE_ALGO"
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
 *      for 12-3 crank wheel, p * 3.0
 */
#define CKPS_GAP_BARRIER(p)  (((p) * ckps.mttf) >> 8)

//Define values for controlling of outputs
#define IGN_OUTPUTS_INIT_VAL 1        //!< value used for initialization
#define IGN_OUTPUTS_ON_VAL   1        //!< value used to turn on ignition channel
#define IGN_OUTPUTS_OFF_VAL  0        //!< value used to turn off ignition channel

#define IGNOUTCB_ON_VAL (ckps.ignout_on_val)
#define IGNOUTCB_OFF_VAL (ckps.ignout_off_val)

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
#define F_IGNIEN    5                 //!< Ignition enabled/disabled
#define F_STROBE    6                 //!< Stroboscope function is active (output mapped to the real I/O)
#define F_HALOUT    7                 //!< Hall output function is active (output mapped to the real I/O)

//Additional flags (see flags2 variable)
#if defined(PHASED_IGNITION) || (defined(PHASE_SENSOR) && defined(FUEL_INJECT))
 #define F_CAMISS    0                //!< Indicates that system has already obtained event from a cam sensor
#endif
#define F_SPSIGN     1                //!< Sign of the measured stroke period (time between TDCs)
#define F_SINGCH     2                //!< indicates that single ignition channel is used
#ifdef PHASE_SENSOR
#define F_CAMREF     3                //!< Specifies to use camshaft sensor as reference
#endif

/**Calculates index of next channel using specified index i*/
#define NEXT_CHIDX(i) (((i) < ckps.chan_number-1) ? (i) + 1 : 0)

/** State variables */
typedef struct
{
 uint16_t icr_prev;                   //!< previous value if Input Capture Register
 volatile uint16_t period_curr;       //!< last measured inter-tooth period
 uint16_t period_prev;                //!< previous value of inter-tooth period
 volatile uint16_t cog;               //!< counts teeth starting from missing teeth (2 revolutions), begins from 1
#ifndef CAM_SYNC
 volatile uint8_t cog360;             //!< counts teeth starting from missing teeth (1 revolution).
#endif
 uint16_t cogang[TEETH_MAX];          //!< look up table for converting cog's number to corresponding angle
 volatile uint8_t eq_tail1;           //!< event queue tail (index in a static array)
 volatile uint8_t eq_head1;           //!< event queue head (index), queue is empty if head = tail
 volatile uint8_t chan_number;        //!< number of ignition channels
 volatile uint8_t wheel_last_cog;     //!< Number of last(present) tooth, numeration begins from 0
 volatile uint8_t wheel_cogs_num;     //!< Number of teeth, including missing
 volatile uint8_t miss_cogs_num;      //!< Count of crank wheel's missing teeth (0, 1, 2)
 volatile uint8_t wheel_cogs_numm1;   //!< wheel_cogs_num - 1
#ifndef CAM_SYNC
 volatile uint16_t wheel_cogs_num2;   //!< Number of teeth which corresponds to 720° (2 revolutions)
#endif
 volatile uint8_t ignout_on_val;      //!< value used to turn on ignition channel
 volatile uint8_t ignout_off_val;     //!< value used to turn off ignition channel
 volatile uint16_t stroke_period;     //!< stores the last measurement of period between neighbor strokes
 uint16_t measure_start_value;        //!< remembers the value of the capture register to measure the half-turn
 uint32_t frq_calc_dividend;          //!< divident for calculating RPM
 volatile uint8_t rising_edge_spark;  //!< flag, indicates that rising edge of ignition pulse will be generated at the moment of spark
 volatile uint16_t cr_acc_time;       //!< accumulation time for dwell control (timer's ticks)
 volatile uint16_t degrees_per_cog;   //!< Number of degrees which corresponds to the 1 tooth
 volatile uint16_t degrees_per_cog_r; //!< Reciprocal of the degrees_per_cog, value * 65536
 volatile uint16_t wheel_deg_r;       //!< constant = wheel_cogs_num*(65536/360)

 volatile uint8_t TCNT0_H;            //!< For supplementing timer/counter 0 up to 16 bits
 volatile uint8_t t1oc;               //!< Timer 1 overflow counter
 volatile uint8_t t1oc_s;             //!< Contains value of t1oc synchronized with stroke_period value

#ifdef SPLIT_ANGLE
 volatile uint8_t chan_number_split;  //!< number of ignition channels with splitting
 volatile uint8_t eq_tail3;           //!< event queue tail (index in a static array)
 volatile uint8_t eq_head3;           //!< event queue head (index), queue is empty if head = tail
#endif
 uint8_t  starting_mode;              //!< state of state machine processing of teeth at the startup

#ifdef FUEL_INJECT
 uint8_t  inj_chidx;                  //!< index of the channel to fire of the injection
#endif
 volatile uint16_t mttf;              //!< factor for calculating gap barrier (missing teeth detection)
}ckpsstate_t;

/**Precalculated data (reference points) and state data for a single channel plug
 */
typedef struct
{
 volatile uint16_t ign_tooth[2];      //!< number of tooth corresponding to the moment of spark, starts from 0 tooth
 volatile uint16_t ign_frac[2];       //!< fraction of tooth corresponding to the moment of spark (value in 1/1024 units)
 volatile uint16_t dwl_tooth[2];      //!< number of tooth correcponding to the moment of start of dwell, starts from 0 tooth
 volatile uint16_t dwl_frac[2];       //!< fraction of correcponding to the moment of start of dwell (value in 1/1024 units)
 volatile uint16_t ign_angle;         //!< Used for storing calculated advance angle (value * ANGLE_MULTIPLIER), starts from 0 tooth

 volatile uint16_t rpm_tooth;         //!< number of tooth corresponding to the moment of sampling period for RPM
 volatile uint16_t rpm_frac;          //!< fraction of tooth corresponding to the moment of sampling period for RPM
 volatile uint16_t msr_tooth;         //!< number of tooth corresponding to the moment of sampling sensors' values
 volatile uint16_t msr_frac;          //!< fraction of tooth corresponding to the moment of sampling sensors' values

 volatile fnptr_t io_callback1;       //!< Address of callback which will be used for settiong of I/O
#ifdef PHASED_IGNITION
 volatile fnptr_t io_callback2;       //!< Second callback used only in semi-sequential ignition mode
#endif

 volatile uint16_t knb_tooth;         //!< number of tooth at which phase selection window for knock detection is opened
 volatile uint16_t knb_frac;          //!< fraction of tooth at which phase selection window for knock detection is opened
 volatile uint16_t kne_tooth;         //!< number of tooth at which phase selection window for knock detection is closed
 volatile uint16_t kne_frac;          //!< fraction of tooth at which phase selection window for knock detection is closed

#ifdef HALL_OUTPUT
 volatile uint16_t hob_tooth;         //!< number of tooth that corresponds to the beginning of pulse (Hall output)
 volatile uint16_t hob_frac;          //!< fraction of tooth that corresponds to the beginning of pulse (Hall output)
 volatile uint16_t hoe_tooth;         //!< number of tooth that corresponds to the end of pulse
 volatile uint16_t hoe_frac;          //!< fraction of tooth that corresponds to the end of pulse
#endif

#ifdef FUEL_INJECT
 volatile uint16_t inj_tooth[2];      //!< Injection timing's tooth
 volatile uint16_t inj_frac[2];       //!< Injection timing's fraction of tooth
#endif

 volatile uint8_t knock_chan;         //!< Selected knock channel (0 - KS_1 or 1 - KS_2)
}chanstate_t;

ckpsstate_t ckps;                         //!< instance of state variables
chanstate_t chanstate[IGN_CHANNELS_MAX];  //!< instance of array of channel's state variables

// Arrange flags in the free I/O register
//  note: may be not effective on other MCUs or even cause bugs! Be aware.
#define flags  GPIOR0                 //!< ATmega1284 has one general purpose I/O register and we use it for first flags variable
#define flags2 TWBR                   //!< Second flags variable in I/O register

/**Accessor macro for RPM dividents table*/
#define FRQ_CALC_DIVIDEND(channum) PGM_GET_DWORD(&frq_calc_dividend[channum])
/**Table srtores dividends for calculating of RPM */
PGM_DECLARE(uint32_t frq_calc_dividend[1+IGN_CHANNELS_MAX]) =
 //     1          2          3          4         5         6         7         8
 {0, 37500000L, 18750000L, 12500000L, 9375000L, 7500000L, 6250000L, 5357143L, 4687500L};

/**Maximum queue size for ignition events, MUST BE power of two (2,4,8 etc) */
#define IGN_QUEUE_SIZE 32

#define QID_DWELL   0             //!< start dwell
#define QID_SPARK   1             //!< finish dwell (spark)
#define QID_RPMSAMP 2             //!< sample RPM
#define QID_MEASURE 3             //!< measure sensors' values
#define QID_KNKBEG  4             //!< knock window begin
#define QID_KNKEND  5             //!< knock window end
#ifdef STROBOSCOPE
#define QID_STROBE  6             //!< finish strobe pulse
#endif
#ifdef HALL_OUTPUT
#define QID_HOPBEG  7             //!< hall output pulse's begin
#define QID_HOPEND  8             //!< hall output pulse's end
#endif

/**Describes event queue entry for T1 COMPA channel*/
typedef struct
{
 uint16_t end_time;              //!< End time of event in ticks of free running timer 1
 uint8_t  id;                    //!< pending action Id
 uint8_t  ch;                    //!< channel number
}ign_queue_t;

/**Event queue for scheduling of ignition spark events (T1 COMPA) */
ign_queue_t ign_eq1[IGN_QUEUE_SIZE];

#ifdef SPLIT_ANGLE
/**Event queue for scheduling of ignition spark events (T3 COMPA) */
ign_queue_t ign_eq3[IGN_QUEUE_SIZE];
#endif

/** Reset specified queue (makes it empty) */
#define QUEUE_RESET(q)  ckps.eq_tail##q = ckps.eq_head##q = 0;

/** Add event into the queue (add to head)
 * q Number of queue
 * r Timer's register (TCNT1 or ICR1)
 * time Time in tics of timer 1 after which event should fire
 * aid ID of pending action, which should be performed
 * chan Number of channel
 */
#define QUEUE_ADD(q, r, time, aid, chan) \
    uint8_t qi = ckps.eq_head##q; \
    while(qi != ckps.eq_tail##q) \
    { \
     uint8_t im1 = (qi - 1) & (IGN_QUEUE_SIZE-1); \
     uint16_t dt = ign_eq##q[im1].end_time - (r); \
     if (dt > EXPEVENT_THRD || (time) > dt) \
      break; \
     ign_eq##q[qi] = ign_eq##q[im1]; \
     qi = im1; \
    } \
    ign_eq##q[qi].end_time = (r) + (time); \
    ign_eq##q[qi].id = (aid); \
    ign_eq##q[qi].ch = (chan); \
    ckps.eq_head##q = (ckps.eq_head##q + 1) & (IGN_QUEUE_SIZE-1); \
    if (qi == ckps.eq_tail##q) \
    { \
     SET_T##q##COMPA((r), (time)); \
    }

/** A more computationally economical version of QUEUE_ADD() macro
 * (without selection of channel)
 */
#define QUEUE_ADDF(q, r, time, aid) \
    uint8_t qi = ckps.eq_head##q; \
    while(qi != ckps.eq_tail##q) \
    { \
     uint8_t im1 = (qi - 1) & (IGN_QUEUE_SIZE-1); \
     uint16_t dt = ign_eq##q[im1].end_time - (r); \
     if (dt > EXPEVENT_THRD || (time) > dt) \
      break; \
     ign_eq##q[qi] = ign_eq##q[im1]; \
     qi = im1; \
    } \
    ign_eq##q[qi].end_time = (r) + (time); \
    ign_eq##q[qi].id = (aid); \
    ckps.eq_head##q = (ckps.eq_head##q + 1) & (IGN_QUEUE_SIZE-1); \
    if (qi == ckps.eq_tail##q) \
    { \
     SET_T##q##COMPA((r), (time)); \
    }

/** Remove event from the queue (remove from tail)*/
#define QUEUE_REMOVE(q) ckps.eq_tail##q = (ckps.eq_tail##q + 1) & (IGN_QUEUE_SIZE-1)

/** Get tail item value from queue */
#define QUEUE_TAIL(q) ign_eq##q[ckps.eq_tail##q]

/** Test is queue empty */
#define QUEUE_IS_EMPTY(q) (ckps.eq_head##q==ckps.eq_tail##q)

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

/**Converts fraction of tooth (1/1024 units) to time (3.2us units)*/
#define FRAC_TO_TIME(frac) ((((uint32_t)ckps.period_curr) * (frac)) >> 10);

void ckps_init_state_variables(void)
{
 _BEGIN_ATOMIC_BLOCK();
 TIMSK1|=_BV(TOIE1);                  //enable Timer 1 overflow interrupt. Used for correct calculation of very low RPM
 ckps.t1oc = 0;                       //reset overflow counter
 ckps.t1oc_s = 255;                   //RPM is very low
 CLEARBIT(flags, F_STROKE);
 CLEARBIT(flags, F_ISSYNC);
 SETBIT(flags, F_IGNIEN);
 CLEARBIT(flags2, F_SPSIGN);
#if defined(PHASED_IGNITION) || (defined(PHASE_SENSOR) && defined(FUEL_INJECT))
 CLEARBIT(flags2, F_CAMISS);
#endif
#ifdef CAM_SYNC
 ckps.cog = 0;
#else
 ckps.cog = ckps.cog360 = 0;
#endif
 ckps.stroke_period = 0xFFFF;
 ckps.cr_acc_time = 0;
 ckps.starting_mode = 0;
 QUEUE_RESET(1);
#ifdef SPLIT_ANGLE
 QUEUE_RESET(3);
#endif
 _END_ATOMIC_BLOCK();
}

void ckps_init_state(void)
{
 _BEGIN_ATOMIC_BLOCK();
 ckps_init_state_variables();
 CLEARBIT(flags, F_ERROR);

 WRITEBIT(flags, F_STROBE, IOCFG_CHECK(IOP_STROBE));
 WRITEBIT(flags, F_HALOUT, IOCFG_CHECK(IOP_HALL_OUT));

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

/** Converts dwell time to angle
 * \param Angle (value * ANGLE_MULTIPLIER)
 */
static uint16_t dwell_to_angle(void)
{
 uint16_t period_curr;
 _BEGIN_ATOMIC_BLOCK();
 period_curr = ckps.period_curr;
 _END_ATOMIC_BLOCK();
 //limit dwell time, convert it to corresponding angle (value * ANGLE_MULTIPLIER)
 uint16_t dpw = ckps.cr_acc_time;
#if defined(PHASED_IGNITION) && !defined(CAM_SYNC)
 uint32_t maxdwl = (((uint32_t)period_curr) * ckps.wheel_cogs_num2) - PGM_GET_WORD(&fw_data.exdata.dwl_dead_time);
#else
 uint32_t maxdwl = (((uint32_t)period_curr) * ckps.wheel_cogs_num) - PGM_GET_WORD(&fw_data.exdata.dwl_dead_time);
#endif
 if (dpw > maxdwl)
  dpw = maxdwl;
 return (((uint32_t)dpw) * ckps.degrees_per_cog) / period_curr;
}

/** Ensures that angle will be in the allowed range (0...720)
 * \param angle Angle to be normalized, may be negative
 * \return Normalized and correct value of angle
 */
static inline uint16_t _normalize_angle(int16_t angle)
{
 if (angle > ANGLE_MAGNITUDE(720))
  return angle - ANGLE_MAGNITUDE(720);
 if (angle < 0)
  return ANGLE_MAGNITUDE(720) + angle;
 return angle;
}

/** Converts angle (0...720) to the pair: number of tooth + fraction of tooth (1024 discretes per tooth)
 * \param angle Source angle, value * ANGLE_MULTIPLIER
 * \param tooth_num Pointer to variable which will receive number of tooth
 * \param tooth_fr Pointer to variable which will receive fraction of tooth (1/1024)
 */
void static angle_to_tooth(int16_t angle, uint16_t* tooth_num, uint16_t* tooth_fr)
{
 int16_t tooth = ((((int32_t)angle) * ckps.wheel_deg_r) >> (16+5)) - 1;
 if (tooth < 0)
#ifdef CAM_SYNC
  tooth+= ckps.wheel_cogs_num;
#else
  tooth+= ckps.wheel_cogs_num2;
#endif
 uint16_t frac = _normalize_angle(angle - ckps.cogang[tooth]);
 *tooth_fr = ((((uint32_t)frac) * 1024) * ckps.degrees_per_cog_r) >> 16;
 *tooth_num = tooth;
}

void ckps_set_advance_angle(int16_t angle)
{
 uint8_t _t, i;
 for(i = 0; i < ckps.chan_number; ++i)
 {
  //calculate angle corresponding to the moment of spark
  int16_t ign_angle = _normalize_angle(((int16_t)PGM_GET_WORD(&fw_data.exdata.tdc_angle[i])) - angle);
  chanstate[i].ign_angle = ign_angle; //save value
  uint16_t tooth1, frac1;
  angle_to_tooth(ign_angle, &tooth1, &frac1);
  //calculate angle corresponding to the moment of dwell (it must be updated too!)
  int16_t dwl_angle = _normalize_angle(ign_angle + (ckps.rising_edge_spark ? dwell_to_angle() : - dwell_to_angle()));
  uint16_t tooth2, frac2;
  angle_to_tooth(dwl_angle, &tooth2, &frac2);

  _t=_SAVE_INTERRUPT();
  _DISABLE_INTERRUPT();
  chanstate[i].ign_tooth[0] = tooth1;
  chanstate[i].ign_frac[0] = frac1;
  chanstate[i].dwl_tooth[0] = tooth2;
  chanstate[i].dwl_frac[0] = frac2;
  _RESTORE_INTERRUPT(_t);
 }
}

#ifdef SPLIT_ANGLE
void ckps_set_advance_angle1(int16_t angle)
{
 uint8_t i, _t;
 for(i = SPLIT_OFFSET; i < ckps.chan_number_split; ++i)
 {
  //calculate angle corresponding to the moment of spark
  int16_t ign_angle = _normalize_angle(((int16_t)PGM_GET_WORD(&fw_data.exdata.tdc_angle[i])) - angle);
  chanstate[i].ign_angle = ign_angle; //save value
  uint16_t tooth1, frac1;
  angle_to_tooth(ign_angle, &tooth1, &frac1);
  //calculate angle corresponding to the moment of dwell (it must be updated too!)
  int16_t dwl_angle = _normalize_angle(ign_angle + (ckps.rising_edge_spark ? dwell_to_angle() : - dwell_to_angle()));
  uint16_t tooth2, frac2;
  angle_to_tooth(dwl_angle, &tooth2, &frac2);

  _t=_SAVE_INTERRUPT();
  _DISABLE_INTERRUPT();
  chanstate[i].ign_tooth[0] = tooth1;
  chanstate[i].ign_frac[0] = frac1;
  chanstate[i].dwl_tooth[0] = tooth2;
  chanstate[i].dwl_frac[0] = frac2;
  _RESTORE_INTERRUPT(_t);
 }
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


/** Synchronize values of event's angles to use them in interrupts safely */
static inline void sync_ign_angle(void)
{
 uint8_t i;
 for(i = 0; i < ckps.chan_number; ++i)
 {
  _BEGIN_ATOMIC_BLOCK();
  chanstate[i].dwl_tooth[1] = chanstate[i].dwl_tooth[0];
  chanstate[i].dwl_frac[1] = chanstate[i].dwl_frac[0];
  chanstate[i].ign_tooth[1] = chanstate[i].ign_tooth[0];
  chanstate[i].ign_frac[1] = chanstate[i].ign_frac[0];
  _END_ATOMIC_BLOCK();
 }
#ifdef SPLIT_ANGLE
 for(i = SPLIT_OFFSET; i < ckps.chan_number_split; ++i)
 {
  _BEGIN_ATOMIC_BLOCK();
  chanstate[i].dwl_tooth[1] = chanstate[i].dwl_tooth[0];
  chanstate[i].dwl_frac[1] = chanstate[i].dwl_frac[0];
  chanstate[i].ign_tooth[1] = chanstate[i].ign_tooth[0];
  chanstate[i].ign_frac[1] = chanstate[i].ign_frac[0];
  _END_ATOMIC_BLOCK();
 }
#endif
}

#ifdef FUEL_INJECT
/** Synchronize injection angle values */
static inline void sync_inj_angle(void)
{
 uint8_t i;
 _BEGIN_ATOMIC_BLOCK();
 for(i = 0; i < ckps.chan_number; ++i)
 {
  chanstate[i].inj_tooth[1] = chanstate[i].inj_tooth[0];
  chanstate[i].inj_frac[1] = chanstate[i].inj_frac[0];
 }
 _END_ATOMIC_BLOCK();
}
#endif

void ckps_set_cogs_btdc(uint8_t cogs_btdc)
{
 //stub. This function is not used by new algorithm
}

void ckps_set_acc_time(uint16_t i_acc_time)
{
 uint8_t i;
 ckps.cr_acc_time = i_acc_time;
 //update angles for channels
 for(i = 0; i < ckps.chan_number; ++i)
 {
  int16_t angle = ((int16_t)chanstate[i].ign_angle) + (ckps.rising_edge_spark ? dwell_to_angle() : - dwell_to_angle());
  int16_t dwl_angle = _normalize_angle(angle);
  uint16_t tooth, frac;
  angle_to_tooth(dwl_angle, &tooth, &frac);
  _BEGIN_ATOMIC_BLOCK();
  chanstate[i].dwl_tooth[0] = tooth;
  chanstate[i].dwl_frac[0] = frac;
  _END_ATOMIC_BLOCK();
 }
#ifdef SPLIT_ANGLE
 for(i = SPLIT_OFFSET; i < ckps.chan_number_split; ++i)
 {
  int16_t angle = ((int16_t)chanstate[i].ign_angle) + (ckps.rising_edge_spark ? dwell_to_angle() : - dwell_to_angle());
  int16_t dwl_angle = _normalize_angle(angle);
  uint16_t tooth, frac;
  angle_to_tooth(dwl_angle, &tooth, &frac);
  _BEGIN_ATOMIC_BLOCK();
  chanstate[i].dwl_tooth[0] = tooth;
  chanstate[i].dwl_frac[0] = frac;
  _END_ATOMIC_BLOCK();
 }
#endif
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
  //Tune for full sequential mode if cam sensor works, otherwise tune for semi-sequential mode
#ifdef CAM_SYNC
  set_channels_fs(1);
#else
  set_channels_fs(cams_is_ready());
#endif
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
}

void ckps_set_knock_window(int16_t begin, int16_t end)
{
 for(uint8_t i = 0; i < ckps.chan_number; ++i)
 {
  int16_t knb_angle = _normalize_angle(((int16_t)PGM_GET_WORD(&fw_data.exdata.tdc_angle[i])) + begin);
  uint16_t tooth1, frac1;
  angle_to_tooth(knb_angle, &tooth1, &frac1);
  int16_t kne_angle = _normalize_angle(((int16_t)PGM_GET_WORD(&fw_data.exdata.tdc_angle[i])) + end);
  uint16_t tooth2, frac2;
  angle_to_tooth(kne_angle, &tooth2, &frac2);

  _BEGIN_ATOMIC_BLOCKN(0);
  chanstate[i].knb_tooth = tooth1;
  chanstate[i].knb_frac = frac1;
  _END_ATOMIC_BLOCKN(0);
  _BEGIN_ATOMIC_BLOCKN(1);
  chanstate[i].kne_tooth = tooth2;
  chanstate[i].kne_frac = frac2;
  _END_ATOMIC_BLOCKN(1);
 }
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
#ifdef CAM_SYNC
  set_channels_fs(1);
#else
  set_channels_fs(cams_is_ready()); //Tune for full sequential mode if cam sensor works, otherwise tune for semi-sequential mode
#endif
#endif
 }
}

#ifdef HALL_OUTPUT
void ckps_set_hall_pulse(int16_t i_offset, uint16_t i_duration)
{
 for(uint8_t i = 0; i < ckps.chan_number; ++i)
 {
  int16_t hob_angle = _normalize_angle(((int16_t)PGM_GET_WORD(&fw_data.exdata.tdc_angle[i])) - i_offset);
  uint16_t tooth1, frac1;
  angle_to_tooth(hob_angle, &tooth1, &frac1);
  int16_t hoe_angle = _normalize_angle(hob_angle + i_duration);
  uint16_t tooth2, frac2;
  angle_to_tooth(hoe_angle, &tooth2, &frac2);

  _BEGIN_ATOMIC_BLOCKN(0);
  chanstate[i].hob_tooth = tooth1;
  chanstate[i].hob_frac = frac1;
  _END_ATOMIC_BLOCKN(0);
  _BEGIN_ATOMIC_BLOCKN(1);
  chanstate[i].hoe_tooth = tooth2;
  chanstate[i].hoe_frac = frac2;
  _END_ATOMIC_BLOCKN(1);
 }
}
#endif

void ckps_set_cogs_num(uint8_t norm_num, uint8_t miss_num)
{
 uint16_t i; uint8_t _t;
#ifdef PHASE_SENSOR
#ifdef CAM_SYNC
 uint16_t err_thrd = norm_num + (norm_num >> 3); //+ 12.5%
#else
 uint16_t err_thrd = (norm_num * 2) + (norm_num >> 3); //+ 12.5%
#endif
#endif
 uint16_t degrees_per_cog, degrees_per_cog_r;

 //precalculate value of degrees per 1 cog, it is fractional number multiplied by ANGLE_MULTIPLIER
#ifdef CAM_SYNC
 degrees_per_cog = (((((uint32_t)720) << 8) / norm_num) * ANGLE_MULTIPLIER) >> 8;
#else
 degrees_per_cog = (((((uint32_t)360) << 8) / norm_num) * ANGLE_MULTIPLIER) >> 8;
#endif

 //precalculate value of 1 / degrees_per_cog (reciprocal), result value multiplied by ~65536
 degrees_per_cog_r = (1*65535) / degrees_per_cog;

 ckps.wheel_deg_r = ((uint16_t)norm_num)*182; //182 = (1/360)*65536

 _t=_SAVE_INTERRUPT();
 _DISABLE_INTERRUPT();
 ckps.wheel_last_cog = (norm_num - miss_num) - 1; //calculate number of last cog
 ckps.wheel_cogs_num = norm_num;             //set number of teeth (normal and missing)
 ckps.miss_cogs_num = miss_num;
 ckps.wheel_cogs_numm1 = norm_num - 1;
#ifndef CAM_SYNC
 ckps.wheel_cogs_num2 = norm_num * 2;
#endif
 ckps.degrees_per_cog = degrees_per_cog;
 ckps.degrees_per_cog_r = degrees_per_cog_r; //reciprocal of the degrees_per_cog
#ifdef PHASE_SENSOR
 cams_set_error_threshold(err_thrd);
#endif
 _RESTORE_INTERRUPT(_t);

 //build look up table which will contain angles for each cog, value * ANGLE_MULTIPLIER
#ifdef CAM_SYNC
 for(i = 0; i < ckps.wheel_cogs_num; ++i)
  ckps.cogang[i] = ((((uint32_t)(720))*ANGLE_MULTIPLIER) * i) / ckps.wheel_cogs_num;
#else
 for(i = 0; i < ckps.wheel_cogs_num2; ++i)
  ckps.cogang[i] = ((((uint32_t)(720))*ANGLE_MULTIPLIER) * i) / ckps.wheel_cogs_num2;
#endif

 //Set angles for measuring values of sensors and rpm
 for(i = 0; i < ckps.chan_number; ++i)
 {
  int16_t msr_angle = _normalize_angle(((int16_t)PGM_GET_WORD(&fw_data.exdata.tdc_angle[i])) - PGM_GET_WORD(&fw_data.exdata.smp_angle));
  uint16_t tooth1, frac1;
  angle_to_tooth(msr_angle, &tooth1, &frac1);
  int16_t rpm_angle = _normalize_angle(PGM_GET_WORD(&fw_data.exdata.tdc_angle[0]) + (((uint32_t)ANGLE_MAGNITUDE(720.0)) * i) / ckps.chan_number);
  uint16_t tooth2, frac2;
  angle_to_tooth(rpm_angle, &tooth2, &frac2);

  uint8_t _t=_SAVE_INTERRUPT();
  _DISABLE_INTERRUPT();
  chanstate[i].msr_tooth = tooth1;
  chanstate[i].msr_frac = frac1;
  _ENABLE_INTERRUPT();
  _DISABLE_INTERRUPT();
  chanstate[i].rpm_tooth = tooth2;
  chanstate[i].rpm_frac = frac2;
  _RESTORE_INTERRUPT(_t);
 }
}

#ifdef FUEL_INJECT
void ckps_set_inj_timing(int16_t phase, uint16_t pw, uint8_t mode)
{
 uint8_t i;
 uint16_t period_curr;
 //TODO: We can do some optimization in the future - set timing only if it is not equal to current (already set one)

 phase = ANGLE_MAGNITUDE(720.0) - phase;

 _BEGIN_ATOMIC_BLOCK();
 period_curr = ckps.period_curr;
 _END_ATOMIC_BLOCK();

 //Apply selected injection pulse option: begin of squirt, middle of squirt or end of squirt
 if (mode > INJANGLESPEC_BEGIN)
 {
  //convert delay to angle (value * ANGLE_MULTIPLIER). TODO: how to escape from slow division?
  uint16_t pw_angle = (((uint32_t)pw) * ckps.degrees_per_cog) / period_curr;
  if (mode == INJANGLESPEC_MIDDLE)
   pw_angle>>= 1;
  //apply, rotate angle if need
  phase-=pw_angle;
  if (phase < 0)
   phase+=ANGLE_MAGNITUDE(720.0);
 }
 //---------------------------------------------------------

 for(i = 0; i < ckps.chan_number; ++i)
 {
  uint16_t angle = PGM_GET_WORD(&fw_data.exdata.tdc_angle[i]) + phase;
  if (angle > ANGLE_MAGNITUDE(720))
   angle-= ANGLE_MAGNITUDE(720);    //phase is periodical
  uint16_t tooth, frac;
  angle_to_tooth(angle, &tooth, &frac);

  _BEGIN_ATOMIC_BLOCK();
   chanstate[i].inj_tooth[0] = tooth;
   chanstate[i].inj_frac[0] = frac;
  _END_ATOMIC_BLOCK();
 }
}
#endif

#ifdef PHASE_SENSOR
void ckps_use_cam_ref_s(uint8_t i_camref)
{
 WRITEBIT(flags2, F_CAMREF, i_camref);
}
#endif

/**Interrupt handler for Compare/Match channel A of timer T1. Used for spark output, strobe and dwell time control
 */
ISR(TIMER1_COMPA_vect)
{
 TIMSK1&= ~_BV(OCIE1A); //disable this interrupt
expired:
 switch(QUEUE_TAIL(1).id) //what exactly happen?
 {
  case QID_DWELL: //start accumulation
   if (CHECKBIT(flags, F_IGNIEN)) //Does ignition enabled?
   {
    ((iocfg_pfn_set)chanstate[QUEUE_TAIL(1).ch].io_callback1)(IGNOUTCB_OFF_VAL);
#ifdef PHASED_IGNITION
    ((iocfg_pfn_set)chanstate[QUEUE_TAIL(1).ch].io_callback2)(IGNOUTCB_OFF_VAL);
#endif
   }
   break;

  case QID_SPARK: //end accumulation - spark
  {
   ((iocfg_pfn_set)chanstate[QUEUE_TAIL(1).ch].io_callback1)(IGNOUTCB_ON_VAL);
#ifdef PHASED_IGNITION
   ((iocfg_pfn_set)chanstate[QUEUE_TAIL(1).ch].io_callback2)(IGNOUTCB_ON_VAL);
#endif

#ifdef STROBOSCOPE
   if (CHECKBIT(flags, F_STROBE) && (0==QUEUE_TAIL(1).ch))
   {
    IOCFG_SET(IOP_STROBE, 1);  //start pulse
    QUEUE_REMOVE(1); // remove already processed spark event from queue here. We will skip redundant removing using goto operator
    QUEUE_ADDF(1, TCNT1, STROBE_PW, QID_STROBE); //strobe pulse is 100uS by default
    goto settmr; //skip calling QUEUE_REMOVE() at the bottom
   }
#endif
   break;
  }

  case QID_RPMSAMP:
  {
   uint16_t tmr = TCNT1;
   //save period value if it is correct
   if (CHECKBIT(flags, F_VHTPER))
   {
    ckps.stroke_period = (tmr - ckps.measure_start_value);
    WRITEBIT(flags2, F_SPSIGN, tmr < ckps.measure_start_value); //save sign
    ckps.t1oc_s = ckps.t1oc, ckps.t1oc = 0; //save value and reset counter
   }
   ckps.measure_start_value = tmr;
   SETBIT(flags, F_VHTPER);
   SETBIT(flags, F_STROKE); //set the stroke-synchronozation event
   break;
  }

  case QID_MEASURE:
   adc_begin_measure(_AB(ckps.stroke_period, 1) < 4);//start the process of measuring analog input values
   break;

  case QID_KNKBEG:
   {
   uint8_t i = QUEUE_TAIL(1).ch;
   knock_set_channel(chanstate[NEXT_CHIDX(i)].knock_chan); //set KS channel for next ignition event
   knock_set_integration_mode(KNOCK_INTMODE_INT); //start listening a detonation (opening the window)
   }
   break;

  case QID_KNKEND:
   //finish listening a detonation (closing the window) and start the process of measuring integrated value
   knock_set_integration_mode(KNOCK_INTMODE_HOLD);
   knock_start_settings_latching();//start the process of downloading the settings into the HIP9011 (and getting ADC result for TPIC8101)
#ifndef TPIC8101
   adc_begin_measure_knock(_AB(ckps.stroke_period, 1) < 4);
#endif
   break;

#ifdef STROBOSCOPE
  case QID_STROBE:
   IOCFG_SET(IOP_STROBE, 0);  //end pulse
   break;
#endif

#ifdef HALL_OUTPUT
  case QID_HOPBEG:
   IOCFG_SET(IOP_HALL_OUT, 1);
   break;

  case QID_HOPEND:
   IOCFG_SET(IOP_HALL_OUT, 0);
   break;
#endif
 }

 //Remove already processed event from queue. After that, if queue is not empty, then start
 //next event in chain. Also, prevent effects when event already expired.
 QUEUE_REMOVE(1);
settmr:
 if (!QUEUE_IS_EMPTY(1))
 {
  uint16_t t = (QUEUE_TAIL(1).end_time-(uint16_t)2) - TCNT1; //substruct a little value to avoid missing possible pre-expiring event
  if (t > EXPEVENT_THRD)             //end_time < TCNT1, so, it is expired (forbidden range is EXPEVENT_THRD...65535)
  {
   goto expired;                     //already expired
  }
  else
  {
   SET_T1COMPA(TCNT1, t);
  }
 }
}

#ifdef SPLIT_ANGLE
/** Timer 3 compare interrupt A - used for second ignition channels (angle splitting for rotary engines)*/
ISR(TIMER3_COMPA_vect)
{
 TIMSK3&= ~_BV(OCIE3A); //disable this interrupt
expired:
 switch(QUEUE_TAIL(3).id) //what exactly happen?
 {
  case QID_DWELL: //start accumulation
   if (CHECKBIT(flags, F_IGNIEN)) //Does ignition enabled?
   {
    ((iocfg_pfn_set)chanstate[QUEUE_TAIL(3).ch].io_callback1)(IGNOUTCB_OFF_VAL);
#ifdef PHASED_IGNITION
    ((iocfg_pfn_set)chanstate[QUEUE_TAIL(3).ch].io_callback2)(IGNOUTCB_OFF_VAL);
#endif
   }
   break;

  case QID_SPARK:
  {
   //line of port in the low level, now set it into a high level - makes the transistor to close and coil to stop 
   //the accumulation of energy (spark)
   ((iocfg_pfn_set)chanstate[QUEUE_TAIL(3).ch].io_callback1)(IGNOUTCB_ON_VAL);
#ifdef PHASED_IGNITION
   ((iocfg_pfn_set)chanstate[QUEUE_TAIL(3).ch].io_callback2)(IGNOUTCB_ON_VAL);
#endif
   break;
  }
 }

 //Remove already processed event from queue. After that, if queue is not empty, then start
 //next event in chain. Also, prevent effects when event already expired.
 QUEUE_REMOVE(3);
 if (!QUEUE_IS_EMPTY(3))
 {
  uint16_t t = (QUEUE_TAIL(3).end_time-(uint16_t)2) - TCNT1; //substruct a little value to avoid missing possible pre-expiring event
  if (t > EXPEVENT_THRD)             //end_time < TCNT3, so, it is expired (forbidden range is EXPEVENT_THRD...65535)
  {
   goto expired;                     //already expired
  }
  else
  {
   SET_T3COMPA(TCNT3, t);
  }
 }
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
   if (ckps.cog >= PGM_GET_WORD(&fw_data.exdata.ckps_skip_trig)) // number of teeth that wes skip at the start of cranking before synchronization
   {
#ifdef PHASE_SENSOR
    if (CHECKBIT(flags2, F_CAMREF))
     ckps.starting_mode = 1; //switch to this mode if cam reference was enabled
    else
#endif
#if defined(PHASED_IGNITION) && !defined(CAM_SYNC)
    //if cylinder number is even, then cam synchronization will be performed later
    ckps.starting_mode = (ckps.chan_number & 1) ? 1 : 2;
#else
    ckps.starting_mode = 2; //even number of cylinders only
#endif
    //note: for full sequential injection (even for odd cyl. number engines) we don't need to wait for cam sensor pulse, because we start with simultateous injection mode.
   }
   break;

#ifdef PHASE_SENSOR
  case 1: //we fall into this state only if number of cylinders is odd (in full-sequential ignition mode) or if cam sensor was selected as reference
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
#ifdef CAM_SYNC
    SETBIT(flags, F_ISSYNC);
    ckps.cog = 0; //first tooth
    return 1; //finish
#else
    if (CHECKBIT(flags2, F_CAMREF))
    {
     SETBIT(flags, F_ISSYNC);
     ckps.cog = ckps.cog360 = 0; //first tooth
     return 1; //finish
    }
    else
     ckps.starting_mode = 2;
#endif
   }
   break;
#endif

  case 2: //find out missing teeth
   //if missing teeth = 0, then reference will be identified by additional VR sensor (REF_S input)
   if ((0==ckps.miss_cogs_num) ? cams_vr_is_event_r() : (ckps.period_curr > CKPS_GAP_BARRIER(ckps.period_prev)))
   {
#ifdef CAM_SYNC
#ifdef PHASED_IGNITION
    if (CHECKBIT(flags2, F_SINGCH))
     set_channels_sc(); //single channel mode
    else
     set_channels_fs(1);
#endif
#ifdef FUEL_INJECT
    inject_set_fullsequential(1);
#endif
#endif
    SETBIT(flags, F_ISSYNC);
    ckps.period_curr = ckps.period_prev;  //exclude value of missing teeth's period
#ifdef CAM_SYNC
    ckps.cog = 0; //first tooth
#else
    ckps.cog = ckps.cog360 = 0; //first tooth
#endif
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
 uint8_t ch;
 for(ch = 0; ch < ckps.chan_number; ++ch)
 {
  //program queue for dwell event
  if (chanstate[ch].dwl_tooth[1] == ckps.cog)
  {
   uint16_t delay = FRAC_TO_TIME(chanstate[ch].dwl_frac[1]);
   QUEUE_ADD(1, ICR1, (uint16_t)delay, QID_DWELL, ch);
  }

  //program queue for spark event
  if (chanstate[ch].ign_tooth[1] == ckps.cog)
  {
   uint16_t delay = FRAC_TO_TIME(chanstate[ch].ign_frac[1]);
   QUEUE_ADD(1, ICR1, (uint16_t)delay, QID_SPARK, ch);
   //sync values
   chanstate[ch].dwl_tooth[1] = chanstate[ch].dwl_tooth[0];
   chanstate[ch].dwl_frac[1] = chanstate[ch].dwl_frac[0];
   chanstate[ch].ign_tooth[1] = chanstate[ch].ign_tooth[0];
   chanstate[ch].ign_frac[1] = chanstate[ch].ign_frac[0];
  }

  //program queue for rpm sample event
  if (chanstate[ch].rpm_tooth == ckps.cog)
  {
   uint16_t delay = FRAC_TO_TIME(chanstate[ch].rpm_frac);
   QUEUE_ADDF(1, ICR1, (uint16_t)delay, QID_RPMSAMP);
  }

  //program queue for sensors' sample event
  if (chanstate[ch].msr_tooth == ckps.cog)
  {
   uint16_t delay = FRAC_TO_TIME(chanstate[ch].msr_frac);
   QUEUE_ADDF(1, ICR1, (uint16_t)delay, QID_MEASURE);
  }

  if (CHECKBIT(flags, F_USEKNK))
  {
   //program queue 'knock window begin' event
   if (chanstate[ch].knb_tooth == ckps.cog)
   {
    uint16_t delay = FRAC_TO_TIME(chanstate[ch].knb_frac);
    QUEUE_ADDF(1, ICR1, (uint16_t)delay, QID_KNKBEG);
   }

   //program queue for 'knock window end' event
   if (chanstate[ch].kne_tooth == ckps.cog)
   {
    uint16_t delay = FRAC_TO_TIME(chanstate[ch].kne_frac);
    QUEUE_ADDF(1, ICR1, (uint16_t)delay, QID_KNKEND);
   }
  }

#ifdef HALL_OUTPUT
  if (CHECKBIT(flags, F_HALOUT))
  {
   //program queue for hall output pulse's start event
   if (chanstate[ch].hob_tooth == ckps.cog)
   {
    uint16_t delay = FRAC_TO_TIME(chanstate[ch].hob_frac);
    QUEUE_ADDF(1, ICR1, (uint16_t)delay, QID_HOPBEG);
   }

   //program queue for hall output pulse's end event
   if (chanstate[ch].hoe_tooth == ckps.cog)
   {
    uint16_t delay = FRAC_TO_TIME(chanstate[ch].hoe_frac);
    QUEUE_ADDF(1, ICR1, (uint16_t)delay, QID_HOPEND);
   }
  }
#endif

#ifdef FUEL_INJECT
  //control injection timing using teeth and COMPB timer channel
  if (chanstate[ch].inj_tooth[1] == ckps.cog)
  {
   uint16_t delay = FRAC_TO_TIME(chanstate[ch].inj_frac[1]);
   ckps.inj_chidx = ch;  //remember number of channel to be fired
   SET_T1COMPB(ICR1, delay);
   sync_inj_angle();
  }
#endif
 }

#ifdef SPLIT_ANGLE
 for(ch = SPLIT_OFFSET; ch < ckps.chan_number_split; ++ch)
 {
  //program queue for dwell event
  if (chanstate[ch].dwl_tooth[1] == ckps.cog)
  {
   uint16_t delay = FRAC_TO_TIME(chanstate[ch].dwl_frac[1]);
   QUEUE_ADD(3, ICR1, (uint16_t)delay, QID_DWELL, ch);
  }

  //program queue for spark event
  if (chanstate[ch].ign_tooth[1] == ckps.cog)
  {
   uint16_t delay = FRAC_TO_TIME(chanstate[ch].ign_frac[1]);
   QUEUE_ADD(3, ICR1, (uint16_t)delay, QID_SPARK, ch);
   //sync values
   chanstate[ch].dwl_tooth[1] = chanstate[ch].dwl_tooth[0];
   chanstate[ch].dwl_frac[1] = chanstate[ch].dwl_frac[0];
   chanstate[ch].ign_tooth[1] = chanstate[ch].ign_tooth[0];
   chanstate[ch].ign_frac[1] = chanstate[ch].ign_frac[0];
  }
 }
#endif

 ++ckps.cog;

#ifdef CAM_SYNC
#ifdef PHASE_SENSOR
 //search for level's toggle from camshaft sensor on each cog
 if (CHECKBIT(flags2, F_CAMREF))
  cams_detect_edge();
#endif
#else
#ifdef PHASE_SENSOR
 //search for level's toggle from camshaft sensor on each cog
 cams_detect_edge();
#if defined(PHASED_IGNITION) || defined(FUEL_INJECT)
 if (!CHECKBIT(flags2, F_CAMREF))
 {
  if (cams_is_event_r())
  {
   //Synchronize. We rely that cam sensor event (e.g. falling edge) coming before missing teeth
   if (ckps.cog < ckps.wheel_cogs_num)
    ckps.cog+= ckps.wheel_cogs_num;

   //Turn on full sequential mode because Cam sensor is OK
   if (!CHECKBIT(flags2, F_CAMISS))
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
    SETBIT(flags2, F_CAMISS);
   }
  }
  if (cams_is_error())
  {
   //Turn off full sequential mode because cam sensor is not OK
   if (CHECKBIT(flags2, F_CAMISS))
   {
#ifdef PHASED_IGNITION
    if (CHECKBIT(flags2, F_SINGCH))
     set_channels_sc(); //single channel mode
    else
     set_channels_fs(0); //<--valid only for engines with even number of cylinders
#endif
#ifdef FUEL_INJECT
    inject_set_fullsequential(0);
#endif
    CLEARBIT(flags2, F_CAMISS);
   }
  }
 }
#endif
#endif
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
   sync_ign_angle();
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
#ifndef CAM_SYNC
  if ((ckps.cog360 == ckps.wheel_cogs_num))
   ckps.cog360 = 0;
#endif
  if (cams_is_event_r())
  {
#ifdef CAM_SYNC
   if (ckps.cog != ckps.wheel_cogs_num)
#else
   if (ckps.cog != ckps.wheel_cogs_num2) //check if sync is correct
#endif
    SETBIT(flags, F_ERROR); //ERROR
   ckps.cog = 0; //each 720°
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
#ifdef CAM_SYNC
   if ((ckps.cog != ckps.wheel_cogs_num)) //also taking into account recovered teeth
    SETBIT(flags, F_ERROR); //ERROR
   ckps.cog = 0;
   ckps.period_curr = ckps.period_prev;  //exclude value of missing teeth's period (for missing teeth only)
#else
   if ((ckps.cog360 != ckps.wheel_cogs_num)) //also taking into account recovered teeth
   {
    SETBIT(flags, F_ERROR); //ERROR
    ckps.cog = 0;
    //TODO: maybe we need to turn off full sequential mode
   }
   //Reset 360° tooth counter to the first tooth
   ckps.cog360 = 0;
   //Also reset 720° tooth counter
   if (ckps.cog == ckps.wheel_cogs_num2)
    ckps.cog = 0;
   ckps.period_curr = ckps.period_prev;  //exclude value of missing teeth's period (for missing teeth only)
#endif
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
#ifdef CAM_SYNC
  if (ckps.miss_cogs_num && ckps.cog == ckps.wheel_last_cog)
#else
  if (ckps.miss_cogs_num && ckps.cog360 == ckps.wheel_last_cog)
#endif
   set_timer0(ckps.period_curr);
 }

 //call handler for normal teeth
 process_ckps_cogs();
#ifndef CAM_SYNC
 ++ckps.cog360;
#endif
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

  //start timer to recover 60th tooth
#ifdef CAM_SYNC
  if (ckps.cog != ckps.wheel_cogs_numm1)
#else
  if (ckps.cog360 != ckps.wheel_cogs_numm1)
#endif
  set_timer0(ckps.period_curr);

  //Call handler for missing teeth
  process_ckps_cogs();
#ifndef CAM_SYNC
  ++ckps.cog360;
#endif
 }
}

/** Timer 1 overflow interrupt.
 * Used to count timer 1 overflows to obtain correct revolution period at very low RPMs (10...400 min-1)
 */
ISR(TIMER1_OVF_vect)
{
 ++ckps.t1oc;
}

void ckps_set_knock_chanmap(uint8_t chanmap)
{
 for(uint8_t i = 0; i < ckps.chan_number; ++i)
  chanstate[i].knock_chan = !!CHECKBIT(chanmap, i);
}

void ckps_set_mttf(uint16_t mttf)
{
 _BEGIN_ATOMIC_BLOCK();
 ckps.mttf = mttf;
 _END_ATOMIC_BLOCK();
}

#endif //ODDFIRE_ALGO
