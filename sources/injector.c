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

/** \file injector.c
 * \author Alexey A. Shabelnikov
 * Implementation of fuel injectors' control
 */

#ifdef FUEL_INJECT

#include "port/port.h"
#include "port/avrio.h"
#include "port/interrupt.h"
#include "port/intrinsic.h"
#include "port/pgmspace.h"
#include "camsens.h"
#include "bitmask.h"
#include "injector.h"
#include "ioconfig.h"
#include "tables.h"
#include "ecudata.h"
#include "mathemat.h"
#include <stdlib.h>

#ifndef AIRTEMP_SENS
 #error "You can not use FUEL_INJECT option without AIRTEMP_SENS"
#endif

#ifdef CARB_AFR
 #error "You can not use FUEL_INJECT option together with CARB_AFR"
#endif

#ifdef SECU3T
 #define INJ_ON  0   //!< Injector is turned on
 #define INJ_OFF 1   //!< Injector is turned off
#else //SECU-3i
 #define INJ_ON  1   //!< Injector is turned on
 #define INJ_OFF 0   //!< Injector is turned off
#endif

//from ckps.c
uint16_t ckps_get_stroke_period(void);

/**COMPB interrupt calibration*/
#define INJ_COMPB_CALIB 2

/**Maximum queue size per timer channel */
#define INJ_CH_QUEUE_SIZE 4

/** Define injector state variables structure*/
typedef struct
{
 volatile uint8_t tmr2b_h;       //!< used in timer2 COMPB interrupt to perform 16-bit timing
 volatile uint8_t tmr0b_h;       //!< used in timer0 COMPB interrupt to perform 16-bit timing
 volatile uint8_t cyl_number;    //!< number of engine cylinders
 uint8_t  num_squirts;           //!< number of squirts per cycle
 volatile uint8_t fuelcut;       //!< fuelcut flag
 volatile uint8_t prime_pulse;   //!< prime pulse count (multiplier)
 volatile uint16_t prime_time;   //!< prime pulse duration
 volatile uint8_t cfg;           //!< injection configuration
 uint8_t  tmr_chan;              //!< number of current timer channel
 uint8_t  squirt_mask;           //!< squirt mask (see calc_squirt_mask() function)

 //See inj_eq1 and inj_eq2 global variables
 volatile uint8_t eq_tail1;      //!< event queue tail (index in a static array)
 volatile uint8_t eq_head1;      //!< event queue head (index), queue is empty if head = tail
 volatile uint8_t eq_tail2;      //!< event queue tail (index in a static array)
 volatile uint8_t eq_head2;      //!< event queue head (index), queue is empty if head = tail

 volatile uint8_t active_chan;   //!< active channels
 volatile uint8_t mask_chan;     //!< for masking of channels

 volatile uint8_t shrinktime;    //!< flag, indicates that injection time should be reduced: 0 - no changes, 1 - 2 times, 2 - N times (N = number of cylinders)
 volatile uint8_t rowswt_add;    //!< value being added to output index for switching to second inj. row
 volatile uint8_t rowmod;        //!< flag, indicates inj. row switching mode: 0 - normal, 1 - first row, 2 - second row
}inj_state_t;

/**Describes injector channels*/
typedef struct
{
 /**Address of callback which will be used for settiong of I/O */
 volatile fnptr_t io_callback1;  //!< callback pointer (function which sets corresponding I/O)
 volatile fnptr_t io_callback2;  //!< second callback to allow semi-sequential mode with separate outputs
 uint8_t io_map;                 //!< for mapping of channel number to a particular I/O
 volatile uint16_t inj_time;     //!< Current injection time, used in interrupts
}inj_chanstate_t;


/**Describes event queue entry*/
typedef struct
{
 uint16_t end_time;              //!< End time of event in ticks of free running timer 1
 uint8_t  chan_idx;              //!< Associated injection channel number which will be turned off
}inj_queue_t;

/** Global instance of injector state variable structure*/
inj_state_t inj = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

/** I/O information for each channel */
inj_chanstate_t inj_chanstate[INJ_CHANNELS_MAX];

/**Event queue for scheduling of injection outputs control (1-st timer channel) */
inj_queue_t inj_eq1[INJ_CH_QUEUE_SIZE];

/**Event queue for scheduling of injection outputs control (2-nd timer channel) */
inj_queue_t inj_eq2[INJ_CH_QUEUE_SIZE];

/** Add event into the queue (add to head) */
#define QUEUE_ADD(q, time, chan) \
    inj_eq##q[inj.eq_head##q].end_time = TCNT1 + (time); \
    inj_eq##q[inj.eq_head##q].chan_idx = (chan); \
    if (++inj.eq_head##q >= INJ_CH_QUEUE_SIZE) \
     inj.eq_head##q = 0;

/** Remove event from the queue (remove from tail)*/
#define QUEUE_REMOVE(q) \
   if (++inj.eq_tail##q >= INJ_CH_QUEUE_SIZE) \
    inj.eq_tail##q = 0;

/** Test is queue empty */
#define QUEUE_IS_EMPTY(q) (inj.eq_head##q==inj.eq_tail##q)

/** Get tail item value from queue */
#define QUEUE_TAIL(q) inj_eq##q[inj.eq_tail##q]

/** Reset specified queue */
#define QUEUE_RESET(q)  inj.eq_tail##q = inj.eq_head##q = 0;


/** Get value of I/O callback by index. This function is necessary for supporting of 5,6 inj. channels for SECU-3T and 6,7,8 inj.channels for SECU-3i
 * \param index Index of callback */
static inline fnptr_t get_callback_inj(uint8_t index)
{
#ifdef SECU3T
 return (index < (IOP_INJ_OUT4+1)) ? IOCFG_CB(index) : IOCFG_CB(index + IOP_INJPLG_OFF);
#else //SECU-3i
 return (index < (IOP_INJ_OUT5+1)) ? IOCFG_CB(index) : IOCFG_CB(index + IOP_INJPLG_OFF);
#endif
}

/**Tune channels' I/O for semi-sequential mode with separate channels or full sequential injection mode 
 * \param fs_mode 0 - semi-sequential, 1 - full sequential
 */
static void set_channels_fs(uint8_t fs_mode)
{
 uint8_t _t, i = 0, ch = 0, ch2 = fs_mode ? 0 : inj.cyl_number / 2, iss;
 uint8_t idxNum = (inj.num_squirts == inj.cyl_number && !fs_mode) ? ch2 : inj.cyl_number;

 //set flag indicating that injection time must be reduced by 2 or N times (depending on number of cylinders)
 if (inj.cfg == INJCFG_FULLSEQUENTIAL && !fs_mode)
  inj.shrinktime = (inj.cyl_number & 1) ? 2 : 1;
 else
  inj.shrinktime = 0;

 for(; i < inj.cyl_number; ++i)
 {
  iss = (ch + ch2);
  if (iss >= inj.cyl_number)
   iss-=inj.cyl_number;

  _t=_SAVE_INTERRUPT();
  _DISABLE_INTERRUPT();
  if (CHECKBIT(inj.squirt_mask, i)) {
   inj_chanstate[i].io_callback1 = get_callback_inj(IOP_INJ_OUT1 + inj.rowswt_add + ch);
   inj_chanstate[i].io_callback2 = get_callback_inj(IOP_INJ_OUT1 + inj.rowswt_add + iss);
   inj_chanstate[i].io_map = i;
   ++ch;
  }
  _RESTORE_INTERRUPT(_t);
  if (ch >= idxNum)
   ch = 0;
 }

 //turn off unused channels
 idxNum = inj.rowmod ? INJ_CHANNELS_MAX / 2 : INJ_CHANNELS_MAX;
 for(i =  inj.cyl_number; i < idxNum; ++i)
  ((iocfg_pfn_set)get_callback_inj(IOP_INJ_OUT1 + inj.rowswt_add + i))(INJ_OFF);
}

/**Tune channels' I/O for semi-sequential or 2 banks alternating injection mode
 * \param _2bnk 0 - set to semi-sequential mode, 1 - set to 2 banks alternating mode
 */
static void set_channels_ss(uint8_t _2bnk)
{
 inj.shrinktime = 0; //not used by this mode
 uint8_t idxNum = (inj.num_squirts == inj.cyl_number) ? inj.cyl_number / 2 : inj.cyl_number;
 uint8_t _t, i = 0, ch = 0;
 for(; i < inj.cyl_number; ++i)
 {
  fnptr_t value = get_callback_inj(IOP_INJ_OUT1 + inj.rowswt_add + ch);
  _t=_SAVE_INTERRUPT();
  _DISABLE_INTERRUPT();
  if (CHECKBIT(inj.squirt_mask, i)) {
   inj_chanstate[i].io_callback1 = value;
   inj_chanstate[i].io_callback2 = value;
   inj_chanstate[i].io_map = i;
   ch = _2bnk ? ch ^ 1 : ch + 1;
  }
  _RESTORE_INTERRUPT(_t);
  if (ch >= idxNum)
   ch = 0;
 }

 //turn off unused channels
 idxNum = inj.rowmod ? INJ_CHANNELS_MAX / 2 : INJ_CHANNELS_MAX;
 for(i = _2bnk ? 2 : (inj.cyl_number / 2); i < idxNum; ++i)
  ((iocfg_pfn_set)get_callback_inj(IOP_INJ_OUT1 + inj.rowswt_add + i))(INJ_OFF);
}

void inject_init_state(void)
{
 inj.cfg = INJCFG_THROTTLEBODY;
 for(uint8_t i = 0; i < INJ_CHANNELS_MAX; ++i)
  inj_chanstate[i].inj_time = 0xFFFF;
 inj.fuelcut = 1;  //no fuel cut
 inj.prime_pulse = 0; //no prime pulse
 inj.prime_time = 0;
 inj.tmr_chan = 0;
 inj.active_chan = 0;
 inj.mask_chan = 0xFF;
 inj.shrinktime = 0;
 QUEUE_RESET(1);   //head = tail
 QUEUE_RESET(2);   //head = tail
}

#ifdef SECU3T
/*Turn on/off all injectors **/
#define SET_ALL_INJ(state) \
   IOCFG_SET(IOP_INJ_OUT1, (state));           /*injector 1 */ \
   IOCFG_SET(IOP_INJ_OUT2, (state));           /*injector 2 */ \
   IOCFG_SET(IOP_INJ_OUT3, (state));           /*injector 3 */ \
   IOCFG_SET(IOP_INJ_OUT4, (state));           /*injector 4 */ \
   IOCFG_SET(IOP_INJ_OUT5, (state));           /*injector 5 */ \
   IOCFG_SET(IOP_INJ_OUT6, (state));           /*injector 6 */
#else //SECU-3i
/*Turn on/off all injectors **/
#define SET_ALL_INJ(state) \
  if (0==inj.rowmod) \
  { \
   IOCFG_SET(IOP_INJ_OUT1, (state));           /*injector 1 */ \
   IOCFG_SET(IOP_INJ_OUT2, (state));           /*injector 2 */ \
   IOCFG_SET(IOP_INJ_OUT3, (state));           /*injector 3 */ \
   IOCFG_SET(IOP_INJ_OUT4, (state));           /*injector 4 */ \
   IOCFG_SET(IOP_INJ_OUT5, (state));           /*injector 5 */ \
   IOCFG_SET(IOP_INJ_OUT6, (state));           /*injector 6 */ \
   IOCFG_SET(IOP_INJ_OUT7, (state));           /*injector 7 */ \
   IOCFG_SET(IOP_INJ_OUT8, (state));           /*injector 8 */ \
  } \
  else if (1==inj.rowmod) \
  { \
   IOCFG_SET(IOP_INJ_OUT1, (state));           /*injector 1 */ \
   IOCFG_SET(IOP_INJ_OUT2, (state));           /*injector 2 */ \
   IOCFG_SET(IOP_INJ_OUT3, (state));           /*injector 3 */ \
   IOCFG_SET(IOP_INJ_OUT4, (state));           /*injector 4 */ \
  } \
  else \
  { \
   IOCFG_SET(IOP_INJ_OUT5, (state));           /*injector 5 */ \
   IOCFG_SET(IOP_INJ_OUT6, (state));           /*injector 6 */ \
   IOCFG_SET(IOP_INJ_OUT7, (state));           /*injector 7 */ \
   IOCFG_SET(IOP_INJ_OUT8, (state));           /*injector 8 */ \
  }
#endif

void inject_init_ports(void)
{
 IOCFG_INIT(IOP_INJ_OUT1, INJ_OFF);           //injector 1 is turned off
 IOCFG_INIT(IOP_INJ_OUT2, INJ_OFF);           //injector 2 is turned off
 IOCFG_INIT(IOP_INJ_OUT3, INJ_OFF);           //injector 3 is turned off
 IOCFG_INIT(IOP_INJ_OUT4, INJ_OFF);           //injector 4 is turned off
 IOCFG_INIT(IOP_INJ_OUT5, INJ_OFF);           //injector 5 is turned off
 IOCFG_INIT(IOP_INJ_OUT6, INJ_OFF);           //injector 6 is turned off
#ifndef SECU3T //only in SECU-3i
 IOCFG_INIT(IOP_INJ_OUT7, INJ_OFF);           //injector 7 is turned off
 IOCFG_INIT(IOP_INJ_OUT8, INJ_OFF);           //injector 8 is turned off
#endif
}

/** Updates squirt mask */
static void calc_squirt_mask(void)
{
 uint8_t sqr_cyl = 0, i = 0, cyl_inc = inj.cyl_number / inj.num_squirts;
 inj.squirt_mask = 0;
 for(; i < inj.cyl_number; ++i)
 {
  if (sqr_cyl == i)
  {
   SETBIT(inj.squirt_mask, i);
   sqr_cyl+= cyl_inc;
   if (sqr_cyl >= inj.cyl_number)
    sqr_cyl = 0;
  }
 }
}

uint8_t inject_set_cyl_number(uint8_t cylnum)
{
 uint8_t cylprev = inj.cyl_number;
 _BEGIN_ATOMIC_BLOCK();
 inj.cyl_number = cylnum;
 calc_squirt_mask();                          //update squirt mask
 inject_set_config(inj.cfg, inj.rowmod);      //update inj. config
 _END_ATOMIC_BLOCK();
 return cylprev;
}

void inject_set_num_squirts(uint8_t numsqr)
{
 _BEGIN_ATOMIC_BLOCK();
 inj.num_squirts = numsqr;                    //save number of squirts per cycle
 calc_squirt_mask();                          //update squirt mask
 _END_ATOMIC_BLOCK();
}

void inject_set_inj_time(void)
{ //Note, input values are not allowed to be close to zero!
 uint16_t time;
 uint16_t volatile *p_time = (inj.shrinktime == 0) ? &d.inj_pwns[0][0] : &d.inj_pwns[1][0];
 uint8_t ch = inj.rowswt_add;
 for (uint8_t i = 0; i < inj.cyl_number; ++i)
 {
  time = ((p_time[ch]) >> 1) - INJ_COMPB_CALIB;//obtain PW for channel and subtract calibration ticks
  if (0==_AB(time, 0))                         //avoid strange bug which appears when OCR2B is set to the same value as TCNT2
   (_AB(time, 0))++;

  _BEGIN_ATOMIC_BLOCK();
  inj_chanstate[i].inj_time = time;
  _END_ATOMIC_BLOCK();
  ++ch;
 }
}

void inject_set_fuelcut(uint8_t state)
{
 inj.fuelcut = state;
}

void inject_set_config(uint8_t cfg, uint8_t irs)
{
 inj.cfg = cfg;
#ifndef SECU3T //only in SECU-3i
 if (irs)
 {
  if (!d.sens.gas)
  {
   inj.rowswt_add = 0;
   inj.rowmod = 1;  //1st row
  }
  else
  {
   inj.rowswt_add = 4;
   inj.rowmod = 2; //2nd row
  }
 }
 else
 {
  inj.rowswt_add = 0;
  inj.rowmod = 0; //normal mode
 }
#endif
 if (cfg == INJCFG_2BANK_ALTERN)
  set_channels_ss(1);                               //2 banks, alternating
 else if (cfg == INJCFG_SEMISEQUENTIAL)
  set_channels_ss(0);                               //semi-sequential mode
 else if (cfg == INJCFG_SEMISEQSEPAR)
  set_channels_fs(0);                               //semi-sequential with separate channels
#ifdef PHASE_SENSOR
 else if (cfg == INJCFG_FULLSEQUENTIAL)
  set_channels_fs(
#ifdef CAM_SYNC
  1
#else
  cams_is_ready()
#endif
  );                 //full sequential
#endif
 else
 {
  inj.shrinktime = 0; //because shrinking of inj. time is not used by simultaneous and cetral injection modes
 }

#ifndef SECU3T //only in SECU-3i
 if (1==inj.rowmod)
 {
  IOCFG_SET(IOP_INJ_OUT5, INJ_OFF);           /*injector 5 */
  IOCFG_SET(IOP_INJ_OUT6, INJ_OFF);           /*injector 6 */
  IOCFG_SET(IOP_INJ_OUT7, INJ_OFF);           /*injector 7 */
  IOCFG_SET(IOP_INJ_OUT8, INJ_OFF);           /*injector 8 */
 }
 else if (2==inj.rowmod)
 {
  IOCFG_SET(IOP_INJ_OUT1, INJ_OFF);           /*injector 1 */
  IOCFG_SET(IOP_INJ_OUT2, INJ_OFF);           /*injector 2 */
  IOCFG_SET(IOP_INJ_OUT3, INJ_OFF);           /*injector 3 */
  IOCFG_SET(IOP_INJ_OUT4, INJ_OFF);           /*injector 4 */
 }
#endif
}

void inject_start_inj(uint8_t chan)
{
 if (!inj.fuelcut || inj.prime_pulse)
  return; //fuel is OFF or prime pulse is active

 if (CHECKBIT(inj.squirt_mask, chan))
 {
  if (inj.cfg < INJCFG_2BANK_ALTERN || inj.shrinktime == 2)
  {//central/simultaneous, We use only one timer channel
   //interrupts must be disabled!
   _BEGIN_ATOMIC_BLOCK();
   OCR2B = TCNT2 + _AB(inj_chanstate[0].inj_time, 0);
   SETBIT(TIFR2, OCF2B);                      //reset possible pending interrupt flag
   inj.tmr2b_h = _AB(inj_chanstate[0].inj_time, 1);
   SET_ALL_INJ(INJ_ON);                       //turn on injector 1-8
   SETBIT(TIMSK2, OCIE2B);
   _END_ATOMIC_BLOCK();
  }
  else
  {//semi-sequential
   if (!inj.tmr_chan)
   { //use 1-st timer channel
    _BEGIN_ATOMIC_BLOCK();
    ((iocfg_pfn_set)inj_chanstate[chan].io_callback1)(INJ_ON);//turn on current injector pair
    ((iocfg_pfn_set)inj_chanstate[chan].io_callback2)(INJ_ON);

    if (CHECKBIT(inj.active_chan, inj_chanstate[chan].io_map))
     CLEARBIT(inj.mask_chan, inj_chanstate[chan].io_map); //mask it if it is still active

    SETBIT(inj.active_chan, inj_chanstate[chan].io_map);
    uint16_t inj_time = inj_chanstate[chan].inj_time;
    if (QUEUE_IS_EMPTY(1))
    {
     OCR2B = TCNT2 + _AB(inj_time, 0);
     SETBIT(TIFR2, OCF2B);                    //reset possible pending interrupt flag
     inj.tmr2b_h = _AB(inj_time, 1);
     SETBIT(TIMSK2, OCIE2B);
     SETBIT(inj.mask_chan, inj_chanstate[chan].io_map); //unmask channel
    }
    QUEUE_ADD(1, (inj_time << 1), chan);  //append queue by channel requiring processing
    _END_ATOMIC_BLOCK();
    ++inj.tmr_chan;                           //next channel
   }
   else
   { //use 2-nd timer channel
    uint16_t t = (inj_chanstate[chan].inj_time) << 1;          //this timer has 1 tick = 3.2uS
    _BEGIN_ATOMIC_BLOCK();
    ((iocfg_pfn_set)inj_chanstate[chan].io_callback1)(INJ_ON); //turn on current injector pair
    ((iocfg_pfn_set)inj_chanstate[chan].io_callback2)(INJ_ON);

    if (CHECKBIT(inj.active_chan, inj_chanstate[chan].io_map))
     CLEARBIT(inj.mask_chan, inj_chanstate[chan].io_map); //mask it if it is still active

    SETBIT(inj.active_chan, inj_chanstate[chan].io_map);
    if (QUEUE_IS_EMPTY(2))
    {
     OCR0B = TCNT0 + _AB(t, 0);
     SETBIT(TIFR0, OCF0B);                    //reset possible pending interrupt flag
     inj.tmr0b_h = _AB(t, 1);
     SETBIT(TIMSK0, OCIE0B);
     SETBIT(inj.mask_chan, inj_chanstate[chan].io_map); //unmask channel
    }
    QUEUE_ADD(2, t, chan);                    //append queue by channel requiring processing
    _END_ATOMIC_BLOCK();
    inj.tmr_chan = 0;
   }
  }
 }
}

void inject_open_inj(uint16_t time, uint8_t times)
{
  if (0==time || 0==times || !inj.fuelcut) return;
  time = (time >> 1) - INJ_COMPB_CALIB;
  if (0==_AB(time, 0))
   (_AB(time, 0))++;
  inj.prime_time = time;
  _BEGIN_ATOMIC_BLOCK();
  OCR2B = TCNT2 + _AB(time, 0);
  SETBIT(TIFR2, OCF2B);                       //reset possible pending interrupt flag
  inj.tmr2b_h = _AB(time, 1);
  SET_ALL_INJ(INJ_ON);                        //turn on injector 1-8
  SETBIT(TIMSK2, OCIE2B);
  inj.prime_pulse = times;
  _END_ATOMIC_BLOCK();
}

/**Interrupt for controlling of first injector*/
ISR(TIMER2_COMPB_vect)
{
 if (inj.tmr2b_h)
 {
  --inj.tmr2b_h;
 }
 else
 {
  if (inj.cfg < INJCFG_2BANK_ALTERN || inj.prime_pulse || inj.shrinktime == 2)
  {//central/simultaneous
   if (0==inj.prime_pulse)
   {
    SET_ALL_INJ(INJ_OFF);                      //turn off injector 1-8
    CLEARBIT(TIMSK2, OCIE2B);                  //disable this interrupt
   }
   else
   { //continue prime pulse
    if (--inj.prime_pulse)
    {
     OCR2B = TCNT2 + _AB(inj.prime_time, 0);
     SETBIT(TIFR2, OCF2B);                      //reset possible pending interrupt flag
     inj.tmr2b_h = _AB(inj.prime_time, 1);
    }
    else
    {
     SET_ALL_INJ(INJ_OFF);                      //turn off injector 1-8
     CLEARBIT(TIMSK2, OCIE2B);                  //disable this interrupt
    }
   }
  }
  else
  { //semi-sequential
   uint8_t chan_idx = QUEUE_TAIL(1).chan_idx;
   //Do not turn off channel if it is masked
   if (CHECKBIT(inj.mask_chan, inj_chanstate[chan_idx].io_map)) {
    ((iocfg_pfn_set)inj_chanstate[chan_idx].io_callback1)(INJ_OFF); //turn off current injector pair
    ((iocfg_pfn_set)inj_chanstate[chan_idx].io_callback2)(INJ_OFF);
    CLEARBIT(inj.active_chan, inj_chanstate[chan_idx].io_map);
   }
   SETBIT(inj.mask_chan, inj_chanstate[chan_idx].io_map); //unmask

   QUEUE_REMOVE(1);
   if (!QUEUE_IS_EMPTY(1))
   {
    uint16_t t = QUEUE_TAIL(1).end_time - TCNT1;
    if (t > 20000) //end_time < TCNT1   TODO: How can we do this check better?
     t = 4;
    t = t >> 1; //1 tick = 6.4us

    uint8_t TCNT2_L = _AB(t, 0);
    if (!TCNT2_L)
     TCNT2_L++;
    OCR2B = TCNT2 + TCNT2_L;
    SETBIT(TIFR2, OCF2B);                     //reset possible pending interrupt flag
    inj.tmr2b_h = _AB(t, 1);
    SETBIT(TIMSK2, OCIE2B);
   }
   else
   {
    CLEARBIT(TIMSK2, OCIE2B);                 //disable this interrupt
   }
  }
 }
}

//todo: we can try to swap TIMER2_COMPA_vect and TIMER0_COMPA_vect, so TIMER2_COMPx_vect will be used for injectors
/**Interrupt for controlling of second injector*/
ISR(TIMER0_COMPB_vect)
{
 if (inj.tmr0b_h)
 {
  --inj.tmr0b_h;
 }
 else
 {
  if (inj.cfg < INJCFG_2BANK_ALTERN)
  { //central/simultaneous
  }
  else
  { //semi-sequential
   uint8_t chan_idx = QUEUE_TAIL(2).chan_idx;
   //Do not turn off channel if it is masked
   if (CHECKBIT(inj.mask_chan, inj_chanstate[chan_idx].io_map)) {
    ((iocfg_pfn_set)inj_chanstate[chan_idx].io_callback1)(INJ_OFF); //turn off current injector pair
    ((iocfg_pfn_set)inj_chanstate[chan_idx].io_callback2)(INJ_OFF);
    CLEARBIT(inj.active_chan, inj_chanstate[chan_idx].io_map);
   }
   SETBIT(inj.mask_chan, inj_chanstate[chan_idx].io_map); //unmask

   QUEUE_REMOVE(2);
   if (!QUEUE_IS_EMPTY(2))
   {
    uint16_t t = QUEUE_TAIL(2).end_time - TCNT1;
    if (t > 20000) //end_time < TCNT1, so, it is expired  TODO: How can we do this check better?
     t = 2;

    uint8_t TCNT0_L = _AB(t, 0);
    if (!TCNT0_L)
     TCNT0_L++;
    OCR0B = TCNT0 + TCNT0_L;
    SETBIT(TIFR0, OCF0B);                     //reset possible pending interrupt flag
    inj.tmr0b_h = _AB(t, 1);
    SETBIT(TIMSK0, OCIE0B);
   }
   else
   {
    CLEARBIT(TIMSK0, OCIE0B);                 //disable this interrupt
   }
  }
 }
}

void inject_set_fullsequential(uint8_t mode)
{
 if (inj.cfg == INJCFG_FULLSEQUENTIAL)
 {
  set_channels_fs(mode);
  inject_set_inj_time();
 }
}

void inject_calc_fuel_flow(void)
{
 //TODO: limit inj.PW if it exceed period between successive injections (case when injector(s) is always turned on)
 //TODO: How to take into account prime pulse injection?
 //TODO: maybe we need to average inj_fff?

 //stroke period measured between spark strokes (not each stroke). So, for 1 cyl. engine it is 2 revolutions, for 2 cylinder engine it is 1 revolution and so on.
 uint32_t cycleper = ((uint32_t)ckps_get_stroke_period()) * d.param.ckps_engine_cyl;

 //calculate value of Nsi variable (Nsi - number of simultaneously working injectors)
 uint8_t Nsi = d.param.ckps_engine_cyl; //for [simultaneous] or [full sequential without cam sensor on the odd-cylinder num. engines]
 if (inj.cfg == INJCFG_THROTTLEBODY)
  Nsi = 1; //only single injector works simultaneously
 else if (inj.cfg == INJCFG_2BANK_ALTERN)
  Nsi = Nsi >> 1; // = Ncyl / 2
 else if (inj.cfg == INJCFG_SEMISEQUENTIAL || inj.cfg == INJCFG_SEMISEQSEPAR)
  Nsi = 2;       //two injectors work simultaneously
 else if (inj.cfg == INJCFG_FULLSEQUENTIAL)
 {
  if (inj.shrinktime == 0)
   Nsi = 1; //normal full sequential
  else if (inj.shrinktime == 1)
   Nsi = 2; //even-cylinder num. engines, like for semi-sequential (full sequential without cam sensor - failure mode)
  //p.s. we don't change default value of Nsi (set at the top) if shrinktime = 2
 }

 int32_t inj_time_raw = ((int32_t)d.inj_pw) - d.inj_dt;
 if (inj_time_raw < 0)
  inj_time_raw = 0;

 //Formula: frq = Ifr * Kduty * k, Kduty = (PW * Nsqr * Nsi) / Tc;
 // frq - frequency in Hz
 // Ifr - injector flow rate in cc/min
 // Kduty - Koefficient of duty
 // k - constant, which is equal to 16000/(1000*60)
 // PW - current injection PW, excluding dead time
 // Nsqr - number of squirts per cycle
 // Nsi - Number of simultaneously working injectors
 // Tc - current period of engine cycle

 //calculate and save result
 //inj_flow_rate * 64, fff_const * 65536, result must be * 256
 if (inj.fuelcut && d.eng_running)
 {
  uint32_t fff = (((((((uint32_t)inj_time_raw) * d.param.inj_flow_rate[d.sens.gas]) / cycleper) * inj.num_squirts) * Nsi) * d.param.fff_const) >> 14;
  if (fff > 65535)
   fff = 65535; //255 Hz maximum (prevent overflow of 16 bit variable)
  d.inj_fff = fff;
 }
 else
  d.inj_fff = 0; //no flow of fuel, because injector(s) are turned off
}

uint8_t inject_is_shrinked(void)
{
 return inj.shrinktime > 0;
}

uint8_t inject_calc_duty(void)
{
 //stroke period measured between spark strokes (not each stroke). So, for 1 cyl. engine it is 2 revolutions, for 2 cylinder engine it is 1 revolution and so on.
 uint32_t cycleper = ((uint32_t)ckps_get_stroke_period()) * d.param.ckps_engine_cyl;

 uint8_t num_squirts = 1;

 uint8_t stmul = 1;
 if (inj.cfg == INJCFG_FULLSEQUENTIAL)
 {
  if (inj.shrinktime == 0)
   stmul = 1; //normal full sequential
  else if (inj.shrinktime == 1)
   stmul = 2; //even-cylinder num. engines, like for semi-sequential (full sequential without cam sensor - failure mode)
  else
   stmul = d.param.ckps_engine_cyl; //odd-cylinder num. engines: N times
 }
 else if (inj.cfg == INJCFG_2BANK_ALTERN)
  num_squirts = inj.num_squirts / 2;
 else if ((inj.cfg == INJCFG_SEMISEQUENTIAL) || (inj.cfg == INJCFG_SEMISEQSEPAR))
  num_squirts = inj.num_squirts / (d.param.ckps_engine_cyl >> 1);
 else //Throttle body or simultaneous
  num_squirts = inj.num_squirts;

 uint16_t duty = (((uint32_t)d.inj_pw) * num_squirts * stmul * 200) / cycleper;
 if (duty > 200)
  duty = 200;  //limit to 100%

 return duty;
}

void inject_calc_fuel_cons(void)
{
 static uint16_t TCNT1_p = 0;
 uint16_t TCNT1_c = TCNT1;

 uint16_t dtcnt = TCNT1_c - TCNT1_p;
 if (dtcnt > 39062) //125ms (125000/3.2)
 {
  TCNT1_p = TCNT1_c;

  //dtcnt - time near to 125ms, but not exactly equal because this function is called from the main loop. So, we will compensate
  //this difference in following calculations

  //Given:
  //fff_const = N / (1000 * 60) * 65536, where N - number of pulses per 1L of burnt fuel
  //fuel consumption (L/h) = (3600 * fhz) / N, where fhz - frequency in Hz of the fuel consumption signal.
  //Then:
  //N = (fff_const * (1000 * 60)) / 65536;
  //fuel consumption (L/h) = (3600 * fhz) / ((fff_const * (1000 * 60)) / 65536) =  (3600 * fhz * 65536) / (fff_const * (1000 * 60));

  //Calculating consumption of fuel:
  //inj_fff = frequency in Hz * 256;
  //L per hour * 256 = (3600 * inj_fff * 65536) / (fff_const * 1000 * 60);
  //L per second  * 256 = (inj_fff * 65536) / (fff_const * 1000 * 60) = (inj_fff / fff_const)  * (65536 / (1000 * 60));

  //We need to get result in L/0.125s * 2^27, then:
  //L per second * 2^8 * 2^3 * 2^16 = (inj_fff * 65536) / fff_const; note, we have omitted (65536 / (1000 * 60)) multiplier
  //2^3 appeared because we switched to use L/0.125s units from L/s units.

  uint32_t fc_add = ((uint32_t)d.inj_fff*65536) / d.param.fff_const;

  //time correction: dcnt/39062 * 65536/(1000*60) = (dcnt* 1,092266667)/39062 = dcnt/35763

  //divide by 4 to avoid overflow, also 35763 / 4 = 8941
  dtcnt = dtcnt >> 2;

  //apply time correction combined with 65536/(1000*60) multiplier
  //as a result fc_add is fuel consumed by 0.125s, value in L * 2^27
  fc_add = (fc_add * dtcnt) / 8941;

  //10000L max.
  if (d.cons_fuel >= 10000UL*1024UL)
  {
   d.cons_fuel_int = 0;
   d.cons_fuel_imm = 0;
  }

  //accumulate to intermidiate variable
  d.cons_fuel_imm+= fc_add;

  //calculate value for user (this valie is sent to PC)
  fc_add = (d.cons_fuel_imm >> 9);
  d.cons_fuel = (d.cons_fuel_int + fc_add) >> 8;

  //accumulate intermidiate value in the long term variable
  if (d.cons_fuel_imm & 0x80000000)
  {
   d.cons_fuel_int+= fc_add;
   d.cons_fuel_imm = 0;
  }
 }
}

#endif
