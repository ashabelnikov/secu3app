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
 * Implementation of fuel injector control
 * (Реализация управления топливной форсункой).
 */

#ifdef FUEL_INJECT

#include "port/port.h"
#include "port/avrio.h"
#include "port/interrupt.h"
#include "port/intrinsic.h"
#include "port/pgmspace.h"
#include "bitmask.h"
#include "injector.h"
#include "ioconfig.h"
#include "tables.h"

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


/**COMPB interrupt calibration*/
#define INJ_COMPB_CALIB 2

/**Maximum number of injection channels */
#define INJ_CHANNELS_MAX 8

/**Maximum queue size per timer channel */
#define INJ_CH_QUEUE_SIZE 4

/** Define injector state variables structure*/
typedef struct
{
 volatile uint16_t inj_time;     //!< Current injection time, used in interrupts
 volatile uint8_t tmr2b_h;       //!< used in timer2 COMPB interrupt to perform 16-bit timing
 volatile uint8_t tmr0b_h;       //!< used in timer0 COMPB interrupt to perform 16-bit timing
 volatile uint8_t cyl_number;    //!< number of engine cylinders
 uint8_t  num_squirts;           //!< number of squirts per cycle
 volatile uint8_t fuelcut;       //!< fuelcut flag
 volatile uint8_t prime_pulse;   //!< prime pulse flag
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
}inj_state_t;

/**Describes injector channels*/
typedef struct
{
 /**Address of callback which will be used for settiong of I/O */
 volatile fnptr_t io_callback1;  //!< callback pointer (function which sets corresponding I/O)
 uint8_t io_map;                 //!< for mapping of channel number to a particular I/O
}inj_chanstate_t;


/**Describes event queue entry*/
typedef struct
{
 uint16_t end_time;              //!< End time of event in ticks of free running timer 1
 uint8_t  chan_idx;              //!< Associated injection channel number which will be turned off
}inj_queue_t;

/** Global instance of injector state variable structure*/
inj_state_t inj;

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


/**Tune channels' I/O for semi-sequential injection mode */
static void set_channels_ss(void)
{
 if (inj.num_squirts == inj.cyl_number)
 { //normal semi-sequential mode (number of squirts = number of cylinders)
  uint8_t _t, i = 0, chan = inj.cyl_number / 2;
  for(; i < chan; ++i)
  {
   fnptr_t value = IOCFG_CB(IOP_INJ_OUT1 + i);
   _t=_SAVE_INTERRUPT();
   _DISABLE_INTERRUPT();
   inj_chanstate[i].io_callback1 = value;
   inj_chanstate[i].io_map = i;
   inj_chanstate[i + chan].io_callback1 = value;
   inj_chanstate[i + chan].io_map = i;
   _RESTORE_INTERRUPT(_t);
  }
 }
 else
 { //number of squirts is half of number of cylinders
  uint8_t _t, i = 0, ch = 0;
  for(; i < inj.cyl_number; ++i)
  {
   fnptr_t value = IOCFG_CB(IOP_INJ_OUT1 + ch);
   _t=_SAVE_INTERRUPT();
   _DISABLE_INTERRUPT();
   if (CHECKBIT(inj.squirt_mask, i)) {
    inj_chanstate[i].io_callback1 = value;
    inj_chanstate[i].io_map = i;
    ++ch;
   }
   _RESTORE_INTERRUPT(_t);
  }
 }
}

/**Set channels' I/O for 2 banks alternating injection mode */
static void set_channels_2bnk(void)
{
 uint8_t _t, i = 0, ch = 0;
 for(; i < inj.cyl_number; ++i)
 {
  fnptr_t value = IOCFG_CB(IOP_INJ_OUT1 + ch);
  _t=_SAVE_INTERRUPT();
  _DISABLE_INTERRUPT();
  if (CHECKBIT(inj.squirt_mask, i)) {
   inj_chanstate[i].io_callback1 = value;
   inj_chanstate[i].io_map = i;
   ch ^= 1;
  }
  _RESTORE_INTERRUPT(_t);
 }
}

void inject_init_state(void)
{
 inj.cfg = INJCFG_THROTTLEBODY;
 inj.inj_time = 0xFFFF;
 inj.fuelcut = 1;  //no fuel cut
 inj.prime_pulse = 0; //no prime pulse
 inj.tmr_chan = 0;
 inj.active_chan = 0;
 inj.mask_chan = 0xFF;
 QUEUE_RESET(1);   //head = tail
 QUEUE_RESET(2);   //head = tail
}

void inject_init_ports(void)
{
 IOCFG_INIT(IOP_INJ_OUT1, INJ_OFF);           //injector 1 is turned off
 IOCFG_INIT(IOP_INJ_OUT2, INJ_OFF);           //injector 2 is turned off
 IOCFG_INIT(IOP_INJ_OUT3, INJ_OFF);           //injector 3 is turned off
 IOCFG_INIT(IOP_INJ_OUT4, INJ_OFF);           //injector 4 is turned off
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

void inject_set_cyl_number(uint8_t cylnum)
{
 _BEGIN_ATOMIC_BLOCK();
 inj.cyl_number = cylnum;
 calc_squirt_mask();                          //update squirt mask
 if (inj.cfg == INJCFG_2BANK_ALTERN)
  set_channels_2bnk();                        //2 banks, alternating
 else if (inj.cfg == INJCFG_SEMISEQUENTIAL)   //semi-sequential mode
  set_channels_ss();
 _END_ATOMIC_BLOCK();
}

void inject_set_num_squirts(uint8_t numsqr)
{
 _BEGIN_ATOMIC_BLOCK();
 inj.num_squirts = numsqr;                    //save number of squirts per cycle
 calc_squirt_mask();                          //update squirt mask
 _END_ATOMIC_BLOCK();
}

void inject_set_inj_time(uint16_t time)
{
 if (time < 16)                               //restrict minimum injection time to 50uS
  time = 16;
 time = (time >> 1) - INJ_COMPB_CALIB;        //subtract calibration ticks
 if (0==_AB(time, 0))                         //avoid strange bug which appears when OCR2B is set to the same value as TCNT2
  (_AB(time, 0))++;

 _BEGIN_ATOMIC_BLOCK();
 inj.inj_time = time;
 _END_ATOMIC_BLOCK();
}

void inject_set_fuelcut(uint8_t state)
{
 inj.fuelcut = state;
}

void inject_set_config(uint8_t cfg)
{
 inj.cfg = cfg;
 if (cfg == INJCFG_2BANK_ALTERN)
  set_channels_2bnk();                             //2 banks, alternating
 else if (cfg == INJCFG_SEMISEQUENTIAL)            //semi-sequential mode
  set_channels_ss();
}

void inject_start_inj(uint8_t chan)
{
 if (!inj.fuelcut)
  return; //fuel is OFF

 if (CHECKBIT(inj.squirt_mask, chan))
 {
  if (inj.cfg < INJCFG_2BANK_ALTERN)
  {//central/simultaneous, We use only one timer channel
   //interrupts must be disabled!
   _BEGIN_ATOMIC_BLOCK();
   OCR2B = TCNT2 + _AB(inj.inj_time, 0);
   inj.tmr2b_h = _AB(inj.inj_time, 1);
   IOCFG_SET(IOP_INJ_OUT1, INJ_ON);           //turn on injector 1
   IOCFG_SET(IOP_INJ_OUT2, INJ_ON);           //turn on injector 2
   IOCFG_SET(IOP_INJ_OUT3, INJ_ON);           //turn on injector 3
   IOCFG_SET(IOP_INJ_OUT4, INJ_ON);           //turn on injector 4
   SETBIT(TIMSK2, OCIE2B);
   SETBIT(TIFR2, OCF2B);                      //reset possible pending interrupt flag
   _END_ATOMIC_BLOCK();
  }
  else
  {//semi-sequential
   if (!inj.tmr_chan)
   { //use 1-st timer channel
    _BEGIN_ATOMIC_BLOCK();
    ((iocfg_pfn_set)inj_chanstate[chan].io_callback1)(INJ_ON);//turn on current injector pair

    if (CHECKBIT(inj.active_chan, inj_chanstate[chan].io_map))
     CLEARBIT(inj.mask_chan, inj_chanstate[chan].io_map); //mask it if it is still active

    SETBIT(inj.active_chan, inj_chanstate[chan].io_map);
    if (QUEUE_IS_EMPTY(1))
    {
     OCR2B = TCNT2 + _AB(inj.inj_time, 0);
     inj.tmr2b_h = _AB(inj.inj_time, 1);
     SETBIT(TIMSK2, OCIE2B);
     SETBIT(TIFR2, OCF2B);                    //reset possible pending interrupt flag
     SETBIT(inj.mask_chan, inj_chanstate[chan].io_map); //unmask channel
    }
    QUEUE_ADD(1, (inj.inj_time << 1), chan);  //append queue by channel requiring processing
    _END_ATOMIC_BLOCK();
    ++inj.tmr_chan;                           //next channel
   }
   else
   { //use 2-nd timer channel
    uint16_t t = inj.inj_time << 1;           //this timer has 1 tick = 3.2uS
    _BEGIN_ATOMIC_BLOCK();
    ((iocfg_pfn_set)inj_chanstate[chan].io_callback1)(INJ_ON); //turn on current injector pair

    if (CHECKBIT(inj.active_chan, inj_chanstate[chan].io_map))
     CLEARBIT(inj.mask_chan, inj_chanstate[chan].io_map); //mask it if it is still active

    SETBIT(inj.active_chan, inj_chanstate[chan].io_map);
    if (QUEUE_IS_EMPTY(2))
    {
     OCR0B = TCNT0 + _AB(t, 0);
     inj.tmr0b_h = _AB(t, 1);
     SETBIT(TIMSK0, OCIE0B);
     SETBIT(TIFR0, OCF0B);                    //reset possible pending interrupt flag
     SETBIT(inj.mask_chan, inj_chanstate[chan].io_map); //unmask channel
    }
    QUEUE_ADD(2, t, chan);                    //append queue by channel requiring processing
    _END_ATOMIC_BLOCK();
    inj.tmr_chan = 0;
   }
  }
 }
}

void inject_open_inj(uint16_t time)
{
  if (0==time || !inj.fuelcut) return;
  time = (time >> 1) - INJ_COMPB_CALIB;
  if (0==_AB(time, 0))
   (_AB(time, 0))++;
  _BEGIN_ATOMIC_BLOCK();
  OCR2B = TCNT2 + _AB(time, 0);
  inj.tmr2b_h = _AB(time, 1);
  IOCFG_SET(IOP_INJ_OUT1, INJ_ON);            //turn on injector 1
  IOCFG_SET(IOP_INJ_OUT2, INJ_ON);            //turn on injector 2
  IOCFG_SET(IOP_INJ_OUT3, INJ_ON);            //turn on injector 3
  IOCFG_SET(IOP_INJ_OUT4, INJ_ON);            //turn on injector 4
  SETBIT(TIMSK2, OCIE2B);
  SETBIT(TIFR2, OCF2B);                       //reset possible pending interrupt flag
  inj.prime_pulse = 1;
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
  if (inj.cfg < INJCFG_2BANK_ALTERN || inj.prime_pulse)
  {//central/simultaneous
   IOCFG_SET(IOP_INJ_OUT1, INJ_OFF);          //turn off injector 1
   IOCFG_SET(IOP_INJ_OUT2, INJ_OFF);          //turn off injector 2
   IOCFG_SET(IOP_INJ_OUT3, INJ_OFF);          //turn off injector 3
   IOCFG_SET(IOP_INJ_OUT4, INJ_OFF);          //turn off injector 4
   CLEARBIT(TIMSK2, OCIE2B);                  //disable this interrupt
   inj.prime_pulse = 0;
  }
  else
  { //semi-sequential
   uint8_t chan_idx = QUEUE_TAIL(1).chan_idx;
   //Do not turn off channel if it is masked
   if (CHECKBIT(inj.mask_chan, inj_chanstate[chan_idx].io_map)) {
    ((iocfg_pfn_set)inj_chanstate[chan_idx].io_callback1)(INJ_OFF); //turn off current injector pair
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
    OCR2B = TCNT2 + _AB(t, 0);
    inj.tmr2b_h = _AB(t, 1);
    SETBIT(TIMSK2, OCIE2B);
    SETBIT(TIFR2, OCF2B);                     //reset possible pending interrupt flag
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
   IOCFG_SET(IOP_INJ_OUT1, INJ_OFF);          //turn off injector 1
   IOCFG_SET(IOP_INJ_OUT2, INJ_OFF);          //turn off injector 2
   IOCFG_SET(IOP_INJ_OUT3, INJ_OFF);          //turn off injector 3
   IOCFG_SET(IOP_INJ_OUT4, INJ_OFF);          //turn off injector 4
   CLEARBIT(TIMSK0, OCIE0B);                  //disable this interrupt
  }
  else
  { //semi-sequential
   uint8_t chan_idx = QUEUE_TAIL(2).chan_idx;
   //Do not turn off channel if it is masked
   if (CHECKBIT(inj.mask_chan, inj_chanstate[chan_idx].io_map)) {
    ((iocfg_pfn_set)inj_chanstate[chan_idx].io_callback1)(INJ_OFF); //turn off current injector pair
    CLEARBIT(inj.active_chan, inj_chanstate[chan_idx].io_map);
   }
   SETBIT(inj.mask_chan, inj_chanstate[chan_idx].io_map); //unmask

   QUEUE_REMOVE(2);
   if (!QUEUE_IS_EMPTY(2))
   {
    uint16_t t = QUEUE_TAIL(2).end_time - TCNT1;
    if (t > 20000) //end_time < TCNT1, so, it is expired  TODO: How can we do this check better?
     t = 2;
    OCR0B = TCNT0 + _AB(t, 0);
    inj.tmr0b_h = _AB(t, 1);
    SETBIT(TIMSK0, OCIE0B);
    SETBIT(TIFR0, OCF0B);                     //reset possible pending interrupt flag
   }
   else
   {
    CLEARBIT(TIMSK0, OCIE0B);                 //disable this interrupt
   }
  }
 }
}


#endif
