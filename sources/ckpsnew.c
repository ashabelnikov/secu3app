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

/** \file ckps.c
 * Implementation of crankshaft position sensor's processing.
 * (���������� ��������� ������� ��������� ���������).
 */

#include "port/avrio.h"
#include "port/interrupt.h"
#include "port/intrinsic.h"
#include "port/port.h"
#include "adc.h"
#include "bitmask.h"
#include "camsens.h"
#include "ckps.h"
#include "ioconfig.h"
#include "magnitude.h"

#include "secu3.h"
#include "knock.h"

#ifdef PHASED_IGNITION
//PHASED_IGNITION can't be used without PHASE_SENSOR
#if !defined(PHASE_SENSOR)
 #error "You can not use phased ignition without phase sensor. Define PHASE_SENSOR if it is present in the system or do not use phased ignition!"
#endif
 void on_cam_edge(void);
 void on_cam_error(void);
#endif

/**Maximum number of ignition channels */
#define IGN_CHANNELS_MAX      8

#ifndef WHEEL_36_1 //60-2
 /** Number of teeth (including absent) (���������� ������ (������� �������������)) */
 #define WHEEL_COGS_NUM   60
 /** Number of absent teeth (���������� ������������� ������) */
 #define WHEEL_COGS_LACK  2
 /**Number of teeth before TDC which determines moment of advance angle latching, start of measurements from sensors,
  * latching of settings into HIP9011 (���-�� ������ �� �.�.� ������������ ������ �������� ���, ����� ��������� ��������,
  * �������� �������� � HIP)  */
 #define WHEEL_LATCH_BTDC 11
 /** p * 2.5,  barrier for detecting of missing teeth (������ ��� �������� �����������) = 2.5 */
 #define CKPS_GAP_BARRIER(p) (((p) << 1) + ((p) >> 1))
 /**Number of teeth per 1 ignition channel           1   2   3   4   5   6   7  8 */
 prog_uint8_t cogsperchan[1+IGN_CHANNELS_MAX] = {0, 120, 60, 40, 30, 24, 20, 0, 15};
#else //36-1
 #define WHEEL_COGS_NUM   36
 #define WHEEL_COGS_LACK  1
 #define WHEEL_LATCH_BTDC 7   //70� (70 ��������)
 #define CKPS_GAP_BARRIER(p) ((p) + ((p) >> 1))  // p * 1.5
 prog_uint8_t cogsperchan[1+IGN_CHANNELS_MAX] = {0,  72, 36, 24, 18,  0, 12, 0, 9};
#endif

/** Number of degrees which corresponds to one rotation (���������� �������� ������������ �� ���� ��� �����) */
#define CKPS_DEGREES_PER_COG (360 / WHEEL_COGS_NUM)

/** Number of last(present) tooth, numeration begins from 1! (����� ����������(�������������) ����, ��������� ���������� � 1!) */
#define WHEEL_LAST_COG (WHEEL_COGS_NUM - WHEEL_COGS_LACK)

/** number of teeth that will be skipped at the start before synchronization
 * (���������� ������ ������� ����� ������������ ��� ������ ����� ��������������) */
#define CKPS_ON_START_SKIP_COGS      5

/** Access Input Capture Register */
#define GetICR() (ICR1)

/** Used to indicate that none from ignition channels are selected
 * (������������ ��� �������� ���� ��� �� ���� ����� ��������� �� ������) */
#define CKPS_CHANNEL_MODENA  255

//Specify values depending on invertion option
#ifndef INVERSE_IGN_OUTPUTS
 #define IGN_OUTPUTS_INIT_VAL 1  //!< value used for initialization
 #define IGN_OUTPUTS_ON_VAL   1  //!< value used to turn on ignition channel
 #define IGN_OUTPUTS_OFF_VAL  0  //!< value used to turn off ignition channel
#else //outputs inversion mode (����� �������� �������)
 #define IGN_OUTPUTS_INIT_VAL 0
 #define IGN_OUTPUTS_ON_VAL   0
 #define IGN_OUTPUTS_OFF_VAL  1
#endif

/** Flags */
typedef struct
{
 uint8_t  ckps_error_flag:1;                 //!< CKP error flag, set in the CKP's interrupt, reset after processing (������� ������ ����, ��������������� � ���������� �� ����, ������������ ����� ���������)
 uint8_t  ckps_is_valid_half_turn_period:1;  //!< used to indicate that measured period is valid (actually measured)
 uint8_t  ckps_is_synchronized:1;            //!< indicates that synchronization has been completed (missing teeth found)
 uint8_t  ckps_new_engine_cycle_happen:1;    //!< flag for synchronization with rotation (���� ������������� � ���������)
 uint8_t  ckps_use_knock_channel:1;          //!< flag which indicates using of knock channel (������� ������������� ������ ���������)
 uint8_t  ckps_need_to_set_channel:1;        //!< indicates that it is necessary to set channel
#ifdef DWELL_CONTROL
 uint8_t  ckps_need_to_set_channel_b:1;      //!< Indicates that it is necessary to set channel (COMPB)
#endif
 uint8_t  ckps_ign_enabled:1;                //!< Ignition enabled/disabled
}ckpsflags_t;

/** State variables */
typedef struct
{
 uint16_t icr_prev;                   //!< previous value if Input Capture Register (���������� �������� �������� �������)
 volatile uint16_t period_curr;       //!< last measured inter-tooth period (�������� ���������� ��������� ������)
 uint16_t period_prev;                //!< previous value of inter-tooth period (���������� �������� ���������� �������)
 volatile uint8_t cog;                //!< counts teeth starting from missing teeth (2 revolutions), begins from 1 (������� ����� ����� ������, �������� ������� � 1)
 volatile uint8_t cog360;             //!< counts teeth starting from missing teeth (1 revolution).
 uint16_t measure_start_value;        //!< remembers the value of the capture register to measure the half-turn (���������� �������� �������� ������� ��� ��������� ������� �����������)
 uint16_t current_angle;              //!< counts out given advance angle during the passage of each tooth (����������� �������� ��� ��� ����������� ������� ����)
 volatile uint16_t half_turn_period;  //!< stores the last measurement of the passage of teeth n (������ ��������� ��������� ������� ����������� n ������)
 int16_t  advance_angle;              //!< required adv.angle * ANGLE_MULTIPLAYER (��������� ��� * ANGLE_MULTIPLAYER)
 volatile int16_t advance_angle_buffered;//!< buffered value of advance angle (to ensure correct latching)
 uint8_t  ignition_cogs;              //!< number of teeth determine the duration of ignition drive pulse (���-�� ������ ������������ ������������ ��������� ������� ������������)
 uint8_t  starting_mode;              //!< state of state machine processing of teeth at the startup (��������� ��������� �������� ��������� ������ �� �����)
 uint8_t  channel_mode;               //!< determines which channel of the ignition to run at the moment (���������� ����� ����� ��������� ����� ��������� � ������ ������)
 volatile uint8_t  cogs_btdc;         //!< number of teeth from missing teeth to TDC of the first cylinder (���-�� ������ �� ����������� �� �.�.� ������� ��������)
 int8_t   knock_wnd_begin_abs;        //!< begin of the phase selection window of detonation in the teeth of wheel, relatively to TDC (������ ���� ������� �������� ��������� � ������ ����� ������������ �.�.�)
 int8_t   knock_wnd_end_abs;          //!< end of the phase selection window of detonation in the teeth of wheel, relatively to TDC (����� ���� ������� �������� ��������� � ������ ����� ������������ �.�.�)
 volatile uint8_t chan_number;        //!< number of ignition channels (���-�� ������� ���������)
 uint32_t frq_calc_dividend;          //!< divident for calculating RPM (������� ��� ������� ������� ��������)
#ifdef DWELL_CONTROL
 volatile uint16_t cr_acc_time;       //!< accumulation time for dwell control (timer's ticks)
 uint8_t  channel_mode_b;             //!< determines which channel of the ignition to start accumulate at the moment (���������� ����� ����� ��������� ����� ����������� ������� � ������ ������)
 uint32_t acc_delay;                  //!< delay between last ignition and next accumulation
 uint16_t tmrval_saved;               //!< value of timer at the moment of each spark
 uint16_t period_saved;               //!< inter-tooth period at the moment of each spark
 uint8_t  add_period_taken;           //!< indicates that time between spark and next cog was taken into account
#endif
 volatile uint8_t chan_mask;          //!< mask used to disable multi-channel mode and use single channel
#ifdef HALL_OUTPUT
 int8_t   hop_offset;                 //!< Hall output: start of pulse in tooth of wheel relatively to TDC
 uint8_t  hop_duration;               //!< Hall output: duration of pulse in tooth of wheel
#endif
}ckpsstate_t;
 
/**Precalculated data (reference points) and state data for a single channel plug
 * ��������������� ������(������� �����) � ������ ��������� ��� ���������� ������ ���������
 */
typedef struct
{
#ifndef DWELL_CONTROL
 /** Counts out teeth for ignition pulse (����������� ����� �������� ���������) */
 volatile uint8_t ignition_pulse_cogs;
#endif

#ifdef HALL_OUTPUT
 volatile uint8_t hop_begin_cog;      //!< Hall output: tooth number that corresponds to the beginning of pulse  
 volatile uint8_t hop_end_cog;        //!< Hall output: tooth number that corresponds to the end of pulse
#endif

 /**Address of callback which will be used for settiong of I/O */
 volatile fnptr_t io_callback;

 /** Determines number of tooth (relatively to TDC) at which "latching" of data is performed (���������� ����� ���� (������������ �.�.�.) �� ������� ���������� "������������" ������) */
 volatile uint8_t cogs_latch;
 /** Determines number of tooth at which measurement of rotation period is performed (���������� ����� ���� �� ������� ������������ ��������� ������� �������� ��������� (����� ���. �������)) */
 volatile uint8_t cogs_btdc;
 /** Determines number of tooth at which phase selection window for knock detection is opened (���������� ����� ���� �� ������� ����������� ���� ������� �������� ������� �� (������ ��������������)) */
 volatile uint8_t knock_wnd_begin;
 /** Determines number of tooth at which phase selection window for knock detection is closed (���������� ����� ���� �� ������� ����������� ���� ������� �������� ������� �� (����� ��������������)) */
 volatile uint8_t knock_wnd_end;
}chanstate_t;

ckpsstate_t ckps;                         //!< instance of state variables
chanstate_t chanstate[IGN_CHANNELS_MAX];  //!< instance of array of channel's state variables

/** Arrange in the free I/O registers (��������� � ��������� ��������� �����/������) */
#define flags ( (volatile ckpsflags_t*)(&TWAR) ) //note: may be not effective on other MCUs

/** Supplement timer/counter 0 up to 16 bits, use R15 (��� ���������� �������/�������� 0 �� 16 ��������, ���������� R15) */
#ifdef __ICCAVR__
 __no_init __regvar uint8_t TCNT0_H@15;
#else //GCC
 uint8_t TCNT0_H __attribute__((section (".noinit")));
#endif

/**This will speedup calculations - replaces 8-bit division. */
#define COGSPERCHAN(channum) PGM_GET_BYTE(&cogsperchan[channum])

/**Table srtores dividends for calculating of RPM */
#define FRQ_CALC_DIVIDEND(channum) PGM_GET_DWORD(&frq_calc_dividend[channum])
prog_uint32_t frq_calc_dividend[1+IGN_CHANNELS_MAX] = 
 //     1          2          3          4         5         6      7      8
 {0, 30000000L, 15000000L, 10000000L, 7500000L, 6000000L, 5000000L, 0L, 3750000L};

void ckps_init_state_variables(void)
{
#ifndef DWELL_CONTROL
 uint8_t i;
 _BEGIN_ATOMIC_BLOCK();
 //Set to value that means to do nothing with outputs
 //(��������� � �������� ���������� ������ �� ������ � ��������) 
 for(i = 0; i < IGN_CHANNELS_MAX; ++i)
  chanstate[i].ignition_pulse_cogs = 255;
#else
 _BEGIN_ATOMIC_BLOCK();
 ckps.cr_acc_time = 0;
 ckps.channel_mode_b = CKPS_CHANNEL_MODENA;
 flags->ckps_need_to_set_channel_b = 0;
#endif

 ckps.cog = ckps.cog360 = 0;
 ckps.half_turn_period = 0xFFFF;
 ckps.advance_angle = 0;
 ckps.advance_angle_buffered = 0;
 ckps.starting_mode = 0;
 ckps.channel_mode = CKPS_CHANNEL_MODENA;
#ifdef PHASED_IGNITION
 cams_set_callbacks(&on_cam_edge, &on_cam_error);
 cams_set_error_threshold(WHEEL_COGS_NUM * 2);
#endif
 flags->ckps_need_to_set_channel = 0;
 flags->ckps_new_engine_cycle_happen = 0;
 flags->ckps_is_synchronized = 0;
 flags->ckps_ign_enabled = 1;
 TCCR0 = 0; //timer is stopped (������������� ������0)
 _END_ATOMIC_BLOCK();
}

void ckps_init_state(void)
{
 _BEGIN_ATOMIC_BLOCK();
 ckps_init_state_variables();
 flags->ckps_error_flag = 0;

 //Compare channels do not connected to lines of ports (normal port mode)
 //(������ Compare �� ���������� � ������ ������ (���������� ����� ������))
 TCCR1A = 0;

 //(Noise reduction(���������� ����), rising edge of capture(�������� ����� �������), clock = 250kHz)
 TCCR1B = _BV(ICNC1)|_BV(ICES1)|_BV(CS11)|_BV(CS10);

 //enable input capture and Compare A interrupts of timer 1, also overflow interrupt of timer 0
 //(��������� ���������� �� ������� � ��������� � ������� 1, � ����� �� ������������ ������� 0)
 TIMSK|= _BV(TICIE1)/*|_BV(OCIE1A)*/|_BV(TOIE0);
 _END_ATOMIC_BLOCK();
}

void ckps_set_advance_angle(int16_t angle)
{
 _BEGIN_ATOMIC_BLOCK();
 ckps.advance_angle_buffered = angle;
 _END_ATOMIC_BLOCK();
}

void ckps_init_ports(void)
{
 PORTD|= _BV(PD6); // pullup for ICP1 (�������� ��� ICP1)

 //after ignition is on, igniters must not be in the accumulation mode,
 //therefore set low level on their inputs
 //(����� ��������� ��������� ����������� �� ������ ���� � ������ ����������,
 //������� ������������� �� �� ������ ������ �������)
 iocfg_i_ign_out1(IGN_OUTPUTS_INIT_VAL);                //init 1-st ignition channel
 iocfg_i_ign_out2(IGN_OUTPUTS_INIT_VAL);                //init 2-nd ignition channel
 IOCFG_INIT(IOP_IGN_OUT3, IGN_OUTPUTS_INIT_VAL);        //init 3-rd (can be remapped)
 IOCFG_INIT(IOP_IGN_OUT4, IGN_OUTPUTS_INIT_VAL);        //init 4-th (can be remapped)
 IOCFG_INIT(IOP_ADD_IO1, IGN_OUTPUTS_INIT_VAL);         //init 5-th (can be remapped)
 IOCFG_INIT(IOP_ADD_IO2, IGN_OUTPUTS_INIT_VAL);         //init 6-th (can be remapped)

 //init I/O for Hall output if it is enabled
#ifdef HALL_OUTPUT
 IOCFG_INIT(IOP_HALL_OUT, 1);
#endif
}

//Calculation of instantaneous frequency of crankshaft rotation from the measured period between the cycles of the engine
//(for example for 4-cylinder, 4-stroke it is 180�) 
//Period measured in the discretes of timer (one discrete = 4us), one minute = 60 seconds, one second has 1,000,000 us.
//������������ ���������� ������� �������� ��������� �� ����������� ������� ����� ������� ��������� 
//(�������� ��� 4-������������, 4-� �������� ��� 180 ��������)
//������ � ��������� ������� (���� �������� = 4���), � ����� ������ 60 ���, � ����� ������� 1000000 ���.
uint16_t ckps_calculate_instant_freq(void)
{
 uint16_t period;
 _DISABLE_INTERRUPT();
 period = ckps.half_turn_period;           //ensure atomic acces to variable (������������ ��������� ������ � ����������)
 _ENABLE_INTERRUPT();

 //if equal to minimum, this means engine is stopped (���� ����� �������, ������ ��������� �����������)
 if (period != 0xFFFF)
  return (ckps.frq_calc_dividend)/(period);
 else
  return 0;
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
 * Tooth number should not be greater than WHEEL_COGS_NUM*2 or less than zero
 */
uint8_t _normalize_tn(int8_t i_tn)
{
 if (i_tn > (WHEEL_COGS_NUM * 2))
  return i_tn - (WHEEL_COGS_NUM * 2);
 if (i_tn < 0)
  return i_tn + (WHEEL_COGS_NUM * 2);
 return i_tn;
}

void ckps_set_cogs_btdc(uint8_t cogs_btdc)
{
 uint8_t _t, i;
 // pre-compute and store the reference points (teeth) (������� ��������� � ��������� ������� ����� (�����))
 // cogs_per_cycle - number of wheel teeth attributable to a single cycle of engine (���������� ������ ����� ������������ �� ���� ���� ���������)
 uint8_t cogs_per_cycle = COGSPERCHAN(ckps.chan_number);
 _t=_SAVE_INTERRUPT();
 _DISABLE_INTERRUPT();
 for(i = 0; i < ckps.chan_number; ++i)
 {
  uint8_t tdc = (cogs_btdc + i * cogs_per_cycle);
  chanstate[i].cogs_btdc = _normalize_tn(tdc);
  chanstate[i].cogs_latch = _normalize_tn(tdc - WHEEL_LATCH_BTDC);
  chanstate[i].knock_wnd_begin = _normalize_tn(tdc + ckps.knock_wnd_begin_abs);
  chanstate[i].knock_wnd_end = _normalize_tn(tdc + ckps.knock_wnd_end_abs);
#ifdef HALL_OUTPUT
  //update Hall output pulse parameters because they depend on ckps.cogs_btdc parameter
  chanstate[i].hop_begin_cog = _normalize_tn(tdc - ckps.hop_offset);
  chanstate[i].hop_end_cog = _normalize_tn(chanstate[i].hop_begin_cog + ckps.hop_duration);
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
#endif

uint8_t ckps_is_error(void)
{
 return flags->ckps_error_flag;
}

void ckps_reset_error(void)
{
 flags->ckps_error_flag = 0;
}

void ckps_use_knock_channel(uint8_t use_knock_channel)
{
 flags->ckps_use_knock_channel = use_knock_channel;
}

uint8_t ckps_is_cycle_cutover_r()
{
 uint8_t result;
 _BEGIN_ATOMIC_BLOCK();
 result = flags->ckps_new_engine_cycle_happen;
 flags->ckps_new_engine_cycle_happen = 0;
 _END_ATOMIC_BLOCK();
 return result;
}

uint8_t ckps_get_current_cog(void)
{
 return ckps.cog;
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

/**Tune channels' I/O for semi-sequential ignition mode (wasted spark) */
void set_channels_ss(void)
{
 uint8_t _t, i = 0, chan = ckps.chan_number / 2;
 for(; i < chan; ++i)
 {
  fnptr_t value;
  if (0==i)
   value = (fnptr_t)_IOREM_GPTR(iocfg_s_ign_out1);
  else if (1==i)
   value = (fnptr_t)_IOREM_GPTR(iocfg_s_ign_out2);
  else
   value = IOCFG_CB(i);

  _t=_SAVE_INTERRUPT();
  _DISABLE_INTERRUPT();
  chanstate[i].io_callback = value;
  chanstate[i + chan].io_callback = value;
  _RESTORE_INTERRUPT(_t);
 }
}

/**Tune channels' I/O for full sequential ignition mode */
void set_channels_fs(void)
{
 uint8_t _t, i = 0;
 for(; i < ckps.chan_number; ++i)
 {
  fnptr_t value;
  if (0==i)
   value = (fnptr_t)_IOREM_GPTR(iocfg_s_ign_out1);
  else if (1==i)
   value = (fnptr_t)_IOREM_GPTR(iocfg_s_ign_out2);
  else
   value = IOCFG_CB(i);

  _t=_SAVE_INTERRUPT();
  _DISABLE_INTERRUPT();
  chanstate[i].io_callback = value;
  _RESTORE_INTERRUPT(_t);
 }
}

void ckps_set_cyl_number(uint8_t i_cyl_number)
{
 _BEGIN_ATOMIC_BLOCK();
 ckps.chan_number = i_cyl_number;
 _END_ATOMIC_BLOCK();

 ckps.frq_calc_dividend = FRQ_CALC_DIVIDEND(i_cyl_number);

 //We have to retune I/O configuration after changing of cylinder number
#ifndef PHASED_IGNITION
 set_channels_ss();  // Tune for semi-sequential mode
#else //phased ignition
 if (!cams_is_ready())
  set_channels_ss(); // Tune for semi-sequential mode if cam sensor doesn't work
 else
  set_channels_fs(); // Tune for full sequential mode
#endif

 //TODO: calculations previosly made by ckps_set_cogs_btdc()|ckps_set_knock_window()|ckps_set_hall_pulse() becomes invalid!
 //So, ckps_set_cogs_btdc() must be called again. Do it here or in place where this function called.
}

void ckps_set_knock_window(int16_t begin, int16_t end)
{
 uint8_t _t, i, cogs_per_cycle;
 //translate from degrees to teeth (��������� �� �������� � �����)
 ckps.knock_wnd_begin_abs = begin / (CKPS_DEGREES_PER_COG * ANGLE_MULTIPLAYER);
 ckps.knock_wnd_end_abs = end / (CKPS_DEGREES_PER_COG * ANGLE_MULTIPLAYER);

 cogs_per_cycle = COGSPERCHAN(ckps.chan_number);
 _t=_SAVE_INTERRUPT();
 _DISABLE_INTERRUPT();
 for(i = 0; i < ckps.chan_number; ++i)
 {
  uint8_t tdc = (ckps.cogs_btdc + i * cogs_per_cycle);
  chanstate[i].knock_wnd_begin = _normalize_tn(tdc + ckps.knock_wnd_begin_abs);
  chanstate[i].knock_wnd_end = _normalize_tn(tdc + ckps.knock_wnd_end_abs);
 }
 _RESTORE_INTERRUPT(_t);
}

void ckps_enable_ignition(uint8_t i_cutoff)
{
 flags->ckps_ign_enabled = i_cutoff;
}

void ckps_set_merge_outs(uint8_t i_merge)
{
 ckps.chan_mask = i_merge ? 0x00 : 0xFF;
}

#ifdef HALL_OUTPUT
void ckps_set_hall_pulse(int8_t i_offset, uint8_t i_duration)
{
 uint8_t _t, i, cogs_per_cycle;
 //save values because we will access them from other function
 ckps.hop_offset = i_offset;
 ckps.hop_duration = i_duration;

 cogs_per_cycle = COGSPERCHAN(ckps.chan_number);
 _t=_SAVE_INTERRUPT();
 _DISABLE_INTERRUPT();
 for(i = 0; i < ckps.chan_number; ++i)
 {
  uint8_t tdc = (ckps.cogs_btdc + i * cogs_per_cycle);
  chanstate[i].hop_begin_cog = _normalize_tn(tdc - ckps.hop_offset);
  chanstate[i].hop_end_cog = _normalize_tn(chanstate[i].hop_begin_cog + ckps.hop_duration);
 }
 _RESTORE_INTERRUPT(_t);
}
#endif

/** Turn OFF specified ignition channel
 * \param i_channel number of ignition channel to turn off
 */
INLINE
void turn_off_ignition_channel(uint8_t i_channel)
{
 if (!flags->ckps_ign_enabled)
  return; //ignition disabled
 //Completion of igniter's ignition drive pulse, transfer line of port into a low level - makes 
 //the igniter go to the regime of energy accumulation
 //���������� �������� ������� �����������, ������� ����� ����� � ������ ������� - ����������
 //���������� ������� � ����� ���������� �������
 ((iocfg_pfn_set)chanstate[i_channel].io_callback)(IGN_OUTPUTS_OFF_VAL);
}

#ifdef PHASED_IGNITION
/** Callback from cam sensor. Executed on each edge,
 * this function turns on phased ignition.
 * Note: it is executed from interrupt handler */
void on_cam_edge(void)
{
 //init_chan_cyl(0, ckps.chan_number);
 //TODO:
}

/** Callback from cam sensor. Executed on error (cam is missing),
 * this function turns off phased ignition and turns on wasted spark.
 * Note: it is executed from interrupt handler */
void on_cam_error(void)
{
 //init_chan_cyl(255, ckps.chan_number);
 //TODO:
}
#endif

/**Interrupt handler for Compare/Match channel A of timer T1
 * ������ ���������� �� ���������� ������ � ������� �1
 */
ISR(TIMER1_COMPA_vect)
{
#ifdef COOLINGFAN_PWM
 uint8_t timsk_sv, ucsrb_sv = UCSRB;
#endif

#ifdef DWELL_CONTROL
 ckps.tmrval_saved = TCNT1;
#endif

 TIMSK&= ~_BV(OCIE1A); //disable interrupt (��������� ����������)

 //line of port in the low level, now set it into a high level - makes the igniter to stop 
 //the accumulation of energy and close the transistor (spark)
 //(����� ����� � ������ ������, ������ ��������� � � ������� ������� - ���������� ���������� ����������
 //���������� ������� � ������� ���������� (�����)).
 if (CKPS_CHANNEL_MODENA == ckps.channel_mode)
  return; //none of channels selected (������� ����� �� ������)

 ((iocfg_pfn_set)chanstate[ckps.channel_mode].io_callback)(IGN_OUTPUTS_ON_VAL);

 //-----------------------------------------------------
#ifdef COOLINGFAN_PWM
 timsk_sv = TIMSK;
 //ADCSRA&=~_BV(ADIE);            //??
 UCSRB&=~(_BV(RXCIE)|_BV(UDRIE)); //mask UART interrupts
 TIMSK&= _BV(OCIE2)|_BV(TOIE2);   //mask all timer interrupts except OCF2 and TOV2
 _ENABLE_INTERRUPT();
#endif
 //-----------------------------------------------------

#ifdef DWELL_CONTROL 
 ckps.acc_delay = ((uint32_t)ckps.period_curr) * COGSPERCHAN(ckps.chan_number); 
 if (ckps.cr_acc_time > ckps.acc_delay-120)
  ckps.cr_acc_time = ckps.acc_delay-120;  //restrict accumulation time. Dead band = 500us
 ckps.acc_delay-= ckps.cr_acc_time;
 ckps.period_saved = ckps.period_curr; //remember current inter-tooth period

 ckps.channel_mode_b = (ckps.channel_mode < ckps.chan_number-1) ? ckps.channel_mode + 1 : 0 ;
 ckps.channel_mode_b&= ckps.chan_mask;
 ckps.add_period_taken = 0;
 flags->ckps_need_to_set_channel_b = 1;

 //We remembered value of TCNT1 at the top of of this function. But input capture event
 //may occur when interrupts were already disabled (by hardware) but value of timer is still
 //not saved. Another words, ICR1 must not be less than tmrval_saved.
 if (TIFR & _BV(ICF1))
  ckps.tmrval_saved = ICR1;
#else
 //start counting the duration of pulse in the teeth (�������� ������ ������������ �������� � ������)
 chanstate[ckps.channel_mode].ignition_pulse_cogs = 0;
#endif

 //-----------------------------------------------------
#ifdef COOLINGFAN_PWM
 _DISABLE_INTERRUPT();
 //ADCSRA|=_BV(ADIE);
 UCSRB = ucsrb_sv;
 TIMSK = timsk_sv;
#endif
 //-----------------------------------------------------
}

#ifdef DWELL_CONTROL
/**Interrupt handler for Compare/Match channel B of timer T1. Used for dwell control.
 * ������ ���������� �� ���������� ������ B ������� �1. ������������ ��� ���������� ����������� ������� ��.
 */
ISR(TIMER1_COMPB_vect)
{
 TIMSK&= ~_BV(OCIE1B); //��������� ����������
 //start accumulation
 turn_off_ignition_channel(ckps.channel_mode_b);
 ckps.channel_mode_b = CKPS_CHANNEL_MODENA;
}
#endif

/**Initialization of timer 0 using specified value and start, clock = 250kHz
 * It is assumed that this function called when all interrupts are disabled
 * (������������� ������� 0 ��������� ��������� � ������, clock = 250kHz.
 * �������������� ��� ����� ���� ������� ����� ����������� ��� ����������� �����������)
 * \param value value for load into timer (�������� ��� �������� � ������)
 */
INLINE
void set_timer0(uint16_t value)
{
 TCNT0_H = _AB(value, 1);
 TCNT0 = 255 - _AB(value, 0);
 TCCR0  = _BV(CS01)|_BV(CS00);
}

/**Helpful function, used at the startup of engine
 * (��������������� �������, ������������ �� ����� �����)
 * \return 1 when synchronization is finished, othrewise 0 (1 ����� ������������� ��������, ����� 0)
 */
uint8_t sync_at_startup(void)
{
 switch(ckps.starting_mode)
 {
  case 0: //skip certain number of teeth (������� ������������� ���-�� ������)
   /////////////////////////////////////////
   flags->ckps_is_valid_half_turn_period = 0;
   /////////////////////////////////////////
   if (ckps.cog >= CKPS_ON_START_SKIP_COGS) 
    ckps.starting_mode = 1;
   break;
  case 1: //find out missing teeth (����� �����������)
   if (ckps.period_curr > CKPS_GAP_BARRIER(ckps.period_prev)) 
   {
    flags->ckps_is_synchronized = 1;
    ckps.period_curr = ckps.period_prev;  //exclude value of missing teeth's period
    ckps.cog = 1; //first tooth (1-� ���)
    ckps.cog360 = 1;
    return 1; //finish process of synchronization (����� �������� �������������)
   }
   break;
  }
 ckps.icr_prev = GetICR();
 ckps.period_prev = ckps.period_curr;
 ++ckps.cog;
 return 0; //continue process of synchronization (����������� �������� �������������)
}

/**This procedure called for all teeth (including recovered teeth)
 * ���������. ���������� ��� ���� ������ ����� (������������ � ����������������)
 */
void process_ckps_cogs(void)
{
 uint16_t diff;
 uint8_t i, timsk_sv = TIMSK;

 //-----------------------------------------------------
 //Software PWM is very sensitive even to small delays. So, we need to allow OCF2 and TOV2
 //interrupts occur during processing of this handler.
#ifdef COOLINGFAN_PWM
 //remember current state, mask all not urgent interrupts and enable interrupts
 uint8_t ucsrb_sv = UCSRB;
 //ADCSRA&=~_BV(ADIE);            //??
 UCSRB&=~(_BV(RXCIE)|_BV(UDRIE)); //mask UART interrupts
 TIMSK&= _BV(OCIE2)|_BV(TOIE2);   //mask all timer interrupts except OCF2 and TOV2
 _ENABLE_INTERRUPT();
#endif
 //-----------------------------------------------------

#ifdef DWELL_CONTROL
 if (ckps.channel_mode_b != CKPS_CHANNEL_MODENA)
 {
  //We must take into account time elapsed between last spark and following tooth.
  //Because ICR1 can not be less than tmrval_saved we are using addition modulo 65535
  //to calculate difference (elapsed time).
  if (!ckps.add_period_taken)
  {
   ckps.acc_delay-=((uint16_t)(~ckps.tmrval_saved)) + 1 + GetICR();
   ckps.add_period_taken = 1;
  }
  //Correct our prediction on each cog 
  ckps.acc_delay+=((int32_t)ckps.period_curr - ckps.period_saved);

  //Do we have to set COMPB ? (We have to set COMPB if less than 2 periods remain)
  if (flags->ckps_need_to_set_channel_b && ckps.acc_delay <= (ckps.period_curr << 1))
  {
   OCR1B = GetICR() + ckps.acc_delay + ((int32_t)ckps.period_curr - ckps.period_saved);
   TIFR = _BV(OCF1B);
   timsk_sv|= _BV(OCIE1B);
   flags->ckps_need_to_set_channel_b = 0; // To avoid entering into setup mode (����� �� ����� � ����� ��������� ��� ���)
  }
  ckps.acc_delay-=ckps.period_curr;
 }
#endif

 if (flags->ckps_use_knock_channel)
 {
  for(i = 0; i < ckps.chan_number; ++i)
  {
   //start listening a detonation (opening the window)
   //�������� ������� ��������� (�������� ����)
   if (ckps.cog == chanstate[i].knock_wnd_begin)
    knock_set_integration_mode(KNOCK_INTMODE_INT);

   //finish listening a detonation (closing the window) and start the process of measuring integrated value
   //����������� ������� ��������� (�������� ����) � ��������� ������� ��������� ������������ ��������
   if (ckps.cog == chanstate[i].knock_wnd_end)
   {
    knock_set_integration_mode(KNOCK_INTMODE_HOLD);
    adc_begin_measure_knock(_AB(ckps.half_turn_period, 1) < 4);
   }
  }
 }

 for(i = 0; i < ckps.chan_number; ++i)
 {
  //for 66� before TDC (before working cycle) establish new advance angle to be actuated,
  //before this moment value was stored in a temporary buffer.
  //�� 66 �������� �� �.�.� ����� ������� ������ ������������� ����� ��� ��� ����������, ���
  //�� ����� �������� �� ��������� ������.
  if (ckps.cog == chanstate[i].cogs_latch)
  {
   ckps.channel_mode = (i & ckps.chan_mask); //remember number of channel (���������� ����� ������)
   flags->ckps_need_to_set_channel = 1;      //establish an indication that needed to count advance angle (������������� ������� ����, ��� ����� ����������� ���)
   //start counting of advance angle (�������� ������ ���� ����������)
   ckps.current_angle = ANGLE_MAGNITUDE(CKPS_DEGREES_PER_COG) * WHEEL_LATCH_BTDC; // those same 66� (�� ����� 66�)
   ckps.advance_angle = ckps.advance_angle_buffered; //advance angle with all the adjustments (say, 15�)(���������� �� ����� ��������������� (��������, 15�))
   knock_start_settings_latching();//start the process of downloading the settings into the HIP9011 (��������� ������� �������� �������� � HIP)
   adc_begin_measure(_AB(ckps.half_turn_period, 1) < 4);//start the process of measuring analog input values (������ �������� ��������� �������� ���������� ������)
  }

  //teeth of end/beginning of the measurement of rotation period - TDC Read and save the measured period,
  //then remember current value of count for the next measurement
  //(����� ����������/������ ��������� �������� ��������  - �.�.�. ���������� � ���������� ����������� �������,
  //����� ����������� �������� �������� �������� ��� ���������� ���������)
  if (ckps.cog==chanstate[i].cogs_btdc) 
  {
   //if it was overflow, then set the maximum possible time
   //���� ���� ������������ �� ������������� ����������� ��������� �����
   if (((ckps.period_curr > 1250) || !flags->ckps_is_valid_half_turn_period))
    ckps.half_turn_period = 0xFFFF;
   else
    ckps.half_turn_period = (GetICR() - ckps.measure_start_value);

   ckps.measure_start_value = GetICR();
   flags->ckps_is_valid_half_turn_period = 1;
   /////////////////////////////////////////
   flags->ckps_new_engine_cycle_happen = 1; //set the cycle-synchronozation event (������������� ������� �������� �������������)
   /////////////////////////////////////////
  }

#ifdef HALL_OUTPUT
  if (ckps.cog == chanstate[i].hop_begin_cog)
   IOCFG_SET(IOP_HALL_OUT, 1);
  if (ckps.cog == chanstate[i].hop_end_cog)
   IOCFG_SET(IOP_HALL_OUT, 0);
#endif
 }

 //Preparing to start the ignition for the current channel (if the right moment became)
 //���������� � ������� ��������� ��� �������� ������ (���� �������� ������ ������)
 if (flags->ckps_need_to_set_channel && ckps.channel_mode!= CKPS_CHANNEL_MODENA)
 {
  diff = ckps.current_angle - ckps.advance_angle;
  if (diff <= (ANGLE_MAGNITUDE(CKPS_DEGREES_PER_COG) << 1))
  {
   //before starting the ignition it is left to count less than 2 teeth. It is necessary to prepare the compare module
   //(�� ������� ��������� �������� ��������� ������ 2-x ������. ���������� ����������� ������ ���������)
   //TODO: replace heavy division by multiplication with magic number. This will reduce up to 40uS !
   if (ckps.period_curr < 128)
    OCR1A = GetICR() + (diff * (ckps.period_curr)) / ANGLE_MAGNITUDE(CKPS_DEGREES_PER_COG);
   else
    OCR1A = GetICR() + ((uint32_t)diff * (ckps.period_curr)) / ANGLE_MAGNITUDE(CKPS_DEGREES_PER_COG);
   TIFR = _BV(OCF1A);
   flags->ckps_need_to_set_channel = 0; // For avoiding to enter into setup mode (����� �� ����� � ����� ��������� ��� ���)
   timsk_sv|= _BV(OCIE1A); //enable Compare A interrupt (��������� ����������)
  }
 }

#ifndef DWELL_CONTROL
 //finish the ignition trigger pulses for igniter(s) and immediately increase the number of tooth for processed channel
 //����������� �������� ������� �����������(��) � ����� ����������� ����� ���� ��� ������������� ������
 for(i = 0; i < ckps.chan_number; ++i)
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

 //tooth passed - angle before TDC decriased by 6�  (for 60-2) or 10� (for 36-1).
 //(������ ��� - ���� �� �.�.�. ���������� �� 6 ���� (��� 60-2) ��� 10 ���� (��� 36-1))
 ckps.current_angle-= ANGLE_MAGNITUDE(CKPS_DEGREES_PER_COG);
 ++ckps.cog;

#ifdef PHASE_SENSOR
 //search for level's toggle from camshaft sensor on each cog
 cams_detect_edge();
#endif

 //-----------------------------------------------------
#ifdef COOLINGFAN_PWM
 //disable interrupts and restore previous states of masked interrupts
 _DISABLE_INTERRUPT();
 //ADCSRA|=_BV(ADIE);
 UCSRB = ucsrb_sv;
#endif
 TIMSK = timsk_sv;
 //-----------------------------------------------------
}

/**Input capture interrupt of timer 1 (called at passage of each tooth)
 * ���������� �� ������� ������� 1 (���������� ��� ����������� ���������� ����)
 */
ISR(TIMER1_CAPT_vect)
{
 ckps.period_curr = GetICR() - ckps.icr_prev;

 //At the start of engine, skipping a certain number of teeth for initializing
 //the memory of previous periods. Then look for missing teeth.
 //��� ������ ���������, ���������� ������������ ���-�� ������ ��� �������������
 //������ ���������� ��������. ����� ���� �����������.
 if (!flags->ckps_is_synchronized)
 {
  if (sync_at_startup())
   goto synchronized_enter;
  return;
 }

 //Each period, check for missing teeth, and if, after discovering of missing teeth
 //count of teeth being found incorrect, then set error flag.
 //(������ ������ ��������� �� �����������, � ���� ����� ����������� �����������
 //��������� ��� ���-�� ������ ������������, �� ������������� ������� ������).
 if (ckps.period_curr > CKPS_GAP_BARRIER(ckps.period_prev))
 {
  if ((ckps.cog360 != (WHEEL_COGS_NUM + 1))) //also taking into account recovered teeth (��������� ����� ��������������� �����)
   flags->ckps_error_flag = 1; //ERROR
  //Reset 360� tooth counter to the first tooth (1-� ���)
  ckps.cog360 = 1;
  //Also reset 720� tooth counter
  if (ckps.cog == ((2*WHEEL_COGS_NUM) + 1))
   ckps.cog = 1;
  ckps.period_curr = ckps.period_prev;  //exclude value of missing teeth's period
 }

synchronized_enter:
 //If the last tooth before missing teeth, we begin the countdown for
 //the restoration of missing teeth, as the initial data using the last
 //value of inter-teeth period.
 //(���� ��������� ��� ����� ������������, �� �������� ������ ������� ���
 //�������������� ������������� ������, � �������� �������� ������ ����������
 //��������� �������� ���������� �������).
 if (ckps.cog360 == WHEEL_LAST_COG)
  set_timer0(ckps.period_curr);

 //call handler for normal teeth (�������� ���������� ��� ���������� ������)
 process_ckps_cogs();
 ++ckps.cog360;

 ckps.icr_prev = GetICR();
 ckps.period_prev = ckps.period_curr;
}

/**Purpose of this interrupt handler is to supplement timer up to 16 bits and call procedure
 * for processing teeth when set 16 bit timer expires
 * (������ ����� ����������� ��������� ������ �� 16-�� �������� � �������� ���������
 * ��������� ������ �� ��������� �������������� 16-�� ���������� �������). */
ISR(TIMER0_OVF_vect)
{
 if (TCNT0_H!=0)  //Did high byte exhaust (������� ���� �� ��������) ?
 {
  TCNT0 = 0;
  --TCNT0_H;
 }
 else
 {//the countdown is over (������ ������� ����������)
  ICR1 = TCNT1;  //simulate input capture 
  TCCR0 = 0;     //stop timer (������������� ������)

#if (WHEEL_COGS_LACK > 1)
  //start timer to recover 60th tooth (��������� ������ ����� ������������ 60-� (���������) ���)
  if (ckps.cog360 == (WHEEL_COGS_NUM-1))
   set_timer0(ckps.period_curr);
#endif

  //Call handler for missing teeth (�������� ���������� ��� ������������� ������)
  process_ckps_cogs();
  ++ckps.cog360;
 }
}