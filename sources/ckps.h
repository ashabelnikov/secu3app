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

/** \file ckps.h
 * \author Alexey A. Shabelnikov
 * Processing of crankshaft position sensor.
 */

#ifndef _CKPS_H_
#define _CKPS_H_

#include <stdint.h>

/**Scaling factor of crankshaft rotation angle, appears in the calculations and operations of the division
 * so it should be a multiple of degree of 2.
 */
#define ANGLE_MULTIPLIER   32

/**Initialization of CKP module (hardware & variables)
 */
void ckps_init_state(void);

/** Set edge type for CKP sensor
 * \param edge_type 0 - falling, 1 - rising
 */
void ckps_set_edge_type(uint8_t edge_type);

/** Set count of teeth before TDC.
 * \param cogs_btdc E.g values for 60-2 wheel: 17,18,19,20,21,22,23
                               for 36-1 wheel: 8,9,10,11,12,13,14
 * \details For 4-cylinder engine. If the crankshaft is in a position corresponding to top dead center (tdc) of the first cylinder's piston,
 * then according to the standards the middle of 20th tooth of sync wheel must be in the opposite to the CKP's core
 * (counting out against the direction of rotation from the place of the cutout).
 */
void ckps_set_cogs_btdc(uint8_t cogs_btdc);


#ifndef DWELL_CONTROL
/** Set duration of ignition pulse drive
 * \param cogs duration of pulse, countable in the teeth of wheel
 * \details For standard igniters duration of driving pulses must be 1/3, when significant deviation to the smaller side
 * is present then igniters can became out of action. If you connect two outputs together to one igniter, you must put
 * a value of 10, if double channel mode then 40. The values given for the 60-2 wheel and 4 cylinder engine.
 */
void ckps_set_ignition_cogs(uint8_t cogs);
#else
/**Dwell control. Set accumulation time
 * \param i_acc_time accumulation time in timer's ticks (1 tick = 4uS)
 */
void ckps_set_acc_time(uint16_t i_acc_time);
/**Set rising/falling edge of ignition pulse on spark
 * \param rising_edge 0 - falling edge on spark (default), 1 - rising edge on spark
 */
void ckps_set_rising_spark(uint8_t rising_edge);
#endif

/** Set andvance angle
 * \param angle advance angle * ANGLE_MULTIPLIER
 */
void ckps_set_advance_angle(int16_t angle);
#ifdef SPLIT_ANGLE
void ckps_set_advance_angle1(int16_t angle);
#endif

/** Calculate instant RPM using last measured period
 * \return RPM (min-1)
 */
uint16_t ckps_calculate_instant_freq(void);

/** Set pahse selection window for detonation
 * \param begin begin of window (degrees relatively to t.d.c)
 * \param end end of window (degrees relatively to t.d.c)
 */
void ckps_set_knock_window(int16_t begin, int16_t end);

/** Set to use or not to use knock detection
 * \param use_knock_channel 1 - use, 0 - do not use
 */
void ckps_use_knock_channel(uint8_t use_knock_channel);

/** \return nonzero if error was detected */
uint8_t ckps_is_error(void);

/** Reset detected errors */
void ckps_reset_error(void);

/**\return 1 if there was engine stroke and reset flag!
 * \details Used to perform synchronization with rotation of crankshaft.
 */
uint8_t ckps_is_stroke_event_r(void);

/** Initialization of state variables */
void ckps_init_state_variables(void);

/** \return returns 1 if number of current tooth has been changed
 */
uint8_t ckps_is_cog_changed(void);

/** Set number of engine's cylinders
 * \param i_cyl_number allowed values(допустимые значения): *1,2,*3,4,*5,6,8
 * * these values are allowed only if firmware compliled with PHASED_IGNITION option
 */
void ckps_set_cyl_number(uint8_t i_cyl_number);

/** Initialization of used I/O ports */
void ckps_init_ports(void);

/** Enable/disable ignition
 * \param i_cutoff Acceptable values: 1 - enable ignition, 0 - disable ignition
 */
void ckps_enable_ignition(uint8_t i_cutoff);

/** Enable/disbale merging of ignition outputs
 * \param i_merge 1 - merge, 0 - normal mode
 */
void ckps_set_merge_outs(uint8_t i_merge);

#ifdef HALL_OUTPUT
/** Set parameters for Hall output pulse
 * \param i_offset - offset relatively to TDC (if > 0, then BTDC), value * ANGLE_MULTIPLIER
 * \param i_duration - duration of pulse, value * ANGLE_MULTIPLIER
 */
void ckps_set_hall_pulse(int16_t i_offset, uint16_t i_duration);
#endif

/** Set number of cranck wheel's teeth
 * \param norm_num Number of cranck wheel's teeth, including missing teeth (16...200)
 * \param miss_num Number of missing cranck wheel's teeth (0, 1, 2)
 */
void ckps_set_cogs_num(uint8_t norm_num, uint8_t miss_num);

#if defined(HALL_SYNC) || defined(CKPS_NPLUS1)
/** Enable/disable spark generation using shutter entering (used on startup - at low RPM)
 * Note: This function is applicable only when synchronization from Hall sensor is selected
 * \param i_shutter 1 - use shutter, 0 - don't use shutter (use timer)
 */
void ckps_set_shutter_spark(uint8_t i_shutter);

/** Sets shutter's window width in degrees of crankshaft
 * \param width Width in degrees of cranckshaft of the shutter window, the value must be > 0
 */
void ckps_set_shutter_wnd_width(int16_t width);

/** Set degrees BTDC
 * \param degrees BTDC * ANGLE_MULTIPLIER
 */
void ckps_set_degrees_btdc(int16_t degrees_btdc);
#endif

#ifdef FUEL_INJECT
/** Set injection timing relatively to TDC (value in crankshaft degrees BTDC)
 * \param phase Injection timing in degrees of wheel * ANGLE_MULTIPLIER
 * \param pw Current value of inj. PW
 * \param mode 0, 1, 2
 */
void ckps_set_inj_timing(int16_t phase, uint16_t pw, uint8_t mode);
#endif

#ifdef PHASE_SENSOR
/** Set sync. mode when cam sensor is used as reference sensor with non-missing tooth wheel.
 * \param i_camref 1 - use cam as reference sensor, 0 - use separate ref.sensor or missing tooth wheel
 */
void ckps_use_cam_ref_s(uint8_t i_camref);
#endif

#ifdef FUEL_INJECT
/** Get current period of 1 stroke*/
uint16_t ckps_get_stroke_period(void);
#endif

/** Sets bit map containing selection of KS_n for each channel (cylinder)
 * \param chanmap Bit map containing selection of KS_n for each channel (1 bit per cylinder)
 */
void ckps_set_knock_chanmap(uint8_t chanmap);

/* Sets missing teeth threshold factor, e.g. 2.5 for 60-2
 * \param mttf Factor value * 256
 */
void ckps_set_mttf(uint16_t mttf);

#ifdef FUEL_INJECT
/**Enables switching into a full sequential mode*/
void ckps_enable_fullsequential(void);
#endif

#endif //_CKPS_H_
