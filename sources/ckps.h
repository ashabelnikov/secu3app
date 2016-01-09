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
 * (Обработка датчика положения коленвала).
 */

#ifndef _CKPS_H_
#define _CKPS_H_

#include <stdint.h>

/**Scaling factor of crankshaft rotation angle, appears in the calculations and operations of the division
 * so it should be a multiple of degree of 2 (коэффициент масштабирования углов поворота коленвала, фигурирует
 * в вычислениях и операциях деления поэтому он должен быть кратен степени 2).
 */
#define ANGLE_MULTIPLIER   32

/**Initialization of CKP module (hardware & variables)
 * (инициализирет структуру данных/состояния ДПКВ и железо на которое он мапится)
 */
void ckps_init_state(void);

/** Set edge type for CKP sensor (Установить тип фронта ДПКВ)
 * \param edge_type 0 - falling (отрицательный), 1 - rising (положительный)
 */
void ckps_set_edge_type(uint8_t edge_type);

/** Set count of teeth before TDC.
 * \param cogs_btdc E.g values for 60-2 wheel: 17,18,19,20,21,22,23 
                               for 36-1 wheel: 8,9,10,11,12,13,14
 * \details For 4-cylinder engine. If the crankshaft is in a position corresponding to top dead center (tdc) of the first cylinder's piston,
 * then according to the standards the middle of 20th tooth of sync wheel must be in the opposite to the CKP's core
 * (counting out against the direction of rotation from the place of the cutout).
 * (Для 4-х цилиндрового двигателя. Если коленчатый вал установлен в положение, соответствующее верхней мертвой точке(t.d.c.) поршня первого цилиндра, то
 * по стандарту напротив середины сердечника ДПКВ должен находиться 20-й зуб диска синхронизации (считаем против
 * направления вращения от места выреза)).
 */
void ckps_set_cogs_btdc(uint8_t cogs_btdc);


#ifndef DWELL_CONTROL
/** Set duration of ignition pulse drive (устанавливает длительность импульса зажигания в зубьях)
 * \param cogs duration of pulse, countable in the teeth of wheel
 * \details For standard igniters duration of driving pulses must be 1/3, when significant deviation to the smaller side
 * is present then igniters can became out of action. If you connect two outputs together to one igniter, you must put
 * a value of 10, if double channel mode then 40. The values given for the 60-2 wheel and 4 cylinder engine.
 * (Для стандартных коммутаторов длительность импульса запуска должна быть 1/3, при значительном отклонении в меньшую сторону
 * возможен выход коммутатора из строя.  Если соединять два выхода вместе для одного коммутатора, то необходимо ставить
 * значение 10, если двухканальный режим то 40. Значения указаны для шкива 60-2 и 4-х цилиндрового двигателя).
 */
void ckps_set_ignition_cogs(uint8_t cogs);
#else
/**Dwell control. Set accumulation time
 * \param i_acc_time accumulation time in timer's ticks (1 tick = 4uS)
 */
void ckps_set_acc_time(uint16_t i_acc_time);
#endif

/** Set andvance angle
 * (устанавливает УОЗ для реализации в алгоритме)
 * \param angle advance angle * ANGLE_MULTIPLIER
 */
void ckps_set_advance_angle(int16_t angle);

/** Calculate instant RPM using last measured period
 * (Высчитывание мгновенной частоты вращения коленвала основываясь на последнем измеренном значении периода)
 * \return RPM (min-1)
 */
uint16_t ckps_calculate_instant_freq(void);

/** Set pahse selection window for detonation (установка окна фазовой селекции детонации)
 * \param begin begin of window (degrees relatively to t.d.c) (Начало окна в градусах относительно в.м.т)
 * \param end end of window (degrees relatively to t.d.c) (Конец окна в градусах относительно в.м.т)
 */
void ckps_set_knock_window(int16_t begin, int16_t end);

/** Set to use or not to use knock detection (устанавливает обслуживать или не обслуживать канал детонации)
 * \param use_knock_channel 1 - use, 0 - do not use
 */
void ckps_use_knock_channel(uint8_t use_knock_channel);

/** \return nonzero if error was detected */
uint8_t ckps_is_error(void);

/** Reset detected errors */
void ckps_reset_error(void);

/**\return 1 if there was engine stroke and reset flag!
 * (эта функция возвращает 1 если был новый такт зажигания и сразу сбрасывает событие!)
 * \details Used to perform synchronization with rotation of crankshaft.
 */
uint8_t ckps_is_stroke_event_r(void);

/** Initialization of state variables */
void ckps_init_state_variables(void);

/** \return returns 1 if number of current tooth has been changed
 *  (возвращает 1, если номер текущего зуба изменился)
 */
uint8_t ckps_is_cog_changed(void);

/** Set number of engine's cylinders (установка кол-ва цилиндров двигателя (четное число))
 * \param i_cyl_number allowed values(допустимые значения): *1,2,*3,4,*5,6,8
 * * these values are allowed only if firmware compliled with PHASED_IGNITION option
 */
void ckps_set_cyl_number(uint8_t i_cyl_number);

/** Initialization of used I/O ports (производит инициализацию линий портов) */
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
 * \param i_offset - offset in tooth relatively to TDC (if > 0, then BTDC)
 * \param i_duration - duration of pulse in tooth
 */
void ckps_set_hall_pulse(int8_t i_offset, uint8_t i_duration);
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
#endif

#ifdef FUEL_INJECT
/** Set injection timing relatively to TDC
 * \param phase Injection timing in degrees of wheel * ANGLE_MULTIPLIER
 */
void ckps_set_inj_timing(int16_t phase);
#endif

#endif //_CKPS_H_
