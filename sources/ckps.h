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

/** \file ckps.h
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
#define ANGLE_MULTIPLAYER            32

/**Initialization of CKP module (hardware & variables)
 * (инициализирет структуру данных/состояния ДПКВ и железо на которое он мапится)
 */
void ckps_init_state(void);

/** Set edge type for CKP sensor (Установить тип фронта ДПКВ)
 * \param edge_type 0 - falling (отрицательный), 1 - rising (положительный)
 */
void ckps_set_edge_type(uint8_t edge_type);

/** Set count of teeth before TDC.
 * \param cogs_btdc Valid values: 18,19,20,21,22 (for 60-2 wheel), 10,11,12,13,14 (for 36-1 wheel)
 *                 (Допустимые значения: 18,19,20,21,22 (для 60-2 шкива), 10,11,12,13,14 (для 36-1 шкива)).
 * \details If the crankshaft is in a position corresponding to top dead center (tdc) of the first cylinder's piston,
 * then according to the standards the middle of 20th tooth of sync wheel must be in the opposite to the CKP's core
 * (counting out against the direction of rotation from the place of the cutout).
 * (Если коленчатый вал установлен в положение, соответствующее верхней мертвой точке(t.d.c.) поршня первого цилиндра, то
 * по стандарту напротив середины сердечника ДПКВ должен находиться 20-й зуб диска синхронизации (считаем против
 * направления вращения от места выреза)).
 */
void ckps_set_cogs_btdc(uint8_t cogs_btdc);


#ifndef COIL_REGULATION
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
/**Coil regulation. Set accumulation time
 * \param i_acc_time accumulation time in timer's ticks (1 tick = 4uS)
 */
void ckps_set_acc_time(uint16_t i_acc_time);
#endif

/** Set andvance angle
 * (устанавливает УОЗ для реализации в алгоритме)
 * \param angle advance angle * ANGLE_MULTIPLAYER
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

/**\return 1 if there was engine cycle and resets flag!
 * (эта функция возвращает 1 если был новый цикл зажигания и сразу сбрасывает событие!)
 * \details Used to perform synchronization with rotation of crankshaft.
 */
uint8_t ckps_is_cycle_cutover_r(void);

/** Initialization of state variables */
void ckps_init_state_variables(void);

/** \return Number of current tooth (возвращает номер текущего зуба) */
uint8_t ckps_get_current_cog(void);

/** \return returns 1 if number of current tooth has been changed
 *          (возвращает 1, если номер текущего зуба изменился)
 */
uint8_t ckps_is_cog_changed(void);

/** Set number of engine's cylinders (установка кол-ва цилиндров двигателя (четное число))
 * \param i_cyl_number allowed values(допустимые значения): 2,4,6,8
 */
void ckps_set_cyl_number(uint8_t i_cyl_number);

/** Initialization of used I/O ports (производит инициализацию линий портов) */
void ckps_init_ports(void);

#endif //_CKPS_H_
