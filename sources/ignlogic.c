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

/** \file ignlogic.c
 * Implementation of logic determining calculation and regulation of anvance angle
 * (Реализация логики определяющей вычисление и регулирование угла опережения).
 */

#include "port/port.h"
#include "funconv.h"
#include "ignlogic.h"
#include "secu3.h"

/**Reserved value used to indicate that value is not used in corresponding mode*/
#define AAV_NOTUSED 0x7FFF

typedef struct
{
#ifdef FUEL_INJECT
 uint16_t aftstr_enrich_counter; //!< Stroke counter used in afterstart enrichment implementation
#endif
}logic_state_t;

/**Instance of internal state variables structure*/
static logic_state_t lgs;


void ignlogic_init(void)
{
#ifdef FUEL_INJECT
 lgs.aftstr_enrich_counter = 0;
#endif
}

int16_t ignlogic_system_state_machine(struct ecudata_t* d)
{
 int16_t angle;
 switch(d->engine_mode)
 {
  case EM_START: //режим пуска
   if (d->sens.inst_frq > d->param.smap_abandon)
   {
    d->engine_mode = EM_IDLE;
    idling_regulator_init();
#ifdef FUEL_INJECT
    lgs.aftstr_enrich_counter = d->param.inj_aftstr_strokes; //init engine strokes counter
#endif
   }
   angle = d->corr.strt_aalt = start_function(d);//базовый УОЗ - функция для пуска
   d->corr.idle_aalt = d->corr.work_aalt = d->corr.temp_aalt = d->corr.airt_aalt = d->corr.idlreg_aac = AAV_NOTUSED;
   d->airflow = 0;                         //в режиме пуска нет расхода

#ifdef FUEL_INJECT
   { //PW = CRANKING + DEADTIME
   uint32_t pw = inj_cranking_pw(d);
   pw+= inj_dead_time(d);
   }
#endif
   break;

  case EM_IDLE: //режим холостого хода
   if (d->sens.carb)//педаль газа нажали - в рабочий режим
   {
    d->engine_mode = EM_WORK;
   }
   work_function(d, 1);                    //обновляем значение расхода воздуха
   angle = d->corr.idle_aalt = idling_function(d); //базовый УОЗ - функция для ХХ
   d->corr.temp_aalt = coolant_function(d);//добавляем к УОЗ температурную коррекцию
   angle+=d->corr.temp_aalt;
#ifdef AIRTEMP_SENS
   d->corr.airt_aalt = airtemp_function(d);//add air temperature correction
   angle+=d->corr.airt_aalt;
#else
   d->corr.airt_aalt = 0;
#endif
   d->corr.idlreg_aac = idling_pregulator(d,&idle_period_time_counter);//добавляем регулировку
   angle+=d->corr.idlreg_aac;
   d->corr.strt_aalt = d->corr.work_aalt = AAV_NOTUSED;

#ifdef FUEL_INJECT
   {//PW = BASE + WARMUP + AFTSTR_ENRICH + DEADTIME
   uint32_t pw = inj_base_pw(d);
   pw = (pw * inj_warmup_en(d)) >> 7;
   if (lgs.aftstr_enrich_counter)
    pw = (pw * d->param.inj_aftstr_enrich) >> 2;
   pw+= inj_dead_time(d);
   d->inj_pw = pw > 65535 ? 65535 : pw;
   }
#endif

   break;

  case EM_WORK: //рабочий режим
   if (!d->sens.carb)//педаль газа отпустили - в переходной режим ХХ
   {
    d->engine_mode = EM_IDLE;
    idling_regulator_init();
   }

#ifdef SM_CONTROL
   //air flow will be always 1 if choke RPM regulator is active
   if (d->choke_rpm_reg)
   {
    work_function(d, 1);                    //обновляем значение расхода воздуха
    angle = d->corr.idle_aalt = idling_function(d);//базовый УОЗ - функция для ХХ
    d->corr.work_aalt = AAV_NOTUSED;
   }
   else
   {
    angle = d->corr.work_aalt = work_function(d, 0);//базовый УОЗ - функция рабочего режима
    d->corr.idle_aalt = AAV_NOTUSED;
   }
#else
   angle = d->corr.work_aalt = work_function(d, 0);//базовый УОЗ - функция рабочего режима
   d->corr.idle_aalt = AAV_NOTUSED;
#endif

   d->corr.temp_aalt = coolant_function(d);//добавляем к УОЗ температурную коррекцию;
   angle+=d->corr.temp_aalt;
#ifdef AIRTEMP_SENS
   d->corr.airt_aalt = airtemp_function(d);//add air temperature correction;
   angle+=d->corr.airt_aalt;
#else
   d->corr.airt_aalt = 0;
#endif
   //отнимаем поправку полученную от регулятора по детонации
   angle-=d->corr.knock_retard;
   d->corr.strt_aalt = d->corr.idlreg_aac = AAV_NOTUSED;

#ifdef FUEL_INJECT
   {//PW = BASE + WARMUP + AFTSTR_ENRICH + DEADTIME
   uint32_t pw = inj_base_pw(d);
   pw = (pw * inj_warmup_en(d)) >> 7;
   if (lgs.aftstr_enrich_counter)
    pw = (pw * d->param.inj_aftstr_enrich) >> 2;
   pw+= inj_dead_time(d);
   d->inj_pw = pw > 65535 ? 65535 : pw;
   }
#endif

   break;

  default:  //непонятная ситуация - угол в ноль
   angle = 0;
#ifdef FUEL_INJECT
   d->inj_pw = 0;
#endif
   break;
 }
 return angle; //return calculated advance angle
}

void ignlogic_stroke_event_notification(void)
{
#ifdef FUEL_INJECT
 if (lgs.aftstr_enrich_counter)
  --lgs.aftstr_enrich_counter;
#endif
}
