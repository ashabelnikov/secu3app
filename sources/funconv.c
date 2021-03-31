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

/** \file funconv.c
 * \author Alexey A. Shabelnikov
 * Implementation of core mathematics and regulation logic.
 */

#include "port/pgmspace.h"
#include "port/port.h"
#include <stdlib.h>
#include "bitmask.h"
#include "ckps.h"
#include "ecudata.h"
#include "funconv.h"
#include "ioconfig.h"
#include "magnitude.h"
#include "mathemat.h"
#include "vstimer.h"
#include "eculogic.h"  //EM_START

#if defined(FUEL_INJECT) && !defined(AIRTEMP_SENS)
 #error "You can not use FUEL_INJECT option without AIRTEMP_SENS"
#endif

 #define secu3_offsetof(type,member)   ((size_t)(&((type *)0)->member))
//For use with fn_dat pointer, because it can point either to FLASH or RAM
#ifdef REALTIME_TABLES
 /**Macro for abstraction under getting bytes from RAM or FLASH (RAM version) */
 #define _GB(x) ((int8_t)d.mm_ptr8(secu3_offsetof(struct f_data_t, x)))
 #define _GW(x) ((int16_t)d.mm_ptr16(secu3_offsetof(struct f_data_t, x)))
 #define _GBU(x) (d.mm_ptr8(secu3_offsetof(struct f_data_t, x)))
 #define _GWU(x) (d.mm_ptr16(secu3_offsetof(struct f_data_t, x)))
 #define _GWU12(x,i,j) (d.mm_ptr12(secu3_offsetof(struct f_data_t, x), (i*16+j) )) //note: hard coded size of array
#else
 #define _GB(x) ((int8_t)(PGM_GET_BYTE(&d.fn_dat->x)))    //!< Macro for abstraction under getting bytes from RAM or FLASH (FLASH version)
 #define _GW(x) ((int16_t)(PGM_GET_WORD(&d.fn_dat->x)))   //!< Macro for abstraction under getting words from RAM or FLASH (FLASH version)
 #define _GBU(x) (PGM_GET_BYTE(&d.fn_dat->x))             //!< Unsigned version of _GB
 #define _GWU(x) (PGM_GET_WORD(&d.fn_dat->x))             //!< Unsigned version of _GW
 #define _GWU12(x,i,j) (mm_get_w12_pgm(secu3_offsetof(struct f_data_t, x), (i*16+j))) //note: hard coded size of array
#endif

//not redundant sanity check...
#if (F_WRK_POINTS_L != INJ_VE_POINTS_L) || (F_WRK_POINTS_L != GASDOSE_POS_TPS_SIZE)
 #error "Check related code!"
#endif
#if (F_WRK_POINTS_F != INJ_VE_POINTS_F) || (F_WRK_POINTS_F != F_IDL_POINTS) || (F_WRK_POINTS_F != GASDOSE_POS_RPM_SIZE)
 #error "Check related code!"
#endif

//for CLT arguments checking
#if (F_TMP_POINTS != INJ_CRANKING_LOOKUP_TABLE_SIZE) || (F_TMP_POINTS != INJ_IAC_POS_TABLE_SIZE) || (F_TMP_POINTS != INJ_AFTSTR_LOOKUP_TABLE_SIZE) || (F_TMP_POINTS != INJ_WARMUP_LOOKUP_TABLE_SIZE) || (F_TMP_POINTS != INJ_TARGET_RPM_TABLE_SIZE)
 #error "Check related code!"
#endif

/**TPS % between two interpolation points, additionally multiplied by 16 */
#define TPS_AXIS_STEP TPS_MAGNITUDE((100.0*16)/(F_WRK_POINTS_L-1))

/**State variables, local use*/
typedef struct
{
 int16_t la_rpm;       //!< RPM axis argument
 int16_t la_load;      //!< Load axis argument
 int16_t la_grad;      //!< amount of load per one cell on the load axis
 int16_t la_l;         //!< index on the load axis
 int8_t  la_lp1;       //!< la_l + 1
 int8_t  la_f;         //!< index on the rpm axis
 int8_t  la_fp1;       //!< la_f + 1
 //CLT args:
 int8_t  ta_i;         //!< index
 int8_t  ta_i1;        //!< index + 1
 int16_t ta_clt;       //!< temperature (CLT)
#ifndef SECU3T
 //GRTS args:
 int8_t  ga_i;         //!< index
 int8_t  ga_i1;        //!< index + 1
 int16_t ga_grt;       //!< temperature (GRTS)
#endif
 //precalculated values:
 int16_t vecurr;       //!< current value of VE (value * 2048)
 int16_t afrcurr;      //!< current value of AFR (value * 256)
 //AE decay:
 uint8_t  ae_decay_counter; //!< AE decay counter
 int16_t  aef_decay;        //!< AE factor value at the start of decay
}fcs_t;

/**Instance of state variables*/
fcs_t fcs = {0};

/* Calculates synthetic load values basing on MAP, TPS and TPS switch point table
 * Uses d ECU data structure
 * \return load value in % * 64 (0...100%)
 */
static int16_t calc_synthetic_load(void)
{
 int16_t load = ROUND(50.0 * 64); //50%
 uint16_t pbaro = (((uint32_t)d.sens.baro_press) * ROUND(0.9*16384)) >> 14; //90% of barometric pressure

 if (d.sens.map < pbaro)
 { //find value on the load axis corresponding to current MAP
  load = (((int32_t) d.sens.map * load) / pbaro);
 }
 else
 {
  uint16_t swtpt = tpsswt_function(); //% * 2
  if (d.sens.tps >= swtpt) //find value on the load axis corresponding to current TPS
   load = simple_interpolation(d.sens.tps, load, ROUND(100.0*64), swtpt, (TPS_MAGNITUDE(100.0) - swtpt), 4) >> 2;
 }

 return load;
}

/** Get upper load value
 * Uses d ECU data structure
 * \return value of upper load
 */
int16_t get_load_upper(void)
{
 return ((d.param.load_src_cfg == 1) ? d.sens.baro_press : d.param.load_upper);
}

#if defined(FUEL_INJECT) || defined(GD_CONTROL)
/** Calculates air flow as rpm*load
 * Uses d ECU data structure
 * returns air flow value / 32
 */
static uint16_t calc_airflow(void)
{
 uint32_t x_raw = ((int32_t)d.sens.inst_frq * d.load) >> (6+5); //value / 32
 x_raw = (x_raw * fcs.vecurr) >> 11; //apply VE
 return (x_raw > 65535) ? 65535 : x_raw;
}
#endif

/**Calculates argument values needed for some 2d and 3d lookup tables. Fills la_x values in the fcs_t structure
 * Uses d ECU data structure
 * Note! This function must be called before any other function which expects precalculated results
 */
void calc_lookup_args(void)
{
 //-----------------------------------------
 fcs.la_rpm = d.sens.inst_frq;

 //find interpolation points, then restrict RPM if it fall outside set range
 for(fcs.la_f = F_WRK_POINTS_F-2; fcs.la_f >= 0; fcs.la_f--)
  if (fcs.la_rpm >= PGM_GET_WORD(&fw_data.exdata.rpm_grid_points[fcs.la_f])) break;

 //lookup table works from rpm_grid_points[0] and upper
 if (fcs.la_f < 0)  {fcs.la_f = 0; fcs.la_rpm = PGM_GET_WORD(&fw_data.exdata.rpm_grid_points[0]);}
 if (fcs.la_rpm > PGM_GET_WORD(&fw_data.exdata.rpm_grid_points[F_WRK_POINTS_F-1])) fcs.la_rpm = PGM_GET_WORD(&fw_data.exdata.rpm_grid_points[F_WRK_POINTS_F-1]);
 fcs.la_fp1 = fcs.la_f + 1;

 //-----------------------------------------
 if (d.param.load_src_cfg < 2)       //Speed-density or Speed-density (baro)
  d.load = d.sens.map;
 else if (d.param.load_src_cfg == 2) //Alpha-N
  d.load = d.sens.tps << 5;
 else                                //mixed (MAP+TPS)
  d.load = calc_synthetic_load();

 if (CHECKBIT(d.param.func_flags, FUNC_LDAX_GRID))
 { //use grid table
  fcs.la_load = d.load;

  if (fcs.la_load > PGM_GET_WORD(&fw_data.exdata.load_grid_points[0]))
   fcs.la_load = PGM_GET_WORD(&fw_data.exdata.load_grid_points[0]);
  if (fcs.la_load < PGM_GET_WORD(&fw_data.exdata.load_grid_points[F_WRK_POINTS_L-1]))
   fcs.la_load = PGM_GET_WORD(&fw_data.exdata.load_grid_points[F_WRK_POINTS_L-1]);

  for(fcs.la_l = 1; fcs.la_l < F_WRK_POINTS_L; ++fcs.la_l)
   if (fcs.la_load >= PGM_GET_WORD(&fw_data.exdata.load_grid_points[fcs.la_l])) break;
  fcs.la_lp1 = fcs.la_l - 1;

  //update air flow variable (find nearest point)
  if (fcs.la_load < (PGM_GET_WORD(&fw_data.exdata.load_grid_points[fcs.la_lp1]) - (PGM_GET_WORD(&fw_data.exdata.load_grid_sizes[fcs.la_lp1]) / 2)))
   d.airflow = (F_WRK_POINTS_L-1) - fcs.la_lp1;
  else
   d.airflow = F_WRK_POINTS_L - fcs.la_lp1;
 }
 else
 {
  //Calculate arguments for load axis:
  fcs.la_load = (get_load_upper() - d.load);
  if (fcs.la_load < 0) fcs.la_load = 0;

  //load_upper - value of the upper load, load_lower - value of the lower load
  //todo: replace division by 1/x multiplication
  fcs.la_grad = (get_load_upper() - d.param.load_lower) / (F_WRK_POINTS_L - 1); //divide by number of points on the load axis - 1
  if (fcs.la_grad < 1)
   fcs.la_grad = 1;  //exclude division by zero and negative value in case when upper pressure < lower pressure

  fcs.la_l = (fcs.la_load / fcs.la_grad);

  if (fcs.la_l >= (F_WRK_POINTS_L - 1))
   fcs.la_lp1 = fcs.la_l = F_WRK_POINTS_L - 1;
  else
   fcs.la_lp1 = fcs.la_l + 1;

  //update air flow variable (find nearest point)
  if (fcs.la_load < ((fcs.la_grad * fcs.la_lp1) - (fcs.la_grad / 2)))
   d.airflow = (F_WRK_POINTS_L+1) - fcs.la_lp1;
  else
   d.airflow = F_WRK_POINTS_L - fcs.la_lp1;
 }

 //-----------------------------------------
 //Coolant temperature arguments:
 fcs.ta_clt = d.sens.temperat;

 //find interpolation points, then restrict CLT if it fall outside set range
 for(fcs.ta_i = F_TMP_POINTS-2; fcs.ta_i >= 0; fcs.ta_i--)
  if (fcs.ta_clt >= ((int16_t)PGM_GET_WORD(&fw_data.exdata.clt_grid_points[fcs.ta_i]))) break;

 //lookup table works from clt_grid_points[0] and upper
 if (fcs.ta_i < 0)  {fcs.ta_i = 0; fcs.ta_clt = PGM_GET_WORD(&fw_data.exdata.clt_grid_points[0]);}
 if (fcs.ta_clt > ((int16_t)PGM_GET_WORD(&fw_data.exdata.clt_grid_points[F_TMP_POINTS-1]))) fcs.ta_clt = PGM_GET_WORD(&fw_data.exdata.clt_grid_points[F_TMP_POINTS-1]);
 fcs.ta_i1 = fcs.ta_i + 1;

#if !defined(SECU3T) && defined(MCP3204)
 //-----------------------------------------
 //GRTS arguments:
 fcs.ga_grt = d.sens.grts;

 //find interpolation points, then restrict CLT if it fall outside set range
 for(fcs.ga_i = F_TMP_POINTS-2; fcs.ga_i >= 0; fcs.ga_i--)
  if (fcs.ga_grt >= ((int16_t)PGM_GET_WORD(&fw_data.exdata.clt_grid_points[fcs.ga_i]))) break;

 //lookup table works from clt_grid_points[0] and upper
 if (fcs.ga_i < 0)  {fcs.ga_i = 0; fcs.ga_grt = PGM_GET_WORD(&fw_data.exdata.clt_grid_points[0]);}
 if (fcs.ga_grt > ((int16_t)PGM_GET_WORD(&fw_data.exdata.clt_grid_points[F_TMP_POINTS-1]))) fcs.ga_grt = PGM_GET_WORD(&fw_data.exdata.clt_grid_points[F_TMP_POINTS-1]);
 fcs.ga_i1 = fcs.ga_i + 1;
#endif

//-------------------------------------------
#if defined(FUEL_INJECT) || defined(GD_CONTROL)
 d.sens.rxlaf = calc_airflow();
#endif
}

// Implements function of ignition timing vs RPM for idling
// Returns ignition timing value * 32 (2 * 16 = 32).
int16_t idling_function(void)
{
 return simple_interpolation(fcs.la_rpm, _GB(f_idl[fcs.la_f]), _GB(f_idl[fcs.la_fp1]),
        PGM_GET_WORD(&fw_data.exdata.rpm_grid_points[fcs.la_f]), PGM_GET_WORD(&fw_data.exdata.rpm_grid_sizes[fcs.la_f]), 16);
}


// Реализует функцию УОЗ от оборотов для пуска двигателя
// Возвращает значение угла опережения в целом виде * 32, 2 * 16 = 32.
int16_t start_function(void)
{
 int16_t i, i1, rpm = d.sens.inst_frq;

 if (rpm < 200) rpm = 200; //200 - minimal value on the RPM axis

 i = (rpm - 200) / 40;   //40 - step on the RPM axis

 if (i >= F_STR_POINTS-1) i = i1 = F_STR_POINTS-1;
  else i1 = i + 1;

 return simple_interpolation(rpm, _GB(f_str[i]), _GB(f_str[i1]), (i * 40) + 200, 40, 16);
}

// Реализует функцию УОЗ от оборотов(мин-1) и нагрузки(кПа) для рабочего режима двигателя
// Возвращает значение угла опережения в целом виде * 32, 2 * 16 = 32.
int16_t work_function(void)
{
 uint8_t use_grid = CHECKBIT(d.param.func_flags, FUNC_LDAX_GRID);
 return bilinear_interpolation(fcs.la_rpm, fcs.la_load,
        _GB(f_wrk[fcs.la_l][fcs.la_f]),
        _GB(f_wrk[fcs.la_lp1][fcs.la_f]),
        _GB(f_wrk[fcs.la_lp1][fcs.la_fp1]),
        _GB(f_wrk[fcs.la_l][fcs.la_fp1]),
        PGM_GET_WORD(&fw_data.exdata.rpm_grid_points[fcs.la_f]),
        use_grid ? PGM_GET_WORD(&fw_data.exdata.load_grid_points[fcs.la_l]) : (fcs.la_grad * fcs.la_l),
        PGM_GET_WORD(&fw_data.exdata.rpm_grid_sizes[fcs.la_f]),
        use_grid ? PGM_GET_WORD(&fw_data.exdata.load_grid_sizes[fcs.la_lp1]) : fcs.la_grad, 16);
}

//Реализует функцию коррекции УОЗ по температуре(град. Цельсия) охлаждающей жидкости
// Возвращает значение угла опережения в целом виде * 32, 2 * 16 = 32.
int16_t coolant_function(uint8_t mode)
{
 if (!CHECKBIT(d.param.tmp_flags, TMPF_CLT_USE))
  return 0;   //no correction if CLT sensor is turned off

 if (mode) //work mode
  return simple_interpolation(fcs.ta_clt, _GB(f_tmp[fcs.ta_i]), _GB(f_tmp[fcs.ta_i1]),
        PGM_GET_WORD(&fw_data.exdata.clt_grid_points[fcs.ta_i]), PGM_GET_WORD(&fw_data.exdata.clt_grid_sizes[fcs.ta_i]), 16);
 else //idling mode
  return simple_interpolation(fcs.ta_clt, _GB(f_tmp_idl[fcs.ta_i]), _GB(f_tmp_idl[fcs.ta_i1]),
        PGM_GET_WORD(&fw_data.exdata.clt_grid_points[fcs.ta_i]), PGM_GET_WORD(&fw_data.exdata.clt_grid_sizes[fcs.ta_i]), 16);
}

int16_t crkclt_function(void)
{
 if (!CHECKBIT(d.param.tmp_flags, TMPF_CLT_USE))
  return 0;   //no correction if CLT sensor is turned off

 return simple_interpolation(fcs.ta_clt, (int8_t)PGM_GET_BYTE(&fw_data.exdata.cts_crkcorr[fcs.ta_i]), (int8_t)PGM_GET_BYTE(&fw_data.exdata.cts_crkcorr[fcs.ta_i1]),
        PGM_GET_WORD(&fw_data.exdata.clt_grid_points[fcs.ta_i]), PGM_GET_WORD(&fw_data.exdata.clt_grid_sizes[fcs.ta_i]), 16);
}

//Idling regulator
/**Describes state data for idling regulator */
typedef struct
{
 //память регулятора для хранения последнего значения управляющего воздействия (коррекции)
 int16_t output_state;   //!< regulator's memory
 uint8_t enter_state;    //!< used for entering delay implementation
}idlregul_state_t;

/**Variable. State data for idling regulator */
idlregul_state_t idl_prstate;

//сброс состояния РХХ
void idling_regulator_init(void)
{
 idl_prstate.output_state = 0;
 idl_prstate.enter_state = 0;
}

//Интегральный регулятор (интегрирование по выходу) для регулирования оборотов ХХ углом опережения зажигания
// Возвращает значение угла опережения в целом виде * 32.
int16_t idling_pregulator(volatile s_timer8_t* io_timer)
{
 int16_t error,factor,idling_rpm;
 #define IRUSDIV 1

 //если PXX отключен или обороты значительно выше от нормальных холостых оборотов
 // или двигатель не прогрет то выходим  с нулевой корректировкой
 if (!CHECKBIT(d.param.idl_flags, IRF_USE_REGULATOR) || (d.sens.temperat < d.param.idlreg_turn_on_temp && CHECKBIT(d.param.tmp_flags, TMPF_CLT_USE)
#ifdef FUEL_INJECT
  && d.param.idling_rpm  //Don't use temperature turn on threshold if lookup table used
#endif
    ))
  return 0;

 //don't use regulator on gas if specified in parameters
 if (d.sens.gas && !CHECKBIT(d.param.idl_flags, IRF_USE_REGONGAS))
  return 0;

#ifdef FUEL_INJECT
 //if param.idling_rpm == 0 then use target RPM from lookup table
 idling_rpm = d.param.idling_rpm ? d.param.idling_rpm : inj_idling_rpm();
#else
 idling_rpm = d.param.idling_rpm;
#endif

 //regulator will be turned on with delay
 if (d.sens.frequen < (idling_rpm + PGM_GET_WORD(&fw_data.exdata.idlreg_captrange)))
 {
  switch(idl_prstate.enter_state)
  {
   case 0:
    s_timer_set(*io_timer, PGM_GET_BYTE(&fw_data.exdata.idlent_timval));
    ++idl_prstate.enter_state; //=1
    return 0; //no correction
   case 1:
    if (s_timer_is_action(*io_timer))
    {
     ++idl_prstate.enter_state; //=2
     s_timer_set(*io_timer, IDLE_PERIOD_TIME_VALUE);
     break;
    }
    return 0; //no correction
  }
 }
 else
  return 0; //no correction

 //вычисляем значение ошибки, ограничиваем ошибку (если нужно), а также, если мы в зоне
 //нечувствительности, то используем расчитанную ранее коррекцию.
 error = idling_rpm - d.sens.frequen;
 restrict_value_to(&error, -200, 200);
 if (abs(error) <= d.param.MINEFR)
  return idl_prstate.output_state >> IRUSDIV;

 //select corresponding coefficient depending on sign of error
 if (error > 0)
  factor = d.param.ifac1;
 else
  factor = d.param.ifac2;

 //update regulator's value only by timer events
 if (s_timer_is_action(*io_timer))
 {
  s_timer_set(*io_timer, IDLE_PERIOD_TIME_VALUE);

  if (CHECKBIT(d.param.idl_flags, IRF_PREG_MODE))
   idl_prstate.output_state = (((int32_t)error) * factor) >> 8; //P-regulator mode
  else
   idl_prstate.output_state+= (((int32_t)error) * factor) >> 8; //factor multiplied by 256
 }
 //limit correction by min and max values, specified by user
 restrict_value_to(&idl_prstate.output_state, d.param.idlreg_min_angle << IRUSDIV, d.param.idlreg_max_angle << IRUSDIV);

 return idl_prstate.output_state >> IRUSDIV;
}

//Нелинейный фильтр ограничивающий скорость изменения УОЗ на переходных режимах двигателя
//new_advance_angle - новое значение УОЗ
//ip_prev_state - значение УОЗ в предыдущем цикле
//intstep_p,intstep_m - значения положительного и отрицательного шагов интегрирования, положительные числа
//Возвращает скорректированный УОЗ
int16_t advance_angle_inhibitor(int16_t new_advance_angle, int16_t* ip_prev_state, int16_t intstep_p, int16_t intstep_m)
{
 int16_t difference;
 difference = new_advance_angle - *ip_prev_state;

 if (difference > intstep_p)
 {
  (*ip_prev_state)+= intstep_p;
  return *ip_prev_state;
 }

 if (difference < -intstep_m)
 {
  (*ip_prev_state)-= intstep_m;
  return *ip_prev_state;
 }

 //текущий УОЗ будет предыдущим в следующий раз
 *ip_prev_state = new_advance_angle;
 return *ip_prev_state;
}

// Реализует функцию коэффициента усиления аттенюатора от оборотов
// Возвращает код 0...63 соответсутвующий определенному коэфф. усиления
//(см. HIP9011 datasheet).
uint8_t knock_attenuator_function()
{
 int16_t i, i1, rpm = d.sens.inst_frq;

 if (rpm < 200) rpm = 200; //200 - минимальное значение оборотов по оси

 i = (rpm - 200) / 60;   //60 - шаг по оборотам

 if (i >= (KC_ATTENUATOR_LOOKUP_TABLE_SIZE-1))
  i = i1 = (KC_ATTENUATOR_LOOKUP_TABLE_SIZE-1);
 else
  i1 = i + 1;

 return simple_interpolation(rpm, PGM_GET_BYTE(&fw_data.exdata.attenuator_table[i]),
        PGM_GET_BYTE(&fw_data.exdata.attenuator_table[i1]), (i * 60) + 200, 60, 16) >> 4;
}

#if defined(DWELL_CONTROL) || defined(FUEL_INJECT)

#if (INJ_DT_LOOKUP_TABLE_SIZE != COIL_ON_TIME_LOOKUP_TABLE_SIZE)
 #error "Check related code"
#endif

uint16_t accumulation_time(uint8_t mode)
{
 int16_t i, i1, voltage = d.sens.voltage;

 if (voltage < VOLTAGE_MAGNITUDE(5.4))
  voltage = VOLTAGE_MAGNITUDE(5.4); //5.4 -  minimum voltage value corresponding to 1st value in table for 12V board voltage

 i = (voltage - VOLTAGE_MAGNITUDE(5.4)) / VOLTAGE_MAGNITUDE(0.4);   //0.4 - voltage step

 if (i >= COIL_ON_TIME_LOOKUP_TABLE_SIZE-1) i = i1 = COIL_ON_TIME_LOOKUP_TABLE_SIZE-1;
  else i1 = i + 1;

#ifdef FUEL_INJECT
 if (mode) //dead time
  return simple_interpolation(voltage, _GWU(inj_dead_time[i]), _GWU(inj_dead_time[i1]),  //<--values in table are unsigned
        (i * VOLTAGE_MAGNITUDE(0.4)) + VOLTAGE_MAGNITUDE(5.4), VOLTAGE_MAGNITUDE(0.4), 8) >> 3;
 else //dwell time
#endif
  return simple_interpolation(voltage, PGM_GET_WORD(&fw_data.exdata.coil_on_time[i]), PGM_GET_WORD(&fw_data.exdata.coil_on_time[i1]),
        (i * VOLTAGE_MAGNITUDE(0.4)) + VOLTAGE_MAGNITUDE(5.4), VOLTAGE_MAGNITUDE(0.4), 4) >> 2;
}
#endif

#if defined(THERMISTOR_CS) || defined(AIRTEMP_SENS) || !defined(SECU3T)
int16_t thermistor_lookup(uint16_t adcvalue, int16_t _PGM *lutab)
{
 int16_t i, i1;

 //Voltage value at the start of axis in ADC discretes
 uint16_t v_start = PGM_GET_WORD(&lutab[THERMISTOR_LOOKUP_TABLE_SIZE]);
 //Voltage value at the end of axis in ADC discretes
 uint16_t v_end = PGM_GET_WORD(&lutab[THERMISTOR_LOOKUP_TABLE_SIZE+1]);

 uint16_t v_step = (v_end - v_start) / (THERMISTOR_LOOKUP_TABLE_SIZE - 1);

 if (adcvalue < v_start)
  adcvalue = v_start;

 i = (adcvalue - v_start) / v_step;

 if (i >= THERMISTOR_LOOKUP_TABLE_SIZE-1) i = i1 = THERMISTOR_LOOKUP_TABLE_SIZE-1;
 else i1 = i + 1;

 return (simple_interpolation(adcvalue, (int16_t)PGM_GET_WORD(&lutab[i]), (int16_t)PGM_GET_WORD(&lutab[i1]), //<--values in table are signed
        (i * v_step) + v_start, v_step, 16)) >> 4;
}
#endif

#if defined(SM_CONTROL) && !defined(FUEL_INJECT)
/**Describes state data for idling regulator */
typedef struct
{
 int16_t int_state;   //!< regulator's memory (integrated error)
}chokeregul_state_t;

/**Variable. State data for choke RPM regulator */
chokeregul_state_t choke_regstate;

//reset of choke RPM regulator state
void chokerpm_regulator_init(void)
{
 choke_regstate.int_state = 0;
}

int16_t choke_rpm_regulator(int16_t* p_prev_corr)
{
 int16_t error, rpm;

 if (!CHECKBIT(d.param.choke_flags, CKF_USECLRPMREG))
 {
  *p_prev_corr = 0;
  return 0; //regulator is turned off, return zero correction
 }

 //calculate target RPM value for regulator
 rpm = inj_idling_rpm();

 error = rpm - d.sens.frequen;
 if (abs(error) <= 50)   //dead band is +/-50 RPM
  return *p_prev_corr;

 choke_regstate.int_state+= error >> 2; //update integrator's state
 restrict_value_to(&choke_regstate.int_state, -28000, 28000); //restrict integrаtor output

 *p_prev_corr = (((int32_t)d.param.choke_rpm_if) * choke_regstate.int_state) >> 12; //additional 4 shift bits to reduce regulator's influence
/* if (0)
 {
  #define _PROPFACT(x) ((int16_t)(x * 8))
  (*p_prev_corr)+= (error * _PROPFACT(0.5)) >> 3; //proportional part
 }*/
 restrict_value_to(p_prev_corr, -d.param.sm_steps, d.param.sm_steps); //range must be: +/- d.param.sm_steps

 return *p_prev_corr;
}

uint16_t choke_cranking_time(void)
{
 int16_t t = d.sens.temperat; //clt

 if (!CHECKBIT(d.param.tmp_flags, TMPF_CLT_USE))
  return d.param.choke_corr_time[0];   //coolant temperature sensor is not enabled (or not installed), use low temp. value

 //-30 CLT corresponding to first value of time
 // 40 CLT corresponding to second value of time
 restrict_value_to(&t, TEMPERATURE_MAGNITUDE(-30), TEMPERATURE_MAGNITUDE(40));

 //calculate time using an interpolation
 return simple_interpolation(t, d.param.choke_corr_time[0], d.param.choke_corr_time[1],
 TEMPERATURE_MAGNITUDE(-30), TEMPERATURE_MAGNITUDE(70), 4) >> 2;
}

#endif

#ifdef AIRTEMP_SENS
//Реализует функцию коррекции УОЗ по температуре воздуха(°C)
// Возвращает значение угла опережения в целом виде * 32
int16_t airtemp_function(void)
{
 int16_t i, i1, t = d.sens.air_temp;

 if (!IOCFG_CHECK(IOP_AIR_TEMP))
  return 0;   //do not use correcton if air temperature sensor is turned off

 //-30 - minimum temperature value
 if (t < TEMPERATURE_MAGNITUDE(-30))
  t = TEMPERATURE_MAGNITUDE(-30);

 //10 - step between interpolation points
 i = (t - TEMPERATURE_MAGNITUDE(-30)) / TEMPERATURE_MAGNITUDE(10);

 if (i >= ATS_CORR_LOOKUP_TABLE_SIZE-1) i = i1 = ATS_CORR_LOOKUP_TABLE_SIZE-1;
 else i1 = i + 1;

 return simple_interpolation(t, (int8_t)PGM_GET_BYTE(&fw_data.exdata.ats_corr[i]), (int8_t)PGM_GET_BYTE(&fw_data.exdata.ats_corr[i1]), //<--values in table are signed
 (i * TEMPERATURE_MAGNITUDE(10)) + TEMPERATURE_MAGNITUDE(-30), TEMPERATURE_MAGNITUDE(10), 16);
}

#endif //AIRTEMP_SENS

#if defined(FUEL_INJECT) || defined(GD_CONTROL)
uint16_t inj_cranking_pw(void)
{
 if (!CHECKBIT(d.param.tmp_flags, TMPF_CLT_USE))
  return 1000;   //coolant temperature sensor is not enabled, default is 3.2mS

 return simple_interpolation(fcs.ta_clt, _GWU(inj_cranking[fcs.ta_i]), _GWU(inj_cranking[fcs.ta_i1]),  //<--values in table are unsigned
        PGM_GET_WORD(&fw_data.exdata.clt_grid_points[fcs.ta_i]), PGM_GET_WORD(&fw_data.exdata.clt_grid_sizes[fcs.ta_i]), 1) /*>> 0*/;
}

void calc_ve_afr(void)
{
 uint8_t use_grid = CHECKBIT(d.param.func_flags, FUNC_LDAX_GRID);
 if (d.sens.carb || (!d.sens.gas && !PGM_GET_WORD(&fw_data.exdata.idl_ve)) || (d.sens.gas && !PGM_GET_WORD(&fw_data.exdata.idl_ve_g)))
 {//look into VE table
  fcs.vecurr = bilinear_interpolation(fcs.la_rpm, fcs.la_load,
        _GWU12(inj_ve,fcs.la_l,fcs.la_f),   //values in table are unsigned (12-bit!)
        _GWU12(inj_ve,fcs.la_lp1,fcs.la_f),
        _GWU12(inj_ve,fcs.la_lp1,fcs.la_fp1),
        _GWU12(inj_ve,fcs.la_l,fcs.la_fp1),
        PGM_GET_WORD(&fw_data.exdata.rpm_grid_points[fcs.la_f]),
        use_grid ? PGM_GET_WORD(&fw_data.exdata.load_grid_points[fcs.la_l]) : (fcs.la_grad * fcs.la_l),
        PGM_GET_WORD(&fw_data.exdata.rpm_grid_sizes[fcs.la_f]),
        use_grid ? PGM_GET_WORD(&fw_data.exdata.load_grid_sizes[fcs.la_l]) : fcs.la_grad, 8) >> 3;

  if (d.param.ve2_map_func != VE2MF_1ST)
  {
   int16_t tps = (TPS_MAGNITUDE(100.0) - d.sens.tps) * 16;
   int8_t t = (tps / TPS_AXIS_STEP), tp1;

   if (t >= (GASDOSE_POS_TPS_SIZE - 1))
    tp1 = t = GASDOSE_POS_TPS_SIZE - 1;
   else
    tp1 = t + 1;

   int16_t ve2 = bilinear_interpolation(fcs.la_rpm, tps,  //note that tps is additionally multiplied by 16
        _GWU12(inj_ve2, t, fcs.la_f),
        _GWU12(inj_ve2, tp1, fcs.la_f),
        _GWU12(inj_ve2, tp1, fcs.la_fp1),
        _GWU12(inj_ve2, t, fcs.la_fp1),
        PGM_GET_WORD(&fw_data.exdata.rpm_grid_points[fcs.la_f]),
        (TPS_AXIS_STEP*t),
        PGM_GET_WORD(&fw_data.exdata.rpm_grid_sizes[fcs.la_f]),
        TPS_AXIS_STEP, 8) >> 3;

   if (d.param.ve2_map_func == VE2MF_MUL)
    fcs.vecurr = (((int32_t)fcs.vecurr) * ve2) >> 11;
   if (d.param.ve2_map_func == VE2MF_ADD)
    fcs.vecurr = fcs.vecurr + ve2;
  }
 }
 else
  fcs.vecurr = d.sens.gas ? PGM_GET_WORD(&fw_data.exdata.idl_ve_g) : PGM_GET_WORD(&fw_data.exdata.idl_ve);

 //look into AFR table
 fcs.afrcurr = bilinear_interpolation(fcs.la_rpm, fcs.la_load,
        _GBU(inj_afr[fcs.la_l][fcs.la_f]),  //values in table are unsigned
        _GBU(inj_afr[fcs.la_lp1][fcs.la_f]),
        _GBU(inj_afr[fcs.la_lp1][fcs.la_fp1]),
        _GBU(inj_afr[fcs.la_l][fcs.la_fp1]),
        PGM_GET_WORD(&fw_data.exdata.rpm_grid_points[fcs.la_f]),
        use_grid ? PGM_GET_WORD(&fw_data.exdata.load_grid_points[fcs.la_l]) : (fcs.la_grad * fcs.la_l),
        PGM_GET_WORD(&fw_data.exdata.rpm_grid_sizes[fcs.la_f]),
        use_grid ? PGM_GET_WORD(&fw_data.exdata.load_grid_sizes[fcs.la_l]) : fcs.la_grad, 16);
 fcs.afrcurr+=(8*256);

 d.corr.afr = fcs.afrcurr >> 1; //update value of AFR
}


/** Calculates corrected MAT based on the coefficient from a lookup table, IAT and CTS sensors
 * \param d Pointer to ECU data structure
 * \return corrected MAT value (temperature units, Celsius)
 */
static int16_t inj_corrected_mat(void)
{
 int16_t i, i1; uint16_t x = d.sens.rxlaf;
 //x contains value of air flow * 32

 //air flow value at the start of axis
 uint16_t x_start = _GWU(inj_iatclt_corr[INJ_IATCLT_CORR_SIZE]);
 //air flow value at the end of axis
 uint16_t x_end = _GWU(inj_iatclt_corr[INJ_IATCLT_CORR_SIZE+1]);

 //additionally divide x by 2, because simple_interpolation is limited to 32767
 x = x >> 1;
 x_start = x_start >> 1;
 x_end = x_end >> 1;

 uint16_t x_step = (x_end - x_start) / (INJ_IATCLT_CORR_SIZE - 1);

 if (x < x_start)
  x = x_start;

 uint16_t coeff = 0;
 if (x_step > 0)
 {
  i = (x - x_start) / x_step;

  if (i >= INJ_IATCLT_CORR_SIZE-1) i = i1 = INJ_IATCLT_CORR_SIZE-1;
  else i1 = i + 1;

  coeff = (simple_interpolation(x, _GWU(inj_iatclt_corr[i]), _GWU(inj_iatclt_corr[i1]), //<--values in table are unsigned
          (i * x_step) + x_start, x_step, 2));
 }

 //Corrected MAT = (CLT - IAT) * coefficient(load*rpm) + IAT,
 //at this point coefficient is multiplied by 16384
 return (int16_t)(((int32_t)(d.sens.temperat - d.sens.air_temp) * coeff) >> (13+1)) + d.sens.air_temp;
}

#endif

#ifdef FUEL_INJECT

uint16_t inj_base_pw(void)
{
 uint32_t pw32;
 uint8_t  nsht = 0; //no division
 //if air density correction map used, then use normal conditions MAT = 20°С instead if real MAT
 int16_t CorrectedMAT = CHECKBIT(d.param.inj_flags, INJFLG_USEAIRDEN) ? TEMPERATURE_MAGNITUDE(293.15) : (inj_corrected_mat() + TEMPERATURE_MAGNITUDE(273.15));

 if (d.param.load_src_cfg == 2) //Alpha-N
 {
  if (PGM_GET_BYTE(&fw_data.exdata.an_tps_mul)==1)
   pw32 = (((uint32_t)(d.sens.tps << 5)) * d.param.inj_sd_igl_const[d.sens.gas]) / CorrectedMAT;
  else
  { //0 or 2
   uint16_t map = (PGM_GET_BYTE(&fw_data.exdata.an_tps_mul)==2) ? PRESSURE_MAGNITUDE(101.5) : d.sens.map;
   pw32 = (((uint32_t)(map >> nsht)) * d.param.inj_sd_igl_const[d.sens.gas]) / CorrectedMAT;
  }
 }
 else //Speed-density or mixed (Speed-density + Alpha-N)
 {
  //Calculate basic pulse width. Calculations are based on the ideal gas law and precalulated constant
  //All Ideal gas law arguments except MAP and air temperature were drove in to the constant, this dramatically increases performance
  //Note that inj_sd_igl_const constant must not exceed 524288

  if (d.sens.map > PRESSURE_MAGNITUDE(250.0))
   nsht = 2;        //pressure will be divided by 4
  else if (d.sens.map > PRESSURE_MAGNITUDE(125.0))
   nsht = 1;        //pressure will be divided by 2

  pw32 = ((uint32_t)(d.sens.map >> nsht) * d.param.inj_sd_igl_const[d.sens.gas]) / CorrectedMAT;
 }

 pw32>>=(4-nsht);  //after this shift pw32 value is basic pulse width, nsht compensates previous divide of MAP

 //apply VE
 pw32 = (pw32 * fcs.vecurr) >> 11;

 //apply AFR
 pw32=(pw32 * nr_1x_afr(fcs.afrcurr << 2)) >> 15; //apply AFR table

 //return restricted value (16 bit)
 return ((pw32 > 65535) ? 65535 : pw32);
}

int16_t inj_timing_lookup(void)
{
 uint8_t use_grid = CHECKBIT(d.param.func_flags, FUNC_LDAX_GRID);
 int16_t it = bilinear_interpolation(fcs.la_rpm, fcs.la_load,
        _GWU12(inj_timing,fcs.la_l,fcs.la_f),
        _GWU12(inj_timing,fcs.la_lp1,fcs.la_f),
        _GWU12(inj_timing,fcs.la_lp1,fcs.la_fp1),
        _GWU12(inj_timing,fcs.la_l,fcs.la_fp1),
        PGM_GET_WORD(&fw_data.exdata.rpm_grid_points[fcs.la_f]),
        use_grid ? PGM_GET_WORD(&fw_data.exdata.load_grid_points[fcs.la_l]) : (fcs.la_grad * fcs.la_l),
        PGM_GET_WORD(&fw_data.exdata.rpm_grid_sizes[fcs.la_f]),
        use_grid ? PGM_GET_WORD(&fw_data.exdata.load_grid_sizes[fcs.la_l]) : fcs.la_grad, 8);
 if (it > ROUND(720.0*16))
  it-=ROUND(720.0*16);
 return (it << 1);
}

int16_t param_inj_timing(uint8_t mode)
{
 int16_t it = mode ? d.param.inj_timing[d.sens.gas] : d.param.inj_timing_crk[d.sens.gas];
 if (it > ROUND(720.0*16))
  it-=ROUND(720.0*16);
 return (it << 1);
}

#endif //FUEL_INJECT

#if defined(FUEL_INJECT) || defined(SM_CONTROL)
void inj_init_prev_clt(prev_temp_t* p_pt)
{
 p_pt->clt = fcs.ta_clt;
 p_pt->i = fcs.ta_i;
 p_pt->i1 = fcs.ta_i1;
}

uint8_t inj_iac_pos_lookup(prev_temp_t* p_pt, uint8_t mode)
{
 uint8_t i = fcs.ta_i, i1 = fcs.ta_i1;
 int16_t t = fcs.ta_clt;

 if (!CHECKBIT(d.param.tmp_flags, TMPF_CLT_USE))
  return 0;   //CLT sensor is turned off

 //if difference between current and previous temperature values is less than +/-0.5,
 //then previous value will be used for calculations.
 if (abs(p_pt->clt - t) < TEMPERATURE_MAGNITUDE(0.25))
 {
  t = p_pt->clt;
  i = p_pt->i;
  i1 = p_pt->i1;
 }
 else
 {
  inj_init_prev_clt(p_pt); //make it current
 }

#ifndef FUEL_INJECT
 if (0==(mode ? _GBU(inj_iac_run_pos[i1]) : _GBU(inj_iac_crank_pos[i1])))
  return 0; //asked by alvikagal: open choke fully if next value in the lookup table is zero
 else
#endif
 //run/cranking
 return simple_interpolation(t, mode ? _GBU(inj_iac_run_pos[i]) : _GBU(inj_iac_crank_pos[i]), mode ? _GBU(inj_iac_run_pos[i1]) : _GBU(inj_iac_crank_pos[i1]),  //<--values in table are unsigned
  PGM_GET_WORD(&fw_data.exdata.clt_grid_points[i]), PGM_GET_WORD(&fw_data.exdata.clt_grid_sizes[i]), 16) >> 4;
}
#endif

#if defined(FUEL_INJECT) || defined(GD_CONTROL)
uint8_t inj_aftstr_en(void)
{
 if (!CHECKBIT(d.param.tmp_flags, TMPF_CLT_USE))
  return 0;   //coolant temperature sensor is not enabled (or not installed), no afterstart enrichment

 return simple_interpolation(fcs.ta_clt, _GBU(inj_aftstr[fcs.ta_i]), _GBU(inj_aftstr[fcs.ta_i1]),  //<--values in table are unsigned
        PGM_GET_WORD(&fw_data.exdata.clt_grid_points[fcs.ta_i]), PGM_GET_WORD(&fw_data.exdata.clt_grid_sizes[fcs.ta_i]), 16) >> 4;
}

uint8_t inj_warmup_en(void)
{
 if (!CHECKBIT(d.param.tmp_flags, TMPF_CLT_USE))
  return 128;   //coolant temperature sensor is not enabled (or not installed), no warmup enrichment

 return simple_interpolation(fcs.ta_clt, _GBU(inj_warmup[fcs.ta_i]), _GBU(inj_warmup[fcs.ta_i1]),  //<--values in table are unsigned
        PGM_GET_WORD(&fw_data.exdata.clt_grid_points[fcs.ta_i]), PGM_GET_WORD(&fw_data.exdata.clt_grid_sizes[fcs.ta_i]), 16) >> 4;
}

int16_t inj_ae_tps_lookup(int16_t tpsdot)
{
 int8_t i;

 for(i = INJ_AE_TPS_LOOKUP_TABLE_SIZE-2; i >= 0; i--)
  if (tpsdot >= ((int16_t)_GB(inj_ae_tps_bins[i])*10)) break;

 if (i < 0)  {i = 0; tpsdot = (int16_t)_GB(inj_ae_tps_bins[0])*10;}
 if (tpsdot > ((int16_t)_GB(inj_ae_tps_bins[INJ_AE_TPS_LOOKUP_TABLE_SIZE-1])*10)) tpsdot = ((int16_t)_GB(inj_ae_tps_bins[INJ_AE_TPS_LOOKUP_TABLE_SIZE-1])*10);

 return simple_interpolation(tpsdot,
             ((int16_t)_GBU(inj_ae_tps_enr[i]))-55, ((int16_t)_GBU(inj_ae_tps_enr[i+1]))-55,  //<--values in inj_ae_tps_enr table are unsigned
             ((int16_t)_GB(inj_ae_tps_bins[i]))*10,((int16_t)(_GB(inj_ae_tps_bins[i+1])-_GB(inj_ae_tps_bins[i])))*10, 164) >> 7; //*1.28, so output value will be x 128
}

uint8_t inj_ae_rpm_lookup(void)
{
 int8_t i;
 int16_t rpm = d.sens.inst_frq; //min-1

 for(i = INJ_AE_RPM_LOOKUP_TABLE_SIZE-2; i >= 0; i--)
  if (d.sens.inst_frq >= _GBU(inj_ae_rpm_bins[i])*100) break;

 if (i < 0)  {i = 0; rpm = _GBU(inj_ae_rpm_bins[0])*100;}
 if (rpm > _GBU(inj_ae_rpm_bins[INJ_AE_RPM_LOOKUP_TABLE_SIZE-1])*100) rpm = _GBU(inj_ae_rpm_bins[INJ_AE_RPM_LOOKUP_TABLE_SIZE-1])*100;

 return simple_interpolation(rpm,
             ((int16_t)_GBU(inj_ae_rpm_enr[i])), ((int16_t)_GBU(inj_ae_rpm_enr[i+1])),  //<--values in table are unsigned
             _GBU(inj_ae_rpm_bins[i])*100,(_GBU(inj_ae_rpm_bins[i+1])-_GBU(inj_ae_rpm_bins[i]))*100, 16) >> 4; //<--values of bins are unsigned
}

uint16_t inj_ae_clt_corr(void)
{
 int16_t t = d.sens.temperat; //clt

 if (!CHECKBIT(d.param.tmp_flags, TMPF_CLT_USE))
  return 128;   //coolant temperature sensor is not enabled (or not installed), no correction

 //-30 - temperature when correction factor is as specified by inj_ae_coldacc_mult
 // 70 - temperature when correction factor doesn't take effect
 restrict_value_to(&t, TEMPERATURE_MAGNITUDE(-30), TEMPERATURE_MAGNITUDE(70));

 return simple_interpolation(t,
             ((int16_t)(d.param.inj_ae_coldacc_mult))+128, 128,
             TEMPERATURE_MAGNITUDE(-30.0), TEMPERATURE_MAGNITUDE(100), 32) >> 5; //70 - (-30) = 100
}
#endif

#ifdef FUEL_INJECT
uint16_t inj_prime_pw(void)
{
 int16_t t = d.sens.temperat; //clt

 if (!CHECKBIT(d.param.tmp_flags, TMPF_CLT_USE))
  return d.param.inj_prime_hot;   //coolant temperature sensor is not enabled (or not installed), use hot PW

 //-30 - temperature for "cold" PW
 // 70 - temperature for "hot" PW
 restrict_value_to(&t, TEMPERATURE_MAGNITUDE(-30), TEMPERATURE_MAGNITUDE(70));

 return simple_interpolation(t, d.param.inj_prime_cold, d.param.inj_prime_hot,
             TEMPERATURE_MAGNITUDE(-30.0), TEMPERATURE_MAGNITUDE(100), 1);
}

uint16_t inj_idlreg_rigidity(uint16_t targ_map, uint16_t targ_rpm)
{
 #define RAD_MAG(v) ROUND((v) * 1024)
 //if targ_map == 0, then do not use load component
 uint8_t k_load = targ_map ?  ROUND(2.0*32) : 0, k_rpm = ROUND(2.0*32); //value * 32, max 6.0

 //normalize values (function argument components)
 //as a result dload and drpm values multiplied by 1024
 //NOTE: We rely that difference (upper_pressure - lower_pressure) is not less than 1/5 of maximum value of MAP (otherwise owerflow may occur)
 int16_t dload = (((int32_t)abs(((int16_t)d.load) - targ_map) * (int16_t)128 * k_load) / (get_load_upper() - d.param.load_lower)) >> 2; //TODO: use d.sens.map or d.load ???
 int16_t drpm = (((int32_t)abs(((int16_t)d.sens.inst_frq) - targ_rpm) * (int16_t)128 * k_rpm) / (PGM_GET_WORD(&fw_data.exdata.rpm_grid_points[RPM_GRID_SIZE-1]) - PGM_GET_WORD(&fw_data.exdata.rpm_grid_points[0]))) >> 2;

 //calculate argument R = SQRT(dload^2 + drpm^2)
 int16_t i, i1, R = ui32_sqrt(((int32_t)dload * dload) + ((int32_t)drpm * drpm));

 if (R > RAD_MAG(1.0))
  R = RAD_MAG(1.0);

 uint16_t arg = R >> 2;
 if (arg > 254)  //note: value 255 is reserved
  arg = 254;
 d.corr.rigid_arg = arg;

 i = R / RAD_MAG(0.1428);   //0.1428 - horizontal axis step

 if (i >= INJ_IDL_RIGIDITY_SIZE-1) i = i1 = INJ_IDL_RIGIDITY_SIZE-1;
  else i1 = i + 1;

 return simple_interpolation(R, _GWU(inj_idl_rigidity[i]), _GWU(inj_idl_rigidity[i1]),  //<--values in table are unsigned
        (i * RAD_MAG(0.1428)), RAD_MAG(0.1428), 8) >> 3;
}

uint16_t inj_iacmixtcorr_lookup(void)
{
 int16_t i, i1, x = d.choke_pos << 6; //value * 128

 //IAC pos. value at the start of axis
 uint16_t x_start = _GW(inj_iac_corr[INJ_IAC_CORR_SIZE]);
 //IAC pos. value at the end of axis
 uint16_t x_end = _GW(inj_iac_corr[INJ_IAC_CORR_SIZE+1]);

 uint16_t x_step = (x_end - x_start) / (INJ_IAC_CORR_SIZE - 1);

 if (x < x_start)
  x = x_start;

 int16_t corr = 0;
 if (x_step > 0)
 {
  i = (x - x_start) / x_step;

  if (i >= INJ_IAC_CORR_SIZE-1) i = i1 = INJ_IAC_CORR_SIZE-1;
  else i1 = i + 1;

  corr = (simple_interpolation(x, _GW(inj_iac_corr[i]), _GW(inj_iac_corr[i1]), //<--values in table are unsigned
         (i * x_step) + x_start, x_step, 2)) >> 1;
 }

 //Calculate weight coefficient:

 x = d.sens.tps << 6; //value * 128

 //IAC pos. value at the start of axis
 x_start = (_GBU(inj_iac_corr_w[INJ_IAC_CORR_W_SIZE])) << 6;
 //IAC pos. value at the end of axis
 x_end = (_GBU(inj_iac_corr_w[INJ_IAC_CORR_W_SIZE+1])) << 6;

 x_step = (x_end - x_start) / (INJ_IAC_CORR_W_SIZE - 1);

 if (x < x_start)
  x = x_start;

 uint16_t corr_w = 0;
 if (x_step > 0)
 {
  i = (x - x_start) / x_step;

  if (i >= INJ_IAC_CORR_W_SIZE-1) i = i1 = INJ_IAC_CORR_W_SIZE-1;
  else i1 = i + 1;

  corr_w = (simple_interpolation(x, _GBU(inj_iac_corr_w[i]), _GBU(inj_iac_corr_w[i1]), //<--values in table are unsigned
          (i * x_step) + x_start, x_step, 32));
 }

 //calculate final value
 return 8192 + (int16_t)(((int32_t)corr * corr_w) >> (8+5));
}

#endif //FUEL_INJECT

#if defined(FUEL_INJECT) || defined(SM_CONTROL)
uint16_t inj_idling_rpm(void)
{
 if (!CHECKBIT(d.param.tmp_flags, TMPF_CLT_USE))
  return 900;   //coolant temperature sensor is not enabled (or not installed)

 return (simple_interpolation(fcs.ta_clt, _GBU(inj_target_rpm[fcs.ta_i]), _GBU(inj_target_rpm[fcs.ta_i1]),  //<--values in table are unsigned
         PGM_GET_WORD(&fw_data.exdata.clt_grid_points[fcs.ta_i]), PGM_GET_WORD(&fw_data.exdata.clt_grid_sizes[fcs.ta_i]), 16) >> 4) * 10;
}
#endif

uint16_t tpsswt_function(void)
{
 return simple_interpolation(fcs.la_rpm, _GB(inj_tpsswt[fcs.la_f]), _GB(inj_tpsswt[fcs.la_fp1]),
        PGM_GET_WORD(&fw_data.exdata.rpm_grid_points[fcs.la_f]), PGM_GET_WORD(&fw_data.exdata.rpm_grid_sizes[fcs.la_f]), 16) >> 4;
}

#ifdef PA4_INP_IGNTIM
int16_t pa4_function(uint16_t adcvalue)
{
 int16_t i, i1;

 //Voltage value at the start of axis in ADC discretes
 uint16_t v_start = ROUND(0.00 / ADC_DISCRETE);
 //Voltage value at the end of axis in ADC discretes
 uint16_t v_end = ROUND(5.00 / ADC_DISCRETE);

 uint16_t v_step = (v_end - v_start) / (PA4_LOOKUP_TABLE_SIZE - 1);

 if (adcvalue < v_start)
  adcvalue = v_start;

 i = (adcvalue - v_start) / v_step;

 if (i >= PA4_LOOKUP_TABLE_SIZE-1) i = i1 = PA4_LOOKUP_TABLE_SIZE-1;
 else i1 = i + 1;

 return  simple_interpolation(adcvalue, (int8_t)PGM_GET_BYTE(&fw_data.exdata.pa4_igntim_corr[i]), (int8_t)PGM_GET_BYTE(&fw_data.exdata.pa4_igntim_corr[i1]), //<--values in table are signed
        (i * v_step) + v_start, v_step, 16);
}

#endif //PA4_INP_IGNTIM

#ifdef GD_CONTROL
/**Use VE and AFR tables for gas dosator. Result = VE * AFR * K, where K is stoichiometry constant
 * \return value * 2048
 */
uint16_t gd_ve_afr(void)
{
 int32_t corr = fcs.vecurr; //apply VE table

 corr=(corr * ((uint16_t)d.param.gd_lambda_stoichval)) >> 7; // multiply by stoichiometry AFR value specified by user

 corr=(corr * nr_1x_afr(fcs.afrcurr << 2)) >> 15;  //apply AFR value

 return corr; //return correction value * 2048
}

/** Calculation of gas dosator position, based on (TPS,RPM)
 * Uses d ECU data structure
 * \return Gas dosator position in % (value * 2)
 */
int16_t gdp_function(void)
{
 int16_t tps = (TPS_MAGNITUDE(100.0) - d.sens.tps) * 16;
 int8_t t = (tps / TPS_AXIS_STEP), tp1;

 if (t >= (GASDOSE_POS_TPS_SIZE - 1))
  tp1 = t = GASDOSE_POS_TPS_SIZE - 1;
 else
  tp1 = t + 1;

 return bilinear_interpolation(fcs.la_rpm, tps,  //note that tps is additionally multiplied by 16
        PGM_GET_BYTE(&fw_data.exdata.gasdose_pos[t][fcs.la_f]),
        PGM_GET_BYTE(&fw_data.exdata.gasdose_pos[tp1][fcs.la_f]),
        PGM_GET_BYTE(&fw_data.exdata.gasdose_pos[tp1][fcs.la_fp1]),
        PGM_GET_BYTE(&fw_data.exdata.gasdose_pos[t][fcs.la_fp1]),
        PGM_GET_WORD(&fw_data.exdata.rpm_grid_points[fcs.la_f]),
        (TPS_AXIS_STEP*t),
        PGM_GET_WORD(&fw_data.exdata.rpm_grid_sizes[fcs.la_f]),
        TPS_AXIS_STEP, 16) >> 4;
}

#endif //GD_CONTROL


#if defined(FUEL_INJECT) || defined(CARB_AFR) || defined(GD_CONTROL)
int16_t ego_curve_lookup(void)
{
 int16_t i, i1, voltage = d.sens.lambda1; /*d.sens.inst_add_i1*/

 //Voltage value at the start of axis in ADC discretes
 uint16_t v_start = _GWU(inj_ego_curve[INJ_EGO_CURVE_SIZE]);
 //Voltage value at the end of axis in ADC discretes
 uint16_t v_end = _GWU(inj_ego_curve[INJ_EGO_CURVE_SIZE+1]);

 uint16_t v_step = (v_end - v_start) / (INJ_EGO_CURVE_SIZE - 1);

 if (voltage < v_start)
  voltage = v_start;

 i = (voltage - v_start) / v_step;

 if (i >= INJ_EGO_CURVE_SIZE-1) i = i1 = INJ_EGO_CURVE_SIZE-1;
 else i1 = i + 1;

 return (simple_interpolation(voltage, _GWU(inj_ego_curve[i]), _GWU(inj_ego_curve[i1]), //<--values in table are unsigned
        (i * v_step) + v_start, v_step, 4)) >> 2;
}
#endif

#if defined(FUEL_INJECT) /*|| defined(CARB_AFR)*/ || defined(GD_CONTROL)
int16_t ego_curve_min(void)
{
 int16_t a = _GWU(inj_ego_curve[0]);
 int16_t b = _GWU(inj_ego_curve[INJ_EGO_CURVE_SIZE-1]);
 return  a < b ? a : b;
}

int16_t ego_curve_max(void)
{
 int16_t a = _GWU(inj_ego_curve[0]);
 int16_t b = _GWU(inj_ego_curve[INJ_EGO_CURVE_SIZE-1]);
 return a > b ? a : b;
}

uint8_t scale_aftstr_enrich(uint16_t enrich_counter)
{
 int16_t aftstr_strk = aftstr_strokes(d.sens.gas);
 //do scaling of ASE factor (scale down)
 int16_t counter = aftstr_strk - enrich_counter; //convert decreasing to increasing
 if (counter < 0) counter = 0;
 return ((uint32_t)inj_aftstr_en() * (aftstr_strk - counter)) / aftstr_strk;
}

int16_t barocorr_lookup(void)
{
 int16_t i, i1, press = d.sens.baro_press;

 //Pressure value at the start of axis
 uint16_t p_start = PGM_GET_WORD(&fw_data.exdata.barocorr[BAROCORR_SIZE]);
 //Pressure value at the end of axis
 uint16_t p_end = PGM_GET_WORD(&fw_data.exdata.barocorr[BAROCORR_SIZE+1]);

 uint16_t p_step = (p_end - p_start) / (BAROCORR_SIZE - 1);

 if (press < p_start)
  press = p_start;

 i = (press - p_start) / p_step;

 if (i >= BAROCORR_SIZE-1) i = i1 = BAROCORR_SIZE-1;
 else i1 = i + 1;

 return (simple_interpolation(press, (int16_t)PGM_GET_WORD(&fw_data.exdata.barocorr[i]), (int16_t)PGM_GET_WORD(&fw_data.exdata.barocorr[i1]), //<--values in table are signed
        (i * p_step) + p_start, p_step, 4)) >> 2;
}


#endif

#if defined(FUEL_INJECT) || defined(SM_CONTROL) || defined(GD_CONTROL)

uint8_t inj_airtemp_corr(uint8_t rawmat)
{
 int16_t i, i1, t = rawmat ? d.sens.air_temp :
#if defined(FUEL_INJECT) || defined(GD_CONTROL)
 inj_corrected_mat(); //use corrected MAT or raw value
#else //SM_CONTROL only (choke)
 d.sens.air_temp; //use raw MAT value (rawmat ignored)
#endif

 if (!IOCFG_CHECK(IOP_AIR_TEMP))
  return 128;   //do not use correcton if air temperature sensor is turned off

 //-30 - minimum temperature value
 if (t < TEMPERATURE_MAGNITUDE(-30))
  t = TEMPERATURE_MAGNITUDE(-30);

 //10 - step between interpolation points
 i = (t - TEMPERATURE_MAGNITUDE(-30)) / TEMPERATURE_MAGNITUDE(10);

 if (i >= INJ_ATS_CORR_SIZE-1) i = i1 = INJ_ATS_CORR_SIZE-1;
 else i1 = i + 1;

 return simple_interpolation(t, _GBU(inj_ats_corr[i]), _GBU(inj_ats_corr[i1]), //<--values in table are signed
 (i * TEMPERATURE_MAGNITUDE(10)) + TEMPERATURE_MAGNITUDE(-30), TEMPERATURE_MAGNITUDE(10), 16) >> 4;
}

#endif

#if (defined(FUEL_INJECT) || defined(GD_CONTROL)) && !defined(SECU3T)
uint8_t inj_gts_pwcorr(void)
{
 int16_t i, i1, t = d.sens.tmp2;

 if (!IOCFG_CHECK(IOP_TMP2))
  return 128;   //do not use correcton if gas temperature sensor is turned off

 //-30 - minimum temperature value
 if (t < TEMPERATURE_MAGNITUDE(-30))
  t = TEMPERATURE_MAGNITUDE(-30);

 //10 - step between interpolation points
 i = (t - TEMPERATURE_MAGNITUDE(-30)) / TEMPERATURE_MAGNITUDE(10);

 if (i >= INJ_GTS_CORR_SIZE-1) i = i1 = INJ_GTS_CORR_SIZE-1;
 else i1 = i + 1;

 return simple_interpolation(t, _GBU(inj_gts_corr[i]), _GBU(inj_gts_corr[i1]), //<--values in table are unsigned
 (i * TEMPERATURE_MAGNITUDE(10)) + TEMPERATURE_MAGNITUDE(-30), TEMPERATURE_MAGNITUDE(10), 16) >> 4;
}

uint8_t inj_gps_pwcorr(void)
{
 int16_t i, i1, p = CHECKBIT(d.param.inj_flags, INJFLG_USEDIFFPRESS) ? (d.sens.map2 - d.sens.map) : d.sens.map2;

 if (!IOCFG_CHECK(IOP_MAP2))
  return 128;   //do not use correcton if gas pressure sensor is turned off

 //Pressure value at the start of axis
 uint16_t p_start = ((uint16_t)_GBU(inj_gps_corr[INJ_GPS_CORR_SIZE])) * (2 * 64);
 //Pressure value at the end of axis
 uint16_t p_end = ((uint16_t)_GBU(inj_gps_corr[INJ_GPS_CORR_SIZE+1])) * (2 * 64);

 uint16_t p_step = (p_end - p_start) / (INJ_GPS_CORR_SIZE - 1);

 if (p < p_start)
  p = p_start;

 uint8_t coeff = 128; //1.0
 if (p_step > 0)
 {
  i = (p - p_start) / p_step;

  if (i >= INJ_GPS_CORR_SIZE-1) i = i1 = INJ_GPS_CORR_SIZE-1;
  else i1 = i + 1;

  coeff = (simple_interpolation(p, _GBU(inj_gps_corr[i]), _GBU(inj_gps_corr[i1]), //<--values in table are unsigned
          (i * p_step) + p_start, p_step, 64)) >> 6;
 }
 return coeff;
}
#endif

#if defined(FUEL_INJECT) || defined(SM_CONTROL) || defined(GD_CONTROL)
/** Checks for conditions activating engine blowing mode. Writes result value into d.floodclear flag
 * \return 1 - engine blowing should be active, 0 - not active
 */
uint8_t engine_blowing_cond(void)
{
 d.floodclear = ((d.sens.tps > d.param.inj_floodclear_tps) && (0 != d.param.inj_floodclear_tps)) && (d.engine_mode == EM_START || !PGM_GET_BYTE(&fw_data.exdata.fldclr_start));
 return d.floodclear;
}
#endif

#if defined(FUEL_INJECT) || defined(GD_CONTROL)
/**"Normal conditions" constant for calculating of NC pulse width, value of this constant = 1397*/
#define PWNC_CONST ROUND((100.0*MAP_PHYSICAL_MAGNITUDE_MULTIPLIER*256) / (293.15*TEMP_PHYSICAL_MAGNITUDE_MULTIPLIER))

int32_t acc_enrich_calc(uint8_t mode, int16_t stoich_val)
{
 //calculate normal conditions PW, MAP=100kPa, IAT=20.C, AFR=14.7 (petrol) or d.param.gd_lambda_stoichval (gas).
 //For AFR=14.7 and inj_sd_igl_const=86207 we should get result near to 2000.48
 int32_t pwnc = mode ? GD_MAGNITUDE(100.0) : ((((((uint32_t)PWNC_CONST) * nr_1x_afr(stoich_val << 3)) >> 12) * d.param.inj_sd_igl_const[d.sens.gas]) >> 15);
 int16_t aef = inj_ae_tps_lookup(d.sens.tpsdot);               //calculate basic AE factor value

 if (abs(d.sens.tpsdot) < d.param.inj_ae_tpsdot_thrd)
 {
  //stop decay if gas pedal fully released
  if (!d.sens.carb)
   fcs.ae_decay_counter = 0;
  d.acceleration = (fcs.ae_decay_counter > 0);
  //apply decay factor
  aef = (((int32_t)fcs.aef_decay) * fcs.ae_decay_counter) / d.param.inj_ae_decay_time; //TODO: replace division by multiplication with 1 / inj_ae_decay_time constant
 }
 else
 {
  fcs.aef_decay = inj_ae_tps_lookup((d.sens.tpsdot < 0) ? -((int16_t)d.param.inj_ae_tpsdot_thrd) : d.param.inj_ae_tpsdot_thrd); //aef
  fcs.ae_decay_counter = d.param.inj_ae_decay_time; //init counter
  d.acceleration = 1;
 }

 if (aef >= 0)
  aef = ((int32_t)aef * inj_ae_clt_corr()) >> 7;   //apply CLT correction factor to AE factor
 else
  aef = (((int32_t)aef) * (((uint16_t)65535) / inj_ae_clt_corr())) >> (16-7); //use inverse CLT correction factor if AE factor is negative, so mixture will be less lean when engine is cold

 aef = ((int32_t)aef * inj_ae_rpm_lookup()) >> 7; //apply RPM correction factor to AE factor
 return (pwnc * aef) >> 7;                        //apply AE factor to the normal conditions PW
}

void acc_enrich_decay_counter(void)
{
 if (fcs.ae_decay_counter)
  --fcs.ae_decay_counter; //update AE decay counter
}
#endif

uint16_t cranking_thrd_rpm(void)
{
 if (!CHECKBIT(d.param.tmp_flags, TMPF_CLT_USE))
  return d.param.starter_off;   //coolant temperature sensor is not enabled (or not installed), use simple constant

 return (simple_interpolation(fcs.ta_clt, PGM_GET_BYTE(&fw_data.exdata.cranking_thrd[fcs.ta_i]), PGM_GET_BYTE(&fw_data.exdata.cranking_thrd[fcs.ta_i1]),  //<--values in table are unsigned
         PGM_GET_WORD(&fw_data.exdata.clt_grid_points[fcs.ta_i]), PGM_GET_WORD(&fw_data.exdata.clt_grid_sizes[fcs.ta_i]), 16) >> 4) * 10;
}

uint16_t cranking_thrd_tmr(void)
{
 if (!CHECKBIT(d.param.tmp_flags, TMPF_CLT_USE))
  return PGM_GET_BYTE(&fw_data.exdata.stbl_str_cnt);   //coolant temperature sensor is not enabled (or not installed), use simple constant

 return (simple_interpolation(fcs.ta_clt, PGM_GET_BYTE(&fw_data.exdata.cranking_time[fcs.ta_i]), PGM_GET_BYTE(&fw_data.exdata.cranking_time[fcs.ta_i1]),  //<--values in table are unsigned
         PGM_GET_WORD(&fw_data.exdata.clt_grid_points[fcs.ta_i]), PGM_GET_WORD(&fw_data.exdata.clt_grid_sizes[fcs.ta_i]), 16) >> 4) * 10;
}

uint16_t smapaban_thrd_rpm(void)
{
 if (!CHECKBIT(d.param.tmp_flags, TMPF_CLT_USE))
  return d.param.smap_abandon;   //coolant temperature sensor is not enabled (or not installed). use simple constant

 return (simple_interpolation(fcs.ta_clt, PGM_GET_BYTE(&fw_data.exdata.smapaban_thrd[fcs.ta_i]), PGM_GET_BYTE(&fw_data.exdata.smapaban_thrd[fcs.ta_i1]),  //<--values in table are unsigned
         PGM_GET_WORD(&fw_data.exdata.clt_grid_points[fcs.ta_i]), PGM_GET_WORD(&fw_data.exdata.clt_grid_sizes[fcs.ta_i]), 16) >> 4) * 10;
}

#ifdef _PLATFORM_M1284_
/**Gets knock zone flag from a look up table using current RPM and TPS values
 * \return flag value (0, 1)
 */
uint8_t knock_zone_val(void)
{
 uint16_t rpm = d.sens.inst_frq;
 int16_t tps = d.sens.tps * 16;
 int8_t f, t;

 //find interpolation points, then restrict RPM if it fall outside set range
 for(f = F_WRK_POINTS_F-1; f >= 0; f--)
  if (rpm >= PGM_GET_WORD(&fw_data.exdata.rpm_grid_points[f])) break;
 //lookup table works from rpm_grid_points[0] and upper
 if (f < 0) f = 0;

 t = (tps / TPS_AXIS_STEP);
 if (t > (KNKZONE_TPS_SIZE - 1))
  t = KNKZONE_TPS_SIZE - 1;

 t = (KNKZONE_TPS_SIZE-1)-t;
 return ((uint8_t)(PGM_GET_WORD(&fw_data.exdata.knock_zones[t]) >> f)) & 1;
}

#endif

uint16_t pwm_function(uint8_t mode)
{
 uint8_t use_grid = CHECKBIT(d.param.func_flags, FUNC_LDAX_GRID);
 if (0==mode)
 return bilinear_interpolation(fcs.la_rpm, fcs.la_load,
        _GBU(pwm_duty1[fcs.la_l][fcs.la_f]),   //<-- values are unsigned
        _GBU(pwm_duty1[fcs.la_lp1][fcs.la_f]),
        _GBU(pwm_duty1[fcs.la_lp1][fcs.la_fp1]),
        _GBU(pwm_duty1[fcs.la_l][fcs.la_fp1]),
        PGM_GET_WORD(&fw_data.exdata.rpm_grid_points[fcs.la_f]),
        use_grid ? PGM_GET_WORD(&fw_data.exdata.load_grid_points[fcs.la_l]) : (fcs.la_grad * fcs.la_l),
        PGM_GET_WORD(&fw_data.exdata.rpm_grid_sizes[fcs.la_f]),
        use_grid ? PGM_GET_WORD(&fw_data.exdata.load_grid_sizes[fcs.la_l]) : fcs.la_grad, 64) >> 6;
 else
 return bilinear_interpolation(fcs.la_rpm, fcs.la_load,
        _GBU(pwm_duty2[fcs.la_l][fcs.la_f]),   //<-- values are unsigned
        _GBU(pwm_duty2[fcs.la_lp1][fcs.la_f]),
        _GBU(pwm_duty2[fcs.la_lp1][fcs.la_fp1]),
        _GBU(pwm_duty2[fcs.la_l][fcs.la_fp1]),
        PGM_GET_WORD(&fw_data.exdata.rpm_grid_points[fcs.la_f]),
        use_grid ? PGM_GET_WORD(&fw_data.exdata.load_grid_points[fcs.la_l]) : (fcs.la_grad * fcs.la_l),
        PGM_GET_WORD(&fw_data.exdata.rpm_grid_sizes[fcs.la_f]),
        use_grid ? PGM_GET_WORD(&fw_data.exdata.load_grid_sizes[fcs.la_l]) : fcs.la_grad, 64) >> 6;
}

#ifdef SPLIT_ANGLE
// Used for rotary engines. Note: we use pwm_duty1 map (it is shared between spliting and PWM)
// Returns anvance angle value * 32, 2 * 16 = 32.
int16_t split_function(void)
{
 uint8_t use_grid = CHECKBIT(d.param.func_flags, FUNC_LDAX_GRID);
 return bilinear_interpolation(fcs.la_rpm, fcs.la_load,
        _GB(pwm_duty1[fcs.la_l][fcs.la_f]),   //<-- values are signed
        _GB(pwm_duty1[fcs.la_lp1][fcs.la_f]),
        _GB(pwm_duty1[fcs.la_lp1][fcs.la_fp1]),
        _GB(pwm_duty1[fcs.la_l][fcs.la_fp1]),
        PGM_GET_WORD(&fw_data.exdata.rpm_grid_points[fcs.la_f]),
        use_grid ? PGM_GET_WORD(&fw_data.exdata.load_grid_points[fcs.la_l]) : (fcs.la_grad * fcs.la_l),
        PGM_GET_WORD(&fw_data.exdata.rpm_grid_sizes[fcs.la_f]),
        use_grid ? PGM_GET_WORD(&fw_data.exdata.load_grid_sizes[fcs.la_l]) : fcs.la_grad, 16);
}
#endif

#ifndef SECU3T
uint8_t grheat_pwm_duty(void)
{
 return simple_interpolation(fcs.ga_grt, PGM_GET_BYTE(&fw_data.exdata.grheat_duty[fcs.ga_i]), PGM_GET_BYTE(&fw_data.exdata.grheat_duty[fcs.ga_i1]),
        PGM_GET_WORD(&fw_data.exdata.clt_grid_points[fcs.ga_i]), PGM_GET_WORD(&fw_data.exdata.clt_grid_sizes[fcs.ga_i]), 16) >> 4;
}

uint16_t grv_delay(void)
{
 return simple_interpolation(fcs.ga_grt, PGM_GET_WORD(&fw_data.exdata.grv_delay[fcs.ga_i]), PGM_GET_WORD(&fw_data.exdata.grv_delay[fcs.ga_i1]),
        PGM_GET_WORD(&fw_data.exdata.clt_grid_points[fcs.ga_i]), PGM_GET_WORD(&fw_data.exdata.clt_grid_sizes[fcs.ga_i]), 2) >> 1;
}
#endif

#if defined(FUEL_INJECT) || defined(GD_CONTROL)
uint16_t pwmiac_ucoef(void)
{
 int16_t i, i1, voltage = d.sens.voltage;

 if (voltage < VOLTAGE_MAGNITUDE(5.4))
  voltage = VOLTAGE_MAGNITUDE(5.4); //5.4 -  minimum voltage value corresponding to 1st value in table for 12V board voltage

 i = (voltage - VOLTAGE_MAGNITUDE(5.4)) / VOLTAGE_MAGNITUDE(0.8);   //0.8 - voltage step

 if (i >= PWMIAC_UCOEF_SIZE-1) i = i1 = PWMIAC_UCOEF_SIZE-1;
  else i1 = i + 1;

 return simple_interpolation(voltage, PGM_GET_WORD(&fw_data.exdata.pwmiac_ucoef[i]), PGM_GET_WORD(&fw_data.exdata.pwmiac_ucoef[i1]),
        (i * VOLTAGE_MAGNITUDE(0.8)) + VOLTAGE_MAGNITUDE(5.4), VOLTAGE_MAGNITUDE(0.8), 2) >> 1;
}
#endif

#if defined(FUEL_INJECT) || defined(GD_CONTROL)
uint16_t aftstr_strokes(uint8_t mode)
{
 if (!CHECKBIT(d.param.tmp_flags, TMPF_CLT_USE))
  return ((uint16_t)(mode ? d.param.inj_aftstr_strokes1 : d.param.inj_aftstr_strokes)) << 2;   //coolant temperature sensor is not enabled (or not installed). use simple constant

 if (mode)
 { //gas
  if (0==d.param.inj_aftstr_strokes1)
   return simple_interpolation(fcs.ta_clt, PGM_GET_BYTE(&fw_data.exdata.inj_aftstr_strk1[fcs.ta_i]), PGM_GET_BYTE(&fw_data.exdata.inj_aftstr_strk1[fcs.ta_i1]),  //<--values in table are unsigned
         PGM_GET_WORD(&fw_data.exdata.clt_grid_points[fcs.ta_i]), PGM_GET_WORD(&fw_data.exdata.clt_grid_sizes[fcs.ta_i]), 16) >> 4;
  else
   return ((uint16_t)d.param.inj_aftstr_strokes1) << 2;
 }
 else
 { //petrol
  if (0==d.param.inj_aftstr_strokes)
   return simple_interpolation(fcs.ta_clt, PGM_GET_BYTE(&fw_data.exdata.inj_aftstr_strk0[fcs.ta_i]), PGM_GET_BYTE(&fw_data.exdata.inj_aftstr_strk0[fcs.ta_i1]),  //<--values in table are unsigned
         PGM_GET_WORD(&fw_data.exdata.clt_grid_points[fcs.ta_i]), PGM_GET_WORD(&fw_data.exdata.clt_grid_sizes[fcs.ta_i]), 16) >> 4;
  else
   return ((uint16_t)d.param.inj_aftstr_strokes) << 2;
 }
}
#endif

#if defined(FUEL_INJECT)
int8_t inj_iac_mat_corr(void)
{
 int16_t i, i1, t = inj_corrected_mat(); //use corrected MAT
 if (!IOCFG_CHECK(IOP_AIR_TEMP))
  return 0;   //do not use correcton if air temperature sensor is turned off

 //-30 - minimum temperature value
 if (t < TEMPERATURE_MAGNITUDE(-30))
  t = TEMPERATURE_MAGNITUDE(-30);

 //10 - step between interpolation points
 i = (t - TEMPERATURE_MAGNITUDE(-30)) / TEMPERATURE_MAGNITUDE(10);

 if (i >= INJ_ATS_CORR_SIZE-1) i = i1 = INJ_ATS_CORR_SIZE-1;
 else i1 = i + 1;

 return simple_interpolation(t, _GB(iac_mat_corr[i]), _GB(iac_mat_corr[i1]), //<--values in table are signed
 (i * TEMPERATURE_MAGNITUDE(10)) + TEMPERATURE_MAGNITUDE(-30), TEMPERATURE_MAGNITUDE(10), 128) >> 7;
}
#endif

#if !defined(SECU3T) && defined(MCP3204)
int16_t exsens_lookup(uint16_t adcvalue, int16_t _PGM *lutab)
{
 int16_t i, i1;
#if ((FTLS_LOOKUP_TABLE_SIZE!=EGTS_LOOKUP_TABLE_SIZE) || (FTLS_LOOKUP_TABLE_SIZE!=OPS_LOOKUP_TABLE_SIZE))
 #error "Sizes of maps differ!"
#endif
 //Voltage value at the start of axis in ADC discretes
 uint16_t v_start = PGM_GET_WORD(&lutab[FTLS_LOOKUP_TABLE_SIZE]);
 //Voltage value at the end of axis in ADC discretes
 uint16_t v_end = PGM_GET_WORD(&lutab[FTLS_LOOKUP_TABLE_SIZE+1]);

 uint16_t v_step = (v_end - v_start) / (FTLS_LOOKUP_TABLE_SIZE - 1);

 if (adcvalue < v_start)
  adcvalue = v_start;

 i = (adcvalue - v_start) / v_step;

 if (i >= FTLS_LOOKUP_TABLE_SIZE-1) i = i1 = FTLS_LOOKUP_TABLE_SIZE-1;
 else i1 = i + 1;

 return (simple_interpolation(adcvalue, (int16_t)PGM_GET_WORD(&lutab[i]), (int16_t)PGM_GET_WORD(&lutab[i1]), //<--values in table are signed
        (i * v_step) + v_start, v_step, 4)) >> 2;
}
#endif

#ifndef SECU3T
int16_t injpwcoef_function(uint16_t adcvalue)
{
 int16_t i, i1;

 //Voltage value at the start of axis in ADC discretes
 uint16_t v_start = ROUND(0.00 / ADC_DISCRETE);
 //Voltage value at the end of axis in ADC discretes
 uint16_t v_end = ROUND(5.00 / ADC_DISCRETE);

 uint16_t v_step = (v_end - v_start) / (INJPWCOEF_LUT_SIZE - 1);

 if (adcvalue < v_start)
  adcvalue = v_start;

 i = (adcvalue - v_start) / v_step;

 if (i >= INJPWCOEF_LUT_SIZE-1) i = i1 = INJPWCOEF_LUT_SIZE-1;
 else i1 = i + 1;

 return  simple_interpolation(adcvalue, PGM_GET_WORD(&fw_data.exdata.injpw_coef[i]), PGM_GET_WORD(&fw_data.exdata.injpw_coef[i1]),
        (i * v_step) + v_start, v_step, 4) >> 2;
}
#endif //SECU-3i
