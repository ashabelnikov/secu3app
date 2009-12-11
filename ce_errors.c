 /****************************************************************
 *       SECU-3  - An open source, free engine control unit
 *    Designed by Alexey A. Shabelnikov. Ukraine, Gorlovka 2007.
 *       Microprocessors systems - design & programming.
 *    contacts:
 *              http://secu-3.narod.ru
 *              ICQ: 405-791-931
 ****************************************************************/

#include <iom16.h>
#include "ce_errors.h"
#include "bitmask.h"
#include "ckps.h"
#include "knock.h"
#include "vstimer.h"
#include "suspendop.h"
#include "eeprom.h"
#include "secu3.h"

typedef struct
{
 uint16_t ecuerrors;         //максимум 16 кодов ошибок
 uint16_t merged_errors;     //кеширует ошибки для сбережения ресурса EEPROM
 uint16_t write_errors;      //ф. eeprom_start_wr_data() запускает фоновый процесс! 
}ce_state_t;

ce_state_t ce_state = {0,0,0};

//операции над ошибками
#pragma inline
void ce_set_error(uint8_t error)  
{
 SETBIT(ce_state.ecuerrors, error);
}

#pragma inline
void ce_clear_error(uint8_t error)
{
 CLEARBIT(ce_state.ecuerrors, error);
}
 
//При возникновении любой ошибки, СЕ загорается на фиксированное время. Если ошибка не исчезает (например испорчен код программы),
//то CE будет гореть непрерывно. При запуске программы СЕ загорается на 0.5 сек. для индицирования работоспособности. 
void ce_check_engine(ecudata* d, s_timer8* ce_control_time_counter)
{
 uint16_t temp_errors;
  
 //если была ошибка ДПКВ то устанавливаем бит соответствующей ошибки
 if (ckps_is_error())
 {
  ce_set_error(ECUERROR_CKPS_MALFUNCTION);
  ckps_reset_error();        
 }
 else
 {
  ce_clear_error(ECUERROR_CKPS_MALFUNCTION);  
 }

 //если была ошибка канала детонации
 if (d->param.knock_use_knock_channel)
  if (knock_is_error())
  {
   ce_set_error(ECUERROR_KSP_CHIP_FAILED);
   knock_reset_error();        
  }
  else
  {
   ce_clear_error(ECUERROR_KSP_CHIP_FAILED);  
  }

 //если таймер отсчитал время, то гасим СЕ
 if (s_timer_is_action(*ce_control_time_counter))
  SET_CE_STATE(0);       

 //если есть хотя бы одна ошибка - зажигаем СЕ и запускаем таймер 
 if (ce_state.ecuerrors!=0)
 {
  s_timer_set(*ce_control_time_counter, CE_CONTROL_STATE_TIME_VALUE);
  SET_CE_STATE(1);  
 }

 temp_errors = (ce_state.merged_errors | ce_state.ecuerrors);
 if (temp_errors!=ce_state.merged_errors) //появилась ли ошибка которой нет в merged_errors?
 {
  //так как на момент возникновения новой ошибки EEPROM может быть занято (например сохранением параметров),
  //то необходимо запустить отложенную операцию, которая будет автоматически выполнена как только EEPROM
  //освободится. 
  sop_set_operation(SOP_SAVE_CE_MERGED_ERRORS);
 }

 ce_state.merged_errors = temp_errors;

 //переносим биты ошибок в кеш для передачи.
 d->ecuerrors_for_transfer|= ce_state.ecuerrors;
}

void ce_save_marged_errors(void)
{
 uint16_t temp_errors;
 eeprom_read(&temp_errors, EEPROM_ECUERRORS_START, sizeof(uint16_t));
 ce_state.write_errors = temp_errors | ce_state.merged_errors; 
 if (ce_state.write_errors!=temp_errors)    
  eeprom_start_wr_data(0, EEPROM_ECUERRORS_START, (uint8_t*)&ce_state.write_errors, sizeof(uint16_t));      
}
