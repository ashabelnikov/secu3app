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

/** \file uart.c
 * Implementation of service for performing communication via UART.
 * (Реализация поддержки обмена данными через UART).
 */

#include "port/avrio.h"
#include "port/interrupt.h"
#include "port/intrinsic.h"
#include "port/pgmspace.h"
#include "port/port.h"
#include <string.h>
#include "bitmask.h"
#include "eeprom.h"
#include "secu3.h"
#include "uart.h"
#include "ufcodes.h"

//Mega64 compatibility
#ifdef _PLATFORM_M64_
#ifndef RXEN
 #define RXEN  RXEN0
#endif
#ifndef TXEN
 #define TXEN  TXEN0
#endif
#ifndef UDRE
 #define UDRE  UDRE0
#endif
#ifndef RXC
 #define RXC   RXC0
#endif
#ifndef RXCIE
 #define RXCIE RXCIE0
#endif
#ifndef UDRIE
 #define UDRIE UDRIE0
#endif
#ifndef UCSZ0
 #define UCSZ0 UCSZ00
#endif
#ifndef UCSZ1
 #define UCSZ1 UCSZ01
#endif
#ifndef U2X
 #define U2X U2X0
#endif
#define UDR   UDR0
#define UBRRL UBRR0L
#define UBRRH UBRR0H
#define UCSRA UCSR0A
#define UCSRB UCSR0B
#define UCSRC UCSR0C
#define USART_UDRE_vect USART0_UDRE_vect
#define USART_RXC_vect USART0_RXC_vect
#endif

//Idenfifiers used in EDITAB_PAR
#define ETTS_GASOLINE_SET 0 //!< tables's set: gasoline id
#define ETTS_GAS_SET      1 //!< tables's set: gas id

#define ETMT_STRT_MAP 0     //!< start map id
#define ETMT_IDLE_MAP 1     //!< idle map id
#define ETMT_WORK_MAP 2     //!< work map id
#define ETMT_TEMP_MAP 3     //!< temp.corr. map id
#define ETMT_NAME_STR 4     //!< name of tables's set id

/**Define internal state variables */
typedef struct
{
 uint8_t send_mode;                     //!< current descriptor of packets beeing send
 uint8_t recv_buf[UART_RECV_BUFF_SIZE]; //!< receiver's buffer
 uint8_t send_buf[UART_SEND_BUFF_SIZE]; //!< transmitter's buffer
 volatile uint8_t send_size;            //!< size of data to be send
 uint8_t send_index;                    //!< index in transmitter's buffer
 volatile uint8_t recv_size;            //!< size of received data
 uint8_t recv_index;                    //!< index in receiver's buffer
}uartstate_t;

/**State variables */
uartstate_t uart;

/**For BIN-->HEX encoding */
PGM_DECLARE(uint8_t hdig[]) = "0123456789ABCDEF";

/**Decodes from HEX to BIN */
#define HTOD(h) (((h)<0x3A) ? ((h)-'0') : ((h)-'A'+10))

//--------вспомогательные функции для построения пакетов-------------

/**Appends sender's buffer by sequence of bytes from programm memory 
 * note! can NOT be used for binary data! */
#define build_fs(src, size) \
{ \
 memcpy_P(&uart.send_buf[uart.send_size],(src),(size)); \
 uart.send_size+=(size); \
}

/**Appends sender's buffer by sequence of bytes from RAM 
 * note! can NOT be used for binary data! */
#define build_rs(src, size) \
{ \
 memcpy(&uart.send_buf[uart.send_size],(src),(size)); \
 uart.send_size+=(size); \
}

/**Appends sender's buffer by one HEX byte */
#define build_i4h(i) {uart.send_buf[uart.send_size++] = ((i)+0x30);}

/**Appends sender's buffer by two HEX bytes
 * \param i 8-bit value to be converted into hex
 */
static void build_i8h(uint8_t i)
{
 uart.send_buf[uart.send_size++] = PGM_GET_BYTE(&hdig[i/16]);    //старший байт HEX числа
 uart.send_buf[uart.send_size++] = PGM_GET_BYTE(&hdig[i%16]);    //младший байт HEX числа
}

/**Appends sender's buffer by 4 HEX bytes
 * \param i 16-bit value to be converted into hex
 */
static void build_i16h(uint16_t i)
{
 uart.send_buf[uart.send_size++] = PGM_GET_BYTE(&hdig[_AB(i,1)/16]);    //старший байт HEX числа (старший байт)
 uart.send_buf[uart.send_size++] = PGM_GET_BYTE(&hdig[_AB(i,1)%16]);    //младший байт HEX числа (старший байт)
 uart.send_buf[uart.send_size++] = PGM_GET_BYTE(&hdig[_AB(i,0)/16]);    //старший байт HEX числа (младший байт)
 uart.send_buf[uart.send_size++] = PGM_GET_BYTE(&hdig[_AB(i,0)%16]);    //младший байт HEX числа (младший байт)
}

/**Appends sender's buffer by 8 HEX bytes
 * \param i 32-bit value to be converted into hex
 */
static void build_i32h(uint32_t i)
{
 build_i16h(i>>16);
 build_i16h(i);
}

/**Appends sender's buffer by sequence of bytes from RAM buffer
 * can be used for binary data */
static void build_rb(const uint8_t* ramBuffer, uint8_t size)
{
 while(size--) build_i8h(*ramBuffer++);
}

//----------вспомагательные функции для распознавания пакетов---------
/**Recepts sequence of bytes from receiver's buffer and places it into the RAM buffer
 * can NOT be used for binary data */
static void recept_rs(uint8_t* ramBuffer, uint8_t size)
{ 
 if (size > uart.recv_size)
  size = uart.recv_size;
 while(size--) *ramBuffer++ = uart.recv_buf[uart.recv_index++];
}

/**Retrieves from receiver's buffer 4-bit value */
#define recept_i4h() (uart.recv_buf[uart.recv_index++] - 0x30)

/**Retrieves from receiver's buffer 8-bit value
 * \return retrieved value
 */
static uint8_t recept_i8h(void)
{
 uint8_t i8;
 i8 = HTOD(uart.recv_buf[uart.recv_index])<<4;
 ++uart.recv_index;
 i8|= HTOD(uart.recv_buf[uart.recv_index]);
 ++uart.recv_index;
 return i8;
}

/**Retrieves from receiver's buffer 16-bit value
 * \return retrieved value
 */
static uint16_t recept_i16h(void)
{
 uint16_t i16;
 _AB(i16,1) = (HTOD(uart.recv_buf[uart.recv_index]))<<4;
 ++uart.recv_index;
 _AB(i16,1)|= (HTOD(uart.recv_buf[uart.recv_index]));
 ++uart.recv_index;
 _AB(i16,0) = (HTOD(uart.recv_buf[uart.recv_index]))<<4;
 ++uart.recv_index;
 _AB(i16,0)|= (HTOD(uart.recv_buf[uart.recv_index]));
 ++uart.recv_index;
 return i16;
}

/**Retrieves from receiver's buffer 32-bit value
 * \return retrieved value
 */
static uint32_t recept_i32h(void)
{
 uint32_t i = 0;
 i = recept_i16h();
 i = i << 16;
 i|=recept_i16h();
 return i;
}

/**Recepts sequence of bytes from receiver's buffer and places it into the RAM buffer
 * can be used for binary data */
static void recept_rb(uint8_t* ramBuffer, uint8_t size)
{
 uint8_t rcvsize = uart.recv_size >> 1; //two hex symbols per byte
 if (size > rcvsize)
  size = rcvsize;
 while(size--) *ramBuffer++ = recept_i8h();
}
//--------------------------------------------------------------------

/**Makes sender to start sending */
static void uart_begin_send(void)
{
 uart.send_index = 0;
 _DISABLE_INTERRUPT();
 UCSRB |= _BV(UDRIE); /* enable UDRE interrupt */
 _ENABLE_INTERRUPT();
}

void uart_send_packet(struct ecudata_t* d, uint8_t send_mode)
{
 static uint8_t index = 0;

 //служит индексом во время сборки пакетов, а после сборки будет содержать размер пакета
 uart.send_size = 0;

 if (send_mode==0) //используем текущий дескриптор
  send_mode = uart.send_mode;

 //общая часть для всех пакетов
 uart.send_buf[uart.send_size++] = '@';
 uart.send_buf[uart.send_size++] = send_mode;

 switch(send_mode)
 {
  case TEMPER_PAR:
   build_i4h(d->param.tmp_use);
   build_i4h(d->param.vent_pwm);
   build_i4h(d->param.cts_use_map);
   build_i16h(d->param.vent_on);
   build_i16h(d->param.vent_off);
   break;

  case CARBUR_PAR:
   build_i16h(d->param.ie_lot);
   build_i16h(d->param.ie_hit);
   build_i4h(d->param.carb_invers);
   build_i16h(d->param.fe_on_threshold);
   build_i16h(d->param.ie_lot_g);
   build_i16h(d->param.ie_hit_g);
   build_i8h(d->param.shutoff_delay);
   break;

  case IDLREG_PAR:
   build_i4h(d->param.idl_regul);
   build_i16h(d->param.ifac1);
   build_i16h(d->param.ifac2);
   build_i16h(d->param.MINEFR);
   build_i16h(d->param.idling_rpm);
   build_i16h(d->param.idlreg_min_angle);
   build_i16h(d->param.idlreg_max_angle);
   break;

  case ANGLES_PAR:
   build_i16h(d->param.max_angle);
   build_i16h(d->param.min_angle);
   build_i16h(d->param.angle_corr);
   build_i16h(d->param.angle_dec_spead);
   build_i16h(d->param.angle_inc_spead);
   build_i4h(d->param.zero_adv_ang);
   break;

  case FUNSET_PAR:
   build_i8h(d->param.fn_gasoline);
   build_i8h(d->param.fn_gas);
   build_i16h(d->param.map_lower_pressure);
   build_i16h(d->param.map_upper_pressure);
   build_i16h(d->param.map_curve_offset);
   build_i16h(d->param.map_curve_gradient);
   break;

  case STARTR_PAR:
   build_i16h(d->param.starter_off);
   build_i16h(d->param.smap_abandon);
   break;

  case FNNAME_DAT:
   build_i8h(TABLES_NUMBER + TUNABLE_TABLES_NUMBER);
#ifdef REALTIME_TABLES
   if (index < TABLES_NUMBER) //from FLASH
   {
    build_i8h(index);
    build_fs(fw_data.tables[index].name, F_NAME_SIZE);
   }
   else //from EEPROM
   {
    if (eeprom_is_idle())
    {
     build_i8h(index);
     eeprom_read(&uart.send_buf[uart.send_size], (uint16_t)((f_data_t*)(EEPROM_REALTIME_TABLES_START))[index - TABLES_NUMBER].name, F_NAME_SIZE);
     uart.send_size+=F_NAME_SIZE;
    }
    else //skip this item - will be transferred next time
    {
     index = TABLES_NUMBER - 1;
     build_i8h(index);
     build_fs(fw_data.tables[index].name, F_NAME_SIZE);
    }
   }
#else
   build_i8h(index);
   build_fs(fw_data.tables[index].name, F_NAME_SIZE);
#endif
   ++index;
   if (index>=(TABLES_NUMBER + TUNABLE_TABLES_NUMBER)) index = 0;
    break;

  case SENSOR_DAT:
   build_i16h(d->sens.frequen);
   build_i16h(d->sens.map);
   build_i16h(d->sens.voltage);
   build_i16h(d->sens.temperat);
   build_i16h(d->curr_angle);
   build_i16h(d->sens.knock_k);  // <-- knock value
   build_i16h(d->knock_retard);  // <-- knock retard
   build_i8h(d->airflow);
   //boolean values
   build_i8h((d->ie_valve   << 0) |
             (d->sens.carb  << 1) |
             (d->sens.gas   << 2) |
             (d->fe_valve   << 3) |
             (d->ce_state   << 4));
   break;

  case ADCCOR_PAR:
   build_i16h(d->param.map_adc_factor);
   build_i32h(d->param.map_adc_correction);
   build_i16h(d->param.ubat_adc_factor);
   build_i32h(d->param.ubat_adc_correction);
   build_i16h(d->param.temp_adc_factor);
   build_i32h(d->param.temp_adc_correction);
   break;

  case ADCRAW_DAT:
   build_i16h(d->sens.map_raw);
   build_i16h(d->sens.voltage_raw);
   build_i16h(d->sens.temperat_raw);
   build_i16h(d->sens.knock_k);   //<-- knock signal level
   break;

  case CKPS_PAR:
   build_i4h(d->param.ckps_edge_type);
   build_i8h(d->param.ckps_cogs_btdc);
   build_i8h(d->param.ckps_ignit_cogs);
   build_i8h(d->param.ckps_engine_cyl);
   build_i4h(d->param.merge_ign_outs);
   build_i8h(d->param.ckps_cogs_num);
   build_i8h(d->param.ckps_miss_num);
   break;

  case OP_COMP_NC:
   build_i16h(d->op_comp_code);
   break;

  case CE_ERR_CODES:
   build_i16h(d->ecuerrors_for_transfer);
   break;

  case KNOCK_PAR:
   build_i4h(d->param.knock_use_knock_channel);
   build_i8h(d->param.knock_bpf_frequency);
   build_i16h(d->param.knock_k_wnd_begin_angle);
   build_i16h(d->param.knock_k_wnd_end_angle);
   build_i8h(d->param.knock_int_time_const);

   build_i16h(d->param.knock_retard_step);
   build_i16h(d->param.knock_advance_step);
   build_i16h(d->param.knock_max_retard);
   build_i16h(d->param.knock_threshold);
   build_i8h(d->param.knock_recovery_delay);
   break;

  case CE_SAVED_ERR:
   build_i16h(d->ecuerrors_saved_transfer);
   break;

  case FWINFO_DAT:
   //проверка на то, чтобы мы не вылезли за пределы буфера. 3 символа - заголовок и конец пакета.
#if ((UART_SEND_BUFF_SIZE - 3) < FW_SIGNATURE_INFO_SIZE+8)
 #error "Out of buffer!"
#endif
   build_fs(fw_data.exdata.fw_signature_info, FW_SIGNATURE_INFO_SIZE);
   build_i32h(PGM_GET_DWORD(&fw_data.cddata.config)); //<--compile-time options
   break;

  case MISCEL_PAR:
   build_i16h(d->param.uart_divisor);
   build_i8h(d->param.uart_period_t_ms);
   build_i4h(d->param.ign_cutoff);
   build_i16h(d->param.ign_cutoff_thrd);
   build_i8h(d->param.hop_start_cogs);
   build_i8h(d->param.hop_durat_cogs);
   break;
 
#ifdef REALTIME_TABLES
//Following finite state machine will transfer all table's data
  case EDITAB_PAR:
  {
   static uint8_t fuel = 0, state = 0, wrk_index = 0;
   build_i4h(fuel);
   build_i4h(state);
   switch(state)
   {
    case ETMT_STRT_MAP: //start map
     build_i8h(0); //<--not used
     build_rb((uint8_t*)&d->tables_ram[fuel].f_str, F_STR_POINTS);
     state = ETMT_IDLE_MAP;
     break;
    case ETMT_IDLE_MAP: //idle map
     build_i8h(0); //<--not used
     build_rb((uint8_t*)&d->tables_ram[fuel].f_idl, F_IDL_POINTS);
     state = ETMT_WORK_MAP, wrk_index = 0;
     break;
    case ETMT_WORK_MAP: //work map
     build_i8h(wrk_index*F_WRK_POINTS_L);
     build_rb((uint8_t*)&d->tables_ram[fuel].f_wrk[wrk_index][0], F_WRK_POINTS_F);
     if (wrk_index >= F_WRK_POINTS_L-1 )
     {
      wrk_index = 0;
      state = ETMT_TEMP_MAP;
     }
     else
      ++wrk_index; 
     break;
    case ETMT_TEMP_MAP: //temper. correction.
     build_i8h(0); //<--not used
     build_rb((uint8_t*)&d->tables_ram[fuel].f_tmp, F_TMP_POINTS);
     state = ETMT_NAME_STR;
     break;
    case ETMT_NAME_STR:
     build_i8h(0); //<--not used
     build_rs(d->tables_ram[fuel].name, F_NAME_SIZE);
     if (fuel >= ETTS_GAS_SET)  //last
      fuel = ETTS_GASOLINE_SET; //first
     else
      ++fuel;
     state = ETMT_STRT_MAP;
     break;
   }
   break;
  }
#endif

#ifdef DEBUG_VARIABLES
  case DBGVAR_DAT:
   build_i16h(/*Your variable here*/0);
   build_i16h(/*Your variable here*/0);
   build_i16h(/*Your variable here*/0);
   build_i16h(/*Your variable here*/0);
   break;
#endif
#ifdef DIAGNOSTICS
  case DIAGINP_DAT:
   build_i16h(d->diag_inp.voltage);
   build_i16h(d->diag_inp.map);
   build_i16h(d->diag_inp.temp);
   build_i16h(d->diag_inp.add_io1);
   build_i16h(d->diag_inp.add_io2);
   build_i16h(d->diag_inp.carb);
   build_i16h(d->diag_inp.ks_1);
   build_i16h(d->diag_inp.ks_2);
   build_i8h(d->diag_inp.bits);
   break;
#endif
 }//switch

 //общая часть для всех пакетов
 uart.send_buf[uart.send_size++] = '\r';

 //буфер передатчика содержит полностью готовый пакет - начинаем передачу
 uart_begin_send();
}

//TODO: remove it from here. It must be in secu3.c, use callback. E.g. on_bl_starting()
/** Initialization of used I/O ports (производит инициализацию линий портов) */
void ckps_init_ports(void);

uint8_t uart_recept_packet(struct ecudata_t* d)
{
 //буфер приемника содержит дескриптор пакета и данные
 uint8_t temp;
 uint8_t descriptor;

 uart.recv_index = 0;

 descriptor = uart.recv_buf[uart.recv_index++];

// TODO: сделать проверку uart_recv_size для каждого типа пакета.
// Проверять байты пакетов на принадлежность к шестнадцатерчным символам

 //интерпретируем данные принятого фрейма в зависимости от дескриптора
 switch(descriptor)
 {
  case CHANGEMODE:
   uart_set_send_mode(uart.recv_buf[uart.recv_index++]);
   break;

  case BOOTLOADER:
   //TODO: in the future use callback and move following code out
   //передатчик занят. необходимо подождать его освобождения и только потом запускать бутлоадер
   while (uart_is_sender_busy());
   //если в бутлоадере есть команда "cli", то эту строчку можно убрать
   _DISABLE_INTERRUPT();
   ckps_init_ports();
   //прыгаем на бутлоадер минуя проверку перемычки
   boot_loader_start();
   break;

  case TEMPER_PAR:
   d->param.tmp_use   = recept_i4h();
   d->param.vent_pwm  = recept_i4h();
   d->param.cts_use_map = recept_i4h();
   d->param.vent_on   = recept_i16h();
   d->param.vent_off  = recept_i16h();
   break;

  case CARBUR_PAR:
   d->param.ie_lot  = recept_i16h();
   d->param.ie_hit  = recept_i16h();
   d->param.carb_invers= recept_i4h();
   d->param.fe_on_threshold= recept_i16h();
   d->param.ie_lot_g = recept_i16h();
   d->param.ie_hit_g = recept_i16h();
   d->param.shutoff_delay = recept_i8h();
   break;

  case IDLREG_PAR:
   d->param.idl_regul = recept_i4h();
   d->param.ifac1     = recept_i16h();
   d->param.ifac2     = recept_i16h();
   d->param.MINEFR    = recept_i16h();
   d->param.idling_rpm = recept_i16h();
   d->param.idlreg_min_angle = recept_i16h();
   d->param.idlreg_max_angle = recept_i16h();
   break;

  case ANGLES_PAR:
   d->param.max_angle = recept_i16h();
   d->param.min_angle = recept_i16h();
   d->param.angle_corr= recept_i16h();
   d->param.angle_dec_spead = recept_i16h();
   d->param.angle_inc_spead = recept_i16h();
   d->param.zero_adv_ang = recept_i4h();
   break;

  case FUNSET_PAR:
   temp = recept_i8h();
   if (temp < TABLES_NUMBER + TUNABLE_TABLES_NUMBER)
    d->param.fn_gasoline = temp;

   temp = recept_i8h();
   if (temp < TABLES_NUMBER + TUNABLE_TABLES_NUMBER)
    d->param.fn_gas = temp;

   d->param.map_lower_pressure = recept_i16h();
   d->param.map_upper_pressure = recept_i16h();
   d->param.map_curve_offset = recept_i16h();
   d->param.map_curve_gradient = recept_i16h();
   break;

  case STARTR_PAR:
   d->param.starter_off = recept_i16h();
   d->param.smap_abandon= recept_i16h();
   break;

  case ADCCOR_PAR:
   d->param.map_adc_factor     = recept_i16h();
   d->param.map_adc_correction = recept_i32h();
   d->param.ubat_adc_factor    = recept_i16h();
   d->param.ubat_adc_correction= recept_i32h();
   d->param.temp_adc_factor    = recept_i16h();
   d->param.temp_adc_correction= recept_i32h();
   break;

  case CKPS_PAR:
   d->param.ckps_edge_type = recept_i4h();
   d->param.ckps_cogs_btdc  = recept_i8h();
   d->param.ckps_ignit_cogs = recept_i8h();
   d->param.ckps_engine_cyl = recept_i8h();
   d->param.merge_ign_outs = recept_i4h();
   d->param.ckps_cogs_num = recept_i8h();
   d->param.ckps_miss_num = recept_i8h();
   break;

  case OP_COMP_NC:
   d->op_actn_code = recept_i16h();
   break;

  case KNOCK_PAR:
   d->param.knock_use_knock_channel = recept_i4h();
   d->param.knock_bpf_frequency   = recept_i8h();
   d->param.knock_k_wnd_begin_angle = recept_i16h();
   d->param.knock_k_wnd_end_angle = recept_i16h();
   d->param.knock_int_time_const = recept_i8h();

   d->param.knock_retard_step = recept_i16h();
   d->param.knock_advance_step = recept_i16h();
   d->param.knock_max_retard = recept_i16h();
   d->param.knock_threshold = recept_i16h();
   d->param.knock_recovery_delay = recept_i8h();
   break;

  case CE_SAVED_ERR:
   d->ecuerrors_saved_transfer = recept_i16h();
   break;

  case MISCEL_PAR:
   d->param.uart_divisor = recept_i16h();
   d->param.uart_period_t_ms = recept_i8h();
   d->param.ign_cutoff = recept_i4h();
   d->param.ign_cutoff_thrd = recept_i16h();
   d->param.hop_start_cogs = recept_i8h();
   d->param.hop_durat_cogs = recept_i8h();
   break;

#ifdef REALTIME_TABLES
  case EDITAB_PAR:
  {
   uint8_t fuel = recept_i4h();
   uint8_t state = recept_i4h();
   uint8_t addr = recept_i8h();
   uart.recv_size-=5; //[d][x][x][xx]
   switch(state)
   {
    case ETMT_STRT_MAP: //start map
     recept_rb(((uint8_t*)&d->tables_ram[fuel].f_str) + addr, F_STR_POINTS); /*F_STR_POINTS max*/
     break;
    case ETMT_IDLE_MAP: //idle map
     recept_rb(((uint8_t*)&d->tables_ram[fuel].f_idl) + addr, F_IDL_POINTS); /*F_IDL_POINTS max*/
     break;
    case ETMT_WORK_MAP: //work map
     recept_rb(((uint8_t*)&d->tables_ram[fuel].f_wrk[0][0]) + addr, F_WRK_POINTS_F); /*F_WRK_POINTS_F max*/
     break;
    case ETMT_TEMP_MAP: //temper. correction map
     recept_rb(((uint8_t*)&d->tables_ram[fuel].f_tmp) + addr, F_TMP_POINTS); /*F_TMP_POINTS max*/
     break;
    case ETMT_NAME_STR: //name
     recept_rs((d->tables_ram[fuel].name) + addr, F_NAME_SIZE); /*F_NAME_SIZE max*/
     break;
   }
  }
  break;
#endif
#ifdef DIAGNOSTICS
  case DIAGOUT_DAT:
   d->diag_out = recept_i16h();
   break;
#endif
 }//switch

 return descriptor;
}


void uart_notify_processed(void)
{
 uart.recv_size = 0;
}

uint8_t uart_is_sender_busy(void)
{
 return (uart.send_size > 0);
}

uint8_t uart_is_packet_received(void)
{
 return (uart.recv_size > 0);
}

uint8_t uart_get_send_mode(void)
{
 return uart.send_mode;
}

uint8_t uart_set_send_mode(uint8_t descriptor)
{
 return uart.send_mode = descriptor;
}

void uart_init(uint16_t baud)
{
 // Set baud rate
 UBRRH = (uint8_t)(baud>>8);
 UBRRL = (uint8_t)baud;
 UCSRA = _BV(U2X);                                           //удвоение используем для минимизации ошибки
 UCSRB=_BV(RXCIE)|_BV(RXEN)|_BV(TXEN);                       //приемник,прерывание по приему и передатчик разрешены
#ifdef URSEL
 UCSRC=_BV(URSEL)/*|_BV(USBS)*/|_BV(UCSZ1)|_BV(UCSZ0);       //8 бит, 1 стоп, нет контроля четности
#else
 UCSRC=/*_BV(USBS)|*/_BV(UCSZ1)|_BV(UCSZ0);                  //8 бит, 1 стоп, нет контроля четности
#endif

 uart.send_size = 0;                                         //передатчик ни чем не озабочен
 uart.recv_size = 0;                                         //нет принятых данных
 uart.send_mode = SENSOR_DAT;
}


/**Interrupt handler for the transfer of bytes through the UART (transmitter data register empty)
 *Обработчик прерывания по передаче байтов через UART (регистр данных передатчика пуст)
 */
ISR(USART_UDRE_vect)
{
 if (uart.send_size > 0)
 {
  UDR = uart.send_buf[uart.send_index];
  --uart.send_size;
  ++uart.send_index;
 }
 else
 {//все данные переданы
  UCSRB &= ~_BV(UDRIE); // disable UDRE interrupt
 }
}

/**Interrupt handler for receive data through the UART */
ISR(USART_RXC_vect)
{
 static uint8_t state=0;
 uint8_t chr = UDR;

 _ENABLE_INTERRUPT();
 switch(state)
 {
  case 0:            //принимаем (ожидаем символ начала посылки)
   if (uart.recv_size!=0) //предыдущий принятый фрейм еще не обработан, а нам уже прислали новый.
    break;

   if (chr=='!')   //начало пакета?
   {
    state = 1;
    uart.recv_index = 0;
   }
   break;

  case 1:           //прием данных посылки
   if (chr=='\r')
   {
    state = 0;       //КА в исходное состояние
    uart.recv_size = uart.recv_index; //данные готовы, сохраняем их размер
   }
   else
   {
    if (uart.recv_index >= UART_RECV_BUFF_SIZE)
    {
     //Ошибка: переполнение! - КА в исходное состояние, фрейм нельзя считать принятым!
     state = 0;
    }
    else
     uart.recv_buf[uart.recv_index++] = chr;
   }
   break;
 }
}
