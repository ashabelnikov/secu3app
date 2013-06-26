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

/** \file knock.c
 * Implementation of knock chip related functions.
 * Service of HIP9011 knock signal processing chip
 * (���������� ������������ ���� HIP9011 ��������������� ������ ���������).
 */

#include "port/avrio.h"
#include "port/interrupt.h"
#include "port/intrinsic.h"
#include "port/port.h"
#include "bitmask.h"
#include "knock.h"
#include "secu3.h"
#include "ce_errors.h"

//HIP9011 - Knock Signal Processor.

//Command codes and quick description (crib)
#define KSP_SET_BANDPASS       0x00   //!< 00FFFFFF, F - BPF frequency code
#define KSP_SET_GAIN           0x80   //!< 10GGGGGG, G - gain code
#define KSP_SET_INTEGRATOR     0xC0   //!< 110IIIII, I - integrator time constant code
#define KSP_SET_PRESCALER      0x40   //!< 01XPPPPZ, X - don't care, P - prescaler code, Z - SO terminal status
#define KSP_SET_CHANNEL        0xE0   //!< 111TTTTC, T - diagnostic mode code, C - channel code
// if Z = 0, then SO terminal is active, otherwise Hi Z
// if C = 0, then channel 0 selected, otherwise channel 1
// Question. How to use T - bits, how many diagnostic modes we have? Datasheet doesn't contain
//such information...
// SO directly corresponds to SI,(if enabled) without delay.


//SO status values
#define KSP_SO_TERMINAL_ACTIVE 0x00   //!< code for activation of SO terminal
#define KSP_SO_TERMINAL_HIZ    0x01   //!< code for deactivation of SO terminal

//prescaler
#define KSP_PRESCALER_4MHZ     0x00   //!< code for setup prescaler (4mHz crystal)
#define KSP_PRESCALER_16MHZ    0x0C   //!< code for setup prescaler (16mHz crystal)

#define SET_KSP_CS(v) WRITEBIT(PORTB, PB4, v) //!< SS controls chip selection
#ifdef SECU3T /*SECU-3T*/
 #define SET_KSP_INTHOLD(v) WRITEBIT(PORTC, PC4, v) //!< Switches between integration/hold modes
#else         /*SECU-3*/
 #define SET_KSP_INTHOLD(v) WRITEBIT(PORTD, PD3, v)
#endif
#define SET_KSP_TEST(v) WRITEBIT(PORTB, PB3, v)     //!< Switches chip into diagnostic mode

//4 and 16 mHz crystals can be used (4mHz is default value)
#if defined(Z1_CRYSTAL_16MHZ) | defined(SECU3T)
 #define KSP_PRESCALER_VALUE KSP_PRESCALER_16MHZ  //!< set prescaler for 16mHz crystal
#else
 #define KSP_PRESCALER_VALUE KSP_PRESCALER_4MHZ   //!< set prescaler for 4mHz crystal
#endif

/**This data structure intended for duplication of data of current state
 * of signal processor */
typedef struct
{
 uint8_t ksp_bpf;                       //!< band pass frequency
 volatile uint8_t ksp_gain;             //!< attenuator gain
 volatile uint8_t ksp_inttime;          //!< integrator's time constant
#ifdef SECU3T
 volatile uint8_t ksp_channel;          //!< current channel number (2 channels are available)
#endif
 volatile uint8_t ksp_interrupt_state;  //!< for state machine executed inside interrupt handler
 uint8_t ksp_error;                     //!< stores errors flags
 volatile uint8_t ksp_last_word;        //!< used to control of latching
}kspstate_t;

/**State variables */
kspstate_t ksp;

//For work with hardware part of SPI
/**Initialization of SPI in master mode */
static void spi_master_init(void);
/**Transmit single byte via SPI
 * \param i_byte byte to transmit
 */
static void spi_master_transmit(uint8_t i_byte);

void knock_set_integration_mode(uint8_t mode)
{
 SET_KSP_INTHOLD(mode);
}

uint8_t knock_module_initialize(void)
{
 uint8_t i, response;
 uint8_t init_data[2] = {KSP_SET_PRESCALER | KSP_PRESCALER_VALUE | KSP_SO_TERMINAL_ACTIVE,
                         KSP_SET_CHANNEL | KSP_CHANNEL_0};
 uint8_t _t;

 _t=_SAVE_INTERRUPT();
 _DISABLE_INTERRUPT();

 //Setting HOLD mode for integrator and "Run" mode for chip at all.
 SET_KSP_TEST(1);
 SET_KSP_INTHOLD(KNOCK_INTMODE_HOLD);
 SET_KSP_CS(1);

 spi_master_init();
 ksp.ksp_interrupt_state = 0; //init state machine
 ksp.ksp_error = 0;

 //set prescaler first
 SET_KSP_CS(0);
 spi_master_transmit(init_data[0]);
 SET_KSP_CS(1);

 //Setting SO terminal active and perform initialization. For each parameter perform
 //checking for response and correcntess of received data.
 for(i = 0; i < 2; ++i)
 {
  SET_KSP_CS(0);
  spi_master_transmit(init_data[i]);
  SET_KSP_CS(1);
  response = SPDR;
  if (response!=init_data[i])
  {
   _RESTORE_INTERRUPT(_t);
   return 0; //error - chip doesn't respond!
  }
 }

 _RESTORE_INTERRUPT(_t);
 //Initialization completed successfully
 return 1;
}

//Initializes SPI in master mode
static void spi_master_init(void)
{
 _BEGIN_ATOMIC_BLOCK();
 // enable SPI, master, clock = fck/16, data on falling edge of SCK
 SPCR = _BV(SPE)|_BV(MSTR)|_BV(SPR0)|_BV(CPHA);
 _END_ATOMIC_BLOCK();
}

//Sends one byte via SPI
//i_byte - byte for sending
static void spi_master_transmit(uint8_t i_byte)
{
 _NO_OPERATION();
 _NO_OPERATION();
 //Begin of sending
 SPDR = i_byte;
 //Waiting for completion of sending
 while(!(SPSR & _BV(SPIF)));
 _NO_OPERATION();
 _NO_OPERATION();
}

void knock_start_settings_latching(void)
{
 if (ksp.ksp_interrupt_state)
  ksp.ksp_error = 1;

 SET_KSP_CS(0);
 ksp.ksp_interrupt_state = 1;
 SPDR = ksp.ksp_last_word = ksp.ksp_bpf;
 //enable interrupt, sending of the remaining data will be completed in
 //interrupt's state machine
 SPCR|= _BV(SPIE);
}

uint8_t knock_is_latching_idle(void)
{
 return (ksp.ksp_interrupt_state) ? 0 : 1;
}

void knock_set_band_pass(uint8_t freq)
{
 _BEGIN_ATOMIC_BLOCK();
 ksp.ksp_bpf = KSP_SET_BANDPASS | (freq & 0x3F);
 _END_ATOMIC_BLOCK();
}

void knock_set_gain(uint8_t gain)
{
 _BEGIN_ATOMIC_BLOCK();
 ksp.ksp_gain = KSP_SET_GAIN | (gain & 0x3F);
 _END_ATOMIC_BLOCK();
}

void knock_set_int_time_constant(uint8_t inttime)
{
 _BEGIN_ATOMIC_BLOCK();
 ksp.ksp_inttime = KSP_SET_INTEGRATOR | (inttime & 0x1F);
 _END_ATOMIC_BLOCK();
}

#ifdef SECU3T
void knock_set_channel(uint8_t channel)
{
 _BEGIN_ATOMIC_BLOCK();
 ksp.ksp_channel = KSP_SET_CHANNEL | (channel & 0x01);
 _END_ATOMIC_BLOCK();
}
#endif

uint8_t knock_is_error(void)
{
 return ksp.ksp_error;
}

void knock_reset_error(void)
{
 ksp.ksp_error = 0;
}

/** Interrupt handler from SPI */
ISR(SPI_STC_vect)
{
 uint8_t t = SPDR;
 //signal processor requires transition of CS into high level after each sent
 //byte, at least for 200ns
 SET_KSP_CS(1);

 _ENABLE_INTERRUPT();

 switch(ksp.ksp_interrupt_state)
 {
  case 0:   //state machine stopped
   break;

  case 1: //BPF loaded
   SET_KSP_CS(0);
   ksp.ksp_interrupt_state = 2;
   if (t!=ksp.ksp_last_word)
    ksp.ksp_error = 1;
   SPDR = ksp.ksp_last_word = ksp.ksp_gain;
   break;

  case 2: //Gain loaded
   SET_KSP_CS(0);
   ksp.ksp_interrupt_state = 3;
   if (t!=ksp.ksp_last_word)
    ksp.ksp_error = 1;
   SPDR = ksp.ksp_last_word = ksp.ksp_inttime;
   break;

#ifndef SECU3T
  case 3: //Int.Time loaded
   if (t!=ksp.ksp_last_word)
    ksp.ksp_error = 1;
   //disable interrupt and switch state machine into initial state - ready to new load
   SPCR&= ~_BV(SPIE);
   ksp.ksp_interrupt_state = 0;
   break;
#else /*SECU-3T*/
  case 3: //Int.Time loaded
   SET_KSP_CS(0);
   ksp.ksp_interrupt_state = 4;
   if (t!=ksp.ksp_last_word)
    ksp.ksp_error = 1;
   SPDR = ksp.ksp_last_word = ksp.ksp_channel;
   break;

  case 4: //channel number loaded
   if (t!=ksp.ksp_last_word)
    ksp.ksp_error = 1;
   //disable interrupt and switch state machine into initial state - ready to new load
   SPCR&= ~_BV(SPIE);
   ksp.ksp_interrupt_state = 0;
   break;
#endif
 }
}

void knock_init_ports(void)
{
 PORTB|= _BV(PB4)|_BV(PB3); //interface with HIP9011 turned off (CS=1, TEST=1, MOSI=0, SCK=0)
#ifdef SECU3T
 PORTC&=~_BV(PC4);
#else
 PORTD&=~_BV(PD3);          //INT/~HOLD = 0 (hold mode)
#endif
 DDRB |= _BV(DDB7)|_BV(DDB5)|_BV(DDB4)|_BV(DDB3);
#ifdef SECU3T
 DDRC |= _BV(DDC4);
#else
 DDRD |= _BV(DDD3);
#endif
}

void knock_init(struct params_t *param, uint8_t gain)
{
 //��������������� ������������� ���������� ����������� ���������� ���������
 knock_set_band_pass(param->knock_bpf_frequency);
 knock_set_gain(gain);
 knock_set_int_time_constant(param->knock_int_time_const);

 if (param->knock_use_knock_channel)
  if (!knock_module_initialize())
  {//��� ����������� ���������� ��������� ���������� - �������� ��
   ce_set_error(ECUERROR_KSP_CHIP_FAILED);
  }
}
