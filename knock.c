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

#include <inavr.h>
#include <ioavr.h>
#include "knock.h"

//HIP9011 - Knock Signal Processor.

//Command codes and quick description (crib)
#define KSP_SET_BANDPASS       0x00   //00FFFFFF, F - BPF frequency code
#define KSP_SET_GAIN           0x80   //10GGGGGG, G - gain code
#define KSP_SET_INTEGRATOR     0xC0   //110IIIII, I - integrator time constant code
#define KSP_SET_PRESCALER      0x40   //01XPPPPZ, X - don't care, P - prescaler code, Z - SO terminal status
#define KSP_SET_CHANNEL        0xE0   //111TTTTC, T - diagnostic mode code, C - channel code
// if Z = 0, then SO terminal is active, otherwise Hi Z
// if C = 0, then channel 0 selected, otherwise channel 1  
// Question. How to use T - bits, how many diagnostic modes we have? Datasheet doesn't contain 
//such information...
// SO directly corresponds to SI,(if enabled) without delay.


//SO status values
#define KSP_SO_TERMINAL_ACTIVE 0x00
#define KSP_SO_TERMINAL_HIZ    0x01

//channel selection values
#define KSP_CHANNEL_0          0x00
#define KSP_CHANNEL_1          0x01

//prescaler
#define KSP_PRESCALER_4MHZ     0x00


#define KSP_CS PORTB_Bit4        //SS controls chip selection
#define KSP_INTHOLD PORTD_Bit3   //Switches between integration/hold modes
#define KSP_TEST PORTB_Bit3      //Switches chip into diagnostic mode


//This data structure intended for duplication of data of current state
// of signal processor
typedef struct
{
 uint8_t ksp_bpf;
 volatile uint8_t ksp_gain;
 volatile uint8_t ksp_inttime;
 volatile uint8_t ksp_interrupt_state;
 uint8_t ksp_error;
 volatile uint8_t ksp_last_word;
}kspstate_t;

kspstate_t ksp;

//For work with hardware part of SPI
void spi_master_init(void);
void spi_master_transmit(uint8_t i_byte);

void knock_set_integration_mode(uint8_t mode)
{
 KSP_INTHOLD = mode;
}

uint8_t knock_module_initialize(void)
{
 uint8_t i, response;
 uint8_t init_data[2] = {KSP_SET_PRESCALER | KSP_PRESCALER_4MHZ | KSP_SO_TERMINAL_ACTIVE,
                         KSP_SET_CHANNEL | KSP_CHANNEL_0};
 uint8_t _t;

 _t=__save_interrupt();
 __disable_interrupt();

 //Setting HOLD mode for integrator and "Run" mode for chip at all.
 KSP_TEST = 1;
 KSP_INTHOLD = KNOCK_INTMODE_HOLD;
 KSP_CS = 1;

 spi_master_init();
 ksp.ksp_interrupt_state = 0; //init state machine
 ksp.ksp_error = 0;

 //set prescaler first
 KSP_CS = 0;
 spi_master_transmit(init_data[0]);
 KSP_CS = 1;

 //Setting SO terminal active and perform initialization. For each parameter perform
 //checking for response and correcntess of received data.
 for(i = 0; i < 2; ++i)
 {
  KSP_CS = 0;
  spi_master_transmit(init_data[i]);
  KSP_CS = 1;
  response = SPDR; 
  if (response!=init_data[i])
  {
   __restore_interrupt(_t);
   return 0; //error - chip doesn't respond!
  }
 }

 __restore_interrupt(_t);
 //Initialization completed successfully
 return 1;
}

//Initializes SPI in master mode
__monitor
void spi_master_init(void)
{
 // enable SPI, master, clock = fck/16, data on falling edge of SCK
 SPCR = (1 << SPE)|(1 << MSTR)|(1 << SPR0)|(1 << CPHA);
}

//Sends one byte via SPI
//i_byte - byte for sending
void spi_master_transmit(uint8_t i_byte)
{
 __no_operation();
 __no_operation();
 //Begin of sending
 SPDR = i_byte;
 //Waiting for completion of sending
 while(!(SPSR & (1 << SPIF)));
 __no_operation();
 __no_operation();
}

void knock_start_settings_latching(void)
{
 if (ksp.ksp_interrupt_state)
  ksp.ksp_error = 1;
 
 KSP_CS = 0;
 ksp.ksp_interrupt_state = 1;
 SPDR = ksp.ksp_last_word = ksp.ksp_bpf;
 //enable interrupt, sending of the remaining data will be completed in
 //interrupt's state machine
 SPCR|= (1 << SPIE);
}

uint8_t knock_is_latching_idle(void)
{
 return (ksp.ksp_interrupt_state) ? 0 : 1;
}

__monitor
void knock_set_band_pass(uint8_t freq)
{ 
 ksp.ksp_bpf = KSP_SET_BANDPASS | (freq & 0x3F);
}

__monitor
void knock_set_gain(uint8_t gain)
{
 ksp.ksp_gain = KSP_SET_GAIN | (gain & 0x3F);
}

__monitor
void knock_set_int_time_constant(uint8_t inttime)
{
 ksp.ksp_inttime = KSP_SET_INTEGRATOR | (inttime & 0x1F);
}

uint8_t knock_is_error(void)
{
 return ksp.ksp_error;
}

void knock_reset_error(void)
{
 ksp.ksp_error = 0;
}

#pragma vector=SPI_STC_vect
__interrupt void spi_dataready_isr(void)
{
 uint8_t t;
 //signal processor requires transition of CS into high level after each sent
 //byte, at least for 200ns
 KSP_CS = 1;

 t = SPDR;

 switch(ksp.ksp_interrupt_state)
 {
  case 0:   //state machine stopped
   break;

  case 1: //BPF loaded
   KSP_CS = 0;
   ksp.ksp_interrupt_state = 2;
   if (t!=ksp.ksp_last_word)
    ksp.ksp_error = 1;
   SPDR = ksp.ksp_last_word = ksp.ksp_gain;
   break;

  case 2: //Gain loaded
   KSP_CS = 0;
   ksp.ksp_interrupt_state = 3;
   if (t!=ksp.ksp_last_word)
    ksp.ksp_error = 1;
   SPDR = ksp.ksp_last_word = ksp.ksp_inttime;
   break;

  case 3: //Int.Time loaded
   if (t!=ksp.ksp_last_word) 
    ksp.ksp_error = 1;
   //disable interrupt and switch state machine into initial state - ready to new load
   SPCR&= ~(1 << SPIE); 
   ksp.ksp_interrupt_state = 0;
   break;    
 }
}

void knock_init_ports(void)
{
 PORTB|= (1<<PB4)|(1<<PB3); //interface with HIP9011 turned off (CS=1, TEST=1, MOSI=0, SCK=0)
 PORTD&=~(1<<PD3);          //INT/~HOLD = 0 (hold mode)
 DDRB |= (1<<DDB7)|(1<<DDB5)|(1<<DDB4)|(1<<DDB3);   
 DDRD |= (1<<DDD3);
}
