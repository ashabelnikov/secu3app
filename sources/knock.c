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

/** \file knock.c
 * \author Alexey A. Shabelnikov
 * Implementation of knock chip related functions.
 * Service of HIP9011 knock signal processing chip
 */

#include "port/avrio.h"
#include "port/interrupt.h"
#include "port/intrinsic.h"
#include "port/port.h"
#include "bitmask.h"
#include "knock.h"

#ifndef SECU3T //---SECU-3i---
//See ioconfig.c for more information
extern uint8_t  spi_PORTA;   //!< Bits of inputs read by SPI
extern uint8_t  spi_PORTB;   //!< Bits of outputs controlled by SPI
extern uint8_t  spi_IODIRA;  //!< Direction control bits for SPI PORTA (inputs)
extern uint8_t  spi_IODIRB;  //!< Direction control bits for SPI PORTB (outputs)
extern uint8_t  spi_GPPUA;   //!< Pull-up resistors control register A
extern uint8_t  spi_GPPUB;   //!< Pull-up resistors control register B
#endif

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
#define KSP_PRESCALER_16MHZ    0x0C   //!< code for setup prescaler (16mHz crystal)
#define KSP_PRESCALER_20MHZ    0x0E   //!< code for setup prescaler (20mHz crystal)

#define SET_KSP_CS(v) WRITEBIT(PORTB, PB4, v) //!< SS controls chip selection
#define SET_KSP_INTHOLD(v) WRITEBIT(PORTC, PC4, v) //!< Switches between integration/hold modes (SECU-3T)
#define SET_KSP_TEST(v) WRITEBIT(PORTB, PB3, v)     //!< Switches chip into diagnostic mode (SECU-3T) or controls EX_CS (SECU-3i)
#define KSP_PRESCALER_VALUE KSP_PRESCALER_20MHZ  //!< set prescaler for 20mHz crystal

/**This data structure intended for duplication of data of current state
 * of signal processor */
typedef struct
{
 uint8_t ksp_bpf;                       //!< band pass frequency
 volatile uint8_t ksp_gain;             //!< attenuator gain
 volatile uint8_t ksp_inttime;          //!< integrator's time constant
 volatile uint8_t ksp_channel;          //!< current channel number (2 channels are available)
 volatile uint8_t ksp_interrupt_state;  //!< for state machine executed inside interrupt handler
 uint8_t ksp_error;                     //!< stores errors flags
#ifndef SECU3T //---SECU-3i---
 volatile uint8_t pending_request;      //!< pending requests flag
#endif
}kspstate_t;

/**State variables */
kspstate_t ksp;

//For working with hardware part of SPI
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
 //Note: In the SECU-3i TEST is not connected to PB3 (left NC)
 SET_KSP_TEST(1);
 SET_KSP_INTHOLD(KNOCK_INTMODE_HOLD);
 SET_KSP_CS(1);

 spi_master_init();
 ksp.ksp_interrupt_state = 0; //init state machine
 ksp.ksp_error = 0;

 CLEARBIT(SPCR, CPOL);

#ifndef SECU3T //---SECU-3i---
 ksp.pending_request = 0;
#endif //SECU-3i

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

#ifndef SECU3T //---SECU-3i---

uint8_t knock_expander_initialize()
{
 uint8_t _t;

 _t=_SAVE_INTERRUPT();
 _DISABLE_INTERRUPT();
 ksp.pending_request = 0;
 ksp.ksp_interrupt_state = 0; //init state machine

 spi_master_init();

 SETBIT(SPCR, CPOL);

 SET_KSP_TEST(0);
 _DELAY_US(2);
 SET_KSP_TEST(1);
 _DELAY_US(2);

 //configure port expander MCP23S17
 //expander will be by default in the sequential mode, BANK=0,
 //so, we don't need to initialize IOCON register
/* SET_KSP_TEST(0);
 spi_master_transmit(0x40);        //write
 spi_master_transmit(0x0A);        //address = 0x0A
 spi_master_transmit(0x00);        //IOCON = 0 (BANK=0, SEQOP=0, HAEN=0)
 SET_KSP_TEST(1);
 _DELAY_US(2);*/

 //pull-up resistors:
 SET_KSP_TEST(0);
 spi_master_transmit(0x40);        //write
 spi_master_transmit(0x0C);        //address = 0x0C
 spi_master_transmit(spi_GPPUA);   //GPPUA = spi_GPPUA
 spi_master_transmit(spi_GPPUB);   //GPPUB = spi_GPPUB
 SET_KSP_TEST(1);
 _DELAY_US(2);

 //ports' direction:
 SET_KSP_TEST(0);
 spi_master_transmit(0x40);        //write
 spi_master_transmit(0x00);        //address = 0x00
 spi_master_transmit(spi_IODIRA);  //IODIRA = spi_IODIRA
 spi_master_transmit(spi_IODIRB);  //IODIRB = spi_IODIRB
 SET_KSP_TEST(1);
 _DELAY_US(2);

 //ports' values:
 SET_KSP_TEST(0);
 spi_master_transmit(0x40);        //write
 spi_master_transmit(0x12);        //address = 0x12
 spi_master_transmit(spi_PORTA);   //GPIOA = spi_GPIOA
 spi_master_transmit(spi_PORTB);   //GPIOB = spi_GPIOB
 SET_KSP_TEST(1);
 _DELAY_US(2);

 _RESTORE_INTERRUPT(_t);
 return 1; //successfully
}
#endif

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
 _BEGIN_ATOMIC_BLOCK();
 if (0==ksp.ksp_interrupt_state)
 {
  CLEARBIT(SPCR, CPOL);
  SET_KSP_CS(0);
  ksp.ksp_interrupt_state = 1;
  SPDR = ksp.ksp_bpf;
  //enable interrupt, sending of the remaining data will be completed in
  //interrupt's state machine
  SPCR|= _BV(SPIE);
 }
 else if (ksp.ksp_interrupt_state < 5)
  ksp.ksp_error = 1; //previous latching is not finished yet
#ifndef SECU3T //---SECU-3i---
 else
  ksp.pending_request = 1; //if busy by MCP23S17, then just set request flag and exit
#endif
 _END_ATOMIC_BLOCK();
}

#ifndef SECU3T //---SECU-3i---
void knock_start_expander_latching(void)
{
// _BEGIN_ATOMIC_BLOCK(); we rely that at the moment of calling of this function interrupts are disabled, so don't disable it twice
 if (0==ksp.ksp_interrupt_state)
 {
  SETBIT(SPCR, CPOL);
 _NO_OPERATION();
 _NO_OPERATION();
 _NO_OPERATION();
 _NO_OPERATION();
  SET_KSP_TEST(0);
  ksp.ksp_interrupt_state = 5;
 _NO_OPERATION();
 _NO_OPERATION();
 _NO_OPERATION();
 _NO_OPERATION();
  SPDR = 0x40; //write
  //enable interrupt, sending of the remaining data will be completed in
  //interrupt's state machine
  SPCR|= _BV(SPIE);
 }
 else if (ksp.ksp_interrupt_state > 4)
  ksp.ksp_error = 1; //previous latching is not finished yet
 else
  ksp.pending_request = 1; //if busy by HIP9011, then set request flag and exit
// _END_ATOMIC_BLOCK();
}
#endif

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

void knock_set_channel(uint8_t channel)
{
 _BEGIN_ATOMIC_BLOCK();
 ksp.ksp_channel = KSP_SET_CHANNEL | (channel & 0x01);
 _END_ATOMIC_BLOCK();
}

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
 //signal processor requires transition of CS into high level after each sent
 //byte, at least for 200ns
#ifndef SECU3T //---SECU-3i---
 if (ksp.ksp_interrupt_state < 5)
#endif
 { SET_KSP_CS(1); }

 _ENABLE_INTERRUPT();

 switch(ksp.ksp_interrupt_state)
 {
  case 0:   //state machine stopped
   break;

  case 1: //BPF loaded
   SET_KSP_CS(0);
   ++ksp.ksp_interrupt_state;
   SPDR = ksp.ksp_gain;
   break;

  case 2: //Gain loaded
   SET_KSP_CS(0);
   ++ksp.ksp_interrupt_state;
   SPDR = ksp.ksp_inttime;
   break;

  case 3: //Int.Time loaded
   SET_KSP_CS(0);
   ++ksp.ksp_interrupt_state;
   SPDR = ksp.ksp_channel;
   break;

  case 4: //channel number loaded
#ifdef SECU3T
   ksp.ksp_interrupt_state = 0; //idle
   //disable interrupt and switch state machine into initial state - ready to new load
   SPCR&= ~_BV(SPIE);
   break;
#else //---SECU-3i---
   {
   _BEGIN_ATOMIC_BLOCK();
   if (ksp.pending_request)
   {//start loading data into the expander chip
    SETBIT(SPCR, CPOL);
    SET_KSP_TEST(0);
    ksp.pending_request = 0;
    ++ksp.ksp_interrupt_state; // busy (state = 5)
    SPDR = 0x40; //write opcode
   }
   else
   {
    ksp.ksp_interrupt_state = 0; //idle
    //disable interrupt and switch state machine into initial state - ready to new load
    SPCR&= ~_BV(SPIE);
   }
   _END_ATOMIC_BLOCK();
   }
   break;

  case 5: //expander, opcode loaded
   SPDR = 0x13; //address of the GPIOB
   ++ksp.ksp_interrupt_state;
   break;
  case 6: //GPIOB address loaded
   SPDR = spi_PORTB;
   ++ksp.ksp_interrupt_state;
   break;
  case 7: //GPIOB wrote!

   SET_KSP_TEST(1); //deselect the device
   ++ksp.ksp_interrupt_state;
   _NO_OPERATION();
   _NO_OPERATION();
   _NO_OPERATION();
   _NO_OPERATION();
   SET_KSP_TEST(0);
   _NO_OPERATION();
   _NO_OPERATION();
   _NO_OPERATION();
   _NO_OPERATION();
   SPDR = 0x41;  //read opcode
   break;
  case 8: //expander, opcode loaded
   SPDR = 0x12; //address of the GPIOA
   ++ksp.ksp_interrupt_state;
   break;
  case 9: //GPIOA address loaded
   SPDR = 0x00;  //shift read register
   ++ksp.ksp_interrupt_state;
   break;
  case 10: //GPIOA read!
   spi_PORTA = SPDR;
   SET_KSP_TEST(1);
   _BEGIN_ATOMIC_BLOCK();
   if (ksp.pending_request)
   {//start loading data into the knock chip
    CLEARBIT(SPCR, CPOL);
    SET_KSP_CS(0);
    ksp.pending_request = 0;
    ksp.ksp_interrupt_state = 1; //busy (state = 1)
    SPDR = ksp.ksp_bpf;
   }
   else
   {
    ksp.ksp_interrupt_state = 0; //idle
    //disable interrupt and switch state machine into initial state - ready to new load
    SPCR&= ~_BV(SPIE);
   }
   _END_ATOMIC_BLOCK();
   break;
#endif

 }
}

void knock_init_ports(void)
{
 PORTB|= _BV(PB4)|_BV(PB3); //interface with HIP9011 turned off (CS=1, TEST=1, MOSI=0, SCK=0)
 PORTC&=~_BV(PC4);
 DDRB |= _BV(DDB7)|_BV(DDB5)|_BV(DDB4)|_BV(DDB3);
 DDRC |= _BV(DDC4);
}
