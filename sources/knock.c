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
#include "obd.h"   //can_t struct
#include <string.h>
#include "tables.h"    //IOP_CAN_CS
#include "ioconfig.h"  //IOP_CAN_CS

//----------------------------------------------------------------------------
#ifdef OBD_SUPPORT
#define SPI_RESET        0xC0 //!< Resets internal registers to the default state, sets Configuration mode.
#define SPI_WRITE        0x02 //!< Writes data to the register beginning at the selected address.
#define SPI_READ_STATUS  0xA0 //!< Quick polling command that reads several status bits for transmit and receive functions.
#define SPI_RX_STATUS    0xB0 //!< Quick polling command that indicates filter match and message type (standard, extended and/or remote) of received message.
#define SPI_READ_RX      0x90 //!< Read RX buffer, 1001 0NM0, NM  - address pointer.
#define SPI_WRITE_TX     0x40 //!< Load TX buffer, 0100 0ABC, ABC - address of buffer.
#define SPI_RTS          0x80 //!< Instructs controller to begin message transmission sequence for any of the transmit buffers, 1000 0NNN, NNN - select buffer TXB2...TXB0.
#define CNF3             0x28 //!< Address of the bit timing configuration register 3. Control of the bit timing for the CAN bus interface.
#define CNF2             0x29 //!< Address of the bit timing configuration register 2. Control of the bit timing for the CAN bus interface.
#define CNF1             0x2A //!< Address of the bit timing configuration register 1. Control of the bit timing for the CAN bus interface.
#define RTR              6    //!< Bit number of the RTR (Remote Transmission Request). See TXBnDLC and RXBnDLC registers 
#define IDE              3    //!< Bit number of the IDE (Extended Identifier Flag). See RXBnSIDL register
#define SRR              4    //!< Bit number of the Standard Frame Remote Transmit Request (valid only if IDE bit = 0). See RXBnSIDL register
#define CANCTRL          0x0F //!< Address of the CANCTRL register. CAN control. 

//Chip select signal pin. In SECU-3T we use PB3. In SECU-3i we use any pin remapped to CAN_CS
#ifdef SECU3T
 #define INIT_CAN_CS() SET_CAN_CS(1);
 #define SET_CAN_CS(v) SET_KSP_TEST(v)
#else //SECU-3i
 #define INIT_CAN_CS() IOCFG_INIT(IOP_CAN_CS, 1)
 #define SET_CAN_CS(v) IOCFG_SET(IOP_CAN_CS, (v))
#endif
#endif //OBD_SUPPORT
//----------------------------------------------------------------------------

#ifndef SECU3T //---SECU-3i---
//See ioconfig.c for more information
extern uint8_t  spi_PORTA;   //!< Bits of inputs read by SPI
extern uint8_t  spi_PORTB;   //!< Bits of outputs controlled by SPI
extern uint8_t  spi_IODIRA;  //!< Direction control bits for SPI PORTA (inputs)
extern uint8_t  spi_IODIRB;  //!< Direction control bits for SPI PORTB (outputs)
extern uint8_t  spi_GPPUA;   //!< Pull-up resistors control register A
extern uint8_t  spi_GPPUB;   //!< Pull-up resistors control register B
#endif
//----------------------------------------------------------------------------

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
#ifdef TPIC8101
#define KSP_SET_ADVANCED       0x71   //!< 01110001, Set SPI configuration to the advanced mode (TPIC8101 only)
#endif

//SO status values
#define KSP_SO_TERMINAL_ACTIVE 0x00   //!< code for activation of SO terminal
#define KSP_SO_TERMINAL_HIZ    0x01   //!< code for deactivation of SO terminal

//prescaler
#define KSP_PRESCALER_16MHZ    0x0C   //!< code for setup prescaler (16MHz crystal)
#define KSP_PRESCALER_20MHZ    0x0E   //!< code for setup prescaler (20MHz crystal)

#define SET_KSP_CS(v) WRITEBIT(PORTB, PB4, v) //!< SS controls chip selection
#define SET_KSP_INTHOLD(v) WRITEBIT(PORTC, PC4, v) //!< Switches between integration/hold modes (SECU-3T)
#define SET_KSP_TEST(v) WRITEBIT(PORTB, PB3, v)     //!< Switches chip into diagnostic mode (SECU-3T) or controls EX_CS (SECU-3i)
#define KSP_PRESCALER_VALUE KSP_PRESCALER_20MHZ  //!< set prescaler for 20MHz crystal

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
#if !defined(SECU3T) || defined(OBD_SUPPORT) //---SECU-3i---
 volatile uint8_t pending_request;      //!< pending requests flag
#endif
#ifdef OBD_SUPPORT
 volatile uint8_t can_pending_tx;       //!< pending message exists and wait for transmition
 volatile uint8_t can_pending_rx;       //!< pending process of checking for or getting a message
 volatile uint8_t can_received;         //!< indicates existing message to obtain
 can_t can_msg_tx;                      //!< CAN message (TX)
 can_t can_msg_rx;                      //!< CAN message (RX)
 uint8_t can_data_idx;                  //!< used by TX and RX sequences of the state machine
 uint8_t can_buff_addr;                 //!< used by TX sequence to store address of buffer
 uint8_t can_rxb_priority;              //!< used to change order of reading RXB0 and RXB1
#endif
#ifdef TPIC8101
 volatile uint16_t adc_value;           //!< Complete 10-bit ADC value read via SPI
 volatile uint8_t adc_low;              //!< Low byte of ADC value read via SPI
#endif
#if !defined(SECU3T) && defined(MCP3204)
 uint8_t spiadc_chidx;
 volatile uint8_t spiadc_hibyte;
#endif
}kspstate_t;

/**State variables */
kspstate_t ksp;

//For working with hardware part of SPI
/**Initialization of SPI in master mode */
static void spi_master_init(void)
{
 _BEGIN_ATOMIC_BLOCK();
 // enable SPI, master, clock = fck/16, data on falling edge of SCK
 SPCR = _BV(SPE)|_BV(MSTR)|_BV(SPR0)|_BV(CPHA);
 _END_ATOMIC_BLOCK();
}

/**Transmit single byte via SPI
 * \param i_byte byte to transmit
 */
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

#ifdef OBD_SUPPORT
/**Write to one of MCP2515 registers*/
static void mcp2515_write_register(uint8_t adress, uint8_t data)
{
 SET_CAN_CS(0);
 spi_master_transmit(SPI_WRITE);
 spi_master_transmit(adress);
 spi_master_transmit(data);
 SET_CAN_CS(1);
}

void knock_set_can_filter(uint8_t idx, uint16_t id)
{
 uint8_t addr;
 switch(idx)
 {
  case 0: addr = 0x00; break;
  case 1: addr = 0x04; break;
  case 2: addr = 0x08; break;
  case 3: addr = 0x10; break;
  case 4: addr = 0x14; break;
  case 5: addr = 0x18; break;
  default: return; //wrong number of filter
 }

 SET_CAN_CS(0);
 spi_master_transmit(SPI_WRITE);
 spi_master_transmit(addr);
 spi_master_transmit(id >> 3); //[10-3] bits
 spi_master_transmit(id << 5); //[2-0]  bits
 SET_CAN_CS(1);
}

void knock_set_can_mask(uint8_t idx, uint16_t id)
{
 uint8_t addr;
 switch(idx)
 {
  case 0: addr = 0x20; break;
  case 1: addr = 0x24; break;
  default: return; //wrong number of mask
 }
 SET_CAN_CS(0);
 spi_master_transmit(SPI_WRITE);
 spi_master_transmit(addr);
 spi_master_transmit(id >> 3); //[10-3] bits
 spi_master_transmit(id << 5); //[2-0]  bits
 SET_CAN_CS(1); 
}

#endif

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
#ifdef SECU3T
 SET_KSP_INTHOLD(KNOCK_INTMODE_HOLD);
 SET_KSP_CS(1);
#else //SECU3i
 IOCFG_SETF(IOP_TACH_O, KNOCK_INTMODE_HOLD);
 IOCFG_SETF(IOP_KSP_CS, 1);
#endif

 spi_master_init();
 ksp.ksp_interrupt_state = 0; //init state machine, stopped
 ksp.ksp_error = 0;

 CLEARBIT(SPCR, CPOL);

#ifndef SECU3T //---SECU-3i---
 ksp.pending_request = 0;
#endif //SECU-3i

 //set prescaler first
#ifdef SECU3T
 SET_KSP_CS(0);
#else //SECU3i
 IOCFG_SETF(IOP_KSP_CS, 0);
#endif
 spi_master_transmit(init_data[0]);
#ifdef SECU3T
 SET_KSP_CS(1);
#else //SECU3i
 IOCFG_SETF(IOP_KSP_CS, 1);
#endif

 //Setting SO terminal active and perform initialization. For each parameter perform
 //checking for response and correcntess of received data.
 for(i = 0; i < 2; ++i)
 {
#ifdef SECU3T
 SET_KSP_CS(0);
#else //SECU3i
 IOCFG_SETF(IOP_KSP_CS, 0);
#endif
  spi_master_transmit(init_data[i]);
#ifdef SECU3T
 SET_KSP_CS(1);
#else //SECU3i
 IOCFG_SETF(IOP_KSP_CS, 1);
#endif
  response = SPDR;
  if (response!=init_data[i])
  {
   _RESTORE_INTERRUPT(_t);
   return 0; //error - chip doesn't respond!
  }
 }

#ifdef TPIC8101
 //Finally, go into the advanced mode. In this mode we will be able to read int.output via SPI
#ifdef SECU3T
 SET_KSP_CS(0);
#else //SECU3i
 IOCFG_SETF(IOP_KSP_CS, 0);
#endif
 spi_master_transmit(KSP_SET_ADVANCED);
#ifdef SECU3T
 SET_KSP_CS(1);
#else //SECU3i
 IOCFG_SETF(IOP_KSP_CS, 1);
#endif
#endif

 _RESTORE_INTERRUPT(_t);
 //Initialization completed successfully
 return 1;
}

#if !defined(SECU3T) || defined(OBD_SUPPORT) //---SECU-3i---

uint8_t knock_expander_initialize()
{
 uint8_t _t;

 _t=_SAVE_INTERRUPT();
 _DISABLE_INTERRUPT();
 ksp.pending_request = 0;
 ksp.ksp_interrupt_state = 0; //init state machine

 spi_master_init();

 SETBIT(SPCR, CPOL);

//initialize MCP2515 if supported
#ifdef OBD_SUPPORT
 ksp.can_pending_tx = 0;  //no pending messages to transmit
 ksp.can_pending_rx = 0;  //no pending received messages 
 ksp.can_received = 0;    //no received messages
 ksp.can_rxb_priority = 0;//start reading from RXB0
 //put mcp2515 into the configuration mode and wait some time
 SET_CAN_CS(0);
 spi_master_transmit(SPI_RESET);
 SET_CAN_CS(1);
 _DELAY_US(20);
 //configure it
 SET_CAN_CS(0);
 spi_master_transmit(SPI_WRITE);
 spi_master_transmit(CNF3);
 spi_master_transmit(0x01);      // Bitrate 500 kbps at 8 MHz
 spi_master_transmit(0x91);      // CNF2
 spi_master_transmit(0x40);      // CNF1
 SET_CAN_CS(1);
 //set CAN filters and masks
 obd_init_filters();
 //reset MCP2515 to normal mode
 mcp2515_write_register(CANCTRL, 0);
#endif

//initialize port expander (SECU-3i only)
#ifndef SECU3T

 SET_KSP_TEST(0);
 _DELAY_US(2);
 SET_KSP_TEST(1);
 _DELAY_US(2);

 //configure port expander MCP23S17
#ifdef MCP3204
 ksp.spiadc_chidx = 0;
 //disable default sequential mode (set byte mode), BANK=0,
 SET_KSP_TEST(0);
 spi_master_transmit(0x40);        //write
 spi_master_transmit(0x0A);        //address = 0x0A
 spi_master_transmit(0x20);        //set SEQOP bit in the IOCON register
 SET_KSP_TEST(1);
 _DELAY_US(2);
#endif

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
#endif

 _RESTORE_INTERRUPT(_t);
 return 1; //successfully
}
#endif

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
 else if (ksp.ksp_interrupt_state < 6)
  ksp.ksp_error = 1; //previous latching is not finished yet
#ifndef SECU3T //---SECU-3i---
 else
  ksp.pending_request = 1; //if busy by MCP23S17, then just set request flag and exit
#endif
 _END_ATOMIC_BLOCK();
}

#if !defined(SECU3T) || defined(OBD_SUPPORT) //---SECU-3i---
void knock_start_expander_latching(void)
{
// _BEGIN_ATOMIC_BLOCK(); we rely that at the moment of calling of this function interrupts are disabled, so don't disable it twice
#ifdef SECU3T
  if (!ksp.can_pending_tx && !ksp.can_pending_rx)
   return; //no CAN messages waiting on sending or receiving, so, nothing to do in the SECU-3T
#endif
 if (0==ksp.ksp_interrupt_state)
 {
  SETBIT(SPCR, CPOL);
 _NO_OPERATION();  //todo: check maybe these 4 NOPs are not needed
 _NO_OPERATION();
 _NO_OPERATION();
 _NO_OPERATION();
#ifdef SECU3T
  SET_CAN_CS(0);
  ksp.ksp_interrupt_state = 19;
 _NO_OPERATION();
 _NO_OPERATION();
 _NO_OPERATION();
 _NO_OPERATION();
  SPDR = SPI_READ_STATUS;
#else //SECU-3i
  SET_KSP_TEST(0);
  ksp.ksp_interrupt_state = 6;
 _NO_OPERATION();
 _NO_OPERATION();
 _NO_OPERATION();
 _NO_OPERATION();
  SPDR = 0x40; //write
#endif
  //enable interrupt, sending of the remaining data will be completed in
  //interrupt's state machine
  SPCR|= _BV(SPIE);
 }
 else if (ksp.ksp_interrupt_state > 5)
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
#if !defined(SECU3T) || defined(OBD_SUPPORT) //---SECU-3i---
 if (ksp.ksp_interrupt_state < 6)
#endif
 { SET_KSP_CS(1); }

 //make chance for pending interrupts to be processed with less delay
 _ENABLE_INTERRUPT();
 _DISABLE_INTERRUPT();

 //interrupts are disabled now!
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
   SPDR = KSP_SET_PRESCALER | KSP_PRESCALER_VALUE | KSP_SO_TERMINAL_ACTIVE;
   break;

  case 3: //prescaler and SDO status loaded
   SET_KSP_CS(0);
   ++ksp.ksp_interrupt_state;
   SPDR = ksp.ksp_channel;
   break;

  case 4: //channel number loaded
#ifdef TPIC8101
   ksp.adc_low = SPDR; //save D7-D0 bits of ADC result
#endif
   SET_KSP_CS(0);
   ++ksp.ksp_interrupt_state;
   SPDR = ksp.ksp_inttime;
   break;

  case 5: //Int.Time loaded
#ifdef TPIC8101
   ksp.adc_value = (((uint16_t)SPDR) << 2) | ksp.adc_low; //save complete 10-bit value
#endif
#ifdef SECU3T
#ifdef OBD_SUPPORT
   if (ksp.pending_request)
   {//start loading data into the CAN chip
    SETBIT(SPCR, CPOL);
    SET_CAN_CS(0);
    ksp.pending_request = 0;
    if (ksp.can_pending_tx)
    {//pending TX
     ksp.ksp_interrupt_state = 19; // busy (state = 19)
     SPDR = SPI_READ_STATUS;
    }
    else
    {//pending RX
     ksp.ksp_interrupt_state = 30; // busy (state = 30)
     SPDR = SPI_RX_STATUS;
    }
   }
   else
#endif
   {
    ksp.ksp_interrupt_state = 0;   //idle    
    SPCR&= ~_BV(SPIE);             //disable interrupt and switch state machine into initial state - ready to new load
   }
   break;
#else //---SECU-3i---

   if (ksp.pending_request)
   {//start loading data into the expander chip
    SETBIT(SPCR, CPOL);
    SET_KSP_TEST(0);
    ksp.pending_request = 0;
    ++ksp.ksp_interrupt_state;     // busy (state = 6)
    SPDR = 0x40; //write opcode
   }
   else
   {
    ksp.ksp_interrupt_state = 0;  //idle    
    SPCR&= ~_BV(SPIE);            //disable interrupt and switch state machine into initial state - ready to new load
   }
   break;

//-----IO expander sequece-----
  case 6: //expander, opcode loaded
   SPDR = 0x13; //address of the GPIOB
   ++ksp.ksp_interrupt_state;
   break;
  case 7: //GPIOB address loaded
   SPDR = spi_PORTB;
#ifdef MCP3204
   ++ksp.ksp_interrupt_state;
   break;
  case 8: //GPIOB wrote
   SPDR = spi_PORTA & 0x7F; //set MCP3204 CS!
   ++ksp.ksp_interrupt_state;
   break;
  case 9: //GPIOA wrote!
   SET_KSP_TEST(1); //deselect the device
   ++ksp.ksp_interrupt_state;
   _NO_OPERATION();
   _NO_OPERATION();
   SPDR = (ksp.spiadc_chidx >> 2) | 0x6; //D2, single-ended
   break;
  case 10: //MCP3204 first byte wrote
   SPDR = ksp.spiadc_chidx << 6; //D1, D0
   ++ksp.ksp_interrupt_state;
   break;
  case 11: //MCP3204 second byte wrote
   ksp.spiadc_hibyte = SPDR;
   ++ksp.ksp_interrupt_state;
   SPDR = 0x00;
   break;
  case 12: //MCP3204 third byte wrote
   SET_KSP_TEST(0);
   spiadc_chan[ksp.spiadc_chidx] = (ksp.spiadc_hibyte << 8) | SPDR;
   ksp.spiadc_chidx = (ksp.spiadc_chidx + 1) & (SPIADC_CHNUM - 1);
   ++ksp.ksp_interrupt_state;
   SPDR = 0x40; //write opcode
   break;
  case 13: //expander, opcode loaded
   SPDR = 0x12; //address of the GPIOA
   ++ksp.ksp_interrupt_state;
   break;
  case 14: //GPIOA address loaded
   SPDR = spi_PORTA; //clear MCP3204 CS!
   ++ksp.ksp_interrupt_state;
   break;
#else
   ksp.ksp_interrupt_state = 15;
   break;
#endif
  case 15: //GPIOA wrote
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
  case 16: //expander, opcode loaded
   SPDR = 0x12; //address of the GPIOA
   ++ksp.ksp_interrupt_state;
   break;
  case 17: //GPIOA address loaded
   SPDR = 0x00;  //shift read register
   ++ksp.ksp_interrupt_state;
   break;
  case 18: //GPIOA read!
   spi_PORTA = SPDR;
   SET_KSP_TEST(1);
#ifdef OBD_SUPPORT
   if (ksp.can_pending_tx)
   { //start TX sequence
    SET_CAN_CS(0);
    _NO_OPERATION();
    _NO_OPERATION();
    ksp.ksp_interrupt_state = 19; // busy (state = 19)
    SPDR = SPI_READ_STATUS;
   }
   else if (ksp.can_pending_rx)
   { //start RX sequence
    SET_CAN_CS(0);
    _NO_OPERATION();
    _NO_OPERATION();
    ksp.ksp_interrupt_state = 30; // busy (state = 30)
    SPDR = SPI_RX_STATUS;
   }
   else
#endif
   {
    goto check_pending_kspbpf;
   }
   break;
#endif

//-----CAN transmitter sequence-----
#ifdef OBD_SUPPORT
   case 19:
    SPDR = 0xFF; //shift read register
    ++ksp.ksp_interrupt_state;
    break;
   case 20: //read status!
    SET_CAN_CS(1);
    if (!CHECKBIT(SPDR, 2))
     ksp.can_buff_addr = 0x00;
    else if (!CHECKBIT(SPDR, 4))
     ksp.can_buff_addr = 0x02;
    else if (!CHECKBIT(SPDR, 6))
     ksp.can_buff_addr = 0x04;
    else
     goto can_tx_finish; // All buffers are busy, message can't be sent. We will get a try next time    
   _NO_OPERATION();
   _NO_OPERATION();
    SET_CAN_CS(0);
   _NO_OPERATION();
   _NO_OPERATION();
   _NO_OPERATION();
   _NO_OPERATION();
    SPDR = SPI_WRITE_TX | ksp.can_buff_addr;
    ++ksp.ksp_interrupt_state;
    break;
   case 21:
    SPDR = (ksp.can_msg_tx.id >> 3);
    ++ksp.ksp_interrupt_state;
    break;
   case 22:
    SPDR = (ksp.can_msg_tx.id << 5);
    ++ksp.ksp_interrupt_state;
    break;
   case 23:
    SPDR = 0;
    ++ksp.ksp_interrupt_state;
    break;
   case 24:
    SPDR = 0;
    ++ksp.ksp_interrupt_state;
    break;
   case 25:
    if (ksp.can_msg_tx.flags.rtr)
    {
     SPDR = _BV(RTR) | ksp.can_msg_tx.length;
     ksp.ksp_interrupt_state = 27;
    }
    else
    {
     SPDR = ksp.can_msg_tx.length; //length must be  > 0!
     ksp.can_data_idx = 0;
     ++ksp.ksp_interrupt_state;
    }
    break;
   case 26: //length set!
    SPDR = ksp.can_msg_tx.data[ksp.can_data_idx++];
    if (ksp.can_data_idx >= ksp.can_msg_tx.length)
     ++ksp.ksp_interrupt_state;
    break;
   case 27:
    SET_CAN_CS(1);
   _NO_OPERATION();
   _NO_OPERATION();
   _NO_OPERATION();
   _NO_OPERATION();
    SET_CAN_CS(0);
    SPDR = SPI_RTS | ((ksp.can_buff_addr == 0) ? 1 : ksp.can_buff_addr);
    ++ksp.ksp_interrupt_state;
    break;
   case 28:
    SET_CAN_CS(1);
    ksp.can_pending_tx = 0;
can_tx_finish:
    if (ksp.can_pending_rx)
    {
    _NO_OPERATION();
    _NO_OPERATION();
     SET_CAN_CS(0);
    _NO_OPERATION();
    _NO_OPERATION();
     ksp.ksp_interrupt_state = 30; // busy (state = 30)
     SPDR = SPI_RX_STATUS;
    }
    else
    {
     goto check_pending_kspbpf;
    }
    break;

//-----CAN receiver sequence-----
   case 30:                        //RX_STATUS instruction sent
    SPDR = 0xFF;                   //shift read register
    ++ksp.ksp_interrupt_state;
    break;
   case 31:                        //RX STATUS read
    SET_CAN_CS(1);
    if (!(ksp.can_rxb_priority & 1))
    { //buffer 0 has priority
     if (CHECKBIT(SPDR, 6))
     { //message in buffer 0
      _NO_OPERATION();
      _NO_OPERATION();
      _NO_OPERATION();
       SET_CAN_CS(0);
      _NO_OPERATION();
      _NO_OPERATION();
      _NO_OPERATION();
      _NO_OPERATION();
      SPDR = SPI_READ_RX;           //READ RX BUFFER SPI instruction (from buffer 0)
     }
     else if (CHECKBIT(SPDR, 7))
     { //message in buffer 1
      _NO_OPERATION();
       SET_CAN_CS(0);
      _NO_OPERATION();
      _NO_OPERATION();
      _NO_OPERATION();
      _NO_OPERATION();
      SPDR = SPI_READ_RX | 0x04;    //READ RX BUFFER SPI instruction (from buffer 1)
     }
     else 
     {
      goto can_rx_finish;           //RX buffers are empty
     }
    }
    else
    { //buffer 1 has priority
     if (CHECKBIT(SPDR, 7))
     { //message in buffer 1
      _NO_OPERATION();
      _NO_OPERATION();
      _NO_OPERATION();
       SET_CAN_CS(0);
      _NO_OPERATION();
      _NO_OPERATION();
      _NO_OPERATION();
      _NO_OPERATION();
      SPDR = SPI_READ_RX | 0x04;    //READ RX BUFFER SPI instruction (from buffer 1)
     }
     else if (CHECKBIT(SPDR, 6))
     { //message in buffer 0
      _NO_OPERATION();
       SET_CAN_CS(0);
      _NO_OPERATION();
      _NO_OPERATION();
      _NO_OPERATION();
      _NO_OPERATION();
      SPDR = SPI_READ_RX;           //READ RX BUFFER SPI instruction (from buffer 0)
     }
     else 
     {
      goto can_rx_finish;           //RX buffers are empty
     }
    }
    ++ksp.can_rxb_priority;        //change priority to other buffer for the next time
    ++ksp.ksp_interrupt_state;
    break;
   case 32:                        //READ_RX instruction sent
    SPDR = 0xFF;                   //shift read register
    ++ksp.ksp_interrupt_state;
    break;
   case 33:                        //got RXBnSIDH
    ksp.can_msg_rx.id = ((uint16_t)SPDR) << 3; //store bits [10-3] of ID
    SPDR = 0xFF;                   //shift read register
    ++ksp.ksp_interrupt_state;
    break;
   case 34:                        //got RXBnSIDL
    ksp.can_msg_rx.id |= SPDR >> 5;            //store bits [2-0] of ID
    ksp.can_msg_rx.flags.rtr = CHECKBIT(SPDR, SRR) ? 1 : 0;  //store RTR bit, see RXBnSIDL register description for more info
    if (CHECKBIT(SPDR, IDE))
     goto can_rx_finish;           //ignore exteded frames, abort
    SPDR = 0xFF;                   //shift read register to skip RXBnEID8
    ++ksp.ksp_interrupt_state;
    break;
   case 35:                        //got RXBnEID8
    SPDR = 0xFF;                   //shift read register to skip RXBnEID0
    ++ksp.ksp_interrupt_state;
    break;
   case 36:                        //got RXBnEID0
    SPDR = 0xFF;                   //shift read register to read RXBnDLC
    ++ksp.ksp_interrupt_state;
    break;
   case 37:                        //got RXBnDLC
    ksp.can_msg_rx.length = SPDR & 0x0F;         //store length    
    ksp.can_data_idx = 0;
    SPDR = 0xFF;                   //shift read register to read the 1st byte of data
    ++ksp.ksp_interrupt_state;
    break;
   case 38:                        //got data byte
    ksp.can_msg_rx.data[ksp.can_data_idx++] = SPDR;
    if (ksp.can_data_idx < ksp.can_msg_rx.length)
    {
     SPDR = 0xFF;                  //shift read register to read the next byte
     break;                        //continue to read data
    }
    ksp.can_received = 1;          //success!
    //and fall into rx finish
can_rx_finish:
    SET_CAN_CS(1);                 //automatically clearing the associated receive flag, RXnIF (CANINTF), when CS is raised at the end of the command.
    ksp.can_pending_rx = 0;
#endif //OBD_SUPPORT

//-----Load KSP BPF sequence-----
#if defined(OBD_SUPPORT) || !defined(SECU3T)
check_pending_kspbpf:
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
     SPCR&= ~_BV(SPIE);           //disable interrupt and switch state machine into initial state - ready for new loading
    }
    break;
#endif //OBD_SUPPORT or SECU3i
 }
 _ENABLE_INTERRUPT();
}

void knock_init_ports(void)
{
#ifdef SECU3T
 PORTB|= _BV(PB4)|_BV(PB3); //interface with HIP9011 turned off (CS=1, TEST=1, MOSI=0, SCK=0)
 PORTC&=~_BV(PC4);
 DDRC |= _BV(DDC4);
#else //SECU3i
 IOCFG_INIT(IOP_KSP_CS, 1); //interface with HIP9011 turned off (CS=1, TEST=1, MOSI=0, SCK=0)
 SETBIT(PORTB, PB3);
 IOCFG_INIT(IOP_TACH_O, 0);
#endif
 DDRB |= _BV(DDB7)|_BV(DDB5)|_BV(DDB4)|_BV(DDB3);

#ifdef OBD_SUPPORT
 INIT_CAN_CS();
#endif
}

#ifdef OBD_SUPPORT
uint8_t knock_push_can_message(struct can_t* msg)
{
 if (ksp.can_pending_tx)
  return 0; //transmition of previous message is not finished yet
 memcpy(&ksp.can_msg_tx, msg, sizeof(can_t)); //copy message
 ksp.can_pending_tx = 1;  //will be cleared in the interrupt
 return 1; //success!
}

uint8_t knock_is_idle_can_tx(void)
{
 return !ksp.can_pending_tx;
}

void knock_check_can_message(void)
{
 if (ksp.can_pending_rx || ksp.can_received)
  return; //previous checking/getting is not finished yet
 ksp.can_pending_rx = 1;  //will be cleared in the interrupt
}

uint8_t knock_get_can_message(struct can_t* msg)
{
 if (ksp.can_received) //set in the interrupt after receiving message
 {
  memcpy(msg, &ksp.can_msg_rx, sizeof(can_t)); //copy message
  ksp.can_received = 0;
  return 1; //message exist and copied
 }
 return 0; //no received and pending messages
}

#endif

#ifdef TPIC8101
uint16_t knock_get_adc_value(void)
{
 _BEGIN_ATOMIC_BLOCK();
 uint16_t value = ksp.adc_value;
 _END_ATOMIC_BLOCK();
 return value;
}
#endif

#ifndef SECU3T
void knock_read_expander(void)
{
  //Note: we rely on that interrupts are disabled!
 SETBIT(SPCR, CPOL);             //set clock polarity required by expander chip
 SET_KSP_TEST(0);                //select chip
 spi_master_transmit(0x41);      //read opcode
 spi_master_transmit(0x12);      //address of the GPIOA
 spi_master_transmit(0x00);      //shift read register
 spi_PORTA = SPDR;               //save read value
 SET_KSP_TEST(1);                //deselect chip
}

void knock_write_expander(void)
{
 //Note: we rely on that interrupts are disabled!
 SETBIT(SPCR, CPOL);             //set clock polarity required by expander chip
 SET_KSP_TEST(0);                //select chip
 spi_master_transmit(0x40);      //write opcode
 spi_master_transmit(0x13);      //address of the GPIOB
 spi_master_transmit(spi_PORTB); //write new value into GPIOB
 SET_KSP_TEST(1);                //deselect chip
}

#endif
