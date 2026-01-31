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

/** \file camsens.c
 * \author Alexey A. Shabelnikov
 * Implementation of camshaft position sensor's processing (two types - Hall and VR) and referense sensor processing.
 * Also, this module contains processing of VSS.
 */

#include "port/avrio.h"
#include "port/interrupt.h"
#include "port/intrinsic.h"
#include "port/port.h"
#include "bitmask.h"
#include "camsens.h"
#include "ioconfig.h"
#include "tables.h"

#if defined(HALL_SYNC) || defined(CKPS_NPLUS1)
void ProcessInterrupt0(void);         //!< External ISR function, see hall.c for more information about this function
void ProcessInterrupt1(void);         //!< External ISR function, see hall.c for more information about this function
#endif

#ifdef SPEED_SENSOR
#define F_USEVSS 0                    //!< Flag, indicates that VSS is used (mapped to real I/O)
#endif

#ifdef PHASE_SENSOR
#define F_USECAM 1                    //!< Flag, indicates that cam sensor is used (mapped to real I/O)
#endif

#if defined(SPEED_SENSOR) || defined(PHASE_SENSOR)
#define flags TWSR                    //!< Only 2 bits are allowed to be used as R/W
#endif

//warning! Variable and bit number define must be same as in hall.c
#if defined(HALL_SYNC) || defined(CKPS_NPLUS1)
#define F_SELEDGE   2                 //!< indicates selected edge type, falling edge is default
#define flags2 TWBR                   //!< flags
#endif

/** Defines state variables */
typedef struct
{
 //Reference sensor stuff
 volatile uint8_t vr_event;           //!< flag which indicates event from reference sensor

 //Cam sensor only stuff
#ifdef PHASE_SENSOR
 volatile uint8_t cam_ok;             //!< indicates presence of cam sensor (works properly)
 uint8_t cam_error;                   //!< error flag, indicates error
 uint16_t cam_err_threshold;          //!< error threshold in teeth
 volatile uint16_t cam_err_counter;   //!< teeth counter
 volatile uint8_t cam_event;          //!< flag which indicates Hall cam sensor's event
 volatile uint8_t cam_enable;         //!< flag, indicates that cam sensor processing is enabled
#endif

 //VSS sensor only stuff
#ifdef SPEED_SENSOR
 uint16_t spdsens_period_prev;        //!< for storing previous value of timer counting time between speed sensor pulse interrupts
 volatile uint16_t spdsens_period;    //!< period between speed sensor pulses (1 tick  = 4us)
 uint16_t spdsens_period_buff;        //!< period between speed sensor pulses (buffered and everflow free)
 volatile uint16_t spdsens_counter;   //!< number of speed sensor's pulses since last ignition turn on and before next clearing
 volatile uint8_t spdsens_event;      //!< indicates pending event from speed sensor
 uint8_t spdsens_state;               //!< Used in special state machine to filter overflows
#endif

 volatile uint8_t ref_s_inpalt;       //!< shows which input is mapped to REF_S (0 - remapped to no ISR function, 1 - normal operation (not remapped), 2 - VSS, 3 - PS)
 volatile uint8_t ps_inpalt;          //!< shows which input is mapped to PS (0 - remapped to no ISR function, 1 - normal operation (not remapped), 2 - VSS)
}camstate_t;

/** Global instance of cam sensor state variables */
camstate_t camstate = {0,
#ifdef PHASE_SENSOR
 0,0,0,0,0,1,
#endif
#ifdef SPEED_SENSOR
 0,0xFFFF,0xFFFF,0,0,0,
#endif
 0,0
};

void cams_init_ports()
{
 IOCFG_INIT(IOP_REF_S, 1);   //use pullup resistor
 IOCFG_INIT(IOP_PS, 1);      //use pullup resistor
}

void cams_init_state_variables(void)
{
#ifdef PHASE_SENSOR
 camstate.cam_ok = 0; //not Ok
 camstate.cam_err_counter = 0;
 camstate.cam_event = 0;
#endif
#if !defined(HALL_SYNC) && !defined(CKPS_NPLUS1)
 camstate.vr_event = 0;
#endif
}

void cams_init_state(void)
{
 _BEGIN_ATOMIC_BLOCK();

 //Collect information about remapping of REF_S input.
 //Interrupt edge for VR input depends on REF_S inversion, but will be overriden by cams_vr_set_edge_type() if REF_S is not remapped.
 if (IOCFG_CMPN(IOP_REF_S, IOP_REF_S))
 {
  camstate.ref_s_inpalt = 1;                //REF_S not remapped, normal operation - reference sensor
  EICRA|= _BV(ISC01) | _BV(ISC00);
 }
 else if (IOCFG_CMPI(IOP_REF_S, IOP_REF_S))
 {
  camstate.ref_s_inpalt = 1;                //REF_S not remapped, normal operation - reference sensor
  EICRA|= _BV(ISC01) | 0;                   //inversion
 }
#ifdef SPEED_SENSOR
 else if (IOCFG_CMPN(IOP_REF_S, IOP_SPDSENS))
 {
  camstate.ref_s_inpalt = 2;                //REF_S remapped to VSS
  EICRA|= _BV(ISC01) | _BV(ISC00);
 }
 else if (IOCFG_CMPI(IOP_REF_S, IOP_SPDSENS))
 {
  camstate.ref_s_inpalt = 2;                //REF_S remapped to VSS
  EICRA|= _BV(ISC01) | 0;                   //inversion
 }
#endif
#ifdef PHASE_SENSOR
 else if (IOCFG_CMPN(IOP_REF_S, IOP_PS))
 {
  camstate.ref_s_inpalt = 3;                //REF_S remapped to PS
  EICRA|= _BV(ISC01) | _BV(ISC00);
 }
 else if (IOCFG_CMPI(IOP_REF_S, IOP_PS))
 {
  camstate.ref_s_inpalt = 3;                //REF_S remapped to PS
  EICRA|= _BV(ISC01) | 0;                   //inversion
 }
#endif
#if defined(HALL_SYNC) || defined(CKPS_NPLUS1)
 //remapping REF_S to CKPS takes sense only with HALL_SYNC or CKPS_NPLUS1
 else if (IOCFG_CMPN(IOP_REF_S, IOP_CKPS))
 {
  camstate.ref_s_inpalt = 4;                //REF_S remapped to CKP sensor
  EICRA|= _BV(ISC01) | _BV(ISC00);
  WRITEBIT(flags2, F_SELEDGE, 1);           //save selected edge type
 }
 else if (IOCFG_CMPI(IOP_REF_S, IOP_CKPS))
 {
  camstate.ref_s_inpalt = 4;                //REF_S remapped to CKP sensor
  EICRA|= _BV(ISC01) | 0;                   //inversion
  WRITEBIT(flags2, F_SELEDGE, 0);           //save selected edge type
 }
#endif
 else
  camstate.ref_s_inpalt = 0;                //REF_S remapped to other input, no ISR

 //Collect information about remapping of the PS input
 //Interrupt edge for Hall input depends only on PS inversion
 if (IOCFG_CMPN(IOP_PS, IOP_PS))
 {
  camstate.ps_inpalt = 1;                   //PS not remapped, normal operation - phase sensor
  EICRA|= _BV(ISC11) | _BV(ISC10);
 }
 else if (IOCFG_CMPI(IOP_PS, IOP_PS))
 {
  camstate.ps_inpalt = 1;                   //PS not remapped, normal operation - phase sensor
  EICRA|= _BV(ISC11) | 0;                   //inverted
 }
#ifdef SPEED_SENSOR
 else if (IOCFG_CMPN(IOP_PS, IOP_SPDSENS))
 {
  camstate.ps_inpalt = 2;                   //PS remapped to VSS
  EICRA|= _BV(ISC11) | _BV(ISC10);
 }
 else if (IOCFG_CMPI(IOP_PS, IOP_SPDSENS))
 {
  camstate.ps_inpalt = 2;                   //PS remapped to VSS
  EICRA|= _BV(ISC11) | 0;                   //inverted
 }
#endif
#if defined(HALL_SYNC) || defined(CKPS_NPLUS1)
 //remapping PS to CKPS takes sense only with HALL_SYNC or CKPS_NPLUS1
 else if (IOCFG_CMPN(IOP_PS, IOP_CKPS))
 {
  camstate.ps_inpalt = 3;                   //PS remapped to CKP sensor
  EICRA|= _BV(ISC11) | _BV(ISC10);
  WRITEBIT(flags2, F_SELEDGE, 1);           //save selected edge type
 }
 else if (IOCFG_CMPI(IOP_PS, IOP_CKPS))
 {
  camstate.ps_inpalt = 3;                   //PS remapped to CKP sensor
  EICRA|= _BV(ISC11) | 0;                   //inverted
  WRITEBIT(flags2, F_SELEDGE, 0);           //save selected edge type
 }
#endif
 else
  camstate.ps_inpalt = 0;                   //PS remapped to other input, no ISR
 ///////////////////////////////////////////////////////////////////////////////////

 //Helpful flag variables, which will speed up code execution
#ifdef SPEED_SENSOR
 WRITEBIT(flags, F_USEVSS, (camstate.ref_s_inpalt == 2) || (camstate.ps_inpalt == 2));
#endif
#ifdef PHASE_SENSOR
 WRITEBIT(flags, F_USECAM, (camstate.ref_s_inpalt == 3) || (camstate.ps_inpalt == 1));
#endif

#if defined(PHASE_SENSOR) || defined(SPEED_SENSOR) || defined(HALL_SYNC) || defined(CKPS_NPLUS1)
  //INT1 enabled only when cam or VSS sensor is utilized in the firmware or HALL_SYNC option used AND if
  //PS mapped to PS, VSS or CKPS, for other cases ISR is not required.
  //So, if firmware compiled without PHASE_SENSOR, SPEED_SENSOR and HALL_SYNC options, then we don't
  //require INT1 interrupt at all.
 if (0!=camstate.ps_inpalt)
  EIMSK|= _BV(INT1);
#endif

 //INT0 doesn't depend on PHASE_SENSOR and SPEED_SENSOR compilation options
 if (0!=camstate.ref_s_inpalt)
  EIMSK|= _BV(INT0);              //INT0 enabled only when REF_S input mapped to input which requires ISR (default REF_S requires it)

 _END_ATOMIC_BLOCK();
}

void cams_control(void)
{
#ifdef SPEED_SENSOR
 uint16_t t1_curr, t1_prev, period;
 uint8_t _t, event;
 if (!CHECKBIT(flags, F_USEVSS))
  return;                                //speed sensor is not enabled
 _t = _SAVE_INTERRUPT();
 _DISABLE_INTERRUPT();
 t1_curr = TCNT1;                        //current value of timer
 t1_prev = camstate.spdsens_period_prev; //value of timer remembered in ISR
 _RESTORE_INTERRUPT(_t);                 //reenable interrupts
 _DISABLE_INTERRUPT();
 period = camstate.spdsens_period;       //read and reset flag
 event = camstate.spdsens_event;         //
 camstate.spdsens_event = 0;
 _RESTORE_INTERRUPT(_t);

 switch(camstate.spdsens_state)
 {
  case 0:
   if (event)
    ++camstate.spdsens_state;            //skip first "dirty" event
   break;
  case 1:
   if ((t1_curr - t1_prev) > 62000)
   { //overflow
    camstate.spdsens_period_buff = 0xFFFF;
    camstate.spdsens_state = 0;
   }
   else if (event)
    camstate.spdsens_period_buff = period;
   break;
 }
#endif
}

#if !defined(HALL_SYNC) && !defined(CKPS_NPLUS1)
uint8_t cams_vr_is_event_r(void)
{
 uint8_t result;
 _BEGIN_ATOMIC_BLOCK();
 result = camstate.vr_event;
 camstate.vr_event = 0; //reset event flag
 _END_ATOMIC_BLOCK();
 return result;
}
#endif

#ifdef PHASE_SENSOR
/**Must be called from ISR to process cam sensor*/
#define PROCESS_CAM() {\
  camstate.cam_ok = 1;\
  camstate.cam_err_counter = 0;\
  camstate.cam_event = 1;\
}
#endif

#ifdef SPEED_SENSOR
/**Must be called from ISR to process VSS*/
#define PROCESS_VSS() {\
  ++camstate.spdsens_counter;\
  camstate.spdsens_period = TCNT1 - camstate.spdsens_period_prev;\
  camstate.spdsens_period_prev = TCNT1;\
  camstate.spdsens_event = 1;\
}
#endif

/**Interrupt from the VR sensor. Marked as REF_S on the schematics */
ISR(INT0_vect)
{
#if defined(SPEED_SENSOR) || defined(PHASE_SENSOR) || defined(HALL_SYNC) || defined(CKPS_NPLUS1)
 //In this case this ISR can be used either for: vehicle speed sensor, cam/Hall sensor or reference sensor
 //
 if (1==camstate.ref_s_inpalt)      //reference sensor (normal INT0 operation)
 {
#if !defined(HALL_SYNC) && !defined(CKPS_NPLUS1) //not used by Hall and N+1
  camstate.vr_event = 1;            //set event flag
#endif
 }

//note: PHASE_SENSOR and HALL_SYNC are options which can't be used together (for now)
#ifdef PHASE_SENSOR
 else if (3==camstate.ref_s_inpalt) //INT0 used for cam sensor
  PROCESS_CAM()
#endif
#if defined(HALL_SYNC) || defined(CKPS_NPLUS1)
 else if (4==camstate.ref_s_inpalt) //INT0 used for synchronization from Hall sensor
  ProcessInterrupt0();              //call ISR if CKPS mapped to REF_S
#endif
#ifdef SPEED_SENSOR
 else if (2==camstate.ref_s_inpalt) //INT0 used for VSS
  PROCESS_VSS()
#endif

#else
 //Simple case.
 //In this case this ISR used only for reference sensor (normal INT0 operation)
 //
 camstate.vr_event = 1; //set event flag
#endif
}

void cams_vr_set_edge_type(uint8_t edge_type)
{
#if defined(HALL_SYNC) || defined(CKPS_NPLUS1)
 if (camstate.ref_s_inpalt == 4) {
  WRITEBIT(flags2, F_SELEDGE, edge_type); //save selected edge type for hall.c
 }
#endif

 _BEGIN_ATOMIC_BLOCK();
 if (edge_type)
  EICRA|= _BV(ISC00); //rising
 else
  EICRA&= ~_BV(ISC00);//falling
 _END_ATOMIC_BLOCK();
}

//Functionality added to compilation only when PHASE_SENSOR defined
#ifdef PHASE_SENSOR
void cams_set_error_threshold(uint16_t threshold)
{
 camstate.cam_err_threshold = threshold;
}

void cams_detect_edge(void)
{
 if (!CHECKBIT(flags, F_USECAM) || !camstate.cam_enable)
  return; //do nothing if cam sensor is not mapped to PS or remapped to REF_S or not used currently

 if (++camstate.cam_err_counter > camstate.cam_err_threshold)
 {
  camstate.cam_ok = 0;
  camstate.cam_error = 1;
  camstate.cam_err_counter = 0;
 }
}

uint8_t cams_is_ready(void)
{
 return camstate.cam_ok;
}

uint8_t cams_is_error(void)
{
 return camstate.cam_error;
}

void cams_reset_error(void)
{
 camstate.cam_error = 0;
}

uint8_t cams_is_event_r(void)
{
 uint8_t result;
 _BEGIN_ATOMIC_BLOCK();
 result = camstate.cam_event;
 camstate.cam_event = 0; //reset event flag
 _END_ATOMIC_BLOCK();
 return result;
}

void cams_enable_cam(uint8_t enable)
{
 camstate.cam_enable = enable;
 if (camstate.ref_s_inpalt == 3) //cam sensor on the REF_S
 {
  WRITEBIT(EIMSK, INT0, enable);
 }
 else if ((camstate.ps_inpalt == 1)) //cam sensor on the PS
 {
  WRITEBIT(EIMSK, INT1, enable);
 }
}
#endif //PHASE_SENSOR

//We need following ISR if cam sensor or speed sensor is selected for compilation
#if defined(PHASE_SENSOR) || defined(SPEED_SENSOR) || defined(HALL_SYNC) || defined(CKPS_NPLUS1)

/**Interrupt from CAM sensor (Hall)*/
ISR(INT1_vect)
{
#if defined(HALL_SYNC) || defined(CKPS_NPLUS1) //Synchronization from hall sensor
 if (3==camstate.ps_inpalt)
  ProcessInterrupt1();         //call INT1 handler if PS input is remapped to CKPS (Hall sensor is connected to PS)
#endif
#ifdef SPEED_SENSOR
 if (2==camstate.ps_inpalt)
  PROCESS_VSS();               //PS remapped to VSS, you should use CKPS input for Hall sensor instead of PS
#endif
#ifdef PHASE_SENSOR
 if (1==camstate.ps_inpalt)
  PROCESS_CAM()                //PS is not remapped to other function, so use it for cam sensor
#endif
}

#endif //defined(PHASE_SENSOR) || defined(SPEED_SENSOR) || defined(HALL_SYNC) || defined(CKPS_NPLUS1)

#ifdef SPEED_SENSOR
uint16_t spdsens_get_period(void)
{
 return CHECKBIT(flags, F_USEVSS) ? camstate.spdsens_period_buff : 0;
}

uint16_t spdsens_get_pulse_count(uint8_t reset)
{
 uint16_t value;
 if (reset)
 {
  _BEGIN_ATOMIC_BLOCK();
  value = camstate.spdsens_counter;
  camstate.spdsens_counter = 0; //also reset counter
  _END_ATOMIC_BLOCK();
 }
 else
 {
  _BEGIN_ATOMIC_BLOCK();
  value = camstate.spdsens_counter; //just get value of counter
  _END_ATOMIC_BLOCK();
 }
 return CHECKBIT(flags, F_USEVSS) ? value : 0;
}
#endif
