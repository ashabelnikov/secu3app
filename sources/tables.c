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

/** \file tables.c
 * \author Alexey A. Shabelnikov
 * Tables and datastructures stored in the frmware (instances).
 */

#include "port/port.h"
#include "adc.h"
#include "bitmask.h"
#include "compilopt.h"
#include "ioconfig.h"
#include "magnitude.h"
#include "tables.h"
#include "uart.h"

/**Helpful macro used for pointer conversion */
#define _FNC(a) ((fnptr_t)(a))

/**Helpful macro for creating of I/O remapping version number*/
#define IOREMVER(maj, min) ((maj)<<4 | ((min) & 0xf))

/**For specifying of dwell times in the lookup table*/
#define _DLV(v) ROUND(((v)*312.5))

/**For specifying of choke position in the lookup table*/
#define _CLV(v) ROUND(((v)*2.0))

/**For specifying of temperature values in the lookup table*/
#define _TLV(v) ROUND(((v)*4.0))

/**ADC compensation factor, see magnitude.h for more information*/
#define _ACF ADC_COMP_FACTOR(ADC_VREF_FACTOR)
#define _ACF1 ADC_COMP_FACTOR(ADC_MCP3204_FACTOR)

/**ADC compensation correction, see magnitude.h for more information*/
#define _ACC ADC_COMP_CORR(ADC_VREF_FACTOR, 0.0)
#define _ACC1 ADC_COMP_CORR(ADC_MCP3204_FACTOR, 0.0)

/**For specifying values in the warmup enrichment lookup table*/
#define _WLV(v) ROUND((((v)/100.0)*128.0))

/**For specifying values in VE table*/
#define _VE(v) ROUND(((v)*2048))

/**For specifying values in AFR table*/
#define _FR(v) ROUND((((v)-8.0)*16))

/**For specifying values in EGO curve table*/
#define _ER(v) ROUND((v)*128.0)

/**For setting cylinder displacement value*/
#define CYL_DISP(v) ROUND(((v)*16384.0))

/**For setting injector flow rate value*/
#define INJ_FLRT(v) ROUND(((v)*64.0))

/**For setting system time values, v in seconds*/
#define SYS_TIME_S(v) ROUND(((v)*100.0))
/**Same as SYS_TIME_S, but multiplied only by 10*/
#define SYS_TIMEX10_S(v) ROUND((v)*10)

/**For setting lambda correction values*/
#define EGO_CORR(v) ROUND((((v)/100.0)*512.0))

/**For filling afterstart enrichment lookup table */
#define _ASE(v) ROUND((((v)/100.0)*128.0))

/**For filling bins of the AE's TPS lookup table*/
#define AE_TPS_B(v) ROUND((v) / 10.0)
/**For filling values of the AE's TPS lookup table*/
#define AE_TPS_V(v) ROUND((((v)/100.0)*100.0)+55.0)
/**For filling bins of the AE's RPM lookup table*/
#define AE_RPM_B(v) ROUND((v) / 100.0)
/**For filling values of the AE's RPM lookup table*/
#define AE_RPM_V(v) ROUND(((v)/100.0)*128.0)

/**For setting AE cold accel. multiplier value*/
#define AE_CAM(v) ROUND(((v)-1.0)*128.0)

/**Cooling fan PWM frequency*/
#define _FAN_PWMFRQ(v) ROUND((1.0/(v))*524288.0)

//For encoding of gas dose actuator position value
#define _GD(v) GD_MAGNITUDE(v)

//For encoding of injection timing map values
#define _IT(v) ROUND((v) * 2.0)

//For indling target RPM
#define _IR(v) ROUND((v) / 10.0)

//For indling regulator's rigidity function
#define _IRR(v) ROUND(((v) * 128.0))

//IAC correction weight
#define _MW(v) ROUND((v) * 256.0)
#define _MWX(v) ROUND((v) * 2.0)
//IAC correction
#define _MC(v) ROUND(((v)-1.0) * 8192.0)
#define _MCX(v) ROUND((v) * 128.0)

//IAT/CLT correction
#define _IC(v) ROUND((v) * 8192.0)
#define _ICX(v) ROUND((v) / 32.0)

//For packing values into 12-bit cells
#define _PACK2(v1,v2) ((uint8_t)v1), ((uint8_t)(((v1)>>8) | ((v2)<<4))), ((uint8_t)((v2)>>4))
#define _PACK16(v01,v02,v03,v04,v05,v06,v07,v08,v09,v10,v11,v12,v13,v14,v15,v16) \
 _PACK2(v01,v02),_PACK2(v03,v04),_PACK2(v05,v06),_PACK2(v07,v08),_PACK2(v09,v10),_PACK2(v11,v12),_PACK2(v13,v14),_PACK2(v15,v16)

/**wrapper macro for filling arrays*/
#define _AL16(v0,v1,v2,v3,v4,v5,v6,v7,v8,v9,v10,v11,v12,v13,v14,v15) v0,v1,v2,v3,v4,v5,v6,v7,v8,v9,v10,v11,v12,v13,v14,v15

/**Reverse order of items (for filling arrays)*/
#define _REVARR16(l0,l1,l2,l3,l4,l5,l6,l7,l8,l9,l10,l11,l12,l13,l14,l15) l15,l14,l13,l12,l11,l10,l9,l8,l7,l6,l5,l4,l3,l2,l1,l0

//Barometric correction
#define _BC(v) ROUND((v) * 4096.0)
#define _BCX(v) ROUND((v) * 64)

//Manual ign.timing correction
#define _PA4LV(v) ROUND((v) * 2.0)

//For specifying of throttle position
#define _TP(v) ROUND((v) * 2.0)

/***/
#define _GPSX(v) ROUND((v)/2)

/**Helpful macro for filling of lookup table*/
#define _HPV(v) SYSTIM_MAGS(v)

/***/
#define _CRT(v) ((v) / 10)

/***/
#define _CTM(v) (v)

/***/
#define _SMA(v) ((v) / 10)

/***/
#define _CLT(v) TEMPERATURE_MAGNITUDE(v)

/***/
#define _PW(d) ROUND(((d)*255.0)/100.0)

/**for filling fuel tank's level map*/
#define _FTL(v) ROUND((v) * 64.0)

/**for filling exhaust gas's temperature map*/
#define _EGT(v) ROUND((v) * 4.0)

/**for filling oil pressure's map*/
#define _OIP(v) ROUND((v) * 256.0)

/**Used to encode values in the knock zones look up table*/
#define _KNR(b15, b14, b13, b12, b11, b10, b9, b8, b7, b6, b5, b4, b3, b2 ,b1, b0) (_CBV16(b15, 0) | _CBV16(b14, 1) | _CBV16(b13, 2) | _CBV16(b12, 3) | _CBV16(b11, 4) | _CBV16(b10, 5) | _CBV16(b9, 6) | _CBV16(b8, 7) | _CBV16(b7, 8) | _CBV16(b6, 9) | _CBV16(b5, 10) | _CBV16(b4, 11) | _CBV16(b3, 12) | _CBV16(b2, 13) | _CBV16(b1, 14) | _CBV16(b0, 15))

/**For encoding of PWM IAC voltage coefficients */
#define _UF(v) ROUND((v)*4096.0)

/**For specifying load values*/
#define _LM(v) PRESSURE_MAGNITUDE(v)

/**For specifying inj. PW coeff. values*/
#define _IPC(v) ROUND((v)*4096.0)

/**For filling of MAF's flow curve*/
#define _MAF(v) ROUND((v)*64.0)

/**Inj. multiplier (0.5...1.49) */
#define _IML(v) ROUND(((v) - 0.5)*256.0)

/**Inj. addition (ms) in 25.6 us steps*/
#define _IAD(v) ROUND((v)/0.0256)

/**Fuel density correction */
#define _FDC(v) ROUNDU16((v)*16384.0)

/**Fill whole firmware data */
PGM_FIXED_ADDR_OBJ(fw_data_t fw_data, ".firmware_data") =
{
 /**Fill data residing in code area*/
 {
  //Add new data here

  /**I/O remapping. Match slots and plugs for default configuration*/
  {

#ifdef SECU3T
   //normal slots (initialization)
   {_FNC(iocfg_i_ign_out1), _FNC(iocfg_i_ign_out2), _FNC(iocfg_i_ign_out3), _FNC(iocfg_i_ign_out4),
    _FNC(iocfg_i_add_o1), _FNC(iocfg_i_add_o2), _FNC(iocfg_i_ecf), _FNC(iocfg_i_st_block),
    _FNC(iocfg_i_ie), _FNC(iocfg_i_fe),
    _FNC(iocfg_i_ps),     //PS input initialization
    _FNC(iocfg_i_add_i1), //ADD_I1 input initialization
    _FNC(iocfg_i_add_i2), //ADD_I2 input initialization
    _FNC(iocfg_i_ce), _FNC(iocfg_i_bl), _FNC(iocfg_i_de),
    _FNC(iocfg_i_gas_v), _FNC(iocfg_i_ref_s), _FNC(iocfg_i_ckps),
    _FNC(iocfg_i_map_s),0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 //<-- zero means that these slots are not implemented in this firmware
   },//inverted slots (initialization)
   {_FNC(iocfg_i_ign_out1i), _FNC(iocfg_i_ign_out2i), _FNC(iocfg_i_ign_out3i), _FNC(iocfg_i_ign_out4i),
    _FNC(iocfg_i_add_o1i), _FNC(iocfg_i_add_o2i), _FNC(iocfg_i_ecfi), _FNC(iocfg_i_st_blocki),
    _FNC(iocfg_i_iei), _FNC(iocfg_i_fei),
    _FNC(iocfg_i_psi),              //PS input initialization
    _FNC(iocfg_i_add_i1i), //ADD_I1 input initialization
    _FNC(iocfg_i_add_i2i), //ADD_I2 input initialization
    _FNC(iocfg_i_cei), _FNC(iocfg_i_bli), _FNC(iocfg_i_dei),
    _FNC(iocfg_i_gas_vi), _FNC(iocfg_i_ref_si), _FNC(iocfg_i_ckpsi),
    _FNC(iocfg_i_map_si),0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 //<-- zero means that these slots are not implemented in this firmware
   },//normal slots (get/set value)
   {_FNC(iocfg_s_ign_out1), _FNC(iocfg_s_ign_out2), _FNC(iocfg_s_ign_out3), _FNC(iocfg_s_ign_out4),
    _FNC(iocfg_s_add_o1), _FNC(iocfg_s_add_o2), _FNC(iocfg_s_ecf), _FNC(iocfg_s_st_block),
    _FNC(iocfg_s_ie), _FNC(iocfg_s_fe),
    _FNC(iocfg_g_ps),              //PS input get value
    _FNC(iocfg_g_add_i1), //ADD_I1 input get value
    _FNC(iocfg_g_add_i2), //ADD_I2 input get value
    _FNC(iocfg_s_ce), _FNC(iocfg_s_bl), _FNC(iocfg_s_de),
    _FNC(iocfg_g_gas_v), _FNC(iocfg_g_ref_s), _FNC(iocfg_g_ckps),
    _FNC(iocfg_g_map_s),0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 //<-- zero means that these slots are not implemented in this firmware
   },//inverted slots (get/set value)
   {_FNC(iocfg_s_ign_out1i), _FNC(iocfg_s_ign_out2i), _FNC(iocfg_s_ign_out3i), _FNC(iocfg_s_ign_out4i),
    _FNC(iocfg_s_add_o1i), _FNC(iocfg_s_add_o2i), _FNC(iocfg_s_ecfi), _FNC(iocfg_s_st_blocki),
    _FNC(iocfg_s_iei), _FNC(iocfg_s_fei),
    _FNC(iocfg_g_psi),              //PS input get value
    _FNC(iocfg_g_add_i1i), //ADD_I1 input get value
    _FNC(iocfg_g_add_i2i), //ADD_I2 input get value
    _FNC(iocfg_s_cei), _FNC(iocfg_s_bli), _FNC(iocfg_s_dei),
    _FNC(iocfg_g_gas_vi), _FNC(iocfg_g_ref_si), _FNC(iocfg_g_ckpsi),
    _FNC(iocfg_g_map_si),0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 //<-- zero means that these slots are not implemented in this firmware
   },
   //plugs
   {_FNC(iocfg_i_ign_out1), _FNC(iocfg_i_ign_out2), _FNC(iocfg_i_ign_out3), _FNC(iocfg_i_ign_out4),
    _FNC(iocfg_i_add_o1), _FNC(iocfg_i_add_o2), _FNC(iocfg_i_ecf), _FNC(iocfg_i_st_block),
    _FNC(iocfg_i_ie), _FNC(iocfg_i_fe), _FNC(iocfg_i_ps), _FNC(iocfg_i_add_i1),
    _FNC(iocfg_i_add_i2), _FNC(iocfg_i_ce), _FNC(iocfg_i_bl), _FNC(iocfg_i_de),
    _FNC(iocfg_i_gas_v), _FNC(iocfg_i_ref_s), _FNC(iocfg_i_ckps), _FNC(iocfg_i_map_s), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, //<-- mapped to slots by default
    _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub),
    _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub),
    _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub),
    _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub),
    _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub),
    _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub),
    _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub),
    _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub),
    _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub),
    _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub),
    _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub),
    _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub),
    _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub)
   },
   {_FNC(iocfg_s_ign_out1), _FNC(iocfg_s_ign_out2), _FNC(iocfg_s_ign_out3), _FNC(iocfg_s_ign_out4),
    _FNC(iocfg_s_add_o1), _FNC(iocfg_s_add_o2), _FNC(iocfg_s_ecf), _FNC(iocfg_s_st_block),
    _FNC(iocfg_s_ie), _FNC(iocfg_s_fe), _FNC(iocfg_g_ps), _FNC(iocfg_g_add_i1),
    _FNC(iocfg_g_add_i2), _FNC(iocfg_s_ce), _FNC(iocfg_s_bl), _FNC(iocfg_s_de),
    _FNC(iocfg_g_gas_v), _FNC(iocfg_g_ref_s), _FNC(iocfg_g_ckps), _FNC(iocfg_g_map_s), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, //<-- mapped to slots by default
    _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub),
    _FNC(iocfg_g_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_g_stub),
    _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_g_stub), _FNC(iocfg_g_stub),
    _FNC(iocfg_s_stub), _FNC(iocfg_g_stub), _FNC(iocfg_g_stub), _FNC(iocfg_s_stub),
    _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub),
    _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub),
    _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub),
    _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_g_stub),
    _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub),
    _FNC(iocfg_g_stub), _FNC(iocfg_s_stub), _FNC(iocfg_g_stub), _FNC(iocfg_g_stub),
    _FNC(iocfg_g_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub),
    _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub),
    _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub),
   },

#else //---SECU-3i---

   //normal slots (initialization)
   {_FNC(iocfg_i_ign_out1), _FNC(iocfg_i_ign_out2), _FNC(iocfg_i_ign_out3), _FNC(iocfg_i_ign_out4),
    _FNC(iocfg_i_ign_out5), _FNC(iocfg_i_ecf), _FNC(iocfg_i_inj_out1), _FNC(iocfg_i_inj_out2),
    _FNC(iocfg_i_inj_out3), _FNC(iocfg_i_inj_out4), _FNC(iocfg_i_inj_out5),_FNC(iocfg_i_bl),
    _FNC(iocfg_i_de),_FNC(iocfg_i_st_block), _FNC(iocfg_i_ce), _FNC(iocfg_i_fpmp_o),
    _FNC(iocfg_i_pwrr_o), _FNC(iocfg_i_evap_o), _FNC(iocfg_i_o2sh_o), _FNC(iocfg_i_cond_o),
    _FNC(iocfg_i_add_o2), _FNC(iocfg_i_ps), _FNC(iocfg_i_ref_s), _FNC(iocfg_i_ckps),
    _FNC(iocfg_i_add_i1), _FNC(iocfg_i_add_i2), _FNC(iocfg_i_add_i3), _FNC(iocfg_i_gas_v),
    _FNC(iocfg_i_ign), _FNC(iocfg_i_cond_i), _FNC(iocfg_i_epas_i),_FNC(iocfg_i_add_i4),
    _FNC(iocfg_i_tach_o),_FNC(iocfg_i_ksp_cs),_FNC(iocfg_i_add_i5),_FNC(iocfg_i_add_i6),
    _FNC(iocfg_i_add_i7), _FNC(iocfg_i_add_i8),_FNC(iocfg_i_map_s),_FNC(iocfg_i_oilp_i), _FNC(iocfg_i_gens_i),0,0,0,0,0,0,0,0 //<-- zero means that these slots are not implemented in this firmware
   },//inverted slots (initialization)
   {_FNC(iocfg_i_ign_out1i), _FNC(iocfg_i_ign_out2i), _FNC(iocfg_i_ign_out3i), _FNC(iocfg_i_ign_out4i),
    _FNC(iocfg_i_ign_out5i), _FNC(iocfg_i_ecfi), _FNC(iocfg_i_inj_out1i), _FNC(iocfg_i_inj_out2i),
    _FNC(iocfg_i_inj_out3i), _FNC(iocfg_i_inj_out4i), _FNC(iocfg_i_inj_out5i),_FNC(iocfg_i_bli),
    _FNC(iocfg_i_dei),_FNC(iocfg_i_st_blocki), _FNC(iocfg_i_cei), _FNC(iocfg_i_fpmp_oi),
    _FNC(iocfg_i_pwrr_oi), _FNC(iocfg_i_evap_oi), _FNC(iocfg_i_o2sh_oi), _FNC(iocfg_i_cond_oi),
    _FNC(iocfg_i_add_o2i), _FNC(iocfg_i_psi), _FNC(iocfg_i_ref_si), _FNC(iocfg_i_ckpsi),
    _FNC(iocfg_i_add_i1i), _FNC(iocfg_i_add_i2i), _FNC(iocfg_i_add_i3i), _FNC(iocfg_i_gas_vi),
    _FNC(iocfg_i_igni), _FNC(iocfg_i_cond_ii), _FNC(iocfg_i_epas_ii),_FNC(iocfg_i_add_i4i),
    _FNC(iocfg_i_tach_oi), _FNC(iocfg_i_ksp_csi), _FNC(iocfg_i_add_i5i), _FNC(iocfg_i_add_i6i),
    _FNC(iocfg_i_add_i7i), _FNC(iocfg_i_add_i8i),_FNC(iocfg_i_map_si),_FNC(iocfg_i_oilp_ii), _FNC(iocfg_i_gens_ii),0,0,0,0,0,0,0,0 //<-- zero means that these slots are not implemented in this firmware
   },//normal slots (get/set value)
   {_FNC(iocfg_s_ign_out1), _FNC(iocfg_s_ign_out2), _FNC(iocfg_s_ign_out3), _FNC(iocfg_s_ign_out4),
    _FNC(iocfg_s_ign_out5), _FNC(iocfg_s_ecf), _FNC(iocfg_i_inj_out1), _FNC(iocfg_s_inj_out2),
    _FNC(iocfg_s_inj_out3), _FNC(iocfg_s_inj_out4), _FNC(iocfg_s_inj_out5),_FNC(iocfg_s_bl),
    _FNC(iocfg_s_de),_FNC(iocfg_s_st_block), _FNC(iocfg_s_ce), _FNC(iocfg_s_fpmp_o),
    _FNC(iocfg_s_pwrr_o), _FNC(iocfg_s_evap_o), _FNC(iocfg_s_o2sh_o), _FNC(iocfg_s_cond_o),
    _FNC(iocfg_s_add_o2), _FNC(iocfg_g_ps), _FNC(iocfg_g_ref_s), _FNC(iocfg_g_ckps),
    _FNC(iocfg_g_add_i1), _FNC(iocfg_g_add_i2), _FNC(iocfg_g_add_i3), _FNC(iocfg_g_gas_v),
    _FNC(iocfg_g_ign), _FNC(iocfg_g_cond_i), _FNC(iocfg_g_epas_i), _FNC(iocfg_g_add_i4),
    _FNC(iocfg_s_tach_o), _FNC(iocfg_s_ksp_cs),_FNC(iocfg_g_add_i5), _FNC(iocfg_g_add_i6),
    _FNC(iocfg_g_add_i7), _FNC(iocfg_g_add_i8),_FNC(iocfg_g_map_s),_FNC(iocfg_g_oilp_i), _FNC(iocfg_g_gens_i),0,0,0,0,0,0,0,0 //<-- zero means that these slots are not implemented in this firmware
   },//inverted slots (get/set value)
   {_FNC(iocfg_s_ign_out1i), _FNC(iocfg_s_ign_out2i), _FNC(iocfg_s_ign_out3i), _FNC(iocfg_s_ign_out4i),
    _FNC(iocfg_s_ign_out5i), _FNC(iocfg_s_ecfi), _FNC(iocfg_i_inj_out1i), _FNC(iocfg_s_inj_out2i),
    _FNC(iocfg_s_inj_out3i), _FNC(iocfg_s_inj_out4i), _FNC(iocfg_s_inj_out5i),_FNC(iocfg_s_bli),
    _FNC(iocfg_s_dei),_FNC(iocfg_s_st_blocki), _FNC(iocfg_s_cei), _FNC(iocfg_s_fpmp_oi),
    _FNC(iocfg_s_pwrr_oi), _FNC(iocfg_s_evap_oi), _FNC(iocfg_s_o2sh_oi), _FNC(iocfg_s_cond_oi),
    _FNC(iocfg_s_add_o2i), _FNC(iocfg_g_psi), _FNC(iocfg_g_ref_si), _FNC(iocfg_g_ckpsi),
    _FNC(iocfg_g_add_i1i), _FNC(iocfg_g_add_i2i), _FNC(iocfg_g_add_i3i), _FNC(iocfg_g_gas_vi),
    _FNC(iocfg_g_igni), _FNC(iocfg_g_cond_ii), _FNC(iocfg_g_epas_ii),_FNC(iocfg_g_add_i4i),
    _FNC(iocfg_s_tach_oi), _FNC(iocfg_s_ksp_csi), _FNC(iocfg_g_add_i5i), _FNC(iocfg_g_add_i6i),
    _FNC(iocfg_g_add_i7i), _FNC(iocfg_g_add_i8i),_FNC(iocfg_g_map_si),_FNC(iocfg_g_oilp_ii), _FNC(iocfg_g_gens_ii),0,0,0,0,0,0,0,0 //<-- zero means that these slots are not implemented in this firmware
   },
   //plugs
   {_FNC(iocfg_i_ign_out1), _FNC(iocfg_i_ign_out2), _FNC(iocfg_i_ign_out3), _FNC(iocfg_i_ign_out4),
    _FNC(iocfg_i_ign_out5), _FNC(iocfg_i_ecf), _FNC(iocfg_i_inj_out1), _FNC(iocfg_i_inj_out2),
    _FNC(iocfg_i_inj_out3), _FNC(iocfg_i_inj_out4), _FNC(iocfg_i_inj_out5), _FNC(iocfg_s_stub),
    _FNC(iocfg_s_stub), _FNC(iocfg_i_st_block), _FNC(iocfg_i_ce), _FNC(iocfg_i_fpmp_o),
    _FNC(iocfg_i_pwrr_o), _FNC(iocfg_i_evap_o), _FNC(iocfg_i_o2sh_o), _FNC(iocfg_i_cond_o),
    _FNC(iocfg_i_add_o2), _FNC(iocfg_i_ps), _FNC(iocfg_i_ref_s), _FNC(iocfg_i_ckps),
    _FNC(iocfg_i_add_i1), _FNC(iocfg_i_add_i2), _FNC(iocfg_i_add_i3), _FNC(iocfg_i_gas_v),
    _FNC(iocfg_i_ign), _FNC(iocfg_i_cond_i), _FNC(iocfg_i_epas_i), _FNC(iocfg_i_add_i4),
    _FNC(iocfg_i_tach_o),_FNC(iocfg_i_ksp_cs), _FNC(iocfg_i_add_i5),  _FNC(iocfg_i_add_i6),
    _FNC(iocfg_i_add_i7), _FNC(iocfg_i_add_i8),_FNC(iocfg_i_map_s),_FNC(iocfg_i_oilp_i), _FNC(iocfg_i_gens_i),0,0,0,0,0,0,0,0, //<-- mapped to slots by default
    _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_i_bl),
    _FNC(iocfg_i_de), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub),
    _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub),
    _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub),
    _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub),
    _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub),
    _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub),
    _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub),
    _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub),
    _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub),
    _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub),
    _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub),
    _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub)
   },
   {_FNC(iocfg_s_ign_out1), _FNC(iocfg_s_ign_out2), _FNC(iocfg_s_ign_out3), _FNC(iocfg_s_ign_out4),
    _FNC(iocfg_s_ign_out5), _FNC(iocfg_s_ecf), _FNC(iocfg_i_inj_out1), _FNC(iocfg_s_inj_out2),
    _FNC(iocfg_s_inj_out3), _FNC(iocfg_s_inj_out4), _FNC(iocfg_s_inj_out5),_FNC(iocfg_s_stub),
    _FNC(iocfg_s_stub),_FNC(iocfg_s_st_block), _FNC(iocfg_s_ce), _FNC(iocfg_s_fpmp_o),
    _FNC(iocfg_s_pwrr_o), _FNC(iocfg_s_evap_o), _FNC(iocfg_s_o2sh_o), _FNC(iocfg_s_cond_o),
    _FNC(iocfg_s_add_o2), _FNC(iocfg_g_ps), _FNC(iocfg_g_ref_s), _FNC(iocfg_g_ckps),
    _FNC(iocfg_g_add_i1), _FNC(iocfg_g_add_i2), _FNC(iocfg_g_add_i3), _FNC(iocfg_g_gas_v),
    _FNC(iocfg_g_ign), _FNC(iocfg_g_cond_i), _FNC(iocfg_g_epas_i),_FNC(iocfg_g_add_i4),
    _FNC(iocfg_s_tach_o), _FNC(iocfg_s_ksp_cs), _FNC(iocfg_g_add_i5),  _FNC(iocfg_g_add_i6),
    _FNC(iocfg_g_add_i7),_FNC(iocfg_g_add_i8),_FNC(iocfg_g_map_s), _FNC(iocfg_g_oilp_i), _FNC(iocfg_g_gens_i),0,0,0,0,0,0,0,0, //<-- zero means that these slots are not implemented in this firmware
    _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_bl),
    _FNC(iocfg_s_de),   _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub),
    _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub),
    _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub),
    _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub),
    _FNC(iocfg_g_stub), _FNC(iocfg_g_stub), _FNC(iocfg_g_stub), _FNC(iocfg_g_stub),
    _FNC(iocfg_g_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub),
    _FNC(iocfg_s_stub), _FNC(iocfg_g_stub), _FNC(iocfg_g_stub), _FNC(iocfg_g_stub),
    _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_g_stub),
    _FNC(iocfg_s_stub), _FNC(iocfg_g_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub),
    _FNC(iocfg_g_stub), _FNC(iocfg_g_stub), _FNC(iocfg_g_stub), _FNC(iocfg_g_stub),
    _FNC(iocfg_g_stub), _FNC(iocfg_g_stub), _FNC(iocfg_g_stub), _FNC(iocfg_g_stub),
    _FNC(iocfg_g_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub)
   },

#endif
   _FNC(iocfg_s_stub), _FNC(iocfg_g_stub), //<-- stub, stub

   //Version of this structure - 3.8
   IOREMVER(3,8),

   //2 bytes - size of this structure
   sizeof(iorem_slots_t),
  },

  /**32-bit config data*/
  _CBV32(COPT_OBD_SUPPORT, 0) | _CBV32(COPT_ATMEGA1284, 1) | _CBV32(COPT_ODDFIRE_ALGO, 2) | _CBV32(0/*COPT_ATMEGA128, left for compatibility*/, 3) |
  _CBV32(COPT_SPLIT_ANGLE, 4) | _CBV32(COPT_TPIC8101, 5) | _CBV32(COPT_CAM_SYNC, 6) | _CBV32(COPT_DWELL_CONTROL, 7) |
  _CBV32(COPT_COOLINGFAN_PWM, 8) | _CBV32(COPT_REALTIME_TABLES, 9) | _CBV32(COPT_ICCAVR_COMPILER, 10) | _CBV32(COPT_AVRGCC_COMPILER, 11) |
  _CBV32(COPT_DEBUG_VARIABLES, 12) | _CBV32(COPT_PHASE_SENSOR, 13) | _CBV32(COPT_PHASED_IGNITION, 14) | _CBV32(COPT_FUEL_PUMP, 15) |
  _CBV32(COPT_THERMISTOR_CS, 16) | _CBV32(COPT_SECU3T, 17) | _CBV32(COPT_DIAGNOSTICS, 18) | _CBV32(COPT_HALL_OUTPUT, 19) |
  _CBV32(COPT_REV9_BOARD, 20) | _CBV32(COPT_STROBOSCOPE, 21) | _CBV32(COPT_SM_CONTROL, 22) | _CBV32(COPT_VREF_5V, 23) |
  _CBV32(COPT_HALL_SYNC, 24) | _CBV32(0/*UART_BINARY, left for compatibility*/, 25) | _CBV32(COPT_CKPS_2CHIGN, 26) | _CBV32(COPT_ATMEGA644, 27) |
  _CBV32(COPT_FUEL_INJECT, 28) | _CBV32(COPT_GD_CONTROL, 29) | _CBV32(COPT_CARB_AFR, 30) | _CBV32(COPT_CKPS_NPLUS1, 31),

  /**Reserved bytes*/
  {0,0,0},

  /**Version of the firmware. Do not forget to write out same value into the signature info! */
  0x50,

  /**2 bytes - size of this structure. */
  sizeof(cd_data_t)
 },

 /** Fill reserve parameters with default values */
 {
  .starter_off =                 600,
  .smap_abandon =                650,

  .load_lower =                  1920,
  .load_upper =                  6400,
  .map_curve_offset =            54,
  .map_curve_gradient =          1089,
  .map2_curve_offset =           54,
  .map2_curve_gradient =         1089,

  .carb_invers =                 0,
  .tps_curve_offset =            160,
  .tps_curve_gradient =          0,
  .tps_threshold =               0,

  .ie_lot =                      1900,
  .ie_hit =                      2100,
  .ie_lot_g =                    1900,
  .ie_hit_g =                    2100,
#ifdef FUEL_INJECT
  .fe_on_threshold =             PRESSURE_MAGNITUDE(500.0),
#else
  .fe_on_threshold =             PRESSURE_MAGNITUDE(6.12),
#endif
  .shutoff_delay =               0,
  .fuelcut_map_thrd =            1280,
  .fuelcut_cts_thrd =            60,

  .angle_dec_speed =             96,
  .angle_inc_speed =             96,
  .max_angle =                   1600,
  .min_angle =                   0,
  .angle_corr =                  0,
  .zero_adv_ang =                0,

  .fn_gasoline =                 0,
  .fn_gas =                      0,

  .idl_flags =                   _BV(IRF_USE_REGONGAS) | _BV(IRF_USE_CLONGAS), //use regulator on gas, use clesed-loop on gas
  .idling_rpm =                  800,
  .ifac1 =                       25,
  .ifac2 =                       25,
  .MINEFR =                      20,
  .idlreg_min_angle =           -160,
  .idlreg_max_angle =            320,
  .idlreg_turn_on_temp =         200,

  .tmp_flags =                   1,                  //use CLT sensor, don't use PWM, don't use CLT curce map
  .vent_on =                     392,
  .vent_off =                    384,
  .vent_pwmfrq =                 _FAN_PWMFRQ(5000),  //5000Hz

  .map_adc_factor =              _ACF,
  .map_adc_correction =          _ACC,
  .ubat_adc_factor =             _ACF,
  .ubat_adc_correction =         _ACC,
  .temp_adc_factor =             _ACF,
  .temp_adc_correction =         _ACC,
  .tps_adc_factor =              _ACF,
  .tps_adc_correction =          _ACC,
  .ai1_adc_factor =              _ACF,
  .ai1_adc_correction =          _ACC,
  .ai2_adc_factor =              _ACF,
  .ai2_adc_correction =          _ACC,

  .ckps_cogs_btdc =              20,
  .ckps_ignit_cogs =             10,
  .ckps_engine_cyl =             4,
  .ckps_cogs_num =               60,
  .ckps_miss_num =               2,
  .hall_flags =                  0x00,
  .hall_wnd_width =              1920,

  .ign_cutoff =                  0,
  .ign_cutoff_thrd =             7500,

  .hop_start_ang =              0,
  .hop_durat_ang =              60.0*32,

#if (FW_BAUD_RATE == 19200)
  .uart_divisor =                CBR_19200,
#elif (FW_BAUD_RATE == 38400)
  .uart_divisor =                CBR_38400,
#elif (FW_BAUD_RATE == 57600)
  .uart_divisor =                CBR_57600,
#elif (FW_BAUD_RATE == 115200)
  .uart_divisor =                CBR_115200,
#elif (FW_BAUD_RATE == 250000)
  .uart_divisor =                CBR_250000,
#else
 #error "Invalid baud rate value specified for firmware!"
#endif

  .uart_period_t_ms =            2,

  .knock_use_knock_channel =     0,
  .knock_bpf_frequency =         35,
  .knock_k_wnd_begin_angle =     0,
  .knock_k_wnd_end_angle =       800,
  .knock_int_time_const =        23,
  .knock_retard_step =           128,
  .knock_advance_step =          8,
  .knock_max_retard =            128,
  .knock_threshold =             1000,
  .knock_recovery_delay =        2,

  .sm_steps =                    800,
  .choke_rpm_if =                51,
  .choke_corr_time =             {500, 250},

  .bt_flags =                    _BV(BTF_SET_BBR) | _BV(BTF_CHK_FWCRC)
#ifndef SECU3T
| _BV(BTF_BT_TYPE0) | _BV(BTF_USE_BT)
#endif
,
  .ibtn_keys =                   {{0,0,0,0,0,0},{0,0,0,0,0,0}},/**<--iButton keys database. Write out your own 48-bit keys here */

  .uni_output =                  {{0,0,0,220,200,220,200},{0,0,0,220,200,220,200},{0,0,0,220,200,220,200},{0,0,0,220,200,220,200},{0,0,0,220,200,220,200},{0,0,0,220,200,220,200}},
  .uniout_12lf =                 15,                   //logic function between 1st and 2nd outputs

  .inj_flags =                   0,                    //
  .inj_config =                  {0x12, 0x12},         //multi-point simultaneous injection, 2 squirts per cycle
  .inj_flow_rate =               {INJ_FLRT(200.0), INJ_FLRT(200.0)}, //200 cc/min          (for management software only)
  .inj_cyl_disp =                CYL_DISP(0.375),      //0.375L (1.5/4)      (for management software only)
  .inj_sd_igl_const =            {86207, 114192},      //((0.375L * 3.482 * 18750000) / 142g) * ((1 * 4) / (2 * 4)), petrol density is 0.71 g/cc, 1bank,4cyl,2squirts,4injectors
                                                       //((0.375L * 3.482 * 18750000) / 107.2g) * ((1 * 4) / (2 * 4)), LPG density is 0.536 g/cc, 1bank,4cyl,2squirts,4injectors
  .inj_prime_cold =              _DLV(6.0),            //6 ms at -30°C
  .inj_prime_hot =               _DLV(2.0),            //2 ms at 70°C
  .inj_prime_delay =             SYS_TIMEX10_S(2.0),   //fire prime pulse after 2 seconds

  .inj_cranktorun_time =         SYS_TIME_S(3.00),     //3 seconds
  .inj_aftstr_strokes =          38,                   //152 strokes

  .inj_lambda_str_per_stp =      8,                    //8 strokes
  .inj_lambda_step_size_p =      EGO_CORR(2.5),        //2.5%
  .inj_lambda_corr_limit_p =     EGO_CORR(30.0),       //30% max
  .inj_lambda_swt_point =        VOLTAGE_MAGNITUDE(0.5), //0.5V
  .inj_lambda_temp_thrd =        TEMPERATURE_MAGNITUDE(60.0), //60°C
  .inj_lambda_rpm_thrd =         1200,                 //1200 min-1
  .inj_lambda_activ_delay =      45,                   //activation after 45 seconds

  .inj_ae_tpsdot_thrd =          50,                   //50%/sec
  .inj_ae_coldacc_mult =         AE_CAM(1.5),          //*150% at -30°C, allowed range is 1.0...6.00

  .gd_steps =                    256,                  //256 steps, gas dose number of steps

  .inj_timing =                  {0,0},                //TDC (0 = 720°)

  .flpmp_flags =                 _BV(FPF_OFFONGAS),    //turn off fuel pump when GAS_V = 1

  .choke_flags =                 0,                    //do not turn off additional startup closing, RPM regulator when fuel type is gas and don't use throttle position in choke initialization

  .revlim_lot =                  8000,                 // 8000 min-1
  .revlim_hit =                  8100,                 // 8100 min-1

  .inj_timing_crk =              {0,0},                //TDC

  .gd_fc_closing =               GD_MAGNITUDE(30),     //close for 30%

  .inj_lambda_step_size_m =      EGO_CORR(2.5),        //2.5%
  .inj_lambda_corr_limit_m =     EGO_CORR(30.0),       //30% max

  .gd_lambda_corr_limit_p =      EGO_CORR(30.0),       //30% max
  .gd_lambda_corr_limit_m =      EGO_CORR(30.0),       //30% max

  .inj_lambda_dead_band =        VOLTAGE_MAGNITUDE(0.0), //zero dead band by default

  .load_src_cfg =                0,                    //default is MAP

  .idl_to_run_add =              60,                   //30%
  .rpm_on_run_add =              20,                   //200 min-1
  .idl_reg_p =                   {25,25},              //0.1 (proportional)
  .idl_reg_i =                   {25,25},              //0.1 (integral)
  .idl_coef_thrd1 =              64,                   //rpm x 1.5
  .idl_coef_thrd2 =              110,                  //rpm x 1.86
  .idl_intrpm_lim =              20,                   //200 min-1
  .idl_map_value =               1600,                 //25 kPa

  .inj_lambda_senstype =         0,                    //NBO sensor type
  .gd_lambda_stoichval =         1984,                 //15.5

  .inj_lambda_ms_per_stp =       0,                    //0 by default

  .idl_iacminpos =               20,                   //10%
  .idl_iacmaxpos =               180,                  //90%

  .hall_degrees_btdc =           60*32,                //60° BTDC

  .vss_period_dist =             ROUNDU16(0.16666*32768), //0.16666m per pulse, max 0.9999

  .inj_anglespec =               ((INJANGLESPEC_BEGIN << 4) || INJANGLESPEC_BEGIN),   //beginning of pulse

  .evap_afbegin =                938,                  //rpm*load = 30000
  .evap_afslope =                6320,                 //0.0060

  .sm_freq =                     0,                    //300Hz

  .ai3_adc_factor =              _ACF,
  .ai3_adc_correction =          _ACC,
  .ai4_adc_factor =              _ACF,
  .ai4_adc_correction =          _ACC,

  .cond_pvt_on =                 VOLTAGE_MAGNITUDE(1.6),
  .cond_pvt_off =                VOLTAGE_MAGNITUDE(2.5),

  .inj_ae_decay_time =           50,

  .cond_min_rpm =                1200,

  .inj_lambda_flags =           _BV(LAMFLG_IDLCORR),

  .gd_freq =                     0,                    //300Hz

  .gd_maxfreqinit =              0,

  .fff_const =                   17476,               // = (16000 / (1000 * 60)) * 65536

  .mapsel_uni =                  0xFF,                //usage of univ. output conditions for selection of map sets disabled

  .barocorr_type =               0,                   //no barometric correction

  .inj_floodclear_tps =          TPS_MAGNITUDE(90.0),

  .vent_tmr =                    6000,                //60 sec.

  .fp_timeout_strt =             SYS_TIME_S(5.0)/10,  //5 seconds

  .eh_heating_time =             {15, 30},            //15 and 30 seconds
  .eh_temper_thrd =              70,                  //70 °C
  .eh_heating_act =              SYSTIM_MAGS(0.06),   //60 ms
  .eh_aflow_thrd =               10000,               //value = (load * rpm) / 32

  .inj_min_pw =                  {39,39},             //1ms, 1ms

  .pwmfrq =                      {_FAN_PWMFRQ(1000), _FAN_PWMFRQ(1000)}, //1000Hz both channels

  .ai5_adc_factor =              _ACF1,               //use MCP3204 factor
  .ai5_adc_correction =          _ACC1,
  .ai6_adc_factor =              _ACF1,
  .ai6_adc_correction =          _ACC1,
  .ai7_adc_factor =              _ACF1,
  .ai7_adc_correction =          _ACC1,
  .ai8_adc_factor =              _ACF1,
  .ai8_adc_correction =          _ACC1,

  .func_flags =                  0,

  .inj_aftstr_strokes1 =         38,

  .ve2_map_func =                VE2MF_1ST,           //use 1st VE map

  .inj_ae_type =                 0,                   //accel. pump
  .inj_ae_time =                 30,                  //30 strokes

  .knock_selch =                 0,                   //use 1st sensor for all channels (cylinders)

  .knkclt_thrd =                 TEMPERATURE_MAGNITUDE(70.0), //70°C

  .iac_reg_db =                  10,                  //10 min-1


  .inj_maf_const =               {30947, 40994},      //(((120 * 18750000) / 142g) * (1 / (2 * 4))) / 64, petrol density is 0.71 g/cc, 1bank,2squirts,4injectors
                                                      //(((120 * 18750000) / 107.2g) * (1 / (2 * 4))) / 64, LPG density is 0.536 g/cc, 1bank,2squirts,4injectors
  .mafload_const =               861909,              //(((101.35 * 64 * 128) * 120) / (64 * 0.0012041)) / (375cc * 4), 4cyl


  .adc_flags =                   1,                   //multilpy ADC offset by corr. factor

  .fuelcut_uni =                 0x0F,                //Don't use uni.outs for fuel cut
  .igncut_uni =                  0x0F,                //Don't use uni.outs for ignition cut

  .gas_v_uni =                   0x0F,                //Don't use uni.outs for emulation of GAS_V input

  .inj_max_pw =                  {31250, 31250},      //100ms for petrol and gas

  .igntim_flags =                _BV(IGNTF_MANIDL),   //use manual ignition timing on idling
  .shift_igntim =                0,

  .stbl_str_cnt =                10,                  //10 strokes
  .strt_flags =                  _BV(STRTF_FLDCLRSTR),// start of engine in flood clear mode enabled

  .inj_ae_ballance =             128,                 //50%
  .inj_ae_mapdot_thrd =          50,                  //50kPa per second

  .reserved =                    {0},
  .crc =                         0
 },

 /**Fill additional data with default values */
 {
  /**Attenuator's lookup table (for knock channel), by default k = 1.000 */
  {0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,
   0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,
   0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,
   0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,
   0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,
   0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,
   0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,
   0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E
  },

  /**Lookup table for controlling coil's accumulation time (dwell control) */
  {//  5.4         5.8         6.2         6.6         7.0        7.4        7.8         8.2
   _DLV(15.00),_DLV(13.60),_DLV(12.50),_DLV(11.40),_DLV(10.20),_DLV(9.25),_DLV(8.30),_DLV(7.55),
   //  8.6         9.0         9.4         9.8        10.2       10.6       11.0        11.4
   _DLV(6.80), _DLV(6.30), _DLV(5.80), _DLV(5.34), _DLV(4.88), _DLV(4.53),_DLV(4.19),_DLV(3.94),
   // 11.8        12.2        12.6        13.0        13.4       13.8       14.2        14.6
   _DLV(3.69), _DLV(3.44), _DLV(3.19), _DLV(3.03), _DLV(2.90), _DLV(2.70),_DLV(2.55),_DLV(2.43),
   // 15.0        15.4        15.8        16.2        16.6       17.0       17.4        17.8
   _DLV(2.30), _DLV(2.22), _DLV(2.13), _DLV(2.00), _DLV(1.88), _DLV(1.85),_DLV(1.82),_DLV(1.80),
  },

  /**Fill coolant temperature sensor lookup table*/
  {_TLV(120.0), _TLV(100.0), _TLV(81.0), _TLV(65.5), _TLV(56.0), _TLV(48.5), _TLV(42.0), _TLV(36.0),
   _TLV(30.0), _TLV(23.0), _TLV(18.0), _TLV(12.0), _TLV(4.0), _TLV(-4.0), _TLV(-15.0), _TLV(-37.0),
  ROUND(0.0 / ADC_DISCRETE), ROUND(4.98 / ADC_DISCRETE)},

  /**Fill air temperature sensor lookup table (temperature vs voltage)*/
  {_TLV(120.0), _TLV(100.0), _TLV(81.0), _TLV(65.5), _TLV(56.0), _TLV(48.5), _TLV(42.0), _TLV(36.0),
   _TLV(30.0), _TLV(23.0), _TLV(18.0), _TLV(12.0), _TLV(4.0), _TLV(-4.0), _TLV(-15.0), _TLV(-37.0),
   ROUND(0.0 / ADC_DISCRETE), ROUND(4.98 / ADC_DISCRETE)},

  /**Fill air temperature lookup table for advance angle correction*/
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},

  /**Fill RPM grid points lookup table*/
  {600,720,840,990,1170,1380,1650,1950,2310,2730,3210,3840,4530,5370,6360,7500},
  /**Fill PRM grid cell sizes lookup table*/
  {120,120,150,180, 210, 270, 300, 360, 420, 480, 630, 690, 840, 990, 1140},


  {//  600       720         840        990       1170        1380       1650       1950       2310       2730       3210      3840        4530      5370        6360      7500 (min-1)
   _REVARR16(
   {_AL16(_GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0))}, //100% 16
   {_AL16(_GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0))}, //     15
   {_AL16(_GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0))}, //     14
   {_AL16(_GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0))}, //80%  13
   {_AL16(_GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0))}, //     12
   {_AL16(_GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0))}, //     11
   {_AL16(_GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0))}, //60%  10
   {_AL16(_GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0))}, //      9
   {_AL16(_GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0))}, //      8
   {_AL16(_GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0))}, //40%   7
   {_AL16(_GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0))}, //      6
   {_AL16(_GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0))}, //      5
   {_AL16(_GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0))}, //20%   4
   {_AL16(_GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0))}, //      3
   {_AL16(_GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0))}, //      2
   {_AL16(_GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0))}) //0%    1
  },

  //CE settings
  {
   .map_v_min = VOLTAGE_MAGNITUDE(0.0),
   .map_v_max = VOLTAGE_MAGNITUDE(4.97),
   .map_v_em = VOLTAGE_MAGNITUDE(1.00),
   .map_v_flg = 1,

   .vbat_v_min = VOLTAGE_MAGNITUDE(8.00),
   .vbat_v_max = VOLTAGE_MAGNITUDE(16.00),
   .vbat_v_em = VOLTAGE_MAGNITUDE(14.00),
   .vbat_v_flg = 1,

#ifdef THERMISTOR_CS
   .cts_v_min = VOLTAGE_MAGNITUDE(0.01), //2.28 (for LM235 sensor)
   .cts_v_max = VOLTAGE_MAGNITUDE(4.97), //3.93 (for LM235 sensor)
   .cts_v_em = VOLTAGE_MAGNITUDE(0.48),
#else
   .cts_v_min = VOLTAGE_MAGNITUDE(2.28),
   .cts_v_max = VOLTAGE_MAGNITUDE(3.93),
   .cts_v_em = VOLTAGE_MAGNITUDE(3.20),
#endif
   .cts_v_flg = 1,

   .ks_v_min = VOLTAGE_MAGNITUDE(0.00),
   .ks_v_max = VOLTAGE_MAGNITUDE(4.97),
   .ks_v_em = VOLTAGE_MAGNITUDE(0.10),
   .ks_v_flg = 1,

   .tps_v_min = VOLTAGE_MAGNITUDE(0.00),
   .tps_v_max = VOLTAGE_MAGNITUDE(5.10),
   .tps_v_em = VOLTAGE_MAGNITUDE(0.675),
   .tps_v_flg = 1,

   .add_i1_v_min = VOLTAGE_MAGNITUDE(0.00),
   .add_i1_v_max = VOLTAGE_MAGNITUDE(5.10),
   .add_i1_v_em = VOLTAGE_MAGNITUDE(0.43),
   .add_i1_v_flg = 1,

   .add_i2_v_min = VOLTAGE_MAGNITUDE(0.00),
   .add_i2_v_max = VOLTAGE_MAGNITUDE(5.10),
   .add_i2_v_em = VOLTAGE_MAGNITUDE(3.65),
   .add_i2_v_flg = 1,

   .add_i3_v_min = VOLTAGE_MAGNITUDE(0.00),
   .add_i3_v_max = VOLTAGE_MAGNITUDE(5.10),
   .add_i3_v_em = VOLTAGE_MAGNITUDE(2.50),
   .add_i3_v_flg = 1,

   .add_i4_v_min = VOLTAGE_MAGNITUDE(0.00),
   .add_i4_v_max = VOLTAGE_MAGNITUDE(5.10),
   .add_i4_v_em = VOLTAGE_MAGNITUDE(2.50),
   .add_i4_v_flg = 1,

   .add_i5_v_min = VOLTAGE_MAGNITUDE(0.00),
   .add_i5_v_max = VOLTAGE_MAGNITUDE(5.10),
   .add_i5_v_em = VOLTAGE_MAGNITUDE(2.50),
   .add_i5_v_flg = 1,

   .add_i6_v_min = VOLTAGE_MAGNITUDE(0.00),
   .add_i6_v_max = VOLTAGE_MAGNITUDE(5.10),
   .add_i6_v_em = VOLTAGE_MAGNITUDE(2.50),
   .add_i6_v_flg = 1,

   .add_i7_v_min = VOLTAGE_MAGNITUDE(0.00),
   .add_i7_v_max = VOLTAGE_MAGNITUDE(5.10),
   .add_i7_v_em = VOLTAGE_MAGNITUDE(2.50),
   .add_i7_v_flg = 1,

   .add_i8_v_min = VOLTAGE_MAGNITUDE(0.00),
   .add_i8_v_max = VOLTAGE_MAGNITUDE(5.10),
   .add_i8_v_em = VOLTAGE_MAGNITUDE(2.50),
   .add_i8_v_flg = 1,

   .oilpress_thrd = 128, //0.5 kg/cm2
   .oilpress_timer = 1000 //1000 strokes
  },

   /**Fill barometric correction lookup table*/
   {//
    _BC(1.000),_BC(1.000),_BC(1.000),_BC(1.000),_BC(1.000),_BC(1.000),_BC(1.000),_BC(1.000),_BC(1.000),
    _BCX(70.0),_BCX(110.0) //
   },

   /** Fill ignition timing vs voltage. Linear function with small dead band near to 2.5V */
   {_PA4LV(-10.5),_PA4LV(-09.0),_PA4LV(-07.5),_PA4LV(-6.0),_PA4LV(-04.5),_PA4LV(-03.0),_PA4LV(-01.5),_PA4LV(00.0),
    _PA4LV( 00.0),_PA4LV( 01.5),_PA4LV( 03.0),_PA4LV( 04.5),_PA4LV( 06.0),_PA4LV( 07.5),_PA4LV( 09.0),_PA4LV(10.5)
   },

  /**Fill TEMP2 sensor lookup table (temperature vs voltage)*/
  {_TLV(120.0), _TLV(100.0), _TLV(81.0), _TLV(65.5), _TLV(56.0), _TLV(48.5), _TLV(42.0), _TLV(36.0),
   _TLV(30.0), _TLV(23.0), _TLV(18.0), _TLV(12.0), _TLV(4.0), _TLV(-4.0), _TLV(-15.0), _TLV(-37.0),
   ROUND(0.0 / ADC_DISCRETE), ROUND(4.98 / ADC_DISCRETE)
  },

  //Correction of ignition timing vs CLT on cranking
  {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},

  //EGO heating pause time (seconds) vs board voltage (Volts)
  {//  5.4         5.8         6.2         6.6         7.0        7.4        7.8         8.2
   _HPV(0.00),_HPV(0.00),_HPV(0.00),_HPV(0.00),_HPV(0.00),_HPV(0.01),_HPV(0.01),_HPV(0.02),
   //  8.6         9.0         9.4         9.8        10.2       10.6       11.0        11.4
   _HPV(0.06), _HPV(0.07), _HPV(0.08), _HPV(0.10), _HPV(0.12), _HPV(0.14),_HPV(0.16),_HPV(0.18),
   // 11.8        12.2        12.6        13.0        13.4       13.8       14.2        14.6
   _HPV(0.20), _HPV(0.21), _HPV(0.22), _HPV(0.23), _HPV(0.24), _HPV(0.27),_HPV(0.30),_HPV(0.32),
   // 15.0        15.4        15.8        16.2        16.6       17.0       17.4        17.8
   _HPV(0.34), _HPV(0.37), _HPV(0.40), _HPV(0.44), _HPV(0.48), _HPV(0.50),_HPV(0.52),_HPV(0.54),
  },

  //Cranking RPM threshold vs coolant temperature
  {
   _CRT(600),_CRT(600),_CRT(600),_CRT(600),_CRT(600),_CRT(600),_CRT(600),_CRT(600),
   _CRT(600),_CRT(600),_CRT(600),_CRT(600),_CRT(600),_CRT(600),_CRT(600),_CRT(600),
  },

  //Cranking time (strokes) after reaching of threshold vs coolant temperature
  {
   _CTM(0),_CTM(0),_CTM(0),_CTM(0),_CTM(0),_CTM(0),_CTM(0),_CTM(0),
   _CTM(0),_CTM(0),_CTM(0),_CTM(0),_CTM(0),_CTM(0),_CTM(0),_CTM(0),
  },

  //Start map abandon RPM vs coolant temperature
  {
   _SMA(800),_SMA(800),_SMA(800),_SMA(800),_SMA(800),_SMA(800),_SMA(800),_SMA(800),
   _SMA(800),_SMA(800),_SMA(800),_SMA(800),_SMA(800),_SMA(800),_SMA(800),_SMA(800),
  },

  /**Fill CLT grid points lookup table*/
  {_CLT(-30.0),_CLT(-20.0),_CLT(-10.0),_CLT(0.0),_CLT(10.0),_CLT(20.0),_CLT(30.0),_CLT(40.0),_CLT(50.0),_CLT(60.0),_CLT(70.0),_CLT(80.0),_CLT(90.0),_CLT(100.0),_CLT(110.0),_CLT(120.0)},
  /**Fill CLT grid cell sizes lookup table*/
  {_CLT(10.0),_CLT(10.0),_CLT(10.0),_CLT(10.0),_CLT(10.0),_CLT(10.0),_CLT(10.0),_CLT(10.0),_CLT(10.0),_CLT(10.0),_CLT(10.0),_CLT(10.0),_CLT(10.0),_CLT(10.0),_CLT(10.0)},

  //Fill knock zones map
  //600 720 840 990 1170 1380 1650 1950 2310 2730 3210 3840 4530 5370 6360 7500 (min-1)
  //  -->
  {
  _REVARR16(
  _KNR(1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1), //16
  _KNR(1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1), //15
  _KNR(1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1), //14
  _KNR(1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1), //13
  _KNR(1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1), //12
  _KNR(1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1), //11
  _KNR(1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1), //10
  _KNR(1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1), //9
  _KNR(1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1), //8
  _KNR(1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1), //7
  _KNR(1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1), //6
  _KNR(1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1), //5
  _KNR(1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1), //4
  _KNR(1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1), //3
  _KNR(1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1), //2  ^
  _KNR(1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1)) //1  |
  },

/*
 _REVARR16(
 _KNR(0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1), //16
 _KNR(0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1), //15
 _KNR(0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1), //14
 _KNR(0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1), //13
 _KNR(0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1), //12
 _KNR(0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1), //11
 _KNR(0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1), //10
 _KNR(0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0), //9
 _KNR(0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0), //8
 _KNR(0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0), //7
 _KNR(0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0), //6
 _KNR(0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0), //5
 _KNR(0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0), //4
 _KNR(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0), //3
 _KNR(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0), //2  ^
 _KNR(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)) //1  |
*/

  /**Fill GRTEMP sensor lookup table (temperature vs voltage)*/
  {_TLV(120.0), _TLV(100.0), _TLV(81.0), _TLV(65.5), _TLV(56.0), _TLV(48.5), _TLV(42.0), _TLV(36.0),
   _TLV(30.0), _TLV(23.0), _TLV(18.0), _TLV(12.0), _TLV(4.0), _TLV(-4.0), _TLV(-15.0), _TLV(-37.0),
   ROUND(0.0 / ADC_DISCRETE), ROUND(4.98 / ADC_DISCRETE)
  },

  /**Fill gas reducer's heating duty look up table (%)*/
  {200,200,200,200,200,160,120,80,40,0,0,0,0,0,0,0},

  /**Fill PWM IAC duty coefficient vs board voltage map
   *  5.4      6.2      7.0      7.8      8.6      9.4      10.2    11.0     11.8     12.6     13.4     14.2     15.0     15.8     16.6     17.4 */
  {_UF(1.0),_UF(1.0),_UF(1.0),_UF(1.0),_UF(1.0),_UF(1.0),_UF(1.0),_UF(1.0),_UF(1.0),_UF(1.0),_UF(1.0),_UF(1.0),_UF(1.0),_UF(1.0),_UF(1.0),_UF(1.0)},

  /**Fill load grid points lookup table*/
  {_LM(25.0),_LM(30.0),_LM(35.0),_LM(40.0),_LM(45.0),_LM(50.0),_LM(55.0),_LM(60.0),_LM(65.0),_LM(70.0),_LM(75.0),_LM(80.0),_LM(85.0),_LM(90.0),_LM(95.0),_LM(100.0)},

  /**Fill load grid cell sizes lookup table*/
  {_LM(5.0),_LM(5.0),_LM(5.0),_LM(5.0),_LM(5.0),_LM(5.0),_LM(5.0),_LM(5.0),_LM(5.0),_LM(5.0),_LM(5.0),_LM(5.0),_LM(5.0),_LM(5.0),_LM(5.0)},

  /**Fill after start enrichment strokes vs coolant temperature map */
  {150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150},

  /**Fill after start enrichment strokes vs coolant temperature map */
  {150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150},

  /**Fill gas valve's opening delay vs gas reducer's temperature map*/
  {1200,1100,1000,900,800,700,600,500,420,340,260,180,100,50,30,10},

  /**Fill "fuel tank level vs voltage" map*/
  {
   _FTL(0.0),_FTL(0.0),_FTL(0.0),_FTL(0.0),_FTL(0.0),_FTL(0.0),_FTL(0.0),_FTL(0.0),
   _FTL(0.0),_FTL(0.0),_FTL(0.0),_FTL(0.0),_FTL(0.0),_FTL(0.0),_FTL(0.0),_FTL(0.0),_FTL(0.0),
   ROUND(0.0 / ADC_DISCRETE), ROUND(5.00 / ADC_DISCRETE)
  },

  /**Fill "exhaust gas temperature vs voltage" map*/
  {
   _EGT(0.0),_EGT(0.0),_EGT(0.0),_EGT(0.0),_EGT(0.0),_EGT(0.0),_EGT(0.0),_EGT(0.0),
   _EGT(0.0),_EGT(0.0),_EGT(0.0),_EGT(0.0),_EGT(0.0),_EGT(0.0),_EGT(0.0),_EGT(0.0),_EGT(0.0),
   ROUND(0.0 / ADC_DISCRETE), ROUND(5.00 / ADC_DISCRETE)
  },

  /**Fill "oil pressure vs voltage" map*/
  {
   _OIP(0.0),_OIP(0.0),_OIP(0.0),_OIP(0.0),_OIP(0.0),_OIP(0.0),_OIP(0.0),_OIP(0.0),
   _OIP(0.0),_OIP(0.0),_OIP(0.0),_OIP(0.0),_OIP(0.0),_OIP(0.0),_OIP(0.0),_OIP(0.0),_OIP(0.0),
   ROUND(0.0 / ADC_DISCRETE), ROUND(5.00 / ADC_DISCRETE)
  },

  /**Fill inj. PW coeff. vs voltage map*/
  {
   _IPC(1.000),_IPC(1.000),_IPC(1.000),_IPC(1.000),_IPC(1.000),_IPC(1.000),_IPC(1.000),_IPC(1.000),
   _IPC(1.000),_IPC(1.000),_IPC(1.000),_IPC(1.000),_IPC(1.000),_IPC(1.000),_IPC(1.000),_IPC(1.000),_IPC(1.000)
  },

  /**Fill MAF's flow curve map (flow in g/sec vs voltage)*/
  { //0 - 0.0V; 63 - 5.0V
   _MAF(0.00),  _MAF(0.5),   _MAF(1.0),   _MAF(1.69),  _MAF(2.02),  _MAF(2.54),  _MAF(3.07),  _MAF(3.61),
   _MAF(4.17),  _MAF(4.77),  _MAF(5.49),  _MAF(6.32),  _MAF(7.21),  _MAF(8.13),  _MAF(9.09),  _MAF(10.08),
   _MAF(11.02), _MAF(11.93), _MAF(12.92), _MAF(14.26), _MAF(16.16), _MAF(18.65), _MAF(21.01), _MAF(22.76),
   _MAF(24.22), _MAF(26.02), _MAF(28.16), _MAF(30.51), _MAF(33.10), _MAF(35.92), _MAF(38.95), _MAF(42.33),
   _MAF(46.30), _MAF(50.49), _MAF(54.61), _MAF(58.05), _MAF(62.15), _MAF(66.49), _MAF(70.99), _MAF(75.65),
   _MAF(80.55), _MAF(85.74), _MAF(91.19), _MAF(96.87), _MAF(102.71),_MAF(108.69),_MAF(114.79),_MAF(121.01),
   _MAF(127.37),_MAF(133.87),_MAF(140.53),_MAF(147.39),_MAF(154.51),_MAF(161.95),_MAF(169.78),_MAF(177.90),
   _MAF(186.12),_MAF(194.20),_MAF(202.14),_MAF(210.56),_MAF(220.17),_MAF(231.68),_MAF(246.10),_MAF(261.22),
   650, ROUND(0.0 / ADC_DISCRETE), ROUND(5.00 / ADC_DISCRETE)
  },

  /**Fill FTLS correction's coefficient look up table*/
  {//  5.4         5.8         6.2         6.6         7.0        7.4        7.8         8.2
   _UF(1.000), _UF(1.000), _UF(1.000), _UF(1.000), _UF(1.000), _UF(1.000),_UF(1.000), _UF(1.000),
   //  8.6         9.0         9.4         9.8        10.2       10.6       11.0        11.4
   _UF(1.000), _UF(1.000), _UF(1.000), _UF(1.000), _UF(1.000), _UF(1.000),_UF(1.000), _UF(1.000),
   // 11.8        12.2        12.6        13.0        13.4       13.8       14.2        14.6
   _UF(1.000), _UF(1.000), _UF(1.000), _UF(1.000), _UF(1.000), _UF(1.000),_UF(1.000), _UF(1.000),
   // 15.0        15.4        15.8        16.2        16.6       17.0       17.4        17.8
   _UF(1.000), _UF(1.000), _UF(1.000), _UF(1.000), _UF(1.000), _UF(1.000),_UF(1.000), _UF(1.000),
  },

  //Fill lambda zones map
  //600 720 840 990 1170 1380 1650 1950 2310 2730 3210 3840 4530 5370 6360 7500 (min-1)
  //   -->
  {
  _REVARR16(
  _KNR(1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1), //16
  _KNR(1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1), //15
  _KNR(1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1), //14
  _KNR(1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1), //13
  _KNR(1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1), //12
  _KNR(1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1), //11
  _KNR(1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1), //10
  _KNR(1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1), //9
  _KNR(1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1), //8
  _KNR(1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1), //7
  _KNR(1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1), //6
  _KNR(1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1), //5
  _KNR(1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1), //4
  _KNR(1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1), //3
  _KNR(1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1), //2  ^
  _KNR(1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1)) //1  |
  },

  /**Fill "fuel temperature vs voltage" map*/
  {
   _EGT(0.0),_EGT(0.0),_EGT(0.0),_EGT(0.0),_EGT(0.0),_EGT(0.0),_EGT(0.0),_EGT(0.0),
   _EGT(0.0),_EGT(0.0),_EGT(0.0),_EGT(0.0),_EGT(0.0),_EGT(0.0),_EGT(0.0),_EGT(0.0),_EGT(0.0),
   ROUND(0.0 / ADC_DISCRETE), ROUND(5.00 / ADC_DISCRETE)
  },

  /**Fill fuel density correction map (coefficient vs temperature of fuel in ramp)*/
  {_FDC(1.0000),_FDC(1.0000),_FDC(1.0000),_FDC(1.0000),_FDC(1.0000),_FDC(1.0000),_FDC(1.0000),_FDC(1.0000),
   _FDC(1.0000),_FDC(1.0000),_FDC(1.0000),_FDC(1.0000),_FDC(1.0000),_FDC(1.0000),_FDC(1.0000),_FDC(1.0000)
  },

  /**reserved*/
  {0},

  .evap_clt = TEMPERATURE_MAGNITUDE(75.0), //75°C
  .evap_tps_lo = TPS_MAGNITUDE(4.0), //4%
  .evap_tps_hi = TPS_MAGNITUDE(98.0), //98%
  .fi_enter_strokes = 5,  //5 strokes
  .fi_leave_strokes = 5,  //5 strokes
  .iac_cond_add = 15*2,    //+15%
  .aircond_clt = TEMPERATURE_MAGNITUDE(75.0), //75°C
  .aircond_tps = TPS_MAGNITUDE(68.0), //68%
  .idl_ve = ROUND(0*2048), //turned off
  .frap = PRESSURE_MAGNITUDE(0.0), //absolute pressure in the fuel rail
  .idl_ve_g = ROUND(0*2048), //turned off
  .reserv_0 = 0, //reserved
  .heating_t_off = TEMPERATURE_MAGNITUDE(65.0), //65°C
  .heating_time = 100, //10 min
  .idltorun_stp_en = 8, //0.25%
  .idltorun_stp_le = 8, //0.25%
            // FRQ  MAP  BAT  TMP  TPS  AI1  AI2  SPD  AI3  AI4  AI5  AI6  AI7  AI8
  .inpavnum = { 4,   4,   4,   8,   4,   4,   4,   8,   4,   4,   4,   4,   4,   4}, //Number of averages for each input
  .vent_delay = 0,
  .vent_iacoff = 0,
  .epas_iacoff = 0,
  .vent_pwmsteps = 31, //number of PWM discretes for 5kHz with 20MHz quartz
  .vent_minband = 2,
  .an_tps_mul = 0,     //0 - multiply by MAP, 1 - multiply by TPS
  .hall_predict = 0,  //last interval
  .vtachom_mult = ROUNDU16(0.5*8192), //generate tachometer pulses as for 4 cylinder 4 stroke engine
  .grheat_time = SYSTIM_MAGS(180), //3 minutes
  .add_i1_sub = 1,
  .add_i2_sub = 2,
  .idlreg_captrange = 200, //200 min-1

  .idlent_timval = 150,
  .gasval_ontime = 500, //5 sec.

  //             114°  294°  474°   654°
  .tdc_angle = {3648, 9408, 15168, 20928, 0, 0, 0 ,0},
  .smp_angle = 66*32,  //66°
  .dwl_dead_time = 312, //1ms

  .sfc_tps_thrd = TPS_MAGNITUDE(3.0), //3%
  .evap_map_thrd = PRESSURE_MAGNITUDE(250.0),
  .ckps_skip_trig = 5, //skip 5 teeth

  .maninjpw_idl = 1, //use manual inj.PW correction on idling

  .oilpress_cut = 0, //do not cut ignition and fuel

  .tpsdot_mindt = 10000, //32ms
  .irr_k_load = ROUND(2.0*32),
  .irr_k_rpm = ROUND(2.0*32),

  .cold_eng_int = 1,
  .iacreg_period = 10, //100ms
  .iacreg_turn_on_temp = 200,

  .vent_maxband = 30,
  .pwron_time = SYSTIM_MAGS(0.1),   //0.1 sec
  .pwron_time1 = SYSTIM_MAGS(0.1),   //0.1 sec

  .ltft_mode = 0,      //LTFT is off
  .ltft_learn_clt = TEMPERATURE_MAGNITUDE(90.0), //90°C
  .ltft_cell_band = 51, //~20%
  .ltft_stab_time = 10, //!< 100ms
  .ltft_learn_grad = 13, //~0.05

  .pwrrelay_uni = 0x0F, //not used

  .ltft_learn_gpa = PRESSURE_MAGNITUDE(0.0),  //0 kPa
  .ltft_learn_gpd = PRESSURE_MAGNITUDE(0.0),  //0 kPa
  .ltft_neigh_rad = 15, //all cells
  .ltft_sigswt_num = 4, //4 successive switches

  .thrass_algo = 0,

  .btbaud_use = {1,0,0,0,0}, //use only 9600 because this is default (factory) baud rate used by Bluetooth

  .mapdot_mindt = 10000, //32ms

  .uart_silent = 0, //don't use silent mode by default

  .ltft_stab_str = 0, //not used, ltft_stab_time is used by default

  .fueldens_corr_use = 2, //use for both fuels

  .fts_source = 0, //use CTS+IAT model

  .tmrpmtc_mode = 0, //use standard calculation of RPM thresholds

  .vent_pwm_turnoff_hyst = TEMPERATURE_MAGNITUDE(0.5), //0.5°C

  .save_param_timeout = 3000, //30 seconds

  .fuelcut_vss_thrd = VSSSPEED_MAG(10.0), //10km/h

  .evap_on_vss_thrd = VSSSPEED_MAG(10.0), // 10km/h
  .evap_off_vss_thrd = VSSSPEED_MAG(7.0), // 7km/h
  .iac_onrunadd_vss_thrd = VSSSPEED_MAG(7.0), // 7km/h

  .iac_min_rpm_on_run = 1000, //1000 min-1

  /**reserved bytes*/
  {0}
 },

 /**Fill tables with default data */
 {
  {
   {'2','1','0','8','3',' ','S','t','a','n','d','a','r','d',' ',' '},                  //name of set
   //maps for ignition
   {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x04,0x08,0x0C,0x10,0x14,0x14,0x14},  //cranking map
   {0x14,0x14,0x14,0x14,0x14,0x14,0x17,0x1D,0x28,0x32,0x36,0x37,0x37,0x37,0x37,0x37},  //idling map
   {
   _REVARR16(
    {_AL16(0x04,0x05,0x06,0x07,0x0A,0x0C,0x10,0x15,0x1A,0x20,0x24,0x27,0x28,0x28,0x28,0x28)}, //working map
    {_AL16(0x04,0x05,0x06,0x08,0x0B,0x0D,0x10,0x16,0x1B,0x21,0x26,0x29,0x2A,0x2A,0x2A,0x2A)},
    {_AL16(0x04,0x05,0x08,0x08,0x0C,0x0E,0x12,0x18,0x1E,0x23,0x28,0x2A,0x2C,0x2C,0x2C,0x2C)},
    {_AL16(0x04,0x06,0x08,0x0A,0x0C,0x10,0x14,0x1B,0x21,0x26,0x28,0x2A,0x2C,0x2C,0x2C,0x2D)},
    {_AL16(0x06,0x07,0x0A,0x0C,0x0E,0x12,0x18,0x20,0x26,0x2A,0x2C,0x2D,0x2E,0x2E,0x2F,0x30)},
    {_AL16(0x08,0x08,0x0A,0x0E,0x12,0x16,0x1E,0x27,0x2D,0x2F,0x30,0x31,0x33,0x34,0x35,0x36)},
    {_AL16(0x0A,0x0B,0x0D,0x10,0x14,0x1B,0x24,0x2D,0x32,0x33,0x35,0x37,0x39,0x3A,0x3A,0x3B)},
    {_AL16(0x0C,0x10,0x12,0x15,0x1A,0x21,0x29,0x32,0x36,0x37,0x39,0x3C,0x3E,0x40,0x40,0x40)},
    {_AL16(0x10,0x16,0x18,0x1C,0x22,0x28,0x2E,0x36,0x3A,0x3B,0x3D,0x3F,0x41,0x44,0x44,0x44)},
    {_AL16(0x16,0x1C,0x1E,0x22,0x27,0x2E,0x34,0x3A,0x3D,0x3E,0x40,0x42,0x44,0x48,0x48,0x48)},
    {_AL16(0x1C,0x21,0x22,0x26,0x2A,0x31,0x38,0x3F,0x41,0x42,0x44,0x45,0x48,0x4A,0x4A,0x4B)},
    {_AL16(0x1E,0x22,0x24,0x27,0x2B,0x33,0x3B,0x42,0x45,0x45,0x47,0x48,0x4A,0x4C,0x4C,0x4C)},
    {_AL16(0x20,0x24,0x26,0x29,0x2E,0x36,0x3D,0x45,0x47,0x47,0x48,0x49,0x4A,0x4C,0x4C,0x4D)},
    {_AL16(0x20,0x24,0x27,0x2B,0x31,0x37,0x3F,0x45,0x47,0x48,0x49,0x49,0x4B,0x4D,0x4D,0x4D)},
    {_AL16(0x1E,0x22,0x26,0x2C,0x31,0x39,0x40,0x46,0x48,0x4A,0x4A,0x4B,0x4D,0x4E,0x4E,0x4E)},
    {_AL16(0x1E,0x21,0x25,0x29,0x2F,0x36,0x3F,0x45,0x49,0x4B,0x4C,0x4D,0x4F,0x4F,0x4F,0x4F)})
   },
   {0x22,0x1C,0x19,0x16,0x13,0x0F,0x0C,0x0A,0x07,0x05,0x02,0x00,0x00,0xFD,0xF6,0xEC},  //Correction of ignition timing vs CLT

   //Maps for fuel injection:
   /**Fill VE lookup table, value can be in range 0...1.99 */
   {//  600       720        840       990      1170      1380     1650      1950      2310      2730       3210      3840      4530      5370      6360      7500 (min-1)
   _REVARR16(
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, //16
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, //15
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, //14
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, //13
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, //12
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, //11
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, //10
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, // 9
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, // 8
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, // 7
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, // 6
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, // 5
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, // 4
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, // 3
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, // 2
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}) // 1
   },
   /**Fill AFR lookup table, value can be in range 8.1...22.0 */
   {//  600       720        840       990      1170      1380     1650      1950      2310      2730       3210      3840      4530      5370      6360      7500 (min-1)
   _REVARR16(
    {_AL16(_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7))}, //16
    {_AL16(_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7))}, //15
    {_AL16(_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7))}, //14
    {_AL16(_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7))}, //13
    {_AL16(_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7))}, //12
    {_AL16(_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7))}, //11
    {_AL16(_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7))}, //10
    {_AL16(_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7))}, // 9
    {_AL16(_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7))}, // 8
    {_AL16(_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7))}, // 7
    {_AL16(_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7))}, // 6
    {_AL16(_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7))}, // 5
    {_AL16(_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7))}, // 4
    {_AL16(_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7))}, // 3
    {_AL16(_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7))}, // 2
    {_AL16(_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7))}) // 1
   },
   /**Fill injection timing map, value can be in range -360...360 */
   {//  600       720        840       990      1170      1380     1650      1950      2310      2730       3210      3840      4530      5370      6360      7500 (min-1)
   _REVARR16(
    {_PACK16(_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0))}, //16
    {_PACK16(_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0))}, //15
    {_PACK16(_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0))}, //14
    {_PACK16(_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0))}, //13
    {_PACK16(_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0))}, //12
    {_PACK16(_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0))}, //11
    {_PACK16(_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0))}, //10
    {_PACK16(_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0))}, // 9
    {_PACK16(_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0))}, // 8
    {_PACK16(_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0))}, // 7
    {_PACK16(_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0))}, // 6
    {_PACK16(_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0))}, // 5
    {_PACK16(_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0))}, // 4
    {_PACK16(_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0))}, // 3
    {_PACK16(_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0))}, // 2
    {_PACK16(_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0))}) // 1
   },

   /**Fill cranking pulse width lookup table, time in ms vs coolant temperature */
   {// -30           -20        -10           0        10            20         30          40
    _DLV(10.20), _DLV(9.30), _DLV(8.60), _DLV(8.00), _DLV(7.25), _DLV(6.65), _DLV(6.00), _DLV(5.4),
    //  50            60         70          80        90           100        110         120
    _DLV(4.80), _DLV(4.20), _DLV(3.75), _DLV(3.30), _DLV(2.90), _DLV(2.55), _DLV(2.30), _DLV(2.15)
   },
   /**Fill warmup enrichment lookup table, factor(0...199%) vs coolant temperature */
   {// -30        -20        -10          0        10          20         30         40
    _WLV(160), _WLV(156), _WLV(152), _WLV(147), _WLV(142), _WLV(137), _WLV(133), _WLV(129),
    //  50         60         70         80        90          100        110        120
    _WLV(126), _WLV(123), _WLV(120), _WLV(117), _WLV(114), _WLV(110), _WLV(105), _WLV(100)
   },
   /**Fill injector dead time lookup table (Siemens DEKA ZMZ6354), time in ms vs voltage */
   {//  5.4       5.8        6.2        6.6         7.0        7.4        7.8        8.2
    _DLV(5.80),_DLV(4.50),_DLV(3.80),_DLV(3.30),_DLV(3.00),_DLV(2.75),_DLV(2.50),_DLV(2.30),
    //  8.6       9.0        9.4        9.8        10.2       10.6       11.0       11.4
    _DLV(2.12),_DLV(2.00),_DLV(1.90),_DLV(1.82),_DLV(1.75),_DLV(1.67),_DLV(1.60),_DLV(1.50),
    // 11.8      12.2       12.6       13.0        13.4       13.8       14.2       14.6
    _DLV(1.45),_DLV(1.40),_DLV(1.34),_DLV(1.30),_DLV(1.26),_DLV(1.22),_DLV(1.18),_DLV(1.16),
    // 15.0      15.4       15.8        16.2       16.6        17.0      17.4       17.8
    _DLV(1.12),_DLV(1.10),_DLV(1.06),_DLV(1.04),_DLV(1.02),_DLV(0.99),_DLV(0.97),_DLV(0.96)
   },
   /**Fill IAC/PWM open-loop position lookup table (run mode) */
   {// -30           -20        -10           0        10            20         30          40
    _CLV(46.0), _CLV(41.0), _CLV(38.0), _CLV(35.0), _CLV(33.0), _CLV(31.5), _CLV(30.4), _CLV(29.0),
    //  50            60         70          80        90           100        110         120
    _CLV(28.0), _CLV(26.7), _CLV(25.5), _CLV(24.4), _CLV(23.4), _CLV(22.2), _CLV(21.0), _CLV(20.0)
   },
   /**Fill IAC/PWM open-loop position lookup table (cranking mode) */
   {// -30           -20        -10           0        10            20         30          40
    _CLV(65.0), _CLV(60.0), _CLV(55.0), _CLV(50.0), _CLV(46.0), _CLV(42.0), _CLV(38.2), _CLV(36.0),
    //  50            60         70          80        90           100        110         120
    _CLV(33.3), _CLV(31.5), _CLV(30.0), _CLV(28.5), _CLV(27.0), _CLV(25.7), _CLV(24.8), _CLV(24.5)
   },
   /**Fill values of the AE's TPS lookup table, range is -55...199% */
   {
    AE_TPS_V(0.0), AE_TPS_V(0.0), AE_TPS_V(0.0), AE_TPS_V(0.0), AE_TPS_V(1.0), AE_TPS_V(20.0), AE_TPS_V(120.0), AE_TPS_V(160.0)
   },
   /**Fill bins of the AE's TPS lookup table, range is -1000...+1000% / 1s */
   {
    AE_TPS_B(-500.0), AE_TPS_B(-200.0), AE_TPS_B(-100.0), AE_TPS_B(-50.0), AE_TPS_B(50.0), AE_TPS_B(100.0), AE_TPS_B(200.0), AE_TPS_B(500.0)
   },
   /**Fill values of the AE's RPM lookup table, %, range is 0...199% */
   {
    AE_RPM_V(100.0), AE_RPM_V(70.0), AE_RPM_V(40.0), AE_RPM_V(30.0)
   },
   /**Fill bins of the AE's RPM lookup table, range is 0...25000min-1 */
   {
    AE_RPM_B(1000), AE_RPM_B(2000), AE_RPM_B(4000), AE_RPM_B(8000)
   },
   /**Fill afterstart enrichment lookup table, range is 0...199%, value indicates how many % will be added to fuel */
   {// -30           -20        -10           0        10            20         30          40
    _ASE(45.0), _ASE(41.0), _ASE(38.0), _ASE(35.0), _ASE(33.0), _ASE(31.5), _ASE(30.4), _ASE(29.0),
    //  50            60         70          80        90           100        110         120
    _ASE(28.0), _ASE(26.7), _ASE(25.5), _ASE(24.4), _ASE(23.4), _ASE(22.2), _ASE(21.0), _ASE(20.0)
   },


   /**Fill idle target RPM vs coolant temperature */
   {//  -30      -20       -10        0         10        20        30        40        50        60       70        80        90       100        110      120
    _IR(1600),_IR(1570),_IR(1550),_IR(1500),_IR(1350),_IR(1300),_IR(1190),_IR(1100),_IR(1100),_IR(1100),_IR(1100),_IR(1100),_IR(1100),_IR(1110),_IR(1130),_IR(1160)
   },

   /**Fill idle regulator's rigidity function*/
   {
    _IRR(0.25), _IRR(0.35), _IRR(0.5), _IRR(1.5), _IRR(3.0), _IRR(4.0), _IRR(4.5), _IRR(4.75)
   },

   /**Fill EGO AFR curve look up table*/
   {_ER(22.03),_ER(19.17),_ER(17.24),_ER(16.35),_ER(15.68),_ER(15.25),_ER(15.00),_ER(14.70),_ER(14.30),_ER(14.00),_ER(13.70),_ER(13.18),_ER(12.50),_ER(11.77),_ER(10.90),_ER(10.00),
   ROUND(0.01 / ADC_DISCRETE), ROUND(1.00 / ADC_DISCRETE)},

   /**Fill mixture correction weight vs TPS position*/
   {//   0         4          8        12        16        20       24        28        32        36        40        44       48         52       56         60
    _MW(0.996),_MW(0.51),_MW(0.40),_MW(0.32),_MW(0.28),_MW(0.24),_MW(0.22),_MW(0.17),_MW(0.14),_MW(0.12),_MW(0.09),_MW(0.07),_MW(0.05),_MW(0.04),_MW(0.02),_MW(0.00),
    _MWX(0.00),_MWX(60.0) //0-60% TPS
   },

   /**Fill mixture correction vs IAC position*/
   {//   0        10         20         30        40          50          60        70
    _MC(1.000),_MC(1.000),_MC(1.000),_MC(1.000),_MC(1.000),_MC(1.000),_MC(1.000),_MC(1.000),
    _MCX(00.0),_MCX(70.0) //0-70% IAC pos.
   },

   /**Fill IAT/CLT correction lookup table*/
   {//
    _IC(1.000),_IC(0.500),_IC(0.160),_IC(0.08),_IC(0.050),_IC(0.035),_IC(0.025),_IC(0.010),
    _ICX(5000),_ICX(960000) //
   },

   //  600       720        840       990      1170      1380     1650      1950      2310      2730       3210      3840      4530      5370      6360      7500 (min-1)
   {_TP(0.00),_TP(0.00),_TP(0.00),_TP(0.00),_TP(0.00),_TP(0.00),_TP(0.00),_TP(0.00),_TP(0.00),_TP(0.00),_TP(0.00),_TP(0.00),_TP(0.00),_TP(0.00),_TP(0.00),_TP(0.00)},

   /**Fill gas temperature's correction lookup table, coefficient vs gas temperature */
   {// -30         -20        -10         0        10            20         30        40
    _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00),
    //  50          60         70        80        90           100        110       120
    _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00)
   },

   /**Fill gas pressure's correction lookup table, coefficient vs gas pressure */
   {
    _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00),_ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00),
    _ER(1.00), _GPSX(100.0), _GPSX(400.0),
   },

   /**Fill air temperature's correction lookup table (inj.), coefficient vs air temperature */
   {// -30         -20        -10         0        10            20         30        40
    _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00),
    //  50          60         70        80        90           100        110       120
    _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00)
   },

   /**Fill PWM1 duty lookup table, value can be in range 0...100% */
   {//  600       720        840       990      1170      1380     1650      1950      2310      2730       3210      3840      4530      5370      6360      7500 (min-1)
#ifdef SPLIT_ANGLE
   _REVARR16(
    {_AL16(0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14)}, //split angle
    {_AL16(0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14)},
    {_AL16(0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14)},
    {_AL16(0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14)},
    {_AL16(0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14)},
    {_AL16(0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14)},
    {_AL16(0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14)},
    {_AL16(0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14)},
    {_AL16(0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14)},
    {_AL16(0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14)},
    {_AL16(0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14)},
    {_AL16(0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14)},
    {_AL16(0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14)},
    {_AL16(0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14)},
    {_AL16(0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14)},
    {_AL16(0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14)})
#else
   _REVARR16(
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, //16
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, //15
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, //14
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, //13
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, //12
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, //11
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, //10
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, // 9
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, // 8
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, // 7
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, // 6
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, // 5
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, // 4
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, // 3
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, // 2
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}) // 1
#endif
   },
#ifdef SPLIT_ANGLE
   1,  //split
#else
   0,  //duty
#endif
   /**Fill PWM2 duty lookup table, value can be in range 0...100% */
   {//  600       720        840       990      1170      1380     1650      1950      2310      2730       3210      3840      4530      5370      6360      7500 (min-1)
   _REVARR16(
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, //16
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, //15
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, //14
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, //13
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, //12
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, //11
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, //10
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, // 9
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, // 8
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, // 7
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, // 6
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, // 5
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, // 4
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, // 3
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, // 2
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}) // 1
   },
   {0x22,0x1C,0x19,0x16,0x13,0x0F,0x0C,0x0A,0x07,0x05,0x02,0x00,0x00,0xFD,0xF6,0xEC},  //Correction of ignition timing vs CLT (for idling)

   {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, //IAC pos. correction vs MAT

   /**Fill secondary VE lookup table, value can be in range 0...1.99 */
   {//  600       720        840       990      1170      1380     1650      1950      2310      2730       3210      3840      4530      5370      6360      7500 (min-1)
   _REVARR16(
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, //16
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, //15
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, //14
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, //13
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, //12
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, //11
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, //10
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, // 9
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, // 8
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, // 7
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, // 6
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, // 5
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, // 4
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, // 3
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, // 2
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}) // 1
   },

   //  600       720        840       990      1170      1380     1650      1950      2310      2730       3210      3840      4530      5370      6360      7500 (min-1)
   {_TP(50.0),_TP(50.0),_TP(50.0),_TP(50.0),_TP(50.0),_TP(50.0),_TP(50.0),_TP(50.0),_TP(50.0),_TP(50.0),_TP(50.0),_TP(50.0),_TP(50.0),_TP(50.0),_TP(50.0),_TP(50.0)},

   //Fill inj. multiplier map
   //     1           2           3           4           5           6           7           8
   { _IML(1.00), _IML(1.00), _IML(1.00), _IML(1.00), _IML(1.00), _IML(1.00), _IML(1.00), _IML(1.00)},

   //Fill inj. addition map
   //    1           2           3           4           5           6           7           8
   {_IAD(0.00), _IAD(0.00), _IAD(0.00), _IAD(0.00), _IAD(0.00), _IAD(0.00), _IAD(0.00), _IAD(0.00)},

   /**Fill values of the AE's MAP lookup table, range is -55...199% */
   {
    AE_TPS_V(0.0), AE_TPS_V(0.0), AE_TPS_V(0.0), AE_TPS_V(0.0), AE_TPS_V(1.0), AE_TPS_V(20.0), AE_TPS_V(120.0), AE_TPS_V(160.0)
   },
   /**Fill bins of the AE's MAP lookup table, range is -1000...+1000kPa / 1s */
   {
    AE_TPS_B(-500.0), AE_TPS_B(-200.0), AE_TPS_B(-100.0), AE_TPS_B(-50.0), AE_TPS_B(50.0), AE_TPS_B(100.0), AE_TPS_B(200.0), AE_TPS_B(500.0)
   },

   //Fill throttle assist map
   {_CLV(0), _CLV(0), _CLV(0), _CLV(0), _CLV(0), _CLV(0), _CLV(0), _CLV(0), _CLV(0), _CLV(0), _CLV(0), _CLV(0), _CLV(0), _CLV(0), _CLV(0), _CLV(0)},

   /**reserved bytes */
   {0},
   .checksum = 0
  },

  {
   {'2','1','0','8','3',' ','D','y','n','a','m','i','c',' ',' ',' '},                  //name of set
   //tables used for ignition
   {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x04,0x08,0x0C,0x10,0x14,0x14,0x14},  //start map
   {0x14,0x14,0x14,0x14,0x14,0x14,0x17,0x1D,0x28,0x32,0x36,0x37,0x37,0x37,0x37,0x37},  //ÕÕ êàðòà
   {
   _REVARR16(
    {_AL16(0x09,0x09,0x09,0x09,0x11,0x14,0x1A,0x20,0x26,0x24,0x30,0x33,0x3B,0x43,0x45,0x45)}, //work map
    {_AL16(0x09,0x09,0x09,0x09,0x11,0x14,0x1A,0x20,0x26,0x24,0x30,0x33,0x3B,0x43,0x45,0x45)},
    {_AL16(0x09,0x09,0x09,0x09,0x11,0x14,0x1A,0x20,0x26,0x24,0x30,0x33,0x3B,0x43,0x45,0x45)},
    {_AL16(0x09,0x09,0x09,0x09,0x13,0x14,0x1A,0x20,0x26,0x26,0x32,0x33,0x3F,0x43,0x45,0x45)},
    {_AL16(0x09,0x09,0x09,0x0A,0x13,0x16,0x1C,0x22,0x28,0x2F,0x39,0x3B,0x3F,0x47,0x49,0x49)},
    {_AL16(0x09,0x09,0x09,0x0B,0x15,0x28,0x2C,0x35,0x3A,0x41,0x42,0x43,0x45,0x45,0x4B,0x4B)},
    {_AL16(0x0B,0x0D,0x17,0x1D,0x25,0x2D,0x33,0x38,0x3F,0x45,0x4A,0x48,0x48,0x48,0x4A,0x4C)},
    {_AL16(0x16,0x1A,0x22,0x26,0x2D,0x33,0x39,0x3D,0x46,0x48,0x4D,0x4A,0x4A,0x4A,0x4A,0x50)},
    {_AL16(0x21,0x27,0x2F,0x35,0x33,0x36,0x3D,0x41,0x49,0x4B,0x4F,0x4C,0x4C,0x4C,0x4A,0x52)},
    {_AL16(0x28,0x2E,0x3A,0x3A,0x37,0x37,0x3D,0x45,0x4C,0x4D,0x4F,0x4F,0x4F,0x52,0x52,0x56)},
    {_AL16(0x2E,0x38,0x3E,0x40,0x38,0x37,0x45,0x49,0x4E,0x4F,0x51,0x50,0x50,0x54,0x58,0x58)},
    {_AL16(0x30,0x3E,0x42,0x40,0x38,0x3D,0x47,0x4F,0x52,0x50,0x4F,0x4E,0x4E,0x54,0x54,0x5A)},
    {_AL16(0x32,0x40,0x46,0x48,0x48,0x49,0x4B,0x4E,0x51,0x52,0x4E,0x4D,0x4B,0x54,0x58,0x58)},
    {_AL16(0x2E,0x3C,0x40,0x42,0x46,0x41,0x47,0x4B,0x4E,0x4E,0x4E,0x4D,0x45,0x54,0x54,0x56)},
    {_AL16(0x28,0x32,0x36,0x38,0x36,0x39,0x43,0x49,0x4B,0x4E,0x48,0x48,0x49,0x50,0x54,0x54)},
    {_AL16(0x24,0x28,0x28,0x28,0x30,0x35,0x3F,0x47,0x4B,0x4E,0x47,0x46,0x48,0x4C,0x50,0x50)})
   },
   {0x22,0x1C,0x19,0x16,0x13,0x0F,0x0C,0x0A,0x07,0x05,0x02,0x00,0x00,0xFD,0xF6,0xEC},  //coolant temperature correction map

   //tables used for injection
   /**Fill VE lookup table, value can be in range 0...1.99 */
   {//  600       720        840       990      1170      1380     1650      1950      2310      2730       3210      3840      4530      5370      6360      7500 (min-1)
   _REVARR16(
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, //16
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, //15
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, //14
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, //13
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, //12
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, //11
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, //10
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, // 9
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, // 8
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, // 7
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, // 6
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, // 5
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, // 4
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, // 3
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, // 2
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}) // 1
   },
   /**Fill AFR lookup table, value can be in range 8.1...22.0 */
   {//  600       720        840       990      1170      1380     1650      1950      2310      2730       3210      3840      4530      5370      6360      7500 (min-1)
   _REVARR16(
    {_AL16(_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7))}, //16
    {_AL16(_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7))}, //15
    {_AL16(_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7))}, //14
    {_AL16(_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7))}, //13
    {_AL16(_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7))}, //12
    {_AL16(_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7))}, //11
    {_AL16(_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7))}, //10
    {_AL16(_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7))}, // 9
    {_AL16(_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7))}, // 8
    {_AL16(_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7))}, // 7
    {_AL16(_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7))}, // 6
    {_AL16(_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7))}, // 5
    {_AL16(_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7))}, // 4
    {_AL16(_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7))}, // 3
    {_AL16(_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7))}, // 2
    {_AL16(_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7))}) // 1
   },
   /**Fill injection timing map, value can be in range -360...360 */
   {//  600       720        840       990      1170      1380     1650      1950      2310      2730       3210      3840      4530      5370      6360      7500 (min-1)
   _REVARR16(
    {_PACK16(_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0))}, //16
    {_PACK16(_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0))}, //15
    {_PACK16(_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0))}, //14
    {_PACK16(_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0))}, //13
    {_PACK16(_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0))}, //12
    {_PACK16(_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0))}, //11
    {_PACK16(_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0))}, //10
    {_PACK16(_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0))}, // 9
    {_PACK16(_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0))}, // 8
    {_PACK16(_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0))}, // 7
    {_PACK16(_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0))}, // 6
    {_PACK16(_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0))}, // 5
    {_PACK16(_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0))}, // 4
    {_PACK16(_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0))}, // 3
    {_PACK16(_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0))}, // 2
    {_PACK16(_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0))}) // 1
   },

   /**Fill cranking pulse width lookup table, time in ms vs coolant temperature */
   {// -30           -20        -10           0        10            20         30          40
    _DLV(10.20), _DLV(9.30), _DLV(8.60), _DLV(8.00), _DLV(7.25), _DLV(6.65), _DLV(6.00), _DLV(5.4),
    //  50            60         70          80        90           100        110         120
    _DLV(4.80), _DLV(4.20), _DLV(3.75), _DLV(3.30), _DLV(2.90), _DLV(2.55), _DLV(2.30), _DLV(2.15)
   },
   /**Fill warmup enrichment lookup table, factor(0...199%) vs coolant temperature */
   {// -30        -20        -10          0        10          20         30         40
    _WLV(160), _WLV(156), _WLV(152), _WLV(147), _WLV(142), _WLV(137), _WLV(133), _WLV(129),
    //  50         60         70         80        90          100        110       120
    _WLV(126), _WLV(123), _WLV(120), _WLV(117), _WLV(114), _WLV(110), _WLV(105), _WLV(100)
   },
   /**Fill injector dead time lookup table (Siemens DEKA ZMZ6354), time in ms vs voltage */
   {//  5.4       5.8        6.2        6.6         7.0        7.4        7.8        8.2
    _DLV(5.80),_DLV(4.50),_DLV(3.80),_DLV(3.30),_DLV(3.00),_DLV(2.75),_DLV(2.50),_DLV(2.30),
    //  8.6       9.0        9.4        9.8        10.2       10.6       11.0       11.4
    _DLV(2.12),_DLV(2.00),_DLV(1.90),_DLV(1.82),_DLV(1.75),_DLV(1.67),_DLV(1.60),_DLV(1.50),
    // 11.8      12.2       12.6       13.0        13.4       13.8       14.2       14.6
    _DLV(1.45),_DLV(1.40),_DLV(1.34),_DLV(1.30),_DLV(1.26),_DLV(1.22),_DLV(1.18),_DLV(1.16),
    // 15.0      15.4       15.8        16.2       16.6        17.0      17.4       17.8
    _DLV(1.12),_DLV(1.10),_DLV(1.06),_DLV(1.04),_DLV(1.02),_DLV(0.99),_DLV(0.97),_DLV(0.96)
   },
   /**Fill IAC/PWM open-loop position lookup table (run mode) */
   {// -30           -20        -10           0        10            20         30          40
    _CLV(46.0), _CLV(41.0), _CLV(38.0), _CLV(35.0), _CLV(33.0), _CLV(31.5), _CLV(30.4), _CLV(29.0),
    //  50            60         70          80        90           100        110         120
    _CLV(28.0), _CLV(26.7), _CLV(25.5), _CLV(24.4), _CLV(23.4), _CLV(22.2), _CLV(21.0), _CLV(20.0)
   },
   /**Fill IAC/PWM open-loop position lookup table (cranking mode) */
   {// -30           -20        -10           0        10            20         30          40
    _CLV(65.0), _CLV(60.0), _CLV(55.0), _CLV(50.0), _CLV(46.0), _CLV(42.0), _CLV(38.2), _CLV(36.0),
    //  50            60         70          80        90           100        110         120
    _CLV(33.3), _CLV(31.5), _CLV(30.0), _CLV(28.5), _CLV(27.0), _CLV(25.7), _CLV(24.8), _CLV(24.5)
   },
   /**Fill values of the AE's TPS lookup table, range is -55...199% */
   {
    AE_TPS_V(0.0), AE_TPS_V(0.0), AE_TPS_V(0.0), AE_TPS_V(0.0), AE_TPS_V(1.0), AE_TPS_V(20.0), AE_TPS_V(120.0), AE_TPS_V(160.0)
   },
   /**Fill bins of the AE's TPS lookup table, range is -1000...+1000% / 1s */
   {
    AE_TPS_B(-500.0), AE_TPS_B(-200.0), AE_TPS_B(-100.0), AE_TPS_B(-50.0), AE_TPS_B(50.0), AE_TPS_B(100.0), AE_TPS_B(200.0), AE_TPS_B(500.0)
   },
   /**Fill values of the AE's RPM lookup table, %, range is 0...199% */
   {
    AE_RPM_V(100.0), AE_RPM_V(70.0), AE_RPM_V(40.0), AE_RPM_V(30.0)
   },
   /**Fill bins of the AE's RPM lookup table, range is 0...25000min-1 */
   {
    AE_RPM_B(1000), AE_RPM_B(2000), AE_RPM_B(4000), AE_RPM_B(8000)
   },
   /**Fill afterstart enrichment lookup table, range is 0...199%, value indicates how many % will be added to fuel */
   {// -30           -20        -10           0        10            20         30          40
    _ASE(45.0), _ASE(41.0), _ASE(38.0), _ASE(35.0), _ASE(33.0), _ASE(31.5), _ASE(30.4), _ASE(29.0),
    //  50            60         70          80        90           100        110         120
    _ASE(28.0), _ASE(26.7), _ASE(25.5), _ASE(24.4), _ASE(23.4), _ASE(22.2), _ASE(21.0), _ASE(20.0)
   },

   /**Fill idle target RPM vs coolant temperature */
   {//  -30      -20       -10        0         10        20        30        40        50        60       70        80        90       100        110      120
    _IR(1600),_IR(1570),_IR(1550),_IR(1500),_IR(1350),_IR(1300),_IR(1190),_IR(1100),_IR(1100),_IR(1100),_IR(1100),_IR(1100),_IR(1100),_IR(1110),_IR(1130),_IR(1160)
   },

   /**Fill idle regulator's rigidity function*/
   {
    _IRR(0.25), _IRR(0.35), _IRR(0.5), _IRR(1.5), _IRR(3.0), _IRR(4.0), _IRR(4.5), _IRR(4.75)
   },

   /**Fill EGO AFR curve looku up table*/
   {_ER(22.03),_ER(19.17),_ER(17.24),_ER(16.35),_ER(15.68),_ER(15.25),_ER(15.00),_ER(14.70),_ER(14.30),_ER(14.00),_ER(13.70),_ER(13.18),_ER(12.50),_ER(11.77),_ER(10.90),_ER(10.00),
   ROUND(0.01 / ADC_DISCRETE), ROUND(1.00 / ADC_DISCRETE)},

   /**Fill mixture correction weight vs TPS position*/
   {//   0         4          8        12        16        20       24        28        32        36        40        44       48         52       56         60
    _MW(0.996),_MW(0.51),_MW(0.40),_MW(0.32),_MW(0.28),_MW(0.24),_MW(0.22),_MW(0.17),_MW(0.14),_MW(0.12),_MW(0.09),_MW(0.07),_MW(0.05),_MW(0.04),_MW(0.02),_MW(0.00),
    _MWX(0.00),_MWX(60.0) //0-60% TPS
   },

   /**Fill mixture correction vs IAC position*/
   {//   0        10         20         30        40          50          60        70
    _MC(1.000),_MC(1.000),_MC(1.000),_MC(1.000),_MC(1.000),_MC(1.000),_MC(1.000),_MC(1.000),
    _MCX(00.0),_MCX(70.0) //0-70% IAC pos.
   },

   /**Fill IAT/CLT correction lookup table*/
   {//
    _IC(1.000),_IC(0.500),_IC(0.160),_IC(0.08),_IC(0.050),_IC(0.035),_IC(0.025),_IC(0.010),
    _ICX(5000),_ICX(960000) //
   },

   //  600       720        840       990      1170      1380     1650      1950      2310      2730       3210      3840      4530      5370      6360      7500 (min-1)
   {_TP(0.00),_TP(0.00),_TP(0.00),_TP(0.00),_TP(0.00),_TP(0.00),_TP(0.00),_TP(0.00),_TP(0.00),_TP(0.00),_TP(0.00),_TP(0.00),_TP(0.00),_TP(0.00),_TP(0.00),_TP(0.00)},

   /**Fill gas temperature's correction lookup table, coefficient vs gas temperature */
   {// -30         -20        -10         0        10            20         30        40
    _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00),
    //  50          60         70        80        90           100        110       120
    _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00)
   },

   /**Fill gas pressure's correction lookup table, coefficient vs gas pressure */
   {
    _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00),_ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00),
    _ER(1.00), _GPSX(100.0), _GPSX(400.0),
   },

   /**Fill air temperature's correction lookup table (inj.), coefficient vs air temperature */
   {// -30         -20        -10         0        10            20         30        40
    _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00),
    //  50          60         70        80        90           100        110       120
    _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00)
   },

   /**Fill PWM1 duty lookup table, value can be in range 0...100% */
   {//  600       720        840       990      1170      1380     1650      1950      2310      2730       3210      3840      4530      5370      6360      7500 (min-1)
#ifdef SPLIT_ANGLE
   _REVARR16(
    {_AL16(0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14)}, //split angle
    {_AL16(0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14)},
    {_AL16(0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14)},
    {_AL16(0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14)},
    {_AL16(0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14)},
    {_AL16(0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14)},
    {_AL16(0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14)},
    {_AL16(0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14)},
    {_AL16(0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14)},
    {_AL16(0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14)},
    {_AL16(0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14)},
    {_AL16(0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14)},
    {_AL16(0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14)},
    {_AL16(0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14)},
    {_AL16(0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14)},
    {_AL16(0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14)})
#else
   _REVARR16(
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, //16
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, //15
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, //14
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, //13
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, //12
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, //11
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, //10
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, // 9
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, // 8
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, // 7
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, // 6
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, // 5
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, // 4
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, // 3
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, // 2
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}) // 1
#endif
   },
#ifdef SPLIT_ANGLE
   1,  //split
#else
   0,  //duty
#endif
   /**Fill PWM2 duty lookup table, value can be in range 0...100% */
   {//  600       720        840       990      1170      1380     1650      1950      2310      2730       3210      3840      4530      5370      6360      7500 (min-1)
   _REVARR16(
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, //16
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, //15
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, //14
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, //13
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, //12
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, //11
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, //10
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, // 9
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, // 8
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, // 7
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, // 6
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, // 5
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, // 4
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, // 3
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, // 2
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}) // 1
   },
   {0x22,0x1C,0x19,0x16,0x13,0x0F,0x0C,0x0A,0x07,0x05,0x02,0x00,0x00,0xFD,0xF6,0xEC},  //Correction of ignition timing vs CLT (for idling)

   {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, //IAC pos. correction vs MAT

   /**Fill secondary VE lookup table, value can be in range 0...1.99 */
   {//  600       720        840       990      1170      1380     1650      1950      2310      2730       3210      3840      4530      5370      6360      7500 (min-1)
   _REVARR16(
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, //16
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, //15
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, //14
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, //13
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, //12
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, //11
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, //10
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, // 9
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, // 8
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, // 7
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, // 6
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, // 5
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, // 4
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, // 3
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, // 2
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}) // 1
   },

   //  600       720        840       990      1170      1380     1650      1950      2310      2730       3210      3840      4530      5370      6360      7500 (min-1)
   {_TP(50.0),_TP(50.0),_TP(50.0),_TP(50.0),_TP(50.0),_TP(50.0),_TP(50.0),_TP(50.0),_TP(50.0),_TP(50.0),_TP(50.0),_TP(50.0),_TP(50.0),_TP(50.0),_TP(50.0),_TP(50.0)},

   //Fill inj. multiplier map
   //     1           2           3           4           5           6           7           8
   { _IML(1.00), _IML(1.00), _IML(1.00), _IML(1.00), _IML(1.00), _IML(1.00), _IML(1.00), _IML(1.00)},

   //Fill inj. addition map
   //    1           2           3           4           5           6           7           8
   {_IAD(0.00), _IAD(0.00), _IAD(0.00), _IAD(0.00), _IAD(0.00), _IAD(0.00), _IAD(0.00), _IAD(0.00)},

   /**Fill values of the AE's MAP lookup table, range is -55...199% */
   {
    AE_TPS_V(0.0), AE_TPS_V(0.0), AE_TPS_V(0.0), AE_TPS_V(0.0), AE_TPS_V(1.0), AE_TPS_V(20.0), AE_TPS_V(120.0), AE_TPS_V(160.0)
   },
   /**Fill bins of the AE's MAP lookup table, range is -1000...+1000kPa / 1s */
   {
    AE_TPS_B(-500.0), AE_TPS_B(-200.0), AE_TPS_B(-100.0), AE_TPS_B(-50.0), AE_TPS_B(50.0), AE_TPS_B(100.0), AE_TPS_B(200.0), AE_TPS_B(500.0)
   },

   //Fill throttle assist map
   {_CLV(0), _CLV(0), _CLV(0), _CLV(0), _CLV(0), _CLV(0), _CLV(0), _CLV(0), _CLV(0), _CLV(0), _CLV(0), _CLV(0), _CLV(0), _CLV(0), _CLV(0), _CLV(0)},

   /**reserved bytes */
   {0},
   .checksum = 0
  },

  {
   {'C','l','a','s','s','i','c',' ','1','.','5','L',' ',' ',' ',' '},
   {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x04,0x08,0x0C,0x10,0x14,0x14,0x14},
   {0x14,0x14,0x14,0x14,0x14,0x14,0x17,0x1D,0x28,0x32,0x36,0x37,0x37,0x37,0x37,0x37},  //ignition timing map for idling mode
   {
   _REVARR16(
    {_AL16(0x0C,0x0C,0x0E,0x10,0x11,0x12,0x14,0x16,0x1B,0x1D,0x1F,0x21,0x22,0x24,0x27,0x27)},
    {_AL16(0x0E,0x0E,0x10,0x11,0x12,0x13,0x15,0x17,0x1C,0x20,0x24,0x25,0x26,0x26,0x29,0x29)},
    {_AL16(0x10,0x10,0x12,0x14,0x15,0x16,0x17,0x19,0x1E,0x22,0x26,0x27,0x29,0x28,0x2C,0x2C)},
    {_AL16(0x12,0x12,0x14,0x16,0x17,0x17,0x18,0x1D,0x21,0x26,0x2A,0x2B,0x2C,0x2D,0x2F,0x2F)},
    {_AL16(0x14,0x14,0x16,0x18,0x19,0x1B,0x23,0x2E,0x30,0x30,0x30,0x30,0x31,0x33,0x33,0x33)},
    {_AL16(0x16,0x16,0x18,0x1A,0x1B,0x23,0x2A,0x32,0x3C,0x3A,0x3A,0x3A,0x3B,0x38,0x38,0x38)},
    {_AL16(0x18,0x18,0x1A,0x1E,0x23,0x26,0x32,0x3A,0x3F,0x3F,0x3F,0x3F,0x3F,0x3F,0x3F,0x3F)},
    {_AL16(0x1B,0x1B,0x20,0x20,0x24,0x2C,0x36,0x40,0x43,0x43,0x43,0x43,0x43,0x46,0x46,0x46)},
    {_AL16(0x1E,0x1E,0x20,0x22,0x26,0x2C,0x38,0x42,0x46,0x47,0x47,0x47,0x47,0x48,0x49,0x49)},
    {_AL16(0x1E,0x1E,0x20,0x22,0x26,0x32,0x3C,0x45,0x47,0x48,0x48,0x48,0x49,0x4A,0x4B,0x4B)},
    {_AL16(0x1E,0x1E,0x20,0x22,0x26,0x34,0x40,0x48,0x4A,0x4A,0x4A,0x4A,0x4A,0x4C,0x4C,0x4C)},
    {_AL16(0x1E,0x24,0x28,0x30,0x36,0x3C,0x42,0x48,0x4C,0x4C,0x4C,0x4C,0x4C,0x4E,0x4E,0x4E)},
    {_AL16(0x1B,0x24,0x28,0x30,0x36,0x3C,0x42,0x48,0x4C,0x4C,0x4C,0x4C,0x4C,0x4E,0x4E,0x4E)},
    {_AL16(0x19,0x24,0x28,0x30,0x36,0x3C,0x42,0x46,0x4A,0x4A,0x4A,0x4A,0x4A,0x4B,0x4B,0x4B)},
    {_AL16(0x17,0x24,0x28,0x30,0x36,0x3C,0x42,0x46,0x4A,0x4A,0x4A,0x4A,0x4A,0x4A,0x4A,0x4A)},
    {_AL16(0x15,0x24,0x28,0x30,0x36,0x3C,0x42,0x43,0x43,0x43,0x43,0x44,0x45,0x49,0x49,0x49)})
   },
   {0x22,0x1C,0x19,0x16,0x13,0x0F,0x0C,0x0A,0x07,0x05,0x02,0x00,0x00,0xFD,0xF6,0xEC},

   //Maps for fuel injection:
   /**Fill VE lookup table, value can be in range 0...1.99 */
   {//  600       720        840       990      1170      1380     1650      1950      2310      2730       3210      3840      4530      5370      6360      7500 (min-1)
   _REVARR16(
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, //16
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, //15
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, //14
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, //13
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, //12
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, //11
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, //10
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, // 9
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, // 8
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, // 7
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, // 6
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, // 5
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, // 4
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, // 3
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, // 2
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}) // 1
   },
   /**Fill AFR lookup table, value can be in range 8.1...22.0 */
   {//  600       720        840       990      1170      1380     1650      1950      2310      2730       3210      3840      4530      5370      6360      7500 (min-1)
   _REVARR16(
    {_AL16(_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7))}, //16
    {_AL16(_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7))}, //15
    {_AL16(_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7))}, //14
    {_AL16(_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7))}, //13
    {_AL16(_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7))}, //12
    {_AL16(_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7))}, //11
    {_AL16(_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7))}, //10
    {_AL16(_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7))}, // 9
    {_AL16(_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7))}, // 8
    {_AL16(_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7))}, // 7
    {_AL16(_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7))}, // 6
    {_AL16(_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7))}, // 5
    {_AL16(_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7))}, // 4
    {_AL16(_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7))}, // 3
    {_AL16(_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7))}, // 2
    {_AL16(_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7))}) // 1
   },
   /**Fill injection timing map, value can be in range -360...360 */
   {//  600       720        840       990      1170      1380     1650      1950      2310      2730       3210      3840      4530      5370      6360      7500 (min-1)
   _REVARR16(
    {_PACK16(_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0))}, //16
    {_PACK16(_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0))}, //15
    {_PACK16(_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0))}, //14
    {_PACK16(_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0))}, //13
    {_PACK16(_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0))}, //12
    {_PACK16(_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0))}, //11
    {_PACK16(_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0))}, //10
    {_PACK16(_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0))}, // 9
    {_PACK16(_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0))}, // 8
    {_PACK16(_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0))}, // 7
    {_PACK16(_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0))}, // 6
    {_PACK16(_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0))}, // 5
    {_PACK16(_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0))}, // 4
    {_PACK16(_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0))}, // 3
    {_PACK16(_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0))}, // 2
    {_PACK16(_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0))}) // 1
   },

   /**Fill cranking pulse width lookup table, time in ms vs coolant temperature */
   {// -30           -20        -10           0        10            20         30          40
    _DLV(10.20), _DLV(9.30), _DLV(8.60), _DLV(8.00), _DLV(7.25), _DLV(6.65), _DLV(6.00), _DLV(5.4),
    //  50            60         70          80        90           100        110         120
    _DLV(4.80), _DLV(4.20), _DLV(3.75), _DLV(3.30), _DLV(2.90), _DLV(2.55), _DLV(2.30), _DLV(2.15)
   },
   /**Fill warmup enrichment lookup table, factor(0...199%) vs coolant temperature */
   {// -30        -20        -10          0        10          20         30        40
    _WLV(160), _WLV(156), _WLV(152), _WLV(147), _WLV(142), _WLV(137), _WLV(133), _WLV(129),
    //  50         60         70         80        90          100        110       120
    _WLV(126), _WLV(123), _WLV(120), _WLV(117), _WLV(114), _WLV(110), _WLV(105), _WLV(100)
   },
   /**Fill injector dead time lookup table (Siemens DEKA ZMZ6354), time in ms vs voltage */
   {//  5.4       5.8        6.2        6.6         7.0        7.4        7.8        8.2
    _DLV(5.80),_DLV(4.50),_DLV(3.80),_DLV(3.30),_DLV(3.00),_DLV(2.75),_DLV(2.50),_DLV(2.30),
    //  8.6       9.0        9.4        9.8        10.2       10.6       11.0       11.4
    _DLV(2.12),_DLV(2.00),_DLV(1.90),_DLV(1.82),_DLV(1.75),_DLV(1.67),_DLV(1.60),_DLV(1.50),
    // 11.8      12.2       12.6       13.0        13.4       13.8       14.2       14.6
    _DLV(1.45),_DLV(1.40),_DLV(1.34),_DLV(1.30),_DLV(1.26),_DLV(1.22),_DLV(1.18),_DLV(1.16),
    // 15.0      15.4       15.8        16.2       16.6        17.0      17.4       17.8
    _DLV(1.12),_DLV(1.10),_DLV(1.06),_DLV(1.04),_DLV(1.02),_DLV(0.99),_DLV(0.97),_DLV(0.96)
   },
   /**Fill IAC/PWM open-loop position lookup table (run mode) */
   {// -30           -20        -10           0        10            20         30          40
    _CLV(46.0), _CLV(41.0), _CLV(38.0), _CLV(35.0), _CLV(33.0), _CLV(31.5), _CLV(30.4), _CLV(29.0),
    //  50            60         70          80        90           100        110         120
    _CLV(28.0), _CLV(26.7), _CLV(25.5), _CLV(24.4), _CLV(23.4), _CLV(22.2), _CLV(21.0), _CLV(20.0)
   },
   /**Fill IAC/PWM open-loop position lookup table (cranking mode) */
   {// -30           -20        -10           0        10            20         30          40
    _CLV(65.0), _CLV(60.0), _CLV(55.0), _CLV(50.0), _CLV(46.0), _CLV(42.0), _CLV(38.2), _CLV(36.0),
    //  50            60         70          80        90           100        110         120
    _CLV(33.3), _CLV(31.5), _CLV(30.0), _CLV(28.5), _CLV(27.0), _CLV(25.7), _CLV(24.8), _CLV(24.5)
   },
   /**Fill values of the AE's TPS lookup table, range is -55...199% */
   {
    AE_TPS_V(0.0), AE_TPS_V(0.0), AE_TPS_V(0.0), AE_TPS_V(0.0), AE_TPS_V(1.0), AE_TPS_V(20.0), AE_TPS_V(120.0), AE_TPS_V(160.0)
   },
   /**Fill bins of the AE's TPS lookup table, range is -1000...+1000% / 1s */
   {
    AE_TPS_B(-500.0), AE_TPS_B(-200.0), AE_TPS_B(-100.0), AE_TPS_B(-50.0), AE_TPS_B(50.0), AE_TPS_B(100.0), AE_TPS_B(200.0), AE_TPS_B(500.0)
   },
   /**Fill values of the AE's RPM lookup table, %, range is 0...199% */
   {
    AE_RPM_V(100.0), AE_RPM_V(70.0), AE_RPM_V(40.0), AE_RPM_V(30.0)
   },
   /**Fill bins of the AE's RPM lookup table, range is 0...25000min-1 */
   {
    AE_RPM_B(1000), AE_RPM_B(2000), AE_RPM_B(4000), AE_RPM_B(8000)
   },
   /**Fill afterstart enrichment lookup table, range is 0...199%, value indicates how many % will be added to fuel */
   {// -30           -20        -10           0        10            20         30          40
    _ASE(45.0), _ASE(41.0), _ASE(38.0), _ASE(35.0), _ASE(33.0), _ASE(31.5), _ASE(30.4), _ASE(29.0),
    //  50            60         70          80        90           100        110         120
    _ASE(28.0), _ASE(26.7), _ASE(25.5), _ASE(24.4), _ASE(23.4), _ASE(22.2), _ASE(21.0), _ASE(20.0)
   },

   /**Fill idle target RPM vs coolant temperature */
   {//  -30      -20       -10        0         10        20        30        40        50        60       70        80        90       100        110      120
    _IR(1600),_IR(1570),_IR(1550),_IR(1500),_IR(1350),_IR(1300),_IR(1190),_IR(1100),_IR(1100),_IR(1100),_IR(1100),_IR(1100),_IR(1100),_IR(1110),_IR(1130),_IR(1160)
   },

   /**Fill idle regulator's rigidity function*/
   {
    _IRR(0.25), _IRR(0.35), _IRR(0.5), _IRR(1.5), _IRR(3.0), _IRR(4.0), _IRR(4.5), _IRR(4.75)
   },

   /**Fill EGO AFR curve looku up table*/
   {_ER(22.03),_ER(19.17),_ER(17.24),_ER(16.35),_ER(15.68),_ER(15.25),_ER(15.00),_ER(14.70),_ER(14.30),_ER(14.00),_ER(13.70),_ER(13.18),_ER(12.50),_ER(11.77),_ER(10.90),_ER(10.00),
   ROUND(0.01 / ADC_DISCRETE), ROUND(1.00 / ADC_DISCRETE)},

   /**Fill mixture correction weight vs TPS position*/
   {//   0         4          8        12        16        20       24        28        32        36        40        44       48         52       56         60
    _MW(0.996),_MW(0.51),_MW(0.40),_MW(0.32),_MW(0.28),_MW(0.24),_MW(0.22),_MW(0.17),_MW(0.14),_MW(0.12),_MW(0.09),_MW(0.07),_MW(0.05),_MW(0.04),_MW(0.02),_MW(0.00),
    _MWX(0.00),_MWX(60.0) //0-60% TPS
   },

   /**Fill mixture correction vs IAC position*/
   {//   0        10         20         30        40          50          60        70
    _MC(1.000),_MC(1.000),_MC(1.000),_MC(1.000),_MC(1.000),_MC(1.000),_MC(1.000),_MC(1.000),
    _MCX(00.0),_MCX(70.0) //0-70% IAC pos.
   },

   /**Fill IAT/CLT correction lookup table*/
   {//
    _IC(1.000),_IC(0.500),_IC(0.160),_IC(0.08),_IC(0.050),_IC(0.035),_IC(0.025),_IC(0.010),
    _ICX(5000),_ICX(960000) //
   },

   //  600       720        840       990      1170      1380     1650      1950      2310      2730       3210      3840      4530      5370      6360      7500 (min-1)
   {_TP(0.00),_TP(0.00),_TP(0.00),_TP(0.00),_TP(0.00),_TP(0.00),_TP(0.00),_TP(0.00),_TP(0.00),_TP(0.00),_TP(0.00),_TP(0.00),_TP(0.00),_TP(0.00),_TP(0.00),_TP(0.00)},

   /**Fill gas temperature's correction lookup table, coefficient vs gas temperature */
   {// -30         -20        -10         0        10            20         30        40
    _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00),
    //  50          60         70        80        90           100        110       120
    _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00)
   },

   /**Fill gas pressure's correction lookup table, coefficient vs gas pressure */
   {
    _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00),_ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00),
    _ER(1.00), _GPSX(100.0), _GPSX(400.0),
   },

   /**Fill air temperature's correction lookup table (inj.), coefficient vs air temperature */
   {// -30         -20        -10         0        10            20         30        40
    _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00),
    //  50          60         70        80        90           100        110       120
    _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00)
   },

   /**Fill PWM1 duty lookup table, value can be in range 0...100% */
   {//  600       720        840       990      1170      1380     1650      1950      2310      2730       3210      3840      4530      5370      6360      7500 (min-1)
#ifdef SPLIT_ANGLE
   _REVARR16(
    {_AL16(0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14)}, //split angle
    {_AL16(0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14)},
    {_AL16(0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14)},
    {_AL16(0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14)},
    {_AL16(0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14)},
    {_AL16(0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14)},
    {_AL16(0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14)},
    {_AL16(0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14)},
    {_AL16(0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14)},
    {_AL16(0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14)},
    {_AL16(0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14)},
    {_AL16(0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14)},
    {_AL16(0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14)},
    {_AL16(0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14)},
    {_AL16(0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14)},
    {_AL16(0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14)})
#else
   _REVARR16(
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, //16
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, //15
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, //14
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, //13
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, //12
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, //11
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, //10
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, // 9
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, // 8
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, // 7
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, // 6
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, // 5
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, // 4
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, // 3
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, // 2
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}) // 1
#endif
   },
#ifdef SPLIT_ANGLE
   1,  //split
#else
   0,  //duty
#endif
   /**Fill PWM2 duty lookup table, value can be in range 0...100% */
   {//  600       720        840       990      1170      1380     1650      1950      2310      2730       3210      3840      4530      5370      6360      7500 (min-1)
   _REVARR16(
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, //16
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, //15
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, //14
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, //13
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, //12
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, //11
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, //10
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, // 9
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, // 8
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, // 7
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, // 6
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, // 5
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, // 4
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, // 3
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, // 2
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}) // 1
   },
   {0x22,0x1C,0x19,0x16,0x13,0x0F,0x0C,0x0A,0x07,0x05,0x02,0x00,0x00,0xFD,0xF6,0xEC},  //Correction of ignition timing vs CLT (for idling)

   {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, //IAC pos. correction vs MAT

   /**Fill secondary VE lookup table, value can be in range 0...1.99 */
   {//  600       720        840       990      1170      1380     1650      1950      2310      2730       3210      3840      4530      5370      6360      7500 (min-1)
   _REVARR16(
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, //16
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, //15
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, //14
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, //13
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, //12
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, //11
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, //10
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, // 9
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, // 8
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, // 7
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, // 6
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, // 5
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, // 4
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, // 3
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, // 2
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}) // 1
   },

   //  600       720        840       990      1170      1380     1650      1950      2310      2730       3210      3840      4530      5370      6360      7500 (min-1)
   {_TP(50.0),_TP(50.0),_TP(50.0),_TP(50.0),_TP(50.0),_TP(50.0),_TP(50.0),_TP(50.0),_TP(50.0),_TP(50.0),_TP(50.0),_TP(50.0),_TP(50.0),_TP(50.0),_TP(50.0),_TP(50.0)},

   //Fill inj. multiplier map
   //     1           2           3           4           5           6           7           8
   { _IML(1.00), _IML(1.00), _IML(1.00), _IML(1.00), _IML(1.00), _IML(1.00), _IML(1.00), _IML(1.00)},

   //Fill inj. addition map
   //    1           2           3           4           5           6           7           8
   {_IAD(0.00), _IAD(0.00), _IAD(0.00), _IAD(0.00), _IAD(0.00), _IAD(0.00), _IAD(0.00), _IAD(0.00)},

   /**Fill values of the AE's MAP lookup table, range is -55...199% */
   {
    AE_TPS_V(0.0), AE_TPS_V(0.0), AE_TPS_V(0.0), AE_TPS_V(0.0), AE_TPS_V(1.0), AE_TPS_V(20.0), AE_TPS_V(120.0), AE_TPS_V(160.0)
   },
   /**Fill bins of the AE's MAP lookup table, range is -1000...+1000kPa / 1s */
   {
    AE_TPS_B(-500.0), AE_TPS_B(-200.0), AE_TPS_B(-100.0), AE_TPS_B(-50.0), AE_TPS_B(50.0), AE_TPS_B(100.0), AE_TPS_B(200.0), AE_TPS_B(500.0)
   },

   //Fill throttle assist map
   {_CLV(0), _CLV(0), _CLV(0), _CLV(0), _CLV(0), _CLV(0), _CLV(0), _CLV(0), _CLV(0), _CLV(0), _CLV(0), _CLV(0), _CLV(0), _CLV(0), _CLV(0), _CLV(0)},

   /**reserved bytes */
   {0},
   .checksum = 0
  },

  {//Ignition maps
   {'C','l','a','s','s','i','c',' ','1','.','6','L',' ',' ',' ',' '},
   {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x04,0x08,0x0C,0x10,0x14,0x14,0x14},  //cranking map
   {0x14,0x14,0x14,0x14,0x14,0x14,0x17,0x1D,0x28,0x32,0x36,0x37,0x37,0x37,0x37,0x37},  //idling map
   {
   _REVARR16(
    {_AL16(0x14,0x15,0x17,0x19,0x1B,0x1D,0x20,0x24,0x29,0x2D,0x2F,0x31,0x33,0x35,0x20,0x20)}, //work map
    {_AL16(0x16,0x18,0x1A,0x1C,0x1E,0x20,0x23,0x27,0x2C,0x31,0x34,0x36,0x38,0x3A,0x20,0x20)},
    {_AL16(0x18,0x1A,0x1C,0x1E,0x21,0x24,0x28,0x2D,0x33,0x38,0x3C,0x3E,0x3F,0x41,0x20,0x20)},
    {_AL16(0x1A,0x1C,0x1E,0x21,0x24,0x28,0x2D,0x33,0x39,0x3F,0x43,0x45,0x46,0x47,0x20,0x20)},
    {_AL16(0x1C,0x1E,0x20,0x23,0x26,0x2B,0x31,0x39,0x40,0x46,0x49,0x4A,0x4B,0x4C,0x20,0x20)},
    {_AL16(0x1E,0x20,0x22,0x25,0x29,0x30,0x36,0x3E,0x46,0x4D,0x51,0x52,0x52,0x52,0x20,0x20)},
    {_AL16(0x20,0x22,0x24,0x28,0x2C,0x34,0x3A,0x42,0x4A,0x51,0x55,0x57,0x58,0x59,0x20,0x20)},
    {_AL16(0x22,0x24,0x28,0x2C,0x30,0x38,0x3E,0x46,0x4E,0x54,0x59,0x5B,0x5D,0x5E,0x1C,0x1C)},
    {_AL16(0x24,0x28,0x2C,0x30,0x34,0x3D,0x42,0x49,0x51,0x57,0x5C,0x5E,0x5F,0x5F,0x18,0x18)},
    {_AL16(0x26,0x2A,0x2E,0x32,0x38,0x40,0x44,0x4B,0x53,0x59,0x5E,0x5F,0x5F,0x5F,0x14,0x14)},
    {_AL16(0x28,0x2C,0x2E,0x32,0x3A,0x42,0x46,0x4C,0x54,0x5A,0x5E,0x5F,0x5F,0x5F,0x10,0x10)},
    {_AL16(0x28,0x2C,0x2E,0x32,0x3A,0x42,0x48,0x4C,0x54,0x5A,0x5E,0x5F,0x5F,0x5F,0x10,0x10)},
    {_AL16(0x24,0x24,0x24,0x24,0x2C,0x42,0x48,0x4C,0x54,0x5A,0x5C,0x5E,0x5F,0x5F,0x10,0x10)},
    {_AL16(0x24,0x24,0x24,0x24,0x2C,0x42,0x48,0x4C,0x54,0x5A,0x5C,0x5E,0x5F,0x5F,0x10,0x10)},
    {_AL16(0x24,0x24,0x24,0x24,0x2C,0x40,0x46,0x48,0x50,0x54,0x56,0x58,0x58,0x58,0x10,0x10)},
    {_AL16(0x24,0x24,0x24,0x24,0x2C,0x2C,0x2C,0x2C,0x2C,0x2C,0x2C,0x2C,0x2C,0x2C,0x10,0x10)})
   },
   {0x22,0x1C,0x19,0x16,0x13,0x0F,0x0C,0x0A,0x07,0x05,0x02,0x00,0x00,0xFD,0xF6,0xEC}, //CLT correction map

   //Fuel injection maps
   /**Fill VE lookup table, value can be in range 0...1.99 */
   {//  600       720        840       990      1170      1380     1650      1950      2310      2730       3210      3840      4530      5370      6360      7500 (min-1)
   _REVARR16(
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, //16
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, //15
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, //14
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, //13
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, //12
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, //11
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, //10
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, // 9
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, // 8
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, // 7
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, // 6
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, // 5
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, // 4
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, // 3
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, // 2
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}) // 1
   },
   /**Fill AFR lookup table, value can be in range 8.1...22.0 */
   {//  600       720        840       990      1170      1380     1650      1950      2310      2730       3210      3840      4530      5370      6360      7500 (min-1)
   _REVARR16(
    {_AL16(_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7))}, //16
    {_AL16(_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7))}, //15
    {_AL16(_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7))}, //14
    {_AL16(_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7))}, //13
    {_AL16(_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7))}, //12
    {_AL16(_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7))}, //11
    {_AL16(_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7))}, //10
    {_AL16(_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7))}, // 9
    {_AL16(_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7))}, // 8
    {_AL16(_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7))}, // 7
    {_AL16(_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7))}, // 6
    {_AL16(_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7))}, // 5
    {_AL16(_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7))}, // 4
    {_AL16(_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7))}, // 3
    {_AL16(_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7))}, // 2
    {_AL16(_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7))}) // 1
   },
   /**Fill injection timing map, value can be in range -360...360 */
   {//  600       720        840       990      1170      1380     1650      1950      2310      2730       3210      3840      4530      5370      6360      7500 (min-1)
   _REVARR16(
    {_PACK16(_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0))}, //16
    {_PACK16(_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0))}, //15
    {_PACK16(_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0))}, //14
    {_PACK16(_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0))}, //13
    {_PACK16(_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0))}, //12
    {_PACK16(_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0))}, //11
    {_PACK16(_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0))}, //10
    {_PACK16(_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0))}, // 9
    {_PACK16(_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0))}, // 8
    {_PACK16(_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0))}, // 7
    {_PACK16(_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0))}, // 6
    {_PACK16(_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0))}, // 5
    {_PACK16(_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0))}, // 4
    {_PACK16(_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0))}, // 3
    {_PACK16(_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0))}, // 2
    {_PACK16(_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0))}) // 1
   },

   /**Fill cranking pulse width lookup table, time in ms vs coolant temperature */
   {// -30           -20        -10           0        10            20         30          40
    _DLV(10.20), _DLV(9.30), _DLV(8.60), _DLV(8.00), _DLV(7.25), _DLV(6.65), _DLV(6.00), _DLV(5.4),
    //  50            60         70          80        90           100        110         120
    _DLV(4.80), _DLV(4.20), _DLV(3.75), _DLV(3.30), _DLV(2.90), _DLV(2.55), _DLV(2.30), _DLV(2.15)
   },
   /**Fill warmup enrichment lookup table, factor(0...199%) vs coolant temperature */
   {// -30        -20        -10          0        10           20         30        40
    _WLV(160), _WLV(156), _WLV(152), _WLV(147), _WLV(142), _WLV(137), _WLV(133), _WLV(129),
    //  50         60         70         80        90          100        110        120
    _WLV(126), _WLV(123), _WLV(120), _WLV(117), _WLV(114), _WLV(110), _WLV(105), _WLV(100)
   },
   /**Fill injector dead time lookup table (Siemens DEKA ZMZ6354), time in ms vs voltage */
   {//  5.4       5.8        6.2        6.6         7.0        7.4        7.8        8.2
    _DLV(5.80),_DLV(4.50),_DLV(3.80),_DLV(3.30),_DLV(3.00),_DLV(2.75),_DLV(2.50),_DLV(2.30),
    //  8.6       9.0        9.4        9.8        10.2       10.6       11.0       11.4
    _DLV(2.12),_DLV(2.00),_DLV(1.90),_DLV(1.82),_DLV(1.75),_DLV(1.67),_DLV(1.60),_DLV(1.50),
    // 11.8      12.2       12.6       13.0        13.4       13.8       14.2       14.6
    _DLV(1.45),_DLV(1.40),_DLV(1.34),_DLV(1.30),_DLV(1.26),_DLV(1.22),_DLV(1.18),_DLV(1.16),
    // 15.0      15.4       15.8        16.2       16.6        17.0      17.4       17.8
    _DLV(1.12),_DLV(1.10),_DLV(1.06),_DLV(1.04),_DLV(1.02),_DLV(0.99),_DLV(0.97),_DLV(0.96)
   },
   /**Fill IAC/PWM open-loop position lookup table (run mode) */
   {// -30           -20        -10           0        10            20         30          40
    _CLV(46.0), _CLV(41.0), _CLV(38.0), _CLV(35.0), _CLV(33.0), _CLV(31.5), _CLV(30.4), _CLV(29.0),
    //  50            60         70          80        90           100        110         120
    _CLV(28.0), _CLV(26.7), _CLV(25.5), _CLV(24.4), _CLV(23.4), _CLV(22.2), _CLV(21.0), _CLV(20.0)
   },
   /**Fill IAC/PWM open-loop position lookup table (cranking mode) */
   {// -30           -20        -10           0        10            20         30          40
    _CLV(65.0), _CLV(60.0), _CLV(55.0), _CLV(50.0), _CLV(46.0), _CLV(42.0), _CLV(38.2), _CLV(36.0),
    //  50            60         70          80        90           100        110         120
    _CLV(33.3), _CLV(31.5), _CLV(30.0), _CLV(28.5), _CLV(27.0), _CLV(25.7), _CLV(24.8), _CLV(24.5)
   },
   /**Fill values of the AE's TPS lookup table, range is -55...199% */
   {
    AE_TPS_V(0.0), AE_TPS_V(0.0), AE_TPS_V(0.0), AE_TPS_V(0.0), AE_TPS_V(1.0), AE_TPS_V(20.0), AE_TPS_V(120.0), AE_TPS_V(160.0)
   },
   /**Fill bins of the AE's TPS lookup table, range is -1000...+1000% / 1s */
   {
    AE_TPS_B(-500.0), AE_TPS_B(-200.0), AE_TPS_B(-100.0), AE_TPS_B(-50.0), AE_TPS_B(50.0), AE_TPS_B(100.0), AE_TPS_B(200.0), AE_TPS_B(500.0)
   },
   /**Fill values of the AE's RPM lookup table, %, range is 0...199% */
   {
    AE_RPM_V(100.0), AE_RPM_V(70.0), AE_RPM_V(40.0), AE_RPM_V(30.0)
   },
   /**Fill bins of the AE's RPM lookup table, range is 0...25000min-1 */
   {
    AE_RPM_B(1000), AE_RPM_B(2000), AE_RPM_B(4000), AE_RPM_B(8000)
   },
   /**Fill afterstart enrichment lookup table, range is 0...199%, value indicates how many % will be added to fuel */
   {// -30           -20        -10           0        10            20         30          40
    _ASE(45.0), _ASE(41.0), _ASE(38.0), _ASE(35.0), _ASE(33.0), _ASE(31.5), _ASE(30.4), _ASE(29.0),
    //  50            60         70          80        90           100        110         120
    _ASE(28.0), _ASE(26.7), _ASE(25.5), _ASE(24.4), _ASE(23.4), _ASE(22.2), _ASE(21.0), _ASE(20.0)
   },

   /**Fill idle target RPM vs coolant temperature */
   {//  -30      -20       -10        0         10        20        30        40        50        60       70        80        90       100        110      120
    _IR(1600),_IR(1570),_IR(1550),_IR(1500),_IR(1350),_IR(1300),_IR(1190),_IR(1100),_IR(1100),_IR(1100),_IR(1100),_IR(1100),_IR(1100),_IR(1110),_IR(1130),_IR(1160)
   },

   /**Fill idle regulator's rigidity function*/
   {
    _IRR(0.25), _IRR(0.35), _IRR(0.5), _IRR(1.5), _IRR(3.0), _IRR(4.0), _IRR(4.5), _IRR(4.75)
   },

   /**Fill EGO AFR curve looku up table*/
   {_ER(22.03),_ER(19.17),_ER(17.24),_ER(16.35),_ER(15.68),_ER(15.25),_ER(15.00),_ER(14.70),_ER(14.30),_ER(14.00),_ER(13.70),_ER(13.18),_ER(12.50),_ER(11.77),_ER(10.90),_ER(10.00),
   ROUND(0.01 / ADC_DISCRETE), ROUND(1.00 / ADC_DISCRETE)},

   /**Fill mixture correction weight vs TPS position*/
   {//   0         4          8        12        16        20       24        28        32        36        40        44       48         52       56         60
    _MW(0.996),_MW(0.51),_MW(0.40),_MW(0.32),_MW(0.28),_MW(0.24),_MW(0.22),_MW(0.17),_MW(0.14),_MW(0.12),_MW(0.09),_MW(0.07),_MW(0.05),_MW(0.04),_MW(0.02),_MW(0.00),
    _MWX(0.00),_MWX(60.0) //0-60% TPS
   },

   /**Fill mixture correction vs IAC position*/
   {//   0        10         20         30        40          50          60        70
    _MC(1.000),_MC(1.000),_MC(1.000),_MC(1.000),_MC(1.000),_MC(1.000),_MC(1.000),_MC(1.000),
    _MCX(00.0),_MCX(70.0) //0-70% IAC pos.
   },

   /**Fill IAT/CLT correction lookup table*/
   {//
    _IC(1.000),_IC(0.500),_IC(0.160),_IC(0.08),_IC(0.050),_IC(0.035),_IC(0.025),_IC(0.010),
    _ICX(5000),_ICX(960000) //
   },

   //  600       720        840       990      1170      1380     1650      1950      2310      2730       3210      3840      4530      5370      6360      7500 (min-1)
   {_TP(0.00),_TP(0.00),_TP(0.00),_TP(0.00),_TP(0.00),_TP(0.00),_TP(0.00),_TP(0.00),_TP(0.00),_TP(0.00),_TP(0.00),_TP(0.00),_TP(0.00),_TP(0.00),_TP(0.00),_TP(0.00)},

   /**Fill gas temperature's correction lookup table, coefficient vs gas temperature */
   {// -30         -20        -10         0        10            20         30        40
    _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00),
    //  50          60         70        80        90           100        110       120
    _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00)
   },

   /**Fill gas pressure's correction lookup table, coefficient vs gas pressure */
   {
    _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00),_ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00),
    _ER(1.00), _GPSX(100.0), _GPSX(400.0),
   },

   /**Fill air temperature's correction lookup table (inj.), coefficient vs air temperature */
   {// -30         -20        -10         0        10            20         30        40
    _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00),
    //  50          60         70        80        90           100        110       120
    _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00), _ER(1.00)
   },

   /**Fill PWM1 duty lookup table, value can be in range 0...100% */
   {//  600       720        840       990      1170      1380     1650      1950      2310      2730       3210      3840      4530      5370      6360      7500 (min-1)
#ifdef SPLIT_ANGLE
   _REVARR16(
    {_AL16(0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14)}, //split angle
    {_AL16(0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14)},
    {_AL16(0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14)},
    {_AL16(0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14)},
    {_AL16(0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14)},
    {_AL16(0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14)},
    {_AL16(0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14)},
    {_AL16(0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14)},
    {_AL16(0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14)},
    {_AL16(0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14)},
    {_AL16(0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14)},
    {_AL16(0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14)},
    {_AL16(0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14)},
    {_AL16(0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14)},
    {_AL16(0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14)},
    {_AL16(0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14)})
#else
   _REVARR16(
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, //16
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, //15
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, //14
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, //13
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, //12
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, //11
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, //10
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, // 9
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, // 8
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, // 7
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, // 6
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, // 5
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, // 4
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, // 3
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, // 2
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}) // 1
#endif
   },
#ifdef SPLIT_ANGLE
   1,  //split
#else
   0,  //duty
#endif
   /**Fill PWM2 duty lookup table, value can be in range 0...100% */
   {//  600       720        840       990      1170      1380     1650      1950      2310      2730       3210      3840      4530      5370      6360      7500 (min-1)
   _REVARR16(
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, //16
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, //15
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, //14
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, //13
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, //12
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, //11
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, //10
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, // 9
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, // 8
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, // 7
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, // 6
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, // 5
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, // 4
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, // 3
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}, // 2
    {_AL16(_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0),_PW(50.0))}) // 1
   },
   {0x22,0x1C,0x19,0x16,0x13,0x0F,0x0C,0x0A,0x07,0x05,0x02,0x00,0x00,0xFD,0xF6,0xEC},  //Correction of ignition timing vs CLT (for idling)

   {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, //IAC pos. correction vs MAT

   /**Fill secondary VE lookup table, value can be in range 0...1.99 */
   {//  600       720        840       990      1170      1380     1650      1950      2310      2730       3210      3840      4530      5370      6360      7500 (min-1)
   _REVARR16(
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, //16
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, //15
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, //14
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, //13
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, //12
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, //11
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, //10
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, // 9
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, // 8
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, // 7
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, // 6
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, // 5
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, // 4
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, // 3
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}, // 2
    {_PACK16(_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00))}) // 1
   },

   //  600       720        840       990      1170      1380     1650      1950      2310      2730       3210      3840      4530      5370      6360      7500 (min-1)
   {_TP(50.0),_TP(50.0),_TP(50.0),_TP(50.0),_TP(50.0),_TP(50.0),_TP(50.0),_TP(50.0),_TP(50.0),_TP(50.0),_TP(50.0),_TP(50.0),_TP(50.0),_TP(50.0),_TP(50.0),_TP(50.0)},

   //Fill inj. multiplier map
   //     1           2           3           4           5           6           7           8
   { _IML(1.00), _IML(1.00), _IML(1.00), _IML(1.00), _IML(1.00), _IML(1.00), _IML(1.00), _IML(1.00)},

   //Fill inj. addition map
   //    1           2           3           4           5           6           7           8
   {_IAD(0.00), _IAD(0.00), _IAD(0.00), _IAD(0.00), _IAD(0.00), _IAD(0.00), _IAD(0.00), _IAD(0.00)},

   /**Fill values of the AE's MAP lookup table, range is -55...199% */
   {
    AE_TPS_V(0.0), AE_TPS_V(0.0), AE_TPS_V(0.0), AE_TPS_V(0.0), AE_TPS_V(1.0), AE_TPS_V(20.0), AE_TPS_V(120.0), AE_TPS_V(160.0)
   },
   /**Fill bins of the AE's MAP lookup table, range is -1000...+1000kPa / 1s */
   {
    AE_TPS_B(-500.0), AE_TPS_B(-200.0), AE_TPS_B(-100.0), AE_TPS_B(-50.0), AE_TPS_B(50.0), AE_TPS_B(100.0), AE_TPS_B(200.0), AE_TPS_B(500.0)
   },

   //Fill throttle assist map
   {_CLV(0), _CLV(0), _CLV(0), _CLV(0), _CLV(0), _CLV(0), _CLV(0), _CLV(0), _CLV(0), _CLV(0), _CLV(0), _CLV(0), _CLV(0), _CLV(0), _CLV(0), _CLV(0)},

   /**reserved bytes */
   {0},
   .checksum = 0
  }
 },

 /**Size of this string must be equal to FW_SIGNATURE_INFO_SIZE!
  * Date in format Mmm dd yyyy.
  * Do not forget to write out same value of version into to the fw_version field of cd_data_t!
  */
 {"SECU-3 firmware v5.0. Build ["__DATE__"]       "},

 /**Version of this structure - 0.0*/
 0x00,

 /**Size of all data for checking */
 (sizeof(fw_data_t) - sizeof(cd_data_t)),

 /**Contains check sum for whole firmware */
 0x0000
};

/**Firmware information */
PGM_DECLARE(uint8_t fwinfo[FWINFOSIZE]) =
 {0x53,0x45,0x43,0x55,0x2d,0x33,0x20,0x46,0x69,0x72,0x6d,0x77,0x61,0x72,0x65,0x2c,
  0x20,0x43,0x6f,0x70,0x79,0x72,0x69,0x67,0x68,0x74,0x20,0x28,0x43,0x29,0x20,0x32,
  0x30,0x30,0x37,0x20,0x41,0x2e,0x20,0x53,0x68,0x61,0x62,0x65,0x6c,0x6e,0x69,0x6b,
  0x6f,0x76,0x2c,0x20,0x68,0x74,0x74,0x70,0x3a,0x2f,0x2f,0x73,0x65,0x63,0x75,0x2d,
  0x33,0x2e,0x6f,0x72,0x67};

