 /****************************************************************
 *       SECU-3  - An open source, free engine control unit
 *    Designed by Alexey A. Shabelnikov. Ukraine, Gorlovka 2007.
 *       Microprocessors systems - design & programming.
 *    contacts:
 *              http://secu-3.narod.ru
 *              ICQ: 405-791-931
 ****************************************************************/

#include "crc_lib.h"

#define                 P_16        0xA001     //полином


/* --- crc16() ----------------------------------------------------------------
*  Контрольная сумма crc16
*    BYTE *buf - указатель на байтовый буфер
*    short num - размер буфера в байтах
*  Возвращает
*    unsigned short crc16 буфера
* ----------------------------------------------------------------------------*/
WORD crc16( BYTE *buf, WORD num )
{
unsigned int i;
unsigned short crc = 0xffff;

  while ( num-- )
  { 
    crc ^= *buf++;
    i = 8;
    do
    { 
      if ( crc & 1 )
        crc = ( crc >> 1 ) ^ P_16;
      else
        crc >>= 1;
    } while ( --i );
  }  
  return( crc );
}


WORD crc16f(BYTE __flash *buf, WORD num )
{
unsigned int i;
unsigned short crc = 0xffff;

  while ( num-- )
  { 
    crc ^= *buf++;   
    i = 8;
    do
    { 
      if ( crc & 1 )
        crc = ( crc >> 1 ) ^ P_16;
      else
        crc >>= 1;
    } while ( --i );
  }
  
  return( crc );
}

