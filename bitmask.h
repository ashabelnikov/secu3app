 /****************************************************************
 *       SECU-3  - An open source, free engine control unit
 *    Designed by Alexey A. Shabelnikov. Ukraine, Gorlovka 2007.
 *       Microprocessors systems - design & programming.
 *    contacts:
 *              http://secu-3.narod.ru
 *              ICQ: 405-791-931
 ****************************************************************/

#define SETBIT(x,y)   ((x) |= (1<<(y)))    /* установка бита y в байте x*/
#define CLEARBIT(x,y) ((x) &= (~(1<<(y)))) /* сброс бита y в байте x*/
#define CHECKBIT(x,y) ((x) & (1<<(y)))     /* проверка бита y в байте x*/

#ifdef LITTLE_ENDIAN_DATA_FORMAT //little-endian data store format (Intel)
  #define GETBYTE(src,rel) *(((unsigned char*)&(src)+(rel)))
  #define SETBYTE(des,rel) *(((unsigned char*)&(des)+(rel)))
#else                           //big-endian data store format (Motorola) 
  #define GETBYTE(src,rel) *(((unsigned char*)&(src)+sizeof((src))-1-(rel)))
  #define SETBYTE(des,rel) *(((unsigned char*)&(des)+sizeof((des))-1-(rel)))
#endif  

