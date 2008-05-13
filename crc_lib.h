
#if !defined( BYTE )
  #define BYTE          unsigned char
#endif

#if !defined( WORD )
  #define WORD          unsigned short
#endif

WORD crc16( BYTE *buf, WORD num);
WORD crc16f(BYTE __flash *buf, WORD num);

