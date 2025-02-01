#ifndef __FONTS_H__
#define __FONTS_H__

#include "system.h"

typedef struct 
{
       unsigned char Index[2];	
       char Msk[32];
}typFNT_GB16; 

typedef struct 
{
       unsigned char Index[2];	
       char Msk[72];
}typFNT_GB24;

typedef struct 
{
       unsigned char Index[2];	
       char Msk[128];
}typFNT_GB32; 

extern const unsigned char asc2_1206[95][12];
extern const unsigned char asc2_1608[95][16];
extern const unsigned char asc2_2412[95][36];

extern typFNT_GB16 tfont16[];
extern typFNT_GB24 tfont24[];
extern typFNT_GB32 tfont32[];

u16 get_tfont16_size(void);
u16 get_tfont24_size(void);
u16 get_tfont32_size(void);

#endif	// __FONTS_H__

