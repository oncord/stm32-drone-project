#ifndef __FSIA6B_H
#define __FSIA6B_H
#ifdef __cplusplus
	extern "C" {
#endif
#include "main.h"

typedef struct _FSiA6B_iBus
{
	unsigned short RH;   // right stick horizontal
	unsigned short RV;   // right stick vertical
	unsigned short LV;	 // left stick vertical
	unsigned short LH; 	 // left stick horizontal
	unsigned short SwA;
	unsigned short SwB;
	unsigned short SwC;
	unsigned short SwD;
	unsigned short VrA;
	unsigned short VrB;

	unsigned char FailSafe;
} FSiA6B_iBus;

extern FSiA6B_iBus iBus;

unsigned char iBus_Check_CHKSUM(unsigned char* data, unsigned char len);
void iBus_Parse(unsigned char *data, FSiA6B_iBus* iBus);
void FSiA6B_UART5_Initialization(void);

#ifdef __cplusplus
}
#endif
#endif /* __FSIA6B_H */
