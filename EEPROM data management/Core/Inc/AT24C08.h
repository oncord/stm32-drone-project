#ifndef INC_AT24C08_H_
#define INC_AT24C08_H_

#include "main.h"

extern I2C_HandleTypeDef hi2c;

void AT24C08_Page_Write(unsigned char page, unsigned char *data, unsigned char len);
void AT24C08_Page_Read(unsigned char page, unsigned char *data, unsigned char len);
void EP_PID_Gain_Write(unsigned char id, float PGain, float IGain, float DGain);
unsigned char EP_PID_Gain_Read(unsigned char id, float *PGain, float *IGain, float *DGain);

typedef union _Parser {
	unsigned char byte[4];
	float f;
} Parser;

#endif /* INC_AT24C08_H_ */
