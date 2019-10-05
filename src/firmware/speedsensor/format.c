#include "format.h"

void formatuint1(char *dest, uint8_t val){
	static const char DEC_DIGITS[10] = "0123456789";
	*dest = DEC_DIGITS[val%10];
	//*dest = val%10 + 48;
}

void formatuint2(char *dest, uint8_t val){
	formatuint1(dest, val/10);
	formatuint1(dest + 1, val);
}

void formatuint4(char *dest, uint16_t val){
	formatuint2(dest, val/100);
	formatuint2(dest + 2, val%100);
}

void formatuint8(char *dest, uint32_t val){
	formatuint4(dest, val/10000);
	formatuint4(dest + 4, val%10000);
}

void formatuint16(char *dest, uint32_t val){
	formatuint8(dest, val/100000000);
	formatuint8(dest + 8, val);
}

