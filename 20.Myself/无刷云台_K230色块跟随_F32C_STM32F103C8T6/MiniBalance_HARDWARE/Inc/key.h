#ifndef __KEY_H
#define __KEY_H

#include "sys.h"

#define KEY PAin(5)

//°´¼ü×´Ì¬Ã¶¾Ù
enum {
	key_stateless,
	single_click,
	double_click,
	long_click
};

void KEY_Init(void);
u8 KEY_Scan(u16 Frequency,u16 filter_times);

#endif

