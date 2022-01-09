#ifndef DOT_MAX7219_DRIVER_H 
#define DOT_MAX7219_DRIVER_H

#include <stdint.h>

void initialiseDisplay(void);
void clearDisplay(void);
void Display_Test(uint8_t Test_font);

#endif /* DOT_MAX7219_DRIVER_H */