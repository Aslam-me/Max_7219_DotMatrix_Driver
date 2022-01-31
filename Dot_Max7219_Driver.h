#ifndef DOT_MAX7219_DRIVER_H 
#define DOT_MAX7219_DRIVER_H

#include <linux/types.h>
 
int Initialise_Display(void);
void clearDisplay(void);
void Display_Test(uint8_t Test_font);
void Rotate_Font(const uint8_t* In_data, uint8_t* Out_data);
void Display_data_for_4_Display_Test(const uint8_t* In_ptr);
void Scroll_text(const uint8_t* In_ptr);

#endif /* DOT_MAX7219_DRIVER_H */
