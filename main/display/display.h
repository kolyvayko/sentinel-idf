#ifndef DISPLAY_H
#define DISPLAY_H

#include <ssd1306.h>

ssd1306_handle_t init_display();
void display_show_data(ssd1306_handle_t d, int val1, int val2);

#endif