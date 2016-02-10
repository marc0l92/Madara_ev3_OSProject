#include "ev3lib_gui.h"
#include "ev3.h"
#include "ev3_light.h"

uint8_t debug_led_stauts = 0;

void init_gui(){
	ev3_init_lcd();
	ev3_init_button();
	ev3_init_led();
	
	ev3_clear_lcd();
}

void debug_message(char *message){
	#if ENABLE_DEBUG
	ev3_text_lcd_normal(0,0, message);
	printf("%s", message);
	if(debug_led_stauts){
		ev3_set_led(RIGHT_LED, GREEN_LED, 255);
		ev3_set_led(LEFT_LED, GREEN_LED, 0);
		debug_led_stauts = 0;
	}else{
		ev3_set_led(RIGHT_LED, GREEN_LED, 0);
		ev3_set_led(LEFT_LED, GREEN_LED, 255);
		debug_led_stauts = 1;
	}
	#endif
}

void error_message(char *message){
	printf("%s", message);
	fflush(stdout);
	ev3_clear_lcd();
	ev3_text_lcd_normal(0,0, message);
	while(1){
		set_light(LIT_RIGHT, LIT_RED);
		set_light(LIT_LEFT, LIT_OFF);
		sleep(1);
		set_light(LIT_RIGHT, LIT_OFF);
		set_light(LIT_LEFT, LIT_RED);
		sleep(1);
	}
}

void destroy_gui(){
	ev3_quit_led();
}
