#include "eglib.h"
#include "eglib/display.h"
#include "eglib/drawing.h"

// shared buffer for 
//  - glyph rendering (max. ever needed is 27 * 48 * 3)
//  - partial frame buffering (gauge indicator needs max. 56 x 56 x 3, (length+width/2)^2 x 3)
// 
static uint8_t eglib_buffer[EGLIB_FRAMEBUFFER_SIZE]; // 14kb

void eglib_Init(eglib_t *eglib, const hal_t *hal_driver, void *hal_driver_config_ptr, const display_t *display_driver, void *display_config_ptr)
{
	eglib->hal.driver = hal_driver;
	eglib->hal.config_ptr = hal_driver_config_ptr;
	eglib->hal.comm_active = false;
	// eglib->hal.i2c_send_slave_addr = 0;

	eglib->display.driver = display_driver;
	eglib->display.config_ptr = display_config_ptr;
	eglib->display.refreshing = false;

	eglib->drawing.filled_mode = false;
	eglib->drawing.font_origin = FONT_BOTTOM;


	for(size_t i=0 ; i < sizeof(eglib->drawing.color_index) / sizeof(*eglib->drawing.color_index) ; i++) {
		eglib->drawing.color_index[i].r = 0;
		eglib->drawing.color_index[i].g = 0;
		eglib->drawing.color_index[i].b = 0;
	}

	eglib->hal.driver->init(eglib);
	eglib->display.driver->init(eglib);
    coordinate_t width, height;
    eglib->display.driver->get_dimension(eglib, &width, &height);
    eglib->do_buffer = false;
    eglib->drawing.clip_xmin = 0;
    eglib->drawing.clip_xmax = width;
    eglib->drawing.clip_ymin = 0;
    eglib->drawing.clip_ymax = height;
    eglib->drawing.buffer = eglib_buffer;
}

void eglib_SleepIn(eglib_t *eglib) {
	eglib->display.driver->sleep_in(eglib);
	eglib->hal.driver->sleep_in(eglib);
}

void eglib_SleepOut(eglib_t *eglib) {
	eglib->hal.driver->sleep_out(eglib);
	eglib->display.driver->sleep_out(eglib);
}
