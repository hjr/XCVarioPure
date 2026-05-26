
#pragma once

#include <eglib.h>
#include <cstdint>
#include <cstring>
#include <algorithm>

// later we want to get rid of UGC, so lets add all needed API definitions here

union ucg_color_t
{
    struct {
        uint8_t r;
        uint8_t g;
        uint8_t b;
    };
	uint8_t color[3];             /* 0: Red, 1: Green, 2: Blue */
    constexpr ucg_color_t() : color{0, 0, 0} {}
    constexpr ucg_color_t(uint8_t r_, uint8_t g_, uint8_t b_) : color{r_, g_, b_} {}
    void fadeTo(const ucg_color_t& target, float fade) {
        r = ifade(target.r, fade);
        g = ifade(target.g, fade);
        b = ifade(target.b, fade);
    } 
    static constexpr uint8_t ifade(int target, float fade){
        return uint8_t(255 - std::clamp(int((256 - target) * fade), 0, 255)); // be able to fade to 255 as well
        
    };
};

constexpr color_t to_color(const ucg_color_t& ucg) noexcept
{
    return {.r = ucg.r, .g = ucg.g, .b = ucg.b};
}

constexpr ucg_color_t to_ucg(const color_t& c) noexcept
{
    ucg_color_t result{};
    result.r = c.r;
    result.g = c.g;
    result.b = c.b;
    return result;
}

#define UCG_DRAW_UPPER_RIGHT EGLIB_DRAW_UPPER_RIGHT
#define UCG_DRAW_UPPER_LEFT  EGLIB_DRAW_UPPER_LEFT
#define UCG_DRAW_LOWER_LEFT  EGLIB_DRAW_LOWER_LEFT
#define UCG_DRAW_LOWER_RIGHT EGLIB_DRAW_LOWER_RIGHT
#define UCG_DRAW_ALL         EGLIB_DRAW_ALL

#define UCG_PRINT_DIR_LR 0x00
#define UCG_PRINT_DIR_TD 0x01
#define UCG_PRINT_DIR_RL 0x02
#define UCG_PRINT_DIR_BU 0x03

typedef enum _e_font_mode { UCG_FONT_MODE_TRANSPARENT, UCG_FONT_MODE_SOLID } e_font_mode;


typedef enum _fonts_enum {
	// UCG_FONT_9x15B_MF,
	// UCG_FONT_NCENR14_HR,
	UCG_FONT_FUB11_TR,
	UCG_FONT_FUB11_HR,
	UCG_FONT_FUB14_HR,
	// UCG_FONT_FUB14_HF,
	// UCG_FONT_FUB17_HF,
	UCG_FONT_FUB20_HN,
	UCG_FONT_FUB20_HR,
	// UCG_FONT_FUB20_HF,
	// UCG_FONT_FUB25_HR,
	UCG_FONT_FUB25_HF,
	// UCG_FONT_FUR25_HF,
	UCG_FONT_FUB35_HN,
	// UCG_FONT_FUB35_HR,
	// UCG_FONT_FUR14_HF,
	// UCG_FONT_FUR20_HF,
	// UCG_FONT_PROFONT22_MR,
	UCG_FONT_FUB25_HN,
	UCG_FONT_FUB11_HN,
	UCG_FONT_FUB14_HN
	// EGLIB_FONT_FREE_SANSBOLD_66,
} e_fonts_enum;

// to be activated as soon as ucg.h is replaced by AdaptUGC.h

// extern const uint8_t ucg_font_9x15B_mf[];
// extern const uint8_t ucg_font_ncenR14_hr[];
extern const uint8_t ucg_font_fub11_tr[];
extern const uint8_t ucg_font_fub11_hr[];
extern const uint8_t ucg_font_fub14_hn[];
extern const uint8_t ucg_font_fub14_hr[];
// extern const uint8_t ucg_font_fub14_hf[];
// extern const uint8_t ucg_font_fur14_hf[];
// extern const uint8_t ucg_font_fub17_hf[];
extern const uint8_t ucg_font_fub20_hn[];
extern const uint8_t ucg_font_fub20_hr[];
// extern const uint8_t ucg_font_fub20_hf[];
// extern const uint8_t ucg_font_fur20_hf[];
// extern const uint8_t ucg_font_fub25_hr[];
extern const uint8_t ucg_font_fub25_hf[];
// extern const uint8_t ucg_font_fur25_hf[];
extern const uint8_t ucg_font_fub35_hn[];
// extern const uint8_t ucg_font_fub35_hr[];
extern const uint8_t ucg_font_profont22_mr[];
extern const uint8_t ucg_font_fub25_hn[];
// extern const uint8_t ucg_font_fub11_hn[];
// extern const uint8_t eglib_font_free_sansbold_66[];


class AdaptUGC
{
public:
	// init
	void begin();
	int16_t getDisplayWidth() const;
	int16_t getDisplayHeight() const;
	inline void undoClipRange() { eglib_undoClipRange(eglib);}
	// color
	inline void setColor( uint8_t idx, uint8_t r, uint8_t g, uint8_t b ) { eglib_SetIndexColor(eglib, idx, r, g, b); }
	inline void setColor( uint8_t r, uint8_t g, uint8_t b ) { eglib_SetIndexColor(eglib, 0, r, g, b); }

	// graphics
	inline void drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1)  { eglib_DrawLine(eglib, x0, y0, x1, y1); }
	inline void drawBox(int16_t x, int16_t y, int16_t w, int16_t h)  { eglib_DrawBox(eglib, x, y, w, h); }
	inline void drawFrame(int16_t x, int16_t y, int16_t w, int16_t h)  { eglib_DrawFrame(eglib, x, y, w, h); }
	inline void drawHLine(int16_t x, int16_t y, int16_t len)  { eglib_DrawHLine(eglib, x, y, len); }
	inline void drawVLine(int16_t x, int16_t y, int16_t len)  { eglib_DrawVLine(eglib, x, y, len); }
	inline void drawPixel(int16_t x, int16_t y)  { eglib_DrawPixel(eglib, x, y); }
	inline void drawPixelColor(int16_t x, int16_t y, ucg_color_t color)  { eglib_DrawPixelColor(eglib, x, y, to_color(color)); }
	inline void drawRBox(int16_t x, int16_t y, int16_t w, int16_t h, int16_t r)  { eglib_DrawRoundBox(eglib, x, y, w, h, r); }
	inline void drawRFrame(int16_t x, int16_t y, int16_t w, int16_t h, int16_t r)  { eglib_DrawRoundFrame(eglib, x, y, w, h, r); }
	inline void drawTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2)  { eglib_DrawFilledTriangle(eglib, x0, y0, x1, y1, x2, y2); }
	inline void drawTetragon(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, int16_t x3, int16_t y3)  { eglib_DrawTetragon(eglib, x0, y0, x1, y1, x2, y2, x3, y3); }
	inline void drawCircle(int16_t x, int16_t y, int16_t radius, uint8_t options=EGLIB_DRAW_ALL){ eglib_DrawCircle(eglib, x, y, radius, options); }
	inline void drawDisc(int16_t x, int16_t y, int16_t radius, uint8_t options){	eglib_DrawDisc(eglib, x, y, radius, options);	}

	// Text Printing
	size_t write(uint8_t c);
	size_t write(const uint8_t *buffer, size_t size);
	size_t printf(const char * format, ...)  __attribute__ ((format (printf, 2, 3)));
	inline size_t print(const char *str) { return write((const uint8_t *)str, strlen(str)); }
	inline size_t print(char c) { return write(c); }
	size_t print(long, int base=10);
	inline size_t print(int i, int base=10) { return print(long(i), base); }
	inline void setPrintPos(int16_t x, int16_t y) { eglib_print_xpos = x; eglib_print_ypos = y; };
	inline void setPrintDir(uint8_t d) { eglib_print_dir = d; }
	inline int16_t getCharWidth( const char c ) { return eglib_GetCharWidth(eglib, c); };
	inline int16_t getStrWidth( const char *s ) { return eglib_GetTextWidth(eglib, s); };
	inline int16_t IdxToTextWidth(const char *text, int16_t width_limit) { return eglib_IdxToTextWidth(eglib, text, width_limit); }

	// Font related
	void setFont(const uint8_t *f, bool filled=false );
	void setFontMode( uint8_t is_transparent ) {};  // no concept for transparent fonts in eglib, as it appears
	inline void setFontPosBottom() { eglib_setFontOrigin( eglib, FONT_BOTTOM ); }
	inline void setFontPosCenter() { eglib_setFontOrigin( eglib, FONT_MIDDLE ); }
	inline void setFontPosTop() { eglib_setFontOrigin( eglib, FONT_TOP ); }
	inline int16_t getFontAscent() { return eglib->drawing.font->ascent; }
	inline int16_t getFontDescent() { return eglib->drawing.font->descent; }
	inline int16_t getFontHeight() { return eglib->drawing.font->ascent + eglib->drawing.font->descent; }
	inline int16_t getFontLineSpace() { return eglib->drawing.font->line_space; }


	// scrolling, clipping, clear
	inline void clearScreen() { eglib_ClearScreen( eglib ); }
	inline void scrollLines(int16_t lines) {  eglib_scrollScreen( eglib, lines ); }                    // display driver function  todo
	inline void scrollSetMargins( int16_t top, int16_t bottom ) { eglib_setScrollMargins( eglib, top, bottom ); } // display driver function
	inline void setClipRange( int16_t x, int16_t y, int16_t w, int16_t h ) { eglib_setClipRange(eglib, x, y, w, h ); }

	// partial frame buffer add-on
	void startBuffering( int16_t x, int16_t y, int16_t w, int16_t h );
    void finishBuffering();

private:
	inline void advanceCursor( size_t delta );
	size_t printNumber(unsigned long n, uint8_t base);

	int16_t eglib_print_xpos = 0, eglib_print_ypos = 0;
	uint8_t eglib_print_dir = UCG_PRINT_DIR_LR;
	eglib_t * eglib;
};
