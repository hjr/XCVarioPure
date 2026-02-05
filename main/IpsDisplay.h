
/*
 * IpsDisplay.h
 *
 *  Created on: Jan 25, 2018
 *      Author: iltis
 */

#pragma once

#include "math/vector_3d_fwd.h"

#include <MPU.h>
#include <string>
#include <cstdint>

typedef enum e_sreens { INIT_DISPLAY_NULL, INIT_DISPLAY_RETRO=1, INIT_DISPLAY_FLARM=2, INIT_DISPLAY_GLOAD=4, INIT_DISPLAY_HORIZON=8 } e_screens_t;
extern int screens_init;

class AdaptUGC;
class PolarGauge;
class WindIndicator;
class McCready;
class S2FBar;
class Battery;
class Temperature;
class Altimeter;
class MultiGauge;
class CruiseStatus;
class FlapsBox;
class Quaternion;

// fixme needs a home
float getHeading();


// Some geometry helper
struct Point {
    int16_t x, y;
    Point operator+(Point p) const;
    Point& operator+=(const Point &p) { x += p.x; y += p.y; return *this; }
    Point rotate(float alpha) const;
};

// Hesse form of a 2d straight line: Normal x Pxy + d = 0
struct Line {
    Line() = default;
    Line(Quaternion q, int16_t cx, int16_t cy);
    float _nx;
    float _ny;
    float _d;
    float fct(Point p);
    Point intersect(Point p1, Point p2) const;
    bool operator==(const Line &r) const;
    bool similar(const Line &r) const;
};

//
// Routines that manipulate the display contents must always be called from the DrawDisplay context!
// Functions that are safe to be called from any context are marked.
//
class IpsDisplay {
public:
	IpsDisplay( AdaptUGC *aucg );
	~IpsDisplay();
	static void begin();
	static void bootDisplay();
	static void setGlobalColors();
	static void writeText( int line, const char *text );
	static void writeText( int line, std::string &text );
	static void drawDisplay();

	static void drawLoadDisplay( float loadFactor );
	static void drawLoadDisplayTexts();
	static void initDisplay();
	static void clear();   // erase whole display
	static void redrawValues();
    // the following may be called from any context (!)
	static void setBottomDirty();
	static void setCruiseChanged();
    static void setQnhChanged();

	static inline AdaptUGC *getDisplay() { return ucg; };
	static AdaptUGC *ucg;

    static void clipRectByLine(Point *rect, Line &l, Point *above, int *na, Point *below, int *nb);
    static void drawPolygon(Point *pts, int n);
    static Point projectToDisplayPlane(const vector_f &obj, float focus);
    static Point clipToScreenCenter(Point p);

  private:
    static PolarGauge *MAINgauge;
    static PolarGauge *WNDgauge;
    static McCready *MCgauge;
    static S2FBar *S2FBARgauge;
    static Battery *BATgauge;
    static Altimeter *ALTgauge;
    static MultiGauge *TOPgauge;
    static CruiseStatus *VCSTATgauge;
    static FlapsBox *FLAPSgauge;
    static Temperature* OATgauge;

    static int tick;

    // local variabls for dynamic display
    static Point screen_edge[4];

    static void drawBT();
    static void drawCable(int16_t x, int16_t y);
    static void drawWifi(int x, int y);
    static void drawConnection(int16_t x, int16_t y);
    static void initLoadDisplay();
};

extern IpsDisplay *Display;
