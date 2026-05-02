
/*
 * IpsDisplay.h
 *
 *  Created on: Jan 25, 2018
 *      Author: iltis
 */

#pragma once

#include "math/vector_3d_fwd.h"
#include "math/Units.h"

#include <string>
#include <cstdint>

typedef enum e_sreens { INIT_DISPLAY_NULL, INIT_DISPLAY_RETRO=1, INIT_DISPLAY_FLARM=2, INIT_DISPLAY_GLOAD=4, INIT_DISPLAY_HORIZON=8 } e_screens_t;
extern int screens_init;

class AdaptUGC;
class PolarGauge;
class WindIndicator;
class WindIcon;
class McCready;
class S2FBar;
class Battery;
class Temperature;
class Altimeter;
class MultiGauge;
class CruiseStatus;
class FlapsBox;
class Quaternion;
class Connection;

// fixme needs a home
rad_t getHeading();
rad_t getWCA();

// Some geometry helper
struct Point;
using Vector2d = Point;

struct Point {
    Point() = default;
    constexpr Point(int16_t x, int16_t y) : x(x), y(y) {}
    int16_t x=0, y=0;
    Point operator+(const Point &p) const;
    Point& operator+=(const Point &p) { x += p.x; y += p.y; return *this; }
    Point& operator+=(int16_t val) { x += val; y += val; return *this; }
    Point operator-(const Point &p) const;
    Point& operator-=(const Point &p) { x -= p.x; y -= p.y; return *this; }
    Point& operator-=(int16_t val) { x -= val; y -= val; return *this; }
    // Point operator*(const Point &p) const;
    // Point& operator*=(const Point &p) { x *= p.x; y *= p.y; return *this; }
    Point rotate(rad_t alpha) const;
    static void scaleNshift(const Point *pts, int n, Point shift, float scale, Point *ret);
    static void rotate(const Point *pts, int n, int16_t ad2, Point *ret);
    static Point centralProjection(const vector_f &p, float focus); // todo should go into vector class, but needs Point for return value
    int dot(const Vector2d &v) const { return x * v.x + y * v.y; }
};

struct BoundingBox {
    Point pmin;
    Point pmax;
    constexpr BoundingBox() : pmin(Point(INT16_MAX, INT16_MAX)), pmax(Point(INT16_MIN, INT16_MIN)) {}
    constexpr BoundingBox(Point p1, Point P2) : pmin(p1), pmax(P2) {}
    constexpr void initialize(Point p) { pmin = pmax = p; }
    void add(Point p);
    void add(const Point *pts, int n);
    void enlarge(int16_t val);
    bool isIn(Point p) const;
};

// Hesse form of a 2d straight line: Normal x Pxy + d = 0
struct Line {
    Line() = default;
    Line(const Quaternion &q, bool respect_mbox = false);
    float _nx = 0;
    float _ny = 0;
    float _d = 0;
    float fct(Point p) const; // the Hesse function evaluated at point p
    Point intersect(Point p1, Point p2) const;
    bool operator==(const Line &r) const;
    bool similar(const Line &r) const;
    bool isDefined() const { return _nx != 0 || _ny != 0; }
    inline float getD() const { return _d; }
    Point mapToHorizon(Point p) const;
    Point limitToScreen(Point p, bool respect_mbox) const;
    float getDistance(Point p) const { return fct(p); }
};

//
// Routines that manipulate the display contents must always be called from the DrawDisplay context!
// Functions that are safe to be called from any context are marked.
//
class IpsDisplay {
public:
	IpsDisplay( AdaptUGC *aucg );
	~IpsDisplay();
	static void setupDisplay();
	static void setGlobalColors();
	static void writeText( int line, const char *text );
	static void writeText( int line, std::string &text );
	static void drawDisplay();

    static void clipPolygonByLine(const Point *poly, int n, const Line &l, Point *above, int *na, Point *below, int *nb);
    static void drawPolygon(Point *pts, int n);
    static void drawPolyFrame(Point *pts, int n);

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

  private:
    static PolarGauge *MAINgauge;
    static PolarGauge *WNDgauge;
    static WindIcon *WNDicon;
    static McCready *MCgauge;
    static S2FBar *S2FBARgauge;
    static Battery *BATgauge;
    static Altimeter *ALTgauge;
    static MultiGauge *TOPgauge;
    static CruiseStatus *VCSTATgauge;
    static FlapsBox *FLAPSgauge;
    static Temperature* OATgauge;
    static Connection* CONNgauge;

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
