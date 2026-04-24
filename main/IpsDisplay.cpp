/*
 * IpsDisplay.cpp
 *
 *  Created on: Oct 7, 2019
 *      Author: iltis
 *
 */

#include "IpsDisplay.h"

#include "screen/element/MultiGauge.h"
#include "screen/element/PolarGauge.h"
#include "screen/element/WindIndicator.h"
#include "screen/element/WindIcon.h"
#include "screen/element/McCready.h"
#include "screen/element/Temperature.h"
#include "screen/element/S2FBar.h"
#include "screen/element/Battery.h"
#include "screen/element/Altimeter.h"
#include "screen/element/CruiseStatus.h"
#include "screen/element/FlapsBox.h"
#include "screen/element/Connection.h"
#include "screen/MessageBox.h"

#include "sensor/imu/AccMPU6050.h"
#include "sensor/VarioFilter.h"
#include "math/Trigonometry.h"
#include "math/Floats.h"
#include "math/Quaternion.h"
#include "math/vector_3d_fwd.h"
#include "math/Units.h"
#include "Atmosphere.h"
#include "Flap.h"
#include "Flarm.h"
#include "setup/CruiseMode.h"
#include "wind/Wind.h"
#include "driver/time/AliveMonitor.h"
#include "setup/SetupNG.h"
#include "CenterAid.h"
#include "AdaptUGC.h"
#include "Colors.h"
#include "sensor.h"
#include "logdefnone.h"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <algorithm>

////////////////////////////
// types


// local variables
int screens_init = INIT_DISPLAY_NULL;

int   IpsDisplay::tick = 0;


// Average Vario data
PolarGauge* IpsDisplay::MAINgauge = nullptr;
PolarGauge* IpsDisplay::WNDgauge = nullptr;
WindIcon*  IpsDisplay::WNDicon = nullptr;
McCready*   IpsDisplay::MCgauge = nullptr;
S2FBar*     IpsDisplay::S2FBARgauge = nullptr;
Battery*    IpsDisplay::BATgauge = nullptr;
Altimeter*	IpsDisplay::ALTgauge = nullptr;
MultiGauge*	IpsDisplay::TOPgauge = nullptr;
CruiseStatus* IpsDisplay::VCSTATgauge = nullptr;
FlapsBox*   IpsDisplay::FLAPSgauge = nullptr;
Temperature* IpsDisplay::OATgauge = nullptr;
Connection* IpsDisplay::CONNgauge = nullptr;

int16_t DISPLAY_H;
int16_t DISPLAY_W;

IpsDisplay *Display = nullptr;

static int16_t AMIDY;
static int16_t AMIDX;
static int16_t AVGOFFX;
static int16_t UPPERYPOS;
static int16_t LOWERYPOS;
constexpr const float OPT_Y_IN = 0.262f;

static int16_t INNER_RIGHT_ALIGN = 170;
static int16_t LOAD_MPG_POS = 0;
static int16_t LOAD_MIAS_POS = 0;

AdaptUGC *IpsDisplay::ucg = 0;
Point IpsDisplay::screen_edge[4];

static union {
    struct {
        uint8_t bottom_dirty       : 1;
        uint8_t mode_dirty         : 1;
        uint8_t flp_speed_msg_shown : 1;
    };
    uint8_t packed;
} flags = {};


////////////////////////////
// Geometry helpers

Point Point::operator+(const Point &p) const {
    Point tmp = p;
    return tmp += *this;
}
Point Point::operator-(const Point &p) const {
    Point tmp = *this;
    return tmp -= p;
}
// Point Point::operator*(const Point &p) const {
//     Point tmp = *this;
//     return tmp *= p;
// }
// alpha [rad]
Point Point::rotate(rad_t alpha) const {
    float cos_a = fast_cos_rad(alpha);
    float sin_a = fast_sin_rad(alpha);
    return Point( x * cos_a - y * sin_a, x * sin_a + y * cos_a );
}
void Point::scaleNshift(const Point *pts, int n, Point shift, float scale, Point *ret)
{
    for ( int i = 0; i < n; i++ ) {
        ret[i].x = fast_iroundf(pts[i].x * scale) + shift.x;
        ret[i].y = fast_iroundf(pts[i].y * scale) + shift.y;
    }
}
void Point::rotate(const Point *pts, int n, int16_t ad2, Point *ret)
{
    float cos_a = fast_cos_idx(ad2);
    float sin_a = fast_sin_idx(ad2);
    for ( int i = 0; i < n; i++ ) {
        int16_t x = pts[i].x;
        int16_t y = pts[i].y;
        ret[i].x = fast_iroundf(x * cos_a - y * sin_a);
        ret[i].y = fast_iroundf(x * sin_a + y * cos_a);
    }
}
// NED to NAV frame with Z axes up
Point Point::centralProjection(const vector_f &obj, float focus) {
    // camera model: pinhole camera at origin, looking along X axis, display plane at focus distance
    focus *= fast_signf(obj.x); // mirror objects behind the plane
    // project to display plane
    if ( std::abs(obj.x) < 1e-4f ) {
        // avoid division by zero
        return Point( -10000, -10000 );
    }
    focus /= obj.x;
    return Point(fast_iroundf(focus * obj.y), fast_iroundf(focus * -obj.z));
}

// Hesse horizon line parameters in the gliders Y/Z plane
// quaternion q is attitude in NED frame, 
// cx/cy is the center of the display frame in pixel coordinates
// line result is directly in display coordinates with x to the right and y down,
// and d is the signed distance of the line to the center of the display
Line::Line(const Quaternion &q) { //, int16_t cx, int16_t cy) {
    // normal vector of the plane
    vector_f up = q.rotate(vector_f(0, 0, -1)); // Earth-Up ENU im Body frame
    _nx = up.y;
    _ny = up.z;
    _d = std::clamp(-up.x, -0.7f, 0.7f) * 100; // projection scale to visible range
    _d += _nx * DISPLAY_W/2 + _ny * DISPLAY_H/2; // offset to the middle of the screen
    float norm = sqrtf(_nx*_nx + _ny*_ny);
    _nx /= norm;
    _ny /= norm;
    _d  /= norm;
    // ESP_LOGI(FNAME, "normV %0.1f,%0.1f,%0.1f", _nx, _ny, _d);
}
// evaluate line function at point p
float Line::fct(Point p) const {
    return _nx * p.x + _ny * p.y - _d;
}
// compute intersection point of line with segment p1-p2
Point Line::intersect(Point p1, Point p2) const {
    int dx = p2.x - p1.x;
    int dy = p2.y - p1.y;
    float num = _d - (_nx * p1.x + _ny * p1.y);
    float den = _nx * dx + _ny * dy;
    float t = std::clamp(num / den, 0.f, 1.f); // prevents extrusion due to rounding
    Point ret;
    ret.x = fast_iroundf(dx * t + p1.x);
    ret.y = fast_iroundf(dy * t + p1.y);
    return ret;
}
bool Line::operator==(const Line &r) const
{
    return floatEqualFast(_nx, r._nx) && floatEqualFast(_ny, r._ny) && floatEqualFast(_d, r._d);
}
bool Line::similar(const Line &r) const
{
    return (floatEqualFastAbs(_nx, r._nx, 1e-2f)) && (floatEqualFastAbs(_ny, r._ny, 1e-2f)) && (floatEqualFastAbs(_d, r._d, 2.f));
}
Point Line::mapToHorizon(Point obj) const
{
    Point ret(DISPLAY_W/2, DISPLAY_H/2);
    float dist = _nx * ret.x + _ny * ret.y - _d; // Project screen reference
    // mapped to horizon line coordinates (P = Horizon_Mid + u·t + v·n)
    ret.x = ret.x - dist * _nx + obj.x * -_ny + obj.y * _nx;
    ret.y = ret.y - dist * _ny + obj.x * _nx + obj.y * _ny;
    return ret;
}

// limit target mapping to screen area, 
// returns a point on the screen border if the target is outside
Point Line::limitToScreen(Point p, bool respect_mbox) const
{
    // check p against boundaries
    const int16_t display_h = (respect_mbox ? (DISPLAY_H - MBOX->getBoxHeight()) : DISPLAY_H);
    if (p.x >= 0 && p.x < DISPLAY_W && p.y >= 0 && p.y < display_h) {
        return p;
    }

    // project screen reference
    float dist = _nx * DISPLAY_W/2 + _ny * DISPLAY_H/2 - _d;
    const int16_t cx = std::clamp(DISPLAY_W/2 - fast_iroundf(dist * _nx), 0, DISPLAY_W-1);
    const int16_t cy = std::clamp(display_h/2 - fast_iroundf(dist * _ny), 0, display_h-1);

    // direction vector from center to point
    int16_t dx = p.x - cx;
    int16_t dy = p.y - cy;


    // find intersection with screen border and chop scale
    float t_min = 1.f;
    if ( p.x < 0 ) {
        float t = (float)(-cx) / dx;
        if (t < t_min) t_min = t;
    }
    else if ( p.x >= DISPLAY_W ) {
        float t = (float)(DISPLAY_W-1 - cx) / dx;
        if (t < t_min) t_min = t;
    }
    if ( p.y < 0 ) {
        float t = (float)(-cy) / dy;
        if (t < t_min) t_min = t;
    }
    else if ( p.y >= display_h ) {
        float t = (float)(display_h-1 - cy) / dy;
        if (t < t_min) t_min = t;
    }

    // Compute intersection
    return Point(cx + t_min * dx, cy + t_min * dy);
}

// Clip a polygon by line into above and below parts
void IpsDisplay::clipPolygonByLine(const Point *poly, int n, const Line &l, Point *above, int *na, Point *below, int *nb)
{
    const float myEPS = 0.25f;
    auto inside = [&](float f) { return f > myEPS; };
    auto onLine = [&](float f) { return abs(f) <= myEPS; };

    if ( ! poly ) {
        poly = screen_edge;
        n = 4;
    }
    *na = 0;
    *nb = 0;

    for (int i=0; i<n; i++) {
        Point P1 = poly[i];
        Point P2 = poly[(i+1)%n];
        float f1 = l.fct(P1);
        float f2 = l.fct(P2);
        bool in1 = inside(f1);
        bool in2 = inside(f2);

        if (onLine(f1)) {
            // P1 exactly on line -> goes into BOTH lists
            above[(*na)++] = P1;
            below[(*nb)++] = P1;
        }
        else if (in1) {
            above[(*na)++] = P1;
            // ESP_LOGI(FNAME, "P%d above %d,%d", i, P1.x, P1.y);
        }
        else {
            below[(*nb)++] = P1;
        }

        // Check on a possible intersection of screen edge with line
        if (in1 != in2) {
            Point I = l.intersect(P1, P2);
            // ESP_LOGI(FNAME, "I%d at %d,%d", i, I.x, I.y);
            above[(*na)++] = I;
            below[(*nb)++] = I;
        }
    }
}
// Draw filled polygon defined by pts
// n - number of points [0 .. 5]
void IpsDisplay::drawPolygon(Point *pts, int n)
{
    if (n < 3) return; // nothing to draw
    // ESP_LOGI(FNAME, "drawPolygon with %d pts", n);
    int i = 0;
    if (n > 3) {
        // while (i+3 < n) {
            ucg->drawTetragon( pts[i].x, pts[i].y, pts[i+1].x, pts[i+1].y, pts[i+2].x, pts[i+2].y, pts[i+3].x, pts[i+3].y );
            i += 3;
        // }
        if (i+1 < n) {
            ucg->drawTriangle(pts[i].x, pts[i].y,pts[i+1].x, pts[i+1].y,pts[0].x, pts[0].y);
        }
    }
    else {
        // n == 3
        ucg->drawTriangle(pts[i].x, pts[i].y,pts[i+1].x, pts[i+1].y,pts[i+2].x, pts[i+2].y);
        i += 2;
    }
}
void IpsDisplay::drawPolyFrame(Point *pts, int n)
{
    if (n < 3) return; // nothing to draw
    for( int i = 0; i < n; i++ ) {
        int j = (i+1) % n;
        ucg->drawLine(pts[i].x, pts[i].y, pts[j].x, pts[j].y);
    }
}
void IpsDisplay::superBBox(const Point *pts, int n, BoundingBox &bbox)
{
    for( int i = 0; i < n; i++ ) {
        if (pts[i].x < bbox[0].x) bbox[0].x = pts[i].x;
        if (pts[i].x > bbox[1].x) bbox[1].x = pts[i].x;
        if (pts[i].y < bbox[0].y) bbox[0].y = pts[i].y;
        if (pts[i].y > bbox[1].y) bbox[1].y = pts[i].y;
    }
}

static void initRefs()
{
	AVGOFFX = gflags.isPro ? -5-38 : -4;
    UPPERYPOS = OPT_Y_IN * DISPLAY_H + 19;
    LOWERYPOS = (1. - OPT_Y_IN) * DISPLAY_H + 19;
	INNER_RIGHT_ALIGN = DISPLAY_W - 44;
	LOAD_MPG_POS = DISPLAY_H*0.33;
	LOAD_MIAS_POS = DISPLAY_H*0.63;

	// grab screen layout
	AMIDX = gflags.isPro ? (DISPLAY_W/2 + 30) : (DISPLAY_W/2 + 16);
	AMIDY = (DISPLAY_H)/2 - (gflags.isPro ? 0 : 2);
	if ( display_orientation.get() == DISPLAY_NINETY ) {
		INNER_RIGHT_ALIGN = DISPLAY_W - 74;
		AMIDX = DISPLAY_W/2 - 46;
		AVGOFFX = -2;
	}
    else if ( !gflags.isPro ) {
        INNER_RIGHT_ALIGN = 68;
        UPPERYPOS = 32;
        LOWERYPOS = DISPLAY_H - 1;
    }
}

////////////////////////////
// IpsDisplay implementation
IpsDisplay::IpsDisplay( AdaptUGC *aucg ) {
	ucg = aucg;
	tick = 0;
    DISPLAY_W = ucg->getDisplayWidth();
    DISPLAY_H = ucg->getDisplayHeight();
    screen_edge[0] = {0, 0};
    screen_edge[1] = {DISPLAY_W, 0};
    screen_edge[2] = {DISPLAY_W, DISPLAY_H};
    screen_edge[3] = {0, DISPLAY_H};
}

IpsDisplay::~IpsDisplay() {
    // never gets deleted, save the flash memory
    // if (MAINgauge) {
    //     delete MAINgauge;
    //     MAINgauge = nullptr;
    // }
    // if (WNDgauge) {
    //     delete WNDgauge;
    //     WNDgauge = nullptr;
    // }
    // if (MCgauge) {
    //     delete MCgauge;
    //     MCgauge = nullptr;
    // }
    // if (OATgauge) {
    //     delete OATgauge;
    //     OATgauge = nullptr;
    // }
    // if (CONNgauge) {
    //     delete CONNgauge;
    //     CONNgauge = nullptr;
    // }
    // if (S2FBARgauge) {
    //     delete S2FBARgauge;
    //     S2FBARgauge = nullptr;
    // }
    // if (BATgauge) {
    //     delete BATgauge;
    //     BATgauge = nullptr;
    // }
    // if (ALTgauge) {
    //     delete ALTgauge;
    //     ALTgauge = nullptr;
    // }
    // if (TOPgauge) {
    //     delete TOPgauge;
    //     TOPgauge = nullptr;
    // }
    // if (VCSTATgauge) {
    //     delete VCSTATgauge;
    //     VCSTATgauge = nullptr;
    // }
    // if (FLAPSgauge) {
    //     delete FLAPSgauge;
    //     FLAPSgauge = nullptr;
    // }
}

void IpsDisplay::writeText( int line, const char *text )
{
	ucg->setFont(ucg_font_ncenR14_hr, true );
	ucg->setPrintPos( 1, 26*line );
	ucg->setColor(COLOR_WHITE);
	ucg->printf("%s", text);
}

void IpsDisplay::writeText( int line, std::string &text ){
	ucg->setFont(ucg_font_ncenR14_hr, true );
	ucg->setPrintPos( 1, 26*line );
	ucg->setColor(COLOR_WHITE);
	ucg->printf("%s",text.c_str());
}


void IpsDisplay::clear(){
	// ESP_LOGI(FNAME,"display clear()");
	ucg->setColor( COLOR_BLACK );
	ucg->drawBox( 0,0,DISPLAY_W,DISPLAY_H );
	screens_init = INIT_DISPLAY_NULL;
}

void IpsDisplay::setupDisplay() {
    ucg->begin();
    setGlobalColors();
	ucg->setColor(1, COLOR_BLACK );
	ucg->setColor(0, COLOR_WHITE );
	clear();
	ucg->setFont(ucg_font_fub11_tr);
}

void IpsDisplay::setGlobalColors() {
    // set global color variables according to selected display_variant
	if ( display_variant.get() == DISPLAY_WHITE_ON_BLACK ) {
		g_col_background = 255;
		g_col_highlight = 0;
		g_col_header_r=179;
		g_col_header_g=171;
		g_col_header_b=164;
		g_col_header_light_r=94;
		g_col_header_light_g=87;
		g_col_header_light_b=0;
	}
	else {
		g_col_background = 0;
		g_col_highlight = 255;
		g_col_header_r=179;
		g_col_header_g=171;
		g_col_header_b=164;
		g_col_header_light_r=161;
		g_col_header_light_g=168;
		g_col_header_light_b=255;
	}
}

void IpsDisplay::initDisplay() {
    // ESP_LOGI(FNAME,"IpsDisplay::initDisplay()");
    setGlobalColors();
    clear();

    // Create common elements
    initRefs();

    if (!MAINgauge) {
        int16_t scale_geometry = (display_orientation.get() == DISPLAY_NINETY) ? 120 : (gflags.isPro ? 90 : 128 );
        MAINgauge = new PolarGauge(AMIDX, AMIDY, scale_geometry, 
                        DISPLAY_H/2 - ((display_orientation.get() == DISPLAY_NINETY) ? 20 : 35), 
                        gflags.isPro ? PolarGauge::XCVPRO : PolarGauge::CLUB);
    }
    MAINgauge->setFigOffset(AVGOFFX, 0);
    MAINgauge->setUnit(VarioUnit->scale);
    MAINgauge->setRange(scale_range.get(), 0.f, log_scale.get());
    MAINgauge->setColor(needle_color.get());
    if (vario_mc_gauge.get() ) {
        if (!S2FBARgauge) {
            S2FBARgauge = new S2FBar(DISPLAY_W - 15, AMIDY - 61, 28, 8);
        }
    }
    else {
        if ( S2FBARgauge) {
            delete S2FBARgauge;
            S2FBARgauge = nullptr;
        }
    }
    if (vario_mc_gauge.get() && gflags.isPro) {
        if (!MCgauge) {
            MCgauge = new McCready(40, DISPLAY_H + 2);
        }
    }
    else {
        if ( MCgauge ) {
            delete MCgauge;
            MCgauge = nullptr;
        }
    }
    if ( !OATgauge ) {
        // OATgauge = new Temperature(58, 32);
    }
    if ( !CONNgauge ) {
        // CONNgauge = new Connection(DISPLAY_W-25, 24, display_orientation.get() == DISPLAY_NINETY);
    }
    if (battery_display.get() ) {
        if (!BATgauge) {
            BATgauge = new Battery(DISPLAY_W - 10, DISPLAY_H - 12, display_orientation.get() == DISPLAY_NINETY);
        }
    }
    else {
        if ( BATgauge ) {
            delete BATgauge;
            BATgauge = nullptr;
        }
    }
    if ( !VCSTATgauge ) {
        // VCSTATgauge = new CruiseStatus(INNER_RIGHT_ALIGN - 6, 22);
    }
    if ( FLAP && flapbox_enable.get() ) {
        if (!FLAPSgauge) {
            FLAPSgauge = new FlapsBox(FLAP, DISPLAY_W - 29, AMIDY + ((display_orientation.get() == DISPLAY_NINETY) ? 0 : S2FBARgauge ? 17 : 0), true);
        }
    }
    else {
        if ( FLAPSgauge ) {
            delete FLAPSgauge;
            FLAPSgauge = nullptr;
        }
    }

    ucg->setFontPosBottom();

    MAINgauge->drawScale();
    MAINgauge->forceAllRedraw();

    if (!WNDgauge) {
        // create it always, because also the center aid is using it
        WNDgauge = new PolarGauge(AMIDX + AVGOFFX, AMIDY, 360, 58, PolarGauge::COMPASS);
    }
    // WNDgauge->enableWindIndicator(wind_enable.get() > WA_OFF, wind_enable.get() == WA_EXTERNAL);
    WNDgauge->setColor(needle_color.get());
    WNDgauge->setUnit(SpeedUnit->scale);

    if (vario_centeraid.get()) {
        CenterAid::create(*WNDgauge);
    } else {
        CenterAid::remove();
    }

    if (VCSTATgauge) {
        VCSTATgauge->useSymbol(true);
    }
    if (MCgauge) {
        MCgauge->setLarge(display_orientation.get() != DISPLAY_NINETY);
    }
    if ( OATgauge ) {
        OATgauge->setLarge(display_orientation.get() != DISPLAY_NINETY);
    }


    if (vario_lower_gauge.get() == MultiGauge::GAUGE_ALTIMETER) {
        if (!ALTgauge) {
            ALTgauge = new Altimeter(INNER_RIGHT_ALIGN, LOWERYPOS, gflags.isPro || display_orientation.get() == DISPLAY_NINETY);
        }
    } else {
        if (ALTgauge) {
            delete ALTgauge;
            ALTgauge = nullptr;
        }
    }
    if (vario_lower_gauge.get() == MultiGauge::GAUGE_WIND) {
        if (!WNDicon) {
            WNDicon = new WindIcon(1, LOWERYPOS, 18);
        }
    } else {
        if (WNDicon) {
            delete WNDicon;
            WNDicon = nullptr;
        }
    }
    if (vario_upper_gauge.get()) {
        if (!TOPgauge) {
            TOPgauge = new MultiGauge(INNER_RIGHT_ALIGN, UPPERYPOS, (MultiGauge::MultiDisplay)vario_upper_gauge.get(), gflags.isPro || display_orientation.get() == DISPLAY_NINETY);
        }
        TOPgauge->setDisplay((MultiGauge::MultiDisplay)(vario_upper_gauge.get()));
    } else {
        if (TOPgauge) {
            delete TOPgauge;
            TOPgauge = nullptr;
        }
    }
    if (S2FBARgauge) {
        if (display_orientation.get() == DISPLAY_NINETY) {
            S2FBARgauge->setRef(DISPLAY_W - 120, AMIDY);
            S2FBARgauge->setWidth(36);
        } else {
            if (FLAPSgauge) {
                S2FBARgauge->setRef(DISPLAY_W - 15, AMIDY - 61);
                S2FBARgauge->setWidth(28);
            } else {
                S2FBARgauge->setRef(DISPLAY_W - 17, AMIDY);
                S2FBARgauge->setWidth(28);
            }
        }
    }
    if (VCSTATgauge) {
        if (display_orientation.get() == DISPLAY_NINETY) {
            VCSTATgauge->setRef(INNER_RIGHT_ALIGN + 6, AMIDY - 12);
        } else {
            VCSTATgauge->setRef(INNER_RIGHT_ALIGN - 6, 22);
        }
    }
    if (FLAPSgauge) {
        if (display_orientation.get() == DISPLAY_NINETY) {
            FLAPSgauge->setLength(120);
        } else {
            FLAPSgauge->setLength(90);
            FLAPSgauge->setRef(DISPLAY_W - 29, AMIDY + (S2FBARgauge ? 17 : 0));
        }
    }

    // Unit's
    if (TOPgauge) {
        TOPgauge->drawUnit();
    }

    redrawValues();
}

void IpsDisplay::redrawValues()
{
    // ESP_LOGI(FNAME,"IpsDisplay::redrawValues()");
    if (MCgauge) {
        MCgauge->forceRedraw();
    }
    if ( OATgauge ) {
        OATgauge->forceRedraw();
    }
    if ( CONNgauge ) {
        CONNgauge->forceRedraw();
    }
    if ( BATgauge ) {
        BATgauge->forceRedraw();
    }
    if (ALTgauge) {
        ALTgauge->forceRedraw();
    }
    if (TOPgauge) {
        TOPgauge->forceRedraw();
    }
    if (S2FBARgauge) {
        S2FBARgauge->forceRedraw();
        flags.mode_dirty = true;
    }

    if ( FLAPSgauge ) {
        FLAPSgauge->forceRedraw();
    }
    if (theCenteraid) {
        theCenteraid->forceRedraw();
    }
    if (WNDicon) {
        WNDicon->forceRedraw();
    }
}

void IpsDisplay::setBottomDirty()
{
    flags.bottom_dirty = true;
}
void IpsDisplay::setCruiseChanged()
{
    flags.mode_dirty = true;
}
void IpsDisplay::setQnhChanged() {
    if ( ALTgauge ) {
        ALTgauge->forceRedraw();
    }
}


//////////////////////////////////////////////
// The load display

static float old_gmax = 100;
static float old_gmin = -100;
static float old_ias_max = -1;

void IpsDisplay::drawLoadDisplayTexts(){
	ucg->setFont(ucg_font_fub11_hr, true);
	const char *text = "ext. G-Loads";
	int16_t text_width = ucg->getStrWidth( text );
	ucg->setPrintPos(DISPLAY_W-10-text_width, LOAD_MPG_POS);
	ucg->setColor( COLOR_HEADER_LIGHT );
	ucg->print( text );
	text = "max. IAS";
	text_width = ucg->getStrWidth( text );
	ucg->setPrintPos(DISPLAY_W-10-text_width, LOAD_MIAS_POS);
	ucg->print( text );
}

void IpsDisplay::initLoadDisplay(){
	clear();
	ESP_LOGI(FNAME,"initLoadDisplay()");
	ucg->setColor( COLOR_HEADER );
	ucg->setFont(ucg_font_fub11_hr);
	ucg->setPrintPos(20,20);
	ucg->print("G-Force");
	drawLoadDisplayTexts();
	int max_gscale = gload_pos_limit.get() + 2;
	int min_gscale = gload_neg_limit.get() - 2;
	if ( ! MAINgauge ) { // shared with
		int16_t scale_geometry = ( display_orientation.get() == DISPLAY_NINETY ) ? 120 : 90;
		MAINgauge = new PolarGauge(AMIDX, AMIDY, scale_geometry, DISPLAY_H/2-20, PolarGauge::GLOAD);
	}
	MAINgauge->setFigOffset(0, 0);
	MAINgauge->setUnit(1.);
	MAINgauge->setRange(max_gscale, 1.f, false);
	MAINgauge->setColor(needle_color.get());
	// put the scale colored section into the background
	MAINgauge->colorRange(gload_pos_limit_low.get(), gload_pos_limit.get(), PolarGauge::ORANGE);
	MAINgauge->colorRange(gload_pos_limit.get(), max_gscale, PolarGauge::RED);
	MAINgauge->colorRange(gload_neg_limit_low.get(), gload_neg_limit.get(), PolarGauge::ORANGE);
	MAINgauge->colorRange(gload_neg_limit.get(), min_gscale, PolarGauge::RED);
	MAINgauge->drawScale();
	MAINgauge->forceAllRedraw();
	old_gmax = 100;
	old_gmin = -100;
	old_ias_max = -1;
	flags.bottom_dirty = false;
	ESP_LOGI(FNAME,"initLoadDisplay end");
}


void IpsDisplay::drawLoadDisplay( float loadFactor ){
	// ESP_LOGI(FNAME,"drawLoadDisplay %1.1f tick: %d", loadFactor, tick );
	tick++;

	if( !(screens_init & INIT_DISPLAY_GLOAD) ){
		initLoadDisplay();
		screens_init |= INIT_DISPLAY_GLOAD;
	}
	// draw G pointer
	MAINgauge->drawIndicator( loadFactor );

	// G load digital
	if( !(tick%3) ) {
		MAINgauge->drawFigure(loadFactor);
	}

	// Min/Max values
	if( old_gmax != gload_pos_max.get() || old_gmin != gload_neg_max.get() || !(tick%10) ) {
		if( gload_pos_max.get() < gload_pos_limit.get() && gload_neg_max.get() > gload_neg_limit.get()) {
			ucg->setColor( COLOR_LGREY );
		}
		else {
			ucg->setColor( COLOR_RED );
		}

		ucg->setFont(ucg_font_fub14_hr, true);
		char buf[60];
		sprintf( buf, "  %+1.1f / %+1.1f g", gload_pos_max.get(), gload_neg_max.get() );
		int16_t text_width = ucg->getStrWidth( buf );
		ucg->setPrintPos(DISPLAY_W-10-text_width, LOAD_MPG_POS+24);
		ucg->print(buf);
		old_gmax = gload_pos_max.get();
		old_gmin = gload_neg_max.get();
	}
	if( old_ias_max != airspeed_max.get() || !(tick%10)){
		if( airspeed_max.get() < v_max.get() ) {
			ucg->setColor( COLOR_LGREY );
		} else {
			ucg->setColor( COLOR_RED );
		}
		
		ucg->setFont(ucg_font_fub14_hr, true);
		char buf[60];
		sprintf( buf, "  %3d %s", fast_iroundf_positive(SpeedUnit->apply(airspeed_max.get())), SpeedUnit->getName() );
		int16_t text_width = ucg->getStrWidth( buf );
		ucg->setPrintPos(DISPLAY_W-10-text_width, LOAD_MIAS_POS+24);
		ucg->print(buf);
		old_ias_max = airspeed_max.get();
	}

	if( !(tick%10)){
		drawLoadDisplayTexts();
	}
	if ( flags.bottom_dirty ) {
		flags.bottom_dirty = false;
		initLoadDisplay();
	}
}

static rad_t computeHeading(rad_t track, mps_t groundspeed, mps_t tas, int16_t wind_vdir, mps_t windspeed)
{
    // wind vector
    float wx = windspeed * fast_cos_idx(wind_vdir);
    float wy = windspeed * fast_sin_idx(wind_vdir);

    // ground vector
    float gx = groundspeed * fast_cos_rad(track);
    float gy = groundspeed * fast_sin_rad(track);
    

    // air vector = ground - wind
    mps_t ax = gx * tas - wx;
    mps_t ay = gy * tas - wy;

    return atan2f(ay, ax);
}

rad_t getHeading() { // fixme move to compass or GNSS/INS
    // if ( mag_hdt.getValid() ) {
    //     // use mag compass heading if valid
    //     return mag_hdt.get();
    // }
    if( Flarm::gpsStatus() && tas.getValid() && synoptic_wind.getValid() ) {
       // fall back to GPS course
        WindData wd = static_cast<WindData>(synoptic_wind.get());
        return computeHeading(Flarm::getGndCourse(), Flarm::getGndSpeed(), tas.get(), wd.getVDeg2(), wd.getVal());
    }
    else if ( Flarm::gpsStatus() ) {
        // fall back to GPS course without wind correction
        return Flarm::getGndCourse();
    }
    return 0.;
}

rad_t getWCA() {
    if( Flarm::gpsStatus() && tas.getValid() && synoptic_wind.getValid() ) {
        WindData wd = static_cast<WindData>(synoptic_wind.get());
        rad_t hdg = computeHeading(Flarm::getGndCourse(), Flarm::getGndSpeed(), tas.get(), wd.getVDeg2(), wd.getVal());
        return Vector::angleDiff(hdg, Flarm::getGndCourse());
    }
    return 0.;
}

// max. 10 Hz update rate for the display
void IpsDisplay::drawDisplay(){
	if( !(screens_init & INIT_DISPLAY_RETRO) ){
		initDisplay();
		screens_init |= INIT_DISPLAY_RETRO;
        return; // split the first draw into two calls
	}
	tick++;

	// todo integrate better into screen element
    mps_t te_ms = te_vario.get();
    mps_t te_avg_ms = bmpVario.getAvgVario();
    mps_t polar_sink_ms = bmpVario.getPolarSink();
	if ( CRMOD.isNetto() ) {
		te_ms -= polar_sink_ms;
		te_avg_ms -= polar_sink_ms; // average
	}
	if ( CRMOD.getVMode() == CruiseMode::MODE_REL_NETTO ) { // Super Netto, considering circling sink
		te_ms += Speed2Fly.getCirclingSink( ias.get() );
		te_avg_ms += Speed2Fly.getCirclingSink( ias.get() );
	}

    // average Climb
    if (!(tick % 2)) {
        MAINgauge->drawFigure(te_avg_ms);
    }

    // S2F bar
    if ( (!(tick % 11) || flags.mode_dirty) && S2FBARgauge ) {
        // static float s=0; // check the bar code
        // s2fd = sin(s) * 42.;
        // s+=0.04;
        S2FBARgauge->draw(Speed2Fly.getDelta(), CRMOD.getCMode());
    }

    // MC val
    if (MCgauge && !(tick % 5)) {
        MCgauge->draw(MC.get());
    }

    // Bluetooth etc
    if (!(tick % 12)) {
        if ( CONNgauge ) {
            CONNgauge->draw();
        }
    }

    // Upper gauge
    if (vario_upper_gauge.get() && !(tick % 3)) {
        TOPgauge->draw();
    }

    // Altitude
    if (ALTgauge) {
        ALTgauge->draw(altitude.get());
    }

    // Wind & center aid
    if (!(tick % 5)) {
        if (theCenteraid && !CRMOD.getCMode()) {
            theCenteraid->drawCenterAid();
        }
        if (wind_enable.get() > WA_OFF && WNDicon) {
            // static int idir=0;
            // static int ival=5;
            // // the wind simulator to check the wind indicator
            // idir = (idir + (rand()%5))%360; //  a"-" triggers the disc bug"
            // // idir++;
            // // ival = (ival+(rand()%5))%360;

            // WindData swind(Units::deg_to_rad(idir), (mps_t)(fast_sin_idx(idir*1.5)+1)/2*60.f/3.6f - 1);
            // // WindData iwind(Units::deg_to_rad(ival), (mps_t)ival/3.6);

            WindData swind, iwind;
            if (wind_enable.get() & WA_BOTH) {
                if (synoptic_wind.getValid()) {
                    swind = static_cast<WindData>(synoptic_wind.get());
                }
            } else {
                iwind.raw = ext_inst_wind.get();
                swind.raw = ext_syn_wind.get();
            }
            ESP_LOGI(FNAME, "draw wind swind: %d@%.1f cwind: %d@%.1f", swind.getDeg(), swind.getVal(), iwind.getDeg(), iwind.getVal());
            // WNDgauge->drawWind(swind, iwind);
            WNDicon->draw(swind);
        }
    }

    // Vario indicator
    MAINgauge->draw(te_ms);
    if (gflags.isPro && CRMOD.isGross()) {
        MAINgauge->drawPolarSink(polar_sink_ms);
    }

    // Battery
    if (!(tick % 15)) {
        if ( BATgauge ) {
            BATgauge->draw(battery_voltage.get());
        }
    }

    // Temperature Value
    if( !(tick%12) ) {
        if ( OATgauge ) {
            OATgauge->draw((accSensor) ? accSensor->getTempStatus() : temp_status_t::MPU_T_UNKNOWN);
        }
    }

    // WK-Indicator
    if (FLAPSgauge && !(tick % 3)) {
        FLAPSgauge->draw(ias.get());
        // Check on flap speeds defined
        if ( FLAP->getNrPositions() == 0 && ! flags.flp_speed_msg_shown) {
            MBOX->pushMessage(2, "Pls. set flap speeds" );
            flags.flp_speed_msg_shown = true;
        }
    }

    // Cruise mode or circling
    if( flags.mode_dirty ) {
        if (gflags.isPro) {
            if (VCSTATgauge) {
                VCSTATgauge->draw();
            }
        } else {
            MAINgauge->forceRedrawMode();
        }

        if (vario_centeraid.get()) {
            if (CRMOD.getCMode()) {
                WNDgauge->clearGauge();
                // WNDgauge->drawRose();
            }
            // else {
            //     WNDgauge->clearGauge();
            // }
        }
        flags.mode_dirty = false;
    }

    // Medium Climb Indicator
    if (!(tick % 4)) {
        // static float s = 0; // check the diamond
        // average_climb.set(sin(s) * 2.);
        // s += 0.1;
        MAINgauge->drawAvgClimb();
    }

    if (flags.bottom_dirty) {
        ESP_LOGI(FNAME, "redraw scale bottom");
        flags.bottom_dirty = false;
        MAINgauge->drawScaleBottom();
        MAINgauge->forceAllRedraw();
        if (MCgauge)
        {
            MCgauge->forceRedraw();
        }
        if ( BATgauge ) {
            BATgauge->forceRedraw();
        }
        if ( ALTgauge && ! gflags.isPro ) {
            ALTgauge->forceRedraw();
        }
        if ( WNDicon ) {
            WNDicon->forceRedraw();
        }
    }
    // ESP_LOGI(FNAME,"IpsDisplay::drawDisplay  TE=%0.1f  x0:%d y0:%d x2:%d y2:%d", te, x0, y0, x2,y2 );
}


