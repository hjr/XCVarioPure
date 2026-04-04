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
#include "wind/StraightWind.h"
#include "wind/CircleWind.h"
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

Point Point::operator+(Point p) const {
    return p += *this;
}
// alpha [rad]
Point Point::rotate(float alpha) const {
    float cos_a = fast_cos_rad(alpha);
    float sin_a = fast_sin_rad(alpha);
    return Point( x * cos_a - y * sin_a, x * sin_a + y * cos_a );
}


// Hesse horizon line parameters in the gliders Y/Z plane
// quaternion q is attitude in NED frame, 
// cx/cy is the center of the display
Line::Line(Quaternion q, int16_t cx, int16_t cy) {
    // normal vector of the plane
    vector_f up = q.rotate(vector_f(0, 0, 1));
    _nx = -up.y;
    _ny = -up.z;
    _d = std::clamp(up.x, -0.7f, 0.7f) * 100; // projection scale to visible range
    _d += _nx * cx + _ny * cy; // offset to the middle of the screen
    // ESP_LOGI(FNAME, "normV %0.1f,%0.1f,%0.1f", _nx, _ny, _d);
}
// evaluate line function at point p
float Line::fct(Point p) {
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
    ret.x = fast_iroundf(dx * t) + p1.x;
    ret.y = fast_iroundf(dy * t) + p1.y;
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


// Clip rectangle by line into above and below parts
void IpsDisplay::clipRectByLine(Point *rect, Line &l, Point *above, int *na, Point *below, int *nb)
{
    const int myEPS = 1;
    auto inside = [&](int f) { return f > myEPS; };
    auto onLine = [&](int f) { return abs(f) <= myEPS; };

    if ( ! rect ) {
        rect = screen_edge;
    }
    *na = 0;
    *nb = 0;

    for (int i=0; i<4; i++) {
        Point P1 = rect[i];
        Point P2 = rect[(i+1)&3];
        int f1 = l.fct(P1);
        int f2 = l.fct(P2);
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
// Project from glider reference into avionik display plane (Y-Z plane)
Point IpsDisplay::projectToDisplayPlane(const vector_f &obj, float focus)
{
    // camera model: pinhole camera at origin, looking along X axis, display plane at focus distance
    focus *= fast_signf(obj.x);
    // project to display plane
    if ( std::abs(obj.x) < 1e-4f ) {
        // avoid division by zero
        return Point( -10000, -10000 );
    }
    float u = focus * (obj.y / obj.x);
    float v = focus * (obj.z / obj.x);

    return Point(DISPLAY_W/2 - u, DISPLAY_H/2 - v);
}
// Centered screen clipping
Point IpsDisplay::clipToScreenCenter(Point p)
{
    const int16_t cx = DISPLAY_W/2;
    const int16_t cy = DISPLAY_H/2;

    // direction vector from center to point
    int16_t dx = p.x - cx;
    int16_t dy = p.y - cy;

    // check p against boundaries
    if (p.x >= 0 && p.x < DISPLAY_W && p.y >= 0 && p.y < DISPLAY_H) {
        return p;
    }

    // find intersection with screen border and chop scale
    float t_min = 1.f;
    if ( p.x < 0 ) {
        float t = (float)(-cx) / dx;
        if (t < t_min) t_min = t;
    }
    else if ( p.x >= DISPLAY_W ) {
        float t = (float)(cx) / dx;
        if (t < t_min) t_min = t;
    }
    if ( p.y < 0 ) {
        float t = (float)(-cy) / dy;
        if (t < t_min) t_min = t;
    }
    else if ( p.y >= DISPLAY_H ) {
        float t = (float)(cy) / dy;
        if (t < t_min) t_min = t;
    }

    // Compute intersection
    return Point(cx + t_min * dx, cy + t_min * dy);
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
	AMIDY = (DISPLAY_H)/2;
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
	// ESP_LOGI(FNAME,"IpsDisplay::bootDisplay()");
	if( display_type.get() == ST7789_2INCH_12P )
		ucg->setRedBlueSwap( true );
	if( display_type.get() == ILI9341_TFT_18P )
		ucg->invertDisplay( true );
	// ESP_LOGI(FNAME,"clear boot");
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
        MAINgauge = new PolarGauge(AMIDX, AMIDY, scale_geometry, DISPLAY_H/2 - ((gflags.isPro || display_orientation.get() == DISPLAY_NINETY) ? 20 : 36), 
                            gflags.isPro ? PolarGauge::XCVPRO : PolarGauge::CLUB);
    }
    MAINgauge->setUnit(VarioUnit->scale);
    MAINgauge->setRange(scale_range.get(), 0.f, log_scale.get());
    MAINgauge->setColor(needle_color.get());
    if (vario_mc_gauge.get() ) {
        if (!S2FBARgauge) {
            S2FBARgauge = new S2FBar(DISPLAY_W - 15, AMIDY - 64, 28, 8);
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
    if (!BATgauge) {
        BATgauge = new Battery(DISPLAY_W - 10, DISPLAY_H - 12, display_orientation.get() == DISPLAY_NINETY || !gflags.isPro);
    }
    if ( !VCSTATgauge ) {
        // VCSTATgauge = new CruiseStatus(INNER_RIGHT_ALIGN - 6, 22);
    }
    if ( FLAP && flapbox_enable.get() ) {
        if (!FLAPSgauge) {
            FLAPSgauge = new FlapsBox(FLAP, DISPLAY_W - 29, AMIDY + (gflags.isPro ? 0 : (display_orientation.get() == DISPLAY_NINETY) ? 0 : 22), true);
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
    MAINgauge->setFigOffset(AVGOFFX, 0);

    if (!WNDgauge) {
        // create it always, because also the center aid is using it
        WNDgauge = new PolarGauge(AMIDX + AVGOFFX, AMIDY, 360, 52, PolarGauge::COMPASS);
    }
    WNDgauge->enableWindIndicator(wind_enable.get() > WA_OFF, wind_enable.get() == WA_EXTERNAL);
    WNDgauge->setWindRef(wind_reference.get());
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


    if (vario_lower_gauge.get()) {
        if (!ALTgauge) {
            ALTgauge = new Altimeter(INNER_RIGHT_ALIGN, LOWERYPOS, gflags.isPro || display_orientation.get() == DISPLAY_NINETY);
        }
    } else {
        if (ALTgauge) {
            delete ALTgauge;
            ALTgauge = nullptr;
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
                S2FBARgauge->setRef(DISPLAY_W - 15, AMIDY - 64);
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
            FLAPSgauge->setLength(100);
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


float getHeading() { // fixme move to compass
	float heading = 0;
	heading = mag_hdt.get();
	if( (heading < 0) && Flarm::gpsStatus() ) {
		// fall back to GPS course
		heading = Flarm::getGndCourse();
	}
	return heading;
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
    if (!(tick % 11) && S2FBARgauge && CRMOD.getCMode() ) {
        // static float s=0; // check the bar code
        // s2fd = sin(s) * 42.;
        // s+=0.04;
        S2FBARgauge->draw(Speed2Fly.getDelta(), s2f_ideal.get());
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
        } else if (wind_enable.get() > WA_OFF) {
            // static int16_t wdir=-1, idir=-1;
            // static int16_t wval=0, ival=0;
            // the wind simulator to check the wind indicator
            // float d = (rand()%180) / M_PI_2;
            // idir += abs(sin(d)) + 2;
            // ival = int((wval+d))%120;

            int16_t wdir = -1, inst_dir = -1;
            mps_t wval = 0, inst_val = 0;
            if (wind_enable.get() & WA_BOTH) {
                if (straightWind && !straightWind->getWind(&wdir, &wval)) {
                    wdir = -1;
                }

                if (circleWind && !circleWind->getWind(&wdir, &wval)) {
                    wdir = -1;
                }
            } else {
                inst_dir = extwind_inst_dir.get();
                inst_val = extwind_inst_speed.get();
                wdir = extwind_sptc_dir.get();
                wval = extwind_sptc_speed.get();
            }
            WNDgauge->drawWind(wdir, wval, inst_dir, inst_val);
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
            if (S2FBARgauge && !CRMOD.getCMode()) {
                S2FBARgauge->clear();
            }
        }
        WNDgauge->clearGauge();
        if (!vario_centeraid.get() || CRMOD.getCMode()) {
            WNDgauge->drawRose();
        }
        flags.mode_dirty = false;
    }

    // Medium Climb Indicator
    if (!(tick % 4)) {
        // static float s = 0; // check the diamond
        // average_climb.set(sin(s) * 2.);
        // s += 0.1;
        MAINgauge->drawAVG();
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
    }
    // ESP_LOGI(FNAME,"IpsDisplay::drawDisplay  TE=%0.1f  x0:%d y0:%d x2:%d y2:%d", te, x0, y0, x2,y2 );
}


