#pragma once

#include "setup/SetupNG.h"

namespace Units
{
	float Speed(float as);
	float Distance(float d);
	int SpeedRounded(float as);
	float kmh2knots(float kmh);
	float kmh2ms(float kmh);
	float ms2kmh(float ms);
	float knots2kmh(float knots);
	float Airspeed2Kmh(float as);
	float ActualWingloadCorrection(float v);
	float TemperatureUnit(float t);
	const char* TemperatureUnitStr(int idx = -1);
	const char* SpeedUnitStr(int u = -1);
	float Vario(const float te);
	float Qnh(float qnh);
	int QnhRounded(float qnh);
	float hPa2inHg(float hpa);
	float inHg2hPa(float inhg);
	float knots2ms(float knots);
	float ms2knots(float knots);
	float ms2mph(float ms);
	float ms2fpm(float ms);
	float Vario2ms(float var);
	float mcval2knots(float mc);
	const char *VarioUnit();
	const char *QnhUnit(int unit = -1);
	const char *VarioUnitLong(int unit = -1);
	float Altitude(float alt, int unit = -1);
	float meters2feet(float m);
	float feet2meters(float f);
	float meters2FL(float m);
	const char *AltitudeUnit(int unit = -1);
	const char *AltitudeUnitMeterOrFeet(int unit = -1);
	const char *DistanceUnit(int unit = -1);
	float value(float val, e_quantity_t u);
	const char *unit(e_quantity_t u);
};
