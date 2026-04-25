
#pragma once

class PressureSensor;
class AirspeedSensor;

class LeakTest{

public:
	static void start( PressureSensor* bmpBA, PressureSensor* bmpTE, AirspeedSensor *as );

};



