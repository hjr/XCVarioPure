/*
 * S2F.h
 *
 *  Created on: Dec 26, 2018
 *      Author: iltis
 */

#pragma once



class S2F {
public:
	S2F( );
	~S2F();
	void begin();
	void calculateOverweight();
	void modifyPolar();
	bool IsValid() const { return _valid;};
	void recalculatePolar();
	void change_ballast();
	void change_mc();
	void setPolar();
	static bool isPolarEqualTo(int idx);
	float speed( float st, bool circling=false );
	float getStallSpeed() const { return _stall_speed_ms * 3.6; }
	float sink( float v );
	float minsink_speed() { return _min_sink_speed; };
	void recalcSinkNSpeeds();
	float circlingSink(float v);
	float cw( float v );

private:
	static float getBallastPercent();
	void test( void );
	float getN();
	float getVn( float v );
	bool calcValidPolar();

	float myballast;
	static float bal_percent;
	float a0,a1,a2;
	float w0,w1,w2;
	float _min_sink_speed;
	float _min_sink;
	float _circling_speed;
	float _circling_sink;
	float _stall_speed_ms;
	bool _valid = false;
};

extern S2F Speed2Fly;
