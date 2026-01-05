#pragma once

#include "PressureSensor.h"

#include <cstdint>

namespace i2cbus {
    class I2C;
}


class SPL06_007: public PressureSensor {
public:
	SPL06_007(SensorId id);
	virtual ~SPL06_007() {};
	bool  setSPIBus(gpio_num_t _sclk, gpio_num_t _mosi, gpio_num_t _miso, uint32_t _freq ) { return true; };
	const char* name() const override { return "SPL06_007"; }
	bool probe() override;
	bool setup() override;
	bool selfTest( float &t, float &p );
	float doRead() override;
	float readTemperature( bool& success ) override;

private:
	float get_temp_c( bool &ok );
	float get_temp_f();
	int32_t get_praw( bool &ok );
	float get_praw_sc( bool &ok );

	int32_t get_traw( bool &ok );
	float get_traw_sc( bool &ok );

	float get_scale_factor( int reg );

	// uint8_t get_spl_id(){ return i2c_read_uint8( 0x0D ); }		// Get ID Register 		0x0D
	// uint8_t get_spl_prs_cfg(){ return i2c_read_uint8( 0x06 ); };	// Get PRS_CFG Register	0x06
	// uint8_t get_spl_tmp_cfg(){ return i2c_read_uint8( 0x07 ); };	// Get TMP_CFG Register	0x07
	// uint8_t get_spl_meas_cfg(){ return i2c_read_uint8( 0x08 ); };	// Get MEAS_CFG Register	0x08
	// uint8_t get_spl_cfg_reg(){ return i2c_read_uint8( 0x09 ); };	// Get CFG_REG Register	0x09
	// uint8_t get_spl_int_sts(){ return i2c_read_uint8( 0x0A ); };	// Get INT_STS Register	0x0A
	// uint8_t get_spl_fifo_sts(){ return i2c_read_uint8( 0x0B ); };	// Get FIFO_STS Register	0x0B


	int16_t get_16bit( uint8_t addr );
	int16_t get_c0();
	int16_t get_c1();
	int32_t get_c00();
	int32_t get_c10();


	int32_t c00,c10;
	int16_t c0,c1;
	int16_t c01,c11,c20,c21,c30;

	uint8_t i2c_read_uint8( uint8_t eeaddress );
	bool i2c_read_bytes( uint8_t eeaddress, int num, uint8_t *data );

	i2cbus::I2C *_bus;
	uint8_t _address;
	float  _scale_factor_p;
	float  _scale_factor_t;
	int    errors;
	int32_t last_praw;
	int32_t _traw;
	int32_t last_traw;
	uint32_t tick;
	float  last_p;
};


