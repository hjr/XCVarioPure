/*
 * SetupMenu.cpp
 *
 *  Created on: Feb 4, 2018
 *      Author: iltis
 */

#include "setup/SetupMenuValFloat.h"

#include "setup/SetupMenu.h"
#include "AdaptUGC.h"
#include "IpsDisplay.h"
#include "sensor/VarioFilter.h"
#include "math/Units.h"
#include "sensor.h"
#include "logdefnone.h"

#include <algorithm>

extern AdaptUGC *MYUCG;

char SetupMenuValFloat::_val_str[50];

static float valueAsQuantity(float val, quantity_t q) {
    switch (q) {
        case quantity_t::QUANT_NONE:
            return val;
        case quantity_t::QUANT_TEMPERATURE:
            return TempUnit->apply(Units::C_to_K(val));
        case quantity_t::QUANT_ALT:
            return AltUnit->apply(val);
        case quantity_t::QUANT_HSPEED:
            return SpeedUnit->apply(val);
        case quantity_t::QUANT_HSLEGACY:
            return SpeedUnit->apply(Units::kmh_to_mps(val));
        case quantity_t::QUANT_VSPEED:
            return VarioUnit->apply(val);
        case quantity_t::QUANT_QNH:
            return PressureUnit->apply(val);
        // case quantity_t::QUANT_MASS:
        // case quantity_t::QUANT_TIME:
        default:
            return val;
    }
}

static const char* unitFromQuantity(quantity_t q)
{
	switch (q)
	{
	case quantity_t::QUANT_NONE:
		return "";
	case quantity_t::QUANT_TEMPERATURE:
		return TempUnit->getName();
	case quantity_t::QUANT_ALT:
		return AltUnit->getName();
	case quantity_t::QUANT_HSPEED:
    case quantity_t::QUANT_HSLEGACY:
		return SpeedUnit->getName();
	case quantity_t::QUANT_VSPEED:
		return VarioUnit->getName();
	case quantity_t::QUANT_QNH:
		return PressureUnit->getName();
	default:
		return "";
	}
}


SetupMenuValFloat::SetupMenuValFloat( const char* title, const char *unit, int (*action)( SetupMenuValFloat *p ), 
	bool end_menu, SetupNG<float> *anvs, e_restart_mode_t restart, bool live_update) :
	MenuEntry(title),
	_action(action),
	_nvs(anvs)
{
	ESP_LOGI(FNAME,"SetupMenuValFloat( %s ) ", title );
	if( unit != 0 && *unit != '\0' ) {
		_unit = unit;
        ESP_LOGI(FNAME,"unit %s", _unit );
	}
	else if ( _nvs ) {
		_unit = unitFromQuantity( _nvs->quantityType() );
        ESP_LOGI(FNAME,"nvs unit %s", _unit );
	}

	assert(_nvs); // the implementation does not allow
	_value = _nvs->get();
	if ( _nvs->hasLimits() ) {
		_min = _nvs->getMin();
		_max = _nvs->getMax();
		_step = _nvs->getStep();
	}

	bits._restart = restart;
	bits._end_menu = end_menu;
	bits._precision = 2;
	bits._live_update = live_update;
	if( _step >= 1 ) {
		bits._precision = 0;
	}
}

void SetupMenuValFloat::enter()
{
	if (isLocked()) { return; }
	_value_safe = _value = _nvs->get();
	MenuEntry::enter();
}

const char *SetupMenuValFloat::value() const
{
	float uval = valueAsQuantity( _value, _nvs->quantityType() );
	// ESP_LOGI(FNAME,"value() ulen: %d val: %f, utype: %d unitval: %f", strlen( _unit ), _nvs->get(), _nvs->quantityType(), uval  );
	sprintf(_val_str,"%0.*f %s   ", bits._precision, uval, _unit );
	return _val_str;
}

void SetupMenuValFloat::setPrecision( int prec ){
	bits._precision = prec;
}

void SetupMenuValFloat::display(int mode)
{
	ESP_LOGI(FNAME,"display %s", _title.c_str());
	if ( bits._is_inline ) {
		indentHighlight(_parent->getHighlight());
	}
	else {
		menuPrintLn(_title.c_str(), 0, 5 );
		displayVal();
	}
	if( _action != 0 ) {
		(*_action)( this );
	}
}

void SetupMenuValFloat::refresh()
{
	_value = _nvs->get();
}

void SetupMenuValFloat::displayVal()
{
	// ESP_LOGI(FNAME,"displayVal %s", v);
	if ( bits._is_inline ) {
		indentPrintLn(value());
	}
	else {
		MYUCG->setFont(ucg_font_fub25_hf, true);
		menuPrintLn(value(), 2);
		MYUCG->setFont(ucg_font_ncenR14_hr);
	}
}

void SetupMenuValFloat::rot(int count) {
    // ESP_LOGI(FNAME,"val rot %d times ", count );
    _value += _step * count;
    _value = std::clamp(_value, _min, _max);

    if (bits._live_update) {
        _nvs->set(_value, false, false);
    }

    displayVal();
    
    if (_action != 0) {
        (*_action)(this);
    }
}

void SetupMenuValFloat::longPress()
{
	press(); // implicit trigger also on long press actions when in Setup menu.
}

void SetupMenuValFloat::press()
{
	ESP_LOGI(FNAME,"SetupMenuValFloat value: %f", _value );
	ESP_LOGI(FNAME,"Check if _value: %f != _value_safe: %f", _value, _value_safe );

	// if ( _exit_action ) {
	// 	_exit_action( this );
	// }

	if( _value != _value_safe ){
		_nvs->set( _value );
        if (_action != 0) {
           (*_action)(this);
        }
		if ( hasHelp() ) {
			SavedDelay();
		}
		ESP_LOGI(FNAME,"Yes restart:%d", bits._restart);
		_value_safe = _value;
		if( bits._restart == RST_ON_EXIT ) {
			scheduleReboot();
		}else if( bits._restart == RST_IMMEDIATE ){
			MenuEntry::reBoot();
		}
	}

	if( bits._end_menu ) {
		exit(-1);
		return;
	}
	exit();
}
