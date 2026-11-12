/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "CruiseMode.h"

#include "setup/SetupNG.h"
#include "IpsDisplay.h"
#include "logdefnone.h"

CruiseMode CRMOD; // a global variable

CruiseMode::CruiseMode()
{
    updateCache();
}

void CruiseMode::setCMode(bool cruise)
{
    if ( ! _lock ) {
        if ( cruise != _cmode ) {
            cruise_mode.set(cruise);
            _cm_changed = true;
        }
    }
}

void CruiseMode::lockTo(bool cruise)
{
    _lock = true;
    cruise_mode.set(cruise);
}

// callback to any change on cruise_mode setup
void CruiseMode::updateCache()
{
    // set the cruise mode cache
    _cmode = cruise_mode.get();
    ESP_LOGI(FNAME, "update cruise mode: %d", _cmode);

    // set the vario mode cache
    const bool relative = netto_mode.get() == static_cast<int>(e_netto_mode::NETTO_RELATIVE); // Super Netto
    const int vm = vario_mode.get();
    if (vm == static_cast<int>(e_vario_mode::VARIO_NETTO) || (_cmode && vm == static_cast<int>(e_vario_mode::CRUISE_ONLY_NETTO))) {
        _vmode_setup = relative ? MODE_REL_NETTO : MODE_NORMAL_NETTO;
    } else {
        _vmode_setup = MODE_BRUTTO;
    }
    if (te_comp_enable.get() == TE_AIRMASS) {
        _vmode_setup |= AIRMASS_BIT;
    }
    applyAirMass();

    // set the audio mode cache
    _audio_vario = ( ! _cmode || (cruise_audio_mode.get() != AUDIO_S2F) );
    // set the audio chopping style cache
    // breaks can be filled with silence or with dual tone
    _audio_chopping = (_audio_vario && (chopping_mode.get() & VARIO_CHOP) ) ||
            (!_audio_vario && (chopping_mode.get() & S2F_CHOP) );
}

// without a valid air mass vario the TE vario of the same mode
void CruiseMode::applyAirMass()
{
    _vmode = _airmass_available ? _vmode_setup : (_vmode_setup & ~AIRMASS_BIT);
}

void CruiseMode::setAirMassAvailable(bool available)
{
    _airmass_available = available;
    applyAirMass();
    if (isAirMass() != _airmass_shown) {
        // the vario started or stopped showing the air mass movement
        _airmass_shown = isAirMass();
        _cm_changed = true; // restart the averages
        IpsDisplay::setCruiseChanged(); // refresh the mode label
    }
}

const char* CruiseMode::label(LabelStyle style) const
{
    const bool compact = style == LABEL_COMPACT;
    switch (_vmode) {
    case MODE_NORMAL_NETTO:
        return compact ? "  net" : "netto";
    case MODE_REL_NETTO:
        return compact ? "snet" : "s-netto";
    case MODE_AIRMASS_GROSS:
        return compact ? " gair" : "g-air";
    case MODE_AIRMASS:
        return compact ? "  air" : "air";
    case MODE_REL_AIRMASS:
        return compact ? " sair" : "s-air";
    default:
        return ""; // gross
    }
}

// call in context that should go in synch with the pending cruise mode change
void CruiseMode::promoteChange()
{
    _cm_changed = false;
    if (_pending_change) {
        _cm_changed = true;
        _pending_change = false;
    }
}
