/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#pragma once

#include <cstdint>

// A wrapper to the logic of the setup variables:
// vario_mode - persistent info, but individual for master/client, about gross/netto
// netto_mode - persistent info, but individual for master/client, about used netto flavor (in case)
// cruise_mode - volatile status of the s2f mode, whereas syncronized bidirectional
//
// This class as a combinated logic gives straight access to
//  - S2F/Vario status queries
//  - Vario gauge display status
// and allows the straight modification of e.g. S2F mode respecting the implication
// to the vario gauge display status.


class CruiseMode
{
public:
    using VarioModeBits = enum : uint8_t { MODE_BRUTTO=0, MODE_NORMAL_NETTO=1, MODE_REL_NETTO=3 }; // vario mode bits
    CruiseMode();
    bool getCMode() const { return _cmode; };
    bool getChanged() const { return _cm_changed; }
    bool isGross() const { return _vmode == MODE_BRUTTO; }
    bool isNet() const { return _vmode != MODE_BRUTTO; }
    bool audioIsVario() const { return _audio_vario; }
    bool audioIsChopping() const { return _audio_chopping; }
    int16_t getVMode() const { return _vmode; }

    // setter
    void setCMode(bool cruise);
    void lockTo(bool cruise);
    void unlockCMode() { _lock = false; }
    void updateCache();
    void promoteChange(); // call only once per sensor loop cycle

private:
    uint8_t _vmode = 0; // cached vario mode
    uint8_t _cmode :1 = false; // cached cruise mode
    uint8_t _pending_change :1 = false; // to synch with sensor loop
    uint8_t _cm_changed :1 = false; // changed in this sensor loop cycle
    uint8_t _audio_vario :1 = true; // cached audio mode; true == vario mode, false == s2f mode
    uint8_t _audio_chopping :1 = true; // true for chopping mode (silent or dual)
    uint8_t _lock :1 = false; // lock the cruise mode
};


extern CruiseMode CRMOD;
