/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#pragma once

#include "math/Units.h"
#include "math/Floats.h"

#include <cstdint>

enum class WindReference : uint8_t { WR_HEADING, WR_NORTH };

// A compact 32 bit representation of wind direction and strength used internally
// - direction in deg/2 is a fix point with one bit mantissa, i.e. 0..719°
// - value in m/s is the wind speed with three bit mantissa
union WindData
{
    struct {
        uint16_t dir; // 0..719° in [0,5°], northwind as 0, e.g. south wind as 360
        uint16_t val; // internally 3bit fix point [m/s]; could be any unit, e.g. m/s or km/h; 8 bit alias 230km/h might be enough range
        // 8 bit might by used as flegs here, e.g. for live/tas or other flags, but currently unused
    } __attribute__((packed));
    uint32_t raw = 0xffff;

    // ctors
    constexpr WindData() = default;
    constexpr WindData(uint32_t d) : raw(d) {}
    constexpr WindData(int16_t wdir, int16_t wval) : dir(wdir), val(wval) {}
    constexpr WindData(rad_t wdir, mps_t wval) : dir(fast_iroundf(Units::rad_to_deg(wdir) * 2) % 720), val(fast_iroundf(wval * 8)) {}
    // getters
    constexpr int getDeg2() const { return dir; }
    constexpr int getVDeg2() const { return dir + 360; } // get wind vector direction -> +180° shift
    constexpr int getDeg() const { return dir/2; }
    constexpr mps_t getVal() const { return static_cast<float>(val) / 8.0f; }
    constexpr bool isValid() const { return raw != 0xffff; }
    constexpr int data() const { return raw; }
    // setters
    constexpr void setDeg2(int16_t d) { dir = d % 720; }
    constexpr void setKmh(kmh_t v) { val = fast_iroundf(v * Units::kmh_to_mps(8)); }
    constexpr void inclHeading(rad_t d) { dir = (dir - fast_iroundf(Units::rad_to_deg(d) * 2)) % 720; }
    // compare
    constexpr bool operator==(WindData other) const { return raw == other.raw; }
    constexpr bool operator!=(WindData other) const { return raw != other.raw; }
};