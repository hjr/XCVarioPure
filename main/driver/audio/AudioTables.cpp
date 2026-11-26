/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "AudioTables.h"

#if __has_include(<esp_attr.h>)
#include <esp_attr.h>
#endif
#ifndef DRAM_ATTR
#define DRAM_ATTR
#endif

namespace audiotab {

namespace {
constexpr std::array<int16_t, RC_LEN> makeRc()
{
    std::array<int16_t, RC_LEN> t{};
    for (int i = 0; i < RC_LEN; i++) {
        t[i] = (int16_t)cround(32767. * 0.5 * (1. - ccos(PI * i / (RC_LEN - 1))));
    }
    return t;
}
constexpr std::array<uint32_t, EXP2_LEN> makeExp2()
{
    std::array<uint32_t, EXP2_LEN> t{};
    for (int i = 0; i < EXP2_LEN; i++) {
        const long long v = cround(1073741824. * cexp2((double)i / (EXP2_LEN - 1)));
        t[i] = (uint32_t)(v > 2147483647LL ? 2147483647LL : v);
    }
    return t;
}
} // namespace

const std::array<int16_t, SIN_LEN> SIN_Q15 = makeSine<int16_t, SIN_LEN>(32767.);
// read per sample chunk in the DMA interrupt: keep in internal RAM
DRAM_ATTR const std::array<int16_t, RC_LEN> RC_Q15 = makeRc();
DRAM_ATTR const std::array<uint32_t, EXP2_LEN> EXP2_Q30 = makeExp2();

} // namespace audiotab
