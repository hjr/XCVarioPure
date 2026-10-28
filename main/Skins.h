#pragma once

#include "Colors.h"   // pulls in skin_t, skin_rgb_t, g_skin

// Number of built-in skins
static constexpr int SKIN_COUNT = 9;

// Array of pointers to all built-in skins (index == NVS value)
extern const skin_t* const SKIN_PRESETS[SKIN_COUNT];

// Human-readable names (same index order as SKIN_PRESETS)
extern const char* const SKIN_NAME[SKIN_COUNT];

// Apply skin[index] to g_skin.  Clamps index to [0, SKIN_COUNT-1].
void skin_apply(int index);
