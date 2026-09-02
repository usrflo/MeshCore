#pragma once

// Corridor radius codes shared by the corridor check (CorridorCheck.h) and the
// on-node corridor proposal generator (CorridorPropose.h).  Kept free of any
// Arduino/mesh dependencies so native gtests can include it.

#include <stdint.h>

// Radius lookup table indexed by the 4-bit radius_code field.
// Code 15 means "unlimited" (FLT_MAX) — an always-inside anchor.
static const float CORRIDOR_RADIUS_KM[16] = {
    1.0f,   // 0
    2.0f,   // 1
    3.0f,   // 2
    5.0f,   // 3
    8.0f,   // 4
   12.0f,   // 5
   20.0f,   // 6
   30.0f,   // 7
   50.0f,   // 8
   80.0f,   // 9
  120.0f,   // 10
  200.0f,   // 11
  300.0f,   // 12
  500.0f,   // 13
  800.0f,   // 14
  3.4028235e+38f  // 15 — unlimited (FLT_MAX without including <float.h>)
};

#define CORRIDOR_RADIUS_UNLIMITED_KM  3.4028235e+38f
