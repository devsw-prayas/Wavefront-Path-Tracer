#pragma once

// ----------------------------------------------------------------------------------
// This macro definition is to make sure both VS22/26 as well
// as VScode detect AVX2 enabled code and higlight appropirately
// ----------------------------------------------------------------------------------

#if defined(__INTELLISENSE__) || (defined(_MSC_VER) && !defined(__AVX2__))
#define EDITOR_MODE 1
#endif


// ----------------------------------------------------------------------------------
// Paste this code inside a vectorized source or header to make
// sure Intellisense higlights it correctly
//
// #if !defined(__AVX2__) && !defined(EDITOR_MODE)
// #error "SIMD Vector Backend requires AVX2 flag to be enabled during compilation"
// #else
//
// Code.....
// #endif
//
// ----------------------------------------------------------------------------------

#define WF_FORCEINLINE __forceinline
#define WF_UNREACHABLE() __assume(0)
#define WF_FALLTHROUGH [[fallthrough]]


// ----------------------------------------------------------------------------------
// Standard Library Headers (for PCH)
// ----------------------------------------------------------------------------------
#include <cstddef>
#include <cstdint>
#include <cmath>
#include <numbers>
#include <algorithm>
#include <memory>
#include <vector>
#include <string>
#include <fstream>
#include <climits>
#include <chrono>
#include <thread>
