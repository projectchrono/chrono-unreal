// Project Chrono include guard - BEGIN half.
//
// Deliberately has NO include guard / #pragma once: it is a bracket that must be re-evaluated every time.
//
//   #include "ChronoIncludeBegin.h"
//   #include "chrono/physics/ChSystemNSC.h"
//   ...
//   #include "ChronoIncludeEnd.h"
//
// Every #include "ChronoIncludeBegin.h" MUST be matched by exactly one #include "ChronoIncludeEnd.h".
//
// What it does:
//  * marks the region as third-party for Unreal (THIRD_PARTY_INCLUDES_START) and disables the warning
//    families Unreal escalates to errors (undefined identifiers, shadowing, deprecation, unsafe casts, ...);
//  * pushes and removes Unreal / Windows macros that collide with identifiers used by Chrono, Bullet, HACD
//    and Eigen (check, verify, ensure, min, max, PI, TEXT, DEBUG, ...);
//  * makes sure <windows.h> (pulled in via chrono/utils/ChSocket.h and chrono_thirdparty/filesystem) does
//    not define min/max and stays lean.
//
// The matching End header restores everything and additionally strips the <windows.h> function-name
// macros (GetObject, PlaySound, ...) so that subsequent Unreal headers are unaffected.

#ifdef CHRONO_INCLUDE_GUARD_ACTIVE
	#error "ChronoIncludeBegin.h included twice without a matching ChronoIncludeEnd.h"
#endif
#define CHRONO_INCLUDE_GUARD_ACTIVE 1

#include "HAL/Platform.h"
#include "Misc/CoreMiscDefines.h"

#ifndef NOMINMAX
	#define NOMINMAX
#endif

THIRD_PARTY_INCLUDES_START
PRAGMA_DISABLE_UNDEFINED_IDENTIFIER_WARNINGS
PRAGMA_DISABLE_SHADOW_VARIABLE_WARNINGS
PRAGMA_DISABLE_DEPRECATION_WARNINGS
PRAGMA_DISABLE_UNSAFE_TYPECAST_WARNINGS
PRAGMA_DISABLE_SWITCH_UNHANDLED_ENUM_CASE_WARNINGS
PRAGMA_DISABLE_MISSING_VIRTUAL_DESTRUCTOR_WARNINGS
PRAGMA_DISABLE_REORDER_WARNINGS

// Unreal macros that clash with Chrono / Bullet / HACD / Eigen identifiers.
UE_PUSH_MACRO("check")
UE_PUSH_MACRO("checkf")
UE_PUSH_MACRO("verify")
UE_PUSH_MACRO("verifyf")
UE_PUSH_MACRO("ensure")
UE_PUSH_MACRO("ensureMsgf")
UE_PUSH_MACRO("TEXT")
UE_PUSH_MACRO("PI")
UE_PUSH_MACRO("DEBUG")
UE_PUSH_MACRO("min")
UE_PUSH_MACRO("max")
UE_PUSH_MACRO("abs")
UE_PUSH_MACRO("Assert")
UE_PUSH_MACRO("DEPRECATED")
UE_PUSH_MACRO("OVERRIDE")
UE_PUSH_MACRO("FINAL")
UE_PUSH_MACRO("CONSTEXPR")
UE_PUSH_MACRO("dynamic_cast")
UE_PUSH_MACRO("InterlockedIncrement")
UE_PUSH_MACRO("InterlockedDecrement")
UE_PUSH_MACRO("InterlockedExchange")
UE_PUSH_MACRO("InterlockedCompareExchange")
#undef check
#undef checkf
#undef verify
#undef verifyf
#undef ensure
#undef ensureMsgf
#undef TEXT
#undef PI
#undef DEBUG
#undef min
#undef max
#undef abs
#undef Assert
#undef DEPRECATED
#undef OVERRIDE
#undef FINAL
#undef CONSTEXPR
#undef dynamic_cast   // CoreUObject/Templates/Casts.h redefines it; Chrono needs the real operator (consumers must enable RTTI)
#undef InterlockedIncrement
#undef InterlockedDecrement
#undef InterlockedExchange
#undef InterlockedCompareExchange

#ifdef _MSC_VER
	#pragma warning(push)
	// Chrono's own CMake suppressions ...
	#pragma warning(disable: 4100 4127 4201 4244 4250 4251 4275 4324 4505 5038 26451 26495 26812)
	// ... plus the ones Unreal's /W4-like settings add on top for Chrono / Bullet / Eigen headers.
	#pragma warning(disable: 4018 4189 4245 4263 4264 4265 4267 4305 4310 4389 4456 4457 4458 4459 4592 4611 4619 4623 4625 4626 4668 4701 4702 4703 4706 4800 4828 4946 4996 5026 5027 5054 5205 5219 5267)
#endif
