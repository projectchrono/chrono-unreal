// Project Chrono include guard - END half. See ChronoIncludeBegin.h for usage.
//
// Deliberately has NO include guard / #pragma once.

#ifndef CHRONO_INCLUDE_GUARD_ACTIVE
	#error "ChronoIncludeEnd.h included without a preceding ChronoIncludeBegin.h"
#endif
#undef CHRONO_INCLUDE_GUARD_ACTIVE

#ifdef _MSC_VER
	#pragma warning(pop)
#endif

// <windows.h> / <winsock2.h> may have been pulled in (chrono/utils/ChSocket.h, chrono_thirdparty/filesystem/path.h,
// Bullet threading). Strip the function-name macros that break Unreal code that follows (e.g. UObject::GetObject,
// FGenericPlatformMisc::GetEnvironmentVariable, UAudioComponent::PlaySound, ...). Unreal itself does the same in
// Windows/HideWindowsPlatformTypes.h / MinWindows.h.
#undef min
#undef max
#undef near
#undef far
#undef small
#undef GetObject
#undef GetMessage
#undef PostMessage
#undef SendMessage
#undef DrawText
#undef DrawState
#undef LoadString
#undef LoadImage
#undef LoadIcon
#undef LoadCursor
#undef LoadBitmap
#undef GetClassName
#undef GetClassInfo
#undef GetCurrentTime
#undef GetTempPath
#undef GetTempFileName
#undef GetFileAttributes
#undef GetFullPathName
#undef GetLongPathName
#undef GetShortPathName
#undef GetEnvironmentVariable
#undef SetEnvironmentVariable
#undef GetUserName
#undef GetComputerName
#undef GetCommandLine
#undef GetFreeSpace
#undef GetProp
#undef SetProp
#undef RemoveProp
#undef GetForm
#undef SetForm
#undef PlaySound
#undef UpdateResource
#undef FindResource
#undef LoadLibrary
#undef FindText
#undef FindWindow
#undef CreateWindow
#undef CreateDialog
#undef CreateEvent
#undef CreateMutex
#undef CreateSemaphore
#undef CreateService
#undef CreateFont
#undef CreateFile
#undef CreateDirectory
#undef RemoveDirectory
#undef DeleteFile
#undef MoveFile
#undef CopyFile
#undef ReplaceFile
#undef OpenFile
#undef ReportEvent
#undef Yield
#undef IsMinimized
#undef IsMaximized
#undef GetGeoInfo
#undef GetCharWidth
#undef GetTextMetrics
#undef DeviceCapabilities
#undef EnumResourceTypes
#undef StartService
#undef GetNextSibling
#undef GetFirstChild
#undef GetDefaultPrinter
#undef LoadKeyboardLayout
#undef InterlockedIncrement
#undef InterlockedDecrement
#undef InterlockedExchange
#undef InterlockedCompareExchange
#undef MemoryBarrier
#undef ZeroMemory
#undef CopyMemory
#undef MoveMemory
#undef FillMemory
#undef CDECL
#undef DEBUG
#undef PI
#undef TEXT
#undef abs
#undef check
#undef verify
#undef ensure

// Restore Unreal's macros (reverse order of the pushes in ChronoIncludeBegin.h).
UE_POP_MACRO("InterlockedCompareExchange")
UE_POP_MACRO("InterlockedExchange")
UE_POP_MACRO("InterlockedDecrement")
UE_POP_MACRO("InterlockedIncrement")
UE_POP_MACRO("dynamic_cast")
UE_POP_MACRO("CONSTEXPR")
UE_POP_MACRO("FINAL")
UE_POP_MACRO("OVERRIDE")
UE_POP_MACRO("DEPRECATED")
UE_POP_MACRO("Assert")
UE_POP_MACRO("abs")
UE_POP_MACRO("max")
UE_POP_MACRO("min")
UE_POP_MACRO("DEBUG")
UE_POP_MACRO("PI")
UE_POP_MACRO("TEXT")
UE_POP_MACRO("ensureMsgf")
UE_POP_MACRO("ensure")
UE_POP_MACRO("verifyf")
UE_POP_MACRO("verify")
UE_POP_MACRO("checkf")
UE_POP_MACRO("check")

PRAGMA_ENABLE_REORDER_WARNINGS
PRAGMA_ENABLE_MISSING_VIRTUAL_DESTRUCTOR_WARNINGS
PRAGMA_RESTORE_SWITCH_UNHANDLED_ENUM_CASE_WARNINGS
PRAGMA_RESTORE_UNSAFE_TYPECAST_WARNINGS
PRAGMA_ENABLE_DEPRECATION_WARNINGS
PRAGMA_ENABLE_SHADOW_VARIABLE_WARNINGS
PRAGMA_ENABLE_UNDEFINED_IDENTIFIER_WARNINGS
THIRD_PARTY_INCLUDES_END
