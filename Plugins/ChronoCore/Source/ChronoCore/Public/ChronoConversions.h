// Conversions between Unreal and Project Chrono conventions.
//
//  Unreal : left-handed,  Z up, centimetres, FQuat(X, Y, Z, W)
//  Chrono : right-handed, Z up, metres,      ChQuaternion(e0 = w, e1 = x, e2 = y, e3 = z)
//
// The two frames are related by mirroring the Y axis. Positions are scaled (cm <-> m) and Y-flipped;
// directions/normals are only Y-flipped; a rotation reflected through the XZ plane keeps w and y and negates
// x and z, so quaternions map as  FQuat(x, y, z, w)  <->  ChQuaternion(w, -x, y, -z).
// All conversions are exact inverses of each other (round trips reproduce the input component-wise).

#pragma once

#include "CoreMinimal.h"
#include "Math/Color.h"
#include "Math/Quat.h"
#include "Math/Transform.h"
#include "Math/Vector.h"

#include "ChronoIncludeBegin.h"
#include "chrono/core/ChVector3.h"
#include "chrono/core/ChQuaternion.h"
#include "chrono/core/ChCoordsys.h"
#include "chrono/core/ChFrame.h"
#include "chrono/assets/ChColor.h"
#include "ChronoIncludeEnd.h"

struct FChronoConversions
{
	// -------------------------------------------------------------------------
	// Scale factors and unit constants
	// -------------------------------------------------------------------------

	/// Unreal centimetres -> Chrono metres.
	static constexpr double UE2CH_SCALE = 0.01;
	/// Chrono metres -> Unreal centimetres.
	static constexpr double CH2UE_SCALE = 100.0;

	static constexpr float MetresPerSecondToKPH = 3.6f;
	static constexpr float MetresPerSecondToMPH = 2.23694f;
	static constexpr float MPHToMetresPerSecond = 0.44704f;
	static constexpr float KPHToMetresPerSecond = 0.277778f;
	static constexpr float CMToKms = 0.00001f;
	static constexpr float CMToMiles = 0.0000062137f;
	static constexpr float CMToMetres = 0.01f;
	static constexpr float MetresToCM = 100.0f;
	static constexpr float InchToCM = 2.54f;
	static constexpr float CMToInch = 0.39370079f;
	static constexpr float MetresToMiles = 0.000621371f;
	static constexpr float MetresToKms = 0.001f;
	static constexpr float KmsToMetres = 1000.0f;
	static constexpr float MilesToMetres = 1609.34f;
	static constexpr float MilesToKms = 1.60934f;
	static constexpr float KmsToMiles = 0.621371f;
	static constexpr float GallonsToLitres = 3.78541f;
	static constexpr float LitresToGallons = 0.264172f;

	// -------------------------------------------------------------------------
	// Scalars
	// -------------------------------------------------------------------------

	/// Length: Unreal cm -> Chrono m.
	static FORCEINLINE double ToChronoSize(double Size) { return Size * UE2CH_SCALE; }
	/// Length: Chrono m -> Unreal cm.
	static FORCEINLINE double ToUnrealSize(double Size) { return Size * CH2UE_SCALE; }

	// -------------------------------------------------------------------------
	// Vectors
	// -------------------------------------------------------------------------

	/// Position / length vector: scale cm -> m and flip Y.
	static FORCEINLINE chrono::ChVector3d ToChronoPos(const FVector& V)
	{
		return chrono::ChVector3d(ToChronoSize(V.X), ToChronoSize(-V.Y), ToChronoSize(V.Z));
	}

	/// Unit-less direction / normal / angular velocity: flip Y only.
	static FORCEINLINE chrono::ChVector3d ToChronoDir(const FVector& V)
	{
		return chrono::ChVector3d(V.X, -V.Y, V.Z);
	}

	/// Alias of ToChronoDir for readability.
	static FORCEINLINE chrono::ChVector3d ToChronoNormal(const FVector& V) { return ToChronoDir(V); }

	/// Scale factors: no flip, no unit change.
	static FORCEINLINE chrono::ChVector3d ToChronoScale(const FVector& V)
	{
		return chrono::ChVector3d(V.X, V.Y, V.Z);
	}

	/// Position / length vector: scale m -> cm and flip Y.
	static FORCEINLINE FVector ToUnrealPos(const chrono::ChVector3d& V)
	{
		return FVector(ToUnrealSize(V.x()), ToUnrealSize(-V.y()), ToUnrealSize(V.z()));
	}

	/// Unit-less direction / normal / angular velocity: flip Y only.
	static FORCEINLINE FVector ToUnrealDir(const chrono::ChVector3d& V)
	{
		return FVector(V.x(), -V.y(), V.z());
	}

	/// Alias of ToUnrealDir for readability.
	static FORCEINLINE FVector ToUnrealNormal(const chrono::ChVector3d& V) { return ToUnrealDir(V); }

	/// Scale factors: no flip, no unit change.
	static FORCEINLINE FVector ToUnrealScale(const chrono::ChVector3d& V)
	{
		return FVector(V.x(), V.y(), V.z());
	}

	// -------------------------------------------------------------------------
	// Rotations
	// -------------------------------------------------------------------------

	/// FQuat(x, y, z, w) -> ChQuaternion(w, -x, y, -z)
	static FORCEINLINE chrono::ChQuaterniond ToChronoQuat(const FQuat& Q)
	{
		return chrono::ChQuaterniond(Q.W, -Q.X, Q.Y, -Q.Z);
	}

	/// ChQuaternion(e0, e1, e2, e3) -> FQuat(-e1, e2, -e3, e0)
	static FORCEINLINE FQuat ToUnrealQuat(const chrono::ChQuaterniond& Q)
	{
		return FQuat(-Q.e1(), Q.e2(), -Q.e3(), Q.e0());
	}

	static FORCEINLINE FQuat ToUnrealQuat(const chrono::ChQuaternionf& Q)
	{
		return FQuat(-Q.e1(), Q.e2(), -Q.e3(), Q.e0());
	}

	// -------------------------------------------------------------------------
	// Transforms (scale is not representable in Chrono frames and is ignored)
	// -------------------------------------------------------------------------

	static FORCEINLINE chrono::ChCoordsysd ToChronoCoordsys(const FTransform& T)
	{
		return chrono::ChCoordsysd(ToChronoPos(T.GetLocation()), ToChronoQuat(T.GetRotation()));
	}

	static FORCEINLINE chrono::ChFramed ToChronoFrame(const FTransform& T)
	{
		return chrono::ChFramed(ToChronoPos(T.GetLocation()), ToChronoQuat(T.GetRotation()));
	}

	static FORCEINLINE FTransform ToUnrealTransform(const chrono::ChCoordsysd& C)
	{
		return FTransform(ToUnrealQuat(C.rot), ToUnrealPos(C.pos));
	}

	static FORCEINLINE FTransform ToUnrealTransform(const chrono::ChFramed& F)
	{
		return FTransform(ToUnrealQuat(F.GetRot()), ToUnrealPos(F.GetPos()));
	}

	// -------------------------------------------------------------------------
	// Colours (ChColor channels are floats in [0,1])
	// -------------------------------------------------------------------------

	static FORCEINLINE FLinearColor ToUnrealLinearColor(const chrono::ChColor& C)
	{
		return FLinearColor(C.R, C.G, C.B, 1.0f);
	}

	static FORCEINLINE FColor ToUnrealColor(const chrono::ChColor& C)
	{
		return ToUnrealLinearColor(C).ToFColor(/*bSRGB=*/ true);
	}

	static FORCEINLINE chrono::ChColor ToChronoColor(const FLinearColor& C)
	{
		return chrono::ChColor(C.R, C.G, C.B);
	}

	static FORCEINLINE chrono::ChColor ToChronoColor(const FColor& C)
	{
		return ToChronoColor(FLinearColor(C));
	}
};
