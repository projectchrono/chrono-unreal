#pragma once

#include "CoreMinimal.h"
#include "Common.h"
#include "ChronoConversions.h"

// Unreal <-> Chrono conversions for NextGenCars.
//
// The scale / frame definition (cm <-> m, mirrored Y axis, quaternion mapping) lives in the ChronoCore plugin
// (FChronoConversions). This header only forwards to it and adds the NextGenCars specific helpers
// (ChronoTransform, unit constants), so both code bases share a single definition.

//Even though both unreal and chrono are set to use metres as units, unreal's default units are still CM under the hood,
// and it only converts to units within the editor viewport. So need to convert CM to M here also
constexpr double UE2CH_SCALE = FChronoConversions::UE2CH_SCALE;
constexpr double CH2UE_SCALE = FChronoConversions::CH2UE_SCALE;

constexpr float MetresPerSecondToKPH = FChronoConversions::MetresPerSecondToKPH;
constexpr float MetresPerSecondToMPH = FChronoConversions::MetresPerSecondToMPH;
constexpr float MPHtoMetresPerSecond = FChronoConversions::MPHToMetresPerSecond;
constexpr float KPHtoMetresPerSecond = FChronoConversions::KPHToMetresPerSecond;
constexpr float CMToKms = FChronoConversions::CMToKms;
constexpr float CMtoMiles = FChronoConversions::CMToMiles;
constexpr float CMtoMetres = FChronoConversions::CMToMetres;
constexpr float InchToCM = FChronoConversions::InchToCM;
constexpr float CMToInch = FChronoConversions::CMToInch;
constexpr float MetresToMiles = FChronoConversions::MetresToMiles;
constexpr float MetresToKms = FChronoConversions::MetresToKms;
constexpr float KmsToMetres = FChronoConversions::KmsToMetres;
constexpr float MilesToMeters = FChronoConversions::MilesToMetres;
constexpr float MilesToKms = FChronoConversions::MilesToKms;
constexpr float KmsToMiles = FChronoConversions::KmsToMiles;
constexpr float GallonsToLitres = FChronoConversions::GallonsToLitres;
constexpr float LitresToGallons = FChronoConversions::LitresToGallons;

class Conversions
{
public:
	
	static inline void UnrealVecToChrono(const FVector& UnrealVec, ChVector3d& ChronoVec)
	{
		ChronoVec = FChronoConversions::ToChronoDir(UnrealVec);
	}

	static inline Vector UnrealVecToChrono(const FVector& UnrealVec)
	{
		return FChronoConversions::ToChronoDir(UnrealVec);
	}

	static inline double ToChronoSize(double size)
	{
		return FChronoConversions::ToChronoSize(size);
	}

	static inline double ToUnrealSize(double size)
	{
		return FChronoConversions::ToUnrealSize(size);
	}	

	static inline FColor ToUnrealColor(const ChColor& ChronoColor)
	{
		return FChronoConversions::ToUnrealColor(ChronoColor);
	}

	static inline FVector ToUnrealPos(const ChVector3d& Vec)
	{
		return FChronoConversions::ToUnrealPos(Vec);
	}

	static inline FVector ToUnrealNormal(const ChVector3d& Vec)
	{
		return FChronoConversions::ToUnrealNormal(Vec);
	}

	static inline FQuat ToUnrealQuat(const QuaternionF& ChronoQuat)
	{
		return FChronoConversions::ToUnrealQuat(ChronoQuat);
	}

	static inline FQuat ToUnrealQuat(const Quaternion& ChronoQuat)
	{
		return FChronoConversions::ToUnrealQuat(ChronoQuat);
	}

	static inline Vector ToChronoVec(const FVector& Vec)
	{
		return FChronoConversions::ToChronoPos(Vec);
	}

	static inline Vector ToChronoNormal(const FVector& Vec)
	{
		return FChronoConversions::ToChronoNormal(Vec);
	}

	/// Unit-less direction: flip Y only (no cm -> m scaling, directions are normalised by the caller).
	static inline Vector ToChronoDir(const FVector& Vec)
	{
		return FChronoConversions::ToChronoDir(Vec);
	}

	static inline Vector ToChronoScale(const FVector& Vec)
	{
		return FChronoConversions::ToChronoScale(Vec);
	}

	static inline Quaternion ToChronoQuat(const FQuat& Rotation)
	{
		return FChronoConversions::ToChronoQuat(Rotation);
	}

	static inline ChronoTransform ToChronoTransform(const FTransform& UnrealTransform)
	{
		return ChronoTransform(Conversions::ToChronoVec(UnrealTransform.GetLocation()), Conversions::ToChronoQuat(UnrealTransform.GetRotation()));
	}

	static inline FTransform ToUETransform(const ChronoTransform& ChTx)
	{
		return FTransform(Conversions::ToUnrealQuat(ChTx.Rotation), Conversions::ToUnrealPos(ChTx.Location));
	}

	static inline FTransform ToUETransform(const chrono::ChCoordsysd& ChCoords)
	{
		return FChronoConversions::ToUnrealTransform(ChCoords);
	}
};
