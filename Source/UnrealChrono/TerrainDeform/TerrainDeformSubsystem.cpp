#include "TerrainDeformSubsystem.h"

#include "Engine/Texture2D.h"
#include "Engine/TextureRenderTarget2D.h"
#include "Engine/World.h"
#include "GameFramework/PlayerController.h"
#include "Kismet/KismetMaterialLibrary.h"
#include "Materials/MaterialParameterCollection.h"
#include "TerrainDeformPhysicsCache.h"
#include "TerrainDeformRender.h"
#include "TerrainDeformSettings.h"
#include "TextureResource.h"
#if WITH_EDITOR
#include "TextureCompiler.h"
#endif

namespace
{
	// A window only re-centres in steps of this many texels so strip refreshes stay rare.
	constexpr int32 CenterSnapTexels = 32;

	UTextureRenderTarget2D* MakeTarget(UObject* Outer, ETextureRenderTargetFormat Format, int32 Size, const TCHAR* Name)
	{
		UTextureRenderTarget2D* RT = NewObject<UTextureRenderTarget2D>(Outer, Name);
		RT->RenderTargetFormat = Format;
		RT->bSupportsUAV = true;
		RT->bCanCreateUAV = true;
		RT->bAutoGenerateMips = false;
		RT->AddressX = TA_Wrap;
		RT->AddressY = TA_Wrap;
		RT->ClearColor = FLinearColor::Black;
		RT->InitAutoFormat(Size, Size);
		RT->UpdateResourceImmediate(true);
		return RT;
	}

	/** Uses the asset if it is the right size and format (enabling UAV on it), otherwise returns null. */
	UTextureRenderTarget2D* UsableAsset(UTextureRenderTarget2D* RT, ETextureRenderTargetFormat Format, int32 Size)
	{
		if (!RT)
		{
			return nullptr;
		}
		// Resize in memory when the asset differs. Setting the size through the editor would pop a modal
		// "are you sure?" dialog above 2048, so assign the members directly and recreate the resource.
		// bSupportsUAV is what CreateResource() turns into bCanCreateUAV.
		const bool bWrongSize = RT->SizeX != Size || RT->SizeY != Size;
		const bool bWrongFormat = RT->RenderTargetFormat != Format;
		if (bWrongSize || bWrongFormat || !RT->bSupportsUAV || !RT->bCanCreateUAV)
		{
			if (bWrongSize || bWrongFormat)
			{
				UE_LOG(LogTemp, Display, TEXT("TerrainDeform: resizing render target %s from %dx%d to %dx%d in memory."),
					*RT->GetName(), RT->SizeX, RT->SizeY, Size, Size);
			}
			RT->SizeX = Size;
			RT->SizeY = Size;
			RT->RenderTargetFormat = Format;
			RT->bSupportsUAV = true;
			RT->bCanCreateUAV = true;
			RT->UpdateResource();
		}
		return RT;
	}
}

bool UTerrainDeformSubsystem::ShouldCreateSubsystem(UObject* Outer) const
{
	const UWorld* World = Cast<UWorld>(Outer);
	return World && (World->IsGameWorld()) && !IsRunningDedicatedServer();
}

void UTerrainDeformSubsystem::Initialize(FSubsystemCollectionBase& Collection)
{
	Super::Initialize(Collection);

	const UTerrainDeformSettings* Config = GetDefault<UTerrainDeformSettings>();

	UTextureRenderTarget2D* HeightAssets[FTerrainDeformSettings::NumLayers] = { Config->HeightTarget.LoadSynchronous(), Config->FineHeightTarget.LoadSynchronous() };
	UTextureRenderTarget2D* NormalAssets[FTerrainDeformSettings::NumLayers] = { Config->NormalTarget.LoadSynchronous(), Config->FineNormalTarget.LoadSynchronous() };
	for (int32 L = 0; L < FTerrainDeformSettings::NumLayers; ++L)
	{
		const int32 Size = Settings.Layers[L].WindowSize;
		HeightTexture[L] = UsableAsset(HeightAssets[L], RTF_R32f, Size);
		NormalTexture[L] = UsableAsset(NormalAssets[L], RTF_RGBA8, Size);
		if (!HeightTexture[L])
		{
			HeightTexture[L] = MakeTarget(this, RTF_R32f, Size, *FString::Printf(TEXT("TerrainDeformHeight%d"), L));
		}
		if (!NormalTexture[L])
		{
			NormalTexture[L] = MakeTarget(this, RTF_RGBA8, Size, *FString::Printf(TEXT("TerrainDeformNormal%d"), L));
		}
	}

	TreadTexture = Config->TreadTexture.LoadSynchronous();
	if (!TreadTexture)
	{
		UE_LOG(LogTemp, Warning, TEXT("TerrainDeform: tread texture not found; tracks will have no tread."));
	}
	else
	{
#if WITH_EDITOR
		// Imported textures compile asynchronously and show a 32x32 placeholder until done. Wait, so the stamp
		// shader never samples the placeholder.
		FTextureCompilingManager::Get().FinishCompilation({ TreadTexture });
#endif
		UE_LOG(LogTemp, Display, TEXT("TerrainDeform: tread texture %s %dx%d, resource %s"), *TreadTexture->GetName(),
			TreadTexture->GetSizeX(), TreadTexture->GetSizeY(), TreadTexture->GetResource() ? TEXT("present") : TEXT("MISSING"));
	}
	ParameterCollection = Config->ParameterCollection.LoadSynchronous();

	ViewExtension = FSceneViewExtensions::NewExtension<FTerrainDeformViewExtension>(GetWorld(), Settings);
	// Pass the resources, not their RHI textures: those are created on the render thread and may not exist yet.
	for (int32 L = 0; L < FTerrainDeformSettings::NumLayers; ++L)
	{
		ViewExtension->SetOutputResources(L,
			HeightTexture[L]->GameThread_GetRenderTargetResource(),
			NormalTexture[L]->GameThread_GetRenderTargetResource());
	}
	// The texture's reference handle, not its resource: the handle stays valid if the resource is replaced.
	ViewExtension->SetTreadTexture(TreadTexture ? TreadTexture->TextureReference.TextureReferenceRHI : FTextureReferenceRHIRef());
}

void UTerrainDeformSubsystem::Deinitialize()
{
	// The sweep task uses the view extension; let it finish first.
	SweepTask.Wait();
	ViewExtension.Reset();
	FTerrainDeformPhysicsCache::Clear();
	for (int32 L = 0; L < FTerrainDeformSettings::NumLayers; ++L)
	{
		HeightTexture[L] = nullptr;
		NormalTexture[L] = nullptr;
	}
	TreadTexture = nullptr;
	Super::Deinitialize();
}

TStatId UTerrainDeformSubsystem::GetStatId() const
{
	RETURN_QUICK_DECLARE_CYCLE_STAT(UTerrainDeformSubsystem, STATGROUP_Tickables);
}

void UTerrainDeformSubsystem::AddStamp(const FTerrainDeformStamp& Stamp)
{
	FScopeLock Lock(&StampLock);
	PendingStamps.Add(Stamp);
}

void UTerrainDeformSubsystem::AddTrackStamp(int32 SourceId, const FTerrainDeformStamp& Stamp, bool bAllowWhenStationary)
{
	FScopeLock Lock(&StampLock);
	FTrackRequest& Request = PendingTrackRequests.AddDefaulted_GetRef();
	Request.SourceId = SourceId;
	Request.Stamp = Stamp;
	Request.bAllowWhenStationary = bAllowWhenStationary;
}

void UTerrainDeformSubsystem::SweepTrackRequests(TArray<FTrackRequest>&& Requests)
{
	// Longest straight run one stamp may sweep; keeps the swept footprint plus its berm inside the stamp tile.
	constexpr float MaxSegmentCm = 60.f;
	// Beyond this the source teleported (respawn, reset); don't smear a track across the gap.
	constexpr float TeleportCm = 500.f;
	// Movement below this (in fine texels) counts as stationary.
	const float MinMoveCm = 0.5f * Settings.Layers[FTerrainDeformSettings::FineLayer].TexelSize;

	TArray<FTerrainDeformStamp> Out;
	Out.Reserve(Requests.Num() * 2);
	for (const FTrackRequest& Request : Requests)
	{
		const FTerrainDeformStamp& Stamp = Request.Stamp;
		const int32 SourceId = Request.SourceId;
		const FVector2f Now = Stamp.Center;
		const FVector2f* Last = LastTrackPosition.Find(SourceId);
		if (!Last || FVector2f::Distance(*Last, Now) > TeleportCm)
		{
			LastTrackPosition.Add(SourceId, Now);
			Out.Add(Stamp);
			continue;
		}

		const FVector2f Delta = Now - *Last;
		const float Dist = Delta.Size();
		if (Dist < MinMoveCm)
		{
			// Effectively stationary. Keep the old anchor so slow creep still accumulates into a swept segment, and only
			// stamp in place when the wheel is spinning (digging in).
			if (Request.bAllowWhenStationary)
			{
				Out.Add(Stamp);
			}
			continue;
		}

		float& PathDistance = TrackPathDistance.FindOrAdd(SourceId);
		FTerrainDeformStamp Swept = Stamp;
		Swept.Forward = Delta / Dist;
		const int32 Pieces = FMath::Clamp(FMath::CeilToInt(Dist / MaxSegmentCm), 1, 8);
		const float PieceLength = Dist / Pieces;
		Swept.SegmentHalf = 0.5f * PieceLength;
		for (int32 Piece = 0; Piece < Pieces; ++Piece)
		{
			Swept.Center = *Last + Delta * ((Piece + 0.5f) / Pieces);
			Swept.PathStart = PathDistance + Piece * PieceLength;
			Out.Add(Swept);
		}
		PathDistance += Dist;
		LastTrackPosition.Add(SourceId, Now);
	}

	if (ViewExtension.IsValid())
	{
		ViewExtension->AppendStamps(MoveTemp(Out));
	}
}

void UTerrainDeformSubsystem::AddDebugStamp(FVector2D Center, FVector2D Forward, float HalfLength, float HalfWidth, float Sinkage)
{
	FTerrainDeformStamp Stamp;
	Stamp.Center = FVector2f(Center);
	Stamp.Forward = FVector2f(Forward);
	Stamp.HalfLength = HalfLength;
	Stamp.HalfWidth = HalfWidth;
	Stamp.Sinkage = Sinkage;
	AddStamp(Stamp);
}

void UTerrainDeformSubsystem::RequestVolumeCheck(int32 Layer)
{
	if (ViewExtension.IsValid())
	{
		ViewExtension->RequestVolumeCheck(Layer);
	}
}

bool UTerrainDeformSubsystem::GetLastVolumeStats(int64& OutSum, int32& OutMin, int32& OutMax, int32& OutNonZero) const
{
	OutSum = 0; OutMin = 0; OutMax = 0; OutNonZero = 0;
	return ViewExtension.IsValid() && ViewExtension->GetLastStats(OutSum, OutMin, OutMax, OutNonZero);
}

void UTerrainDeformSubsystem::SetFocusLocation(FVector WorldLocation)
{
	FocusLocation = WorldLocation;
	bHasFocus = true;
}

void UTerrainDeformSubsystem::Tick(float DeltaTime)
{
	if (!ViewExtension.IsValid())
	{
		return;
	}

	FVector Focus = FocusLocation;
	if (!bHasFocus)
	{
		const APlayerController* PC = GetWorld()->GetFirstPlayerController();
		if (!PC || !PC->GetViewTarget())
		{
			return;
		}
		Focus = PC->GetViewTarget()->GetActorLocation();
	}

	FTerrainDeformFrame Frame;
	FVector2D CenterWorld[FTerrainDeformSettings::NumLayers];
	for (int32 L = 0; L < FTerrainDeformSettings::NumLayers; ++L)
	{
		const float TexelSize = Settings.Layers[L].TexelSize;
		const float Step = TexelSize * CenterSnapTexels;
		Frame.CenterTexel[L] = FIntPoint(
			FMath::RoundToInt(Focus.X / Step) * CenterSnapTexels,
			FMath::RoundToInt(Focus.Y / Step) * CenterSnapTexels);
		CenterWorld[L] = FVector2D(Frame.CenterTexel[L].X, Frame.CenterTexel[L].Y) * TexelSize;
	}
	{
		FScopeLock Lock(&StampLock);
		Frame.Stamps = MoveTemp(PendingStamps);
		PendingStamps.Reset();
	}
	ViewExtension->Submit(MoveTemp(Frame));

	// Footprint sweeping runs on a worker. One task at a time (it owns the per source history); while it is busy the
	// requests stay queued, nothing is dropped. The window centres above are already submitted, so a late task is harmless.
	if (SweepTask.IsCompleted())
	{
		TArray<FTrackRequest> Requests;
		{
			FScopeLock Lock(&StampLock);
			Requests = MoveTemp(PendingTrackRequests);
			PendingTrackRequests.Reset();
		}
		if (Requests.Num() > 0)
		{
			SweepTask = UE::Tasks::Launch(TEXT("TerrainDeform.SweepTracks"), [this, Requests = MoveTemp(Requests)]() mutable
			{
				SweepTrackRequests(MoveTemp(Requests));
			});
		}
	}

	if (ParameterCollection)
	{
		const FTerrainDeformLayerConfig& Coarse = Settings.Layers[FTerrainDeformSettings::CoarseLayer];
		const FTerrainDeformLayerConfig& Fine = Settings.Layers[FTerrainDeformSettings::FineLayer];
		// DeformCenter = (coarse x, coarse y, fine x, fine y); DeformLayers = (coarse texel, coarse window, fine texel, fine window).
		UKismetMaterialLibrary::SetVectorParameterValue(this, ParameterCollection, TEXT("DeformCenter"),
			FLinearColor(CenterWorld[0].X, CenterWorld[0].Y, CenterWorld[1].X, CenterWorld[1].Y));
		UKismetMaterialLibrary::SetVectorParameterValue(this, ParameterCollection, TEXT("DeformLayers"),
			FLinearColor(Coarse.TexelSize, Coarse.WindowSize, Fine.TexelSize, Fine.WindowSize));
	}
}
