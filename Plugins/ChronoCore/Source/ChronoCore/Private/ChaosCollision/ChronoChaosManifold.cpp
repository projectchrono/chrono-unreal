// FChronoManifoldCache - persistent contact manifolds with reaction-cache carry-over.

#include "ChaosCollision/ChronoChaosManifold.h"

void FChronoManifoldCache::Update(const FChronoManifoldKey& Key, const ChronoChaosNarrowphase::FContactArray& Fresh, const FTransform& ATM, double MatchDistanceCm, int32 RunSerial)
{
	if (Fresh.Num() == 0)
	{
		Manifolds.Remove(Key);
		return;
	}

	FChronoManifold& Manifold = Manifolds.FindOrAdd(Key);
	const TArray<FChronoManifoldPoint, TInlineAllocator<4>> Previous = MoveTemp(Manifold.Points);
	Manifold.Points.Reset(Fresh.Num());
	Manifold.LastRunSerial = RunSerial;

	const double MatchDistanceSq = MatchDistanceCm * MatchDistanceCm;
	TArray<bool, TInlineAllocator<4>> PreviousUsed;
	PreviousUsed.Init(false, Previous.Num());

	for (const ChronoChaosNarrowphase::FContact& Contact : Fresh)
	{
		FChronoManifoldPoint& Point = Manifold.Points.AddDefaulted_GetRef();
		Point.PointA = Contact.PointA;
		Point.PointB = Contact.PointB;
		Point.Normal = Contact.Normal;
		Point.Separation = Contact.Separation;
		Point.FaceIndexB = Contact.FaceIndexB;
		Point.LocalA = ATM.InverseTransformPositionNoScale(Contact.PointA);

		// Carry the reaction cache over from the closest previous point on A (each previous point is used once).
		int32 Best = INDEX_NONE;
		double BestDistSq = MatchDistanceSq;
		for (int32 i = 0; i < Previous.Num(); ++i)
		{
			if (PreviousUsed[i])
			{
				continue;
			}
			const double DistSq = FVector::DistSquared(Previous[i].LocalA, Point.LocalA);
			if (DistSq <= BestDistSq)
			{
				BestDistSq = DistSq;
				Best = i;
			}
		}
		if (Best != INDEX_NONE)
		{
			PreviousUsed[Best] = true;
			FMemory::Memcpy(Point.ReactionCache, Previous[Best].ReactionCache, sizeof(Point.ReactionCache));
		}
	}
}

void FChronoManifoldCache::Purge(int32 RunSerial)
{
	for (TMap<FChronoManifoldKey, FChronoManifold>::TIterator It(Manifolds); It; ++It)
	{
		if (It.Value().LastRunSerial != RunSerial)
		{
			It.RemoveCurrent();
		}
	}
}

int32 FChronoManifoldCache::NumPoints() const
{
	int32 Count = 0;
	for (const TPair<FChronoManifoldKey, FChronoManifold>& Pair : Manifolds)
	{
		Count += Pair.Value.Points.Num();
	}
	return Count;
}

void FChronoManifoldCache::RemoveObject(const void* Object)
{
	for (TMap<FChronoManifoldKey, FChronoManifold>::TIterator It(Manifolds); It; ++It)
	{
		if (It.Key().ObjectA == Object || It.Key().ObjectB == Object)
		{
			It.RemoveCurrent();
		}
	}
}
