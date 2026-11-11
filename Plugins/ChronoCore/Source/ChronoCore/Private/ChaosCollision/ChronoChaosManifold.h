// Persistent contact manifolds of the Chaos collision backend.
//
// The narrowphase produces fresh contacts every Chrono substep; the manifold cache keeps them per shape pair so that
// Chrono's NSC solver can warm-start from the previous reactions (ChCollisionInfo::reaction_cache). Fresh points are
// matched to the previous ones by their position on shape A; unmatched points start with a zero cache. Pairs that
// produced no contact in a Run() are dropped by Purge().

#pragma once

#include "CoreMinimal.h"

#include "ChaosCollision/ChronoChaosNarrowphase.h"

/// Identifies a shape pair: two proxies (or a proxy and a non-Chrono particle) and a leaf / shape index on each.
struct FChronoManifoldKey
{
	/// Chrono model (ChCollisionModelChaos*) of side A.
	const void* ObjectA = nullptr;
	int32 ShapeA = INDEX_NONE;
	/// Chrono model (ChCollisionModelChaos*) or, when bUnrealB is set, non-Chrono particle (FGeometryParticleHandle*) of side B.
	const void* ObjectB = nullptr;
	int32 ShapeB = INDEX_NONE;
	bool bUnrealB = false;

	/// Creation order of the objects (not their addresses): the manifolds are reported to Chrono sorted by it, so the contact
	/// order - and with it the result of the sequential contact solver - is the same from run to run.
	uint32 SerialA = 0;
	uint32 SerialB = 0;

	/// Deterministic report order.
	static bool ReportOrder(const FChronoManifoldKey& L, const FChronoManifoldKey& R)
	{
		if (L.SerialA != R.SerialA) return L.SerialA < R.SerialA;
		if (L.ShapeA != R.ShapeA) return L.ShapeA < R.ShapeA;
		if (L.bUnrealB != R.bUnrealB) return !L.bUnrealB;
		if (L.SerialB != R.SerialB) return L.SerialB < R.SerialB;
		return L.ShapeB < R.ShapeB;
	}

	friend bool operator==(const FChronoManifoldKey& L, const FChronoManifoldKey& R)
	{
		return L.ObjectA == R.ObjectA && L.ShapeA == R.ShapeA && L.ObjectB == R.ObjectB && L.ShapeB == R.ShapeB && L.bUnrealB == R.bUnrealB;
	}

	friend uint32 GetTypeHash(const FChronoManifoldKey& Key)
	{
		return HashCombine(HashCombine(PointerHash(Key.ObjectA), ::GetTypeHash(Key.ShapeA)),
		                   HashCombine(PointerHash(Key.ObjectB), ::GetTypeHash(Key.ShapeB) ^ (Key.bUnrealB ? 0x9E3779B9u : 0u)));
	}
};

/// One persistent contact point (world space, Unreal cm; normal from A to B).
struct FChronoManifoldPoint
{
	FVector PointA = FVector::ZeroVector;
	FVector PointB = FVector::ZeroVector;
	FVector Normal = FVector::UpVector;
	double Separation = 0.0;
	int32 FaceIndexB = INDEX_NONE;

	/// Point on A in A's (leaf) local space - identity for matching across substeps.
	FVector LocalA = FVector::ZeroVector;

	/// Handed to Chrono as ChCollisionInfo::reaction_cache (NSC stores 3 reactions, SMC may use more).
	float ReactionCache[6] = { 0, 0, 0, 0, 0, 0 };
};

struct FChronoManifold
{
	TArray<FChronoManifoldPoint, TInlineAllocator<4>> Points;

	/// Serial of the Run() that last refreshed this manifold (for purging).
	int32 LastRunSerial = -1;
};

/// All manifolds of a collision system.
class FChronoManifoldCache
{
public:
	/// Replace the manifold of 'Key' with 'Fresh' (world-space contacts), carrying reaction caches over from points
	/// whose A-local positions match within MatchDistanceCm. 'ATM' is the world transform of leaf A. An empty 'Fresh'
	/// removes the manifold.
	void Update(const FChronoManifoldKey& Key, const ChronoChaosNarrowphase::FContactArray& Fresh, const FTransform& ATM, double MatchDistanceCm, int32 RunSerial);

	/// Remove every manifold that was not updated in 'RunSerial'.
	void Purge(int32 RunSerial);

	void Reset() { Manifolds.Reset(); }

	int32 Num() const { return Manifolds.Num(); }

	int32 NumPoints() const;

	/// Remove every manifold involving 'Object' (a proxy or particle that went away).
	void RemoveObject(const void* Object);

	template <typename Visitor>
	void ForEach(const Visitor& Visit) const
	{
		for (const TPair<FChronoManifoldKey, FChronoManifold>& Pair : Manifolds)
		{
			Visit(Pair.Key, Pair.Value);
		}
	}

	/// Visits the manifolds in the deterministic order of FChronoManifoldKey::ReportOrder (TMap order depends on addresses).
	template <typename Visitor>
	void ForEach(const Visitor& Visit)
	{
		Sorted.Reset(Manifolds.Num());
		for (TPair<FChronoManifoldKey, FChronoManifold>& Pair : Manifolds)
		{
			Sorted.Add(&Pair);
		}
		Sorted.Sort([](const TPair<FChronoManifoldKey, FChronoManifold>& L, const TPair<FChronoManifoldKey, FChronoManifold>& R)
		{
			return FChronoManifoldKey::ReportOrder(L.Key, R.Key);
		});
		for (TPair<FChronoManifoldKey, FChronoManifold>* Pair : Sorted)
		{
			Visit(Pair->Key, Pair->Value);
		}
	}

private:
	TMap<FChronoManifoldKey, FChronoManifold> Manifolds;
	TArray<TPair<FChronoManifoldKey, FChronoManifold>*> Sorted;
};
