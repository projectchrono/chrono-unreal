#pragma once

#include "CoreMinimal.h"
#include "BackendComparisonTypes.h"
#include "ChronoSimulation.h"

// Chrono-only rigid body scenes for the backend comparison: a fixed ground, optional fixed walls and a set of falling
// bodies. The scene is described as plain data (generated on the game thread from a fixed seed) and built on the physics
// thread, so Bullet and Chaos run the identical Chrono system and only the collision detection differs.

namespace BackendCompare
{
	enum class EBenchShape : uint8 { Box, Sphere, Capsule, Cylinder, Hull };

	struct FBenchBodySpec
	{
		EBenchShape Shape = EBenchShape::Box;
		/// Box: full sizes x y z. Sphere: radius. Capsule: radius, length of the cylindrical part. Cylinder: radius, height
		/// (axis = local Z). Hull: scale of the unit octahedron (x y z half extents).
		FVector Size = FVector(0.3, 0.3, 0.3);
		double Density = 500.0;
		FVector Position = FVector::ZeroVector;      // m, relative to the scene origin
		FQuat Rotation = FQuat::Identity;
		FVector LinearVelocity = FVector::ZeroVector;
		FVector AngularVelocity = FVector::ZeroVector;
	};

	struct FBenchFixedBox
	{
		FVector Center = FVector::ZeroVector;        // m, relative to the scene origin
		FVector Size = FVector::OneVector;           // full sizes
	};

	/// A fixed (immovable) collider of any shape, same size conventions as FBenchBodySpec.
	struct FBenchFixedShape
	{
		EBenchShape Shape = EBenchShape::Box;
		FVector Size = FVector::OneVector;
		FVector Position = FVector::ZeroVector;      // m, relative to the scene origin
		FQuat Rotation = FQuat::Identity;
	};

	struct FBenchScene
	{
		FString Name;
		/// Fixed colliders besides the ground slab (collider test scenes).
		TArray<FBenchFixedShape> FixedShapes;
		/// Camera framing for the on screen view: distance (m) and the point looked at (m, relative to the scene origin)
		double ViewDistance = 11.0;
		FVector ViewTarget = FVector(0, 0, 0.6);
		double Duration = 4.0;
		TArray<FBenchBodySpec> Bodies;
		TArray<FBenchFixedBox> FixedBoxes;
		double Friction = 0.5;
		double Restitution = 0.1;
	};

	/// Bodies dropped far apart: almost every body only meets the ground, so trajectories stay comparable.
	FBenchScene MakeSparseScene(int32 Seed);
	/// Cylinders (wheel sized) and spheres launched rolling without slip on the ground: isolates the rolling contact.
	FBenchScene MakeRollScene(int32 Seed);
	/// A dense stack of bodies falling into a walled container: chaotic, compared through scene level statistics.
	FBenchScene MakePileScene(int32 Seed, int32 NumX = 5, int32 NumY = 5, int32 NumZ = 8);

	// Collider scenes: one dynamic shape meeting one kind of collider at a time (or each other), compared body by body.
	/// Every dynamic shape dropped on every kind of fixed collider (box, sphere, capsule, cylinder, hull, ramp).
	FBenchScene MakePairScene();
	/// Stacks of boxes, upright cylinders and alternating boxes / cylinders: resting contact stability and rest height.
	FBenchScene MakeStackScene();
	/// Shapes released at rest on a 22 degree ramp with low friction: sliding and rolling distance.
	FBenchScene MakeSlideScene();
	/// Shapes dropped flat on the ground with a restitution of 0.6: bounce heights.
	FBenchScene MakeBounceScene();

	/// Builds the scene in the system (physics thread) and fills the recording layout. Origin = Chrono position of the scene origin.
	void BuildBenchScene(chrono::ChSystem& System, const FBenchScene& Scene, const FVector& Origin, FRunRecording& Recording, TArray<std::shared_ptr<chrono::ChBody>>& OutBodies);

	/// Stepper that records the dynamic bodies every substep (physics thread).
	class FRigidBenchStepper : public IChronoStepper
	{
	public:
		FRigidBenchStepper(TSharedPtr<FRunRecording> InRecording, TArray<std::shared_ptr<chrono::ChBody>> InBodies, int32 InSampleEvery);
		virtual void SubStep(FChronoStepContext& Context) override;

	private:
		TSharedPtr<FRunRecording> Recording;
		TArray<std::shared_ptr<chrono::ChBody>> Bodies;
		int32 SampleEvery = 1;
		int64 StepCount = 0;
	};
}
