#include "RigidBodyBenchmark.h"

#include "ChronoTypes.h"
#include "Math/RandomStream.h"

namespace BackendCompare
{
	namespace
	{
		FQuat RandomRotation(FRandomStream& Rng)
		{
			FQuat Q(Rng.FRandRange(-1.f, 1.f), Rng.FRandRange(-1.f, 1.f), Rng.FRandRange(-1.f, 1.f), Rng.FRandRange(-1.f, 1.f));
			if (Q.SizeSquared() < 1e-6)
			{
				return FQuat::Identity;
			}
			Q.Normalize();
			return Q;
		}

		FBenchBodySpec RandomBodySpec(FRandomStream& Rng, int32 Index)
		{
			FBenchBodySpec S;
			S.Shape = (EBenchShape)(Index % 5);
			switch (S.Shape)
			{
			case EBenchShape::Box:      S.Size = FVector(Rng.FRandRange(0.25, 0.45), Rng.FRandRange(0.25, 0.45), Rng.FRandRange(0.25, 0.45)); break;
			case EBenchShape::Sphere:   S.Size = FVector(Rng.FRandRange(0.12, 0.22), 0, 0); break;
			case EBenchShape::Capsule:  S.Size = FVector(Rng.FRandRange(0.08, 0.14), Rng.FRandRange(0.2, 0.4), 0); break;
			case EBenchShape::Cylinder: S.Size = FVector(Rng.FRandRange(0.12, 0.2), Rng.FRandRange(0.2, 0.4), 0); break;
			case EBenchShape::Hull:     S.Size = FVector(Rng.FRandRange(0.18, 0.3), Rng.FRandRange(0.15, 0.25), Rng.FRandRange(0.12, 0.2)); break;
			}
			S.Density = Rng.FRandRange(300.0, 900.0);
			S.Rotation = RandomRotation(Rng);
			return S;
		}
	}

	FBenchScene MakeSparseScene(int32 Seed)
	{
		FBenchScene Scene;
		Scene.Name = TEXT("sparse");
		Scene.Duration = 4.0;
		FRandomStream Rng(Seed);

		// 6 x 6 bodies on a 2 m grid, dropped from 0.6 .. 2.5 m
		const int32 N = 6;
		for (int32 j = 0; j < N; ++j)
		{
			for (int32 i = 0; i < N; ++i)
			{
				FBenchBodySpec S = RandomBodySpec(Rng, i + j * N);
				S.Position = FVector((i - 0.5 * (N - 1)) * 2.0, (j - 0.5 * (N - 1)) * 2.0, Rng.FRandRange(0.6, 2.5));
				Scene.Bodies.Add(S);
			}
		}
		return Scene;
	}

	FBenchScene MakeRollScene(int32 Seed)
	{
		FBenchScene Scene;
		Scene.Name = TEXT("roll");
		Scene.Duration = 5.0;
		(void)Seed;

		// Five cylinders (axis along Y, lying on their curved side) and five spheres of the same radii, side by side, all
		// launched along +X with the angular velocity of rolling without slipping (w = v / r)
		const double Radii[5] = { 0.2, 0.25, 0.3, 0.35, 0.4 };
		const FQuat LieDown(FVector::ForwardVector, UE_DOUBLE_HALF_PI);   // local Z (the cylinder axis) -> world Y
		for (int32 Row = 0; Row < 2; ++Row)
		{
			for (int32 i = 0; i < 5; ++i)
			{
				const double R = Radii[i];
				const double V = 5.0;
				FBenchBodySpec S;
				S.Shape = Row == 0 ? EBenchShape::Cylinder : EBenchShape::Sphere;
				S.Size = Row == 0 ? FVector(R, 0.24, 0) : FVector(R, 0, 0);
				S.Density = 500.0;
				S.Position = FVector(-15.0, (Row * 5 + i - 4.5) * 1.2, R + 0.002);
				S.Rotation = Row == 0 ? LieDown : FQuat::Identity;
				S.LinearVelocity = FVector(V, 0, 0);
				S.AngularVelocity = FVector(0, V / R, 0);
				Scene.Bodies.Add(S);
			}
		}
		return Scene;
	}

	FBenchScene MakePileScene(int32 Seed, int32 NumX, int32 NumY, int32 NumZ)
	{
		FBenchScene Scene;
		Scene.Name = TEXT("pile");
		Scene.Duration = 6.0;
		FRandomStream Rng(Seed);

		// Container 2.6 m x 2.6 m, walls 1.5 m high
		const double Half = 1.3, WallHeight = 1.5, WallThickness = 0.2;
		Scene.FixedBoxes.Add({ FVector(Half + 0.5 * WallThickness, 0, 0.5 * WallHeight), FVector(WallThickness, 2 * Half + 2 * WallThickness, WallHeight) });
		Scene.FixedBoxes.Add({ FVector(-Half - 0.5 * WallThickness, 0, 0.5 * WallHeight), FVector(WallThickness, 2 * Half + 2 * WallThickness, WallHeight) });
		Scene.FixedBoxes.Add({ FVector(0, Half + 0.5 * WallThickness, 0.5 * WallHeight), FVector(2 * Half, WallThickness, WallHeight) });
		Scene.FixedBoxes.Add({ FVector(0, -Half - 0.5 * WallThickness, 0.5 * WallHeight), FVector(2 * Half, WallThickness, WallHeight) });

		const double Spacing = 0.5;
		int32 Index = 0;
		for (int32 k = 0; k < NumZ; ++k)
		{
			for (int32 j = 0; j < NumY; ++j)
			{
				for (int32 i = 0; i < NumX; ++i)
				{
					FBenchBodySpec S = RandomBodySpec(Rng, Index++);
					S.Position = FVector((i - 0.5 * (NumX - 1)) * Spacing + Rng.FRandRange(-0.05, 0.05),
						(j - 0.5 * (NumY - 1)) * Spacing + Rng.FRandRange(-0.05, 0.05), 0.5 + k * Spacing);
					Scene.Bodies.Add(S);
				}
			}
		}
		return Scene;
	}

	namespace
	{
		// Dynamic shape sizes of the collider scenes (Size conventions of FBenchBodySpec)
		FBenchBodySpec ColliderBody(EBenchShape Shape, double Density = 500.0)
		{
			FBenchBodySpec S;
			S.Shape = Shape;
			S.Density = Density;
			switch (Shape)
			{
			case EBenchShape::Box:      S.Size = FVector(0.30, 0.25, 0.20); break;
			case EBenchShape::Sphere:   S.Size = FVector(0.15, 0, 0); break;
			case EBenchShape::Capsule:  S.Size = FVector(0.10, 0.30, 0); break;
			case EBenchShape::Cylinder: S.Size = FVector(0.15, 0.30, 0); break;
			case EBenchShape::Hull:     S.Size = FVector(0.20, 0.15, 0.15); break;
			}
			return S;
		}

		/// Half extent along world Z of the dynamic shape resting on its natural face
		double RestHalfHeight(const FBenchBodySpec& S)
		{
			switch (S.Shape)
			{
			case EBenchShape::Box:      return 0.5 * S.Size.Z;
			case EBenchShape::Sphere:   return S.Size.X;
			case EBenchShape::Capsule:  return 0.5 * S.Size.Y + S.Size.X;
			case EBenchShape::Cylinder: return 0.5 * S.Size.Y;
			default:                    return S.Size.Z;
			}
		}
	}

	FBenchScene MakePairScene()
	{
		FBenchScene Scene;
		Scene.Name = TEXT("col_pairs");
		Scene.Duration = 4.0;
		Scene.Friction = 0.5;
		Scene.Restitution = 0.1;
		Scene.ViewDistance = 17.0;
		Scene.ViewTarget = FVector(6.0, 4.8, 0.0);

		// Columns: the kind of fixed collider (shared by the five bodies dropped on it). Rows: the kind of dynamic shape.
		const double Spacing = 2.4;
		const FQuat Ramp(FVector::YAxisVector, FMath::DegreesToRadians(20.0));
		struct FFixedKind { EBenchShape Shape; FVector Size; double Top; FQuat Rotation; };
		const FFixedKind Kinds[6] = {
			{ EBenchShape::Box,      FVector(1.2, 1.2, 0.4),   0.40, FQuat::Identity },
			{ EBenchShape::Sphere,   FVector(0.45, 0, 0),      0.90, FQuat::Identity },
			{ EBenchShape::Capsule,  FVector(0.25, 0.9, 0),    0.50, FQuat(FVector::XAxisVector, UE_DOUBLE_HALF_PI) },   // lying along Y
			{ EBenchShape::Cylinder, FVector(0.45, 0.6, 0),    0.60, FQuat::Identity },                                  // upright
			{ EBenchShape::Hull,     FVector(0.55, 0.55, 0.3), 0.60, FQuat::Identity },
			{ EBenchShape::Box,      FVector(1.6, 1.2, 0.1),   0.35, Ramp },
		};
		const EBenchShape Dynamics[5] = { EBenchShape::Box, EBenchShape::Sphere, EBenchShape::Capsule, EBenchShape::Cylinder, EBenchShape::Hull };

		for (int32 Row = 0; Row < 5; ++Row)
		{
			for (int32 Col = 0; Col < 6; ++Col)
			{
				const FFixedKind& K = Kinds[Col];
				const FVector Cell(Col * Spacing, Row * Spacing, 0.0);

				if (Row < 5)
				{
					FBenchFixedShape F;
					F.Shape = K.Shape;
					F.Size = K.Size;
					F.Rotation = K.Rotation;
					double CenterZ = K.Top - 0.5 * K.Size.Z;
					if (K.Shape == EBenchShape::Sphere || K.Shape == EBenchShape::Capsule) { CenterZ = K.Top - K.Size.X; }
					else if (K.Shape == EBenchShape::Cylinder) { CenterZ = K.Top - 0.5 * K.Size.Y; }
					else if (K.Shape == EBenchShape::Hull) { CenterZ = K.Top - K.Size.Z; }
					F.Position = Cell + FVector(0, 0, CenterZ);
					Scene.FixedShapes.Add(F);   // one collider per cell: bodies of different rows are 2.4 m apart in Y
				}

				FBenchBodySpec S = ColliderBody(Dynamics[Row]);
				S.Position = Cell + FVector(0.07, 0.04, K.Top + RestHalfHeight(S) + 0.55);
				// A fixed small tilt so no body lands exactly flat (a flat landing is knife-edge for the contact normal)
				S.Rotation = FQuat(FVector(1, 0.5, 0).GetSafeNormal(), FMath::DegreesToRadians(12.0));
				Scene.Bodies.Add(S);
			}
		}
		return Scene;
	}

	FBenchScene MakeStackScene()
	{
		FBenchScene Scene;
		Scene.Name = TEXT("col_stack");
		Scene.Duration = 5.0;
		Scene.Friction = 0.5;
		Scene.Restitution = 0.0;
		Scene.ViewDistance = 11.0;
		Scene.ViewTarget = FVector(0, 0, 1.0);

		// Stack 0: six boxes. Stack 1: six upright cylinders. Stack 2: alternating box / cylinder.
		const double Gap = 0.002;
		for (int32 Stack = 0; Stack < 3; ++Stack)
		{
			double Z = Gap;
			for (int32 i = 0; i < 6; ++i)
			{
				const bool bCylinder = Stack == 1 || (Stack == 2 && (i % 2 == 1));
				FBenchBodySpec S = ColliderBody(bCylinder ? EBenchShape::Cylinder : EBenchShape::Box);
				S.Size = bCylinder ? FVector(0.20, 0.25, 0) : FVector(0.40, 0.40, 0.25);
				const double HalfHeight = bCylinder ? 0.5 * S.Size.Y : 0.5 * S.Size.Z;
				// small alternating lateral offsets so the stack is not a perfect column
				S.Position = FVector((Stack - 1) * 2.0 + ((i % 2) ? 0.012 : -0.012), (i % 3 - 1) * 0.006, Z + HalfHeight);
				Z += 2.0 * HalfHeight + Gap;
				Scene.Bodies.Add(S);
			}
		}
		return Scene;
	}

	FBenchScene MakeSlideScene()
	{
		FBenchScene Scene;
		Scene.Name = TEXT("col_slide");
		Scene.Duration = 4.0;
		Scene.Friction = 0.3;
		Scene.Restitution = 0.0;
		Scene.ViewDistance = 10.0;
		Scene.ViewTarget = FVector(0.5, 0.0, 2.5);

		const double Angle = FMath::DegreesToRadians(22.0);
		const FQuat RampRot(FVector::YAxisVector, Angle);          // surface descends towards +X
		const double Thick = 0.2;
		FBenchFixedShape Ramp;
		Ramp.Shape = EBenchShape::Box;
		Ramp.Size = FVector(8.0, 6.0, Thick);
		Ramp.Position = FVector(0, 0, 3.0);
		Ramp.Rotation = RampRot;
		Scene.FixedShapes.Add(Ramp);

		// Five lanes: box, sphere, cylinder (axis across the slope: rolls), capsule (axis across: rolls), hull
		const FQuat LieDown(FVector::XAxisVector, UE_DOUBLE_HALF_PI);
		const EBenchShape Shapes[5] = { EBenchShape::Box, EBenchShape::Sphere, EBenchShape::Cylinder, EBenchShape::Capsule, EBenchShape::Hull };
		for (int32 Lane = 0; Lane < 5; ++Lane)
		{
			FBenchBodySpec S = ColliderBody(Shapes[Lane]);
			double Radius = RestHalfHeight(S);
			FQuat Local = FQuat::Identity;
			if (Shapes[Lane] == EBenchShape::Cylinder || Shapes[Lane] == EBenchShape::Capsule) { Radius = S.Size.X; Local = LieDown; }
			const FVector InRamp(-2.6, (Lane - 2) * 0.9, 0.5 * Thick + Radius + 0.003);
			S.Position = Ramp.Position + RampRot.RotateVector(InRamp);
			S.Rotation = RampRot * Local;
			Scene.Bodies.Add(S);
		}
		return Scene;
	}

	FBenchScene MakeBounceScene()
	{
		FBenchScene Scene;
		Scene.Name = TEXT("col_bounce");
		Scene.Duration = 4.0;
		Scene.Friction = 0.5;
		Scene.Restitution = 0.6;
		Scene.ViewDistance = 11.0;
		Scene.ViewTarget = FVector(0, 0, 1.0);

		// Dropped from 1.5 m (lowest point), exactly flat on their natural face
		const FQuat LieDown(FVector::XAxisVector, UE_DOUBLE_HALF_PI);
		struct FDrop { EBenchShape Shape; FQuat Rotation; double HalfHeight; };
		const FBenchBodySpec Box = ColliderBody(EBenchShape::Box), Sphere = ColliderBody(EBenchShape::Sphere), Capsule = ColliderBody(EBenchShape::Capsule),
			Cylinder = ColliderBody(EBenchShape::Cylinder), Hull = ColliderBody(EBenchShape::Hull);
		const FDrop Drops[7] = {
			{ EBenchShape::Box,      FQuat::Identity, 0.5 * Box.Size.Z },
			{ EBenchShape::Sphere,   FQuat::Identity, Sphere.Size.X },
			{ EBenchShape::Capsule,  FQuat::Identity, 0.5 * Capsule.Size.Y + Capsule.Size.X },          // standing
			{ EBenchShape::Capsule,  LieDown,         Capsule.Size.X },                                  // lying
			{ EBenchShape::Cylinder, FQuat::Identity, 0.5 * Cylinder.Size.Y },                           // flat cap down
			{ EBenchShape::Cylinder, LieDown,         Cylinder.Size.X },                                 // on its side
			{ EBenchShape::Hull,     FQuat::Identity, Hull.Size.Z },                                     // tip down
		};
		for (int32 i = 0; i < 7; ++i)
		{
			FBenchBodySpec S = ColliderBody(Drops[i].Shape);
			S.Position = FVector((i - 3) * 1.0, 0, Drops[i].HalfHeight + 1.5);
			S.Rotation = Drops[i].Rotation;
			Scene.Bodies.Add(S);
		}
		return Scene;
	}

	namespace
	{
		std::shared_ptr<chrono::ChBody> MakeShapeBody(const FBenchBodySpec& S, const std::shared_ptr<chrono::ChContactMaterial>& Material)
		{
			using namespace chrono;
			std::shared_ptr<ChBody> Body;
			switch (S.Shape)
			{
			case EBenchShape::Box:
				Body = chrono_types::make_shared<ChBodyEasyBox>(S.Size.X, S.Size.Y, S.Size.Z, S.Density, false, true, Material);
				break;
			case EBenchShape::Sphere:
				Body = chrono_types::make_shared<ChBodyEasySphere>(S.Size.X, S.Density, false, true, Material);
				break;
			case EBenchShape::Cylinder:
				Body = chrono_types::make_shared<ChBodyEasyCylinder>(ChAxis::Z, S.Size.X, S.Size.Y, S.Density, false, true, Material);
				break;
			case EBenchShape::Hull:
			{
				std::vector<ChVector3d> Points = {
					ChVector3d(S.Size.X, 0, 0), ChVector3d(-S.Size.X, 0, 0), ChVector3d(0, S.Size.Y, 0),
					ChVector3d(0, -S.Size.Y, 0), ChVector3d(0, 0, S.Size.Z), ChVector3d(0, 0, -S.Size.Z) };
				Body = chrono_types::make_shared<ChBodyEasyConvexHull>(Points, S.Density, false, true, Material);
				break;
			}
			case EBenchShape::Capsule:
			default:
			{
				// Capsule along Z: no "easy" body, so set the shape and the inertia (cylinder + two hemispheres) by hand
				const double R = S.Size.X, L = S.Size.Y;
				const double Mc = S.Density * UE_DOUBLE_PI * R * R * L;
				const double Ms = S.Density * 4.0 / 3.0 * UE_DOUBLE_PI * R * R * R;
				const double Izz = Mc * R * R / 2.0 + Ms * 2.0 / 5.0 * R * R;
				const double Ixx = Mc * (3.0 * R * R + L * L) / 12.0 + Ms * (2.0 / 5.0 * R * R + L * L / 4.0 + 3.0 / 8.0 * R * L);
				Body = chrono_types::make_shared<ChBody>();
				Body->SetMass(Mc + Ms);
				Body->SetInertiaXX(ChVector3d(Ixx, Ixx, Izz));
				Body->AddCollisionShape(chrono_types::make_shared<ChCollisionShapeCapsule>(Material, R, L));
				Body->EnableCollision(true);
				break;
			}
			}
			return Body;
		}
	}

	void BuildBenchScene(chrono::ChSystem& System, const FBenchScene& Scene, const FVector& Origin, FRunRecording& Recording, TArray<std::shared_ptr<chrono::ChBody>>& OutBodies)
	{
		using namespace chrono;

		auto Material = chrono_types::make_shared<ChContactMaterialNSC>();
		Material->SetFriction((float)Scene.Friction);
		Material->SetRestitution((float)Scene.Restitution);

		const ChVector3d O(Origin.X, Origin.Y, Origin.Z);

		const auto AddFixed = [&](const FVector& Center, const FVector& Size)
		{
			auto Box = chrono_types::make_shared<ChBodyEasyBox>(Size.X, Size.Y, Size.Z, 1000.0, false, true, Material);
			Box->SetPos(O + ChVector3d(Center.X, Center.Y, Center.Z));
			Box->SetFixed(true);
			System.AddBody(Box);
		};
		AddFixed(FVector(0, 0, -0.5), FVector(60, 60, 1));
		for (const FBenchFixedBox& Wall : Scene.FixedBoxes)
		{
			AddFixed(Wall.Center, Wall.Size);
		}
		for (const FBenchFixedShape& F : Scene.FixedShapes)
		{
			FBenchBodySpec Spec;
			Spec.Shape = F.Shape;
			Spec.Size = F.Size;
			std::shared_ptr<ChBody> Fixed = MakeShapeBody(Spec, Material);
			Fixed->SetPos(O + ChVector3d(F.Position.X, F.Position.Y, F.Position.Z));
			Fixed->SetRot(ChQuaterniond(F.Rotation.W, F.Rotation.X, F.Rotation.Y, F.Rotation.Z));
			Fixed->SetFixed(true);
			System.AddBody(Fixed);
		}

		FScopeLock ScopeLock(&Recording.Lock);
		Recording.BodyNames.Reset();
		Recording.BodyMass.Reset();
		Recording.ExtraNames.Reset();

		int32 Index = 0;
		for (const FBenchBodySpec& S : Scene.Bodies)
		{
			static const TCHAR* ShapeNames[] = { TEXT("box"), TEXT("sphere"), TEXT("capsule"), TEXT("cylinder"), TEXT("hull") };
			std::shared_ptr<ChBody> Body = MakeShapeBody(S, Material);
			Body->SetPos(O + ChVector3d(S.Position.X, S.Position.Y, S.Position.Z));
			Body->SetRot(ChQuaterniond(S.Rotation.W, S.Rotation.X, S.Rotation.Y, S.Rotation.Z));
			Body->SetPosDt(ChVector3d(S.LinearVelocity.X, S.LinearVelocity.Y, S.LinearVelocity.Z));
			Body->SetAngVelParent(ChVector3d(S.AngularVelocity.X, S.AngularVelocity.Y, S.AngularVelocity.Z));
			Body->SetSleepingAllowed(false);
			System.AddBody(Body);

			OutBodies.Add(Body);
			Recording.BodyNames.Add(FString::Printf(TEXT("%s_%03d"), ShapeNames[(int32)S.Shape], Index++));
			Recording.BodyMass.Add(Body->GetMass());
		}
	}

	FRigidBenchStepper::FRigidBenchStepper(TSharedPtr<FRunRecording> InRecording, TArray<std::shared_ptr<chrono::ChBody>> InBodies, int32 InSampleEvery)
		: Recording(MoveTemp(InRecording)), Bodies(MoveTemp(InBodies)), SampleEvery(FMath::Max(InSampleEvery, 1))
	{
	}

	void FRigidBenchStepper::SubStep(FChronoStepContext& Context)
	{
		Context.System.DoStepDynamics(Context.Dt);
		++StepCount;
		if (!Recording || (StepCount % SampleEvery) != 0)
		{
			return;
		}

		TArray<double> Data;
		Data.SetNumUninitialized(Bodies.Num() * BodyStride);
		for (int32 b = 0; b < Bodies.Num(); ++b)
		{
			double* D = &Data[b * BodyStride];
			const chrono::ChBody& Body = *Bodies[b];
			const chrono::ChVector3d P = Body.GetPos();
			const chrono::ChQuaterniond Q = Body.GetRot();
			const chrono::ChVector3d V = Body.GetPosDt();
			const chrono::ChVector3d W = Body.GetAngVelParent();
			D[0] = P.x(); D[1] = P.y(); D[2] = P.z();
			D[3] = Q.e0(); D[4] = Q.e1(); D[5] = Q.e2(); D[6] = Q.e3();
			D[7] = V.x(); D[8] = V.y(); D[9] = V.z();
			D[10] = W.x(); D[11] = W.y(); D[12] = W.z();
		}
		Recording->AddSample((double)StepCount * Context.Dt, Data.GetData(), nullptr);
	}
}
