#pragma once

#include "CoreMinimal.h"
#include "HAL/CriticalSection.h"

#include <atomic>

// Plain data + analysis for the Bullet vs Chaos backend comparison (see BackendComparison.h). No Chrono / UObject
// access in here: recordings are written on the physics thread and analysed on the game thread.

namespace BackendCompare
{
	/// Doubles stored per body and sample: position (3, Chrono frame, m), rotation (w, x, y, z), linear velocity (3, m/s),
	/// angular velocity (3, rad/s, world frame).
	constexpr int32 BodyStride = 13;
	constexpr int32 TerrainProbeStride = 9;

	/// One key of a driver input script. Values are held linearly interpolated between keys, indexed by the time since the
	/// vehicle was built (not by the Chrono system time, which also runs while the vehicle is still being created).
	struct FScriptKey
	{
		float Time = 0.f;
		float Throttle = 0.f;
		float Steering = 0.f;
		float Braking = 0.f;
	};

	/// Time series written by the physics thread, read by the game thread once bDone is set.
	struct FRunRecording
	{
		FCriticalSection Lock;

		FString Label;
		TArray<FString> BodyNames;
		/// Mass of each body (kg), for the kinetic energy; 0 = unknown
		TArray<double> BodyMass;
		TArray<FString> ExtraNames;

		/// Sample times (seconds since the scenario started)
		TArray<double> Times;
		/// [sample][body][BodyStride]
		TArray<double> BodyData;
		/// [sample][extra]
		TArray<double> ExtraData;

		/// Terrain query probe (vehicle runs): per point x, y, hit, height, nx, ny, nz, friction, type - see TerrainProbeStride
		TArray<double> TerrainProbe;

		/// Scenario length; the physics thread sets bDone when a sample at or beyond it was written
		double Duration = 0.0;
		std::atomic<bool> bDone{ false };

		int32 NumBodies() const { return BodyNames.Num(); }
		int32 NumExtras() const { return ExtraNames.Num(); }
		int32 NumSamples() const { return Times.Num(); }
		const double* GetBody(int32 Sample, int32 Body) const { return &BodyData[((int64)Sample * NumBodies() + Body) * BodyStride]; }
		double GetExtra(int32 Sample, int32 Extra) const { return ExtraData[(int64)Sample * NumExtras() + Extra]; }

		/// Physics thread. Body points at NumBodies() * BodyStride doubles, Extra at NumExtras() doubles.
		void AddSample(double Time, const double* Body, const double* Extra)
		{
			FScopeLock ScopeLock(&Lock);
			Times.Add(Time);
			BodyData.Append(Body, (int64)NumBodies() * BodyStride);
			if (NumExtras() > 0)
			{
				ExtraData.Append(Extra, NumExtras());
			}
			if (Time >= Duration)
			{
				bDone = true;
			}
		}
	};

	struct FSeriesStats
	{
		double Rms = 0.0;
		double Max = 0.0;
		double Final = 0.0;
	};

	/// Difference of two recordings sampled on the same time grid
	struct FTrajectoryComparison
	{
		FString Name;
		bool bValid = false;
		FString Error;

		int32 NumSamples = 0;
		double Duration = 0.0;

		/// Over all bodies and samples: position (m), orientation (degrees), linear velocity (m/s)
		FSeriesStats Position, Rotation, Velocity;
		/// Per body: RMS / max / final position error (m)
		TArray<FString> BodyNames;
		TArray<FSeriesStats> BodyPosition;
		/// Per body orientation error (degrees) and linear velocity error (m/s)
		TArray<FSeriesStats> BodyRotation;
		TArray<FSeriesStats> BodyVelocity;
		/// Worst position error over the bodies at each sample (m)
		TArray<double> WorstPositionOverTime;
		TArray<double> Times;
		/// First time the worst position error exceeded 5 cm (-1 = never)
		double TimeToDivergence5cm = -1.0;

		/// Extras (scalar telemetry such as speed / rpm): RMS / max / final of the difference
		TArray<FString> ExtraNames;
		TArray<FSeriesStats> Extra;
		/// Largest absolute value of each extra in A (to express the errors as a fraction)
		TArray<double> ExtraPeakA;

		/// Path length of body 0 in A (m): the natural length scale of a driven scenario
		double PathLengthA = 0.0;
	};

	/// Whole-scene quantities of one recording (rigid body piles): these stay meaningful when individual trajectories
	/// diverge chaotically.
	struct FAggregateSeries
	{
		TArray<double> Times;
		TArray<double> KineticEnergy;   // J, linear + rotational is not available without inertia: linear only
		TArray<double> CenterOfMassZ;   // m
		TArray<double> HeightP90;       // m, 90th percentile of the body heights (robust "pile height")
		TArray<double> NumMoving;       // bodies faster than 5 cm/s
	};

	struct FAggregateComparison
	{
		bool bValid = false;
		/// RMS of the kinetic energy difference as a fraction of the peak energy of A
		double KineticEnergyRmsFraction = 0.0;
		/// Max |COM z difference| (m)
		double CenterOfMassZMax = 0.0;
		/// Difference of the 90th percentile body height at the end (m)
		double FinalHeightP90Diff = 0.0;
		/// Bodies still moving at the end (A, B)
		double FinalMovingA = 0.0;
		double FinalMovingB = 0.0;
		/// Time after which less than 5% of the bodies move (-1 = never), A and B
		double SettleTimeA = -1.0;
		double SettleTimeB = -1.0;
		/// Bodies that left the arena (z below -1 m) at the end, A and B
		int32 EscapedA = 0;
		int32 EscapedB = 0;
	};

	/// Agreement of the terrain queries of two backends at the same fixed points
	struct FTerrainProbeComparison
	{
		bool bValid = false;
		int32 NumPoints = 0;
		int32 HitMismatches = 0;
		int32 TypeMismatches = 0;
		double HeightMax = 0.0;       // m
		double HeightMaxX = 0.0, HeightMaxY = 0.0;
		double NormalMaxDeg = 0.0;
		double FrictionMax = 0.0;
		double FrictionA = 0.0, FrictionB = 0.0;   // first point where they differ most
	};
	FTerrainProbeComparison CompareTerrainProbes(const FRunRecording& A, const FRunRecording& B);

	FTrajectoryComparison CompareTrajectories(const FString& Name, const FRunRecording& A, const FRunRecording& B);
	FAggregateSeries ComputeAggregate(const FRunRecording& Rec);
	FAggregateComparison CompareAggregates(const FAggregateSeries& A, const FAggregateSeries& B, int32 NumBodies, const FRunRecording& RecA, const FRunRecording& RecB);

	/// CSV: one row per sample, one column group per body
	bool WriteRecordingCsv(const FString& Path, const FRunRecording& Rec);
	/// CSV: time, worst position error, plus the per body position error over time
	bool WriteComparisonCsv(const FString& Path, const FRunRecording& A, const FRunRecording& B);
}
