#include "BackendComparisonTypes.h"

#include "Misc/FileHelper.h"
#include "Misc/Paths.h"

namespace BackendCompare
{
	namespace
	{
		inline double Dist(const double* A, const double* B)
		{
			const double X = A[0] - B[0], Y = A[1] - B[1], Z = A[2] - B[2];
			return FMath::Sqrt(X * X + Y * Y + Z * Z);
		}

		inline double AngleDeg(const double* QA, const double* QB)
		{
			const double Dot = FMath::Abs(QA[0] * QB[0] + QA[1] * QB[1] + QA[2] * QB[2] + QA[3] * QB[3]);
			return FMath::RadiansToDegrees(2.0 * FMath::Acos(FMath::Clamp(Dot, 0.0, 1.0)));
		}

		struct FAccumulator
		{
			double SumSq = 0.0;
			double Max = 0.0;
			double Last = 0.0;
			int64 Count = 0;

			void Add(double V)
			{
				SumSq += V * V;
				Max = FMath::Max(Max, FMath::Abs(V));
				Last = V;
				++Count;
			}
			FSeriesStats Get() const
			{
				FSeriesStats S;
				S.Rms = Count > 0 ? FMath::Sqrt(SumSq / (double)Count) : 0.0;
				S.Max = Max;
				S.Final = Last;
				return S;
			}
		};
	}

	FTerrainProbeComparison CompareTerrainProbes(const FRunRecording& A, const FRunRecording& B)
	{
		FTerrainProbeComparison C;
		if (A.TerrainProbe.Num() == 0 || A.TerrainProbe.Num() != B.TerrainProbe.Num())
		{
			return C;
		}
		C.NumPoints = A.TerrainProbe.Num() / TerrainProbeStride;
		for (int32 i = 0; i < C.NumPoints; ++i)
		{
			const double* PA = &A.TerrainProbe[i * TerrainProbeStride];
			const double* PB = &B.TerrainProbe[i * TerrainProbeStride];
			if ((PA[2] > 0.5) != (PB[2] > 0.5))
			{
				++C.HitMismatches;
				continue;
			}
			if (PA[2] < 0.5)
			{
				continue;
			}
			const double DH = FMath::Abs(PA[3] - PB[3]);
			if (DH > C.HeightMax)
			{
				C.HeightMax = DH;
				C.HeightMaxX = PA[0];
				C.HeightMaxY = PA[1];
			}
			const double Dot = FMath::Clamp(PA[4] * PB[4] + PA[5] * PB[5] + PA[6] * PB[6], -1.0, 1.0);
			C.NormalMaxDeg = FMath::Max(C.NormalMaxDeg, FMath::RadiansToDegrees(FMath::Acos(Dot)));
			const double DF = FMath::Abs(PA[7] - PB[7]);
			if (DF > C.FrictionMax)
			{
				C.FrictionMax = DF;
				C.FrictionA = PA[7];
				C.FrictionB = PB[7];
			}
			if (FMath::RoundToInt(PA[8]) != FMath::RoundToInt(PB[8]))
			{
				++C.TypeMismatches;
			}
		}
		C.bValid = true;
		return C;
	}

	FTrajectoryComparison CompareTrajectories(const FString& Name, const FRunRecording& A, const FRunRecording& B)
	{
		FTrajectoryComparison R;
		R.Name = Name;

		if (A.NumBodies() != B.NumBodies() || A.NumExtras() != B.NumExtras())
		{
			R.Error = TEXT("recordings have a different layout");
			return R;
		}
		const int32 N = FMath::Min(A.NumSamples(), B.NumSamples());
		if (N < 2)
		{
			R.Error = FString::Printf(TEXT("too few samples (%d vs %d)"), A.NumSamples(), B.NumSamples());
			return R;
		}
		// Both runs sample on the same grid of simulation time; refuse to compare if they do not
		for (int32 i = 0; i < N; ++i)
		{
			if (FMath::Abs(A.Times[i] - B.Times[i]) > 1e-6)
			{
				R.Error = FString::Printf(TEXT("sample %d is at t=%.6f in A and t=%.6f in B"), i, A.Times[i], B.Times[i]);
				return R;
			}
		}

		const int32 NB = A.NumBodies();
		const int32 NE = A.NumExtras();
		R.NumSamples = N;
		R.Duration = A.Times[N - 1];
		R.BodyNames = A.BodyNames;
		R.ExtraNames = A.ExtraNames;

		FAccumulator Pos, Rot, Vel;
		TArray<FAccumulator> BodyPos;
		BodyPos.SetNum(NB);
		TArray<FAccumulator> BodyRot, BodyVel;
		BodyRot.SetNum(NB);
		BodyVel.SetNum(NB);
		TArray<FAccumulator> Extra;
		Extra.SetNum(NE);
		R.ExtraPeakA.SetNumZeroed(NE);
		R.WorstPositionOverTime.Reserve(N);
		R.Times.Reserve(N);

		for (int32 s = 0; s < N; ++s)
		{
			double Worst = 0.0;
			for (int32 b = 0; b < NB; ++b)
			{
				const double* PA = A.GetBody(s, b);
				const double* PB = B.GetBody(s, b);
				const double DP = Dist(PA, PB);
				Pos.Add(DP);
				BodyPos[b].Add(DP);
				Worst = FMath::Max(Worst, DP);
				const double DR = AngleDeg(PA + 3, PB + 3);
				Rot.Add(DR);
				BodyRot[b].Add(DR);
				const double DV = Dist(PA + 7, PB + 7);
				Vel.Add(DV);
				BodyVel[b].Add(DV);
			}
			R.WorstPositionOverTime.Add(Worst);
			R.Times.Add(A.Times[s]);
			if (R.TimeToDivergence5cm < 0.0 && Worst > 0.05)
			{
				R.TimeToDivergence5cm = A.Times[s];
			}
			for (int32 e = 0; e < NE; ++e)
			{
				const double VA = A.GetExtra(s, e);
				Extra[e].Add(VA - B.GetExtra(s, e));
				R.ExtraPeakA[e] = FMath::Max(R.ExtraPeakA[e], FMath::Abs(VA));
			}
			if (s > 0)
			{
				R.PathLengthA += Dist(A.GetBody(s, 0), A.GetBody(s - 1, 0));
			}
		}

		R.Position = Pos.Get();
		R.Rotation = Rot.Get();
		R.Velocity = Vel.Get();
		for (int32 b = 0; b < NB; ++b)
		{
			R.BodyPosition.Add(BodyPos[b].Get());
			R.BodyRotation.Add(BodyRot[b].Get());
			R.BodyVelocity.Add(BodyVel[b].Get());
		}
		for (int32 e = 0; e < NE; ++e)
		{
			R.Extra.Add(Extra[e].Get());
		}
		R.bValid = true;
		return R;
	}

	FAggregateSeries ComputeAggregate(const FRunRecording& Rec)
	{
		FAggregateSeries S;
		const int32 N = Rec.NumSamples();
		const int32 NB = Rec.NumBodies();
		for (int32 s = 0; s < N; ++s)
		{
			double KE = 0.0, MassSum = 0.0, ComZ = 0.0, Moving = 0.0;
			TArray<double> Heights;
			for (int32 b = 0; b < NB; ++b)
			{
				const double* P = Rec.GetBody(s, b);
				const double Mass = Rec.BodyMass.IsValidIndex(b) ? Rec.BodyMass[b] : 0.0;
				const double V2 = P[7] * P[7] + P[8] * P[8] + P[9] * P[9];
				if (Mass > 0.0)
				{
					KE += 0.5 * Mass * V2;
					MassSum += Mass;
					ComZ += Mass * P[2];
					Heights.Add(P[2]);
					if (V2 > 0.05 * 0.05)
					{
						Moving += 1.0;
					}
				}
			}
			S.Times.Add(Rec.Times[s]);
			S.KineticEnergy.Add(KE);
			S.CenterOfMassZ.Add(MassSum > 0.0 ? ComZ / MassSum : 0.0);
			Heights.Sort();
			S.HeightP90.Add(Heights.Num() > 0 ? Heights[FMath::Min(Heights.Num() - 1, (int32)(0.9 * Heights.Num()))] : 0.0);
			S.NumMoving.Add(Moving);
		}
		return S;
	}

	FAggregateComparison CompareAggregates(const FAggregateSeries& A, const FAggregateSeries& B, int32 NumBodies, const FRunRecording& RecA, const FRunRecording& RecB)
	{
		FAggregateComparison C;
		const int32 N = FMath::Min(A.Times.Num(), B.Times.Num());
		if (N < 2)
		{
			return C;
		}

		double PeakKE = 1e-12, SumSq = 0.0;
		for (int32 s = 0; s < N; ++s)
		{
			PeakKE = FMath::Max(PeakKE, A.KineticEnergy[s]);
			const double D = A.KineticEnergy[s] - B.KineticEnergy[s];
			SumSq += D * D;
			C.CenterOfMassZMax = FMath::Max(C.CenterOfMassZMax, FMath::Abs(A.CenterOfMassZ[s] - B.CenterOfMassZ[s]));
		}
		C.KineticEnergyRmsFraction = FMath::Sqrt(SumSq / N) / PeakKE;
		C.FinalHeightP90Diff = A.HeightP90[N - 1] - B.HeightP90[N - 1];
		C.FinalMovingA = A.NumMoving[N - 1];
		C.FinalMovingB = B.NumMoving[N - 1];

		const auto SettleTime = [&](const FAggregateSeries& S) -> double
		{
			// The last time at least 5% of the bodies were moving; settled from the following sample on
			double Last = -1.0;
			for (int32 s = 0; s < N; ++s)
			{
				if (S.NumMoving[s] >= 0.05 * NumBodies)
				{
					Last = S.Times[s];
				}
			}
			return Last < 0.0 ? 0.0 : (Last >= S.Times[N - 1] ? -1.0 : Last);
		};
		C.SettleTimeA = SettleTime(A);
		C.SettleTimeB = SettleTime(B);

		const auto Escaped = [&](const FRunRecording& Rec) -> int32
		{
			int32 Count = 0;
			const int32 Last = Rec.NumSamples() - 1;
			for (int32 b = 0; b < Rec.NumBodies(); ++b)
			{
				if (Rec.BodyMass.IsValidIndex(b) && Rec.BodyMass[b] > 0.0 && Rec.GetBody(Last, b)[2] < -1.0)
				{
					++Count;
				}
			}
			return Count;
		};
		C.EscapedA = Escaped(RecA);
		C.EscapedB = Escaped(RecB);
		C.bValid = true;
		return C;
	}

	bool WriteRecordingCsv(const FString& Path, const FRunRecording& Rec)
	{
		FString Out;
		Out += TEXT("time");
		for (const FString& Name : Rec.BodyNames)
		{
			for (const TCHAR* Field : { TEXT("px"), TEXT("py"), TEXT("pz"), TEXT("qw"), TEXT("qx"), TEXT("qy"), TEXT("qz"), TEXT("vx"), TEXT("vy"), TEXT("vz"), TEXT("wx"), TEXT("wy"), TEXT("wz") })
			{
				Out += FString::Printf(TEXT(",%s.%s"), *Name, Field);
			}
		}
		for (const FString& Name : Rec.ExtraNames)
		{
			Out += TEXT(",") + Name;
		}
		Out += TEXT("\n");
		for (int32 s = 0; s < Rec.NumSamples(); ++s)
		{
			Out += FString::Printf(TEXT("%.6f"), Rec.Times[s]);
			for (int32 b = 0; b < Rec.NumBodies(); ++b)
			{
				const double* P = Rec.GetBody(s, b);
				for (int32 k = 0; k < BodyStride; ++k)
				{
					Out += FString::Printf(TEXT(",%.9g"), P[k]);
				}
			}
			for (int32 e = 0; e < Rec.NumExtras(); ++e)
			{
				Out += FString::Printf(TEXT(",%.9g"), Rec.GetExtra(s, e));
			}
			Out += TEXT("\n");
		}
		return FFileHelper::SaveStringToFile(Out, *Path);
	}

	bool WriteComparisonCsv(const FString& Path, const FRunRecording& A, const FRunRecording& B)
	{
		const int32 N = FMath::Min(A.NumSamples(), B.NumSamples());
		FString Out = TEXT("time,worst_position_error_m");
		for (const FString& Name : A.BodyNames)
		{
			Out += FString::Printf(TEXT(",%s.position_error_m,%s.rotation_error_deg"), *Name, *Name);
		}
		Out += TEXT("\n");
		for (int32 s = 0; s < N; ++s)
		{
			double Worst = 0.0;
			FString Row;
			for (int32 b = 0; b < A.NumBodies(); ++b)
			{
				const double D = Dist(A.GetBody(s, b), B.GetBody(s, b));
				Worst = FMath::Max(Worst, D);
				Row += FString::Printf(TEXT(",%.9g,%.9g"), D, AngleDeg(A.GetBody(s, b) + 3, B.GetBody(s, b) + 3));
			}
			Out += FString::Printf(TEXT("%.6f,%.9g%s\n"), A.Times[s], Worst, *Row);
		}
		return FFileHelper::SaveStringToFile(Out, *Path);
	}
}
