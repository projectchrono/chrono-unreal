#include "ChDiscreteTire.h"

#include "CoreMinimal.h"

#include "TerrainTypes.h"

#include <algorithm>
#include <cmath>

using namespace chrono;
using namespace chrono::vehicle;
using DiscreteTire::ESurfaceClass;

namespace
{
	//Terrain surface type of the Unreal physical material -> contact law of the tire. Hard ground keeps the friction
	//of the terrain query. Only the loose ground classes get the terramechanics law: dirt, grass and gravel stay
	//firm ground until they are calibrated on purpose (SetSurface()).
	ESurfaceClass DiscreteTireSurfaceClassOf(ETerrainType Type)
	{
		switch (Type)
		{
		case ETerrainType::Sand: return ESurfaceClass::Sand;
		case ETerrainType::Snow: return ESurfaceClass::Snow;
		case ETerrainType::Mud: return ESurfaceClass::Mud;
		default: return ESurfaceClass::Hard;
		}
	}

	DiscreteTire::Vec3 DiscreteTireToLocal(const ChVector3d& v, const ChVector3d& x, const ChVector3d& y, const ChVector3d& z)
	{
		return DiscreteTire::Vec3{ Vdot(v, x), Vdot(v, y), Vdot(v, z) };
	}
}

ChDiscreteTire::ChDiscreteTire(const std::string& name) : ChForceElementTire(name)
{
	m_tireforce.force = ChVector3d(0, 0, 0);
	m_tireforce.point = ChVector3d(0, 0, 0);
	m_tireforce.moment = ChVector3d(0, 0, 0);
	m_rollingRadius = m_params.radius;
}

void ChDiscreteTire::GuessTireData(double tireLoad, double tireWidth, double ratio, double rimDia, double pinflLi, double pinflUse, double dampingRatio)
{
	const double secth = tireWidth * ratio;  //section height
	const double deflMax = 0.16 * secth;     //deflection at the rated load

	m_params.width = tireWidth;
	m_params.radius = secth + 0.5 * rimDia;
	m_params.crownDrop = 0.04 * secth;
	m_params.dlim = 0.55 * secth;  //the rim / bump stop takes over

	m_nominalLoad = 0.5 * tireLoad * std::pow(pinflUse / pinflLi, 0.8);
	m_cz = tireLoad / deflMax;
	m_dz = 2.0 * dampingRatio * std::sqrt(m_cz * GetTireMass());

	//Normalized slip stiffnesses of the TMeasy passenger car data set (dfx0 / pn, dfy0 / pn)
	m_params.Calibrate(m_cz, m_dz, m_nominalLoad, 18.3741, 15.9826);
	m_params.rollingResistance = 0.015;
	m_params.forceLimit = 4.0 * tireLoad;  //4 x the rated load of the tire, far above anything a tire that is not bottomed on its rim produces

	m_rollingRadius = m_params.radius;
}

void ChDiscreteTire::BuildTreadTables()
{
	const int N = m_params.numTread;
	m_cosA.resize(size_t(N));
	m_sinA.resize(size_t(N));
	m_ux.assign(size_t(N), 0.0);
	m_uz.assign(size_t(N), 0.0);
	for (int i = 0; i < N; ++i)
	{
		const double a = 2.0 * DiscreteTire::Pi * i / N;
		m_cosA[size_t(i)] = std::cos(a);
		m_sinA[size_t(i)] = std::sin(a);
	}
}

void ChDiscreteTire::Initialize(std::shared_ptr<ChWheel> wheel)
{
	ChTire::Initialize(wheel);

	SetDiscreteTireParams();

	m_model.Configure(m_params);
	m_params = m_model.GetParams();  //sanitized (odd station count, column count <= strips ...)

	//Ground sample layout: columns across the tread (centres of the outermost strips and in between), stations along
	//the rolling direction covering the tread radius plus a margin, so a face in front of the tire is seen
	m_ground.Resize(m_params.numColumns, m_params.numStations);
	const double reach = m_params.reachScale * m_params.radius;
	for (int m = 0; m < m_params.numStations; ++m)
		m_ground.stationX[size_t(m)] = -reach + 2.0 * reach * m / (m_params.numStations - 1);
	const double yo = 0.5 * (m_params.numStrips - 1) * m_params.width / m_params.numStrips;
	for (int c = 0; c < m_params.numColumns; ++c)
		m_ground.colY[size_t(c)] = (m_params.numColumns == 1) ? 0.0 : -yo + 2.0 * yo * c / (m_params.numColumns - 1);

	BuildTreadTables();
	m_sampled.assign(size_t(m_params.numStations) * m_params.numColumns, 0);
	m_rollingRadius = m_params.radius;
	m_forceGlobal = TerrainForce();
	m_bInitialized = true;
}

bool ChDiscreteTire::SampleGround(const ChTerrain& terrain, const INextGenTerrain* NextGen, const ChVector3d& loc, FGroundSample& out)
{
	out = FGroundSample();
	++m_diag.NumTerrainQueries;

	if (NextGen)
	{
		FTerrainQueryResult Result;
		if (!NextGen->QueryTerrain(loc, Result) || !Result.bHit)
			return false;
		out.bHit = true;
		out.Point = Result.Point;
		out.Friction = Result.Friction;
		out.Class = m_bDeformableSoils ? DiscreteTireSurfaceClassOf(Result.TerrainType) : ESurfaceClass::Hard;
		return true;
	}

	ChVector3d point, normal;
	double height = 0.0;
	float friction = 0.8f;
	terrain.GetProperties(loc, point, height, normal, friction);
	//Terrains report a far away height when there is no ground below (e.g. ChaosTerrain::OffMapHeight)
	if (ChWorldFrame::Height(loc) - height > 100.0)
		return false;
	out.bHit = true;
	out.Point = point;
	out.Friction = friction;
	out.Class = ESurfaceClass::Hard;
	return true;
}

void ChDiscreteTire::SetFreeContactFrame(const ChVector3d& up, const ChVector3d& axle, const WheelState& ws)
{
	ChVector3d fwd = Vcross(axle, up);
	if (fwd.Length2() < 1.0e-8)
		fwd = ws.rot.GetAxisX();
	fwd.Normalize();
	const ChVector3d lat = Vcross(up, fwd);
	ChMatrix33<> rot(fwd, lat, up);
	m_data.frame.pos = ws.pos - up * m_params.radius;
	m_data.frame.rot = rot.GetQuaternion();
	m_data.vel = ChVector3d(0, 0, 0);
	m_data.normal_force = 0.0;
	m_data.depth = 0.0;
}

void ChDiscreteTire::Synchronize(double time, const ChTerrain& terrain)
{
	m_time = time;
	m_forceGlobal = TerrainForce();
	m_tireforce.force = ChVector3d(0, 0, 0);
	m_tireforce.moment = ChVector3d(0, 0, 0);
	m_diag = FDiagnostics();
	m_bGroundValid = false;
	if (!m_bInitialized)
		return;

	const WheelState ws = m_wheel->GetState();
	const ChVector3d up = ChWorldFrame::Vertical();
	const ChVector3d C = ws.pos;

	// ---- wheel heading frame: x forward (horizontal, in the wheel plane), y along the axle, z = x cross y
	const ChVector3d axle = ws.rot.GetAxisY();
	ChVector3d fx = Vcross(axle, up);
	if (fx.Length2() < 1.0e-8)  //axle (almost) vertical, wheel lying on its side
	{
		fx = ws.rot.GetAxisX();
		fx -= Vdot(fx, axle) * axle;
	}
	fx.Normalize();
	const ChVector3d fz = Vcross(fx, axle);

	if (&terrain != m_cachedTerrain)
	{
		m_cachedTerrain = &terrain;
		m_cachedNextGen = dynamic_cast<const INextGenTerrain*>(&terrain);
	}
	const INextGenTerrain* NextGen = m_cachedNextGen;

	const int M = m_params.numStations;
	const int nCols = m_params.numColumns;
	const double R = m_params.radius;
	const double farBelow = -4.0 * R;  //where the profile is put when a query finds no ground

	// ---- airborne gate: coarse queries along the centre column. Only when every one of them is far below the
	// wheel, the tire cannot touch anything (a ditch edge or a curb next to a wheel that hangs over a hole is
	// caught by the outer stations)
	const int centreCol = nCols / 2;
	const double gate = m_params.gateFactor * R;
	std::fill(m_sampled.begin(), m_sampled.end(), char(0));
	std::vector<char>& sampled = m_sampled;

	auto sampleCell = [&](int col, int m)
	{
		const size_t idx = size_t(col) * M + size_t(m);
		if (sampled[idx])
			return;
		sampled[idx] = 1;
		const ChVector3d loc = C + axle * m_ground.colY[size_t(col)] + fx * m_ground.stationX[size_t(m)];
		FGroundSample s;
		if (SampleGround(terrain, NextGen, loc, s))
		{
			m_ground.z[idx] = Vdot(s.Point - C, fz);
			m_ground.mu[idx] = std::clamp(s.Friction, 0.05f, 1.5f);
			m_ground.cls[idx] = uint8_t(s.Class);
		}
		else
		{
			m_ground.z[idx] = farBelow;
			m_ground.mu[idx] = 0.8f;
			m_ground.cls[idx] = uint8_t(ESurfaceClass::Hard);
		}
	};

	bool bNear = false;
	const int coarse[5] = { 0, M / 4, M / 2, (3 * M) / 4, M - 1 };
	for (int k = 0; k < 5; ++k)
	{
		sampleCell(centreCol, coarse[k]);
		//height of the ground above the lowest point of the wheel plane, along the wheel "up" direction
		if (-m_ground.z[size_t(centreCol) * M + size_t(coarse[k])] < gate)
			bNear = true;
	}

	if (!bNear)
	{
		//Airborne
		if (m_bWasInContact)
			m_model.Reset();
		m_bWasInContact = false;
		m_data.in_contact = false;
		m_rollingRadius = R;
		m_diag.bAirborne = true;
		SetFreeContactFrame(up, axle, ws);
		CalculateKinematics(ws, m_data.frame);
		return;
	}

	for (int c = 0; c < nCols; ++c)
		for (int m = 0; m < M; ++m)
			sampleCell(c, m);

	m_headingX = fx;
	m_headingAxle = axle;
	m_headingZ = fz;
	m_bGroundValid = true;

	// ---- tread element directions (they rotate with the wheel body) in the heading frame
	const ChVector3d xb = ws.rot.GetAxisX();
	const ChVector3d zb = ws.rot.GetAxisZ();
	for (int i = 0; i < m_params.numTread; ++i)
	{
		const ChVector3d u = xb * m_cosA[size_t(i)] + zb * m_sinA[size_t(i)];
		m_ux[size_t(i)] = Vdot(u, fx);
		m_uz[size_t(i)] = Vdot(u, fz);
	}

	DiscreteTire::StepInput in;
	in.ground = &m_ground;
	in.ux = m_ux.data();
	in.uz = m_uz.data();
	in.vC = DiscreteTireToLocal(ws.lin_vel, fx, axle, fz);
	in.omega = DiscreteTireToLocal(ws.ang_vel, fx, axle, fz);
	in.dt = (m_lastDt > 0.0) ? m_lastDt : GetStepsize();

	DiscreteTire::StepOutput out;
	m_model.Evaluate(in, out);

	const bool bFinite = std::isfinite(out.force.x) && std::isfinite(out.force.y) && std::isfinite(out.force.z) &&
						 std::isfinite(out.moment.x) && std::isfinite(out.moment.y) && std::isfinite(out.moment.z);
	if (!bFinite)
	{
		m_model.Reset();
		out = DiscreteTire::StepOutput();
	}

	m_diag.bAirborne = false;
	m_diag.bInContact = out.inContact;
	m_diag.Load = out.load;
	m_diag.LoadHard = out.loadHard;
	m_diag.MaxPenetration = out.maxPenetration;
	m_diag.MaxTireDeflection = out.maxTireDeflection;
	m_diag.MaxSinkage = out.maxSinkage;
	m_diag.PatchLength = out.patchLength;
	m_diag.MeanFrictionRatio = out.meanMu;
	m_diag.NumContactElements = out.numContact;

	//Diagnostic for overloaded wheels (hard cornering into the bump stop, landings): rate limited to ~1 per second
	if (out.load > m_overloadWarnN && (m_time - m_lastWarnTime) > 1.0)
	{
		m_lastWarnTime = m_time;
		const double speed = ws.lin_vel.Length();
		UE_LOG(LogTemp, Warning, TEXT("DiscreteTire %hs: load %.0f N, penetration %.1f mm, tire deflection %.1f mm, Fy %.0f N, speed %.1f m/s, vertical speed %.2f m/s"),
			   GetName().c_str(), out.load, out.maxPenetration * 1000.0, out.maxTireDeflection * 1000.0, out.force.y, speed, Vdot(ws.lin_vel, up));
	}

	m_bWasInContact = out.inContact;
	m_data.in_contact = out.inContact;
	m_rollingRadius = out.inContact ? out.rollingRadius : R;

	if (!out.inContact)
	{
		SetFreeContactFrame(up, axle, ws);
		CalculateKinematics(ws, m_data.frame);
		return;
	}

	// ---- forces and moments to the world frame; the application point is the wheel centre
	const ChVector3d F = fx * out.force.x + axle * out.force.y + fz * out.force.z;
	const ChVector3d Mc = fx * out.moment.x + axle * out.moment.y + fz * out.moment.z;
	m_forceGlobal.point = C;
	m_forceGlobal.force = F;
	m_forceGlobal.moment = Mc;

	// ---- contact frame (z: mean surface normal, x: rolling direction, origin: load centroid) and the same forces
	// expressed in it (about the frame origin) for ReportTireForceLocal()
	ChVector3d n = fx * out.normal.x + axle * out.normal.y + fz * out.normal.z;
	n.Normalize();
	ChVector3d fwd = Vcross(axle, n);
	if (fwd.Length2() < 1.0e-8)
		fwd = fx;
	fwd.Normalize();
	const ChVector3d lat = Vcross(n, fwd);
	const ChVector3d origin = C + fx * out.centroid.x + axle * out.centroid.y + fz * out.centroid.z;
	ChMatrix33<> rot(fwd, lat, n);
	m_data.frame.pos = origin;
	m_data.frame.rot = rot.GetQuaternion();
	m_data.vel = m_data.frame.TransformDirectionParentToLocal(ws.lin_vel);
	m_data.normal_force = out.load;
	m_data.depth = out.maxPenetration;

	const ChVector3d Mo = Mc - Vcross(origin - C, F);
	m_tireforce.force = m_data.frame.TransformDirectionParentToLocal(F);
	m_tireforce.moment = m_data.frame.TransformDirectionParentToLocal(Mo);
	m_tireforce.point = origin;

	CalculateKinematics(ws, m_data.frame);
}

void ChDiscreteTire::Advance(double step)
{
	if (step <= 0.0)
		return;
	m_lastDt = step;
	m_model.Advance(step);
}
