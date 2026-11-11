#pragma once

// =============================================================================
// ChDiscreteTire: real time off-road handling tire for Chrono::Vehicle
//
// Base class choice: ChForceElementTire. ChTMeasyTire (and the Pacejka tires) reduce the tire to one contact point
// and a slip force law; that cannot represent a curb face, a rock or a rut. ChForceElementTire gives the parts every
// handling tire needs (inertia, force reporting, checkpointing, visualization) and leaves the contact model free.
// Nothing in the Chrono plugin is modified: this class only extends it.
//
// The contact model lives in DiscreteTireCore (no Chrono dependency, unit tested stand alone). This class:
//   * builds the local ground description of the tire every step by vertical terrain queries on a small grid
//     (columns across the tread x stations along the rolling direction) through INextGenTerrain when the terrain
//     provides it (Chaos / Bullet backends: also friction and surface type), else ChTerrain::GetProperties
//   * gates the queries: a wheel well above the ground costs one query
//   * converts velocities in, forces / moments out, and reports the contact frame like the other Chrono tires
//
// Subclasses provide the tire data in SetDiscreteTireParams() (see TestCar_DiscreteTire).
// =============================================================================

#include "ChronoTypes.h"
#include "DiscreteTireCore.h"

class INextGenTerrain;

class ChDiscreteTire : public chrono::vehicle::ChForceElementTire
{
public:
	//Per step diagnostics (physics thread), for telemetry and tuning
	struct FDiagnostics
	{
		bool bAirborne = true;
		bool bInContact = false;
		double Load = 0.0;             //sum of the normal loads [N]
		double LoadHard = 0.0;         //part on hard surface elements [N]
		double MaxPenetration = 0.0;   //max normal penetration of a tread element (carcass + soil) [m]
		double MaxTireDeflection = 0.0;
		double MaxSinkage = 0.0;       //max soil sinkage [m]
		double PatchLength = 0.0;      //[m]
		double MeanFrictionRatio = 0.0;//|Ft| / Fn of the loaded elements
		int NumContactElements = 0;
		int NumTerrainQueries = 0;
	};

	ChDiscreteTire(const std::string& name);
	virtual ~ChDiscreteTire() {}

	virtual std::string GetTemplateName() const override { return "DiscreteTire"; }

	//Effective rolling radius (R - d/3 under load), as used for the slip definitions
	virtual double GetRadius() const override { return m_rollingRadius; }
	virtual double GetWidth() const override { return m_params.width; }
	virtual double GetVisualizationWidth() const override { return m_params.width; }
	virtual double GetDeflection() const override { return m_diag.MaxTireDeflection; }

	double GetUnloadedRadius() const { return m_params.radius; }
	const FDiagnostics& GetDiagnostics() const { return m_diag; }
	const DiscreteTire::Model& GetModel() const { return m_model; }

	//Ground description of the last step for visual deformation of the tire mesh: heights [m] along HeadingZ relative to
	//the wheel centre at (station x along HeadingX, column y along Axle). Only valid when every cell was sampled this step
	//(not airborne); see DiscreteTire::GroundProfile for the layout.
	struct FGroundView
	{
		bool bValid = false;
		const DiscreteTire::GroundProfile* Profile = nullptr;
		chrono::ChVector3d HeadingX, Axle, HeadingZ;  //world frame of the profile (wheel heading frame)
	};
	FGroundView GetGroundView() const { return FGroundView{ m_bGroundValid, &m_ground, m_headingX, m_headingAxle, m_headingZ }; }
	DiscreteTire::TireParams& AccessParams() { return m_params; }

	//Deformable ground classes (sand, snow, mud) on/off. Off: every surface is treated as hard ground with the
	//friction of the terrain query.
	void SetDeformableSoilsEnabled(bool bEnable) { m_bDeformableSoils = bEnable; }

	//Override the ground law of a surface class (call after the tire was initialized)
	void SetSurface(DiscreteTire::ESurfaceClass Class, const DiscreteTire::SurfaceProps& Props) { m_model.SetSurface(Class, Props); }

	//Guess the tire data from the classic size description (same inputs as ChTMeasyTire::GuessPassCar70Par so the
	//two tires are interchangeable): section height = width * ratio, vertical stiffness from the deflection at the
	//rated load (16% of the section height), damping from the damping ratio, slip stiffnesses of a passenger tire.
	void GuessTireData(double tireLoad,       //rated load [N]
					   double tireWidth,      //[m]
					   double ratio,          //aspect ratio (0.4 = 40%)
					   double rimDia,         //rim diameter [m]
					   double pinflLi,        //inflation pressure at the load index [Pa]
					   double pinflUse,       //inflation pressure in use [Pa]
					   double dampingRatio = 0.5);

	// chrono::vehicle::ChTire
	virtual void Initialize(std::shared_ptr<chrono::vehicle::ChWheel> wheel) override;
	virtual void Synchronize(double time, const chrono::vehicle::ChTerrain& terrain) override;
	virtual void Advance(double step) override;

protected:
	//Subclass hook: fill AccessParams() (radius, width, discretization ...) and call GuessTireData().
	virtual void SetDiscreteTireParams() = 0;

	// chrono::vehicle::ChForceElementTire. The contact model does not use these two (kept for the interface and
	// for tools that query the flat road equivalents).
	virtual double GetNormalStiffnessForce(double depth) const override { return m_cz * depth; }
	virtual double GetNormalDampingForce(double depth, double velocity) const override { return m_dz * velocity; }

	virtual chrono::vehicle::TerrainForce GetTireForce() const override { return m_forceGlobal; }

	//Nominal data kept from GuessTireData
	double m_cz = 0.0;
	double m_dz = 0.0;
	double m_nominalLoad = 0.0;

private:
	struct FGroundSample
	{
		bool bHit = false;
		chrono::ChVector3d Point;
		float Friction = 0.8f;
		DiscreteTire::ESurfaceClass Class = DiscreteTire::ESurfaceClass::Hard;
	};

	bool SampleGround(const chrono::vehicle::ChTerrain& terrain, const INextGenTerrain* NextGen, const chrono::ChVector3d& loc, FGroundSample& out);
	void BuildTreadTables();
	void SetFreeContactFrame(const chrono::ChVector3d& up, const chrono::ChVector3d& axle, const chrono::vehicle::WheelState& ws);

	DiscreteTire::TireParams m_params;
	DiscreteTire::Model m_model;
	DiscreteTire::GroundProfile m_ground;
	std::vector<double> m_cosA, m_sinA;  //tread element angles in the wheel body frame
	std::vector<double> m_ux, m_uz;      //tread element directions in the wheel heading frame (per step)
	std::vector<char> m_sampled;         //ground cells already queried this step

	chrono::vehicle::TerrainForce m_forceGlobal;
	FDiagnostics m_diag;
	double m_time = 0.0;
	double m_lastWarnTime = -10.0;
	double m_overloadWarnN = 20000.0;  //log a warning above this tire load [N]
	double m_lastDt = 0.0;
	double m_rollingRadius = 0.0;
	bool m_bDeformableSoils = true;
	bool m_bInitialized = false;
	bool m_bWasInContact = false;
	bool m_bGroundValid = false;
	chrono::ChVector3d m_headingX, m_headingAxle, m_headingZ;

	const chrono::vehicle::ChTerrain* m_cachedTerrain = nullptr;
	const INextGenTerrain* m_cachedNextGen = nullptr;
};
