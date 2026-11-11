#pragma once

// =============================================================================
// Discrete off-road tire core (no Chrono / Unreal dependencies)
//
// A real time tire model that has to do three things at once:
//   * climb over rigid objects and rough terrain      -> discrete radial tread elements ray-cast against a sampled
//                                                        ground profile ("enveloping"), forces act along the local
//                                                        surface normal, so a curb face pushes the tire back and up
//   * handle well on concrete / track surfaces        -> brush type friction: every tread element carries a bristle
//                                                        (shear displacement) state, Coulomb limit with load and
//                                                        sliding speed sensitivity, calibrated to slip stiffnesses
//   * work on deformable ground (sand, snow, mud ...) -> terramechanics of Bekker / Wong / Janosi-Hanamoto, see
//                                                        L. Mancardi, "Implementation of an off-road tyre model for
//                                                        real-time dynamic driving simulator", Politecnico di Torino
//                                                        2020 (chapters 3 to 5)
//
// How the thesis model maps onto this core
//   thesis                                          | here
//   ------------------------------------------------+---------------------------------------------------------------
//   sigma = (kc/b + kphi) z^n              (3.8)    | SoilCurve: loading branch of the pressure sinkage relation
//   unloading line p = pu - ku (zu - z)    (3.12)   | SoilCurve: unloading branch, per element memory (zu, su) ->
//                                                    | the asymmetric stress distribution that makes compaction
//                                                    | resistance, without the thesis' fixed exit angle assumption
//   tau = (c + sigma tan(phi)) (1-exp(-j/K)) (3.13) | ShearStress(): same law, j is the element's shear displacement
//   jx = Ru[(te-t) - (1-Sd)(sin te - sin t)] (3.4)  | integrated in time from the true tread tip velocity, which is
//                                                    | exactly what (3.4) is in closed form for steady rolling. Works
//                                                    | at standstill, reversing, locked wheel, on slopes and obstacles
//   entry angle by fzero / lookup table   (5.2/5.3) | not needed: the wheel position is a dynamic state of the
//                                                    | vehicle, the tire returns the force for the given penetration
//                                                    | (sinkage is found by the vehicle solver, not by iteration)
//   soil damping (7.4.1)                            | SurfaceProps::damping, viscous, on the soil share of the stroke
//   Fz, Fx, Fy, My integrals (3.14 - 3.17)          | sums over the elements (plus Mx, Mz for free)
//   not modelled in the thesis: multi pass, bulldozing -> not modelled here either (hooks are documented in the .cpp)
//
// Frame convention (used by everything in this header): the "wheel heading frame" attached to the wheel centre
//   x  forward (horizontal, in the wheel plane)
//   y  along the axle (to the left)
//   z  x cross y, i.e. "up" in the wheel plane (world up for an upright wheel)
// The Chrono glue (ChDiscreteTire) converts to and from world space.
// =============================================================================

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace DiscreteTire
{
	constexpr double Pi = 3.14159265358979323846;

	//Surface classes with their own contact law. Hard = rigid ground, tire deflects (asphalt, concrete, rock, ice ...,
	//friction comes from the terrain query). The rest are deformable soils.
	enum class ESurfaceClass : uint8_t
	{
		Hard = 0,
		Sand,
		Snow,
		Mud,
		Count
	};

	//Deformable soil / hard surface description of one surface class
	struct SurfaceProps
	{
		bool deformable = false;

		//Bekker pressure sinkage sigma = (kc / b + kphi) z^n. Units as in the thesis: kc [N/m^(n+1)], kphi [N/m^(n+2)]
		double kc = 0.0;
		double kphi = 0.0;
		double n = 1.0;

		//Mohr Coulomb: tau_max = c + sigma tan(phi)
		double cohesion = 0.0;  //[Pa]
		double phi = 0.0;       //[rad]

		//Janosi Hanamoto shear deformation moduli [m]
		double Kx = 0.025;
		double Ky = 0.025;

		//Unloading stiffness as a multiple of the secant loading stiffness at the maximum sinkage (Wong's ku). Larger
		//means more plastic (less springback). 1 = elastic soil, no compaction resistance.
		double unloadRatio = 8.0;

		//Viscous damping of the soil [Pa s/m] (thesis 7.4.1)
		double damping = 0.0;
	};

	//Library of soil presets. Dry sand, LETE sand and sandy loam are the exact data of the thesis (tables 1 to 3, taken
	//from the Adams/Car soil database). Snow and mud are typical literature values (Wong, "Theory of Ground Vehicles")
	//and are NOT validated, tune them before relying on them.
	SurfaceProps MakeDrySand();
	SurfaceProps MakeLeteSand();
	SurfaceProps MakeSandyLoam();
	SurfaceProps MakeSnow();
	SurfaceProps MakeMud();

	//Persistent state of one tread element (ring index x lateral strip). Elements are fixed to the tread, so the state
	//rolls with the wheel and is reset when the element leaves the ground.
	struct ElementState
	{
		bool inContact = false;
		double jx = 0.0, jy = 0.0;  //shear displacement of the tread w.r.t. the ground (bristle deflection) [m]
		double zu = 0.0, su = 0.0;  //soil memory: maximum sinkage reached [m] and the stress at that point [Pa]
		double vx = 0.0, vy = 0.0;  //tread slip velocity in the element tangent frame of the last evaluation [m/s]
		double jcap = 0.0;          //limit of the shear displacement magnitude [m]
		double eta = 1.0;           //lateral / longitudinal shear stiffness ratio
		double sigma = 0.0;         //normal contact stress of the last evaluation [Pa] (diagnostics)
		double tau = 0.0;           //magnitude of the shear stress of the last evaluation [Pa] (diagnostics)
		double tauMax = 0.0;        //shear strength of the last evaluation [Pa] (diagnostics)
	};

	struct TireParams
	{
		//Geometry
		double radius = 0.39;     //unloaded radius [m]
		double width = 0.26;      //tread width [m]
		double crownDrop = 0.006; //radius reduction at the shoulders [m] (drives camber response)

		//Discretization
		int numTread = 120;      //tread elements around the circumference
		int numStrips = 5;      //lateral strips
		int numColumns = 3;     //lateral columns of ground samples (<= numStrips), interpolated to the strips
		int numStations = 15;   //ground samples per column (odd, along the rolling direction)
		double reachScale = 1.05; //ground samples span +- reachScale * radius
		double maxUpComp = 0.40;  //tread elements above this up component (uz) are ignored (roof of the wheel)

		//Vertical: contact stress sigma = kp d / sqrt(d + eps) of the normal penetration d of a tread element. For
		//d >> eps this is kp sqrt(d), which gives a stiffness that is linear in the wheel deflection on flat ground
		//(patch length ~ sqrt(d), stress ~ sqrt(d)... integrated over the patch: Fz ~ d). The regularisation eps
		//makes the stress grow linearly at the edge of the patch (no infinite slope, no ripple from the ring).
		//Beyond dlim the rim / bump stop takes over with the stiffness kbump. cp is the viscous stress per unit
		//velocity of the WHEEL CENTRE towards the ground (not of the tread passing through the patch).
		double kp = 7.0e5;       //[Pa / sqrt(m)]
		double eps = 0.001;      //[m]
		double cp = 8.0e4;       //[Pa s / m]
		double dlim = 0.06;      //[m] normal deflection where the bump stop starts
		double kbump = 2.0e8;    //[Pa / m]
		double dampRamp = 0.002; //damping fades in over this penetration [m] (no force jump at first touch)
		//The viscous stress is limited to dampCap times the elastic stress of the element. Without it the damper force
		//grows without bound with the impact speed (Dz * 7 m/s is more than 25 kN for a wheel landing hard), the load
		//jumps and, because the tread friction follows the load, the lateral force with it. That is the kind of load
		//spike that pulls a wheel off its spindle in a hard turn over a bump.
		double dampCap = 2.0;
		//Fuse: total force on the tire [N] above which force and moment are scaled down together (keeps the direction).
		//Set by GuessTireData() from the rated load; a wheel that gets there is already bottomed on its rim.
		double forceLimit = 1.0e12;

		//The effective rolling radius of a loaded tire is Re = R - rollFactor * d (Rill: 1/3) with d the deflection,
		//zero slip <=> V = omega * Re. (The tread itself runs on the loaded radius R - d, see Evaluate.)
		double rollFactor = 1.0 / 3.0;

		//Tangential (hard surfaces): initial slope of the shear stress vs displacement [Pa/m], friction level
		double kbx = 1.9e7;
		double kby = 1.7e7;
		double tauDamp = 0.005;    //bristle damping time [s] (keeps the explicit tangential force stable, needs > dt)
		double dampSpeed = 2.0;    //[m/s] the bristle damper fades out above this rolling speed
		//Extra bristle damping near standstill. The damper above is a stability aid (tauDamp = 5 ms), but it is also the
		//ONLY tangential damping the tire has, and a car standing on the tangential (shear) stiffness of its tires
		//is a mass on a spring: the damping ratio is zeta = tauDamp * omega / 2. On sand (omega ~ 8 rad/s, 1.3 Hz)
		//that is 0.02, so after a hard stop the car rings back and forth for seconds. The effective damping time is
		//tauDamp + (tauDampStandstill - tauDamp) * 1 / (1 + (vRoll / standstillSpeed)^2). The damper acts on the SLIP
		//velocity of the tread (not on the vehicle speed) and is limited by the friction strength, so a freely rolling
		//wheel feels almost nothing. Equal to tauDamp (the default) means no extra damping, i.e. the previous behaviour.
		double tauDampStandstill = 0.005; //[s]
		double standstillSpeed = 1.5;     //[m/s] the extra standstill damping fades out above this rolling speed
		double gripScale = 1.70;   //peak friction of the tire rubber relative to the terrain friction value
		double loadSens = 0.10;    //friction ~ (sigmaRef / sigma)^loadSens
		double sigmaRef = 2.5e5;   //[Pa]
		double slideDrop = 0.35;   //friction loss at high sliding speed (creates the peak of the slip curve)
		double slideSpeed = 5.0;   //[m/s]
		//Load saturation of the friction (like TMeasy's pn_max): the stress used for the shear strength saturates
		//softly, sigma_f = cap * tanh(sigma / cap). Without it the lateral force keeps growing with the load and a
		//wheel driven hard into its bump stop would receive tens of kN sideways. Set by Calibrate().
		double sigmaFrictionCap = 1.0e12;
		double muMin = 0.05;
		double muMax = 2.0;

		//Rolling resistance of the hard contact (soil compaction resistance comes from the soil model itself)
		double rollingResistance = 0.015;
		double rollingOmega0 = 1.0; //[rad/s] smoothing of the sign of the wheel speed

		//Ground gate: wheel further than gateFactor * radius above the ground below its centre is airborne
		double gateFactor = 1.2;

		//Empirical corrections of Calibrate() (the closed forms assume small angles and an infinitely fine ring, the
		//saturation of the shear law lowers the initial slope). Determined with the standalone test harness.
		double corrVertical = 1.3;
		double corrLongSlip = 1.21;
		double corrLatSlip = 1.14;

		//Fill kp, cp, kbx, kby from the classic tire data (vertical stiffness/damping [N/m, Ns/m], nominal load and
		//the normalized slip stiffnesses C/Fz of the flat road contact patch).
		void Calibrate(double cz, double dz, double nominalLoad, double normLongStiff, double normLatStiff);

		int NumElements() const { return numTread * numStrips; }
	};

	//Ground description of one step, all in the wheel heading frame (heights relative to the wheel centre, negative
	//below it). Filled by the glue with vertical terrain queries at (station x, column y).
	struct GroundProfile
	{
		int numColumns = 0;
		int numStations = 0;
		std::vector<double> colY;      //[numColumns] lateral offset of each column
		std::vector<double> stationX;  //[numStations] station positions along x
		std::vector<double> z;         //[col * numStations + m] ground height
		std::vector<float> mu;         //terrain friction value
		std::vector<uint8_t> cls;      //ESurfaceClass

		void Resize(int columns, int stations)
		{
			numColumns = columns;
			numStations = stations;
			colY.assign(columns, 0.0);
			stationX.assign(stations, 0.0);
			z.assign(size_t(columns) * stations, 0.0);
			mu.assign(size_t(columns) * stations, 0.8f);
			cls.assign(size_t(columns) * stations, uint8_t(ESurfaceClass::Hard));
		}
	};

	struct Vec3
	{
		double x = 0, y = 0, z = 0;
	};

	struct StepInput
	{
		const GroundProfile* ground = nullptr;
		//In-plane unit direction of every tread element (wheel heading frame): ux forward, uz up. numTread entries.
		const double* ux = nullptr;
		const double* uz = nullptr;
		Vec3 vC;         //wheel centre velocity (heading frame)
		Vec3 omega;      //wheel angular velocity (heading frame), omega.y is the spin
		double dt = 0.003; //expected step (used to estimate the age of a new contact inside the step)
	};

	struct StepOutput
	{
		bool inContact = false;
		Vec3 force;    //total force on the tire (heading frame)
		Vec3 moment;   //total moment about the wheel centre (heading frame)
		Vec3 centroid; //load weighted contact point
		Vec3 normal;   //load weighted mean surface normal (unit)
		double load = 0.0;         //sum of the normal loads [N]
		double loadHard = 0.0;     //part carried by hard surface elements [N]
		double maxPenetration = 0; //max normal penetration (tire deflection + soil sinkage) [m]
		double maxTireDeflection = 0;
		double maxSinkage = 0;     //max soil sinkage [m]
		double meanMu = 0;         //load weighted tangential friction ratio (|F_t| / F_n of the elements)
		double patchLength = 0;    //extent of the loaded elements along x [m]
		double rollingRadius = 0;  //effective rolling radius [m]
		int numContact = 0;
	};

	class Model
	{
	public:
		void Configure(const TireParams& params);
		void SetSurface(ESurfaceClass cls, const SurfaceProps& props);
		const SurfaceProps& GetSurface(ESurfaceClass cls) const { return surfaces[size_t(cls)]; }
		const TireParams& GetParams() const { return P; }

		//Drop all tread state (tire teleported / restarted)
		void Reset();

		//Forces and moments for the given ground; updates the element states that do not need the time step
		void Evaluate(const StepInput& in, StepOutput& out);

		//Integrate the shear displacements of the elements in contact
		void Advance(double dt);

		const std::vector<ElementState>& Elements() const { return states; }

	private:
		TireParams P;
		SurfaceProps surfaces[size_t(ESurfaceClass::Count)];
		std::vector<ElementState> states;

		//Per step scratch (allocated once)
		std::vector<double> segX, segH, segGy, segMu;
		std::vector<uint8_t> segCls;
		std::vector<int> nearSeg;
		struct Hit
		{
			bool ok = false;
			double r = 0, n2x = 0, n2z = 1, gy = 0, mu = 0.8, c2 = 1, deltaR = 0;
			uint8_t cls = 0;
		};
		std::vector<Hit> hits;
		double dAlpha = 0.0;
		double lastMaxDefl = 0.0;  //carcass deflection of the previous evaluation (rolling radius offset)

		void ResetState(ElementState& s) const { s = ElementState(); }
	};
}
