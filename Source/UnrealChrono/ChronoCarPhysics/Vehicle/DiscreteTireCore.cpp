#include "DiscreteTireCore.h"

namespace DiscreteTire
{
	// -------------------------------------------------------------------------
	// Surface presets
	// -------------------------------------------------------------------------

	SurfaceProps MakeDrySand()
	{
		//Thesis table 1 (dry sand, similar to desert sand)
		SurfaceProps s;
		s.deformable = true;
		s.kc = 950.0;
		s.kphi = 1530000.0;
		s.n = 1.1;
		s.cohesion = 1040.0;
		s.phi = 0.489;
		s.Kx = 0.036;
		s.Ky = 0.013;
		s.damping = 3.0e4;
		return s;
	}

	SurfaceProps MakeLeteSand()
	{
		//Thesis table 2 (LETE sand, friable dirt, more compact than the dry sand)
		SurfaceProps s;
		s.deformable = true;
		s.kc = 6940.0;
		s.kphi = 505800.0;
		s.n = 0.71;
		s.cohesion = 1150.0;
		s.phi = 0.5498;
		s.Kx = 0.036;
		s.Ky = 0.013;
		s.damping = 3.0e4;
		return s;
	}

	SurfaceProps MakeSandyLoam()
	{
		//Thesis table 3 (sandy loam, river banks)
		SurfaceProps s;
		s.deformable = true;
		s.kc = 6900.0;
		s.kphi = 752000.0;
		s.n = 0.66;
		s.cohesion = 3700.0;
		s.phi = 0.520;
		s.Kx = 0.036;
		s.Ky = 0.013;
		s.damping = 3.0e4;
		return s;
	}

	SurfaceProps MakeSnow()
	{
		//Fresh snow, literature values (Wong). Not validated.
		SurfaceProps s;
		s.deformable = true;
		s.kc = 4370.0;
		s.kphi = 196720.0;
		s.n = 1.6;
		s.cohesion = 1030.0;
		s.phi = 0.344;
		s.Kx = 0.025;
		s.Ky = 0.025;
		s.unloadRatio = 10.0;
		s.damping = 2.0e4;
		return s;
	}

	SurfaceProps MakeMud()
	{
		//Clayey soil, literature values (Wong). Not validated.
		SurfaceProps s;
		s.deformable = true;
		s.kc = 13190.0;
		s.kphi = 692150.0;
		s.n = 0.5;
		s.cohesion = 4140.0;
		s.phi = 0.227;
		s.Kx = 0.010;
		s.Ky = 0.010;
		s.unloadRatio = 10.0;
		s.damping = 4.0e4;
		return s;
	}

	// -------------------------------------------------------------------------
	// Calibration from classic tire data
	// -------------------------------------------------------------------------

	void TireParams::Calibrate(double cz, double dz, double nominalLoad, double normLongStiff, double normLatStiff)
	{
		const double d0 = std::max(nominalLoad / std::max(cz, 1.0), 1.0e-3);

		//Flat ground, small angles: with sigma = kp sqrt(delta) the vertical force is linear in the deflection d,
		//Fz = kp * width * pi * sqrt(R/2) * d  ->  Cz
		kp = corrVertical * cz / (width * Pi * std::sqrt(0.5 * radius));

		//Damping force of the elements at the nominal deflection: 2 cp width sqrt(2 R d0) per unit velocity -> Dz
		cp = dz / (2.0 * width * std::sqrt(2.0 * radius * d0));

		//Brush model: slip stiffness C = kb * width * l^2 / 2 with the patch length l^2 = 8 R d = 8 R Fz / Cz
		kbx = corrLongSlip * normLongStiff * cz / (4.0 * width * radius);
		kby = corrLatSlip * normLatStiff * cz / (4.0 * width * radius);

		kbump = 16.0 * kp / (2.0 * std::sqrt(std::max(dlim, 1.0e-3)));

		//Mean contact pressure at the nominal load (patch length sqrt(8 R d0)); friction saturates at 3x that
		const double p0 = nominalLoad / (width * std::sqrt(8.0 * radius * d0));
		sigmaFrictionCap = 3.0 * p0;
	}

	// -------------------------------------------------------------------------
	// Contact laws
	// -------------------------------------------------------------------------

	namespace
	{
		//Tire carcass: sigma(d) = kp d / sqrt(d + eps), linear continuation with kbump beyond dlim, and its inverse
		inline double TireStress(const TireParams& p, double d)
		{
			if (d <= 0.0)
				return 0.0;
			if (d <= p.dlim)
				return p.kp * d / std::sqrt(d + p.eps);
			return p.kp * p.dlim / std::sqrt(p.dlim + p.eps) + p.kbump * (d - p.dlim);
		}

		inline double TireDeflection(const TireParams& p, double s)
		{
			if (s <= 0.0)
				return 0.0;
			const double sl = p.kp * p.dlim / std::sqrt(p.dlim + p.eps);
			if (s <= sl)
			{
				//kp^2 d^2 - s^2 d - s^2 eps = 0
				const double s2 = s * s;
				return (s2 + std::sqrt(s2 * s2 + 4.0 * p.kp * p.kp * s2 * p.eps)) / (2.0 * p.kp * p.kp);
			}
			return p.dlim + (s - sl) / p.kbump;
		}

		inline double TireDeflectionSlope(const TireParams& p, double s)
		{
			const double sl = p.kp * p.dlim / std::sqrt(p.dlim + p.eps);
			if (s <= sl)
			{
				const double d = TireDeflection(p, s);
				const double dsdd = p.kp * std::pow(d + p.eps, -1.5) * (0.5 * d + p.eps);
				return 1.0 / dsdd;
			}
			return 1.0 / p.kbump;
		}

		//Soil under one tread element: Bekker loading branch plus the Wong unloading line through the point of
		//maximum sinkage (zu, su). Sink(s) is the inverse: sinkage as function of the contact stress.
		struct SoilCurve
		{
			double ks = 1.0;     //kc / b + kphi
			double invn = 1.0;
			double zu = 0.0;
			double su = 0.0;
			double ku = 1.0;

			double Sink(double s) const
			{
				if (s >= su)
					return std::pow(s / ks, invn);
				return zu - (su - s) / ku;
			}

			double Slope(double s) const
			{
				if (s >= su)
				{
					s = std::max(s, 1.0e-9);
					return invn / ks * std::pow(s / ks, invn - 1.0);
				}
				return 1.0 / ku;
			}
		};

		//Carcass and soil are in series: the penetration of the tread tip into the (undeformed) ground is shared,
		//delta = deflection(sigma) + sinkage(sigma), with the same contact stress in both. Monotone in sigma, solved
		//with a safeguarded Newton iteration. Returns the stress; sinkage is returned through zOut.
		double SolveSeries(const TireParams& p, const SoilCurve& soil, double delta, double& zOut)
		{
			const double z0 = soil.Sink(0.0);  //permanent set of the rut (0 without memory)
			if (z0 >= delta)
			{
				zOut = std::min(z0, delta);
				return 0.0;  //the tread does not reach the bottom of the rut
			}

			double lo = 0.0;
			double hi = TireStress(p, delta);  //g(hi) >= 0 because the carcass alone would take all of delta
			double s = hi;
			for (int it = 0; it < 40; ++it)
			{
				const double g = TireDeflection(p, s) + soil.Sink(s) - delta;
				if (std::fabs(g) < 2.0e-7)
					break;
				if (g > 0.0)
					hi = s;
				else
					lo = s;
				const double dg = TireDeflectionSlope(p, s) + soil.Slope(s);
				double sn = (dg > 1.0e-30) ? s - g / dg : 0.5 * (lo + hi);
				if (!(sn > lo && sn < hi))
					sn = 0.5 * (lo + hi);
				s = sn;
			}
			zOut = soil.Sink(s);
			return s;
		}
	}

	// -------------------------------------------------------------------------
	// Model
	// -------------------------------------------------------------------------

	void Model::Configure(const TireParams& params)
	{
		P = params;
		P.numTread = std::max(P.numTread, 12);
		P.numStrips = std::max(P.numStrips, 1);
		P.numColumns = std::clamp(P.numColumns, 1, P.numStrips);
		P.numStations = std::max(P.numStations | 1, 3);  //odd, at least 3 (a station at the wheel centre)

		dAlpha = 2.0 * Pi / P.numTread;
		states.assign(size_t(P.NumElements()), ElementState());

		const size_t points = size_t(P.numStations) + 2;
		segX.assign(points, 0.0);
		segH.assign(points, 0.0);
		segGy.assign(points, 0.0);
		segMu.assign(points, 0.8);
		segCls.assign(points, uint8_t(ESurfaceClass::Hard));
		nearSeg.reserve(points);

		surfaces[size_t(ESurfaceClass::Hard)] = SurfaceProps();
		surfaces[size_t(ESurfaceClass::Sand)] = MakeDrySand();
		surfaces[size_t(ESurfaceClass::Snow)] = MakeSnow();
		surfaces[size_t(ESurfaceClass::Mud)] = MakeMud();
	}

	void Model::SetSurface(ESurfaceClass cls, const SurfaceProps& props)
	{
		surfaces[size_t(cls)] = props;
	}

	void Model::Reset()
	{
		for (ElementState& s : states)
			ResetState(s);
	}

	void Model::Evaluate(const StepInput& in, StepOutput& out)
	{
		out = StepOutput();
		out.rollingRadius = P.radius;

		const GroundProfile& G = *in.ground;
		const int M = G.numStations;
		const int W = P.numStrips;
		const int N = P.numTread;
		const double stripW = P.width / W;
		const double ext = 3.0 * P.radius;  //flat extension of the profile beyond the outermost stations

		//The bristle damper only keeps the explicit tangential force stable at standstill / crawling speed; it must
		//not add a speed proportional slip stiffness while rolling, so it fades out with speed.
		const double vRoll = std::max(std::sqrt(in.vC.x * in.vC.x + in.vC.y * in.vC.y), std::fabs(in.omega.y) * P.radius);
		const double dampScale = 1.0 / (1.0 + (vRoll / P.dampSpeed) * (vRoll / P.dampSpeed));
		//Damping time: tauDamp while rolling, growing towards tauDampStandstill as the tire comes to rest (see TireParams)
		const double standstillWeight = 1.0 / (1.0 + (vRoll / P.standstillSpeed) * (vRoll / P.standstillSpeed));
		const double tauDampEff = P.tauDamp + (P.tauDampStandstill - P.tauDamp) * standstillWeight;
		const double beltOffset = (1.0 - P.rollFactor) * lastMaxDefl;

		double sumLoad = 0.0, sumTangential = 0.0, sumHard = 0.0;
		Vec3 centroid, normalSum;
		double minX = 1.0e9, maxX = -1.0e9;

		for (int j = 0; j < W; ++j)
		{
			const double yj = (j - 0.5 * (W - 1)) * stripW;
			const double t = 2.0 * yj / P.width;
			const double Rj = P.radius - P.crownDrop * t * t;

			// ---- profile of this strip: interpolate between the two bracketing sample columns
			int ca = 0, cb = 0;
			double frac = 0.0;
			if (G.numColumns > 1)
			{
				ca = 0;
				while (ca < G.numColumns - 2 && yj > G.colY[ca + 1])
					++ca;
				cb = ca + 1;
				const double span = G.colY[cb] - G.colY[ca];
				frac = span > 1.0e-9 ? std::clamp((yj - G.colY[ca]) / span, 0.0, 1.0) : 0.0;
			}
			const double dy = (cb != ca) ? (G.colY[cb] - G.colY[ca]) : 1.0;

			for (int m = 0; m < M; ++m)
			{
				const double ha = G.z[size_t(ca) * M + m];
				const double hb = G.z[size_t(cb) * M + m];
				const size_t k = size_t(m) + 1;
				segX[k] = G.stationX[m];
				segH[k] = ha + (hb - ha) * frac;
				segGy[k] = (cb != ca) ? (hb - ha) / dy : 0.0;
				const size_t src = ((frac < 0.5) ? size_t(ca) : size_t(cb)) * M + m;
				segMu[k] = G.mu[src];
				segCls[k] = G.cls[src];
			}
			segX[0] = G.stationX[0] - ext;
			segH[0] = segH[1];
			segGy[0] = segGy[1];
			segMu[0] = segMu[1];
			segCls[0] = segCls[1];
			segX[size_t(M) + 1] = G.stationX[M - 1] + ext;
			segH[size_t(M) + 1] = segH[size_t(M)];
			segGy[size_t(M) + 1] = segGy[size_t(M)];
			segMu[size_t(M) + 1] = segMu[size_t(M)];
			segCls[size_t(M) + 1] = segCls[size_t(M)];

			// ---- segments that can be reached by a tread element (closer than the tread radius)
			nearSeg.clear();
			const double reach2 = (Rj + 0.02) * (Rj + 0.02);
			double groundAtCentre = segH[1];
			for (int s = 0; s <= M; ++s)
			{
				const double x0 = segX[size_t(s)], h0 = segH[size_t(s)];
				const double ex = segX[size_t(s) + 1] - x0, eh = segH[size_t(s) + 1] - h0;
				const double l2 = ex * ex + eh * eh;
				const double tt = l2 > 1.0e-18 ? std::clamp(-(x0 * ex + h0 * eh) / l2, 0.0, 1.0) : 0.0;
				const double px = x0 + tt * ex, ph = h0 + tt * eh;
				if (px * px + ph * ph <= reach2)
					nearSeg.push_back(s);
				if (x0 <= 0.0 && x0 + ex >= 0.0 && ex > 1.0e-12)
					groundAtCentre = h0 + eh * (-x0 / ex);
			}
			const bool buried = groundAtCentre > 0.0;  //wheel centre below the ground surface: emergency push-out
			if (nearSeg.empty() && !buried)
			{
				for (int i = 0; i < N; ++i)
				{
					ElementState& es = states[size_t(j) * N + i];
					if (es.inContact)
						ResetState(es);
				}
				continue;
			}

			// ---- phase 1: ray of every tread element against the profile, and the entry edge of the patch
			hits.resize(size_t(N));
			double xFront = -1.0e9, xRear = 1.0e9;
			for (int i = 0; i < N; ++i)
			{
				Hit& h = hits[size_t(i)];
				h = Hit();
				const double ux = in.ux[i], uz = in.uz[i];
				if (uz > P.maxUpComp)
					continue;

				double r = 0.0, n2x = 0.0, n2z = 1.0, tHit = 0.0, gy = 0.0, mu = 0.8;
				uint8_t cls = uint8_t(ESurfaceClass::Hard);
				bool found = false;
				if (buried)
				{
					found = true;
					gy = 0.0;
					mu = segMu[1];
					cls = segCls[1];
				}
				else
				{
					double best = Rj;
					int bestSeg = -1;
					for (int s : nearSeg)
					{
						const double x0 = segX[size_t(s)], h0 = segH[size_t(s)];
						const double ex = segX[size_t(s) + 1] - x0, eh = segH[size_t(s) + 1] - h0;
						const double den = ux * eh - uz * ex;
						if (std::fabs(den) < 1.0e-12)
							continue;
						const double rr = (x0 * eh - h0 * ex) / den;
						const double tt = (x0 * uz - h0 * ux) / den;
						if (rr > 1.0e-6 && rr < best && tt >= -1.0e-9 && tt <= 1.0 + 1.0e-9)
						{
							best = rr;
							bestSeg = s;
							tHit = tt;
						}
					}
					if (bestSeg >= 0)
					{
						const size_t s0 = size_t(bestSeg), s1 = s0 + 1;
						const double ex = segX[s1] - segX[s0], eh = segH[s1] - segH[s0];
						const double invL = 1.0 / std::sqrt(ex * ex + eh * eh);
						found = true;
						r = best;
						n2x = -eh * invL;
						n2z = ex * invL;
						gy = segGy[s0] + tHit * (segGy[s1] - segGy[s0]);
						mu = segMu[s0] + tHit * (segMu[s1] - segMu[s0]);
						cls = (tHit < 0.5) ? segCls[s0] : segCls[s1];
					}
				}
				if (!found)
					continue;

				double c2 = -(ux * n2x + uz * n2z);
				if (buried)
					c2 = std::max(c2, 0.2);
				const double deltaR = Rj - r;
				if (c2 < 0.02 || deltaR <= 0.0)
					continue;

				h.ok = true;
				h.r = r;
				h.n2x = n2x;
				h.n2z = n2z;
				h.gy = gy;
				h.mu = mu;
				h.cls = cls;
				h.c2 = c2;
				h.deltaR = deltaR;
				xFront = std::max(xFront, r * ux);
				xRear = std::min(xRear, r * ux);
			}
			//The tread enters the patch at the front when the wheel rolls forward, at the rear when it rolls backward
			const bool forwardRoll = in.omega.y >= 0.0;
			const double xEntry = forwardRoll ? xFront : xRear;

			// ---- phase 2: contact laws
			for (int i = 0; i < N; ++i)
			{
				ElementState& es = states[size_t(j) * N + i];
				const Hit& h = hits[size_t(i)];
				if (!h.ok)
				{
					if (es.inContact)
						ResetState(es);
					continue;
				}
				const double ux = in.ux[i], uz = in.uz[i];
				const double r = h.r, n2x = h.n2x, n2z = h.n2z, gy = h.gy, c2 = h.c2, deltaR = h.deltaR;
				const double mu = h.mu;
				const uint8_t cls = h.cls;
				const double deltaN = deltaR * c2;  //penetration of the tread tip normal to the surface

				// ---- 3D surface normal: in-plane slope from the profile segment, cross slope from the columns
				Vec3 n{ n2x, -gy * n2z, n2z };
				{
					const double inv = 1.0 / std::sqrt(n.x * n.x + n.y * n.y + n.z * n.z);
					n.x *= inv;
					n.y *= inv;
					n.z *= inv;
				}

				// ---- normal stress. A fresh element has no soil memory: its state is reset when it leaves the ground.
				const SurfaceProps& surf = surfaces[std::min<size_t>(cls, size_t(ESurfaceClass::Count) - 1)];
				double sigmaStatic = 0.0, zSoil = 0.0;
				if (surf.deformable)
				{
					SoilCurve soil;
					soil.ks = surf.kc / P.width + surf.kphi;
					soil.invn = 1.0 / std::max(surf.n, 0.05);
					soil.zu = es.zu;
					soil.su = es.su;
					soil.ku = (es.zu > 1.0e-9 && es.su > 0.0) ? std::max(surf.unloadRatio, 1.0) * es.su / es.zu : 1.0;
					sigmaStatic = SolveSeries(P, soil, deltaN, zSoil);
					if (sigmaStatic > 0.0 && zSoil > es.zu)  //loading: remember the deepest point
					{
						es.zu = zSoil;
						es.su = sigmaStatic;
					}
				}
				else
				{
					sigmaStatic = TireStress(P, deltaN);
				}

				//The damper acts on the approach velocity of the wheel centre towards the surface, not on the tread
				//passing through the patch (that would be a speed dependent rolling drag of unrealistic size)
				const double ddeltaC = -(in.vC.x * n.x + in.vC.y * n.y + in.vC.z * n.z);
				const double share = surf.deformable ? std::clamp(zSoil / deltaN, 0.0, 1.0) : 0.0;
				const double cEff = P.cp * (1.0 - share) + surf.damping * share;
				const double ramp = std::min(1.0, deltaN / P.dampRamp);
				double sigma = 0.0;
				if (sigmaStatic > 1.0e-9)
					sigma = std::max(0.0, sigmaStatic + std::min(cEff * ramp * ddeltaC, P.dampCap * sigmaStatic));

				// ---- effective contact frame. On hard ground the tire flattens against the ground: pressure along the ground
				// normal, shear in the ground plane. On soil the wheel stays round and the soil conforms to it: pressure
				// radial on the wheel arc and shear tangent to it (thesis eq. 3.14 / 3.15: Fx = int(tau cos - sigma sin)),
				// which is where the compaction resistance comes from. Blend by the soil share of the penetration.
				Vec3 nf = n;
				double areaFactor = c2;  //tread area projected on the contact surface
				if (share > 0.0)
				{
					Vec3 d{ -ux, 0.0, -uz };
					nf = Vec3{ (1.0 - share) * n.x + share * d.x, (1.0 - share) * n.y, (1.0 - share) * n.z + share * d.z };
					const double inv = 1.0 / std::sqrt(nf.x * nf.x + nf.y * nf.y + nf.z * nf.z);
					nf.x *= inv;
					nf.y *= inv;
					nf.z *= inv;
					areaFactor = (1.0 - share) * c2 + share;
				}

				// ---- kinematics of the tread where it meets the ground. On hard ground the tread tip is at the hit
				// point, so for flat ground r*cos(phi) is constant along the patch and the slip velocity is uniform (a
				// flattened belt). On soil the tread stays at the radius reduced by the carcass deflection only and
				// the soil surface cuts through it: rigid wheel arc, v = R w - Vx cos(theta) as in the thesis (3.3).
				const double tireDefR = std::max(0.0, deltaN - zSoil) / c2;
				const double rTip = Rj - tireDefR;
				const Vec3 Pt{ rTip * ux, yj, rTip * uz };
				const Vec3 v{ in.vC.x + in.omega.y * Pt.z - in.omega.z * Pt.y,
							  in.vC.y + in.omega.z * Pt.x - in.omega.x * Pt.z,
							  in.vC.z + in.omega.x * Pt.y - in.omega.y * Pt.x };
				const double vn = v.x * nf.x + v.y * nf.y + v.z * nf.z;
				const double ddelta = -vn;  //penetration rate of the tread material (age of a new contact)
				const Vec3 vt{ v.x - vn * nf.x, v.y - vn * nf.y, v.z - vn * nf.z };

				//Tangent frame of the element: x along the rolling direction (axle cross normal), y = n cross x
				Vec3 xk{ nf.z, 0.0, -nf.x };
				{
					const double l = std::sqrt(xk.x * xk.x + xk.z * xk.z);
					if (l > 1.0e-6)
					{
						xk.x /= l;
						xk.z /= l;
					}
					else
					{
						xk = Vec3{ 1.0, 0.0, 0.0 };
					}
				}
				const Vec3 yk{ nf.y * xk.z - nf.z * xk.y, nf.z * xk.x - nf.x * xk.z, nf.x * xk.y - nf.y * xk.x };
				//A loaded tire rolls on R - rollFactor * d, larger than the loaded radius r_stat = R - d the tip
				//velocity was computed for: the belt runs faster by omega * (1 - rollFactor) * d (hard ground only).
				const double vx = vt.x * xk.x + vt.y * xk.y + vt.z * xk.z - in.omega.y * beltOffset * (1.0 - share);
				const double vy = vt.x * yk.x + vt.y * yk.y + vt.z * yk.z;

				// ---- new contact: the element touched the ground somewhere inside the last step
				if (!es.inContact)
				{
					es.inContact = true;
					//Age of the new contact inside the step: either it dipped into the ground (penetration rate), or the
					//tread carried it into the patch from the entry edge (a fast rolling wheel turns by about the length of
					//the patch per step, so an element can appear anywhere in the patch)
					const double tread = std::fabs(in.omega.y) * rTip;
					const double ageRoll = (tread > 1.0e-3) ? std::fabs(xEntry - r * ux) / tread : 0.0;
					const double ageDip = (ddelta > 1.0e-6) ? deltaN / ddelta : 0.0;
					const double age = std::clamp(std::max(ageRoll, ageDip), 0.0, in.dt);
					es.jx = vx * age;
					es.jy = vy * age;
				}

				// ---- tangential stress: shear displacement law, limited by the Coulomb / Mohr-Coulomb strength
				double tauMax, slope0, eta, cap;
				if (surf.deformable)
				{
					tauMax = surf.cohesion + sigma * std::tan(surf.phi);
					slope0 = tauMax / surf.Kx;
					eta = surf.Kx / surf.Ky;
					cap = 4.0 * surf.Kx;
				}
				else
				{
					const double sigmaF = P.sigmaFrictionCap * std::tanh(sigma / P.sigmaFrictionCap);
					const double sigmaSafe = std::max(sigmaF, 1.0e3);
					const double sens = std::clamp(std::pow(P.sigmaRef / sigmaSafe, P.loadSens), 0.8, 1.25);
					const double vs = std::sqrt(vx * vx + vy * vy);
					const double slide = 1.0 - P.slideDrop * (1.0 - std::exp(-vs / P.slideSpeed));
					const double muEff = std::clamp(double(mu) * P.gripScale * sens * slide, P.muMin, P.muMax);
					tauMax = muEff * sigmaF;
					slope0 = P.kbx;
					eta = P.kby / P.kbx;
					cap = 4.0 * tauMax / slope0;
				}
				es.eta = eta;
				es.jcap = cap;
				es.vx = vx;
				es.vy = vy;

				double Tx = 0.0, Ty = 0.0;
				if (tauMax > 1.0e-6 && slope0 > 0.0)
				{
					const double jeff = std::sqrt(es.jx * es.jx + eta * eta * es.jy * es.jy);
					if (jeff > cap)
					{
						const double k = cap / jeff;
						es.jx *= k;
						es.jy *= k;
					}
					const double jeff2 = std::min(jeff, cap);
					if (jeff2 > 1.0e-12)
					{
						const double tauS = tauMax * (1.0 - std::exp(-slope0 * jeff2 / tauMax));
						Tx = tauS * es.jx / jeff2;
						Ty = tauS * eta * es.jy / jeff2;
					}
					const double cd = slope0 * tauDampEff * dampScale;
					Tx += cd * vx;
					Ty += cd * eta * vy;
					const double mag = std::sqrt(Tx * Tx + Ty * Ty);
					if (mag > tauMax)
					{
						const double k = tauMax / mag;
						Tx *= k;
						Ty *= k;
					}
				}

				es.sigma = sigma;
				es.tau = std::sqrt(Tx * Tx + Ty * Ty);
				es.tauMax = tauMax;

				// ---- element force on the tire and moment about the wheel centre
				const double A = stripW * Rj * dAlpha * areaFactor;
				const double Fn = sigma * A;
				const Vec3 F{ Fn * nf.x - A * (Tx * xk.x + Ty * yk.x),
							  Fn * nf.y - A * (Tx * xk.y + Ty * yk.y),
							  Fn * nf.z - A * (Tx * xk.z + Ty * yk.z) };
				out.force.x += F.x;
				out.force.y += F.y;
				out.force.z += F.z;
				out.moment.x += Pt.y * F.z - Pt.z * F.y;
				out.moment.y += Pt.z * F.x - Pt.x * F.z;
				out.moment.z += Pt.x * F.y - Pt.y * F.x;

				out.maxPenetration = std::max(out.maxPenetration, deltaN);
				out.maxTireDeflection = std::max(out.maxTireDeflection, deltaN - zSoil);
				out.maxSinkage = std::max(out.maxSinkage, zSoil);
				++out.numContact;

				if (Fn > 0.0)
				{
					sumLoad += Fn;
					sumTangential += A * std::sqrt(Tx * Tx + Ty * Ty);
					if (!surf.deformable)
						sumHard += Fn;
					centroid.x += Fn * Pt.x;
					centroid.y += Fn * Pt.y;
					centroid.z += Fn * Pt.z;
					normalSum.x += Fn * nf.x;
					normalSum.y += Fn * nf.y;
					normalSum.z += Fn * nf.z;
					minX = std::min(minX, Pt.x);
					maxX = std::max(maxX, Pt.x);
				}
			}
		}

		//Force fuse, see TireParams::forceLimit
		const double fMag = std::sqrt(out.force.x * out.force.x + out.force.y * out.force.y + out.force.z * out.force.z);
		if (fMag > P.forceLimit)
		{
			const double k = P.forceLimit / fMag;
			out.force.x *= k;
			out.force.y *= k;
			out.force.z *= k;
			out.moment.x *= k;
			out.moment.y *= k;
			out.moment.z *= k;
		}

		lastMaxDefl = out.maxTireDeflection;
		out.load = sumLoad;
		out.loadHard = sumHard;
		out.inContact = sumLoad > 0.0;
		if (out.inContact)
		{
			const double inv = 1.0 / sumLoad;
			out.centroid = Vec3{ centroid.x * inv, centroid.y * inv, centroid.z * inv };
			const double nl = std::sqrt(normalSum.x * normalSum.x + normalSum.y * normalSum.y + normalSum.z * normalSum.z);
			out.normal = nl > 1.0e-12 ? Vec3{ normalSum.x / nl, normalSum.y / nl, normalSum.z / nl } : Vec3{ 0, 0, 1 };
			out.meanMu = sumTangential * inv;
			out.patchLength = std::max(0.0, maxX - minX);
			out.rollingRadius = P.radius - P.rollFactor * out.maxTireDeflection;

			//Rolling resistance of the hard contact (hysteresis of the carcass), opposes the wheel spin
			out.moment.y -= P.rollingResistance * sumHard * out.rollingRadius * std::tanh(in.omega.y / P.rollingOmega0);
		}
	}

	void Model::Advance(double dt)
	{
		for (ElementState& es : states)
		{
			if (!es.inContact)
				continue;
			es.jx += es.vx * dt;
			es.jy += es.vy * dt;
			const double jeff = std::sqrt(es.jx * es.jx + es.eta * es.eta * es.jy * es.jy);
			if (jeff > es.jcap && jeff > 0.0)
			{
				const double k = es.jcap / jeff;
				es.jx *= k;
				es.jy *= k;
			}
		}
	}
}
