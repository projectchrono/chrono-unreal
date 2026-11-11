#pragma once
#include "ChronoTypes.h"

#include "ChronoIncludeBegin.h"
#include "chrono_vehicle/wheeled_vehicle/suspension/ChDoubleWishboneReduced.h"
#include "ChronoIncludeEnd.h"

#include <Common.h>
#include "ChronoVehicle.h"

// Reduced double wishbone suspension (one wishbone per side, shock + tierod). The hardpoints come from the vehicle
// setup; the same class is used on both axles with different parameters.
class TestCar_DoubleWishbone : public chrono::vehicle::ChDoubleWishboneReduced
{
public:
	TestCar_DoubleWishbone(const std::string& NameInit, const std::map<int, Vector>& HardpointsIn, const FDoubleWishboneParams& ParamsIn);
	~TestCar_DoubleWishbone() {}

	virtual double getSpindleRadius() const override { return 0.15; }
	virtual double getSpindleWidth() const override { return 0.06; }

protected:
	virtual double getSpindleMass() const override { return Params.SpindleMass; }
	virtual double getUprightMass() const override { return Params.UprightMass; }
	virtual double getUprightRadius() const override { return 0.02; }

	virtual const ChVector3d getLocation(PointId WhichPoint) override;
	virtual const ChVector3d& getSpindleInertia() const override { return Params.SpindleInertia; }
	virtual const ChVector3d& getUprightInertia() const override { return Params.UprightInertia; }

	virtual double getAxleInertia() const override { return Params.AxleInertia; }
	virtual double getSpringRestLength() const override { return Params.SpringRestLength; }

	virtual std::shared_ptr<chrono::ChLinkTSDA::ForceFunctor> getShockForceFunctor() const override { return ShockForce; }

private:
	std::shared_ptr<chrono::ChLinkTSDA::ForceFunctor> ShockForce;
	std::map<int, Vector> Hardpoints;
	FDoubleWishboneParams Params;
};
