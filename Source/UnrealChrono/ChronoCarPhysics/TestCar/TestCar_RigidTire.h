#pragma once
#include "ChronoTypes.h"
#include "ChronoWheelSetup.h"
#include <Common.h>

// Rigid cylinder tire: collides through its collision shape (a Bullet body, or on the Chaos backend a proxy in the Unreal
// scene). Radius, width, mass and contact material come from the FChWheelSetup.
class TestCar_RigidTire : public chrono::vehicle::ChRigidTire
{
public:
    TestCar_RigidTire(const std::string& name, const FChWheelSetup& SetupInit);
    ~TestCar_RigidTire() {}

    virtual double GetRadius() const override { return m_radius; }
    virtual double GetWidth() const override { return m_width; }
    virtual double GetTireMass() const override { return m_mass; }
    virtual chrono::ChVector3d GetTireInertia() const override { return m_inertia; }

protected:
    virtual void CreateContactMaterial(chrono::ChContactMethod contact_method) override;

private:
    FChWheelSetup m_setup;
    double m_radius;
    double m_width;
    double m_mass;
    chrono::ChVector3d m_inertia;
};
