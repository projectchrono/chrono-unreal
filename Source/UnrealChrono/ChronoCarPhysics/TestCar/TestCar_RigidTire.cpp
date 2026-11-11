#include "TestCar_RigidTire.h"

using namespace chrono;
using namespace chrono::vehicle;

TestCar_RigidTire::TestCar_RigidTire(const std::string& name, const FChWheelSetup& SetupInit) : ChRigidTire(name),
    m_setup(SetupInit),
    m_radius(SetupInit.RigidRadius),
    m_width(SetupInit.RigidWidth),
    m_mass(SetupInit.RigidMass),
    m_inertia(SetupInit.RigidInertia[0], SetupInit.RigidInertia[1], SetupInit.RigidInertia[2])
{
}

void TestCar_RigidTire::CreateContactMaterial(ChContactMethod contact_method)
{
    ChContactMaterialData minfo;
    minfo.mu = (float)(m_setup.RigidFriction * m_setup.GripScale);
    minfo.cr = (float)m_setup.RigidRestitution;
    minfo.Y = (float)m_setup.RigidYoungModulus;
    m_material = minfo.CreateMaterial(contact_method);
}
