#include "TestCar.h"

using namespace chrono;
using namespace chrono::vehicle;

void TestCar::Create()
{
    m_tire_collision_type = VehicleSetup->TireCollisionType;
    m_usetierod_bodies = false;
    SetChassisFixed(false);
    m_chassisCollisionType = PhysicsCollisionType::HULLS;

    m_chassis = chrono_types::make_shared<TestCarChassis>("TestCarChassis", m_fixed, m_chassisCollisionType, VehicleSetup->ChassisCollisionHulls);
    m_wheelbase = 2.885;
    m_tire_step_size = 0.003;
    // Discrete is the default; anything that is not RIGID or TMEASY falls back to it
    const ETireModelType TireModel = VehicleSetup->TireModel == ETireModelType::RIGID || VehicleSetup->TireModel == ETireModelType::TMEASY
        ? VehicleSetup->TireModel : ETireModelType::DISCRETE;
    SetTireType(TireModel);
    m_area = 3.64;
    m_air_density = AirDensity;
    m_Cd = 0.35;
    m_driveType = DrivelineTypeWV::AWD;
    m_powertrainType = PowertrainModelType::SIMPLE_MAP;

    SetContactMethod(ChContactMethod::NSC);
    // Collision backend (Bullet / Chaos) is configured on the hosted system by FChronoVehicleWorld::Build
    SetSteeringType(SteeringTypeWV::RACK_PINION);
    SetBrakeType(m_brake_type);
    SetDriveType(m_driveType);
    SetChassisCollisionType(m_chassisCollisionType);

    const FChWheelSetup& Wheels = VehicleSetup->Wheels;

    m_axles.resize(2);
    m_axles[0] = chrono_types::make_shared<ChAxle>();
    m_axles[1] = chrono_types::make_shared<ChAxle>();
    m_axles[0]->m_suspension = chrono_types::make_shared<TestCar_DoubleWishbone>("FrontSusp", VehicleSetup->FrontSuspensionLocations, VehicleSetup->Front);
    m_axles[1]->m_suspension = chrono_types::make_shared<TestCar_DoubleWishbone>("RearSusp", VehicleSetup->RearSuspensionLocations, VehicleSetup->Rear);

    m_axles[0]->m_wheels.resize(2);
    m_axles[1]->m_wheels.resize(2);
    m_axles[0]->m_wheels[0] = chrono_types::make_shared<TestCar_Wheel>("Wheel_FL", Wheels);
    m_axles[0]->m_wheels[1] = chrono_types::make_shared<TestCar_Wheel>("Wheel_FR", Wheels);
    m_axles[1]->m_wheels[0] = chrono_types::make_shared<TestCar_Wheel>("Wheel_RL", Wheels);
    m_axles[1]->m_wheels[1] = chrono_types::make_shared<TestCar_Wheel>("Wheel_RR", Wheels);

    m_axles[0]->m_brake_left = chrono_types::make_shared<TestCar_BrakeShafts>("Brake_FL");
    m_axles[0]->m_brake_right = chrono_types::make_shared<TestCar_BrakeShafts>("Brake_FR");
    m_axles[1]->m_brake_left = chrono_types::make_shared<TestCar_BrakeShafts>("Brake_RL");
    m_axles[1]->m_brake_right = chrono_types::make_shared<TestCar_BrakeShafts>("Brake_RR");

    m_max_steering_angle = 35.0;
    m_steerings.resize(1);
    m_steerings[0] = chrono_types::make_shared<TestCar_RackAndPinion>("Steering");

    m_driveline = chrono_types::make_shared<TestCar_Driveline4WD>("Driveline");

    const auto MakeTire = [&](const char* Name) -> std::shared_ptr<ChTire>
    {
        switch (TireModel)
        {
        case ETireModelType::RIGID: return chrono_types::make_shared<TestCar_RigidTire>(Name, Wheels);
        case ETireModelType::TMEASY: return chrono_types::make_shared<TestCar_TMeasyTire>(Name, Wheels);
        default: return chrono_types::make_shared<TestCar_DiscreteTire>(Name, Wheels);
        }
    };
    FL_Tire = MakeTire("FL");
    FR_Tire = MakeTire("FR");
    RL_Tire = MakeTire("RL");
    RR_Tire = MakeTire("RR");

    FR_Wheel = GetAxle(0)->m_wheels[RIGHT];
    FL_Wheel = GetAxle(0)->m_wheels[LEFT];
    RL_Wheel = GetAxle(1)->m_wheels[LEFT];
    RR_Wheel = GetAxle(1)->m_wheels[RIGHT];

    m_tire_mass = FL_Tire->GetMass();
}

void TestCar::Initialize(const chrono::ChCoordsys<>& WorldStartPos, double chassisFwdVel)
{
    SetInitPosition(WorldStartPos);
    m_chassis->Initialize(this, WorldStartPos, chassisFwdVel, VehicleCollisionFamily::CHASSIS_FAMILY);
    m_chassis->EnableCollision(m_chassisCollisionType != PhysicsCollisionType::NONE);

    m_chassis_body = m_chassis->GetBody();
    if (auto collision_model = m_chassis_body->GetCollisionModel())
    {
        collision_model->SetFamilyGroup(VEHICLE_GROUP);
        collision_model->SetFamilyMask(VEHICLE_MASK);
    }

    m_steerings[0]->Initialize(m_chassis, ChVector3<>(1.24498, 0, 0.101322), ChQuaternion<>(1, 0, 0, 0));

    m_axles[0]->Initialize(m_chassis, nullptr, m_steerings[0], Vector(1.535, 0, 0.38), ChVector3<>(0), 0.0, m_omega[0], m_omega[1]);   // front
    m_axles[1]->Initialize(m_chassis, nullptr, nullptr, Vector(-1.35, 0, 0.407), ChVector3<>(0), 0.0, m_omega[2], m_omega[3]);        // rear

    std::vector<int> driven_susp_indexes(m_driveline->GetNumDrivenAxles());
    driven_susp_indexes[0] = 0;
    driven_susp_indexes[1] = 1;
    m_driveline->Initialize(m_chassis, m_axles, driven_susp_indexes);

    SetChassisVisualizationType(VisualizationType::NONE);

    if (m_apply_drag)
    {
        GetChassis()->SetAerodynamicDrag(m_Cd, m_area, m_air_density);
    }

    std::shared_ptr<ChEngine> engine = chrono_types::make_shared<TestCar_EngineMap>("Engine", VehicleSetup.get());
    std::shared_ptr<ChTransmission> transmission = chrono_types::make_shared<TestCar_MapTransmission>("Powertrain");
    InitializePowertrain(chrono_types::make_shared<ChPowertrainAssembly>(engine, transmission));

    static_cast<TestCarChassis*>(m_chassis.get())->FrontDriverWheel = FL_Wheel;
    static_cast<TestCarChassis*>(m_chassis.get())->FrontPassengerWheel = FR_Wheel;
    static_cast<TestCarChassis*>(m_chassis.get())->RearDriverWheel = RL_Wheel;
    static_cast<TestCarChassis*>(m_chassis.get())->RearPassengerWheel = RR_Wheel;

    InitializeTire(FL_Tire, GetAxle(0)->m_wheels[LEFT], VisualizationType::NONE);
    InitializeTire(FR_Tire, GetAxle(0)->m_wheels[RIGHT], VisualizationType::NONE);
    InitializeTire(RL_Tire, GetAxle(1)->m_wheels[LEFT], VisualizationType::NONE);
    InitializeTire(RR_Tire, GetAxle(1)->m_wheels[RIGHT], VisualizationType::NONE);

    for (auto& axle : GetAxles())
    {
        for (auto& wheel : axle->GetWheels())
        {
            if (m_tire_step_size > 0)
            {
                wheel->GetTire()->SetStepsize(m_tire_step_size);
            }
        }
    }
    EnableBrakeLocking(m_brake_locking);

    InitializeInertiaProperties();
}

ChVehicleData* TestCar::UpdateVehicleData()
{
    m_VehicleData.RPM = (float)GetEngine()->GetMotorSpeed();
    m_VehicleData.Torque = (float)GetEngine()->GetOutputMotorshaftTorque();
    // Chrono numbers reverse -1 and neutral 0; the vehicle data contract is "0 == reverse, 1+ == forward"
    m_VehicleData.CurrentGear = std::max(GetTransmission()->GetCurrentGear(), 0);
    m_VehicleData.BrakeInput = (float)m_axles[0]->GetBrake(LEFT)->GetBrakeTorque();
    m_VehicleData.Speed = (float)GetSpeed();
    m_VehicleData.Load = (float)GetEngine()->GetOutputMotorshaftTorque();

    const std::shared_ptr<ChWheel> W[4] = { FL_Wheel, FR_Wheel, RL_Wheel, RR_Wheel };
    float* Slip[4] = { &m_VehicleData.FL_SlipAngle, &m_VehicleData.FR_SlipAngle, &m_VehicleData.RL_SlipAngle, &m_VehicleData.RR_SlipAngle };
    float* Long[4] = { &m_VehicleData.FL_LongitudinalSlip, &m_VehicleData.FR_LongitudinalSlip, &m_VehicleData.RL_LongitudinalSlip, &m_VehicleData.RR_LongitudinalSlip };
    float* Camber[4] = { &m_VehicleData.FL_CamberAngle, &m_VehicleData.FR_CamberAngle, &m_VehicleData.RL_CamberAngle, &m_VehicleData.RR_CamberAngle };
    float* Defl[4] = { &m_VehicleData.FL_TireDeflection, &m_VehicleData.FR_TireDeflection, &m_VehicleData.RL_TireDeflection, &m_VehicleData.RR_TireDeflection };
    chrono::ChVector3f* Contact[4] = { &m_VehicleData.FL_ContactPoint, &m_VehicleData.FR_ContactPoint, &m_VehicleData.RL_ContactPoint, &m_VehicleData.RR_ContactPoint };
    for (int i = 0; i < 4; ++i)
    {
        const std::shared_ptr<ChTire> Tire = W[i]->GetTire();
        *Slip[i] = (float)Tire->GetSlipAngle();
        *Long[i] = (float)Tire->GetLongitudinalSlip();
        *Camber[i] = (float)Tire->GetCamberAngle();
        *Defl[i] = (float)Tire->GetDeflection();
        if (auto ForceTire = std::dynamic_pointer_cast<chrono::vehicle::ChForceElementTire>(Tire))
            *Contact[i] = ForceTire->ReportTireContactData().frame.pos;
    }

    return &m_VehicleData;
}
