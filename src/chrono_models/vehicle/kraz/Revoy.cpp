#include "chrono_models/vehicle/kraz/Kraz_tractor_Brake.h"
#include "chrono_models/vehicle/kraz/Kraz_tractor_Wheel.h"
#include "chrono_models/vehicle/kraz/Kraz_tractor_FrontSuspension.h"
#include "chrono_models/vehicle/kraz/Revoy_Chassis.h"
#include "chrono_models/vehicle/kraz/Revoy_Steering.h"
#include "chrono_models/vehicle/kraz/Revoy_EngineSimpleMap.h"
#include "chrono_models/vehicle/kraz/Revoy_AutomaticTransmissionSimpleMap.h"
#include "chrono_models/vehicle/kraz/Revoy_Driveline.h"
#include "chrono_models/vehicle/kraz/Revoy.h"

namespace chrono {
namespace vehicle {
namespace kraz {

// -----------------------------------------------------------------------------
Revoy::Revoy(bool fixed, CollisionType chassis_collision_type, ChContactMethod contactMethod)
    : ChWheeledVehicle("Revoy", contactMethod) {
    Create(fixed, chassis_collision_type);
}

Revoy::Revoy(ChSystem* system, bool fixed, CollisionType chassis_collision_type)
    : ChWheeledVehicle("Revoy", system) {
    Create(fixed, chassis_collision_type);
}

void Revoy::Create(bool fixed, CollisionType chassis_collision_type) {
    // Create the chassis subsystem
    m_chassis_as_rear = chrono_types::make_shared<Revoy_Chassis>("Chassis", fixed, chassis_collision_type);
    m_chassis = m_chassis_as_rear;
    m_connector = chrono_types::make_shared<Revoy_Connector>("Connector");

    // Create the axle subsystems
    m_axles.resize(1);
    m_axles[0] = chrono_types::make_shared<ChAxle>();

    m_axles[0]->m_suspension = chrono_types::make_shared<Kraz_tractor_FrontSuspension>("Suspension");

    m_axles[0]->m_wheels.resize(4);
    m_axles[0]->m_wheels[0] = chrono_types::make_shared<Kraz_tractor_Wheel>("Wheel_Li");
    m_axles[0]->m_wheels[1] = chrono_types::make_shared<Kraz_tractor_Wheel>("Wheel_Lo");
    m_axles[0]->m_wheels[2] = chrono_types::make_shared<Kraz_tractor_Wheel>("Wheel_Ri");
    m_axles[0]->m_wheels[3] = chrono_types::make_shared<Kraz_tractor_Wheel>("Wheel_Ro");

    m_axles[0]->m_brake_left = chrono_types::make_shared<Kraz_tractor_Brake>("Brake_L");
    m_axles[0]->m_brake_right = chrono_types::make_shared<Kraz_tractor_Brake>("Brake_R");

    // Create the steering subsystem
    m_steerings.resize(1);
    m_steerings[0] = chrono_types::make_shared<Revoy_Steering>("Revoy_Steering");

    // Create the driveline
    m_driveline = chrono_types::make_shared<Revoy_Driveline>("driveline");
}

// -----------------------------------------------------------------------------
void Revoy::Initialize(std::shared_ptr<ChChassis> frontChassis,
                       const ChCoordsys<>& chassisPos,
                       double chassisFwdVel) {
    // Initialize the chassis subsystem.
    m_chassis_as_rear->Initialize(frontChassis, WheeledCollisionFamily::CHASSIS);

    // Initialize the connector
    m_connector->Initialize(frontChassis, m_chassis_as_rear);

    // Initialize the steering subsystem (specify the steering subsystem's frame relative to the chassis reference
    // frame).
    m_steerings[0]->Initialize(m_chassis, ChVector3d(0, 0, 0), ChQuaternion<>(1, 0, 0, 0));

    // Initialize the axle subsystem.
    const double twin_tire_dist = 0.33528;  // Michelin for 12.00 R 20
    m_axles[0]->Initialize(m_chassis, nullptr, m_steerings[0], ChVector3d(-6, 0, 0), ChVector3d(0), twin_tire_dist);

    // Initialize the driveline subsystem (6x4 = rear axles are driven)
    std::vector<int> driven_susp = {0};
    m_driveline->Initialize(m_chassis, m_axles, driven_susp);
    m_driveline->LockAxleDifferential(0, false);
    
    std::shared_ptr<ChEngine> engine = chrono_types::make_shared<Revoy_EngineSimpleMap>("Revoy_Engine");
    std::shared_ptr<ChTransmission> transmission = chrono_types::make_shared<Revoy_AutomaticTransmissionSimpleMap>("Revoy_Transmission");

    auto powertrain = chrono_types::make_shared<ChPowertrainAssembly>(engine, transmission);
    this->InitializePowertrain(powertrain);

    // Invoke base class method
    ChWheeledVehicle::Initialize(chassisPos, chassisFwdVel);
}

void Revoy::DebugLog(int what) {}

}  // end namespace kraz
}  // end namespace vehicle
}  // end namespace chrono
