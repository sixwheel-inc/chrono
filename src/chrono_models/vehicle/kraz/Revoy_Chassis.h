#ifndef REVOY_CHASSIS_H
#define REVOY_CHASSIS_H

#include "chrono_vehicle/chassis/ChRigidChassis.h"
#include "chrono_vehicle/chassis/ChChassisConnectorFifthWheel.h"

#include "chrono_models/ChApiModels.h"
#include "chrono_models/vehicle/ChVehicleModelDefs.h"

#include <iostream>

namespace chrono {
namespace vehicle {
namespace kraz {

/// Revoy chassis subsystem.
class CH_MODELS_API Revoy_Chassis : public ChRigidChassisRear {
  public:
    Revoy_Chassis(const std::string& name,
                  bool fixed = false,
                  CollisionType chassis_collision_type = CollisionType::NONE);
    ~Revoy_Chassis() {}

    /// Get the location (in the local frame of this chassis) of the connection to the front chassis.
    virtual const ChVector3d& GetLocalPosFrontConnector() const override { return m_front_connector_loc; }
    /// Get the location (in the local frame of this chassis) of the connection to the rear chassis.
    virtual const ChVector3d GetLocalPosRearConnector() const override { return m_rear_connector_loc; }

    /// Get the local driver position and orientation.
    /// This is a coordinate system relative to the chassis reference frame.
    //virtual ChCoordsys<> GetLocalDriverCoordsys() const override { return m_driverCsys; }

  protected:
    virtual double GetBodyMass() const override { return m_body_mass; }
    virtual ChMatrix33<> GetBodyInertia() const override { return m_body_inertia; }
    virtual ChFrame<> GetBodyCOMFrame() const override { return ChFrame<>(m_body_COM_loc, QUNIT); }

    ChMatrix33<> m_body_inertia;

    static const double m_body_mass;
    static const ChVector3d m_body_inertiaXX;
    static const ChVector3d m_body_inertiaXY;
    static const ChVector3d m_body_COM_loc;
    static const ChVector3d m_front_connector_loc;
    static const ChVector3d m_rear_connector_loc;
    static const ChCoordsys<> m_driverCsys;
};

class CH_MODELS_API Revoy_Connector : public ChChassisConnectorFifthWheel {
  public:
    Revoy_Connector(const std::string& name) : ChChassisConnectorFifthWheel(name) {}
    void GetHitchForce() {
      std::cout << "x: " << m_joint->GetReaction2().force.x() << ", y: " << m_joint->GetReaction2().force.y() << ", z: " << m_joint->GetReaction2().force.z() << std::endl;
    }
    ~Revoy_Connector() {}
};

}  // end namespace kraz
}  // end namespace vehicle
}  // end namespace chrono

#endif
