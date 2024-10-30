#include "chrono_models/vehicle/kraz/Revoy_Driveline.h"

namespace chrono {
namespace vehicle {
namespace kraz {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double Revoy_Driveline::m_driveshaft_inertia = 0.5;
const double Revoy_Driveline::m_differentialbox_inertia = 0.6;

const double Revoy_Driveline::m_conicalgear_ratio = 1.0 / 4.3;

const double Revoy_Driveline::m_axle_differential_locking_limit = 100;

// -----------------------------------------------------------------------------
// Constructor of the Revoy_Driveline
// the direction of the motor block is along the X axis, while the directions of
// the axles is along the Y axis (relative to the chassis coordinate frame),
// -----------------------------------------------------------------------------
Revoy_Driveline::Revoy_Driveline(const std::string& name) : ChShaftsDriveline2WD(name) {
    SetMotorBlockDirection(ChVector3d(1, 0, 0));
    SetAxleDirection(ChVector3d(0, 1, 0));
}

}  // end namespace kraz
}  // end namespace vehicle
}  // end namespace chrono
