#include "chrono_models/vehicle/kraz/Revoy_Steering.h"

namespace chrono {
namespace vehicle {
namespace kraz {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

using namespace chrono;
using namespace chrono::vehicle;

const double Revoy_Steering::m_pitmanArmMass = 1.605;

const double Revoy_Steering::m_pitmanArmRadius = 0.02;

const double Revoy_Steering::m_maxAngle = 22.7 * (CH_PI / 180);

const ChVector3d Revoy_Steering::m_pitmanArmInertiaMoments(0.00638*10, 0.00756*10, 0.00150*10);
const ChVector3d Revoy_Steering::m_pitmanArmInertiaProducts(0.0, 0.0, 0.0);

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
Revoy_Steering::Revoy_Steering(const std::string& name) : ChRotaryArm(name) {}

// -----------------------------------------------------------------------------
// Implementations of the getLocation() and getDirection() virtual methods.
// -----------------------------------------------------------------------------
const ChVector3d Revoy_Steering::getLocation(PointId which) {
    const double ofs = 0.081;
    switch (which) {
        case ARM_L:
            return ChVector3d(1.0, 0.708341392 - ofs, 0.1);
        case ARM_C:
            return ChVector3d(1.0, 0.708341392 - ofs, 0.3);
        default:
            return ChVector3d(0, 0, 0);
    }
}

const ChVector3d Revoy_Steering::getDirection(DirectionId which) {
    switch (which) {
        case REV_AXIS:
            return ChVector3d(0, 1, 0);
        default:
            return ChVector3d(0, 1, 0);
    }
}

}  // end namespace kraz
}  // end namespace vehicle
}  // end namespace chrono
