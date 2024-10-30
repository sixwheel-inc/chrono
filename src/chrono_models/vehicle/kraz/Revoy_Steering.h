#ifndef REVOY_STEERING_H
#define REVOY_STEERING_H

#include "chrono_vehicle/wheeled_vehicle/steering/ChRotaryArm.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace kraz {

/// RotaryArm steering subsystem for the Revoy.
class CH_MODELS_API Revoy_Steering : public ChRotaryArm {
  public:
    Revoy_Steering(const std::string& name);
    ~Revoy_Steering() {}

    virtual double getPitmanArmMass() const override { return m_pitmanArmMass; }

    virtual double getPitmanArmRadius() const override { return m_pitmanArmRadius; }

    virtual const ChVector3d& getPitmanArmInertiaMoments() const override { return m_pitmanArmInertiaMoments; }
    virtual const ChVector3d& getPitmanArmInertiaProducts() const override { return m_pitmanArmInertiaProducts; }

    virtual double getMaxAngle() const override { return m_maxAngle; }

    virtual const chrono::ChVector3d getLocation(PointId which) override;
    virtual const chrono::ChVector3d getDirection(DirectionId which) override;

  private:
    static const double m_pitmanArmMass;

    static const double m_pitmanArmRadius;

    static const double m_maxAngle;

    static const ChVector3d m_pitmanArmInertiaMoments;
    static const ChVector3d m_pitmanArmInertiaProducts;
};


}  // end namespace kraz
}  // end namespace vehicle
}  // end namespace chrono

#endif
