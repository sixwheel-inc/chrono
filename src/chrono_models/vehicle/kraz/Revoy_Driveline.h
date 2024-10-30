#ifndef REVOY_DRIVELINE_H
#define REVOY_DRIVELINE_H

#include "chrono_vehicle/wheeled_vehicle/driveline/ChShaftsDriveline2WD.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace kraz {

/// Shafts-based 2-WD driveline for the REvoy vehicle.
class CH_MODELS_API Revoy_Driveline : public ChShaftsDriveline2WD {
  public:
    Revoy_Driveline(const std::string& name);

    ~Revoy_Driveline() {}

    virtual double GetDriveshaftInertia() const override { return m_driveshaft_inertia; }
    virtual double GetDifferentialBoxInertia() const override { return m_differentialbox_inertia; }

    virtual double GetConicalGearRatio() const override { return m_conicalgear_ratio; }

    virtual double GetAxleDifferentialLockingLimit() const override { return m_axle_differential_locking_limit; }

  private:
    // Shaft inertias
    static const double m_driveshaft_inertia;
    static const double m_differentialbox_inertia;

    // Gear ratio
    static const double m_conicalgear_ratio;

    // Differential locking torque limit.
    static const double m_axle_differential_locking_limit;
};

}  // end namespace kraz
}  // end namespace vehicle
}  // end namespace chrono

#endif
