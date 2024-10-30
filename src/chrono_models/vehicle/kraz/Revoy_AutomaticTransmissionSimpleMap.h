#ifndef REVOY_AUTOMATIC_TRANSMISSION_SIMPLE_MAP_H
#define REVOY_AUTOMATIC_TRANSMISSION_SIMPLE_MAP_H

#include "chrono_vehicle/powertrain/ChAutomaticTransmissionSimpleMap.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace kraz {

/// Simple Kraz tractor powertrain subsystem (based on engine speed-torque maps).
class CH_MODELS_API Revoy_AutomaticTransmissionSimpleMap : public ChAutomaticTransmissionSimpleMap {
  public:
    Revoy_AutomaticTransmissionSimpleMap(const std::string& name);

    /// Set the transmission gear ratios (one or more forward gear ratios and a single reverse gear ratio).
    virtual void SetGearRatios(std::vector<double>& fwd, double& rev) override;

    /// Set the ideal shift points for automatic gear shifting.
    /// For each forward gear, specify a pair (min, max) with the minimum and
    /// maximum engine speed for shifting (down and up, respectively).
    virtual void SetShiftPoints(
        std::vector<std::pair<double, double>>& shift_bands  ///< [out] down-shift/up-shift points
        ) override;
};

}  // end namespace kraz
}  // end namespace vehicle
}  // end namespace chrono

#endif
