#ifndef REVOY_ENGINE_SIMPLE_MAP_H
#define REVOY_ENGINE_SIMPLE_MAP_H

#include "chrono_vehicle/powertrain/ChEngineSimpleMap.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace kraz {

/// Simple Revoy EngineSimpleMap subsystem. EV motor
class CH_MODELS_API Revoy_EngineSimpleMap : public ChEngineSimpleMap {
  public:
    Revoy_EngineSimpleMap(const std::string& name);

    /// Specify maximum engine speed.
    virtual double GetMaxEngineSpeed() override;

    /// Set the engine speed-torque maps.
    /// A concrete class must add the speed-torque points to the provided maps,
    /// using the ChFunctionInterp::AddPoint() function.
    virtual void SetEngineTorqueMaps(ChFunctionInterp& map0,  ///< [out] engine map at zero throttle
                                     ChFunctionInterp& mapF   ///< [out] engine map at full throttle
                                     ) override;
};


}  // end namespace kraz
}  // end namespace vehicle
}  // end namespace chrono

#endif
