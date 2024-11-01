#ifndef REVOY_KRAZ_H
#define REVOY_KRAZ_H

#include "chrono_models/ChApiModels.h"

#include "chrono_models/vehicle/kraz/Revoy.h"
#include "chrono_models/vehicle/kraz/Kraz_tractor.h"
#include "chrono_models/vehicle/kraz/Kraz_tractor_EngineSimpleMap.h"
#include "chrono_models/vehicle/kraz/Kraz_tractor_AutomaticTransmissionSimpleMap.h"
#include "chrono_models/vehicle/kraz/Kraz_tractor_Tire.h"
#include "chrono_models/vehicle/kraz/Kraz_trailer.h"
#include "chrono_models/vehicle/kraz/Kraz_trailer_Tire.h"

namespace chrono {
namespace vehicle {
namespace kraz {

/// @addtogroup vehicle_models_kraz
/// @{

/// Definition of the Kraz 64431 semi-trailer truck assembly.
/// This class encapsulates a concrete wheeled vehicle model for the tractor, the powertrain and tires for the tractor,
/// as well as the trailer and its tires.
class CH_MODELS_API RevoyKraz {
  public:
    RevoyKraz();
    RevoyKraz(ChSystem* system);
    ~RevoyKraz();

    void SetContactMethod(ChContactMethod val) { m_contactMethod = val; }
    void SetCollisionSystemType(ChCollisionSystem::Type collsys_type) { m_collsysType = collsys_type; }

    void SetChassisFixed(bool val) { m_fixed = val; }
    void SetChassisCollisionType(CollisionType val) { m_chassisCollisionType = val; }

    void SetInitPosition(const ChCoordsys<>& pos) { m_initPos = pos; }
    void SetInitFwdVel(double fwdVel) { m_initFwdVel = fwdVel; }
    void SetInitWheelAngVel(const std::vector<double>& omega) { m_initOmega = omega; }

    void SetTireStepSize(double step_size) { m_tire_step_size = step_size; }

    ChSystem* GetSystem() const { return m_tractor->GetSystem(); }
    ChWheeledVehicle& GetTractor() const { return *m_tractor; }
    std::shared_ptr<ChChassis> GetTractorChassis() const { return m_tractor->GetChassis(); }
    std::shared_ptr<ChBodyAuxRef> GetTractorChassisBody() const { return m_tractor->GetChassisBody(); }
    ChWheeledTrailer& GetTrailer() const { return *m_trailer; }
    ChWheeledVehicle& GetRevoy() const { return *m_revoy; }

    void Initialize();

    void SetChassisVisualizationType(VisualizationType vis_tractor, VisualizationType vis_revoy, VisualizationType vis_trailer);
    void SetSuspensionVisualizationType(VisualizationType vis_tractor, VisualizationType vis_revoy, VisualizationType vis_trailer);
    void SetSteeringVisualizationType(VisualizationType vis);
    void SetWheelVisualizationType(VisualizationType vis_tractor, VisualizationType vis_revoy, VisualizationType vis_trailer);
    void SetTireVisualizationType(VisualizationType vis_tractor, VisualizationType vis_revoy, VisualizationType vis_trailer);

    void Synchronize(double time, const DriverInputs& driver_inputs, const ChTerrain& terrain);

    /// Revoy Input
    /// will use this when co-simulating w/ Revoy controls
    // void Synchronize(
    //         double time, 
    //         const DriverInputs& driver_inputs, 
    //         const RevoyInputs& revoyInputs, const ChTerrain& terrain);
    
    void Advance(double step);

  private:
    ChContactMethod m_contactMethod;
    ChCollisionSystem::Type m_collsysType;
    CollisionType m_chassisCollisionType;
    bool m_fixed;

    double m_tire_step_size;

    ChCoordsys<> m_initPos;
    double m_initFwdVel;
    std::vector<double> m_initOmega;

    EngineModelType m_engineType;
    TransmissionModelType m_transmissionType;

    ChSystem* m_system;
    Kraz_tractor* m_tractor;
    Revoy* m_revoy;
    Kraz_trailer* m_trailer;
};

/// @} vehicle_models_kraz

}  // end namespace kraz
}  // end namespace vehicle
}  // end namespace chrono

#endif
