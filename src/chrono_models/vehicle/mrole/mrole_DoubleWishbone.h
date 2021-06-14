// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban, Justin Madsen, Daniel Melanz
// =============================================================================
//
// Front and Rear mrole suspension subsystems (double A-arm)
//
// These concrete suspension subsystems are defined with respect to right-handed
// frames with X pointing towards the front, Y to the left, and Z up (as imposed
// by the base class ChDoubleWishbone) and origins at the midpoint between the
// lower control arms' connection points to the chassis.
//
// All point locations are provided for the left half of the suspension.
//
// =============================================================================

#ifndef MROLE_DOUBLEWISHBONE_H
#define MROLE_DOUBLEWISHBONE_H

#include "chrono_vehicle/wheeled_vehicle/suspension/ChDoubleWishbone.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace mrole {

/// @addtogroup vehicle_models_hmmwv
/// @{

/// Full double wishbone front suspension for the mrole vehicle.
/// The control arms are modeled using rigid bodies.
class CH_MODELS_API mrole_DoubleWishboneFront : public ChDoubleWishbone {
  public:
    mrole_DoubleWishboneFront(const std::string& name);
    ~mrole_DoubleWishboneFront();

    virtual double getSpindleMass() const override { return m_spindleMass; }
    virtual double getUCAMass() const override { return m_UCAMass; }
    virtual double getLCAMass() const override { return m_LCAMass; }
    virtual double getUprightMass() const override { return m_uprightMass; }

    virtual double getSpindleRadius() const override { return m_spindleRadius; }
    virtual double getSpindleWidth() const override { return m_spindleWidth; }
    virtual double getUCARadius() const override { return m_UCARadius; }
    virtual double getLCARadius() const override { return m_LCARadius; }
    virtual double getUprightRadius() const override { return m_uprightRadius; }

    virtual const ChVector<>& getSpindleInertia() const override { return m_spindleInertia; }
    virtual const ChVector<>& getUCAInertiaMoments() const override { return m_UCAInertiaMoments; }
    virtual const ChVector<>& getUCAInertiaProducts() const override { return m_UCAInertiaProducts; }
    virtual const ChVector<>& getLCAInertiaMoments() const override { return m_LCAInertiaMoments; }
    virtual const ChVector<>& getLCAInertiaProducts() const override { return m_LCAInertiaProducts; }
    virtual const ChVector<>& getUprightInertiaMoments() const override { return m_uprightInertiaMoments; }
    virtual const ChVector<>& getUprightInertiaProducts() const override { return m_uprightInertiaProducts; }

    virtual double getAxleInertia() const override { return m_axleInertia; }

    virtual double getSpringRestLength() const override { return m_springRestLength; }
    virtual std::shared_ptr<ChLinkTSDA::ForceFunctor> getSpringForceFunctor() const override { return m_springForceCB; }
    virtual std::shared_ptr<ChLinkTSDA::ForceFunctor> getShockForceFunctor() const override { return m_shockForceCB; }

  private:
    virtual const ChVector<> getLocation(PointId which) override;

    std::shared_ptr<ChLinkTSDA::ForceFunctor> m_springForceCB;
    std::shared_ptr<ChLinkTSDA::ForceFunctor> m_shockForceCB;

    static const double m_spindleMass;
    static const double m_UCAMass;
    static const double m_LCAMass;
    static const double m_uprightMass;

    static const double m_spindleRadius;
    static const double m_spindleWidth;
    static const double m_UCARadius;
    static const double m_LCARadius;
    static const double m_uprightRadius;

    static const ChVector<> m_spindleInertia;
    static const ChVector<> m_UCAInertiaMoments;
    static const ChVector<> m_UCAInertiaProducts;
    static const ChVector<> m_LCAInertiaMoments;
    static const ChVector<> m_LCAInertiaProducts;
    static const ChVector<> m_uprightInertiaMoments;
    static const ChVector<> m_uprightInertiaProducts;

    static const double m_axleInertia;

    static const double m_springCoefficient;
    static const double m_springRestLength;
    static const double m_springMaxLength;
    static const double m_springMinLength;

    static const double m_damperCoefCompression;
    static const double m_damperDegresCompression;
    static const double m_damperDegresExpansion;
    static const double m_damperCoefExpansion;
};

// -----------------------------------------------------------------------------

/// Full double wishbone rear suspension for the mrole vehicle.
/// The control arms are modeled using rigid bodies.
class CH_MODELS_API mrole_DoubleWishboneRear : public ChDoubleWishbone {
  public:
    mrole_DoubleWishboneRear(const std::string& name);
    ~mrole_DoubleWishboneRear();

    virtual double getSpindleMass() const override { return m_spindleMass; }
    virtual double getUCAMass() const override { return m_UCAMass; }
    virtual double getLCAMass() const override { return m_LCAMass; }
    virtual double getUprightMass() const override { return m_uprightMass; }

    virtual double getSpindleRadius() const override { return m_spindleRadius; }
    virtual double getSpindleWidth() const override { return m_spindleWidth; }
    virtual double getUCARadius() const override { return m_UCARadius; }
    virtual double getLCARadius() const override { return m_LCARadius; }
    virtual double getUprightRadius() const override { return m_uprightRadius; }

    virtual const ChVector<>& getSpindleInertia() const override { return m_spindleInertia; }
    virtual const ChVector<>& getUCAInertiaMoments() const override { return m_UCAInertiaMoments; }
    virtual const ChVector<>& getUCAInertiaProducts() const override { return m_UCAInertiaProducts; }
    virtual const ChVector<>& getLCAInertiaMoments() const override { return m_LCAInertiaMoments; }
    virtual const ChVector<>& getLCAInertiaProducts() const override { return m_LCAInertiaProducts; }
    virtual const ChVector<>& getUprightInertiaMoments() const override { return m_uprightInertiaMoments; }
    virtual const ChVector<>& getUprightInertiaProducts() const override { return m_uprightInertiaProducts; }

    virtual double getAxleInertia() const override { return m_axleInertia; }

    virtual double getSpringRestLength() const override { return m_springRestLength; }
    virtual std::shared_ptr<ChLinkTSDA::ForceFunctor> getSpringForceFunctor() const override { return m_springForceCB; }
    virtual std::shared_ptr<ChLinkTSDA::ForceFunctor> getShockForceFunctor() const override { return m_shockForceCB; }

  private:
    virtual const ChVector<> getLocation(PointId which) override;

    std::shared_ptr<ChLinkTSDA::ForceFunctor> m_springForceCB;
    std::shared_ptr<ChLinkTSDA::ForceFunctor> m_shockForceCB;

    static const double m_spindleMass;
    static const double m_UCAMass;
    static const double m_LCAMass;
    static const double m_uprightMass;

    static const double m_spindleRadius;
    static const double m_spindleWidth;
    static const double m_UCARadius;
    static const double m_LCARadius;
    static const double m_uprightRadius;

    static const ChVector<> m_spindleInertia;
    static const ChVector<> m_UCAInertiaMoments;
    static const ChVector<> m_UCAInertiaProducts;
    static const ChVector<> m_LCAInertiaMoments;
    static const ChVector<> m_LCAInertiaProducts;
    static const ChVector<> m_uprightInertiaMoments;
    static const ChVector<> m_uprightInertiaProducts;

    static const double m_axleInertia;

    static const double m_springCoefficient;
    static const double m_springRestLength;
    static const double m_springMaxLength;
    static const double m_springMinLength;

    static const double m_damperCoefCompression;
    static const double m_damperDegresCompression;
    static const double m_damperDegresExpansion;
    static const double m_damperCoefExpansion;
};

/// @} vehicle_models_hmmwv

}  // namespace mrole
}  // end namespace vehicle
}  // end namespace chrono

#endif