#include "chrono_models/vehicle/kraz/Revoy_AutomaticTransmissionSimpleMap.h"

namespace chrono {
namespace vehicle {
namespace kraz {

const double rpm2rads = CH_PI / 30;

Revoy_AutomaticTransmissionSimpleMap::Revoy_AutomaticTransmissionSimpleMap(const std::string& name)
    : ChAutomaticTransmissionSimpleMap(name) {}

void Revoy_AutomaticTransmissionSimpleMap::SetGearRatios(std::vector<double>& fwd, double& rev) {
    rev = -0.162337662;

    fwd.push_back(0.162337662);
    fwd.push_back(0.220750552);
    fwd.push_back(0.283286119);
    fwd.push_back(0.414937759);
    fwd.push_back(0.571428571);
    fwd.push_back(0.78125);
    fwd.push_back(1.0);
}

void Revoy_AutomaticTransmissionSimpleMap::SetShiftPoints(std::vector<std::pair<double, double>>& shift_bands) {
    shift_bands.push_back(std::pair<double, double>(1000 * rpm2rads, 2226 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(1000 * rpm2rads, 2226 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(1000 * rpm2rads, 2225 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(1000 * rpm2rads, 2210 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(1000 * rpm2rads, 2226 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(1000 * rpm2rads, 2225 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(1000 * rpm2rads, 2700 * rpm2rads));
}

}  // end namespace kraz
}  // end namespace vehicle
}  // end namespace chrono
