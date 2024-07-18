//
// Created by Rainer Gericke on 18.07.24.
//

#include "ChXYChartInfo.h"

namespace chrono {
namespace vsg3d {

ChXYChartInfo::ChXYChartInfo() {
    title = "Title";
    xlabel = "x";
    ylabel = "y";
}

ChXYChartInfo::~ChXYChartInfo() {}

bool ChXYChartInfo::DataComplete() {
    bool ret = false;
    // at least two data items must be set, bost data vectors must have the same size
    if ((xdata.size() > 1) && (xdata.size() == ydata.size()))
        ret = true;
    return ret;
}

}  // namespace vsg3d
}  // namespace chrono