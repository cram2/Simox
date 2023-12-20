#include "SimoxQwtColorMap.h"
#include <qwt_interval.h>

namespace simox::qt
{
    SimoxQwtColorMap::SimoxQwtColorMap(const std::string& n):
        SimoxQwtColorMap(simox::color::cmaps::Named::get(n))
    {
    }

    SimoxQwtColorMap::SimoxQwtColorMap(const color::ColorMap& cm):
        cm{cm}
    {
    }

    QRgb SimoxQwtColorMap::rgb(const QwtInterval& interval, double value) const
    {
        const auto c = cm.at(value,
                             interval.minValue(),
                             interval.maxValue());
        return QColor{c.r, c.g, c.b}.rgb();
    }

    uint SimoxQwtColorMap::colorIndex(int numColors,
                                 const QwtInterval& interval, double value) const
    {
        return 0;
    }

}
