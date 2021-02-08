#pragma once

#include <qwt/qwt_color_map.h>

#include <SimoxUtility/color/ColorMap.h>
#include <SimoxUtility/color/cmaps/Named.h>

namespace simox::qt
{
    class SimoxQwtColorMap: public QwtColorMap
    {
    public:
        SimoxQwtColorMap(const std::string& n = "viridis");
        SimoxQwtColorMap(const simox::color::ColorMap& cm);
        QRgb rgb(const QwtInterval& interval, double value) const override;
        unsigned char colorIndex(const QwtInterval& interval, double value) const override;
    private:
        ::simox::color::ColorMap cm;
    };
}
