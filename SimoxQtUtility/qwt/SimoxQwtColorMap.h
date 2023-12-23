#pragma once

#include <SimoxUtility/color/ColorMap.h>
#include <SimoxUtility/color/cmaps/Named.h>

#include <qwt_color_map.h>

namespace simox::qt
{
    class SimoxQwtColorMap : public QwtColorMap
    {
    public:
        SimoxQwtColorMap(const std::string& n = "viridis");
        SimoxQwtColorMap(const simox::color::ColorMap& cm);
        QRgb rgb(const QwtInterval& interval, double value) const override;
#if QWT_VERSION > 0x060200
        uint colorIndex(int numColors, const QwtInterval& interval, double value) const override;
#else
        unsigned char colorIndex(const QwtInterval& interval, double value) const override;
#endif
    private:
        ::simox::color::ColorMap cm;
    };
} // namespace simox::qt
