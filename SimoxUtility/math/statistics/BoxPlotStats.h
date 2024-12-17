#pragma once

#include <vector>

#include "measures.h"

namespace simox::math
{

    /**
     * @brief Computes and stores statistical measures found in a box plot.
     */
    template <typename Float>
    class BoxPlotStats
    {
    public:
        BoxPlotStats() = default;
        BoxPlotStats(const std::vector<Float>& values, bool isSorted = false, Float whisk = 1.5);

        void set(const std::vector<Float>& values, bool isSorted = false);

        Float whisk = 1.5;

        Float minimum;
        Float minWhisker;
        Float lowerQuartile;
        Float median;
        Float upperQuartile;
        Float maxWhisker;
        Float maximum;

        std::vector<Float> outliers;
    };

    // Explicit template instantiation
    // template class BoxPlotStats<float>;
    // template class BoxPlotStats<double>;
    using BoxPlotStatsf = BoxPlotStats<float>;
    using BoxPlotStatsd = BoxPlotStats<double>;

    template <typename Float>
    BoxPlotStats<Float>::BoxPlotStats(const std::vector<Float>& values,
                                      bool isSorted,
                                      Float whisk) :
        whisk(whisk)
    {
        set(values, isSorted);
    }

    template <typename Float>
    void
    BoxPlotStats<Float>::set(const std::vector<Float>& _values, bool isSorted)
    {
        const std::vector<Float>& values = isSorted ? _values : math::sorted(_values);

        this->minimum = math::min(values, true);
        this->maximum = math::max(values, true);

        this->lowerQuartile = math::lower_quartile(values, true);
        this->median = math::median(values, true);
        this->upperQuartile = math::upper_quartile(values, true);

        const Float iqr = math::interquartile_range(lowerQuartile, upperQuartile);

        this->minWhisker = lowerQuartile - whisk * iqr;
        this->maxWhisker = upperQuartile + whisk * iqr;

        // Compute outliers and correct whiskers if necessary.
        {
            auto it = values.begin();
            for (; it != values.end() && *it < minWhisker; ++it)
            {
                outliers.push_back(*it);
            }
            minWhisker = (it != values.begin()) ? *it : minimum;
        }
        {
            auto rit = values.rbegin();
            for (; rit != values.rend() && *rit > maxWhisker; ++rit)
            {
                outliers.push_back(*rit);
            }
            maxWhisker = (rit != values.rbegin()) ? *rit : maximum;
        }
    }


} // namespace simox::math
