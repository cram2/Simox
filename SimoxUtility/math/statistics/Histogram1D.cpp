#include "Histogram1D.h"

#include <algorithm>

namespace simox::math
{


    Histogram1D::Histogram1D()
    {
    }

    Histogram1D::Histogram1D(const std::vector<float>& data, std::size_t numBins)
    {
        setMinMax(data);
        resetBins(numBins);
        insert(data);
    }

    Histogram1D::Histogram1D(const std::vector<float>& data,
                             float min,
                             float max,
                             std::size_t numBins)
    {
        setMinMax(min, max);
        resetBins(numBins);
        insert(data);
    }

    void
    Histogram1D::resetBins(std::size_t numBins)
    {
        bins.assign(numBins, 0);
    }

    std::size_t
    Histogram1D::valueToIndex(float value) const
    {
        // normalized = 0..1
        float normalized = (value - min) / (max - min);
        std::size_t index = static_cast<std::size_t>(normalized * getNumberOfBins());

        // avoid out-of-bounce errors
        // (if Histogram1D is falsely used, this could lead to peaks at the edges)
        return std::max(std::size_t(0), std::min(getNumberOfBins() - 1, index));
    }

    float
    Histogram1D::indexToValue(std::size_t index) const
    {
        float normalized = static_cast<float>(index) / getNumberOfBins();
        float value = normalized * (max - min) + min;
        return value;
    }

    void
    Histogram1D::insert(float value)
    {
        bins[valueToIndex(value)]++;
    }

    void
    Histogram1D::insert(const std::vector<float>& values)
    {
        for (float v : values)
        {
            insert(v);
        }
    }

    void
    Histogram1D::setMinMax(float min, float max)
    {
        this->min = min;
        this->max = max;
    }

    void
    Histogram1D::setMinMax(const std::vector<float>& data)
    {
        float min = data.front();
        float max = data.front();

        for (float d : data)
        {
            min = std::min(min, d);
            max = std::max(max, d);
        }
        setMinMax(min, max);
    }

    const std::vector<std::size_t>&
    Histogram1D::getBins() const
    {
        return bins;
    }

    std::size_t
    Histogram1D::getNumberOfBins() const
    {
        return bins.size();
    }

    float
    Histogram1D::getMin() const
    {
        return min;
    }

    float
    Histogram1D::getMax() const
    {
        return max;
    }

    std::size_t
    Histogram1D::getMinBinIndex() const
    {
        return static_cast<std::size_t>(
            std::distance(bins.begin(), std::min_element(bins.begin(), bins.end())));
    }

    float
    Histogram1D::getMinBinValue() const
    {
        return indexToValue(getMaxBinIndex());
    }

    std::size_t
    Histogram1D::getMaxBinIndex() const
    {
        return static_cast<std::size_t>(
            std::distance(bins.begin(), std::max_element(bins.begin(), bins.end())));
    }

    float
    Histogram1D::getMaxBinValue() const
    {
        return indexToValue(getMaxBinIndex());
    }

    void
    Histogram1D::applyMedianFilter(std::size_t size)
    {
        std::vector<std::size_t> newBins(bins.size());
        std::vector<std::size_t> neighborhood(2 * size + 1);

        for (std::size_t index = 0; index < bins.size(); index++)
        {
            // handle cases at borders
            std::size_t beginIndex = std::max(index - size, std::size_t(0));
            std::size_t endIndex = std::min(index + size + 1, bins.size());
            std::size_t num = endIndex - beginIndex; // common case: num == 2*size+1

            // example: index = 3, size = 2
            // => beginIndex = 3-2 = 1, endIndex = 3+2+1 = 6, num = 6-1 = 5
            // => neighborhood = [1, 6) == [1,5]

            neighborhood.assign(bins.begin() + beginIndex, bins.begin() + endIndex);

            std::sort(neighborhood.begin(), neighborhood.begin() + num);

            newBins[index] = neighborhood[num / 2]; // == median
            // (ignore cases where num is even, this only happens at the borders)
        }

        bins = newBins;
    }

    std::ostream&
    operator<<(std::ostream& os, const Histogram1D& histo)
    {
        os << "Histogram1D:\n";
        os << "min:;" << histo.min << ";max:;" << histo.max << ";\n";

        os << "Index:;";
        for (std::size_t index = 0; index < histo.bins.size(); index++)
        {
            os << index << ";";
        }
        os << "\n";
        os << "Values:;";
        for (std::size_t index = 0; index < histo.bins.size(); index++)
        {
            os << histo.indexToValue(index) << ";";
        }
        os << "\n";
        os << "Counts:;";
        for (std::size_t index = 0; index < histo.bins.size(); index++)
        {
            os << histo.bins[index] << ";";
        }
        os << "\n";

        return os;
    }

} // namespace simox::math
