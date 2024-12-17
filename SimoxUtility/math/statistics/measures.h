#pragma once

#include <algorithm>
#include <cmath>
#include <sstream>
#include <vector>

#include <SimoxUtility/error/SimoxError.h>
#include <SimoxUtility/math/mean.h>

namespace simox::math
{

    namespace detail
    {
        template <typename Float>
        void
        check_not_empty(const std::vector<Float>& values)
        {
            if (values.empty())
            {
                throw simox::error::SimoxError("Passed vector of values is empty.");
            }
        }
    } // namespace detail

    template <typename Float>
    std::vector<Float>
    sorted(const std::vector<Float>& values)
    {
        std::vector<Float> s = values;
        std::sort(s.begin(), s.end());
        return s;
    }

    template <typename Float>
    Float
    min(const std::vector<Float>& values, bool isSorted = false)
    {
        detail::check_not_empty(values);
        return isSorted ? values.front() : *std::min_element(values.begin(), values.end());
    }

    template <typename Float>
    Float
    max(const std::vector<Float>& values, bool isSorted = false)
    {
        detail::check_not_empty(values);
        return isSorted ? values.back() : *std::max_element(values.begin(), values.end());
    }

    template <typename Float>
    Float
    mean(const std::vector<Float>& values)
    {
        detail::check_not_empty(values);
        return simox::math::mean(values.begin(), values.end());
    }

    template <typename Float>
    Float
    stddev(const std::vector<Float>& values, Float mean)
    {
        detail::check_not_empty(values);
        Float sum = 0;
        for (Float v : values)
        {
            Float diff = v - mean;
            sum += diff * diff;
        }
        Float variance = sum / (values.size() - 1);
        return std::sqrt(variance);
    }

    template <typename Float>
    Float
    stddev(const std::vector<Float>& values)
    {
        return stddev(values, mean(values));
    }

    template <typename Float>
    Float
    quantile(const std::vector<Float>& _values, Float q, bool isSorted = false)
    {
        if (q < 0 || q > 1)
        {
            std::stringstream ss;
            ss << "Quantile must be in [0, 1], but was " << q << ".";
            throw error::SimoxError(ss.str());
        }
        detail::check_not_empty(_values);
        const std::vector<Float>& values = isSorted ? _values : sorted(_values);

        if (q <= 0)
        {
            return values.front();
        }
        if (q >= 1)
        {
            return values.back();
        }

        Float loc = q * (values.size() - 1);
        int iloc = int(std::floor(loc));

        if (loc == iloc)
        {
            return values[iloc];
        }
        else
        {
            Float t = loc - iloc;
            return (1 - t) * values[iloc] + t * values[iloc + 1];
        }
    }

    template <typename Float>
    Float
    lower_quartile(const std::vector<Float>& values, bool isSorted = false)
    {
        return quantile<Float>(values, .25, isSorted);
    }

    template <typename Float>
    Float
    median(const std::vector<Float>& values, bool isSorted = false)
    {
        return quantile<Float>(values, .5, isSorted);
    }

    template <typename Float>
    Float
    upper_quartile(const std::vector<Float>& values, bool isSorted = false)
    {
        return quantile<Float>(values, .75, isSorted);
    }

    template <typename Float>
    Float
    interquartile_range(Float lowerQuartile, Float upperQuartile)
    {
        return upperQuartile - lowerQuartile;
    }

    template <typename Float>
    Float
    interquartile_range(const std::vector<Float>& _values, bool isSorted = false)
    {
        detail::check_not_empty(_values);

        const std::vector<Float>& values = isSorted ? _values : sorted(_values);
        return interquartile_range(lower_quartile(values, true), upper_quartile(values, true));
    }

} // namespace simox::math
