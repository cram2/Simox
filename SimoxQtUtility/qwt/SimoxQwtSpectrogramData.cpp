#include "SimoxQwtSpectrogramData.h"

namespace simox::qt
{
    SimoxQwtSpectrogramData::SimoxQwtSpectrogramData(std::size_t iterations,
                                                     std::size_t bucket_count,
                                                     float frequency)
    {
        setFrequency(frequency, bucket_count);
        setIterations(iterations);
        //        setInterval(Qt::ZAxis, QwtInterval(0.0, 10.0));
    }

    double
    SimoxQwtSpectrogramData::value(double fx, double fy) const
    {
        const std::size_t x = fx;
        const std::size_t y = fy / _bucket_height;
        if (x >= _vals.size())
        {
            return 0;
        }
        if (y >= _vals.at(x).size())
        {
            return 0;
        }
        return _vals.at(x).at(y);
    }

    const std::vector<float>&
    SimoxQwtSpectrogramData::update(const std::vector<std::complex<float>>& v)
    {
        return update(v, v.size());
    }

    const std::vector<float>&
    SimoxQwtSpectrogramData::update(const std::vector<std::complex<float>>& v, std::size_t n)
    {
        std::vector<float> init(n);
        for (std::size_t i = 0; i < n; ++i)
        {
            init.at(i) = std::abs(v.at(i));
        }
        return update(std::move(init));
    }

    const std::vector<float>&
    SimoxQwtSpectrogramData::update(std::vector<float> v)
    {
        while (_vals.size() < _iterations)
        {
            _vals.emplace_front();
        }
        _vals.emplace_back(std::move(v));
        while (_vals.size() > _iterations)
        {
            _vals.pop_front();
        }
        return _vals.back();
    }

    std::size_t
    SimoxQwtSpectrogramData::iterations() const
    {
        return _iterations;
    }

    void
    SimoxQwtSpectrogramData::setIterations(std::size_t iterations)
    {
        _iterations = iterations;
        //        setInterval(Qt::XAxis, QwtInterval(0, iterations));
    }

    void
    SimoxQwtSpectrogramData::setFrequency(float freq, std::size_t bucket_count)
    {
        _bucket_height = freq / 2 / bucket_count;
        _bucket_count = bucket_count;
        //        setInterval(Qt::YAxis, QwtInterval(0, freq / 2));
    }
} // namespace simox::qt
