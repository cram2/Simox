#pragma once

#include <complex>
#include <deque>
#include <vector>

#include <qwt_plot_spectrogram.h>

namespace simox::qt
{
    class SimoxQwtSpectrogramData : public QwtRasterData
    {
    public:
        SimoxQwtSpectrogramData(std::size_t iterations, std::size_t max_fft_width, float frequency);

        virtual double value(double fx, double fy) const;

        const std::vector<float>& update(const std::vector<std::complex<float>>& v, std::size_t n);
        const std::vector<float>& update(const std::vector<std::complex<float>>& v);
        const std::vector<float>& update(std::vector<float> v);
        std::size_t iterations() const;
        void setIterations(std::size_t iterations);
        void setFrequency(float freq, std::size_t bucket_count);

    private:
        std::deque<std::vector<float>> _vals;
        std::size_t _iterations;
        float _bucket_height;
        std::size_t _bucket_count;
    };
} // namespace simox::qt
