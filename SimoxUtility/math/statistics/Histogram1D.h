#pragma once

#include <ostream>
#include <vector>

namespace simox::math
{

    /**
     * @brief Histogram1D for one-dimensional float data.
     */
    class Histogram1D
    {
    public:
        /// No initialization constructor.
        Histogram1D();

        /// Construct with the given data.
        /// Minimum and maximum are derived automatically.
        Histogram1D(const std::vector<float>& data, std::size_t numBins = 128);

        /// Construct with the given data and given limits.
        Histogram1D(const std::vector<float>& data,
                    float min,
                    float max,
                    std::size_t numBins = 128);


        /// Set the number of bins and set them to 0.
        void resetBins(std::size_t numBins);

        /// Insert the given value into the Histogram1D.
        void insert(float value);
        /// Inserts the given values into the Histogram1D.
        void insert(const std::vector<float>& value);

        /// Set the limits.
        void setMinMax(float min, float max);
        /// Set the limits from the given data.
        void setMinMax(const std::vector<float>& data);


        /// Get the bins.
        const std::vector<std::size_t>& getBins() const;

        /// Get the number of bins.
        std::size_t getNumberOfBins() const;

        ///  Transfer the given value to its corresponding bin index.
        std::size_t valueToIndex(float value) const;
        /// Get the value at the center of the bin with the given index.
        float indexToValue(std::size_t index) const;

        /// Get the minimum, i.e. the lower limit.
        float getMin() const;
        /// Get the maximum, i.e. the upper limit.
        float getMax() const;

        /// Return the index of the bin containing the fewest data points.
        std::size_t getMinBinIndex() const;
        /// Returns the corresponding value of the bin containing the fewest data points.
        float getMinBinValue() const;

        /// Return the index of the bin containing the most data points.
        std::size_t getMaxBinIndex() const;
        /// Returns the corresponding value of the bin containing the most data points.
        float getMaxBinValue() const;


        /**
         * @brief Applies a median filter to the Histogram1D bins.
         *
         * The size specifies the number of neighours (in each direction)
         * considered.
         * That is, if k = size, 2k+1 values are considered for each point
         * (k neighbours in each direction).
         *
         * @param size the size of the neighbourhood (in each direction)
         */
        void applyMedianFilter(std::size_t size = 2);


        /// Streams a CVS-like description of the Histogram1D containing the bin data.
        friend std::ostream& operator<<(std::ostream& os, const Histogram1D& histo);


    private:
        /// The minimum mapped value.
        float min;
        /// The maximum mapped value.
        float max;

        /// The bins.
        std::vector<std::size_t> bins;
    };

} // namespace simox::math
