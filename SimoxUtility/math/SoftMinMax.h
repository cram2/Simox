#pragma once

#include <queue>

namespace simox::math
{

    /**
     * @brief The SoftMinMax class can be used to find soft min and max of a set of floats.
     *
     * Soft means that some values are allowed to be >= soft min / <= soft max
     * (including the soft min/max), respectively.
     * A quantile parameter in [0..0.5] specifies the proportion of values
     * which are allowed to excess the soft min/max.
     */
    class SoftMinMax
    {
    public:
        /// No initialization constructor.
        SoftMinMax();

        /**
         * @brief Constructs a SoftMinMax for given quantile and number of values.
         *
         * @param quantile
         *  The proportion of values that may excess the soft min/max. Must be in [0..0.5].
         * @param numValues the total number of values that will be added. Must be > 0.
         *
         * @throws `std::invalid_argument`
         *  If one of the parameters value does not meet the requirements
         */
        SoftMinMax(float quantile, std::size_t numValues);

        /**
         * Reinitializes the SoftMinMax with the given arguments.
         * @see `SoftMinMax()`
         * @throws `std::invalid_argument`
         *  If one of the parameters value does not meet the requirements.
         */
        void reset(float quantile, std::size_t numValues);

        /// Add a value to the considered collection.
        /// @note Only values excessing the current soft min/max are stored.
        void add(float value);

        /// Get the current soft min.
        /// @throws `std::out_of_range` If no element was added.
        float getSoftMin() const;
        /// Get the current soft max.
        /// @throws `std::out_of_range` If no element was added.
        float getSoftMax() const;


    private:
        /// Returns the number of elements that are allowed to be <= softMin / >= softMax.
        std::size_t numOutsideSoftMinMax() const;

        /// The allowed size of the priority queues
        std::size_t allowedHeapSize() const;


        /// The quantile in [0, 0.5].
        float quantile = 0;
        /// The number of elements to be added.
        std::size_t num_elements = 0;

        std::size_t allowed_heap_size_cache = 0;


        using Container = std::vector<float>;
        using MaxCompare = std::less<float>; // Results in max heap.
        using MinCompare = std::greater<float>; // Results in min heap.

        using MinQueue = std::priority_queue<float, Container, MaxCompare>;
        using MaxQueue = std::priority_queue<float, Container, MinCompare>;

        /// Stores all elements <= softMin == top()
        MinQueue minQueue;
        /// Stores all elements >= softMax == top()
        MaxQueue maxQueue;
        // Invariant: minQueue.size() == maxQueue.size()
    };

} // namespace simox::math
