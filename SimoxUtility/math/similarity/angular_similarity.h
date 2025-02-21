#pragma once


// STD/STL
#include <cmath>

// Eigen
#include <eigen3/Eigen/Core>

// Simox
#include <SimoxUtility/math/similarity/cosine_similarity.h>

namespace simox::math
{

    /**
     * @brief Computes the angular similarity between two vectors.
     *
     * Output is [0, 1], where 0 is most dissimilar and 1 perfect match.
     *
     * @see https://en.wikipedia.org/wiki/Cosine_similarity#Angular_distance_and_similarity
     *
     * @param v1 Vector 1.
     * @param v2 Vector 2.
     * @return Element of [0, 1].
     */
    template <int rows>
    float
    angular_similarity(const Eigen::Matrix<float, rows, 1>& v1,
                       const Eigen::Matrix<float, rows, 1>& v2)
    {
        const float cosine_similarity = math::cosine_similarity(v1, v2);
        const float angular_distance = std::acos(cosine_similarity) / M_PI;
        const float angular_similarity = 1.f - angular_distance;

        // Clamp to deal with numerical inaccuracies.
        return std::clamp(angular_similarity, 0.f, 1.f);
    }

} // namespace simox::math
