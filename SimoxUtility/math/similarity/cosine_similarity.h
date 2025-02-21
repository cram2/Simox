#pragma once


#include <eigen3/Eigen/Core>

namespace simox::math
{

    /**
     * @brief Computes the cosine similarity between two vectors.
     *
     * Output is [-1, 1], where -1 is most dissimilar and 1 perfect match.  0 means that the vectors
     * are orthogonal.
     *
     * @see https://en.wikipedia.org/wiki/Cosine_similarity#Definition
     *
     * @param v1 Vector 1.
     * @param v2 Vector 2.
     * @return Element of [-1, 1].
     */
    template <int rows>
    float
    cosine_similarity(const Eigen::Matrix<float, rows, 1>& v1,
                      const Eigen::Matrix<float, rows, 1>& v2)
    {
        if (v1.isZero() or v2.isZero())
        {
            throw std::logic_error{"Cosine similarity is not defined on operands which are zero."};
        }

        const float cosine_similarity = v1.dot(v2) / (v1.norm() * v2.norm());

        // Clamp to deal with numerical inaccuracies.
        return std::clamp(cosine_similarity, -1.f, 1.f);
    }

} // namespace simox::math
