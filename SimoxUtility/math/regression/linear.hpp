#pragma once

#include <eigen3/Eigen/Core>
#include <Eigen/Dense>

namespace simox::math
{

    /**
     * @brief A linear regression model.
     *
     * The linear model has the form
     *
     *      y = a + b * x,
     *
     * or per dimension
     *
     *      y_i = a_i + b_i * x  (i = 1 .. d),
     *
     * where
     *
     * - x is the scalar input variable (e.g. time)
     * - y is an n-D vector output variable (e.g. position)
     * - a is an n-D bias vector
     * - b is an n-D slope vector
     *
     * In matrix notation, the equation system represented by the model is:
     *
     * [[ a_1  b_1 ]            [ y_1 ]
     *  [   ...    ]    [ 1 ] = [ ... ]
     *  [ a_i  b_i ]  * [ x ] = [ y_i ]
     *  [   ...    ]          = [ ... ]
     *  [ a_d  b_d ]]           [ y_d ]
     *
     * Given data x[j] in R and y[j] \in R^3 (j = 1 .. n),
     * the regression solves the following equation system(s)
     * for [ a_i, b_i ] (i = 1 .. d):
     *
     * [[ 1  x[1] ]             [ y[1]_i ]
     *  [   ...   ]   [ a_i ]   [  ...   ]
     *  [ 1  x[j] ] * [ b_i ] = [ y[j]_i ]
     *  [   ...   ]             [  ...   ]
     *  [ 1  x[n] ]]            [ y[n]_i ]
     */
    template <int _Dim, typename _FloatT = double>
    class LinearRegression
    {
    public:
        static constexpr int Dim = _Dim;
        using FloatT = _FloatT;
        using VectorT = typename Eigen::Matrix<FloatT, Dim, 1>;
        using CoefficientsMatrixT = typename Eigen::Matrix<FloatT, Dim, 2>;


    public:
        /**
         * The coefficients of the bias term (a_i) and input variable x (b_i)
         * [[ a_1  b_1 ]
         *  [ ...  ... ]
         *  [ a_d  b_d ]]
         */
        CoefficientsMatrixT coefficients = CoefficientsMatrixT::Zero();

        /// The input offset, so the virtual input x' = x + offset.
        FloatT inputOffset = 0;

        /**
         * @brief Fit a linear regression model to the given data.
         *
         * xs and ys must not be empty. All vectors in ys must have the same size
         * even if Dim == Eigen::Dynamic.
         *
         * @param xs The input variables.
         * @param ys The output variables.
         * @param offsetInput If true, the inputs are offset to x' = x - x_0.
         * @return The regression model.
         */
        static LinearRegression
        Fit(const std::vector<FloatT>& xs, const std::vector<VectorT>& ys, bool offsetInput = false)
        {
            using VectorX = typename Eigen::Matrix<FloatT, Eigen::Dynamic, 1>;
            using MatrixX2 = typename Eigen::Matrix<FloatT, Eigen::Dynamic, 2>;
            using MatrixDX = typename Eigen::Matrix<FloatT, Dim, Eigen::Dynamic>;

            if (offsetInput and xs.at(0) != 0)
            {
                FloatT offset = -xs.at(0); // Move x_0 to 0.
                std::vector<FloatT> virtualXs = xs;
                for (FloatT& x : virtualXs)
                {
                    x = x + offset;
                }
                LinearRegression r = LinearRegression::Fit(virtualXs, ys, false);
                r.inputOffset = offset;
                return r;
            }

            MatrixDX ysMatrix(ys.at(0).size(), ys.size());
            for (long col = 0; col < ysMatrix.cols(); ++col)
            {
                ysMatrix.col(col) = ys.at(col);
            }

            // The matrix of the predictor functions evaluated at the corresponding xs.
            // Since this is a linear regression, the constant function a(t) = 1 and identity
            // b(t) = t are used.
            MatrixX2 linFuncMatrix(xs.size(), 2);
            linFuncMatrix.col(0) = VectorX::Ones(xs.size());
            linFuncMatrix.col(1) = Eigen::Map<const VectorX>(xs.data(), xs.size());

            // `linFuncMatrix` is poorly conditioned for xs that are close together
            // (e.g. time stamps), so the normal equation would lose a lot of precision.
            auto qrDecomp = linFuncMatrix.colPivHouseholderQr();

            // Each coordinate can be treated individually (general multivariate regression).
            // `coeffs` contains a_i and b_i in a_0 + b_0 * t = x, a_1 + b_1 * t = y, etc.
            CoefficientsMatrixT coeffs(ysMatrix.rows(), 2);
            for (int dim = 0; dim < ysMatrix.rows(); ++dim)
            {
                VectorX coords = ysMatrix.row(dim).transpose();
                coeffs.row(dim) = qrDecomp.solve(coords);
            }

            return LinearRegression{.coefficients = coeffs};
        }

        /**
         * @brief Predict the output variable of the given input variable.
         * @param x The input variable.
         * @return The predicted output variable.
         */
        VectorT
        predict(FloatT x) const
        {
            Eigen::Matrix<FloatT, 2, 1> input;
            input << 1.0, x + inputOffset;
            return coefficients * input;
        }
    };

    template <int _Dim, typename _FloatT = double>
    std::ostream&
    operator<<(std::ostream& os, const LinearRegression<_Dim, _FloatT>& r)
    {
        os << "<LinearRegression y = a + b * x with [ ";
        for (Eigen::Index row = 0; row < r.coefficients.rows(); ++row)
        {
            if (row != 0)
            {
                os << " | ";
            }
            os << "y_" << row << " = " << r.coefficients(row, 0) << " + " << r.coefficients(row, 1)
               << " * x";
        }
        os << " ] and input offset " << r.inputOffset << ">";
        return os;
    }

} // namespace simox::math
