#ifndef BUTTERWORTH_FILTER_HPP
#define BUTTERWORTH_FILTER_HPP

#include <Eigen/Dense>
#include <vector>

using scalar_t = double;
namespace ocs2::mobile_manipulator
{
    class ButterworthFilter {
    public:
        /**
         * @brief Constructor
         * @param dof Number of DOFs (e.g., joints)
         * @param cutoffHz Cutoff frequency in Hz
         * @param sampleHz Sampling frequency in Hz
         */
        ButterworthFilter(size_t dof, scalar_t cutoffHz, scalar_t sampleHz);

        /**
         * @brief Apply the filter to a new input vector
         * @param input Current input (size = DOF)
         * @return Smoothed output (size = DOF)
         */
        Eigen::Matrix<scalar_t, Eigen::Dynamic, 1> update(const Eigen::Matrix<scalar_t, Eigen::Dynamic, 1>& input);

        /**
         * @brief Reset filter state
         */
        void reset();

    private:
        size_t dof_;  // number of joints / signals

        // Filter coefficients
        scalar_t b0_, b1_, b2_, a1_, a2_, callCounter_, skipCount_;

        // State history
        Eigen::VectorXd x1_, x2_;  // previous inputs
        Eigen::VectorXd y1_, y2_;  // previous outputs

        void computeCoefficients(scalar_t cutoff, scalar_t fs);
    };
}
#endif // BUTTERWORTH_FILTER_HPP
