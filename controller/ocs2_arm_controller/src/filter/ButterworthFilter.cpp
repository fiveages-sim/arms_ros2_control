#include "ocs2_arm_controller/filter/ButterworthFilter.h"
#include <cmath>
namespace ocs2::mobile_manipulator
{
    ButterworthFilter::ButterworthFilter(size_t dof, scalar_t cutoffHz, scalar_t sampleHz)
        : dof_(dof), callCounter_(0), skipCount_(20) {
        computeCoefficients(cutoffHz, sampleHz);
        x1_ = Eigen::VectorXd::Zero(dof_);
        x2_ = Eigen::VectorXd::Zero(dof_);
        y1_ = Eigen::VectorXd::Zero(dof_);
        y2_ = Eigen::VectorXd::Zero(dof_);
    }

    void ButterworthFilter::computeCoefficients(scalar_t cutoff, scalar_t fs) {
        const scalar_t PI = 3.14159265358979323846;
        scalar_t wc = std::tan(PI * cutoff / fs);  // pre-warped analog frequency
        scalar_t k1 = std::sqrt(2.0) * wc;
        scalar_t k2 = wc * wc;
        scalar_t a0 = 1.0 + k1 + k2;

        b0_ = k2 / a0;
        b1_ = 2.0 * b0_;
        b2_ = b0_;
        a1_ = 2.0 * (k2 - 1.0) / a0;
        a2_ = (1.0 - k1 + k2) / a0;
    }

    Eigen::Matrix<scalar_t, Eigen::Dynamic, 1>
    ButterworthFilter::update(const Eigen::Matrix<scalar_t, Eigen::Dynamic, 1>& input) {
        Eigen::VectorXd output(dof_);
        // Skip filtering during first few calls
        if (callCounter_ < skipCount_) {
            ++callCounter_;
            x2_ = x1_;
            x1_ = input;
            y2_ = y1_;
            y1_ = input;
            // No need to update x2_, y1_, y2_ unless you want smoother transition
            return input;
        }
        for (size_t i = 0; i < dof_; ++i) {
            scalar_t in = input(i);

            scalar_t out = b0_ * in + b1_ * x1_(i) + b2_ * x2_(i)
                        - a1_ * y1_(i) - a2_ * y2_(i);

            // Update state
            x2_(i) = x1_(i);
            x1_(i) = in;
            y2_(i) = y1_(i);
            y1_(i) = out;   

            output(i) = out;
        }
        return output;
    }

    void ButterworthFilter::reset() {
        x1_.setZero();
        x2_.setZero();
        y1_.setZero();
        y2_.setZero();
        callCounter_ = 0;
    }
}