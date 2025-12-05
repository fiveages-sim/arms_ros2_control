#pragma once
#include <Eigen/Dense>

namespace planning {
    class JointVector {

    public:
        JointVector();
        ~JointVector() = default;

        explicit JointVector(size_t n);

        JointVector(const JointVector &arg);

        JointVector(Eigen::VectorXd arg);

        void resize(size_t n);

        size_t getJointSize();

        JointVector &operator=(const JointVector &arg);

        double operator()(size_t index) const;

        double &operator()(size_t index);

        JointVector operator-() const;

        JointVector operator+(const JointVector &arg) const;

        JointVector operator-(const JointVector &arg) const;

        JointVector operator*(double arg) const;

        static JointVector zero(size_t n);

        static JointVector one(size_t n);

        Eigen::VectorXd data;
    };

} // namespace planning
