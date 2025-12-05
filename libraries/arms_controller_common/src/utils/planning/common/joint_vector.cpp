#include "arms_controller_common/utils/planning/common/joint_vector.h"
namespace planning {
JointVector::JointVector(){};

JointVector::JointVector(size_t n) : data(n) { data.setZero(); };

JointVector::JointVector(const JointVector &arg) : data(arg.data){};
JointVector::JointVector(Eigen::VectorXd arg) : data(arg){};
void JointVector::resize(size_t n) {
  data.resize(n);
  data.setZero();
};

size_t JointVector::getJointSize() { return data.size(); };

JointVector &JointVector::operator=(const JointVector &arg) {
  data = arg.data;
  return *this;
}

double JointVector::operator()(size_t index) const { return data(index); };

double &JointVector::operator()(size_t index) { return data(index); };

JointVector JointVector::operator-() const { return JointVector(-data); };

JointVector JointVector::operator+(const JointVector &arg) const {
  return JointVector(data + arg.data);
};

JointVector JointVector::operator-(const JointVector &arg) const {
  return JointVector(data - arg.data);
};

JointVector JointVector::operator*(double arg) const {
  return JointVector(data * arg);
};

JointVector JointVector::zero(size_t n) {
  return JointVector(Eigen::VectorXd::Zero(n));
};

JointVector JointVector::one(size_t n) {
  return JointVector(Eigen::VectorXd::Ones(n));
};
} // namespace planning
