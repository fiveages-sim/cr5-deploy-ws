//
// GravityCompensation implementation (Pinocchio RNEA).
//
#include "ht_gravity_compensation/gravity_compensation.hpp"

#include <stdexcept>

namespace ht_gravity_compensation
{

GravityCompensation::GravityCompensation(
  const std::string & urdf_string, const std::vector<double> & gravity)
{
  if (urdf_string.empty())
  {
    throw std::invalid_argument("GravityCompensation: empty URDF string");
  }

  pinocchio::urdf::buildModelFromXML(urdf_string, model_);
  data_ = pinocchio::Data(model_);

  if (gravity.size() == 3)
  {
    model_.gravity = pinocchio::Motion::Zero();
    model_.gravity.linear() << gravity[0], gravity[1], gravity[2];
  }
}

Eigen::VectorXd GravityCompensation::calculateStaticTorques(
  const Eigen::VectorXd & q) const
{
  return pinocchio::rnea(
    model_, data_, q,
    Eigen::VectorXd::Zero(model_.nv),
    Eigen::VectorXd::Zero(model_.nv));
}

}  // namespace ht_gravity_compensation
