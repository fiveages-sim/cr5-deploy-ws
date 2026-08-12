//
// Pinocchio-based gravity compensation for Panthera HT.
//
// Builds a Pinocchio model from a URDF string and computes static gravity
// torques via RNEA with zero velocity / zero acceleration.
//
#pragma once

#include <string>
#include <vector>

#include <Eigen/Dense>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#pragma GCC diagnostic pop

namespace ht_gravity_compensation
{

/// 基于 Pinocchio 的重力补偿动力学封装（静态重力矩）。
class GravityCompensation
{
public:
  /// @param urdf_string 完整 URDF XML（如 /robot_description 参数内容）
  /// @param gravity 世界系重力加速度 [gx, gy, gz]（默认 [0, 0, -9.81]；
  ///               双臂 body_rpy 非零时需按安装方向旋转）
  explicit GravityCompensation(
    const std::string & urdf_string,
    const std::vector<double> & gravity = {0.0, 0.0, -9.81});

  /// 对给定关节位置计算静态重力矩（模型 nq 维输入，nv 维输出）
  Eigen::VectorXd calculateStaticTorques(const Eigen::VectorXd & q) const;

  const pinocchio::Model & getModel() const { return model_; }
  size_t getNumJoints() const { return model_.nq; }
  bool isValid() const { return model_.nq > 0; }

private:
  pinocchio::Model model_;
  mutable pinocchio::Data data_;
};

}  // namespace ht_gravity_compensation
