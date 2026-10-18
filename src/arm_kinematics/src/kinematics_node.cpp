#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <urdf/model.h>

#include "arm_interfaces/srv/solve_fk.hpp"
#include "arm_interfaces/srv/solve_ik.hpp"

namespace {

geometry_msgs::msg::Pose toPose(const KDL::Frame & frame) {
  geometry_msgs::msg::Pose pose;
  pose.position.x = frame.p.x();
  pose.position.y = frame.p.y();
  pose.position.z = frame.p.z();
  double x = 0.0, y = 0.0, z = 0.0, w = 1.0;
  frame.M.GetQuaternion(x, y, z, w);
  pose.orientation.x = x;
  pose.orientation.y = y;
  pose.orientation.z = z;
  pose.orientation.w = w;
  return pose;
}

KDL::Frame toFrame(const geometry_msgs::msg::Pose & pose) {
  return KDL::Frame(
    KDL::Rotation::Quaternion(
      pose.orientation.x, pose.orientation.y,
      pose.orientation.z, pose.orientation.w),
    KDL::Vector(pose.position.x, pose.position.y, pose.position.z));
}

}  // namespace

class KinematicsNode : public rclcpp::Node {
public:
  KinematicsNode()
  : Node("kinematics_node"), rng_(42) {
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    tip_frame_ = declare_parameter<std::string>("tip_frame", "tcp");
    max_restarts_ = declare_parameter<int>("max_restarts", 8);
    const std::string urdf_xml =
      declare_parameter<std::string>("robot_description", "");

    if (urdf_xml.empty()) {
      throw std::runtime_error("robot_description parameter is empty");
    }

    KDL::Tree tree;
    if (!kdl_parser::treeFromString(urdf_xml, tree)) {
      throw std::runtime_error("failed to parse robot_description into a KDL tree");
    }
    if (!tree.getChain(base_frame_, tip_frame_, chain_)) {
      throw std::runtime_error(
              "failed to build KDL chain " + base_frame_ + " -> " + tip_frame_);
    }

    n_joints_ = static_cast<int>(chain_.getNrOfJoints());
    if (n_joints_ <= 0) {
      throw std::runtime_error("KDL chain has no movable joints");
    }

    q_min_.resize(n_joints_);
    q_max_.resize(n_joints_);
    joint_names_.clear();

    urdf::Model model;
    if (!model.initString(urdf_xml)) {
      throw std::runtime_error("failed to parse URDF joint limits");
    }

    int joint_index = 0;
    for (unsigned int i = 0; i < chain_.getNrOfSegments(); ++i) {
      const KDL::Joint & joint = chain_.getSegment(i).getJoint();
      if (joint.getType() == KDL::Joint::None) {
        continue;
      }
      joint_names_.push_back(joint.getName());
      const auto urdf_joint = model.getJoint(joint.getName());
      if (urdf_joint && urdf_joint->limits) {
        q_min_(joint_index) = urdf_joint->limits->lower;
        q_max_(joint_index) = urdf_joint->limits->upper;
      } else {
        q_min_(joint_index) = -3.14;
        q_max_(joint_index) = 3.14;
      }
      ++joint_index;
    }
    joint_names_.resize(n_joints_);

    fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_);
    Eigen::Matrix<double, 6, 1> weights;
    weights << 1.0, 1.0, 1.0, 0.1, 0.1, 0.1;
    ik_solver_ =
      std::make_unique<KDL::ChainIkSolverPos_LMA>(chain_, weights, 1e-4, 500);

    current_q_.resize(n_joints_);
    current_q_.data.setZero();

    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
          auto it = std::find(joint_names_.begin(), joint_names_.end(), msg->name[i]);
          if (it == joint_names_.end()) {
            continue;
          }
          const size_t idx = static_cast<size_t>(it - joint_names_.begin());
          current_q_(static_cast<unsigned int>(idx)) = msg->position[i];
          have_current_ = true;
        }
      });

    fk_service_ = create_service<arm_interfaces::srv::SolveFK>(
      "kinematics/solve_fk",
      std::bind(&KinematicsNode::handleFk, this,
      std::placeholders::_1, std::placeholders::_2));
    ik_service_ = create_service<arm_interfaces::srv::SolveIK>(
      "kinematics/solve_ik",
      std::bind(&KinematicsNode::handleIk, this,
      std::placeholders::_1, std::placeholders::_2));

    std::string names;
    for (const auto & name : joint_names_) {
      names += name + " ";
    }
    RCLCPP_INFO(
      get_logger(), "KDL chain %s -> %s, %d joints: %s",
      base_frame_.c_str(), tip_frame_.c_str(), n_joints_, names.c_str());
  }

private:
  void handleFk(
    const std::shared_ptr<arm_interfaces::srv::SolveFK::Request> request,
    std::shared_ptr<arm_interfaces::srv::SolveFK::Response> response)
  {
    if (static_cast<int>(request->joints.size()) != n_joints_) {
      response->success = false;
      response->message =
        "expected " + std::to_string(n_joints_) + " joint values";
      return;
    }

    KDL::JntArray q(n_joints_);
    for (int i = 0; i < n_joints_; ++i) {
      q(i) = request->joints[static_cast<size_t>(i)];
    }

    KDL::Frame frame;
    if (fk_solver_->JntToCart(q, frame) < 0) {
      response->success = false;
      response->message = "KDL forward kinematics failed";
      return;
    }

    response->success = true;
    response->pose = toPose(frame);
    response->message = "ok";
  }

  void handleIk(
    const std::shared_ptr<arm_interfaces::srv::SolveIK::Request> request,
    std::shared_ptr<arm_interfaces::srv::SolveIK::Response> response)
  {
    if (request->target.orientation.w == 0.0 &&
      request->target.orientation.x == 0.0 &&
      request->target.orientation.y == 0.0 &&
      request->target.orientation.z == 0.0)
    {
      response->success = false;
      response->message = "target quaternion is all zeros";
      return;
    }

    const KDL::Frame target = toFrame(request->target);

    std::vector<KDL::JntArray> seeds;
    KDL::JntArray seed(n_joints_);
    if (static_cast<int>(request->seed.size()) == n_joints_) {
      for (int i = 0; i < n_joints_; ++i) {
        seed(i) = request->seed[static_cast<size_t>(i)];
      }
      seeds.push_back(seed);
    }
    if (have_current_) {
      seeds.push_back(current_q_);
    }
    KDL::JntArray zero_seed(n_joints_);
    for (int i = 0; i < n_joints_; ++i) {
      zero_seed(i) = std::min(std::max(0.0, q_min_(i)), q_max_(i));
    }
    seeds.push_back(zero_seed);

    if (seeds.empty()) {
      seeds.push_back(zero_seed);
    }

    std::uniform_real_distribution<double> noise(-1.0, 1.0);
    for (int attempt = 0; attempt < max_restarts_; ++attempt) {
      KDL::JntArray perturbed = seeds.front();
      for (int i = 0; i < n_joints_; ++i) {
        const double value = perturbed(i) + noise(rng_);
        perturbed(i) = std::min(std::max(value, q_min_(i)), q_max_(i));
      }
      seeds.push_back(perturbed);
    }

    double best_cost = std::numeric_limits<double>::max();
    KDL::JntArray best_q(n_joints_);
    double best_pos = 0.0;
    double best_rot = 0.0;
    bool found = false;

    for (const auto & candidate : seeds) {
      KDL::JntArray q_out(n_joints_);
      if (ik_solver_->JntToCart(candidate, q_out, target) < 0) {
        continue;
      }

      bool within_limits = true;
      for (int i = 0; i < n_joints_; ++i) {
        if (!std::isfinite(q_out(i)) ||
          q_out(i) < q_min_(i) - 1e-4 || q_out(i) > q_max_(i) + 1e-4)
        {
          within_limits = false;
          break;
        }
      }
      if (!within_limits) {
        continue;
      }

      KDL::Frame reached;
      if (fk_solver_->JntToCart(q_out, reached) < 0) {
        continue;
      }

      const double pos_error = (reached.p - target.p).Norm();
      const double rot_error =
        (target.M * reached.M.Inverse()).GetRot().Norm();
      const double cost = pos_error + 0.1 * rot_error;

      if (cost < best_cost) {
        best_cost = cost;
        best_q = q_out;
        best_pos = pos_error;
        best_rot = rot_error;
      }

      if (pos_error < 0.005 && rot_error < 0.10) {
        found = true;
        break;
      }
    }

    response->pos_error = best_pos;
    response->rot_error = best_rot;

    if (!found) {
      response->success = false;
      response->message =
        "IK failed after " + std::to_string(seeds.size()) +
        " seeds (best pos err " + std::to_string(best_pos * 1000.0) +
        " mm, rot err " + std::to_string(best_rot) + " rad)";
      return;
    }

    response->success = true;
    response->joints.resize(static_cast<size_t>(n_joints_));
    for (int i = 0; i < n_joints_; ++i) {
      response->joints[static_cast<size_t>(i)] = best_q(i);
    }
    response->message = "ok";
  }

  std::string base_frame_;
  std::string tip_frame_;
  int max_restarts_{8};
  int n_joints_{0};
  bool have_current_{false};

  KDL::Chain chain_;
  KDL::JntArray q_min_;
  KDL::JntArray q_max_;
  KDL::JntArray current_q_;
  std::vector<std::string> joint_names_;

  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
  std::unique_ptr<KDL::ChainIkSolverPos_LMA> ik_solver_;
  std::mt19937 rng_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Service<arm_interfaces::srv::SolveFK>::SharedPtr fk_service_;
  rclcpp::Service<arm_interfaces::srv::SolveIK>::SharedPtr ik_service_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<KinematicsNode>();
    rclcpp::spin(node);
  } catch (const std::exception & exc) {
    RCLCPP_FATAL(rclcpp::get_logger("kinematics_node"), "%s", exc.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
