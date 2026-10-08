#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <chrono>
#include <cmath>
#include <memory>
#include <thread>
#include <vector>

// program variables
static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_node");
static const std::string PLANNING_GROUP_ROBOT = "ur_manipulator";
static const std::string PLANNING_GROUP_GRIPPER = "gripper";

class ApproachRetreatTrajectory {
public:
  ApproachRetreatTrajectory(rclcpp::Node::SharedPtr base_node_)
      : base_node_(base_node_) {
    RCLCPP_INFO(LOGGER, "Initializing Class: Approach Retreat Trajectory...");

    // configure node options
    rclcpp::NodeOptions node_options;
    // auto-declare node_options parameters from overrides
    node_options.automatically_declare_parameters_from_overrides(true);

    // initialize move_group node
    move_group_node_ =
        rclcpp::Node::make_shared("move_group_node", node_options);
    // start move_group node in a new executor thread and spin it
    executor_.add_node(move_group_node_);
    std::thread([this]() { this->executor_.spin(); }).detach();

    // initialize move_group robot interfaces
    move_group_robot_ = std::make_shared<MoveGroupInterface>(
        move_group_node_, PLANNING_GROUP_ROBOT);

    // initialize move_group gripper interfaces
    move_group_gripper_ = std::make_shared<MoveGroupInterface>(
        move_group_node_, PLANNING_GROUP_GRIPPER);

    // get initial state of robot
    joint_model_group_robot_ =
        move_group_robot_->getCurrentState()->getJointModelGroup(
            PLANNING_GROUP_ROBOT);

    // get initial state of gripper
    joint_model_group_gripper_ =
        move_group_gripper_->getCurrentState()->getJointModelGroup(
            PLANNING_GROUP_GRIPPER);

    // print out basic system information
    RCLCPP_INFO(LOGGER, "Planning Frame: %s",
                move_group_robot_->getPlanningFrame().c_str());
    RCLCPP_INFO(LOGGER, "End Effector Link: %s",
                move_group_robot_->getEndEffectorLink().c_str());
    RCLCPP_INFO(LOGGER, "Available Planning Groups:");
    std::vector<std::string> group_names =
        move_group_robot_->getJointModelGroupNames();
    // more efficient method than std::copy() method used in the docs
    for (long unsigned int i = 0; i < group_names.size(); i++) {
      RCLCPP_INFO(LOGGER, "Group %ld: %s", i, group_names[i].c_str());
    }

    // get current state of robot
    current_state_robot_ = move_group_robot_->getCurrentState(10);
    current_state_robot_->copyJointGroupPositions(joint_model_group_robot_,
                                                  joint_group_positions_robot_);

    // get current state of gripper
    current_state_gripper_ = move_group_gripper_->getCurrentState(10);
    current_state_gripper_->copyJointGroupPositions(
        joint_model_group_gripper_, joint_group_positions_gripper_);

    // set start state of robot to current state
    move_group_robot_->setStartStateToCurrentState();

    // set start state of gripper to current state
    move_group_gripper_->setStartStateToCurrentState();

    // indicate initialization
    RCLCPP_INFO(LOGGER, "Class Initialized: Approach Retreat Trajectory");
  }

  ~ApproachRetreatTrajectory() {
    // indicate termination
    RCLCPP_INFO(LOGGER, "Class Terminated: Approach Retreat Trajectory");
  }

  void execute_trajectory_plan() {

    // setup the home pose target
    RCLCPP_INFO(LOGGER, "Preparing Home Trajectory...");
    setup_named_pose("home", PLANNING_GROUP_ROBOT);
    RCLCPP_INFO(LOGGER, "Planning Home Trajectory...");
    plan_trajectory(PLANNING_GROUP_ROBOT);
    RCLCPP_INFO(LOGGER, "Home Trajectory Planning Complete");
    RCLCPP_INFO(LOGGER, "Executing Home Trajectory...");
    execute_trajectory(PLANNING_GROUP_ROBOT);
    RCLCPP_INFO(LOGGER, "Home Trajectory Complete");

    // setup the move towards object target
    RCLCPP_INFO(LOGGER, "Preparing move near object Trajectory...");
    setup_goal_pose_target(+0.343, +0.132, +0.264, -1.000, +0.000, +0.000,
                           +0.000);
    // plan and execute the trajectory
    RCLCPP_INFO(LOGGER, "Planning move near object Trajectory...");
    plan_trajectory(PLANNING_GROUP_ROBOT);
    RCLCPP_INFO(LOGGER, "Move near object Trajectory Planning Complete");
    RCLCPP_INFO(LOGGER, "Executing move near object Trajectory...");
    execute_trajectory(PLANNING_GROUP_ROBOT);
    RCLCPP_INFO(LOGGER, "Move near object Trajectory Complete");

    // Open the gripper
    RCLCPP_INFO(LOGGER, "Preparing to open the gripper...");
    setup_named_pose("gripper_open", PLANNING_GROUP_GRIPPER);
    RCLCPP_INFO(LOGGER, "Planning opening the gripper...");
    plan_trajectory(PLANNING_GROUP_GRIPPER);
    RCLCPP_INFO(LOGGER, "Gripper open Planning Complete");
    RCLCPP_INFO(LOGGER, "Executing gripper open Trajectory...");
    execute_trajectory(PLANNING_GROUP_GRIPPER);
    RCLCPP_INFO(LOGGER, "Gripper open Trajectory Complete");

    // setup the approach to target
    RCLCPP_INFO(LOGGER, "Preparing Approach Trajectory...");
    target_pose_robot_.position.z = target_pose_robot_.position.z - delta_;
    move_group_robot_->setPoseTarget(target_pose_robot_);
    // plan and execute the trajectory
    RCLCPP_INFO(LOGGER, "Planning Approach Trajectory...");
    plan_trajectory(PLANNING_GROUP_ROBOT);
    RCLCPP_INFO(LOGGER, "Approach Trajectory Planning Complete");
    RCLCPP_INFO(LOGGER, "Executing Approach Trajectory...");
    execute_trajectory(PLANNING_GROUP_ROBOT);
    RCLCPP_INFO(LOGGER, "Approach Trajectory Complete");

    // Closing the gripper
    RCLCPP_INFO(LOGGER, "Preparring to close gripper...");
    setup_joint_value_gripper(+0.500);
    // plan and execute the trajectory
    RCLCPP_INFO(LOGGER, "Planning gripper close Trajectory...");
    plan_trajectory(PLANNING_GROUP_GRIPPER);
    RCLCPP_INFO(LOGGER, "Gripper Close Planning Complete");
    RCLCPP_INFO(LOGGER, "Executing Gripper Close Trajectory...");
    execute_trajectory(PLANNING_GROUP_GRIPPER);
    RCLCPP_INFO(LOGGER, "Gripper Close Trajectory Complete");

    // setup the retreat from target
    RCLCPP_INFO(LOGGER, "Preparing Retreat Trajectory...");
    target_pose_robot_.position.z = target_pose_robot_.position.z + delta_;
    move_group_robot_->setPoseTarget(target_pose_robot_);
    // plan and execute the trajectory
    RCLCPP_INFO(LOGGER, "Planning Retreat Trajectory...");
    plan_trajectory(PLANNING_GROUP_ROBOT);
    RCLCPP_INFO(LOGGER, "Retreat Trajectory Planning Complete");
    RCLCPP_INFO(LOGGER, "Executing Retreat Trajectory...");
    execute_trajectory(PLANNING_GROUP_ROBOT);
    RCLCPP_INFO(LOGGER, "Retreat Trajectory Complete");
  }

private:
  // using shorthand for lengthy class references
  using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
  using JointModelGroup = moveit::core::JointModelGroup;
  using RobotStatePtr = moveit::core::RobotStatePtr;
  using Plan = MoveGroupInterface::Plan;
  using Pose = geometry_msgs::msg::Pose;

  // declare rclcpp base node class
  rclcpp::Node::SharedPtr base_node_;

  // declare move_group node
  rclcpp::Node::SharedPtr move_group_node_;

  // declare single threaded executor for move_group node
  rclcpp::executors::SingleThreadedExecutor executor_;

  // declare move_group_interface variables for robot and gripper
  std::shared_ptr<MoveGroupInterface> move_group_robot_;
  std::shared_ptr<MoveGroupInterface> move_group_gripper_;

  // declare joint_model_group for robot and gripper
  const JointModelGroup *joint_model_group_robot_;
  const JointModelGroup *joint_model_group_gripper_;

  // declare trajectory planning variables for robot
  std::vector<double> joint_group_positions_robot_;
  RobotStatePtr current_state_robot_;
  Plan kinematics_trajectory_plan_;
  Pose target_pose_robot_;
  bool plan_success_robot_ = false;

  // declare trajectory planning variables for gripper
  std::vector<double> joint_group_positions_gripper_;
  RobotStatePtr current_state_gripper_;
  Plan gripper_trajectory_plan_;
  bool plan_success_gripper_ = false;

  // How much to move arm in approach/retreat
  double delta_ = 0.04;

  void setup_goal_pose_target(float pos_x, float pos_y, float pos_z,
                              float quat_x, float quat_y, float quat_z,
                              float quat_w) {
    // set the pose values for end effector of robot arm
    target_pose_robot_.position.x = pos_x;
    target_pose_robot_.position.y = pos_y;
    target_pose_robot_.position.z = pos_z;
    target_pose_robot_.orientation.x = quat_x;
    target_pose_robot_.orientation.y = quat_y;
    target_pose_robot_.orientation.z = quat_z;
    target_pose_robot_.orientation.w = quat_w;
    move_group_robot_->setPoseTarget(target_pose_robot_);
  }

  void setup_joint_value_gripper(float angle) {
    // set the joint values for each joint of gripper
    // based on values provided
    joint_group_positions_gripper_[2] = angle;
    move_group_gripper_->setJointValueTarget(joint_group_positions_gripper_);
  }

  void plan_trajectory(const std::string &group_name) {

    if (group_name == PLANNING_GROUP_GRIPPER) {
      // plan the gripper action
      plan_success_gripper_ =
          (move_group_gripper_->plan(gripper_trajectory_plan_) ==
           moveit::core::MoveItErrorCode::SUCCESS);

    } else if (group_name == PLANNING_GROUP_ROBOT) {
      // plan the trajectory to target using kinematics
      plan_success_robot_ =
          (move_group_robot_->plan(kinematics_trajectory_plan_) ==
           moveit::core::MoveItErrorCode::SUCCESS);
    } else {
      RCLCPP_ERROR(LOGGER, "Unknown planning group: %s", group_name.c_str());
    }
  }

  void execute_trajectory(const std::string &group_name) {

    if (group_name == PLANNING_GROUP_GRIPPER) {
      // execute the planned gripper action
      if (plan_success_gripper_) {
        move_group_gripper_->execute(gripper_trajectory_plan_);
        RCLCPP_INFO(LOGGER, "Gripper Action Command Success !");
      } else {
        RCLCPP_INFO(LOGGER, "Gripper Action Command Failed !");
      }
    } else if (group_name == PLANNING_GROUP_ROBOT) {
      // execute the planned trajectory to target using kinematics
      if (plan_success_robot_) {
        move_group_robot_->execute(kinematics_trajectory_plan_);
        RCLCPP_INFO(LOGGER, "Robot Kinematics Trajectory Success !");
      } else {
        RCLCPP_INFO(LOGGER, "Robot Kinematics Trajectory Failed !");
      }
    } else {
      RCLCPP_ERROR(LOGGER, "Unknown planning group: %s", group_name.c_str());
    }
  }

  void setup_named_pose(const std::string &pose_name,
                        const std::string &group_name) {
    // set the joint values for each joint of gripper
    // based on predefined pose names
    if (group_name == PLANNING_GROUP_GRIPPER) {
      move_group_gripper_->setNamedTarget(pose_name);
    } else if (group_name == PLANNING_GROUP_ROBOT) {
      move_group_robot_->setNamedTarget(pose_name);
    } else {
      RCLCPP_ERROR(LOGGER, "Unknown planning group: %s", group_name.c_str());
    }
  }

}; // class ApproachRetreatTrajectory

int main(int argc, char **argv) {

  // initialize program node
  rclcpp::init(argc, argv);

  // initialize base_node as shared pointer
  std::shared_ptr<rclcpp::Node> base_node =
      std::make_shared<rclcpp::Node>("goal_pose_trajectory");

  // instantiate class
  ApproachRetreatTrajectory approach_retreat_trajectory(base_node);

  // execute trajectory plan
  approach_retreat_trajectory.execute_trajectory_plan();

  // shutdown ros2 node
  rclcpp::shutdown();

  return 0;
}

// End of Code
