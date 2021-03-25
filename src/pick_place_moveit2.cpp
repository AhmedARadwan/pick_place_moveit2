#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_cpp_demo");

class MoveItCppDemo
{
public:
  MoveItCppDemo(const rclcpp::Node::SharedPtr& node)
    : node_(node)
    , robot_state_publisher_(node_->create_publisher<moveit_msgs::msg::DisplayRobotState>("display_robot_state", 1))
  {
  }

  void run()
  {
    RCLCPP_INFO(LOGGER, "Initialize MoveItCpp");
    moveit_cpp_ = std::make_shared<moveit::planning_interface::MoveItCpp>(node_);
    moveit_cpp_->getPlanningSceneMonitor()->providePlanningSceneService();  // let RViz display query PlanningScene
    moveit_cpp_->getPlanningSceneMonitor()->setPlanningScenePublishingFrequency(100);

    RCLCPP_INFO(LOGGER, "Initialize PlanningComponent");
    moveit::planning_interface::PlanningComponent arm("panda_arm", moveit_cpp_);

    // A little delay before running the plan
    rclcpp::sleep_for(std::chrono::seconds(3));

    // Create collision object, planning shouldn't be too easy
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = "panda_link0";
    collision_object.id = "box";

    shape_msgs::msg::SolidPrimitive box;
    box.type = box.BOX;
    box.dimensions = { 0.1, 0.4, 0.1 };

    geometry_msgs::msg::Pose box_pose;
    box_pose.position.x = 0.4;
    box_pose.position.y = 0.0;
    box_pose.position.z = 0.8;

    collision_object.primitives.push_back(box);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;
    
    shape_msgs::msg::SolidPrimitive box2;
    box2.type = box.BOX;
    box2.dimensions = { 0.1, 0.4, 0.1 };

    geometry_msgs::msg::Pose box2_pose;
    box2_pose.position.x = 0.2;
    box2_pose.position.y = 0.0;
    box2_pose.position.z = 1.0;

    collision_object.primitives.push_back(box2);
    collision_object.primitive_poses.push_back(box2_pose);
    collision_object.operation = collision_object.ADD;

    // Add object to planning scene
    {  // Lock PlanningScene
      planning_scene_monitor::LockedPlanningSceneRW scene(moveit_cpp_->getPlanningSceneMonitor());
      scene->processCollisionObjectMsg(collision_object);
    }  // Unlock PlanningScene

    // Set joint state goal
    RCLCPP_INFO(LOGGER, "Set goal");
    
    geometry_msgs::msg::PoseStamped target_pose1;
    target_pose1.header.frame_id = "panda_link0";
    target_pose1.pose.orientation.w = 1.0;
    target_pose1.pose.position.x = 0.28;
    target_pose1.pose.position.y = -0.2;
    target_pose1.pose.position.z = 0.5;

    arm.setGoal(target_pose1, "panda_link8");
    
    // arm.setGoal(goal, std::string("end_effector"));  // "extended"

    // Run actual plan
    RCLCPP_INFO(LOGGER, "Plan to goal");
    const auto plan_solution = arm.plan();
    if (plan_solution)
    {
      RCLCPP_INFO(LOGGER, "arm.execute()");
      arm.execute();
    }
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<moveit_msgs::msg::DisplayRobotState>::SharedPtr robot_state_publisher_;
  moveit::planning_interface::MoveItCppPtr moveit_cpp_;
};

int main(int argc, char** argv)
{
  RCLCPP_INFO(LOGGER, "Initialize node");
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  // This enables loading undeclared parameters
  // best practice would be to declare parameters in the corresponding classes
  // and provide descriptions about expected use
  node_options.automatically_declare_parameters_from_overrides(true);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("run_moveit_cpp", "", node_options);

  MoveItCppDemo demo(node);
  std::thread run_demo([&demo]() {
    // Let RViz initialize before running demo
    // TODO(henningkayser): use lifecycle events to launch node
    rclcpp::sleep_for(std::chrono::seconds(5));
    demo.run();
  });

  rclcpp::spin(node);
  run_demo.join();

  return 0;
}
