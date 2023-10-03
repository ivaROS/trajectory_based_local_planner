#ifndef TRAJECTORY_BASED_BASE_LOCAL_PLANNER_H
#define TRAJECTORY_BASED_BASE_LOCAL_PLANNER_H

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>

#include <turtlebot_trajectory_generator/near_identity.h>
#include <trajectory_generator_ros_interface.h>

#include <turtlebot_trajectory_testing/obstacle_avoidance_controller.h>
// #include <turtlebot_trajectory_testing/ni_config_utility.h>
#include <turtlebot_trajectory_testing/turtlebot_trajectory_tester.h>
#include <pips_trajectory_testing/pips_trajectory_tester.h>

namespace trajectory_based_local_planner {
  /**
   * @class TrajectoryBaseLocalPlanner
   * @brief Provides an interface for local planners used in navigation. All local planners written as plugins for the navigation stack must adhere to this interface.
   */
  template <typename traj_type_name>
  class TrajectoryBaseLocalPlanner{
    public:
      typedef std::shared_ptr<TurtlebotGenAndTest> GenAndTest_ptr;
      
//       /**
//        * @brief  Given the current position, orientation, and velocity of the robot, compute velocity commands to send to the base
//        * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
//        * @return True if a valid velocity command was found, false otherwise
//        */
//       virtual bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel) = 0;
      
      virtual traj_type_name generateLocalTrajectory(nav_msgs::Odometry::ConstPtr odom) = 0;

//       /**
//        * @brief  Check if the goal pose has been achieved by the local planner
//        * @return True if achieved, false otherwise
//        */
//       virtual bool isGoalReached() = 0;
      virtual bool trajectoryCheck() = 0;

      /**
       * @brief  Set the plan that the local planner is following
       * @param plan The plan to pass to the local planner
       * @return True if the plan was updated successfully, false otherwise
       */
      virtual bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) = 0;

      /**
       * @brief  Constructs the local planner
       * @param name The name to give this instance of the local planner
       * @param ni_util A pointer to a near identity util
       * @param traj_tester A pointer to the pips trajectory tester
       */
      virtual void initialize(std::string name, boost::shared_ptr<turtlebot_trajectory_testing::NIConfigUtility>& ni_util, GenAndTest_ptr& traj_tester) = 0;
      
      virtual void initialize(std::string name) = 0;
      
      virtual void reset() = 0;

      /**
       * @brief  Virtual destructor for the interface
       */
      virtual ~TrajectoryBaseLocalPlanner(){}

    protected:
      TrajectoryBaseLocalPlanner(){}
      
      boost::shared_ptr<turtlebot_trajectory_testing::NIConfigUtility> ni_util_;
      GenAndTest_ptr traj_tester_;
  };
};  // namespace nav_core

#endif  // TRAJECTORY_BASED_BASE_LOCAL_PLANNER_H
