#ifndef PIPS_RAY_LOCAL_PLANNER_ROS_H_
#define PIPS_RAY_LOCAL_PLANNER_ROS_H_

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <sensor_msgs/LaserScan.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <tf2_ros/transform_listener.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_path_msg/tf2_path_msg.h>

// #include <tf/transform_listener.h>
// 
// #include <dynamic_reconfigure/server.h>
// #include <dwa_local_planner/DWAPlannerConfig.h>
// 
// #include <angles/angles.h>
// 
// #include <nav_msgs/Odometry.h>
// 
// #include <costmap_2d/costmap_2d_ros.h>
// #include <nav_core/base_local_planner.h>
// #include <base_local_planner/latched_stop_rotate_controller.h>
// 
// #include <base_local_planner/odometry_helper_ros.h>
// 
// #include <dwa_local_planner/dwa_planner.h>

#include <trajectory_based_local_planner/trajectory_base_local_planner.h>
#include <trajectory_based_local_planner/pips_ray_local_planner.h>

namespace trajectory_based_local_planner {
  /**
   * @class PipsRayLocalPlannerROS
   * @brief ROS Wrapper for the PipsRayLocalPlannerROS that adheres to the
   * TrajectoryBaseLocalPlanner interface and can be used as a plugin for move_base.
   */
  class PipsRayLocalPlannerROS : public trajectory_based_local_planner::TrajectoryBaseLocalPlanner<turtlebot_trajectory_testing::pips_trajectory_ptr> {
    public:
        typedef TurtlebotGenAndTest::traj_func_type traj_func_type;
        typedef TurtlebotGenAndTest::traj_func_ptr traj_func_ptr;
        
        /**
        * @brief  Constructor for PipsRayLocalPlannerROS wrapper
        */
        PipsRayLocalPlannerROS();

        /**
        * @brief  Constructs the ros wrapper
        * @param name The name to give this instance of the trajectory planner
        * @param ni_util A pointer to a near identity util
        * @param traj_tester A pointer to the pips trajectory tester
        */
        void initialize(std::string name, boost::shared_ptr<turtlebot_trajectory_testing::NIConfigUtility>& ni_util, GenAndTest_ptr& traj_tester);
        
        void initialize(std::string name);

        /**
        * @brief  Destructor for the wrapper
        */
        ~PipsRayLocalPlannerROS();

    //       /**
    //        * @brief  Given the current position, orientation, and velocity of the robot,
    //        * compute velocity commands to send to the base
    //        * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
    //        * @return True if a valid trajectory was found, false otherwise
    //        */
    //       bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);
        
        turtlebot_trajectory_testing::pips_trajectory_ptr generateLocalTrajectory(nav_msgs::Odometry::ConstPtr odom);

        /**
        * @brief  Set the plan that the controller is following
        * @param local_global_plan The plan to pass to the controller
        * @return True if the plan was updated successfully, false otherwise
        */
        bool setPlan(const std::vector<geometry_msgs::PoseStamped>& local_global_plan);

    //       /**
    //        * @brief  Check if the goal pose has been achieved
    //        * @return True if achieved, false otherwise
    //        */
    //       bool isGoalReached();
        bool trajectoryCheck() { return true; }
        
        void reset() { }


        bool isInitialized() {
            return initialized_;
        }
        
        void egocircleCB(const sensor_msgs::LaserScan::ConstPtr& egocircle, const sensor_msgs::LaserScan::ConstPtr& inflated_egocircle);
        
        void publishGapMarkers(const std::vector<pips_ray_local_planner::Gap>& gap_list, ros::Publisher& publisher, std::vector<float> color_vec);

    private:
        double sample_tf_;
//         ros::NodeHandle nh_, pnh_;
        std::string name_;
        std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        
        std::string base_frame_id_, global_frame_id_;

        double v_max_, w_max_, a_max_, w_dot_max_;
        
        message_filters::Subscriber<sensor_msgs::LaserScan> ego_sub_, inflated_ego_sub_;
        // Change to approximate time
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, sensor_msgs::LaserScan> ego_sync_policy;
        typedef message_filters::Synchronizer<ego_sync_policy> ego_synchronizer;
        boost::shared_ptr<ego_synchronizer> synced_ego_;
        
        sensor_msgs::LaserScan::ConstPtr egocircle_, inflated_egocircle_;
        
        boost::shared_ptr<pips_ray_local_planner::PipsRayLocalPlanner> pips_local_planner_;
        
        geometry_msgs::PoseStamped local_goal_;
        bool got_gap_;
//       /**
//        * @brief Callback to update the local planner's parameters based on dynamic reconfigure
//        */
//       void reconfigureCB(DWAPlannerConfig &config, uint32_t level);

//         void publishLocalPlan(std::vector<geometry_msgs::PoseStamped>& path);
// 
//         void publishGlobalPlan(std::vector<geometry_msgs::PoseStamped>& path);

        // for visualisation, publishers of global and local plan
        ros::Publisher local_goal_pub_, gaps_pub_, chosen_gap_pub_;
            
        
        
    //       dynamic_reconfigure::Server<DWAPlannerConfig> *dsrv_;
    //       dwa_local_planner::DWAPlannerConfig default_config_;
        bool setup_;
    //       tf::Stamped<tf::Pose> current_pose_;
    // 
    //       base_local_planner::LatchedStopRotateController latchedStopRotateController_;


        bool initialized_;


//       base_local_planner::OdometryHelperRos odom_helper_;
//       std::string odom_topic_;
  };
};
#endif
