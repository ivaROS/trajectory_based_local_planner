#include <trajectory_based_local_planner/trajectory_base_local_planner.h>
#include <trajectory_based_local_planner/pips_ray_local_planner_ros.h>
// #include <Eigen/Core>
#include <cmath>

#include <ros/console.h>

#include <pluginlib/class_list_macros.h>

// #include <base_local_planner/goal_functions.h>
#include <nav_msgs/Path.h>

//register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(trajectory_based_local_planner::PipsRayLocalPlannerROS, trajectory_based_local_planner::TrajectoryBaseLocalPlanner<turtlebot_trajectory_testing::pips_trajectory_ptr>);

namespace trajectory_based_local_planner {

//   void PipsRayLocalPlannerROS::reconfigureCB(DWAPlannerConfig &config, uint32_t level) {
//       if (setup_ && config.restore_defaults) {
//         config = default_config_;
//         config.restore_defaults = false;
//       }
//       if ( ! setup_) {
//         default_config_ = config;
//         setup_ = true;
//       }
// 
//       // update generic local planner params
//       base_local_planner::LocalPlannerLimits limits;
//       limits.max_trans_vel = config.max_trans_vel;
//       limits.min_trans_vel = config.min_trans_vel;
//       limits.max_vel_x = config.max_vel_x;
//       limits.min_vel_x = config.min_vel_x;
//       limits.max_vel_y = config.max_vel_y;
//       limits.min_vel_y = config.min_vel_y;
//       limits.max_rot_vel = config.max_rot_vel;
//       limits.min_rot_vel = config.min_rot_vel;
//       limits.acc_lim_x = config.acc_lim_x;
//       limits.acc_lim_y = config.acc_lim_y;
//       limits.acc_lim_theta = config.acc_lim_theta;
//       limits.acc_limit_trans = config.acc_limit_trans;
//       limits.xy_goal_tolerance = config.xy_goal_tolerance;
//       limits.yaw_goal_tolerance = config.yaw_goal_tolerance;
//       limits.prune_plan = config.prune_plan;
//       limits.trans_stopped_vel = config.trans_stopped_vel;
//       limits.rot_stopped_vel = config.rot_stopped_vel;
//       planner_util_.reconfigureCB(limits, config.restore_defaults);
// 
//       // update dwa specific configuration
//       dp_->reconfigure(config);
//   }

    PipsRayLocalPlannerROS::PipsRayLocalPlannerROS() : 
        setup_(false),
        initialized_(false)
    {
    }

    void PipsRayLocalPlannerROS::initialize(
        std::string name, 
        boost::shared_ptr<turtlebot_trajectory_testing::NIConfigUtility>& ni_util, 
        GenAndTest_ptr& traj_tester) 
    {
        name_ = name;
        if (! isInitialized()) 
        {
        
            ros::NodeHandle nh(name);
            ros::NodeHandle pnh("~/" + name);
            
            ni_util_ = ni_util;
            traj_tester_ = traj_tester;

            initialized_ = true;
            
            tfBuffer_ = std::make_shared<tf2_ros::Buffer>();
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);
            
            local_goal_pub_ = nh.advertise<geometry_msgs::PoseStamped>("local_goal", 1);
            gaps_pub_ = nh.advertise<visualization_msgs::Marker>("gaps", 1);
            chosen_gap_pub_ = nh.advertise<visualization_msgs::Marker>("chosen_gaps", 1);
            
            std::string egocircle_topic = "/point_scan";
            std::string inflated_egocircle_topic = "/inflated_point_scan";
            pnh.getParam("egocircle_topic", egocircle_topic);
            pnh.setParam("egocircle_topic", egocircle_topic);
            pnh.getParam("inflated_egocircle_topic", inflated_egocircle_topic);
            pnh.setParam("inflated_egocircle_topic", inflated_egocircle_topic);
            
            ego_sub_.subscribe ( nh, egocircle_topic, 50 );
            inflated_ego_sub_.subscribe ( nh, inflated_egocircle_topic, 50 );
            synced_ego_.reset ( new ego_synchronizer ( ego_synchronizer ( 50 ), ego_sub_, inflated_ego_sub_) );
            synced_ego_->registerCallback ( bind ( &PipsRayLocalPlannerROS::egocircleCB, this, _1, _2) );
            
            pips_local_planner_ = boost::make_shared<pips_ray_local_planner::PipsRayLocalPlanner>(name);
            
            base_frame_id_ = "base_link";
            global_frame_id_ = "map";
            pnh.getParam("base_frame_id", base_frame_id_);
            pnh.setParam("base_frame_id", base_frame_id_);
            pnh.getParam("global_frame_id", global_frame_id_);
            pnh.setParam("global_frame_id", global_frame_id_);
            
            pips_local_planner_->init(base_frame_id_, global_frame_id_);
            
            sample_tf_ = 3;
            
            // local, gap params
            double local_goal_angle_range = 360.;
            double gap_angle_range = 360.;
            double min_gap_dist = 0.3, angleDiff = 90.;
            
            // ni traj sample params
            int num_paths = 5;
            double v_des = 0.3, path_limits = 0.6;
            double straight_weight = 0.2;
            double alpha = 0.4;
            double samples_angle_limit = 60.;
            
            pnh.getParam("sample_tf", sample_tf_);
            pnh.getParam("local_goal_angle_range", local_goal_angle_range);
            pnh.getParam("gap_angle_range", gap_angle_range);
            pnh.getParam("min_gap_dist", min_gap_dist);
            pnh.getParam("angleDiff", angleDiff);
            pnh.getParam("num_paths", num_paths);
            pnh.getParam("v_des", v_des);
            pnh.getParam("path_limits", path_limits);
            pnh.getParam("straight_weight", straight_weight);
            pnh.getParam("alpha", alpha);
            pnh.getParam("samples_angle_limit", samples_angle_limit);
            
            pnh.setParam("sample_tf", sample_tf_);
            pnh.setParam("local_goal_angle_range", local_goal_angle_range);
            pnh.setParam("gap_angle_range", gap_angle_range);
            pnh.setParam("min_gap_dist", min_gap_dist);
            pnh.setParam("angleDiff", angleDiff);
            pnh.setParam("num_paths", num_paths);
            pnh.setParam("v_des", v_des);
            pnh.setParam("path_limits", path_limits);
            pnh.setParam("straight_weight", straight_weight);
            pnh.setParam("alpha", alpha);
            pnh.setParam("samples_angle_limit", samples_angle_limit);
            
            pips_ray_local_planner::pips_ray_param params(
                local_goal_angle_range, gap_angle_range, min_gap_dist, angleDiff,
                num_paths, v_des, path_limits,
                straight_weight, alpha, samples_angle_limit
            );
            
            pips_local_planner_->setParams(params);

            v_max_ = 0.5;
            w_max_ = 4;
            a_max_ = .55;
            w_dot_max_ = 1.78;

            pnh.getParam("v_max", v_max_);
            pnh.getParam("w_max", w_max_);
            pnh.getParam("a_max", a_max_);
            pnh.getParam("w_dot_max", w_dot_max_);

            pnh.setParam("v_max", v_max_);
            pnh.setParam("w_max", w_max_);
            pnh.setParam("a_max", a_max_);
            pnh.setParam("w_dot_max", w_dot_max_);

    //         dsrv_ = new dynamic_reconfigure::Server<DWAPlannerConfig>(private_nh);
    //         dynamic_reconfigure::Server<DWAPlannerConfig>::CallbackType cb = boost::bind(&DWAPlannerROS::reconfigureCB, this, _1, _2);
    //         dsrv_->setCallback(cb);
        }
        else{
            ROS_WARN("This planner has already been initialized, doing nothing.");
        }
    }
    
    void PipsRayLocalPlannerROS::initialize(
        std::string name) 
    {
        name_ = name;
        ROS_ERROR_STREAM_NAMED(name_, "Not valid for this local planner plugin.");
    }

    bool PipsRayLocalPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& local_global_plan) {
        if (! isInitialized()) {
            ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
            return false;
        }
        
        if(!pips_local_planner_->egocircle_registered_)
        {
            ROS_ERROR("Egocircle is not received.");
            return false;
        }
        //when we get a new plan, we also want to clear any latch we may have on goal tolerances

        ROS_INFO("Got new plan");
//         std::vector<geometry_msgs::PoseStamped> local_plan;
        nav_msgs::Path local_path;
        local_path.header = local_global_plan[0].header;
        local_path.poses = local_global_plan;
        
//         try
//         {
//             local_path = tfBuffer_->transform(global_path, base_frame_id_, ros::Time::now(), global_frame_id_);
//         }
//         catch (tf2::TransformException &ex) 
//         {
//             //ROS_WARN_STREAM("Error transforming path: from [" << input_path_msg.header.frame_id << "](" << input_path_msg.header.stamp << ") to " << Controller::base_frame_id_ << "](" <<cc_wrapper_->getCurrentHeader().stamp << "): "  << ex.what());
//             ROS_WARN_STREAM("Error transforming path: from [" << orig_global_plan[0].header.frame_id << "](" << orig_global_plan[0].header.stamp << ") to [" << base_frame_id_ << "](" << ros::Time::now() << "): " << ex.what());
//             
//             return false;
//         }
        
        got_gap_ = pips_local_planner_->run(local_path);
        
        if(got_gap_)
        {
            if(local_goal_pub_.getNumSubscribers() != 0)
            {
                local_goal_pub_.publish(pips_local_planner_->local_goal_);
            }
            
            if(gaps_pub_.getNumSubscribers() != 0)
            {
                std::vector<float> color_vec{0, 1, 0};
                publishGapMarkers(pips_local_planner_->getGapList(), gaps_pub_, color_vec);
            }
            
            if(chosen_gap_pub_.getNumSubscribers() != 0)
            {
                std::vector<float> color_vec{1, 1, 0};
                std::vector<pips_ray_local_planner::Gap> pub_gap;
                pub_gap.push_back(pips_local_planner_->local_final_gap_);
                publishGapMarkers(pub_gap, chosen_gap_pub_, color_vec);
            }
        }
        
//         return dp_->setPlan(orig_global_plan);
        return true;
    }

//     bool PipsRayLocalPlannerROS::isGoalReached() {
//         if (! isInitialized()) {
//         ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
//         return false;
//         }
//         if ( ! costmap_ros_->getRobotPose(current_pose_)) {
//         ROS_ERROR("Could not get robot pose");
//         return false;
//         }
// 
//         if(latchedStopRotateController_.isGoalReached(&planner_util_, odom_helper_, current_pose_)) {
//         ROS_INFO("Goal reached");
//         return true;
//         } else {
//         return false;
//         }
//     }

//     void DWAPlannerROS::publishLocalPlan(std::vector<geometry_msgs::PoseStamped>& path) {
//         base_local_planner::publishPlan(path, l_plan_pub_);
//     }
// 
// 
//     void DWAPlannerROS::publishGlobalPlan(std::vector<geometry_msgs::PoseStamped>& path) {
//         base_local_planner::publishPlan(path, g_plan_pub_);
//     }

    void PipsRayLocalPlannerROS::egocircleCB(const sensor_msgs::LaserScan::ConstPtr& egocircle, const sensor_msgs::LaserScan::ConstPtr& inflated_egocircle)
    {
        egocircle_ = egocircle;
        inflated_egocircle_ = inflated_egocircle;
        pips_local_planner_->updateEgocircle(egocircle, inflated_egocircle);
    }
    
    void PipsRayLocalPlannerROS::publishGapMarkers(const std::vector<pips_ray_local_planner::Gap>& gap_list, ros::Publisher& publisher, std::vector<float> color_vec)
    {
//         visualization_msgs::MarkerArray markers;
        
        visualization_msgs::Marker marker;
        marker.header.frame_id = base_frame_id_;
        marker.header.stamp = inflated_egocircle_->header.stamp;
        marker.ns = "gap";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        
        marker.scale.x = 0.1;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = color_vec[0];
        marker.color.g = color_vec[1];
        marker.color.b = color_vec[2];
        
        for(pips_ray_local_planner::Gap gap : gap_list)
        {
            geometry_msgs::Point sp, ep;
            
            gap.getPointsList(pips_local_planner_->getInfaltedEgocircleMax(), sp, ep);
            marker.points.push_back(sp);
            marker.points.push_back(ep);
        }
        
        publisher.publish(marker);
        
    }

    PipsRayLocalPlannerROS::~PipsRayLocalPlannerROS()
    {
        //make sure to clean things up
    //     delete dsrv_;
    }


    turtlebot_trajectory_testing::pips_trajectory_ptr PipsRayLocalPlannerROS::generateLocalTrajectory(nav_msgs::Odometry::ConstPtr odom)
    {
        double tf = sample_tf_;
        
        auto traj_funcs = pips_local_planner_->getTrajSamples(got_gap_, pips_local_planner_->local_final_gap_, pips_local_planner_->local_goal_, tf);
        
        ros::WallTime start_ni = ros::WallTime::now();
        
        std::vector<traj_func_ptr> trajs(traj_funcs.size());
      
        double v_max = v_max_;
        double w_max = w_max_;     // 4
        double a_max = a_max_;
        double w_dot_max = w_dot_max_;  // 1.78
        
//         turtlebot_trajectory_generator::near_identity ni(1,5,1,.01,v_max,w_max,a_max,w_dot_max);
        turtlebot_trajectory_generator::near_identity ni(1,5,1,.01,v_max,w_max,a_max,w_dot_max);
        
        
        for(size_t i = 0; i < traj_funcs.size(); i++)
        {
            traj_func_ptr traj = std::make_shared<traj_func_type>(ni);
            traj->setTrajFunc(traj_funcs[i]);
            
            trajs[i] = traj;
        }
        
        trajectory_generator::traj_params_ptr params = std::make_shared<trajectory_generator::traj_params>();
        params->tf=tf;
        
        ROS_DEBUG_STREAM_NAMED(name_, "Generate ni timing:" << (ros::WallTime::now() - start_ni).toSec());
        //             std::vector<traj_func_ptr> trajectory_functions = TurtlebotObstacleAvoidanceController::getTrajectoryFunctions();
        auto valid_trajs = traj_tester_->run(trajs, odom, params);
        
        ROS_DEBUG_STREAM_NAMED(name_, "Found " << valid_trajs.size() << " non colliding  trajectories");
        
        auto chosen_traj = pips_local_planner_->chooseExecutedPath(valid_trajs);
        
        return chosen_traj;
    }


}


