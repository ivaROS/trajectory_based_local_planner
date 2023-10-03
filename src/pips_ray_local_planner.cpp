#include <trajectory_based_local_planner/pips_ray_local_planner.h>

#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <math.h>


namespace pips_ray_local_planner
{
    PipsRayLocalPlanner::PipsRayLocalPlanner(std::string name)
    {
        name_ = name;
        
//         tfBuffer_ = std::make_shared<tf2_ros::Buffer>();
//         tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);
    }
    
    void PipsRayLocalPlanner::init(std::string base_frame_id, std::string global_frame_id)
    {
        initialized_ = true;
        egocircle_registered_ = false;
        
        base_frame_id_ = base_frame_id;
        global_frame_id_ = global_frame_id;
        
//         num_paths_ = 5;
//         v_des_ = 0.3;
//         path_limits_ = .6;
    }

    
    template<typename T>
    T PipsRayLocalPlanner::getCenterLongestTrajectory(const std::vector<T>& valid_trajs)
    {
        std::vector<T> longest_trajs;
        
        ros::Duration longest_length;
        for(size_t i=0; i < valid_trajs.size(); i++)
        {
            ros::Duration length = valid_trajs[i]->getDuration();
            if(length > longest_length)
            {
                longest_trajs.clear();
                longest_trajs.push_back(valid_trajs[i]);
                longest_length = length;
            }
            else if(length == longest_length)
            {
                longest_trajs.push_back(valid_trajs[i]);
            }
        }
        
        T longest_traj = longest_trajs[longest_trajs.size()/2];
        
        return longest_traj;
    }
    
    template<typename T>
    T PipsRayLocalPlanner::getGoalWeightedTrajectory(const std::vector<T>& valid_trajs)
    {
        std::vector<T> longest_trajs;
        
        ros::Duration longest_length;
        for(size_t i=0; i < valid_trajs.size(); i++)
        {
            ros::Duration length = valid_trajs[i]->getDuration();
            if(length > longest_length)
            {
                longest_trajs.clear();
                longest_trajs.push_back(valid_trajs[i]);
                longest_length = length;
            }
            else if(length == longest_length)
            {
                longest_trajs.push_back(valid_trajs[i]);
            }
        }
        
        T longest_traj = longest_trajs[0];
        double local_goal_angle = trajectory_based_local_planner::utils::getLocalPoseAngle(local_goal_);
        double traj_angle = atan2(longest_traj->x_vec.back().y, longest_traj->x_vec.back().x);
        double min_angle_diff = std::abs(local_goal_angle - traj_angle);
        
        for(size_t i = 1; i < longest_trajs.size(); i++)
        {
            double cc_angle = atan2(longest_trajs[i]->x_vec.back().y, longest_trajs[i]->x_vec.back().x);
            double cc_angle_diff = std::abs(local_goal_angle - cc_angle);
            
            if(cc_angle_diff <= min_angle_diff)
            {
                longest_traj = longest_trajs[i];
                min_angle_diff = cc_angle_diff;
            }
        }
        
        return longest_traj;
    }
    
    
    std::vector<turtlebot_trajectory_generator::desired_traj_func::Ptr> PipsRayLocalPlanner::getOrientedAngledStraight()
    {
//         int num_paths = 5;
//         double v_des = .15;
//         double path_limits = .8;
        
        //Set trajectory departure angles and speed
        std::vector<double> dep_angles = {-path_limits_/2,path_limits_/2}; //,.6,.8,1,1.2,1.6,2,2.4};
        
        std::vector<turtlebot_trajectory_generator::desired_traj_func::Ptr> trajectory_functions(num_paths_);
        
        for(size_t i = 0; i < num_paths_; i++)
        {
            double dep_angle;
            if(num_paths_ == 1)
            {
                dep_angle = 0;
            }
            else
            {
                dep_angle = dep_angles[0] + i*(dep_angles[1] - dep_angles[0])/(num_paths_ - 1); 
            }
            trajectory_functions[i] = std::make_shared<turtlebot_trajectory_functions::AngledStraight>(dep_angle, v_des_);
        
        }
        return trajectory_functions;
    }
    
    geometry_msgs::PoseStamped PipsRayLocalPlanner::getLocalGoal(const nav_msgs::Path& local_plan)
    {
        geometry_msgs::PoseStamped local_goal;
        
        bool foundLocalGoal = false;
//         double angle_range = 2 * M_PI;
        
        for(size_t i = 0; i < local_plan.poses.size(); i++)
        {
            double dist = trajectory_based_local_planner::utils::getLocalPoseDist(local_plan.poses[i]);
            double angle = trajectory_based_local_planner::utils::getLocalPoseAngle(local_plan.poses[i]);
            int ego_idx = (int) roundf((angle - egocircle_->angle_min) / egocircle_->angle_increment);
            ego_idx = (ego_idx >=0 ) ? ego_idx : 0;
            ego_idx = (ego_idx < egocircle_->ranges.size()) ? ego_idx : egocircle_->ranges.size() - 1;
            
            if(angle < -local_goal_angle_range_/2 || angle > local_goal_angle_range_/2)
                continue;
            
            if(dist < egocircle_->ranges[ego_idx] && dist < egocircle_max_ - 0.5)
            {
                continue;
            }
            else
            {
                if(i == 0)
                {
                    foundLocalGoal = false;
                }
                else
                {
                    foundLocalGoal = true;

                    local_goal = local_plan.poses[i - 1];
                    local_goal.header.frame_id = base_frame_id_;
                    local_goal.header.stamp = ros::Time::now();
                }
                
                break;
            }
        }
        
        if(!foundLocalGoal)
        {
            geometry_msgs::PoseStamped robot_pose;
            tf2::toMsg(tf2::Transform::getIdentity(), robot_pose.pose);
            robot_pose.pose.position.x += 2;
            robot_pose.header.frame_id = base_frame_id_;
            robot_pose.header.stamp = ros::Time::now();
            local_goal = robot_pose;
        }
        
//         if(local_goal_pub_.getNumSubscribers() != 0)
//         {
//             local_goal_pub_.publish(local_goal);
//         }
        
        return local_goal;
    }
    
    bool PipsRayLocalPlanner::setPlan(const nav_msgs::Path& orig_local_global_plan, geometry_msgs::PoseStamped& local_goal)
    {
        if(!initialized_){
            ROS_ERROR("Planner have not been initialized.");
            return false;
        }
//         local_plan_ = orig_local_global_plan;
        
        local_goal = getLocalGoal(orig_local_global_plan);
        
        return true;
    }
    
    std::vector<Gap> PipsRayLocalPlanner::findLocalGaps()
    {
        std::vector<Gap> gap_list;
        
        double start_limit = - gap_angle_range_ / 2;
        double end_limit = gap_angle_range_ / 2;
        int start_idx = (int) roundf((start_limit - inflated_egocircle_->angle_min) / inflated_egocircle_->angle_increment);
        start_idx = (start_idx >= 0) ? start_idx : 0;
        int end_idx = (int) roundf((end_limit - inflated_egocircle_->angle_min) / inflated_egocircle_->angle_increment);
        end_idx = (end_idx < inflated_egocircle_->ranges.size()) ? end_idx : inflated_egocircle_->ranges.size() - 1;
//         ROS_INFO_STREAM(start_idx << " " << end_idx);
        
        int gap_start_idx = -1;
        for(size_t i = start_idx; i <= end_idx; i++)
        {
            double curr_range = inflated_egocircle_->ranges[i];
            if(gap_start_idx < 0 && (curr_range >= inflated_egocircle_max_ || std::isnan(curr_range)))
            {
                gap_start_idx = i;
            }
            else if(gap_start_idx >= 0 && (curr_range < inflated_egocircle_max_ || std::isnan(curr_range)))
            {
                int gap_end_idx = i - 1;
                Gap gap(gap_start_idx, gap_end_idx);
                gap.getAngle(inflated_egocircle_->angle_min, inflated_egocircle_->angle_increment);
                gap_list.push_back(gap);
                gap_start_idx = -1;
            }
            else if(gap_start_idx >= 0 && i == end_idx)
            {
                int gap_end_idx = i;
                Gap gap(gap_start_idx, gap_end_idx);
                gap.getAngle(inflated_egocircle_->angle_min, inflated_egocircle_->angle_increment);
                gap_list.push_back(gap);
                gap_start_idx = -1;
            }
            
        }
        
        std::vector<Gap> finalized_gap;
        
        if(gap_list.size() > 0)
        {
            Gap curr_gap, prev_gap;
            prev_gap = gap_list[0];
            
            double robot_thresh = 0;
            int gap_index_thresh = 3;
            for(size_t i = 1; i < gap_list.size(); i++)
            {
                curr_gap = gap_list[i];
                double gap_dist = curr_gap.getGapDist(inflated_egocircle_max_);
                if(gap_dist < robot_thresh)
                    continue;
                
                if(curr_gap.start_idx_ - prev_gap.end_idx_ > gap_index_thresh)
                {
                    finalized_gap.push_back(prev_gap);
                    prev_gap = curr_gap;
                }
                else
                {
                    Gap merged_gap(prev_gap.start_idx_, curr_gap.end_idx_);
                    merged_gap.getAngle(inflated_egocircle_->angle_min, inflated_egocircle_->angle_increment);
                    prev_gap = merged_gap;
                }
            }
            finalized_gap.push_back(prev_gap);
        }
        ROS_DEBUG_STREAM("Found finalized " << finalized_gap.size() << " gaps from " << gap_list.size() << " gaps. ");
        
        
        
        return finalized_gap;
    }
    
//     void PipsRayLocalPlanner::publishGapMarkers(const std::vector<Gap>& gap_list, ros::Publisher& publisher, std::vector<float> color_vec)
//     {
// //         visualization_msgs::MarkerArray markers;
//         
//         visualization_msgs::Marker marker;
//         marker.header.frame_id = base_frame_id_;
//         marker.header.stamp = inflated_egocircle_->header.stamp;
//         marker.ns = "gap";
//         marker.id = 0;
//         marker.type = visualization_msgs::Marker::LINE_LIST;
//         marker.action = visualization_msgs::Marker::ADD;
//         
//         marker.scale.x = 0.1;
//         marker.color.a = 1.0; // Don't forget to set the alpha!
//         marker.color.r = color_vec[0];
//         marker.color.g = color_vec[1];
//         marker.color.b = color_vec[2];
//         
//         for(Gap gap : gap_list)
//         {
//             geometry_msgs::Point sp, ep;
//             
//             gap.getPointsList(inflated_egocircle_max_, sp, ep);
//             marker.points.push_back(sp);
//             marker.points.push_back(ep);
//         }
//         
//         publisher.publish(marker);
//         
//     }
    
    bool PipsRayLocalPlanner::goalWeightedGap(std::vector<Gap>& input_gaps, Gap& output_gap, const geometry_msgs::PoseStamped& local_goal)
    {
//         double min_gap_dist = 0.3;
        bool foundGap = false;
//         double angleDiff = M_PI / 2;
        
        if(input_gaps.size() == 0)
            return false;
        
        double local_goal_angle = trajectory_based_local_planner::utils::getLocalPoseAngle(local_goal);
        
        output_gap = input_gaps[0];
        double best_angle_diff = std::abs(input_gaps[0].getMidAngle() - local_goal_angle);
        
        for(Gap gap : input_gaps)
        {
            if(gap.getGapDist(inflated_egocircle_max_) < min_gap_dist_)
                continue;
            
            double curr_angle_diff = std::abs(gap.getMidAngle() - local_goal_angle);
            if(curr_angle_diff > angleDiff_ && (local_goal_angle < gap.start_angle_ || local_goal_angle > gap.end_angle_))
                continue;
            
            foundGap = true;
            
            if(curr_angle_diff <= best_angle_diff)
            {
                output_gap = gap;
                best_angle_diff = curr_angle_diff;
            }
        }
        
        
        return foundGap;
        
    }
    
    bool PipsRayLocalPlanner::run(const nav_msgs::Path& orig_local_global_plan)
    {
        ros::WallTime start_local_goal = ros::WallTime::now();
        setPlan(orig_local_global_plan, local_goal_);
        ros::WallTime end_local_goal = ros::WallTime::now();
        ROS_INFO_STREAM("Local goal timing:" << (end_local_goal - start_local_goal).toSec()*1000 << " ms.");
        
        gap_list_.clear();
        gap_list_ = findLocalGaps();
//         if(gaps_pub_.getNumSubscribers() != 0)
//         {
//             std::vector<float> color_vec{0, 1, 0};
//             publishGapMarkers(gap_list, gaps_pub_, color_vec);
//         }
        ros::WallTime end_all_gaps = ros::WallTime::now();
        ROS_INFO_STREAM("Find gaps timing:" << (end_all_gaps - end_local_goal).toSec()*1000 << " ms.");
        
        bool got_gap = goalWeightedGap(gap_list_, local_final_gap_, local_goal_);
//         if(got_gap && chosen_gap_pub_.getNumSubscribers() != 0)
//         {
//             std::vector<float> color_vec{1, 1, 0};
//             std::vector<Gap> pub_gap;
//             pub_gap.push_back(local_final_gap_);
//             publishGapMarkers(pub_gap, chosen_gap_pub_, color_vec);
//         }
        ros::WallTime end_weighted_gap = ros::WallTime::now();
        ROS_DEBUG_STREAM("Weighted goal timing:" << (end_weighted_gap - end_all_gaps).toSec()*1000 << " ms.");
        
        return got_gap;
        
    }
    
    std::vector<turtlebot_trajectory_generator::desired_traj_func::Ptr> PipsRayLocalPlanner::getGapTrajSamples(Gap& local_final_gap, geometry_msgs::PoseStamped& local_goal, double tf)
    {
        double gap_mid_angle = local_final_gap.getMidAngle();
        double half_angle = std::abs(gap_mid_angle - local_final_gap.start_angle_) + 0.2 / inflated_egocircle_max_;
        double local_goal_angle = trajectory_based_local_planner::utils::getLocalPoseAngle(local_goal);
//         double straight_weight = 0.2;
//         double alpha = 0.4;
        double samples_angle = (1 - straight_weight_) * (alpha_ * local_goal_angle + (1 - alpha_) * gap_mid_angle);
        
        ROS_INFO_STREAM("Samples angle before limit: " << samples_angle / M_PI * 180);
        
//         double samples_angle_limit = M_PI / 3;
        samples_angle = (std::abs(samples_angle) <= samples_angle_limit_) ? samples_angle : trajectory_based_local_planner::utils::sgn(samples_angle) * samples_angle_limit_;
        
//         int num_paths = 5;
//         double v_des = 0.2;
//         double path_limits = .6;
        half_angle = (half_angle <= path_limits_) ? half_angle : path_limits_;
        
        double v_des = v_des_;
        if(local_goal_angle < -M_PI/2 || local_goal_angle > M_PI/2)
        {
            v_des /= 2.;
        }
        
        ros::NodeHandle n("~");
        if(samples_angle >= 0)
        {
            n.setParam("limited_angle_rotate_recovery/rotate_direction", 0);
        }
        else
        {
            n.setParam("limited_angle_rotate_recovery/rotate_direction", 1);
        }
        
        ROS_INFO_STREAM("Samples angle: " << samples_angle / M_PI * 180 << ", half angle: " << half_angle / M_PI * 180 << ", path limits: " << path_limits_ / M_PI * 180);
        
        //Set trajectory departure angles and speed
        std::vector<double> dep_angles = {samples_angle - half_angle, samples_angle + half_angle}; //,.6,.8,1,1.2,1.6,2,2.4};
        
        bool isAngledStraight = false;
        
        double traj_length = v_des * tf;
        
        std::vector<turtlebot_trajectory_generator::desired_traj_func::Ptr> trajectory_functions(num_paths_);
        
        for(size_t i = 0; i < num_paths_; i++)
        {
            double dep_angle;
            if(num_paths_ == 1)
            {
                dep_angle = 0;
            }
            else
            {
                dep_angle = dep_angles[0] + i*(dep_angles[1] - dep_angles[0])/(num_paths_ - 1); 
            }
            
            if(isAngledStraight || num_paths_ == 1)
            {
                trajectory_functions[i] = std::make_shared<turtlebot_trajectory_functions::AngledStraight>(dep_angle, v_des);
            }
            else
            {
                // Circled traj
                double circle_angle = 2 * std::abs(dep_angle);
                double r = traj_length / circle_angle;
                if(dep_angle >= 0)
                    r = -r;
                trajectory_functions[i] = std::make_shared<turtlebot_trajectory_functions::Circle>(v_des, r);
            }
        
        }
        return trajectory_functions;
    }
    
    std::vector<turtlebot_trajectory_generator::desired_traj_func::Ptr> PipsRayLocalPlanner::getTrajSamples(bool got_gap, Gap& local_final_gap, geometry_msgs::PoseStamped& local_goal, double tf)
    {
        if(got_gap)
        {
            return getGapTrajSamples(local_final_gap, local_goal, tf);
        }
        else
        {
            return getOrientedAngledStraight();
        }
    }
    
    
    turtlebot_trajectory_testing::pips_trajectory_ptr PipsRayLocalPlanner::chooseExecutedPath(const std::vector<turtlebot_trajectory_testing::pips_trajectory_ptr>& valid_trajs)
    {
        bool goal_weighted_traj = true;
        
        if(valid_trajs.size() >0)
        {
            if(goal_weighted_traj)
            {
                auto chosen_traj = getGoalWeightedTrajectory(valid_trajs);
                
                ROS_INFO_STREAM_NAMED(name_, "Length of chosen trajectory: " << chosen_traj->getDuration());
                
                return chosen_traj;
            }
            else
            {
                auto chosen_traj = getCenterLongestTrajectory(valid_trajs);
                
                ROS_INFO_STREAM_NAMED(name_, "Length of longest trajectory: " << chosen_traj->getDuration());
                
                return chosen_traj;
            }
        }
        
        return nullptr;
    }
    
    
}
