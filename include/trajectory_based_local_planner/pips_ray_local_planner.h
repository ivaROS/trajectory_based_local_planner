#ifndef PIPS_RAY_LOCAL_PLANNER_H
#define PIPS_RAY_LOCAL_PLANNER_H

//#include <dynamic_reconfigure/server.h>
#include <turtlebot_trajectory_generator/near_identity.h>
#include <turtlebot_trajectory_functions/angled_straight.h>
#include <turtlebot_trajectory_functions/circle.h>
#include <turtlebot_trajectory_testing/obstacle_avoidance_controller.h>

#include <trajectory_based_local_planner/utils.h>

#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_path_msg/tf2_path_msg.h>

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

#include <ros/ros.h>
#include <memory>
#include <cmath>
#include <limits>

namespace pips_ray_local_planner
{
    struct pips_ray_param
    {
        // local, gap params
        double local_goal_angle_range_;
        double gap_angle_range_;
        double min_gap_dist_, angleDiff_;
        
        // ni traj sample params
        int num_paths_;
        double v_des_, path_limits_;
        double straight_weight_;
        double alpha_;
        double samples_angle_limit_;
        
        pips_ray_param()
        {
            // local, gap params
            local_goal_angle_range_ = 2*M_PI;
            gap_angle_range_ = 2*M_PI;
            min_gap_dist_ = 0.3, angleDiff_ = M_PI / 2;
            
            // ni traj sample params
            num_paths_ = 5;
            v_des_ = 0.3, path_limits_ = 90.;
            straight_weight_ = 0.2;
            alpha_ = 0.4;
            samples_angle_limit_ = M_PI / 3;
        }
        
        pips_ray_param(
            double local_goal_angle_range,
            double gap_angle_range,
            double min_gap_dist, 
            double angleDiff,
            int num_paths,
            double v_des, 
            double path_limits,
            double straight_weight,
            double alpha,
            double samples_angle_limit
        )
        {
            // local, gap params
            local_goal_angle_range_ = local_goal_angle_range / 180.0 * M_PI ;
            gap_angle_range_ = gap_angle_range / 180.0 * M_PI;
            min_gap_dist_ = min_gap_dist;
            angleDiff_ = angleDiff / 180.0 * M_PI;
            
            // ni traj sample params
            num_paths_ = num_paths;
            v_des_ = v_des; 
            path_limits_ = path_limits;
            straight_weight_ = straight_weight;
            alpha_ = alpha;
            samples_angle_limit_ = samples_angle_limit / 180.0 * M_PI;
        }
    };
    
    struct Gap
    {
        int start_idx_, end_idx_;
        double start_angle_, end_angle_;
        bool found_idx_ = false, found_angle_ = false;
        
        Gap(){}
        
        Gap(double sa, double ea)
        {
            found_angle_ = true;
            start_angle_ = sa;
            end_angle_ = ea;
        }
        
        Gap(int si, int ei)
        {
            found_idx_ = true;
            start_idx_ = si;
            end_idx_ = ei;
        }
        
        void getAngle(double angle_min, double angle_increment)
        {
            if(!found_angle_)
            {
                start_angle_ = start_idx_ * angle_increment + angle_min;
                end_angle_ = end_idx_ * angle_increment + angle_min;
                found_angle_ = true;
            }
        }
        
        void getIdx(double angle_min, double angle_increment, int ranges_size)
        {
            if(!found_idx_)
            {
                start_idx_ = (int) roundf((start_angle_ - angle_min) / angle_increment);
                start_idx_ = (start_idx_ >= 0) ? start_idx_ : 0;
                end_idx_ = (int) roundf((end_angle_ - angle_min) / angle_increment);
                end_idx_ = (end_idx_ < ranges_size) ? end_idx_ : ranges_size - 1;
                found_idx_ = true;
            }
        }
        
        double getGapDist(double range)
        {
            double ang_abs = std::abs(end_angle_ - start_angle_);
            return range * ang_abs;
        }
        
        double getMidAngle()
        {
            return (start_angle_ + end_angle_) / 2;
        }
        
        double getMidAngle() const
        {
            return getMidAngle();
        }
        
        void getPointsList(double range, geometry_msgs::Point& start_pt, geometry_msgs::Point& end_pt)
        {
            float z = 0.3;
            float start_x = range * cos(start_angle_);
            float start_y = range * sin(start_angle_);
            float end_x = range * cos(end_angle_);
            float end_y = range * sin(end_angle_);
            
            start_pt.x = start_x;
            start_pt.y = start_y;
            start_pt.z = z;
            
            end_pt.x = end_x;
            end_pt.y = end_y;
            end_pt.z = z;
        }
    };
    
    
    class PipsRayLocalPlanner
    {
    private:
        
        std::string base_frame_id_;
        std::string global_frame_id_;
        
//         std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
//         std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        
        sensor_msgs::LaserScan::ConstPtr egocircle_, inflated_egocircle_;
        double egocircle_max_ = 0, inflated_egocircle_max_ = 0;
        nav_msgs::Path local_plan_;
        
        // local, gap params
        double local_goal_angle_range_ = 2*M_PI;
        double gap_angle_range_ = 2*M_PI;
        double min_gap_dist_ = 0.3, angleDiff_ = M_PI / 2;
        
        // ni traj sample params
        int num_paths_ = 5;
        double v_des_ = 0.3, path_limits_ = 90. / 180. * M_PI;
        double straight_weight_ = 0.2;
        double alpha_ = 0.4;
        double samples_angle_limit_ = M_PI / 3;
        
    protected:
        std::vector<Gap> gap_list_;
        
        template<typename T>
        T getCenterLongestTrajectory(const std::vector<T>& valid_trajs);
        
        template<typename T>
        T getGoalWeightedTrajectory(const std::vector<T>& valid_trajs);
        
        std::vector<turtlebot_trajectory_generator::desired_traj_func::Ptr> getOrientedAngledStraight();
        
        geometry_msgs::PoseStamped getLocalGoal(const nav_msgs::Path& local_plan);
        
        std::vector<turtlebot_trajectory_generator::desired_traj_func::Ptr> getGapTrajSamples(Gap& local_final_gap, geometry_msgs::PoseStamped& local_goal, double tf);
        
        
        
    public:
        std::string name_;
        bool initialized_;
        
        bool egocircle_registered_;
        
        geometry_msgs::PoseStamped local_goal_;
        Gap local_final_gap_;
        
        PipsRayLocalPlanner(std::string name);
        ~PipsRayLocalPlanner(){}
        
        void init(std::string base_frame_id, std::string global_frame_id);
        
        std::vector<turtlebot_trajectory_generator::desired_traj_func::Ptr> getTrajSamples(bool got_gap, Gap& local_final_gap, geometry_msgs::PoseStamped& local_goal, double tf);
        turtlebot_trajectory_testing::pips_trajectory_ptr chooseExecutedPath(const std::vector<turtlebot_trajectory_testing::pips_trajectory_ptr>& valid_trajs);
        
//         void publishGapMarkers(const std::vector<Gap>& gap_list, ros::Publisher& publisher, std::vector<float> color_vec);
        
        bool run(const nav_msgs::Path& orig_local_global_plan);
        
        bool setPlan(const nav_msgs::Path& orig_local_global_plan, geometry_msgs::PoseStamped& local_goal);
        
        std::vector<Gap> findLocalGaps();
        bool goalWeightedGap(std::vector<Gap>& input_gaps, Gap& output_gap, const geometry_msgs::PoseStamped& local_goal);
        
        void updateEgocircle(const sensor_msgs::LaserScan::ConstPtr egocircle, const sensor_msgs::LaserScan::ConstPtr inflated_egocircle)
        {
            egocircle_registered_ = true;
            
            egocircle_ = egocircle;
            inflated_egocircle_ = inflated_egocircle;
            
            if(egocircle_max_ == 0)
                egocircle_max_ = *std::max_element(egocircle_->ranges.begin(), egocircle_->ranges.end()); 
            
            if(inflated_egocircle_max_ == 0)
                inflated_egocircle_max_ = *std::max_element(inflated_egocircle_->ranges.begin(), inflated_egocircle_->ranges.end()); 
        }
        
        void setParams(pips_ray_param params)
        {
            // local, gap params
            local_goal_angle_range_ = params.local_goal_angle_range_;
            gap_angle_range_ = params.gap_angle_range_;
            min_gap_dist_ = params.min_gap_dist_;
            angleDiff_ = params.angleDiff_;
            
            // ni traj sample params
            num_paths_ = params.num_paths_;
            v_des_ = params.v_des_;
            path_limits_ = params.path_limits_ / 180. * M_PI;
            straight_weight_ = params.straight_weight_;
            alpha_ = params.alpha_;
            samples_angle_limit_ = params.samples_angle_limit_;
        }
        
        double getInfaltedEgocircleMax() { return inflated_egocircle_max_; }
        std::vector<Gap> getGapList() { return gap_list_; }
        
    };
    
}


#endif
