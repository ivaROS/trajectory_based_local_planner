#ifndef TRAJECTORY_BASED_LOCAL_PLANNER_UTILS_H
#define TRAJECTORY_BASED_LOCAL_PLANNER_UTILS_H

#include <geometry_msgs/PoseStamped.h>
#include <math.h>

namespace trajectory_based_local_planner
{
    namespace utils
    {
        double getLocalPoseDist(geometry_msgs::PoseStamped p)
        {
            double dist_x = p.pose.position.x;
            double dist_y = p.pose.position.y;
                
            return sqrt(dist_x*dist_x + dist_y*dist_y);
        }
        
        double getLocalPoseAngle(geometry_msgs::PoseStamped p)
        {
            return atan2(p.pose.position.y, p.pose.position.x);
        }
        
        double getPoseDiff(const geometry_msgs::PoseStamped pose_a, const geometry_msgs::PoseStamped pose_b)
        {
            double x_diff = pose_a.pose.position.x - pose_b.pose.position.x;
            double y_diff = pose_a.pose.position.y - pose_b.pose.position.y;
            double dist = sqrt(x_diff*x_diff + y_diff*y_diff);
            
            return dist;
        }
        
        template <typename T> int sgn(T val) {
            return (T(0) < val) - (val < T(0));
        }
    }
}


#endif
