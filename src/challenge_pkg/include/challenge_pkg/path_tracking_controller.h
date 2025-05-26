#ifndef PATH_TRACKING_CONTROLLER_H
#define PATH_TRACKING_CONTROLLER_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <tf/tf.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib/server/simple_action_server.h>
#include <challenge_pkg/PauseAction.h>
#include <challenge_pkg/SetPath.h>
#include <challenge_pkg/pure_pursuit.h>
#include <vector>
#include <string>
#include <cmath>

enum class PathTrackingState {
    IDLE,
    PAUSED,
    PAUSING,
    TRACKING,
    REACHED,
    ERROR
};

class PathTrackingController {
public:
    PathTrackingController();

private:
    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_;
    ros::Publisher cmd_pub_;
    ros::Publisher state_pub_;
    ros::ServiceServer path_service_;

    actionlib::SimpleActionServer<challenge_pkg::PauseAction> as_;
    challenge_pkg::PauseFeedback pause_action_feedback_;
    challenge_pkg::PauseResult pause_action_result_;

    PurePursuit controller_;
    PathTrackingState state_;

    nav_msgs::Path path_;
    geometry_msgs::Point current_position_;
    geometry_msgs::Vector3 current_linear_twist_;
    float current_orientation_;

    float comfort_deceleration_;
    float comfort_steering_angle_velocity_;
    float max_speed_;
    float max_steering_angle_;
    float wheelbase_;
    float lookahead_distance_;

    int last_path_point_idx_;

    bool setPathService(challenge_pkg::SetPath::Request &req, challenge_pkg::SetPath::Response &res);
    void executePauseAction();
    void odomCallback(const nav_msgs::Odometry::ConstPtr& odom);

    std::vector<Point> navPath2VectorPoints(const nav_msgs::Path& path);
    std::string stateToString(PathTrackingState state);
    bool switchState(PathTrackingState state);
    void sendAckermannCommand(float steering_angle, float steering_angle_velocity, float speed, float acceleration);
    void hasReached();
    void hasPaused();
};

#endif 
