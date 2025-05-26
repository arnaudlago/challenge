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
#include <map> 
#include <iostream>
#include <challenge_pkg/path_tracking_controller.h>
#include <challenge_pkg/pure_pursuit.h>


PathTrackingController::PathTrackingController() :
     as_(nh_, "path_tracking_controller", false),
     state_(PathTrackingState::IDLE)
{
    nh_.param<float>("/path_tracking_controller/comfort_deceleration", comfort_deceleration_, 3.0);
    nh_.param<float>("/path_tracking_controller/comfort_steering_angle_velocity", comfort_steering_angle_velocity_, 0.0);
    nh_.param<float>("/path_tracking_controller/max_speed", max_speed_, 10.0);
    nh_.param<float>("/path_tracking_controller/max_steering_angle", max_steering_angle_, 0.5);
    nh_.param<float>("/path_tracking_controller/wheelbase", wheelbase_, 1.75);
    nh_.param<float>("/path_tracking_controller/lookahead_distance", lookahead_distance_, 5.0);

    controller_ = PurePursuit(lookahead_distance_, wheelbase_);

    as_.registerGoalCallback(boost::bind(&PathTrackingController::executePauseAction, this));

    odom_sub_ = nh_.subscribe("/odometry", 1, &PathTrackingController::odomCallback, this);
    cmd_pub_ = nh_.advertise<ackermann_msgs::AckermannDrive>("/ackermann_cmd", 1);
    state_pub_ = nh_.advertise<std_msgs::String>("/path_tracking_controller/state", 1);
    path_service_ = nh_.advertiseService("/path_tracking_controller/set_path", &PathTrackingController::setPathService, this);

    as_.start();
}

bool PathTrackingController::setPathService(challenge_pkg::SetPath::Request &req, challenge_pkg::SetPath::Response  &res) 
{
    ROS_INFO("New path received");

    //Prevent to replace the current path by an empty one while tracking
    if (req.path.poses.empty() && state_ == PathTrackingState::TRACKING) 
    {
        ROS_WARN("Received an empty path while TRACKING, request failed.");
        res.success = false;
        return false;
    }
    bool result = controller_.SetPath(navPath2VectorPoints(req.path));
    if (result)
    {   
        switchState(PathTrackingState::PAUSED);
    }
    else
    {
        ROS_WARN("Path received is invalid");
        switchState(PathTrackingState::IDLE);
    }

    res.success = result;
    return result;
}

void PathTrackingController::executePauseAction() 
{
    if (as_.acceptNewGoal()->pause) 
    {
        ROS_INFO("Pause message received");

        bool result = switchState(PathTrackingState::PAUSING);

        pause_action_feedback_.status = "PAUSING";
        
        as_.publishFeedback(pause_action_feedback_);
    } 
    else 
    {
        ROS_INFO("Track message received");

        pause_action_feedback_.status = "TRACKING";

        switchState(PathTrackingState::TRACKING);

        as_.publishFeedback(pause_action_feedback_);

        pause_action_result_.success = true;

        as_.setSucceeded(pause_action_result_);
    }
    
}

void PathTrackingController::odomCallback(const nav_msgs::Odometry::ConstPtr& odom) 
{
    current_position_ = odom->pose.pose.position;

    current_orientation_ = tf::getYaw(odom->pose.pose.orientation);

    geometry_msgs::Vector3 current_linear_twist = odom->twist.twist.linear;

    // Set PathTrackingState to REACHED if true
    hasReached();

    // Set PathTrackingState to PAUSED if true and sends pause action result.
    // If PAUSING send pause action feedback
    hasPaused();

    if (state_ == PathTrackingState::IDLE ||
        state_ == PathTrackingState::PAUSED ||
        state_ == PathTrackingState::ERROR ||
        state_ == PathTrackingState::REACHED)
    {
        sendAckermannCommand(0, 0, 0, 0);
        return;
    }

    float cmd_steering = controller_.ComputeSteeringAngle(last_path_point_idx_,
        current_position_.x, current_position_.y, current_orientation_, max_steering_angle_);

    float cmd_speed = controller_.ComputeSpeed(last_path_point_idx_,
        current_position_.x, current_position_.y, comfort_deceleration_, max_speed_);

    if (state_ == PathTrackingState::PAUSING)
    {
        cmd_speed  = 0.0;
    }

    sendAckermannCommand(cmd_steering, comfort_steering_angle_velocity_, cmd_speed, comfort_deceleration_);
}


// Converts nav_msgs::Path to a std::vector<Point> 
// std::vector<Point> format is used by the pure_pursuit library
std::vector<Point> PathTrackingController::navPath2VectorPoints (const nav_msgs::Path& path)
{
    std::vector<Point> result;
    for (size_t i = 0; i < path.poses.size(); i++)
    {
        Point point = {path.poses[i].pose.position.x, path.poses[i].pose.position.y};
        result.push_back(point);
    }
    return result;
}

// Convertd the PathTrackingState enum type into a string 
// Used for logging
std::string PathTrackingController::stateToString(PathTrackingState state)
{
    std::string result;

    switch (state)
    {
        case PathTrackingState::ERROR:
            result = "ERROR";
            break;
        case PathTrackingState::IDLE:
            result = "IDLE";
            break;
        case PathTrackingState::PAUSED:
            result = "PAUSED";
            break;
        case PathTrackingState::PAUSING:
            result = "PAUSING";
            break;
        case PathTrackingState::TRACKING:
            result = "TRACKING";
            break;
        case PathTrackingState::REACHED:
            result = "REACHED";
            break;
        
        default:
            result = "ERROR";
            break;
    }
        
    return result;
}

// Switches the PathTrackingState to a new one
// act as simple state machine logic for the node
bool PathTrackingController::switchState(PathTrackingState state)
{
    auto result = false;
    switch (state)
    {
        case PathTrackingState::ERROR:
            if (state_ == PathTrackingState::PAUSING ||
                state_ == PathTrackingState::TRACKING) { result = true; }
            break;
        case PathTrackingState::IDLE:
            if (state_ != PathTrackingState::IDLE ) { result = true; }
            break;
        case PathTrackingState::PAUSED:
            if (state_ == PathTrackingState::PAUSING ||
                state_ == PathTrackingState::IDLE ||
                state_ == PathTrackingState::REACHED) { result = true; }                
            break;
        case PathTrackingState::PAUSING:
            if (state_ == PathTrackingState::TRACKING) { result = true; }  
            break;
        case PathTrackingState::TRACKING:
            if (state_ == PathTrackingState::PAUSED) { result = true; }  
            break;
        case PathTrackingState::REACHED:
            if (state_ == PathTrackingState::PAUSING ||
                state_ == PathTrackingState::TRACKING) { result = true; }
            break;
        
        default:
            break;
    }
    if (result)
    {
        ROS_INFO("SWitching State from %s to %s", stateToString(state_).c_str(),stateToString(state).c_str());
        state_ = state;

        std_msgs::String str;
        str.data = stateToString(state_);
        state_pub_.publish(str);
    }

    return result;
}

// Publishes the AckermannCommand
void PathTrackingController::sendAckermannCommand(float steering_angle, float steering_angle_velocity, float speed, float acceleration)
{
        ackermann_msgs::AckermannDrive cmd_msg;
        cmd_msg.steering_angle = steering_angle;
        cmd_msg.steering_angle_velocity = steering_angle_velocity;
        cmd_msg.speed = speed;
        cmd_msg.acceleration = acceleration;
        cmd_msg.jerk = 0;
        cmd_pub_.publish(cmd_msg);
}

// Verifies if the vehicle has reached the end of the trajectory
// - if yes, the PathTrackingState is switched to REACHED by checking 
// if the nearest path point is the last of the trajectory or of the remaining distance if inferior to a threshold
// if the vehicle reach the end of the trajectory while PAUSING, the setPause command  is considered Failed by design.
void PathTrackingController::hasReached()
{
    //Check if destination is reaced
    if (state_ == PathTrackingState::PAUSING ||
        state_ == PathTrackingState::TRACKING)
    {
        auto idx = controller_.FindNearestPathPointIndex(last_path_point_idx_, current_position_.x, current_position_.y);
        last_path_point_idx_ = idx;

        auto remaining_distance = controller_.ComputeRemainingDistance(last_path_point_idx_);

        if (last_path_point_idx_ == path_.poses.size() - 1 || remaining_distance < 0.05)
        {
            //Destination reached
            switchState(PathTrackingState::REACHED);

            // If Pause request active consider it as sucessfull
            if (as_.isActive())
            {
                pause_action_result_.success = false;
                as_.setSucceeded(pause_action_result_);
            }
        }
    }
}

// Verifies if the vehicle is PAUSED, the vehicle is considered PAUSED if in PAUSING State 
// and linear speed inferior to a threshold.
// if the conditions are met, the PathTrackingState is switched to PAUSED
void PathTrackingController::hasPaused()
{
    //Check if pause action if finished by checking the vehicle is stopped
    if (state_ == PathTrackingState::PAUSING)
    {
        //compute linearspeed
        auto lvx = current_linear_twist_.x;
        auto lvy = current_linear_twist_.y;
        auto lvz = current_linear_twist_.z;

        auto linear_speed =  sqrt(pow(lvx, 2) + pow(lvy, 2) + pow(lvz, 2));
    
        if (linear_speed < 0.1)
        {            
            switchState(PathTrackingState::PAUSED);

            if (as_.isActive())
            {
                pause_action_result_.success = true;
                as_.setSucceeded(pause_action_result_);
            }
        }
        else if (as_.isActive())
        {
                pause_action_feedback_.status = "PAUSING";
                as_.publishFeedback(pause_action_feedback_);
        }

    }
}