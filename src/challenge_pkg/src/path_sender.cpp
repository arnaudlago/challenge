#include "challenge_pkg/path_sender.h"
#include <fstream>
#include <sstream>

    

PathSender::PathSender() :
    ac_("path_tracking_controller", true),
    client_(nh_.serviceClient<challenge_pkg::SetPath>(serciceName)),
    path_file_("")
{
    nh_.param<std::string>("/path_sender/path", path_file_, "");

    ros::service::waitForService(serciceName);

    ac_.waitForServer();

    ROS_INFO("PathSender Initialized");
}

//Sends the path specified to the PathTrackingController and
//wait for its response.
bool PathSender::setPath(nav_msgs::Path path)
{
    ROS_INFO("Send Path request");
    // Create the service request
    challenge_pkg::SetPath srv;

    // Assign the path to the service request
    srv.request.path = path;
    // Call the service
    if (client_.call(srv))
    {
        if (srv.response.success)
        {
            ROS_INFO("Path sent successfully.");
            return true;
        }
        else
        {
            ROS_WARN("Failed to set path.");
            return false;
        }
    }
    else
    {
        ROS_WARN("Failed to call service.");
        return false;
    }
}

//Read CSV file and retrun a Path
//Returns an empty path is the file is not valid
nav_msgs::Path PathSender::generatePathFromCSV()
{
    nav_msgs::Path path;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = "map"; // Adjust the frame ID as necessary

    std::ifstream file(path_file_);
    if (!file.is_open())
    {
        ROS_ERROR("Failed to open file: %s", path_file_.c_str());
        return path; // Return empty path
    }

    std::string line;
    while (std::getline(file, line))
    {
        std::stringstream ss(line);
        std::string token;

        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = "map";

        // Read position (x, y, z)
        std::getline(ss, token, ',');
        pose.pose.position.x = std::stod(token);

        std::getline(ss, token, ',');
        pose.pose.position.y = std::stod(token);

        std::getline(ss, token, ',');
        pose.pose.position.z = std::stod(token);

        // Add the pose to the path
        path.poses.push_back(pose);
    }

    file.close();
    ROS_INFO("Path generated");
    return path;
}

//Send Pause action command to the PathTrackingController and wait for its completion
//Since Pausing action can take time due to decceleration of the vehicle,
//the timeout duration is long.
bool PathSender::sendPause(bool pause)
{
    if (pause)
    { 
        ROS_INFO("Send Pause request"); 
    }
    else 
    { 
        ROS_INFO("Send Track request"); 
    }

    challenge_pkg::PauseGoal track;
    track.pause = pause;
    ac_.sendGoal(track);

    //wait for the action to return
    bool finished_before_timeout = ac_.waitForResult(ros::Duration(30.0));
        
    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = ac_.getState();
        ROS_INFO("Action finished: %s",state.toString().c_str());
        return true;
    }
    else
    {
        ROS_WARN("Action did not finish before the time out.");
        ac_.cancelGoal();
        return false;
    }
}

bool PathSender::checkPath(nav_msgs::Path path)
{
    return !(path.poses.empty() || path.poses.size() < 2);
}


