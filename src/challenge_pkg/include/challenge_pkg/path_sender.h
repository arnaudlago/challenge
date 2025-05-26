#ifndef PATH_SENDER_H
#define PATH_SENDER_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <challenge_pkg/SetPath.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <challenge_pkg/PauseAction.h>
#include <string>

class PathSender {
public:
    PathSender();

    bool setPath(nav_msgs::Path path);
    nav_msgs::Path generatePathFromCSV();
    bool sendPause(bool pause);
    bool checkPath(nav_msgs::Path path);

private:
    const std::string serciceName = "/path_tracking_controller/set_path";

    ros::NodeHandle nh_;
    ros::ServiceClient client_;
    actionlib::SimpleActionClient<challenge_pkg::PauseAction> ac_;
    std::string path_file_;
};

#endif  // PATH_SENDER_H
