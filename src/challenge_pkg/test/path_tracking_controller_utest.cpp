#include <gtest/gtest.h>
#include <ros/ros.h>
#include <ros/service_client.h>
#include <challenge_pkg/SetPath.h>
#include <std_srvs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <std_msgs/String.h>
#include <actionlib/client/simple_action_client.h>
#include <challenge_pkg/PauseAction.h>

// Global test node handle
ros::NodeHandle* nh_ = nullptr;
std::vector<geometry_msgs::PoseStamped> path_;
ros::Publisher odom_pub_;

// Helper function to wait for a ROS topic to have a message
template<typename T>
T waitForMessage(const std::string& topic, const ros::Duration& timeout = ros::Duration(1.0)) {
    auto msg = ros::topic::waitForMessage<T>(topic, timeout);
    if (!msg) {
        throw std::runtime_error("No message received on topic: " + topic);
    }
    return *msg;
}

// Test fixture class
class PathTrackingControllerTest : public ::testing::Test {
protected:
    // Called before every test case
    void SetUp() override {
        // Initialize ROS client connections
        path_client_ = nh_->serviceClient<challenge_pkg::SetPath>("/path_tracking_controller/set_path");
        pause_action_client_.reset(new actionlib::SimpleActionClient<challenge_pkg::PauseAction>("path_tracking_controller", true));
        odom_pub_ = nh_->advertise<nav_msgs::Odometry>("/odometry", 1);

        // Wait for the action server to start
        ASSERT_TRUE(pause_action_client_->waitForServer(ros::Duration(2.0)));

        // Ensure the node is running
        ASSERT_TRUE(ros::service::exists("/path_tracking_controller/set_path", true));
    }

    // Helper method to send a simple path
    bool sendPath(const std::vector<geometry_msgs::PoseStamped>& poses) {
        challenge_pkg::SetPath srv;
        for (const auto& pose : poses) {
            srv.request.path.poses.push_back(pose);
        }
        return path_client_.call(srv) && srv.response.success;
    }

    ros::ServiceClient path_client_;
    std::unique_ptr<actionlib::SimpleActionClient<challenge_pkg::PauseAction>> pause_action_client_;
};

// Test case: Set a valid path
TEST_F(PathTrackingControllerTest, TestSetPath) {
    // Send the path
    EXPECT_EQ(sendPath(path_), true);

    std::vector<geometry_msgs::PoseStamped> emptyPath;
    EXPECT_EQ(sendPath(emptyPath), false);
}

TEST_F(PathTrackingControllerTest, TestAckermannTopicExists)
{
    // Check if the "/test/arnaud" topic exists
    ros::master::V_TopicInfo master_topics;
    ros::master::getTopics(master_topics);

    bool topic_exists = false;
    for (const auto& topic_info : master_topics)
    {
        if (topic_info.name == "/ackermann_cmd")
        {
            topic_exists = true;
            break;
        }
    }

    EXPECT_TRUE(topic_exists);
}

int main(int argc, char** argv) {
    
    geometry_msgs::PoseStamped pose1, pose2, pose3, pose4;
    pose1.pose.position.x = 0.0;
    pose1.pose.position.y = 0.0;
    pose2.pose.position.x = 1.0;
    pose2.pose.position.y = 1.0;
    pose3.pose.position.x = 2.0;
    pose3.pose.position.y = 2.0;
    pose4.pose.position.x = 3.0;
    pose4.pose.position.y = 3.0;
    path_.push_back(pose1);
    path_.push_back(pose2);
    path_.push_back(pose3);
    path_.push_back(pose4);
    
    ros::init(argc, argv, "path_tracking_controller_utest");
    ros::NodeHandle nh;
    nh_ = &nh;

    testing::InitGoogleTest(&argc, argv);

    int result = RUN_ALL_TESTS();

    ros::shutdown();
    
    return result;
}
