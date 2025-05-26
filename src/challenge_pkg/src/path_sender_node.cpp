#include "challenge_pkg/path_sender.h"
#include <unistd.h>

//Main function to test the PathTrackingController
//Send a path 
//Send Pause (false) command to initiate the tracking
//Wait 10 seconds
//Send Pause (true) command to pause the tracking
//Wait 10 seconds
//Send Pause (false) command to initiate the tracking
int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_sender");

    PathSender pathSender;

    nav_msgs::Path path = pathSender.generatePathFromCSV();

    auto result = pathSender.setPath(path);
    if (!result)
    {
        return -1;
    }
    
    pathSender.sendPause(false);

    sleep(10);

    pathSender.sendPause(true);
  
    sleep(5);
    
    pathSender.sendPause(false);

    return 0;
}