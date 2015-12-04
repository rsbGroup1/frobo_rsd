// Includes
#include <ros/ros.h>
#include <cstdlib>
#include <string>
#include <iostream>
#include <sstream>
#include <ostream>
#include <fstream>
#include <time.h>
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "mr_main/run.h"

std::ofstream logFile;

void logCallback(std_msgs::String msgs)
{
    //Determine if the msgs should be written
    if(msgs.data.size() > 0 && (msgs.data[0] == '1' || msgs.data[0] == '2' || msgs.data[0] == '3') )
    {
        // write timestamp
        time_t rawtime;
        std::tm timeinfo;
        time (&rawtime);
        timeinfo = *(localtime (&rawtime));
        logFile << (1900+timeinfo.tm_year) << "-" << 1+timeinfo.tm_mon << "-" << timeinfo.tm_mday << " "
                 << timeinfo.tm_hour << ":" << timeinfo.tm_min << ":" << timeinfo.tm_sec;
        // write msgs
        for(int i = 4; i < msgs.data.size()-1; i++)
        {
            logFile << msgs.data[i];
        }
        logFile << "\n";
        // write to file -> flush
        logFile.flush();
    }
    return;
}

int main()
{
    // Setup ROS Arguments
    char** argv = NULL;
    int argc = 0;

    // Init ROS Node
    ros::init (argc, argv, "MR_Log");
    ros::NodeHandle nh;
    ros::NodeHandle pNh ("~");

    // Topic names
    std::string logSub, logDir;

    pNh.param<std::string> ("log_sub", logSub, "/mrHMI/status");
    pNh.param<std::string> ("log_dir", logDir, "/tmp/");


    // Subscriber
     ros::Subscriber logSubscriber =  nh.subscribe<std_msgs::String> (logSub, 20,logCallback);
     // open log file if not existing
     std::cout << "Log Path: " << logDir << "MR_Log.txt" << std::endl;
     logFile.open((logDir+"MR_Log.txt").c_str(),std::ios_base::app);

     logFile << "************************ STARTUP **************************\n";


    // ROS Spin: Handle callbacks
     ros::Rate rate(20);
    while(ros::ok())
    {
        ROS_INFO("spinning");
        ros::spinOnce();
        rate.sleep();
    }


     logFile.close();
    // Return
    return 0;
}
