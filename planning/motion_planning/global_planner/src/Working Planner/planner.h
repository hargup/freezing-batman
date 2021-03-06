#ifndef _PLANNER_H_
#define _PLANNER_H_
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cvaux.hpp>
#include <opencv/cxcore.h>
#include <cv.h>
#include <highgui.h>
#include <iostream>
#include <time.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "devices.h"
// #include "Utils/SerialPortLinux/serial_lnx.h"
//using namespace cv;
//#define MATDATA(img,x,y,n) img.at<cv::Vec3b>(x,y)[n]
#define LEFT_CMD 0
#define RIGHT_CMD 1

#define MAP_MAX 1000
#define LOOP_RATE 10
#define WAIT_TIME 100

extern char local_map[1000][1000];
extern cv::Mat map_img;
extern int ol_overflow;
//extern geometry_msgs::Twist precmdvel;
extern int last_cmd;

typedef struct Triplet {
    int x, y, z;
} Triplet;

typedef struct TripletFP {
    double x, y, z;
} TripletFP;

typedef struct Pose {
    Triplet position;
    TripletFP orientation; // Roll - Pitch - Yaw
} Pose;

typedef struct LatLong {
    double latitude;
    double longitude;
} LatLong;

typedef struct Odom {
    double left_velocity;
    double right_velocity;
} Odom;


namespace planner_space {

    typedef struct command {
        int left_velocity, right_velocity;
    } command;

    class Planner {
    public:
        //        static  ros::NodeHandle nh;
        //
        //
        //     static ros::Publisher vel_pub;

        static void loadPlanner(char** argv);
        static std::vector<Triplet> findPath(Triplet bot, Triplet target, cv::Mat map_img);
        // static geometry_msgs::Twist findPathDT(Triplet bot, Triplet target, cv::Mat map_img);
        static void finBot();
    };
}

#endif
