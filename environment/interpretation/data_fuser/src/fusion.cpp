#include "data_fuser/fusion.h"

using namespace sensor_msgs;
using namespace message_filters;
static int obstacle_removal_dilation_size = 35; //variable used for dilating and eroding.. to be changed only if dimension of image changes.
void callback(const ImageConstPtr& lane_image, const ImageConstPtr& lidar_image) {
    cv_bridge::CvImagePtr lane_map;
    cv_bridge::CvImagePtr lidar_map;

    try {
        lane_map = cv_bridge::toCvCopy(lane_image, image_encodings::MONO8);
        lidar_map = cv_bridge::toCvCopy(lidar_image, image_encodings::MONO8);
        cv::Mat fusion_map = cv::Mat::zeros(lane_map->image.size(), CV_8UC1);

        for (int i = 0; i < fusion_map.rows; i++) {
            for (int j = 0; j < fusion_map.cols; j++) {
                if (lane_map->image.at<uchar>(i, j) == 255 || lidar_map->image.at<uchar>(i, j) == 255) {
                    fusion_map.at<uchar>(i, j) = 255;
                } else {
                    fusion_map.at<uchar>(i, j) = 0;
                }
            }
        }

        cv_bridge::CvImage message;
        message.encoding = image_encodings::MONO8;
        message.image = fusion_map;
        world_map_publisher.publish(message.toImageMsg());
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

void singleCallback(const ImageConstPtr& image) {
    cv_bridge::CvImagePtr cv_image;
    try {
        cv_image = cv_bridge::toCvCopy(image, image_encodings::MONO8);
        cv_bridge::CvImage message;
        message.encoding = image_encodings::MONO8;
        cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT,
                                                cv::Size(2 * obstacle_removal_dilation_size + 1, 2 * obstacle_removal_dilation_size + 1),
                                                cv::Point(obstacle_removal_dilation_size, obstacle_removal_dilation_size));
        cv::dilate(cv_image->image, cv_image->image, element);
        message.image = cv_image->image;
        world_map_publisher.publish(message.toImageMsg());
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}