#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/ColorRGBA.h"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

class ROSNode
{
public:
  ROSNode(ros::NodeHandle &_n) {
    n = &_n;
  };

  ros::NodeHandle *n;
};
ROSNode *rn;

ros::Publisher redImg_pub;
float th[2] = {0, 255};
namespace enc = sensor_msgs::image_encodings;

void callback_image(const sensor_msgs::Image& msg);
void callback_depth(const sensor_msgs::Image& msg);
cv::Mat findColor(const cv::Mat & inputBGRimage, int rng);
cv::Mat findLines(const cv::Mat & img);
void inRange(const cv::Mat& _in, cv::Mat *_out);
void callback_th(const std_msgs::ColorRGBA& msg);

int main(int argc, char **argv) {
    ros::init(argc, argv, "agribot_redline_detection");
    ros::NodeHandle nn;
    rn = new ROSNode(nn);

    ros::Subscriber sub_image = rn->n->subscribe("/camera/color/image_raw", 1, callback_image);
    ros::Subscriber sub_depth = rn->n->subscribe("/camera/depth/image_rect_raw", 1, callback_depth);
    ros::Subscriber sub_th = rn->n->subscribe("/hsv", 1, callback_th);
    redImg_pub = rn->n->advertise<sensor_msgs::Image>("/agribot/redline", 100);
    
    ROS_INFO("agribot_redline_detection : start");
    ros::Rate loop_rate(30);
    while(ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
    ROS_INFO("agribot_redline_detection : end");
}

void callback_depth(const sensor_msgs::Image& msg) {
    //ROS_INFO("callback_depth : start");
    //sensor_msgs::ImageConstPtr xx = &msg;
    //sensor_msgs::ImageConstPtr xx(&msg);
    ROS_INFO("D : timestamp : %d.%d", msg.header.stamp.sec, msg.header.stamp.nsec);
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, "16UC1");
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Process cv_ptr->image using OpenCV

    //ROS_INFO("callback_depth : end");
}

void callback_image(const sensor_msgs::Image& msg) {
    //ROS_INFO("callback_image : start");
    //sensor_msgs::ImageConstPtr xx = &msg;
    //sensor_msgs::ImageConstPtr xx(&msg);
    ROS_INFO("I : timestamp : %d.%d", msg.header.stamp.sec, msg.header.stamp.nsec);
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Process cv_ptr->image using OpenCV
    cv_ptr->image = findColor(cv_ptr->image, 15);

    // cv_ptr->image = findLine(cv_ptr->image);

    // cv_bridge::CvImage out_msg;
    // out_msg.header   = cv_ptr->header; // Same timestamp and tf frame as input image
    // out_msg.encoding = enc::BGR8; // Or whatever
    // out_msg.image    = cv_ptr->image; // Your cv::Mat

    redImg_pub.publish(cv_ptr->toImageMsg());
    //ROS_INFO("callback_image : end");
}

cv::Mat findColor(const cv::Mat & inputBGRimage, int rng=15)
{
    // Make sure that your input image uses the channel order B, G, R (check not implemented).
    cv::Mat input = inputBGRimage.clone();
    cv::Mat imageHSV;//(input.rows, input.cols, CV_8UC3);
    cv::Mat imgThreshold, imgThreshold0, imgThreshold1;//(input.rows, input.cols, CV_8UC1);
    assert( ! input.empty() );

    // convert input-image to HSV-image
    cv::blur(input, input, cv::Size(5,5));
    cv::cvtColor( input, imageHSV, cv::COLOR_BGR2HSV );
 
    // In the HSV-color space the color 'red' is located around the H-value 0 and also around the
    // H-value 180. That is why you need to threshold your image twice and the combine the results.
    cv::inRange(imageHSV, cv::Scalar(th[0], 25, 25, 0), cv::Scalar(th[1], 255, 255, 0), imgThreshold);
    inRange(imageHSV, &input);

    imgThreshold = findLines(imgThreshold);
    cv::cvtColor(imgThreshold, imgThreshold, cv::COLOR_GRAY2RGB);
    return imgThreshold;

    // if ( rng > 0 )
    // {
    //     cv::inRange(imageHSV, cv::Scalar(180-rng, 53, 185, 0), cv::Scalar(180, 255, 255, 0), imgThreshold1);
    //     cv::bitwise_or( imgThreshold0, imgThreshold1, imgThreshold );
    // }
    // else
    // {
    //     imgThreshold = imgThreshold0;
    // }

    // return imgThreshold;
}

void inRange(const cv::Mat& _in, cv::Mat *_out) {
    for(int row=0; row<_in.rows; ++row) {
        for(int col=0; col<_in.cols; ++col) {
            //_in.at<unsigned char>(row, col);
            cv::Vec3b hsv = _in.at<cv::Vec3b>(row, col);
            if( hsv[0] >= th[0] && hsv[0] <= th[1] &&
                hsv[1] >=  25 && hsv[1] <= 255 &&
                hsv[2] >=  25 && hsv[2] <= 255 ) {
                cv::Vec3b &color = _out->at<cv::Vec3b>(row, col);
                color[0] = 0;
                color[1] = 0;
                color[2] = 255;
            }
        }
    }
}
cv::Mat findLines(const cv::Mat & img) {
    // int lowThreshold = 1;
    // const int ratio = 3;
    // const int kernel_size = 7;
    // cv::Canny(img, img, lowThreshold, lowThreshold*ratio, kernel_size );
    cv::Mat dst(img.rows, img.cols, img.channels(), cv::Scalar(0, 0, 0));
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(img, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    int mmax = -1, indx;
    for(int i=0; i<contours.size(); i++) {
        double currentArea = cv::contourArea(contours[i]);
        if(currentArea > mmax) {
            mmax = currentArea;
            indx = i;
        }
    }
    cv::Scalar color(255, 255, 255);
    // cv::drawContours(dst, contours, indx, color, cv::FILLED, 8, hierarchy);
    return dst;
}

void callback_th(const std_msgs::ColorRGBA& msg) {
    th[0] = msg.r;
    th[1] = msg.g;
}
