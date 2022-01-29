#include <ros/ros.h>
#include <vector>
#include <nav_msgs/OccupancyGrid.h>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/stitching.hpp>
#include <opencv2/opencv.hpp>

namespace mapstitcher {

class mapStitcher{
private:
    ros::NodeHandle nh_;
    ros::Subscriber s1_;
    ros::Subscriber s2_;
    ros::Publisher p_;
    grid_map::GridMap m1_;
    grid_map::GridMap m2_;
    grid_map::GridMap m_;
    cv_bridge::CvImage img1_;
    cv_bridge::CvImage img2_;
    cv::Stitcher::Mode mode = cv::Stitcher::Mode::SCANS;
    ros::Timer stitching_timer;
    double frequency;
    std::string map1_name;
    std::string map2_name;

    const int MAX_FEATURES;
    const float GOOD_MATCH_PERCENT;
    
public:
    mapStitcher();
    ~mapStitcher();
    void map1Callback(const nav_msgs::OccupancyGrid &msg);
    void map2Callback(const nav_msgs::OccupancyGrid &msg);
    void applyStitching ();

};

} //namespace