#include<map_stitcher/MapStitcher.hh>

//s21DVPEKR

namespace mapstitcher {

mapStitcher::mapStitcher():nh_("~") {

    nh_.param("frequency", frequency, double(1.0));
    ROS_INFO("Frequency is set to %f", frequency);
    nh_.param<std::string>("map1_name", map1_name, std::string("/map1"));
    nh_.param<std::string>("map2_name", map2_name, std::string("/map2"));
    s1_ = nh_.subscribe(map1_name, 10, &mapStitcher::map1Callback, this);
    s2_ = nh_.subscribe(map2_name, 10, &mapStitcher::map2Callback, this);
    p_ = nh_.advertise<nav_msgs::OccupancyGrid>("/map", 10);
    stitching_timer = nh_.createTimer(ros::Duration(1/frequency), [this](const ros::TimerEvent&){applyStitching();});
}

mapStitcher::~mapStitcher(){
    
}

void mapStitcher::map1Callback(const nav_msgs::OccupancyGrid &msg) {
    grid_map::GridMapRosConverter::fromOccupancyGrid(msg, std::string("layer1"), m1_);
    grid_map::GridMapRosConverter::toCvImage(m1_, std::string("layer1"), sensor_msgs::image_encodings::MONO8, img1_);
}

void mapStitcher::map2Callback(const nav_msgs::OccupancyGrid &msg) {
    grid_map::GridMapRosConverter::fromOccupancyGrid(msg, std::string("layer1"), m2_);
    grid_map::GridMapRosConverter::toCvImage(m2_, std::string("layer1"), sensor_msgs::image_encodings::MONO8, img2_);
}

void mapStitcher::applyStitching () {

    if( !(img1_.image.empty()) && !(img2_.image.empty()) ){
        ROS_INFO("Images will be stitched");
        cv::Mat img;
        std::vector<cv::Mat>  v{img1_.image, img2_.image};
        cv::Ptr<cv::Stitcher> st = cv::Stitcher::create(mode);
        cv::Stitcher::Status status = st->stitch(v, img);

        if(status == cv::Stitcher::Status::OK){
            // grid_map::GridMapRosConverter::initializeFromImage();
            
            cv_bridge::CvImage img_bridge;
            sensor_msgs::Image img_msg; // >> message to be sent
            std_msgs::Header header; // empty header
            // header.seq = counter; // user defined counter
            header.stamp = ros::Time::now(); // time
            img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, img);
            img_bridge.toImageMsg(img_msg); // from cv_bridge to sensor_msgs::Image
            // pub_img.publish(img_msg); // ros::Publisher pub_img = node.advertise<sensor_msgs::Image>("topic", queuesize);

            nav_msgs::OccupancyGrid o;
            grid_map::GridMapRosConverter::initializeFromImage(img_msg, 1.0, m_);
            grid_map::GridMapRosConverter::toOccupancyGrid(m_, "layer1", 0, 255, o);
            p_.publish(o);
        }
    } else {
        ROS_INFO("Images are empty");
    }
}

} //namespace

int main(int argc, char** argv){
    ros::init(argc,argv, "map_stitcher_node");
    ROS_INFO("%s", ros::this_node::getName().c_str());
    mapstitcher::mapStitcher ms;
    ros::spin();
    return 0;
}