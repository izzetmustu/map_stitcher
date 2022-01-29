#include<map_stitcher/MapStitcher.hh>

//s21DVPEKR

namespace mapstitcher {

mapStitcher::mapStitcher():nh_("~"), MAX_FEATURES(500), GOOD_MATCH_PERCENT(0.15f) {

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
    // grid_map::GridMapRosConverter::fromOccupancyGrid(msg, std::string("layer1"), m1_);
    // grid_map::GridMapRosConverter::toCvImage(m1_, std::string("layer1"), sensor_msgs::image_encodings::MONO8, img1_);
}

void mapStitcher::map2Callback(const nav_msgs::OccupancyGrid &msg) {
    // grid_map::GridMapRosConverter::fromOccupancyGrid(msg, std::string("layer1"), m2_);
    // grid_map::GridMapRosConverter::toCvImage(m2_, std::string("layer1"), sensor_msgs::image_encodings::MONO8, img2_);
}

void mapStitcher::applyStitching () {

    // if( !(img1_.image.empty()) && !(img2_.image.empty()) ){
    //     ROS_INFO("Images will be stitched");
    //     cv::Mat img;
    //     std::vector<cv::Mat>  v{img1_.image, img2_.image};
    //     cv::Ptr<cv::Stitcher> st = cv::Stitcher::create(mode);
    //     cv::Stitcher::Status status = st->stitch(v, img);

    //     if(status == cv::Stitcher::Status::OK){
    //         // grid_map::GridMapRosConverter::initializeFromImage();
            
    //         cv_bridge::CvImage img_bridge;
    //         sensor_msgs::Image img_msg; // >> message to be sent
    //         std_msgs::Header header; // empty header
    //         // header.seq = counter; // user defined counter
    //         header.stamp = ros::Time::now(); // time
    //         img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, img);
    //         img_bridge.toImageMsg(img_msg); // from cv_bridge to sensor_msgs::Image
    //         // pub_img.publish(img_msg); // ros::Publisher pub_img = node.advertise<sensor_msgs::Image>("topic", queuesize);

    //         nav_msgs::OccupancyGrid o;
    //         grid_map::GridMapRosConverter::initializeFromImage(img_msg, 1.0, m_);
    //         grid_map::GridMapRosConverter::toOccupancyGrid(m_, "layer1", 0, 255, o);
    //         p_.publish(o);
    //     }
    // } else {
    //     ROS_INFO("Images are empty");
    // }


    cv::Mat im1 = cv::imread("/home/izzet/catkin_ws/src/map_stitcher/src/1.jpg");
    cv::Mat im2 = cv::imread("/home/izzet/catkin_ws/src/map_stitcher/src/2.jpg");


    // Convert images to grayscale
    cv::Mat im1Gray, im2Gray;
    cvtColor(im1, im1Gray, cv::COLOR_BGR2GRAY);
    cvtColor(im2, im2Gray, cv::COLOR_BGR2GRAY);

    // Variables to store keypoints and descriptors
    std::vector<cv::KeyPoint> keypoints1, keypoints2;
    cv::Mat descriptors1, descriptors2;

    // Detect ORB features and compute descriptors.
    cv::Ptr<cv::Feature2D> orb = cv::ORB::create(MAX_FEATURES);
    orb->detectAndCompute(im1Gray, cv::Mat(), keypoints1, descriptors1);
    orb->detectAndCompute(im2Gray, cv::Mat(), keypoints2, descriptors2);

    // Match features.
    std::vector<cv::DMatch> matches;
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
    matcher->match(descriptors1, descriptors2, matches, cv::Mat());

    // Sort matches by score
    std::sort(matches.begin(), matches.end());

    // Remove not so good matches
    const int numGoodMatches = matches.size() * GOOD_MATCH_PERCENT;
    matches.erase(matches.begin()+numGoodMatches, matches.end());

    // Draw top matches
    cv::Mat imMatches;
    drawMatches(im1, keypoints1, im2, keypoints2, matches, imMatches);
    cv::imwrite("/home/izzet/catkin_ws/src/map_stitcher/src/matches.jpg", imMatches);

    // Extract location of good matches
    std::vector<cv::Point2f> points1, points2;

    for( size_t i = 0; i < matches.size(); i++ )
    {
        points1.push_back( keypoints1[ matches[i].queryIdx ].pt );
        points2.push_back( keypoints2[ matches[i].trainIdx ].pt );
    }

    // Find homography
    //cv::Mat h = cv::getAffineTransform(points1, points2);
    cv::Mat h = cv::findHomography(points2, points1, cv::RANSAC);

    cv::Mat im1Reg;
    // Use homography to warp image
    //cv::warpAffine(im1, im1Reg, h, im2.size());
    cv::warpPerspective(im1, im1Reg, h, im2.size());

    im1Reg = im1Reg + im2;

    std::string outFilename("/home/izzet/catkin_ws/src/map_stitcher/src/3.jpg");
    std::cout << "Saving aligned image : " << outFilename << std::endl;
    cv::imwrite(outFilename, im1Reg);




}

} //namespace

int main(int argc, char** argv){
    ros::init(argc,argv, "map_stitcher_node");
    ROS_INFO("%s", ros::this_node::getName().c_str());
    mapstitcher::mapStitcher ms;
    ros::spin();
    return 0;
}