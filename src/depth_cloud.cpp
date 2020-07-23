#include <iostream>
#include <string>
#include <queue>
#include <vector>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <tf/transform_listener.h>

ros::Publisher pub_pcl;
ros::Publisher pub_pcl_clean;

void receiveImages(const sensor_msgs::ImageConstPtr& rgbMsg, const sensor_msgs::ImageConstPtr& depthMsg, const sensor_msgs::CameraInfoConstPtr& infoMsg)
{
    std::cout << "RGB " << rgbMsg->header.seq << " encoding " << rgbMsg->encoding << std::endl;
    cv_bridge::CvImagePtr cv_ptr_rgb;
    try
    {
      cv_ptr_rgb = cv_bridge::toCvCopy(rgbMsg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    std::cout << "DEPTH " << depthMsg->header.seq <<  " encoding " << depthMsg->encoding << std::endl;
    cv_bridge::CvImagePtr cv_ptr_d;
    try
    {
      cv_ptr_d = cv_bridge::toCvCopy(depthMsg, sensor_msgs::image_encodings::MONO16);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

//    // Show both images side-by-side
//    cv::Mat dst;
//    cv::vconcat(cv_ptr_rgb->image, cv_ptr_d->image, dst);
//    cv::rotate(dst, dst, cv::ROTATE_90_COUNTERCLOCKWISE);
//    cv::imshow("saida_nova",dst);
//    cv::waitKey(3);


    //    [fx  0 cx]           [ 1/fx;    0; -cx/fx]
    // K= [ 0 fy cy]     invK= [    0; 1/fy; -cy/fy]
    //    [ 0  0  1]           [    0;    0;      1]
    float fx = infoMsg->K[0];
    float fy = infoMsg->K[4];
    float cx = infoMsg->K[2];
    float cy = infoMsg->K[5];
    std::cout << "fx " << fx << " fy " << fy << " cx " << cx << " cy " << cy << std::endl;


    // Depth information (in meters) is encoded in the following
    float depth_scale = (1/6553.5); // depth_i = value_of_pixel_i*(1/6,553.5);

    std::vector<float> points;
    std::vector<cv::Vec3b> colors;

    std::vector<float> points_clean;
    std::vector<cv::Vec3b> colors_clean;
    int gap = 30;

    float maxId = -1;

    for(int i =0;i<cv_ptr_d->image.rows;i++){
        const ushort* row_ptr = cv_ptr_d->image.ptr<ushort>(i);
        const float* color_row_ptr = cv_ptr_rgb->image.ptr<float>(i);
        for(int j=0;j<cv_ptr_d->image.cols;j++){
            ushort id=row_ptr[j];
            float d=depth_scale*(float)id;

            if((float)id > maxId) maxId = float(id);

//            std::cout << id << " " << d << std::endl;

            if(d>0 && d<50){
//            if(id!=0){

                // (x,y,z)^T = invK*(u,v,w)^T
                points.push_back(-((float)j-cx)*d/fx); // x
                points.push_back(-((float)i-cy)*d/fy); // y
                points.push_back(d);           // z

                cv::Vec3b pixel = cv_ptr_rgb->image.at<cv::Vec3b>(i,j);
//                std::cout << "rgb " << (int)pixel[0] << ' ' << (int)pixel[1] << ' ' << (int)pixel[2] << std::endl;

                colors.push_back(pixel); // rgb

                if(j%gap == 0){
                    points_clean.push_back(-((float)j-cx)*d/fx); // x
                    points_clean.push_back(-((float)i-cy)*d/fy); // y
                    points_clean.push_back(d);           // z

                    colors_clean.push_back(pixel);
                }

            }
        }
    }

//    std::cout << "CHEGOU!" << std::endl;

    int n_points = points.size()/3;

    std::cout << "Points " << n_points << std::endl;

    if(n_points<1)
        return;


    // Create a PointCloud2
    sensor_msgs::PointCloud2 cloud_msg;
    sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
//    modifier.setPointCloud2Fields(3, "x", 1, sensor_msgs::PointField::FLOAT32,
//                                     "y", 1, sensor_msgs::PointField::FLOAT32,
//                                     "z", 1, sensor_msgs::PointField::FLOAT32);
//    modifier.setPointCloud2FieldsByString(1, "xyz");
//    modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::PointField::FLOAT32,
//                                     "y", 1, sensor_msgs::PointField::FLOAT32,
//                                     "z", 1, sensor_msgs::PointField::FLOAT32,
//                                   "rgb", 1, sensor_msgs::PointField::FLOAT32);
    modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
    modifier.resize(n_points);

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(cloud_msg, "r");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(cloud_msg, "g");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(cloud_msg, "b");

    cloud_msg.height = 1;
    cloud_msg.width = n_points;
    cloud_msg.header.frame_id = cv_ptr_d->header.frame_id;
    cloud_msg.header.seq = cv_ptr_d->header.seq;
    cloud_msg.header.stamp = cv_ptr_d->header.stamp;

    float minX, maxX, minY, maxY, minZ, maxZ;
    minX=maxX=points[0];
    minY=maxY=points[1];
    minZ=maxZ=points[2];



    for(size_t i=0; i<n_points; ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_r, ++iter_g, ++iter_b){
        *iter_x = points[3*i+0];
        *iter_y = points[3*i+1];
        *iter_z = points[3*i+2];

        *iter_r = colors[i][2];
        *iter_g = colors[i][1];
        *iter_b = colors[i][0];

        if(points[3*i+0] < minX) minX=points[3*i+0];
        if(points[3*i+0] > maxX) maxX=points[3*i+0];
        if(points[3*i+1] < minY) minY=points[3*i+1];
        if(points[3*i+1] > maxY) maxY=points[3*i+1];
        if(points[3*i+2] < minZ) minZ=points[3*i+2];
        if(points[3*i+2] > maxZ) maxZ=points[3*i+2];


//        std::cout << *iter_x << " " << *iter_y << " " << *iter_z << std::endl;
    }

    std::cout << "X: " << minX << " " << maxX << " Y: " << minY << " " << maxY << " Z: " << minZ << " " << maxZ << " depth " << maxId << std::endl;

//    std::cerr << ".";

    pub_pcl.publish(cloud_msg);


    n_points = points_clean.size()/3;
    std::cout << "Clean Points " << n_points << std::endl;

    // Create a second PointCloud2
    sensor_msgs::PointCloud2 clean_cloud_msg;
    sensor_msgs::PointCloud2Modifier clean_modifier(clean_cloud_msg);
    clean_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
    clean_modifier.resize(n_points);

    sensor_msgs::PointCloud2Iterator<float> clean_iter_x(clean_cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> clean_iter_y(clean_cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> clean_iter_z(clean_cloud_msg, "z");
    sensor_msgs::PointCloud2Iterator<uint8_t> clean_iter_r(clean_cloud_msg, "r");
    sensor_msgs::PointCloud2Iterator<uint8_t> clean_iter_g(clean_cloud_msg, "g");
    sensor_msgs::PointCloud2Iterator<uint8_t> clean_iter_b(clean_cloud_msg, "b");

    clean_cloud_msg.height = 1;
    clean_cloud_msg.width = n_points;
    clean_cloud_msg.header.frame_id = cv_ptr_d->header.frame_id;
    clean_cloud_msg.header.seq = cv_ptr_d->header.seq;
    clean_cloud_msg.header.stamp = cv_ptr_d->header.stamp;

    for(size_t i=0; i<n_points; ++i, ++clean_iter_x, ++clean_iter_y, ++clean_iter_z, ++clean_iter_r, ++clean_iter_g, ++clean_iter_b){
        *clean_iter_x = points_clean[3*i+0];
        *clean_iter_y = points_clean[3*i+1];
        *clean_iter_z = points_clean[3*i+2];

        *clean_iter_r = colors_clean[i][2];
        *clean_iter_g = colors_clean[i][1];
        *clean_iter_b = colors_clean[i][0];
    }

    pub_pcl_clean.publish(clean_cloud_msg);


}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "generate_depth_cloud");
    ROS_INFO("generate_depth_cloud");

    ros::WallTime start, last, current;

    ros::NodeHandle* n_ = new ros::NodeHandle("~");
    ros::Rate* rate_ = new ros::Rate(100);
//    tf::TransformListener* listener;

    start = ros::WallTime::now();
    last = start;

    std::string cameraName, rgbTopic, depthTopic, camInfoTopic, cloudTopic, cleanCloudTopic;
    n_->getParam("camera_name",cameraName);
    rgbTopic     = cameraName + "/rgb/image_raw";
    depthTopic   = cameraName + "/depth/image_raw";
    camInfoTopic = cameraName + "/depth/camera_info";
    cloudTopic   = cameraName + "/depth/points";
    cleanCloudTopic = cameraName + "/depth/points_clean";


//    n_->getParam("rgbTopic",rgbTopic);
//    n_->getParam("depthTopic",depthTopic);
//    n_->getParam("camInfoTopic",camInfoTopic);
//    cloudTopic = depthTopic;
//    cloudTopic.erase(cloudTopic.rfind('/'));
//    cloudTopic = cloudTopic + "/points";

    std::cout << "RGB    :" << rgbTopic << std::endl;
    std::cout << "DEPTH  :" << depthTopic << std::endl;
    std::cout << "INFO   :" << camInfoTopic << std::endl;
    std::cout << "POINTS :" << cloudTopic << std::endl;

    // If not synchronized, subscribe to each topic with a different callback
//    ros::Subscriber sub_rgb_   = n_->subscribe(rgbTopic, 100, &receiveRGBImage);
//    ros::Subscriber sub_depth_ = n_->subscribe(depthTopic, 100, &receiveDepthImage);

    // If topics are synchronized, subscribe to them with a single callback
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(*n_, rgbTopic, 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(*n_, depthTopic, 1);
    message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub(*n_, camInfoTopic, 1);
    message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo> sync(rgb_sub, depth_sub, info_sub, 100);
    sync.registerCallback(receiveImages);

    pub_pcl = n_->advertise<sensor_msgs::PointCloud2>(cloudTopic, 1);
    pub_pcl_clean = n_->advertise<sensor_msgs::PointCloud2>(cleanCloudTopic, 1);




    while (ros::ok())
    {
//        // If not synchronized, use auxiliar queues
//        if(!rgbVec.empty() && !depthVec.empty()){
//            std::cout << "SIZES " << rgbVec.size() << " " << depthVec.size() << " | IMAGES " << rgbVec.front()->header.seq << " " << depthVec.front()->header.seq << std::endl;
//            if(rgbVec.front()->header.seq == depthVec.front()->header.seq){

//                analyzeImages(rgbVec.front(),depthVec.front());

//                rgbVec.pop();
//                depthVec.pop();
//            }

//        }

        ros::spinOnce();
        rate_->sleep();
    }


//    ros::spin();
    return 0;
}
