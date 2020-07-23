#include <iostream>
#include <string>

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>

ros::Publisher pub_odom;
nav_msgs::Odometry odom;

void receivePose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &value)
{
    
//  STRUCTURE OF geometry_msgs::PoseWithCovarianceStamped

//# This expresses an estimated pose with a reference coordinate frame and timestamp

    //Header header
    //    # Standard metadata for higher-level stamped data types.
    //    # This is generally used to communicate timestamped data
    //    # in a particular coordinate frame.
    //    #
    //    # sequence ID: consecutively increasing ID
    //    uint32 seq
    //    #Two-integer timestamp that is expressed as:
    //    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    //    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    //    # time-handling sugar is provided by the client library
    //    time stamp
    //    #Frame this data is associated with
    //    # 0: no frame
    //    # 1: global frame
    //    string frame_id
    odom.header = value->header;

    odom.child_frame_id="base_link";

    //# This represents a pose in free space with uncertainty.

        //Pose pose
            //# A representation of pose in free space, composed of position and orientation.
            //Point position
                //# This contains the position of a point in free space
                //float64 x
                //float64 y
                //float64 z
            //Quaternion orientation
                //# This represents an orientation in free space in quaternion form.
                //float64 x
                //float64 y
                //float64 z
                //float64 w
        odom.pose.pose = value->pose.pose;

        //# Row-major representation of the 6x6 covariance matrix
        //# The orientation parameters use a fixed-axis representation.
        //# In order, the parameters are:
        //# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
        //float64[36] covariance
        odom.pose.covariance = value->pose.covariance;

    // odom.twist missing

    pub_odom.publish(odom);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pub_odom");
    ROS_INFO("pub_odom");

    ros::NodeHandle* n_ = new ros::NodeHandle("~");
    ros::Rate* rate_ = new ros::Rate(100);

    std::string inputTopic  = "/poseupdate";
    std::string outputTopic = "/odom";

    ros::Subscriber sub_pose_odom_   = n_->subscribe(inputTopic, 100, &receivePose);
    pub_odom = n_->advertise<nav_msgs::Odometry>(outputTopic, 1);

    // while (ros::ok())
    // {
    //     ros::spinOnce();
    //     rate_->sleep();
    // }

    ros::spin();
    return 0;
}
