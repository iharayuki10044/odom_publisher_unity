#include "odom_publisher_unity/odom_publisher_unity.h"

OdomPublisherUnity::OdomPublisherUnity()
    : private_nh("~")
{
    private_nh.param("HZ", HZ, 10.0);

    // odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 1);
    unity_sub = private_nh.subscribe("/pose_unit04", 1, &OdomPublisherUnity::unity_callback, this);
    // odom_tf_pub = new tf::TransformBroadcaster();
    // odom_tf_listener = new tf::TransformListener();

    // odom_tf_listener->waitForTransform("odom", "base_link", ros::Time(0), ros::Duration(10.0));
    // odom_tf_listener->lookupTransform("odom", "base_link", ros::Time(0), odom_tf);
    // odom_tf_listener->waitForTransform("odom", "base_footprint", ros::Time(0), ros::Duration(10.0));
    // odom_tf_listener->lookupTransform("odom", "base_footprint", ros::Time(0), odom_tf_footprint);
    // odom_tf_listener->waitForTransform("odom", "base_laser", ros::Time(0), ros::Duration(10.0));
    // odom_tf_listener->lookupTransform("odom", "base_laser", ros::Time(0), odom_tf_laser);
    // odom_tf_listener->waitForTransform("odom", "base_scan", ros::Time(0), ros::Duration(10.0));
    // odom_tf_listener->lookupTransform("odom", "base_scan", ros::Time(0), odom_tf_scan);
    // odom_tf_listener->waitForTransform("odom", "base_stabilized", ros::Time(0), ros::Duration(10.0));
    // odom_tf_listener->lookupTransform("odom", "base_stabilized", ros::Time(0), odom_tf_stabilized);
    // odom_tf_listener->waitForTransform("odom", "base_scan_front", ros::Time(0), ros::Duration(10.0));
    // odom_tf_listener->lookupTransform("odom", "base_scan_front", ros::Time(0), odom_tf_scan_front);
    // odom_tf_listener->waitForTransform("odom", "base_scan_back", ros::Time(0), ros::Duration(10.0));
    // odom_tf_listener->lookupTransform("odom", "base_scan_back", ros::Time(0), odom_tf_scan_back);
    // odom_tf_listener->waitForTransform("odom", "base_scan_left", ros::Time(0), ros::Duration(10);

    unity_subscribed = false;
    is_firstmsgs = true;

}

void OdomPublisherUnity::unity_callback(const geometry_msgs::PoseStamped& msg)
{
    unity_pose = msg;
    unity_subscribed = true;

    if(is_firstmsgs){
        init_unity_pose = msg;
        is_firstmsgs = false;
    }

}

void OdomPublisherUnity::process()
{
    ros::Rate loop_rate(HZ);
    std::cout<<"------odom_publisher_unity------"<<std::endl;
 
    while(ros::ok()){
        if(unity_subscribed){
            std::cout<<"is_firstmsgs: "<<is_firstmsgs<<std::endl; 
            std::cout<<"unity_pose: "<<unity_pose<<std::endl;
            std::cout<<"init_unity_pose: "<<init_unity_pose<<std::endl;
            // std::cout<<"unity_pose - init_unity_pose: "<<unity_pose - init_unity_pose<<std::endl;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odom_publisher_unity");
    OdomPublisherUnity odom_publisher_unity;
    odom_publisher_unity.process(); 
    return 0;
}