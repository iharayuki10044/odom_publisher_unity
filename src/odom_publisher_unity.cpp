#include "odom_publisher_unity/odom_publisher_unity.h"

OdomPublisherUnity::OdomPublisherUnity()
    : private_nh("~")
{
    private_nh.param("HZ", HZ, 10.0);

    odom_sub = private_nh.subscribe("/odom_unit04", 1, &OdomPublisherUnity::odom_callback, this);     
    // odom_tf_pub = new tf::TransformBroadcaster();
    // odom_tf_listener = new tf::TransformListener();

    pose_pub = private_nh.advertise<geometry_msgs::PoseStamped>("/pose", 1);

    unity_subscribed = false;
    is_firstmsgs = true;

}


void OdomPublisherUnity::odom_callback(const nav_msgs::Odometry& msg)
{
    odom_pose = msg;
    odom_subscribed = true;

    geometry_msgs::PoseStamped pose_stamped;    

    pose_stamped.header = odom_pose.header;
    pose_stamped.pose = odom_pose.pose.pose;

    // robot位置と姿勢(クォータニオン)の取得
    double x = pose_stamped.pose.position.x;
    double y = pose_stamped.pose.position.y;
    double z = pose_stamped.pose.position.z;
    geometry_msgs::Quaternion q = pose_stamped.pose.orientation;

    // robot座標系の原点となるロボットの位置姿勢情報格納用変数
    geometry_msgs::TransformStamped robot_pose;

    // 現在時間の格納 
    robot_pose.header.stamp = ros::Time::now();

    // 座標系map と　robot の指定
    robot_pose.header.frame_id = "map";
    robot_pose.child_frame_id = "base_footprint";

    robot_pose.transform.translation.x = x;
    robot_pose.transform.translation.y = y;
    robot_pose.transform.translation.z = z;
    robot_pose.transform.rotation = q;

    // ロボットの位置姿勢情報をTFに送信
    robot_pose_broadcaster.sendTransform(robot_pose);

    pose_pub.publish(pose_stamped);

    // std::cout << "odom tf send ok" << std::endl;

} 

void OdomPublisherUnity::process()
{
    ros::Rate loop_rate(HZ);
    std::cout<<"------odom_publisher_unity------"<<std::endl;
 
    while(ros::ok()){
        if(unity_subscribed){
            // std::cout<<"is_firstmsgs: "<<is_firstmsgs<<std::endl; 
            // std::cout<<"unity_pose.position: "<<unity_pose<<std::endl;
            // std::cout<<"init_unity_pose_position: "<<init_unity_pose<<std::endl;
            // std::cout<<"unity_odom_pose.position: "<<unity_odom_pose<<std::endl; 
        }
        if(odom_subscribed){
            // std::cout<<"odom_pose.position: "<<odom_pose<<std::endl;
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