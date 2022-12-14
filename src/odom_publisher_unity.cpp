#include "odom_publisher_unity/odom_publisher_unity.h"

OdomPublisherUnity::OdomPublisherUnity()
    : private_nh("~")
{
    private_nh.param("HZ", HZ, 10.0);

    // odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 1);
    unity_sub = private_nh.subscribe("/pose_unit04", 1, &OdomPublisherUnity::unity_callback, this);
    // odom_tf_pub = new tf::TransformBroadcaster();
    // odom_tf_listener = new tf::TransformListener();

    unity_odom_pub = private_nh.advertise<geometry_msgs::PoseStamped>("/pose", 1);

    unity_subscribed = false;
    is_firstmsgs = true;

}

void OdomPublisherUnity::unity_callback(const geometry_msgs::PoseStamped& msg)
{
    unity_pose = msg;
    unity_subscribed = true;

    if(is_firstmsgs){
        init_unity_pose = msg;
        init_unity_pose.header.frame_id = "Unity";
    
        //  //Robot位置と姿勢(x,y,yaw)の取得
        // double x = unity_pose.pose.position.x;
        // double y = unity_pose.pose.position.y;
        // double z = unity_pose.pose.position.z;
 
        // //yawのデータからクォータニオンを作成
        // geometry_msgs::Quaternion robot_quat=unity_pose.pose.orientation;
        // geometry_msgs::Quaternion transform_quat = robot_quat;

        // //map座標系の元となるロボットの位置姿勢情報格納用変数の作成
        // geometry_msgs::TransformStamped map_origin_pose_tf;

        // //現在の時間の格納
        // map_origin_pose_tf.header.stamp = ros::Time::now();

        // //座標系unityとmapの指定
        // map_origin_pose_tf.header.frame_id = "Unity";
        // map_origin_pose_tf.child_frame_id  = "map";

        // //Unity座標系からみたmap座標系の原点位置と方向の格納
        // map_origin_pose_tf.transform.translation.x = x;
        // map_origin_pose_tf.transform.translation.y = y;
        // map_origin_pose_tf.transform.translation.z = z;
        // map_origin_pose_tf.transform.rotation = robot_quat;

        // //tf情報をbroadcast(座標系の設定)
        // robot_pose_broadcaster.sendTransform(map_origin_pose_tf);

        // double roll, pitch, yaw;
        // tf::Matrix3x3(tf::Quaternion(robot_quat.x, robot_quat.y, robot_quat.z, robot_quat.w)).getRPY(roll, pitch, yaw);

        // std::cout << "init_unity_roll : "<< roll / 3.1415 *180 << std::endl;
        // std::cout << "init_unity_pitch : "<< pitch / 3.1415 *180 << std::endl;
        // std::cout << "init_unity_yaw : " << yaw /  3.1415 *180 << std::endl;

        // is_firstmsgs = false;
    
    }


    unity_odom_pose = msg;
    unity_odom_pose.pose.position.x = unity_pose.pose.position.x - init_unity_pose.pose.position.x;
    unity_odom_pose.pose.position.y = unity_pose.pose.position.y - init_unity_pose.pose.position.y;
    unity_odom_pose.pose.position.z = unity_pose.pose.position.z - init_unity_pose.pose.position.z;

    unity_odom_pose.header.stamp = ros::Time::now();
    unity_odom_pose.header.frame_id = "map";

    unity_odom_pub.publish(unity_odom_pose);
    
    //Robot位置と姿勢(x,y,yaw)の取得
    double x = unity_odom_pose.pose.position.x;
    double y = unity_odom_pose.pose.position.y;
    double z = unity_odom_pose.pose.position.z;
 
    //yawのデータからクォータニオンを作成
    geometry_msgs::Quaternion now_quat;
    now_quat.w = 0.0;
    now_quat.x = 0.0;
    now_quat.y = 0.0;
    now_quat.z = 1.0;
    
    //robot座標系の元となるロボットの位置姿勢情報格納用変数の作成
    geometry_msgs::TransformStamped robotState;
    
    //現在の時間の格納
    robotState.header.stamp = ros::Time::now();
    
    //座標系globalとrobotの指定
    robotState.header.frame_id = "map";
    robotState.child_frame_id  = "base_footprint";
    
    //global座標系からみたrobot座標系の原点位置と方向の格納
    robotState.transform.translation.x = x;
    robotState.transform.translation.y = y;
    robotState.transform.translation.z = z;
    robotState.transform.rotation = unity_pose.pose.orientation;
    // robotState.transform.rotation = now_quat;

    //tf情報をbroadcast(座標系の設定)
    robot_pose_broadcaster.sendTransform(robotState);

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