#ifndef __ODOM_PUBLISHER_UNITY_H__
#define __ODOM_PUBLISHER_UNITY_H__

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <eigen3/Eigen/Dense> 

#include <tf2_ros/transform_broadcaster.h>

class OdomPublisherUnity{
    public:
        OdomPublisherUnity();
        void process();

        void unity_callback(const geometry_msgs::PoseStamped&);

    private:
        double HZ;
        double tf_roatation;

        bool unity_subscribed;
        bool is_firstmsgs;

        geometry_msgs::PoseStamped unity_pose;
        geometry_msgs::PoseStamped init_unity_pose;
        geometry_msgs::PoseStamped unity_odom_pose;

        ros::NodeHandle private_nh;
        
        ros::Subscriber unity_sub;
        ros::Publisher unity_odom_pub;

        //TF Broadcasterの実体化
        // tf unity to map 
        tf2_ros::TransformBroadcaster robot_pose_broadcaster;

        // tf map to unity
        tf2_ros::TransformBroadcaster map_to_basefoot_broadcaster;


};

#endif // __ODOM_PUBLISHER_UNITY_H__