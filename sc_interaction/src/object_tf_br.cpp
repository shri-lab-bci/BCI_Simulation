#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <turtlesim/Pose.h>
#include "sc_interaction/ObjectInfo.h"
#include "sc_interaction/ObjectInfos.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2_msgs/TFMessage.h>

#include <string>
#include <stdio.h>
#include <sstream> 
#include <iostream>

using namespace std;

void callback_object_info(const sc_interaction::ObjectInfos::ConstPtr& msg){
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    tf2_msgs::TFMessage transformArray;
    tf2::Quaternion q; 
    transformArray.transforms.clear();

    for(size_t i=0; i<msg->object_infos.size(); ++i)
    {  
        const sc_interaction::ObjectInfo &raw_data = msg->object_infos[i];
        // Name the TF Marker as string format
        string child_frame_id = "Person_";
        string pg;    
        stringstream s;
        s << raw_data.id;
        pg = s.str();
        child_frame_id.append(pg);

        transformStamped.header.stamp = ros::Time::now();    
        transformStamped.header.frame_id = "camera_link";
        transformStamped.child_frame_id = child_frame_id;
        transformStamped.transform.translation.x = raw_data.point.x;
        transformStamped.transform.translation.y = raw_data.point.y;
        transformStamped.transform.translation.z = raw_data.point.z;    
        q.setRPY(0, 0, 0);
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();
        transformArray.transforms.push_back(transformStamped);
    }  

    br.sendTransform(transformArray.transforms); 
}

void callback_marker_info(const sc_interaction::ObjectInfos::ConstPtr& msg){
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    tf2_msgs::TFMessage transformArray;
    tf2::Quaternion q; 
    transformArray.transforms.clear();

    for(size_t i=0; i<msg->object_infos.size(); ++i)
    {  
        const sc_interaction::ObjectInfo &raw_data = msg->object_infos[i];
        // Name the TF Marker as string format
        string child_frame_id = "Marker_";
        string pg;    
        stringstream s;
        s << raw_data.id;
        pg = s.str();
        child_frame_id.append(pg);

        transformStamped.header.stamp = ros::Time::now();    
        transformStamped.header.frame_id = "base_scan";
        transformStamped.child_frame_id = child_frame_id;
        transformStamped.transform.translation.x = raw_data.point.x;
        transformStamped.transform.translation.y = raw_data.point.y;
        transformStamped.transform.translation.z = raw_data.point.z;    
        q.setRPY(0, 0, 0);
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();
        transformArray.transforms.push_back(transformStamped);
    }  

    br.sendTransform(transformArray.transforms); 
}

int main(int argc, char** argv){
    ros::init(argc, argv, "object_tf_br");

    ros::NodeHandle node;
    ros::Subscriber object_info_sub = node.subscribe("/recognition/object_infos", 10,  &callback_object_info);
    ros::Subscriber marker_info_sub = node.subscribe("/recognition/marker3d_infos", 10,  &callback_marker_info);
  
    ros::spin();
    return 0;
};
