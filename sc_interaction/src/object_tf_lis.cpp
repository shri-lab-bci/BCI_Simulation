#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"
#include "sc_interaction/ObjectInfo.h"
#include "sc_interaction/ObjectInfos.h"

#include <string>
#include <stdio.h>
#include <sstream> 
#include <iostream>

using namespace std;

int g_nSize;

// Empty data arrays(type of vector) for Marker data processing
sc_interaction::ObjectInfo g_DataForSub;
sc_interaction::ObjectInfo g_DataForPub;
sc_interaction::ObjectInfos g_aDataForSub; 
sc_interaction::ObjectInfos g_aDataForPub;

void callback_object_infos(const sc_interaction::ObjectInfos::ConstPtr& msg) {
    g_nSize = msg->object_infos.size();

    g_aDataForSub.object_infos.clear(); 
    for(size_t i=0; i<msg->object_infos.size(); ++i)
    {    
        const sc_interaction::ObjectInfo &raw_data = msg->object_infos[i];
        g_DataForSub.id = raw_data.id;
        g_DataForSub.point.x = raw_data.point.x;
        g_DataForSub.point.y = raw_data.point.y;
        g_DataForSub.point.z = raw_data.point.z;
        g_aDataForSub.object_infos.push_back(g_DataForSub);
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "object_tf_lis");
    ros::NodeHandle node;

    ros::Publisher object_info_tf_pub = node.advertise<sc_interaction::ObjectInfos>("/recognition/object_infos_tf", 10);
    ros::Subscriber object_info_sub = node.subscribe("/recognition/object_infos", 10,  &callback_object_infos);

    // Make tf2 buffer for saving TF data
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    ros::Rate r(40);
    while(ros::ok())
    {
        g_aDataForPub.object_infos.clear();
        for(size_t i=0; i<g_nSize; ++i){       
            int id = g_aDataForSub.object_infos[i].id;
            
            string child_frame_id = "Person_";
            string pg;    
            stringstream s;
            s << id;
            pg = s.str();
            child_frame_id.append(pg);      

            geometry_msgs::TransformStamped transformStamped;
            try{
                transformStamped = tfBuffer.lookupTransform("map", child_frame_id, ros::Time(0));
            }
            catch (tf2::TransformException &ex) {
                ROS_WARN("%s",ex.what());
                ros::Duration(1.0).sleep();
                continue;
            }   
            g_DataForPub.id = id;
            g_DataForPub.point.x = transformStamped.transform.translation.x;
            g_DataForPub.point.y = transformStamped.transform.translation.y;
            g_DataForPub.point.z = transformStamped.transform.translation.z;
            g_aDataForPub.object_infos.push_back(g_DataForPub);
        }
        object_info_tf_pub.publish(g_aDataForPub);
        ros::spinOnce();
        r.sleep();
    }
}
