#include <iostream>
#include <string>
#include <vector>
#include <unordered_map>
#include <chrono>
#include<math.h>
#include<time.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <message_filters/subscriber.h>  
#include <message_filters/time_synchronizer.h> 

#include "autoware_msgs/DetectedObjectArray.h"
#include "autoware_msgs/DetectedObject.h"

#define CAR_THRESHOLD 1
#define PERSON_THRESHOLD 0.1

using namespace message_filters;  

autoware_msgs::DetectedObjectArray pointpillarsResultArray;
autoware_msgs::DetectedObjectArray fusionResultArray;
autoware_msgs::DetectedObjectArray finalResultArray;

ros::Publisher pub_final_results;

bool isSameObject(autoware_msgs::DetectedObject fusion_result, autoware_msgs::DetectedObject pointpillars_result);



void callback(const autoware_msgs::DetectedObjectArray::ConstPtr &in_pointpillar_detections, const autoware_msgs::DetectedObjectArray::ConstPtr &in_fusion_detections)  //回调中包含多个消息  
{  
    pointpillarsResultArray.header = in_pointpillar_detections->header;
    pointpillarsResultArray.objects.clear();

    finalResultArray.header = in_pointpillar_detections->header;
    finalResultArray.objects.clear();

    for (size_t i = 0; i < in_pointpillar_detections->objects.size(); i++)
    {
        // std::cout<<"Pointpillar get a object: "<< in_range_detections->objects[i].label<<std::endl;
        pointpillarsResultArray.objects.push_back(in_pointpillar_detections->objects[i]);
    }

    fusionResultArray.header = in_fusion_detections->header;
    fusionResultArray.objects.clear();

    for (size_t i = 0; i < in_fusion_detections->objects.size(); i++)
    {
        bool sameFlag = false;
        //remove same object
        if( in_fusion_detections->objects[i].label == "car" || in_fusion_detections->objects[i].label == "person" || in_fusion_detections->objects[i].label == "truck" || in_fusion_detections->objects[i].label == "bicycle"){
            for(size_t j=0;j<pointpillarsResultArray.objects.size();j++){
                if(isSameObject(in_fusion_detections->objects[i],pointpillarsResultArray.objects[j])==true){
                    sameFlag = true;
                }    
            }
            if(!sameFlag){
                finalResultArray.objects.push_back(in_fusion_detections->objects[i]);
                // std::cout<<"Fusion get a fusion object: "<< in_fusion_detections->objects[i].label<<std::endl;
            }
        }
    }


    for(size_t i=0;i<pointpillarsResultArray.objects.size();i++){
        finalResultArray.objects.push_back(pointpillarsResultArray.objects[i]);
        // std::cout<<"Label: "<<pointpillarsResultArray.objects[i].label<<", "<<pointpillarsResultArray.objects[i].dimensions<<std::endl;
        // std::cout<<"Pose: "<<pointpillarsResultArray.objects[i].pose.position<<std::endl;
        // std::cout<<"Orientation: "<<pointpillarsResultArray.objects[i].pose.orientation<<std::endl;
    }
    if(pointpillarsResultArray.objects.size()>0){
        ROS_INFO("origin num: %d, final num: %d",pointpillarsResultArray.objects.size(), finalResultArray.objects.size());
    }
    pub_final_results.publish(finalResultArray);
  // Solve all of perception here...  
} 




bool isSameObject(autoware_msgs::DetectedObject fusion_result, autoware_msgs::DetectedObject pointpillars_result) {
    float dx,dy,dz,result_distance;
    dx = fusion_result.pose.position.x-pointpillars_result.pose.position.x;
    dy = fusion_result.pose.position.y-pointpillars_result.pose.position.y;
    dz = fusion_result.pose.position.z-pointpillars_result.pose.position.z;
    result_distance = sqrt(dx*dx+dy*dy);
    if(fusion_result.label=="person" && fabs(result_distance)<PERSON_THRESHOLD){
        return true;
    }else if(fabs(result_distance)<CAR_THRESHOLD){
        // ROS_INFO("Same Object: %.2f",result_distance);
        return true;
    }else{
        // ROS_INFO("Different Object: %.2f",result_distance);
        return false;
    }
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "processObject");
    ros::NodeHandle n("~");

    pub_final_results = n.advertise<autoware_msgs::DetectedObjectArray>("/detection/final_result/objects", 30);

    message_filters::Subscriber<autoware_msgs::DetectedObjectArray> sub_fusion(n, "/detection/fusion_tools/objects", 1);             // topic1 输入  
    message_filters::Subscriber<autoware_msgs::DetectedObjectArray> sub_pointpillar(n, "/detection/lidar_detector_3d/objects", 1);   // topic2 输入  
    TimeSynchronizer<autoware_msgs::DetectedObjectArray, autoware_msgs::DetectedObjectArray> sync(sub_pointpillar, sub_fusion, 30);       // 同步  
    sync.registerCallback(boost::bind(&callback, _1, _2));    

    // ros::Subscriber sub_fusion = n.subscribe("/detection/fusion_tools/objects", 1000, fusionCallback);
    // ros::Subscriber sub_pointpillar = n.subscribe("/detection/lidar_detector_3d/objects", 1000, pointpillarCallback);

    ros::Rate r(30);  //    ros::Rate r(10.0 / publish_delay);
    while (ros::ok())
    {
        // processObject();
        // pub_final_results.publish(finalResultArray);
        // ROS_INFO("Publish Final Result ok!");
        // pub_final_results.publish(finalResultArray);
        ros::spinOnce();	
        r.sleep();
    }
    return 0;
}