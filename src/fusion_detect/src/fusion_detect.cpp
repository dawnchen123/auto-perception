#include <iostream>
#include <string>
#include <vector>
#include <unordered_map>
#include <chrono>
#include<math.h>
#include<time.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <message_filters/subscriber.h>  
#include <message_filters/time_synchronizer.h> 

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "autoware_msgs/DetectedObjectArray.h"
#include "autoware_msgs/DetectedObject.h"

#include <numeric>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#define CAR_THRESHOLD 1
#define PERSON_THRESHOLD 0.1

using namespace message_filters;  




struct LidarPoint { // single lidar point in space
    double x,y,z,r; // x,y,z in [m], r is point reflectivity
};

autoware_msgs::DetectedObjectArray pointpillarsResultArray;
autoware_msgs::DetectedObjectArray fusionResultArray;
autoware_msgs::DetectedObjectArray finalResultArray;

ros::Publisher pub_final_results;
ros::Publisher image_pub_;
ros::Publisher marker_pub_box_;

bool isSameObject(autoware_msgs::DetectedObject fusion_result, autoware_msgs::DetectedObject pointpillars_result);
void getImageBox(autoware_msgs::DetectedObjectArray pointpillars_result, cv::Mat yolo_img);
void ClearAllMarker();

void callback(const autoware_msgs::DetectedObjectArray::ConstPtr &in_pointpillar_detections, const autoware_msgs::DetectedObjectArray::ConstPtr &in_fusion_detections,const sensor_msgs::Image::ConstPtr & img_msg)  //回调中包含多个消息  
{  
    pointpillarsResultArray.header = in_pointpillar_detections->header;
    pointpillarsResultArray.objects.clear();

    finalResultArray.header = in_pointpillar_detections->header;
    finalResultArray.objects.clear();

    for (size_t i = 0; i < in_pointpillar_detections->objects.size(); i++)
    {        
        // std::cout<<"Orientation: "<<pointpillarsResultArray.objects[i].pose.orientation<<std::endl;
        // std::cout<<"Pointpillar get a object: "<< in_range_detections->objects[i].label<<std::endl;
        pointpillarsResultArray.objects.push_back(in_pointpillar_detections->objects[i]);
    }

    fusionResultArray.header = in_fusion_detections->header;
    fusionResultArray.objects.clear();

    for (size_t i = 0; i < in_fusion_detections->objects.size(); i++)
    {
        bool sameFlag = false;
        //remove same object
        // if( in_fusion_detections->objects[i].label == "car" || in_fusion_detections->objects[i].label == "person" || in_fusion_detections->objects[i].label == "truck" || in_fusion_detections->objects[i].label == "bicycle"){
        if( in_fusion_detections->objects[i].label == "person" || in_fusion_detections->objects[i].label == "bicycle" || in_fusion_detections->objects[i].label =="motorbike"){
            // for(size_t j=0;j<pointpillarsResultArray.objects.size();j++){
            //     if(isSameObject(in_fusion_detections->objects[i],pointpillarsResultArray.objects[j])==true){
            //         sameFlag = true;
            //     }    
            // }
            // if(!sameFlag){
                finalResultArray.objects.push_back(in_fusion_detections->objects[i]);
                // std::cout<<"Fusion get a fusion object: "<< in_fusion_detections->objects[i].label<<std::endl;
            // }
        }
    }


    for(size_t i=0;i<pointpillarsResultArray.objects.size();i++){
        finalResultArray.objects.push_back(pointpillarsResultArray.objects[i]);
        // std::cout<<"Label: "<<pointpillarsResultArray.objects[i].label<<", "<<pointpillarsResultArray.objects[i].dimensions.x<<std::endl;
        // std::cout<<"Pose: "<<pointpillarsResultArray.objects[i].pose.position<<std::endl;
        // std::cout<<"Orientation: "<<pointpillarsResultArray.objects[i].pose.orientation<<std::endl;
    }
    if(pointpillarsResultArray.objects.size()>0){
        ROS_INFO("origin num: %d, final num: %d",pointpillarsResultArray.objects.size(), finalResultArray.objects.size());
    }

    cv::Mat origin_img;
    //ROS_INFO("IMAGE RECIEVIE!!!!!!!!!!!!!!!!!!!");
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(img_msg,"bgr8");
    cv_ptr->image.copyTo(origin_img);
    getImageBox(pointpillarsResultArray,origin_img);

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

Eigen::Matrix<double, 4, 4> P2_cam;
Eigen::Matrix<double, 4, 4> R_rect;
Eigen::Matrix<double, 4, 4> Tr_velo_to_cam;

void load_Calibration(std::string file_name)
{
    FILE *fp = fopen(file_name.c_str(), "r");
    if (!fp) {
        printf("open Calib error!!!\n");
        return;
    }
    char str[255];
    double temp[12];

    ///P
    for(int i = 0; i < 4; i++)
    {
        fscanf(fp, "%s %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
                    str, &temp[0], &temp[1], &temp[2],&temp[3], &temp[4], &temp[5], &temp[6],
                    &temp[7], &temp[8], &temp[9], &temp[10], &temp[11]);
        if(i==2){
            P2_cam <<  temp[0], temp[1], temp[2], temp[3], 
              temp[4], temp[5], temp[6], temp[7],
              temp[8], temp[9], temp[10], temp[11],
              0,        0,      0,       1;
        }
    }

    // ///R0_rect
    // fscanf(fp, "%s %lf %lf %lf %lf %lf %lf %lf %lf %lf",
    //                 str, &temp[0], &temp[1], &temp[2],&temp[3], &temp[4], &temp[5], &temp[6],
    //                 &temp[7], &temp[8]);

    // R_rect << temp[0], temp[1], temp[2], 0, 
    //         temp[3], temp[4], temp[5], 0,
    //         temp[6], temp[7], temp[8], 0,
    //         0,        0,      0,       1;

    // std::cout<<"R_rect: "<<R_rect<<std::endl;
    
    fscanf(fp, "%s %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
                    str, &temp[0], &temp[1], &temp[2],&temp[3], &temp[4], &temp[5], &temp[6],
                    &temp[7], &temp[8], &temp[9], &temp[10], &temp[11]);
    
    Tr_velo_to_cam << temp[0], temp[1], temp[2], temp[3], 
              temp[4], temp[5], temp[6], temp[7],
              temp[8], temp[9], temp[10], temp[11],
              0, 0, 0, 1;



    ROS_INFO("Load Calib finished!");
    fclose(fp);
}

void getImageBox(autoware_msgs::DetectedObjectArray pointpillars_result, cv::Mat yolo_img){ 
    cv::Mat image_send = yolo_img.clone();
    ClearAllMarker();
    visualization_msgs::MarkerArray marker_array_box;
    visualization_msgs::Marker line_strip;
    line_strip.header.frame_id = "velodyne";
    line_strip.header.stamp = ros::Time::now();
    line_strip.ns = "apollo::perception";
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_strip.scale.x = 0.2;
    line_strip.color.r = 0.0;
    line_strip.color.g = 1.0;
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;
    line_strip.points.resize(5);

    int marker_id = 0;
    for(int obj = 0; obj < pointpillars_result.objects.size(); ++obj) {
        Eigen::Vector3d center(pointpillars_result.objects[obj].pose.position.x,
                                                        pointpillars_result.objects[obj].pose.position.y,
                                                        pointpillars_result.objects[obj].pose.position.z);
        double half_l = pointpillars_result.objects[obj].dimensions.x/2;
        double half_w = pointpillars_result.objects[obj].dimensions.y/2;
        double h =pointpillars_result.objects[obj].dimensions.z;

        tf::Quaternion RQ2;
        double roll,pitch,yaw;
        tf::quaternionMsgToTF(pointpillars_result.objects[obj].pose.orientation,RQ2);
        // std::cout<<"Label: "<<pointpillars_result.objects[obj].label<<", "<<pointpillars_result.objects[obj].dimensions<<std::endl;
        // std::cout<<"Pose: "<<pointpillars_result.objects[obj].pose.position<<std::endl;
        tf::Matrix3x3(RQ2).getRPY(roll,pitch,yaw);  

        Eigen::Vector3d ldir(cos(yaw), sin(yaw), 0);
        Eigen::Vector3d odir(-ldir[1], ldir[0], 0);

        Eigen::Vector3d bottom_quad[8];
        bottom_quad[0] = center + ldir * -half_l + odir * -half_w;  // A(-half_l, -half_w)
        bottom_quad[1] = center + ldir * -half_l + odir * half_w;   // B(-half_l, half_w)
        bottom_quad[2] = center + ldir * half_l + odir * half_w;    // C(half_l, half_w)
        bottom_quad[3] = center + ldir * half_l + odir * -half_w;   // D(half_l, -half_w)


    /*Draw box start*/
    geometry_msgs::Point p1,p2,p3,p4,p5,p6,p7,p8;

    p1.x = p5.x = bottom_quad[0][0];
    p1.y = p5.y = bottom_quad[0][1];
    p1.z = bottom_quad[0][2]-h/2;
    p5.z =bottom_quad[0][2]+h/2;
    p2.x = p6.x = bottom_quad[1][0];
    p2.y = p6.y = bottom_quad[1][1];
    p2.z = bottom_quad[1][2]-h/2;
    p6.z = bottom_quad[1][2]+h/2;
    p3.x = p7.x = bottom_quad[2][0];
    p3.y = p7.y = bottom_quad[2][1];
    p3.z = bottom_quad[2][2]-h/2;
    p7.z = bottom_quad[2][2]+h/2;
    p4.x = p8.x = bottom_quad[3][0];
    p4.y = p8.y = bottom_quad[3][1];
    p4.z = bottom_quad[3][2]-h/2;
    p8.z = bottom_quad[3][2]+h/2;
    line_strip.id = marker_id;
    line_strip.points[0] = p1;
    line_strip.points[1] = p2;
    line_strip.points[2] = p3;
    line_strip.points[3] = p4;
    line_strip.points[4] = p1;
    marker_array_box.markers.push_back(line_strip);
    marker_id++;
    line_strip.id = marker_id;
    line_strip.points[0] = p5;
    line_strip.points[1] = p6;
    line_strip.points[2] = p7;
    line_strip.points[3] = p8;
    line_strip.points[4] = p5;
    marker_array_box.markers.push_back(line_strip);
    marker_id++;
    line_strip.id = marker_id;
    line_strip.points[0] = p1;
    line_strip.points[1] = p5;
    line_strip.points[2] = p8;
    line_strip.points[3] = p4;
    line_strip.points[4] = p1;
    marker_array_box.markers.push_back(line_strip);
    marker_id++;
    line_strip.id = marker_id;
    line_strip.points[0] = p2;
    line_strip.points[1] = p6;
    line_strip.points[2] = p7;
    line_strip.points[3] = p3;
    line_strip.points[4] = p2;
    marker_array_box.markers.push_back(line_strip);
    /*Draw box end*/

        // top 4 vertices
        bottom_quad[4] = bottom_quad[0];
        bottom_quad[4](2) += h;
        bottom_quad[5] = bottom_quad[1];
        bottom_quad[5](2) += h;
        bottom_quad[6] = bottom_quad[2];
        bottom_quad[6](2) += h;
        bottom_quad[7] = bottom_quad[3];
        bottom_quad[7](2) += h;

        Eigen::MatrixXd OBB(8, 4);
        OBB << bottom_quad[0](0), bottom_quad[0](1), bottom_quad[0](2), 1,
            bottom_quad[1](0), bottom_quad[1](1), bottom_quad[1](2), 1,
            bottom_quad[2](0), bottom_quad[2](1), bottom_quad[2](2), 1,
            bottom_quad[3](0), bottom_quad[3](1), bottom_quad[3](2), 1,
            bottom_quad[4](0), bottom_quad[4](1), bottom_quad[4](2), 1,
            bottom_quad[5](0), bottom_quad[5](1), bottom_quad[5](2), 1,
            bottom_quad[6](0), bottom_quad[6](1), bottom_quad[6](2), 1,
            bottom_quad[7](0), bottom_quad[7](1), bottom_quad[7](2), 1;

        // std::cout<<"Calcular Matrix: "<<P2_cam*R_rect *Tr_velo_to_cam.inverse()<<std::endl;

        Eigen::Matrix<double, 4, 4> CameraMat;
        CameraMat <<  7.188560000000e+02,0.000000000000e+00,6.071928000000e+02,4.538225000000e+01,
            0.000000000000e+00,7.188560000000e+02,1.852157000000e+02,-1.130887000000e-01,
            0.000000000000e+00,0.000000000000e+00,1.000000000000e+00,3.779761000000e-03,
            0,        0,      0,       1;
        Eigen::Matrix<double, 4, 4> CameraExtrinsicMat;
        CameraExtrinsicMat <<  7.533745e-03, 1.480249e-02, 9.998621e-01, -4.069766e-03,
          -9.999714e-01, 7.280733e-04, 7.523790e-03, -7.631618e-02,
          -6.166020e-04, -9.998902e-01, 1.480755e-02, -2.717806e-01, 
          0., 0., 0., 1.;

        
        // Eigen::MatrixXd pts_2d_hom = (P2_cam*CameraExtrinsicMat.inverse() * OBB.transpose()).transpose(); //8x3

        Eigen::MatrixXd pts_2d_hom = (CameraMat*CameraExtrinsicMat.inverse()  * OBB.transpose()).transpose(); //8x3
        int min_u, max_u, min_v, max_v;
        for(int i = 0; i < 8; ++i) {
            pts_2d_hom(i, 0) /= pts_2d_hom(i, 2);
            pts_2d_hom(i, 1) /= pts_2d_hom(i, 2);
            if(i == 0) {
                min_u = pts_2d_hom(i, 0); max_u = pts_2d_hom(i, 0);
                min_v = pts_2d_hom(i, 1); max_v = pts_2d_hom(i, 1);
            } else {
                min_u = std::min(min_u, static_cast<int>(pts_2d_hom(i, 0)));
                max_u = std::max(max_u, static_cast<int>(pts_2d_hom(i, 0)));
                min_v = std::min(min_v, static_cast<int>(pts_2d_hom(i, 1)));
                max_v = std::max(max_v, static_cast<int>(pts_2d_hom(i, 1)));
            }
            // std::cout<<"min_u: "<<min_u<<", max_u: "<< max_u<<" min_v: "<<min_v<<", max_v: "<< max_v<<std::endl;
        }

        // Declare cv Point to keep pixel coordinatres
        // Declare image_points vector to keep image coordinates
        // returned by CameraReproh->Project3Dpoint
        std::vector<cv::Point> image_points;
        for (int i = 0; i < 8; i++) {
            cv::Point image_point;
            image_point.x = pts_2d_hom(i, 0);
            image_point.y = pts_2d_hom(i, 1);
            image_points.push_back(image_point);
        }
        // Draw 12 lines that costructs box
        cv::Scalar clr = cv::Scalar(0, 255, 0);
        cv::Scalar clr_b = cv::Scalar(0, 255, 0);
        cv::Scalar clr_ta = cv::Scalar(0, 255, 0);

        // cv::Scalar clr = cv::Scalar(0, 0, 255);
        // cv::Scalar clr_b = cv::Scalar(255, 0, 0);
        // cv::Scalar clr_ta = cv::Scalar(0, 255, 255);

        if (!(min_u <= 0 || min_v <= 0 || max_u >= yolo_img.cols || max_v >= yolo_img.rows)) {
            cv::line(image_send, image_points[0], image_points[1], clr_b, 2, 8);
            cv::line(image_send, image_points[0], image_points[3], clr, 2, 8);
            cv::line(image_send, image_points[0], image_points[4], clr_ta, 2, 8);
            cv::line(image_send, image_points[1], image_points[2], clr, 2, 8);
            cv::line(image_send, image_points[1], image_points[5], clr_ta, 2, 8);
            cv::line(image_send, image_points[2], image_points[6], clr_ta, 2, 8);
            cv::line(image_send, image_points[2], image_points[3], clr_b, 2, 8);
            cv::line(image_send, image_points[3], image_points[7], clr_ta, 2, 8);
            cv::line(image_send, image_points[7], image_points[4], clr, 2, 8);
            cv::line(image_send, image_points[7], image_points[6], clr_b, 2, 8);
            cv::line(image_send, image_points[4], image_points[5], clr_b, 2, 8);
            cv::line(image_send, image_points[5], image_points[6], clr, 2, 8);

            // cv::rectangle(image_send, cv::Point(min_u, min_v), cv::Point(max_u, max_v), clr_b, 2);
        }
    }
    sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(),"bgr8",image_send).toImageMsg();
    sensor_msgs::Image img_pub = *img_msg;
    image_pub_.publish(img_pub);
    marker_pub_box_.publish(marker_array_box);
}

void ClearAllMarker() {
  visualization_msgs::MarkerArray::Ptr clear_marker_array(new visualization_msgs::MarkerArray);
  visualization_msgs::Marker dummy_marker;
  dummy_marker.action = visualization_msgs::Marker::DELETEALL;
  clear_marker_array->markers.push_back(dummy_marker);  
  marker_pub_box_.publish(clear_marker_array);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "processObject");
    ros::NodeHandle n("~");

    // load_Calibration("/home/xjtuiair/workspace/fusionDetect_ws/src/fusion_detect/param/calib.txt");

    pub_final_results = n.advertise<autoware_msgs::DetectedObjectArray>("/detection/final_result/objects", 30);
    image_pub_ = n.advertise<sensor_msgs::Image>("/fusion_image", 30);
    marker_pub_box_ = n.advertise<visualization_msgs::MarkerArray>("/lidar_perception/marker_box",1);
    // ros::Subscriber image_sub_ = n.subscribe<sensor_msgs::Image>("/image_raw",1,image_process);

    message_filters::Subscriber<autoware_msgs::DetectedObjectArray> sub_fusion(n, "/detection/fusion_tools/objects", 1);             // topic1 输入  
    message_filters::Subscriber<autoware_msgs::DetectedObjectArray> sub_pointpillar(n, "/detection/lidar_detector_3d/objects", 1);   // topic2 输入 
    message_filters::Subscriber<sensor_msgs::Image>sub_image(n,"/image_raw",1);
    TimeSynchronizer<autoware_msgs::DetectedObjectArray, autoware_msgs::DetectedObjectArray,sensor_msgs::Image> sync(sub_pointpillar, sub_fusion,sub_image, 30);       // 同步  
    sync.registerCallback(boost::bind(&callback, _1, _2, _3));    

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