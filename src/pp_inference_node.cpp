#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include "lidar_3d_detector/DetectedObjectArray.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cuda_runtime.h>
#include "pointpillar.h"  // 包含你的pointpillar头文件
#include <visualization_msgs/MarkerArray.h>

#define checkCudaErrors(status)                                   \
{                                                                 \
  if (status != 0)                                                \
  {                                                               \
    std::cout << "Cuda failure: " << cudaGetErrorString(status)   \
              << " at line " << __LINE__                          \
              << " in file " << __FILE__                          \
              << " error status: " << status                      \
              << std::endl;                                       \
              abort();                                            \
    }                                                             \
}

class PointPillarNode
{
public:
  PointPillarNode(ros::NodeHandle &nh)
  {
    // 参数初始化
    nh.param<std::vector<std::string>>("class_names", class_names, std::vector<std::string>());
    nh.param<float>("nms_iou_thresh", nms_iou_thresh, 0.01);
    nh.param<int>("pre_nms_top_n", pre_nms_top_n, 4096);
    nh.param<std::string>("lidar_input_topic", lidar_input_topic, "/rslidar");
    nh.param<std::string>("model_path", model_path, "");
    nh.param<std::string>("engine_path", engine_path, "");
    nh.param<std::string>("data_type", data_type, "fp16");
    nh.param<float>("intensity_scale", intensity_scale, 1.0);

    cudaStream_t stream = NULL;
    pointpillar = new PointPillar(model_path, engine_path, stream, data_type);
    // 发布与订阅
    publisher_ = nh.advertise<lidar_3d_detector::DetectedObjectArray>("bbox", 700);
    publisher_2 = nh.advertise<visualization_msgs::MarkerArray>("point_pillars_bbox", 1);
    subscriber_ = nh.subscribe(lidar_input_topic, 700, &PointPillarNode::topicCallback, this);
  }

private:
  std::vector<std::string> class_names;
  float nms_iou_thresh;
  int pre_nms_top_n;
  bool do_profile{false};
  std::string lidar_input_topic;
  std::string model_path;
  std::string engine_path;
  std::string data_type;
  float intensity_scale;
  tf2::Quaternion myQuaternion;
  cudaStream_t stream = NULL;
  PointPillar* pointpillar;

  ros::Publisher publisher_;
  ros::Publisher publisher_2;
  ros::Subscriber subscriber_;

  void topicCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    assert(data_type == "fp32" || data_type == "fp16");
    cudaEvent_t start, stop;
    float elapsedTime = 0.0f;
    cudaStream_t stream = NULL;

    checkCudaErrors(cudaEventCreate(&start));
    checkCudaErrors(cudaEventCreate(&stop));
    checkCudaErrors(cudaStreamCreate(&stream));

    std::vector<Bndbox> nms_pred;
    nms_pred.reserve(100);

    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg, *pcl_cloud);

    unsigned int points_size = pcl_cloud->points.size();
    std::vector<float> pcl_data;

    for (const auto& point : pcl_cloud->points) {
      pcl_data.push_back(point.x);
      pcl_data.push_back(point.y);
      pcl_data.push_back(point.z);
      pcl_data.push_back(point.intensity / intensity_scale);
    }

    float* points = static_cast<float *>(pcl_data.data());
    unsigned int points_data_size = points_size * sizeof(float) * 4;

    float *points_data = nullptr;
    unsigned int *points_num = nullptr;

    checkCudaErrors(cudaMallocManaged((void **)&points_data, points_data_size));
    checkCudaErrors(cudaMallocManaged((void **)&points_num, sizeof(unsigned int)));
    checkCudaErrors(cudaMemcpy(points_data, points, points_data_size, cudaMemcpyDefault));
    checkCudaErrors(cudaMemcpy(points_num, &points_size, sizeof(unsigned int), cudaMemcpyDefault));
    checkCudaErrors(cudaDeviceSynchronize());

    cudaEventRecord(start, stream);

    pointpillar->doinfer(points_data, points_num, nms_pred, nms_iou_thresh, pre_nms_top_n, class_names, do_profile);

    lidar_3d_detector::DetectedObjectArray pc_detection_arr;
    visualization_msgs::MarkerArray vis_pc_detection_arr;
    pc_detection_arr.header = msg->header;
    int marker_id = 0;  // 初始化marker的ID
    for (const auto& bbox : nms_pred) {
      lidar_3d_detector::DetectedObject detection;
      geometry_msgs::Quaternion orientation;
      detection.pose.position.x = bbox.x;
      detection.pose.position.y = bbox.y;
      detection.pose.position.z = bbox.z;
      detection.dimensions.x = bbox.l;
      detection.dimensions.y = bbox.w;
      detection.dimensions.z = bbox.h;

      myQuaternion.setRPY(0, 0, bbox.rt);
      orientation = tf2::toMsg(myQuaternion);
      detection.pose.orientation = orientation;
      detection.label = std::to_string(bbox.id);
      detection.score = bbox.score;

      detection.header = msg->header;
      pc_detection_arr.objects.push_back(detection);

      visualization_msgs::Marker marker;
      marker.type = visualization_msgs::Marker::LINE_STRIP;
      marker.header = msg->header;
      marker.action = visualization_msgs::Marker::ADD;
      marker.id = marker_id++;
      geometry_msgs::Point pos1, pos2, pos3, pos4, pos5, pos6, pos7, pos8;

      marker.pose = detection.pose;
      pos1.x = bbox.l / 2;
      pos1.y = bbox.w / 2;
      pos1.z = 0;

      pos2.x = bbox.l / 2;
      pos2.y = bbox.w / 2;
      pos2.z = -bbox.h;

      pos3.x = bbox.l / 2;
      pos3.y = -bbox.w / 2;
      pos3.z = -bbox.h;

      pos4.x = bbox.l / 2;
      pos4.y = -bbox.w / 2;
      pos4.z = 0;

      pos5.x = -bbox.l / 2;
      pos5.y = -bbox.w / 2;
      pos5.z = 0;

      pos6.x = -bbox.l / 2;
      pos6.y = -bbox.w / 2;
      pos6.z = -bbox.h;

      pos7.x = -bbox.l / 2;
      pos7.y = bbox.w / 2;
      pos7.z = -bbox.h;

      pos8.x = -bbox.l / 2;
      pos8.y = bbox.w / 2;
      pos8.z = 0;
      marker.points.push_back(pos1);
      marker.points.push_back(pos2);
      marker.points.push_back(pos3);
      marker.points.push_back(pos4);
      marker.points.push_back(pos5);
      marker.points.push_back(pos6);
      marker.points.push_back(pos7);
      marker.points.push_back(pos8);
      marker.points.push_back(pos1);
      marker.points.push_back(pos4);
      marker.points.push_back(pos3);
      marker.points.push_back(pos6);
      marker.points.push_back(pos5);
      marker.points.push_back(pos8);
      marker.points.push_back(pos7);
      marker.points.push_back(pos2);
      switch (bbox.id)
      {
      case 0: //Vehicle
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        break;
      case 1: //Pedestrian
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        break;
      case 2: //Cyclist
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        break;
      }
      marker.scale.x = 0.1;
      marker.color.a = 1.0;
      marker.lifetime.fromSec(0.1);
      vis_pc_detection_arr.markers.push_back(marker);
    }

    publisher_.publish(pc_detection_arr);
    publisher_2.publish(vis_pc_detection_arr);

    cudaEventRecord(stop, stream);
    cudaEventSynchronize(stop);
    cudaEventElapsedTime(&elapsedTime, start, stop);

    ROS_INFO("TIME: %.2f ms, Objects detected: %lu", elapsedTime, nms_pred.size());

    checkCudaErrors(cudaFree(points_data));
    checkCudaErrors(cudaFree(points_num));
    nms_pred.clear();

    checkCudaErrors(cudaEventDestroy(start));
    checkCudaErrors(cudaEventDestroy(stop));
    checkCudaErrors(cudaStreamDestroy(stream));
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "point_pillar_node");
  ros::NodeHandle nh;
  
  PointPillarNode node(nh);
  
  ros::spin();

  return 0;
}
