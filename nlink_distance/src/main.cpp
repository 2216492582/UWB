#include <nlink_parser/IotFrame0.h>
#include <nlink_parser/LinktrackAnchorframe0.h>
#include <nlink_parser/LinktrackAoaNodeframe0.h>
#include <nlink_parser/LinktrackNodeframe0.h>
#include <nlink_parser/LinktrackNodeframe1.h>
#include <nlink_parser/LinktrackNodeframe2.h>
#include <nlink_parser/LinktrackNodeframe3.h>
#include <nlink_parser/LinktrackNodeframe4.h>
#include <nlink_parser/LinktrackNodeframe5.h>
#include <nlink_parser/LinktrackNodeframe6.h>
#include <nlink_parser/LinktrackTagframe0.h>
#include <nlink_parser/TofsenseFrame0.h>
#include <nlink_parser/TofsenseMFrame0.h>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <nlink_distance/DistanceArray_all.h>
#include <nlink_distance/DistanceArray_oneself.h>
#include <Kalman_Filter.hpp>
#include <tool.hpp>
// #include <geometry_msgs/PoseStamped.h>
#include <fstream> // 包含文件操作的头文件
#include <iostream> // 包含文件操作的头文件
#include <vector>
#include <sstream>
#include <algorithm>
#include <nav_msgs/Odometry.h>

// #define UWB_id  0 //表示当前为哪个uwb模块（测速使用，后续从msg中判断属于哪个模块）
#define DATA_length  32 //表示数传模式接收数据数组长度
#define UWB_capacity 8 //表示uwb模块数量

nlink_distance::DistanceArray_oneself distance_msg;  //存储distance的msg
nlink_distance::DistanceArray_all distance_all_msg;  //存储distance_all的msg

nav_msgs::Odometry Broadcast_msg;  //存储distance_all的msg


std::vector<double> get_time(6,0);
double distance,distance_kf_before;  //卡尔曼滤波前后的距离值

int UWB_id = 0;//表示当前为哪个uwb模块

// uint rec_data[8][32]; //接收到的uwb数传数据，后续使用二维vector实现
std::vector<std::vector<uint8_t> > rec_data_uwb(8,std::vector<uint8_t>(DATA_length,0));

std::vector<uint8_t > rec_data_broadcast(44);  //接收到的广播数据

std::vector<std::vector<float> > distance_DATA(8,std::vector<float>(8,0));
std::vector<float> Broadcast_DATA(11);

KalmanFilter kf(0.1, 15, 1, 0.0);  //初始化一个卡尔曼滤波器

//将字节数据转换回float数组
// Function to convert byte array back to float array
std::vector<float> convertBytesToFloatArray(const std::vector<uint8_t>& byteData) {
    size_t floatCount = byteData.size() / sizeof(float);
    std::vector<float> floatArray(floatCount);
    std::memcpy(floatArray.data(), byteData.data(), byteData.size());
    return floatArray;
}

//将接收到的uwb数据使转为msg
void Vector2Msg(int UWB_id,int index,float data) {
  switch (UWB_id)
  {
  case 0:
    distance_all_msg.uwb0[index] = data;
    break;
  case 1:
    distance_all_msg.uwb1[index] = data;
    break;
  case 2:
    distance_all_msg.uwb2[index] = data;
    break;
  case 3:
    distance_all_msg.uwb3[index] = data;
    break;
  case 4:
    distance_all_msg.uwb4[index] = data;
    break;
  case 5:
    distance_all_msg.uwb5[index] = data;
    break;
  case 6:
    distance_all_msg.uwb6[index] = data;
    break;
  case 7:
    distance_all_msg.uwb7[index] = data;
    break;
  default:
    break;
  }
  // std::memcpy(arr, vec.data(), vec.size() * sizeof(float));
}

//将接收到的广播数据使转为msg
void vector2BroadcastMsg(void)
{
  Broadcast_msg.pose.pose.position.x = Broadcast_DATA[0];
  Broadcast_msg.pose.pose.position.y = Broadcast_DATA[1];
  Broadcast_msg.pose.pose.position.z =  Broadcast_DATA[2];
  Broadcast_msg.pose.pose.orientation.x = Broadcast_DATA[3];
  Broadcast_msg.pose.pose.orientation.y = Broadcast_DATA[4];
  Broadcast_msg.pose.pose.orientation.z = Broadcast_DATA[5];
  Broadcast_msg.pose.pose.orientation.w = Broadcast_DATA[6];
  Broadcast_msg.pose.covariance[0] = Broadcast_DATA[7];
  Broadcast_msg.pose.covariance[1] = Broadcast_DATA[8];
  Broadcast_msg.twist.covariance[0] = Broadcast_DATA[9];
  Broadcast_msg.twist.covariance[1] = Broadcast_DATA[10];
}

//nodeframe2的回调函数，用于接受uwb的相对测距数据
void nodeframe2Callback(const nlink_parser::LinktrackNodeframe2 &msg)
{
  UWB_id = msg.id;
  for(auto& node:msg.nodes)
  {
    distance_kf_before = node.dis;
    distance = kf.filter(distance_kf_before); //测距值抖动较大，使用卡尔曼滤波器
    distance_msg.distances[node.id] =  distance;  
    distance_msg.distances[UWB_id] = 0;       //uwb模块自身与自身的距离值设置为0
  }
  for (int i = 0; i < 8; ++i)   //将本模块与其他模块的距离数组元素赋值给distance_DATA
  {
    distance_DATA[UWB_id][i] = distance_msg.distances[i];
    Vector2Msg(UWB_id,i,distance_msg.distances[i]);
  }
}


//nodeframe0的回调函数，用于接受uwb的数传数据
void nodeframe0Callback(const nlink_parser::LinktrackNodeframe0 &msg)
{
  for(auto& node:msg.nodes)
  {
    // node.id代表不同uwb广播发出的数据
    for(int i = 0; i < node.data.size(); i++)
    {
      //接受到的uwb数据
      if(i < 32)
      {
        rec_data_uwb[node.id][i] = node.data[i]; // 前32个数据为uwb数据
      }
      if(i >= 32)
      {
        rec_data_broadcast[i] = node.data[i]; // 前32个数据为uwb数据
      }
    }
    // convertBytesToFloatArray(rec_data[node.id])
    distance_DATA[node.id] = convertBytesToFloatArray(rec_data_uwb[node.id]);  //将数据转换为float数组
    Broadcast_DATA = convertBytesToFloatArray(rec_data_broadcast);  //将接收到的广播数据转换会float数组
    vector2BroadcastMsg(); //将接收到的广播数据使转为msg
    for (int i = 0; i < 8; ++i)   //将本模块与其他模块的距离数组元素赋值给distance_DATA
    {
      Vector2Msg(node.id,i,distance_DATA[node.id][i]);
    }
  }
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "linktrack_example");
  ros::NodeHandle nh;
  //测试使用
  // std::ofstream outFile0("/home/tgm/uwb0.txt");
  // if (!outFile0.is_open()) {
  //   ROS_ERROR("open file failed");
  //   return -1;
  // }
  ros::Subscriber sub = nh.subscribe("/nlink_linktrack_nodeframe2", 1000, nodeframe2Callback);   //订阅UWB发送的距离话题
  ros::Subscriber data_sub1 = nh.subscribe("/nlink_linktrack_nodeframe0", 1000, nodeframe0Callback);   //订阅UWB发送的数据话题
  //创建发布者，发布距离信息，此处发布有8个元素的数组，表示该模块到其他模块的距离，模块号对应的数组元素为0
  ros::Publisher distance_onself_pub = nh.advertise<nlink_distance::DistanceArray_oneself>("/distance_topic", 10); 
  ros::Publisher distance_all_pub = nh.advertise<nlink_distance::DistanceArray_all>("/distance_topic_all", 10); 
  ros::Publisher Broadcast_pub = nh.advertise<nav_msgs::Odometry>("/communicate_client", 10); 
  distance_msg.distances = {0, 0, 0, 0, 0, 0, 0, 0};
   
  // 循环发布消息
  ros::Rate loop_rate(20); // 设置发布频率为20Hz
  while (ros::ok()) {

      distance_onself_pub.publish(distance_msg); // 发布消息
      distance_all_pub.publish(distance_all_msg); // 发布消息
      Broadcast_pub.publish(Broadcast_msg);//发布广播信息


      ros::spinOnce();             // 处理回调函数
      loop_rate.sleep();           // 休眠以维持发布频率
  }
  return 0;
}
