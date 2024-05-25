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
#include <nlink_distance/DistanceArray.h>
#include <Kalman_Filter.hpp>
// #include <geometry_msgs/PoseStamped.h>
#include <fstream> // 包含文件操作的头文件
#include <iostream> // 包含文件操作的头文件
std::vector<Eigen::Vector3d> target_pos(6);
nlink_distance::DistanceArray distance_msg;  //存储distance的msg
std::vector<double> get_time(6,0);
double distance,distance_kf_before;

int UWB_id = 1; //表示当前为哪个uwb模块
uint rec_data[8][20];

KalmanFilter kf(0.1, 0.1, 0.1, 0.0);  //初始化一个卡尔曼滤波器




void nodeframe2Callback(const nlink_parser::LinktrackNodeframe2 &msg)
{
  // std::cout<< msg << std::endl;
  for(auto& node:msg.nodes)
  {
    // std::cout<< node << std::endl;
    // get_time[node.id]=ros::Time::now().toSec();
    distance_kf_before = node.dis;
    distance = kf.filter(distance_kf_before);
    distance_msg.distances[node.id] =  distance;  //
    distance_msg.distances[UWB_id] = 0;
    // smooth_val = kf.filter(val);
  }
}

void nodeframe0Callback(const nlink_parser::LinktrackNodeframe0 &msg)
{
  // std::cout<< msg << std::endl;
  for(auto& node:msg.nodes)
  {
    // node.id
    for(int i = 0; i < node.data.size(); i++)
    {
      rec_data[node.id][i] = node.data[i];
      // std::cout<< rec_data[0][i] << std::endl;
    }

    // get_time[node.id]=ros::Time::now().toSec();

  }
}




int main(int argc, char **argv)
{
  ros::init(argc, argv, "linktrack_example");
  ros::NodeHandle nh;
  // target_pos.resize(4);//初始化4个


  // std::ofstream outFile0("/home/tgm/uwb0.txt");
  // if (!outFile0.is_open()) {
  //   ROS_ERROR("open file failed");
  //   return -1;
  // }
  // std::ofstream outFile1("/home/tgm/uwb1.txt");
  // if (!outFile1.is_open()) {
  //   ROS_ERROR("open file failed");
  //   return -1;
  // }


  ros::Subscriber sub = nh.subscribe("/nlink_linktrack_nodeframe2", 1000, nodeframe2Callback);   //订阅UWB发送的距离话题
  ros::Subscriber data_sub1 = nh.subscribe("/nlink_linktrack_nodeframe0", 1000, nodeframe0Callback);   //订阅UWB发送的数据话题
// 创建一个发布者，发布名为"/pose_stamped_topic"的话题，消息类型为geometry_msgs::PoseStamped
  ros::Publisher pub = nh.advertise<nlink_distance::DistanceArray>("/distance_topic", 10);
  // ros::Publisher pub = nh.advertise<nlink_distance::DistanceArray>("/distance_topic", 10);
  distance_msg.distances = {0, 0, 0, 0, 0, 0, 0, 0}; 


  // std::ofstream outFile2("/home/lqh/mapping/uwb2.txt");
  // if (!outFile2.is_open()) {
  //   ROS_ERROR("open file failed");
  //   return -1;
  // }
  // std::ofstream outFile3("/home/lqh/mapping/uwb3.txt");
  // if (!outFile3.is_open()) {
  //   ROS_ERROR("open file failed");
  //   return -1;
  // }
  // std::ofstream outFile4("/home/lqh/mapping/diff0.txt");
  // if (!outFile4.is_open()) {
  //   ROS_ERROR("open file failed");
  //   return -1;
  // }
  // std::ofstream outFile5("/home/lqh/mapping/diff1.txt");
  // if (!outFile5.is_open()) {
  //   ROS_ERROR("open file failed");
  //   return -1;
  // }

  // 循环发布消息
  ros::Rate loop_rate(20); // 设置发布频率为1Hz
  while (ros::ok()) {
    // if()
    // if(get_time[0]!=0 && get_time[1]!=0 && abs(get_time[0]-get_time[1])<2){
        
    //     // outFile0 << target_pos[0][0] <<", "<<target_pos[0][1] <<", " <<target_pos[0][2] << std::endl;
    //     // outFile1 << target_pos[1][0] <<", "<<target_pos[1][1] <<", " <<target_pos[1][2] << std::endl;
    //     // auto tmpa=target_pos[1]-target_pos[0];
    //     // outFile4 << tmpa[0] <<", "<<tmpa[1] <<", " <<tmpa[2] << std::endl;        
        
    //     // 填充消息对象的内容

    //     poseStampedMsg.header.stamp = ros::Time::now();
    //     poseStampedMsg.header.frame_id = "0"; // 设置坐标系
    //     auto tmp=(target_pos[0]+target_pos[1])/2;


    //     poseStampedMsg.pose.position.x = tmp.x();   // 设置位置
    //     poseStampedMsg.pose.position.y = tmp.y();
    //     poseStampedMsg.pose.position.z = tmp.z();

    //     poseStampedMsg.pose.orientation.x = target_pos[0].x(); // 设置方向
    //     poseStampedMsg.pose.orientation.y = target_pos[0].y();
    //     poseStampedMsg.pose.orientation.z = target_pos[1].x();
    //     poseStampedMsg.pose.orientation.w = target_pos[1].y();

    //     pub.publish(poseStampedMsg); // 发布消息
    //     get_time[0]=0;
    //     get_time[1]=0;
        
    //   }

    //   if(get_time[2]!=0 && get_time[3]!=0 && abs(get_time[2]-get_time[3])<2){

    //     // outFile2 << target_pos[2][0] <<", "<<target_pos[2][1] <<", " <<target_pos[2][2] << std::endl;
    //     // outFile3 << target_pos[3][0] <<", "<<target_pos[3][1] <<", " <<target_pos[3][2] << std::endl;
    //     // auto tmpa=target_pos[3]-target_pos[2];
    //     // outFile5 << tmpa[0] <<", "<<tmpa[1] <<", " <<tmpa[2] << std::endl;  

    //     // 填充消息对象的内容
    //     poseStampedMsg.header.stamp = ros::Time::now();
    //     poseStampedMsg.header.frame_id = "1"; // 设置坐标系
    //     auto tmp=(target_pos[2]+target_pos[3])/2;

    //     poseStampedMsg.pose.position.x = tmp.x();   // 设置位置
    //     poseStampedMsg.pose.position.y = tmp.y();
    //     poseStampedMsg.pose.position.z = tmp.z();

    //     poseStampedMsg.pose.orientation.x = target_pos[2].x(); // 设置方向
    //     poseStampedMsg.pose.orientation.y = target_pos[2].y();
    //     poseStampedMsg.pose.orientation.z = target_pos[3].x();
    //     poseStampedMsg.pose.orientation.w = target_pos[3].y();

    //     pub.publish(poseStampedMsg); // 发布消息
    //     get_time[2]=0;
    //     get_time[3]=0;
    //   }

    //   if(get_time[4]!=0 && get_time[5]!=0 && abs(get_time[4]-get_time[5])<2){
    //     // outFile0 << target_pos[0][0] <<", "<<target_pos[0][1] <<", " <<target_pos[0][2] << std::endl;
    //     // outFile1 << target_pos[1][0] <<", "<<target_pos[1][1] <<", " <<target_pos[1][2] << std::endl;
    //     // auto tmpa=target_pos[1]-target_pos[0];
    //     // outFile4 << tmpa[0] <<", "<<tmpa[1] <<", " <<tmpa[2] << std::endl;        
        
    //     // 填充消息对象的内容

    //     poseStampedMsg.header.stamp = ros::Time::now();
    //     poseStampedMsg.header.frame_id = "2"; // 设置坐标系
    //     auto tmp=(target_pos[4]+target_pos[5])/2;


    //     poseStampedMsg.pose.position.x = tmp.x();   // 设置位置
    //     poseStampedMsg.pose.position.y = tmp.y();
    //     poseStampedMsg.pose.position.z = tmp.z();

    //     poseStampedMsg.pose.orientation.x = target_pos[4].x(); // 设置方向
    //     poseStampedMsg.pose.orientation.y = target_pos[4].y();
    //     poseStampedMsg.pose.orientation.z = target_pos[5].x();
    //     poseStampedMsg.pose.orientation.w = target_pos[5].y();
        // outFile0 << distance_kf_before << std::endl;
        // outFile1 << distance << std::endl;
        pub.publish(distance_msg); // 发布消息
    //     get_time[4]=0;
    //     get_time[5]=0;
        
    //   }

      ros::spinOnce();             // 处理回调函数
      loop_rate.sleep();           // 休眠以维持发布频率
  }
  return 0;
}
