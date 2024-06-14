#include <ros/ros.h>

#include "init.h"
#include "init_serial.h"
#include "protocol_extracter/nprotocol_extracter.h"

#include <iomanip>
#include <iostream>
#include <vector>
#include <nlink_distance/DistanceArray_oneself.h>
#include <nav_msgs/Odometry.h>

// nlink_distance::DistanceArray distance_msg;  //存储distance的msg
float distance_array[8];  //uwb距离数组
float Broadcast_data[11];  //广播转发数据使用数组

void printHexData(const std::string &data) {
  if (!data.empty()) {
    std::cout << "data received: ";
    for (int i = 0; i < data.size(); ++i) {
      std::cout << std::hex << std::setfill('0') << std::setw(2)
                << int(uint8_t(data.at(i))) << " ";
    }
    std::cout << std::endl;
  }
}

//将float数组转换为字节数据
std::vector<uint8_t> convertFloatArrayToBytes(const float* array, size_t arraySize) {
    // Calculate the number of bytes needed
    size_t byteSize = arraySize * sizeof(float);
    // Create a vector to hold the byte representation of the float array
    std::vector<uint8_t> byteData(byteSize);
    // Copy the float array into the byteData vector
    std::memcpy(byteData.data(), array, byteSize);

    return byteData;
}

//将字节数据转换回float数组
// Function to convert byte array back to float array
std::vector<float> convertBytesToFloatArray(const std::vector<uint8_t>& byteData) {
    size_t floatCount = byteData.size() / sizeof(float);
    std::vector<float> floatArray(floatCount);
    std::memcpy(floatArray.data(), byteData.data(), byteData.size());
    return floatArray;
}


std::vector<uint8_t> byteData;//用于存储float转化为byte的数据 uwb数组
std::vector<uint8_t> Forward_byteData;//用于存储float转化为byte的数据 广播数据数组
//DistanceMsg的回调函数，用于接受距离数据，并将其转化为字节数据存储，等待后续发送
void DistanceMsgCallback(const nlink_distance::DistanceArray_oneself &msg)
{
  // distance_msg = msg;
  // std::cout << msg << std::endl;
  for(int i = 0; i < msg.distances.size(); i++){
    distance_array[i] = msg.distances[i];
  }
  byteData = convertFloatArrayToBytes(distance_array, 8);
  // std::cout << distance_array[2] << std::endl;  //长度为32
}


//communicate_server的回调函数，用于接受广播数据，并将其转化为字节数据存储，等待后续发送
void communicate_MsgCallback(const nav_msgs::Odometry &msg)
{
  Broadcast_data[0] = msg.pose.pose.position.x; 
  Broadcast_data[1] = msg.pose.pose.position.y; 
  Broadcast_data[2] = msg.pose.pose.position.z;
  Broadcast_data[3] = msg.pose.pose.orientation.x; 
  Broadcast_data[4] = msg.pose.pose.orientation.y; 
  Broadcast_data[5] = msg.pose.pose.orientation.z;
  Broadcast_data[6] = msg.pose.pose.orientation.w; 
  Broadcast_data[7] = msg.pose.covariance[0];
  Broadcast_data[8] = msg.pose.covariance[1];
  Broadcast_data[9] = msg.twist.covariance[0];  
  Broadcast_data[10] = msg.twist.covariance[1]; 
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "linktrack_parser");
  ros::NodeHandle nh;
  serial::Serial serial;
  initSerial(&serial);
  NProtocolExtracter protocol_extraction;
  linktrack::Init init(&protocol_extraction, &serial);
  ros::Subscriber data_sub1 = nh.subscribe("/distance_topic", 1000, DistanceMsgCallback);   //订阅距离话题
  ros::Subscriber data_sub2 = nh.subscribe("/communicate_server", 1000, DistanceMsgCallback);   //订阅距离话题
  ros::Rate loop_rate(1000);

  while (ros::ok()) {
    // std::cout << byteData.size() << std::endl;
    auto available_bytes = serial.available();
    std::string str_received;
    if (available_bytes) {
      serial.read(str_received, available_bytes);
      serial.write(byteData); //串口写函数
      Forward_byteData = convertFloatArrayToBytes(Broadcast_data,11);
      serial.write(Forward_byteData); //串口写函数
      // printHexData(str_received);
      protocol_extraction.AddNewData(str_received);
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  return EXIT_SUCCESS;
}
