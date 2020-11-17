// ROS Libs
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include "serial/serial.h"
#include <ros/console.h>

// Native_Libs
#include <string>
#include <cmath>
#include <iostream>

// ROS Node and Publishers
ros::NodeHandle * nh;
ros::NodeHandle * pnh;
ros::Publisher odom_pub;

// ROS Callbacks
void twist_callback(const geometry_msgs::Twist& msg);
void update_callback(const ros::TimerEvent &);

// ROS Params
int covar_samples;
float frequency;
int baud;
std::string port;
std::string frame_id;
std::string child_frame_id;

// Custom Types
typedef struct {
  // Pose
  float x;
  float y;
  float theta;
  // Twist
  float dx;
  float omega;
} OdomEntry;

// Constants
const std::string eol = "\xAA\xAA";

// Global_Vars
serial::Serial * serialPort;
std::vector<OdomEntry> odomHistory;
int odom_seq = 0;
static const float DEG_TO_RAD = M_PI / 180.0F;
float cmdFwd = 0;
float cmdRot = 0;

// Utility Methods
int32_t buffer_pop_int32(uint8_t const* buffer, int* index);
void buffer_put_int32(uint8_t* buffer, int* index, int32_t value);
int16_t buffer_pop_int16(uint8_t const* buffer, int* index);
void buffer_put_int16(uint8_t* buffer, int* index, int16_t value);
void buffer_put_float32(uint8_t* buffer, int* index, float value);
float buffer_pop_float32(uint8_t* buffer, int* index);
bool calculate_covariance(boost::array<double, 36> &pose_mat, boost::array<double, 36> &twist_mat);

int main(int argc, char** argv) {
  // Init ROS
  ros::init(argc, argv, "rova_bridge_node");
  nh = new ros::NodeHandle();
  pnh = new ros::NodeHandle("~");

  // Params
  pnh->param<std::string>("pose_frame_id", frame_id, "odom");
  pnh->param<std::string>("twist_frame_id", child_frame_id, "base_link");
  pnh->param<int>("covar_samples", covar_samples, 10);
  pnh->param<float>("frequency", frequency, 50);
  pnh->param<std::string>("port", port, "/dev/ttyUSB0");
  pnh->param<int>("baud", baud, 115200);

  // Setup node
  odomHistory.resize(covar_samples);
  serialPort = new serial::Serial(port, baud, serial::Timeout::simpleTimeout(100));

  // Subscribers
  ros::Subscriber twist_sub = nh->subscribe("cmd_vel", 5, twist_callback);

  // Publishers
  odom_pub = nh->advertise<nav_msgs::Odometry>("odom", 5);

  // Timers
  ros::Timer update_timer = nh->createTimer(ros::Duration(1.0 / frequency), update_callback);

  // Spin
  ros::spin();
}

void update_callback(const ros::TimerEvent &) {
  uint8_t buffer[8];
  int index = 0;
  buffer_put_float32(buffer, &index, cmdFwd);
  buffer_put_float32(buffer, &index, cmdRot);

  if (serialPort->isOpen()) {
    serialPort->write(buffer, 8);
    std::string readstr;
    int count = serialPort->readline(readstr, 65536, eol);
    if (count == 14) {
      uint8_t readbuf[14];
      memcpy(readbuf, readstr.c_str(), 14); // c++ won't let me reinterpret
      int idx = 0;
      float velFwd = buffer_pop_float32(readbuf, &idx);
      float velRot = buffer_pop_float32(readbuf, &idx);

      OdomEntry entry = odomHistory[odom_seq % covar_samples];
      entry.x = 0;
      entry.y = 0;
      entry.theta = 0;
      entry.dx = velFwd;
      entry.omega = velRot;

      nav_msgs::Odometry odom_msg;
      // Add header
      odom_msg.header.stamp = ros::Time::now();
      odom_msg.header.seq = odom_seq++;
      odom_msg.header.frame_id = frame_id;
      odom_msg.child_frame_id = child_frame_id;

      // Put data in message
      // We only use a subset of the total pose/twist pair, so some fields are permanently zero
      odom_msg.pose.pose.position.x = entry.x;
      odom_msg.pose.pose.position.y = entry.y;
      odom_msg.pose.pose.position.z = 0.0f;
      odom_msg.pose.pose.orientation.x = 0.0f;
      odom_msg.pose.pose.orientation.y = 0.0f;
      odom_msg.pose.pose.orientation.z = sin(entry.theta / 2.0f); // Very simple way to make a quaternion around one axis
      odom_msg.pose.pose.orientation.w = cos(entry.theta / 2.0f);
      odom_msg.twist.twist.linear.x = entry.dx;
      odom_msg.twist.twist.linear.y = 0.0f;
      odom_msg.twist.twist.linear.z = 0.0f;
      odom_msg.twist.twist.angular.x = 0.0f;
      odom_msg.twist.twist.angular.y = 0.0f;
      odom_msg.twist.twist.angular.z = entry.omega;

      // Calculate covariance and publish
      if (calculate_covariance(odom_msg.pose.covariance, odom_msg.twist.covariance)) {
        odom_pub.publish(odom_msg);
      }
    }
  }
  else
  {
    ROS_ERROR("Failed to open port at %s", port);
  }
  
}

void twist_callback(const geometry_msgs::Twist& msg) {
  cmdFwd = msg.linear.x;
  cmdRot = msg.angular.z;
}

/**
 * Calculates the covariance matrices based on the odom history and stores the results in the provided arrays
 * Returns true if the returned covariance is valid, otherwise false
 */
bool calculate_covariance(boost::array<double, 36> &pose_mat, boost::array<double, 36> &twist_mat) {
  int count = std::min(odom_seq, covar_samples);
  if (count < 2) {
    return false; // Did not calculate covariance
  }
  OdomEntry avg = {};
  // Calculate averages
  for (int i = 0; i < count; i++) {
    avg.x += odomHistory[i].x;
    avg.y += odomHistory[i].y;
    avg.theta += odomHistory[i].theta;
    avg.dx += odomHistory[i].dx;
    avg.omega += odomHistory[i].omega;
  }
  avg.x /= count;
  avg.y /= count;
  avg.theta /= count;
  avg.dx /= count;
  avg.omega /= count;
  float avg_pose[] = {avg.x, avg.y, 0.0f, 0.0f, 0.0f, avg.theta};
  float avg_twist[] = {avg.dx, 0.0f, 0.0f, 0.0f, 0.0f, avg.omega};
  // Calculate covariance
  // See https://en.wikipedia.org/wiki/Covariance#Calculating_the_sample_covariance
  for (int i = 0; i < count; i++) {
    float item_pose[] = {odomHistory[i].x, odomHistory[i].y, 0.0f, 0.0f, 0.0f, odomHistory[i].theta};
    float item_twist[] = {odomHistory[i].dx, 0.0f, 0.0f, 0.0f, 0.0f, odomHistory[i].omega};
    for (int x = 0; x < 6; x++) {
      for (int y = 0; y < 6; y++) {
        int idx = 6*x + y;
        pose_mat[idx] = 0;
        twist_mat[idx] = 0;
        // Average mean error difference
        
        pose_mat[idx] += (item_pose[x] - avg_pose[x]) * (item_pose[y] - avg_pose[y]);
        twist_mat[idx] += (item_twist[x] - avg_twist[x]) * (item_twist[y] - avg_twist[y]);
        
        // Normalize
        pose_mat[idx] /= count - 1;
        twist_mat[idx] /= count - 1;
      }
    }
  }
  return true;
}

int32_t buffer_pop_int32(uint8_t const* buffer, int* index) {
  int32_t buf = 0;
  buf = buffer[(*index)++] << 24;
  buf |= buffer[(*index)++] << 16;
  buf |= buffer[(*index)++] << 8;
  buf |= buffer[(*index)++];
  return buf;
}

void buffer_put_int32(uint8_t* buffer, int* index, int32_t const value) {
  buffer[(*index)++] = value >> 24;
  buffer[(*index)++] = value >> 16;
  buffer[(*index)++] = value >> 8;
  buffer[(*index)++] = value;
}

int16_t buffer_pop_int16(uint8_t const* buffer, int* index) {
  int16_t buf = 0;
  buf |= buffer[(*index)++] << 8;
  buf |= buffer[(*index)++];
  return buf;
}

void buffer_put_int16(uint8_t* buffer, int* index, int16_t const value) {
  buffer[(*index)++] = value >> 8;
  buffer[(*index)++] = value;
}

float buffer_pop_float32(uint8_t* buffer, int* index) {
  float buf = 0;
  memcpy(&buf, buffer + (*index), sizeof(float));
  *index += sizeof(float);
  return buf;
}

void buffer_put_float32(uint8_t* buffer, int* index, float const value) {
  memcpy(buffer + (*index), &value, sizeof(float));
  *index += sizeof(float);
}