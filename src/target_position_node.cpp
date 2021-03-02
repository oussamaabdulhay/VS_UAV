#include "ros/ros.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include "geometry_msgs/PointStamped.h"
#include "sensor_msgs/CameraInfo.h"


ros::Publisher pub_object_pos;
tf2_ros::Buffer tf_Buffer;
geometry_msgs::TransformStamped cam_to_cam_fixed, cam_to_body, t_stamped;
geometry_msgs::PointStamped X_m;
geometry_msgs::Point X_px_c, X_px_c_rotated;
tf2::Vector3 T;
tf2::Matrix3x3 R;
float _depth = 3.8, _f = 616.5, _cx = 320, _cy = 240;
float roll, pitch, yaw, yawrt;
ros::Time yaw_t;

void cam_info_callback(const sensor_msgs::CameraInfo& cam_info){
    _f = cam_info.K[0];
    _cx = cam_info.K[2];
    _cy = cam_info.K[5];
}

void pixelCallback(const geometry_msgs::PointStamped& msg){
    X_px_c.y = -1*(msg.point.x -_cx);
    X_px_c.z = -1*(msg.point.y -_cy);
    X_px_c.x = _f;
    ros::Time msg_time = msg.header.stamp;
    
  try{
     cam_to_cam_fixed = tf_Buffer.lookupTransform("body_fixed", "body", msg_time, ros::Duration(0.1));
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("Failure %s\n", ex.what()); //Print exception which was caught
  }
  R.setRotation(tf2::Quaternion(cam_to_cam_fixed.transform.rotation.x, cam_to_cam_fixed.transform.rotation.y, cam_to_cam_fixed.transform.rotation.z, cam_to_cam_fixed.transform.rotation.w));
  tf2::doTransform(X_px_c, X_px_c_rotated, cam_to_cam_fixed); //Cam to cam_fixed
  
  tf2::Vector3 X, X_t;
  X.setX((_depth * X_px_c_rotated.x)/X_px_c_rotated.y);
  X.setY(_depth);
  X.setZ((_depth * X_px_c_rotated.z)/X_px_c_rotated.y);
  X_t = X + R*T; //cam fixed to body fixed
  X_m.header.frame_id = "body_fixed";
  X_m.header.stamp = msg_time;
  X_m.point.x = X_t.getX();
  X_m.point.y = X_t.getY();
  X_m.point.z = X_t.getZ();

  pub_object_pos.publish(X_m);

}

void yawrtCallback(const geometry_msgs::Point& msg){
  yawrt = msg.x;
}
void yawCallback(const geometry_msgs::Point& msg){
  yaw = msg.x;
  yaw_t = ros::Time::now();
}

void rollCallback(const geometry_msgs::Point& msg){
    roll=msg.x;
}

void pitchCallback(const geometry_msgs::Point& msg){
    pitch = msg.x;
    ros::Time msg_time = ros::Time::now();
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    geometry_msgs::PointStamped X_m_body, target_pos;

  transformStamped.header.stamp = msg_time;
  transformStamped.header.frame_id = "body_fixed";
  transformStamped.child_frame_id = "body";
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw+yawrt*(ros::Duration(msg_time - yaw_t).toSec()));
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();
  br.sendTransform(transformStamped);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "position_node");
  ros::NodeHandle nh;
  tf2_ros::TransformListener tfListener(tf_Buffer);
   
  ros::Subscriber sub_pixel = nh.subscribe("/ball_center",10, &pixelCallback);
  ros::Subscriber cam_info_sub = nh.subscribe("/camera/color/camera_info", 10, &cam_info_callback);
  pub_object_pos = nh.advertise<geometry_msgs::PointStamped>("/target_point", 1);

  ros::Subscriber sub_roll = nh.subscribe("/providers/roll", 10, &rollCallback);
  ros::Subscriber sub_pitch = nh.subscribe("/providers/pitch", 10, &pitchCallback);
  ros::Subscriber sub_yaw = nh.subscribe("/providers/yaw", 10, &yawCallback);
  ros::Subscriber sub_yaw_rt = nh.subscribe("/providers/yaw_rate", 10, &yawrtCallback);
  cam_to_body = tf_Buffer.lookupTransform("body", "camera", ros::Time(0), ros::Duration(5.0));

  T = tf2::Vector3(cam_to_body.transform.translation.x, cam_to_body.transform.translation.y, cam_to_body.transform.translation.z);

  ros::spin();
  return 0;
};