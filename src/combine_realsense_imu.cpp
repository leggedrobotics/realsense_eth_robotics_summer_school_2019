#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <mutex>



class ImuCombiner {
public:

  ImuCombiner(ros::NodeHandle& nh) : nh_(nh) {
    gyro_sub_ = nh_.subscribe("gyro", 10, &ImuCombiner::gyroCallback, this);
    accel_sub_ = nh_.subscribe("accel", 10, &ImuCombiner::accelCallback, this);

    imu_pub_ = nh_.advertise<sensor_msgs::Imu>("imu", 10);
  }

private:


  void gyroCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(imu_msg_mutex_);

    imu_msg_.header = msg->header;
    imu_msg_.angular_velocity = msg->angular_velocity;
    imu_msg_.angular_velocity_covariance = msg->angular_velocity_covariance;

    imu_pub_.publish(imu_msg_);
  }



  void accelCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(imu_msg_mutex_);

    imu_msg_.header = msg->header;
    imu_msg_.linear_acceleration = msg->linear_acceleration;
    imu_msg_.linear_acceleration_covariance = msg->linear_acceleration_covariance;

    imu_pub_.publish(imu_msg_);
  }

  

  ros::NodeHandle nh_;
  ros::Publisher imu_pub_;
  ros::Subscriber gyro_sub_;
  ros::Subscriber accel_sub_;

  sensor_msgs::Imu imu_msg_{};
  std::mutex imu_msg_mutex_;
};



int main(int argc, char** argv) {
  ros::init(argc, argv, "realsense_imu_combiner");
  ros::NodeHandle nh("~");
  ImuCombiner combiner(nh);

  ros::spin();
}
