#include <string>

#include <geometry_msgs/PoseStamped.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/param.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Temperature.h>
#include <tf/transform_broadcaster.h>

#include <bno055_usb_stick/bno055_usb_stick.hpp>
#include <bno055_usb_stick/decoder.hpp>
#include <bno055_usb_stick_msgs/Output.h>

#include <boost/asio/io_service.hpp>
#include <boost/shared_ptr.hpp>

#include <pigpiod_if2.h>

namespace bus = bno055_usb_stick;

int pigpio_ptr = 0;
int trig_seq = 0;
int trigger_counter = 0;
std::string trig_frame_id = "external_cam_trigger";
ros::Publisher trigger_pub;

ros::Publisher out_pub;
ros::Publisher imu_pub;
ros::Publisher pose_pub;
std::string pose_frame_id;
ros::Publisher mag_pub;
ros::Publisher temp_pub;
boost::shared_ptr< tf::TransformBroadcaster > tf_pub;
std::string tf_frame_id, tf_child_frame_id;
bool invert_tf;
bool trigger_enable;
int trigger_divider;
int gpio_num;

void publish(const bno055_usb_stick_msgs::Output &output) {
  const ros::Time cbk_time = ros::Time::now();
  if (out_pub.getNumSubscribers() > 0) {
    out_pub.publish(output);
  }
  if (imu_pub.getNumSubscribers() > 0) {
    trigger_counter++;
    trigger_counter = trigger_counter % trigger_divider;
    if (trigger_counter == 0) {
      if (trigger_enable) {
      if (0 != gpio_trigger(pigpio_ptr, gpio_num, 100, 1)) {
	ROS_INFO("Trigger Error");
	}
      }
      std_msgs::Header trig;
      trig.stamp = cbk_time;
      trig.seq = trig_seq++;
      trig.frame_id = trig_frame_id;
      trigger_pub.publish(trig);
    }
    imu_pub.publish(bus::Decoder::toImuMsg(output));
  }
  if (pose_pub.getNumSubscribers() > 0) {
    pose_pub.publish(bus::Decoder::toPoseMsg(output, pose_frame_id));
  }
  if (mag_pub.getNumSubscribers() > 0) {
    mag_pub.publish(bus::Decoder::toMagMsg(output));
  }
  if (temp_pub.getNumSubscribers() > 0) {
    temp_pub.publish(bus::Decoder::toTempMsg(output));
  }
  if (tf_pub) {
    tf_pub->sendTransform(
        bus::Decoder::toTFTransform(output, tf_frame_id, tf_child_frame_id, invert_tf));
  }
}

int main(int argc, char *argv[]) {
  // init ROS
  ros::init(argc, argv, "bno055_usb_stick_node");
  ros::NodeHandle nh;
  
  // load parameters
  pose_frame_id = ros::param::param< std::string >("~pose_frame_id", "fixed");
  const bool publish_tf(ros::param::param("~publish_tf", false));
  tf_frame_id = ros::param::param< std::string >("~tf_frame_id", "fixed");
  tf_child_frame_id = ros::param::param< std::string >("~tf_child_frame_id", "bno055");
  invert_tf = ros::param::param("~invert_tf", false);
  trigger_divider = ros::param::param("~trigger_divider", 5);
  gpio_num = ros::param::param("~gpio_num", 23);
  trigger_enable = ros::param::param("~trigger_enable", true);

  // init pigpio
  if (trigger_enable) {
	  pigpio_ptr = pigpio_start(NULL, NULL);
	  if (pigpio_ptr < 0) return 1;

	  if (0 != set_mode(pigpio_ptr, gpio_num, PI_OUTPUT)) return 1;
	  if (0 != set_pull_up_down(pigpio_ptr, gpio_num, PI_PUD_DOWN)) return 1;
	  if (0 != gpio_write(pigpio_ptr, gpio_num, 0)) return 1;
  }

  // setup publishers
  trigger_pub = nh.advertise< std_msgs::Header >("trigger", 1);
  out_pub = nh.advertise< bno055_usb_stick_msgs::Output >("output", 1);
  imu_pub = nh.advertise< sensor_msgs::Imu >("imu", 1);
  pose_pub = nh.advertise< geometry_msgs::PoseStamped >("pose", 1);
  mag_pub = nh.advertise< sensor_msgs::MagneticField >("magnetic_field", 1);
  temp_pub = nh.advertise< sensor_msgs::Temperature >("temperature", 1);
  if (publish_tf) {
    tf_pub.reset(new tf::TransformBroadcaster);
  }

  // construct the worker
  boost::asio::io_service asio_service;
  bus::BNO055USBStick device(asio_service, publish);

  // run the worker
  while (nh.ok()) {
    asio_service.run_one();
  }

  pigpio_stop(pigpio_ptr);

  return 0;
}
