#include <ros/package.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <mutex>

#include <string>
#include <mrs_msgs/BacaProtocol.h>
#include <mrs_msgs/GripperDiagnostics.h>



class MrsGripper 
{
public:
  MrsGripper();

  // Services 
  ros::ServiceServer grip_service;
  ros::ServiceServer ungrip_service;
  
  // Callbacks
  bool callbackGrip(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  bool callbackUngrip(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  // Callback for custom serial protocol
  void callbackBaca(const mrs_msgs::BacaProtocolConstPtr &msg);

  // Nodes
  ros::NodeHandle nh_;
  // Publishers
  ros::Publisher baca_protocol_publisher;
  ros::Publisher gripper_diagnostics_publisher;

  // Subscribers
  ros::Subscriber baca_protocol_subscriber;

  std::mutex mutex_msg;
};



MrsGripper::MrsGripper() 
{

  // Get paramters
  nh_ = ros::NodeHandle("~");

  ros::Time::waitForValid();

  baca_protocol_publisher       = nh_.advertise<mrs_msgs::BacaProtocol>("baca_protocol_out", 1);
  gripper_diagnostics_publisher = nh_.advertise<mrs_msgs::GripperDiagnostics>("gripper_diagnostics_out", 1);

  baca_protocol_subscriber = nh_.subscribe("baca_protocol_in", 1, &MrsGripper::callbackBaca, this, ros::TransportHints().tcpNoDelay());

  grip_service       = nh_.advertiseService("grip_in", &MrsGripper::callbackGrip, this);
  ungrip_service     = nh_.advertiseService("ungrip_in", &MrsGripper::callbackUngrip, this);
 
}


// | ------------------------ callbacks ------------------------ |

// Extracts useful information from incoming serial communication (0x43 is ID for gripper)
void MrsGripper::callbackBaca(const mrs_msgs::BacaProtocolConstPtr &msg) {

  // Msg Processing for gripper w/ only Prox Sensor
  if (msg->payload.size() == 4 && msg->payload[0] == 0x43 && msg->checksum_correct == true)
  {
      mrs_msgs::GripperDiagnostics gripper_msg;
      gripper_msg.stamp = ros::Time::now();
     
      // Setting up gripper sensor configuration 
      gripper_msg.has_hall = false;
      gripper_msg.has_proximity = true;
      gripper_msg.has_ultrasonic = false;
      
      // Processing Raw Msg
      int8_t proximity_val = msg->payload[1];
      gripper_msg.proximity1_debug = proximity_val;
      
      // Feedback and Magnet State
      bool proximity_feedback = msg->payload[2];
      gripper_msg.gripper_on = msg->payload[3];

      // Final Feedback 
      gripper_msg.gripping_object = proximity_feedback;
      gripper_diagnostics_publisher.publish(gripper_msg);
  }
  
 
  // Msg Processing for gripper Hall Sensor + Prox Sensor
  else if (msg->payload.size() == 8 && msg->payload[0] == 0x43 && msg->checksum_correct == true)
  {
      mrs_msgs::GripperDiagnostics gripper_msg;
      gripper_msg.stamp = ros::Time::now();
      //gripper_msg.gripper_on = true;  
       
      // Setting up gripper sensor configuration 
      gripper_msg.has_hall = true;
      gripper_msg.has_proximity = true;
      gripper_msg.has_ultrasonic = false;
      
      // Diagnostics Info
      int8_t hall_val = msg->payload[1];
      gripper_msg.hall1_debug = hall_val;
      
      int8_t proximity_val = msg->payload[2];
      gripper_msg.proximity1_debug = proximity_val;
      
      // Feedback & Gripper Status 
      bool hall_feedback = msg->payload[3];
      bool proximity_feedback = msg->payload[4];
      gripper_msg.gripping_object = msg->payload[5]; // Final Feedback
      gripper_msg.gripper_on = msg->payload[6];
     
      // Publish Message 
      gripper_diagnostics_publisher.publish(gripper_msg);
  }
}


// ROS Service callback to grip

bool MrsGripper::callbackGrip(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
  mrs_msgs::BacaProtocol msg_out;
  msg_out.stamp   = ros::Time::now();
  msg_out.payload = {0x40};
  baca_protocol_publisher.publish(msg_out);
  res.message = "Gripper has been activated";
  res.success = true;
  return true;
}

// ROS Service callback to ungrip

bool MrsGripper::callbackUngrip(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
  mrs_msgs::BacaProtocol msg_out;
  msg_out.stamp   = ros::Time::now();
  msg_out.payload = {0x41};
  baca_protocol_publisher.publish(msg_out);
  res.message = "Gripper has been deactivated";
  res.success = true;
  return true;
}


int main(int argc, char **argv) {

  ros::init(argc, argv, "mrs_gripper");
  MrsGripper gripper;
  while (ros::ok()) {
    ros::spin();
  }
  return 0;
}

