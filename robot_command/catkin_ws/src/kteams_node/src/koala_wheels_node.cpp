// Usage : 
// rosrun koala_node camera_node koala3.smart.metz.supelec.fr 4100

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>

#include <kteams_driver.h>
#include <stdexcept>
#include <functional>

void decode_command(const geometry_msgs::TwistConstPtr& wheel_speed_msg, const kteams::koala::WheelSpeed& client) {
  client.write(wheel_speed_msg->linear.x, wheel_speed_msg->angular.z);
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "koala_wheels");

  if(argc != 3) {
    std::cerr << "Usage : " << argv[0] << " hostname port" << std::endl;
    return -1;
  }

  ros::NodeHandle nh;

  kteams::koala::WheelServer wheel_server;

  // Definition of the koala_driver nodes
  kteams::koala::IRReader ir_reader(wheel_server);
  kteams::koala::WheelSpeed speed_client(wheel_server);


  // Definition of the ROS publishers
  ros::Publisher wheel_speed_publisher = nh.advertise<geometry_msgs::TwistStamped>("perception_wheel_speed", 1);

  ros::Publisher ir_range_publisher = nh.advertise<std_msgs::Float32MultiArray>("ir/range", 1);
  ros::Publisher ir_raw_publisher = nh.advertise<std_msgs::Int32MultiArray>("ir/raw", 1);

  // Definition of the ROS subscriber to post the new speeds
  ros::Subscriber wheel_speed_subscriber = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1, boost::bind(decode_command, _1, speed_client));

  // Open the connection to the wheel server
  std::string hostname = argv[1];
  std::string port = argv[2];
  ROS_INFO("Connecting to %s:%s", hostname.c_str(), port.c_str());
  wheel_server.connect(hostname, port);
  ROS_INFO("Connected to %s:%s", hostname.c_str(), port.c_str());



  // Definition of the koala_driver messages and ROS messages
  std::array<int, kteams::koala::IRReader::nb_ir_sensors> ir_raw;
  std::array<double, kteams::koala::IRReader::nb_ir_sensors> ir_ranges;
  double linear_speed, angular_speed;
  geometry_msgs::TwistStamped msg_wheel_speed;
  std_msgs::Float32MultiArray msg_ir;
  msg_ir.layout.dim.resize(1);
  msg_ir.layout.dim[0].label = "Sensor";
  msg_ir.layout.dim[0].size = kteams::koala::IRReader::nb_ir_sensors;
  msg_ir.layout.dim[0].stride = kteams::koala::IRReader::nb_ir_sensors;
  msg_ir.data.resize(kteams::koala::IRReader::nb_ir_sensors);

  std_msgs::Int32MultiArray msg_ir_raw;
  msg_ir_raw.layout = msg_ir.layout;
  msg_ir_raw.data.resize(kteams::koala::IRReader::nb_ir_sensors);

  int msg_id = 0;

  while (ros::ok()) {

    // Read and post the IR data
    ir_reader.read(ir_ranges, ir_raw);
    std::copy(ir_ranges.begin(), ir_ranges.end(), msg_ir.data.begin());
    ir_range_publisher.publish(msg_ir);

    std::copy(ir_raw.begin(), ir_raw.end(), msg_ir_raw.data.begin());
    ir_raw_publisher.publish(msg_ir_raw);

    // Read and post the wheel speed
    speed_client.read(linear_speed, angular_speed);
    msg_wheel_speed.twist.linear.x = linear_speed;
    msg_wheel_speed.twist.linear.y = 0.0;
    msg_wheel_speed.twist.linear.z = 0.0;
    msg_wheel_speed.twist.angular.x = 0.0;
    msg_wheel_speed.twist.angular.y = 0.0;
    msg_wheel_speed.twist.angular.z = angular_speed;
    msg_wheel_speed.header.seq = msg_id;
    msg_wheel_speed.header.stamp = ros::Time::now();
    msg_wheel_speed.header.frame_id = "base_link";
    wheel_speed_publisher.publish(msg_wheel_speed);
    
    ++msg_id;
    ros::spinOnce();

  }
  // Close the connection to the wheel server
  wheel_server.close();

}
