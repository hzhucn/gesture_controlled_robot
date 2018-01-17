// Usage : 
// rosrun koala_node camera_position_node koala3.smart.metz.supelec.fr 4101

// Publish on /perception_camera_position
// Subscribes to /cmd_pos for changing the camera position

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>

#include <kteams_driver.h>

void decode_command(const geometry_msgs::PointStampedConstPtr& camera_position_msg, kteams::koala::CameraPositionClient& client) {
  client.write(camera_position_msg->point.x,camera_position_msg->point.y);
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "koala_camera_position");
  
  if(argc != 3) {
    std::cerr << "Usage : " << argv[0] << " hostname port" << std::endl;
    return -1;
  }

  ros::NodeHandle camera_nh;


 // Open the connection to the camera position server
  kteams::koala::CameraPositionClient camera_client;
  std::string hostname = argv[1];
  std::string port = argv[2];
  ROS_INFO("Connecting to %s:%s", hostname.c_str(), port.c_str());
  camera_client.connect(hostname, port);
  ROS_INFO("Connected to %s:%s", hostname.c_str(), port.c_str());


  // Definition of the ROS publisher/subscriber
  ros::Publisher camera_position_publisher = camera_nh.advertise<geometry_msgs::PointStamped>("perception_camera_position", 1);
  ros::Subscriber camera_position_subscriber = camera_nh.subscribe<geometry_msgs::PointStamped>("cmd_pos", 1, boost::bind(decode_command, _1, boost::ref(camera_client)));

  int msg_id = 0;
  geometry_msgs::PointStamped msg;
  int pan, tilt;
  while(ros::ok()) {
    
    // Read and post the camera position
    camera_client.read(pan, tilt);

    msg.header.seq = msg_id++;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "camera_pan_tilt_link";
    msg.point.x = pan;
    msg.point.y = tilt;
    msg.point.z = 0;
    camera_position_publisher.publish(msg);

    ros::spinOnce();
  }
  camera_client.close();
}
