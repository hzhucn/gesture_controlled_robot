// Usage : 
// rosrun koala_node camera_node koala3.smart.metz.supelec.fr 4102
// rosrun rqt_image_view rqt_image_view

// Publish on image_raw   RGB8

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <camera_info_manager/camera_info_manager.h>

#include <kteams_driver.h>
#include <stdexcept>

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "koala_camera");

  if(argc != 3) {
    std::cerr << "Usage : " << argv[0] << " hostname port" << std::endl;
    return -1;
  }
  
  ros::NodeHandle camera_nh("camera");
  image_transport::ImageTransport it(camera_nh);
  image_transport::Publisher pub = it.advertise("image_raw", 1);

  // Load the camera info data
  boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_;
  cinfo_.reset(new camera_info_manager::CameraInfoManager(camera_nh));
  std::string camera_info_url("package://koala_node/share/camera_info.yml"); 
  if (cinfo_->validateURL(camera_info_url)) {
    cinfo_->loadCameraInfo(camera_info_url);
  }

  std::string hostname(argv[1]);
  std::string port(argv[2]);
  ROS_INFO("Connecting to %s:%s", hostname.c_str(), port.c_str());

  // Open the connection to the Koala video server
  kteams::koala::ConnectionParams params;
  kteams::koala::open_connection(hostname, port, params);

  ROS_INFO("Connected to %s:%s", hostname.c_str(), port.c_str());


  sensor_msgs::Image msg;
  msg.header.frame_id = "camera_pan_tilt_link";
  msg.encoding = sensor_msgs::image_encodings::RGB8;

  // Loop for streaming the video
  ros::Rate loop_rate(30);
  unsigned int seq = 0;
  while (ros::ok()) {
    
    try {
      kteams::koala::read_image(params);

      msg.header.seq = seq++;
      msg.header.stamp = ros::Time::now();
      msg.height = params.height;
      msg.width =params. width;
      msg.step = 3 * params.width;
      msg.data.resize(3*params.width*params.height);
      memcpy(&msg.data[0],  params.decompressed_image, 3*params.width*params.height);

      pub.publish(msg);
    }
    catch(std::logic_error& e) {
      ROS_WARN("camera_node exception : %s", e.what());
    }


    ros::spinOnce();
    loop_rate.sleep();
  }
  
  // Close the connection to the Koala video server
  kteams::koala::close_connection(params);
}
