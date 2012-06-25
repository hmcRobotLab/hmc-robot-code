#include "odomPubRos.h"
#include "data_capture_ros.h"

#define dbg(...) fprintf(stderr, __VA_ARGS__)

sig_atomic_t shutdown_flag = 0;
static void
sig_action(int signal, siginfo_t *s, void *user)
{
  fprintf(stderr,"Shutting Down!\n");
  shutdown_flag = 1;
}

std::string
isometryToString(const Eigen::Isometry3d& m)
{
  char result[80];
  memset(result, 0, sizeof(result));
  Eigen::Vector3d xyz = m.translation();
  Eigen::Vector3d rpy = m.rotation().eulerAngles(0, 1, 2);
  snprintf(result, 79, "%6.2f %6.2f %6.2f %6.2f %6.2f %6.2f", 
      xyz(0), xyz(1), xyz(2), 
      rpy(0) * 180/M_PI, rpy(1) * 180/M_PI, rpy(2) * 180/M_PI);
  return std::string(result);
}

ros::NodeHandle nh;
int main(int argc, char **argv)
{
  // initialize the node and various publishers
  ros::init(argc, argv, "fovis_odom");
  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
  tf::TransformBroadcaster odom_broadcaster;

  // initialize the device
  fovis_ros_kinect::DataCapture* cap = new fovis_ros_kinect::DataCapture();
  //if(!cap->initialize()) {
  //  fprintf(stderr, "Unable to initialize OpenNI sensor\n");
  //  return 1;
  //}
  //if(!cap->startDataCapture()) {
  //  fprintf(stderr, "Unable to start data capture\n");
  //  return 1;
  //}

  // get the RGB camera parameters of our device
  fovis::Rectification rect(cap->getRgbParameters());

  fovis::VisualOdometryOptions options = 
      fovis::VisualOdometry::getDefaultOptions();
  // If we wanted to play around with the different VO parameters, we could set
  // them here in the "options" variable.

  // setup the visual odometry
  fovis::VisualOdometry* odom = new fovis::VisualOdometry(&rect, options);

  // exit cleanly on CTL-C
  struct sigaction new_action;
  new_action.sa_sigaction = sig_action;
  sigemptyset(&new_action.sa_mask);
  new_action.sa_flags = 0;
  sigaction(SIGINT, &new_action, NULL);
  sigaction(SIGTERM, &new_action, NULL);
  sigaction(SIGHUP, &new_action, NULL);

  while(!shutdown_flag) {
    if(!cap->captureOne()) {
      fprintf(stderr, "Capture failed\n");
      break;
    }

    odom->processFrame(cap->getGrayImage(), cap->getDepthImage());

    // get the integrated pose estimate.
    Eigen::Isometry3d cam_to_local = odom->getPose();
    Eigen::Quaternion<double> quat = Eigen::Quaternion<double>(cam_to_local.rotation());

    double w = quat.w();
    double qx = quat.z();
    double qy = -quat.x();
    double qz = -quat.y();

    Eigen::Vector3d xyz = cam_to_local.translation();
    double x = xyz(2);
    double y = -xyz(0);
    double z = -xyz(1);

    tf::Quaternion tf_odom_quat = tf::Quaternion(qx,qy,qz,w);
    geometry_msgs::Quaternion gm_odom_quat;
    tf::quaternionTFToMsg(tf_odom_quat, gm_odom_quat);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = ros::Time::now();;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = z;
    odom_trans.transform.rotation = gm_odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = ros::Time::now();
    odom_msg.header.frame_id = "odom";

    //set the position
    odom_msg.pose.pose.position.x = x;
    odom_msg.pose.pose.position.y = y;
    odom_msg.pose.pose.position.z = z;
    odom_msg.pose.pose.orientation = gm_odom_quat;

    //set the velocity
    odom_msg.child_frame_id = "base_link";
    //odom_msg.twist.twist.linear.x = vx;
    //odom_msg.twist.twist.linear.y = vy;
    //odom_msg.twist.twist.angular.z = vth;

    //publish the message
    odom_pub.publish(odom_msg);

       

    // get the motion estimate for this frame to the previous frame.
    Eigen::Isometry3d motion_estimate = odom->getMotionEstimate();

    // display the motion estimate.  These values are all given in the RGB
    // camera frame, where +Z is forward, +X points right, +Y points down, and
    // the origin is located at the focal point of the RGB camera.
    //std::cout << isometryToString(cam_to_local) << " " << 
    //  isometryToString(motion_estimate) << "\n";
  }

  printf("Shutting down\n");
  cap->stopDataCapture();
  delete odom;
  delete cap;
  return 0;
}
