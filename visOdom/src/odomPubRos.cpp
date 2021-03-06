#include "odomPubRos.h"
#include "data_capture_ros.h"

sig_atomic_t shutdown_flag = 0;
static void
sig_action(int signal, siginfo_t *s, void *user)
{
  fprintf(stderr,"Shutting Down!\n");
  ros::requestShutdown();
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

void test(const sensor_msgs::ImageConstPtr& msg)
{
  printf ("arg\n");
}

int main(int argc, char **argv)
{
  std::string odomFrameName = "visOdom";
  std::string baseFrameName = "vis_base_link";
  // initialize the node and various publishers
  ros::init(argc, argv, "fovis_odom");
  ros::NodeHandle nh;
  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>(odomFrameName, 50);
  tf::TransformBroadcaster odom_broadcaster;

  // initialize the device
  fovis_ros_kinect::DataCapture* cap = new fovis_ros_kinect::DataCapture();
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber graySub = it.subscribe("/camera/rgb/image_mono",5,&fovis_ros_kinect::DataCapture::processGray,cap);
  image_transport::Subscriber depthSub = it.subscribe("/camera/depth/image",5,&fovis_ros_kinect::DataCapture::processDepth,cap);
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
  options["feature-window-size"] = "9";
  options["max-pyramid-level"] = "3";
  options["inlier-max-reprojection-error"] = "1.5";
  options["clique-inlier-threshold"] = "0.1";
  options["min-features-for-estimate"] = "10";
  options["max-mean-reprojection-error"] = "10.0";
  options["use-subpixel-refinement"] = "true";
  options["feature-search-window"] = "25";
  options["target-pixels-per-feature"] = "250";
  options["update-target-features-with-refined"] = "false";

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

  while(!(cap->capturing))
  {
    ros::spinOnce();
  }

  //We need this to calculate velocity
  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  //A check to avoid divison by zero errors
  bool pastFirstLoop = false;

  while(!shutdown_flag) {
    ros::spinOnce();
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
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = odomFrameName;
    odom_trans.child_frame_id = baseFrameName;

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = z;
    odom_trans.transform.rotation = gm_odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = odomFrameName;

    //set the position
    odom_msg.pose.pose.position.x = x;
    odom_msg.pose.pose.position.y = y;
    odom_msg.pose.pose.position.z = z;
    odom_msg.pose.pose.orientation = gm_odom_quat;

    // Now get the motion estimate for this frame to the previous frame.
    Eigen::Isometry3d motion_estimate = odom->getMotionEstimate();

    if (pastFirstLoop) {
      Eigen::Vector3d rpy = motion_estimate.rotation().eulerAngles(0, 1, 2);
      double dr = rpy(0) * 180/M_PI;
      double dp = rpy(1) * 180/M_PI;
      double dyaw = rpy(2) * 180/M_PI;

      Eigen::Vector3d vxyz = motion_estimate.translation();
      double dx = vxyz(2);
      double dy = -vxyz(0);
      double dz = -vxyz(1);

      // Now get the time differential between the two frames
      double dt = (current_time - last_time).toSec();

      double vx = dx/dt;
      double vy = dy/dt;
      double vz = dz/dt;

      double vr = dr/dt;
      double vp = dp/dt;
      double vyaw = dy/dt;

      //set the velocity
      odom_msg.child_frame_id = baseFrameName;
      odom_msg.twist.twist.linear.x = vx;
      odom_msg.twist.twist.linear.y = vy;
      odom_msg.twist.twist.linear.z = vz;
      //odom_msg.twist.twist.angular.w = vw;
      odom_msg.twist.twist.angular.x = vr;
      odom_msg.twist.twist.angular.y = vp;
      odom_msg.twist.twist.angular.z = vyaw;
    }

    //publish the message
    odom_pub.publish(odom_msg);

    //Set the time:
    last_time = current_time;
    current_time = ros::Time::now();  
    if (!pastFirstLoop)
      pastFirstLoop = true;

    // display the motion estimate.  These values are all given in the RGB
    // camera frame, where +Z is forward, +X points right, +Y points down, and
    // the origin is located at the focal point of the RGB camera.
    //std::cout << isometryToString(cam_to_local) << " " << 
    //  isometryToString(motion_estimate) << "\n";
  }

  printf("Shutting down\n");
  delete cap;
  delete odom;
  return 0;
}
