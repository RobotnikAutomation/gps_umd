/*
 * Translates sensor_msgs/NavSat{Fix,Status} into nav_msgs/Odometry using UTM
 */

#include <ros/ros.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/NavSatFix.h>
#include <gps_common/conversions.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Trigger.h>

using namespace gps_common;

static ros::Publisher odom_pub;
std::string frame_id, child_frame_id, odom_topic, fix_topic;
double rot_cov;
double datum_northing;
double datum_easting;
double datum_altitude;
bool reset_datum = true;

/**
 * Callback function to process GPS fix messages
 * and publish corresponding odometry messages.
 */
void callback(const sensor_msgs::NavSatFixConstPtr& fix) {
  if (fix->status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX) {
    ROS_INFO("No fix.");
    return;
  }

  if (fix->header.stamp == ros::Time(0)) {
    return;
  }

  double northing, easting;
  std::string zone;

  // Convert latitude and longitude to UTM coordinates
  LLtoUTM(fix->latitude, fix->longitude, northing, easting, zone);
  
  if (reset_datum) {
    datum_easting = easting;
    datum_northing = northing;
    datum_altitude = fix->altitude;
    reset_datum = false;
  }

  if (odom_pub) {
    nav_msgs::Odometry odom;
    odom.header.stamp = fix->header.stamp;

    odom.header.frame_id = frame_id.empty() ? fix->header.frame_id : frame_id;
    odom.child_frame_id = child_frame_id;

    odom.pose.pose.position.x = easting - datum_easting;
    odom.pose.pose.position.y = northing - datum_northing;
    odom.pose.pose.position.z = fix->altitude - datum_altitude;
    
    odom.pose.pose.orientation.x = 0;
    odom.pose.pose.orientation.y = 0;
    odom.pose.pose.orientation.z = 0;
    odom.pose.pose.orientation.w = 1;
    
    // Use ENU covariance to build XYZRPY covariance
    boost::array<double, 36> covariance = {{
      fix->position_covariance[0],
      fix->position_covariance[1],
      fix->position_covariance[2],
      0, 0, 0,
      fix->position_covariance[3],
      fix->position_covariance[4],
      fix->position_covariance[5],
      0, 0, 0,
      fix->position_covariance[6],
      fix->position_covariance[7],
      fix->position_covariance[8],
      0, 0, 0,
      0, 0, 0, rot_cov, 0, 0,
      0, 0, 0, 0, rot_cov, 0,
      0, 0, 0, 0, 0, rot_cov
    }};

    odom.pose.covariance = covariance;

    odom_pub.publish(odom);
  }
}

/**
 * Service callback to reset the datum for odometry calculations.
 */
bool resetDatumCallback(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response) {
  reset_datum = true;
  ROS_INFO("Datum reset.");
  response.success = true;
  response.message = "Datum has been reset.";
  return true;
}

int main (int argc, char **argv) {
  ros::init(argc, argv, "utm_odometry_node");
  ros::NodeHandle node;
  ros::NodeHandle priv_node("~");

  priv_node.param<std::string>("frame_id", frame_id, "");
  priv_node.param<std::string>("child_frame_id", child_frame_id, "");
  priv_node.param<std::string>("odom_topic", odom_topic, "odom");
  priv_node.param<std::string>("fix_topic", fix_topic, "fix");
  priv_node.param<double>("rot_covariance", rot_cov, 99999.0);

  odom_pub = node.advertise<nav_msgs::Odometry>(odom_topic, 10);
  ros::Subscriber fix_sub = node.subscribe(fix_topic, 10, callback);
  ros::ServiceServer service = node.advertiseService("reset_datum", resetDatumCallback);

  ros::spin();
}
