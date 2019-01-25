#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <signal.h>
#include <std_msgs/Header.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <fstream>
#include <vector>

template <typename T>
T readParam(ros::NodeHandle &n, const std::string & name) {
  T ans;
  if (n.getParam(name, ans)) {
    ROS_INFO_STREAM("Loaded " << name << ": " << ans);
  } else {
    ROS_ERROR_STREAM("Failed to load " << name);
    n.shutdown();
  }
  return ans;
}


struct DataRec {
  double timestamp;
  Eigen::Vector3d trans;
  Eigen::Quaterniond rot;
};

std::vector<DataRec> record_buffer_;
int idx = 1;
std::string rec_file;
ros::Time last_timestamp;
bool first_msg_recorded = false;

void saveRecords(const std::vector<DataRec> &data) {
  std::ofstream fout(rec_file, std::ios::app);
  if (fout.is_open()) {
    fout.setf(std::ios::fixed, std::ios::floatfield);
    for (const DataRec &rec : data) {
      fout.precision(0);
      fout << rec.timestamp * 1e9 << ",";
      fout.precision(5);
      fout << rec.trans.x() << "," << rec.trans.y() << "," << rec.trans.z()
           << "," << rec.rot.w() << "," << rec.rot.x() << "," << rec.rot.y()
           << "," << rec.rot.z() << std::endl;
    }
    fout.flush();
    fout.close();
  } else {
    ROS_ERROR_STREAM("Cannot open file to save records " << rec_file);
  }
}

void termHandler(int sig) {
  saveRecords(record_buffer_);
  ros::shutdown();
}

void saveCurrentPose(const std_msgs::Header &header,
                     const geometry_msgs::Pose &pose) {
  first_msg_recorded = true;
  last_timestamp = ros::Time::now();
  if (record_buffer_.size() > 50) {
    saveRecords(record_buffer_);
    record_buffer_.clear();
  }
  DataRec dt;
  dt.rot = Eigen::Quaterniond(pose.orientation.w, pose.orientation.x,
                              pose.orientation.y, pose.orientation.z);
  dt.trans =
      Eigen::Vector3d{pose.position.x, pose.position.y, pose.position.z};
  dt.timestamp = header.stamp.toSec();
  record_buffer_.push_back(std::move(dt));
}

void pose_callback(const geometry_msgs::PoseStampedConstPtr &pose_msg) {
    ROS_DEBUG("odom callback!");
    saveCurrentPose(pose_msg->header, pose_msg->pose);
}

void odom_callback(const nav_msgs::OdometryConstPtr &odom_msg) {
  ROS_DEBUG("odom callback!");
  saveCurrentPose(odom_msg->header, odom_msg->pose.pose);
}

int main(int argc, char **argv) {
  record_buffer_.reserve(100);
  ros::init(argc, argv, "benchmark_recorder");
  ros::NodeHandle n("~");
  signal(SIGTERM, termHandler);
  signal(SIGINT, termHandler);
  signal(SIGKILL, termHandler);

  rec_file = readParam<std::string>(n, "rec_file");

  ros::Subscriber sub_odom =
      n.subscribe("estimated_odometry", 1000, odom_callback);

  ros::Subscriber sub_pose = n.subscribe("estimated_pose", 1000, pose_callback);

  ros::Rate loop_rate(20);
  while (ros::ok()) {
    // no incomming message for more than 10 seconds -> terminate
    if (first_msg_recorded && (ros::Time::now() - last_timestamp).toSec() > 20) {
      break;
    }
    ros::spinOnce();

    loop_rate.sleep();
  }
   saveRecords(record_buffer_);
   return 0;
}
