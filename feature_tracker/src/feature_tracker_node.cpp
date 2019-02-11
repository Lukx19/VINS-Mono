#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/image_encodings.h>
#include <signal.h>
#include <std_msgs/Bool.h>
#include "feature_tracker.h"

#define SHOW_UNDISTORTION 0

vector<uchar> r_status;
vector<float> r_err;
queue<sensor_msgs::ImageConstPtr> img_buf;

ros::Publisher pub_img, pub_match;
ros::Publisher pub_restart;

std::vector<FeatureTracker> trackers;
double first_image_time;
int pub_count = 1;
bool first_image_flag = true;
double last_image_time = 0;
bool init_pub = 0;

void init_callback(const std_msgs::BoolConstPtr& init_msg) {
  ROS_INFO_STREAM("IS INITIALIZING " << init_msg->data);
  for (auto&& tracker : trackers) {
    if (init_msg->data) {
      tracker.useInitTracking();
    } else {
      tracker.useStandardTracking();
    }
  }
}

cv_bridge::CvImageConstPtr msgToCvMono8(const sensor_msgs::ImageConstPtr& img_msg)
{
  cv_bridge::CvImageConstPtr ptr;
  if (img_msg->encoding == "8UC1") {
    sensor_msgs::Image img;
    img.header = img_msg->header;
    img.height = img_msg->height;
    img.width = img_msg->width;
    img.is_bigendian = img_msg->is_bigendian;
    img.step = img_msg->step;
    img.data = img_msg->data;
    img.encoding = "mono8";
    ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
  } else{
    ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
  }
  return ptr;
}

void publish_results(const std_msgs::Header &header,const cv::Mat & depth_uv = cv::Mat()) {
  pub_count++;
  sensor_msgs::PointCloudPtr feature_points(new sensor_msgs::PointCloud);
  sensor_msgs::ChannelFloat32 id_of_point;
  sensor_msgs::ChannelFloat32 u_of_point;
  sensor_msgs::ChannelFloat32 v_of_point;
  sensor_msgs::ChannelFloat32 velocity_x_of_point;
  sensor_msgs::ChannelFloat32 velocity_y_of_point;

  feature_points->header = header;
  feature_points->header.frame_id = "world";

  bool available_depth = !depth_uv.empty();

  vector<set<int>> hash_ids(NUM_OF_CAM);
  for (int i = 0; i < NUM_OF_CAM; i++) {
    auto& un_pts = trackers[i].cur_un_pts;
    auto& cur_pts = trackers[i].cur_pts;
    auto& ids = trackers[i].ids;
    auto& pts_velocity = trackers[i].pts_velocity;
    for (unsigned int j = 0; j < ids.size(); j++) {
      if (trackers[i].track_cnt[j] > 1) {
        int p_id = ids[j];
        hash_ids[i].insert(p_id);
        geometry_msgs::Point32 p;
        p.x = un_pts[j].x;
        p.y = un_pts[j].y;
        if(available_depth){
          double depth = depth_uv.at<float>(cur_pts[j].y, cur_pts[j].x);
          // std::cout << depth<<"  ";
          if (depth > 0) {
            p.z = depth;
          }else{
            p.z = -1;
          }
        }else{
          p.z = -1;
        }

        feature_points->points.push_back(p);
        id_of_point.values.push_back(p_id * NUM_OF_CAM + i);
        u_of_point.values.push_back(cur_pts[j].x);
        v_of_point.values.push_back(cur_pts[j].y);
        velocity_x_of_point.values.push_back(pts_velocity[j].x);
        velocity_y_of_point.values.push_back(pts_velocity[j].y);
      }
    }
  }
  feature_points->channels.push_back(id_of_point);
  feature_points->channels.push_back(u_of_point);
  feature_points->channels.push_back(v_of_point);
  feature_points->channels.push_back(velocity_x_of_point);
  feature_points->channels.push_back(velocity_y_of_point);
  ROS_DEBUG("publish %f, at %f", feature_points->header.stamp.toSec(), ros::Time::now().toSec());
  // skip the first image; since no optical speed on frist image
  if (!init_pub) {
    init_pub = 1;
  } else
    pub_img.publish(feature_points);
}

void publish_visualizations(const sensor_msgs::ImageConstPtr& img_msg){
  cv_bridge::CvImageConstPtr ptr = msgToCvMono8(img_msg);
  cv::Mat show_img = ptr->image;
  ptr = cv_bridge::cvtColor(ptr, sensor_msgs::image_encodings::BGR8);
  // cv::Mat stereo_img(ROW * NUM_OF_CAM, COL, CV_8UC3);
  cv::Mat stereo_img = ptr->image;

  for (int i = 0; i < NUM_OF_CAM; i++) {
    cv::Mat tmp_img = stereo_img.rowRange(i * ROW, (i + 1) * ROW);
    // cv::Mat detected_edges,masked_img;
    // masked_img.create( show_img.size(), show_img.type() );
    // int ratio = 3;
    // int kernel_size = 3;
    // int lowThreshold = 50;
    // cv::blur( show_img, detected_edges, cv::Size(3,3) );
    // cv::Canny( detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size );
    // masked_img = cv::Scalar::all(0);
    // show_img.copyTo(masked_img, detected_edges);
    // cv::cvtColor(masked_img, tmp_img, CV_GRAY2RGB);
    cv::cvtColor(show_img, tmp_img, CV_GRAY2RGB);

    for (unsigned int j = 0; j < trackers[i].cur_pts.size(); j++) {
      double len = std::min(1.0, 1.0 * trackers[i].track_cnt[j] / WINDOW_SIZE);
      cv::circle(tmp_img, trackers[i].cur_pts[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
      // draw speed line
      // Vector2d tmp_cur_un_pts (trackers[i].cur_un_pts[j].x, trackers[i].cur_un_pts[j].y);
      // Vector2d tmp_pts_velocity (trackers[i].pts_velocity[j].x,
      // trackers[i].pts_velocity[j].y); Vector3d tmp_prev_un_pts; tmp_prev_un_pts.head(2) =
      // tmp_cur_un_pts - 0.10 * tmp_pts_velocity; tmp_prev_un_pts.z() = 1; Vector2d tmp_prev_uv;
      // trackers[i].m_camera->spaceToPlane(tmp_prev_un_pts, tmp_prev_uv);
      // cv::line(tmp_img, trackers[i].cur_pts[j], cv::Point2f(tmp_prev_uv.x(),
      // tmp_prev_uv.y()), cv::Scalar(255 , 0, 0), 1 , 8, 0);

      // char name[10];
      // sprintf(name, "%d", trackers[i].ids[j]);
      // cv::putText(tmp_img, name, trackers[i].cur_pts[j], cv::FONT_HERSHEY_SIMPLEX, 0.5,
      // cv::Scalar(0, 0, 0));
    }
    }
    // cv::imshow("vis", stereo_img);
    // cv::waitKey(5);
    pub_match.publish(ptr->toImageMsg());
}
// return true if image should be published
bool process_img(const sensor_msgs::ImageConstPtr& img_msg) {
   if (first_image_flag) {
    first_image_flag = false;
    first_image_time = img_msg->header.stamp.toSec();
    last_image_time = img_msg->header.stamp.toSec();
    return false;
  }
  // detect unstable camera stream
  if (img_msg->header.stamp.toSec() - last_image_time > 1.0 ||
      img_msg->header.stamp.toSec() < last_image_time) {
    ROS_WARN("image discontinue! reset the feature tracker!");
    first_image_flag = true;
    last_image_time = 0;
    pub_count = 1;
    std_msgs::Bool restart_flag;
    restart_flag.data = true;
    pub_restart.publish(restart_flag);
    return false;
  }
  last_image_time = img_msg->header.stamp.toSec();

  // frequency control
  bool pub_this_frame = false;
  if (round(1.0 * pub_count / (img_msg->header.stamp.toSec() - first_image_time)) <= FREQ) {
    pub_this_frame = true;
    // reset the frequency control
    if (abs(1.0 * pub_count / (img_msg->header.stamp.toSec() - first_image_time) - FREQ) <
        0.01 * FREQ) {
      first_image_time = img_msg->header.stamp.toSec();
      pub_count = 0;
    }
  }

  cv_bridge::CvImageConstPtr ptr = msgToCvMono8(img_msg);


  TicToc t_r;
  for (int i = 0; i < NUM_OF_CAM; i++) {
    ROS_DEBUG("processing camera %d", i);
    if (i != 1 || !STEREO_TRACK) {
      trackers[i].readImage(ptr->image.rowRange(ROW * i, ROW * (i + 1)),
                            img_msg->header.stamp.toSec(), pub_this_frame);

    } else {
      if (EQUALIZE) {
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
        clahe->apply(ptr->image.rowRange(ROW * i, ROW * (i + 1)), trackers[i].cur_img);
      } else
        trackers[i].cur_img = ptr->image.rowRange(ROW * i, ROW * (i + 1));
    }

#if SHOW_UNDISTORTION
    trackers[i].showUndistortion("undistrotion_" + std::to_string(i));
#endif
  }

  for (unsigned int i = 0;; i++) {
    bool completed = false;
    for (int j = 0; j < NUM_OF_CAM; j++)
      if (j != 1 || !STEREO_TRACK)
        completed |= trackers[j].updateID(i);
    if (!completed)
      break;
  }
  ROS_DEBUG("whole feature tracker processing costs: %f", t_r.toc());
  return pub_this_frame;
}

void img_callback(const sensor_msgs::ImageConstPtr& img_msg) {
  bool pub_this_frame = process_img(img_msg);
  if (pub_this_frame) {
    publish_results(img_msg->header);
    publish_visualizations(img_msg);
  }

}

void depth_img_callback(const sensor_msgs::ImageConstPtr& image,
                        const sensor_msgs::ImageConstPtr& depth_image) {
  // ROS_INFO("depth image arrived");
  bool pub_this_frame = process_img(image);
  if (pub_this_frame) {
    cv_bridge::CvImageConstPtr depth_img_cv;
    cv::Mat depth_mat;
    depth_img_cv = cv_bridge::toCvShare(depth_image,depth_image->encoding);
    // Convert the uints to floats
    depth_img_cv->image.convertTo(depth_mat, CV_32F, 0.001);
    // std::cout << depth_mat << std::endl;
    publish_results(image->header, depth_mat);
    publish_visualizations(image);
  }
}

void termHandler(int sig) {
  for (auto&& tr : trackers) {
    tr.printStatistics();
  }
  ros::Duration(5).sleep();
  ros::shutdown();
}

int main(int argc, char** argv) {
  signal(SIGTERM, termHandler);
  signal(SIGINT, termHandler);
  signal(SIGKILL, termHandler);

  ros::init(argc, argv, "feature_tracker");
  ros::NodeHandle n("~");
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
  readParameters(n);
  trackers.resize(NUM_OF_CAM);
  for (int i = 0; i < NUM_OF_CAM; i++)
    trackers[i].readIntrinsicParameter(CAM_NAMES[i]);

  if (FISHEYE) {
    for (int i = 0; i < NUM_OF_CAM; i++) {
      trackers[i].fisheye_mask = cv::imread(FISHEYE_MASK, 0);
      if (!trackers[i].fisheye_mask.data) {
        ROS_ERROR("load mask fail");
        ROS_BREAK();
      } else
        ROS_INFO("load mask success");
    }
  }
  ros::Subscriber sub_img;
  message_filters::Subscriber<sensor_msgs::Image> image_sub(n, IMAGE_TOPIC, 1);
  message_filters::Subscriber<sensor_msgs::Image> image_depth_sub(n, DEPTH_IMAGE_TOPIC, 1);
  using MySyncPolicy=message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image>;
  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(3), image_sub, image_depth_sub);

  // message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync(image_sub, image_depth_sub, 100);
  if (RGBD_CAM) {
    sync.registerCallback(boost::bind(&depth_img_callback, _1, _2));
  } else {
    sub_img = n.subscribe(IMAGE_TOPIC, 100, img_callback);
  }
  ros::Subscriber sub_initialize = n.subscribe("init", 100, init_callback);
  pub_img = n.advertise<sensor_msgs::PointCloud>("feature", 1000);
  pub_match = n.advertise<sensor_msgs::Image>("feature_img", 1000);
  pub_restart = n.advertise<std_msgs::Bool>("restart", 1000);
  /*
  if (SHOW_TRACK)
      cv::namedWindow("vis", cv::WINDOW_NORMAL);
  */
  ros::spin();
  termHandler(1);
  return 0;
}

// new points velocity is 0, pub or not?
// track cnt > 1 pub?y is 0, pub or not?
// track cnt > 1 pub?