#include "feature_tracker.h"
#include "timer.h"

int FeatureTracker::n_id = 0;

bool inBorder(const cv::Point2f& pt) {
  const int BORDER_SIZE = 1;
  int img_x = cvRound(pt.x);
  int img_y = cvRound(pt.y);
  return BORDER_SIZE <= img_x && img_x < COL - BORDER_SIZE && BORDER_SIZE <= img_y &&
         img_y < ROW - BORDER_SIZE;
}

void reduceVector(vector<cv::Point2f>& v, vector<uchar> status) {
  int j = 0;
  for (int i = 0; i < int(v.size()); i++) {
    if (status[i]) {
      v[j++] = v[i];
    }
  }
  v.resize(j);
}

void reduceVector(vector<int>& v, vector<uchar> status) {
  int j = 0;
  for (int i = 0; i < int(v.size()); i++)
    if (status[i])
      v[j++] = v[i];
  v.resize(j);
}

// auto distanceReduction(const std::vector<cv::KeyPoint>& features, size_t circle_radius,
//                        size_t max_features) -> std::vector<cv::KeyPoint> {
//   std::vector<cv::KeyPoint> res;
//   if (features.size() == 0)
//     return res;

//   std::vector<cv::KeyPoint> working = features;
//   std::vector<cv::KeyPoint> working_temp;
//   working_temp.resize(features.size());
//   std::sort(working.begin(), working.end(), [](const cv::KeyPoint& a, const cv::KeyPoint& b) {
//     return (a.response > b.response) ? true : false;
//   });
//   cv::Point2f chosen_p;
//   float sq_radius = circle_radius * circle_radius;

//   while (working.size() > 0) {
//     res.push_back(working[0]);
//     if(res.size() >= max_features){
//       break;
//     }
//     chosen_p = res.back().pt;
//     for (auto&& kp : working) {
//       float x_diff = (kp.pt.x - chosen_p.x) * (kp.pt.x - chosen_p.x);
//       float y_diff = (kp.pt.y - chosen_p.y) * (kp.pt.y - chosen_p.y);
//       if (x_diff + y_diff > sq_radius) {
//         working_temp.push_back(std::move(kp));
//       }
//     }
//     working = std::move(working_temp);
//     working_temp.clear();
//   }
//   return res;
// }

auto distanceReductionGrid(const std::vector<cv::KeyPoint>& corners, int img_height, int img_width,
                           size_t min_dist, size_t max_corners) -> std::vector<cv::KeyPoint> {
  // Partition the image into larger grids
  int w = img_width, h = img_height;

  const size_t cell_size = min_dist;
  const size_t grid_width = (w + cell_size - 1) / cell_size;
  const size_t grid_height = (h + cell_size - 1) / cell_size;
  size_t ncorners = 0;
  std::vector<cv::KeyPoint> res;
  if (corners.size() == 0) {
    return res;
  }
  std::vector<cv::KeyPoint> corners_sorted = corners;
  std::sort(corners_sorted.begin(), corners_sorted.end(),
            [](const cv::KeyPoint& a, const cv::KeyPoint& b) {
              return (a.response > b.response) ? true : false;
            });

  std::vector<std::vector<cv::KeyPoint>> grid(grid_width * grid_height);
  min_dist *= min_dist;

  for (const cv::KeyPoint& c : corners_sorted) {
    bool good = true;

    size_t x_cell = c.pt.x / cell_size;
    size_t y_cell = c.pt.y / cell_size;

    size_t x1 = x_cell - 1;
    size_t y1 = y_cell - 1;
    size_t x2 = x_cell + 1;
    size_t y2 = y_cell + 1;

    // boundary check
    x1 = std::max(std::numeric_limits<size_t>::min(), x1);
    y1 = std::max(std::numeric_limits<size_t>::min(), y1);
    x2 = std::min(grid_width - 1, x2);
    y2 = std::min(grid_height - 1, y2);

    for (size_t yy = y1; yy <= y2; yy++) {
      for (size_t xx = x1; xx <= x2; xx++) {
        std::vector<cv::KeyPoint>& m = grid[yy * grid_width + xx];

        if (m.size()) {
          for (size_t j = 0; j < m.size(); j++) {
            float dx = c.pt.x - m[j].pt.x;
            float dy = c.pt.y - m[j].pt.y;

            if (dx * dx + dy * dy < min_dist) {
              good = false;
              break;
            }
          }
        }
      }
      if (!good) {
        break;
      }
    }
    if (good) {
      grid[y_cell * grid_width + x_cell].push_back(c);

      res.push_back(c);
      ++ncorners;

      if (max_corners > 0 && ncorners == max_corners)
        break;
    }
  }
  return res;
}

FeatureTracker::FeatureTracker() {
  region_pts.resize(BLOCK_PER_ROW * BLOCK_PER_COL, 0);
  std::cout << "region_pts: " << region_pts.size() << std::endl;
  regional_kp_.resize(BLOCK_PER_ROW * BLOCK_PER_COL, {});
  region_threshold_.resize(BLOCK_PER_ROW * BLOCK_PER_COL, FAST_THRESHOLD);
  if(USE_INIT_TRACKING){
    useInitTracking();
  }else{
    useStandardTracking();
  }
}

void FeatureTracker::useInitTracking() {
  max_num_features_ = static_cast<size_t>(INIT_MAX_CNT);
  init_tracking_ = true;
  feature_type_ = "GoodFeatures";
  max_num_features_region_ = (max_num_features_ / (BLOCK_PER_ROW * BLOCK_PER_COL));
}

void FeatureTracker::useStandardTracking() {
  max_num_features_ = static_cast<size_t>(MAX_CNT);
  init_tracking_ = false;
  max_num_features_region_ = MAX_CNT_PER_BLOCK;
  feature_type_ = ALG;
}
void FeatureTracker::printStatistics() const {
  std::cout << "\n" + Timer::toString() << std::endl;
  std::cout << "Feature_length: " << feature_length_.toString() << std::endl;
}

void FeatureTracker::setMask() {
  if (FISHEYE)
    mask = fisheye_mask.clone();
  else
    mask = cv::Mat(ROW, COL, CV_8UC1, cv::Scalar(255));

  // prefer to keep features that are tracked for long time
  using TrackPair = std::pair<int, std::tuple<cv::Point2f, int, int>>;
  vector<TrackPair> cnt_pts_id;

  for (unsigned int i = 0; i < forw_pts.size(); i++) {
    cnt_pts_id.push_back(
        std::make_pair(track_cnt[i], std::make_tuple(forw_pts[i], ids[i], region_ids[i])));
  }

  sort(cnt_pts_id.begin(), cnt_pts_id.end(),
       [](const TrackPair& a, const TrackPair& b) { return a.first > b.first; });

  forw_pts.clear();
  ids.clear();
  track_cnt.clear();
  region_ids.clear();
  for (auto&& pt : region_pts) {
    pt = 0;
  }
  for (auto& it : cnt_pts_id) {
    cv::Point2f point;
    int id;
    int region;
    std::tie(point, id, region) = it.second;
    if (mask.at<uchar>(point) == 255) {
      forw_pts.push_back(point);
      ids.push_back(id);
      region_ids.push_back(region);
      region_pts[region] += 1;
      track_cnt.push_back(it.first);
      cv::circle(mask, point, MIN_DIST, 0, -1);
    }
  }
}

void FeatureTracker::addPoints(const std::vector<cv::Point2f>& points, int region_id) {
  for (const cv::Point2f& p : points) {
    forw_pts.push_back(p);
    ids.push_back(-1);
    track_cnt.push_back(1);
    region_ids.push_back(region_id);
  }
  region_pts.at(region_id) += points.size();
}

void FeatureTracker::addPoints(const std::vector<cv::KeyPoint>& points, int region_id) {
  std::vector<cv::Point2f> converted;
  cv::KeyPoint::convert(points, converted);
  addPoints(converted, region_id);
}

void FeatureTracker::addPoints(const std::vector<CvPoint>& points, int region_id) {
  std::vector<cv::Point2f> converted;
  converted.reserve(points.size());
  for (auto&& pt : points) {
    converted.push_back(pt);
  }
  addPoints(converted, region_id);
}

void FeatureTracker::reducePoints(const std::vector<uchar>& mask) {
  for (size_t i = 0; i < mask.size(); ++i) {
    if (mask.at(i) == 0) {
      region_pts[region_ids[i]] -= 1;
      feature_length_.addVal(track_cnt[i]);
    }
  }

  reduceVector(region_ids, mask);
  reduceVector(prev_pts, mask);
  reduceVector(cur_pts, mask);
  reduceVector(forw_pts, mask);
  reduceVector(cur_un_pts, mask);
  reduceVector(ids, mask);
  reduceVector(track_cnt, mask);
}

void FeatureTracker::prepareForPublish() {
  if(ENABLE_F_REJECTION){
    rejectWithF();
  }
  TicToc t_m;
  setMask();
  if (mask.empty())
    cout << "mask is empty " << endl;
  if (mask.type() != CV_8UC1)
    cout << "mask type wrong " << endl;
  if (mask.size() != forw_img.size())
    cout << "wrong size " << endl;
  ROS_DEBUG("set mask costs %fms", t_m.toc());

  std::vector<cv::KeyPoint> feature_pts;
  int n_max_cnt = max_num_features_ - static_cast<int>(forw_pts.size());
  if (n_max_cnt <= 0) {
    return;
  }
  cv::Mat detected_edges;
  if (EDGE_PREFERENCE) {
    Timer::start("canny");
    int ratio = 3;
    int kernel_size = 3;
    int lowThreshold = 50;
    cv::blur(forw_img, detected_edges, cv::Size(3, 3));
    cv::Canny(detected_edges, detected_edges, lowThreshold, lowThreshold * ratio, kernel_size);
    Timer::stop("canny");
  }

  ROS_DEBUG("detect feature begins");
  TicToc t_t;
  if (feature_type_ == "GoodFeatures") {
    Timer::start("good_features");
    cv::Ptr<cv::GFTTDetector> gf = cv::GFTTDetector::create(n_max_cnt, 0.01, MIN_DIST);
    gf->detect(forw_img, feature_pts, mask);
    // cv::goodFeaturesToTrack(forw_img, n_pts, n_max_cnt, 0.01, MIN_DIST, mask);
    addPoints(feature_pts);
    Timer::stop("good_features");
  } else if (feature_type_ == "FAST") {
    Timer::start("fast_features");
    cv::Ptr<cv::FastFeatureDetector> ff = cv::FastFeatureDetector::create(FAST_THRESHOLD, false);
    ff->detect(forw_img, feature_pts, mask);
    Timer::start("distance_reduction");
    feature_pts =
        distanceReductionGrid(feature_pts, forw_img.rows, forw_img.cols, MIN_DIST, n_max_cnt);
    Timer::stop("distance_reduction");

    // Timer::start("distance_reduction2");
    // feature_pts = distanceReduction(feature_pts, MIN_DIST, n_max_cnt);
    // Timer::stop("distance_reduction2");

    addPoints(feature_pts);
    Timer::stop("fast_features");
  } else if (feature_type_ == "AGAST") {
    Timer::start("AGAST_features");
    cv::Ptr<cv::AgastFeatureDetector> agf = cv::AgastFeatureDetector::create(
        FAST_THRESHOLD, true, cv::AgastFeatureDetector::AGAST_7_12d);
    agf->detect(forw_img, feature_pts, mask);
    if (EDGE_PREFERENCE) {
      for (cv::KeyPoint& kp : feature_pts) {
        if (detected_edges.at<uchar>(kp.pt.y, kp.pt.x) > 200) {
          kp.response *= 2;
        }
      }
    }
    feature_pts =
        distanceReductionGrid(feature_pts, forw_img.rows, forw_img.cols, MIN_DIST, n_max_cnt);
    // cv::KeyPointsFilter::retainBest(feature_pts, n_max_cnt);
    addPoints(feature_pts);
    Timer::stop("AGAST_features");
  } else if (feature_type_ == "FASTBlock" || feature_type_ == "GoodBlock" || feature_type_=="AGASTBlock") {
    int window_rows = forw_img.rows / BLOCK_PER_ROW;
    int window_cols = forw_img.cols / BLOCK_PER_COL;
    size_t region_id = 0;
    std::vector<cv::Point2f> good_pts;
    std::vector<CvPoint> cvp_pts;
    // ROS_INFO_STREAM("before: "<< printVector(region_pts));
    // ROS_INFO_STREAM("thresholds: " << printVector(region_threshold_));
    for (int row = 0; row <= forw_img.rows - window_rows; row += window_rows) {
      for (int col = 0; col <= forw_img.cols - window_cols; col += window_cols) {
        if (region_pts.at(region_id) < max_num_features_region_) {
          int missing_features = max_num_features_region_ - region_pts.at(region_id);
          // not need. It is here just in case something went weird.
          if (missing_features <= 0) {
            missing_features = 0;
            ROS_WARN("Missing features in feature detector is less than 0");
          }
          if (missing_features > max_num_features_region_) {
            missing_features = max_num_features_region_;
            ROS_WARN("Missing features in feature detector is greater than maximum allowed %i",
                     MAX_CNT_PER_BLOCK);
          }
          cv::Rect window(col, row, window_cols, window_rows);
          if (feature_type_ == "FASTBlock" || feature_type_ == "AGASTBlock") {
            if (feature_type_ == "AGASTBlock") {
              cv::Ptr<cv::AgastFeatureDetector> agf = cv::AgastFeatureDetector::create(
                  region_threshold_[region_id], true, cv::AgastFeatureDetector::AGAST_7_12d);
              agf->detect(forw_img(window), feature_pts, mask(window));
            } else {
              cv::Ptr<cv::FastFeatureDetector> ff =
                  cv::FastFeatureDetector::create(region_threshold_[region_id], true);
              ff->detect(forw_img(window), feature_pts, mask(window));
            }
            if (ADAPTIVE_THRESHOLD) {
              int threshold_change = static_cast<int>(std::ceil(FAST_THRESHOLD / 2));
              if (feature_pts.size() > missing_features) {
                region_threshold_[region_id] += threshold_change;
              } else {
                if (region_threshold_[region_id] > FAST_THRESHOLD) {
                  region_threshold_[region_id] -= threshold_change;
                }
              }
            }
            if (EDGE_PREFERENCE) {
              for (cv::KeyPoint& kp : feature_pts) {
                if (detected_edges.at<uchar>(kp.pt.y+row, kp.pt.x+col) > 200) {
                  kp.response *= 2;
                }
              }
            }
            feature_pts = distanceReductionGrid(feature_pts, window_rows, window_cols, MIN_DIST,
                                                missing_features);

            for (cv::KeyPoint& pt : feature_pts) {
              pt.pt.x += col;
              pt.pt.y += row;
            }
            addPoints(feature_pts, region_id);
            feature_pts.clear();
          } else {
            cv::goodFeaturesToTrack(forw_img(window), good_pts, missing_features, 0.01, MIN_DIST);
            for (cv::Point2f& pt : good_pts) {
              pt.x += col;
              pt.y += row;
            }
            addPoints(good_pts, region_id);
            good_pts.clear();
          }
        }
        ++region_id;
      }
    }
  }
  // ROS_INFO_STREAM("after: " << printVector(region_pts));
  // ROS_DEBUG("detect feature costs: %fms", t_t.toc());
  // ROS_INFO_STREAM("\n" + Timer::toString());
}

void FeatureTracker::readImage(const cv::Mat& _img, double _cur_time, bool pub_this_frame) {
  Timer::start("readImage");
  cv::Mat img;
  TicToc t_r;
  cur_time = _cur_time;

  if (EQUALIZE) {
    Timer::start("equalize");
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
    TicToc t_c;
    clahe->apply(_img, img);
    Timer::stop("equalize");
    ROS_DEBUG("CLAHE costs: %fms", t_c.toc());
  } else {
    img = _img;
  }

  if (forw_img.empty()) {
    prev_img = cur_img = forw_img = img;
  } else {
    forw_img = img;
  }

  forw_pts.clear();

  if (cur_pts.size() > 0) {
    TicToc t_o;
    vector<uchar> status;
    vector<float> err;
    Timer::start("Flow");
    // forw_pts is just copy of original cur_pts
    cv::calcOpticalFlowPyrLK(cur_img, forw_img, cur_pts, forw_pts, status, err, cv::Size(21, 21),
                             3);
    for (int i = 0; i < int(forw_pts.size()); i++) {
      if (status[i] && !inBorder(forw_pts[i])) {
        status[i] = 0;
      }
    }
    reducePoints(status);
    Timer::stop("Flow");
    ROS_DEBUG("temporal optical flow costs: %fms", t_o.toc());
  }
  // std::cout << "------after track---------------" << forw_pts.size() << "  " << cur_pts.size()
  //           << std::endl;
  // increase trucking counter -> features with higher count are prefered
  for (auto& n : track_cnt)
    n++;

  if (pub_this_frame) {
    Timer::start("prepareForPublish");
    prepareForPublish();
    Timer::stop("prepareForPublish");
  }
  // std::cout << "------after add new features---------------" << forw_pts.size() << "  "
  // << cur_pts.size() << std::endl;
  prev_img = cur_img;
  prev_pts = cur_pts;
  prev_un_pts = cur_un_pts;
  cur_img = forw_img;
  cur_pts = forw_pts;
  undistortedPoints();
  prev_time = cur_time;
  Timer::stop("readImage");
  // ROS_INFO_STREAM(Timer::toString());
}

void FeatureTracker::rejectWithF() {
  Timer::start("rejectWithF");
  if (forw_pts.size() >= 8) {
    ROS_DEBUG("FM ransac begins");
    TicToc t_f;
    vector<cv::Point2f> un_cur_pts(cur_pts.size()), un_forw_pts(forw_pts.size());
    for (unsigned int i = 0; i < cur_pts.size(); i++) {
      Eigen::Vector3d tmp_p;
      m_camera->liftProjective(Eigen::Vector2d(cur_pts[i].x, cur_pts[i].y), tmp_p);
      tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
      tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
      un_cur_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());

      m_camera->liftProjective(Eigen::Vector2d(forw_pts[i].x, forw_pts[i].y), tmp_p);
      tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
      tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
      un_forw_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
    }

    vector<uchar> status;
    cv::findFundamentalMat(un_cur_pts, un_forw_pts, cv::FM_RANSAC, F_THRESHOLD, 0.99, status);
    int size_a = cur_pts.size();
    reducePoints(status);

    ROS_DEBUG("FM ransac: %d -> %lu: %f", size_a, forw_pts.size(), 1.0 * forw_pts.size() / size_a);
    ROS_DEBUG("FM ransac costs: %fms", t_f.toc());
  }
  Timer::stop("rejectWithF");
}

bool FeatureTracker::updateID(unsigned int i) {
  if (i < ids.size()) {
    if (ids[i] == -1)
      ids[i] = n_id++;
    return true;
  } else
    return false;
}

void FeatureTracker::readIntrinsicParameter(const string& calib_file) {
  ROS_INFO("reading paramerter of camera %s", calib_file.c_str());
  m_camera = CameraFactory::instance()->generateCameraFromYamlFile(calib_file);
}

void FeatureTracker::showUndistortion(const string& name) {
  cv::Mat undistortedImg(ROW + 600, COL + 600, CV_8UC1, cv::Scalar(0));
  vector<Eigen::Vector2d> distortedp, undistortedp;
  for (int i = 0; i < COL; i++)
    for (int j = 0; j < ROW; j++) {
      Eigen::Vector2d a(i, j);
      Eigen::Vector3d b;
      m_camera->liftProjective(a, b);
      distortedp.push_back(a);
      undistortedp.push_back(Eigen::Vector2d(b.x() / b.z(), b.y() / b.z()));
      // printf("%f,%f->%f,%f,%f\n)\n", a.x(), a.y(), b.x(), b.y(), b.z());
    }
  for (int i = 0; i < int(undistortedp.size()); i++) {
    cv::Mat pp(3, 1, CV_32FC1);
    pp.at<float>(0, 0) = undistortedp[i].x() * FOCAL_LENGTH + COL / 2;
    pp.at<float>(1, 0) = undistortedp[i].y() * FOCAL_LENGTH + ROW / 2;
    pp.at<float>(2, 0) = 1.0;
    // cout << trackerData[0].K << endl;
    // printf("%lf %lf\n", p.at<float>(1, 0), p.at<float>(0, 0));
    // printf("%lf %lf\n", pp.at<float>(1, 0), pp.at<float>(0, 0));
    if (pp.at<float>(1, 0) + 300 >= 0 && pp.at<float>(1, 0) + 300 < ROW + 600 &&
        pp.at<float>(0, 0) + 300 >= 0 && pp.at<float>(0, 0) + 300 < COL + 600) {
      undistortedImg.at<uchar>(pp.at<float>(1, 0) + 300, pp.at<float>(0, 0) + 300) =
          cur_img.at<uchar>(distortedp[i].y(), distortedp[i].x());
    } else {
      // ROS_ERROR("(%f %f) -> (%f %f)", distortedp[i].y, distortedp[i].x, pp.at<float>(1, 0),
      // pp.at<float>(0, 0));
    }
  }
  cv::imshow(name, undistortedImg);
  cv::waitKey(0);
}

void FeatureTracker::undistortedPoints() {
  Timer::start("undistortedPoints");
  cur_un_pts.clear();
  cur_un_pts_map.clear();
  // cv::undistortPoints(cur_pts, un_pts, K, cv::Mat());
  for (unsigned int i = 0; i < cur_pts.size(); i++) {
    Eigen::Vector2d a(cur_pts[i].x, cur_pts[i].y);
    Eigen::Vector3d b;
    m_camera->liftProjective(a, b);
    cur_un_pts.push_back(cv::Point2f(b.x() / b.z(), b.y() / b.z()));
    cur_un_pts_map.insert(make_pair(ids[i], cv::Point2f(b.x() / b.z(), b.y() / b.z())));
    // ROS_INFO("cur pts id %d %f %f", ids[i], cur_un_pts[i].x, cur_un_pts[i].y);
  }
  // caculate points velocity
  if (!prev_un_pts_map.empty()) {
    double dt = cur_time - prev_time;
    pts_velocity.clear();
    for (unsigned int i = 0; i < cur_un_pts.size(); i++) {
      if (ids[i] != -1) {
        std::map<int, cv::Point2f>::iterator it;
        it = prev_un_pts_map.find(ids[i]);
        if (it != prev_un_pts_map.end()) {
          double v_x = (cur_un_pts[i].x - it->second.x) / dt;
          double v_y = (cur_un_pts[i].y - it->second.y) / dt;
          pts_velocity.push_back(cv::Point2f(v_x, v_y));
        } else
          pts_velocity.push_back(cv::Point2f(0, 0));
      } else {
        pts_velocity.push_back(cv::Point2f(0, 0));
      }
    }
  } else {
    for (unsigned int i = 0; i < cur_pts.size(); i++) {
      pts_velocity.push_back(cv::Point2f(0, 0));
    }
  }
  prev_un_pts_map = cur_un_pts_map;
  Timer::stop("undistortedPoints");
}
