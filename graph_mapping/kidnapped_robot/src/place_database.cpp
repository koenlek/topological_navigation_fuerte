#include <kidnapped_robot/place_database.h>
#include <opencv/highgui.h>
#include <cassert>

namespace kidnapped_robot {

// Helper functions for storing/extracting STL vectors
template<typename T>
void PlaceDatabase::bindVector(sqlite3_stmt *stmt, int col, const std::vector<T>& vec)
{
  checkErr(sqlite3_bind_blob(stmt, col, &vec[0], vec.size() * sizeof(T), SQLITE_TRANSIENT),
           "Couldn't bind vector data");
}

template<typename T>
void extractVector(sqlite3_stmt *stmt, int col, std::vector<T>& vec, int num_items = -1)
{
  const void* data = sqlite3_column_blob(stmt, col);
  int bytes = sqlite3_column_bytes(stmt, col);
  if (num_items == -1)
    num_items = bytes / sizeof(T);
  assert(bytes == num_items * (int)sizeof(T));
  vec.resize(num_items);
  memcpy(&vec[0], data, bytes);
}

PlaceDatabase::PlaceDatabase()
  : persistent_db_(NULL),
    insert_stmt_(NULL), select_transforms_stmt_(NULL), select_frame_stmt_(NULL),
    select_spatial_stmt_(NULL), select_images_stmt_(NULL)
{
}

PlaceDatabase::PlaceDatabase(const std::string& db_file, const std::string& tree_file, const std::string& weights_file)
  : persistent_db_(NULL),
    insert_stmt_(NULL), select_transforms_stmt_(NULL), select_frame_stmt_(NULL),
    select_spatial_stmt_(NULL), select_images_stmt_(NULL)
{
  load(db_file, tree_file, weights_file);
}

PlaceDatabase::~PlaceDatabase()
{
  // Finalize prepared statements
  checkErr(sqlite3_finalize(insert_stmt_), "Couldn't finalize INSERT statement");
  checkErr(sqlite3_finalize(select_transforms_stmt_), "Couldn't finalize SELECT transforms statement");
  checkErr(sqlite3_finalize(select_frame_stmt_), "Couldn't finalize SELECT frame statement");
  checkErr(sqlite3_finalize(select_spatial_stmt_), "Couldn't finalize SELECT spatial statement");
  checkErr(sqlite3_finalize(select_images_stmt_), "Couldn't finalize SELECT images statement");

  // Close database connection
  checkErr(sqlite3_close(persistent_db_), "Couldn't close database");
}

void PlaceDatabase::load(const std::string& db_file, const std::string& tree_file, const std::string& weights_file)
{
  loadPersistentDatabase(db_file);
  voctree_.load(tree_file);
  loadDocumentDatabase(weights_file);
  prepareReusedStatements();
}

void PlaceDatabase::loadPersistentDatabase(const std::string& db_file)
{
  // Load associated data
  checkErr(sqlite3_open(db_file.c_str(), &persistent_db_), "Couldn't open database");

  // Create database schema
  static const char CREATE_TABLE_SQL[] =
    "CREATE TABLE IF NOT EXISTS places"
    // Row ID matches with doc ID in document_db_
    "(id INTEGER PRIMARY KEY,"
    // Time stamp
    " stamp INTEGER,"
    // Robot odometric pose in map frame
    " map_position_x DOUBLE, map_position_y DOUBLE, map_position_z DOUBLE,"
    " map_orientation_x DOUBLE, map_orientation_y DOUBLE, map_orientation_z DOUBLE, map_orientation_w DOUBLE,"
    // Transform to camera frame
    " optical_position_x DOUBLE, optical_position_y DOUBLE, optical_position_z DOUBLE,"
    " optical_orientation_x DOUBLE, optical_orientation_y DOUBLE,"
    " optical_orientation_z DOUBLE, optical_orientation_w DOUBLE,"
    // Camera intrinsics, stereo if cam_tx != 0.0
    " cam_fx DOUBLE, cam_fy DOUBLE, cam_cx DOUBLE, cam_cy DOUBLE, cam_tx DOUBLE,"
    // For stereo, keypoint disparities for points with good stereo matches
    " disparities BLOB, good_points BLOB,"
    // Keypoints and descriptors for matching
    " num_keypoints INTEGER, keypoints BLOB, descriptor_length INTEGER, descriptor_data BLOB,"
    // Document vector for repopulating document_db_
    " document BLOB,"
    // Optionally save images (JPEG compressed) for visualization
    " left_image BLOB, right_image BLOB)";
  checkErr(sqlite3_exec(persistent_db_, CREATE_TABLE_SQL, NULL, NULL, NULL), "Couldn't CREATE TABLE");

  // Create index on map_position_x to speed up spatial queries
  /// @todo Could use R*Tree module instead
  static const char CREATE_INDEX_SQL[] = "CREATE INDEX IF NOT EXISTS place_index ON places(map_position_x)";
  checkErr(sqlite3_exec(persistent_db_, CREATE_INDEX_SQL, NULL, NULL, NULL), "Couldn't CREATE INDEX");
}

void PlaceDatabase::loadDocumentDatabase(const std::string& weights_file)
{
  document_db_.loadWeights(weights_file);

  // Repopulate documents
  static const char SELECT_DOC_SQL[] = "SELECT id, document FROM places";
  sqlite3_stmt *stmt = NULL;
  checkErr(sqlite3_prepare_v2(persistent_db_, SELECT_DOC_SQL, sizeof(SELECT_DOC_SQL), &stmt, NULL),
           "Couldn't prepare SELECT documents statement");

  vt::Document words;
  int err;
  int64_t id = 0;
  while ((err = sqlite3_step(stmt)) == SQLITE_ROW) {
    id = sqlite3_column_int(stmt, 0);
    extractVector(stmt, 1, words);
    vt::DocId doc_id = document_db_.insert(words);
    doc_to_place_id_[doc_id] = id;
  }
  printf("Last loaded document id = %d\n", (int)id);
  checkErr(err, "Error executing spatial query", SQLITE_DONE);

  checkErr(sqlite3_finalize(stmt), "Couldn't finalize SELECT documents statement");
}

void PlaceDatabase::prepareReusedStatements()
{
  // Prepared statement for inserting some data
  static const char INSERT_SQL[] =
    "INSERT INTO places VALUES "
    "($id, $stamp,"
    " $map_pos_x, $map_pos_y, $map_pos_z, $map_ori_x, $map_ori_y, $map_ori_z, $map_ori_w,"
    " $opt_pos_x, $opt_pos_y, $opt_pos_z, $opt_ori_x, $opt_ori_y, $opt_ori_z, $opt_ori_w,"
    " $fx, $fy, $cx, $cy, $tx, $disps, $good_pts,"
    " $num_pts, $keypts, $desc_len, $desc_data, $document, $left, $right)";
  checkErr(sqlite3_prepare_v2(persistent_db_, INSERT_SQL, sizeof(INSERT_SQL), &insert_stmt_, NULL),
           "Couldn't prepare INSERT statement");
  
  map_pose_param_index_ = sqlite3_bind_parameter_index(insert_stmt_, "$map_pos_x");
  optical_transform_param_index_ = sqlite3_bind_parameter_index(insert_stmt_, "$opt_pos_x");
  camera_param_index_ = sqlite3_bind_parameter_index(insert_stmt_, "$fx");
  disparities_param_index_ = sqlite3_bind_parameter_index(insert_stmt_, "$disps");
  keypoints_param_index_ = sqlite3_bind_parameter_index(insert_stmt_, "$num_pts");
  
  assert(map_pose_param_index_ != 0);
  assert(optical_transform_param_index_ != 0);
  assert(camera_param_index_ != 0);
  assert(disparities_param_index_ != 0);
  assert(keypoints_param_index_ != 0);

  // Prepared statement for getting transform data for a particular place
  static const char SELECT_TRANSFORMS_SQL[] =
    "SELECT map_position_x, map_position_y, map_position_z,"
    "       map_orientation_x, map_orientation_y, map_orientation_z, map_orientation_w,"
    "       optical_position_x, optical_position_y, optical_position_z,"
    "       optical_orientation_x, optical_orientation_y, optical_orientation_z, optical_orientation_w "
    "FROM places WHERE id = ?";
  checkErr(sqlite3_prepare_v2(persistent_db_, SELECT_TRANSFORMS_SQL, sizeof(SELECT_TRANSFORMS_SQL),
                              &select_transforms_stmt_, NULL),
           "Couldn't prepare SELECT transforms statement");

  // Prepared statement for getting frame data for a particular place
  static const char SELECT_FRAME_SQL[] =
    "SELECT cam_fx, cam_fy, cam_cx, cam_cy, cam_tx, disparities, good_points,"
    "       num_keypoints, keypoints, descriptor_length, descriptor_data "
    "FROM places WHERE id = ?";
  checkErr(sqlite3_prepare_v2(persistent_db_, SELECT_FRAME_SQL, sizeof(SELECT_FRAME_SQL),
                              &select_frame_stmt_, NULL),
           "Couldn't prepare SELECT frame statement");

  // Prepared statement for spatial queries
  static const char SELECT_SPATIAL_SQL[] =
    "SELECT id FROM places WHERE map_position_x BETWEEN $min_x AND $max_x AND"
    "                            map_position_y BETWEEN $min_y AND $max_y"
    "                      ORDER BY id";
  checkErr(sqlite3_prepare_v2(persistent_db_, SELECT_SPATIAL_SQL, sizeof(SELECT_SPATIAL_SQL),
                              &select_spatial_stmt_, NULL),
           "Couldn't prepare SELECT spatial statement");

  static const char SELECT_IMAGES_SQL[] = "SELECT left_image, right_image FROM places WHERE id = ?";
  checkErr(sqlite3_prepare_v2(persistent_db_, SELECT_IMAGES_SQL, sizeof(SELECT_IMAGES_SQL),
                              &select_images_stmt_, NULL),
           "Couldn't prepare SELECT images statement");
}

int64_t PlaceDatabase::add(ros::Time stamp, const tf::Pose& pose_in_map, const tf::Transform& optical_transform,
                           const frame_common::Frame& frame, int64_t id)
{
  assert((int)frame.kpts.size() == frame.dtors.rows);

  // Add quantized image descriptor to document database
  vt::Document words(frame.dtors.rows);
  for (int i = 0; i < frame.dtors.rows; ++i)
    words[i] = voctree_.quantize(frame.dtors.row(i));
  vt::DocId doc_id = document_db_.insert(words);

  // Record doc id -> place id mapping
  if (id == AUTO_ID)
    id = doc_id;
  doc_to_place_id_[doc_id] = id;
  
  /// @todo Move persistent db stuff to separate function
  checkErr(sqlite3_bind_int64(insert_stmt_, 1, id), "Couldn't bind row id");
  
  // Bind time stamp encoded as 64-bit integer
  sqlite3_int64 time = stamp.toNSec();
  checkErr(sqlite3_bind_int64(insert_stmt_, 2, time), "Couldn't bind stamp");

  // Bind current map pose and camera transform
  bindTransform(pose_in_map, map_pose_param_index_);
  bindTransform(optical_transform, optical_transform_param_index_);

  // Bind camera parameters
  bindCameraParams(frame.cam);

  // Bind disparities and "good points" list
  int disp_index = disparities_param_index_;
  bindVector(insert_stmt_, disp_index, frame.disps);
  bindVector(insert_stmt_, disp_index + 1, frame.goodPts);
  
  // Bind keypoint data
  int keypt_index = keypoints_param_index_;
  checkErr(sqlite3_bind_int(insert_stmt_, keypt_index, frame.kpts.size()), "Couldn't bind num_keypoints");
  bindVector(insert_stmt_, keypt_index + 1, frame.kpts);

  // Bind descriptor data. step of cv::Mat is implicit, blob length / num keypoints.
  checkErr(sqlite3_bind_int(insert_stmt_, keypt_index + 2, frame.dtors.cols), "Couldn't bind descriptor_length");
  checkErr(sqlite3_bind_blob(insert_stmt_, keypt_index + 3, frame.dtors.data,
                             frame.dtors.rows * frame.dtors.step, SQLITE_TRANSIENT),
           "Couldn't bind descriptor data");

  // Bind document vector data
  bindVector(insert_stmt_, keypt_index + 4, words);

  // Save images if present
  bindImage(keypt_index + 5, frame.img);
  bindImage(keypt_index + 6, frame.imgRight);

  // Execute INSERT statement
  checkErr(sqlite3_step(insert_stmt_), "INSERT statement not done", SQLITE_DONE);
  
  // Reset prepared statement to reuse on next call
  checkErr(sqlite3_reset(insert_stmt_), "Couldn't reset INSERT statement");
  checkErr(sqlite3_clear_bindings(insert_stmt_), "Couldn't clear bindings on INSERT statement");

  // Return row id of newly-added place
  return id;
}

void PlaceDatabase::findMatching(const frame_common::Frame& query_frame, size_t N, vt::Matches& matches) const
{
  /// @todo Copy-pasta from add()
  vt::Document words(query_frame.dtors.rows);
  for (int i = 0; i < query_frame.dtors.rows; ++i)
    words[i] = voctree_.quantize(query_frame.dtors.row(i));
  document_db_.find(words, N, matches);
  // Now map document ids to user-visible place ids
  for (int i = 0; i < (int)matches.size(); ++i)
  {
    vt::DocId &id = matches[i].id;
    IdMap::const_iterator iter = doc_to_place_id_.find(id);
    assert(iter != doc_to_place_id_.end());
    id = iter->second;
  }
}

void PlaceDatabase::findInRegion(cv::Rect_<double> bounding_box, std::vector<int64_t>& ids) const
{
  // Bind bounding box in the map to the query
  checkErr(sqlite3_bind_double(select_spatial_stmt_, 1, bounding_box.x), "Bind error");
  checkErr(sqlite3_bind_double(select_spatial_stmt_, 2, bounding_box.x + bounding_box.width), "Bind error");
  checkErr(sqlite3_bind_double(select_spatial_stmt_, 3, bounding_box.y), "Bind error");
  checkErr(sqlite3_bind_double(select_spatial_stmt_, 4, bounding_box.y + bounding_box.height), "Bind error");

  ids.clear();
  int err;
  while ((err = sqlite3_step(select_spatial_stmt_)) == SQLITE_ROW)
    ids.push_back( sqlite3_column_int(select_spatial_stmt_, 0) );
  
  checkErr(err, "Error executing spatial query", SQLITE_DONE);

  // Reset prepared statement to reuse on next call
  checkErr(sqlite3_reset(select_spatial_stmt_), "Couldn't reset SELECT spatial statement");
  checkErr(sqlite3_clear_bindings(select_spatial_stmt_), "Couldn't clear bindings on SELECT spatial statment");
}

void PlaceDatabase::getFrame(int64_t id, frame_common::Frame& frame) const
{
  checkErr(sqlite3_bind_int64(select_frame_stmt_, 1, id), "Couldn't bind row id");
  checkErr(sqlite3_step(select_frame_stmt_), "SELECT frame didn't return data", SQLITE_ROW);

  // Extract camera parameters
  frame_common::CamParams cam;
  cam.fx = sqlite3_column_double(select_frame_stmt_, 0);
  cam.fy = sqlite3_column_double(select_frame_stmt_, 1);
  cam.cx = sqlite3_column_double(select_frame_stmt_, 2);
  cam.cy = sqlite3_column_double(select_frame_stmt_, 3);
  cam.tx = sqlite3_column_double(select_frame_stmt_, 4);
  frame.setCamParams(cam);

  // Extract disparities and "good points" index
  int num_keypoints = sqlite3_column_int(select_frame_stmt_, 7);
  extractVector(select_frame_stmt_, 5, frame.disps, num_keypoints);
  extractVector(select_frame_stmt_, 6, frame.goodPts, num_keypoints);

  // Extract keypoints and descriptors
  extractVector(select_frame_stmt_, 8, frame.kpts, num_keypoints);
  int descriptor_length = sqlite3_column_int(select_frame_stmt_, 9);
  assert(descriptor_length > 0);
  const void* desc_data = sqlite3_column_blob(select_frame_stmt_, 10);
  int desc_bytes = sqlite3_column_bytes(select_frame_stmt_, 10);
  int step = desc_bytes / num_keypoints;
  assert(step >= descriptor_length * (int)sizeof(float));
  const cv::Mat tmp(num_keypoints, descriptor_length, cv::DataType<float>::type,
                    const_cast<void*>(desc_data), step);
  tmp.copyTo(frame.dtors);

  // Finally, reconstruct frame.pts from keypoints and disparities
  frame.pts.resize(num_keypoints);
  for (int i = 0; i < num_keypoints; ++i) {
    if (frame.goodPts[i]) {
      Eigen::Vector3d pt(frame.kpts[i].pt.x, frame.kpts[i].pt.y, frame.disps[i]);
      frame.pts[i].head<3>() = frame.pix2cam(pt);
      frame.pts[i](3) = 1.0;
    }
  }
  
  checkErr(sqlite3_step(select_frame_stmt_), "SELECT frame returned more than one row", SQLITE_DONE);
  checkErr(sqlite3_reset(select_frame_stmt_), "Couldn't reset SELECT frame statement");
}

void PlaceDatabase::getTransforms(int64_t id, tf::Pose& pose_in_map, tf::Transform& optical_transform) const
{
  checkErr(sqlite3_bind_int64(select_transforms_stmt_, 1, id), "Couldn't bind row id");
  checkErr(sqlite3_step(select_transforms_stmt_), "SELECT transforms didn't return data", SQLITE_ROW);

  extractTransform(pose_in_map, 0);
  extractTransform(optical_transform, 7);

  checkErr(sqlite3_step(select_transforms_stmt_), "SELECT transforms returned more than one row", SQLITE_DONE);
  checkErr(sqlite3_reset(select_transforms_stmt_), "Couldn't reset SELECT transforms statement");
}

void PlaceDatabase::getImages(int64_t id, cv::Mat& left, cv::Mat& right)
{
  checkErr(sqlite3_bind_int64(select_images_stmt_, 1, id), "Couldn't bind row id");
  checkErr(sqlite3_step(select_images_stmt_), "SELECT images didn't return data", SQLITE_ROW);

  extractImage(left, 0);
  extractImage(right, 1);

  checkErr(sqlite3_step(select_images_stmt_), "SELECT images returned more than one row", SQLITE_DONE);
  checkErr(sqlite3_reset(select_images_stmt_), "Couldn't reset SELECT images statement");
}

void PlaceDatabase::checkErr(int returned_code, const char* msg, int expected_code) const
{
  if (returned_code != expected_code) {
    char full_msg[512];
    snprintf(full_msg, sizeof(full_msg), "%s: %s\n", msg, sqlite3_errmsg(persistent_db_));
    throw std::runtime_error(full_msg);
  }
}

void PlaceDatabase::extractTransform(tf::Transform& xfm, int start_index) const
{
  xfm.getOrigin().setValue(sqlite3_column_double(select_transforms_stmt_, start_index),
                           sqlite3_column_double(select_transforms_stmt_, start_index + 1),
                           sqlite3_column_double(select_transforms_stmt_, start_index + 2));

  tf::Quaternion q(sqlite3_column_double(select_transforms_stmt_, start_index + 3),
                 sqlite3_column_double(select_transforms_stmt_, start_index + 4),
                 sqlite3_column_double(select_transforms_stmt_, start_index + 5),
                 sqlite3_column_double(select_transforms_stmt_, start_index + 6));
  xfm.setRotation(q);
}

void PlaceDatabase::bindTransform(const tf::Transform& xfm, int param_index)
{
  // Position
  checkErr(sqlite3_bind_double(insert_stmt_, param_index + 0, xfm.getOrigin().x()), "Bind error");
  checkErr(sqlite3_bind_double(insert_stmt_, param_index + 1, xfm.getOrigin().y()), "Bind error");
  checkErr(sqlite3_bind_double(insert_stmt_, param_index + 2, xfm.getOrigin().z()), "Bind error");

  // Orientation
  tf::Quaternion q = xfm.getRotation();
  checkErr(sqlite3_bind_double(insert_stmt_, param_index + 3, q.x()), "Bind error");
  checkErr(sqlite3_bind_double(insert_stmt_, param_index + 4, q.y()), "Bind error");
  checkErr(sqlite3_bind_double(insert_stmt_, param_index + 5, q.z()), "Bind error");
  checkErr(sqlite3_bind_double(insert_stmt_, param_index + 6, q.w()), "Bind error");
}

void PlaceDatabase::bindCameraParams(const frame_common::CamParams& cam)
{
  int idx = camera_param_index_;
  checkErr(sqlite3_bind_double(insert_stmt_, idx + 0, cam.fx), "Bind error");
  checkErr(sqlite3_bind_double(insert_stmt_, idx + 1, cam.fy), "Bind error");
  checkErr(sqlite3_bind_double(insert_stmt_, idx + 2, cam.cx), "Bind error");
  checkErr(sqlite3_bind_double(insert_stmt_, idx + 3, cam.cy), "Bind error");
  checkErr(sqlite3_bind_double(insert_stmt_, idx + 4, cam.tx), "Bind error");
}

void PlaceDatabase::extractImage(cv::Mat& image, int col) const
{
  std::vector<uchar> buf;
  extractVector(select_images_stmt_, col, buf);
  if (buf.size() == 0)
    image = cv::Mat();
  else {
    cv::Mat mat(1, buf.size(), CV_8UC1, &buf[0]);
    image = cv::imdecode(mat, CV_LOAD_IMAGE_ANYCOLOR);
  }
}

void PlaceDatabase::bindImage(int col, const cv::Mat& image)
{
  if (image.empty()) return;

  std::vector<uchar> buf;
  std::vector<int> params(2);
  params[0] = CV_IMWRITE_JPEG_QUALITY;
  params[1] = 80; // 80 % quality
  cv::imencode(".jpeg", image, buf, params);
  bindVector(insert_stmt_, col, buf);
}

} // namespace kidnapped_robot
