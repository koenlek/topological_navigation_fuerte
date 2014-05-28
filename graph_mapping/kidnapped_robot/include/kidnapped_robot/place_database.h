#ifndef KIDNAPPED_ROBOT_PLACE_DATABASE_H
#define KIDNAPPED_ROBOT_PLACE_DATABASE_H

#include <sqlite3.h>
#include <vocabulary_tree/generic_tree.h>
#include <vocabulary_tree/database.h>
#include <frame_common/frame.h>
#include <tf/transform_datatypes.h>
#include <limits>

namespace kidnapped_robot {

class PlaceDatabase
{
  sqlite3 *persistent_db_;
  vt::GenericTree voctree_;
  vt::Database document_db_;
  typedef std::map<vt::DocId, int64_t> IdMap;
  IdMap doc_to_place_id_; // mapping between document_db_ id and place id exposed to user

  sqlite3_stmt *insert_stmt_;
  int map_pose_param_index_, optical_transform_param_index_, camera_param_index_;
  int disparities_param_index_, keypoints_param_index_;
  mutable sqlite3_stmt  *select_transforms_stmt_, *select_frame_stmt_, *select_spatial_stmt_, *select_images_stmt_;

public:
  PlaceDatabase();
  
  PlaceDatabase(const std::string& db_file, const std::string& tree_file, const std::string& weights_file);

  ~PlaceDatabase();

  void load(const std::string& db_file, const std::string& tree_file, const std::string& weights_file);

  static const int64_t AUTO_ID = LONG_MIN;
  
  int64_t add(ros::Time stamp, const tf::Pose& pose_in_map, const tf::Transform& optical_transform,
              const frame_common::Frame& frame, int64_t id = AUTO_ID);
  
  void findMatching(const frame_common::Frame& query_frame, size_t N, vt::Matches& matches) const;

  void findInRegion(cv::Rect_<double> bounding_box, std::vector<int64_t>& ids) const;

  void getFrame(int64_t id, frame_common::Frame& frame) const;

  void getTransforms(int64_t id, tf::Pose& pose_in_map, tf::Transform& optical_transform) const;

  void getImages(int64_t id, cv::Mat& left, cv::Mat& right);

  sqlite3* getSqlite() { return persistent_db_; }

  //int64_t size() const;

protected:
  void checkErr(int returned_code, const char* msg, int expected_code = SQLITE_OK) const;

  void loadPersistentDatabase(const std::string& db_file);

  void loadDocumentDatabase(const std::string& weights_file);

  void prepareReusedStatements();

  void extractTransform(tf::Transform& xfm, int start_index) const;

  void extractImage(cv::Mat& image, int col) const;

  void bindTransform(const tf::Transform& xfm, int param_index);

  void bindCameraParams(const frame_common::CamParams& cam);

  void bindImage(int col, const cv::Mat& image);

  template<typename T> void bindVector(sqlite3_stmt *stmt, int col, const std::vector<T>& vec);

  
};

} // namespace kidnapped_robot

#endif
