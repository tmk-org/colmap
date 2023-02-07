#ifndef COLMAP_DATABASE_SQLITE_H
#define COLMAP_DATABASE_SQLITE_H

#include "database.h"

namespace colmap {

// Database class to read and write images, features, cameras, matches, etc.
// from a SQLite database. The class is not thread-safe and must not be accessed
// concurrently. The class is optimized for single-thread speed and for optimal
// performance, wrap multiple method calls inside a leading `BeginTransaction`
// and trailing `EndTransaction`.

class Database : public DatabaseRoot {
 public:
  const static int kSchemaVersion = 1;

  Database();
  explicit Database(const std::string& path);
  ~Database() override;

  // Open and close database. The same database should not be opened
  // concurrently in multiple threads or processes.
  void Open(const std::string& path) override;
  void Close() override;

  // Check if entry already exists in database. For image pairs, the order of
  // `image_id1` and `image_id2` does not matter.
  bool ExistsCamera(const camera_t camera_id) const override;
  bool ExistsImage(const image_t image_id) const override;
  bool ExistsImageWithName(std::string name) const override;
  bool ExistsKeypoints(const image_t image_id) const override;
  bool ExistsDescriptors(const image_t image_id) const override;
  bool ExistsMatches(const image_t image_id1, const image_t image_id2) const override;
  bool ExistsInlierMatches(const image_t image_id1,
                           const image_t image_id2) const override;

  // Number of rows in `cameras` table.
  size_t NumCameras() const override;

  //  Number of rows in `images` table.
  size_t NumImages() const override;

  // Sum of `rows` column in `keypoints` table, i.e. number of total keypoints.
  size_t NumKeypoints() const override;

  // The number of keypoints for the image with most features.
  size_t MaxNumKeypoints() const override;

  // Number of descriptors for specific image.
  size_t NumKeypointsForImage(const image_t image_id) const override;

  // Sum of `rows` column in `descriptors` table,
  // i.e. number of total descriptors.
  size_t NumDescriptors() const override;

  // The number of descriptors for the image with most features.
  size_t MaxNumDescriptors() const override;

  // Number of descriptors for specific image.
  size_t NumDescriptorsForImage(const image_t image_id) const override;

  // Sum of `rows` column in `matches` table, i.e. number of total matches.
  size_t NumMatches() const override;

  // Sum of `rows` column in `two_view_geometries` table,
  // i.e. number of total inlier matches.
  size_t NumInlierMatches() const override;

  // Number of rows in `matches` table.
  size_t NumMatchedImagePairs() const override;

  // Number of rows in `two_view_geometries` table.
  size_t NumVerifiedImagePairs() const override;

  // Read an existing entry in the database. The user is responsible for making
  // sure that the entry actually exists. For image pairs, the order of
  // `image_id1` and `image_id2` does not matter.
  Camera ReadCamera(const camera_t camera_id) const override;
  std::vector<Camera> ReadAllCameras() const override;

  Image ReadImage(const image_t image_id) const override;
  Image ReadImageWithName(const std::string& name) const override;
  std::vector<Image> ReadAllImages() const override;

  FeatureKeypoints ReadKeypoints(const image_t image_id) const override;
  FeatureDescriptors ReadDescriptors(const image_t image_id) const override;

  FeatureMatches ReadMatches(const image_t image_id1,
                             const image_t image_id2) const override;
  std::vector<std::pair<image_pair_t, FeatureMatches>> ReadAllMatches() const override;

  TwoViewGeometry ReadTwoViewGeometry(const image_t image_id1,
                                      const image_t image_id2) const override;
  void ReadTwoViewGeometries(
      std::vector<image_pair_t>* image_pair_ids,
      std::vector<TwoViewGeometry>* two_view_geometries) const override;

  // Read all image pairs that have an entry in the `NumVerifiedImagePairs`
  // table with at least one inlier match and their number of inlier matches.
  void ReadTwoViewGeometryNumInliers(
      std::vector<std::pair<image_t, image_t>>* image_pairs,
      std::vector<int>* num_inliers) const override;

  // Add new camera and return its database identifier. If `use_camera_id`
  // is false a new identifier is automatically generated.
  camera_t WriteCamera(const Camera& camera,
                       const bool use_camera_id = false) override;

  // Add new image and return its database identifier. If `use_image_id`
  // is false a new identifier is automatically generated.
  image_t WriteImage(const Image& image, const bool use_image_id = false) override;

  // Write a new entry in the database. The user is responsible for making sure
  // that the entry does not yet exist. For image pairs, the order of
  // `image_id1` and `image_id2` does not matter.
  void WriteKeypoints(const image_t image_id,
                      const FeatureKeypoints& keypoints) override;
  void WriteDescriptors(const image_t image_id,
                        const FeatureDescriptors& descriptors) override;
  void WriteMatches(const image_t image_id1, const image_t image_id2,
                    const FeatureMatches& matches) override;
  void WriteTwoViewGeometry(const image_t image_id1, const image_t image_id2,
                            const TwoViewGeometry& two_view_geometry) override;

  // Update an existing camera in the database. The user is responsible for
  // making sure that the entry already exists.
  void UpdateCamera(const Camera& camera) override;

  // Update an existing image in the database. The user is responsible for
  // making sure that the entry already exists.
  void UpdateImage(const Image& image) override;

  // Delete matches of an image pair.
  void DeleteMatches(const image_t image_id1, const image_t image_id2) override;

  // Delete inlier matches of an image pair.
  void DeleteInlierMatches(const image_t image_id1,
                           const image_t image_id2) override;

  // Clear all database tables
  void ClearAllTables() override;

  // Clear the entire cameras table
  void ClearCameras() override;

  // Clear the entire images, keypoints, and descriptors tables
  void ClearImages() override;

  // Clear the entire descriptors table
  void ClearDescriptors() override;

  // Clear the entire keypoints table
  void ClearKeypoints() override;

  // Clear the entire matches table.
  void ClearMatches() override;

  // Clear the entire inlier matches table.
  void ClearTwoViewGeometries() override;

  void Connect(SignalFn fn) override {};

 private:
  friend class DatabaseTransaction;

  // Combine multiple queries into one transaction by wrapping a code section
  // into a `BeginTransaction` and `EndTransaction`. You can create a scoped
  // transaction with `DatabaseTransaction` that ends when the transaction
  // object is destructed. Combining queries results in faster transaction time
  // due to reduced locking of the database etc.
  void BeginTransaction() const override;
  void EndTransaction() const override;

  // Prepare SQL statements once at construction of the database, and reuse
  // the statements for multiple queries by resetting their states.
  void PrepareSQLStatements();
  void FinalizeSQLStatements();

  // Create database tables, if not existing, called when opening a database.
  void CreateTables() const;
  void CreateCameraTable() const;
  void CreateImageTable() const;
  void CreateKeypointsTable() const;
  void CreateDescriptorsTable() const;
  void CreateMatchesTable() const;
  void CreateTwoViewGeometriesTable() const;

  void UpdateSchema() const;

  bool ExistsTable(const std::string& table_name) const;
  bool ExistsColumn(const std::string& table_name,
                    const std::string& column_name) const;

  bool ExistsRowId(sqlite3_stmt* sql_stmt, const sqlite3_int64 row_id) const;
  bool ExistsRowString(sqlite3_stmt* sql_stmt,
                       const std::string& row_entry) const;

  size_t CountRows(const std::string& table) const;
  size_t CountRowsForEntry(sqlite3_stmt* sql_stmt,
                           const sqlite3_int64 row_id) const;
  size_t SumColumn(const std::string& column, const std::string& table) const;
  size_t MaxColumn(const std::string& column, const std::string& table) const;

  sqlite3* database_ = nullptr;
  // Check if elements got removed from the database to only apply
  // the VACUUM command in such case
  mutable bool database_cleared_ = false;
  // A collection of all `sqlite3_stmt` objects for deletion in the destructor.
  std::vector<sqlite3_stmt*> sql_stmts_;

  // num_*
  sqlite3_stmt* sql_stmt_num_keypoints_ = nullptr;
  sqlite3_stmt* sql_stmt_num_descriptors_ = nullptr;

  // exists_*
  sqlite3_stmt* sql_stmt_exists_camera_ = nullptr;
  sqlite3_stmt* sql_stmt_exists_image_id_ = nullptr;
  sqlite3_stmt* sql_stmt_exists_image_name_ = nullptr;
  sqlite3_stmt* sql_stmt_exists_keypoints_ = nullptr;
  sqlite3_stmt* sql_stmt_exists_descriptors_ = nullptr;
  sqlite3_stmt* sql_stmt_exists_matches_ = nullptr;
  sqlite3_stmt* sql_stmt_exists_two_view_geometry_ = nullptr;

  // add_*
  sqlite3_stmt* sql_stmt_add_camera_ = nullptr;
  sqlite3_stmt* sql_stmt_add_image_ = nullptr;

  // update_*
  sqlite3_stmt* sql_stmt_update_camera_ = nullptr;
  sqlite3_stmt* sql_stmt_update_image_ = nullptr;

  // read_*
  sqlite3_stmt* sql_stmt_read_camera_ = nullptr;
  sqlite3_stmt* sql_stmt_read_cameras_ = nullptr;
  sqlite3_stmt* sql_stmt_read_image_id_ = nullptr;
  sqlite3_stmt* sql_stmt_read_image_name_ = nullptr;
  sqlite3_stmt* sql_stmt_read_images_ = nullptr;
  sqlite3_stmt* sql_stmt_read_keypoints_ = nullptr;
  sqlite3_stmt* sql_stmt_read_descriptors_ = nullptr;
  sqlite3_stmt* sql_stmt_read_matches_ = nullptr;
  sqlite3_stmt* sql_stmt_read_matches_all_ = nullptr;
  sqlite3_stmt* sql_stmt_read_two_view_geometry_ = nullptr;
  sqlite3_stmt* sql_stmt_read_two_view_geometries_ = nullptr;
  sqlite3_stmt* sql_stmt_read_two_view_geometry_num_inliers_ = nullptr;

  // write_*
  sqlite3_stmt* sql_stmt_write_keypoints_ = nullptr;
  sqlite3_stmt* sql_stmt_write_descriptors_ = nullptr;
  sqlite3_stmt* sql_stmt_write_matches_ = nullptr;
  sqlite3_stmt* sql_stmt_write_two_view_geometry_ = nullptr;

  // delete_*
  sqlite3_stmt* sql_stmt_delete_matches_ = nullptr;
  sqlite3_stmt* sql_stmt_delete_two_view_geometry_ = nullptr;

  // clear_*
  sqlite3_stmt* sql_stmt_clear_cameras_ = nullptr;
  sqlite3_stmt* sql_stmt_clear_images_ = nullptr;
  sqlite3_stmt* sql_stmt_clear_descriptors_ = nullptr;
  sqlite3_stmt* sql_stmt_clear_keypoints_ = nullptr;
  sqlite3_stmt* sql_stmt_clear_matches_ = nullptr;
  sqlite3_stmt* sql_stmt_clear_two_view_geometries_ = nullptr;
};

} // namespace colmap

#endif  // COLMAP_DATABASE_SQLITE_H
