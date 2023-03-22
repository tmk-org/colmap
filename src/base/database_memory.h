#ifndef COLMAP_DATABASE_MEMORY_H
#define COLMAP_DATABASE_MEMORY_H

#include "database.h"

namespace colmap {

class MemoryDatabase : public DatabaseRoot {
 public:
  MemoryDatabase(size_t capacity = 1024);
  ~MemoryDatabase() override = default;

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

  InputSignal onLoad;

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

  void ClearMatches() override;
  void ClearCameras() override;
  void ClearImages() override;
  void ClearKeypoints() override;
  void ClearDescriptors() override;
  void ClearTwoViewGeometries() override;
  void ClearAllTables() override;

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

  void Connect(SignalFn fn) override;

 private:
  friend class DatabaseTransaction;

  void BeginTransaction() const override;
  void EndTransaction() const override;

  // Check if elements got removed from the database to only apply
  // the VACUUM command in such case
  mutable bool database_cleared_ = false;

  // Ensure that only one database object at a time updates the schema of a
  // database. Since the schema is updated every time a database is opened, this
  // is to ensure that there are no race conditions ("database locked" error
  // messages) when the user actually only intends to read from the database,
  // which requires to open it.
  static std::mutex update_schema_mutex_;

  // Used to ensure that only one transaction is active at the same time.
  std::mutex transaction_mutex_;

  std::vector<Camera> cameras_;
  //  std::vector<internal::ImageData> images_data_;
  std::map<image_t, FeatureKeypoints> keypoints_;
  std::map<image_t, FeatureDescriptors> descriptors_;
  std::vector<Image> images_;
  std::map<image_pair_t, FeatureMatches> matches_;
  std::map<image_pair_t, TwoViewGeometry> two_view_geometries_;
};



} // namespace colmap

#endif  // COLMAP_DATABASE_MEMORY_H
