#include "database_memory.h"

namespace colmap {

void MemoryDatabase::Open(const std::string& path) {
  Close();
}

void MemoryDatabase::Close() {
}

bool MemoryDatabase::ExistsCamera(const camera_t camera_id) const {
  return camera_id - 1 < cameras_.size();
}

bool MemoryDatabase::ExistsImage(const image_t image_id) const {
  return image_id - 1 < images_.size();
}

bool MemoryDatabase::ExistsImageWithName(std::string name) const {
  for (const auto& image : images_) {
    if (image.Name() == name) {
      return true;
    }
  }
  return false;
}

bool MemoryDatabase::ExistsKeypoints(const image_t image_id) const {
  return keypoints_.count(image_id - 1);
}

bool MemoryDatabase::ExistsDescriptors(const image_t image_id) const {
  return descriptors_.count(image_id - 1);
}

bool MemoryDatabase::ExistsMatches(const image_t image_id1,
                                   const image_t image_id2) const {
  image_pair_t image_pair = MemoryDatabase::ImagePairToPairId(image_id1, image_id2);
  return matches_.count(image_pair);
}

bool MemoryDatabase::ExistsInlierMatches(const image_t image_id1,
                                         const image_t image_id2) const {
  image_pair_t image_pair = MemoryDatabase::ImagePairToPairId(image_id1, image_id2);
  return two_view_geometries_.count(image_pair);
}

size_t MemoryDatabase::NumCameras() const {
  return cameras_.size();
}

size_t MemoryDatabase::NumImages() const {
  return images_.size();
}

size_t MemoryDatabase::NumKeypoints() const {
  size_t num_keypoints = 0;
  for (const auto& keypoint : keypoints_) {
    num_keypoints += keypoint.second.size();
  }
  return num_keypoints;
}

size_t MemoryDatabase::MaxNumKeypoints() const {
  size_t max_num_keypoints = std::numeric_limits<size_t>::min();
  for (const auto& keypoint : keypoints_) {
    if (keypoint.second.size() > max_num_keypoints) {
      max_num_keypoints = keypoint.second.size();
    }
  }
  return max_num_keypoints;
}

size_t MemoryDatabase::NumKeypointsForImage(const image_t image_id) const {
  if (!ExistsImage(image_id) || keypoints_.empty()) {
    return 0;
  }
  return keypoints_.at(image_id - 1).size();
}

size_t MemoryDatabase::NumDescriptors() const {
  size_t num_descriptors = 0;
  for (const auto& descriptor : descriptors_) {
    num_descriptors += descriptor.second.rows();
  }
  return num_descriptors;
}

size_t MemoryDatabase::MaxNumDescriptors() const {
  size_t max_num_descriptors = std::numeric_limits<size_t>::min();
  for (const auto& descriptor : descriptors_) {
    if (descriptor.second.rows() > max_num_descriptors) {
      max_num_descriptors = descriptor.second.rows();
    }
  }
  return max_num_descriptors;
}

size_t MemoryDatabase::NumDescriptorsForImage(const image_t image_id) const {
  if (!ExistsImage(image_id) || descriptors_.empty()) {
    return 0;
  }
  return descriptors_.at(image_id - 1).rows();
}

size_t MemoryDatabase::NumMatches() const {
  size_t num_matches = 0;
  for (const auto& match : matches_) {
    num_matches += match.second.size();
  }
  return num_matches;
}

size_t MemoryDatabase::NumInlierMatches() const {
  size_t num_inliers = 0;
  for (const auto& inlier : two_view_geometries_) {
    num_inliers += inlier.second.inlier_matches.size();
  }
  return num_inliers;
}

size_t MemoryDatabase::NumMatchedImagePairs() const {
  return matches_.size();
}

size_t MemoryDatabase::NumVerifiedImagePairs() const {
  return two_view_geometries_.size();
}

Camera MemoryDatabase::ReadCamera(const camera_t camera_id) const {
  return cameras_[camera_id - 1];
}

std::vector<Camera> MemoryDatabase::ReadAllCameras() const {
  return cameras_;
}

Image MemoryDatabase::ReadImage(const image_t image_id) const {
  return images_[image_id - 1];
}

Image MemoryDatabase::ReadImageWithName(const std::string& name) const {
  for (const auto& image : images_) {
    if (image.Name() == name) {
      return image;
    }
  }
}

std::vector<Image> MemoryDatabase::ReadAllImages() const {
  return images_;
}

FeatureKeypoints MemoryDatabase::ReadKeypoints(const image_t image_id) const {
  return keypoints_.at(image_id - 1);
}

FeatureDescriptors MemoryDatabase::ReadDescriptors(const image_t image_id) const {
  return descriptors_.at(image_id - 1);
}

FeatureMatches MemoryDatabase::ReadMatches(image_t image_id1,
                                           image_t image_id2) const {
  const image_pair_t pair_id = MemoryDatabase::ImagePairToPairId(image_id1, image_id2);
  return matches_.at(pair_id);
}

std::vector<std::pair<image_pair_t, FeatureMatches>> MemoryDatabase::ReadAllMatches()
const {
  std::vector<std::pair<image_pair_t, FeatureMatches>> all_matches;
  for (const auto& match : matches_) {
    all_matches.emplace_back(match.first, match.second);
  }

  return all_matches;
}

TwoViewGeometry MemoryDatabase::ReadTwoViewGeometry(const image_t image_id1,
                                                    const image_t image_id2) const {
  image_pair_t image_pair = MemoryDatabase::ImagePairToPairId(image_id1, image_id2);
  TwoViewGeometry twoViewGeometry = two_view_geometries_.at(image_pair);
  if (MemoryDatabase::SwapImagePair(image_id1, image_id2)) {
    twoViewGeometry.Invert();
  }
  return twoViewGeometry;
}

void MemoryDatabase::ReadTwoViewGeometries(
    std::vector<image_pair_t>* image_pair_ids,
    std::vector<TwoViewGeometry>* two_view_geometries) const {
  for(auto it = two_view_geometries_.begin(); it != two_view_geometries_.end(); ++it) {
    image_pair_ids->push_back(it->first);
    two_view_geometries->push_back(it->second);
  }
}

void MemoryDatabase::ReadTwoViewGeometryNumInliers(
    std::vector<std::pair<image_t, image_t>>* image_pairs,
    std::vector<int>* num_inliers) const {
  const auto num_inlier_matches = NumInlierMatches();
  image_pairs->reserve(num_inlier_matches);
  num_inliers->reserve(num_inlier_matches);

  for (const auto& two_view_geometry : two_view_geometries_) {
    image_t image_id1;
    image_t image_id2;
    MemoryDatabase::PairIdToImagePair(two_view_geometry.first, &image_id1, &image_id2);
    image_pairs->emplace_back(image_id1, image_id2);
    num_inliers->push_back(two_view_geometry.second.inlier_matches.size());
  }
}

camera_t MemoryDatabase::WriteCamera(const Camera& camera,
                                     const bool use_camera_id) {
  cameras_.push_back(camera);
  // TODO: Need to doublecheck
  if (use_camera_id) {
    cameras_.back().SetCameraId(camera.CameraId());
  }
  else {
    cameras_.back().SetCameraId(cameras_.size());
  }
  return cameras_.size();
}

image_t MemoryDatabase::WriteImage(const Image& image,
                                   const bool use_image_id) {
  images_.push_back(image);
  // TODO: Think about it.
  if (use_image_id) {
    images_.back().SetImageId(image.ImageId());
  }
  else {
    images_.back().SetImageId(images_.size());
  }

  return images_.size();
}

void MemoryDatabase::WriteKeypoints(const image_t image_id,
                                    const FeatureKeypoints& keypoints) {
  keypoints_[image_id - 1] = keypoints;
}

void MemoryDatabase::WriteDescriptors(const image_t image_id,
                                      const FeatureDescriptors& descriptors) {
  descriptors_[image_id - 1] = descriptors;
  onLoad(images_.size());
}

void MemoryDatabase::WriteMatches(const image_t image_id1, const image_t image_id2,
                                  const FeatureMatches& matches) {
  image_pair_t pair_id = MemoryDatabase::ImagePairToPairId(image_id1, image_id2);
  if (matches.size() > 0) {
    matches_[pair_id] = matches;
  }
}

void MemoryDatabase::WriteTwoViewGeometry(
    const image_t image_id1, const image_t image_id2,
    const TwoViewGeometry& two_view_geometry) {
  image_pair_t pair_id = MemoryDatabase::ImagePairToPairId(image_id1, image_id2);
  if (two_view_geometry.inlier_matches.size()>0) {
    two_view_geometries_.emplace(pair_id, two_view_geometry);
  }
}

void MemoryDatabase::ClearMatches() {
  matches_.clear();
}

void MemoryDatabase::ClearTwoViewGeometries() {
  two_view_geometries_.clear();
}

void MemoryDatabase::ClearCameras() {
  cameras_.clear();
}

void MemoryDatabase::ClearImages() {
  images_.clear();
}

void MemoryDatabase::ClearDescriptors() {
  descriptors_.clear();
}

void MemoryDatabase::ClearKeypoints() {
  keypoints_.clear();
}

void MemoryDatabase::ClearAllTables() {
  ClearCameras();
  ClearDescriptors();
  ClearKeypoints();
  ClearImages();
  ClearMatches();
  ClearTwoViewGeometries();
}

void MemoryDatabase::UpdateCamera(const Camera& camera) {
  cameras_[camera.CameraId() - 1].SetModelId(camera.ModelId());
  cameras_[camera.CameraId() - 1].SetWidth(camera.Width());
  cameras_[camera.CameraId() - 1].SetHeight(camera.Height());
  cameras_[camera.CameraId() - 1].SetParams(camera.Params());
  cameras_[camera.CameraId() - 1].SetPriorFocalLength(camera.HasPriorFocalLength());
  cameras_[camera.CameraId() - 1].SetCameraId(camera.CameraId());
}

void MemoryDatabase::UpdateImage(const Image& image) {
  images_[image.ImageId() - 1].SetName(image.Name());
  images_[image.ImageId() - 1].SetCameraId(image.CameraId());
  images_[image.ImageId() - 1].SetQvecPrior(image.QvecPrior());
  images_[image.ImageId() - 1].SetTvecPrior(image.TvecPrior());
  images_[image.ImageId() - 1].SetImageId(image.ImageId());
}

void MemoryDatabase::DeleteMatches(const image_t image_id1,
                                   const image_t image_id2) {
  image_pair_t pair_id = MemoryDatabase::ImagePairToPairId(image_id1, image_id2);
  matches_.erase(pair_id);
}

void MemoryDatabase::DeleteInlierMatches(const image_t image_id1,
                                         const image_t image_id2) {
  image_pair_t pair_id = MemoryDatabase::ImagePairToPairId(image_id1, image_id2);
  two_view_geometries_.erase(pair_id);
}

void MemoryDatabase::BeginTransaction() const {}

void MemoryDatabase::EndTransaction() const {}

} // namespace colmap