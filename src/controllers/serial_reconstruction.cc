#include "serial_reconstruction.h"

#include "feature/extraction.h"
#include "feature/matching.h"
#include "util/misc.h"
#include "util/option_manager.h"

namespace colmap {

SerialReconstructionController::SerialReconstructionController(
    const OptionManager& options, ReconstructionManager* reconstruction_manager,
    size_t max_buffer_size)
    : option_manager_(options),
      reconstruction_manager_(reconstruction_manager),
      database_(std::make_shared<MemoryDatabase>()),
      reader_options_(*options.image_reader) {
  CHECK_NOTNULL(reconstruction_manager_);

  matching_queue_.reset(new JobQueue<image_t>(max_buffer_size));
  reader_queue_.reset(new JobQueue<internal::ImageData>(max_buffer_size));

  feature_extractor_.reset(new SerialSiftFeatureExtractor(
      *option_manager_.sift_extraction, database_, reader_queue_.get()));

  sequential_matcher_.reset(new SerialSequentialFeatureMatcher(
      *option_manager_.sequential_matching, *option_manager_.sift_matching,
      database_, matching_queue_.get()));

  incremental_mapper_.reset(new IncrementalMapperController(
      option_manager_.mapper.get(), "", database_, reconstruction_manager_));

  database_->onLoad.connect(boost::bind(&SerialReconstructionController::onLoad,
                                        this, boost::placeholders::_1));
}

void SerialReconstructionController::Stop(bool isReconstruct) {
  reader_queue_->Wait();
  reader_queue_->Stop();

  feature_extractor_->Wait();
  feature_extractor_.reset();
  reader_queue_->Clear();

  matching_queue_->Wait();
  for (size_t i = 0; i < matching_overlap_.size(); ++i) {
    if (matching_overlap_[i] > 0) {
      matching_queue_->Push(i + 1);
    }
  }
  matching_queue_->Wait();
  matching_queue_->Stop();

  sequential_matcher_->Wait();
  sequential_matcher_.reset();
  matching_queue_->Clear();

  if (isReconstruct) {
    RunIncrementalMapper();
  }

  Thread::Stop();
}

void SerialReconstructionController::Run() {
  if (IsStopped()) {
    return;
  }

  RunFeatureExtraction();

  if (IsStopped()) {
    return;
  }

  RunFeatureMatching();
}

void SerialReconstructionController::RunFeatureExtraction() {
  CHECK(feature_extractor_);
  feature_extractor_->Start();
}

void SerialReconstructionController::RunFeatureMatching() {
  CHECK(sequential_matcher_);
  sequential_matcher_->Start();
}

void SerialReconstructionController::RunIncrementalMapper() {
  CHECK(incremental_mapper_);
  incremental_mapper_->Start();
  incremental_mapper_->Wait();
  incremental_mapper_.reset();

  const auto sparse_path = JoinPaths(*option_manager_.project_path, "sparse");
  CreateDirIfNotExists(sparse_path);
  reconstruction_manager_->Write(sparse_path, &option_manager_);
}

void SerialReconstructionController::onLoad(image_t id) {
  std::unique_lock<std::mutex> lock(overlap_mutex_);
  int overlap = option_manager_.sequential_matching->overlap - 1;

  // Сheck if there are enough processed images
  if (matching_overlap_[id - 1] == overlap) {
    matching_overlap_[id - 1] = 0;
    matching_queue_->Push(id);
  }

  // Update number of processed images
  for (int i = id - 2; i > int(id) - 2 - overlap && i >= 0; --i) {
    if (++matching_overlap_[i] == overlap &&
        database_->ExistsDescriptors(i + 1)) {
      matching_overlap_[i] = 0;
      matching_queue_->Push(i + 1);
    }
  }
}

void SerialReconstructionController::AddImageData(
    internal::ImageData image_data) {
  DatabaseTransaction database_transaction(database_.get());
  std::unique_lock<std::mutex> lock(overlap_mutex_);

  if (cameras_ids_correspondence_.contains(image_data.camera.CameraId())) {
    image_data.image.SetCameraId(
        cameras_ids_correspondence_[image_data.camera.CameraId()]);
  } else {
    auto camera_id = database_->WriteCamera(image_data.camera);
    cameras_ids_correspondence_[image_data.camera.CameraId()] = camera_id;
    image_data.image.SetCameraId(camera_id);
  }

  auto orig_image_id = image_data.image.ImageId();
  image_data.image.SetImageId(database_->WriteImage(image_data.image));
  images_ids_correspondence_[image_data.image.ImageId()] = orig_image_id;

  matching_overlap_.push_back(0);

  reader_queue_->Push(image_data);
}

const std::unordered_map<image_t, image_t>&
SerialReconstructionController::getImageCorrespondences() const {
  return images_ids_correspondence_;
}

const std::unordered_map<camera_t, camera_t>&
SerialReconstructionController::getCameraCorrespondences() const {
  return cameras_ids_correspondence_;
}

}  // namespace colmap
