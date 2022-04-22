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

void SerialReconstructionController::Stop() {
  reader_queue_->Wait();
  reader_queue_->Stop();

  feature_extractor_->Wait();
  feature_extractor_.reset();
  reader_queue_->Clear();

  matching_queue_->Wait();
  for (size_t i = 0; i < matching_overlap_.size(); ++i) {
    if (matching_overlap_[i] > 0) {
      matching_queue_->Push(i+1);
    }
  }
  matching_queue_->Wait();
  matching_queue_->Stop();

  sequential_matcher_->Wait();
  sequential_matcher_.reset();
  matching_queue_->Clear();

  DatabaseTransaction database_transaction(database_.get());

  auto matches = database_->ReadAllMatches();
  image_t curr_id;
  for (auto match : matches) {
    image_t image_id1, image_id2;
    DatabaseRoot::PairIdToImagePair(match.first, &image_id1, &image_id2);
    if (curr_id != image_id1){
      std::cout << std::endl;
      curr_id = image_id1;
    }
    std::cout << "Match: <" << image_id1 << ", " << image_id2 << "> " << match.second.size() <<  std::endl; 
  }

  RunIncrementalMapper();

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

// 2 2 1 0 0 0
void SerialReconstructionController::onLoad(image_t id) {
  std::unique_lock<std::mutex> lock(overlap_mutex_);
  // size_t overlap =
  //     std::min((size_t)(id - 1 + option_manager_.sequential_matching->overlap),
  //              matching_overlap_.size());
  int overlap = option_manager_.sequential_matching->overlap-1;

  // Сheck if all images have been processed before
  if (matching_overlap_[id - 1] == overlap) {
    matching_overlap_[id - 1] = 0;
    matching_queue_->Push(id);
  }

  // Update number of unprocessed images
  for (int i = id-2; i > int(id)-2-overlap && i >= 0; --i) {
    if (++matching_overlap_[i] == overlap && database_->ExistsDescriptors(i + 1)) {
      matching_overlap_[i] = 0;
      matching_queue_->Push(i + 1);
    }
  }
}

void SerialReconstructionController::AddImageData(
    internal::ImageData image_data) {
  DatabaseTransaction database_transaction(database_.get());
  std::unique_lock<std::mutex> lock(overlap_mutex_);

  if (database_->ExistsCamera(1)) {
    image_data.image.SetCameraId(1);
  } else {
    image_data.image.SetCameraId(database_->WriteCamera(image_data.camera));
  }

  if (image_data.image.ImageId() == kInvalidImageId) {
    image_data.image.SetImageId(database_->WriteImage(image_data.image));
  }

  matching_overlap_.push_back(0);

  // if (matching_overlap_.size() > 0) {
  //   matching_overlap_.push_back(
  //       std::min(matching_overlap_.back() + 1,
  //                (image_t)option_manager_.sequential_matching->overlap - 1));
  // } else {
  //   matching_overlap_.push_back(1);
  // }

  reader_queue_->Push(image_data);
}

}  // namespace colmap
