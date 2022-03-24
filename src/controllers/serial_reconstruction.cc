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

  reader_options_.database_path = *option_manager_.database_path;
  reader_options_.image_path = *option_manager_.image_path;
  if (!option_manager_.image_reader->mask_path.empty()) {
    reader_options_.mask_path = option_manager_.image_reader->mask_path;
  }

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
  reader_queue_->Stop();
  reader_queue_->Clear();
  feature_extractor_->Stop();
  feature_extractor_->Wait();
  feature_extractor_.reset();

  matching_queue_->Stop();
  matching_queue_->Clear();
  sequential_matcher_->Stop();
  sequential_matcher_->Wait();
  sequential_matcher_.reset();

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
}

void SerialReconstructionController::onLoad(image_t id) {
  matching_queue_->Push(id);
}

void SerialReconstructionController::AddImageData(
    internal::ImageData image_data) {
  DatabaseTransaction database_transaction(database_.get());
  image_data.image.SetCameraId(database_->WriteCamera(image_data.camera));

  reader_queue_->Push(image_data);
}

}  // namespace colmap
