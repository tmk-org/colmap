#include "ControllerMod.h"

#include "base/undistortion.h"
#include "controllers/incremental_mapper.h"
#include "feature/extraction.h"
#include "feature/matching.h"
#include "mvs/fusion.h"
#include "mvs/meshing.h"
#include "mvs/patch_match.h"
#include "util/misc.h"
#include "util/option_manager.h"

namespace colmap {

ControllerMod::ControllerMod(const OptionManager& options,
                             ReconstructionManager* reconstruction_manager,
                             size_t max_buffer_size)
    : option_manager_(options),
      reconstruction_manager_(reconstruction_manager),
      database_(new MemoryDatabase()),
      reader_options_(*options.image_reader) {
  CHECK_NOTNULL(reconstruction_manager_);

  reader_options_.database_path = *option_manager_.database_path;
  reader_options_.image_path = *option_manager_.image_path;
  if (!option_manager_.image_reader->mask_path.empty()) {
    reader_options_.mask_path = option_manager_.image_reader->mask_path;
  }

  matching_queue_.reset(new JobQueue<image_t>(max_buffer_size));
  reader_queue_.reset(new JobQueue<internal::ImageData>(max_buffer_size));

  feature_extractor_.reset(new SiftFeatureExtractor(
      reader_options_, *option_manager_.sift_extraction, database_,
      reader_queue_.get()));

  sequential_matcher_.reset(new SequentialFeatureMatcher(
      *option_manager_.sequential_matching, *option_manager_.sift_matching,
      database_, matching_queue_.get()));

  database_->onLoad.connect(
      boost::bind(&ControllerMod::onLoad, this, boost::placeholders::_1));
}

void ControllerMod::Stop() { Thread::Stop(); }

void ControllerMod::Run() {
  if (IsStopped()) {
    return;
  }

  RunFeatureExtraction();

  if (IsStopped()) {
    return;
  }

  RunFeatureMatching();
}

void ControllerMod::RunFeatureExtraction() {
  CHECK(feature_extractor_);
  feature_extractor_->Start();
  // feature_extractor_->Wait();
  // feature_extractor_.reset();
}

void ControllerMod::RunFeatureMatching() {
  CHECK(sequential_matcher_);
  sequential_matcher_->Start();
  // sequential_matcher_->Wait();
  // sequential_matcher_.reset();
}

void ControllerMod::onLoad(image_t id) {
  std::cout << "Image " << id << " was added." << std::endl;
  matching_queue_->Push(id);
}

void ControllerMod::AddImageData(internal::ImageData image_data) {
  DatabaseTransaction database_transaction(database_);
  image_data.image.SetCameraId(database_->WriteCamera(image_data.camera));

  reader_queue_->Push(image_data);
}

}  // namespace colmap
