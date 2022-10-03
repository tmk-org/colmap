#include "test_reconstruction.h"

#include "feature/extraction.h"
#include "feature/matching.h"
#include "util/misc.h"
#include "util/option_manager.h"

namespace colmap {

TestReconstructionController::TestReconstructionController(
    const OptionManager& options, ReconstructionManager* reconstruction_manager)
    : option_manager_(options),
      reconstruction_manager_(reconstruction_manager),
      reader_options_(*options.image_reader) {
  CHECK_NOTNULL(reconstruction_manager_);

  reader_options_.database_path = *option_manager_.database_path;
  reader_options_.image_path = *option_manager_.image_path;
  if (!option_manager_.image_reader->mask_path.empty()) {
    reader_options_.mask_path = option_manager_.image_reader->mask_path;
  }

  feature_extractor_.reset(new SiftFeatureExtractor(
      reader_options_, *option_manager_.sift_extraction));

  sequential_matcher_.reset(new SequentialFeatureMatcher(
      *option_manager_.sequential_matching, *option_manager_.sift_matching,
      *option_manager_.database_path));

  incremental_mapper_.reset(new IncrementalMapperController(
      option_manager_.mapper.get(), *option_manager_.image_path, *option_manager_.database_path, reconstruction_manager_));
}

void TestReconstructionController::Stop() { Thread::Stop(); }

void TestReconstructionController::Run() {
  if (IsStopped()) {
    return;
  }

  RunFeatureExtraction();

  if (IsStopped()) {
    return;
  }

  RunFeatureMatching();

  if (IsStopped()) {
    return;
  }

  RunIncrementalMapper();
}

void TestReconstructionController::RunFeatureExtraction() {
  CHECK(feature_extractor_);
  feature_extractor_->Start();
  feature_extractor_->Wait();
  feature_extractor_.reset();
}

void TestReconstructionController::RunFeatureMatching() {
  CHECK(sequential_matcher_);
  sequential_matcher_->Start();
  sequential_matcher_->Wait();
  sequential_matcher_.reset();
}

void TestReconstructionController::RunIncrementalMapper() {
  CHECK(incremental_mapper_);
  incremental_mapper_->Start();
  incremental_mapper_->Wait();
  incremental_mapper_.reset();

  const auto sparse_path = JoinPaths(*option_manager_.project_path, "sparse");
  CreateDirIfNotExists(sparse_path);
  reconstruction_manager_->Write(sparse_path, &option_manager_);
}

}  // namespace colmap
