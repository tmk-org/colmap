#ifndef TEST_RECONSTRUCTION_H_
#define TEST_RECONSTRUCTION_H_

#include <memory>
#include <string>

#include "base/image_reader.h"
#include "base/reconstruction_manager.h"
#include "feature/extraction.h"
#include "util/option_manager.h"
#include "util/threading.h"

namespace colmap {

class TestReconstructionController : public Thread {
 public:
  TestReconstructionController(const OptionManager& options,
                               ReconstructionManager* reconstruction_manager);

  void Stop() override;

  void Run() override;
  void RunFeatureExtraction();
  void RunFeatureMatching();

 private:
  OptionManager option_manager_;
  ReconstructionManager* reconstruction_manager_;

  ImageReaderOptions reader_options_;

  std::unique_ptr<Thread> feature_extractor_;
  std::unique_ptr<Thread> sequential_matcher_;
};

}  // namespace colmap

#endif  // SERIAL_RECONSTRUCTION_H_
