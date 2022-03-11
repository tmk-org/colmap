#ifndef CONTROLLER_MOD_H_
#define CONTROLLER_MOD_H_

#include <memory>
#include <string>

#include "base/image_reader.h"
#include "base/reconstruction_manager.h"
#include "feature/extraction.h"
#include "util/option_manager.h"
#include "util/threading.h"

namespace colmap {

class ControllerMod : public Thread {
 public:
  ControllerMod(const OptionManager& options,
                ReconstructionManager* reconstruction_manager,
                size_t max_buffer_size = 20);

  void Stop() override;

  void Run() override;
  void RunFeatureExtraction();
  void RunFeatureMatching();

  void AddImageData(internal::ImageData image_data);

 private:
  void onLoad(image_t id);

  OptionManager option_manager_;
  ReconstructionManager* reconstruction_manager_;

  IDatabase* database_;
  ImageReaderOptions reader_options_;

  std::unique_ptr<JobQueue<image_t>> matching_queue_;
  std::unique_ptr<JobQueue<internal::ImageData>> reader_queue_;

  std::unique_ptr<Thread> feature_extractor_;
  std::unique_ptr<Thread> sequential_matcher_;
};

}  // namespace colmap

#endif  // CONTROLLER_MOD_H_
