#ifndef CONTROLLER_MOD_H_
#define CONTROLLER_MOD_H_

#include <string>
#include <memory>

#include "base/reconstruction_manager.h"
#include "base/image_reader.h"
#include "util/option_manager.h"
#include "util/threading.h"

namespace colmap {

class ControllerMod : public Thread {
 public:
  ControllerMod(const OptionManager& options,
                ReconstructionManager* reconstruction_manager);

  void Stop() override;

  void Run() override;
  void RunFeatureExtraction();
  void RunFeatureMatching();

  void onLoad(image_t id);

 private:
  OptionManager option_manager_;
  ReconstructionManager* reconstruction_manager_;

  IDatabase* database_;
  ImageReaderOptions reader_options_;

  std::unique_ptr<JobQueue<image_t>> ids_queue_;
  std::unique_ptr<Thread> feature_extractor_;
  std::unique_ptr<Thread> sequential_matcher_;
};

}  // namespace colmap

#endif  // CONTROLLER_MOD_H_
