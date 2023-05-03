#ifndef SERIAL_RECONSTRUCTION_H_
#define SERIAL_RECONSTRUCTION_H_

#include <memory>
#include <string>
#include <unordered_map>

#include "base/image_reader.h"
#include "base/reconstruction_manager.h"
#include "controllers/incremental_mapper.h"
#include "feature/extraction.h"
#include "util/option_manager.h"
#include "util/threading.h"

namespace colmap {

class SerialReconstructionController : public Thread {
 public:
    using image_id_range_t=std::pair<image_t,image_t>;
 public:
  SerialReconstructionController(const OptionManager& options,
                                 ReconstructionManager* reconstruction_manager,
                                 size_t max_buffer_size = 20);

  void Stop(bool isReconstruct=false);
  
  void Run() override;
  void RunFeatureExtraction();
  void RunFeatureMatching();
  void RunIncrementalMapper();
  void SetExcludeRange(const image_id_range_t& range);
  void SetExcludeRangeFromIdToEnd(image_t idfrom);
  void SetExcludeRangeFromBeginToId(image_t idto);
  void AddImageData(internal::ImageData image_data);
  
  const std::unordered_map<camera_t, camera_t>& getImageCorrespondences() const;
  const std::unordered_map<image_t, image_t>& getCameraCorrespondences() const;
    boost::signals2::signal<bool()> OnFeatureExtractionStart,   OnFeatureExtractionStop;
    boost::signals2::signal<bool()> OnFeatureMatchingStart,     OnFeatureMatchingStop;
    boost::signals2::signal<bool()> OnRecounstructionStart;
    boost::signals2::signal<void(size_t,size_t,size_t)> FeatureExtractorStateChanged;
 private:
  void onLoad(image_t id);
  void CheckAgainstExclude();
  OptionManager option_manager_;
  ReconstructionManager* reconstruction_manager_;

  std::shared_ptr<IDatabase> database_;
  ImageReaderOptions reader_options_;

  std::unique_ptr<JobQueue<image_t>> matching_queue_;
  std::unique_ptr<JobQueue<internal::ImageData>> reader_queue_;

  std::unique_ptr<Thread> feature_extractor_;
  std::unique_ptr<Thread> sequential_matcher_;
  std::unique_ptr<Thread> incremental_mapper_;

  std::map<image_t,int> matching_overlap_;
  std::mutex overlap_mutex_;

  std::unordered_map<camera_t, camera_t> cameras_ids_correspondence_;
  std::unordered_map<image_t, image_t> images_ids_correspondence_;
  size_t _max_buffer_size;
  std::vector<image_id_range_t> _rangesToExclude;
  std::mutex _rangesToExcludeMutex;
};

}  // namespace colmap

#endif  // SERIAL_RECONSTRUCTION_H_
