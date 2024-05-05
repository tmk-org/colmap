#include "serial_reconstruction.h"

#include "feature/extraction.h"
#include "feature/matching.h"
#include "util/misc.h"
#include "util/option_manager.h"
#include "util/timer.h"

namespace colmap {

bool operator==(const SerialReconstructionController::image_id_range_t& l,const SerialReconstructionController::image_id_range_t& r)
{
    if(&l == &r)
    {
        return true;
    }
    return l.first==r.first && l.second==r.second;
}

bool operator^(const SerialReconstructionController::image_id_range_t& l,const image_t& id)
{
    return id >= l.first && id<=l.second;
}

SerialReconstructionController::SerialReconstructionController(
    const OptionManager& options, ReconstructionManager* reconstruction_manager,
    size_t max_buffer_size)
    : option_manager_(options),
      reconstruction_manager_(reconstruction_manager),
      database_(std::make_shared<MemoryDatabase>()),
      reader_options_(*options.image_reader),
      _max_buffer_size(max_buffer_size) {
  CHECK_NOTNULL(reconstruction_manager_);

  matching_queue_.reset(new JobQueue<image_t>(max_buffer_size));
  reader_queue_.reset(new JobQueue<internal::ImageData>(max_buffer_size));
   auto* pSerialSiftFeatureExtractor=new SerialSiftFeatureExtractor(
                *option_manager_.sift_extraction, 
                database_, 
                reader_queue_.get());
  feature_extractor_.reset(pSerialSiftFeatureExtractor);
  feature_extractor_->AddCallback(  STARTED_CALLBACK,
                                    [this]()
                                    {
                                        if(!OnFeatureExtractionStart.empty())
                                        {
                                            OnFeatureExtractionStart();
                                        }
                                    });
  feature_extractor_->AddCallback(  FINISHED_CALLBACK,
                                    [this]()
                                    {
                                        if(!OnFeatureExtractionStop.empty())
                                        {
                                            OnFeatureExtractionStop();
                                        }
                                    });
  pSerialSiftFeatureExtractor->connectStateHandler(FeatureExtractorStateChanged);
  sequential_matcher_.reset(new SerialSequentialFeatureMatcher(
      *option_manager_.sequential_matching, *option_manager_.sift_matching,
      database_, matching_queue_.get()));
  sequential_matcher_->AddCallback(  STARTED_CALLBACK,
                                    [this]()
                                    {
                                        if(!OnFeatureMatchingStart.empty())
                                        {
                                            OnFeatureMatchingStart();
                                        }
                                    });
  sequential_matcher_->AddCallback(  FINISHED_CALLBACK,
                                    [this]()
                                    {
                                        if(!OnFeatureMatchingStop.empty())
                                        {
                                            OnFeatureMatchingStop();
                                        }
                                    });


  incremental_mapper_.reset(new IncrementalMapperController(
      option_manager_.mapper.get(), "", database_, reconstruction_manager_));
  incremental_mapper_->AddCallback(STARTED_CALLBACK,
                                    [this]()
                                    {
                                        if(!OnRecounstructionStart.empty())
                                        {
                                            OnRecounstructionStart();
                                        }
                                    });
  database_->Connect([this](auto arg){onLoad(arg);});
}

void SerialReconstructionController::Stop(bool isReconstruct) {
  reader_queue_->Wait();
  reader_queue_->Stop();
  LOG(INFO) << "after stopping reader_queue_.size() "<< reader_queue_->Size();
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
  LOG(INFO) << "after stopping matching_queue_.size() "<< matching_queue_->Size();
  sequential_matcher_->Wait();
  sequential_matcher_.reset();
  matching_queue_->Clear();

  if (isReconstruct) {
    RunIncrementalMapper();
  }
  LOG(INFO) << "RunIncrementalMapper() finished";
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
  CheckAgainstExclude();
  incremental_mapper_->Start();
  LOG(INFO)<< "incremental mapper started...";
  incremental_mapper_->Wait();
  LOG(INFO)<< "incremental mapper finished...";
  incremental_mapper_.reset();

  const auto sparse_path = JoinPaths(*option_manager_.project_path, "sparse");
  CreateDirIfNotExists(sparse_path);
  reconstruction_manager_->Write(sparse_path, &option_manager_);
}

void SerialReconstructionController::onLoad(image_t id) {
  std::unique_lock<std::mutex> lock(overlap_mutex_);
//  int overlap = option_manager_.sequential_matching->overlap - 1;
  int overlap = 30;

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
//  Timer timer;
    std::list<std::tuple<std::string,int,double> > times;
    {
        std::scoped_lock lock(this->_rangesToExcludeMutex);
        if(!    _rangesToExclude.empty()
            &&  std::find_if(   _rangesToExclude.begin(),
                                _rangesToExclude.end(),
                                [&](const auto& p_range)->bool{return p_range^image_data.image.ImageId();})!=_rangesToExclude.end())
        {
            LOG(INFO)<< "frame [" << image_data.image.ImageId() << " , " <<image_data.image.CameraId()<< " skipped"<<std::endl;
            return;
        }
    }
    DatabaseTransaction database_transaction(database_.get());
//    timer.Pause();
//    times.emplace_back(std::make_tuple(__FUNCTION__,__LINE__,timer.ElapsedMicroSeconds()));
//    timer.Resume();
    std::unique_lock<std::mutex> lock(overlap_mutex_);
//    timer.Pause();
//    times.emplace_back(std::make_tuple(__FUNCTION__,__LINE__,timer.ElapsedMicroSeconds()));
//    timer.Resume();
    if (cameras_ids_correspondence_.find(image_data.camera.CameraId() )!= cameras_ids_correspondence_.end() ) {
      image_data.image.SetCameraId(
          cameras_ids_correspondence_[image_data.camera.CameraId()]);
    } else {
      auto camera_id = database_->WriteCamera(image_data.camera);
      cameras_ids_correspondence_[image_data.camera.CameraId()] = camera_id;
      image_data.image.SetCameraId(camera_id);
    }
//    timer.Pause();
//    times.emplace_back(std::make_tuple(__FUNCTION__,__LINE__,timer.ElapsedMicroSeconds()));
//    timer.Resume();
    auto orig_image_id = image_data.image.ImageId();
    image_data.image.SetImageId(database_->WriteImage(image_data.image));
    images_ids_correspondence_[image_data.image.ImageId()] = orig_image_id;
//    timer.Pause();
//    times.emplace_back(std::make_tuple(__FUNCTION__,__LINE__,timer.ElapsedMicroSeconds()));
//    timer.Resume();
    matching_overlap_.push_back(0);
  //}
  if(reader_queue_->Size() == _max_buffer_size)
  {
    if(!FeatureExtractorStateChanged.empty())
    {
        FeatureExtractorStateChanged(_max_buffer_size,0,0);
    }
  }
  reader_queue_->Push(image_data);
//  timer.Pause();
//  times.emplace_back(std::make_tuple(__FUNCTION__,__LINE__,timer.ElapsedMicroSeconds()));
//  double prev_dur=0;
//  for(auto& tt : times)
//  {
//    auto&[fname,line,dur] = tt;
//    std::cout << fname <<" :"<<line<<" :"<<((dur-prev_dur)/1000.0)<<" ms"<<std::endl;
//    prev_dur = dur;
//  }
}

const std::unordered_map<image_t, image_t>&
SerialReconstructionController::getImageCorrespondences() const {
  return images_ids_correspondence_;
}

const std::unordered_map<camera_t, camera_t>&
SerialReconstructionController::getCameraCorrespondences() const {
  return cameras_ids_correspondence_;
}

void SerialReconstructionController::SetExcludeRange(const SerialReconstructionController::image_id_range_t& range)
{
    std::scoped_lock lock(_rangesToExcludeMutex);
    if( _rangesToExclude.empty() || 
        std::find(_rangesToExclude.begin(),_rangesToExclude.end(),
                    range)==_rangesToExclude.end())
    {
        _rangesToExclude.push_back(range);
    }
}

void SerialReconstructionController::SetExcludeRangeFromIdToEnd(image_t idfrom)
{
    SetExcludeRange(std::make_pair(idfrom,std::numeric_limits<image_t>::max()));
}

void SerialReconstructionController::SetExcludeRangeFromBeginToId(image_t idto)
{
    SetExcludeRange(std::make_pair(std::numeric_limits<image_t>::min(),idto));
}

void SerialReconstructionController::CheckAgainstExclude()
{
    decltype(_rangesToExclude) ranges;
    {
        std::scoped_lock lock(_rangesToExcludeMutex);
        if(_rangesToExclude.empty())
        {
            return;
        }
        ranges = _rangesToExclude;
    }
    DatabaseTransaction database_transaction(database_.get());
    int imageNum= (int)(database_->NumImages() & std::numeric_limits<int>::max());
    std::list<image_t> imagesToRemove;
    for(int i=0;i<imageNum;i++)
    {
        auto initial_frame_id = images_ids_correspondence_[i];
        if(std::find_if(ranges.begin(),ranges.end(),[&](const auto& r)->bool{return r^initial_frame_id;})!=ranges.end())
        {
            imagesToRemove.push_back(initial_frame_id);
        }
    }
    std::list<std::pair<image_t,image_t>> pairs;
    for(const auto& id:imagesToRemove)
    {
        for(auto i=0;i<imageNum;i++)
        {
            if ( database_->ExistsInlierMatches(id,i))
            {
                pairs.push_back(std::make_pair(id,i));
            }
        }
    }
    for(auto [id1,id2] : pairs)
    {
        database_->DeleteMatches(id1,id2);
        database_->DeleteInlierMatches(id1,id2);
        database_->WriteKeypoints(id1,colmap::FeatureKeypoints{});
    }
}
}  // namespace colmap
