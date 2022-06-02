// Copyright (c) 2018, ETH Zurich and UNC Chapel Hill.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//
//     * Neither the name of ETH Zurich and UNC Chapel Hill nor the names of
//       its contributors may be used to endorse or promote products derived
//       from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Author: Johannes L. Schoenberger (jsch-at-demuc-dot-de)

#ifndef COLMAP_SRC_FEATURE_EXTRACTION_H_
#define COLMAP_SRC_FEATURE_EXTRACTION_H_

#include "base/database.h"
#include "base/database_sqlite.h"
#include "base/database_memory.h"
#include "base/image_reader.h"
#include "feature/sift.h"
#include "util/opengl_utils.h"
#include "util/threading.h"
#include <boost/signals2.hpp>
namespace colmap {

namespace internal {

struct ImageData;

}  // namespace internal

// Feature extraction class to extract features for all images in a directory.
class ISiftFeatureExtractor : public Thread {
 public:
  ISiftFeatureExtractor(const SiftExtractionOptions& sift_options,
                        std::shared_ptr<IDatabase> database)
      : sift_options_(sift_options), database_(database){};
  virtual ~ISiftFeatureExtractor() = default;

 protected:
  virtual void Run() = 0;

  const SiftExtractionOptions sift_options_;
  std::shared_ptr<IDatabase> database_;

  std::vector<std::unique_ptr<Thread>> resizers_;
  std::vector<std::unique_ptr<Thread>> extractors_;
  std::unique_ptr<Thread> writer_;

  std::unique_ptr<JobQueue<internal::ImageData>> resizer_queue_;
  std::unique_ptr<JobQueue<internal::ImageData>> extractor_queue_;
  std::unique_ptr<JobQueue<internal::ImageData>> writer_queue_;
};

class SiftFeatureExtractor : public ISiftFeatureExtractor {
 public:
  SiftFeatureExtractor(const ImageReaderOptions& reader_options,
                       const SiftExtractionOptions& sift_options);

 private:
  void Run();

  const ImageReaderOptions reader_options_;

  ImageReader image_reader_;
};

class SerialSiftFeatureExtractor : public ISiftFeatureExtractor {
 public:
  SerialSiftFeatureExtractor(const SiftExtractionOptions& sift_options,
                             std::shared_ptr<IDatabase> database,
                             JobQueue<internal::ImageData>* reader_queue);
  const boost::signals2::connection& connectStateHandler(const boost::signals2::signal<void(size_t,size_t,size_t)>::slot_type& invokable);
 private:
  void Run();

  image_t last_image_id_;

  JobQueue<internal::ImageData>* reader_queue_;
  boost::signals2::signal<void(size_t,size_t,size_t)> _runStateHandler;
  std::list< boost::signals2::scoped_connection > _runStateHandlerConnections;
};

// Import features from text files. Each image must have a corresponding text
// file with the same name and an additional ".txt" suffix.
class FeatureImporter : public Thread {
 public:
  FeatureImporter(const ImageReaderOptions& reader_options,
                  const std::string& import_path);

 private:
  void Run();

  const ImageReaderOptions reader_options_;
  const std::string import_path_;
};

////////////////////////////////////////////////////////////////////////////////
// Implementation
////////////////////////////////////////////////////////////////////////////////

namespace internal {

class ImageData {
public:
  ImageReader::Status status = ImageReader::Status::FAILURE;
  ImageData();
  ImageData(const ImageData&);
  ImageData(ImageData&&);
  ImageData& operator=(const ImageData&);
  ImageData& operator=(ImageData&&);
  
  ~ImageData();
  Camera camera;
  Image  image;
  Bitmap bitmap;
  Bitmap mask;

  FeatureKeypoints keypoints;
  FeatureDescriptors descriptors;
};

class ImageResizerThread : public Thread {
 public:
  ImageResizerThread(const int max_image_size, JobQueue<ImageData>* input_queue,
                     JobQueue<ImageData>* output_queue);

 private:
  void Run();

  const int max_image_size_;

  JobQueue<ImageData>* input_queue_;
  JobQueue<ImageData>* output_queue_;
};

class SiftFeatureExtractorThread : public Thread {
 public:
  SiftFeatureExtractorThread(const SiftExtractionOptions& sift_options,
                             const std::shared_ptr<Bitmap>& camera_mask,
                             JobQueue<ImageData>* input_queue,
                             JobQueue<ImageData>* output_queue);

 private:
  void Run();

  const SiftExtractionOptions sift_options_;
  std::shared_ptr<Bitmap> camera_mask_;

  std::unique_ptr<OpenGLContextManager> opengl_context_;

  JobQueue<ImageData>* input_queue_;
  JobQueue<ImageData>* output_queue_;
};

class FeatureWriterThread : public Thread {
 public:
  FeatureWriterThread(const size_t num_images, IDatabase* database,
                      JobQueue<ImageData>* input_queue);

 private:
  void Run();

  const size_t num_images_;
  IDatabase* database_;
  JobQueue<ImageData>* input_queue_;
};

}  // namespace internal

}  // namespace colmap

#endif  // COLMAP_SRC_FEATURE_EXTRACTION_H_
