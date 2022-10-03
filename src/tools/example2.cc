#include <filesystem>
#include <iostream>

#include "base/reconstruction_manager.h"
#include "controllers/serial_reconstruction.h"
#include "controllers/test_reconstruction.h"
#include "util/logging.h"
#include "util/misc.h"
#include "util/option_manager.h"
#include <chrono>
#include <thread>
using namespace colmap;

// Simple example that reads and writes a reconstruction.
int main(int argc, char** argv) {
    auto start = std::chrono::system_clock::now();
  InitializeGlog(argv);

  std::string input_path;
  std::string output_path;

  OptionManager options;
  options.AddRequiredOption("input_path", &input_path);
  options.AddRequiredOption("output_path", &output_path);

  options.Parse(argc, argv);
  options.AddAllOptions();
    //options.AddMatchingOptions();
  *options.image_path = input_path;
  *options.database_path = JoinPaths(output_path, "/database.db");
  *options.project_path = output_path;
  

  options.ModifyForVideoData();
  options.ModifyForHighQuality();

  // ReconstructionManager testReconstruction;

  // TestReconstructionController testController(options, &testReconstruction);
  // testController.Run();
    //*options.database_path
  ReconstructionManager reconstruction;
  SerialReconstructionController controller(options, &reconstruction);
  controller.Run();

  std::vector<std::filesystem::path> files_in_directory;
  std::copy(std::filesystem::directory_iterator(input_path),
            std::filesystem::directory_iterator(),
            std::back_inserter(files_in_directory));
  std::sort(files_in_directory.begin(), files_in_directory.end());

  for (const auto& file : files_in_directory) {
    internal::ImageData image_data;
    if (!image_data.bitmap.Read(file.string(), false)) {
      continue;
    }

    image_data.camera.SetModelIdFromName("SIMPLE_RADIAL");
    double focal_length =
        1.2 * std::max(image_data.bitmap.Width(), image_data.bitmap.Height());
    image_data.camera.InitializeWithId(image_data.camera.ModelId(),
                                       focal_length, image_data.bitmap.Width(),
                                       image_data.bitmap.Height());

    image_data.camera.SetWidth(static_cast<size_t>(image_data.bitmap.Width()));
    image_data.camera.SetHeight(
        static_cast<size_t>(image_data.bitmap.Height()));

    image_data.image.SetName(file.filename().string());

    image_data.status = ImageReader::Status::SUCCESS;
    controller.AddImageData(image_data);
  }

  std::this_thread::sleep_for(std::chrono::seconds(20));

  std::cout << "Stopping controller ..." << std::endl;

  controller.Stop();
  //double call tends to assertion failed inside RunIncrementalMapper()
  //controller.RunIncrementalMapper();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start);
    std::cout << "processing took " << duration.count() << "seconds" <<std::endl;
  return EXIT_SUCCESS;
}