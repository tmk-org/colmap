#include <iostream>

#include "base/reconstruction_manager.h"
#include "controllers/serial_reconstruction.h"
#include "util/logging.h"
#include "util/option_manager.h"
#include "util/misc.h"

using namespace colmap;

// Simple example that reads and writes a reconstruction.
int main(int argc, char** argv) {
  InitializeGlog(argv);

  std::string input_path;
  std::string output_path;

  OptionManager options;
  options.AddRequiredOption("input_path", &input_path);
  options.AddRequiredOption("output_path", &output_path);

  options.Parse(argc, argv);
  options.AddAllOptions();

  *options.image_path = input_path;
  *options.database_path = JoinPaths(output_path, "/database.db");

  options.ModifyForVideoData();
  options.ModifyForLowQuality();

  ReconstructionManager reconstruction;

  SerialReconstructionController controller(options, &reconstruction);
  controller.Run();

  for (const auto& file : boost::filesystem::directory_iterator(input_path)) {
    internal::ImageData image_data;
    if (!image_data.bitmap.Read(file.path().string(), false)) {
      continue;
    }

    image_data.camera.SetWidth(static_cast<size_t>(image_data.bitmap.Width()));
    image_data.camera.SetHeight(
        static_cast<size_t>(image_data.bitmap.Height()));
    image_data.camera.SetModelIdFromName("SIMPLE_RADIAL");

    image_data.image.SetName(file.path().filename().string());

    image_data.status = ImageReader::Status::SUCCESS;
    controller.AddImageData(image_data);
  }

  std::this_thread::sleep_for(std::chrono::seconds(20));

  std::cout << "Stopping controller ..." << std::endl;

  controller.Stop();

  return EXIT_SUCCESS;
}