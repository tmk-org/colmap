#include <iostream>

#include "base/reconstruction_manager.h"
#include "controllers/serial_reconstruction.h"
#include "util/logging.h"
#include "util/option_manager.h"

using namespace colmap;

// Simple example that reads and writes a reconstruction.
int main(int argc, char** argv) {
  InitializeGlog(argv);

  std::string input_path =
      "/home/prestigh/Documents/TMK/OpravkaMod";
  ;
  std::string output_path =
      "/home/prestigh/Documents/TMK/OpravkaModTest";

  OptionManager options;

  options.AddAllOptions();

  *options.image_path = input_path;
  *options.database_path = output_path + "/database.db";

  options.ModifyForVideoData();
  options.ModifyForLowQuality();

  ReconstructionManager reconstruction;

  SerialReconstructionController controller(options, &reconstruction);
  controller.Run();

  for (size_t i = 65; i < 71; ++i) {
    internal::ImageData image_data;
    image_data.bitmap.Read(
        input_path + "/00000000" + std::to_string(i) + ".png", false);

    image_data.camera.SetWidth(static_cast<size_t>(image_data.bitmap.Width()));
    image_data.camera.SetHeight(
        static_cast<size_t>(image_data.bitmap.Height()));
    image_data.camera.SetModelIdFromName("SIMPLE_RADIAL");

    image_data.image.SetName("00000000" + std::to_string(i) + ".png");

    image_data.status = ImageReader::Status::SUCCESS;
    controller.AddImageData(image_data);
  }

  std::this_thread::sleep_for(std::chrono::seconds(20));

  std::cout << "Stopping controller ..." << std::endl;

  //controller.RunIncrementalMapper();
  controller.Stop();

  return EXIT_SUCCESS;
}