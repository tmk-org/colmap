#include <iostream>

#include "ControllerMod.h"

#include "base/reconstruction_manager.h"
#include "util/logging.h"
#include "util/option_manager.h"

using namespace colmap;

// Simple example that reads and writes a reconstruction.
int main(int argc, char** argv) {
  InitializeGlog(argv);

  std::string input_path = "/home/evgenii/Documents/data/23_06_2021_оправки_стерео/test/images";;
  std::string output_path = "/home/evgenii/Documents/data/23_06_2021_оправки_стерео/test";

  OptionManager options;

  options.AddAllOptions();

  *options.image_path = input_path;
  *options.database_path = output_path + "/database.db";

  options.ModifyForVideoData();
  options.ModifyForLowQuality();

  ReconstructionManager reconstruction;

  ControllerMod controller(options, &reconstruction);
  controller.Run();

  std::this_thread::sleep_for(std::chrono::seconds(100));

  std::cout << "Colmap" << std::endl;

  return EXIT_SUCCESS;
}