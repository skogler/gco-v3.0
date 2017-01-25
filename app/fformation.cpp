// author: Viktor Richter vrichter@techfak.uni-bielefeld.de
// licence: MIT

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <fformation/Settings.h>
#include <fformation/Features.h>
#include <fformation/GroundTruth.h>

using fformation::Settings;
using fformation::SettingsGC;
using fformation::Features;
using fformation::GroundTruth;

namespace pt = boost::property_tree;

void print_help(int exit_code) {
  std::cout << "Reads fformation-datasets as json.\n"
            << "  call: parser <filename>" << std::endl;
  exit(exit_code);
}

int main(const int argc, const char **args) {

  if (argc < 1) {
    std::cerr << "Not enough parameters passed." << std::endl;
    print_help(-1);
  }

  std::string path = args[1];
  std::string features_path = path + "/features.json";
  std::string groundtruth_path = path + "/groundtruth.json";
  std::string settings_path = path + "/settings.json";
  std::string settingsgc_path = path + "/settings_gc.json";

  Features features = Features::readMatlabJson(features_path);
  GroundTruth groundtruth = GroundTruth::readMatlabJson(groundtruth_path);
  Settings settings = Settings::readMatlabJson(settings_path);
  SettingsGC settingsgc = SettingsGC::readMatlabJson(settingsgc_path);

}
