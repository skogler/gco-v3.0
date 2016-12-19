// author: Viktor Richter vrichter@techfak.uni-bielefeld.de
// licance: MIT

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

namespace pt = boost::property_tree;

template <typename T> T createFromTree(const pt::ptree &tree);

struct Features {};

template <> Features createFromTree(const pt::ptree &tree) {
  std::vector<int> a;
  for (auto i : a) {
    i++;
  }
  for (auto node : tree) {
    std::cout << "Node: " << node.first.c_str() << " - " << node.second.size() << std::endl;
  }
}

struct GroundTruth {};

struct Settings {};

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
  std::string ground_truth_path = path + "/groundtruth.json";
  std::string settings_path = path + "/settings.json";

  pt::ptree features_data, ground_truth_data, settings_data;
  try{
    pt::read_json(features_path, features_data);
    pt::read_json(ground_truth_path, ground_truth_data);
    pt::read_json(settings_path, settings_data);
  } catch (const pt::json_parser_error& e){
    std::cerr << "Error while parsing data: " << e.what() << std::endl;
    exit(-2);
  }

  Features features = createFromTree<Features>(features_data);
  // GroundTruth ground_truth = createFromTree<GroundTruth>(ground_truth_data);
  // Settings settings        = createFromTree<Settings>(settings_data);
}
