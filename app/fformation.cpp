// author: Viktor Richter vrichter@techfak.uni-bielefeld.de
// licence: MIT

#include "GCoptimization.h"
#include <boost/program_options.hpp>
#include <fformation/Evaluation.h>
#include <fformation/Features.h>
#include <fformation/GroundTruth.h>
#include <fformation/GroupDetectorFactory.h>
#include <fformation/Settings.h>
#include <iostream>

using fformation::GroupDetectorFactory;
using fformation::GroupDetector;
using fformation::Options;
using fformation::Option;
using fformation::Features;
using fformation::GroundTruth;
using fformation::Settings;
using fformation::Evaluation;
using fformation::Observation;
using fformation::Classification;
using fformation::IdGroup;
using fformation::Person;
using fformation::PersonId;
using fformation::Position2D;
using fformation::Timestamp;

class GraphCutsOptimization : public fformation::GroupDetector {
public:
  GraphCutsOptimization(const fformation::Options optinos);

  GroupDetectorFactory::ConstructorFunction static creator();

  virtual Classification detect(const Observation &observation) const final;

  Classification init(const Observation observation) const;

  Classification updateAssignment(const std::vector<Person> &persons,
                                  const std::vector<Position2D> &group_centers,
                                  const Timestamp &timestamp) const;
  std::vector<Position2D>
  updateGroupCenters(const Observation &observation,
                     const Classification &assignment,
                     const Person::Stride &stride) const;

private:
  double _mdl;
  bool _shuffle;
  Person::Stride _stride;
};

GroupDetectorFactory &factory =
    GroupDetectorFactory::getDefaultInstance().addDetector(
        "gco", GraphCutsOptimization::creator());

static std::string getClassificators(std::string prefix) {
  std::stringstream str;
  str << prefix;
  str << " ( ";
  for (auto cl : factory.listDetectors()) {
    str << cl << " | ";
  }
  str << ")";
  return str.str();
}

int main(const int argc, const char **args) {
  boost::program_options::variables_map program_options;
  boost::program_options::options_description desc("Allowed options");
  desc.add_options()("help,h", "produce help message");
  desc.add_options()(
      "classificator,c",
      boost::program_options::value<std::string>()->default_value("list"),
      getClassificators(
          "Which classificator should be used for evaluation. Possible: ")
          .c_str());
  desc.add_options()(
      "evaluation,e",
      boost::program_options::value<std::string>()->default_value(
          "threshold=0.6666"),
      "May be used to override evaluation settings and default settings "
      "from settings.json");
  desc.add_options()(
      "dataset,d", boost::program_options::value<std::string>()->required(),
      "The root path of the evaluation dataset. The path is expected "
      "to contain features.json, groundtruth.json and settings.json");
  try {
    boost::program_options::store(
        boost::program_options::parse_command_line(argc, args, desc),
        program_options);
    boost::program_options::notify(program_options);
  } catch (const std::exception &e) {
    std::cerr << "Error while parsing command line parameters:\n\t" << e.what()
              << "\n";
    std::cerr << desc << std::endl;
    return 1;
  }

  if (program_options.count("help")) {
    std::cout << desc << "\n";
    return 0;
  }

  std::string path = program_options["dataset"].as<std::string>();
  std::string features_path = path + "/features.json";
  std::string groundtruth_path = path + "/groundtruth.json";
  std::string settings_path = path + "/settings.json";
  Settings settings = Settings::readMatlabJson(settings_path);

  auto config = GroupDetectorFactory::parseConfig(
      program_options["classificator"].as<std::string>());
  config.second.insert(Option("stride", settings.stride()));
  config.second.insert(Option("mdl", settings.mdl()));

  GroupDetector::Ptr detector = factory.create(config.first, config.second);

  Features features = Features::readMatlabJson(features_path);
  GroundTruth groundtruth = GroundTruth::readMatlabJson(groundtruth_path);

  Evaluation evaluation(features, groundtruth, settings, *detector.get(),
                        Options::parseFromString(
                            program_options["evaluation"].as<std::string>()));
  evaluation.printMatlabOutput(std::cout, false);
}

size_t findSeed(const fformation::Options &options) {
  if (options.hasOption("seed")) {
    return options.getOption("seed").convertValue<size_t>();
  } else {
    std::random_device rd;
    std::uniform_int_distribution<int> dist;
    return dist(rd);
  }
}

GraphCutsOptimization::GraphCutsOptimization(const fformation::Options options)
    : GroupDetector(options),
      _mdl(options.getOption("mdl").convertValue<double>(
          fformation::validators::MinMax<double>(
              0., std::numeric_limits<double>::max()))),
      _stride(options.getOption("stride").convertValue<Person::Stride>(
          fformation::validators::MinMax<double>(
              0., std::numeric_limits<Person::Stride>::max()))),
      _shuffle(options.hasOption("shuffle")) {}

GroupDetectorFactory::ConstructorFunction GraphCutsOptimization::creator() {
  return [](const Options &options) {
    return GroupDetector::Ptr(new GraphCutsOptimization(options));
  };
}

Classification
GraphCutsOptimization::init(const Observation observation) const {
  fformation::NonGroupDetector init;
  return init.detect(observation);
}

std::vector<Position2D>
GraphCutsOptimization::updateGroupCenters(const Observation &obs,
                                          const Classification &cl,
                                          const Person::Stride &stride) const {
  std::vector<Position2D> group_centers;
  for (auto group : cl.createGroups(obs)) {
    group_centers.push_back(group.calculateCenter(stride));
  }
  return group_centers;
}

void calculateCosts(const Person &person, const size_t person_id,
                    const std::vector<Person> &other,
                    const std::vector<Position2D> group_centers,
                    const Person::Stride &stride,
                    GCoptimizationGeneralGraph &gco) {
  for (size_t label = 0; label < group_centers.size(); ++label) {
    double cost = person.calculateDistanceCosts(group_centers[label], stride);
    for (size_t site = 0; site < other.size(); ++site) {
      cost += person.calculateVisibilityCost(group_centers[label], other[site]);
    }
    if (cost >= GCO_MAX_ENERGYTERM) {
      cost = (double)GCO_MAX_ENERGYTERM;
    }
    gco.setDataCost(person_id, label, cost);
  }
}

void setNeighbors(const std::vector<Person> &persons, const size_t person_id,
                  GCoptimizationGeneralGraph &gco) {
  for (size_t other_person = person_id + 1; other_person < persons.size();
       ++other_person) {
    gco.setNeighbors(person_id, other_person,
                     (persons[person_id].pose().position() -
                      persons[other_person].pose().position())
                         .norm());
  }
}

Classification createClassification(GCoptimizationGeneralGraph &gco,
                                    const std::vector<Person> &persons,
                                    const Timestamp &timestamp) {
  std::map<GCoptimization::LabelID, std::set<PersonId>> assignment;
  for (size_t person = 0; person < persons.size(); ++person) {
    auto assigned = gco.whatLabel(person);
    assignment[assigned].insert(persons[person].id());
  }
  std::vector<IdGroup> groups;
  for (auto group : assignment) {
    groups.push_back(IdGroup(group.second));
  }
  return Classification(timestamp, groups);
}

Classification GraphCutsOptimization::updateAssignment(
    const std::vector<Person> &persons,
    const std::vector<Position2D> &group_centers,
    const Timestamp &timestamp) const {
  // calculate assignment costs for all persons to all group centers
  GCoptimizationGeneralGraph gco(persons.size(), group_centers.size());
  for (size_t site = 0; site < persons.size(); ++site) {
    calculateCosts(persons[site], site, persons, group_centers, _stride, gco);
    setNeighbors(persons, site, gco);
  }
  gco.setLabelCost(_mdl);
  gco.expansion(10); // optimize
  return createClassification(gco, persons, timestamp);
}

template <typename T>
std::vector<T> shuffle(bool noop, std::vector<T> &src,
                       std::mt19937 &generator) {
  std::shuffle(src.begin(), src.end(), generator);
  return src;
}

Classification
GraphCutsOptimization::detect(const Observation &observation) const {
  std::mt19937 rng(findSeed(options()));
  Classification classification = init(observation);
  if (observation.group().persons().size() < 2) {
    return classification;
  }
  double old_cost = std::numeric_limits<double>::max();
  double current_cost =
      classification.calculateCosts(observation, _stride, _mdl);
  std::vector<Person> persons;
  persons.reserve(observation.group().persons().size());
  for (auto person : observation.group().persons()) {
    persons.push_back(person.second);
  }
  shuffle(_shuffle, persons, rng);
  do {
    std::vector<Position2D> group_centers =
        updateGroupCenters(observation, classification, _stride);
    shuffle(_shuffle, group_centers, rng);
    classification =
        updateAssignment(persons, group_centers, observation.timestamp());
    old_cost = current_cost;
    current_cost = classification.calculateCosts(observation, _stride, _mdl);
  } while (current_cost < old_cost && classification.idGroups().size() > 1);
  return classification;
}
