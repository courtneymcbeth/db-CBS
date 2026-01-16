#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <limits>

#include <yaml-cpp/yaml.h>
#include <boost/program_options.hpp>

#include "dynobench/general_utils.hpp"
#include "dynoplan/dbrrt/dbrrt.hpp"

using namespace dynobench;
using namespace dynoplan;

namespace po = boost::program_options;

int main(int argc, char *argv[]) {

  Options_trajopt options_trajopt;
  Options_dbrrt options_dbrrt;
  po::options_description desc("Allowed options for DB-RRT planner");

  std::string cfg_file, results_file, env_file, models_base_path;

  // Add command line options
  set_from_boostop(desc, VAR_WITH_NAME(cfg_file));
  set_from_boostop(desc, VAR_WITH_NAME(results_file));
  set_from_boostop(desc, VAR_WITH_NAME(env_file));
  set_from_boostop(desc, VAR_WITH_NAME(models_base_path));
  options_trajopt.add_options(desc);
  options_dbrrt.add_options(desc);

  // Parse command line arguments
  try {
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    if (vm.count("help") != 0u) {
      std::cout << "DB-RRT Motion Planning Algorithm\n\n" << desc << "\n";
      return 0;
    }
  } catch (po::error &e) {
    std::cerr << "Error: " << e.what() << std::endl << std::endl;
    std::cerr << desc << std::endl;
    return 1;
  }

  // Load configuration from YAML file if provided
  if (cfg_file != "") {
    std::cout << "Loading configuration from: " << cfg_file << std::endl;
    options_dbrrt.read_from_yaml(cfg_file.c_str());
    options_trajopt.read_from_yaml(cfg_file.c_str());
  }

  // Load problem from environment file
  if (env_file.empty()) {
    std::cerr << "Error: env_file must be specified!" << std::endl;
    return 1;
  }

  std::cout << "Loading problem from: " << env_file << std::endl;
  Problem problem(env_file.c_str());
  problem.models_base_path = models_base_path;

  // Initialize trajectory and output info
  Trajectory traj;
  Info_out out_db;

  // Print DB-RRT options
  std::cout << "\n*** DB-RRT Options ***" << std::endl;
  std::cout << "Note: Converting time limit to milliseconds (1000x)" << std::endl;
  options_dbrrt.timelimit *= 1000; // Convert to milliseconds
  options_dbrrt.print(std::cout);
  std::cout << "***\n" << std::endl;

  // Create robot model
  std::cout << "Creating robot model: " << problem.robotType << std::endl;
  std::shared_ptr<dynobench::Model_robot> robot = dynobench::robot_factory(
      (problem.models_base_path + problem.robotType + ".yaml").c_str(),
      problem.p_lb, problem.p_ub);

  // Load environment into robot
  load_env(*robot, problem);

  // Load motion primitives
  std::vector<Motion> motions;
  if (!options_dbrrt.motionsFile.empty()) {
    std::cout << "Loading motion primitives from: " << options_dbrrt.motionsFile << std::endl;
    load_motion_primitives_new(
        options_dbrrt.motionsFile, *robot, motions, options_dbrrt.max_motions,
        options_dbrrt.cut_actions, false, options_dbrrt.check_cols);
    options_dbrrt.motions_ptr = &motions;
    std::cout << "Loaded " << motions.size() << " motion primitives" << std::endl;
  } else {
    std::cout << "Warning: No motion primitives file specified!" << std::endl;
  }

  // Run DB-RRT planner
  std::cout << "\n=== Running DB-RRT Planner ===" << std::endl;
  auto start_time = std::chrono::high_resolution_clock::now();

  idbrrt(problem, robot, options_dbrrt, options_trajopt, traj, out_db);

  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

  std::cout << "=== Planning Complete ===" << std::endl;
  std::cout << "Total time: " << duration.count() << " ms" << std::endl;

  // Print output information
  std::cout << "\n*** Planning Results ***" << std::endl;
  out_db.to_yaml(std::cout);
  std::cout << "***\n" << std::endl;

  // Write results to file if specified
  if (!results_file.empty()) {
    std::cout << "Writing results to: " << results_file << std::endl;
    std::ofstream results(results_file);
    results << "alg: db-rrt" << std::endl;
    results << "time_stamp: " << get_time_stamp() << std::endl;
    results << "env_file: " << env_file << std::endl;
    results << "cfg_file: " << cfg_file << std::endl;
    results << "results_file: " << results_file << std::endl;
    results << "total_planning_time_ms: " << duration.count() << std::endl;
    results << "\noptions_dbrrt:" << std::endl;
    options_dbrrt.print(results, "  ");
    results << "\noptions_trajopt:" << std::endl;
    options_trajopt.print(results, "  ");
    results << "\nresults:" << std::endl;
    out_db.to_yaml(results);
    results.close();
  }

  // Return success or failure based on whether a solution was found
  if (out_db.solved) {
    std::cout << "\n✓ Solution found!" << std::endl;
    return EXIT_SUCCESS;
  } else {
    std::cout << "\n✗ No solution found." << std::endl;
    return EXIT_FAILURE;
  }
}
