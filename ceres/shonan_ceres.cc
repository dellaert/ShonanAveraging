/**
 * @file SOn.h
 * @brief CLI for running Shonan Averaging with Ceres back-end
 * @author Frank Dellaert
 * @date August 2020
 */

#include "SOn_parameterization.h"
#include "parameters.h"
#include "shonan_factor.h"

#include "ceres/ceres.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "read_g2o.h"
#include "types.h"

#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <vector>

DEFINE_string(input, "", "The pose graph definition filename in g2o format.");

using ceres::examples::Constraint3d;
using ceres::examples::MapOfPoses;
using ceres::examples::Pose3d;
using ceres::examples::VectorOfConstraints;
using std::cout;
using std::endl;

static ceres::Parameters<int> kParameters;

namespace shonan {

// Constructs the nonlinear least squares optimization problem from the pose
// graph constraints.
void BuildOptimizationProblem(const VectorOfConstraints &constraints,
                              const MapOfPoses &poses,
                              ceres::Problem *problem) {
  CHECK(problem != NULL);
  if (constraints.empty()) {
    LOG(INFO) << "No constraints, no problem to optimize.";
    return;
  }

  size_t p = 3;
  auto *SOn_local_parameterization = new SOnParameterization(p);

  for (const Constraint3d &constraint : constraints) {
    // Create factor. Ceres will take ownership of the pointer.
    Eigen::Quaterniond q = constraint.t_be.q;
    auto R12 = q.toRotationMatrix();
    auto factor = new ShonanFactor<3>(SOn(R12), p);

    // Add to problem aka factor graph
    int id1 = constraint.id_begin, id2 = constraint.id_end;
    std::vector<double *> unsafe = kParameters.Unsafe({id1, id2});
    problem->AddResidualBlock(factor, nullptr, unsafe[0], unsafe[1]);
    for (auto block : unsafe) {
      problem->SetParameterization(block, SOn_local_parameterization);
    }
  }
}

// Returns true if the solve was successful.
bool SolveOptimizationProblem(ceres::Problem *problem) {
  CHECK(problem != NULL);

  ceres::Solver::Options options;
  options.max_num_iterations = 200;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;

  ceres::Solver::Summary summary;
  ceres::Solve(options, problem, &summary);

  std::cout << summary.FullReport() << '\n';

  return summary.IsSolutionUsable();
}

// Output the poses to the file with format: id x y z q_x q_y q_z q_w.
bool OutputRotations(const std::string &filename, const MapOfPoses &poses) {
  std::fstream outfile;
  outfile.open(filename.c_str(), std::istream::out);
  if (!outfile) {
    LOG(ERROR) << "Error opening the file: " << filename;
    return false;
  }
  for (const auto &pair : poses) {
    outfile << pair.first << ":\n" << kParameters.At<SOn>(pair.first) << '\n';
  }
  return true;
}

} // namespace shonan

int main(int argc, char **argv) {
  using shonan::SOn;
  
  google::InitGoogleLogging(argv[0]);
  GFLAGS_NAMESPACE::ParseCommandLineFlags(&argc, &argv, true);

  CHECK(FLAGS_input != "") << "Need to specify the filename to read.";

  MapOfPoses poses;
  VectorOfConstraints constraints;

  CHECK(ReadG2oFile(FLAGS_input, &poses, &constraints))
      << "Error reading the file: " << FLAGS_input;

  std::cout << "Number of poses: " << poses.size() << '\n';
  std::cout << "Number of constraints: " << constraints.size() << '\n';

  // Add parameters
  for (const auto &pair : poses) {
    auto R = pair.second.q.toRotationMatrix();
    cout << pair.first << ":" << endl;
    cout << SOn(R) << endl << endl;
    kParameters.Insert(pair.first, SOn(R));
  }

  // Print parameters
  for (const auto &pair : poses) {
    cout << kParameters.Unsafe(pair.first) << ":" << endl;
    cout << kParameters.At<SOn>(pair.first) << endl << endl;
  }

  CHECK(shonan::OutputRotations("poses_original.txt", poses))
      << "Error outputting to poses_original.txt";

  ceres::Problem problem;
  shonan::BuildOptimizationProblem(constraints, poses, &problem);

  // Print parameters
  for (const auto &pair : poses) {
    cout << kParameters.Unsafe(pair.first) << ":" << endl;
    cout << kParameters.At<SOn>(pair.first) << endl << endl;
  }

  CHECK(shonan::SolveOptimizationProblem(&problem))
      << "The solve was not successful, exiting.";

  CHECK(shonan::OutputRotations("poses_optimized.txt", poses))
      << "Error outputting to poses_original.txt";

  return 0;
}
