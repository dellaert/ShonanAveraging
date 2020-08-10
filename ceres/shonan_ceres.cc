/**
 * @file SOn.h
 * @brief CLI for running Shonan Averaging with Ceres back-end
 * @author Frank Dellaert
 * @date August 2020
 */

#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "ceres/ceres.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "read_g2o.h"
#include "types.h"

DEFINE_string(input, "", "The pose graph definition filename in g2o format.");

using ceres::examples::Pose3d;
using ceres::examples::MapOfPoses;
using ceres::examples::Constraint3d;
using ceres::examples::VectorOfConstraints;

namespace shonan {

// Constructs the nonlinear least squares optimization problem from the pose
// graph constraints.
void BuildOptimizationProblem(const VectorOfConstraints &constraints,
                              MapOfPoses *poses, ceres::Problem *problem) {}

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
    outfile << pair.first << " " << pair.second.p.transpose() << " "
            << pair.second.q.x() << " " << pair.second.q.y() << " "
            << pair.second.q.z() << " " << pair.second.q.w() << '\n';
  }
  return true;
}

} // namespace shonan

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  GFLAGS_NAMESPACE::ParseCommandLineFlags(&argc, &argv, true);

  CHECK(FLAGS_input != "") << "Need to specify the filename to read.";

  MapOfPoses poses;
  VectorOfConstraints constraints;

  CHECK(ReadG2oFile(FLAGS_input, &poses, &constraints))
      << "Error reading the file: " << FLAGS_input;

  std::cout << "Number of poses: " << poses.size() << '\n';
  std::cout << "Number of constraints: " << constraints.size() << '\n';

  CHECK(shonan::OutputRotations("poses_original.txt", poses))
      << "Error outputting to poses_original.txt";

  ceres::Problem problem;
  shonan::BuildOptimizationProblem(constraints, &poses, &problem);

  CHECK(shonan::SolveOptimizationProblem(&problem))
      << "The solve was not successful, exiting.";

  CHECK(shonan::OutputRotations("poses_optimized.txt", poses))
      << "Error outputting to poses_original.txt";

  return 0;
}
