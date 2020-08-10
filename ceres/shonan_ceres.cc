/**
 * @file SOn.h
 * @brief CLI for running Shonan Averaging with Ceres back-end
 * @author Frank Dellaert
 * @date August 2020
 */

#include "SOn_parameterization.h"
#include "frobenius_prior.h"
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

namespace shonan {

// Create rotation parameter blocks in parameters from read poses.
void InitializeRotationUnknowns(const ceres::examples::MapOfPoses &poses,
                                size_t p,
                                ceres::ParameterBuffer<SOn> *parameters) {
  for (const auto &pair : poses) {
    auto R = pair.second.q.toRotationMatrix();
    SOn initial = SOn::Lift(p, pair.second.q.toRotationMatrix());
    parameters->PushBack(initial);
  }
}

// Constructs the nonlinear least squares optimization problem from the pose
// graph constraints.
void BuildOptimizationProblem(
    const ceres::examples::VectorOfConstraints &constraints, size_t p,
    ceres::ParameterBuffer<SOn> *parameters, ceres::Problem *problem) {
  CHECK(problem != NULL);
  if (constraints.empty()) {
    LOG(INFO) << "No constraints, no problem to optimize.";
    return;
  }

  auto *SOn_local_parameterization = new SOnParameterization(p);

  // Add all pairwise factors
  for (const ceres::examples::Constraint3d &constraint : constraints) {
    // Create factor. Ceres will take ownership of the pointer.
    Eigen::Quaterniond q = constraint.t_be.q;
    auto R12 = q.toRotationMatrix();
    auto factor = new ShonanFactor<3>(SOn(R12), p);

    // Add to problem aka factor graph
    size_t id1 = constraint.id_begin, id2 = constraint.id_end;
    std::vector<double *> unsafe = parameters->Unsafe({id1, id2});
    // ceres::ResidualBlockId id =
    problem->AddResidualBlock(factor, nullptr, unsafe[0], unsafe[1]);
    for (auto block : unsafe) {
      problem->SetParameterization(block, SOn_local_parameterization);
    }
  }

  // Add prior
  double *unsafe = parameters->Unsafe(0);
  SOn mean = parameters->At(0);
  // SOn mean(p);
  auto factor = new FrobeniusPrior(mean);
  problem->AddResidualBlock(factor, nullptr, unsafe);
  problem->SetParameterization(unsafe, SOn_local_parameterization);
}

// Returns true if the solve was successful.
bool SolveOptimizationProblem(ceres::Problem *problem) {
  CHECK(problem != NULL);

  ceres::Solver::Options options;
  options.max_num_iterations = 200;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  options.minimizer_progress_to_stdout = true;

  ceres::Solver::Summary summary;
  ceres::Solve(options, problem, &summary);

  std::cout << summary.FullReport() << '\n';

  return summary.IsSolutionUsable();
}

// Output the poses to the file
bool OutputRotations(const std::string &filename,
                     const ceres::ParameterBuffer<SOn> &parameters) {
  std::fstream outfile;
  outfile.open(filename.c_str(), std::istream::out);
  if (!outfile) {
    LOG(ERROR) << "Error opening the file: " << filename;
    return false;
  }
  outfile << parameters << std::endl;
  return true;
}
} // namespace shonan

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  GFLAGS_NAMESPACE::ParseCommandLineFlags(&argc, &argv, true);

  CHECK(FLAGS_input != "") << "Need to specify the filename to read.";

  ceres::examples::MapOfPoses poses;
  ceres::examples::VectorOfConstraints constraints;
  CHECK(ReadG2oFile(FLAGS_input, &poses, &constraints))
      << "Error reading the file: " << FLAGS_input;
  std::cout << "Number of poses: " << poses.size() << '\n';
  std::cout << "Number of constraints: " << constraints.size() << '\n';

  size_t p = 4;
  ceres::ParameterBuffer<shonan::SOn> parameters;
  InitializeRotationUnknowns(poses, p, &parameters);

  ceres::Problem problem;
  shonan::BuildOptimizationProblem(constraints, p, &parameters, &problem);

  CHECK(shonan::OutputRotations("rotations_original.txt", parameters))
      << "Error outputting to rotations_original.txt";

  CHECK(shonan::SolveOptimizationProblem(&problem))
      << "The solve was not successful, exiting.";

  CHECK(shonan::OutputRotations("rotations_optimized.txt", parameters))
      << "Error outputting to rotations_original.txt";

  return 0;
}
