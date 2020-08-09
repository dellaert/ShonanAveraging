#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "ceres/ceres.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "read_g2o.h"

DEFINE_string(input, "", "The pose graph definition filename in g2o format.");

namespace ceres {
namespace examples {
namespace {

// The state for each vertex in the pose graph.
struct Pose2d {
  double x;
  double y;
  double yaw_radians;

  // The name of the data type in the g2o file format.
  static std::string name() { return "VERTEX_SE2"; }
};

// Normalizes the angle in radians between [-pi and pi).
template <typename T> inline T NormalizeAngle(const T &angle_radians) {
  // Use ceres::floor because it is specialized for double and Jet types.
  T two_pi(2.0 * M_PI);
  return angle_radians -
         two_pi * ceres::floor((angle_radians + T(M_PI)) / two_pi);
}

inline std::istream &operator>>(std::istream &input, Pose2d &pose) {
  input >> pose.x >> pose.y >> pose.yaw_radians;
  // Normalize the angle between -pi to pi.
  pose.yaw_radians = NormalizeAngle(pose.yaw_radians);
  return input;
}

// The constraint between two vertices in the pose graph. The constraint is the
// transformation from vertex id_begin to vertex id_end.
struct Constraint2d {
  int id_begin;
  int id_end;

  double x;
  double y;
  double yaw_radians;

  // The inverse of the covariance matrix for the measurement. The order of the
  // entries are x, y, and yaw.
  Eigen::Matrix3d information;

  // The name of the data type in the g2o file format.
  static std::string name() { return "EDGE_SE2"; }
};

inline std::istream &operator>>(std::istream &input, Constraint2d &constraint) {
  input >> constraint.id_begin >> constraint.id_end >> constraint.x >>
      constraint.y >> constraint.yaw_radians >> constraint.information(0, 0) >>
      constraint.information(0, 1) >> constraint.information(0, 2) >>
      constraint.information(1, 1) >> constraint.information(1, 2) >>
      constraint.information(2, 2);

  // Set the lower triangular part of the information matrix.
  constraint.information(1, 0) = constraint.information(0, 1);
  constraint.information(2, 0) = constraint.information(0, 2);
  constraint.information(2, 1) = constraint.information(1, 2);

  // Normalize the angle between -pi to pi.
  constraint.yaw_radians = NormalizeAngle(constraint.yaw_radians);
  return input;
}

// Reads a single pose from the input and inserts it into the map. Returns false
// if there is a duplicate entry.
template <typename Pose, typename Allocator>
bool ReadVertex(std::ifstream *infile,
                std::map<int, Pose, std::less<int>, Allocator> *poses) {
  int id;
  Pose pose;
  *infile >> id >> pose;

  // Ensure we don't have duplicate poses.
  if (poses->find(id) != poses->end()) {
    LOG(ERROR) << "Duplicate vertex with ID: " << id;
    return false;
  }
  (*poses)[id] = pose;

  return true;
}

// Reads the contraints between two vertices in the pose graph
template <typename Constraint, typename Allocator>
void ReadConstraint(std::ifstream *infile,
                    std::vector<Constraint, Allocator> *constraints) {
  Constraint constraint;
  *infile >> constraint;

  constraints->push_back(constraint);
}

template <typename T> Eigen::Matrix<T, 2, 2> RotationMatrix2D(T yaw_radians) {
  const T cos_yaw = ceres::cos(yaw_radians);
  const T sin_yaw = ceres::sin(yaw_radians);

  Eigen::Matrix<T, 2, 2> rotation;
  rotation << cos_yaw, -sin_yaw, sin_yaw, cos_yaw;
  return rotation;
}

// Computes the error term for two poses that have a relative pose measurement
// between them. Let the hat variables be the measurement.
//
// residual =  information^{1/2} * [  r_a^T * (p_b - p_a) - \hat{p_ab}   ]
//                                 [ Normalize(yaw_b - yaw_a - \hat{yaw_ab}) ]
//
// where r_a is the rotation matrix that rotates a vector represented in frame A
// into the global frame, and Normalize(*) ensures the angles are in the range
// [-pi, pi).
class PoseGraph2dErrorTerm {
public:
  PoseGraph2dErrorTerm(double x_ab, double y_ab, double yaw_ab_radians,
                       const Eigen::Matrix3d &sqrt_information)
      : p_ab_(x_ab, y_ab), yaw_ab_radians_(yaw_ab_radians),
        sqrt_information_(sqrt_information) {}

  template <typename T>
  bool operator()(const T *const x_a, const T *const y_a, const T *const yaw_a,
                  const T *const x_b, const T *const y_b, const T *const yaw_b,
                  T *residuals_ptr) const {
    const Eigen::Matrix<T, 2, 1> p_a(*x_a, *y_a);
    const Eigen::Matrix<T, 2, 1> p_b(*x_b, *y_b);

    Eigen::Map<Eigen::Matrix<T, 3, 1>> residuals_map(residuals_ptr);

    residuals_map.template head<2>() =
        RotationMatrix2D(*yaw_a).transpose() * (p_b - p_a) - p_ab_.cast<T>();
    residuals_map(2) = ceres::examples::NormalizeAngle(
        (*yaw_b - *yaw_a) - static_cast<T>(yaw_ab_radians_));

    // Scale the residuals by the square root information matrix to account for
    // the measurement uncertainty.
    residuals_map = sqrt_information_.template cast<T>() * residuals_map;

    return true;
  }

  static ceres::CostFunction *Create(double x_ab, double y_ab,
                                     double yaw_ab_radians,
                                     const Eigen::Matrix3d &sqrt_information) {
    return (new ceres::AutoDiffCostFunction<PoseGraph2dErrorTerm, 3, 1, 1, 1, 1,
                                            1, 1>(new PoseGraph2dErrorTerm(
        x_ab, y_ab, yaw_ab_radians, sqrt_information)));
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  // The position of B relative to A in the A frame.
  const Eigen::Vector2d p_ab_;
  // The orientation of frame B relative to frame A.
  const double yaw_ab_radians_;
  // The inverse square root of the measurement covariance matrix.
  const Eigen::Matrix3d sqrt_information_;
};

// Defines a local parameterization for updating the angle to be constrained in
// [-pi to pi).
class AngleLocalParameterization {
public:
  template <typename T>
  bool operator()(const T *theta_radians, const T *delta_theta_radians,
                  T *theta_radians_plus_delta) const {
    *theta_radians_plus_delta =
        NormalizeAngle(*theta_radians + *delta_theta_radians);

    return true;
  }

  static ceres::LocalParameterization *Create() {
    return (new ceres::AutoDiffLocalParameterization<AngleLocalParameterization,
                                                     1, 1>);
  }
};

// Constructs the nonlinear least squares optimization problem from the pose
// graph constraints.
void BuildOptimizationProblem(const std::vector<Constraint2d> &constraints,
                              std::map<int, Pose2d> *poses,
                              ceres::Problem *problem) {
  CHECK(poses != NULL);
  CHECK(problem != NULL);
  if (constraints.empty()) {
    LOG(INFO) << "No constraints, no problem to optimize.";
    return;
  }

  ceres::LossFunction *loss_function = NULL;
  ceres::LocalParameterization *angle_local_parameterization =
      AngleLocalParameterization::Create();

  for (const Constraint2d &constraint : constraints) {
    std::map<int, Pose2d>::iterator it1 = poses->find(constraint.id_begin);
    CHECK(it1 != poses->end())
        << "Pose with ID: " << constraint.id_begin << " not found.";
    std::map<int, Pose2d>::iterator it2 = poses->find(constraint.id_end);
    CHECK(it2 != poses->end())
        << "Pose with ID: " << constraint.id_end << " not found.";

    const Eigen::Matrix3d sqrt_information =
        constraint.information.llt().matrixL();
    // Ceres will take ownership of the pointer.
    ceres::CostFunction *cost_function = PoseGraph2dErrorTerm::Create(
        constraint.x, constraint.y, constraint.yaw_radians, sqrt_information);
    problem->AddResidualBlock(cost_function, loss_function, &it1->second.x,
                              &it1->second.y, &it1->second.yaw_radians,
                              &it2->second.x, &it2->second.y,
                              &it2->second.yaw_radians);

    problem->SetParameterization(&it1->second.yaw_radians,
                                 angle_local_parameterization);
    problem->SetParameterization(&it2->second.yaw_radians,
                                 angle_local_parameterization);
  }

  // The pose graph optimization problem has three DOFs that are not fully
  // constrained. This is typically referred to as gauge freedom. You can apply
  // a rigid body transformation to all the nodes and the optimization problem
  // will still have the exact same cost. The Levenberg-Marquardt algorithm has
  // internal damping which mitigate this issue, but it1 is better to properly
  // constrain the gauge freedom. This can be done by setting one of the poses
  // as constant so the optimizer cannot change it1.
  std::map<int, Pose2d>::iterator pose_start_iter = poses->begin();
  CHECK(pose_start_iter != poses->end()) << "There are no poses.";
  problem->SetParameterBlockConstant(&pose_start_iter->second.x);
  problem->SetParameterBlockConstant(&pose_start_iter->second.y);
  problem->SetParameterBlockConstant(&pose_start_iter->second.yaw_radians);
}

// Returns true if the solve was successful.
bool SolveOptimizationProblem(ceres::Problem *problem) {
  CHECK(problem != NULL);

  ceres::Solver::Options options;
  options.max_num_iterations = 100;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  options.minimizer_progress_to_stdout = true;

  ceres::Solver::Summary summary;
  ceres::Solve(options, problem, &summary);

  std::cout << summary.FullReport() << '\n';

  return summary.IsSolutionUsable();
}

// Output the poses to the file with format: ID x y yaw_radians.
bool OutputPoses(const std::string &filename,
                 const std::map<int, Pose2d> &poses) {
  std::fstream outfile;
  outfile.open(filename.c_str(), std::istream::out);
  if (!outfile) {
    std::cerr << "Error opening the file: " << filename << '\n';
    return false;
  }
  for (std::map<int, Pose2d>::const_iterator poses_iter = poses.begin();
       poses_iter != poses.end(); ++poses_iter) {
    const std::map<int, Pose2d>::value_type &pair = *poses_iter;
    outfile << pair.first << " " << pair.second.x << " " << pair.second.y << ' '
            << pair.second.yaw_radians << '\n';
  }
  return true;
}

} // namespace
} // namespace examples
} // namespace ceres

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  GFLAGS_NAMESPACE::ParseCommandLineFlags(&argc, &argv, true);

  CHECK(FLAGS_input != "") << "Need to specify the filename to read.";

  std::map<int, ceres::examples::Pose2d> poses;
  std::vector<ceres::examples::Constraint2d> constraints;

  CHECK(ceres::examples::ReadG2oFile(FLAGS_input, &poses, &constraints))
      << "Error reading the file: " << FLAGS_input;

  std::cout << "Number of poses: " << poses.size() << '\n';
  std::cout << "Number of constraints: " << constraints.size() << '\n';

  CHECK(ceres::examples::OutputPoses("poses_original.txt", poses))
      << "Error outputting to poses_original.txt";

  ceres::Problem problem;
  ceres::examples::BuildOptimizationProblem(constraints, &poses, &problem);

  CHECK(ceres::examples::SolveOptimizationProblem(&problem))
      << "The solve was not successful, exiting.";

  CHECK(ceres::examples::OutputPoses("poses_optimized.txt", poses))
      << "Error outputting to poses_original.txt";

  return 0;
}
