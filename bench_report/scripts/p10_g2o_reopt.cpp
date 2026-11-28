// reopt GRAPH.g2o INIT.tum ITER OUT.tum [loop_trans_info_scale]
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/core/factory.h>
#include <fstream>
#include <iostream>
#include <iomanip>
G2O_USE_TYPE_GROUP(slam3d);
int main(int argc, char** argv) {
  g2o::SparseOptimizer opt;
  auto ls = std::make_unique<g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>>();
  opt.setAlgorithm(new g2o::OptimizationAlgorithmLevenberg(std::make_unique<g2o::BlockSolver_6_3>(std::move(ls))));
  std::ifstream in(argv[1]); if (!opt.load(in)) { std::cerr << "load failed\n"; return 1; }
  std::ifstream ti(argv[2]); std::vector<double> stamps; double t,x,y,z,qx,qy,qz,qw; int id = 0;
  while (ti >> t >> x >> y >> z >> qx >> qy >> qz >> qw) {
    auto* v = dynamic_cast<g2o::VertexSE3*>(opt.vertex(id));
    if (v) { Eigen::Isometry3d T = Eigen::Isometry3d::Identity(); T.linear() = Eigen::Quaterniond(qw,qx,qy,qz).toRotationMatrix(); T.translation() << x,y,z; v->setEstimate(T); }
    stamps.push_back(t); ++id;
  }
  double scale = argc > 5 ? atof(argv[5]) : 1.0; double arot = argc > 6 ? atof(argv[6]) : 1.0; double atr = argc > 7 ? atof(argv[7]) : 1.0;
  int nloop = 0;
  for (auto* e : opt.edges()) { auto* se = dynamic_cast<g2o::EdgeSE3*>(e);
    int a = se->vertices()[0]->id(), b = se->vertices()[1]->id();
    if (b - a > 20) { ++nloop; auto I = se->information(); I.topLeftCorner<3,3>() *= scale; se->setInformation(I); } else { auto I = se->information(); I.topLeftCorner<3,3>() *= atr; I.bottomRightCorner<3,3>() *= arot; se->setInformation(I); } }
  opt.vertex(0)->setFixed(true);
  opt.initializeOptimization(); opt.computeActiveErrors(); double c0 = opt.activeChi2();
  int it = atoi(argv[3]); opt.optimize(it); double c1 = opt.activeChi2();
  std::cerr << "loops " << nloop << " chi2 " << c0 << " -> " << c1 << "\n";
  std::ofstream out(argv[4]); out << std::fixed << std::setprecision(9);
  for (size_t i = 0; i < stamps.size(); ++i) { auto* v = dynamic_cast<g2o::VertexSE3*>(opt.vertex(i)); auto T = v->estimate(); Eigen::Quaterniond q(T.linear());
    out << stamps[i] << " " << T.translation().x() << " " << T.translation().y() << " " << T.translation().z() << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n"; }
}
