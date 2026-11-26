// Copyright 2026 Sasaki
// All rights reserved.
//
// Software License Agreement (BSD 2-Clause Simplified License)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

// pose_graph_uncertainty_report: per-vertex marginal pose uncertainty of a
// saved pose graph (docs/roadmap/active-slam-2026-11.md, P1). Input is a
// pose_graph.g2o file or a map bundle directory that holds one; output is
// pose_graph_uncertainty.csv (one row per vertex) and
// pose_graph_uncertainty_summary.yaml. No ROS, no wall clock, no randomness:
// the same input bytes produce the same output bytes.

#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

#include "graph_based_slam/pose_graph_uncertainty.hpp"

namespace
{

void printUsage()
{
  std::cout <<
    "usage: pose_graph_uncertainty_report --input <pose_graph.g2o | map_bundle_dir>\n"
    "                                     --output-dir <dir>\n"
    "  Writes pose_graph_uncertainty.csv and pose_graph_uncertainty_summary.yaml.\n"
    "  Sigmas are in the units the backend's edge information weights imply: they rank\n"
    "  vertices and compare graphs built with the same weights; they are not metres of error.\n";
}

bool writeFile(const std::filesystem::path & path, const std::string & bytes)
{
  std::ofstream out(path, std::ios::binary);
  out << bytes;
  return static_cast<bool>(out);
}

}  // namespace

int main(int argc, char ** argv)
{
  std::string input;
  std::string output_dir;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    const bool has_value = i + 1 < argc;
    if (arg == "--input" && has_value) {
      input = argv[++i];
    } else if (arg == "--output-dir" && has_value) {
      output_dir = argv[++i];
    } else {
      printUsage();
      return arg == "--help" ? 0 : 1;
    }
  }
  if (input.empty() || output_dir.empty()) {
    printUsage();
    return 1;
  }

  std::filesystem::path graph_path(input);
  if (std::filesystem::is_directory(graph_path)) {
    graph_path /= "pose_graph.g2o";
  }
  std::ifstream graph_file(graph_path);
  if (!graph_file) {
    std::cerr << "error: cannot read " << graph_path.string() << std::endl;
    return 1;
  }

  namespace pgu = graphslam::pose_graph_uncertainty;
  pgu::ParsedGraph graph;
  std::string error;
  if (!pgu::parseG2o(graph_file, graph, error)) {
    std::cerr << "error: " << graph_path.string() << ": " << error << std::endl;
    return 1;
  }
  const pgu::UncertaintyReport report = pgu::computeUncertainty(graph);

  std::error_code ec;
  std::filesystem::create_directories(output_dir, ec);
  const std::filesystem::path out_dir(output_dir);
  const std::string summary = pgu::formatSummaryYaml(graph, report);
  if (!writeFile(out_dir / "pose_graph_uncertainty_summary.yaml", summary)) {
    std::cerr << "error: cannot write to " << output_dir << std::endl;
    return 1;
  }
  std::cout << summary;
  if (!report.ok) {
    std::cerr << "error: " << report.error << std::endl;
    return 2;
  }
  if (!writeFile(out_dir / "pose_graph_uncertainty.csv", pgu::formatCsv(report))) {
    std::cerr << "error: cannot write to " << output_dir << std::endl;
    return 1;
  }
  return 0;
}
