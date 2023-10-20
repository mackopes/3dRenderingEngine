#pragma once

#include <Eigen/Dense>
#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>

#include "colourT.hpp"

using Eigen::Vector3f;
using Eigen::Vector2f;

struct PolygonData{
  std::vector<Vector3f> vertices;
  std::vector<colourT> diffuse_colours;
  std::vector<Vector3f> normals;
};

struct FaceVectorData {
  int vertex_index;
  int vertex_texture_index;
  int vertex_normal_index;
};

class Mesh {
  private:
    std::vector<Vector3f> vertices;
    std::vector<Vector2f> vertices_texture;
    std::vector<Vector3f> vertices_normal;
    std::vector<std::vector<FaceVectorData>> faces;

    cv::Mat texture;

    bool parse_vertex(std::stringstream& line_stream);
    bool parse_face(std::stringstream& line_stream);
    bool parse_vertex_texture(std::stringstream& line_stream);
    bool parse_vertex_normal(std::stringstream& line_stream);

  public:

    std::vector<Vector3f> triangle(int i);
    bool parse_line(const std::string& line);
    PolygonData polygon_data(int i);
    bool attach_texture(std::string texture_path);

    int n_triangles() { return faces.size(); }
    void print() {
      for (auto &v: vertices) {
        std::cout << v << std::endl;
      }

      for (auto &v: faces) {
        std::cout << "f: ";
        for (auto f: v) {
          std::cout << f.vertex_index << ' ';
        }
        std::cout << std::endl;
      }
    }
};
