#include <Eigen/Dense>

#include "render.hpp"
#include "mesh_data.hpp"
#include "camera.hpp"
#include "colourT.hpp"
#include "math/lingebra.hpp"

namespace render {
  float clamp(float value, float min, float max) {
    return std::max(min, std::min(max, value));
  }


  // TODO: merge with maximal_coordinate
  float minimal_coordinate(std::vector<Vector2f> vertices, int coordinate_index) {
    float r = vertices[0][coordinate_index];

    for (auto vertex: vertices) {
      r = std::min(r, vertex[coordinate_index]);
    }

    return r;
  }

  float maximal_coordinate(std::vector<Vector2f> vertices, int coordinate_index) {
    float r = vertices[0][coordinate_index];

    for (auto vertex: vertices) {
      r = std::max(r, vertex[coordinate_index]);
    }

    return r;
  }


  std::vector<Vector3f> perspective_projection(const std::vector<Eigen::Vector4f> &vertices, float focal_length) {
    std::vector<Vector3f> r = math::project_vectors(vertices); // get rid of the last coordinate
    for (auto& v: r) {
      v(2) = v(2) / -focal_length;
    }

    return r;
  }

  bool abc_matrix_inverse(const std::vector<Vector2f> &vertices, Eigen::Matrix3f &abc_inverse) {
    // TODO: check if correct and implement as in the notes
    Eigen::Matrix3f abc_mat {
      {vertices[0][0], vertices[1][0], vertices[2][0]},
      {vertices[0][1], vertices[1][1], vertices[2][1]},
      {1, 1, 1},
    };

    bool invertible;
    float determinant;
    abc_mat.computeInverseAndDetWithCheck(abc_inverse, determinant, invertible);

    if (!invertible) {
      std::cout << "barycentric_coordinates: matrix is not invertible" << std::endl;
      return false;
    }

    return true;
  }

  struct FragmentCache {
    bool valid {true};
    std::vector<Eigen::Vector2f> projected_vertices;
    float projected_vertices0020;
    float projected_vertices1020;
    float projected_vertices0121;
    float projected_vertices1121;
    float projected_vertices20;
    float projected_vertices21;

    // matrix that transforms pixel coordinates to barycentric coordinates
    // Eigen::Matrix3f abc_inverse;

    FragmentCache(const std::vector<Eigen::Vector2f> &vertices): projected_vertices{vertices} {
      // if (!abc_matrix_inverse(projected_vertices, abc_inverse)) {
      //   std::cout << "FragmentCache: matrix is not invertible" << std::endl;
      //   valid = false;
      // }

      projected_vertices0020 = projected_vertices[0][0] - projected_vertices[2][0];
      projected_vertices1020 = projected_vertices[1][0] - projected_vertices[2][0];
      projected_vertices0121 = projected_vertices[0][1] - projected_vertices[2][1];
      projected_vertices1121 = projected_vertices[1][1] - projected_vertices[2][1];
      projected_vertices20 = projected_vertices[2][0];
      projected_vertices21 = projected_vertices[2][1];
    }
  };

  bool barycentric_coordinates_slow(const FragmentCache &cache, const Vector2f &P, Vector3f &res) {

    // Vector3f p_resized = {P[0], P[1], 1};
    // res = cache.abc_inverse * p_resized;

    res = {1, 1, 1};

    return true;
  }

  bool barycentric_coordinates(const FragmentCache &cache, const Vector2f &P, Vector3f &res, float tolerance = 1e-9) {

    Eigen::Vector3f first_vector {
      cache.projected_vertices0020,
      cache.projected_vertices1020,
      cache.projected_vertices20 - P[0]
    };

    Eigen::Vector3f second_vector {
      cache.projected_vertices0121,
      cache.projected_vertices1121,
      cache.projected_vertices21 - P[1]
    };

    Eigen::Vector3f cross = first_vector.cross(second_vector);

    if (std::abs(cross[2]) < tolerance) {
      return false;
    }

    float cross02 = cross[0] / cross[2];
    float cross12 = cross[1] / cross[2];

    // res = {cross[0]/cross[2], cross[1]/cross[2], 1 - cross[0]/cross[2] - cross[1]/cross[2]};
    res = {cross02, cross12, 1 - cross02 - cross12};

    return true;
  }

  bool polygon_pointing_camera(PolygonData &polygon_data, Camera &camera) {
    Eigen::Vector4f view4 = camera.get_c2w() * Eigen::Vector4f{0, 0, -1, 0};
    Eigen::Vector3f view = math::project_vector(view4);

    for (auto normal: polygon_data.normals) {
      if (view.dot(normal) < 0) {
        return true;
      }
    }

    return false;
  }

  void render_triangle(PolygonData &polygon_data, Camera &camera, cv::Mat &img, cv::Mat &z_buffer) {
    // TODO: split into multiple functions
    // TODO: we have to do frustum culling. trivial edge case: what if part of the triangle is behind the camera?
    // even worse, what if the point has z-coordinate 0?

    if (!polygon_pointing_camera(polygon_data, camera)) {
      return;
    }

    std::vector<Eigen::Vector4f> vertices = math::embed_vectors(polygon_data.vertices, 1.0f);
    std::vector<Eigen::Vector4f> camera_vertices = math::broadcast_multiply(camera.get_w2c(), vertices);
    std::vector<Eigen::Vector3f> perspective_vertices = perspective_projection(camera_vertices, camera.get_focal_length());
    std::vector<Eigen::Vector2f> projected_vertices = math::project_vectors(perspective_vertices);

    FragmentCache cache{projected_vertices};

    if (!cache.valid) {
      return;
    }

    const double min_x = clamp(minimal_coordinate(projected_vertices, 0), camera.min_x(), camera.max_x());
    const double max_x = clamp(maximal_coordinate(projected_vertices, 0), camera.min_x(), camera.max_x());
    const double min_y = clamp(minimal_coordinate(projected_vertices, 1), camera.min_y(), camera.max_y());
    const double max_y = clamp(maximal_coordinate(projected_vertices, 1), camera.min_y(), camera.max_y());

    Eigen::Vector2i min_pixel = camera.image_to_pixel({min_x, min_y});
    Eigen::Vector2i max_pixel = camera.image_to_pixel({max_x, max_y});

    // TODO min_i is in camera space, but we need to iterate over pixels

    for (int x = min_pixel[0]; x <= max_pixel[0]; ++x) {
      for (int y = min_pixel[1]; y <= max_pixel[1]; ++y) {
        Vector3f barycentric;

        // do this in the pixel space to avoid pixel_to_image call
        if (!barycentric_coordinates(cache, camera.pixel_to_image({x, y}), barycentric)) {
          continue;
        }

        // check if the point is inside the triangle
        if (barycentric[0] < 1e-5 || barycentric[1] < 1e-5 || barycentric[2] < 1e-5) {
          continue;
        }

        // z-value is the weighted sum of the z-values of camera_vertices
        // TODO: optimise by not throwing away the z-value
        double z_value = 0;
        for (int i = 0; i < 3; i++) {
          z_value += barycentric[i] * camera_vertices[i][2];
        }

        z_value = -z_value;

        // check if the point is in front of the camera
        if (z_value < camera.get_focal_length()) {
          continue;
        }

        int flipped_y = camera.get_image_height() - y - 1;

        // check if the point is closer than the one already in the z-buffer
        if (z_value > z_buffer.at<double>(flipped_y, x)) {
          continue;
        }

        colourT colour = weighted_average(polygon_data.diffuse_colours, {barycentric[0], barycentric[1], barycentric[2]});
        img.at<cv::Vec3b>(flipped_y, x) = colour.cv_colour();
        z_buffer.at<double>(flipped_y, x) = z_value;
      }
    }
  }
}

cv::Mat render_mesh(Camera &camera, Mesh &mesh_data) {
  cv::Mat image{camera.get_image_height(), camera.get_image_width(), CV_8UC3, cv::Scalar(0, 0, 0)};
  cv::Mat z_buffer{camera.get_image_height(), camera.get_image_width(), CV_64F, cv::Scalar(1e10)};


  // #pragma omp parallel for
  for (int i = 0; i < mesh_data.n_triangles(); ++i) {
    PolygonData polygon_data = mesh_data.polygon_data(i);

    render::render_triangle(polygon_data, camera, image, z_buffer);
  }




  return image;
}