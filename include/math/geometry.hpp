#pragma once

#include <Eigen/Dense>
#include <cmath>

namespace math {

  Eigen::Matrix4f rot_x(float angle) {
    Eigen::Matrix4f r;
    float s = std::sin(angle);
    float c = std::cos(angle);

    r << 1, 0,  0, 0,
         0, c, -s, 0,
         0, s,  c, 0,
         0, 0,  0, 1;
    return r;
  }

  Eigen::Matrix4f rot_y(float angle) {
    Eigen::Matrix4f r;
    float s = std::sin(angle);
    float c = std::cos(angle);

    r <<  c, 0, s, 0,
          0, 1, 0, 0,
         -s, 0, c, 0,
          0, 0, 0, 1;
    return r;
  }

  Eigen::Matrix4f rot_z(float angle) {
    Eigen::Matrix4f r;
    float s = std::sin(angle);
    float c = std::cos(angle);

    r << c, -s, 0, 0,
         s,  c, 0, 0,
         0,  0, 1, 0,
         0,  0, 0, 1;
    return r;
  }

  Eigen::Matrix4f translate(const Eigen::Vector3f& t) {
    Eigen::Matrix4f r;
    r << 1, 0, 0, t[0],
         0, 1, 0, t[1],
         0, 0, 1, t[2],
         0, 0, 0, 1;
    return r;
  }

}