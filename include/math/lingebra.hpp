#pragma once

#include <Eigen/Dense>

namespace math {

  template <typename T, int N>
  Eigen::Matrix<T, N+1, 1> embed_vector(const Eigen::Matrix<T, N, 1>& v, T w) {
    Eigen::Matrix<T, N+1, 1> result;
    result << v, w;
    return result;
  }

  template <typename T, int N>
  std::vector<Eigen::Matrix<T, N+1, 1>> embed_vectors(const std::vector<Eigen::Matrix<T, N, 1>>& vectors, T w) {
    std::vector<Eigen::Matrix<T, N+1, 1>> result;
    result.reserve(vectors.size());

    for (const auto& v : vectors) {
      result.push_back(embed_vector(v, w));
    }

    return result;
  }

  template <typename T, int N>
  Eigen::Matrix<T, N-1, 1> project_vector(const Eigen::Matrix<T, N, 1>& v) {
    Eigen::Matrix<T, N-1, 1> result;

    if (v(N-1) == 0) {
      # pragma unroll(N-1)
      for (int i = 0; i < N-1; i++) {
        result(i) = v(i);
      }
    } else {
      # pragma unroll(N-1)
      for (int i = 0; i < N-1; i++) {
        result(i) = v(i) / v(N-1);
      }
    }

    return result;
  }

  template <typename T, int N>
  std::vector<Eigen::Matrix<T, N-1, 1>> project_vectors(const std::vector<Eigen::Matrix<T, N, 1>>& vectors) {
    std::vector<Eigen::Matrix<T, N-1, 1>> result;
    result.reserve(vectors.size());

    for (const auto& v : vectors) {
      result.push_back(project_vector(v));
    }

    return result;
  }

  template <typename T, int N>
  std::vector<Eigen::Matrix<T, N, 1>> broadcast_multiply(const Eigen::Matrix<T, N, N> &mat, const std::vector<Eigen::Matrix<T, N, 1>> &vectors) {
    std::vector<Eigen::Matrix<T, N, 1>> result;
    result.reserve(vectors.size());

    for (const auto& v : vectors) {
      result.push_back(mat * v);
    }

    return result;
  }

}